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
#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>
#ifndef K2
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#else
#include <ti/csl/csl_serdes_restore_default.h>
#include <ti/csl/csl_serdes_aif2.h>
#endif
#include <ti/csl/soc.h>
#include <ti/csl/csl_cgem.h>

#include <ti/drv/aif2/AIF_defs.h>
#include <ti/drv/aif2/AIF_init_dat.h>
#include <ti/drv/aif2/AIF_fsync.h>
#include <ti/drv/aif2/AIF_calcParam.h>
#include <ti/drv/aif2/aif2_osal.h>
#include <ti/drv/aif2/aif2ver.h>
#include <ti/drv/aif2/AIF_cfg.h>

#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>

#define __AIF_INIT_C
#include <ti/drv/aif2/AIF_init.h>

// Bit count over 32 bits
#ifdef _TMS320C6X
#define _bitc32(a) _dotpu4(_bitc4(a),0x01010101)
#else
static uint32_t _bitc32(uint32_t i)
{
    i = i - ((i >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}
#endif

#ifdef _TMS320C6X
#pragma CODE_SECTION(AIF_initHw, ".text:aifDriver");
#pragma CODE_SECTION(AIF_startHw, ".text:aifDriver");
#pragma CODE_SECTION(AIF_initDio, ".text:aifDriver");
#pragma CODE_SECTION(AIF_initPktDma, ".text:aifDriver");
#pragma CODE_SECTION(AIF_getEgressGroupId, ".text:aifDriver");
#ifdef K2
#pragma CODE_SECTION(AIF_serdesConfig, ".text:aifDriver");
#pragma CODE_SECTION(AIF_serdesRestoreDefault, ".text:aifDriver");
#endif
#pragma CODE_SECTION(AIF_configureAtEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_enableAtEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_disableAtEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_configureEgrDioEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_enableEgrDioEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_disableEgrDioEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_configureIngrDioEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_enableIngrDioEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_disableIngrDioEvent, ".text:aifDriver");
#pragma CODE_SECTION(AIF_configureEgrDioEngine, ".text:aifDriver");
#pragma CODE_SECTION(AIF_configureIngrDioEngine, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setRadTimerTc, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setPhyTimerInit, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setUlRadTimerInit, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setDlRadTimerInit, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setRmLinkSetupParams, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setLinkPiMax, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setPeFrameTC, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setPeFrameMsgTc, ".text:aifDriver");
#pragma CODE_SECTION(AIF_setPdChDioOffset, ".text:aifDriver");
#pragma CODE_SECTION(AIF2_getVersion, ".text:aifDriver");
#pragma CODE_SECTION(AIF2_getVersionStr, ".text:aifDriver");

#pragma CODE_SECTION(initEgrGroupInfo, ".text:aifDriver");
#pragma CODE_SECTION(initIngrGroupInfo, ".text:aifDriver");
#pragma CODE_SECTION(addAxCtoEgrGroupTable, ".text:aifDriver");
#pragma CODE_SECTION(addAxCtoIngrGroupTable, ".text:aifDriver");
#pragma CODE_SECTION(getEgrGroupId, ".text:aifDriver");

#pragma DATA_SECTION(aif2Setup,".far:aifDriver");
#pragma DATA_SECTION(aif2Param,".far:aifDriver");
#pragma DATA_SECTION(linkSetup,".far:aifDriver");
#pragma DATA_SECTION(globalSetup,".far:aifDriver");
#pragma DATA_SECTION(commonSetup,".far:aifDriver");
#pragma DATA_SECTION(SdCommonSetup,".far:aifDriver");
#pragma DATA_SECTION(PdCommonSetup,".far:aifDriver");
#pragma DATA_SECTION(PeCommonSetup,".far:aifDriver");
#pragma DATA_SECTION(IngrDbSetup,".far:aifDriver");
#pragma DATA_SECTION(EgrDbSetup,".far:aifDriver");
#pragma DATA_SECTION(AdCommonSetup,".far:aifDriver");
#pragma DATA_SECTION(AdDioSetup,".far:aifDriver");
#pragma DATA_SECTION(AtCommonSetup,".far:aifDriver");
#pragma DATA_SECTION(AtEventSetup,".far:aifDriver");
#pragma DATA_SECTION(PhyTimerTc,".far:aifDriver");
#pragma DATA_SECTION(RadTimerTc,".far:aifDriver");
#pragma DATA_SECTION(PhyTimerInit,".far:aifDriver");
#pragma DATA_SECTION(RadTimerInit,".far:aifDriver");
#pragma DATA_SECTION(UlRadTimerInit,".far:aifDriver");
#pragma DATA_SECTION(DlRadTimerInit,".far:aifDriver");
#pragma DATA_SECTION(ComLinkSetup,".far:aifDriver");
#pragma DATA_SECTION(SdLinkSetup,".far:aifDriver");
#pragma DATA_SECTION(RmLinkSetup,".far:aifDriver");
#pragma DATA_SECTION(TmLinkSetup,".far:aifDriver");
#pragma DATA_SECTION(PdLinkSetup,".far:aifDriver");
#pragma DATA_SECTION(PeLinkSetup,".far:aifDriver");
#pragma DATA_SECTION(RtLinkSetup,".far:aifDriver");
#pragma DATA_SECTION(AtLinkSetup,".far:aifDriver");
#pragma DATA_SECTION(Aif2Obj,".far:aifDriver");
#pragma DATA_SECTION(Aif2Context,".far:aifDriver");
#pragma DATA_SECTION(superPacket,".far:aifDriver");
#pragma DATA_SECTION(egrGroupInfo,".far:aifDriver");
#pragma DATA_SECTION(aif2LldVersionStr,".far:aifDriver");
#endif

#ifdef _TMS320C6X
#ifdef _LITTLE_ENDIAN
#define DEVICE_LE
#endif
#else
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define DEVICE_LE
#endif
#endif


static Aif2Fl_Setup               aif2Setup;//Aif2 HW setup
static Aif2Fl_Param  			   aif2Param;//AIF2 module specific parameters
static Aif2Fl_LinkSetup           linkSetup[AIF_MAX_NUM_LINKS];// Setup for links
static Aif2Fl_GlobalSetup         globalSetup;// global config for AIF2
static Aif2Fl_CommonSetup         commonSetup; // Setup for common params

static Aif2Fl_SdCommonSetup       SdCommonSetup;//SERDES common setup
static Aif2Fl_PdCommonSetup       PdCommonSetup;//PD common setup
static Aif2Fl_PeCommonSetup       PeCommonSetup;//PE common setup
static Aif2Fl_IngrDbSetup         IngrDbSetup;// Ingress data buffer setup
static Aif2Fl_EgrDbSetup          EgrDbSetup;// Egress data buffer setup
static Aif2Fl_AdCommonSetup       AdCommonSetup;// Aif2 DMA common setup
static Aif2Fl_AdDioSetup          AdDioSetup;// Aif2 DIO common setup
static Aif2Fl_AtCommonSetup       AtCommonSetup; // Aif2 Timer common  setup
static Aif2Fl_AtEventSetup        AtEventSetup; // Aif2 Timer external and internal event  setup
static Aif2Fl_AtCountObj          PhyTimerTc;// AT Phy Terminal Count setup
static Aif2Fl_AtCountObj          RadTimerTc;// AT Rad Terminal Count setup
static Aif2Fl_AtCountObj          PhyTimerInit;// AT Phy Init value setup
static Aif2Fl_AtCountObj          RadTimerInit;// AT Rad Init value setup
static Aif2Fl_AtCountObj          UlRadTimerInit;// AT Rad Init value setup
static Aif2Fl_AtCountObj          DlRadTimerInit;// AT Rad Init value setup

static Aif2Fl_CommonLinkSetup     ComLinkSetup[AIF_MAX_NUM_LINKS]; // Aif2 link common setup
static Aif2Fl_SdLinkSetup         SdLinkSetup[AIF_MAX_NUM_LINKS]; //SERDES link setup
static Aif2Fl_RmLinkSetup         RmLinkSetup[AIF_MAX_NUM_LINKS]; //RM link setup
static Aif2Fl_TmLinkSetup         TmLinkSetup[AIF_MAX_NUM_LINKS]; //TM link setup
static Aif2Fl_PdLinkSetup         PdLinkSetup[AIF_MAX_NUM_LINKS]; //PD link setup
static Aif2Fl_PeLinkSetup         PeLinkSetup[AIF_MAX_NUM_LINKS]; //PE link setup
static Aif2Fl_RtLinkSetup         RtLinkSetup[AIF_MAX_NUM_LINKS]; //RT link setup
static Aif2Fl_AtLinkSetup         AtLinkSetup[AIF_MAX_NUM_LINKS]; // Aif2 timer link setup (Pi, Delta, PE signal)

static Aif2Fl_Obj                 Aif2Obj;
static Aif2Fl_Context             Aif2Context;

static uint16_t				 		   superPacket;

#ifndef _TMS320C6X
extern AIF_InitCfg                *pGlobalAif2Cfg;
#endif

#ifdef K2
#define QMSS_AIF_QUEUE_BASE                       512
#endif

//Initialize AIF2 DIO buffers
void 
AIF_initDio(
		AIF_ConfigHandle    hAif
)
{
	uint32_t i,j, start_channel_pe, start_channel_pd, num_channel_pe, num_channel_pd;
	int DIO_nb_link[3]={0,0,0};
	int DIO_first_link[3]={-1,-1,-1};

	// Compute first link and num link per DIO engine
	for(i=0; i< AIF_MAX_NUM_LINKS; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
		{
			DIO_nb_link[hAif->linkConfig[i].dioEngine]+=1;
			if (DIO_first_link[hAif->linkConfig[i].dioEngine] == (-1))
			{
				DIO_first_link[hAif->linkConfig[i].dioEngine] = i;
				hAif->dioConfig[hAif->linkConfig[i].dioEngine].mode = hAif->linkConfig[i].mode;
			}
		}
	}

	for(i=0; i<AIF2_MAX_NUM_DIO_ENGINE; i++)
	{
		hAif->dioConfig[i].numLink            = DIO_nb_link[i];
		hAif->dioConfig[i].firstLink          = DIO_first_link[i];
	}

	// Compute first DB channel offset and last DB channel offset for each DIO engine
	start_channel_pe = 0;
	start_channel_pd = 0;
	for(i=0; i<AIF2_MAX_NUM_DIO_ENGINE; i++)
	{
		num_channel_pe = 0;
		num_channel_pd = 0;
		for(j=hAif->dioConfig[i].firstLink;j<(hAif->dioConfig[i].firstLink + hAif->dioConfig[i].numLink);j++)
		{
			if(1==hAif->linkConfig[j].linkEnable)
		    {
				num_channel_pe+=hAif->linkConfig[j].numPeAxC;
				num_channel_pd+=hAif->linkConfig[j].numPdAxC;
		    }

		}
		hAif->dioConfig[i].offsetPeDBCH         = start_channel_pe;
		start_channel_pe                        += num_channel_pe;
		hAif->dioConfig[i].numPeDBCH            = num_channel_pe;

		hAif->dioConfig[i].offsetPdDBCH         = start_channel_pd;
		start_channel_pd                        += num_channel_pd;
		hAif->dioConfig[i].numPdDBCH            = num_channel_pd;
	}

	/* Initialize DIO engine counters for each of the links */
	memset((char*)aif2DioIntCount,(char)0x00, sizeof(aif2DioIntCount));

}

int32_t
AIF_initPktDma(
		AIF_ConfigHandle    hAif
)
{
#ifdef _TMS320C6X
	Cppi_CpDmaInitCfg    aif2CPDMACfg;
	Cppi_DescCfg         descCfg;
	Cppi_MonolithicDesc *ptrMonoDesc;
	Cppi_HostDesc       *ptrHostDesc;
	Cppi_DescTag         descTag;
    Cppi_RxChInitCfg     rxCfg;
    Cppi_TxChInitCfg     txCfg;
	Qmss_Queue           queueInfo;
	uint32_t				 axcConfigTx, axcConfigRx;

	uint32_t               psMsgEnable = 0;
	uint8_t                isAllocated;
	uint32_t				 i,j,k, numAxC, numAllocated;
	uint32_t				 *dataPtr;

    memset(&descTag, 0x00, sizeof(Cppi_DescTag));
    memset(&descCfg, 0x00, sizeof(Cppi_DescCfg));

   /* Initialize Pkt dma if necessary */
    for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
	   if (hAif->linkConfig[i].psMsgEnable){
		   	   psMsgEnable = 1;
	   }
    }

    for(i=0; i< AIF_MAX_NUM_LINKS; i++)
    {
    	if (hAif->linkConfig[i].linkEnable == 1)
    	{
    	    if (hAif->superPacket == true)
    	    {
    	    	superPacket = true;
    	    	hAif->linkConfig[i].numPdAxC=1;
			} else {
				superPacket = false;
			}
    	}
    }



	/* Initialize the AIF2 CPDMA config structure. */
	memset ((void *)&aif2CPDMACfg, 0, sizeof(Cppi_CpDmaInitCfg));

	/* Setup the AIF2 CPDMA Configuration. */
	aif2CPDMACfg.dmaNum = Cppi_CpDma_AIF_CPDMA;
	
	if (hAif->pktdmaOrDioEngine == AIF2FL_DIO)
	{
		aif2CPDMACfg.writeFifoDepth = 8;
	}

	/* Open the AIF2 CPDMA. Needed even if DIO used, as we need to disable PKTDMA loopback mode */
	hAif->pktDmaConfig.hCppi = Cppi_open (&aif2CPDMACfg);

	/* Disable the AIF2-CPDMA Loopback mode in the CPPI */
	Cppi_setCpdmaLoopback (hAif->pktDmaConfig.hCppi, 0);

    /* If PktDma mode and/or ctrl messages enabled, open and populate free queues, open tx/rx queues, configure flows  */
    if (psMsgEnable == 1)
    {
		/***********************************************************************
		 ********************** Transmit Configuration *************************
		 ***********************************************************************/
    	for(i=0; i< AIF2_CPRI_MAX_CW_SUBSTREAM; i++)
    	{
	        if (hAif->pktDmaConfig.hRxFlowCtrl[i] != NULL)
	        {

	        	/***********************************************************************
	        	 ********************** Transmit Queues Configuration ******************
	        	 ***********************************************************************/

				/* Initialize the Transmit descriptors. We want all the transmit descriptors to go
				 * back to the Transmit Free Queue after transmission. */
				descCfg.memRegion                 = hAif->pktDmaConfig.txRegionCtrl[i];
				descCfg.descNum                   = hAif->pktDmaConfig.txNumDescCtrl[i];
				descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
				descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
				descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
				descCfg.descType                  = Cppi_DescType_MONOLITHIC;
				descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
				descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
				descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
				descCfg.returnPushPolicy          = Qmss_Location_TAIL;
				descCfg.cfg.mono.dataOffset		  = 12; // size of header for control words

				/* Initialize the descriptors and place all of them into the general purpose queue */
				hAif->pktDmaConfig.txFqCtrl[i]  = Cppi_initDescriptor (&descCfg, &numAllocated);
				if (hAif->pktDmaConfig.txFqCtrl[i] < 0)
				{
					Aif2_osalLog("Error: AIF2 Transmit Completion Queue failed to open\n");
					return -1;
				}
				/* Get the Queue Information for the Transmit Free Queue */
				queueInfo = Qmss_getQueueNumber(hAif->pktDmaConfig.txFqCtrl[i]);

				/* Pop off all descriptors from the Queue and add the missing init parameters. */
				for (j = 0; j < hAif->pktDmaConfig.txNumDescCtrl[i]; j++)
				{
					/* Get a mono descriptor from the free GP queue. */
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hAif->pktDmaConfig.txFqCtrl[i]));

					Cppi_setPacketLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,hAif->pktDmaConfig.txDescSizeCtrl[i]);
					descTag.srcTagLo = 124+i;
					Cppi_setTag(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,&descTag);
					Cppi_setPSFlags(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
					Cppi_setPSLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
					Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);

					/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
					Qmss_queuePushDesc(hAif->pktDmaConfig.txFqCtrl[i], (uint32_t*)ptrMonoDesc);
				}

				/* Open the AIF2 transmit queue for this control stream */
				hAif->pktDmaConfig.txQCtrl[i] = Qmss_queueOpen (Qmss_QueueType_AIF_QUEUE , QMSS_AIF_QUEUE_BASE+124+i, &isAllocated);
				if (hAif->pktDmaConfig.txQCtrl[i] < 0)
				{
					Aif2_osalLog("Error: AIF2 Transmit Queue failed to open\n");
					return -1;
				}

				/***********************************************************************
	        	 ********************** Receive Queues Configuration *******************
	        	 ***********************************************************************/

				/* Initialize the Receive descriptors */
				descCfg.memRegion                 = hAif->pktDmaConfig.rxRegionCtrl[i];
				descCfg.descNum                   = hAif->pktDmaConfig.rxNumDescCtrl[i];
				descCfg.destQueueNum              = hAif->pktDmaConfig.hRxFlowCtrl[i]->rx_fdq0_sz0_qnum;
				descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
				descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
				descCfg.descType                  = (Cppi_DescType)hAif->pktDmaConfig.hRxFlowCtrl[i]->rx_desc_type;
				descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
				descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
				descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
				descCfg.returnPushPolicy          = Qmss_Location_TAIL;
				descCfg.cfg.mono.dataOffset		  = hAif->pktDmaConfig.hRxFlowCtrl[i]->rx_sop_offset; // size of header for control words

				/* Initialize the descriptors and place all of them into the general purpose queue */
				hAif->pktDmaConfig.rxFqCtrl[i]  = Cppi_initDescriptor (&descCfg, &numAllocated);
				if (hAif->pktDmaConfig.rxFqCtrl[i] < 0)
				{
					Aif2_osalLog("Error: AIF2 Receive Completion Queue failed to open\n");
					return -1;
				}

				/* Get the Queue Information for the Transmit Free Queue */
				queueInfo = Qmss_getQueueNumber(hAif->pktDmaConfig.rxFqCtrl[i]);

				/* Pop off all descriptors from the Queue and add the missing init parameters. */
				for (j = 0; j < hAif->pktDmaConfig.rxNumDescCtrl[i]; j++)
				{
					/* Get a mono descriptor from the free GP queue. */
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hAif->pktDmaConfig.rxFqCtrl[i]));

					Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);

					/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
					Qmss_queuePushDesc(hAif->pktDmaConfig.rxFqCtrl[i], (uint32_t*)ptrMonoDesc);
				}

				/* Open the AIF2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
				hAif->pktDmaConfig.rxQCtrl[i] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hAif->pktDmaConfig.hRxFlowCtrl[i]->rx_dest_qnum, &isAllocated);
				if (hAif->pktDmaConfig.rxQCtrl[i] < 0)
				{
					Aif2_osalLog("Error: AIF2 Receive Queue failed to open\n");
					return -1;
				}

				/***********************************************************************
	        	 ********************** PktDma channel Configuration *******************
	        	 ***********************************************************************/

				/* Open the AIF2 Receive Channel and keep it disabled */
				rxCfg.channelNum = 124+i;
				rxCfg.rxEnable   = Cppi_ChState_CHANNEL_ENABLE;
				hAif->pktDmaConfig.rxChCtrl[i] = Cppi_rxChannelOpen(hAif->pktDmaConfig.hCppi, &rxCfg, &isAllocated);
				if (hAif->pktDmaConfig.rxChCtrl[i] == NULL)
				{
					Aif2_osalLog("Error: Opening AIF2 Rx channel %d failed\n", rxCfg.channelNum);
					return -1;
				}
				/* Open the AIF2 Transmit Channel and keep it disabled. */
				txCfg.channelNum   = 124+i;
				txCfg.priority     = 0;
				txCfg.txEnable     = Cppi_ChState_CHANNEL_ENABLE;
				txCfg.filterEPIB   = 0;
				txCfg.filterPS     = 0;
				txCfg.aifMonoMode  = 0;
				hAif->pktDmaConfig.txChCtrl[i] = Cppi_txChannelOpen(hAif->pktDmaConfig.hCppi, &txCfg, &isAllocated);
				if (hAif->pktDmaConfig.txChCtrl[i] == NULL)
				{
					Aif2_osalLog("Error: Opening AIF2 Tx channel %d failed\n", txCfg.channelNum);
					return -1;
				}

				/***********************************************************************
	        	 ********************** Receive Flow Configuration *********************
	        	 ***********************************************************************/
				hAif->pktDmaConfig.hRxFlowCtrl[i]->flowIdNum = 124+i;
                Cppi_configureRxFlow (hAif->pktDmaConfig.hCppi, hAif->pktDmaConfig.hRxFlowCtrl[i], &isAllocated);

	        }
    	}
    }

    if (hAif->pktdmaOrDioEngine == AIF2FL_CPPI)
    {
    	if ((hAif->mode == AIF_GENERICPACKET_MODE)){

    	    axcConfigTx =0;
    	    axcConfigRx =0;
    		for(i=0; i< AIF_MAX_NUM_LINKS; i++){
				/***********************************************************************
				 ********************** Transmit Configuration *************************
				 ***********************************************************************/
				if (hAif->linkConfig[i].linkEnable) // check for first RxFlow existing
				{
					numAxC = hAif->linkConfig[i].numPeAxC;
					for (k=0; k <numAxC ; k++)
					{
						/***********************************************************************
						 ********************** Transmit Queues Configuration ******************
						 ***********************************************************************/

						/* Initialize the Transmit descriptors. We want all the transmit descriptors to go
						 * back to the Transmit Free Queue after transmission. */
						descCfg.memRegion                 = hAif->pktDmaConfig.txRegionAxC[axcConfigTx];
						descCfg.descNum                   = hAif->pktDmaConfig.txNumDescAxC[axcConfigTx];
						descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
						descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
						descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
						descCfg.descType                  = Cppi_DescType_HOST;
						descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
						descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
						descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
						//descCfg.returnPushPolicy          = Qmss_Location_TAIL; //FIXME


						/* Initialize the descriptors and place all of them into the general purpose queue */
						hAif->pktDmaConfig.txFqAxC[axcConfigTx]  = Cppi_initDescriptor (&descCfg, &numAllocated);
						if (hAif->pktDmaConfig.txFqAxC[axcConfigTx] < 0)
						{
							Aif2_osalLog("Error: AIF2 Transmit Completion Queue failed to open\n");
							return -1;
						}
						/* Get the Queue Information for the Transmit Free Queue */
						queueInfo = Qmss_getQueueNumber(hAif->pktDmaConfig.txFqAxC[axcConfigTx]);

						/* Pop off all descriptors from the Queue and add the missing init parameters. */
						for (j = 0; j < hAif->pktDmaConfig.txNumDescAxC[axcConfigTx]; j++)
						{
							dataPtr = hAif->pktDmaConfig.txDataBuff[axcConfigTx] + (j * hAif->pktDmaConfig.txDescSizeAxC[axcConfigTx] / 4);
							/* Get a mono descriptor from the free GP queue. */
							ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hAif->pktDmaConfig.txFqAxC[axcConfigTx]));
							Cppi_setData(descCfg.descType, (Cppi_Desc*)ptrHostDesc,(uint8_t*) dataPtr, hAif->pktDmaConfig.txDescSizeAxC[axcConfigTx]);
							Cppi_setDataLen(descCfg.descType,(Cppi_Desc*)ptrHostDesc,0);
							Cppi_setOriginalBufInfo(descCfg.descType, (Cppi_Desc*)ptrHostDesc,(uint8_t*) dataPtr, hAif->pktDmaConfig.txDescSizeAxC[axcConfigTx]);
							Cppi_getNextBD(descCfg.descType,NULL);
							Cppi_setReturnPolicy(descCfg.descType, (Cppi_Desc*)ptrHostDesc,(Cppi_ReturnPolicy)1);
							Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrHostDesc,queueInfo);

							/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
							Qmss_queuePushDesc(hAif->pktDmaConfig.txFqAxC[axcConfigTx], (uint32_t*)ptrHostDesc);
						}

						/* Open the AIF2 transmit queue for this control stream */
						hAif->pktDmaConfig.txQAxC[axcConfigTx] = Qmss_queueOpen (Qmss_QueueType_AIF_QUEUE , QMSS_AIF_QUEUE_BASE+axcConfigTx, &isAllocated);
						if (hAif->pktDmaConfig.txQAxC[axcConfigTx] < 0)
						{
							Aif2_osalLog("Error: AIF2 Transmit Queue failed to open\n");
							return -1;
						}

						/* Open the AIF2 Transmit Channel and keep it disabled. */
                        memset(&txCfg, 0, sizeof(txCfg));
                        txCfg.channelNum   = axcConfigTx;
                        txCfg.priority     = 0;
                        txCfg.txEnable     = Cppi_ChState_CHANNEL_ENABLE;
                        txCfg.filterEPIB   = 0;
                        txCfg.filterPS     = 0;
                        txCfg.aifMonoMode  = 0;
                        hAif->pktDmaConfig.txChAxC[axcConfigTx] = Cppi_txChannelOpen(hAif->pktDmaConfig.hCppi, &txCfg, &isAllocated);
                        if (hAif->pktDmaConfig.txChAxC[axcConfigTx] == NULL)
                        {
                            Aif2_osalLog("Error: Opening AIF2 Tx channel %d failed\n", txCfg.channelNum);
                            return -1;
                        }

						axcConfigTx++;
					}

					/***********************************************************************
					 ********************** Receive Queues Configuration *******************
					 ***********************************************************************/
					numAxC = hAif->linkConfig[i].numPdAxC;
					for (k=0; k <numAxC ; k++)
					{
						/* Initialize the Receive descriptors */
						descCfg.memRegion                 = hAif->pktDmaConfig.rxRegionAxC[axcConfigRx];
						descCfg.descNum                   = hAif->pktDmaConfig.rxNumDescAxC[axcConfigRx];
						descCfg.destQueueNum              = hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->rx_fdq0_sz0_qnum;
						descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
						descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
						descCfg.descType                  = (Cppi_DescType)hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->rx_desc_type;
						descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
						descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
						descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
						descCfg.returnPushPolicy          = Qmss_Location_TAIL;

						/* Initialize the descriptors and place all of them into the general purpose queue */
						hAif->pktDmaConfig.rxFqAxC[axcConfigRx]  = Cppi_initDescriptor (&descCfg, &numAllocated);
						if (hAif->pktDmaConfig.rxFqAxC[axcConfigRx] < 0)
						{
							Aif2_osalLog("Error: AIF2 Receive Completion Queue failed to open\n");
							return -1;
						}

						/* Get the Queue Information for the Transmit Free Queue */
						queueInfo = Qmss_getQueueNumber(hAif->pktDmaConfig.rxFqAxC[axcConfigRx]);

						/* Pop off all descriptors from the Queue and add the missing init parameters. */
						for (j = 0; j < hAif->pktDmaConfig.rxNumDescAxC[axcConfigRx]; j++)
						{
							dataPtr = hAif->pktDmaConfig.rxDataBuff[axcConfigRx] + (j * hAif->pktDmaConfig.txDescSizeAxC[axcConfigRx] / 4);
							/* Get a mono descriptor from the free GP queue. */
							ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hAif->pktDmaConfig.rxFqAxC[axcConfigRx]));
							Cppi_setDataLen(descCfg.descType,(Cppi_Desc*)ptrHostDesc,0);
							Cppi_setOriginalBufInfo(descCfg.descType, (Cppi_Desc*)ptrHostDesc, (uint8_t*) dataPtr, hAif->pktDmaConfig.txDescSizeAxC[axcConfigRx]);
							Cppi_getNextBD(descCfg.descType,NULL);
							Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrHostDesc,queueInfo);

							/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
							Qmss_queuePushDesc(hAif->pktDmaConfig.rxFqAxC[axcConfigRx], (uint32_t*)ptrHostDesc);
						}

						/* Open the AIF2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
						hAif->pktDmaConfig.rxQAxC[axcConfigRx] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->rx_dest_qnum, &isAllocated);
						if (hAif->pktDmaConfig.rxQAxC[axcConfigRx] < 0)
						{
							Aif2_osalLog("Error: AIF2 Receive Queue failed to open\n");
							return -1;
						}

                        /* Open the AIF2 Receive Channel and keep it disabled */
                        memset(&rxCfg, 0, sizeof(rxCfg));
                        rxCfg.channelNum = axcConfigRx;
                        rxCfg.rxEnable   = Cppi_ChState_CHANNEL_ENABLE;
                        hAif->pktDmaConfig.rxChAxC[axcConfigRx] = Cppi_rxChannelOpen(hAif->pktDmaConfig.hCppi, &rxCfg, &isAllocated);
                        if (hAif->pktDmaConfig.rxChAxC[axcConfigRx] == NULL)
                        {
                            Aif2_osalLog("Error: Opening AIF2 Rx channel %d failed\n", rxCfg.channelNum);
                            return -1;
                        }

                        /***********************************************************************
                         ********************** Receive Flow Configuration *********************
                         ***********************************************************************/
                        hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->flowIdNum = axcConfigRx;
                        Cppi_configureRxFlow (hAif->pktDmaConfig.hCppi, hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx], &isAllocated);

						axcConfigRx++;
					}

				}
			}
    	}
    	if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)){
    		axcConfigTx =0;
    		axcConfigRx =0;
    		for(i=0; i< AIF_MAX_NUM_LINKS; i++){
				if ((hAif->linkConfig[i].linkEnable)&&(hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx] != NULL)) // check for first RxFlow existing
					{
					numAxC = hAif->linkConfig[i].numPeAxC;
					for (k=0; k <numAxC ; k++)
						{
						/***********************************************************************
						 ********************** Transmit Configuration *************************
						 ***********************************************************************/

    					/* Initialize the Transmit descriptors. We want all the transmit descriptors to go
    					 * back to the Transmit Free Queue after transmission. */
    					descCfg.memRegion                 = hAif->pktDmaConfig.txRegionAxC[axcConfigTx];
    					descCfg.descNum                   = hAif->pktDmaConfig.txNumDescAxC[axcConfigTx];
    					descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
    					descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    					descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
    					descCfg.descType                  = Cppi_DescType_MONOLITHIC;
    					descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
    					descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
    					descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
    					descCfg.returnPushPolicy          = Qmss_Location_TAIL;
    					descCfg.cfg.mono.dataOffset		  = 12; // size of header for control words
    					if (descCfg.descNum != 0)
    					{
							/* Initialize the descriptors and place all of them into the general purpose queue */
							hAif->pktDmaConfig.txFqAxC[axcConfigTx]  = Cppi_initDescriptor (&descCfg, &numAllocated);
							if (hAif->pktDmaConfig.txFqAxC[axcConfigTx] < 0)
							{
							Aif2_osalLog("Error: AIF2 Transmit Completion Queue failed to open\n");
							return -1;
							}
							/* Get the Queue Information for the Transmit Free Queue */
							queueInfo = Qmss_getQueueNumber(hAif->pktDmaConfig.txFqAxC[axcConfigTx]);

							/* Pop off all descriptors from the Queue and add the missing init parameters. */
							for (j = 0; j < hAif->pktDmaConfig.txNumDescAxC[axcConfigTx]; j++)
								{
								/* Get a mono descriptor from the free GP queue. */
								ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hAif->pktDmaConfig.txFqAxC[axcConfigTx]));
								Cppi_setPacketLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,hAif->pktDmaConfig.txDescSizeAxC[axcConfigTx]);
								descTag.srcTagLo = 0;
								Cppi_setTag(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,&descTag);
								Cppi_setPSFlags(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
								Cppi_setPSLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
								Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);
								/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
								Qmss_queuePushDesc(hAif->pktDmaConfig.txFqAxC[axcConfigTx], (uint32_t*)ptrMonoDesc);
								}

							/* Open the AIF2 transmit queue for this AxC stream */
							hAif->pktDmaConfig.txQAxC[axcConfigTx] = Qmss_queueOpen (Qmss_QueueType_AIF_QUEUE , QMSS_AIF_QUEUE_BASE+axcConfigTx, &isAllocated);
							if (hAif->pktDmaConfig.txQAxC[axcConfigTx] < 0)
								{
								Aif2_osalLog("Error: AIF2 Transmit Queue failed to open\n");
								return -1;
								}

							/***********************************************************************
							 ********************** PktDma channel Configuration *******************
							 ***********************************************************************/

							/* Open the AIF2 Transmit Channel and keep it disabled. */
							memset(&txCfg, 0, sizeof(txCfg));
							txCfg.channelNum   = axcConfigTx;
							txCfg.priority     = 0;
							txCfg.txEnable     = Cppi_ChState_CHANNEL_ENABLE;
							txCfg.filterEPIB   = 0;
							txCfg.filterPS     = 0;
							/* It is possible for the FFTC to send output packets directly to the AIF2 without any DSP intervention. In that case, the AIF_MONO_MODE should
							 * be turned off and FFTC uses a descriptor size field of 16 bytes as normal mode, because FFTC can not set 64 bytes to DESC_SIZE field before
							 * pushing the descriptor into the AIF2 TX queue.
							 */
							txCfg.aifMonoMode  = 0;

							hAif->pktDmaConfig.txChAxC[axcConfigTx] = Cppi_txChannelOpen(hAif->pktDmaConfig.hCppi, &txCfg, &isAllocated);
							if (hAif->pktDmaConfig.txChAxC[axcConfigTx] == NULL)
								{
								Aif2_osalLog("Error: Opening AIF2 Tx channel %d failed\n", txCfg.channelNum);
								return -1;
								}
    					}
						axcConfigTx++;
    					} ///eof Transmit config


    				/***********************************************************************
    	        	 ********************** Receive Queues Configuration *******************
    	        	 ***********************************************************************/
					numAxC = hAif->linkConfig[i].numPdAxC;
					for (k=0; k <numAxC ; k++){
							/* Initialize the Receive descriptors */
							/* Initialize the Receive descriptors */
							descCfg.memRegion                 = hAif->pktDmaConfig.rxRegionAxC[axcConfigRx];
							descCfg.descNum                   = hAif->pktDmaConfig.rxNumDescAxC[axcConfigRx];
							descCfg.destQueueNum              = hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->rx_fdq0_sz0_qnum;
							descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
							descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
							descCfg.descType                  = (Cppi_DescType)hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->rx_desc_type;
							descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
							descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
							descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
							descCfg.returnPushPolicy          = Qmss_Location_TAIL;
							descCfg.cfg.mono.dataOffset		  = hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->rx_sop_offset; // size of header

	    					if (descCfg.descNum != 0)
	    					{
								/* Initialize the descriptors and place all of them into the general purpose queue */
								hAif->pktDmaConfig.rxFqAxC[axcConfigRx]  = Cppi_initDescriptor (&descCfg, &numAllocated);
								if (hAif->pktDmaConfig.rxFqAxC[axcConfigRx] < 0)
									{
									Aif2_osalLog("Error: AIF2 Receive Completion Queue failed to open\n");
									return -1;
									}

								/* Get the Queue Information for the Transmit Free Queue */
								queueInfo = Qmss_getQueueNumber(hAif->pktDmaConfig.rxFqAxC[axcConfigRx]);

								/* Pop off all descriptors from the Queue and add the missing init parameters. */
								for (j = 0; j < hAif->pktDmaConfig.rxNumDescAxC[axcConfigRx]; j++)
									{
									/* Get a mono descriptor from the free GP queue. */
									ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hAif->pktDmaConfig.rxFqAxC[axcConfigRx]));
									Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);

									/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
									Qmss_queuePushDesc(hAif->pktDmaConfig.rxFqAxC[axcConfigRx], (uint32_t*)ptrMonoDesc);
									}

								//Qmss_QueueType_GENERAL_PURPOSE_QUEUE to be verified

								/* Open the AIF2 receive queue for this control stream, queueType does not matter since we specify a valid queue number */
								hAif->pktDmaConfig.rxQAxC[axcConfigRx] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->rx_dest_qnum, &isAllocated);
								if (hAif->pktDmaConfig.rxQAxC[axcConfigRx] < 0)
									{
									Aif2_osalLog("Error: AIF2 Receive Queue failed to open\n");
									return -1;
									}

								/***********************************************************************
								 ********************** PktDma channel Configuration *******************
								 ***********************************************************************/

								/* Open the AIF2 Receive Channel and keep it disabled */
								memset(&rxCfg, 0, sizeof(rxCfg));
								rxCfg.channelNum = axcConfigRx;
								rxCfg.rxEnable   = Cppi_ChState_CHANNEL_ENABLE;
								hAif->pktDmaConfig.rxChAxC[axcConfigRx] = Cppi_rxChannelOpen(hAif->pktDmaConfig.hCppi, &rxCfg, &isAllocated);
								if (hAif->pktDmaConfig.rxChAxC[axcConfigRx] == NULL)
									{
									Aif2_osalLog("Error: Opening AIF2 Rx channel %d failed\n", rxCfg.channelNum);
									return -1;
									}
								/***********************************************************************
								 ********************** Receive Flow Configuration *********************
								 ***********************************************************************/
								hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx]->flowIdNum = axcConfigRx;
								Cppi_configureRxFlow (hAif->pktDmaConfig.hCppi, hAif->pktDmaConfig.hRxFlowAxC[axcConfigRx], &isAllocated);
	    					}

							axcConfigRx++;
						}
				}
    		}
        }
    }
#else
    Aif2_osalLog("Error: AIF2 LLD API supported only on C6X %d \n", (uint32_t)hAif);
#endif
    return 0;
}

typedef struct {
	AIF_SampleRate samplingRate;
	AIF_LteCpType cpType;
	uint32_t startAxC;
	uint32_t endAxC;
	uint32_t isPopulated; //0 if not, 1 if yes
} baseGroupInfo_t;

typedef struct {
	baseGroupInfo_t g;
	uint32_t cellId;
} egrGroupInfo_t;

#define ingrGroupInfo_t baseGroupInfo_t

#define AIF_NUM_PD_PE_FRAME_TC_GROUPS 6
egrGroupInfo_t egrGroupInfo[AIF_NUM_PD_PE_FRAME_TC_GROUPS];

static void initEgrGroupInfo(void)
{
	memset(egrGroupInfo, 0, AIF_NUM_PD_PE_FRAME_TC_GROUPS * sizeof(egrGroupInfo_t));
}

static void initIngrGroupInfo(ingrGroupInfo_t *ingrGroupInfoTable, uint32_t tableSize)
{
	memset(ingrGroupInfoTable, 0, tableSize * sizeof(ingrGroupInfo_t));
}

//returns groupId
static uint32_t addAxCtoEgrGroupTable(uint32_t AxC, AIF_SampleRate samplingRate, AIF_LteCpType cpType, 
									uint32_t cellId, uint32_t *newEntry)
{
	uint32_t i, found = 0, id;
	egrGroupInfo_t *g;
	
	for(i = 0; i < AIF_NUM_PD_PE_FRAME_TC_GROUPS; i++) {
		g = &egrGroupInfo[i];
		if (g->g.isPopulated) {
			//check if entries match
			if ((samplingRate == g->g.samplingRate) && (cpType == g->g.cpType) && (cellId == g->cellId)) {
				//matched, add AxC
				if (AxC < g->g.startAxC) {
					g->g.startAxC = AxC;
				}
				if (AxC > g->g.endAxC) {
					g->g.endAxC = AxC;
				}
				*newEntry = 0;
				return(i);
			}
		}
	}

	//find the first unpopulated group and populate it
	for(i = 0; i < AIF_NUM_PD_PE_FRAME_TC_GROUPS; i++) {
		g = &egrGroupInfo[i];
		if (! g->g.isPopulated) {
			found = 1;
			id = i;
			break;
		}
	}
	if (! found) {  //error
		return(AIF_NUM_PD_PE_FRAME_TC_GROUPS); //return invalid groupId
	}

	//populate
	g->g.samplingRate = samplingRate;
	g->cellId = cellId;
	g->g.cpType = cpType;
	g->g.startAxC = AxC;
	g->g.isPopulated = 1;
	*newEntry = 1;
	return(id);
}

//returns groupId
static uint32_t addAxCtoIngrGroupTable(ingrGroupInfo_t *ingrGroupInfoTable, uint32_t tableSize, uint32_t AxC, 
						AIF_SampleRate samplingRate, AIF_LteCpType cpType, uint32_t *newEntry)
{
	uint32_t i, found = 0, id;
	ingrGroupInfo_t *g;

	for(i = 0; i < tableSize; i++) {
		g = &ingrGroupInfoTable[i];
		if (g->isPopulated) {
			//check if entries match
			if ((samplingRate == g->samplingRate) && (cpType == g->cpType)) {
				//matched, add AxC
				if (AxC < g->startAxC) {
					g->startAxC = AxC;
				}
				if (AxC > g->endAxC) {
					g->endAxC = AxC;
				}
				*newEntry = 0;
				return(i);
			}
		}
	}

	//find the first unpopulated group and populate it
	for(i = 0; i < tableSize; i++) {
		g = &ingrGroupInfoTable[i];
		if (! g->isPopulated) {
			found = 1;
			id = i;
			break;
		}
	}
	if (! found) {  //error
		return(tableSize); //invalid group Id
	}
	//populate
	g->samplingRate = samplingRate;
	g->cpType = cpType;
	g->startAxC = AxC;
	g->isPopulated = 1;
	*newEntry = 1;
	return(id);
}

static uint32_t getEgrGroupId(uint32_t AxC)
{
	uint32_t i;

	for(i = 0; i < AIF_NUM_PD_PE_FRAME_TC_GROUPS; i++) {
		if ((AxC >= egrGroupInfo[i].g.startAxC) && (AxC <= egrGroupInfo[i].g.endAxC)) {
			return(i);
		}
	}
	return(AIF_NUM_PD_PE_FRAME_TC_GROUPS); //invalid group
}

uint32_t AIF_getEgressGroupId(uint32_t AxC)
{
	return(getEgrGroupId(AxC));
}

/* Setup AIF given user configuration*/

void AIF_initHw(
		AIF_ConfigHandle  hAif
)
{
  Aif2Fl_Status status;

  uint32_t  	i,j,k,z;
#ifndef K2
  uint8_t  	serdes_blockb8_used =0;
  uint8_t  	serdes_blockb4_used =0;
#endif

  uint32_t 	  offset_channel_pe, offset_channel_pd, max_channel_pe, max_channel_pd, obsai_type=0, first_link, baseAddress, baseIngr=0, baseEgr=0;
  uint32_t 	  EventModulo,DioFrameEventOffset;
  uint32_t    dbmxPe[6];
  uint32_t    dbmxPd;
  uint32_t    numAxCPd;
  uint32_t    maxAxC = 0, totalAxC = 0;
  uint32_t	  FrameMsg1=0, FrameMsg=0, FrameMsg2=0;
  uint32_t 	  tddSubFrameBitMap, offsetTdd, tddSpecSubFrameBitMap;
  uint8_t 	  tddTable[140];
  uint32_t 	  lte1_4 = 0;

  uint32_t	  samplePerFrame = 0;
  uint8_t     frameIndexSc  = 0;
  uint32_t    linkRate;

  AIF_SampleRate sampleRate;
  uint32_t    groupId=0, cellId, isNewGroup=0;
  AIF_LteCpType lteCpType;
  ingrGroupInfo_t ingrGroupInfo[AIF_NUM_PD_PE_FRAME_TC_GROUPS];

  uint32_t    clockCountTc[7];

   //////////////////Initialize Aif2 structures to avoid unwanted configuration ////////////////////////////////////////
   memset(&globalSetup, 0, sizeof(globalSetup));
   for(i=0; i< AIF_MAX_NUM_LINKS; i++)
      memset(&linkSetup[i], 0, sizeof(Aif2Fl_LinkSetup));
   memset(&commonSetup, 0, sizeof(commonSetup));

   memset(&SdCommonSetup, 0, sizeof(SdCommonSetup));
   memset(&PdCommonSetup, 0, sizeof(PdCommonSetup));
   memset(&PeCommonSetup, 0, sizeof(PeCommonSetup));
   memset(&IngrDbSetup, 0, sizeof(IngrDbSetup));
   memset(&EgrDbSetup, 0, sizeof(EgrDbSetup));
   memset(&AdCommonSetup, 0, sizeof(AdCommonSetup));
   memset(&AdDioSetup, 0, sizeof(AdDioSetup));
   memset(&AtCommonSetup, 0, sizeof(AtCommonSetup));
   memset(&AtEventSetup, 0, sizeof(AtEventSetup));
   memset(&PhyTimerInit, 0, sizeof(PhyTimerInit));
   memset(&RadTimerInit, 0, sizeof(RadTimerInit));
   memset(&UlRadTimerInit, 0, sizeof(UlRadTimerInit));
   memset(&DlRadTimerInit, 0, sizeof(DlRadTimerInit));
   memset(&PhyTimerTc, 0, sizeof(PhyTimerTc));
   memset(&RadTimerTc, 0, sizeof(RadTimerTc));

   for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
	   memset(&ComLinkSetup[i], 0, sizeof(Aif2Fl_CommonLinkSetup));
	   memset(&SdLinkSetup[i], 0, sizeof(Aif2Fl_SdLinkSetup));
	   memset(&RmLinkSetup[i], 0, sizeof(Aif2Fl_RmLinkSetup));
	   memset(&TmLinkSetup[i], 0, sizeof(Aif2Fl_TmLinkSetup));
	   memset(&PdLinkSetup[i], 0, sizeof(Aif2Fl_PdLinkSetup));
	   memset(&PeLinkSetup[i], 0, sizeof(Aif2Fl_PeLinkSetup));
	   memset(&RtLinkSetup[i], 0, sizeof(Aif2Fl_RtLinkSetup));
	   memset(&AtLinkSetup[i], 0, sizeof(Aif2Fl_AtLinkSetup));
   }

   //FIXME definition of superpacket variable already in pktdmaInit
   for(i=0; i< AIF_MAX_NUM_LINKS; i++)
   {
	   if (hAif->linkConfig[i].linkEnable == 1)
	   {
		   if (hAif->superPacket == true)
		   {
			   superPacket = true;
			   hAif->linkConfig[i].numPdAxC=1;
		   } else {
			   superPacket = false;
		   }
	   }
   }

   /* Initialize CSL library, this step is required - Does not do anything at the moment! */
   status = Aif2Fl_init(&Aif2Context);
   if (status != AIF2FL_SOK)
	   Aif2_osalLog("bad context initialization \n");

#ifndef _TMS320C6X
   pGlobalAif2Cfg = hAif->hAif2SerDesBaseAddr;
#endif

   hAif->hFl = Aif2Fl_open(&Aif2Obj, 0, &aif2Param, &status);

   hAif->hAif2Setup = &aif2Setup;

   /////////////////populating AIF2 major setup structures //////////////////////////////////////////////////////////
   aif2Setup.globalSetup = &globalSetup;
   aif2Setup.commonSetup = &commonSetup;
   for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
	   aif2Setup.linkSetup[i] = &linkSetup[i];
   }

   // populate global config fields
   for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
      if(1==hAif->linkConfig[i].linkEnable) {
    	  globalSetup.ActiveLink[i] = true;
    	  totalAxC = totalAxC+ hAif->linkConfig[i].numPeAxC;
      }
   }
   globalSetup.frameMode = AIF2FL_FRAME_MODE_NORMAL;

   //populate common config fields
#ifdef K2
   commonSetup.pSdCommonSetup = NULL;
#else   
   commonSetup.pSdCommonSetup = &SdCommonSetup;
#endif   
   commonSetup.pPdCommonSetup = &PdCommonSetup;
   commonSetup.pPeCommonSetup = &PeCommonSetup;
   commonSetup.pIngrDbSetup   = &IngrDbSetup;
   commonSetup.pEgrDbSetup    = &EgrDbSetup;
   commonSetup.pAdCommonSetup = &AdCommonSetup;
   commonSetup.pAdDioSetup    = &AdDioSetup;
   commonSetup.pAtCommonSetup = &AtCommonSetup;
   commonSetup.pAtEventSetup  = &AtEventSetup;

   for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
	   ////////////Link Setup (Do this setup repeatedly with different link setup structure if user wants to use multiple links)////////////
	   //populate link config fields for link 1
	   linkSetup[i].linkIndex     = (Aif2Fl_LinkIndex)i;
	   linkSetup[i].pComLinkSetup = &ComLinkSetup[i];
#ifdef K2	   
	   linkSetup[i].pSdLinkSetup  = NULL;
#else
	   linkSetup[i].pSdLinkSetup  = &SdLinkSetup[i];
#endif	   
	   linkSetup[i].pRmLinkSetup  = &RmLinkSetup[i];
	   linkSetup[i].pTmLinkSetup  = &TmLinkSetup[i];
	   linkSetup[i].pPdLinkSetup  = &PdLinkSetup[i];
	   linkSetup[i].pPeLinkSetup  = &PeLinkSetup[i];
	   linkSetup[i].pRtLinkSetup  = &RtLinkSetup[i];
	   linkSetup[i].pAtLinkSetup  = &AtLinkSetup[i];
   }

   for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
	   //Link Common setup
		ComLinkSetup[i].linkProtocol 	= hAif->protocol;
		ComLinkSetup[i].IngrDataWidth 	= hAif->linkConfig[i].inboundDataWidth;
		ComLinkSetup[i].EgrDataWidth 	= hAif->linkConfig[i].outboundDataWidth;
		if(8==hAif->linkConfig[i].linkRate){
			ComLinkSetup[i].linkRate = AIF2FL_LINK_RATE_8x;
		}
		else if (2==hAif->linkConfig[i].linkRate) {
			ComLinkSetup[i].linkRate = AIF2FL_LINK_RATE_2x;
		}
		else if (5==hAif->linkConfig[i].linkRate) {
			ComLinkSetup[i].linkRate = AIF2FL_LINK_RATE_5x; //only for CPRI
		}
		else {
			ComLinkSetup[i].linkRate = AIF2FL_LINK_RATE_4x;
		}
   }

#ifndef K2
	for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
		// populate SD link fields default for AIF2 2 AIF2 or Loopback modes
        SdLinkSetup[i].rxAlign 					= AIF2FL_SD_RX_COMMA_ALIGNMENT_ENABLE;
        SdLinkSetup[i].rxLos 					= AIF2FL_SD_RX_LOS_ENABLE;
        SdLinkSetup[i].rxCdrAlgorithm 			= (Aif2Fl_SdRxCdrAlg)1; // Newer CSL AIF2FL_SD_RX_CDR_SECOND_ORDER_THRESH_7 (not in PDK yet); Older CSL AIF2FL_SD_RX_CDR_FIRST_ORDER_THRESH_17;
        SdLinkSetup[i].rxInvertPolarity 		= AIF2FL_SD_RX_NORMAL_POLARITY;
        SdLinkSetup[i].rxTermination 			= AIF2FL_SD_RX_TERM_COMMON_POINT_0_7;
        SdLinkSetup[i].rxEqualizerConfig 		= AIF2FL_SD_RX_EQ_ADAPTIVE;
        SdLinkSetup[i].bRxEqHold 				= false; //inline with rxEqualizerConfig
        SdLinkSetup[i].bRxOffsetComp 			= true; //for equalization not used
        SdLinkSetup[i].bEnableTxSyncMater 		= true; //enable link as master lane for Synchro
        SdLinkSetup[i].txInvertPolarity 		= AIF2FL_SD_TX_PAIR_NORMAL_POLARITY;
        SdLinkSetup[i].txOutputSwing 			= AIF2FL_SD_TX_OUTPUT_SWING_14;
        SdLinkSetup[i].txPrecursorTapWeight 	= AIF2FL_SD_TX_PRE_TAP_WEIGHT_2; //- 5%
        SdLinkSetup[i].txPostcursorTapWeight 	= AIF2FL_SD_TX_POST_TAP_WEIGHT_24; // +20%
        SdLinkSetup[i].bTxFirFilterUpdate 		= true;
        if (hAif->linkConfig[i].comType == AIF1_2_AIF2) {
               SdLinkSetup[i].rxLos 					= AIF2FL_SD_RX_LOS_DISABLE;
               SdLinkSetup[i].rxCdrAlgorithm 			= AIF2FL_SD_RX_CDR_FIRST_ORDER_THRESH_1;
               SdLinkSetup[i].txOutputSwing 			= AIF2FL_SD_TX_OUTPUT_SWING_15;
               SdLinkSetup[i].bRxOffsetComp 			= false;
               SdLinkSetup[i].txPrecursorTapWeight 	    = AIF2FL_SD_TX_PRE_TAP_WEIGHT_0;
               SdLinkSetup[i].txPostcursorTapWeight 	= AIF2FL_SD_TX_POST_TAP_WEIGHT_0;
               SdLinkSetup[i].bTxFirFilterUpdate 		= false;//FIR filter update on
        }
	}

#else
	   /*SdLinkSetup[0].rxAlign = AIF2FL_SD_RX_COMMA_ALIGNMENT_ENABLE;
	   SdLinkSetup[0].rxLos = AIF2FL_SD_RX_LOS_ENABLE;*/
#endif

	for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
	    //TM link setup
	    TmLinkSetup[i].bEnableTmLink = true;
	    TmLinkSetup[i].bEnableRmLos = false;
	    if (8==hAif->linkConfig[i].linkRate && hAif->protocol == AIF2FL_LINK_PROTOCOL_OBSAI)
	    	TmLinkSetup[i].SeedValue = 0x2;//this should be non-zero value for scrambler
	    else
	    	TmLinkSetup[i].SeedValue = 0x1;//this should be non-zero value for scrambler
	    if (8==hAif->linkConfig[i].linkRate && hAif->protocol == AIF2FL_LINK_PROTOCOL_OBSAI)
	    	TmLinkSetup[i].bEnableScrambler = true;//8x rates requires scrambler
	    else
	    	TmLinkSetup[i].bEnableScrambler = false;//other rates requires no scrambler
	    if (AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol){
		    TmLinkSetup[i].pCpriTmSetup.L1InbandEn = 0;//disable 9 bits mask
		    TmLinkSetup[i].pCpriTmSetup.RmLinkLosError = (Aif2Fl_LinkIndex)i;//select link  as source RM link
		    TmLinkSetup[i].pCpriTmSetup.RmLinkLofError = (Aif2Fl_LinkIndex)i;//select link  as source RM link
		    TmLinkSetup[i].pCpriTmSetup.RmLinkLosRx = (Aif2Fl_LinkIndex)i;//select link  as source RM link
		    TmLinkSetup[i].pCpriTmSetup.RmLinkLofRx = (Aif2Fl_LinkIndex)i;//select link  as source RM link
		    TmLinkSetup[i].pCpriTmSetup.RmLinkRaiRx = (Aif2Fl_LinkIndex)i;//select link  as source RM link
		    TmLinkSetup[i].pCpriTmSetup.TxStartup = 0;
		    TmLinkSetup[i].pCpriTmSetup.TxPointerP = 20;
		    TmLinkSetup[i].pCpriTmSetup.TxProtocolVer = 1;
	    }
	}

	for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
		// populate Rx MAC link fields
		RmLinkSetup[i].bEnableRmLink = true; //EnableRxMac
		RmLinkSetup[i].RmFifoThold = AIF2FL_RM_FIFO_THOLD_4DUAL; // 4 dual words or 8 dual words to add more delay for adjusting jitter
		RmLinkSetup[i].bEnableLcvControl = true; //line code violation counter
		RmLinkSetup[i].RmErrorSuppress = AIF2FL_RM_ERROR_ALLOW; // to be verified in aif1
		RmLinkSetup[i].bEnableSdAutoAlign = false;
		if (8==hAif->linkConfig[i].linkRate && hAif->protocol == AIF2FL_LINK_PROTOCOL_OBSAI)
			RmLinkSetup[i].bEnableScrambler = true;//4x link rate requires no scrambler
		else
			RmLinkSetup[i].bEnableScrambler = false;//4x link rate requires no scrambler
		RmLinkSetup[i].bEnableLcvUnsync = false;
		//RmLinkSetup[i].bEnableLcvControl 	= false;
		RmLinkSetup[i].bEnableWatchDog = false;
		RmLinkSetup[i].WatchDogWrap = 0xFF;//set watch dog wrap value
		RmLinkSetup[i].bEnableClockQuality = false;
		RmLinkSetup[i].ClockMonitorWrap = 0;
		if (hAif->linkConfig[i].comType == AIF2_2_AIF2 || hAif->linkConfig[i].comType == AIF2_LOOPBACK) {
			RmLinkSetup[i].losDetThreshold = AIF2_RM_LOS_DET_THOLD;
			RmLinkSetup[i].SyncThreshold = AIF2_RM_SYNC_THOLD;
			RmLinkSetup[i].FrameSyncThreshold = AIF2_RM_SYNC_THOLD;
			RmLinkSetup[i].UnsyncThreshold = AIF2_RM_UNSYNC_THOLD;
			RmLinkSetup[i].FrameUnsyncThreshold = AIF2_RM_UNSYNC_THOLD;
		} else {
			RmLinkSetup[i].losDetThreshold = AIF1_RM_LOS_DET_THOLD;
			RmLinkSetup[i].SyncThreshold = AIF1_RM_SYNC_THOLD;
			RmLinkSetup[i].FrameSyncThreshold = AIF1_RM_SYNC_THOLD;
			RmLinkSetup[i].UnsyncThreshold = AIF1_RM_UNSYNC_THOLD;
			RmLinkSetup[i].FrameUnsyncThreshold = AIF1_RM_UNSYNC_THOLD;
		}
	}

   for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
		//RT link setup
	    if (hAif->linkConfig[i].RtEnabled)
	    {
	    	RtLinkSetup[i].CiSelect = hAif->linkConfig[i].RtLinkRout;
	    	RtLinkSetup[i].bEnableEmptyMsg = true;
	    	RtLinkSetup[i].RtConfig = AIF2FL_RT_MODE_RETRANSMIT;// takes PE input only
	    } else {
			RtLinkSetup[i].CiSelect = (Aif2Fl_LinkIndex)i;
			RtLinkSetup[i].bEnableEmptyMsg = true;
			RtLinkSetup[i].RtConfig = AIF2FL_RT_MODE_TRANSMIT;// takes PE input only
	    }
	}

    // Compute first DB channel for each link
    offset_channel_pe = 0; offset_channel_pd = 0; first_link = AIF_MAX_NUM_LINKS + 1;
    for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
    	if (1 == hAif->linkConfig[i].linkEnable) {
			hAif->linkConfig[i].firstPeDBCH = offset_channel_pe;
			hAif->linkConfig[i].firstPdDBCH = offset_channel_pd;
			offset_channel_pe += hAif->linkConfig[i].numPeAxC;
			offset_channel_pd += hAif->linkConfig[i].numPdAxC;
			if (first_link == AIF_MAX_NUM_LINKS + 1) first_link = i;
		}
    }
    max_channel_pe = offset_channel_pe;
    max_channel_pd = offset_channel_pd;


    for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
		// populate PD link fields
    	if((hAif->linkConfig[i].numPdAxC != 0) || (hAif->linkConfig[i].psMsgEnable == 1))
    		PdLinkSetup[i].bEnablePdLink = true; //EnablePd
		PdLinkSetup[i].Crc8Poly = AIF2_CRC8_POLY;
		PdLinkSetup[i].Crc8Seed = AIF2_CRC8_SEED;
		maxAxC	= hAif->linkConfig[i].numPdAxC;
		if (AIF2FL_LINK_PROTOCOL_CPRI == hAif->protocol) {
			if ((AIF2FL_DATA_WIDTH_16_BIT==hAif->linkConfig[i].inboundDataWidth)||
			(AIF2FL_DATA_WIDTH_8_BIT==hAif->linkConfig[i].inboundDataWidth))
			{
				if (hAif->linkConfig[i].linkRate == 8)
					dbmxPd		= hAif->linkConfig[i].linkRate*4 - 2;
				else
					dbmxPd		= hAif->linkConfig[i].linkRate*4 - 1;
				//numAxCPe	= maxAxC;
			} else {
				dbmxPd		= hAif->linkConfig[i].linkRate*4;
			}
			numAxCPd	= maxAxC; // default for WCDMA
			if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)){
				if (superPacket == 0)
				{
					if (hAif->linkConfig[i].sampleRate == AIF_SRATE_1P92MHZ)   dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/1920);
					if (hAif->linkConfig[i].sampleRate == AIF_SRATE_7P68MHZ)   dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/7680);
					if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_15P36MHZ) dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/15360);
					if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_23P04MHZ) dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/30720); // same as LTE15
					if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_30P72MHZ) dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/30720);
				} else {
					dbmxPd      = hAif->linkConfig[i].numPdAxC;
				}
			}
			if (hAif->mode == AIF_LTE_WCDMA_MODE){
				if (hAif->linkConfig[i].mode == LTE)
				{
					if (superPacket == 0)
					{
//						dbmxPd	    = hAif->linkConfig[i].linkRate >> 1;
						if (hAif->linkConfig[i].sampleRate == AIF_SRATE_1P92MHZ)   dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/1920);
						if (hAif->linkConfig[i].sampleRate == AIF_SRATE_7P68MHZ)   dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/7680);
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_15P36MHZ) dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/15360);
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_23P04MHZ) dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/30720); // same as LTE15
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_30P72MHZ) dbmxPd		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/30720);
					}
					else
						dbmxPd      = hAif->linkConfig[i].numPdAxC;
				} else {
					if ((AIF2FL_DATA_WIDTH_16_BIT==hAif->linkConfig[i].inboundDataWidth)||
							(AIF2FL_DATA_WIDTH_8_BIT==hAif->linkConfig[i].inboundDataWidth))
					{
						if (hAif->linkConfig[i].linkRate == 8)
							dbmxPd		= hAif->linkConfig[i].linkRate*4 - 2;
						else
							dbmxPd		= hAif->linkConfig[i].linkRate*4 - 1;
					} else {
						dbmxPd		= hAif->linkConfig[i].linkRate*4;
					}
				}
			}
			if (hAif->mode == AIF_GENERICPACKET_MODE) numAxCPd = 1;
			PdLinkSetup[i].CpriEnetStrip = 0;
			PdLinkSetup[i].CpriCwNullDelimitor = 0xFB;//K 27.7 charactor
			if (hAif->linkConfig[i].inboundDataType==AIF2FL_LINK_DATA_TYPE_RSA)
			{
				PdLinkSetup[i].PdCpriCrcType[0] = AIF2FL_CRC_8BIT;
				if (hAif->linkConfig[i].inboundDataWidth == AIF2FL_DATA_WIDTH_7_BIT) PdLinkSetup[i].CpriAxCPack = AIF2FL_CPRI_7BIT_SAMPLE;
				else PdLinkSetup[i].CpriAxCPack = AIF2FL_CPRI_8BIT_SAMPLE;
			} else {
				PdLinkSetup[i].PdCpriCrcType[0] = AIF2FL_CRC_16BIT;
				if (hAif->mode != AIF_GENERICPACKET_MODE){
					if (hAif->linkConfig[i].inboundDataWidth == AIF2FL_DATA_WIDTH_15_BIT) PdLinkSetup[i].CpriAxCPack = AIF2FL_CPRI_15BIT_SAMPLE;
					else PdLinkSetup[i].CpriAxCPack = AIF2FL_CPRI_16BIT_SAMPLE;
				}
			}
			PdLinkSetup[i].bEnableCpriCrc[0] = false;//disable CPRI CRC for control channel 0,
			if (hAif->mode == AIF_GENERICPACKET_MODE){
				PdLinkSetup[i].PdPackDmaCh[hAif->linkConfig[i].firstPdDBCH] = hAif->linkConfig[i].firstPdDBCH;
				PdLinkSetup[i].bEnablePack[hAif->linkConfig[i].firstPdDBCH] = true;// true to enable CPRI control channel 0 packing
				PdLinkSetup[i].CpriCwPktDelimitor[hAif->linkConfig[i].firstPdDBCH] = AIF2FL_CW_DELIM_4B5B;
			} else {
				for (j=0;j<AIF2_CPRI_MAX_CW_SUBSTREAM;j++)
				{
					if (hAif->linkConfig[i].psMsgEnable)
					{
						PdLinkSetup[i].bEnableCpriCrc[j] = false;//disable CPRI CRC for control channel 0,
						PdLinkSetup[i].PdCpriCrcType[j] = AIF2FL_CRC_32BIT;
					}
					PdLinkSetup[i].PdPackDmaCh[j] = 124+j;//Set DB channel 124 as a dma ch for control channel 0
					if (hAif->linkConfig[i].psMsgEnable) PdLinkSetup[i].bEnablePack[j] = true;
					else								 PdLinkSetup[i].bEnablePack[j] = false;
					if (j==0) PdLinkSetup[i].CpriCwPktDelimitor[j] = AIF2FL_CW_DELIM_4B5B;
					else      PdLinkSetup[i].CpriCwPktDelimitor[j] = AIF2FL_CW_DELIM_NULLDELM;
				}
			}

			PdLinkSetup[i].PdCpriDualBitMap.DbmX = dbmxPd - 1;// set X-1 where X calculated in calcParam
			if (hAif->mode == AIF_GENERICPACKET_MODE) PdLinkSetup[i].PdCpriDualBitMap.DbmXBubble = 0;
			else PdLinkSetup[i].PdCpriDualBitMap.DbmXBubble = 1;//2 bubbles of 1 AxC sample
			PdLinkSetup[i].PdCpriDualBitMap.Dbm1Mult = 0;//set n-1
			PdLinkSetup[i].PdCpriDualBitMap.Dbm1Size = 0;//set n-1
			PdLinkSetup[i].PdCpriDualBitMap.Dbm1Map[0] = 0x0;
			PdLinkSetup[i].PdCpriDualBitMap.Dbm2Size = 0;
			PdLinkSetup[i].PdCpriDualBitMap.Dbm2Map[0] = 0x0;
			if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_23P04MHZ)
			{
				if (hAif->linkConfig[i].linkRate == 4)
				{
					PdLinkSetup[i].PdCpriDualBitMap.DbmX = 1; // forcing format to A0, A1,A0,A1,A0,A1,B,B
					PdLinkSetup[i].PdCpriDualBitMap.DbmXBubble = 1;// bubbles of 2 AxC samples
					PdLinkSetup[i].PdCpriDualBitMap.Dbm1Mult = 0;//set n-1
					PdLinkSetup[i].PdCpriDualBitMap.Dbm1Size = 2;//set n-1
					PdLinkSetup[i].PdCpriDualBitMap.Dbm1Map[0] = 0x4;
					PdLinkSetup[i].PdCpriDualBitMap.Dbm2Size = 0;
					PdLinkSetup[i].PdCpriDualBitMap.Dbm2Map[0] = 0x0;
				} else if (hAif->linkConfig[i].linkRate == 8)
				{
					PdLinkSetup[i].PdCpriDualBitMap.DbmX = 4; // forcing format to A0, A1,A2,A3,A4,A0, A1,A2,A3,A4,B A0, A1,A2,A3,A4
					PdLinkSetup[i].PdCpriDualBitMap.DbmXBubble = 0;// bubbles of 1 AxC sample
					PdLinkSetup[i].PdCpriDualBitMap.Dbm1Mult = 0;//set n-1
					PdLinkSetup[i].PdCpriDualBitMap.Dbm1Size = 2;//set n-1
					PdLinkSetup[i].PdCpriDualBitMap.Dbm1Map[0] = 0x2;
					PdLinkSetup[i].PdCpriDualBitMap.Dbm2Size = 0;
					PdLinkSetup[i].PdCpriDualBitMap.Dbm2Map[0] = 0x0;
				}
			}

			if (((superPacket == 0) && (hAif->mode == AIF_LTE_FDD_MODE)) || (hAif->mode == AIF_LTE_WCDMA_MODE) || (hAif->mode == AIF_LTE_TDD_MODE))
			{
				k=0;
				if (hAif->linkConfig[i].multiRate == 0){
					for (z=0;z<=numAxCPd;z++)
					{
						for (j=0; j<dbmxPd; j++)
									{
							if ((hAif->linkConfig[i].maskPattern[k]>>j)&1)
							{
								PdLinkSetup[i].CpriDmaCh[j+(hAif->linkConfig[i].firstPdAxC)] = ((hAif->linkConfig[i].firstPdDBCH) + z);
								PdLinkSetup[i].bEnableCpriX[j+(hAif->linkConfig[i].firstPdAxC)] = TRUE; //enable CPRI X channel
							}
						}
						k++;
					}
				} else {
					for (z=0;z<numAxCPd;z++)
					{
						for (j=0; j<dbmxPd; j++)
						{
							if ((hAif->AxCconfig[hAif->linkConfig[i].firstPdDBCH+k].maskPattern>>j)&1)
							{
								PdLinkSetup[i].CpriDmaCh[j] = ((hAif->linkConfig[i].firstPdDBCH) + z);
								PdLinkSetup[i].bEnableCpriX[j] = TRUE; //enable CPRI X channel
							}
						}
						k++;
					}
				}
			}

			/*
			 * If the DSP 0 will send data on 8x link to DSP 1 and from DSP 1 retransmitted to DSP 0
			 * On DSP 1 side I want to enable PD channels 0-14 --- > pdCommonSetup_.PdChConfig[i].bChannelEn and map the AxC 0 - 14 to those channels.
			 * On DSP 0 side I want to enable PD channels 0-14 ---> pdCommonSetup_.PdChConfig[i].bChannelEn and map the AxC 15 - 29 to those channels.
			 * For the PD, you need to enable 15 channels (i = [0-14])  with pdCommonSetup_.PdChConfig[i].bChannelEn.
			 * Then map those channels to the correct AxCs:
			 * PdLinkSetup[i].CpriDmaCh[j] = i; with i = [0-14] and (j = [0-14] on DSP_1, j = [15-30] on DSP_0).
			 *
			 */

			for (j = 0; j < numAxCPd; j++)//cpri id lut setup for X position 0,1,2... numAxC   //FIXME
			{
				if ((superPacket == 1) || (((hAif->mode != AIF_LTE_FDD_MODE) && (hAif->mode != AIF_LTE_TDD_MODE) && (hAif->mode != AIF_LTE_WCDMA_MODE))))
				{
					PdLinkSetup[i].CpriDmaCh[j+(hAif->linkConfig[i].firstPdAxC)] = (hAif->linkConfig[i].firstPdDBCH + j);
					PdLinkSetup[i].bEnableCpriX[j+(hAif->linkConfig[i].firstPdAxC)] = true; //enable CPRI X channel
				}
				if (hAif->mode == AIF_GENERICPACKET_MODE)
					PdLinkSetup[i].bEnableCpriPkt[j+(hAif->linkConfig[i].firstPdAxC)] = true;//use AxC data mode
				else
					PdLinkSetup[i].bEnableCpriPkt[j+(hAif->linkConfig[i].firstPdAxC)] = false;//use AxC data mode
				if ((hAif->linkConfig[i].linkRate==8) && (hAif->linkConfig[i].inboundDataType==AIF2FL_LINK_DATA_TYPE_RSA) && (hAif->linkConfig[i].firstPdDBCH+21<(j+(hAif->linkConfig[i].firstPdAxC)) && (j+(hAif->linkConfig[i].firstPdAxC))<hAif->linkConfig[i].firstPdDBCH+30))
					PdLinkSetup[i].Cpri8WordOffset[j+(hAif->linkConfig[i].firstPdAxC)] = 1;
				else
					PdLinkSetup[i].Cpri8WordOffset[j+(hAif->linkConfig[i].firstPdAxC)] = hAif->linkConfig[i].cpri8WordOffset[j];//more detailed CPRI AxC offset
			}
			if ((hAif->mode != AIF_GENERICPACKET_MODE) && ( 8!= hAif->linkConfig[i].linkRate)){
				for (j = 0; j < 256; j++)//cpri cw lut setup
				{
					if (((j >= 20) && (j < 64)) || ((j >= 84) && (j < 128)) || ((j>= 148) &&\
							(j < 192)) || ((j >= 212) && (j < 256))) {
						PdLinkSetup[i].CpriCwChannel[j] = 0; //set CW sub channel num to pack 0
						PdLinkSetup[i].bEnableCpriCw[j] = true; //enable CW sub channel for FastC&M
					}
				}
			}
		} else { // OBSAI
			obsai_type = 0;
			if (hAif->mode == AIF_WCDMA_MODE) {
				obsai_type = AIF2_OBSAI_TYPE_WCDMA_FDD;
			}
			if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)) {
				obsai_type = AIF2_OBSAI_TYPE_LTE;
			}
			if (hAif->mode == AIF_GENERICPACKET_MODE)
				obsai_type = AIF2_OBSAI_TYPE_GENERIC;
			if (hAif->mode == AIF_LTE_WCDMA_MODE)
			{
				if (hAif->linkConfig[i].mode == LTE)
				{
					obsai_type = AIF2_OBSAI_TYPE_LTE;
				} else {
				obsai_type = AIF2_OBSAI_TYPE_WCDMA_FDD;
				}
			}

			if (hAif->mode == AIF_GENERICPACKET_MODE){
				PdLinkSetup[i].PdTypeLut[obsai_type].ObsaiTsFormat    = AIF2FL_TSTAMP_FORMAT_GEN_PKT;
				PdLinkSetup[i].PdTypeLut[obsai_type].PdCrcType        = AIF2FL_CRC_16BIT;
				PdLinkSetup[i].PdTypeLut[obsai_type].bEnableCrc       = false;
				PdLinkSetup[i].PdTypeLut[obsai_type].PdObsaiMode      = AIF2FL_PD_DATA_PKT;

			} else {
				PdLinkSetup[i].PdTypeLut[obsai_type].ObsaiTsFormat    = AIF2FL_TSTAMP_FORMAT_NORM_TS;
				if (hAif->linkConfig[i].inboundDataType==AIF2FL_LINK_DATA_TYPE_RSA)
					PdLinkSetup[i].PdTypeLut[obsai_type].PdCrcType    = AIF2FL_CRC_8BIT;
				else
					PdLinkSetup[i].PdTypeLut[obsai_type].PdCrcType    = AIF2FL_CRC_16BIT;
				PdLinkSetup[i].PdTypeLut[obsai_type].bEnableCrc       = false;
				PdLinkSetup[i].PdTypeLut[obsai_type].PdObsaiMode      = AIF2FL_PD_DATA_AXC;
			}
			PdLinkSetup[i].PdTypeLut[obsai_type].bEnableEnetStrip = false;
			PdLinkSetup[i].PdTypeLut[obsai_type].bEnableCrcHeader = false;//OBSAI crc calculation for header
		}
	}

    for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
		if (hAif->linkConfig[i].RtEnabled==0)
		{
			// populate PE link fields
			if((hAif->linkConfig[i].numPeAxC != 0) || (hAif->linkConfig[i].psMsgEnable == 1))
				PeLinkSetup[i].bEnablePeLink = true; //bEnablePe = true;
			PeLinkSetup[i].PeCppiDioSel = hAif->pktdmaOrDioEngine;
			if (hAif->mode == AIF_LTE_TDD_MODE)
				PeLinkSetup[i].TddAxc = true;
			else
				PeLinkSetup[i].TddAxc = false;
			PeLinkSetup[i].Crc8Poly = AIF2_CRC8_POLY;
			PeLinkSetup[i].Crc8Seed = AIF2_CRC8_SEED;
			if (AIF2FL_LINK_PROTOCOL_CPRI == hAif->protocol) {
				maxAxC	= hAif->linkConfig[i].numPeAxC;
				if ((AIF2FL_DATA_WIDTH_16_BIT==hAif->linkConfig[i].outboundDataWidth)||
					(AIF2FL_DATA_WIDTH_8_BIT==hAif->linkConfig[i].outboundDataWidth))
					{
						if (hAif->linkConfig[i].linkRate == 8)
							dbmxPe[i]		= hAif->linkConfig[i].linkRate*4 - 2;
						else
							dbmxPe[i]		= hAif->linkConfig[i].linkRate*4 - 1;
					} else {
						dbmxPe[i]		= hAif->linkConfig[i].linkRate*4;
					}
				if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)){  
						//dbmxPe = 2;
						if (hAif->linkConfig[i].sampleRate == AIF_SRATE_1P92MHZ)   dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/1920);
						if (hAif->linkConfig[i].sampleRate == AIF_SRATE_7P68MHZ)   dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/7680);
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_15P36MHZ) dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/15360);
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_23P04MHZ) dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/30720); // same as LTE20
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_30P72MHZ) dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/30720);
				}

				if (hAif->mode == AIF_LTE_WCDMA_MODE)
				{
					if (hAif->linkConfig[i].mode == LTE)
					{
						if (hAif->linkConfig[i].sampleRate == AIF_SRATE_1P92MHZ)   dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/1920);
						if (hAif->linkConfig[i].sampleRate == AIF_SRATE_7P68MHZ)   dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/7680);
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_15P36MHZ) dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/15360);
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_23P04MHZ) dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/30720); // same as LTE20
						if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_30P72MHZ) dbmxPe[i]		= _bitc32(hAif->linkConfig[i].maskPattern[0]) * (hAif->linkConfig[i].linkRate*4*256*15/30720);
					} else {
						if ((AIF2FL_DATA_WIDTH_16_BIT==hAif->linkConfig[i].outboundDataWidth)||
							(AIF2FL_DATA_WIDTH_8_BIT==hAif->linkConfig[i].outboundDataWidth))
						{
							if (hAif->linkConfig[i].linkRate == 8)
								dbmxPe[i]		= hAif->linkConfig[i].linkRate*4 - 2;
							else
								dbmxPe[i]		= hAif->linkConfig[i].linkRate*4 - 1;
						} else {
							dbmxPe[i]		= hAif->linkConfig[i].linkRate*4;
						}
					}
				}
				PeLinkSetup[i].PeDelay = AIF2_DB_PE_DELAY_CPRI;
				PeLinkSetup[i].PeCpriDualBitMap.DbmX = dbmxPe[i] - 1;//15 for 4x link speed with 8bit data. set X-1
				if (hAif->mode == AIF_GENERICPACKET_MODE) PeLinkSetup[i].PeCpriDualBitMap.DbmXBubble = 0;
				else PeLinkSetup[i].PeCpriDualBitMap.DbmXBubble = 1;//2 bubbles of 1 AxC sample
				PeLinkSetup[i].PeCpriDualBitMap.Dbm1Mult = 0;//set n-1
				PeLinkSetup[i].PeCpriDualBitMap.Dbm1Size = 0;//set n-1
				PeLinkSetup[i].PeCpriDualBitMap.Dbm1Map[0] = 0x0;
				PeLinkSetup[i].PeCpriDualBitMap.Dbm2Size = 0;
				PeLinkSetup[i].PeCpriDualBitMap.Dbm2Map[0] = 0x0;

				if (hAif->linkConfig[i].sampleRate  == AIF_SRATE_23P04MHZ) {
					if (hAif->linkConfig[i].linkRate == 4)
					{
						PeLinkSetup[i].PeCpriDualBitMap.DbmX = 1; // forcing format
						PeLinkSetup[i].PeCpriDualBitMap.DbmXBubble = 1;//bubbles of 2 AxC samples
						PeLinkSetup[i].PeCpriDualBitMap.Dbm1Mult = 0;//set n-1
						PeLinkSetup[i].PeCpriDualBitMap.Dbm1Size = 2;//set n-1
						PeLinkSetup[i].PeCpriDualBitMap.Dbm1Map[0] = 0x4;
						PeLinkSetup[i].PeCpriDualBitMap.Dbm2Size = 0;
						PeLinkSetup[i].PeCpriDualBitMap.Dbm2Map[0] = 0x0;
					} else if (hAif->linkConfig[i].linkRate == 8)
					{
						PeLinkSetup[i].PeCpriDualBitMap.DbmX = 4; // forcing format
						PeLinkSetup[i].PeCpriDualBitMap.DbmXBubble = 0;//bubbles of 1 AxC sample
						PeLinkSetup[i].PeCpriDualBitMap.Dbm1Mult = 0;//set n-1
						PeLinkSetup[i].PeCpriDualBitMap.Dbm1Size = 2;//set n-1
						PeLinkSetup[i].PeCpriDualBitMap.Dbm1Map[0] = 0x2;
						PeLinkSetup[i].PeCpriDualBitMap.Dbm2Size = 0;
						PeLinkSetup[i].PeCpriDualBitMap.Dbm2Map[0] = 0x0;
					}
				}
				if (hAif->linkConfig[i].outboundDataType==AIF2FL_LINK_DATA_TYPE_RSA)
				{
					if (hAif->linkConfig[i].outboundDataWidth == AIF2FL_DATA_WIDTH_7_BIT) PeLinkSetup[i].CpriAxCPack = AIF2FL_CPRI_7BIT_SAMPLE;
					else PeLinkSetup[i].CpriAxCPack = AIF2FL_CPRI_8BIT_SAMPLE;
				} else {
					if (hAif->linkConfig[i].outboundDataWidth == AIF2FL_DATA_WIDTH_15_BIT) PeLinkSetup[i].CpriAxCPack = AIF2FL_CPRI_15BIT_SAMPLE;
					else PeLinkSetup[i].CpriAxCPack = AIF2FL_CPRI_16BIT_SAMPLE;
				}
				PeLinkSetup[i].CpriCwNullDelimitor = 0xFB;//K 27.7 charactor
				if (hAif->mode == AIF_GENERICPACKET_MODE){
					PeLinkSetup[i].bEnablePack[hAif->linkConfig[i].firstPeDBCH] = true;//trial at false
					PeLinkSetup[i].PePackDmaCh[hAif->linkConfig[i].firstPeDBCH] = hAif->linkConfig[i].firstPeDBCH;
					PeLinkSetup[i].CpriCwPktDelimitor[hAif->linkConfig[i].firstPeDBCH] = AIF2FL_CW_DELIM_4B5B;
				} else {
					for (j=0;j<AIF2_CPRI_MAX_CW_SUBSTREAM;j++)
					{
						if (hAif->linkConfig[i].psMsgEnable) PeLinkSetup[i].bEnablePack[j] = true;
						else								 PeLinkSetup[i].bEnablePack[j] = false;
						if (j == 0)		PeLinkSetup[i].CpriCwPktDelimitor[j] = AIF2FL_CW_DELIM_4B5B;
						else			PeLinkSetup[i].CpriCwPktDelimitor[j] = AIF2FL_CW_DELIM_NULLDELM;
						PeLinkSetup[i].PePackDmaCh[j] = 124+j;
					}
					if ((hAif->mode != AIF_GENERICPACKET_MODE) && ( 8!= hAif->linkConfig[i].linkRate)){
						for (j = 0; j < 256; j++)//cpri cw lut setup
						{
							if (((j >= 20) && (j < 64)) || ((j >= 84) && (j < 128)) || ((j>= 148) &&\
									(j < 192)) || ((j >= 212) && (j < 256))) {
								PeLinkSetup[i].CpriCwChannel[j] = 0; //set CW sub channel num to pack 0
								PeLinkSetup[i].bEnableCpriCw[j] = true; //enable CW sub channel for FastC&M
							}
						}
					}
				}
			} else {
				PeLinkSetup[i].bEnObsaiBubbleBW = false;
				PeLinkSetup[i].PeDelay = AIF2_DB_PE_DELAY_OBSAI;//28 sys_clks delay between DB and PE
			}
		}
	}


    // Based on computations from AIF_calcParam
    for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
		   AtLinkSetup[i].PE1Offset = hAif->linkConfig[i].pe2Offset - 10;
		   AtLinkSetup[i].PE2Offset = hAif->linkConfig[i].pe2Offset;
		   AtLinkSetup[i].DeltaOffset = hAif->linkConfig[i].deltaOffset;
		   AtLinkSetup[i].PiMin = hAif->linkConfig[i].piMin;
		   AtLinkSetup[i].PiMax = hAif->linkConfig[i].piMin + 100;
		   AtLinkSetup[i].IsNegativeDelta = false;//positive delta
    }
#ifndef K2
	// Identify which SerDes block to use
	for(i=AIF2FL_LINK_0; i<AIF2FL_LINK_4; i++)
	{
		// moved to link setup
		if(1==hAif->linkConfig[i].linkEnable) serdes_blockb8_used=1;
	}

	for(i=AIF2FL_LINK_4; i<(AIF2FL_LINK_5 + 1); i++)
	{
		// moved to link setup
		if(1==hAif->linkConfig[i].linkEnable) serdes_blockb4_used=1;
	}

	//SD common setup
	if ( serdes_blockb8_used ==1){
		SdCommonSetup.bEnablePllB8          = true;
		SdCommonSetup.CLKBYP_B8             = AIF2FL_PLL_CLOCK_NO_BYPASS;
		SdCommonSetup.LB_B8                 = AIF2FL_PLL_LOOP_BAND_MID;//High BW is also fine
		SdCommonSetup.VoltRangeB8           = AIF2FL_PLL_VOLTAGE_LOW;//fixed factor
		SdCommonSetup.SleepPllB8            = AIF2FL_PLL_AWAKE;
		if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) {
			//20x for obsai with 153.6Mhz sys_clock, 25Xfor OBSAI when reference clock is 122.88 Mhz
			if (153600==hAif->aif2ClkSpeedKhz)
				SdCommonSetup.pllMpyFactorB8 = AIF2FL_PLL_MUL_FACTOR_20X;
			else // 128.88Mhz
				SdCommonSetup.pllMpyFactorB8 = AIF2FL_PLL_MUL_FACTOR_25X;
		} else {
			//16x for cpri with 153.6Mhz sys_clock, 20X for CPRI when reference clock is 122.88 Mhz
			if (153600==hAif->aif2ClkSpeedKhz)
				SdCommonSetup.pllMpyFactorB8 = AIF2FL_PLL_MUL_FACTOR_16X;
			else // 128.88Mhz
				SdCommonSetup.pllMpyFactorB8 = AIF2FL_PLL_MUL_FACTOR_20X;
		}
		SdCommonSetup.SysClockSelect         = AIF2FL_SD_BYTECLOCK_FROM_B8;
	}
	if ( serdes_blockb4_used ==1){
		SdCommonSetup.bEnablePllB4           = true;
		SdCommonSetup.CLKBYP_B4              = AIF2FL_PLL_CLOCK_NO_BYPASS;
		SdCommonSetup.LB_B4                  = AIF2FL_PLL_LOOP_BAND_MID;//High BW is also fine
		SdCommonSetup.VoltRangeB4            = AIF2FL_PLL_VOLTAGE_LOW;//fixed factor
		SdCommonSetup.SleepPllB4             = AIF2FL_PLL_AWAKE;
		if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) {
			//20x for obsai with 153.6Mhz sys_clock, 25Xfor OBSAI when reference clock is 122.88 Mhz
			if (153600==hAif->aif2ClkSpeedKhz)
				SdCommonSetup.pllMpyFactorB4 = AIF2FL_PLL_MUL_FACTOR_20X;
			else // 128.88Mhz
				SdCommonSetup.pllMpyFactorB4 = AIF2FL_PLL_MUL_FACTOR_25X;
		} else {
			//16x for cpri with 153.6Mhz sys_clock, 20X for CPRI when reference clock is 122.88 Mhz
			if (153600==hAif->aif2ClkSpeedKhz)
				SdCommonSetup.pllMpyFactorB4 = AIF2FL_PLL_MUL_FACTOR_16X;
			else // 128.88Mhz
				SdCommonSetup.pllMpyFactorB4 = AIF2FL_PLL_MUL_FACTOR_20X;
		}
		if ( serdes_blockb8_used ==0)
			SdCommonSetup.SysClockSelect     = AIF2FL_SD_BYTECLOCK_FROM_B4;
	}

	for(i=0, j=0; i< AIF_MAX_NUM_LINKS; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
			  SdCommonSetup.DisableLinkClock[i] = false;//enable link clock
		else
			  SdCommonSetup.DisableLinkClock[i] = true; //disable link clock
	}
#else
	   /*SERDES CSL now out of AIF2 - dedicated functions in AIF2 LLD were created to handle K-II SerDes settings*/
#endif

	//PD common setup
	PdCommonSetup.PdCppiDioSel       	 = hAif->pktdmaOrDioEngine;
	if(AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol) {
		PdCommonSetup.AxCOffsetWin       = AIF2_CPRI_AXC_OFFSET_WIN;
		// Radio frame size for CPRI
		PdCommonSetup.PdRadtTC           = AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER;
	} else {
		PdCommonSetup.AxCOffsetWin       = AIF2_OBSAI_AXC_OFFSET_WIN;
		// Radio frame size for OBSAI
		PdCommonSetup.PdRadtTC           = AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;
	}
	if (hAif->mode == AIF_WCDMA_MODE) {
		PdCommonSetup.PdFrameTC[0].FrameIndexSc  = 0;//start index
		PdCommonSetup.PdFrameTC[0].FrameIndexTc  = 0;//terminal index
		PdCommonSetup.PdFrameTC[0].FrameSymbolTc = 15-1;//15 slots for WCDMA
		PdCommonSetup.PdFrameMsgTc[0]            = 639;
		PdCommonSetup.PdFrameTC[1].FrameIndexSc  = 1;//start index
		PdCommonSetup.PdFrameTC[1].FrameIndexTc  = 1;//terminal index
		PdCommonSetup.PdFrameTC[1].FrameSymbolTc = 15-1;//15 slots for WCDMA
		PdCommonSetup.PdFrameMsgTc[1]            = 639;
	}

	if (hAif->mode == AIF_GENERICPACKET_MODE) {
		// TO BE FIXED
	}

	z = first_link; //track to which link this channel corresponds
	frameIndexSc = 0;
	initIngrGroupInfo(&ingrGroupInfo[0], AIF_NUM_PD_PE_FRAME_TC_GROUPS);
	for(i=0;i<max_channel_pd;i++)//for channel 0,...to max channels used by all links
	{
		if (i >= (hAif->linkConfig[z].firstPdDBCH + hAif->linkConfig[z].numPdAxC))
		{
			do{
				z++;
			}while((0 == hAif->linkConfig[z].linkEnable));
		}

		if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)) {
			if (hAif->linkConfig[z].multiRate == 0)
				sampleRate = hAif->linkConfig[z].sampleRate;
			else
				sampleRate = hAif->AxCconfig[i].sampleRate;
			linkRate = hAif->linkConfig[z].linkRate;
			lteCpType = hAif->AxCconfig[i].lteCpType;
			if ((groupId = addAxCtoIngrGroupTable(&ingrGroupInfo[0], AIF_NUM_PD_PE_FRAME_TC_GROUPS, i, sampleRate, lteCpType, &isNewGroup))
			     == AIF_NUM_PD_PE_FRAME_TC_GROUPS) {
				Aif2_osalLog("Fatal Error: addAxCtoIngrGroupTable returned bad ID\n");
			}
			
			if (isNewGroup) {
				if (hAif->mode == AIF_LTE_WCDMA_MODE) {
					PdCommonSetup.PdFrameTC[groupId].FrameIndexSc  = frameIndexSc;//start index
					PdCommonSetup.PdFrameTC[groupId].FrameIndexTc  = frameIndexSc;//terminal index
					if (sampleRate == AIF_SRATE_3P84MHZ)
						PdCommonSetup.PdFrameTC[groupId].FrameSymbolTc = 15-1;//140 symbols for LTE
					else
						PdCommonSetup.PdFrameTC[groupId].FrameSymbolTc = 120-1;//140 symbols for LTE
					frameIndexSc += 1;
				} else if (sampleRate != AIF_SRATE_1P92MHZ) {
					if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
						PdCommonSetup.PdFrameTC[groupId].FrameIndexSc  = frameIndexSc;//start index
						PdCommonSetup.PdFrameTC[groupId].FrameIndexTc  = frameIndexSc + (AIF2_LTE_SYMBOL_NUM-1);//terminal index
						PdCommonSetup.PdFrameTC[groupId].FrameSymbolTc = AIF2_LTE_FRAME_SYMBOL_NUM-1;//140 symbols for LTE
						frameIndexSc += AIF2_LTE_SYMBOL_NUM;
					}
					else { // Extended CP
						PdCommonSetup.PdFrameTC[groupId].FrameIndexSc  = frameIndexSc;//start index
						PdCommonSetup.PdFrameTC[groupId].FrameIndexTc  = frameIndexSc + (AIF2_LTE_SYMBOL_NUM_EXT_CP-1);//terminal index
						PdCommonSetup.PdFrameTC[groupId].FrameSymbolTc = AIF2_LTE_FRAME_SYMBOL_NUM_EXT_CP-1;//120 symbols for LTE
						frameIndexSc += AIF2_LTE_SYMBOL_NUM_EXT_CP;
					}
				} else { //AIF_SRATE_1P92MHZ
					PdCommonSetup.PdFrameTC[groupId].FrameIndexSc  = frameIndexSc;//start index
					PdCommonSetup.PdFrameTC[groupId].FrameIndexTc  = frameIndexSc;//terminal index
					PdCommonSetup.PdFrameTC[groupId].FrameSymbolTc = 20-1;//20 slots for LTE
					frameIndexSc += 1;
				}
				if (sampleRate == AIF_SRATE_1P92MHZ) {
					FrameMsg1 	= AIF2_LTE1P4_FFT_SIZE + AIF2_LTE1P4_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE1P4_FFT_SIZE + AIF2_LTE1P4_CYPRENORMAL_SIZE;
					FrameMsg2 	= AIF2_LTE1P4_FFT_SIZE + AIF2_LTE1P4_CYPREEXTENDED_SIZE;
					totalAxC    = linkRate*4*256*15/7680;
					lte1_4 = 1;
				}
				if (sampleRate == AIF_SRATE_3P84MHZ) {
					FrameMsg1 	= 0;
					FrameMsg	= 0;
					FrameMsg2	= 2560;
					totalAxC    = 0;
				}
				if (sampleRate == AIF_SRATE_7P68MHZ) {
					FrameMsg1 	= AIF2_LTE5_FFT_SIZE + AIF2_LTE5_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE5_FFT_SIZE + AIF2_LTE5_CYPRENORMAL_SIZE;
					FrameMsg2	= AIF2_LTE5_FFT_SIZE + AIF2_LTE5_CYPREEXTENDED_SIZE;
					totalAxC    = linkRate*4*256*15/7680;
				}
				if (sampleRate == AIF_SRATE_15P36MHZ) {
					FrameMsg1 	= AIF2_LTE10_FFT_SIZE + AIF2_LTE10_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE10_FFT_SIZE + AIF2_LTE10_CYPRENORMAL_SIZE;
					FrameMsg2	= AIF2_LTE10_FFT_SIZE + AIF2_LTE10_CYPREEXTENDED_SIZE;
					totalAxC    = linkRate*4*256*15/15360;
				}
				if (sampleRate == AIF_SRATE_23P04MHZ) {
					FrameMsg1 	= AIF2_LTE15_FFT_SIZE + AIF2_LTE15_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE15_FFT_SIZE + AIF2_LTE15_CYPRENORMAL_SIZE;
					FrameMsg2	= AIF2_LTE15_FFT_SIZE + AIF2_LTE15_CYPREEXTENDED_SIZE;
					totalAxC    = linkRate*4*256*15/30720; // same as LTE20
				}
				if (sampleRate == AIF_SRATE_30P72MHZ) {
					FrameMsg1 	= AIF2_LTE20_FFT_SIZE + AIF2_LTE20_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE20_FFT_SIZE + AIF2_LTE20_CYPRENORMAL_SIZE;
					FrameMsg2	= AIF2_LTE20_FFT_SIZE + AIF2_LTE20_CYPREEXTENDED_SIZE;
					totalAxC    = linkRate*4*256*15/30720;
				}

				if (superPacket == 0){
					if (hAif->mode == AIF_LTE_WCDMA_MODE)
					{
						PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc] = (FrameMsg2/ AIF2_NUM_WORDS_PER_QWORD) - 1;
					} else if (lte1_4 == 1) {
						if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
							PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc] = ((FrameMsg1+6*FrameMsg)/ AIF2_NUM_WORDS_PER_QWORD) - 1;
						}
						else {
							PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc] = ((6*FrameMsg2)/ AIF2_NUM_WORDS_PER_QWORD) - 1;
						}
					} else {
						if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
							//frame message terminal count for first normal cyclic prefix symbol
							PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc] = (FrameMsg1/ AIF2_NUM_WORDS_PER_QWORD) - 1;
							//frame message terminal count for other 6 normal cyclic prefix LTE symbols
							for(j=1;j<(AIF2_LTE_SYMBOL_NUM);j++){
								PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc + j] = (FrameMsg /AIF2_NUM_WORDS_PER_QWORD) - 1;
							}
						}
						else { //extended
							//frame message terminal count for all 6 extended cyclic prefix LTE symbols
							for(j=0 ;j<(AIF2_LTE_SYMBOL_NUM_EXT_CP);j++){
								PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc+ j] = (FrameMsg2 /AIF2_NUM_WORDS_PER_QWORD) - 1;
							}
						}
					}
				}else{
					if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
						//frame message terminal count for first normal cyclic prefix symbol
						PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc] = (totalAxC)*(FrameMsg1 / AIF2_NUM_WORDS_PER_QWORD) - 1;
						//frame message terminal count for other 6 normal cyclic prefix LTE symbols
						for(j=1;j<(AIF2_LTE_SYMBOL_NUM);j++){
							PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc + j] = (totalAxC)*(FrameMsg /AIF2_NUM_WORDS_PER_QWORD) - 1;
						}
					}
					else {
						//frame message terminal count for all 6 extended cyclic prefix LTE symbols
						for(j=0;j<(AIF2_LTE_SYMBOL_NUM_EXT_CP);j++){
							PdCommonSetup.PdFrameMsgTc[PdCommonSetup.PdFrameTC[groupId].FrameIndexSc + j] = (totalAxC)*(FrameMsg2 /AIF2_NUM_WORDS_PER_QWORD) - 1;
						}
					}
				}
			}
		}

		if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) {

			if (hAif->mode == AIF_LTE_WCDMA_MODE)
			{
				if (hAif->linkConfig[z].mode == LTE)
				{
					obsai_type = AIF2_OBSAI_TYPE_LTE;
				} else {
					obsai_type = AIF2_OBSAI_TYPE_WCDMA_FDD;
				}
			}

			PdCommonSetup.PdRoute[i].RouteTs = 0;//Route OBSAI time stamp
			PdCommonSetup.PdRoute[i].RouteType = obsai_type;//Route OBSAI type
			PdCommonSetup.PdRoute[i].RouteAddr = i;//Route OBSAI address

			if (hAif->mode == AIF_GENERICPACKET_MODE){
				PdCommonSetup.PdRoute[i].RouteLink = (Aif2Fl_LinkIndex) (AIF2FL_LINK_0 + z);//Route link
				PdCommonSetup.PdRoute[i].RouteMask = AIF2FL_ROUTE_MASK_4LSB;
				//PdCommonSetup.PdRoute[i].RouteTs = 0x10;
			} else {
				PdCommonSetup.PdRoute[i].RouteMask = AIF2FL_ROUTE_MASK_NONE;//Route TS mask
				PdCommonSetup.AxCOffset[i] = hAif->AxCconfig[i].pdAxCOffset;
				PdCommonSetup.AxCOffsetWin = AIF2_OBSAI_AXC_OFFSET_WIN;  //AxC offset window
				if (hAif->mode == AIF_LTE_WCDMA_MODE) // dual mode
				{
					if (hAif->linkConfig[z].mode == LTE)
						PdCommonSetup.PdChConfig1[i].FrameCounter = 0;//framing counter group number
					else
						PdCommonSetup.PdChConfig1[i].FrameCounter = 1;//framing counter group number
				} else {
					if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)){
						PdCommonSetup.PdChConfig1[i].FrameCounter = groupId;
					} else {
						PdCommonSetup.PdChConfig1[i].FrameCounter = 0;//framing counter group number
					}
				}
				PdCommonSetup.PdRoute[i].RouteLink = z;//Route link
			}
		} else { // CPRI
			PdCommonSetup.AxCOffset[i] = hAif->AxCconfig[i].pdAxCOffset;
			if ((8==hAif->linkConfig[z].linkRate) && (hAif->mode == AIF_WCDMA_MODE)) {
				PdCommonSetup.PdChConfig1[i].FrameCounter = 1;//framing counter group number
			} else if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)) {
				PdCommonSetup.PdChConfig1[i].FrameCounter = groupId;
			} else {
				PdCommonSetup.PdChConfig1[i].FrameCounter = 0;//framing counter group number
			}
		}
		PdCommonSetup.PdChConfig[i].bChannelEn = true;//Channel enable
		PdCommonSetup.PdChConfig[i].DataFormat = hAif->linkConfig[z].inboundDataType; //Data format
		PdCommonSetup.PdChConfig1[i].bTsWatchDogEn = false;//disable watchdog
		PdCommonSetup.PdChConfig1[i].DataFormat = AIF2FL_GSM_DATA_OTHER;//Non GSM data
		PdCommonSetup.PdChConfig1[i].DioOffset = 0;//Use zero offset for simple test
		if (hAif->mode == AIF_LTE_TDD_MODE)
		{
			tddSubFrameBitMap = hAif->linkConfig[z].lteTddUlDlCfg[i];
			for (k=0;k<10;k++)
			{
				// DL SF
				if ((tddSubFrameBitMap & 0x00000003) == 0x00000000) {
					for (j=0;j<(AIF2_LTE_SYMBOL_NUM*2);j++)
					{
						tddTable[j+AIF2_LTE_SYMBOL_NUM*2*k] = 0;
					}
				}
				// UL SF
				if ((tddSubFrameBitMap & 0x00000003) == 0x00000003) {
					for (j=0;j<(AIF2_LTE_SYMBOL_NUM*2);j++)
					{
						tddTable[j+AIF2_LTE_SYMBOL_NUM*2*k] = 1;
					}
				}
				// Special SF
				if ((tddSubFrameBitMap & 0x00000003) == 0x00000001) {
					tddSpecSubFrameBitMap = hAif->linkConfig[z].lteTddSsfNcpCfg[i];
					for (j=0;j<(AIF2_LTE_SYMBOL_NUM*2);j++)
					{
						if ((tddSpecSubFrameBitMap & 0x00000003) == 0x00000003) {
							tddTable[j+AIF2_LTE_SYMBOL_NUM*2*k] = 1;
						} else {
							tddTable[j+AIF2_LTE_SYMBOL_NUM*2*k] = 0;
						}
						tddSpecSubFrameBitMap>>=2;
					}
				}
				tddSubFrameBitMap>>=2;
			}

			offsetTdd = 0;
			PdCommonSetup.PdChConfig1[i].TddEnable = 0;
			PdCommonSetup.TddEnable1[i] = 0;
			PdCommonSetup.TddEnable2[i] = 0;
			PdCommonSetup.TddEnable3[i] = 0;
			PdCommonSetup.TddEnable4[i] = 0;
			for (j=0;j<16;j++)
			{
				PdCommonSetup.PdChConfig1[i].TddEnable += tddTable[j+offsetTdd]<<j;
			}
			offsetTdd += j;
			for (j=0;j<32;j++)
			{
				PdCommonSetup.TddEnable1[i] += tddTable[j+offsetTdd]<<j;	//enables all symbols(FDD)
			}
			offsetTdd += j;
			for (j=0;j<32;j++)
			{
				PdCommonSetup.TddEnable2[i] += tddTable[j+offsetTdd]<<j;	//enables all symbols(FDD)
			}
			offsetTdd += j;
			for (j=0;j<32;j++)
			{
				PdCommonSetup.TddEnable3[i] += tddTable[j+offsetTdd]<<j;	//enables all symbols(FDD)
			}
			offsetTdd += j;
			for (j=0;j<32;j++)
			{
				PdCommonSetup.TddEnable4[i] += tddTable[j+offsetTdd]<<j;	//enables all symbols(FDD)
			}

		} else {
			PdCommonSetup.PdChConfig1[i].TddEnable = 0xFFFF;//enables all symbols(FDD)
			PdCommonSetup.TddEnable1[i] = 0xFFFFFFFF;//enables all symbols(FDD)
			PdCommonSetup.TddEnable2[i] = 0xFFFFFFFF;//enables all symbols(FDD)
			PdCommonSetup.TddEnable3[i] = 0xFFFFFFFF;//enables all symbols(FDD)
			PdCommonSetup.TddEnable4[i] = 0xFFFFFFFF;//enables all symbols(FDD)
		}
	}

	// channel for ps
	for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
		if ((hAif->linkConfig[i].psMsgEnable) && (hAif->linkConfig[i].linkEnable)) {
			for (j=0;j<AIF2_CPRI_MAX_CW_SUBSTREAM;j++)
			{
				// map control stream 0 of each link to channel 124
				PdCommonSetup.PdChConfig[124+j].bChannelEn = true;
				PdCommonSetup.PdChConfig[124+j].DataFormat = AIF2FL_LINK_DATA_TYPE_NORMAL;
			}
		}
	}

	//PE common setup
	if (hAif->mode == AIF_GENERICPACKET_MODE){
		if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
			PeCommonSetup.PeTokenPhase = 1;//Phase alignment for scheduling DMA OBSAI: only lsb is used
		else
			PeCommonSetup.PeTokenPhase = 0;//Phase alignment for scheduling DMA OBSAI: only lsb is used
		PeCommonSetup.EnetHeaderSelect = 0;//bit order for Ethernet preamble and SOF
	} else {
		if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
			PeCommonSetup.PeTokenPhase = 1;//Phase alignment for scheduling DMA OBSAI: only lsb is used
		else
			PeCommonSetup.PeTokenPhase = 0;//Phase alignment for scheduling DMA OBSAI: only lsb is used

		PeCommonSetup.EnetHeaderSelect = 0;//bit order for Ethernet preamble and SOF
	}
	if (hAif->mode == AIF_WCDMA_MODE) {
		PeCommonSetup.GlobalDioLen = AIF2FL_DB_DIO_LEN_128;
	}
	if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)){
		PeCommonSetup.EnetHeaderSelect = 1;//bit order for Ethernet preamble and SOF
	}
	if (hAif->mode == AIF_LTE_WCDMA_MODE){
		PeCommonSetup.EnetHeaderSelect = 1;//bit order for Ethernet preamble and SOF
		PeCommonSetup.GlobalDioLen = AIF2FL_DB_DIO_LEN_256;
	}

    if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)) {
		//Doubling of entries for normal CP to go to sub-frame for supporting MBFSN.
		#define MBSFN_FACTOR 2
		frameIndexSc = 0;
		initEgrGroupInfo();
	} else {	
		for (i = 0; i < 6; i++) {
			PeCommonSetup.PeFrameTC[i].FrameIndexSc  = PdCommonSetup.PdFrameTC[i].FrameIndexSc;
			PeCommonSetup.PeFrameTC[i].FrameIndexTc  = PdCommonSetup.PdFrameTC[i].FrameIndexTc;
			PeCommonSetup.PeFrameTC[i].FrameSymbolTc = PdCommonSetup.PdFrameTC[i].FrameSymbolTc;
		}
	}	

	z = first_link; //track to which link this channel corresponds
	k = 0;
	for (i = 0; i < max_channel_pe; i++)//for channel 0,...to max channels used by all links
	{
		if (i >= (hAif->linkConfig[z].firstPeDBCH
				+ hAif->linkConfig[z].numPeAxC)) {
			do {
				z++;
			} while ((0 == hAif->linkConfig[z].linkEnable));
			k = 0;
		}
		if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)) {
			if (hAif->linkConfig[z].multiRate == 0)
				sampleRate = hAif->linkConfig[z].sampleRate;
			else
				sampleRate = hAif->AxCconfig[i].sampleRate;
			linkRate = hAif->linkConfig[z].linkRate;
			lteCpType = hAif->AxCconfig[i].lteCpType;
			cellId = hAif->AxCconfig[i].egressLTEcellId;

			if ((groupId = addAxCtoEgrGroupTable(i, sampleRate, lteCpType, cellId, &isNewGroup))
			     == AIF_NUM_PD_PE_FRAME_TC_GROUPS) {
				Aif2_osalLog("Fatal Error: addAxCtoEgrGroupTable returned bad ID\n");
			}

			if (isNewGroup) {
				if (hAif->mode == AIF_LTE_WCDMA_MODE)
				{
					PeCommonSetup.PeFrameTC[groupId].FrameIndexSc  = frameIndexSc;//start index
					PeCommonSetup.PeFrameTC[groupId].FrameIndexTc  = frameIndexSc;//terminal index
					if (sampleRate == AIF_SRATE_3P84MHZ)
						PeCommonSetup.PeFrameTC[groupId].FrameSymbolTc = 15-1;//20 slots for LTE
					else
						PeCommonSetup.PeFrameTC[groupId].FrameSymbolTc = 120-1;//20 slots for LTE
					frameIndexSc += 1;
				} else if (sampleRate != AIF_SRATE_1P92MHZ) {
					if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
						PeCommonSetup.PeFrameTC[groupId].FrameIndexSc  = frameIndexSc;//start index
						PeCommonSetup.PeFrameTC[groupId].FrameIndexTc  = frameIndexSc + ((MBSFN_FACTOR * AIF2_LTE_SYMBOL_NUM) - 1);//terminal index
						PeCommonSetup.PeFrameTC[groupId].FrameSymbolTc = AIF2_LTE_FRAME_SYMBOL_NUM-1;//140 symbols for LTE
						frameIndexSc += (MBSFN_FACTOR * AIF2_LTE_SYMBOL_NUM);
					}
					else { // Extended CP
						PeCommonSetup.PeFrameTC[groupId].FrameIndexSc  = frameIndexSc;//start index
						PeCommonSetup.PeFrameTC[groupId].FrameIndexTc  = frameIndexSc + (AIF2_LTE_SYMBOL_NUM_EXT_CP-1);//terminal index
						PeCommonSetup.PeFrameTC[groupId].FrameSymbolTc = AIF2_LTE_FRAME_SYMBOL_NUM_EXT_CP-1;//120 symbols for LTE
						frameIndexSc += AIF2_LTE_SYMBOL_NUM_EXT_CP;
					}
				} else { //AIF_SRATE_1P92MHZ
					PeCommonSetup.PeFrameTC[groupId].FrameIndexSc  = frameIndexSc;//start index
					PeCommonSetup.PeFrameTC[groupId].FrameIndexTc  = frameIndexSc;//terminal index
					PeCommonSetup.PeFrameTC[groupId].FrameSymbolTc = 20-1;//20 slots for LTE
					frameIndexSc += 1;
				}

				if (sampleRate == AIF_SRATE_1P92MHZ) {
					FrameMsg1 	= AIF2_LTE1P4_FFT_SIZE + AIF2_LTE1P4_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE1P4_FFT_SIZE + AIF2_LTE1P4_CYPRENORMAL_SIZE;
					FrameMsg2 	= AIF2_LTE1P4_FFT_SIZE + AIF2_LTE1P4_CYPREEXTENDED_SIZE;
				}
				if (sampleRate == AIF_SRATE_3P84MHZ) {
					FrameMsg1 	= 0;
					FrameMsg	= 0;
					FrameMsg2	= 2560;
					totalAxC    = 0;
				}
				if (sampleRate == AIF_SRATE_7P68MHZ) {
					FrameMsg1 	= AIF2_LTE5_FFT_SIZE + AIF2_LTE5_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE5_FFT_SIZE + AIF2_LTE5_CYPRENORMAL_SIZE;
					FrameMsg2	= AIF2_LTE5_FFT_SIZE + AIF2_LTE5_CYPREEXTENDED_SIZE;
				}
				if (sampleRate == AIF_SRATE_15P36MHZ) {
					FrameMsg1 	= AIF2_LTE10_FFT_SIZE + AIF2_LTE10_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE10_FFT_SIZE + AIF2_LTE10_CYPRENORMAL_SIZE;
					FrameMsg2	= AIF2_LTE10_FFT_SIZE + AIF2_LTE10_CYPREEXTENDED_SIZE;
				}
				if (sampleRate == AIF_SRATE_23P04MHZ) {
					FrameMsg1 	= AIF2_LTE15_FFT_SIZE + AIF2_LTE15_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE15_FFT_SIZE + AIF2_LTE15_CYPRENORMAL_SIZE;
					FrameMsg2	= AIF2_LTE15_FFT_SIZE + AIF2_LTE15_CYPREEXTENDED_SIZE;
				}
				if (sampleRate == AIF_SRATE_30P72MHZ) {
					FrameMsg1 	= AIF2_LTE20_FFT_SIZE + AIF2_LTE20_CYPRENORMAL1_SIZE;
					FrameMsg	= AIF2_LTE20_FFT_SIZE + AIF2_LTE20_CYPRENORMAL_SIZE;
					FrameMsg2	= AIF2_LTE20_FFT_SIZE + AIF2_LTE20_CYPREEXTENDED_SIZE;
				}

				if (AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol){
					if (hAif->mode == AIF_LTE_WCDMA_MODE)
					{
						PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc] = FrameMsg2 - 1;
					} else if (sampleRate == AIF_SRATE_1P92MHZ) {
						if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
							PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc] = (FrameMsg1+6*FrameMsg) - 1;
						}
						else {
							PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc] = (6*FrameMsg2) - 1;
						}
					} else {
						if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
							PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc] = FrameMsg1 - 1;
							//frame message terminal count for other 6 normal cyclic prefix LTE symbols
							for(j=1;j<(AIF2_LTE_SYMBOL_NUM);j++){
								PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc + j] = FrameMsg - 1;
							}
							//Copy 1st slot into 2nd slot for MBSFN
							for(j = AIF2_LTE_SYMBOL_NUM; j < (MBSFN_FACTOR * AIF2_LTE_SYMBOL_NUM); j++) {
								PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc + j] =
								PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc + (j - AIF2_LTE_SYMBOL_NUM)];
							}
						}
						else {
							//frame message terminal count for all 6 extended cyclic prefix LTE symbols
							for(j=0;j<(AIF2_LTE_SYMBOL_NUM_EXT_CP);j++){
								PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc + j] = FrameMsg2 - 1;
							}
						}
					}
				} else {
					if (sampleRate == AIF_SRATE_1P92MHZ)
					{
						if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
							PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc] = ((FrameMsg1+6*FrameMsg)/AIF2_NUM_WORDS_PER_QWORD) - 1;
						}
						else {
							PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc] = (6*FrameMsg2) - 1;
						}
					} else {
						if (lteCpType == AIF2_LTE_CPTYPE_NORMAL) {
							PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc] = (FrameMsg1/AIF2_NUM_WORDS_PER_QWORD) - 1;
							//frame message terminal count for other 6 normal cyclic prefix LTE symbols
							for(j=1;j<(AIF2_LTE_SYMBOL_NUM);j++){
								PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc + j] = (FrameMsg/AIF2_NUM_WORDS_PER_QWORD) - 1;
							}
							//Copy 1st slot into 2nd slot for MBSFN
							for(j = AIF2_LTE_SYMBOL_NUM; j < (MBSFN_FACTOR * AIF2_LTE_SYMBOL_NUM); j++) {
								PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc + j] =
								PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc + (j - AIF2_LTE_SYMBOL_NUM)];
							}
						}
						else {
							//frame message terminal count for all 6 extended cyclic prefix LTE symbols
							for(j=0;j<(AIF2_LTE_SYMBOL_NUM_EXT_CP);j++){
								PeCommonSetup.PeFrameMsgTc[PeCommonSetup.PeFrameTC[groupId].FrameIndexSc + j] = (FrameMsg2/AIF2_NUM_WORDS_PER_QWORD) - 1;
							}
						}
					}
				}
			}
		}

		PeCommonSetup.bEnableCh[i]               = true;//Enable PE channel
		PeCommonSetup.PeDmaCh0[i].bCrcEn         = false;//disable CRC
		if (hAif->linkConfig[z].RtEnabled==1)
			PeCommonSetup.PeDmaCh0[i].RtControl      = AIF2FL_PE_RT_RETRANS;//use PE insert option
		else
			PeCommonSetup.PeDmaCh0[i].RtControl      = AIF2FL_PE_RT_INSERT;//use PE insert option
		if (hAif->linkConfig[i].outboundDataType==AIF2FL_LINK_DATA_TYPE_RSA)
		{
			PeCommonSetup.PeDmaCh0[i].CrcType     = AIF2FL_CRC_8BIT;//CRC type
		} else {
			PeCommonSetup.PeDmaCh0[i].CrcType     = AIF2FL_CRC_16BIT;//CRC type
		}
		PeCommonSetup.PeDmaCh0[i].isEthernet      = false;//AxC data
		PeCommonSetup.PeDmaCh0[i].CrcObsaiHeader  = false;
		PeCommonSetup.PeInFifo[i].SyncSymbol      = 0;//Sync symbol offset
		if ((hAif->mode == AIF_WCDMA_MODE) || (hAif->mode == AIF_GENERICPACKET_MODE) || (hAif->linkConfig[z].mode == WCDMA)) {
			PeCommonSetup.PeInFifo[i].MFifoWmark      = 2;//Message FIFO water mark
			PeCommonSetup.PeInFifo[i].MFifoFullLevel  = 3;//Message FIFO full level
		}
		else {
			PeCommonSetup.PeInFifo[i].MFifoWmark      = 3;//Message FIFO water mark
			PeCommonSetup.PeInFifo[i].MFifoFullLevel  = 5;//Message FIFO full level
		}
		if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) {

			if (hAif->mode == AIF_LTE_WCDMA_MODE)
			{
				if (hAif->linkConfig[z].mode == LTE)
				{
					PeCommonSetup.PeDmaCh0[i].FrameTC        = 0;//use framing terminal count 0
					obsai_type = AIF2_OBSAI_TYPE_LTE;
				} else {
					PeCommonSetup.PeDmaCh0[i].FrameTC        = 1;//use framing terminal count 0
					obsai_type = AIF2_OBSAI_TYPE_WCDMA_FDD;
				}
			} else {
				if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)) {
					PeCommonSetup.PeDmaCh0[i].FrameTC = groupId;

				} else {
					PeCommonSetup.PeDmaCh0[i].FrameTC = 0;//framing counter group number
				}
			}
			PeCommonSetup.PeAxcOffset[i]           = hAif->AxCconfig[i].peAxCOffset;
			PeCommonSetup.PeChObsaiType[i]         = obsai_type;//OBSAI header type
			PeCommonSetup.PeChObsaiAddr[i]         = i;//OBSAI header address (use simple rising sequence)
			if (hAif->mode == AIF_GENERICPACKET_MODE){
				PeCommonSetup.PeChObsaiTS[i]           = 0x0;//OBSAI header Time Stamp
				PeCommonSetup.PeChObsaiTsMask[i]       = AIF2FL_ROUTE_MASK_4LSB;
				PeCommonSetup.PeChObsaiTsfomat[i]      = AIF2FL_TSTAMP_FORMAT_GEN_PKT;//OBSAI header noraml TS format
				PeCommonSetup.PeObsaiPkt[i]            = true;//Select OBSAI AxC or packet mode
				PeCommonSetup.ChIndex0[k+(64*z)]       = i;
				PeCommonSetup.bEnableChIndex0[k+(64*z)]= true;
			} else {
				PeCommonSetup.PeChObsaiTsMask[i]       = AIF2FL_ROUTE_MASK_NONE;//not use Ts field
				PeCommonSetup.PeChObsaiTsfomat[i]      = AIF2FL_TSTAMP_FORMAT_NORM_TS;//OBSAI header noraml TS format
				PeCommonSetup.PeObsaiPkt[i]            = false;//Select OBSAI AxC or packet mode
				PeCommonSetup.PeChObsaiTS[i]           = 0;//OBSAI header Time Stamp
				PeCommonSetup.ChIndex0[k+(64*z)]       = i;
				PeCommonSetup.bEnableChIndex0[k+(64*z)]= true;
			}
			PeCommonSetup.PeBbHop[i]               = false;//Take OBSAI address from CPPI PS bits
		} else { // CPRI
			if ((8==hAif->linkConfig[z].linkRate) && (hAif->mode == AIF_WCDMA_MODE)) {
				PeCommonSetup.PeDmaCh0[i].FrameTC        = 1;//use framing terminal count 0
			} else if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)) {
					PeCommonSetup.PeDmaCh0[i].FrameTC = groupId;
			} else {
				PeCommonSetup.PeDmaCh0[i].FrameTC = 0;//framing counter group number
			}
			if ((8==hAif->linkConfig[z].linkRate) && (hAif->linkConfig[z].outboundDataType == AIF2FL_LINK_DATA_TYPE_RSA) && (hAif->linkConfig[z].firstPeDBCH+21<i && i<hAif->linkConfig[z].firstPeDBCH+30))
				PeCommonSetup.PeAxcOffset[i]           = hAif->AxCconfig[i].peAxCOffset + 64;
			else
				PeCommonSetup.PeAxcOffset[i]           = hAif->AxCconfig[i].peAxCOffset;
			if (hAif->mode == AIF_LTE_FDD_MODE || hAif->mode == AIF_LTE_TDD_MODE || (hAif->mode == AIF_LTE_WCDMA_MODE))
			{
				for(j=0;j<dbmxPe[z];j++)
				{
					// ChIndex0 for link0, ChIndex1 for link1            i for AxC (maxAxC) k for AxC of the specific link
					if (z == (Aif2Fl_LinkIndex)0){
						if (hAif->linkConfig[z].multiRate == 0){
							if ((hAif->linkConfig[z].maskPattern[k]>>j)&1)
							{
								PeCommonSetup.ChIndex0[j]          = i;
								PeCommonSetup.bEnableChIndex0[j]   = true;
							}
						} else {
							if ((hAif->AxCconfig[i].maskPattern>>j)&1)
							{
								PeCommonSetup.ChIndex0[j]          = i;
								PeCommonSetup.bEnableChIndex0[j]   = true;
							}
						}
						PeCommonSetup.CpriPktEn0[j]        = false;
					}
					if (z == (Aif2Fl_LinkIndex)1){
						if (hAif->linkConfig[z].multiRate == 0){
							if ((hAif->linkConfig[z].maskPattern[k]>>j)&1)
							{
								PeCommonSetup.ChIndex1[j]          = i;
								PeCommonSetup.bEnableChIndex1[j]   = true;
							}
						} else {
							if ((hAif->AxCconfig[i].maskPattern>>j)&1)
							{
								PeCommonSetup.ChIndex1[j]          = i;
								PeCommonSetup.bEnableChIndex1[j]   = true;
							}
						}
						PeCommonSetup.CpriPktEn1[j]        = false;
					}
					if (z == (Aif2Fl_LinkIndex) 2) {
						if (hAif->linkConfig[z].multiRate == 0){
							if ((hAif->linkConfig[z].maskPattern[k]>>j)&1)
							{
								PeCommonSetup.ChIndex2[j]          = i;
								PeCommonSetup.bEnableChIndex2[j]   = true;
							}
						} else {
							if ((hAif->AxCconfig[i].maskPattern>>j)&1)
							{
								PeCommonSetup.ChIndex2[j]          = i;
								PeCommonSetup.bEnableChIndex2[j]   = true;
							}
						}
						PeCommonSetup.CpriPktEn2[j]        = false;
					}
					if (z == (Aif2Fl_LinkIndex) 3) {
						if (hAif->linkConfig[z].multiRate == 0){
							if ((hAif->linkConfig[z].maskPattern[k]>>j)&1)
							{
								PeCommonSetup.ChIndex3[j]          = i;
								PeCommonSetup.bEnableChIndex3[j]   = true;
							}
						} else {
							if ((hAif->AxCconfig[i].maskPattern>>j)&1)
							{
								PeCommonSetup.ChIndex3[j]          = i;
								PeCommonSetup.bEnableChIndex3[j]   = true;
							}
						}
						PeCommonSetup.CpriPktEn3[j]        = false;
					}
					if (z == (Aif2Fl_LinkIndex) 4) {
						if (hAif->linkConfig[z].multiRate == 0){
							if ((hAif->linkConfig[z].maskPattern[k]>>j)&1)
							{
								PeCommonSetup.ChIndex4[j]          = i;
								PeCommonSetup.bEnableChIndex4[j]   = true;
							}	
						} else {
							if ((hAif->AxCconfig[i].maskPattern>>j)&1)
							{
								PeCommonSetup.ChIndex4[j]          = i;
								PeCommonSetup.bEnableChIndex4[j]   = true;
							}
						}
						PeCommonSetup.CpriPktEn4[j]        = false;
					}
					if (z == (Aif2Fl_LinkIndex) 5) {
						if (hAif->linkConfig[z].multiRate == 0){
							if ((hAif->linkConfig[z].maskPattern[k]>>j)&1)
							{
								PeCommonSetup.ChIndex5[j]          = i;
								PeCommonSetup.bEnableChIndex5[j]   = true;
							}
						} else {
							if ((hAif->AxCconfig[i].maskPattern>>j)&1)
							{
								PeCommonSetup.ChIndex5[j]          = i;
								PeCommonSetup.bEnableChIndex5[j]   = true;
							}
						}
						PeCommonSetup.CpriPktEn5[j]        = false;
					}
				}
			} else {
				// ChIndex0 for link0, ChIndex1 for link1
				if (z == (Aif2Fl_LinkIndex)0){
					PeCommonSetup.ChIndex0[k]          = i;
					PeCommonSetup.bEnableChIndex0[k]   = true;
					if (hAif->mode == AIF_GENERICPACKET_MODE) PeCommonSetup.CpriPktEn0[k] = true;
					else PeCommonSetup.CpriPktEn0[k]        = false;
				}
				if (z == (Aif2Fl_LinkIndex)1){
					PeCommonSetup.ChIndex1[k]          = i;
					PeCommonSetup.bEnableChIndex1[k]   = true;
					if (hAif->mode == AIF_GENERICPACKET_MODE) PeCommonSetup.CpriPktEn1[k] = true;
					else PeCommonSetup.CpriPktEn1[k]        = false;
				}
				if (z == (Aif2Fl_LinkIndex) 2) {
					PeCommonSetup.ChIndex2[k]          = i;
					PeCommonSetup.bEnableChIndex2[k]   = true;
					if (hAif->mode == AIF_GENERICPACKET_MODE) PeCommonSetup.CpriPktEn2[k] = true;
					else PeCommonSetup.CpriPktEn2[k]        = false;
				}
				if (z == (Aif2Fl_LinkIndex) 3) {
					PeCommonSetup.ChIndex3[k]          = i;
					PeCommonSetup.bEnableChIndex3[k]   = true;
					if (hAif->mode == AIF_GENERICPACKET_MODE) PeCommonSetup.CpriPktEn3[k] = true;
					else PeCommonSetup.CpriPktEn3[k]        = false;
				}
				if (z == (Aif2Fl_LinkIndex) 4) {
					PeCommonSetup.ChIndex4[k]          = i;
					PeCommonSetup.bEnableChIndex4[k]   = true;
					if (hAif->mode == AIF_GENERICPACKET_MODE) PeCommonSetup.CpriPktEn4[k] = true;
					else PeCommonSetup.CpriPktEn4[k]        = false;
				}
				if (z == (Aif2Fl_LinkIndex) 5) {
					PeCommonSetup.ChIndex5[k]          = i;
					PeCommonSetup.bEnableChIndex5[k]   = true;
					if (hAif->mode == AIF_GENERICPACKET_MODE) PeCommonSetup.CpriPktEn5[k] = true;
					else PeCommonSetup.CpriPktEn5[k]        = false;
				}
			}
		}
		k++;
	}

	if (AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol){
		//2560 CPRI samples (4 byte) are in  WCDMA slot time
		if (hAif->mode == AIF_WCDMA_MODE) {
			PeCommonSetup.PeFrameMsgTc[0] = ((PdCommonSetup.PdFrameMsgTc[0]+1)*4)-1;
			PeCommonSetup.PeFrameMsgTc[1] = ((PdCommonSetup.PdFrameMsgTc[1]+1)*4)-1;
		}

		// channel for ps
		for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
			if ((hAif->linkConfig[i].psMsgEnable) && (hAif->linkConfig[i].linkEnable)) {
				for(j=0;j<AIF2_CPRI_MAX_CW_SUBSTREAM;j++)
				{
					PeCommonSetup.bEnableCh[124+j] = true;//Enable Control channel
					PeCommonSetup.PeDmaCh0[124+j].bCrcEn = false;//disable CRC
					PeCommonSetup.PeDmaCh0[124+j].RtControl = AIF2FL_PE_RT_INSERT;//use PE insert option
					if (j == 0)		PeCommonSetup.PeDmaCh0[124+j].isEthernet = false;
					else			PeCommonSetup.PeDmaCh0[124+j].isEthernet = false;
					PeCommonSetup.PeInFifo[124+j].MFifoWmark = 2;//Message FIFO water mark
					PeCommonSetup.PeInFifo[124+j].MFifoFullLevel = 3;//Message FIFO full level
				}
			}
		}
	} else {
		if ((hAif->mode == AIF_WCDMA_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)) {
			PeCommonSetup.PeFrameMsgTc[0] = PdCommonSetup.PdFrameMsgTc[0];//OBSAI frame message terminal count for slot 0 ~ 14
			PeCommonSetup.PeFrameMsgTc[1] = PdCommonSetup.PdFrameMsgTc[1];
		}

		for (i = 0; i < AIF_MAX_NUM_LINKS; i++) {
			if (hAif->linkConfig[i].linkEnable) {
				// channel for ps (TO BE DEBUGGED)
				if (hAif->linkConfig[i].psMsgEnable) {
					PeCommonSetup.PeChObsaiType[122 + i]    = AIF2_OBSAI_TYPE_WCDMA_FDD;//OBSAI header type
					PeCommonSetup.PeChObsaiTS[122 + i]      = 0;//OBSAI header Time Stamp
					PeCommonSetup.PeChObsaiAddr[122 + i]    = i;//OBSAI header address (use simple rising sequence)
					PeCommonSetup.PeChObsaiTsMask[122 + i]  = AIF2FL_ROUTE_MASK_NONE;//not use Ts field
					PeCommonSetup.PeChObsaiTsfomat[122 + i]	= AIF2FL_TSTAMP_FORMAT_NORM_TS;//OBSAI header noraml TS format
					PeCommonSetup.PeObsaiPkt[122 + i]       = true;//Select OBSAI  packet mode
					PeCommonSetup.PeBbHop[122 + i]          = true;//Take OBSAI address from CPPI PS bits
					PeCommonSetup.PeModuloTc[i + 6].bEnableRule      = true; //enable rule
					PeCommonSetup.PeModuloTc[i + 6].RuleModulo       = 0;//modulo termical count(Modulo -1)
					PeCommonSetup.PeModuloTc[i + 6].bRuleObsaiCtlMsg = true;
					PeCommonSetup.PeModuloTc[i + 6].RuleIndex        = 0;//Setup modulo rule index (0 ~ Modulo-1)
					PeCommonSetup.PeModuloTc[i + 6].RuleLink         = i;//Route egress modulo rule 0 to link 0
					PeCommonSetup.PeObsaiDualBitMap[i + 6].DbmX      = 0;//DbmX number(1 PS channel for 4x link). set X-1 value
					PeCommonSetup.PeObsaiDualBitMap[i + 6].DbmXBubble= 0;//OBSAI 4 AxC sample length bubble
					PeCommonSetup.PeObsaiDualBitMap[i + 6].Dbm1Mult  = 0;//Dbm1 repetition number set n-1 value
					PeCommonSetup.PeObsaiDualBitMap[i + 6].Dbm1Size  = 0;//Dbm1 size (1 ~ 100) set n-1 value
					PeCommonSetup.PeObsaiDualBitMap[i + 6].Dbm1Map[0]= 0x0;// no bubble
					PeCommonSetup.PeObsaiDualBitMap[i + 6].Dbm2Size   = 0; //Dbm2 size (0 ~ 70)set n value
					PeCommonSetup.PeObsaiDualBitMap[i + 6].Dbm2Map[0] = 0x0;
				}

				//modulo rule 0 setup
				PeCommonSetup.PeModuloTc[i].bEnableRule      = true; //enable rule
				PeCommonSetup.PeModuloTc[i].RuleModulo       = 0;//modulo termical count(Modulo -1)
				PeCommonSetup.PeModuloTc[i].bRuleObsaiCtlMsg = false;
				PeCommonSetup.PeModuloTc[i].RuleIndex        = 0;//Setup modulo rule index (0 ~ Modulo-1)
				PeCommonSetup.PeModuloTc[i].RuleLink         = i;//Route egress modulo rule 0 to link 0

				//DBM rule 0 setup
				if (hAif->linkConfig[i].sampleRate == AIF_SRATE_23P04MHZ) {
					PeCommonSetup.PeObsaiDualBitMap[i].Dbm1Mult   = 0;//Dbm1 repetition number set n-1 value
					PeCommonSetup.PeObsaiDualBitMap[i].Dbm2Size   = 0; //Dbm2 size (0 ~ 70)set n value
					PeCommonSetup.PeObsaiDualBitMap[i].Dbm2Map[0] = 0x0;
					if (hAif->linkConfig[i].linkRate == 8) {
						PeCommonSetup.PeObsaiDualBitMap[i].DbmX		  = (hAif->linkConfig[i].linkRate*4*256*15/23040) - 1;
						PeCommonSetup.PeObsaiDualBitMap[i].DbmXBubble = 0;//OBSAI 4 AxC sample length bubble
						PeCommonSetup.PeObsaiDualBitMap[i].Dbm1Map[0] = 0x2;
						PeCommonSetup.PeObsaiDualBitMap[i].Dbm1Size   = 2;//Dbm1 size (1 ~ 100) set n-1 value
					} else if (hAif->linkConfig[i].linkRate == 4) {
						PeCommonSetup.PeObsaiDualBitMap[i].DbmX		  = 7; // Eight message slot for only 2 AxCs
						PeCommonSetup.PeObsaiDualBitMap[i].DbmXBubble = 0;//OBSAI two msg length bubble
						PeCommonSetup.PeObsaiDualBitMap[i].Dbm1Map[0] = 0x0;
						PeCommonSetup.PeObsaiDualBitMap[i].Dbm1Size   = 0;//Dbm1 size (1 ~ 100) set n-1 value
						for(j=2;j<8;j+=2)
						{
							PeCommonSetup.ChIndex0[j+(64*i)]         = PeCommonSetup.ChIndex0[0+(64*i)];
							PeCommonSetup.ChIndex0[j+1+(64*i)]       = PeCommonSetup.ChIndex0[1+(64*i)];
							if (j<6) {
								PeCommonSetup.bEnableChIndex0[j+(64*i)]  = true;
								PeCommonSetup.bEnableChIndex0[j+1+(64*i)]= true;
							} else {
								PeCommonSetup.bEnableChIndex0[j+(64*i)]  = false;
								PeCommonSetup.bEnableChIndex0[j+1+(64*i)]= false;
							}
						}
					}
				} else {
					if (hAif->mode == AIF_WCDMA_MODE)
						PeCommonSetup.PeObsaiDualBitMap[i].DbmX		  = (hAif->linkConfig[i].linkRate*4) - 1; //hAif->linkConfig[i].numPeAxC - 1;//DbmX number(16 for 4x link). set X-1 value
					else
						PeCommonSetup.PeObsaiDualBitMap[i].DbmX		  = hAif->linkConfig[i].numPeAxC - 1;//DbmX number(16 for 4x link). set X-1 value
					PeCommonSetup.PeObsaiDualBitMap[i].DbmXBubble = 0;//OBSAI 4 AxC sample length bubble
					PeCommonSetup.PeObsaiDualBitMap[i].Dbm1Mult   = 0;//Dbm1 repetition number set n-1 value
					PeCommonSetup.PeObsaiDualBitMap[i].Dbm1Size   = 0;//Dbm1 size (1 ~ 100) set n-1 value
					PeCommonSetup.PeObsaiDualBitMap[i].Dbm1Map[0] = 0x0;// no bubble
					PeCommonSetup.PeObsaiDualBitMap[i].Dbm2Size   = 0; //Dbm2 size (0 ~ 70)set n value
					PeCommonSetup.PeObsaiDualBitMap[i].Dbm2Map[0] = 0x0;
				}
			}

		}
	}

	if (hAif->pktdmaOrDioEngine == AIF2FL_DIO)
	{
		//Ingress DB setup
		IngrDbSetup.bEnableIngrDb = true; //Enable Ingress DB
		if (hAif->mode == AIF_LTE_WCDMA_MODE)
			IngrDbSetup.DioBufferLen = AIF2FL_DB_DIO_LEN_256; //Ingress DB DIO buffer length
		else
			IngrDbSetup.DioBufferLen = AIF2FL_DB_DIO_LEN_128; //Ingress DB DIO buffer length

		z           = first_link; //track to which link this channel corresponds
		baseAddress = AIF2_DB_BASE_ADDR_I_FIFO_0;
		for (i = 0; i < max_channel_pd; i++)//for channel 0,...to max channels used by all links
		{
			if (i >= (hAif->linkConfig[z].firstPdDBCH
					+ hAif->linkConfig[z].numPdAxC)) {
				do {
					z++;
				} while ((0 == hAif->linkConfig[z].linkEnable));
			}
			IngrDbSetup.bEnableChannel[i]               = true; //Enable 4 Ingress DB channel
			IngrDbSetup.IngrDbChannel[i].BaseAddress    = baseAddress;
			if (((hAif->linkConfig[z].multiRate == 0) && (hAif->mode == AIF_LTE_WCDMA_MODE && hAif->linkConfig[z].mode == LTE)) || ((hAif->linkConfig[z].multiRate == 1) && (hAif->mode == AIF_LTE_WCDMA_MODE && (hAif->AxCconfig[i].sampleRate != AIF_SRATE_3P84MHZ))))
			{
				IngrDbSetup.IngrDbChannel[i].DataSwap   = AIF2FL_DB_WORD_SWAP; //No swap for DL
				baseAddress                            += 2;
				IngrDbSetup.IngrDbChannel[i].IQOrder        = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			} else {
				if (hAif->linkConfig[z].inboundDataType==AIF2FL_LINK_DATA_TYPE_NORMAL) {
					IngrDbSetup.IngrDbChannel[i].DataSwap   = AIF2FL_DB_WORD_SWAP; //No swap for DL
					baseAddress                            += 1;
				} else {
					IngrDbSetup.IngrDbChannel[i].DataSwap   = AIF2FL_DB_BYTE_SWAP; //swap for UL
					baseAddress                            += 2;
				}
				IngrDbSetup.IngrDbChannel[i].IQOrder        = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			}
		}
        baseIngr = baseAddress;

		//Egress DB setup
		EgrDbSetup.bEnableEgrDb = true; //Enable Egress DB
		if (hAif->mode == AIF_LTE_WCDMA_MODE)
		{
			EgrDbSetup.DioBufferLen = AIF2FL_DB_DIO_LEN_256; //Egress DB DIO buffer length
			EgrDbSetup.PmControl    = AIF2FL_DB_PM_TOKEN_FIFO;//to enhance CPRI performance
		} else {
			EgrDbSetup.DioBufferLen = AIF2FL_DB_DIO_LEN_128; //Egress DB DIO buffer length
			EgrDbSetup.PmControl    = AIF2FL_DB_AXC_TOKEN_FIFO;//to enhance CPRI performance
		}
		z           = first_link; //track to which link this channel corresponds
		baseAddress = AIF2_DB_BASE_ADDR_E_FIFO_0;
		for (i = 0; i < max_channel_pe; i++)//for channel 0,...to max channels used by all links
		{
			if (i >= (hAif->linkConfig[z].firstPeDBCH
					+ hAif->linkConfig[z].numPeAxC)) {
				do {
					z++;
				} while ((0 == hAif->linkConfig[z].linkEnable));
			}
			EgrDbSetup.bEnableChannel[i]               = true; //Enable 4 Egress DB channel
			EgrDbSetup.EgrDbChannel[i].BaseAddress    = baseAddress;
			if (((hAif->linkConfig[z].multiRate == 0) && (hAif->mode == AIF_LTE_WCDMA_MODE && hAif->linkConfig[z].mode == LTE)) || ((hAif->linkConfig[z].multiRate == 1) && (hAif->mode == AIF_LTE_WCDMA_MODE && (hAif->AxCconfig[i].sampleRate != AIF_SRATE_3P84MHZ))))
			{
				EgrDbSetup.EgrDbChannel[i].DataSwap   = AIF2FL_DB_WORD_SWAP; //No swap for DL
				baseAddress                            += 2;
				EgrDbSetup.EgrDbChannel[i].IQOrder        = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			} else {
				if (hAif->linkConfig[z].outboundDataType==AIF2FL_LINK_DATA_TYPE_NORMAL)
				{
					EgrDbSetup.EgrDbChannel[i].DataSwap   = AIF2FL_DB_WORD_SWAP; //No swap for DL
					baseAddress                            += 1;
				} else {
					EgrDbSetup.EgrDbChannel[i].DataSwap   = AIF2FL_DB_BYTE_SWAP; //swap for UL
					baseAddress                            += 2;
				}
				EgrDbSetup.EgrDbChannel[i].IQOrder        = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			}
		}
		baseEgr = baseAddress;

	} else {
		if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)) {

		   IngrDbSetup.bEnableIngrDb                  = true; //Enable Ingress DB
			z           = first_link; //track to which link this channel corresponds
			baseAddress = AIF2_DB_BASE_ADDR_I_FIFO_0;
			for (i = 0; i < max_channel_pd; i++)//for channel 0,...to max channels used by all links
			{
				if (i >= (hAif->linkConfig[z].firstPdDBCH
						+ hAif->linkConfig[z].numPdAxC)) {
					do {
						z++;
					} while ((0 == hAif->linkConfig[z].linkEnable));
				}
				IngrDbSetup.bEnableChannel[i]               = true; //Enable 4 Ingress DB channel
#ifdef DEVICE_LE
				IngrDbSetup.IngrDbChannel[i].DataSwap   	= AIF2FL_DB_WORD_SWAP;  //Word  swap for DL (LE), No SWAP for (BE)
#else
				IngrDbSetup.IngrDbChannel[i].DataSwap   	= AIF2FL_DB_NO_SWAP;  //Word  swap for DL (LE), No SWAP for (BE)
#endif
				IngrDbSetup.IngrDbChannel[i].BaseAddress = baseAddress;
				
				if (hAif->linkConfig[z].multiRate == 0){
					if (hAif->linkConfig[z].sampleRate == AIF_SRATE_1P92MHZ)
					{
						baseAddress += i;
					} else {
						baseAddress += (1 << hAif->AxCconfig[i].ingressBufDepth);
					}	
				} else {
					if (hAif->AxCconfig[i].sampleRate == AIF_SRATE_1P92MHZ)
					{
						baseAddress += i;
					} else {
						baseAddress += (1 << hAif->AxCconfig[i].ingressBufDepth);
					}
				}
				
				IngrDbSetup.IngrDbChannel[i].BufDepth      	= hAif->AxCconfig[i].ingressBufDepth; //Set DB FIFO depth for channel 0 to 64 QW(Quad word)

				IngrDbSetup.IngrDbChannel[i].IQOrder        = AIF2FL_DB_IQ_NO_SWAP; //No Order change
		        IngrDbSetup.IngrDbChannel[i].bEnablePsData = true; //Enable 4 bytes PS data
		        IngrDbSetup.IngrDbChannel[i].PacketType    = 0; //User data

			}
			
			baseIngr = baseAddress;

		    //Egress DB setup

		    EgrDbSetup.bEnableEgrDb    = true; //Enable Ingress DB
		    EgrDbSetup.PmControl       = AIF2FL_DB_AXC_TOKEN_FIFO;//to enhance CPRI performance
			z           = first_link; //track to which link this channel corresponds
			baseAddress = AIF2_DB_BASE_ADDR_E_FIFO_0;
			for (i = 0; i < max_channel_pe; i++)//for channel 0,...to max channels used by all links
			{
				if (i >= (hAif->linkConfig[z].firstPeDBCH
						+ hAif->linkConfig[z].numPeAxC)) {
					do {
						z++;
					} while ((0 == hAif->linkConfig[z].linkEnable));
				}
				EgrDbSetup.bEnableChannel[i]               = true; //Enable 4 Egress DB channel

#ifdef DEVICE_LE
					EgrDbSetup.EgrDbChannel[i].DataSwap   	= AIF2FL_DB_WORD_SWAP; //Word  swap for DL (LE), No SWAP for (BE)
#else
					EgrDbSetup.EgrDbChannel[i].DataSwap   	= AIF2FL_DB_NO_SWAP; //Word  swap for DL (LE), No SWAP for (BE)
#endif
				EgrDbSetup.EgrDbChannel[i].BaseAddress = baseAddress;
				if (hAif->linkConfig[z].multiRate == 0){
					if (hAif->linkConfig[z].sampleRate == AIF_SRATE_1P92MHZ)
					{
						baseAddress += i;
					} else {
						baseAddress += (1 << hAif->AxCconfig[i].egressBufDepth);
					}
				} else {
					if (hAif->AxCconfig[i].sampleRate == AIF_SRATE_1P92MHZ)
					{
						baseAddress += i;
					} else {
						baseAddress += (1 << hAif->AxCconfig[i].egressBufDepth);
					}
				}
				EgrDbSetup.EgrDbChannel[i].BufDepth    = hAif->AxCconfig[i].egressBufDepth; //Set DB FIFO depth for channel 0 to 64 QW(Quad word)
				EgrDbSetup.EgrDbChannel[i].IQOrder     = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			}

			baseEgr = baseAddress;			
		}
		if ((hAif->mode == AIF_GENERICPACKET_MODE) ) {

			IngrDbSetup.bEnableIngrDb                  = true; //Enable Ingress DB
			z           = first_link; //track to which link this channel corresponds
			baseAddress = AIF2_DB_BASE_ADDR_I_FIFO_0;
			for (i = 0; i < max_channel_pd; i++)//for channel 0,...to max channels used by all links
			{
				if (i >= (hAif->linkConfig[z].firstPdDBCH
						+ hAif->linkConfig[z].numPdAxC)) {
					do {
						z++;
					} while ((0 == hAif->linkConfig[z].linkEnable));
				}
				IngrDbSetup.bEnableChannel[i]               = true; //Enable 4 Ingress DB channel
				IngrDbSetup.IngrDbChannel[i].BaseAddress    = baseAddress;
				baseAddress                           		+= 8;
				IngrDbSetup.IngrDbChannel[i].BufDepth      	= hAif->AxCconfig[i].ingressBufDepth; //Set DB FIFO depth for channel 0 to 64 QW(Quad word)
			    IngrDbSetup.IngrDbChannel[i].DataSwap      = AIF2FL_DB_WORD_SWAP; //DL
			    IngrDbSetup.IngrDbChannel[i].IQOrder       = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			    IngrDbSetup.IngrDbChannel[i].bEnablePsData = false; //Enable 4 bytes PS data
			    IngrDbSetup.IngrDbChannel[i].PacketType    = 0; //User data
			}
			baseIngr = baseAddress;

			   //Egress DB setup
			EgrDbSetup.bEnableEgrDb                    = true; //Enable Ingress DB
			if (AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol) EgrDbSetup.PmControl = AIF2FL_DB_AXC_TOKEN_FIFO;//for normal packet performance
			else EgrDbSetup.PmControl                  = AIF2FL_DB_PM_TOKEN_FIFO;//for normal packet performance
			z           = first_link; //track to which link this channel corresponds
			baseAddress = AIF2_DB_BASE_ADDR_E_FIFO_0;
			for (i = 0; i < max_channel_pe; i++)//for channel 0,...to max channels used by all links
			{
				if (i >= (hAif->linkConfig[z].firstPeDBCH
						+ hAif->linkConfig[z].numPeAxC)) {
					do {
						z++;
					} while ((0 == hAif->linkConfig[z].linkEnable));
				}
				EgrDbSetup.bEnableChannel[i]              = true; //Enable 4 Egress DB channel
				EgrDbSetup.EgrDbChannel[i].BaseAddress	  = baseAddress;
				baseAddress                               += 8;
				EgrDbSetup.EgrDbChannel[i].BufDepth        = hAif->AxCconfig[i].egressBufDepth; //Set DB FIFO depth for channel 0
				EgrDbSetup.EgrDbChannel[i].DataSwap        = AIF2FL_DB_WORD_SWAP; //DL
				EgrDbSetup.EgrDbChannel[i].IQOrder        = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			}
			baseEgr = baseAddress;
		}
	}
	//configure for PS
	for (i=0; i<AIF_MAX_NUM_LINKS; i++)
	{
		if ((hAif->linkConfig[i].psMsgEnable) && (hAif->linkConfig[i].linkEnable)) {
			if (AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol) {
				for(j=0;j<AIF2_CPRI_MAX_CW_SUBSTREAM;j++)
				{
					IngrDbSetup.bEnableChannel[124+j]            = true; //Enable 4 Ingress DB channel
					IngrDbSetup.IngrDbChannel[124+j].BaseAddress = baseIngr+(j*8);
#ifdef DEVICE_LE
					IngrDbSetup.IngrDbChannel[124+j].DataSwap    = AIF2FL_DB_BYTE_SWAP;//AIF2FL_DB_WORD_SWAP; //No swap
#else					
					IngrDbSetup.IngrDbChannel[124+j].DataSwap    = AIF2FL_DB_NO_SWAP;//AIF2FL_DB_WORD_SWAP; //No swap
#endif					
					IngrDbSetup.IngrDbChannel[124+j].IQOrder     = AIF2FL_DB_IQ_NO_SWAP; //No Order change
				}
			} else { // OBSAI
				IngrDbSetup.bEnableChannel[122+i]            = true; //Enable 4 Ingress DB channel
				IngrDbSetup.IngrDbChannel[122+i].BaseAddress = baseIngr;
				IngrDbSetup.IngrDbChannel[122+i].DataSwap    = AIF2FL_DB_BYTE_SWAP; //AIF2FL_DB_WORD_SWAP; //No swap
				IngrDbSetup.IngrDbChannel[122+i].IQOrder     = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			}
		}
	}

	//configure for PS
	for (i=0; i<AIF_MAX_NUM_LINKS; i++)
	{
		if ((hAif->linkConfig[i].psMsgEnable) && (hAif->linkConfig[i].linkEnable)) {
			if (AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol) {
				for(j=0;j<AIF2_CPRI_MAX_CW_SUBSTREAM;j++)
				{
					EgrDbSetup.bEnableChannel[124+j]            = true; //Enable 4 Egress DB channel
					EgrDbSetup.EgrDbChannel[124+j].BaseAddress = baseEgr+(j*8);
#ifdef DEVICE_LE
					EgrDbSetup.EgrDbChannel[124+j].DataSwap    = AIF2FL_DB_BYTE_SWAP;//AIF2FL_DB_WORD_SWAP; //No swap
#else
					EgrDbSetup.EgrDbChannel[124+j].DataSwap    = AIF2FL_DB_NO_SWAP;//AIF2FL_DB_WORD_SWAP; //No swap
#endif					
					EgrDbSetup.EgrDbChannel[124+j].IQOrder     = AIF2FL_DB_IQ_NO_SWAP; //No Order change
				}
			} else { // OBSAI
				EgrDbSetup.bEnableChannel[122+i]            = true; //Enable 4 Egress DB channel
				EgrDbSetup.EgrDbChannel[122+i].BaseAddress = baseEgr;
				EgrDbSetup.EgrDbChannel[122+i].DataSwap    = AIF2FL_DB_BYTE_SWAP;//AIF2FL_DB_WORD_SWAP; //No swap
				EgrDbSetup.EgrDbChannel[122+i].IQOrder     = AIF2FL_DB_IQ_NO_SWAP; //No Order change
			}
		}
	}

	// AD Common
	AdCommonSetup.IngrGlobalEnable = true;
	AdCommonSetup.EgrGlobalEnable = true;
	AdCommonSetup.FailMode = AIF2FL_AD_DROP;//drop fail packet
	AdCommonSetup.Tx_QueNum = AIF2_BASE_TX_QUE_NUM;//base egress queue number setup to 512
	if (hAif->pktdmaOrDioEngine == AIF2FL_DIO)
	{
		//DIO setup
		AdCommonSetup.IngrGlobalDioEnable = true;
		AdCommonSetup.EgrGlobalDioEnable = true;
		AdCommonSetup.IngrPriority = AIF2FL_AD_DIO_PRI;
		AdCommonSetup.EgrPriority = AIF2FL_AD_AXC_PRI;
	} else {
		// PKTDMA setup
		AdCommonSetup.IngrGlobalDioEnable = false;
		AdCommonSetup.EgrGlobalDioEnable = false;
		AdCommonSetup.IngrPriority = AIF2FL_AD_PKT_PRI;
		if ((hAif->mode == AIF_GENERICPACKET_MODE) )
			AdCommonSetup.EgrPriority = AIF2FL_AD_NON_AXC_PRI;
		else 	AdCommonSetup.EgrPriority = AIF2FL_AD_AXC_PRI;
	}

	// AD DIO engine setup
	if (hAif->pktdmaOrDioEngine == AIF2FL_DIO)
	{
		for (i=0; i<AIF2_MAX_NUM_DIO_ENGINE; i++)
		{
			if (hAif->dioConfig[i].numLink != 0)
			{
				if (hAif->dioConfig[i].mode == LTE)
				{
					if (hAif->dioConfig[i].numPdDBCH) AdDioSetup.IngrDioEngineEnable[i]          = true;
					AdDioSetup.IngrDioEngine[i].BcnTableSelect = AIF2FL_AD_DIO_BCN_TABLE_0;
					AdDioSetup.IngrDioEngine[i].NumAxC         = hAif->dioConfig[i].numPdDBCH - 1;
					AdDioSetup.IngrDioEngine[i].DmaNumBlock    = hAif->dioConfig[i].inNumBlock - 1; //block wrap2 value (n-1)
					AdDioSetup.IngrDioEngine[i].NumQuadWord        = AIF2FL_AD_4QUAD;//Use 1 QW per channel in DL for ABT
					AdDioSetup.IngrDioEngine[i].bEnEgressRsaFormat = false;
					AdDioSetup.IngrDioEngine[i].DmaBlockAddrStride = 4;
					AdDioSetup.IngrDioEngine[i].bEnDmaChannel      = true; //Enable Dma channel
					AdDioSetup.IngrDioEngine[i].DmaBurstLen        = AIF2FL_AD_4QUAD;//1 max QW burst per one transfer
					AdDioSetup.IngrDioEngine[i].DmaBaseAddr        = (uint32_t)hAif->dioConfig[i].in;
					AdDioSetup.IngrDioEngine[i].DmaBurstAddrStride = 4 * hAif->dioConfig[i].inNumBlock;//DMA burst stride to 1 (wrap1)if 1QW and 4 for 4QW for DMABurstLen
					for (k = 0, j = hAif->dioConfig[i].offsetPdDBCH; j < (hAif->dioConfig[i].numPdDBCH + hAif->dioConfig[i].offsetPdDBCH); j++, k++)
					{
							AdDioSetup.IngrDioEngine[i].DBCN[k] = j; //set ingress table DBCN for channel 0
					}

					if (hAif->linkConfig[i].RtEnabled==0)
					{
						if (hAif->dioConfig[i].numPeDBCH) AdDioSetup.EgrDioEngineEnable[i]          = true;
						AdDioSetup.EgrDioEngine[i].BcnTableSelect = AIF2FL_AD_DIO_BCN_TABLE_0;
						AdDioSetup.EgrDioEngine[i].NumAxC         = hAif->dioConfig[i].numPeDBCH - 1;
						AdDioSetup.EgrDioEngine[i].DmaNumBlock    = hAif->dioConfig[i].outNumBlock - 1; //block wrap2 value (n-1)
						AdDioSetup.EgrDioEngine[i].NumQuadWord        = AIF2FL_AD_4QUAD;//Use 1 QW per channel in DL for ABT
						AdDioSetup.EgrDioEngine[i].bEnEgressRsaFormat = false;
						AdDioSetup.EgrDioEngine[i].DmaBlockAddrStride = 4;
						AdDioSetup.EgrDioEngine[i].bEnDmaChannel      = true; //Enable Dma channel
						AdDioSetup.EgrDioEngine[i].DmaBurstLen        = AIF2FL_AD_4QUAD;//1 max QW burst per one transfer
						AdDioSetup.EgrDioEngine[i].DmaBaseAddr        = (uint32_t)hAif->dioConfig[i].out;
						AdDioSetup.EgrDioEngine[i].DmaBurstAddrStride = 4 *hAif->dioConfig[i].outNumBlock;//DMA burst stride to 1 (wrap1)if 1QW and 4 for 4QW for DMABurstLen
						for (k = 0, j = hAif->dioConfig[i].offsetPeDBCH; j < (hAif->dioConfig[i].numPeDBCH + hAif->dioConfig[i].offsetPeDBCH); j++, k++)
						{
								AdDioSetup.EgrDioEngine[i].DBCN[k] = j; //set ingress table DBCN for channel 0
						}
					}
				} else {
					//DIO Ingress-Egress for L2/RSA are engine0
					if (hAif->dioConfig[i].numPdDBCH) AdDioSetup.IngrDioEngineEnable[i]          = true;
					AdDioSetup.IngrDioEngine[i].BcnTableSelect = AIF2FL_AD_DIO_BCN_TABLE_0;
					AdDioSetup.IngrDioEngine[i].NumAxC         = hAif->dioConfig[i].numPdDBCH - 1;
					AdDioSetup.IngrDioEngine[i].DmaNumBlock    = hAif->dioConfig[i].inNumBlock - 1; //block wrap2 value (n-1)
					if (hAif->linkConfig[hAif->dioConfig[i].firstLink].inboundDataType==AIF2FL_LINK_DATA_TYPE_RSA)
					{
						AdDioSetup.IngrDioEngine[i].NumQuadWord        = AIF2FL_AD_2QUAD;//Use 2 QW per channel in UL
						AdDioSetup.IngrDioEngine[i].bEnEgressRsaFormat = true;
						if (hAif->dioConfig[i].usedWithRAC == 0) {
							AdDioSetup.IngrDioEngine[i].DmaBlockAddrStride = 2 * (hAif->dioConfig[i].numPdDBCH);
						} else {
							// The fact that the RAC FE is mapped with a jump of 0x800 every 8 chips with 4 blocks every 32-chip period
							//  -> Block Stride is 0x80
							//  -> Burst Stried is 4
							AdDioSetup.IngrDioEngine[i].DmaBlockAddrStride = 0x80;
						}
					} else {
						AdDioSetup.IngrDioEngine[i].NumQuadWord        = AIF2FL_AD_1QUAD;//Use 1 QW per channel in DL for ABT
						AdDioSetup.IngrDioEngine[i].bEnEgressRsaFormat = false;
						AdDioSetup.IngrDioEngine[i].DmaBlockAddrStride = hAif->dioConfig[i].numPdDBCH;
					}
					AdDioSetup.IngrDioEngine[i].bEnDmaChannel      = true; //Enable Dma channel
					AdDioSetup.IngrDioEngine[i].DmaBurstLen        = AIF2FL_AD_4QUAD;//1 max QW burst per one transfer
					AdDioSetup.IngrDioEngine[i].DmaBaseAddr        = (uint32_t)hAif->dioConfig[i].in;
					AdDioSetup.IngrDioEngine[i].DmaBurstAddrStride = 4;//DMA burst stride to 1 (wrap1)if 1QW and 4 for 4QW for DMABurstLen
					for (k = 0, j = hAif->dioConfig[i].offsetPdDBCH; j < (hAif->dioConfig[i].numPdDBCH + hAif->dioConfig[i].offsetPdDBCH); j++, k++)
					{
							AdDioSetup.IngrDioEngine[i].DBCN[k] = j; //set ingress table DBCN for channel 0
					}

					if (hAif->linkConfig[i].RtEnabled==0)
					{
						if (hAif->dioConfig[i].numPeDBCH) AdDioSetup.EgrDioEngineEnable[i]          = true;
						AdDioSetup.EgrDioEngine[i].BcnTableSelect = AIF2FL_AD_DIO_BCN_TABLE_0;
						AdDioSetup.EgrDioEngine[i].NumAxC         = hAif->dioConfig[i].numPeDBCH - 1;
						AdDioSetup.EgrDioEngine[i].DmaNumBlock    = hAif->dioConfig[i].outNumBlock - 1; //block wrap2 value (n-1)
						if (hAif->linkConfig[hAif->dioConfig[i].firstLink].outboundDataType==AIF2FL_LINK_DATA_TYPE_RSA)
						{
							AdDioSetup.EgrDioEngine[i].NumQuadWord        = AIF2FL_AD_2QUAD;//Use 2 QW per channel in UL
							AdDioSetup.EgrDioEngine[i].bEnEgressRsaFormat = true;
							AdDioSetup.EgrDioEngine[i].DmaBlockAddrStride = 2 * (hAif->dioConfig[i].numPeDBCH);
						} else {
							AdDioSetup.EgrDioEngine[i].NumQuadWord        = AIF2FL_AD_1QUAD;//Use 1 QW per channel in DL for ABT
							AdDioSetup.EgrDioEngine[i].bEnEgressRsaFormat = false;
							AdDioSetup.EgrDioEngine[i].DmaBlockAddrStride = hAif->dioConfig[i].numPeDBCH;
						}
						AdDioSetup.EgrDioEngine[i].bEnDmaChannel      = true; //Enable Dma channel
						AdDioSetup.EgrDioEngine[i].DmaBurstLen        = AIF2FL_AD_4QUAD;//1 max QW burst per one transfer
						AdDioSetup.EgrDioEngine[i].DmaBaseAddr        = (uint32_t)hAif->dioConfig[i].out;
						AdDioSetup.EgrDioEngine[i].DmaBurstAddrStride = 4;//DMA burst stride to 1 (wrap1)if 1QW and 4 for 4QW for DMABurstLen
						for (k = 0, j = hAif->dioConfig[i].offsetPeDBCH; j < (hAif->dioConfig[i].numPeDBCH + hAif->dioConfig[i].offsetPeDBCH); j++, k++)
						{
								AdDioSetup.EgrDioEngine[i].DBCN[k] = j; //set ingress table DBCN for channel 0
						}
					}
				}
			}
			if (hAif->dioConfig[i].duplicateOnDioEng2)
			{
				AdDioSetup.IngrDioEngineEnable[2]=AdDioSetup.IngrDioEngineEnable[i];
				AdDioSetup.IngrDioEngine[2]=AdDioSetup.IngrDioEngine[i];
				AdDioSetup.EgrDioEngineEnable[2]=AdDioSetup.EgrDioEngineEnable[i];
				AdDioSetup.EgrDioEngine[2]=AdDioSetup.EgrDioEngine[i];
				AdDioSetup.IngrDioEngine[2].DmaBaseAddr = (uint32_t)hAif->dioConfig[2].in;
			}
		}
	}

	//AT Common setup
	if (hAif->aif2TimerSyncSource != AIF2FL_PHYT_CMP_SYNC) {
		AtCommonSetup.PhySyncSel     = hAif->aif2TimerSyncSource;
	} else {
		AtCommonSetup.PhySyncSel     = AIF2FL_CHIP_INPUT_SYNC;
	}
	if (hAif->phytCompValue)
	{
		AtCommonSetup.RadSyncSel     = AIF2FL_PHYT_CMP_SYNC;
		AtCommonSetup.PhytCompValue  = hAif->phytCompValue;

	} else {
		AtCommonSetup.RadSyncSel     = hAif->aif2TimerSyncSource;
		AtCommonSetup.PhytCompValue  = 0;
	}

	AtCommonSetup.AutoResyncMode = hAif->autoResyncMode;
	AtCommonSetup.SyncMode       = AIF2FL_NON_RP1_MODE; // same with AIF and Fsync
	AtCommonSetup.CrcMode        = AIF2FL_AT_CRC_DONT_USE;//Do not use RP1 CRC in this test same with AIF and Fsync

	if ((hAif->mode == AIF_WCDMA_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)) {
		if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
			AtCommonSetup.WcdmaDivTC = 79; //80 is default divide value for WCDMA 307.2/3.84Mhz
			AtCommonSetup.AtTerminalCount.RadClockCountTc[0] = AIF2_CLOCK_COUNT_TC_WCDMA_FDD;//set WCDMA Clock count TC to 204799
		}
		else {
			AtCommonSetup.WcdmaDivTC = 63; //64 is default divide value for WCDMA CPRI
			AtCommonSetup.AtTerminalCount.RadClockCountTc[0] = AIF2_CLOCK_COUNT_TC_WCDMA_FDD_CPRI;
		}
	}
	if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)) {
		if (hAif->radTimerConfig[0].userSpecified == false) {
			if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
				if (lte1_4 == 1)
				{
					AtCommonSetup.AtTerminalCount.RadClockCountTc[0] = (AIF2_CLOCK_COUNT_TC_OBSAI+1)/2-1;
				} else {
					AtCommonSetup.AtTerminalCount.RadClockCountTc[0] = (AIF2_CLOCK_COUNT_TC_FIRST_OFDM_SYM_OBSAI-1);//set Clock count TC for OBSAI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[1] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_OBSAI-1);//set Clock count TC for OBSAI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[2] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_OBSAI-1);//set Clock count TC for OBSAI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[3] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_OBSAI-1);//set Clock count TC for OBSAI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[4] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_OBSAI-1);//set Clock count TC for OBSAI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[5] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_OBSAI-1);//set Clock count TC for OBSAI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[6] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_OBSAI-1);//set Clock count TC for OBSAI
				}
			} else {
				if (lte1_4 == 1)
				{
					AtCommonSetup.AtTerminalCount.RadClockCountTc[0] = (AIF2_CLOCK_COUNT_TC_CPRI+1)/2-1;
				} else {
					AtCommonSetup.AtTerminalCount.RadClockCountTc[0] = (AIF2_CLOCK_COUNT_TC_FIRST_OFDM_SYM_CPRI-1);//set Clock count TC for CPRI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[1] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_CPRI-1);//set Clock count TC forCPRI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[2] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_CPRI-1);//set Clock count TC forCPRI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[3] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_CPRI-1);//set Clock count TC forCPRI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[4] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_CPRI-1);//set Clock count TC forCPRI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[5] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_CPRI-1);//set Clock count TC forCPRI
					AtCommonSetup.AtTerminalCount.RadClockCountTc[6] = (AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_CPRI-1);//set Clock count TC forCPRI
				}
			}
		} else {
			if (hAif->radTimerConfig[0].lte.cpType == AIF2_LTE_CPTYPE_NORMAL) {
				if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
					clockCountTc[0] = AIF2_CLOCK_COUNT_TC_FIRST_OFDM_SYM_OBSAI;//set Clock count TC for OBSAI
					for (i=1;i<AIF2_LTE_SYMBOL_NUM;i++) {
						clockCountTc[i] = AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_OBSAI;//set Clock count TC for OBSAI
					}
				} else {
					clockCountTc[0] = AIF2_CLOCK_COUNT_TC_FIRST_OFDM_SYM_CPRI;//set Clock count TC for OBSAI
					for (i=1;i<AIF2_LTE_SYMBOL_NUM;i++) {
						clockCountTc[i] = AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_CPRI;//set Clock count TC for OBSAI
					}
				}
				k = 0;
				for (i=0;i<AIF2_LTE_SYMBOL_NUM;i++) {
					for (j=0;j<hAif->radTimerConfig[0].lte.numSymbolsForSymbolStrobe;j++) {
						AtCommonSetup.AtTerminalCount.RadClockCountTc[i] += clockCountTc[k++];
						if (k==AIF2_LTE_SYMBOL_NUM) k=0;
					}
					AtCommonSetup.AtTerminalCount.RadClockCountTc[i] -= 1;
				}
			} else { // Extended CP
				if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
					for (i=0;i<AIF2_LTE_SYMBOL_NUM_EXT_CP;i++) {
						AtCommonSetup.AtTerminalCount.RadClockCountTc[i] = (hAif->radTimerConfig[0].lte.numSymbolsForSymbolStrobe*AIF2_CLOCK_COUNT_TC_EXTCP_OFDM_SYM_OBSAI)-1;//set Clock count TC for OBSAI
					}
				} else {
					for (i=0;i<AIF2_LTE_SYMBOL_NUM_EXT_CP;i++) {
						AtCommonSetup.AtTerminalCount.RadClockCountTc[i] = (hAif->radTimerConfig[0].lte.numSymbolsForSymbolStrobe*AIF2_CLOCK_COUNT_TC_EXTCP_OFDM_SYM_CPRI)-1;//set Clock count TC for OBSAI
					}
				}
			}
		}
	}

	if ((hAif->mode == AIF_GENERICPACKET_MODE)){
		if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
			AtCommonSetup.PhytCompValue = 0;
			AtCommonSetup.AtTerminalCount.RadClockCountTc[0] = AIF2_CLOCK_COUNT_TC_OBSAI;//CLOCK_COUNT_TC_LTE_FDD;//set Clock count TC to 307199
		} else {
			AtCommonSetup.AtTerminalCount.RadClockCountTc[0] = AIF2_CLOCK_COUNT_TC_CPRI;//set Clock count TC for CPRI
		}
	}
	AtCommonSetup.AtInit.pPhyTimerInit   = &PhyTimerInit;
	PhyTimerInit.ClockNum    = 0;
	PhyTimerInit.FrameLsbNum = 0;
	PhyTimerInit.FrameMsbNum = 0;

	AtCommonSetup.AtInit.pRadTimerInit   = &RadTimerInit;
	if (hAif->radTimerConfig[0].userSpecified == false) {
		if (hAif->aif2TimerSyncSource != AIF2FL_PHYT_CMP_SYNC) {
			RadTimerInit.ClockNum = 0;
		} else {
			RadTimerInit.ClockNum = 2;
		}
		RadTimerInit.SymbolNum    = 0;
		RadTimerInit.FrameLsbNum  = 0;
		RadTimerInit.FrameMsbNum  = 0;
	} else {
		RadTimerInit.ClockNum     = hAif->radTimerConfig[0].initClockNum;
		RadTimerInit.SymbolNum    = hAif->radTimerConfig[0].initSymbolNum;
		RadTimerInit.FrameLsbNum  = hAif->radTimerConfig[0].initFrameLsbNum;
		RadTimerInit.FrameMsbNum  = hAif->radTimerConfig[0].initFrameMsbNum;
	}

	AtCommonSetup.AtInit.pUlRadTimerInit = &UlRadTimerInit;
	UlRadTimerInit.SymbolNum   = 14; // WCDMA same with AIF and Fsync
	UlRadTimerInit.FrameLsbNum = 4095;
	UlRadTimerInit.FrameMsbNum = 0;
	UlRadTimerInit.FcbMinusOne = 1; // Applies to RP1 interface only
	//full symbol clock value - first ingress DIO event time (as for RAC processing)
	if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
		UlRadTimerInit.ClockNum = 204800 - ((8*80) + hAif->linkConfig[hAif->dioConfig[0].firstLink].piMin + 20 + 180);
	} else {
		UlRadTimerInit.ClockNum = 163840 - ((8*64) + hAif->linkConfig[hAif->dioConfig[0].firstLink].piMin + 20 + 160);
	}

	AtCommonSetup.AtTerminalCount.pPhyTimerTc = &PhyTimerTc;
	PhyTimerTc.FrameLsbNum  = AIF2_FRAME_COUNT_TC_PHY_TIMER;//set phy Frame TC to 4095
	if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
		PhyTimerTc.ClockNum = AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER; //set phy clock TC to 3071999=10ms
	} else {
		PhyTimerTc.ClockNum = AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER; //set phy clock TC to 2457600 - 1
	}

	AtCommonSetup.AtTerminalCount.pRadTimerTc = &RadTimerTc;
	if ((hAif->mode == AIF_WCDMA_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)) {
			RadTimerTc.FrameLsbNum  = AIF2_FRAME_COUNT_TC_WCDMA_FDD;//set WCDMA Frame TC to 4095
			RadTimerTc.SymbolNum    = AIF2_SLOT_COUNT_TC_WCDMA_FDD; //set WCDMA Slot TC to 14
	}
	if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)) {
		if (hAif->radTimerConfig[0].userSpecified == false) {
			RadTimerTc.FrameLsbNum  = AIF2_FRAME_COUNT_TC_WCDMA_FDD;//set LTE Frame TC to 4095
			if (lte1_4 == 1){
				//RadTimerTc.ClockNum = AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER;
				RadTimerTc.SymbolNum    = 20 -1; //set LTE 20 slot in 10ms
				RadTimerTc.LutIndexNum 	= 0; //set LTE LutIndex TC
			} else {
				/*if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
					RadTimerTc.ClockNum = AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;
				else
					RadTimerTc.ClockNum = AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER;*/
				RadTimerTc.SymbolNum    = 140 -1; //set LTE 10 subframes in 10ms
				RadTimerTc.LutIndexNum 	= AIF2_LTE_SYMBOL_NUM - 1; //set LTE LutIndex TC
			}
		} else {
			// user defined init values
			RadTimerTc.FrameLsbNum  = hAif->radTimerConfig[0].frameTerminalCount - 1;
			if (hAif->radTimerConfig[0].lte.cpType == AIF2_LTE_CPTYPE_NORMAL) {
				RadTimerTc.LutIndexNum 	= AIF2_LTE_SYMBOL_NUM - 1;
			} else {
				RadTimerTc.LutIndexNum 	= AIF2_LTE_SYMBOL_NUM_EXT_CP - 1;
			}
			RadTimerTc.SymbolNum = hAif->radTimerConfig[0].lte.numSymbolStrobesForFrameStrobe - 1;
		}
	}
	if ((hAif->mode == AIF_GENERICPACKET_MODE)) {
	    RadTimerTc.FrameLsbNum = AIF2_FRAME_COUNT_TC_WCDMA_FDD;//set Frame TC to 4095
	    RadTimerTc.SymbolNum = 9; //set Symbol TC to 9  -- need to set a definition in AIF_defs.h
	    RadTimerTc.LutIndexNum = 0; //set LutIndex TC to 0
	}

	// Event 7 - 10 ms tick from PHYT
	AtEventSetup.AtRadEvent[7].EventSelect     = AIF2FL_EVENT_7;
	AtEventSetup.AtRadEvent[7].EventOffset     = 0;
	if ((hAif->mode == AIF_GENERICPACKET_MODE)) AtEventSetup.AtRadEvent[7].EvtStrobeSel = AIF2FL_RADT_FRAME;
	else AtEventSetup.AtRadEvent[7].EvtStrobeSel    = AIF2FL_PHYT_FRAME;
	if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
		AtEventSetup.AtRadEvent[7].EventModulo = AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;
	else
		AtEventSetup.AtRadEvent[7].EventModulo = AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER + 1;
	AtEventSetup.AtRadEvent[7].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
	AtEventSetup.AtRadEvent[7].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
	AtEventSetup.bEnableRadEvent[7] = true;//Enable Event

	if (hAif->mode == AIF_WCDMA_MODE)
	{
	   //AT Event setup for generating 4 chip trigger for TAC (Event 8 is specified for this purpose) WCDMA OBSAI DL
	   AtEventSetup.AtRadEvent[8].EventSelect  = AIF2FL_EVENT_8;
	   AtEventSetup.AtRadEvent[8].EventOffset  = 0;
	   AtEventSetup.AtRadEvent[8].EvtStrobeSel = AIF2FL_RADT_FRAME;
	   if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
		   AtEventSetup.AtRadEvent[8].EventModulo  = 319; //set Modulus count (WCDMA 4 chip time)
	   else
		   AtEventSetup.AtRadEvent[8].EventModulo  	= 255; //set Modulus count (WCDMA 4 chip time)
	   AtEventSetup.AtRadEvent[8].EventMaskLsb 		= 0xFFFFFFFF;
	   AtEventSetup.AtRadEvent[8].EventMaskMsb 		= 0xFFFFFFFF;
	   AtEventSetup.bEnableRadEvent[8] = true;//Enable Event

	   //AT Event setup to generate 32 chip trigger for RAC_A (Event 9) WCDMA  UL
	   AtEventSetup.AtRadEvent[9].EventSelect 	= AIF2FL_EVENT_9;
	   AtEventSetup.AtRadEvent[9].EvtStrobeSel 	= AIF2FL_ULRADT_FRAME; //First Ul frame event will occurr after 4480 clocks
	   if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) {
	      AtEventSetup.AtRadEvent[9].EventModulo    = 2559; //set Modulus count for event 1 (WCDMA 32 chip time)
		  AtEventSetup.AtRadEvent[9].EventOffset 	= 31 * 80;
	   } else {
	      AtEventSetup.AtRadEvent[9].EventModulo 	= 2047; //set Modulus count for event 1 (WCDMA 32 chip time)
		  AtEventSetup.AtRadEvent[9].EventOffset 	= 31 * 64;
	   }
	   AtEventSetup.AtRadEvent[9].EventMaskLsb 		= 0xFFFFFFFF; //set all mask to for WCDMA FDD
	   AtEventSetup.AtRadEvent[9].EventMaskMsb 		= 0xFFFFFFFF; //set all mask to for WCDMA FDD
	   AtEventSetup.bEnableRadEvent[9] = true;//Enable Event*/

	   //AT Event setup to generate 32 chip trigger for RAC_B (Event 10) WCDMA UL
	   AtEventSetup.AtRadEvent[10].EventSelect 	= AIF2FL_EVENT_10;
	   AtEventSetup.AtRadEvent[10].EventOffset 	= 0;
	   AtEventSetup.AtRadEvent[10].EvtStrobeSel = AIF2FL_ULRADT_FRAME;
	   if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) {
	      AtEventSetup.AtRadEvent[10].EventModulo    = 2559; //set Modulus count for event 1 (WCDMA 32 chip time)
		  AtEventSetup.AtRadEvent[10].EventOffset 	= 31 * 80;
	   } else {
	      AtEventSetup.AtRadEvent[10].EventModulo 	= 2047; //set Modulus count for event 1 (WCDMA 32 chip time)
		  AtEventSetup.AtRadEvent[10].EventOffset 	= 31 * 64;
	   }
	   AtEventSetup.AtRadEvent[10].EventMaskLsb 	= 0xFFFFFFFF; //set all mask to for WCDMA FDD
	   AtEventSetup.AtRadEvent[10].EventMaskMsb 	= 0xFFFFFFFF; //set all mask to for WCDMA FDD
	   AtEventSetup.bEnableRadEvent[10] = true;//Enable Event*/
#ifdef K2 // support K-II devices with up to 4 RAC
	   //AT Event setup to generate 32 chip trigger for RAC_C (Event 11) WCDMA  UL
	   AtEventSetup.AtRadEvent[11].EventSelect 	= AIF2FL_EVENT_11;
	   AtEventSetup.AtRadEvent[11].EvtStrobeSel 	= AIF2FL_ULRADT_FRAME; //First Ul frame event will occurr after 4480 clocks
	   if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) {
	      AtEventSetup.AtRadEvent[11].EventModulo    = 2559; //set Modulus count for event 1 (WCDMA 32 chip time)
		  AtEventSetup.AtRadEvent[11].EventOffset 	= 31 * 80;
	   } else {
	      AtEventSetup.AtRadEvent[11].EventModulo 	= 2047; //set Modulus count for event 1 (WCDMA 32 chip time)
		  AtEventSetup.AtRadEvent[11].EventOffset 	= 31 * 64;
	   }
	   AtEventSetup.AtRadEvent[11].EventMaskLsb 		= 0xFFFFFFFF; //set all mask to for WCDMA FDD
	   AtEventSetup.AtRadEvent[11].EventMaskMsb 		= 0xFFFFFFFF; //set all mask to for WCDMA FDD
	   AtEventSetup.bEnableRadEvent[11] = true;//Enable Event*/

	   //AT Event setup to generate 32 chip trigger for RAC_D (Event 12) WCDMA UL
	   AtEventSetup.AtRadEvent[12].EventSelect 	= AIF2FL_EVENT_12;
	   AtEventSetup.AtRadEvent[12].EventOffset 	= 0;
	   AtEventSetup.AtRadEvent[12].EvtStrobeSel = AIF2FL_ULRADT_FRAME;
	   if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) {
	      AtEventSetup.AtRadEvent[12].EventModulo    = 2559; //set Modulus count for event 1 (WCDMA 32 chip time)
		  AtEventSetup.AtRadEvent[12].EventOffset 	= 31 * 80;
	   } else {
	      AtEventSetup.AtRadEvent[12].EventModulo 	= 2047; //set Modulus count for event 1 (WCDMA 32 chip time)
		  AtEventSetup.AtRadEvent[12].EventOffset 	= 31 * 64;
	   }
	   AtEventSetup.AtRadEvent[12].EventMaskLsb 	= 0xFFFFFFFF; //set all mask to for WCDMA FDD
	   AtEventSetup.AtRadEvent[12].EventMaskMsb 	= 0xFFFFFFFF; //set all mask to for WCDMA FDD
	   AtEventSetup.bEnableRadEvent[12] = true;//Enable Event*/
#endif
	}

	if (hAif->mode == AIF_LTE_FDD_MODE) {
		if(lte1_4 == 1)
		{
			AtEventSetup.AtRadEvent[5].EventSelect	= AIF2FL_EVENT_5;
			AtEventSetup.AtRadEvent[5].EventOffset  = 800;
			AtEventSetup.AtRadEvent[5].EvtStrobeSel = AIF2FL_RADT_FRAME;
			AtEventSetup.AtRadEvent[5].EventModulo 	= 122879;	//LTE 0.5ms slot time
			AtEventSetup.AtRadEvent[5].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
			AtEventSetup.AtRadEvent[5].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
			AtEventSetup.bEnableRadEvent[5] = true;//Enable Event
		} else {
			// event for superpacket post processing with EDMA
			AtEventSetup.AtRadEvent[0].EventSelect	= AIF2FL_EVENT_0;
			AtEventSetup.AtRadEvent[0].EventOffset  = 0;
			AtEventSetup.AtRadEvent[0].EvtStrobeSel = AIF2FL_ULRADT_FRAME;
			AtEventSetup.AtRadEvent[0].EventModulo 	= 0xFFFFFFFF;
			AtEventSetup.AtRadEvent[0].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
			AtEventSetup.AtRadEvent[0].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
			AtEventSetup.bEnableRadEvent[0] = true;//Enable Event

			// Event for timeSlot event generation
			AtEventSetup.AtRadEvent[5].EventSelect	= AIF2FL_EVENT_5;
			AtEventSetup.AtRadEvent[5].EventOffset  = 800; //98304; //800
			AtEventSetup.AtRadEvent[5].EvtStrobeSel = AIF2FL_RADT_FRAME;
			if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
				AtEventSetup.AtRadEvent[5].EventModulo 	= 153599; //LTE 0.5ms slot time
			else
				AtEventSetup.AtRadEvent[5].EventModulo 	= 122879; //LTE 0.5ms slot time
			AtEventSetup.AtRadEvent[5].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
			AtEventSetup.AtRadEvent[5].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
			AtEventSetup.bEnableRadEvent[5] = true;//Enable Event


			// Event for SubFrame event generation
			AtEventSetup.AtRadEvent[6].EventSelect	= AIF2FL_EVENT_6;
			AtEventSetup.AtRadEvent[6].EventOffset  = 800;
			//AtEventSetup.AtRadEvent[6].EventOffset  = AIF2_CLOCK_COUNT_TC_CPRI - (AIF2_CLOCK_COUNT_TC_CPRI / 10); // TEST
			AtEventSetup.AtRadEvent[6].EvtStrobeSel = AIF2FL_RADT_FRAME;
			if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
				AtEventSetup.AtRadEvent[6].EventModulo 	= 307199;//LTE 1ms sub-frame time
			else
				AtEventSetup.AtRadEvent[6].EventModulo 	= AIF2_CLOCK_COUNT_TC_CPRI;//LTE 1ms sub-frame time
			AtEventSetup.AtRadEvent[6].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
			AtEventSetup.AtRadEvent[6].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
			AtEventSetup.bEnableRadEvent[6] = true;//Enable Event
		}

	}

	if (hAif->mode == AIF_LTE_TDD_MODE)
	{
		// Event for timeSlot event generation
		AtEventSetup.AtRadEvent[5].EventSelect	= AIF2FL_EVENT_5;
		AtEventSetup.AtRadEvent[5].EventOffset  = 800; //98304; //800
		AtEventSetup.AtRadEvent[5].EvtStrobeSel = AIF2FL_RADT_FRAME;
		if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
			AtEventSetup.AtRadEvent[5].EventModulo 	= 153599; //LTE 0.5ms slot time
		else
			AtEventSetup.AtRadEvent[5].EventModulo 	= 122879; //LTE 0.5ms slot time
		AtEventSetup.AtRadEvent[5].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
		AtEventSetup.AtRadEvent[5].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
		AtEventSetup.bEnableRadEvent[5] = true;//Enable Event


		// Event for Symbol event generation
		AtEventSetup.AtRadEvent[6].EventSelect	= AIF2FL_EVENT_6;
		AtEventSetup.AtRadEvent[6].EventOffset  = 800;  //3128;
		AtEventSetup.AtRadEvent[6].EvtStrobeSel = AIF2FL_RADT_SYMBOL;
		AtEventSetup.AtRadEvent[6].EventModulo 	= 0x3FFFFF;   //17535;//LTE symbol time
		AtEventSetup.AtRadEvent[6].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
		AtEventSetup.AtRadEvent[6].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
		AtEventSetup.bEnableRadEvent[6] = true;//Enable Event
	}

	if (hAif->mode == AIF_LTE_WCDMA_MODE)
	{

		// Event for timeSlot event generation
		AtEventSetup.AtRadEvent[5].EventSelect	= AIF2FL_EVENT_5;
		AtEventSetup.AtRadEvent[5].EventOffset  = 800; //98304; //800
		AtEventSetup.AtRadEvent[5].EvtStrobeSel = AIF2FL_RADT_FRAME;
		AtEventSetup.AtRadEvent[5].EventModulo 	= 122879; //LTE 0.5ms slot time
		AtEventSetup.AtRadEvent[5].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
		AtEventSetup.AtRadEvent[5].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
		AtEventSetup.bEnableRadEvent[5] = true;//Enable Event

		//Configure and enable AIF2 Event 6 for Edma process for dual mode
		AtEventSetup.AtRadEvent[6].EventSelect	= AIF2FL_EVENT_6;
		AtEventSetup.AtRadEvent[6].EventOffset  = 11000; //98304; //800
		AtEventSetup.AtRadEvent[6].EvtStrobeSel = AIF2FL_RADT_FRAME;
		AtEventSetup.AtRadEvent[6].EventModulo 	= 17535; //the longest symbol
		AtEventSetup.AtRadEvent[6].EventMaskLsb = 0xFFFFFFFF; // only for GSM so NA
		AtEventSetup.AtRadEvent[6].EventMaskMsb = 0xFFFFFFFF; // only for GSM so NA
		AtEventSetup.bEnableRadEvent[6] = true;//Enable Event
	}

	//AT Event setup for DIO engines
	if (hAif->pktdmaOrDioEngine == AIF2FL_DIO)
	{
		for (i=0; i<AIF2_MAX_NUM_DIO_ENGINE; i++)
		{
			if(hAif->dioConfig[i].sampleRate == AIF_SRATE_30P72MHZ)
				samplePerFrame = 307200;
			else if(hAif->dioConfig[i].sampleRate == AIF_SRATE_15P36MHZ)
				samplePerFrame = 153600;
			else if(hAif->dioConfig[i].sampleRate == AIF_SRATE_7P68MHZ)
				samplePerFrame = 76800;
			else
				samplePerFrame = 38400;
			if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
				if (hAif->dioConfig[i].mode == LTE)
				{
					EventModulo = (AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*16; //2 chip event for LTE
//					DioFrameEventOffset = 840;//Pi Max(350) + 2 WCDMA chip time(130) + PD delay and fuzzy factors(180)
					DioFrameEventOffset = EventModulo + hAif->linkConfig[hAif->dioConfig[i].firstLink].piMin + 20 + 180 ;
				} else {
					if (hAif->linkConfig[hAif->dioConfig[i].firstLink].inboundDataType==AIF2FL_LINK_DATA_TYPE_RSA){
						EventModulo = (AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*8; //Modulus count for In DIO event (WCDMA 8 chip time) OBSAI
					} else {
						EventModulo = (AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*4; //Modulus count for In DIO event (WCDMA 4 chip time) OBSAI
					}
					//DioFrameEventOffset = EventModulo + hAif->linkConfig[hAif->dioConfig[i].firstLink].pe2Offset + 60 + 20 + 180 ;//1190;//Pi Max + 4 WCDMA chip time + PD delay and fuzzy factors  for ingress DMA timing
					DioFrameEventOffset = EventModulo + hAif->linkConfig[hAif->dioConfig[i].firstLink].piMin + 20 + 180 ;
				}
			} else {
				if (hAif->dioConfig[i].mode == LTE)
				{
					EventModulo = (AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*16; //2 chip event for LTE
//					DioFrameEventOffset = 660;//Pi Max(350) + 2 WCDMA chip time(130) + PD delay and fuzzy factors(180)
					DioFrameEventOffset = EventModulo + hAif->linkConfig[hAif->dioConfig[i].firstLink].piMin + 20 + 160 ;
				} else {
					if (hAif->linkConfig[hAif->dioConfig[i].firstLink].inboundDataType==AIF2FL_LINK_DATA_TYPE_RSA){
						EventModulo = (AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*8; //(8 chip * 64 CPRI clock -1) set Modulus count for In DIO event 0 (WCDMA 8 chip time)
					} else {
						EventModulo = (AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*4; //(4 chip * 64 CPRI clock -1) set Modulus count for In DIO event 0 (WCDMA 4 chip time)
					}
					DioFrameEventOffset = EventModulo + hAif->linkConfig[hAif->dioConfig[i].firstLink].piMin + 20 + 160 ;
				}
			}
			if (hAif->dioConfig[i].numLink != 0)
			{
				AtEventSetup.AtIngrDioEvent[i].EventSelect = (Aif2Fl_AtEventIndex) (AIF2FL_IN_DIO_EVENT_0+i);//(Aif2Fl_AtEventIndex) (AIF2FL_IN_DIO_EVENT_0+i);//Select In DIO Event 1
				AtEventSetup.AtIngrDioEvent[i].EventOffset = 0;//fine offset value
				AtEventSetup.AtIngrDioEvent[i].EvtStrobeSel = AIF2FL_RADT_FRAME;
				AtEventSetup.AtIngrDioEvent[i].EventModulo = EventModulo  - 1;// set Modulus count for In DIO event (WCDMA 4 chip time)
				AtEventSetup.AtIngrDioEvent[i].DioFrameEventOffset = DioFrameEventOffset;
				AtEventSetup.AtIngrDioEvent[i].DioFrameStrobeSel = AIF2FL_RADT_FRAME; //frame event strobe selection
				if (hAif->dioConfig[i].numPdDBCH) AtEventSetup.bEnableIngrDioEvent[i] = true;//Enable In DIO Event 1 L2/RSA
			}

			if (AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol){
				if (hAif->dioConfig[i].mode == LTE)
				{
					EventModulo = (AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*16; //2 chip event for LTE
					DioFrameEventOffset = 0;
				} else {
					if (hAif->linkConfig[hAif->dioConfig[i].firstLink].outboundDataType==AIF2FL_LINK_DATA_TYPE_RSA){
						EventModulo = (AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*8; //Modulus count for In DIO event (WCDMA 8 chip time) OBSAI
					} else {
						EventModulo = (AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*4; //Modulus count for In DIO event (WCDMA 4 chip time) OBSAI
					}
					DioFrameEventOffset = 0;
				}
			} else {
				if (hAif->dioConfig[i].mode == LTE)
				{
					EventModulo = (AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*16; //2 chip event for LTE
					DioFrameEventOffset = 0;
				} else {
					if (hAif->linkConfig[hAif->dioConfig[i].firstLink].outboundDataType==AIF2FL_LINK_DATA_TYPE_RSA){
						EventModulo = (AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*8; //(8 chip * 64 CPRI clock -1) set Modulus count for In DIO event 0 (WCDMA 8 chip time)
					} else {
						EventModulo = (AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1)/samplePerFrame*4; //(4 chip * 64 CPRI clock -1) set Modulus count for In DIO event 0 (WCDMA 4 chip time)
					}
					DioFrameEventOffset = 0;
				}
			}

			if ((hAif->dioConfig[i].numLink != 0) && (hAif->linkConfig[i].RtEnabled==0))
			{
				AtEventSetup.AtEgrDioEvent[i].EventSelect = (Aif2Fl_AtEventIndex) (AIF2FL_E_DIO_EVENT_0+i); //Select E DIO Event 1
				AtEventSetup.AtEgrDioEvent[i].EventOffset = 0;//300;//delay for TAC operation time
				AtEventSetup.AtEgrDioEvent[i].EvtStrobeSel = AIF2FL_RADT_FRAME;
				AtEventSetup.AtEgrDioEvent[i].EventModulo = EventModulo - 1;//set Modulus count for E DIO event (WCDMA 4 chip time)
				AtEventSetup.AtEgrDioEvent[i].DioFrameEventOffset = DioFrameEventOffset;
				AtEventSetup.AtEgrDioEvent[i].DioFrameStrobeSel = AIF2FL_RADT_FRAME; //frame event strobe selection
				if (hAif->dioConfig[i].numPeDBCH) AtEventSetup.bEnableEgrDioEvent[i] = true;//Enable E DIO Event 1  L2/RSA
//#ifdef K2
//				AtEventSetup.AtEgrDioEvent[i].EvtStrobeSel = AIF2FL_RADT_SYMBOL;
//				AtEventSetup.AtEgrDioEvent[i].EventOffset = 230;//fine offset value
//#endif
			}
			if (hAif->dioConfig[i].duplicateOnDioEng2)
			{
				AtEventSetup.AtIngrDioEvent[2]=AtEventSetup.AtIngrDioEvent[i];
				AtEventSetup.AtEgrDioEvent[2]=AtEventSetup.AtEgrDioEvent[i];
				AtEventSetup.AtIngrDioEvent[2].EventSelect = (Aif2Fl_AtEventIndex) (AIF2FL_IN_DIO_EVENT_2);
				AtEventSetup.AtEgrDioEvent[2].EventSelect = (Aif2Fl_AtEventIndex) (AIF2FL_E_DIO_EVENT_2);
				AtEventSetup.bEnableIngrDioEvent[2] = true;//Enable E DIO Event 1  L2/RSA
				AtEventSetup.bEnableEgrDioEvent[2] = true;//Enable E DIO Event 1  L2/RSA
			}
		}
	}

} //eof AIF_initHw

/* Start AIF given this user configuration*/
#ifdef K2
static void AIF_serdesConfig(AIF_ConfigHandle  hAif, uint8_t enableB8, uint8_t enableB4);
#endif
void AIF_startHw(
		AIF_ConfigHandle  hAif
)
{
  uint16_t       ctrlArg;
  int32_t  i;
#ifndef K2
  uint16_t response =0;
#endif
  uint8_t  serdes_blockb8_used =0;
  uint8_t  serdes_blockb4_used =0;
#ifdef _TMS320C6X
  Cppi_RxChInitCfg     dioRxCfg;
  Cppi_TxChInitCfg     dioTxCfg;
  uint8_t                isAllocated;
#else
  // used for DIO mode only
  CSL_Cppidma_global_configRegs* pktDMAGlobalCfg = (CSL_Cppidma_global_configRegs*)((uint32_t)(hAif->hAif2SerDesBaseAddr->dev.bases[0].cfgBase) + 0x14000);
  CSL_Cppidma_rx_channel_configRegs* pktDMARxCfg = (CSL_Cppidma_rx_channel_configRegs*)((uint32_t)(hAif->hAif2SerDesBaseAddr->dev.bases[0].cfgBase) + 0x18000);
  CSL_Cppidma_tx_channel_configRegs* pktDMATxCfg = (CSL_Cppidma_tx_channel_configRegs*)((uint32_t)(hAif->hAif2SerDesBaseAddr->dev.bases[0].cfgBase) + 0x16000);
#endif
  Aif2Fl_Status 			status;

   // Identify which SerDes block to use
	for(i=AIF2FL_LINK_0; i<AIF2FL_LINK_4; i++)
	{
		// moved to link setup
		if(1==hAif->linkConfig[i].linkEnable) serdes_blockb8_used=1;
	}

	for(i=AIF2FL_LINK_4; i<(AIF2FL_LINK_5 + 1); i++)
	{
		// moved to link setup
		if(1==hAif->linkConfig[i].linkEnable) serdes_blockb4_used=1;
	}
#ifdef K2
   AIF_serdesConfig(hAif, serdes_blockb8_used, serdes_blockb4_used);
#endif      

   /****** Do AIF2 HW setup (set all MMRs above) **********************************************************************/
   status = Aif2Fl_hwSetup(hAif->hFl, hAif->hAif2Setup);

   if(status != AIF2FL_SOK)
	   Aif2_osalLog( "hwSetup structure is not initialized \n");

    ctrlArg = true;
	for(i= 0; i< AIF_MAX_NUM_LINKS; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
		{

			hAif->hFl->arg_link = (Aif2Fl_LinkIndex)i;
#ifndef K2
			if (( serdes_blockb8_used ==1)&& (i<4)) while(response == 0)
			{ // wait until SD PLL is locked
				if(AIF2FL_SOK != Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_SD_B8_PLL_LOCK, (void *)&response))
					Aif2_osalLog ("unable to get Hw status \n");
			}
			response = 0;
			if (( serdes_blockb4_used ==1)&& (i>=4)) while(response == 0) 
			{
				if( AIF2FL_SOK != Aif2Fl_getHwStatus(hAif->hFl, AIF2FL_QUERY_SD_B4_PLL_LOCK, (void *)&response))
					Aif2_osalLog ("unable to get Hw status \n");
			}

			if (hAif->linkConfig[i].comType == AIF2_LOOPBACK) {       //setup links for internal loopback mode
				ctrlArg = true;
				if (AIF2FL_SOK != Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_ENABLE_DISABLE_LINK_LOOPBACK, (void *)&ctrlArg))
					Aif2_osalLog ("aif2 enable link loopback fail \n");
			}
			else {
				ctrlArg = false;
				if (AIF2FL_SOK != Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_ENABLE_DISABLE_LINK_LOOPBACK, (void *)&ctrlArg))
				Aif2_osalLog ("aif2 disable link loopback fail \n");
			}
#endif
			ctrlArg = true;
			if (AIF2FL_SOK != Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_ENABLE_DISABLE_TX_LINK, (void *)&ctrlArg))
				Aif2_osalLog ("aif2 enable Tx link fail \n");
			if (AIF2FL_SOK != Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_ENABLE_DISABLE_RX_LINK, (void *)&ctrlArg))
				Aif2_osalLog ("aif2 enable Rx link fail \n");
		}
	}	

	// Disable loopback in PKT DMA (used also for DIO mode) - done in AIF_initPktDma()
	if (hAif->pktdmaOrDioEngine == AIF2FL_DIO)
	{
#ifdef _TMS320C6X
	    // Enable channel 128 for DIO mode
		// enable Rx and Tx for DIO
		memset(&dioRxCfg, 0, sizeof(dioRxCfg));
		dioRxCfg.channelNum = 128;
		dioRxCfg.rxEnable   = Cppi_ChState_CHANNEL_DISABLE;
		hAif->pktDmaConfig.dioRxChAxC  = Cppi_rxChannelOpen(hAif->pktDmaConfig.hCppi, &dioRxCfg, &isAllocated);
        if (hAif->pktDmaConfig.dioRxChAxC == NULL)
		{
			Aif2_osalLog("Error: Opening AIF2 Rx channel %d failed\n", dioRxCfg.channelNum);
		}
		memset(&dioTxCfg, 0, sizeof(dioTxCfg));
		dioTxCfg.channelNum   = 128;
		dioTxCfg.txEnable     = Cppi_ChState_CHANNEL_DISABLE;
		hAif->pktDmaConfig.dioTxChAxC  = Cppi_txChannelOpen(hAif->pktDmaConfig.hCppi, &dioTxCfg, &isAllocated);
        if (hAif->pktDmaConfig.dioTxChAxC == NULL)
		{
			Aif2_osalLog("Error: Opening AIF2 Tx channel %d failed\n", dioTxCfg.channelNum);
		}
		Cppi_channelEnable (hAif->pktDmaConfig.dioTxChAxC);
		Cppi_channelEnable (hAif->pktDmaConfig.dioRxChAxC);
#else
		// Enable channel 128 for DIO mode
		// disable loopback
	    CSL_FINS (pktDMAGlobalCfg->EMULATION_CONTROL_REG,CPPIDMA_GLOBAL_CONFIG_EMULATION_CONTROL_REG_LOOPBACK_EN, 0);
		//set write arbitration FIFO depth to 8
		CSL_FINS(pktDMAGlobalCfg->PERF_CONTROL_REG, CPPIDMA_GLOBAL_CONFIG_PERF_CONTROL_REG_WARB_FIFO_DEPTH, 8);
		// enable Tx for DIO
		CSL_FINS(pktDMATxCfg->TX_CHANNEL_GLOBAL_CONFIG[128].TX_CHANNEL_GLOBAL_CONFIG_REG_A, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE, (uint32_t)1);
		// enable Rx for DIO
		CSL_FINS(pktDMARxCfg->RX_CHANNEL_GLOBAL_CONFIG[128].RX_CHANNEL_GLOBAL_CONFIG_REG, CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE, (uint32_t)1);
#endif
		
	}


	/* Initialize synchronization counters for event tracking */
	AIF_initFsync(hAif);

	ctrlArg = true;
	//AT Arm timer
	if (AIF2FL_SOK != Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_AT_ARM_TIMER, (void *)&ctrlArg))
		Aif2_osalLog("aif2 AT arm timer fail \n");

	Aif2_osalMulticoreSyncBarrier();

} //eof AIF_startHw

void
AIF_configureAtEvent(
		Aif2Fl_AtEvent* hAtEvent
)
{
 uint32_t evtNum = (uint32_t)hAtEvent->EventSelect;

    AtEventSetup.AtRadEvent[evtNum].EventSelect 	= hAtEvent->EventSelect;
    AtEventSetup.AtRadEvent[evtNum].EventOffset 	= hAtEvent->EventOffset;
    AtEventSetup.AtRadEvent[evtNum].EvtStrobeSel    = hAtEvent->EvtStrobeSel;
    AtEventSetup.AtRadEvent[evtNum].EventModulo 	= hAtEvent->EventModulo;
    AtEventSetup.AtRadEvent[evtNum].EventMaskLsb    = hAtEvent->EventMaskLsb;
    AtEventSetup.AtRadEvent[evtNum].EventMaskMsb    = hAtEvent->EventMaskMsb;
}

void AIF_enableAtEvent(
		Aif2Fl_AtEventIndex event
)
{ 
    AtEventSetup.bEnableRadEvent[(uint32_t)event] = true;
}

void AIF_disableAtEvent(
		Aif2Fl_AtEventIndex event
)
{
    AtEventSetup.bEnableRadEvent[(uint32_t)event] = false;
}

void
AIF_configureEgrDioEvent(
		Aif2Fl_AtEvent* hAtEvent,
		uint32_t           dioNum

)
{
	AtEventSetup.AtEgrDioEvent[dioNum].EventSelect         = hAtEvent->EventSelect;     //Select E DIO Event (for instance AIF2FL_E_DIO_EVENT_0)
	AtEventSetup.AtEgrDioEvent[dioNum].EventOffset         = hAtEvent->EventOffset;
	AtEventSetup.AtEgrDioEvent[dioNum].EvtStrobeSel        = hAtEvent->EvtStrobeSel;
	AtEventSetup.AtEgrDioEvent[dioNum].EventModulo         = hAtEvent->EventModulo;
	AtEventSetup.AtEgrDioEvent[dioNum].DioFrameEventOffset = hAtEvent->DioFrameEventOffset;
	AtEventSetup.AtEgrDioEvent[dioNum].DioFrameStrobeSel   = hAtEvent->DioFrameStrobeSel;
}

void AIF_enableEgrDioEvent(
		uint32_t           dioNum
)
{
	AtEventSetup.bEnableEgrDioEvent[dioNum] = true;
}

void AIF_disableEgrDioEvent(
		uint32_t           dioNum
)
{
	AtEventSetup.bEnableEgrDioEvent[dioNum] = false;
}

void
AIF_configureIngrDioEvent(
		Aif2Fl_AtEvent* hAtEvent,
		uint32_t           dioNum

)
{
	AtEventSetup.AtIngrDioEvent[dioNum].EventSelect         = hAtEvent->EventSelect;     //Select E DIO Event (for instance AIF2FL_E_DIO_EVENT_0)
	AtEventSetup.AtIngrDioEvent[dioNum].EventOffset         = hAtEvent->EventOffset;
	AtEventSetup.AtIngrDioEvent[dioNum].EvtStrobeSel        = hAtEvent->EvtStrobeSel;
	AtEventSetup.AtIngrDioEvent[dioNum].EventModulo         = hAtEvent->EventModulo;
	AtEventSetup.AtIngrDioEvent[dioNum].DioFrameEventOffset = hAtEvent->DioFrameEventOffset;
	AtEventSetup.AtIngrDioEvent[dioNum].DioFrameStrobeSel   = hAtEvent->DioFrameStrobeSel;
}

void AIF_enableIngrDioEvent(
		uint32_t           dioNum
)
{
	AtEventSetup.bEnableIngrDioEvent[dioNum] = true;
}

void AIF_disableIngrDioEvent(
		uint32_t           dioNum
)
{
	AtEventSetup.bEnableIngrDioEvent[dioNum] = false;
}

void
AIF_configureEgrDioEngine(
		Aif2Fl_AdDioEngine* hEgrDioEngine,
		uint32_t               dioNum
)
{
uint32_t k;
	AdDioSetup.EgrDioEngineEnable[dioNum]          = true;
	AdDioSetup.EgrDioEngine[dioNum].BcnTableSelect = hEgrDioEngine->BcnTableSelect;
	AdDioSetup.EgrDioEngine[dioNum].NumAxC         = hEgrDioEngine->NumAxC;
	AdDioSetup.EgrDioEngine[dioNum].DmaNumBlock    = hEgrDioEngine->DmaNumBlock;
	AdDioSetup.EgrDioEngine[dioNum].NumQuadWord        = hEgrDioEngine->NumQuadWord;
	AdDioSetup.EgrDioEngine[dioNum].bEnEgressRsaFormat = hEgrDioEngine->bEnEgressRsaFormat;
	AdDioSetup.EgrDioEngine[dioNum].DmaBlockAddrStride = hEgrDioEngine->DmaBlockAddrStride;
	AdDioSetup.EgrDioEngine[dioNum].bEnDmaChannel      = true; //Enable Dma channel
	AdDioSetup.EgrDioEngine[dioNum].DmaBurstLen        = hEgrDioEngine->DmaBurstLen;
	AdDioSetup.EgrDioEngine[dioNum].DmaBaseAddr        = hEgrDioEngine->DmaBaseAddr;
	AdDioSetup.EgrDioEngine[dioNum].DmaBurstAddrStride = hEgrDioEngine->DmaBurstAddrStride;
	for (k = 0; k < 64; k++)
	{
			AdDioSetup.EgrDioEngine[dioNum].DBCN[k] = hEgrDioEngine->DBCN[k]; //set egress table DBCN for channel 0
	}
}

void
AIF_configureIngrDioEngine(
		Aif2Fl_AdDioEngine* hIngrDioEngine,
		uint32_t               dioNum
)
{
uint32_t k;
	AdDioSetup.IngrDioEngineEnable[dioNum]          = true;
	AdDioSetup.IngrDioEngine[dioNum].BcnTableSelect = hIngrDioEngine->BcnTableSelect;
	AdDioSetup.IngrDioEngine[dioNum].NumAxC         = hIngrDioEngine->NumAxC;
	AdDioSetup.IngrDioEngine[dioNum].DmaNumBlock    = hIngrDioEngine->DmaNumBlock;
	AdDioSetup.IngrDioEngine[dioNum].NumQuadWord        = hIngrDioEngine->NumQuadWord;
#ifdef K2
	AdDioSetup.IngrDioEngine[dioNum].bEnIngressRsaFormat = hIngrDioEngine->bEnIngressRsaFormat;
#endif
	AdDioSetup.IngrDioEngine[dioNum].DmaBlockAddrStride = hIngrDioEngine->DmaBlockAddrStride;
	AdDioSetup.IngrDioEngine[dioNum].bEnDmaChannel      = true; //Enable Dma channel
	AdDioSetup.IngrDioEngine[dioNum].DmaBurstLen        = hIngrDioEngine->DmaBurstLen;
	AdDioSetup.IngrDioEngine[dioNum].DmaBaseAddr        = hIngrDioEngine->DmaBaseAddr;
	AdDioSetup.IngrDioEngine[dioNum].DmaBurstAddrStride = hIngrDioEngine->DmaBurstAddrStride;
	for (k = 0; k < 64; k++)
	{
			AdDioSetup.IngrDioEngine[dioNum].DBCN[k] = hIngrDioEngine->DBCN[k]; //set ingress table DBCN for channel 0
	}
}

void
AIF_setRadTimerTc(
		uint32_t clockNum,
		uint8_t symbolNum,
		uint32_t frameLsbNum,
		uint8_t lutIndexNum,
		uint32_t *radClockCountTc
)
{
	uint32_t i;

	RadTimerTc.ClockNum = clockNum;
	RadTimerTc.SymbolNum = symbolNum;
	RadTimerTc.FrameLsbNum = frameLsbNum;
	RadTimerTc.LutIndexNum = lutIndexNum;
	for (i=0;i<=lutIndexNum;i++)
	{
		AtCommonSetup.AtTerminalCount.RadClockCountTc[i] = radClockCountTc[i];
	}
}

void
AIF_setPhyTimerInit(
   uint32_t       ClockNum,
   uint32_t       FrameLsbNum,
   uint32_t       FrameMsbNum
)
{
	PhyTimerInit.ClockNum    = ClockNum;
	PhyTimerInit.FrameLsbNum = FrameLsbNum;
	PhyTimerInit.FrameMsbNum = FrameMsbNum;
}

void
AIF_setUlRadTimerInit(
   uint32_t       SymbolNum,
   uint32_t       ClockNum,
   uint32_t       FrameLsbNum,
   uint32_t       FrameMsbNum,
   uint16_t         FcbMinusOne
)
{
	UlRadTimerInit.ClockNum   = ClockNum;
	UlRadTimerInit.SymbolNum   = SymbolNum;
	UlRadTimerInit.FrameLsbNum = FrameLsbNum;
	UlRadTimerInit.FrameMsbNum = FrameMsbNum;
	UlRadTimerInit.FcbMinusOne = FcbMinusOne;
}

void
AIF_setDlRadTimerInit(
   uint32_t       SymbolNum,
   uint32_t       ClockNum,
   uint32_t       FrameLsbNum,
   uint32_t       FrameMsbNum,
   uint16_t         FcbMinusOne
)
{
	DlRadTimerInit.ClockNum   = ClockNum;
	DlRadTimerInit.SymbolNum   = SymbolNum;
	DlRadTimerInit.FrameLsbNum = FrameLsbNum;
	DlRadTimerInit.FrameMsbNum = FrameMsbNum;
	DlRadTimerInit.FcbMinusOne = FcbMinusOne;
}

void
AIF_setRmLinkSetupParams (
   int32_t                 link,
   Aif2Fl_RmFifoThold   RmFifoThold,
   uint16_t                  bEnableLcvControl,
   uint16_t                losDetThreshold,
   uint16_t                SyncThreshold,
   uint16_t                FrameSyncThreshold,
   uint16_t                UnsyncThreshold,
   uint16_t                FrameUnsyncThreshold
)
{
	RmLinkSetup[link].RmFifoThold = RmFifoThold; 
	RmLinkSetup[link].bEnableLcvControl = bEnableLcvControl;
	RmLinkSetup[link].losDetThreshold = losDetThreshold;
	RmLinkSetup[link].SyncThreshold = SyncThreshold;
	RmLinkSetup[link].FrameSyncThreshold = FrameSyncThreshold;
	RmLinkSetup[link].UnsyncThreshold = UnsyncThreshold;
	RmLinkSetup[link].FrameUnsyncThreshold = FrameUnsyncThreshold;
}

void
AIF_setLinkPiMax (
   int32_t                 link,
   uint32_t                piMax
)
{
	AtLinkSetup[link].PiMax = piMax;
}

void AIF_setPeFrameTC (
   int32_t index,
   Aif2Fl_FrameCounter *cfg
)
{
	PeCommonSetup.PeFrameTC[index].FrameIndexSc = cfg->FrameIndexSc;
	PeCommonSetup.PeFrameTC[index].FrameIndexTc = cfg->FrameIndexTc; 
	PeCommonSetup.PeFrameTC[index].FrameSymbolTc = cfg->FrameSymbolTc;
}   

void AIF_setPeFrameMsgTc (
    int32_t indx,
	uint16_t val
)
{
	PeCommonSetup.PeFrameMsgTc[indx] = val;
}

void AIF_setPdChDioOffset (
    int32_t indx,
	uint8_t val
)
{
	PdCommonSetup.PdChConfig1[indx].DioOffset = val;
}

uint32_t AIF2_getVersion (void)
{
    return AIF2_DRV_VERSION_ID;
}

const char   aif2LldVersionStr[] = AIF2_DRV_VERSION_STR ":" __DATE__  ":" __TIME__;

const char* AIF2_getVersionStr (void)
{
    return aif2LldVersionStr;
}

#ifdef K2

static uint32_t AIF2FL_SERDES_B4_CFG_REGS;
static uint32_t AIF2FL_SERDES_B8_CFG_REGS;
static uint32_t AIF2FL_CONTROL_REGS;

static void AIF_serdesRestoreDefault(
		uint32_t base_addr
)
{
	uint32_t i;

	CSL_SerDes_COMLANE_Restore_Default(base_addr);
	for(i=0; i < 4; i++)
	{
		CSL_SerDes_Lane_Restore_Default(base_addr, i);
	}
	CSL_SerDes_CMU_Restore_Default(base_addr);
}

static void AIF_serdesConfig(
		AIF_ConfigHandle  hAif,
		uint8_t enableB8,
		uint8_t enableB4
)
{
	uint32_t retval, i;
	CSL_SERDES_REF_CLOCK refClock=CSL_SERDES_REF_CLOCK_122p88M;
	CSL_SERDES_LINK_RATE serdesRate;
	CSL_SERDES_LOOPBACK  loopback;

#ifndef _TMS320C6X
    AIF2FL_SERDES_B4_CFG_REGS   =    (uint32_t)(hAif->hAif2SerDesBaseAddr->dev.bases[0].serDesB4CfgBase);
    AIF2FL_SERDES_B8_CFG_REGS   =    (uint32_t)(hAif->hAif2SerDesBaseAddr->dev.bases[0].serDesB8CfgBase);
    AIF2FL_CONTROL_REGS         =    (uint32_t)(hAif->hAif2SerDesBaseAddr->dev.bases[0].cfgBase);
#else
	AIF2FL_SERDES_B4_CFG_REGS   =    (0x02324000);
    AIF2FL_SERDES_B8_CFG_REGS   =    (0x02326000);
    AIF2FL_CONTROL_REGS         =    (0x01F00000);
#endif

	if (122880==hAif->aif2ClkSpeedKhz) {
		refClock =  CSL_SERDES_REF_CLOCK_122p88M;
	} else if (153600==hAif->aif2ClkSpeedKhz) {
		refClock =  CSL_SERDES_REF_CLOCK_153p6M;
	} else {
		Aif2_osalLog("Error: Invalid reference clock\n");
	}

	if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol) //used for OBSAI 2x,4x,8x and CPRI 5x
	{
		serdesRate = CSL_SERDES_LINK_RATE_6p144G;
	} else { //used for CPRI 2x,4x,8x
		serdesRate = CSL_SERDES_LINK_RATE_4p9152G;
	}

	// Call the shutdown for each SerDes and restore default values if SerDes were already in use
	CSL_AIF2SerdesShutdown(AIF2FL_SERDES_B8_CFG_REGS);
	CSL_AIF2SerdesShutdown(AIF2FL_SERDES_B4_CFG_REGS);
	if (!CSL_AIF2SerdesIsReset(AIF2FL_SERDES_B8_CFG_REGS)) {
		AIF_serdesRestoreDefault(AIF2FL_SERDES_B8_CFG_REGS);
	}
	if (!CSL_AIF2SerdesIsReset(AIF2FL_SERDES_B4_CFG_REGS)) {
		AIF_serdesRestoreDefault(AIF2FL_SERDES_B4_CFG_REGS);
	}

	if (enableB8)
	{
		CSL_AIF2SerdesInitB8(AIF2FL_SERDES_B8_CFG_REGS, refClock, serdesRate);
		for(i=0; i < 4; i++)
		{
			if(1==hAif->linkConfig[i].linkEnable) {
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
			if(1==hAif->linkConfig[AIF2FL_LINK_4 + i].linkEnable) {
				CSL_AIF2SerdesLaneConfig(AIF2FL_SERDES_B4_CFG_REGS, refClock, serdesRate, i);
			}
		}
		CSL_AIF2SerdesComEnable(AIF2FL_SERDES_B4_CFG_REGS);
	}

	//AIF2 Lane Enable
	for(i=0; i < 6; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable) {
			if (hAif->linkConfig[i].comType == AIF2_LOOPBACK) {       //setup links for internal loopback mode
				loopback = CSL_SERDES_LOOPBACK_ENABLED;
			} else {
				loopback = CSL_SERDES_LOOPBACK_DISABLED;
			}
			if(hAif->linkConfig[i].linkRate == 8) {
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
		//AIF2 B4 PLL Enable
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
		if(1==hAif->linkConfig[i].linkEnable) {
			CSL_AIF2LinkClkEnable(AIF2FL_CONTROL_REGS, i);
		} else {
			CSL_AIF2LinkClkDisable(AIF2FL_CONTROL_REGS, i);
		}
     }
}

#endif // K2

////////////////////

