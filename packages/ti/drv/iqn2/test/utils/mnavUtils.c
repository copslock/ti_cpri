/****************************************************************************\
 *           (C) Copyright 2013, Texas Instruments, Inc.                    *
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <c6x.h>

#include <ti/csl/csl.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_qm_queue.h>

#include <ti/csl/soc.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>
#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

#ifdef USERM
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>
#endif
#ifdef DEVICE_K2L
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#ifdef USERM
#include <ti/drv/rm/device/k2l/global-resource-list.c>
#include <ti/drv/rm/device/k2l/policy_dsp_arm.c>
#endif
#endif
#include <ti/drv/cppi/cppi_desc.h>

#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif


#define __MNAVUTILS_C
#include "cslUtils.h"
#include "mnavUtils.h"

#ifdef _TMS320C6X
#pragma CODE_SECTION(UTILS_initQmss, ".text:tools");
#pragma CODE_SECTION(UTILS_insertQmssRegion, ".text:tools");
#pragma CODE_SECTION(UTILS_initCppiChannel, ".text:tools");
#pragma CODE_SECTION(UTILS_initPktDma, ".text:tools");
#pragma CODE_SECTION(UTILS_resetCppi, ".text:tools");
#pragma CODE_SECTION(UTILS_resetQmss, ".text:tools");
#pragma CODE_SECTION(UTILS_resetRm, ".text:tools");
#endif
Qmss_InitCfg   qmssInitConfig;
#ifdef USERM
Rm_Handle               rmHandle = NULL;
Rm_ServiceHandle       *rmServiceHandle;
#endif


////// global variable for Mix Mode //////////////
Cppi_RxFlowCfg          rxFlowCpriDM;
Qmss_QueueHnd           freeHostQueHnd;

int32_t
UTILS_initQmss(uint32_t *region, uint32_t num_desc, uint32_t desc_size, uint32_t num_desc_tot)
{

#if (DSPCORE == 0)
    int32_t               result;
    Qmss_MemRegInfo     memRegInfo;
#ifdef USERM
    /* RM configuration */
    Rm_InitCfg           rmInitCfg;
    char                 rmServerName[RM_NAME_MAX_CHARS] = "RM_Server";
    int32_t                rmResult;
    Cppi_StartCfg        cppiStartCfg;
#endif

#ifdef USERM
    /* Create the Server instance */
    memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
    rmInitCfg.instName = rmServerName;
    rmInitCfg.instType = Rm_instType_SERVER;
    rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
    rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspPlusArmPolicy;
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    if (rmResult != RM_OK)
    {
        printf ("Error Core %d : Initializing Resource Manager error code : %d\n", DNUM, rmResult);
        return -1;
    }

    rmServiceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    if (rmResult != RM_OK)
    {
        printf ("Error Core %d : Creating RM service handle error code : %d\n", DNUM, rmResult);
        return -1;
    }
#endif
    /* Initialize the QMSS Configuration block. */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up the linking RAM. Use the internal Linking RAM.
     * LLD will configure the internal linking RAM address and maximum internal linking RAM size if
     * a value of zero is specified. Linking RAM1 is not used */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0X00007FFF; //Use shared internal linking Ram for K2
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.mode = Qmss_Mode_JOINT_LOADBALANCED;

//    if (psmessage)
    	qmssInitConfig.maxDescNum      = num_desc_tot;
//    else
//    	qmssInitConfig.maxDescNum      = num_desc;
#ifndef USERM
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
#ifdef _BIG_ENDIAN
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_le);
#endif
#endif

#ifdef USERM
    /* Bypass hardware initialization as it is done within Kernel */
    qmssInitConfig.qmssHwStatus     =   QMSS_HW_INIT_COMPLETE;
    qmssGblCfgParams.qmRmServiceHandle = rmServiceHandle;
#endif
    /* Initialize Queue Manager Sub System */
    result = Qmss_init (&qmssInitConfig, (Qmss_GlobalConfigParams *)&qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        printf("Error initializing Queue Manager SubSystem error code : %d\n", result);
        return -1;
    }
#endif

    /* Start the QMSS. */
    if (Qmss_start() != QMSS_SOK)
    {
        printf("Error: Unable to start the QMSS\n");
        return -1;
    }

    /* Empty all QMSS queues */
#if 0
    for (i=0;i<8192;i++){
    	Qmss_queueEmpty(i);
    }
#endif

#ifndef DFE_CTL
    if (region != NULL)
    {
        /* Memory Region 0 Configuration */
        memRegInfo.descBase         = (uint32_t *)UTILS_local2GlobalAddr(region);
        memRegInfo.descSize         = desc_size;
        memRegInfo.descNum          = num_desc;
        memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
        memRegInfo.memRegion        = (Qmss_MemRegion) (UTILS_getMemRegionNum(region));
        memRegInfo.startIndex       = 0;

        /* Initialize and inset the memory region. */
        if (region) {
            result = Qmss_insertMemoryRegion (&memRegInfo);
            if (result < QMSS_SOK)
            {
                printf("Error inserting memory region: %d\n", result);
                return -1;
            }
        }
    }
#endif

    /* Initialize CPPI CPDMA */
    result = Cppi_init ((Cppi_GlobalConfigParams *)&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        printf("Error initializing Queue Manager SubSystem error code : %d\n", result);
        return -1;
    }
#ifdef USERM
    if (rmServiceHandle) {
        cppiStartCfg.rmServiceHandle = rmServiceHandle;
        Cppi_startCfg(&cppiStartCfg);
    }
#endif

    //qmss_initialized = 1;
    return 0;
}


int32_t
UTILS_insertQmssRegion(uint32_t *region, Qmss_MemRegion mem_region, uint32_t num_desc, uint32_t desc_size, uint32_t start_index)
{
	int32_t               result;
	Qmss_MemRegInfo     memRegInfo;

    /* Memory Region 1 Configuration */
    memRegInfo.descBase         = (uint32_t *)UTILS_local2GlobalAddr(region);
    memRegInfo.descSize         = desc_size;
    memRegInfo.descNum          = num_desc;
    memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memRegInfo.memRegion        = mem_region;
    memRegInfo.startIndex       = start_index;

    /* Initialize and inset the memory region. */
    if (region) {
    	result = Qmss_insertMemoryRegion (&memRegInfo);
    	if (result < QMSS_SOK)
    	{
    		printf("Error inserting memory region: %d\n", result);
    		return -1;
    	}
    }

    return 0;
}

extern Cppi_Handle hCppi;
extern uint32_t 	synchronize;
int32_t
UTILS_initPktDma(
		PktDmaConfigHandle hPktDma
)
{
	Cppi_CpDmaInitCfg    iqn2CPDMACfg;
	Cppi_DescCfg         descCfg;
	Cppi_MonolithicDesc *ptrMonoDesc;
	Cppi_HostDesc       *ptrHostDesc;
	Cppi_DescTag         descTag;
	Qmss_Queue           queueInfo;
	uint32_t				 axcConfigTx, axcConfigRx;
	uint32_t				 ctrlConfigTx, ctrlConfigRx;

	uint32_t               psMsgEnable = 0;
	uint8_t                isAllocated;
	uint32_t				 i,j,k, numAxC, numAllocated;
//	uint32_t				 *dataPtr;
	uint32_t			QentryCount;

    memset(&descTag, 0x00, sizeof(Cppi_DescTag));
    memset(&descCfg, 0x00, sizeof(Cppi_DescCfg));

   /* Initialize Pkt dma if necessary */
	if (hPktDma->psMsgEnable)
		psMsgEnable = 1;

	/* Initialize the IQN2 CPDMA config structure. */
	memset ((void *)&iqn2CPDMACfg, 0, sizeof(Cppi_CpDmaInitCfg));

	/* Setup the IQN2 CPDMA Configuration. */
	iqn2CPDMACfg.dmaNum = Cppi_CpDma_IQN_CPDMA;
	iqn2CPDMACfg.writeFifoDepth = 32; // default
//	iqn2CPDMACfg.timeoutCount = 2048;

//	if (hPktDma->mode != IQN_LTE_FDD_MODE && hPktDma->mode != IQN_LTE_TDD_MODE && hPktDma->mode != IQN_GENERICPACKET_MODE)
//	{
//		IQN2CPDMACfg.writeFifoDepth = 8;
//	}

	/* Open the IQN2 CPDMA. Needed even if DIO used, as we need to disable PKTDMA loopback mode */
	hPktDma->hCppi = Cppi_open (&iqn2CPDMACfg);

	/* Disable the IQN2-CPDMA Loopback mode in the CPPI */
	Cppi_setCpdmaLoopback (hPktDma->hCppi, 0);

	axcConfigTx = hPktDma->firstAxC;
	axcConfigRx = hPktDma->firstAxC;
	if ((hPktDma->hRxFlowAxC[axcConfigRx] != NULL)) // check for first RxFlow existing
	{
		numAxC 		= hPktDma->numAxC;

		for (k=0; k <numAxC ; k++)
		{
			/***********************************************************************
			 ********************** Transmit Configuration *************************
			 ***********************************************************************/

			/* Initialize the Transmit descriptors. We want all the transmit descriptors to go
			 * back to the Transmit Free Queue after transmission. */
			descCfg.memRegion                 = hPktDma->txRegionAxC[axcConfigTx];
			descCfg.descNum                   = hPktDma->txNumDescAxC[axcConfigTx];
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
				hPktDma->txFqAxC[axcConfigTx]  = Cppi_initDescriptor (&descCfg, &numAllocated);
				if (hPktDma->txFqAxC[axcConfigTx] < 0)
				{
					printf("Error: IQN2 Transmit Completion Queue failed to open\n");
					return -1;
				}
				/* Get the Queue Information for the Transmit Free Queue */
				queueInfo = Qmss_getQueueNumber(hPktDma->txFqAxC[axcConfigTx]);

				/* Pop off all descriptors from the Queue and add the missing init parameters. */
				for (j = 0; j < hPktDma->txNumDescAxC[axcConfigTx]; j++)
				{
					/* Get a mono descriptor from the free GP queue. */
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[axcConfigTx]));
					Cppi_setPacketLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,hPktDma->txDescSizeAxC[axcConfigTx]);
					descTag.srcTagLo = 0;
					Cppi_setTag(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,&descTag);
					Cppi_setPSFlags(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
					Cppi_setPSLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
					Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);
					/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
					UTILS_cacheWriteBack((void*)ptrMonoDesc, descCfg.cfg.mono.dataOffset);
					Qmss_queuePushDesc(hPktDma->txFqAxC[axcConfigTx], (uint32_t*)ptrMonoDesc);
				}

				/* Open the IQN2 transmit queue for this AxC stream */
				hPktDma->txQAxC[axcConfigTx] = Qmss_queueOpen (Qmss_QueueType_IQNET_QUEUE , QMSS_IQNET_QUEUE_BASE+axcConfigTx, &isAllocated);
				if (hPktDma->txQAxC[axcConfigTx] < 0)
					{
					printf("Error: IQN2 Transmit Queue failed to open\n");
					return -1;
					}
			}
			axcConfigTx++;
		} ///eof Transmit config


		/***********************************************************************
		 ********************** Receive Queues Configuration *******************
		 ***********************************************************************/

		for (k=0; k <numAxC ; k++)
		{
			/* Initialize the Receive descriptors */
			/* Initialize the Receive descriptors */
			descCfg.memRegion                 = hPktDma->rxRegionAxC[axcConfigRx];
			descCfg.descNum                   = hPktDma->rxNumDescAxC[axcConfigRx];
			descCfg.destQueueNum              = hPktDma->hRxFlowAxC[axcConfigRx]->rx_fdq0_sz0_qnum;
			descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
			descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
			descCfg.descType                  = (Cppi_DescType)hPktDma->hRxFlowAxC[axcConfigRx]->rx_desc_type;
			descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
			descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
			descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
			descCfg.returnPushPolicy          = Qmss_Location_TAIL;
			descCfg.cfg.mono.dataOffset		  = hPktDma->hRxFlowAxC[axcConfigRx]->rx_sop_offset; // size of header

			if (descCfg.descNum != 0)
			{
				/* Initialize the descriptors and place all of them into the general purpose queue */
				hPktDma->rxFqAxC[axcConfigRx]  = Cppi_initDescriptor (&descCfg, &numAllocated);
				if (hPktDma->rxFqAxC[axcConfigRx] < 0)
				{
					printf("Error: IQN2 Receive Completion Queue failed to open\n");
					return -1;
				}

				/* Get the Queue Information for the Transmit Free Queue */
				queueInfo = Qmss_getQueueNumber(hPktDma->rxFqAxC[axcConfigRx]);

				/* Pop off all descriptors from the Queue and add the missing init parameters. */
				for (j = 0; j < hPktDma->rxNumDescAxC[axcConfigRx]; j++)
				{
					/* Get a mono descriptor from the free GP queue. */
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxFqAxC[axcConfigRx]));
					Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);
					/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
					UTILS_cacheWriteBack((void*)ptrMonoDesc, descCfg.cfg.mono.dataOffset);
					Qmss_queuePushDesc(hPktDma->rxFqAxC[axcConfigRx], (uint32_t*)ptrMonoDesc);
				}

				//Qmss_QueueType_GENERAL_PURPOSE_QUEUE to be verified

				/* Open the IQN2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
				hPktDma->rxQAxC[axcConfigRx] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hPktDma->hRxFlowAxC[axcConfigRx]->rx_dest_qnum, &isAllocated);
				if (hPktDma->rxQAxC[axcConfigRx] < 0)
				{
					printf("Error: IQN2 Receive Queue failed to open\n");
					return -1;
				}

				/***********************************************************************
				 ********************** Receive Flow Configuration *********************
				 ***********************************************************************/
				hPktDma->hRxFlowAxC[axcConfigRx]->flowIdNum = axcConfigRx;
				Cppi_configureRxFlow (hPktDma->hCppi, hPktDma->hRxFlowAxC[axcConfigRx], &isAllocated);
			}

			axcConfigRx++;
		}
	}

	if (hPktDma->ctrlDescType != Cppi_DescType_HOST)
	{
        /* If PktDma mode and/or ctrl messages enabled, open and populate free queues, open tx/rx queues, configure flows  */
        if (psMsgEnable == 1)
        {
            /***********************************************************************
             ********************** Transmit Configuration *************************
             ***********************************************************************/
            ctrlConfigTx = hPktDma->firstCtrl;
            ctrlConfigRx = hPktDma->firstCtrl;
            for(i=0; i< hPktDma->numCtrl; i++)
            {
                if (hPktDma->hRxFlowCtrl[i] != NULL)
                {

                    /***********************************************************************
                     ********************** Transmit Queues Configuration ******************
                     ***********************************************************************/

                    /* Initialize the Transmit descriptors. We want all the transmit descriptors to go
                     * back to the Transmit Free Queue after transmission. */
                    descCfg.memRegion                 = hPktDma->txRegionCtrl[i];
                    descCfg.descNum                   = hPktDma->txNumDescCtrl[i];
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
                    hPktDma->txFqCtrl[i]  = Cppi_initDescriptor (&descCfg, &numAllocated);
                    if (hPktDma->txFqCtrl[i] < 0)
                    {
                        printf("Error: IQN2 Transmit Completion CTRL Queue failed to open\n");
                        return -1;
                    }
                    QentryCount = Qmss_getQueueEntryCount(hPktDma->txFqCtrl[i]);
                    /* Get the Queue Information for the Transmit Free Queue */
                    queueInfo = Qmss_getQueueNumber(hPktDma->txFqCtrl[i]);

                    /* Pop off all descriptors from the Queue and add the missing init parameters. */
                    for (j = 0; j < hPktDma->txNumDescCtrl[i]; j++)
                    {
                        /* Get a mono descriptor from the free GP queue. */
                        ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[i]));

                        Cppi_setPacketLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,hPktDma->txDescSizeCtrl[i]);
    //					descTag.srcTagLo = 124+i;
    //					Cppi_setTag(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,&descTag);
                        Cppi_setPSFlags(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
                        Cppi_setPSLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
                        Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);
                        /* Add the packet descriptor to the AIF2 Transmit Free Queue. */
                        UTILS_cacheWriteBack((void*)ptrMonoDesc, descCfg.cfg.mono.dataOffset);
                        Qmss_queuePushDesc(hPktDma->txFqCtrl[i], (uint32_t*)ptrMonoDesc);
                    }

                    /* Open the IQN2 transmit queue for this control stream */
                    hPktDma->txQCtrl[i] = Qmss_queueOpen (Qmss_QueueType_IQNET_QUEUE , QMSS_IQNET_QUEUE_BASE+ctrlConfigTx+i, &isAllocated);
                    if (hPktDma->txQCtrl[i] < 0)
                    {
                        printf("Error: IQN2 Transmit Queue failed to open\n");
                        return -1;
                    }


                    /***********************************************************************
                     ********************** Receive Queues Configuration *******************
                     ***********************************************************************/

                    /* Initialize the Receive descriptors */
                    descCfg.memRegion                 = hPktDma->rxRegionCtrl[i];
                    descCfg.descNum                   = hPktDma->rxNumDescCtrl[i];
                    descCfg.destQueueNum              = hPktDma->hRxFlowCtrl[i]->rx_fdq0_sz0_qnum;
                    descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
                    descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
                    descCfg.descType                  = (Cppi_DescType)hPktDma->hRxFlowCtrl[i]->rx_desc_type;
                    descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
                    descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
                    descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
                    descCfg.returnPushPolicy          = Qmss_Location_TAIL;
                    descCfg.cfg.mono.dataOffset		  = hPktDma->hRxFlowCtrl[i]->rx_sop_offset; // size of header for control words

                    /* Initialize the descriptors and place all of them into the general purpose queue */
                    hPktDma->rxFqCtrl[i]  = Cppi_initDescriptor (&descCfg, &numAllocated);
                    if (hPktDma->rxFqCtrl[i] < 0)
                    {
                        printf("Error: IQN2 Receive Completion Queue failed to open\n");
                        return -1;
                    }
                    QentryCount = Qmss_getQueueEntryCount(hPktDma->rxFqCtrl[i]);

                    /* Get the Queue Information for the Transmit Free Queue */
                    queueInfo = Qmss_getQueueNumber(hPktDma->rxFqCtrl[i]);

                    /* Pop off all descriptors from the Queue and add the missing init parameters. */
                    for (j = 0; j < hPktDma->rxNumDescCtrl[i]; j++)
                    {
                        /* Get a mono descriptor from the free GP queue. */
                        ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxFqCtrl[i]));
                        Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);
                        /* Add the packet descriptor to the AIF2 Transmit Free Queue. */
                        UTILS_cacheWriteBack((void*)ptrMonoDesc, descCfg.cfg.mono.dataOffset);
                        Qmss_queuePushDesc(hPktDma->rxFqCtrl[i], (uint32_t*)ptrMonoDesc);
                    }


                    /* Open the IQN2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
                    hPktDma->rxQCtrl[i] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hPktDma->hRxFlowCtrl[i]->rx_dest_qnum, &isAllocated);
                    if (hPktDma->rxQCtrl[i] < 0)
                    {
                        printf("Error: IQN2 Receive Queue failed to open\n");
                        return -1;
                    }


                    /***********************************************************************
                     ********************** Receive Flow Configuration *********************
                     ***********************************************************************/
                    hPktDma->hRxFlowCtrl[i]->flowIdNum = ctrlConfigRx+i;
                    Cppi_configureRxFlow (hPktDma->hCppi, hPktDma->hRxFlowCtrl[i], &isAllocated);

                }
            }
        }
	} else {

        /* If PktDma mode and/or ctrl messages enabled, open and populate free queues, open tx/rx queues, configure flows  */
        if (psMsgEnable == 1)
        {
            uint32_t *txBuffer;
            uint32_t *rxBuffer;
            /***********************************************************************
             ********************** Transmit Configuration *************************
             ***********************************************************************/
            ctrlConfigTx = hPktDma->firstCtrl;
            ctrlConfigRx = hPktDma->firstCtrl;
            for(i=0; i< hPktDma->numCtrl; i++)
            {
                if (hPktDma->hRxFlowCtrl[i] != NULL)
                {

                    txBuffer = hPktDma->txDataBuff[i];
                    rxBuffer = hPktDma->rxDataBuff[i];
                    /***********************************************************************
                     ********************** Transmit Queues Configuration ******************
                     ***********************************************************************/

                    /* Initialize the Transmit descriptors. We want all the transmit descriptors to go
                     * back to the Transmit Free Queue after transmission. */
                    descCfg.memRegion                 = hPktDma->txRegionCtrl[i];
                    descCfg.descNum                   = hPktDma->txNumDescCtrl[i];
                    descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
                    descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
                    descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
                    descCfg.descType                  = Cppi_DescType_HOST;
                    descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
                    descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
                    descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
                    descCfg.returnPushPolicy          = Qmss_Location_TAIL;
                    descCfg.cfg.host.returnPolicy	  = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET; // return entire packet
                    descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

                    /* Initialize the descriptors and place all of them into the general purpose queue */
                    hPktDma->txFqCtrl[i]  = Cppi_initDescriptor (&descCfg, &numAllocated);
                    if (hPktDma->txFqCtrl[i] < 0)
                    {
                        printf("Error: IQN2 Transmit Completion CTRL Queue failed to open\n");
                        return -1;
                    }
                    QentryCount = Qmss_getQueueEntryCount(hPktDma->txFqCtrl[i]);
                    /* Get the Queue Information for the Transmit Free Queue */
                    queueInfo = Qmss_getQueueNumber(hPktDma->txFqCtrl[i]);

                    /* Pop off all descriptors from the Queue and add the missing init parameters. */
                    for (j = 0; j < hPktDma->txNumDescCtrl[i]; j++)
                    {
                        /* Get a host descriptor from the free GP queue. */
                        ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[i]));

                        Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, queueInfo);

                        /* Set PS data */
                        Cppi_setPSLocation (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, Cppi_PSLoc_PS_IN_DESC);

                        Cppi_setPSFlags(descCfg.descType,(Cppi_Desc*)ptrHostDesc,0);

    //					Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t *) &psData, 0);

                        /* Add data buffer */
                        Cppi_setData(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(Uint8*) &txBuffer[j*hPktDma->txDescSizeCtrl[i]/4], hPktDma->txDescSizeCtrl[i]);

                        /* Save original buffer information */
                        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc, (Uint8 *) (Uint8*) &txBuffer[(j*hPktDma->txDescSizeCtrl[i])/4], hPktDma->txDescSizeCtrl[i]);

                        Cppi_setReturnPolicy (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc,(Cppi_ReturnPolicy) 0);

                        Cppi_setReturnPushPolicy (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc,(Qmss_Location) 0);

                        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (Cppi_Desc*)NULL);

                        /* Set packet length */
                        Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc, hPktDma->txDescSizeCtrl[i]);

    //					descTag.destTagHi = 0;					descTag.destTagLo = 0;
    //					descTag.srcTagHi  = 0;					descTag.srcTagLo  = k;
    //					Cppi_setTag(Cppi_DescType_HOST,(Cppi_Desc *)ptrHostDesc,&descTag);

                        /* Add the packet descriptor to the AIF2 Transmit Free Queue. */
                        UTILS_cacheWriteBack((void*)ptrHostDesc, 64);
    //					Qmss_queuePushDesc(hPktDma->txFqCtrl[i], (uint32_t*)ptrHostDesc);
                        Qmss_queuePushDescSize(hPktDma->txFqCtrl[i], (uint32_t*)ptrHostDesc, 64);
                    }
                    QentryCount = Qmss_getQueueEntryCount(hPktDma->txFqCtrl[i]);

                    /* Open the IQN2 transmit queue for this control stream */
                    hPktDma->txQCtrl[i] = Qmss_queueOpen (Qmss_QueueType_IQNET_QUEUE , QMSS_IQNET_QUEUE_BASE+ctrlConfigTx+i, &isAllocated);
                    if (hPktDma->txQCtrl[i] < 0)
                    {
                        printf("Error: IQN2 Transmit Queue failed to open\n");
                        return -1;
                    }
                    QentryCount = Qmss_getQueueEntryCount(hPktDma->txQCtrl[i]);

                    /***********************************************************************
                     ********************** Receive Queues Configuration *******************
                     ***********************************************************************/

                    /* Initialize the Receive descriptors */
                    descCfg.memRegion                 = hPktDma->rxRegionCtrl[i];
                    descCfg.descNum                   = hPktDma->rxNumDescCtrl[i];
                    descCfg.destQueueNum              = hPktDma->hRxFlowCtrl[i]->rx_fdq0_sz0_qnum;
                    descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
                    descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
                    descCfg.descType                  = (Cppi_DescType)hPktDma->hRxFlowCtrl[i]->rx_desc_type;
                    descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
                    descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
                    descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
                    descCfg.returnPushPolicy          = Qmss_Location_TAIL;
                    descCfg.cfg.host.returnPolicy	  = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET; // return entire packet
                    descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

                    /* Initialize the descriptors and place all of them into the general purpose queue */
                    hPktDma->rxFqCtrl[i]  = Cppi_initDescriptor (&descCfg, &numAllocated);
                    if (hPktDma->rxFqCtrl[i] < 0)
                    {
                        printf("Error: IQN2 Receive Completion Queue failed to open\n");
                        return -1;
                    }

                    QentryCount = Qmss_getQueueEntryCount(hPktDma->rxFqCtrl[i]);

                    /* Get the Queue Information for the Transmit Free Queue */
                    queueInfo = Qmss_getQueueNumber(hPktDma->rxFqCtrl[i]);

                    /* Pop off all descriptors from the Queue and add the missing init parameters. */
                    for (j = 0; j < hPktDma->rxNumDescCtrl[i]; j++)
                    {
                        /* Get a host descriptor from the free Rx queue. */
                        ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxFqCtrl[i]));

                        Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, queueInfo);

                        /* Set PS data */
                        Cppi_setPSLocation (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, Cppi_PSLoc_PS_IN_DESC);

                        Cppi_setPSFlags(descCfg.descType,(Cppi_Desc*)ptrHostDesc,0);

    //                  Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t *) &psData, 0);

                        /* Add data buffer */
                        Cppi_setData(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(Uint8*) &rxBuffer[j*hPktDma->rxDescSizeCtrl[i]/4], hPktDma->rxDescSizeCtrl[i]);

                        /* Save original buffer information */
                        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc, (Uint8 *) (Uint8*) &rxBuffer[(j*hPktDma->rxDescSizeCtrl[i])/4], hPktDma->rxDescSizeCtrl[i]);

                        Cppi_setReturnPolicy (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc,(Cppi_ReturnPolicy) 0);

                        Cppi_setReturnPushPolicy (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc,(Qmss_Location) 0);

                        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (Cppi_Desc*)NULL);

                        /* Set packet length */
                        Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc, hPktDma->rxDescSizeCtrl[i]);

                        /* Add the packet descriptor to the AIF2 Transmit Free Queue. */
                        UTILS_cacheWriteBack((void*)ptrHostDesc, 64);
                        Qmss_queuePushDescSize(hPktDma->rxFqCtrl[i], (uint32_t*)ptrHostDesc, 64);
                    }

                    QentryCount = Qmss_getQueueEntryCount(hPktDma->rxFqCtrl[i]);

                    /* Open the IQN2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
                    hPktDma->rxQCtrl[i] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hPktDma->hRxFlowCtrl[i]->rx_dest_qnum, &isAllocated);
                    if (hPktDma->rxQCtrl[i] < 0)
                    {
                        printf("Error: IQN2 Receive Queue failed to open\n");
                        return -1;
                    }
                    QentryCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[i]);

                    /***********************************************************************
                     ********************** Receive Flow Configuration *********************
                     ***********************************************************************/
                    hPktDma->hRxFlowCtrl[i]->flowIdNum = ctrlConfigRx+i;
                    Cppi_configureRxFlow (hPktDma->hCppi, hPktDma->hRxFlowCtrl[i], &isAllocated);

                }
            }
        }
	}
	/*i = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[0]);
	i = Qmss_getQueueEntryCount(hPktDma->rxFqCtrl[0]);
	i = Qmss_getQueueEntryCount(hPktDma->txQCtrl[0]);
	i = Qmss_getQueueEntryCount(hPktDma->txFqCtrl[0]);*/

    return 0;
}

extern uint32_t numcore2sync;
void
UTILS_initCppiChannel(PktDmaConfigHandle hPktDma)
{
    Cppi_RxChInitCfg     rxCfg;
    Cppi_TxChInitCfg     txCfg;
    uint32_t 				 k,numCh,numCtrl, firstChan, firstCtl;
    uint8_t                isAllocated;

    numCh = 0;
    numCtrl = 0;

    numCh = hPktDma->numAxC;
    numCtrl = hPktDma->numCtrl;
    firstChan = hPktDma->firstAxC;
    firstCtl = hPktDma->firstCtrl;
	/***********************************************************************
	 ********************** PktDma channel Configuration *******************
	 ***********************************************************************/
    for (k=firstChan; k<(numCh+firstChan); k++)
    {
		/* Open the IQN2 Transmit Channel and keep it disabled. */
		memset(&txCfg, 0, sizeof(txCfg));
		txCfg.channelNum   = k;
		txCfg.priority     = 0;
		txCfg.txEnable     = Cppi_ChState_CHANNEL_ENABLE;
		txCfg.filterEPIB   = 0;
		txCfg.filterPS     = 0;
		/* It is possible for the FFTC to send output packets directly to the IQN2 without any DSP intervention. In that case, the IQN_MONO_MODE should
		 * be turned off and FFTC uses a descriptor size field of 16 bytes as normal mode, because FFTC can not set 64 bytes to DESC_SIZE field before
		 * pushing the descriptor into the IQN2 TX queue.
		 */
		txCfg.aifMonoMode  = 0;

		hPktDma->txChAxC[k] = Cppi_txChannelOpen(hPktDma->hCppi, &txCfg, &isAllocated);
		if (hPktDma->txChAxC[k] == NULL)
			{
			printf("Error: Opening IQN2 Tx channel %d failed\n", txCfg.channelNum);
			return;
			}

		/***********************************************************************
		 ********************** PktDma channel Configuration *******************
		 ***********************************************************************/

		/* Open the IQN2 Receive Channel and keep it disabled */
		memset(&rxCfg, 0, sizeof(rxCfg));
		rxCfg.channelNum = k;
		rxCfg.rxEnable   = Cppi_ChState_CHANNEL_ENABLE;

		hPktDma->rxChAxC[k] = Cppi_rxChannelOpen(hPktDma->hCppi, &rxCfg, &isAllocated);
		if (hPktDma->rxChAxC[k] == NULL)
		{
			printf("Error: Opening IQN2 Rx channel %d failed\n", rxCfg.channelNum);
			return;
		}
    }

    for (k=firstCtl; k<(numCtrl+firstCtl); k++)
    {
    	/* Open the IQN2 Transmit Channel and keep it disabled. */
		memset(&txCfg, 0, sizeof(txCfg));
		txCfg.channelNum   = k;
		txCfg.priority     = 7;
		txCfg.txEnable     = Cppi_ChState_CHANNEL_ENABLE;
		txCfg.filterEPIB   = 0;
		txCfg.filterPS     = 0;
		/* It is possible for the FFTC to send output packets directly to the IQN2 without any DSP intervention. In that case, the IQN_MONO_MODE should
		 * be turned off and FFTC uses a descriptor size field of 16 bytes as normal mode, because FFTC can not set 64 bytes to DESC_SIZE field before
		 * pushing the descriptor into the IQN2 TX queue.
		 */
		txCfg.aifMonoMode  = 0;

		hPktDma->txChCtrl[k] = Cppi_txChannelOpen(hPktDma->hCppi, &txCfg, &isAllocated);
		if (hPktDma->txChCtrl[k] == NULL)
		{
			printf("Error: Opening IQN2 Tx channel %d failed\n", txCfg.channelNum);
			return;
		}

		/***********************************************************************
		 ********************** PktDma channel Configuration *******************
		 ***********************************************************************/

		/* Open the IQN2 Receive Channel and keep it disabled */
		memset(&rxCfg, 0, sizeof(rxCfg));
		rxCfg.channelNum = k;
		rxCfg.rxEnable   = Cppi_ChState_CHANNEL_ENABLE;

		hPktDma->rxChCtrl[k] = Cppi_rxChannelOpen(hPktDma->hCppi, &rxCfg, &isAllocated);
		if (hPktDma->rxChCtrl[k] == NULL)
		{
			printf("Error: Opening IQN2 Rx channel %d failed\n", rxCfg.channelNum);
			return;
		}
    }
}

void
UTILS_resetCppi(
        PktDmaConfigHandle hPktDma
)
{
uint32_t i;

    for (i =hPktDma->firstAxC ; i< (hPktDma->firstAxC+hPktDma->numAxC); i++)
    {
        if (hPktDma->txChAxC[i]) {Cppi_channelDisable (hPktDma->txChAxC[i]); Cppi_channelClose (hPktDma->txChAxC[i]); hPktDma->txChAxC[i] = NULL;}
        if (hPktDma->rxChAxC[i]) {Cppi_channelDisable (hPktDma->rxChAxC[i]); Cppi_channelClose (hPktDma->rxChAxC[i]); hPktDma->rxChAxC[i] = NULL;}
    }
    for(i=hPktDma->firstCtrl; i< (hPktDma->firstCtrl+hPktDma->numCtrl); i++)
    {
        if (hPktDma->txChCtrl[i]) {Cppi_channelDisable (hPktDma->txChCtrl[i]); Cppi_channelClose (hPktDma->txChCtrl[i]); hPktDma->txChCtrl[i] = NULL;}
        if (hPktDma->rxChCtrl[i]) {Cppi_channelDisable (hPktDma->rxChCtrl[i]); Cppi_channelClose (hPktDma->rxChCtrl[i]); hPktDma->rxChCtrl[i] = NULL;}
    }

    if (hPktDma->hCppi) {Cppi_close (hPktDma->hCppi); hPktDma->hCppi = NULL;}

    Cppi_exit();

}

void
UTILS_resetQmss(
        PktDmaConfigHandle hPktDma
)
{
uint32_t i;

    for (i =hPktDma->firstAxC ; i< (hPktDma->firstAxC+hPktDma->numAxC); i++) {
        Qmss_queueEmpty(hPktDma->txQAxC[i]);
        Qmss_queueEmpty(hPktDma->rxQAxC[i]);
        Qmss_queueEmpty(hPktDma->txFqAxC[i]);
        Qmss_queueEmpty(hPktDma->rxFqAxC[i]);

        Qmss_queueClose(hPktDma->txQAxC[i]);
        Qmss_queueClose(hPktDma->rxQAxC[i]);
        Qmss_queueClose(hPktDma->txFqAxC[i]);
        Qmss_queueClose(hPktDma->rxFqAxC[i]);
    }
    if (hPktDma->numAxC) {
        Qmss_removeMemoryRegion(hPktDma->rxRegionAxC[hPktDma->firstAxC], 0);
        Qmss_removeMemoryRegion(hPktDma->txRegionAxC[hPktDma->firstAxC], 0);
        for (i =(hPktDma->firstAxC+1) ; i< (hPktDma->firstAxC+hPktDma->numAxC); i++) {
            if (hPktDma->rxRegionAxC[i] != hPktDma->rxRegionAxC[i-1]) {
                Qmss_removeMemoryRegion(hPktDma->rxRegionAxC[i], 0);
            }
        	if (hPktDma->txRegionAxC[i] != hPktDma->txRegionAxC[i-1]) {
                Qmss_removeMemoryRegion(hPktDma->txRegionAxC[i], 0);
            }
        }
    }

    for (i =hPktDma->firstCtrl ; i< (hPktDma->firstCtrl+hPktDma->numCtrl); i++) {
        Qmss_queueEmpty(hPktDma->txQCtrl[i]);
        Qmss_queueEmpty(hPktDma->rxQCtrl[i]);
        Qmss_queueEmpty(hPktDma->txFqCtrl[i]);
        Qmss_queueEmpty(hPktDma->rxFqCtrl[i]);

        Qmss_queueClose(hPktDma->txQCtrl[i]);
        Qmss_queueClose(hPktDma->rxQCtrl[i]);
        Qmss_queueClose(hPktDma->txFqCtrl[i]);
        Qmss_queueClose(hPktDma->rxFqCtrl[i]);
    }
    if (hPktDma->numCtrl) {
        Qmss_removeMemoryRegion(hPktDma->txRegionCtrl[hPktDma->firstCtrl], 0);
        for (i =(hPktDma->firstCtrl+1) ; i< (hPktDma->firstCtrl+hPktDma->numCtrl); i++) {
            if (hPktDma->txRegionCtrl[i] != hPktDma->txRegionCtrl[i-1]) {
                Qmss_removeMemoryRegion(hPktDma->txRegionCtrl[i], 0);
            }
        }
    }

    Qmss_exit();

}

void
UTILS_resetRm(
)
{
#ifdef USERM
    int32_t                rmResult;
 	 if (rmHandle)
 	 {
 		rmResult = Rm_serviceCloseHandle(rmServiceHandle);
 		if (rmResult != RM_OK)
 		{
 			printf ("Error Core %d : Deleting RM service handle error code : %d\n", DNUM, rmResult);
 			return;
 		}
 		rmResult = Rm_delete(rmHandle, 1);
		if (rmResult != RM_OK)
		{
			printf ("Error Core %d : Deleting Resource Manager error code : %d\n", DNUM, rmResult);
			return;
		}
		rmHandle = NULL;
 	 }
#endif
}



////////////////////
