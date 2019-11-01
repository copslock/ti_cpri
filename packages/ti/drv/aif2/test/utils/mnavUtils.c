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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <c6x.h>

#if EVM_TYPE == 0
#include <EVM.h>
#endif

#include <ti/csl/csl.h>
#if defined(DEVICE_K2K)
#include <ti/csl/soc/k2k/src/cslr_device.h>
#include <ti/csl/soc/k2k/src/csl_device_interrupt.h>
#include <ti/csl/soc/k2k/src/csl_qm_queue.h>
#endif
#if defined(DEVICE_K2H)
#include <ti/csl/soc/k2h/src/cslr_device.h>
#include <ti/csl/soc/k2h/src/csl_device_interrupt.h>
#include <ti/csl/soc/k2h/src/csl_qm_queue.h>
#endif
#include <ti/csl/cslr_tpcc.h>
#include <ti/csl/csl_pllc.h>
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H)
#include <ti/csl/csl_pllcAux.h>
#endif
#include <ti/csl/soc.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_pllc.h>
#include <ti/csl/csl_bootcfgAux.h>


#include <ti/drv/aif2/aif2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

#include <ti/drv/qmss/qmss_firmware.h>

#ifdef USERM
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>
#endif
#if defined(DEVICE_K2K)
#include <ti/drv/qmss/device/k2k/src/qmss_device.c>
#include <ti/drv/cppi/device/k2k/src/cppi_device.c>
#ifdef USERM
#include <ti/drv/rm/device/k2k/global-resource-list.c>
#include <ti/drv/rm/device/k2k/policy_dsp_arm.c>
#endif
#elif defined(DEVICE_K2H)
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#ifdef USERM
#include <ti/drv/rm/device/k2h/global-resource-list.c>
#include <ti/drv/rm/device/k2h/policy_dsp_arm.c>
#endif
#else
#include <ti/drv/qmss/device/qmss_device.c>
#include <ti/drv/cppi/device/cppi_device.c>
#endif
#include <ti/drv/cppi/cppi_desc.h>

#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif

//#include <ti/drv/aif2/test/multicore/lte_process.h>

#define __MNAVUTILS_C
#include "mnavUtils.h"

#pragma CODE_SECTION(UTILS_initQmss, ".text:tools");
#pragma CODE_SECTION(UTILS_resetQmss, ".text:tools");
#pragma CODE_SECTION(UTILS_hostLocalTransferInit, ".text:tools");
#pragma CODE_SECTION(UTILS_initPktDma, ".text:tools");

/////////
//KeyStone II compatibility
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
#define AIF2FL_CONTROL_REGS					CSL_AIF_CFG_REGS
#define AIF2FL_CFG_CPPI_DMA_GLOBAL_CFG_REGS 	CSL_AIF_CFG_PKTDMA_GLOBAL_CFG_REGS
#define AIF2FL_CFG_CPPI_DMA_RX_CFG_REGS 		CSL_AIF_CFG_PKTDMA_RX_CFG_REGS
#define AIF2FL_CFG_CPPI_DMA_TX_CFG_REGS		CSL_AIF_CFG_PKTDMA_TX_CFG_REGS
#define CSL_CGEM0_5_REG_BASE_ADDRESS_REGS		CSL_C66X_COREPAC_REG_BASE_ADDRESS_REGS
#endif

Qmss_InitCfg   qmssInitConfig;
#ifdef USERM
Rm_Handle               rmHandle = NULL;
Rm_ServiceHandle       *rmServiceHandle;
#endif
////// global variable for Mix Mode //////////////
Cppi_RxFlowCfg          rxFlowCpriDM;
Qmss_QueueHnd           freeHostQueHnd;

uint32_t NumSeg[14] =     {1    , 1    , 1    , 2    , 1    , 2    , 1    , 1    , 1    , 1    , 2    , 1    , 2    , 1   };
uint32_t Seg1Offset[14] = {2928 , 0    , 2208 , 4400 , 1472 , 3664 , 736  , 2928 , 0    , 2208 , 4400 , 1472 , 3664 , 736 };
uint32_t Seg1Len[14] =    {2192 , 2208 , 2192 , 720  , 2192 , 1456 , 2192 , 2192 , 2208 , 2192 , 720  , 2192 , 1456 , 2192 };
uint32_t Seg2Offset[14] = {0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    };
uint32_t Seg2Len[14] =    {0    , 0    , 0    , 1472 , 0    , 736  , 0    , 0    , 0    , 0    , 1472 , 0    , 736  , 0    };

extern AIF_ConfigHandle hConfigAif;
//extern Cppi_RxFlowCfg rxFlow[];
////// Local functions to this file - prototype


static uint32_t qmss_initialized = 0;

int32_t
UTILS_initQmss(uint32_t *region, uint32_t num_desc, uint32_t desc_size, uint32_t psmessage, uint32_t *regioncw)
{

    int32_t               result;
    Qmss_MemRegInfo     memRegInfo;
#ifdef USERM
    /* RM configuration */
    Rm_InitCfg           rmInitCfg;
    char                 rmServerName[RM_NAME_MAX_CHARS] = "RM_Server";
    int32_t                rmResult;
    Cppi_StartCfg        cppiStartCfg;
#endif

	if (DNUM == 0)
	{
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
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
		qmssInitConfig.linkingRAM0Size = 0X00007FFF; //Use shared internal linking Ram for K2
#else
		qmssInitConfig.linkingRAM0Size = 0;
#endif
		qmssInitConfig.linkingRAM1Base = 0;
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
		qmssInitConfig.mode = Qmss_Mode_JOINT_LOADBALANCED;
#endif
		if (psmessage)
			qmssInitConfig.maxDescNum      = num_desc+32;
		else
			qmssInitConfig.maxDescNum      = num_desc;
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
	}

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

	if (DNUM == 0)
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

		if (psmessage)
		{
			/* Memory Region 0 Configuration */
			memRegInfo.descBase         = (uint32_t *)UTILS_local2GlobalAddr(regioncw);
			memRegInfo.descSize         = 1920;
			memRegInfo.descNum          = 32;
			memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
			memRegInfo.memRegion        = (Qmss_MemRegion) ((UTILS_getMemRegionNum(region))+1);
			memRegInfo.startIndex       = num_desc;

			/* Initialize and inset the memory region. */
			if (regioncw) {
				result = Qmss_insertMemoryRegion (&memRegInfo);
				if (result < QMSS_SOK)
				{
					printf("Error inserting memory region CW: %d\n", result);
					return -1;
				}
			}

		}

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

		qmss_initialized = 1;
	}

    return 0;
}

void
UTILS_resetQmss(
)
{
 	 if (qmss_initialized)
 	 {
 		 Cppi_exit();
 		 qmss_initialized = 0;
 	 }
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

void
UTILS_hostLocalTransferInit (
		uint32_t *host_region,
		Qmss_MemRegion memRegion,
		uint32_t* txBuffer,
		uint32_t* rxBuffer,
		Qmss_QueueHnd* txFq,
		Qmss_QueueHnd* txQ,
		Qmss_QueueHnd* rxFq,
		Qmss_QueueHnd* rxQ,
		Qmss_QueueHnd* txCmplQueHnd,
		uint32_t nbLteCh
)
{

	// variables for QMSS
	int32_t               result;
	// variables for CPPI
	Cppi_CpDmaInitCfg    aif2CPDMACfg;
	Cppi_DescCfg         descCfg;
	Cppi_HostDesc       *ptrHostDesc;
	Cppi_HostDesc       *ptrLinkHostDesc;
	Cppi_RxChInitCfg     rxCfg;
	Cppi_TxChInitCfg     txCfg;
	Qmss_Queue           queueInfo;
	Cppi_Handle          cppiHnd;
	Cppi_FlowHnd         rxFlowHnd;
	Cppi_DescTag 		 descTag;
	Cppi_ChHnd           txCh;
	Cppi_ChHnd           rxCh;
	uint8_t                isAllocated;
	uint32_t				 j,k,numAllocated;
	uint32_t				 *dataPtr;
	uint8_t				 psData;
    uint32_t              i;
	Qmss_MemRegInfo     memRegInfo;

	/*********************************************************************************************
	 *********************************************************************************************
	 * 								QMSS Initialization											**
	 * *******************************************************************************************
	 * *******************************************************************************************/

	printf("Memory region initialization\n");

	/* Initialize the QMSS Configuration block. */
	memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

	/* Set up the linking RAM. Use the internal Linking RAM.
	 * LLD will configure the internal linking RAM address and maximum internal linking RAM size if
	 * a value of zero is specified. Linking RAM1 is not used */
	/*qmssInitConfig.linkingRAM0Base = 0;
	qmssInitConfig.linkingRAM0Size = 128;
	qmssInitConfig.linkingRAM1Base = 0;
	qmssInitConfig.maxDescNum      = 128;*/
    qmssInitConfig.linkingRAM0Base = 0;
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    qmssInitConfig.linkingRAM0Size = 0X00007FFF; //Use shared internal linking Ram for K2
#else
    qmssInitConfig.linkingRAM0Size = 0;
#endif
    qmssInitConfig.linkingRAM1Base = 0;
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    qmssInitConfig.mode = Qmss_Mode_JOINT_LOADBALANCED;
#endif

	qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
#ifdef _BIG_ENDIAN
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_le);
#endif
    qmssInitConfig.maxDescNum      = 128;


	/* Initialize Queue Manager Sub System */
	result = Qmss_init (&qmssInitConfig, (Qmss_GlobalConfigParams *)&qmssGblCfgParams);
	if (result != QMSS_SOK)
	{
		printf("Error initializing Queue Manager SubSystem error code : %d\n", result);
	}

	/* Start the QMSS. */
	if (Qmss_start() != QMSS_SOK)
	{
		printf("Error: Unable to start the QMSS\n");
	}

	/* Empty all QMSS queues */
	for (i=0;i<8192;i++){
		Qmss_queueEmpty(i);
	}

	/* Initialize CPPI CPDMA */
	result = Cppi_init ((Cppi_GlobalConfigParams *)&cppiGblCfgParams);
	if (result != CPPI_SOK)
	{
		printf("Error initializing Queue Manager SubSystem error code : %d\n", result);
	}

	/* Memory Region 0 Configuration */
	memRegInfo.descBase         = (uint32_t *) UTILS_local2GlobalAddr ((uint32_t)host_region);
	memRegInfo.descSize         = SIZE_HOST_DESC;
	memRegInfo.descNum          = 128;
	memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
	memRegInfo.memRegion        = memRegion;
	memRegInfo.startIndex       = 0;

	/* Initialize and inset the memory region. */
	result = Qmss_insertMemoryRegion (&memRegInfo);
	if (result < QMSS_SOK)
	{
		printf("Error inserting memory region: %d\n", result);
	}

	/*********************************************************************************************
	 ****************************** CPPI Initialization	******************************************
	 * *******************************************************************************************/

	printf("CPPI initialization\n");


	/* Set up QMSS CPDMA configuration */
	memset ((void *) &aif2CPDMACfg, 0, sizeof (Cppi_CpDmaInitCfg));
	aif2CPDMACfg.dmaNum         = Cppi_CpDma_QMSS_CPDMA;

	/* Open QMSS CPDMA */
	cppiHnd = (Cppi_Handle) Cppi_open (&aif2CPDMACfg);
	if (cppiHnd == NULL)
	{
		printf("Error Core %d : Initializing QMSS CPPI CPDMA %d\n", 0, aif2CPDMACfg.dmaNum);
	}
	/* Enable the AIF2-CPDMA Loopback mode in the CPPI */
	Cppi_setCpdmaLoopback (cppiHnd, 1);


	/***********************************************************************
	 ********************** Free queue Configuration ***********************
	 ***********************************************************************/
	printf ("\n~~~~~~~~~~~~~~~Initializing descriptors~~~~~~~~~~~~~~~\n");

	/* Setup the descriptors */
	descCfg.memRegion = memRegion; //FIXME
	descCfg.descNum = (((NUM_DESC*2)+2)*2)*nbLteCh;
	descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
	descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
	descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
	descCfg.descType = Cppi_DescType_HOST;
	descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;
	/* Descriptor should be recycled back to freeQue allocated since destQueueNum is < 0 */
	descCfg.returnQueue.qMgr = QMSS_PARAM_NOT_SPECIFIED;
	descCfg.returnQueue.qNum = QMSS_PARAM_NOT_SPECIFIED;
	descCfg.returnPushPolicy = Qmss_Location_TAIL;
	descCfg.cfg.host.returnPolicy = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
	descCfg.cfg.host.psLocation = Cppi_PSLoc_PS_IN_DESC;

	/* Initialize the descriptors and push to host free Queue */
	if ((freeHostQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
	{
		printf ("Initializing host descriptor error code: %d \n", freeHostQueHnd);
	}
	else
		printf ("Number of host descriptors requested : %d. Number of descriptors allocated : %d \n",
			descCfg.descNum, numAllocated);



	for (k=0;k<nbLteCh;k++)
	{
		/***********************************************************************
		 ********************** Transmit Queues Configuration ******************
		 ***********************************************************************/

		/* Open transmit free descriptor queue */
		if ((txFq[k] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
		{
			printf ("Opening Transmit free descriptor queue\n");
			return;
		}

		if ((txCmplQueHnd[k] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, CPPI_COMPLETION_QUE_NUM+k, &isAllocated)) < 0)
		{
			printf ("Opening Tx Completion Queue Number\n");
		}
		else
			printf ("Tx Completion Queue Number : %d opened\n", txCmplQueHnd[k]);
			queueInfo = Qmss_getQueueNumber(txFq[k]);
			psData = 0xAA;

		/* Pop off all descriptors from the Queue and add the missing init parameters. */
		for (j = 0; j < NUM_DESC*2; j++)
		{
			/* Get a host descriptor from the free host queue. */
			ptrHostDesc = (Cppi_HostDesc *)Qmss_queuePop(freeHostQueHnd);
			if (NumSeg[j] == 2)
				ptrLinkHostDesc = (Cppi_HostDesc *)Qmss_queuePop(freeHostQueHnd);

			/* Set return queue */
			queueInfo = Qmss_getQueueNumber(txCmplQueHnd[k]);
			Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, queueInfo);

			/* Set PS data */
			Cppi_setPSLocation (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, Cppi_PSLoc_PS_IN_DESC);

			Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t *) &psData, SIZE_PS_DATA);

			/* Add data buffer */
			Cppi_setData(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(uint8_t*) UTILS_local2GlobalAddr((uint32_t)&txBuffer[Seg1Offset[j]+(5120*k)]), (Seg1Len[j]*4));

			/* Save original buffer information */
			Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc, (uint8_t *) UTILS_local2GlobalAddr((uint32_t)&txBuffer[Seg1Offset[j]+(5120*k)]), (Seg1Len[j]*4));

			Cppi_setReturnPolicy (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc,(Cppi_ReturnPolicy) 1);

			Cppi_setReturnPushPolicy (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc,(Qmss_Location) 0);

			/****************************************************************
			 * 					Link descriptor init						*
			 * **************************************************************/
			if (NumSeg[j] == 2)
			{
				Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (Cppi_Desc*)ptrLinkHostDesc);

				/* Add data buffer */
				Cppi_setData(Cppi_DescType_HOST, (Cppi_Desc*)ptrLinkHostDesc,(uint8_t*) UTILS_local2GlobalAddr((uint32_t)&txBuffer[(5120*k)]), (Seg2Len[j]*4));

				/* Save original buffer information */
				Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc*)ptrLinkHostDesc,(uint8_t*) UTILS_local2GlobalAddr((uint32_t)&txBuffer[(5120*k)]), (Seg2Len[j]*4));

				Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrLinkHostDesc, (Cppi_Desc*)NULL);
			} else {
				Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (Cppi_Desc*)NULL);
			}

			/* Set packet length */
			Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc*) ptrHostDesc, ((Seg1Len[j] + Seg2Len[j])*4));

			descTag.destTagHi = 0;					descTag.destTagLo = 0;
			descTag.srcTagHi  = 0;					descTag.srcTagLo  = k;
			Cppi_setTag(Cppi_DescType_HOST,(Cppi_Desc *)ptrHostDesc,&descTag);
			/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
			Qmss_queuePushDescSize(txFq[k], (uint32_t*)ptrHostDesc, SIZE_HOST_DESC);
		}


		/* Open receive free descriptor queue */
		if ((rxFq[k] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
		{
			printf ("Opening Transmit free descriptor queue\n");
		}

		/* Get the Queue Information for the Transmit Free Queue */
		queueInfo = Qmss_getQueueNumber(rxFq[k]);

		/* Pop off all descriptors from the Queue and add the missing init parameters. */
		for (j = 0; j < NUM_DESC*2; j++)
		{
			if (j == 0)
				dataPtr = &rxBuffer[13168+((15360*k))];
			else if(j == 1 || j == 8)
				dataPtr = &rxBuffer[(15360*k)];
			else
				dataPtr += (Seg1Len[j-1] + Seg2Len[j-1]);
			/* Get a host descriptor from the free host queue. */
			ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(freeHostQueHnd));

			Cppi_setReturnQueue(Cppi_DescType_HOST,(Cppi_Desc*)ptrHostDesc,queueInfo);

			Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t *) UTILS_local2GlobalAddr ((uint32_t) dataPtr), ((Seg1Len[j] + Seg2Len[j])*4));

			Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t *) UTILS_local2GlobalAddr ((uint32_t) dataPtr), ((Seg1Len[j] + Seg2Len[j])*4));

			Qmss_queuePushDesc(rxFq[k], (uint32_t*)ptrHostDesc);
		}



		/***********************************************************************
		 ********************** PktDma channel Configuration *******************
		 ***********************************************************************/

		/* Open the AIF2 Receive Channel and keep it disabled */
		memset(&rxCfg, 0, sizeof(rxCfg));
		rxCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
		rxCfg.rxEnable   = Cppi_ChState_CHANNEL_DISABLE;
		rxCh = Cppi_rxChannelOpen(cppiHnd, &rxCfg, &isAllocated);
		if (rxCh == NULL)
		{
			printf("Error: Opening AIF2 Rx channel %d failed\n", rxCfg.channelNum);
		} else {
			printf ("Opened Rx channel : %d\n", Cppi_getChannelNumber (rxCh));
		}

		/* Open the AIF2 Transmit Channel and keep it disabled. */
		memset(&txCfg, 0, sizeof(txCfg));
		txCfg.channelNum   = CPPI_PARAM_NOT_SPECIFIED;
		txCfg.priority     = 0;
		txCfg.txEnable     = Cppi_ChState_CHANNEL_DISABLE;
		txCfg.filterEPIB   = 0;
		txCfg.filterPS     = 0;
		txCfg.aifMonoMode  = 0;
		txCh = Cppi_txChannelOpen(cppiHnd, &txCfg, &isAllocated);
		if (txCh == NULL)
		{
			printf("Error: Opening AIF2 Tx channel %d failed\n", txCfg.channelNum);
		} else {
			printf ("Opened Tx channel : %d\n", Cppi_getChannelNumber (txCh));
		}

		/* Open the AIF2 transmit queue for this control stream */
		txQ[k] = Qmss_queueOpen (Qmss_QueueType_INFRASTRUCTURE_QUEUE , QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
		if (txQ[k] < 0)
		{
			printf("Error: AIF2 Transmit Queue failed to open\n");
		}

		/* Open the AIF2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
		rxQ[k] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
		if (rxQ[k] < 0)
		{
			printf("Error: AIF2 Receive Queue failed to open\n");
		}

		printf ("Free Queue Number : %d opened\n", rxFq[k]);
		printf ("Transmit Free Queue Number : %d opened\n", txFq[k]);
		printf ("Receive Queue Number : %d opened\n", rxQ[k]);
		printf ("Transmit Queue Number : %d opened\n", txQ[k]);

		/***********************************************************************
		 ********************** Receive Flow Configuration *********************
		 ***********************************************************************/
		memset(&rxFlowCpriDM, 0, sizeof(Cppi_RxFlowCfg));
		rxFlowCpriDM.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;
		queueInfo = Qmss_getQueueNumber(rxQ[k]);
		rxFlowCpriDM.rx_dest_qnum     = queueInfo.qNum; //HOST_RX_Q;
		rxFlowCpriDM.rx_dest_qmgr	  = queueInfo.qMgr;
		queueInfo = Qmss_getQueueNumber(rxFq[k]);
		rxFlowCpriDM.rx_fdq0_sz0_qnum = queueInfo.qNum; //HOST_RX_FDQ;
		rxFlowCpriDM.rx_fdq0_sz0_qmgr = queueInfo.qMgr;
		rxFlowCpriDM.rx_desc_type     = (uint8_t)Cppi_DescType_HOST;    // HOST
		rxFlowCpriDM.rx_psinfo_present = 1;
		rxFlowCpriDM.rx_ps_location   = 0;
		rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCpriDM, &isAllocated);
		if (rxFlowHnd == NULL)
		{
			printf ("Opening Rx flow : %d\n", rxFlowCpriDM.flowIdNum);
		}
		else
			printf ("Opened Rx flow : %d\n", Cppi_getFlowId(rxFlowHnd));

		/* Enable transmit channel */
		if (Cppi_channelEnable (txCh) != CPPI_SOK)
		{
			printf ("Error enabling Tx channel : %d\n", Cppi_getChannelNumber (txCh));
		}
		else
			printf ("Tx channel : %d enabled \n", Cppi_getChannelNumber (txCh));

		/* Enable receive channel */
		if (Cppi_channelEnable (rxCh) != CPPI_SOK)
		{
			printf ("Error enabling Rx channel : %d\n", Cppi_getChannelNumber (rxCh));
		}
		else
			printf ("Rx channel : %d enabled \n", Cppi_getChannelNumber (rxCh));

		printf ("\n-------------------------Queue status-------------------------\n");
		result = Qmss_getQueueEntryCount (txQ[k]);
		printf ("Transmit Queue %d Entry Count : %d \n", txQ[k], result);

		result = Qmss_getQueueEntryCount (txFq[k]);
		printf ("Tx Free Queue %d Entry Count : %d \n", txFq[k], result);

		result = Qmss_getQueueEntryCount (rxFq[k]);
		printf ("Rx Free Queue %d Entry Count : %d \n", rxFq[k], result);

		result = Qmss_getQueueEntryCount (rxQ[k]);
		printf ("Receive Queue %d Entry Count : %d \n", rxQ[k], result);

		result = Qmss_getQueueEntryCount (txCmplQueHnd[k]);
		printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd[k], result);
	}

}

extern Cppi_Handle hCppi;
extern uint32_t 	synchronize;
int32_t
UTILS_initPktDma(
		AIF_PktDmaConfigHandle hPktDma
)
{
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
	if (hPktDma->psMsgEnable)
		psMsgEnable = 1;

	/* Initialize the AIF2 CPDMA config structure. */
	memset ((void *)&aif2CPDMACfg, 0, sizeof(Cppi_CpDmaInitCfg));

	/* Setup the AIF2 CPDMA Configuration. */
	aif2CPDMACfg.dmaNum = Cppi_CpDma_AIF_CPDMA;

	if (hPktDma->mode != AIF_LTE_FDD_MODE && hPktDma->mode != AIF_LTE_TDD_MODE && hPktDma->mode != AIF_GENERICPACKET_MODE) //FIXME
	{
		aif2CPDMACfg.writeFifoDepth = 8;
	}

	/* Open the AIF2 CPDMA. Needed even if DIO used, as we need to disable PKTDMA loopback mode */
	hPktDma->hCppi = Cppi_open (&aif2CPDMACfg);

	/* Disable the AIF2-CPDMA Loopback mode in the CPPI */
	Cppi_setCpdmaLoopback (hPktDma->hCppi, 0);

    /* If PktDma mode and/or ctrl messages enabled, open and populate free queues, open tx/rx queues, configure flows  */
    if (psMsgEnable == 1)
    {
		/***********************************************************************
		 ********************** Transmit Configuration *************************
		 ***********************************************************************/
    	for(i=0; i< AIF2_CPRI_MAX_CW_SUBSTREAM; i++)
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
					printf("Error: AIF2 Transmit Completion Queue failed to open\n");
					return -1;
				}
				/* Get the Queue Information for the Transmit Free Queue */
				queueInfo = Qmss_getQueueNumber(hPktDma->txFqCtrl[i]);

				/* Pop off all descriptors from the Queue and add the missing init parameters. */
				for (j = 0; j < hPktDma->txNumDescCtrl[i]; j++)
				{
					/* Get a mono descriptor from the free GP queue. */
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[i]));

					Cppi_setPacketLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,hPktDma->txDescSizeCtrl[i]);
					descTag.srcTagLo = 124+i;
					Cppi_setTag(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,&descTag);
					Cppi_setPSFlags(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
					Cppi_setPSLen(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,0);
					Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);

					/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
					Qmss_queuePushDesc(hPktDma->txFqCtrl[i], (uint32_t*)ptrMonoDesc);
				}

				/* Open the AIF2 transmit queue for this control stream */
				hPktDma->txQCtrl[i] = Qmss_queueOpen (Qmss_QueueType_AIF_QUEUE , QMSS_AIF_QUEUE_BASE+124+i, &isAllocated);
				if (hPktDma->txQCtrl[i] < 0)
				{
					printf("Error: AIF2 Transmit Queue failed to open\n");
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
					printf("Error: AIF2 Receive Completion Queue failed to open\n");
					return -1;
				}

				/* Get the Queue Information for the Transmit Free Queue */
				queueInfo = Qmss_getQueueNumber(hPktDma->rxFqCtrl[i]);

				/* Pop off all descriptors from the Queue and add the missing init parameters. */
				for (j = 0; j < hPktDma->rxNumDescCtrl[i]; j++)
				{
					/* Get a mono descriptor from the free GP queue. */
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxFqCtrl[i]));

					Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrMonoDesc,queueInfo);

					/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
					Qmss_queuePushDesc(hPktDma->rxFqCtrl[i], (uint32_t*)ptrMonoDesc);
				}

				/* Open the AIF2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
				hPktDma->rxQCtrl[i] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hPktDma->hRxFlowCtrl[i]->rx_dest_qnum, &isAllocated);
				if (hPktDma->rxQCtrl[i] < 0)
				{
					printf("Error: AIF2 Receive Queue failed to open\n");
					return -1;
				}


				/***********************************************************************
	        	 ********************** PktDma channel Configuration *******************
	        	 ***********************************************************************/

				/* Open the AIF2 Receive Channel and keep it disabled */
				rxCfg.channelNum = 124+i;
				rxCfg.rxEnable   = Cppi_ChState_CHANNEL_ENABLE;
				hPktDma->rxChCtrl[i] = Cppi_rxChannelOpen(hPktDma->hCppi, &rxCfg, &isAllocated);
				if (hPktDma->rxChCtrl[i] == NULL)
				{
					printf("Error: Opening AIF2 Rx channel %d failed\n", rxCfg.channelNum);
					return -1;
				}
				/* Open the AIF2 Transmit Channel and keep it disabled. */
				txCfg.channelNum   = 124+i;
				txCfg.priority     = 0;
				txCfg.txEnable     = Cppi_ChState_CHANNEL_ENABLE;
				txCfg.filterEPIB   = 0;
				txCfg.filterPS     = 0;
				txCfg.aifMonoMode  = 0;
				hPktDma->txChCtrl[i] = Cppi_txChannelOpen(hPktDma->hCppi, &txCfg, &isAllocated);
				if (hPktDma->txChCtrl[i] == NULL)
				{
					printf("Error: Opening AIF2 Tx channel %d failed\n", txCfg.channelNum);
					return -1;
				}

				/***********************************************************************
	        	 ********************** Receive Flow Configuration *********************
	        	 ***********************************************************************/
				hPktDma->hRxFlowCtrl[i]->flowIdNum = 124+i;
                Cppi_configureRxFlow (hPktDma->hCppi, hPktDma->hRxFlowCtrl[i], &isAllocated);

	        }
    	}
    }

	if ((hPktDma->mode == AIF_GENERICPACKET_MODE)){
		axcConfigTx = hPktDma->firstAxC;
		axcConfigRx = hPktDma->firstAxC;
		/***********************************************************************
		 ********************** Transmit Configuration *************************
		 ***********************************************************************/
		if (hPktDma->hRxFlowAxC[axcConfigTx] != NULL) // check for first RxFlow existing
		{
			numAxC 		= hPktDma->numAxC;
			for (k=0; k <numAxC ; k++)
			{
				/***********************************************************************
				 ********************** Transmit Queues Configuration ******************
				 ***********************************************************************/

				/* Initialize the Transmit descriptors. We want all the transmit descriptors to go
				 * back to the Transmit Free Queue after transmission. */
				descCfg.memRegion                 = hPktDma->txRegionAxC[axcConfigTx];
				descCfg.descNum                   = hPktDma->txNumDescAxC[axcConfigTx];
				descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
				descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
				descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
				descCfg.descType                  = Cppi_DescType_HOST;
				descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
				descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
				descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
				//descCfg.returnPushPolicy          = Qmss_Location_TAIL; //FIXME


				/* Initialize the descriptors and place all of them into the general purpose queue */
				hPktDma->txFqAxC[axcConfigTx]  = Cppi_initDescriptor (&descCfg, &numAllocated);
				if (hPktDma->txFqAxC[axcConfigTx] < 0)
				{
					printf("Error: AIF2 Transmit Completion Queue failed to open\n");
					return -1;
				}
				/* Get the Queue Information for the Transmit Free Queue */
				queueInfo = Qmss_getQueueNumber(hPktDma->txFqAxC[axcConfigTx]);

				/* Pop off all descriptors from the Queue and add the missing init parameters. */
				for (j = 0; j < hPktDma->txNumDescAxC[axcConfigTx]; j++)
				{
					dataPtr = hPktDma->txDataBuff[axcConfigTx] + (j * hPktDma->txDescSizeAxC[axcConfigTx] / 4);
					/* Get a mono descriptor from the free GP queue. */
					ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[axcConfigTx]));
					Cppi_setData(descCfg.descType, (Cppi_Desc*)ptrHostDesc,(uint8_t*) dataPtr, hPktDma->txDescSizeAxC[axcConfigTx]);
					Cppi_setDataLen(descCfg.descType,(Cppi_Desc*)ptrHostDesc,0);
					Cppi_setOriginalBufInfo(descCfg.descType, (Cppi_Desc*)ptrHostDesc,(uint8_t*) dataPtr, hPktDma->txDescSizeAxC[axcConfigTx]);
					Cppi_getNextBD(descCfg.descType,NULL);
					Cppi_setReturnPolicy(descCfg.descType, (Cppi_Desc*)ptrHostDesc,(Cppi_ReturnPolicy)1);
					Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrHostDesc,queueInfo);

					/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
					Qmss_queuePushDesc(hPktDma->txFqAxC[axcConfigTx], (uint32_t*)ptrHostDesc);
				}

				/* Open the AIF2 transmit queue for this control stream */
				hPktDma->txQAxC[axcConfigTx] = Qmss_queueOpen (Qmss_QueueType_AIF_QUEUE , QMSS_AIF_QUEUE_BASE+i, &isAllocated);
				if (hPktDma->txQAxC[axcConfigTx] < 0)
				{
					printf("Error: AIF2 Transmit Queue failed to open\n");
					return -1;
				}

				/* Open the AIF2 Transmit Channel and keep it disabled. */
				memset(&txCfg, 0, sizeof(txCfg));
				txCfg.channelNum   = i;
				txCfg.priority     = 0;
				txCfg.txEnable     = Cppi_ChState_CHANNEL_ENABLE;
				txCfg.filterEPIB   = 0;
				txCfg.filterPS     = 0;
				txCfg.aifMonoMode  = 0;
				hPktDma->txChAxC[axcConfigTx] = Cppi_txChannelOpen(hPktDma->hCppi, &txCfg, &isAllocated);
				if (hPktDma->txChAxC[axcConfigTx] == NULL)
				{
					printf("Error: Opening AIF2 Tx channel %d failed\n", txCfg.channelNum);
					return -1;
				}

				axcConfigTx++;
			}

			/***********************************************************************
			 ********************** Receive Queues Configuration *******************
			 ***********************************************************************/

			for (k=0; k <numAxC ; k++)
			{
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

				/* Initialize the descriptors and place all of them into the general purpose queue */
				hPktDma->rxFqAxC[axcConfigRx]  = Cppi_initDescriptor (&descCfg, &numAllocated);
				if (hPktDma->rxFqAxC[axcConfigRx] < 0)
				{
					printf("Error: AIF2 Receive Completion Queue failed to open\n");
					return -1;
				}

				/* Get the Queue Information for the Transmit Free Queue */
				queueInfo = Qmss_getQueueNumber(hPktDma->rxFqAxC[axcConfigRx]);

				/* Pop off all descriptors from the Queue and add the missing init parameters. */
				for (j = 0; j < hPktDma->rxNumDescAxC[axcConfigRx]; j++)
				{
					dataPtr = hPktDma->rxDataBuff[axcConfigRx] + (j * hPktDma->txDescSizeAxC[axcConfigRx] / 4);
					/* Get a mono descriptor from the free GP queue. */
					ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxFqAxC[axcConfigRx]));
					Cppi_setDataLen(descCfg.descType,(Cppi_Desc*)ptrHostDesc,0);
					Cppi_setOriginalBufInfo(descCfg.descType, (Cppi_Desc*)ptrHostDesc, (uint8_t*) dataPtr, hPktDma->txDescSizeAxC[axcConfigRx]);
					Cppi_getNextBD(descCfg.descType,NULL);
					Cppi_setReturnQueue(descCfg.descType,(Cppi_Desc*)ptrHostDesc,queueInfo);

					/* Add the packet descriptor to the AIF2 Transmit Free Queue. */
					Qmss_queuePushDesc(hPktDma->rxFqAxC[axcConfigRx], (uint32_t*)ptrHostDesc);
				}

				/* Open the AIF2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
				hPktDma->rxQAxC[axcConfigRx] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hPktDma->hRxFlowAxC[axcConfigRx]->rx_dest_qnum, &isAllocated);
				if (hPktDma->rxQAxC[axcConfigRx] < 0)
				{
					printf("Error: AIF2 Receive Queue failed to open\n");
					return -1;
				}

				/***********************************************************************
				 ********************** PktDma channel Configuration *******************
				 ***********************************************************************/

				/* Open the AIF2 Receive Channel and keep it disabled */
				memset(&rxCfg, 0, sizeof(rxCfg));
				rxCfg.channelNum = axcConfigRx;
				rxCfg.rxEnable   = Cppi_ChState_CHANNEL_ENABLE;
				hPktDma->rxChAxC[axcConfigRx] = Cppi_rxChannelOpen(hPktDma->hCppi, &rxCfg, &isAllocated);
				if (hPktDma->rxChAxC[axcConfigRx] == NULL)
				{
					printf("Error: Opening AIF2 Rx channel %d failed\n", rxCfg.channelNum);
					return -1;
				}

				/***********************************************************************
				 ********************** Receive Flow Configuration *********************
				 ***********************************************************************/
				hPktDma->hRxFlowAxC[axcConfigRx]->flowIdNum = i;
				Cppi_configureRxFlow (hPktDma->hCppi, hPktDma->hRxFlowAxC[axcConfigRx], &isAllocated);
			}
		}
	}
	if ((hPktDma->mode == AIF_LTE_FDD_MODE) || (hPktDma->mode == AIF_LTE_TDD_MODE)){
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
						printf("Error: AIF2 Transmit Completion Queue failed to open\n");
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
						Qmss_queuePushDesc(hPktDma->txFqAxC[axcConfigTx], (uint32_t*)ptrMonoDesc);
						}

					/* Open the AIF2 transmit queue for this AxC stream */
					hPktDma->txQAxC[axcConfigTx] = Qmss_queueOpen (Qmss_QueueType_AIF_QUEUE , QMSS_AIF_QUEUE_BASE+axcConfigTx, &isAllocated);
					if (hPktDma->txQAxC[axcConfigTx] < 0)
						{
						printf("Error: AIF2 Transmit Queue failed to open\n");
						return -1;
						}
				}
				axcConfigTx++;
			} ///eof Transmit config


			/***********************************************************************
			 ********************** Receive Queues Configuration *******************
			 ***********************************************************************/

			for (k=0; k <numAxC ; k++){
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
						printf("Error: AIF2 Receive Completion Queue failed to open\n");
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
						Qmss_queuePushDesc(hPktDma->rxFqAxC[axcConfigRx], (uint32_t*)ptrMonoDesc);
						}

					//Qmss_QueueType_GENERAL_PURPOSE_QUEUE to be verified

					/* Open the AIF2 transmit queue for this control stream, queueType does not matter since we specify a valid queue number */
					hPktDma->rxQAxC[axcConfigRx] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE , hPktDma->hRxFlowAxC[axcConfigRx]->rx_dest_qnum, &isAllocated);
					if (hPktDma->rxQAxC[axcConfigRx] < 0)
						{
						printf("Error: AIF2 Receive Queue failed to open\n");
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
	}
    return 0;

}

extern uint32_t numcore2sync;
void
UTILS_initCppiChannel(AIF_CfgHandle hAifCfgObj, AIF_PktDmaConfigHandle hPktDma)
{
    Cppi_RxChInitCfg     rxCfg;
    Cppi_TxChInitCfg     txCfg;
    uint32_t 				 i,j,k,numCh;
    uint8_t                isAllocated;

    numCh = 0;
    for (i=0; i<numcore2sync; i++)
    {
    	for (j=0;j<AIF_MAX_NUM_LINKS;j++)
    	{
    		if (hAifCfgObj->coreCfg[i].linkEnable[j])
    			numCh += hAifCfgObj->coreCfg[i].numAxC[j];
    	}
    }

	/***********************************************************************
	 ********************** PktDma channel Configuration *******************
	 ***********************************************************************/
    for (k=0; k<numCh; k++)
    {
		/* Open the AIF2 Transmit Channel and keep it disabled. */
		memset(&txCfg, 0, sizeof(txCfg));
		txCfg.channelNum   = k;
		txCfg.priority     = 0;
		txCfg.txEnable     = Cppi_ChState_CHANNEL_ENABLE;
		txCfg.filterEPIB   = 0;
		txCfg.filterPS     = 0;
		/* It is possible for the FFTC to send output packets directly to the AIF2 without any DSP intervention. In that case, the AIF_MONO_MODE should
		 * be turned off and FFTC uses a descriptor size field of 16 bytes as normal mode, because FFTC can not set 64 bytes to DESC_SIZE field before
		 * pushing the descriptor into the AIF2 TX queue.
		 */
		txCfg.aifMonoMode  = 0;


		if (Cppi_txChannelOpen(hPktDma->hCppi, &txCfg, &isAllocated) == NULL)
			{
			printf("Error: Opening AIF2 Tx channel %d failed\n", txCfg.channelNum);
			return;
			}

		/***********************************************************************
		 ********************** PktDma channel Configuration *******************
		 ***********************************************************************/

		/* Open the AIF2 Receive Channel and keep it disabled */
		memset(&rxCfg, 0, sizeof(rxCfg));
		rxCfg.channelNum = k;
		rxCfg.rxEnable   = Cppi_ChState_CHANNEL_ENABLE;
		if (Cppi_rxChannelOpen(hPktDma->hCppi, &rxCfg, &isAllocated) == NULL)
		{
			printf("Error: Opening AIF2 Rx channel %d failed\n", rxCfg.channelNum);
			return;
		}
    }
}


void Utils_populatePktDmaCfgObj(
		AIF_CoreCfgHandle      	ptrAifCoreCfg,
		AIF_PktDmaConfigHandle 	hPktDma,
		uint8_t*					region,
		uint32_t 					lteSymbolSize,
		uint32_t 					nbSymbol,
		uint32_t 					rxQ,
		uint32_t 					rxFq,
		Cppi_RxFlowCfg			*rxFlow
)
{
	uint32_t chan, i, idx;

	chan = 0;
	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
	{
		if (ptrAifCoreCfg->linkEnable[i] == 1)
		{
			chan = ptrAifCoreCfg->firstAxCIndex[i];
			hPktDma->firstAxC 		= ptrAifCoreCfg->firstAxCIndex[i];
			hPktDma->numAxC 		= ptrAifCoreCfg->numAxC[i];
			hPktDma->mode 			= ptrAifCoreCfg->mode;
			// PktDma parameters
			for (idx=0 ; idx<(ptrAifCoreCfg->numAxC[i]) ; idx++)
			{
				hPktDma->txRegionAxC[chan]   	= UTILS_getMemRegionNum(region); //ptrCfg->memRegionResponse[0].memRegionHandle;
				hPktDma->txNumDescAxC[chan]  	= nbSymbol*2; // double num of Pkts
				hPktDma->txDescSizeAxC[chan] 	= lteSymbolSize;
				hPktDma->rxRegionAxC[chan]   	= UTILS_getMemRegionNum(region); //ptrCfg->memRegionResponse[0].memRegionHandle;
				hPktDma->rxNumDescAxC[chan]  	= nbSymbol*2; // double num of Pkts
				hPktDma->rxDescSizeAxC[chan] 	= lteSymbolSize;

				memset(&rxFlow[chan], 0, sizeof(Cppi_RxFlowCfg));
				rxFlow[chan].rx_dest_qnum     	= rxQ+chan;
				rxFlow[chan].rx_fdq0_sz0_qnum 	= rxFq+chan;
				rxFlow[chan].rx_desc_type     	= (uint8_t)Cppi_DescType_MONOLITHIC;    // MONO
				rxFlow[chan].rx_fdq1_qnum     	= rxFq+chan;
				rxFlow[chan].rx_fdq2_qnum     	= rxFq+chan;
				rxFlow[chan].rx_fdq3_qnum     	= rxFq+chan;
				rxFlow[chan].rx_psinfo_present 	= 1;
		//		rxFlow[i][idx].rx_ps_location   = 1;
				rxFlow[chan].rx_sop_offset    	= 12+4;   // desc header size for Monolithic packet with PS

				hPktDma->hRxFlowAxC[chan]   	= &rxFlow[chan];
				hPktDma->hRxFlowCtrl[chan]  	= NULL;
				chan++;
			}
		}
	}
	return;
}

////////////////////
