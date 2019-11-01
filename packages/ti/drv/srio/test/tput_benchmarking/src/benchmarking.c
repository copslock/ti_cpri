/*
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 *
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

/**
 *   @file  benchmark.c
 *
 *   @brief   
 *      Main file for benchmarking SRIO throughput and latency. This code is
 *      designed to use one core for the Consumer (Receive side) and one core
 *      for the Producer (Transmit side). The Consumer side is started first
 *      and then the Producer side is started. The Producer side is designed to
 *      transmit as fast as it can with the Consumer side monitoring and letting
 *      the Producer know when packets have been dropped. When packets are
 *      dropped the Producer will go into a pacing search mode to determine
 *      the minimum delay needed to transfer packets without losing any. This
 *      main file calls the Consumer and Producer functions.
 *
 */

/* Standard library, XDC and sysbios Include Files. */
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/family/c66/tci66xx/CpIntc.h>
#include <ti/sysbios/BIOS.h>

/* IPC includes */ 
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MultiProc.h>

/* CSL Chip Functional Layer */
#include <ti/csl/csl_chip.h>

/* CSL Cache Functional Layer */
#include <ti/csl/csl_cacheAux.h>

/* CSL PSC Include Files */
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>
#include <ti/csl/csl_device_interrupt.h>
/* SRIO Driver Include File. */
#include <srio_osal.h>
#include <ti/drv/srio/srio_drv.h>

/* Application Include Files */
#include <srio_laneconfig.h>
#include <srio_tput.h>

/* Producer-Consumer Include Files. */
#include <benchmarking.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

/* CSL TSC Module */
#include <ti/csl/csl_tsc.h>

#if !defined(CSL_CIC0_SRIO_INTDST0)
#define CSL_CIC0_SRIO_INTDST0   CSL_INTC0_INTDST0   
#endif

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

/* There is a sRIO related issue in the CDMA where if the payload fills the buffer exactly
 * (such as a 4KB message into a 4KB buffer) it will chain a second buffer that will not
 * contain any data. The work around is to add an extra byte beyond the maximum payload
 * size expected to the receive buffer size to avoid an exact fill situation. */
#define RX_BUFFER_BYTES_ADJUSTMENT	1

/* These are the srio device identifiers used in the Application */
const uint32_t DEVICE_ID1_16BIT    = CONSUMER_16BIT_DEVICE_ID;
const uint32_t DEVICE_ID1_8BIT     = CONSUMER_8BIT_DEVICE_ID;
const uint32_t DEVICE_ID2_16BIT    = PRODUCER_16BIT_DEVICE_ID;
const uint32_t DEVICE_ID2_8BIT     = PRODUCER_8BIT_DEVICE_ID;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Global test control variables */
struct_testcontrol testControl = {
		SRIO_LANE_SPEED,					// srioLaneRateGbps_e	srio_laneSpeedGbps;
		SRIO_PORT_WIDTH,					// srioLanesMode_e		srio_lanesMode;
		TESTS_TO_RUN,						// srioTestsToRun_e		srio_testsToRun;
		NUM_SECS_TO_RUN_EACH_PACKET_SIZE,	// uint32_t				srio_testTimeInSeconds;
		CORE_TO_INITIALIZE_SRIO,			// int32_t				srio_initCorenum;
		IS_BOARD_TO_BOARD,					// bool					srio_isBoardToBoard;
		IS_OVER_EXTERNAL_SRIO_SWITCH,		// bool					srio_isExternalSrioSwitch;
		USE_LOOPBACK_MODE,					// bool					srio_isLoopbackMode;
		USE_SRIO_16_BIT_ID,					// bool					srio_isDeviceID16Bit;
		RUN_LATENCY_MEASUREMENT,			// bool					srio_isLatencyTest;
		DISPLAY_RUN_PROGRESS_MESSAGES,		// bool					srio_isRunProgress;
		(MESSAGE_INITIAL_DATA_SIZE - SOFTWARE_HEADER_BYTES),	// int32_t				srio_payloadSize;
		(MESSAGE_MAX_DATA_SIZE - SOFTWARE_HEADER_BYTES),		// int32_t				srio_payloadEndSize;
		DIO_INITIAL_DATA_SIZE,				// int32_t				srio_dioPayloadSize;
		DIO_MAX_DATA_SIZE,					// int32_t				srio_dioPayloadEndSize;
};

/* Application variables */
uint64_t srio_numPackets = (uint64_t)MIN_NUM_PACKETS_TO_SEND;		/* Initial value only. Determined by the test seconds specified */
uint64_t srio_numDioPackets = (uint64_t)MIN_NUM_PACKETS_TO_SEND;	/* Initial value only. Determined by the test seconds specified */
uint64_t srio_latencyNumPackets = (uint64_t)100;					/* Static value */
uint32_t srio_iterationCount = 10;	/* Initial value only. Determined by the test seconds specified */
uint32_t srio_device_ID1;			/* Consumer. Value is set in main */
uint32_t srio_device_ID2;			/* Producer. Value is set in main */

/* Code has only been tested in polled mode, interrupt mode is not guaranteed or expected to work. */
bool srio_usePolledMode = TRUE;


/* Memory allocated for the descriptors. This is 16 bit aligned. */
#pragma DATA_ALIGN (host_region_consumer, 16)
uint8_t             host_region_consumer[NUM_HOST_DESC * SIZE_HOST_DESC];

/* Memory used for the accumulator list. */
#pragma DATA_ALIGN (gHiPriAccumList, 16)
uint32_t            gHiPriAccumList[64];

/* Shared Memory Variable to ensure synchronizing SRIO initialization
 * with all the other cores. */
/* Created an array to pad the cache line with SRIO_MAX_CACHE_ALIGN size */
#pragma DATA_ALIGN   (isSRIOInitialized, 128)
#pragma DATA_SECTION (isSRIOInitialized, ".srioSharedMem");
volatile Uint32     isSRIOInitialized[(SRIO_MAX_CACHE_ALIGN / sizeof(Uint32))] = { 0 };

/* Consumer Management Socket. */
Srio_SockHandle     mgmtSocket;

/* Global SRIO Driver Handle. */
Srio_DrvHandle      hAppManagedSrioDrv;

/* Memory Heap Queue which has descriptors and buffers allocated and linked together. */
Qmss_QueueHnd       memoryHeapQueue;


/**********************************************************************
 ************************ External Definitions ************************
 **********************************************************************/

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/* SRIO Device Initialization Function */
extern int32_t SrioDevice_init (void);


/**********************************************************************
 ************* COMMON INITIALIZATION AND SETUP FUNCTIONS **************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      Utility function which converts a local address to global.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Global Address
 */
static uint32_t l2_global_address (uint32_t addr)
{
	uint32_t coreNum;

	/* Get the core number. */
	coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

	/* Compute the global address. */
	return (addr + (0x10000000 + (coreNum*0x1000000)));
}

/**
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for SRIO. 
 *      The SRIO power domain is turned OFF by default.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t enable_srio (void)
{
    /* Set SRIO Power domain to ON */
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_SRIO);

    /* Enable the clocks too for SRIO */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_SRIO, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_SRIO);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_SRIO));

    /* Return SRIO PSC status */
    if ((CSL_PSC_getPowerDomainState(CSL_PSC_PD_SRIO) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (CSL_PSC_LPSC_SRIO) == PSC_MODSTATE_ENABLE))
    {
        /* SRIO ON. Ready for use */
        return 0;
    }
    else
    {
        /* SRIO Power on failed. Return error */
        return -1;
    }
}

/**
 *  @b Description
 *  @n  
 *      Application Raw Receive Cleanup API.
 *
 *  @retval
 *      Not Applicable.
 */
static void myAppRawRxFree (Srio_DrvBuffer hDrvBuffer)
{
    Qmss_QueueHnd       returnQueueHnd;
    Cppi_HostDesc*      ptrHostDesc;
 
    /* Get the pointer to the descriptor. */
    ptrHostDesc = (Cppi_HostDesc*)hDrvBuffer;
 
    /* Cycle through all the linked descriptors and clean them up. */
    while (ptrHostDesc != NULL)
    {
        /* Get the return queue. */
        returnQueueHnd = Qmss_getQueueHandle (Cppi_getReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc));
 
        /* Push the descriptor into the return queue. */
        Qmss_queuePushDesc (returnQueueHnd, (Ptr)ptrHostDesc);
 
        /* Get the next linked descriptor. */
        ptrHostDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST,(Cppi_Desc*)ptrHostDesc);
    }
}

/**
 *  @b Description
 *  @n  
 *      The function initializes and setups the SRIO Driver configuration
 *      to operate in application based mode.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t setupSRIOAppConfig (Qmss_MemRegion memRegion)
{
    Qmss_QueueHnd   myRxFreeQueueHnd;
    Qmss_QueueHnd   myRxCompletionQueueHnd;
    Qmss_QueueHnd   tmpQueueHnd;
    uint32_t        numAllocated;
    uint8_t         isAllocated;
    Cppi_DescCfg    descCfg;
    uint16_t        index;
    Cppi_HostDesc*  ptrHostDesc;
    uint8_t*        ptrRxData;
    Srio_DrvConfig  appCfg;
    Error_Block	    errorBlock;
    Qmss_Queue      queueInfo;
    int32_t         coreToQueueSelector[4];
	uint32_t coreNum;

	/* Get the core number. */
	coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Initialize the SRIO Driver Configuration. */
    memset ((Void *)&appCfg, 0, sizeof(Srio_DrvConfig));

    /* Create the application receive free queue. */
    myRxFreeQueueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED,
                                       &isAllocated);
    if (myRxFreeQueueHnd < 0)
	{
	    System_printf ("Error: Unable to create receive queues.\n");
	    return -1;
    }    

    /* This is the table which maps the core to a specific receive queue for interrupt support. */
    if (srio_usePolledMode)
    {
        /* Create the application receive completion queue. */
        myRxCompletionQueueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 
                                                 QMSS_PARAM_NOT_SPECIFIED, 
                                                 &isAllocated);        
        if (myRxCompletionQueueHnd < 0)
	    {
	        System_printf ("Error: Unable to create the receive completion queue.\n");
    	    return -1;
        }
    }
    else
    {
    	/* High priority accumulation and interrupt support queues */
		coreToQueueSelector[0] = HIGH_PRIORITY_ACC_INT_QUEUE_BASE;
		coreToQueueSelector[1] = HIGH_PRIORITY_ACC_INT_QUEUE_BASE + 1;
		coreToQueueSelector[2] = HIGH_PRIORITY_ACC_INT_QUEUE_BASE + 2;
		coreToQueueSelector[3] = HIGH_PRIORITY_ACC_INT_QUEUE_BASE + 3;

        /* Create the application receive completion queue. */
        myRxCompletionQueueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 
                                                 coreToQueueSelector[CSL_chipReadReg(CSL_CHIP_DNUM)], 
                                                 &isAllocated);
        if (myRxCompletionQueueHnd < 0)
	    {
	        System_printf ("Error: Unable to create the receive completion queue.\n");
    	    return -1;
        }
    }

    /* Application created queue which stores all the receive buffers. */
    memset(&descCfg,0,sizeof(Cppi_DescCfg));
    descCfg.memRegion                 = memRegion;
	descCfg.descNum                   = ((coreNum == 0) ? NUM_RX_DESC : NUM_RX_DESC_ON_TX_SIDE);
	descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
	descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
	descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
	descCfg.descType                  = Cppi_DescType_HOST;
	descCfg.returnQueue               = Qmss_getQueueNumber (myRxFreeQueueHnd);
	descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.returnPushPolicy          = Qmss_Location_HEAD;
	descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
	descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

    /* Initialize the descriptors. */
	tmpQueueHnd = Cppi_initDescriptor (&descCfg, &numAllocated);
	if (tmpQueueHnd < 0)
	    return -1;

    /* Initialize the application receive buffers. */
    for (index = 0; index < descCfg.descNum; index++)
    {
	    /* Pop off a descriptor */
	    ptrHostDesc = (Cppi_HostDesc *)Qmss_queuePop(tmpQueueHnd);
	    if (ptrHostDesc == NULL)
	        return -1;
	
	    /* Allocate the receive buffer where the data will be received into by the SRIO CPDMA. */
	    /* Make sure to adjust the RX buffer size to work around the SRIO CPDMA issue.         */
	    ptrRxData = (uint8_t*)Memory_alloc (NULL, (MESSAGE_MAX_DATA_SIZE+RX_BUFFER_BYTES_ADJUSTMENT), 0, &errorBlock);
	    if (ptrRxData == NULL)
	        return -1;

        /* Convert the address to a global address. */
        ptrRxData = (uint8_t*)l2_global_address((uint32_t)ptrRxData);        

        /* Set the DATA and ORIGNAL DATA in the buffer descriptor. */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t*)ptrRxData, (MESSAGE_MAX_DATA_SIZE+RX_BUFFER_BYTES_ADJUSTMENT));
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t*)ptrRxData, (MESSAGE_MAX_DATA_SIZE+RX_BUFFER_BYTES_ADJUSTMENT));

        /* Debug Message: */
        debugPrintf_pp ("Debug: Rx Free Descriptor 0x%p with DataBuffer 0x%p\n", ptrHostDesc, ptrRxData);
        /* Add the packet descriptor to the Application Receive Free Queue. */
	    Qmss_queuePushDesc (myRxFreeQueueHnd, (uint32_t*)ptrHostDesc);
    }

    /* Close the temporary queue. */
    Qmss_queueClose (tmpQueueHnd);

    /* Setup the SRIO Driver Configuration: This is application managed configuration */
    appCfg.bAppManagedConfig = TRUE;

    /* Get the queue information about the receive completion queue. */
    queueInfo = Qmss_getQueueNumber (myRxCompletionQueueHnd);

    /* The application managed configuration is capable of reception. */
    appCfg.u.appManagedCfg.bIsRxFlowCfgValid = 1;

    /* Configure the Receive Flow */
	appCfg.u.appManagedCfg.rxFlowCfg.flowIdNum          = -1;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_qnum       = queueInfo.qNum;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_qmgr       = queueInfo.qMgr;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_sop_offset      = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_ps_location     = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_desc_type       = 0x1; /* Host Descriptor. */
	appCfg.u.appManagedCfg.rxFlowCfg.rx_error_handling  = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_psinfo_present  = 0x1; /* PS Information */
	appCfg.u.appManagedCfg.rxFlowCfg.rx_einfo_present   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo     = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi     = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo      = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi      = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo_sel = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi_sel = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo_sel  = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi_sel  = 0x0;

    /* Disable Receive size thresholds. */
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0_en = 0x0;
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1_en = 0x0;
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2_en = 0x0;

	/* Use the Application Receive Free Queue for picking all descriptors. */
	queueInfo = Qmss_getQueueNumber (myRxFreeQueueHnd);
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qnum       = queueInfo.qNum;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qmgr       = queueInfo.qMgr;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qnum       = 0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qmgr       = 0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qnum       = 0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qmgr       = 0;

    /* Use the Receive Queue for picking the SOP packet also. */
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qnum   = queueInfo.qNum;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qmgr   = queueInfo.qMgr;

    /* There are no size thresholds configured. */
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0    = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1    = 0x0;
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2    = 0x0;

    /* The other threshold queues do not need to be configured */
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qnum   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qmgr   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qnum   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qmgr   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qnum   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qmgr   = 0x0;

    if (srio_usePolledMode)
    {
    	/* Polling Mode: So don't program the accumulator. */
    	appCfg.u.appManagedCfg.bIsAccumlatorCfgValid = 0;
    }
    else
    {
		/* Program the accumulator. */
		appCfg.u.appManagedCfg.bIsAccumlatorCfgValid = 1;
    }

    /* Accumulator Configuration. */
	appCfg.u.appManagedCfg.accCfg.channel             = CSL_chipReadReg (CSL_CHIP_DNUM);
	appCfg.u.appManagedCfg.accCfg.command             = Qmss_AccCmd_ENABLE_CHANNEL;
	appCfg.u.appManagedCfg.accCfg.queueEnMask         = 0;
	appCfg.u.appManagedCfg.accCfg.queMgrIndex         = coreToQueueSelector[CSL_chipReadReg(CSL_CHIP_DNUM)];
	appCfg.u.appManagedCfg.accCfg.maxPageEntries      = sizeof(gHiPriAccumList)/8;
	appCfg.u.appManagedCfg.accCfg.timerLoadCount      = 0;
	appCfg.u.appManagedCfg.accCfg.interruptPacingMode = Qmss_AccPacingMode_LAST_INTERRUPT;
	appCfg.u.appManagedCfg.accCfg.listEntrySize       = Qmss_AccEntrySize_REG_D;
	appCfg.u.appManagedCfg.accCfg.listCountMode       = Qmss_AccCountMode_ENTRY_COUNT;
	appCfg.u.appManagedCfg.accCfg.multiQueueMode      = Qmss_AccQueueMode_SINGLE_QUEUE;

    /* Initialize the accumulator list memory */
    memset ((void *)&gHiPriAccumList[0], 0, sizeof(gHiPriAccumList));
    appCfg.u.appManagedCfg.accCfg.listAddress = l2_global_address((uint32_t)&gHiPriAccumList[0]);

    /* Populate the rest of the configuration. */
    appCfg.u.appManagedCfg.rawRxFreeDrvBuffer = myAppRawRxFree;
    appCfg.u.appManagedCfg.txQueueNum = QMSS_PARAM_NOT_SPECIFIED;
 
    /* Set the PKTDMA TX channel priority to low */
    appCfg.u.appManagedCfg.srioPktDmaTxPrio = Srio_PktDma_Prio_Low;

    /* Start the application Managed SRIO Driver. */
    hAppManagedSrioDrv = Srio_start (&appCfg);
    if (hAppManagedSrioDrv == NULL)
    {
        System_printf ("Error: Application Managed SRIO Driver failed to start\n");
        return -1;
    }

    if (testControl.srio_isRunProgress)
    {
    	/* Debug Message: */
        System_printf ("Debug: AppConfig RxFreeQueue: 0x%x RxCompletionQueue: 0x%x\n",
                        myRxFreeQueueHnd, myRxCompletionQueueHnd);
        System_printf ("Debug: Accumulator List: 0x%x\n", appCfg.u.appManagedCfg.accCfg.listAddress);
    }

    /* Configuration was successful. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      System Initialization Code. This is added here only for illustrative
 *      purposes and needs to be invoked once during initialization at 
 *      system startup.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t system_init (void)
{
    int32_t             result;
    Qmss_InitCfg        qmssInitConfig;

    /* Initialize the QMSS Configuration block. */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up the linking RAM. Use the internal Linking RAM. 
     * LLD will configure the internal linking RAM address and maximum internal linking RAM size if 
     * a value of zero is specified. Linking RAM1 is not used */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_HOST_DESC*2;

#ifdef xdc_target__bigEndian
    /* PDSP Configuration: Big Endian */
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_be);
#else
    /* PDSP Configuration: Little Endian */
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_le);
#endif

    /* Initialize Queue Manager Sub System */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem error code : %d\n", result);
        return -1;
    }

    /* Start the QMSS. */
    if (Qmss_start() != QMSS_SOK)
    {
        System_printf ("Error: Unable to start the QMSS\n");
        return -1;
    }

    /* Initialize CPPI CPDMA */
    result = Cppi_init (&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem error code : %d\n", result);
        return -1;
    }

    if (testControl.srio_isRunProgress)
    {
    	/* CPPI and Queue Manager are initialized. */
    	System_printf ("Debug: Queue Manager and CPPI are initialized.\n");
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function initializes the memory pool which is used by the 
 *      Producer Application. 
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t initMemPool (Qmss_MemRegion memRegion)
{
    Cppi_DescCfg        descCfg;
    uint32_t            numAllocated;
    uint32_t            index;
    uint8_t*            ptrDataBuffer;
    Error_Block	        errorBlock;
    Qmss_QueueHnd       tempQueue;
    uint8_t             isAllocated;
    Cppi_HostDesc*      ptrHostDesc;
	uint32_t coreNum;

	/* Get the core number. */
	coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);


    /* Create a general purpose heap queue. */
    memoryHeapQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (memoryHeapQueue < 0)
        return -1;

    /* Populate the CPPI descriptor configuration */
    memset(&descCfg,0,sizeof(Cppi_DescCfg));
    descCfg.memRegion                 = memRegion;
	descCfg.descNum                   = ((coreNum == 0) ? NUM_TX_DESC_ON_RX_SIDE : NUM_TX_DESC);
    descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType                  = Cppi_DescType_HOST;
    descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.returnPushPolicy          = Qmss_Location_TAIL;
    descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
    descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

    /* Initialize the descriptors and place all of them into the Memory Heap Queue. */
    tempQueue  = Cppi_initDescriptor (&descCfg, &numAllocated);
    if (tempQueue < 0)
        return -1;

    /* Did we get all the descriptors we requested for? */
    if (numAllocated != descCfg.descNum)
        return -1;

    /* Now attach buffers to these descriptors. */
    for (index = 0; index < numAllocated; index++)
    {
        /* Allocate a data buffer from the System Heap. */
        ptrDataBuffer = Memory_alloc (NULL, MESSAGE_MAX_DATA_SIZE, 0, &errorBlock);
        if (ptrDataBuffer == NULL)
            return -1;

        /* Convert the address to a global address. */
        ptrDataBuffer = (uint8_t*)l2_global_address((uint32_t)ptrDataBuffer);

        /* Pop off the descriptor from the temporary queue. */
        ptrHostDesc = Qmss_queuePop (tempQueue);

        /* Set the data and payload length. */
	    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, ptrDataBuffer, MESSAGE_MAX_DATA_SIZE);
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, ptrDataBuffer, MESSAGE_MAX_DATA_SIZE);

        /* Initialize the return queue to be the same as the memory heap queue. */
        Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, Qmss_getQueueNumber(memoryHeapQueue));

        /* Debug Message: */
        debugPrintf_pp ("Debug: Tx Free Descriptor 0x%p with DataBuffer 0x%p\n", ptrHostDesc, ptrDataBuffer);

        /* Push the descriptor back into the Heap queue. */
        Qmss_queuePushDesc (memoryHeapQueue, (uint32_t *)ptrHostDesc);
    }

    /* Close the temporary queue. */
    Qmss_queueClose (tempQueue);

    if (testControl.srio_isRunProgress)
    {
    	/* Debug Message: */
    	System_printf ("Debug: Memory Heap Queue 0x%x\n", memoryHeapQueue);
    }
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      This is the consumer control task.
 *      (Make sure that the consumer and producer tests are in the same sequence)
 *
 *  @retval
 *      Not Applicable.
 */
static void consumerTests (void)
{
	/* DIO NWRITE throughput and latency tests */
	if (testControl.srio_testsToRun == srio_all_available_tests || testControl.srio_testsToRun == srio_nwrite_tests || testControl.srio_testsToRun == srio_nwrite_nread_tests)
	{
		if (consumerDIONWriteThroughput() <  0)
			return;
	}

	/* DIO NREAD throughput and latency tests */
	if (testControl.srio_testsToRun == srio_all_available_tests || testControl.srio_testsToRun == srio_nread_tests || testControl.srio_testsToRun == srio_nwrite_nread_tests)
	{
		if (consumerDIONReadThroughput() <  0)
			return;
	}

	/* Message Type-11 throughput and latency tests */
	if (testControl.srio_testsToRun == srio_all_available_tests || testControl.srio_testsToRun == srio_type11_tests)
	{
		if (consumerType11Throughput() < 0)
			return;
	}
}

/**
 *  @b Description
 *  @n
 *      This is the producer control task.
 *      (Make sure that the producer and consumer tests are in the same sequence)
 *
 *  @retval
 *      Not Applicable.
 */
static void producerTests (void)
{
	/* DIO NWRITE throughput and latency tests */
	if (testControl.srio_testsToRun == srio_all_available_tests || testControl.srio_testsToRun == srio_nwrite_tests || testControl.srio_testsToRun == srio_nwrite_nread_tests)
	{
		if (producerDIONWriteThroughput() <  0)
			return;
	}

	/* DIO NREAD throughput and latency tests */
	if (testControl.srio_testsToRun == srio_all_available_tests || testControl.srio_testsToRun == srio_nread_tests || testControl.srio_testsToRun == srio_nwrite_nread_tests)
	{
		if (producerDIONReadThroughput() <  0)
			return;
	}

	/* Message Type-11 throughput and latency tests */
	if (testControl.srio_testsToRun == srio_all_available_tests || testControl.srio_testsToRun == srio_type11_tests)
	{
		if (producerType11Throughput() < 0)
			return;
	}
}

/**
 *  @b Description
 *  @n
 *      Utility function that is required by the IPC module to set the proc Id.
 *      The proc Id is set via this function instead of hard coding it in the .cfg file
 *
 *  @retval
 *      Not Applicable.
 */
Void myStartupFxn (Void)
{
	MultiProc_setLocalId (CSL_chipReadReg (CSL_CHIP_DNUM));
}

/**
 *  @b Description
 *  @n
 *      Entry point for the test code.
 *
 *  @retval
 *      Not Applicable.
 */
void main (int argc, char* argv[])
{
    Qmss_MemRegInfo     memRegInfo;
    int32_t             result;
    int32_t             eventId;
	uint32_t			coreNum;
	
	setBiosCpuFrequencyIndicatorIfNeeded();

	/* Get the core number. */
	coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

#ifdef SIMULATOR_SUPPORT
	System_printf ("\n Error on core %lu: This example code does not run on the simulator.\n\n", coreNum);
	return;
#endif

	/* If test parameters have been passed to this application then set the appropriate
	 * test control variables, otherwise use the default settings of the control variables. */
	if (setStartParameters(argc, argv, coreNum) < 0 )
		return;

    /* Enable the TSC */
	CSL_tscEnable ();

    if (!testControl.srio_isBoardToBoard)
    {
		/* Initialize the heap in shared memory. Using IPC module to do that */
		Ipc_start ();
    }

    /* Initialize the Time stamp. */
    TSCH = 0;
    TSCL = 0;

    /* Set SRIO ID for 16 or 8 bit operation */
	if (testControl.srio_isDeviceID16Bit)
    {
    	srio_device_ID1 = DEVICE_ID1_16BIT;
    	srio_device_ID2 = DEVICE_ID2_16BIT;
    }
    else
    {
    	srio_device_ID1 = DEVICE_ID1_8BIT;
    	srio_device_ID2 = DEVICE_ID2_8BIT;
    }

	/* Turn off internal loop-back mode if going board to board */
	testControl.srio_isLoopbackMode = ((testControl.srio_isBoardToBoard) ? FALSE : testControl.srio_isLoopbackMode);

	/* Turn off internal loop-back mode if using an external SRIO switch */
	testControl.srio_isLoopbackMode = ((testControl.srio_isExternalSrioSwitch) ? FALSE : testControl.srio_isLoopbackMode);

    /* Initializations depend on the core type. */
	if (coreNum == testControl.srio_initCorenum)
	{
		/* Debug Message: */
        System_printf ("********************************\n");
		System_printf ("*********** %s ***********\n", ((coreNum == CONSUMER_CORE) ? "CONSUMER" : "PRODUCER"));
        System_printf ("********************************\n");
        System_printf ("WARNING: Please ensure that the CONSUMER is executing before running the PRODUCER!!\n");

        /* Initialize the CPPI and QMSS Modules. */
		if (system_init () < 0)
			return;
	
		/* Power on SRIO peripheral before using it */
		if (enable_srio () < 0)
		{
			System_printf ("Error: SRIO PSC Initialization Failed\n");
			return;
		}

        /* Initialize the Host Region. */
        memset ((void *)&host_region_consumer, 0, sizeof(host_region_consumer));

		/* Memory Region Configuration */
		memRegInfo.descBase         = (uint32_t *)l2_global_address((uint32_t)host_region_consumer);
		memRegInfo.descSize         = SIZE_HOST_DESC;
		memRegInfo.descNum          = NUM_HOST_DESC;
		memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
		memRegInfo.memRegion        = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
	
		/* Initialize and inset the memory region. */
		result = Qmss_insertMemoryRegion (&memRegInfo); 
		if (result < QMSS_SOK)
		{
			System_printf ("Error inserting memory region: %d\n", result);
			return;
		}
	
		/* Device Specific SRIO Initializations: This should always be called before
		 * initializing the SRIO Driver. */
		if (SrioDevice_init () < 0)
			return;
	
		/* Initialize the SRIO Driver */
		if (Srio_init () < 0)
			return;

        /* Write to the SHARED memory location at this point in time. The other cores cannot execute
         * till the SRIO Driver is up and running. */
        isSRIOInitialized[0] = 1;

        /* The SRIO IP block has been initialized. We need to writeback the cache here because it will
         * ensure that the rest of the cores which are waiting for SRIO to be initialized would now be
         * woken up. */
        CACHE_wbL1d ((void *) &isSRIOInitialized[0], 128, CACHE_FENCE_WAIT);

        /* Create Internal Heap */
        if (initMemPool((Qmss_MemRegion)result) < 0)
            return;

    	if (coreNum == CONSUMER_CORE)
    	{
			/* Initialize the DIO Consumer Memory Block. */
			{
				uint8_t*            ptrMemory;
				uint16_t            index;

				/* The DIO Consumer Memory Block right @ the end of the L2 Memory. */
				ptrMemory = (uint8_t*)l2_global_address((uint32_t)(0x880000 - DIO_MAX_DATA_SIZE));

				/* Initialize the DIO Consumer Memory Block. */
				for (index = 0; index < DIO_MAX_DATA_SIZE; index++)
					ptrMemory[index] = 0xBB;
			}
    	}

	    /* Clear Errors*/
	    clearSrioStatusErrors ();
	}
	else
	{
        /* Debug Message: */
        System_printf ("********************************\n");
        System_printf ("*********** PRODUCER ***********\n");
        System_printf ("********************************\n");
        System_printf ("WARNING: Please ensure that the CONSUMER is executing before running the PRODUCER!!\n");

        /* All other cores need to wait for the SRIO to be initialized before they proceed with the test. */ 
        System_printf ("Debug(Core %d): Waiting for SRIO to be initialized.\n", coreNum);

        /* All other cores loop around forever till the SRIO is up and running. 
         * We need to invalidate the cache so that we always read this from the memory. */
        while (isSRIOInitialized[0] == 0)
            CACHE_invL1d ((void *) &isSRIOInitialized[0], 128, CACHE_FENCE_WAIT);

        /* Start the QMSS. */
        if (Qmss_start() != QMSS_SOK)
        {
            System_printf ("Error: Unable to start the QMSS\n");
            return;
        }

        /* Initialize the Host Region. */
        memset ((void *)&host_region_consumer, 0, sizeof(host_region_consumer));

		/* Memory Region Configuration */
		memRegInfo.descBase         = (uint32_t *)l2_global_address((uint32_t)host_region_consumer);
		memRegInfo.descSize         = SIZE_HOST_DESC;
		memRegInfo.descNum          = NUM_HOST_DESC;
		memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
		memRegInfo.memRegion        = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
	
		/* Initialize and inset the memory region. */
		result = Qmss_insertMemoryRegion (&memRegInfo); 
		if (result < QMSS_SOK)
		{
			System_printf ("Error inserting memory region: %d\n", result);
			return;
		}        

        /* Create Internal Heap */
        if (initMemPool((Qmss_MemRegion)result) < 0)
            return;
	}

    /* Create the Application Managed Configuration. */
    if (setupSRIOAppConfig((Qmss_MemRegion)result) == -1)
        return;

    /* Hook up the interrupts if using interrupt mode. */
    if (!srio_usePolledMode)
    {
    	/* Hook up the SRIO interrupts with the core. */
		EventCombiner_dispatchPlug (48, (EventCombiner_FuncPtr)Srio_rxCompletionIsr, (UArg)hAppManagedSrioDrv, TRUE);
		EventCombiner_enableEvent (48);

		/* SRIO DIO: Interrupts need to be routed from the CPINTC0 to GEM Event.
		 *  - We have configured DIO Interrupts to get routed to Interrupt Destination 0
		 *    (Refer to the CSL_SRIO_SetDoorbellRoute API configuration in the SRIO Initialization)
		 *  - We want this to mapped to Host Interrupt 8
		 *
		 *  Map the System Interrupt i.e. the Interrupt Destination 0 interrupt to the DIO ISR Handler. */
		CpIntc_dispatchPlug(CSL_CIC0_SRIO_INTDST0, myDIOIsr, (UArg)hAppManagedSrioDrv, TRUE);

		/* SRIO DIO: Configuration is for CPINTC0. We map system interrupt 112 to Host Interrupt 8. */
		CpIntc_mapSysIntToHostInt(0, CSL_CIC0_SRIO_INTDST0, 8);

		/* SRIO DIO: Enable the Host Interrupt. */
		CpIntc_enableHostInt(0, 8);

		/* SRIO DIO: Get the event id associated with the host interrupt. */
		eventId = CpIntc_getEventId(8);

		/* SRIO DIO: Plug the CPINTC Dispatcher. */
		EventCombiner_dispatchPlug (eventId, CpIntc_dispatch, 8, TRUE);

		/* Debug Message */
		System_printf ("Debug: Interrupts Registration complete\n");
    }
    else
    {
        /* Polled Mode */
        System_printf ("Debug: Running test in polled mode.\n");
    }

    /* Debug Message: */
    System_printf ("Debug: SRIO Driver handle 0x%x.\n\n\n", hAppManagedSrioDrv);

    /* Create the DIO Management Socket. */
    mgmtSocket = createMgmtSocket ();

    /* Start the appropriate tasks. */
    if (coreNum == CONSUMER_CORE)
    {
        /* Start Consumer side tests */
        consumerTests();
    }
    else
    {
        /* Start Producer side tests */
        producerTests();
    }
}

