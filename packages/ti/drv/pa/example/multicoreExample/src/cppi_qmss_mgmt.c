/**
 * @file cppi_qmss_mgmt.c
 *
 * @brief
 *  This file holds all the APIs required to configure CPPI/QMSS LLDs and
 *  to send/receive data using PA/QM.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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
#include <multicore_example.h>

#ifdef __LINUX_USER_SPACE
#include "armv7/linux/fw_test.h"
#include "armv7/linux/fw_mem_allocator.h"

extern uint8_t                                *gPaCmdBuf1;
extern uint8_t                                *gPaCmdBuf2;

#if defined(SOC_K2H)
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (SOC_K2K)
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (SOC_K2L)
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#elif defined (SOC_K2E)
#include <ti/drv/qmss/device/k2e/src/qmss_device.c>
#include <ti/drv/cppi/device/k2e/src/cppi_device.c>
#else /*Default */
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#endif /* Device */
#else
#include <ti/drv/qmss/qmss_firmware.h>
#endif /*  __LINUX_USER_SPACE */

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;
/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

#ifdef __LINUX_USER_SPACE
#if ( !defined( _LITTLE_ENDIAN ) && !defined( _BIG_ENDIAN ) ) \
||  ( defined(_LITTLE_ENDIAN ) && defined( _BIG_ENDIAN ) )
#error either _LITTLE_ENDIAN or _BIG_ENDIAN must be defined
#endif
#endif

/* Number of Tx Free descriptors to allocate */
#define     NUM_TX_DESC                 NUM_HOST_DESC/2

/* Number of Rx Free descriptors to allocate */
#define     NUM_RX_DESC                 NUM_HOST_DESC/2

/* Buffer sizes configured for
 * -  maximum command size to PA
 * -  Maximum size of the control messages
 *    from DSP
 * - Maximum size of the packets being transmitted
 */
#define TX_BUF_SIZE 		(((300+15)/16)*16)
#define RX_BUF_SIZE 		(((2400+15)/16)*16)


/* Host Descriptor Region - [Size of descriptor * Number of descriptors]
 *
 * MUST be 16 byte aligned.
 */
#ifdef __LINUX_USER_SPACE
uint8_t                                *gHostDesc = 0;
uint8_t                                *cppiMemTX[NUM_TX_DESC];
uint8_t                                *cppiMemRX[NUM_RX_DESC];
#else
#ifdef _TMS320C6X
#pragma DATA_ALIGN (gHostDesc, 128)
#pragma DATA_SECTION(gHostDesc, ".sharedDDR")
uint8_t gHostDesc[SIZE_HOST_DESC * NUM_HOST_DESC];
/* Buffers to be used for TX */
#pragma DATA_SECTION (cppiMemTX, ".cppiMemTX");
#pragma DATA_ALIGN(cppiMemTX, 16)
uint8_t cppiMemTX[NUM_TX_DESC][TX_BUF_SIZE];

/* Buffers to be used for RX */
#pragma DATA_SECTION (cppiMemRX, ".cppiMemRX");
#pragma DATA_ALIGN(cppiMemRX, 16)
uint8_t cppiMemRX[NUM_RX_DESC][RX_BUF_SIZE];
#else
uint8_t gHostDesc[SIZE_HOST_DESC * NUM_HOST_DESC] __attribute__ ((aligned (128)));
/* Buffers to be used for TX */
uint8_t cppiMemTX[NUM_TX_DESC][TX_BUF_SIZE] __attribute__ ((aligned (16)));
/* Buffers to be used for RX */
uint8_t cppiMemRX[NUM_RX_DESC][RX_BUF_SIZE] __attribute__ ((aligned (16)));
#endif
#endif

#ifndef __LINUX_USER_SPACE
/**
 *  @b Description
 *  @n
 *      The function is used to get the handle to the CPPI memory heap.
 *      If the application is run on a multiple cores then place the CPPI global
 *      variables in shared memory.
 *
 *  @retval
 *      Not Applicable
 */
static void cppiHeapInit ()
{
    cppiHeap = HeapMem_Handle_upCast (cppiSharedHeap);
}
#endif

extern int32_t Clear_PASS (int portNum);

/** ============================================================================
 *   @n@b Init_Qmss
 *
 *   @b Description
 *   @n This API initializes the QMSS LLD on core 0 only.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Init_Qmss (void)
{
    int32_t                       result;
    Qmss_MemRegInfo             memCfg;
    Qmss_InitCfg                qmssInitConfig;
    Cppi_DescCfg                cppiDescCfg;
    uint32_t                      numAllocated;
    Qmss_GlobalConfigParams     *fw_qmssGblCfgParams;

    /* Initialize QMSS */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up QMSS configuration */
    /* Use internal linking RAM */
    qmssInitConfig.linkingRAM0Base  =   0;
    qmssInitConfig.linkingRAM0Size  =   0;
    qmssInitConfig.linkingRAM1Base  =   0x0;
    qmssInitConfig.maxDescNum       =   NUM_HOST_DESC;

    /* Provide the firmware for DSP use case */
#ifdef __LINUX_USER_SPACE
    /* Bypass hardware initialization as it is done within Kernel */
    qmssInitConfig.qmssHwStatus     =   QMSS_HW_INIT_COMPLETE;
#else
    if (no_bootMode == TRUE)
    {
       qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
#ifdef _LITTLE_ENDIAN
       qmssInitConfig.pdspFirmware[0].firmware = (void *) &acc48_le;
       qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_le);
#else
       qmssInitConfig.pdspFirmware[0].firmware = (void *) &acc48_be;
       qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_be);
#endif
    }
    else
    {
        /* Bypass hardware initialization as it is done within Kernel */
        qmssInitConfig.qmssHwStatus     =   QMSS_HW_INIT_COMPLETE;
    }
#endif

#if RM
    qmssGblCfgParams.qmRmServiceHandle = rmClientServiceHandle;
#endif

#ifndef __LINUX_USER_SPACE
    /* Initialize the heap in shared memory for CPPI data structures */
    cppiHeapInit ();
#endif

    /* Initialize the Queue Manager */
    fw_qmssGblCfgParams = &qmssGblCfgParams;
    get_qmssGblCfgParamsRegsPhy2Virt(fw_qmssGblCfgParams);
    result = Qmss_init (&qmssInitConfig, fw_qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem, Error code : %d\n", result);
        return -1;
    }

    /* Start Queue manager on this core */
    Qmss_start ();

    /* Setup the descriptor memory regions.
     *
     * The Descriptor base addresses MUST be global addresses and
     * all memory regions MUST be setup in ascending order of the
     * descriptor base addresses.
     */
#ifdef __LINUX_USER_SPACE
    gHostDesc = (uint8_t*)fw_memAlloc((NUM_HOST_DESC *
                                       SIZE_HOST_DESC),
                                       CACHE_LINESZ);
    {
       uint32_t i;

	   for (i = 0; i < NUM_TX_DESC; i++) {
	   	   cppiMemTX[i] = (uint8_t*) fw_memAlloc(TX_BUF_SIZE, CACHE_LINESZ);
	   }
	   for (i = 0; i < NUM_RX_DESC; i++) {
	   	   cppiMemRX[i] = (uint8_t*) fw_memAlloc(RX_BUF_SIZE, CACHE_LINESZ);
	   }
    }

#endif
    /* Initialize and setup CPSW Host Descriptors required for example */
    memset (gHostDesc, 0, SIZE_HOST_DESC * NUM_HOST_DESC);
    memCfg.descBase             =   (uint32_t *) gHostDesc;
    memCfg.descSize             =   SIZE_HOST_DESC;
    memCfg.descNum              =   NUM_HOST_DESC;
    memCfg.manageDescFlag       =   Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memCfg.memRegion            =   Qmss_MemRegion_MEMORY_REGION0;
    memCfg.startIndex           =   0;

    /* Insert Host Descriptor memory region */
    result = Qmss_insertMemoryRegion(&memCfg);
    if (result == QMSS_MEMREGION_ALREADY_INITIALIZED)
    {
        System_printf ("Memory Region %d already Initialized \n", memCfg.memRegion);
    }
    else if (result < QMSS_SOK)
    {
        System_printf ("Error: Inserting memory region %d, Error code : %d\n", memCfg.memRegion, result);
        return -1;
    }

    /* Initialize all the descriptors we just allocated on the
     * memory region above. Setup the descriptors with some well
     * known values before we use them for data transfers.
     */
    memset (&cppiDescCfg, 0, sizeof (cppiDescCfg));
    cppiDescCfg.queueGroup      =   0;
    cppiDescCfg.memRegion       =   Qmss_MemRegion_MEMORY_REGION0;
    cppiDescCfg.descNum         =   NUM_HOST_DESC;
    cppiDescCfg.destQueueNum    =   QMSS_PARAM_NOT_SPECIFIED;
    cppiDescCfg.queueType       =   Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    cppiDescCfg.initDesc        =   Cppi_InitDesc_INIT_DESCRIPTOR;
    cppiDescCfg.descType        =   Cppi_DescType_HOST;

    /* By default:
     *      (1) Return descriptors to tail of queue
     *      (2) Always return entire packet to this free queue
     *      (3) Set that PS Data is always present in start of SOP buffer
     *      (4) Configure free q num < 4K, hence qMgr = 0
     *      (5) Recycle back to the same Free queue by default.
     */
    cppiDescCfg.returnPushPolicy            =   Qmss_Location_TAIL;
    cppiDescCfg.cfg.host.returnPolicy       =   Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
    cppiDescCfg.cfg.host.psLocation         =   Cppi_PSLoc_PS_IN_DESC;
    cppiDescCfg.returnQueue.qMgr            =   QMSS_PARAM_NOT_SPECIFIED;
    cppiDescCfg.returnQueue.qNum            =   QMSS_PARAM_NOT_SPECIFIED;
    cppiDescCfg.epibPresent                 =   Cppi_EPIB_EPIB_PRESENT;

    /* Initialize the descriptors, create a free queue and push descriptors to a global free queue */
    if ((gGlobalFreeQHnd = Cppi_initDescriptor (&cppiDescCfg, &numAllocated)) <= 0)
    {
        System_printf ("Error Initializing Free Descriptors, Error: %d \n", gGlobalFreeQHnd);
        return -1;
    }
    else
    {
        System_printf ("Initializing Free Descriptors. \n");
    }

    /* Queue Manager Initialization Done */
    return 0;
}


/** ============================================================================
 *   @n@b Init_Qmss_Local
 *
 *   @b Description
 *   @n This API initializes the QMSS LLD in cores other than core 0.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Init_Qmss_Local (void)
{
#ifndef __LINUX_USER_SPACE
  int32_t            result;
#if RM
      Qmss_StartCfg    qmssStartCfg;

	  /* Start Queue Manager SubSystem with RM */
	  memset (&qmssStartCfg, 0, sizeof (Qmss_StartCfg));
	  if (rmClientServiceHandle) {
	  	qmssStartCfg.rmServiceHandle = rmClientServiceHandle;
		result = Qmss_startCfg (&qmssStartCfg);
	  }
#endif /* RM */

  while(1)
  {
      /* Block until Qmss_init() has completed by core 0 */
      result = Qmss_start();
      if(result == QMSS_NOT_INITIALIZED)
      {
          System_printf ("QMSS Not yet Initialized\n");
          continue;
      }
      else if (result != QMSS_SOK)  {
        System_printf ("Qmss_start failed with error code %d\n", result);
        return (-1);
      }

      if (result == QMSS_SOK)
      {
          break;
      }
  }
#else
    int32_t                       result;
    Qmss_InitCfg                qmssInitConfig;
    Qmss_GlobalConfigParams     *fw_qmssGblCfgParams;

    /* Initialize QMSS */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up QMSS configuration */
    /* Use internal linking RAM */
    qmssInitConfig.linkingRAM0Base  =   0;
    qmssInitConfig.linkingRAM0Size  =   0;
    qmssInitConfig.linkingRAM1Base  =   0x0;
    qmssInitConfig.maxDescNum       =   NUM_HOST_DESC;

    /* Provide the firmware for DSP use case */
    /* Bypass hardware initialization as it is done within Kernel */
    qmssInitConfig.qmssHwStatus     =   QMSS_HW_INIT_COMPLETE;

#if RM
    qmssGblCfgParams.qmRmServiceHandle = rmClientServiceHandle;
#endif
    /* Initialize the Queue Manager */
    fw_qmssGblCfgParams = &qmssGblCfgParams;
    get_qmssGblCfgParamsRegsPhy2Virt(fw_qmssGblCfgParams);
    result = Qmss_init (&qmssInitConfig, fw_qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem, Error code : %d\n", result);
        return -1;
    }

    /* Start Queue manager on this core */
    Qmss_start ();
#endif
  return 0;
}

/** ============================================================================
 *   @n@b Init_Cppi_Local
 *
 *   @b Description
 *   @n This API initializes the CPPI LLD, opens the PASS CPDMA and opens up
 *      the Tx, Rx channels required for data transfers.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Init_Cppi_Local (void)
{
#ifndef __LINUX_USER_SPACE
     Cppi_StartCfg			cppiStartCfg;
     if (rmClientServiceHandle) {
		memset (&cppiStartCfg, 0, sizeof (Cppi_StartCfg));
		/* Register RM with CPPI */
		cppiStartCfg.rmServiceHandle = rmClientServiceHandle;
		Cppi_startCfg(&cppiStartCfg);
     }
 	gCpdmaHnd    = (Cppi_Handle)   fw_shmGetEntry(gCpdmaHndAddr);
#else
    int32_t 					  result;
    Cppi_CpDmaInitCfg			cpdmaCfg;
    Cppi_StartCfg				cppiStartCfg;
    Cppi_GlobalConfigParams 		fw_cppiGblCfgParams;

    /* Initialize CPPI LLD */
    fw_cppiGblCfgParams = cppiGblCfgParams;
    /* Initialize CPPI LLD */
    get_cppiGblCfgParamsRegsPhy2Virt(&fw_cppiGblCfgParams);
    result = Cppi_init (&fw_cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
    	System_printf ("Error initializing CPPI LLD, Error code : %d\n", result);
    	return -1;
    }

    if (rmClientServiceHandle) {
       /* Register RM with CPPI */
       cppiStartCfg.rmServiceHandle = rmClientServiceHandle;
       Cppi_startCfg(&cppiStartCfg);
    }

    /* Initialize PASS CPDMA */
    memset (&cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
    cpdmaCfg.dmaNum     = Cppi_CpDma_PASS_CPDMA;
    if ((gCpdmaHnd = Cppi_open (&cpdmaCfg)) == NULL)
    {
        System_printf ("Error initializing CPPI for PASS CPDMA %d \n", cpdmaCfg.dmaNum);
        return -1;
    }
#endif

   return 0;

}

/** ============================================================================
 *   @n@b Init_Cppi
 *
 *   @b Description
 *   @n This API initializes the CPPI LLD, opens the PASS CPDMA and opens up
 *      the Tx, Rx channels required for data transfers.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Init_Cppi (void)
{
    int32_t                       result, i;
    Cppi_CpDmaInitCfg           cpdmaCfg;
    uint8_t                       isAllocated;
    Cppi_TxChInitCfg            txChCfg;
    Cppi_RxChInitCfg            rxChInitCfg;
    Cppi_StartCfg               cppiStartCfg;
    Cppi_GlobalConfigParams         fw_cppiGblCfgParams;
    uint32_t cppi_pa_tx_ch_disable_list[NUM_PA_TX_CHANNELS];
    uint32_t cppi_pa_rx_ch_disable_list[NUM_PA_RX_CHANNELS];
    uint32_t disable_list_flag = 0;

    /* Clear the list by default */
    memset (cppi_pa_tx_ch_disable_list, 0, sizeof (cppi_pa_tx_ch_disable_list));
    memset (cppi_pa_rx_ch_disable_list, 0, sizeof (cppi_pa_rx_ch_disable_list));

    /* Initialize CPPI LLD */
    fw_cppiGblCfgParams = cppiGblCfgParams;
    /* Initialize CPPI LLD */
    get_cppiGblCfgParamsRegsPhy2Virt(&fw_cppiGblCfgParams);
    result = Cppi_init (&fw_cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        System_printf ("Error initializing CPPI LLD, Error code : %d\n", result);
        return -1;
    }

    if (rmClientServiceHandle) {
       /* Register RM with CPPI */
       cppiStartCfg.rmServiceHandle = rmClientServiceHandle;
       Cppi_startCfg(&cppiStartCfg);
    }


    /* Initialize PASS CPDMA */
    memset (&cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
    cpdmaCfg.dmaNum     = Cppi_CpDma_PASS_CPDMA;
    if ((gCpdmaHnd = Cppi_open (&cpdmaCfg)) == NULL)
    {
        System_printf ("Error initializing CPPI for PASS CPDMA %d \n", cpdmaCfg.dmaNum);
        return -1;
    }

    /* Open all CPPI Tx Channels. These will be used to send data to PASS/CPSW */
    for (i = 0, disable_list_flag = 0; i < NUM_PA_TX_CHANNELS; i ++)
    {
        txChCfg.channelNum      =   i;       /* CPPI channels are mapped one-one to the PA Tx queues */
        txChCfg.txEnable        =   Cppi_ChState_CHANNEL_DISABLE;  /* Disable the channel for now. */
        txChCfg.filterEPIB      =   0;
        txChCfg.filterPS        =   0;
        txChCfg.aifMonoMode     =   0;
        txChCfg.priority        =   2;
        if ((gCpdmaTxChanHnd[i] = Cppi_txChannelOpen (gCpdmaHnd, &txChCfg, &isAllocated)) == NULL)
        {
            if (no_bootMode == TRUE)
            {
        	   System_printf ("Error opening Tx channel %d\n", txChCfg.channelNum);
               return -1;
            }
            else
            {
            	cppi_pa_tx_ch_disable_list[i] = 1;
            	disable_list_flag = 1;
            	continue;
            }
        }

        Cppi_channelEnable (gCpdmaTxChanHnd[i]);
    }

    /* Print if there are any CPPI Tx channels that are not enabled by above code, presuming linux enabled it */
    if (disable_list_flag)
    {
    	System_printf ("Unable to open below cppi tx channels...presuming linux has already enabled it \n");
		for (i = 0; i< NUM_PA_TX_CHANNELS; i++)
		{
			 if (cppi_pa_tx_ch_disable_list[i])
				 System_printf ("%d ", i);
		}
		System_printf (" \n ");
    }

    /* Open all CPPI Rx channels. These will be used by PA to stream data out. */
    for (i = 0, disable_list_flag = 0; i < NUM_PA_RX_CHANNELS; i++)
    {
        /* Open a CPPI Rx channel that will be used by PA to stream data out. */
        rxChInitCfg.channelNum  =   i;
        rxChInitCfg.rxEnable    =   Cppi_ChState_CHANNEL_DISABLE;
        if ((gCpdmaRxChanHnd[i] = Cppi_rxChannelOpen (gCpdmaHnd, &rxChInitCfg, &isAllocated)) == NULL)
        {
            if (no_bootMode == TRUE)
            {
            	System_printf ("Error opening Rx channel: %d \n", rxChInitCfg.channelNum);
                return -1;
            }
            else
            {
            	cppi_pa_rx_ch_disable_list[i] = 1;
            	disable_list_flag = 1;
            	continue;
            }
        }

        /* Also enable Rx Channel */
        Cppi_channelEnable (gCpdmaRxChanHnd[i]);
    }

    /* Print if there are any CPPI Rx channels that are not enabled by above code, presuming linux enabled it */
    if (disable_list_flag)
    {
    	System_printf ("Unable to open below cppi Rx channels...presuming linux has already enabled it \n");
		for (i = 0; i<NUM_PA_RX_CHANNELS; i++)
		{
			 if (cppi_pa_rx_ch_disable_list[i])
				 System_printf ("%d ", i);
		}
		System_printf (" \n ");
    }

    /* Clear CPPI Loobpack bit in PASS CDMA Global Emulation Control Register */
    Cppi_setCpdmaLoopback(gCpdmaHnd, 0);

    /* CPPI Init Done. Return success */
    return 0;
}

/** ============================================================================
 *   @n@b Setup_Tx_local
 *
 *   @b Description
 *   @n This API sets up all relevant data structures and configuration required
 *      for sending data to PASS/Ethernet. It sets up a Tx free descriptor queue,
 *      PASS Tx queues required for send.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
void Setup_Tx_local(void)
{
	uint32_t i, pa_tx_queues;
	uint8_t isAllocated;
	Qmss_QueueHnd				txFreeQHnd;
	Qmss_Queue					txFreeQInfo;
	/*Now open the Tx Free Queue Handles for the slave process */
	txFreeQHnd	= (Qmss_QueueHnd) fw_shmGetEntry(gTxFreeQHndAddr);

	txFreeQInfo.qNum = Qmss_getQIDFromHandle (txFreeQHnd);
	for (i = 0, pa_tx_queues = QMSS_PASS_QUEUE_BASE; i < NUM_PA_TX_QUEUES; i++, pa_tx_queues++)
	{
		if ((gPaTxQHnd[i] = Qmss_queueOpen (Qmss_QueueType_PASS_QUEUE, pa_tx_queues, &isAllocated)) < 0)
		{
			System_printf ("Error opening PA Tx queue (slaves), err:%d \n", gPaTxQHnd[i]);
			APP_exit(-1);
		}
	}
	/* Open a Tx Free Descriptor Queue (Tx FDQ).
	 *
	 * This queue will be used to hold Tx free decriptors that can be filled
	 * later with data buffers for transmission onto wire.
	 */
	if ((gTxFreeQHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, txFreeQInfo.qNum, &isAllocated)) < 0)
	{
		System_printf ("Error opening Tx Free descriptor queue(slaves), err:%d \n", gTxFreeQHnd);
		APP_exit (-1);
	}

	if (gTxFreeQHnd != txFreeQHnd) {
		System_printf ("Error Obtaining Tx Free descriptor handle \n");
		APP_exit(-1);
	}


}

/** ============================================================================
 *   @n@b Setup_Tx
 *
 *   @b Description
 *   @n This API sets up all relevant data structures and configuration required
 *      for sending data to PASS/Ethernet. It sets up a Tx free descriptor queue,
 *      PASS Tx queues required for send.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Setup_Tx (void)
{
    uint8_t                       isAllocated, i;
    Qmss_Queue                  qInfo;
    Ptr                   		pCppiDesc;

    /* Open all Transmit (Tx) queues.
     *
     * These queues are used to send data to PA PDSP/CPSW.
     */
    for (i = 0; i < NUM_PA_TX_QUEUES; i ++)
    {

        if ((gPaTxQHnd[i] = Qmss_queueOpen (Qmss_QueueType_PASS_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
        {
            System_printf ("Error opening PA Tx queue (master) err: %d\n", gPaTxQHnd[i]);
            return -1;
        }
    }

    /* Open a Tx Free Descriptor Queue (Tx FDQ).
     *
     * This queue will be used to hold Tx free decriptors that can be filled
     * later with data buffers for transmission onto wire.
     */
    if ((gTxFreeQHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error opening Tx Free descriptor queue (master), err: %d \n", gTxFreeQHnd);
        return -1;
    }

    SYS_CACHE_WB ((void *)&gTxFreeQHnd, 128, CACHE_WAIT);


    qInfo = Qmss_getQueueNumber (gTxFreeQHnd);

    /* Attach some free descriptors to the Tx free queue we just opened. */
    for (i = 0; i < NUM_TX_DESC; i++)
    {
        /* Get a free descriptor from the global free queue we setup
         * during initialization.
         */
        if ((pCppiDesc = Qmss_queuePop (gGlobalFreeQHnd)) == NULL)
        {
            break;
        }

        /* The descriptor address returned from the hardware has the
         * descriptor size appended to the address in the last 4 bits.
         *
         * To get the true descriptor size, always mask off the last
         * 4 bits of the address.
         */
        pCppiDesc = (Ptr) ((uint32_t) pCppiDesc & 0xFFFFFFF0);

        /* Populate the Tx free descriptor with the buffer. */
        Cppi_setData (Cppi_DescType_HOST, pCppiDesc, (uint8_t *)(cppiMemTX[i]), TX_BUF_SIZE);

        /* Save original buffer information */
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, pCppiDesc, (uint8_t *)(cppiMemTX[i]), TX_BUF_SIZE);

        /* Setup the Completion queue:
         *
         * Setup the return policy for this desc to return to the free q we just
         * setup instead of the global free queue.
         */
        Cppi_setReturnQueue ((Cppi_DescType) Cppi_DescType_HOST, pCppiDesc, qInfo);

        Cppi_setPacketLen    (Cppi_DescType_HOST, pCppiDesc, TX_BUF_SIZE);

        SYS_CACHE_WB (pCppiDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);

        /* Push descriptor to Tx free queue */
        Qmss_queuePushDescSize (gTxFreeQHnd, pCppiDesc, SIZE_HOST_DESC);
    }
    if (i != NUM_TX_DESC)
    {
        System_printf ("Error allocating Tx free descriptors \n");
        return -1;
    }

    /* All done with Rx configuration. Return success. */
    return 0;
}

/** ============================================================================
 *   @n@b Setup_Rx
 *
 *   @b Description
 *   @n This API sets up all relevant data structures and configuration required
 *      for receiving data from PASS/Ethernet. It sets up a Rx free descriptor queue
 *      with some empty pre-allocated buffers to receive data, and an Rx queue
 *      to which the Rxed data is streamed for the example application.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Setup_Rx (void)
{
    uint8_t                       isAllocated, i;
    Qmss_Queue                  rxFreeQInfo, rxQInfo;
    Ptr                   		pCppiDesc;
    Cppi_RxFlowCfg              rxFlowCfg;
    Ptr                         pDataBuffer;
    uint32_t                      mySWInfo[] = {0x11112222, 0x33334444};

    /* Get the core/process Id number. */
#ifdef __LINUX_USER_SPACE
#else
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);
#endif
    /* Open a Receive (Rx) queue.
     *
     * This queue will be used to hold all the packets received by PASS/CPSW
     *
     */
    if ((gRxQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error opening gRxQHnd queue \n");
        return -1;
    }
    rxQInfo = Qmss_getQueueNumber (gRxQHnd);

    /* The following RX queues are shared between cores, so their
       initialization is done by core zero only*/
    if(coreNum == SYSINIT)
    {
        /* Open a Rx Free Descriptor Queue (Rx FDQ).
         *
         * This queue will hold all the Rx free decriptors. These descriptors will be
         * used by the PASS CPDMA to hold data received via CPSW.
         */
        if ((gRxFreeQHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
        {
            System_printf ("Error opening Rx Free descriptor queue \n");
            return -1;
        }

        SYS_CACHE_WB ((void *)&gRxFreeQHnd, 128, CACHE_WAIT);

        rxFreeQInfo = Qmss_getQueueNumber (gRxFreeQHnd);

        /* Attach some free descriptors to the Rx free queue we just opened. */
        for (i = 0; i < NUM_RX_DESC; i++)
        {
            /* Get a free descriptor from the global free queue we setup
             * during initialization.
             */
            if ((pCppiDesc = Qmss_queuePop (gGlobalFreeQHnd)) == NULL)
            {
                System_printf ("Error poping descriptor.\n");
                break;
            }

            /* The descriptor address returned from the hardware has the
             * descriptor size appended to the address in the last 4 bits.
             *
             * To get the true descriptor size, always mask off the last
             * 4 bits of the address.
             */
            pCppiDesc = (Ptr) ((uint32_t) pCppiDesc & 0xFFFFFFF0);

            pDataBuffer = (uint8_t *)(cppiMemRX[i]);
            /* Populate the Rx free descriptor with the buffer we just allocated. */
            Cppi_setData (Cppi_DescType_HOST, pCppiDesc, (uint8_t *)pDataBuffer, RX_BUF_SIZE);

            /* Save original buffer information */
            Cppi_setOriginalBufInfo (Cppi_DescType_HOST, pCppiDesc, (uint8_t *)pDataBuffer, RX_BUF_SIZE);


            /* Setup the Completion queue:
             *
             * Setup the return policy for this desc to return to the free q we just
             * setup instead of the global free queue.
             */
            Cppi_setReturnQueue (Cppi_DescType_HOST, pCppiDesc, rxFreeQInfo);

            Cppi_setSoftwareInfo (Cppi_DescType_HOST, pCppiDesc, (uint8_t *) mySWInfo);

            Cppi_setPacketLen    (Cppi_DescType_HOST, pCppiDesc, RX_BUF_SIZE);

            SYS_CACHE_WB (pCppiDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);

            /* Push descriptor to Tx free queue */
            Qmss_queuePushDescSize (gRxFreeQHnd, pCppiDesc, SIZE_HOST_DESC);
        }
        if (i != NUM_RX_DESC)
        {
            System_printf ("Error allocating Rx free descriptors \n");
            return -1;
        }
    }
	else {
  		  Qmss_Queue                  rxFreeQInfo;
          Qmss_QueueHnd               rxFreeQHnd;

   	      /*Now open the Rx Free Queue Handles for the slave process */
	      rxFreeQHnd  = (Qmss_QueueHnd) fw_shmGetEntry(gRxFreeQHndAddr);

            rxFreeQInfo.qNum = Qmss_getQIDFromHandle (rxFreeQHnd);
        /* Open a Rx Free Descriptor Queue (Rx FDQ).
         *
         * This queue will hold all the Rx free decriptors. These descriptors will be
         * used by the PASS CPDMA to hold data received via CPSW.
         */
        if ((gRxFreeQHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, rxFreeQInfo.qNum, &isAllocated)) < 0)
        {
            System_printf ("Error opening Rx Free descriptor queue \n");
            return -1;
        }

		if (gRxFreeQHnd != rxFreeQHnd) {
			System_printf ("Error Obtaining Tx Free descriptor handle \n");
			return -1;
		}
	}


    /* Setup a Rx Flow on each core. The only difference among the cores is the rxQInfo.
     *
     * A Rx flow encapsulates all relevant data properties that CPDMA would
     * have to know in order to succefully receive data.
     */
    /* Initialize the flow configuration */
    memset (&rxFlowCfg, 0, sizeof(Cppi_RxFlowCfg));
    rxFreeQInfo = Qmss_getQueueNumber (gRxFreeQHnd);

    /* Let CPPI pick the next available flow */
    rxFlowCfg.flowIdNum             =   CPPI_PARAM_NOT_SPECIFIED;

    rxFlowCfg.rx_dest_qmgr          =   rxQInfo.qMgr;
    rxFlowCfg.rx_dest_qnum          =   rxQInfo.qNum;
    rxFlowCfg.rx_desc_type          =   Cppi_DescType_HOST;

    rxFlowCfg.rx_ps_location        =   Cppi_PSLoc_PS_IN_DESC;
    rxFlowCfg.rx_psinfo_present     =   1;    /* Enable PS info */

    rxFlowCfg.rx_error_handling     =   0;    /* Drop the packet, do not retry on starvation by default */
    rxFlowCfg.rx_einfo_present      =   1;    /* EPIB info present */

    rxFlowCfg.rx_dest_tag_lo_sel    =   0;    /* Disable tagging */
    rxFlowCfg.rx_dest_tag_hi_sel    =   0;
    rxFlowCfg.rx_src_tag_lo_sel     =   0;
    rxFlowCfg.rx_src_tag_hi_sel     =   0;

    rxFlowCfg.rx_size_thresh0_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh1_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh2_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh0       =   0x0;
    rxFlowCfg.rx_size_thresh1       =   0x0;
    rxFlowCfg.rx_size_thresh2       =   0x0;

    rxFlowCfg.rx_fdq0_sz0_qmgr      =   rxFreeQInfo.qMgr; /* Setup the Receive free queue for the flow */
    rxFlowCfg.rx_fdq0_sz0_qnum      =   rxFreeQInfo.qNum;
    rxFlowCfg.rx_fdq0_sz1_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz1_qmgr      =   0x0;
    rxFlowCfg.rx_fdq0_sz2_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz2_qmgr      =   0x0;
    rxFlowCfg.rx_fdq0_sz3_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz3_qmgr      =   0x0;

    rxFlowCfg.rx_fdq1_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
    rxFlowCfg.rx_fdq1_qmgr          =   rxFreeQInfo.qMgr;
    rxFlowCfg.rx_fdq2_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
    rxFlowCfg.rx_fdq2_qmgr          =   rxFreeQInfo.qMgr;
    rxFlowCfg.rx_fdq3_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
    rxFlowCfg.rx_fdq3_qmgr          =   rxFreeQInfo.qMgr;

    /* Configure the Rx flow */
    if ((gRxFlowHnd = Cppi_configureRxFlow (gCpdmaHnd, &rxFlowCfg, &isAllocated)) == NULL)
    {
        System_printf ("Error configuring Rx flow \n");
        return -1;
    }

    /* All done with Rx configuration. Return success. */
    return 0;
}

/** ============================================================================
 *   @n@b PrintPkt
 *
 *   @b Description
 *   @n This API prints the packet in aligned format.
 *
 * =============================================================================
 */
void PrintPkt (uint8_t * pkt, uint32_t len, const char* func_name, uint32_t id)
{
#if 0
    uint32_t i;

	System_printf ("pkt information (len = %d) from function(core id %d): %s\n+++++++++++++++++++\n", len, id, func_name);

	for (i = 1; i <= len ; i++ ) {
	   System_printf ("0x%02x ", pkt[i-1]);
       if (!(i%8))
	   	  System_printf ("\n");

	}
	System_printf ("\n+++++++++++++++++++\n", func_name);
#endif
}


/** ============================================================================
 *   @n@b SendPacket
 *
 *   @b Description
 *   @n This API is called to actually send out data onto wire using ethernet.
 *      On success, this API increments a global Tx counter to indicate the same.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t SendPacket (void)
{
    Cppi_HostDesc*  pCppiDesc;
    uint32_t          dataBufferSize;
#ifndef __LINUX_USER_SPACE
    uint32_t        key;
#endif

    char            psFlags = (cpswSimTest)?pa_EMAC_PORT_NOT_SPECIFIED:pa_EMAC_PORT_1;

#ifdef NSS_GEN2
    Cppi_DescTag    tag;
#endif

    /* Get a free descriptor from the global free queue we setup
     * during initialization.
     */
    if ((pCppiDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("No Tx free descriptor. Cant run send/rcv test \n");
        return -1;
    }

    /* The descriptor address returned from the hardware has the
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor size, always mask off the last
     * 4 bits of the address.
     */
    pCppiDesc = (Ptr) ((uint32_t) pCppiDesc & 0xFFFFFFF0);


    dataBufferSize  =   PACKET_SIZE;

#ifndef __LINUX_USER_SPACE
    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();
#endif

    SYS_CACHE_INV (pCppiDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);

    Cppi_setData (  Cppi_DescType_HOST,
                    (Cppi_Desc *) pCppiDesc,
                    (uint8_t *) Convert_CoreLocal2GlobalAddr((uint32_t)pktMatch),
                    dataBufferSize
                 );
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, dataBufferSize);

#ifndef __LINUX_USER_SPACE
    if (cpswLpbkMode != CPSW_LOOPBACK_NONE)
    {
        /* Force the packet to specific EMAC port if loopback is enabled */
    	#ifndef NSS_GEN2
    	Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, psFlags);
    	#else
    	tag.srcTagHi  = 0;
    	tag.srcTagLo  = 0;
    	tag.destTagHi = 0;
    	tag.destTagLo = psFlags;
    	Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, (Cppi_DescTag *)&tag);
    	#endif

    }
    else
    {
        Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, 0);
    }
#else
    {
        /* Force the packet to specific EMAC port if loopback is enabled */
  #ifndef NSS_GEN2
    	Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, psFlags);
  #else
    	tag.srcTagHi  = 0;
    	tag.srcTagLo  = 0;
    	tag.destTagHi = 0;
    	tag.destTagLo = psFlags;
    	Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, (Cppi_DescTag *)&tag);
  #endif

    }
#endif

    /* Clear PS Data */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, 0);

    SYS_CACHE_WB (pCppiDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);

#ifndef __LINUX_USER_SPACE
    /* Reenable Interrupts. */
    Hwi_restore(key);
#endif

#ifndef __LINUX_USER_SPACE
    /* Send the packet out the mac. It will loop back to PA if the mac/switch
     * have been configured properly
     */
    if (no_bootMode == TRUE)
    	Qmss_queuePushDescSize (gPaTxQHnd[TF_PA_Q_EMAC], pCppiDesc, SIZE_HOST_DESC);
    else
    	Qmss_queuePushDescSize (gPaTxQHnd[TF_PA_Q_INPUT], pCppiDesc, SIZE_HOST_DESC); /* internal loop back */

#else
    Qmss_queuePushDescSize (gPaTxQHnd[TF_PA_Q_INPUT], pCppiDesc, SIZE_HOST_DESC);
#endif
    /* Increment the application transmit counter */
    gTxCounter ++;

    /* Give some time for the PA to process the packet */
    CycleDelay (10000);

    return 0;
}

/** ============================================================================
 *   @n@b ReceivePacket
 *
 *   @b Description
 *   @n This API is called to Receive packets.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t ReceivePacket (void)
{
	Cppi_Desc     *hd;
	Int            j;
    int32_t          status=0;

	/* Wait for a data packet from PA */
    for (j = 0; j < 100; j++)
    {
      CycleDelay (1000);
      if (Qmss_getQueueEntryCount (gRxQHnd) > 0)
      {
        hd = (Cppi_Desc *)(((uint32_t)Qmss_queuePop (gRxQHnd)) & ~0xf);
        if(VerifyPacket(hd) != 0)
            status=-1;
      }
    }

    return (status);
}

/** ============================================================================
 *   @n@b VerifyPacket
 *
 *   @b Description
 *   @n This API verifies a packet received against the expected data and
 *      returns 0 to inidcate success and -1 to indicate a mismatch.
 *
 *   @param[in]
 *   @n pCppiDesc           Packet descriptor received.
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t VerifyPacket (Cppi_Desc* pCppiDesc)
{
	Cppi_HostDesc               *pHostDesc;
	uint8_t                       *pDataBuffer;
	int32_t                       i;

    pHostDesc = (Cppi_HostDesc *)pCppiDesc;

#ifndef __LINUX_USER_SPACE
    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();
#endif

    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);

    SYS_CACHE_INV ((Ptr)(pHostDesc->buffPtr), pHostDesc->buffLen, CACHE_FENCE_WAIT);

    /* Verify the application software info we received is same
     * as what we had sent earlier.
     */
    if (pHostDesc->softwareInfo0 != 0xaaaaaaaa)
    {
        System_printf ("VerifyPacket: Found an entry in receive queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                        pHostDesc->softwareInfo0, 0xaaaaaaaa);

        pHostDesc->buffLen = pHostDesc->origBufferLen;
        SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
        Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

        return -1;
    }

    /* Verify the packet matches what we had sent */
    pDataBuffer = (uint8_t *) pHostDesc->buffPtr;
    for (i = 0; i < PACKET_SIZE; i++)
    {
        if (pktMatch[i] != pDataBuffer[i])
        {
            System_printf ("VerifyPacket: Byte %d expected 0x%02x, found 0x%02x\n", i, pktMatch[i], pDataBuffer[i]);
            System_flush();

            /* Free the packet back to the Rx FDQ */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
            return -1;
        }
    }

    /* Increment Rx counter to indicate the number of successfully
     * received packets by the example app.
     */
    gRxCounter ++;

    /* Reset the buffer lenght and put the descriptor back on the free queue */
    pHostDesc->buffLen = pHostDesc->origBufferLen;
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    Qmss_queuePush (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

    /* Verify packet done. Return success. */
	return 0;
}

/** ============================================================================
 *   @n@b ModifyPacket
 *
 *   @b Description
 *   @n This API modifies the data packet as a function on the core number.
 *      Each core will be associated with a different UDP port.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
void ModifyPacket (uint32_t destPortKey)
{
#ifdef __LINUX_USER_SPACE
    pktMatch = (void *)fw_memAlloc_pCore(PACKET_SIZE,CACHE_LINESZ);
    if(pktMatch == NULL)
    	APP_exit(-1);
    /* Populate the Tx free descriptor with the buffer. */
    memcpy(pktMatch,pktMatchBuf,PACKET_SIZE);
#else
	pktMatch = pktMatchBuf;
#endif

    /* Modify UDP destination port so that the packet is
       routed to the correct core*/
    pktMatch[PACKET_UDP_DEST_PORT_SHIFT+1] += destPortKey;

    /* Modify first byte of the packet payload to have
       an unique packet payload per core */
    pktMatch[PACKET_PAYLOAD_SHIFT] += destPortKey;


}

void closeAllOpenedQueues(uint32_t procId) {

	  int i;
	  extern Qmss_QueueHnd                           gPaCfgCmdRespQHnd;

	  /* The 10 PA transmit queues (corresponding to the 10 tx cdma channels */
   	  for (i = 0; i < NUM_PA_TX_QUEUES; i++)  {
   	  	 Qmss_queueEmpty (gPaTxQHnd[i]);
   	  	 Qmss_queueClose (gPaTxQHnd[i]);
   	  }

   	  if (procId == SYSINIT)
   	  {
   		Qmss_queueEmpty(gGlobalFreeQHnd);
   		Qmss_queueClose(gGlobalFreeQHnd);

   		/* Empty the remaining queues */
        Qmss_queueEmpty (gTxFreeQHnd);
   	    Qmss_queueEmpty (gRxFreeQHnd);
      }

	 Qmss_queueClose (gTxFreeQHnd);
	 Qmss_queueClose (gRxFreeQHnd);

  	 Qmss_queueEmpty (gPaCfgCmdRespQHnd);
	 Qmss_queueEmpty (gRxQHnd);
   	 Qmss_queueClose (gPaCfgCmdRespQHnd);
	 Qmss_queueClose (gRxQHnd);

	 return;

}

int clearFramework(uint32_t procId)
{
	int i;
	Qmss_Result qmss_result;
    Cppi_Result cppi_result;

    /* clean PA resources */
    if(Clear_PASS((int)procId) < 0)
        return (-1);

       /* clear the flows */
   if ((cppi_result = Cppi_closeRxFlow (gRxFlowHnd)) != CPPI_SOK) {
      	return (-1);
   }

   /* Close the queues that were setup */
   closeAllOpenedQueues(procId);

   if (procId == SYSINIT) {
        /* Close the cpDma setup */
       for (i = 0; i < NUM_PA_RX_CHANNELS; i++)  {
	   	 if (gCpdmaRxChanHnd[i]) {
	     	 if ((cppi_result = Cppi_channelClose (gCpdmaRxChanHnd[i])) != CPPI_SOK) {
	     	 	return (cppi_result);
	     	 }
	   	 }
       }
       for (i = 0; i < NUM_PA_TX_CHANNELS; i++)  {
	   	 if (gCpdmaTxChanHnd[i]) {
	     	 if ((cppi_result = Cppi_channelClose (gCpdmaTxChanHnd[i])) != CPPI_SOK) {
	     	 	return (cppi_result);
	     	 }
	   	 }
       }
   }

#ifndef __LINUX_USER_SPACE
	if (procId == SYSINIT)
#endif
	{
		/* Close CPPI CPDMA instance */
		if ((cppi_result = Cppi_close (gCpdmaHnd)) != CPPI_SOK)
		{
			errorCount++;
			System_printf ("Error Core %d : Closing CPPI CPDMA error code : %d\n", coreNum, cppi_result);
		}
		else
		{
			System_printf ("Core %d : CPPI CPDMA closed successfully\n", coreNum);
		}

		/* Deinitialize CPPI LLD */
		if ((cppi_result = Cppi_exit ()) != CPPI_SOK)
		{
			errorCount++;
			System_printf ("Error Core %d : Exiting CPPI error code : %d\n", coreNum, cppi_result);
		}
		else
		{
			System_printf ("Core %d : CPPI exit successful\n", coreNum);
		}
    }

	/* Free the memory regions */
	if (procId == SYSINIT) {
	    if ((qmss_result = Qmss_removeMemoryRegion (Qmss_MemRegion_MEMORY_REGION0, 0)) != QMSS_SOK)
	    {
	         System_printf ("Error Core : Remove memory region error code : %d\n", qmss_result);
	    }
	}

	/* Exit QMSS */
#ifdef __LINUX_USER_SPACE
    System_printf ("Core %d: exit QMSS\n", coreNum);
    if ((qmss_result = Qmss_exit ()))
    {
        errorCount++;
        System_printf ("Error Core %d : exit error code : %d\n", coreNum, qmss_result);
    }
#else
    if (procId == SYSINIT)
    {
        System_printf ("Core %d: exit QMSS\n", coreNum);
        while ((qmss_result = Qmss_exit()) != QMSS_SOK)
        {
            yield(); /* Wait for other cores to close their queues */
        }
    }
#endif
	return (0);
}


