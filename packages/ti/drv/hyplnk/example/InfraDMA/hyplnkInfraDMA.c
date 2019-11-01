/**
 *   @file  infrastructure_mode.c
 *
 *   @brief
 *      This is the QMSS infrastructure mode example code. Runs both in polling and accumulator mode.
 *      Uses monolithic descriptors for data transfer.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2016, Texas Instruments, Inc.
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
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <xdc/cfg/global.h>
#include <string.h>
#include <c6x.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/csl_cacheAux.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

/* CSL RL includes */
#include <ti/csl/csl_chip.h>
#include "hyplnkInfraDMA.h"


/************************ GLOBAL VARIABLES ********************/
#pragma DATA_SECTION(monolithicDesc,".bss:QMSSData")//use MSMC memory for test mode
#pragma DATA_ALIGN (monolithicDesc, hyplnk_EXAMPLE_LINE_SIZE)
UInt8               monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC];
#pragma DATA_ALIGN (txDataBuff, hyplnk_EXAMPLE_LINE_SIZE)
UInt8                   txDataBuff[SIZE_DATA_BUFFER_PACKET];
#pragma DATA_ALIGN (rxDataBuff, hyplnk_EXAMPLE_LINE_SIZE)
UInt8                   rxDataBuff[SIZE_DATA_BUFFER_PACKET * NUM_PACKETS];
/* CPDMA configuration */
Cppi_CpDmaInitCfg       cpdmaCfg;
/* Tx channel configuration */
Cppi_TxChInitCfg        txChCfg;
/* Rx channel configuration */
Cppi_RxChInitCfg        rxChCfg;
/* Rx flow configuration */
Cppi_RxFlowCfg          rxFlowCfg;
/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         monoMemInfo;
/* Handle to CPPI heap */
IHeap_Handle            cppiHeap;
/* Core number */
UInt32                  coreNum;
/* Define queues for common FDQs */
#define MONO_TX_FDQ            2000
#define MONO_RX_FDQ            2001

#define CPPI_MEM_SIZE 0x1000
#define QMSS_MEM_SIZE 0x1000

typedef struct {
    unsigned int cppi_mem_idx;
    unsigned char cppi_mem[CPPI_MEM_SIZE];
} cppi_mem_mgmt;

typedef struct {
    unsigned int qmss_mem_idx;
    unsigned char qmss_mem[QMSS_MEM_SIZE];
} qmss_mem_mgmt;

/* Allocate memory for CPPI and QMSS*/
#pragma DATA_SECTION(cppi_malloc_mem, ".cppi");
#pragma DATA_ALIGN (cppi_malloc_mem, hyplnk_EXAMPLE_LINE_SIZE)
cppi_mem_mgmt cppi_malloc_mem;
#pragma DATA_SECTION(qmss_malloc_mem, ".qmss");
#pragma DATA_ALIGN (qmss_malloc_mem, hyplnk_EXAMPLE_LINE_SIZE)
qmss_mem_mgmt qmss_malloc_mem;

#ifdef INFRA_DEBUG_MODE
	int debug=1;
#else
	int debug=0;
#endif

/************************ EXTERN VARIABLES ********************/

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;
/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/*************************** FUNCTIONS ************************/

/* The following functions initialize CPPI and QMSS memory index to zero */
void initCppiMem () {

    cppi_malloc_mem.cppi_mem_idx = 0;

}

void initQmssMem () {

    qmss_malloc_mem.qmss_mem_idx = 0;

}

/**
 *  @b Description
 *  @n
 *      Utility function which converts a local GEM L2 memory address
 *      to global memory address.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Computed L2 global Address
 */

static UInt32 l2_global_address (UInt32 addr)
{
	UInt32 corenum;

	if ((addr >= 0x800000) && (addr < 0xa00000)) {
	    /* Get the core number. */
	    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);

	    /* Compute the global address. */
	    return (addr + (0x10000000 + (corenum * 0x1000000)));
	}else{
		return addr;
	}
}

/**
 *  @b Description
 *  @n
 *      Simple compare function used in qsort function.
 *      This is used to sort the addresses for proper memory
 *      region placement
 *
 *  @param[in]  address1
 *      Address to be compared to address2
 *
 *  @param[in]  address2
 *      Address to be compared to address1
 *
 *  @retval
 *      Comparison value of the two addresses.
 */
int compareAddresses(const void* address1, const void* address2){
	hyplnkMemRegionsAddress_t *adr1 = (hyplnkMemRegionsAddress_t*)address1;
	hyplnkMemRegionsAddress_t *adr2 = (hyplnkMemRegionsAddress_t*)address2;

	return ( adr1->MRaddresses - adr2->MRaddresses );
}


/**
 *  @b Description
 *  @n
 *      Entry point for the example code.
 *      This is an QMSS infrastructure mode example. Works in both polled or accumulated mode
 *
 *      It performs the following
 *          - Initializes the Queue Manager low level driver.
 *          - Initializes the CPPI low level driver.
 *          - Opens the CPPI CPDMA in queue manager
 *
 *  @param[in]  remoteQMSSPtr
 *     Remote address for queue manager
 *
 *  @param[in]  remoteMemRegionPtr
 *     Local address for data to be transfered
 *
 *  @param[in]  cppiHnd
 *     Empty handle that will be intialized
 *
 *  @retval
 *      Handle for data transfer and deinitializes functions
 */
Cppi_Handle InfraDMAInit (int remoteQMSSPtr, int remoteMemRegionPtr, Qmss_Result *localRegion, Qmss_Result *remoteRegion, Cppi_Handle cppiHnd)
{
    Qmss_Result   result;
    hyplnkMemRegionsAddress_t MemoryRegion[2];

    if(debug) printf ("**************************************************\n");
    if(debug) printf ("*********QMSS Infrastructure Mode Example ********\n");
    if(debug) printf ("**************************************************\n");

    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

    /* Initialize the heap in shared memory for CPPI data structures */
	initCppiMem();
	initQmssMem();

	if(debug)printf ("********************Test running on Core %d ********************\n", coreNum);

	if(debug)printf ("\n-----------------------Initializing---------------------------\n");

	/* Initialize data buffer and QMSS configuration*/
    memset(txDataBuff, 0x00, sizeof(txDataBuff));
    memset ((Void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up the linking RAM. Use the internal Linking RAM.
     * LLD will configure the internal linking RAM address and maximum internal linking RAM size if
     * a value of zero is specified.
     * Linking RAM1 is not used */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_MONOLITHIC_DESC*2;

#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_le);
#endif

    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", coreNum, result);
        return NULL;
    }

    /* Start Queue Manager SubSystem */
    result = Qmss_start ();
    if (result != QMSS_SOK)
    {
        printf ("Core %d : Error starting Queue Manager error code : %d\n", coreNum, result);
    }

    /* Initialize CPPI LLD */
    result = Cppi_init (&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        printf ("Error Core %d : Initializing CPPI LLD error code : %d\n", coreNum, result);
    }

    /* Set up QMSS CPDMA configuration */
    memset ((Void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
    cpdmaCfg.dmaNum = Cppi_CpDma_QMSS_CPDMA;
    cpdmaCfg.qm0BaseAddress = QMSS_DATA_ADDR; //local QM area
    cpdmaCfg.qm1BaseAddress = QMSS_DATA_ADDR + 0x00010000; //local QM area
    cpdmaCfg.qm2BaseAddress = remoteQMSSPtr; //remote QM area
    if(debug)printf ("Remote Queue Pointer for qm2BaseAddress 0x%p\n", remoteQMSSPtr);

    /* Open QMSS CPDMA */
    cppiHnd = (Cppi_Handle) Cppi_open (&cpdmaCfg);
    if (cppiHnd == NULL)
    {
        printf ("Error Core %d : Initializing QMSS CPPI CPDMA %d\n", coreNum, cpdmaCfg.dmaNum);
        return NULL;
    }

    /* Sort addresses to configure memory regions in order */
    MemoryRegion[0].MRaddresses=(UInt32)(l2_global_address ((UInt32) monolithicDesc));
    MemoryRegion[1].MRaddresses=(UInt32)remoteMemRegionPtr;
    MemoryRegion[0].isRemote = 0;
    MemoryRegion[1].isRemote = 1;
    qsort(MemoryRegion,2,sizeof(MemoryRegion[0]),compareAddresses);

    memset(monolithicDesc, 0xFF, sizeof(monolithicDesc));
    memset ((Void *) &monolithicDesc, 0, SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC);

    /* Setup memory region 0 for local QM */
    monoMemInfo.descBase = (UInt32 *)  MemoryRegion[0].MRaddresses;
    monoMemInfo.descSize = SIZE_MONOLITHIC_DESC;
    monoMemInfo.descNum = NUM_MONOLITHIC_DESC;
    monoMemInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    monoMemInfo.memRegion = Qmss_MemRegion_MEMORY_REGION0;
    monoMemInfo.startIndex = 0;

    result = Qmss_insertMemoryRegion (&monoMemInfo);
    if (result < QMSS_SOK)
    {
        printf ("Error Core %d : Inserting memory region %d error code : %d\n", coreNum, monoMemInfo.memRegion, result);
    }
    else
    	if(debug)printf ("Core %d : Memory region %d inserted\n", coreNum, result);

    if (MemoryRegion[0].isRemote)
    	*remoteRegion = result;
    else
    	*localRegion = result;

	/* Setup memory region 1 for remote QM */
    monoMemInfo.descBase = (UInt32 *)  MemoryRegion[1].MRaddresses;
	monoMemInfo.descSize = SIZE_MONOLITHIC_DESC;
	monoMemInfo.descNum = NUM_MONOLITHIC_DESC;
	monoMemInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
	monoMemInfo.memRegion = Qmss_MemRegion_MEMORY_REGION1;
	monoMemInfo.startIndex = 32;

	result = Qmss_insertMemoryRegion(&monoMemInfo);
	if (result < QMSS_SOK) {
		printf("Error Core %d : Inserting memory region %d error code : %d\n",coreNum, monoMemInfo.memRegion, result);
	} else
		if(debug)printf("Core %d : Memory region %d inserted\n", coreNum, result);

	if (MemoryRegion[1].isRemote)
    	*remoteRegion = result;
    else
    	*localRegion = result;

	return cppiHnd;
}

/**
 *  @b Description
 *  @n
 *      Data transfer using monolithic descriptors
 *
 *      It performs the following
 *          - Initializes descriptors and pushes to free queue
 *          - Programs accumulator
 *          - Pushes packets on Tx channel
 *          - Processes the accumulated packets from Rx channel
 *          - Closes Rx and Tx channel
 *          - Closes all open queues
 *
 *  @param[in]  cppiHnd
 *      Handle to access CPPI queue manager CPDM
 *
 *  @param[out]  RXpacket
 *      Array of blocks that holds the output packets
 *
 *  @param[in]  transferValue
 *      Value that will be set into the descriptor multiple times before being sent out.
 *
 *  @retval
 *      Total transfer time
 */
int alreadyAllocated = 0;
int InfraDMATransfer (Cppi_Handle cppiHnd, uint32_t transferValue, Qmss_Result localRegion,Qmss_Result remoteRegion,
		hyplnkQMSSExamplePacketBlock_t *HyperLinkMemRegAddr)
{
    Qmss_Result             result;
    UInt32                  numAllocated, i, length, destLen;
    uint32_t                flowId;
    UInt8                   isAllocated;
    UInt8                   *dataBuffPtr;
    Cppi_ChHnd              rxChHnd, txChHnd;
    Qmss_QueueHnd           txQueHnd, rxQueHnd, freeQueHnd, txFreeQueHnd,txCmplQueHnd;
    Cppi_DescCfg            descCfg;
    Cppi_Desc               *monoDescPtr;
    Cppi_FlowHnd            rxFlowHnd;
    Qmss_Queue              queInfo;
    Cppi_Desc               *rxPkt;
    uint64_t                startTime, totalTime;
    Cppi_DescTag            tag;

    if(debug) printf ("\n***************** Data transfer monolithic mode ***************\n\n");

    /* Setup the descriptors for transmit free queue */
    /* Memory region obtained is zero since there is only Qmss_insertMemoryRegion() call.
     * else find the memory region using Qmss_getMemoryRegionCfg() */
    descCfg.memRegion = (Qmss_MemRegion) localRegion;
    if(alreadyAllocated==0) descCfg.descNum = NUM_MONOLITHIC_DESC/2;
    else descCfg.descNum = 0;
    descCfg.destQueueNum = MONO_TX_FDQ;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_MONOLITHIC;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;

    descCfg.cfg.mono.dataOffset = MONOLITHIC_DESC_DATA_OFFSET;
    
    /* Descriptor should be recycled back to Queue Number 1000 */
    descCfg.returnQueue.qMgr = CPPI_COMPLETION_QUE_MGR;
    descCfg.returnQueue.qNum = MONO_TX_FDQ;

    //if(alreadyAllocated==0) memset((void *) (l2_global_address ((UInt32) monolithicDesc)),0,sizeof(monolithicDesc));

    /* Initialize the descriptors and push to free Queue */
    if ((txFreeQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
	{
        printf ("Error Core %d : Initializing Tx descriptor error code: %d \n", coreNum, txFreeQueHnd);
		return -1;
	}
    else
    	if(debug)printf ("Core %d : Number of Tx descriptors requested : %d. Number of descriptors allocated : %d \n", coreNum, descCfg.descNum, numAllocated);

#ifdef hyplnk_CACHE_ENABLE
	CACHE_wbInvL1d ((void *) (l2_global_address ((UInt32) monolithicDesc)), sizeof(monolithicDesc), CACHE_FENCE_WAIT);
	CACHE_wbInvL2  ((void *) (l2_global_address ((UInt32) monolithicDesc)), sizeof(monolithicDesc), CACHE_FENCE_WAIT);
#endif

    /* Setup the descriptors for receive free queue */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = (Qmss_MemRegion) remoteRegion;
    if(alreadyAllocated==0) descCfg.descNum = NUM_MONOLITHIC_DESC/2;
    else descCfg.descNum = 0;
    descCfg.destQueueNum = MONO_RX_FDQ;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_MONOLITHIC;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.cfg.mono.dataOffset = MONOLITHIC_DESC_DATA_OFFSET;

    /* Descriptor should be recycled back to queue allocated above */
    descCfg.returnQueue.qMgr = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qNum = QMSS_PARAM_NOT_SPECIFIED;
    
    //if(alreadyAllocated==0) memset((void *) HyperLinkMemRegAddr,0, sizeof(monolithicDesc));//using a temp address

    /* Initialize the descriptors and push to free Queue */
    if ((freeQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
	{
        printf ("Error Core %d : Initializing Rx descriptor error code: %d \n", coreNum, freeQueHnd);
		return -1;
	}
    else
    	if(debug) printf ("Core %d : Number of Rx descriptors requested : %d. Number of descriptors allocated : %d \n", coreNum, descCfg.descNum, numAllocated);
    alreadyAllocated=1;

#ifdef hyplnk_CACHE_ENABLE
	CACHE_wbInvL1d ((void *) HyperLinkMemRegAddr, sizeof(monolithicDesc), CACHE_FENCE_WAIT);
	CACHE_wbInvL2  ((void *) HyperLinkMemRegAddr, sizeof(monolithicDesc), CACHE_FENCE_WAIT);
#endif

    /* Set up Rx Channel parameters */
    rxChCfg.channelNum = 0;
    rxChCfg.rxEnable = Cppi_ChState_CHANNEL_DISABLE;
    
    /* Open Rx Channel */
    rxChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
    if (rxChHnd == NULL)
    {
        printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
		return -1;
    }
    else
    	if(debug) printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));

    /* Set up Tx Channel parameters */
    txChCfg.channelNum = 0;
    txChCfg.priority = 0;
    txChCfg.filterEPIB = 0;
    txChCfg.filterPS = 0;
    txChCfg.aifMonoMode = 0;
    txChCfg.txEnable = Cppi_ChState_CHANNEL_DISABLE;
    
    /* Open Tx Channel */
    txChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
    if (txChHnd == NULL)
    {
        printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
		return -1;
    }
    else
    	if(debug) printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));

    /* Opens transmit queue. This is the infrastructure queue */
    if ((txQueHnd = Qmss_queueOpen (Qmss_QueueType_INFRASTRUCTURE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
	{
        printf ("Error Core %d : Opening Transmit Queue Number\n", coreNum);
		return -1;
	}
    else
    	if(debug) printf ("Core %d : Transmit Queue Number : %d opened\n", coreNum, txQueHnd);

    /* Opens receive queue */
    if ((rxQueHnd = Qmss_queueOpenInGroup (0, Qmss_QueueType_HIGH_PRIORITY_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
	{
        printf ("Error Core %d : Opening Receive Queue Number\n", coreNum);
		return -1;
	}
    else
    	if(debug) printf ("Core %d : Receive Queue Number : %d opened\n", coreNum, rxQueHnd);

    /* Opens transmit completion queue. */
    if ((txCmplQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, CPPI_COMPLETION_QUE_NUM, &isAllocated)) < 0)
	{
        printf ("Error Core %d : Opening Tx Completion Queue Number\n", coreNum);
		return -1;
	}
    else
    	if(debug) printf ("Core %d : Tx Completion Queue Number : %d opened\n", coreNum, txCmplQueHnd);

    if(debug) printf ("Core %d : Free Queue Number : %d opened\n", coreNum, freeQueHnd);
    if(debug) printf ("Core %d : Transmit Free Queue Number : %d opened\n", coreNum, txFreeQueHnd);


    /* Set transmit queue threshold to high and when there is at least one packet */
    /* Setting threshold on transmit queue is not required anymore. tx pending queue is not hooked to threshold.
     * Qmss_setQueueThreshold (txQueHnd, 1, 1);
     */

    /* Setup Rx flow parameters */
    memset ((Void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Don't specify flow number and let CPPI allocate the next available one */
    rxFlowCfg.flowIdNum = 0;
    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (rxQueHnd);
    if(debug) printf ("queInfo.qMgr value %d \n",queInfo.qMgr);
    rxFlowCfg.rx_dest_qnum = queInfo.qNum;
    rxFlowCfg.rx_dest_qmgr = 2;
    rxFlowCfg.rx_sop_offset = MONOLITHIC_DESC_DATA_OFFSET;
    rxFlowCfg.rx_desc_type = Cppi_DescType_MONOLITHIC; 

    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (freeQueHnd);
    if (queInfo.qNum >4096)
    {
        printf ("Error Core %d : Queue number %d is invalid\n", coreNum, queInfo.qNum);
		return -1;
    }
    rxFlowCfg.rx_fdq0_sz0_qnum = queInfo.qNum;
    rxFlowCfg.rx_fdq0_sz0_qmgr = 2+queInfo.qMgr;



    /* Fill in some data */
    /* Since incoming data is 32 bits and data can only be read in bytes,
     * the following adjustments are needed to get the whole word*/
      for (i = 0; i < (SIZE_DATA_BUFFER_PACKET/4); i++){
#ifdef _BIG_ENDIAN
    	  txDataBuff[4*i+0] =transferValue>>24;
    	  txDataBuff[4*i+1] =transferValue>>16;
    	  txDataBuff[4*i+2] =transferValue>>8;
    	  txDataBuff[4*i+3] =transferValue>>0;
#else
    	  txDataBuff[4*i+0] =transferValue>>0;
    	  txDataBuff[4*i+1] =transferValue>>8;
    	  txDataBuff[4*i+2] =transferValue>>16;
    	  txDataBuff[4*i+3] =transferValue>>24;
#endif
      }

    /* Configure Rx flow */
    rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
    if (rxFlowHnd == NULL)
    {
        printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
		return -1;
    }

    flowId = Cppi_getFlowId(rxFlowHnd);
    if(debug) printf ("Core %d : Opened Rx flow : %d\n", coreNum, flowId);


    if(debug) printf ("\n--------------------Transmitting packets----------------------\n");


    for (i = 0; i < NUM_PACKETS; i++)
    {
        /* Get a free descriptor */
        if ((monoDescPtr = (Cppi_Desc *) Qmss_queuePop (txFreeQueHnd)) == NULL)
        {
            printf ("Error Core %d : Getting descriptor from Queue Number\n", coreNum, txFreeQueHnd);
    		return -1;
        }

#ifdef hyplnk_CACHE_ENABLE
		CACHE_invL1d ((void *) monoDescPtr, SIZE_MONOLITHIC_DESC, CACHE_WAIT);
		CACHE_invL2  ((void *) monoDescPtr, SIZE_MONOLITHIC_DESC, CACHE_WAIT);
#endif
        /* Add data buffer */
        Cppi_setData (Cppi_DescType_MONOLITHIC, monoDescPtr, (UInt8 *) &txDataBuff, SIZE_DATA_BUFFER_PACKET);

#ifdef hyplnk_CACHE_ENABLE
		CACHE_wbInvL1d ((void *) monoDescPtr, SIZE_MONOLITHIC_DESC, CACHE_FENCE_WAIT);
		CACHE_wbInvL2  ((void *) monoDescPtr, SIZE_MONOLITHIC_DESC, CACHE_FENCE_WAIT);
#endif

        /* Set tag information - this selects the flow */
        tag.destTagLo = 0;
        tag.destTagHi = 0;
        tag.srcTagLo = flowId;
        tag.srcTagHi = 0;

        Cppi_setTag (Cppi_DescType_MONOLITHIC, monoDescPtr, &tag);

        /* Set packet length */
        Cppi_setPacketLen (Cppi_DescType_MONOLITHIC, monoDescPtr, SIZE_DATA_BUFFER_PACKET);

        if(debug) printf ("Core %d : Transmitting descriptor 0x%p\n", coreNum, monoDescPtr);

#ifdef hyplnk_CACHE_ENABLE
		CACHE_wbInvL1d ((void *) monoDescPtr, SIZE_MONOLITHIC_DESC, CACHE_FENCE_WAIT);
		CACHE_wbInvL2  ((void *) monoDescPtr, SIZE_MONOLITHIC_DESC, CACHE_FENCE_WAIT);
#endif

        /* Push descriptor to Tx queue */
        Qmss_queuePushDescSize (txQueHnd, (UInt32 *) monoDescPtr, MONOLITHIC_DESC_DATA_OFFSET);
    }

    startTime = hyplnkExampleReadTime(); //Start time as soon as channels are enabled

    /* Enable transmit channel */
       if (Cppi_channelEnable (txChHnd) != CPPI_SOK)
           printf ("Error Core %d : Enabling Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));
       else
    	   if(debug) printf ("Core %d : Tx channel : %d enabled \n", coreNum, Cppi_getChannelNumber (txChHnd));

       /* Enable receive channel */
       if (Cppi_channelEnable (rxChHnd) != CPPI_SOK)
           printf ("Error Core %d : Enabling Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));
       else
    	   if(debug) printf ("Core %d : Rx channel : %d enabled \n", coreNum, Cppi_getChannelNumber (rxChHnd));


       if(debug){
    	      printf ("\n-------------------------Queue status-------------------------\n");
    result = Qmss_getQueueEntryCount (txQueHnd);
    printf ("Transmit Queue %d Entry Count : %d \n", txQueHnd, result);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    printf ("Tx Free Queue %d Entry Count : %d \n", txFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (freeQueHnd);
    printf ("Rx Free Queue %d Entry Count : %d \n", freeQueHnd, result);

    result = Qmss_getQueueEntryCount (rxQueHnd);
    printf ("Receive Queue %d Entry Count : %d \n", rxQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd, result);

    printf ("\n--------------------Receiving packets-------------------------\n");
       }


    while (Qmss_getQueueEntryCount (rxQueHnd) < NUM_PACKETS);


    /* Get each rx packet information and recycle descriptor*/
    while ((rxPkt = (Cppi_Desc *) QMSS_DESC_PTR (Qmss_queuePop (rxQueHnd))) != NULL)
    {
#ifdef hyplnk_CACHE_ENABLE
		CACHE_invL1d ((void *) rxPkt, SIZE_MONOLITHIC_DESC, CACHE_WAIT);
		CACHE_invL2  ((void *) rxPkt, SIZE_MONOLITHIC_DESC, CACHE_WAIT);
#endif
        length = Cppi_getPacketLen (Cppi_DescType_MONOLITHIC, rxPkt);

        if(debug) printf ("Core %d : Received descriptor 0x%p of length : %d\n", coreNum, rxPkt, length);

        /* Get data buffer */
        Cppi_getData (Cppi_DescType_MONOLITHIC, rxPkt, &dataBuffPtr, &destLen);

#ifdef hyplnk_CACHE_ENABLE
		CACHE_wbInvL1d ((void *) rxPkt, SIZE_MONOLITHIC_DESC, CACHE_FENCE_WAIT);
		CACHE_wbInvL2  ((void *) rxPkt, SIZE_MONOLITHIC_DESC, CACHE_FENCE_WAIT);
#endif

        /* Recycle the descriptors */
        queInfo = Cppi_getReturnQueue (Cppi_DescType_MONOLITHIC, rxPkt);

#ifdef hyplnk_CACHE_ENABLE
		CACHE_wbInvL1d ((void *) rxPkt, SIZE_MONOLITHIC_DESC, CACHE_FENCE_WAIT);
		CACHE_wbInvL2  ((void *) rxPkt, SIZE_MONOLITHIC_DESC, CACHE_FENCE_WAIT);
#endif

        /* Push descriptor back to free queue */
        Qmss_queuePushDesc (Qmss_getQueueHandle(queInfo), (UInt32 *) rxPkt);
    }

    totalTime = (hyplnkExampleReadTime() - startTime);

    if(debug) printf ("\n--------------------Deinitializing----------------------------\n");

    if(debug){
    result = Qmss_getQueueEntryCount (txQueHnd);
    printf ("Transmit Queue %d Entry Count : %d \n", txQueHnd, result);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    printf ("Tx Free Queue %d Entry Count : %d \n", txFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (freeQueHnd);
    printf ("Rx Free Queue %d Entry Count : %d \n", freeQueHnd, result);

    result = Qmss_getQueueEntryCount (rxQueHnd);
    printf ("Receive Queue %d Entry Count : %d \n", rxQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd, result);
    }

    /* Close Tx channel */
    if ((result = Cppi_channelClose (txChHnd)) != CPPI_SOK){
        printf ("Error Core %d : Closing Tx channel error code : %d\n", coreNum, result);
	    return -1;
    }
    else
    	if(debug) printf ("Core %d : Tx Channel closed successfully. Ref count : %d\n", coreNum, result);

    /* Close Rx channel */
    if ((result = Cppi_channelClose (rxChHnd)) != CPPI_SOK){
        printf ("Error Core %d : Closing Rx channel error code : %d\n", coreNum, result);
		return -1;
    }
    else
    	if(debug) printf ("Core %d : Rx Channel closed successfully. Ref count : %d\n", coreNum, result);

    /* Close Rx flow */
    if ((result = Cppi_closeRxFlow (rxFlowHnd)) != CPPI_SOK){
        printf ("Error Core %d : Closing Rx flow error code : %d\n", coreNum, result);
		return -1;
    }
    else
    	if(debug)  printf ("Core %d : Rx flow closed successfully. Ref count : %d\n", coreNum, result);

    /* Close the queues */
    if ((result = Qmss_queueClose (rxQueHnd)) != CPPI_SOK){
        printf ("Error Core %d : Closing Rx queue error code : %d\n", coreNum, result);
		return -1;
    }
    else
    	if(debug) printf ("Core %d : Rx queue closed successfully. Ref count : %d\n", coreNum, result);

    if ((result = Qmss_queueClose (txQueHnd)) != CPPI_SOK){
        printf ("Error Core %d : Closing tx queue error code : %d\n", coreNum, result);
		return -1;
    }
    else
    	if(debug) printf ("Core %d : Tx queue closed successfully. Ref count : %d\n", coreNum, result);

    if ((result = Qmss_queueClose (freeQueHnd)) != CPPI_SOK){
        printf ("Error Core %d : Closing free queue error code : %d\n", coreNum, result);
		return -1;
    }
    else
    	if(debug) printf ("Core %d : Free queue closed successfully. Ref count : %d\n", coreNum, result);

    if ((result = Qmss_queueClose (txCmplQueHnd)) != CPPI_SOK){
        printf ("Error Core %d : Closing transmit completion queue error code : %d\n", coreNum, result);
		return -1;
    }
    else
    	if(debug) printf ("Core %d : Transmit completion queue closed successfully. Ref count : %d\n", coreNum, result);

    if ((result = Qmss_queueClose (txFreeQueHnd)) != CPPI_SOK){
        printf ("Error Core %d : Closing transmit freequeue error code : %d\n", coreNum, result);
		return -1;
    }
    else
    	if(debug) printf ("Core %d : Transmit free queue closed successfully. Ref count : %d\n", coreNum, result);

    return totalTime;
}

/**
 *  @b Description
 *  @n
 *      Deinitializes CPPI
 *
 *      It performs the following
 *          - Closes CPDMA instance
 *          - Deinitializes CPPI LLD
 *
 *  @param[in]  cppiHnd
 *      Handle to access CPPI queue manager CPDMA
 *
 *  @retval
 *      Nothing
 */
void InfraDMADeinit(Cppi_Handle cppiHnd) {
	Qmss_Result result;
	coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

	/* Close CPPI CPDMA instance */
	if ((result = Cppi_close(cppiHnd)) != CPPI_SOK)
		printf("Error Core %d : Closing CPPI CPDMA error code : %d\n",
				coreNum, result);
	else
		if(debug) printf("Core %d : CPPI CPDMA closed successfully\n", coreNum);

	/* Deinitialize CPPI LLD */
	if ((result = Cppi_exit()) != CPPI_SOK)
		printf("Error Core %d : Exiting CPPI error code : %d\n", coreNum,
				result);
	else
		if(debug) printf("Core %d : CPPI exit successful\n", coreNum);

	if(debug) printf("*******************************************************\n");
	if(debug) printf("*********QMSS Infrastructure Mode Example Done*********\n");
	if(debug) printf("*******************************************************\n");
}


