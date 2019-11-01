/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include <xdc/std.h>
#include <xdc/cfg/global.h>

/* XDC.RUNTIME module Headers */
#include <xdc/runtime/System.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Timestamp.h>

/* IPC module Headers */
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/HeapBufMP.h>
#include <ti/ipc/SharedRegion.h>

/* PDK module Headers */
#include <ti/platform/platform.h>

/* BIOS6 module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/c66/Cache.h>

/* CSL modules */
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_chip.h>

/* QMSS LLD */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

/* CPPI LLD */
#include <ti/drv/cppi/cppi_drv.h>

/* SRIO LLD */
#include <ti/drv/srio/srio_drv.h>
 
#include <ti/transport/ipc/examples/common/bench_common.h>

/************************ EXTERN VARIABLES ********************/
/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;
/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;
/**************************************************************/

#define NUM_HOST_DESC         numDescriptors
#define SIZE_HOST_DESC        descriptorSize
#define SIZE_CPPI_HEAP		  cppiHeapSize

#define SRIO_MTU_SIZE srioMtuSize

#define HEAP_NAME   "myHeapBuf"
#define HEAP_ID         appMsgQHeapId

/* Number of times to run the loop */
#define NUMLOOPS        100
#define NUMIGNORED      (5)
#define NUM_MSGS        (10)

/* Benchmark parameters */
Char localQueueName[6];
Char nextQueueName[6];
Char prevQueueName[6];

UInt numCores = 0;
UInt16 prevCoreId;

UInt16 selfId;
UInt64 timeAdj = 0;
Types_FreqHz timerFreq, cpuFreq;

/* Results */
UInt32 rawtimestamps[NUMLOOPS];
UInt32 latencies[NUMLOOPS - 1];

MessageQ_Handle messageQ = NULL;
MessageQ_QueueId nextQueueId, prevQueueId;

UInt64 timeLength = 0;

/* These are the device identifiers used used in the TEST Application */
const UInt32 DEVICE_ID1_16BIT    = Srio16BitDeviceId1;
const UInt32 DEVICE_ID1_8BIT     = Srio8BitDeviceId1;
const UInt32 DEVICE_ID2_16BIT    = Srio16BitDeviceId2;
const UInt32 DEVICE_ID2_8BIT     = Srio8BitDeviceId2;

/* These are the garbage queues used in the test Application */
const UInt32 GARBAGE_LEN_QUEUE = 905;
const UInt32 GARBAGE_TOUT_QUEUE = SrioGarbageQ; /* Timeout errors must be cleaned up by transport */
const UInt32 GARBAGE_RETRY_QUEUE = 907;
const UInt32 GARBAGE_TRANS_ERR_QUEUE = SrioGarbageQ; /* Transmission errors must be cleaned up by transport */
const UInt32 GARBAGE_PROG_QUEUE = 909;
const UInt32 GARBAGE_SSIZE_QUEUE = 910;

Float cpuTimerFreqRatio;

Statistics latencyStats;


/* Descriptor pool [Size of descriptor * Number of descriptors] */
/* place this host descritor pool in shared memory */
#pragma DATA_SECTION (hostDesc, ".desc");
#pragma DATA_ALIGN (hostDesc, 16)
UInt8               hostDesc[SIZE_HOST_DESC * NUM_HOST_DESC];

/* Statically created shared heap for CPPI since IPC does create a
 * shared heap for SharedRegion prior to Ipc_attach */
#pragma DATA_SECTION (cppiHeap, ".cppi_heap");
#pragma DATA_ALIGN (cppiHeap, 128)
UInt8               cppiHeap[SIZE_CPPI_HEAP];

#define NUM_MSGS_TO_PREALLOC (5)

#pragma DATA_SECTION (txMsgPtrs, ".msgQ_ptrs");
TstMsg *txMsgPtrs[NUM_MSGS_TO_PREALLOC];

#pragma DATA_SECTION (rxMsgPtrs, ".msgQ_ptrs");
TstMsg *rxMsgPtrs[NUM_MSGS_TO_PREALLOC];

#pragma DATA_SECTION (isSRIOInitialized, ".srioSharedMem");
volatile Uint32     isSRIOInitialized    = 0;

/**
 *  @b Description
 *  @n  
 *      The function provides the initialization sequence for the SRIO IP
 *      block. This can be modified by customers for their application and
 *      configuration.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
extern int32_t SrioDevice_init (void);

/**
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for SRIO. 
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static Int32 enable_srio (void)
{
#ifndef SIMULATOR_SUPPORT
    /* SRIO power domain is turned OFF by default. It needs to be turned on before doing any 
     * SRIO device register access. This not required for the simulator. */

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
#else
    /* PSC is not supported on simulator. Return success always */
    return 0;
#endif
}

/**
 *  @b Description
 *  @n  
 *      This functions prints the statistics gathered for the transport during
 *      the latency test.
 */
Void printStatistics()
{
    UInt32 timeElapsed;
    UInt i;

    /* Convert timestamps to CPU time */
    for (i = 0; i < NUMLOOPS; i++) {
        rawtimestamps[i] *= cpuTimerFreqRatio;
    }
    
    for (i = 0; i < NUMLOOPS - 1; i++) {
        latencies[i] = (rawtimestamps[i + 1] - rawtimestamps[i]) / numCores;
    }

    getStats(latencies + NUMIGNORED, NUMLOOPS - NUMIGNORED - 2, &latencyStats);
    
    timeElapsed =  rawtimestamps[NUMLOOPS - NUMIGNORED - 2] -
            rawtimestamps[NUMIGNORED];
    /* Throughput = time elapsed divided by total #of of hops */
    
    System_printf("======== SYSTEM ATTRIBUTES ======== \n");
    System_printf("Device name:                  %s\n", DEVICENAME);
    System_printf("Processor names:              %s\n", PROCNAMES);
    System_printf("CPU Freq:                     %d MHz\n", 
        cpuFreq.lo / 1000000);
    System_printf("Timer Freq:                   %d MHz\n\n", 
        timerFreq.lo / 1000000);

    System_printf("======== BENCHMARK ATTRIBUTES ======== \n");
    System_printf("MessageQ setup delegate:      %s\n", TRANSPORTSETUP);
    System_printf("Number of processors:         %d\n", numCores);
    System_printf("Number of messages received:  %d\n", latencyStats.numVals);
    System_printf("Build profile:                %s\n\n", BUILDPROFILE);

    System_printf("======== MESSAGEQ BENCHMARK RESULTS ======== \n");    
    System_printf("Average 1-way latency:        %10d (cycles/msg)           %10d (ns/msg)\n", 
        (UInt32)latencyStats.mean, CYCLES_TO_NS(latencyStats.mean, cpuFreq.lo));
    System_printf("Maximum 1-way latency:        %10d (cycles/msg) (#%5d)  %10d (ns/msg)\n", 
        latencyStats.max, latencyStats.maxIndex, CYCLES_TO_NS(latencyStats.max, cpuFreq.lo));
    System_printf("Minimum 1-way latency:        %10d (cycles/msg) (#%5d)  %10d (ns/msg)\n", 
        latencyStats.min, latencyStats.minIndex, CYCLES_TO_NS(latencyStats.min, cpuFreq.lo)); 
    System_printf("Standard deviation:           %10d (cycles/msg)\n", 
        (UInt32)latencyStats.stddev);
    System_printf("Total time elapsed:           %10d (cycles)     %10d (us)\n",
        timeElapsed, CYCLES_TO_US(timeElapsed, cpuFreq.lo));
}

/**
 *  @b Description
 *  @n  
 *      This function initalizes the platform.  It has called at startup.  This is defined in the
 *      .cfg file via the Startup.firstFxns.$add('&initPlatform'); definition.
 */
void initPlatform(void)
{
  platform_init_flags  pFormFlags;
  platform_init_config pFormConfig;
  /* Status of the call to initialize the platform */
  UInt32 pFormStatus;

  /* Only run on single core */
  if (CSL_chipReadReg (CSL_CHIP_DNUM) == 0)
  {
    /*
     * You can choose what to initialize on the platform by setting the following
     * flags. Things like the DDR, PLL, etc should have been set by the boot loader.
    */
    memset( (void *) &pFormFlags,  0, sizeof(platform_init_flags));
    memset( (void *) &pFormConfig, 0, sizeof(platform_init_config));

    pFormFlags.pll = 0; /* PLLs for clocking  	*/
    pFormFlags.ddr  = 0; /* External memory 		*/
    pFormFlags.tcsl = 1; /* Time stamp counter 	*/
    pFormFlags.phy  = 0; /* Ethernet 			*/
    pFormFlags.ecc  = 0; /* Memory ECC 			*/

    pFormConfig.pllm = 0;	/* Use libraries default clock divisor */
    pFormStatus = platform_init(&pFormFlags, &pFormConfig);

    /* If we initialized the platform okay */
    if (pFormStatus != Platform_EOK)
    {
  	 /* Initialization of the platform failed. */
  	 System_printf("Platform failed to initialize. Error code %d \n", pFormStatus);
    }
  }
}

/**
 *  @b Description
 *  @n  
 *      This functions measures latency by sending a message from core0 to core1. 
 *      Core1 relays all received messages back to core 2.  Core0 will measure the roundtrip latency.
 */
static void measure_latency()
{
    Int              status;
    UInt numReceived;
    MessageQ_Msg     msg;

    System_printf("tsk0. selfproc=%d nextQueueName (%s) openned, nextQueueId=%d\n", CSL_chipReadReg (CSL_CHIP_DNUM), nextQueueName, nextQueueId);

    if (selfId == 0) {
        msg = MessageQ_alloc(HEAP_ID, MESSAGE_SIZE_IN_BYTES);
        if (msg == NULL) {
           System_abort("MessageQ_alloc failed\n");
        }

        System_printf("tsk0. selfProc=%d calling MessageQ_put(nextQueueName=%s). msg=0x%x\n", CSL_chipReadReg (CSL_CHIP_DNUM), nextQueueName, msg);
        /* Kick off the loop */
        status = MessageQ_put(nextQueueId, msg);
        if (status < 0) {
            System_abort("MessageQ_put failed\n");
        }
    }

    for (numReceived = 0; numReceived < NUMLOOPS; numReceived++) {
    //while (1) {
        /* Get a message */
        status = MessageQ_get(messageQ, &msg, MessageQ_FOREVER);
        if (status < 0) {
            System_abort("MessageQ_get failed\n");
        }

        if (selfId == 0) {
            rawtimestamps[numReceived] = Timestamp_get32();

            if (numReceived == NUMLOOPS - 1) {
                printStatistics();

                // free the Message.
                MessageQ_free(msg);
                break;
            }
        }

        status = MessageQ_put(nextQueueId, msg);
        if (status < 0) {
            System_abort("MessageQ_put failed\n");
        }
    }
}

/**
 *  @b Description
 *  @n  
 *      This functions allocate all messages to be sent up front on core0.  Synchronize core 0 and core1.
 *      Core 0 sends all messages to Core1 in a burst.  Core 1 receives all the messages 
 *      and measure the throughput over the time it took to send all the messages.
 */
static void thruputTxRxPairPreallocFullLoad(void)
{ 
  /* Source to be executed on Core 0 */
  if (selfId == 0)
  {
    Int16 receiveCores[2] = {1, CORE_SYNC_NULL_CORE};  /* Last entry must be CORE_SYNC_NULL_CORE */
    UInt32 numTxMsgs =  (NUM_MSGS_TO_PREALLOC-1); /* First message used to synchronize cores */
    Int status;
#if VERBOSE_MODE    
    UInt32 numSends = 0;
#endif

    System_printf("\nThroughput via upfront allocation: Allocate all messages up front, sync cores, send all messages from core 0 to core 1\n");

    status = allocateMessages(NUM_MSGS_TO_PREALLOC, HEAP_ID, &txMsgPtrs[0]);
    if (status != 0)
    {
      System_printf("Message Preallocation failed for core %d after allocating %d messages.\n", selfId, status);
      detachAll(MultiProc_getNumProcessors());
      System_exit(0);
    }

    /* simplified for now since only one core */
    syncSendCore (&receiveCores[0], messageQ, &nextQueueId, &txMsgPtrs[0], TRUE);

    /* Send all messages to core 1.  The last message sent will have a flag signifying to core 1 that
      * all messages have been sent.  Start with second message in the txMsgPtrs array because the 
      * first was used (and freed by the transport) for the syncSendCore. */
#if VERBOSE_MODE
    numSends = sendMessages(1, &numTxMsgs, &nextQueueId, &txMsgPtrs[1]);
    System_printf ("Core %d: Sent a total of %d messages.\n", selfId, numSends);
#else
    sendMessages(1, &numTxMsgs, &nextQueueId, &txMsgPtrs[1]);
#endif
  }

  /* Source to be executed on Core 1 */
  if (selfId == 1)
  {
    Int16 sendCores[2] = {0, CORE_SYNC_NULL_CORE};  /* Last entry must be CORE_SYNC_NULL_CORE */
    UInt32 numReceived = 0;
    UInt32 delay = 0;
    Int status;
    UInt64 timeStamp;

#if VERBOSE_MODE
    System_printf("Core %d: Per message work delay is %dus\n", selfId, delay);
#endif

    /* Synchronize cores prior to starting test */
    syncReceiveCore (&sendCores[0], messageQ, &nextQueueId);

    /* Take time at start of test */
    timeStamp = getStartTime64();

    numReceived = receiveMessages(&sendCores[0], messageQ, &rxMsgPtrs[0], delay);

    /* Get execution time to transfer all messages */
    timeLength = getExecutionTime64(timeStamp, timeAdj);

#if VERBOSE_MODE
    System_printf("Core %d: Received a total of %d messages.\n", selfId, numReceived);
#endif

    /* Calculate throughput over all messages */
    calculateThroughput (numReceived, timeLength, cpuFreq);

    /* Free the messages received */
    status = freeMessages(numReceived, &rxMsgPtrs[0]);
    if (status < 0)
    {
      System_printf("Message free failed for Core %d\n", selfId);
    }
  }
}

/**
 *  @b Description
 *  @n  
 *      This configures the descriptor region and initializes CPPI, QMSS, and SRIO.
 *      This function should only be called once per chip.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
Int32 systemInit (Void)
{
  Cppi_InitCfg cppiHeapInit;  /* Static CPPI heap */
  Qmss_InitCfg qmssInitConfig;   /* QMSS configuration */
  Qmss_MemRegInfo memInfo; /* Memory region configuration information */
  Qmss_Result result;
  UInt32 coreNum;
  
  coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

  System_printf ("\n-----------------------Initializing---------------------------\n");
  
  System_printf ("Core %d : L1D cache size %d. L2 cache size %d.\n", coreNum, CACHE_getL1DSize(), CACHE_getL2Size());

  memset ((Void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));
  
  /* Set up the linking RAM. Use the internal Linking RAM. 
   * LLD will configure the internal linking RAM address and maximum internal linking RAM size if 
   * a value of zero is specified.
   * Linking RAM1 is not used */
  qmssInitConfig.linkingRAM0Base = 0;
  qmssInitConfig.linkingRAM0Size = 0;
  qmssInitConfig.linkingRAM1Base = 0;
  qmssInitConfig.maxDescNum      = NUM_HOST_DESC /* total of other descriptors here */;


#ifdef xdc_target__bigEndian
  qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
  qmssInitConfig.pdspFirmware[0].firmware = (void *) &acc48_be;
  qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_be);
#else
  qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
  qmssInitConfig.pdspFirmware[0].firmware = (void *) &acc48_le;
  qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_le);
#endif

  /* Initialize Queue Manager SubSystem */
  result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
  if (result != QMSS_SOK)
  {
      System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", coreNum, result);
      return -1;
  }

  /* Start the QMSS. */
  if (Qmss_start() != QMSS_SOK)
  {
      System_printf ("Error: Unable to start the QMSS\n");
      return -1;
  }

  cppiHeapInit.heapParams.staticHeapBase = &cppiHeap[0];
  cppiHeapInit.heapParams.staticHeapSize = SIZE_CPPI_HEAP;
  cppiHeapInit.heapParams.heapAlignPow2 = 7; /* Power of 7 (128 byte) */
  cppiHeapInit.heapParams.dynamicHeapBlockSize = -1; /* Shut off malloc if block runs out */
  result = Cppi_initCfg (&cppiGblCfgParams, &cppiHeapInit);
  if (result != CPPI_SOK)
  {
      System_printf ("Error Core %d : Initializing CPPI LLD error code : %d\n", coreNum, result);
  }

  System_printf ("address of hostDesc[] = 0x%x. Converted=0x%x\n", hostDesc, l2_global_address ((UInt32) hostDesc));

  /* Setup memory region for host descriptors */
  memset ((Void *) &hostDesc, 0, SIZE_HOST_DESC * NUM_HOST_DESC);
  memInfo.descBase       = (UInt32 *) hostDesc;	/* descriptor pool is in MSMC */
  memInfo.descSize       = SIZE_HOST_DESC;
  memInfo.descNum        = NUM_HOST_DESC;
  memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
  memInfo.memRegion      = (Qmss_MemRegion) descriptorMemRegion;
  memInfo.startIndex     = 0;

  result = Qmss_insertMemoryRegion (&memInfo);
  if (result < QMSS_SOK)
  {
      System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", coreNum, memInfo.memRegion, result);
      return -1;
  }
  else
  {
      System_printf ("Core %d : Memory region %d inserted\n", coreNum, result);
  }

  /* Writeback the descriptor pool.  Writeback all data cache.
    * Wait until operation is complete. */    
  Cache_wb (hostDesc, 
                     SIZE_HOST_DESC * NUM_HOST_DESC,
                     Cache_Type_ALLD, TRUE);
  
  return 0;
}

/**
 *  @b Description
 *  @n  
 *      Task which kicks off the latency and throughput tests
 */
Void tsk0(UArg arg0, UArg arg1)
{
    Int status;
    HeapBufMP_Handle              heapHandle;
    HeapBufMP_Params              heapBufParams;

    System_printf("tsk0 starting\n");

    if (selfId == 0)
    {      
      /* Create the heap that will be used to allocate messages. */     
      HeapBufMP_Params_init(&heapBufParams);
      heapBufParams.regionId       = 0;
      heapBufParams.name           = HEAP_NAME;
      heapBufParams.numBlocks      = NUM_HOST_DESC;
      heapBufParams.blockSize      = SRIO_MTU_SIZE;
      heapHandle = HeapBufMP_create(&heapBufParams);
      if (heapHandle == NULL) 
      {
        System_abort("HeapBufMP_create failed\n" );
      }
    }
    else
    {
      /* Open the heap created by the other processor. Loop until opened. */
      do 
      {
        status = HeapBufMP_open(HEAP_NAME, &heapHandle);
      } while (status < 0);
    }

    /* Register this heap with MessageQ */
    MessageQ_registerHeap((IHeap_Handle)heapHandle, HEAP_ID);

    /* Open the 'next' remote message queue. Spin until it is ready. */
    do {
        status = MessageQ_open(nextQueueName, &nextQueueId);
    }
    while (status < 0);

    measure_latency();

    thruputTxRxPairPreallocFullLoad();

    detachAll(MultiProc_getNumProcessors());
    System_exit(0);
}

/**
 *  @b Description
 *  @n  
 *      Main - Initialize the system and start BIOS
 */
Int main(Int argc, Char* argv[])
{
  Int32 result = 0;
  Types_Timestamp64 time64;
  UInt64 timeStamp;

  Timestamp_getFreq(&timerFreq);
  System_printf("timerFreq.lo = %d. timerFreq.hi = %d\n", timerFreq.lo, timerFreq.hi);

  BIOS_getCpuFreq(&cpuFreq);
  System_printf("cpuFreq.lo = %d. cpuFreq.hi = %d\n", cpuFreq.lo, cpuFreq.hi);
  
  cpuTimerFreqRatio = (Float)cpuFreq.lo / (Float)timerFreq.lo;

  Timestamp_get64(&time64);
  timeStamp = TIMESTAMP64_TO_UINT64(time64.hi,time64.lo);
  timeAdj = TIMESTAMP64_TO_UINT64(time64.hi,time64.lo) - timeStamp;

  selfId = CSL_chipReadReg (CSL_CHIP_DNUM);
  
  System_printf("Core (\"%s\") starting\n", MultiProc_getName(selfId));
  
  if (numCores == 0) {
      numCores = MultiProc_getNumProcessors();
  }

  /* System initializations for each core. */
  if (selfId == 0) 
  {
    /* SRIO, QMSS, and CPPI system wide initializations are run on
      * this core */
  	 result = systemInit();
  	 if (result != 0)
    {
      System_printf("Error (%d) while initializing QMSS\n", result);
      return (0);
  	 }

    /* Power on SRIO peripheral before using it */
    if (enable_srio () < 0)
    {
        System_printf ("Error: SRIO PSC Initialization Failed\n");
        return (0);
    }
    
    /* Device Specific SRIO Initializations: This should always be called before
      * initializing the SRIO Driver. */
	if (SrioDevice_init() < 0)
    	return (0);        

    /* Initialize the SRIO Driver */
    if (Srio_init () < 0)
    {
        System_printf ("Error: SRIO Driver Initialization Failed\n");
        return (0);
    }

	/* SRIO Driver is operational at this time. */
    System_printf ("Debug(Core %d): SRIO Driver has been initialized\n", selfId);

    /* Write to the SHARED memory location at this point in time. The other cores cannot execute
      * till the SRIO Driver is up and running. */
    isSRIOInitialized = 1;

    /* The SRIO IP block has been initialized. We need to writeback the cache here because it will
      * ensure that the rest of the cores which are waiting for SRIO to be initialized would now be
      * woken up. */
    Cache_wb ((void *) &isSRIOInitialized, 4, Cache_Type_L1D, TRUE);
  }
  else
  {
    /* System initialization performed on another core.  Only need to start QMSS for this core */
    System_printf ("Debug(Core %d): Waiting for SRIO to be initialized.\n", selfId);
  
    /* All other cores loop around forever till the SRIO is up and running. 
     * We need to invalidate the cache so that we always read this from the memory. */
    while (isSRIOInitialized == 0)
        Cache_inv ((void *) &isSRIOInitialized, 4, Cache_Type_L1D, TRUE);
  
    /* Start the QMSS. */
    if (Qmss_start() != QMSS_SOK)
    {
        System_printf ("Error: Unable to start the QMSS\n");
        return (0);
    }
  
    System_printf ("Debug(Core %d): SRIO can now be used.\n", selfId);
  }

  /* Each core must initialize memory used for the packet descriptor buffers within SRIO */
  if (Osal_dataBufferInitMemory(SRIO_MTU_SIZE) < 0)
  {
    System_printf("Core: %d ERROR: SRIO memory initialization failed\n", selfId);
    return (0);
  }

  /* calling IPC attach. After this attach, QMSS is started and also the descriptor pool is also init'ed */
  attachAll(numCores);
     
  prevCoreId = (selfId - 1 + numCores) % numCores;    
  
  System_sprintf(localQueueName, "CORE%d", selfId);
  System_sprintf(nextQueueName, "CORE%d", 
      ((selfId + 1) % numCores));
  System_sprintf(prevQueueName, "CORE%d", prevCoreId);

  System_printf("localQueueName=%s. nextQueueName=%s. prevQueueName=%s\n", 
                  localQueueName,  nextQueueName, prevQueueName);
        
  /* Create a message queue. */
  messageQ = MessageQ_create(localQueueName, NULL);    
  if (messageQ == NULL) {
      System_abort("MessageQ_create failed\n" );
  }

  BIOS_start();

  System_printf("done BIOS_start\n", result);

  return (0);
}

