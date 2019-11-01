/* --COPYRIGHT--,BSD
 * Copyright (c) 2011, Texas Instruments Incorporated
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
#include <xdc/runtime/Error.h>

/* IPC module Headers */
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/sdo/ipc/nsremote/NameServerMessageQ.h>
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

#include <ti/transport/ipc/srio/transports/TransportSrio.h>

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

#define HEAP_NAME   "consumerHeap"
#define HEAP_ID         appMsgQHeapId

/* Local core variables */
UInt16 selfId;
UInt16 multiProcId;
Char localQueueName[6];
MessageQ_Handle messageQ = NULL;
NameServerMessageQ_Handle nsHandle = NULL;
TransportSrio_Handle srioHandle = NULL;

/* These are the device identifiers used in the test Application */
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

/* Descriptor pool [Size of descriptor * Number of descriptors]
  * place this host descritor pool in shared memory */
#pragma DATA_SECTION (hostDesc, ".desc");
#pragma DATA_ALIGN (hostDesc, 16)
UInt8               hostDesc[SIZE_HOST_DESC * NUM_HOST_DESC];

/* Statically created shared heap for CPPI since IPC does create a
 * shared heap for SharedRegion prior to Ipc_attach */
#pragma DATA_SECTION (cppiHeap, ".cppi_heap");
#pragma DATA_ALIGN (cppiHeap, 128)
UInt8               cppiHeap[SIZE_CPPI_HEAP];

/* Placed in MSMC */
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
  
  coreNum = multiProcId;

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
      System_printf ("Error Core %d: Initializing Queue Manager SubSystem error code : %d\n", coreNum, result);
      return -1;
  }

  /* Start the QMSS. */
  if (Qmss_start() != QMSS_SOK)
  {
      System_printf ("Error Core %d: Unable to start the QMSS\n", coreNum);
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

  /* Setup memory region for host descriptors */
  memset ((Void *) &hostDesc, 0, SIZE_HOST_DESC * NUM_HOST_DESC);
  memInfo.descBase       = (UInt32 *) hostDesc;	/* Descriptors are in MSMC */
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
 *      This function enables the power/clock domains for SRIO. 
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static Int32 enableSrio (void)
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
 *      This functions receives msgs from a remote core located on another chip
 *
 *  @param[in]  queueId
 *      QueueId to send msgs on
 *
 *  @param[in]  remoteCoreOffset
 *      Offset into global core array defined in .cfg defining which
 *      chip the msgs should be sent to.
 */
static void receiveOffChipPackets(void)
{
  Int              status;
  MessageQ_Msg     msg;
  UInt count = 0;

  System_printf("Global Core %d: Receiving packets from an off-chip core.\n", multiProcId);

  /* Receive four packets */
  while (count != 4)
  {
    status = MessageQ_get(messageQ, &msg, MessageQ_FOREVER);
    if (status < 0) 
    {
        System_abort("MessageQ_put failed\n");
    }

    /* Free the message. */
    MessageQ_free(msg);
    count++;
  }
  System_printf("Core %d: received %d messages from core %d\n",
                       multiProcId, count, msg->srcProc);
  System_printf("Chip to chip message transfer passed!\n");
}

/**
 *  @b Description
 *  @n  
 *      Producer task used to send packets off-chip from one local core
 */
Void tsk0(UArg arg0, UArg arg1)
{
  Int32 status;
  HeapBufMP_Handle              heapHandle;
  HeapBufMP_Params              heapBufParams;
  
  System_printf("Core %d: tsk0 starting\n", multiProcId);

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

  /* Each core must register this heap with MessageQ */
  MessageQ_registerHeap((IHeap_Handle)heapHandle, HEAP_ID);

  /* Receive packets from remote chip.  Assume two cores per chip.  */
  receiveOffChipPackets();

  /* Finished, cleanup and detach */
  TransportSrio_delete (&srioHandle);
  NameServerMessageQ_delete(&nsHandle);
  detachAll(MultiProc_getNumProcsInCluster());
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
  Error_Block eb;

  selfId = CSL_chipReadReg (CSL_CHIP_DNUM);
  multiProcId = MultiProc_self();

  System_printf("Local Core (\"%s\") starting\n", MultiProc_getName(multiProcId));
  System_printf("Local Core ID: %d\n", selfId);
  System_printf("Global Core ID: %d\n", multiProcId);

  /* System initializations for each core. */
  if (selfId == 0) 
  {
    /* SRIO, QMSS, and CPPI system wide initializations are run on
      * this core */
    result = systemInit();
    if (result != 0)
    {
      System_printf("Core: %d ERROR: while initializing QMSS\n", multiProcId, result);
      return (0);
    }

    /* Power on SRIO peripheral before using it */
    if (enableSrio () < 0)
    {
      System_printf ("Core: %d ERROR: SRIO PSC Initialization Failed\n", multiProcId);
      return (0);
    }

    /* Device Specific SRIO Initializations: This should always be called before
      * initializing the SRIO Driver. */
    if (SrioDevice_init() < 0)
    {
    	return (0);        
    }

    /* Initialize the SRIO Driver */
    if (Srio_init () < 0)
    {
      System_printf ("Core: %d ERROR: SRIO Driver Initialization Failed\n", multiProcId);
      return (0);
    }

    /* SRIO Driver is operational at this time. */
    System_printf ("Core %d: SRIO Driver has been initialized\n", multiProcId);

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
    
    System_printf ("Core %d: Waiting for SRIO to be initialized.\n", multiProcId);
  
    /* All other cores loop forever until SRIO is up and running. 
      * We need to invalidate the cache so that we always read this from the memory. */
    while (isSRIOInitialized == 0)
    {
      Cache_inv ((void *) &isSRIOInitialized, 4, Cache_Type_L1D, TRUE);
    }
  
    /* Start the QMSS. */
    if (Qmss_start() != QMSS_SOK)
    {
      System_printf ("Core: %d ERROR: Unable to start the QMSS\n", multiProcId);
      return (0);
    }
  
    System_printf ("Core %d: SRIO can now be used.\n", multiProcId);
  }

  /* Attach all local cores to one another.  IPC does not allow attaching to cores on remote devices */
  attachAll(MultiProc_getNumProcsInCluster());

  /* Create messageQ to remote proc .  This will use srioTransport to send/receive nameserver
   * messages to/from remote chip.  A MessageQ heap must be registered prior to calling 
   * NameServerMessageQ_create()*/
  Error_init(&eb);    
  nsHandle = NameServerMessageQ_create(multiProcId-MultiProc_getBaseIdOfCluster(), NULL, &eb);
  if (nsHandle == NULL) 
  {
    System_abort("NameServerMessageQ_create() failed");
  }
  /* Register a transport for messages received from off-chip cores */
  Error_init(&eb);
  srioHandle = TransportSrio_create(multiProcId-MultiProc_getBaseIdOfCluster(), NULL, &eb);

  /* Create a queue to receive messages */
  System_sprintf(localQueueName, "CORE%d", multiProcId);
  System_printf("localQueueName=%s\n", localQueueName);
        
  /* Create the message queue. */
  messageQ = MessageQ_create(localQueueName, NULL);    
  if (messageQ == NULL) {
      System_abort("ERROR: MessageQ_create failed\n");
  }

  /* Start BIOS and all defined tasks.  Function will not return since it acts as the scheduler. */
  BIOS_start();

  /* should not reach here */
  return (0);
}

