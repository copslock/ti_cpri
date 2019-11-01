/* 
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
 * */
/*
 *  ======== TransportQmss.c ========
 */

/* XDC includes */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>

/* BIOS includes */
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c66/tci66xx/CpIntc.h> 
#include <ti/sysbios/family/c66/Cache.h>

/* IPC external includes */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>

/* IPC internal includes */
#include <ti/sdo/ipc/_MessageQ.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>

/* CSL Includes */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* Intrinsic Includes */
#include <c6x.h>

/* Transport Includes */
#include "package/internal/TransportQmss.xdc.h"

#define ENABLE_PUT_ISR_ASSERTS  1
#define USE_CSL_CACHE_APIS 0 /* If enabled this will reduce cycle counts within the _put and _isr functions */

/* Event ID is chosen based on interrupt mapping in http://focus.ti.com/lit/ug/sprugr9/sprugr9.pdf 
  * One QMSS driver, using one high priority QMSS channel, is created per core.  Core 0 through 7 will use
  * high priority queues 704-711.  These all map to Event ID 48 */
#define QMSS_HIGH_PRIO_INT0_EVENT_ID (48)
#define QMSS_QUEUES_PER_EVENTID (8) /* On c6678 there are eight queues that map to the event ID.  one
                                                                    * for each core */

#define MONOLITHIC_DESC_DATA_OFFSET 16
#define CACHE_TYPE					Cache_Type_L1D

#define QMSS_INTC_QUEUE_COL 0
#define QMSS_INTC0_SYSTEM_EVENT_COL 1
#define QMSS_INTC1_SYSTEM_EVENT_COL 2
/* Number of queue allocations it will take before getting to QPEND queues usable by cores 4-7 */
#define QMSS_MAX_NUM_UNUSED_QUEUES 6

/* CPINTC0 QPEND queue to CPINTC0 system event mapping.  The mapping can be found
  * in section 5.3 Interrupt Maps of SPRUGR9 - Keystone Architecture Multicore Navigator User Guide.
  * CPINTC0, used for GEM cores 0 - 3, can be used for queues 652 through 665.  CPINTC1, used for
  * GEM cores 4 - 7, can be used for queues 658 through 671. */
const UInt cpIntc_QToEvtMap [QMSS_MAX_INTC_QUEUE][3] = {{652, 47, 0}, {653, 91, 0},
                                                                                                {654, 93, 0}, {655, 95, 0},
                                                                                                {656, 97, 0}, {657, 151, 0},
                                                                                                {658, 152, 47}, {659, 153, 91},
                                                                                                {660, 154, 93}, {661, 155, 95},
                                                                                                {662, 156, 97}, {663, 157, 151},
                                                                                                {664, 158, 152}, {665, 159, 153},                                                                         
                                                                                                {666, 0, 154}, {667, 0, 155},
                                                                                                {668, 0, 156}, {669, 0, 157},
                                                                                                {670, 0, 158}, {671, 0, 159},                                                                         
                                                                                               };

/* Mappings of GEM event to Host interrupts.  See section 7.5.1 of 
  * SPRS691 - TMS320C6678 Data Manual for more information. */
#define GEM_EVT_21_TO_HOST_INT(n) (32 + 0 + (11 * n))
#define GEM_EVT_22_TO_HOST_INT(n) (32 + 1 + (11 * n))
#define GEM_EVT_23_TO_HOST_INT(n) (32 + 2 + (11 * n))
#define GEM_EVT_24_TO_HOST_INT(n) (32 + 3 + (11 * n))
#define GEM_EVT_25_TO_HOST_INT(n) (32 + 4 + (11 * n))
#define GEM_EVT_26_TO_HOST_INT(n) (32 + 5 + (11 * n))
#define GEM_EVT_27_TO_HOST_INT(n) (32 + 6 + (11 * n))
#define GEM_EVT_28_TO_HOST_INT(n) (32 + 7 + (11 * n))
#define GEM_EVT_29_TO_HOST_INT(n) (32 + 8 + (11 * n))
#define GEM_EVT_30_TO_HOST_INT(n) (32 + 9 + (11 * n))
#define GEM_EVT_31_TO_HOST_INT(n) (32 + 10 + (11 * n))
#define GEM_EVT_62_TO_HOST_INT(n) (2 + (8 * n))
#define GEM_EVT_63_TO_HOST_INT(n) (3 + (8 * n))
#define GEM_EVT_92_TO_HOST_INT(n) (4 + (8 * n))
#define GEM_EVT_93_TO_HOST_INT(n) (5 + (8 * n))
#define GEM_EVT_94_TO_HOST_INT(n) (6 + (8 * n))
#define GEM_EVT_95_TO_HOST_INT(n) (7 + (8 * n))
#define GEM_EVT_102_TO_HOST_INT(n) (0)
#define GEM_EVT_103_TO_HOST_INT(n) (1)
#define GEM_EVT_104_TO_HOST_INT(n) (8)
#define GEM_EVT_105_TO_HOST_INT(n) (9)
#define GEM_EVT_106_TO_HOST_INT(n) (16)
#define GEM_EVT_107_TO_HOST_INT(n) (17)
#define GEM_EVT_108_TO_HOST_INT(n) (24)
#define GEM_EVT_109_TO_HOST_INT(n) (25)

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
  UInt32 coreNum;

  if (addr & 0xF0000000)
  {
    /* Address is already global, return */
    return (addr);
  }

  /* Get the core number. */
  coreNum = CSL_chipReadReg (CSL_CHIP_DNUM); 

  /* Compute the global address. */
  return (addr + (0x10000000 + (coreNum * 0x1000000)));
}

static UInt32 global_l2_address (UInt32 addr) 
{ 
  /* Convert only if address is global */
  if (addr & 0xF0000000)
  {
    return (addr - (0x10000000 + (CSL_chipReadReg (CSL_CHIP_DNUM) * 0x1000000)));
  }
  else
  {
    return (addr);
  }
}

Int getIntcHostInt (Int32 coreNum, UInt gemEvent)
{
  Int32 coreMultiplier = ((coreNum < 4) ? coreNum : (coreNum - 4));
  
  /* Translate selected GEM event to INTC host interrupt */
  switch (gemEvent)
  {
    case 21:
      return (GEM_EVT_21_TO_HOST_INT (coreMultiplier));
    case 22:
      return (GEM_EVT_22_TO_HOST_INT (coreMultiplier));
    case 23:
      return (GEM_EVT_23_TO_HOST_INT (coreMultiplier));
    case 24:
      return (GEM_EVT_24_TO_HOST_INT (coreMultiplier)); 
    case 25:
      return (GEM_EVT_25_TO_HOST_INT (coreMultiplier));
    case 26:
      return (GEM_EVT_26_TO_HOST_INT (coreMultiplier));
    case 27:
      return (GEM_EVT_27_TO_HOST_INT (coreMultiplier));
    case 28:
      return (GEM_EVT_28_TO_HOST_INT (coreMultiplier));
    case 29:
      return (GEM_EVT_29_TO_HOST_INT (coreMultiplier));
    case 30:
      return (GEM_EVT_30_TO_HOST_INT (coreMultiplier));
    case 31:
      return (GEM_EVT_31_TO_HOST_INT (coreMultiplier));
    case 62:
      return (GEM_EVT_62_TO_HOST_INT (coreMultiplier));
    case 63:
      return (GEM_EVT_63_TO_HOST_INT (coreMultiplier));
    case 92:
      return (GEM_EVT_92_TO_HOST_INT (coreMultiplier));
    case 93:
      return (GEM_EVT_93_TO_HOST_INT (coreMultiplier));
    case 94:
      return (GEM_EVT_94_TO_HOST_INT (coreMultiplier));
    case 95:
      return (GEM_EVT_95_TO_HOST_INT (coreMultiplier));
    case 102:
      return (GEM_EVT_102_TO_HOST_INT (coreMultiplier));
    case 103:
      return (GEM_EVT_103_TO_HOST_INT (coreMultiplier));
    case 104:
      return (GEM_EVT_104_TO_HOST_INT (coreMultiplier));
    case 105:
      return (GEM_EVT_105_TO_HOST_INT (coreMultiplier));
    case 106:
      return (GEM_EVT_106_TO_HOST_INT (coreMultiplier));
    case 107:
      return (GEM_EVT_107_TO_HOST_INT (coreMultiplier));
    case 108:
      return (GEM_EVT_108_TO_HOST_INT (coreMultiplier));
    case 109:
      return (GEM_EVT_109_TO_HOST_INT (coreMultiplier));
    default:
      /* No mapping, return -1 for error handling */
      return (-1);
  }
}

/*
 *************************************************************************
 *                       Module functions
 *************************************************************************
 */




/*
 *************************************************************************
 *                       Instance functions
 *************************************************************************
 */
 
/*
 *  ======== TransportQmss_Instance_init ========
 */
Int TransportQmss_Instance_init(TransportQmss_Object *obj,
        UInt16 procId, const TransportQmss_Params *params,
        Error_Block *eb)
{
  Int                 i;
  Int32              coreNum;
  SharedRegion_Entry  entry;
  Qmss_Result  result;
  Cppi_DescCfg descCfg;
  Qmss_QueueHnd freeQueueHdlr, rxQueueHdlr;
  UInt32 numAllocated; 
  UInt16 channel;
  UInt8           isAllocated;
  Qmss_AccCmdCfg      cfg;
  Qmss_Queue          queInfo;
  Hwi_Params          hwiAttrs;
  UInt16 queue;
  Int16 isrEventId;
  Bool flag;
  UInt32 hwiKey;

  /* Get this core's number */
  coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
  
  /* Check whether remote proc ID has been set and isn't the same as the
    * local proc ID */
  Assert_isTrue ((params->remoteProcId != MultiProc_INVALIDID) &&
                           (params->remoteProcId != MultiProc_self()),
                           TransportQmss_A_qmssProcIdInvalid);

  /* Transfer transport initialization parameters to transport object.  Object
    * will be used when registering interrupt and registering transport after 
    * all configurations complete.  Every call to transport Instance_init 
    * should result in a unique object */
  obj->priority      = params->priority;
  obj->remoteProcId  = procId;
  obj->intVectId = params->intVectorId;
  /* save cache option */
  obj->cacheEnabled = params->cacheEnabled;
  obj->memRegion = params->descMemRegion;

  /* Get descriptor region information */
  SharedRegion_getEntry(params->descMemRegion, &entry);  

  /* Disable interrupts prior to checking and incrementing qmssInitialized to
    * avoid any conflicts with this code being executed in another task */
  hwiKey = Hwi_disable();
  
  /* Configure and start the QMSS queues - One receive queue created per core.
    * A single free queue will be created for all cores */
  /* START: Once per core configuration */
  if (TransportQmss_module->qmssInitialized == 0) 
  {
    /* Increment module variable each time this core attaches to another.  
      * Will be used to track when to close this core's socket. */
    TransportQmss_module->qmssInitialized++;
    /* Restore interrupts */
    Hwi_restore(hwiKey);

    /* Start QMSS once on each core */
    result = Qmss_start ();
    Assert_isTrue ((result == QMSS_SOK), TransportQmss_A_qmssError);   

    /* Allocate space to store each core's receive queue ID and the single 
      * free queue ID */    
    if (TransportQmss_module->rxQueueId == NULL) 
    {           
      TransportQmss_module->rxQueueId = Memory_alloc(0,
          sizeof(UInt32) * (ti_sdo_utils_MultiProc_numProcessors+1), 0, eb);
    }

    if (!TransportQmss_useAccumulatorLogic)
    {
      /* If using QPEND queues, allocate space to store each core's INTC system interrupt. */    
      if (TransportQmss_module->cpIntSystemInt == NULL) 
      {           
        TransportQmss_module->cpIntSystemInt = Memory_alloc(0,
            sizeof(UInt32) * ti_sdo_utils_MultiProc_numProcessors, 0, eb);
      }
    }

    /* Only setup queue's on a single core.  The information will be passed to the
      * the rest of the core's through IPC */
    if (MultiProc_self() == entry.ownerProcId) 
    {    
      /* Setup the descriptors for the receive free queue */
      descCfg.memRegion           = (Qmss_MemRegion) params->descMemRegion;
      descCfg.descNum             = TransportQmss_numDescriptors;
      descCfg.destQueueNum        = QMSS_PARAM_NOT_SPECIFIED; 
      descCfg.queueType           = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
      descCfg.initDesc            = Cppi_InitDesc_INIT_DESCRIPTOR;
      descCfg.descType            = Cppi_DescType_MONOLITHIC;
      descCfg.epibPresent         = Cppi_EPIB_NO_EPIB_PRESENT;
      descCfg.cfg.mono.dataOffset = MONOLITHIC_DESC_DATA_OFFSET;            
      descCfg.returnQueue.qMgr    = QMSS_PARAM_NOT_SPECIFIED;
      descCfg.returnQueue.qNum    = QMSS_PARAM_NOT_SPECIFIED;
      
      /* Initialize the descriptors and push to free Queue */
      freeQueueHdlr = Cppi_initDescriptor (&descCfg, &numAllocated);
      
      Assert_isTrue ((freeQueueHdlr > 0), TransportQmss_A_qmssError);               

      /* Open each core's receive queue and store the number in the global array */
      for (i = 0; i < (ti_sdo_utils_MultiProc_numProcessors); i++) 
      {            
        if (TransportQmss_useAccumulatorLogic)
        {
          /* Open an accumulator queue */
          rxQueueHdlr = Qmss_queueOpen (Qmss_QueueType_HIGH_PRIORITY_QUEUE, 
                                                               QMSS_PARAM_NOT_SPECIFIED, &isAllocated);   
        }
        else
        {
          Int intcQIndex;
          Int intcSysEvtIndex = (i < 4 ? QMSS_INTC0_SYSTEM_EVENT_COL : QMSS_INTC1_SYSTEM_EVENT_COL);
          Qmss_QueueHnd unusedQueues[QMSS_MAX_NUM_UNUSED_QUEUES];
          Int unusedQueuesIndex = 0;
          Bool intcQValid = FALSE;
          Int j;

          /* Open INTC queues until assigned queue with an INTC system event tied to it. */
          while (!intcQValid)
          {
            rxQueueHdlr = Qmss_queueOpen (Qmss_QueueType_INTC_QUEUE, 
                                                                 QMSS_PARAM_NOT_SPECIFIED, &isAllocated);

            intcQIndex = rxQueueHdlr - QMSS_INTC_QUEUE_BASE;

            /* Check if the assigned queue has a system event for the core opening the queue.  If not, close the queue. */
            if (cpIntc_QToEvtMap[intcQIndex][intcSysEvtIndex] == 0)
            {
              /* Store the unusable queue for closing after a valid queue has been found */
              unusedQueues[unusedQueuesIndex++] = rxQueueHdlr;

              /* Assert if the last available QPEND queue is not valid for the given core.  Essentially
                * no more queues are available for the transport. */
              Assert_isTrue ((intcQIndex < (QMSS_MAX_INTC_QUEUE - 1)), TransportQmss_A_qmssOutOfQpendQueues);
            }
            else
            {
              /* System event exists, use this queue */
              intcQValid = TRUE;
              /* Store system event in the transport module. */
              TransportQmss_module->cpIntSystemInt[i] = cpIntc_QToEvtMap[intcQIndex][intcSysEvtIndex];
            }
          }

          /* Found a valid queue for the core.  Clean up the queues that were allocated that were not valid for
            * the given core */
          for (j = 0; j < unusedQueuesIndex; j++)
          {
            Qmss_queueClose (unusedQueues[j]);
          }
        }

        Assert_isTrue ((rxQueueHdlr > 0), TransportQmss_A_qmssError);
    
        TransportQmss_module->rxQueueId[i] = rxQueueHdlr;
      }
            
      /* The last queue in the global array is the free queue. 
        * For example, there are 8 cores in the system.  There will be 9 queues. 
        * Queues 0 - 7 are the receive queues, queue 9 is the free queue.
        */             
      TransportQmss_module->rxQueueId[ti_sdo_utils_MultiProc_numProcessors] = freeQueueHdlr;

      Cache_wb(TransportQmss_module->rxQueueId, 
                       sizeof(UInt32) * (ti_sdo_utils_MultiProc_numProcessors+1), CACHE_TYPE, TRUE);

      /* Provide global queue array to other cores */
      Ipc_writeConfig(params->remoteProcId, 0x12345678,
                                 TransportQmss_module->rxQueueId,
                                 sizeof(UInt32) * (ti_sdo_utils_MultiProc_numProcessors + 1));

      if (!TransportQmss_useAccumulatorLogic)
      {
        Cache_wb(TransportQmss_module->cpIntSystemInt, 
                         sizeof(UInt32) * ti_sdo_utils_MultiProc_numProcessors, CACHE_TYPE, TRUE);
        
        /* Provide global system int array to other cores */
        Ipc_writeConfig(params->remoteProcId, 0x87654321,
                                   TransportQmss_module->cpIntSystemInt,
                                   sizeof(UInt32) * ti_sdo_utils_MultiProc_numProcessors);
      }
    }
    else
    {
      /* Get the queue information from the owner core.  Read queue info from shared 
        * memory to this core*/
      Ipc_readConfig(params->remoteProcId, 0x12345678,
                                TransportQmss_module->rxQueueId,
                                sizeof(UInt32) * (ti_sdo_utils_MultiProc_numProcessors + 1));
      
      /* readConfig will pull values into L2.  Need to write them back to memory */
      Cache_wb(TransportQmss_module->rxQueueId, 
                        sizeof(UInt32) * (ti_sdo_utils_MultiProc_numProcessors+1), CACHE_TYPE, TRUE);
      
      if (!TransportQmss_useAccumulatorLogic)
      {
        /* Get the system event information from the owner core.  Read system event info 
          * from shared memory to this core*/
        Ipc_readConfig(params->remoteProcId, 0x87654321,
                                  TransportQmss_module->cpIntSystemInt,
                                  sizeof(UInt32) * ti_sdo_utils_MultiProc_numProcessors);

        /* readConfig will pull values into L2.  Need to write them back to memory */
        Cache_wb(TransportQmss_module->cpIntSystemInt, 
                         sizeof(UInt32) * ti_sdo_utils_MultiProc_numProcessors, CACHE_TYPE, TRUE);
      }
    }

    /* Program accumulator if transport is to use it */
    if (TransportQmss_useAccumulatorLogic)
    {
      
      /* Initialize the accumulator list */ 
      if (TransportQmss_module->hiPrioList == NULL)
      {       
        Assert_isTrue ((TransportQmss_accuHiPriListSize >= ((TransportQmss_intThreshold * 2) + 2)), 
                                  TransportQmss_A_AccumulatorListSize); 
        
        /* Allocate memory for the accumulator list */    
        TransportQmss_module->hiPrioList = (UInt32*)l2_global_address(
                        (UInt32) Memory_alloc(0,
                        sizeof(UInt32)*(TransportQmss_accuHiPriListSize),
                        16, /* 16-byte aligned */
                        eb));
                                    
        Assert_isTrue ((TransportQmss_module->hiPrioList != NULL), 
                                  TransportQmss_A_qmssAccumulatorListNULL);
      }

      /* Program the accumulator - doing this once for each core */
      rxQueueHdlr = TransportQmss_module->rxQueueId[coreNum];
      
      /* It's this core's own channel, and not the channel of the remote core */
      channel = rxQueueHdlr - QMSS_HIGH_PRIORITY_QUEUE_BASE;
      
      /* Program the high priority accumulator */
      memset ((Void *)(TransportQmss_module->hiPrioList), 0, 
              sizeof(UInt32)*(TransportQmss_accuHiPriListSize));

      cfg.channel             = channel;
      cfg.command             = Qmss_AccCmd_ENABLE_CHANNEL;
      cfg.queueEnMask         = 0;    
      cfg.listAddress         = (UInt32)(TransportQmss_module->hiPrioList);
      
      /* Get queue manager and queue number from handle */
      queInfo                 = Qmss_getQueueNumber (rxQueueHdlr);
      cfg.queMgrIndex         = queInfo.qNum;
      cfg.maxPageEntries      = TransportQmss_intThreshold + 1;
      cfg.timerLoadCount      = TransportQmss_timerLoadCount;
      cfg.listEntrySize       = Qmss_AccEntrySize_REG_D;
      cfg.listCountMode       = Qmss_AccCountMode_ENTRY_COUNT;
      cfg.multiQueueMode      = Qmss_AccQueueMode_SINGLE_QUEUE;
      
      if (TransportQmss_pacingEnabled) 
      {
        cfg.interruptPacingMode = Qmss_AccPacingMode_LAST_INTERRUPT;
      } 
      else 
      {
        cfg.interruptPacingMode = Qmss_AccPacingMode_NONE;
      }
              
      result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg);
      Assert_isTrue ((result == QMSS_ACC_SOK), TransportQmss_A_qmssError);    
    }
    
    /* Register interrupt handler */
    if (TransportQmss_useAccumulatorLogic)
    {
      /* Register the accumulator based ISR with BIOS */
      isrEventId = QMSS_HIGH_PRIO_INT0_EVENT_ID;
      queue = rxQueueHdlr - QMSS_HIGH_PRIORITY_QUEUE_BASE;
      while (queue >= QMSS_QUEUES_PER_EVENTID)
      {
        isrEventId++;
        queue -= QMSS_QUEUES_PER_EVENTID;
      }
          
      Hwi_Params_init(&hwiAttrs);
      hwiAttrs.maskSetting = Hwi_MaskingOption_SELF;
      hwiAttrs.arg         = (UArg) obj;
      hwiAttrs.eventId     = isrEventId;
      Hwi_create(params->intVectorId, (Hwi_FuncPtr)TransportQmss_Accumulator_isr, &hwiAttrs, eb); 

      Hwi_enableInterrupt(params->intVectorId); 
    }
    else
    {
      /* Register INTC ISR with BIOS */
      UInt cpIntId = (coreNum < 4 ? 0 : 1);
      UInt hostInt = getIntcHostInt (coreNum, TransportQmss_intcGemEvent);

      /* Map the Queue's system int to GEM event via the CpIntc module */
      CpIntc_mapSysIntToHostInt(cpIntId, TransportQmss_module->cpIntSystemInt[coreNum],
                                                     hostInt);
      CpIntc_dispatchPlug(TransportQmss_module->cpIntSystemInt[coreNum], 
                                        (CpIntc_FuncPtr)TransportQmss_Qpend_isr, (UArg) obj, 1);
      CpIntc_enableHostInt(cpIntId, hostInt);
      isrEventId = CpIntc_getEventId(hostInt);

      Hwi_Params_init(&hwiAttrs);
      hwiAttrs.arg         = hostInt;
      hwiAttrs.eventId     = isrEventId;
      hwiAttrs.enableInt = TRUE;

      /* Register CpIntc_dispatch with BIOS Hwi module.  This will invoke the Qpend_isr */
      Hwi_create(params->intVectorId, (Hwi_FuncPtr)CpIntc_dispatch, &hwiAttrs, eb); 
      Hwi_enableInterrupt(params->intVectorId); 
    }
  }
  /* END: Once per core configuration */
  else
  {
    /* Increment module variable each time this core attaches to another.  
      * Will be used to track when to close this core's socket. */
    TransportQmss_module->qmssInitialized++;
    /* Restore interrupts */
    Hwi_restore(hwiKey);

    if (coreNum == entry.ownerProcId) 
    {  
      /* If it's the owner core provide global queue array to remote core */
      Ipc_writeConfig(params->remoteProcId, 0x12345678,
                                 TransportQmss_module->rxQueueId,
                                 sizeof(UInt32) * (ti_sdo_utils_MultiProc_numProcessors + 1));
      
      if (!TransportQmss_useAccumulatorLogic)
      {
        /* If it's the owner core provide global system int array to other cores */
        Ipc_writeConfig(params->remoteProcId, 0x87654321,
                                   TransportQmss_module->cpIntSystemInt,
                                   sizeof(UInt32) * ti_sdo_utils_MultiProc_numProcessors);
      }
    }
  }

  /* Register the transport with MessageQ */
  flag = ti_sdo_ipc_MessageQ_registerTransport(
          TransportQmss_Handle_upCast(obj), procId, params->priority);
  if (flag == FALSE) 
  {
      return (1);
  }

  return (0);
}

/*
 *  ======== TransportQmss_Instance_finalize ========
 */
Void TransportQmss_Instance_finalize(TransportQmss_Object* obj, Int status)
{
  Hwi_Handle hwiHandle;
  Int32 result;    
  SharedRegion_Entry entry;
  UInt32 i;
  UInt32 channel;
  Int32 coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
  
  
  /* Decrement the module variable tracking number of connections */
  TransportQmss_module->qmssInitialized--;
  
  /* Only close QMSS after this core detaches from all other cores */
  if (TransportQmss_module->qmssInitialized == 0)
  {
    /* Unregister interrupt */
    UInt cpIntId = (coreNum < 4 ? 0 : 1);
    UInt hostInt = getIntcHostInt (coreNum, TransportQmss_intcGemEvent);

    CpIntc_disableHostInt(cpIntId, hostInt);
    
    hwiHandle = Hwi_getHandle(obj->intVectId);
    Hwi_delete(&hwiHandle);   

    /* get region 0 information */
    SharedRegion_getEntry(obj->memRegion, &entry);
    
    /* delete all queues and disable all accumulator channels */
    if (MultiProc_self() == entry.ownerProcId) 
    {
      if (TransportQmss_useAccumulatorLogic)
      {
        for (i=0; i < ti_sdo_utils_MultiProc_numProcessors; i++) 
        {    
          /* find the accumulator channel */
          channel = TransportQmss_module->rxQueueId[i] - QMSS_HIGH_PRIORITY_QUEUE_BASE;
                          
          /* Disable accumulator */    
          result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, channel);
          Assert_isTrue ((result == QMSS_ACC_SOK), TransportQmss_A_qmssError);
        }   
      }
        
      /* close all queues */
      for (i=0; i < (ti_sdo_utils_MultiProc_numProcessors+1); i++) 
      {    
        result = Qmss_queueClose (TransportQmss_module->rxQueueId[i]);
        Assert_isTrue ((result == CPPI_SOK), TransportQmss_A_qmssError);
      }

      /* Free global receive queue number memory.  Should only be done by the
        * memory region owner after it has detached from all cores. */
      if (TransportQmss_module->rxQueueId != NULL)
      {   
        Memory_free(0, TransportQmss_module->rxQueueId,
            sizeof(UInt32) * (ti_sdo_utils_MultiProc_numProcessors+1));
      }     
    }
    
    if (TransportQmss_useAccumulatorLogic)
    {
      /* Free allocated accumulator list memory */    
      if (TransportQmss_module->hiPrioList != NULL) 
      {
        Memory_free(0,
            (UInt32*)(global_l2_address((UInt32)TransportQmss_module->hiPrioList)),
            sizeof(UInt32)*(TransportQmss_accuHiPriListSize));
      }   
    }
    else
    {
      /* Free allocated accumulator list memory */    
      if (TransportQmss_module->cpIntSystemInt != NULL) 
      {
        Memory_free(0,
            (UInt32*)(global_l2_address((UInt32)TransportQmss_module->cpIntSystemInt)),
            sizeof(UInt32) * ti_sdo_utils_MultiProc_numProcessors);
      }  
    }
  }

  switch(status) 
  {
    case 0: /* MessageQ_registerTransport succeeded */
      ti_sdo_ipc_MessageQ_unregisterTransport(obj->remoteProcId,
          obj->priority);

      /* fall thru OK */
    case 1: /* MessageQ_registerTransport failed */
      break;
  }
}

/*
 *  ======== TransportQmss_put ========
 *  Routine used to send packets via QMSS driver
 */
Bool TransportQmss_put(TransportQmss_Object *obj, Ptr msg)
{
  Qmss_QueueHnd rxQueueHdlr = TransportQmss_module->rxQueueId[obj->remoteProcId];
  Qmss_QueueHnd freeQueueHdlr = TransportQmss_module->rxQueueId[ti_sdo_utils_MultiProc_numProcessors];
  Cppi_Desc* monoDescPtr;
#if USE_CSL_CACHE_APIS  
  UInt32 old_int;

  old_int = _disable_interrupts();
#endif  

  /* Get a free descriptor */
  monoDescPtr = (Cppi_Desc *) Qmss_queuePop (freeQueueHdlr);
#if ENABLE_PUT_ISR_ASSERTS
  Assert_isTrue(monoDescPtr != NULL, TransportQmss_A_qmssError);
#endif
      
  if (monoDescPtr == NULL) 
  {
    return (FALSE);
  }

  /* Writeback the MessageQ msg to shared memory (MSMC/DDR3) */
#if USE_CSL_CACHE_APIS 
  CACHE_wbL1d (msg, ((MessageQ_Msg)(msg))->msgSize, CACHE_WAIT);
#else
  Cache_wb(msg, ((MessageQ_Msg)(msg))->msgSize, CACHE_TYPE, TRUE);
#endif

  /* Add data buffer */
  Cppi_setData (Cppi_DescType_MONOLITHIC, monoDescPtr, (UInt8 *) &msg, sizeof(Ptr));
  Cppi_setDataLen(Cppi_DescType_MONOLITHIC, monoDescPtr, sizeof(Ptr));

  /* If the descriptor is configured in shared memory (by setting the flag 
   * in RTSC cfg file) and cache is enabled do a cache wb for the whole descriptor 
   */
  if (TransportQmss_descriptorIsInSharedMem && obj->cacheEnabled) 
  {
#if USE_CSL_CACHE_APIS     
    CACHE_wbL1d (monoDescPtr, TransportQmss_descriptorSize, CACHE_FENCE_WAIT);
#else
    Cache_wb(monoDescPtr, TransportQmss_descriptorSize, CACHE_TYPE, TRUE);
#endif
  }
    
  /* Push descriptor to Rx queue of remote core */
  Qmss_queuePushDescSize (rxQueueHdlr, (UInt32 *) monoDescPtr, TransportQmss_descriptorSize);

#if USE_CSL_CACHE_APIS 
  _restore_interrupts(old_int);
#endif

  return (TRUE);
}

/*
 *  ======== TransportQmss_control ========
 */
Bool TransportQmss_control(TransportQmss_Object *obj, UInt cmd,
    UArg cmdArg)
{
    return (FALSE);
}

/*
 *  ======== TransportQmss_getStatus ========
 */
Int TransportQmss_getStatus(TransportQmss_Object *obj)
{
    return (0);
}

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*
 *  ======== TransportQmss_setErrFxn ========
 */
Void TransportQmss_setErrFxn(TransportQmss_ErrFxn errFxn)
{
    /* Ignore the errFxn */
}

/*
 *************************************************************************
 *                       Internal functions
 *************************************************************************
 */

/*
 *  ======== TransportQmss_Accumulator_isr ========
 */
Void TransportQmss_Accumulator_isr(UArg arg)
{
  TransportQmss_Object *obj;
  UInt32 coreNum;
  Int16 channel;
  Qmss_QueueHnd freeQueueHdlr, rxQueueHdlr;
  UInt32 *accuList;
  UInt32 *msgAddrInDesc;
  MessageQ_Msg rxMsg;
  UInt32 buffLen;
  Cppi_Desc *pCppiDesc;
  UInt32 queueId;
  UInt32 count, i;
#if USE_CSL_CACHE_APIS  
  UInt32 old_int;

  old_int = _disable_interrupts();
#endif    

  /* Get the accumulator channel number to ACK the interrupt */
  coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
  rxQueueHdlr = TransportQmss_module->rxQueueId[coreNum];
  channel = rxQueueHdlr - QMSS_HIGH_PRIORITY_QUEUE_BASE;
  
  obj = (TransportQmss_Object *)arg;    /* this obj might not be the obj associated with this interrupt, 
                                              but it has many of the configuration data available. */

#if ENABLE_PUT_ISR_ASSERTS  
  /* Make sure the TransportQmss_Object is not NULL */
  Assert_isTrue(obj != NULL, TransportQmss_A_qmssIsrObjectNULL);
  /* Make sure the high priority list is not NULL */
  Assert_isTrue(TransportQmss_module->hiPrioList != NULL, TransportQmss_A_qmssAccumulatorListNULL);
#endif  
  
  /* Read accumulator list */
  /* hiPrioList is already converted to global address at init */    
  accuList = (UInt32 *)TransportQmss_module->hiPrioList;
  
  if (TransportQmss_module->usePingList)
  {    
    count = accuList[0]; 
  } 
  else 
  {
    count = accuList[TransportQmss_intThreshold+1];
  }

  if (count == 0) 
  {    
    /* Clear INTD */
    Qmss_ackInterrupt (channel, 1);
    Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, channel);

    return; /* Shouldn't be here */
  }
      
  freeQueueHdlr = TransportQmss_module->rxQueueId[ti_sdo_utils_MultiProc_numProcessors];

  for (i = 1; i <= count; i ++)
  {
    if (TransportQmss_module->usePingList) 
    {
      pCppiDesc = (Cppi_Desc *) accuList [i];
    } 
    else 
    {
      pCppiDesc = (Cppi_Desc *) accuList[i + TransportQmss_intThreshold + 1];
    }

    /* Descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor size, always mask off the last 
     * 4 bits of the address.
     */
    pCppiDesc = (Ptr) ((UInt32) pCppiDesc & 0xFFFFFFF0);

    /* If the descriptor is configured in shared memory (by setting the flag 
     * in RTSC cfg file), do a cache wb for the whole descriptor now. This descriptor size
     * should be small - 32 bytes
     */
    if (TransportQmss_descriptorIsInSharedMem && obj->cacheEnabled) 
    {
#if USE_CSL_CACHE_APIS
      CACHE_invL1d (pCppiDesc, TransportQmss_descriptorSize, CACHE_WAIT);
#else
      Cache_inv(pCppiDesc, TransportQmss_descriptorSize, CACHE_TYPE, TRUE);
#endif
    }        
    
    Cppi_getData (Cppi_DescType_MONOLITHIC, pCppiDesc, (UInt8 **) &msgAddrInDesc,  &buffLen);
    
    rxMsg = (MessageQ_Msg) *msgAddrInDesc;

    /* Invalidate the MessageQ msg prior to accessing it for the queueId */
#if USE_CSL_CACHE_APIS
    CACHE_invL1d (rxMsg, rxMsg->msgSize, CACHE_WAIT);
#else    
    Cache_inv(rxMsg, rxMsg->msgSize, CACHE_TYPE, TRUE);
#endif

    /* Push the received message up to the messageQ layer. */
    queueId = MessageQ_getDstQueue(rxMsg);
    
    /* Put messageQ buffer on queue */
    MessageQ_put(queueId, rxMsg);
    
    /* Recycle Rx BDs. */
    Qmss_queuePushDesc (freeQueueHdlr, (Void *)pCppiDesc);                
  }   /* end for */ 

  /* ping/pong switching */
  if (TransportQmss_module->usePingList) 
  {        
    /* switch to pong list */
    TransportQmss_module->usePingList = 0; 
  }
  else 
  {
    /* switch to ping list */
    TransportQmss_module->usePingList = 1; 
  }

#if USE_CSL_CACHE_APIS 
  _restore_interrupts(old_int);
#endif
  
  /* Set the EOI to indicate host is done processing. The page is freed by host to accumulator.
   * Accumulator can start writing to the freed page.
   */
  Qmss_ackInterrupt (channel, 1);
  Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, channel);
}

/*
 *  ======== TransportQmss_Qpend_isr ========
 */
Void TransportQmss_Qpend_isr(UArg arg)
{
  TransportQmss_Object *obj;
  UInt32 coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
  Qmss_QueueHnd freeQueueHdlr, rxQueueHdlr;
  UInt32 *msgAddrInDesc;
  MessageQ_Msg rxMsg;
  UInt32 buffLen;
  Cppi_Desc *pCppiDesc;
  UInt32 queueId;
  UInt cpIntId = ((coreNum < 4) ? 0 : 1);
#if USE_CSL_CACHE_APIS  
  UInt32 old_int;

  old_int = _disable_interrupts();
#endif    

  /* Disable system interrupt while in ISR */
  CpIntc_disableSysInt(cpIntId, TransportQmss_module->cpIntSystemInt[coreNum]);

  /* Get the accumulator channel number to ACK the interrupt */
  rxQueueHdlr = TransportQmss_module->rxQueueId[coreNum];
  freeQueueHdlr = TransportQmss_module->rxQueueId[ti_sdo_utils_MultiProc_numProcessors];
  
  obj = (TransportQmss_Object *)arg;    /* this obj might not be the obj associated with this interrupt, 
                                              but it has many of the configuration data available. */

#if ENABLE_PUT_ISR_ASSERTS                                              
  /* Make sure the TransportQmss_Object is not NULL */
  Assert_isTrue(obj != NULL, TransportQmss_A_qmssIsrObjectNULL);
  Assert_isTrue(Qmss_getQueueEntryCount (rxQueueHdlr) != 0, 
                          TransportQmss_A_qmssAccumulatorListNULL);
#endif  

  while ((pCppiDesc = (Cppi_Desc *) QMSS_DESC_PTR (Qmss_queuePop (rxQueueHdlr))) != NULL)
  {
    /* If the descriptor is configured in shared memory (by setting the flag 
     * in RTSC cfg file), do a cache wb for the whole descriptor now. This descriptor size
     * should be small - 32 bytes
     */
    if (TransportQmss_descriptorIsInSharedMem && obj->cacheEnabled) 
    {
#if USE_CSL_CACHE_APIS 
      CACHE_invL1d (pCppiDesc, TransportQmss_descriptorSize, CACHE_WAIT);
#else
      Cache_inv(pCppiDesc, TransportQmss_descriptorSize, CACHE_TYPE, TRUE);
#endif
    }        
    
    Cppi_getData (Cppi_DescType_MONOLITHIC, pCppiDesc, (UInt8 **) &msgAddrInDesc,  &buffLen);
    
    rxMsg = (MessageQ_Msg) *msgAddrInDesc;

    /* Invalidate the MessageQ msg prior to accessing it for the queueId */
#if USE_CSL_CACHE_APIS 
    CACHE_invL1d (rxMsg, rxMsg->msgSize, CACHE_WAIT);
#else
    Cache_inv(rxMsg, rxMsg->msgSize, CACHE_TYPE, TRUE);
#endif

    /* Push the received message up to the messageQ layer. */
    queueId = MessageQ_getDstQueue(rxMsg);
    
    /* Put messageQ buffer on queue */
    MessageQ_put(queueId, rxMsg);
    
    /* Recycle Rx BDs. */
    Qmss_queuePushDesc (freeQueueHdlr, (Void *)pCppiDesc);                
  }   /* end while */ 

#if USE_CSL_CACHE_APIS 
  _restore_interrupts(old_int);
#endif

  CpIntc_clearSysInt(cpIntId, TransportQmss_module->cpIntSystemInt[coreNum]);

  /* Enable the system interrupt upon ISR completion */
  CpIntc_enableSysInt(cpIntId, TransportQmss_module->cpIntSystemInt[coreNum]);
}

