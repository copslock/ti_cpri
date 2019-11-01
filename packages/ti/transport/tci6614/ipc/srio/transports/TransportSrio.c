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
 *  ======== TransportSrio.c ========
 */

/* XDC includes */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>

/* BIOS includes */
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c66/Cache.h>

/* IPC external includes */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MultiProc.h>

/* IPC internal includes */
#include <ti/sdo/ipc/_MessageQ.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

#include <ti/csl/csl_chip.h>

#include "package/internal/TransportSrio.xdc.h"

/* Event ID is chosen based on interrupt mapping in http://focus.ti.com/lit/ug/sprugr9/sprugr9.pdf 
  * One SRIO driver, using one high priority QMSS channel, is created per core.  Core 0 through 3 will use
  * high priority queues 704-707.  These all map to Event ID 48 */
#define QMSS_HIGH_PRIO_INT0_EVENT_ID (48)
#define QMSS_QUEUES_PER_EVENTID (4) /* On c6670 there are eight queues that map to the event ID.  one
                                                                    * for each core */

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

/*
 *************************************************************************
 *                       Module functions
 *************************************************************************
 */

/**
 *  @b Description
 *  @n  
 *      Application Raw Receive Cleanup API.
 *
 *  @retval
 *      Not Applicable.
 */
static void TransportSrio_RawRxFree(Srio_DrvBuffer hDrvBuffer)
{
  Qmss_QueueHnd  returnQueueHnd;
  MessageQ_Msg rxMsg;
  Int32 status;
  UInt32 rxNumBytes;

  /* Get the return queue. */
  returnQueueHnd = Qmss_getQueueHandle(Cppi_getReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)hDrvBuffer));

  /* Get the received MessageQ msg. */
  Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)hDrvBuffer, (UInt8**)&rxMsg,(UInt32*)&rxNumBytes);

  /* Free the MessageQ Msg */
  status = MessageQ_free(rxMsg);
  Assert_isTrue ((status >= 0), TransportSrio_A_srioMsgQFreeError);

  /* Allocate a new MessageQ msg to link with the descriptor */
  rxMsg = MessageQ_alloc(TransportSrio_module->rxMsgQHeapId, TransportSrio_srioMaxMtuSizeBytes);

  /* Set the DATA and ORIGINAL DATA in the buffer descriptor. */
  Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hDrvBuffer, (UInt8 *)rxMsg,
                          TransportSrio_srioMaxMtuSizeBytes);
  Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hDrvBuffer, (UInt8 *)rxMsg,
                                           TransportSrio_srioMaxMtuSizeBytes); 

  /* Writeback changes to descriptor */
  Cache_wb((Ptr) hDrvBuffer, TransportSrio_descriptorSize, Cache_Type_L1D, TRUE);  

  /* Push the descriptor into the return queue. */
  Qmss_queuePushDescSize (returnQueueHnd, (Ptr)hDrvBuffer, sizeof(Cppi_HostDesc));
}


/*
 *************************************************************************
 *                       Instance functions
 *************************************************************************
 */
 
/*
 *  ======== TransportSrio_Instance_init ========
 */
Int TransportSrio_Instance_init(TransportSrio_Object *obj,
        UInt16 procId, const TransportSrio_Params *params,
        Error_Block *eb)
{
  Int32              globalCoreNum;
  Srio_DrvConfig  cfg;
  Srio_DrvHandle  hSrioDriver;
  UInt8           isAllocated;
  Qmss_QueueHnd rxFreeQueueHdlr, rxQueueHdlr, tmpQueueHdlr, txCompQueueHdlr, txFreeQueueHdlr;
  UInt32 numAllocated; 
  UInt16 index;
  Cppi_DescCfg    descCfg;
  Cppi_HostDesc*  pHostDesc;  
  Hwi_Params          hwiAttrs;
  Srio_SockHandle         srioSocket;
  Srio_SockBindAddrInfo   bindInfo;
  Qmss_Queue queueInfo;
  Int16 isrEventId;
  UInt16 queue;
  Int32 result;
  Bool flag;
  UInt32 hwiKey;

  /* Get this core's number */
  globalCoreNum = MultiProc_self();
  
  /* Check whether remote proc ID has been set and isn't the same as the
    * local proc ID */
  Assert_isTrue ((procId != MultiProc_INVALIDID) &&
                           (procId != globalCoreNum),
                           TransportSrio_A_srioProcIdInvalid);

  /* Disable interrupts prior to checking and incrementing srioDriverCreated to
    * any conflicts with this code being executed in another task */
  hwiKey = Hwi_disable();
  
  /* Configure and start the SRIO Driver - One driver created per core.
    * Also create and bind a socket to this core.  Only one socket will be
    * created per core. */
  /* START: Once per core configuration */
  if (TransportSrio_module->srioDriverCreated == 0) 
  {
    /* Check whether remote proc ID has been set.  Transport instance init could fail if the application
      * calls TransportSrio_create to establish a manual connection to a core off-board without attaching to a 
      * local core first and then passing NULL for the transportSrio_params. */
    Assert_isTrue ((params->remoteProcId != MultiProc_INVALIDID),
                             TransportSrio_A_srioProcIdInvalid);
    
    /* Increment module variable each time this core attaches to another.  
      * Will be used to track when to close this core's socket. */
    TransportSrio_module->srioDriverCreated++;
    /* Restore interrupts */
    Hwi_restore(hwiKey);
    
    /* Set the HEAP ID for message Q alloc in the ISR */
    TransportSrio_module->rxMsgQHeapId = params->rxMessageQHeapId;
    
    /* Initialize the SRIO Driver Configuration. */
    memset ((Void *)&cfg, 0, sizeof(Srio_DrvConfig));

    /* The application will manage the driver buffers attached to the packet descriptors.  */
    cfg.bAppManagedConfig = TRUE;

    /* Create the application receive free queue. */
    rxFreeQueueHdlr = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 
                                                                QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (rxFreeQueueHdlr < 0)
	{
	    return (1);
    }

    /* Create the application receive completion accumulator queue. */
    rxQueueHdlr = Qmss_queueOpen (Qmss_QueueType_HIGH_PRIORITY_QUEUE, 
                                                         QMSS_PARAM_NOT_SPECIFIED, &isAllocated);    
    if (rxQueueHdlr < 0)
    {
      return (1);
    }

    /* Store the receive Q for instance_finalize */
    TransportSrio_module->srioRxQ = (UInt32 *) rxQueueHdlr;

    /* Setup descriptors in the receive free queue */
    descCfg.memRegion                 = (Qmss_MemRegion) params->descMemRegion;
    descCfg.descNum                   = TransportSrio_srioNumRxDescriptors;
    descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType                  = Cppi_DescType_HOST;
    descCfg.returnQueue               = Qmss_getQueueNumber (rxFreeQueueHdlr);
    descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.returnPushPolicy          = Qmss_Location_HEAD;
    descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
    descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

    /* Initialize the descriptors. */
    tmpQueueHdlr = Cppi_initDescriptor (&descCfg, &numAllocated);
    if (tmpQueueHdlr < 0)
    {
      return (1);
    }

    /* Initialize the application receive buffers. */
    for (index = 0; index < descCfg.descNum; index++)
    {
      /* Pop off a descriptor */
      pHostDesc = (Cppi_HostDesc *)Qmss_queuePop(tmpQueueHdlr);
      if (pHostDesc == NULL)
      {
        return (1);
      }
      
      /* Data buffers will be linked to these descriptors via the TransportSrio_initRxDescBufs function. 
        * This function should be called after Instance_init (Ipc_attach completes and before any transfers
        * occur */
        
      /* Add the packet descriptor to the Application Receive Free Queue. */
      Qmss_queuePushDesc (rxFreeQueueHdlr, (uint32_t*)pHostDesc);
    }

    /* Store receive Free queue for later initialization */
    TransportSrio_module->srioRxFreeQ = (UInt32 *) rxFreeQueueHdlr;

    /* Close the temporary queue. */
    Qmss_queueClose (tmpQueueHdlr);

    /* Get the queue information about the receive completion queue. */
    queueInfo = Qmss_getQueueNumber (rxQueueHdlr);

    /* The application managed configuration is capable of reception. */
    cfg.u.appManagedCfg.bIsRxFlowCfgValid = 1;

    /* Configure the Receive Flow */
    cfg.u.appManagedCfg.rxFlowCfg.flowIdNum          = -1;
    cfg.u.appManagedCfg.rxFlowCfg.rx_dest_qnum       = queueInfo.qNum;
    cfg.u.appManagedCfg.rxFlowCfg.rx_dest_qmgr       = queueInfo.qMgr;
    cfg.u.appManagedCfg.rxFlowCfg.rx_sop_offset      = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_ps_location     = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_desc_type       = 0x1; /* Host Descriptor. */
    cfg.u.appManagedCfg.rxFlowCfg.rx_error_handling  = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_psinfo_present  = 0x1; /* PS Information */
    cfg.u.appManagedCfg.rxFlowCfg.rx_einfo_present   = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo     = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi     = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo      = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi      = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo_sel = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi_sel = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo_sel  = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi_sel  = 0x0;

    /* Disable Receive size thresholds. */
    cfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0_en = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1_en = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2_en = 0x0;

    /* Use the Application Receive Free Queue for picking all descriptors. */
    queueInfo = Qmss_getQueueNumber (rxFreeQueueHdlr);
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qnum       = queueInfo.qNum;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qmgr       = queueInfo.qMgr;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qnum       = 0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qmgr       = 0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qnum       = 0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qmgr       = 0;

    /* Use the Receive Queue for picking the SOP packet also. */
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qnum   = queueInfo.qNum;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qmgr   = queueInfo.qMgr;

    /* There are no size thresholds configured. */
    cfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0    = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1    = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2    = 0x0;

    /* The other threshold queues do not need to be configured */
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qnum   = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qmgr   = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qnum   = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qmgr   = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qnum   = 0x0;
    cfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qmgr   = 0x0;

        /* Populate the rest of the configuration. */
    cfg.u.appManagedCfg.rawRxFreeDrvBuffer = TransportSrio_RawRxFree;
    cfg.u.appManagedCfg.rxDescSize  = TransportSrio_descriptorSize;

    /* Program the accumulator. */
    cfg.u.appManagedCfg.bIsAccumlatorCfgValid = 1;

      /* Accumulator Configuration. */
    cfg.u.appManagedCfg.accCfg.channel             = rxQueueHdlr - QMSS_HIGH_PRIORITY_QUEUE_BASE;
    cfg.u.appManagedCfg.accCfg.command             = Qmss_AccCmd_ENABLE_CHANNEL;
    cfg.u.appManagedCfg.accCfg.queueEnMask         = 0;
    queueInfo = Qmss_getQueueNumber (rxQueueHdlr);
    cfg.u.appManagedCfg.accCfg.queMgrIndex         = queueInfo.qNum;
    cfg.u.appManagedCfg.accCfg.maxPageEntries      = TransportSrio_intThreshold + 1;
    cfg.u.appManagedCfg.accCfg.timerLoadCount      = TransportSrio_timerLoadCount;
    cfg.u.appManagedCfg.accCfg.interruptPacingMode = Qmss_AccPacingMode_LAST_INTERRUPT;
    if (TransportSrio_pacingEnabled)
    {
      cfg.u.appManagedCfg.accCfg.interruptPacingMode = Qmss_AccPacingMode_LAST_INTERRUPT;
    } 
    else 
    {
      cfg.u.appManagedCfg.accCfg.interruptPacingMode = Qmss_AccPacingMode_NONE;
    }    
    cfg.u.appManagedCfg.accCfg.listEntrySize       = Qmss_AccEntrySize_REG_D;
    cfg.u.appManagedCfg.accCfg.listCountMode       = Qmss_AccCountMode_ENTRY_COUNT;
    cfg.u.appManagedCfg.accCfg.multiQueueMode      = Qmss_AccQueueMode_SINGLE_QUEUE;

    /* Configure the accumulator list */
    if (TransportSrio_module->hiPrioList == NULL)
    {       
      Assert_isTrue ((TransportSrio_accuHiPriListSize >= ((TransportSrio_intThreshold * 2) + 2)), 
                                TransportSrio_A_srioAccumulatorListSize); 
      
      /* Allocate memory for the accumulator list */    
      TransportSrio_module->hiPrioList = (UInt32*)l2_global_address(
                      (UInt32) Memory_alloc(0,
                      sizeof(UInt32)*(TransportSrio_accuHiPriListSize),
                      16, /* 16-byte aligned */
                      eb));
                                  
      Assert_isTrue ((TransportSrio_module->hiPrioList != NULL), 
                                TransportSrio_A_srioAccumulatorListNULL);
    }

    /* program the high priority accumulator */
    memset ((Void *)(TransportSrio_module->hiPrioList), 0, 
            sizeof(UInt32)*(TransportSrio_accuHiPriListSize));

    cfg.u.appManagedCfg.accCfg.listAddress = (UInt32)TransportSrio_module->hiPrioList;

    /* Specify that Srio queue allocation is up to the driver */
    cfg.u.appManagedCfg.txQueueNum = QMSS_PARAM_NOT_SPECIFIED;

    /* Create the SRIO driver and store in module instance */
    hSrioDriver = Srio_start(&cfg);
    Assert_isTrue ((hSrioDriver != NULL), TransportSrio_A_srioDriverError);

    TransportSrio_module->srioDriverHandle = hSrioDriver;
 
    /* Register interrupt handler */
    isrEventId = QMSS_HIGH_PRIO_INT0_EVENT_ID;
    queue = rxQueueHdlr - QMSS_HIGH_PRIORITY_QUEUE_BASE;
    while (queue >= QMSS_QUEUES_PER_EVENTID)
    {
      isrEventId++;
      queue -= QMSS_QUEUES_PER_EVENTID;
    }
        
    Hwi_Params_init(&hwiAttrs);
    hwiAttrs.maskSetting = Hwi_MaskingOption_SELF;
    hwiAttrs.arg         = (UArg)TransportSrio_module->srioDriverHandle;
    hwiAttrs.eventId     = isrEventId;
    Hwi_create(params->intVectorId, (Hwi_FuncPtr)TransportSrio_isr, &hwiAttrs, eb);

    Hwi_enableInterrupt(params->intVectorId);

    /* Open SRIO Socket in non-blocking mode.  Non-Blocking mode is used because the
      * receive ISR will loop on the Srio_sockRecv until no data is available. */
    srioSocket =  Srio_sockOpen (TransportSrio_module->srioDriverHandle, Srio_SocketType_RAW_TYPE11, FALSE);
    Assert_isTrue ((srioSocket != NULL), TransportSrio_A_srioSocketError);

    /* Initialize the core bindings using the configuration settings */
    bindInfo.type11.tt = TransportSrio_srioCoreTT[globalCoreNum];  /* Num bits in identifier for this core */
    bindInfo.type11.id = TransportSrio_srioCoreDeviceId[globalCoreNum]; /* Device ID for this core */
    bindInfo.type11.mbox = TransportSrio_srioCoreMailbox[globalCoreNum];  /* Mailbox for this core */
    bindInfo.type11.letter = TransportSrio_srioCoreLetter[globalCoreNum]; /* Letter for this core */
    bindInfo.type11.segMap = TransportSrio_srioCoreSegMap[globalCoreNum]; /* Segmentation map for this core */

    /* Bind the SRIO socket */
    result = Srio_sockBind (srioSocket, &bindInfo);
    Assert_isTrue ((result >= 0), TransportSrio_A_srioSocketBindError);
    TransportSrio_module->srioSocketHandle = srioSocket;

    /* Set socket receive queue to be number of descriptors RX descriptors to avoid socket receive overflow */
    result = Srio_setSockOpt (srioSocket, Srio_Opt_PENDING_PKT_COUNT, 
                                              (void *) &TransportSrio_srioNumRxDescriptors, sizeof(UInt16));
    Assert_isTrue ((result >= 0), TransportSrio_A_srioSocketOptionError);

    /* Create the transmit completion queue.  This will be used to clean up the linked msgQ buffers. */
    txCompQueueHdlr = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 
                                                                  QMSS_PARAM_NOT_SPECIFIED, &isAllocated);    
    if (txCompQueueHdlr < 0)
    {
      return (1);
    }

    /* Populate the CPPI descriptor configuration. */
    descCfg.memRegion                 = (Qmss_MemRegion) params->descMemRegion;
    descCfg.descNum                   = TransportSrio_srioNumTxDescriptors;
    descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType                  = Cppi_DescType_HOST;
    descCfg.returnQueue = Qmss_getQueueNumber (txCompQueueHdlr);
    descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.returnPushPolicy          = Qmss_Location_TAIL;
    descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
    descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

    /* Initialize the descriptors and place all of them into the general purpose temporary queue */
    txFreeQueueHdlr  = Cppi_initDescriptor (&descCfg, &numAllocated);
    if (txFreeQueueHdlr < 0)
    {
        return (1);
    }

    TransportSrio_module->srioTxFreeQ = (UInt32 *) txFreeQueueHdlr;
    TransportSrio_module->srioTxCompQ = (UInt32 *) txCompQueueHdlr;
  }
  /* END: Once per core configuration */
  else
  {
    /* Increment module variable each time this core attaches to another.  
      * Will be used to track when to close this core's socket. */
    TransportSrio_module->srioDriverCreated++;
    /* Restore interrupts */
    Hwi_restore(hwiKey);
  }

  obj->priority      = params->priority;
  obj->remoteProcId  = procId;
  obj->intVectId = params->intVectorId;

  /* Register the transport with MessageQ */
  flag = ti_sdo_ipc_MessageQ_registerTransport(
          TransportSrio_Handle_upCast(obj), procId, params->priority);
  if (flag == FALSE) 
  {
      return (1);
  }

  return (0);
}

/*
 *  ======== TransportSrio_Instance_finalize ========
 */
Void TransportSrio_Instance_finalize(TransportSrio_Object* obj, Int status)
{
  Hwi_Handle hwiHandle;
  Int32 result;
  UInt32 channel;
  Qmss_QueueHnd txCompQ = (Qmss_QueueHnd) TransportSrio_module->srioTxCompQ;
  Qmss_QueueHnd txErrorQ = (Qmss_QueueHnd) TransportSrio_srioGarbageQ;
  Cppi_HostDesc *pHostDesc;
  Ptr msg;
  Int32 msgSize;
  
  /* Decrement the module variable tracking number of connections */
  TransportSrio_module->srioDriverCreated--;
  
  /* Only close the socket after this core detaches from all other cores */
  if (TransportSrio_module->srioDriverCreated == 0)
  {
    /* Check the error queue to clean up any dropped packet descriptors. */
    if (Qmss_getQueueEntryCount (txErrorQ))
    {
      while ((pHostDesc = (Cppi_HostDesc *) QMSS_DESC_PTR (Qmss_queuePop (txErrorQ))) != NULL)
      {
        /* Invalidate the descriptor prior to accessing it for the return queue */
        Cache_inv((Ptr) pHostDesc, TransportSrio_descriptorSize, Cache_Type_L1D, TRUE);
        
        /* Push packet onto return queue for cleanup */
        Qmss_queuePushDesc (Qmss_getQueueHandle(Cppi_getReturnQueue(Cppi_DescType_HOST, 
                                                     (Cppi_Desc *)pHostDesc)), (UInt32 *)pHostDesc);
      }
    }
  
    /* Cleanup any packets in the tx completion queue */
    if (Qmss_getQueueEntryCount (txCompQ))
    {
      while ((pHostDesc = (Cppi_HostDesc *) QMSS_DESC_PTR (Qmss_queuePop (txCompQ))) != NULL)
      {
        /* Invalidate the descriptor prior to accessing it for its payload */
        Cache_inv((Ptr) pHostDesc, TransportSrio_descriptorSize, Cache_Type_L1D, TRUE);
        
        Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *) pHostDesc, (UInt8**)&msg,(UInt32*)&msgSize);

        /* Free the MsgQ msg */
        status = MessageQ_free(msg);
        Assert_isTrue ((status >= 0), TransportSrio_A_srioMsgQFreeError);

        /* Push the descriptor into the free queue. */
        Qmss_queuePushDesc ((Qmss_QueueHnd) TransportSrio_module->srioTxFreeQ, (Ptr)pHostDesc);
      }
    }
    
    /* Unregister interrupt */
    hwiHandle = Hwi_getHandle(obj->intVectId);
    Hwi_delete(&hwiHandle);        

    /* Close the socket */
    result = Srio_sockClose (TransportSrio_module->srioSocketHandle);
    Assert_isTrue ((result >= 0), TransportSrio_A_srioSocketCloseError);

    /* find the accumulator channel */
    channel = (UInt32) (TransportSrio_module->srioRxQ - QMSS_HIGH_PRIORITY_QUEUE_BASE);
                          
    /* Disable accumulator */    
    result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, channel);

    /* close all queues */
    result = Qmss_queueClose ((Qmss_QueueHnd) TransportSrio_module->srioTxCompQ);
    Assert_isTrue ((result == CPPI_SOK), TransportSrio_A_srioQueueCloseError);

    result = Qmss_queueClose ((Qmss_QueueHnd) TransportSrio_module->srioTxFreeQ);
    Assert_isTrue ((result == CPPI_SOK), TransportSrio_A_srioQueueCloseError);

    result = Qmss_queueClose ((Qmss_QueueHnd) TransportSrio_module->srioRxFreeQ);
    Assert_isTrue ((result == CPPI_SOK), TransportSrio_A_srioQueueCloseError);

    result = Qmss_queueClose ((Qmss_QueueHnd) TransportSrio_module->srioRxQ);
    Assert_isTrue ((result == CPPI_SOK), TransportSrio_A_srioQueueCloseError);
     
    /* Free allocated memory */    
    if (TransportSrio_module->hiPrioList != NULL)
    {
      Memory_free(0,
          (UInt32*)(global_l2_address((UInt32)TransportSrio_module->hiPrioList)),
          sizeof(UInt32)*(TransportSrio_accuHiPriListSize));
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
 *  ======== TransportSrio_put ========
 *  Routine used to send packets via SRIO driver
 */
Bool TransportSrio_put(TransportSrio_Object *obj, Ptr msg)
{
  Cppi_HostDesc *pHostDesc;
  Srio_SockAddrInfo dstSrioAddr;
  Int32 status;
  Int32 msgSize = MessageQ_getMsgSize(msg);
  Qmss_QueueHnd txCompQ = (Qmss_QueueHnd) TransportSrio_module->srioTxCompQ;
  Qmss_QueueHnd txErrorQ = (Qmss_QueueHnd) TransportSrio_srioGarbageQ;
  
  /* Get a transmit buffer from the SRIO Driver. */
  pHostDesc = (Cppi_HostDesc *)Qmss_queuePop((Qmss_QueueHnd) TransportSrio_module->srioTxFreeQ);

  Cache_wb(msg, msgSize, Cache_Type_L1D, TRUE);

  /* Populate the Transmit Packet Descriptor data with the messageQ msg */
  Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, (UInt8*)msg, msgSize);
  Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, (UInt8*)msg, msgSize);

  /* Configure the packet length */
  Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, msgSize);

  /* Set the destination address. */
  dstSrioAddr.type11.tt = TransportSrio_srioCoreTT[obj->remoteProcId]; /* Num bits in identifier for remote core */
  dstSrioAddr.type11.id = TransportSrio_srioCoreDeviceId[obj->remoteProcId]; /* Device ID for remote core */
  dstSrioAddr.type11.mbox = TransportSrio_srioCoreMailbox[obj->remoteProcId];   /* Mailbox for remote core */
  dstSrioAddr.type11.letter = TransportSrio_srioCoreLetter[obj->remoteProcId];  /* Letter for remote core */

  /* Send the data out from the producer core to the consumer core. */
  status = Srio_sockSend (TransportSrio_module->srioSocketHandle, (Srio_DrvBuffer)pHostDesc, 
                                         TransportSrio_descriptorSize, &dstSrioAddr);
  Assert_isTrue ((status >= 0), TransportSrio_A_srioSocketSendError);    

  /* Failure when sending */
  if (status < 0)
  {
    /* Free the allocated messageQ msg and descriptor prior to returning. */
    status = MessageQ_free(msg);
    Assert_isTrue ((status >= 0), TransportSrio_A_srioMsgQFreeError);

    /* Push the descriptor into the return queue. */
    Qmss_queuePushDescSize ((Qmss_QueueHnd) TransportSrio_module->srioTxFreeQ, (Ptr)pHostDesc, sizeof(Cppi_HostDesc));
    return (FALSE);
  }

  /* Check the error queue to clean up any dropped packet descriptors. */
  if (Qmss_getQueueEntryCount (txErrorQ))
  {
    while ((pHostDesc = (Cppi_HostDesc *) QMSS_DESC_PTR (Qmss_queuePop (txErrorQ))) != NULL)
    {
      /* Invalidate the descriptor prior to accessing it for the return queue */
      Cache_inv((Ptr) pHostDesc, TransportSrio_descriptorSize, Cache_Type_L1D, TRUE);
      
      /* Push packet onto return queue for cleanup */
      Qmss_queuePushDesc (Qmss_getQueueHandle(Cppi_getReturnQueue(Cppi_DescType_HOST, 
                                                   (Cppi_Desc *)pHostDesc)), (UInt32 *)pHostDesc);
    }
  }

  /* Check tx completion queue to clean up any used descriptors */
  if (Qmss_getQueueEntryCount (txCompQ) >= TransportSrio_numTxDescToCleanUp)
  {
    while ((pHostDesc = (Cppi_HostDesc *) QMSS_DESC_PTR (Qmss_queuePop (txCompQ))) != NULL)
    {
      /* Invalidate the descriptor prior to accessing it for its payload */
      Cache_inv((Ptr) pHostDesc, TransportSrio_descriptorSize, Cache_Type_L1D, TRUE);
      
      Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *) pHostDesc, (UInt8**)&msg,(UInt32*)&msgSize);

      /* Free the MsgQ msg */
      status = MessageQ_free(msg);
      Assert_isTrue ((status >= 0), TransportSrio_A_srioMsgQFreeError);

      /* Push the descriptor into the free queue. */
      Qmss_queuePushDesc ((Qmss_QueueHnd) TransportSrio_module->srioTxFreeQ, (Ptr)pHostDesc);
    }
  }
  
  return (TRUE);
}

/*
 *  ======== TransportSrio_control ========
 */
Bool TransportSrio_control(TransportSrio_Object *obj, UInt cmd,
    UArg cmdArg)
{
    return (FALSE);
}

/*
 *  ======== TransportSrio_getStatus ========
 */
Int TransportSrio_getStatus(TransportSrio_Object *obj)
{
    return (0);
}

UInt32 TransportSrio_initRxDescBufs(void)
{
  UInt16 index;
  Cppi_HostDesc*  pHostDesc;
  MessageQ_Msg pRxMsg;
  Qmss_QueueHnd rxFreeQHdlr = (Qmss_QueueHnd) TransportSrio_module->srioRxFreeQ;
  
  /* Initialize the application receive buffers. */
  for (index = 0; index < TransportSrio_srioNumRxDescriptors; index++)
  {
    /* Pop off a descriptor */
    pHostDesc = (Cppi_HostDesc *)Qmss_queuePop(rxFreeQHdlr);
    if (pHostDesc == NULL)
    {
      return (1);
    }

    /* Allocate MessageQ buffers where the data will be placed by the SRIO CPDMA. */
    pRxMsg = MessageQ_alloc(TransportSrio_module->rxMsgQHeapId, TransportSrio_srioMaxMtuSizeBytes);
    if (pRxMsg == NULL)
    {
      return (1);
    }   

    /* Set the DATA and ORIGINAL DATA in the buffer descriptor. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, (uint8_t*)pRxMsg, TransportSrio_srioMaxMtuSizeBytes);
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, (uint8_t*)pRxMsg, TransportSrio_srioMaxMtuSizeBytes);

    /* Writeback changes to descriptor */
    Cache_wb(pHostDesc, TransportSrio_descriptorSize, Cache_Type_L1D, TRUE);  

    /* Add the packet descriptor back to the Application Receive Free Queue. */
    Qmss_queuePushDesc (rxFreeQHdlr, (uint32_t*)pHostDesc);
  }

  return (0);
}

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*
 *  ======== TransportSrio_setErrFxn ========
 */
Void TransportSrio_setErrFxn(TransportSrio_ErrFxn errFxn)
{
    /* Ignore the errFxn */
}

/*
 *************************************************************************
 *                       Internal functions
 *************************************************************************
 */

/*
 *  ======== TransportSrio_isr ========
 */
Void TransportSrio_isr(UArg arg)
{
  Srio_DrvHandle  hSrioDriver;  
  Cppi_HostDesc *pRxHostDesc;
  Srio_SockAddrInfo srcSocket;
  MessageQ_Msg rxMsg;
  UInt32 rxNumBytes;
  UInt32 queueId;
  
  hSrioDriver = (Srio_DrvHandle) arg;
  /* Make sure the TransportSrio_srioDriverHandle is not NULL */
  Assert_isTrue(hSrioDriver != NULL, TransportSrio_A_srioDrvHandleError);

  /* Make sure the high priority list is not NULL */
  Assert_isTrue(TransportSrio_module->hiPrioList != NULL, TransportSrio_A_srioAccumulatorListNULL);

  /* Invoke the Srio ISR task to handle the received data.  This will handle ISR ACK */ 
  Srio_rxCompletionIsr (hSrioDriver);

  /* Get all data received from the socket */
  while(Srio_sockRecv (TransportSrio_module->srioSocketHandle, (Srio_DrvBuffer*) &pRxHostDesc, &srcSocket))
  {
    /* Get the received messageQ msg. */
	Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *) pRxHostDesc, (UInt8**)&rxMsg,(UInt32*)&rxNumBytes);

    /* Invalidate the MessageQ msg prior to accessing it for the queueId */
    Cache_inv(rxMsg, rxNumBytes, Cache_Type_L1D, TRUE);

    /* Reset heapId to rxMsgQHeapId.  This heap is overwritten when the message is received over SRIO */
    rxMsg->heapId = TransportSrio_module->rxMsgQHeapId;
    
    queueId = MessageQ_getDstQueue(rxMsg);
    
    /* Put messageQ buffer on queue */
    MessageQ_put(queueId, rxMsg);
    
    /* Cleanup the received data payload. */
    
    /* Get a new MessageQ buffer to point to be pointed to by the descriptor - Buffer must be 
      * allocated from a HeapBufMP instance.  This should be set up by the application */
    rxMsg = MessageQ_alloc(TransportSrio_module->rxMsgQHeapId, TransportSrio_srioMaxMtuSizeBytes);

    /* Set the DATA and ORIGINAL DATA in the buffer descriptor. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pRxHostDesc, (UInt8 *)rxMsg, 
                            TransportSrio_srioMaxMtuSizeBytes);
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)pRxHostDesc, (UInt8 *)rxMsg, 
                                             TransportSrio_srioMaxMtuSizeBytes);

    /* Writeback changes to descriptor */
    Cache_wb(pRxHostDesc, TransportSrio_descriptorSize, Cache_Type_L1D, TRUE);  

    /* Add the packet descriptor to the Application Receive Free Queue. */
    Qmss_queuePushDesc (Qmss_getQueueHandle(Cppi_getReturnQueue(Cppi_DescType_HOST, 
                                           (Cppi_Desc *)pRxHostDesc)), (UInt32 *)pRxHostDesc);
  }
}

