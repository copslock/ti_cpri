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
 *  ======== TransportSrioSetup.c ========
 */

/* XDC includes */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Startup.h>
#include <xdc/runtime/IHeap.h>

/* IPC external includes */
#include <ti/ipc/HeapBufMP.h>
#include <ti/ipc/GateMP.h>
#include <ti/ipc/MultiProc.h>

/* IPC internal includes */
#include <ti/sdo/ipc/_MessageQ.h>
#include <ti/sdo/ipc/_SharedRegion.h>

#include "package/internal/TransportSrioSetup.xdc.h"

#include <ti/transport/ipc/srio/transports/TransportSrio.h>

/*
 *  ======== TransportSrioSetup_attach ========
 */
Int TransportSrioSetup_attach(UInt16 remoteProcId, Ptr sharedAddr)
{
    TransportSrio_Handle handle;
    TransportSrio_Params transportSrioParams;
    Int status = MessageQ_E_FAIL;
    Error_Block eb;

    Error_init(&eb);

    /* init params and set default values */
    TransportSrio_Params_init(&transportSrioParams);
    transportSrioParams.intVectorId     = TransportSrioSetup_dspIntVectId;
    transportSrioParams.descMemRegion      = TransportSrioSetup_descMemRegion;
    transportSrioParams.rxMessageQHeapId      = TransportSrioSetup_messageQHeapId;
    transportSrioParams.remoteProcId  = remoteProcId;

    handle = TransportSrio_create(remoteProcId, &transportSrioParams, &eb);
    
    if (handle != NULL) 
    {
      TransportSrioSetup_module->handles[remoteProcId] = handle;
      status = MessageQ_S_SUCCESS;
    }

    return (status);
}

/*
 *  ======== TransportSrioSetup_detach ========
 */
Int TransportSrioSetup_detach(UInt16 remoteProcId)
{
    TransportSrio_Handle handle;

    handle = TransportSrioSetup_module->handles[remoteProcId];

    /* Trying to detach an un-attached processor should fail */
    if (handle == NULL) {
        return (MessageQ_E_FAIL);
    }

    /* Unregister the instance */
    TransportSrioSetup_module->handles[remoteProcId] = NULL;

    TransportSrio_delete(&handle);

    return (MessageQ_S_SUCCESS);
}

/*
 *  ======== TransportSrioSetup_isRegistered ========
 */
Bool TransportSrioSetup_isRegistered(UInt16 remoteProcId)
{
    Bool registered;

    registered = (TransportSrioSetup_module->handles[remoteProcId] != NULL);

    return (registered);
}

/*
 *  ======== TransportSrioSetup_sharedMemReq ========
 */
SizeT TransportSrioSetup_sharedMemReq(Ptr sharedAddr)
{
    return (0);
}

/*
 *  ======== TransportSrioSetup_Module_startup ========
 */
Int TransportSrioSetup_Module_startup (Int phase)
{
  UInt32 reservationSize;
  HeapBufMP_Params heapBufParams;
  
  HeapBufMP_Params_init(&heapBufParams);
  heapBufParams.regionId = TransportSrioSetup_descMemRegion;
  /* Allocate space for at least two buffers per descriptor.  Allowing one buffer to be processed while the other is linked
    * to the descriptor for the next received packet */
  heapBufParams.numBlocks = TransportSrioSetup_numRxDescBuffs;
  heapBufParams.blockSize = TransportSrio_srioMaxMtuSizeBytes;

  reservationSize = HeapBufMP_sharedMemReq(&heapBufParams);
  TransportSrioSetup_module ->reservedMemAddr = (UInt32 *) ti_sdo_ipc_SharedRegion_reserveMemory(TransportSrioSetup_descMemRegion, 
                                                                                                                                                 reservationSize);
  return (Startup_DONE);
}

/*
 *  ======== TransportSrioSetup_setupRxDescBufs ========
 */
Int TransportSrioSetup_setupRxDescBufs (UArg arg, UInt16 input)
{
  SharedRegion_Entry entry;
  GateMP_Handle gateMpHandle;
  GateMP_Params gateMpParams;  
  HeapBufMP_Handle heapHandle;
  HeapBufMP_Params heapBufParams;
  
  SharedRegion_getEntry(TransportSrioSetup_descMemRegion, &entry);

  /* Check if owner of memory region */
  if (entry.ownerProcId == MultiProc_self())
  {
    GateMP_Params_init (&gateMpParams);
    gateMpParams.localProtect = GateMP_LocalProtect_INTERRUPT;
    gateMpHandle = GateMP_create (&gateMpParams);
    
    HeapBufMP_Params_init(&heapBufParams);
    heapBufParams.sharedAddr = TransportSrioSetup_module ->reservedMemAddr;
    /* Should be same as what was defined in TransportSrioSetup_Module_Startup */
    heapBufParams.numBlocks = TransportSrioSetup_numRxDescBuffs;
    heapBufParams.blockSize = TransportSrio_srioMaxMtuSizeBytes;
    heapBufParams.gate = gateMpHandle;
    heapHandle = HeapBufMP_create(&heapBufParams);
  }
  else
  {
    /* Open the heap created by the other processor. Loop until opened. */
   HeapBufMP_openByAddr(TransportSrioSetup_module ->reservedMemAddr, &heapHandle);
  }

  /* Register this heap with MessageQ */
  MessageQ_registerHeap((IHeap_Handle)heapHandle, TransportSrioSetup_messageQHeapId);

  /* Initialize the receive side descriptors with MsgQ msgs */
  TransportSrio_initRxDescBufs();

  return (0);
}

