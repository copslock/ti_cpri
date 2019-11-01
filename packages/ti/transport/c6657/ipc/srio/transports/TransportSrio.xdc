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
 *  ======== TransportSrio.xdc ========
 */
import xdc.runtime.Assert;

import ti.sdo.utils.MultiProc;

import xdc.rov.ViewInfo;

/*!
 *  ======== TransportSrio ========
 *  Transport for MessageQ that acts with SRIO.
 *
 *  TransportShmSingle is a simplified version of {@link TransportShm}.  This
 *  transport uses the Notify module to do all the real work.  The 'put'
 *  function passes the message to the other processor using the 'payload'
 *  parameter to Notify_sendEvent().  The receive side simply casts this
 *  payload parameter to a MessageQ_Msg and enqueues it locally.
 *
 *  CAVEATS:
 *
 *  The sender will spin in Notify_sendEvent until the receive side has
 *  read the previous message.  This is Notify-driver specific.
 *  NotifyDriverShm  will spin before sending a new event if the prior event
 *  hasn't been handled.  Some Notify drivers may support a FIFO or queue
 *  for these events to mitigate this busy-wait affect.
 */
@InstanceFinalize
@InstanceInitError

module TransportSrio inherits ti.sdo.ipc.interfaces.IMessageQTransport
{
    /*!
     *  ======== A_srioProcIdInvalid ========
     *  Assert raised when MultiProc ID is invalid.
     */    
    config Assert.Id A_srioProcIdInvalid  = {
        msg: "A_srioProcIdInvalid: MultiProc ID is invalid."
    };

    /*!
     *  ======== A_srioRxCompQError ========
     *  Assert raised when SRIO returns error for RX completion queue.
     */    
    config Assert.Id A_srioRxCompQError  = {
        msg: "A_srioRxCompQError: Unable to open the SRIO Receive Completion Queue."
    };

    /*!
     *  ======== A_srioAccumulatorListSize ========
     *  Assert raised when the SRIO accumulator list is not sized correctly.
     */    
    config Assert.Id A_srioAccumulatorListSize  = {
        msg: "A_srioAccumulatorListSize: Accumulator list not sized correctly."
    };

    /*!
     *  ======== A_srioAccumulatorListNULL ========
     *  Assert raised when the SRIO accumulator list is NULL.
     */    
    config Assert.Id A_srioAccumulatorListNULL  = {
        msg: "A_srioAccumulatorListNULL: Accumulator list is NULL."
    };    
    
    /*!
     *  ======== A_srioDriverError ========
     *  Assert raised when the SRIO driver returns NULL.
     */    
    config Assert.Id A_srioDriverError  = {
        msg: "A_srioDriverError: SRIO Driver Start Failed."
    };

    /*!
     *  ======== A_srioSocketError ========
     *  Assert raised when the SRIO socket returns NULL.
     */    
    config Assert.Id A_srioSocketError  = {
        msg: "A_srioSocketError: Unable to open SRIO socket."
    };

    /*!
     *  ======== A_srioSocketBindError ========
     *  Assert raised when the SRIO socket bind operation returns an error.
     */    
    config Assert.Id A_srioSocketBindError  = {
        msg: "A_srioSocketBindError: SRIO socket bind operation failed."
    };

    /*!
     *  ======== A_srioSocketOptionError ========
     *  Assert raised when the SRIO socket option operation returns an error.
     */    
    config Assert.Id A_srioSocketOptionError  = {
        msg: "A_srioSocketOptionError: SRIO socket option operation failed."
    };

    /*!
     *  ======== A_srioSocketCloseError ========
     *  Assert raised when the SRIO socket close operation returns an error.
     */    
    config Assert.Id A_srioSocketCloseError  = {
        msg: "A_srioSocketCloseError: SRIO socket close operation failed."
    };

    /*!
     *  ======== A_srioQueueCloseError ========
     *  Assert raised when the SRIO queue close operation returns an error.
     */    
    config Assert.Id A_srioQueueCloseError  = {
        msg: "A_srioQueueCloseError: SRIO queue close operation failed."
    };    

    /*!
     *  ======== A_srioAllocTxBufPtrError ========
     *  Assert raised when the SRIO allocTransmitBuffer operation fails receiving a NULL pointer
     */    
    config Assert.Id A_srioAllocTxBufPtrError  = {
        msg: "A_srioAllocTxBufPtrError: SRIO TX buffer allocation received a NULL pointer."
    };

    /*!
     *  ======== A_srioAllocTxBufLenError ========
     *  Assert raised when the SRIO allocTransmitBuffer operation fails allocating no length in buffer
     */    
    config Assert.Id A_srioAllocTxBufLenError  = {
        msg: "A_srioAllocTxBufLenError: SRIO TX buffer allocation received a buffer of zero length"
    };

    /*!
     *  ======== A_srioMsgQFreeError ========
     *  Assert raised when the MessageQ free fails in SRIO transport put
     */    
    config Assert.Id A_srioMsgQFreeError  = {
        msg: "A_srioMsgQFreeError: MessageQ free failed"
    };

    /*!
     *  ======== A_srioSocketSendError ========
     *  Assert raised when the SRIO socket send operation returns an error.
     */    
    config Assert.Id A_srioSocketSendError  = {
        msg: "A_srioSocketSendError: SRIO socket send operation failed."
    };

    /*!
     *  ======== A_srioDrvHandleError ========
     *  Assert raised when the SRIO driver handle is NULL.
     */    
    config Assert.Id A_srioDrvHandleError  = {
        msg: "A_srioDrvHandleError: SRIO driver handle is NULL."
    };    

    /*!
     *  ======== initRxDescBufs ========
     *  Initialize the Receive side descriptor buffers to MessageQ msg buffers
     */
    UInt32 initRxDescBufs();
    
    /*!
     *  ======== sharedMemReq ========
     *  Amount of shared memory required for creation of each instance
     *
     *  @param(params)      Pointer to parameters that will be used in the
     *                      create
     *
     *  @a(returns)         Number of MAUs in shared memory needed to create 
     *                      the instance.
     */
    SizeT sharedMemReq(const Params *params);

   /*!
     *  accumulator entries before interrupt fires
     */
    config UInt intThreshold = 1;

   /*!
     *  ======== accuHiPriListSize ========
     *  number of  ping & pong items in high priority accumulator list 
     */
    config UInt accuHiPriListSize = 32;

    /*!
     *  ======== pacing enabled  ========
     *  Enable interrupt pacing for high priority queue.
     */
    config UInt pacingEnabled = true;

    /*!
     *  ======== pacing enabled  ========
     *  Timer tick delay since last interrupt.
     */
    config UInt timerLoadCount = 0;

    /*!
     *  ======== srioMaxMtuSizeBytes ========
     *  SRIO maximum transmissable unit in bytes
     */
    config UInt32 srioMaxMtuSizeBytes = 256;

    /*!
     *  ======== srioNumTxDescriptors ========
     *  Number of Transmit Descriptors to allocate for SRIO
     */
    config UInt32 srioNumTxDescriptors = 4;

    /*!
     *  ======== srioNumRxDescriptors ========
     *  Number of Receive Descriptors to allocate for SRIO
     */
    config UInt32 srioNumRxDescriptors = 4;

    /*!
     *  ======== numTxDescToCleanUp ========
     *  Number of Transmit Descriptors to clean up in the TransportSrio_put function
     *  This number should be modified with care it should be no greater 
     *  than (srioNumTxDescriptors - 1)
     */
    config UInt32 numTxDescToCleanUp = 1;

    /*!
      *  ======== descriptorSize ========
      *  Size of the descriptors used by SRIO in bytes.
      */
    config UInt descriptorSize = 32;    

   /*!
     *  ======== srioGarbageQueue ========
     *  The SRIO queue where packets dropped by SRIO are put.  There are six different
     *  garbage types.  The SRIO transport supports tying one Q to all types for recycling of
     *  dropped packets.  If specific error handling is desired for the individual garbage types
     *  the application should implement this.
     */
    config UInt32 srioGarbageQ = 905;
    
    /* -------- Begin SRIO Core Definitions -------- */
    /* The maximum number of cores supported in the system (across all chips)
     *  is defined.  A TT, DeviceId, Mailbox, Letter, and SegMap value must be specified for
     *  each core.  The DeviceID, Mailbox, Letter combination must be unique for each core.
     *  Each chip will have a local core offset, for the case where its cores are not 0-7 in the
     *  global core map.  When putting and getting cores, a remote core offset will be used 
     *  by the application to index to the destination core's set of cores in the global core map.
     *  All values will be initialized in the TransportSrio.xs file.  The can be assigned new values
     *  in the application's .cfg. 
     */

    /*!
     *  ======== srioMaxNumSystemCores ========
     *  Maximum number of cores supported in the system (all chips).
     */
    config UInt16 srioMaxNumSystemCores;
    
    /*!
     *  ======== srioCoreTT ========
     *  Array containing the transport type of each core's socket
     *  1: 16-bit identifiers
     *  0: 8 -bit identifiers
     *  The array length and values are initialized in TransportSrio.xs
     */
    config UInt16 srioCoreTT[];

    /*!
     *  ======== srioCoreDeviceId ========
     *  Array containing the identifier for each core's socket
     *  The array length and values are initialized in TransportSrio.xs
     */
    config UInt16 srioCoreDeviceId[];

    /*!
     *  ======== srioCoreMailbox ========
     *  Array containing the mailbox number for each core's socket
     *  The array length and values are initialized in TransportSrio.xs
     */
    config UInt16 srioCoreMailbox[];

    /*!
     *  ======== srioCoreLetter ========
     *  Array containing the letter number for each core's socket
     *  Value cannot be greater than 4.
     *  The array length and values are initialized in TransportSrio.xs
     */
    config UInt16 srioCoreLetter[];

    /*!
     *  ======== srioCoreSegMap ========
     *  Array containing the segmentation mapping of each core's socket
     *  1: Multi-segment
     *  0: Single segment
     *  The array length and values are initialized in TransportSrio.xs
     */
    config UInt16 srioCoreSegMap[];

    /* -------- End SRIO Core Definitions -------- */
    
instance:

    /*!
     *  ======== cacheLineSize ========
     *  The cache line size of the shared memory
     *
     *  This value should be configured 
     */
    config SizeT cacheLineSize = 128;
    
    /*!
     *  ======== remoteProcId ========
     *  The MultiProc ID corresponding to the remote processor
     *
     *  This parameter must be set for every device.  The
     *  {@link ti.sdo.utils.MultiProc#getId} call can be used to obtain
     *  a MultiProc id given the remote processor's name.
     */
    config UInt16 remoteProcId = MultiProc.INVALIDID;

    /*!
     *  ======== intVectorId ========
     *  Interrupt vector ID to be used by the driver.
     */
    config UInt intVectorId = ~1u;

    /*!
     *  ======== descMemRegion ========
     *  Descriptor memory region defined by the application
     */
    config UInt descMemRegion = 0;

    /*!
     *  ======== messageQHeapid ========
     *  Receive Message Q Heap ID defined by TransportSrioSetup
     */
    config UInt16 rxMessageQHeapId = 0;

internal:

    /*! 
     *  Plugs the interrupt and executes the callback functions according
     *  to event priority
     */
    Void isr(UArg arg);

    struct Module_State 
    {
        UInt32 *hiPrioList;
        Void *srioDriverHandle;
        UInt32 srioDriverCreated; 
        Void *srioSocketHandle;
        UInt32 *srioTxCompQ;
        UInt32 *srioTxFreeQ;
        UInt32 *srioRxFreeQ;
        UInt32 *srioRxQ;
        UInt16 rxMsgQHeapId;
    }

    /* Instance State object */
    struct Instance_State 
    {
        UInt16          remoteProcId;  /* dst proc id                   */
        UInt16          priority;      /* priority to register          */
        UInt32          intVectId;  /* Interrupt vector ID used for this core */
    }
}

