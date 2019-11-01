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
 *  ======== TransportQmss.xdc ========
 */
import xdc.runtime.Assert;

import ti.sdo.utils.MultiProc;

import xdc.rov.ViewInfo;

/*!
 *  ======== TransportQmss ========
 *  Transport for MessageQ that acts with QMSS.
 *
 */
@InstanceFinalize
@InstanceInitError

module TransportQmss inherits ti.sdo.ipc.interfaces.IMessageQTransport
{
    /*!
     *  ======== A_qmssProcIdInvalid ========
     *  Assert raised when MultiProc ID is invalid.
     */    
    config Assert.Id A_qmssProcIdInvalid  = {
        msg: "A_qmssProcIdInvalid: MultiProc ID is invalid."
    };
    
     /*!
     *  ======== A_qmssError ========
     *  Assert raised when a Qmss returns error.
     */    
    config Assert.Id A_qmssError  = {
        msg: "A_qmssError: Qmss set up error"
    };
    
    /*!
     *  ======== A_qmssAccumulatorListNULL ========
     *  Assert raised when the QMSS accumulator list is NULL.
     */    
    config Assert.Id A_qmssAccumulatorListNULL  = {
        msg: "A_qmssAccumulatorListNULL: Accumulator list is NULL."
    };    

    /*!
     *  ======== A_qmssAccumulatorListSize ========
     *  Assert raised when the QMSS accumulator list is not sized correctly.
     */    
    config Assert.Id A_AccumulatorListSize  = {
        msg: "A_AccumulatorListSize: Accumulator list not sized correctly."
    };    

    /*!
     *  ======== A_qmssIsrObjectNULL ========
     *  Assert raised when the QMSS ISR receives a NULL object.
     */    
    config Assert.Id A_qmssIsrObjectNULL  = {
        msg: "A_qmssIsrObjectNULL: Object received by ISR is NULL."
    };        

   /*!
     *  ======== numDescriptors ========
     *  Number of descriptors created for use by QMSS. 
     */
    config UInt numDescriptors = 32;

    /*!
      *  ======== descriptorSize ========
      *  Size of the descriptors used by QMSS in bytes.
      */
    config UInt descriptorSize = 32;    
    
    /*!
      *  ======== descriptorIsInSharedMem ========
      *  If descriptor memory region is in shared memory (MSMC, DDR3) this flag needs to 
      *  set to true. Otherwise if descriptor memory region is in L2, set it to false.
      */
    config UInt descriptorIsInSharedMem = true;

   /*!
     *  ======== useAccumulatorLogic ========
     *  The QMSS transport will use the accumulator to retrieve and process descriptors if 
     *  set to true.  If set to false the QMSS transport will use QPEND queues to store
     *  descriptors and inform cores of descriptors pushed on the queues.
     */
    config UInt useAccumulatorLogic = false;

    /*!
     *  ======== intThreshold ========    
     *  accumulator entries before interrupt fires.  Only used in accumulator mode.
     */
    config UInt intThreshold = 1;

    /*!
     *  ======== accuHiPriListSize ========
     *  number of  ping & pong items in high priority accumulator list.  Only used in 
     *  accumulator mode.
     */
    config UInt accuHiPriListSize = 32;
    
    /*!
     *  ======== pacingEnabled  ========
     *  Enable interrupt pacing for high priority queue.  Only used in accumulator mode.
     */
    config UInt pacingEnabled = true;

    /*!
     *  ======== timerLoadCount  ========
     *  Timer tick delay since last interrupt.  Only used in accumulator mode.
     */
    config UInt timerLoadCount = 1;

    /*!
     *  ======== intcGemEvent  ========
     *  GEM event to be tied to interrupt controller host interrupt.  Only used in QPEND mode.
     */
    config UInt intcGemEvent = 92;    
    
instance:

    /*!
     *  ======== cacheEnabled ========
     *  Whether cache operations will be performed
     *
     *  If it is known that no cache operations are needed for this instance
     *  set this flag to FALSE.  If {@link #sharedAddr} lies within a shared
     *  region and the cache enabled setting for the region is FALSE,
     *  then the value specified here will be overriden to FALSE.
     */
    config Bool cacheEnabled = true;

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

internal:

    /*!
      *  ======== Accumulator_isr ========
      *  ISR invoked when the QMSS transport is configured to use the 
      *  QMSS accumulator.  The accumulator ISR retrieves received descriptors
      *  from the accumulator ping-pong list.
      */
    Void Accumulator_isr(UArg arg);

    /*!
      *  ======== Qpend_isr ========
      *  ISR invoked when the QMSS transport is configured to use the 
      *  high priority QPEND queues.  The QPEND ISR retrieves received descriptors
      *  from the QPEND queues directly.
      */
    Void Qpend_isr(UArg arg);

    struct Module_State 
    {
        UInt32 *rxQueueId;        
        UInt32 *hiPrioList;
        UInt  qmssInitialized;
        UInt32  usePingList;
        UInt32 *cpIntSystemInt;
    }

    /* Instance State object */
    struct Instance_State 
    {
        UInt16          remoteProcId;  /* dst proc id                   */
        UInt16          priority;      /* priority to register          */
        UInt32          intVectId;  /* Interrupt vector ID used for this core */
        Bool             cacheEnabled;   /* set by param or SharedRegion     */
        UInt             memRegion; /* Memory region where descriptors are placed */
    }
}

