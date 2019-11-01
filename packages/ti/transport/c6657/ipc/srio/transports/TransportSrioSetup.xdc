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
 *  ======== TransportSrioSetup.xdc ========
 */

/*!
 *  ======== TransportSrioSetup ========
 *  Manages the setup of TransportSrio instances.
 *
 *  create or open the TransportSrio for each pair of devices.
 */
@ModuleStartup

module TransportSrioSetup inherits ti.sdo.ipc.interfaces.ITransportSetup
{

    /* The interrupt vector id */
    config UInt dspIntVectId = 8;

    /* The memory region to be used for the descriptors: default is zero but can be reassigned in app */
    config UInt descMemRegion = 0;

    /* The number of buffers to reserve for receive side descriptors.  Should be at least twice as large as
      * number of receive-side descriptors * number of cores on chip.  This at least allows one packet to 
      * be worked on by app while another packet is received. */
    config UInt16 numRxDescBuffs = 16;

    /* The heap ID used by MessageQ: default is zero but can be reassigned in the app */
    config UInt16 messageQHeapId = 1;

internal:
    
    Int setupRxDescBufs(UArg arg, UInt16 input);
    
    /* Module Status object */
    struct Module_State {
        TransportSrio.Handle handles[]; /* handle per remote proc */
        UInt32 *reservedMemAddr; /* pointer to start of reserved memory in shared region */
    }
}

