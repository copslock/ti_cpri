/* 
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
 * */
/*
 *  ======== TransportSrio.xs ========
 */

var TransportSrio = null;
var MultiProc        = null;
var MessageQ     = null;
var Ipc              = null;
var Memory           = null;
var Timestamp        = null;

/*
 *  ======== module$use ========
 */
function module$use()
{
    MessageQ     = xdc.useModule("ti.sdo.ipc.MessageQ");
    Ipc             = xdc.useModule("ti.sdo.ipc.Ipc");
}

/*
 *  ======== module$static$init ========
 */
function module$static$init(mod, params)
{
  /* Init SRIO Transport params */
  mod.hiPrioList = null;
  mod.srioDriverHandle = null;
  mod.srioDriverCreated = 0;
  mod.srioSocketHandle = null;
  mod.srioTxCompQ = null;
  mod.srioTxFreeQ = null;
  mod.srioRxFreeQ = null;
  mod.srioRxQ = null;
  mod.rxMsgQHeapId = 0;
}

/*
 *  ======== module$meta$init ========
 */
function module$meta$init()
{
  /* Only process during "cfg" phase */
  if (xdc.om.$name != "cfg") 
  {
    return;
  }

  TransportSrio = this;

  TransportSrio.srioMaxNumSystemCores = 2;

  TransportSrio.srioCoreTT.length = TransportSrio.srioMaxNumSystemCores;
  TransportSrio.srioCoreTT[0] = 1;
  TransportSrio.srioCoreTT[1] = 1;

  TransportSrio.srioCoreDeviceId.length = TransportSrio.srioMaxNumSystemCores;
  TransportSrio.srioCoreDeviceId[0] = 0xBEEF;
  TransportSrio.srioCoreDeviceId[1] = 0xBEEF;

  TransportSrio.srioCoreMailbox.length = TransportSrio.srioMaxNumSystemCores;
  TransportSrio.srioCoreMailbox[0] = 0;
  TransportSrio.srioCoreMailbox[1] = 0;

  TransportSrio.srioCoreLetter.length = TransportSrio.srioMaxNumSystemCores;
  TransportSrio.srioCoreLetter[0] = 0;
  TransportSrio.srioCoreLetter[1] = 1;

  TransportSrio.srioCoreSegMap.length = TransportSrio.srioMaxNumSystemCores;
  TransportSrio.srioCoreSegMap[0] = 0;
  TransportSrio.srioCoreSegMap[1] = 0;
}

