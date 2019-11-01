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
 *  ======== TransportQmss.xs ========
 */

var TransportQmss = null;
var MultiProc        = null;
var MessageQ     = null;
var Ipc              = null;
var Memory           = null;
var SharedRegion     = null;
var Timestamp        = null;
var CpIntc = null;

/*
 *  ======== module$use ========
 */
function module$use()
{
    MessageQ     = xdc.useModule("ti.sdo.ipc.MessageQ");
    Ipc             = xdc.useModule("ti.sdo.ipc.Ipc");
    SharedRegion    = xdc.useModule("ti.sdo.ipc.SharedRegion");
    Cache           = xdc.useModule("ti.sysbios.hal.Cache");
    Timestamp = xdc.useModule("xdc.runtime.Timestamp");
    CpIntc = xdc.useModule("ti.sysbios.family.c66.tci66xx.CpIntc");
}

/*
 *  ======== module$static$init ========
 */
function module$static$init(mod, params)
{
  /* Init QMSS Transport params */
  mod.rxQueueId = null;    
  mod.hiPrioList = null;
  mod.qmssInitialized = 0;
  mod.usePingList = 1;
  mod.cpIntSystemInt = null;  
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

  TransportQmss = this;
}

