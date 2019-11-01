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
 *  ======== TransportQmssSetup.c ========
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>

#include <ti/transport/ipc/qmss/transports/TransportQmss.h>

#include "package/internal/TransportQmssSetup.xdc.h"

#include <ti/sdo/ipc/_MessageQ.h>
#include <ti/sdo/utils/_MultiProc.h>

/*
 *  ======== TransportQmssSetup_attach ========
 */
Int TransportQmssSetup_attach(UInt16 remoteProcId, Ptr sharedAddr)
{
    TransportQmss_Handle handle;
    TransportQmss_Params transportQmssParams;
    Int status = MessageQ_E_FAIL;
    Error_Block eb;

    /* Initialize the error block */
    Error_init(&eb);

    /* init params and set default values */
    TransportQmss_Params_init(&transportQmssParams);
    transportQmssParams.intVectorId     = TransportQmssSetup_dspIntVectId;
    transportQmssParams.descMemRegion      = TransportQmssSetup_descMemRegion;
    transportQmssParams.remoteProcId  = remoteProcId;
    transportQmssParams.cacheEnabled = TransportQmssSetup_cacheEnabled;

    handle = TransportQmss_create(remoteProcId, &transportQmssParams, &eb);
    
    if (handle != NULL) 
    {
      TransportQmssSetup_module->handles[remoteProcId] = handle;
      status = MessageQ_S_SUCCESS;
    }

    return (status);
}

/*
 *  ======== TransportQmssSetup_detach ========
 */
Int TransportQmssSetup_detach(UInt16 remoteProcId)
{
    TransportQmss_Handle handle;

    handle = TransportQmssSetup_module->handles[remoteProcId];

    /* Trying to detach an un-attached processor should fail */
    if (handle == NULL) {
        return (MessageQ_E_FAIL);
    }

    /* Unregister the instance */
    TransportQmssSetup_module->handles[remoteProcId] = NULL;

    TransportQmss_delete(&handle);

    return (MessageQ_S_SUCCESS);
}

/*
 *  ======== TransportQmssSetup_isRegistered ========
 */
Bool TransportQmssSetup_isRegistered(UInt16 remoteProcId)
{
    Bool registered;

    registered = (TransportQmssSetup_module->handles[remoteProcId] != NULL);

    return (registered);
}

/*
 *  ======== TransportQmssSetup_sharedMemReq ========
 */
SizeT TransportQmssSetup_sharedMemReq(Ptr sharedAddr)
{
    return (0);
}

