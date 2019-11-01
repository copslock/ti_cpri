/**
 *   @file  rm_services.c
 *
 *   @brief   
 *      This is the Resource Manager services source.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2016, Texas Instruments, Inc.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  \par
*/

/* RM external API includes */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>

/* RM internal API includes */
#include <ti/drv/rm/include/rm_loc.h>
#include <ti/drv/rm/include/rm_internal.h>

/* RM OSAL layer */
#include <rm_osal.h>

/**********************************************************************
 ********************** Internal Functions ****************************
 **********************************************************************/

/* FUNCTION PURPOSE: Internal Callback to unblock RM instance
 ***********************************************************************
 * DESCRIPTION: Internal callback function executed when the result
 *              of a service request has been received from a remote
 *              instance.  The original service request did not specify
 *              a callback function so the Rm_serviceHandler is blocked
 *              waiting for the response.  This function unblocks the
 *              Rm_serviceHandler to return the response to the 
 *              application.
 */
void rmServiceInternalCallback(Rm_Handle rmHandle)
{
    Rm_Inst *rmInst = (Rm_Inst *)rmHandle;

    /* Unblock so Rm_serviceHandler can provide response to application */
    Rm_osalTaskUnblock(rmInst->blockHandle);
}

/**********************************************************************
 ********************** Application visible APIs **********************
 **********************************************************************/

/* FUNCTION PURPOSE: Handles application component service requests
 ***********************************************************************
 * DESCRIPTION: Receives service requests from application components
 *              and routes them to the transaction processor.  If
 *              the service can be handled immediately the response
 *              will be provided in the service response.  If the
 *              service requires a blocking operation the handler
 *              will provide a service ID back to the application.
 *              The response will be sent at a later time via the
 *              application supplied callback function.
 */
void Rm_serviceHandler (void *rmHandle, const Rm_ServiceReqInfo *serviceRequest,
                        Rm_ServiceRespInfo *serviceResponse)
{
    Rm_Inst        *rmInst = (Rm_Inst *)rmHandle;
    char           *instanceName;
    Rm_Transaction *transaction;
    void           *key;
    void           *mtKey;

    if (serviceRequest->type >= Rm_service_LAST) {
        serviceResponse->serviceState = RM_ERROR_INVALID_SERVICE_TYPE;
        return;
    }

    if (serviceRequest->resourceName) {
        if ((strlen(serviceRequest->resourceName) + 1) > RM_NAME_MAX_CHARS) {
            serviceResponse->serviceState = RM_ERROR_RESOURCE_NAME_TOO_LONG;
            return;
        }
    }

    if (serviceRequest->resourceNsName) {
        if ((strlen(serviceRequest->resourceNsName) + 1) > RM_NAME_MAX_CHARS) {
            serviceResponse->serviceState = RM_ERROR_NAMESERVER_NAME_TOO_LONG;
            return;
        }
    }
    
    if (rmInst->isLocked) {
        serviceResponse->serviceState = RM_SERVICE_DENIED_RM_INSTANCE_LOCKED;
        return;
    }

    RM_SS_INST_INV_ENTER_CS(rmInst, key);
    RM_SC_INST_INV_ENTER_CS(rmInst, key);

    if (rmInst->mtSemObj) {
        mtKey = Rm_osalMtCsEnter(rmInst->mtSemObj);
    }

    /* Copy location of instance name to local variable in case Shared Client
     * needs to transfer control to a shared server */
    instanceName = rmInst->instName;
    if (rmInst->instType == Rm_instType_SHARED_CLIENT) {
        /* Transfer control to shared server instance */
        rmInst = rmInst->u.sharedClient.sharedServerHandle;
    }

    if ((transaction = rmTransactionQueueAdd(rmInst))) {
        transaction->type = serviceRequest->type;
        rm_strncpy(transaction->serviceSrcInstName, instanceName,
                   RM_NAME_MAX_CHARS);
        transaction->u.callback.serviceCallback = serviceRequest->callback.serviceCallback;
        transaction->state = RM_SERVICE_PROCESSING;
        if (serviceRequest->resourceName) {
            rm_strncpy(transaction->resourceInfo.name,
                       serviceRequest->resourceName, RM_NAME_MAX_CHARS);
        }
        transaction->resourceInfo.base = serviceRequest->resourceBase;
        transaction->resourceInfo.length = serviceRequest->resourceLength;
        transaction->resourceInfo.alignment = serviceRequest->resourceAlignment;
        transaction->resourceInfo.ownerCount = RM_RESOURCE_NUM_OWNERS_INVALID;
        transaction->resourceInfo.instAllocCount = RM_INST_ALLOC_COUNT_INVALID;
        if (serviceRequest->resourceNsName) {
            rm_strncpy(transaction->resourceInfo.nameServerName,
                       serviceRequest->resourceNsName, RM_NAME_MAX_CHARS);
        }

        /* Process received transaction */
        rmProcessRouter(rmInst, transaction);

        memset((void *)serviceResponse, 0, sizeof(*serviceResponse));

        if ((rmInst->instType == Rm_instType_SHARED_SERVER) && 
            (transaction->state == RM_SERVICE_PROCESSING)) {
            /* Shared Server should always return a fully processed
             * transaction */
            serviceResponse->serviceState = RM_ERROR_SHARED_INSTANCE_UNFINISHED_REQ;
            rmTransactionQueueDelete(rmInst, transaction->localId);
        } else {
            if ((transaction->state == RM_SERVICE_PROCESSING) &&
                (transaction->u.callback.serviceCallback == NULL)) {
                /* Block until response is received.  Response will be
                 * received in transaction. */
                Rm_osalTaskBlock(rmInst->blockHandle);
            }

            serviceResponse->rmHandle = rmHandle;
            serviceResponse->serviceState = transaction->state;
            /* Owner and instance allocation count will only be set within RM
             * under certain circumstances. */
            serviceResponse->resourceNumOwners = transaction->resourceInfo.ownerCount;
            serviceResponse->instAllocCount = transaction->resourceInfo.instAllocCount;
            if ((serviceResponse->serviceState == RM_SERVICE_PROCESSING) ||
                (serviceResponse->serviceState == RM_SERVICE_APPROVED_STATIC) ||
                (serviceResponse->serviceState ==
                 RM_SERVICE_PENDING_SERVER_RESPONSE)) {
                /* Service still being processed.  Static requests will have
                 * their validation responses sent once all transports have
                 * been established.  Provide transaction ID back to component
                 * so it can sort service responses received via callback
                 * function */
                serviceResponse->serviceId = transaction->localId;
            }

            if ((serviceResponse->serviceState == RM_SERVICE_APPROVED) ||
                (serviceResponse->serviceState == RM_SERVICE_APPROVED_STATIC)) {
                rm_strncpy(serviceResponse->resourceName,
                           transaction->resourceInfo.name, RM_NAME_MAX_CHARS);
                serviceResponse->resourceBase = transaction->resourceInfo.base;
                serviceResponse->resourceLength = transaction->resourceInfo.length;
            }

            /* Transactions still processing not deleted from queue including
             * static transactions which will be verified once all transports
             * are up */
            if ((serviceResponse->serviceState != RM_SERVICE_PROCESSING) &&
                (serviceResponse->serviceState != RM_SERVICE_APPROVED_STATIC) &&
                (serviceResponse->serviceState !=
                 RM_SERVICE_PENDING_SERVER_RESPONSE)) {
                rmTransactionQueueDelete(rmInst, transaction->localId);
            }
        }
    } else {
        serviceResponse->serviceState = RM_ERROR_SERVICE_TRANS_NOT_CREATED;
    }

    /* Free sem object using originating instance in case the Shared Client to
     * Shared Server instance switch took place */
    if (((Rm_Inst *)rmHandle)->mtSemObj) {
        Rm_osalMtCsExit(((Rm_Inst *)rmHandle)->mtSemObj, mtKey);
    }
    /* Shared Client switches to Shared Server instance so only need
     * SS_EXIT_CS macro */
    RM_SS_INST_WB_EXIT_CS(rmInst, key);
    return;
}

/* FUNCTION PURPOSE: Opens the RM instance service handle
 ***********************************************************************
 * DESCRIPTION: Returns the service handle for an RM instance.  Only
 *              one service handle is opened per instance.
 */
Rm_ServiceHandle *Rm_serviceOpenHandle(Rm_Handle rmHandle, int32_t *result)
{
    Rm_Inst          *rmInst = (Rm_Inst *)rmHandle;
    Rm_ServiceHandle *serviceHandle = NULL;
    void             *key;
    void             *mtKey;
    
    RM_SS_INST_INV_ENTER_CS(rmInst, key);
    if (rmInst->mtSemObj) {
        mtKey = Rm_osalMtCsEnter(rmInst->mtSemObj);
    }

    *result = RM_OK;

    serviceHandle = rmInst->serviceHandle;
    if (serviceHandle == NULL) {
        serviceHandle = Rm_osalMalloc(sizeof(*serviceHandle));
        if (serviceHandle) {
            serviceHandle->rmHandle = rmHandle;
            serviceHandle->Rm_serviceHandler = Rm_serviceHandler;
            RM_SS_OBJ_WB(rmInst, serviceHandle, Rm_ServiceHandle);
            rmInst->serviceHandle = serviceHandle;
        } else {
            *result = RM_ERROR_SERVICE_HANDLE_MEM_ALLOC_FAILED;
        }
    }

    if (rmInst->mtSemObj) {
        Rm_osalMtCsExit(rmInst->mtSemObj, mtKey);
    }
    RM_SS_INST_WB_EXIT_CS(rmInst, key);
    return (serviceHandle);
}

/* FUNCTION PURPOSE: Closes the RM instance service handle
 ***********************************************************************
 * DESCRIPTION: Closes the service handle for an RM instance.
 */
int32_t Rm_serviceCloseHandle(Rm_ServiceHandle *rmServiceHandle)
{
    Rm_Inst *rmInst = (Rm_Inst *)rmServiceHandle->rmHandle;
    int32_t  retVal = RM_OK;
    void    *key;
    void    *mtKey;

    RM_SS_INST_INV_ENTER_CS(rmInst, key);
    if (rmInst->mtSemObj) {
        mtKey = Rm_osalMtCsEnter(rmInst->mtSemObj);
    }

    if (rmInst->serviceHandle) {
        Rm_osalFree((void *)rmServiceHandle, sizeof(*rmServiceHandle));
        rmInst->serviceHandle = NULL;
    } else {
        retVal = RM_ERROR_SERVICE_HANDLE_ALREADY_CLOSED;
    }

    if (rmInst->mtSemObj) {
        Rm_osalMtCsExit(rmInst->mtSemObj, mtKey);
    }
    RM_SS_INST_WB_EXIT_CS(rmInst, key);
    return(retVal);
}

