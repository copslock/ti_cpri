/**
 *   @file  rm.c
 *
 *   @brief   
 *      This is the Resource Manager source.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2015, Texas Instruments, Inc.
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

/* Standard includes */
#include <stdint.h>

/* RM external includes */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rmver.h>
#include <ti/drv/rm/rm_services.h>
#include <ti/drv/rm/rm_transport.h>

/* RM internal includes */
#include <ti/drv/rm/include/rm_internal.h>
#include <ti/drv/rm/include/rm_loc.h>
#include <ti/drv/rm/include/rm_allocatorloc.h>
#include <ti/drv/rm/include/rm_transportloc.h>
#include <ti/drv/rm/include/rm_nameserverloc.h>
#include <ti/drv/rm/include/rm_servicesloc.h>

/* RM LIBFDT includes */
#include <ti/drv/rm/util/libfdt/libfdt.h>

/* RM OSAL layer */
#include <rm_osal.h>

/**********************************************************************
 ************************** Globals ***********************************
 **********************************************************************/

/* Global Variable which describes the RM Version Information */
const char  rmVersionStr[] = RM_VERSION_STR ":" __DATE__  ":" __TIME__;

/**********************************************************************
 ************************ Local Functions *****************************
 **********************************************************************/

/* FUNCTION PURPOSE: Initializes a RM inst's transaction sequence number
 ***********************************************************************
 * DESCRIPTION: The RM instance transaction sequence number can never
 *              have a value of 0 to avoid conflicts with transactions
 *              that have a remoteOriginatingId of 0 (transaction ID
 *              will be used as the remoteOriginatingId for
 *              transactions that are responses to requests).
 */
static uint32_t transactionInitSequenceNum(void)
{
    return (1);
}

/* FUNCTION PURPOSE: Provides a sequence number for new transactions
 ***********************************************************************
 * DESCRIPTION: Returns a sequence number for a new transaction
 *              specific to a RM instance.  Handles rollover of
 *              sequence number.
 */
static uint32_t transactionGetSequenceNum(Rm_Inst *rmInst)
{
    rmInst->transactionSeqNum++;
    if (!rmInst->transactionSeqNum) {
        rmInst->transactionSeqNum++;
    }
    return(rmInst->transactionSeqNum);
}

/* FUNCTION PURPOSE: Creates a resource request packet
 ***********************************************************************
 * DESCRIPTION: Returns a RM packet handle that points to a RM
 *              resource request packet that has been prepared
 *              for sending to another RM instance.  The packet
 *              is allocated via the rmAllocPkt API using the
 *              appTransport handle provided by the application
 */
static Rm_Packet *createResourceReqPkt(Rm_ResourceReqPktType resReqType,
                                       Rm_Transport *dstTrans,
                                       const char *locInstName,
                                       Rm_Transaction *transaction,
                                       Rm_PacketHandle *pktHandle)
{
    Rm_Packet             *rmPkt = NULL;
    Rm_ResourceRequestPkt *resourceReqPkt = NULL;

    rmPkt = dstTrans->callouts.rmAllocPkt(dstTrans->appTransportHandle,
                                          sizeof(Rm_Packet), pktHandle);
    if ((rmPkt == NULL) || (pktHandle == NULL)) {
        transaction->state = RM_ERROR_TRANSPORT_ALLOC_PKT_ERROR;
        goto errorExit;
    }

    rmPkt->pktType = Rm_pktType_RESOURCE_REQUEST;
    resourceReqPkt = (Rm_ResourceRequestPkt *) rmPkt->data;
    resourceReqPkt->requestId       = transaction->localId;
    resourceReqPkt->resourceReqType = resReqType;
    rm_strncpy(resourceReqPkt->pktSrcInstName, locInstName,
               RM_NAME_MAX_CHARS);
    rm_strncpy(resourceReqPkt->serviceSrcInstName,
               transaction->serviceSrcInstName, RM_NAME_MAX_CHARS);
    memcpy((void *)&(resourceReqPkt->resourceInfo),
           (void *)&(transaction->resourceInfo),
           sizeof(Rm_ResourceInfo));

errorExit:
    return(rmPkt);
}

/* FUNCTION PURPOSE: Creates a resource response packet
 ***********************************************************************
 * DESCRIPTION: Returns a RM packet handle that points to a RM
 *              resource response packet that has been prepared
 *              for sending to another RM instance.  The packet
 *              is allocated via the rmAllocPkt API using the
 *              appTransport handle provided by the application
 */
static void createResourceResponsePkt(Rm_Packet *rmPkt,
                                      Rm_Transaction *transaction)
{
    Rm_ResourceResponsePkt *resourceRespPkt = NULL;

    rmPkt->pktType = Rm_pktType_RESOURCE_RESPONSE;
    resourceRespPkt = (Rm_ResourceResponsePkt *)rmPkt->data;
    resourceRespPkt->responseId = transaction->remoteOriginatingId;
    resourceRespPkt->requestState = transaction->state;
    memcpy((void *)&(resourceRespPkt->resourceInfo),
           (void *)&(transaction->resourceInfo),
           sizeof(Rm_ResourceInfo));
}

/* FUNCTION PURPOSE: Creates a NameServer request packet
 ***********************************************************************
 * DESCRIPTION: Returns a RM packet handle that points to a RM
 *              NameServer request packet that has been prepared
 *              for sending to another RM instance.  The packet
 *              is allocated via the rmAllocPkt API using the
 *              appTransport handle provided by the application
 */
static Rm_Packet *createNsRequestPkt(Rm_NsReqPktType nsReqType,
                                     Rm_Transport *dstTrans,
                                     const char *locInstName,
                                     Rm_Transaction *transaction,
                                     Rm_PacketHandle *pktHandle)
{
    Rm_Packet       *rmPkt = NULL;
    Rm_NsRequestPkt *nsReqPkt = NULL;


    rmPkt = dstTrans->callouts.rmAllocPkt(dstTrans->appTransportHandle,
                                          sizeof(Rm_Packet), pktHandle);
    if ((rmPkt == NULL) || (pktHandle == NULL)) {
        transaction->state = RM_ERROR_TRANSPORT_ALLOC_PKT_ERROR;
        goto errorExit;
    }

    rmPkt->pktType = Rm_pktType_NAMESERVER_REQUEST;
    nsReqPkt = (Rm_NsRequestPkt *)rmPkt->data;
    nsReqPkt->requestId     = transaction->localId;
    nsReqPkt->nsRequestType = nsReqType;
    rm_strncpy(nsReqPkt->pktSrcInstName, locInstName, RM_NAME_MAX_CHARS);
    rm_strncpy(nsReqPkt->serviceSrcInstName, transaction->serviceSrcInstName,
               RM_NAME_MAX_CHARS);
    memcpy((void *)&(nsReqPkt->resourceInfo),
           (void *)&(transaction->resourceInfo),
           sizeof(Rm_ResourceInfo));

errorExit:
    return(rmPkt);
}

/* FUNCTION PURPOSE: Creates a NameServer response packet
 ***********************************************************************
 * DESCRIPTION: Returns a RM packet handle that points to a RM
 *              NameServer response packet that has been prepared
 *              for sending to another RM instance.  The packet
 *              is allocated via the rmAllocPkt API using the
 *              appTransport handle provided by the application
 */
static void createNsResponsePkt(Rm_Packet *rmPkt, Rm_Transaction *transaction)
{
    Rm_NsResponsePkt *nsRespPkt = NULL;

    rmPkt->pktType = Rm_pktType_NAMESERVER_RESPONSE;
    nsRespPkt = (Rm_NsResponsePkt *)rmPkt->data;
    nsRespPkt->responseId = transaction->remoteOriginatingId;
    nsRespPkt->requestState = transaction->state;
}

/* FUNCTION PURPOSE: Creates a request packet
 ***********************************************************************
 * DESCRIPTION: Returns a RM packet handle that points to a request packet.
 *              The request packet type is based on the transaction type.
 */
static Rm_Packet *createRequestPkt(Rm_Transport *dstTrans,
                                   const char *locInstName,
                                   Rm_Transaction *transaction,
                                   Rm_PacketHandle *pktHandle)
{
    Rm_Packet *rmPkt = NULL;

    switch (transaction->type) {
        case Rm_service_RESOURCE_ALLOCATE_INIT:
            rmPkt = createResourceReqPkt(Rm_resReqPktType_ALLOCATE_INIT,
                                         dstTrans, locInstName, transaction,
                                         pktHandle);
            break;
        case Rm_service_RESOURCE_ALLOCATE_USE:
            rmPkt = createResourceReqPkt(Rm_resReqPktType_ALLOCATE_USE,
                                         dstTrans, locInstName, transaction,
                                         pktHandle);
            break;
        case Rm_service_RESOURCE_STATUS:
            rmPkt = createResourceReqPkt(Rm_resReqPktType_GET_STATUS,
                                         dstTrans, locInstName, transaction,
                                         pktHandle);
            break;
        case Rm_service_RESOURCE_FREE:
            rmPkt = createResourceReqPkt(Rm_resReqPktType_FREE,
                                         dstTrans, locInstName, transaction,
                                         pktHandle);
            break;
        case Rm_service_RESOURCE_GET_BY_NAME:
            rmPkt = createResourceReqPkt(Rm_resReqPktType_GET_NAMED,
                                         dstTrans, locInstName, transaction,
                                         pktHandle);
            break;
        case Rm_service_RESOURCE_MAP_TO_NAME:
            rmPkt = createNsRequestPkt(Rm_nsReqPktType_MAP_RESOURCE,
                                       dstTrans, locInstName, transaction,
                                       pktHandle);
            break;
        case Rm_service_RESOURCE_UNMAP_NAME:
            rmPkt = createNsRequestPkt(Rm_nsReqPktType_UNMAP_RESOURCE,
                                       dstTrans, locInstName, transaction,
                                       pktHandle);
            break;
        default:
            transaction->state = RM_ERROR_INVALID_SERVICE_TYPE;
            break;
    }

    return(rmPkt);
}

/* FUNCTION PURPOSE: Issues a service response to application
 ***********************************************************************
 * DESCRIPTION: Provides a service response back to the application
 *              using the service callback function provided to
 *              the RM instance at the time of the service request.
 */
static void serviceResponder (Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    Rm_ServiceRespInfo serviceResponse;

    serviceResponse.rmHandle = (Rm_Handle)rmInst;
    /* The responseTransaction will contain the resultant state details of
     * the requestTransaction's service request */
    serviceResponse.serviceState = transaction->state;
    /* Pass back the ID that was provided to the component when it requested
     * the service */
    serviceResponse.serviceId = transaction->localId;
    /* Owner and instance allocation count will only be set within RM under
     * certain circumstances. */
    serviceResponse.resourceNumOwners = transaction->resourceInfo.ownerCount;
    serviceResponse.instAllocCount = transaction->resourceInfo.instAllocCount;

    /* Service was approved and service was an allocate request.  The resource
     * data is passed back to the component */
    if ((serviceResponse.serviceState == RM_SERVICE_APPROVED) &&
        ((transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) ||
         (transaction->type == Rm_service_RESOURCE_ALLOCATE_USE) ||
         (transaction->type == Rm_service_RESOURCE_FREE) ||
         (transaction->type == Rm_service_RESOURCE_STATUS) ||
         (transaction->type == Rm_service_RESOURCE_GET_BY_NAME))) {
        rm_strncpy(serviceResponse.resourceName, transaction->resourceInfo.name,
                   RM_NAME_MAX_CHARS);
        serviceResponse.resourceBase = transaction->resourceInfo.base;
        serviceResponse.resourceLength = transaction->resourceInfo.length;
    }

    if (transaction->u.callback.serviceCallback) {
        /* Issue the callback to the requesting component with the response
         * information */
        transaction->u.callback.serviceCallback(&serviceResponse);
        /* Delete the transaction from the transaction queue */
        rmTransactionQueueDelete(rmInst, transaction->localId);
    } else {
        rmServiceInternalCallback((Rm_Handle)rmInst);
    }

    return;
}

/* FUNCTION PURPOSE: Sends RM response packets
 ***********************************************************************
 * DESCRIPTION: Sends RM response packets to RM instance's that sent
 *              RM request packets to the RM instance.  The response
 *              is sent via the RM transport API which is plugged
 *              with an application created transport path.
 */
static void transactionResponder (Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    Rm_Transport    *dstTransport = transaction->u.respTrans;
    Rm_Packet       *rmPkt = NULL;
    Rm_PacketHandle  pktHandle = NULL;

    rmPkt = dstTransport->callouts.rmAllocPkt(dstTransport->appTransportHandle,
                                              sizeof(Rm_Packet), &pktHandle);
    if (!rmPkt || !pktHandle) {
        transaction->state = RM_ERROR_TRANSPORT_ALLOC_PKT_ERROR;
        goto errorExit;
    }

    switch (transaction->type) {
        case Rm_service_RESOURCE_ALLOCATE_INIT:
        case Rm_service_RESOURCE_ALLOCATE_USE:
        case Rm_service_RESOURCE_STATUS:
        case Rm_service_RESOURCE_FREE:
        case Rm_service_RESOURCE_GET_BY_NAME:
            createResourceResponsePkt(rmPkt, transaction);
            break;
        case Rm_service_RESOURCE_MAP_TO_NAME:
        case Rm_service_RESOURCE_UNMAP_NAME:
            createNsResponsePkt(rmPkt, transaction);
            break;
        default:
            transaction->state = RM_ERROR_INVALID_SERVICE_TYPE;
            goto errorExit;
    }
    if (dstTransport->callouts.rmSendPkt(dstTransport->appTransportHandle,
                                         pktHandle) < RM_OK) {
        transaction->state = RM_ERROR_TRANSPORT_SEND_ERROR;
        goto errorExit;
    }

    /* Response packet sent: Delete transaction from queue */
    rmTransactionQueueDelete(rmInst, transaction->localId);

errorExit:
    /* Do not delete transaction on transport error.  Transport error
     * transactions should be visible from Rm_printInstanceStatus() */
    return;
}

/* FUNCTION PURPOSE: Sends RM request packets
 ***********************************************************************
 * DESCRIPTION: Sends RM request packets to RM instance's that are
 *              capable of forwarding or validating service requests.
 *              The request is sent via the RM transport API which is
 *              plugged with an application created transport path.
 */
static void transactionForwarder(Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    Rm_Transport *dstTrans;

    if (rmInst->instType == Rm_instType_CLIENT) {
        dstTrans = rmTransportFindRemoteInstType(rmInst->transports,
                                                 Rm_instType_CLIENT_DELEGATE);
        if (dstTrans == NULL) {
            dstTrans = rmTransportFindRemoteInstType(rmInst->transports,
                                                     Rm_instType_SERVER);
        }
    } else if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
        dstTrans = rmTransportFindRemoteInstType(rmInst->transports,
                                                 Rm_instType_SERVER);
    } else {
        dstTrans = NULL;
    }

    /* Just queue transaction if transport hasn't been registered.  Do not
     * return error */
    if (dstTrans) {
        Rm_Packet       *rmPkt = NULL;
        Rm_PacketHandle  pktHandle = NULL;

        rmPkt = createRequestPkt(dstTrans, rmInst->instName, transaction,
                                 &pktHandle);
        if ((rmPkt == NULL) || (pktHandle == NULL)) {
            /* Error returned via transaction->state */
            goto errorExit;
        }

        if (dstTrans->callouts.rmSendPkt(dstTrans->appTransportHandle,
                                         pktHandle) < RM_OK) {
            transaction->state = RM_ERROR_TRANSPORT_SEND_ERROR;
            goto errorExit;
        }
        transaction->hasBeenForwarded = RM_TRUE;
        /* Transaction not deleted.  Waiting for response from RM CD or
         * Server */
    } else {
        transaction->state = RM_ERROR_TRANSPORT_HANDLE_DOES_NOT_EXIST;
    }
errorExit:
    /* Do not delete transaction on transport error.  Transport error
     * transactions should be visible from Rm_printInstanceStatus() */
    return;
}

/* FUNCTION PURPOSE: Handles static allocation requests
 ***********************************************************************
 * DESCRIPTION: Validates allocation requests received on CDs and
 *              Clients prior to the instance's registering
 *              with a Server.  The allocation request is validated
 *              against a static policy.
 */
static void staticAllocationHandler(Rm_Handle rmHandle,
                                    Rm_Transaction *transaction)
{
    Rm_Inst           *rmInst = (Rm_Inst *)rmHandle;
    Rm_AllocatorNode  *allocNode = NULL;
    Rm_PolicyCheckCfg  privCheckCfg;

    if (rmInst->allocatorTree) {
        allocNode = rmAllocatorFind(rmHandle, transaction->resourceInfo.name);

        if (allocNode &&
            ((transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) ||
             (transaction->type == Rm_service_RESOURCE_ALLOCATE_USE))) {
            /* Check request against static policy */
            memset((void *)&privCheckCfg, 0, sizeof(Rm_PolicyCheckCfg));

            if (transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) {
                privCheckCfg.type = Rm_policyCheck_INIT;
            } else {
                privCheckCfg.type = Rm_policyCheck_USE;
            }
            privCheckCfg.negCheck       = RM_FALSE;
            privCheckCfg.validInstNode  = rmPolicyGetValidInstNode(rmHandle,
                                                              rmInst->instName);
            privCheckCfg.polTree        = allocNode->policyRoot;
            privCheckCfg.resourceBase   = transaction->resourceInfo.base;
            privCheckCfg.resourceLength = transaction->resourceInfo.length;

            if (rmPolicyCheckPrivilege(&privCheckCfg)) {
                transaction->state = RM_SERVICE_APPROVED_STATIC;
            } else {
                transaction->state = RM_SERVICE_DENIED_BY_STATIC_POLICY;
            }
        } else {
            transaction->state = RM_SERVICE_DENIED_INVALID_STATIC_REQUEST;
        }
    } else {
        transaction->state = RM_ERROR_REQ_FAILED_NO_STATIC_POLICY;
    }
}

/* FUNCTION PURPOSE: Requests resources from Server for CD
 ***********************************************************************
 * DESCRIPTION: Function creates a service request to allocate resources
 *              from the Server for local management by the CD.  The
 *              transaction which causes this request is put in the
 *              pending state in order to wait for the response from the 
 *              Server
 */
static int32_t cdRequestServerResources(Rm_Inst *rmInst,
                                        Rm_Transaction *transaction)
{
    Rm_AllocatorNode *allocNode = NULL;
    Rm_Transaction   *newTrans = NULL;
    uint32_t          allocSize = 0;
    int32_t           retVal;

    allocNode = rmAllocatorFind((Rm_Handle)rmInst,
                                transaction->resourceInfo.name);

    if (allocNode) {
        if ((allocSize = rmPolicyGetCdAllocSize(allocNode->policyRoot))) {
            if ((newTrans = rmTransactionQueueAdd(rmInst))) {
                newTrans->type = transaction->type;
                rm_strncpy(newTrans->serviceSrcInstName, rmInst->instName,
                           RM_NAME_MAX_CHARS);
                newTrans->state = RM_SERVICE_PROCESSING;
                rm_strncpy(newTrans->resourceInfo.name,
                           transaction->resourceInfo.name,
                           RM_NAME_MAX_CHARS);
                newTrans->resourceInfo.base = RM_RESOURCE_BASE_UNSPECIFIED;
                /* Make sure request length will satisfy transaction length */
                newTrans->resourceInfo.length = allocSize;
                while (newTrans->resourceInfo.length <
                       transaction->resourceInfo.length) {
                    newTrans->resourceInfo.length += allocSize;
                }
                newTrans->resourceInfo.alignment = transaction->resourceInfo.alignment;
                newTrans->pendingTransactionId = transaction->localId;
                transactionForwarder(rmInst, newTrans);

                retVal = RM_SERVICE_PENDING_SERVER_RESPONSE;
            } else {
                retVal = RM_ERROR_TRANS_REQ_TO_SERVER_NOT_CREATED;
            }
        } else {
            /* Forward request to Server for completion if policy has 
             * no allocation size for resource */
            retVal = RM_SERVICE_PROCESSING;
        }
    } else {
        /* Resource could not be found in policy */
        retVal = RM_SERVICE_DENIED_RES_DOES_NOT_EXIST;
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Frees local resources to Server from CD
 ***********************************************************************
 * DESCRIPTION: Function creates a service request to free locally
 *              managed resources that are now localized back to
 *              the Server.
 */
static int32_t cdFreeResourcesToServer(Rm_Inst *rmInst,
                                       Rm_Transaction *transaction)
{
    int32_t         baseToFree = transaction->resourceInfo.base;
    uint32_t        lenToFree = transaction->resourceInfo.length;
    Rm_Transaction *newTrans = NULL; 
    /* This function should only be called after a free was approved */
    int32_t         retVal = RM_SERVICE_APPROVED;    

    /* Did free result in a localized free node that can be given back to
     * Server?  If so create transaction to Server to free localized
     * resource node */
    if (rmAllocatorGetNodeLocalization((Rm_Handle)rmInst,
                                       transaction->resourceInfo.name,
                                       &baseToFree, &lenToFree)) {
        if ((newTrans = rmTransactionQueueAdd(rmInst))) {
            newTrans->type = transaction->type;
            rm_strncpy(newTrans->serviceSrcInstName, rmInst->instName,
                       RM_NAME_MAX_CHARS);
            newTrans->state = RM_SERVICE_PROCESSING;
            rm_strncpy(newTrans->resourceInfo.name,
                       transaction->resourceInfo.name, RM_NAME_MAX_CHARS);
            newTrans->resourceInfo.base = baseToFree;
            newTrans->resourceInfo.length = lenToFree;
            newTrans->pendingTransactionId = transaction->localId;
            transactionForwarder(rmInst, newTrans);

            retVal = RM_SERVICE_PENDING_SERVER_RESPONSE;
        } else {
            /* Error: Need to re-allocate what was freed */
            retVal = RM_ERROR_TRANS_REQ_TO_SERVER_NOT_CREATED;
        }
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Arbitrates allocation service requests
 ***********************************************************************
 * DESCRIPTION: Issues a set of allocator operations in order to
 *              handle a received allocation request.  Allocation
 *              requests are always forwarded to the Server on Client
 *              CD instances.  If a request is made with a NameServer
 *              name the resource base and length parameters are
 *              retrieved from the NameServer prior to the allocation
 *              attempt.
 */
static void allocationHandler (Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    Rm_AllocatorOpInfo   opInfo;
    Rm_NameServerObjCfg  nameServerObjCfg;
    int32_t              retVal = transaction->state;

    memset((void *)&opInfo, 0, sizeof(Rm_AllocatorOpInfo));
    opInfo.resourceInfo = &transaction->resourceInfo;
    opInfo.serviceInstNode = rmPolicyGetValidInstNode((Rm_Handle)rmInst,
                                              transaction->serviceSrcInstName);
    if (opInfo.serviceInstNode == NULL) {
        retVal = RM_SERVICE_DENIED_INST_NAME_NOT_VALID;
        goto errorExit;
    }

    if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
        if (transaction->resourceInfo.base != RM_RESOURCE_BASE_UNSPECIFIED) {
            if (rmAllocatorFind((Rm_Handle)rmInst,
                                transaction->resourceInfo.name)) {
                /* Attempt to allocate from local resources that were provided
                 * by Server */
                if (transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) {
                    opInfo.operation = Rm_allocatorOp_ALLOCATE_INIT;
                } else if (transaction->type ==
                           Rm_service_RESOURCE_ALLOCATE_USE) {
                    opInfo.operation = Rm_allocatorOp_ALLOCATE_USE;
                } else {
                    retVal = RM_ERROR_INVALID_SERVICE_TYPE;
                    goto errorExit;
                }
                retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);

                if (retVal == RM_SERVICE_DENIED_RES_RANGE_DOES_NOT_EXIST) {
                    /* Request resource range was not found within local
                     * resources provided by Server.  Set back to PROCESSING so
                     * request is forwarded to Server */
                    retVal = RM_SERVICE_PROCESSING;
                }
            }
        } else {
            if (rmAllocatorFind((Rm_Handle)rmInst,
                                transaction->resourceInfo.name)) {
                int32_t oldAlign = transaction->resourceInfo.alignment;

                /* Attempt to allocate from local resources that were provided
                 * by Server */
                if (transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) {
                    opInfo.operation = Rm_allocatorOp_PRE_ALLOCATE_INIT;
                } else if (transaction->type ==
                         Rm_service_RESOURCE_ALLOCATE_USE) {
                    opInfo.operation = Rm_allocatorOp_PRE_ALLOCATE_USE;
                } else {
                    retVal = RM_ERROR_INVALID_SERVICE_TYPE;
                    goto errorExit;
                }
                retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);

                if (retVal == RM_SERVICE_PROCESSING) {
                    if (transaction->type ==
                        Rm_service_RESOURCE_ALLOCATE_INIT) {
                        opInfo.operation = Rm_allocatorOp_ALLOCATE_INIT;
                    } else if (transaction->type ==
                               Rm_service_RESOURCE_ALLOCATE_USE) {
                        opInfo.operation = Rm_allocatorOp_ALLOCATE_USE;
                    } else {
                        retVal = RM_ERROR_INVALID_SERVICE_TYPE;
                        goto errorExit;
                    }
                    retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);

                    if (retVal == RM_SERVICE_DENIED_RES_RANGE_DOES_NOT_EXIST) {
                        /* Request resource range was not found within local
                         * resources provided by Server.  Set back to
                         * PROCESSING so request is forwarded to Server */
                        retVal = RM_SERVICE_PROCESSING;
                    }
                } else if (retVal == RM_SERVICE_DENIED_RES_ALLOC_REQS_NOT_MET) {
                    if (transaction->pendingTransactionId) {
                        /* Request to Server for resources to complete
                         * transaction locally performed once already.  Forward
                         * transaction to Server for completion */
                        retVal = RM_SERVICE_PROCESSING;
                    } else {
                        /* Restore base and alignment since they were replaced
                         * in pre-allocate routine */
                        transaction->resourceInfo.base = RM_RESOURCE_BASE_UNSPECIFIED;
                        transaction->resourceInfo.alignment = oldAlign;

                        retVal = cdRequestServerResources(rmInst, transaction);
                    }
                }
                /* else: fall through to return */
            } else {
                retVal = cdRequestServerResources(rmInst, transaction);
            }
        }
    } else if ((rmInst->instType == Rm_instType_SERVER)||
               (rmInst->instType == Rm_instType_SHARED_SERVER)) {
        /* Populated NameServer name has precedence over base */
        if (strlen(transaction->resourceInfo.nameServerName) > 0) {
            if (rmInst->instType == Rm_instType_SHARED_SERVER) {
                rmNameServerTreeInv(rmInst->u.server.nameServer);
            }
            memset((void *)&nameServerObjCfg, 0, sizeof(Rm_NameServerObjCfg));
            nameServerObjCfg.nameServerTree = rmInst->u.server.nameServer;
            nameServerObjCfg.nodeCfg.objName = transaction->resourceInfo.nameServerName;
            if ((retVal = rmNameServerFindObject(&nameServerObjCfg)) ==
                RM_SERVICE_PROCESSING) {
                rm_strncpy(transaction->resourceInfo.name,
                           nameServerObjCfg.nodeCfg.resourceName,
                           RM_NAME_MAX_CHARS);
                transaction->resourceInfo.base = nameServerObjCfg.nodeCfg.resourceBase;
                transaction->resourceInfo.length = nameServerObjCfg.nodeCfg.resourceLength;
            } else {
                goto errorExit;
            }
        }

        if (transaction->resourceInfo.base == RM_RESOURCE_BASE_UNSPECIFIED) {
            if (transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) {
                opInfo.operation = Rm_allocatorOp_PRE_ALLOCATE_INIT;
            } else if (transaction->type == Rm_service_RESOURCE_ALLOCATE_USE) {
                opInfo.operation = Rm_allocatorOp_PRE_ALLOCATE_USE;
            } else {
                retVal = RM_ERROR_INVALID_SERVICE_TYPE;
                goto errorExit;
            }
            retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);
        }

        if (retVal == RM_SERVICE_PROCESSING) {
            if (transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) {
                opInfo.operation = Rm_allocatorOp_ALLOCATE_INIT;
            } else if (transaction->type == Rm_service_RESOURCE_ALLOCATE_USE) {
                opInfo.operation = Rm_allocatorOp_ALLOCATE_USE;
            } else {
                retVal = RM_ERROR_INVALID_SERVICE_TYPE;
                goto errorExit;
            }
            retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);
        }
    } else {
        retVal = RM_ERROR_INVALID_INST_TYPE;
    }
errorExit:
    transaction->state = retVal;
}

/* FUNCTION PURPOSE: Handles resource status service requests
 ***********************************************************************
 * DESCRIPTION: Issues a set of allocator operations to retrieve the
 *              current status (currently just owner reference count)
 *              for the resource specified in the transaction
 */
static void statusHandler(Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    Rm_AllocatorOpInfo  opInfo;
    Rm_NameServerObjCfg nameServerObjCfg;
    int32_t             retVal = transaction->state;

    memset((void *)&opInfo, 0, sizeof(Rm_AllocatorOpInfo));
    opInfo.operation = Rm_allocatorOp_GET_STATUS;
    opInfo.resourceInfo = &transaction->resourceInfo;
    opInfo.serviceInstNode = rmPolicyGetValidInstNode((Rm_Handle)rmInst,
                                            transaction->serviceSrcInstName);
    if (opInfo.serviceInstNode == NULL) {
        retVal = RM_SERVICE_DENIED_INST_NAME_NOT_VALID;
        goto errorExit;
    }

    if ((strlen(transaction->resourceInfo.nameServerName) == 0) &&
        ((transaction->resourceInfo.base == RM_RESOURCE_BASE_UNSPECIFIED) ||
         (transaction->resourceInfo.length == 0))) {
        retVal = RM_SERVICE_DENIED_RES_DOES_NOT_EXIST;
        goto errorExit;
    }

    if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
        if (rmAllocatorFind((Rm_Handle)rmInst,
                            transaction->resourceInfo.name)) {
            /* Attempt to get status from local resources that were provided by
             * Server */
            retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);

            if (retVal == RM_SERVICE_DENIED_RES_RANGE_DOES_NOT_EXIST) {
                /* Request resource range was not found within local allocator
                 * resources provided by Server.  Set back to PROCESSING so
                 * request is forwarded to Server */
                retVal = RM_SERVICE_PROCESSING;
            }
        }
    } else if ((rmInst->instType == Rm_instType_SERVER)||
               (rmInst->instType == Rm_instType_SHARED_SERVER)) {
        /* Populated NameServer name has precedence over base and length
         * values */
        if (strlen(transaction->resourceInfo.nameServerName) > 0) {
            if (rmInst->instType == Rm_instType_SHARED_SERVER) {
                rmNameServerTreeInv(rmInst->u.server.nameServer);
            }
            memset((void *)&nameServerObjCfg, 0, sizeof(Rm_NameServerObjCfg));
            nameServerObjCfg.nameServerTree = rmInst->u.server.nameServer;
            nameServerObjCfg.nodeCfg.objName = transaction->resourceInfo.nameServerName;
            if ((retVal = rmNameServerFindObject(&nameServerObjCfg)) ==
                RM_SERVICE_PROCESSING) {
                rm_strncpy(transaction->resourceInfo.name,
                           nameServerObjCfg.nodeCfg.resourceName,
                           RM_NAME_MAX_CHARS);
                transaction->resourceInfo.base = nameServerObjCfg.nodeCfg.resourceBase;
                transaction->resourceInfo.length = nameServerObjCfg.nodeCfg.resourceLength;
            } else {
                goto errorExit;
            }
        }
        retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);
    } else {
        retVal = RM_ERROR_INVALID_INST_TYPE;
    }
errorExit:
    transaction->state = retVal;
}

/* FUNCTION PURPOSE: Arbitrates free service requests
 ***********************************************************************
 * DESCRIPTION: Issues a set of allocator operations in order to
 *              handle a received free request.  Free
 *              requests are always forwarded to the Server on Client
 *              CD instances.  If a request is made with a NameServer
 *              name the resource base and length parameters are
 *              retrieved from the NameServer prior to the free
 *              attempt.
 */
static void freeHandler(Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    Rm_AllocatorOpInfo  opInfo; 
    Rm_NameServerObjCfg nameServerObjCfg;
    int32_t             retVal = transaction->state;

    memset((void *)&opInfo, 0, sizeof(Rm_AllocatorOpInfo));
    opInfo.operation = Rm_allocatorOp_FREE;
    opInfo.resourceInfo = &transaction->resourceInfo;
    opInfo.serviceInstNode = rmPolicyGetValidInstNode((Rm_Handle)rmInst,
                                            transaction->serviceSrcInstName);
    if (opInfo.serviceInstNode == NULL) {
        retVal = RM_SERVICE_DENIED_INST_NAME_NOT_VALID;
        goto errorExit;
    }

    if ((strlen(transaction->resourceInfo.nameServerName) == 0) &&
        ((transaction->resourceInfo.base == RM_RESOURCE_BASE_UNSPECIFIED) ||
         (transaction->resourceInfo.length == 0))) {
        retVal = RM_SERVICE_DENIED_RES_DOES_NOT_EXIST;
        goto errorExit;
    }

    if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
        /* Attempt to free from local resources that were provided by Server */
        retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);

        if (retVal == RM_SERVICE_APPROVED) {
            /* Check if free allows local resources to be freed back to
             * Server */
            retVal = cdFreeResourcesToServer(rmInst, transaction);
        } else if ((retVal == RM_SERVICE_DENIED_RES_RANGE_DOES_NOT_EXIST) ||
                 (retVal == RM_SERVICE_DENIED_RES_DOES_NOT_EXIST)) {
            /* Requested resource or its range were not found within local
             * allocator resources provided by Server.  Set back to PROCESSING
             * so request is forwarded to Server */
            retVal = RM_SERVICE_PROCESSING;
        }
        /* else: fall through to exit */
    } else if ((rmInst->instType == Rm_instType_SERVER) ||
             (rmInst->instType == Rm_instType_SHARED_SERVER)) {
        /* Populated NameServer name has precedence over base */
        if (strlen(transaction->resourceInfo.nameServerName) > 0) {
            if (rmInst->instType == Rm_instType_SHARED_SERVER) {
                rmNameServerTreeInv(rmInst->u.server.nameServer);
            }                    
            memset((void *)&nameServerObjCfg, 0, sizeof(Rm_NameServerObjCfg));
            nameServerObjCfg.nameServerTree = rmInst->u.server.nameServer;
            nameServerObjCfg.nodeCfg.objName = transaction->resourceInfo.nameServerName;
            if ((retVal = rmNameServerFindObject(&nameServerObjCfg)) ==
                RM_SERVICE_PROCESSING) {
                rm_strncpy(transaction->resourceInfo.name,
                           nameServerObjCfg.nodeCfg.resourceName,
                           RM_NAME_MAX_CHARS);
                transaction->resourceInfo.base = nameServerObjCfg.nodeCfg.resourceBase;
                transaction->resourceInfo.length = nameServerObjCfg.nodeCfg.resourceLength;
            } else {
                goto errorExit;
            }
        }
        retVal = rmAllocatorOperation((Rm_Handle)rmInst, &opInfo);
    } else {
        retVal = RM_ERROR_INVALID_INST_TYPE;
    }
errorExit:
    transaction->state = retVal;
}

/* FUNCTION PURPOSE: Client transaction handling process
 ***********************************************************************
 * DESCRIPTION: Client process for handling transactions created
 *              from services received via the service handle or the
 *              transport.  The Client process:
 *                  - Performs static allocations if no transport
 *                    to CD or Server has been registered
 *                  - Forwards all service requests to CD or Server
 *                    once transport has been registered
 */
static void clientProcess(Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    Rm_Transaction *transQ;

    if (!rmInst->registeredWithDelegateOrServer) {
        staticAllocationHandler((Rm_Handle)rmInst, transaction);
    } else {
        if (transaction->state == RM_SERVICE_PROCESSING) {
            /* Forward all new transactions to CD or Server */
            transactionForwarder(rmInst, transaction);
        } else {
            /* Transaction validated.  Return result. */
            serviceResponder(rmInst, transaction);
        }

        /* Forward any queued static requests that weren't forwarded */
        transQ = rmInst->transactionQueue;
        while(transQ) {
            if ((transQ->state == RM_SERVICE_APPROVED_STATIC) &&
                (!transQ->hasBeenForwarded)) {
                transactionForwarder(rmInst, transQ);
            }
            transQ = transQ->nextTransaction;
        }
    }
    /* Let call stack return transaction result app via Rm_serviceHandler */
}

/* FUNCTION PURPOSE: Client Delegate transaction handling process
 ***********************************************************************
 * DESCRIPTION: Client Delegate process for handling transactions created
 *              from services received via the service handle or the
 *              transport.  The Client Delegate process:
 *                  - Performs static allocations if no transport
 *                    to Server has been registered
 *                  - Forwards all NameServer related service requests 
 *                    to Server once transport has been registered
 *                  - Attempts to complete resource service requests
 *                    received from registered Clients
 */
static void cdProcess(Rm_Inst *rmInst, Rm_Transaction *transaction)
{      
    Rm_Transaction   *newTrans = NULL;
    Rm_AllocatorNode *allocator = NULL;
    Rm_Transaction   *transQ;

    if (!rmInst->registeredWithDelegateOrServer) {
        if ((transaction->state == RM_SERVICE_PROCESSING) &&
            (strncmp(transaction->serviceSrcInstName, rmInst->instName,
             RM_NAME_MAX_CHARS) == 0)) {
            /* Attempt static allocation of requests originating from CD inst */
            staticAllocationHandler((Rm_Handle)rmInst, transaction);
        }
        /* Everything else left in transaction queue for forwarding once
         * transport to Server is registered */
    } else {
        if (transaction->pendingTransactionId) {
            Rm_Transaction *pendingTrans = rmTransactionQueueFind(rmInst,
                                             transaction->pendingTransactionId);

            /* Transaction is response from Server for transaction sent to get
             * information in order to complete pending transaction */
            if (transaction->state == RM_SERVICE_APPROVED) {
                if (transaction->type == Rm_service_RESOURCE_GET_BY_NAME) {
                    /* Transfer resource data tied to name to pending
                     * transaction */
                    rm_strncpy(pendingTrans->resourceInfo.name,
                               transaction->resourceInfo.name,
                               RM_NAME_MAX_CHARS);
                    pendingTrans->resourceInfo.base   = transaction->resourceInfo.base;
                    pendingTrans->resourceInfo.length = transaction->resourceInfo.length;
                    /* Delete NS name from pending transaction so Server isn't
                     * queried again */
                    memset(pendingTrans->resourceInfo.nameServerName, 0,
                           RM_NAME_MAX_CHARS);
                    /* Now that resource values have been retrieved clear
                     * pending transaction ID so CD doesn't think a resource
                     * request was sent to Server already for more local
                     * resources */
                    pendingTrans->pendingTransactionId = 0;

                    /* Return original transaction to processing state to
                     * attempt completion. */
                    pendingTrans->state = RM_SERVICE_PROCESSING;
                } else if ((transaction->type ==
                            Rm_service_RESOURCE_ALLOCATE_INIT) ||
                           (transaction->type ==
                            Rm_service_RESOURCE_ALLOCATE_USE)) {
                    /* Add resources provided by Server to those managed by
                     * CD */
                    if ((allocator = rmAllocatorFind((Rm_Handle)rmInst,
                                             transaction->resourceInfo.name))) {
                        rmAllocatorAddResNode((Rm_Handle)rmInst,
                                              allocator,
                                              transaction->resourceInfo.base,
                                              transaction->resourceInfo.length);
                    }

                    /* Return original transaction to processing state to
                     * attempt completion */
                    pendingTrans->state = RM_SERVICE_PROCESSING;
                } else if (transaction->type == Rm_service_RESOURCE_FREE) {
                    allocator = rmAllocatorFind((Rm_Handle)rmInst,
                                                transaction->resourceInfo.name);

                    /* Local resource freed on Server.  Remove node in
                     * local allocator's resource tree as well */
                    rmAllocatorDeleteResNode((Rm_Handle)rmInst,
                                             allocator,
                                             transaction->resourceInfo.base,
                                             transaction->resourceInfo.length);

                    /* Allow original free to complete */
                    pendingTrans->state = RM_SERVICE_APPROVED;
                } else {
                    pendingTrans->state = RM_ERROR_SERVER_RESP_INVALID_SERVICE_TYPE;
                }
            } else {
                if (transaction->type == Rm_service_RESOURCE_FREE) {
                    /* Error occurred when trying to free local resource on
                     * Server.  Reinsert local resources freed by original
                     * request */
                    Rm_AllocatorOpInfo   opInfo;

                    memset((void *)&opInfo, 0, sizeof(Rm_AllocatorOpInfo));
                    opInfo.resourceInfo = &pendingTrans->resourceInfo;
                    opInfo.serviceInstNode = rmPolicyGetValidInstNode((Rm_Handle)rmInst,
                                              pendingTrans->serviceSrcInstName);
                    /* Can't regain the original type of allocate.  Default to
                     * init */
                    opInfo.operation = Rm_allocatorOp_ALLOCATE_INIT;
                    if (rmAllocatorOperation((Rm_Handle)rmInst, &opInfo) !=
                        RM_SERVICE_APPROVED) {
                        transaction->state = RM_ERROR_LOST_RESOURCES_ON_CD;
                    }
                }
                /* Transfer error or denial to pending transaction */
                pendingTrans->state = transaction->state;
            }
            rmTransactionQueueDelete(rmInst, transaction->localId);
            /* Switch to pending transaction */
            transaction = pendingTrans;
        }

        if ((transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) ||
            (transaction->type == Rm_service_RESOURCE_ALLOCATE_USE) ||
            (transaction->type == Rm_service_RESOURCE_STATUS) ||
            (transaction->type == Rm_service_RESOURCE_FREE)) {
            if ((transaction->state == RM_SERVICE_PROCESSING) &&
                (strlen(transaction->resourceInfo.nameServerName) > 0)) {
                /* Create and forward new transaction to Server to
                 * retrieve resource data mapped to name */
                if ((newTrans = rmTransactionQueueAdd(rmInst))) {
                    newTrans->type = Rm_service_RESOURCE_GET_BY_NAME;
                    rm_strncpy(newTrans->serviceSrcInstName, rmInst->instName,
                               RM_NAME_MAX_CHARS);
                    newTrans->state = RM_SERVICE_PROCESSING;
                    rm_strncpy(newTrans->resourceInfo.nameServerName,
                               transaction->resourceInfo.nameServerName,
                               RM_NAME_MAX_CHARS);
                    newTrans->pendingTransactionId = transaction->localId;
                    transactionForwarder(rmInst, newTrans);

                    transaction->state = RM_SERVICE_PENDING_SERVER_RESPONSE;
                } else {
                    transaction->state = RM_ERROR_TRANS_REQ_TO_SERVER_NOT_CREATED;
                }
            }
        }

        if (transaction->state == RM_SERVICE_PROCESSING) {
            switch(transaction->type) {
                case Rm_service_RESOURCE_ALLOCATE_INIT:
                case Rm_service_RESOURCE_ALLOCATE_USE:
                    allocationHandler(rmInst, transaction);
                    break;
                case Rm_service_RESOURCE_FREE:
                    freeHandler(rmInst, transaction);
                    break;
                case Rm_service_RESOURCE_STATUS:
                    statusHandler(rmInst, transaction);
                    break;
                case Rm_service_RESOURCE_MAP_TO_NAME:
                case Rm_service_RESOURCE_GET_BY_NAME:
                case Rm_service_RESOURCE_UNMAP_NAME:
                    /* Forward all NameServer-based transactions */
                    break;
                default:
                    transaction->state = RM_ERROR_INVALID_SERVICE_TYPE;
                    break;
            }
        }

        if (transaction->state == RM_SERVICE_PROCESSING) {
            uint32_t transId = transaction->localId;

            /* NameServer transaction or CD could not complete alloc/free
             * transaction.  Forward to Server */
            transactionForwarder(rmInst, transaction);

            /* Refresh transaction for reentrancy of cases where mix of Client
             * CD and Server are running on the same core and connected via
             * transport implemented over direct function calls instead of
             * traditional transport that returns after sending the data */
            transaction = rmTransactionQueueFind(rmInst, transId);
        }

        if (transaction) {
            if ((transaction->state != RM_SERVICE_PROCESSING) &&
                (transaction->state != RM_SERVICE_PENDING_SERVER_RESPONSE)) {
                /* Transaction completed by CD or completed response received
                 * from Server.  Return result */
                if (strncmp(transaction->serviceSrcInstName, rmInst->instName,
                    RM_NAME_MAX_CHARS)) {
                    /* Transaction did not originate on this instance */
                    transactionResponder(rmInst, transaction);
                } else {
                    /* Transaction originated on this instance */
                    serviceResponder(rmInst, transaction);
                }
            }
        }

        /* Attempt allocation of any queued static requests:
         * RM_SERVICE_APPROVED_STATIC - Originated locally
         * RM_SERVICE_PROCESSING - Received from any registered Clients */
        transQ = rmInst->transactionQueue;
        while(transQ) {
            if (((transQ->state == RM_SERVICE_PROCESSING) ||
                 (transQ->state == RM_SERVICE_APPROVED_STATIC)) &&
                (!transQ->hasBeenForwarded)) {
                transactionForwarder(rmInst, transQ);
            }
            transQ = transQ->nextTransaction;
        }
    }
}

/* FUNCTION PURPOSE: Server transaction handling process
 ***********************************************************************
 * DESCRIPTION: Server process for handling transactions created
 *              from services received via the service handle or the
 *              transport.  The Server process:
 *                  - Validates all service requests received from
 *                    the service handle and registered CDs and
 *                    Clients
 */
static void serverProcess(Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    Rm_NameServerObjCfg  nameServerObjCfg;

    switch (transaction->type) {
        case Rm_service_RESOURCE_STATUS:
            statusHandler(rmInst, transaction);
            break;
        case Rm_service_RESOURCE_ALLOCATE_INIT:
        case Rm_service_RESOURCE_ALLOCATE_USE:
            allocationHandler(rmInst, transaction);
            break;
        case Rm_service_RESOURCE_FREE:
            freeHandler(rmInst, transaction);
            break;
        case Rm_service_RESOURCE_MAP_TO_NAME:
        case Rm_service_RESOURCE_GET_BY_NAME:
        case Rm_service_RESOURCE_UNMAP_NAME:
            if (rmInst->u.server.nameServer) {
                if (rmInst->instType == Rm_instType_SHARED_SERVER) {
                    rmNameServerTreeInv(rmInst->u.server.nameServer);
                }                
                memset((void *)&nameServerObjCfg, 0,
                       sizeof(Rm_NameServerObjCfg));
                nameServerObjCfg.nameServerTree = rmInst->u.server.nameServer;
                nameServerObjCfg.nodeCfg.objName = transaction->resourceInfo.nameServerName;
                if (transaction->type == Rm_service_RESOURCE_MAP_TO_NAME) {
                    nameServerObjCfg.nodeCfg.resourceName = transaction->resourceInfo.name;
                    nameServerObjCfg.nodeCfg.resourceBase= transaction->resourceInfo.base;
                    nameServerObjCfg.nodeCfg.resourceLength = transaction->resourceInfo.length;
                    transaction->state = rmNameServerAddObject(&nameServerObjCfg);
                } else if (transaction->type ==
                           Rm_service_RESOURCE_GET_BY_NAME) {
                    if ((transaction->state = rmNameServerFindObject(&nameServerObjCfg)) ==
                        RM_SERVICE_PROCESSING) {
                        rm_strncpy(transaction->resourceInfo.name,
                                   nameServerObjCfg.nodeCfg.resourceName,
                                   RM_NAME_MAX_CHARS);
                        transaction->resourceInfo.base = nameServerObjCfg.nodeCfg.resourceBase;
                        transaction->resourceInfo.length = nameServerObjCfg.nodeCfg.resourceLength;
                        transaction->state = RM_SERVICE_APPROVED;
                    }
                } else if (transaction->type ==
                           Rm_service_RESOURCE_UNMAP_NAME) {
                    transaction->state = rmNameServerDeleteObject(&nameServerObjCfg);
                } else {
                    transaction->state = RM_ERROR_INVALID_SERVICE_TYPE;
                }

                if (rmInst->instType == Rm_instType_SHARED_SERVER) {
                    rmNameServerTreeWb(rmInst->u.server.nameServer);
                }
            } else {
                transaction->state = RM_ERROR_NAMESERVER_DOES_NOT_EXIST;
            }
            break;
        default:
            transaction->state = RM_ERROR_INVALID_SERVICE_TYPE;
            break;
    }

    /* Source of shared server transaction will always be local. */
    if (rmInst->instType != Rm_instType_SHARED_SERVER) {
        if (strncmp(transaction->serviceSrcInstName, rmInst->instName,
                    RM_NAME_MAX_CHARS)) {
            /* Source of transaction was not Server, return transaction via
             * responder */
            transactionResponder(rmInst, transaction);
        }
    }
    /* Otherwise let call stack return transaction result app via
     * Rm_serviceHandler */ 
}

/**********************************************************************
 ********************** Internal Functions ****************************
 **********************************************************************/

/* FUNCTION PURPOSE: Adds a transaction
 ***********************************************************************
 * DESCRIPTION: Returns a pointer to a newly created transaction.
 *              The transaction is created based on a new service
 *              request received via the service API or the
 *              transport API (service forwarded from another instance)
 */
Rm_Transaction *rmTransactionQueueAdd(Rm_Inst *rmInst)
{
    Rm_Transaction *transactionQueue = rmInst->transactionQueue;
    Rm_Transaction *newTransaction   = NULL;

    newTransaction = Rm_osalMalloc(sizeof(Rm_Transaction));
    if (newTransaction) {
        memset((void *)newTransaction, 0, sizeof(Rm_Transaction));

        newTransaction->localId = transactionGetSequenceNum(rmInst);
        newTransaction->nextTransaction = NULL;  
        if (transactionQueue) {
            while (transactionQueue->nextTransaction) {
                transactionQueue = transactionQueue->nextTransaction;
            }
            transactionQueue->nextTransaction = newTransaction;
        } else {
            rmInst->transactionQueue = newTransaction;
        }
    }
    return(newTransaction);
}

/* FUNCTION PURPOSE: Finds a transaction
 ***********************************************************************
 * DESCRIPTION: Returns a pointer to a transaction resident
 *              in the transaction queue that matches the provided
 *              transaction ID.
 */
Rm_Transaction *rmTransactionQueueFind(Rm_Inst *rmInst, uint32_t transactionId)
{
    Rm_Transaction *transaction = rmInst->transactionQueue;

    while (transaction) {
        if (transaction->localId == transactionId) {
            break;
        }
        transaction = transaction->nextTransaction;
    }

    return(transaction);
}

/* FUNCTION PURPOSE: Deletes a transaction
 ***********************************************************************
 * DESCRIPTION: Deletes the transaction with the provided transaction
 *              ID from the instance's transaction queue.
 */
int32_t rmTransactionQueueDelete(Rm_Inst *rmInst, uint32_t transactionId)
{
    Rm_Transaction *transaction = rmInst->transactionQueue;
    Rm_Transaction *prevTransaction = NULL;
    int32_t         retVal = RM_OK;

    while (transaction) {
        if (transaction->localId == transactionId) {
            break;
        }

        prevTransaction = transaction;
        transaction = transaction->nextTransaction;
    }

    if (transaction) {
        if (prevTransaction == NULL) {
            /* Transaction at start of queue. Map second transaction to start
             * of queue as long as more than one transactions. */
            rmInst->transactionQueue = transaction->nextTransaction;
        } else {
            /* Transaction in middle or end of queue. */
            prevTransaction->nextTransaction = transaction->nextTransaction;
        }
        Rm_osalFree((void *)transaction, sizeof(Rm_Transaction));
    } else {
        retVal = RM_ERROR_SERVICE_TRANS_DOES_NOT_EXIST;
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Routes a transaction for processing
 ***********************************************************************
 * DESCRIPTION: Routes a received transaction to the appropriate
 *              instance processing routine
 */
void rmProcessRouter(Rm_Inst *rmInst, Rm_Transaction *transaction)
{
    if (rmInst->instType == Rm_instType_CLIENT) {
        clientProcess(rmInst, transaction);
    } else if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
        cdProcess(rmInst, transaction);
    } else if ((rmInst->instType == Rm_instType_SERVER) ||
               (rmInst->instType == Rm_instType_SHARED_SERVER)) {
        serverProcess(rmInst, transaction);
    } else {
        transaction->state = RM_ERROR_INVALID_INST_TYPE;
    }
}

/**********************************************************************
 ********************** Application visible APIs **********************
 **********************************************************************/

/* FUNCTION PURPOSE: Display status of managed resources
 ***********************************************************************
 * DESCRIPTION: Prints the status (allocate/free status, as well as
 *              owners) for all resources managed by the RM 
 *              instance network.  Also, prints the NameServer name
 *              entries.  The number of resource range owners is
 *              returned as well.  This function is only available on
 *              Server and CD instances.
 */
int32_t Rm_resourceStatus(Rm_Handle rmHandle, int printResources)
{
    Rm_Inst          *rmInst = (Rm_Inst *)rmHandle;
    Rm_AllocatorTree *allocTree = NULL;
    Rm_AllocatorNode *allocator;
    Rm_Owner         *owners;
    Rm_ResourceTree  *treeRoot;
    Rm_ResourceNode  *treeNode;
    int32_t           totalResOwners = 0;
    void             *key;
    void             *mtKey;

    RM_SS_INST_INV_ENTER_CS(rmInst, key);
    RM_SC_INST_INV_ENTER_CS(rmInst, key);
    if (rmInst->mtSemObj) {
        mtKey = Rm_osalMtCsEnter(rmInst->mtSemObj);
    }

    if (rmInst->instType != Rm_instType_CLIENT) {
        Rm_osalLog("Instance name: %s\n", rmInst->instName);
        Rm_osalLog("Handle: 0x%08x\n", rmHandle);
        if (rmInst->instType == Rm_instType_SERVER) {
            Rm_osalLog("Type:   Server\n");
        } else if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
            Rm_osalLog("Type:   Client Delegate\n");
        } else if (rmInst->instType == Rm_instType_SHARED_SERVER) {
            Rm_osalLog("Type:   Shared Server\n");
        } else if (rmInst->instType == Rm_instType_SHARED_CLIENT) {
            Rm_osalLog("Type:   Shared Client\n");
        } else {
            Rm_osalLog("Type:   UNKNOWN\n");
            goto errorExit;
        }

        Rm_osalLog("\nResource Status:\n\n");
    }

    if (rmInst->instType == Rm_instType_SHARED_CLIENT) {
        /* Transfer control to shared server instance */
        rmInst = rmInst->u.sharedClient.sharedServerHandle;
    }

    if ((rmInst->instType == Rm_instType_SERVER) ||
        (rmInst->instType == Rm_instType_SHARED_SERVER) ||
        (rmInst->instType == Rm_instType_CLIENT_DELEGATE)) {

        allocTree = rmInst->allocatorTree;
        if (rmInst->instType == Rm_instType_SHARED_SERVER) {
            rmAllocatorTreeInv(allocTree);
        }

        RB_FOREACH(allocator, _Rm_AllocatorTree, allocTree) {
            RM_SS_OBJ_INV(rmInst, allocator, Rm_AllocatorNode);
            if (printResources) {
                Rm_osalLog("Resource: %s\n", allocator->resourceName);
            }

            treeRoot = allocator->resourceRoot;
            if (rmInst->instType == Rm_instType_SHARED_SERVER) {
                rmResourceTreeInv(treeRoot);
            }
            RB_FOREACH(treeNode, _Rm_AllocatorResourceTree, treeRoot) {
                if (printResources) {
                    if ((treeNode->base >= 65536) ||
                        ((treeNode->base + treeNode->length - 1) >= 65536)) {
                        /* Print in hex if number is very large */
                        Rm_osalLog("          0x%08x - 0x%08x ",
                                   treeNode->base,
                                   treeNode->base + treeNode->length - 1);
                    } else {
                        Rm_osalLog("          %10d - %10d ",
                                   treeNode->base,
                                   treeNode->base + treeNode->length - 1);
                    }
                }

                if (treeNode->allocationCount == 0) {
                    if (printResources) {
                        Rm_osalLog("FREE\n");
                    }
                } else {
                    owners = treeNode->ownerList;
                    while (owners) {
                        RM_SS_OBJ_INV(rmInst, owners, Rm_Owner);
                        if (printResources) {
                            Rm_osalLog("%s (%d) ", owners->instNameNode->name,
                                       owners->refCnt);
                        }
                        totalResOwners++;
                        owners = owners->nextOwner;
                    }
                    if (printResources) {
                        Rm_osalLog("\n");
                    }
                }
            }
        }

        if ((rmInst->instType == Rm_instType_SERVER) ||
            (rmInst->instType == Rm_instType_SHARED_SERVER)) {
            if (printResources) {
                rmNameServerPrintObjects((Rm_Handle)rmInst);
            }
        }
    } else {
        totalResOwners = RM_ERROR_INVALID_RES_STATUS_INSTANCE;
    }

errorExit:
    /* Free sem object using originating instance in case the Shared Client to Shared
     * Server instance switch took place */
    if (((Rm_Inst *)rmHandle)->mtSemObj) {
        Rm_osalMtCsExit(((Rm_Inst *)rmHandle)->mtSemObj, mtKey);
    }
    RM_SS_INST_WB_EXIT_CS(rmInst, key);
    return(totalResOwners);
}

/* FUNCTION PURPOSE: Display status of a RM instance
 ***********************************************************************
 * DESCRIPTION: Prints the current status of various RM instance
 *              properties such as the state of all transactions
 *              in the transaction queue and registered transports
 */
void Rm_instanceStatus(Rm_Handle rmHandle)
{
    Rm_Inst        *rmInst = (Rm_Inst *)rmHandle;
    Rm_Transport   *transportList = NULL;
    Rm_Transaction *transactionQ = NULL;
    void           *key;
    void           *mtKey;

    RM_SS_INST_INV_ENTER_CS(rmInst, key);
    RM_SC_INST_INV_ENTER_CS(rmInst, key);
    if (rmInst->mtSemObj) {
        mtKey = Rm_osalMtCsEnter(rmInst->mtSemObj);
    }

    Rm_osalLog("Instance name: %s\n", rmInst->instName);
    Rm_osalLog("Handle: 0x%08x\n", rmHandle);    
    if (rmInst->instType == Rm_instType_SERVER) {
        Rm_osalLog("Type:   Server\n");
    } else if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
        Rm_osalLog("Type:   Client Delegate\n");
    } else if (rmInst->instType == Rm_instType_CLIENT) {
        Rm_osalLog("Type:   Client\n");
    } else if (rmInst->instType == Rm_instType_SHARED_SERVER) {
        Rm_osalLog("Type:   Shared Server\n");
    } else if (rmInst->instType == Rm_instType_SHARED_CLIENT) {
        Rm_osalLog("Type:   Shared Client\n");

        Rm_osalLog("\nShared Server Properties:\n");
        /* Transfer to Shared Server instance to print out transport and
         * transaction status */
        rmInst = rmInst->u.sharedClient.sharedServerHandle;
        Rm_osalLog("Instance name: %s\n", rmInst->instName);
        Rm_osalLog("Handle: 0x%08x\n", rmHandle);
    } else {
        Rm_osalLog("Type:   UNKNOWN\n");
        goto errorExit;
    }

    transportList = rmInst->transports;
    if (transportList) {
        Rm_osalLog("\nRegistered Transports:\n");
        while (transportList) {
            RM_SS_OBJ_INV(rmInst, transportList, Rm_Transport);
            if (transportList->remoteInstType == Rm_instType_SERVER) {
                Rm_osalLog("    Remote instType:    Server\n");
            } else if (transportList->remoteInstType ==
                       Rm_instType_CLIENT_DELEGATE) {
                Rm_osalLog("    Remote instType:    Client Delegate\n");
            } else {
                Rm_osalLog("    Remote instType:    Client\n");
            }
            Rm_osalLog("    appTransportHandle: 0x%08x\n",
                       transportList->appTransportHandle);
            Rm_osalLog("\n");
            transportList = transportList->nextTransport;
        }
    }

    transactionQ = rmInst->transactionQueue;
    if (transactionQ) {
        Rm_osalLog("\nQueued Service Transactions:\n");
        while (transactionQ) {
            RM_SS_OBJ_INV(rmInst, transactionQ, Rm_Transaction);
            Rm_osalLog("    Service type:       %d\n",
                       transactionQ->type);
            Rm_osalLog("    Service ID:         %d\n", transactionQ->localId);
            Rm_osalLog("    Service srcInstName %s\n",
                       transactionQ->serviceSrcInstName);
            Rm_osalLog("    Service state:      %d\n", transactionQ->state);
            Rm_osalLog("    Resource name:      %s\n",
                       transactionQ->resourceInfo.name);
            Rm_osalLog("    Resource base:      %d\n",
                       transactionQ->resourceInfo.base);
            Rm_osalLog("    Resource length:    %d\n",
                       transactionQ->resourceInfo.length);
            Rm_osalLog("    Resource alignment: %d\n",
                       transactionQ->resourceInfo.alignment);
            Rm_osalLog("    Resource NS name:   %s\n",
                       transactionQ->resourceInfo.nameServerName);
            Rm_osalLog("\n");
            transactionQ = transactionQ->nextTransaction;
        }
    }

errorExit:
    /* Free sem object using originating instance in case the Shared Client
     * to Shared Server instance switch took place */
    if (((Rm_Inst *)rmHandle)->mtSemObj) {
        Rm_osalMtCsExit(((Rm_Inst *)rmHandle)->mtSemObj, mtKey);
    }
    RM_SS_INST_WB_EXIT_CS(rmInst, key);
}

/* FUNCTION PURPOSE: RM instance creation and initialization
 ***********************************************************************
 * DESCRIPTION: Returns a new RM instance created and initialized
 *              using the parameters provided via the initCfg
 *              structure.
 */
Rm_Handle Rm_init(const Rm_InitCfg *initCfg, int32_t *result)
{
    Rm_Inst  *rmInst = NULL;
    void     *grlDtb = NULL;
    void     *policyDtb = NULL;
    void     *linuxDtb = NULL;
    int       addLinux = RM_FALSE;
    void     *key;

    *result = RM_OK;

    if ((initCfg->instName == NULL) ||
        ((strlen(initCfg->instName) + 1) > RM_NAME_MAX_CHARS)) {
        *result = RM_ERROR_INVALID_INST_NAME;
        goto errorExit;
    }

    if (initCfg->instType >= Rm_instType_LAST) {
        *result = RM_ERROR_INVALID_INST_TYPE;
        goto errorExit;
    }

    /* Create and initialize instance */
    rmInst = Rm_osalMalloc(sizeof(*rmInst));
    memset((void *)rmInst, 0, sizeof(*rmInst));
    rmInst->isLocked = RM_FALSE;
    rmInst->registeredWithDelegateOrServer = RM_FALSE;
    rmInst->transactionSeqNum = transactionInitSequenceNum();

    rmInst->instType = initCfg->instType;
    rm_strncpy(rmInst->instName, initCfg->instName, RM_NAME_MAX_CHARS);
    rmInst->mtSemObj = initCfg->mtSemObj;

    if ((rmInst->instType == Rm_instType_SERVER) ||
        (rmInst->instType == Rm_instType_SHARED_SERVER)) {
        if (!initCfg->instCfg.serverCfg.globalResourceList ||
            !initCfg->instCfg.serverCfg.globalPolicy) {
            *result = RM_ERROR_INVALID_SERVER_CONFIGURATION;
            goto errorExit;
        }

        if (initCfg->instCfg.serverCfg.linuxDtb) {
            linuxDtb = initCfg->instCfg.serverCfg.linuxDtb;
            addLinux = RM_TRUE;
        }

        /* Create valid instance list from policy.  Must be done prior to
         * parsing GRL so that Linux resources can be reserved correctly */
        policyDtb = initCfg->instCfg.serverCfg.globalPolicy;
        rmInst->validInstTree = rmPolicyVInstTreeInit(rmInst, policyDtb,
                                                      addLinux, result);
        if (*result != RM_OK) {
            goto errorExit;
        }

        rmNameServerInit((Rm_Handle)rmInst);
        grlDtb = initCfg->instCfg.serverCfg.globalResourceList;
        if ((*result = rmAllocatorTreeInit(rmInst, grlDtb,
                                           policyDtb, linuxDtb)) != RM_OK) {
            goto errorExit;
        }
    } else if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
        if (!initCfg->instCfg.cdCfg.cdPolicy) {
            *result = RM_ERROR_INVALID_CD_CONFIGURATION;
            goto errorExit;
        }

        policyDtb = initCfg->instCfg.cdCfg.cdPolicy;
        rmInst->validInstTree = rmPolicyVInstTreeInit(rmInst, policyDtb,
                                                      addLinux, result);
        if (*result != RM_OK) {
            goto errorExit;
        }

        if ((*result = rmAllocatorTreeInit(rmInst, NULL,
                                           policyDtb, NULL)) != RM_OK) {
            goto errorExit;
        }

        /* Remove once CD instance is stable - tracked by SDOCM00100797 */
        *result = RM_WARNING_CD_INSTANCE_NOT_STABLE;

    } else if (rmInst->instType == Rm_instType_CLIENT) {
        if (initCfg->instCfg.clientCfg.staticPolicy) {
            policyDtb = initCfg->instCfg.clientCfg.staticPolicy;
            rmInst->validInstTree = rmPolicyVInstTreeInit(rmInst, policyDtb,
                                                          addLinux, result);
            if (*result != RM_OK) {
                goto errorExit;
            }

            if ((*result = rmAllocatorTreeInit(rmInst, NULL,
                                               policyDtb, NULL)) != RM_OK) {
                goto errorExit;
            }
        }
        
    } else if (rmInst->instType == Rm_instType_SHARED_CLIENT) {
        Rm_Handle  sHdl = initCfg->instCfg.sharedClientCfg.sharedServerHandle;
        Rm_Inst   *ssInst = NULL;

        if (sHdl) {
            rmInst->u.sharedClient.sharedServerHandle = sHdl;
            /* Invalidate the Shared server instance structure on this core to
             * get the latest instance data. */
            key = Rm_osalCsEnter();
            Rm_osalBeginMemAccess((void *)sHdl, sizeof(Rm_Inst));
            ssInst = rmInst->u.sharedClient.sharedServerHandle;
            if (ssInst->instType != Rm_instType_SHARED_SERVER) {
                *result = RM_ERROR_INVALID_SHARED_SERVER_HANDLE;
                Rm_osalCsExit(key);
                goto errorExit;
            } else {
                /* Invalidate all the trees */
                rmPolicyValidInstTreeInv(ssInst->validInstTree);
                rmAllocatorTreeInv(ssInst->allocatorTree);
                rmNameServerTreeInv(ssInst->u.server.nameServer);
            }
            Rm_osalCsExit(key);
        } else {
            *result = RM_ERROR_INVALID_SHARED_SERVER_HANDLE;
            goto errorExit;
        }
    } else {
        *result = RM_ERROR_INVALID_INST_TYPE;
        goto errorExit;
    }

    if (rmInst->instType == Rm_instType_SHARED_SERVER) {
        /* Writeback instance and trees for other cores */
        rmPolicyValidInstTreeWb(rmInst->validInstTree);
        rmAllocatorTreeWb(rmInst->allocatorTree);
        rmNameServerTreeWb(rmInst->u.server.nameServer);
        Rm_osalEndMemAccess((void *)rmInst, sizeof(*rmInst));
    } else if (rmInst->instType != Rm_instType_SHARED_CLIENT) {
        /* Create the instance's task blocking mechanism */
        rmInst->blockHandle = Rm_osalTaskBlockCreate();
    }
    /* else: just return handle */

    return((Rm_Handle)rmInst);

errorExit:
    if (rmInst) {
        rmAllocatorTreeDelete((Rm_Handle)rmInst);
        rmNameServerDelete((Rm_Handle)rmInst);
        rmPolicyVInstTreeDelete((Rm_Handle)rmInst);
        Rm_osalFree((void *)rmInst, sizeof(*rmInst));
    }
    return(NULL);
}

/* FUNCTION PURPOSE: Deletes an RM instance
 ***********************************************************************
 * DESCRIPTION: Frees all memory associated with an RM instance
 *              as long as all transports have been unregistered
 *              and the service handle has been closed
 */
int32_t Rm_delete(Rm_Handle rmHandle, int ignorePendingServices)
{
    Rm_Inst *rmInst = (Rm_Inst *)rmHandle;
    void    *key;

    key = Rm_osalCsEnter();
    if (rmInst->instType == Rm_instType_SHARED_SERVER) {
        Rm_osalBeginMemAccess((void *)rmInst, sizeof(*rmInst));
    }

    if (rmInst->serviceHandle) {
        return(RM_ERROR_CANT_DELETE_WITH_OPEN_SERV_HNDL);
    } else if (rmInst->transports) {
        return(RM_ERROR_CANT_DELETE_WITH_REGD_TRANSPORT);
    } else if (rmInst->transactionQueue && !ignorePendingServices) {
        return(RM_ERROR_CANT_DELETE_PENDING_TRANSACTIONS);
    }
    /* else: delete instance since no more deletion failure cases */

    if (rmInst->instType != Rm_instType_SHARED_CLIENT) {
        rmNameServerDelete(rmHandle);
        rmAllocatorTreeDelete(rmHandle);
        rmPolicyVInstTreeDelete(rmHandle);

        /* Delete any transactions */
        while(rmInst->transactionQueue) {
            rmTransactionQueueDelete(rmInst, rmInst->transactionQueue->localId);
        }

        if (rmInst->instType != Rm_instType_SHARED_SERVER) {
            /* Delete the instance's task blocking mechanism */
            Rm_osalTaskBlockDelete(rmInst->blockHandle);
        } else {
            rmInst->allocatorTree       = NULL;
            rmInst->validInstTree       = NULL;
            rmInst->u.server.nameServer = NULL;
            Rm_osalEndMemAccess((void *)rmInst, sizeof(*rmInst));
        }
    }

    Rm_osalFree((void *)rmInst, sizeof(*rmInst));
    Rm_osalCsExit(key);
    return(RM_OK);
}

/* FUNCTION PURPOSE: Returns RM version information
 ***********************************************************************
 */
uint32_t Rm_getVersion(void)
{
    return(RM_VERSION_ID);
}

/* FUNCTION PURPOSE: Returns RM version string
 ***********************************************************************
 */
const char* Rm_getVersionStr(void)
{
    return(rmVersionStr);
}

