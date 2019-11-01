/**
 *   @file  rm_transport.c
 *
 *   @brief   
 *      This is the Resource Manager transport source.
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

/* RM external includes */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>

/* RM internal includes */
#include <ti/drv/rm/include/rm_loc.h>
#include <ti/drv/rm/include/rm_transportloc.h>

/* RM OSAL layer */
#include <rm_osal.h>

/**********************************************************************
 ************************ Local Functions *****************************
 **********************************************************************/

/* FUNCTION PURPOSE: Adds a transport
 ***********************************************************************
 * DESCRIPTION: Returns a pointer to a transport structure that
 *              was created, initialized, and added to the 
 *              instance transport list.
 */
static Rm_Transport *transportAdd(const Rm_TransportCfg *transportCfg)
{
    Rm_Inst      *rmInst = (Rm_Inst *) transportCfg->rmHandle;
    Rm_Transport *transports = rmInst->transports;
    Rm_Transport *newTransport = NULL;

    newTransport = Rm_osalMalloc (sizeof(Rm_Transport));
    memset((void *)newTransport, 0, sizeof(Rm_Transport));

    if (newTransport) {
        newTransport->rmHandle = transportCfg->rmHandle;
        newTransport->appTransportHandle = transportCfg->appTransportHandle;
        newTransport->remoteInstType = transportCfg->remoteInstType;
        newTransport->callouts.rmAllocPkt = transportCfg->transportCallouts.rmAllocPkt;
        newTransport->callouts.rmSendPkt = transportCfg->transportCallouts.rmSendPkt;
        newTransport->nextTransport = NULL;

        if (transports) {
            while (transports->nextTransport) {
                transports = transports->nextTransport;
            }
            transports->nextTransport = newTransport;
        } else {
            rmInst->transports = newTransport;
        }
    }
    return(newTransport);
}

/* FUNCTION PURPOSE: Tests if a transport is registered to an instance
 ***********************************************************************
 * DESCRIPTION: Returns TRUE if the supplied transport is found in
 *              the instance transport list.  Otherwise, returns FALSE
 */
static int transportIsRegistered(Rm_Handle rmHandle, Rm_Transport *transport)
{
    Rm_Inst      *rmInst = (Rm_Inst *)rmHandle;
    Rm_Transport *transportList = (Rm_Transport *)rmInst->transports;

    while (transportList) {
        if (transportList == transport) {
            return(RM_TRUE);
        }
        transportList = transportList->nextTransport;
    }
    return(RM_FALSE);
}

/* FUNCTION PURPOSE: Deletes a transport
 ***********************************************************************
 * DESCRIPTION: Removes a transport from an instance transport list
 *              and then frees the memory associated with the transport
 *              data structure
 */
static void transportDelete(Rm_Transport *transport)
{
    Rm_Inst      *rmInst = (Rm_Inst *)transport->rmHandle;
    Rm_Transport *transportList = (Rm_Transport *)rmInst->transports;
    Rm_Transport *prevTransport = NULL;

    /* Get previous transport in list */
    while (transportList) {
        if (transportList == transport) {
            break;
        }
        prevTransport = transportList;
        transportList = transportList->nextTransport;
    }

    if (prevTransport == NULL) {
         rmInst->transports = transport->nextTransport;
    } else {
         prevTransport->nextTransport = transport->nextTransport;
    }
    Rm_osalFree((void *)transport, sizeof(Rm_Transport));
}

/* FUNCTION PURPOSE: Returns RM packet source instance
 ***********************************************************************
 * DESCRIPTION: Can return the RM instance name for one of two things:
 *                  - RM instance from which provided RM packet originated
 *                  - RM instance from which the service request contained
 *                    in the provided RM packet originated
 */
static int32_t getPktSrcNames(const Rm_Packet *pkt, char *pktSrc,
                              char *serviceSrc, int32_t bufLen)
{
    int32_t retVal = RM_OK;

    if (bufLen != RM_NAME_MAX_CHARS) {
        retVal = RM_ERROR_SRC_NAME_BUF_INVALID_SIZE;
    } else {
        switch (pkt->pktType) {
            case Rm_pktType_RESOURCE_REQUEST:
            {
                Rm_ResourceRequestPkt *resourceReqPkt = (Rm_ResourceRequestPkt *)pkt->data;

                if (pktSrc){
                    rm_strncpy(pktSrc, resourceReqPkt->pktSrcInstName,
                               RM_NAME_MAX_CHARS);
                }

                if (serviceSrc) {
                    rm_strncpy(serviceSrc, resourceReqPkt->serviceSrcInstName,
                               RM_NAME_MAX_CHARS);
                }
                break;
            }
            case Rm_pktType_NAMESERVER_REQUEST:
            {
                Rm_NsRequestPkt *nsRequestPkt = (Rm_NsRequestPkt *)pkt->data;

                if (pktSrc){
                    rm_strncpy(pktSrc, nsRequestPkt->pktSrcInstName,
                               RM_NAME_MAX_CHARS);
                }

                if (serviceSrc) {
                    rm_strncpy(serviceSrc, nsRequestPkt->serviceSrcInstName,
                               RM_NAME_MAX_CHARS);
                }
                break;
            }
            case Rm_pktType_RESOURCE_RESPONSE:
            case Rm_pktType_NAMESERVER_RESPONSE:
            default:
                retVal = RM_ERROR_PKT_AND_SERVICE_SRC_NOT_AVAIL;
                break;
        }
    }

    return(retVal);
}

/**********************************************************************
 ********************** Internal Functions ****************************
 **********************************************************************/

/* FUNCTION PURPOSE: Finds a transport based on remote inst type
 ***********************************************************************
 * DESCRIPTION: Returns a pointer to the transport within an instance's
 *              transport list that matches the provided remote
 *              instance type.  NULL is returned if no transports in 
 *              the list match the remote instance type.
 */
Rm_Transport *rmTransportFindRemoteInstType(Rm_Transport *transports,
                                            Rm_InstType remoteInstType)
{
    while (transports) {
        if (transports->remoteInstType == remoteInstType) {
            break;
        }
        transports = transports->nextTransport;
    }
    return(transports);
}

/**********************************************************************
 ********************* Application visible APIs ***********************
 **********************************************************************/

/* FUNCTION PURPOSE: Registers an app transport with a RM instance
 ***********************************************************************
 * DESCRIPTION: Returns a transport handle to the application that
 *              has been registered with an RM instance.  The handle
 *              is used by the application to route packets to the
 *              proper RM instance's based on the application
 *              transport receive code.  The handle is also used
 *              internally by RM to route request and response 
 *              packets to the correct application transports. NULL
 *              is returned for the transport handle if any errors
 *              are encountered.
 */
Rm_TransportHandle Rm_transportRegister(const Rm_TransportCfg *transportCfg,
                                        int32_t *result)
{
    Rm_Inst      *rmInst = (Rm_Inst *) transportCfg->rmHandle;
    Rm_Transport *transport = NULL;
    void         *key;
    void         *mtKey;

    *result = RM_OK;

    RM_SS_INST_INV_ENTER_CS(rmInst, key);
    if (rmInst->mtSemObj) {
        mtKey = Rm_osalMtCsEnter(rmInst->mtSemObj);
    }

    /* Shared servers and clients cannot connect to anyone */
    if ((rmInst->instType == Rm_instType_SHARED_SERVER) ||
        (rmInst->instType == Rm_instType_SHARED_CLIENT)){
        *result = RM_ERROR_SHARED_INSTANCE_CANNOT_REG_TRANS;
        goto errorExit;
    }

    /* No one can connect to a shared server
     * RM Servers cannot connect to other Servers.  
     * RM Client Delegates cannot connect to other Client Delegates.
     * RM Clients cannot connect to other Clients */
    if ((transportCfg->remoteInstType == Rm_instType_SHARED_SERVER) ||
        ((rmInst->instType == Rm_instType_SERVER) &&
         (transportCfg->remoteInstType == Rm_instType_SERVER)) ||
        ((rmInst->instType == Rm_instType_CLIENT_DELEGATE) &&
         (transportCfg->remoteInstType == Rm_instType_CLIENT_DELEGATE)) ||
        ((rmInst->instType == Rm_instType_CLIENT) &&
         (transportCfg->remoteInstType == Rm_instType_CLIENT))) {
        *result = RM_ERROR_INVALID_REMOTE_INST_TYPE;
        goto errorExit;
    }

    /* Verify Clients are not registering with more than one Client Delegate or
     * Server. And that Client Delegate is not registering with more than one
     * Server. */
    if (rmInst->registeredWithDelegateOrServer &&
        (((rmInst->instType == Rm_instType_CLIENT) &&
          (transportCfg->remoteInstType == Rm_instType_CLIENT_DELEGATE)) ||
         ((rmInst->instType == Rm_instType_CLIENT_DELEGATE) &&
          (transportCfg->remoteInstType == Rm_instType_SERVER)))) {
        *result = RM_ERROR_ALREADY_REGD_SERVER_OR_CD;
        goto errorExit;
    }

    if (!transportCfg->transportCallouts.rmAllocPkt) {
        *result = RM_ERROR_TRANSPORT_ALLOC_PKT_NOT_REGD;
        goto errorExit;
    }
    if (!transportCfg->transportCallouts.rmSendPkt) {
        *result = RM_ERROR_TRANSPORT_SEND_NOT_REGD;
        goto errorExit;
    }

    transport = transportAdd(transportCfg);
    if ((transport->remoteInstType == Rm_instType_CLIENT_DELEGATE) ||
        (transport->remoteInstType == Rm_instType_SERVER)) {
        rmInst->registeredWithDelegateOrServer = RM_TRUE;
    }
    
errorExit:
    if (rmInst->mtSemObj) {
        Rm_osalMtCsExit(rmInst->mtSemObj, mtKey);
    }
    RM_SS_INST_WB_EXIT_CS(rmInst, key);
    return ((Rm_TransportHandle) transport);
}

/* FUNCTION PURPOSE: Reconfigures an instance's transport handle
 ***********************************************************************
 * DESCRIPTION: Reconfigures a transport handle based on the provided
 *              configuration parameters if it exists within the 
 *              instance.
 */
int32_t Rm_transportReconfig(Rm_TransportHandle transportHandle,
                             const Rm_TransportReCfg *reCfg)
{
    Rm_Transport *transport = (Rm_Transport *)transportHandle;
    Rm_Inst      *rmInst = (Rm_Inst *)transport->rmHandle;
    void         *key;
    void         *mtKey;
    int32_t       retVal = RM_OK;

    RM_SS_INST_INV_ENTER_CS(rmInst, key);
    if (rmInst->mtSemObj) {
        mtKey = Rm_osalMtCsEnter(rmInst->mtSemObj);
    }

    if (transportIsRegistered(transport->rmHandle, transport)) {
        /* Reconfigure existing transport's appTransportHandle.  Used in cases
         * where instances running on same core connected by function call. */
        transport->appTransportHandle = reCfg->appTransportHandle;

        if (reCfg->transportCallouts.rmAllocPkt) {
            transport->callouts.rmAllocPkt = reCfg->transportCallouts.rmAllocPkt;
        }
        if (reCfg->transportCallouts.rmSendPkt) {
            transport->callouts.rmSendPkt = reCfg->transportCallouts.rmSendPkt;
        }
    } else {
        retVal = RM_ERROR_TRANSPORT_HANDLE_DOES_NOT_EXIST;
    }

    if (rmInst->mtSemObj) {
        Rm_osalMtCsExit(rmInst->mtSemObj, mtKey);
    }
    RM_SS_INST_WB_EXIT_CS(rmInst, key);
    return(retVal);
}

/* FUNCTION PURPOSE: Unregisters an app transport from a RM instance
 ***********************************************************************
 * DESCRIPTION: Deletes a registered transport from the transport
 *              list and cleans up the memory associated with the
 *              transport data structure.
 */
int32_t Rm_transportUnregister(Rm_TransportHandle transportHandle)
{
    Rm_Transport *transport = (Rm_Transport *)transportHandle;
    Rm_Inst      *rmInst = (Rm_Inst *)transport->rmHandle;
    int32_t       retVal = RM_OK;

    if (transportIsRegistered(transport->rmHandle, transport)) {
        if ((transport->remoteInstType == Rm_instType_CLIENT_DELEGATE) ||
            (transport->remoteInstType == Rm_instType_SERVER)) {
            rmInst->registeredWithDelegateOrServer = RM_FALSE;
        }
        transportDelete(transport);
    } else {
        retVal = RM_ERROR_TRANSPORT_HANDLE_DOES_NOT_EXIST;
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Returns a RM packet's service source instance name
 ***********************************************************************
 * DESCRIPTION: Returns the RM instance name from which the service
 *              encapsulated in the RM packet originated.
 */
int32_t Rm_receiveGetPktServiceSrcName(const Rm_Packet *pkt,
                                       char *serviceInstName,
                                       int32_t charBufLen)
{
    return(getPktSrcNames(pkt, NULL, serviceInstName, charBufLen));
}

/* FUNCTION PURPOSE: Returns a RM packet's source instance name
 ***********************************************************************
 * DESCRIPTION: Returns the RM instance name from which the RM packet
 *              originated.
 */
int32_t Rm_receiveGetPktSrcName(const Rm_Packet *pkt, char *pktInstName,
                                int32_t charBufLen)
{
    return(getPktSrcNames(pkt, pktInstName, NULL, charBufLen));
}

/* FUNCTION PURPOSE: Receives RM packets
 ***********************************************************************
 * DESCRIPTION: The application provides RM packets received on the
 *              application transports to RM via this API.  Function
 *              can be called from polling or ISR contexts.  Assume 
 *              invoking application will free packet after this
 *              function returns.
 */
int32_t Rm_receivePacket(Rm_TransportHandle transportHandle,
                         const Rm_Packet *pkt)
{
    Rm_Transport   *transport = (Rm_Transport *)transportHandle;
    Rm_Inst        *rmInst = (Rm_Inst *)transport->rmHandle;
    Rm_Transaction *transaction;
    int32_t         retVal = RM_OK;

    if (!transportIsRegistered(transport->rmHandle, transport)) {
        retVal = RM_ERROR_TRANSPORT_HANDLE_DOES_NOT_EXIST;
        goto errorExit;
    }

    switch (pkt->pktType) {
        case Rm_pktType_RESOURCE_REQUEST:
        {
            Rm_ResourceRequestPkt *resourceReqPkt = (Rm_ResourceRequestPkt *)pkt->data;

            transaction = rmTransactionQueueAdd(rmInst);
            transaction->remoteOriginatingId = resourceReqPkt->requestId;
            transaction->u.respTrans = transport;
            if (resourceReqPkt->resourceReqType ==
                Rm_resReqPktType_ALLOCATE_INIT) {
                transaction->type = Rm_service_RESOURCE_ALLOCATE_INIT;
            } else if (resourceReqPkt->resourceReqType ==
                       Rm_resReqPktType_ALLOCATE_USE) {
                transaction->type = Rm_service_RESOURCE_ALLOCATE_USE;
            } else if (resourceReqPkt->resourceReqType ==
                       Rm_resReqPktType_GET_STATUS) {
                transaction->type = Rm_service_RESOURCE_STATUS;
            } else if (resourceReqPkt->resourceReqType ==
                       Rm_resReqPktType_FREE) {
                transaction->type = Rm_service_RESOURCE_FREE;
            } else if (resourceReqPkt->resourceReqType ==
                       Rm_resReqPktType_GET_NAMED) {
                transaction->type = Rm_service_RESOURCE_GET_BY_NAME;
            } else {
                retVal = RM_ERROR_RECEIVED_INVALID_PACKET_TYPE;
                goto errorExit;
            }
            rm_strncpy(transaction->serviceSrcInstName,
                       resourceReqPkt->serviceSrcInstName, RM_NAME_MAX_CHARS);
            transaction->state = RM_SERVICE_PROCESSING;
            memcpy((void *)&(transaction->resourceInfo),
                   (void *)&(resourceReqPkt->resourceInfo),
                   sizeof(Rm_ResourceInfo));
            break;
        }
        case Rm_pktType_RESOURCE_RESPONSE:
        {
            Rm_ResourceResponsePkt *resourceRespPkt = (Rm_ResourceResponsePkt *)pkt->data;

            if ((transaction = rmTransactionQueueFind(rmInst,
                                                resourceRespPkt->responseId))) {
                if ((transaction->state == RM_SERVICE_APPROVED_STATIC) &&
                    (resourceRespPkt->requestState != RM_SERVICE_APPROVED)) {
                    /* Lock the RM instance since service validated against
                     * static policy failed against Server's global policy */
                    rmInst->isLocked = RM_TRUE;
                }
                transaction->state = resourceRespPkt->requestState;

                if ((transaction->state == RM_SERVICE_APPROVED) &&
                    ((transaction->type == Rm_service_RESOURCE_ALLOCATE_INIT) ||
                     (transaction->type == Rm_service_RESOURCE_ALLOCATE_USE) ||
                     (transaction->type == Rm_service_RESOURCE_STATUS) ||
                     (transaction->type == Rm_service_RESOURCE_GET_BY_NAME))) {
                    memcpy((void *)&(transaction->resourceInfo),
                           (void *)&(resourceRespPkt->resourceInfo),
                           sizeof(Rm_ResourceInfo));
                } else {
                    /* Always copy owner count and instance allocation count */
                    transaction->resourceInfo.ownerCount = resourceRespPkt->resourceInfo.ownerCount;
                    transaction->resourceInfo.instAllocCount = resourceRespPkt->resourceInfo.instAllocCount;
                }
            } else {
                retVal = RM_ERROR_PKT_RESP_DOES_NOT_MATCH_ANY_REQ;
                goto errorExit;
            }
            break;
        }
        case Rm_pktType_NAMESERVER_REQUEST:
        {
            Rm_NsRequestPkt *nsRequestPkt = (Rm_NsRequestPkt *)pkt->data;

            transaction = rmTransactionQueueAdd(rmInst);
            transaction->remoteOriginatingId = nsRequestPkt->requestId;
            transaction->u.respTrans = transport;

            if (nsRequestPkt->nsRequestType == Rm_nsReqPktType_MAP_RESOURCE) {
                transaction->type = Rm_service_RESOURCE_MAP_TO_NAME;
            } else if (nsRequestPkt->nsRequestType ==
                       Rm_nsReqPktType_UNMAP_RESOURCE) {
                transaction->type = Rm_service_RESOURCE_UNMAP_NAME;
            }

            rm_strncpy(transaction->serviceSrcInstName,
                       nsRequestPkt->serviceSrcInstName, RM_NAME_MAX_CHARS);
            transaction->state = RM_SERVICE_PROCESSING;
            memcpy((void *)&(transaction->resourceInfo),
                   (void *)&(nsRequestPkt->resourceInfo),
                    sizeof(Rm_ResourceInfo));
            break;
        }
        case Rm_pktType_NAMESERVER_RESPONSE:
        {
            Rm_NsResponsePkt *nsResponsePkt = (Rm_NsResponsePkt *)pkt->data;

            if ((transaction = rmTransactionQueueFind(rmInst,
                                                  nsResponsePkt->responseId))) {
                if ((transaction->state == RM_SERVICE_APPROVED_STATIC) &&
                    (nsResponsePkt->requestState != RM_SERVICE_APPROVED)) {
                    /* Lock the RM instance since service validated against
                     * static policy failed against Server's global policy */
                    rmInst->isLocked = RM_TRUE;
                }
                transaction->state = nsResponsePkt->requestState;
            } else {
                retVal = RM_ERROR_PKT_RESP_DOES_NOT_MATCH_ANY_REQ;
                goto errorExit;
            }
            break;
        }
        default:
            retVal = RM_ERROR_RECEIVED_INVALID_PACKET_TYPE;
            goto errorExit;
    }

    /* Process received transaction */
    rmProcessRouter(rmInst, transaction);
errorExit:
    return(retVal);
}

