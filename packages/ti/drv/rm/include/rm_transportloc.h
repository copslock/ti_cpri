/*
 *  file  rm_transportloc.h
 *
 *  Private data structures of Resource Manager transport layer.
 *
 *  ============================================================================
 *      (C) Copyright 2012-2015, Texas Instruments, Inc.
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

#ifndef RM_TRANSPORTLOC_H_
#define RM_TRANSPORTLOC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* RM external includes */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>

/* RM internal includes */
#include <ti/drv/rm/include/rm_internal.h>

/* This macro generates compilier error if postulate is false, so
 * allows 0 overhead compile time size check.  This "works" when
 * the expression contains sizeof() which otherwise doesn't work
 * with preprocessor */
#define RM_COMPILE_TIME_SIZE_CHECK(postulate)                           \
   do {                                                                 \
       typedef struct {                                                 \
         uint8_t NegativeSizeIfPostulateFalse[((int)(postulate))*2 - 1];\
       } PostulateCheck_t;                                              \
   }                                                                    \
   while (0)

/* Check assumptions RM request/response packets fit in
 * RM_TRANSPORT_PACKET_MAX_SIZE_BYTES */
#define RM_PACKET_SIZE_CHECK \
    RM_COMPILE_TIME_SIZE_CHECK(RM_TRANSPORT_PACKET_MAX_SIZE_BYTES >= \
                               sizeof(Rm_ResourceRequestPkt));       \
    RM_COMPILE_TIME_SIZE_CHECK(RM_TRANSPORT_PACKET_MAX_SIZE_BYTES >= \
                               sizeof(Rm_ResourceResponsePkt));      \
    RM_COMPILE_TIME_SIZE_CHECK(RM_TRANSPORT_PACKET_MAX_SIZE_BYTES >= \
                               sizeof(Rm_NsRequestPkt));             \
    RM_COMPILE_TIME_SIZE_CHECK(RM_TRANSPORT_PACKET_MAX_SIZE_BYTES >= \
                               sizeof(Rm_NsResponsePkt));            \

/* RM transport list node */
typedef struct Rm_Transport_s {
    /* RM instance this node was registered to */
    Rm_Handle              rmHandle;
    /* Application transport handle provided by application */
    Rm_AppTransportHandle  appTransportHandle;
    /* Remote RM instance type */
    Rm_InstType            remoteInstType;
    /* Transport callout functions provided by application during transport
     * registration */
    Rm_TransportCallouts   callouts;    
    /* Link to next transport node */
    struct Rm_Transport_s *nextTransport;
} Rm_Transport;

/**********************************************************************
 ************************* RM Packet Defs *****************************
 **********************************************************************/

/* Resource Request Packet Types */
typedef enum {
    /* Resource allocate for init request */
    Rm_resReqPktType_ALLOCATE_INIT = 0,
    /* Resource allocate for use request */
    Rm_resReqPktType_ALLOCATE_USE,
    /* Resource status request */
    Rm_resReqPktType_GET_STATUS,
    /* Free resource */
    Rm_resReqPktType_FREE,
    /* Get name resource */
    Rm_resReqPktType_GET_NAMED,
    /* Invalid request packet type */
    Rm_resReqPktType_INVALID
} Rm_ResourceReqPktType;

/* Resource Request Packet - Packet sent to CDs and Servers to request
 * a resource service that cannot be handled on the local RM instance due to
 * permission restrictions. */
typedef struct {
    /* Transaction ID associated with the request packet */
    uint32_t              requestId;  
    /* Resource request type */
    Rm_ResourceReqPktType resourceReqType;
    /* RM instance name from which the request packet originated.  Used by
     * receiving instance to route the response */
    char                  pktSrcInstName[RM_NAME_MAX_CHARS];
    /* Name of RM instance that is issuing the service request. The instance
     * name will be used to validate the request against the RM policy defined
     * for the instance. */
    char                  serviceSrcInstName[RM_NAME_MAX_CHARS];
    /* Resource request information */
    Rm_ResourceInfo       resourceInfo;
} Rm_ResourceRequestPkt;

/* Resource Response Packet - Packet sent to RM instances that requested
 * a resource service.  The packet will contain resource response
 * information based on the request */
typedef struct {
    /* responseID will equal the requestId received in the resource request
     * packet.  This ID should be associated with a transaction stored in
     * the RM instance that sent the resource request packet */
    uint32_t        responseId;
    /* State of the request.  Resource request denied, or error.  The
     * return values are externally visible in rm.h */
    int32_t         requestState;
    /* Resource response information */
    Rm_ResourceInfo resourceInfo;
} Rm_ResourceResponsePkt;

/* NameServer Request Packet Types */
typedef enum {
    /* Request to map the specified name to the specified resources */
    Rm_nsReqPktType_MAP_RESOURCE = 0,
    /* Request to unmap the specified name from the specifed resources */
    Rm_nsReqPktType_UNMAP_RESOURCE,
    /* Invalide NS request type */
    Rm_nsReqPktType_INVALID
} Rm_NsReqPktType;

/* NameServer Request Packet - Packet sent by RM instances containing
 * NameServer mapping or unmapping data */
typedef struct {
    /* Transaction ID associated with the NameServer request packet */
    uint32_t        requestId;
    /* NameServer request type */
    Rm_NsReqPktType nsRequestType;
    /* RM instance name from which the request packet originated.  Used by
     * receiving instance to route the response */
    char            pktSrcInstName[RM_NAME_MAX_CHARS];
    /* RM instance name making the service request.  Policies may restrict who
     * can and cannot map and unmap resources via the RM NameServer */
    char            serviceSrcInstName[RM_NAME_MAX_CHARS];
    /* Resource request information */
    Rm_ResourceInfo resourceInfo;
} Rm_NsRequestPkt;

/* NameServer Response Packet - Provides NameServer transaction response
 * data to RM instances that request a name map or unmap */
typedef struct {
    /* responseID will equal the requestId received in the NameServer request
     * packet.  This ID should be associated with a transaction stored in 
     * the RM instance that sent the resource request packet */
    uint32_t responseId;
    /* State of the request.  Resource request denied, or error.  The
     * return values are externally visible in rm.h */
    int32_t  requestState;
} Rm_NsResponsePkt;

Rm_Transport *rmTransportFindRemoteInstType(Rm_Transport *transports,
                                            Rm_InstType remoteInstType);

#ifdef __cplusplus
}
#endif

#endif /* RM_TRANSPORTLOC_H_ */

