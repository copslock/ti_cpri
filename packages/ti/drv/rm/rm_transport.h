/**
 *   @file  rm_transport.h
 *
 *   @brief   
 *      This is the RM include file for the generic transport interface 
 *      used by RM to exchange RM control signals and data between RM instances
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

#ifndef RM_TRANSPORT_H_
#define RM_TRANSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes */
#include <stdbool.h>

/* RM includes */
#include <ti/drv/rm/rm.h>

/**
@addtogroup RM_TRANSPORT_API
@{
*/

/**
 * @brief Maximum size of RM transport packet 
 */
#define RM_TRANSPORT_PACKET_MAX_SIZE_BYTES (256)

/**
 * @brief RM transport handle
 */
typedef void *Rm_TransportHandle;

/**
 * @brief Application transport handle.  Void casted application transport
 *        data structure.  The Rm_AppTransportHandle provided by the
 *        application could be anything from a queue number to a pointer
 *        to an application transport data structure.  RM will provide
 *        a Rm_AppTransportHandle to the application any time RM 
 *        wants to alloc or send a packet via the transport callouts
 */
typedef void *Rm_AppTransportHandle;

/**
 * @brief A void pointer to the start of a registered application transport
 *        packet buffer.  The Rm_PacketHandle may be different from the 
 *        Rm_Packet pointer based on the application transport.  For example,
 *        for a QMSS based transport the Rm_PacketHandle may point to the
 *        beginning of a Host descriptor where as the Rm_Packet pointer would
 *        point to the beginning of the data buffer linked with the Host
 *        descriptor.
 */
typedef void *Rm_PacketHandle;

/**
 * @brief RM packet types
 */
typedef enum {
    /** Allocate/Free request packet */
    Rm_pktType_RESOURCE_REQUEST = 0,
    /** Allocate/Free response packet */
    Rm_pktType_RESOURCE_RESPONSE,
    /** RM NameServer mapping request */
    Rm_pktType_NAMESERVER_REQUEST,
    /** RM NameServer mapping response */
    Rm_pktType_NAMESERVER_RESPONSE
} Rm_pktType;

/**
 * @brief RM transport layer packet
 */
typedef struct {
    /** Packet length in bytes.  Written by application when allocated */
    uint32_t   pktLenBytes;
    /** Type of RM packet contained in data byte array */
    Rm_pktType pktType;
    /** Byte array containing RM packet data */
    char       data[RM_TRANSPORT_PACKET_MAX_SIZE_BYTES];
} Rm_Packet;

/** 
 * @brief RM transport callout functions used by RM to allocate and send
 *        RM packets via the application data paths between RM instances
 */
typedef struct {
    /**
     *  @b Description
     *  @n  
     *      This function pointer describes the RM transport layer packet
     *      allocation function.  The application must supply an alloc function
     *      to RM instances at transport registration if the RM instance is
     *      intended to communicate with other RM instances.  The function
     *      provided by the application must match this prototype.  The provided
     *      function implements the allocation of packet buffers from the 
     *      application data path that the RM transport will use to send
     *      messages between RM instances.  The Rm_Packet pointer will point to
     *      start of the data buffer containing the RM packet data.  The
     *      pktHandle will point to the start of the application data path
     *      "packet" data structure.  The Rm_Packet pointer and pktHandle cannot
     *      be NULL.  They will either be different or the same value based on
     *      the application transport.
     *
     *  @param[in]  appTransport
     *      Application transport handle to allocate packet from.  This value
     *      is provided by the application at transport registration.
     *
     *  @param[in]  pktSize
     *      Size of buffer needed by RM for the RM packet.  The application
     *      must place this value in the pktLenBytes field of the Rm_Packet.
     *      after the buffer has been allocated.
     *
     *  @param[out]  pktHandle
     *      Pointer to the start of the application's transport "packet".
     *      Could be the pointer to the start of a QMSS descriptor,
     *      network packet, etc.
     *
     *  @retval
     *      Success - Pointer to allocated packet buffer.
     *  @retval
     *      Failure - NULL
     */
    Rm_Packet *(*rmAllocPkt)(Rm_AppTransportHandle appTransport,
                             uint32_t pktSize,
                             Rm_PacketHandle *pktHandle);
    /**
     *  @b Description
     *  @n
     *      This function pointer describes the RM transport layer packet
     *      send function.  The application must supply a send function
     *      to RM instances at transport registration if the RM instance is
     *      intended to communicate with other RM instances.  The function
     *      provided by the application must match this prototype.  The provided
     *      function implements the sending of application data path packets,
     *      encapsulating RM packets, between RM instances.  The pktHandle will
     *      point to the start of the application data path "packet" data
     *      structure.
     *
     *      RM assumes that the application will free application data
     *      path packet and the buffer containing the Rm_Packet after
     *      the send operation.
     *
     *  @param[in]  appTransport
     *      Application transport handle to send packet on.  This value
     *      is provided by the application at transport registration.
     *
     *  @param[in]  pktHandle
     *      Pointer to the start of the application's transport "packet".
     *      Could be the pointer to the start of a QMSS descriptor,
     *      network packet, etc.
     *
     *  @retval
     *      0   - Packet sent okay.
     *  @retval
     *      < 0 - Packet send failed.
     */
    int32_t (*rmSendPkt)(Rm_AppTransportHandle appTransport,
                         Rm_PacketHandle pktHandle);
} Rm_TransportCallouts;

/**
 * @brief RM transport registration configuration structure
 */
typedef struct {
    /** RM Handle for which the transport is being registered */
    Rm_Handle              rmHandle;
    /** Application transport handle associated with the registered
     *  transport.  This value can be anything that helps the application
     *  identify the application transport "pipe" for which a RM packet
     *  should be sent on.  For example, a QMSS queue number, network
     *  port data structure, etc. */
    Rm_AppTransportHandle  appTransportHandle;
    /** Remote RM instance type at other end of the application transport
     *  "pipe". */
    Rm_InstType            remoteInstType;
    /** Pointers to application implemented transport APIs.  The APIs for the
     *  provided functions must match the prototypes defined in the
     *  #Rm_TransportCallouts structure.  Callouts need to be defined
     *  for each registered transport.  However, based on the application's
     *  implementation of transport code the same function can be provided
     *  for each transport registration.  In these cases, the transport
     *  must mux/demux the packets it receives from RM based on the
     *  #Rm_TransportHandle. */
    Rm_TransportCallouts   transportCallouts;
} Rm_TransportCfg;

/**
 * @brief RM transport reconfiguration structure
 */
typedef struct {
    /** Application transport handle associated with the registered
     *  transport.  This value can be anything that helps the application
     *  identify the application transport "pipe" for which a RM packet
     *  should be sent on.  For example, a QMSS queue number, network
     *  port data structure, etc. */
    Rm_AppTransportHandle  appTransportHandle;
    /** Pointers to application implemented transport APIs.  The APIs for the
     *  provided functions must match the prototypes defined in the
     *  #Rm_TransportCallouts structure.  Callouts need to be defined
     *  for each registered transport.  However, based on the application's
     *  implementation of transport code the same function can be provided
     *  for each transport registration.  In these cases, the transport
     *  must mux/demux the packets it receives from RM based on the
     *  #Rm_TransportHandle. */
    Rm_TransportCallouts   transportCallouts;
} Rm_TransportReCfg;

/**
 *  @b Description
 *  @n
 *      This function is used to register transports with RM for sending and
 *      receiving packets between RM instances over application transport
 *      data paths.
 *
 *      RM Transport Restrictions:
 *      a) RM Servers cannot register with other Servers
 *      b) RM CDs cannot register with other CDs
 *      c) RM Clients cannot register with other Clients
 *      d) Clients cannot register with more than one CD or Server
 *      e) Clients cannot register with both a CD and Server (either or)
 *      f) CDs cannot register with more than one Server
 *
 *  @param[in]  transportCfg
 *      Pointer to the transport registration configuration structure.
 *
 *  @param[out] result
 *      Pointer to a signed int used to return any errors encountered during
 *      the transport registration process.
 *
 *  @retval
 *      Success - RM_TransportHandle and result = #RM_OK
 *  @retval
 *      Failure - NULL RM_TransportHandle and
 *                result = #RM_ERROR_INVALID_REMOTE_INST_TYPE
 *  @retval
 *      Failure - NULL RM_TransportHandle and
 *                result = #RM_ERROR_ALREADY_REGD_SERVER_OR_CD
 *  @retval
 *      Failure - NULL RM_TransportHandle and
 *                result = #RM_ERROR_TRANSPORT_ALLOC_PKT_NOT_REGD
 *  @retval
 *      Failure - NULL RM_TransportHandle and
 *                result = #RM_ERROR_TRANSPORT_SEND_NOT_REGD 
 */
Rm_TransportHandle Rm_transportRegister(const Rm_TransportCfg *transportCfg,
                                        int32_t *result);

/**
 *  @b Description
 *  @n
 *      This function reconfigures an existing transport handle using the
 *      provided transport configurations
 *
 *  @param[in]  transportHandle
 *      Transport handle to unregister.  The RM instance that the handle will
 *      be unregistered from is contained within the transportHandle.
 *
 *  @param[in]  transportReCfg
 *      Pointer to the transport registration configuration structure.
 *
 *  @retval
 *      Success - #RM_OK
 *  @retval
 *      Failure - #RM_ERROR_TRANSPORT_HANDLE_DOES_NOT_EXIST
 */
int32_t Rm_transportReconfig(Rm_TransportHandle transportHandle,
                             const Rm_TransportReCfg *transportReCfg);

/**
 *  @b Description
 *  @n
 *      This function unregisters the provided transportHandle from the 
 *      RM instance
 *
 *  @param[in]  transportHandle
 *      Transport handle to unregister.  The RM instance that the handle will
 *      be unregistered from is contained within the transportHandle.
 *
 *  @retval
 *      Success - #RM_OK
 *  @retval
 *      Failure - #RM_ERROR_TRANSPORT_HANDLE_DOES_NOT_EXIST
 */
int32_t Rm_transportUnregister(Rm_TransportHandle transportHandle);

/**
 *  @b Description
 *  @n
 *      This function returns the RM instance name from which the service
 *      encapsulated within the RM packet originated
 *
 *      Restrictions: This API is only valid for Rm_pktType_RESOURCE_REQUEST
 *                    and Rm_pktType_NAMESERVER_REQUEST packet types
 *
 *  @param[in]  pkt
 *      RM packet to extract service source instance name from.
 *
 *  @param[out] serviceInstName
 *      Pointer to a character array that will contain the RM instance name
 *      that originated the service request.  The character array MUST be
 *      #RM_NAME_MAX_CHARS bytes in length
 *
 *  @param[in]  charBufLen
 *      Length of the provided pktInstName buffer
 *
 *  @retval
 *      Success - #RM_OK
 *  @retval
 *      Failure - #RM_ERROR_PKT_AND_SERVICE_SRC_NOT_AVAIL
 *      Failure - #RM_ERROR_SRC_NAME_BUF_INVALID_SIZE
 */
int32_t Rm_receiveGetPktServiceSrcName(const Rm_Packet *pkt,
                                       char *serviceInstName,
                                       int32_t charBufLen);

/**
 *  @b Description
 *  @n
 *      This function returns the RM instance name from which the RM packet
 *      originated
 *
 *      Restrictions: This API is only valid for Rm_pktType_RESOURCE_REQUEST
 *                    and Rm_pktType_NAMESERVER_REQUEST packet types
 *
 *  @param[in]  pkt
 *      RM packet to extract packet source instance name from.
 *
 *  @param[out] pktInstName
 *      Pointer to a character array that will contain the RM instance name
 *      that originated the RM request packet.  The character array MUST be
 *      #RM_NAME_MAX_CHARS bytes in length
 *
 *  @param[in]  charBufLen
 *      Length of the provided pktInstName buffer
 *
 *  @retval
 *      Success - #RM_OK
 *  @retval
 *      Failure - #RM_ERROR_PKT_AND_SERVICE_SRC_NOT_AVAIL
 *      Failure - #RM_ERROR_SRC_NAME_BUF_INVALID_SIZE
 */
int32_t Rm_receiveGetPktSrcName(const Rm_Packet *pkt,
                                char *pktInstName,
                                int32_t charBufLen);

/**
 *  @b Description
 *  @n
 *      This function is called by the application when it has received a RM
 *      packet for processing.  The application provides the transportHandle
 *      associated with the RM instance that should receive the packet and
 *      a pointer to the data buffer containing the Rm_Packet.
 *
 *      RM assumes that the application will free the data buffer containing
 *      the Rm_Packet after RM has finished processing the packet.
 *
 *  @param[in]  transportHandle
 *      RM transportHandle containing the instance that should process the
 *      received packet.
 *
 *  @param[in] pkt
 *      Pointer to the data buffer containing the Rm_Packet
 *
 *  @retval
 *      Success - #RM_OK
 *  @retval
 *      Failure - < #RM_OK
 */
int32_t Rm_receivePacket(Rm_TransportHandle transportHandle,
                         const Rm_Packet *pkt);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* RM_TRANSPORT_H_ */
