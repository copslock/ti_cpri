/******************************************************************************
 * FILE PURPOSE:  Top level interface file for NWAL Module
 ******************************************************************************
 * FILE NAME:   nwal.h
 *
 * DESCRIPTION: NWAL API definitions
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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
 */
/* ========================================================================= */
/**
 *   @file  nwal.h
 *
 *   path  ti/drv/nwal/nwal.h
 *
 *   @brief  Network Adaptation Layer Unit sub-system API and Data Definitions
 *
 */

/**  @mainpage Network Adaptation Layer
 *
 *   @section intro  Introduction
 *
 *   The network adaptation layer provides high level driver functionality
 *   abstracting NetCP LLDs PA and SA.
 *
 *  The driver can be used by application to offload hardware accelerator
 *  functionality for packets to and from network offered in Keystone family
 *  of devices.APIs exposed by NWAL provides both blocking/Synchronous and
 *  non blocking/Asynchronous support for the configuration which needs to
 *  be sent to NetCP.In the case of non blocking APIs, result for
 *  configuration is returned to application via a callback. Module also
 *  provides optional functionality of Transmit MAC/IPSec/IP/UDP header
 *  generation for outgoing packets
 *
 *      - Classification of packets based on L2: MAC header fields
 *      - Classification of packets based on L3: IP header fields
 *      - Routing of packets to host based on L4 UDP or L5 GTPU ID
 *      - Unidirectional IPSec SA creation and deletion
 *      - Unidirectional IPSec Security Policy creation and deletion
 *
 *   Following will be the external dependencies for the module:
 *      - Initialization of Queue Manager Subsystem
 *      - Initialization of memory buffer pool with packet DMA resources
 *        including descriptors.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2010-2012 Texas Instruments, Inc.
 *  \par
 */


#ifndef _NWAL_H
#define _NWAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * Shut off: remark #880-D: parameter "descType" was never referenced
*
* This is better than removing the argument since removal would break
* backwards compatibility
*/
#ifdef _TMS320C6X
#pragma diag_suppress 880
#pragma diag_suppress 681
#elif defined(__GNUC__)
/* Same for GCC:
* warning: unused parameter descType [-Wunused-parameter]
*/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
/* System level header files */
#include <stdint.h>
#include <stdlib.h>
#include <ti/drv/nwal/nwal_netcp.h>
#include <ti/drv/nwal/nwal_tune.h>

/* Define NWAL Driver Module as a master group in Doxygen format and
 * add all NWAL Driver API
 * definitions to this group.
 */
/** @defgroup nwal_module NWAL Module API
 *  @{
 */
/** @} */

/** @defgroup nwal_api_functions NWAL API's
 *  @ingroup nwal_module
 */
/** @defgroup nwal_api_structures NWAL API Data Structures
 *  @ingroup nwal_module
 */
/** @defgroup nwal_api_constants NWAL API Constants
 *  @ingroup nwal_module
 */
/** @defgroup ExternalCallbacks NWAL External Call backs
 *  @ingroup nwal_module
 */

/**
 *   @defgroup nwal_RetValue NWAL API Return Values
 *   @ingroup nwal_module
 */
/**  @ingroup nwal_RetValue */
typedef int16_t nwal_RetValue;
/* @{ */
/**
 *  @def  nwal_OK
 *        NWAL return code -- Function executed successfully
 */
#define nwal_OK                                 0

/**
 *  @def  nwal_TRANS_COMPLETE
 *        NWAL return code -- NWAL Transaction complete in synchronous
 *        mode.Callback will not be called.
 */
#define nwal_TRANS_COMPLETE                     1

/**
 *  @def  nwal_ERR_INVALID_CMD_DEST
 *        Invalid Command Destination Q from PA
 */
#define nwal_ERR_INVALID_CMD_DEST               -1

/**
 *  @def  nwal_ERR_NO_FREE_CMD_DESC
 *        No free command Descriptor available
 */
#define nwal_ERR_NO_FREE_CMD_DESC               -2

/**
 *  @def  nwal_ERR_PA
 *        Error returned by PA LLD/PA
 */
#define nwal_ERR_PA                             -3


/**
 *  @def  nwal_ERR_NO_FREE_CMD_BUF
 *        No free command Descriptor available
 */
#define nwal_ERR_NO_FREE_CMD_BUF                -4


/**
 *  @def  nwal_ERR_INVALID_HANDLE
 *        Invalid Handle
 */
#define nwal_ERR_INVALID_HANDLE                 -5

/**
 *  @def  nwal_ERR_NO_FREE_BUF
 *        No free buffer
 */
#define nwal_ERR_NO_FREE_BUF                    -6

/**
 *  @def  nwal_ERR_INVALID_ADDR
 *        Invalid Address for Lookup
 */
#define nwal_ERR_INVALID_ADDR                   -7

/**
 *  @def  nwal_ERR_INVALID_PARAM
 *        Invalid Parameter to API
 */
#define nwal_ERR_INVALID_PARAM                  -8

/**
 *  @def  nwal_ERR_PA_PREV_REQ
 *        Error returned by PAfor Dependent handle. Example an IP configuration
 *        request has failed because of error in MAC configuration request
 */
#define nwal_ERR_PA_PREV_REQ                    -9

/**
 *  @def  nwal_ERR_MEM_ALLOC
 *        Error Allocating memory to NWAL
 */
#define nwal_ERR_MEM_ALLOC                      -10

/**
 *  @def  nwal_ERR_SA
 *        Error returned by PA LLD/PA
 */
#define nwal_ERR_SA                             -11

/**
 *  @def  nwal_ERR_RES_UNAVAILABLE
 *        Error Resource not available
 */
#define nwal_ERR_RES_UNAVAILABLE                -12

/**
 *  @def  nwal_ERR_INVALID_KEY
 *        Error Resource not available
 */
#define nwal_ERR_INVALID_KEY                    -13

/**
 *  @def  nwal_ERR_INVALID_PREV_HANDLE_STATE
 *        Dependent NWAL handle is not yet active
 */
#define nwal_ERR_INVALID_PREV_HANDLE_STATE       -14

/**
 *  @def  nwal_ERR_INVALID_STATE
 *        Incorrect State for API call.
 */
#define nwal_ERR_INVALID_STATE                   -15

/**
 *  @def  nwal_ERR_POWER_DOMAIN_FAIL
 *        Failure powering on the domain.
 */
#define nwal_ERR_POWER_DOMAIN_FAIL              -16

 /**
 *  @def  nwal_ERR_PA_DOWNLOAD
 *        Error downloading PA firmware.
 */
#define nwal_ERR_PA_DOWNLOAD                    -17

/**
 *  @def  nwal_ERR_CPPI
 *        Error from CPPI module.
 */
#define nwal_ERR_CPPI                           -18

/**
 *  @def  nwal_ERR_QMSS
 *        Error from CPPI module.
 */
#define nwal_ERR_QMSS                           -19

/**
 *  @def  nwal_ERR_DEST_MISMATCH
 *        Error Destination Mismatch
 */
#define nwal_ERR_DEST_MISMATCH                  -20

/**
 *  @def  nwal_ERR_SA_NOT_ENABLED
 *        SA not enabled for NWAL library
 */
#define nwal_ERR_SA_NOT_ENABLED                  -21

/**
 *  @def  nwal_ERR_INVALID_PROC_ID
 *        ErrorIncorrect Processor ID
 */
#define nwal_ERR_INVALID_PROC_ID                 -22

/**
 *  @def  nwal_ERR_L2L3_UNAVAILABLE
 *        Error Resource not available
 */
#define nwal_ERR_L2L3_UNAVAILABLE                -23


/**
 *  @def  nwal_ERR_PORT_UNAVAILABLE
 *        Error Resource not available
 */
#define nwal_ERR_PORT_UNAVAILABLE                -24


/**
 *  @def  nwal_ERR_PKT_LIB
 *        Unexpected Error from Packet LIB module
 */
#define nwal_ERR_PKT_LIB                         -25

/**
 *  @def  nwal_ERR_POLICY_CHECK_FAIL
 *        Error during Policy verification
 */
#define nwal_ERR_POLICY_CHECK_FAIL              -26
/*  @}  */  /* ingroup */

/**
 * @ingroup nwal_api_constants
 * @brief  Handle owned by NWAL abstracted to Application.
 * @details Application to use this handle to identify NWAL configuration
 */
typedef void * nwal_Handle;
#define nwal_HANDLE_INVALID     NULL

/**
 * @ingroup nwal_api_constants
 * @brief  Handle owned by Application.
 * @details NWAL uses this handle while interfacing to application
 */
typedef void * nwal_AppId;

/**
 * @ingroup nwal_api_constants
 * @brief  NWAL Instance abstracted to Application.
 * @details Application to use this handle to identify an NWAL Instance
 *
 */
typedef void * nwal_Inst;

/**
 * @ingroup nwal_api_constants
 * @brief  Transaction ID type
 * @details Type of transaction ID for the non Blocking API call
 */
typedef uint16_t nwal_TransID_t;

/**
 * @ingroup nwal_api_constants
 * @brief  Boolean Type definition
 * @details Boolean definitions for Module
 */
typedef uint16_t nwal_Bool_t;
/* @{ */
/**
 *   @ingroup nwal_api_constants
 *   @brief TRUE
 */
#define  nwal_TRUE  1
/*  @} */
/* @{ */
/**
 *  @ingroup nwal_api_constants
 *  @brief FALSE
 */
#define  nwal_FALSE  0
/*  @} */
/*  @}  */  /* ingroup */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   MAC packet Type
 *  @brief  MAC packet Type
 *
 *  @details MAC packet Type
 */
/* @{ */
/**
 *  @def  NWAL_MAC_PKT_UNKNOWN
 *        Unknown Mac Packet Type from NetCP.
 *        Application would be required to retrieve the correct type
 */
typedef uint16_t nwal_macPktType_t;
#define NWAL_MAC_PKT_UNKNOWN                0x1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_MAC_PKT_UNICAST
 *        Unicast MAC packet
 */

#define NWAL_MAC_PKT_UNICAST                0x2
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_MAC_PKT_BROADCAST
 *        Broadcast MAC packet
 */

#define NWAL_MAC_PKT_BROADCAST              0x3
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_MAC_PKT_MULTICAST
 *        Multicast MAC packet
 */

#define NWAL_MAC_PKT_MULTICAST              0x4
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_MAC_PKT_LOCAL_ADDR_MISMATCH
 *        Packets not for local MAC point
 */

#define NWAL_MAC_PKT_LOCAL_ADDR_MISMATCH    0x5
/*  @}  */
/** @} */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   RX Flag 1
 *  @brief  List of NetCP actions completed for incoming packet
 *
 *  @details List of NetCP actions completed for incoming packet
 */
/* @{ */
/**
 *  @def  NWAL_RX_FLAG1_META_DATA_VALID
 *        Meta data information is valid. If this bit is not set
 *        all other fields in meta data related to packet will not be
 *        valid
 */
typedef uint32_t nwal_rxFlag1_t;
#define NWAL_RX_FLAG1_META_DATA_VALID               0x80000000
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_UNKNOWN
 *        IPV4 Header Checksum verification result not done NetCP and hence
 *        unknown
 */
#define NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_MASK         0x00000003
#define NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_SHIFT        0
#define NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_UNKNOWN      0
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_NACK
 *        IPV4 Header Checksum verification failed at NetCP
 */
#define NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_NACK         1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_ACK
 *        IPV4 Header Checksum verification passed at NetCP
 */
#define NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_ACK          3
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_UNKNOWN
 *        L4 Header Checksum verification result not done NetCP and hence
 *        unknown
 */
#define NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_MASK             0x0000000C
#define NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_SHIFT            2
#define NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_UNKNOWN          0
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_NACK
 *        IPV4 Header Checksum verification failed at NetCP
 */
#define NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_NACK           1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_ACK
 *        IPV4 Header Checksum verification passed at NetCP
 */
#define NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_ACK            3
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_RX_IPSEC_CRYPTO_DONE_OK
 *        IPSec Decryption/Authentication passed
 */
#define NWAL_RX_IPSEC_CRYPTO_DONE_OK                0x00000020
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_RX_IPSEC_WINDOW_DONE_OK
 *        IPSec Decryption/Authentication passed
 */
#define NWAL_RX_IPSEC_WINDOW_DONE_OK                0x00000040
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_RX_IP_FRAGMENT_PKT
 *        IP Fragment Packet to be reassembled by Host which had not
 *        gone through full PA classification.
 */
#define NWAL_RX_IP_FRAGMENT_PKT                     0x00000080
/*  @}  */
/** @} */


/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   NWAL Route types
 *  @brief  NWAL Route types
 *
 *  @details NWAL Route types.Used to specify the mode of priority-based routing 
 *   or interface based routing.
 *  PASS forwards the matched packets to the desired QoS queue which is equal
 *  to the base queue plus an offset specified by the VLAN priority or DSCP value
 *  in prority-based routing. 
 *  For interface based routing,PASS forwards the matched packets to the desired 
 *  host queue which is equal to the base queue plus an offset specified by the
 *  receive emac port (interface) number number with the CPPI
 *  flow which is equal to the base flow number plus the EMAC port (interface) number
 *  optionally in interface-based routing..
 */
/* @{ */
/**
 *  @def  NWAL_ROUTE_VLAN_PRIORITY
 *        Route by using VLAN bits as priority
 */
typedef uint32_t nwalRouteType_t;

#define NWAL_ROUTE_VLAN_PRIORITY        0x1
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_ROUTE_DSCP_PRIORITY
 *        Route by using DSCP value as priority
 */
#define NWAL_ROUTE_DSCP_PRIORITY        0x2
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_ROUTE_RX_INTF
 *        Route by using EMAC port (interface) number as destination queue offset
 */
#define NWAL_ROUTE_RX_INTF              0x4
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_ROUTE_RX_INTF
 *        Route by using EMAC port (interface) number as both
 *        destination queue and CPPI flow offset
 */
#define NWAL_ROUTE_RX_INTF_W_FLOW       0x8
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_ROUTE_PKTTYPE_EQOS
 *        Route by using priority map for enhanced QoS to support L2 Shapper
 */
#define NWAL_ROUTE_PKTTYPE_EQOS       0x10
/** @} */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   EMAC Port Id
 *  @brief  Incoming/Outgoing Port Id: 1 based
 *
 *  @details Incoming/Outgoing Port Id: 1 based
 */
/* @{ */
/**
 *  @def  NWAL_ENET_PORT_UNKNOWN
 *        ENET Port Not Applicable.
 *        For TX direction switch will redirect to appropriate port.
 */
typedef uint16_t nwal_enetPort_t;
#define NWAL_ENET_PORT_UNKNOWN         0
/*  @}  */
/** @} */

/**
 *  @ingroup nwal_api_structures
 *  @brief NWAL Packet meta data information for incoming packet
 *
 *  @details The parameters in this structure are used to provide additional
 *           details for the incoming packet
 */
typedef struct {
    nwal_AppId          appId;          /**<  Application ID registered during
                                          *   configuration for the packet
                                          *   stream
                                          */
    Ti_Pkt*             pPkt;           /**<  Packet received from NetCP */
    nwal_rxFlag1_t      rxFlag1;        /**<  NetCP completed actions
                                           *  @see nwal_rxFlag1_t
                                           */
    nwal_macPktType_t   pktType;        /**<  Packet Type
                                           *  @see nwal_macPktType_t
                                           */
    uint32_t            startOffset;    /**<  Start Offset for the packet */
    uint32_t            pktLen;         /**<  Length of the packet */
    uint16_t            l3OffBytes;     /**<  Offset in bytes indicating
                                           *  start of IP header.
                                           */
    uint16_t            l4ProtoType;    /**<  Protocol type for Layer 4 in
                                           *  IP header (UDP, ICMP)
                                           */
    uint16_t            l4OffBytes;     /**<  Offset in bytes indicating
                                           *  start of L4:UDP/TCP/ICMP header.
                                           */
    uint16_t            ploadOffBytes;  /**<  Offset in bytes to the start
                                           *  of the payload.Value 0 indicates
                                           *  not set.
                                           */
    uint16_t            ploadLen;       /**<  Length of payload. Value 0
                                          *   indicates not set.
                                          */
    nwal_enetPort_t     enetPort;       /**<  RX Enet Port.
                                          *   @see nwal_enetPort_t
                                          */
}nwalRxPktInfo_t;



/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   TX Flag 1
 *  @brief  List of actions to be completed by NetCP for outgoing packet
 *
 *  @details List of actions to be completed by NetCP for outgoing packet
 */
/* @{ */
/**
 *  @def  NWAL_TX_FLAG1_META_DATA_VALID
 *        Meta data information is valid. If this bit is not set
 *        all other fields in meta data related to packet will not be
 *        valid
 */
typedef uint32_t nwal_txFlag1_t;
#define NWAL_TX_FLAG1_META_DATA_VALID                   0x80000000
/*  @}  */


/* @{ */
/**
 *  @def  NWAL_TX_FLAG1_DO_IPV4_CHKSUM
 *        IPV4 Header Checksum offload to hardware. Only supported for
 *        innermost IP header of the packet in the case of tunnel configuration
 *        Packet received at NWAL already should have rest of the header
 *        populated
 */
#define NWAL_TX_FLAG1_DO_IPV4_CHKSUM                    0x00000001
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_TX_FLAG1_DO_UDP_CHKSUM
 *        UDP checksum to be computed at NetCP
 */
#define NWAL_TX_FLAG1_DO_UDP_CHKSUM                     0x00000002
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_TX_FLAG1_DO_TCP_CHKSUM
 *        TCP checksum to be computed at NetCP
 */
#define NWAL_TX_FLAG1_DO_TCP_CHKSUM                     0x00000004
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_TX_FLAG1_DO_UPDATE_ETHER_LEN
 *        Update Length for the Ethernel Header eg: 802.3.Only one Length field
 *        update is currently supported.Offset to the length would need to be
 *        provided through etherLenOffBytes configuration
 */
#define NWAL_TX_FLAG1_DO_UPDATE_ETHER_LEN               0x00000008
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_TX_FLAG1_DO_IPSEC_ESP_CRYPTO
 *        IPSec Crypto/Authentication to be done at NetCP
 */
#define NWAL_TX_FLAG1_DO_IPSEC_CRYPTO_MASK              0x00000060
#define NWAL_TX_FLAG1_DO_IPSEC_ESP_CRYPTO               0x00000020
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_TX_FLAG1_DO_IPSEC_AH_CRYPTO
 *        IPSec Crypto/Authentication to be done at NetCP
 */
#define NWAL_TX_FLAG1_DO_IPSEC_AH_CRYPTO               0x00000040
/*  @}  */

/** @} */
/**
 *  @ingroup nwal_api_constants
 *  @brief  Maximum size allowed for  Authentication Tag bytes
 *
 *  @details  Maximum size allowed for  Authentication Tag bytes
 */
#define NWAL_IPSEC_AH_MAX_AUTH_TAG_BYTES    pa_MAX_PATCH_BYTES

/**
 *  @ingroup nwal_api_structures
 *  @brief NWAL Packet meta data information for outgoing packet
 *
 *  @details The parameters in this structure are used to provide additional
 *           details for the outgoing packet. In the case of NWAL updating the
 *           packet header structure content will be modified internally
 *           within NWAL to reflect correct offset of the packet headers
 */
typedef struct {
    Ti_Pkt*             pPkt;           /**<  Packet to be transmitted through
                                          *   NetCP: PA/SA/EMAC
                                          */
    nwal_txFlag1_t      txFlag1;        /**<  NetCP completed actions
                                          *   @see nwal_txFlag1_t
                                          */
    nwal_Bool_t         lpbackPass;     /** Loopback the packet at NetCP PASS.
                                          * Useful for debugging any
                                          * classification action at NetCP
                                          */

    nwal_enetPort_t     enetPort;       /**<  TX Enet Port 1 based.Eg:
                                          *   For transmitting through
                                          *   first port configure as 1
                                          *   @see nwal_enetPort_t
                                          *   Configuring to 
                                          *   NWAL_ENET_PORT_UNKNOWN will let
                                          *   CPSW decide on outgoing port.
                                          *   In case of application invoking
                                          *   nwal_initPSCmdInfo API it would 
                                          *   need to either select a port or 
                                          *   or configure to 
                                          *   NWAL_ENET_PORT_UNKNOWN. Switching
                                          *   between types during packet TX
                                          *   is not supported
                                         */

    /*** Below configuration would be required for TX packets with
     *   headers already populated by Application. In the case of NWAL
     *   prepending the protocol headers, fields will be modified
     *   inside the module
    ***/
    uint32_t            mtuSize;       /** Maximum MTU size for the IP
                                           * fragmentation functionality at
                                           * NetCP. In the case of tunnel IP
                                           * outer IP only will be fragmented
                                           * Configuring NULL will disable the
                                           * fragmentation at NetCP
                                           */
    uint32_t            startOffset;    /**<  Start Offset for the packet */
    uint16_t            saOffBytes;     /**<  Offset from base of the packet
                                          *   to the header
                                          *   of protocol per the following
                                          *   list:
                                          *   IPSEC ESP with AH: IP header
                                          *   IPSEC ESP: ESP Header
                                          */
    uint16_t            saPayloadLen;    /**< Length of the payload starting
                                           *  from saOffBytes to the end of
                                           *  the protocol
                                           */
    uint16_t            saAhIcvOffBytes; /**< Offset to the ICV field in the
                                           *  case of IPSec AH mode for
                                           *  authentication tag insertion
                                           *  by NETCP.Reset to
                                           *  zero in the case of ESP mode
                                           */
    uint16_t            saAhMacSize;      /**< Size of the authentication tag
                                           *  to be inserted by NetCP in the
                                           *  case of IPSec AH mode. Reset to
                                           *  zero in the case of ESP mode
                                           *  Max size: @ref
                                           *  NWAL_IPSEC_AH_MAX_AUTH_TAG_BYTES
                                           */
    uint16_t            etherLenOffBytes;/**< Byte offset for updating Ethernet
                                           *  Length in the case of 802.3 header
                                           *  Length field is assumed to be of
                                           *  2 octet length by NWAL. The update
                                           *  will be done along with fragmentation
                                           *  by NetCP.In the case of tunnel packet
                                           *  only outer IP fragmentation is supported
                                           * through NetCP
                                           */
    uint16_t            ipOffBytes;      /**<  Offset in bytes indicating
                                          *   start for inner most IP header.
                                          *   Would be required for packet
                                          *   with only one level IP header.
                                          *   The configuration will be used
                                          *   for IP header checksum
                                          *   offload to hardware
                                          */
    uint16_t            l4OffBytes;     /**<  Offset in bytes indicating
                                          *   start of UDP/TCP Header for
                                          *   checksum computation
                                          */
    uint16_t            l4HdrLen;         /**<  Length of L4 TCP/UDP Header  */
    uint16_t            pseudoHdrChecksum;/**< Pseudo Header checksum for L4 */
    uint16_t            ploadLen;         /**<  Length of L4 payload */

}nwalTxPktInfo_t;

/**
 *  @ingroup nwal_api_structures
 *  @brief NWAL Data mode meta data payload information from NetCP
 *
 *  @details The parameters in this structure are used to provide additional
 *           details for the incoming payload from NetCP
 */
typedef struct {
    nwal_AppId          appId;     /**<  Application ID registered during
                                    *   configuration for the packet stream
                                    */
    Ti_Pkt*             pPkt;      /**<  Payload post Crypto action from NetCP
                                    */
    nwal_AppId          appCtxId;  /**< Application context ID passed in the
                                     *  nwal_sendDM() API side band Data mode
                                     *  request.
                                     */
    uint32_t            authTagLen;/**<  Length of Authentication Tag */
    uint32_t*           pAuthTag;  /**<  Authentication Tag */
}nwalDmRxPayloadInfo_t;


/**
 *  @ingroup nwal_api_structures
 *  @brief NWAL Data Mode Payload information for packet to SA
 *
 *  @details The parameters in this structure are used to provide
 *           details of payload being sent to SA for offloading
 *           Encrypt/Decrypt/Authentication
 */
typedef struct {
    Ti_Pkt*             pPkt;        /**< Payload to be transmitted to SA */
    nwal_AppId          appCtxId;    /**< Application context ID identifying
                                       *  the data mode request. The 32 bit ID
                                       *  will be received as echo back for the
                                       *  response from NetCP in
                                       *  nwalDmRxPayloadInfo_t
                                       */
    uint8_t             encOffset;   /**< Specify the offset to the
                                       *  encrypted/decrypted data in the
                                       *  packet in bytes
                                       */
    uint16_t            encSize;     /**< Specify the total number of bytes
                                       *  to be encrypted or decrypted
                                       */
    uint8_t*            pEncIV;      /**< Contain the initialization vectors
                                       *  used in certain encryption modes.
                                       *  @note: IV should be specified here
                                       *  in GCM/CCM mode
                                       */
    uint8_t             authOffset;  /**< Specify the offset to the
                                       *  authenticated data in the packet
                                       *  in bytes
                                       */
    uint16_t            authSize;    /**< Specify the total number of bytes
                                       *  to be authenticated
                                       */
    uint8_t*            pAuthIV;      /**< Contain the initialization vectors
                                       *   used in certain authentication
                                       *   modes.
                                       *   @note: IV should be specified
                                       *   here in GMAC mode
                                       */
    uint16_t            aadSize;      /**< Size of additional authenticated
                                       *   data in bytes
                                       */
    uint8_t*            pAad;         /**< Contain the additional
                                        *  authenticated data in
                                        *  GCM/CCM modes
                                        */
}nwalDmTxPayloadInfo_t;


/*** External Prototype definitions  ***/
/**
 * @ingroup ExternalCallbacks
 *
 * @brief  nwal_CmdCallBack Callback function for non blocking configuration
 *         request
 *
 *
 * @details The call back function need to be registered by application
 *          in the case of all asynchronous configuration command request
 *          to NetCP.
 *          Call back is initiated after results  for configuration request
 *          is available from NetCP module.
 *
 */
typedef void nwal_CmdCallBack(nwal_AppId        appId,
                              nwal_TransID_t    transId,
                              nwal_RetValue     ret);

/**
 * @ingroup ExternalCallbacks
 *
 * @brief  nwal_rxPktCallBack Callback function for incoming packets from NWAL
 *
 * @details The call back function need to be registered by application
 *  @param[in]  appCookie   Application cookie passed per poll API call.
 *  @param[in]  numPkts     Number of Packets.
 *                          Max Value @see NWAL_MAX_RX_PKT_THRESHOLD
 *  @param[in]  pPktInfo    Array of packets with meta information
 *  @param[in]  timestamp   Timestamp for incoming packet when callback is
 *                          initiated
 *  @param[out] pFreePkt    Array per packet. To be set by application to
 *                          indicate if packet needs to be freed by NWAL
 *                          after callback returns. Set to nwal_TRUE if
 *                          packet needs to be freed by NWAL
 */
typedef void nwal_rxPktCallBack(uint32_t            appCookie,
                                uint16_t            numPkts,
                                nwalRxPktInfo_t*    pPktInfo,
                                uint64_t            timestamp,
                                nwal_Bool_t*        pFreePkt);

/**
 * @ingroup ExternalCallbacks
 *
 * @brief  nwal_CmdPaStatsReply Callback function for statistics response
 *
 * @details The call back function will be called in the case of all
 *          asynchronous stats command request to NetCP. NWAL will call
 *          call back to provide the result for configuration request to
 *          application.
 */
typedef void nwal_CmdPaStatsReply (nwal_AppId       appId,
                                   nwal_TransID_t   transId,
                                   paSysStats_t     *stats);

/**
 * @ingroup ExternalCallbacks
 *
 * @brief  nwal_rxDmCallback function callback to be used for Data Mode
 *         payload received from SA
 *
 * @details The call back function need to be registered by application
 *  @param[in]  appCookie   Application cookie passed per poll API call.
 *  @param[in]  numPkts     Number of Packets.
 *                          Max Value @see NWAL_MAX_RX_PKT_THRESHOLD
 *  @param[in]  pDmPayload  Array of Data Mode Payload
 *  @param[out] pFreePkt    Array per packet. To be set by application
 *                          to indicate if packet needs to be freed by NWAL
 *                          after callback returns. Set to nwal_TRUE if
 *                          packet needs to be freed by NWAL
 */
typedef void nwal_rxDmCallback( uint32_t                appCookie,
                                uint16_t                numPkts,
                                nwalDmRxPayloadInfo_t*  pDmRxPayloadInfo,
                                nwal_Bool_t*            pFreePkt);

/**
 * @ingroup ExternalCallbacks
 *
 * @brief  nwal_rxReassemProc function callback being called from NWAL
 *         if NWAL_CTRL_CFG_PA_ASSISTED_REASSEM is enabled through @see nwal_control.
 *         Callback is called at Application for SW IP reassembly of
 *         fragmented packet.PA Sample reassembly routine paEx_reassemLibProc()
 *         can  optionally be used for reassembly
 *
 * @details The call back function need to be registered by application
 *  @param[in]  pPkt   Application cookie passed per poll API call.
 *  @param[in]  destQ  Applicable for PASS-assisted IP reassembly 
 *                     NWAL_CTRL_CFG_PA_ASSISTED_REASSEM: Application to push the 
 *                     fully reassembled packets destination queue 
 *                     
 */
 typedef int nwal_rxReassemProc(Ti_Pkt*    pPkt, NWAL_queueHnd destQ);
/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Configuration for action by NETCP when next route classification
 *          fails
 *  @brief  Action from NETCP when next route classification check fails
 *
 *  @details Configuration action to either terminate to host or discard
 *           packets in case next route classification fails. An example for
 *           MAC/L2 would be IP L3 classification fails
 */
/* @{ */
/**
 *  @def  NWAL_NEXT_ROUTE_FAIL_ACTION_DISCARD
 *        Discard all packets which does not match next route classification.
 *
 *
 */
typedef uint16_t nwal_nextRtFailAction_t;
#define NWAL_NEXT_ROUTE_FAIL_ACTION_DISCARD         0x0
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_NEXT_ROUTE_FAIL_ACTION_HOST
 *        Terminate next route mismatch packets to host for further
 *        processing
 *
 */
#define NWAL_NEXT_ROUTE_FAIL_ACTION_HOST            0x1

/*  @}  */
/** @}  */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Configuration for action by NETCP when classification matches
 *  @brief  Action from NETCP when classification check passes
 *
 *  @details Configuration actions in case classification matches at NetCP.
 */
/* @{ */
typedef uint16_t nwal_matchAction_t;
/**
 *  @def  NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE
 *        NetCP to continue parsing for next route when classification rule
 *        matches
 *
 */
#define NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE       0x0

/*  @}  */
/* @{ */
/**
 *  @def  NWAL_MATCH_ACTION_DISCARD
 *        Discard all packets which does not match next route classification.
 *
 *
 */
#define NWAL_MATCH_ACTION_DISCARD                   0x1
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_MATCH_ACTION_HOST
 *        NetCP to terminate packet at host if classification matches
 *
 */
#define NWAL_MATCH_ACTION_HOST                      0x2
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_MATCH_ACTION_EMAC
 *        NetCP to terminate packet at EMAC port
 *
 */
#define NWAL_MATCH_ACTION_EMAC                      0x4
/*  @}  */

/** @}  */



/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   IP Options Valid Parameters
 *  @brief  Bit map indicating valid Parameters for the configuration.
 *
 *  @details Defines valid parameters for IP Options
 */
/* @{ */
/**
 *  @def  NWAL_IP_OPT_VALID_PARAMS_L4_PROTO
 *        Valid Protocol Type.
 */
#define NWAL_IP_OPT_VALID_PARAMS_L4_PROTO       0x1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_IP_OPT_VALID_PARAMS_TOS
 *        Valid Type of Service
 */
#define NWAL_IP_OPT_VALID_PARAMS_TOS            0x2
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_IP_OPT_VALID_PARAMS_FLOW_LABEL
 *        Valid Flow Label
 */
#define NWAL_IP_OPT_VALID_PARAMS_FLOW_LABEL     0x4
/*  @}  */
typedef struct {
    uint16_t    validParams;/** Valid Parameter bit field */
    uint8_t     proto;      /** Layer 4 protocol: IANA assigned values */
    uint8_t     tos;        /** Will represent Traffic class if address
                              * family is IPv6
                              */
    uint32_t    flowLabel;  /** 20 bit lsbs will represent IPv6 flow label */
}nwalIpOpt_t;
/** @} */
/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   MAC Options Valid Parameters
 *  @brief  Bit map indicating valid Parameters for the configuration.
 *
 *  @details Defines valid parameters for MAC Options
 */
/* @{ */
/**
 *  @def  NWAL_MAC_OPT_VALID_PARAM_VLAN_ID
 *        Valid VLAN ID.
 */
/* valid param flags */
#define NWAL_MAC_OPT_VALID_PARAM_VLAN_ID    0x1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_MAC_OPT_VALID_PARAM_VLAN_PRIO
 *        Valid VLAN Priority
 */
#define NWAL_MAC_OPT_VALID_PARAM_VLAN_PRIO  0x2
/*  @}  */
/** @} */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Configuration for Frame format in MAC header
 *  @brief  MAC frame format configuration
 *
 *  @details MAC frame format configuration
 */
/* @{ */
/**
 *  @def  NWAL_MAC_OPT_FRAME_FORMAT_802_3
 *        802_3 Frame format.
 */
typedef uint16_t nwal_macOptFrameFormat_t;
#define NWAL_MAC_OPT_FRAME_FORMAT_802_3     0x1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_MAC_OPT_FRAME_FORMAT_DIX
 *        DIX Frame format.
 */

#define NWAL_MAC_OPT_FRAME_FORMAT_DIX       0x2
/*  @}  */
/** @} */
typedef struct  {
    uint16_t                    validParams;/** Valid Parameter Bit map */
    uint16_t                    vlanId;     /** VLAN ID */
    nwal_macOptFrameFormat_t    frameFormat;/** @see nwal_macOptFrameFormat_t
                                              */
    uint8_t                     vlanPrio;   /** VLAN Priority */
} nwalMacOpt_t;

/**
 * @brief  Defines the IP version type used.
 *
 * @details The packet accelerator module parses both IPv4 and IPv6 network
 *          layer headers.
 *          This group is used to distinguish which type of header will be
 *          used.
 */
/** @ingroup nwal_IpType */
typedef uint16_t nwal_IpType;
/* @{ */
/**
 *  @def  nwal_IPV4
 *        IPv4
 */
#define  nwal_IPV4  pa_IPV4

/**
 *  @def   nwal_IPV6
 *        IPv6
 */
#define  nwal_IPV6  pa_IPV6

/*  @}  */  /* ingroup */

/**
 *  @ingroup nwal_api_structures
 *  @{
 * @brief nwalMacAddr MAC address specification
 *
 * @details  This type is used to pass MAC addresses to the module.
 *           The most significant byte of the mac address is placed in
 *           array element 0.
 */
#define NWAL_MAC_ADDR_SIZE       6
typedef unsigned char nwalMacAddr_t[NWAL_MAC_ADDR_SIZE];
/** @} */


/**
 *  @ingroup nwal_api_structures
 *  @brief nwalMacParam_t  structure.
 *
 *  @details Configuration parameters for nwal_setMacIface() API
 */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Valid Parameter configuration for nwal_setMacIface API
 *  @brief  Valid Parameter configuration
 *
 *  @details Valid Parameter to configure optional parameters.
 */
/* @{ */
/**
 *  @def  NWAL_SET_MAC_VALID_PARAM_IFNUM
 *        Restrict MAC packets from NetCP if received from a particular
 *        Interface and destination MAC
 *
 */
#define NWAL_SET_MAC_VALID_PARAM_IFNUM          0x1
/** @} */

/* @{ */
/**
 *  @def  NWAL_SET_MAC_VALID_PARAM_VLAN_ID
 *        VLAN ID is valid
 *
 */
#define NWAL_SET_MAC_VALID_PARAM_VLAN_ID        0x2
/** @} */


/* @{ */
/**
 *  @def  NWAL_SET_MAC_VALID_PARAM_ROUTE_TYPE
 *        Route Type is valid
 *
 */
#define NWAL_SET_MAC_VALID_PARAM_ROUTE_TYPE     0x4
/** @} */

/* @{ */
/**
 *  @def  NWAL_SET_MAC_VALID_PARAM_REMOTE_MAC
 *        Remote Mac is valid 
 *
 */
#define NWAL_SET_MAC_VALID_PARAM_REMOTE_MAC     0x8
/** @} */
#define NWAL_SET_MAC_VALID_PARAM_ETHER_TYPE     0x10
/** @} */
/** @} */

typedef struct  {
    uint16_t                validParams;    /**< Valid Parameter configuration.
                                              *  @see nwalSetMacValidParam
                                              */
    uint16_t                ifNum;          /**< Interface ID: One based */
    uint16_t                vlanId;         /**< VLAN tag ID */
    nwalMacAddr_t           macAddr;        /**< Local MAC address. */
    nwalMacAddr_t           remMacAddr;     /**< Remote MAC address */
    nwal_matchAction_t      matchAction;    /**< Action upon matching
                                              *  classification rule at
                                              *  NetCP @see nwal_matchAction_t
                                              */
    nwal_nextRtFailAction_t failAction;     /**< Configuration for action
                                              *  when next route classification
                                              *  fails
                                              *  @see nwalNextRouteFailAction
                                              */
    int16_t                 appRxPktFlowId; /**< Optional: Application managed
                                              * Flow ID for any packet to host
                                              * from this classification
                                              * entry. In case if NWAL
                                              * managed flow needs to be used
                                              * set to NWAL_FLOW_NOT_SPECIFIED
                                              */
    NWAL_queueHnd           appRxPktQueue;  /**< Optional: Application managed
                                              * Queue handle for any packet
                                              * to host from this
                                              * classification entry.
                                              * In case if NWAL managed Queue
                                              * needs to be used set to
                                              * NWAL_QUEUE_NOT_SPECIFIED
                                              */
    nwalRouteType_t     routeType;          /** Optional: Routing Type */
    uint16_t            etherType;          /** Optional: Ethertype field */
    uint16_t            egress_switch_port;  /* only valid for EQOS mode */

}nwalMacParam_t;


/**
 *  @ingroup nwal_api_structures
 *  @{
 * @brief  nwalIpv4Addr IPv4 address specification
 *
 * @details  This type is used to pass IPv4 addresses to the module.
 *           The most significant byte of the IP address is placed in
 *           array element 0.
 */
#define NWAL_IPV4_ADDR_SIZE      4
typedef unsigned char nwalIpv4Addr_t[NWAL_IPV4_ADDR_SIZE];
/** @} */

/**
 *  @ingroup nwal_api_structures
 *  @{
 * @brief  nwalIpv6Addr IPv6 address specificiation
 *
 * @details  This type is used to pass IPv6 addresses  to the module.
 *           The most significant byte of the IP address is placed in
 *           array element 0.
 */
#define NWAL_IPV6_ADDR_SIZE      16
typedef unsigned char nwalIpv6Addr_t[NWAL_IPV6_ADDR_SIZE];
/** @} */


/**
 *  @ingroup nwal_api_structures
 *  @{
 * @brief  IP address specification
 *
 * @details  This union is used to specify an IP address to the module.
 *           The type in the union is determined through other parameters
 *           passed to the module (see @see IpValues).
 */
typedef union  {

  nwalIpv6Addr_t  ipv6;   /**< IPv6 address */
  nwalIpv4Addr_t  ipv4;   /**< IPv4 address */

} nwalIpAddr_t;
/** @} */


/**
 *  @ingroup nwal_api_structures
 *  @brief nwalAddIPParam  structure.
 *
 *  @details Configuration parameters for nwal_setIPAddr() API
 */
 /**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Valid Parameter configuration for nwal_setMacIface API
 *  @brief  Valid Parameter configuration
 *
 *  @details Valid Parameter to configure optional parameters.
 */
/* @{ */
/**
 *  @def  NWAL_SET_IP_VALID_PARAM_ROUTE_TYPE
 *        Route Type is valid
 *
 */
#define NWAL_SET_IP_VALID_PARAM_ROUTE_TYPE     0x1
/** @} */

/* @{ */
/**
 *  @def  NWAL_SET_IP_VALID_PARAM_REMOTE_IP
 *        Remote IP is valid 
 *
 */
#define NWAL_SET_IP_VALID_PARAM_REMOTE_IP     0x2
/** @} */
/** @} */
typedef struct  {
    uint16_t                validParams;    /**< Valid Parameter configuration.
                                              *  @see nwalSetMacValidParam
                                              */
    nwal_IpType             ipType;         /**< IPv4/IPv6 @see nwal_IpType */
    nwalIpAddr_t            locIpAddr;      /**< Local or Destination
                                              * IP address for incoming
                                              *  packets
                                              */
    nwalIpAddr_t            remIpAddr;      /**< Remote or Source
                                              * IP address for incoming
                                              *  packets
                                              */
    nwalIpOpt_t             ipOpt;          /**< IP Options @see nwalIpOpt_t */
    nwal_matchAction_t      matchAction;    /**< Action upon matching
                                              *  classification rule at
                                              *  NetCP @see nwal_matchAction_t
                                              */
    nwal_nextRtFailAction_t failAction;     /**< Configuration for action
                                              *  when next route classification
                                              *  fails
                                              *  @see nwalNextRouteFailAction
                                              */
    int16_t                 appRxPktFlowId; /** Optional: Application managed
                                              * Flow ID for any packet to
                                              * host from this classification
                                              * entry. In case if NWAL managed
                                              * flow needs to be used set to
                                              * NWAL_FLOW_NOT_SPECIFIED
                                              */
    NWAL_queueHnd           appRxPktQueue;  /** Optional: Application managed
                                              * Queue handle for any packet
                                              * to host from this
                                              * classification entry.
                                              * In case if NWAL managed Queue
                                              * needs to be used set to
                                              * NWAL_QUEUE_NOT_SPECIFIED
                                              */
    nwalRouteType_t         routeType;      /** Optional: Routing Type */
}nwalIpParam_t;

/**
 * @ingroup nwal_api_structures
 * @brief NWAL Device Specific Configuration Structure
 *
 * @details Required device configurations parameters
*/
/*  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   nssGen configration
 *  @brief  Specifies the version of NSS nwal is being configure for.
 *
 *  @details Specifies the version of the Network Subsytem
 *           nwal is being configure for.
 */
/* @{ */
/**
 *  @def  NWAL_CFG_NSS_GEN1
 *
 */

/**
 *  @ingroup nwal_api_structures
 *  @brief Buffer pool definition for RX and TX path.
 *
 *  @details Details for tMemory buffer pool being initialized by
 *         application.A memory buffer pool identifies the packet Lib Heap.
 *         For incoming packets, NWAL uses below configuration to create
 *         default CPPI flows for both incoming control and packet traffic.
 *         NetCP is expected to use  flow configuration to select
 *         appropriate heap while routing packet.
 *         For the TX direction NWAL will use the heap to allocate a buffer
 *         in case transmit header needs to be created for outgoing packet.
 *
 */
typedef struct  {
    uint8_t             descSize;   /**< Size of the descriptors */
    uint16_t            bufSize;    /**< Size of buffers */
    Pktlib_HeapHandle   heapHandle; /**< Heap containing free descriptors
                                      *  with buffers
                                      */
} nwalBufPool_t;

/**
 *  @ingroup nwal_api_structures
 *  @brief NWAL Multi Buffer pool configuration
 *
 *  @details Application owns descriptor allocation and buffers in queues.
 *           Definition covers an array of buffer pool configuration to NWAL
 */
#define NWAL_MAX_BUF_POOLS                  4   /** Maximum number of
                                                  * Buffer Pools
                                                  */
typedef struct  {
    uint8_t         numBufPools;                /**<Number of free
                                                 *  pools in a flow
                                                 *  Should be less
                                                 * than @see NWAL_MAX_BUF_POOLS
                                                 */
    nwalBufPool_t   bufPool[NWAL_MAX_BUF_POOLS];/**< Array of free Buffer
                                                 *   pools.
                                                 */
}nwalMbufPool_t;

#define NWAL_DEF_MAX_PKTS_PER_POLL      100     /**< Default packets processed
                                                  *  by NWAL per poll
                                                  */

    /**
     *  @ingroup nwal_api_structures
     *  @brief NWAL configuration parameters for PA and SASS base addresses
     *
     *  @details Application configures PA and SASS base addesses to NWAL
     */
typedef struct  {
    void*               paVirtBaseAddr;  /** Optional Virtual Base address of
                                           * PA Sub system if MMU is enabled
                                           * in master device where
                                           * @see nwal_create API is called
                                           * Set this to 0 in case of NWAL
                                           * using physical address from CSL
                                           * module
                                           */
    void*               pInstPoolPaBaseAddr;  /** Base address of the global shared memory pool 
                                             *  from which global  PA LLD instance & channel 
                                             *  instance memory is allocated
                                             */
    void*               pSaVirtBaseAddr;  /** Optional Virtual Base address
                                           * of SA/Crypto Sub system if
                                           * MMU is enabled in master device
                                           * where @see nwal_create API is
                                           * called.
                                           * Set this to 0 in case of NWAL
                                           * using physical address from CSL
                                           * module
                                           */
    void*               pInstPoolSaBaseAddr; /** Base address of the global shared memory 
                                              *  pool from which global SA LLD instance & 
                                              *  channel instance memory is allocated.*/
    void*               pScPoolBaseAddr;     /** Base address of the global shared memory pool
                                              *  from which SA security context memory is 
                                              *  allocated. This is a DMA
                                              */
}nwalBaseAddrCfg_t;



/**
 *  @ingroup nwal_api_structures
 *  @brief NWAL Global System Level configuration
 *
 *  @details Global configuration valid for entire device
 */
typedef struct  {
    uint32_t            validParams;    /**< Valid Parameters for
                                          *  Optional config
                                          */
    nwalMbufPool_t      pa2SaBufPool;    /**< Buffer pool for PA to SA
                                           *  packet exchange
                                           */
    nwalMbufPool_t      sa2PaBufPool;    /**< Buffer pool for SA to PA
                                           *  packet exchange. Created
                                           *  separate one to reduce
                                           *  contention of buffers between
                                           *  RX and TX
                                           */
    uint8_t             hopLimit;        /** Hop Limit for outgoing IP
                                           * Packets
                                           */
    nwal_Bool_t         paPowerOn;       /** Set to nwal_TRUE if NetCP
                                           * PKTPROC and CPGMAC power domain
                                           * is already out of reset.
                                           * By setting this NWAL will bypass
                                           * the step
                                           */
    nwal_Bool_t         paFwActive;      /** Set to nwal_TRUE if NetCP
                                           * PKTPROC and CPGMACpower domain
                                           * is already out of reset
                                           * By setting this NWAL will bypass
                                           * downloading formware to PA PDSPs
                                           * out of reset
                                           */
    nwal_Bool_t         saPowerOn;       /** Set to nwal_TRUE if NetCP Crypto
                                           * power domain is already out of
                                           * reset.
                                           * By setting this NWAL will bypass
                                           * taking out of reset for SA Crypto
                                           * Power domain
                                           */
    nwal_Bool_t         saFwActive;      /** Set to nwal_TRUE if NetCP SA
                                           * Firmware is already downloaded
                                           * externally
                                           * By setting this NWAL will bypass
                                           * SA Crypto firmware download
                                           *
                                           */
    NWAL_queueHnd       rxDefPktQ;       /** Optional parameter in case if
                                           *  application would need to
                                           *  redirect
                                           *  all default packets not matching
                                           *  classification rules to a queue.
                                           *  Set to NWAL_QUEUE_NOT_SPECIFIED
                                           *  if not used
                                           */

    Pa_Handle           paHandle;          /** Optional. Only required for the
                                            *  case of PA LLD resources being
                                            *  initialized outside NWAL and application 
                                            *  wants to start a previously created PA LLD 
                                            *  instance. If provided NWAL will skip
                                            *  creation of the PA LLD.
                                            *  Initialize to NULL for default
                                            *  PA resources being fully
                                            *  configured through NWAL
                                            */

    void*               saHandle;           /** Optional. Only required for the
                                             *  case of SA LLD resources being
                                             *  initialized outside NWAL and application 
                                             *  wants to start a previously created SA LLD 
                                             *  instance. If provided NWAL will skip
                                             *  creation of the SA LLD.
                                             *  Initialize to NULL for default
                                             *  PA resources being fully
                                             *  configured through NWAL
                                             */
    nwalBaseAddrCfg_t*     pBaseAddrCfg;      /* PA and SASS base adddres
                                             * configuration parameters
                                             */

} nwalGlobCfg_t;

/**
 *  @ingroup nwal_api_structures
 *  @brief NWAL Local per core configuration
 *
 *  @details Local configuration  per core
 */
typedef struct  {
   nwalMbufPool_t        rxPktPool;         /**< Memory buffer pool for
                                              *  incoming packets from NetCP
                                              *  to host
                                              */
   uint32_t              rxSopPktOffset;    /**< Start of packet offset
                                              *  for the packets allocated
                                              *  by hardware. The space will
                                              *  be configured to allow headroom
                                              *  space for the packets received
                                              *  using default flow configured
                                              *  at NWAL for the given process
                                              *  In case of non zero recommended
                                              *  value would be to have the
                                              *  offset at cacheline boundary
                                              *  for improved performance
                                              */

   uint32_t              rxPktTailRoomSz;    /**< Tailroom size for the
                                              *   incoming packet Ensure that
                                              *   buffer received from NetCP has
                                              *   enough tail room for
                                              *   application to update
                                              *   additions including any
                                              *   trailer
                                              */
   nwalMbufPool_t        txPktPool;         /**< Memory buffer pool for
                                              *  outgoing packets from Host
                                              *  to NetCP
                                              */
   nwalMbufPool_t        rxCtlPool;         /**< Memory buffer pool for
                                              *  control response from NetCP
                                              *  to Host
                                              */
   nwalMbufPool_t        txCtlPool;         /**< Memory buffer pool for
                                              *  control request from Host
                                              *  to NetCP
                                              */
   nwal_rxPktCallBack*   pRxPktCallBack;    /**< Global default callback
                                              *  function upon for processing
                                              *  packets at fast path
                                             */
   nwal_CmdCallBack*     pCmdCallBack;      /**< Global default callback
                                             *   function upon confirmation
                                             *   of  configuration response.
                                             *   @see nwal_CmdCallBack
                                             */
   nwal_CmdPaStatsReply* pPaStatsCallBack;  /** Global default callback
                                             *  function for response to
                                             *  Stats request from PA
                                             */
   nwal_rxDmCallback*    pRxDmCallBack;     /**< Global default callback
                                             *   function for receiving
                                             *   payload for data mode
                                             *   channels after crypto
                                             *   functionality is completed
                                             *   by NetCP
                                             */
}nwalLocCfg_t;


/** @name COMMON (Common Interface) APIs
 *
 */
/*@{*/


/**
 * @ingroup nwal_api_structures
 * @brief NWAL Memory Size Configuration Structure
 *
 * @details Required configurations to factor in size requirement for all
 *          buffers used within module.
 */
typedef struct  {
    uint16_t        nProc;              /**< Maximum number of processes
                                          *  invoking NWAL APIs. Each of the
                                          *  processes can be  a DSP-Core ID
                                          *  in the case of DSP only
                                          *  architecture
                                          */
    int             nMaxMacAddress;     /**< Maximum number of MAC Addresses
                                          *  to be configured at NETCP
                                          */
    int             nMaxIpAddress;      /**< Maximum number of IP Addresses
                                          *  to be configured at NETCP. In
                                          *  case if IPSec configuration is
                                          *  enabled the count should also
                                          *  include number of inner IPs
                                          */
    int             nMaxL4Ports;        /**< Maximum number of UDP/GTPU to
                                          *  be configured at NETCP
                                          */
    int             nMaxIpSecChannels;  /**< Maximum number of unidirectional
                                          *  IPSec Channels.
                                          *  RX and TX to be accounted
                                          * separately
                                          */
    int             nMaxDmSecChannels;  /**< Maximum number of unidirectional
                                          *  Data Mode Security Channels.
                                          * RX and TX to be accounted
                                          * separately
                                          */
    int             nMaxL2L3Hdr;        /**< Applicable for application
                                          *  utilizing NWAL infrastructure to
                                          *  prepare transmit header.
                                          *  Should indicate maximum number of
                                          *  unique IP local/remote endpoints
                                          *  pairs for the system. An example
                                          *  would be 2000 connections at
                                          *  local device terminating to 5
                                          *  different remote
                                          *  IP endpoints.nMaxL2L3Hdr would be
                                          *  5. If all 2000 connections are
                                          *  expected to terminate to
                                          *  different unique remote endpoints
                                          *  then this configuration
                                          *  should reflect 2000. NWAL memory
                                          *  sizing for TX L2L3 header will be
                                          *  sized accordingly
                                          *  For application not using NWAL
                                          *  transmit header creation
                                          *  infrastructure  this count can be
                                          *  initialized to zero.
                                          */
    Pa_Handle       pahandle;           /**< Optional.Only required  for the
                                         *   case of PA LLD resources being
                                         *   initialized outside NWAL. If
                                         *   provided NWAL will skip
                                         *   initialization of PA LLD
                                         *   Initialize to NULL for default
                                         *   PA resources being fully
                                         *   configured through NWAL
                                         */
} nwalSizeInfo_t;



/**
 * @brief  Defines the IPSec Protocol configuration for channel.
 *
 * @details Define for IPSec protocol configuration for channel
 *  @ingroup nwal_api_constants
 *  @{ */

typedef uint16_t nwal_IpSecProto;
/* @{ */
/**
 *  @def  nwal_IpSecProtoESPNATT
 *        IPSEC NATT Mode
 */
#define  nwal_IpSecProtoESPNATT     0
/*  @}  */
/* @{ */
/**
 *  @def  nwal_IpSecProtoAH
 *        IPSEC AH Mode
 */
#define  nwal_IpSecProtoAH     3
/*  @}  */
/* @{ */
/**
 *  @def  nwal_IpSecProtoESP
 *        IPSEC ESP Mode
 */
#define  nwal_IpSecProtoESP     4
/*  @} *//* ingroup */
/** @} */

/**
 * @ingroup nwal_api_structures
 * @brief IPSec SA Configuration Parameters for the channel
 *
 * @details Configuration uniquely identifying an IPSec channel.
 */
 
/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Valid Parameter configuration for nwalSaIpSecParam_t
 *  @brief  Valid Parameter configuration
 *
 *  @details Valid Parameter to configure optional parameters.
 */
/* @{ */
/**
 *  @def  NWAL_SA_INFO_VALID_PARAM_ESN
 *        Valid ESN configuration
 *
 */

#define NWAL_SA_INFO_VALID_PARAM_ESN        0x01
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_SA_INFO_VALID_PARAM_ROUTE_TYPE
 *        Valid Route Type configuration
 *
 */

#define NWAL_SA_INFO_VALID_PARAM_ROUTE_TYPE        0x02
/*  @}  */
/** @} */


/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   SA Mode for the IPSec Channel
 *  @brief  SA Mode for the IPSec Channel
 *
 *  @details SA Mode configuration
 */
/* @{ */
/**
 *  @def  nwal_SA_MODE_TRANSPORT
 *        Transport Mode
 *
 */
typedef uint16_t      nwal_saMode;
#define     nwal_SA_MODE_TRANSPORT      0
/*  @}  */
/* @{ */
/**
 *  @def  nwal_SA_MODE_TUNNEL
 *        Tunnel Mode
 *
 */
#define     nwal_SA_MODE_TUNNEL         1
/*  @}  */
/** @} */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Channel Direction Inbound/Outbound
 *  @brief  Channel Direction Inbound/Outbound
 *
 *  @details Channel configuration
 */
/* @{ */
/**
 *  @def  NWAL_SA_DIR_INBOUND
 *        Inbound Channel. NetCP would need to authentication/decryption
 *
 */
typedef uint16_t      nwal_SaDir;
#define NWAL_SA_DIR_INBOUND          1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_DIR_OUTBOUND
 *        Outbound.NetCP would need to perform encryption and add
 *        authentication tag
 */
#define NWAL_SA_DIR_OUTBOUND         2
/*  @}  */
/** @} */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   SA Authentication Algorithms
 *  @brief  Authentication Algorithms
 *
 *  @details Authentication Algorithm supported
 */
/* @{ */
/**
 *  @def  NWAL_SA_AALG_NULL
 *        No idviudal Authentication
 *
 */
typedef uint16_t      nwal_saAALG;
#define NWAL_SA_AALG_NULL               0
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_AALG_HMAC_MD5
 *        HMAC with SHA1 mode
 */
#define NWAL_SA_AALG_HMAC_MD5           1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_AALG_HMAC_SHA1
 *        HMAC with 224-bit SHA2 mode
 */
#define NWAL_SA_AALG_HMAC_SHA1          2
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_AALG_HMAC_SHA2_224
 *        HMAC with 224-bit SHA2 mode
 */
#define NWAL_SA_AALG_HMAC_SHA2_224      3
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_AALG_HMAC_SHA2_256
 *        HMAC with 256-bit SHA2 mode
 */
#define NWAL_SA_AALG_HMAC_SHA2_256      4
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_AALG_GMAC
 *        Galois Message Authentication Code mode
 */
#define NWAL_SA_AALG_GMAC               5
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_AALG_AES_XCBC
 *        AES Extended Cipher Block Chaining -
 *        Message Autnentication Code mode
 */
#define NWAL_SA_AALG_AES_XCBC           6
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_AALG_HMAC_SHA2_256_RFC4868
 *        HMAC with 256-bit SHA2 mode, ICV
 *        length truncated to 128 bits
 */
#define NWAL_SA_AALG_HMAC_SHA2_256_RFC4868      7
/*  @}  */
/** @} */


/**
 *  @ingroup nwal_api_constants
 *  @{
 *  @name   Encryption Algorithm Configuration
 *  @brief  Encryption Algorithm Configuration
 *
 *  @details Encryption Algorithm supported
 */
/* @{ */
/**
 *  @def  NWAL_SA_EALG_NULL
 *        No encryption
 *
 */
/* Encryption Algorithm Configuration */
typedef uint16_t      nwal_saEALG;
#define NWAL_SA_EALG_NULL           0
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_EALG_AES_CTR
 *        AES Counter mode
 */
#define NWAL_SA_EALG_AES_CTR        1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_EALG_AES_CBC
 *        AES CBC mode
 */
#define NWAL_SA_EALG_AES_CBC        3
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_EALG_DES_CBC
 *        DES CBC mode
 */
#define NWAL_SA_EALG_DES_CBC        4
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_EALG_3DES_CBC
 *        DES CBC mode
 */
#define NWAL_SA_EALG_3DES_CBC       5
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_EALG_AES_CCM
 *        Counter with CBC-MAC mode
 */
#define NWAL_SA_EALG_AES_CCM        6
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_EALG_AES_GCM
 *        Galois Counter mode
 */
#define NWAL_SA_EALG_AES_GCM        7
/*  @}  */
/** @} */

typedef struct  {
    uint32_t            spi;            /**< IPSec Security Parameter index */
    nwalIpAddr_t        src;            /**< Source IP Address */
    nwalIpAddr_t        dst;            /**< Destination Address */
    nwal_IpSecProto     proto;          /**< IpSec Proto */
} nwalSaIpSecId_t;

typedef struct  {
    uint32_t                validParams;    /**< Valid Parameters for
                                              *  Optional config
                                              */
    nwal_saMode             saMode;         /**< Tunnel/ Transport mode */
    uint32_t                replayWindow;   /**< Replay Window Size */
    nwal_SaDir              dir;            /**  Direction for the channel.
                                              *  Inbound or Outbound
                                              */
    uint32_t                esnLo;          /**< Initial Value of Extended
                                              *  Sequence Number LSB
                                              */
    uint32_t                esnHi;          /**< Initial Value of Extended
                                              *  Sequence Number MSB
                                              */
    nwal_saAALG             authMode;       /**< Authentication Algorithm */
    nwal_saEALG             cipherMode;     /**< Encryption Algorithm */
    nwalMacAddr_t           remMacAddr;     /**< Remote MAC address */
    uint16_t                macSize;        /**< Specify the size of the
                                              *  authentication tag in bytes
                                              */
    nwal_matchAction_t      matchAction;    /**< Action upon matching
                                              *  classification rule at NetCP
                                              * @see nwal_matchAction_t
                                              *  Note setting the action to
                                              *  NWAL_MATCH_ACTION_HOST will
                                              *  require application
                                              *  to perform necessary
                                              *  authentication/decryption on
                                              *  IPSec Header or use
                                              *  side band mode
                                              *  Applicable only for
                                              *  @see NWAL_SA_DIR_INBOUND
                                              */
    nwal_nextRtFailAction_t failAction;     /**< Configuration for action when
                                              *  next route classification fails
                                              *  @see nwalNextRouteFailAction
                                              *  Applicable only for
                                              *  @see NWAL_SA_DIR_INBOUND
                                              */
    int16_t                 appRxPktFlowId; /** Optional: Application managed
                                              * Flow ID for any packet to host
                                              * from this classification
                                              * entry. In case if NWAL managed
                                              * flow needs to be used set to
                                              * NWAL_FLOW_NOT_SPECIFIED
                                              */
    NWAL_queueHnd           appRxPktQueue;  /** Optional: Application managed
                                              * Queue handle for any packet to
                                              * host from this classification
                                              * entry. In case if NWAL managed
                                              * Queue needs to be used set to
                                              * NWAL_QUEUE_NOT_SPECIFIED
                                              */
    nwalRouteType_t         routeType;      /** Optional: Routing Type */
} nwalSaIpSecParam_t;
/** @} */

/** @} */
/**
 *  @ingroup nwal_api_structures
 *  @name   Key Size
 *  @brief nwalSecKeyParams_t  structure.
 *
 *  @details Key configuration input for the channel.
 */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Maximum Key Size configuration
 *  @brief  Maximum Key Size configuration
 *
 *  @details Maximum size for Encryption and Authentication Keys
 */
/* @{ */
/**
 *  @def  NWAL_SA_MAX_AUTH_KEY_LEN
 *        Maximum size for Authentication Key
 */
#define NWAL_SA_MAX_AUTH_KEY_LEN    64
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_SA_MAX_ENC_KEY_LEN
 *        Maximum size for Encryption Kepys
 *
 */
#define NWAL_SA_MAX_ENC_KEY_LEN     32
/*  @}  */

typedef struct  {

    uint16_t            encKeySize;  /**< in bytes */
    uint16_t            macKeySize;  /**< in bytes */
    uint8_t*            pEncKey;     /**< Encryption Key */
    uint8_t*            pAuthKey;    /**< Authentication Key */
} nwalSecKeyParams_t;
/** @} */


/**
 * @ingroup nwal_api_structures
 * @brief nwalCreateSA configuration parameters
 *
 * @details nwalCreateSA configuration input.
 */
typedef struct  {
    union {
        nwal_Handle         macHandle;     /**<Will be used to retrieve local
                                             * MAC. For Outbound SA the handle
                                             * will reflect source MAC address
                                             * For Inbound SA handle will
                                             * reflect destination MAC
                                             */
        nwal_Handle         handle;
    }h;
    nwal_IpType             ipType;         /**< IPv4/IPv6 */
    nwalSaIpSecParam_t      saIpSecParam;    /**< IPSec Configuration parameters
                                               */
    nwalSecKeyParams_t      keyParam;       /**< Key configuration */
} nwalCreateSAParams_t;

/**
 * @ingroup nwal_api_structures
 * @brief Data Mode SA Configuration Parameters for the channel
 *
 * @details Configuration uniquely identifying an Data Mode SA Channel.
 */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Data Mode Channel Type Encryption/Decryption
 *  @brief  Data Mode Channel Type Encryption/Decryption
 *
 *  @details Channel configuration
 */
/* @{ */
/**
 *  @def  NWAL_DM_CHAN_DECRYPT
 *        Data Mode Channel to be used for Decryption
 */
typedef uint16_t      nwal_DmChnType;
#define NWAL_DM_CHAN_DECRYPT        0
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_DM_CHAN_ENCRYPT
 *        Data Mode Channel to be used for Encryption
 */
#define NWAL_DM_CHAN_ENCRYPT        1
/*  @}  */
/** @} */



typedef struct  {
    nwal_DmChnType          dmChnType;      /**< Data Mode Channel Type:
                                              *  @see NWAL_DM_CHAN_DECRYPT
                                              *  or @see NWAL_DM_CHAN_ENCRYPT
                                              */
    uint32_t                replayWindow;   /**< Replay Window Size */
    nwal_saAALG             authMode;       /**< Authentication Algorithm */
    nwal_saEALG             cipherMode;     /**< Encryption Algorithm */
    uint16_t                macSize;        /**< Specify the size of the
                                              *  authentication tag in bytes
                                              */
    uint16_t                aadSize;        /**< Specify the size of the
                                              *  additional authenticated data
                                              *  in bytes used in CCM and GCM
                                              *  modes
                                              */
    nwal_Bool_t             enc1st;         /**< @see nwal_TRUE : Perform
                                              *  encryption first;
                                              *  @see nwal_FALSE : Perform
                                              *  authentication first
                                              */
} nwalDmSAParams_t;


/**
 * @ingroup nwal_api_structures
 * @brief nwalCreateSA configuration parameters
 *
 * @details nwalCreateSA configuration input.
 */
typedef struct  {
    nwalDmSAParams_t        dmSaParam;      /**< Data Mode Configuration
                                              *  parameters
                                              */
    nwalSecKeyParams_t      keyParam;       /**< Key configuration */
} nwalCreateDmSAParams_t;



/**
 * @ingroup nwal_api_structures
 * @brief nwalAddSP configuration parameters
 *
 * @details Add Security Profile for SA Channel.
 */
/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Valid Parameter configuration for nwal_setSecPolicy API
 *  @brief  Valid Parameter configuration
 *
 *  @details Valid Parameter to configure optional parameters.
 */

/* @{ */
/**
 *  @def  NWAL_SET_SEC_POLICY_VALID_PARAM_ROUTE_TYPE
 *        Route Type is valid
 *
 */
#define NWAL_SET_SEC_POLICY_VALID_PARAM_ROUTE_TYPE     0x1
/** @} */

/** @} */



typedef struct  {
    nwal_Handle             handle;         /**< nwalSaHandle in the case of
                                              *  IPSEC SA or macHandle in the
                                              *  case of non IPSEC
                                              */
    uint32_t                validParams;    /**< Valid Parameters for
                                              *  Optional config
                                              */
    nwal_SaDir              dir;            /** Direction for the channel.
                                              * Inbound or Outbound
                                              * @see nwal_SaDir
                                              */
    nwal_IpType             ipType;        /**< IPv4/IPv6 */
    nwalIpAddr_t            dst;            /** Destination Address */
    nwalIpAddr_t            src;            /** Source Address */
    nwalIpOpt_t             ipOpt;          /** IP Options @see nwalIpOpt_t */
    nwal_matchAction_t      matchAction;    /**< Action upon matching
                                              *  classification rule at NetCP
                                              *  @see nwal_matchAction_t
                                              *  Applicable only for
                                              *  @see NWAL_SA_DIR_INBOUND
                                              */
    nwal_nextRtFailAction_t failAction;     /**< Configuration for action when
                                              *  next route classification fails
                                              *  @see nwalNextRouteFailAction
                                              *  Applicable only for
                                              *  @see NWAL_SA_DIR_INBOUND
                                              */
    int16_t                 appRxPktFlowId; /** Optional: Application managed
                                              * Flow ID for any packet to host
                                              * from this classification
                                              * entry. In case if NWAL managed
                                              * flow needs to be used set to
                                              * NWAL_FLOW_NOT_SPECIFIED
                                              */
    NWAL_queueHnd           appRxPktQueue;  /** Optional: Application managed
                                              * Queue handle for any packet to
                                              * host from this classification
                                              * entry. In case if NWAL managed
                                              * Queue needs to be used set to
                                              * NWAL_QUEUE_NOT_SPECIFIED
                                              */
    nwalRouteType_t         routeType;  /** Optional: Routing Type */
} nwalSecPolParams_t;
/** @} */


/**
 *
 * @ingroup nwal_api_structures
 *  @{
 * @brief Application Layer protocol type for payload
 *
 * @details Defines Application layer protocol for the channel.
 */
typedef uint16_t      nwal_appProtoType_t;
/* @{ */
/**
 *  @def  NWAL_APP_PLOAD_PROTO_UDP
 *        Protocol Type for Application payload is UDP
 *
 */
#define NWAL_APP_PLOAD_PROTO_UDP    0x01
/** @} */
/* @{ */
/**
 *  @def  NWAL_APP_PLOAD_PROTO_GTPU
 *        Protocol Type for Application payload is GTPU
 *
 */
#define NWAL_APP_PLOAD_PROTO_GTPU   0x02
/** @} */
/* @{ */
/**
 *  @def  NWAL_APP_PLOAD_16_BIT_PORT
 *        All other 16 bit ports
 *
 */
#define NWAL_APP_PLOAD_16_BIT_PORT   0x03
/** @} */
typedef union  {
    uint16_t                udpPort;/** UDP Port */
    uint32_t                gtpTeid;/** GTPU Tunnel ID */
    uint16_t                port;   /** All other 16 bit ports eg: TCP etc */
} nwalAppProto_t;
/** @} */

/**
 *
 * @ingroup nwal_api_structures
 *  @{
 * @brief Configuration details for packets being received for the connection
 *
 * @details Configuration details for packets being received for the connection.
 */
/*  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Valid Parameter configuration for nwal_addConn API
 *  @brief  Valid Parameter configuration
 *
 *  @details Valid Parameter to configure optional parameters.
 */

/* @{ */
/**
 *  @def  NWAL_SET_SEC_POLICY_VALID_PARAM_ROUTE_TYPE
 *        Route Type is valid
 *
 */
#define NWAL_SET_CONN_VALID_PARAM_ROUTE_TYPE     0x1
/** @} */

/** @} */

typedef struct  {
    nwal_Handle             inHandle;   /**< Inbound Handle should be either:
                                          *     - For IPSecNWAL handle returned
                                          *       for nwal_setSecPolicy()
                                          *       NWAL_SA_DIR_INBOUND
                                          *     or
                                          *     - NWAL handle returned from
                                          *       nwal_setIPAddr ()
                                          */
    uint32_t                validParams;    /**< Valid Parameters for
                                              *  Optional config
                                              */
    nwalAppProto_t          appProto;       /**< Application protocol type
                                              *  @see nwalAppProto_t */
    uint8_t                 rxCoreId;       /**< DSP-Core ID (zero based) where
                                              *  packet needs to be terminated
                                              */
    nwal_matchAction_t      matchAction;    /**< Action upon matching
                                              *  classification rule at NetCP
                                              *  @see nwal_matchAction_t
                                              *  Valid values
                                              *  @see NWAL_MATCH_ACTION_DISCARD
                                              *  or @see NWAL_MATCH_ACTION_HOST
                                              */
    int16_t                 appRxPktFlowId; /** Optional: Application managed
                                              * Flow ID for any packet to host
                                              * from this classification
                                              * entry. In case if NWAL managed
                                              * flow needs to be used set to
                                              * NWAL_FLOW_NOT_SPECIFIED
                                              */
    NWAL_queueHnd           appRxPktQueue;  /** Optional: Application managed
                                              * Queue handle for any packet to
                                              * host from this classification
                                              * entry. In case if NWAL managed
                                              * Queue needs to be used set to
                                              * NWAL_QUEUE_NOT_SPECIFIED
                                              */
    nwalRouteType_t         routeType;  /** Optional: Routing Type */
} nwalRxConnCfg_t;
/** @} */

/**
 *
 *  @ingroup nwal_api_structures
 *  @{
 * @brief Configuration details for packets being transmitted for the connection
 *        Will be used in the case of NWAL module being used to create TX
 *        headers
 *
 * @details Configuration details for packets being transmitted for the
 *        connection will be used in the case of NWAL module being used to
 *        create TX headers
 */

typedef struct  {
    nwal_Handle             outHandle;   /**< Outbound Handle should be either:
                                          *     - NWAL handle returned for
                                          *       nwal_setSecPolicy()
                                          *       NWAL_SA_DIR_OUTBOUND
                                          *     or
                                          *     - NULL in the case of non IPSec
                                          *       configuration
                                          */
    nwalMacAddr_t           remMacAddr;  /**< Remote Mac Address
                                           *  In case of IPSec configuration
                                           *  Remote MAC address configuration
                                           *  from SA channel configuration will
                                           *  override this value
                                           */
    nwalMacOpt_t            macOpt;      /**< MAC header configuration for
                                           *  outgoing packet
                                           */
    uint32_t                mtuSize;       /** Maximum MTU size limited at MAC
                                           * interface. Enables IP fragmentation
                                           * functionality at NetCP. In the case
                                           * of tunnel IP outer IP only will be
                                           * fragmented. Configuring NULL will
                                           * disable the fragmentation at NetCP
                                           */
    nwal_IpType             ipType;      /**< IPv4/IPv6 :nwal_IpType */
    nwalIpAddr_t            remIpAddr;   /**< Remote IP Address */
    nwalIpOpt_t             ipOpt;       /**< IP header configuration for
                                           *  outgoing packet
                                           */
    nwalAppProto_t          appProto;
} nwalTxConnCfg_t;
/** @} */

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Bit Map indicating the queues to be polled by NWAL
 *  @brief    Bit Map indicating the queues to be polled by NWAL
 *
 *  @details   Bit Map indicating the queues to be polled by NWAL.
 */
/* @{ */
/**
 *  @def  nwal_POLL_DEFAULT_GLOB_PKT_Q
 *        Poll for common Global L2/L3 packets which failed next route
 *        classification at NetCP
 */
typedef uint16_t      nwal_pollPktQCtl;
#define     nwal_POLL_DEFAULT_GLOB_PKT_Q    0x0001  /**< Poll for all packets
                                                      *  received per system.
                                                      *  This includes
                                                      *  packets received
                                                      *  through next route
                                                      *  fail for MAC/[IPSec]/IP
                                                      *  classification or
                                                      *  packets matching
                                                      *  MAC/[IPSec]/IP
                                                      *  classification
                                                      *  and being terminated
                                                      *  to host at
                                                      *  @see rxDefPktQ.
                                                      */
/*  @}  */
/* @{ */
/**
 *  @def  nwal_POLL_DEFAULT_PER_PROC_PKT_Q
 *        Poll for packets to be terminated to Fast Path core
 *
 */
#define     nwal_POLL_DEFAULT_PER_PROC_PKT_Q  0x0002  /**< Poll for all packets
                                                       *  matching L4 or L5
                                                       *  classification
                                                       *  from NetCP through
                                                       *  NWAL managed flow/
                                                       *  queue per process
                                                       *  terminated through
                                                       *  @see rxL4PktQ
                                                       */
/*  @}  */
/* @{ */
/**
 *  @def  nwal_POLL_APP_MANAGED_PKT_Q
 *        Poll for packets to be terminated at Application managed packet
 *        Queue/Flow
 */
#define     nwal_POLL_APP_MANAGED_PKT_Q     0x0004  /**< Poll for all packets
                                                      *  from NetCP received at
                                                      *  Application managed
                                                      *  flow/queue
                                                      */
/*  @}  */
/** @} */


/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name  Control for queue to be polled by NWAL for Data Mode SA channel
 *  @brief Control for queue to be polled by NWAL for Data Mode SA channel
 *
 *  @details Control for queue to be polled by NWAL. Only one queue can be
 *           polled per API Call
 */
/* @{ */
/**
 *  @def  nwal_POLL_DM_DEF_SB_SA_Q
 *        Poll default Side Band SA Response Q
 *        from NetCP
 */
typedef uint16_t      nwal_pollDmQCtl;
#define     nwal_POLL_DM_DEF_SB_SA_Q            1
/*  @}  */
/* @{ */
/**
 *  @def  nwal_POLL_DM_APP_MANAGED_Q
 *        Poll for Data Mode Application managed packet Queue
 *
 */
#define     nwal_POLL_DM_APP_MANAGED_Q          3
/*  @}  */
/** @} */

/**
 * @ingroup nwal_api_constants
 * @brief  Handle owned by NWAL abstracted to Application.
 * @details Application to use this handle to identify NWAL configuration
 */
 /**
 *  @def  nwal_MAX_NETCP_PASS_ACTIVE_PDSP       (pa_CMD_TX_DEST_5+1)
 *        Maximum number of active NETCP PASS PDSPs
 */
#define   nwal_MAX_NETCP_PASS_ACTIVE_PDSP     NSS_PA_NUM_PDSPS


/**
 *
 * @ingroup nwal_api_structures
 * @brief Global context information at NWAL
 *
 * @details Global context information at NWAL
 */
typedef struct  {
    int16_t                 rxPaSaFlowId;  /**< Flow Handle for packets from PA
                                             *  to SA
                                             */
    int16_t                 rxSaPaFlowId;  /**< Flow Handle for packets from SA
                                             *  to PA
                                             */
    NWAL_queueHnd           rxDefPktQ;   /**< Per system level default packet
                                           *  Queue for termination of all
                                           *  packets except L4
                                           *  This includes packets received
                                           *  through next route fail for
                                           *  MAC/[IPSec]/IP
                                           *  classification or packets matching
                                           *  MAC/[IPSec]/IP classification
                                           *  and being terminated at host.
                                           */
    NWAL_queueHnd           defFlowQ;    /**< Default Queue for all exception
                                           *  packets for default flows created
                                           *  by NWAL
                                           */
    NWAL_hwChanHandle       passCppiHandle;/** CPPI handle for PASS module
                                            *  created by NWAL
                                            */
    int16_t                 numPaPDSPs;  /** Number of NetCP PDSPs being
                                           * used by NWAL
                                           */
    uint32_t                pdspVer[NSS_PA_NUM_PDSPS];
    int                     extErr;      /**< Extended Error details for
                                           *  nwalCreate API
                                           */
} nwalGlobCxtInfo_t;

/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Valid bitmap for internal handles
 *  @brief  Global configuration for unclassified packets from NetCP
 *
 *  @details Global configuration for unclassified packets from NetCP
 *           To be configured after @ref nwal_start
 */
/* @{ */
/**
 *  @def  NWAL_CTRL_CXT_VALID_PA_MAC_HANDLE
 *        Valid PA MAC Handle. Applicable when nwal_Handle being
 *        passed is the one returned from  either
 *        @see nwal_setMacIface / @see nwal_setIPAddr / 
 *        Outer IP Handle in the case of @see nwal_setSecAssoc
 */
typedef uint32_t      nwal_ChanCxtValidBitMap_t;
#define NWAL_CTRL_CXT_VALID_PA_MAC_HANDLE           0x00000001
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CXT_VALID_PA_OUTER_IP_HANDLE
 *        Valid PA IP Handle. Applicable when nwal_Handle being
 *        passed is the one returned from  Outer IP Handle retruned
 *        through @see nwal_setSecAssoc
 */
#define NWAL_CTRL_CXT_VALID_PA_OUTER_IP_HANDLE      0x00000002
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CXT_VALID_PA_INNER_IP_HANDLE
 *        Valid PA IP Handle. Applicable when nwal_Handle being
 *        passed is the one returned from   @see nwal_setIPAddr or  
 *        @see nwal_setSecPolicy 
 */
#define NWAL_CTRL_CXT_VALID_PA_INNER_IP_HANDLE      0x00000004
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CXT_VALID_PA_L4_HANDLE
 *        Valid PA UDP Handle. Applicable when nwal_Handle being
 *        passed is the one returned from   @see nwal_addConn 
 */
#define NWAL_CTRL_CXT_VALID_PA_L4_HANDLE            0x00000008
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CXT_VALID_IPSEC_SA_HANDLE
 *        Valid SA Handle. Applicable when nwal_Handle being
 *        passed is the one returned from   @see nwal_setSecAssoc
 */
#define NWAL_CTRL_CXT_VALID_IPSEC_SA_HANDLE               0x00000010
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CXT_VALID_DM_SA_HANDLE
 *        Valid SA Handle. Applicable when nwal_Handle being
 *        passed is the one returned from   @see nwal_setDMSecAssoc 
 */
#define NWAL_CTRL_CXT_VALID_DM_SA_HANDLE               0x00000020
/*  @}  */
/* @{ */
/**
 *
 * @ingroup nwal_api_structures
 * @brief Channel context information stored within NWAL channel handle
 *
 * @details Channel context information at NWAL
 */
typedef struct  {
    nwal_ChanCxtValidBitMap_t   validBitMap;/**< Valid Bit map */
    paHandleL2L3_t          paMacHandle;   /**< PA MAC Handle if
                                             * NWAL_CTRL_CXT_VALID_PA_MAC_HANDLE
                                             * is set
                                            */
    paHandleL2L3_t          paOuterIpHandle; /**< PA Outer IP Handle for IPSec 
                                             * tunnel config if
                                             * NWAL_CTRL_CXT_VALID_PA_OUTER_IP_HANDLE
                                             * is set
                                             */
    paHandleL2L3_t          paInnerIpHandle; /**< PA Inner IP Handle  if
                                             * NWAL_CTRL_CXT_VALID_PA_INNER_IP_HANDLE
                                             * is set
                                             */
    paHandleL4_t            paL4Handle;     /**< Layer 4 Handle if
                                             * NWAL_CTRL_CXT_VALID_PA_L4_HANDLE
                                             * is set
                                             */
#ifdef NWAL_ENABLE_SA   
    Sa_ChanHandle           saChanHandle;   /**< SA IP Sec handle or Data Mode
                                             *   handle if
                                             * NWAL_CTRL_CXT_VALID_DM_SA_HANDLE
                                             * is set
                                             */
#endif
}nwalChanCxtInfo_t;
/**
 *
 * @ingroup nwal_api_structures
 * @brief Local (per process) context information in NWAL
 *
 * @details Local (per process) context information in NWAL
 */

typedef struct  {
    uint16_t                numPendPAReq;/**< Number of pending PA requests for
                                           *  process
                                           */
    NWAL_queueHnd           rxCtlQ;      /**< Response Queue for receiving
                                           *  control response
                                           */
    NWAL_queueHnd           rxL4PktQ;    /**< Default Queue for receiving L4
                                           *  Packets
                                           */
    NWAL_queueHnd           rxSbSaQ;     /**< Packet Queue to receive response
                                          * for all data mode SA related
                                          * responses
                                          */
    uint16_t                extErr;      /**< Extended Error details from NetCP
                                           */
    int16_t                 rxPktFlowId; /**< Default Flow Handle for packets
                                           *  from NetCP to host
                                           */
    int16_t                 rxCtlFlowId; /**< Default Flow Handle for control
                                           *  response from NetCP to host
                                           */
Cppi_Handle           cppiHandle;           /* PASS CDMA handle for this process */
} nwalLocCxtInfo_t;


/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Global configuration for unclassified packets from NetCP
 *  @brief  Global configuration for unclassified packets from NetCP
 *
 *  @details Global configuration for unclassified packets from NetCP
 *           To be configured after @ref nwal_start
 */
/* @{ */
/**
 *  @def  NWAL_CTRL_CFG_ALL_EXCEPTIONS
 *        Configure NetCP for all Exception Packets
 *        defined under pa_EROUTE_XXX. Refer PA API file under
 *        ti/drv/pa/pa.h .
 */
typedef uint16_t      nwal_ctlCfg_t;
#define NWAL_CTRL_CFG_ALL_EXCEPTIONS        0
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CFG_SINGLE_EXCEPTION
 *        Configure NetCP for single Exception packets
 *        defined under pa_EROUTE_XXX. Refer PA API file under
 *        ti/drv/pa/pa.h .
 */
#define NWAL_CTRL_CFG_SINGLE_EXCEPTION              0x1
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CFG_PA_ASSISTED_REASSEM
 *        NetCP Assisted Reasssembly. The feature will
 *        allow incoming fragmented packets to be reassembled at application and
 *        being further redirected back to PA for further classification. The 
 *        feature will also maintain ordering of incoming packets from network.
 *        In order to enable PA Assisted Reassembly matchAction need to be 
 *        configured to NWAL_MATCH_ACTION_HOST and application need to 
 *        register callback pRxReassemProc.
 *        For disabling PA Assisted Reassembly feature matchAction need to be 
 *        configured to NWAL_MATCH_ACTION_DISCARD
 */
#define NWAL_CTRL_CFG_PA_ASSISTED_REASSEM           0x2
/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CFG_EMAC_IF_EGRESS_EQOS_MODE
 *        Configure NETCP for EMAC interface-based enhanced QoS Mode for egress ethernet traffic
 *
 */
#define NWAL_CTRL_CFG_EMAC_IF_EGRESS_EQOS_MODE      0x4

/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CFG_EMAC_IF_DEFAULT_ROUTE
 *        Configure NETCP for EMAC interface-based Default Route for ingresss ethernet traffic
 *
 */
#define NWAL_CTRL_CFG_EMAC_IF_INGRESS_DEFAULT_ROUTE         0x8

/*  @}  */
/* @{ */
/**
 *  @def  NWAL_CTRL_CFG_NATT
 *        Configure NETCP to enable NATT
 *
 */
#define NWAL_CTRL_CFG_NATT         0x10

/** @} */
/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Valid Parameter configuration for nwal_control API
 *  @brief  Valid Parameter configuration
 *
 *  @details Valid Parameter to configure optional parameters.
 */

/* @{ */
/**
 *  @def  NWAL_CONTROL_VALID_PARAM_ROUTE_TYPE
 *        Route Type is valid
 *
 */
#define NWAL_CONTROL_VALID_PARAM_ROUTE_TYPE     0x1
/** @} */

/** @} */

typedef struct  {
    uint32_t                validParams;    /**< Valid Parameters for
                                              *  Optional config
                                              */
    nwal_ctlCfg_t           pktCtl;         /**< Bitmap for packet control cfg
                                              */
    uint16_t                pa_EROUTE_Id;   /**< Applicable only for
                                              *  NWAL_CTRL_CFG_SINGLE_EXCEPTION
                                              *  Select single exception ID
                                              *  pa_EROUTE_XXX
                                              */
    nwal_AppId              appId;          /**< Application ID to be received
                                              *  for incoming exception packets
                                              *  Note if all exception packets
                                              *  are configured to terminate to
                                              *  to host nwalRxPktInfo_t->rxFlag1
                                              *  will indicate
                                              *  NWAL_RX_IP_FRAGMENT_PKT for
                                              *  fragment packets
                                              *  Not applicable for 
                                              *  NWAL_CTRL_CFG_PA_ASSISTED_REASSEM
                                              */
                                              
    nwal_rxReassemProc*     pRxReassemProc; /**< Rx Reassembly routine to be 
                                              *  called once NWAL identifies 
                                              *  incoming fragmented packet.  
                                              *  Mandatory for the case of 
                                              *  NWAL_CTRL_CFG_PA_ASSISTED_REASSEM 
                                              *  Previous callback if provided will
                                              *  be updated.
                                              */
    nwal_matchAction_t      matchAction;    /**< Action upon matching
                                              *  classification rule at
                                              *  NetCP @see nwal_matchAction_t
                                              *  NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE not
                                              *  valid configuration in this case
                                              */
    int16_t                 appRxPktFlowId; /** Optional: Application managed
                                              * Flow ID for any packet to host
                                              * from this classification
                                              * entry. In case if NWAL managed
                                              * flow needs to be used set to
                                              * NWAL_FLOW_NOT_SPECIFIED
                                              */
    NWAL_queueHnd           appRxPktQueue;  /** Optional: Application managed
                                              * Queue handle for packets being
                                              * routed to host. For default set
                                              * Queue to NWAL_QUEUE_NOT_SPECIFIED
                                              * Packets can be polled using
                                              * @ref nwal_pollPkt with
                                              * nwal_POLL_DEFAULT_GLOB_PKT_Q
                                              */
    nwalRouteType_t     routeType;  /** Optional: Routing Type */
    uint8_t             egressDefPri;       /**< Optional: Specify the global default priority for untagged non IP egress traffic 
                                              *  for enhanced QoS mode (refer to ti/drv/pa/pa.h, appendix7) */
    uint16_t            enableNatt;         /** Optional:  Enable or disable NATT
                                               */
    uint16_t            nattPort;           /** Optional:  Port to be used for NAT-T
                                               */
} nwalCtlInfo_t;

/**
 *  @brief   Define the maximum number of buffers the module can request
 *
 */
#define nwal_N_BUFS                             pa_N_BUFS /**< Number of Buffers to be
                                                    *  used by NWAL
                                                    */

#define nwal_BUF_INDEX_INST                     0   /** Buffer containing
                                                      * NWAL Global Instance
                                                      */
#define nwal_BUF_INDEX_INT_HANDLES              1   /** Buffer for storing
                                                      * MAC/IPSec/IP/Port/
                                                      * Transmit Header Handles
                                                      */
#define nwal_BUF_INDEX_PA_LLD_BUF0              2   /** Buffer used by PA LLD */
#define nwal_BUF_INDEX_PA_LLD_BUF1              3   /** Buffer used by PA LLD */
#define nwal_BUF_INDEX_PA_LLD_BUF2              4   /** Buffer used by PA LLD */
#define nwal_BUF_INDEX_SA_LLD_HANDLE            5   /** Buffer used by SA LLD */
#define nwal_BUF_INDEX_SA_CONTEXT               6   /** Buffer used by SA LLD */
#define nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE       7   /** Buffer for managing
                                                      * SA LLD
                                                      */


#define NWAL_TRANSID_SPIN_WAIT                  0xFFFF  /**< Reserved
                                                          *  Transaction ID for
                                                          *  Blocking the API
                                                          * until transaction is
                                                          * complete.
                                                          * NWAL module will do
                                                          * spin wait until
                                                          * transaction is
                                                          * complete
                                                          */
/**
 *  @ingroup nwal_api_functions
 *  @brief      API to retrieve memory buffer requirement by NWAL module
 *
 *  @details    Input to the API is configuration for NWAL through
 *              @see nwalSizeInfo_t.
 *              API returns buffer size  and alignment size requirement for the
 *              module
 *
 *  nwalSizeInfo_t Configuration information for Memory sizing.
 *  sizes        Buffer sizes
 *  aligns       Alignment size
 *  @retval      @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre         None
 */
nwal_RetValue nwal_getBufferReq(nwalSizeInfo_t *sizeCfg,
                                int sizes[nwal_N_BUFS],
                                int aligns[nwal_N_BUFS]);


/**
 *  @ingroup nwal_api_functions
 *  @brief      API to retrieve memory buffer requirement for  NWAL local 
                context instances in the process.
 *
 *  @details    Input to the API is number of threads running in the process.
 *              API returns buffer size  requirement for the local context instances.
 *
 *  @param[in]  nThreads number of threads running in the process.

*  @param[out]  pSize buffer size for local context instances.
 *  @retval      @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre         None
 */
    nwal_RetValue nwal_getLocContextBufferReq(int nThreads,
                                              uint32_t* pSize);

/**
 *  @brief      API to retrieve memory buffer requirement for NWAL local 
 *              contexts. One local context per thread.
 *
 *  @details    
 *              @see nwalSizeInfo_t.
 *              API returns buffer size requirements of NWAL local context
 *              structures.
 *
 *  nThreads    Number of threads per process.
 *  sizes        Buffer sizes
 *  @retval      @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre         None
 */
nwal_RetValue nwal_getLocContextBufferReq(int nThreads, uint32_t* pSize);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API instantiates the driver and allocated global resources and
 *          is pre-requisite
 *
 *  @details Allocates global resources valid  per system level common across
 *           all DSP cores
 *
 *           All NetCP  related initialization will be done inside the module.
 *           Module also supports a possible use case of NetCP initialization
 *           being done by application. In that case application would need to
 *           provide paHandle in the API. All QM and PktDMA initialization is
 *           required to be done outside module.
 *          Following are the resource initialization  prerequisite to NWAL
 *          module initialization:
 *           - QMSS Initialization for the link RAM and descriptors
 *           - CPPI Initialization of descriptors
 *           - Buffer pool being made available through queues containing
 *          descriptors and linked buffers
 *  @param[in]  pCfg    Input conguration
 *  @param[in]  pSizeInfo Configuration information for Memory sizing.
 *  @param[in]  sizes Memory Size for the buffers allocated. Expected to be
 *              same as output returned by NWAL during  nwal_getBufferReq() API
 *  @param[in]  bases  Array of the memory buffer base addresses
 *  @param[out] pNwalInst NWAL Global Instance handle.
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        @see nwal_getBufferReq
 */
nwal_RetValue nwal_create ( const nwalGlobCfg_t*    pCfg,
                            nwalSizeInfo_t*         pSizeInfo,
                            int                     sizes[nwal_N_BUFS],
                            void*                   bases[nwal_N_BUFS],
                            nwal_Inst*              pNwalInst);
/**
 *  @ingroup nwal_api_functions
 *  @brief  API to configure process specific information.
 *
 *  @details Needs to be called before nwal_create.
 *
 *  @param[in]  pSharedBase shared memory base address for nwal global context.
 *  @param[in]  pLocalCtxInstPool virtual address for start of local context memory.
 *  @param[in]  pBaseAddrCfg NWAL configuration parameters for PA and SASS base addresses
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        
 */
nwal_RetValue nwal_createProc(void*                 pSharedBase,
                              void*                 pLocalCtxInstPool,
                              nwalBaseAddrCfg_t*    pBaseAddrCfg);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API frees the NetCP resources allocated byNWAL
 *  @details Frees global resources allocated at NetCP. To be called only one
 *           per system
 *  @param[in]  nwalInst NWAL Instance  identifier
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        @see nwal_create
 */
nwal_RetValue nwal_delete(nwal_Inst     nwalInst);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API to retrieve global resources created by NWAL at the end of
 *          @see nwal_create() API
 *  @details API to retrieve global resources created by NWAL through
 *          @see nwal_create() API
 *  @param[in]  nwalInst NWAL Instance  identifier
 *  @param[out]  pInfo    NWAL global context information
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        @see nwal_create
 */
nwal_RetValue nwal_getGlobCxtInfo(nwal_Inst             nwalInst,
                                  nwalGlobCxtInfo_t*    pInfo);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API would need to be called for all cores as a pre-requisite.
 *          API allows local per core  related resource configuration to NWAL
 *
 *  @details  Following resource initialization is handled in  API:
 *           - Control and packet flows for each core
 *  @param[in]  nwalInst NWAL Instance  identifier
 *  @param[in]  pCfg    @see nwalLocCfg_t
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        @see nwal_create
 */
nwal_RetValue nwal_start (  nwal_Inst                   nwalInst,
                            const nwalLocCfg_t*         pCfg);

/**
 *  @ingroup nwal_api_functions
 *  @brief  Control API for additional global configuration at NetCP
 *
 *  @details This function results in configuration of system level rules to NetCP.
 *           API is blocking and will return only after successful configuration
 *           at NetCP.
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  pCtlInfo    Control Parameters
 *
 *  @retval      @see nwal_OK on success. Error
 *               codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_control(nwal_Inst                  nwalInst,
                           nwalCtlInfo_t*             pCtlInfo);
/**
 *  @ingroup nwal_api_functions
 *  @brief  Control API for Emac Port global configuration at NetCP
 *
 *  @details This function results in configuration of Emac Port level rules to NetCP.
 *           API is blocking and will return only after successful configuration
 *           at NetCP.
 *  @param[in]  nwalInst    NWAL Instance identifier
 *  @param[in]  pCtlInfo    Control Parameters per Emac Port
 *
 *  @retval      @see nwal_OK on success. Error
 *               codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_emacPortCfg(nwal_Inst                  nwalInst,
                                   paEmacPortConfig_t*             pCtlInfo);



/**
 *  @ingroup nwal_api_functions
 *  @brief  API to retrieve local per process resources created by NWAL at
 *          the end of @see nwal_start() APInwal_initPSCmdInfo
 *  @details API to retrieve local per process resources created by NWAL at
 *          the end of @see nwal_start() API
 *  @param[in]  nwalInst NWAL Instance  identifier
 *  @param[out]  pInfo    NWAL Local context information
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        @see nwal_create
 */
nwal_RetValue nwal_getLocCxtInfo(nwal_Inst              nwalInst,
                                 nwalLocCxtInfo_t*      pInfo);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API to retrieve internal context information from channel resources
 *          maintained by NWAL. Selective NetCP PA/SA channel handles are exposed
 *          to handle the case of multiple owner use case for PA/SA LLD
 *  @details API to retrieve internal context information from channel resources
 *          maintained by NWAL. Selective NetCP PA/SA channel handles are exposed
 *          to handle the case of multiple owner use case for PA/SA LLD
 *  @param[in]  nwalInst NWAL Instance  identifier
 *  @param[in]  nwalHandle NWAL handle returned from any config APIs
 *  @param[out]  pInfo    NWAL Channel context information
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        @see nwal_create
 */
nwal_RetValue nwal_getChanCxtInfo(nwal_Inst              nwalInst,
                                 nwal_Handle            nwalHandle,
                                 nwalChanCxtInfo_t*     pInfo);
/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_getMacIface  API will check for already configuration for
 *         MAC entry
 *
 *  @details The API lookups L2/MAC related configuration at NWAL.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  pParam      @see nwalMacParam_t
 *  @param[in]  pIfHandle   Output handle from NWAL if found.
 *                          @see nwal_Handle
 *
 *  @retval     nwal_TRUE if found / nwal_FALSE if not configured
 *  @pre        @see nwal_start
 */
nwal_Bool_t nwal_getMacIface  ( nwal_Inst             nwalInst,
                                nwalMacParam_t*       pParam,
                                nwal_Handle*          pIfHandle);

/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_setMacIface  Configures MAC LUT entry at NetCP.
 *
 *  @details The API configures L2/MAC related configuration to NetCP.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    The ID will be  returned back by NWAL in
 *                         nwal_CmdCallBack() upon completion of the
 *                         transaction. Set this to NWAL_TRANSID_SPIN_WAIT
 *                         in case of blocking API call
 *                         It is recommended not to initiate a blocking API call
 *                         when a callback event driven  API is in progress.
 *  @param[in]  appId       Application ID to be registered. The handle will be
 *                          used for all interfaces
 *                          from NWAL to application: 1) Asynchronous
 *                          confirmation of configuration response
 *                          2) Any packets received from NetCP related to this
 *                          handle
 *  @param[in]  pParam      Configuration parameters.
 *                          @see nwalMacParam_t
 *  @param[in]  pIfHandle   Output handle from NWAL. To be used for any next
 *                          route classification based on this interface handle.
 *                          @see nwal_Handle
 *
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        @see nwal_start
 */
nwal_RetValue nwal_setMacIface   (nwal_Inst             nwalInst,
                                  nwal_TransID_t        transId,
                                  nwal_AppId            appId,
                                  nwalMacParam_t*       pParam,
                                  nwal_Handle*          pIfHandle);

/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_delMacIface  Delete MAC LUT entry at NetCP.
 *
 *  @details The API deletes NetCP configuration received through
 *          @see nwal_setMacIface
 *
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    The ID will be  returned back by NWAL in
 *                         nwal_CmdCallBack()
 *                         upon completion of the transaction.Set this to
 *                         NWAL_TRANSID_SPIN_WAIT in case of blocking API call
 *                         It is recommended not to initiate a blocking API
 *                         call when a callback event driven API is in progress.
 *  @param[in]  ifHandle   Handle returned to application from
 *                         @see nwal_setMacIface.
 *
 *  @retval     @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre        @see nwal_setMacIface
 */
nwal_RetValue nwal_delMacIface   (nwal_Inst         nwalInst,
                                  nwal_TransID_t    transId,
                                  nwal_Handle       ifHandle);
/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_getIPAddr  API will check if there is already IP Address
 *                         configured by application.If found returns the handle
 *
 *  @details The API lookups IP related configuration at NWAL.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  pParam      IP Configuration parameters
 *  @param[in]  pPrevHandle  Dependent MAC/IP handle
 *  @param[in]  pIpHandle   Output handle from NWAL if found.
 *                          @see nwal_Handle
 *
 *  @retval                 Value @see nwal_TRUE if found @see nwal_FALSE if not
 *                          configured
 *  @pre         @see nwal_setMacIface
 */
nwal_Bool_t nwal_getIPAddr (    nwal_Inst          nwalInst,
                                nwalIpParam_t*     pParam,
                                nwal_Handle        pPrevHandle,
                                nwal_Handle*       pIpHandle);
/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_setIPAddr  Add IP Address configuration to NETCP.
 *
 *  @details API configures NetCP to add classification rule for local
 *          IP Address
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    The ID will be  returned back by NWAL in
 *                         nwal_CmdCallBack() upon completion of the
 *                         transaction.Set this to NWAL_TRANSID_SPIN_WAIT in
 *                         case of blocking API call
 *                         It is recommended not to initiate a blocking API
 *                         call when a callback event driven API is in progress.
 *  @param[in]  appId   Application ID to be registered. The handle will be
 *                      used for all interfaces from NWAL to application:
 *                      1) Asynchronous confirmation of configuration response
 *                      2) Any packets received from NetCP related to this
 *                         handle
 *  @param[in]  ifHandle    Handle returned to application from
 *                          @see nwal_setMacIface.
 *  @param[in]  pParam      Configuration parameters for IP classification
 *  @param[out] pIpHandle   Handle for IP related resource
 *
 *  @retval      @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_setMacIface
 */
nwal_RetValue nwal_setIPAddr  ( nwal_Inst               nwalInst,
                                nwal_TransID_t          transId,
                                nwal_AppId              appId,
                                nwal_Handle             ifHandle,
                                nwalIpParam_t*          pParam,
                                nwal_Handle*            pIpHandle);

/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_deleteIPAddr  Delete IP Address configuration at NETCP.
 *
 *  @details API removes NetCP IP Addres classification rule for local
 *           IP Address
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    The ID will be  returned back by NWAL in
 *                         nwal_CmdCallBack()
 *                         upon completion of the transaction.Set this to
 *                         NWAL_TRANSID_SPIN_WAIT in case of blocking API call
 *                         It is recommended not to initiate a blocking API
 *                         call when a callback event driven API is in progress.
 *  @param[in]  ipHandle   Handle returned to application from
 *                          @see nwal_setIPAddr.
 *
 *  @retval      @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_setIPAddr
 */
nwal_RetValue nwal_delIPAddr  ( nwal_Inst       nwalInst,
                                nwal_TransID_t  transId,
                                nwal_Handle     ipHandle);


/**
 *  @ingroup nwal_api_functions
 *  @brief  Get an IPSec Security Association Channel
 *
 *  @details This function looks up for an existing outer IPSec Security
 *           Association Channel.
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  pSaId       SA ID uniquely identifying Tunnel
 *  @param[in]  dir         Direction @see NWAL_SA_DIR_INBOUND or
 *                          @see NWAL_SA_DIR_OUTBOUND
 *  @param[out] pNwalSecAssocHandle  Returns nwal_TRUE with NWAL Handle for
 *                                  SA channel
 *                          if found, nwal_FALSE if not found return NULL
 *                          as handle
 *  @param[out] pSwInfo0  SA Security Channel Context as identified by NetCP
 *                        word 0
 *  @param[out] pSwInfo1  SA Security Channel Context as identified by NetCP
 *                        word 1
 *  @retval               Value @see nwal_TRUE if found @see nwal_FALSE if not
 *                        configured
 *  @pre         @see nwal_setMacIface
 */
nwal_Bool_t nwal_getSecAssoc(   nwal_Inst               nwalInst,
                                nwalSaIpSecId_t*        pSaId,
                                nwal_SaDir              dir,
                                nwal_Handle*            pNwalSecAssocHandle,
                                uint32_t*               pSwInfo0,
                                uint32_t*               pSwInfo1);

			     
/**
 *  @ingroup nwal_api_functions
 *  @brief  API to create outer IPSec Security Association Channel
 *           For inbound direction API will trigger allocating resources at
 *           NetCP. Application can either
 *           block by passing transaction ID as NWAL_TRANSID_SPIN_WAIT or wait
 *           for call back being called with the results
 *           For outbound, API will return nwal_TRANS_COMPLETE and no further
 *           callback will be called.Application can free transaction ID after
 *           returning from API.
 *  @details This function create outer IPSec Security Association Channel.
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    Transaction Id maintained by application.
 *                         Applicable only in the case of inbound direction.
 *                         The ID will be  returned back by NWAL in
 *                          nwal_CmdCallBack()
 *                         upon completion of the transaction.Set this to
 *                         NWAL_TRANSID_SPIN_WAIT in case of blocking API call
 *                         It is recommended not to initiate a blocking API call
 *                         when a callback event driven
 *                         API is in progress.
 *  @param[in]  appId   Application ID to be registered. The handle will be used
 *                      for all interfaces  from NWAL to application:
 *                          1) Asynchronous  confirmation of configuration
 *                             response
 *                          2) Any packets received from NetCP related to this
 *                          handle
 *  @param[in]  pSaId       SA ID uniquely identifying Tunnel
 *  @param[in]  pCreateParam  Configuration parameters.
 *  @param[out] pNwalSecAssocHandle  NWAL Handle for SA channel
 *
 *  @retval      @see nwal_OK or @see nwal_TRANS_COMPLETE on success. Error
 *               codes @see nwal_RetValue
 *  @pre         @see nwal_setMacIface
 */
nwal_RetValue nwal_setSecAssoc( nwal_Inst               nwalInst,
                                nwal_TransID_t          transId,
                                nwal_AppId              appId,
                                nwalSaIpSecId_t*        pSaId,
                                nwalCreateSAParams_t*   pCreateParam,
                                nwal_Handle*            pNwalSecAssocHandle);
#ifdef NWAL_ENABLE_SA
/**
 *  @ingroup nwal_api_functions
 *  @brief  API to query the SA (IPSec) channel Stats.
 *
 *  @details This function to query IPSec channel Stats.
 *  @param[in]  nwalInst            NWAL Instance  identifier
 *  @param[in]  nwalSecAssocHandle  NWAL Handle for SA channel
 *  @param[out] pSaIpsecStats       Output IPSec stats
 *
 *  @retval      @see nwal_OK or @see nwal_TRANS_COMPLETE on success. Error
 *               codes @see nwal_RetValue
 *  @pre         @see nwal_setMacIface
 */
nwal_RetValue nwal_getSecAssocStats(nwal_Inst          nwalInst,
                                    nwal_Handle        nwalSecAssocHandle,
                                    Sa_IpsecStats_t*   pSaIpsecStats);
#endif
/**
 *  @ingroup nwal_api_functions
 *  @brief  API to delete an existing Policy Handle
 *           For inbound direction API will trigger allocating resources at
 *           NetCP. Application can either
 *           block by passing transaction ID as NWAL_TRANSID_SPIN_WAIT or wait
 *           for call back being called with the results
 *           For outbound, API will return nwal_TRANS_COMPLETE and no further
 *           callback will be called.Application can free transaction ID after
 *           returning from API.
 *  @details This function results in  freeing up SA related resource
 *           for Security profile related resources
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    Transaction Id maintained by application. Applicable
 *                         only in the case of inbound direction.
 *                         The ID will be  returned back by NWAL in
 *                         nwal_CmdCallBack()
 *                         upon completion of the transaction.Set this to
 *                         NWAL_TRANSID_SPIN_WAIT in case of blocking API call
 *                         It is recommended not to initiate a blocking API call
 *                         when a callback event driven
 *                         API is in progress.
 *  @param[in]  nwalSecAssocHandle   Connection handle returned from
 *                          @see nwal_setSecAssoc
 *
 *  @retval      @see nwal_OK or @see nwal_TRANS_COMPLETE on success.
 *               Error codes @see nwal_RetValue
 *  @pre         @see nwal_setSecAssoc
 */
nwal_RetValue nwal_delSecAssoc( nwal_Inst           nwalInst,
                                nwal_TransID_t      transId,
                                nwal_Handle         nwalSecAssocHandle);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API to retrieve handle for existing security policy for a connection
 *
 *  @details API to retrieve handle for existing security policy for a
 *           connection.
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  pPolParam   Configuration information for Security profile for
 *                          IPSec Channel.
 *  @param[out] pNwalSecPolHandle  Returns nwal_TRUE with NWAL Handle for SA
 *              channel if found, nwal_FALSE if not found with NULL as handle
 *  @retval                 Value @see nwal_TRUE if found @see nwal_FALSE if not
 *                          configured
 *  @pre         @see nwal_setSecAssoc
 */
nwal_Bool_t nwal_getSecPolicy(  nwal_Inst           nwalInst,
                                nwalSecPolParams_t* pPolParam,
                                nwal_Handle*        pNwalSecPolHandle);
/**
 *  @ingroup nwal_api_functions
 *  @brief  API to create policy for a connection
 *
 *  @details This function results in configuration of Inner IP LUT entry for
 *           the RX side.
 *           For inbound direction API will trigger allocating resources at
 *           NetCP. Application can either
 *           block by passing transaction ID as NWAL_TRANSID_SPIN_WAIT or wait
 *           for call back being called with the results
 *           For outbound, API will return nwal_TRANS_COMPLETE and no further
 *           callback will be called.Application can free transaction ID after
 *           returning from API.
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    Transaction Id maintained by application. Applicable
 *                         only in the case of inbound Sec Policy.
 *                         The ID will be  returned back by NWAL in
 *                         nwal_CmdCallBack()  upon completion of the
 *                         transaction.Set this to NWAL_TRANSID_SPIN_WAIT in
 *                         case of blocking API call
 *                         It is recommended not to initiate a blocking API call
 *                         when a callback event driven API is in progress.
 *  @param[in]  appId   Application ID to be registered. The handle will be
 *                      used for all interfaces from NWAL to application:
 *                      1) Asynchronous confirmation of configuration response
 *                      2) Any packets received from NetCP related to this
 *                         handle
 *  @param[in]  pPolParam   Configuration information for Security profile for
 *                          IPSec Channel.
 *  @param[out] pNwalSecPolHandle  NWAL Handle identifying the policy
 *                                 configuration
 *
 *  @retval      @see nwal_OK or @see nwal_TRANS_COMPLETE on success. Error
 *               codes @see nwal_RetValue
 *  @pre         @see nwal_setSecAssoc
 */
nwal_RetValue nwal_setSecPolicy(nwal_Inst           nwalInst,
                                nwal_TransID_t      transId,
                                nwal_AppId          appId,
                                nwalSecPolParams_t* pPolParam,
                                nwal_Handle*        pNwalSecPolHandle);


/**
 *  @ingroup nwal_api_functions
 *  @brief  API to delete an existing Policy Handle
 *
 *  @details This function results in initiation of freeing up resource for
 *           Security profile related resources.
 *           For inbound direction API will trigger allocating resources at
 *           NetCP. Application can either block by passing transaction ID as
 *           NWAL_TRANSID_SPIN_WAIT or wait for call back being called with
 *           the results
 *           For outbound, API will return nwal_TRANS_COMPLETE and no further
 *           callback will be called.Application can
 *           free transaction ID after returning from API.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    Transaction Id maintained by application.
 *                         Applicable only in the case of inbound Sec Policy.
 *                         The ID will be  returned back by NWAL in
 *                        nwal_CmdCallBack() upon completion of the transaction.
 *                         Set this to zero in case of blocking API call
 *                         It is recommended not to initiate a blocking API call
 *                         when a callback event driven API is in progress.
 *  @param[in]  nwalSecPolHandle   Connection handle returned from
 *                                 nwal_setSecPolicy.
 *
 *  @retval      @see nwal_OK or @see nwal_TRANS_COMPLETE on success.
 *               Error codes @see nwal_RetValue
 *  @pre         @see nwal_setSecPolicy
 */
nwal_RetValue nwal_delSecPolicy(    nwal_Inst           nwalInst,
                                    nwal_TransID_t      transId,
                                    nwal_Handle         nwalSecPolHandle);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API to create connection establishment
 *
 *  @details This function results in configuration of L4 LUT entry for the
 *           RX side.For TX side the function will collect all details to
 *           enable header creation. Supports RX only or RX/TX channel
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    Transaction Id maintained by application.
 *                         Applicable only in the case of inbound Sec Policy.
 *                         The ID will be  returned back by NWAL in
 *                         nwal_CmdCallBack() upon completion of the transaction.
 *                         Set this to zero in case of blocking API call
 *                         It is recommended not to initiate a blocking API call
 *                         when a callback event driven API is in progress.
 *  @param[in]  appId   Application ID to be registered. The handle will be
 *                      used for all interfaces  from NWAL to application:
 *                      1) Asynchronous confirmation of configuration response
 *                      2) Any packets received from NetCP related to this
 *                         handle
 *  @param[in]  proto    Layer 4 protocol type
 *  @param[in]  pRxConnCfg    Configuration for RX packets for the channel.
 *  @param[in]  pTxConnCfg    [Optional] Configuration for TX packets for the
 *                            channel. Will be applicable if NWAL module is used
 *                            for creating TX header for the channel.If provided
 *                            as NULL connection will be configured as RX only
 *  @param[out] pNwalConHandle  Connection handle being returned to application
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_setSecPolicy for IPSec or @see nwal_setIPAddr for
 *               non IPSec
 */
nwal_RetValue nwal_addConn( nwal_Inst           nwalInst,
                            nwal_TransID_t      transId,
                            nwal_AppId          appId,
                            nwal_appProtoType_t proto,
                            nwalRxConnCfg_t*    pRxConnCfg,
                            nwalTxConnCfg_t*    pTxConnCfg,
                            nwal_Handle*        pNwalConHandle);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API to delete an existing connection
 *
 *  @details This function results in deletion of existing connection
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    Transaction Id maintained by application.
 *                         Applicable only in the case of inbound Sec Policy.
 *                         The ID will be  returned back by NWAL in
 *                         nwal_CmdCallBack() upon completion of the transaction.
 *                         Set this to zero in case of blocking API call
 *                         It is recommended not to initiate a blocking API call
 *                         when a callback event driven API is in progress.
 *  @param[in]  nwalConHandle  Connection handle being returned to application
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_addConn
 */
nwal_RetValue nwal_delConn( nwal_Inst           nwalInst,
                            nwal_TransID_t      transId,
                            nwal_Handle         nwalConHandle);


/**
 *  @ingroup nwal_api_functions
 *  @brief  API for run time configuration connection establishment
 *
 *  @details This function results in configuration of header details for
 *           TX side
 *
 *  @param[in]  nwalInst        NWAL Instance  identifier
 *  @param[in]  nwalConHandle  Connection handle to be reconfigured
 *  @param[in]  pTxConnCfg     Configuration for TX packets for the channel.
 *                             Will be applicable if NWAL module is used for
 *                             creating TX header for the channel.
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_addConn
 */
nwal_RetValue nwal_cfgConn(     nwal_Inst           nwalInst,
                                nwal_Handle         nwalConHandle,
                                nwalTxConnCfg_t*    pTxConnCfg);


/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_refreshConn  Refreshes a connection for the core.
 *  @details API to be used to refresh an NWAL handle created external to the
 *           core
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  nwalHandle  Handle created external to the core. The handle
 *                          could be the one returned from following APIs
 *                          @see nwal_setMacIface,
 *                          @see nwal_setIPAddr , @see nwal_setSecAssoc,
 *                          @see nwal_setSecPolicy
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_addConn
 */
nwal_RetValue nwal_refreshConn( nwal_Inst               nwalInst,
                                nwal_Handle             nwalHandle);

/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_getPAStats  Get Statistics from PA
 *
 *  @details The API is to querry the stats from PA.If multiple request is
 *           initiated callback will be only called with latest transId
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  transId    Transaction Id maintained by application.
 *                         Applicable only in the case of inbound Sec Policy.
 *                         The ID will be  returned back by NWAL in
 *                         nwal_CmdPaStatsReply() upon completion of the
 *                         transaction.Set this to zero in case of blocking API
 *                         call. It is recommended not to initiate a blocking
 *                         API call when a callback event driven API is in
 *                         progress.
 *  @param[in]  pPaStats   Valid only if pStatsCallback is NULL and stats
 *                         requested as blocking call
 *  @param[in]  doClear    True to clear the stats. Alternatively False
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_getPAStats    (nwal_Inst             nwalInst,
                                  nwal_TransID_t        transId,
                                  paSysStats_t*         pPaStats,
                                  nwal_Bool_t           doClear);

#ifdef NWAL_ENABLE_SA
/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_getSASysStats  Get Global SA Statistics
 *
 *  @details The API is to querry global stats from SA.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  pPaStats   Valid only if pStatsCallback is NULL and stats
 *                         requested as blocking call
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_getSASysStats (nwal_Inst             nwalInst,
                                  Sa_SysStats_t*        pSaStats);

#endif
/**
 *  @ingroup nwal_api_functions
 *  @brief  nwal_setDMSecAssoc  API for creating Data Mode Security Association
 *  @details API for creating Data Mode Security Association
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  appId   Application ID to be registered. NWAL will pass the
 *                      same ID back for the payload received back from NetCP
 *                      after crypto completion
 *  @param[in]  pCreateParam  Configuration parameters.
 *  @param[out] pNwalDmSaHandle  NWAL Handle for SA Data mode channel
 *
 *  @retval      @see nwal_OK on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_setDMSecAssoc( nwal_Inst               nwalInst,
                                  nwal_AppId              appId,
                                  nwalCreateDmSAParams_t* pCreateParam,
                                  nwal_Handle*            pNwalDmSaHandle);

/**
 *  @ingroup nwal_api_functions
 *  @brief  API to delete an existing Data Mode Security Channel
 *  @details This function results in  freeing up SA related resources for
 *           Data Mode channel
 *
 *  @param[in]  nwalInst        NWAL Instance  identifier
 *  @param[in]  nwalDmSaHandle  Data Mode SA Handle @see nwal_setDMSecAssoc
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_setDMSecAssoc
 */
nwal_RetValue nwal_delDMSecAssoc( nwal_Inst           nwalInst,
                                  nwal_Handle         nwalDmSaHandle);


#ifdef NWAL_ENABLE_SA
/**
 *  @ingroup nwal_api_functions
 *  @brief  API to querry the SA (Data Mode) channel Stats.
 *
 *  @details This function querries Data Mode Channel Stats.
 *  @param[in]  nwalInst          NWAL Instance  identifier
 *  @param[in]  nwalDmSaHandle    NWAL Handle for SA channel
 *  @param[out] pSaDataModeStats  Data Mode stats
 *
 *  @retval      @see nwal_OK or @see nwal_TRANS_COMPLETE on success. Error
 *               codes @see nwal_RetValue
 *  @pre         @see nwal_setMacIface
 */
nwal_RetValue nwal_getDataModeStats(nwal_Inst           nwalInst,
                                    nwal_Handle         nwalDmSaHandle,
                                    Sa_DataModeStats_t* pSaDataModeStats);
#endif
/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_sendRaw  Transmit raw packet to the PA
 *
 *  @details The API transmits the raw packet to the PA.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  lpbackPass  Loopback the packet at NetCP PASS. Useful
 *                          for debugging any classification action
 *                          at NetCP
 *  @param[in]  bufLen      Length of buffer being transmitted.
 *  @param[in]  pBuf        Byte array with the payload
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_sendRaw  (nwal_Inst      nwalInst,
                             nwal_Bool_t    lpbackPass,
                             uint16_t       bufLen,
                             uint8_t*       pBuf);


/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_updateProtoHdr  Update protocol header to the packet
 *
 *  @details The API will result in prepending the packet
 *           with:MAC/[IPSec]/IP/UDP header for the packet based on the
 *           @see nwalTxConnCfg_t information for the connection.
 *           Recommended sequence with minimal cycle consumption would be
 *           1) Call nwal_updateProtoHdr for every packet
 *           2) Call nwal_initPSCmdInfo only once to get PS Command for
 *              NetCP offload
 *           3) Call nwal_updateXXX macros or nwal_send() for transmission
 *              of packets
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  nwalHandle  Should be connHandle returned from @see
 *                          nwal_addConn
 *  @param[in]  pPktInfo    TX metadata with pkt containing payload.
 *                          API will update header specific offset information
 *                          to the metadata during update @see nwalTxPktInfo_t
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_updateProtoHdr  (nwal_Inst         nwalInst,
                                    nwal_Handle       nwalHandle,
                                    nwalTxPktInfo_t*  pPktInfo);


/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_send  Transmit  packet out for a connection
 *
 *  @details The API will result in transmission of packets out after creation
 *           of necessary command labels.
 *           Application is required to handle any garbage collection for
 *           freeing the packet if return queue for the packet is not pointing
 *           to free queue.
 *
 *           In the case of Application requiring NWAL to update L2/L3/L4
 *           protocol header before transmit, API  @see nwal_updateProtoHdr
 *           need to be invoked before calling @see nwal_send.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  nwalHandle  Can be
 *                          - nwalSecAssocHandle  NWAL Handle for SA channel
 *                            returned from @see nwal_setSecAssoc API in case
 *                            if application has inserted the protocol headers
 *                          - Ignored in all other cases
 *  @param[in]  pPktInfo    Transmit packet information @see nwalTxPktInfo_t
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_send  (nwal_Inst         nwalInst,
                          nwal_Handle       nwalHandle,
                          nwalTxPktInfo_t*  pPktInfo);

/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_sendDM  Transmit payload for side band Data Mode Channel
 *
 *  @details The API will result in transmission of payload for side band
 *           crypto offload to SA for data mode channel
 *
 *  @param[in]  nwalInst        NWAL Instance  identifier
 *  @param[in]  nwalDmSaHandle  Data Mode SA Handle @see nwal_setDMSecAssoc
 *  @param[in]  pDmPloadInfo    Data Mode Payload with meta data
 *                              @see nwalDmTxPayloadInfo_t
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_sendDM(nwal_Inst                 nwalInst,
                          nwal_Handle               nwalDmSaHandle,
                          nwalDmTxPayloadInfo_t*    pDmPloadInfo);


/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_pollCtl  API for polling Control message response from NetCP:
 *                       PA Subsystem.
 *
 *  @details The API will poll for control message response from NetCP PA
 *           Subsystem
 *  @param[in]  nwalInst            NWAL Instance  identifier
 *  @param[in]  pCmdCallBack        Optional:Callback for configuration
 *                                  response processing at application.
 *                                  If passed as NULL,NWAL will use default
 *                                  callback passed during @see nwal_start
 *  @param[in]  pPaStatsCallBack    Optional:Callback for PA Stats response
 *                                  processing at application.
 *                                  If passed as NULL, NWAL will use default
 *                                  callback passed during @see nwal_start
 *  @pre       @see nwal_start
 */
void nwal_pollCtl(  nwal_Inst                nwalInst,
                    nwal_CmdCallBack*        pCmdCallBack,
                    nwal_CmdPaStatsReply*    pPaStatsCallBack);


/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_pollPkt  API for polling packets from network.
 *
 *  @details The API will poll for packets from Network. In case if application
 *           provides  an application managed packet Queue, module will poll
 *           packet from that Queue.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  pktQCtl     Bitmap for polling packets from queue for poll
 *                          @see nwal_pollPktQCtl
 *  @param[in]  appCookie   Optional Application context information if required
 *                          for a poll. Set to NULL if not used
 *  @param[in]  maxPkts     Poll and accumulate upto this count per Queue.
 *                          Maximum value can be  @see NWAL_MAX_RX_PKT_THRESHOLD
 *  @param[in]  appRxPktQueue    Optional. Application managed Queue Handle.
 *                          Required only if API is called for
 *                          @see nwal_POLL_APP_MANAGED_PKT_Q. Set to
 *                          NWAL_QUEUE_NOT_SPECIFIED if not used
 *  @param[in]  pRxPktCallBack  Optional:Callback for packet processing at
 *                              application. Set to NULL for NWAL to use default
 *  @retval     Number of packets returned through callback
 *                                callback configured during @see nwal_start
 *  @pre       @see nwal_start
 */
uint16_t nwal_pollPkt(nwal_Inst            nwalInst,
                     nwal_pollPktQCtl      pktQCtl,
                     uint32_t              appCookie,
                     uint16_t              maxPkts,
                     NWAL_queueHnd         appRxPktQueue,
                     nwal_rxPktCallBack*   pRxPktCallBack);

/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_pollDm  API for polling Data Mode payload from NetCP after
 *                      Crypto completion
 *
 *  @details The API will poll for payload/packet back from NetCP after Crypto
 *           completion. In case if application provides an application managed
 *           packet Queue, module will poll packet from that Queue.
 *
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  dmQCtl      Bitmap for polling packets from queue for poll
 *                          @see nwal_pollDmQCtl
 *  @param[in]  appCookie   Optional Application context information if required
 *                          for a poll callback. Set to NULL if not used
 *  @param[in]  maxPkts     Poll and accumulate upto this count per Queue.
 *                          Maximum value can be  @see NWAL_MAX_RX_PKT_THRESHOLD
 *  @param[in]  appRxQueue  Optional. Application managed Queue Handle.
 *                          Required only if API is called for @see
 *                          nwal_POLL_APP_MANAGED_PKT_Q. Set to
 *                          NWAL_QUEUE_NOT_SPECIFIED if not used
 *  @param[in]  pRxDmCallBack   Optional:Callback for packet processing at
 *                              application. Set to NULL for NWAL to use
 *                              default
 *  @retval     Number of packets returned through callback
 *                                callback configured during @see nwal_start
 *  @pre       @see nwal_start
 */
uint16_t nwal_pollDm(nwal_Inst              nwalInst,
                     nwal_pollDmQCtl        dmQCtl,
                     uint32_t               appCookie,
                     uint16_t               maxPkts,
                     NWAL_queueHnd          appRxQueue,
                     nwal_rxDmCallback*     pRxDmCallBack);

/*@}*/ /* @name COMMON APIs */

/*@}*/ /* @name COMMON APIs End Doxygen groups*/
#ifdef __cplusplus
}
#endif


#endif  /* _NWAL_H */
