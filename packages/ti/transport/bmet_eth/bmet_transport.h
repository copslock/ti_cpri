/**
 *   @file  bmet_transport.h
 *
 *   @brief   
 *      This is a very slim library implementation on top of Keystone NetCP
 *      to send a buffer with a pre-compiled packet header. This file has 
 *      Bare metal transport definitions and API's to configure the 
 *      Bare Metal Ethernet Transport Module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012 - 2018, Texas Instruments, Inc.
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
 
#ifndef __BMET_TRANSPORT_H__
#define __BMET_TRANSPORT_H__

/** 
 * @mainpage BareMetalEthernetTransport (BMET ETH) library
 *
 * Defines the Functions, Data Structures, Enumerations and Macros
 * within BMET (Bare Metal Ethernet Transport) module.
 *
 */


/* QMSS Include */
#include "ti/drv/qmss/qmss_drv.h"

/* CSL include files */
#include "ti/csl/cslr_device.h"
#include "ti/csl/csl_chip.h"
#include "ti/csl/csl_chipAux.h"

/* CPPI include files */
#include "ti/drv/cppi/cppi_drv.h"
#include "ti/drv/cppi/cppi_desc.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BMET_ETHERNET_INFO BMET Ethernet Trasnport Information
 */
/*@{*/

/**
 * @brief   This is the transport_HANDLE  
 */
typedef void *  transport_HANDLE;

/**
 * @brief   Macros for the transport
 */
#ifdef _BIG_ENDIAN 
#define  htons(a) (a)
#define  htonl(a) (a)
#define  ntohl(a) (a)
#define  ntohs(a) (a)
#else
#define  htons(a)    ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )
#define  htonl(a)    ( (((a)>>24)&0xff) + (((a)>>8)&0xff00) + \
                       (((a)<<8)&0xff0000) + (((a)<<24)&0xff000000) )
#define  ntohl(a)   htonl(a)
#define  ntohs(a)   htons(a)
#endif /* _BIG_ENDIAN */


/**
 * @brief   Keystone I Ethernet Queue number
 */
#define BMET_ETH_TX_QUEUE_NUM_KEYSTONE1         648

/**
 * @brief  Maximum number of descriptors the bare metal ethernet transport can handle 
 */
#define BMET_MAX_NUM_TRANSPORT_DESC             1024

/*@}*/  /* defgroup */

/** @defgroup BMET_PACKET_HEADER BMET Ethernet Packet Header Defines
 */
/*@{*/
/**
 * @brief  Maximum number of descriptors the bare metal ethernet transport can handle 
 */
/* Parmeters for network header */
#define BMET_ETHHDR_SIZE     14
#define BMET_VLANHDR_SIZE    4
#define BMET_IPHDR_SIZE      20
#define BMET_UDPHDR_SIZE     8

/**
 * @brief   Total network packet header size
 */
#define BMET_PKT_HEADER_SIZE (BMET_ETHHDR_SIZE + BMET_VLANHDR_SIZE + BMET_IPHDR_SIZE + BMET_UDPHDR_SIZE)

/**
 * @brief   This is the protocol identification field in the Ethernet
 * header which identifies the packet as an IPv4 packet.
 */
#define BMET_ETH_IP                  0x800

/**
 * @brief   This is the protocol identification field in the Ethernet
 * header which identifies the packet as a VLAN Packet.
 */
#define BMET_ETH_VLAN                0x8100

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as a UDP packet.
 */
#define BMET_IPPROTO_UDP             17

/**
 * @brief   This is the EthHeader  
 */
typedef struct Bmet_EthHeader
{
    uint8_t   DstMac[6];
    uint8_t   SrcMac[6];
    uint16_t  Type;
}Bmet_EthHeader;

/**
 * @brief   This is the VLANHeader  
 */
typedef struct Bmet_VLANHeader
{
    uint16_t  tci;
    uint16_t  protocol;
}Bmet_VLANHeader;

/**
 * @brief   This is the IPHeader  
 */
typedef struct Bmet_IPHeader
{
    uint8_t    VerLen;
    uint8_t    Tos;
    uint16_t   TotalLen;
    uint16_t   Id;
    uint16_t   FlagOff;
    uint8_t    Ttl;
    uint8_t    Protocol;
    uint16_t   Checksum;
    uint8_t    IPSrc[4];
    uint8_t    IPDst[4];
    uint8_t    Options[1];
}Bmet_IPHeader;

/**
 * @brief   This is the UDPHeader  
 */
typedef struct Bmet_UDPHeader
{
    uint16_t   SrcPort;
    uint16_t   DstPort;
    uint16_t   Length;
    uint16_t   UDPChecksum;
}Bmet_UDPHeader;

/*@}*/  /* defgroup */


/** @defgroup  BMET_Status BMET Status Flags 
 */
/*@{*/

/**
 *  @brief This structure contains BMET Status information
 */
typedef enum bmetStatus 
{
  BMET_ATTCHED_BD_NULL = -9,
  /**<BMET attached BD found null */
  BMET_ATTCHED_BD_NOT_MATCHED = -8,
  /**<BMET attached BD descriptor does not match */
  BMET_DESC_FOUND_FREE_BEFORE_FREE = -7,
  /**<BMET descriptor found free before freed by library */
  BMET_DESC_INDEX_EXCEED_FAIL = -6,
  /**< BMET descriptor index exceeded the configured limit */  
  BMET_QUEUEOPEN_FAIL = -5,
  /**< BMET QUEUE OPEN is not successful */
  BMET_DESC_ALOCFAIL = -4,
  /**< BMET Descriptor allocation failed */  
  BMET_FAIL = -3, 
  /**< BMET General operation failed */
  BMET_UNSUPPORTED = -2,
  /**< BMET Unsupported arguments */
  BMET_DESC_NOTAVAILABLE = -1,
  /**< BMET No descriptor not available for sending the packet */   
  BMET_SUCCESS = 0
  /**< BMET ethernet created succesfully */   
} bmetStatus_e;

/**
 *  @brief This structure contains BMET Channel Statistics Information
 */
typedef struct  bmetChStats
{
  uint32_t pktsSentCount;
  /**< total number of logs (packets) sent per core */  
  uint32_t pktsReturnCount;
  /**< Total number of returned pkts that were poped from the return queue */  
  uint32_t returnQueueNum;
  /**< Assigned number to the return queue for the BMET transport */  
  uint32_t noPdAvailableCount;
  /**< Number of packets that were not sent because no packet descriptor PD was available */
} bmetChStats_t;


/*@}*/  /* defgroup */

/** @defgroup  BMET_Configuration BMET Configuration Flags 
  */
/*@{*/

/**
 *  @brief Indicates the VLAN Header presense 
 */
typedef enum bmetVlanHdrInfo 
{
  BMET_NO_VLAN_HEADER,
  /**< The precompiled packet would not have VLAN header information */
  BMET_PUT_VLAN_HEADER
  /**< The precompiled packet would have VLAN header information */
} bmetVlanHdrInfo_e;

/**
 *  @brief bmet channel configuration  structure
 */
typedef struct bmetChConfig
{
  uint32_t         moduleId;
  /**< ModuleId provided by the system */

  uint32_t         eth_tx_queue_num;
  /**< Ethernet Queue Number */

  Qmss_QueueHnd     global_free_queue_hnd;
  /**< System Free Queue pool with some Descriptors available */
  
  uint32_t          num_trasport_desc;
  /**< Number of Transport descriptors */ 

  uint32_t    sgmii_send_port_num;
  /**< SGMII send port configuration for the descriptor */  

  uint32_t      desc_size;
  /**< descriptor size */  

  uint32_t         payload_size;
  /**< Payload Size */  

  uint32_t       send_vlan_header;
  /**< VLAN Header persence */  
  
  uint8_t  IPDst[4];
  /**< Source IP address */

  uint8_t  MacDst[6];
  /**< Destination MAC address */  

  uint8_t  IPSrc[4];
  /**< Source IP address */  

  uint8_t  MacSrc[6];  
  /**< Source MAC address */  

  uint32_t local_udp_port;
  /**< Local UDP Port number */

  uint32_t remote_udp_port;
  /**< remote UDP port number */
} bmetChConfig_t;

/*@}*/  /* defgroup */

/** @defgroup  BMET_Instance BMET Instance Variables 
 */
/*@{*/

/**
 *  @brief structure with index info for the array of Packet descriptors
 */
typedef struct  bmetDescArrayIndex
{
  uint32_t  nextFreeDesc;
  /**< index for the next available PD that can be used */  
  uint32_t  numFreeDesc;
  /**< total number fo free descriptors and is ready to be used */  
} bmetDescArrayIndex_t;

/**
 *  @brief  Structure with info for each descriptor that can be used
 *  for transporting
 */
typedef struct  bmetDescInfo
{
  Cppi_HostDesc*  packetDesc;
  /**< pointer to packet descriptor */  
  Cppi_HostDesc*  bufferDesc;
  /**< pointer to buffer descriptor */  
  uint32_t        free;
  /**< Descriptor free or not? TRUE or FALSE */  
} bmetDescInfo_t;

/**
 *  @brief  BMET Instance Structure
 */
typedef struct  bmetInst
{
  uint8_t   bmetPktHeader[BMET_PKT_HEADER_SIZE];  
  /**< Pre-build packet header, please don't move this needs to be cache aligned */  

  bmetDescArrayIndex_t desc_array_index;
  /**< Current descriptor array index to use */
  uint32_t        host_desc_size;
  /**< Host descriptor size */
  
  bmetChStats_t  bmetCh_stats;
  /**< channel status information */

  Qmss_QueueHnd     system_free_queue_hnd;  
  /**< System Free Queue pool with some Descriptors available */  

  Qmss_QueueHnd   gTxReturnQHnd;  
  /**< BMET Transmit Complete Queue. */  

  Qmss_QueueHnd   ethTxQueueHnd;
  /**< Ethernet Tx Queue */  
  
  uint32_t          numTrasportDesc;
  /**< Number of Transport descriptors */  

  uint32_t         payload_size;
  /**< Payload Size */  

  bmetDescInfo_t  bmetDescList[BMET_MAX_NUM_TRANSPORT_DESC/2];
  /**< Max Desc Info BMET can handle */  
  
  uint32_t         moduleId;
  /**< ModuleId configured by the system */
} bmetInst_t;

/*@}*/  /* defgroup */

/** @defgroup BMET_API Ethernet Trasnport API Information
 */
/*@{*/

/**
 * ============================================================================
 *  @n@b bmet_destroy
 *
 *  @b  brief
 *  @n  Frees up bmet instance created (Returns back all the descriptors back to
 *      system
 *
 *  @param[in]  handle
 *      Pointer to the bmet transport handle to be destroyed
 *
 *  @return
 *      Status of the operation
 *
 * =============================================================================
 */
bmetStatus_e             bmet_destroy(transport_HANDLE handle);

/**
 * ============================================================================
 *  @n@b bmet_send
 *
 *  @b  brief
 *  @n  sends a buffer over netcp with precompiled packet header
 *
 *  @param[in]  handle
 *      Pointer to the transport handle
 *
 *  @param[in]  logBuf
 *      Pointer to the logger buffer to be send over NetCP
 *
 *  @param[in]  identity
 *      ID of the send to be put in the descriptor; used to get the send status
 *      Should not be set to -1 since -1 is used as error return value
 *
 *  @return
 *      Status of the operation (0 for Success, -1 for fail)
 *
 * =============================================================================
 */
int32_t  bmet_send(transport_HANDLE handle, uint8_t* logBuf, int32_t identity);

/**
 * ============================================================================
 *  @n@b bmet_status
 *
 *  @b  brief
 *  @n  get the status on the packets send earlier
 *
 *  @param[in]  handle
 *      Pointer to the transport handle
 *
 *
 *  @return
 *      identity of the send during success, -1 for fail
 *
 * =============================================================================
 */
int32_t  bmet_get_status(transport_HANDLE handle);

/**
 * ============================================================================
 *  @n@b bmet_create
 *
 *  @b  brief
 *  @n  create the bmet instance
 *
 *  @param[in]  bmetConfig
 *      Pointer to the bmet configuration structure
 *
 *  @return
 *      bmet transport handle during success, NULL during fail
 *
 * =============================================================================
 */
transport_HANDLE             bmet_create(bmetChConfig_t  bmetConfig);
/*@}*/  /* defgroup */

/** @defgroup BMET_ETHERNET_INST_INFO BMET Ethernet Trasnport Information
 */
/*@{*/
/**
 * @brief   BMET Instance Size
 */
#define BMET_INST_SIZE        sizeof (bmetInst_t)
/**
 * @brief   Alignment for the BMET instance
 */
#define BMET_INST_ALIGN       128
/*@}*/  /* defgroup */


#ifdef __cplusplus
}
#endif

#endif /* __BMET_TRANSPORT_H__ */


