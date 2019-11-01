#ifndef _PA_FC_H
#define _PA_FC_H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <stdint.h>
#include <stdlib.h>

#include <ti/drv/pa/pa.h>

/* ============================================================= */
/**
 *   @file  pa_fc.h
 *
 *   path  ti/drv/pa/pa_fc.h
 *
 *   @brief  Packet Accelerator (PA) sub-system LLD Flow Cache related API and Data Definitions
 *
 *  ============================================================================
 *  Copyright (c) Texas Instruments Incorporated 2013
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

/**
 *  @page appendix4 Flow Cache 
 *
 *  The PASS Flow Cache operation consits of the following three sub-operations:
 *      - Flow Cache Classification: Classify up to 256 established egress flows based on  
 *                                   the inner IP and L4 parameters. 
 *      - Egress Flow Operation: Perform up to 4-level packet modification such as IP mangling,  
 *                               IPSEC framing and encryption, L2 framing and etc per packet 
 *                               modification records.
 *      - Ingress Packet Forwarding: Route the ingress packets to the Egress processing unit  
 *                                   as one of the classification routing options.
 *
 *  The flow cache operation can be used to help glue the ingress path to the egress path to 
 *  establish a fully automated processing chain without host intervention. It may be only 
 *  applied to egress packets from the host to establish fully-offloaded egress chain operation 
 *  with or without flow cache classification. 
 *
 *  The flow cache classification module resides at PASS Egress0 clsuter and it supports 256
 *  LUT1 entries, each entry consists of any combination of IP and UDP/TCP parameters as defined
 *  at @ref paFcInfo_t. Each flow cache entry is also assocaited with a pre-determined egress flow
 *  where the egress packets will undertake up to 4-level of packet modification defined by
 *  the egress flow packet modification records, which are described at the next section. The API 
 *  @ref Pa_addFc() and @ref Pa_delFcHandle are used to add and delete the Flow Cache entries respectively. 
 *
 *  Each egress flow consists of up to four packet modification stages where the packet can undergo  
 *  transformations in the egress path, before it is sent out from PASS. The four packet modification
 *  stages are 
 *  @li Inner L3/L4 Processing  
 *  @li Outer L3/IPSEC Processing
 *  @li IPSEC AH/ NAT Processing
 *  @li L2 Processing
 *
 *  The detailed operation of each packet modification stage is defined at its corresponding egress flow 
 *  record defintion sections(@ref paEfRecLevel1_t, @ref paEfRecLevel2_t, @ref paEfRecLevel3_t and
 *  @ref paEfRecLevel4_t) The API @ref Pa_configEflowRecords is used to add/delete single or multiple
 *  packet modification records.
 *
 *  The ingress traffic may be routed to the egress path at each of the following four classification stages:
 *  - L2 classification: Only L2 processing is allowed at the egress path to support application 
 *                       such as MAC routing
 *  - Outer IP classification: Support outer L3/IPSEC and L2 processing only without Flow Cache classification
 *                             Support all four egress stages with Flow Cache classification
 *  - Inner IP classification: Support all four egress stages with and without Flow Cache classification 
 *  - L4 classification: Support all four egress stages with and without Flow Cache classification 
 *
 *  The advanced routing information data structure @ref paRouteInfo2_t with flow cache operation
 *  data structure @ref paEfOpInfo_t are used to define flow cache route.
 */

/**
 *  @defgroup paEflowErouteTypes PA Egress Flow Exception Route Types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Exception Route Types
 *
 *  @brief  These values are used to define exception route conditions in egress path.
 *
 *  @details  The Egress Flow exception route defines the global routing information when the exception 
 *            condition such as Flow Cache lookup failure, packet parsing failure, TCP control detection 
 *            and etc. Multiple exception routes can be configured through @ref Pa_configEflowExceptionRoute. 
 *            The PASS will drop all exception route packets by default until the corresponding exception
 *            route is configured.
 */
/*  @{  */
/**
 *
 *   @def  pa_EFLOW_EROUTE_FC_FAIL
 *         packet failed to match in Flow Cache (LUT1) table
 */
#define pa_EFLOW_EROUTE_FC_FAIL         0

/**
 *  @def pa_EFLOW_EROUTE_PARSE_FAIL
 *       packet failed to parse
 */
#define pa_EFLOW_EROUTE_PARSE_FAIL      1

/**
 *  @def pa_EFLOW_EROUTE_IP_FRAG
 *       IP fragmented packet 
 */
#define pa_EFLOW_EROUTE_IP_FRAG         2

/**
 *  @def pa_EFLOW_EROUTE_IPV6_OPT_FAIL
 *       Packet failed due to unsupported IPV6 option header
 */
#define pa_EFLOW_EROUTE_IPV6_OPT_FAIL   3

/**
 *  @def pa_EFLOW_EROUTE_IP_OPTIONS
 *       IP packet with IPv4 options or IPv6 extension headers
 */
#define pa_EFLOW_EROUTE_IP_OPTIONS      4

/**
 *  @def pa_EFLOW_EROUTE_IP_EXPIRE
 *       IP packet with TTL expired or Hop Limitis reached
 */
#define pa_EFLOW_EROUTE_IP_EXPIRE       5

/**
 *  @def pa_EFLOW_EROUTE_TCP_CTRL
 *       TCP Control packets where one or more control bits 
 *       (SYN, FIN, and RST) are set
 */
#define pa_EFLOW_EROUTE_TCP_CTRL        6

/**
 *  @def pa_EFLOW_EROUTE_INVALID_REC
 *       Invalid Egree Flow Records
 */
#define pa_EFLOW_EROUTE_INVALID_REC     7


/**
 *  @def  pa_EFLOW_EROUTE_SYSTEM_FAIL
 *        Sub-system detected internal error
 */
#define pa_EFLOW_EROUTE_SYSTEM_FAIL     10

/**
 *  @def   pa_EFLOW_EROUTE_MAX
 *         The maximum number of global route types
 */
#define pa_EFLOW_EROUTE_MAX             11

/*  @}  */  
/** @} */

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configEflowExceptionRoute configures the routing of packets based on an exception condition such as
 *          no match, parsing failure, TCP control packet etc in the egress direction (PASS Gen2 only)
 *
 *  @details  This function is used to configure the sub-system to route packets that satisfy an exception 
 *            rule or condition (see @ref paEflowErouteTypes) in the egress direction. For example,
 *            - failure to flow cache table match
 *            - parsing error i.e. the sub-system is not able to continue parsing the packets
 *            - TCP contrl packet
 *            - system failure  
 *
 *            From one to @ref pa_EFLOW_EROUTE_MAX routes can be specified through a single call to this
 *            function.  Parameter nRoute is used to specify how many routes are contained in the
 *            routeTypes and eRoutes arrays. A value of 0 nRoutes results in no action by the function.
 *
 *            By default when each exception type is detected the packet is discarded silently. Once the
 *            route is changed through a call to this function it remains in the new state until the
 *            function is called again to explicitly change that route. The only way to revert back
 *            to the default of silent discard is to call this function again.
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the API @ref Pa_forwardResult.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[in]    nRoute      The number of exception routes specified
 *  @param[in]    routeTypes  Array of exception routing types (@ref paEflowErouteTypes)
 *  @param[in]    eRoutes     Array of exception packet routing configuration
 *  @param[out]   cmd         Buffer where the sub-system command is created
 *  @param[in]    cmdSize     The size of the passCmd buffer
 *  @param[in]    reply       Where the response to the PASS command is routed
 *  @param[out]   cmdDest     Value (@ref cmdTxDest)
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 *
 *  @note: This API is not supported at the first generation PASS
 */
paReturn_t Pa_configEflowExceptionRoute (Pa_Handle       iHandle,
                                         int             nRoute,
                                         int            *routeTypes,
                                         paRouteInfo_t  *eRoutes,
                                         paCmd_t         cmd,
                                         uint16_t       *cmdSize,
                                         paCmdReply_t   *reply,
                                         int            *cmdDest);

/**
 *  @defgroup paEflowRecTypes PA Egress Flow Record types 
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Record types
 *
 *  @brief  These values are used to define the modification record types in the egress flow operation 
 *
 */
/*  @{  */
/**
 *
 *   @def  pa_EFLOW_REC_TYPE_LVL1
 *         Level1: Inner L3/L4 Processing
 */
#define pa_EFLOW_REC_TYPE_LVL1         1

/**
 *
 *   @def  pa_EFLOW_REC_TYPE_LVL2
 *         Level2: Outer L3/IPSEC Processing 
 */
#define pa_EFLOW_REC_TYPE_LVL2         2
/**
 *
 *   @def  pa_EFLOW_REC_TYPE_LVL3
 *         Level3: IPSEC AH/IPSEC ESP NAT-T Processing 
 */
#define pa_EFLOW_REC_TYPE_LVL3         3

/**
 *
 *   @def  pa_EFLOW_REC_TYPE_LVL4
 *         Level1: L2 Processing
 */
#define pa_EFLOW_REC_TYPE_LVL4         4

 
/*  @}  */  
/** @} */


/**
 *  @defgroup paEfLvl1RecordValidBits  PA Egress Flow Level One Record Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Level One Record Valid Bit Definitions
 *
 *  Bitmap definition of the validBitMap in @ref paEfRecLevel1_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS
 *        - Level One record control flags is present
 */
#define pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS         0x0001

/**
 *  @def  pa_EF_LVL1_RECORD_VALID_IP_SRC
 *        - Source IP address in packet should be replaced with the one in record
 */
#define pa_EF_LVL1_RECORD_VALID_IP_SRC             0x0002

/**
 *  @def  pa_EF_LVL1_RECORD_VALID_IP_DST
 *        - Destination IP address in packet should be replaced with the one in record
 */
#define pa_EF_LVL1_RECORD_VALID_IP_DST             0x0004

/**
 *  @def  pa_EF_LVL1_RECORD_VALID_FLOW_LABEL
 *        - IPv6 flow label in packet should be replaced with the one in record
 */
#define pa_EF_LVL1_RECORD_VALID_FLOW_LABEL         0x0008

/**
 *  @def  pa_EF_LVL1_RECORD_VALID_TOS_CLASS
 *        - IPv4 type of service or IPv6 traffic class in packet should be replaced with the one in record
 */
#define pa_EF_LVL1_RECORD_VALID_TOS_CLASS          0x0010

/**
 *  @def  pa_EF_LVL1_RECORD_VALID_IP_MTU
 *        Inner IP MTU is specified, perform Inner IP fragmentation is necessary 
 */
#define pa_EF_LVL1_RECORD_VALID_IP_MTU             0x0020 

/**
 *  @def  pa_EF_LVL1_RECORD_VALID_SRC_PORT
 *        TCP/UDP source port in packet should be replaced with the one in record 
 */
#define pa_EF_LVL1_RECORD_VALID_SRC_PORT           0x0040 
/**
 *  @def  pa_EF_LVL1_RECORD_VALID_DST_PORT
 *        TCP/UDP destination port in packet should be replaced with the one in record 
 */
#define pa_EF_LVL1_RECORD_VALID_DST_PORT           0x0080 


/* @} */ /* ingroup */
/** @} */

/**
 *  @defgroup paEfLvl1RecordCtrlFlags  PA Egress Flow Level One Record Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Level One Record Control Flag Definitions
 *  Bitmap definition of the ctrlFlags in @ref paEfRecLevel1_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM
 *        Flag -- 1: Recalculate IP header Checksum 
 */
#define pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM         0x0001 
/**
 *  @def  pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM
 *        Flag -- 1: Recalculate TCP/UDP Checksum
 */
#define pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM         0x0002 
/**
 *  @def  pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC
 *        Flag -- 1: Decrement TTL
 */
#define pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC          0x0004 
/**
 *  @def  pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_TCP_CTRL
 *        Flag -- 1: Detect and forward TCP control packet through egress flow exception route
 */
#define pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_TCP_CTRL     0x0008 
/**
 *  @def  pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_OPTION
 *        Flag -- 1: Detect and forward IP packet with IPv4 options or IPv6 extension headers through egress flow exception route
 */
#define pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_OPTION    0x0010 
/**
 *  @def  pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT
 *        Flag -- 1: Detect and forward fragmented IP packet through egress flow exception route
 */
#define pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT  0x0020 
/**
 *  @def  pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE
 *        Flag -- 1: Detect and forward expired IP packet through egress flow exception route
 */
#define pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE    0x0040 
/**
 *  @def  pa_EF_LVL1_RECORD_CONTROL_REMOVE_OUTER_IP_HDR_TRAIL
 *        Flag -- 1: Remove outer IP header up to the inner IP and the associated trail
 *
 *  @note: The option is used to remove the outer IP and associated trail from the forwarding
 *         packets where only L2 header needs to be replaced.
 */
#define pa_EF_LVL1_RECORD_CONTROL_REMOVE_OUTER_IP_HDR_TRAIL    0x0080 


/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Egress Flow Level One Record data structure
 *
 *  @details  The PA Egress Flow level one record is used to instruct PASS to perform Inner L3/L4 Processing as specified below:
 *            -IP header updates
 *              - Source/Destination addresses 
 *              - TOS/Traffic Class
 *              - Flow Label
 *              - TTL
 *            -IPv4 header checksum calculation
 *            -TCP/UDP header updates
 *              - Source/Destination Port
 *            -TCP/UDP checksum calculation
 *            -Inner IP Fragmentation pre-processing
 *            
 *            paEfRecLevel1_t defines the configuration parameters of the egress flow level one record
 *            Since not all fields are used all the time, validBitMap is used to specify which optional field 
 *            is used for packet modification. 
 */
typedef struct  {
  uint16_t    validBitMap;/**< Valid Bitmap corresponding to each optional field as defined at @ref paEfLvl1RecordValidBits */
  uint16_t    ctrlBitMap; /**< Level one record control flag Bitmap as defined at @ref paEfLvl1RecordCtrlFlags */
  int         ipType;     /**< IP type (Mandatory if srcIp or dest Ip is sepcified) @ref IpValues */
  paIpAddr_t  src;        /**< Source IP address */
  paIpAddr_t  dst;        /**< Destination IP address */
  uint32_t    flow;       /**< IPv6 flow label in 20 lsbs */
  uint8_t     tos;        /**< IP Type of Service (IPv4) / Traffic class (IPv6) */
  uint16_t    mtu;        /**< IP MTU size */
  uint16_t    srcPort;    /**< TCP/UDP Source Port Number */
  uint16_t    dstPort;    /**< TCP/UDP Destinatio Port Number */
} paEfRecLevel1_t;

/**
 *  @defgroup paIpsecProto IPSEC Protocols
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name IPSEC Protocols
 *
 *  Parameter ipsecProto of paEfRecIpsecParams_t should be set to
 *  one of these types.
 */ 
/*@{*/
typedef enum {
  pa_IPSEC_PROTO_ESP = 0,             
  pa_IPSEC_PROTO_AH                    
} paIpsecProto_e;
/*@}*/
/** @} */

/**
 *  @defgroup paEfRecIpsecConfigCtrlBit  PA Egress Flow Record IPSEC Configuration Control Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Record IPSEC Configuration Control Bit Definitions
 *
 *  Bitmap definition of the ctrlBitMap in paEfRecIpsecParams_t. 
 *  
 */ 
/*@{*/
/**
 *  @def  pa_EF_RECORD_IPSEC_USE_LOC_DMA
 *        Control Info -- 0:Use PASS global DMA to forward packets to SA (default)
 *                        1:Use PASS local DMA to forward packets to SA  
 */
#define pa_EF_RECORD_IPSEC_USE_LOC_DMA       0x0001 
/*@}*/
/** @} */


/**
 * @ingroup palld_api_structures
 * @brief Egress Flow IPSEC Configuration Parameters structure
 *
 * Data structure defines the IPSEC specific configuration parameters within the egress flow record
 *
 */
typedef struct {
  uint16_t    ipsecProto; /**< Specify the IPSEC protocol as defined at @ref paIpsecProto */
  uint16_t    ctrlBitMap; /**< Various control information as specified at @ref paEfRecIpsecConfigCtrlBit */                                                                       
  uint8_t     encBlkSize; /**< Specify the encryption block size 
                               1: Stream encryption and no alignment requirement
                               4: AES-CTR or other stream-like encryption with 4-byte 
                                  alignment
                                  block size: block encryption algorithm */
  uint8_t     ivSize;     /**< Specify the size of the initialization vector in bytes */  
  uint8_t     macSize;    /**< Specify the size of the authentication tag in bytes */ 
  uint8_t     flowId;     /**< Specify the 8-bit CPPI Flow ID which instructs how the link-buffer queues are used for forwarding packets */
  uint16_t    queueId;    /**< Specify the 16-bit SA input queue ID */
  uint32_t    spi;        /**< Specify the SPI (Security Parameter Index) */
  uint32_t    saInfo0;    /**< Specify SA-specific software info word 0 */
  uint32_t    saInfo1;    /**< Specify SA-specific software info word 1 */
} paEfRecIpsecParams_t;

/**
 *  @defgroup paEfLvl2RecordValidBits  PA Egress Flow Level two Record Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Level two Record Valid Bit Definitions
 *
 *  Bitmap definition of the validBitMap in @ref paEfRecLevel2_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS
 *        - Level two record control flags is present
 */
#define pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS         0x0001

/**
 *  @def  pa_EF_LVL2_RECORD_VALID_IP_MTU
 *        Outer IP MTU is specified, perform Outer IP fragmentation is necessary 
 */
#define pa_EF_LVL2_RECORD_VALID_IP_MTU             0x0002 

/**
 *  @def  pa_EF_LVL2_RECORD_VALID_IPSEC
 *        - IPSEC parameters are present
 */
#define pa_EF_LVL2_RECORD_VALID_IPSEC              0x0004

/* @} */ /* ingroup */
/** @} */

/**
 *  @defgroup paEfLvl2RecordCtrlFlags  PA EgressFlow Level Two Record Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA EgressFlow Level Two Record Control Flag Definitions
 *  Bitmap definition of the ctrlFlags in @ref paEfRecLevel2_t. 
 */ 
/*@{*/

/**
 *  @def  pa_EF_LVL2_RECORD_CONTROL_SINGLE_IP
 *        Flag -- 1: There is only one IP layer, i.e. there is no inner IP
 *
 *  @note This flag should be set in IPSEC transport mode where only one IP layer is present. 
 */
#define pa_EF_LVL2_RECORD_CONTROL_SINGLE_IP                 0x0001 
/**
 *  @def  pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL
 *        Flag -- 1: Format and insert IPSEC ESP/AH header and trailer into the packets
 *                0: Use the IPSEC ESP/AH header and trailer existing in the packet
 *
 *  @note: The option is only valid if the outer IP header is not inserted or replaced
 *         by this record.
 */
#define pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL    0x0002 
/**
 *  @def  pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC
 *        Flag -- 1: Decrement TTL at the outer IP header
 */
#define pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC              0x0004 
/**
 *  @def  pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_OPTION
 *        Flag -- 2: Detect and forward IP packet with IPv4 options or IPv6 extension headers through egress flow exception route
 */
#define pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_OPTION        0x0008 
/**
 *  @def  pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT
 *        Flag -- 1: Detect and forward fragmented IP packet through egress flow exception route
 */
#define pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT      0x0010 
/**
 *  @def  pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE
 *        Flag -- 1: Detect and forward expired IP packet through egress flow exception route
 */
#define pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE        0x0020 
/*@}*/
/** @} */

/**
 *   @ingroup palld_api_constants
 *   @def  pa_MAX_EF_REC_IP_HDR_LEN
 *         The maximum IP Header Length in bytes within an egress flow level 2 record
 */
#define pa_MAX_EF_REC_IP_HDR_LEN        40 

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Egress Flow Level Two Record data structure
 *
 *  @details  The PA Egress Flow level two record is used to instruct PASS to perform outer L3 and IPSEC Processing as specified below:
 *            - Insert/Update outer IP header
 *              - Update payload length
 *              - Update protocol/nextHeader if IPSEC is enabled
 *              - Update TTL
 *              - IPv4 header checksum calculation
 *            - Insert/Update IPSEC ESP/AH header
 *            - Replace/Insert IPSEC ESP Trailer
 *            - Replace/Insert IPSEC ESP ICV
 *            - Complete Inner IP Fragmentation from previous stage
 *            - Outer IP fragmentation pre-processing 
 *            
 *            paEfRecLevel2_t defines the configuration parameters of the egress flow level two record
 *            Since not all fields are used all the time, validBitMap is used to specify which optional field 
 *            is used for packet modification.
 *
 *  @note: The egress flow level two record should be provided if outer IP is present and/or IPSEC operation is required. 
 */
typedef struct  {
  uint16_t    validBitMap;/**< Valid Bitmap corresponding to each optional field as defined at @ref paEfLvl2RecordValidBits */
  uint16_t    ctrlBitMap; /**< Level two record control flag Bitmap as defined at @ref paEfLvl2RecordCtrlFlags */
  uint16_t    mtu;        /**< Outer IP MTU size */
  uint16_t    ipHdrSize;  /**< IP header size in bytes (0:Retain the outer IP header in the packet */
  uint8_t*    ipHdr;      /**< Pointer to the IP header */     
  paEfRecIpsecParams_t  ipsec;  /**< IPSEC related parameters */
} paEfRecLevel2_t;

/**
 *  @defgroup paEfLvl3RecordValidBits  PA Egress Flow Level Three Record Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Level Three Record Valid Bit Definitions
 *
 *  Bitmap definition of the validBitMap in @ref paEfRecLevel3_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EF_LVL3_RECORD_VALID_CTRL_FLAGS
 *        - Level three record control flags is present
 */
#define pa_EF_LVL3_RECORD_VALID_CTRL_FLAGS         0x0001
/**
 *  @def  pa_EF_LVL3_RECORD_VALID_IP_MTU
 *        IP MTU is specified, perform IP fragmentation adjustment due to the extra header insertion
 *
 *  @note: The MTU sise should be consistent with the one at the corresponding level 2 or level 1
 *         record. 
 */
#define pa_EF_LVL3_RECORD_VALID_IP_MTU             0x0002 
/**
 *  @def  pa_EF_LVL3_RECORD_VALID_IPSEC
 *        - 1: IPSEC AH parameters are present
 *          0: NAT-T parameters are present
 */
#define pa_EF_LVL3_RECORD_VALID_IPSEC              0x0004

/* @} */ /* ingroup */
/** @} */

/**
 *  @defgroup paEfLvl3RecordCtrlFlags  PA EgressFlow Level Three Record Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA EgressFlow Level Three Record Control Flag Definitions
 *  Bitmap definition of the ctrlFlags in @ref paEfRecLevel3_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EF_LVL3_RECORD_CONTROL_REPLACE_HDR
 *        Flag -- 1: Replace IPSEC AH or NAT-T header in the packet
 *                0: Format and insert IPSEC AH and NAT-T header into the packet
 */
#define pa_EF_LVL3_RECORD_CONTROL_REPLACE_HDR               0x0001 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Egress Flow Level Three Record data structure
 *
 *  @details  The PA Egress Flow level three record is used to instruct PASS to perform IPSEC AH or IPSEC ESP NAT-T 
 *            Processing as specified below:
 *            - Update outer IP header
 *              - Update payload length
 *              - Update protocol/nextHeader
 *              - IPv4 header checksum calculation
 *            - Insert/Update IPSEC AH header
 *            - Insert/Update IPSEC NAT-T UDP header
 *            - Outer IP fragmentation pre-processing 
 *            
 *            paEfRecLevel3_t defines the configuration parameters of the egress flow level three record
 *            Since not all fields are used all the time, validBitMap is used to specify which optional field 
 *            is used for packet modification. 
 */
typedef struct  {
  uint16_t    validBitMap;/**< Valid Bitmap corresponding to each optional field as defined at @ref paEfLvl3RecordValidBits */
  uint16_t    ctrlBitMap; /**< Level three record control flag Bitmap as defined at @ref paEfLvl3RecordCtrlFlags */
  uint16_t    mtu;        /**< Outer IP MTU size */
  uint16_t    srcPort;    /**< Source port number of NAT-T UDP header */
  uint16_t    dstPort;    /**< Destination port number of NAT-T UDP header */
  paEfRecIpsecParams_t  ipsec;  /**< IPSEC related parameters */
} paEfRecLevel3_t;

/**
 *  @defgroup paEfLvl4RecordValidBits  PA Egress Flow Level Four Record Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Level Four Record Valid Bit Definitions
 *
 *  Bitmap definition of the validBitMap in @ref paEfRecLevel4_t. 
 */ 
/**
 *  @def  pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS
 *        - Level four record control flags is present
 */
#define pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS              0x0001

/**
 *  @def  pa_EF_LVL4_RECORD_VALID_802_3
 *        - L2 is 802.3 and the length filed offset of the 802.3 header is valid
 */
#define pa_EF_LVL4_RECORD_VALID_802_3                   0x0002

/**
 *  @def  pa_EF_LVL4_RECORD_VALID_PPPoE
 *        - L2 header include PPPoE and the length filed offset of PPPoE header is valid
 */
#define pa_EF_LVL4_RECORD_VALID_PPPoE                   0x0004

/**
 *  @def  pa_EF_LVL4_RECORD_VALID_VLAN1
 *        - Inner VLAN (0x8100) in the packet should be replaced with the one in record
 *          Inner VLAN (0x8100,VLAN1) should be inserted into the L2 header if it is
 *          not in the packet.
 */
#define pa_EF_LVL4_RECORD_VALID_VLAN1                   0x0008

/**
 *  @def  pa_EF_LVL4_RECORD_VALID_VLAN1
 *        - Outer VLAN (0x88a8) in the packet should be replaced with the one in record
 *          Outer VLAN (0x88a8,VLAN2) should be inserted into the L2 header if it is
 *          not in the packet.
 *
 */
#define pa_EF_LVL4_RECORD_VALID_VLAN2                   0x0010

/**
 *  @def  pa_EF_LVL4_RECORD_VALID_MIN_PKTSIZE
 *        - Perform tx padding check to ensure the minimum packet size as specified by minPktSize
 */
#define pa_EF_LVL4_RECORD_VALID_MIN_PKTSIZE             0x0020

/**
 *  @def  pa_REF_LVL4_RECORD_VALID_ROUTE_PRIORITY_TYPE
 *        - Optional parameter priorityType when Host routing is valid
 */
#define pa_REF_LVL4_RECORD_VALID_ROUTE_PRIORITY_TYPE    0x0040

/* @} */ /* ingroup */
/** @} */

/**
 *  @defgroup paEfLvl4RecordCtrlFlags  PA EgressFlow Level Four Record Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA EgressFlow Level Four Record Control Flag Definitions
 *  Bitmap definition of the ctrlFlags in @ref paEfRecLevel4_t. 
 */ 
/*@{*/

/*@}*/
/** @} */

/**
 *   @ingroup palld_api_constants
 *   @def  pa_MAX_EF_REC_L2_HDR_LEN
 *         The maximum Layer 2 Header Length in bytes within an egress flow level four record
 */
#define pa_MAX_EF_REC_L2_HDR_LEN        40 

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Egress Flow Level Four Record data structure
 *
 *  @details  The PA Egress Flow level four record is used to instruct PASS to perform L2 Processing as specified below:
 *            - Insert/Update L2 header
 *              - Update 802.3 payload length
 *              - Update PPPoE payload length
 *              - Insert/update QinQ VLAN
 *              - Insert/update VLAN
 *            - Patch AH Header with the ICV from SA
 *            - Outer IP Fragmentation
 *            
 *            paEfRecLevel4_t defines the configuration parameters of the egress flow level one record
 *            Since not all fields are used all the time, validBitMap is used to specify which optional field 
 *            is used for packet modification. 
 *
 *  @note Egress flow level four record is mandatory for all egress flows.
 */
typedef struct  {
  uint16_t    validBitMap;/**< Valid Bitmap corresponding to each optional field as defined at @ref paEfLvl4RecordValidBits */
  uint16_t    ctrlBitMap; /**< Level four record control flag Bitmap as defined at @ref paEfLvl4RecordCtrlFlags */
  uint16_t    lenOffsetPPPoE; /**< offset to the PPPoE header */
  uint16_t    lenOffset802p3; /**< offset to the length field within 802.3 header */
  uint16_t    vlan1;       /**< inner VLAN (0x8100) */
  uint16_t    vlan2;       /**< outer VLAN (QinQ) */
  uint16_t    l2HdrSize;   /**< L2 header size in bytes */
  uint8_t*    l2Hdr;       /**< Pointer to the L2 header */     
  uint8_t     minPktSize;  /**< Minimum tx packet size: tx padding of zero is required if the length of tx packet is smaller than this value */
  uint8_t     dest;        /**< Specify egress destination (HOST, EMAC and SRIO) */
  uint8_t     flowId;      /**< Specify the 8-bit CPPI Flow ID which instructs how the link-buffer queues are used for forwarding packets */
  uint8_t     pktType_emacCtrl;/**<  For destination SRIO, specify the 5-bit packet type toward SRIO 
                                     For destination HOST, EMAC, specify the EMAC control @ref emcOutputCtrlBits to the network */
  uint32_t    swInfo0;     /**< Placed in SwInfo0 for packets to host; Placed in the PS Info for packets to SRIO*/
  uint32_t    swInfo1;     /**< Placed in the PS Info for packets to SRIO */
  uint16_t    queueId;     /**< Specify the 16-bit egress queue ID */
  uint16_t    priIfType;   /**< For Host route only, specify priority-based and/or interfcae-based routing mode as
                                defined at @ref paRoutePriIntf_e */
} paEfRecLevel4_t;

/**
 *  @defgroup paEfRecordCtrlFlags  PA EgressFlow Record Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Record Control Flag Definitions
 *  Bitmap definition of the ctrlBitMap in @ref paEfRec_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EF_RECORD_CONTROL_ENABLE
 *        Flag -- 0: Enable the egress flow record
 *                1: Disable the egress flow record 
 */
#define pa_EF_RECORD_CONTROL_ENABLE                         0x0001 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_constants
 *  @brief   Define the maximum number of PASS egress flow records 
 *
 */
#define pa_MAX_EF_LVL1_RECORDS      256
#define pa_MAX_EF_LVL2_RECORDS      256
#define pa_MAX_EF_LVL3_RECORDS      256
#define pa_MAX_EF_LVL4_RECORDS      256

/**
 * @ingroup palld_api_structures
 * @brief PA Egress Flow Record data structure
 *
 * Data structure defines the configuration parameters of the egress flow record 
 */
typedef struct {
  uint16_t        index;       /**< The record index of the egress flow table of the type specified here */
  uint16_t        type;        /**< Egress flow record type as defined at @ref paEflowRecTypes */
  uint16_t        ctrlBitMap;  /**< Egrss flow record control flag Bitmap as defined at @ref paEfRecordCtrlFlags */
  union {
    paEfRecLevel1_t    level1; /**< Specify the level one record */  
    paEfRecLevel2_t    level2; /**< Specify the level two record */  
    paEfRecLevel3_t    level3; /**< Specify the level three record */  
    paEfRecLevel4_t    level4; /**< Specify the level four record */  
  } u;                         /**< Specify the configurtaion parameters of the specified record type */
} paEfRec_t;

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configEflowRecords configures the egress flow records  (PASS Gen2 only)
 *
 *  @details  This function is used to configure multiple egress flow records. Each egress flow 
 *            consists of up to 4-level of operations which are defined by the egress flow record 
 *            of each level respectively.
 *            Refer to the egress flow record defintions for detailed actions and configuration 
 *            parameters of each record level.
 *
 *            Each egress flow record is created and refered to based on its record index.  
 *            Once the record is created through a call to this function it remains effective 
 *            until this function is called again to explicitly overwrite its content. It is not 
 *            recommended to update a record when it is still used by one or more packet routes.  
 *           
 *            The recommended Egress Flow Record update procedure is as the followings:
 *            - Step 1: Call @ref Pa_delFcHandle to remove all egress flows which invokes 
 *                      the egress flow records to be updated and wait for the corresponding
 *                      command acknowledge packet.
 *            - Step 2: Remove all IP forwarding flows, which invoke the egress flow records
 *                      to be updated, in the ingress path.
 *            - Step 3: Wait for a short period of time such as 1us to allow the remaining 
 *                      egress packets to pass through the PASS.
 *            - Step 4: Call this API to re-configure the egress flow records.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[in]    nRecords    The number of egress flow records specified
 *  @param[out]   nRecProc    The number of egress flow records processed successfuly
 *  @param[in]    records     Array of egress flow records
 *  @pre                      A driver instance must be created and tables initialized
 *
 *  @note: This API is not supported at the first generation PASS
 */
paReturn_t Pa_configEflowRecords (Pa_Handle       iHandle,
                                  int             nRecords,
                                  int            *nRecProc,    
                                  paEfRec_t      *records
                                  );
                                  
/**
 *  @defgroup paFcInfoValidBit  PA Flow Cache Matching Info Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Flow Cache Matching Info Valid Bit Definitions
 *  Bitmap definition of the validBitfield in paFcInfo_t. 
 *  It allows selective Flow Cache matching parameters
 */ 
/*@{*/
/**
 *  @def  pa_FC_INFO_VALID_SRC_IP
 *        srcIp is present
 */
#define pa_FC_INFO_VALID_SRC_IP            0x0001 
/**
 *  @def  pa_FC_INFO_VALID_SRC_IP_MASK
 *        srcIpMask is present. This flag is valid only if srcIp is present.
 *        If srcIp is present and this flag is clear, it means all IP address bits are valid.
 *  @note: only IP subnet mask is supported.
 */
#define pa_FC_INFO_VALID_SRC_IP_MASK       0x0002 
/**
 *  @def  pa_FC_INFO_VALID_DST_IP
 *        dstIp is present
 */
#define pa_FC_INFO_VALID_DST_IP            0x0004 
/**
 *  @def  pa_FC_INFO_VALID_DST_IP_MASK
 *        dstIpMask is present. This flag is valid only if dstIp is present.
 *        If dstIp is present and this flag is clear, it means all IP address bits are valid.
 *  @note: only IP subnet mask is supported.
 */
#define pa_FC_INFO_VALID_DST_IP_MASK       0x0008 
/**
 *  @def  pa_FC_INFO_VALID_CTRL_FLAG
 *        ctrlFlag and ctrlFlagMask are present
 */
#define pa_FC_INFO_VALID_CTRL_FLAG         0x0010
/**
 *  @def  pa_FC_INFO_VALID_PROTO
 *        proto is present
 */
#define pa_FC_INFO_VALID_PROTO             0x0020 
/**
 *  @def  pa_FC_INFO_VALID_DSCP
 *        dscp is present
 */
#define pa_FC_INFO_VALID_DSCP              0x0040 
/**
 *  @def  pa_FC_INFO_VALID_SRC_PORT
 *        srcPortBegin and srcPortEnd  are present 
 */
#define pa_FC_INFO_VALID_SRC_PORT          0x0100 
/**
 *  @def  pa_FC_INFO_VALID_DST_PORT
 *        dstPortBegin and dstPortEnd are present 
 */
#define pa_FC_INFO_VALID_DST_PORT          0x0200 

/*@}*/
/** @} */

/**
 *  @defgroup paFcInfoCtrlFlags  PA Flow Cache Info Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Flow Cache Info Control Flag Definitions
 *  Bitmap definition of the ctrlFlags and ctrlFlagsMask in @ref paFcInfo_t. 
 */ 
/*@{*/
/**
 *  @def  pa_FC_INFO_CONTROL_FLAG_FRAG
 *        Flag -- 1: IP fragments 
 */
#define pa_FC_INFO_CONTROL_FLAG_FRAG           0x0001 
/**
 *  @def  pa_FC_INFO_CONTROL_FLAG_CONTAIN_L4
 *        Flag -- 1: Packet or fragment which conatins L4 header 
 */
#define pa_FC_INFO_CONTROL_FLAG_CONTAIN_L4     0x0002 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Flow cache lookup information
 *
 *  @details  paFcInfo_t is used to specifiy the Flow cache matching parameters.
 */
typedef struct  {
  uint16_t    validBitMap;  /**< Specify valid parameters as defined at @ref paFcInfoValidBit */
  uint16_t    ctrlFlag;     /**< Specify Flow Cache control flags as defined at @ref paFcInfoCtrlFlags */
  uint16_t    ctrlFlagMask; /**< Flow Cache control flag valid masks */
  uint16_t    ipType;       /**< @ref IpValues */
  paIpAddr_t  srcIp;        /**< Source IP address */
  paIpAddr_t  srcIpMask;    /**< Source IP subnet mask*/
  paIpAddr_t  dstIp;        /**< Destination IP address */
  paIpAddr_t  dstIpMask;    /**< Destination IP subnet mask */
  uint8_t     proto;        /**< IP Protocol (IPv4) / Next Header (IPv6) */
  uint8_t     dscp;         /**< DSCP value */
  uint16_t    srcPortBegin; /**< Minimum Source Port Number */
  uint16_t    srcPortEnd;   /**< Maximum Source Port Number */
  uint16_t    dstPortBegin; /**< Minimum Destinatio Port Number */
  uint16_t    dstPortEnd;   /**< Maximum Destinatio Port Number */
} paFcInfo_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Flow Cache Entry Statistics Structure
 *
 * @details This structures define the PA Flow Cache per-entry statistics provided 
 *          with API function @ref Pa_queryFcStats ().
 */

typedef struct paFcStats_s  {
  uint32_t   nMatchPackets;     /**< Number of packets which matchs the ACL rule */
} paFcStats_t;

/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_addFc adds a Flow Cache entry to the Flow Cache table  (PASS Gen2 only)
 *
 *   @details  This function is used to add or replace an entry in the Flow Cache table.
 *             A new entry is added if the Flow Cache configuration does not match the value of any existing entry
 *             in the module handle table. If the Flow Cache configuration is same of an existing entry then the 
 *             egress flow operation information for the existing entry is changed to the values provided with the 
 *             input parameter efOpInfo.
 *
 *             The caller can also force the existing entry to be replaced by the new one by passing the handle of
 *             the existing entry through retHandle. 
 *
 *             On return the command buffer (cmd) contains a formatted command for the sub-system. The
 *             destination for the command is provided in cmdDest. The module user must send the formatted
 *             command to the sub-system. The sub-system will generate a reply and this reply must be
 *             sent back to this module through the API @ref Pa_forwardResult.
 *
 *             The flow cache entry should be deleted by invoking API @ref Pa_delFcHandle.
 *             @note The existing entry can not be replaced when it is still at pending response state. The return code
 *                   pa_INVALID_DUP_ENTRY or pa_PENDING_FC_ENTRY will be used in this case. 
 *
 *   @param[in]    iHandle     The driver instance handle
 *   @param[in]    index       Specify the index of the LUT1 entry (0-63). Set to pa_LUT1_INDEX_NOT_SPECIFIED if not specified
 *   @param[in]    efOpInfo    Specify egress flow operation per match as @ref paEfOpInfo_t
 *   @param[in]    fcInfo      Value @ref paFcInfo_t
 *   @param[out]   retHandle   Pointer to the returned FC handle
 *   @param[out]   cmd         Buffer where the PASS command is created
 *   @param[in]    cmdSize     The size of the cmd buffer
 *   @param[in]    reply       Where the response to the PASS command is routed
 *   @param[out]   cmdDest     Value (@ref cmdTxDest)
 *   @retval                   Value (@ref ReturnValues)
 *   @pre                      A driver instance must be created and tables initialized
 *
 *  @note: This API is not supported at the first generation PASS
 */
paReturn_t  Pa_addFc   (Pa_Handle         iHandle,
                        int               index,
                        paEfOpInfo_t      *efOpInfo,
                        paFcInfo_t        *fcInfo,
                        paHandleFc_t      *retHandle,
                        paCmd_t           cmd,
                        uint16_t          *cmdSize,
                        paCmdReply_t      *reply,
                        int               *cmdDest );
                        
/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_delFcHandle deletes a Flow Cache handle (PASS Gen2 only)
 *
 *   @details  This function is used to remove an entry from the LUT1-FC lookup table
 *
 *   @param[in]     iHandle     The driver instance handle
 *   @param[in]     handle      Pointer to the FC handle to delete
 *   @param[out]    cmd         Where the created command is placed
 *   @param[in]     cmdSize     The size of the cmd buffer
 *   @param[in]     reply       Where the sub-system sends the command reply
 *   @param[out]    cmdDest     Value (@ref cmdTxDest)
 *   @retval                    Value (@ref ReturnValues)
 *   @pre                       A driver instance must be created and tables initialized
 *
 *  @note: This API is not supported at the first generation PASS
 */
paReturn_t Pa_delFcHandle (Pa_Handle       iHandle,
                           paHandleFc_t    *handle, 
                           paCmd_t         cmd,
                           uint16_t        *cmdSize,
                           paCmdReply_t    *reply,
                           int             *cmdDest );
                           
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_queryFcStats queries Flow Cache per-entry statistics (PASS Gen2 only)
 *
 *  @details  This function is used to query the Flow Cache per-entry statistics. 
 *            The statistics can be optionally cleared after reading through the doClear parameter.
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    fcHandle   The FC handle
 *  @param[in]    doClear    If TRUE then stats are cleared after being read
 *  @param[out]   pFcStats   Pointer to the FcStats buffer
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 *
 *  @note: This API is not supported at the first generation PASS
 */
paReturn_t Pa_queryFcStats (Pa_Handle     iHandle,
                            paHandleFc_t  fcHandle,
                            uint16_t      doClear, 
                            paFcStats_t   *pFcStats);
                           
                        

#ifdef __cplusplus
}
#endif
  

#endif  /* _PA_FC_H */

 
