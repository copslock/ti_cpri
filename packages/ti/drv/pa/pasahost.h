#ifndef _PASAHOST_H
#define _PASAHOST_H
/**
 *   @file  pasahost.h
 *
 *   @brief   
 *      This file defines constants, data structures and macros used
 *      among the PA LLD, SA LLD and the host.  
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009-2013 Texas Instruments, Inc.
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

#ifdef __cplusplus
extern "C" {
#endif
/** @defgroup pasaho_if_module PA/SA/Host Interface
 *  @{
 */
/** @} */

/** @defgroup pasaho_if_macros PA/SA/Host Macros
 *  @ingroup pasaho_if_module
 */

/** @defgroup pasaho_if_structures PA/SA/Host Data Structures
 *  @ingroup pasaho_if_module
 */

/** @defgroup pasaho_if_constants PA/SA/Host Constants (enum's and define's)
 *  @ingroup pasaho_if_module
 */


/**
 *  @defgroup pasahoCommands PA/SA/Host Commands
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PA/SA/Host Commands
 *
 *  Define PASS Firmware Commands
 *  These values are placed in the 3 msbits of the protocol specific information
 */
/*@{*/
/**
 *  @def  PASAHO_CONFIGURE
 *        PA/SA Configuration command
 */

#define PASAHO_CONFIGURE			4

/* PA commands for receive packet PDSPs */
/**
 *  @def  PASAHO_PARX_PARSECMD
 *        Instruct PDSP to parse the receive packet
 */

#define PASAHO_PARX_PARSECMD		0

/**
 *  @def  PASAHO_PARX_MULTI_ROUTE
 *        Instruct PDSP to perform multiple routing
 */

#define PASAHO_PARX_MULTI_ROUTE 	5

/* PA commands for modify packet PDSPs */
/**
 *  @def  PASAHO_PAMOD_CMPT_CHKSUM
 *        Instruct PDSP to compute checksum
 */

#define PASAHO_PAMOD_CMPT_CHKSUM	0

/**
 *  @def  PASAHO_PAMOD_CMPT_CRC
 *        Instruct PDSP to compute CRC
 */

#define PASAHO_PAMOD_CMPT_CRC		1

/**
 *  @def  PASAHO_PAMOD_PATCH
 *        Instruct PDSP to perform blind patch
 */

#define PASAHO_PAMOD_PATCH			2

/**
 *  @def  PASAHO_PAMOD_NROUTE
 *        Provide PDSP with the next routing information
 */

#define PASAHO_PAMOD_NROUTE			3


/**
 *  @def  PASAHO_PAMOD_EF_OP
 *        Instruct PDSP to perform egress flow operation
 */

#define PASAHO_PAMOD_EF_OP      	5

/**
 *  @def  PASAHO_PAMOD_REPORT_TIMESTAMP
 *        Instruct PDSP to report the system timestamp at the timestamp field of the packet descriptor 
 *        when the tx packet is delivered out of the PASS 
 */

#define PASAHO_PAMOD_REPORT_TIMESTAMP  6   


/**
 *  @def  PASAHO_PAMOD_GROUP_7
 *        Define this group command so that several command can share the same command code. They will be distinguished
 *        by its unique 5-bit sub-command code as defined at
 */

#define PASAHO_PAMOD_GROUP_7        7   


/**
 *  @def  PASAHO_PAMOD_DUMMY
 *        No action is required. It is for SA alignment only
 */

#define PASAHO_PAMOD_DUMMY			PASAHO_PAMOD_GROUP_7

/**
 *  @def  PASAHO_PAMOD_IP_FRAGMENT
 *        Instruct PDSP to perform IPv4 fragmentation. The transmit IP packets will be divided into smaller 
 *        IP fragments with the updated IPv4 header and checksum based on the specified MTU size and forwarded 
 *        to the destination specified by the next route command. It is up to the module user to format the
 *        correct IPv4 header. The IP fragmentation command will be ignored if any error is detected.
 */
#define PASAHO_PAMOD_IP_FRAGMENT    PASAHO_PAMOD_GROUP_7  

/**
 *  @def  PASAHO_PAMOD_PATCH_MSG_LEN
 *        Instruct PDSP to perform message length patching after IPv4 fragmentation operation. This command is
 *        valid only if it is in conjunction with the PASAHO_PAMOD_IP_FRAGMENT command.
 */
#define PASAHO_PAMOD_PATCH_MSG_LEN  PASAHO_PAMOD_GROUP_7  

/**
 *  @def  PASAHO_PAMOD_EMAC_CRC_VERIFY
 *        Instruct PDSP to perform Ethernet CRC verification for egress traffic. This command is provided as a workarond 
 *        for the following GbE errata at some keystone devices. (Gen1 support only)
 *
 *        The GbE switch may drop packets in TX path when:
 *        - full gigabit speeds are sustained and
 *        - the packet size is not a 32 bit multiple (1499, 1498, 1497, 1495, etc.) and
 *        - Ethernet CRC is included in the last 4 bytes of the packet sent to the switch
 *
 *  @note The Ethernet CRC Verify command can not be combined with any other tx commands, all other commands will be ignored
 *        by PASS when this command is processed.
 */
 
#define PASAHO_PAMOD_EMAC_CRC_VERIFY  PASAHO_PAMOD_GROUP_7   
  
/**
 *  @def  PASAHO_PAMOD_PATCH_MSG_TIME
 *        Instruct PDSP to perform message time insert for packets such as Ethernet OAM. (Gen2 support only)
 */
#define PASAHO_PAMOD_PATCH_MSG_TIME  PASAHO_PAMOD_GROUP_7 

/**
 *  @def  PASAHO_PAMOD_PATCH_MSG_COUNT
 *        Instruct PDSP to perform message count insert for packets such as Ethernet OAM. 
 */
#define PASAHO_PAMOD_PATCH_MSG_COUNT  PASAHO_PAMOD_GROUP_7 

/* SA commands */
/**
 *  @def  PASAHO_SA_LONG_INFO
 *        Provide SA with the packet parsing information in the long form
 */

#define PASAHO_SA_LONG_INFO         0

/**
 *  @def  PASAHO_SA_SHORT_INFO
 *        Provide SA with the packet parsing information in the short form
 */

#define PASAHO_SA_SHORT_INFO        1


/**
 *  @def  PASAHO_SA_AIR_INFO
 *        Provide SA with the packet parsing information for the air ciphering 
 *        operation
 */

#define PASAHO_SA_AIR_INFO          2


/*@}*/
/** @} */

/** @name PASAHO Common Macros
 *  
 */
/*@{*/

/**
 *  @ingroup pasaho_if_macros
 *  @brief  PASAHO_READ_BITFIELD is used to read the specific bit fields
 *
 *  @details  It is one of the main macros for accessing configuration bit fields
 *            Input parameter a contains bit field
 *            b is bit offset withing bit field
 *            c is number of bits used by that parameter
 */

#define PASAHO_READ_BITFIELD(a,b,c)    (((a)>>(b)) & ((1UL<<(c))-1))

/**
 *  @ingroup ingroup pasaho_if_macros
 *  @brief  PASAHO_SET_BITFIELD is used to set the specific bit fields
 *
 *  @details  It is one of the main macros for accessing configuration bit fields
 *            Input parameter a contains bit field
 *            b is bit offset withing bit field
 *            c is number of bits used by that parameter
 *            x is new value of parameter that is packed in this bit field
 *
 *  @note    It enforces strict setting to prevent overflow into other bits, would
 *           cost program space for additional protection. 
 */

#define PASAHO_SET_BITFIELD(a,x,b,c)   (a) &= ~(((1UL<<(c))-1)<<(b)), \
                                       (a) |= (((x) & ((1UL<<(c))-1))<<(b))

/**
 *  @ingroup ingroup pasaho_if_macros
 *  @brief  PASAHO_SET_CMDID is used to set the command ID
 */
#define PASAHO_SET_CMDID(x,v)  PASAHO_SET_BITFIELD((x)->word0, (v), 29,3)

/**
 *  @ingroup ingroup pasaho_if_macros
 *  @brief  PASAHO_PACFG_CMD is used to set the PA configuration command only
 */
#define PASAHO_PACFG_CMD        (((uint32_t)PASAHO_CONFIGURE << 5) << 24)

/*@}*/ /* @name PASAHO Common Macros */


/**
 *  @defgroup pasahoHeaderTypes PASS Header Types
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PASS Header Types
 *  Definition of protocol header types used at the PASS PDSP Firmwase. In the 
 *  long info field these values specify what the next header type will be
 *  at the next parse offset 
 */ 
/*@{*/

typedef enum {

  PASAHO_HDR_MAC        = 0,        /**< MAC */
  PASAHO_HDR_VLAN,                  /**< VLAN */
  PASAHO_HDR_MPLS,                  /**< MPLS */
  PASAHO_HDR_IPv4,                  /**< IPv4 */
  PASAHO_HDR_IPv6,                  /**< IPv6 */
  PASAHO_HDR_IPv6_EXT_HOP,          /**< IPv6 hop by hop extenstion header */
  PASAHO_HDR_IPv6_EXT_ROUTE,        /**< IPv6 routing extenstion header */
  PASAHO_HDR_IPv6_EXT_FRAG,         /**< IPv6 fragmentation extention header */
  PASAHO_HDR_IPv6_EXT_DEST,         /**< IPv6 destination options header */
  PASAHO_HDR_GRE,                   /**< Generic Routing Encapsulation header */
  PASAHO_HDR_ESP,                   /**< Encapsulating Security Payload header */
  PASAHO_HDR_ESP_DECODED,           /**< Decoded Encapsulating Security Payload header */
  PASAHO_HDR_AUTH,                  /**< Authentication header */
  PASAHO_HDR_CUSTOM_C1,             /**< Custom classify 1 header */
  PASAHO_HDR_PPPoE,                 /**< PPPoE Header */
  PASAHO_HDR_SCTP,                  /**< SCTP Header */
  PASAHO_HDR_UNKNOWN,               /**< Next header type is unknown */
  PASAHO_HDR_UDP,                   /**< User Datagram Protocol header */
  PASAHO_HDR_UDP_LITE,              /**< Lightweight User Datagram Protocol header */
  PASAHO_HDR_TCP,                   /**< Transmission Control Protocol header */
  PASAHO_HDR_GTPU,                  /**< GTPU header */
  PASAHO_HDR_ESP_DECODED_C2,        /**< Decoded Encapsulating Security Payload header at Classifyer2 */
  PASAHO_HDR_CUSTOM_C2              /**< Custom classify 2 header */
 
} pasaho_HeaderType_e;
/*@}*/
/** @} */

/**
 *  @defgroup pasahoSubCmdCode PASS Sub-Command Code
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PASS Sub-Command Code
 *  Definition of the 5-bit sub-command codes which is used to specify the group 7 commands. 
 */ 
/*@{*/

typedef enum {

  PASAHO_SUB_CMD_DUMMY           = 0,   /**< Dummy */
  PASAHO_SUB_CMD_IP_FRAG         = 1,   /**< IPv4 fragmentation */
  PASAHO_SUB_CMD_PATCH_MSG_LEN   = 2,   /**< Message length Patching */ 
  PASAHO_SUB_CMD_INS_TIME        = 3,   /**< Ethernet EOAM insert time at specified offset  (Gen2 only) */
  PASAHO_SUB_CMD_INS_COUNT       = 4,   /**< Ethernet EOAM insert count at specified offset (Gen2 only) */
  PASAHO_SUB_CMD_EMAC_CRC_VERIFY = 3    /**< Ethernet CRC Verification (Gen1 only)*/ 
} pasaho_SubCmdCode_e;
/*@}*/
/** @} */

/**
 *  @defgroup pasahoPktType PASS Packet Type
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PASS Packet Type
 *  Definition of the MAC or IP packet types. 
 */ 
/*@{*/

typedef enum {

  PASAHO_PKT_TYPE_UNICAST        = 0,   /**< Unicast MAC/IP  */
  PASAHO_PKT_TYPE_BROADCAST,            /**< Broadcast MAC/IP */
  PASAHO_PKT_TYPE_MULTICAST             /**< Multicast MAC/IP */ 
} pasaho_pktType_e;
/*@}*/
/** @} */



/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoCmdInfo_t defines the general short command information
 *
 */

typedef struct pasahoCmdInfo_s {
    uint32_t  word0;    /**< Control block word 0 */
} pasahoCmdInfo_t;

/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoLongInfo_t defines the packet parsing information in the long format. 
 *          The information is structured as an array of 32 bit values. These values
 *          are broken down through macros. This allows the representation to be
 *          endian independent to the hardware which operates only on 32 bit values.
 *
 *  @details  
 */

typedef struct pasahoLongInfo_s  {

  uint32_t   word0;  /**< Control block word 0 */
  uint32_t   word1;  /**< Control block word 1 */
  uint32_t   word2;  /**< Control block word 2 */
  uint32_t   word3;  /**< Control block word 3 */
  uint32_t   word4;  /**< Control block word 4 */
  uint32_t   word5;  /**< Control block word 5 */
  uint32_t   word6;  /**< Control block word 6 */
  uint32_t   word7;  /**< Control block word 7 */
  uint32_t   word8;  /**< Control block word 8 */
  uint32_t   word9;  /**< Control block word 9 */
  uint32_t   word10; /**< Control block word 10 (optional) */
  uint32_t   word11; /**< Control block word 11 (optional) */
  uint32_t   word12; /**< Control block word 12 (optional) */
  uint32_t   word13; /**< Control block word 13 (optional) */  
} pasahoLongInfo_t;

/** 
 *  @defgroup PASAHO_long_info_command_gen1_macros  PASAHO Long Info Command Macros For First Generation PASS
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info Command Macros For First Generation PASS
 *  Macros used by the PASAHO Long Info Command
 */
/*@{*/

#define PASAHO_LINFO_READ_CMDID_GEN1(x)          PASAHO_READ_BITFIELD((x)->word0,29,3)    /**< Extract the command ID defined at @ref pasahoCommands (PASS Gen1)*/
#define PASAHO_LINFO_READ_RECLEN_GEN1(x)         PASAHO_READ_BITFIELD((x)->word0,24,5)    /**< Extract the block length (PASS Gen1)*/
#define PASAHO_LINFO_READ_START_OFFSET_GEN1(x)   PASAHO_READ_BITFIELD((x)->word0,0,16)    /**< Extract the next parse start offset (PASS Gen1)*/

#define PASAHO_LINFO_IS_MAC_BROADCAST_GEN1(x)    PASAHO_READ_BITFIELD((x)->word0,16,1)    /**< Indicate whether it is a broadcast MAC packet (PASS Gen1)*/
#define PASAHO_LINFO_IS_MAC_MULTICAST_GEN1(x)    PASAHO_READ_BITFIELD((x)->word0,17,1)    /**< Indicate whether it is a multicast MAC packet (PASS Gen1)*/
#define PASAHO_LINFO_READ_MAC_PKTTYPE_GEN1(x)    PASAHO_READ_BITFIELD((x)->word0,16,2)    /**< Extract the MAC packet type (PASS Gen1)*/

#define PASAHO_LINFO_IS_IP_BROADCAST_GEN1(x)     PASAHO_READ_BITFIELD((x)->word0,18,1)    /**< Indicate whether it is a broadcast IP packet (PASS Gen1)*/
#define PASAHO_LINFO_IS_IP_MULTICAST_GEN1(x)     PASAHO_READ_BITFIELD((x)->word0,19,1)    /**< Indicate whether it is a multicast IP packet (PASS Gen1)*/
#define PASAHO_LINFO_READ_IP_PKTTYPE_GEN1(x)     PASAHO_READ_BITFIELD((x)->word0,18,2)    /**< Extract the IP packet type (PASS Gen1)*/

#define PASAHO_LINFO_READ_END_OFFSET_GEN1(x)     PASAHO_READ_BITFIELD((x)->word1,16,16)   /**< Extract the end of packet parse offset (PASS Gen1)*/
#define PASAHO_LINFO_READ_EIDX_GEN1(x)           PASAHO_READ_BITFIELD((x)->word1,11,5)    /**< Extract the error index (PASS Gen1)*/
#define PASAHO_LINFO_READ_PMATCH_GEN1(x)         PASAHO_READ_BITFIELD((x)->word1,10,1)    /**< Extract the previous match flag (PASS Gen1)*/
#define PASAHO_LINFO_READ_L1_PDSP_ID_GEN1(x)     PASAHO_READ_BITFIELD((x)->word1,6,3)     /**< Extract the first parse module ID (PASS Gen1)*/
#define PASAHO_LINFO_READ_L1_IDX_GEN1(x)         PASAHO_READ_BITFIELD((x)->word1,0,6)     /**< Extract the first parse module match index (PASS Gen1)*/

#define PASAHO_LINFO_READ_L3_OFFSET_GEN1(x)      PASAHO_READ_BITFIELD((x)->word2,24,8)    /**< Extract the offset to the level 3 header (PASS Gen1)*/
#define PASAHO_LINFO_READ_L4_OFFSET_GEN1(x)      PASAHO_READ_BITFIELD((x)->word2,16,8)    /**< Extract the offset to the level 4 header (PASS Gen1)*/
#define PASAHO_LINFO_READ_L5_OFFSET_GEN1(x)      PASAHO_READ_BITFIELD((x)->word2,8,8)     /**< Extract the offset to the level 5 header (PASS Gen1)*/
#define PASAHO_LINFO_READ_ESP_AH_OFFSET_GEN1(x)  PASAHO_READ_BITFIELD((x)->word2,0,8)     /**< Extract the offset to the security header (PASS Gen1)*/

#define PASAHO_LINFO_READ_HDR_BITMASK_GEN1(x)    PASAHO_READ_BITFIELD((x)->word3,21,11)   /**< Extract the bitmask of parsed header types (PASS Gen1)*/
#define PASAHO_LINFO_READ_HDR_BITMASK2_GEN1(x)   PASAHO_READ_BITFIELD((x)->word3, 4,4)    /**< Extract the bitmask2 of parsed header types (PASS Gen1)*/
#define PASAHO_LINFO_READ_NXT_HDR_TYPE_GEN1(x)   PASAHO_READ_BITFIELD((x)->word3,16,5)    /**< Extract the next header to parse type (PASS Gen1)*/
#define PASAHO_LINFO_READ_VLAN_COUNT_GEN1(x)     PASAHO_READ_BITFIELD((x)->word3,14,2)    /**< Extract the number of VLAN tags found (PASS Gen1)*/
#define PASAHO_LINFO_READ_IP_COUNT_GEN1(x)       PASAHO_READ_BITFIELD((x)->word3,8,3)     /**< Extract the number of IP headers found (PASS Gen1)*/
#define PASAHO_LINFO_READ_GRE_COUNT_GEN1(x)      PASAHO_READ_BITFIELD((x)->word3,11,3)    /**< Extract the number of GRE headers found (PASS Gen1)*/
#define PASAHO_LINFO_READ_FLAG_FRAG_GEN1(x)      PASAHO_READ_BITFIELD((x)->word3,3,1)     /**< Extract the fragmentation found flag (PASS Gen1)*/
#define PASAHO_LINFO_READ_INPORT_GEN1(x)         PASAHO_READ_BITFIELD((x)->word3,0,3)     /**< Extract the (1-based) input EMAC port number  
                                                                                               0: Indicates that the packet does not enter PASS 
                                                                                               through CPSW (PASS Gen1)*/
#define PASAHO_LINFO_READ_INNER_IP_OFFSET_GEN1(x) PASAHO_READ_BITFIELD((x)->word4,16,8)   /**< Extract the offset to the most inner IP header (PASS Gen1)*/
#define PASAHO_LINFO_READ_TSTAMP_MSB_GEN1(x)     (x)->word6                               /**< Extract the most significant 32-bit of the 64-bit timestamp */
#define PASAHO_LINFO_READ_TSTAMP_MSB2_GEN1(x)    (x)->word8                               /**< Extract the most significant 32-bit of the 64-bit timestamp 
                                                                                          where the paylaod data has been copied to word6/7 within  
                                                                                          pasahoLongInfo_t by using PASS command pa_CMD_COPY_DATA_TO_PSINFO */

/*@}*/ /* PASAHO_long_info_command_gen1_macros */
/** @}*/ /* @name PASAHO Long Info Command Macros For First Generation PASS */

/** 
 *  @defgroup PASAHO_long_info_command_gen2_macros  PASAHO Long Info Command Macros For Second Generation PASS
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info Command Macros For Second Generation PASS
 *  Macros used by the PASAHO Long Info Command
 */
/*@{*/

#define PASAHO_LINFO_READ_CMDID_GEN2(x)          PASAHO_READ_BITFIELD((x)->word0,29,3)    /**< Extract the command ID defined at @ref pasahoCommands (PASS Gen2)*/
#define PASAHO_LINFO_READ_RECLEN_GEN2(x)         PASAHO_READ_BITFIELD((x)->word0,24,5)    /**< Extract the block length (PASS Gen2)*/
#define PASAHO_LINFO_READ_START_OFFSET_GEN2(x)   PASAHO_READ_BITFIELD((x)->word0,0,8)     /**< Extract the next parse start offset (PASS Gen2)*/

#define PASAHO_LINFO_IS_MAC_BROADCAST_GEN2(x)    PASAHO_READ_BITFIELD((x)->word0,16,1)    /**< Indicate whether it is a broadcast MAC packet (PASS Gen2)*/
#define PASAHO_LINFO_IS_MAC_MULTICAST_GEN2(x)    PASAHO_READ_BITFIELD((x)->word0,17,1)    /**< Indicate whether it is a multicast MAC packet (PASS Gen2)*/
#define PASAHO_LINFO_READ_MAC_PKTTYPE_GEN2(x)    PASAHO_READ_BITFIELD((x)->word0,16,2)    /**< Extract the MAC packet type (PASS Gen2)*/

#define PASAHO_LINFO_IS_IP_BROADCAST_GEN2(x)     PASAHO_READ_BITFIELD((x)->word0,16,1)    /**< Indicate whether it is a broadcast IP packet (PASS Gen2)*/
#define PASAHO_LINFO_IS_IP_MULTICAST_GEN2(x)     PASAHO_READ_BITFIELD((x)->word0,17,1)    /**< Indicate whether it is a multicast IP packet (PASS Gen2)*/
#define PASAHO_LINFO_READ_IP_PKTTYPE_GEN2(x)     PASAHO_READ_BITFIELD((x)->word0,16,2)    /**< Extract the IP packet type (PASS Gen2)*/

#define PASAHO_LINFO_READ_PMATCH_GEN2(x)         PASAHO_READ_BITFIELD((x)->word0,23,1)    /**< Extract the previous match flag (PASS Gen2)*/
#define PASAHO_LINFO_READ_FLAG_FRAG_GEN2(x)      PASAHO_READ_BITFIELD((x)->word0,19,1)    /**< Extract the fragmentation found flag (PASS Gen2)*/

#define PASAHO_LINFO_READ_END_OFFSET_GEN2(x)     PASAHO_READ_BITFIELD((x)->word1,16,16)   /**< Extract the end of packet parse offset (PASS Gen2)*/
#define PASAHO_LINFO_READ_EIDX_GEN2(x)           PASAHO_READ_BITFIELD((x)->word1,10,6)    /**< Extract the exception index (PASS Gen2)*/
#define PASAHO_LINFO_READ_NXT_HDR_TYPE_GEN2(x)   PASAHO_READ_BITFIELD((x)->word1,0,6)     /**< Extract the next header to parse type (PASS Gen2)*/
#define PASAHO_LINFO_READ_INPORT_GEN2(x)         PASAHO_READ_BITFIELD((x)->word1,6,4)     /**< Extract the (1-based) input EMAC port number  
                                                                                               through CPSW (PASS Gen2)*/

#define PASAHO_LINFO_READ_L3_OFFSET_GEN2(x)      PASAHO_READ_BITFIELD((x)->word2,24,8)    /**< Extract the offset to the level 3 header (PASS Gen2)*/
#define PASAHO_LINFO_READ_L4_OFFSET_GEN2(x)      PASAHO_READ_BITFIELD((x)->word2,16,8)    /**< Extract the offset to the level 4 header (PASS Gen2)*/
#define PASAHO_LINFO_READ_L5_OFFSET_GEN2(x)      PASAHO_READ_BITFIELD((x)->word2,8,8)     /**< Extract the offset to the level 5 header (PASS Gen2)*/
#define PASAHO_LINFO_READ_ESP_AH_OFFSET_GEN2(x)  PASAHO_READ_BITFIELD((x)->word2,0,8)     /**< Extract the offset to the security header (PASS Gen2)*/

#define PASAHO_LINFO_READ_L1_PDSP_ID_GEN2(x)     PASAHO_READ_BITFIELD((x)->word3,26,6)    /**< Extract the first parse module ID (PASS Gen2)*/
#define PASAHO_LINFO_READ_L1_IDX_GEN2(x)         PASAHO_READ_BITFIELD((x)->word3,16,10)   /**< Extract the first parse module match index (PASS Gen2)*/
#define PASAHO_LINFO_READ_HDR_BITMASK_GEN2(x)    PASAHO_READ_BITFIELD((x)->word3,0,16)    /**< Extract the bitmask of parsed header types (PASS Gen2)*/
#define PASAHO_LINFO_READ_HDR_BITMASK2_GEN2(x)   0                                        /**< Extract the bitmask2 of parsed header types (PASS Gen2)*/
#define PASAHO_LINFO_READ_VLAN_COUNT_GEN2(x)     PASAHO_READ_BITFIELD((x)->word4,6,2)     /**< Extract the number of VLAN tags found (PASS Gen2)*/
#define PASAHO_LINFO_READ_IP_COUNT_GEN2(x)       PASAHO_READ_BITFIELD((x)->word4,0,3)     /**< Extract the number of IP headers found (PASS Gen2)*/
#define PASAHO_LINFO_READ_GRE_COUNT_GEN2(x)      PASAHO_READ_BITFIELD((x)->word4,3,3)     /**< Extract the number of GRE headers found (PASS Gen2)*/

#define PASAHO_LINFO_READ_INNER_IP_OFFSET_GEN2(x) PASAHO_READ_BITFIELD((x)->word5,24,8)   /**< Extract the offset to the inner IP header (PASS Gen2)*/
#define PASAHO_LINFO_READ_TSTAMP_MSB_GEN2(x)     (x)->word6                               /**< Extract the most significant 32-bit of the 64-bit timestamp */
#define PASAHO_LINFO_READ_TSTAMP_MSB2_GEN2(x)    (x)->word8                               /**< Extract the most significant 32-bit of the 64-bit timestamp 
                                                                                          where the paylaod data has been copied to word6/7 within  
                                                                                          pasahoLongInfo_t by using PASS command pa_CMD_COPY_DATA_TO_PSINFO */
#define PASAHO_LINFO_READ_PATSTAMP_LSW_GEN2(x)   (x)->word9                               /**< Extract the most significant 32-bit of the 64-bit PA timestamp */
#define PASAHO_LINFO_READ_PATSTAMP_MSW_GEN2(x)   (x)->word8                               /**< Extract the least significant 32-bit of the 64-bit PA timestamp */

#define PASAHO_LINFO_READ_PATSTAMP_LSW2_GEN2(x)  (x)->word11                              /**< Extract the most significant 32-bit of the 64-bit PA timestamp
                                                                                          where the paylaod data has been copied to word6/7 within
                                                                                          pasahoLongInfo_t by using PASS command pa_CMD_COPY_DATA_TO_PSINFO */
#define PASAHO_LINFO_READ_PATSTAMP_MSW2_GEN2(x)  (x)->word10                              /**< Extract the most significant 32-bit of the 64-bit PA timestamp
                                                                                          where the paylaod data has been copied to word6/7 within
                                                                                          pasahoLongInfo_t by using PASS command pa_CMD_COPY_DATA_TO_PSINFO */

#define PASAHO_LINFO_READ_EOAM_TF_MATCH_CNT_GEN2(x)   (x)->word5                               /**< Extract the Ethernet OAM target flow match count, valid during 
                                                                                               EOAM mode on LMM/LMR control packts only (PASS Gen2)*/
#define PASAHO_LINFO_READ_EOAM_PKT_MEG_LEVEL_GEN2(x)  PASAHO_READ_BITFIELD((x)->word2,24,8)    /**< Extract the MEG Level during a valid EOAM packet measurement, valid during 
                                                                                               EOAM mode only (PASS Gen2)*/  
#define PASAHO_LINFO_READ_EOAM_PKT_OPCODE_GEN2(x)     PASAHO_READ_BITFIELD((x)->word2,16,8)    /**< Extract the MEG Level during a valid EOAM packet measurement, valid during 
                                                                                                 EOAM mode only (PASS Gen2)*/                                                                                               
                                                                                               

/*@}*/ /* PASAHO_long_info_command_gen2 macros */
/** @}*/ /* @name PASAHO Long Info Command Macros For Second Generation PASS */

/** 
 *  @defgroup PASAHO_long_info_command_macros  PASAHO Long Info Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info Command Macros
 *  Macros used by the PASAHO Long Info Command
 */
/*@{*/

#ifdef NSS_GEN2
#define PASAHO_LINFO_READ_CMDID(x)          PASAHO_LINFO_READ_CMDID_GEN2(x)           /**< Extract the command ID defined at @ref pasahoCommands */
#define PASAHO_LINFO_READ_RECLEN(x)         PASAHO_LINFO_READ_RECLEN_GEN2(x)          /**< Extract the block length */
#define PASAHO_LINFO_READ_START_OFFSET(x)   PASAHO_LINFO_READ_START_OFFSET_GEN2(x)    /**< Extract the next parse start offset */

#define PASAHO_LINFO_IS_MAC_BROADCAST(x)    PASAHO_LINFO_IS_MAC_BROADCAST_GEN2(x)     /**< Indicate whether it is a broadcast MAC packet */
#define PASAHO_LINFO_IS_MAC_MULTICAST(x)    PASAHO_LINFO_IS_MAC_MULTICAST_GEN2(x)     /**< Indicate whether it is a multicast MAC packet */
#define PASAHO_LINFO_READ_MAC_PKTTYPE(x)    PASAHO_LINFO_READ_MAC_PKTTYPE_GEN2(x)     /**< Extract the MAC packet type */

#define PASAHO_LINFO_IS_IP_BROADCAST(x)     PASAHO_LINFO_IS_IP_BROADCAST_GEN2(x)      /**< Indicate whether it is a broadcast IP packet */
#define PASAHO_LINFO_IS_IP_MULTICAST(x)     PASAHO_LINFO_IS_IP_MULTICAST_GEN2(x)      /**< Indicate whether it is a multicast IP packet */
#define PASAHO_LINFO_READ_IP_PKTTYPE(x)     PASAHO_LINFO_READ_IP_PKTTYPE_GEN2(x)      /**< Extract the IP packet type */

#define PASAHO_LINFO_READ_PMATCH(x)         PASAHO_LINFO_READ_PMATCH_GEN2(x)          /**< Extract the previous match flag */
#define PASAHO_LINFO_READ_FLAG_FRAG(x)      PASAHO_LINFO_READ_FLAG_FRAG_GEN2(x)       /**< Extract the fragmentation found flag */

#define PASAHO_LINFO_READ_END_OFFSET(x)     PASAHO_LINFO_READ_END_OFFSET_GEN2(x)      /**< Extract the end of packet parse offset */
#define PASAHO_LINFO_READ_EIDX(x)           PASAHO_LINFO_READ_EIDX_GEN2(x)            /**< Extract the exception index */
#define PASAHO_LINFO_READ_NXT_HDR_TYPE(x)   PASAHO_LINFO_READ_NXT_HDR_TYPE_GEN2(x)    /**< Extract the next header to parse type */
#define PASAHO_LINFO_READ_INPORT(x)         PASAHO_LINFO_READ_INPORT_GEN2(x)          /**< Extract the (1-based) input EMAC port number  
                                                                                           through CPSW */

#define PASAHO_LINFO_READ_L3_OFFSET(x)      PASAHO_LINFO_READ_L3_OFFSET_GEN2(x)       /**< Extract the offset to the level 3 header */
#define PASAHO_LINFO_READ_L4_OFFSET(x)      PASAHO_LINFO_READ_L4_OFFSET_GEN2(x)       /**< Extract the offset to the level 4 header */
#define PASAHO_LINFO_READ_L5_OFFSET(x)      PASAHO_LINFO_READ_L5_OFFSET_GEN2(x)       /**< Extract the offset to the level 5 header */
#define PASAHO_LINFO_READ_ESP_AH_OFFSET(x)  PASAHO_LINFO_READ_ESP_AH_OFFSET_GEN2(x)   /**< Extract the offset to the security header */

#define PASAHO_LINFO_READ_L1_PDSP_ID(x)     PASAHO_LINFO_READ_L1_PDSP_ID_GEN2(x)      /**< Extract the first parse module ID */
#define PASAHO_LINFO_READ_L1_IDX(x)         PASAHO_LINFO_READ_L1_IDX_GEN2(x)          /**< Extract the first parse module match index */
#define PASAHO_LINFO_READ_HDR_BITMASK(x)    PASAHO_LINFO_READ_HDR_BITMASK_GEN2(x)     /**< Extract the bitmask of parsed header types */
#define PASAHO_LINFO_READ_HDR_BITMASK2(x)   PASAHO_LINFO_READ_HDR_BITMASK2_GEN2(x)    /**< Extract the bitmask2 of parsed header types */
#define PASAHO_LINFO_READ_VLAN_COUNT(x)     PASAHO_LINFO_READ_VLAN_COUNT_GEN2(x)      /**< Extract the number of VLAN tags found */
#define PASAHO_LINFO_READ_IP_COUNT(x)       PASAHO_LINFO_READ_IP_COUNT_GEN2(x)        /**< Extract the number of IP headers found */
#define PASAHO_LINFO_READ_GRE_COUNT(x)      PASAHO_LINFO_READ_GRE_COUNT_GEN2(x)       /**< Extract the number of GRE headers found */

#define PASAHO_LINFO_READ_INNER_IP_OFFSET(x)PASAHO_LINFO_READ_INNER_IP_OFFSET_GEN2(x) /**< Extract the offset to the inner IP header */
#define PASAHO_LINFO_READ_TSTAMP_MSB(x)     PASAHO_LINFO_READ_TSTAMP_MSB_GEN2(x)      /**< Extract the most significant 32-bit of the 64-bit timestamp */
#define PASAHO_LINFO_READ_TSTAMP_MSB2(x)    PASAHO_LINFO_READ_TSTAMP_MSB2_GEN2(x)     /**< Extract the most significant 32-bit of the 64-bit timestamp 
                                                                                           where the paylaod data has been copied to word6/7 within  
                                                                                           pasahoLongInfo_t by using PASS command pa_CMD_COPY_DATA_TO_PSINFO */
#define PASAHO_LINFO_READ_EOAM_TF_MATCH_CNT(x)  PASAHO_LINFO_READ_EOAM_TF_MATCH_CNT_GEN2(x) /**< Extract the EOAM target flow match count for LMM/LMR packets */
#define PASAHO_LINFO_READ_EOAM_PKT_MEG_LEVEL(x) PASAHO_LINFO_READ_EOAM_PKT_MEG_LEVEL_GEN2(x) /**< Extract the EOAM MEG level for that packet */
#define PASAHO_LINFO_READ_EOAM_PKT_OPCODE(x) PASAHO_LINFO_READ_EOAM_PKT_OPCODE_GEN2(x) /**< Extract the EOAM Opcode for that packet */

#else
#define PASAHO_LINFO_READ_CMDID(x)          PASAHO_LINFO_READ_CMDID_GEN1(x)           /**< Extract the command ID defined at @ref pasahoCommands */
#define PASAHO_LINFO_READ_RECLEN(x)         PASAHO_LINFO_READ_RECLEN_GEN1(x)          /**< Extract the block length */
#define PASAHO_LINFO_READ_START_OFFSET(x)   PASAHO_LINFO_READ_START_OFFSET_GEN1(x)    /**< Extract the next parse start offset */

#define PASAHO_LINFO_IS_MAC_BROADCAST(x)    PASAHO_LINFO_IS_MAC_BROADCAST_GEN1(x)     /**< Indicate whether it is a broadcast MAC packet */
#define PASAHO_LINFO_IS_MAC_MULTICAST(x)    PASAHO_LINFO_IS_MAC_MULTICAST_GEN1(x)     /**< Indicate whether it is a multicast MAC packet */
#define PASAHO_LINFO_READ_MAC_PKTTYPE(x)    PASAHO_LINFO_READ_MAC_PKTTYPE_GEN1(x)     /**< Extract the MAC packet type */

#define PASAHO_LINFO_IS_IP_BROADCAST(x)     PASAHO_LINFO_IS_IP_BROADCAST_GEN1(x)      /**< Indicate whether it is a broadcast IP packet */
#define PASAHO_LINFO_IS_IP_MULTICAST(x)     PASAHO_LINFO_IS_IP_MULTICAST_GEN1(x)      /**< Indicate whether it is a multicast IP packet */
#define PASAHO_LINFO_READ_IP_PKTTYPE(x)     PASAHO_LINFO_READ_IP_PKTTYPE_GEN1(x)      /**< Extract the IP packet type */

#define PASAHO_LINFO_READ_END_OFFSET(x)     PASAHO_LINFO_READ_END_OFFSET_GEN1(x)      /**< Extract the end of packet parse offset */
#define PASAHO_LINFO_READ_EIDX(x)           PASAHO_LINFO_READ_EIDX_GEN1(x)            /**< Extract the error index */
#define PASAHO_LINFO_READ_PMATCH(x)         PASAHO_LINFO_READ_PMATCH_GEN1(x)          /**< Extract the previous match flag */
#define PASAHO_LINFO_READ_L1_PDSP_ID(x)     PASAHO_LINFO_READ_L1_PDSP_ID_GEN1(x)      /**< Extract the first parse module ID */
#define PASAHO_LINFO_READ_L1_IDX(x)         PASAHO_LINFO_READ_L1_IDX_GEN1(x)          /**< Extract the first parse module match index */

#define PASAHO_LINFO_READ_L3_OFFSET(x)      PASAHO_LINFO_READ_L3_OFFSET_GEN1(x)       /**< Extract the offset to the level 3 header */
#define PASAHO_LINFO_READ_L4_OFFSET(x)      PASAHO_LINFO_READ_L4_OFFSET_GEN1(x)       /**< Extract the offset to the level 4 header */
#define PASAHO_LINFO_READ_L5_OFFSET(x)      PASAHO_LINFO_READ_L5_OFFSET_GEN1(x)       /**< Extract the offset to the level 5 header */
#define PASAHO_LINFO_READ_ESP_AH_OFFSET(x)  PASAHO_LINFO_READ_ESP_AH_OFFSET_GEN1(x)   /**< Extract the offset to the security header */

#define PASAHO_LINFO_READ_HDR_BITMASK(x)    PASAHO_LINFO_READ_HDR_BITMASK_GEN1(x)     /**< Extract the bitmask of parsed header types */
#define PASAHO_LINFO_READ_HDR_BITMASK2(x)   PASAHO_LINFO_READ_HDR_BITMASK2_GEN1(x)    /**< Extract the bitmask2 of parsed header types */
#define PASAHO_LINFO_READ_NXT_HDR_TYPE(x)   PASAHO_LINFO_READ_NXT_HDR_TYPE_GEN1(x)    /**< Extract the next header to parse type */
#define PASAHO_LINFO_READ_VLAN_COUNT(x)     PASAHO_LINFO_READ_VLAN_COUNT_GEN1(x)      /**< Extract the number of VLAN tags found */
#define PASAHO_LINFO_READ_IP_COUNT(x)       PASAHO_LINFO_READ_IP_COUNT_GEN1(x)        /**< Extract the number of IP headers found */
#define PASAHO_LINFO_READ_GRE_COUNT(x)      PASAHO_LINFO_READ_GRE_COUNT_GEN1(x)       /**< Extract the number of GRE headers found */
#define PASAHO_LINFO_READ_FLAG_FRAG(x)      PASAHO_LINFO_READ_FLAG_FRAG_GEN1(x)       /**< Extract the fragmentation found flag */
#define PASAHO_LINFO_READ_INPORT(x)         PASAHO_LINFO_READ_INPORT_GEN1(x)          /**< Extract the (1-based) input EMAC port number  
                                                                                           0: Indicates that the packet does not enter PASS 
                                                                                           through CPSW */
#define PASAHO_LINFO_READ_INNER_IP_OFFSET(x)PASAHO_LINFO_READ_INNER_IP_OFFSET_GEN1(x) /**< Extract the offset to the inner IP header */
#define PASAHO_LINFO_READ_TSTAMP_MSB(x)     PASAHO_LINFO_READ_TSTAMP_MSB_GEN1(x)      /**< Extract the most significant 32-bit of the 64-bit timestamp */
#define PASAHO_LINFO_READ_TSTAMP_MSB2(x)    PASAHO_LINFO_READ_TSTAMP_MSB2_GEN1(x)     /**< Extract the most significant 32-bit of the 64-bit timestamp 
                                                                                           where the paylaod data has been copied to word6/7 within  
                                                                                           pasahoLongInfo_t by using PASS command pa_CMD_COPY_DATA_TO_PSINFO */
#endif

/* Extract Protocol Information */

/*@}*/ /* PASAHO_long_info_command_macros */
/** @}*/ /* @name PASAHO Long Info Command Macros */

/**
 *  @defgroup PASAHO_long_info_proto_ind_gen1_macros  PASAHO Long Info Protocol Indication Macros For First Generation PASS
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info Protocol Indication Macros For First Generation PASS
 */
#define PASAHO_LINFO_IS_MAC_GEN1(x)              PASAHO_READ_BITFIELD((x)->word3,21,1)    /**< Indicate whether it is a MAC packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_802_3_GEN1(x)            PASAHO_READ_BITFIELD((x)->word3,7,1)     /**< Indicate whether it is a 802.3 packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_WITH_VLAN_GEN1(x)        PASAHO_LINFO_READ_VLAN_COUNT(x)          /**< Indicate whether it is a MAC packet with VLAN (Pass Gen1)*/
#define PASAHO_LINFO_IS_WITH_MPLS_GEN1(x)        PASAHO_READ_BITFIELD((x)->word3,23,1)    /**< Indicate whether it is a MAC packet with MPLS (Pass Gen1)*/
#define PASAHO_LINFO_IS_PPPoE_GEN1(x)            PASAHO_READ_BITFIELD((x)->word3,6,1)     /**< Indicate whether it is a PPPoE packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_IP_GEN1(x)               PASAHO_LINFO_READ_IP_COUNT(x)            /**< Indicate whether it is an IP packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_IPSEC_ESP_GEN1(x)        PASAHO_READ_BITFIELD((x)->word3,25,1)    /**< Indicate whether it is an IPSEC ESP packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_IPSEC_AH_GEN1(x)         PASAHO_READ_BITFIELD((x)->word3,26,1)    /**< Indicate whether it is an IPSEC AH packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_UDP_GEN1(x)              PASAHO_READ_BITFIELD((x)->word3,27,1)    /**< Indicate whether it is an UDP packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_UDP_LITE_GEN1(x)         PASAHO_READ_BITFIELD((x)->word3,28,1)    /**< Indicate whether it is an UDP Lite packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_TCP_GEN1(x)              PASAHO_READ_BITFIELD((x)->word3,29,1)    /**< Indicate whether it is a TCP packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_GRE_GEN1(x)              PASAHO_LINFO_READ_GRE_COUNT(x)           /**< Indicate whether it is a GRE packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_GTPU_GEN1(x)             PASAHO_READ_BITFIELD((x)->word3,30,1)    /**< Indicate whether it is a GTPU packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_CUSTOM_GEN1(x)           PASAHO_READ_BITFIELD((x)->word3,31,1)    /**< Indicate whether it is a Custom packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_SCTP_GEN1(x)             PASAHO_READ_BITFIELD((x)->word3,4,1)     /**< Indicate whether it is a SCTP packet (Pass Gen1)*/
#define PASAHO_LINFO_IS_IPSEC_NAT_T_GEN1(x)      PASAHO_READ_BITFIELD((x)->word3,5,1)     /**< Indicate whether it is an IPSEC NAT-T packet (Pass Gen1)*/

/*@}*/ /* PASAHO_long_info_proto_ind_gen1_macross */
/** @}*/ /* @name PASAHO Long Info Protocol Indication Macross For First Generation PASS */

/**
 *  @defgroup PASAHO_long_info_proto_ind_gen2_macros  PASAHO Long Info Protocol Indication Macros For Second Generation PASS
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info Protocol Indication Macros For Second Generation PASS
 */
#define PASAHO_LINFO_IS_MAC_GEN2(x)              PASAHO_READ_BITFIELD((x)->word3,0,1)     /**< Indicate whether it is a MAC packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_WITH_VLAN_GEN2(x)        PASAHO_LINFO_READ_VLAN_COUNT(x)          /**< Indicate whether it is a MAC packet with VLAN (Pass Gen2)*/
#define PASAHO_LINFO_IS_WITH_MPLS_GEN2(x)        PASAHO_READ_BITFIELD((x)->word3,2,1)     /**< Indicate whether it is a MAC packet with MPLS (Pass Gen2)*/
#define PASAHO_LINFO_IS_802_3_GEN2(x)            PASAHO_READ_BITFIELD((x)->word3,3,1)     /**< Indicate whether it is a 802.3 packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_PPPoE_GEN2(x)            PASAHO_READ_BITFIELD((x)->word3,4,1)     /**< Indicate whether it is a PPPoE packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_IP_GEN2(x)               PASAHO_LINFO_READ_IP_COUNT(x)            /**< Indicate whether it is an IP packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_IPv4_GEN2(x)             PASAHO_READ_BITFIELD((x)->word3,5,1)     /**< Indicate whether it is an IPv4 packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_IPv6_GEN2(x)             PASAHO_READ_BITFIELD((x)->word3,6,1)     /**< Indicate whether it is an IPv4 packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_IP_OPTIONS_GEN2(x)       PASAHO_READ_BITFIELD((x)->word3,7,1)     /**< Indicate whether there are IPV4 options or IPv6 extention headers (Pass Gen2)*/
#define PASAHO_LINFO_IS_IPSEC_ESP_GEN2(x)        PASAHO_READ_BITFIELD((x)->word3,8,1)     /**< Indicate whether it is an IPSEC ESP packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_IPSEC_AH_GEN2(x)         PASAHO_READ_BITFIELD((x)->word3,9,1)     /**< Indicate whether it is an IPSEC AH packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_SCTP_GEN2(x)             PASAHO_READ_BITFIELD((x)->word3,10,1)    /**< Indicate whether it is a SCTP packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_UDP_GEN2(x)              PASAHO_READ_BITFIELD((x)->word3,11,1)    /**< Indicate whether it is an UDP packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_UDP_LITE_GEN2(x)         PASAHO_READ_BITFIELD((x)->word3,11,1)    /**< Indicate whether it is an UDP Lite packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_TCP_GEN2(x)              PASAHO_READ_BITFIELD((x)->word3,12,1)    /**< Indicate whether it is a TCP packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_GRE_GEN2(x)              PASAHO_LINFO_READ_GRE_COUNT(x)           /**< Indicate whether it is a GRE packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_GTPU_GEN2(x)             PASAHO_READ_BITFIELD((x)->word3,13,1)    /**< Indicate whether it is a GTPU packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_CUSTOM_GEN2(x)           PASAHO_READ_BITFIELD((x)->word3,14,1)    /**< Indicate whether it is a Custom packet (Pass Gen2)*/
#define PASAHO_LINFO_IS_IPSEC_NAT_T_GEN2(x)      PASAHO_READ_BITFIELD((x)->word3,15,1)    /**< Indicate whether it is an IPSEC NAT-T packet (Pass Gen2)*/

/*@}*/ /* PASAHO_long_info_proto_ind_macross */
/** @}*/ /* @name PASAHO Long Info Protocol Indication Macross For Second Generation PASS */

/**
 *  @defgroup PASAHO_long_info_proto_ind_macros  PASAHO Long Info Protocol Indication Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info Protocol Indication Macros
 */
#ifdef NSS_GEN2
#define PASAHO_LINFO_IS_MAC(x)          PASAHO_LINFO_IS_MAC_GEN2(x)             /**< Indicate whether it is a MAC packet */
#define PASAHO_LINFO_IS_WITH_VLAN(x)    PASAHO_LINFO_IS_WITH_VLAN_GEN2(x)       /**< Indicate whether it is a MAC packet with VLAN */
#define PASAHO_LINFO_IS_WITH_MPLS(x)    PASAHO_LINFO_IS_WITH_MPLS_GEN2(x)       /**< Indicate whether it is a MAC packet with MPLS */
#define PASAHO_LINFO_IS_802_3(x)        PASAHO_LINFO_IS_802_3_GEN2(x)           /**< Indicate whether it is a 802.3 packet */
#define PASAHO_LINFO_IS_PPPoE(x)        PASAHO_LINFO_IS_PPPoE_GEN2(x)           /**< Indicate whether it is a PPPoE packet */
#define PASAHO_LINFO_IS_IP(x)           PASAHO_LINFO_IS_IP_GEN2(x)              /**< Indicate whether it is an IP packet */
#define PASAHO_LINFO_IS_IPv4(x)         PASAHO_LINFO_IS_IPv4_GEN2(x)            /**< Indicate whether it is an IPv4 packet */
#define PASAHO_LINFO_IS_IPv6(x)         PASAHO_LINFO_IS_IPv6_GEN2(x)            /**< Indicate whether it is an IPv4 packet */
#define PASAHO_LINFO_IS_IP_OPTIONS(x)   PASAHO_LINFO_IS_IP_OPTIONS_GEN2(x)      /**< Indicate whether there are IPV4 options or IPv6 extention headers */
#define PASAHO_LINFO_IS_IPSEC_ESP(x)    PASAHO_LINFO_IS_IPSEC_ESP_GEN2(x)       /**< Indicate whether it is an IPSEC ESP packet */
#define PASAHO_LINFO_IS_IPSEC_AH(x)     PASAHO_LINFO_IS_IPSEC_AH_GEN2(x)        /**< Indicate whether it is an IPSEC AH packet */
#define PASAHO_LINFO_IS_SCTP(x)         PASAHO_LINFO_IS_SCTP_GEN2(x)            /**< Indicate whether it is a SCTP packet */
#define PASAHO_LINFO_IS_UDP(x)          PASAHO_LINFO_IS_UDP_GEN2(x)             /**< Indicate whether it is an UDP packet */
#define PASAHO_LINFO_IS_UDP_LITE(x)     PASAHO_LINFO_IS_UDP_LITE_GEN2(x)        /**< Indicate whether it is an UDP Lite packet */
#define PASAHO_LINFO_IS_TCP(x)          PASAHO_LINFO_IS_TCP_GEN2(x)             /**< Indicate whether it is a TCP packet */
#define PASAHO_LINFO_IS_GRE(x)          PASAHO_LINFO_IS_GRE_GEN2(x)             /**< Indicate whether it is a GRE packet */
#define PASAHO_LINFO_IS_GTPU(x)         PASAHO_LINFO_IS_GTPU_GEN2(x)            /**< Indicate whether it is a GTPU packet */
#define PASAHO_LINFO_IS_CUSTOM(x)       PASAHO_LINFO_IS_CUSTOM_GEN2(x)          /**< Indicate whether it is a Custom packet */
#define PASAHO_LINFO_IS_IPSEC_NAT_T(x)  PASAHO_LINFO_IS_IPSEC_NAT_T_GEN2(x)     /**< Indicate whether it is an IPSEC NAT-T packet */
#else
#define PASAHO_LINFO_IS_MAC(x)          PASAHO_LINFO_IS_MAC_GEN1(x)             /**< Indicate whether it is a MAC packet */
#define PASAHO_LINFO_IS_802_3(x)        PASAHO_LINFO_IS_802_3_GEN1(x)           /**< Indicate whether it is a 802.3 packet */
#define PASAHO_LINFO_IS_WITH_VLAN(x)    PASAHO_LINFO_IS_WITH_VLAN_GEN1(x)       /**< Indicate whether it is a MAC packet with VLAN */
#define PASAHO_LINFO_IS_WITH_MPLS(x)    PASAHO_LINFO_IS_WITH_MPLS_GEN1(x)       /**< Indicate whether it is a MAC packet with MPLS */
#define PASAHO_LINFO_IS_PPPoE(x)        PASAHO_LINFO_IS_PPPoE_GEN1(x)           /**< Indicate whether it is a PPPoE packet */
#define PASAHO_LINFO_IS_IP(x)           PASAHO_LINFO_IS_IP_GEN1(x)              /**< Indicate whether it is an IP packet */
#define PASAHO_LINFO_IS_IPSEC_ESP(x)    PASAHO_LINFO_IS_IPSEC_ESP_GEN1(x)       /**< Indicate whether it is an IPSEC ESP packet */
#define PASAHO_LINFO_IS_IPSEC_AH(x)     PASAHO_LINFO_IS_IPSEC_AH_GEN1(x)        /**< Indicate whether it is an IPSEC AH packet */
#define PASAHO_LINFO_IS_UDP(x)          PASAHO_LINFO_IS_UDP_GEN1(x)             /**< Indicate whether it is an UDP packet */
#define PASAHO_LINFO_IS_UDP_LITE(x)     PASAHO_LINFO_IS_UDP_LITE_GEN1(x)        /**< Indicate whether it is an UDP Lite packet */
#define PASAHO_LINFO_IS_TCP(x)          PASAHO_LINFO_IS_TCP_GEN1(x)             /**< Indicate whether it is a TCP packet */
#define PASAHO_LINFO_IS_GRE(x)          PASAHO_LINFO_IS_GRE_GEN1(x)             /**< Indicate whether it is a GRE packet */
#define PASAHO_LINFO_IS_GTPU(x)         PASAHO_LINFO_IS_GTPU_GEN1(x)            /**< Indicate whether it is a GTPU packet */
#define PASAHO_LINFO_IS_CUSTOM(x)       PASAHO_LINFO_IS_CUSTOM_GEN1(x)          /**< Indicate whether it is a Custom packet */
#define PASAHO_LINFO_IS_SCTP(x)         PASAHO_LINFO_IS_SCTP_GEN1(x)            /**< Indicate whether it is a SCTP packet */
#define PASAHO_LINFO_IS_IPSEC_NAT_T(x)  PASAHO_LINFO_IS_IPSEC_NAT_T_GEN1(x)     /**< Indicate whether it is an IPSEC NAT-T packet */
#endif

/*@}*/ /* PASAHO_long_info_proto_ind_macross */
/** @}*/ /* @name PASAHO Long Info Protocol Indication Macross */

/**
 *  @defgroup PASAHO_long_info_ipReassm_gen1_macros  PASAHO Long Info IP Reassembly Macros For First Generation PASS
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info IpReassm Macros For First Generation PASS
 *  Macros used by the PASAHO PASS-assisted IP Reassembly Operation
 *
 */
#define PASAHO_LINFO_READ_TFINDEX_GEN1(x)        PASAHO_READ_BITFIELD((x)->word4,24,8)     /**< Extract the IP Reassembly Traffic Flow Index (PASS Gen1)*/
#define PASAHO_LINFO_READ_FRANCNT_GEN1(x)        PASAHO_READ_BITFIELD((x)->word4,16,8)     /**< Extract the IP Reassembly Fragment count (PASS Gen1)*/

#define PASAHO_LINFO_SET_TFINDEX_GEN1(x, v)      PASAHO_SET_BITFIELD((x)->word4,(v),24,8)  /**< Set the IP Reassembly Traffic Flow Index (PASS Gen1)*/
#define PASAHO_LINFO_SET_FRANCNT_GEN1(x, v)      PASAHO_SET_BITFIELD((x)->word4,(v),16,8)  /**< Set the IP Reassembly Fragment count (PASS Gen1)*/

#define PASAHO_LINFO_IS_IPSEC_GEN1(x)            PASAHO_READ_BITFIELD((x)->word3,25,2)     /**< Indicate whether it is an IPSEC packet (PASS Gen1)*/
#define PASAHO_LINFO_CLR_IPSEC_GEN1(x)           PASAHO_SET_BITFIELD((x)->word3,0,25,2)    /**< Clear IPSEC indication bits (PASS Gen1)*/
#define PASAHO_LINFO_CLR_IPSEC_ESP_GEN1(x)       PASAHO_SET_BITFIELD((x)->word3,0,26,1)    /**< Clear IPSEC ESP indication bit (PASS Gen1)*/
#define PASAHO_LINFO_CLR_IPSEC_AH_GEN1(x)        PASAHO_SET_BITFIELD((x)->word3,0,25,1)    /**< Claer IPSEC AH indication bit (PASS Gen1)*/
#define PASAHO_LINFO_CLR_FLAG_FRAG_GEN1(x)       PASAHO_SET_BITFIELD((x)->word3,0,3,1)     /**< Clear the fragmentation found flag (PASS Gen1)*/


#define PASAHO_LINFO_SET_START_OFFSET_GEN1(x, v) PASAHO_SET_BITFIELD((x)->word0,(v),0,16)  /**< Update the next parse start offset (PASS Gen1)*/
#define PASAHO_LINFO_SET_END_OFFSET_GEN1(x, v)   PASAHO_SET_BITFIELD((x)->word1,(v),16,16) /**< Update the end of packet parse offset (PASS Gen1)*/

#define PASAHO_LINFO_SET_NULL_PKT_IND_GEN1(x, v) PASAHO_SET_BITFIELD((x)->word0,(v),21,1)  /**< Set the null packet flag which indicates that the packet should be dropped. 
                                                                                           This flag should be set for the null packet to be delivered to PASS when
                                                                                           the reassembly timeout occurs    (PASS Gen1)*/

/*@}*/ /* PASAHO_long_info_ipReassm_gen1_macros */
/** @}*/ /* @name PASAHO Long Info IpReassm Macros For First Generation PASS */

/**
 *  @defgroup PASAHO_long_info_ipReassm_gen2_macros  PASAHO Long Info IP Reassembly Macros For Second Generation PASS
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info IpReassm Macros
 *  Macros used by the PASAHO PASS-assisted IP Reassembly Operation For Second Generation PASS
 *
 */
#define PASAHO_LINFO_READ_TFINDEX_GEN2(x)        PASAHO_READ_BITFIELD((x)->word5,24,8)     /**< Extract the IP Reassembly Traffic Flow Index (Pass Gen2)*/
#define PASAHO_LINFO_READ_FRANCNT_GEN2(x)        PASAHO_READ_BITFIELD((x)->word5,16,8)     /**< Extract the IP Reassembly Fragment count (Pass Gen2)*/

#define PASAHO_LINFO_SET_TFINDEX_GEN2(x, v)      PASAHO_SET_BITFIELD((x)->word5,(v),24,8)  /**< Set the IP Reassembly Traffic Flow Index (Pass Gen2)*/
#define PASAHO_LINFO_SET_FRANCNT_GEN2(x, v)      PASAHO_SET_BITFIELD((x)->word5,(v),16,8)  /**< Set the IP Reassembly Fragment count (Pass Gen2)*/

#define PASAHO_LINFO_IS_IPSEC_GEN2(x)            PASAHO_READ_BITFIELD((x)->word3,8,2)      /**< Indicate whether it is an IPSEC packet (Pass Gen2)*/
#define PASAHO_LINFO_CLR_IPSEC_GEN2(x)           PASAHO_SET_BITFIELD((x)->word3,0,8,2)     /**< Clear IPSEC indication bits (Pass Gen2)*/
#define PASAHO_LINFO_CLR_IPSEC_ESP_GEN2(x)       PASAHO_SET_BITFIELD((x)->word3,0,8,1)     /**< Clear IPSEC ESP indication bit (Pass Gen2)*/
#define PASAHO_LINFO_CLR_IPSEC_AH_GEN2(x)        PASAHO_SET_BITFIELD((x)->word3,0,9,1)     /**< Claer IPSEC AH indication bit (Pass Gen2)*/
#define PASAHO_LINFO_CLR_FLAG_FRAG_GEN2(x)       PASAHO_SET_BITFIELD((x)->word1,0,19,1)    /**< Clear the fragmentation found flag (Pass Gen2)*/


#define PASAHO_LINFO_SET_START_OFFSET_GEN2(x, v) PASAHO_SET_BITFIELD((x)->word0,(v),0,8)   /**< Update the next parse start offset (Pass Gen2)*/
#define PASAHO_LINFO_SET_END_OFFSET_GEN2(x, v)   PASAHO_SET_BITFIELD((x)->word1,(v),16,16) /**< Update the end of packet parse offset (Pass Gen2)*/
#define PASAHO_LINFO_SET_NXT_HDR_TYPE_GEN2(x, v) PASAHO_SET_BITFIELD((x)->word1,(v),0,6)   /**< Update the next header to parse type (Pass Gen2)*/

#define PASAHO_LINFO_SET_NULL_PKT_IND_GEN2(x, v) PASAHO_SET_BITFIELD((x)->word0,(v),13,1)  /**< Set the null packet flag which indicates that the packet should be dropped. 
                                                                                                This flag should be set for the null packet to be delivered to PASS when
                                                                                                the reassembly timeout occurs    (Pass Gen2)*/
/*@}*/ /* PASAHO_long_info_ipReassm_gen2_macros */
/** @}*/ /* @name PASAHO Long Info IpReassm Macros For Second Generation PASS */

/**
 *  @defgroup PASAHO_long_info_ipReassm_macros  PASAHO Long Info IP Reassembly Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info IpReassm Macros
 *  Macros used by the PASAHO PASS-assisted IP Reassembly Operation
 *
 */
#ifdef NSS_GEN2 
#define PASAHO_LINFO_READ_TFINDEX(x)        PASAHO_LINFO_READ_TFINDEX_GEN2(x)         /**< Extract the IP Reassembly Traffic Flow Index */
#define PASAHO_LINFO_READ_FRANCNT(x)        PASAHO_LINFO_READ_FRANCNT_GEN2(x)         /**< Extract the IP Reassembly Fragment count */

#define PASAHO_LINFO_SET_TFINDEX(x, v)      PASAHO_LINFO_SET_TFINDEX_GEN2(x, v)       /**< Set the IP Reassembly Traffic Flow Index */
#define PASAHO_LINFO_SET_FRANCNT(x, v)      PASAHO_LINFO_SET_FRANCNT_GEN2(x, v)       /**< Set the IP Reassembly Fragment count */

#define PASAHO_LINFO_IS_IPSEC(x)            PASAHO_LINFO_IS_IPSEC_GEN2(x)             /**< Indicate whether it is an IPSEC packet */
#define PASAHO_LINFO_CLR_IPSEC(x)           PASAHO_LINFO_CLR_IPSEC_GEN2(x)            /**< Clear IPSEC indication bits */
#define PASAHO_LINFO_CLR_IPSEC_ESP(x)       PASAHO_LINFO_CLR_IPSEC_ESP_GEN2(x)        /**< Clear IPSEC ESP indication bit */
#define PASAHO_LINFO_CLR_IPSEC_AH(x)        PASAHO_LINFO_CLR_IPSEC_AH_GEN2(x)         /**< Claer IPSEC AH indication bit */
#define PASAHO_LINFO_CLR_FLAG_FRAG(x)       PASAHO_LINFO_CLR_FLAG_FRAG_GEN2(x)        /**< Clear the fragmentation found flag */


#define PASAHO_LINFO_SET_START_OFFSET(x, v) PASAHO_LINFO_SET_START_OFFSET_GEN2(x, v)  /**< Update the next parse start offset */
#define PASAHO_LINFO_SET_END_OFFSET(x, v)   PASAHO_LINFO_SET_END_OFFSET_GEN2(x, v)    /**< Update the end of packet parse offset */
#define PASAHO_LINFO_SET_NXT_HDR_TYPE(x, v) PASAHO_LINFO_SET_NXT_HDR_TYPE_GEN2(x, v)  /**< Update the next header to parse type */

#define PASAHO_LINFO_SET_NULL_PKT_IND(x, v) PASAHO_LINFO_SET_NULL_PKT_IND_GEN2(x, v)  /**< Set the null packet flag which indicates that the packet should be dropped. 
                                                                                           This flag should be set for the null packet to be delivered to PASS when
                                                                                           the reassembly timeout occurs    */
#else
#define PASAHO_LINFO_READ_TFINDEX(x)        PASAHO_LINFO_READ_TFINDEX_GEN1(x)         /**< Extract the IP Reassembly Traffic Flow Index */
#define PASAHO_LINFO_READ_FRANCNT(x)        PASAHO_LINFO_READ_FRANCNT_GEN1(x)         /**< Extract the IP Reassembly Fragment count */

#define PASAHO_LINFO_SET_TFINDEX(x, v)      PASAHO_LINFO_SET_TFINDEX_GEN1(x, v)       /**< Set the IP Reassembly Traffic Flow Index */
#define PASAHO_LINFO_SET_FRANCNT(x, v)      PASAHO_LINFO_SET_FRANCNT_GEN1(x, v)       /**< Set the IP Reassembly Fragment count */

#define PASAHO_LINFO_IS_IPSEC(x)            PASAHO_LINFO_IS_IPSEC_GEN1(x)             /**< Indicate whether it is an IPSEC packet */
#define PASAHO_LINFO_CLR_IPSEC(x)           PASAHO_LINFO_CLR_IPSEC_GEN1(x)            /**< Clear IPSEC indication bits */
#define PASAHO_LINFO_CLR_IPSEC_ESP(x)       PASAHO_LINFO_CLR_IPSEC_ESP_GEN1(x)        /**< Clear IPSEC ESP indication bit */
#define PASAHO_LINFO_CLR_IPSEC_AH(x)        PASAHO_LINFO_CLR_IPSEC_AH_GEN1(x)         /**< Claer IPSEC AH indication bit */
#define PASAHO_LINFO_CLR_FLAG_FRAG(x)       PASAHO_LINFO_CLR_FLAG_FRAG_GEN1(x)        /**< Clear the fragmentation found flag */


#define PASAHO_LINFO_SET_START_OFFSET(x, v) PASAHO_LINFO_SET_START_OFFSET_GEN1(x, v)  /**< Update the next parse start offset */
#define PASAHO_LINFO_SET_END_OFFSET(x, v)   PASAHO_LINFO_SET_END_OFFSET_GEN1(x, v)    /**< Update the end of packet parse offset */

#define PASAHO_LINFO_SET_NULL_PKT_IND(x, v) PASAHO_LINFO_SET_NULL_PKT_IND_GEN1(x, v)  /**< Set the null packet flag which indicates that the packet should be dropped. 
                                                                                           This flag should be set for the null packet to be delivered to PASS when
                                                                                           the reassembly timeout occurs    */

#endif                                                                                           
/*@}*/ /* PASAHO_long_info_ipReassm_macros */
/** @}*/ /* @name PASAHO Long Info IpReassm Macros */

/**
 *  @def  PA_INV_TF_INDEX
 *        PASS-asssited IP reassembly traffic flow index to indicate that no traffic flow is available 
 */
#define PA_INV_TF_INDEX     0xFF    

/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoShortInfo_t defines the packet parsing information in the short format
 *
 *  @details pasahoShortInfo_t defines the packet parsing information in terms of
 *           payload offset and payload length as described below
 *           SRTP:      offset to the RTP header; RTP payload length including ICV
 *           IPSEC AH:  offset to the Outer IP; IP payload length
 *           IPSEC ESP: offset to the ESP header; ESP papload length including ICV
 */

typedef struct pasahoShortInfo_s {
    uint32_t  word0;   /**< Control block word 0 */
    uint32_t  word1;   /**< Optional supplement data (It may be padding for alignment only) */
} pasahoShortInfo_t;

/** 
 *  @defgroup PASAHO_short_info_command_macros  PASAHO Short Info Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Short Info Command Macros
 *  Macros used by the PASAHO Short Info Command
 *  
 */
/*@{*/
#define PASAHO_SINFO_READ_CMDID(x)          PASAHO_READ_BITFIELD((x)->word0,29,3)                           /**< Extract the command ID defined at @ref pasahoCommands */
#define PASAHO_SINFO_RESD_PAYLOAD_OFFSET(x) PASAHO_READ_BITFIELD((x)->word0,16,8)                           /**< Extract the offset to the packet payload */
#define PASAHO_SINFO_READ_PAYLOAD_LENGTH(x) PASAHO_READ_BITFIELD((x)->word0,0,16)                           /**< Extract the byte length of the payload */

#define PASAHO_SINFO_SET_PAYLOAD_OFFSET(x, v)  PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)                  /**< Set the offset to the payload */
#define PASAHO_SINFO_SET_PAYLOAD_LENGTH(x, v)  PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16)                 /**< Set the payload length */
#define PASAHO_SINFO_FORMAT_CMD(offset, len)   (((offset) << 16) | (len) | (PASAHO_SA_SHORT_INFO << 29))    /**< Format the entire short info command */

/*@}*/ /* PASAHO_short_info_command_macros */
/** @}*/ /* @name PASAHO Short Info Command Macros */

/* Header bitmask bits */
/**
 *  @defgroup pasahoHeaderBitmapGen1  PA/SA/HO Header Bitmap Bit Definitions For First Generation PASS
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PA/SA/HO Header Bitmap Bit Definitions For First Generation PASS
 *  Bitmap definition of the protocol header bitmask at the long info of First Generation PASS. 
 */ 
/*@{*/
#define PASAHO_HDR_BITMASK_MAC_GEN1     (1 << 0)     /**< MAC present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_VLAN_GEN1    (1 << 1)     /**< VLAN present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_MPLS_GEN1    (1 << 2)     /**< MPLS present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_IP_GEN1      (1 << 3)     /**< IP present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_ESP_GEN1     (1 << 4)     /**< IPSEC/ESP present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_AH_GEN1      (1 << 5)     /**< IPSEC/AH present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_UDP_GEN1     (1 << 6)     /**< UDP present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_UDPLITE_GEN1 (1 << 7)     /**< UDPLITE present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_TCP_GEN1     (1 << 8)     /**< TCP present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_GTPU_GEN1    (1 << 9)     /**< GTPU present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK_CUSTOM_GEN1  (1 << 10)    /**< Custom header present (PASS Gen1)*/

#define PASAHO_HDR_BITMASK2_SCTP_GEN1          (1 << 0)     /**< SCTP present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK2_IPSEC_NAT_T_GEN1   (1 << 1)     /**< IPSEC NAT-T present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK2_PPPoE_GEN1         (1 << 2)     /**< PPPoE present (PASS Gen1)*/
#define PASAHO_HDR_BITMASK2_802_3_GEN1         (1 << 3)     /**< 802.3 present (PASS Gen1)*/

/*@}*/
/** @} */

/* Header bitmask bits */
/**
 *  @defgroup pasahoHeaderBitmapGen2  PA/SA/HO Header Bitmap Bit Definitions For Second Generation PASS
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PA/SA/HO Header Bitmap Bit Definitions For Second Generation PASS
 *  Bitmap definition of the protocol header bitmask at the long info  For Second Generation PASS. 
 */ 
/*@{*/
#define PASAHO_HDR_BITMASK_MAC_GEN2     (1 << 0)     /**< MAC present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_VLAN_GEN2    (1 << 1)     /**< VLAN present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_MPLS_GEN2    (1 << 2)     /**< MPLS present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_802_3_GEN2   (1 << 3)     /**< 802.3 present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_PPPoE_GEN2   (1 << 4)     /**< PPPoE present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_IPv4_GEN2    (1 << 5)     /**< IPv4 present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_IPv6_GEN2    (1 << 6)     /**< IPv6 present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_IP_OPTS_GEN2 (1 << 7)     /**< IPv4 options or IPv6 extension headers present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_ESP_GEN2     (1 << 8)     /**< IPSEC/ESP present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_AH_GEN2      (1 << 9)     /**< IPSEC/AH present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_SCTP_GEN2    (1 << 10)    /**< SCTP present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_UDP_GEN2     (1 << 11)    /**< UDP present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_UDPLITE_GEN2 (1 << 11)    /**< UDPLITE present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_TCP_GEN2     (1 << 12)    /**< TCP present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_GTPU_GEN2    (1 << 13)     /**< GTPU present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_CUSTOM_GEN2  (1 << 14)    /**< Custom header present (PASS Gen2)*/
#define PASAHO_HDR_BITMASK_IPSEC_NAT_T_GEN2   (1 << 15)     /**< IPSEC NAT-T present (PASS Gen2)*/

/*@}*/
/** @} */

/* Header bitmask bits */
/**
 *  @defgroup pasahoHeaderBitmap  PA/SA/HO Header Bitmap Bit Definitions
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PA/SA/HO Header Bitmap Bit Definitions
 *  Bitmap definition of the protocol header bitmask at the long info. 
 */ 
/*@{*/
#ifdef NSS_GEN2
#define PASAHO_HDR_BITMASK_MAC         PASAHO_HDR_BITMASK_MAC_GEN2          /**< MAC present */
#define PASAHO_HDR_BITMASK_VLAN        PASAHO_HDR_BITMASK_VLAN_GEN2         /**< VLAN present */
#define PASAHO_HDR_BITMASK_MPLS        PASAHO_HDR_BITMASK_MPLS_GEN2         /**< MPLS present */
#define PASAHO_HDR_BITMASK_802_3       PASAHO_HDR_BITMASK_802_3_GEN2        /**< 802.3 present */
#define PASAHO_HDR_BITMASK_PPPoE       PASAHO_HDR_BITMASK_PPPoE_GEN2        /**< PPPoE present */
#define PASAHO_HDR_BITMASK_IPv4        PASAHO_HDR_BITMASK_IPv4_GEN2         /**< IPv4 present */
#define PASAHO_HDR_BITMASK_IPv6        PASAHO_HDR_BITMASK_IPv6_GEN2         /**< IPv6 present */
#define PASAHO_HDR_BITMASK_IP_OPTS     PASAHO_HDR_BITMASK_IP_OPTS_GEN2      /**< IPv4 options or IPv6 extension headers present */
#define PASAHO_HDR_BITMASK_ESP         PASAHO_HDR_BITMASK_ESP_GEN2          /**< IPSEC/ESP present */
#define PASAHO_HDR_BITMASK_AH          PASAHO_HDR_BITMASK_AH_GEN2           /**< IPSEC/AH present */
#define PASAHO_HDR_BITMASK_SCTP        PASAHO_HDR_BITMASK_SCTP_GEN2         /**< SCTP present */
#define PASAHO_HDR_BITMASK_UDP         PASAHO_HDR_BITMASK_UDP_GEN2          /**< UDP present */
#define PASAHO_HDR_BITMASK_UDPLITE     PASAHO_HDR_BITMASK_UDPLITE_GEN2      /**< UDPLITE present */
#define PASAHO_HDR_BITMASK_TCP         PASAHO_HDR_BITMASK_TCP_GEN2          /**< TCP present */
#define PASAHO_HDR_BITMASK_GTPU        PASAHO_HDR_BITMASK_GTPU_GEN2         /**< GTPU present */
#define PASAHO_HDR_BITMASK_CUSTOM      PASAHO_HDR_BITMASK_CUSTOM_GEN2       /**< Custom header present */
#define PASAHO_HDR_BITMASK_IPSEC_NAT_T PASAHO_HDR_BITMASK_IPSEC_NAT_T_GEN2  /**< IPSEC NAT-T present */
#else
#define PASAHO_HDR_BITMASK_MAC         PASAHO_HDR_BITMASK_MAC_GEN1          /**< MAC present */
#define PASAHO_HDR_BITMASK_VLAN        PASAHO_HDR_BITMASK_VLAN_GEN1         /**< VLAN present */
#define PASAHO_HDR_BITMASK_MPLS        PASAHO_HDR_BITMASK_MPLS_GEN1         /**< MPLS present */
#define PASAHO_HDR_BITMASK_IP          PASAHO_HDR_BITMASK_IP_GEN1           /**< IP present */
#define PASAHO_HDR_BITMASK_ESP         PASAHO_HDR_BITMASK_ESP_GEN1          /**< IPSEC/ESP present */
#define PASAHO_HDR_BITMASK_AH          PASAHO_HDR_BITMASK_AH_GEN1           /**< IPSEC/AH present */
#define PASAHO_HDR_BITMASK_UDP         PASAHO_HDR_BITMASK_UDP_GEN1          /**< UDP present */
#define PASAHO_HDR_BITMASK_UDPLITE     PASAHO_HDR_BITMASK_UDPLITE_GEN1      /**< UDPLITE present */
#define PASAHO_HDR_BITMASK_TCP         PASAHO_HDR_BITMASK_TCP_GEN1          /**< TCP present */
#define PASAHO_HDR_BITMASK_GTPU        PASAHO_HDR_BITMASK_GTPU_GEN1         /**< GTPU present */
#define PASAHO_HDR_BITMASK_CUSTOM      PASAHO_HDR_BITMASK_CUSTOM_GEN1       /**< Custom header present */

#define PASAHO_HDR_BITMASK2_SCTP        PASAHO_HDR_BITMASK2_SCTP_GEN1       /**< SCTP present */
#define PASAHO_HDR_BITMASK2_IPSEC_NAT_T PASAHO_HDR_BITMASK2_IPSEC_NAT_T_GEN1/**< IPSEC NAT-T present */
#define PASAHO_HDR_BITMASK2_PPPoE       PASAHO_HDR_BITMASK2_PPPoE_GEN1      /**< PPPoE present */
#define PASAHO_HDR_BITMASK2_802_3       PASAHO_HDR_BITMASK2_802_3_GEN1      /**< 802.3 present */
#endif

/*@}*/
/** @} */




/* Next Route command */
/**
 *  @defgroup pasahoNrDestGen1  PA/SA/HO Destination Types at NextRoute Command of First Generation PASS
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PA/SA/HO Destination Types at NextRoute Command of First Generation PASS
 *  Bitmap definition of the destination type at the nextRoute command of First Generation PASS. 
 */ 
/*@{*/
#define PASAHO_NR_DEST_PKTDMA_GEN1      6            /**< NextRoute Destination: PKTDMA (PASS Gen1)*/
#define PASAHO_NR_DEST_ETH_GEN1         7            /**< NextRoute Destination: Ethernet Port (PASS Gen1)*/
#define PASAHO_NR_DEST_SRIO_GEN1        0            /**< NextRoute Destination: SRIO (PASS Gen1)*/

/*@}*/
/** @} */

/**
 *  @defgroup pasahoNrDestGen2  PA/SA/HO Destination Types at NextRoute Command of Second Generation PASS
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PA/SA/HO Destination Types at NextRoute Command of Second Generation PASS
 *  Bitmap definition of the destination type at the nextRoute command of Second Generation PASS. 
 */ 
/*@{*/
#define PASAHO_NR_DEST_PKTDMA_GEN2      0            /**< NextRoute Destination: PKTDMA (PASS Gen2)*/
#define PASAHO_NR_DEST_ETH_GEN2         2            /**< NextRoute Destination: Ethernet Port (PASS Gen2)*/
#define PASAHO_NR_DEST_SRIO_GEN2        7            /**< NextRoute Destination: SRIO (PASS Gen2)*/

/*@}*/
/** @} */

/**
 *  @defgroup pasahoNrDest  PA/SA/HO Destination Types at NextRoute Command
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PA/SA/HO Destination Types at NextRoute Command
 *  Bitmap definition of the destination type at the nextRoute command
 */ 
#ifdef NSS_GEN2
#define PASAHO_NR_DEST_PKTDMA      PASAHO_NR_DEST_PKTDMA_GEN2   /**< NextRoute Destination: PKTDMA */
#define PASAHO_NR_DEST_ETH         PASAHO_NR_DEST_ETH_GEN2      /**< NextRoute Destination: Ethernet Port */
#define PASAHO_NR_DEST_SRIO        PASAHO_NR_DEST_SRIO_GEN2     /**< NextRoute Destination: SRIO */
#else
#define PASAHO_NR_DEST_PKTDMA      PASAHO_NR_DEST_PKTDMA_GEN1   /**< NextRoute Destination: PKTDMA */
#define PASAHO_NR_DEST_ETH         PASAHO_NR_DEST_ETH_GEN1      /**< NextRoute Destination: Ethernet Port */
#define PASAHO_NR_DEST_SRIO        PASAHO_NR_DEST_SRIO_GEN1     /**< NextRoute Destination: SRIO */
#endif

/*@}*/
/** @} */

/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoNextRoute_t defines the next route command. The command structure is defined as 32 bit
 *          values to work with the hardware regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoNextRoute_s  {
    uint32_t  word0;          /**< Contains the next route command information. @ref PASAHO_next_route_command_macros */
    uint32_t  swInfo0;        /**< Information placed into returned descriptor. Used if next destination is the host */
    uint32_t  swInfo1;        /**< Information placed into returned descriptor. Used if next destination is the host */
    uint32_t  word1;          /**< Contains the optional information such as pktType for SRIO. @ref PASAHO_next_route_command_macros */
} pasahoNextRoute_t;

/** 
 *  @defgroup PASAHO_next_route_command_macros  PASAHO Next Route Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Next Route Command Macros
 *  Macros used by the PASAHO Next Route Command
 *  
 */
/*@{*/

#define PASAHO_SET_N(x,v)       PASAHO_SET_BITFIELD((x)->word0, (v), 28, 1)    /**< Sets the N bit which indicates the next command should be executed prior to the route command */
#define PASAHO_SET_E(x,v)       PASAHO_SET_BITFIELD((x)->word0, (v), 27, 1)    /**< Sets the E bit which indicates the extened parameters (packet type and/or control flags) are present for SRIO */
#define PASAHO_SET_DEST(x,v)    PASAHO_SET_BITFIELD((x)->word0, (v), 24, 3)    /**< Sets the destination of the route defined at @ref pktDest */
#define PASAHO_SET_FLOW(x,v)    PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)    /**< Specifies the flow to use for packets sent to the host */
#define PASAHO_SET_QUEUE(x,v)   PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16)   /**< Specifies the queue to use for packets send to the host */
#define PASAHO_SET_PKTTYPE(x,v) PASAHO_SET_BITFIELD((x)->word1, (v), 24, 8)    /**< Specifies the packet type to use for packets send to the SRIO */
#define PASAHO_SET_TX_PADDING(x,v) PASAHO_SET_BITFIELD((x)->word1, (v), 0, 1)  /**< Sets the tx padding bit which indicates PASS should check padding condition and provide L2 zero padding if required */
#define PASAHO_SET_TX_STATS(x,v) PASAHO_SET_BITFIELD((x)->word1, (v), 1, 1)    /**< Sets the tx stats bit which indicates PASS should increment the user statistics chain pointed by the user statistics index */
#define PASAHO_SET_RPT_TX_TIMESTAMP(x,v) PASAHO_SET_BITFIELD((x)->word1, (v), 2, 1)  /**< Sets the report tx timestamp bit which indicates swInfo0 is required to configure CPTS Tx timestamp report */
#define PASAHO_SET_USR_STATS_INDEX(x,v) PASAHO_SET_BITFIELD((x)->word1, (v), 8, 16) /**< Specifies index of the first user-defined statistics to be updated */

/*@}*/ /* PASAHO_next_route_command_macros */
/** @}*/ /* @name PASAHO Next Route Command Macros */


/* Compute checksum command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoComChkCrc_t defines the checksum and CRC generation command. The command structure is defined as
 *          32 bit values to wrok with the hardware regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoComChkCrc_s  {
    uint32_t  word0;        /**<  @ref PASAHO_chksum_command_macros */
    uint32_t  word1;        /**<  @ref PASAHO_chksum_command_macros */
    uint32_t  word2;        /**<  @ref PASAHO_chksum_command_macros */

} pasahoComChkCrc_t;

/** 
 *  @defgroup PASAHO_chksum_command_macros  PASAHO Checksum/CRC Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Checksum/CRC Command Macros
 *  Macros used by the ASAHO Checksum/CRC Command 
 */
/*@{*/

#define PASAHO_CHKCRC_SET_NEG0(x,v)        PASAHO_SET_BITFIELD((x)->word0, (v), 23, 1)           /**< Sets the negative 0 flag - if set a checksum computed as 0 will be sent as 0xffff */
#define PASAHO_CHKCRC_SET_CTRL(x,v)        PASAHO_SET_BITFIELD((x)->word0, (v), 16, 4)           /**< Sets the optional flags of the CRC/Checksum command */
#define PASAHO_CHKCRC_SET_CRCSIZE(x,v)     PASAHO_SET_BITFIELD((x)->word0, (v), 8,  8)           /**< Sets the size of the crc in bytes (PASS Gen2 only) */
#define PASAHO_CHKCRC_SET_START(x,v)       PASAHO_SET_BITFIELD((x)->word0, (v), 0,  8)           /**< Sets the start offset of the checksum/crc */
#define PASAHO_CHKCRC_SET_LEN(x,v)         PASAHO_SET_BITFIELD((x)->word1, (v), 16, 16)          /**< Sets the length of the checksum/crc */
#define PASAHO_CHKCRC_SET_RESULT_OFF(x,v)  PASAHO_SET_BITFIELD((x)->word1, (v), 0,  16)          /**< Sets the offset to where to paste the checksum/crc into the packet */
#define PASAHO_CHKCRC_SET_INITVAL(x,v)     PASAHO_SET_BITFIELD((x)->word2, (v), 16, 16)          /**< Sets the initial value of the 16-bit checksum */
#define PASAHO_CHKCRC_SET_INITVAL32(x,v)   (x)->word2 = (v)                                      /**< Sets the initial value of the 32-bit crc (PASS Gen2 only)*/

/*@}*/ /* @name PASAHO Checksum/CRC Command Macros */
/** @}*/ /* PASAHO_chksum_command_macros */


/* Blind patch command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoComBlindPatch_t defines the blind patch command. The command structure is defined as
 *          32 bit values to work with the hardware regardless of the device endianness.
 *
 *  @details 
 */

#define PASAHO_BPATCH_MAX_PATCH_WORDS   4
typedef struct pasahoComBlindPatch_s  {
    uint32_t   word0;                                   /**<  @ref PASAHO_blind_patch_command_macros */
    uint32_t   patch[PASAHO_BPATCH_MAX_PATCH_WORDS];    /**<  @ref PASAHO_blind_patch_command_macros */
    
} pasahoComBlindPatch_t;

/** 
 *  @defgroup PASAHO_blind_patch_command_macros  PASAHO Blind Patch Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Blind Patch Command Macros
 *  Macros used by the PASAHO Blind Patch Command
 */

#define PASAHO_BPATCH_SET_PATCH_NBYTES(x,v)    \
                          PASAHO_SET_BITFIELD((x)->word0, v, 24,  5)
/**< Sets the number of bytes to patch */
                          
#define PASAHO_BPATCH_SET_PATCH_CMDSIZE(x,v)   \
                          PASAHO_SET_BITFIELD((x)->word0, v, 20, 4)
/**< Sets the size of the command in 32 bit word units */
                          
#define PASAHO_BPATCH_SET_OVERWRITE(x,v)       \
                          PASAHO_SET_BITFIELD((x)->word0, v, 19, 1)
/**< Sets the overwrite flag. If set the patch will overwrite existing packet data, 
     otherwise data may be inserted */  
     
#define PASAHO_BPATCH_SET_DELETE(x,v)         \
                          PASAHO_SET_BITFIELD((x)->word0, v, 18, 1)
/**< Sets the delete flag. If set, no data will be inserted */                          
                             
                          
#define PASAHO_BPATCH_SET_OFFSET(x,v)         \
                          PASAHO_SET_BITFIELD((x)->word0, v, 0,  16)
/**< Sets the offset to the start of the patch */                          
                          
#define PASAHO_BPATCH_SET_PATCH_BYTE(x, byteNum, byte)  \
                                PASAHO_SET_BITFIELD((x)->patch[(byteNum) >> 2], byte, ((3 - (byteNum & 0x3)) << 3), 8)
/**< Sets the data to patch */                                                   
                                
/*@}*/ /* @name PASAHO Blind Patch Command Macros */
/** @}*/ /* PASAHO_blind_patch_command_macros */        

/* Report Timestamp command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoReportTimestamp_t defines the report timestamp command. The command structure is defined as 32 bit
 *          values to work with the hardware regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoReportTimestamp_s  {
    uint32_t  word0;          /**< Contains the report timestamp command information. @ref PASAHO_report_timestamp_command_macros */
    uint32_t  swInfo0;        /**< Information placed into returned descriptor of the reporting packet.  */
} pasahoReportTimestamp_t;

/** 
 *  @defgroup PASAHO_report_timestamp_command_macros PASAHO Report Timestamp Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Report Timestamp Command Macros
 *  Macros used by the PASAHO Report Timestamp Command
 *  
 */
/*@{*/
#define PASAHO_SET_REPORT_FLOW(x,v)    PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)    /**< Specifies the flow to use for report packets sent to the host */
#define PASAHO_SET_REPORT_QUEUE(x,v)   PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16)   /**< Specifies the queue to use for report packets send to the host */

/*@}*/ /* @name PASAHO Report Timestamp Command Macros */
/** @}*/ /* PASAHO_report_timestamp_command_macros */        

/* IP Fragmentation command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoIpFrag_t defines the IP Fragmentation command. The command structure is defined as 32 bit
 *          values to work with the hardware regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoIpFrag_s  {
    uint32_t  word0;          /**< Contains the ip fragmentation command information. @ref PASAHO_ip_frag_command_macros */
} pasahoIpFrag_t;

/** 
 *  @defgroup PASAHO_ip_frag_command_macros  PASAHO IP Fragmentation Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO IP Fragmentation Command Macros
 *  Macros used by the PASAHO IP Fragmentation Command
 *  
 */
/*@{*/

#define PASAHO_SET_SUB_CODE_IP_FRAG(x) PASAHO_SET_BITFIELD((x)->word0, PASAHO_SUB_CMD_IP_FRAG, 24, 5)    /**< Set sub-command code to indicate IP Fragmentation command */
#define PASAHO_SET_SUB_CODE(x,v)  PASAHO_SET_BITFIELD((x)->word0, (v), 24, 5)  /**< Specifies the sub-command code */
#define PASAHO_SET_IP_OFFSET(x,v) PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)  /**< Specifies the offset to the IP header to be fragmented */
#define PASAHO_SET_MTU_SIZE(x,v)  PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16) /**< Specifies the MTU size */

/*@}*/ /* @name PASAHO IP Fragmentation Command Macros */
/** @}*/ /* PASAHO_ip_frag_command_macros */


/* Patch Message Length command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoPatchMsgLen_t defines the message length patching command. The command structure is defined as 32 bit
 *          values to work with the hardware regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoPatchMsgLen_s  {
    uint32_t  word0;          /**< Contains the message length patching command information. @ref PASAHO_patch_msg_len_command_macros */
} pasahoPatchMsgLen_t;

/** 
 *  @defgroup PASAHO_patch_msg_len_command_macros  PASAHO Message Length Patching Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Message Length Patching Command Macros
 *  Macros used by the PASAHO Message Length Patching Command
 *  
 */
/*@{*/

#define PASAHO_SET_SUB_CODE_PATCH_MSG_LEN(x) PASAHO_SET_BITFIELD((x)->word0, PASAHO_SUB_CMD_PATCH_MSG_LEN, 24, 5)    /**< Set sub-command code to indicate Message Length Patching command */
#define PASAHO_SET_MSGLEN_OFFSET(x,v) PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)  /**< Specifies the offset to the message length field to be patched */
#define PASAHO_SET_MSGLEN_SIZE(x,v) PASAHO_SET_BITFIELD((x)->word0, (v), 15,  1)   /**< Specifies the size of the length field (0: 16-bit; 1: 32-bit) */
#define PASAHO_SET_MSGLEN(x,v)  PASAHO_SET_BITFIELD((x)->word0, (v), 0,  15)       /**< Specifies the message length excluding the IP header and payload length */

/*@}*/ /* @name PASAHO Message Length Patching Command Macros */
/** @}*/ /* PASAHO_patch_msg_len_command_macros */

/* Compute checksum command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoEfOp_t defines the egress flow operation command which is used to instruct PASS to 
 *          perform optional flow cache lookup and egress packet modification according to the associated
 *          egress flow records. The command structure is defined as 32 bit values to wrok with the hardware 
 *          regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoEfOp_s  {
    uint32_t  word0;        /**<  @ref PASAHO_ef_op_command_macros */
    uint32_t  word1;        /**<  @ref PASAHO_ef_op_command_macros */
    uint32_t  word2;        /**<  @ref PASAHO_ef_op_command_macros */
    uint32_t  word3;        /**<  @ref PASAHO_ef_op_command_macros */
    uint32_t  word4;        /**<  @ref PASAHO_ef_op_command_macros */
    uint32_t  word5;        /**<  @ref PASAHO_ef_op_command_macros */

} pasahoComEfOp_t;

/**
 *  @defgroup pasahoEfOpCtrlBitmap  PA/SA/HO Egress Flow Opertaion Control Bitmap Bit Definitions
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PA/SA/HO Egress Flow Opertaion Control Bitmap Bit Definitions
 *  Bitmap definition of the control bitmap at the Egress Flow operation info. 
 */ 
/*@{*/

#define PASAHO_HDR_EF_OP_CTRL_FC           (1 << 15)    /**< Enable Flow Cache lookup */
#define PASAHO_HDR_EF_OP_CTRL_LVL4_REC     (1 << 7)     /**< Level 4 record prespent */
#define PASAHO_HDR_EF_OP_CTRL_LVL3_REC     (1 << 6)     /**< Level 3 record prespent */
#define PASAHO_HDR_EF_OP_CTRL_LVL2_REC     (1 << 5)     /**< Level 2 record prespent */
#define PASAHO_HDR_EF_OP_CTRL_LVL1_REC     (1 << 4)     /**< Level 1 record prespent */

/*@}*/
/** @} */


/** 
 *  @defgroup PASAHO_ef_op_command_macros  PASAHO Egress Flow Operation Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Egress Flow Operation Command Macros
 *  Macros used by the PASAHO Egress Flow Operation Command 
 */
/*@{*/
#define PASAHO_EF_OP_SET_CTRL(x,v)          PASAHO_SET_BITFIELD((x)->word0,(v), 8, 16)           /**< Sets the control flags of the Flow Cache operation command */
#define PASAHO_EF_OP_SET_CTRL_FC(x,v)       PASAHO_SET_BITFIELD((x)->word0,(v), 23, 1)           /**< Enable/Disable Flow Cache lookup */
#define PASAHO_EF_OP_SET_CTRL_LVL4(x,v)     PASAHO_SET_BITFIELD((x)->word0,(v), 15, 1)           /**< Enable/Disable Level 4 Egress Flow record */
#define PASAHO_EF_OP_SET_CTRL_LVL3(x,v)     PASAHO_SET_BITFIELD((x)->word0,(v), 14, 1)           /**< Enable/Disable Level 3 Egress Flow record */
#define PASAHO_EF_OP_SET_CTRL_LVL2(x,v)     PASAHO_SET_BITFIELD((x)->word0,(v), 13, 1)           /**< Enable/Disable Level 2 Egress Flow record */
#define PASAHO_EF_OP_SET_CTRL_LVL1(x,v)     PASAHO_SET_BITFIELD((x)->word0,(v), 12, 1)           /**< Enable/Disable Level 1 Egress Flow record */

#define PASAHO_EF_OP_SET_L2_OFFSET(x, v)    PASAHO_SET_BITFIELD((x)->word1,(v),8,8)              /**< Set the l2 offset */
#define PASAHO_EF_OP_SET_L3_OFFSET2(x, v)   PASAHO_SET_BITFIELD((x)->word1,(v),0,8)              /**< Set the l3 offset for inner or fisrt IP */
#define PASAHO_EF_OP_SET_L3_OFFSET(x, v)    PASAHO_SET_BITFIELD((x)->word2,(v),24,8)             /**< Set the l3 offset for outer or fisrt IP */
#define PASAHO_EF_OP_SET_IPSEC_OFFSET(x, v) PASAHO_SET_BITFIELD((x)->word2,(v),0, 8)             /**< Set the IPSEC offset if the IPSEC header exists in the packet */
#define PASAHO_EF_OP_SET_END_OFFSET(x, v)   PASAHO_SET_BITFIELD((x)->word1,(v),16,16)            /**< Set the end offset as end of L4 (UDP/UDPLite/TCP) payload */

#define PASAHO_EF_OP_SET_LVL1_REC(x, v)     PASAHO_SET_BITFIELD((x)->word3,(v),24, 8)            /**< Set the index of Egress Flow level 1 record */
#define PASAHO_EF_OP_SET_LVL2_REC(x, v)     PASAHO_SET_BITFIELD((x)->word3,(v),16, 8)            /**< Set the index of Egress Flow level 2 record */
#define PASAHO_EF_OP_SET_LVL3_REC(x, v)     PASAHO_SET_BITFIELD((x)->word3,(v), 8, 8)            /**< Set the index of Egress Flow level 3 record */
#define PASAHO_EF_OP_SET_LVL4_REC(x, v)     PASAHO_SET_BITFIELD((x)->word3,(v), 0, 8)            /**< Set the index of Egress Flow level 4 record */

/*@}*/
/** @} */


/* EMAC CRC Verify command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoEmacCrcVerify_t defines the Ethernet CRC Verify command. The command structure is defined as 32 bit
 *          values to work with the hardware regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoEmacCrcVerify_s  {
    uint32_t  word0;          /**< Contains the Ethernet CRC Verify command information. @ref PASAHO_emac_crc_verify_command_macros */
} pasahoEmacCrcVerify_t;

/** 
 *  @defgroup PASAHO_emac_crc_verify_command_macros  PASAHO EMAC CRC Verify Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO EMAC CRC Verify Command Macros
 *  Macros used by the PASAHO EMAC CRC Verify Command
 *  
 */
/*@{*/

#define PASAHO_SET_SUB_CODE_EMAC_CRC_VERIFY(x) PASAHO_SET_BITFIELD((x)->word0, PASAHO_SUB_CMD_EMAC_CRC_VERIFY, 24, 5)    /**< Set sub-command code to indicate EMAC CRC Verify command */
#define PASAHO_SET_EMACPORT(x,v) PASAHO_SET_BITFIELD((x)->word0, (v), 20, 4)    /**< Specifies the one-based destination EMAC port number 
                                                                                     where 0 indicates standard ethernet switch forwarding */

/*@}*/ /* @name PASAHO EMAC CRC Verify Command Macros */
/** @}*/ /* PASAHO_emac_crc_verify_command_macros */

/* Insert Message time command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoInsMsgTime_t defines the message time insert command. The command structure is defined as 32 bit
 *          values to work with the hardware regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoInsMsgTime_s  {
    uint32_t  word0;          /**< Contains the message time patching command information. @ref PASAHO_insert_msg_time_command_macros */
} pasahoInsMsgTime_t;

/** 
 *  @defgroup PASAHO_insert_msg_time_command_macros  PASAHO Message Time Insert Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Message Time Insert Command Macros
 *  Macros used by the PASAHO Message Time Insert Command
 *  
 */
/*@{*/

#define PASAHO_SET_SUB_CODE_INS_MSG_TIME(x) PASAHO_SET_BITFIELD((x)->word0, PASAHO_SUB_CMD_INS_TIME, 24, 5)    /**< Set sub-command code to indicate Message Time Insert command */
#define PASAHO_SET_INS_OFFSET_MSG_TIME(x,offset)   PASAHO_SET_BITFIELD((x)->word0, offset, 0, 16)             /**< Set the offset from start of packet to insert the 8 byte time */

/*@}*/ /* @name PASAHO Message Time Insert Command Macros */
/** @}*/ /* PASAHO_insert_msg_time_command_macros */

/* Insert Message Count command */
/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoInsMsgCount_t defines the message count insert command. The command structure is defined as 32 bit
 *          values to work with the hardware regardless of the device endianness.
 *
 *  @details 
 */

typedef struct pasahoInsMsgCount_s  {
    uint32_t  word0;          /**< Contains the message count patching command information. @ref PASAHO_insert_msg_count_command_macros */
    uint32_t  word1;          /**< Contains the message count patching command information. @ref PASAHO_insert_msg_count_command_macros */
} pasahoInsMsgCount_t;

/** 
 *  @defgroup PASAHO_insert_msg_count_command_macros  PASAHO Message Count Insert Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Message Count Insert Command Macros
 *  Macros used by the PASAHO Message Count Insert Command
 *  
 */
/*@{*/

#define PASAHO_SET_SUB_CODE_INS_MSG_COUNT(x)        PASAHO_SET_BITFIELD((x)->word0, PASAHO_SUB_CMD_INS_COUNT, 24, 5)    /**< Set sub-command code to indicate Message Count Insert command */
#define PASAHO_SET_INS_OFFSET_MSG_COUNT(x,offset)   PASAHO_SET_BITFIELD((x)->word0, offset, 0, 16)                      /**< Set the offset from start of packet to insert the 4 byte count */
#define PASAHO_SET_COUNTER_INDEX_FOR_MSG_COUNT(x,offset)   PASAHO_SET_BITFIELD((x)->word1, offset, 16, 16)              /**< Set the counter index to read from, to insert the 4 byte count */

/*@}*/ /* @name PASAHO Message Count Insert Command Macros */
/** @}*/ /* PASAHO_insert_msg_count_command_macros */


#ifdef __cplusplus
}
#endif

#endif  /* _PASAHO_H */
