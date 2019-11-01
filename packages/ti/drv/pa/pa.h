#ifndef _PA_H
#define _PA_H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <stdint.h>
#include <stdlib.h>

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
 * expectation is all these catch up with some other intelligent 
 * tools like coverity, Klocwork etc, instead of dump GNU 
 * warnings
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif


#include <ti/drv/pa/paver.h>

/* ============================================================= */
/**
 *   @file  pa.h
 *
 *   path  ti/drv/pa/pa.h
 *
 *   @brief  Packet Accelerator (PA) sub-system LLD API and Data Definitions
 *
 *  ============================================================================
 *  Copyright (c) Texas Instruments Incorporated 2009-2014
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
 
/**  @mainpage Packet Accelerator Low Level Driver
 *
 *   \image html doxydoc.wmf
 *
 *   @section intro  Introduction
 *
 *   The packet accelerator sub-system (PASS) is designed to provide the input packet classification, checksum/CRC 
 *   verification and generation, data manipulation and etc. The first generation PASS consists of the following 
 *   resources
 *      - Six PDSPs for packet and command processing
 *      - Three 64-entry LUT1 (connected to PDSP0, PDSP1 and PDSP2) for Layer 2/3 or custom LUT1 lookup
 *      - One 8192-entry LUT2 (connected to PDSP3) for Layer 4/5 or custom LUT2 lookup
 *      - Six programmable CRC engines (connected to each PDSP respectively) for CRC computation and verification
 *      - Six 16-bit general purpose timers
 *
 *   The packet accelerator low level driver (PA LLD) provides configuration and control of the packet accelerator
 *   sub-system (PASS). The sub-system provides from network packet classification and routing based on  
 *   network header information (see @ref netlayers). The packet accelerator low level driver module 
 *   (referred to as the module) provides APIs to configure the criteria used for from-network packet
 *   routing.
 *
 *   The module attempts to abstract the operation of the PASS from the application. The module uses the following rules 
 *   when configuring the PASS:
 *      - All received packets from Ethernet and/or SRIO are routed to PDSP0
 *      - PDSP0 does L0-L2 (MAC/SRIO) lookup using LUT1-0. If the packet is IP, it is forwarded to PDSP1
 *      - PDSP1 does the outer IP or Custom LUT1 lookup using LUT1-1
 *      - PDSP2 does any subsequent IP or Custom LUT1 lookup using LUT1-2
 *      - PDSP3 does all TCP/UDP and Custom LUT2 lookup using LUT2
 *      - PDSP4 is used for post-lookup processes such as checksum/CRC result verification.
 *      - PDSP4/5 can be used for pre-transmission operation such as transmit checksum generation.
 *
 *   With the exception of some initial setup functions, the module does not communicate directly with
 *   the sub-system. The output of the module is a formatted data block along with a destination address.
 *   The module user must send the formatted data to the sub-system. This is typically done by linking the
 *   created data block to a host packet descriptor, and then using the addressing information to send
 *   the created packet to the sub-system through the queue manager and PKTDMA.
 *
 *   For packets to the network, the sub-system provides ones complement checksum or CRC generation over
 *   a range provided by the module user. The range is not determined by sub-system by parsing the
 *   to-network packet, since it is assumed that the creator of the packet already has the start offset,
 *   length, initial checksum value and etc.
 *
 *   The low level driver maintains two tables of layer 2 and layer 3 configuration information. The memory 
 *   for these tables is provided by the module user at run time. The module maintains ownership of these 
 *   tables and the module user must not write to the memory once provided to the module.
 *
 *   In multi-core devices the module can be used in two different configurations. In independent core
 *   mode each core in a device has a unique set of tables. Although it is legal for any core to
 *   reference handles from other cores, this is not typically done. In this case cache coherency and
 *   cross core semaphores are not implemented by the module user. In common core mode there is only
 *   one set of tables and they are shared by all cores. Each core that uses the module must initialize
 *   it, but each core will provide the exact same buffers to the module. The module user will have
 *   the first core to initialize the module also initialize the table. Other cores will initialize their
 *   internal state but not initialize the table. In this mode @ref cache coherency and cross core @ref semaphores
 *   must be implemented by the module user to insure the integrity of the tables.
 *
 *   The second generation of the packet accelerator sub-system (PASS) of the new Keystone2 device is enhanced to
 *   support fully-offloaded fast-path operations in both ingress and egress directions. The second generation PASS 
 *   provides the following functionalities:
 *      - Ethernet and SRIO packet classification
 *      - Stateless L3/L4 Firewall (ACL)
 *      - Outer and Inner IP packet classification
 *      - Ethernet OAM classification
 *      - Outer and inner IP reassembly
 *      - TCP/UDP/GTPU based LUT2 classification
 *      - IPv4 and TCP/UDP checksum generation and verification
 *      - SCTP or custom CRC generation and verification
 *      - Programmable system statistics
 *      - Post-Classification operation such as packet patch, protocol header and/or trailer removal
 *      - Egress or forwarding traffic flow cache operations
 *          - Inner IP L3/L4 patching
 *          - Inner IP fragmentation
 *          - Outer IP insertion/update 
 *          - Pre-IPSEC and Post-IPSEC processing
 *          - Outer IP fragmentation
 *          - L2 header insertion/update
 *
 *   The second generation PASS consists of five ingress stages (Ingress0-4), a post-processing stage (Post) 
 *   and three egress stages (Egress 0-2). Each stage has its intended function, which is described briefly in 
 *   the sub-sections below.  Ingress packets (from the Ethernet Switch through PA to the host) are expected to 
 *   follow the flow Ingress 0 -> Ingress 1 -> Ingress 2 -> Ingress 3-> Ingress 4 -> Post -> Host.  Egress packets 
 *   (from the host through PA out the switch) are expected to follow the flow Egress 0 -> Egress 1 -> Egress 2 -> 
 *   Ethernet Switch.  Ingress packets can be directly routed to egress path without host intervention. The packets 
 *   can also be routed between PASS and SASS (Security Accelerator sub-system) multiple times to perform encryption,  
 *   decryption and authentication operation. 
 *      - Ingress 0 (2 PDSPs and 2 256-entry LUT1 engines): 
 *         - PDSP0 and LUT1_0: SRIO/MAC header parsing and classification
 *         - PDSP1 and LUT1_1 (normal mode): L3/L4 header parsing and pre-IPSEC firewall (ACL) lookup
 *         - PDSP1 and LUT1_1 (Eoam mode): L2 header parsing and Ethernet OAM lookup.
 *      - Ingress 1 (2 PDSPs and 2 256-entry LUT1 engines):
 *        - PDSP0 and LUT1_0: Outer IP or custom header parsing and classification
 *        - PDSP1 and LUT1_1: IPSEC NAT-T detection, IPSEC header parsing and classification
 *      - Ingress 2 (1 PDSP and 1 256-entry LUT1 engine):
 *        - PDSP0 and LUT1_0: 2nd IPSEC Header parsing and classification
 *      - Ingress 3 (1 PDSP and 1 256-entry LUT1 engine):
 *        - PDSP0 and LUT1_0: L3/L4 hearer parsing and post-IPSEC firewall (ACL) lookup
 *      - Ingress 4 (2 PDSPs, 1 256-entry LUT1 engine and 1 3000-entry LUT2 engine)
 *        - PDSP0 and LUT1_0: Inner IP or custom header parsing and classification
 *        - PDSP1 and LUT2: TCP/UDP/GTPU/Custom header parsing and LUT2 lookup
 *      - Post (2 PDSPs): Post-classification processing
 *      - Egress 0 (3 PDSPs and 1 256-entry LUT1 engine)
 *        - PDSP0: and LUT1_0: Inner L3/L4 header parsing and Flow cache lookup
 *        - PDSP1: Inner L3/L4 Header update and Tx command processing
 *        - PDSP2: Outer IP insertion/update, IPSEC pre-processing, inner IP fragmentation and Tx command processing
 *      - Egress 1 (1 PDSP): NAT-T header insertion or IPSEC pre-processing
 *      - Egress 2 (1 PDSP)  L2 header insertion/update and outer IP fragmentation
 *  
 *  The second generation PASS also provides a Reassembly engine (RA) which can be connected from Ingress 0 and Ingress 3
 *  stage to perform outer and inner IP reassembly and the reassembled packets will be delivered to Ingress 1 and Ingress 4
 *  stage respectively. Besides, there is a programmable statistics engine which is used to provide PASS system statistics, 
 *  ACL and Flow cache pre-entry statistics and user-defined statistics.
 *
 *  To maintain backward compatibility, the second generation PASS LLD maintains the same APIs of the first generation LLD.
 *  New APIs are added for the new features such as ACL, Flow Cache and etc only.
 *
 */
 
/* Define PALLD Module as a master group in Doxygen format and add all PA LLD API 
   definitions to this group. */
/** @defgroup palld_module PA LLD Module API
 *  @{
 */
/** @} */
   
/** @defgroup palld_api_functions PA LLD Functions
 *  @ingroup palld_module
 */
 
/** @defgroup palld_api_macros PA LLD Macros
 *  @ingroup palld_module
 */

/** @defgroup palld_api_structures PA LLD Data Structures
 *  @ingroup palld_module
 */

/** @defgroup palld_api_constants PA LLD Constants (enum's and define's)
 *  @ingroup palld_module
 */
 
/**
 *  @def  pa_PARAMS_NOT_SPECIFIED
 *        Used for unspecified classification parameters
 */
#define pa_PARAMS_NOT_SPECIFIED                      0xFFFF

/**
 *  @def  pa_LUT_INST_NOT_SPECIFIED
 *        Used if LUT1(or LUT2) instance is not specified
 *        In the case, the PA LLD will decide which LUT instance to use based on the API type and the previous link information 
 */
#define pa_LUT_INST_NOT_SPECIFIED                    -1

/**
 *  @def  pa_LUT1_INDEX_NOT_SPECIFIED
 *        Used if LUT1 index is not specified
 *        In the case, the PASS will use the first available entry 
 */
#define pa_LUT1_INDEX_NOT_SPECIFIED                  -1

/**
 *  @def  pa_MAX_NUM_LUT1_ENTRIES
 *        The maximum number of LUT1 entries
 *
 *  @note These definitions are not used by LLD. They are defined here for reference
 *        purpose only.
 *         
 */
#define pa_MAX_NUM_LUT1_ENTRIES_GEN1                 64
#define pa_MAX_NUM_LUT1_ENTRIES_GEN2                256
#ifndef NSS_GEN2
#define pa_MAX_NUM_LUT1_ENTRIES                      pa_MAX_NUM_LUT1_ENTRIES_GEN1
#else 
#define pa_MAX_NUM_LUT1_ENTRIES                      pa_MAX_NUM_LUT1_ENTRIES_GEN2
#endif

/**
 *  @defgroup  ReturnValues  Function Return Values
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PALLD Function Return Codes
 *
 *  Error codes returned by PALLD API functions.
 */
/*@{*/
/**
 *  @def  pa_OK
 *        PA return code -- Function executed successfully
 */
#define pa_OK	                                 0

/**
 *  @def  pa_ERR_CONFIG
 *        Invalid configuration provided to PA
 */
#define pa_ERR_CONFIG                          -10

/**
 *   @def pa_INSUFFICIENT_CMD_BUFFER_SIZE
 *        The provided buffer was too small to hold the command
 */
#define pa_INSUFFICIENT_CMD_BUFFER_SIZE        -11

/**
 *  @def pa_INVALID_CMD_REPLY_DEST
 *       An invalid destination was provided for command replies
 */
#define pa_INVALID_CMD_REPLY_DEST              -12

/**
 *   @def  pa_DUP_ENTRY
 *         A duplicate active entry was found in the handle table.
 *         If the module user intends to replace the associate routing 
 *         information for the same entry, command packet should be
 *         delivered to the PASS via the PKTDMA sub-system     
 *         Otherwise, module user may decide to drop the command packet and 
 *         free the buffer.
 */ 
#define pa_DUP_ENTRY                           -13

/**
 *   @def  pa_INVALID_DUP_ENTRY
 *         A duplicate pending entry was found in the handle table
 *         This entry can not be handled until the pending entry
 *         becomes active
 */ 
#define pa_INVALID_DUP_ENTRY                   -14

/**
 *   @def  pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT
 *         A more specific entry was found in the handle table
 *
 *   @note: This error is depreciated at the next generation keystone device
 */
#define pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT   -15

/**
 *   @def  pa_INVALID_MPLS_LABEL
 *         An MPLS label exceeded 20 bits
 */
#define pa_INVALID_MPLS_LABEL                  -16

/**
 *   @def  pa_HANDLE_TABLE_FULL
 *         No room for an entry in the L2 table
 */
#define pa_HANDLE_TABLE_FULL                   -17

/**
 *   @def  pa_INVALID_INPUT_HANDLE
 *         Invalid handle provided
 */
#define pa_INVALID_INPUT_HANDLE                -18

/**
 *   @def  pa_HANDLE_INACTIVE
 *         Operation requested on an inactive handle
 */
#define pa_HANDLE_INACTIVE                     -19
 
/**
 *   @def  pa_INVALID_IP_FLOW
 *         A flow label exceeded 20 bits
 */ 
#define  pa_INVALID_IP_FLOW                    -20

/**
 *   @def  pa_WARN_ACTIVE_HANDLE_ACKED
 *         Sub-system reported activation of a handle already marked active
 */
#define pa_WARN_ACTIVE_HANDLE_ACKED            -21

/**
 *   @def  pa_LUT_ENTRY_FAILED
 *         Sub-system could not make an entry to the LUT1 table
 */
#define pa_LUT_ENTRY_FAILED                    -22


/**
 *   @def  pa_RESUBMIT_COMMAND
 *         Sub-system could not handle the command due to memory. Command must be resubmitted
 */
#define pa_RESUBMIT_COMMAND                    -23

/**
 *   @def  pa_SYSTEM_STATE_INVALID
 *         Tried to download an image to a running PDSP
 */
#define pa_SYSTEM_STATE_INVALID                -24

/**
 *   @def  pa_INVALID_LUT1_INDEX
 *         LUT1 index exceeds the LUT1 table range
 */
#define pa_INVALID_LUT1_INDEX                  -25

/**
 *   @def  pa_WARN_LNK_CNT_UNSYNC
 *         Warning: Link counter out of sync
 */
#define pa_WARN_LNK_CNT_UNSYNC                 -26

/**
 *   @def pa_CMDSET_TOO_BIG
 *        The total length of commads in the command set exceeds the limit
 */
#define pa_CMDSET_TOO_BIG                      -27

/**
 *   @def  pa_INVALID_LUT_INST
 *         The specified LUT1 or LUT2 instance does not exist
 */
#define pa_INVALID_LUT_INST                    -28

/** 
  *  @def pa_RESOURCE_INIT_DENIED
  *       The resource initialization permission denied 
  */
#define pa_RESOURCE_INIT_DENIED                -29

/** 
  *  @def pa_RESOURCE_USE_DENIED
  *       The resource usage permission denied 
  */
#define pa_RESOURCE_USE_DENIED                 -30


/** 
  *  @def pa_RESOURCE_FREE_DENIED
  *       The resource free permission denied 
  */
#define pa_RESOURCE_FREE_DENIED                -31

/** 
  *  @def pa_FIRMWARE_REVISION_DIFFERENCE
  *       The firmware revision difference 
  */
#define pa_FIRMWARE_REVISION_DIFFERENCE        -32

/** 
  *  @def pa_VIRTUAL_LINK_TABLE_FULL
  *       Virtual link table is full 
  */
#define pa_VIRTUAL_LINK_TABLE_FULL             -33
/**
 *   @def  pa_INVALID_DUP_ACL_ENTRY
 *         A duplicate ACL entry is found in the ACL table
 *         The ACL entry should be deleted before the same 
 *         entry with updated action can be added.  
 *         
 */ 
#define pa_INVALID_DUP_ACL_ENTRY               -34

/**
 *   @def  pa_INVALID_ACL_ACTION
 *         The specified ACL action is not supported
 */
#define pa_INVALID_ACL_ACTION                  -35

/**
 *   @def  pa_INVALID_EF_REC_INDEX
 *         The index of Egress Flow record is out of range 
 */
#define pa_INVALID_EF_REC_INDEX                -36


/**
 *   @def  pa_EF_REC_CONFIG_ERR
 *         Egress Flow record update is rejected by PASS 
 */
#define pa_EF_REC_CONFIG_ERR                   -37

/**
 *   @def  pa_PENDING_FC_ENTRY
 *         A pending Flow Cache entry is intended to be replaced
 *         with another entry by invoking API Pa_addFc() while
 *         it is still pending to be added into PASS LUT1 table. 
 *         This entry can not be replaced until it becomes active
 */ 
#define pa_PENDING_FC_ENTRY                    -38

/** 
  *  @def pa_API_UNSUPPORTED
  *       The API is not supported by this generation of PASS
  */
#define pa_API_UNSUPPORTED                     -39

/** 
  *  @def pa_INVALID_INPUT_POINTER
  *       Some of required input pointers are null
  */
#define pa_INVALID_INPUT_POINTER               -40

/** 
  *  @def pa_ACL_BUSY
  *       PASS ACL is busy and can not accept new operations, please retry after the time
  *       as specified by the @ref Pa_addAcl API
  */
#define pa_ACL_BUSY                            -41

/** 
  *  @def pa_SUB_SYSTEM_BASE_ADDR_NULL
  *       PASS Base address configured in PA LLD is NULL - serious error
  */
#define pa_SUB_SYSTEM_BASE_ADDR_NULL           -42


/** 
  *  @def pa_LUT2_TABLE_FULL
  *       PASS LUT2 Table is full, no more entries can be added unless an
  *       entry is deleted to create a room to a next entry 
  */
#define pa_LUT2_TABLE_FULL                     -43


/*@}*/
/** @} */


/**
 *  @defgroup  cmdMinBufSize Command buffer minimum size requirements
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Command buffer minimum sizes
 *
 *  Define command buffer minimum size requirements.
 */
/* @{ */

/**
 *  @def  pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_addSrio and @ref Pa_addCustomLUT1 function
 */
#define pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES          124

/**
 *  @def  pa_ADD_MAC_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_addMac and @ref Pa_addMac2 function
 */
#define pa_ADD_MAC_MIN_CMD_BUF_SIZE_BYTES           pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES

/**
 *  @def  pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_delHandle function
 */
#define pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES        32

/**
 *  @def  pa_DEL_L4_HANDLE_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_delL4Handle function
 */
#define pa_DEL_L4_HANDLE_MIN_CMD_BUF_SIZE_BYTES     28

/**
 *  @def  pa_ADD_IP_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_addIp and @ref Pa_addIp2 functions
 */
#define pa_ADD_IP_MIN_CMD_BUF_SIZE_BYTES            240 

/**
 *  @def  pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_addCustomLUT2 function
 */
#define pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES          48

/**
 *  @def  pa_ADD_PORT_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_addPort function
 */
#define pa_ADD_PORT_MIN_CMD_BUF_SIZE_BYTES          pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES

/**
 *  @def  pa_CONFIG_EXCEPTION_ROUTE_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_configExceptionRoute and @ref Pa_configEflowExceptionRoute function
 */
#define pa_CONFIG_EXCEPTION_ROUTE_MIN_CMD_BUF_SIZE_BYTES   520

/**
 *  @def  pa_CONFIG_CRC_ENGINE_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_configCrcEngine function
 */
#define pa_CONFIG_CRC_ENGINE_MIN_CMD_BUF_SIZE_BYTES  88

/**
 *  @def  pa_CONFIG_MULTI_ROUTE_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_configMultiRoute function
 */
#define pa_CONFIG_MULTI_ROUTE_MIN_CMD_BUF_SIZE_BYTES 84

/**
 *  @def  pa_SET_CUSTOM_LUT1_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_setCustomLUT1 function
 */
#define pa_SET_CUSTOM_LUT1_MIN_CMD_BUF_SIZE_BYTES    60

/**
 *  @def  pa_SET_CUSTOM_LUT2_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_setCustomLUT2 function
 */
#define pa_SET_CUSTOM_LUT2_MIN_CMD_BUF_SIZE_BYTES    36

/**
 *  @def  pa_CONFIG_CMD_SET_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size allowed when using the @ref Pa_configCmdSet and @ref Pa_formatTxCmd function
 */
#define pa_CONFIG_CMD_SET_MIN_CMD_BUF_SIZE_BYTES     144

/**
 *  @def  pa_REQUEST_STATS_MIN_CMD_BUF_SIZE_BYTES
 *        The minimum command buffer size required when using the @ref Pa_requestStats and @ref Pa_requestUsrStats functions
 */
#define pa_REQUEST_STATS_MIN_CMD_BUF_SIZE_BYTES      24

/**
 *  @def  pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_configUsrStats function with the maximum number of 
 *        user-defined statistics. The size of command packet is calculated as 20 + (number of statistic entries) * 4. 
 */
#define pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES   2068

/**
 *  @def  pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_SYS_CONFIG) function to perform PASS 
 *        global configuration.
 */
#define pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES      108

/**
 *  @def  pa_802_1ag_DET_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_802_1ag_CONFIG) function to configure 
 *        the 802.1ag packet detector.
 */
#define pa_802_1ag_DET_MIN_CMD_BUF_SIZE_BYTES        24

/**
 *  @def  pa_IPSEC_NAT_T_DET_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_IPSEC_NAT_T_CONFIG) function to configure 
 *        the IPSEC NAT-T packet detector.
 */
#define pa_IPSEC_NAT_T_DET_MIN_CMD_BUF_SIZE_BYTES    24

/**
 *  @def  pa_GTPU_CONFIG_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_GTPU_CONFIG) function to configure 
 *        the GTUP classification operation.
 */
#define pa_GTPU_CONFIG_MIN_CMD_BUF_SIZE_BYTES        24

/**
 *  @def  pa_EMAC_PORT_MIRROR_CONFIG_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_EMAC_PORT_CONFIG, pa_EMAC_PORT_CFG_MIRROR) 
 *        function to configure EMAC Mirror operation with the maxmium number of EMAC ports.
 *        The size of command packet is calculated as 24 + (number of configured EMAC ports) * 12.
 *        
 */
#define pa_EMAC_PORT_MIRROR_CONFIG_MIN_CMD_BUF_SIZE_BYTES   120
#define pa_EMAC_PORT_MIRROR_CONFIG_PACKET_SIZE(numPorts)    (24 + (numPorts)*12)

/**
 *  @def  pa_EMAC_PORT_PKT_CAPTURE_CONFIG_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_EMAC_PORT_CONFIG, pa_EMAC_PORT_CFG_PKT_CAPTURE) 
 *        function to configure EMAC packet capture operation with the maxmium number of CPSW ports.
 *        The size of command packet is calculated as 24 + (number of configured CPSW ports) * 12.
 *  
 *  @note CPPI port (CPSW port 0) packet capture is only applicable at the egress direction.
 *        
 */
#define pa_EMAC_PORT_PKT_CAPTURE_CONFIG_MIN_CMD_BUF_SIZE_BYTES   136
#define pa_EMAC_PORT_PKT_CAPTURE_CONFIG_PACKET_SIZE(numPorts)    (24 + (numPorts)*12)

/**
 *  @def  pa_EMAC_PORT_DEFAULT_ROUTE_CONFIG_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_EMAC_PORT_CONFIG, pa_EMAC_PORT_CFG_DEFAULT_ROUTE) 
 *        function to configure EMAC default routes with the maxmium number of EMAC ports.
 *        The size of command packet is calculated as 24 + (number of configured EMAC ports) * 52.
 *        
 */
#define pa_EMAC_PORT_DEFAULT_ROUTE_CONFIG_MIN_CMD_BUF_SIZE_BYTES   440
#define pa_EMAC_PORT_DEFAULT_ROUTE_CONFIG_PACKET_SIZE(numPorts)    (24 + (numPorts)*52)

/**
 *  @def  pa_EMAC_PORT_EQoS_MODE_CONFIG_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_EMAC_PORT_CONFIG, pa_EMAC_PORT_CFG_EQoS_MODE) 
 *        function to configure EMAC default routes with the maxmium number of EMAC ports.
 *        The size of command packet is calculated as 24 + (number of configured EMAC ports) * 152.
 *        
 */
#define pa_EMAC_PORT_EQoS_MODE_CONFIG_MIN_CMD_BUF_SIZE_BYTES   1240
#define pa_EMAC_PORT_EQoS_MODE_CONFIG_PACKET_SIZE(numPorts)    (24 + (numPorts)*152)

/**
 *  @def  pa_EMAC_PORT_CONFIG_MIN_CMD_BUF_SIZE_BYTES
 *        The minmium command buffer size allowed when using the @ref Pa_control (pa_CONTROL_EMAC_PORT_CONFIG) function to configure 
 *        the ethernet port configuration operations with the maxmium number of EMAC EQoS ports.
 *        
 */
#define pa_EMAC_PORT_CONFIG_MIN_CMD_BUF_SIZE_BYTES   pa_EMAC_PORT_EQoS_MODE_CONFIG_MIN_CMD_BUF_SIZE_BYTES

/**
 *  @def  pa_MAX_CMD_BUF_SIZE_BYTES
 *        The maximum command buffer size requested when using any PA API call which generates command packet.
 */
#define pa_MAX_CMD_BUF_SIZE_BYTES                    2068

/* @} */  
/** @} */


/**
 * @ingroup palld_api_structures
 * @brief MAC address specification
 *
 * @details  This type is used to pass MAC addresses (see @ref netlayers) to the module. The most significant byte
 *           of the mac address is placed in array element 0.
 */
#define pa_MAC_ADDR_SIZE       6 
typedef unsigned char paMacAddr_t[pa_MAC_ADDR_SIZE];

/**
 * @ingroup palld_api_structures
 * @brief  IPv4 address specification
 *
 * @details  This type is used to pass IPv4 addresses (see @ref netlayers) to the module. The most significant byte
 *           of the IP address is placed in array element 0.
 */
#define pa_IPV4_ADDR_SIZE      4
typedef unsigned char paIpv4Addr_t[pa_IPV4_ADDR_SIZE];

/**
 * @ingroup palld_api_structures
 * @brief  IPv6 address specificiation
 *
 * @details  This type is used to pass IPv6 addresses (see @ref netlayers) to the module. The most significant byte
 *           of the IP address is placed in array element 0.
 */
#define pa_IPV6_ADDR_SIZE      16 
typedef unsigned char paIpv6Addr_t[pa_IPV6_ADDR_SIZE];

/**
 * @ingroup palld_api_structures
 * @brief  IP address specification
 *
 * @details  This union is used to specify an IP address to the module. The type in the union is determined
 *           through other parameters passed to the module (see @ref IpValues).
 */
typedef union  {

  paIpv6Addr_t  ipv6;   /**< IPv6 address */
  paIpv4Addr_t  ipv4;   /**< IPv4 address */
  
} paIpAddr_t;

/**
 *  @defgroup  IpValues IP types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   IP Values
 *  @brief  Defines the IP version type used.
 *
 *  @details The packet accelerator module parses both IPv4 and IPv6 network layer headers (see @ref netlayers). 
 *           This group is used to distinguish which type of header will be used.
 */
/* @{ */
/**
 *  @def  pa_IPV4
 *        IPv4
 */
#define  pa_IPV4  4

/**
 *  @def   pa_IPV6
 *        IPv6
 */
#define  pa_IPV6  6
  
/*  @}  */  
/** @} */


/**
 * @ingroup palld_api_structures
 * @brief Specification of Pa_Handle 
 *
 * The Pa_Handle is used to identify a PA LLD instance
 */
typedef void*  Pa_Handle;

/**
 * @ingroup palld_api_structures
 * @brief  PA handle specification for L2 and L3 (LUT1) handles
 *
 * @details  This type is used to reference L2 and L3 (LUT1) routing information (see @ref netlayers). The module
 *           user is responsible for storing the handle and using it to refer to routing information already
 *           created through calls to @ref Pa_addMac, @ref Pa_addSrio, @ref Pa_addCustomLUT1 and @ref Pa_addIp.
 */
typedef void*  paHandleL2L3_t;

/**
 * @ingroup palld_api_structures
 * @brief  PA link handle specification for L2, L3 (LUT1) and virtual link handles
 *
 * @details  This type is used to reference L2, L3 (LUT1) and virtual link information. The module
 *           user is responsible for storing the handle and using it to refer to L2/L3/Virtual link handle already
 *           created through calls to @ref Pa_addMac, @ref Pa_addSrio, @ref Pa_addCustomLUT1, @ref Pa_addIp and
 *           @ref Pa_addVirtualLink
 */
typedef void* paLnkHandle_t;

/**
 * @ingroup palld_api_structures
 * @brief  PA handle specification for ACL (LUT1) handles
 *
 * @details  This type is used to reference ACL (LUT1) entry with the ACL table. The module
 *           user is responsible for storing the handle and using it to refer to ACL entry already
 *           created through calls to @ref Pa_addAcl.
 */
typedef void*  paHandleAcl_t;

/**
 * @ingroup palld_api_structures
 * @brief  PA handle specification for EOAM (LUT1) handles.
 *
 * @details  This type is used to reference EOAM (LUT1) entry with the EOAM table. The module
 *           user is responsible for storing the handle and using it to refer to EOAM entry already
 *           created through calls to @ref Pa_addEoamFlow. 
 *           Please refer to @ref appendix8 for details about EOAM mode.
 */
typedef void*  paHandleEoam_t;

/**
 * @ingroup palld_api_structures
 * @brief  PA handle specification for Flow Cache (LUT1) handles
 *
 * @details  This type is used to reference Flow Cache (LUT1) entry with the Flow Cache (FC) table. The module
 *           user is responsible for storing the handle and using it to refer to Flow Cache entry already
 *           created through calls to @ref Pa_addFc.
 */
typedef void*  paHandleFc_t;

/**
 * @brief The un-linked inner IP handle
 *
 * @details This handle value is used to specify an inner IP (tunnel) which the application does not
 *          want to link to an outer IP address.
 */
#define PA_LLD_HANDLE_IP_INNER  ((paHandleL2L3_t)1)

/**
 *  @ingroup palld_api_structures
 *  @brief  PA handle specification for L4 (LUT2) handles
 *
 *  @details  This type is used to reference L4 (LUT2) routing information (see @ref netlayers). The module user
 *            is responsible for storing the handle. It is used again only to delete a created route.
 *            
 */
typedef uint32_t paHandleL4_t[3];


/**
 *  @ingroup palld_api_structures
 *  @brief A generic entry handle types
 *
 *  @details  The union of both entry handle types used by the module is used only in function @ref Pa_forwardResult.
 *            The function will return the corresponding entry type and its handle in the command response packets when a LUT1
 *            or LUT2 entry is added into the LUT1/LUT2 table successfully.
 *            The handle entry will be set to zero in all other cases 
 */
typedef union  {

  paHandleL2L3_t  l2l3Handle;  /**<  Level 2 or level 3 handle created by @ref Pa_addMac @ref Pa_addSrio, @ref Pa_addCustomLUT1 or @ref Pa_addIp */
  paHandleAcl_t   aclHandle;   /**<  ACL handle created by @ref Pa_addAcl (Gen2 only) */
  paHandleEoam_t  eoamHandle;  /**<  EOAM handle created by @ref Pa_addEoamFlow (Gen2 only) */
  paHandleFc_t    fclHandle;   /**<  Flow Cache handle created by @ref Pa_addFc (Gen2 only) */
  paHandleL4_t    l4Handle;    /**<  Level 4 handle created by @ref Pa_addPort or @ref Pa_addCustomLUT2 */

} paEntryHandle_t;

/**
 *  @ingroup palld_api_constants
 *  @{
 *  @brief  The number of bytes available for custom lookup
 *
 *  @details Custom lookup sizes are fixed by hardware
 */
#define pa_NUM_BYTES_CUSTOM_LUT1  32
#define pa_NUM_BYTES_CUSTOM_LUT2   4
/** @} */

/**
 *  @defgroup HandleTypes Handle Types
 *
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Handle Types
 *
 *  @brief  These values are used to describe what type of handle is referenced.
 *
 *  @details  These values are used only for function @ref Pa_forwardResult. The function returns with a copy
 *            of the handle, which the module user should already have, along with the type of handle. The
 *            module user can use this information to verify that a particular handle has been fully activated
 *            and can be used for linking reference in calls to @ref Pa_addIp, @ref Pa_addCustomLUT1, 
 *            @ref Pa_addCustomLUT2,  @ref Pa_addPort or @ref Pa_addAcl.
 */
/*  @{  */
/**
 *
  *  @def  pa_L2_HANDLE
  *        Level 2 (MAC/SRIO) handle
  */
#define  pa_L2_HANDLE   2

/**
 *  @def  pa_L3_HANDLE
 *        Level 3 (IP, Custom LUT1) handle
 */
#define  pa_L3_HANDLE   3

/**
 *  @def  pa_L4_HANDLE
 *        Level 4 (TCP/UDP/GTP-U/Custom LUT2) handle
 */
#define  pa_L4_HANDLE   4

/**
 *  @def  pa_ACL_HANDLE
 *        ACL (Access Control List) handle
 */
#define  pa_ACL_HANDLE  10 

/**
 *  @def  pa_FC_HANDLE
 *        FC (Flow Cache) handle
 */
#define  pa_FC_HANDLE   11 

/**
 *  @def  pa_EOAM_HANDLE
 *        EOAM (Ethernet OAM) handle
 */
#define  pa_EOAM_HANDLE   12 

/**
 *  @def  pa_INVALID_HANDLE
 *        Invalid handle type
 */
#define pa_INVALID_HANDLE   -1

/*  @}  */  
/** @} */

/**
 *  @defgroup ErouteTypes Exception Route Types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Exception Route Types
 *
 *  @brief  These values are used to define exception route conditions.
 *
 *  @details  The exception route defines the global routing information when the exception condition such
 *            as LUT1 lookup failure, packet parsing failure, broadcast packet detection and etc. Multiple
 *            exception routes can be configured through @ref Pa_configExceptionRoute. All the exception
 *            routes are disabled by default.
 */
/*  @{  */
/**
 *
 *   @def  pa_EROUTE_L2L3_FAIL
 *         packet failed to match in L2/L3 (LUT1) table
 */
#define pa_EROUTE_L2L3_FAIL         0

/**
 *  @def  pa_EROUTE_VLAN_MAX_DEPTH
 *        packet exceeded maximum number of VLAN tags
 */
#define pa_EROUTE_VLAN_MAX_DEPTH    1

/**
 *  @def  pa_EROUTE_IP_MAX_DEPTH
 *        packet exceeded maximum number of IP headers
 */
#define pa_EROUTE_IP_MAX_DEPTH      2

/**
 *  @def  pa_EROUTE_MPLS_MAX_DEPTH
 *        packet exceeded maximum number of MPLS headers
 */
#define pa_EROUTE_MPLS_MAX_DEPTH    3

/**
 *  @def  pa_EROUTE_GRE_MAX_DEPTH
 *        packet exceeded maximum number of GRE headers
 */
#define pa_EROUTE_GRE_MAX_DEPTH     4

/**
 *  @def pa_EROUTE_PARSE_FAIL
 *       packet failed to parse
 */
#define pa_EROUTE_PARSE_FAIL        5

/**
 *  @def pa_EROUTE_L4_FAIL
 *       packet failed to match in L4 (LUT2) table
 */
#define pa_EROUTE_L4_FAIL           6

/**
 *  @def pa_EROUTE_IP_FRAG
 *       IP fragmented packet 
 */
#define pa_EROUTE_IP_FRAG           7

/**
 *  @def pa_EROUTE_IPV6_OPT_FAIL
 *       Packet failed due to unsupported IPV6 option header
 */
#define pa_EROUTE_IPV6_OPT_FAIL     8

/**
 *  @def pa_EROUTE_UDP_LITE_FAIL
 *       UDP lite packet had invalid checksum coverage
 */
#define pa_EROUTE_UDP_LITE_FAIL     9

/**
 *  @def pa_EROUTE_ROUTE_OPTION
 *       IP routing had incomplete routes
 */
#define pa_EROUTE_ROUTE_OPTION      10

/**
 *  @def  pa_EROUTE_SYSTEM_FAIL
 *        Sub-system detected internal error
 */
#define pa_EROUTE_SYSTEM_FAIL       11

/**
 *  @def pa_EROUTE_MAC_BROADCAST
 *       MAC broadcast packet which is not specified at the lookup table
 */
#define pa_EROUTE_MAC_BROADCAST     12

/**
 *  @def pa_EROUTE_MAC_MULTICAST
 *       MAC multicast packet which is not specified at the lookup table
 */
#define pa_EROUTE_MAC_MULTICAST     13

/**
 *  @def pa_EROUTE_IP_BROADCAST
 *       IP broadcast packet which is not specified at the lookup table
 */
#define pa_EROUTE_IP_BROADCAST      14

/**
 *  @def pa_EROUTE_IP_MULTICAST
 *       IP multicast packet which is not specified at the lookup table
 */
#define pa_EROUTE_IP_MULTICAST      15

/**
 *  @def pa_EROUTE_GTPU_MESSAGE_TYPE_1
 *       GTP-U PING Request packet
 */
#define pa_EROUTE_GTPU_MESSAGE_TYPE_1      16

/**
 *  @def pa_EROUTE_GTPU_MESSAGE_TYPE_2
 *       GTP-U PING Response packet
 */
#define pa_EROUTE_GTPU_MESSAGE_TYPE_2      17

/**
 *  @def pa_EROUTE_GTPU_MESSAGE_TYPE_26
 *       GTP-U Error Indication packet
 */
#define pa_EROUTE_GTPU_MESSAGE_TYPE_26     18

/**
 *  @def pa_EROUTE_GTPU_MESSAGE_TYPE_31
 *       GTP-U Supported Header Notification packet
 */
#define pa_EROUTE_GTPU_MESSAGE_TYPE_31     19

/**
 *  @def pa_EROUTE_GTPU_MESSAGE_TYPE_254
 *       GTP-U End Marker packet
 */
#define pa_EROUTE_GTPU_MESSAGE_TYPE_254    20

/**
 *  @def pa_EROUTE_GTPU_FAIL
 *       Packet failed due to GTPU parsing error or unsupporte dmessage types
 */
#define pa_EROUTE_GTPU_FAIL                21

/**
 *  @def pa_EROUTE_PPPoE_FAIL
 *       Packet failed due to PPPoE session packet parsing error
 */
#define pa_EROUTE_PPPoE_FAIL               22

/**
 *  @def pa_EROUTE_PPPoE_CTRL
 *       PPPoE session stage non-IP packets
 */
#define pa_EROUTE_PPPoE_CTRL               23

/**
 *  @def pa_EROUTE_802_1ag
 *       802.1ag Packet
 */
#define pa_EROUTE_802_1ag                  24

/**
 *  @def pa_EROUTE_IP_FAIL
 *       Packet failed due to invalid IP header
 */
#define pa_EROUTE_IP_FAIL                  25

/**
 *  @def pa_EROUTE_NAT_T_KEEPALIVE
 *       NAT-T Keep Alive packet where UDP Length = 9, data = 0xFF
 */
#define pa_EROUTE_NAT_T_KEEPALIVE          26

/**
 *  @def pa_EROUTE_NAT_T_CTRL
 *       NAT-T control packet where UDP Length > 12 and the first 4 payload bytes are equal to 0
 */
#define pa_EROUTE_NAT_T_CTRL               27

/**
 *  @def pa_EROUTE_NAT_T_DATA
 *       NAT-T IPSEC ESP data packet where UDP Length > 12 and the first 4 payload bytes are not equal to 0
 */
#define pa_EROUTE_NAT_T_DATA               28

/**
 *  @def pa_EROUTE_NAT_T_FAIL
 *       Invalid NAT-T packet
 */
#define pa_EROUTE_NAT_T_FAIL               29

/**
 *  @def pa_EROUTE_GTPU_MATCH_FAIL
 *       GTPU match failed 
 */
#define pa_EROUTE_GTPU_MATCH_FAIL          30

/**
 *  @def   pa_EROUTE_MAX
 *         The maximum number of global route types
 */
#define pa_EROUTE_MAX                      31

/*  @}  */  
/** @} */


/**
 *  @defgroup NextHeaderTypes Next Header types 
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Next Header types
 *
 *  @brief  These values are used to define the next header (protocol) types for continus parsing after the 
 *          SRIO and custom parsing.
 *
 *  @details  The next header type can be derived from the upper layer header in a standard Ethernet packet.
 *            For SRIO and custom LUT1 lookup, the next header type should be provided by the user in function
 *            @ref Pa_setCustomLUT1 and @ref Pa_addSrio. 
 */
/*  @{  */
/**
 *
 *   @def  pa_HDR_TYPE_MAC
 *         MAC header
 */
#define pa_HDR_TYPE_MAC         0

/**
 *
 *  @def   pa_HDR_TYPE_IPV4
 *         IPv4 header
 */
#define pa_HDR_TYPE_IPV4        1


/**
 *
 *  @def   pa_HDR_TYPE_IPV6
 *         IPv6 header
 */
#define pa_HDR_TYPE_IPV6        2

/**
 *
 *  @def   pa_HDR_TYPE_CUSTOM_LUT1
 *         Custom LUT1 header
 */
#define pa_HDR_TYPE_CUSTOM_LUT1 3

/**
 *
 *  @def   pa_HDR_TYPE_UDP
 *         UDP header
 */
#define pa_HDR_TYPE_UDP         4

/**
 *
 *  @def   pa_HDR_TYPE_UDP_LITE
 */
#define pa_HDR_TYPE_UDP_LITE    5

/**
 *
 *  @def   pa_HDR_TYPE_TCP
 *         TCP header
 */
#define pa_HDR_TYPE_TCP         6

/**
 *
 *  @def   pa_HDR_TYPE_CUSTOM_LUT2
 *         Custom LUT2 header
 */
#define pa_HDR_TYPE_CUSTOM_LUT2 7

/**
 *
 *  @def   pa_HDR_TYPE_UNKNOWN
 *         next header type is not specified 
 */
#define pa_HDR_TYPE_UNKNOWN     8
 
/*  @}  */  
/** @} */

/** 
 * @ingroup palld_api_structures
 * @brief pa RM Handle
 */
typedef void *  pa_RmHnd;

/** 
 * @ingroup palld_api_structures
 * @brief PA start configuration structure
 */
typedef struct
{    
    pa_RmHnd  rmServiceHandle;         /**< Resource Manager service handle */
    uint32_t  baseAddr;                /**< Specify the PASS base address */
    void*     instPoolBaseAddr;        /**< Base address of the global shared memory pool from which global
                                            LLD instance & channel instance memory is allocated */
} paStartCfg_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  Pointer to the buffer where the PASS command is placed
 *
 *  @details  Functions in this module produce formatted commands that must be sent to the packet accelerator
 *            sub-system. These commands are always referred to through this type.
 */
typedef void*  paCmd_t;

/**
 * @ingroup palld_api_structures
 * @brief  PA Ip traffic flow information structure
 *
 * @details Snap shot of the PA Reassembly traffic flow information structure
 */
typedef struct
{
    int32_t  index;       /** < active traffic flow index, -1:for inactive traffic flow */
    uint32_t srcIp;       /** < source IP address: complete IP address for IPv4 OR lower 32 bits in IPv6 */
    uint32_t dstIp;       /** < destination IP address:  complete IP address for IPv4 OR lower 32 bits in IPv6 */
    uint16_t proto;       /** < protocol field in IP header */
    uint16_t count;       /** < number of pending fragments and non-fragmented pkts */
} pa_ReassemblyFlow_t;

/**
 * @ingroup palld_api_structures
 * @brief  PA Ip Reassembly control context snap shot Information Structure
 *
 * @details The PA Reassembly control context structure contains the snap shot of the
 *         reassembly context information.
 */
typedef struct
{
  uint16_t    numTF;       /** < Maximum number of traffic flow entries */
  uint16_t    numActiveTF; /** < number of active traffic flows */
  uint16_t    queue;       /** < The destination queue where PASS will deliver
                                the packets which require reassembly assistance */
  uint16_t    flowId;      /** < CPPI Flow which instructs how free queues are used
                                for receiving the packets */
  pa_ReassemblyFlow_t    traffic_flow[32];    /**< traffic flow snap shot */
} pa_trafficFlow_t;

typedef struct
{
    pa_trafficFlow_t  outer; /** < Outer IP traffic flow Reassembly context */
    pa_trafficFlow_t  inner; /** < Inner IP traffic flow Reassembly context */
} pa_ReassemblyContext_t;



/**
 *  @defgroup dbgInfoType  PA Packet Debug Operation type Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Packet Debug information type
 *
 *  Debug information type in @ref paSnapShotDebugInfo_t
 *
 */
/*@{*/

/**
 *  @def  pa_DBG_INFO_TYPE_REASSEMBLY_ENABLE
 *        Debug Info -- Set: Perform snap shot of reassembly control information
 *                      Clear: Snap shot operation is not performed
 */
#define pa_DBG_INFO_TYPE_REASSEMBLY_ENABLE       0x0001

/*@}*/
/** @} */

/**
 * @ingroup palld_api_structures
 * @brief  PA Debug Information Structure
 *
 * @details The PA debug information structure contains the snap shot of the
 *          hardware registers sampled at a particular time.
 */
typedef struct
{
    uint32_t    debugInfoType; /**< Debug extract operation control
                                    information as defined at @ref dbgInfoType */

    union {
        pa_ReassemblyContext_t reassemContext;
    } u;
} paSnapShotDebugInfo_t;

/**
 * @ingroup palld_api_structures
 * @brief  PA Size Configuration Structure
 *
 * @details The module is configured at run time with a maximum number of handles supported. The module
 *          maintains a set of handles and links between handles.
 */
typedef struct  {

  int nMaxL2;   /**< Maximum number of L2 handles supported */
  int nMaxL3;   /**< Maximum number of L3 handles supported */
  int nUsrStats;/**< Maximum number of user-defined statistics supported (maximum: 512)*/
  int nMaxVlnk; /**< Maximum number of virtual links supported */
  int nMaxAcl;  /**< Maximum number of Stateless ACL handles supported (Gen2 only)*/
  int nMaxFc;   /**< Maximum number of Flow Cache Hanndles supported (Gen2 only) */
  int nMaxEoam; /**< Maximum number of EOAM Hanndles supported (Gen2 only). Please refer to @ref appendix8 for details about EOAM mode. */  
} paSizeInfo_t;

/**
 *  @ingroup salld_api_constants
 *  @{
 *  @brief  PA Reassembly Engine related constant definitions
 */
#define pa_RA_MAX_HEAP_REGIONS    2    /**< Maxmium number of RA Heap Regions */ 
#define pa_RA_NUM_GROUPS          2    /**< Number of RA groups */

/**
 *  @ingroup palld_api_structures
 *  @brief PA Reassembly  Engine global config structure
 *
 *  @details The parameters in this structure are used to configure the Reassembly
 *           engine with PASS
 */
typedef struct  {
  int           ipv4MinPktSize;     /**< Specify the minimum packet size in bytes for a fragment of an
                                         Ipv4 packet that is not the last fragment. The deafult value 
                                         is 68 byte to contain 60 bytes of IP header including options
                                         plus 8-byte of payload */
  int           numCxts;            /**< Total number of contexts the RA handles. This value affects 
                                         the amount of heap memory that needs to be allocated. This 
                                         value must be between 0x1 and 0x400 (1k). If set to 0, all 
                                         fragments will be discarded. The default value is 0x400. 
                                         Note: Each context requires 65kB heap memory */
  int           cxtDiscardThresh;   /**< Number of concurrent contexts that, once reached, causes the 
                                         oldest current context to be forcibly timed out.  To prevent 
                                         this behavior, this value should be programmed to be equal to 
                                         or greater than the Total Contexts. This value must be between 
                                         0x1 and 0x400 (1k). The default value is 0x400. */
  int           nodeDiscardThresh;  /**< Number of Nodes that, once reached, causes the oldest current 
                                         context to be forcibly timed out.  To prevent this behavior, 
                                         this value should be programmed to be the maximum value 
                                         (there are 4K total nodes). This value must be between 0x1 
                                         and 0x1000 (4K). The default value is 0xFFF. */
  int           cxtTimeout;         /**< Amount of time (in ms) after a new context has been allocated until 
                                         that context times out.  If timeout occurs before a packet is completely 
                                         reassembled and the SOP fragment has been received, a packet containing 
                                         the IP header and the first 8 bytes of data is forwarded up to the host 
                                         that it can respond with an ICPM Time Exceeded message as per RFC 792.  
                                         If a context times out and the SOP fragment has not been received, 
                                         the packet is discarded and the context freed. */ 
  int           clockRate;          /**< Clock rate of the Reassembly engine in MHz. */                                        
  int           heapRegionThresh;   /**< Number of contexts handled in Region 0 of the Reassembly Heap. All contexts 
                                         in excess of this number are handled in Region 1. If Region 1 is not used, 
                                         this value should be set equal to (or higher) than Total Contexts. */
  uint64_t      heapBase[pa_RA_MAX_HEAP_REGIONS];   /**< Reassembly Heap addresses which should be 64-byte aligned */                                                                                                                                                                                                                                       
} paRaConfig_t;

/**
 * @ingroup palld_api_structures
 * @brief  PA LUT1 Information Structure
 *
 * @details LLD can return the LUT1 information using @ref Pa_getLUT1Info API calls.
 */
typedef struct {
  int lut1Inst;   /**< which LUT1 (0-2) Instance is used */
  int lut1Index; /**< which LUT1 entry (0-63) is used  */
} paLUT1Info_t;

/**
 *  @ingroup palld_api_structures
 *  @brief PA Initialization config structure
 *
 *  @details The parameters in this structure are used to do initial setup
 *           of the driver including its base address and other default settings.
 *           
 *  @note    The stream interface switch controls the destination of the traffic
 *           from the Ethernet switch. The default setting of the streaming
 *           interface switch is to route all traffic to host queues.  
 *           This module is designed to receive the incoming packets at the PDSP0.
 *           If the initDeafultRoute is set to TRUE, this module will re-configure
 *           the stream interface switch to route all traffic to PDSP0. Otherwise,
 *           it is the module user's reponsibility to deliver incoming packets
 *           to PDSP0 via the CPPI/QMSS interface.           
 */
typedef struct  {
  uint16_t      initTable;          /**< If True then the L2/L3/ACL tables are initialized */
  uint16_t      initDefaultRoute;   /**< If True then the switch default route is set to PASS PDSP0 */
  uint32_t      baseAddr;           /**< Specify the PASS base address */
  void* 	    instPoolBaseAddr;	/**< Base address of the global shared memory pool from which global 
									     LLD instance & channel instance memory is allocated */
  pa_RmHnd      rmServiceHandle;    /**< Resource Manager service handle */
  paSizeInfo_t* sizeCfg;            /**< Pointer to the size configuration information */
  paRaConfig_t* raCfg;              /**< Pointer to the RA global configuration information (Gen2 only) */
} paConfig_t;

/**
 *  @ingroup salld_api_constants
 *  @{
 *  @brief  Protocol Limit related constant definitions
 */
#define pa_PROTOCOL_LIMIT_NUM_VLANS_DEF    2    /**< Number of VLAN supported: default value */ 
#define pa_PROTOCOL_LIMIT_NUM_IP_DEF       2    /**< Number of IP layers supported: default value */ 
#define pa_PROTOCOL_LIMIT_NUM_GRE_DEF      2    /**< Number of GRE layers supported: default value */
#define pa_PROTOCOL_LIMIT_NUM_VLANS_MAX    3    /**< Number of VLAN supported: maximum value */ 
#define pa_PROTOCOL_LIMIT_NUM_IP_MAX       7    /**< Number of IP layers supported: maximum value */ 
#define pa_PROTOCOL_LIMIT_NUM_GRE_MAX      7    /**< Number of GRE layers supported: maximum value */

/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Protocol-specific Limitations.
 *
 *  @details  paProtocolLimit_t is used to defines the protocol-specific restrictions. For example, 
 *            it is necessary to limit the number of protocol layers such as GRE of the input packets 
 *            to prevent the irregular packets take too much processing time. 
 *            The PASS will detect the packets which violate the protocol-specific restrictions and either discard 
 *            or forward the packets to host queues which can be specified through API @ref Pa_configExceptionRoute.
 * 
 *  @note     The PASS will work when non-default values are used. However, it may limit the supported packet rate 
 *            below wire rate.
 */
typedef struct {

  uint8_t vlanMax; /**< Maximum number of VLANs supported, default = 2, maximum = 3 */
  uint8_t ipMax;   /**< Maximum number of IP layers supported, default = 2, maximum = 7 */
  uint8_t greMax;  /**< Maximum number of GRE layers supported, default = 2, maximum = 7 */

} paProtocolLimit_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  IP Reassembly Configuration Information.
 *
 *  @details  paIpReassmConfig_t is used to configure the PA-assisted IP reassembly operation. Two separate structures are used 
 *            for the outer IP and inner IP respectively. The IP reassembly assistance feature is disabled until 
 *            this information is provided. See section @ref appendix3 for deatiled description. 
 *  @note The maximum number of traffic flows is limited due to processing time and internal memory restriction.
 */
typedef struct {

  uint8_t numTrafficFlow; /**< Maximum number of IP reassembly traffic flows supported, default = 0, maximum = 32 */
  uint8_t destFlowId;     /**< CPPI flow which instructs how the link-buffer queues are used for forwarding packets */
  uint16_t destQueue;     /**< Destination host queue where PASS will deliver the packets which require IP reassembly assistance */

} paIpReassmConfig_t;

/**
 *  @ingroup palld_api_constants
 *  @brief   Define the maximum number of IP reassembly traffic flows 
 *
 */
#define pa_MAX_IP_REASM_TRAFFIC_FLOWS   32 

/**
 *  @ingroup palld_api_structures
 *  @brief  Command Set Configuration Information.
 *
 *  @details  paCmdSetConfig_t defines command set configuration parameters such as the maximum number of command sets.
 *            The PASS supports either 64 of 64-byte or 32 of 128-byte command sets. The number of command sets should 
 *            be configured at system startup. 
 */
typedef struct {

  uint8_t  numCmdSets; /**<  Number of command sets supported (32, 64), default = 64
                             @note If the number of command sets is set to 64, then each command entry will be limited to 64 bytes.
                                   If the number of command sets is set to 32, then each command entry will be limited to 128 bytes */
} paCmdSetConfig_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  User-defined Statistics Configuration Information.
 *
 *  @details  paUsrStatsConfig_t defines the configuration parameters for multi-level hierarchical user-defined statistics 
 *            operation such as the number of user-defined counters. There are up to 512 user-defined statistics consisting of 
 *            some 64-bit counters and some 32-bit counters whereas the total size of all counters cannot exceed 2048 bytes. 
 *            The user-defined statistics feature is disabled until this configuration is invoked through API @ref Pa_control. 
 *
 *            - 64-bit Counters index: 0 - (num64bCounters - 1)
 *            - 32-bit Counters index: num64bCounters - (numCounters - 1)
 */
typedef struct {

  uint16_t numCounters;     /**< Total number of user-defined counters, default = 0, maximum = 512 */
  uint16_t num64bCounters;  /**< Number of 64-bit user-defined counters, default = 0, maximum = 256 */
   
} paUsrStatsConfig_t;
 
/**
 *  @ingroup salld_api_constants
 *  @brief   Define the maximum number of user-defined statistics the module supports.
 *
 */
#define pa_USR_STATS_MAX_COUNTERS        512

/**
 *  @ingroup salld_api_constants
 *  @brief   Define the maximum number of user-defined 64-bit statistics 
 *
 */
#define pa_USR_STATS_MAX_64B_COUNTERS    (pa_USR_STATS_MAX_COUNTERS/2)
/**
 *  @ingroup salld_api_constants
 *  @brief   Define the maximum number of user-defined 32-bit statistics 
 *
 */
#define pa_USR_STATS_MAX_32B_COUNTERS    pa_USR_STATS_MAX_COUNTERS

/**
 *  @defgroup paUsrStatsSizes PA User-defined Ststaistics Counter Sizes
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name User-defined Ststaistics Counter Sizes
 *
 *  Definition of Counter size of the User-defined Statistics
 */ 
/** @ingroup paUsrStatsSizes */
/*@{*/
typedef enum {
  pa_USR_STATS_SIZE_32B = 0,   /**< 32-bit Counter */
  pa_USR_STATS_SIZE_64B        /**< 64-bit Counter */
} paUsrStatsSizes_e;
/*@}*/
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  Queue Diversion Configuration Information.
 *
 *  @details  The PASS supports optional queue diversion operation per LUT2 entry replacement. 
 *            paQueueDivertConfigl_t contains configuration information for the atomic queue diversion operation.
 *            The queue diversion feature is disabled until this configuration is invoked through API @ref Pa_control. 
 *
 */
typedef struct {

  uint16_t destQueue;     /**< Destination queue where PASS will deliver the LUT2 response packet which contains the 
                               queue diversion information */
  uint8_t  destFlowId;    /**< CPPI flow which instructs how the link-buffer queues are used for forwarding 
                               the LUT2 response packets */
} paQueueDivertConfig_t;


/**
 *  @defgroup paPktControlInfo  PA Packet Control Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Packet Control Bit Definitions
 *
 *  Bitmap definition of the ctrlBitMap in @ref paPacketControlConfig_t
 *  and @ref paPacketControl2Config_t. 
 */ 
/*@{*/
/**
 *  @def  pa_PKT_CTRL_HDR_VERIFY_PPPoE
 *        Control Info -- Set: Perform enhanced error check of the PPPoE header
 *                        Clear: Perform basic error check of the PPPoE header
 */
#define pa_PKT_CTRL_HDR_VERIFY_PPPoE               0x0001 
/**
 *  @def  pa_PKT_CTRL_HDR_VERIFY_IP
 *        Control Info -- Set: Perform enhanced error check of the IP header
 *                        Clear: Perform basic error check of the IP header
 */
#define pa_PKT_CTRL_HDR_VERIFY_IP                  0x0002 
/**
 *  @def  pa_PKT_CTRL_MAC_PADDING_CHK
 *        Control Info -- Set: Perform MAC (802.3) padding check
 *                             The packet with illegal padding will be dropped  
 *                        Clear: Do not perform MAC (802.3) padding check
 */
#define pa_PKT_CTRL_MAC_PADDING_CHK                0x0004 
/**
 *  @def  pa_PKT_CTRL_IP_FRAGS_TO_EROUTE
 *        Control Info -- Set: Forward IP Fragments through the exception route regardless of the routing destination 
 *                        Clear: Forward IP Fragments through the exception route only if the routing destination is set to SASS or CONTINUE_PARSE
 */
#define pa_PKT_CTRL_IP_FRAGS_TO_EROUTE             0x0008 
/**
 *  @def  pa_PKT_CTRL_L3OFFSET_TO_INNER_IP
 *        Control Info -- Set: L3offset of the packet information points to the inner IP header prior to payload
 *                        Clear: L3offset of the packet information points to the outer IP header (default)
 */
#define pa_PKT_CTRL_L3OFFSET_TO_INNER_IP           0x0010 

/**
 *  @def  pa_PKT_CTRL_EMAC_IF_IGRESS_CLONE
 *        Control Info -- Set: Enable EMAC interface-based packet capture/mirror for ingress ethernet traffic
 *                        Clear: disable EMAC interface-based packet capture/mirror (default) for ingress ethernet traffic
 *  @note This definition is only vaild at @ref paPacketControl2Config_t. 
 */
#define pa_PKT_CTRL_EMAC_IF_IGRESS_CLONE           0x0020

/**
 *  @def  pa_PKT_CTRL_EMAC_IF_EGRESS_CLONE
 *        Control Info -- Set: Enable EMAC interface-based packet capture/mirror for egress ethernet traffic
 *                        Clear: disable EMAC interface-based packet capture/mirror (default) for egress ethernet traffic
 *  @note This definition is only vaild at @ref paPacketControl2Config_t. 
 */
#define pa_PKT_CTRL_EMAC_IF_EGRESS_CLONE           0x0040

/**
 *  @def  pa_PKT_CTRL_EMAC_IF_INGRESS_DEFAULT_ROUTE   
 *        Control Info -- Set: Enable EMAC interface-based ingress packet default route
 *                        Clear: disable EMAC interface-based ingress packet default route
 *  @note This definition is only vaild at @ref paPacketControl2Config_t. 
 */
#define pa_PKT_CTRL_EMAC_IF_INGRESS_DEFAULT_ROUTE  0x0080

/**
 *  @def  pa_PKT_CTRL_EMAC_IF_EGRESS_EQoS_MODE
 *        Control Info -- Set: Enable EMAC interface-based enhanced QoS Mode for egress ethernet traffic
 *                        Clear: Disable EMAC interface-based enhanced QoS Mode for egress ethernet traffic
 *  @note This definition is only vaild at @ref paPacketControl2Config_t. 
 */
#define pa_PKT_CTRL_EMAC_IF_EGRESS_EQoS_MODE       0x0100

/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Packet Control Configuration Information.
 *
 *  @details  This data structure defines miscellaneous packet control information for some non-default PASS operations. 
 *            For example, PASS always performs basic protocol header verification to ensure that it can continue parsing the  
 *            current and next protocol header. The PASS will perform enhanced error check of protocol headers specified
 *            by this configuration. For example,
 *            PPPoE header in session mode:
 *              - Version = 1
 *              - Type = 1
 *              - Code = 0
 *
 *            IPv4 header:
 *              - Header length >= 20
 *              - Total length > 20
 *              - Source address is not broadcast
 *              - Destination address is not 0
 *              - TTL is not 0
 *
 *  @note refer to the @ref ErouteTypes for the corresponding exception routes.
 *  @note This data structure will be depreciated and replaced with paPacketControl2Config_t due to its limitation that
 *        all desired control bits and other parameters must be provided when it is invoked every time.
 */
typedef struct {

  uint16_t ctrlBitMap;              /**< Packet control bit as defined at @ref paPktControlInfo */
  uint16_t rxPaddingErrStatsIndex;  /**< Specify the user statistics index of Rx padding error counter */
  uint16_t txPaddingStatsIndex;     /**< Specify the user statistics index of Tx MAC padding counter */

} paPacketControlConfig_t;

/**
 *  @defgroup paPktControlValidBits  PA Packet Control Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Packet Control Valid Bit Definitions
 *
 *  Bitmap definition of the validBitmap in @ref paPacketControl2Config_t. 
 */ 
/*@{*/

/**
 *  @def  pa_PKT_CTRL2_VALID_PPPoE_HDR_CHECK
 *        - Header check configuration for PPPoE is present in the configuration
 */
#define pa_PKT_CTRL2_VALID_PPPoE_HDR_CHECK     pa_PKT_CTRL_HDR_VERIFY_PPPoE

/**
 *  @def  pa_PKT_CTRL2_VALID_IP_HDR_CHECK
 *        - Header check configuration for IP is present in the configuration
 */
#define pa_PKT_CTRL2_VALID_IP_HDR_CHECK        pa_PKT_CTRL_HDR_VERIFY_IP

/**
 *  @def  pa_PKT_CTRL2_VALID_MAC_PADDING_CHECK
 *        - MAC padding check configuration is present in the configuration
 */
#define pa_PKT_CTRL2_VALID_MAC_PADDING_CHECK   pa_PKT_CTRL_MAC_PADDING_CHK

/**
 *  @def  pa_PKT_CTRL2_VALID_IP_FRAGS_TO_EROUTE
 *        - IP fragmentation exception routing configuration is present in the configuration
 */
#define pa_PKT_CTRL2_VALID_IP_FRAGS_TO_EROUTE  pa_PKT_CTRL_IP_FRAGS_TO_EROUTE

/**
 *  @def  pa_PKT_CTRL2_VALID_L3_OFFSET
 *        - L3 offset to inner/outer IP configuration is present in the configuration
 */
#define pa_PKT_CTRL2_VALID_L3_OFFSET           pa_PKT_CTRL_L3OFFSET_TO_INNER_IP

/**
 *  @def  pa_PKT_CTRL2_VALID_EMAC_IF_IGRESS_CLONE
 *        - Valid Ingress packet capture/mirror configuration is present in the configuration
 */
#define pa_PKT_CTRL2_VALID_EMAC_IF_IGRESS_CLONE        pa_PKT_CTRL_EMAC_IF_IGRESS_CLONE

/**
 *  @def  pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_CLONE
 *        - Valid Egress packet capture/mirror configuration is present in the configuration
 */
#define pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_CLONE        pa_PKT_CTRL_EMAC_IF_EGRESS_CLONE
/**
 *  @def  pa_PKT_CTRL2_VALID_EMAC_IF_INGRESS_DEFAULT_ROUTE   
 *        - Valid emac interface ingress default route configuration is present
 */
#define pa_PKT_CTRL2_VALID_EMAC_IF_INGRESS_DEFAULT_ROUTE pa_PKT_CTRL_EMAC_IF_INGRESS_DEFAULT_ROUTE

/**
 *  @def  pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_EQoS_MODE 
 *        - Valid emac interface egress enhanced QoS mode is present
 */
#define pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_EQoS_MODE      pa_PKT_CTRL_EMAC_IF_EGRESS_EQoS_MODE

/**
 *  @def  pa_PKT_CTRL2_VALID_PADDING_STATS_INDEX 
 *        - Valid rxPaddingErrStatsIndex and txPaddingStatsIndex are present
 */
#define pa_PKT_CTRL2_VALID_PADDING_STATS_INDEX           0x8000

/* @} */ 
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Packet Control Configuration2 Information.
 *
 *  @brief  Enhanced Packet Control configuration structure
 *
 *  @details  paPacketControl2Config_t is the upgraded version of paPacketControlConfig_t to support 
 *            individual feature control without affecting the other feature operations. It is achieved
 *            by introducing the parameter validBitMap where only the valid control bits and their
 *            associated parameters will be processed by PASS and the other feature operations will
 *            not be affected. 
 */
typedef struct {

  uint16_t validBitMap;             /**< Valid control bits as defined at @ref paPktControlValidBits */
  uint16_t ctrlBitMap;              /**< Packet control bit as defined at @ref paPktControlInfo */
  uint16_t rxPaddingErrStatsIndex;  /**< Specify the user statistics index of Rx padding error counter 
                                         note This parameter is valid only if pa_PKT_CTRL2_VALID_PADDING_STATS_INDEX is set.*/
  uint16_t txPaddingStatsIndex;     /**< Specify the user statistics index of Tx MAC padding counter 
                                         note This parameter is valid only if pa_PKT_CTRL2_VALID_PADDING_STATS_INDEX is set.*/
  uint8_t  egressDefPri;            /**< Specify the global default priority for untagged non IP egress traffic 
                                         for enhanced QoS mode (refer to @ref appendix7)
                                         @note This parameter is valid only if both pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_EQoS_MODE 
                                         and the corresponding enable bit are set. */
   
} paPacketControl2Config_t;


/**
 *  @defgroup  paAclActionTypes PA ACL action types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   PA ACL action types
 *  @brief  Define the ACL action types.
 *
 *  @details Define actions to be taken when an ACL entry is matched
 */
/* @{ */
/**
 *  @def  pa_ACL_ACTION_PERMIT
 *        Allow matched packets to be forwarded to the next stage
 */
#define  pa_ACL_ACTION_PERMIT    0

/**
 *  @def  pa_ACL_ACTION_DENY
 *        Matched packets should be dropped
 */
#define  pa_ACL_ACTION_DENY      1

/**
 *  @def  pa_ACL_ACTION_MARK
 *        Matched packets should be forwarded with a mark which may be used later by hardware or software
 */
#define  pa_ACL_ACTION_MARK      2

/**
 *
 *   @def  pa_ACL_ACTION_HOST
 *         The packet should be forwarded to host for further processing 
 *   @note This action is only applicable to default rule
 */
#define pa_ACL_ACTION_HOST       3

#define pa_ACL_ACTION_MAX        pa_ACL_ACTION_HOST
  
/*  @}  */  
/** @} */

/**
 *  @defgroup  paAclInsertModes PA ACL insert modes
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   PA ACL Insert Modes
 *  @brief  Define the ACL insert mode types.
 *
 *  @details ACL entries are ordered entries, each entry having a priority associated with it.
 *
 *           When application has a prior knowledge about the 
 *           new (latest) entry being inserted typically has highest priority against the 
 *           entries already done OR 
 *           new (latest) entry being inserted typically has lowest priority against the 
 *           entries already done OR 
 *           has no prior information on the priority of the new entry, it can indicate it 
 *           in the ACL insert mode parameter during the ACL configuration @ref paAclConfig_t
 *        
 *           This information can be used inside LLD to minimize the manual re-score operations as a result of
 *           inserting ordered entries
 */
/* @{ */
/**
 *  @def  pa_ACL_INSERT_TOP
 *        Application adds new ACL entry to the top of the ACL table typically (typically new entry that is going to be inserted has highest priority).
 */
#define  pa_ACL_INSERT_TOP         2

/**
 *  @def  pa_ACL_INSERT_BOTTOM
 *        Application adds new ACL entry to the bottom of the ACL table typically (typically new entry  that is going to be inserted has lowest priority).
 */
#define  pa_ACL_INSERT_BOTTOM      1

/**
 *  @def  pa_ACL_INSERT_RANDOM
 *        Application adds new ACL entry in any order (application has no prior knowledge on the priority of the new entries that are going to be inserted).
 */
#define  pa_ACL_INSERT_RANDOM      0
 
/*  @}  */  
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  Stateless ACL Configuration Information.
 *
 *  @details  paAclConfig_t is used to configure the default rule of stateless ACL operation. The PASS will follow
 *            this rule if no matches are found at the ACL table. Two separate structures are used 
 *            for the outer ACL and inner ACL respectively. The default rule is set to packet FORWARDING until  
 *            this information is provided. 
 */
typedef struct {

  int      action;      /**< Default action (Deny/Permit/Host) as sepcified at @ref paAclActionTypes */
  uint8_t  destFlowId;  /**< CPPI flow which instructs how the link-buffer queues are used for forwarding packets to host. 
                             (valid only if action = pa_ACL_ACTION_HOST) */
  uint16_t destQueue;   /**< Destination host queue where PASS will deliver the packets if no ACL matches found 
                             (valid only if action = pa_ACL_ACTION_HOST) */
  int      insertMode;  /**< Typical insert order (Top/Bottom/Random) as specified at @ref paAclInsertModes */                              
} paAclConfig_t;

/**
 *  @defgroup paRACtrlInfo  PA RA Control Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA RA Control Bit Definitions
 *
 *  Bitmap definition of the ctrlBitMap in @ref paRaGroupConfig_t. 
 *         
 */ 
/*@{*/
/**
 *  @def  pa_RA_CTRL_ENABLE
 *        Control Info -- Set: Enable Reassembly operation, forward all packets to RA
 *                        Clear: Disable Reassembly operation, bypass RA
 */
#define pa_RA_CTRL_ENABLE           0x0001 
/*@}*/
/**
 *  @def  pa_RA_CTRL_USE_LOCAL_DMA
 *        Control Info -- Set: Use NetCP internal DMA to send packets from PASS to RA engine
 *                        Clear: Use global DMA to send packets from PASS to RA engine
 */
#define pa_RA_CTRL_USE_LOCAL_DMA    0x0002 
/**
 *  @def  pa_RA_CTRL_TO_QUEUE
 *        Control Info -- Set: Forward RA output packets to the host queue specified by the RA output CPPI flow
 *                        Clear: Forward RA output pakets to the next PASS classification stage
 *  @note: The lower 8-bit of source tag of the input CPPI flow should be set to the output CPPI flow when this set
 *         this bit is set and the default queue of the output CPPI flow should be set to the desired destination queue 
 */
#define pa_RA_CTRL_TO_QUEUE         0x0004 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  RA exception Route Information.
 *
 *  @details  The reassembly engine is degined to forward packets to host queue specified by user when timwout or other
 *            error condition occurs. paRaERouteInfo_t contains the routing information required for this operation. 
 */
typedef struct  {

  int      dest;      /**<  Packet destination as defined at @ref pktDest (Host and Discard only) */
  uint8_t  flowId;    /**<  Specifies CPPI flow which defines free queues are used for receiving packets */
  uint16_t queue;     /**<  Specifies the destination host queue */
} paRaERouteInfo_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Reassembly Engine Group Configuration Information.
 *
 *  @details  paRaGroupConfig_t is used to specify the group specific configuration parameters of the PASS 
 *            reassembly Engine.  Two separate structures are used for the outer IP and inner IP reassembly 
 *            respectively. 
 */
typedef struct {
  uint16_t              ctrlBitMap;    /**< RA control info as defined at @ref paRACtrlInfo */
  uint8_t               flowId;        /**< Specify the RA CPPI flow which defines free queues and other paramters 
                                            for sending packets from PASS to RA */ 
  paRaERouteInfo_t      timeoutER;     /**< Specify exception route for timeout packets */
  paRaERouteInfo_t      critErrER;     /**< Specify exception route for packets with critical error */
  paRaERouteInfo_t      genErrER;      /**< Specify exception route for packets with non-critical error*/
} paRaGroupConfig_t;

/**
 *  @ingroup palld_api_constants
 *  @brief   Define the maximum number ethernet protocol types to be excluded from EOAM classification 
 *
 *  @details The application can specify the list of ethernet types to be excluded from EOAM classification
 *           i.e., the target flow statistics count would not be incremented even though the match happens 
 *           when the packet ethernet type matches the list.
 *
 *           Please refer to @ref appendix8 for details about EOAM mode.
 *
 */
#define pa_MAX_ETH_PROTO_EOAM_EXCLUSION    8 

/**
 *  @ingroup palld_api_structures
 *  @brief  PA time offset correction.
 *
 *  @details  paSetTimeOffset_t is used to set the 1588 time offsets at PASS time 0
 *            Please refer to @ref appendix8 for details about EOAM mode.
 *
 */
typedef struct {
  uint32_t           offset_sec;  /**< 1588 Time offset in seconds (needed for features like EOAM) */
  uint32_t           offset_ns;   /**< 1588 Time offset in nano seconds (needed for features like EOAM) */  
}paSetTimeOffset_t;

/**
 @defgroup paInputFreq Packet Accelerator PLL Frequencies
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Packet Accelerator PLL input frequency (Gen2 only)
 *
 *  paInputFreq_e is used to provide the input frequency to PASS programmed either through PA clock or
 *  system clock in MHz. This parameter is useful in converting the ticks to seconds and nano seconds.  
 *
 *  @details This information is required for firmware to convert the time ticks to seconds and nano
 *           seconds. The mathematical operations and the scheme involved in such conversion
 *           requires the input frequency. Hence, not all possible frequencies are supported. 
 *           The converted time can be used to patch the time in the message using 
 *           @ref pa_CMD_PATCH_TIME. The mathematical operations
 *           in firmware are optimized for these frequency lists.
 *
 *  @note This parameter can be ignored if EOAM mode is not enabled and the message is not patched 
 *        for time.
 */ 
/** @ingroup paInputFreq */
/*@{*/
typedef enum {
  pa_INPUT_FREQ_1000MHZ = 1,       /**< PASS Input frequency is 1000 MHz */
  pa_INPUT_FREQ_1050MHZ,           /**< PASS Input frequency is 1050 MHz */
  pa_INPUT_FREQ_1049p6MHZ          /**< PASS Input frequency is 1049.6 MHz */
} paInputFreq_e;

/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Ethernet OAM target flow match Statistics control configuration Information.
 *
 *  @details  paEoamTfExcCtrlConfig_t is used to exclude few ethernet types from EOAM target classification
 *            Please refer to @ref appendix8 for details about EOAM mode.
 *
 */
typedef struct {
  uint8_t                  numProtoExcl;                          /**< Number of protocol to be excluded from EOAM target classifiation */
  uint16_t                 exclEthTypes[pa_MAX_ETH_PROTO_EOAM_EXCLUSION]; /**< maximum number of ethernet types to be excluded from EOAM target classification */  
} paEoamTfExcCtrlConfig_t;

/**
 *  @defgroup paEoamGlobalValidInfo  PA EOAM Global Configuration Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Global Config Valid Bit Definitions
 *
 *  Bitmap definition of the validBitmap in @ref paEoamGlobalConfig_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EOAM_VALID_STATS_CTRL
 *        - Control to increment the Ethernet OAM target flow matches is present
 */
#define pa_EOAM_VALID_STATS_CTRL              (1<<0)

/* @} */ /* ingroup */
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  PA Ethernet OAM Global Configuration Information.
 *
 *  @details  paEoamGlobalConfig_t is used to configure the Ethernet OAM parameters.
 *
 *            Please refer to @ref appendix8 for details about EOAM mode.
 *
 *  @warning EOAM mode can not co-exist with Outer ACL firewall operations. Please make sure
 *           Outer ACL is not configured during EOAM system configuration. There is no dynamic
 *           switching between the original mode and EOAM mode. 
 */
typedef struct {
  uint32_t                 validBitMap; /**< Valid control bits as defined at @ref paEoamGlobalValidInfo */
  uint32_t                 enable;      /**< Enable/Disable EOAM feature, As Outer ACL and EOAM can not co-exist, make sure Outer ACL
                                             entries are all removed before adding the EOAM entries in the LUT table */
  paInputFreq_e            freq;        /**< Mandatory: PA Input Frequency in MHz as defined at @ref paInputFreq_e */ 
  paEoamTfExcCtrlConfig_t  statsCtrl;   /**< Ethernet OAM target flow exclusion protocol control @ref paEoamTfExcCtrlConfig_t */
} paEoamGlobalConfig_t;

/**
 *  @defgroup paQueueBounceRoutingClass PA Queue Bounce Routing Class
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Queue Bounce Routing Class
 *
 *  Definition of PA Queue Bounce Routing Classes
 */
/** @ingroup paQueueBounceRoutingClass */
/*@{*/
typedef enum {
  pa_QUEUE_BOUNCE_ROUTING_CLASS_CMD_RET = 0,   /**< Command Return */
  pa_QUEUE_BOUNCE_ROUTING_CLASS_QoS,           /**< Ingress QoS Packets */
  pa_QUEUE_BOUNCE_ROUTING_CLASS_CAPTURE,       /**< Packet Capture */
  pa_QUEUE_BOUNCE_ROUTING_CLASS_IP_REASSEMBLY, /**< IP Reassembly-assisted packets */
  pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC,          /**< All other Traffic */
  PA_MAX_QUEUE_BOUNCE_ROUTING_CLASSES
}paQueueBounceRoutingClass_e ;
/*@}*/
/** @} */

/**
 *  @defgroup  paQueueBounceOperationTypes PA Queue Bounce operation modes
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   PA Queue Bounce operation modes
 *  @brief  Define the Queue Bounce operation modes.
 */
/* @{ */
/**
 *  @def  pa_QUEUE_BOUNCE_OP_NONE
 *        No bounce, use the user specified destination queue as it is
 */
#define  pa_QUEUE_BOUNCE_OP_NONE    0

/**
 *  @def  pa_QUEUE_BOUNCE_OP_DDR
 *        Add control bits to indicate bouncing to the DDR Queue
 */
#define  pa_QUEUE_BOUNCE_OP_DDR     1

/**
 *  @def  pa_QUEUE_BOUNCE_OP_MSMC
 *        Add control bits to indicate bouncing to the MSMC queue
 */
#define  pa_QUEUE_BOUNCE_OP_MSMC    2


/**
 *  @def  pa_QUEUE_BOUNCE_OP_MAX
 *        Number of Queue Bounce Operation modes
 */
#define pa_QUEUE_BOUNCE_OP_MAX        pa_QUEUE_BOUNCE_OP_MSMC
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Queue Bounce Configuration Information.
 *
 *  @details  paQueueBounceConfig_t is used to configure the PA Queue Bounce operation as described at @ref appendix9.
 *
 *  @note The Queue Bounce Configuration should be specified at PA system initialization and only once. The dynamic
 *  re-configuration is not supported and may cause undefined behaviors.
 */
typedef struct {

  uint32_t  enable;         /**< Enable/Disable(1/0) Queue Bounce operation, default = 0 (disable) */
  uint16_t  ddrQueueId;     /**< Bounce queue where PASS will deliver the host-routed packet with DDR bit set */
  uint16_t  msmcQueueId;    /**< Bounce queue where PASS will deliver the host-routed packet with MSMC bit set */
  uint16_t  hwQueueBegin;   /**< Queue number of the first NetCP hardware queue */
  uint16_t  hwQueueEnd;     /**< Queue number of the last NetCP hardware queue */
  uint16_t  defOp[PA_MAX_QUEUE_BOUNCE_ROUTING_CLASSES];  /**< Default Queue Bounce operations per class */

} paQueueBounceConfig_t;

/**
 *  @defgroup  paQueueBounceControlBits PA Queue Bounce Control Bits and related definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   PA Queue Bounce Control Bits
 *  @brief  PA Queue Bounce Control Bits and related definitions
 */
/* @{ */
/**
 *  @def  pa_QUEUE_BOUNCE_CTRL_DEFAULT
 *        Use default rule
 */
#define  pa_QUEUE_BOUNCE_CTRL_DEFAULT   0

/**
 *  @def  pa_QUEUE_BOUNCE_CTRL_DDR
 *        Bounce to the DDR Queue
 */
#define  pa_QUEUE_BOUNCE_CTRL_DDR       1

/**
 *  @def  pa_QUEUE_BOUNCE_CTRL_MSMC
 *        Bounce to the MSMC queue
 */
#define  pa_QUEUE_BOUNCE_CTRL_MSMC      2

/**
 *  @def  pa_QUEUE_BOUNCE_CTRL_NONE
 *        No bounce, clear the control bits
 */
#define  pa_QUEUE_BOUNCE_CTRL_NONE      3

/**
 *  @def  pa_QUEUE_BOUNCE_CTRL_LOC
 *        Bit location of the queue bounce control bits
 */
#define pa_QUEUE_BOUNCE_CTRL_LOC        14

/**
 *  @def  pa_QUEUE_BOUNCE_QUEUE_MASK
 *        Actual queue number mask
 */
#define pa_QUEUE_BOUNCE_QUEUE_MASK      0x3FFF

/*@}*/
/** @} */

/**
 *  @defgroup PA_queue_bounce_op_macros  PA Queue Bounce Operation Macros
 *  @ingroup palld_api_macros
 *  @{
 *  @name PA Queue Bounce Operation Macros
 *  Macros used by the PA Queue Bounce Operation to insert/clear control bits
 */
/*@{*/
#define PA_BOUNCE_QUEUE_DDR(queueId)        (((queueId) & pa_QUEUE_BOUNCE_QUEUE_MASK) | (pa_QUEUE_BOUNCE_CTRL_DDR << pa_QUEUE_BOUNCE_CTRL_LOC))    /**< Insert control bits to indicate DDR queue bouncing */
#define PA_BOUNCE_QUEUE_MSMC(queueId)       (((queueId) & pa_QUEUE_BOUNCE_QUEUE_MASK) | (pa_QUEUE_BOUNCE_CTRL_MSMC << pa_QUEUE_BOUNCE_CTRL_LOC))   /**< Insert control bits to indicate MSMC queue bouncing */
#define PA_BOUNCE_QUEUE_NONE(queueId)       (((queueId) & pa_QUEUE_BOUNCE_QUEUE_MASK) | (pa_QUEUE_BOUNCE_CTRL_NONE << pa_QUEUE_BOUNCE_CTRL_LOC))   /**< Incert control bits to indicate no queue bouncing */
#define PA_BOUNCE_QUEUE_DEFAULT(queueId)    ((queueId) & pa_QUEUE_BOUNCE_QUEUE_MASK)    /**< Clear control bits to indicate default operation */

/*@}*/ /* PA_queue_bounce_op_macros */
/** @}*/ /* PA Queue Bounce Operation Macros */

/**
 * @ingroup palld_api_structures
 * @brief PA System Configuration Information structure
 *
 * @details paSysConfig_t contains pointers to the system-level configuration structures defined above. The null pointer 
 *          indicates the configuration of the corresponding sub-group is not required.
 */
typedef struct {
  paProtocolLimit_t*        pProtoLimit;           /**< Pointer to the protocol limit configuration structure */
  paIpReassmConfig_t*       pOutIpReassmConfig;    /**< Pointer to the outer IP PASS-assisted Reassembly configuration structure */
  paIpReassmConfig_t*       pInIpReassmConfig;     /**< Pointer to the inner IP PASS-assisted Reassembly configuration structure */
  paCmdSetConfig_t*         pCmdSetConfig;         /**< Pointer to the command set configuration structure */
  paUsrStatsConfig_t*       pUsrStatsConfig;       /**< Pointer to the user-defined statistics configuration structure */
  paQueueDivertConfig_t*    pQueueDivertConfig;    /**< Pointer to the queue-diversion configuration structure */
  paPacketControlConfig_t*  pPktControl;           /**< Pointer to the packet control configuration structure */
  paQueueBounceConfig_t*    pQueueBounceConfig;    /**< Pointer to the Queue Bounce configuration structure */
  paAclConfig_t*            pOutAclConfig;         /**< Pointer to the outer ACL configuration structure */
  paAclConfig_t*            pInAclConfig;          /**< Pointer to the inner ACL configuration structure */
  paRaGroupConfig_t*        pOutIpRaGroupConfig;   /**< Poimter to the outer IP Reassembly group configuration structure */
  paRaGroupConfig_t*        pInIpRaGroupConfig;    /**< Poimter to the inner IP Reassembly group configuration structure */
  paPacketControl2Config_t* pPktControl2;          /**< Pointer to the packet control 2 configuration structure */
  paEoamGlobalConfig_t*     pEoamConfig;           /**< Pointer to the EOAM Global configuration structure */
} paSysConfig_t;

/**
 *  @defgroup pa802p1agDetectInfo  PA 802.1ag Detector Control Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA 802.1ag Detector Control Bit Definitions
 *
 *  Bitmap definition of the ctrlBitMap in @ref pa802p1agDetConfig_t. 
 *         
 */ 
/*@{*/
/**
 *  @def  pa_802_1ag_DETECT_ENABLE
 *        Control Info -- Set: Enable 802.1ag Detector
 *                        Clear: Disable 802.1ag Detector
 */
#define pa_802_1ag_DETECT_ENABLE           0x0001 
/**
 *  @def  pa_802_1ag_DETECT_STANDARD
 *        Control Info -- Set: Perform 802.1ag packet detection per 802.1ag formal standard
 *                        Clear:  Perform 802.1ag packet detection per 802.1ag draft 
 */
#define pa_802_1ag_DETECT_STANDARD         0x0002 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief   802.1ag Detection Configuration Information.
 *
 *  @details  The 802.1ag packet can be recognized with ether type equal to 0x8902 normally. However, the PASS can be 
 *            configured to further qualify the IEEE 802.1ag packet per one of the following criteria: 
 *            - 802.1ag standard: Destion MAC address = 01-80-c2-00-00-3x, Ether type = 0x8902
 *            - 802.1ag draft: Destion MAC address = 01-80-c2-xx-xx-xx, Ether type = 0x8902 
 *
 *  @note The 802.1ag detector is disabled by default.
 *  @note refer to the @ref ErouteTypes for the corresponding exception routes.
 * 
 */
typedef struct {
  uint16_t ctrlBitMap;     /**< 802.1ag Detector control info as defined at @ref pa802p1agDetectInfo */
} pa802p1agDetConfig_t;


/**
 *  @defgroup ipsecNatTCtrlInfo  PA IPSEC NAT-T Control Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA IPSEC NAT-T Control Bit Definitions
 *
 *  Bitmap definition of the ctrlBitMap in @ref paIpsecNatTConfig_t. 
 *         
 */ 
/*@{*/
/**
 *  @def  pa_IPSEC_NAT_T_CTRL_ENABLE
 *        Control Info -- Set: Enable IPSEC NAT-T packet detection
 *                        Clear: Disable IPSEC NAT-T packet detection
 */
#define pa_IPSEC_NAT_T_CTRL_ENABLE           0x0001 
/**
 *  @def  pa_IPSEC_NAT_T_CTRL_LOC_LUT1
 *        Control Info -- Set: Perform IPSEC NAT-T packet detection at Ingress 1 (LUT1) stage
 *                        Clear: Perform IPSEC NAT-T packet detection at Ingress 4 (LUT2) stage (default)
 *
 *  @details The IPSEC ESP NAT-T packet detector is implemented at the processing stage (PDSP3) where 
 *           the LUT2 classification occurs at the first generation PASS. The drawback is that the 
 *           detected  IPSEC ESP NAT-T packet has to be re-routed into the PASS Outer IP processing 
 *           stage (PDSP1) for continuous processing and this operation reduces the overall throughput.
 *           In the 2nd generation PASS, the IPSEC NAT-T detector is implemented within Ingress 1 
 *           (Outer IP and IPSEC) processing stage to avoid the re-entry operation. However, the detector
 *           is also implemented at the Ingress4 (LUT2) stage to maintain backward compatibility.
 *           It is recommended to set this flag to one to enable the IPSEC ESP NAT-T detector at Ingress 1
 *           stage to maintain the maximum PASS throughput.
 *
 *  @note: This feature is only supported by the second generation of PASS and this control bit will be 
 *         ignored at the device which uses the first generation of PASS.
 *         
 */
#define pa_IPSEC_NAT_T_CTRL_LOC_LUT1         0x0002 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief   IPSEC NAT-T Packet Detection Configuration Information.
 *
 *  @details  paIpsecNatTConfig_t is used to configure the IPSEC NAT-T packet detector which is disabled
 *            until this configuration is invoked through API @ref Pa_control. 
 *
 *  @note The IPSEC NAT-T packet detector is disabled by default.
 *  @note refer to the @ref ErouteTypes for the corresponding exception routes.
 * 
 */
typedef struct {

  uint16_t ctrlBitMap;     /**< IPSEC NAT-T control info as defined at @ref ipsecNatTCtrlInfo */
  uint16_t udpPort;        /**< Specify the UDP port number which uniquely identifies the IPSEC NAT-T packets */
} paIpsecNatTConfig_t;

/**
 *  @defgroup paGtpuCtrlInfo  PA GTPU Control Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA GTPU Control Bit Definitions
 *
 *  Bitmap definition of the ctrlBitmap in @ref paGtpuConfig_t. 
 *         
 */ 
/*@{*/
/**
 *  @def  pa_GTPU_CTRL_USE_LINK
 *        Control Info -- Set: GTU-U classification vector consists of the least significant 24-bit of tunnel ID and 8-bit link 
 *                             of previous matching
 *                        Clear: GTU-U classification vector consists of the 32-bit of tunnel ID only (Default)
 */
#define pa_GTPU_CTRL_USE_LINK              0x0001 

/**
 *  @def  pa_GTPU_CTRL_ROUTE_END_MARKER_AS_GPDU
 *        Control Info -- Set: Configures the GTP-U message routing rule such that the packets with message type 254 (end markers)
 *                             are routed the same way as message type 255 (G-PDU), meaning GTPU TEID would be recovered and routed for 
 *                             LUT2 match.
 *                        Clear: End message routing send to a configured flow/exception route (Default)
 */
#define pa_GTPU_CTRL_ROUTE_END_MARKER_AS_GPDU  0x0002 

/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief   GTP-U Configuration Information.
 *
 *  @details  Due to the LUT2 engine using 32-bit matching parameter, the default GTP-U classification is solely based 
 *            on its 32-bit tunnel ID. However, it is desirable to match the GTP-U tunnel with both tunnel ID and 
 *            previous link information. This configuration can be used to modify GTP-U classification vector by 
 *            combining least significant 24-bit of tunnel ID and an 8-bit previous link. It should be passed to
 *            @ref Pa_control() API at system startup.
 *
 *  @note GTP-U configuration should be performed at system startup. PASS does not support GTP-U 
 *        reconfiguration at run time.
 *  @note This configuration is used at the first generation of PASS and it is still supported by the second generation PASS 
 *        for backward compatibility only. It does not have real effect since the advanced LUT2 engine supports GTPU 32-bit 
 *        Tunnel-ID classification with L3 link. It is not necessary to restrict the effective tunnel-ID to 24-bit. 
 * 
 */
typedef struct {
  uint16_t ctrlBitMap;     /**< GTP-U configuration control info as defined at @ref paGtpuCtrlInfo */
} paGtpuConfig_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  The return type for module functions
 *
 *  @details  Function calls to this module return values used to determine if the command was successful or
 *            the reason for failure (see @ref ReturnValues).
 */

typedef int paReturn_t;

/**
 *   @ingroup palld_api_structures
 *   @brief  paCmdReply_t is used to specify command result (from PASS) routing information
 *
 *   @details Commands sent to packet accelerator sub-system will generate replies. These replies
 *            can be either discarded by the sub-system or routed to a queue. Command replies that
 *            must be forwarded back to this module are detailed for each command. The module user
 *            typically either selects a unique destination queue for command replies, or else supplies
 *            a unique value for replyId. This value is placed into software info word 0 in the 
 *            packet descriptor for the returned command. The data in the returned packet is not
 *            typically examined by the module user, but passed directly back to this module through 
 *            API function @ref Pa_forwardResult to examine the results of the command.
 */
typedef struct  {

  int       dest;        /**<  Packet destination, must be pa_DEST_HOST or pa_DEST_DISCARD, see @ref pktDest */
  uint32_t  replyId;     /**<  Value placed in swinfo0 in reply packet */
  uint16_t  queue;       /**<  Destination queue for destination pa_DEST_HOST */
  uint8_t   flowId;      /**<  Flow ID used on command reply from PASS */
  
} paCmdReply_t;

/**
 *  @ingroup palld_api_constants
 *  @brief   Define the maximum number of buffers the module can request
 *
 */
#define pa_N_BUFS_GEN1          5
#define pa_N_BUFS_GEN2          8
 
#define pa_N_BUFS               pa_N_BUFS_GEN2

/**
 *  @defgroup  paBufIndex PA Memory Buffer Index
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   PA Memory Buffer Index
 *  @brief  Define the buffer inedex of the PA LLD memory blocks.
 *
 */
/* @{ */
/**
 *  @def  pa_BUF_INST
 *        PA LLD instance buffer
 */
#define pa_BUF_INST             0
/**
 *  @def  pa_BUF_L2_TABLE
 *        PA LLD match table of Layer 2 (MAC/SRIO) entries
 */
#define pa_BUF_L2_TABLE         1
/**
 *  @def  pa_BUF_L3_TABLE
 *        PA LLD match table of Layer 3 (IP/CustomLUT1) entries
 */
#define pa_BUF_L3_TABLE         2
/**
 *  @def  pa_BUF_USR_STATS_TABLE
 *        PA LLD link table of user-defined statistics 
 */
#define pa_BUF_USR_STATS_TABLE  3
/**
 *  @def  pa_BUF_VLINK_TABLE
 *        PA LLD match table of virtual link entries
 */
#define pa_BUF_VLINK_TABLE      4
/**
 *  @def  pa_BUF_ACL_TABLE
 *        PA LLD match table of ACL entries 
 *
 *  @note This definition is valid for the second generation PASS only.
 */
#define pa_BUF_ACL_TABLE        5
/**
 *  @def  pa_BUF_FC_TABLE
 *        PA LLD match table of Flow Cache entries 
 *
 *  @note This definition is valid for the second generation PASS only.
 */
#define pa_BUF_FC_TABLE         6

/**
 *  @def  pa_BUF_EOAM_TABLE
 *        PA LLD match table of EOAM entries such as Y1731
 *        Please refer to @ref appendix8 for details about EOAM mode.*
 *  @note This definition is valid for the second generation PASS only.
 */
#define pa_BUF_EOAM_TABLE       7

/*  @}  */  
/** @} */


/**
 *  @ingroup palld_api_functions
 *  @brief Pa_getBufferReq returns the memory requirements for the PA driver
 *
 *  @details This function returns the memory buffer requirements in term
 *           of the size and alignment array. The PA LLD requires up to 
 *           four memory blocks as described below:
 *           - PA Instance: PA instance data
 *           - L2 Table: Layer-2 (MAC/SRIO) entry information
 *           - L3 Table: Layer-3 (IP/Custom LUT1) entry information
 *           - User Statistics Link Table: User-defined Statistics entry information (Optional)
 *
 *  @param[in]   sizeCfg     Size configuration information
 *  @param[out]  sizes       Array of size requirements
 *  @param[out]  aligns      Array of alignment requirements
 *  @retval                  Value (@ref ReturnValues)
 *
 *  @note This function specifies the minimum memory buffer requirements, it is up to the
 *        module user to round up the buffer alignemnt and size to the cache line boundary
 *        to ensure cache coherency if cacheable memory is used.
 */
paReturn_t Pa_getBufferReq (paSizeInfo_t *sizeCfg, int sizes[], int aligns[]);

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_create creates the PA driver instance
 *
 *  @details This function initializes the PA driver based on user configuration
 *
 *  @param[in]  cfg     Configuration information
 *  @param[in]  bases   Array of the memory buffer base addresses 
 *  @param[out] pHandle Instance handle. This is a pointer to an initialized
 *                      instance structure. 
 *  @retval             Value (@ref ReturnValues)
 */
paReturn_t Pa_create (paConfig_t *cfg, void* bases[], Pa_Handle *pHandle);

/**
 *  @ingroup palld_api_functions
 *  @brief   Pa_startCfg Adds PA configuration
 *  @details This function needs to be called from all cores to initialize PA with 
 *           per core configurations
 *
 *  @param[in]  handle  The PA LLD instance identifier
 *  @param[in]  startCfg PA start configuration
 *  @retval             Value (@ref ReturnValues)
 */
paReturn_t Pa_startCfg (Pa_Handle handle, paStartCfg_t *startCfg);

/**
 *  @ingroup palld_api_functions
 *  @brief Pa_close decativates the PA driver instance
 *
 *  @details This function deactivates the PA driver instance, all the associated
 *           memory buffers can be freed after this call. 
 *
 *  @param[in]  handle  The PA LLD instance identifier
 *  @param[out] bases   Array of the memory buffer base addresses 
 *  @retval             Value (@ref ReturnValues)
 */
paReturn_t Pa_close (Pa_Handle handle, void* bases[]);

/**
 * @defgroup  pktDest Routed Packet Destinations
 * @ingroup palld_api_constants
 * @{
 *
 * @name Routed Packet Destinations
 *
 * @brief The module user specifies packet destinations for packets exiting the packet accelerator sub-system.
 *
 * @details  The destination of packets that leave the packet accelerator sub-system
 *           are provided to the module in the @ref paRouteInfo_t structure and passed
 *           to the module through the @ref Pa_addMac, @ref Pa_addSrio, @ref Pa_addIp, @ref Pa_addCustomLUT1, 
 *           @ref Pa_addCustomLUT2 and @ref Pa_addPort functions
 */
/** @ingroup pktDest */
/* @{ */

/** 
 *  @def  pa_DEST_DISCARD
 *        packet is discarded
 */
#define  pa_DEST_DISCARD  3  /**< Packet is discarded */

/** 
 *  @def  pa_DEST_CONTINUE_PARSE_LUT1
 *        packet remains in PA sub-system for more parsing and LUT1 classification
 */
#define  pa_DEST_CONTINUE_PARSE_LUT1  4 /**< Packet remains in PA sub-system for more parsing and LUT1 classification */

/** 
 *  @def  pa_DEST_CONTINUE_PARSE_LUT2
 *        packet remains in PA sub-system for more parsing and LUT2 classification. 
 */
#define  pa_DEST_CONTINUE_PARSE_LUT2  5  /**< Packet remains in PA sub-system for more parsing and LUT2 classification */

/**
 *  @def  pa_DEST_HOST
 *        host thread 
 */
#define  pa_DEST_HOST   6   /**< Packet is routed to host */

/** 
 *  @def  pa_DEST_EMAC
 *        ethernet mac port (of the switch)
 */
#define  pa_DEST_EMAC   7   /**< Packet is routed to  EMAC */

/** 
 *  @def  pa_DEST_SASS
 *        security accelerator destination 
 */
#define  pa_DEST_SASS   8   /**< Packet is routed to SA */

/** 
 *  @def  pa_DEST_SASS_LOC_DMA
 *        security accelerator destination via local DMA
 *
 *  @note This definition is valid for the second generation of PASS only.
 */
#define  pa_DEST_SASS_LOC_DMA   11   /**< Packet is routed to SA through local DMA */

/** 
 *  @def  pa_DEST_SRIO
 *        SRIO interface
 */
#define  pa_DEST_SRIO   9   /**< Packet is routed to SRIO */

/** 
 *  @def  pa_DEST_CASCADED_FORWARDING_LUT1
 *        Cascaded forwarding packet remains in PA sub-system for next LUT1 (IP) parsing. Those packets are expected to
 *        be delivered to QoS queues based on the VLAN/DSCP priority at the next stage so that some PASS actions such
 *        as IP reassembly and IP fragment exception route will be disabled.
 */
#define  pa_DEST_CASCADED_FORWARDING_LUT1  10  

/** 
 *  @def  pa_DEST_EFLOW
 *        packet remains in PA sub-system for egress flow operation
 *
 *  @note This definition is valid for the second generation of PASS only.
 */
#define  pa_DEST_EFLOW   12   /**< Packet is routed to Egress Flow Path */

/** 
 *  @def  pa_DEST_RES_1
 *        Reseved destination for internal usage
 *
 *  @note This definition is valid for the second generation of PASS only.
 */
#define  pa_DEST_RES_1  20   

/** 
 *  @def  pa_DEST_RES_2
 *        Reseved destination for internal usage
 *
 *  @note This definition is valid for the second generation of PASS only.
 */
#define  pa_DEST_RES_2  21   

  
/*  @}  */  
/** @} */

/**
 * @defgroup  paEmacPort Ethernet MAC port
 * @ingroup palld_api_constants
 * @{
 *
 * @name Ethernet MAC port
 *
 * @brief The module user specifies the Ethernet MAC port of the ingress and egress packets.
 *
 * @details  In the from-network direction, the module user can specify the input port as one of classification parameters.
 *           In the to-network direction, the module user can force the egress packets to be sent over the specified 
 *           destination Ethernet MAC port of the switch regreless of its states or configurations.
 */
/** @ingroup customType */
/* @{ */
/**
 *  @def  pa_EMAC_PORT_NOT_SPECIFIED
 *        From-Netwprk: Don't care
 *        To-Network: Use standard switch forwarding 
 */
#define  pa_EMAC_PORT_NOT_SPECIFIED   0

/* @def   pa_EMAC_PORT_0
 *        Use EMAC Port 0
 */
#define  pa_EMAC_PORT_0               1   

/* @def   pa_EMAC_PORT_1
 *        Use EMAC Port 1
 */
#define  pa_EMAC_PORT_1               2   

/* @def   pa_EMAC_PORT_2
 *        Use EMAC Port 2
 */
#define  pa_EMAC_PORT_2               3   

/* @def   pa_EMAC_PORT_3
 *        Use EMAC Port 3
 */
#define  pa_EMAC_PORT_3               4   

/* @def   pa_EMAC_PORT_4
 *        Use EMAC Port 4
 */
#define  pa_EMAC_PORT_4               5   

/* @def   pa_EMAC_PORT_5
 *        Use EMAC Port 5
 */
#define  pa_EMAC_PORT_5               6   

/* @def   pa_EMAC_PORT_6
 *        Use EMAC Port 6
 */
#define  pa_EMAC_PORT_6               7   

/* @def   pa_EMAC_PORT_7
 *        Use EMAC Port 7
 */
#define  pa_EMAC_PORT_7               8

/* @def   pa_CPPI_PORT
 *        Use CPPI PORT
 */
#define  pa_CPPI_PORT                 pa_EMAC_PORT_NOT_SPECIFIED

/*  @}  */  
/** @} */

/**
 * @defgroup emcOutputCtrlBits Ethernet MAC Output Control Bit Definitions
 * @ingroup palld_api_constants
 * @{
 *
 * @name Ethernet MAC Output Control Bit Definition
 *
 * Bitmap definition of the emacCtrl at @ref paRouteInfo_t. 
 *  
 */ 
/*@{*/
/**
 *  @def  pa_EMAC_CTRL_PORT_MASK
 *        Control Info -- EMAC port mask
 */
#define pa_EMAC_CTRL_PORT_MASK            0x0F 
/**
 *  @def  pa_EMAC_CTRL_CRC_DISABLE
 *        Control Info -- 0:EMAC port computes and inserts CRC
 *                        1:EMAC port does not generate CRC  
 */
#define pa_EMAC_CTRL_CRC_DISABLE          0x80 

/*  @}  */  
/** @} */

/**
 * @defgroup  customType Custom Classification Types
 * @ingroup palld_api_constants
 * @{
 *
 * @name Custom Classification Types
 *
 * @brief The module user specifies the custom classification types.
 *
 * @details  The optional custom classification rule may be used to further parse and calssify the incoming
 *           packet.
 */
/** @ingroup customType */
/* @{ */
/**
 *  @def  pa_CUSTOM_TYPE_NONE
 *        Use standard classification 
 */
#define  pa_CUSTOM_TYPE_NONE   0   

/* @def   pa_CUSTOM_TYPE_LUT1
 *        Custom classification with LUT1
 */
#define  pa_CUSTOM_TYPE_LUT1   1   

/* @def   pa_CUSTOM_TYPE_LUT2
 *        Custom classification with LUT2
 */
#define  pa_CUSTOM_TYPE_LUT2   2   

/*  @}  */  
/** @} */

/**
 *   @brief  The maximum number of LUT1 Custom Types supported
 */
#define pa_MAX_CUSTOM_TYPES_LUT1   4


/**
 *   @brief  The maximum number of LUT2 Custom Types supported
 */
#define pa_MAX_CUSTOM_TYPES_LUT2   16


/**
 * @defgroup  cmdTxDestGen1 Command/Transmit Packet Destinations for first generation NSS
 * @ingroup palld_api_constants
 * @{
 *
 * @name Command/Transmit Packet Destinations for first generation NSS
 *
 * @brief These values specify the offsets to the NSS Tx base queue and they are used by the module user to deliver 
 *        the configuration packets to the specific PDSP Cluster within PASS.
 *
 * @note These values are used by LLD as the return value of cmdDest of PASS configuration APIs. They are defined here
 *       for reference purpose only.
 */
/* @{ */
/**
 *   @def  pa_CMD_TX_DEST_0_GEN1
 *         Destination PDSP0
 */
#define  pa_CMD_TX_DEST_0_GEN1    0  /**< Packet is sent to PDSP0 */

/**
 *   @def  pa_CMD_TX_DEST_1_GEN1
 *         Destination PDSP1
 */
#define  pa_CMD_TX_DEST_1_GEN1    1  /**< Packet is sent to PDSP1 */

/**
 *   @def  pa_CMD_TX_DEST_2_GEN1
 *         Destination PDSP2
 */
#define  pa_CMD_TX_DEST_2_GEN1    2  /**< Packet is sent to PDSP2 */

/**
 *   @def  pa_CMD_TX_DEST_3_GEN1
 *         Destination PDSP3
 */
#define  pa_CMD_TX_DEST_3_GEN1    3  /**< Packet is sent to PDSP3 */

/**
 *   @def  pa_CMD_TX_DEST_4_GEN1
 *         Destination PDSP4
 */
#define  pa_CMD_TX_DEST_4_GEN1    4  /**< Packet is sent to PDSP4 */

/**
 *   @def  pa_CMD_TX_DEST_5_GEN1
 *         Destination PDSP5
 */
#define  pa_CMD_TX_DEST_5_GEN1    5  /**< Packet is sent to PDSP5  */

/*  @}  */  
/** @} */

/**
 * @defgroup  cmdTxDestGen2 Command/Transmit Packet Destinations for second generation NSS
 * @ingroup palld_api_constants
 * @{
 *
 * @name Command/Transmit Packet Destinations for second generation NSS
 *
 * @brief These values specify the offset to the NSS Tx base queue and they are used by the module user to deliver 
 *        the configuration packets to the specific PDSP Cluster within PASS.
 *
 * @note These values are used by LLD as the return value of cmdDest of PASS configuration APIs. They are defined here
 *       for reference purpose only.
 */
/* @{ */
/**
 *   @def  pa_CMD_TX_DEST_0_GEN2
 *         Destination CLUSTER0
 */
#define  pa_CMD_TX_DEST_0_GEN2    8  /**< Packet is sent to INGRESS0 */

/**
 *   @def  pa_CMD_TX_DEST_1_GEN2
 *         Destination CLUSTER1
 */
#define  pa_CMD_TX_DEST_1_GEN2    9  /**< Packet is sent to INGRESS1 */

/**
 *   @def  pa_CMD_TX_DEST_2_GEN2
 *         Destination CLUSTER2
 */
#define  pa_CMD_TX_DEST_2_GEN2   10  /**< Packet is sent to INGRESS2 */

/**
 *   @def  pa_CMD_TX_DEST_3_GEN2
 *         Destination CLUSTER3
 */
#define  pa_CMD_TX_DEST_3_GEN2   11  /**< Packet is sent to INGRESS3 */

/**
 *   @def  pa_CMD_TX_DEST_4_GEN2
 *         Destination CLUSTER4
 */
#define  pa_CMD_TX_DEST_4_GEN2   12  /**< Packet is sent to INGRESS4 */

/**
 *   @def  pa_CMD_TX_DEST_5_GEN2
 *         Destination CLUSTER5
 */
#define  pa_CMD_TX_DEST_5_GEN2   13  /**< Packet is sent to POST  */

/**
 *   @def  pa_CMD_TX_DEST_6_GEN2
 *         Destination CLUSTER6
 */
#define  pa_CMD_TX_DEST_6_GEN2   14  /**< Packet is sent to EGRESS0  */
 
/**
 *   @def  pa_CMD_TX_DEST_7_GEN2
 *         Destination CLUSTER7
 */
#define  pa_CMD_TX_DEST_7_GEN2   15  /**< Packet is sent to EGRESS1  */
/**
 *   @def  pa_CMD_TX_DEST_8_GEN2
 *         Destination CLUSTER8
 */
#define  pa_CMD_TX_DEST_8_GEN2   16  /**< Packet is sent to EGRESS2  */
 
/*  @}  */  
/** @} */

/**
 * @defgroup  cmdTxDest Command/Transmit Packet Destinations for NSS
 * @ingroup palld_api_constants
 * @{
 *
 * @name Command/Transmit Packet Destinations for NSS
 *
 * @brief Define the command destination based on the compiler switch NSS_GEN2 to cover both @ref cmdTxDestGen1
 *        and @ref cmdTxDestGen2. These values are used by the LLD only and are not required by the application.
 */
/* @{ */
#ifndef NSS_GEN2
#define  pa_CMD_TX_DEST_0        pa_CMD_TX_DEST_0_GEN1
#define  pa_CMD_TX_DEST_1        pa_CMD_TX_DEST_1_GEN1
#define  pa_CMD_TX_DEST_2        pa_CMD_TX_DEST_2_GEN1
#define  pa_CMD_TX_DEST_3        pa_CMD_TX_DEST_3_GEN1
#define  pa_CMD_TX_DEST_4        pa_CMD_TX_DEST_4_GEN1
#define  pa_CMD_TX_DEST_5        pa_CMD_TX_DEST_5_GEN1
#else
#define  pa_CMD_TX_DEST_0        pa_CMD_TX_DEST_0_GEN2
#define  pa_CMD_TX_DEST_1        pa_CMD_TX_DEST_1_GEN2
#define  pa_CMD_TX_DEST_2        pa_CMD_TX_DEST_2_GEN2
#define  pa_CMD_TX_DEST_3        pa_CMD_TX_DEST_3_GEN2
#define  pa_CMD_TX_DEST_4        pa_CMD_TX_DEST_4_GEN2
#define  pa_CMD_TX_DEST_5        pa_CMD_TX_DEST_5_GEN2
#define  pa_CMD_TX_DEST_6        pa_CMD_TX_DEST_6_GEN2
#define  pa_CMD_TX_DEST_7        pa_CMD_TX_DEST_7_GEN2
#define  pa_CMD_TX_DEST_8        pa_CMD_TX_DEST_8_GEN2
#endif

/*  @}  */  
/** @} */

/**
 * @defgroup  paLut1Inst PA LUT1 Instance Destinations
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA LUT1 Instance Destinations
 *
 * @brief These values are used by the module user to specify the LUT1 table instance used by the specified IP, ACL or customLUT1 entry.
 * @note PA LLD will determine the appropriate LUT1 instance to add/configure LUT1 entry based on the types of API and the linking information 
 *       in normal operation, i.e. when lutInst is set to pa_LUT_INST_NOT_SPECIFIED. These values are only used by module users, who want to maintain their own LUT1 tables, 
 *       to overwrite the default rules.  
 */
/* @{ */
/**
 *   @def  pa_LUT1_INST_0_0
 *         LUT1 instance of Ingress0, PDSP0
 */
#define  pa_LUT1_INST_0_0    0  /**< LUT1 table connected to Ingress0, PDSP0 */

/**
 *   @def  pa_LUT1_INST_0_1
 *         LUT1 instance of Ingress0, PDSP1
 */
#define  pa_LUT1_INST_0_1    1  /**< LUT1 table connected to Ingress0, PDSP1 */

/**
 *   @def  pa_LUT1_INST_1_0
 *         LUT1 instance of Ingress1, PDSP0
 */
#define  pa_LUT1_INST_1_0    2  /**< LUT1 table connected to Ingress1, PDSP0 */

/**
 *   @def  pa_LUT1_INST_0_1
 *         LUT1 instance of Ingress1, PDSP1
 */
#define  pa_LUT1_INST_1_1    3  /**< LUT1 table connected to Ingress1, PDSP1 */

/**
 *   @def  pa_LUT1_INST_2_0
 *         LUT1 instance of Ingress2, PDSP0
 */
#define  pa_LUT1_INST_2_0    4  /**< LUT1 table connected to Ingress2, PDSP0 */

/**
 *   @def  pa_LUT1_INST_3_0
 *         LUT1 instance of Ingress3, PDSP0
 */
#define  pa_LUT1_INST_3_0    5  /**< LUT1 table connected to Ingress3, PDSP0 */

/**
 *   @def  pa_LUT1_INST_4_0
 *         LUT1 instance of Ingress4, PDSP0
 */
#define  pa_LUT1_INST_4_0    6  /**< LUT1 table connected to Ingress4, PDSP0 */

/**
 *   @def  pa_LUT1_INST_5_0
 *         LUT1 instance of Egress0, PDSP0
 */
#define  pa_LUT1_INST_5_0    7  /**< LUT1 table connected to Egress0, PDSP0 */


/**< LUT1 instances of First Generation PASS */
#define  pa_LUT1_INST_0_GEN1    0   /**< LUT1 table connected to PDSP0 (PASS Gen1)*/
#define  pa_LUT1_INST_1_GEN1    1   /**< LUT1 table connected to PDSP1 (PASS Gen1)*/
#define  pa_LUT1_INST_2_GEN1    2   /**< LUT1 table connected to PDSP2 (PASS Gen1)*/
#define  pa_LUT1_INST_MAX_GEN1  pa_LUT1_INST_2_GEN1   


/**< LUT1 instances of Second Generation PASS */
#define  pa_LUT1_INST_0_GEN2    pa_LUT1_INST_0_0   /**< LUT1 table equivalent to Netcp 1.0 LUT1_0  (Pass Gen2)*/
#define  pa_LUT1_INST_1_GEN2    pa_LUT1_INST_1_0   /**< LUT1 table equivalent to Netcp 1.0 LUT1_1  (Pass Gen2)*/
#define  pa_LUT1_INST_2_GEN2    pa_LUT1_INST_4_0   /**< LUT1 table equivalent to Netcp 1.0 LUT1_2  (Pass Gen2)*/
#define  pa_LUT1_INST_MAX_GEN2  pa_LUT1_INST_5_0   

/**
 *
 * @name Common LUT1 instance for NSS
 *
 * @brief Define the LUT1 instance based on the compiler switch NSS_GEN2 to cover both generations of NSS.
 *        These values are intended to be used by the LLD only. For the application which maintain the LUT1
 *        tables should either use the LUT1 instance definitions with _GEN1 and _GEN2 suffix or these definitions
 *        with the compiler switch NSS_GEN2 defined or undefined.
 */


#ifndef NSS_GEN2
/**
 *   @def  pa_LUT1_INST_0
 *         LUT1 instance 0
 */
#define  pa_LUT1_INST_0    pa_LUT1_INST_0_GEN1   /**< LUT1 Instance 0 for MAC/SRIO */

/**
 *   @def  pa_LUT1_INST_1
 *         LUT1 instance 1
 */
#define  pa_LUT1_INST_1    pa_LUT1_INST_1_GEN1   /**< LUT1 instance 1 for Outer IP */

/**
 *   @def  pa_LUT1_INST_2
 *         LUT1 instance 2
 */
#define  pa_LUT1_INST_2    pa_LUT1_INST_2_GEN1   /**< LUT1 instance 2 for Inner IP */

/**
 *   @def  pa_LUT1_INST_MAX
 *         Specify the maximum LUT1 instance
 */
#define  pa_LUT1_INST_MAX  pa_LUT1_INST_MAX_GEN1   

#else

/**
 *   @def  pa_LUT1_INST_0
 *         LUT1 instance 0
 */
#define  pa_LUT1_INST_0    pa_LUT1_INST_0_GEN2   /**< LUT1 Instance 0  for MAC/SRIO */

/**
 *   @def  pa_LUT1_INST_1
 *         LUT1 instance 1
 */
#define  pa_LUT1_INST_1    pa_LUT1_INST_1_GEN2   /**< LUT1 instance 1  for Outer IP */

/**
 *   @def  pa_LUT1_INST_2
 *         LUT1 instance 2
 */
#define  pa_LUT1_INST_2    pa_LUT1_INST_2_GEN2   /**< LUT1 Instance 2  for Inner IP */

/**
 *   @def  pa_LUT1_INST_MAX
 *         Specify the maximum LUT1 instance
 */
#define  pa_LUT1_INST_MAX  pa_LUT1_INST_MAX_GEN2   

#endif
/*  @}  */  
/** @} */

/**
 * @defgroup  paAclInst PA ACL LUT Instance Destinations
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA ACL Lut Instance Destinations
 *
 * @brief These values are used by the module user to specify the ACL Lut instance 
 *
 * @note These definitions are valid for the second generation PASS only.
 */
/* @{ */

/**
 *   @def  pa_ACL_INST_OUTER_IP
 *         LUT1 instance of ACL Table 0 for Outer IP
 */
#define  pa_ACL_INST_OUTER_IP     pa_LUT1_INST_0_1   /**< LUT1 table used for ACL Table 0  */


/**
 *   @def  pa_ACL_INST_INNER_IP
 *         LUT1 instance of ACL Table 1 for Inner IP
 */
#define  pa_ACL_INST_INNER_IP     pa_LUT1_INST_3_0   /**< LUT1 table used for ACL Table 1  */
 
/*  @}  */  
/** @} */


/**
 * @defgroup  paCrcInst PA CRC Engine Instance Destinations
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA CRC Engine Instance Destinations
 *
 * @brief These values are used by the module user to specify the CRC Engine instance 
 *
 * @note These definitions are valid for the second generation PASS only.
 */
/* @{ */
/**
 *   @def  pa_CRC_INST_0_0
 *         CRC instance of Ingress0
 */
#define  pa_CRC_INST_0_0    0  /**< CRC Engine between Ingress0, CDE0 and CED1 */

/**
 *   @def  pa_CRC_INST_1_0
 *         CRC instance of Ingress1
 */
#define  pa_CRC_INST_1_0    1  /**< CRC Engine between Ingress1, CDE0 and CED1 */

/**
 *   @def  pa_CRC_INST_4_0
 *         LUT1 instance of Ingress4
 */
#define  pa_CRC_INST_4_0    2  /**< CRC Engine between Ingress4, CDE0 and CED1 */

/**
 *   @def  pa_CRC_INST_5_0
 *         LUT1 instance of Post
 */
#define  pa_CRC_INST_5_0    3  /**< CRC Engine between Post, CDE0 and CED1 */

/**
 *   @def  pa_CRC_INST_6_0
 *         CRC instance 0 of Egress0
 */
#define  pa_CRC_INST_6_0    4  /**< CRC Engine between Egress0, CDE0 and CED1 */

/**
 *   @def  pa_CRC_INST_6_1
 *         CRC instance 1 of Egress0
 */
#define  pa_CRC_INST_6_1    5  /**< CRC Engine between Egress0, CDE1 and CED2 */

/**
 *   @def  pa_CRC_INST_MAX
 *         Specify the maximum CRC Engine instance
 */
#define  pa_CRC_INST_MAX  pa_CRC_INST_6_1   
 
/*  @}  */  
/** @} */

/**
 * @defgroup  paRaInst PA RA Instance Destinations
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA RA Instance Destinations
 *
 * @brief These values are used by the module user to specify the RA instance (group)
 *
 * @note These definitions are valid for the second generation PASS only.
 */
/* @{ */
/**
 *   @def  pa_RA_INST_0
 *         RA instance of Outer IP
 */
#define  pa_RA_INST_0       0  /**< RA instance to be accessed from Ingress0, PDSP1 for outer IP reassembly */


/**
 *   @def  pa_RA_INST_1
 *         RA instance of Inner IP
 */
#define  pa_RA_INST_1       1  /**< RA instance to be accessed from Ingress3, PDSP0 for inner IP reassembly */

/**
 *   @def  pa_RA_INST_MAX
 *         Specify the maximum RA instance
 */
#define  pa_RA_INST_MAX     pa_RA_INST_1   
 
/*  @}  */  
/** @} */

/**
 * @defgroup  paCmdCode Command Code
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA Command Codes
 *
 * @brief Define the commands which can be executed in PASS 
 *
 * @details  A single command or a set of commands can be executed to support fully-offloaded 
 *           data path in both the transmit (to network) and receive (from network) directions.  
 *           In the to-network direction, the list of commands formatted by the module should 
 *           be stored as the protocol-specific information at the packet descriptor with the 
 *           packet. The commands will be executed in order at PASS and the associated security
 *           accelerator sub-system (SASS). The executed commands will be removed by PASS and 
 *           SASS so that the output packet will not contain any command.
 *           In the from-network direction, the list of commands formatted by the module will 
 *           be stored at the PASS as a command set which can be referred to by the command set 
 *           index. A single command including a command set can be executed per the enhanced 
 *           routing information @ref paRouteInfo_t after a LUT1 or LUT2 matches.
 *
 * @note     The packet offset specified at each command of the command list should be strictly 
 *           in ascending order becasue the PASS processes the list of commands in order and it
 *           can not move backwards. The command violating the order requirement may be detected
 *           and rejected by the API @ref Pa_formatTxCmd and @ref Pa_configCmdSet. In the case, 
 *           the order constraint can not be validated at the LLD, the violating command will 
 *           be ignored by the PASS.    
 */
/** @ingroup paCmdCode */
/* @{ */
/**
 *  @def  pa_CMD_NONE
 *        End of commands 
 */
#define  pa_CMD_NONE                        0   

/* @def   pa_CMD_NEXT_ROUTE
 *        Specifies next route 
 */
#define  pa_CMD_NEXT_ROUTE                  1   

/*  @def  pa_CMD_CRC_OP
 *        CRC generation or verification 
 */
#define  pa_CMD_CRC_OP                      2   

/*  @def  pa_CMD_COPY_DATA_TO_PSINFO
 *        Copy Data from the packet to the PS Info Area in the packet descriptor 
 */
#define  pa_CMD_COPY_DATA_TO_PSINFO         3

/*  @def  pa_CMD_PATCH_DATA
 *        Insert or patch packet data at the specific location  
 */
#define  pa_CMD_PATCH_DATA                  4  

/*  @def  pa_CMD_TX_CHECKSUM
 *        Compute and insert checksum  
 */
#define  pa_CMD_TX_CHECKSUM                 5 

/*  @def  pa_CMD_MULTI_ROUTE
 *        Duplicate packet to multiple destinations  
 */
#define  pa_CMD_MULTI_ROUTE                 6  

/*  @def  pa_CMD_REPORT_TX_TIMESTAMP
 *        Report the tx packet exit time in term of PASS 48-bit timestamp
 */
#define  pa_CMD_REPORT_TX_TIMESTAMP         7 

/*  @def  pa_CMD_REMOVE_HEADER
 *        Remove the parsed packet header 
 *  @note It should be the first command in the rx command set 
 */
#define  pa_CMD_REMOVE_HEADER               8 

/*  @def  pa_CMD_REMOVE_TAIL
 *        Remove the parsed packet tail 
 *
 *  @note It should be the last command next to the next route or multi-route command       
 */
#define  pa_CMD_REMOVE_TAIL                 9 


/*  @def  pa_CMD_CMDSET
 *        Specify the command set to be executed  
 */
#define  pa_CMD_CMDSET                      10   

/*  @def  pa_CMD_SA_PAYLOAD
 *        Specify the payload information required by SASS  
 */
#define  pa_CMD_SA_PAYLOAD                  11   

/*  @def  pa_CMD_IP_FRAGMENT
 *        Perform IPv4 fragmentation  
 */
#define  pa_CMD_IP_FRAGMENT                 12 

/*  @def  pa_CMD_USR_STATS
 *        Update the specified user-defined counter and the counters which are linked to this counter  
 */
#define  pa_CMD_USR_STATS                   13    
   
   
/*  @def  pa_CMD_CMDSET_AND_USR_STATS
 *        Combination of the CMDSET and USR_STATS commands. 
 *  @note It is only used as a command executed after the last classification per the enhanced routing 
 *        information      
 */
#define  pa_CMD_CMDSET_AND_USR_STATS        14   

/*  @def  pa_CMD_PATCH_MSG_LEN
 *        Update the message length field within some L2 protocol header such as 802.3 and PPPoE after the
 *        potential IP fragmentation operation
 *  @note This command is only used in conjunction with the pa_CMD_IP_FRAGMENT command.    
 */
#define  pa_CMD_PATCH_MSG_LEN               15 

/*  @def  pa_CMD_VERIFY_PKT_ERROR
 *        Verify the packet error based on the CPPI error flags as specified at @ref appendix2 and forward
 *        the error packet to the specified destination 
 *  @note This packet error verification is not applicable to the CRC verification operation within the same
 *        command set.
 *  @note This command should be either the last command or the second last to the nextRoute command since 
 *        all commands following this operation will be ignored if packet error is found. 
 */
#define  pa_CMD_VERIFY_PKT_ERROR            16 


/*  @def  pa_CMD_SPLIT
 *        Split the packet into header and payload portion to be delivered to different queues with 
 *        different CPPI flows 
 *  @note This command is only supported in the from-network direction
 *  @note This command should be placed ahead of any pa_CMD_PATCH command so that the header size can be adjusted accordingly
 *  @note The first 8-byte of psInfo area is reserved for this operation, therefore, the destOffset of pa_CMD_COPY_DATA_TO_PSINFO
 *        commands within the same command set should be 8 or larger.
 *        
 */

#define  pa_CMD_SPLIT                      17

/*  @def  pa_CMD_EF_OP
 *        Egress Flow operation command either triggers flow cache lookup to find the corresponding packet modification records 
 *        or provides those records directly. 
 *  @note This command can not be combined with any other commands
 */
#define  pa_CMD_EF_OP                      18 

/*  @def  pa_CMD_PATCH_TIME
 *        Patch the time values in packets (Gen 2 support only)
 */
#define  pa_CMD_PATCH_TIME                   19  

/*  @def  pa_CMD_PATCH_COUNT
 *        Patch the time values in packets (Gen 2 support only)
 */
#define  pa_CMD_PATCH_COUNT                  20

/*  @def  pa_CMD_EMAC_CRC_VERIFY
 *        Perfrom the Ethernet CRC verification for egress traffic. (Applicable only for Gen1) 
 *        Refer to the description of data structure @ref paCmdEmacCrcVerify_t for details.
 */

#define  pa_CMD_EMAC_CRC_VERIFY              21
   
/*  @}  */  
/** @} */

/**
 *  @defgroup routeCtrlInfo  PA Routing Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Routing Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paCmdNextRoute_t. 
 */ 
/*@{*/
/**
 *  @def  pa_NEXT_ROUTE_PARAM_PRESENT
 *        Control Info -- Set: Routing information such as flowId, queue are in command for egress packets
 *                        Clear: Routing information such as flowId, queue are in packet for ingress packets
 */
#define pa_NEXT_ROUTE_PARAM_PRESENT              0x0001 
/**
 *  @def  pa_NEXT_ROUTE_PROC_NEXT_CMD
 *        Control Info -- Set: Process the next command prior to forward the packet to its final destination
 *                        Clear: Forward the packet to the next destination without executing any more command
 *  @note: The data patch command (pa_CMD_PATCH_DATA) is the only one which can follow the next route command.
 *  @note: This option is only valid in the transmit (to-network) direction      
 */
#define pa_NEXT_ROUTE_PROC_NEXT_CMD              0x0002 
/**
 *  @def  pa_NEXT_ROUTE_PROC_MULTI_ROUTE
 *        Control Info -- Set: Multi-route is valid, the packet should be forwarded and then perform multi-route
 *                        Clear: Multi-route is invalid
 *  @note: This option is only valid in the receive (from-network) direction      
 */
#define pa_NEXT_ROUTE_PROC_MULTI_ROUTE           0x0004 
/**
 *  @def  pa_NEXT_ROUTE_TX_L2_PADDING
 *        Control Info -- Set: Perform MAC padding for packet with size smaller than 60
 *                        Clear: Do not perform MAC padding
 *  @note: This option is only valid in the transmit (to-network) direction      
 */
#define pa_NEXT_ROUTE_TX_L2_PADDING              0x0008 
/**
 *  @def  pa_NEXT_ROUTE_PROC_USR_STATS
 *        Control Info -- Set: User-defined statistics index is valid, update the chain of user-defined statistics specified
 *                             by statsIndex 
 *                        Clear: User-defined statistics index is invalid
 *  @note: This option is only valid in the egress (to-network) direction      
 */
#define pa_NEXT_ROUTE_PROC_USR_STATS             0x0010 

/**
 *  @def  pa_NEXT_ROUTE_RPT_TX_TIMESTAMP
 *        Control Info -- Set: Instruct switch to report the transmit timestamp with the associated CPTS domain, message type and 
 *                             sequence number encoded in the swInfo0. 
 *                        Clear: swInfo0 is invalid 
 *  @note: This option is only valid in the egress (to-network) direction on NSS_GEN2 devices when dest is set to pa_DEST_EMAC.   
 */
#define pa_NEXT_ROUTE_RPT_TX_TIMESTAMP           0x0020 

/*@}*/
/** @} */

/**
 *  @ingroup palld_api_macros
 *  @brief  pa_FORMAT_REPORT_TIMESTAMP_INFO is used to format the CPTS report timestamp information at swInfo0
 *
 *  @details  This macro is used to construct the swInfo0 with associated CPTS domain, message type and sequence id where
 *            swInfo0 is used to instruct the CPSW to report transmit timestamp as a CPTS event
 *
 */
#define pa_FORMAT_REPORT_TIMESTAMP_INFO(domain, msgType, seqId)             0x80000000UL              |  \
                                                                            (((domain) & 0xFF) << 20) |  \
                                                                            (((msgType) & 0x0F) << 16)|  \
                                                                            ((seqId & 0xFFFF))

/**
 *  @ingroup palld_api_structures
 *  @brief  Next Route Command
 *
 *  @details paCmdNextRoute_t defines the final route information
 *           The next route command can be used in both to-network and from-network directions. 
 *           In the to-network direction, it may be used multiple times to route traffic between PASS and SASS 
 *           before the packet is finally forwarded to the network. For example, the following steps show the 
 *           SRTP over IPSEC AH to-network traffic:
 *  @verbatim 
               1. Packet is delivered to SASS for SRTP operation
               2. Packet is delivered to PASS for UDP checksum operation
               3. Packet is delivered to SASS for IPSEC AH operation
               4. Packet is delivered to PASS for AH authentication tag insertion
               5. Packet is delivered to the network.
    @endverbatim
 *           The next route commands are required for step 3 and 5. The complete routing information should be provided 
 *           in the to-network direction.
 *
 *           In the from-network direction, the next route command is used only if the multiple routes are required or when
 *           dest is set to EMAC to forward the ingress packets out to another EMAC port. 
 *           In this case, only the parameter "ctrlBitfield", "multiRouteIndex" and/or "dest" are valid. After all the 
 *           commands in the command set are executed, the PASS will deliver packets to their desired destination based 
 *           on the parameters specified at the routing information upon the LUT1/LUT2 matching.  
 *           If the next route command is specified, it must be the last command within a command set. The commands following 
 *           the next route command will not be executed.  
 */

typedef struct {

  uint16_t    ctrlBitfield;    /**< Routing control information as defined at @ref routeCtrlInfo */
  int         dest;            /**< Packet destination as defined at @ref pktDest */
  uint8_t     pktType_emacCtrl;/**<  For destination SRIO, specify the 5-bit packet type toward SRIO 
                                     For destination HOST, EMAC, specify the EMAC control @ref emcOutputCtrlBits to the network */
  uint8_t     flowId;          /**< For host, SA or SRIO destinations, specifies return free descriptor setup */
  uint16_t    queue;           /**< For host, SA or SRIO destinations, specifies the dest queue */
  uint32_t    swInfo0;         /**< Placed in SwInfo0 for packets to host or SA; Placed in the PS Info for packets to SRIO*/
  uint32_t    swInfo1;         /**< Placed in SwInfo1 for packets to the SA; Placed in the PS Info for packets to SRIO */
  uint16_t    multiRouteIndex; /**< Multi-route index. It is valid in the from-network direction only */
  uint16_t    statsIndex;      /**< Index of the first user-defined statistics to be updated. 
                                    This optional parameter is valid in the to-network direction only */
} paCmdNextRoute_t;

/**
 *  @defgroup  crcFrameTypes CRC Frame types 
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name CRC Frame types
 *
 *  @brief  Define the frame types which are used to extract and derive the CRC operation parameters such as CRC starting 
 *          offset and CRC payload length from the frame header. 
 *
 *  @details  Both the payload length and the byte location where CRC calculation begins may vary in some protocl 
 *            frame such as WCDMA FP HS-DSCH Data Frame type 2 and type 3. The CRC Frame type is used for PASS to
 *            extract and/or derive the CRC starting offset and payload length.
 *
 *  @note     Only the following frame types are supported.
 */
/*  @{  */
/**
 *
 *   @def  pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2
 *         WCDMA FP HS-DSCH Data Frame Type 2
 */
#define pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2         0

/**
 *
 *  @def   pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE3
 *         WCDMA FP HS-DSCH Data Frame Type 3
 */
#define pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE3         1

#define pa_CRC_OP_FRAME_TYPE_MAX                          pa_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE3

 
/*  @}  */  
/** @} */


/**
 *  @defgroup crcOpCtrlInfo  PA CRC Command Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA CRC Command Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paCmdCrcOp_t. 
 */ 
/*@{*/
/**
 *  @def  pa_CRC_OP_CRC_VALIDATE
 *        Control Info -- Set: CRC Validate
 *                        Clear: CRC Computation
 */
#define pa_CRC_OP_CRC_VALIDATE              0x0001 
/**
 *  @def  pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER
 *        Control Info -- Set: CRC length field in the header
 *                        Clear: CRC length specified in command
 */
#define pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER  0x0002 
/**
 *  @def  pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE
 *        Control Info -- Set: Payload length field resides prior to the parsed header offset 
 *                             length field offset = offset from the current parsed header - lenOffset
 *                        Clear: Payload length field resides after the parsed header offset
 *                             length field offset = offset from the current parsed header + lenOffset
 */
#define pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE  0x0004 
/**
 *  @def  pa_CRC_OP_CRC_FRAME_TYPE
 *        Control Info -- Set: Frame Type is specified
 *                        Clear: Frame Type is not specified, use offset 
 *                               parameter
 */
#define pa_CRC_OP_CRC_FRAME_TYPE            0x0008 
/**
 *  @def  pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD
 *        Control Info -- Set: CRC field following payload
 *                        Clear: CRC offset specified in command
 */
#define pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD 0x0010 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  CRC Generation/Verification Command
 *
 *  @details paCmdCrcOp_t is used to create CRC operation command instruct the PASS to 
 *           perform CRC operation in both to-network and from-network directions. The
 *           module user is responsible for configuring the corresponding CRC engines
 *           which are used for the specified CRC operation. 
 *
 *           In the to-network direction, the payload offset, payload length and CRC offset 
 *           should be available in the command. The generated CRC will be inserted into
 *           the CRC location in the packet.
 *
 *           In the from-network direction, the payload length is either a constant or 
 *           available in the custom header. The CRC verification will be performed by
 *           the CRC engine connected to the PDSP where the CRC command is executed. 
 *           The CRC verification result will be indicated by the error flags within 
 *           the CPPI descriptor as described at section table @ref appendix2 
 */

typedef struct {

  uint16_t    ctrlBitfield;    /**< CRC operation control information as defined at @ref crcOpCtrlInfo */
  uint16_t    startOffset;     /**< Byte location, from SOP/Protocol Header, where the CRC computation begins 
                                    if frame type is not specified
                                    Byte location, from SOP/Protocol header, where the specific frame header begins
                                    if frame type is specified
                                    In to-network direction: offset from SOP
                                    In from-network direction: offset from the current parsed header 
                                    */
  uint16_t    len;             /**< Number of bytes covered by the CRC computation 
                                    valid only if pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER is clear */
  uint16_t    lenOffset;       /**< Payload length field offset in the custom header */
  uint16_t    lenMask;         /**< Payload length field mask */
  uint16_t    lenAdjust;       /**< Payload length adjustment: valid only if pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER is set */
  uint16_t    crcOffset;       /**< Offset from CRC computation starting location to the CRC field */
  uint16_t    crcSize;         /**< Size of CRC in bytes (PASS Gen2 only) */ 
  uint16_t    frameType;       /**< Frame type @ref crcFrameTypes, vaild if pa_CRC_OP_CRC_FRAME_TYPE is set */   
  uint32_t    initValue;       /**< CRC initial value (PASS Gen2 only) */   
} paCmdCrcOp_t;

/**
 *  @defgroup splitOpCtrlInfo  PA SPLIT Command Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA SPLIT Command Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paCmdSplitOp_t. 
 */ 
/*@{*/
/**
 *  @def  pa_SPLIT_OP_FRAME_TYPE
 *        Control Info -- Set: Frame Type is specified
 *                        Clear: Frame Type is not specified, use offset 
 *                               parameter
 */
#define pa_SPLIT_OP_FRAME_TYPE            0x0001 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Split Command
 *
 *  @details paCmdSplitOp_t is used to create Split command to instruct the PASS to 
 *           divide the ingress packet into the header and payload portion and deliver them
 *           to specified destination queues with specified CPPI flows respectively.
 *           Where the destination information of the header packet is specified by the 
 *           classification routing information and the destination information of the payload  
 *           packet is specified in this structure. 
 *
 */
typedef struct {
  uint16_t    ctrlBitfield; /**< Split operation control information as defined at @ref splitOpCtrlInfo */
  uint16_t    startOffset;  /**< Byte location, from Protocol Header, where the payload begins 
                                 if frame type is not specified
                                 Byte location, from Protocol header, where the specific frame header begins
                                 if frame type is specified
                                 In from-network direction: offset from the current parsed header 
                                 */
  uint16_t    frameType;    /**< Frame type @ref crcFrameTypes, vaild if pa_SPLIT_OP_FRAME_TYPE is set */         
  uint16_t    destQueue;    /**< Host queue for the payload packet */
  uint16_t    flowId;       /**< CPPI flow which instructs how link-buffer queues are used for sending payload packets. */   
                            
} paCmdSplitOp_t;   

/**
 *  @ingroup palld_api_structures
 *  @brief  Transmit checksum configuration
 *
 *  @details  paTxChksum_t is used in the call to @ref Pa_formatTxRoute or @ref Pa_formatTxCmd to create a tx 
 *            command header that instructs the packet accelerator sub-system to generate ones' complement
 *             checksums into network packets. The checksums are typically used for TCP and UDP payload checksums as
 *            well as IPv4 header checksums. In the case of TCP and UDP payload checksums the psuedo header
 *            checksum must be pre-calculated and provided, the sub-system does not calculate it.
 */
typedef struct {

  uint16_t startOffset;   /**<  Byte location, from SOP, where the checksum calculation begins */
  uint16_t lengthBytes;   /**<  Number of bytes covered by the checksum. Must be even */
  uint16_t resultOffset;  /**<  Byte offset, from startOffset, to place the resulting checksum */
  uint16_t initialSum;    /**<  Initial value of the checksum */
  uint16_t negative0;     /**<  If TRUE, a computed value of 0 is written as -0 */

} paTxChksum_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  patch time in EOAM packet configuration
 *
 *  @details  paPatchTime_t is used in the call @ref Pa_formatTxCmd to create a tx 
 *            command header that instructs the packet accelerator sub-system to patch the
 *            time bytes in the specified offset of the packet.
 *            Please refer to @ref appendix8 for details about EOAM mode.
 */
typedef struct {
  uint16_t startOffset;   /**<  Byte location, from SOP, to insert 8 bytes time values 4 byte second, 4 byte nano second */
} paPatchTime_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  patch Count in EOAM packet configuration
 *
 *  @details  paPatchCount_t is used in the call @ref Pa_formatTxCmd to create a tx 
 *            command header that instructs the packet accelerator sub-system to patch the
 *            specified user stats counter bytes in the specified offset of the packet.
 *            Please refer to @ref appendix8 for details about EOAM mode.
 */
typedef struct {

  uint16_t startOffset;   /**<  Byte location, from SOP, to insert count values */
  uint16_t countIndex;    /**<  Counter index to insert */  
} paPatchCount_t;

/**
 *  @defgroup copyCtrlInfo  PA Copy Command Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Copy Command Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paCmdCopy_t. 
 *         
 */ 
/*@{*/
/**
 *  @def  pa_COPY_OP_FROM_END
 *        Control Info -- Set: Copy data from the end of the payload
 *                        Clear: Copy data from the beginning of the payload
 */
#define pa_COPY_OP_FROM_END                 0x0001 
/*@}*/
/** @} */
 
/**
 *  @ingroup palld_api_structures
 *  @brief  Copy Command
 *
 *  @details paCmdCopy_t is used to define how to copy number of bytes from the data packet to 
 *           the descriptor. The copy command is used to instruct the PASS to copy up to 8 byte 
 *           from packet to the PS info section in the packet descriptor in the from-network direction. 
 *           If the desired copy area crosses over the packet boundary, then garbage data will be copied.
 *
 *  @note: There are 20-byte packet information stored in the PS Info section. It is recommended to copy 
 *         packet data after the packet information area. Otherwise, the packet information will be 
 *         overwritten. There are upto 12 bytes can be copied with the packet information or upto
 *         32 bytes can be copied without the packet information. 
 */

typedef struct {

  uint16_t    ctrlBitfield;    /**< Copy operation control information as defined at @ref copyCtrlInfo */
  uint16_t    srcOffset;       /**< Offset from the start of current protocol header for the data copy to begin */
  uint16_t    destOffset;      /**< Offset from the top of the PSInfo for the data to be copied to */
  uint16_t    numBytes;        /**< Number of bytes to be copied */   
} paCmdCopy_t;


/**
 *  @ingroup palld_api_structures
 *  @brief  Multi-route Command
 *
 *  @details paCmdMultiRoute_t is used to specify the desired PA multi-route set.
 *           The multi-route command instructs the PASS to route the packets to multiple 
 *           destinations in the from-network direction only. It must be the last command 
 *           within a command set. The commands following the multi-route command will
 *           not be executed.  
 */
typedef struct {

  uint16_t    index;        /**<  Multi-route set Index */
} paCmdMultiRoute_t;


/**
 *  @ingroup palld_api_constants
 *  @def  pa_MAX_CMD_SETS
 *        The maximum number of command sets supported
 */
#define pa_MAX_CMD_SETS     64

/**
 *  @ingroup palld_api_structures
 *  @brief  Command Set Command
 *
 *  @details paCmdSet_t is used to specify the desired PA command set. The command set command 
 *           instructs the PASS to execute a list of commands after a LUT1 or LUT2 match occurs. 
 *           It is one of the command which can be embedded within the @ref paRouteInfo_t. 
 */
typedef struct {

  uint16_t    index;        /**< Command Set Index */
} paCmdSet_t;

/**
 *   @ingroup palld_api_constants
 *   @def  pa_MAX_PATCH_BYTES
 *         The maximum number of bytes that a patch command can accept
 */
#define pa_MAX_PATCH_BYTES     16      /**< PATCH Command in to-netweok direction */
#define pa_MAX_RX_PATCH_BYTES  32      /**< PATCH Command within a command set */

/**
 *  @defgroup patchCtrlInfo  PA Patch Command Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Patch Command Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paPatchInfo_t. 
 *         
 */ 
/*@{*/
/**
 *  @def  pa_PATCH_OP_INSERT
 *        Control Info -- Set: Insert data into the packet
 *                        Clear: Patch data replaces existing packet data
 */
#define pa_PATCH_OP_INSERT                 0x0001 
/**
 *  @def  pa_PATCH_OP_MAC_HDR
 *        Control Info -- Set: Replace MAC header with patch data
 *                        Clear: Normal Patch/Insert operation
 */
#define pa_PATCH_OP_MAC_HDR                0x0002 
/**
 *  @def  pa_PATCH_OP_DELETE
 *        Control Info -- Set: Delete data in the packet
 *                        Clear: Normal Patch/Insert operation
 */
#define pa_PATCH_OP_DELETE                 0x0004 
/*@}*/
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  Packet patching configuration
 *
 *  @details paPatchInfo_t is used to create data patch command. The patch command is used to patch 
 *           existing data or insert data in the packet in both to-network and from-network directions.
 *
 *           In the to-network direction, it can be used to patch the authentication tag provided by SASS 
 *           into the AH header within the packet. In this case, the patch data is not present at the command 
 *           when it is formatted and it is appended by the SASS. The @ref Pa_formatRoutePatch is used to create
 *           a command block along with a packet routing command to forward the packet after the patch is complete
 *
 *           In the from-network direction, it can be used to insert up to 32 bytes to the offset location
 *           as part of the command set to be executed after a LUT1 or LUT2 match. 
 *           This command can be used to patch the entire MAC header for MAC router functionality. It may be further 
 *           enhanced and combined with other commands to support IP forwarding operation in the future.
 *           A short version of the patch command can be used to insert up to 2 bytes into the current parsing
 *           location of the packet after a LUT2 match.
 */

typedef struct {

  uint16_t   ctrlBitfield;      /**<  Patch operation control information as defined at @ref patchCtrlInfo */
  uint16_t   nPatchBytes;       /**<  The number of bytes to be patched */
  uint16_t   totalPatchSize;    /**<  The number of patch bytes in the patch command, must be >= to nPatchBytes and a multiple of 4 bytes */
  uint16_t   offset;            /**<  Offset from the start of the packet for the patch to begin in the to-network direction 
                                      Offset from the start of the current header for the patch to begin in the from-network direction */
  uint8_t    *patchData;        /**<  Pointer to the patch data */

} paPatchInfo_t;



/**
 *  @ingroup palld_api_structures
 *  @brief  paPayloadInfo_t defines the packet payload information in the short format.
 *          It is required by the Security Accelerator sub-system (SASS)
 *
 *  @details paPayloadInfo_t defines the packet parsing information in terms of
 *           payload offset and payload length as described below
 *  @li      SRTP:      offset to the RTP header; RTP payload length including ICV
 *  @li      IPSEC AH:  offset to the Outer IP; IP payload length
 *  @li      IPSEC ESP: offset to the ESP header; ESP papload length including ICV
 */

typedef struct  {
    uint16_t  offset;    /**< The offset to where the SA packet parsing starts */
    uint16_t  len;       /**< The total length of the protocal payload to be processed by SA */
    uint32_t  supData;   /**< Optional supplement data such as the 32-bit CountC for some 3GPP operation modes */
} paPayloadInfo_t;


/**
 *  @ingroup palld_api_structures
 *  @brief   Tx timestamp reporting information
 *
 *  @details paCmdTxTimestamp_t specifies the tx timestamp reporting information. The report tx timestamp command is used to instruct 
 *           the PASS to report the PA timestamp when the packet is transmitting out of PASS in a return (null) packet to the specified 
 *           host queue. The transmit timestamp may be used for the Precision Timing Protocol (PTP). The reported tx timestamp will be 
 *           a 64-bit value, with the lower 32 bits stored in timestamp field, and the upper 32 bits stored in swInfo1.
 *
 *  @pre     API @ref Pa_configTimestamp() should be called to enable PASS system timestamp.             
 */

typedef struct  {
    uint16_t  destQueue; /**< Host queue for the tx timestamp reporting packet */
    uint16_t  flowId;    /**< CPPI flow which instructs how link-buffer queues are used for sending tx timestamp reporting packets. */
    uint32_t  swInfo0;   /**< lower 32 bit value returned in the descriptor as swInfo0 which can be used as event identifier */
} paCmdTxTimestamp_t;

/**
 *  @ingroup palld_api_structures
 *  @brief   IP fragmentation information
 *
 *  @details paCmdIpFrag_t is used to create the IPv4 fragment command. The IP fragment command is used to instruct the PASS to 
 *           perform IPv4 fragmentation operation. This operation can be applied to both inner IP prior to IPSEC encapsulation and 
 *           outer IP after IPSEC encapsulation.  This command should go with a next route command which provides the destination 
 *           information prior to the fragmentation operation. 
 *    
 *           For the inner IP fragmentation, follow the following procedure:
 *  @li	     Host sends packets with the IP fragment command and the destination queue set to a host queue to PASS PDSP5 
 *           for IP fragmentation operation.
 *  @li      All fragments will be delivered to the specified host queue.
 *  @li      Host adds the outer MAC/IP header, invokes the SA LLD sendData function and then sends the fragments to the SA queue.
 *  @li      Each fragment will be encrypted, authenticated and forwarded to the final destination.
 *
 *           For the outer IP fragmentation, the overall operation is stated below: 
 *  @li      Packet is delivered to SASS for IPSEC operation
 *  @li	     Packet is delivered to PASS for IP Fragmentation operation
 *  @li      The entire packet or its fragments are delivered to the network.
 *
 *  @note the next route command is required for step 2
 *  @note The IP fragment command can not be combined with some other tx commands such as checksum and CRC commands since
 *        those commands may require the PASS operation across multiple fragments. The workaround is to break the tx commands into
 *        two groups. The first group consists of the checksum, CRC, other commands and a next route command which routes the packet
 *        back to the same PDSP to execute the second command group which consists of the IP fragment command and the next route 
 *        command which points to the final destination.
 *
 *        The IP fragment command can be combined with a single blind patch command to support the IPSEC AH use case in which the SASS 
 *        passes the IPSEC AH packet with the blind patch command to the PASS so that the autentication tag can be inserted into the AH 
 *        header. The recommended order of the tx commands is as the followings:
 *        - pa_CMD_IP_FRAGMENT
 *        - pa_CMD_NEXT_ROUTE with flag pa_NEXT_ROUTE_PROC_NEXT_CMD set
 *        - pa_CMD_PATCH_DATA 
 *
 *        The IP fragment command can be also combined with up to two message length patching commands to support the message length 
 *        field updating for each IP fragment. This operation is required for certain L2 header which contains a length field such as
 *        802.3 and PPPoE. The order of tx command is as the followings:
 *        - pa_CMD_PATCH_MSG_LEN (optional)
 *        - pa_CMD_PATCH_MSG_LEN (optional)
 *        - pa_CMD_IP_FRAGMENT
 *        - pa_CMD_NEXT_ROUTE 
 */

typedef struct  {
    uint16_t  ipOffset; /**< Offset to the IP header. */
    uint16_t  mtuSize;  /**< Size of the maximum transmission unit (>= 68) */
} paCmdIpFrag_t;

/**
 *  @ingroup palld_api_structures
 *  @brief   Message length patching configuration
 *
 *  @details paPatchMsgLenInfo_t is used to create message length patch command which is used in conjunction with
 *           the Ip fragmentation command. This command instruct the PASS to update the message length field within 
 *           some L2 protocol header such as 802.3 and PPPoE after the potential IP fragmentation operation.
 *
 *           The PASS support up to two message length patching operations per IP fragmentation command.
 */

typedef struct {

  uint8_t    msgLenSize;    /**<  Size of message length field in bytes (@note only 2-byte and 4=byte message length is supported) */
  uint8_t    offset;        /**<  Offset from the start of the packet to the message length field */ 
  uint16_t   msgLen;        /**<  Message length excluding the IP header and payload length */

} paPatchMsgLenInfo_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  User-defined Statistics Command
 *
 *  @details paCmdUsrStats_t is used to specify the desired User-defined counter. The user stats command 
 *           instructs the PASS to update the specified user-defined counter and all the counters which are 
 *           linked to this counter 
 *           It is one of the command which can be embedded within the @ref paRouteInfo_t. 
 */
typedef struct {
  uint16_t    index;        /**< User-defined statistics index */
} paCmdUsrStats_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  Command Set plus User-defined Statistics Command
 *
 *  @details paCmdSetUsrStats_t is used to specify the desired PA command set and User-defined counter. This command
 *           provides the module user a mechanism to specify different user-defined counters with the same command set 
 *           for different LUT entries and vice versa. 
 *           This command instructs the PASS to update the specified user-defined counter and all the counters which are 
 *           linked to this counter and then execute the specified command set.
 *           It is one of the command which can be embedded within the @ref paRouteInfo_t. 
 */
typedef struct {
  uint16_t    setIndex;          /**< Commad Set Index */
  uint16_t    statsIndex;        /**< User-defined statistics index */
} paCmdSetUsrStats_t;


/**
 *  @defgroup pktErrInfo  PA Packet Error Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Packet Error Info Bit Definitions
 *
 *  Bitmap definition of the errorBitfield in @ref paCmdVerifyPktErr_t. 
 *         
 */ 
/*@{*/
/**
 *  @def  pa_PKT_ERR_IP_CHECKSUM
 *        Control Info -- Set: Re-direct packet if IP checksum error occurs
 *                        Clear: Ignore IP checksum Error
 */
#define pa_PKT_ERR_IP_CHECKSUM                 0x0001 
/**
 *  @def  pa_PKT_ERR_L4_CHECKSUM
 *        Control Info -- Set: Re-direct packet if UDP/TCP checksum error occurs
 *                        Clear: Ignore UDP/TCP checksum Error
 */
#define pa_PKT_ERR_L4_CHECKSUM                 0x0002 
/**
 *  @def  pa_PKT_ERR_CRC
 *        Control Info -- Set: Re-direct packet if CRC error occurs
 *                        Clear: Ignore CRC Error
 */
#define pa_PKT_ERR_CRC                         0x0004 
/*@}*/
/** @} */
 
/**
 *  @ingroup palld_api_structures
 *  @brief  Verify Packet Error Command
 *
 *  @details paCmdVerifyPktErr_t is used to construct the "Verify Packet Error" command. The  
 *           IPv4 header checksum, UDP/TCP checksum and SCTP CRC-32c checksum verification are performed by 
 *           the PASS autonomously while the CRC verification is performed per command. The corresponding error bit 
 *           in the CPPI descriptor will be set and can be verified by the application when packet is delivered 
 *           to the host. This command instructs PASS to examine the specified error flags and forward the error 
 *           packet accordingly.
 */

typedef struct {

  uint16_t errorBitfield;   /**<  Packet Error information as defined at @ref pktErrInfo */
  uint8_t  dest;            /**<  Packet destination as defined at @ref pktDest */
  uint8_t  flowId;          /**<  For host destination, specifies CPPI flow which defines free queues are used for receiving packets */
  uint16_t queue;           /**<  For host destination, specifies the destination queue */
  uint32_t swInfo0;         /**<  Placed in SwInfo0 for packets to host */
} paCmdVerifyPktErr_t;

/**
 *  @ingroup palld_api_structures
 *  @brief   EMAC CRC Verification information
 *
 *  @details paCmdEmacCrcVerify_t is used to create the EMAC CRC verification command which is used to instruct the PASS to 
 *           perform Ethernet CRC verification for forwarding to-network traffic. The egress packet with this command is 
 *           expected to be a forwarding Ethernet packet with CRC. PASS will perform CRC verification against the CRC
 *           value at the packet, if CRC is good, the packet will be forwarded to the desired EMAC port and if CRC 
 *           is bad, then the packet will be dropped.
 *           Application should invoke Pa_configCrcEngine() API to format the CRC configuration packet for Ethernet CRC and 
 *           then forward configuration packet to Tx command prtocessing PDSP (PDSP5) before this command is used. 
 *            
 *           This command is used to provide a workaround for following GbE errata at some keystone devices:
 *
 *           The GbE switch may drop packets in TX path when:
 *           - full gigabit speeds are sustained and
 *           - the packet size is not a 32 bit multiple (1499, 1498, 1497, 1495, etc.) and
 *           - Ethernet CRC is included in the last 4 bytes of the packet sent to the switch
 *    
 *  @note The Ethernet CRC Verify command can not be combined with any other tx commands, all other commands will be ignored
 *        by PASS when this command is processed.
 *
 */

typedef struct  {
    uint16_t  emacPort; /**< Specify the output EMAC port number as define at @ref paEmacPort. */
} paCmdEmacCrcVerify_t;

/**
 *  @defgroup efOpCtrlInfo  PA Egress Flow Command Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Command Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paCmdEfOp_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EF_OP_CMD_FC_LOOKUP
 *        Control Info -- Set: Perform flow cache lookup to look for the associated packet modification records per match
 *                        Clear: Skip flow cache lookup and use the packet modification records specified in this command.
 */
#define pa_EF_OP_CMD_FC_LOOKUP             0x0001 
/**
 *  @def  pa_EF_OP_CMD_VALID_LVL1
 *        Control Info -- Egress Flow level 1 index is present
 */
#define pa_EF_OP_CMD_VALID_LVL1            0x0010 
/**
 *  @def  pa_EF_OP_CMD_VALID_LVL2
 *        Control Info -- Egress Flow level 2 index is present
 */
#define pa_EF_OP_CMD_VALID_LVL2            0x0020 
/**
 *  @def  pa_EF_OP_CMD_VALID_LVL3
 *        Control Info -- Egress Flow level 3 index is present
 */
#define pa_EF_OP_CMD_VALID_LVL3            0x0040 
/**
 *  @def  pa_EF_OP_CMD_VALID_LVL4
 *        Control Info -- Egress Flow level 4 index is present
 */
#define pa_EF_OP_CMD_VALID_LVL4            0x0080 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Egress Flow Operation Command 
 *
 *  @details paCmdEfOp_t is used to create Egress Flow operation command which instructs 
 *           the PASS to perform optional flow cache lookup to find the associated 
 *           packet modification records or provides those records in the command. Then
 *           PASS will execute the specified packet modification records in order to 
 *           perform one or multiple of the following actions:
 *           - Update inner L3/L4 headers
 *           - Insert or update outer L3 header
 *           - Insert IPSEC header and trailer
 *           - Perform inner and/or outer IP fragmentation
 *           - Insert or update L2 header
 */

typedef struct {

  uint16_t    ctrlBitfield; /**< Egress Flow operation control information as defined at @ref efOpCtrlInfo */
  uint16_t    l2Offset;     /**< Offset to the layer 2 header from SOP */
  uint16_t    l3Offset;     /**< Offset to the outer IP from SOP */
  uint16_t    l3Offset2;    /**< Offset to the inner IP from SOP, which should be set to L3Offset if there is 
                                 only one IP layer */
  uint16_t    ipsecOffset;  /**< Offset to the IPSEC ESP/AH header if the IPSEC header resides in the egress
                                 packets */                                     
  uint16_t    endOffset;     /**< Offset to the end of L4 (UDP/UDPLite/TCP) payload */
  uint16_t    lvl1Index;    /**< Specify egress flow level 1 record index */
  uint16_t    lvl2Index;    /**< Specify egress flow level 2 record index */
  uint16_t    lvl3Index;    /**< Specify egress flow level 3 record index */
  uint16_t    lvl4Index;    /**< Specify egress flow level 4 record index */
} paCmdEfOp_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Command Information structure
 *
 * @details Data structure defines PA commands. The PA command can be invoked by the @ref paRouteInfo_t as a simple command. 
 *          They are the building blocks for function @ref Pa_configCmdSet to create a list of commands refered as a command
 *          set in the from-network direction. They can be used by the function @ref Pa_formatTxCmd to create or update the
 *          list of tx commands.
 *
 */
typedef struct {
  uint16_t    cmd;         /**< Specify the PA command code as defined at @ref paCmdCode */
  union {
    paCmdNextRoute_t   route;    /**< Specify nextRoute command specific parameters */
    paTxChksum_t       chksum;   /**< Specify Tx Checksum command specific parameters */
    paCmdCrcOp_t       crcOp;    /**< Specify CRC operation command specific parameters */
    paCmdCopy_t        copy;     /**< Specify Copy command specific parameters */
    paPatchInfo_t      patch;    /**< Specify Patch command specific parameters */
    paPayloadInfo_t    payload;  /**< Specify the payload information required by SA */
    paCmdSet_t         cmdSet;   /**< Specify Command Set command specific parameters */
    paCmdMultiRoute_t  mRoute;   /**< Specify Multi-route command specific parameters */
    paCmdTxTimestamp_t txTs;     /**< Specify Report Tx Timestamp command specific parameters */
    paCmdIpFrag_t      ipFrag;   /**< Specify IP fragmentation command specific parameters */
    paCmdUsrStats_t    usrStats; /**< Specify User-defined Statistics command specific parameters */
    paCmdSetUsrStats_t cmdSetUsrStats; /**< Specify Command Set and User-defined Statistics command specific parameters */
    paPatchMsgLenInfo_t patchMsgLen;   /**< Specify Patch Message Length command specific parameters */
    paCmdVerifyPktErr_t verifyPktErr;  /**< Specify Packet error Verification command specific parameters */
    paCmdSplitOp_t      split;   /**< Specify Split command sepcific parameters */
    paCmdEmacCrcVerify_t emacCrc;/**< Specify the EMAC CRC Verify command specific parameters */
    paCmdEfOp_t         efOp;    /**< Specify Egress Flow operation command specific parameters (PASS Gen2 only) */
    paPatchTime_t       patchTime; /**< Specify insert time in the messages like Ethernet OAM packets (PASS Gen2 only)*/
    paPatchCount_t      patchCount; /**< Specify insert count in the messages like Ethernet OAM packets (PASS Gen2 only)*/    
  }params;                      /**< Contain the command specific parameters */

} paCmdInfo_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  IP lookup information
 *
 *  @details  paIpInfo_t is used to specifiy the IPv4 or IPv6 parameters used in packet routing.
 *            With the exception of parameter tos, a value of 0 in any parameter means that that
 *            field is not used in packet routing. Since a value of 0 is valid for tos, the paramter
 *            tosCare is used to indicate if the tos field (IPv4) or traffic class (Ipv6) is used 
 *            for packet routing.
 */
typedef struct  {

  paIpAddr_t  src;       /**<  Source IP address */
  paIpAddr_t  dst;       /**<  Destination IP address */
  uint32_t    spi;       /**<  ESP or AH header Security Parameters Index */
  uint32_t    flow;      /**<  IPv6 flow label in 20 lsbs */
  int         ipType;    /**<  @ref IpValues */
  uint16_t    greProto;  /**<  GRE protocol field */
  uint8_t     proto;     /**<  IP Protocol (IPv4) / Next Header (IPv6) */
  uint8_t     tos;       /**<  IP Type of Service (IPv4) / Traffic class (IPv6) */
  uint16_t    tosCare;   /**<  TRUE if the tos value is used for matching */
  uint16_t    sctpPort;  /**<  SCTP Destination Port */
} paIpInfo_t;

/**
 *  @defgroup paIpInfoValidBits  PA IP Info Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA IP Info Valid Bit Definitions
 *
 *  Bitmap definition of the validBitmap in @ref paIpInfo2_t. 
 */ 
/*@{*/
/**
 *  @def  pa_IP_INFO_VALID_SRC
 *        - Source IP address is present
 */
#define pa_IP_INFO_VALID_SRC                (1<<0)

/**
 *  @def  pa_IP_INFO_VALID_DST
 *        - Destination IP address is present
 */
#define pa_IP_INFO_VALID_DST                (1<<1)

/**
 *  @def  pa_IP_INFO_VALID_SPI
 *        - 32-bit Security Parameters Index of IPSEC ESP/AH is present
 */
#define pa_IP_INFO_VALID_SPI                (1<<2)

/**
 *  @def  pa_IP_INFO_VALID_FLOW
 *        - IPv6 flow label is present
 */
#define pa_IP_INFO_VALID_FLOW               (1<<3)

/**
 *  @def  pa_IP_INFO_VALID_GREPROTO
 *        - GRE protocol field is present
 */
#define pa_IP_INFO_VALID_GREPROTO           (1<<4)

/**
 *  @def  pa_IP_INFO_VALID_PROTO
 *        - IPv4 protocol or IPv6 next header is present
 */
#define pa_IP_INFO_VALID_PROTO              (1<<5)

/**
 *  @def  pa_IP_INFO_VALID_TOS
 *        - IPv4 type of service or IPv6 traffic class is present
 */
#define pa_IP_INFO_VALID_TOS                (1<<6)

/**
 *  @def  pa_IP_INFO_VALID_SCTPPORT
 *        - SCTP destination port is present
 */

#define pa_IP_INFO_VALID_SCTPPORT           (1<<7)

/* @} */ /* ingroup */
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Enhanced IP lookup information
 *
 *  @details  paIpInfo2_t is the upgraded version of paIpInfo_t to support additional IP lookup 
 *            parameters over time while still maintaining backward compatibility. Future feature 
 *            enhancements will be supported through this API data structure only.
 *
 *            Since not all fields are used all the time, validBitmap is used to specify which field 
 *            is used for packet classification. 
 */
typedef struct  {
  uint32_t    validBitMap;/**<  32-bit valid Bitmap corresponding to each optional field as defined at @ref paIpInfoValidBits */
  paIpAddr_t  src;        /**<  Source IP address */
  paIpAddr_t  dst;        /**<  Destination IP address */
  uint32_t    spi;        /**<  ESP or AH header Security Parameters Index */
  uint32_t    flow;       /**<  IPv6 flow label in 20 lsbs */
  int         ipType;     /**<  Mandatory if src or dst is valid @ref IpValues */
  uint16_t    greProto;   /**<  GRE protocol field */
  uint8_t     proto;      /**<  IP Protocol (IPv4) / Next Header (IPv6) */
  uint8_t     tos;        /**<  IP Type of Service (IPv4) / Traffic class (IPv6) */
  uint16_t    sctpPort;   /**<  SCTP Destination Port */
} paIpInfo2_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  MAC/Ethernet lookup information
 *
 *  @details  paEthInfo_t is used to specify the MAC/Ethernet parameters used in packet classification.
 *            A value in 0 for any of the fields indicates that the field is not used for
 *            packet classification.
 */
typedef struct  {
  paMacAddr_t        src;           /**< Source MAC addresss  */
  paMacAddr_t        dst;           /**< Destination MAC address */
  uint16_t           vlan;          /**< VLAN tag VID field, 12 lsbs  */
  uint16_t           ethertype;     /**< Ethertype field. */
  uint32_t           mplsTag;       /**< MPLS tag. Only the outer tag is examined */
  uint16_t           inport;        /**< Input EMAC port number as specified by @ref paEmacPort */
} paEthInfo_t;

/**
 *  @defgroup paEthInfoValidBits  PA ETH Info Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA ETH Info Valid Bit Definitions
 *
 *  Bitmap definition of the validBitmap in @ref paEthInfo2_t. 
 */ 
/*@{*/

/**
 *  @def  pa_ETH_INFO_VALID_SRC
 *        - Source MAC is present
 */
#define pa_ETH_INFO_VALID_SRC                   (1<<0)

/**
 *  @def  pa_ETH_INFO_VALID_DST
 *        - Destination MAC is present
 */
#define pa_ETH_INFO_VALID_DST                   (1<<1)

/**
 *  @def  pa_ETH_INFO_VALID_VLAN
 *        - VLAN ID is present
 */
#define pa_ETH_INFO_VALID_VLAN                  (1<<2)

/**
 *  @def  pa_ETH_INFO_VALID_ETHERTYPE
 *        - Ether type is present
 */
#define pa_ETH_INFO_VALID_ETHERTYPE             (1<<3)

/**
 *  @def  pa_ETH_INFO_VALID_MPLSTAG
 *        - MPLS tag is present
 */
#define pa_ETH_INFO_VALID_MPLSTAG               (1<<4)

/**
 *  @def  pa_ETH_INFO_VALID_INPORT
 *        - Input EMAC port is present
 */
#define pa_ETH_INFO_VALID_INPORT                (1<<5)

/**
 *  @def  pa_ETH_INFO_VALID_VLAN_PRI
 *        - Input VLAN PRI is present
 */
#define pa_ETH_INFO_VALID_VLAN_PRI              (1<<6)


/* @} */ /* ingroup */
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Enhanced MAC/Ethernet lookup information
 *
 *  @details  paEthInfo2_t is the upgraded version of paEthInfo_t to support additional MAC lookup 
 *            parameters over time while still maintaining backward compatibility. Future feature 
 *            enhancements will be supported through this API data structure only.
 *
 *            Since not all fields are used all the time, validBitmap is used to specify which field 
 *            is used for packet classification. 
 *
 */
typedef struct  {
  uint32_t           validBitMap;   /**< 32-bit valid Bitmap corresponding to each optional field as defined at @ref paEthInfoValidBits */
  paMacAddr_t        src;           /**< Source MAC addresss  */
  paMacAddr_t        dst;           /**< Destination MAC address */
  uint16_t           vlan;          /**< VLAN tag VID field, 12 lsbs  
                                         @note: Both untagged packets and priority marked packets (i.e. packets with VID = 0)
                                                will be matched when vlan is set to 0 */
  uint16_t           ethertype;     /**< Ethertype field. */
  uint32_t           mplsTag;       /**< MPLS tag. Only the outer tag is examined */
  uint16_t           inport;        /**< Input EMAC port number as specified by @ref paEmacPort */
  uint8_t            vlanPri;       /**< VLAN tag PCP field, 3 bits showing priority */
} paEthInfo2_t;


/**
 *  @ingroup palld_api_structures
 *  @brief  EOAM look up information
 *
 *  @details  paEoamFlowInfo_t is used to specify the flow parameters used in EOAM packet classification.
 *            Please refer to @ref appendix8 for details about EOAM mode.
 *
 */
typedef struct {
  uint16_t    validBitMap;  /**< Valid bit map reserved for future enhancements */
  uint8_t     flowId;       /**< Specifies the packet DMA flow ID, which defines the free queuese are 
                                 used for receiving EOAM control packets */
  uint16_t    destQueue;    /**< Specifies the destination queue used for receiving EOAM control packets */
  uint32_t    swInfo0;      /**< Placed in SwInfo0 for EOAM control packets to host */                                   
  uint16_t    statsIndex;   /**< user defined counter index binded with an EOAM target flow */
  uint8_t     megLevel;     /**< Maintenance Entity Group Level threshold to decide need statistics or not. 
                                 MEG Level is a 3-bit field. It contains an integer value that identifies MEG
                                 level of OAM PDU. Value ranges from 0 to 7 */  
} paEoamFlowInfo_t;

/**
 *  @defgroup paAclInfoValidBit  PA ACL Matching Info Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA ACL Matching Info Valid Bit Definitions
 *  Bitmap definition of the validBitfield in paAclInfo_t. 
 *  It allows selective ACL matching parameters
 */ 
/*@{*/
/**
 *  @def  pa_ACL_INFO_VALID_SRC_IP
 *        srcIp is present
 */
#define pa_ACL_INFO_VALID_SRC_IP            0x0001 
/**
 *  @def  pa_ACL_INFO_VALID_SRC_IP_MASK
 *        srcIpMask is present. This flag is valid only if srcIp is present.
 *        If srcIp is present and this flag is clear, it means all IP address bits are valid.
 */
#define pa_ACL_INFO_VALID_SRC_IP_MASK       0x0002 
/**
 *  @def  pa_ACL_INFO_VALID_DST_IP
 *        dstIp is present
 */
#define pa_ACL_INFO_VALID_DST_IP            0x0004 
/**
 *  @def  pa_ACL_INFO_VALID_DST_IP_MASK
 *        dstIpMask is present. This flag is valid only if dstIp is present.
 *        If dstIp is present and this flag is clear, it means all IP address bits are valid.
 */
#define pa_ACL_INFO_VALID_DST_IP_MASK       0x0008 
/**
 *  @def  pa_ACL_INFO_VALID_CTRL_FLAG
 *        ctrlFlag and ctrlFlagMask are present
 */
#define pa_ACL_INFO_VALID_CTRL_FLAG         0x0010
/**
 *  @def  pa_ACL_INFO_VALID_PROTO
 *        proto is present
 */
#define pa_ACL_INFO_VALID_PROTO             0x0020 
/**
 *  @def  pa_ACL_INFO_VALID_DSCP
 *        dscp is present
 */
#define pa_ACL_INFO_VALID_DSCP              0x0040 
/**
 *  @def  pa_ACL_INFO_VALID_SRC_PORT
 *        srcPortBegin and srcPortEnd  are present */
#define pa_ACL_INFO_VALID_SRC_PORT          0x0100 
/**
 *  @def  pa_ACL_INFO_VALID_DST_PORT
 *        dstPortBegin and dstPortEnd are present */
#define pa_ACL_INFO_VALID_DST_PORT          0x0200 

/*@}*/
/** @} */

/**
 *  @defgroup paAclInfoCtrlFlags  PA ACL Info Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA ACL Info Control Flag Definitions
 *  Bitmap definition of the ctrlFlags and ctrlFlagsMask in paAclnfo_t. 
 */ 
/*@{*/
/**
 *  @def  pa_ACL_INFO_CONTROL_FLAG_FRAG
 *        Flag -- 1: IP fragments 
 */
#define pa_ACL_INFO_CONTROL_FLAG_FRAG           0x0001 
/**
 *  @def  pa_ACL_INFO_CONTROL_FLAG_CONTAIN_L4
 *        Flag -- 1: Packet or fragment which conatins L4 header 
 */
#define pa_ACL_INFO_CONTROL_FLAG_CONTAIN_L4     0x0002 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  ACL lookup information
 *
 *  @details  paAclInfo_t is used to specifiy the ACL matching parameters.
 */
typedef struct  {
  uint16_t    validBitMap;  /**< Specify valid parameters as defined at @ref paAclInfoValidBit */
  uint16_t    ctrlFlag;     /**< Specify ACL contrl flags as defined at @ref paAclInfoCtrlFlags */
  uint16_t    ctrlFlagMask; /**< ACL control flag valid masks */
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
} paAclInfo_t;

/** 
 * @ingroup palld_api_structures
 * @brief SRIO Type11 header information
 *
 * @details  The structure describes the SRIO type 11 specific Lo-L2 header information.
 */
typedef struct paSrioType11Info_s
{
    uint16_t      mbox;     /**< Mail Box */
    uint16_t      letter;   /**< Letter Identifier */
} paSrioType11Info_t;

/**
 * @ingroup palld_api_structures
 * @brief SRIO Type9 header information
 *
 * @details  The structure describes the SRIO type 9 specific L0-L2 header information.
 */
typedef struct paSrioType9Info_s
{
    uint16_t        streamId;  /**< Stream identifier */
    uint16_t        cos;       /**< Class of service  */
} paSrioType9Info_t;


/**
 * @ingroup palld_api_structures
 * @brief  Srio message type specific header information
 *
 * @details  This union is used to specify the SRIO type specific header information to the module. 
 *            The type in the union is determined through other parameters passed to the module 
 *            (see @ref srioMessageTypes).
 */
typedef union  {

  paSrioType9Info_t  type9;   /**< SRIO type 9 specific information */
  paSrioType11Info_t type11;  /**< SRIO type 11 specific information */
  
} paSrioTypeInfo_t;

/**
 *  @defgroup  srioMessageTypes SRIO Message types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   SRIO Type Values
 *  @brief  Defines the SRIO message types.
 *
 *  @details The packet accelerator sub-system parses both SRIO Type 9 and Type 11 message headers (see @ref netlayers). 
 *           This group is used to distinguish which type of header will be used.
 */
/* @{ */
/**
 *  @def  pa_SRIO_TYPE_9
 *        SRIO Message Type 9
 */
#define  pa_SRIO_TYPE_9    9

/**
 *  @def  pa_SRIO_TYPE_11
 *        SRIO Message Type 11
 */
#define  pa_SRIO_TYPE_11   11
  
/*  @}  */  
/** @} */

/**
 *  @defgroup  srioTransportTypes SRIO Transport types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   SRIO Transport Type Values
 *  @brief  Defines the SRIO tansport types used.
 *
 */
/* @{ */
/**
 *  @def  pa_SRIO_TRANSPORT_TYPE_0
 *        SRIO Transport type 0: 8 bit device identifiers 
 */
#define  pa_SRIO_TRANSPORT_TYPE_0    0

/**
 *  @def  pa_SRIO_TRANSPORT_TYPE_1
 *        SRIO Transport type 1: 16 bit device identifiers
 */
#define  pa_SRIO_TRANSPORT_TYPE_1    1
  
/*  @}  */  
/** @} */

/**
 *  @defgroup paSrioInfoValidBits  PA SRIO Info Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA SRIO Info Valid Bit Definitions
 *
 *  Bitmap definition of the validBitMap in @ref paSrioInfo_t. 
 */ 
/*@{*/
/**
 *  @def  pa_SRIO_INFO_VALID_SRC_ID
 *        - srcId is present
 */
#define pa_SRIO_INFO_VALID_SRC_ID               0x0001 
/**
 *  @def  pa_SRIO_INFO_VALID_DEST_ID
 *        - destId is present
 */
#define pa_SRIO_INFO_VALID_DEST_ID              0x0002 
/**
 *  @def  pa_SRIO_INFO_VALID_ID
 *        - Id is present
 */
#define pa_SRIO_INFO_VALID_ID                  (pa_SRIO_INFO_VALID_SRC_ID | pa_SRIO_INFO_VALID_DEST_ID)  

/**
 *  @def  pa_SRIO_INFO_VALID_CC
 *        - cc is present
 */
#define pa_SRIO_INFO_VALID_CC                   0x0004 
/**
 *  @def  pa_SRIO_INFO_VALID_PRI
 *        - pri is present
 */
#define pa_SRIO_INFO_VALID_PRI                  0x0008 
/**
 *  @def  pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID
 *        - typeInfo.type9.streamId is present
 */
#define pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID   0x0010 
/**
 *  @def  pa_SRIO_INFO_VALID_TYPE_INFO_COS
 *        - typeInfo.type9.cos is present
 */
#define pa_SRIO_INFO_VALID_TYPE_INFO_COS        0x0020
/**
 *  @def  pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX
 *        - typeInfo.type11.mbox is present
 */
#define pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX    0x0010 
/**
 *  @def  pa_SRIO_INFO_VALID_TYPE_INFO_LETTER
 *        - typeInfo.type11.letter is present
 */
#define pa_SRIO_INFO_VALID_TYPE_INFO_LETTER     0x0020
/**
 *  @def  pa_SRIO_INFO_VALID_TYPE_INFO
 *        - typeInfo is present
 */
#define pa_SRIO_INFO_VALID_TYPE_INFO            (pa_SRIO_INFO_VALID_TYPE_INFO_COS       |   \
                                                 pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID  |   \
                                                 pa_SRIO_INFO_VALID_TYPE_INFO_LETTER    |   \
                                                 pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX )     
/* @} */ /* ingroup */
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  SRIO lookup information
 *
 *  @details  srioIpInfo_t is used to specifiy the SRIO type 9 and type 11 L0-L2 parameters used in packet routing.
 *            set the corresponding valid bit at validBitmap for the parameters required for SRIO message
 *            classification.  
 *            Where tt should be provided if srcId or destId is required
 *                  msgType should be provided if typeInfo is required 
 */
typedef struct  {

  uint16_t    validBitMap;   /**< Specify which parameters are valid as defined at @ref paSrioInfoValidBits */
  uint16_t    srcId;         /**< Source ID */
  uint16_t    destId;        /**< Destination ID */
  uint16_t    tt;            /**< Transport Type: 16 bit or 8 bit identifiers as defined at @ref srioTransportTypes */  
  uint16_t    cc;            /**< Completion code */
  uint16_t    pri;           /**< 3-bit priority */
  uint16_t    msgType;       /**< Message type as defined at @ref srioMessageTypes */
  paSrioTypeInfo_t typeInfo; /**< Message Type specific parameters */
} paSrioInfo_t;


/**
 *  @ingroup palld_api_structures
 *  @brief  Packet routing configuration
 *
 *  @details paRouteInfo_t is used to specify the physical routing of packets out of the packet accelerator
 *           sub-system. Not all fields are required for all destinations. 
 *  @li      pa_DEST_DISCARD: none
 *  @li      pa_DEST_CONTINUE_PARSE_LUT1: 
 *  @li      pa_DEST_CONTINUE_PARSE_LUT2: customType, customIndex
 *  @li      pa_DEST_HOST: flowId, queue, mRoutehandle, swInfo0, cmd
 *  @li      pa_DEST_SASS: flowId, queue, swInfo0, swInfo1, cmd
 *  @li      pa_DEST_ETH: emacCtrl
 *  @li      pa_DEST_SRIO: flowId, queue, swInfo0, swInfo2, pktType
 */
typedef struct  {

  int      dest;                  /**<  Packet destination as defined at @ref pktDest */
  uint8_t  flowId;                /**<  For host, SA or SRIO destinations, specifies CPPI flow which defines free queues are used for receiving packets */
  uint16_t queue;                 /**<  For host, SA or SRIO destinations, specifies the destination queue */
  int      mRouteIndex;           /**<  For host, Multi-queue routing index (0 to (@ref pa_MAX_MULTI_ROUTE_SETS - 1)) 
                                        or @ref pa_NO_MULTI_ROUTE if multi routing not used */
  uint32_t swInfo0;               /**<  Placed in SwInfo0 for packets to host or SA; Placed in the PS Info for packets to SRIO */
  uint32_t swInfo1;               /**<  Placed in SwInfo1 for packets to the SA; Placed in the PS Info for packets to SRIO */
  int      customType;            /**<  For CONTINUE_PARSE_LUT1/LUT2 only, specifies the custom type as defined at @ref customType */
  uint8_t  customIndex;           /**<  For CONTINUE_PARSE_LUT1/LUT2 only, specifies the custom classification entry index */                                
  uint8_t  pktType_emacCtrl;      /**<  For destination SRIO, specify the 5-bit packet type toward SRIO 
                                        For destination HOST, EMAC, specify the EMAC control @ref emcOutputCtrlBits to the network */
  paCmdInfo_t *pCmd;              /**<  Pointer to the Command info to be executed prior to the packet forwarding. 
                                        NULL: no commads 
                                        @note only the following commands are supported within paRouteInfo_t 
                                              - pa_CMD_PATCH_DATA (up to two bytes only) (LUT2 only)
                                              - pa_CMD_CMDSET
                                              - pa_CMD_USR_STATS
                                              - pa_CMD_CMDSET_AND_USR_STATS 
                                    */                               
} paRouteInfo_t;

/**
 *   @def  pa_NO_MULTI_ROUTE
 *         Multi Route not enabled in this route
 */
#define pa_NO_MULTI_ROUTE   -1

/**
 *   @def  pa_MAX_MULTI_ROUTE_SETS
 *         The maximum number of multi-route sets supported
 */
#define pa_MAX_MULTI_ROUTE_SETS     32

/**
 *   @def  pa_MAX_MULTI_ROUTE_ENTRIES
 *         The maximum number of multi-route entries per muli-route set
 */
#define pa_MAX_MULTI_ROUTE_ENTRIES   8

/**
 *  @defgroup paEfOpInfoCtrlFlags  PA Egress Flow Operation Info Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Operation Info Control Flag Definitions
 *  Bitmap definition of the ctrlFlags in @ref paEfOpInfo_t. 
 */ 
/*@{*/
/**
 *  @def  pa_EF_OP_CONTROL_FLAG_FC_LOOKUP
 *        Flag -- 1: Perform Flow Cache lookup 
 *                0: Do not perform Flow Cache lookup, use the Eflow records specified within @ref paEfOpInfo_t 
 */
#define pa_EF_OP_CONTROL_FLAG_FC_LOOKUP           0x0001 
/*@}*/
/** @} */

/**
 *  @defgroup paEfOpInfoValidBit  PA Egress Flow Operation Info Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Egress Flow Operation Info Valid Bit Definitions
 *  Bitmap definition of the validBitfield in @ref paEfOpInfo_t. 
 *  It allows selective Egress Flow opertaion parameters
 */ 
/*@{*/
/**
 *  @def  pa_EF_OP_INFO_VALID_LVL1
 *        Egress Flow level 1 index is present
 */
#define pa_EF_OP_INFO_VALID_LVL1            0x0001 
/**
 *  @def  pa_EF_OP_INFO_VALID_LVL2
 *        Egress Flow level 2 index is present
 */
#define pa_EF_OP_INFO_VALID_LVL2            0x0002 
/**
 *  @def  pa_EF_OP_INFO_VALID_LVL3
 *        Egress Flow level 3 index is present
 */
#define pa_EF_OP_INFO_VALID_LVL3            0x0004 
/**
 *  @def  pa_EF_OP_INFO_VALID_LVL4
 *        Egress Flow level 4 index is present
 */
#define pa_EF_OP_INFO_VALID_LVL4            0x0008 

/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Egress Flow operation information
 *
 *  @details  paEfOpInfo_t is used to specifiy the Egress Flow operation parameters.
 *            It is used by @ref paRouteInfo2_t in the ingress path for IP forwarding
 *            operation and API @ref Pa_addFc for Flow Cache operation.
 *            Refer to @ref appendix4 for deatiled information
 */
typedef struct  {
  uint16_t    ctrlFlags;    /**< Specify Egress flow control flags as defined at @ref paEfOpInfoCtrlFlags */
  uint16_t    validBitMap;  /**< Specify valid parameters as defined at @ref paEfOpInfoValidBit */
  uint16_t    lvl1Index;    /**< Specify egress flow level 1 record index */
  uint16_t    lvl2Index;    /**< Specify egress flow level 2 record index */
  uint16_t    lvl3Index;    /**< Specify egress flow level 3 record index */
  uint16_t    lvl4Index;    /**< Specify egress flow level 4 record index */
} paEfOpInfo_t;


/**
 @defgroup paPriIntfRouteMode Priority-based or Interface-based routing mode
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Priority-based or Interface-based routing mode
 *
 *  paRoutePriIntf_e is used to specify the mode of priority-based 
 *  or interface-based routing.
 *  PASS forwards the matched packets to the desired QoS queue which is equal
 *  to the base queue plus an offset specified by the VLAN priority or DSCP value
 *  in prority-based routing such as pa_ROUTE_PRIORITY_VLAN or pa_ROUTE_PRIORITY_DSCP.
 *  PASS forwards the matched packets to the desired host queue which is equal 
 *  to the base queue plus an offset as the EMAC port (interface) number with the CPPI
 *  flow which is equal to the base flow number plus the EMAC port (interface) number
 *  optionally in interface-based routing such as pa_ROUTE_INTF or pa_ROUTE_INTF_W_FLOW.
 *  PASS forwards the matched packets to the derived QoS queue with derived CPPI flow
 *  based on the algorithm specified at @ref appendix6 in EQoS routing
 *
 *  @note: There is some use cases where output packets from QoS are delivered to
 *         PASS for pre-routing operation such as tx timestamp report and both 
 *         egress and ingress forwarding packets go through the same QoS. To support
 *         this use case, the PASS is enhanced to delay the post-classification command
 *         set execution until the packets re-entering PASS from QoS if Priority-based
 *         routing is selected. 
 */ 
/** @ingroup paPriIntfRouteMode */
/*@{*/
typedef enum {
  pa_ROUTE_PRIORITY_VLAN = 1, /**< Route by using VLAN P bits as priority */
  pa_ROUTE_PRIORITY_DSCP,     /**< Route by using DSCP bits as priority */
  pa_ROUTE_INTF,              /**< Route by using EMAC port (interface) number as destination queue offset */
  pa_ROUTE_INTF_W_FLOW,       /**< Route by using EMAC port (interface) number as both 
                                   destination queue and CPPI flow offset */   
  pa_ROUTE_EQoS_MODE          /**< Route by using priority map for enhanced QoS to support L2 Shapper */ 
} paRoutePriIntf_e;

/*@}*/
/** @} */

/**
 *  @defgroup paRouteInfoValidBits  PA Route Info Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Route Info Valid Bit Definitions
 *
 *  Bitmap definition of the validBitMap in @ref paRouteInfo2_t. 
 */ 
/*@{*/

/**
 *  @def  pa_ROUTE_INFO_VALID_MROUTEINDEX
 *        - Optional parameter mRouteIndex for Host routing is valid
 */
#define pa_ROUTE_INFO_VALID_MROUTEINDEX             (1<<0)

/**
 *  @def  pa_ROUTE_INFO_VALID_PKTTYPE_EMAC
 *        - Optional parameter pktType_emacCtrl for Host or EMAC routing is valid
 */
#define pa_ROUTE_INFO_VALID_PKTTYPE_EMAC            (1<<1)

/**
 *  @def  pa_ROUTE_INFO_VALID_PCMD
 *        - Optional parameter pCmd is valid
 */
#define pa_ROUTE_INFO_VALID_PCMD                    (1<<2)

/**
 *  @def  pa_ROUTE_INFO_VALID_PRIORITY_TYPE
 *        - Optional parameter priorityType used for Priority-based or interface-based routing is valid
 */
#define pa_ROUTE_INFO_VALID_PRIORITY_TYPE           (1<<3)

/**
 *  @def  pa_ROUTE_INFO_VALID_CTRLBITMAP
 *        - Optional parameter ctrlBitMap is valid
 */
#define pa_ROUTE_INFO_VALID_CTRLBITMAP              (1<<4)


/* @} */ /* ingroup */
/** @} */

/**
 * @defgroup  paRouteInfoCtrlBits PA Route Info Control Bits Definitions
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA Route Info Control Bits Definitions
 *
 * @brief Bitmap definition of the ctrlBitMap in @ref paRouteInfo2_t
 *
 */
/* @{ */
/**
 *  @def  pa_ROUTE_INFO_L2_PKT_CAPTURE
 *        Control Info -- Set: Duplicate and forward the matched L2 packet to the capture queue 
 *
 *  This flag will be check only if the destination is set to pa_DEST_CONTINUE_PARSE_LUT1.
 *  In this case, the parameter flowId, queue and swInfo0 are used to deliver the captured
 *  packet. 
 *
 *  @note: This feature is supported only on PASS Gen2 device. For PASS Gen1 devices, the L2 
 *         packet capture can be implemented with host multi-route feature where the packet 
 *         should be duplicated and routed to the capture queue and the queue 641 for continuous 
 *         classification on PASS Gen1 devices.   
 */
#define pa_ROUTE_INFO_L2_PKT_CAPTURE              0x0001  

/*  @}  */  
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Enhanced Packet routing configuration
 *
 *  @details  paRouteInfo2_t is the upgraded version of paRouteInfo_t to support additional routing 
 *            parameters over time while still maintaining backward compatibility. Future feature 
 *            enhancements will be supported through this API data structure only.
 *
 *            The validBitMap is used to specify which field is used for packet routing. 
 */
typedef struct  {
  uint32_t      validBitMap;      /**< 32-bit valid bitmap corresponding to each optional field as defined at @ref paRouteInfoValidBits */
  int           dest;             /**< Packet destination as defined at @ref pktDest */
  uint8_t       flowId;           /**< For host, SA or SRIO destinations, specifies CPPI flow which defines free queues are used for receiving packets */
  uint16_t      queue;            /**< For host, SA or SRIO destinations, specifies the destination queue */
  int           mRouteIndex;      /**< validBitMap[t0] For host, Multi-queue routing index (0 to (@ref pa_MAX_MULTI_ROUTE_SETS - 1) */
  uint32_t      swInfo0;          /**< For host, SA or SRIO destinations, placed in SwInfo0 for packets to host or SA; Placed in the PS Info for packets to SRIO */
  uint32_t      swInfo1;          /**< For host, SA or SRIO destinations, placed in SwInfo1 for packets to the SA; Placed in the PS Info for packets to SRIO */
  int           customType;       /**< For CONTINUE_PARSE_LUT1/LUT2 only, specifies the custom type as defined at @ref customType */
  uint8_t       customIndex;      /**< For CONTINUE_PARSE_LUT1/LUT2 only, specifies the custom classification entry index */                               
  uint8_t       pktType_emacCtrl; /**< validBitMap[t1] For destination SRIO, specify the 5-bit packet type toward SRIO
                                       For destination HOST, EMAC, specify the EMAC control @ref emcOutputCtrlBits to the network */
  paCmdInfo_t  *pCmd;             /**< validBitMap[t2] Pointer to the Command info to be executed prior to the packet forwarding.
                                       NULL: no commads
                                       @note only the following commands are supported within paRouteInfo_t and paRouteInfo2_t
                                        for ingress packets 
                                             - pa_CMD_PATCH_DATA (up to two bytes only) (LUT2 only)
                                             - pa_CMD_CMDSET
                                             - pa_CMD_USR_STATS
                                             - pa_CMD_CMDSET_AND_USR_STATS
                                       @note the post-classification commands specified by the command set will be executed when the packets re-entering PASS 
                                             from the QoS queue if priority-based routing is selected       
                                    */                                                                                   
  uint8_t       priorityType;     /**< validBitMap[t3]: For Host only, specify priority-based and/or interfcae-based routing mode as 
                                    *  defined at @ref paRoutePriIntf_e 
									*/
  paEfOpInfo_t *efOpInfo;         /**< For EFLOW only, egress flow operation info (PASS Gen2 only) */  
  uint32_t      ctrlBitMap;       /**< validBitMap[t4]: 32-bit control bitmap as defined at @ref paRouteInfoCtrlBits */
} paRouteInfo2_t;

/**
 * @defgroup  paPktCloneCtrlBits  PA Packet Capture/Port Mirror Control Bit Definitions
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA Packet Capture/Port Mirror Control Bit Definitions
 *
 * @brief Bitmap definition of the ctrlBitMap in @ref paPortMirrorConfig_t and @ref paPktCaptureConfig_t.  
 *
 */
/* @{ */
/**
 *  @def  pa_PKT_CLONE_ENABLE
 *        port mirror/packet capture control 
 *        please refer to @ref appendix5 for details about this mode. 
 */
#define  pa_PKT_CLONE_ENABLE                  1 

/**
 *  @def  pa_PKT_CLONE_INGRESS
 *        direction configuration 
 *        Set:   Ingress direction
 *        Clear: Egress  direction
 */
#define  pa_PKT_CLONE_INGRESS                 2 

/*  @}  */  
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  PA Interface based Port Mirror Configuration Information.
 *
 *  @details  paPortMirrorConfig_t is used to specify the port mirror configuration parameters of the PASS.
 *
 *            For the Ingress configuration: 
 *            All the ingress MAC packets entering PDSP0 (Ingress0) will be duplicated and forwarded to the 
 *            specified EMAC port to simulate Ethernet SW port mirroring operation prior to the classification 
 *            (lookup) operation based on the IF based configuration.
 *
 *            For the Egress configuration: 
 *            All the egress packets directed to EMAC port will be duplicated and forwarded 
 *            to specified mirror port to simulate Ethernet SW port mirroring operation 
 *            as part of the nextRoute command processing based on the IF based configuration.
 */
typedef struct {
  uint32_t  ctrlBitMap;       /**< Specifies various port mirror control bits as defined in @ref paPktCloneCtrlBits */
  uint8_t   portToBeMirrored; /**< Specifies port to be mirrored */
  uint8_t   mirrorPort;       /**< The mirror port */
} paPortMirrorConfig_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Interface based Packet Capture Configuration Information.
 *
 *  @details  paPktCaptureConfig_t is used to specify the packet capture configuration parameters of the PASS.
 *
 *            For the Ingress configuration: 
 *            All the ingress MAC packets entering PDSP0 (Ingress0) will be duplicated and forwarded to the 
 *            specified host queue for packet capturing prior to the classification (lookup) 
 *            operation based on the IF based configuration.
 *
 *            For the Egress configuration: 
 *            All the egress packets directed to EMAC port will be duplicated and forwarded 
 *            to the pecified host queue for packet capturing as part of the nextRoute 
 *            command processing based on the IF based configuration.
 */
typedef struct {
  uint32_t  ctrlBitMap;        /**< Specifies various packet capture control bits as defined in @ref paPktCloneCtrlBits */
  uint8_t   portToBeCaptured;  /**< Specifies port to be captured */
  uint32_t  swInfo0;           /**< Placed in SwInfo0 for packets to host */
  uint8_t   flowId;            /**< Specifies CPPI flow which defines free queues to be used for receiving packets */
  uint16_t  queue;             /**< Specifies the destination host queue */
} paPktCaptureConfig_t;

/**
 *  @defgroup paDrouteTypes PA Default Route Types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Default Route Types
 *
 *  @brief  These values are used to define interface-based ingress default route types.
 *
 *  @details  The interface-based ingress default route defines the global routing information for the
 *            packet types such as multicast packet, broadcast packet and non-matched unicast packet. 
 */
/*  @{  */
/**
 *
 *   @def  pa_DROUTE_MULTICAST
 *         Multicast packet default route index
 */
#define pa_DROUTE_MULTICAST        0

/**
 *
 *   @def  pa_DROUTE_BROADCAST
 *         Broadcast packet default route index
 */
#define pa_DROUTE_BROADCAST        1

/**
 *
 *   @def  pa_DROUTE_NO_MATCH
 *         Non-matched unicast packet default route index
 */
#define pa_DROUTE_NO_MATCH         2


/**
 *  @def   pa_DROUTE_MAX
 *         The maximum number of global default route types
 */
#define pa_DROUTE_MAX              3

/*  @}  */  
/** @} */

/**
 *  @defgroup paDefRouteCtrlBits  PA Interface-based Ingress Packet Default Route Control Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA Interface based Ingress Packet Default Route Control Bit Definitions
 *
 *  Bitmap definition of the ctrlBitMap in @ref paDefRouteConfig_t
 *         
 */ 
/*@{*/
/**
 *  @def  pa_EMAC_IF_DEFAULT_ROUTE_MC_ENABLE
 *        Control Info -- Set: Multicast default route enable
 *                        Clear: Multicast default route disable
 */
#define pa_EMAC_IF_DEFAULT_ROUTE_MC_ENABLE                            0x0001  

/**
 *  @def  pa_EMAC_IF_DEFAULT_ROUTE_BC_ENABLE
 *        Control Info -- Set: Broadcast default route enable
 *                        Clear: Broadcast default route disable
 */
#define pa_EMAC_IF_DEFAULT_ROUTE_BC_ENABLE                            0x0002

/**
 *  @def  pa_EMAC_IF_DEFAULT_ROUTE_UC_ENABLE
 *        Control Info -- Set: unicast packet no match default route enable
 *                        Clear: unicast packet no match default route disable
 */
#define pa_EMAC_IF_DEFAULT_ROUTE_UC_ENABLE                            0x0004

/**
 *  @def  pa_EMAC_IF_DEFAULT_ROUTE_MC_PRE_CLASSIFY_ENABLE
 *        Control Info -- Set:  default route for multicast pre classification enable
 *                        Clear: default route for multicast post classification disable
 */
#define pa_EMAC_IF_DEFAULT_ROUTE_MC_PRE_CLASSIFY_ENABLE               0x0008

/**
 *  @def  pa_EMAC_IF_DEFAULT_ROUTE_BC_PRE_CLASSIFY_ENABLE
 *        Control Info -- Set:  default route for broadcast pre classification enable
 *                        Clear: default route for broadcast post classification disable
 */
#define pa_EMAC_IF_DEFAULT_ROUTE_BC_PRE_CLASSIFY_ENABLE               0x0010


/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Interface based Ingress default routing information.
 *
 *  @details  paDefRouteConfig_t is used to specify the ingress default routing 
 *            configuration parameters of the PASS. 
 *            Refer to @ref appendix6 for details
 *            
 */
typedef struct {
  uint32_t               ctrlBitMap;       /**< Specifies various ingress default route control bits as defined at @ref paDefRouteCtrlBits */
  uint8_t                port;             /**< Specifies the ingress EMAC port number (1-based) */  
  paRouteInfo2_t         dRouteInfo[pa_DROUTE_MAX];      /**< Specifies the default route information for each packet type as defined at @ref paDrouteTypes */
} paDefRouteConfig_t;

/**
 * @defgroup  paEQoSCtrlBits PA Enhanced QoS Control Bits Definitions
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA enhanced QoS Control Bits Definitions
 *
 * @brief Bitmap definition of the ctrlBitMap in @ref paEQosModeConfig_t
 *
 */
/* @{ */
/**
 *  @def  pa_IF_EQoS_ROUTE_DP_BIT_MODE
 *        Control Info -- Set: DP-bit mode 
 *                        Clear: DSCP mode
 */
#define pa_IF_EQoS_ROUTE_DP_BIT_MODE              0x0001  

/**
 *  @def  pa_IF_EQoS_PRIORITY_OVERRIDE_ENABLE
 *        Control Info -- Set: priority override enable
 *                        Clear: priority override disable
 */
#define pa_IF_EQoS_PRIORITY_OVERRIDE_ENABLE       0x0002

/**
 *  @def  pa_IF_EQoS_VLAN_OVERRIDE_ENABLE
 *        Control Info -- Set: VLAN override enable (for Egress traffic only)
 *                        Clear: VLAN override disable (for Egress traffic only)
 */
#define pa_IF_EQoS_VLAN_OVERRIDE_ENABLE           0x0004

/*  @}  */  
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  Enhanced QoS Mode Route offset information. This is per dscp or priority bits
 *
 *  @details  paRouteOffset_t specifies the offset for flow and queue 
 *            with respect to a base CPPI flow and base QoS Queue. 
 */
typedef struct  {
  uint8_t   flowOffset;      /**<  Specifies CPPI flow offset */
  uint8_t   queueOffset;     /**<  Specifies the destination host queue offset */
} paRouteOffset_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  PA Interfcae based Enhanced QoS mode information
 *
 *  @details  paEQosModeConfig_t is used to specify the EQoS mode global configuration 
 *            parameters of the PASS. Refer to @ref appendix7 for details.
 */
typedef struct {
  uint32_t              ctrlBitMap;       /**< Specifies various EQoS mode control bits as defined in @ref paEQoSCtrlBits */
  paRouteOffset_t       pbitMap[8];       /**< Specifies the Pbit-to-flow/queue offset mapping */
  paRouteOffset_t       dscpMap[64];      /**< Specifies the DSCP-to-flow/queue offset mapping */
  uint8_t               port;             /**< Specifies the EMAC port number (1-based) for the EQoS configuration*/      
  uint8_t               ingressDefPri;    /**< ingress port default priority */
  uint16_t              vlanId;           /**< Specifies the VLAN ID to be used/replaced at the egress packet */
  uint8_t               flowBase;         /**< Specifies the CPPI flow base for egress (SoC generated) packets */
  uint16_t              queueBase;        /**< Specifies the QoS queue base for egress (SoC generated) packets */  
} paEQosModeConfig_t;

/**
 * @defgroup  paEmacPortCfgType PA EMAC Port Configuration Type
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA EMAC Port Configuration Type
 *
 * @brief Define the PA EMAC port configuration types used at @ref paEmacPortConfig_t
 *
 */
/* @{ */
/**
 *  @def  pa_EMAC_PORT_CFG_MIRROR
 *        port mirror configuration 
 *        Please refer to @ref appendix5 for details about this operation. 
 */
#define  pa_EMAC_PORT_CFG_MIRROR                 0  

/**
 *  @def  pa_EMAC_PORT_CFG_PKT_CAPTURE
 *        packet capture configuration 
 *        Please refer to @ref appendix5 for details about this operation.
 */
#define  pa_EMAC_PORT_CFG_PKT_CAPTURE            1 

/**
 *  @def  pa_EMAC_PORT_CFG_DEFAULT_ROUTE
 *        ingress packet default route configuration 
 *        Please refer to @ref appendix6 for details about this operation.
 */
#define  pa_EMAC_PORT_CFG_DEFAULT_ROUTE          2

/**
 *  @def  pa_EMAC_PORT_CFG_EQoS_MODE
 *        enhanced QoS mode configuration 
 *        please refer to @ref appendix7 for details about this operation.
 */
#define  pa_EMAC_PORT_CFG_EQoS_MODE              3

/*  @}  */  
/** @} */

/**
 *  @def  pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES
 *        The maximum number of emac port configuration entries for interfcae-based EMAC operations.
 *        Please note that this number is limited by the PASS internal memory size and therefore it 
 *        may be smaller than the number of available EMAC ports.
 */
#define pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES_GEN1          5
#define pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES_GEN2          9

#ifndef NSS_GEN2
#define pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES               pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES_GEN1
#else
#define pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES               pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES_GEN2
#endif

/**
 *  @ingroup palld_api_structures
 *  @brief  PA emac port configuration information.
 *
 *  @details  paEmacPortConfig_t is used to specify the interfcae based EMAC port configuration parameters
 *            for the following operations respectively
 *            - EMAC Port Mirroring 
 *            - Packet Capture
 *            - Ingress Default Route
 *            - Enhanced QoS Mode
 *
 *  Please refer to individual operation as described at @ref appendix5, @ref appendix6 and @ref appendix7 for further details.
 *  @note All entries of the port mirror or packet capture array should be in the same direction. The PA LLD will extract the 
 *        direction information of the first entry only.
 *
 */
typedef struct {
  uint16_t                   cfgType;          /**< Specify the EMAC port configuration type as defined at @ref paEmacPortCfgType 
                                                    to specify which structure to use under the union */
  uint16_t                   numEntries;       /**< Specify number of port entries to be configured */
  union {
   paPortMirrorConfig_t     *mirrorCfg;        /**< pointer to port mirror configuration array */
   paPktCaptureConfig_t     *pktCapCfg;        /**< pointer to packet capture configuration array */
   paDefRouteConfig_t       *defRouteCfg;      /**< pointer to default ingress route configuration array */
   paEQosModeConfig_t       *eQoSModeCfg;      /**< pointer to enhanced QoS Mode configuration arary */
  }u;                                          /**< Contain the port configuration specific parameters */
} paEmacPortConfig_t;

/**
 * @defgroup  paCtrlCode PA Control Code
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA Control Code
 *
 * @brief Define the PA LLD control code  
 *
 */
/** @ingroup paCtrlCode */
/* @{ */
/**
 *  @def  pa_CONTROL_SYS_CONFIG
 *        system-level configuration 
 */
#define  pa_CONTROL_SYS_CONFIG              0  

/**
 *  @def  pa_CONTROL_802_1ag_CONFIG
 *        802.1ag Detector configuration 
 */
#define  pa_CONTROL_802_1ag_CONFIG          1 

/**
 *  @def  pa_CONTROL_IPSEC_NAT_T_CONFIG
 *        IPSEC NAT-T Packet Detector configuration 
 */
#define  pa_CONTROL_IPSEC_NAT_T_CONFIG      2  

/**
 *  @def  pa_CONTROL_GTPU_CONFIG
 *        GTP-U configuration 
 */
#define  pa_CONTROL_GTPU_CONFIG             3  

/**
 *  @def  pa_CONTROL_EMAC_PORT_CONFIG
 *        EMAC_PORT_CONFIG configuration 
 */
#define  pa_CONTROL_EMAC_PORT_CONFIG        4

/**
 *  @def  pa_CONTROL_RA_CONFIG
 *        RA configuration 
 */
#define  pa_CONTROL_RA_CONFIG               5

/**
 *  @def  pa_CONTROL_TIME_OFFSET_CONFIG
 *        This control provides a provision to correct the initial values set for time offsets
 *        during Ethernet Operations And maintenance support
 */
#define  pa_CONTROL_TIME_OFFSET_CONFIG      6

/**
 *  @def  pa_CONTROL_MAX_CONFIG_GEN1
 *        Maximum global configuration types on NSS Gen1 devices 
 */
#define  pa_CONTROL_MAX_CONFIG_GEN1         pa_CONTROL_EMAC_PORT_CONFIG

/**
 *  @def  pa_CONTROL_MAX_CONFIG_GEN2
 *        Maximum global configuration types on NSS Gen2 devices
 */
#define  pa_CONTROL_MAX_CONFIG_GEN2         pa_CONTROL_TIME_OFFSET_CONFIG

/*  @}  */  
/** @} */

/**
 * @ingroup palld_api_structures
 * @brief PA Control Information structure
 *
 * @details Data structure defines PA control information used by API @ref Pa_control. 
 *
 */
typedef struct {
  uint16_t    code;                               /**< Specify the PA control code as defined at @ref paCtrlCode */
  union {
    paSysConfig_t              sysCfg;            /**< Specify system-level configuration parameters */
    pa802p1agDetConfig_t       pa802p1agDetCfg;   /**< Specify 802.1ag Detector configuration parameters */
    paIpsecNatTConfig_t        ipsecNatTDetCfg;   /**< Specify IPSEC NAT-T Detector configuration parameters */
    paGtpuConfig_t             gtpuCfg;           /**< Specify GTP-U configuration parameters */
    paEmacPortConfig_t         emacPortCfg;       /**< Specify interface based port configuration information */
    paRaConfig_t               raCfg;             /**< Specify RA global configuration information (PASS Gen2 only) 
                                                       @note RA global configuration does not require command buffer. */                                                              
    paSetTimeOffset_t          tOffsetCfg;        /**< Specify Ethernet OAM time stamp offset information  (PASS Gen2 only) */
  }params;                                        /**< Contain the control operation specific parameters */
} paCtrlInfo_t;

 
/**
 *  @defgroup mrEntryCtrlInfo   Multiroute Entry Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Multiroute Entry Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paMultiRouteEntry_t. 
 */ 
/*@{*/
/**
 *  @def  pa_MULTI_ROUTE_DESCRIPTOR_ONLY
 *        Control Info -- Set: Send descriptor without packet to the destination
 *                        Clear: Send both descriptor and the packet to the destination
 *  
 */
#define pa_MULTI_ROUTE_DESCRIPTOR_ONLY            0x01 
/*@{*/
/**
 *  @def  pa_MULTI_ROUTE_REPLACE_SWINFO
 *        Control Info -- Set: Replace the swInfo0 with the value provided here
 *                        Clear: Keep the original swInfo0 
 *  
 */
#define pa_MULTI_ROUTE_REPLACE_SWINFO             0x02 
/*@}*/
/** @} */
 
/**
 *  @ingroup palld_api_structures
 *  @brief  Packet Multi-route entry configuration
 *
 *  @details paMultiRouteEntry_t is used to specify the physical routing of packets per multi-route entry.
 *           It is only a subset of the Routing information defined at @ref paRouteInfo_t because those common 
 *           parameters such as swInfo0, swInfo1 must be already present in the packet descriptor.
 *           There is no restriction of the destination as long as it is accessible through PKTDMA queue. 
 */
typedef struct  {

  uint8_t   ctrlBitfield;    /**< Multi-Routing control information as defined at @ref mrEntryCtrlInfo */
  uint8_t   flowId;          /**< For host, specifies the CPPI flow which defines the free queues are used for receiving packets */
  uint16_t  queue;           /**< For host, specifies the destination queue */
  uint32_t  swInfo0;         /**< Placed in SwInfo0 for packets to host */

} paMultiRouteEntry_t;

/**
 *  @defgroup paMultiRouteModes Multi-route group configuration mode
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Multi-route group configuration mode
 *
 *  Definition of Multi-route group configuration mode supported in PA sub-system
 */ 
/** @ingroup paMultiRouteModes */
/*@{*/
typedef enum {
  pa_MULTI_ROUTE_MODE_CONFIG = 0, /**< Add or reconfigure the multi-route group */
  pa_MULTI_ROUTE_MODE_RESET       /**< Delete the multi-route group */
} paMultiRouteModes_e;
/*@}*/
/** @} */

/**
 *  @defgroup paCrcSizes PA CRC Sizes
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name CRC Sizes
 *
 *  Definition of CRC sizes supported in PA sub-system
 */ 
/** @ingroup paCrcSizes */
/*@{*/
typedef enum {
  pa_CRC_SIZE_8 = 0,        /**< 8-bit CRC */
  pa_CRC_SIZE_16,           /**< 16-bit CRC */
  pa_CRC_SIZE_24,           /**< 24-bit CRC */
  pa_CRC_SIZE_32            /**< 32-bit CRC */
} paCrcSizes_e;
/*@}*/
/** @} */

/**
 *  @defgroup crcConfigCtrlInfo  CRC Engine Configuration Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name CRC Engine Configuration Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paCrcConfig_t. 
 */ 
/*@{*/
/**
 *  @def  pa_CRC_CONFIG_RIGHT_SHIFT
 *        Control Info -- Set: Right shift CRC (b0 to b7)
 *                        Clear: Left shift CRC (b7 to b0)
 */
#define pa_CRC_CONFIG_RIGHT_SHIFT           0x0001 
/**
 *  @def  pa_CRC_CONFIG_INVERSE_RESULT
 *        Control Info -- Set: a 'NOT' operation is applied to the final CRC result
 */
#define pa_CRC_CONFIG_INVERSE_RESULT        0x0002 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  CRC Engine configuration
 *
 *  @details paCrcConfig_t is used to configure the CRC engines within the PA sub-system.
 *           There are several CRC engines within various processing stages in the PA sub-system.
 *           The locations of CRC engines are defined and described at @ref paCrcInst. 
 *           The CRC engine is used to perform CRC operation required by some network protocol such as
 *           SCTP and/or the user-specified CRC command. It only supports one type of CRC
 *           per configuration.
 *
 *  @note Only one type of CRC calcualtion is supported by one CRC engine per configuration.
 *        It is the responsibility of the module user to configure the specific CRC engine by 
 *        calling @ref Pa_configCrcEngine.
 */
 
typedef struct {

  uint16_t      ctrlBitfield;   /**< CRC configuration control information as defined at @ref crcConfigCtrlInfo */
  paCrcSizes_e  size;           /**< CRC sizes as defined at @ref paCrcSizes_e (PASS Gen1 only)*/
  uint32_t      polynomial;     /**< Specify the CRC polynomial in the format of 0xabcdefgh. For example,
                                     x32+x28+x27+x26+x25+x23+x22+x20+x19+x18+x14+x13+x11+x10+x9+x8+x6+1 
                                     ==> 0x1EDC6F41
                                     x16+x15+x2+1 ==>0x80050000 */
  uint32_t      initValue;      /**< CRC initial value (PASS Gen1 only)*/   
} paCrcConfig_t;


/**
 *  @defgroup timestampScalerFactor Timestamp Scaler Factor
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name Timestamp Scaler Factor
 *
 *  Definition of PA timestamp scaler factor supported in PA sub-system
 *
 *  @note  pa_TIMESTAMP_SCALER_FACTOR_1 is not supported. It is defined here
 *         for reference purpose.
 */ 
/** @ingroup timestampScalerFactor */
/*@{*/
typedef enum {
  pa_TIMESTAMP_SCALER_FACTOR_1 = -1,       
  pa_TIMESTAMP_SCALER_FACTOR_2 = 0,       
  pa_TIMESTAMP_SCALER_FACTOR_4,       
  pa_TIMESTAMP_SCALER_FACTOR_8,       
  pa_TIMESTAMP_SCALER_FACTOR_16,       
  pa_TIMESTAMP_SCALER_FACTOR_32,       
  pa_TIMESTAMP_SCALER_FACTOR_64,       
  pa_TIMESTAMP_SCALER_FACTOR_128,       
  pa_TIMESTAMP_SCALER_FACTOR_256,       
  pa_TIMESTAMP_SCALER_FACTOR_512,       
  pa_TIMESTAMP_SCALER_FACTOR_1024,       
  pa_TIMESTAMP_SCALER_FACTOR_2048,       
  pa_TIMESTAMP_SCALER_FACTOR_4096,       
  pa_TIMESTAMP_SCALER_FACTOR_8192       
} paTimestampScalerFactor_e;
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  Timestamp configuration
 *
 *  @details paTimestampConfig_t is used to configure the timer which is used to generate timestamp in
 *           the PA sub-system.
 *  @verbatim 
             The 16-bit timer connected to PDSP0 is reserved for timestamp generation.
             The timestamp will be 0 until the timer is enabled.
             The timestamp unit is equal to (the scaler factor)/350 us.  
    @endverbatim
 *
 *  @note: The PDSP timer does not support pa_TIMESTAMP_SCALER_FACTOR_1.
 *         The timer will be disabled if unspported scaler factor is used.
 */
 
typedef struct {
  uint16_t                   enable;     /**< Enable/Disable(1/0) the timestamp generation */
  paTimestampScalerFactor_e  factor;     /**< Timestamp scaler factor as defined at @ref timestampScalerFactor */
} paTimestampConfig_t;

/**
 *  @defgroup paUsrStatsTypes PA User-defined Ststaistics Counter Types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name User-defined Ststaistics Counter Types
 *
 *  Definition of Counter types of the User-defined Statistics
 */ 
/** @ingroup paUsrStatsTypes */
/*@{*/
typedef enum {
  pa_USR_STATS_TYPE_PACKET = 0,   /**< Packet Counter */
  pa_USR_STATS_TYPE_BYTE,         /**< Byte Counter */
  pa_USR_STATS_TYPE_DISABLE       /**< Counter to be disabled */
} paUsrStatsTypes_e;
/*@}*/
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  User-defined statistics counter entry configuration information
 *
 *  @details paUsrStatsCounterEntryConfig_t defines the operation parameters of each user-defined statistics.
 */
 
typedef struct {
  uint16_t          cntIndex; /**< Index of the counter */
  uint16_t          cntLnk;   /**< Index of the next level counter. 0xFFFF: No linking counter */ 
  paUsrStatsTypes_e cntType;  /**< Counter type (packet counter */
} paUsrStatsCounterEntryConfig_t;

/**
 *   @def  pa_USR_STATS_LNK_END
 *         Indicate that there is no next layer counter
 */
#define pa_USR_STATS_LNK_END                0xFFFF


/**
 *  @defgroup usrStatsCounterConfigCtrlInfo  User-defined Statistics Counter Configuration Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name User-defined Statistics Counter Configuration Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paUsrStatsCounterConfig_t 
 */ 
/*@{*/
/**
 *  @def  pa_USR_STATS_CONFIG_RESET
 *        Control Info -- Set: Reset all counter control blocks to its default setting (packet counter without link to the next layer)
 */
#define pa_USR_STATS_CONFIG_RESET           0x0001 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  User-defined statistics counter configuration information
 *
 *  @details paUsrStatsCounterConfig_t contains an array of the entry configuration information.
 */
 
typedef struct {
  uint16_t          ctrlBitfield;      /**< User-defined statistics counter configuration control information as defined at @ref usrStatsCounterConfigCtrlInfo */
  uint16_t          numCnt;            /**< Number of counters to be configured */
  paUsrStatsCounterEntryConfig_t* cntInfo;  /**< Array of counter configuration as specified at @ref paUsrStatsCounterEntryConfig_t */
} paUsrStatsCounterConfig_t;

/**
 *  @ingroup palld_api_structures
 *  @brief  User-defined statistics configuration information
 *
 *  @details paUsrStatsConfigInfo_t is used to perform user-defined statistics related configuration. It is used by 
 *           API function @ref Pa_configUsrStats.
 */
typedef struct {
  paUsrStatsCounterConfig_t* pCntCfg;  /**< Pointer to the user-defined statistics counter configuration. */
} paUsrStatsConfigInfo_t;

/**
 *  @defgroup usrStatsAllocCtrlInfo  User-defined Statistics Allocation Control Info Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name User-defined Statistics Allocation Control Info Bit Definitions
 *
 *  Bitmap definition of the ctrlBitField in @ref paUsrStatsAlloc_t 
 */ 
/*@{*/
/**
 *  @def  pa_USR_STATS_ALLOC_64B_CNT
 *        Control Info -- Set: Allocate a 64-bit counter
 *                        Clear: Allocate a 32-bit counter
 */
#define pa_USR_STATS_ALLOC_64B_CNT           0x0001 

/**
 *  @def  pa_USR_STATS_ALLOC_CNT_PRESENT
 *        Control Info -- Set: Counter index is provided. Need to verify whether the counter index is valid.
 *                        Clear: Counter index is not present. Need to allocate one.
 */
#define pa_USR_STATS_ALLOC_CNT_PRESENT       0x0002 

/*@}*/
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  User-defined statistics Allocation information
 *
 *  @details paUsrStatsAlloc_t defines the user-defined statistic allocation parameters which are used to specify 
 *           the charistics of the counter to be allocated or verified.
 */
 
typedef struct {
  uint16_t          ctrlBitfield;  /**< User-defined statistics allocation control information as defined at @ref usrStatsAllocCtrlInfo */
  uint16_t          cntIndex;      /**< Index of the counter */
} paUsrStatsAlloc_t;  


/**
 * @defgroup  paSubSysStates PA Sub-system Queries and States
 * @ingroup palld_api_constants
 * @{
 *
 * @name PA Sub-system Queries and States
 *
 *  PA Sub-system reset state and query arguments used by API function @ref Pa_resetControl
 */
/* @{ */
/**
 *  @def  pa_STATE_RESET  
 *        The Sub-system is in reset
 */
#define pa_STATE_RESET            0  /**< Sub-system state reset */

/**
 *  @def  pa_STATE_ENABLE
 *        The Sub-system state is enabled
 */
#define pa_STATE_ENABLE           1  /**< Sub-system state enable  */

/**
 *  @def  pa_STATE_QUERY
 *        Query the Sub-system state
 */
#define pa_STATE_QUERY            2  /**< Query the Sub-system state */

/**
 *  @def  pa_STATE_INCONSISTENT
 *        The Sub-system state is partially enabled
 */
#define pa_STATE_INCONSISTENT     3  /**< Sub-system is partially enabled */

/**
 *  @def  pa_STATE_INVALID_REQUEST
 *        Invalid state command to the Sub-system
 */
#define pa_STATE_INVALID_REQUEST  4  /**< Invalid state command to the Sub-system */

/**
 *  @def  pa_STATE_ENABLE_FAILED
 *        The Sub-system did not respond after restart
 */
#define pa_STATE_ENABLE_FAILED    5  /**<  The Sub-system did not respond after restart */

/**
 *  @def  pa_STATE_RESOURCE_USE_DENIED
 *        Resource manager denied the firmware use
 */
#define pa_STATE_RESOURCE_USE_DENIED    6  /**<  Resource manager denied the firmware use */

/* @}  */ 
/** @} */


/**
 *  @ingroup palld_api_structures
 *  @brief  paSState_t defines the operating state of the packet accelerator sub-system
 *
 *  @details  The values in @ref paSubSysStates are used both to set the state of the packet accelerator
 *            sub-system (pa_STATE_RESET and pa_STATE_ENABLE) as well as show the current state
 *            of the system (all values).
 */
typedef int paSSstate_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Classify1 Statistics Structure
 *
 * @details This structures define the PA Classify1-specific statistics provided 
 *          with API function @ref Pa_formatStatsReply ().
 */
typedef struct paClassify1Stats_s {

  uint32_t nPackets;                /**< Number of packets entering Classify1 PDSPs */
  uint32_t nIpv4Packets;            /**< Number of IPv4 packets */
  uint32_t nIpv4PacketsInner;       /**< Number of Inner IPv4 packets */
  uint32_t nIpv6Packets;            /**< Number of IPv6 packets */
  uint32_t nIpv6PacketsInner;       /**< Number of Inner IPv6 packets */
  uint32_t nCustomPackets;          /**< Number of custom LUT1 packets */
  uint32_t nSrioPackets;            /**< Number of SRIO packets */
  uint32_t nLlcSnapFail;            /**< Number of packets with corrupt LLC Snap */
  uint32_t nTableMatch;             /**< Number of packets with table match found */
  uint32_t nNoTableMatch;           /**< Number of packets without table match found */
  uint32_t nIpFrag;                 /**< Number of Ingress fragmented IP packets */
  uint32_t nIpDepthOverflow;        /**< Number of packets with too many IP layers */
  uint32_t nVlanDepthOverflow;      /**< Number of packets with too many VLANs */
  uint32_t nGreDepthOverflow;       /**< Number of packets with too many GREs */
  uint32_t nMplsPackets;            /**< Number of MPLS packets */
  uint32_t nParseFail;              /**< Number of packets which can not be parsed */
  uint32_t nInvalidIPv6Opt;         /**< Number of IPv6 packets which contains invalid IPv6 options */
  uint32_t nTxIpFrag;               /**< Number of Egress fragmented IP packets */
  uint32_t nSilentDiscard;          /**< Number of packets dropped */
  uint32_t nInvalidControl;         /**< Number of packet received with invalid control information */
  uint32_t nInvalidState;           /**< Number of times the PA detected an illegal state and recovered */
  uint32_t nSystemFail;             /**< Number of times the PA detected an unrecoverable state and restarted */
  
} paClassify1Stats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Classify2 Statistics Structure
 *
 * @details This structures define the PA Classify2-specific statistics provided 
 *          with API function @ref Pa_formatStatsReply ().
 */
typedef struct paClassify2Stats_s  {
  
  uint32_t nPackets;                /**< Number of packets entering Classify2 PDSP */ 
  uint32_t nUdp;                    /**< Number of UDP packets */
  uint32_t nTcp;                    /**< Number of TCP packets */
  uint32_t nCustom;                 /**< Number of custom LUT2 packets */
  uint32_t reserved3;               /**< Reserved for future use */
  uint32_t reserved4;               /**< Reserved for future use */
  uint32_t nSilentDiscard;          /**< Number of packets dropped */
  uint32_t nInvalidControl;         /**< Number of packet received with invalid control information */

} paClassify2Stats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Modifier Statistics Structure
 *
 * @details This structures define the PA Modifier-specific statistics provided 
 *          with API function @ref Pa_formatStatsReply ().
 */
typedef struct paModifyStats_s   {
  uint32_t nCommandFail;            /**< Number of invalid commands */
  
} paModifyStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Common Statistics Structure
 *
 * @details This structures define the PA Common statistics provided 
 *          with API function @ref Pa_formatStatsReply ().
 */
typedef struct paCommonStats_s  {

  uint32_t reserved5;               /**< Reserved for future use */
  
} paCommonStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA System Statistics Structure
 *
 * @details This structures define the PA System statistics provided 
 *          with API function @ref Pa_formatStatsReply ().
 */

typedef struct paSysStats_s  {

  paClassify1Stats_t classify1;     /**< Classify1-specific statistics */
  paClassify2Stats_t classify2;     /**< Classify2-specific statistics */
  paModifyStats_t    modify;        /**< Modifier-specific statistics */
  paCommonStats_t    common;        /**< Common statistics */
  
} paSysStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA User-defined Statistics Structure
 *
 * @details This structures define the PA User-defined statistics provided 
 *          with API function @ref Pa_requestUsrStats ().
 */

typedef struct paUsrStats_s  {

  uint64_t   count64[pa_USR_STATS_MAX_64B_COUNTERS];     /**< Array of general purpose 64-bit counters */
  uint32_t   count32[pa_USR_STATS_MAX_32B_COUNTERS];     /**< Array of general purpose 32-bit counters */
  
} paUsrStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Reassembly Group Statistics Structure
 *
 * @details This structures define the PA RA group-specific statistics  
 */
typedef struct paRaGroupStats_s {

  uint32_t nReasmPackets;           /**< Number of successfully reassembled packets of Group N*/
  uint32_t nFrags;                  /**< Number of fragmented IP packets of group N*/
  uint32_t nPackets;                /**< Number of packets of Group N */
  uint32_t nCxtTOwSOP;              /**< Number of IP packets where contexts associated with Group N time out  
                                         before being completely reassembled, the SOP fragment has been received */
  uint32_t nCxtTOwSOPBytes;         /**< Number of payload bytes of IP packets where contexts associated with Group N 
                                         time out before being completely reassembled, the SOP fragment has been received */
  uint32_t nCxtTOwoSOP;             /**< Number of IP packets where contexts associated with Group N time out before being 
                                         completely reassembled, the SOP fragment has not been received */
  uint32_t nCxtTOwoSOPBytes;        /**< Number of payload bytes of IP packets where contexts associated with Group N time out 
                                         before being completely reassembled, the SOP fragment has not been received */
  uint32_t nZeroByte;               /**< Number of IP fragments with zero-byte payload */
  uint32_t reserved2;               /**< Reserved for future use */
  uint32_t nIpv6Overlap;            /**< Number of IPv6 packets which are discarded because a fragment of this 
                                         packet arrives that overlaps data previously received in another 
                                         fragment for that same packet */
  uint32_t nIpv6OverlapBytes;       /**< Number of IPv6 payload bytes which are discarded because a fragment of this 
                                         packet arrives that overlaps data previously received in another 
                                         fragment for that same packet */
  uint32_t nLargePackets;           /**< Number of IP packets which are discarded because the completely reassembled 
                                         packet greater than 64KB */
  uint32_t nIpv4TcpErr;             /**< Number of IP packets which are discarded due to TCP error, i.e. a fragment is 
                                         received that has a protocol of TCP and a fragment offset of 1 */
  uint32_t nFragLenErr;             /**< Number of IP packets which are discarded due to an incorrect fragment length */
  uint32_t nIpv4IllegalIHL;         /**< Number of IPv4 fragments which are discarded due to an incorrect IP header length */
  uint32_t nSmallFragments;         /**< Number of IP fragments which is too small 
                                         - IPv4: Packets that are not last fragment and are smaller than minIpv4PktSize 
                                         - IPv6: Packets that are not last fragment and are smaller than 56 bytes 
                                         - IPV6: Packets that are last fragment and are smaller than 49 bytes */
  uint32_t nIllegalFragLen;         /**< Number of IP fragments that is not the last fragment, has a length 
                                         that is not a multiple of 8 bytes */
  uint32_t nCxtCompletedDiscard;    /**< Number of IP fragments which are discarded because they are for a context that has 
                                         already been completed or timed out */
  uint32_t nCxtCompletedDiscardBytes;   /**< Total payload bytes of IP fragments which are discarded because they are for a context that has 
                                         already been completed or timed out */
  
} paRaGroupStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA RA Statistics Structure
 *
 * @details This structures define the PA RA statistics provided 
 *          with API function @ref Pa_requestUsrStats ().
 */

typedef struct paRaStats_s  {
    paRaGroupStats_t group[pa_RA_NUM_GROUPS];   /**< array of group-specific RA statistics */ 
} paRaStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA ACL Entry Statistics Structure
 *
 * @details This structures define the PA ACL per-entry statistics provided 
 *          with API function @ref Pa_queryAclStats ().
 */

typedef struct paAclStats_s  {

  uint32_t   nMatchPackets;     /**< Number of packets which matchs the ACL rule */
  uint32_t   nMatchBytes;       /**< Total bytes of the matched packets */
  
} paAclStats_t;

/**
 * @ingroup palld_api_structures
 * @brief PA Timestamp Structure
 *
 * This structure defines the 64-bit system timestamp provided upon request with @ref Pa_getTimestamp ().
 * ---------------------------------------
 * | 16 bits     | 32 bits     | 16 bits |
 * |  hi_hi      |  hi         | lo      |
 * ---------------------------------------
 *@note: The structure is updated to have upper 16 bit (hi_hi) to be backwards compatible with 48-bit 
 *       timestamp support
 */
typedef struct {
  uint16_t   hi_hi;      /**< Upper Upper 16 bits of the 64-bit PASS timestamp */  
  uint32_t   hi;         /**< Upper 32 bits of the 64-bit PASS timestamp */
  uint16_t   lo;         /**< Lower 16 bits of the 64-bit PASS timestamp */
} paTimestamp_t;

/**
 *  @defgroup paApiParamValidBits  PA API Parameter Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name  PA API Parameter Valid Bit Definitions
 *
 *  Bitmap definition of the validBitMap in @ref paParamDesc. 
 */ 
/*@{*/

/**
 *  @def  pa_PARAM_VALID_LUTINST
 *        - Set: Application specifies the LUT1 instance
 *        - Clear: LLD determines the LUT1 instance based on other input parameters
 */
#define pa_PARAM_VALID_LUTINST              (1<<0)

/**
 *  @def  pa_PARAM_VALID_INDEX
 *        - Set: Application specifies the LUT1 index to insert this entry
 *        - Clear: PASS determines where in the LUT1 table to insert this entrry
 */
#define pa_PARAM_VALID_INDEX                (1<<1)

/**
 *  @def  pa_PARAM_VALID_PREVLINK
 *        - Set: Previous link  is valid and it should be used as part of classification criteria
 *        - Claer: Previous link is inavlid
 */
#define pa_PARAM_VALID_PREVLINK             (1<<2)

/**
 *  @def  pa_PARAM_VALID_NEXTLINK
 *        - Set: The specified virtual link in stead of the physical link should be used as part of 
 *               classification criteria at the next stage
 *        - Clear: Use physical link at the next stage
 */
#define pa_PARAM_VALID_NEXTLINK             (1<<3)

/**
 *  @def  pa_PARAM_VALID_CTRLBITMAP
 *        - Set: there is a valid control bit map
 *        - Clear: control bit map is not valid
 * @note: this is applicable for Gen1 only since LUT1 ordering is no longer a problem in Gen2
 */
#define pa_PARAM_VALID_CTRLBITMAP           (1<<4)

/* @} */ /* ingroup */
/** @} */

/**
 *  @defgroup paApiParamCtrlBits  PA API Parameter Control Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name  PA API Parameter Control Bit Definitions
 *
 *  Bitmap definition of the ctrlBitMap in @ref paParamDesc. 
 */ 
/*@{*/

/**
 *  @def  pa_PARAM_CTRL_REPLACE
 *        - Set: Application specifies the replace index for LUT1 entry (0-63). 
 *        - Clear: No replace action 
 *  @note: applicable for Gen1 only
 */
#define pa_PARAM_CTRL_REPLACE              (1<<0)

/* @} */ /* ingroup */
/** @} */

/**
 *  @ingroup palld_api_structures
 *  @brief  PA API parameters structure
 *
 *  @details  This structure defines the common parameters of the next generation APIs such as
 *            @ref Pa_addMac2 and @ref Pa_addIp2. This structure includes a validBitMap of 
 *            optional parameters so that it can evolve while maintaining backward-compatibility.
 *
 *            The parameter validBitMap specifies which optional parameters are valid 
 *            1: used; 0: not used.
 *              
 */
typedef struct {
  uint32_t        validBitMap;  /**<  32-bit bitmap corresponding to usage of each optional field */
  uint32_t        ctrlBitMap;   /**<  32-bit bitmap corresponding to usage of each control field */
  int             lutInst;      /**<  validBitMap[t0] Specify which LUT1 (0-2) should be used. */
  int             index;        /**<  validBitMap[t1] Specify the index of the LUT1 entry (0-63).*/
  paLnkHandle_t   prevLink;     /**<  validBitMap[t2] An optional L2 or L3 handle, or virtual link handle */
  paLnkHandle_t   nextLink;     /**<  validBitMap[t3] An optional virtual link handle */
  paRouteInfo2_t  *routeInfo;   /**<  Where to send a packet that matches */
  paRouteInfo2_t  *nextRtFail;  /**<  Where to send a packet that matches, but fails to match any entry at the next classification stage */
} paParamDesc;

/**
 *  @defgroup paLut2ApiParamValidBits  PA LUT2 API Parameter Valid Bit Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name  PA LUT2 API Parameter Valid Bit Definitions
 *
 *  Bitmap definition of the validBitmap in @ref paLut2ParamDesc. 
 */ 
/*@{*/

/**
 *  @def  pa_LUT2_PARAM_VALID_CTRL_BITMAP
 *        - Set:   ctrlBitMap is valid
 *        - Clear: ctrlBitMap is not used
 */
#define pa_LUT2_PARAM_VALID_CTRL_BITMAP              (1<<0)

/**
 *  @def  pa_LUT2_PARAM_VALID_DIVERTQ
 *        - Set: Divert Queue is specified for Queue Diversion operation
 *        - Clear: Divert queue is not used
 */
#define pa_LUT2_PARAM_VALID_DIVERTQ                  (1<<1)

/* @} */ /* ingroup */
/** @} */

/**
 *  @defgroup paLut2ParamCtrlFlags  PA LUT2 API Common Parameters Control Flag Definitions
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name PA LUT2 API Common Parameters Control Flag Definitions
 *  Bitmap definition of the ctrlFlags in paLut2ParamDesc. 
 */ 
/*@{*/
/**
 *  @def  pa_LUT2_INFO_CONTROL_FLAG_REPLACE
 *        Flag -- 1: Replace the existing LUT2 entry
 *                0: Add new LUT2 entry 
 */
#define pa_LUT2_INFO_CONTROL_FLAG_REPLACE            (1<<0) 
/*@}*/
/** @} */

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_control performs system-level control and configuration
 *
 *  @details This function performs PASS control operations including system-level figurations. 
 *           The system-level configurations are divided into several sub-groups which can be configured 
 *           independently. The default configuration will be used until this API is invoked.
 *
 *           On return the command buffer (cmd) contains a formatted command for the sub-system when the cmdSize
 *           is set to non-zero. The destination for the command is provided in cmdDest. The module user must send 
 *           the formatted command to the sub-system. The sub-system will generate a reply
 *           and this reply must be sent back to this module through the @ref Pa_forwardResult API.
 *
 *
 *  @param[in]      handle      The PA LLD instance identifier
 *  @param[in]      ctrl        Control information
 *  @param[out]     cmd         Where the created command is placed
 *  @param[in,out]  cmdSize     Input the size of cmd buffer, on output the actual size used. @ref cmdMinBufSize
 *  @param[in]      reply       Where the sub-system sends the command reply
 *  @param[out]     cmdDest     Value (@ref cmdTxDest) 
 *  @retval                     Value (@ref ReturnValues)
 */
paReturn_t Pa_control (Pa_Handle      handle, 
                       paCtrlInfo_t  *ctrl, 
                       paCmd_t        cmd,
                       uint16_t       *cmdSize,
                       paCmdReply_t   *reply,
                       int            *cmdDest);

/**
 *  @ingroup palld_api_structures
 *  @brief  PA LUT2 API parameters structure
 *
 *  @details  This structure defines the common parameters of the next generation LUT2 APIs such as
 *            @ref Pa_addPort2. This structure includes a validBitmap of optional parameters so that 
 *            it can evolve while maintaining backward-compatibility.
 *
 *            The parameter validBitmap specifies which optional parameters are valid 
 *            1: used; 0: not used.
 *              
 */
typedef struct {
  uint32_t        validBitMap;  /**< 32-bit Bitmap corresponding to usage of each optional field as specified at @ref paLut2ApiParamValidBits */
  uint32_t        ctrlFlags;    /**< 32-bit control flags as defined at @ref paLut2ParamCtrlFlags */
  uint16_t        divertQ;      /**< The source queue for atomic queue diversion with LUT2 update */
} paLut2ParamDesc;

/**
 *  @ingroup palld_api_functions
 *  @brief   Pa_addSrio adds a SRIO entry to the L2 table
 *
 *  @details This function is used to add or replace an entry into the L2 table (see @ref netlayers).
 *           A new entry is added if the SRIO configuration info is unique in the modules handle table. 
 *           If the value is not unique then the routing information for the existing entry is changed to
 *           the values provided in the function.
 *
 *           On return the command buffer (cmd) contains a formatted command for the sub-system. The 
 *           destination for the command is provided in cmdDest. The module user must send the formatted
 *           command to the sub-system. The sub-system will generate a reply
 *           and this reply must be sent back to this module through the API @ref Pa_forwardResult.
 *
 *           This command as well as @ref Pa_addIp operate with a strong dependence on entry order.
 *           See section table @ref order for a description on the operation of the sub-system and
 *           table entry ordering.
 *
 *  @param[in]      iHandle         The driver instance handle
 *  @param[in]      index           Specify the index of the LUT1 entry (0-63). Set to pa_LUT1_INDEX_NOT_SPECIFIED if not specified
 *  @param[in]      srioInfo        Value @ref paSrioInfo_t
 *  @param[in]      nextHdr         The next header type to be parsed following the SRIO classification
 *                                  Refer to @ref NextHeaderTypes for all supported protocols
 *                                  Set to pa_HDR_TYPE_UNKNOWN if no further prasing is required
 *  @param[in]      nextHdrOffset   Offset to the next header from the beginning of the packet
 *  @param[in]      routeInfo       Match packet routing information
 *  @param[in]      nextRtFail      Routing information for subsequent match failures
 *  @param[out]     handle          Pointer to L2 Handle
 *  @param[out]     cmd             Where the created command is placed
 *  @param[in,out]  cmdSize         Input the size of cmd buffer, on output the actual size used. @ref cmdMinBufSize
 *  @param[in]      reply           Where the sub-system sends the command reply
 *  @param[out]     cmdDest         Value (@ref cmdTxDest) 
 *  @retval                         Value (@ref ReturnValues)
 *  @pre                            A driver instance must be created and tables initialized
 *
 *  @note No table entry validation will be proformed if the LUT1 index is specified at this function
 *
 */

paReturn_t Pa_addSrio (  Pa_Handle         iHandle,
                         int               index,
                         paSrioInfo_t      *srioInfo,
                         uint16_t          nextHdr,
                         uint16_t          nextHdrOffset,
                         paRouteInfo_t     *routeInfo,
                         paRouteInfo_t     *nextRtFail,
                         paHandleL2L3_t    *handle,
                         paCmd_t           cmd,
                         uint16_t          *cmdSize,
                         paCmdReply_t      *reply,
                         int               *cmdDest);

/**
 *  @ingroup palld_api_functions
 *  @brief   Pa_addMac adds a mac address to the L2 table
 *
 *  @details This function is used to add or replace an entry into the L2 table (see @ref netlayers).
 *           A new entry is added if the MAC configuration info is unique in the modules handle table. If
 *           the value is not unique then the routing information for the existing entry is changed to
 *           the values provided in the function.
 *
 *           L2 values that are not to be used for packet routing are set to 0.
 *
 *           On return the command buffer (cmd) contains a formatted command for the sub-system. The 
 *           destination for the command is provided in cmdDest. The module user must send the formatted
 *           command to the sub-system. The sub-system will generate a reply
 *           and this reply must be sent back to this module through the @ref Pa_forwardResult API.
 *
 *           This command as well as @ref Pa_addIp operate with a strong dependence on entry order.
 *           See section table @ref order for a description on the operation of the sub-system and
 *           table entry ordering.
 *
 *
 *  @param[in]      iHandle     The driver instance handle
 *  @param[in]      index       Specify the index of the LUT1 entry (0-63). Set to pa_LUT1_INDEX_NOT_SPECIFIED if not specified
 *  @param[in]      ethInfo     Value @ref paEthInfo_t
 *  @param[in]      routeInfo   Match packet routing information
 *  @param[in]      nextRtFail  Routing information for subsequent match failures
 *  @param[out]     handle      Pointer to L2 Handle
 *  @param[out]     cmd         Where the created command is placed
 *  @param[in,out]  cmdSize     Input the size of cmd buffer, on output the actual size used. @ref cmdMinBufSize
 *  @param[in]      reply       Where the sub-system sends the command reply
 *  @param[out]     cmdDest     Value (@ref cmdTxDest) 
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized
 *
 *  @note No table entry validation will be proformed if the LUT1 index is specified at this function
 *
 */

paReturn_t Pa_addMac  (  Pa_Handle         iHandle,
                         int               index,
                         paEthInfo_t       *ethInfo,
                         paRouteInfo_t     *routeInfo,
                         paRouteInfo_t     *nextRtFail,
                         paHandleL2L3_t    *handle,
                         paCmd_t           cmd,
                         uint16_t          *cmdSize,
                         paCmdReply_t      *reply,
                         int               *cmdDest);
                         
/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_addMac2 adds a mac address to the L2 table
 *
 *   @details  Pa_addMac2 is the next generation of API to replace @ref Pa_addMac eventually. This new API
 *             covers the entire functionality of Pa_addMac and it is designed to support more features
 *             while maintain backward-compatibility over time.
 *
 *  @param[in]      iHandle     The driver instance handle
 *  @param[in]      ethInfo     Value @ref paEthInfo2_t
 *  @param[in]      params      Common API parameters @ref paParamDesc 
 *  @param[in,out]  retHandle   Pointer to L2 Handle. LLD puts the allocated L2 handle in this location. During 
 *                              L2 replace operation, this parameter would point to the L2 handle to be replaced.
 *  @param[out]     cmd         Where the created command is placed
 *  @param[in,out]  cmdSize     Input the size of cmd buffer, on output the actual size used. @ref cmdMinBufSize
 *  @param[in]      reply       Where the sub-system sends the command reply
 *  @param[out]     cmdDest     Value (@ref cmdTxDest) 
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized
 *              
 */
paReturn_t Pa_addMac2  (  Pa_Handle        iHandle,
                          paEthInfo2_t     *ethInfo,    /**<  Value @ref paEthInfo2_t */
                          paParamDesc      *params,
                          paLnkHandle_t    *retHandle,  /**<  Pointer to the returned L2 handle */
                          paCmd_t          cmd,
                          uint16_t         *cmdSize,
                          paCmdReply_t     *reply,
                          int              *cmdDest);

/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_addEoamFlow adds an Ethernet OAM target flow entry to the EOAM table
 *
 *   @details  A new entry is added if the EOAM flow configuration info is unique in the modules handle table. If
 *           the value is not unique then the routing information for the existing entry is changed to
 *           the values provided in the function.
 *
 *           On return the command buffer (cmd) contains a formatted command for the sub-system. The 
 *           destination for the command is provided in cmdDest. The module user must send the formatted
 *           command to the sub-system. The sub-system will generate a reply
 *           and this reply must be sent back to this module through the @ref Pa_forwardResult API.
 *           Please refer to @ref appendix8 for details about EOAM mode.
 *
 *  @param[in]      iHandle     The driver instance handle
 *  @param[in]      ethInfo     Value @ref paEthInfo2_t
 *  @param[in]      eoamInfo    Ethernet OAM target flow information
 *  @param[out]     handle      Pointer to EOAM Handle. LLD puts the allocated EOAM handle in this location.
 *  @param[out]     cmd         Where the created command is placed
 *  @param[in,out]  cmdSize     Input the size of cmd buffer, on output the actual size used. @ref cmdMinBufSize
 *  @param[in]      reply       Where the sub-system sends the command reply
 *  @param[out]     cmdDest     Value (@ref cmdTxDest) 
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized, Ethernet OAM System Configuration
 *                              and EOAM timer offset configurations are complete.
 *              
 */

paReturn_t Pa_addEoamFlow  (Pa_Handle         iHandle,
                            paEthInfo2_t      *ethInfo,    
                            paEoamFlowInfo_t  *eoamInfo,   
                            paHandleEoam_t    *handle,     
                            paCmd_t           cmd,
                            uint16_t          *cmdSize,
                            paCmdReply_t      *reply,
                            int               *cmdDest);

                         
/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_delHandle deletes a MAC/SRIO or IP handle
 *
 *   @details  This function is used to remove an entry from the sub-system L2 or L3 (LUT1) lookup (see @ref netlayers).
 *             When a handle is deleted it can create stale handles. For example, an L3 handle can reference
 *             an L2 handle, and an L4 handle can reference an L3 handle. The module does not check for 
 *             references to a stale handle, the module user is responsible for maintaining reference coherency.
 *             It is recommended that the handle should not be deleted if the API function @ref Pa_getHandleRefCount 
 *             returns non-zero reference count.
 *
 *   @param[in]     iHandle     The driver instance handle
 *   @param[in]     handle      Pointer to the l2/l3 handle to delete
 *   @param[out]    cmd         Where the created command is placed
 *   @param[in]     cmdSize     The size of the cmd buffer
 *   @param[in]     reply       Where the sub-system sends the command reply
 *   @param[out]    cmdDest     Value (@ref cmdTxDest)
 *   @retval                    Value (@ref ReturnValues)
 *   @pre                       A driver instance must be created and tables initialized
 */
paReturn_t Pa_delHandle (Pa_Handle       iHandle,
                         paHandleL2L3_t  *handle, 
                         paCmd_t         cmd,
                         uint16_t        *cmdSize,
                         paCmdReply_t    *reply,
                         int             *cmdDest );

/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_delL4Handle deletes a UDP/TCP/GTPU/CustomLUT2 handle
 *
 *   @details This function is used to remove an entry from the sub-system L4 (LUT2) handle entry. 
 *
 *   @param[in]      iHandle   The driver instance handle
 *   @param[in, out] handle    Pointer to the L4 handle to delete
 *   @param[out]     cmd       Where the created command is placed
 *   @param[in]      cmdSize   The size of the cmd buffer
 *   @param[in]      reply     Where the sub-system sends the reply
 *   @param[out]     cmdDest   Value (@ref cmdTxDest)
 *   @retval                   Value (@ref ReturnValues)
 *   @pre                      A driver instance must be created and tables initialized
 */
paReturn_t Pa_delL4Handle (Pa_Handle      iHandle,
                          paHandleL4_t    handle, 
                          paCmd_t         cmd,
                          uint16_t        *cmdSize,
                          paCmdReply_t    *reply,
                          int             *cmdDest );
                          
/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_delAclHandle deletes an ACL handle
 *
 *   @details  This function is used to remove an entry from the LUT1-ACL lookup 
 *
 *   @param[in]     iHandle     The driver instance handle
 *   @param[in]     handle      Pointer to the ACL handle to delete
 *   @param[out]    cmd         Where the created command is placed
 *   @param[in]     cmdSize     The size of the cmd buffer
 *   @param[in]     reply       Where the sub-system sends the command reply
 *   @param[out]    cmdDest     Value (@ref cmdTxDest)
 *   @retval                    Value (@ref ReturnValues)
 *   @pre                       A driver instance must be created and tables initialized
 */
paReturn_t Pa_delAclHandle (Pa_Handle       iHandle,
                            paHandleAcl_t   *handle, 
                            paCmd_t         cmd,
                            uint16_t        *cmdSize,
                            paCmdReply_t    *reply,
                            int             *cmdDest );

/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_delEoamHandle deletes an EOAM handle
 *
 *   @details  This function is used to remove an entry from the LUT1-EOAM lookup 
 *            Please refer to @ref appendix8 for details about EOAM mode.
 *
 *   @param[in]     iHandle     The driver instance handle
 *   @param[in]     handle      Pointer to the EOAM handle to delete
 *   @param[out]    cmd         Where the created command is placed
 *   @param[in]     cmdSize     The size of the cmd buffer
 *   @param[in]     reply       Where the sub-system sends the command reply
 *   @param[out]    cmdDest     Value (@ref cmdTxDest)
 *   @retval                    Value (@ref ReturnValues)
 *   @pre                       A driver instance must be created and tables initialized
 */
paReturn_t Pa_delEoamHandle (Pa_Handle       iHandle,
                            paHandleEoam_t  *handle, 
                            paCmd_t         cmd,
                            uint16_t        *cmdSize,
                            paCmdReply_t    *reply,
                            int             *cmdDest );


/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_addIp adds an IP address to the L3 table
 *
 *   @details  This function is used to add or replace an entry in the L3 table (see @ref netlayers).
 *             A new entry is added if the IP configuration info is unique in the modules handle table.
 *             If the value is not unique then the routing information for the existing entry is changed
 *             to the values provided in the function.
 *
 *             The LLD will determine where this entry is added based on following rules
 *             - If there is no previous link or the previous link is a L2 (MAC/SRIO) entry, this entry will be
 *               added into LUT1_1
 *             - If the previous link is L3 (IP/Custom LUT1), this entry will be added into LUT1_2
 *             
 *             The module user can overwrite the default rules by specifying the desired LUT1 instance. 
 *
 *             The PASS will determine which entry of the specified LUT1 table is used for this entry based on
 *             its internal algorithm if the module user does not specify the LUT1 index.  
 *
 *             L3 values that are used for packet routing should be set as described in @ref paIpInfo_t.
 *
 *             The @ref paHandleL2L3_t prevLink is used to link this entry to an L2 or L3 entry already made
 *             by a call to @ref Pa_addMac or Pa_addIp. If the link is enabled then a packet will match the IP 
 *             information provided in ipInfo only if the same packet has already matched at the L2 level as  
 *             described by prevLink. To disable linking the value of prevLink is set to NULL.
 *
 *             On return the command buffer (cmd) contains a formatted command for the sub-system. The
 *             destination for the command is provided in cmdDest. The module user must send the formatted
 *             command to the sub-system. The sub-system will generate a reply and this reply must be
 *             sent back to this module through the API @ref Pa_forwardResult.
 *
 *             This command as well as @ref Pa_addMac operates with a strong dependence on entry order.
 *             See section table @ref order for a description on the operation of the sub-system and
 *             table entry ordering.
 *
 *
 *
 *   @param[in]    iHandle     The driver instance handle
 *   @param[in]    lutInst     Specify which LUT1 (0-2) should be used.  Set to pa_LUT_INST_NOT_SPECIFIED if not specified
 *   @param[in]    index       Specify the index of the LUT1 entry (0-63). Set to pa_LUT1_INDEX_NOT_SPECIFIED if not specified
 *   @param[in]    ipInfo      Value @ref paIpInfo_t
 *   @param[in]    prevLink    An optional L2 or L3 handle
 *   @param[in]    routeInfo   Where to send a packet that matches
 *   @param[in]    nextRtFail  Where to send a packet that matches, but later fails
 *   @param[out]   retHandle   Pointer to the returned L3 handle
 *   @param[out]   cmd         Buffer where the PASS command is created
 *   @param[in]    cmdSize     The size of the cmd buffer
 *   @param[in]    reply       Where the response to the PASS command is routed
 *   @param[out]   cmdDest     Value (@ref cmdTxDest)
 *   @retval                   Value (@ref ReturnValues)
 *   @pre                      A driver instance must be created and tables initialized
 *
 *  @note No table entry validation will be proformed if the LUT1 index is specified at this function.
 *
 *  @note When ipInfo (@ref paIpInfo_t) has only SPI, prevLink parameter is recommended 
 *        to be set for Gen1 and mandatory for Gen2 due to hardware limitations.
 *
 */
paReturn_t  Pa_addIp  ( Pa_Handle          iHandle,
                        int                lutInst,
                        int                index, 
                        paIpInfo_t        *ipInfo,
                        paHandleL2L3_t     prevLink,
                        paRouteInfo_t     *routeInfo,
                        paRouteInfo_t     *nextRtFail,
                        paHandleL2L3_t    *retHandle,
                        paCmd_t            cmd,
                        uint16_t          *cmdSize,
                        paCmdReply_t      *reply,
                        int               *cmdDest );
                        
/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_addIp2 adds an IP address to the L3 table
 *
 *   @details  Pa_addIp2 is the next generation of API to replace @ref Pa_addIp eventually. This new API
 *             covers the entire functionality of Pa_addIP and it is designed to support more features
 *             while maintain backward-compatibility over time.
 *
 *  @param[in]      iHandle     The driver instance handle
 *  @param[in]      ipInfo      Value @ref paIpInfo2_t
 *  @param[in]      params      Common API parameters @ref paParamDesc 
 *  @param[in,out]  retHandle   Pointer to L3 Handle. LLD puts the allocated L3 handle in this location. During 
 *                              L3 replace operation(Gen1 only), this parameter would point to the L3 handle to be replaced.
 *  @param[out]     cmd         Where the created command is placed
 *  @param[in,out]  cmdSize     Input the size of cmd buffer, on output the actual size used. @ref cmdMinBufSize
 *  @param[in]      reply       Where the sub-system sends the command reply
 *  @param[out]     cmdDest     Value (@ref cmdTxDest) 
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized
 * 
 *  @note                       When ipInfo (@ref paIpInfo2_t) has only SPI, prevLink parameter in  @ref paParamDesc is recommended 
 *                              to be set for Gen1 and mandatory for Gen2 due to hardware limitations.
 */
paReturn_t  Pa_addIp2 ( Pa_Handle       iHandle,
                        paIpInfo2_t     *ipInfo,     
                        paParamDesc     *params,
                        paLnkHandle_t   *retHandle,  
                        paCmd_t         cmd,
                        uint16_t        *cmdSize,
                        paCmdReply_t    *reply,
                        int             *cmdDest
                        );

/**
 *  @defgroup  VirtualLnkType Virtual Link types
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   VirtualLnkTypes
 *  @brief  Defines the virtual link destination type
 *
 *  @note   The packet accelerator module supports linking to 
 *          virtual links at OuterIp only at the moment.
 *           
 */
/* @{ */
/**
 *  @def  pa_VIRTUAL_LNK_TYPE_MAC
 *        MAC
 */
#define  pa_VIRTUAL_LNK_TYPE_MAC        0

/**
 *  @def  pa_VIRTUAL_LNK_TYPE_OUTER_IP
 *        Outer IP
 */
#define  pa_VIRTUAL_LNK_TYPE_OUTER_IP   1

/**
 *  @def   pa_VIRTUAL_LNK_TYPE_INNER_IP
 *        Inner IP
 */
#define  pa_VIRTUAL_LNK_TYPE_INNER_IP   2
  
/*  @}  */  
/** @} */

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_addVirtualLink allocates a new virtual link within the PA instance
 *  
 *  @details  This function is called to request a new virtual link 
 *
 *  @param[in]      iHandle     The driver instance handle
 *  @param[in,out]  vlinkHdl    Pointer to virtual link handle
 *  @param[in]      lnkType     Value (@ref VirtualLnkType) 
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized
 *
 */
paReturn_t Pa_addVirtualLink(Pa_Handle       iHandle,
                             paLnkHandle_t   *vlinkHdl, 
                             int8_t          lnkType      
                            );

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_delVirtualLink frees the specified virtual link within the PA instance
 *
 *  @details  This function is used to remove a virtual link 
 *
 *  @param[in]      iHandle     The driver instance handle
 *  @param[in,out]  vlinkHdl    Pointer to virtual link handle
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized
 *
 */
paReturn_t Pa_delVirtualLink(Pa_Handle       iHandle,
                             paLnkHandle_t   *vlinkHdl
                             );
                             
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_allocUsrStats allocates or verifies a set of user-defined statistics
 *  
 *  @details  This function is called to request or verify a number of user-defined statistics. 
 *            The return value pa_RESOURCE_USE_DENIED will be used if there are not enough user-defined 
 *            statistics available or one of the provided counter indexes is not valid.
 *
 *  @param[in]      iHandle     The driver instance handle
 *  @param[in, out] pNumCnt     In:Number of user-defined statistics requested; Out: Number of user-defined statistics allocated  
 *  @param[in, out] cntList     Array of user-defined statistics allocation parameters 
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized
 *
 *  @note: This function is optional when the application owns the entire set of user-defined statistics or uses a set of 
 *         pre-allocated user-defined statistics. However, the PASS will verify the user-defined statistics list and may
 *         return error code pa_RESOURCE_USE_DENIED if RM is enabled when API @ref Pa_configUsrStats, @ref Pa_requestUsrStatsList
 *         and etc are invoked. 
 */
paReturn_t Pa_allocUsrStats(Pa_Handle           iHandle,
                            int                *pNumCnt,
                            paUsrStatsAlloc_t  *cntList
                            );
                            
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_freeUsrStats free a set of user-defined statistics
 *  
 *  @details  This function is called to free a set of user-defined statistics.  
 *             
 *
 *  @param[in]      iHandle     The driver instance handle
 *  @param[in]      numCnt      Number of user-defined statistics to be freed  
 *  @param[in]      cntList     Pointer to list of user-defined statistics to be freed
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized
 *
 *  @note: This function is optional when the application owns the entire set of user-defined statistics or uses a set of 
 *         pre-allocated user-defined statistics. 
 */
paReturn_t Pa_freeUsrStats(Pa_Handle        iHandle,
                           int              numCnt,
                           uint16_t        *cntList
                           );

/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_addAcl adds an ACL entry to the ACL table
 *
 *   @details  This function is used to add an entry in the ACL table.
 *             The PASS ACL table maintains an ordered list of ACL rules. This function will add ACL entries
 *             in descending order, i.e. the new rule will be inserted at the bottom of the ACL table unless
 *             the parameter nextEntry is specified, in this case, the new entry will be added in front of the 
 *             next entry. 
 *
 *             The are two ACL (LUT1) tables supported by PASS, one for outer IP and the other for inner IP.
 *             The ACL instance should be specified by @ref paLut1Inst.  
 *
 *             The @ref paHandleL2L3_t prevLink is used to link this entry to an L2 or L3 entry already made
 *             by a call to @ref Pa_addMac or Pa_addIp. If the link is enabled then a packet will match the ACL 
 *             information provided in aclInfo only if the same packet has already matched at the L2/L3 level as  
 *             described by prevLink. To disable linking the value of prevLink is set to NULL.
 *
 *             On return the command buffer (cmd) contains a formatted command for the sub-system. The
 *             destination for the command is provided in cmdDest. The module user must send the formatted
 *             command to the sub-system. The sub-system will generate a reply and this reply must be
 *             sent back to this module through the API @ref Pa_forwardResult.
 *
 *   @param[in]    iHandle     The driver instance handle
 *   @param[in]    aclInst     Specify which ACL LUT1 (@ref paAclInst) should be used.  
 *   @param[in]    aclAction   Specify ACL match action as @ref paAclActionTypes
 *   @param[in]    aclInfo     Value @ref paAclInfo_t
 *   @param[in]    prevLink    An optional L2 or L3 handle
 *   @param[in]    nextEntry   An optional ACL handle as the next ACL rule
 *   @param[out]   retHandle   Pointer to the returned ACL handle
 *   @param[out]   cmd         Buffer where the PASS command is created
 *   @param[in]    cmdSize     The size of the cmd buffer
 *   @param[in]    reply       Where the response to the PASS command is routed
 *   @param[out]   cmdDest     Value (@ref cmdTxDest)
 *   @param[out]   timeToNextCall Time in microseconds to indicate that this API is not available until this time elapses
 *   @retval                   Value (@ref ReturnValues)
 *   @pre                      A driver instance must be created and tables initialized
 *
 *   @note No table entry validation will be proformed if the LUT1 index is specified at this function
 *   @note To maintain the entry order of the ACL table, the function does not support entry
 *         replacement with updated action. The application needs to call @ref Pa_delAclHandle at first and
 *         then call this API to add the replaced entry. 
 *                              
 */
paReturn_t  Pa_addAcl  (Pa_Handle          iHandle,
                        int                aclInst,
                        int                aclAction,
                        paAclInfo_t       *aclInfo,
                        paHandleL2L3_t     prevLink,
                        paHandleAcl_t      nextEntry,
                        paHandleAcl_t     *retHandle,
                        paCmd_t            cmd,
                        uint16_t          *cmdSize,
                        paCmdReply_t      *reply,
                        int               *cmdDest,
                        uint32_t          *timeToNextCall);

/**
 *  @defgroup  paLut2PortSize LUT2 Port Size Values
 *  @ingroup palld_api_constants
 *  @{
 *
 *  @name   LUT2 Port Size Values
 *  @brief  Defines the LUT2 port size supported by PA.
 *
 *  @details The PA LUT2 supports both 16-bit and 32-bit entry matching. It can be used to classify 
 *           based on the UDP/IP 16-bit destination port with or without upper layer link or the GTP-U
 *           32-bit Tunnel ID. No other Layer 4 or Layer 5 protocol is supported. 
 */
/* @{ */
/**
 *  @def  pa_LUT2_PORT_SIZE_16
 *        16-bit port number such as UDP/TCP port
 *        
 */
#define  pa_LUT2_PORT_SIZE_16    0

/**
 *  @def  pa_LUT2_PORT_SIZE_32
 *        32-bit port number such as GTP-U Tunnel ID
 */
#define  pa_LUT2_PORT_SIZE_32    1
  
/*  @}  */  
/** @} */
                        
                        
/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_addPort adds a destination port to the L4 (LUT2) table
 *
 *   @details This function is used to add an entry to the L4 (LUT2) table (see @ref netlayers). Only the
 *            destination port can be set, along with a link to previous L3 handle 
 *            (see @ref Pa_addIp) through linkHandle.
 *
 *            This module does not keep track of the L4 handles, so calling the function
 *            a second time with the same destination port and link handle will simply replace the
 *            previous entry. It is recommended to set the replace flag to indicate that this entry is
 *            already at the LUT2 table. This feature may be used to change the routing information without 
 *            deleting and adding the matching port. 
 *            This API also initiates the atomic queue diversion operation, which means that the QMSS moves 
 *            the entries in the diverted queue to the destination queue, if the divertQ is specified and 
 *            fReplace flag is set. In this case, the PASS will complete the LUT2 update, wait for the queue 
 *            diversion to be complete and then resume processing incoming packets.
 *            Unlike entries in the L2 and L3 table, the order of entry is not important.  
 *
 *            The type of transport header (TCP/UDP) is not specified here. If the type of transport
 *            is part of the packet routing criteria it is specified in the protocol type field
 *            in @ref paIpInfo_t in the call to @ref Pa_addIp.
 *
 *            This function supports both 16-bit and 32-bit port specified by the parameter portSize.
 *            However, there are the following restrictions for 32-bit ports
 *  @verbatim 
               1. The link to the previous LUT1 match can not be used so that the destID 
                  should be unique regressless of the previous L3 adddreses
               2. The 32-bit LUT2 lookup can not be mixed with the other TCP/UDP or custom LUT2 lookup. 
    @endverbatim
 *   
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the @ref Pa_forwardResult API.
 *
 *   @param[in]     iHandle     The driver instance handle
 *   @param[in]     portSize    The input port size (@ref paLut2PortSize)
 *   @param[in]     destPort    The destination TCP/UDP port
 *   @param[in]     linkHandle  An L3 handle that is linked to the destination port
 *   @param[in]     fReplace    Flag to indicate whether the entry exists
 *   @param[in]     divertQ     The source queue for atomic queue diversion with LUT2 update
 *                              Set to pa_PARAMS_NOT_SPECIFIED if not specified
 *   @param[in]     routeInfo   Where to send a packet that matches
 *   @param[out]    retHandle   A blank handle where the return handle is placed
 *   @param[out]    cmd         Buffer where the PASS command is created
 *   @param[in]     cmdSize     The size of the cmd buffer
 *   @param[out]    reply       Where the response to the PASS command is routed
 *   @param[out]    cmdDest     Value (@ref cmdTxDest)
 *   @retval                    Value (@ref ReturnValues)
 *   @pre                       A driver instance must be created and tables initialized
 *
 *   @note The linkHandle is mandatory for 16-bit TCP/UDP port or 32-bit GTPU port when pa_GTPU_CTRL_USE_LINK is set.
 *         The linkHandle will be ignored for 32-bit GTPU port when pa_GTPU_CTRL_USE_LINK is cleared
 *
 */
paReturn_t  Pa_addPort ( Pa_Handle       iHandle,
                         int             portSize,
                         uint32_t        destPort,
                         paHandleL2L3_t  linkHandle, 
                         uint16_t        fReplace,
                         uint16_t        divertQ,
                         paRouteInfo_t  *routeInfo,
                         paHandleL4_t    retHandle,
                         paCmd_t         cmd,
                         uint16_t       *cmdSize,
                         paCmdReply_t   *reply,
                         int            *cmdDest );
          
/**
 *   @ingroup palld_api_functions
 *   @brief  Pa_addPort2 adds a destination port to the L4 (LUT2) table
 *
 *   @details  Pa_addPort2 is the next generation of API to replace @ref Pa_addPort eventually. This new API
 *             covers the entire functionality of Pa_addPort and it is designed to support more features
 *             while maintain backward-compatibility over time.
 *
 *   @param[in]     iHandle     The driver instance handle
 *   @param[in]     portSize    The input port size (@ref paLut2PortSize)
 *   @param[in]     destPort    The destination TCP/UDP port
 *   @param[in]     linkHandle  An L3 handle that is linked to the destination port
 *   @param[in]     params      Common LUT2 API parameters @ref paLut2ParamDesc 
 *   @param[in]     routeInfo   Where to send a packet that matches
 *   @param[out]    retHandle   A blank handle where the return handle is placed
 *   @param[out]    cmd         Buffer where the PASS command is created
 *   @param[in]     cmdSize     The size of the cmd buffer
 *   @param[out]    reply       Where the response to the PASS command is routed
 *   @param[out]    cmdDest     Value (@ref cmdTxDest)
 *   @retval                    Value (@ref ReturnValues)
 *   @pre                       A driver instance must be created and tables initialized
 *
 */
paReturn_t  Pa_addPort2 (Pa_Handle        iHandle,
                         int              portSize,
                         uint32_t         destPort,
                         paHandleL2L3_t   linkHandle, 
                         paLut2ParamDesc *params,
                         paRouteInfo2_t  *routeInfo,
                         paHandleL4_t     retHandle,
                         paCmd_t          cmd,
                         uint16_t        *cmdSize,
                         paCmdReply_t    *reply,
                         int             *cmdDest );
                         
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_setCustomLUT1 performs the global configuration for level 3 (LUT1) custom lookups
 *
 *  @details  This command is typically issued once per system and is used to configure the
 *            PA for performing network layer 3 (LUT1) custom lookups. 
 *            It specifies the offset and byte masks which the PA
 *            subsystem uses for parsing a packet that has entered custom LUT1 
 *            classification directed from the previous match route.
 *            It also specifies the next header type and offset to be used for continuous 
 *            parsing
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the @ref Pa_forwardResult API.
 *
 *  @param[in]    iHandle          The driver instance handle
 *  @param[in]    custIndex        The level 3 (LUT1) custom index
 *  @param[in]    parseByteOffset  Where the PA begins custom match (relative to the L3 start)
 *  @param[in]    nextHdr          The next header type to be parsed following the custom header
 *                                 Refer to @ref NextHeaderTypes for all supported protocols
 *                                 Set to pa_HDR_TYPE_UNKNOWN if no further prasing is required
 *  @param[in]    nextHdrOffset    Offset to the next header from the beginning of the custom header
 *  @param[in]    byteMasks        The bitmap of bits in the parse that matter
 *  @param[out]   cmd              Buffer where the PASS command is created
 *  @param[in]    cmdSize          On entry the size of the cmd buffer, on exit the size of the command
 *  @param[in]    reply            Where the response to the PASS command is routed
 *  @param[out]   cmdDest          Value (@ref cmdTxDest)
 *  @retval                        Value (@ref ReturnValues)
 *  @pre                           A driver instance must be created and tables initialized
 * 
 *  @note There is up to @ref pa_MAX_CUSTOM_TYPES_LUT1 LUT1 custom types supported by PASS.
 */
paReturn_t Pa_setCustomLUT1 ( Pa_Handle       iHandle,
                              uint16_t        custIndex,
                              uint16_t        parseByteOffset,
                              uint16_t        nextHdr,
                              uint16_t        nextHdrOffset,
                              uint8_t         byteMasks[pa_NUM_BYTES_CUSTOM_LUT1],
                              paCmd_t         cmd,
                              uint16_t       *cmdSize,
                              paCmdReply_t   *reply,
                              int            *cmdDest );
                            
/**
 *  @ingroup palld_api_functions
 *  @brief Pa_AddCustomLUT1 adds a custom lookup entry to the lookup tables (LUT1).
 *
 *  @details  This command is called to add a specific match entry to the L3 (LUT1) lookup table. This
 *            function is called once per desired custom LUT1 match criteria.
 *
 *            The LLD will determine where this entry is added based on following rules
 *            - If there is no previous link or the previous link is a L2 (MAC/SRIO) entry, this entry will be
 *              added into LUT1_1
 *            - If the previous link is L3 (IP/Custom LUT1), this entry will be added into LUT1_2
 *            
 *            The module user can overwrite the default rules by specifying the desired LUT1 instance. 
 *
 *            The PASS will determine which entry of the specified LUT1 table is used for this entry based on
 *            its internal algorithm if the module user does not specify the LUT1 index.  
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the @ref Pa_forwardResult API.
 *
 *  @param[in]   iHandle          The driver instance handle
 *  @param[in]   custIndex        The level 3 (LUT1) custom index
 *  @param[in]   lutInst          Specify which LUT1 (0-2) should be used.  Set to pa_LUT_INST_NOT_SPECIFIED if not specified
 *  @param[in]   index            Specify the index of the LUT1 entry (0-63). Set to pa_LUT1_INDEX_NOT_SPECIFIED if not specified
 *  @param[in]   match            The byte values that describe the match entry
 *  @param[in]   prevLink         An optional L2 or L3 handle that links to this lookup
 *  @param[in]   routeInfo        Where to send a packet that matches
 *  @param[in]   nextRtFail       Where to send a packet that matches here, but fails next parse level
 *  @param[out]  retHandle        The returned L3 handle
 *  @param[out]  cmd              Buffer where the command is created
 *  @param[in]   cmdSize          On entry the size of the cmd buffer, on exit the size of the command
 *  @param[in]   reply            Where the response to the PASS command is routed
 *  @param[out]  cmdDest          Value (@ref cmdTxDest)
 *  @retval                       Value (@ref ReturnValues)
 *  @pre                          A driver instance must be created and tables initialized
 */
paReturn_t Pa_addCustomLUT1 ( Pa_Handle       iHandle,
                              uint16_t        custIndex,
                              int             lutInst,
                              int             index, 
                              uint8_t         match[pa_NUM_BYTES_CUSTOM_LUT1],
                              paHandleL2L3_t  prevLink,
                              paRouteInfo_t  *routeInfo,
                              paRouteInfo_t  *nextRtFail,
                              paHandleL2L3_t *retHandle,
                              paCmd_t         cmd,
                              uint16_t       *cmdSize,
                              paCmdReply_t   *reply,
                              int            *cmdDest );
                            
/**
 *  @ingroup palld_api_functions
 *  @brief Pa_setCustomLUT2 performs the global configuration for level 4 (LUT2) custom lookups
 *
 *  @details  This command is typically called once per system and is used to configure the
 *            PA for performing network layer 4 (LUT2) custom lookups. 
 *            If handleLink is true then only 3 bytes and 3 offsets are available
 *            for matching. The fourth one is used to store the previous match information.
 *            In this case the first 3 values in the byteOffsets and byteMasks arrays are
 *            valid.
 *
 *            If setMask is non-zero, it will be ORed with the first byteMask and the match byte.
 *            It is used to distinguish this custom LUT2 entry from other custom LUT2 and standard 
 *            LUT2 entries.
 *            
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the API @ref Pa_forwardResult.
 *
 *  @param[in]  iHandle         Driver instance handle
 *  @param[in]  custIndex       Level 4 (LUT2) custom index
 *  @param[in]  handleLink      Set to TRUE to use one byte of the match to hold previous match info
 *  @param[in]  custHdrSize     Size of fixed-length custom header in bytes, which is used to adjust 
 *                              location of the next protocol header in case the packet needs to be 
 *                              processed by another module such as SASS or host application. This 
 *                              parameter should be set to zero for all other types of headers
 *  @param[in]  byteOffsets     Array of offsets to the bytes to use in custom matching
 *  @param[in]  byteMasks       Array of bits that are valid in the custom matching
 *  @param[in]  setMask         Bits to be set at the first match byte
 *  @param[out] cmd             Buffer where the command is created
 *  @param[in]  cmdSize         On entry the size of the cmd buffer, on exit the size of the command
 *  @param[in]  reply           Where the response to the PASS command is routed
 *  @param[out] cmdDest         Value (@ref cmdTxDest)
 *  @retval                     Value (@ref ReturnValues)
 *  @pre                        A driver instance must be created and tables initialized
 *
 *  @note There is up to @ref pa_MAX_CUSTOM_TYPES_LUT2 LUT2 custom types supported by PASS.
 */
paReturn_t Pa_setCustomLUT2 ( Pa_Handle       iHandle,
                              uint16_t        custIndex,
                              uint16_t        handleLink,
                              uint16_t        custHdrSize,
                              uint16_t        byteOffsets[pa_NUM_BYTES_CUSTOM_LUT2],
                              uint8_t         byteMasks[pa_NUM_BYTES_CUSTOM_LUT2],
                              uint8_t         setMask,
                              paCmd_t         cmd,
                              uint16_t       *cmdSize,
                              paCmdReply_t   *reply,
                              int            *cmdDest );
                            
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_addCustomLUT2 adds a custom lookup to the LUT2 lookup tables
 * 
 *  @details  This command is called to add a specific entry to the L4 (LUT2) lookup table. This
 *            function is called once per desired custom LUT2 match criteria.
 *            This API also initiates the atomic queue diversion operation, which means that the QMSS moves 
 *            the entries in the diverted queue to the destination queue, if the divertQ is specified and 
 *            fReplace flag is set. In this case, the PASS will complete the LUT2 update, wait for the queue 
 *            diversion to be complete and then resume processing incoming packets.
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the @ref Pa_forwardResult API.
 *
 *  @param[in]  iHandle      The driver instance handle
 *  @param[in]  custIndex    The level 4 (LUT2) custom index
 *  @param[in]  match        The four match values, only 1st three valid if prevLink is non-NULL
 *  @param[in]  prevLink     An optional L2 or L3 handle that links to this lookup
 *  @param[in]  divertQ      The source queue for atomic queue diversion with LUT2 update
 *                           Set to pa_PARAMS_NOT_SPECIFIED if not specified
 *  @param[in]  fReplace     Flag to indicate whether the entry exists
 *  @param[in]  routeInfo    Where to send a packet that matches
 *  @param[out] retHandle    The returned L4 handle
 *  @param[out] cmd          The buffer where the command is created
 *  @param[in]  cmdSize      On entry the size of the cmd buffer, on exit the size of the command
 *  @param[in]  reply        Where the response to the PASS command is routed
 *  @param[out] cmdDest      Value (@ref cmdTxDest)
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 */
paReturn_t Pa_addCustomLUT2 ( Pa_Handle       iHandle,
                              uint16_t        custIndex,
                              uint8_t         match[pa_NUM_BYTES_CUSTOM_LUT2],
                              paHandleL2L3_t  prevLink,
                              uint16_t        fReplace,
                              uint16_t        divertQ,
                              paRouteInfo_t  *routeInfo,
                              paHandleL4_t    retHandle,
                              paCmd_t         cmd,
                              uint16_t       *cmdSize,
                              paCmdReply_t   *reply,
                              int            *cmdDest );
                            
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_forwardResult examines the reply of the sub-system to a command
 *
 *  @details  This command is used to pass the sub-system generated replies to commands back to
 *            this module. Functions @ref Pa_addMac, @ref Pa_addSrio, @ref Pa_addCustomLUT1 and 
 *            @ref Pa_addIp generate replies that must be
 *            forwarded to this module, or else handle deletion and link are not possible. Other
 *            commands generate replies that can be sent to this module which will return any
 *            warnings detected in the sub-system.
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    vresult    The command reply packet from the sub-system
 *  @param[out]   retHandle  Returns the handle associated with the command
 *  @param[out]   handleType Value @ref HandleTypes
 *  @param[out]   cmdDest    Value (@ref cmdTxDest)
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 */
paReturn_t Pa_forwardResult (Pa_Handle iHandle, void *vresult, paEntryHandle_t *retHandle, int *handleType, int *cmdDest);


/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configExceptionRoute configures the routing of packets based on a exception condition such as
 *          MAC briadcast, multicast or error packet
 *
 *  @details  This function is used to configure the sub-system to route packets that satisfy an exception
 *            rule or condition (see @ref ErouteTypes). For example,
 *            - failure to table match
 *            - parsing error i.e. the sub-system is not able to continuethe parse
 *            - MAC broadcast packets
 *            - IP multicast packets  
 *
 *            From one to @ref pa_EROUTE_MAX routes can be specified through a single call to this
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
 *  @param[in]    routeTypes  Array of exception routing types (@ref ErouteTypes)
 *  @param[in]    eRoutes     Array of exception packet routing configuration
 *  @param[out]   cmd         Buffer where the sub-system command is created
 *  @param[in]    cmdSize     The size of the passCmd buffer
 *  @param[in]    reply       Where the response to the PASS command is routed
 *  @param[out]   cmdDest     Value (@ref cmdTxDest)
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 */
paReturn_t Pa_configExceptionRoute (Pa_Handle       iHandle,
                                    int             nRoute,
                                    int            *routeTypes,
                                    paRouteInfo_t  *eRoutes,
                                    paCmd_t         cmd,
                                    uint16_t       *cmdSize,
                                    paCmdReply_t   *reply,
                                    int            *cmdDest);
                                    
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configExceptionRoute2 configures the routing of packets based on a exception condition such as
 *          MAC briadcast, multicast or error packet
 *
 *  @details  Pa_configExceptionRoute2 is the next generation of API to replace @ref Pa_configExceptionRoute 
 *            eventually. This new API covers the entire functionality of Pa_configExceptionRoute and it is 
 *            designed to support more features with the more advanced routing information data structure
 *            while maintain backward-compatibility over time.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[in]    nRoute      The number of exception routes specified
 *  @param[in]    routeTypes  Array of exception routing types (@ref ErouteTypes)
 *  @param[in]    eRoutes     Array of exception packet routing configuration
 *  @param[out]   cmd         Buffer where the sub-system command is created
 *  @param[in]    cmdSize     The size of the passCmd buffer
 *  @param[in]    reply       Where the response to the PASS command is routed
 *  @param[out]   cmdDest     Value (@ref cmdTxDest)
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 */
paReturn_t Pa_configExceptionRoute2 (Pa_Handle       iHandle,
                                     int             nRoute,
                                     int            *routeTypes,
                                     paRouteInfo2_t *eRoutes,
                                     paCmd_t         cmd,
                                     uint16_t       *cmdSize,
                                     paCmdReply_t   *reply,
                                     int            *cmdDest);
                                 
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configCmdSet configures the command set which consists of a list of commands
 *
 *  @details  This function is used to configure the sub-system to format and store a list 
 *            of commands which are executed in order when a match occurs and the command set is 
 *            specified by the routing information.
 *
 *            The command set is created and refered to based on the command set index.  
 *            Once the command set is created through a call to this function it remains effective 
 *            until the function is called again to explicitly overwrite its content. It is not 
 *            recommended to update a command set when it is still used by one or more packet 
 *            routes.  
 *            There are @ref pa_MAX_CMD_SETS of command sets supported by the sub-system
 *
 *            The commands within the command set will be executed in order at PDSP4. The module user is 
 *            responsible for placing the commands in such ways that the packet offsets required by commands should
 *            be in ascending order, otherwise, the unexecutable command will be ignored. The command set
 *            should be terminated with a pa_CMD_NEXT_ROUTE or pa_CMD_MULTI_ROUTE command. If there is
 *            no final route command specified, the PASS will use the default next route command. Please note
 *            that all the commands following the pa_CMD_NEXT_ROUTE or pa_CMD_MULTI_ROUTE command will be ignored.  
 *            
 *            This API supports the following commands (@ref paCmdCode)
 *            @li  pa_CMD_REMOVE_HEADER
 *            @li  pa_CMD_COPY_DATA_TO_PSINFO 
 *            @li  pa_CMD_CRC_OP
 *            @li  pa_CMD_PATCH_DATA
 *            @li  pa_CMD_REMOVE_TAIL
 *            @li  pa_CMD_NEXT_ROUTE
 *            @li  pa_CMD_MULTI_ROUTE
 *            @li  pa_CMD_USR_STATS
 *            @li  pa_CMD_VERIFY_PKT_ERROR
 *            @li  pa_CMD_SPLIT 
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the API @ref Pa_forwardResult.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[in]    index       The command set index
 *  @param[in]    nCmd        The number of commands specified
 *  @param[in]    cmdInfo     Array of command configuration information
 *  @param[out]   cmd         Buffer where the sub-system command is created
 *  @param[in]    cmdSize     The size of the passCmd buffer
 *  @param[in]    reply       Where the response to the PASS command is routed
 *  @param[out]   cmdDest     Value (@ref cmdTxDest)
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 */
paReturn_t Pa_configCmdSet (Pa_Handle       iHandle,
                            uint16_t        index,
                            int             nCmd,
                            paCmdInfo_t    *cmdInfo,
                            paCmd_t         cmd,
                            uint16_t       *cmdSize,
                            paCmdReply_t   *reply,
                            int            *cmdDest);
                            
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configMultiRouteSet configures the multi-route group which consists of packet multi-route
 *          entries
 *
 *  @details  This function is used to configure the sub-system to format and store a multi- 
 *            route set which contains routing information for up to @ref pa_MAX_MULTI_ROUTE_ENTRIES 
 *            destinations.
 *
 *            The multi-route group is created and refered to based on the multi-route index.  
 *            Once the multi-route group is created through a call to this function it remains effective 
 *            until the function is called again to explicitly overwrite its content. It is not 
 *            recommended to update a mult-route group when it is still used by one or more packet 
 *            routes.  
 *
 *            There are @ref pa_MAX_MULTI_ROUTE_SETS of multi-route sets supported by the sub-system
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the API @ref Pa_forwardResult.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[in]    mode        The operation mode (CONFIG or RESET) refer to @ref paMultiRouteModes_e
 *  @param[in]    index       The multi-route index
 *  @param[in]    nRoute      The number of routing entries specified
 *  @param[in]    routeEntry  Array of routing configuration information
 *  @param[out]   cmd         Buffer where the sub-system command is created
 *  @param[in]    cmdSize     The size of the passCmd buffer
 *  @param[in]    reply       Where the response to the PASS command is routed
 *  @param[out]   cmdDest     Value (@ref cmdTxDest)
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 */
paReturn_t Pa_configMultiRoute (Pa_Handle               iHandle,
                                paMultiRouteModes_e     mode,
                                uint16_t                index,
                                uint16_t                nRoute,
                                paMultiRouteEntry_t    *routeEntry,
                                paCmd_t                 cmd,
                                uint16_t               *cmdSize,
                                paCmdReply_t           *reply,
                                int                    *cmdDest);
                                   
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configCrcEngine configures the specified CRC engine
 *
 *  @details  This function is used to configure the specified CRC engine by formating the 
 *            CRC configuration command packet. 
 *
 *            There are multiple CRC engines in the PA sun-system. Each CRC engine is connected to its 
 *            corresponding PDSP and its location is defined at @ref paCrcInst. It performs CRC operation 
 *            required by the some network protocol such as SCTP and/or the user-specified CRC command 
 *            for its corresponding PDSP. The CRC engine is referred by the CRC instance number.
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the @ref Pa_forwardResult API.
 *
 *  @note     Each CRC engine only supports one type of CRC per configuration.
 *            It is up to the module user to configure and use the CRC engine by calling this function
 *            for the specific use cases. For example, the CRC engine (pa_CRC_INST_4_0), which resides 
 *            between Ingress4 CDE0 and CED1, should be configured to perform CRC-32c checksum for 
 *            SCTP over inner-IP use case.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[in]    index       The CRC engine index
 *  @param[in]    cfgInfo     The CRC engine configuration information
 *  @param[out]   cmd         Buffer where the sub-system command is created
 *  @param[in]    cmdSize     The size of the passCmd buffer
 *  @param[in]    reply       Where the response to the PASS command is routed
 *  @param[out]   cmdDest     Value (@ref cmdTxDest)
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 */
paReturn_t Pa_configCrcEngine (Pa_Handle       iHandle,
                               uint16_t        index,
                               paCrcConfig_t  *cfgInfo,
                               paCmd_t         cmd,
                               uint16_t       *cmdSize,
                               paCmdReply_t   *reply,
                               int            *cmdDest);
                               
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configUsrStats configures the user-defined statistics operation
 *
 *  @details  This function performs the counter configuration for the multi-level hierarchical user-defined 
 *            statistics. Each counter can be linked to the next level counter. All counters in its linking 
 *            chain will be incremented when the lowest level counter is updated. The module user can specify 
 *            the type of each counter and how the counter is linked to the next level counter. 
 *            It is not recommended to re-configure the user-defined statistics when one or more counters are 
 *            still used by PASS. The command reply routing is optional because this command is always 
 *            processed by the PA sub-system.  
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the @ref Pa_forwardResult API.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[in]    cfgInfo     The user-defined statistics configuration information
 *  @param[out]   cmd         Buffer where the sub-system command is created
 *  @param[in]    cmdSize     The size of the passCmd buffer
 *  @param[in]    reply       Where the response to the PASS command is routed
 *  @param[out]   cmdDest     Value (@ref cmdTxDest)
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 */
paReturn_t Pa_configUsrStats (Pa_Handle                iHandle,
                              paUsrStatsConfigInfo_t  *cfgInfo,
                              paCmd_t                  cmd,
                              uint16_t                *cmdSize,
                              paCmdReply_t            *reply,
                              int                     *cmdDest);
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_configTimestamp configures the PA timer which is used to generate 48-bit timestamp
 *
 *  @details  This function is used to configure the 16-bit timer reserved for the 48-bit system 
 *            timestamp. The lower 32-bit of the system timestamp will be inserted into the timestamp 
 *            field in the packet descriptor for all input packets. It can be also inserted into 
 *            the timestamp report packets triggered by the egress packets per tx command.
 *            The 16-bit timer connected to Ingress0 PDSP0 is reserved for timestamp generation.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[in]    cfgInfo     The timestamp configuration information
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 *
 */
paReturn_t Pa_configTimestamp (Pa_Handle            iHandle,
                               paTimestampConfig_t  *cfgInfo);
                               
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_getTimestamp returns the 64-bit system timestamp
 *
 *  @details  This function is called to retrieve the current value of 64-bit PASS system timestamp.
 *
 *  @param[in]    iHandle     The driver instance handle
 *  @param[out]   pTimestamp  Pointer to the 64-bit timestamp
 *  @retval                   Value (@ref ReturnValues)
 *  @pre                      A driver instance must be created and tables initialized
 *
 */
paReturn_t Pa_getTimestamp (Pa_Handle            iHandle,
                            paTimestamp_t        *pTimestamp);
                               

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_requestStats requests sub-system statistics (PASS Gen1 only)
 *
 *  @details  This function is used to request the operating statistics from the sub-system. 
 *            The statistics can be optionally cleared after reading through the doClear parameter.
 *            The statistics apply to the entire sub-system, and are not core dependent on multi-core
 *            devices.
 *
 *            On return the command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. The sub-system will generate a reply and this reply
 *            must be sent back to this module through the API @ref Pa_formatStatsReply.
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    doClear    If TRUE then stats are cleared after being read
 *  @param[out]   cmd        Buffer where the sub-system command is created
 *  @param[in]    cmdSize    The size of the cmd buffer
 *  @param[in]    reply      Where the response of the PASS command is routed
 *  @param[out]   cmdDest    Value (@ref cmdTxDest)
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 *  
 *  @note: This API is not supported at the second generation PASS
 */
paReturn_t Pa_requestStats (Pa_Handle      iHandle,
                            uint16_t       doClear, 
                            paCmd_t        cmd, 
                            uint16_t      *cmdSize, 
                            paCmdReply_t  *reply, 
                            int           *cmdDest);
                            
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_querySysStats requests sub-system statistics (PASS Gen2)
 *
 *  @details  This function is used to query the operating statistics from the sub-system. 
 *            The statistics can be optionally cleared after reading through the doClear parameter.
 *            The statistics apply to the entire sub-system, and are not core dependent on multi-core
 *            devices.
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    doClear    If TRUE then stats are cleared after being read
 *  @param[out]   pSysStats  Pointer to the sysStats buffer
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 *
 *  @note: This API is not supported at the first generation PASS
 */
paReturn_t Pa_querySysStats (Pa_Handle      iHandle,
                             uint16_t       doClear, 
                             paSysStats_t  *pSysStats);

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_getDbgpInfo provides the snap shot of internal information for debug purpose only
 *
 *  @details  This function is used to get the debug information from the sub-system
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[out]   dbgInfo    Pointer to the debug info buffer
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 *
 *  @note: This API provides the snap shot information only, the actual values may differ
 *         after the snap shot
 */
paReturn_t Pa_getDbgpInfo(Pa_Handle iHandle, paSnapShotDebugInfo_t *dbgInfo);


/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_getVirtualLinkId provides the hooks to get the virtual link id
 *
 *  @details  This function is used to get the virtual link ID information from the handle
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    vlinkHdl   Pointer to the virtual link handle
 *  @param[out]   lnkId      virtual link ID from the virtual link handle
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 *
 *  @note: This API provides the snap shot information only, the actual values may differ
 *         after the snap shot
 */
paReturn_t Pa_getVirtualLinkId(Pa_Handle iHandle, paLnkHandle_t vlinkHdl, int8_t* lnkId);

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_formatStatsReply formats the stats reply from the PA (PASS Gen1 only)
 *
 *  @details  This function is used to convert the stats from the sub-system into a format
 *            useful for the application
 *
 *  @param[in]    handle    The driver instance handle
 *  @param[in]    cmd       The buffer returned with the request stats response from PA
 *  @retval                 A pointer to the formatted stats
 *  @pre                    A call to @ref Pa_requestStats with output sent to PA and a 
 *                          reply generated from PA.
 *
 *  @note: This API is not supported at the second generation PASS
 *  
 */
paSysStats_t* Pa_formatStatsReply (Pa_Handle    handle,
                                   paCmd_t      cmd);
                                   
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_requestUsrStats requests user-defined statistics
 *
 *  @details  This function is used to request the user-defined statistics from the sub-system. 
 *            If the buffer pointer (pUsrStats) is provided, the statistics will be formatted and 
 *            copied to the buffer. However, the statistics will not be autonomous because some 
 *            related statistics may be updated by the PASS while LLD is reading other statistics.
 *            To request autonomous statistics query, set the buffer pointer (pUsrStats) to NULL and 
 *            LLD will generate the statistics request command packet to be delivered to PASS regardless
 *            of doClear setting. 
 *
 *            The sub-system statistics can be optionally cleared after query if doClear is set whether 
 *            or not the buffer pointer is provided. 
 *            The command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. 
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    doClear    If TRUE then stats are cleared after being read
 *  @param[out]   cmd        Buffer where the sub-system command is created
 *  @param[in]    cmdSize    The size of the cmd buffer
 *  @param[in]    reply      Where the response of the PASS command is routed
 *  @param[out]   cmdDest    Value (@ref cmdTxDest)
 *  @param[out]   pUsrStats  Pointer to the usrStats buffer
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 *
 *  @note This API may be depreciated in the future releases since it can be replaced by API @ref 
 *        Pa_requestUsrStatsList
 */
paReturn_t Pa_requestUsrStats (Pa_Handle      iHandle,
                               uint16_t       doClear, 
                               paCmd_t        cmd, 
                               uint16_t      *cmdSize, 
                               paCmdReply_t  *reply, 
                               int           *cmdDest,
                               paUsrStats_t  *pUsrStats);
                               
/**
 *  @ingroup salld_api_constants
 *  @{
 *  @brief  Indicate that the complete set of user-defined statistics should be leared
 */
#define pa_USR_STATS_CLEAR_ALL    0    /**< This constant indicates that all user-defined statistics should be cleared  */ 

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_requestUsrStatsList is an advanced version of API @ref Pa_requestUsrStats. It requests user-defined 
 *          statistics with option to clear entire or a subset of statistics.
 *
 *  @details  This function is used to request the user-defined statistics from the sub-system 
 *            with option to clear entire or a subset of statistics specified by the list of 
 *            counters. 
 *            If the buffer pointer (pUsrStats) is provided, the statistics will be formatted and 
 *            copied to the buffer. However, the statistics will not be autonomous because some 
 *            related statistics may be updated by the PASS while LLD is reading other statistics.
 *            To request autonomous statistics query, set the buffer pointer (pUsrStats) to NULL and 
 *            LLD will generate the statistics request command packet to be delivered to PASS regardless
 *            of doClear setting. 
 *
 *            The sub-system statistics can be optionally cleared after query if doClear is set. In
 *            this case the formatted command packet will include the list of counters to be cleared. 
 *            The command buffer (cmd) contains a formatted command for the sub-system. 
 *            The destination for the command is provided in cmdDest. The module user must send the
 *            formatted command to the sub-system. 
 *
 *  @note: This function always returns the entire user-defined statistics and it is up to the caller to pick
 *         up the interesting ones. 
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    doClear    If TRUE then stats are cleared after being read
 *  @param[in]    nCnt       The number of counters to be cleared 
 *  @param[in]    cntIndex   Array of counter indexes to be cleared
 *  @param[out]   cmd        Buffer where the sub-system command is created
 *  @param[in]    cmdSize    The size of the cmd buffer
 *  @param[in]    reply      Where the response of the PASS command is routed
 *  @param[out]   cmdDest    Value (@ref cmdTxDest)
 *  @param[out]   pUsrStats  Pointer to the usrStats buffer
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 */
paReturn_t Pa_requestUsrStatsList (Pa_Handle      iHandle,
                                   uint16_t       doClear,
                                   uint16_t       nCnt,
                                   uint16_t      *cntIndex, 
                                   paCmd_t        cmd, 
                                   uint16_t      *cmdSize, 
                                   paCmdReply_t  *reply, 
                                   int           *cmdDest,
                                   paUsrStats_t  *pUsrStats);

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_formatUsrStatsReply formats the user-defined statistics reply from the PASS
 *
 *  @details  This function is used to convert the stats from the sub-system into a format
 *            useful for the application
 *
 *  @param[in]    handle    The driver instance handle
 *  @param[in]    cmd       The buffer returned with the request stats response from PA
 *  @param[out]   pUsrStats Pointer to the usrStats buffer
 *  @retval                 Value (@ref ReturnValues)
 *  @pre                    A call to @ref Pa_requestUsrStats or Pa_requestUsrStatsList with
 *                          buffer pointer pUsrStats set to NULL and output sent to PA and a 
 *                          reply generated from PA.
 */
paReturn_t Pa_formatUsrStatsReply (Pa_Handle      handle,
                                   paCmd_t        cmd,
                                   paUsrStats_t  *pUsrStats);

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_queryRaStats queries RA statistics (PASS Gen2 only)
 *
 *  @details  This function is used to query the RA statistics from the sub-system. 
 *            The statistics will be formatted and copied to the buffer provided.
 *            The sub-system statistics can be then optionally cleared if doClear is set. 
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    doClear    If TRUE then stats are cleared after being read
 *  @param[out]   pRaStats   Pointer to the raStats buffer
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 *
 *  @note: This API is not supported at the first generation PASS
 */
paReturn_t Pa_queryRaStats (Pa_Handle     iHandle,
                            uint16_t      doClear, 
                            paRaStats_t  *pRaStats);
                              
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_queryAclStats queries ACL per-entry statistics (PASS Gen2 only)
 *
 *  @details  This function is used to query the ACL per-entry statistics. 
 *            The statistics can be optionally cleared after reading through the doClear parameter.
 *
 *  @param[in]    iHandle    The driver instance handle
 *  @param[in]    aclHandle  The ACL handle
 *  @param[in]    doClear    If TRUE then stats are cleared after being read
 *  @param[out]   pAclStats  Pointer to the aclStats buffer
 *  @retval                  Value (@ref ReturnValues)
 *  @pre                     A driver instance must be created and tables initialized
 *
 *  @note: This API is not supported at the first generation PASS
 */
paReturn_t Pa_queryAclStats (Pa_Handle      iHandle,
                             paHandleAcl_t  aclHandle,
                             uint16_t       doClear, 
                             paAclStats_t  *pAclStats);

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_formatTxRoute formats the commands to add checksums and route a Tx packet
 *
 *  @details  This function is used to create the command block which is used by the packet accelerator
 *            sub-system to forward the packet with optional checksum generation.
 *            The module user can combine this block with other command blocks that control the security 
 *            accelerator. The combined block is then provided for the transmitted packets in the Protocol 
 *            specific section of the packet descriptor. This API needs only to be called once, and the same 
 *            protocol specific section can be used for every packet in the channel. If the length of the 
 *            checksum area changes with each packet, update the command buffer with the macro 
 *            PASS_SET_TX_CHKSUM_LENGTH()
 *
 *  @note     The Tx commands can be executed at either PDSP4 or PDSP5. However, it is highly
 *            recommended to use PDSP5 for load balance since PDSP4 will be used to execute 
 *            multi-routing and from-network command set.
 *
 *  @param[in]   chk0       Checksum 0 configuration. NULL if no checksum computation required
 *  @param[in]   chk1       Checksum 1 configuration. NULL if no checksum computation required
 *  @param[in]   route      Next packet routing from sub-system
 *  @param[out]  cmdBuffer  The routing command is formed in this buffer
 *  @param[in]   cmdSize    On entry the size of cmdBuffer. On exit the size of the command
 *  @retval                 Value (@ref ReturnValues)
 */
paReturn_t Pa_formatTxRoute  (paTxChksum_t  *chk0,
                              paTxChksum_t  *chk1,
                              paRouteInfo_t *route,
                              void          *cmdBuffer,
                              uint16_t      *cmdSize );
                             
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_formatRoutePatch formats the commands to route a packet and blind patch
 *
 *  @details  This function is used to create the command block which is used by the packet accelerator
 *            sub-system to perform blind patches on the packet. This function user optionally combines
 *            the generated block with other blocks to create compound commands. The command blocks are
 *            attached to data packets in the Protocol specific section of the packet descriptor.
 *
 *  @note     The Tx commands can be executed at either PDSP4 or PDSP5. However, it is highly
 *            recommended to use PDSP5 for load balance since PDSP4 will be used to execute 
 *            multi-routing and from-network command set.
 *
 *  @param[in]   route      Specifies where the packet is sent after the patch is complete
 *  @param[in]   patch      The patch information
 *  @param[out]  cmdBuffer  The routing command is formed in this buffer
 *  @param[in]   cmdSize    On entry this size of cmdBuffer. On exit the amound of cmdBuffer used
 *  @retval                 Value (@ref ReturnValues)
 */
                             
paReturn_t Pa_formatRoutePatch (paRouteInfo_t *route,
                                paPatchInfo_t *patch,
                                void          *cmdBuffer,
                                uint16_t      *cmdSize);
                                
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_formatTxCmd formats a list of commands to be executed on the packets to be transmitted
 *          over the network
 *
 *  @details  This function is used to create, append and update the list of commands which will be 
 *            executed by the packet accelerator and security accelerator sub-systems to perform a sequence 
 *            of actions on the packet. The command block should be attached to data packets in the 
 *            protocol specific section of the packet descriptor. 
 *
 *            This API may be called multiple times to add or update the command block.
 *            The same protocol specific section can be used for every packet in the channel after the
 *            command list is constructed. Multiple MACROs may be used to update some parameters
 *            such as packet length in the command buffer for each packet. 
 *
 *            This API supports the following commands (@ref paCmdCode):
 *            @li pa_CMD_NEXT_ROUTE
 *            @li pa_CMD_CRC_OP
 *            @li pa_CMD_PATCH_DATA
 *            @li pa_CMD_TX_CHECKSUM
 *            @li pa_CMD_REPORT_TX_TIMESTAMP
 *            @li pa_CMD_SA_PAYLOAD
 *            @li pa_CMD_IP_FRAGMENT
 *            @li pa_CMD_PATCH_MSG_LEN
 *
 *  @note     The Tx commands can be executed at either PDSP4 or PDSP5. However, it is highly
 *            recommended to use PDSP5 for load balance since PDSP4 will be used to execute 
 *            multi-routing and from-network command set.
 *
 *  @param[in]   nCmd       The number of commands specified
 *  @param[in]   cmdInfo    Array of command configuration information
 *  @param[in]   offset     The command buffer location where the new commands are inserted
 *  @param[out]  cmdBuffer  Buffer where the sub-system command is created
 *  @param[in]   cmdSize    On entry this size of cmdBuffer. On exit the amound of cmdBuffer used
 *  @retval                 Value (@ref ReturnValues)
 *
 *  @note The command buffer should be 4-byte aligned
 */
                             
paReturn_t Pa_formatTxCmd (int             nCmd,
                           paCmdInfo_t    *cmdInfo,
                           uint16_t        offset,
                           void           *cmdBuffer,
                           uint16_t       *cmdSize);
                             
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_resetControl controls the reset state of the Sub-system
 *
 *  @details  This function is used to assert or release reset for the sub-system. Asserting reset does not
 *            reset any of the sub-system tables (L2, L3 or L4, see @ref netlayers), but only the packet
 *            processing modules. To achieve a complete system reset the system level reset must be asserted
 *            through the power controller.
 *
 *  @param[in]   iHandle    The driver instance handle
 *  @param[in]   newState   Value @ref paSubSysStates
 *  @retval                 Value @ref paSubSysStates
 *  @pre                    None
 *
 *  @note This function will access the PA sub-system registers. It is up to the module user to provide critical
 *        section protection so that only one core or task should use this function at a time.
 */
paSSstate_t Pa_resetControl (Pa_Handle iHandle, paSSstate_t newState);
                             
                             
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_downloadImage downloads a PDSP image to a sub-system with the packet processing modules in reset.
 *
 *  @details This function is used to download an executable PDSP image to the specific packet processing module.
 *           See section table @ref appendix1 for a description of PDSP images provided by this module
 *
 *  @param[in]   iHandle   The driver instance handle
 *  @param[in]   modId     The PDSP number (0-5)
 *  @param[in]   image     The image to download
 *  @param[in]   sizeBytes The size of the image
 *  @retval                Value (@ref ReturnValues)
 *  @pre                   The packet processing modules must be in reset. See @ref Pa_resetControl.
 *
 *  @note This function will access the PA sub-system registers. It is up to the module user to provide critical
 *        section protection so that only one core or task should use this function at a time.
 */
paReturn_t Pa_downloadImage (Pa_Handle iHandle, int modId, void* image, int sizeBytes);

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_getHandleRefCount returns the number of reference channels linked to the LUT1 handle
 *
 *  @details  The LLD maintains the reference counter for LUT1 handles: MAC/IP. Given a handle, 
 *            the LLD would return how many references are being used in next header entry by invoking
 *            the function. For example, Query on MAC handle need to return how many IP handles are 
 *            referencing the MAC handles. Query on IP handle need to return how many next protocol 
 *            entries: IP/UDP are referencing to the IP handle.
 *            Therefore this function can be used to verify whether the LUT1 entry associated with
 *            the handle can be reomved.
 *
 *  @param[in]   iHandle    The driver instance handle
 *  @param[in]   l2l3handle The L2 or L3 handle to be queryed
 *  @param[out]  refCount   The number of reference channels
 *  @retval                 Value (@ref ReturnValues)
 */
paReturn_t Pa_getHandleRefCount ( Pa_Handle       iHandle,
                                  paHandleL2L3_t  l2l3handle,
                                  uint16_t        *refCount );
                                  
/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_getPDSPVersion returns the PA PDSP version information.
 *
 *  @details This function is used to get the PA PDSP version information in 0xAABBCCDD format.
 *           where Arch (AA); API Changes (BB); Major (CC); Minor (DD
 *
 *  @param[in]   iHandle   The driver instance handle
 *  @param[in]   modId     The PDSP number (0-5)
 *  @param[out]  pVersion  The pointer to PDSP version number
 *  @retval                Value (@ref ReturnValues)
 *  @pre                   The PDSP image should be downloaded successfully.
 *
 */
paReturn_t Pa_getPDSPVersion (Pa_Handle iHandle, int modId, uint32_t *pVersion);


/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_getVersion returns the PA LLD version information
 *
 *  @details This function is used to get the version information of the PA LLD in 0xAABBCCDD format.
 *           where Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 *
 *  @retval                32-bit version information
 */
uint32_t Pa_getVersion (void);


/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_getVersionStr returns the PA LLD version string
 *
 *  @details This function is used to get the version string of the PA LLD.
 *
 *  @retval                Version string
 */
const char* Pa_getVersionStr (void);

/**
 *  @ingroup palld_api_functions
 *  @brief  Pa_getLUT1Info returns the LUT1 information.
 *
 *  @details This function is used to get the lut1 information associated with the L2L3handle
 *
 *  @param[in]   iHandle    The driver instance handle
 *  @param[in]   l2l3handle LLD l2l3handle
 *  @param[out]  lut1Info   The pointer to lut1Information structure
 *  @retval                 Value (@ref ReturnValues)
 *  @pre                    The PDSP image should be downloaded successfully.
 *
 */
paReturn_t Pa_getLUT1Info ( Pa_Handle       iHandle,
                            paHandleL2L3_t  l2l3handle,
                            paLUT1Info_t   *lut1Info);


/**
 *  @ingroup palld_api_macros
 *  @brief  pa_RESET_SUBSYSTEM is used to reset the Sub-system
 *
 *  @details  This macro is used to put the packet processing sub-system into reset. It performs the same function
 *            as @ref Pa_resetControl, but in macro form. The module user must define the macro SYSTEM_WRITE32.
 *
 *  @pre      The module user must define a macro called SYSTEM_WRITE32(address, value) which writes a 32 bit
 *            value (value) to global address (address).
 *            
 */
#define pa_RESET_SUBSYSTEM()      \
{                                 \
      CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)CSL_NETCP_CFG_REGS;    \
                                                                        \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[0].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_SOFT_RST_N_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[1].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_SOFT_RST_N_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[2].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_SOFT_RST_N_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[3].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_SOFT_RST_N_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[4].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_SOFT_RST_N_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[5].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_SOFT_RST_N_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PKT_ID.PKT_ID_SOFT_RESET, 1);    \
      SYSTEM_WRITE32(&(passRegs->STATS.STATS_SOFT_RESET, 1);    \
      SYSTEM_WRITE32(&(passRegs->PDSP_TIMER[0].TIMER_CNTRL_REG,   0);    \
      SYSTEM_WRITE32(&(passRegs->PDSP_TIMER[1].TIMER_CNTRL_REG,   0);    \
      SYSTEM_WRITE32(&(passRegs->PDSP_TIMER[2].TIMER_CNTRL_REG,   0);    \
      SYSTEM_WRITE32(&(passRegs->PDSP_TIMER[3].TIMER_CNTRL_REG,   0);    \
      SYSTEM_WRITE32(&(passRegs->PDSP_TIMER[4].TIMER_CNTRL_REG,   0);    \
      SYSTEM_WRITE32(&(passRegs->PDSP_TIMER[5].TIMER_CNTRL_REG,   0);    \
}      
      
/**
 *  @ingroup palld_api_macros
 *  @brief  pa_ENABLE_SUBSYSTEM enables the subsystem.
 *
 *  @details This macro is used to release reset from the packet processing sub-system. It performs the same
 *           function as @ref Pa_resetControl, but in macro from. The module user must define the macro SYSTEM_WRITE32
 *           and SYSTEM_READ32.
 *
 *  @pre     The module user must define the macro SYSTEM_WRITE32(address, value) and SYSTEM_READ32 (address) which
 *           read and write to global address (address). 
 */
#define pa_ENABLE_SUBSYSTEM()    \
{                                 \
      CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)CSL_NETCP_CFG_REGS;    \
                                                                        \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[0].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[1].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[2].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[3].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[4].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK));   \
      SYSTEM_WRITE32(&(passRegs->PDSP_CTLSTAT[5].PDSP_CONTROL), (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK));   \
      while (SYSTEM_READ32(&(passRegs->MAILBOX[0].MBOX_SLOT[0])) == 0);          \
      while (SYSTEM_READ32(&(passRegs->MAILBOX[1].MBOX_SLOT[0])) == 0);          \
      while (SYSTEM_READ32(&(passRegs->MAILBOX[2].MBOX_SLOT[0])) == 0);          \
      while (SYSTEM_READ32(&(passRegs->MAILBOX[3].MBOX_SLOT[0])) == 0);          \
      while (SYSTEM_READ32(&(passRegs->MAILBOX[4].MBOX_SLOT[0])) == 0);          \
      while (SYSTEM_READ32(&(passRegs->MAILBOX[5].MBOX_SLOT[0])) == 0);          \
      SYSTEM_WRITE32(&(passRegs->MAILBOX[0].MBOX_SLOT[1], 1);                    \
      SYSTEM_WRITE32(&(passRegs->MAILBOX[0].MBOX_SLOT[0], 0);                    \
      while (SYSTEM_READ32(&(passRegs->MAILBOX[0].MBOX_SLOT[1])) == 1);          \
      SYSTEM_WRITE32(&(passRegs->MAILBOX[0].MBOX_SLOT[1], 0);                    \
      SYSTEM_WRITE32(&(passRegs->MAILBOX[0].MBOX_SLOT[2], 0);                    \
      SYSTEM_WRITE32(&(passRegs->MAILBOX[0].MBOX_SLOT[3], 0);                    \
      SYSTEM_WRITE32(&(passRegs->MAILBOX[0].MBOX_SLOT[4], 0);                    \
      SYSTEM_WRITE32(&(passRegs->MAILBOX[0].MBOX_SLOT[5], 0);                    \
}      
     
     
/**
 *  @ingroup palld_api_macros
 *  @brief  pa_DOWNLOAD_MODULE downloads an image
 *
 *  @details  This macro provides the same function as @ref Pa_downloadImage. A single image is downloaded to
 *            one of the packet processing modules.
 *
 *  @pre      The module user must define macro SYSTEM_COPY(dest, src, sizeWords) which copies sizeWords from
 *            address src to address dst. The packet processing module must have reset asserted.
 */
#define pa_DOWNLOAD_MODULE(id,img,size)   \
{                                 \
      CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)CSL_NETCP_CFG_REGS;    \
                                                                        \
      SYSTEM_COPY(&(passRegs->PDSP_IRAM[id].PDSP_RAM[0]), img, size);   \
}      
      
/**
 *   @ingroup palld_api_macros
 *   @brief  pa_GET_SYSETM_STATE returns the state of the subsystem
 *
 *   @details  This macro provides the same functionality as @ref Pa_resetControl and returns the 
 *             current state in the macro argument.
 */
#define pa_GET_SYSTEM_STATE(x)      \
  {  int enable=0; int disable=0;   \
      CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)CSL_NETCP_CFG_REGS;    \
     if ( (SYSTEM_READ32(&(passRegs->PDSP_CTLSTAT[0].PDSP_CONTROL)) & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK) == \
                     (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)    ) \
       enable++;  else disable++;                                                        \
     if ( (SYSTEM_READ32(&(passRegs->PDSP_CTLSTAT[1].PDSP_CONTROL)) & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK) == \
                     (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)    ) \
       enable++;  else disable++;                                                        \
     if ( (SYSTEM_READ32(&(passRegs->PDSP_CTLSTAT[2].PDSP_CONTROL)) & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK) == \
                     (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)    ) \
       enable++;  else disable++;                                                        \
     if ( (SYSTEM_READ32(&(passRegs->PDSP_CTLSTAT[3].PDSP_CONTROL)) & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK) == \
                     (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)    ) \
       enable++;  else disable++;                                                        \
     if ( (SYSTEM_READ32(&(passRegs->PDSP_CTLSTAT[4].PDSP_CONTROL)) & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK) == \
                     (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)    ) \
       enable++;  else disable++;                                                        \
     if ( (SYSTEM_READ32(&(passRegs->PDSP_CTLSTAT[5].PDSP_CONTROL)) & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK) == \
                     (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)    ) \
       enable++;  else disable++;                                                        \
     if ( (enable > 0) && (disable > 0) )  (x) = pa_STATE_INCONSISTENT;                  \
     else if (enable > 0)  (x) = pa_STATE_ENABLE;                                        \
     else (x) = pa_STATE_RESET;                                                          \
  }  (x) = (x)
  
  
/**
 *  @ingroup palld_api_macros
 *  @brief  pa_SET_TX_CHKSUM_LENGTH sets the tx checksum length in a tx route block
 *
 *  @details  This macro is used to modify the length of a checksum field in a command packet
 *            created by a call to @ref Pa_formatTxRoute. In many cases packets in an outbound packet
 *            stream will have the same source and destination information (see @ref netlayers) but
 *            differ in the packet length. This macro will change the checksum calculation information
 *            which is sent to the sub-system. The length fields in L2, L3 and L4 must be changed by
 *            the module user before sending the packet, they are not changed by this macro. In the
 *			  case of IP L3 and TCP or UDP L4 the psuedo header checksum must also be changed
 *			  to reflect the change in packet length.
 */
#define pa_SET_TX_CHKSUM_LENGTH(datap,cnum,len)  \
    PASAHO_CHKCRC_SET_LEN        ((&(((pasahoComChkCrc_t *)datap)[cnum])), len)

/**
 *  @ingroup palld_api_macros
 *  @brief pa_SET_TX_INITVAL sets the initial value in a tx route block
 *
 *  @details  This macro is used to modify the initial value of a checksum field in a command packet
 *            created by a call to @ref Pa_formatTxRoute. This macro is used when a single call
 *			  to @ref Pa_formatTxRoute is desired, The application typically follows this with an update 
 *            to the length fields in network headers, either directly or through a blind patch. 
 *            For updates with IPv4 or IPv6 the pseudo header checksum must be updated as well, and this 
 *            macro is used to update the value. Typically the pseudo header checksum will be computed 
 *            with all values except the length, and then updated for each packet with a single ones' complement add.
 */
#define pa_SET_TX_CHKSUM_INITVAL(datap,cnum,val)  \
	PASAHO_CHKCRC_SET_INITVAL   ((&(((pasahoComChkCrc_t *)datap)[cnum])), val)

/**
 * @page netlayers
 *
 *  Network layers define a hierarchy of services delineated by functionality. Each layer can use the functionality
 *  of the next layer below, and offers services to the next layer above. The packet accelerator sub-system examines
 *  and routes packets based on fields in up to three layers of the Ethernet packets or L0-L2 header of the SRIO packets. 
 *  
 *  In layer 2, the MAC (Media Access Control) layer, 
 *  the sub-system classifies IEEE 802.3 packets based on (optionally) the destination MAC, source MAC, Ethertype, and 
 *  VLAN tags. 
 *
 *  In Layer 3, the network layer, IPv4 (Internet Protocol Version 4) and IPv6 (Internet Protocol
 *  Version 6) packets are routed based (optionally) on source IP address, destination IP address, IPv4 protocol,
 *  IPv6 next header, IPv4 Type of Service (recently changed to IPv4 differentiated service in RFC 2474), IPv6
 *  traffic class, and IPv6 flow label. For IP packets with security services the SPI (Security Parameters Index)
 *  is also included in the classification information. For IP packets with SCTP (Stream Control Transmission Protocol) 
 *  the SCTP destination port is also included in the classification information.
 *
 *  In layer 4, the transport layer, UDP (User Datagram Protocol) and TCP (Transmission Control Protocol) packets
 *  are routed based on the destination port. However, the GTP-U (GPRS Tunnelling Protocol User Plane) over UDP packets 
 *  are routed based on its 32-bit TEID (Tunnel ID).   
 *
 *  For SRIO (Serial RapidIO), L0-L2 header information
 *  the sub-system classifies SRIO packets based on (optional) the source ID, destination ID, transport type, priority,
 *  message type, SRIO type 11 mailbox and letter, SRIO type 9 stream ID and class of service.
 * 
 */
 
 
/**
 * @page cache
 *
 * The packet accelerator low level driver module will make call backs to the module user when it
 * is about to read from one of the two tables provided by the module user. If the module user
 * is operating in a multi-core environment with a single set of tables shared by all the cores,
 * then this function is used to tell a local core that it must invalidate its cache, without writeback.
 * This is necessary if cross core cache coherency is not maintained by the hardware in the device.
 * 
 * Without this it is possible for one core to be operating from a locally cached version of the 
 * tables which does not reflect any additions or deletions done by other cores.
 *
 * An alternative is to place the tables into non-cached memory.
 *
 */
  
/**
 *  @page semaphores
 *
 *  The packet accelerator low level driver module will make call backs to the module user when it
 *  is about to modify from one of the two tables provided by the module user. If the module user
 *  is operating in a multi-core environment with a single set of tables shared by all the cores,
 *  then this function is used to tell the application to apply a cross core semaphore. 
 *
 *  When table modification is done the module will again make a call back to the module user
 *  to inform it to release the semaphore.
 */
   
   
/**   
 *  @page order
 *
 *  The sub-system examines the L2 and L3 (LUT1) information (see @ref netlayers) in packets based on internal 
 *  table location. When function @ref Pa_addMac and @ref Pa_addIp are executed and the resulting packet 
 *  forwarded to the sub-system, the sub-system places the new entries at the highest free
 *  table location. When incoming packets are examined, the table is searched from lowest entry location
 *  to highest entry location until the first matching entry is found. That entry is used to route the
 *  packet.
 *
 *  Because of this it is required that entries into the table be made in order from the most general
 *  to the most specific. For example, when adding a mac address it is common to want to route the following:
 *  @li  dest mac only  -  Forward packet to host
 *  @li  dest mac + ethertype - Continue parsing
 *  @li  dest mac + source mac + ethertype - Forward packet to host
 *
 *  To get the desired routing the @ref Pa_addMac commands must be executed and the command packets forwarded
 *  to the sub-system in the order shown above. If they are entered in the reverse order then every packet
 *  which has the value dest MAC will be forwarded to the host since it matches the first entry in the list.
 *
 *  The order dependency applies to calls to @ref Pa_addMac and @ref Pa_addIp, but not to calls between these functions. 
 *  So all MAC entries can be made followed by all IP entries, or in the reverse order (provided the IP entries
 *  do not reference the MAC entries) without changing the operation of the sub-system.
 *
 */
 
/**   
 *  @page appendix1 PDSP image
 *
 *  The sub-sustem contains six PDSPs wihich perform the command and packet processing. There are three PDSP
 *  images provided by the module under the pa/fw directory:
 *  @li Packet Classifier 1 image: classify1_bin.c for PDSP0, PDSP1 and PDSP2
 *  @li Packet Classifier 2 image: classify2_bin.c for PDSP3
 *  @li Packet Modifier image: pam_bin.c for PDSP4 and PDSP5
 *
 *  The PDSP executable images are provided to the module user as c-file contains the binary image. They should
 *  be included by the application and loaded into the corresponding PASS PDSP by invoking the API 
 *  @ref Pa_downloadImage at system startup.
 *
 */
 
 
 /**
  *  @page appendix2 CPPI Error Flags
  *
  *  The sub-system performs IPv4 header checksum, UDP/TCP checksum and SCTP CRC-32c checksum autonomously.
  *  The sub-system can also perform the CRC verification for incoming packet as one of the actions specified 
  *  by the post-classification command set.
  *
  *  The checksum and CRC verification results are recorded at the 4-bit error flags in the CPPI packet descriptor
  *  as described below:
  *  @li bit 3: IPv4 header checksum error
  *  @li bit 2: UDP/TCP or SCTP CRC-32c checksum error
  *  @li bit 1: Custom CRC checksum error
  *  @li bit 0: reserved
  * 
  */
  
 /**
  *  @page appendix3 PA-assisted IP Reassembly Operation
  *
  *  The current version of PASS does not support IP reassembly, the IP fragments can be detected by PASS, forwarded to 
  *  and reassembled at host. The reassembled IP packet may be forwarded back to PASS for continuous classification. 
  *  The drawback of this approach is that the order of the incoming packets will not be maintained.
  *
  *  To provide better support for IP reassembly, the PA-assisted IP Reassembly operation is introduced and summarized below:
  *  @li Array of traffic flows which consist of source IP, destination IP, protocol and counter are maintained at PASS PDSP.
  *  @li A traffic flow is activated by the PDSP when the first IP fragment with the source and destination IP and protocol is 
  *      detected and forwarded.
  *  @li The traffic flow is freed when its packet count reaches 0
  *  @li All packets belong to any active traffic flow will be forwarded to the host so the packet order will be maintained.
  *  @li IP fragments should be forwarded to host with "not availeable" traffic flow id if no traffic flow is available. 
  *      In this case, the packet order is not guaranteed to be maintained.
  *  @li PASS supports up to 32 active traffic flows for outer IP (PDSP1) and inner IP (PDSP2) respectively.
  *  @li The PA-assisted IP Reassembly Operation will be enabled by invoking API @ref Pa_control with the IP reassembly 
  *      configuration @ref paIpReassmConfig_t.
  *
  *  @note The minimum size packet wire rate throughput will not be guaranteed when this feature is enabled and there are active 
  *        traffic flows.
  *  
  *  The host IP reassembly module should interact with PASS and perform the full IP reassembly operation. The module user may choose 
  *  to implement a simplified version of IP reassembly algorithm to save CPU cycle in controlled IP environment. A sample reassembly
  *  module is provided in the PA LLD release package, which demonstrates how to interact with the NetCP to perform the IP reassembly
  *  operation. 
  *
  *  The sample code implements a simplified version of IP reassembly algorithm which supports non-overlapping segments only. The sample 
  *  code performs the following tasks:
  *  @li Maintain the IP reassembly contexts consist of source IP, destination IP, IP identification, protocol, fragments count and the 
  *      corresponding traffic flow id.
  *  @li Forward the non-fragmented IP packet with its flow id and count = 1 to PA PDSP queue. This avoids reordering the non-fragmented packets.
  *  @li For IPSEC inner IP fragments, call SA LLD to perform the post-decryption operation including padding check and IPSEC header 
  *      and authentication tag removal.
  *  @li Forward the reassembled IP packet with its flow id and fragments count to PA PDSP queue.
  *  @li Send a null packet with its flow id and fragments count to PA PDSP queue if the fragments are discarded due to timeout or other error.
  *
  */

/**
 *	@page appendix5 Port Mirror and Packet Capture Operation
 *
 *	The current version of CPSW within NetCP does not support port mirroring feature. The PA LLD and the PASS firmware have been 
 *  enhnaced to support EMAC port mirroring operation OR EMAC port packet capture feature. 
 *
 *  When Port Mirror configuration is enabled, some of the ethernet ports can be configured as mirror ports. Mirror port receives 
 *  and transmits ethernet traffic as normal and other non-mirror ports can be configured to have its traffic mirrored to any 
 *  mirror port. A port that has its traffic mirrored means that all traffic to and/or from this port can also be transmitted 
 *  (mirrored) to its mirror port. PA supports individual ingress and egress control of the EMAC port to be mirrored. The mirror port 
 *  itself can never be mirrored. It is the responsibility of the higher level software to take care of this condition to avoid 
 *  recursion and undesired effects. Packets are mirrored excatly as they are received/transmitted. No additional mac header or
 *  equivalent is placed on these packets.
 *
 *  In addition, PASS also supports the packet capture feature which is valid only if port mirroring is not in use and if it is enabled 
 *  on an interface. The feature works in a similar fashion to port mirroring except that the captured packet will be copied and sent to 
 *  a configured hardware queue instead of the mirror port.
 * 
 *  The host software should enable and configure either the port mirror or packet capture operation on that interface using @ref Pa_control API. 
 *  And global system configuration is required to enable those features system wide. 
 *
 *  @note  The design assumes that the port mirroring feature is not required when the device is operating with the CPSW switch active 
 *  (i.e., ALE bypass disabled) or in a NETCP bridge or s/w bridge mode.
 */

/**
 *	@page appendix6 Ingress Default Route Operation
 *  The feature allows the host to configure PASS to send all packets with broadcast bit set (bit 0 of 1st mac header byte) from ingress port X 
 *  to a corresponding route before or after the LUT look up. The ingress default route provides the route configurations for ingress broadcast(BC) 
 *  and multicast(MC) packets and the unicast packets that do not match L2 entries on an EMAC interface as described below.
 *  - Route BC/MC traffics prior to LUT1 lookup if configured as pre-classification route
 *  - Route unmatched BC/MC traffics from EMAC port X if configured as post-classification route
 *  - Route unmatched unicast traffic from EMAC port X if configured 
 *  - This rule precedes the exception route rule. 
 *  - These features can be globally enabled or disabled using @ref paPacketControl2Config_t configuration along with per interface configurations.
 *
 *  @note When this feature is enabled, the exception routes for multicast/broadcast/unicast packets will not be used..
 *
 */

/**
 *	@page appendix7 Enhanced QoS Mode Operation
 *  Enhanced QoS mode is an advanced priority-based routing algorithm where VLAN P-bit, IP DSCP or the EMAC port-based default priority 
 *  is used to determine the destination QoS queue and CPPI flow. This routing algorithm is required to support egress L2 shaper and is 
 *  described in details here.
 *
 *  For each EMAC interface, PASS will be configured for:
 *	- Base queue (egress only)
 *	- Base flow (egress only
 *	- DSCP_MAP[]   {one entry (= flow offset /queue offset) for each DSCP value, 64 total}
 *	- VLAN_PRIORITY_MAP[]  { one entry (= flow offset/queue offset) for each P-bit value, 8 total}
 *	- Default priority to use per the ingress interface
 *	- Routing mode:    P-bit or  DSCP
 *	- PriorityOverride: True/False
 *
 *  Routing algorithm supports two modes and is described as below:
 *
 *  - DP-bit mode:
 *    - If frame has VLAN tag, use p-bits to lookup shaper input queue # from the VLAN_PRIORITY_MAP[] for the frame's egress port
 *    - If frame is un-tagged, but is an IP packet, use the DSCP value to lookup the shaper input queue # from the DSCP_MAP[] 
 *      for the frame's egress port unless priority override is set for the egress port (see last bullet below).
 *    - If frame is un-tagged, and non-ip , then use the default priority for the frame's ingress port to look up the shaper input queue from 
 *      the egress port's VLAN_PRIORITY_MAP[]. For SOC-generated traffic, the default priority is a separate global configuration item.
 *    - If priority override is set and the packet is IP then do as in un-tagged/non-ip (above bullet).
 *  - DSCP mode:
 *    - If frame is an IP packet, use the DSCP value and the DSCP_MAP [] for the egress port as above to determine the L2 shaper queue to use
 *    - For non-ip packets, use the default priority for the frame's ingress port to look up the shaper input queue from the egress port's 
 *      VLAN_PRIORITY_MAP[]. For SOC-generated traffic, the default priority is a separate global configuration item.
 *    - Priority override setting is not applicable in this mode.
 *  
 */

/**
 *	@page appendix8 Ethernet OAM (EOAM) Mode Operation
 *  EOAM mode is a new mode of operation that includes the EOAM classification and new packet flow as described below. 
 *  The Ingress0-PDSP1 LUT1, is utilized to support Ethernet OAM (EOAM) target flow classification instead of the
 *  firewall of outer IP and Ingress3-PDSP0, is enhanced to filter both outer IP/UDP and inner IP/UDP.
 *
 *  During EOAM classification, packets are inspected for a specific target flow match based on any group 
 *  of Destination MAC address, Source MAC address, VLAN priority, VLAN ID and Ethernet PORT id. 
 *  Each target flow is associated with the corresponding "user defined statistics counter". 
 *  When the target flow match happens, a statistics update happens as per the "statistics update algorithm" described as below.
 *  Further, during the match if the message type/PDU found to be 1DM/DMM/DMR/LMM/LMR PALLD provides configurations to forward that packet to a
 *  host queue.  
 *
 *  - For IPSec transport mode,  Ingress 3 would filter the IP/UDP header
 *  - For IPSec tunnel mode,  Ingress 3 would filter the inner IP/UDP header 
 *  - For a non-cipher packet, Ingress 3 would filter the IP/UDP header 
 *  - IP over IP is NOT supported during this mode.
 *
 *  The above feature can be utilized to compute the "delay measurements" and/or "Ethernet loss measurements" on 1DM/DMM/DMR/LMM/LMR PDU types as 
 *  defined in protocols such as Y.1731 (Please refer to ITU-T specification "http://www.itu.int/rec/T-REC-Y.1731-200605-S/en" for further details on Y.1731)
 *  
 *  - PASS actions for 1DM/DMM/DMR PDU types
 *    - Send Direction
 *      - Patches the 8 bytes of 1588 formatted timestamp information to the specified field of the message as per the Tx command initiated by the upper layer and forward the packet to configured destination
 *    - Receive Direction
 *      - Checks for valid measurement to route to specified queue for this flow and puts the 64 bit PA time stamp in the PSINFO fields of the message
 *
 *  - PASS actions for LMM/LMR PDU types
 *    - Send Direction
 *      - Patches the 4 bytes of specified 32-bit user defined counter values to the specified filed of message as per the Tx command initiated by the upper layer and forward the packet to configured destination
 *    - Receive Direction
 *      - Checks for valid measurement to route to specified queue for this flow and puts the 64 bit PA time stamp in the PSINFO fields of the message
 *      - Also, provide the 4 byte receive packet counter associated with this target flow to PS_INFO fields of the message
 *
 *  - Statistics update algorithm in Rx direction
 *
 *     PA LLD along with firmware for GEN2 supports upto 8 protocols to be excluded from EOAM target flow statistics even though the match happens. The pseudo code showing the packet statistics update is as below.
 *
 * @code
 *    decision statsUpdateDecision(rxEthType, rxMEGLevel, rxPktOpcode, pktExcludeProtolist, TabMEGLevel)
 *    {
 *         if (rxEthType in pktExcludeProtolist)
 *         {
 *              return (noStatisticsUpdate);
 *         }
 *         if (rxEthType is 0x8902)
 *         {
 *             if (rxMEGLevel > TabMEGLevel)
 *             {
 *                return (needStatisticsUpdate);
 *             }
 *             else if (rxMEGLevel < TabMEGLevel)
 *             {
 *                  return (noStatisticsUpdate);
 *             }
 *             else 
 *             {
 *                if (rxPktOpcode is (Y1731_APS_OPCODE OR Y1731_OPCODE_CCM) )
 *                {
 *                  return (needStatisticsUpdate);                 
 *                }
 *                
 *                return (noStatisticsUpdate);  
 *             }
 *         }
 *     }           
 * @endcode
 *
 *  The new packet flow during EOAM mode is as described in below table.
 *  
 *   \image html packetflow.gif
 *
 */


/**
 *	@page appendix9 Destination Queue Bounce Operation
 *
 *	There is a hardware deficiency identified at the Keystone2 devices where memory consistency is not guaranteed for
 *  IO coherent A15 and PktDMA masters at some rare conditions. Therefore it is possible that the data arrival signal
 *  to the ARM (i.e., presence of a descriptor in QMSS queue) may occur prior to the data arriving properly in the
 *  ARM cache.  Thus, the ARM core may access stale data.
 *  To ensure ARM Cache consistency, one of the QMSS PDSP is enhanced to provide a DMA barrier function. Packets destined
 *  to ARM queues may be delivered first to one of the QMSS "bounce" queues serviced by this function. The QMSS PDSP  f/w
 *  will pop packets from these queues, perform the necessary barrier operation (that will cause the ARM cache to get
 *  invalidated for the descriptor and buffer locations), and then will relay the packet to the final destination queue.
 *
 *  The Destination Queue Bounce Operation is designed to enable the QMSS proxy bounce on packets PA sends to queues
 *  served by ARM user space by embedding 2 control bits into the destination queue ID to instruct the PASS firmware
 *  to re-route the packets to the specified QMSS bounce queues.
 *
 *  This operation can be enabled and configured by a global configuration message including the following parameters:
 *  - Enable/Disable
 *  - QMSS Bounce Queue IDs
 *    - DDR Queue: All PktMDA descriptors and buffers use DDR memory only
 *    - MSMC Queue: PktDMA descriptor and buffers may use MSMC memory and/or DDR memory
 *  - Default Behavior map []: Specify the default queue bounce operation for each traffic class such as Command Response
 *    and ingress QoS routing
 *  - NetCP hardware queue info
 *    - Number of NetCP queue
 *    - NetCP queue base
 *
 *  The Destination Queue Bounce operation is described below in details:
 *  - When the queue bounce feature is disabled, all PA LLD APIs work as before and the two control bits of the destination
 *    queue ID will be cleared.
 *  - When the queue bounce feature is enabled, the application may invoke the following PA Macros to specify the
 *    queue ID in PA LLD APIs:
 *     - PA_BOUNCE_QUEUE_DDR(queueId): Bounce to DDR queue
 *     - PA_BOUNCE_QUEUE_MSMC(queueId): Bounce to MSMC queue
 *     - PA_BOUNCE_QUEUE_NONE(queueId): No Bounce
 *    Or the application may allow the LLD to specify the embedded destination queue ID based on the default behavior map
 *    by invoking the following PA Macro optionally
 *     - PA_BOUNCE_QUEUE_DEFAULT(queueId): Use default behavior map by cleraing the two control bits
 *
 *  @note PA_BOUNCE_QUEUE_DEFAULT() is a no-operation, so if queueid is used as is, then default behavior rule will be applied automatically
 *  @note For any SOC h/w queues (as indicated to PA in global configuration), all bounce settings are ignored so that packets to these queues
 *        will never be bounced to the barrier function.
 */

#ifdef __cplusplus
}
#endif
  

#endif  /* _PA_H */
