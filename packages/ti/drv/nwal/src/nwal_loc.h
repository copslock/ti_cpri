/******************************************************************************
 * FILE PURPOSE:  Local data structures used within NWAL module
 ******************************************************************************
 * FILE NAME:   nwal_loc.h
 *
 * DESCRIPTION: NWAL LLD initialization routines
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2012
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
#ifndef __NWAL_LOC_H__
#define __NWAL_LOC_H__

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/drv/nwal/nwal_util.h>
#include "ti/drv/nwal/nwal_osal.h"

#ifdef NWAL_ENABLE_SA
#include <ti/drv/sa/salld.h>
#endif

/* SOC Family flavor adaptation */
#ifdef KeyStone1
#define NWAL_DISABLE_LINUX_MULTI_PROC_PA 1
#define NWAL_DISABLE_LINUX_MULTI_PROC_SA 1
#endif

//extern nwalLocContext_t*        gPLocContext;



#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*****************************************************************************
 * Minimum alignment
 *****************************************************************************/
#define NWAL_MIN_ALIGN_SIZE                8    /* 4 bytes or 32 bit */

/*****************************************************************************
 * Macro to round number of bytes to a size
 *****************************************************************************/
#define NWAL_ROUND_TO_SIZE(roundSize,ELCNT,ELSIZE)\
( ( ( ( (ELCNT) * (ELSIZE)\
        + (roundSize - 1)\
      ) / roundSize\
      * roundSize\
    ) + (ELSIZE) - 1\
  ) / (ELSIZE)\
)

/*****************************************************************************
 * UDP Protocol Definitions
 *****************************************************************************/
/* UDP byte offsets to fields */
#define NWAL_UDP_OFFSET_SRC_PORT       (0)
#define NWAL_UDP_OFFSET_DEST_PORT      (2)
#define NWAL_UDP_OFFSET_LEN            (4)
#define NWAL_UDP_OFFSET_CHKSUM         (6)

/* UDP definitions */
#define NWAL_UDP_HDR_LEN_BYTES         (4*2)
#define NWAL_GTPU_UDP_PORT              2152

/* TCP definitions */
#define NWAL_TCP_OFFSET_CHKSUM         (16)

/*****************************************************************************
 * IP Version 4 Protocol Definitions
 *****************************************************************************/
/* IPV4 byte offsets to fields */
#define NWAL_IPV4_OFFSET_VER_HLEN           (0)
#define NWAL_IPV4_OFFSET_TOS                (1)
#define NWAL_IPV4_OFFSET_LEN                (2)
#define NWAL_IPV4_OFFSET_ID                 (4)
#define NWAL_IPV4_OFFSET_FRAG               (6)
#define NWAL_IPV4_OFFSET_TTL                (8)
#define NWAL_IPV4_OFFSET_PROTO              (9)
#define NWAL_IPV4_OFFSET_HDR_CHKSUM         (10)
#define NWAL_IPV4_OFFSET_SRC_ADDR           (12)
#define NWAL_IPV4_OFFSET_DST_ADDR           (16)
/* IPv4 definitions */
#define NWAL_IPV4_HDR_LEN_BYTES           (20)

#define NWAL_IPV4_VER_HDR_LEN_VAL           0x45
#define NWAL_IPV4_ADDR_LEN                  4
#define NWAL_IPV4_UDP                       0x11
#define NWAL_IP_PROTO_IPV4                  4   /* IPv4 Next Protocol */
#define NWAL_IP_PROTO_IPV6                  41  /* IPv6 Protocol nextHdr */

/*****************************************************************************
 * IP Version 6 Protocol Definitions
 *****************************************************************************/
#define NWAL_IPV6_OFFSET_VER_CLASS_FLOW_HI      0
#define NWAL_IPV6_OFFSET_FLOW_LO                2
#define NWAL_IPV6_OFFSET_PLOAD_LEN              4
#define NWAL_IPV6_OFFSET_NEXT_HDR               6
#define NWAL_IPV6_OFFSET_HOP_LIMIT              7
#define NWAL_IPV6_OFFSET_SRC_ADDR               8
#define NWAL_IPV6_OFFSET_DST_ADDR               24

#define NWAL_IPV6_HDR_LEN_BYTES                 40
#define NWAL_IPV6_ADDR_NUM_BYTES                16

#define NWAL_IPV6_VERSION                    0x06
#define NWAL_IPV6_VER_SHIFT                  12
#define NWAL_IPV6_VER_MASK                   0x000F

#define NWAL_IPV6_TRAFFIC_CLASS_SHIFT        4
#define NWAL_IPV6_TRAFFIC_CLASS_MASK         0xFF

#define NWAL_IPV6_FLOW_LABEL_HI_SHIFT        16
#define NWAL_IPV6_FLOW_LABEL_HI_MASK         0xF0000

#define NWAL_IPV6_FLOW_LABEL_LO_SHIFT        0
#define NWAL_IPV6_FLOW_LABEL_LO_MASK         0xFFFF

#define NWAL_IPV6_LEN_SHIFT                  0
#define NWAL_IPV6_LEN_MASK                   0

/* UDP protocol for next header */
#define NWAL_IPV6_UDP                        0x11
#define NWAL_IPV6_PROTO_SHIFT                8
#define NWAL_IPV6_PROTO_MASK                 0xFF

#define NWAL_IPV6_HOP_SHIFT                  0
#define NWAL_IPV6_HOP_MASK                   0xFF

/* IP Identification value partitioning across multiple cores */
#define NWAL_IP_ID_CORE_SHIFT   10
#define NWAL_IP_ID_CORE_MASK    0xFC00

/*****************************************************************************
 * Ethernet Protocol Definitions
 *****************************************************************************/
#define NWAL_ETH_OFFSET_DEST_MAC          0
#define NWAL_ETH_OFFSET_SRC_MAC           6
#define NWAL_ETH_OFFSET_TYPE_LEN          12
#define NWAL_ETH_VLAN_TAG                 12
#define NWAL_ETH_TYPE_VLAN_TAG            0x8100
#define NWAL_ETH_TYPE_IP                  0x0800
#define NWAL_ETH_TYPE_IPv6                0x86DD
#define NWAL_ETH_MAC_ADDR_LEN             6
#define NWAL_ETH_DIX_HDR_LEN              14

#define NWAL_ETH_VLAN_PRIO_MASK             0x07
#define NWAL_ETH_VLAN_PRIO_SHIFT            13
#define NWAL_ETH_VLAN_RIF_MASK              0x01
#define NWAL_ETH_VLAN_RIF_SHIFT             12
#define NWAL_ETH_VLAN_ID_MASK               0x0FFF
#define NWAL_ETH_VLAN_HDR_SIZE              4
#define NWAL_ETH_802_3_HDR_SIZE             8

/*****************************************************************************
 * NETCP TX Queue Index
 *****************************************************************************/
/*****************************************************************************
 * IP header protocol ID
 *****************************************************************************/
#define NWAL_IP_IN_IP_PROTOCOL              0x4     /* Next Header */
#define NWAL_ESP_PROTOCOL                   0x32
#define NWAL_AH_PROTOCOL                    0x33

/*****************************************************************************
 * SA Packet Header and Tail Margin for TX packets
 *****************************************************************************/
#define NWAL_SA_PKT_HDR_MARGIN              32
#define NWAL_SA_PKT_TAIL_MARGIN             32 /* Reserved Margin for ESP Tail,
                                                * ESP Hash Need to have
                                                * sufficient room in the
                                                * payload descriptor
                                                */
#define NWAL_SA_PKT_MARGIN                  (NWAL_SA_PKT_HDR_MARGIN + \
                                             NWAL_SA_PKT_TAIL_MARGIN)

#define NWAL_DEFAULT_FLOW_SOP_OFFSET        0   /* Start of Packet offset for
                                                 * flow configuration
                                                 */
#define NWAL_DEFAULT_TAILROOM_SIZE          0   /* Size of the tailroom
                                                 */


/*****************************************************************************
 * Definitions for bitmap: NWAL_HANDLE_TYPE_ID_T
 *****************************************************************************/
#define NWAL_HANDLE_TYPE_MASK               0xC000
#define NWAL_HANDLE_TYPE_SHIFT              14
#define NWAL_HANDLE_TYPE_INST               0
#define NWAL_HANDLE_TYPE_TRANS              1



#define NWAL_HANDLE_ID_MASK                 0x3FFF
#define NWAL_HANDLE_ID_SHIFT                0

/* ID values for transactions */
#define NWAL_HANDLE_ID_TRANS_NONE           0
#define NWAL_HANDLE_ID_TRANS_ADD_MAC        1
#define NWAL_HANDLE_ID_TRANS_DEL_MAC        2
#define NWAL_HANDLE_ID_TRANS_ADD_IP         3
#define NWAL_HANDLE_ID_TRANS_DEL_IP         4
#define NWAL_HANDLE_ID_TRANS_ADD_IPSEC      5
#define NWAL_HANDLE_ID_TRANS_DEL_IPSEC      6
#define NWAL_HANDLE_ID_TRANS_ADD_PORT       7
#define NWAL_HANDLE_ID_TRANS_DEL_PORT       8
#define NWAL_HANDLE_ID_TRANS_PA_STAT        9
#define NWAL_HANDLE_ID_TRANS_PA_GLOB_CFG    10

/* ID values for Instances */
#define NWAL_HANDLE_ID_INST_MAC             1
#define NWAL_HANDLE_ID_INST_IP              2
#define NWAL_HANDLE_ID_INST_PORT            3
#define NWAL_HANDLE_ID_INST_IPSEC           4
#define NWAL_HANDLE_ID_INST_L2_L3           5
#define NWAL_HANDLE_ID_INST_DM              6 /* Data Mode Channel */

#define NWAL_HANDLE_MAC_INST ((NWAL_HANDLE_TYPE_INST << NWAL_HANDLE_TYPE_SHIFT) | \
                               NWAL_HANDLE_ID_INST_MAC)

#define NWAL_HANDLE_IP_INST ((NWAL_HANDLE_TYPE_INST << NWAL_HANDLE_TYPE_SHIFT) | \
                              NWAL_HANDLE_ID_INST_IP)

#define NWAL_HANDLE_PORT_INST ((NWAL_HANDLE_TYPE_INST << NWAL_HANDLE_TYPE_SHIFT) | \
                                NWAL_HANDLE_ID_INST_PORT)

#define NWAL_HANDLE_IPSEC_INST ((NWAL_HANDLE_TYPE_INST << NWAL_HANDLE_TYPE_SHIFT) | \
                                 NWAL_HANDLE_ID_INST_IPSEC)

#define NWAL_HANDLE_L2_L3_HDR_INST ((NWAL_HANDLE_TYPE_INST << NWAL_HANDLE_TYPE_SHIFT) | \
                                    NWAL_HANDLE_ID_INST_L2_L3)



#define NWAL_HANDLE_DM_INST ((NWAL_HANDLE_TYPE_INST << NWAL_HANDLE_TYPE_SHIFT) | \
                              NWAL_HANDLE_ID_INST_DM)



#define NWAL_CONV_ADDRESS_TO_OFFSET(base, address)  ((uint32_t) address - (uint32_t) base)

#define NWAL_CONV_OFFSET_TO_ADDRESS(base, offset)  (uint32_t)offset? ((uint32_t)base + (uint32_t)offset):(uint32_t)offset






/******************************************************************************
 * Definitions for bitmap: NWAL_STATE_COUNT_T
 ******************************************************************************/
#define NWAL_STATE_MASK                     0xC000
#define NWAL_STATE_SHIFT                    14

#define NWAL_STATE_INACTIVE                 0
#define NWAL_STATE_ACTIVE                   1
#define NWAL_STATE_CFG_IN_PROGRESS          2

#define NWAL_SET_STATE(stateCount,state)  ((state << NWAL_STATE_SHIFT) & \
                                            NWAL_STATE_MASK)
#define NWAL_GET_STATE(stateCount)        (stateCount >> NWAL_STATE_SHIFT)

#define NWAL_INC_COUNT(stateCount)  (stateCount |(state << NWAL_STATE_SHIFT) & \
                                     NWAL_STATE_MASK)



typedef uint16_t              NWAL_HANDLE_TYPE_ID_T; /* Uniquely identifies
                                                      * handle
                                                      */
typedef uint16_t              NWAL_STATE_COUNT_T;    /* Number of connection
                                                      * Handles using the handle
                                                      */
/******************************************************************************
 * Handle Header identifying a channel
 ******************************************************************************/
typedef struct  {
  NWAL_HANDLE_TYPE_ID_T         handleId;           /* Id identifying type of the
                                                     * channel.
                                                     */
  NWAL_STATE_COUNT_T            stateCount;         /* Number of connection
                                                     * Handles using the handle
                                                     */
}nwalHandleHdr_t;

/******************************************************************************
 * Packet handle signature for incoming packet
 ******************************************************************************/
typedef struct nwalPktHandle_tag {
  nwalHandleHdr_t   handleHdr;    /* Handle header for unique identification */
  nwal_AppId        appId;        /* Application Handle for transaction
                                   */
}nwalPktHandle_T;

/******************************************************************************
 * Internal NWAL Transaction signature for message between PA/NETCP IP and NWAL
 ******************************************************************************/
typedef struct  {
  nwalHandleHdr_t   handleHdr;  /* Header for the handle with transaction ID
                                 * details
                                 */
  nwal_AppId        appId;      /* Application Handle for transaction */
  nwal_TransID_t    transId;    /* Transaction ID being passed for the request */
  void*             nwalHandle;  /* NWAL handle for the received message */
} nwalIntTransInfo_t;

/******************************************************************************
 * Mac Handle information retained within NWAL
 ******************************************************************************/
typedef struct nwalMacInfo_tag{
  nwalHandleHdr_t               handleHdr;      /* Handle header for unique
                                                 * identification
                                                 */
  nwal_AppId                    appId;          /* Application Handle for
                                                 * transaction
                                                 */
  uint32_t                      mtuSize;       /** Maximum MTU size limited at MAC 
                                               * interface. Enables IP fragmentation 
                                               * functionality at NetCP. In the case 
                                               * of tunnel IP outer IP only will be 
                                               * fragmented. Configuring NULL will 
                                               * disable the fragmentation at NetCP
                                               */
  nwalIntTransInfo_t            transInfo;      /* Transaction Info for
                                                 * subsequent response from NWAL
                                                 */
  paEthInfo2_t                   paEthInfo;      /* Being stored for future
                                                 * lookup to retrieve
                                                 * NWAL handle given NetCP PA
                                                 * Configuration
                                                 */
  paHandleL2L3_t                paMacHandle;    /* Handle returned by PA LLD */
} nwalMacInfo_t;

/******************************************************************************
 * IP Handle information retained within NWAL
 ******************************************************************************/
typedef struct nwalIpInfo_tag {
  nwalHandleHdr_t               handleHdr;      /* Internal use: Handle header
                                                 * for unique identification
                                                 */
  nwal_AppId                    appId;          /* Application Handle for
                                                 * transaction
                                                 */
  paIpInfo2_t                    paIpInfo;      /* IP details for adding the
                                                * lookup entry
                                                * Refer paIpInfo2_t in the PA API
                                                * file
                                                */
  nwal_Handle                   prevHandle;     /* Handle for the previous
                                                 * protocol layer
                                                 */
  paHandleL2L3_t                paIpHandle;     /* Handle returned by PA LLD */
  nwalIntTransInfo_t            transInfo;
  nwal_SaDir                    dir;            /*  Direction for the channel.
                                                 *  Inbound or Outbound or duplex
                                                 *  Unidirection would be
                                                 *  applicable for IPSec
                                                 */
} nwalIpInfo_t;

/******************************************************************************
 * L2L3 TX header Resource details
 ******************************************************************************/
#define NWAL_MAX_L2_L3_HDR_BUF_SIZE  (NWAL_ETH_DIX_HDR_LEN +  \
                                      NWAL_ETH_VLAN_HDR_SIZE +  \
                                      NWAL_ETH_802_3_HDR_SIZE + \
                                     (NWAL_IPV6_HDR_LEN_BYTES * 2))
typedef struct nwalL2L3HdrInfo_tag {
  nwalHandleHdr_t           handleHdr;
  nwal_Handle               outHandle;
  nwal_Handle               inHandle;
  uint8_t                   outerIpOffset;
  uint8_t                   outerIpVer;
  uint8_t                   ipOffset;
  uint8_t                   ipVer;
  uint8_t                   hdrLen;
  uint8_t                   outerIpId;
  uint16_t                  ipId;
  uint8_t                   hdrBuf[NWAL_MAX_L2_L3_HDR_BUF_SIZE];
} nwalL2L3HdrInfo_t;

/******************************************************************************
 * L2L3 Header creation Struct parameters
 ******************************************************************************/
typedef struct  {
    nwal_Handle             inHandle;   /* Inbound Handle should be either:
                                          *     - For IPSecNWAL handle returned
                                          *       for nwal_setSecPolicy()
                                          *       NWAL_IPSEC_DIR_INBOUND
                                          *     or
                                          *     - NWAL handle returned from
                                          *       nwal_setIPAddr ()
                                          */
    nwal_Handle             outHandle;   /* Outbound Handle should be either:
                                          *     - NWAL handle returned for
                                          *       nwal_setSecPolicy()
                                          *       NWAL_IPSEC_DIR_OUTBOUND
                                          *     or
                                          *     - NULL in the case of non
                                          *       IPSec configuration
                                          */
    nwalMacAddr_t           remMacAddr; /* Remote Mac Address
                                          */
    nwalMacOpt_t            macOpt;     /* MAC header configuration for outgoing
                                         * packet
                                         */
    nwal_IpType             ipType;      /* IPv4/IPv6 :nwal_IpType */
    nwalIpAddr_t            remIpAddr;   /** Remote IP Address */
    nwalIpOpt_t             ipOpt;       /* IP header configuration for
                                          * outgoing packet
                                          */
} nwalL2L3HdrParam_t;

/******************************************************************************
 * L4 UDP/GTPU Port Information
 ******************************************************************************/
typedef struct nwalPortInfo_tag {
  nwalHandleHdr_t           handleHdr;  /* Handle header for unique
                                         * identification
                                         */
  nwal_AppId                appId;      /* Application Handle for transaction */
  nwal_appProtoType_t       proto;
  nwalRxConnCfg_t           rxConnCfg;
  nwal_Bool_t               isRemActive;
  nwalTxConnCfg_t           txConnCfg;
  paHandleL4_t              paPortHandle;   /* Handle returned by PA LLD */
  nwalIntTransInfo_t        transInfo;
  nwalL2L3HdrInfo_t*        pL2L3HdrInfo;
  uint8_t                   hdrBuf[NWAL_UDP_HDR_LEN_BYTES];
} nwalPortInfo_t;


/******************************************************************************
 * Global Stats maintained within NWAL
 ******************************************************************************/
typedef struct  {
  uint16_t            errNonEpibPkts;       /* Packets received w/o EPIB */
  uint16_t            inActiveTrans;        /* Packets received w inactive Trans */
  uint16_t            errPktInst;           /* Packets received for invalid
                                             * instance
                                             */
  uint16_t            errHandleMismatch;    /* Command response received
                                             *  with incorrect handle
                                             */
  uint16_t            numResubmits;         /* Command response received
                                             *  with incorrect handle
                                             */
} nwalStats_t;

/******************************************************************************
 * NWAL route info within NWAL
 ******************************************************************************/
typedef struct  {
  int16_t                 flowId;
  NWAL_queueHnd           rxPktQ;
  nwalRouteType_t         routeType;
  nwal_matchAction_t      matchAction;
  nwal_nextRtFailAction_t failAction;
  uint16_t                egress_switch_port; 
} nwalLocRouteInfo_t;

#ifdef NWAL_ENABLE_SA
#define NWAL_SEC_CONTEXT_BUF_SIZE     sa_MAX_SC_SIZE /* Size of Security Context.*/
#endif

typedef uint16_t nwal_SaChanState;
#define NWAL_CHAN_SA_UNREGISTERED       0x0
#define NWAL_CHAN_SA_SC_BUF_ALLOCATED   0x1
#define NWAL_CHAN_SA_REGISTERED         0x2
#define NWAL_CHAN_SA_ACTIVE             (NWAL_CHAN_SA_SC_BUF_ALLOCATED | \
                                         NWAL_CHAN_SA_REGISTERED);

typedef struct nwalSaComHdrInfo_tag{
  nwalHandleHdr_t               handleHdr;  /* Internal use: Handle header for
                                             * unique identification
                                             */
  nwal_AppId                    appId;
  uint16_t                      index;
  nwal_SaChanState              saChanState;    /* State of the channel with
                                                 * respect to SA LLD
                                                 */
  struct  nwalGlobContext*      pNwalContext;
  uint16_t                      scBufLen;
  uint8_t*                      pScBuf;        /* Security Context Buffer */
  uint16_t                      saChanBufLen;
  uint8_t*                      pSaChanBuf;    /* Buffer for SA Security channel */
#ifdef NWAL_ENABLE_SA
  Sa_SWInfo_t                   regSwInfo;
  Sa_ChanHandle                 saChanHandle;   /* Handle returned by SA for
                                                 * channel
                                                 */
#endif
}nwalSaComHdrInfo_t;

typedef struct nwalIpSecInfo_tag{
  nwalSaComHdrInfo_t            hdr;
  /* Application Handle for transaction */
  nwal_SaDir                    dir;
  nwalSaIpSecId_t               saId;
  nwal_saMode                   saMode;
  uint16_t                      ivSize;
  uint16_t                      macSize;
  paIpInfo2_t                    paIpInfo;
  nwal_Handle                   prevHandle;
  paHandleL2L3_t                paIpHandle; /* Handle returned by PA LLD */
  nwalIntTransInfo_t            transInfo;
  nwalMacAddr_t                 remMacAddr; /* Destination MAC address for the
                                             * channel
                                             */
}nwalIpSecInfo_t;

/******************************************************************************
 * Data Mode SA Info
 ******************************************************************************/
typedef struct nwalDmSaInfo_tag{
  nwalSaComHdrInfo_t            hdr;

  nwal_DmChnType                dmChnType;  /**< Data Mode Channel Type:
                                               * @ref NWAL_DM_CHAN_DECRYPT
                                               * /@ref NWAL_DM_CHAN_ENCRYPT
                                               */

  uint16_t                      aadSize;
  uint16_t                      encIvSize;
  uint16_t                      authIvSize;
#ifdef NWAL_ENABLE_SA
  uint16_t                      psCmdLabelLen;
  Sa_CmdLbUpdateInfo_t          cmdLbUpdate;
  uint32_t                      psCmdLabel[sa_MAX_CMDLB_SIZE/4];
#endif
}nwalDmSaInfo_t;

/******************************************************************************
 * Local Context Per Proc related resources
 ******************************************************************************/

#define NWAL_LOC_INSTANCE_ACTIVE                0xCDEF
typedef struct  {
    uint16_t            state;
    nwalLocCfg_t        cfg;            /* Configuration received during :
                                         * nwal_start
                                         */
    nwalStats_t         stats;          /* Stats maintained within NWAL */
    paSysStats_t        paSysStats;
    nwalIntTransInfo_t  transInfo;
    uint16_t            numPendPAReq;
    Qmss_QueueHnd       rxCtlQ;         /* PA Control response Queue */
    Qmss_QueueHnd       rxL4PktQ;       /* Default Queue for receiving L4 Packets
                                         */
    Qmss_QueueHnd       rxSbSaQ;        /* Packet Queue to receive response for
                                         * all data mode request to SA
                                         */
    uint16_t            extErr;         /* Log of last extended error from NetCP
                                         */
    Cppi_FlowHnd        rxPktFlow;
    int16_t             rxPktFlowId;    /* Extract the flow ID as the flow Handle
                                         * address is local to core
                                         */
    Cppi_FlowHnd        rxCtlFlow;
    int16_t             rxCtlFlowId;    /* Extract the flow ID as the flow Handle
                                         * address is local to core
                                         */

  nwal_Bool_t           enablePAAssistReassem;/** Enable NetCP Assisted
                                                * Reasssembly. The feature will
                                                * allow incoming fragmented packets
                                                * to be reassembled at host and
                                                * redirected back to PA for further
                                                * classification. The feature will
                                                * also maintain ordering of incoming
                                                * packets from network. Application would
                                                * also need to configure pRxReassemProc
                                                * during @ref nwal_start API call
                                                */
    nwal_rxReassemProc* pRxReassemProc; /**< Rx Reassembly routine to be 
                                              *  called once NWAL identifies 
                                              *  incoming fragmented packet.  
                                              *  Mandatory for the case of 
                                              *  NWAL_CTRL_CFG_PA_ASSISTED_REASSEM 
                                              */
Cppi_Handle           cppiHandle;           /* PASS CDMA handle */
nwal_Bool_t           enablePAEQosMode;     /* Enable NETCP Enhanced Qos Mode */

} nwalLocContext_t;
extern nwalLocContext_t*        gPLocContext;
//extern void*                    pSharedMemBase;


/******************************************************************************
 * Global Per System resources shared across all cores.
 ******************************************************************************/
typedef struct  {
#ifdef NWAL_ENABLE_SA
    Sa_Handle           salld_handle;
#endif
    nwalIpSecInfo_t*    pIpSecInfo;             /* Active IpSecInfo Node */
    nwalDmSaInfo_t*     pDmSaInfo;              /* Active Data Mode SA Info */
 }nwalSaGlobContext_t;





#define NWAL_INSTANCE_ACTIVE                0xABCD



typedef struct  nwalGlobContext {
  uint16_t              state;
  uint16_t              masterProcId;   /* Master Proc ID. ID indicating
                                         * processor which initiated nwal_create
                                         */
  nwalGlobCfg_t         cfg;            /* Configuration received during :
                                         * nwal_GlobInit
                                         */
  nwalSizeInfo_t        memSizeInfo;
  nwalMacInfo_t*        pMacInfo;   /*  MAC Instance */
  nwalIpInfo_t*         pIPInfo;    /*  IP Instance */
  nwalPortInfo_t*       pPortInfo;  /*  Port Instance */
  nwalL2L3HdrInfo_t*    pL2L3HdrInfo; /*  L2L3 Transmit Header */
  nwalSaGlobContext_t   saGlobContext;
  int                   extErr;
  Cppi_Handle           cppiHandle; /* PASS CDMA handle */
  Cppi_FlowHnd          rxPaSaFlow;
  int16_t               rxPaSaFlowId;   /* Extract the flow ID as the flow Handle
                                         * address is local to core
                                         */
  Cppi_FlowHnd          rxSaPaFlow;
  int16_t               rxSaPaFlowId;   /* Extract the flow ID as the flow Handle
                                         * address is local to core
                                         */
  Qmss_QueueHnd         defFlowQ;        /* Default packet queue to be used during
                                          * flow creation
                                          * Should not have any packets ideally
                                          */
  Qmss_QueueHnd         txQ[NSS_NUM_TX_QUEUES];            /* Transmit Queue
                                                               * to NetCP
                                                               */
  Cppi_ChHnd            rxChHnd[NSS_NUM_RX_PKTDMA_CHANNELS];
  Cppi_ChHnd            txChHnd[NSS_NUM_TX_PKTDMA_CHANNELS];
  nwalLocContext_t*     pLocContext;
} nwalGlobContext_t;


typedef struct      nwalProcessContext {
    nwalGlobContext_t*  pSharedMemBase;     /* process base address of shared memory segment */
    void*               pLocalCtxInstPool;  /* pointer to process local context instance segment */
    uint8_t             maxThreads;         /* max number of threads in this process */
    nwalBaseAddrCfg_t   baseAddrCfg;
} nwalProcessContext_t;

extern nwalProcessContext_t     nwalProcessCtx;


#ifdef NWAL_ENABLE_SA

/******************************************************************************
 * IPSec Channel Definitions
 ******************************************************************************/
typedef struct  {
    nwalGlobContext_t*              pNwalContext;
    Sa_SecProto_e                   proto;
    uint32_t                        saID;
    uint16_t                        saChanBufLen;
    uint8_t*                        pSaChanBuf;
    Sa_GenConfigParams_t*           pGenConfig;
    nwal_saAALG                     authMode;
    nwal_saEALG                     cipherMode;
    Sa_ChanHandle*                  pSaChanHandle;
    nwal_Bool_t*                    pAuthKeyReqd;
    nwal_Bool_t*                    pEncKeyReqd;
    uint16_t*                       pEncryptionBlockSize;
    uint16_t*                       pSessionSaltSize;
    uint16_t*                       pIvSize;
    uint16_t*                       pAuthIvSize;
}nwal_utlSecAssocParam_t;

#define NWAL_IPSEC_AH_FIXED_HDR_SIZE    12
/********************************************************************
 * FUNCTION PURPOSE: To insert IP Sec Header and make necessary
 *                   adjustments to the offsets
 ********************************************************************
 * DESCRIPTION: Function to insert IP Sec Header and make necessary
 *              adjustments to the offsets
 ********************************************************************/
static inline nwal_RetValue nwal_InsIpSec  ( nwalGlobContext_t*  pIhandle,
                                      Cppi_HostDesc*      pHdrDesc,
                                      nwalL2L3HdrInfo_t*     pL2L3HdrInfoAddr,
                                      nwalIpSecInfo_t*    pIpSecInfoOut,
                                      uint8_t*            pbuffPtr,
                                      uint8_t             macHdrLen,
                                      uint8_t             ipHdrLen,
                                      nwalTxPktInfo_t*    pPktInfo,
                                      uint32_t*           pSwInfo0,
                                      uint32_t*           pSwInfo1)
{
    Sa_PktInfo_t        salldPktInfo;
    Sa_PktDesc_t*       pSalldPktDesc = &salldPktInfo.pktDesc;
    void*               segments[5] = {NULL, NULL, NULL, NULL, NULL};
    uint16_t            segUsedSizes[5] = {0, 0, 0, 0, 0};
    uint16_t            segAllocSizes[5] = {0, 0, 0, 0, 0};
#if 0
    nwalIpSecInfo_t*    pIpSecInfoOut =
                        (nwalIpSecInfo_t*)pPortInfo->pL2L3HdrInfo->outHandle;
#endif
    int16_t             saRetVal;
    uint32_t            origHdrLoc; /* Location of the Original Header before
                                     * passing to SA
                                     */
    uint16_t            offsetAdjust = 0;
    Cppi_HostDesc*      pPloadDesc;

    if(pIpSecInfoOut == NULL)
    {
        return nwal_ERR_INVALID_STATE;
    }
    origHdrLoc = (uint32_t)pbuffPtr;

    pPloadDesc = Pktlib_getDescFromPacket(pPktInfo->pPkt);

    /* Initialize the packet descriptor */
    pSalldPktDesc->nSegments = 2;    /* Two segments. First one would be header
                                      * and second one would be payload
                                      */
    pSalldPktDesc->segments = segments;
    pSalldPktDesc->segUsedSizes = segUsedSizes;
    pSalldPktDesc->segAllocSizes = segAllocSizes;
    pSalldPktDesc->segments[2] = NULL;
    pSalldPktDesc->segUsedSizes[2] = 0;
    pSalldPktDesc->segAllocSizes[2] = 0;
    pSalldPktDesc->size =
        pL2L3HdrInfoAddr->hdrLen + \
        NWAL_UDP_HDR_LEN_BYTES + \
        pPktInfo->ploadLen;

    /* reserve room for potential IPSEC Header insertion */
    pSalldPktDesc->segments[0] = (void *)(pbuffPtr);
    pSalldPktDesc->segUsedSizes[0] =
                pL2L3HdrInfoAddr->hdrLen + NWAL_UDP_HDR_LEN_BYTES;
    pSalldPktDesc->segAllocSizes[0] = pHdrDesc->origBufferLen ;

    /* reserve room for potential IPSEC Header insertion */
    pSalldPktDesc->segments[1] = (void *)(pPloadDesc->buffPtr);
    pSalldPktDesc->segUsedSizes[1] = pPktInfo->ploadLen;
    pSalldPktDesc->segAllocSizes[1] = pPloadDesc->origBufferLen ;


    /* IPSEC operation */
    if( pIpSecInfoOut->saMode == nwal_SA_MODE_TUNNEL)
    {
        pSalldPktDesc->payloadOffset = pL2L3HdrInfoAddr->outerIpOffset;
        pSalldPktDesc->payloadLen =
            (pSalldPktDesc->size - pL2L3HdrInfoAddr->outerIpOffset);
    }
    else
    {
        pSalldPktDesc->payloadOffset = pL2L3HdrInfoAddr->ipOffset;
        pSalldPktDesc->payloadLen =
            (pSalldPktDesc->size - pL2L3HdrInfoAddr->ipOffset);
    }
    salldPktInfo.validBitMap = 0;

    saRetVal =
        Sa_chanSendData(pIpSecInfoOut->hdr.saChanHandle,
                        &salldPktInfo,
                        nwal_FALSE);
    if(saRetVal != sa_ERR_OK)
    {
        return (nwal_ERR_SA);
    }

    /* Prepare to send pkt to PA/SA */
    Cppi_setData (Cppi_DescType_HOST,
                  (Cppi_Desc *)pHdrDesc,
                  (Ptr)pSalldPktDesc->segments[0],
                  pSalldPktDesc->segUsedSizes[0]);
    Cppi_setData (Cppi_DescType_HOST,
                  (Cppi_Desc *)pPloadDesc,
                  (Ptr)pSalldPktDesc->segments[1],
                  pSalldPktDesc->segUsedSizes[1]);
    Cppi_setPacketLen (Cppi_DescType_HOST,
                       (Cppi_Desc *)pHdrDesc,
                       pSalldPktDesc->size);


    /* Cannot use Pktlib_packetMerge() API as packet length is also changed
     * within the API. Since length being provided by SA LLD needs to
     * be overridden, calling CPPI API directly
     */
    Cppi_linkNextBD(Cppi_DescType_HOST,
                    (Cppi_Desc *)pHdrDesc,
                    (Cppi_Desc *)pPloadDesc);

    /* Prepare PS Information
     * The following commands should be passed in the PS Info section
     * - Payload Info (short format) for IPSEC ESP
     */
    /* Offset to L4/UDP. Adjust the offset for IPSec header inserted by LLD */
    offsetAdjust = origHdrLoc - (uint32_t)(pSalldPktDesc->segments[0]);
    pPktInfo->l4OffBytes += offsetAdjust;
    if(pIpSecInfoOut->saMode == nwal_SA_MODE_TUNNEL)
    {
        pPktInfo->ipOffBytes += offsetAdjust;
    }

    /* TBD Outer IP Checksum is updated by SA LLD
     * In transport mode PDSP will still recalculate and
     * overwrite it.
     */

    pPktInfo->saOffBytes = pSalldPktDesc->payloadOffset;
    pPktInfo->saPayloadLen = pSalldPktDesc->payloadLen;
    pPktInfo->saAhIcvOffBytes = 0;
    pPktInfo->saAhMacSize = 0;
    if(pIpSecInfoOut->paIpInfo.proto == NWAL_AH_PROTOCOL)
    {
        pPktInfo->saAhIcvOffBytes =
                                macHdrLen +
                                ipHdrLen +
                                NWAL_IPSEC_AH_FIXED_HDR_SIZE +
                                pIpSecInfoOut->ivSize;
        pPktInfo->saAhMacSize = pIpSecInfoOut->macSize;
        pPktInfo->txFlag1 |= NWAL_TX_FLAG1_DO_IPSEC_AH_CRYPTO;
    }
    else
    {
        /* Offload IPSec Crypto processing to hardware */
        pPktInfo->txFlag1 |= NWAL_TX_FLAG1_DO_IPSEC_ESP_CRYPTO;
    }

    /* Get the swInfo from the one received during channel creation time */
    *pSwInfo0 = pIpSecInfoOut->hdr.regSwInfo.swInfo[0];
    *pSwInfo1 = pIpSecInfoOut->hdr.regSwInfo.swInfo[1];


    return nwal_OK;

}

/********************************************************************
 * FUNCTION PURPOSE: API for transmitting packet out to network
 ********************************************************************
 * DESCRIPTION: API for transmitting packet out to network
 ********************************************************************/
static inline nwal_RetValue nwal_getSaSwInfo (nwal_Handle   nwalHandle,
                                              uint32_t*     pSwInfo0,
                                              uint32_t*     pSwInfo1)
{
    nwalIpSecInfo_t*    pIpSecInfoOut = NULL;
    nwalL2L3HdrInfo_t*  pL2L3HdrInfoAddr;

    if((((nwalHandleHdr_t *)(nwalHandle))->handleId &
          NWAL_HANDLE_ID_MASK) ==  NWAL_HANDLE_ID_INST_PORT)
    {
        nwalPortInfo_t*     pPortInfo;
        pPortInfo = (nwalPortInfo_t* )nwalHandle;
        pL2L3HdrInfoAddr =
                        (nwalL2L3HdrInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                    pPortInfo->pL2L3HdrInfo));
        nwalHandle = 
                        (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                 pL2L3HdrInfoAddr->outHandle));
    }

    if((((nwalHandleHdr_t *)(nwalHandle))->handleId &
          NWAL_HANDLE_ID_MASK) !=  NWAL_HANDLE_ID_INST_IPSEC)
    {
        return (nwal_ERR_INVALID_HANDLE);
    }

    pIpSecInfoOut = (nwalIpSecInfo_t*)nwalHandle;
    *pSwInfo0 = pIpSecInfoOut->hdr.regSwInfo.swInfo[0];
    *pSwInfo1 = pIpSecInfoOut->hdr.regSwInfo.swInfo[1];
    return nwal_OK;
}


#endif

/******************************************************************************
 * FUNCTION PURPOSE: Function to round the size to next cache line size
 *                   to avoid false sharing
 ******************************************************************************
 * DESCRIPTION: Function to round the size to next cache line size
 *              to avoid false sharing
 *****************************************************************************/
static inline uint16_t nwal_round_size(uint16_t roundSize,
                                       uint16_t numElements,
                                       uint16_t elementSize)
{
    uint16_t    round_multiple=0;
    round_multiple = ((elementSize * numElements) / roundSize);
    if((elementSize * numElements) % roundSize)
    {
        round_multiple++;
    }
    return(round_multiple * roundSize);
}

/******************************************************************************
 * FUNCTION PURPOSE: Bit packing to 16 bit value
 ******************************************************************************
 * DESCRIPTION: Bit packing to 16 bit value
 *
 *****************************************************************************/
static inline void nwalWrite16bits_m (uint8_t *base,
                                      uint16_t byteOffset,
                                      uint16_t val)
{
  char *wptr = ((char *)base + byteOffset);

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  wptr[0] = (char)(val>>8);
  wptr[1] = (char)(val & 0xff);

} /* nwalWrite16bits_m */


/********************************************************************
 *  FUNCTION PURPOSE: nwal_updateIpIDLen Update IP Identification
 ********************************************************************
 ********************************************************************/
static inline void nwal_updateIpIDLen(  uint8_t*           pIpheader,
                                        uint16_t*          pIpId,
                                        nwal_Bool_t        updateId,
                                        uint16_t           datagramLength)
{
    uint16_t ipId;
    unsigned int ipHdrLen;

    /* Update IP Header Length */
    ipHdrLen = (pIpheader[NWAL_IPV4_OFFSET_LEN] << 8)
                + (pIpheader[NWAL_IPV4_OFFSET_LEN+1]);
    ipHdrLen += datagramLength;
    pIpheader[NWAL_IPV4_OFFSET_LEN] = (ipHdrLen >> 8) & 0xff;
    pIpheader[NWAL_IPV4_OFFSET_LEN+1] = (ipHdrLen & 0xff);

    /* Update IP ID */
    ipId = NWAL_osalGetProcId();
    ipId = (ipId << NWAL_IP_ID_CORE_SHIFT) | (*pIpId & NWAL_IP_ID_CORE_MASK);

    nwalWrite16bits_m(pIpheader,NWAL_IPV4_OFFSET_ID,ipId);
    if(updateId == nwal_TRUE)
    {
        /* Update ID for next time */
        *pIpId = (uint16_t)((*pIpId & NWAL_IP_ID_CORE_MASK) + 1);
    }
}
/********************************************************************
 *  FUNCTION PURPOSE: nwalCppi_retPSData Return address to PS data
 *                    region within the descriptor which could be used
 *                    for filling in any PS information
 *                    Difference from cppi_getPSData:
 *                    CPPI macro checks for PS data being valid in descriptor
 *                    through the length before passing the pointer. The check
 *                    is not needed in the case of host updating the PS data.
 *
 *                    In this helper utility PS location is always
 *                    assumed to be in the descriptor and descriptor
 *                    type is always assumed to be host.
 *
 *                    Since these are application specific assumptions, the
 *                    macro is defined within NWAL and not being pushed to
 *                    CPPI.
 ********************************************************************
 ********************************************************************/
static inline uint8_t * nwalCppi_retPSData (Cppi_HostDesc *hostDescPtr)
{
    uint8_t         epibPresent;
    uint8_t         *pPSData;

    epibPresent = CSL_FEXTR (hostDescPtr->packetInfo, 31, 31);
    pPSData = (uint8_t *)
              (((uint8_t *) &hostDescPtr->psData) -
               (!epibPresent * CPPI_HOST_DESC_EPIB_SIZE));
    return(pPSData);
}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_prepIpv6Hdr Prepare IPv6 header
 *                    Optional headers are not handled
 ********************************************************************
 ********************************************************************/
static inline void nwal_updateIpv6PloadLen( uint8_t*  pHdrBase,
                                            uint16_t  ploadLen)
{
    nwalWrite16bits_m(pHdrBase,NWAL_IPV6_OFFSET_PLOAD_LEN,ploadLen);
}
/******************************************************************************
 * Function Prototypes
 ******************************************************************************/

nwal_Handle nwal_getInst
(
    void*        pInstBlock,
    uint32_t     maxNumInstances,
    uint32_t     instSize
);

void  nwal_freeInst
(
    nwal_Handle *pHandle,
    uint32_t        instSize
);

nwal_RetValue nwal_pollCtlQ
(
    nwalGlobContext_t*      pIhandle,
    nwal_CmdCallBack*       pCmdCallBack,
    nwal_CmdPaStatsReply*   pPaStatsCallBack,
    nwalHandleHdr_t*        pRefHandleHdr);

void nwal_InvPreHnd
(
    nwal_Handle*    pHandle
);

nwal_RetValue nwal_prepCmdBuf
(
    nwalGlobContext_t*      pIhandle,
    uint16_t*               pCmdSize,
    paCmdReply_t*           pCmdReply,
    nwalIntTransInfo_t*     pTransInfo,
    Cppi_HostDesc**         ppHd
);

nwal_RetValue nwal_txCmdBuf
(
    nwalGlobContext_t*      pIhandle,
    uint16_t                cmdSize,
    int                     cmdDest,
    Cppi_HostDesc*          pHd
);
void nwal_setTransInfo
(
    nwalIntTransInfo_t*     pTransInfo,
    uint16_t                transType,
    void*                   nwalHandle
);
void nwal_saveTransInfo
(
    nwalIntTransInfo_t*     pIntTransInfo,
    nwal_TransID_t          transId
);

nwal_RetValue nwalDelPaHandle
(
    nwalGlobContext_t*       pIhandle,
    nwalIntTransInfo_t*      pTransInfo,
    paHandleL4_t             paL4Handle,
    paHandleL2L3_t*          paL2L3Handle
);
nwal_RetValue nwal_configIP
(
   nwalGlobContext_t*       pIhandle,
   nwalIpInfo_t*            pIpInfo,
   nwalLocRouteInfo_t*         pRouteInfo
);
nwalBufPool_t* nwal_getLinkedBufQ
(
    nwalBufPool_t*          pLinkBuf,
    uint16_t                bufSize,
    uint16_t                numQ
);

void nwal_convIpParam
(
    nwal_IpType           ipType,
    nwalIpAddr_t*         pDst,
    nwalIpAddr_t*         pSrc,
    nwalIpOpt_t*          pIpOpt,
    paIpInfo2_t*           pPaIpInfo
);

nwal_Bool_t nwalCompareByteArray
(
    uint8_t*  v1,
    uint8_t*  v2,
    int       n
);
void* nwal_getNextInst
(
    void*        pInst,
    uint32_t     instSize
);

void nwal_debug_bk();

#ifdef NWAL_ENABLE_SA
nwal_RetValue nwal_SaStart
(
    nwalGlobContext_t*     pIhandle
);

nwal_RetValue nwal_freeSaChan
(
    nwal_Handle         pNwalhandle,
    nwalIpSecInfo_t*    pIpSecInfo
);

nwal_RetValue nwal_getSaBufferReq
(
    nwalSizeInfo_t*       pSizeInfo,
    uint16_t              cacheLineSize,
    int*                  pSizes,
    int*                  pAligns
);

nwal_RetValue nwal_GlobSaCreate
(
    nwalGlobContext_t*     pIhandle,
    const nwalGlobCfg_t*   pCfg,
    nwalSizeInfo_t*        pSizeInfo,
    int                    sizes[nwal_N_BUFS],
    void*                  bases[nwal_N_BUFS],
    int                    verifySizes[nwal_N_BUFS],
    int                    verifyAligns[nwal_N_BUFS]
);


nwal_RetValue nwal_convRouteInfo(nwalGlobContext_t*      pNwalHandle,
                                 nwalLocContext_t*       pLocContext,
                                 nwalLocRouteInfo_t*        pRouteInfo,
                                 int                     dest,
                                 uint32_t                swInfo,
                                 paRouteInfo_t*          pMatchRoute,
                                 paRouteInfo_t*          pFailRoute,
                                 paRouteInfo2_t*         pMatchRoute2,
                                 paRouteInfo2_t*         pFailRoute2);

uint16_t nwal_utilGetIpPsudoChkSum (uint8_t *ip, uint16_t payloadLen);

#endif  /* NWAL_ENABLE_SA */
#define NWAL_LIB_ENABLE_PROFILE
#ifdef NWAL_LIB_ENABLE_PROFILE
#ifndef __ARMv7
extern cregister volatile unsigned int TSCL;
#endif

static inline unsigned long nwal_read_clock(void)
{
#ifndef __ARMv7
    return TSCL;
#else
        volatile int vval;
        //read clock
        asm volatile("mrc p15, 0, %0, c9, c13, 0" :  "=r"(vval));
        return vval;
#endif
}
#endif

/********************************************************************
 * FUNCTION PURPOSE: Get Local Context based on the procID
 ********************************************************************
 * DESCRIPTION: Get Local Context based on the procID
 ********************************************************************/
static inline nwalLocContext_t*
nwal_getLocContext(nwalGlobContext_t*     pIhandle)
{
    uint16_t    procId = NWAL_osalGetProcId();
    uint16_t    cacheLineSize;
    uint32_t    instAdjSize;
    uint8_t*    pLocContext = NULL;
    cacheLineSize = NWAL_CACHE_LINE_SIZE;
    instAdjSize = nwal_round_size(cacheLineSize, 1, sizeof(nwalLocContext_t));


    if(procId >= pIhandle->memSizeInfo.nProc)
    {
        return (NULL);
    }

    pLocContext = (uint8_t*)((uint32_t)nwalProcessCtx.pLocalCtxInstPool + (instAdjSize * procId));

    return((nwalLocContext_t*)pLocContext);
}
static inline void nwalUpdateRoutePriority(nwalRouteType_t type,
                                      paRouteInfo2_t *pRouteInfo2)
{
    switch (type)
    {
        case NWAL_ROUTE_DSCP_PRIORITY:
            pRouteInfo2->validBitMap |= pa_ROUTE_INFO_VALID_PRIORITY_TYPE;
            pRouteInfo2->priorityType = pa_ROUTE_PRIORITY_DSCP;
            break;
        case NWAL_ROUTE_VLAN_PRIORITY:
            pRouteInfo2->validBitMap |= pa_ROUTE_INFO_VALID_PRIORITY_TYPE;
            pRouteInfo2->priorityType = pa_ROUTE_PRIORITY_VLAN;
            break;
        case NWAL_ROUTE_RX_INTF:
            pRouteInfo2->validBitMap |= pa_ROUTE_INFO_VALID_PRIORITY_TYPE;
            pRouteInfo2->priorityType = pa_ROUTE_INTF;
            break;
        case NWAL_ROUTE_RX_INTF_W_FLOW:
            pRouteInfo2->validBitMap |= pa_ROUTE_INFO_VALID_PRIORITY_TYPE;
            pRouteInfo2->priorityType = pa_ROUTE_INTF_W_FLOW;
            break;
        case NWAL_ROUTE_PKTTYPE_EQOS:
            pRouteInfo2->validBitMap |= (pa_ROUTE_INFO_VALID_PKTTYPE_EMAC | pa_ROUTE_INFO_VALID_PRIORITY_TYPE);
            pRouteInfo2->priorityType = pa_ROUTE_EQoS_MODE;
        
        default:
            break;
    }
}
#endif
#ifdef __cplusplus
}
#endif
#endif /* __NWAL_LOC_H__ */
