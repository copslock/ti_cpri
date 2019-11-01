#ifndef _PAFRM_H
#define _PAFRM_H

/******************************************************************************
 * FILE PURPOSE: PASS Firmware Definitions for the PA LLD 
 ******************************************************************************
 * FILE NAME:   pafrm.h
 *
 * DESCRIPTION: Define PASS firmware related constants and data structures  
 *              which are C-type equivalents corresponding to the PDSP-type 
 *              data strutcures defined at fw/pdsp_pa.h
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2009-2013
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
#include <ti/drv/pa/pa.h>

/* System Resource Definitions */
#define PAFRM_NUM_PDSPS         15
#define PAFRM_NUM_CRC_ENGINES   PAFRM_NUM_PDSPS
#define PAFRM_NUM_TIMERS        PAFRM_NUM_PDSPS

/* Global Configurations */
#define PAFRM_SYS_GLOB_CFG_ADDR            0xA0 
#define PAFRM_SYS_GLOB_CFG_OFFSET          (PAFRM_SYS_GLOB_CFG_ADDR/sizeof(uint32_t))
#define PAFRM_SYS_GLOB_CFG_ID_MAX_HDR           0
#define PAFRM_SYS_GLOB_CFG_ID_OUT_IP_REASSM     1
#define PAFRM_SYS_GLOB_CFG_ID_IN_IP_REASSM      2
#define PAFRM_SYS_GLOB_CFG_ID_CMDSET            3
#define PAFRM_SYS_GLOB_CFG_ID_USR_STATS         4
#define PAFRM_SYS_GLOB_CFG_ID_QUEUE_DIVERT      5
#define PAFRM_SYS_GLOB_CFG_ID_IPSEC_NAT_T       6
#define PAFRM_SYS_GLOB_CFG_ID_MAC_PADDING       7
#define PAFRM_SYS_GLOB_CFG_ID_OUT_IP_ACL        8
#define PAFRM_SYS_GLOB_CFG_ID_IN_IP_ACL         9

/* System Timestamp */
#define PAFRM_SYS_TIMESTAMP_ADDR           0xF0 
#define PAFRM_SYS_TIMESTAMP_OFFSET         (PAFRM_SYS_TIMESTAMP_ADDR/sizeof(uint32_t))
#define PAFRM_SYS_TIMESTAMP_OFFSET_HI      (PAFRM_SYS_TIMESTAMP_OFFSET + 1)

/* User Statistics */
#define PAFRM_USR_STATS_ADDR               0x0800
#define PAFRM_USR_STATS_OFFSET             PAFRM_USR_STATS_ADDR/sizeof(uint32_t)

/* Outer IP Traffic flows */
#define PAFRM_OUT_REASSEM_TRAFFIC_FLOW_ADDR      0x0400
#define PAFRM_OUT_REASSEM_TRAFFIC_FLOW_OFFSET    (PAFRM_OUT_REASSEM_TRAFFIC_FLOW_ADDR/sizeof(uint32_t))

/* Inner IP Traffic flows */
#define PAFRM_IN_REASSEM_TRAFFIC_FLOW_ADDR       0x0400
#define PAFRM_IN_REASSEM_TRAFFIC_FLOW_OFFSET     (PAFRM_IN_REASSEM_TRAFFIC_FLOW_ADDR/sizeof(uint32_t))

/* Outer ACL rescore data table */
#define PAFRM_OUT_ACL_RESCORE_DATA_ADDR          0x0800
#define PAFRM_OUT_ACL_RESCORE_DATA_OFFSET        (PAFRM_OUT_ACL_RESCORE_DATA_ADDR/sizeof(uint32_t))

/* Inner ACL rescore data table */
#define PAFRM_IN_ACL_RESCORE_DATA_ADDR           0x0800
#define PAFRM_IN_ACL_RESCORE_DATA_OFFSET         (PAFRM_IN_ACL_RESCORE_DATA_ADDR/sizeof(uint32_t))

/* Outer ACL rescore header table */
#define PAFRM_OUT_ACL_RESCORE_HDR_ADDR           0x0480
#define PAFRM_OUT_ACL_RESCORE_HDR_OFFSET         (PAFRM_OUT_ACL_RESCORE_HDR_ADDR/sizeof(uint32_t))

/* Inner ACL rescore header table */
#define PAFRM_IN_ACL_RESCORE_HDR_ADDR            0x0480
#define PAFRM_IN_ACL_RESCORE_HDR_OFFSET          (PAFRM_IN_ACL_RESCORE_HDR_ADDR/sizeof(uint32_t))

/* Outer IP Reassembly Control Block */
#define PAFRM_OUT_REASSEM_CTRL_BLK_ADDR    0x0800
#define PAFRM_OUT_REASSEM_CTRL_BLK_OFFSET  (PAFRM_OUT_REASSEM_CTRL_BLK_ADDR/sizeof(uint32_t))

/* Inner IP Reassembly Control Block */
#define PAFRM_IN_REASSEM_CTRL_BLK_ADDR     0x0800
#define PAFRM_IN_REASSEM_CTRL_BLK_OFFSET  (PAFRM_IN_REASSEM_CTRL_BLK_ADDR/sizeof(uint32_t))

/* Eflow Record Configuration Command */
#define PAFRM_EF_REC1_CONFIG_CMD_BASE      0x0800
#define PAFRM_EF_REC1_CONFIG_BUF_BASE      0x0810
#define PAFRM_EF_REC1_CONFIG_CMD_OFFSET    PAFRM_EF_REC1_CONFIG_CMD_BASE/sizeof(uint32_t)
#define PAFRM_EF_REC1_CONFIG_BUF_OFFSET    PAFRM_EF_REC1_CONFIG_BUF_BASE/sizeof(uint32_t)

#define PAFRM_EF_REC2_CONFIG_CMD_BASE      0x0900
#define PAFRM_EF_REC2_CONFIG_BUF_BASE      0x0910
#define PAFRM_EF_REC2_CONFIG_CMD_OFFSET    PAFRM_EF_REC2_CONFIG_CMD_BASE/sizeof(uint32_t)
#define PAFRM_EF_REC2_CONFIG_BUF_OFFSET    PAFRM_EF_REC2_CONFIG_BUF_BASE/sizeof(uint32_t)

#define PAFRM_EF_REC3_CONFIG_CMD_BASE      0x0800
#define PAFRM_EF_REC3_CONFIG_BUF_BASE      0x0810
#define PAFRM_EF_REC3_CONFIG_CMD_OFFSET    PAFRM_EF_REC3_CONFIG_CMD_BASE/sizeof(uint32_t)
#define PAFRM_EF_REC3_CONFIG_BUF_OFFSET    PAFRM_EF_REC3_CONFIG_BUF_BASE/sizeof(uint32_t)

#define PAFRM_EF_REC4_CONFIG_CMD_BASE      0x0800
#define PAFRM_EF_REC4_CONFIG_BUF_BASE      0x0810
#define PAFRM_EF_REC4_CONFIG_CMD_OFFSET    PAFRM_EF_REC4_CONFIG_CMD_BASE/sizeof(uint32_t)
#define PAFRM_EF_REC4_CONFIG_BUF_OFFSET    PAFRM_EF_REC4_CONFIG_BUF_BASE/sizeof(uint32_t)

/* PDSP Versions */
#define PAFRM_PDSP_VERSION_BASE            0x1F04
#define PAFRM_PDSP_VERSION_BASE_MC         0x3F04
#define PAFRM_PDSP_VERSION_BASE_EG0        0x1F04
#define PAFRM_PDSP_VERSION_BASE_EG1        0x0F04
#define PAFRM_PDSP_VERSION_BASE_EG2        0x0F04

#define PAFRM_PDSP_VERSION_SRAM_INDEX      (PAFRM_PDSP_VERSION_BASE/PAFRM_SRAM_SIZE)
#define PAFRM_PDSP_VERSION_SIZE            0x20
#define PAFRM_PDSP_VERSION_OFFSET(base, core)    ((base) + PAFRM_PDSP_VERSION_SIZE*(core))/sizeof(uint32_t)  

/*
 * Routing information used to forward packets to the host (via PKTDMA) 
 */
typedef struct pafrmForwardHost_s  {

  uint32_t  context;            /* Context returned as swInfo0 for matched packet */
  uint8_t   ctrlBitMap;         /* Control bitmap, 1 for enable, 0 for disable
                                 * // /-------------------------------------------------------------------------------------------\
                                 * // | 7                 | 6                       |    |       2     |      1      |     0       |
                                 * // | Selection         |                         |    |VLAN priority|DSCP priority| multiRoute  |
                                 * // | 0: Priority Select| 1: Egress Route Enable  |    |             |    OR       |             |
                                 * // | 1: IF Dest Select | 0: Egress Route Disable |    |             |Flow IF Dest |             |
                                 * // \-------------------------------------------------------------------------------------------/
                                 * True if multiple destination enabled */
  uint8_t   multiIdx;           /* Index of the multiple destination set */
  uint8_t   paPdspRouter;       /* PA PDSP number used as multi-route router */
  uint8_t   psFlags;            /* use the bit 7:4 */ /* bit 7: Disable CRC, bit 6:4 port number (0/1/2) bit 3:0 errflags = 0*/
                                /* psFlags may be required when the packet is forwarded through QoS queue */
  uint8_t   cmd[4];             /* optional simple command: 0 means no command */
  
} pafrmForwardHost_t;   /* 12 bytes */

#define PAFRM_MULTIROUTE_ENABLE               0x1
#define PAFRM_ROUTING_PRIORITY_DSCP_ENABLE    0x2
#define PAFRM_ROUTING_PRIORITY_VLAN_ENABLE    0x4
#define PAFRM_ROUTING_FLOW_IF_BASE_ENABLE     0x2      //0: queue-based only; 1: queue- and flow-based
#define PAFRM_ROUTING_FLOW_EQOS_IF_ENABLE     0x40     //0: EQOS Route Enable; 1:  EQOS Route Disable
#define PAFRM_ROUTING_IF_DEST_SELECT_ENABLE   0x80

/*
 * Routing information used to forward packets to the SA (via PKTDMA) 
 */
typedef struct pafrmForwardSa_s   {

  uint32_t  swInfo0;            /* Packet descriptor swInfo0 required by SA operation */
  uint32_t  swInfo1;            /* Packet descriptor swInfo1 required by SA operation */
  uint8_t   cmd[4];             /* optional simple command: 0 means no command */

} pafrmForwardSa_t;

/*
 * Routing information used to forward packets to the SRIO (via PKTDMA) 
 */
typedef struct pafrmForwardSrio_s   {

  uint32_t  psInfo0;           /* 8-byte protocol-specific information required by SRIO  */
  uint32_t  psInfo1;           /* routing */
  uint8_t   pktType;           /* Packet type specified for SRIO operation */
  uint8_t   rsv4[3];

} pafrmForwardSrio_t;

/*
 * Routing information used to forward packets to the Ethernet port 
 */
typedef struct pafrmForwardEth_s   {
  uint8_t  psFlags;  /* use the bit 7:0 */ /* bit 7: CRC present, bit 6: Contagnoli CRC */
  uint8_t  priority; /* Output EMAC packet priority */
  uint16_t rsvd2;
  uint32_t rsvd3;
  uint8_t  cmd[4];  
} pafrmForwardEth_t;

#define PAFRM_ETH_PS_FLAGS_CRC_PRESENT          0x80
#define PAFRM_ETH_PS_FLAGS_CRC_CANTAGNOLI       0x40
#define PAFRM_ETH_PS_FLAGS_CTRL_MAK             0xc0
#define PAFRM_ETH_PS_FLAGS_PORT_MASK            0x0f     /* pa_EMAC_CTRL_PORT_MASK */      
#define PAFRM_ETH_PS_FLAGS_PORT_SHIFT              0

/* Routing information used to forward packets within PA */
typedef struct pafrmForwardPa_s  {

  uint8_t  paDest;      /* PDSP destination */
  uint8_t  customType;  /* None, LUT1, LUT2 */
  uint8_t  customIdx;   /* Index of the custom type if LUT1 or LUT2 custom */
  uint8_t  flags;       /* Control bitmap, 1 for enable, 0 for disable
                         * // /--------------------------------------------------\
                         * // | 7       3|       2     |      1      |     0     |
                         * // |          |ACL drop     | ACL mark    | cacaded   |
                         *    |          |             |             | forwarding|
                         * // \--------------------------------------------------/
                         * True if multiple destination enabled */
  
  uint32_t context;     /* Context returned as swInfo0 for matched packet */
  uint8_t  cmd[4];
  
} pafrmForwardPa_t;

#define PAFRM_CASCADED_FORWARDING               0x01
#define PAFRM_PA_CTRL_PKT_MARK                  0x02  /* Mark the entry per ACL rule */
#define PAFRM_PA_CTRL_PKT_DROP                  0x04  /* Indicate that the packet should be dropped after reassembly per ACL rule */
#define PAFRM_PA_L2_CAPTURE                     0x08  /* Indicate this packet should be duplicated and delivered to the capture queue */

/* Routing information used to forward packets in egress flow */
typedef struct pafrmForwardEf_s  {

  uint8_t  ctrlFlags;    /* various control flags */
  uint8_t  validBitMap;  /* Egress record valid bit map, if flow cache lookup is not enabled */
  uint16_t ravd1;        /* reserved for alignment */
  uint8_t  lvl1RecIndex; /* Egress Flow level one record index */
  uint8_t  lvl2RecIndex; /* Egress Flow level two record index */
  uint8_t  lvl3RecIndex; /* Egress Flow level three record index */
  uint8_t  lvl4RecIndex; /* Egress Flow level four record index */
  uint32_t rsvd3;
  
} pafrmForwardEf_t;

#define PAFRM_EF_CTRL_FC_LOOKUP                 0x01      /* Flow Cache lookup */

#define PAFRM_EF_VALID_REC_LVL1                 0x01
#define PAFRM_EF_VALID_REC_LVL2                 0x02
#define PAFRM_EF_VALID_REC_LVL3                 0x04
#define PAFRM_EF_VALID_REC_LVL4                 0x08

/* Routing information used to drop the packet */
typedef struct pafrmDiscard_s  {

  uint32_t rsvd2;
  
  uint32_t rsvd3;
  uint8_t  cmd[4];
  
} pafrmDiscard_t;


#define PAFRM_CUSTOM_TYPE_NONE pa_CUSTOM_TYPE_NONE    /* 0 */
#define PAFRM_CUSTOM_TYPE_LUT1 pa_CUSTOM_TYPE_LUT1    /* 1 */
#define PAFRM_CUSTOM_TYPE_LUT2 pa_CUSTOM_TYPE_LUT2    /* 2 */

/* Routing information used to forward Ethernet OAM packets */
typedef struct pafrmEoamForward_s  {
  uint32_t context;             /* Information returned in SwInfo0 for the control packets forwarded to host */
  uint16_t statsIndex;          /* entry statistics index */  
  uint8_t  megLevel;            /* MEG threshold level to compare for counter statistics */    
  uint8_t  rsvd1;
  uint32_t rsvd2;               /* Reserved for future use */
} pafrmForwardEoam_t;

/*
 * Routing information used to forward packets fromm PA sub-system to various destinations
 */
typedef struct pafrmForward_s  {

  uint8_t forwardType;          /* Forwarding type as defined below */
  uint8_t flowId;               /* PKTDMA flow Id, valid if forwarding via PKTDMA */
  uint16_t queue;               /* Destination queue number, valid if forwarding via PKTDMA */
  
  union  {
    pafrmForwardHost_t host;    /* Host specific routing information */
    pafrmForwardSa_t   sa;      /* SA specific routing information */
    pafrmForwardSrio_t srio;    /* SRIO specific routing information */
    pafrmForwardEth_t  eth;     /* Ethernet specific routing information */
    pafrmForwardPa_t   pa;      /* PA internal routing information */
    pafrmForwardEf_t   ef;      /* PA Egress Flow information */
    pafrmDiscard_t     discard; /* Discard specific routing information */
    pafrmForwardEoam_t eoam;    /* Ethernet OAM specific routing information */  
  } u;
  
} pafrmForward_t;

enum  {
 PAFRM_FORWARD_TYPE_HOST = 0,     /* use PAFRM_DEST_CDMA */       
 PAFRM_FORWARD_TYPE_SA,           /* use PAFRM_DEST_CDMA */ 
 PAFRM_FORWARD_TYPE_PA,           /* use pa.paDest */
 PAFRM_FORWARD_TYPE_ETH,          /* use PAFRM_DEST_ETH */
 PAFRM_FORWARD_TYPE_SRIO,         /* use PAFRM_DEST_CDMA */
 PAFRM_FORWARD_TYPE_SA_DIRECT,    /* use flowId as stream ID  */
 PAFRM_FORWARD_TYPE_DISCARD,
 PAFRM_FORWARD_TYPE_EFLOW,
 PAFRM_FORWARD_TYPE_EOAM          
};

#define PAFRM_FORWARD_CONTROL_USE_LOC_DMA       0x80          /* SASS only: Use local DMA */
#define PAFRM_FORWARD_TYPE_MASK                 0x0F

/* 
 * EF Command and Command Response
 * Host will write the command and wait for it be executed by PDSP
 * PDSP acknowledge that the command has been processed by 
 * clearing the command field to 0
 *
 * Command Word0: Command
 * /--------------------------------------------------------\
 * | 31    24   | 23      16  | 15                        0 |
 * |  Command   |  Param1     |       Param2                |
 * \--------------------------------------------------------/
 * Command Word1: Command Respond
 * /--------------------------------------------------------\
 * | 31                                                  0 |
 * |           Command respond code                        |
 * \--------------------------------------------------------/
 * Command Word 2/3 Reserved
 * Command Word 4-19: Command Specific Data
 */
 
/*
 *  Command Definitions
 */        

/*
 *  Egress Flow record configuration command
 *  param1: record type
 *  param2: record index
 */ 
#define  PAFRM_HOST_CMD_CFG_EF_RECORD           1    

/*
 *  Command Response Code
 */
#define  PAFRM_HOST_CMD_RESP_OK                 0
#define  PAFRM_HOST_CMD_RESP_ERROR              1

typedef struct pafrmHostCfgCommand_s {
    uint32_t  cmd;
    uint32_t  resp;
}   pafrmHostCfgCommand_t;

#define  PAFRM_HOST_CMD_DATA_BUF_OFFSET         16
#define  PAFRM_HOST_CMD_DATA_BUF_SIZE           64

#define  PAFRM_HOST_FORMAT_COMMAND(code, param1, param2)        (((code) << 24) | ((param1) << 16) | (param2))

/*
 * PA sub-system configuration data structure definitions
 */

/* PA System configurations: */
/* Max Count configuration */
typedef struct pafrmComMaxCount_s  {

  uint8_t  vlanMaxCount;    /* Maximum number of VLAN supported, default = 2, maximum = 3 (obsolete) */
  uint8_t  ipMaxCount;      /* Maximum number of IP layers supported, default = 2, maximum = 7 */
  uint8_t  greMaxCount;     /* Maximum number of GRE layers supported, default = 2, maximum = 7 */
  uint8_t  rsvd;            /* reserved for 32-bit alignment */
  
} pafrmComMaxCount_t;

/* IP Reassembly Configuration */
typedef struct pafrmIpReassmConfig_s {

  uint8_t numTrafficFlow; /* Maximum number of IP reassembly traffic flows supported, default = 0, maximum = 32 */
  uint8_t destFlowId;     /* CPPI flow which instructs how the link-buffer queues are used for forwarding packets */
  uint16_t destQueue;     /* Destination host queue where PASS will deliver the packets which require IP reassembly assistance */
} pafrmIpReassmConfig_t;

/* Command Set Configuration */
typedef struct pafrmCmdSetConfig_t {

  uint8_t  numCmdSets; /* Number of command sets supported (32, 64) 
                          If the number of command sets is set to 64, then each command entry will be limited to 64 bytes.
                          If the number of command sets is set to 32, then each command entry will be limited to 128 bytes */
  uint8_t  size;       /* maximum size of each command set  */
  uint16_t rsvd;       /* alignment */                  
} pafrmCmdSetConfig_t;

/* User-defined Statistics system configuration */
typedef struct pafrmUsrStatsConfig_s{

  uint16_t numCounters;     /* Number of user-defined counters, default = 0, maximum = 512 */
  uint16_t num64bCounters;  /* Number of 64-bit counters, default = 0, maximum = 256 */
   
} pafrmUsrStatsConfig_t;

/* Queue Diversion Configuration */
typedef struct pafrmQueueDivertConfig_s{

  uint16_t destQueue;     /* Destination queue where PASS will deliver the LUT2 response packet which contains the 
                             queue diversion information */
  uint8_t  destFlowId;    /* CPPI flow which instructs how the link-buffer queues are used for forwarding 
                             the LUT2 response packets */
  uint8_t  rsvd;          /* alignment */                            
} pafrmQueueDivertConfig_t;

/* Packet Control Configuration */
typedef struct pafrmPacketControlConfig_s{

  uint16_t  ctrlBitMap;   /* Packet Control bits */
  uint16_t  validBitMap;  /* Packet Control Valid Bits */   
  uint16_t  rxPaddingErrCntIndex;    /* Specify the user statistics index of Rx MAC padding error counter */
  uint16_t  txPaddingCntIndex;       /* Specify the user statistics index of Tx MAC padding counter */               
  uint8_t   egressDefPri; /* global default priority for untagged non IP egress traffic for enhanced QoS mode */ 
  uint8_t   rsvd1;        /* alignment */
  uint16_t  rsvd2;        /* alignmnet */
} pafrmPacketControlConfig_t;

#define PAFRM_PACKET_VERIFY_PPPoE               pa_PKT_CTRL_HDR_VERIFY_PPPoE
#define PAFRM_PACKET_VERIFY_IP                  pa_PKT_CTRL_HDR_VERIFY_IP
#define PAFRM_PACKET_CTRL_MAC_PADDING_CHK       pa_PKT_CTRL_MAC_PADDING_CHK
#define PAFRM_PACKET_CTRL_IP_FRAGS_TO_EROUTE    pa_PKT_CTRL_IP_FRAGS_TO_EROUTE
#define PAFRM_PACKET_CTRL_L3OFFSET_TO_INNER_IP  pa_PKT_CTRL_L3OFFSET_TO_INNER_IP
#define PAFRM_PACKET_CTRL_INGRESS_CAPTURE       pa_PKT_CTRL_EMAC_IF_IGRESS_CLONE
#define PAFRM_PACKET_CTRL_EGRESS_CAPTURE        pa_PKT_CTRL_EMAC_IF_EGRESS_CLONE
#define PAFRM_PACKET_CTRL_DEFAULT_ROUTE         pa_PKT_CTRL_EMAC_IF_INGRESS_DEFAULT_ROUTE
#define PAFRM_PACKET_CTRL_EGRESS_EQoS_MODE      pa_PKT_CTRL_EMAC_IF_EGRESS_EQoS_MODE
#define PAFRM_PACKET_VALID_PADDING_CNT          pa_PKT_CTRL2_VALID_PADDING_STATS_INDEX

/* ACL Configuration (the same as the header portion of pafrmForward_s */
typedef struct pafrmAclConfig_s{
  uint8_t  action;        /* Drop/Forward/Host */                            
  uint8_t  destFlowId;    /* CPPI flow which instructs how the link-buffer queues are used for forwarding 
                             the LUT2 response packets */
  uint16_t destQueue;     /* Destination queue where PASS will deliver the LUT2 response packet which contains the 
                             queue diversion information */
} pafrmAclConfig_t;

#define PAFRM_ACL_ACTION_FORWARD   PAFRM_FORWARD_TYPE_PA
#define PAFRM_ACL_ACTION_DROP      PAFRM_FORWARD_TYPE_DISCARD
#define PAFRM_ACL_ACTION_HOST      PAFRM_FORWARD_TYPE_HOST
/* RA Control Configuration */
typedef struct pafrmRaConfig_s{
  uint8_t   ctrlBitMap;   /* RA control bits */
  uint8_t   flowId;       /* Input flow Id */ 
  uint16_t  rsvd2;        /* alignment */    
} pafrmRaConfig_t;

#define PAFRM_RA_CONTROL_EN             (1 << 0)     /* RA is enabled */
#define PAFRM_RA_CONTROL_USE_LOC_DMA    (1 << 1)     /* Input traffic uses local DMA */
#define PAFRM_RA_CONTROL_TO_QUEUE       (1 << 2)     /* Output packets are delivered to host queue */             

/* Ethernet OAM Global configuration (Shared Scratch RAM) */
#define     PAFRM_EOAM_MAX_PROTOCOLS    8

/* Queue Bounce Configuration */
typedef struct pafrmQueueBounceConfig_s{

  uint16_t ddrQueue;     /* Bounce queue where PASS will deliver the host-routed packet with DDR bit set */
  uint16_t msmcQueue;    /* Bounce queue where PASS will deliver the host-routed packet with MSMC bit set */
} pafrmQueueBounceConfig_t;

typedef struct pafrmEoamCfg_s {
  uint8_t   ctrlBitMap;   /* EOAM Control Bits */
  uint8_t   rsvd[3];      /* alignment */
  uint32_t  nsRoAcc;      /* nano second accumulation over a roll_over periods */  
  uint32_t  nsNumRoAcc;   /* nano second accumulation over (N * roll_over) periods */  

  uint16_t  numRo;        /* Number of Roll overs to consider for error correction  */  
  uint16_t  mul;          /* Multiplication factor, keeping shift factor as 13 */    
  
  uint16_t  exclEthTypes[PAFRM_EOAM_MAX_PROTOCOLS]; /* Eth Protocols that do not trigger counts on target flow match */
} pafrmEoamCfg_t;

#define PAFRM_EOAM_CONTROL_EN             (1 << 0)     /* EOAM is enabled */

/* time stamp offset set configuration (Shared RAM) */
typedef struct pafrmEoamSetTOffsetCfg_s {
  uint32_t  offset_sec;   /* 4 byte 1588 time offset at PA time 0 for seconds */
  uint32_t  offset_ns;    /* 4 byte 1588 time offset at PA time 0 for nano seconds */
} pafrmSetTOffsetCfg_t;

/* Exception routing enumeration */
/* For reference only */
enum  {

  EROUTE_LUT1_FAIL = 0,  /* packet failed to match in LUT1 table */
  EROUTE_VLAN_MAX_DEPTH, /* packet exceeded maximum number of VLAN tags */
  EROUTE_IP_MAX_DEPTH,   /* packet exceeded maximum number of IP headers */
  EROUTE_MPLS_MAX_DEPTH, /* packet exceeded maximum number of MPLS headers */
  EROUTE_GRE_MAX_DEPTH,  /* packet exceeded maximum number of GRE headers */
  EROUTE_PARSE_FAIL,     /* packet failed to parse */
  EROUTE_LUT2_FAIL,      /* packet failed to match in LUT2 table */
  EROUTE_IP_FRAG,        /* IP fragmented packet found in classify2 lookup */
  EROUTE_IPV6_OPT_FAIL,  /* Packet failed due to unsupported IPV6 option header */
  EROUTE_UDP_LITE_FAIL,  /* Udp lite checksum coverage invalid */
  EROUTE_ROUTE_OPTION,   /* IPv4 strict source route or IPv6 routing extension header */
  EROUTE_SYSTEM_FAIL,    /* Unknown system failure - should never happen */
  EROUTE_MAC_BROADCAST,  /* MAC broadcast packet */
  EROUTE_MAC_MULTICAST,  /* MAC multicast packet */
  EROUTE_IP_BROADCAST,   /* IP broadcast packet */
  EROUTE_IP_MULTICAST,   /* IP multicast packet */
  EROUTE_GTPU_MESSAGE_TYPE_1,   /* GTP-U PING Request packet */
  EROUTE_GTPU_MESSAGE_TYPE_2,   /* GTP-U PING Response packet */
  EROUTE_GTPU_MESSAGE_TYPE_26,  /* GTP-U Error Indication packet */
  EROUTE_GTPU_MESSAGE_TYPE_31,  /* GTP-U Supported Header Notification packet */
  EROUTE_GTPU_MESSAGE_TYPE_254, /* GTP-U End Markr packet */
  EROUTE_GTPU_FAIL,             /* packet failed due to GTPU parsing error or unsupporte dmessage types */
  EROUTE_PPPoE_FAIL,            /* Packet failed due to PPPoE session packet parsing error */
  EROUTE_PPPoE_CTRL,            /* PPPoE session stage non-IP packets */
  EROUTE_802_1ag,               /* 802.1ag Packet*/
  EROUTE_IP_FAIL,               /* Packet failed due to invalid IP header */
  EROUTE_NAT_T_KEEPALIVE,       /* NAT-T Keep Alive packet where UDP Length = 9, data = 0xFF */
  EROUTE_NAT_T_CTRL,            /* NAT-T control packet where UDP Length > 12 and the first 4 payload bytes are equal to 0 */
  EROUTE_NAT_T_DATA,            /* NAT-T IPSEC ESP data packet where UDP Length > 12 and the first 4 payload bytes are not equal to 0 */
  EROUTE_NAT_T_FAIL,            /* Invalid NAT-T packet */
  EROUTE_GTPU_MATCH_FAIL,       /* Packet failed to match GTPU */
  EROUTE_N_MAX                  /* Number of error routes */
};

/* Egress Flow Exception routing enumeration */
/* For reference only */
enum  {

  EF_EROUTE_FC_FAIL = 0,    /* packet failed to match in Flow Cache (LUT1) table */
  EF_EROUTE_PARSE_FAIL,     /* packet failed to parse */
  EF_EROUTE_IP_FRAG,        /* IP fragmented packet found in Flow Cache lookup */
  EF_EROUTE_IPV6_OPT_FAIL,  /* Packet failed due to unsupported IPV6 option header */
  EF_EROUTE_IP_OPTIONS,     /* IP packet with IPv4 options and IPv6 extension headers */
  EF_EROUTE_IP_EXPIRE,      /* IP packet with TTL expired or Hop Limitis reached */
  EF_EROUTE_TCP_CTRL,       /* TCP Control packets */  
  EF_EROUTE_INVALID_REC,    /* Invalid Egree Flow Records */
  EF_EROUTE_8,              /* place holder  */
  EF_EROUTE_9,              /* place holder  */
  EF_EROUTE_SYSTEM_FAIL,    /* Unknown system failure - should never happen */
  EF_EROUTE_N_MAX           /* Number of error routes */
};

/* Configure the exception routes */
typedef struct pafrmComEroute_s  {

  uint32_t         routeBitMap;             /* Exception route vaild bitmap */
  pafrmForward_t   eRoute[EROUTE_N_MAX];    /* Array of exception routing information */
} pafrmComEroute_t;

/* TBD: add size error case for compiler */


/* PA Global Configuartions */
/* Custom LUT1 header processing */
typedef struct pafrmC1Custom_s  {

  uint8_t  idx;             /* Custom LUT1 index */
  uint8_t  nextHdr;         /* next header to be parsed */
  uint16_t offset;          
  uint16_t nextHdrOffset;   /* offset to the next header */
  uint16_t rsvd;            /* reserved for 32-bit alignment */
  uint8_t  bitMask[32];     /* 32 8-bit (256-bit) data masks */
  
} pafrmC1Custom_t;

/* Custom LUT2 header processing */
typedef struct pafrmC2Custom_s  {

  uint8_t  idx;         /* Custom LUT2 index */
  uint8_t  bitSet;      /* Use to set the most significant bits to distinguish the custom entry with others */
  uint8_t  ctrlBitMap;  /* Control Bit Map 
                         * b0: Use Link ==> us the upper link for the last match byte
                         */
#define PAFRM_C2_CUSTOM_CTRL_USE_LINK       0x01                         
  uint8_t  hdrSize;     /* custom header size in bytes */
  uint16_t offset[4];   /* array of offsets to the bytes used in LUT2 custom matching */
  uint8_t  bitMask[4];  /* array of masked bits that are valid in the custom LUT2 matching */
  
} pafrmC2Custom_t;

/* 802.1ag Detection Configuration */
typedef struct pafrm802p1agDet_s  {
  uint8_t  ctrlBitMap;  /* Control Bit Map
                         * b0: enable/disable
                         * b1: standard/draft
                         */
                           
  uint8_t  rsvd;        /* alignment */
  uint16_t rsvd2;       /* alignment */
} pafrm802p1agDet_t;

#define PAFRM_802_1ag_DET_ENABLE       (1 << 0)
#define PAFRM_802_1ag_DET_STANDARD     (1 << 1)


/* IPSEC NAT-T Detection Configuration */
typedef struct pafrmIpsecNatTDet_s  {
  uint8_t  ctrlBitMap;  /* Control Bit Map
                         * b0: enable/disable
                         */
                           
  uint8_t  rsvd;        /* alignment */
  uint16_t udpPort;     /* UDP port number which uniquely identifies the IPSEC NAT-T packets */
} pafrmIpsecNatTDet_t;

#define PAFRM_IPSEC_NAT_T_DET_ENABLE   (1 << 0)

/* GTP-U Classification Configuration */
typedef struct pafrmGtpuCfg_s  {
  uint8_t  ctrlBitMap;  /* Control Bit Map
                         * b0: enable/disable  link
                         */
                           
  uint8_t  rsvd;        /* alignment */
  uint16_t rsvd2;       /* alignment */
} pafrmGtpuCfg_t;

#define PAFRM_GTPU_CTRL_USE_LINK                     (1 << 0)
#define PAFRM_GTPU_CTRL_ROUTE_END_MARKER_AS_GPDU     (1 << 1)

/* Packet Capture configuration */
typedef struct pafrmPktCapIfCfg_s {
  uint8_t   capturePort; /* Capture port */
  uint8_t   rsvd1;
  uint16_t  rsvd2;
  uint8_t   ctrlBitmap; /* Control Bit Map
                         * b0: enable/disable feature
                         * b1: 1: dest = HOST otherwise EMAC
                         */
  uint8_t   emacport_flow; /* Mirror ethernet port or flow information */
  uint16_t  queue;      /* Queue to use for Host destination */

  uint32_t  context;    /* Context returned as swInfo0 for matched packet */
} pafrmPktCapIfCfg_t;

#define PAFRM_PKT_CAP_ENABLE   (1 << 0)
#define PAFRM_PKT_CAP_HOST     (1 << 1)

/* Maximum 9 ports including CPPI ports can be captured */
#define PAFRM_PKT_CAP_MAX_PORT    9

typedef struct pafrmPktCapCfg_s {
  uint8_t            numPorts; /* number of ports to be configured */
  uint8_t            rsvd1;
  uint16_t           rsvd2;
  pafrmPktCapIfCfg_t pktCapCfg[PAFRM_PKT_CAP_MAX_PORT];
} pafrmPktCapCfg_t;


/* Default routing enumeration */
/* For reference only */
enum  {

  DROUTE_MULTICAST = 0,  /* default multicast route */
  DROUTE_BROADCAST,      /* default broadcast route */
  DROUTE_UNICAST,        /* default unicast route */
  DROUTE_N_MAX
};

/* Ingress packet default route configuration for multicast, broadcast and Unicast packets */
typedef struct pafrmDefRouteInfo_s {
  uint8_t        ctrlBitmap; /* Control Bit Map
                              * b0: enable/disable route config for MC packets
                              * b1: enable/disable route config for BC packets
                              * b2: enable/disable route config for UC packets 
                              * b3: Pre classification enabled for default route
                              *     otherwise post classification for default route
                              */                                
  uint8_t        port;       /* ingress port */                           
  uint16_t       rsvd;       /* alignment */                               
  pafrmForward_t dRoute[DROUTE_N_MAX];  /* Has Multicast route, BroadCast Route and Unicast route */					
} pafrmDefRouteInfo_t;

#define PAFRM_DEFAULT_ROUTE_MC_ENABLE             (1 << 0)
#define PAFRM_DEFAULT_ROUTE_BC_ENABLE             (1 << 1)
#define PAFRM_DEFAULT_ROUTE_UC_ENABLE             (1 << 2)
#define PAFRM_DEFAULT_ROUTE_PRE_CLASSIFY_ENABLE   (1 << 3)

/* Maximum 4 ports can be captured */
#define PAFRM_MAX_EMAC_PORT    (PAFRM_PKT_CAP_MAX_PORT - 1)

typedef struct pafrmDefRouteCfg_s {
  uint8_t             numPorts; /* number of ports to be configured */
  uint8_t             rsvd1;
  uint16_t            rsvd2;
  pafrmDefRouteInfo_t routeCfg[PAFRM_MAX_EMAC_PORT];
} pafrmDefRouteCfg_t;

/* Egress packet route configuration for QoS Support */
typedef struct {
  uint8_t               ctrlBitMap;       /* Specifies the control for egress interface based route as defined in @ref PaEQoSCtrlCode_e */
  uint8_t               flowBase;         /* Applicable for egress (SoC generated packets) */
  uint16_t              queueBase;        /* Applicable for egress (SoC generated packets) */  
  
  uint8_t               ingressDefPri;    /* ingress port default priority */
  uint8_t               port;             /* Port for which the configuration is sent */ 
  uint16_t              vlanId;           /* Specifies the VLAN to be used/replaced at the output packet */
  
  paRouteOffset_t       pbitMap[8];       /* Specifies the P bit-to-flow/queue offset mapping */
  paRouteOffset_t       dscpMap[64];      /* Specifies the DSCP-to-flow/queue offset mapping */
} pafrmEQosModeIf_t;

typedef struct pafrmPktEgressRouteCfg_s {
  uint8_t             numPorts;         /* number of ports to be configured */
  uint8_t             scratch;          /* Intermediate scratch area used */
  uint16_t            rsvd;             /* alignment only */
  
  pafrmEQosModeIf_t   eQoSCfg[PAFRM_MAX_EMAC_PORT];
} pafrmEQosModeConfig_t;

/* PA global configuration command */
typedef struct pafrmCommandConfigPa_s
{
  uint16_t validFlag;   /* valid bitmap as defined below */
  uint16_t rsvd2;       /* reserved for alignment */
  
  pafrmComMaxCount_t         maxCounts;      /* Maxmium counts configuration */
  pafrmIpReassmConfig_t      outIpReasm;     /* Outer IP Reassembly Configuration */
  pafrmIpReassmConfig_t      inIpReasm;      /* Inner IP Reassembly Configuration */
  pafrmCmdSetConfig_t        cmdSet;         /* Command Set Configuartion */
  pafrmUsrStatsConfig_t      usrStats;       /* User-defined Statistics Configuration */
  pafrmQueueDivertConfig_t   queueDivert;    /* LUT2 queue diversion configuration */
  pafrmPacketControlConfig_t pktCtrl;        /* Packet control configuration */ 
  pafrmAclConfig_t           outAcl;         /* Outer IP ACL Configuration */
  pafrmAclConfig_t           inAcl;          /* Inner IP ACL Configuration */
  pafrmRaConfig_t            outRa;          /* Outer IP RA configuration */
  pafrmRaConfig_t            inRa;           /* Inner IP RA configuration */
  pafrmQueueBounceConfig_t   queueBounce;    /* Queue bounce configuration */
  pafrmEoamCfg_t             eoam;           /* Ethernet OAM global configuration */  
} pafrmCommandConfigPa_t;

#define PAFRM_COMMAND_CONFIG_VALID_MAX_COUNTS       (1 << 0)
#define PAFRM_COMMAND_CONFIG_VALID_OUT_IP_REASSEM   (1 << 1)
#define PAFRM_COMMAND_CONFIG_VALID_IN_IP_REASSEM    (1 << 2)
#define PAFRM_COMMAND_CONFIG_VALID_CMDSET           (1 << 3)
#define PAFRM_COMMAND_CONFIG_VALID_USR_STATS        (1 << 4)
#define PAFRM_COMMAND_CONFIG_VALID_QUEUE_DIVERT     (1 << 5)
#define PAFRM_COMMAND_CONFIG_VALID_PKT_CTRL         (1 << 6)
#define PAFRM_COMMAND_CONFIG_VALID_OUT_IP_ACL       (1 << 7)
#define PAFRM_COMMAND_CONFIG_VALID_IN_IP_ACL        (1 << 8)
#define PAFRM_COMMAND_CONFIG_VALID_OUT_IP_RA        (1 << 9)
#define PAFRM_COMMAND_CONFIG_VALID_IN_IP_RA         (1 << 10)
#define PAFRM_COMMAND_CONFIG_VALID_QUEUE_BOUNCE     (1 << 11)
#define PAFRM_COMMAND_CONFIG_VALID_EOAM             (1 << 12)

/* PA system configuration command */
typedef struct pafrmCommandSysConfigPa_s
{
  uint8_t  cfgCode;     /* Specify the system configuration code as defined below */
  uint8_t  rsvd1;
  uint16_t rsvd2;       /* reserved for alignment */
  
  union {
    pafrmComEroute_t          eroute;         /* Exception routes configuration */
    pafrmC1Custom_t           customC1Config; /* Custom LUT1 configuration */
    pafrmC2Custom_t           customC2Config; /* Custom LUT2 configuration */
    pafrm802p1agDet_t         pa802p1agDet;   /* 802.1ag packet detection Configuration */
    pafrmIpsecNatTDet_t       ipsecNatTDet;   /* IPSEC NAT-T Packet detection configuratiuon */
    pafrmGtpuCfg_t            gtpuCfg;        /* GTPU classification configuratiuon */
    pafrmPktCapCfg_t          igressPktCapCfg; /* Ingress packet capture configuration */
    pafrmPktCapCfg_t          egressPktCapCfg; /* Egress packet capture configuration */	
    pafrmDefRouteCfg_t        defRouteCfg;     /* Ingress default route configuration */	
    pafrmEQosModeConfig_t     eqosCfg;         /* Eqos configuration */
    pafrmSetTOffsetCfg_t      timeOffsetCfg;   /* time offset configuration */
  } u;  
  
} pafrmCommandSysConfigPa_t;

/* PA system configuration codes */
#define PAFRM_SYSTEM_CONFIG_CODE_EROUTE              0
#define PAFRM_SYSTEM_CONFIG_CODE_CUSTOM_LUT1         1
#define PAFRM_SYSTEM_CONFIG_CODE_CUSTOM_LUT2         2
#define PAFRM_SYSTEM_CONFIG_CODE_802_1AG             3
#define PAFRM_SYSTEM_CONFIG_CODE_IPSEC_NAT_T         4
#define PAFRM_SYSTEM_CONFIG_CODE_GTPU                5
#define PAFRM_SYSTEM_CONFIG_CODE_IGRESS_PCAP         6
#define PAFRM_SYSTEM_CONFIG_CODE_EGRESS_PCAP         7
#define PAFRM_SYSTEM_CONFIG_CODE_DEFAULT_ROUTE       8
#define PAFRM_SYSTEM_CONFIG_CODE_EQoS_MODE           9
#define PAFRM_SYSTEM_CONFIG_CODE_TIME_OFFSET_CFG    10

/* Protocol field values (IPV4) / next header (IPV6) */
#define IP_PROTO_IPV6_HOP_BY_HOP    0   // IPv6 extension header - hop by hop 
#define IP_PROTO_IP_IN_IP           4   // IP tunneling 
#define IP_PROTO_TCP                6 
#define IP_PROTO_UDP               17
#define IP_PROTO_IPV6_IN_IPV4      41   // IP tunneling 
#define IP_PROTO_IPV6_ROUTE        43   // IPv6 extension header - route 
#define IP_PROTO_IPV6_FRAG         44   // IPv6 extension header - fragmentation 
#define IP_PROTO_GRE               47
#define IP_PROTO_ESP               50   // Encapsulating security payload 
#define IP_PROTO_AUTH              51   // Authentication header (ipv4) 
#define IP_PROTO_IPV6_NO_NEXT      59   // IPv6 extention header - no next header      
#define IP_PROTO_IPV6_DEST_OPT     60   // IPv6 extension header - destination options
#define IP_PROTO_SCTP             132
#define IP_PROTO_UDP_LITE         136


/* LUT1 classification mode */
#define PAFRM_LUT1_CLASS_NONE       0
#define PAFRM_LUT1_CLASS_STANDARD   1
#define PAFRM_LUT1_CLASS_IPV4       2
#define PAFRM_LUT1_CLASS_IPV6       3
#define PAFRM_LUT1_CLASS_IPSEC      PAFRM_LUT1_CLASS_IPV6

#define PAFRM_LUT1_CLASS_SHIFT      30      /*  Care0 [31:30] */

/* LUT1 Range mode fot two range parameters */
#define PAFRM_LUT1_RANGE_MODE_NORMAL    0   /* Normal comparsion */
#define PAFRM_LUT1_RANGE_MODE_RANGE     1   /* Use RangeLo and RangeHi for range compare */
#define PAFRM_LUT1_RANGE_MODE_NOT       2   /* NOT comparsion */

#define PAFRM_LUT1_CLASS_SHIFT0        28   /* Care0 [29:28]: Byte 42-43*/
#define PAFRM_LUT1_CLASS_SHIFT1        26   /* Care0 [27:26]: Byte 44-45*/

/* LUT1 NOT operation for byte 41 */
#define PAFRM_LUT1_CMP_OP_NORMAL        0   /* Normal Comparsion */
#define PAFRM_LUT1_CMP_OP_NOT           1   /* Not cpmpare (byte 41) */

#define PAFRM_LUT1_CMP_OP_SHIFT        25   /* Care 0 [25]: Byte 41 */


/* LUT1 Entries (MAC/SRIO/Custom) */
/* Layer 2 packet Type */
#define PAFRM_L2_PKT_TYPE_MAC       0x80
#define PAFRM_L2_PKT_TYPE_SRIO      0x40
#define PAFRM_L2_PKT_TYPE_CUSTOM    0x20

#define PAFRM_LUT1_VALID_DMAC_ALL   0x001F8000  /* Care0: [20:15] byte 4-9 */
#define PAFRM_LUT1_VALID_DMAC_MINUS_BYTE5   0x001F0000
#define PAFRM_LUT1_VALID_SMAC       0x00007E00  /* Care0: [14:9]  byte 10-15 */
#define PAFRM_LUT1_VALID_ETHERTYPE  0x00000180  /* Care0: [8:7]   byte 16-17 */
#define PAFRM_LUT1_VALID_SESSION_ID 0x00000060  /* Care0: [6:5]   byte 18-19 */
#define PAFRM_LUT1_VALID_MPLS       0x0000001E  /* Care0: [4:1]   byte 20-23 */
#define PAFRM_LUT1_VALID_PKTFLAGS   0x01000000  /* Care0: [24:24] Byte 0 */
#define PAFRM_LUT1_VALID_DMAC5      0x00800000  /* Care0: [23:23] Byte 1 */

#define PAFRM_LUT1_VALID_VLANID1    0x00180000  /* Care1: [20:19] Byte 36-37 */
#define PAFRM_LUT1_VALID_VLANID2    0x00060000  /* Care1: [18:17] Byte 38-39 */
#define PAFRM_LUT1_VALID_PKTTYPE    0x00010000  /* Care1: [16:16] Byte 40    */
#define PAFRM_LUT1_VALID_INPORT     0x00008000  /* Care1: [15:15] Byte 41    */
#define PAFRM_LUT1_VALID_VLAN_PRI1  0x00006000  /* Care1: [14:13] Byte 42-43 */
#define PAFRM_LUT1_VALID_VLAN_PRI2  0x00001800  /* Care1: [12:11] Byte 44-45 */
#define PAFRM_LUT1_VALID_SRC_VC     0x00000600  /* Care1: [10: 9] Byte 46-47 */

#define PAFRM_MK_SRC_VC(pdsp, lutIndex)     ((pdsp) << 10) + (lutIndex)
#define PAFRM_GET_PDSPID_FROM_LINK(lnk)     (lnk) >> 10
#define PAFRM_GET_LUTIDX_FROM_LINK(lnk)     (lnk) & 0x3FF
   

typedef struct pafrmComL1Mac_s  {

  /* LUT1 view 1 */
  uint8_t   dmac[6];      /* Destination mac */
  uint8_t   smac[6];      /* Source mac */
  uint16_t  etherType;    /* Ether Type */
  uint16_t  sessionId;    /* PPPoE session ID */
  
  /* LUT1 view 2 */
  uint32_t  mpls;         /* MPLS label */
  uint32_t  rsvd2_2;
  uint32_t  rsvd2_3;
  uint32_t  rsvd2_4;
  
  /* LUT1 view 3 */
  uint8_t   pktFlags;     /* Various packet flags */
#define PAFRM_MAC_FLAG_VLAN1        0x01  
#define PAFRM_MAC_FLAG_VLAN2        0x02  
#define PAFRM_MAC_FLAG_MCAST        0x04  
#define PAFRM_MAC_FLAG_BCAST        0x08  
#define PAFRM_MAC_FLAG_PPPoE        0x10
#define PAFRM_MAC_FLAG_802p3        0x20
#define PAFRM_MAC_FLAG_MPLS         0x40  
  
  uint8_t   dstMac5;      /* Destination MAC address if bitMask is required */
  uint16_t  rsvd4;
  uint16_t  vlanId1;      /* 12-bit ID of inner VLAN (0x8100) */
  uint16_t  vlanId2;      /* 12-bit ID of outer VLAN */
  uint8_t   pktType;      /* Common filed to indicate packet type */
  uint8_t   inport;       /* One-base input EMAC port number */
  uint16_t  vlanPri1;     /* 3-bit priority of inner VLAN (0x8100) */
  uint16_t  vlanPri2;     /* 3-bit priority of outer VLAN */
  uint16_t  srcVC;        /* virtual or physical link */
  
} pafrmComL1Mac_t;

/* SRIO specific vaild bit defitions */
#define PAFRM_LUT1_VALID_SRIO_TYPE_PARAM2 0x00800000  /* Care0: [23:23] Byte 1     */
#define PAFRM_LUT1_VALID_SRIO_TYPE_PARAM1 0x00600000  /* Care0: [22:21] Byte 2-3   */
#define PAFRM_LUT1_VALID_SRIO_SRC_ID      0x00180000  /* Care1: [20:19] Byte 36-37 */
#define PAFRM_LUT1_VALID_SRIO_DST_ID      0x00060000  /* Care1: [18:17] Byte 38-39 */
#define PAFRM_LUT1_VALID_SRIO_CC          0x00008000  /* Care1: [15:15] Byte 41    */
#define PAFRM_LUT1_VALID_SRIO_PRI         0x00006000  /* Care1: [14:13] Byte 42-43 */

typedef struct pafrmComL1Srio_s  {

  /* LUT1 view 1 */
  uint32_t  rsvd1_1;
  uint32_t  rsvd1_2;
  uint32_t  rsvd1_3;
  uint32_t  rsvd1_4;
  
  /* LUT1 view 2 */
  uint16_t  nextHdrOffset;    /* place holder for nextHdr and nextOffset, not used for classification */
  uint8_t   nextHdr;
  uint8_t   rsvd2_1;          
  uint32_t  rsvd2_2;
  uint32_t  rsvd2_3;
  uint32_t  rsvd2_4;
  
  /* LUT1 view 3 */
  uint8_t   pktFlags;      /* Various packet flags */
#define PAFRM_SRIO_FLAG_TYPE9        0x01  
#define PAFRM_SRIO_FLAG_TYPE11       0x02  
#define PAFRM_SRIO_FLAG_PORT8        0x04  
  
  uint8_t   typeParam2;    /* cos or letter */
  uint16_t  typeParam1;    /* stream ID or mailbox */
  uint16_t  srcId;         /* Device Source ID */
  uint16_t  dstId;         /* Device Destination ID */
  uint8_t   pktType;       /* Common filed to indicate packet type */
  uint8_t   cc;            /* Completion code */
  uint16_t  pri;           /* 3-bit priority */
  uint16_t  rsvd3_1;         
  uint16_t  srcVC;         /* virtual or physical link */
  
} pafrmComL1Srio_t;

/* Custom LUT1 specific vaild bit defitions */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK1 0x001E0000  /* Care0: [20:17] Byte 4-7  */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK2 0x0001E000  /* Care0: [16:13] Byte 8-11  */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK3 0x00001E00  /* Care0: [12:9]  Byte 11-15  */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK4 0x000001E0  /* Care0: [8:5] Byte 16-19  */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK5 0x0000001E  /* Care0: [4:1] Byte 20-23  */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK6_0 0x00000001  /* Care0: [0:0] Byte 24   */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK6_1 0xE0000000  /* Care1: [31:29] Byte 25-27 */

#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK7 0x1E000000  /* Care1: [28:25] Byte 28-31  */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_MASK8 0x01E00000  /* Care1: [24:21] Byte 32-35  */

#define PAFRM_LUT1_VALID_CUSTOM_LUT1_0     0x001FFFFF  /* Care0: [20:0]  Byte 4-24 */
#define PAFRM_LUT1_VALID_CUSTOM_LUT1_1     0xFFE00000  /* Care1: [31:21] Byte 25-35 */

typedef struct pafrmComL1Custom_s  {

  /* LUT1 view 1 & 2 */
  uint8_t   match[32];
  
  /* LUT1 view 3 */
  uint32_t  rsvd3_1;       
  uint32_t  rsvd3_2;       
  uint8_t   pktType;       /* Common filed to indicate packet type */
  uint8_t   rsvd3_3;
  uint16_t  rsvd3_4;
  uint16_t  rsvd3_5;         
  uint16_t  srcVC;         /* virtual or physical link */
  
} pafrmComL1Custom_t;

/* Layer 3 packet Type */
#define PAFRM_L3_PKT_TYPE_FIREWALL   0x80
#define PAFRM_L3_PKT_TYPE_IP         0x40
#define PAFRM_L3_PKT_TYPE_CUSTOM     0x20
#define PAFRM_L3_PKT_TYPE_IPSEC      0x10
#define PAFRM_L3_PKT_TYPE_FC         0x08


/* IPv4 specific valiod bit definitions */
#define PAFRM_LUT1_IPV4_SRC_PORT_RANGE    0x10000000 /* Care0: [29:28]  for Byte 42-43 */
#define PAFRM_LUT1_IPV4_DST_PORT_RANGE    0x04000000 /* Care0: [27:26]  for Byte 44-45 */
#define PAFRM_LUT1_IPV4_SUBNET_MASK       0x0000003F
#define PAFRM_LUT1_IPV4_SUBNET_DIP_SHIFT    19    /* Care0 [24:19] */
#define PAFRM_LUT1_IPV4_SUBNET_SIP_SHIFT    13    /* Care0 [18:13] */

#define PAFRM_LUT1_VALID_IPV4_PKTFLAGS    0x00001800 /* Care0: [12:11] Byte 0-1 */
#define PAFRM_LUT1_VALID_IPV4_DSCP        0x00000400 /* Care0: [10:10] Byte 2   */

#define PAFRM_LUT1_VALID_IPV4_PKTTYPE     0x00001000 /* Care1: [12:12] Byte 40    */
#define PAFRM_LUT1_VALID_IPV4_PROTO       0x00000800 /* Care1: [11:11] Byte 41    */
#define PAFRM_LUT1_VALID_IPV4_SRC_PORT    0x00000600 /* Care1: [10:9]  Byte 42-43 */
#define PAFRM_LUT1_VALID_IPV4_DST_PORT    0x00000180 /* Care1: [8:7]   Byte 44-45 */
#define PAFRM_LUT1_VALID_IPV4_SRC_VC      0x00000060 /* Care1: [6:5]   Byte 46-47 */

typedef struct pafrmComL1Ipv4_s  {

  /* LUT1 view 1 */
  uint32_t  dstIp;            /* Destination IP address */         
  uint32_t  srcIp;            /* Source IP address */
  uint32_t  rsvd1_3;
  uint32_t  rsvd1_4;
  
  /* LUT1 view 2 */
  uint32_t  rsvd2_1;          
  uint32_t  rsvd2_2;
  uint32_t  rsvd2_3;
  uint32_t  rsvd2_4;
  
  /* LUT1 view 3 */
  uint16_t  pktFlags;      /* Various packet flags */
#define PAFRM_IP_FLAG_IP_TYPE      0x8000  
#define PAFRM_IP_FLAG_V4           0x8000  
#define PAFRM_IP_FLAG_GRE          0x4000
#define PAFRM_IP_FLAG_SCTP         0x2000
#define PAFRM_IP_FLAG_TCP_DATA     0x1000  
#define PAFRM_IP_FLAG_OPTIONS      0x0800  
#define PAFRM_IP_FLAG_FRAG         0x0400  
#define PAFRM_IP_FLAG_CONTAIN_L4   0x0200
#define PAFRM_IP_FLAG_HOP_LIMIT    0x0100
#define PAFRM_IP_FLAG_IPSEC        0x0080

  uint8_t   dscp;  
  uint8_t   rsvd3_1;      
  uint32_t  rsvd3_2;       
  uint8_t   pktType;       /* Common filed to indicate packet type */
  uint8_t   protocol;      /* Next Layer protocol */
  uint16_t  srcPort;       /* Layer 4 source port number */
  uint16_t  dstPort;       /* Layer 4 destination port number */  
  uint16_t  srcVC;         /* virtual or physical link */
  
} pafrmComL1Ipv4_t;

/* IPv6 specific valiod bit definitions */

#define PAFRM_LUT1_VALID_IPV6_SIP    0x000FFFF0  /* Care0: byte 4-19 */
#define PAFRM_LUT1_VALID_IPV6_DIP_0  0x00000F00  /* Care0: byte 20-*/

/* IPv6 specific valiod bit definitions */
#define PAFRM_LUT1_IPV6_SRC_PORT_RANGE    0x10000000 /* Care0: [29:28]  for Byte 42-43 */
#define PAFRM_LUT1_IPV6_DST_PORT_RANGE    0x04000000 /* Care0: [27:26]  for Byte 44-45 */
#define PAFRM_LUT1_IPV6_SUBNET_MASK       0x000000FF
#define PAFRM_LUT1_IPV6_SUBNET_SIP_SHIFT    17    /* Care0 [24:17] */
#define PAFRM_LUT1_IPV6_SUBNET_DIP_SHIFT     9    /* Care0 [16:9] */

#define PAFRM_LUT1_VALID_IPV6_PKTFLAGS    0x00000180 /* Care0: [8:7] Byte 0-1 */
#define PAFRM_LUT1_VALID_IPV6_DSCP        0x00000040 /* Care0: [6:6] Byte 2   */
#define PAFRM_LUT1_VALID_IPV6_FLOWLABEL   0x0000001E /* Care0: [4:1] Byte 36-39 */
#define PAFRM_LUT1_VALID_IPV6_PKTTYPE     0x00000001 /* Care0: [0:0] Byte 40    */
#define PAFRM_LUT1_VALID_IPV6_PROTO       0x80000000 /* Care1: [31:31] Byte 41    */
#define PAFRM_LUT1_VALID_IPV6_SRC_PORT    0x60000000 /* Care1: [30:29] Byte 42-43 */
#define PAFRM_LUT1_VALID_IPV6_DST_PORT    0x18000000 /* Care1: [28:27] Byte 44-45 */
#define PAFRM_LUT1_VALID_IPV6_SRC_VC      0x06000000 /* Care1: [26:25] Byte 46-47 */

typedef struct pafrmComL1Ipv6_s  {

  /* LUT1 view 1 */
  uint32_t  srcIp0;        /* Source IP address */
  uint32_t  srcIp1;
  uint32_t  srcIp2;
  uint32_t  srcIp3;
  
  /* LUT1 view 2 */
  uint32_t  dstIp0;        /* Destination IP address */   
  uint32_t  dstIp1;
  uint32_t  dstIp2;
  uint32_t  dstIp3;
  
  /* LUT1 view 3 */
  uint16_t  pktFlags;      /* Various packet flags */
#define PAFRM_IP_FLAG_V6           0x4000  
  uint8_t   dscp;  
  uint8_t   rsvd8a;      
  uint32_t  flowLabel;     /* 20-bit Flow Label in the header */  
  uint8_t   pktType;       /* Common filed to indicate packet type */
  uint8_t   protocol;      /* Next Layer protocol */
  uint16_t  srcPort;       /* Layer 4 source port number */
  uint16_t  dstPort;       /* Layer 4 destination port number */  
  uint16_t  srcVC;         /* virtual or physical link */
  
} pafrmComL1Ipv6_t;

/* IPSEC specific valiod bit definitions */

#define PAFRM_LUT1_VALID_IPSEC_PKTFLAGS    0x00000180 /* Care0: [8:7] Byte 0-1 */
#define PAFRM_LUT1_VALID_IPSEC_SPI         0x0000001E /* Care0: [4:1] Byte 36-39 */
#define PAFRM_LUT1_VALID_IPSEC_PKTTYPE     0x00000001 /* Care0: [0:0] Byte 40    */
#define PAFRM_LUT1_VALID_IPSEC_SRC_VC      0x0C000000 /* Care1: [26:25] Byte 46-47 */

typedef struct pafrmComL1Ipsec_s  {

  /* LUT1 view 1 */
  uint32_t  rsvd1_1;
  uint32_t  rsvd1_2;
  uint32_t  rsvd1_3;
  uint32_t  rsvd1_4;
  
  /* LUT1 view 2 */
  uint32_t  rsvd2_1;          
  uint32_t  rsvd2_2;
  uint32_t  rsvd2_3;
  uint32_t  rsvd2_4;
  
  /* LUT1 view 3 */
  uint16_t   pktFlags;      /* Various packet flags */
#define PAFRM_IPSEC_FLAG_ESP        0x8000  
#define PAFRM_IPSEC_FLAG_AH         0x4000  
  uint16_t  rsvd3_1;        
  uint32_t  spi;            /* SPI value */
  uint8_t   pktType;        /* Common filed to indicate packet type */
  uint8_t   rsvd3_2; 
  uint16_t  rsvd3_3;           
  uint16_t  rsvd3_4;         
  uint16_t  srcVC;          /* virtual or physical link */
  
} pafrmComL1Ipsec_t;


/* if PA_LUT1_INDEX_LAST_FREE is used then when the command returns, the value of index
 * will be replaced with the actual index used */
#define PAFRM_HW_LUT1_ENTRIES            256
#define PAFRM_LUT1_INDEX_LAST_FREE       PAFRM_HW_LUT1_ENTRIES

typedef struct pafrmCommandAddLut1_s  {

  uint16_t index;        /* LUT1 index. (ACL entry: stats index ) */
  uint8_t  type;         /* Custom or standard */
  uint8_t  custIndex;    /* Vaild only if type is custom  */
  uint16_t vLinkNum;     /* Virtual Link number if used (ACL Entry: pending lut1 index update offset for rescore) */
  uint16_t statsIndex;   /* entry statistics index (Flow Cache only) */
  
  /* LUT1 views */
  union  {
    pafrmComL1Mac_t       mac;     /* matching information for MAC entry */
    pafrmComL1Srio_t      srio;    /* matching information for SRIO entry */
    pafrmComL1Custom_t    custom;  /* matching information for custom LUT1 entry */
    pafrmComL1Ipv4_t      ipv4;    /* matching information for IPv4 entry*/
    pafrmComL1Ipv6_t      ipv6;    /* matching information for IPv6 entry */
    pafrmComL1Ipsec_t     ipsec;   /* matching information for IPSEC entry */
    
  } u;
  
  /* Command header */
  uint16_t range1Hi;     /* Range High for bytes 44-45 */
  uint16_t range0Hi;     /* Range High for bytes 42-43 */
  uint32_t CBWords0;     /* Care Bits Word0 */
  uint32_t CBWords1;     /* Care Bits Word1 */
  uint16_t bitMask;      /* BitMask for Bytes 0-1 */
  uint16_t priority;     /* Record priority "score", relative index */
  
  /* Routing information when a match is found */
  pafrmForward_t match;
  
  /* Routing information when subsequent match fails - a fragmented packet or
   * inner route. */
  pafrmForward_t nextFail;
  
} pafrmCommandAddLut1_t;

/* define LUT1 entry types */
#define PAFRM_COM_ADD_LUT1_STANDARD    0   /* MAC/IP/IPSEC/ACL/FC */
#define PAFRM_COM_ADD_LUT1_SRIO        1   /* SRIO */
#define PAFRM_COM_ADD_LUT1_CUSTOM      2   /* Custom LUT1 */
#define PAFRM_COM_ADD_LUT1_VLINK       3   /* Standard entry with virtual Link */

/* Delete entry from LUT1 */
typedef struct pafrmCommandDelLut1_s
{
  uint16_t  index;        /*  LUT1 index */
  uint16_t  rsvd;        
  
} pafrmCommandDelLut1_t;

/* Add an entry to LUT2 */
/*
 *  Format a 8-byte byte array in big-endian format
 * 
 *  TCP/UDP:     0:port_hi:port_lo:previous matching link info
 *  32-bit port: port_hi:port_mhi:port_mlo:port_lo:previous link info
 *  custom:      entry identifier or match0: match1: match2:mach3 or previous matching link info  
 *
 */
 
#define PAFRM_DEST_PORT_GTP              2152

#define PAFRM_LUT2_PKT_TYPE_PORT16       0x80
#define PAFRM_LUT2_PKT_TYPE_PORT32       0x40
#define PAFRM_LUT2_PKT_TYPE_CUSTOM       0x20
 
typedef struct pafrmComL2Standard_s {
  uint32_t  portNum;
} pafrmComL2Standard_t;

typedef struct pafrmComL2Custom_s
{
  uint8_t   match[4];
} pafrmComL2Custom_t;


/* Queue Divert information */
typedef struct pafrmComL2QueueDivert_s
{
  uint16_t  destQ;   /* Soutce Queue number */
  uint16_t  srcQ;    /* Destination Queue number */
  
} pafrmComL2QueueDivert_t;
    

typedef struct pafrmCommandAddLut2_s  {

  uint8_t  type;        /* Port16, port32 or custom */
  uint8_t  index;       /* LUT2 custom index if custom LUT2 */
  uint8_t  ctrlBitMap;  /* Control Bitmap      
                         * b0: replace entry
                         * b1: GTPU control
                         * b2-b7: reserved
                         */
#define PAFRM_COM_LUT2_CTRL_REPLACE         0x01   /* replace the entry: Do not increment the counter */  
#define PAFRM_COM_LUT2_CTRL_GTPU            0x02   /* Add: Disable GTPU parsing; Delete: Re-enable GTPU parsing */  
#define PAFRM_COM_LUT2_CTRL_QUEUE_DIVERT    0x04   /* Queue diversion: perform queue diversion */
#define PAFRM_COM_LUT2_CTRL_LINK            0x80   /* Upper linker is required (host only) */  
#define PAFRM_COM_LUT2_CTRL_PORT32          0x40   /* Port32 (host only) */                         
#define PAFRM_COM_LUT2_CTRL_VLINK           0x20   /* Virtual Link (host only) */  

/* Layer 2 packet Type */
#define PAFRM_L4_PKT_TYPE_PORT16            0x80
#define PAFRM_L4_PKT_TYPE_PORT32            0x40
#define PAFRM_L4_PKT_TYPE_CUSTOM            0x20
                         
  uint8_t   l4Type;             
                       
  uint16_t  inkTableIdx;       /* link table Index: host only */
  uint16_t  srcVC;             /* link to the previous matching */
  union  {
 
    pafrmComL2Standard_t  port;     /* standard (port16/32) matching information */
    pafrmComL2Custom_t    custom;   /* custom LUT2 matching information */
   
  } u;
  
  /* Routing information when a match is found */
  pafrmForward_t match;
  
  /* Queue Diversion information */
  pafrmComL2QueueDivert_t   qDivert;    /* Queue Divert information */
  
  
} pafrmCommandAddLut2_t;


/* define LUT2 entry types */
#define PAFRM_COM_ADD_LUT2_STANDARD    0
#define PAFRM_COM_ADD_LUT2_CUSTOM      1

/* Delete an entry in LUT2 */
typedef struct pafrmCommandDelLut2_s  {

  uint8_t  type;        /* Port16, port32 or custom */
  uint8_t  index;       /* LUT2 custom index if custom LUT2 */
  uint8_t  ctrlBitMap;  /* Control Bitmap      
                         * b0: reserved
                         * b1: GTPU control
                         * b1-b7: reserved
                         */
  uint8_t   l4Type;             
                       
  uint16_t  lnkTableIdx;   /* link table Index: host only */
  uint16_t  srcVC;         /* link to the previous matching */
  
  union  {
  
    pafrmComL2Standard_t port;    /* standard (port16/32) matching information */
    pafrmComL2Custom_t   custom;  /* custom LUT2 matching information */
    
  } u;

  
} pafrmCommandDelLut2_t;
  
  
/* define LUT2 entry types */
#define PAFRM_COM_DEL_LUT2_STANDARD   0
#define PAFRM_COM_DEL_LUT2_CUSTOM     1
  

#define PAFRM_STATS_TYPE_SYS         0    /* System Statistics  */
#define PAFRM_STATS_TYPE_USR         1    /* User Defined Statistics  */
 
/* Statistics Request */ 
typedef struct pafrmComReqStats_s
{
  uint8_t  ctrlBitMap; /* Control bit map as defined below */
  uint8_t  type;       /* Statistics Type */
  uint16_t numCnt;     /* number of counters to be cleared */
                       /* Stats clearBitMap follows */
} pafrmCommandReqStats_t;

#define PAFRM_STATS_CTRL_CLR         0x01 /* Clear statistics */
#define PAFRM_STATS_CTRL_REQ         0x02 /* Request Statistics, applicable to user-defined stats only */
                                          /* since PASS needs t return system statistics all the time */ 
                                          
typedef struct pafrmUsrStatsClrStats_s 
{
    uint32_t  bitMap[16];                 /* User-defined statistics clear bitmap corresponding to 512 bits */
} pafrmUsrStatsClrStats_t;           

/*
 * PASS stats request packet consists of the common command header, the stats request header and optional usr stats clear bitmaps.
 * PASS stats reply packet consists of the common command header, the stats request header, and then follows by the system stats
 * or the entire user-defined statistics as 32-bit word array in big-endian format. 
 * The optional user defined stats clear bitmaps may follow and should be ignored by LLD.
 */                             

/* CRC Engine Configuration */
#define PAFRM_CRC_TABLE_SIZE    16

typedef struct pafrmCommandConfigCRC_s
{
  uint8_t   ctrlBitMap;     /* Control bit maps as defined below */
#define PAFRM_CRC_SIZE_8         0
#define PAFRM_CRC_SIZE_16        1
#define PAFRM_CRC_SIZE_24        2
#define PAFRM_CRC_SIZE_32        3

#define PAFRM_CRC_CTRL_CRC_SIZE_MASK    0x3
#define PAFRM_CRC_CTRL_LEFT_SHIFT       0x0
#define PAFRM_CRC_CTRL_RIGHT_SHIFT      0x4
#define PAFRM_CRC_CTRL_INV_RESULT       0x8

  uint8_t   rsvd1;                          /* reserved for alignment */
  uint16_t  rsvd2;                          /* reserved for alignment */
  uint32_t  initVal;                        /* Initial value to use in the CRC calcualtion */
  uint32_t  crcTbl[PAFRM_CRC_TABLE_SIZE];   /* CRC tabls */

} pafrmCommandConfigCRC_t;

/* Command set Configuration */
#define PAFRM_MAX_CMD_SET_SIZE    124

typedef struct pafrmCommandCmdSet_s  {

  uint8_t   idx;      /* The index of the command set */
  uint8_t   numCmd;   /*  Number of commands */
  uint16_t  rsvd;     /* reserved for alignment */
  uint8_t   cmd[PAFRM_MAX_CMD_SET_SIZE];  /* commands  */ 
  
} pafrmCommandCmdSet_t;


/* User-defined Statistics counter entry */
typedef  struct pafrmUsrStatsEntry_s {
    uint16_t  index;      /* Index to the counter */ 
    uint16_t  lnkIndex;   /* Index to the next layer counter 
                           * b15: No link
                           * b14: 0: pkt counter
                           *      1: byte counter
                           * b13: 1: Disable
                           *      0: Enable
                           */       
} pafrmUsrStatsEntry_t;  

#define PAFRM_USR_STATS_LNK_END         PA_USR_STATS_LNK_END /* 0x8000 */
#define PAFRM_USR_STATS_BYTE_CNT        0x4000       
#define PAFRM_USR_STATS_DISABLE         0x2000       

/* User-defined Statistics counter configuration */
typedef struct pafrmUsrStatsCntCfg_s {
    uint8_t   ctrlFlags;     /* b0: reset */   
    uint8_t   rsvd;   
    uint16_t  nEntries;      /* Number of counter configuration blocks 1-256 */
}  pafrmUsrStatsCntCfg_t;  


#define PAFRM_USR_STATS_CFG_CLR_ALL      pa_USR_STATS_CONFIG_RESET 

/* multi-route entry  */
typedef struct pafrmMultiRouteEntry_s  {

  uint8_t  ctrlFlags;       /* b0:descriptor only;b7:active */
  uint8_t  flowId;          /* PKTDMA flow Id */
  uint16_t queue;           /* destination queue number */
  uint32_t swInfo0;         /* swInfo0 at CPPI descriptor */

} pafrmMultiRouteEntry_t;

/* Multi-route control flags */
#define PAFRM_MULTI_RUOTE_CTRL_DESC_ONLY   pa_NEXT_ROUTE_DESCRIPTOR_ONLY      /* Forward packet descriptor only (1 << 0) */
#define PAFRM_MULTI_RUOTE_CTRL_ACTIVE      (1<<7)


#define PAFRM_MAX_HOST_PKT_DUP             pa_MAX_MULTI_ROUTE_ENTRIES  /* 8 */
#define PAFRM_MULTI_ROUTE_NUM_ROUTES       pa_MAX_MULTI_ROUTE_SETS     /* 32 */
#define PAFRM_MULTI_ROUTE_NEXT_FREE_IDX    0xff

/* Configure PA multi-route set */
typedef struct pafrmCommandMultiRoute_s  {

  uint8_t   idx;      /* The index to read/write */
  uint8_t   mode;     /* Add/Delete or read */
  uint8_t   nRoutes;  /* Number of routes, 1 - PA_MAX_HOST_PKT_DUP    */ 
  uint8_t   rsvd;     /* reserved for alignment */
  
  pafrmMultiRouteEntry_t quFl[PAFRM_MAX_HOST_PKT_DUP];   /* The routes */
  
} pafrmCommandMultiRoute_t;

/* Multi-route operation definitions */
enum  {

  PAFRM_COMMAND_MULTI_ROUTE_MODE_ADD,
  PAFRM_COMMAND_MULTI_ROUTE_MODE_DEL,
  PAFRM_COMMAND_MULTI_ROUTE_MODE_GET
  
};

#define PAFRM_CFG_CMD_STATUS_PROC      0
#define PAFRM_CFG_CMD_STATUS_DONE      1

typedef struct pafrmCommandCmdHdr_s  {

  uint8_t   command;  /* Command Header of each command within the multiple command packet */
  uint8_t   offset;   /* Offset to the next command, 0: Indicate the last command */
  uint16_t  comId;    /* general parameter used by host only */
  
} pafrmCommandCmdHdr_t;


/* Commands to PA */
typedef struct pafrmCommand_s  {

  uint8_t   status;        /* Command Status (used by firmware only)                                    */
  uint8_t   pdspIndex;     /* index of the first targeted PDSP in a clsuter                             */
  uint16_t  commandResult; /* Returned to the host, ignored on entry to the PASS                        */

  uint16_t  comId;        /* MULTI_CMDS: used as offset to the next command to be executed              */
                          /* Otherwise: command ID used by LLD only                                     */ 
  uint8_t   command;      /* Command value                                                              */
  uint8_t   magic;        /* Magic value                                                                */
  uint32_t  retContext;   /* Returned in swInfo to identify packet as a command                         */
  uint16_t  replyQueue;   /* Specifies the queue number for the message reply. 0xffff to toss the reply */
  uint8_t   replyDest;    /* Reply destination (host0, host1, discard are the only valid values)        */
  uint8_t   flowId;       /* Flow ID used to assign packet at reply                                     */
  
  uint32_t  cmd;          /* First word of the command */
  
/*  
 * refernce of the valid PA command structures
 *
  union  {
  
    pafrmCommandAddLut1_t     addLut1;
    pafrmCommandDelLut1_t     delLut1;
    pafrmCommandAddLut2_t     addLut2;
    pafrmCommandDelLut2_t     delLut2;
    pafrmCommandConfigPa_t    paConfig;
    pafrmCommandReqStats_t    reqStats;
    pafrmCommandMultiRoute_t  multiRoute; 
    pafrmCommandConfigCRC_t   crcCfg;
    pafrmCommandCmdSet_t      cmdSet;
    pafrmCommandSysConfigPa_t sysConfig;
    pafrmCommandCmdHdr_t      cmdHdr;
  } u;
  
*/
  
  
} pafrmCommand_t;

/* Command values */
enum  {
  PAFRM_CONFIG_COMMAND_RSVD                 = 0,
  PAFRM_CONFIG_COMMAND_ADDREP_LUT1,
  PAFRM_CONFIG_COMMAND_DEL_LUT1,
  PAFRM_CONFIG_COMMAND_ADDREP_LUT2,
  PAFRM_CONFIG_COMMAND_DEL_LUT2,
  PAFRM_CONFIG_COMMAND_CONFIG_PA,
  PAFRM_CONFIG_COMMAND_REQ_STATS,
  PAFRM_CONFIG_COMMAND_REQ_VERSION,
  PAFRM_CONFIG_COMMAND_MULTI_ROUTE,
  PAFRM_CONFIG_COMMAND_CRC_ENGINE,
  PAFRM_CONFIG_COMMAND_CMD_SET,
  PAFRM_CONFIG_COMMAND_USR_STATS,
  PAFRM_CONFIG_COMMAND_SYS_CONFIG,
  PAFRM_CONFIG_COMMAND_MULTI_CMDS           = 100
};



/* Command magic value */
#define PAFRM_CONFIG_COMMAND_SEC_BYTE  0xce

/* Command return values */
enum  {

  PAFRM_COMMAND_RESULT_SUCCESS = 0,              /* Must be 0 */
  PAFRM_COMMAND_RESULT_NO_COMMAND_MAGIC,         /* Command magic value not found */
  
  PAFRM_COMMAND_RESULT_INVALID_CMD,              /* Invalid command identifier */
  
  /* Add entry to LUT1 fails */
  PAFRM_COMMAND_RESULT_LUT1_TYPE_INVALID,        /* Invalid type, custom or standard IP/ethernet */
  PAFRM_COMMAND_RESULT_LUT1_INDEX_INVALID,       /* Invalid LUT1 index (0-63) or no free indices available */
  PAFRM_COMMAND_RESULT_LUT1_MATCH_DEST_INVALID,  /* Sent a match packet to q0 on c1 or c2 - this is illegal. */
  PAFRM_COMMAND_RESULT_LUT1_NMATCH_INVALID,      /* Previous match forward info was somewhere in chunk domain */
  PAFRM_COMMAND_RESULT_LUT1_INVALID_KEYS,        /* Invalid combination found in the key value */
  
  /* Lut 2 entry warnings since the lut can be configured without pdsp */
  PAFRM_COMMAND_RESULT_WARN_OVER_MAX_ENTRIES,
  PAFRM_COMMAND_RESULT_WARN_NEGATIVE_ENTRY_COUNT,
  
  /* Lut 2 entry failures */
  PAFRM_COMMAND_RESULT_LUT2_ADD_BUSY,            /* LUT2 had a lookup and pending config */
  
  /* Not enough room in stats request packet for the reply */
  PAFRM_COMMAND_RESULT_WARN_STATS_REPLY_SIZE,
  
  /* Command sent to PDSP which couldn't handle it */
  PAFRM_COMMAND_RESULT_INVALID_DESTINATION,
  
  /* Add/Delete/Read entries to multi route table */
  PAFRM_COMMAND_RESULT_MULTI_ROUTE_NO_FREE_ENTRIES,    /* Asked to use a free entry, but none found */
  PAFRM_COMMAND_RESULT_MULTI_ROUTE_INVALID_IDX,        /* Illegal index value used */
  PAFRM_COMMAND_RESULT_MULTI_ROUTE_INVALID_MODE,       /* Illegal multi route mode used */
  
  /* Packet size didn't match command */
  PAFRM_COMMAND_RESULT_INVALID_PKT_SIZE,
  
  /* Coustom and Command set index */
  PAFRM_COMMAND_RESULT_INVALID_C1_CUSTOM_IDX,          /* Illegal Custom LUT1 index value used */
  PAFRM_COMMAND_RESULT_INVALID_C2_CUSTOM_IDX,          /* Illegal Custom LUT2 index value used */
  PAFRM_COMMAND_RESULT_INVALID_CMDSET_IDX,             /* Illegal Custom Command Set index value used */
  PAFRM_COMMAND_RESULT_USR_STATS_INVALID_CONFIG,       /* Illegal User Stats Configuration */

  /* LUT2 entry failure - is FULL */
  PAFRM_COMMAND_RESULT_LUT2_FULL                       /* LUT2 is full */
  
};  

/* Destination (route) values */
#define PAFRM_DEST_CDMA0              0           // Packets to Global CDMA
#define PAFRM_DEST_CDMA1              1           // Packets to Local CDMA
#define PAFRM_DEST_ETHERNET1          2           // Packets to Ethernet TX
#define PAFRM_DEST_ETHERNET2          3           // Packets to Ethernet TX
#define PAFRM_DEST_ETHERNET3          4           // Packets to Ethernet TX
#define PAFRM_DEST_ETHERNET4          5           // Packets to Ethernet TX
#define PAFRM_DEST_ETHERNET5          6           // Packets to Ethernet TX
#define PAFRM_DEST_ETHERNET6          7           // Packets to Ethernet TX
#define PAFRM_DEST_ETHERNET7          8           // Packets to Ethernet TX
#define PAFRM_DEST_ETHERNET8          9           // Packets to Ethernet TX
#define PAFRM_DEST_INGRESS0           10           // Packets to Cluster Ingress 0
#define PAFRM_DEST_INGRESS1           11           // Packets to Cluster Ingress 1
#define PAFRM_DEST_INGRESS2           12           // Packets to Cluster Ingress 2
#define PAFRM_DEST_INGRESS3           13           // Packets to Cluster Ingress 3
#define PAFRM_DEST_INGRESS4           14           // Packets to Cluster Ingress 4
#define PAFRM_DEST_POST               15           // Packets to Cluster Post Processing
#define PAFRM_DEST_EGRESS0            16           // Packets to Cluster Egress 0
#define PAFRM_DEST_EGRESS1            17          // Packets to Cluster Egress 1
#define PAFRM_DEST_EGRESS2            18          // Packets to Cluster Egress 2
#define PAFRM_DEST_REASM              19          // Packets to Reasm Accelerator
#define PAFRM_DEST_ACE0               20          // Placeholder for model
#define PAFRM_DEST_ACE1               21          // Placeholder for model
#define PAFRM_DEST_STATSBLOC          22          // Packets to Statsbloc

#define PAFRM_DEST_PKTDMA             PAFRM_DEST_CDMA0
#define PAFRM_DEST_PKTDMA_LOC         PAFRM_DEST_CDMA1
#define PAFRM_DEST_ETH                PAFRM_DEST_ETHERNET1
#define PAFRM_DEST_RA_DISCARD         2

#define PAFRM_CPSW_DEST_CDMA0              0      // Packets to Global CDMA from CPSW
#define PAFRM_CPSW_DEST_CDMA1              1      // Packets to Local CDMA from CPSW
#define PAFRM_CPSW_DEST_INGRESS0           2      // Packets to Cluster Ingress 0 from CPSW
#define PAFRM_CPSW_DEST_INGRESS1           3      // Packets to Cluster Ingress 1 from CPSW
#define PAFRM_CPSW_DEST_INGRESS2           4      // Packets to Cluster Ingress 2 from CPSW
#define PAFRM_CPSW_DEST_INGRESS3           5      // Packets to Cluster Ingress 3 from CPSW
#define PAFRM_CPSW_DEST_INGRESS4           6      // Packets to Cluster Ingress 4 from CPSW
#define PAFRM_CPSW_DEST_POST               7      // Packets to Cluster Post Processing from CPSW
#define PAFRM_CPSW_DEST_EGRESS0            8      // Packets to Cluster Egress 0 from CPSW
#define PAFRM_CPSW_DEST_EGRESS1            9      // Packets to Cluster Egress 1 from CPSW
#define PAFRM_CPSW_DEST_EGRESS2            10     // Packets to Cluster Egress 2 from CPSW


/*
 * The following definitions are only used by nextRuote command which conatins 3-bit destination field 
 *
 */
#define PAFRM_DEST_NR_ACE0        3
#define PAFRM_DEST_NR_ACE1        4 
#define PAFRM_DEST_NR_SRIO        7   /* Virtual number used by next route command only */


#define PAFRM_DEST_DISCARD     0xFF

/* Assigning names based on PDSP functions */
#define PAFRM_DEST_PA_C1_0         PAFRM_DEST_INGRESS0
#define PAFRM_DEST_PA_C1_1         PAFRM_DEST_INGRESS1
#define PAFRM_DEST_PA_C1_2         PAFRM_DEST_INGRESS4 
#define PAFRM_DEST_PA_C2           PAFRM_DEST_INGRESS4
#define PAFRM_DEST_PA_M_0          PAFRM_DEST_POST
#define PAFRM_DEST_PA_M_1          PAFRM_DEST_EGRESS2
                                   
/* The default queue for packets that arrive at the PA and don't match in
 * classify1 (right at init time) */
#define PAFRM_DEFAULT_INIT_Q   0x100

/* Ethertypes recognized by the firmware. */
#define PAFRM_ETHERTYPE_IP          0x0800
#define PAFRM_ETHERTYPE_IPV6        0x86dd
#define PAFRM_ETHERTYPE_VLAN        0x8100
#define PAFRM_ETHERTYPE_SPVLAN      0x88a8
#define PAFRM_ETHERTYPE_MPLS        0x8847
#define PAFRM_ETHERTYPE_MPLS_MULTI  0x8848

/* Next header type values  */
#define PAFRM_HDR_MAC               0
#define PAFRM_HDR_VLAN              1
#define PAFRM_HDR_MPLS              2
#define PAFRM_HDR_IPv4              3
#define PAFRM_HDR_IPv6              4
#define PAFRM_HDR_IPv6_EXT_HOP      5
#define PAFRM_HDR_IPv6_EXT_ROUTE    6
#define PAFRM_HDR_IPv6_EXT_FRAG     7
#define PAFRM_HDR_IPv6_EXT_DEST     8
#define PAFRM_HDR_GRE               9
#define PAFRM_HDR_ESP               10
#define PAFRM_HDR_ESP_DECODED       11
#define PAFRM_HDR_AUTH              12
#define PAFRM_HDR_CUSTOM_C1         13
#define PAFRM_HDR_PPPoE             14
#define PAFRM_HDR_SCTP              15
#define PAFRM_HDR_UNKNOWN           16
#define PAFRM_HDR_UDP               17
#define PAFRM_HDR_UDP_LITE          18
#define PAFRM_HDR_TCP               19
#define PAFRM_HDR_GTPU              20
#define PAFRM_HDR_ESP_DECODED_C2    21
#define PAFRM_HDR_CUSTOM_C2         22

/* Command related definitions */
#define PAFRM_CRC_FLAG_CRC_OFFSET_VALID        0x01
#define PAFRM_CRC_FLAG_CRC_OFFSET_FROM_DESC    0x02
#define PAFRM_CHKSUM_FALG_NEGATIVE             0x01


/* PAFRM receive commands related definitions */

/* 
 * There are the following two groups of PAFRM receive commands:
 * PAFRM short commands which can be used as part of the routing info 
 * PAFRM commands which can be used within a command set
 */
 
#define PAFRM_RX_CMD_NONE           0           /* Dummy command */

/* short commands */
#define PAFRM_RX_CMD_CMDSET             1       /* Execute a command set */
#define PAFRM_RX_CMD_INSERT             2       /* Insert up to two types at the current location */
#define PAFRM_RX_CMD_USR_STATS          3       /* Increment the specific user-statistics chain */
#define PAFRM_RX_CMD_CMDSET_USR_STATS   4       /* Increment User-defined Stats chain and  execute the command set */

/* command set commands */
#define PAFRM_RX_CMD_NEXT_ROUTE         11      /* Specify the next route */
#define PAFRM_RX_CMD_CRC_OP             12      /* CRC generation or verification */
#define PAFRM_RX_CMD_COPY_DATA          13      /* Copy data to the PS Info section */
#define PAFRM_RX_CMD_PATCH_DATA         14      /* Insert or pacth packet data at the specific location */
#define PAFRM_RX_CMD_REMOVE_HDR         15      /* Remove the parsed packet header */
#define PAFRM_RX_CMD_REMOVE_TAIL        16      /* Remove the parsed packet tail */
#define PAFRM_RX_CMD_MULTI_ROUTE        17      /* Duplicate packet to multiple destinations */
#define PAFRM_RX_CMD_VERIFY_PKT_ERROR   18      /* Verify packet error based on error flags */
#define PAFRM_RX_CMD_SPLIT              19       /* Payload splitting */


/* Rx command Header */
typedef struct
{
    uint8_t     cmd;      /* command code as defined above */
    uint8_t     len;      /* command total length */
    uint16_t    rsvd;     /* reserved for alignment */

} pafrmRxCmdHdr_t;


/* Rx command set */
typedef struct
{
    uint8_t     cmd;      /* PAFRM_RX_CMD_CMDSET */
    uint8_t     index;    /* command set index */

} pafrmRxCmdSet_t;

/* Rx insert command */
typedef struct
{
    uint8_t     cmd;        /* PAFRM_RX_CMD_INSERT */
    uint8_t     numBytes;   /* number of bytes to be inserted */
    uint8_t     data0;      /* data to be inserted */
    uint8_t     data1;      /* data to be inserted */
} pafrmRxCmdInsert_t;


/* Rx Usr Stats update command */
typedef struct
{
    uint8_t     cmd;      /* PAFRM_RX_CMD_USR_STATS */
    uint8_t     len;
    uint16_t    index;    /* user-statistics index */

} pafrmRxCmdUsrStats_t;

/*
 * Note: Both "Rx command set" and "Rx Usr Stats update" command  can be issued as part of the routing info.
 *       It is important because LUT1/LUT2 entries with different user-defined statistics can share the same command set
 *       out of the limited number of command sets.
 */
/* Rx command set and Usr Stats update command */
typedef struct
{
    uint8_t     cmd;        /* PAFRM_RX_CMD_CMDSET_USR_STATS */
    uint8_t     setIndex;   /* command set index */
    uint16_t    statsIndex; /* user-statistics index */

} pafrmRxCmdSetUsrStats_t;

/* 
 * Rx Next Route command
 * 
 * The SW Info should be present in the descriptor
 * The destination is always PKTDMA (Host, SA and ETH)
 *
 * Note: The routing to SRIO from command set is not required at this moment.
 *       This structure may need to be enhanced to support SRIO routing
 */       
typedef struct
{
    uint8_t    ctrlFlags;           /* b0: multi-route indication */
    uint8_t    multiRouteIndex;     /* multi-route index if required */
    uint8_t    psFlags;             /* psFlags for EMAC routing */
    uint8_t    rsvd;                /* reserved for alignment */
} pafrmRxCmdNextRoute_t;

#define PAFRM_RX_NEXT_ROUTE_CTRL_MULTI_ROUTE        0x01
#define PAFRM_RX_NEXT_ROUTE_CTRL_EMAC_ROUTE         0x02
#define PAFRM_RX_NEXT_ROUTE_CTRL_PSFLAGS_VALID      0x04

/* Rx CRC verification command */
typedef struct
{
    uint8_t   ctrlFlags;    /* CRC operation control information as defined below */
    uint8_t   lenAdjust;    /* Payload length adjustment */
    uint16_t  startOffset;  /* Byte location, from the protocol header, where the CRC computation begins */
    uint16_t  len;          /* Number of bytes covered by the CRC computation */
    uint8_t   crcSize;      /* size of CRC in bytes */
    uint8_t   lenOffset;    /* Payload length field offset in the header */
    uint16_t  lenMask;      /* Payload length field mask */
    uint16_t  crcOffset;    /* Offset from the protocol header to the CRC field */
    uint32_t  initVal;      /* Init value of CRC computation */
} pafrmRxCmdCrcOp_t;

#define PAFRM_RX_CRC_OP_CTRL_PAYLOAD_LENGTH_IN_HEADER        0x80
#define PAFRM_RX_CRC_OP_CTRL_CRC_FOLLOW_PAYLOAD              0x40
#define PAFRM_RX_CRC_OP_CTRL_FRAME_TYPE_INCLUDED             0x20
#define PAFRM_RX_CRC_OP_CTRL_LEN_OFFSET_NEGATIVE             0x10
#define PAFRM_RX_CRC_OP_CTRL_FRAME_TYPE_MASK                 0x0F

/*
 * WCDMA Iub HS-DSCH Type 2
 * offset = 6 + 2.5n       (n:even)
 *        = 6 + 2.5n + 0.5 (n:odd)
 */
 
#define PAFRM_RX_CRC_OP_CTRL_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2         0

/*
 * WCDMA Iub HS-DSCH Type 3
 * offset = 4 + 2.5n       (n:even)
 *        = 4 + 2.5n + 0.5 (n:odd)
 */

#define PAFRM_RX_CRC_OP_CTRL_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE3         1


/* Rx Split command */
typedef struct
{
    uint8_t   ctrlFlags;    /* Split operation control information as defined below */
    uint8_t   startOffset;  /* Byte location, from the protocol header, where the payload or frame begins */
    uint8_t   frameType;    /* WCDMA Frame type */
    uint8_t   rsvd1;        /* alignment */
    uint8_t   rsvd2;        /* place holder for header size */
    uint8_t   flowId;       /* CPPI flow which instructs how link-buffer queues are used for sending payload packets */
    uint16_t  destQueue;    /* Host queue for the payload packet */
    
} pafrmRxCmdSplitOp_t;

#define PAFRM_RX_SPLIT_OP_CTRL_FRAME_TYPE_INCLUDED           0x80

/* Rx Copy Command */
typedef struct {
    uint8_t  ctrlFlags;     /* Copy operation control information as defined below */
    uint8_t  srcOffset;     /* Offset from the start of current protocol header for the data copy to begin */
    uint8_t  destOffset;    /* Offset from the top of the PSInfo for the data to be copied to */
    uint8_t  numBytes;      /* Number of bytes to be copied */
} pafrmRxCmdCopy_t;

#define PAFRM_RX_COPY_CTRL_FROM_END                        0x01

/* Rx Patch Command */
typedef struct {
    uint8_t  ctrlFlags;     /* Patch operation control flags as defined below */
    uint8_t  offset;        /* Offset from the start of the current header for the patch to begin */
    uint8_t  numBypes;      /* number of bytes to be patched */
    uint8_t  rsvd;          /* reserved for alignment */
    uint32_t data;          /* First set of patch data */ 
} pafrmRxCmdPatch_t;

#define PAFRM_RX_PATCH_CTRL_INSERT         (uint8_t)pa_PATCH_OP_INSERT
#define PAFRM_RX_PATCH_CTRL_MAC_HDR        (uint8_t)pa_PATCH_OP_MAC_HDR
#define PAFRM_RX_PATCH_CTRL_DELETE         (uint8_t)pa_PATCH_OP_DELETE


/* Rx MultiRoute Command */
typedef struct
{
    uint8_t     cmd;      /* PAFRM_RX_CMD_MULTI_ROUTE */
    uint8_t     len;      /* command total length */
    uint8_t     index;    /* index of the multi-route set */
    uint8_t     rsvd;     /* reserved for alignment */

} pafrmRxCmdMultiRoute_t;

/* Rx Packet Error verification command  */
typedef struct
{
    uint8_t     errMask;     /* Error Flag mask */
    uint8_t     forwardType; /* Discard or Host */
    uint8_t     flowId;      /* PKTDMA flow Id */
    uint8_t     rsvd1;       /* reserved for alignment */
    uint16_t    queue;       /* destination queue number */ 
    uint16_t    rsvd2;       /* reserved for alignment */
    uint32_t    swInfo0;     /* swInfo0 at CPPI descriptor */
}  pafrmRxCmdVerifyPktErr_t;

#define PAFRM_RX_PKT_ERR_IP_CHECKSUM       0x08
#define PAFRM_RX_PKT_ERR_L4_CHECKSUM       0x04
#define PAFRM_RX_PKT_ERR_CRC               0x02


/* PASS Egress Flow record related definitions */
#define PA_EF_MAX_REC_INDEX          255

/* Egress Flow record 1 */
typedef struct
{
    uint16_t ctrlFlags;
    uint16_t mtu;                /* MTU size of inner IP */
    uint8_t  tos;                /* IPv4: TOS; Ipv6: Class */
    uint8_t  flowLablelHi;       /* upper 4-bit of IPV6 Flow Label  */
    uint16_t flowLablelLo;       /* lower 16-bit of IPV6 Flow Label */
    uint16_t srcPort;            /* TCP/UDP source port */
    uint16_t dstPort;            /* TCP/UDP destination port */
    uint32_t rsvd1;              /* alignment and future enhancements */
    uint8_t  srcIp[16];
    uint8_t  dstIp[16];
}   paFrmEfRec1_t;

#define PAFRM_EF_REC1_SIZE             48

#define PAFRM_EF_REC_VALID             0x8000


#define PAFRM_EF_REC1_FLAG_IP_SRC_ADDR         0x0001
#define PAFRM_EF_REC1_FLAG_IP_DST_ADDR         0x0002
#define PAFRM_EF_REC1_FLAG_IP_FLOW_LABEL       0x0004
#define PAFRM_EF_REC1_FLAG_IP_TOS_CLASS        0x0008
#define PAFRM_EF_REC1_FLAG_IP_MTU              0x0010    /* IP fragmentation */
#define PAFRM_EF_REC1_FLAG_L4_SRC_PORT         0x0020
#define PAFRM_EF_REC1_FLAG_L4_DST_PORT         0x0040
#define PAFRM_EF_REC1_FLAG_IPV4_CKSUM          0x0080
#define PAFRM_EF_REC1_FLAG_L4_CKSUM            0x0100
#define PAFRM_EF_REC1_FLAG_IP_TTL_UPDATE       0x0200
#define PAFRM_EF_REC1_FLAG_STRIP_OUTER_IP      0x0400
#define PAFRM_EF_REC1_FLAG_EXP_TCP_CTRL        0x0800
#define PAFRM_EF_REC1_FLAG_EXP_IP_OPTION       0x1000
#define PAFRM_EF_REC1_FLAG_EXP_IP_FRAGMENT     0x2000
#define PAFRM_EF_REC1_FLAG_EXP_IP_EXPIRE       0x4000
#define PAFRM_EF_REC1_VALID                    PAFRM_EF_REC_VALID

/* Egress Flow record 2 */
typedef struct
{
    uint16_t ctrlFlags;
    uint16_t mtu;                /* MTU size of inner IP */
    uint8_t  l3HdrSize;          /* L3 Header size in bytes */
    uint8_t  encBlkSize;         /* Encryption block size: 1, 4 and 16 */
    uint8_t  ivSize;             /* Initialization vector size */
    uint8_t  icvSize;            /* Authentication tag size in bytes */  
    uint8_t  rsvd;               /* Alignment */
    uint8_t  flowId;             /* CPPI flow Id or destination thread Id */
    uint16_t queueId;            /* Destination queue Id */
    uint32_t spi;                /* IPSEC SPI value */
    uint32_t swInfo0;            /* Software Info word 0 */
    uint32_t swInfo1;            /* Software Info word 1 */
    uint8_t  l3Hdr[pa_MAX_EF_REC_IP_HDR_LEN];     /* L3 Header */
}  paFrmEfRec2_t;

#define PAFRM_EF_REC2_SIZE             64

#define PAFRM_EF_REC2_FLAG_STRIP_OUTER_IP       0x0001    /* Not used */
#define PAFRM_EF_REC2_FLAG_SINGLE_IP            0x0002
#define PAFRM_EF_REC2_FLAG_IPSEC_PROC           0x0004
#define PAFRM_EF_REC2_FLAG_IPSEC_AH             0x0008
#define PAFRM_EF_REC2_FLAG_INS_IPSEC_HDR        0x0010    /* Insert IPSEC ESP header and IPSEC AH header  */
#define PAFRM_EF_REC2_FLAG_INS_ESP_TRAIL        0x0020
#define PAFRM_EF_REC2_FLAG_IP_TTL_UPDATE        0x0040
#define PAFRM_EF_REC2_FLAG_IP_MTU               0x0080    /* Outer IP fragmentation */
#define PAFRM_EF_REC2_FLAG_LOC_DMA              0x0100    /* Use local DMA */
#define PAFRM_EF_REC2_FLAG_EXP_IP_OPTION        0x1000
#define PAFRM_EF_REC2_FLAG_EXP_IP_FRAGMENT      0x2000
#define PAFRM_EF_REC2_FLAG_EXP_IP_EXPIRE        0x4000
#define PAFRM_EF_REC2_VALID                     PAFRM_EF_REC_VALID


/* Egress Flow record 3 */
typedef struct
{
    uint16_t ctrlFlags;
    uint16_t mtu;                /* MTU size of inner IP */
    uint16_t srcPort;            /* UDP source port */
    uint16_t dstPort;            /* UDP destination port */
} paFrmEfRec3Nat_t;      

typedef struct
{
    uint16_t ctrlFlags;
    uint16_t mtu;                /* MTU size of outer IP */
    uint16_t rsvd1;              /* Alignment */
    uint8_t  ivSize;             /* Initialization vector size in bytes */
    uint8_t  icvSize;            /* Authentication tag size in bytes */  
    uint8_t  rsvd2;              /* Aligment */
    uint8_t  flowId;             /* CPPI flow Id or destination thread Id */
    uint16_t queueId;            /* Destination queue Id */
    uint32_t spi;                /* IPSEC SPI value */
    uint32_t swInfo0;            /* Software Info word 0 */
    uint32_t swInfo1;            /* Software Info word 1 */
} paFrmEfRec3Ah_t;      

#define PAFRM_EF_REC3_SIZE             32 

#define PAFRM_EF_REC3_FLAG_IPSEC_AH             0x0001    /* 1: IPSEC AH, 0: IPSEC ESP NAT-T */
#define PAFRM_EF_REC3_FLAG_REP_HDR              0x0002    /* 1: replace AH or NAT-T header */
#define PAFRM_EF_REC3_FLAG_IP_MTU               0x0004    /* Outer IP fragmentation */
#define PAFRM_EF_REC3_FLAG_LOC_DMA              0x0008    /* Use local DMA */
#define PAFRM_EF_REC3_VALID                     PAFRM_EF_REC_VALID
                            
/* Egress Flow record 4 */
typedef struct
{
    uint16_t ctrlFlags;
    uint8_t  l2HdrSize;          /* L2 Header size in bytes */
    uint8_t  l2LenOffset;        /* Offset to L2 (802.3) length field */
    uint8_t  pppoeOffset;        /* Offset to PPPoE header */
    uint8_t  flowId;             /* CPPI flow Id or destination thread Id */
    uint16_t queueId;            /* Destination queue Id */
    uint8_t  minPktSize;         /* Minimum L2 packet size */
    uint8_t  pktType_psFlags;    /* SRIO:packet Type; EMAC:psFlags */
    uint8_t  destType;           /* Destination type: Host, EMAC or SRIO */
    uint8_t  priority;           /* EMAC port priority */
    uint16_t vlan1;              /* inner VLAN (0x8100) */
    uint16_t vlan2;              /* outer VLAN (QinQ) */
    uint32_t swInfo0;            /* Software Info word 0 or Srio PS Info0 */
    uint32_t swInfo1;            /* Software Info word 1 or Srio PS Info0 */
    uint8_t  l2Hdr[pa_MAX_EF_REC_L2_HDR_LEN];          
} paFrmEfRec4_t;      
#define PAFRM_EF_REC4_SIZE             64

#define PAFRM_EF_REC4_FLAG_STRIP_L2_HDR         0x0001
#define PAFRM_EF_REC4_FLAG_802_3                0x0002
#define PAFRM_EF_REC4_FLAG_PPPoE                0x0004
#define PAFRM_EF_REC4_FLAG_VLAN1                0x0008
#define PAFRM_EF_REC4_FLAG_VLAN2                0x0010    
#define PAFRM_EF_REC4_FLAG_TX_PADDING           0x0020    
#define PAFRM_EF_REC4_ROUTE_PRIORITY_DSCP       0x0040
#define PAFRM_EF_REC4_ROUTE_PRIORITY_VLAN       0x0080

#define PAFRM_EF_REC4_VALID                     PAFRM_EF_REC_VALID

#endif /* _PAFRM_H */




