/*
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#ifndef TEST14PKTS_H_
#define TEST14PKTS_H_

/* Commands to the PA are verified through the value in swinfo0.
 * The 16 ms bits are used as verification, the 16 lbs are for local handle id */
#define T14_CMD_SWINFO0_ADD_MAC_ID      0x11100000  /* Identifies add mac command */
#define T14_CMD_SWINFO0_DEL_MAC_ID      0x11110000  /* Identifies del mac command */
#define T14_CMD_SWINFO0_ADD_IP_ID       0x22200000  /* Identifies the add IP command */
#define T14_CMD_SWINFO0_DEL_IP_ID       0x22210000  /* Identifies the del IP command */
#define T14_CMD_SWINFO0_ADD_PORT_ID     0x22220000  /* Identifies the add port command */
#define T14_CMD_SWINFO0_DEL_PORT_ID     0x22230000  /* Identifies the del port command */
#define T14_CMD_SWINFO0_ADD_ACL_ID      0x22240000  /* Identifies the add ACL command */
#define T14_CMD_SWINFO0_DEL_ACL_ID      0x22250000  /* Identifies the add ACL command */
#define T14_CMD_SWINFO0_ADD_EOAM_ID     0x22260000  /* Identifies the add Eoam command */
#define T14_CMD_SWINFO0_DEL_EOAM_ID     0x22270000  /* Identifies the add Eoam command */
#define T14_CMD_SWINFO0_STATS_REQ_ID    0x33330000  /* Identifies the req stats command */
#define T14_CMD_SWINFO0_CRC_CFG_ID      0x44400000  /* Identifies the CRC config command */
#define T14_CMD_SWINFO0_GLOBAL_CFG_ID   0x44410000  /* Identifies the Global Sys config command */
#define T14_CMD_SWINFO0_TIMEOFFSET_ID   0x44420000  /* Identifies the Global Time offset Command */
#define T14_CMD_SWINFO0_PKT_ID          0x55550000  /* Identifies the packet as a data packet */
#define T14_CMD_SWINFO0_CAP_PKT_ID   	  0x55560000  /* Identifies the packet as a L2 captured data packet */
#define T14_CMD_SWINFO0_USR_STATS_CFG_ID  0x66660000  /* User Stats command */
#define T14_CMD_SWINFO0_PKT_ID1           0x77770000  /* Identifies the packet as a data packet, but from next fail route of Outer IP classfication */
#define T14_CMD_SWINFO0_PKT_ID2           0x88880000  /* Identifies the packet as a data packet, but from next fail route of Inner IP classfication */
#define T14_CMD_SWINFO0_NAT_T_CFG_ID      0x99990000  /* Identifies the NAT-T config command */
#define T14_CMD_SWINFO0_EOAM_PKT_ID       0x0E0A0000  /* EOAM match packet */

#define T14_CMD_SWINFO0_TYPE_MASK         0xffff0000  /* Mask for the command type */
#define T14_CMD_SWINFO0_ID_MASK           0x0000ffff  /* Mask for the local ID */

/* packet test information */
#define T14_MAX_PKTS_PER_ID       10
#define T14_MAX_ENTRIES_PER_LUT   20
#define T14_IF_PKT_INDEX          50

/* Define the bases for all the types which are sending packets to host */
#define T14_L2_NFAIL_BASE           0
#define T14_EOAM_MATCH_BASE       ( T14_L2_NFAIL_BASE   + ( T14_MAX_PKTS_PER_ID * T14_MAX_ENTRIES_PER_LUT ) )
#define T14_L4_MATCH_BASE         ( T14_EOAM_MATCH_BASE + ( T14_MAX_PKTS_PER_ID * T14_MAX_ENTRIES_PER_LUT ) )

#define T14_GET_UNIQUE_IDX(base, lutEntryIndex, pktIndex)   ( ( (lutEntryIndex * T14_MAX_PKTS_PER_ID) + pktIndex ) + base )

/* Bit map definitions for EOAM Packet Test */
#define T14_EOAM_NEED_RECORDS          (1 << 0)
#define T14_PKT_DISCARD                (1 << 1)
#define T14_EOAM_COUNT_VALID           (1 << 2)
#define T14_PKT_MATCH                  (1 << 3)
#define T14_PKT_SENT                   (1 << 4)
#define T14_PKT_RECEIVED               (1 << 5)
#define T14_PKT_TYPE_IPV6              (1 << 6)
#define T14_PKT_TYPE_IPV4              (1 << 7)

//#define T14_NFAIL_Q_EXPECTED           4
//#define T14_EOAM_MATCH_Q_EXPECTED      8

/* Valid rx MAC addresses used during the test */
typedef struct t14EthSetup_s  {
    paEthInfo_t  ethInfo;       /* PA Ethernet configuration structure */
    Bool         acked;         /* Set to TRUE when the reply to the command is received */
} t14EthSetup_t;

#define T14_MAC_NEXT_ROUTE_HOST      0
#define T14_MAC_NEXT_ROUTE_LUT1      1
#define T14_MAC_NEXT_ROUTE_LUT2      2
#define T14_MAC_NEXT_ROUTE_L2CAP     5

#ifdef _TMS320C6X
#pragma DATA_SECTION (t14EthSetup, ".testPkts")
#endif
static t14EthSetup_t t14EthSetup[] =  {

    {
      {
        //pa_ETH_INFO_VALID_DST,                       /* Valid Bit map */
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    /* Source mac is dont care */
          { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Dest mac */
        0,      /* Vlan      */
        0,      /* ethertype */
        0,        /* mpls tag  */
        0      /* input EMAC port */
      },
      FALSE
    },

      {
        {
          //pa_ETH_INFO_VALID_DST,                       /* Valid Bit map */
          { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
          { 0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09 },      /* Dest mac */
          0,      /* Vlan      */
          0,      /* ethertype */
          0,    /* mpls tag  */
          0       /* input EMAC port */
        },
        FALSE
      },

    {
      {
        //pa_ETH_INFO_VALID_DST,                       /* Valid Bit map */
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
            { 0x00, 0x01, 0x02, 0x03, 0x04, 0xbb },      /* Dest mac */
        0,      /* Vlan      */
        0,      /* ethertype */
        0,      /* mpls tag  */
        0      /* input EMAC port */
      },
      FALSE
    },

    {
      {
        //pa_ETH_INFO_VALID_DST,                       /* Valid Bit map */
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
          { 0x00, 0x01, 0x02, 0x03, 0x04, 0xcc },      /* Dest mac */
        0,      /* Vlan      */
        0,      /* ethertype */
        0,      /* mpls tag  */
        0      /* input EMAC port */
      },
      FALSE
    },

     {
      {
        //pa_ETH_INFO_VALID_DST,                       /* Valid Bit map */
          { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
          { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x32 },      /* Dest mac */
          0,      /* Vlan      */
          0,      /* ethertype */
          0,        /* mpls tag  */
        0     /* input EMAC port */
      },
      FALSE
     },
     {
      {
        //pa_ETH_INFO_VALID_DST,                       /* Valid Bit map */
          { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
          { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e },      /* Dest mac */
          0,      /* Vlan      */
          0,      /* ethertype */
          0,        /* mpls tag  */
        0     /* input EMAC port */
      },
      FALSE
     },
    /* Pa_addMac2() entries */
    {
      {
    	  { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x04 },      /* Source mac */
   	 	  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Dest mac is dont care */
    	  0,      /* Vlan      */
    	  0,      /* ethertype */
    	  0,		  /* mpls tag  */
        0       /* input EMAC port */
      },
      FALSE
    }
};

#define T14_IP_NEXT_ROUTE_HOST      0
#define T14_IP_NEXT_ROUTE_LUT1      1
#define T14_IP_NEXT_ROUTE_LUT2      2
#define T14_IP_NEXT_ROUTE_HOST_ESP  3         /* simulate SA ESP offset fixup operation */
#define T14_IP_NEXT_ROUTE_IPSEC2    4

typedef struct t14IpSetup_s  {
    Bool       isPrevLinkL3;/* TRUE: Prev link is L3 Otherwise L2 */
    int        lutInst;     /* Specify which LUT1 (0-2) should be used */
    int        nextRoute;   /* Next route */
    int        lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
    paIpInfo_t ipInfo;      /* PA IP configuration structure */
    Bool       acked;       /* Set to TRUE when the reply to the command is received */

} t14IpSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t14IpSetup, ".testPkts")
#endif
static t14IpSetup_t  t14IpSetup[] = {
    /* IP Entry 0 */
    {
          FALSE,  /* Prev Link L3 */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
      1,  /* Linked to dest mac index 1 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 1 */
    {
          FALSE,      /* Prev Link L3 */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
          0,  /* Linked to dest mac index 0 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 2 */
    {
          FALSE,    /* Prev Link L3 */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
          1,      /* Linked to dest mac index 1 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 3 */
    {
          FALSE,   /* Prev Link L3 */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
          1,      /* Linked to dest mac index 1 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x38 },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 4 */
    {
          FALSE,   /* Prev Link L3 */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT1 */
          0,      /* Linked to dest mac index 0 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x39 },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 5 */
    {
          TRUE,    /* Prev Link L3 */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
          14,      /* Linked to outer IP index 14 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x3a },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 6 (IP/AH) */
    {
          FALSE,   /* Prev Link L3 */
          pa_LUT1_INST_1,                                 /* LUT1 Instance (LUT1_1) */
          T14_IP_NEXT_ROUTE_LUT1,  /* Next Route LUT1 */
      0,  /* Linked to dest mac index 0 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0x44440000, /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0x33,   /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 7 (IP/ESP) */
    {
          FALSE,   /* Prev Link L3 */
          pa_LUT1_INST_1,                                 /* LUT1 Instance (LUT1_1) */
          T14_IP_NEXT_ROUTE_HOST_ESP,  /* Next Route Host (ESP)*/
          1,      /* Linked to dest mac index 1 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x3b },  /* IP Destination address */
        0x55550001, /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0x32,     /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 8 (IP/ESP) */
    {
          FALSE,   /* Prev Link L3 */
          pa_LUT1_INST_1,                                 /* LUT1 Instance (LUT1_1) */
          T14_IP_NEXT_ROUTE_HOST_ESP,  /* Next Route: Host (ESP)*/
          0,  /* Linked to dest mac index 0 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 104, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0x55550002, /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0x32,     /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 9 (IP/NAT-T) */
    {
          FALSE,   /* Prev Link L3 */
          pa_LUT1_INST_1,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT1,  /* Next Route LUT1 */
          0,  /* Linked to dest mac index 0 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        17,     /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 10 (Iink to IP/ESP */
    {
          TRUE,   /* Inner IP */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
          8,      /* Linked to outer IP  index 8 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 106, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0,        /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 11 (ESP link to IP/NAT-T)*/
    {
          TRUE,   /* Outer IP */
          pa_LUT1_INST_1,                      /* LUT1 Instance */
          T14_IP_NEXT_ROUTE_HOST,  /* Next Route: HOST */
          9,      /* Linked to IP entry 9 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0x55550003, /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 12 (Iink to IP/ESP */
    {
          TRUE,   /* Inner IP */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
          11,     /* Linked to outer IP  index 11 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0,        /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },


    /* IP Entry 13 (IP/AH) */
    {
          FALSE,  /* Outer IP */
          pa_LUT1_INST_1,                                 /* LUT1 Instance (LUT1_1) */
          T14_IP_NEXT_ROUTE_LUT1,  /* Next Route LUT1 */
          0,      /* Linked to dest mac index 0 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x3C },  /* IP Destination address */
        0x44440001, /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0x33,   /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 14 (IP/AH/ESP) */
    {
          FALSE,  /* Outer IP */
          pa_LUT1_INST_1,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_IPSEC2,  /* Next Route: next IPSEC stage  */
          0,      /* Linked to dest mac index 0 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x3D },  /* IP Destination address */
        0x44440002, /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0x32,   /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },


    /* IP Entry 15 (Link to IP/AH (0x44440001) */
    {
          TRUE,   /* Inner IP */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
          13,     /* Linked to outer IP index 4 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x3e },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 16 (ESP:Link to IP/AH (0x44440002) */
    {
          TRUE,   /* Inner IP */
          pa_LUT1_INST_1,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_HOST_ESP,  /* Next Route HOST (ESP) */
          14,     /* Linked to outer IP index 4 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0x55550002, /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 17 (ESP:Link to IP/AH/ESP */
    {
          TRUE,   /* Inner IP */
          pa_LUT1_INST_1,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT1,  /* Next Route LUT1 */
          16,     /* Linked to outer IP index 4 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x3a },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 18 (IP forwarding 1) */
    {
          FALSE,     /* Outer IP */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT1 */
          6,  /* Linked to dest mac index 6 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 108, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 19 (IP forwarding 2) */
    {
          TRUE,   /* Inner IP */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_2) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route LUT2 */
          18, /* Linked to outer IP  index 1 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 200, 201, 202, 109, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
        0,      /* SPI */
        0,      /* Flow */
        pa_IPV4,  /* IP Type */
        0,      /* GRE Protocol */
        0,      /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },

    /* IP Entry 20 (IPV6 Cipher Packet (IP/ESP)) */
    {
          TRUE,  /* Inner IP */
          pa_LUT1_INST_2,                                 /* LUT1 Instance (LUT1_1) */
          T14_IP_NEXT_ROUTE_LUT2,  /* Next Route Host (ESP)*/
          7,      /* Linked to dest IP entry 7 */
      { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
        { 0x00, 0x02, 0x04, 0x08,
          0x10, 0x12, 0x14, 0x18,
          0x20, 0x22, 0x24, 0x28,
          0x30, 0x32, 0x34, 0x40 },  /* IP Destination address */
        0, /* SPI */
        0,      /* Flow */
        pa_IPV6,  /* IP Type */
        0,      /* GRE Protocol */
        0,     /* Protocol */
        0,      /* TOS */
        0,      /* TOS Care flag */
              0           /* SCTP port */
      },
      FALSE
    },
  };

typedef struct t14UdpSetup_s  {
    int        lHandleIdx;  /* Linked handle (to previous L3 layer) */
    uint16_t   port;        /* destination port number */
    Bool       acked;       /* Set to TRUE when the reply to the command is received */
} t14UdpSetup_t;

#define T14_UDP_NEXT_ROUTE_HOST      0

#ifdef _TMS320C6X
#pragma DATA_SECTION(t14UdpSetup, ".testPkts")
#endif
static t14UdpSetup_t  t14UdpSetup[] = {
    /* UDP Entry 0 */
    {
        0,      /* Linked to dest ip index 0 */
        0x0555, /* destination port number */
        FALSE
    },

    /* UDP Entry 1 */
    {
        4,      /* Linked to dest ip index 4 */
        0x8102, /* destination port number */
        FALSE
    },
    /* UDP Entry 2 */
    {
        2,      /* Linked to dest ip index 2 */
        0x9000, /* destination port number */
        FALSE
    },

    /* UDP Entry 3 */
    {
        20,      /* Linked to dest ip index 20 */
        0x9002, /* destination port number */
        FALSE
    },

    /* UDP Entry 4 */
    {
        11,      /* Linked to dest ip index 9 (NAT-T) via SPI link (entry 11) */
        0x1194, /* destination port number */
        FALSE
    },

    /* UDP Entry 5 */
    {
        18,      /* Linked to dest ip index 18 */
        0x0666, /* destination port number */
        FALSE
    },

    /* UDP Entry 6 */
    {
        13,      /* Linked to dest ip index 13 */
        0x0888, /* destination port number */
        FALSE
    },

    /* UDP Entry 7 */
    {
        5,      /* Linked to dest ip index 5 */
        0x9102, /* destination port number */
        FALSE
    },

    /* UDP Entry 8 */
    {
        6,      /* Linked to dest ip index 6 */
        0x0777, /* destination port number */
        FALSE
    },

    /* UDP Entry 9 */
    {
        7,      /* Linked to dest ip index 7 */
        0xa002, /* destination port number */
        FALSE
    },

    /* UDP Entry 10 */
    {
        10,     /* Linked to dest ip index 10 */
        0xb000, /* destination port number */
        FALSE
    },

    /* UDP Entry 11 */
    {
        15,     /* Linked to dest ip index 15 */
        0xb002, /* destination port number */
        FALSE
    },

    /* UDP Entry 12 */
    {
        12,     /* Linked to dest ip index 12 */
        0xc000, /* destination port number */
        FALSE
    },

    /* UDP Entry 13 */
    {
        17,     /* Linked to dest ip index 17 */
        0xc002, /* destination port number */
        FALSE
    },

    /* UDP Entry 14 */
    {
        0,      /* Linked to dest ip index 0 */
        0x8004, /* destination port number */
        FALSE
    },


    /* UDP Entry 15 */
    {
        2,      /* Linked to dest ip index 2 */
        0x9004, /* destination port number */
        FALSE
    },

    /* UDP Entry 16 */
    {
        2,      /* Linked to dest ip index 2 */
        0x9006, /* destination port number */
        FALSE
    },

    /* UDP Entry 17 */
    {
        1,      /* Linked to dest ip index 1 */
        0x9008, /* destination port number */
        FALSE
    },
    /* UDP Entry 18 */
    {
        8,      /* Linked to dest ip index 2 */
        0xa200, /* destination port number */
        FALSE
    },
};

typedef struct t14EoamPaSetup_s  {
    int      seqId;         /* Sequential enumeration to identify */
    paEoamFlowInfo_t eoamInfo;  /* PA EOAM configuration structure */
  paEthInfo2_t  ethInfo; /* EOAM Eth Info */
    paReturn_t ret;           /* Expected return code from Pa_addEoamFlow command */
    Bool       acked;           /* Set to TRUE when the reply to the command is received */
} t14EoamPaSetup_t;

/* Ethernet OAM configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t14EoamInfo, ".testPkts")
#endif
static t14EoamPaSetup_t t14EoamInfo[] =  {

    /* ------- Entry 0 ----------- */
    {
        0,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                            /* flowId (would be filled up later in the test code) */
            0,                            /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 0,                    /* SwInfo0 */
            0,                            /* stats index (would be filled up later in the test code) */
            2                              /* meg level */
        },
        /* EOAM eth info */
    {
        pa_ETH_INFO_VALID_DST,                       /* Valid Bit Map */
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
        { 0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09 },      /* Dest mac */
        0,                                           /* Vlan      */
        0,                                           /* ethertype */
        0,                                             /* mpls tag  */
      0,                                           /* input EMAC port */
      0                                            /* Vlan Pri */
    },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* ------- Entry 1 ----------- */
    {
        1,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 1,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_DST,                       /* Valid Bit Map */
            { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
            { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e },      /* Dest mac */
            0,                                           /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            0                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },
#if 1
    /* Entry 2 */
    {
        2,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 2,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_VLAN_PRI,                   /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x00 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x00 },      /* Dest mac */
            0,                                           /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            1                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 3 */
    {
        3,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 3,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_VLAN,                     /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x03 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x03 },      /* Dest mac */
            0x234,                                       /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            1                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 4 */
    {
        4,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 4,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_VLAN     |
            pa_ETH_INFO_VALID_VLAN_PRI,                  /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x04 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x04 },      /* Dest mac */
            0x456,                                       /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            2                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 5 */
    {
        5,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 5,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_DST        |
            pa_ETH_INFO_VALID_VLAN_PRI,                  /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x05 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x05 },      /* Dest mac */
            0,                                           /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            3                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* Entry 6 */
    {
        6,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 6,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_DST |
            pa_ETH_INFO_VALID_VLAN,                      /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x06 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x06 },      /* Dest mac */
            0x438,                                       /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            1                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 7 */
    {
        7,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 7,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_DST       |
            pa_ETH_INFO_VALID_VLAN      |
            pa_ETH_INFO_VALID_VLAN_PRI,                  /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x07 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x07 },      /* Dest mac */
            0x142,                                       /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            4                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 8 */
    {
        8,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 8,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_SRC,                       /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x08 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x08 },      /* Dest mac */
            0,                                           /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            1                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 9 */
    {
        9,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 9,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_SRC      |
            pa_ETH_INFO_VALID_VLAN_PRI,                  /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x09 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x09 },      /* Dest mac */
            0,                                           /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            1                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 10 */
    {
        10,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 10,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_SRC   |
            pa_ETH_INFO_VALID_VLAN,                      /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x10 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x10 },      /* Dest mac */
            0x723,                                       /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            1                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 11 */
    {
        11,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 11,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_VLAN     |
            pa_ETH_INFO_VALID_SRC      |
            pa_ETH_INFO_VALID_VLAN_PRI,                  /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x11 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x11 },      /* Dest mac */
            0x7a1,                                       /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            6                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },


    /* Entry 12 */
    {
        12,          /* Sequential ID */
        /* Eoam Flow */
        {
            0,                             /* validbitMap */
            0,                             /* flowId (would be filled up later in the test code) */
            0,                             /* Queue (would be filled up later in the test code)  */
            T14_CMD_SWINFO0_EOAM_PKT_ID + 12,  /* SwInfo0 */
            0,                              /* stats index (would be filled up later in the test code) */
            2                               /* meg level */
        },
        /* EOAM eth info */
        {
            pa_ETH_INFO_VALID_VLAN |
            pa_ETH_INFO_VALID_DST  |
            pa_ETH_INFO_VALID_SRC  |
            pa_ETH_INFO_VALID_VLAN_PRI,                  /* Valid Bit Map */
            { 0x0e, 0xa0, 0x01, 0x02, 0x03, 0x12 },      /* Source mac is dont care */
            { 0xe0, 0x0a, 0x10, 0x20, 0x30, 0x12 },      /* Dest mac */
            0x786,                                       /* Vlan      */
            0,                                           /* ethertype */
            0,                                           /* mpls tag  */
            0,                                           /* input EMAC port */
            7                                            /* Vlan Pri */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },
#endif
};


typedef struct t14AclPaSetup_s  {
  int      seqId;       /* Sequential enumeration to identify */
  int          handleIdx;   /* Local handle index. Specifies which local handle corresponds to this entry */
  int      aclInst;     /* Specify which ACL LUT1 (0-1) should be used */
  int          lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
  int          nHandleIdx;  /* Next ACL handle Index (This entry should be inserted in front of the rule specified by this one*/
  int          action;      /* ACL action per match */
  paAclInfo_t aclInfo;  /* PA ACL configuration structure */
  paReturn_t ret;           /* Expected return code from pa_addIp command */
  Bool     acked;       /* Set to TRUE when the reply to the command is received */
} t14AclPaSetup_t;

/* Inner ACL configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t14AclSetup, ".testPkts")
#endif
static t14AclPaSetup_t t14AclSetup[] =  {

    /* ------- Entry 0 ----------- */
    {
        0,          /* Sequential ID */
        0,          /* Local Handle Index */
    pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
        -1,         /* L2/L3 Handle Index  */
        -1,         /* next ACL handle index */
        pa_ACL_ACTION_PERMIT,           /* ACL actions */
        {
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
            pa_IPV4,    /* IP Type */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 200, 201, 202, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            0,          /* Protocol */
            0,          /* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* ------- Entry 1 ----------- */
    {
        1,          /* Sequential ID */
         1,         /* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
      -1,     /* L2/L3 Handle Index  */
         -1,            /* next ACL handle index */
        pa_ACL_ACTION_PERMIT,           /* ACL actions */
        {
            pa_ACL_INFO_VALID_SRC_IP |
            pa_ACL_INFO_VALID_DST_IP  ,                          /* validbitMap */
            0,                                                   /* ctrlFlag */
            0,                                                   /* ctrlFlagMask */
            pa_IPV4,    /* IP Type */
            { 110, 111, 112, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 200, 201, 202, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            0,          /* Protocol */
            0,          /* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* ------- Entry 2 ----------- */
    {
        2,          /* Sequential ID */
         2,         /* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
      -1,     /* L2/L3 Handle Index  */
         -1,            /* next ACL handle index */
        pa_ACL_ACTION_PERMIT,           /* ACL actions */
        {
            pa_ACL_INFO_VALID_DST_PORT |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
            pa_IPV4,    /* IP Type */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 200, 201, 202, 103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            0,          /* Protocol */
            0,          /* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0x0400,     /* destPortbegin */
            0x0777,     /* destPort end */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* ------- Entry 3 ----------- */
    {
        3,          /* Sequential ID */
         3,         /* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
      -1,     /* L2/L3 Handle Index  */
         -1,            /* next ACL handle index */
        pa_ACL_ACTION_DENY,                           /* ACL actions */
        {
            pa_ACL_INFO_VALID_PROTO  |
            pa_ACL_INFO_VALID_SRC_IP |
            pa_ACL_INFO_VALID_DST_IP  ,                          /* validbitMap */
            0,                                                   /* ctrlFlag */
            0,                                                   /* ctrlFlagMask */
            pa_IPV4,    /* IP Type */
            { 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            132,        /* Protocol */
            0,          /* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* ------- Entry 4 ----------- */
    {
        4,          /* Sequential ID */
         4,         /* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
        -1,         /* L2/L3 Handle Index  */
         -1,            /* next ACL handle index */
        pa_ACL_ACTION_DENY,         /* ACL actions */
        {
            pa_ACL_INFO_VALID_SRC_PORT |
            pa_ACL_INFO_VALID_DST_PORT |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
            pa_IPV4,    /* IP Type */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            0,          /* Protocol */
            0,          /* DSCP */
            0xAAC0,     /* srcPortbegin */
            0xAAF0,     /* srcPort end */
            0x0400,     /* destPortbegin */
            0x0600,     /* destPort end */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* ------- Entry 5 ----------- */
    {
        5,          /* Sequential ID */
        5,      /* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
        -1,         /* L2/L3 Handle Index  */
        -1,            /* next ACL handle index */
        pa_ACL_ACTION_DENY,           /* ACL actions */
        {
            pa_ACL_INFO_VALID_DST_PORT |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
            pa_IPV4,    /* IP Type */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 10, 11, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            0,          /* Protocol */
            0,          /* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0x0700,     /* destPortbegin */
            0x0900,     /* destPort end */
        },
        pa_ERR_CONFIG,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* ------- Entry 6 ----------- */
    {
        6,          /* Sequential ID */
        6,          /* Local Handle Index */
    pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
        -1,         /* L2/L3 Handle Index  */
        -1,         /* next ACL handle index */
        pa_ACL_ACTION_PERMIT,           /* ACL actions */
        {
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
            pa_IPV4,    /* IP Type */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            0,          /* Protocol */
            0,          /* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },

    /* ------- Entry 7 ----------- */
    {
        7,          /* Sequential ID */
        7,          /* Local Handle Index */
        pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
        -1,         /* L2 Handle Index */
        -1,         /* next ACL handle index */
        pa_ACL_ACTION_DENY,         /* ACL actions */
        {
            pa_ACL_INFO_VALID_SRC_IP        |
            pa_ACL_INFO_VALID_SRC_IP_MASK   |
            pa_ACL_INFO_VALID_DST_IP_MASK   |
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
            pa_IPV4,    /* IP Type */
            { 81, 82, 82, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0xFF, 0xFF, 0xFF, 0xF0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 71, 72, 73, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
            { 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            0,          /* Protocol (ICMP) */
            0,          /* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },
    /* ------- Entry 8 ----------- */
    {
        8,          /* Sequential ID */
        8,          /* Local Handle Index */
    pa_ACL_INST_INNER_IP, /* ACL LUT Instance */
        -1,         /* L2/L3 Handle Index  */
        -1,         /* next ACL handle index */
        pa_ACL_ACTION_PERMIT,           /* ACL actions */
        {
            pa_ACL_INFO_VALID_DST_IP,                             /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
            pa_IPV6,    /* IP Type */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
            { 0x00, 0x02, 0x04, 0x08, 0x10, 0x12, 0x14, 0x18, 0x20, 0x22, 0x24, 0x28, 0x30, 0x32, 0x34, 0x39 },  /* IP Destination address */
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
            0,          /* Protocol */
            0,          /* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0,          /* destPortbegin */
            0,          /* destPort end */
        },
        pa_OK,      /* PA LLD return value expected */
        FALSE       /* Set to true when the command is acked */
    },
};

#define T14_EOAM_PACKET_INDEX_OFFSET 35
#define T14_IP_PACKET_INDEX_OFFSET   6 /* Using the first byte of the source mac as it is not used for match */


/* EOAM packet draft */
/* packet 8 (with APS opcode, triggers statistics for Eoam target flow).
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * EOAM dest = 00:0e:a6:66:57:09   (EOAM info 0)
 * EOAM Stats: Yes, as it is APS linear pkt and TBL MEG=PKT MEG
 * EOAM Pkt: Yes, would not be received in EOAM match queue as it is APS linear pkt
 * Designed to match EOAM configuration 0 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt8, ".testPkts")
#pragma DATA_ALIGN(eoamPkt8, 8)
static const uint8_t eoamPkt8[] = {
#else
static const uint8_t eoamPkt8[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,      /* Vlan */
    0x89, 0x02, 0x40, 0x27, 0x00, 0x0c, 0xCC, 0xCC,      /* UnKnown Opcode (39 = 0x27 = APS Linear) EOAM pkt, MEG = 2, Ver= 0, Pkt MEG == TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x08, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt8Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(2,39),                    /* meg = 2, opcode = 39  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)                      /* target flow count not applicable */
};


/* EOAM packet draft */
/* packet 9 (with CCM opcode, triggers no statistics for Eoam target flow).
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * EOAM dest = 00:0e:a6:66:57:09   (EOAM info 0)
 * EOAM Pkt: Yes, would not be received in EOAM match queue as it is CCM linear pkt
 * Designed to match EOAM configuration 0 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt9, ".testPkts")
#pragma DATA_ALIGN(eoamPkt9, 8)
static const uint8_t eoamPkt9[] = {
#else
static const uint8_t eoamPkt9[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,      /* Vlan */
    0x89, 0x02, 0x40, 0x01, 0x00, 0x0c, 0xCC, 0xCC,      /* UnKnown Opcode (01 = 0x01 = CCM) EOAM pkt, MEG = 2, Ver= 0, Pkt MEG == TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x09, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt9Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(2,1),                    /* L3 offset = 26  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)
};


/* EOAM packet draft demonstrating match of VLAN, VLAN Pri, Source MAC, Dest MAC */
/* packet 10 (with known opcode, triggers statistics for Eoam target flow).
 * VLAN     = 0x786
 * VLAN ID  = 7
 * EOAM dest = e0:0a:10:20:30:12 (EOAM info 12, No MAC Info Entry )
 * EOAM src  = 0e:a0:01:02:03:12 (EOAM info 12, No Mac entry )
 * EOAM Pkt: Yes, would be received in EOAM match queue
 * Designed to match EOAM configuration 12 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt10, ".testPkts")
#pragma DATA_ALIGN(eoamPkt10, 8)
static const uint8_t eoamPkt10[] = {
#else
static const uint8_t eoamPkt10[] __attribute__ ((aligned (8))) = {
#endif

    0xe0, 0x0a, 0x10, 0x20, 0x30, 0x12, 0x0e, 0xa0,
    0x01, 0x02, 0x03, 0x12, 0x81, 0x00, 0xe7, 0x86,      /* Vlan = 0x786, VLAN Pri = 7*/
    0x89, 0x02, 0x60, 0x2b, 0x00, 0x0c, 0xCC, 0xCC,      /* Known Opcode (43 = 0x2b = LMM) EOAM pkt, MEG = 3, Ver= 0, need record as Pkt MEG > TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x0a, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt10Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,0,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(3,43),                    /*MEG = 3, OPCODE=43  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(1)
};


/* EOAM packet draft */
/* packet 11 (with known opcode, triggers statistics for Eoam target flow, count is reported in descriptor).
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * EOAM dest = 00:0e:a6:66:57:09   (EOAM info 0)
 * EOAM Pkt: Yes, would be received in EOAM match queue
 * Designed to match EOAM configuration 0 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt11, ".testPkts")
#pragma DATA_ALIGN(eoamPkt11, 8)
static const uint8_t eoamPkt11[] = {
#else
static const uint8_t eoamPkt11[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,      /* Vlan */
    0x89, 0x02, 0x60, 0x2a, 0x00, 0x0c, 0xCC, 0xCC,      /* Known Opcode (42 = 0x2a = LMR) EOAM pkt, MEG = 3, Ver= 0, need record as Pkt MEG > TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x0b, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt11Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(3,42),                    /* meg level = 3 opcode = 42  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(1)                      /* Count expected for this flow = 1 */
};

/* EOAM packet draft (with corrupted version number) */
/* packet 7 (with known opcode, triggers statistics for Eoam target flow).
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * EOAM dest = 00:0e:a6:66:57:09   (EOAM info 0)
 * EOAM Pkt: No, would not be received in EOAM match queue as the version number is corrupted
 * Designed to match EOAM configuration 0 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt7, ".testPkts")
#pragma DATA_ALIGN(eoamPkt7, 8)
static const uint8_t eoamPkt7[] = {
#else
static const uint8_t eoamPkt7[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,      /* Vlan */
    0x89, 0x02, 0x64, 0x2a, 0x00, 0x0c, 0xCC, 0xCC,      /* Known Opcode (42 = 0x2a = LMR) EOAM pkt, MEG = 3, Ver= 4, need record as Pkt MEG > TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x07, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt7Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(3,42),                    /* L3 offset = 26  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)
};



/* EOAM packet draft */
/* packet 1 (with known opcode, triggers statistics for Eoam target flow).
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * EOAM dest = 00:0e:a6:66:57:09   (EOAM info 0)
 * EOAM Pkt: Yes, would be received in EOAM match queue
 * Designed to match EOAM configuration 0 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt1, ".testPkts")
#pragma DATA_ALIGN(eoamPkt1, 8)
static const uint8_t eoamPkt1[] = {
#else
static const uint8_t eoamPkt1[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,      /* Vlan */
    0x89, 0x02, 0x60, 0x2a, 0x00, 0x0c, 0xCC, 0xCC,      /* Known Opcode (42 = 0x2a = LMR) EOAM pkt, MEG = 3, Ver= 0, need record as Pkt MEG > TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x01, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt1Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(3,42),                    /* L3 offset = 26  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(5)
};

/* EOAM packet draft */
/* packet 6 (with known opcode, triggers statistics for Eoam target flow).
 * mac dest = e0:0a:10:20:30:05  (MAC Info No Match)
 * EOAM dest = e0:0a:10:20:30:05   (EOAM info 5)
 * EOAM Pkt: Yes, would be received in EOAM match queue
 * Designed to match EOAM configuration 5 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt6, ".testPkts")
#pragma DATA_ALIGN(eoamPkt6, 8)
static const uint8_t eoamPkt6[] = {
#else
static const uint8_t eoamPkt6[] __attribute__ ((aligned (8))) = {
#endif
    0xe0, 0x0a, 0x10, 0x20, 0x30, 0x05, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x68, 0x88,      /* Vlan, VLAN Pri = 3 */
    0x89, 0x02, 0x60, 0x2b, 0x00, 0x0c, 0xCC, 0xCC,      /* Known Opcode (43 = 0x2b = LMM) EOAM pkt, MEG = 3, Ver= 0, need record as Pkt MEG > TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x06, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt6Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,0,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(3,43),
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(1)
};

/* EOAM packet draft */
/* packet 0 (with known opcode, triggers NO statistics for Eoam target flow).
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * EOAM dest = 00:0e:a6:66:57:09   (EOAM info 0)
 * EOAM Pkt: Yes, would be received in EOAM match queue
 * Designed to match EOAM configuration 0 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt0, ".testPkts")
#pragma DATA_ALIGN(eoamPkt0, 8)
static const uint8_t eoamPkt0[] = {
#else
static const uint8_t eoamPkt0[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,      /* Vlan */
    0x89, 0x02, 0x20, 0x2a, 0x00, 0x0c, 0xCC, 0xCC,      /* Known Opcode (42 = 0x2a) EOAM pkt, MEG = 1, ver = 0, need no record as PKT MEG < TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x00, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt0Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(1,42),                    /* L3 offset = 26  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)
};

/* EOAM packet draft */
/* packet 2
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * EOAM dest = 00:0e:a6:66:57:09   (EOAM info 0)
 * EOAM Pkt: Unknown opcode (would be received in next fail queue)
 * Triggers No statistics updates for EOAM Info 0
 * Designed to match EOAM configuration 0 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt2, ".testPkts")
#pragma DATA_ALIGN(eoamPkt2, 8)
static const uint8_t eoamPkt2[] = {
#else
static const uint8_t eoamPkt2[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,      /* Vlan */
    0x89, 0x02, 0x20, 0x23, 0x00, 0x0c, 0xCC, 0xCC,      /* UnKnown Opcode (35 = 0x23) EOAM pkt, but matches classification */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x02, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt2Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 42)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 122, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(1,35),                    /* L3 offset = 26  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)
};

/* EOAM packet draft */
/* packet 4
 * mac dest = 01:80:c2:00:00:32  (MAC Info 4)
 * EOAM dest = 01:80:c2:00:00:32   (EOAM info None)
 * EOAM Pkt: known opcode (LMR = 0x2a), Pkt MEG = 1
 * Designed for no match EOAM configuration (seen in next fail queue) */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt4, ".testPkts")
#pragma DATA_ALIGN(eoamPkt4, 8)
static const uint8_t eoamPkt4[] = {
#else
static const uint8_t eoamPkt4[] __attribute__ ((aligned (8))) = {
#endif
    0x01, 0x80, 0xc2, 0x00, 0x00, 0x32, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,      /* Vlan */
    0x89, 0x02, 0x20, 0x2a, 0x00, 0x0c, 0xCC, 0xCC,      /* No Match EOAM classification */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x04, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt4Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(0,0),                    /* L3 offset = 26  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)
};

/* LLDP packet draft */
/* packet 5
 * mac dest = 01:80:c2:00:00:0e  (MAC Info 5 - LLDP multicast)
 * EOAM dest = 01:80:c2:00:00:0e   (EOAM info 1)
 * EOAM Pkt: None, Special Packet matching EOAM target classification, but ignored from target classification
 * Designed for match EOAM configuration 1, but trigger no statistics (expected to be seen in next fail) */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt5, ".testPkts")
#pragma DATA_ALIGN(eoamPkt5, 8)
static const uint8_t eoamPkt5[] = {
#else
static const uint8_t eoamPkt5[] __attribute__ ((aligned (8))) = {
#endif
  0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e, 0x00, 0x01,
  0x30, 0xf9, 0xad, 0xa0, 0x88, 0xcc, 0x02, 0x07,
  0x04, 0x00, 0x01, 0x30, 0xf9, 0xad, 0xa0, 0x04,
  0x04, 0x05, 0x31, 0x2f, 0x31, 0x06, 0x02, 0x00,
  0x78, 0x08, 0x17, 0x05, 0x75, 0x6d, 0x6d, 0x69,
  0x74, 0x33, 0x30, 0x30, 0x2d, 0x34, 0x38, 0x2d,
  0x50, 0x6f, 0x72, 0x74, 0x20, 0x31, 0x30, 0x30,
  0x31, 0x00, 0x0a, 0x0d, 0x53, 0x75, 0x6d, 0x6d,
  0x69, 0x74, 0x33, 0x30, 0x30, 0x2d, 0x34, 0x38,
  0x00, 0x0c, 0x4c, 0x53, 0x75, 0x6d, 0x6d, 0x69,
  0x74, 0x33, 0x30, 0x30, 0x2d, 0x34, 0x38, 0x20,
  0x2d, 0x20, 0x56, 0x65, 0x72, 0x73, 0x69, 0x6f,
  0x6e, 0x20, 0x37, 0x2e, 0x34, 0x65, 0x2e, 0x31,
  0x20, 0x28, 0x42, 0x75, 0x69, 0x6c, 0x64, 0x20,
  0x35, 0x29, 0x20, 0x62, 0x79, 0x20, 0x52, 0x65,
  0x6c, 0x65, 0x61, 0x73, 0x65, 0x5f, 0x4d, 0x61,
  0x73, 0x74, 0x65, 0x72, 0x20, 0x30, 0x35, 0x2f,
  0x32, 0x37, 0x2f, 0x30, 0x35, 0x20, 0x30, 0x34,
  0x3a, 0x35, 0x33, 0x3a, 0x31, 0x31, 0x00, 0x0e,
  0x04, 0x00, 0x14, 0x00, 0x14, 0x10, 0x0e, 0x07,
  0x06, 0x00, 0x01, 0x30, 0xf9, 0xad, 0xa0, 0x02,
  0x00, 0x00, 0x03, 0xe9, 0x00, 0xfe, 0x07, 0x00,
  0x12, 0x0f, 0x02, 0x07, 0x01, 0x00, 0xfe, 0x09,
  0x00, 0x12, 0x0f, 0x01, 0x03, 0x6c, 0x00, 0x00,
  0x10, 0xfe, 0x09, 0x00, 0x12, 0x0f, 0x03, 0x01,
  0x00, 0x00, 0x00, 0x00, 0xfe, 0x06, 0x00, 0x12,
  0x0f, 0x04, 0x05, 0xf2, 0xfe, 0x06, 0x00, 0x80,
  0xc2, 0x01, 0x01, 0xe8, 0xfe, 0x07, 0x00, 0x80,
  0xc2, 0x02, 0x01, 0x00, 0x00, 0xfe, 0x17, 0x00,
  0x80, 0xc2, 0x03, 0x01, 0xe8, 0x10, 0x76, 0x32,
  0x2d, 0x30, 0x34, 0x38, 0x38, 0x2d, 0x30, 0x33,
  0x2d, 0x30, 0x35, 0x30, 0x35, 0x00, 0xfe, 0x05,
  0x00, 0x80, 0xc2, 0x04, 0x00, 0x00, 0x00
};


static const pasahoLongInfo_t eoamPkt5Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(263,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(0,0),                    /* L3 offset = 26  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)
};


/* packet 0
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1)
 * EOAM target flow match: Yes, entry 0
 * out ip dest = 200.201.202.100 (out IP info 0)
 * UDP (destination port = 0x0555) (UDP entry 0, match)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0, ".testPkts")
#endif
static uint8_t pkt0[] = {
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
    0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
    0xca, 0x64, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
    0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
    0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
    0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
    0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
    0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
    0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
    0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
    0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
    0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
    0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
    0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt0Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 1
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * EOAM target flow match: none
 * out ip dest = 200.201.202.104
 * UDP (destination port = 0xa200)
 * fake IPSEC ESP packet  (transport mode)
 * UDP (destination port = 0xa200) (UDP entry 18, match)
 * Designed to match outer IP configuration 8 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1, ".testPkts")
#endif
static uint8_t pkt1[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x8a, 0x00, 0x00, 0x00, 0x00, 0x05, 0x32,
    0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
    0xca, 0x68, 0x55, 0x55, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x01, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00,
    0x00, 0x01, 0x12, 0x34, 0xa2, 0x00, 0x00, 0x58,
    0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
    0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
    0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
    0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
    0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
    0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
    0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
    0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
    0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
    0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
    0xf4, 0xd9, 0x00, 0x11, 0xaa, 0xbb, 0xcc, 0xdd,
    0xaa, 0xbb, 0xcc, 0xdd, 0xaa, 0xbb, 0xcc, 0xdd};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt1Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,58),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(138,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_ESP),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};


/* packet 2
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * EOAM target flow match: none
 * out ip dest = 200.201.202.104 (out IP info 8)
 * in  ip dest = 200.201.202.106 (in  IP info 10)
 * UDP (destination port = 0xb000)
 * fake IPSEC ESP packet  (Tunnel mode)
 * Designed to match outer IP configuration 8, Inner IP configuration 10 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2, ".testPkts")
#endif
static uint8_t pkt2[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x02, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00,
    0x45, 0x00, 0x00, 0x9e, 0x00, 0x00, 0x00, 0x00,
    0x05, 0x32, 0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04,
    0xc8, 0xc9, 0xca, 0x68, 0x55, 0x55, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x01, 0x66, 0x66, 0x66, 0x66,
    0x00, 0x00, 0x00, 0x01, 0x45, 0x00, 0x00, 0x6c,
    0x00, 0x00, 0x00, 0x00, 0x05, 0x11, 0x1d, 0xe7,
    0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9, 0xca, 0x6a,
    0x12, 0x34, 0xb0, 0x00, 0x00, 0x58, 0x9a, 0xe6,
    0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4, 0x78, 0xd8,
    0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1, 0xfc, 0xd9,
    0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57, 0x74, 0x64,
    0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9, 0x57, 0xdb,
    0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e, 0xa6, 0x14,
    0x93, 0x25, 0x56, 0x24, 0x44, 0xdf, 0x59, 0x8d,
    0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89, 0x9d, 0x7e,
    0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88, 0xf5, 0xb4,
    0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d, 0x0f, 0xbd,
    0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68, 0xf4, 0xd9,
    0x00, 0x04, 0xaa, 0xbb, 0xcc, 0xdd, 0xaa, 0xbb,
    0xcc, 0xdd, 0xaa, 0xbb, 0xcc, 0xdd  };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt2Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,78),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(158,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_ESP),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
  TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 3
  * mac dest = 00:01:02:03:04:aa  (MAC Info 0, EOAM Info none)
  * out ip dest = ... 0x30:0x32:0x34:0x39
  * UDP (destination port = 0x8102)
  * Designed to match outer IP configuration 4 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3, ".testPkts")
#endif
 static uint8_t pkt3[] = {
   0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x03, 0xe0,
   0xa6, 0x66, 0x57, 0x04, 0x86, 0xDD,

   0x60, 0x01, 0x01, 0x01,                 /* IPv6 hedaer */
   0x00, 0x68, 0x00, 0xFF,
   0x20, 0x02, 0x9e, 0xda,
   0x6d, 0x30, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00,
   0x00, 0x02, 0x04, 0x08,
   0x10, 0x12, 0x14, 0x18,
   0x20, 0x22, 0x24, 0x28,
   0x30, 0x32, 0x34, 0x39,

   0x11, 0x01, 0x04, 0x08,                /* Hop-by-hop header */
   0x10, 0x12, 0x14, 0x18,
   0x20, 0x22, 0x24, 0x28,
   0x30, 0x32, 0x34, 0x39,

   0x12, 0x34, 0x81, 0x02,               /* UDP Header */
   0x00, 0x00, 0x00, 0x00,


   0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
   0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
   0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
   0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
   0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
   0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
   0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
   0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
   0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
   0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
   0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3Info, ".testPkts")
#endif
 static pasahoLongInfo_t pkt3Info = {
   TF_FORM_PKT_INFO_WORD0(0,24,1,0,78),            /* cmd len = 24, pmatch = 1, frag = 0, start offset = 78 (UDP Header) */
   TF_FORM_PKT_INFO_WORD1(70,0,0,PASAHO_HDR_UDP),     /* end offset = 70, errIdx, portNum = 0, nextHdr = UDP */
   TF_FORM_PKT_INFO_WORD2(14,0,0,0),                 /* L3 offset = 14  */
   TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_UDP),
                           0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
     TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
   TF_FORM_PKT_INFO_WORD5(0)
 };

/* packet 4
  * mac dest = 00:0e:a6:66:57:09  (MAC Info 1, EOAM Entry 0)
  * out ip dest = ... 0x30:0x32:0x34:0x3B
  * UDP (destination port = 0xa002)
  * fake IPSEC ESP packet  (transport mode)
  * Designed to match outer IP configuration 7
  */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4, ".testPkts")
#endif
static uint8_t pkt4[] = {
   0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x04,0xe0,
   0xa6, 0x66, 0x57, 0x04, 0x86, 0xDD,

   0x60, 0x01, 0x01, 0x01,                 /* IPv6 hedaer */
   0x00, 0x86, 0x00, 0xFF,
   0x20, 0x02, 0x9e, 0xda,
   0x6d, 0x30, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00,
   0x00, 0x02, 0x04, 0x08,
   0x10, 0x12, 0x14, 0x18,
   0x20, 0x22, 0x24, 0x28,
   0x30, 0x32, 0x34, 0x3b,

   0x32, 0x01, 0x04, 0x08,                /* Hop-by-hop header */
   0x10, 0x12, 0x14, 0x18,
   0x20, 0x22, 0x24, 0x28,
   0x30, 0x32, 0x34, 0x3b,

   0x55, 0x55, 0x00, 0x01,                /* ESP HEader */
   0x00, 0x00,  0x00, 0x01,
   0x66, 0x66, 0x66, 0x66,
   0x00, 0x00,  0x00, 0x01,

   0x12, 0x34, 0xa0, 0x02,               /* UDP Header */
   0x00, 0x00, 0x00, 0x00,


   0x2d, 0xcf, 0x46, 0x29,
   0x04, 0xb4, 0x78, 0xd8,
   0x68, 0xa7, 0xff, 0x3f,
   0x2b, 0xf1, 0xfc, 0xd9,
   0x7a, 0x96, 0x09, 0x2c,
   0xa5, 0x57, 0x74, 0x64,
   0xc4, 0xaf, 0x15, 0x28,
   0xa4, 0xe9, 0x57, 0xdb,
   0x5e, 0x20, 0xfb, 0x38,
   0xa8, 0x4e, 0xa6, 0x14,
   0x93, 0x25, 0x56, 0x24,
   0x44, 0xdf, 0x59, 0x8d,
   0x43, 0x7b, 0xbe, 0x90,
   0x16, 0x89, 0x9d, 0x7e,
   0x77, 0xc6, 0x2f, 0x26,
   0x98, 0x88, 0xf5, 0xb4,
   0x30, 0xd4, 0x34, 0x9d,
   0x3a, 0x0d, 0x0f, 0xbd,
   0x2f, 0xa1, 0xf7, 0x0f,
   0xd9, 0x68, 0xf4, 0xd9,


   0x00, 0x11, 0xaa, 0xbb,      /* ESP Tag */
   0xcc, 0xdd, 0xaa, 0xbb,
   0xcc, 0xdd, 0xaa, 0xbb,
   0xcc, 0xdd

 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt4Info, ".testPkts")
#endif
 static pasahoLongInfo_t pkt4Info = {
   TF_FORM_PKT_INFO_WORD0(0,24,1,0,94),            /* cmd len = 24, pmatch = 1, frag = 0, start offset = 78 (UDP Header) */
   TF_FORM_PKT_INFO_WORD1(86,0,0,PASAHO_HDR_UDP),     /* end offset = 70, errIdx, portNum = 0, nextHdr = UDP */
   TF_FORM_PKT_INFO_WORD2(14,0,0,0),                 /* L3 offset = 14  */
   TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_UDP),
                           0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
     TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
   TF_FORM_PKT_INFO_WORD5(0)
 };

/* packet 5
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1, EOAM entry 0)
 * out ip dest = 0x30:0x32:0x34:0x3b (out IP info 7)
 * in  ip dest = 0x30:0x32:0x34:0x40 (in  IP info 20)
 * UDP (destination port = 0x9002)
 * fake IPSEC ESP packet  (Tunnel mode)
 * Designed to match outer IP configuration 7, Inner IP configuration 20 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5, ".testPkts")
#endif
 static uint8_t pkt5[] = {
   0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x05, 0xe0,
   0xa6, 0x66, 0x57, 0x04, 0x86, 0xdd,

   0x60, 0x01, 0x01, 0x01,                 /* IPv6 hedaer (next header = 0x00 Hop By Hop */
   0x00, 0xBE, 0x00, 0xff,
   0x20, 0x02, 0x9e, 0xda,
   0x6d, 0x30, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00,
   0x00, 0x02, 0x04, 0x08,
   0x10, 0x12, 0x14, 0x18,
   0x20, 0x22, 0x24, 0x28,
   0x30, 0x32, 0x34, 0x3b,

   0x32, 0x01, 0x04, 0x08,                /* Hop-by-hop header  (next header*/
   0x10, 0x12, 0x14, 0x18,
   0x20, 0x22, 0x24, 0x28,
   0x30, 0x32, 0x34, 0x3b,

   0x55, 0x55, 0x00, 0x01,                /* ESP HEader */
   0x00, 0x00, 0x00, 0x01,
   0x66, 0x66, 0x66, 0x66,
   0x00, 0x00, 0x00, 0x01,

   0x60, 0x01, 0x01, 0x01,                 /* IPv6 hedaer (next header = 0x00 Hop By Hop */
   0x00, 0x68, 0x00, 0xFF,
   0x20, 0x02, 0x9e, 0xda,
   0x6d, 0x30, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00,
   0x00, 0x02, 0x04, 0x08,
   0x10, 0x12, 0x14, 0x18,
   0x20, 0x22, 0x24, 0x28,
   0x30, 0x32, 0x34, 0x40,

   0x11, 0x01, 0x04, 0x08,                /* Hop-by-hop header  (next header*/
   0x10, 0x12, 0x14, 0x18,
   0x20, 0x22, 0x24, 0x28,
   0x30, 0x32, 0x34, 0x40,

   0x12, 0x34, 0x90, 0x02,                /* UDP Header */
   0x00, 0x00, 0x00, 0x00,

   0x2d, 0xcf, 0x46, 0x29,                /* Pay load = 80 bytes */
   0x04, 0xb4, 0x78, 0xd8,
   0x68, 0xa7, 0xff, 0x3f,
   0x2b, 0xf1, 0xfc, 0xd9,
   0x7a, 0x96, 0x09, 0x2c,
   0xa5, 0x57, 0x74, 0x64,
   0xc4, 0xaf, 0x15, 0x28,
   0xa4, 0xe9, 0x57, 0xdb,
   0x5e, 0x20, 0xfb, 0x38,
   0xa8, 0x4e, 0xa6, 0x14,
   0x93, 0x25, 0x56, 0x24,
   0x44, 0xdf, 0x59, 0x8d,
   0x43, 0x7b, 0xbe, 0x90,
   0x16, 0x89, 0x9d, 0x7e,
   0x77, 0xc6, 0x2f, 0x26,
   0x98, 0x88, 0xf5, 0xb4,
   0x30, 0xd4, 0x34, 0x9d,
   0x3a, 0x0d, 0x0f, 0xbd,
   0x2f, 0xa1, 0xf7, 0x0f,
   0xd9, 0x68, 0xf4, 0xd9,

   0x00, 0x29, 0xaa, 0xbb,                /* ESP Tag */
   0xcc, 0xdd, 0xaa, 0xbb,
   0xcc, 0xdd, 0xaa, 0xbb,
   0xcc, 0xdd
 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Info, ".testPkts")
#endif
 static pasahoLongInfo_t pkt5Info = {
   TF_FORM_PKT_INFO_WORD0(0,24,1,0,150),            /* cmd len = 24, pmatch = 1, frag = 0, start offset = 78 (UDP Header) */
   TF_FORM_PKT_INFO_WORD1(142,0,0,PASAHO_HDR_UDP),     /* end offset = 70, errIdx, portNum = 0, nextHdr = UDP */
   TF_FORM_PKT_INFO_WORD2(14,0,0,0),                 /* L3 offset = 14  */
   TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_UDP),
                           0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
     TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
   TF_FORM_PKT_INFO_WORD5(0)
 };

/* acl packet 0
 * mac dest = 00:0e:a6:66:57:09  (MAC Info 1, EOAM entry 0)
 * out ip dest = 200.201.202.100 (out IP info 0)
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6, ".testPkts")
#endif
static uint8_t pkt6[] = {
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x06, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
    0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
    0xca, 0x64, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
    0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
    0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
    0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
    0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
    0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
    0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
    0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
    0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
    0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
    0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
    0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt6Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 7
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * ip src  = 110.111.112.113
 * ip dest = 200.201.202.101 (out IP info 1)
 * UDP (destination port = 0x9008)
 * Designed to match outer IP configuration 1 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt7, ".testPkts")
#endif
static uint8_t pkt7[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x07, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
    0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
    0xca, 0x65, 0x12, 0x34, 0x90, 0x08, 0x00, 0x58,
    0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
    0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
    0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
    0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
    0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
    0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
    0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
    0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
    0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
    0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
    0xf4, 0xd9 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt7Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt7Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 8
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0, EOAM Entry None)
 * out ip src  = 81.82.83.84
 * out ip dest = 200.201.202.105 (out IP info 8)
 * fake IPSEC ESP NAT-T packet
 * Designed to match IP configuration (SPI) 8 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8, ".testPkts")
#endif
static uint8_t pkt8[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x08, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x74, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x51, 0x52, 0x53, 0x54, 0xc8, 0xc9,
	0xca, 0x69, 0xaa, 0xbb, 0x11, 0x94, 0x00, 0x60,
   0x00, 0x00, 0x55, 0x55, 0x00, 0x03, 0x00, 0x00,
	0x00, 0x01, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00,
	0x00, 0x01, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt8Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt8Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 50 (ESP) */
	TF_FORM_PKT_INFO_WORD1(130,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 130, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,42),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_ESP | PASAHO_HDR_BITMASK_IPSEC_NAT_T),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 9 (Fake IPSec AH Packet)
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0, EOAM None)
 * out ip dest = 200.201.202.103 (out IP info 6)
 * AH:0x44440000, icvSize = 12
 * UDP (destination port = 0x0777) (UDP entry 8, match)
 * Designed to match outer IP configuration 6, UDP Entry 8 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9, ".testPkts")
#endif
static uint8_t pkt9[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x09, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x05, 0x33,
    0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
    0xca, 0x67, 0x11, 0x04, 0x00, 0x00, 0x44, 0x44,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x66, 0x66,
    0x66, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x12, 0x34, 0x07, 0x77, 0x00, 0x58,
    0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
    0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
    0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
    0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
    0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
    0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
    0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
    0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
    0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
    0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
    0xf4, 0xd9
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt9Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt9Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,66),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(146,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_AH | PASAHO_HDR_BITMASK_UDP),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};


/* packet 10
 * mac dest = 00:e0:a6:66:57:09  (MAC Info 1, EOAM Info 0)
 * PPPoE session header (verLen = 0x11, code = 0,
 * sessionId = 0x1234, length = 0x006e, prot = 0x0021 (IPv4)
 * out ip src  = 110.111.112.113
 * out ip dest = 200.201.202.102 (out IP info 1)
 * UDP (destination port = 0x9000)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10, ".testPkts")
#endif
static uint8_t pkt10[] = {
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x0a, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x11, 0x00,
    0x12, 0x34, 0x00, 0x6e, 0x00, 0x21, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
    0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0xc8, 0xc9,
    0xca, 0x66, 0x12, 0x34, 0x90, 0x00, 0x00, 0x58,
    0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
    0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
    0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
    0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
    0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
    0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
    0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
    0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
    0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
    0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
    0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt10Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt10Info = {

    TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(130,0,0,PASAHO_HDR_UDP),     /* end offset = 114, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_PKT_INFO_WORD2(14,42,0,0),                  /* L3 offset = 34, l4Offset = 42, l5Offset = 0, ahEspOffset = 0 */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_PPPoE | PASAHO_HDR_BITMASK_UDP),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)

};

/* packet 11
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.82.80
 * out ip dest = 71.72.73.74   (IP info None)
 * fake ICMP packet
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 2 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11, ".testPkts")
#endif
static uint8_t pkt11[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x0b, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01,
    0x43, 0x0c, 0x51, 0x52, 0x52, 0x50, 0x47, 0x48,
    0x49, 0x4a, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
    0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
    0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
    0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
    0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
    0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
    0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
    0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
    0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
    0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
    0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
    0xf4, 0xd9 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt11Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt11Info = {

    TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (ICMP) */
    TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 12
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * PPPoE session header (verLen = 0x11, code = 0,
 * sessionId = 0x1234, length = 0x006e, prot = 0x0021 (IPv4)
 * out ip src  = 110.111.112.113 (out IP info None)
 * out ip dest = 10.11.12.13
 * UDP (destination port = 0x0555, source port = 0xaad0)
 * Designed to match ACL Rule 4 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt12, ".testPkts")
#endif
static uint8_t pkt12[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x0c, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x11, 0x00,
  0x12, 0x34, 0x00, 0x6e, 0x00, 0x21, 0x45, 0x00,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x43, 0x0c, 0x6e, 0x6f, 0x70, 0x71, 0x0a, 0x0b,
	0x0c, 0x0d, 0xaa, 0xd0, 0x05, 0x55, 0x00, 0x58,
	0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt12Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt12Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(130,0,0,PASAHO_HDR_UDP),     /* end offset = 114, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,42,0,0),    	            /* L3 offset = 34, l4Offset = 42, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_PPPoE),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};

/* packet 13
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip src  = 81.82.82.85 (rejected by ACL rule 81.82.82.8x)
 * out ip dest = 71.72.73.74
 * UDP (destination port = 0x0555)
 * Designed to match outer IP configuration 2 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt13, ".testPkts")
#endif
static uint8_t pkt13[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x0d, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
    0x43, 0x0c, 0x51, 0x52, 0x52, 0x55, 0x47, 0x48,
    0x49, 0x4a, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
    0xc0, 0x0b, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
    0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
    0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
    0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
    0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
    0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
    0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
    0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
    0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
    0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
    0xf4, 0xd9 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt13Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt13Info = {

    TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (ICMP) */
    TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UNKNOWN),     /* end offset = 142, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 14
 * mac src = 0e:a0:01:02:03:04  (MAC Info 6)
 * out ip dst = 200.201.202.108 (IP info 18)
 * UDP (destination port = 0x0666)
 * Designed to match outer IP configuration 18
 * data[18] = pktInfo offset
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt14, ".testPkts")
#endif
static uint8_t pkt14[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x22, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x51, 0x52, 0x53, 0x01, 0xc8, 0xc9,
	0xca, 0x6c, 0x12, 0x34, 0x06, 0x66, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt14Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt14Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 122, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 15
 * mac dest = 00:01:02:03:04:aa (MAC Info 0)
 * out ip dest = 71.72.73.70 (out IP info None)
 * UDP (destination port = 0x0555)
 * Fragment: rejected by ACL rule */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt15, ".testPkts")
#endif
static uint8_t pkt15[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x0f, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x20, 0x00, 0x05, 0x11,
    0x19, 0x07, 0x9e, 0xda, 0x6d, 0x0b, 0x47, 0x48,
    0x49, 0x46, 0xaa, 0xbb, 0x05, 0x55, 0x00, 0x58,
    0xeb, 0xc7, 0x14, 0x15,
    0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
    0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
    0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
    0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
    0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
    0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
    0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
    0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt15Info, ".testPkts")
#endif
static pasahoLongInfo_t pkt15Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 122, errIdx, portNum = 0, nextHdr = UDP */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),                   /* L3 offset = 14  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)

};

/* packet 16
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * VLAN, MPLS packet (IPv4)
 * out ip src  = 110.111.112.113 (out IP info None)
 * out ip dest = 10.11.12.13
 * UDP (destination port = 0x0855)
 * Designed to match ACL Rule 5 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt16, ".testPkts")
#pragma DATA_ALIGN(pkt16, 8)
static const uint8_t pkt16[] = {
#else
static const uint8_t pkt16[] __attribute__ ((aligned (8))) = {
#endif
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x10, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,
	0x88, 0x47, 0x12, 0x34, 0x51, 0x64, 0x45, 0x00,
	0x00, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x75, 0x4e, 0x6e, 0x6f, 0x70, 0x71, 0x0a, 0x0b,
	0x0c, 0x0d, 0x12, 0x34, 0x08, 0x55, 0x00, 0x48,
	0x61, 0x89, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
	0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
	0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
	0x70, 0x71
};
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt16Info, ".testPkts")
#endif
static const pasahoLongInfo_t pkt16Info =  {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_IPv4),    /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN | PASAHO_HDR_BITMASK_MPLS),
	                        1, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};

/* Bits 14 and 15 are used to tell where the packet is expected to wind up */
#define T14_PACKET_MATCH_VALID         (1 << 13)      /* Packet will match */
#define T14_PACKET_NFAIL               (2 << 13)      /* Packet will arrive in the next fail queue  */
//#define T14_PACKET_DISCARD             (4 << 13)      /* Packet is discarded by PA */
#define T14_PACKET_USR_STATS_UPD       (8 << 13)      /* Update the user defined stats */
#define T14_PACKET_DEST_MASK           (15 << 13)

#define T14_PACKET_INDEX_MASK          0x0fff

// #define T14_TARGET_FLOW_MATCH_INDEX0 0
// #define T14_TARGET_FLOW_MATCH_INDEX1 1

typedef struct pktTestInfo2_s {
  pktTestInfo_t     testInfo;
  paStatsBmap_t     statsMap[3];  /* Bit map of which stats to increment. Some stats must be incremented 3 times */
  int               eoamFlowNum;  /* Eoam flow match number, if packet is intended to match the EOAM target flow */
  uint32_t          ctrlBitMap;
}pktTestInfo2_t;

typedef struct pktTestInfo3_s {
  pktTestInfo_t     pktTestInfo;
  int               startOffset;  /* Start offset to insert */
  int               eoamFlowNum;  /* Eoam flow match number, if packet is intended to match the EOAM target flow */
  int               isPatchCount; /* Set if patch count is needed otherwise it is patch time for EOAM */
  uint32_t          ctrlBitMap;
}pktTestInfo3_t;

#define T14_PATCH_COUNT 1
#define T14_PATCH_TIME  0

/* EOAM egress packet draft */
/* packet 12 (with known opcode, triggers statistics for Eoam target flow).
 * mac dest =  00:e0:a6:66:57:09 (MAC Info 1, EOAM Entry 0)
 * EOAM Pkt: Yes, would be received in EOAM match queue
 * Designed to match EOAM configuration 5 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt12, ".testPkts")
#pragma DATA_ALIGN(eoamPkt12, 8)
static const uint8_t eoamPkt12[] = {
#else
static const uint8_t eoamPkt12[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x68, 0x88,      /* Vlan Pri = 3 */
    0x89, 0x02, 0x60, 0x2a, 0x00, 0x0c, 0xCC, 0xCC,      /* Known Opcode (42 = 0x2a = LMR) EOAM pkt, MEG = 3, Ver= 0, need record as Pkt MEG > TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x0c, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt12Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(3,42),                    /* meg = 2, opcode = 39  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)                      /* target flow count not applicable */
};

/* EOAM egress packet draft */
/* packet 13 (with known opcode, triggers statistics for Eoam target flow).
 * mac dest =  00:e0:a6:66:57:09 (MAC Info 1, EOAM Entry 0)
 * EOAM Pkt: Yes, would be received in EOAM match queue
 * Designed to match EOAM configuration 5 */

#ifdef _TMS320C6X
#pragma DATA_SECTION(eoamPkt13, ".testPkts")
#pragma DATA_ALIGN(eoamPkt13, 8)
static const uint8_t eoamPkt13[] = {
#else
static const uint8_t eoamPkt13[] __attribute__ ((aligned (8))) = {
#endif
    0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xbb,      /* store packet index at location 6 */
    0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x68, 0x88,      /* Vlan */
    0x89, 0x02, 0x60, 0x2f, 0x00, 0x0c, 0xCC, 0xCC,      /* Known Opcode (47 = 0x2f = DMM) EOAM pkt, MEG = 3, Ver= 0, need record as Pkt MEG > TBL MEG */
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0x00, 0x0d, 0x16, 0x17, 0x11, 0x11,
    0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
    0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
    0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

static const pasahoLongInfo_t eoamPkt13Info =  {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),                /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
    TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
    TF_FORM_EOAM_PKT_INFO_WORD2(3,47),                    /* meg = 2, opcode = 47  */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
    TF_FORM_EOAM_PKT_INFO_WORD5(0)                      /* target flow count not applicable */
};

#undef T14_TEST_HUGE_TIME_COUNTS

pktTestInfo3_t t14EoamEgressPktTestInfo[] =  {
        {
          {
            (uint8_t *)eoamPkt12,
            (pasahoLongInfo_t *)&eoamPkt12,
            sizeof(eoamPkt12),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_COUNT, /* Patch Count */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },


        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },
#ifdef T14_TEST_HUGE_TIME_COUNTS
        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },

        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },

        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },

        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },

        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },

        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },

        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },

        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },
        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },
        {
          {
            (uint8_t *)eoamPkt13,
            (pasahoLongInfo_t *)&eoamPkt13,
            sizeof(eoamPkt13),
            {
              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match */
              0,
              0
            },
            T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 12) /* ID */
          },
          22, /* start Offset location in the packet for inserting count */
          0, /* EOAM Flow Number associated with the entry */
          T14_PATCH_TIME, /* Patch Time */
          T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does deliver to Host Queue for EOAM */
        },
#endif
};

pktTestInfo2_t t14IpPktTestInfo[] =  {

#if 1
  /* Packet 0, Regular IPv4 Packet */
  {
    (uint8_t *)pkt0,
    (pasahoLongInfo_t *)&pkt0Info,
    sizeof(pkt0),
    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
      (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
    },
    T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 0 ,0),  /* Packet will be matched by pkt ID 0 */
    { 0,
      0,
      0
    },
    0,                    /* Expected EOAM target flow number update */
    T14_PKT_MATCH | T14_EOAM_NEED_RECORDS /* Updates the EOAM target flow */
  },

  /* Packet 1, ESP Transport  */
  /* SPI packet: NSS_GEN2: Two matches
                     PASS_GEN1: Single match */
  /* Move here since packet order should be maintained */
  {
      (uint8_t *)pkt1,
      (pasahoLongInfo_t *)&pkt1Info,
      sizeof(pkt1),
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                    /* MAC match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* ESP match */
      },                                   /* SPI match */
      T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 18, 1),  /* Packet will be matched by pkt ID 1 */
      {
        (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP),                                        /* UDP match */
        0,
        0
      },
     -1,              /* Expected EOAM target flow number update (NA)*/
      T14_PKT_MATCH  /* Not a discard packet */
  },

    /* Packet 2 */
      /* SPI packet: NSS_GEN2: Two matches
                     PASS_GEN1: Single match */
      /* Move here since packet order should be maintained */
    {
      (uint8_t *)pkt2,
      (pasahoLongInfo_t *)&pkt2Info,
      sizeof(pkt2),
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* ESP match */
      },                                   /* SPI match */
      T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE,10,2),  /* Packet will be matched by pkt ID 1 */
      {
        (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C2_NUM_UDP),    /* UDP match */
        0,
        0
      },
     -1,              /* Expected EOAM target flow number update (NA)*/
      T14_PKT_MATCH  /* Not a discard packet */
    },

    /* Packet 3, Regular IPv6 Packet */
    {
      (uint8_t *)pkt3,
      (pasahoLongInfo_t *)&pkt3Info,
      sizeof(pkt3),
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),   /* IP match */
        (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
      },
      T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 1, 3),  /* Packet will be matched by pkt ID 1 */
      { 0,
        0,
        0
      },
      0,              /* Expected EOAM target flow number update */
      T14_PKT_MATCH   /* Not a discard packet */
    },

  /* Packet 4, Transport Mode IPv6 Packet */
  {
    (uint8_t *)pkt4,
    (pasahoLongInfo_t *)&pkt4Info,
    sizeof(pkt4),
    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),   /* IP match */
      (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
    },
    T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 9, 4), /* id */
    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* ESP match */
      0,
      0
    },
      0,              /* Expected EOAM target flow number update */
      T14_EOAM_NEED_RECORDS | T14_PKT_MATCH   /* Not a discard packet */
  },


  /* Packet 5, Tunnel Mode IPv6 Packet */
  {
    (uint8_t *)pkt5,
    (pasahoLongInfo_t *)&pkt5Info,
    sizeof(pkt5),
    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),   /* IP Outer match */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),   /* Ip Inner match */
    },
    T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 3, 5), /* id */
    {
      (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C2_NUM_UDP),    /* UDP match */
      0,
      0
    },
      0,              /* Expected EOAM target flow number update */
      T14_EOAM_NEED_RECORDS | T14_PKT_MATCH   /* Not a discard packet */
  },

      /* For ACL test Packets */
      {
        (uint8_t *)pkt6,
        (pasahoLongInfo_t *)&pkt6Info,
        sizeof(pkt6),
        {
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
          (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
          (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C2_NUM_PACKETS)                     /* UDP Match */
        },                                                                                                      /* no other match */
        T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 0, 6), /* id */
        {
          0,
          0,
          0
        },
          0,              /* Expected EOAM target flow number update */
          T14_EOAM_NEED_RECORDS | T14_PKT_MATCH   /* Not a discard packet */
      },


  {
    (uint8_t *)pkt7,
    (pasahoLongInfo_t *)&pkt7Info,
    sizeof(pkt7),
    {
	  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  /* IP match */
      (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C2_NUM_PACKETS)                     /* UDP Match */
    },
    T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 17, 7), /* id */
    {
      0,
      0,
      0
    },
      -1,              /* Expected EOAM target flow number update (NA)*/
      T14_PKT_MATCH   /* Not a discard packet */
  },

  /* Packet 8, IPSec ESP NAT-T Packet */
  {
    (uint8_t *)pkt8,
    (pasahoLongInfo_t *)&pkt8Info,
    sizeof(pkt8),
    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP Outer match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH)                                     /* SPI match */
    },
    T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 4, 8), /* id */
    {
      0,
      0,
      0
    },
      0,              /* Expected EOAM target flow number update */
      T14_PKT_MATCH   /* Not a discard packet */
  },

    /* AH Packet test */
  /* Packet 0, Fake IPSec AH IPv4 Packet */
  {
    (uint8_t *)pkt9,
    (pasahoLongInfo_t *)&pkt9Info,
    sizeof(pkt9),
    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
      (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
    },
    T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 8 ,9),  /* Packet will be matched by pkt ID 9 */
    { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* AH match */
      0,
      0
    },
    -1,                    /* Not Applicable */
    T14_PKT_MATCH         /* Updates the EOAM target flow */
  },
  {
    (uint8_t *)pkt10,
    (pasahoLongInfo_t *)&pkt10Info,
    sizeof(pkt10),
    {
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  /* IP match */
      (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C2_NUM_PACKETS)                     /* UDP Match */
    },
    T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 2, 10), /* id */
    {
      0,
      0,
      0
    },
    0,              /* Expected EOAM target flow number update */
    T14_EOAM_NEED_RECORDS | T14_PKT_MATCH   /* Not a discard packet */
  },

  {
    (uint8_t *)pkt11,
    (pasahoLongInfo_t *)&pkt11Info,
    sizeof(pkt11),
          {
		        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
            0,
            0
          },                                                                                                    /* no other match */
    T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 0, 11), /* id */
    {
      0,
      0,
      0
    },
          -1,              /* Expected EOAM target flow number update (NA) */
      T14_PKT_DISCARD  /* discard packet */
  },

      /* packet rejected by ACL rule */
      /* Packet 12 */
      /* Move here since packet order should be maintained */
      {
          (uint8_t *)pkt12,
          (pasahoLongInfo_t *)&pkt12Info,
          sizeof(pkt12),
          { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
             0,
             0                                                                                        /* No Match */
      },
            T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 0, 12), /* id */
            {
              0,
              0,
              0
            },
                  -1,              /* Expected EOAM target flow number update (NA) */
        T14_PKT_DISCARD                /* discard packet */
      },

    /* packet rejected by ACL rule */
      /* Packet 13 */
      /* Move here since packet order should be maintained */
      {
          (uint8_t *)pkt13,
          (pasahoLongInfo_t *)&pkt13Info,
          sizeof(pkt13),
          { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
             0,
             0                                                                                        /* No Match */
      },
            T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 0, 13), /* id */
            {
              0,
              0,
              0
            },
                  -1,              /* Expected EOAM target flow number update (NA) */
        T14_PKT_DISCARD                /* discard packet */
      },

      /* Move here since packet order should be maintained */
      {
          (uint8_t *)pkt14,
          (pasahoLongInfo_t *)&pkt14Info,
          sizeof(pkt14),
          { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
            (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
            (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
          },
            T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 5, 14), /* id */
            {
              0,
              0,
              0
            },
                  -1,              /* Expected EOAM target flow number update (NA) */
          T14_PKT_MATCH               /* match packet */
      },
      /* Move here since packet order should be maintained */
      {
          (uint8_t *)pkt15,
          (pasahoLongInfo_t *)&pkt15Info,
          sizeof(pkt15),
          { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
            (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_IP_FRAG),                                                                        /* IP match */
            0},                                                                                                       /* No Match */
            T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 0, 15), /* id */
            {
              0,
              0,
              0
            },
                  -1,              /* Expected EOAM target flow number update (NA) */
       T14_PKT_DISCARD                /* discard packet */
      },
#endif
      /* packet rejected by ACL rule */
      /* Packet 16 */
      /* Move here since packet order should be maintained */
      {
          (uint8_t *)pkt16,
          (pasahoLongInfo_t *)&pkt16Info,
          sizeof(pkt16),
          { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | ( 1 << TF_STATS_BM_C1_NUM_MPLS),                                    /* MAC match */
             0,
             0                                                                                        /* No Match */
      },
            T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 0, 16), /* id */
            {
              0,
              0,
              0
            },
                  -1,              /* Expected EOAM target flow number update (NA) */
        T14_PKT_DISCARD                /* discard packet */
      },
    {
      (uint8_t *)NULL,
      (pasahoLongInfo_t *)NULL,
      NULL,
      {
        0,
        0,
        0
      },
      NULL,
      {
        0,
        0,
        0
      },
      -1,                    /* Expected EOAM target flow number update */
     0                       /* Updates the EOAM target flow */
    },
};

typedef struct eoamPktTestInfo_s {
  pktTestInfo_t testInfo;
  int           eoamFlowNum; /* -1: Not applicable, otherwise index of the eoam flow number */
  uint32_t      ctrlBitMap; /*  please refer the Ctrl bit map definitions above */
} eoamPktTestInfo_t;

/* Test Packets for EOAM entries */
eoamPktTestInfo_t t14EoamPktTestInfo[] = {
/* Pkt 0 */
{
 {
   (uint8_t *)eoamPkt11,
   (pasahoLongInfo_t *)&eoamPkt11Info,
    sizeof(eoamPkt11),
    {
      /* This packet is testing the EOAM flow with a wrong version EOAM packet */
      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match, EOAM match, unknown Opcode */
       0,
       0
    },
    T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 11) /* ID */
  },
  0, /* EOAM Flow Number associated with the entry */
  T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does not deliver to Host Queue for EOAM */
},

 /* Pkt 1 */
{
  {
    (uint8_t *)eoamPkt10,
    (pasahoLongInfo_t *)&eoamPkt10Info,
     sizeof(eoamPkt10),
     {
       /* This packet is testing the EOAM flow with a wrong version EOAM packet */
       (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH)  | (1 << TF_STATS_BM_C1_SILENT_DISCARD),  /* MAC No match, EOAM match, unknown Opcode */
        0
     },
     T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 12, 10) /* ID */
   },
   12, /* EOAM Flow Number associated with the entry */
   T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does not deliver to Host Queue for EOAM */
 },
  /* Pkt 2 */

  {
   {
     (uint8_t *)eoamPkt8,
     (pasahoLongInfo_t *)&eoamPkt8Info,
      sizeof(eoamPkt8),
      {
        /* This packet is testing the EOAM flow with a wrong version EOAM packet */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match, EOAM match, unknown Opcode */
        (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),                                   /* IP Match Fails */
         0
      },
      T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 1, 8) /* ID */
    },
    0, /* EOAM Flow Number associated with the entry */
    T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does not deliver to Host Queue for EOAM */
  },

 /* Pkt 3 */
  {
   {
     (uint8_t *)eoamPkt9,
     (pasahoLongInfo_t *)&eoamPkt9Info,
      sizeof(eoamPkt9),
      {
        /* This packet is testing the EOAM flow with a wrong version EOAM packet */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match, EOAM match, unknown Opcode */
        (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),                                   /* IP Match Fails */
         0
      },
      T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 1, 9) /* ID */
    },
    0, /* EOAM Flow Number associated with the entry */
    T14_EOAM_NEED_RECORDS  /* needStats, target flow match, does not deliver to Host Queue for EOAM */
  },
/* Pkt 4 */
 {
   {
     (uint8_t *)eoamPkt7,
     (pasahoLongInfo_t *)&eoamPkt7Info,
      sizeof(eoamPkt7),
      {
        /* This packet is testing the EOAM flow with a wrong version EOAM packet */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* No MAC match, EOAM match, known Opcode */
        (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),                                   /* IP Match Fails */
         0
      },
       T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 1, 7) /* ID */
    },
    0, /* EOAM Flow Number associated with the entry */
    T14_EOAM_NEED_RECORDS  /* needStats, target flow match, but does not deliver to Host Queue for EOAM */
  },
/* Pkt 5 */
  {
    {
      (uint8_t *)eoamPkt6,
      (pasahoLongInfo_t *)&eoamPkt6Info,
       sizeof(eoamPkt6),
       {
         /* This packet is really not silent discard as EOAM classification matches it, but it is discard from L2 match point of view */
         (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH) | (1 << TF_STATS_BM_C1_SILENT_DISCARD),  /* No MAC match, EOAM match, known Opcode */
          0,
          0
       },
        T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 5, 6) /* ID */
     },
     5, /* EOAM Flow Number */
     T14_EOAM_NEED_RECORDS  /* needStats, target flow match */
   },
/* Pkt 6 */
  {
    {
      (uint8_t *)eoamPkt0,
      (pasahoLongInfo_t *)&eoamPkt0Info,
       sizeof(eoamPkt0),
       {
         (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match, EOAM match, known Opcode */
          0,
          0
       },
       T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 0) /* ID */
     },
     -1,
     0 /* needStats, target flow match */
   },
  {
    {
      (uint8_t *)eoamPkt1,
      (pasahoLongInfo_t *)&eoamPkt1Info,
       sizeof(eoamPkt1),
       {
         (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match, EOAM match, known Opcode */
          0,
          0
       },
       T14_GET_UNIQUE_IDX(T14_EOAM_MATCH_BASE, 0, 1) /* ID */
     },
     0,
     T14_EOAM_NEED_RECORDS  /* needStats, target flow match */
   },
   {
     {
       (uint8_t *)eoamPkt2,
       (pasahoLongInfo_t *)&eoamPkt2Info,
        sizeof(eoamPkt2),
        {
          (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match, EOAM Match, Unknown Opcode */
          (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),                                   /* IP Match Fails */
          0
        },
        T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 1, 2) /* ID */
      },
        -1,
      0   /*needStats */
    },
   {
     {
       (uint8_t *)eoamPkt4,
       (pasahoLongInfo_t *)&eoamPkt4Info,
        sizeof(eoamPkt4),
        {
          (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match, EOAM No Match, known Opcode */
          (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),                                   /* IP Match Fails */
          0
        },
        T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 4, 4) /* ID */
      },
      -1,
       0   /*needStats */
    },
   {
     {
       (uint8_t *)eoamPkt5,
       (pasahoLongInfo_t *)&eoamPkt5Info,
        sizeof(eoamPkt5),
        {
          (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  /* MAC match, EOAM Match, Unknown Opcode */
          (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),                                   /* IP Match Fails */
          0
        },
        T14_GET_UNIQUE_IDX(T14_L2_NFAIL_BASE, 5, 5) /* Unique Packet ID */
      },
      -1,
       0   /*needStats */
    },

   /* Last entry - NULL */
   {
     {
       (uint8_t *)NULL,
       (pasahoLongInfo_t *)NULL,
       NULL,
        {
          0,
          0,
          0
        },
        0,
      },
      0,
      0
    },
};

/* packet 0
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip dest = 200.201.202.103 (out IP info 6)
 * UDP (destination port = 0x0777)
 * Designed to match outer IP configuration 6 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (reasmPkt0, ".testPkts")
#endif
static uint8_t reasmPkt0[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x01, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x33,
    0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
    0xca, 0x67, 0x11, 0x04, 0x00, 0x00, 0x44, 0x44,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x66, 0x66,
    0x66, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x12, 0x34, 0x07, 0x77, 0x00, 0x00,
    0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (reasmPkt0Info, ".testPkts")
#endif
static pasahoLongInfo_t reasmPkt0Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,66),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_AUTH),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
    TF_FORM_PKT_INFO_WORD2(14,58,66,34),                 /* L3 offset, l4Offset, l5Offset, ahEspOffset = 34 */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_AH),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};

/* packet 1
 * mac dest = 00:01:02:03:04:aa  (MAC Info 0)
 * out ip dest = 200.201.202.101 (out IP info 0)
 * UDP (destination port = 0x9008)
 * Designed to match outer IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (reasmPkt1, ".testPkts")
#endif
static uint8_t reasmPkt1[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
    0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
    0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
    0xca, 0x65, 0x12, 0x34, 0x90, 0x08, 0x00, 0x00,
    0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (reasmPkt1Info, ".testPkts")
#endif
static pasahoLongInfo_t reasmPkt1Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),                /* cmd len = 24, pmatch = 1, frag = 0, start offset = 42 (UDP Header) */
    TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
    TF_FORM_PKT_INFO_WORD2(14,34,42,0),                 /* L3 offset, l4Offset, l5Offset, ahEspOffset = 0 */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
                            0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)
};

/* reassempkt 2
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x00
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x39
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (reasmPkt2, ".testPkts")
#endif
static uint8_t reasmPkt2[] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
  0x02, 0xe0, 0xa6, 0x66, 0x57, 0x04,
    0x86, 0xdd,
    0x60, 0x01, 0x01, 0x01,      // Iv6 header
    0x00, 0x00, 0x2b, 0xFF,
    0x20, 0x02, 0x9e, 0xda,
    0x6d, 0x30, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01,
    0x00, 0x02, 0x04, 0x08,
    0x10, 0x12, 0x14, 0x18,
    0x20, 0x22, 0x24, 0x28,
    0x30, 0x32, 0x34, 0x39,

      0x11, 0x01, 0x01, 0x00,      // Extension Route header
      0x10, 0x12, 0x14, 0x18,
      0x20, 0x22, 0x24, 0x28,
      0x30, 0x32, 0x34, 0x39,

    0xaa, 0xbb, 0x81, 0x02,
    0x00, 0x00, 0x00, 0x00

};

static pasahoLongInfo_t reasmPkt2Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,78),            /* cmd len = 14, pmatch = 1, frag = 0, start offset */
    TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
    TF_FORM_PKT_INFO_WORD2(14,70,78,0),                 /* L3 offset = 14, l4Offset = 70, l5Offset = 78, ahEspOffset = 0 */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_UDP),
                            0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
      TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)                         /* Pseudo header checksum */

};

/* reassempkt 3
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = .... 0x00, 0x00, 0x00, 0x00
 * Outer ip dest = .... 0x30, 0x32, 0x34, 0x3c
 * Designed to match IP configuration 13 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (reasmPkt3, ".testPkts")
#endif
static uint8_t reasmPkt3[] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
  0x03, 0xe0, 0xa6, 0x66, 0x57, 0x04,
    0x86, 0xdd,
    0x60, 0x01, 0x01, 0x01,      // Iv6 header
    0x00, 0x00, 0x2b, 0xFF,
    0x20, 0x02, 0x9e, 0xda,
    0x6d, 0x30, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01,
    0x00, 0x02, 0x04, 0x08,
    0x10, 0x12, 0x14, 0x18,
    0x20, 0x22, 0x24, 0x28,
    0x30, 0x32, 0x34, 0x3C,

      0x33, 0x01, 0x01, 0x00,      // Extension Route header, next header is Fragment header
      0x10, 0x12, 0x14, 0x18,
      0x20, 0x22, 0x24, 0x28,
      0x30, 0x32, 0x34, 0x3C,

      0x11, 0x04, 0x00, 0x00, 0x44, 0x44,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x66, 0x66,
      0x66, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x01,


    0xaa, 0xbb, 0x08, 0x88,
    0x00, 0x00, 0x00, 0x00

};

static pasahoLongInfo_t reasmPkt3Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,1,0,78),            /* cmd len = 14, pmatch = 1, frag = 0, start offset */
    TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
    TF_FORM_PKT_INFO_WORD2(14,94,102,70),                 /* L3 offset = 14, l4Offset = 94, l5Offset = 102, ahEspOffset = 70 */
    TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv6 | PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_AH),
                            0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
      TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
    TF_FORM_PKT_INFO_WORD5(0)                         /* Pseudo header checksum */

};


#ifdef _TMS320C6X
#pragma DATA_SECTION (t14ReasmPktInfo, ".testPkts")
#endif
static pktTestInfo2_t t14ReasmPktInfo[] =  {
    /* Packet 0 */
    {
      (uint8_t *)reasmPkt0,
      (pasahoLongInfo_t *)&reasmPkt0Info,
      sizeof(reasmPkt0),
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
        (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
      },
      T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 8 ,0),  /* Packet will be matched by pkt ID 1 */
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* AH match */
        0,
        0
      },
      -1,                    /* Not Applicable */
      T14_PKT_TYPE_IPV4 | T14_PKT_MATCH          /* Match for UDP */
    },

    /* Packet 1 */
    {
      (uint8_t *)reasmPkt1,
      (pasahoLongInfo_t *)&reasmPkt1Info,
      sizeof(reasmPkt1),
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),   /* IP match */
        (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
      },
      T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 17 ,1),  /* Packet will be matched by pkt ID 1 */
      { 0,
        0,
        0
      },
      -1,                    /* Not Applicable */
      T14_PKT_TYPE_IPV4 | T14_PKT_MATCH          /* Match for UDP */
    },

    /* Packet 2 */
    {
      (uint8_t *)reasmPkt2,
      (pasahoLongInfo_t *)&reasmPkt2Info,
      sizeof(reasmPkt2),
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),   /* IP match */
        (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
      },
      T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 1 ,2),  /* Packet will be matched by pkt ID 1 */
      { 0,
        0,
        0
      },
      -1,                    /* Not Applicable */
      T14_PKT_TYPE_IPV6 | T14_PKT_MATCH          /* Match for UDP */
    },

    /* Packet 3 */
    {
      (uint8_t *)reasmPkt3,
      (pasahoLongInfo_t *)&reasmPkt3Info,
      sizeof(reasmPkt3),
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* MAC match */
        (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV6),   /* IP match */
        (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)                                         /* UDP match */
      },
      T14_GET_UNIQUE_IDX(T14_L4_MATCH_BASE, 6 ,3),  /* Packet will be matched by pkt ID 3 */
      { (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),                                    /* AH match */
        0,
        0
      },
      -1,                    /* Not Applicable */
      T14_PKT_TYPE_IPV6 | T14_PKT_MATCH          /* Match for UDP */
    },
};

#endif /*TEST14PKTS_H_*/
