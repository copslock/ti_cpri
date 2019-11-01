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

#include "../pautest.h"

#ifdef __LINUX_USER_SPACE
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif

/* Egress Flow and Packet Forwading test
 * This test tests the LLD and firmware the ability to perform Flow Cache operation
 * including Flow Cache Classification, Egress Flow packet modification and Ingress Packet
 * Forwarding.
 *
 * This test invokes the following LLD APIs:
 *  - Pa_addMac and Pa_addMac2
 *  - Pa_addIp and Pa_addIp2
 *  - Pa_addPort and Pa_addPort2
 *  - Pa_addFc
 *  - Pa_delL4Handle
 *  - Pa_delHandle
 *  - Pa_delFcHandle
 *  - Pa_control
 *  - Pa_configExceptionRoute
 *  - Pa_configEflowExceptionRoute
 *  - Pa_configEflowRecords
 *  - Pa_forwardResult
 *  - Pa_requestStats
 *  - Pa_queryFcStats
 *  - Pa_formatTxCmd
 *
 * The test procedure and purpose is described below:
 *  - Call Pa_control to enable the RA-based IP Reassembly
 *  - Call Pa_configEflowExceptionRoute() to setup egress flow exception routes
 *  - Call Pa_configEflowRecords() to setup multiple egress paths with set of
 *    modification records.
 *  - Call Pa_addMac(2), Pa_addIp(2) and Pa_addPort(2) to setup the receiving paths
 *  - Call Pa_addFc to setup flow cache classification entries
 *  - Send variable-length packets to Egress module to verify the following operations
 *    - Flow cache classification
 *    - Inner IP/L4 header modification
 *    - IPv4 header checksum generation
 *    - TCP/UDP checksum generation
 *    - IPv4 fragmentation
 *    - IPv6 fragmentation
 *    - Outer IP insertion and/or update
 *    - IPSEC header/trail insertion
 *    - IPSEC ESP NAT-T header insertion
 *    - IPSEC AH tag insertion
 *    - L2 header insertion and/or update
 *    Both fragmented and non-fragmented packets will be delivered to PASS input queue for
 *    ingess packet classification and reassembly and then be delivered to the matching queue
 *    upon the final UDP lookup
 *  - Send variable-length packets to PASS input queue to verify packet forwarding at the
 *    following four stages:
 *    - L2 classification: for MAC-routing
 *    - Outer IP classification: IP forwarding
 *    - Inner IP classification: IP forwarding
 *    - LUT2 classification: IP forwarding and Flow cache
 *    All the forwarding packets will be delievered back to PASS input queue for another
 *    round of classification after the egress flow manipulation.
 *  - Send packets to Egress flow to test the Egress Flow exception handling
 *  - Send packets to Egress flow to test the QoS based routing
 *  - Verify the following statistics:
 *    - system statistics
 *    - RA statistics
 *    - Flow Cache statistics
 */

static char *tfName = "paTestEFlow";

#ifndef __LINUX_USER_SPACE
/*
 * This definition is used to verify the regress flow to CPSW routing.
 * When it is set, user should run the PA_emacExample to configure the CPSW and enable
 * SGMII internal loopback prior to running PA unit tests.
 *
 * Note: This flag is invalid in Linux user mode.
 */
//#define T13_TEST_EFLOW_DEST_EMAC
#endif

#define T13_NUM_PACKET_GROUPS	10 	/* Number of packet groups where each group contains packets with specified payload size */

/* General purpose queue usage */
#define Q_CMD_RECYCLE		    0	/* Command descriptors/buffers recycled here after sent to PA */
#define Q_CMD_REPLY  		    1	/* Replies from PA routed here */
#define Q_MATCH		  	        2	/* Packets from PA which match a lookup criteria */
#define Q_NFAIL		            3	/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
#define Q_PARSE_ERR		        4	/* Packets which resulted in a parse error */
#define Q_DPKT_RECYCLE		    5	/* Data packet recycle queue */

#define Q_ESP_FIX               10  /* ESP Fixup queue */
#define Q_IP_FRAG               13  /* IP Fragmentation */
#define Q_EF_EXP                14  /* Egess Flow Exception route */
#define Q_L2_RELAY              15  /* Temporary Host queue to forward L2 traffic to PASS input queue */
#define Q_BOUNCE_DDR            16  /* Queue Bounce DDR queue */
#define Q_BOUNCE_MSMC           17  /* Queue Bounce MSMC queue */
#define Q_QoS_BASE              20  /* First QoS queue */

#define T13_NUM_QoS_QUEUES      16

#define T13_CMD_SWINFO0_TYPE_MASK	        0xffff0000
#define T13_CMD_SWINFO0_ID_MASK		        0x0000ffff

/* SWInfo values on command replies */
#define T13_CMD_SWINFO0_ADD_MAC_ID  		0xa1110000
#define T13_CMD_SWINFO0_ADD_IP_ID   		0xa1120000
#define T13_CMD_SWINFO0_ADD_PORT_ID	   	    0xa1130000
#define T13_CMD_SWINFO0_ADD_FC_ID	   	    0xa1140000
#define T13_CMD_SWINFO0_GLOBAL_CFG_ID   	0xa1150000
#define T13_CMD_SWINFO0_NAT_T_CFG_ID        0xa1160000
#define T13_CMD_SWINFO0_EROUTE_CFG_ID       0xa1170000
#define T13_CMD_SWINFO0_USR_STATS_CFG_ID    0xa1180000
#define T13_CMD_SWINFO0_DEL_MAC_ID		    0xa0010000
#define T13_CMD_SWINFO0_DEL_IP_ID		    0xa0020000
#define T13_CMD_SWINFO0_DEL_PORT_ID		    0xa0030000
#define T13_CMD_SWINFO0_DEL_FC_ID		    0xa0040000
#define T13_SWINFO0_PKT_ID                  0xAAAA0000
#define T13_SWINFO0_EXP_PKT_ID              0xBBBB0000
#define T13_SWINFO0_QOS_PKT_ID              0xCCCC0000


#include "test13pkts.h"

/* The number of PA L2 and L3 handles maintained by this test */
#define T13_NUM_L2_ENTRIES                  (sizeof(t13EthSetup)/sizeof(t13EthSetup_t))
#define T13_NUM_FORWARD_L2_ENTRIES          (sizeof(t13EthSetup2)/sizeof(t13EthSetup2_t))
#define T13_NUM_L3_ENTRIES                  (sizeof(t13IpSetup)/sizeof(t13IpSetup_t))
#define T13_NUM_FORWARD_L3_ENTRIES          (sizeof(t13IpSetup2)/sizeof(t13IpSetup2_t))
#define T13_NUM_L4_ENTRIES                  (sizeof(t13UdpSetup)/sizeof(t13UdpSetup_t))
#define T13_NUM_FORWARD_L4_ENTRIES          (sizeof(t13UdpSetup2)/sizeof(t13UdpSetup2_t))

#define T13_NUM_LOCAL_L2_HANDLES   			T13_NUM_L2_ENTRIES + T13_NUM_FORWARD_L2_ENTRIES
#define T13_NUM_LOCAL_L3_HANDLES			T13_NUM_L3_ENTRIES + T13_NUM_FORWARD_L3_ENTRIES
#define T13_NUM_LOCAL_L4_HANDLES	  		T13_NUM_L4_ENTRIES + T13_NUM_FORWARD_L4_ENTRIES
#define T13_NUM_LOCAL_FC_HANDLES            (sizeof(t13FcSetup)/sizeof(t13FcSetup_t))
#define T13_NUM_GEN_CONFIGS                 10     /* Maxmium number of general configuration commands */

 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T13_HANDLE_UNCONFIGURED = 0,
	T13_HANDLE_PENDING_ACK,
	T13_HANDLE_ACTIVE,
	T13_HANDLE_DISABLED
};



 typedef struct t13Handles_s  {

  	paHandleL2L3_t  paHandle;     /* The L3 handle returned by the PA LLD */

 	unsigned int			state;		  /* T4_HANDLE_UNCONFIGURED = handle not configured
 								   * T4_HANDLE_PENDING_ACK = handle configured and sent to pa
 								   * T4_HANDLE_ACTIVE = handle creation acknowledged by pa
 								   * T4_HANDLE_DISABLED = handle was created then released */
    unsigned int            linkCnt;

 } t13Handles_t;

 typedef struct t13HandlesFc_s  {

 	paHandleFc_t   paHandle;

 	unsigned int 		   state;

 } t13HandlesFc_t;

  typedef struct t13HandlesL4_s  {

 	paHandleL4_t   paHandle;

 	unsigned int 		   state;

 } t13HandlesL4_t;


 /* A grouping of run time created grouped together to make cleanup easier on
  * error exit */

 typedef struct t13TestEncap_s  {
 	tFramework_t  *tf;
 	paTest_t      *pat;

 	/* There is one base packet for each L3 table entry */
 	Cppi_HostDesc  *hd[T13_NUM_LOCAL_L3_HANDLES];

 	/* The command to the modify PDSP to add the TCP/UDP checksum and route to PA receive */
 	uint32_t   cmdStack[T13_NUM_LOCAL_L4_HANDLES][(2 * sizeof(pasahoNextRoute_t) + sizeof(pasahoComChkCrc_t) + sizeof(pasahoIpFrag_t) + sizeof(pasahoComBlindPatch_t) + (2 * sizeof(pasahoPatchMsgLen_t))) / sizeof (uint32_t)];

 	/* The +1 is a place holder handle used to pass known invalid configuration into the PA LLD */
 	t13Handles_t    l2Handles[T13_NUM_LOCAL_L2_HANDLES];		/* MAC handles */
 	t13Handles_t    l3Handles[T13_NUM_LOCAL_L3_HANDLES];		/* IP handles  */
 	t13HandlesFc_t  fcHandles[T13_NUM_LOCAL_FC_HANDLES];	    /* Flow Cache handles */
 	t13HandlesL4_t  l4Handles[T13_NUM_LOCAL_L4_HANDLES+1];	    /* UDP/TCP handles */

    unsigned int            genCmdAck[T13_NUM_GEN_CONFIGS];             /* General configurations */


 } t13TestEncap_t;

static paSysStats_t paTestL4ExpectedStats;    /* Expected stats results */
static paRaStats_t paTestRaStats;

#define T13_NUM_USR_STATS            5
#define T13_USR_STATS_ID_EFLOW_0     0
#define T13_USR_STATS_ID_EFLOW_1     1
#define T13_USR_STATS_ID_EFLOW_2     2
#define T13_USR_STATS_ID_EFLOW_3     3
#define T13_USR_STATS_ID_EFLOW_4     4

static  pauUsrStatsEntry_t t13UsrStatsTbl[T13_NUM_USR_STATS] =
{
    /* index, lnkId, fAlloc, f64b, fByteCnt */
    {0, PAU_USR_STATS_NO_LINK, TRUE,   FALSE, FALSE},  /* EFlow 0 count */
    {0, PAU_USR_STATS_NO_LINK, TRUE,   FALSE, FALSE},  /* EFlow 1 count */
    {0, PAU_USR_STATS_NO_LINK, TRUE,   FALSE, FALSE},  /* EFlow 2 count */
    {0, PAU_USR_STATS_NO_LINK, TRUE,   FALSE, FALSE},  /* EFlow 3 count */
    {0, PAU_USR_STATS_NO_LINK, TRUE,   FALSE, FALSE},  /* EFlow 4 count */
};

/* Global configurations */
#define T13_NUM_64B_USR_STATS            64

static paUsrStatsConfig_t   t13UsrStatsCfg =
       {
            pa_USR_STATS_MAX_COUNTERS - T13_NUM_64B_USR_STATS,   /* Number of user stats (448)*/
            T13_NUM_64B_USR_STATS                                /* Number of 64-bit user stats */
       };

static  paRaGroupConfig_t t13OutRaGroupCfg =
        {
            pa_RA_CTRL_ENABLE
            #ifdef NETSS_INTERNAL_PKTDMA
            | pa_RA_CTRL_USE_LOCAL_DMA
            #endif
            ,
            //pa_RA_CTRL_TO_QUEUE,                /* ctrlBitMap */
            0,                                  /* Flow Id (1) */
            {                                   /* TimeoutER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            },

            {                                   /* CritErrER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            },

            {                                   /* genErrER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            }

        };

static  paRaGroupConfig_t t13InRaGroupCfg =
        {
            pa_RA_CTRL_ENABLE
            #ifdef NETSS_INTERNAL_PKTDMA
            | pa_RA_CTRL_USE_LOCAL_DMA
            #endif
            ,
            //pa_RA_CTRL_TO_QUEUE,                /* ctrlBitMap */
            0,                                  /* Flow Id (2) */
            {                                   /* TimeoutER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            },

            {                                   /* CritErrER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            },

            {                                   /* genErrER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            }

        };


static  paIpsecNatTConfig_t  t13NatTCfg =
        {
            pa_IPSEC_NAT_T_CTRL_ENABLE,    /* ctrlBitMap */
            4500                           /* UDP port number */
        };

static	paQueueBounceConfig_t t13QueueBounceCfg =
	        {
	            1,      /* Enable */
                Q_BOUNCE_DDR  + TF_FIRST_GEN_QUEUE,             /* ddrQueueId */
                Q_BOUNCE_MSMC + TF_FIRST_GEN_QUEUE,             /* msmcQueueId */
                TF_PA_TX_QUEUE_BASE,                            /* hwQueueBegin */
                TF_PA_TX_QUEUE_BASE + NSS_NUM_TX_QUEUES - 1,    /* hwQueueEnd */
                {
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Command Return */
                    pa_QUEUE_BOUNCE_OP_MSMC,    /* QoS mode */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Capture Capture */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* IP Reassembly-assisted packets */
                    pa_QUEUE_BOUNCE_OP_DDR      /* All traffics */
                }
	        };



static  paSysConfig_t  t13GlobalCfg =
        {
            NULL,               /* pProtoLimit */
            NULL,               /* pOutIpReassmConfig */
            NULL,               /* pInIpReassmConfig */
            NULL,               /* pCmdSetConfig */
            &t13UsrStatsCfg,    /* pUsrStatsConfig */
            NULL,               /* pQueueDivertConfig */
            NULL,               /* pPktControl */
            &t13QueueBounceCfg, /* pQueueBounceConfig */
            NULL,               /* pOutAclConfig */
            NULL,               /* pInAclConfig */
            &t13OutRaGroupCfg,  /* pOutIpRaGroupConfig */
            &t13InRaGroupCfg    /* pInIpRaGroupConfig */
        };

#define T13_NUM_EXCEPTION_ROUTES    1
 static int t13ErouteTypes[] = {
    pa_EROUTE_NAT_T_DATA
 };

 static paRouteInfo_t t13Eroutes[] = {

    /* NAT-T Data  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   TF_PA_QUEUE_OUTER_IP,/* queue */
 	   -1,					/* Multi route */
 	   0xbabeface,          /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL                 /* No commands */
    }

 };


#define T13_NUM_EF_EXCEPTION_ROUTES    4
 static int t13EfErouteTypes[] = {
    pa_EFLOW_EROUTE_IP_FRAG,
    pa_EFLOW_EROUTE_IP_OPTIONS,
    pa_EFLOW_EROUTE_IP_EXPIRE,
    pa_EFLOW_EROUTE_TCP_CTRL
 };

 static paRouteInfo_t t13EfEroutes[] = {

    /* IP Frag  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_EF_EXP + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T13_SWINFO0_EXP_PKT_ID,     /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL                 /* No commands */
    },

    /* IP Options  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_EF_EXP + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T13_SWINFO0_EXP_PKT_ID + 1,     /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL                 /* No commands */
    },

    /* IP Expire  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_EF_EXP + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T13_SWINFO0_EXP_PKT_ID + 2,     /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL                 /* No commands */
    },

    /* TCP Control  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_EF_EXP + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T13_SWINFO0_EXP_PKT_ID + 3,     /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL                 /* No commands */
    }

 };



static paCmdReply_t cmdReply = {  pa_DEST_HOST,				/* Dest */
 							      0,						/* Reply ID (returned in swinfo0) */
 							   	  0,						/* Queue */
 							      0 };						/* Flow ID */

static paRouteInfo_t matchRoute[5] = {  {  	pa_DEST_HOST,		/* Dest */
 								       		0,					/* Flow ID */
 								       		0,					/* queue */
 								       	   -1,					/* Multi route */
 								       		0,					/* sw Info 0 */
                                            0,                  /* sw Info 1 */
                                            0,                  /* customType : not used  */
                                            0,                  /* customIndex: not used  */
                                            0,                  /* pkyType: for SRIO only */
                                            NULL},              /* No commands            */

 								         {  pa_DEST_CONTINUE_PARSE_LUT1,/* Dest */
 								       		0,							/* Flow ID */
 								       		0,							/* queue */
 								           -1,							/* Multi route */
 								        	0,							/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL},                      /* No commands            */

 								         {  pa_DEST_CONTINUE_PARSE_LUT2,/* Dest */
 								       		0,							/* Flow ID */
 								       		0,							/* queue */
 								           -1,							/* Multi route */
 								        	0,							/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL},                      /* No commands            */

                                         {  pa_DEST_HOST,		/* Dest */
 								       		0,					/* Flow ID */
 								       		0,					/* queue */
 								       	   -1,					/* Multi route */
 								       		0,					/* sw Info 0 */
                                            0,                  /* sw Info 1 */
                                            0,                  /* customType : not used  */
                                            0,                  /* customIndex: not used  */
                                            0,                  /* pkyType: for SRIO only */
                                            NULL},              /* No commands            */

                                         {  pa_DEST_HOST,		/* Dest */
 								       		0,					/* Flow ID */
 								       		TF_PA_QUEUE_IPSEC2,	/* queue */
 								       	   -1,					/* Multi route */
 								       		0,					/* sw Info 0 */
                                            0,                  /* sw Info 1 */
                                            0,                  /* customType : not used  */
                                            0,                  /* customIndex: not used  */
                                            0,                  /* pkyType: for SRIO only */
                                            NULL},              /* No commands            */


 								      };
static paRouteInfo_t   nfailRoute = {  pa_DEST_DISCARD,		/* Dest */
 										0,					/* Flow ID */
 										0,					/* queue */
 										-1,					/* Multi route */
 										0,					/* sw Info 0 */
                                        0,                  /* sw Info 1 */
                                        0,                  /* customType : not used  */
                                        0,                  /* customIndex: not used  */
                                        0,                  /* pkyType: for SRIO only */
                                        NULL};              /* No commands            */

static paEfOpInfo_t  t13efOpInfo[] = {
                                        /* Egress Flow operation info 0 */
                                        {
                                            0,                                  /* ctrlFlags */
                                            pa_EF_OP_INFO_VALID_LVL4,           /* validBitmap */
                                            0,                                  /* lvl1Index */
                                            0,                                  /* lvl2Index */
                                            0,                                  /* lvl3Index */
                                            11                                  /* lvl4Index */
                                        },

                                        /* Egress Flow operation info 1 */
                                        {
                                            0,                                  /* ctrlFlags */
                                            pa_EF_OP_INFO_VALID_LVL2   |        /* validBitmap */
                                            pa_EF_OP_INFO_VALID_LVL4,
                                            0,                                  /* lvl1Index */
                                            20,                                 /* lvl2Index */
                                            0,                                  /* lvl3Index */
                                            11                                  /* lvl4Index */
                                        },

                                        /* Egress Flow operation info 2 */
                                        {
                                            0,                                  /* ctrlFlags */
                                            pa_EF_OP_INFO_VALID_LVL2   |        /* validBitmap */
                                            pa_EF_OP_INFO_VALID_LVL4,
                                            0,                                  /* lvl1Index */
                                            20,                                 /* lvl2Index */
                                            0,                                  /* lvl3Index */
                                            11                                  /* lvl4Index */
                                        },

                                        /* Egress Flow operation info 3 */
                                        {
                                            pa_EF_OP_CMD_FC_LOOKUP,             /* ctrlFlags */
                                            0,                                  /* validBitmap */
                                            0,                                  /* lvl1Index */
                                            0,                                  /* lvl2Index */
                                            0,                                  /* lvl3Index */
                                            0                                   /* lvl4Index */
                                        },

                                        /* Egress Flow operation info 4 */
                                        {
                                            0,                                  /* ctrlFlags */
                                            pa_EF_OP_INFO_VALID_LVL1   |        /* validBitmap */
                                            pa_EF_OP_INFO_VALID_LVL2   |
                                            pa_EF_OP_INFO_VALID_LVL4,
                                            75,                                 /* lvl1Index */
                                            20,                                 /* lvl2Index */
                                            0,                                  /* lvl3Index */
                                            11                                  /* lvl4Index */
                                        }
                                     };

static paRouteInfo2_t matchRoute2[] = {
                                         {
                                            pa_ROUTE_INFO_VALID_PCMD,   /* validBitMap */
                                            pa_DEST_EFLOW,		        /* Dest */
 								       		0,					        /* Flow ID */
 								       		0,					        /* queue */
 								       	    0,					        /* Multi route */
 								       		0, 	                        /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            &t13efOpInfo[0]             /* Egress Flow Info */
                                         },

                                         {
                                            pa_ROUTE_INFO_VALID_PCMD,   /* validBitMap */
                                            pa_DEST_EFLOW,		        /* Dest */
 								       		0,					        /* Flow ID */
 								       		0,					        /* queue */
 								       	    0,					        /* Multi route */
 								       		0, 	                        /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            &t13efOpInfo[1]             /* Egress Flow Info */
                                         },

                                         {
                                            pa_ROUTE_INFO_VALID_PCMD,   /* validBitMap */
                                            pa_DEST_EFLOW,		        /* Dest */
 								       		0,					        /* Flow ID */
 								       		0,					        /* queue */
 								       	    0,					        /* Multi route */
 								       		0, 	                        /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            &t13efOpInfo[2]             /* Egress Flow Info */
                                         },

                                         {
                                            pa_ROUTE_INFO_VALID_PCMD,   /* validBitMap */
                                            pa_DEST_EFLOW,		        /* Dest */
 								       		0,					        /* Flow ID */
 								       		0,					        /* queue */
 								       	    0,					        /* Multi route */
 								       		0, 	                        /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            &t13efOpInfo[3]             /* Egress Flow Info */
                                         },

                                         {
                                            pa_ROUTE_INFO_VALID_PCMD,   /* validBitMap */
                                            pa_DEST_EFLOW,		        /* Dest */
 								       		0,					        /* Flow ID */
 								       		0,					        /* queue */
 								       	    0,					        /* Multi route */
 								       		0, 	                        /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            &t13efOpInfo[4]             /* Egress Flow Info */
                                         }

                                      };

static paRouteInfo2_t nfailRoute2 =  {
                                    0,                   /* validBitMap            */
                                    pa_DEST_DISCARD,	/* Dest */
								    0,					/* Flow ID */
								    0,					/* queue */
								    -1,					/* Multi route */
								    0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */
                                    0,                  /* customType : not used  */
                                    0,                  /* customIndex: not used  */
                                    0,                  /* pkyType: for SRIO only */
                                    NULL,               /* No commands            */
                                    0,                  /* Priority Type          */
                                    NULL                /* efOpInfo */
						       	   };

/*
 * User-definded Statitics Map
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (t13UsrStatsGroup1, ".testPkts")
#endif

static paUsrStatsCounterEntryConfig_t t13UsrStatsGroup1[T13_NUM_USR_STATS];

#ifdef _TMS320C6X
#pragma DATA_SECTION(t13UsrStatsSetup, ".testPkts")
#endif
static pauUsrStatsSetup_t  t13UsrStatsSetup[] = {
    /* entry 0 */
    {
        sizeof(t13UsrStatsGroup1)/sizeof(paUsrStatsCounterEntryConfig_t),    /* number of entries */
        t13UsrStatsGroup1,                                                   /* counter Info table */
        pa_OK                                                               /* Expected return value */
    }
};


#ifdef _TMS320C6X
#pragma DATA_SECTION(t13Encap, ".testPkts")
#endif
static t13TestEncap_t  t13Encap;
static paUsrStats_t    paTestExpectedUsrStats;


#ifdef __LINUX_USER_SPACE

int setupPktTestInfo2(pktTest13Info_t* testInfoPkt, int count, char* tfname)
{
	int i,pktSz;
	uint8_t *pkt;
	int returnVal = 0;

	for (i = 0; i < count; i++)  {
		/* Store the original information */
        pkt   = testInfoPkt[i].pktInfo.pkt;
		pktSz = testInfoPkt[i].pktInfo.pktLen;
        /* Allocate memory for the Packet buffers from Framework, to control phy2Virt and Virt2Phy */
        testInfoPkt[i].pktInfo.pkt = (uint8_t *)fw_memAlloc(pktSz, CACHE_LINESZ);
        if(testInfoPkt[i].pktInfo.pkt == NULL) {
  	        printf ("%s: memAlloc failed for pkt %d\n", tfname, i);
  	        returnVal = -1;
        }
		/* Restore the original pkt information in the allocated memory */
        memcpy(testInfoPkt[i].pktInfo.pkt, pkt, pktSz);
    }

	return (returnVal);
}

#endif

static void t13Cleanup (t13TestEncap_t *tencap, paTestStatus_t status)
{
	int			   i;
	int  	       cmdDest;
 	uint16_t	       cmdSize;
 	paReturn_t     paret;
 	paTestStatus_t newStatus;
 	Cppi_HostDesc *hd;

	/* Wait a bit for any packets in PA to complete */
	utilCycleDelay (5000);

 	/* Delete active L4 handles */
 	for (i = 0; i < T13_NUM_LOCAL_L4_HANDLES; i++)  {

		cmdReply.replyId = T13_CMD_SWINFO0_DEL_PORT_ID + i;
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l4Handles[i].state == T13_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T13_HANDLE_ACTIVE))  {
			hd = testCommonDelL4Handles (tencap->tf, tencap->l4Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				status = PA_TEST_FAILED;
				continue;
			}

			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		    paTestL4ExpectedStats.classify2.nPackets += 1;
 		    paTestL4ExpectedStats.classify1.nPackets += 1;

			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				status = PA_TEST_FAILED;
 			}
 			else
            {
 			    /* Recycle the command packet as well */
 			    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			    if (hd == NULL)  {
 				    System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				    status = PA_TEST_FAILED;
 				    continue;
 			    }
 			    testCommonRecycleLBDesc (tencap->tf, hd);
            }
 		}
 	}

  	/* Delete active L3 Handles */
 	for (i = T13_NUM_LOCAL_L3_HANDLES - 1; i >= 0; i--)  {
 		cmdReply.replyId = T13_CMD_SWINFO0_DEL_IP_ID + i;
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l3Handles[i].state == T13_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T13_HANDLE_ACTIVE))  {
			hd = testCommonDelHandle (tencap->tf, &tencap->l3Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				status = PA_TEST_FAILED;
				continue;
			}

			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
			paTestL4ExpectedStats.classify1.nPackets += 1;

			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				status = PA_TEST_FAILED;
 			}
            else
            {
 			    /* Recycle the command packet as well */
 			    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			    if (hd == NULL)  {
 				    System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				    status = PA_TEST_FAILED;
 				    continue;
 			    }
 			    testCommonRecycleLBDesc (tencap->tf, hd);
            }
 		}
 	}

  	/* Delete active L2 Handles */
 	for (i = 0; i < T13_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T13_CMD_SWINFO0_DEL_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l2Handles[i].state == T13_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T13_HANDLE_ACTIVE))  {
			hd = testCommonDelHandle (tencap->tf, &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				status = PA_TEST_FAILED;
				testCommonRecycleLBDesc (tencap->tf, hd);
				continue;
			}

			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
			paTestL4ExpectedStats.classify1.nPackets += 1;

			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				status = PA_TEST_FAILED;
 			}
 			else
            {
 			    /* Recycle the command packet as well */
 			    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			    if (hd == NULL)  {
 				    System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				    continue;
 			    }
 			    testCommonRecycleLBDesc (tencap->tf, hd);
            }
 		}
 	}

  	/* Delete active FC Handles */
 	for (i = 0; i < T13_NUM_LOCAL_FC_HANDLES; i++)  {
 		cmdReply.replyId = T13_CMD_SWINFO0_DEL_FC_ID + i;
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->fcHandles[i].state == T13_HANDLE_PENDING_ACK) || (tencap->fcHandles[i].state == T13_HANDLE_ACTIVE))  {
			hd = testCommonDelFcHandle (tencap->tf, &tencap->fcHandles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				status = PA_TEST_FAILED;
				continue;
			}

			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				status = PA_TEST_FAILED;
 			}
            else
            {
 			    /* Recycle the command packet as well */
 			    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			    if (hd == NULL)  {
 				    System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				    status = PA_TEST_FAILED;
 				    continue;
 			    }
 			    testCommonRecycleLBDesc (tencap->tf, hd);
            }
 		}
 	}

  	/* Pop any descriptors off of the return queues and restore them to the linked buffer queues or the free Q */
 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_CMD_RECYCLE])> 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 		testCommonRecycleLBDesc (tencap->tf, hd);
 	}

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_CMD_REPLY]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_REPLY])) & ~15);
 		testCommonRecycleLBDesc (tencap->tf, hd);
 	}

 	 while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_MATCH]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
 		testCommonRecycleLBDesc (tencap->tf, hd);
 	}

 	 while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_NFAIL]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_NFAIL])) & ~15);
 		testCommonRecycleLBDesc (tencap->tf, hd);
 	}

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_PARSE_ERR]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_PARSE_ERR])) & ~15);
 		testCommonRecycleLBDesc (tencap->tf, hd);
 	}

  	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
 		Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

 	newStatus = testCommonCheckStats (tencap->tf, tencap->pat, tfName, &paTestL4ExpectedStats, tencap->tf->QLinkedBuf1,
	                       			  tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QGen[Q_CMD_REPLY], TRUE);

	if (newStatus == PA_TEST_FAILED)
		status = PA_TEST_FAILED;

	/* Return result */
    tencap->pat->testStatus = status;

    /* Return */
    Task_exit();
}

static int t13CmdRep (t13TestEncap_t *tencap)
{
	Cppi_HostDesc  *hd;
	uint32_t         *swinfo;
	paReturn_t      paret;
	uint32_t			swinfoType;
	uint32_t			swinfoIdx;
	paEntryHandle_t reth;
    int			    htype;
    int             cmdDest;
    char		   *s;
    unsigned int		   *stateP;
    unsigned int			stateV;
    int				max;

	while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_CMD_REPLY]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_REPLY])) & ~15);

		    Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);

		    swinfoType = swinfo[0] & T13_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T13_CMD_SWINFO0_ID_MASK;

		    paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);


		    switch (swinfoType)  {

		    	case T13_CMD_SWINFO0_ADD_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T13_HANDLE_ACTIVE;
		    		max = T13_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_addMac";
		    		break;

		    	case T13_CMD_SWINFO0_ADD_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T13_HANDLE_ACTIVE;
		    		max = T13_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_addIp";
		    		break;

		    	case T13_CMD_SWINFO0_ADD_FC_ID:
		    		stateP = &tencap->fcHandles[swinfoIdx].state;
		    		stateV = T13_HANDLE_ACTIVE;
		    		max = T13_NUM_LOCAL_FC_HANDLES;
		    		s = "pa_addFc";
		    		break;


		    	case T13_CMD_SWINFO0_ADD_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T13_HANDLE_ACTIVE;
		    		max = T13_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_addPort";
		    		break;

		    	case T13_CMD_SWINFO0_DEL_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T13_HANDLE_DISABLED;
		    		max = T13_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_delMac";
		    		break;

		    	case T13_CMD_SWINFO0_DEL_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T13_HANDLE_DISABLED;
		    		max = T13_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_delIp";
		    		break;

		    	case T13_CMD_SWINFO0_DEL_FC_ID:
		    		stateP = &tencap->fcHandles[swinfoIdx].state;
		    		stateV = T13_HANDLE_DISABLED;
		    		max = T13_NUM_LOCAL_FC_HANDLES;
		    		s = "pa_delFc";
		    		break;


		    	case T13_CMD_SWINFO0_DEL_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T13_HANDLE_DISABLED;
		    		max = T13_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_delPort";
		    		break;

                case T13_CMD_SWINFO0_GLOBAL_CFG_ID:
                case T13_CMD_SWINFO0_NAT_T_CFG_ID:
                case T13_CMD_SWINFO0_EROUTE_CFG_ID:
                    stateP = &tencap->genCmdAck[swinfoIdx];
                    stateV = TRUE;
                    max = T13_NUM_GEN_CONFIGS;
                    s = "Pa_control";
                    break;

		    	default:
		    		System_printf ("%s (%s:%d): Unknown command ID found in swinfo0 (0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
                    return (-1);

		    }

		    /* In this test only valid responses are exected from PA */
		    if (paret != pa_OK)  {
		    	System_printf ("%s (%s:%d): PA command %s returned error code %d, Index = %d\n", tfName, __FILE__, __LINE__, s, paret, swinfoIdx);
                return (-1);
		    }

		   	if (swinfoIdx >= max)  {
		   		System_printf ("%s (%s:%d): Received command ack (%s) for out of range handle (%d, max = %d)\n",
		   						tfName, __FILE__, __LINE__, s, swinfoIdx, max);
                return (-1);
		   	}

		   	/* Response looks valid. Change the internal handle state */
		   	*stateP = stateV;

            /* Return the descriptor used to received the reply from PA */
            testCommonRecycleLBDesc (tencap->tf, hd);
	}

    /* Recycle the command packet */
	while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_CMD_RECYCLE]) > 0)  {
 			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			if (hd == NULL)  {
 				System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				continue;
 			}
 			testCommonRecycleLBDesc (tencap->tf, hd);
    }

    return (0);

}

static paTestStatus_t t13GlobalConfiguration (t13TestEncap_t *tencap)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paCtrlInfo_t    ctrlInfo;

    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = t13GlobalCfg;
    cmdReply.replyId  = T13_CMD_SWINFO0_GLOBAL_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
 	hd = testCommonGlobalConfig (tencap->tf, &ctrlInfo,
 	                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);

   if (hd == NULL)  {

   	 System_printf ("%s: (%s:%d): Failure in GlobalConfig command\n", tfName, __FILE__, __LINE__);
   	 return (PA_TEST_FAILED);
   }

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

    tencap->genCmdAck[0] = FALSE;

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		if(t13CmdRep (tencap))
            return (PA_TEST_FAILED);

		if (tencap->genCmdAck[0])
			break;
		else
			utilCycleDelay (500);
	}

	if (i == 100)  {
		System_printf ("%s: (%s:%d): Pa_control commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}

static paTestStatus_t t13NatTConfiguration (t13TestEncap_t *tencap, int enable)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    paCtrlInfo_t    ctrlInfo;

    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_IPSEC_NAT_T_CONFIG;
    ctrlInfo.params.ipsecNatTDetCfg = t13NatTCfg;
    ctrlInfo.params.ipsecNatTDetCfg.ctrlBitMap = enable?pa_IPSEC_NAT_T_CTRL_ENABLE:0;
    cmdReply.replyId  = T13_CMD_SWINFO0_NAT_T_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
 	hd = testCommonGlobalConfig (tencap->tf, &ctrlInfo,
 	                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);

   if (hd == NULL)  {

   	 System_printf ("%s: (%s:%d): Failure in NAT-T Config command\n", tfName, __FILE__, __LINE__);
   	 return (PA_TEST_FAILED);
   }

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	paTestL4ExpectedStats.classify2.nPackets += 1;
 	paTestL4ExpectedStats.classify1.nPackets += 1;


    tencap->genCmdAck[0] = FALSE;

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		if(t13CmdRep (tencap))
            return (PA_TEST_FAILED);

		if (tencap->genCmdAck[0])
			break;
		else
			utilCycleDelay (500);
	}

	if (i == 100)  {
		System_printf ("%s: (%s:%d): Pa_control (NAT-T) commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}

static paTestStatus_t t13ExceptionRoutes (t13TestEncap_t *tencap)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;

    /* Issue the exception route command */
    cmdReply.replyId  = T13_CMD_SWINFO0_EROUTE_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
    for(i = 0; i < T13_NUM_EXCEPTION_ROUTES; i++)
        t13Eroutes[i].flowId = tencap->tf->tfFlowNum[0];

 	hd = testCommonConfigExceptionRoute (tencap->tf, T13_NUM_EXCEPTION_ROUTES, t13ErouteTypes, t13Eroutes,
 	                                     tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                                     &cmdReply, &cmdDest, &cmdSize, &paret, FALSE);

   if (hd == NULL)  {

   	 System_printf ("%s: (%s:%d): Failure in ConfigExceptionRoute command\n", tfName, __FILE__, __LINE__);
   	 return (PA_TEST_FAILED);

   }

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

    tencap->genCmdAck[0] = FALSE;

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		if(t13CmdRep (tencap))
            return (PA_TEST_FAILED);

		if (tencap->genCmdAck[0])
			break;
		else
			utilCycleDelay (500);
	}

	if (i == 100)  {
		System_printf ("%s: (%s:%d): pa_ConfigExceptionRoute commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}

static paTestStatus_t t13EflowExceptionRoutes (t13TestEncap_t *tencap)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;

    /* Issue the exception route command */
    cmdReply.replyId  = T13_CMD_SWINFO0_EROUTE_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
    for(i = 0; i < T13_NUM_EF_EXCEPTION_ROUTES; i++)
        t13EfEroutes[i].flowId = tencap->tf->tfFlowNum[0];

 	hd = testCommonConfigExceptionRoute (tencap->tf, T13_NUM_EF_EXCEPTION_ROUTES, t13EfErouteTypes, t13EfEroutes,
 	                                     tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                                     &cmdReply, &cmdDest, &cmdSize, &paret, TRUE);

   if (hd == NULL)  {

   	 System_printf ("%s: (%s:%d): Failure in ConfigEfExceptionRoute command\n", tfName, __FILE__, __LINE__);
   	 return (PA_TEST_FAILED);

   }

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

    tencap->genCmdAck[0] = FALSE;

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		if(t13CmdRep (tencap))
            return (PA_TEST_FAILED);

		if (tencap->genCmdAck[0])
			break;
		else
			utilCycleDelay (500);
	}

	if (i == 100)  {
		System_printf ("%s: (%s:%d): Pa_configEflowExceptionRoute commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}


static paTestStatus_t t13EflowConfiguration (t13TestEncap_t *tencap)
{
    int i, nAlloc;
    int n = 0;
    paReturn_t ret;

    /* Construct the Egress Flow records */
    for (i = 0; (i < sizeof(t13EfRec1Setup)/sizeof(t13EfRec1Setup_t)) && (n < T13_MAX_EF_RECORDS); i++, n++)
    {
        t13EfRecs[n].ctrlBitMap = pa_EF_RECORD_CONTROL_ENABLE;
        t13EfRecs[n].index = t13EfRec1Setup[i].index;
        t13EfRecs[n].type  = pa_EFLOW_REC_TYPE_LVL1;
        t13EfRecs[n].u.level1 = t13EfRec1Setup[i].rec;
    }

    for (i = 0; (i < sizeof(t13EfRec2Setup)/sizeof(t13EfRec2Setup_t)) && (n < T13_MAX_EF_RECORDS); i++, n++)
    {
        t13EfRec2Setup[i].rec.ipsec.flowId = tencap->tf->tfFlowNum[0];
        t13EfRecs[n].ctrlBitMap = pa_EF_RECORD_CONTROL_ENABLE;
        t13EfRecs[n].index = t13EfRec2Setup[i].index;
        t13EfRecs[n].type  = pa_EFLOW_REC_TYPE_LVL2;
        t13EfRecs[n].u.level2 = t13EfRec2Setup[i].rec;
    }

    for (i = 0; (i < sizeof(t13EfRec3Setup)/sizeof(t13EfRec3Setup_t)) && (n < T13_MAX_EF_RECORDS); i++, n++)
    {
        t13EfRec3Setup[i].rec.ipsec.flowId = tencap->tf->tfFlowNum[0];
        t13EfRecs[n].ctrlBitMap = pa_EF_RECORD_CONTROL_ENABLE;
        t13EfRecs[n].index = t13EfRec3Setup[i].index;
        t13EfRecs[n].type  = pa_EFLOW_REC_TYPE_LVL3;
        t13EfRecs[n].u.level3 = t13EfRec3Setup[i].rec;
    }

    for (i = 0; (i < sizeof(t13EfRec4Setup)/sizeof(t13EfRec4Setup_t)) && (n < T13_MAX_EF_RECORDS); i++, n++)
    {
        t13EfRec4Setup[i].rec.flowId = tencap->tf->tfFlowNum[0];
        t13EfRecs[n].ctrlBitMap = pa_EF_RECORD_CONTROL_ENABLE;
        t13EfRecs[n].index = t13EfRec4Setup[i].index;
        t13EfRecs[n].type  = pa_EFLOW_REC_TYPE_LVL4;
        t13EfRecs[n].u.level4 = t13EfRec4Setup[i].rec;
    }

    ret = Pa_configEflowRecords(tencap->tf->passHandle, n, &nAlloc, t13EfRecs);

    if (nAlloc < n)
    {
		System_printf ("%s: (%s:%d) t13EflowConfiguration:  %d out of %d Egress Flow records configured", tfName, __FILE__, __LINE__, nAlloc, n);
        System_flush();
    }

    if (ret != pa_OK)
    {
		System_printf ("%s: (%s:%d) Pa_configEflowRecords return error code = %d", tfName, __FILE__, __LINE__, ret);
        System_flush();
        return(PA_TEST_FAILED);

    }
    else
        return(PA_TEST_PASSED);

}

static void t13DispFcStats(t13TestEncap_t *tencap)
{
    int i;
    paReturn_t paret;
    paFcStats_t fcStats;

 	for (i = 0; i < T13_NUM_LOCAL_FC_HANDLES; i++)  {

		if ( tencap->fcHandles[i].state == T13_HANDLE_ACTIVE)  {

            paret = Pa_queryFcStats(tencap->tf->passHandle, tencap->fcHandles[i].paHandle, 0,  &fcStats);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): Pa_queryFcStats returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				continue;
			}

            System_printf ("FC Stats (%d): nMatchPackets = %d\n", i, fcStats.nMatchPackets);
 		}
 	}

    System_flush();
}

/* Simple User-Statistics routine: It does not process link and counter type */
static void t13UpdateUsrStats(paUsrStats_t* pStats, uint16_t cntIndex, uint32_t size)
{
    if (cntIndex < T13_NUM_64B_USR_STATS)
    {
        pStats->count64[cntIndex] += (uint64_t)size ;
    }
    else
    {
        pStats->count32[cntIndex - T13_NUM_64B_USR_STATS] += size;
    }
}

static paTestStatus_t t13OpenL2 (t13TestEncap_t *tencap, t13EthSetup_t *ethSetup, int nL2Handles)
{
	int 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL2Handles; i++)  {
		cmdReply.replyId = T13_CMD_SWINFO0_ADD_MAC_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddMac (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&ethSetup[i].ethInfo, &matchRoute[1], &nfailRoute,
 	    	                   &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
 	        	               &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addMac command, entry number %d\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l2Handles[i].state = T13_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    if(t13CmdRep (tencap))
                return (PA_TEST_FAILED);
            if (tencap->l2Handles[i].state == T13_HANDLE_ACTIVE)
            {
                ethSetup[i].acked = TRUE;
                break;
            }
	    }

	    if (j == 100)  {
		    System_printf ("%s: (%s:%d): pa_addMac command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
	    }

	}

	return (PA_TEST_PASSED);
}

static paTestStatus_t t13OpenL2_2 (t13TestEncap_t *tencap, t13EthSetup2_t *ethSetup, int nL2Handles)
{
	int 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL2Handles; i++)  {
		cmdReply.replyId = T13_CMD_SWINFO0_ADD_MAC_ID + T13_NUM_L2_ENTRIES + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddMac2 (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&ethSetup[i].ethInfo, &matchRoute2[ethSetup[i].routeIdx], &nfailRoute2,
 	    	                    NULL, &tencap->l2Handles[i+T13_NUM_L2_ENTRIES].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
 	        	                &cmdReply, &cmdDest, &cmdSize, 0, &paret);

		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addMac command, entry number %d\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l2Handles[i+T13_NUM_L2_ENTRIES].state = T13_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    if(t13CmdRep (tencap))
                return (PA_TEST_FAILED);
            if (tencap->l2Handles[i+T13_NUM_L2_ENTRIES].state == T13_HANDLE_ACTIVE)
            {
                ethSetup[i].acked = TRUE;
                break;
            }
	    }

	    if (j == 100)  {
		    System_printf ("%s: (%s:%d): pa_addMac2 command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
	    }
	}

	return (PA_TEST_PASSED);
}


static paTestStatus_t t13OpenL3 (t13TestEncap_t *tencap, t13IpSetup_t *ipSetup, int nL3Handles)
{
	int 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL3Handles; i++)  {
		cmdReply.replyId = T13_CMD_SWINFO0_ADD_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddIp (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, &ipSetup[i].ipInfo,
                              &matchRoute[ipSetup[i].nextRoute],
                              &nfailRoute,
							  &tencap->l3Handles[i].paHandle,
							  ipSetup[i].innerIp?tencap->l3Handles[ipSetup[i].lHandleIdx].paHandle:
                                                 tencap->l2Handles[ipSetup[i].lHandleIdx].paHandle,
							  tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf2,
							  &cmdReply, &cmdDest, &cmdSize, &paret);


		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addIp command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t13Cleanup (tencap, PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[i].state = T13_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    if(t13CmdRep (tencap))
                return (PA_TEST_FAILED);
            if (tencap->l3Handles[i].state == T13_HANDLE_ACTIVE)
            {
                ipSetup[i].acked = TRUE;
                break;
            }
	    }

	    if (j == 100)  {
		    System_printf ("%s: (%s:%d): pa_addIp command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
	    }
	}

	return (PA_TEST_PASSED);
}

static paTestStatus_t t13OpenL3_2 (t13TestEncap_t *tencap, t13IpSetup2_t *ipSetup, int nL3Handles)
{
	int 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL3Handles; i++)  {
		cmdReply.replyId = T13_CMD_SWINFO0_ADD_IP_ID + T13_NUM_L3_ENTRIES + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddIp3 (tencap->tf, pa_LUT_INST_NOT_SPECIFIED, pa_LUT1_INDEX_NOT_SPECIFIED, &ipSetup[i].ipInfo,
                              &matchRoute2[ipSetup[i].routeIdx],
                              &nfailRoute2,
							  &tencap->l3Handles[T13_NUM_L3_ENTRIES + i].paHandle,
							  ipSetup[i].innerIp?tencap->l3Handles[ipSetup[i].lHandleIdx].paHandle:
                                                 tencap->l2Handles[ipSetup[i].lHandleIdx].paHandle,
                              NULL,
							  tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf2,
							  &cmdReply, &cmdDest, &cmdSize, 0, &paret);


		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addIp command, entry number %d\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[T13_NUM_L3_ENTRIES + i].state = T13_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    if(t13CmdRep (tencap))
                return (PA_TEST_FAILED);
            if (tencap->l3Handles[T13_NUM_L3_ENTRIES + i].state == T13_HANDLE_ACTIVE)
            {
                ipSetup[i].acked = TRUE;
                break;
            }
	    }

	    if (j == 100)  {
		    System_printf ("%s: (%s:%d): pa_addIp2 command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
	    }
	}

	return (PA_TEST_PASSED);
}


static paTestStatus_t t13OpenFc (t13TestEncap_t *tencap, t13FcSetup_t *fcSetup, int nFcHandles)
{
	int 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nFcHandles; i++)  {
		cmdReply.replyId = T13_CMD_SWINFO0_ADD_FC_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddFc (tencap->tf, fcSetup[i].index, &fcSetup[i].efOpInfo, &fcSetup[i].fcInfo,
							  &tencap->fcHandles[i].paHandle,
							  tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
							  &cmdReply, &cmdDest, &cmdSize, &paret);


		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addFc command, entry number %d with error code = %d", tfName, __FILE__, __LINE__, i, paret);
		    return (PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->fcHandles[i].state = T13_HANDLE_PENDING_ACK;

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    t13CmdRep (tencap);
            if (tencap->fcHandles[i].state == T13_HANDLE_ACTIVE)
            {
                fcSetup[i].acked = TRUE;
                break;
            }
	    }

	    if (j == 100)  {
		    System_printf ("%s: (%s:%d): pa_addFc command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
	    }
	}

	return (PA_TEST_PASSED);
}

static paTestStatus_t t13OpenL4 (t13TestEncap_t *tencap, t13UdpSetup_t *udpSetup, int nL4Handles)
{
	int 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL4Handles; i++)  {

		cmdReply.replyId  = T13_CMD_SWINFO0_ADD_PORT_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
        matchRoute[0].swInfo0 = T13_SWINFO0_PKT_ID + i;

		hd = testCommonAddPort (tencap->tf, pa_LUT2_PORT_SIZE_16, udpSetup[i].port, &matchRoute[0], &tencap->l4Handles[i].paHandle,
								&tencap->l3Handles[udpSetup[i].lHandleIdx].paHandle,
                                tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addPort command, entry number %d\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		paTestL4ExpectedStats.classify2.nPackets += 1;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
 		tencap->l4Handles[i].state = T13_HANDLE_PENDING_ACK;

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    t13CmdRep (tencap);
            if (tencap->l4Handles[i].state == T13_HANDLE_ACTIVE)
            {
                udpSetup[i].acked = TRUE;
                break;
            }
	    }

	    if (j == 100)  {
		    System_printf ("%s: (%s:%d): pa_addPort command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
	    }
	}

	return (PA_TEST_PASSED);
}


static paTestStatus_t t13OpenL4_2 (t13TestEncap_t *tencap, t13UdpSetup2_t *udpSetup, int nL4Handles)
{
	int 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL4Handles; i++)  {

		cmdReply.replyId  = T13_CMD_SWINFO0_ADD_PORT_ID + T13_NUM_L4_ENTRIES + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
        matchRoute[0].swInfo0 = T13_SWINFO0_PKT_ID + i;

		hd = testCommonAddPort3 (tencap->tf, pa_LUT2_PORT_SIZE_16, udpSetup[i].port, FALSE, pa_PARAMS_NOT_SPECIFIED, &matchRoute2[udpSetup[i].routeIdx],
								 &tencap->l4Handles[T13_NUM_L4_ENTRIES + i].paHandle, &tencap->l3Handles[udpSetup[i].lHandleIdx].paHandle,
                                 tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addPort command, entry number %d\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		paTestL4ExpectedStats.classify2.nPackets += 1;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
 		tencap->l4Handles[T13_NUM_L4_ENTRIES + i].state = T13_HANDLE_PENDING_ACK;
 		utilCycleDelay (600);

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    t13CmdRep (tencap);
            if (tencap->l4Handles[T13_NUM_L4_ENTRIES + i].state == T13_HANDLE_ACTIVE)
            {
                udpSetup[i].acked = TRUE;
                break;
            }
	    }

	    if (j == 100)  {
		    System_printf ("%s: (%s:%d): pa_addPort2 command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
		    return (PA_TEST_FAILED);
	    }
	}
	return (PA_TEST_PASSED);
}


#if 0

typedef struct dbgFragInfo_s
{
    int numFrags;
    int numOipFrags;
    int numOipFrags2;
    int inPktSize;
    int lastInPktSize;
} dbgFragInfo_t;

dbgFragInfo_t dbgFragInfo[200];
int dbgNumPkts = 0;

#endif

static void t13UpdatesStats(paSysStats_t *stats, int pktIndex, uint16_t pktLen, uint16_t mtuSize, uint16_t ipOffset, uint16_t l4Offset, pktTest13Info_t *pPktInfo)
{
    /* Asumption: there is no fragmentable IPv6 extension headers */
    int numFrags;
    uint16_t fragSize, lastFragSize;
    uint16_t payloadSize, hdrSize;
    uint16_t inPktSize, lastInPktSize;
    int fInnerIp = pPktInfo->innerIp;
    int fIpv6 = pPktInfo->ipv6;
    int fNatT = pPktInfo->natT;
    int numIpsecHdr = pPktInfo->numIpsecHdr;
    oipFragInfo_t *pOipInfo = &pPktInfo->oipInfo;

    hdrSize = l4Offset - ipOffset;
    payloadSize = (pktLen - ipOffset) - hdrSize;
    lastFragSize = (mtuSize - hdrSize);
    fragSize = lastFragSize & 0xFFF8;

    numFrags = 1;

	stats->classify1.nPackets += (numIpsecHdr);
    stats->classify1.nTableMatch += (numIpsecHdr);

    inPktSize = fragSize + hdrSize;
    if (!fIpv6)
    {
        while (payloadSize >= lastFragSize)
        {
            numFrags += 1;
            payloadSize -= fragSize;
        }

        lastInPktSize = payloadSize + hdrSize;
    }
    else
    {
        fragSize -= 8;
        if ((fragSize + 8) < payloadSize)
        {
            /* note: the condition for non-fragmented packet should not include the fragment header */
            while (payloadSize > fragSize)
            {
                numFrags++;
                payloadSize -= fragSize;
            }

            lastInPktSize = payloadSize + hdrSize + ((numFrags > 1)?8:0);
        }
        else
        {
            lastInPktSize = payloadSize + hdrSize;
        }
    }

    if (numFrags > 1)
    {
	    stats->classify1.nTxIpFrag += numFrags;
	    stats->classify1.nIpFrag   += numFrags;
    }

    #if 0
    dbgFragInfo[dbgNumPkts].numFrags = numFrags;
    dbgFragInfo[dbgNumPkts].inPktSize = inPktSize;
    dbgFragInfo[dbgNumPkts].lastInPktSize = lastInPktSize;
    #endif

    /* MAC update due to fragments */
	stats->classify1.nPackets += (numFrags - 1);
    stats->classify1.nTableMatch += (numFrags - 1);

    /* Outer IP updates due to fragments */
    if (fInnerIp)
    {
	    stats->classify1.nPackets += (numFrags - 1) * (numIpsecHdr + 1);
        stats->classify1.nTableMatch += (numFrags - 1) * (numIpsecHdr + 1);
        if(fIpv6)
		    stats->classify1.nIpv6Packets += (numFrags - 1);
        else
		    stats->classify1.nIpv4Packets += (numFrags - 1);
        if (fNatT)
        {
	        stats->classify2.nPackets += (numFrags - 1);
            stats->classify2.nUdp += (numFrags - 1);
        }
    }

    /* Upadte statistics per outer IP fragmentation */
    if (pOipInfo->mtuSize)
    {
        uint16_t numOipFrags;

        if (pOipInfo->singleIp)
        {
            /* adjust inner packet size: remove ip header size */
            inPktSize -= hdrSize;
            lastInPktSize -= hdrSize;
        }

        /* Fragmentation of the last fragments */
        numOipFrags = 1;
        payloadSize = lastInPktSize + pOipInfo->numExtraBytes;

        lastFragSize = (pOipInfo->mtuSize - pOipInfo->hdrSize);
        fragSize = lastFragSize & 0xFFF8;

        if (!fIpv6)
        {
            while (payloadSize > lastFragSize)
            {
                numOipFrags += 1;
                payloadSize -= fragSize;
            }

        }
        else
        {
            fragSize -= 8;
            if ((fragSize + 8) < payloadSize)
            {
                /* note: the condition for non-fragmented packet should not include the fragment header */
                while (payloadSize > fragSize)
                {
                    numOipFrags++;
                    payloadSize -= fragSize;
                }
            }
        }

        #if 0
        dbgFragInfo[dbgNumPkts].numOipFrags = numOipFrags;
        dbgFragInfo[dbgNumPkts].numOipFrags2 = 0;
        #endif

        if (numOipFrags > 1)
        {
	        stats->classify1.nTxIpFrag += numOipFrags;
	        stats->classify1.nIpFrag   += numOipFrags;

            /* MAC update due to fragments */
	        stats->classify1.nPackets += (numOipFrags - 1);
            stats->classify1.nTableMatch += (numOipFrags - 1);
        }

        if (numFrags > 1)
        {
            numOipFrags = 1;
            payloadSize = inPktSize + pOipInfo->numExtraBytes;

            if (!fIpv6)
            {
                while (payloadSize > lastFragSize)
                {
                    numOipFrags += 1;
                    payloadSize -= fragSize;
                }
            }
            else
            {
                fragSize -= 8;
                if ((fragSize + 8) < payloadSize)
                {
                    /* note: the condition for non-fragmented packet should not include the fragment header */
                    while (payloadSize > fragSize)
                    {
                        numOipFrags++;
                        payloadSize -= fragSize;
                    }
                }
            }

            if (numOipFrags > 1)
            {
	            stats->classify1.nTxIpFrag += numOipFrags * (numFrags - 1);
	            stats->classify1.nIpFrag   += numOipFrags * (numFrags - 1);

                /* MAC update due to fragments */
	            stats->classify1.nPackets += (numOipFrags - 1) * (numFrags - 1);
                stats->classify1.nTableMatch += (numOipFrags - 1) * (numFrags - 1);
            }

            //dbgFragInfo[dbgNumPkts].numOipFrags2 = numOipFrags;

        }

        //dbgNumPkts++;
    }
}

static uint16_t t13MtuSize[T13_NUM_L4_ENTRIES] = {
                                   38,
                                   200,
                                   300,
                                   512,
                                   80,
                                   200,
                                   300,
                                   512,
                                   3000,
                                   3000,
                                   300,
                                   300,
                                   300,
                                   300
                                   };


static uint16_t t13pktPayloadSize[T13_NUM_L4_ENTRIES][10] =
{
    {52,    8,  60, 82,  100, 120,  80, 60,  40,   20},    /* MTU size = 38   */
    {60,  300, 500, 120, 400, 250, 100, 500, 80,   300},   /* MTU size = 200 */
    {300,  60, 500, 600, 400, 100, 100, 700, 350,  100},   /* MTU size = 300 */
    {100, 500, 550, 60,  900, 600, 400, 850, 1500, 1000},  /* MTU size = 512 */
    {32,   40,  60, 80,  100, 120,  80, 60,  40,   20},    /* MTU size = 80   */
    {60,  300, 500, 120, 400, 250, 100, 500, 80,   300},   /* MTU size = 200 */
    {300,  60, 500, 600, 400, 100, 100, 700, 350,  100},   /* MTU size = 300 */
    {100, 500, 550, 60,  900, 600, 400, 850, 1500, 1000},  /* MTU size = 512 */
    {100, 500, 550, 60,  900, 600, 400, 850, 1500, 1000},  /* MTU size = 3000 */
    {100, 500, 550, 60,  900, 600, 400, 850, 1500, 1000},  /* MTU size = 3000 */
    {200,  60, 500, 600, 400, 100, 100, 700, 350,  100},   /* MTU size = 300 */
    {200,  60, 500, 600, 400, 100, 100, 700, 350,  100},   /* MTU size = 300 */
    {200,  60, 500, 600, 400, 100, 100, 700, 350,  100},   /* MTU size = 300 */
    {200,  60, 500, 600, 400, 100, 100, 700, 350,  100}    /* MTU size = 300 */
};

#ifndef __LINUX_USER_SPACE
static uint8_t t13pktBuf[14][2000];
#else
static uint8_t* t13pktBuf[14];
#endif

static int t13SendDataPkts (t13TestEncap_t *tencap, int group, int start, int end)
{
	Qmss_Queue      	q;
	int 				i;
	uint16_t  			cmdStackSize;
	paReturn_t      	paret;
    uint8_t              *buf;
    uint16_t             payloadLen;
    paCmdInfo_t        cmdInfo;
    int                ipOffset, ipOffset2, udpOffset, inIpLen, outIpLen, outHdrLen;
 	volatile int mdebugWait = 1;

    end++;
	/* Attach one free descriptor to each of the packets */
	for (i = start; i < end; i++)  {
		tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);

		if (tencap->hd[i] == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
            return (-1);
		}

		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);

        buf = t13pktBuf[i];
        payloadLen = t13pktPayloadSize[i][group];

  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)buf), (uint32_t)(t13PktInfo[i].pktInfo.pktLen + payloadLen));
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t13PktInfo[i].pktInfo.pktLen + payloadLen);

        memcpy(buf, t13PktInfo[i].pktInfo.pkt, t13PktInfo[i].pktInfo.pktLen);
        testGenPayload(PAU_PAYLOAD_INC8, 0, payloadLen, &buf[t13PktInfo[i].pktInfo.pktLen]);

        /* Update the packet header and command based on the payload size */
        ipOffset  =  t13PktInfo[i].cmdInfo.l3Offset;
        ipOffset2 =  t13PktInfo[i].cmdInfo.l3Offset2;
        udpOffset  =  t13PktInfo[i].l4Offset;
        inIpLen = (udpOffset - ipOffset2) + 8 + payloadLen;
        outHdrLen = ipOffset2 - ipOffset;
        outIpLen = outHdrLen + inIpLen;
        t13PktInfo[i].cmdInfo.endOffset = udpOffset + 8 + payloadLen;

        if (outHdrLen != 0)
        {
            /* It is IP over IP packet */
            if (t13PktInfo[i].ipv6)
            {
                buf[ipOffset + 4] = (outIpLen - 40) >> 8;
                buf[ipOffset + 5] = (outIpLen - 40) & 0xFF;
                buf[ipOffset + 3] = (uint8_t)group;
            }
            else
            {
                buf[ipOffset + 2] = (outIpLen) >> 8;
                buf[ipOffset + 3] = (outIpLen) & 0xFF;
                buf[ipOffset + 5] = (uint8_t)group;
            }
        }

        if (t13PktInfo[i].ipv6)
        {
            buf[ipOffset2 + 4] = (inIpLen - 40) >> 8;
            buf[ipOffset2 + 5] = (inIpLen - 40) & 0xFF;
            buf[ipOffset2 + 3] = (uint8_t)group;
            buf[udpOffset + 4] =  (payloadLen + 8) >> 8;
            buf[udpOffset + 5] =  (payloadLen + 8) & 0xFF;
        }
        else
        {
            buf[ipOffset2 + 2]  = (inIpLen) >> 8;
            buf[ipOffset2 + 3]  = (inIpLen) & 0xFF;
            buf[ipOffset2 + 5]  = (uint8_t)group;
            buf[udpOffset + 4] =  (payloadLen + 8) >> 8;
            buf[udpOffset + 5] =  (payloadLen + 8) & 0xFF;
        }
		cmdStackSize = sizeof(tencap->cmdStack[i]);
        cmdInfo.cmd = pa_CMD_EF_OP;
        cmdInfo.params.efOp = t13PktInfo[i].cmdInfo;

   		paret = Pa_formatTxCmd (    1,
                                	&cmdInfo,
                                	0,
                                	(Ptr)&tencap->cmdStack[i],    /* Command buffer       */
                                	&cmdStackSize);   	          /* Command size         */

        if (paret != pa_OK)  {
        	System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
            return (-1);

        }

  		Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)&tencap->cmdStack[i], cmdStackSize);

        /*
        * Write back the entire cache to make sure that the test packets are updated.
        * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
        *       the portion of packet which is updated.
        *
        */
#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X

        /*
        * Write back the entire cache to make sure that the test packets are updated.
        * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
        *       the portion of packet which is updated.
        *
        */
        CACHE_wbAllL1d(CACHE_WAIT);
        //CACHE_wbAllL2(CACHE_WAIT);
#endif
#endif

		/* Send the data to the modify PDSP */
        //mdebugHaltPdsp(5);
		Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)tencap->hd[i], (uint32_t)t13PktInfo[i].pktInfo.pktLen + payloadLen, TF_SIZE_DESC, Qmss_Location_TAIL);

        //while (mdebugWait);

		testCommonIncStats (t13PktInfo[i].pktInfo.statsMap, &paTestL4ExpectedStats);
        t13UpdatesStats(&paTestL4ExpectedStats, i, t13PktInfo[i].pktInfo.pktLen + payloadLen, t13MtuSize[i], ipOffset2, udpOffset, &t13PktInfo[i]);

        utilCycleDelay (10000);
	}

   /* Wait for descriptors to return. It is assumed that they are returning in order. */
   for (i = 0; i < 100; i++)  {

   	if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= (end - start))
   		break;

   	 utilCycleDelay (1000);
   }

   if (i == 100)  {
   	    System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
        return (-1);
   }

   /* Recycle the descriptors */
   for (i = start; i <  end; i++) {
   	    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
   	    if (tencap->hd[i] == NULL)  {
   		    System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);     \
            return (-1);
   	    }

		Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
        tencap->hd[i] = NULL;
   }

   return (0);

}


/* Search the receive data packet queue for received data packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t13ReceiveDataPkts (t13TestEncap_t *tencap, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;
	int               i;
    int               pktId = 0;
	uint32_t			  flags;
    unsigned int              eflags;
    int               count = 0;
    int               ddrBounceCount=0;

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (5000);
        testCommonRelayQueueBouncePkts (tencap->tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, &ddrBounceCount, NULL);

        /*
         * Look for L2 Relay packets: Examine and push to PASS input queue
         * Set break point to inspect the packet descriptor with the expected EMAC port number
         */
	    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_L2_RELAY]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_L2_RELAY])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received L2 relay packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

		    Qmss_queuePushDescSize (tencap->tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd, TF_SIZE_DESC);

            utilCycleDelay (10000);

	    }

        /* Look for ESP packets: Update the startOffset and endOffset,
         * Push to Ingress3: for nextHdr is IP
         *         Ingress4: Otherwise
         *
         * Note: Fixed ivSize (8) and icvSize (12) are used since it is just a proof-of-concept test.
         *       Those two variables may be added to the packet packet information in enhanced test cases
         */
	    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_ESP_FIX]) > 0)  {
		    uint8_t  startOffset;
            uint16_t endOffset;

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_ESP_FIX])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received ESP packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received ESP packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

            startOffset = PASAHO_LINFO_READ_START_OFFSET(pinfo) + 8;
            PASAHO_LINFO_SET_START_OFFSET(pinfo, startOffset);

            endOffset = PASAHO_LINFO_READ_END_OFFSET(pinfo) - 12;
            PASAHO_LINFO_SET_END_OFFSET(pinfo, endOffset);

            PASAHO_LINFO_SET_NXT_HDR_TYPE(pinfo, PASAHO_HDR_ESP_DECODED);

            while (endOffset > hd->buffLen)
            {
                endOffset -= hd->buffLen;
                hd = (Cppi_HostDesc *)hd->nextBDPtr;
            }

		    Qmss_queuePushDescSize (tencap->tf->QPaTx[TF_PA_Q_FIREWALL2], (Ptr)hd, TF_SIZE_DESC);

            utilCycleDelay (10000);

	    }

	    /* Look for packets in the rx packet queue, verify that the UDP port found matches the next value in
	    * the fifo, then verify the receive packet information */
	    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_MATCH]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((*swInfo & T13_CMD_SWINFO0_TYPE_MASK) != T13_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

			pktId = *swInfo & T13_CMD_SWINFO0_ID_MASK;

		    if (pktId >= T13_NUM_LOCAL_L4_HANDLES)  {
			    System_printf ("%s (%s:%d): Found a packet with unexpected packet id %d (max = %d)\n",
				    tfName, __FILE__, __LINE__, pktId, T13_NUM_LOCAL_L4_HANDLES - 1);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }

			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

		    flags = PASAHO_LINFO_READ_HDR_BITMASK(pinfo);

		    if ((flags & (PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_TCP)) == 0)  {
			    System_printf ("%s (%s:%d): Found a packet without any L4 info\n", tfName, __FILE__, __LINE__);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }


			if (testCommonComparePktInfo (tfName, t13PktInfo[pktId].pktInfo.info, pinfo))  {
				testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

            eflags = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc *)hd) & 0xf;


            if (eflags != 0)  {
                System_printf ("%s (%s:%d): Packet with index %d returned with error flags = 0x%02x\n", tfName, __FILE__, __LINE__, pktId, eflags);
            }

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tencap->tf, hd);

            count++;

		}

        if(count >= expCount)
            break;

	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): Error - unable to recover all packets\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}

	/* Since the packets went to the modify PDSP and then back to the QM, a descriptor
	 * and linked buffer was required while the packet was in the QM. This will
	 * be recycled to the default recycle queue */
	while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
		//Cppi_HostDesc *hdNext;
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
			return (-1);
		}

        testCommonRecycleLBDesc (tencap->tf, hd);
	}

    if(ddrBounceCount < count)
        System_printf("t13ReceiveDataPkts: receives %d queue bounce packets and %d final packets\n", ddrBounceCount, count);

	return (0);

}

static uint16_t t13pktPayloadSize2[5][10] =
{
    {60,  300, 500, 120, 400, 250, 100, 500, 80,   300},
    {300,  60, 500, 600, 400, 100, 100, 700, 350,  100},
    {100, 500, 550, 60,  900, 600, 400, 850, 1450, 1000},
    {32,   40,  60, 80,  100, 120,  80, 60,  40,   20},
    {200, 400, 550, 60,  800, 700, 350, 850, 1250, 900}

};

static int t13SendFwdPkts (t13TestEncap_t *tencap, int group, int start, int end)
{
	Qmss_Queue      	q;
	int 				i;
    uint8_t              *buf;
    uint16_t             payloadLen;
    int                ipOffset, ipOffset2, udpOffset;

    end++;
	/* Attach one free descriptor to each of the packets */
	for (i = start; i < end; i++)  {
		tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);

		if (tencap->hd[i] == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
            return (-1);
		}

		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);

        buf = t13pktBuf[i];
        payloadLen = t13pktPayloadSize2[i][group];

  	    /* Make sure there is no control info.  */
  	    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)tencap->hd[i], 0);

  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)buf), (uint32_t)(t13PktInfo2[i].pktInfo.pktLen + payloadLen));
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t13PktInfo2[i].pktInfo.pktLen + payloadLen);

        memcpy(buf, t13PktInfo2[i].pktInfo.pkt, t13PktInfo2[i].pktInfo.pktLen);
        testGenPayload(PAU_PAYLOAD_INC8, 0, payloadLen, &buf[t13PktInfo2[i].pktInfo.pktLen]);

        /* Update the packet header and command based on the payload size */
        /* IPv4 only at this moment */
        udpOffset = PASAHO_LINFO_READ_L4_OFFSET(t13PktInfo2[i].pktInfo.info);
        ipOffset =  PASAHO_LINFO_READ_L3_OFFSET(t13PktInfo2[i].pktInfo.info);
        if (t13PktInfo2[i].pktInfo.pktLen < (udpOffset + 8))
        {
            /* Adjust udpOffset if there is only one IP at the input packet */
            udpOffset -= 20;
        }
        ipOffset2 = udpOffset - 20;

        if (ipOffset != ipOffset2)
        {
            /* It is IP over IP packet */
            buf[ipOffset + 2] = (payloadLen + 48) >> 8;
            buf[ipOffset + 3] = (payloadLen + 48) & 0xFF;

            buf[ipOffset + 5] = (uint8_t)group;

            /* Update IP checksum */
            utilUpdateIpChksums(&buf[ipOffset]);
        }

        buf[ipOffset2 + 2] = (payloadLen + 28) >> 8;
        buf[ipOffset2 + 3] = (payloadLen + 28) & 0xFF;
        buf[ipOffset2 + 5] = (uint8_t)group;
        buf[udpOffset + 4] =  (payloadLen + 8) >> 8;
        buf[udpOffset + 5] =  (payloadLen + 8) & 0xFF;

        /* Update IP checksum */
        utilUpdateIpChksums(&buf[ipOffset2]);

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X

        /*
        * Write back the entire cache to make sure that the test packets are updated.
        * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
        *       the portion of packet which is updated.
        *
        */
        CACHE_wbAllL1d(CACHE_WAIT);
        //CACHE_wbAllL2(CACHE_WAIT);
#endif
#endif

		/* Send the data to the modify PDSP */
        //mdebugHaltPdsp(5);
		Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_INPUT], (Ptr)tencap->hd[i], (uint32_t)t13PktInfo2[i].pktInfo.pktLen + payloadLen, TF_SIZE_DESC, Qmss_Location_TAIL);

		testCommonIncStats (t13PktInfo2[i].pktInfo.statsMap, &paTestL4ExpectedStats);
		testCommonIncStats (t13PktInfo2[i].statsMap2, &paTestL4ExpectedStats);
        t13UpdateUsrStats(&paTestExpectedUsrStats, t13UsrStatsTbl[i].cntIndex, 1);


        utilCycleDelay (10000);
	}

   /* Wait for descriptors to return. It is assumed that they are returning in order. */
   for (i = 0; i < 100; i++)  {

   	if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= (end - start))
   		break;

   	 utilCycleDelay (1000);
   }

   if (i == 100)  {
   	    System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
        return (-1);

   }

   /* Recycle the descriptors */
   for (i = start; i <  end; i++) {
   	    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
   	    if (tencap->hd[i] == NULL)  {
   		    System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
            return (-1);
   	    }

		Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
        tencap->hd[i] = NULL;
   }

   return (0);

}

/* Search the receive data packet queue for received data packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t13ReceiveFwdPkts (t13TestEncap_t *tencap, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;
	int               i;
    int               pktId = 0;
	uint32_t			  flags;
    unsigned int              eflags;
    int               count = 0;
    int               ddrBounceCount=0;

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (5000);
        testCommonRelayQueueBouncePkts (tencap->tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, &ddrBounceCount, NULL);


	    /* Look for packets in the rx packet queue, verify that the UDP port found matches the next value in
	     * the fifo, then verify the receive packet information */
	    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_MATCH]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((*swInfo & T13_CMD_SWINFO0_TYPE_MASK) != T13_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

			pktId = *swInfo & T13_CMD_SWINFO0_ID_MASK;

		    if (pktId >= T13_NUM_LOCAL_L4_HANDLES)  {
			    System_printf ("%s (%s:%d): Found a packet with unexpected packet id %d (max = %d)\n",
				    tfName, __FILE__, __LINE__, pktId, T13_NUM_LOCAL_L4_HANDLES - 1);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }

			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

		    flags = PASAHO_LINFO_READ_HDR_BITMASK(pinfo);

		    if ((flags & (PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_TCP)) == 0)  {
			    System_printf ("%s (%s:%d): Found a packet without any L4 info\n", tfName, __FILE__, __LINE__);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }


			if (testCommonComparePktInfo (tfName, t13PktInfo2[pktId - T13_FIRST_IP_FWD_ENTRY].pktInfo.info, pinfo))  {
				testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

            eflags = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc *)hd) & 0xf;


            if (eflags != 0)  {
                System_printf ("%s (%s:%d): Packet with index %d returned with error flags = 0x%02x\n", tfName, __FILE__, __LINE__, pktId, eflags);
            }

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tencap->tf, hd);

            count++;

		}

        if(count >= expCount)
            break;

	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): Error - unable to recover all packets\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}

	/* Since the packets went to the modify PDSP and then back to the QM, a descriptor
	 * and linked buffer was required while the packet was in the QM. This will
	 * be recycled to the default recycle queue */
	while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
		//Cppi_HostDesc *hdNext;
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
			return (-1);
		}

        testCommonRecycleLBDesc (tencap->tf, hd);
	}

    if(ddrBounceCount != count)
        System_printf("t13ReceiveFwdPkts: receives %d queue bounce packets and %d final packets\n", ddrBounceCount, count);

	return (0);
}

static int t13SendExpPkts (t13TestEncap_t *tencap)
{
	Qmss_Queue      	q;
	int 				i, j;
	uint16_t  			cmdStackSize;
	paReturn_t      	paret;
    paCmdInfo_t         cmdInfo;
 	volatile int mdebugWait = 1;

	/* Attach one free descriptor to each of the packets */
	for (i = 0; i < sizeof(t13expPktInfo)/sizeof(pktTest13Info_t); i++)  {
		tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);

		if (tencap->hd[i] == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
            System_flush();
			return (-1);
		}

		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);


  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)t13expPktInfo[i].pktInfo.pkt), (uint32_t)(t13expPktInfo[i].pktInfo.pktLen));
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t13expPktInfo[i].pktInfo.pktLen);

		cmdStackSize = sizeof(tencap->cmdStack[i]);
        cmdInfo.cmd = pa_CMD_EF_OP;
        cmdInfo.params.efOp = t13expPktInfo[i].cmdInfo;

   		paret = Pa_formatTxCmd (    1,
                                	&cmdInfo,
                                	0,
                                	(Ptr)&tencap->cmdStack[i],    /* Command buffer       */
                                	&cmdStackSize);   	          /* Command size         */

        if (paret != pa_OK)  {
        	System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
            System_flush();
            return(-1);
        }

  		Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)&tencap->cmdStack[i], cmdStackSize);

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X

        /*
        * Write back the entire cache to make sure that the test packets are updated.
        * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
        *       the portion of packet which is updated.
        *
        */
        CACHE_wbAllL1d(CACHE_WAIT);
        //CACHE_wbAllL2(CACHE_WAIT);
#endif
#endif

		/* Send the data to the modify PDSP */
        //mdebugHaltPdsp(5);
		Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)tencap->hd[i], (uint32_t)t13expPktInfo[i].pktInfo.pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);

        //while (mdebugWait);

        utilCycleDelay (2000);


        /* Wait for descriptors to return. It is assumed that they are returning in order. */
        for (j = 0; j < 100; j++)  {

   	        if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= 1)
   		        break;

   	        utilCycleDelay (1000);
        }

        if (j == 100)  {
   	            System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
                System_flush();
                return (-1);
        }

        /* Recycle the descriptors */
   	    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
   	    if (tencap->hd[i] == NULL)  {
   		    System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
            System_flush();
            return (-1);
   	    }

		Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
        tencap->hd[i] = NULL;

	}

    return (i);

}

/* Search the receive data packet queue for received data packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t13ReceiveExpPkts (t13TestEncap_t *tencap, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	int               i;
    int               count = 0;
    int               ddrBounceCount=0;

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (5000);
        testCommonRelayQueueBouncePkts (tencap->tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, &ddrBounceCount, NULL);

	    /* Look for packets in the rx packet queue, verify that the UDP port found matches the next value in
	     * the fifo, then verify the receive packet information */
	    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_EF_EXP]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_EF_EXP])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((swInfo[0] & T13_CMD_SWINFO0_TYPE_MASK) != T13_SWINFO0_EXP_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, swInfo[0]);
			 	testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

			if (t13expPktInfo[count].pktInfo.idx != swInfo[0])  {
				testCommonRecycleLBDesc (tencap->tf, hd);
                System_printf ("%s (%s:%d): Packet with index %d returned with unexpected swInfo0 = 0x%08x\n", tfName, __FILE__, __LINE__, count, swInfo[0]);

				return (-1);
			}

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tencap->tf, hd);

            count++;

		}

        if(count >= expCount)
            break;

	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): Error - unable to recover all egress flow exception packets\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}

    if(ddrBounceCount != count)
        System_printf("t13ReceiveExpPkts: receives %d queue bounce packets and %d final packets\n", ddrBounceCount, count);

	return (0);
}


static int t13SendQoSPkts (t13TestEncap_t *tencap)
{
	Qmss_Queue      	q;
	int 				i, j;
	uint16_t  			cmdStackSize;
	paReturn_t      	paret;
    paCmdInfo_t         cmdInfo;
 	volatile int mdebugWait = 1;

	/* Attach one free descriptor to each of the packets */
	for (i = 0; i < sizeof(t13qosPktInfo)/sizeof(pktTest13Info_t); i++)  {
		tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);

		if (tencap->hd[i] == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
			t13Cleanup (tencap, PA_TEST_FAILED);
		}

		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);


  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)t13qosPktInfo[i].pktInfo.pkt), (uint32_t)(t13qosPktInfo[i].pktInfo.pktLen));
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t13qosPktInfo[i].pktInfo.pktLen);

		cmdStackSize = sizeof(tencap->cmdStack[i]);
        cmdInfo.cmd = pa_CMD_EF_OP;
        cmdInfo.params.efOp = t13qosPktInfo[i].cmdInfo;

   		paret = Pa_formatTxCmd (    1,
                                	&cmdInfo,
                                	0,
                                	(Ptr)&tencap->cmdStack[i],    /* Command buffer       */
                                	&cmdStackSize);   	          /* Command size         */

        if (paret != pa_OK)  {
        	System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
            return(-1);
        }

  		Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)&tencap->cmdStack[i], cmdStackSize);

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X

        /*
        * Write back the entire cache to make sure that the test packets are updated.
        * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
        *       the portion of packet which is updated.
        *
        */
        CACHE_wbAllL1d(CACHE_WAIT);
        //CACHE_wbAllL2(CACHE_WAIT);
#endif
#endif

		/* Send the data to the modify PDSP */
        //mdebugHaltPdsp(5);
		Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)tencap->hd[i], (uint32_t)t13qosPktInfo[i].pktInfo.pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);

        //while (mdebugWait);

        utilCycleDelay (2000);


        /* Wait for descriptors to return. It is assumed that they are returning in order. */
        for (j = 0; j < 100; j++)  {

   	        if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= 1)
   		        break;

   	        utilCycleDelay (1000);
        }

        if (j == 100)  {
   	            System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
                return (-1);
        }

        /* Recycle the descriptors */
   	    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
   	    if (tencap->hd[i] == NULL)  {
   		    System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
            return (-1);
   	    }

		Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
        tencap->hd[i] = NULL;

	}

    return (i);

}

static Cppi_HostDesc *t13GetQoSPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;

    for (index = Q_QoS_BASE; index < (T13_NUM_QoS_QUEUES +  Q_QoS_BASE); index++)
    {
	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t13GetQoSPk: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }

            return (hd);
	    }
    }

    return (NULL);

}

/* Search the QoS queues for Egress QoS packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t13ReceiveQoSPkts (t13TestEncap_t *tencap, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	int               i;
    int               count = 0;

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (5000);
	    /* Look for packets in the rx QoS queues */
	    while ((hd = t13GetQoSPkt (tencap->tf)) != NULL)  {

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((swInfo[0] & T13_CMD_SWINFO0_TYPE_MASK) != T13_SWINFO0_QOS_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, swInfo[0]);
			 	testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tencap->tf, hd);

            count++;

		}

        if(count >= expCount)
            break;

	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): Error - unable to recover all QoS packets\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}
    
	return (0);
}


#ifdef __LINUX_USER_SPACE
void* paTestEflow (void *args)
{
 	void  *a0 = (void *)((paTestArgs_t *)args)->tf;
 	void  *a1 = (void *)((paTestArgs_t *)args)->pat;
#else
void paTestEflow (UArg a0, UArg a1)
{
#endif
 	int				i,k, n;
 	paTestStatus_t  newStatus;
 	volatile int mdebugWait = 1;
    paCmdInfo_t     cmdInfo[5];

    /* Specify the user statistics associated with the MAC input */
    cmdInfo[0].cmd = cmdInfo[1].cmd = cmdInfo[2].cmd = cmdInfo[3].cmd = cmdInfo[4].cmd = pa_CMD_USR_STATS;

 	/* Initialize the test state */
 	memset (&t13Encap, 0, sizeof(t13Encap));
 	t13Encap.tf  = (tFramework_t *)a0;
 	t13Encap.pat = (paTest_t *)a1;
 	for (i = 0; i < T13_NUM_LOCAL_L4_HANDLES; i++)
 		t13Encap.l4Handles[i].state = T13_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T13_NUM_LOCAL_FC_HANDLES; i++)
 		t13Encap.fcHandles[i].state = T13_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T13_NUM_LOCAL_L3_HANDLES; i++)
 		t13Encap.l3Handles[i].state = T13_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T13_NUM_LOCAL_L2_HANDLES; i++)
 		t13Encap.l2Handles[i].state = T13_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T13_NUM_LOCAL_L4_HANDLES; i++)
 		t13Encap.hd[i] = NULL;

#ifdef __LINUX_USER_SPACE

    i = setupPktTestInfo2(t13expPktInfo, (sizeof(t13expPktInfo) / sizeof(pktTest13Info_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo2(t13expPktInfo): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t13Encap.pat->testStatus = PA_TEST_FAILED;
  	    return (void *)0;
    }

    i = setupPktTestInfo2(t13qosPktInfo, (sizeof(t13qosPktInfo) / sizeof(pktTest13Info_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo2(t13qosPktInfo): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t13Encap.pat->testStatus = PA_TEST_FAILED;
  	    return (void *)0;
    }

    for (i = 0; i < 14; i++)  {
        /* Allocate memory for the Packet buffers */
        t13pktBuf[i] = (uint8_t *)fw_memAlloc(2000, CACHE_LINESZ);
        if(t13pktBuf[i] == NULL) {
  	        printf ("%s: memAlloc failed for pkt %d\n", tfName, i);
 		    t13Encap.pat->testStatus = PA_TEST_FAILED;
  	        return (void *)0;
        }
    }

#endif

    /* Runtime initial values */
    matchRoute[T13_IP_NEXT_ROUTE_HOST].queue = (uint16_t) t13Encap.tf->QGen[Q_MATCH];
    matchRoute[T13_IP_NEXT_ROUTE_HOST].flowId = t13Encap.tf->tfFlowNum[0];
    matchRoute[T13_IP_NEXT_ROUTE_HOST_ESP].queue = (uint16_t) t13Encap.tf->QGen[Q_ESP_FIX];
    matchRoute[T13_IP_NEXT_ROUTE_HOST_ESP].flowId = t13Encap.tf->tfFlowNum[0];
    matchRoute[T13_IP_NEXT_ROUTE_IPSEC2].flowId = t13Encap.tf->tfFlowNum[0];
    /* replace destination queue so that we can examine the eflow packet with expected EMAC
       port setting */
    t13EfRec4Setup[0].rec.queueId = (uint16_t) t13Encap.tf->QGen[Q_L2_RELAY];
    nfailRoute.queue    = (uint16_t) t13Encap.tf->QGen[Q_NFAIL];
    cmdReply.queue      = (uint16_t) t13Encap.tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId     = t13Encap.tf->tfFlowNum[0];

    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL4ExpectedStats, 0, sizeof(paTestL4ExpectedStats));
    memset (&paTestExpectedUsrStats, 0, sizeof(paTestExpectedUsrStats));

    /* Global Configuration */
	newStatus = t13GlobalConfiguration (&t13Encap);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    /* Allocate User-defined statistics */
    if(testCommonAllocUsrStats(t13Encap.tf, tfName, T13_NUM_USR_STATS, t13UsrStatsTbl, t13UsrStatsGroup1))
    {
        newStatus = PA_TEST_FAILED;
        t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    /* Initialize matchRoute2 user-stats command for Egress Flow forwarding operation */
    cmdInfo[0].params.usrStats.index = t13UsrStatsTbl[T13_USR_STATS_ID_EFLOW_0].cntIndex;
    cmdInfo[1].params.usrStats.index = t13UsrStatsTbl[T13_USR_STATS_ID_EFLOW_1].cntIndex;
    cmdInfo[2].params.usrStats.index = t13UsrStatsTbl[T13_USR_STATS_ID_EFLOW_2].cntIndex;
    cmdInfo[3].params.usrStats.index = t13UsrStatsTbl[T13_USR_STATS_ID_EFLOW_3].cntIndex;
    cmdInfo[4].params.usrStats.index = t13UsrStatsTbl[T13_USR_STATS_ID_EFLOW_4].cntIndex;
    matchRoute2[0].pCmd = &cmdInfo[0];
    matchRoute2[1].pCmd = &cmdInfo[1];
    matchRoute2[2].pCmd = &cmdInfo[2];
    matchRoute2[3].pCmd = &cmdInfo[3];
    matchRoute2[4].pCmd = &cmdInfo[4];

    /* Usr Stats configuration */
    newStatus = testCommonUsrStatsSetup (t13Encap.tf, t13Encap.pat, tfName, sizeof(t13UsrStatsSetup)/sizeof(pauUsrStatsSetup_t), t13UsrStatsSetup,
                                         T13_CMD_SWINFO0_USR_STATS_CFG_ID, t13Encap.tf->QLinkedBuf3, t13Encap.tf->QGen[Q_CMD_RECYCLE],
                                         t13Encap.tf->QGen[Q_CMD_REPLY], FALSE);
	if (newStatus == PA_TEST_FAILED)
    {
		t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    /* Exception Route Configurations */
	newStatus = t13ExceptionRoutes (&t13Encap);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    /* Egress Flow Exception Route Configurations */
	newStatus = t13EflowExceptionRoutes (&t13Encap);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    /* NAT-T Configuration */
	newStatus = t13NatTConfiguration (&t13Encap, 1);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    /* Egress Flow Configuration */
	newStatus = t13EflowConfiguration (&t13Encap);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

	/* Burst in the L2 configuraton */
	newStatus = t13OpenL2 (&t13Encap, t13EthSetup, T13_NUM_L2_ENTRIES);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

	newStatus = t13OpenL2_2 (&t13Encap, t13EthSetup2, T13_NUM_FORWARD_L2_ENTRIES);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

	/* Burst in the L3 configuration */
	newStatus = t13OpenL3 (&t13Encap, t13IpSetup, T13_NUM_L3_ENTRIES);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

	newStatus = t13OpenL3_2 (&t13Encap, t13IpSetup2, T13_NUM_FORWARD_L3_ENTRIES);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

	/* Burst in the L4 configuration */
	newStatus = t13OpenL4 (&t13Encap, t13UdpSetup, T13_NUM_L4_ENTRIES);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

	newStatus = t13OpenL4_2 (&t13Encap, t13UdpSetup2, T13_NUM_FORWARD_L4_ENTRIES);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

	/* Burst in the FC configuration */
	newStatus = t13OpenFc (&t13Encap, t13FcSetup, T13_NUM_LOCAL_FC_HANDLES);
	if (newStatus == PA_TEST_FAILED)
    {
	    t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

	/* Send the packets. Use the engress flow to generate the IP header, TCP/UDP checksums.
	 * and IP fragments if necessary */
	for (k = 0; k < T13_NUM_PACKET_GROUPS; k+=1)  {

        /*
         * There are multiple streams of variable-size packets
         * Send multiple test groups at a time to verify the followings:
         *   - PASS can maintain multiple IP reassembly flows
         */
        /* IPv4 fragmentation */
        for (i = 0; i < 1; i++)
        {
            if (t13SendDataPkts (&t13Encap, k + i, 0, 3))
            {
		        t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
                goto paTestEflow_end;
#endif
            }
        }

        if (t13ReceiveDataPkts (&t13Encap, 4 * i))
        {
		    System_printf ("%s (%s:%d):  Receive IPv4 packets group %d fail\n", tfName, __FILE__, __LINE__, k);
		    t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
            goto paTestEflow_end;
#endif

        }

        /* IPv6 fragmentation */
        for (i = 0; i < 1; i++)
        {
            if(t13SendDataPkts (&t13Encap, k + i, 4, 7))
            {
		        t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
                goto paTestEflow_end;
#endif
            }
        }

        if (t13ReceiveDataPkts (&t13Encap, 4 * i))
        {
		    System_printf ("%s (%s:%d):  Receive IPv6 packets group %d fail\n", tfName, __FILE__, __LINE__, k);
		    t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
            goto paTestEflow_end;
#endif
        }

        /* IPsec and NAT-T (8, 13)*/
        for (i = 0; i < 1; i++)
        {
            if(t13SendDataPkts (&t13Encap, k + i, 8, 13))
            {
		        t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
                goto paTestEflow_end;
#endif
            }
        }

        if (t13ReceiveDataPkts (&t13Encap, 6 * i))
        {
		    System_printf ("%s (%s:%d):  Receive IPSEC and NAT-T packets group %d fail\n", tfName, __FILE__, __LINE__, k);
		    t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
            goto paTestEflow_end;
#endif
        }

	}

	/* Test the IP forwarding packets. */
	for (k = 0; k < T13_NUM_PACKET_GROUPS; k+=1)  {

        /* Group 1*/
        for (i = 0; i < 1; i++)
        {
            if(t13SendFwdPkts (&t13Encap, k + i, 0, 4))
            {
		        t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
                goto paTestEflow_end;
#endif
            }
        }

        if (t13ReceiveFwdPkts (&t13Encap, 5 * i))
        {
		    System_printf ("%s (%s:%d):  Receive IP forwarding packets fail\n", tfName, __FILE__, __LINE__);
		    t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
            goto paTestEflow_end;
#endif
        }
	}

    /* Test the Egress Flow exception packet processing */
    n = t13SendExpPkts(&t13Encap);

    if (n < 0)
    {
	    t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    if (t13ReceiveExpPkts (&t13Encap, n))
    {
	    System_printf ("%s (%s:%d):  Receive Exgress Flow exception packets fail\n", tfName, __FILE__, __LINE__);
	    t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    /* Test the QoS packet processing */
    n = t13SendQoSPkts(&t13Encap);
    if (n < 0)
    {
	    t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    if (t13ReceiveQoSPkts (&t13Encap, n))
    {
	    System_printf ("%s (%s:%d):  Receive QoS packets fail\n", tfName, __FILE__, __LINE__);
	    t13Cleanup (&t13Encap, PA_TEST_FAILED);
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    Pa_queryRaStats (t13Encap.tf->passHandle, TRUE, &paTestRaStats);
    testCommonDispRaStats (tfName, &paTestRaStats);

	/* Verify and clear the Usr stats */
	newStatus =  testCommonCheckUsrStatsList (t13Encap.tf, t13Encap.pat, tfName, &paTestExpectedUsrStats, T13_NUM_64B_USR_STATS, T13_NUM_USR_STATS, t13UsrStatsTbl,
                                              t13Encap.tf->QLinkedBuf1, t13Encap.tf->QGen[Q_CMD_RECYCLE], t13Encap.tf->QGen[Q_CMD_REPLY], TRUE);
    /* Note: Expected tx padding count can not be verified with Hardware RA module */
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckUsrStats Failed\n", tfName, __FILE__, __LINE__);
		t13Cleanup (&t13Encap, newStatus);

#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif

    }

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t13Encap.tf, t13Encap.pat, tfName, &paTestL4ExpectedStats, t13Encap.tf->QLinkedBuf1,
	                                   t13Encap.tf->QGen[Q_CMD_RECYCLE], t13Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
	    System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t13Cleanup (&t13Encap, newStatus);
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    /* NAT-T Configuration */
	newStatus = t13NatTConfiguration (&t13Encap, 0);
	if (newStatus == PA_TEST_FAILED)
    {
		t13Cleanup (&t13Encap, newStatus);  /* No return */
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif
    }

    t13DispFcStats(&t13Encap);

    /* Clear all Usr Stats Link */
    newStatus = testCommonUsrStatsConfigReset (t13Encap.tf, t13Encap.pat, tfName, sizeof(t13UsrStatsSetup)/sizeof(pauUsrStatsSetup_t), t13UsrStatsSetup,
                                               T13_CMD_SWINFO0_USR_STATS_CFG_ID, t13Encap.tf->QLinkedBuf3, t13Encap.tf->QGen[Q_CMD_RECYCLE],
                                               t13Encap.tf->QGen[Q_CMD_REPLY]);

	if (newStatus == PA_TEST_FAILED)
    {
	    System_printf ("%s (%s:%d): testCommonUsrStatsConfigReset Failed\n", tfName, __FILE__, __LINE__);
	    t13Cleanup (&t13Encap, newStatus);
#ifdef __LINUX_USER_SPACE
        goto paTestEflow_end;
#endif

    }

    /* Free User-defined statistics */
    if(testCommonFreeUsrStats(t13Encap.tf, tfName, T13_NUM_USR_STATS, t13UsrStatsTbl))
    {
        newStatus = PA_TEST_FAILED;
    }

	/* No return from cleanup */
	t13Cleanup (&t13Encap, PA_TEST_PASSED);

#ifdef __LINUX_USER_SPACE
paTestEflow_end:
    return (void *)0;
#endif

}



