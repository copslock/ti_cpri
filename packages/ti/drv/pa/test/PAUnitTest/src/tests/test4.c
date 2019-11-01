/*
 *
 * Copyright (C) 2010-2014 Texas Instruments Incorporated - http://www.ti.com/
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

/* Add/Delete IP and PDSP routing test
 * This test test the LLD Pa_addIp, Pa_delHandle and Pa_configCrcEngine functions, as well as the
 * PDSP firmware for routing IP packets. This test has the following sub-tests
 *   - Test the LLD for the ability to determine if a new entry invalidates a previous entry
 *   - Test the LLD to enter increasingly strict match requirements (dst, src, etype, spi, flow index, tos, gre protocol, sctp port)
 *   - Test the LLD when entering more entries then configured for
 *   - Test the LLD and firmware for various PASS global configuration
 *   - Test the firmware for the ability to take a burst of Pa_AddIp commands
 *   - Test the firmware for the routing of matches and next step fail routes
 *   - Test the firmware for the ability to detect IP header errors
 *   - Test the firmware for the ability to handle nested IP headers
 *   - Test the firmware for the ability to break out of too many nested IP headers
 *   - Test the firmware for the ability to take a burst of data packets
 *   - Test the firmware for the ability to verify the SCTP CRC32-C
 * 	 - Test the firmware for correct header offset calculation
 *   - Test the firmware for PPPoE and IP header error processing
 *   - Test the LLD/firmware for the ability to delete entries
 *   - Test the LLD/firmware for NAT-T configuation and packet processing
 *   - Test the LLD/firmware for Priority-based routing
 *   - Test the LLD/firmware for Interface-based routing
 *   - Test the LLD/firmware for Enhanced QoS routing in Ingress direction
 *   - Test the firmware for Enhanced QoS Egress packet parsing and routing
 */

 static char *tfName = "paTestL3Routing";

 #define T4_TEST_L3OFFSET_USE_INNER_IP
 #define T4_NUM_PACKET_ITERATIONS	1  		/* Number of times the packet stream is passed through */

 #define T4_EXPTPKT_FIFO_SIZE		20		/* Must hold the number of packets in transit in PA at one time */
 #define T4_NUM_QoS_QUEUES          32
 #define T4_NUM_EQoS_QUEUES         16
 #define T4_NUM_IF_QUEUES            4

 /* General purpose queue usage */
 #define Q_CMD_RECYCLE		  0		/* Command descriptors/buffers recycled here after sent to PA */
 #define Q_CMD_REPLY  		  1		/* Replies from PA routed here */
 #define Q_MATCH		  	  2		/* Packets from PA which match a lookup criteria */
 #define Q_NFAIL		      3		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
 #define Q_PARSE_ERR		  4		/* Packets which resulted in a parse error */
 #define Q_PPPoE_FAIL         5     /* PPPoE Parsing Error */
 #define Q_PPPoE_CTRL         6     /* PPPoE Control Packets */
 #define Q_IP_FAIL            7     /* IP Header Error */
 #define Q_IP_FRAG            8     /* IP Fragments */
 #define Q_NAT_T              Q_PPPoE_FAIL     /* NAT-T exception packets */
 #define Q_DPKT_RECYCLE       9     /* Data Recycle queue */
 #define Q_L2_CAPTURE         10    /* L2 Capture queue */
 #define Q_QoS_BASE           20    /* First QoS queue */
 #define Q_IF_BASE            60


/* Exception Route  packet index */
#define  T4_PPPoE_FAIL_PKT_INDEX        20
#define  T4_PPPoE_CTRL_PKT_INDEX        21
#define  T4_IP_FAIL_PKT_INDEX           22
#define  T4_IP_FRAG_PKT_INDEX           23
#define  T4_NAT_T_KEEPALIVE_PKT_INDEX   24
#define  T4_NAT_T_CTRL_PKT_INDEX        25
#define  T4_NAT_T_DATA_PKT_INDEX        26
#define  T4_NAT_T_FAIL_PKT_INDEX        27
#define  T4_DROP_PKT_INDEX              0xffff
#define  T4_IF_PKT_INDEX                50
#define  T4_QoS_PKT_INDEX               100
#define  T4_EQoS_PKT_INDEX              110


/* The number of PA L2 and L3 handles maintained by this test */
#define T4_NUM_LOCAL_L2_HANDLES   			3
#define T4_FIRST_MAC2_HANDLE                2
#define T4_NUM_LOCAL_L3_OUTER_IP_HANDLES	64
#define T4_NUM_LOCAL_L3_INNER1_IP_HANDLES	62
#define T4_NUM_LOCAL_L3_OUTER_AND_INNER1_IP_HANDLE  (T4_NUM_LOCAL_L3_OUTER_IP_HANDLES + T4_NUM_LOCAL_L3_INNER1_IP_HANDLES)
#define T4_NUM_LOCAL_L3_INNER2_IP_HANDLES	 2		/* Tripple nested IP Headers */
#define T4_NUM_LOCAL_L3_HANDLES				(T4_NUM_LOCAL_L3_OUTER_IP_HANDLES+T4_NUM_LOCAL_L3_INNER1_IP_HANDLES+T4_NUM_LOCAL_L3_INNER2_IP_HANDLES+2)
#define T4_NUM_LOCAL_VL_HANDLES	  			4
#define T4_NUM_LOCAL_L4_HANDLES	  			8
#define T4_LOCAL_L3_EQoS_NFAIL_INDEX        21

typedef struct t4IpPaSetup_s  {
	int        seqId;		/* Sequential enumeration to identify */
	int		   handleIdx;	/* Local handle index. Specifies which local handle corresponds to this entry */
    int        lutInst;     /* Specify which LUT1 (0-2) should be used */
	int		   lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	int		   routeIdx;	/* Which match route index to use, 0 or 1 */
	paIpInfo_t ipInfo;		/* PA IP configuration structure */
	paReturn_t ret;			/* Expected return code from pa_addIp command */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */
	Bool       vlink;       /* Set to TRUE when the tunnel is using virtual link */
} t4IpPaSetup_t;

#ifndef NSS_GEN2
/* replace test cases for Gen1 only */
typedef enum {
  T4_CMD_REPLACE_WITH_INDEX = 0,
  T4_CMD_REPLACE_WITH_NO_INDEX,
  T4_CMD_REPLACE_WITH_ERR_INDEX
} t4ReplaceIpTest_e;

/* This is added to test the replace command added in pa_addIp2 API */
typedef struct t4RepIpPaSetup_s {
  Int                ipSetupTblIndex; /* IP entry test index */
	paReturn_t         ret;			/* Expected return code from pa_addIp command */
  t4ReplaceIpTest_e  repType; /* Replace test cases */
}t4RepIpPaSetup_t;
#endif

#include "test4pkts.h"

/* The total number of buffers with linked descriptors */
#define TOTAL_BUFS   (TF_LINKED_BUF_Q1_NBUFS + TF_LINKED_BUF_Q2_NBUFS + TF_LINKED_BUF_Q3_NBUFS)

 /* Commands to the PA are verified through the value in swinfo0.
  * The 16 ms bits are used as verification, the 16 lbs are for local handle id */
#define T4_CMD_SWINFO0_ADD_MAC_ID  		0x11100000  /* Identifies add mac command */
#define T4_CMD_SWINFO0_DEL_MAC_ID  		0x11110000  /* Identifies del mac command */
#define T4_CMD_SWINFO0_ADD_IP_ID		0x22200000  /* Identifies the add IP command */
#define T4_CMD_SWINFO0_DEL_IP_ID		0x22210000  /* Identifies the del IP command */
#define T4_CMD_SWINFO0_ADD_PORT_ID		0x22220000  /* Identifies the add port command */
#define T4_CMD_SWINFO0_DEL_PORT_ID		0x22230000  /* Identifies the del port command */
#define T4_CMD_SWINFO0_REP_IP_ID		  T4_CMD_SWINFO0_ADD_IP_ID  /* Identifies the replace IP command */
#define T4_CMD_SWINFO0_STATS_REQ_ID		0x33330000	/* Identifies the req stats command */
#define T4_CMD_SWINFO0_CRC_CFG_ID		0x44400000  /* Identifies the CRC config command */
#define T4_CMD_SWINFO0_GLOBAL_CFG_ID   	0x44410000  /* Identifies the Global config command */
#define T4_CMD_SWINFO0_EROUTE_CFG_ID   	0x44420000  /* Identifies the EROUTE config command */
#define T4_CMD_SWINFO0_USR_STATS_CFG_ID 0x44430000  /* Identifies the user Stats config command */
#define T4_CMD_SWINFO0_NAT_T_CFG_ID     0x44440000  /* Identifies the NAT-T config command */
#define T4_CMD_SWINFO0_EQoS_CFG_ID      0x44450000  /* Identifies the EQoS mode config command */
#define T4_CMD_SWINFO0_EQoS_GLOB_CFG_ID 0x44460000  /* Identifies the EQoS Global Control command */
#define T4_CMD_SWINFO0_PKT_ID			0x55550000  /* Identifies the packet as a data packet */
#define T4_CMD_SWINFO0_CAP_PKT_ID   	0x55560000  /* Identifies the packet as a L2 captured data packet */

#define T4_CMD_SWINFO0_TYPE_MASK		0xffff0000  /* Mask for the command type */
#define T4_CMD_SWINFO0_ID_MASK			0x0000ffff  /* Mask for the local ID */


 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T4_HANDLE_UNCONFIGURED = 0,
	T4_HANDLE_PENDING_ACK,
	T4_HANDLE_ACTIVE,
	T4_HANDLE_DISABLED
};

 typedef struct t4Handles_s  {

  	paHandleL2L3_t  paHandle;     /* The L3 handle returned by the PA LLD */

 	unsigned int			state;		  /* T4_HANDLE_UNCONFIGURED = handle not configured
 								   * T4_HANDLE_PENDING_ACK = handle configured and sent to pa
 								   * T4_HANDLE_ACTIVE = handle creation acknowledged by pa
 								   * T4_HANDLE_DISABLED = handle was created then released */

 } t4Handles_t;

 typedef struct t4HandlesL4_s  {

 	paHandleL4_t   paHandle;

 	unsigned int   state;

 } t4HandlesL4_t;

 /* A grouping of run time created grouped together to make cleanup easier on
  * error exit */
 typedef struct t4TestEncap_s  {
 	tFramework_t  *tf;
 	paTest_t      *pat;

 	/* The +1 is a place holder handle used to pass known invalid configuration into the PA LLD */
 	t4Handles_t    l2Handles[T4_NUM_LOCAL_L2_HANDLES+1];		/* MAC handles */
 	t4Handles_t    l3Handles[T4_NUM_LOCAL_L3_HANDLES+1];		/* IP handles  */
    t4Handles_t    vlHandles[T4_NUM_LOCAL_VL_HANDLES+1];        /* vlink handles  */
 	t4HandlesL4_t  l4Handles[T4_NUM_LOCAL_L4_HANDLES+1];		/* UDP/TCP handles */

 } t4TestEncap_t;

static paSysStats_t paTestL4ExpectedStats;    /* Expected stats results */

#define T4_NUM_USR_STATS                   2
#define T4_USR_STATS_ID_L2_PADDING_ERR     0
#define T4_USR_STATS_ID_L2_TX_PADDING      1

static  pauUsrStatsEntry_t t4UsrStatsTbl[T4_NUM_USR_STATS] =
{
    /* index, lnkId, fAlloc, f64b, fByteCnt */
    {0, PAU_USR_STATS_NO_LINK,     TRUE,   FALSE, FALSE},      /* L2 padding error count */
    {0, PAU_USR_STATS_NO_LINK,     TRUE,   FALSE, FALSE},      /* L2 padding count */
};


static paCmdReply_t cmdReply = {  pa_DEST_HOST,				/* Dest */
 							      0,						/* Reply ID (returned in swinfo0) */
 							   	  0,						/* Queue */
 							      0 };						/* Flow ID */

static paRouteInfo_t matchRoute[3] = {  {  	pa_DEST_HOST,		        /* Dest */
 								       		0,					        /* Flow ID */
 								       		0,					        /* queue */
 								       	   -1,					        /* Multi route */
 								       		0,					        /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL},                      /* No commands            */

 								        {  	pa_DEST_CONTINUE_PARSE_LUT2,/* Dest */
 								       		0,							/* Flow ID */
 								       		0,							/* queue */
 								           -1,							/* Multi route */
 								        	0,							/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL},                      /* No commands            */


 								  	    {	pa_DEST_CONTINUE_PARSE_LUT1,/* Dest */
 								  			0,							/* Flow ID */
 								  			0,							/* queue */
 								  		   -1,							/* Multi route */
 								  		    0,							/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL}                       /* No commands            */
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


static paRouteInfo_t   nfailRoute1 = {  pa_DEST_HOST,		/* Dest */
 										0,					/* Flow ID */
 										0,					/* queue */
 										-1,					/* Multi route */
 										0,					/* sw Info 0 */
                                        0,                  /* sw Info 1 */
                                        0,                  /* customType : not used  */
                                        0,                  /* customIndex: not used  */
                                        0,                  /* pkyType: for SRIO only */
                                        NULL};              /* No commands            */
static paRouteInfo2_t matchRoute2[7] = {
                                         {
                                            pa_ROUTE_INFO_VALID_PRIORITY_TYPE,   /* Priority Type  */
                                            pa_DEST_HOST,		        /* Dest */
 								       		0,					        /* Flow ID */
 								       		0,					        /* queue */
 								       	    0,					        /* Multi route */
 								       		T4_QoS_PKT_INDEX +	        /* sw Info 0 */
                                            T4_CMD_SWINFO0_PKT_ID,
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            pa_ROUTE_PRIORITY_DSCP,     /* Priority Type          */
                                            NULL                        /* Egress Flow Info */
                                            },

 								         {
                                            0,                          /* validBitMap            */
                                            pa_DEST_CONTINUE_PARSE_LUT2,/* Dest */
 								       		0,							/* Flow ID */
 								       		0,							/* queue */
 								           -1,							/* Multi route */
 								        	0,							/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL                        /* Egress Flow Info */
 								       		},

 								  	     {
                                            0,                          /* validBitMap            */
                                            pa_DEST_CONTINUE_PARSE_LUT1,/* Dest */
 								  			0,							/* Flow ID */
 								  			0,							/* queue */
 								  		   -1,							/* Multi route */
 								  		    0,							/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL                        /* Egress Flow Info */
 								       		},

 								  	     {
                                            pa_ROUTE_INFO_VALID_CTRLBITMAP,  /* validBitMap            */
                                            pa_DEST_CASCADED_FORWARDING_LUT1,/* Dest */
 								  			0,							/* Flow ID */
 								  			0,							/* queue */
 								  		   -1,							/* Multi route */
 								  		    T4_CMD_SWINFO0_CAP_PKT_ID,	/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                       /* Egress Flow Info */
                                            pa_ROUTE_INFO_L2_PKT_CAPTURE/* ctrlBitMap */
 								       	 },

                                         {
                                            pa_ROUTE_INFO_VALID_PRIORITY_TYPE,   /* Priority Type  */
                                            pa_DEST_HOST,		        /* Dest */
 								       		0,					        /* Flow ID */
 								       		0,					        /* queue */
 								       	    0,					        /* Multi route */
 								       		T4_IF_PKT_INDEX +	        /* sw Info 0 */
                                            T4_CMD_SWINFO0_PKT_ID,
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            pa_ROUTE_INTF_W_FLOW,       /* Priority Type          */
                                            NULL                        /* Egress Flow Info */
                                         },

                                         {
                                            pa_ROUTE_INFO_VALID_PRIORITY_TYPE |  /* priority type and command */
                                            pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,
                                            pa_DEST_HOST,		    /* Dest */
						       		        0,					    /* Flow ID */
						       		        0,					    /* queue */
						       	            0,					    /* Multi route */
 								       		T4_EQoS_PKT_INDEX +	    /* sw Info 0 */
                                            T4_CMD_SWINFO0_PKT_ID,
                                            0,                      /* sw Info 1 */
                                            0,                      /* customType : not used  */
                                            0,                      /* customIndex: not used  */
                                            pa_EMAC_PORT_1,         /* pkyType: for SRIO only */
                                            NULL,                   /* Command set: Next Route */
                                            pa_ROUTE_EQoS_MODE,     /* Priority Type          */
                                            NULL                    /* efOpInfo */
                                         },

                                         {
                                            pa_ROUTE_INFO_VALID_PRIORITY_TYPE |  /* priority type and command */
                                            pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,
                                            pa_DEST_HOST,		    /* Dest */
						       		        0,					    /* Flow ID */
						       		        0,					    /* queue */
						       	            0,					    /* Multi route */
 								       		T4_EQoS_PKT_INDEX +	1 + /* sw Info 0 */
                                            T4_CMD_SWINFO0_PKT_ID,
                                            0,                      /* sw Info 1 */
                                            0,                      /* customType : not used  */
                                            0,                      /* customIndex: not used  */
                                            pa_EMAC_PORT_2,         /* pkyType: for SRIO only */
                                            NULL,                   /* Command set: Next Route */
                                            pa_ROUTE_EQoS_MODE,     /* Priority Type          */
                                            NULL                    /* efOpInfo */
                                         },


                                    };

static paRouteInfo2_t   nfailRoute2 = {
                                        pa_ROUTE_INFO_VALID_PRIORITY_TYPE,  /* Priority Type  */
                                        pa_DEST_HOST,		                /* Dest */
 								       	0,					                /* Flow ID */
 								       	0,					                /* queue */
 								       	0,					                /* Multi route */
 								       	T4_IF_PKT_INDEX +	1 +             /* sw Info 0 */
                                        T4_CMD_SWINFO0_PKT_ID,
                                        0,                                  /* sw Info 1 */
                                        0,                                  /* customType : not used  */
                                        0,                                  /* customIndex: not used  */
                                        0,                                  /* pkyType: for SRIO only */
                                        NULL,                               /* No commands            */
                                        pa_ROUTE_INTF_W_FLOW,               /* Priority Type          */
                                        NULL                                /* Egress Flow Info */
 								      };

static paRouteInfo2_t   nfailRoute3 = {
                                        pa_ROUTE_INFO_VALID_PRIORITY_TYPE |  /* priority type and command */
                                        pa_ROUTE_INFO_VALID_PKTTYPE_EMAC,
                                        pa_DEST_HOST,		    /* Dest */
						       		    0,					    /* Flow ID */
						       		    0,					    /* queue */
						       	        0,					    /* Multi route */
 								       	T4_EQoS_PKT_INDEX +	2 + /* sw Info 0 */
                                        T4_CMD_SWINFO0_PKT_ID,
                                        0,                      /* sw Info 1 */
                                        0,                      /* customType : not used  */
                                        0,                      /* customIndex: not used  */
                                        pa_EMAC_PORT_3,         /* pkyType: for SRIO only */
                                        NULL,                   /* Command set: Next Route */
                                        pa_ROUTE_EQoS_MODE,     /* Priority Type          */
                                        NULL                    /* efOpInfo */

 								      };

/* CRC Configuration of CRC-32C for SCTP */
static paCrcConfig_t   t4CrcCfg = {
                                    pa_CRC_CONFIG_INVERSE_RESULT |
                                    pa_CRC_CONFIG_RIGHT_SHIFT,          /* ctrlBitfield */
                                    pa_CRC_SIZE_32,
                                    0x1EDC6F41,                         /* polynomial */
                                    0xFFFFFFFF                          /* initValue */
                                  };

/* Global configurations */
#define T4_NUM_64B_USR_STATS            64

static paUsrStatsConfig_t   t4UsrStatsCfg =
       {
            pa_USR_STATS_MAX_COUNTERS - T4_NUM_64B_USR_STATS,   /* Number of user stats (448)*/
            T4_NUM_64B_USR_STATS                                /* Number of 64-bit user stats */
       };


static paPacketControlConfig_t  t4PktCtrlCfg =
        {
            pa_PKT_CTRL_HDR_VERIFY_PPPoE   |                    /* ctrlBitMap */
            pa_PKT_CTRL_HDR_VERIFY_IP      |
            pa_PKT_CTRL_IP_FRAGS_TO_EROUTE |
#ifdef T4_TEST_L3OFFSET_USE_INNER_IP
            pa_PKT_CTRL_L3OFFSET_TO_INNER_IP |
#endif
            pa_PKT_CTRL_MAC_PADDING_CHK,
            0,                                                 /* rxPaddingErrStatsIndex */
            0                                                  /* txPaddingStatsIndex */
        };

static  paSysConfig_t  t4GlobalCfg =
        {
            NULL,                   /* pProtoLimit */
            NULL,                   /* pOutIpReassmConfig */
            NULL,                   /* pInIpReassmConfig */
            NULL,                   /* pCmdSetConfig */
            &t4UsrStatsCfg,         /* pUsrStatsConfig */
            NULL,                   /* pQueueDivertConfig */
            NULL,                   /* pPktControl */
            NULL                    /* pQueueBounceConfig */
        };

static  paSysConfig_t  t4GlobalCfg2 =
        {
            NULL,                   /* pProtoLimit */
            NULL,                   /* pOutIpReassmConfig */
            NULL,                   /* pInIpReassmConfig */
            NULL,                   /* pCmdSetConfig */
            NULL,                   /* pUsrStatsConfig */
            NULL,                   /* pQueueDivertConfig */
            &t4PktCtrlCfg,          /* pPktControl */
            NULL                    /* pQueueBounceConfig */
        };

 static int t4ErouteTypes[] = {
    pa_EROUTE_PPPoE_FAIL,
    pa_EROUTE_PPPoE_CTRL,
    pa_EROUTE_IP_FRAG,
    pa_EROUTE_IP_FAIL,
#ifdef NSS_GEN2
    pa_EROUTE_NAT_T_KEEPALIVE,
    pa_EROUTE_NAT_T_CTRL,
    pa_EROUTE_NAT_T_FAIL
#endif
 };

#define T4_NUM_EXCEPTION_ROUTES    sizeof(t4ErouteTypes)/sizeof(int)


 static paRouteInfo2_t t4Eroutes[] = {

    /* PPPoE Fail */
 	{
       0,                   /* valdBitMap */
       pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_PPPoE_FAIL + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T4_PPPoE_FAIL_PKT_INDEX + T4_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL,                /* No commands */
       0,                   /* Priority Type */
       NULL                 /* Egress Flow Info */
    },

    /* PPPoE Control Packet */
 	{
       0,                   /* valdBitMap */
       pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_PPPoE_CTRL + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T4_PPPoE_CTRL_PKT_INDEX + T4_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL,                /* No commands */
       0,                   /* Priority Type */
       NULL                 /* Egress Flow Info */
    },

    /* IP Frag */
 	{
        pa_ROUTE_INFO_VALID_PRIORITY_TYPE,   /* Priority Type  */
        pa_DEST_HOST,		        /* Dest */
 		0,					        /* Flow ID */
        Q_IF_BASE + TF_FIRST_GEN_QUEUE, /* queue */
 		0,					        /* Multi route */
 		T4_IF_PKT_INDEX + 2 +       /* sw Info 0 */
        T4_CMD_SWINFO0_PKT_ID,
        0,                          /* sw Info 1 */
        0,                          /* customType : not used  */
        0,                          /* customIndex: not used  */
        0,                          /* pkyType: for SRIO only */
        NULL,                       /* No commands            */
        pa_ROUTE_INTF_W_FLOW,       /* Priority Type          */
        NULL                        /* Egress Flow Info */
    },

    /* IP Fail */
 	{
       0,                   /* valdBitMap */
       pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_IP_FAIL + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T4_IP_FAIL_PKT_INDEX + T4_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL,                /* No commands */
       0,                   /* Priority Type */
       NULL                 /* Egress Flow Info */
    },

#ifdef NSS_GEN2

    /* NAT-T Keepalive */
 	{
       0,                   /* valdBitMap */
       pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_NAT_T + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T4_NAT_T_KEEPALIVE_PKT_INDEX + T4_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL,                /* No commands */
       0,                   /* Priority Type */
       NULL                 /* Egress Flow Info */
    },

    /* NAT-T Control */
 	{
       0,                   /* valdBitMap */
       pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_NAT_T + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T4_NAT_T_CTRL_PKT_INDEX + T4_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL,                /* No commands */
       0,                   /* Priority Type */
       NULL                 /* Egress Flow Info */
    },

    /* NAT-T FAIL */
 	{
       0,                   /* valdBitMap */
       pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_NAT_T + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T4_NAT_T_FAIL_PKT_INDEX + T4_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */
       0,                   /* customType : not used */
       0,                   /* customIndex: not used */
       0,                   /* pkyType: for SRIO only */
       NULL,                /* No commands */
       0,                   /* Priority Type */
       NULL                 /* Egress Flow Info */
    },

#endif

};

static  paIpsecNatTConfig_t  t4NatTCfg =
        {
            pa_IPSEC_NAT_T_CTRL_ENABLE |     /* ctrlBitMap */
            pa_IPSEC_NAT_T_CTRL_LOC_LUT1,
            4500                             /* UDP port number */
        };


/*
 * User-definded Statitics Map
 *
 * Group 1: counter 0,  packet Counter (Rx Padding Error) no link
 *          counter 1,  packet Counter (Tx Padding) no link
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (t4UsrStatsGroup1, ".testPkts")
#endif

static paUsrStatsCounterEntryConfig_t t4UsrStatsGroup1[T4_NUM_USR_STATS];

#ifdef _TMS320C6X
#pragma DATA_SECTION(t4UsrStatsSetup, ".testPkts")
#endif
static pauUsrStatsSetup_t  t4UsrStatsSetup[] = {
    /* entry 0 */
    {
        sizeof(t4UsrStatsGroup1)/sizeof(paUsrStatsCounterEntryConfig_t),    /* number of entries */
        t4UsrStatsGroup1,                                                   /* counter Info table */
        pa_OK                                                               /* Expected return value */
    }
};

/* EQoS Mode configurations  */
#define T4_EQoS_EGRESS_DEF_PRI         2
static paEQosModeConfig_t t4EQoSModeCfg[] =
{
    /* Entry 0 */
    /* Default Priority only */
    {

        0,                                  /* ctrlBitMap */
        {{0, 0}, {0, 0}, {0, 0}, {0, 0},    /* pbitMap */
         {0, 0}, {0, 0}, {0, 0}, {0, 0}},
        {{0, 0}, {0, 0}, {0, 0}, {0, 0},    /* dscpMap */
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0}},
        pa_EMAC_PORT_0,                     /* port number */
        1,                                  /* ingressDefPri */
        0,                                  /* vlanId */
        0,                                  /* flowBase */
        0                                   /* queueBase */
    },

    /* Entry 1 */
    /* DP-Bit mode */
    {
        pa_IF_EQoS_ROUTE_DP_BIT_MODE |      /* ctrlBitMap */
        pa_IF_EQoS_VLAN_OVERRIDE_ENABLE,
        {{0, 0}, {0, 0}, {0, 0}, {0, 0},    /* pbitMap */
         {0, 1}, {0, 1}, {0, 2}, {0, 2}},
        {{0, 8},  {0, 8},  {0, 8},  {0, 8}, /* dscpMap */
         {0, 8},  {0, 8},  {0, 8},  {0, 8},
         {0, 9},  {0, 9},  {0, 9},  {0, 9},
         {0, 9},  {0, 9},  {0, 9},  {0, 9},
         {0, 10}, {0, 10}, {0, 10}, {0, 10},
         {0, 10}, {0, 10}, {0, 10}, {0, 10},
         {0, 11}, {0, 11}, {0, 11}, {0, 11},
         {0, 11}, {0, 11}, {0, 11}, {0, 11},
         {0, 12}, {0, 12}, {0, 12}, {0, 12},
         {0, 12}, {0, 12}, {0, 12}, {0, 12},
         {0, 13}, {0, 13}, {0, 13}, {0, 13},
         {0, 13}, {0, 13}, {0, 13}, {0, 13},
         {0, 14}, {0, 14}, {0, 14}, {0, 14},
         {0, 14}, {0, 14}, {0, 14}, {0, 14},
         {0, 15}, {0, 15}, {0, 15}, {0, 15},
         {0, 15}, {0, 15}, {0, 15}, {0, 15}},
        pa_EMAC_PORT_1,                     /* port number */
        2,                                  /* ingressDefPri */
        0x0666,                             /* vlanId */
        0,                                  /* flowBase */
        0                                   /* queueBase */
    },

    /* Entry 2 */
    /* DP-Bit mode with priority override */
    {

        pa_IF_EQoS_ROUTE_DP_BIT_MODE |      /* ctrlBitMap */
        pa_IF_EQoS_PRIORITY_OVERRIDE_ENABLE,
        {{0, 3}, {0, 3}, {0, 3}, {0, 3},    /* pbitMap */
         {0, 4}, {0, 4}, {0, 5}, {0, 5}},
        {{0, 0}, {0, 0}, {0, 0}, {0, 0},    /* dscpMap */
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0},
         {0, 0}, {0, 0}, {0, 0}, {0, 0}},
        pa_EMAC_PORT_2,                     /* port number */
        3,                                  /* ingressDefPri */
        0,                                  /* vlanId */
        0,                                  /* flowBase */
        0                                   /* queueBase */
    },

    /* Entry 3 */
    /* DSCP mode */
    {
        pa_IF_EQoS_VLAN_OVERRIDE_ENABLE,    /* ctrlBitMap */
        {{0, 6}, {0, 6}, {0, 6}, {0, 6},    /* pbitMap */
         {0, 7}, {0, 7}, {0, 7}, {0, 7}},
        {{0, 15}, {0, 14}, {0, 13}, {0, 12},/* dscpMap */
         {0, 11}, {0, 10}, {0, 9},  {0, 8},
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8},
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8},
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8},
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8},
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8},
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8},
         {0, 15}, {0, 14}, {0, 13}, {0, 12},
         {0, 11}, {0, 10}, {0, 9},  {0, 8}},
        pa_EMAC_PORT_3,                     /* port number */
        4,                                  /* ingressDefPri */
        0x0777,                             /* vlanId */
        0,                                  /* flowBase */
        0                                   /* queueBase */
    }
};

#define T4_NUM_EQoS_PORTS         sizeof(t4EQoSModeCfg)/sizeof(paEQosModeConfig_t)



static paUsrStats_t  paTestExpectedUsrStats;

 void t4Cleanup (t4TestEncap_t *tencap, paTestStatus_t testStatus)
{
 	int 	       i;
 	int  	       cmdDest;
 	uint16_t	   cmdSize;
 	paReturn_t     paret;
 	Cppi_HostDesc *hd;
 	paTestStatus_t newStatus;

 	/* Delete active L4 handles */
 	for (i = 0; i < T4_NUM_LOCAL_L4_HANDLES; i++)  {

		cmdReply.replyId = T4_CMD_SWINFO0_DEL_PORT_ID + i;  /* T4_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l4Handles[i].state == T4_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T4_HANDLE_ACTIVE))  {
			hd = testCommonDelL4Handles (tencap->tf, tencap->l4Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				continue;
			}

			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				testStatus = PA_TEST_FAILED;
 			}

 			/* Recycle the command packet as well */
 			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			if (hd == NULL)  {
 				System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				continue;
 			}
 			testCommonRecycleLBDesc (tencap->tf, hd);
 		}
 	}

 	/* Delete active L3 Handles */
 	for (i = 0; i < T4_NUM_LOCAL_L3_HANDLES; i++)  {
 		cmdReply.replyId = T4_CMD_SWINFO0_DEL_IP_ID + i;  /* T4_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l3Handles[i].state == T4_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T4_HANDLE_ACTIVE))  {
			hd = testCommonDelHandle (tencap->tf, &tencap->l3Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				System_flush();
				continue;
			}

			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
			paTestL4ExpectedStats.classify1.nPackets += 1;

			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				System_flush();
				testStatus = PA_TEST_FAILED;
 			}
 			/* Recycle the command packet as well */
 			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			if (hd == NULL)  {
 				System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				continue;
 			}
 			testCommonRecycleLBDesc (tencap->tf, hd);
 		}
 	}


 	/* Delete active L2 Handles */
 	for (i = 0; i < T4_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T4_CMD_SWINFO0_DEL_MAC_ID + i;  /* T4_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l2Handles[i].state == T4_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T4_HANDLE_ACTIVE))  {
			hd = testCommonDelHandle (tencap->tf, &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				continue;
			}

			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
			paTestL4ExpectedStats.classify1.nPackets += 1;

			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				testStatus = PA_TEST_FAILED;
 			}

 			/* Recycle the command packet as well */
 			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			if (hd == NULL)  {
 				System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				continue;
 			}
 			testCommonRecycleLBDesc (tencap->tf, hd);
 		}
 	}

    /* Delete Virtual Link */
 	for (i = 0; i < T4_NUM_LOCAL_VL_HANDLES; i++)  {

        if(tencap->vlHandles[i].paHandle == NULL)continue;

        if((paret = Pa_delVirtualLink(tencap->tf->passHandle,        /* PA Handle */
                                      &(tencap->vlHandles[i].paHandle) /* Virtual link handle table*/
                                     )) != pa_OK)
        {
            System_printf ("%s: (%s:%d): Command Pa_delVirtualLink failed with error code %d\n", tfName, __FILE__, __LINE__, paret);
            testStatus = PA_TEST_FAILED;
        }
 	}

    /* Delete Virtual Link */
 	for (i = 0; i < T4_NUM_LOCAL_VL_HANDLES; i++)  {

        if(tencap->vlHandles[i].paHandle == NULL)continue;

        if((paret = Pa_delVirtualLink(tencap->tf->passHandle,        /* PA Handle */
                                      &(tencap->vlHandles[i].paHandle) /* Virtual link handle table*/
                                     )) != pa_OK)
        {
            System_printf ("%s: (%s:%d): Command Pa_delVirtualLink failed with error code %d\n", tfName, __FILE__, __LINE__, paret);
            testStatus = PA_TEST_FAILED;
        }

        tencap->vlHandles[i].state = T4_HANDLE_DISABLED;

 	}

 	/* Pop any descriptors off of the return queues and restore them to the linked buffer queues or the free Q */
 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_CMD_RECYCLE]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 		testCommonRecycleLBDesc (tencap->tf, hd);
 	}

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_CMD_REPLY]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_REPLY])) & ~15);
 		testCommonRecycleLBDesc (tencap->tf, hd);
 	}

 	 while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_MATCH]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
 		Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

 	 while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_NFAIL]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_NFAIL])) & ~15);
 		Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_PARSE_ERR]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_PARSE_ERR])) & ~15);
 		Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_PPPoE_FAIL]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_PPPoE_FAIL])) & ~15);
 		Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_PPPoE_CTRL]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_PPPoE_CTRL])) & ~15);
 		Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_IP_FAIL]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_IP_FAIL])) & ~15);
 		Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

 	newStatus = testCommonCheckStats (tencap->tf, tencap->pat, tfName, &paTestL4ExpectedStats, tencap->tf->QLinkedBuf1,
	                       tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
		testStatus = PA_TEST_FAILED;

	/* Return result */
    tencap->pat->testStatus = testStatus;

    /* Return */
    Task_exit();
}



 /* Check for pa lld return errors. Exit the test on any error condition */
 static void t4HandleError (t4TestEncap_t *tencap, paReturn_t paret, Cppi_HostDesc *hd, int line)
 {

 	if (paret == pa_OK)
 		return;

 	System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, line, paret);

 	if ((hd != NULL) && testCommonRecycleLBDesc (tencap->tf, hd))
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);

	t4Cleanup (tencap, PA_TEST_FAILED);  /* No return */

 }


 /* Look for replies to add Ip commands and verify the results */
void t4L3CmdRep (t4TestEncap_t *tencap, t4IpPaSetup_t *ipSetup, uint32_t maxIdx)
{
	Cppi_HostDesc  *hd;
	uint32_t         *swinfo;
	paReturn_t      paret;
	uint32_t			swinfoType;
	uint32_t			swinfoIdx;
	paEntryHandle_t	reth;
    int			    htype;
    int             cmdDest;

	while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_CMD_REPLY]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_REPLY])) & ~15);

		    Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);

		    swinfoType = swinfo[0] & T4_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T4_CMD_SWINFO0_ID_MASK;

            if (swinfoType != T4_CMD_SWINFO0_ADD_IP_ID)  {
                System_printf ("%s (%s:%d): found packet in command reply queue without add IP swinfo type (found 0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            if (swinfoIdx >= maxIdx)  {
            	System_printf ("%s (%s:%d): found packet in command reply queue with an invalid index %d (>= %d)\n", tfName, __FILE__, __LINE__, swinfoIdx, maxIdx);
            	System_flush();
            	testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            ipSetup[swinfoIdx].acked = TRUE;
            paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);

            if (paret != ipSetup[swinfoIdx].ret)  {
                System_printf ("%s (%s:%d): Pa_forwardResult returned %d, expected %d\n", tfName, __FILE__, __LINE__, paret, ipSetup[swinfoIdx].ret);
                System_flush();
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            testCommonRecycleLBDesc (tencap->tf, hd);

            tencap->l3Handles[ipSetup[swinfoIdx].handleIdx].state = T4_HANDLE_ACTIVE;

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
}

static paTestStatus_t t4ExceptionRoutes (t4TestEncap_t *tencap)
{
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    int             i;

    /* Issue the exception route command */
    cmdReply.replyId  = T4_CMD_SWINFO0_EROUTE_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
    for(i = 0; i < T4_NUM_EXCEPTION_ROUTES; i++)
        t4Eroutes[i].flowId = tencap->tf->tfFlowNum[0];

 	hd = testCommonConfigExceptionRoute2 (tencap->tf, T4_NUM_EXCEPTION_ROUTES, t4ErouteTypes, t4Eroutes,
 	                                      tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                                      &cmdReply, &cmdDest, &cmdSize, &paret);

 	t4HandleError (tencap, paret, hd, __LINE__); /* Will not return on error */

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

	if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, T4_CMD_SWINFO0_EROUTE_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): testCommonConfigExceptionRoute failed\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
 	}

 	/* Recycle the command packet as well */
 	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 		return(PA_TEST_FAILED);
 	}
 	testCommonRecycleLBDesc (tencap->tf, hd);

	return (PA_TEST_PASSED);
}

static paTestStatus_t t4GlobalConfiguration (t4TestEncap_t *tencap, paSysConfig_t *pCfg)
{
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paCtrlInfo_t    ctrlInfo;

    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = *pCfg;
    cmdReply.replyId  = T4_CMD_SWINFO0_GLOBAL_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
 	hd = testCommonGlobalConfig (tencap->tf, &ctrlInfo,
 	                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);

 	t4HandleError (tencap, paret, hd, __LINE__); /* Will not return on error */

    if (hd == NULL)  {

   	  System_printf ("%s: (%s:%d): Failure in GlobalConfig command\n", tfName, __FILE__, __LINE__);
   	  return (PA_TEST_FAILED);
    }

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

	if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, T4_CMD_SWINFO0_GLOBAL_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): testCommonGlobalConfig failed\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
 	}

 	/* Recycle the command packet as well */
 	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 		return(PA_TEST_FAILED);
 	}
 	testCommonRecycleLBDesc (tencap->tf, hd);

	return (PA_TEST_PASSED);
}

static paTestStatus_t t4NatTConfiguration (t4TestEncap_t *tencap, int enable)
{
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t            cmdSize;
    paCtrlInfo_t    ctrlInfo;

    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_IPSEC_NAT_T_CONFIG;
    ctrlInfo.params.ipsecNatTDetCfg = t4NatTCfg;
    if (!enable)
    {
        ctrlInfo.params.ipsecNatTDetCfg.ctrlBitMap  &= ~pa_IPSEC_NAT_T_CTRL_ENABLE;
    }

    cmdReply.replyId  = T4_CMD_SWINFO0_NAT_T_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
 	hd = testCommonGlobalConfig (tencap->tf, &ctrlInfo,
 	                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);

   if (hd == NULL)  {

   	 System_printf ("%s: (%s:%d): Failure in NAT-T Config command\n", tfName, __FILE__, __LINE__);
   	 return (PA_TEST_FAILED);
   }

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    #ifdef NSS_GEN2
 	paTestL4ExpectedStats.classify1.nPackets += 1;
    #else
 	paTestL4ExpectedStats.classify2.nPackets += 1;
    #endif

	if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, T4_CMD_SWINFO0_NAT_T_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): testCommonGlobalConfig failed\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
 	}

 	/* Recycle the command packet as well */
 	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 		return(PA_TEST_FAILED);
 	}
 	testCommonRecycleLBDesc (tencap->tf, hd);

	return (PA_TEST_PASSED);
}

static paEQosModeConfig_t  defEQoSModeCfg[T4_NUM_EQoS_PORTS];

static paTestStatus_t t4ConfigEQoSModes(t4TestEncap_t *tencap, int clear)
{
	int 			i;
	Cppi_HostDesc  *hd = NULL;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;

	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */

    /* Issue EQoS mode configuration command */
    cmdReply.replyId  = T4_CMD_SWINFO0_EQoS_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId   = tencap->tf->tfFlowNum[0];
    if (!clear)
    {
 	    hd = testCommonConfigEQoSMode (tencap->tf, T4_NUM_EQoS_PORTS, t4EQoSModeCfg,
 	                                   tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                                   &cmdReply, &cmdDest, &cmdSize, &paret);
    }
    else
    {
        memset(defEQoSModeCfg, 0, sizeof(defEQoSModeCfg));
        for (i = 0; i < T4_NUM_EQoS_PORTS; i++)
        {
            defEQoSModeCfg[i].port = t4EQoSModeCfg[i].port;
        }

 	    hd = testCommonConfigEQoSMode (tencap->tf, T4_NUM_EQoS_PORTS, defEQoSModeCfg,
 	                                   tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                                   &cmdReply, &cmdDest, &cmdSize, &paret);
    }

    if (hd == NULL)  {

   	  System_printf ("%s: (%s:%d): Failure in ConfigEQoSMode command with error code = %d\n", tfName, __FILE__, __LINE__, paret);
   	  return (PA_TEST_FAILED);

    }

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

	if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, T4_CMD_SWINFO0_EQoS_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): t4ConfigEQoSModes failed\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
 	}

 	/* Recycle the command packet as well */
 	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 		return(PA_TEST_FAILED);
 	}
 	testCommonRecycleLBDesc (tencap->tf, hd);


	return (PA_TEST_PASSED);
}

static paTestStatus_t t4EQoSModeGlobalControl (t4TestEncap_t *tencap, int enable)
{
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
	paSysConfig_t   paSysCfg;
    paCtrlInfo_t    ctrlInfo;

    paPacketControl2Config_t pktControl2;

	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */

    memset(&paSysCfg, 0, sizeof(paSysConfig_t));
    memset(&pktControl2, 0,   sizeof(paPacketControl2Config_t));
    memset (&ctrlInfo, 0, sizeof (paCtrlInfo_t));

    pktControl2.validBitMap = pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_EQoS_MODE;
    if (enable)
    {
        pktControl2.ctrlBitMap   = pa_PKT_CTRL_EMAC_IF_EGRESS_EQoS_MODE;
        pktControl2.egressDefPri = T4_EQoS_EGRESS_DEF_PRI;
    }
    paSysCfg.pPktControl2 = &pktControl2;

  	/* set System Global default configuration */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = paSysCfg;
    cmdReply.replyId  = T4_CMD_SWINFO0_EQoS_GLOB_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId   = tencap->tf->tfFlowNum[0];

 	hd = testCommonGlobalConfig (tencap->tf, &ctrlInfo,
 	                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);

    if (hd == NULL)  {
        System_printf ("%s: (%s:%d): Failure in EQoS Global Control command with error code = %d\n", tfName, __FILE__, __LINE__, paret);
   	    return (PA_TEST_FAILED);
    }

    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

	if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, T4_CMD_SWINFO0_EQoS_GLOB_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): testCommonGlobalConfig failed\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
 	}

 	/* Recycle the command packet as well */
 	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 		return(PA_TEST_FAILED);
 	}
 	testCommonRecycleLBDesc (tencap->tf, hd);

	return (PA_TEST_PASSED);
}

/* Simple User-Statistics routine: It does not process link and counter type */
static void t4UpdateUsrStats(paUsrStats_t* pStats, uint16_t cntIndex)
{
    if (cntIndex < T4_NUM_64B_USR_STATS)
    {
        pStats->count64[cntIndex]++ ;
    }
    else
    {
        pStats->count32[cntIndex - T4_NUM_64B_USR_STATS]++;
    }
}

/*
 * Utility function to derive the LUT1 index from the local index
 * It is used to verify the Pa_addIP() with application specific index
 * The index should be consistent with the system allocated LUT1 index
 */
static int t4GetLUT1Index(int handleIndex)
{

#ifndef __LINUX_USER_SPACE
    if (handleIndex >= T4_NUM_LOCAL_L3_HANDLES)
    {
        return (pa_LUT1_INDEX_NOT_SPECIFIED);
    }

    handleIndex = handleIndex % 64;

    /*
     * Special case for repeated entries
     *
     */
    if((handleIndex == 7) || (handleIndex == 18))
        return (pa_LUT1_INDEX_NOT_SPECIFIED);

    if (handleIndex & 1)
    {
        return (pa_LUT1_INDEX_NOT_SPECIFIED);
    }
    else
    {
        return (63 - handleIndex);
    }
#else

    return (pa_LUT1_INDEX_NOT_SPECIFIED);

#endif

}

#ifndef NSS_GEN2
static paTestStatus_t t4ReplaceIp (t4TestEncap_t  *tencap,
                                   t4RepIpPaSetup_t *repIpSetup,
                                   t4IpPaSetup_t    *ipSetup,
                                   int n,
                                   char *id)
{
#ifndef __LINUX_USER_SPACE
	Int 			i, j, m, index;
  int       lutIndex;
	paReturn_t      paret;
	Int  			      cmdDest;
	UInt16			    cmdSize;
	Cppi_HostDesc  *hd;
  paHandleL2L3_t *l3Handle;
  paHandleL2L3_t  prevLink;
  paLUT1Info_t lut1Info;

  for (i = 0; i < n; i++)  {
    index            = repIpSetup[i].ipSetupTblIndex;
    cmdReply.replyId = T4_CMD_SWINFO0_REP_IP_ID + i;
    cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

    l3Handle = &tencap->l3Handles[ipSetup[index].handleIdx].paHandle;
    prevLink = tencap->l3Handles[ipSetup[index].lHandleIdx].paHandle;

    paret = Pa_getLUT1Info(tencap->tf->passHandle, *l3Handle, &lut1Info);
    if (paret != pa_OK)
      return (PA_TEST_FAILED);

    lutIndex = lut1Info.lut1Index;

    /* Update the test cases as per configuration */
    if (repIpSetup[i].repType == T4_CMD_REPLACE_WITH_ERR_INDEX)
       lutIndex --; /* modify the lutIndex to wrong value */
    else if ( repIpSetup[i].repType == T4_CMD_REPLACE_WITH_NO_INDEX)
       lutIndex = pa_LUT1_INDEX_NOT_SPECIFIED;

    hd = testCommonAddIp3 (tencap->tf,
                           lut1Info.lut1Inst,
                           lutIndex,
                           &ipSetup[index].ipInfo,
                           &matchRoute2[ipSetup[index].routeIdx],
                           &nfailRoute2,
                           l3Handle,  /* ret handle */
                           prevLink,   /* prevlink */
                           NULL,      /* nextlink */
                           tencap->tf->QGen[Q_CMD_RECYCLE],
                           tencap->tf->QLinkedBuf1,
                           &cmdReply, &cmdDest, &cmdSize, pa_PARAM_CTRL_REPLACE, &paret);

		if (hd == NULL)  {

		    /* It's not a failure if the return code from pa was expected */
		    if (paret == repIpSetup[i].ret)  {
			    continue;
		    }

		    System_printf ("%s: (%s:%d): Failure in common addIp command, %s entry number %d\n", tfName, __FILE__, __LINE__, id, ipSetup[i].seqId);
		    t4HandleError (tencap, paret, hd, __LINE__);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[ipSetup[index].handleIdx].state = T4_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

 		t4L3CmdRep (tencap, ipSetup, n);

  }

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t4L3CmdRep (tencap, ipSetup, n);

		for (j = m = 0; j < n; j++)  {
			if (ipSetup[j].acked == TRUE)
				m += 1;
		}

		if (m == n)
			break;
		else
			utilCycleDelay (1000);
	}

	if (i == 100)  {
		System_printf ("%s: (%s:%d): Command %d (out of %d) replace for addIp commands were acked (%s)\n", tfName, __FILE__, __LINE__, m, n, id);
		return (PA_TEST_FAILED);
	}
#endif
	return (PA_TEST_PASSED);

}
#endif

static paTestStatus_t t4OpenIp (t4TestEncap_t *tencap,
                                t4IpPaSetup_t *ipSetup,
                                int n,
                                t4Handles_t *linkedHandles,
                                char *id)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    paRouteInfo_t  *pNfailRoute;

	for (i = 0; i < n; i++)  {
		//cmdReply.replyId = T4_CMD_SWINFO0_ADD_IP_ID + ipSetup[i].seqId;
        cmdReply.replyId = T4_CMD_SWINFO0_ADD_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

        pNfailRoute = (ipSetup[i].routeIdx == 1)?&nfailRoute1:&nfailRoute;
        if(ipSetup[i].vlink){
            /*
             * Only one virtual link and a pair of entries which uses the virtual link is supported.
             * Need to enhance the test template structure to support multiple virtual link.
             */
            /* Virtual link is being requested but not created, we are adding outer ip */
            if(ipSetup[i].handleIdx < T4_NUM_LOCAL_L3_OUTER_IP_HANDLES)
            {
                /* Outer IP entry */
                hd = testCommonAddIp3 (tencap->tf,
                                    ipSetup[i].lutInst,
                                    t4GetLUT1Index(ipSetup[i].handleIdx),
                                    &ipSetup[i].ipInfo,
                                    &matchRoute2[ipSetup[i].routeIdx],
                                    &nfailRoute2,
                                    &tencap->l3Handles[ipSetup[i].handleIdx].paHandle,  /* ret handle */
                                    linkedHandles[ipSetup[i].lHandleIdx].paHandle,      /* prevlink */
                                    tencap->vlHandles[0].paHandle,   /* nextlink */
                                    tencap->tf->QGen[Q_CMD_RECYCLE],
                                    tencap->tf->QLinkedBuf2,
                                    &cmdReply, &cmdDest, &cmdSize, 0, &paret);
            }
            else /* we are adding inner ip entry */
            {
                hd = testCommonAddIp3 (tencap->tf,
                                    ipSetup[i].lutInst,
                                    t4GetLUT1Index(ipSetup[i].handleIdx),
                                    &ipSetup[i].ipInfo,
                                    &matchRoute2[ipSetup[i].routeIdx],
                                    &nfailRoute2,
                                    &tencap->l3Handles[ipSetup[i].handleIdx].paHandle,  /* ret handle */
                                    tencap->vlHandles[ipSetup[i].lHandleIdx].paHandle,   /* prevlink */
                                    NULL,      /* nextlink */
                                    tencap->tf->QGen[Q_CMD_RECYCLE],
                                    tencap->tf->QLinkedBuf2,
                                    &cmdReply, &cmdDest, &cmdSize, 0, &paret);
            }

        }
        else /* adding non-virtualLink IP entry */
        {
            /*
             * Special test case for cascaded forwarding
             *
             */
            if ((ipSetup[i].handleIdx < T4_NUM_LOCAL_L3_OUTER_IP_HANDLES) &&
                (ipSetup[i].lHandleIdx >= T4_FIRST_MAC2_HANDLE))
            {
                paRouteInfo2_t   *nfailRoute = (ipSetup[i].handleIdx == T4_LOCAL_L3_EQoS_NFAIL_INDEX)?
                                               &nfailRoute3:&nfailRoute2;

                /* Cascaded forwarding traffic */
                hd = testCommonAddIp3 (tencap->tf,
                                    ipSetup[i].lutInst,
                                    t4GetLUT1Index(ipSetup[i].handleIdx),
                                    &ipSetup[i].ipInfo,
                                    &matchRoute2[ipSetup[i].routeIdx],
                                    nfailRoute,
                                    &tencap->l3Handles[ipSetup[i].handleIdx].paHandle,  /* ret handle */
                                    linkedHandles[ipSetup[i].lHandleIdx].paHandle,   /* prevlink */
                                    NULL,      /* nextlink */
                                    tencap->tf->QGen[Q_CMD_RECYCLE],
                                    tencap->tf->QLinkedBuf2,
                                    &cmdReply, &cmdDest, &cmdSize, 0, &paret);
            }
            else
            {
                if (ipSetup[i].lHandleIdx >= 0)
                {
		        hd = testCommonAddIp2 (tencap->tf,
                                    ipSetup[i].lutInst,
                                    t4GetLUT1Index(ipSetup[i].handleIdx),
                                    &ipSetup[i].ipInfo,
                                    &matchRoute[ipSetup[i].routeIdx],
                                    pNfailRoute,
            					    &tencap->l3Handles[ipSetup[i].handleIdx].paHandle,
            					    linkedHandles[ipSetup[i].lHandleIdx].paHandle,
            					    tencap->tf->QGen[Q_CMD_RECYCLE],
                                    tencap->tf->QLinkedBuf2,
            					    &cmdReply, &cmdDest, &cmdSize, &paret);
                }
                else
                {
		        hd = testCommonAddIp2 (tencap->tf,
                                    ipSetup[i].lutInst,
                                    t4GetLUT1Index(ipSetup[i].handleIdx),
                                    &ipSetup[i].ipInfo,
                                    &matchRoute[ipSetup[i].routeIdx],
                                    pNfailRoute,
            					    &tencap->l3Handles[ipSetup[i].handleIdx].paHandle,
            					    NULL,
            					    tencap->tf->QGen[Q_CMD_RECYCLE],
                                    tencap->tf->QLinkedBuf2,
            					    &cmdReply, &cmdDest, &cmdSize, &paret);
                }

            }
		}

		if (hd == NULL)  {

		    /* It's not a failure if the return code from pa was expected */
		    if (paret == ipSetup[i].ret)  {
			    ipSetup[i].acked = TRUE;
			    continue;
		    }

		    System_printf ("%s: (%s:%d): Failure in common addIp command, %s entry number %d\n", tfName, __FILE__, __LINE__, id, ipSetup[i].seqId);
		    t4HandleError (tencap, paret, hd, __LINE__);
		}
        else if (paret == pa_DUP_ENTRY) {
            /* The ack will be handle for the first entry */
            ipSetup[i].acked = TRUE;
        }

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[ipSetup[i].handleIdx].state = T4_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

 		t4L3CmdRep (tencap, ipSetup, n);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t4L3CmdRep (tencap, ipSetup, n);

		for (j = m = 0; j < n; j++)  {
			if (ipSetup[j].acked == TRUE)
				m += 1;
		}

		if (m == n)
			break;
		else
			utilCycleDelay (1000);
	}

	if (i == 100)  {
		System_printf ("%s: (%s:%d): Command %d (out of %d) addIp commands were acked (%s)\n", tfName, __FILE__, __LINE__, m, n, id);
		return (PA_TEST_FAILED);
	}

	return (PA_TEST_PASSED);

}

static Cppi_HostDesc  *t4FormPktDataDescr (t4TestEncap_t *tencap, int idx)
{
	Cppi_HostDesc  *hd;
	Qmss_Queue      q;

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);
    if (hd == NULL)  {
        System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
        return (NULL);
    }

    /* Recycle the free descriptors right back to the free descriptor queue */
    q.qMgr = 0;
    q.qNum = tencap->tf->QfreeDesc;

    /* Setup the return for the descriptor */
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);

    /* Attach the data and set the length */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)t4PktInfo[idx].pkt), t4PktInfo[idx].pktLen);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t4PktInfo[idx].pktLen);

    return (hd);
}


/* Look for packets in the receive queue, verify the match */
int t4RxPkts (t4TestEncap_t *tencap, pauFifo_t *fifo)
{
	Cppi_HostDesc    *hd;
	unsigned int			  idx;
	int				  n, i;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;

	/* Look for packets in the receive queue */
	for (i = 0; i < 100; i++) {
		utilCycleDelay (1000);
        /*
         * Look for L2 Capture packets: Examine and recycle
         */
	    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_L2_CAPTURE]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_L2_CAPTURE])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received ESP packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

    		testCommonRecycleLBDesc (tencap->tf, hd);
	    }

    	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_MATCH]) > 0)  {

    		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
    		if (hd == NULL)  {
    			System_printf ("%s (%s:%d): Failed to pop a descriptor off the receive packet queue\n", tfName, __FILE__, __LINE__);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}

    		/* The packets must arrive in order */
    		idx = commonFifoPopElement (fifo, &n);
    		if (n == 0)  {
    			System_printf ("%s (%s:%d): Error - found an empty receive packet tracking index fifo\n", tfName, __FILE__, __LINE__);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}

    		/* Verify the parse information is correct */
    		if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
    			System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}

    		if (testCommonComparePktInfo (tfName, t4PktInfo[idx].info, pinfo))  {
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}

    		testCommonRecycleLBDesc (tencap->tf, hd);

    	}
		/* The packet index fifo must be empty */
		n = commonFifoGetCount (fifo);
		if (n != 0)  {
  		  /* Give some time for all the packets to get through the system */
		  utilCycleDelay (1000);
		}
		else
			break; /* come out fo the loop */
	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): Error - unable to recover all packets\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}

	return (0);

}

static Cppi_HostDesc *t4FormQoSPacket (tFramework_t *tf, paTest_t *pat, int pktIdx)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;

	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = tf->QfreeDesc;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);

  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t4QoSPktInfo[pktIdx].pkt)), t4QoSPktInfo[pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t4QoSPktInfo[pktIdx].pktLen);

  	return (hd);
}


static Cppi_HostDesc *t4GetQoSPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;

    for (index = Q_QoS_BASE; index <= (T4_NUM_QoS_QUEUES +  Q_QoS_BASE); index++)
    {

	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t4GetQoSPk: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }

            return (hd);
	    }

    }

   return (NULL);

}

/* Search the QoS queue for received packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t4ReceiveQoSPkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
    uint8_t            *pkt;
	uint32_t		      infoLen;
	int               i, j;
	unsigned int		      chan;
    int               count = 0;

    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (1000);

        /*
         * Look for L2 Capture packets: Examine and recycle
         */
	    while (Qmss_getQueueEntryCount(tf->QGen[Q_L2_CAPTURE]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_L2_CAPTURE])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received capture packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

    		testCommonRecycleLBDesc (tf, hd);
	    }

		while ((hd = t4GetQoSPkt (tf)) != NULL)  {

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((*swInfo & T4_CMD_SWINFO0_TYPE_MASK) != T4_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			chan = *swInfo & T4_CMD_SWINFO0_ID_MASK;
            count++;

            pkt = (uint8_t *)hd->buffPtr;

			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t4QoSPktInfo) / sizeof(pktTestInfo_t); j++)  {
				if (t4QoSPktInfo[j].idx == chan)  {
					tinfo = &t4QoSPktInfo[j + pkt[18]];
					break;
				}
			}

			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);

		}

        if(count >= expCount)
            break;

	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): QoS Queue Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}

	return (0);
}

static Cppi_HostDesc *t4FormEQoSPacket (tFramework_t *tf, paTest_t *pat, int pktIdx)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
    Cppi_DescTag   tag;

    memset(&tag, 0, sizeof(tag));

	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = tf->QGen[Q_DPKT_RECYCLE];
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);

    /* Set Input interface number */
    tag.srcTagHi = t4eQoSPktTestInfo[pktIdx].emacPort;
    Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)hd, &tag);

#ifdef NSS_GEN2
    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0x8);
#endif

  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t4eQoSPktTestInfo[pktIdx].pktInfo.pkt)), t4eQoSPktTestInfo[pktIdx].pktInfo.pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t4eQoSPktTestInfo[pktIdx].pktInfo.pktLen);

  	return (hd);
}

static Cppi_HostDesc *t4GetEQoSPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;

    for (index = Q_QoS_BASE; index <= (T4_NUM_EQoS_QUEUES +  Q_QoS_BASE); index++)
    {

	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t4GetEQoSPkt: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }

            return (hd);
	    }

    }

   return (NULL);

}

/* Search the EQoS queue for received packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t4ReceiveEQoSPkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		 *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
    uint8_t          *pkt;
	uint32_t		  infoLen;
	int               i, j;
	unsigned int	  chan;
    int               count = 0;

    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (5000);
        /*
         * Look for L2 Capture packets: Examine and recycle
         */
	    while (Qmss_getQueueEntryCount(tf->QGen[Q_L2_CAPTURE]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_L2_CAPTURE])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received capture packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

    		testCommonRecycleLBDesc (tf, hd);
	    }

		while ((hd = t4GetEQoSPkt (tf)) != NULL)  {

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((*swInfo & T4_CMD_SWINFO0_TYPE_MASK) != T4_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			chan = *swInfo & T4_CMD_SWINFO0_ID_MASK;
            count++;
            pkt = (uint8_t *)hd->buffPtr;

			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t4eQoSPktTestInfo) / sizeof(ifPktTestInfo_t); j++)  {
				if (t4eQoSPktTestInfo[j].pktInfo.idx == chan)  {
					tinfo = &t4eQoSPktTestInfo[j + pkt[hd->buffLen-5]].pktInfo;
					break;
				}
			}

			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);

		}

        if(count >= expCount)
            break;

	}

    /* Clear and recycle the free descriptor */
  	while (Qmss_getQueueEntryCount (tf->QGen[Q_DPKT_RECYCLE]) > 0)  {
 	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_DPKT_RECYCLE])) & ~15);
        hd->tagInfo = 0;
        #ifdef NSS_GEN2
            Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
        #endif
 	    Qmss_queuePush (tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): EQoS Queue Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}

	return (0);

}

static paTxChksum_t t4pktChksumIp = {  /* The IP checksum command */

    	0,     	/* Start offset of IP header */
       20,     	/* Checksum length  */
       10,      /* Offset to checksum location RELATIVE TO THE START OF THE TCP/UDP HEADER */
    	0, 		/* Initial value is IP header checksum value */
    	1       /* computed value of 0 written as -0 */

	};

static paTxChksum_t t4pktChksumUdp = {  /* The UDP checksum command */

    	0,     	/* Start offset of UDP header */
    	0,     	/* Checksum length (UDP header + payload */
    	6,      /* Offset to checksum location RELATIVE TO THE START OF THE TCP/UDP HEADER */
    	0, 		/* Initial value is IPv4 pseudo header checksum value */
    	1       /* computed value of 0 written as -0 */

	};


static	paCmdNextRoute_t t4TxPktRoute =  {pa_NEXT_ROUTE_PARAM_PRESENT,   /* ctrlBitfield */
                                          pa_DEST_EMAC,	       /* Dest */
                                          0,                   /* pkyType: for SRIO only */
 								          0,				   /* Flow ID */
 								          0,                   /* queue */
 								          0,				   /* sw Info 0 */
                                          0,                   /* sw Info 1 */
 							              0,  				   /* Multi route */
                                          0};                  /* User-defined stats index */


static Cppi_HostDesc *t4FormEQoSEgressPacket (tFramework_t *tf, paTest_t *pat, int pktIdx, int *pCnt)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	paReturn_t     paret;
    uint8_t        *buf;
    paCmdInfo_t    cmdInfo[4];
    uint32_t       cmdStack[12];
    uint16_t       cmdStackSize = sizeof(cmdStack);
    int            ipOffset, udpOffset;
    int            numCmds = 0;
    uint16_t       mtuSize = t4eQoSEgressPktTestInfo[pktIdx].param[0];

	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = tf->QGen[Q_DPKT_RECYCLE];
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t4eQoSEgressPktTestInfo[pktIdx].pktInfo.pkt)), t4eQoSEgressPktTestInfo[pktIdx].pktInfo.pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t4eQoSEgressPktTestInfo[pktIdx].pktInfo.pktLen);

    /* Construct Tx command */
    udpOffset = PASAHO_LINFO_READ_L4_OFFSET(t4eQoSEgressPktTestInfo[pktIdx].pktInfo.info);
    ipOffset  = PASAHO_LINFO_READ_L3_OFFSET(t4eQoSEgressPktTestInfo[pktIdx].pktInfo.info);
    buf       = t4eQoSEgressPktTestInfo[pktIdx].pktInfo.pkt;

    *pCnt = 1;

    if (ipOffset && (mtuSize == 0))
    {
        t4pktChksumIp.startOffset = ipOffset;
        cmdInfo[numCmds].cmd = pa_CMD_TX_CHECKSUM;
        cmdInfo[numCmds++].params.chksum = t4pktChksumIp;

        if (udpOffset)
        {
            t4pktChksumUdp.startOffset = udpOffset;
            t4pktChksumUdp.lengthBytes = (buf[udpOffset + 4] << 8) + buf[udpOffset + 5];
            t4pktChksumUdp.initialSum  = utilGetIpPsudoChkSum(&buf[ipOffset], t4pktChksumUdp.lengthBytes, 0x11);
            cmdInfo[numCmds].cmd = pa_CMD_TX_CHECKSUM;
            cmdInfo[numCmds++].params.chksum = t4pktChksumUdp;
        }
    }
    else if (mtuSize && ipOffset)
    {
        int ipLen = (buf[ipOffset + 2] << 8) + buf[ipOffset + 3];
        int numFrags = 1;
        uint16_t fragSize, lastFragSize, payloadSize;

        lastFragSize = mtuSize - 20;
        payloadSize = ipLen - 20;
        fragSize = lastFragSize & 0xFFF8;
        while (payloadSize > lastFragSize)
        {
            numFrags += 1;
            payloadSize -= fragSize;
        }

        cmdInfo[numCmds].params.ipFrag.ipOffset = ipOffset;
        cmdInfo[numCmds].params.ipFrag.mtuSize =  mtuSize;
        cmdInfo[numCmds++].cmd = pa_CMD_IP_FRAGMENT;
        *pCnt = numFrags;

        if (numFrags > 1)
        {
            paTestL4ExpectedStats.classify1.nTxIpFrag += numFrags;
        }

    }

    t4TxPktRoute.pktType_emacCtrl =  t4eQoSEgressPktTestInfo[pktIdx].emacPort;
    cmdInfo[numCmds].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[numCmds++].params.route = t4TxPktRoute;

    paret = Pa_formatTxCmd ( numCmds,
                           	 cmdInfo,
                           	 0,
                           	 (Ptr)cmdStack,    /* Command buffer       */
                           	 &cmdStackSize);   /* Command size         */

    if (paret != pa_OK)  {
   	    System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
 		Qmss_queuePush (tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
   	    return (NULL);
    }

    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)cmdStack, cmdStackSize);

  	return (hd);
}

/* Search the EQoS queue for received packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t4ReceiveEQoSEgressPkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	int               i;
    int               count = 0;

    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (1000);
		while ((hd = t4GetEQoSPkt (tf)) != NULL)  {
            count++;

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);
		}

        if(count >= expCount)
            break;

	}

    /* Clear and recycle the free descriptor */
  	while (Qmss_getQueueEntryCount (tf->QGen[Q_DPKT_RECYCLE]) > 0)  {
 	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_DPKT_RECYCLE])) & ~15);
        //hd->tagInfo = 0;
 	    Qmss_queuePush (tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): EQoS Queue Egress Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}

	return (0);

}

static Cppi_HostDesc *t4FormIfPacket (tFramework_t *tf, paTest_t *pat, int pktIdx)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
    Cppi_DescTag   tag;

    memset(&tag, 0, sizeof(tag));

	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}

	/* Setup the return for the descriptor */
    /* Need to reset the source tag */
  	q.qMgr = 0;
  	q.qNum = tf->QGen[Q_DPKT_RECYCLE];
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);

    /* Set Input interface number */
    tag.srcTagHi = pktIdx + 1;
    Cppi_setTag(Cppi_DescType_HOST, (Cppi_Desc *)hd, &tag);
#ifdef NSS_GEN2
    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0x8);
#endif

  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t4IfPktInfo[pktIdx].pkt)), t4IfPktInfo[pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t4IfPktInfo[pktIdx].pktLen);

  	return (hd);
}


static Cppi_HostDesc *t4GetIfPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;

    for (index = Q_IF_BASE; index <= (T4_NUM_IF_QUEUES +  Q_IF_BASE); index++)
    {

	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): t4GetIfPkt: Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }

            return (hd);
	    }
    }

   return (NULL);

}

/* Search the interfcae queues for received packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t4ReceiveIfPkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t	     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
	uint32_t	      infoLen;
	int               i, j;
	unsigned int      chan;
    int               count = 0;

    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (5000);
    }

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (1000);
        /*
         * Look for L2 Capture packets: Examine and recycle
         */
	    while (Qmss_getQueueEntryCount(tf->QGen[Q_L2_CAPTURE]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_L2_CAPTURE])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received capture packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

    		testCommonRecycleLBDesc (tf, hd);
	    }

		while ((hd = t4GetIfPkt (tf)) != NULL)  {

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((*swInfo & T4_CMD_SWINFO0_TYPE_MASK) != T4_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			chan = *swInfo & T4_CMD_SWINFO0_ID_MASK;
            count++;

			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t4IfPktInfo) / sizeof(pktTestInfo_t); j++)  {
				if (t4IfPktInfo[j].idx == chan)  {
					tinfo = &t4IfPktInfo[j];
					break;
				}
			}

			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);

		}

        if(count >= expCount)
            break;

	}

    /* Clear and recycle the free descriptor */
  	while (Qmss_getQueueEntryCount (tf->QGen[Q_DPKT_RECYCLE]) > 0)  {
 	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_DPKT_RECYCLE])) & ~15);
        hd->tagInfo = 0;
        #ifdef NSS_GEN2
            Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
        #endif
 	    Qmss_queuePush (tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}


	if (i == 100)  {
		System_printf ("%s (%s:%d): If Queue Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}

	return (0);

}

static Cppi_HostDesc *t4FormExpPacket (tFramework_t *tf, paTest_t *pat, int pktIdx, int* fDrop)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;

	/* Pop a descriptor off of the free queue list, format the return queue, and attach the packet */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
		return (NULL);
	}

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = tf->QfreeDesc;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

  	/* Make sure there is no control info.  */
  	Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);

  	/* Attach the data and set the length */
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t4ePktInfo[pktIdx].pkt)), t4ePktInfo[pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t4ePktInfo[pktIdx].pktLen);

    if(t4ePktInfo[pktIdx].idx ==  T4_DROP_PKT_INDEX)
    {
        /*
         * Only rx padding error packet is dropped at this moment
         * future enhancements may be necessary.
         */
        t4UpdateUsrStats(&paTestExpectedUsrStats, t4UsrStatsTbl[T4_USR_STATS_ID_L2_PADDING_ERR].cntIndex);
        *fDrop = TRUE;
    }
    else
    {
        *fDrop = FALSE;
    }

  	return (hd);
}

static Cppi_HostDesc *t4GetExpPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;

    for (index = Q_PPPoE_FAIL; index <= Q_IP_FRAG; index++)
    {

	    if (Qmss_getQueueEntryCount(tf->QGen[index]) > 0)  {
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[index])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received packet from General Queue(%d)\n", tfName, __FILE__, __LINE__, index);
			    return (NULL);
		    }

            return (hd);
	    }

    }

   return (NULL);

}

/* Search the exception packet queue for received exceptionm packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t4ReceiveExpPkts (tFramework_t *tf, paTest_t *pat, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
    uint8_t            *pkt;
	uint32_t		      infoLen;
	int               i, j;
	unsigned int		      chan;
    int               count = 0;

    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (10000);
    }

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (1000);
		while ((hd = t4GetExpPkt (tf)) != NULL)  {

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((*swInfo & T4_CMD_SWINFO0_TYPE_MASK) != T4_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			chan = *swInfo & T4_CMD_SWINFO0_ID_MASK;
            count++;

            pkt = (uint8_t *)hd->buffPtr;

			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t4ePktInfo) / sizeof(pktTestInfo_t); j++)  {
				if (t4ePktInfo[j].idx == chan)  {
					tinfo = &t4ePktInfo[j + pkt[6]];
					break;
				}
			}

			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}

			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);

		}

        if(count >= expCount)
            break;

	}

	if (i == 100)  {
		System_printf ("%s (%s:%d): Exception Packet Processing Error - unable to receive all packets (rx = %d, exp = %d)\n", tfName, __FILE__, __LINE__, count, expCount);
        System_flush();
		return (-1);
	}

	return (0);

}

void clearPrevCmdAck(t4IpPaSetup_t *ipPaSetup_ptr, int size)
{
  int i;
  for (i = 0 ; i < size ; i ++)
  {
     ipPaSetup_ptr->acked = FALSE;
     ipPaSetup_ptr++;     
  }
}

#ifdef _TMS320C6X
#pragma DATA_SECTION(t4Encap, ".testPkts")
#endif
static t4TestEncap_t  t4Encap;

#ifdef __LINUX_USER_SPACE
void* paTestL3Routing (void *args)
{
 	void  *a0 = (void *)((paTestArgs_t *)args)->tf;
 	void  *a1 = (void *)((paTestArgs_t *)args)->pat;
#else
void paTestL3Routing (UArg a0, UArg a1)
{
#endif
 	Cppi_HostDesc  *hd[10];
 	paReturn_t      paret;
 	int				i, j, n, cnt, numPkts;
 	int  			cmdDest;
 	uint16_t		cmdSize;
 	paTestStatus_t  newStatus = PA_TEST_PASSED;

 	unsigned int	fifoData[T4_EXPTPKT_FIFO_SIZE];
 	pauFifo_t       fifo =  { 0, 0, T4_EXPTPKT_FIFO_SIZE, NULL };
    int             fDrop, dropCnt;

 	//volatile int mdebugWait = 1;

	/* Initialize the test state */
	fifo.data = fifoData;
 	memset (&t4Encap, 0, sizeof(t4Encap));
 	t4Encap.tf  = (tFramework_t *)a0;
 	t4Encap.pat = (paTest_t *)a1;

 	for (i = 0; i < T4_NUM_LOCAL_L4_HANDLES; i++)
 		t4Encap.l4Handles[i].state = T4_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T4_NUM_LOCAL_L3_HANDLES; i++)
 		t4Encap.l3Handles[i].state = T4_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T4_NUM_LOCAL_L2_HANDLES; i++)
 		t4Encap.l2Handles[i].state = T4_HANDLE_UNCONFIGURED;

    i = setupPktTestInfo(t4PktInfo, (sizeof(t4PktInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t4Encap.pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

	i = setupPktTestInfo(t4QoSPktInfo, (sizeof(t4QoSPktInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t4Encap.pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

	i = setupPktTestInfo(t4IfPktInfo, (sizeof(t4IfPktInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t4Encap.pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

    i = setupPktTestInfo(t4ePktInfo, (sizeof(t4ePktInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
        System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t4Encap.pat->testStatus = PA_TEST_FAILED;
        Task_exit();
    }

    i = setupIfPktTestInfo(t4eQoSPktTestInfo, (sizeof(t4eQoSPktTestInfo) / sizeof(ifPktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupIfPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t4Encap.pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

    i = setupIfPktTestInfo(t4eQoSEgressPktTestInfo, (sizeof(t4eQoSEgressPktTestInfo) / sizeof(ifPktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupIfPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t4Encap.pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }


    /* Runtime initial values */
    matchRoute[0].queue = (uint16_t) t4Encap.tf->QGen[Q_MATCH];
	matchRoute[0].flowId= t4Encap.tf->tfFlowNum[0];
    nfailRoute.queue    = (uint16_t) t4Encap.tf->QGen[Q_NFAIL];
	nfailRoute.flowId   = t4Encap.tf->tfFlowNum[0];
    nfailRoute1.queue   = (uint16_t) t4Encap.tf->QGen[Q_MATCH];
	nfailRoute1.flowId  = t4Encap.tf->tfFlowNum[0];
    cmdReply.queue      = (uint16_t) t4Encap.tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId     = t4Encap.tf->tfFlowNum[0];
    matchRoute2[0].queue = (uint16_t) t4Encap.tf->QGen[Q_QoS_BASE];
    nfailRoute2.queue = matchRoute2[4].queue = (uint16_t) t4Encap.tf->QGen[Q_IF_BASE];
    nfailRoute3.queue    =
    matchRoute2[3].queue = (uint16_t) t4Encap.tf->QGen[Q_L2_CAPTURE];
    matchRoute2[5].queue =
    matchRoute2[6].queue = (uint16_t) t4Encap.tf->QGen[Q_QoS_BASE];
    matchRoute2[0].flowId=
    matchRoute2[3].flowId=
    matchRoute2[4].flowId=
    matchRoute2[5].flowId=
    matchRoute2[6].flowId=
	nfailRoute2.flowId  =
	nfailRoute3.flowId  = t4Encap.tf->tfFlowNum[0];


    for ( i = 0; i < T4_NUM_EQoS_PORTS; i++)
    {
        t4EQoSModeCfg[i].flowBase  = t4Encap.tf->tfFlowNum[0];
        t4EQoSModeCfg[i].queueBase = (uint16_t) t4Encap.tf->QGen[Q_QoS_BASE];
    }

    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL4ExpectedStats, 0, sizeof(paTestL4ExpectedStats));
    memset (&paTestExpectedUsrStats, 0, sizeof(paTestExpectedUsrStats));

    /* TBD: Disable SCTP packet at this moment */
    #ifndef NSS_GEN2
    /*
     * Configure the CRC engine for SCTP CRC-32C checksum
     * The CRC-engine connected to PDSP2 should be configured since the SCTP is within the
     * inner IP paylaod which is parased and lookup at PDSP2
     */
 	cmdReply.replyId = T4_CMD_SWINFO0_CRC_CFG_ID;
 	cmdReply.queue = t4Encap.tf->QGen[Q_CMD_REPLY];

    #endif

    hd[0] = testCommonConfigCrcEngine(t4Encap.tf, 2, &t4CrcCfg,  t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QLinkedBuf3,
                                      &cmdReply, &cmdDest, &cmdSize, &paret);

    t4HandleError (&t4Encap, paret, hd[0], __LINE__); /* Will not return on error */

    #ifndef NSS_GEN2

 	Qmss_queuePush (t4Encap.tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[0], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	paTestL4ExpectedStats.classify1.nPackets += 1;

	if (testCommonWaitCmdReply (t4Encap.tf, t4Encap.pat, tfName, cmdReply.queue, T4_CMD_SWINFO0_CRC_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): testCommonConfigCrcEngine failed\n", tfName, __FILE__, __LINE__);
		newStatus = PA_TEST_FAILED;
 	}

 	if (newStatus == PA_TEST_FAILED)
 		t4Cleanup (&t4Encap, newStatus);  /* No return */


 	/* Recycle the command packet as well */
 	hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (t4Encap.tf->QGen[Q_CMD_RECYCLE])) & ~15);
 	if (hd[0] == NULL)  {
 		System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 		newStatus = PA_TEST_FAILED;
 	}

 	if (newStatus == PA_TEST_FAILED)
 		t4Cleanup (&t4Encap, newStatus);  /* No return */

 	testCommonRecycleLBDesc (t4Encap.tf, hd[0]);
    #endif

    /* Global Configuration */
	newStatus = t4GlobalConfiguration (&t4Encap, &t4GlobalCfg);
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */

    /* Allocate User-defined statistics */
    if(testCommonAllocUsrStats(t4Encap.tf, tfName, T4_NUM_USR_STATS, t4UsrStatsTbl, t4UsrStatsGroup1))
    {
        newStatus = PA_TEST_FAILED;
        t4Cleanup (&t4Encap, newStatus);  /* No return */
    }

    /* Global Configuration 2 */
    t4PktCtrlCfg.rxPaddingErrStatsIndex = t4UsrStatsTbl[T4_USR_STATS_ID_L2_PADDING_ERR].cntIndex;
    t4PktCtrlCfg.txPaddingStatsIndex    = t4UsrStatsTbl[T4_USR_STATS_ID_L2_TX_PADDING].cntIndex;
	newStatus = t4GlobalConfiguration (&t4Encap, &t4GlobalCfg2);
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */

    /* Exception Route Configurations */
	newStatus = t4ExceptionRoutes (&t4Encap);
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */

    /* Usr Stats configuration */
    newStatus = testCommonUsrStatsSetup (t4Encap.tf, t4Encap.pat, tfName, sizeof(t4UsrStatsSetup)/sizeof(pauUsrStatsSetup_t), t4UsrStatsSetup,
                                         T4_CMD_SWINFO0_USR_STATS_CFG_ID, t4Encap.tf->QLinkedBuf3, t4Encap.tf->QGen[Q_CMD_RECYCLE],
                                         t4Encap.tf->QGen[Q_CMD_REPLY], FALSE);
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */


 	/* Add two mac entries into the table. All the test packets will match one of these */
 	for (i = 0; i < T4_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T4_CMD_SWINFO0_ADD_MAC_ID + i;  /* T4_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue = t4Encap.tf->QGen[Q_CMD_REPLY];

        if (i < T4_FIRST_MAC2_HANDLE)
        {
 		    hd[i] = testCommonAddMac (t4Encap.tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t4EthInfo[i], &matchRoute[2], &nfailRoute,
 	    	                          &t4Encap.l2Handles[i].paHandle, t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QLinkedBuf1,
 	                                  &cmdReply, &cmdDest, &cmdSize, &paret);
        }
        else
        {
 		    hd[i] = testCommonAddMac2 (t4Encap.tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t4EthInfo[i], &matchRoute2[3], &nfailRoute2,
 	    	                           NULL, &t4Encap.l2Handles[i].paHandle, t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QLinkedBuf1,
 	                                   &cmdReply, &cmdDest, &cmdSize, 0, &paret);

        }

 	    t4HandleError (&t4Encap, paret, hd[i], __LINE__); /* Will not return on error */

 	}


 	/* Send the commands to PA. Each will result in 1 packet in classify1 */
 	for (i = 0; i < T4_NUM_LOCAL_L2_HANDLES; i++)  {
 		Qmss_queuePush (t4Encap.tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		t4Encap.l2Handles[i].state = T4_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
 	}

 	/* Wait for the responses. They will be received in the order in which they were sent */
 	for (i = 0; i < T4_NUM_LOCAL_L2_HANDLES; i++)  {

		if (testCommonWaitCmdReply (t4Encap.tf, t4Encap.pat, tfName, cmdReply.queue, T4_CMD_SWINFO0_ADD_MAC_ID + i, __LINE__)) {
			System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
			newStatus = PA_TEST_FAILED;
            break;
 		}

 		/* Recycle the command packet as well */
 		hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (t4Encap.tf->QGen[Q_CMD_RECYCLE])) & ~15);
 		if (hd[0] == NULL)  {
 			System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 			newStatus = PA_TEST_FAILED;
            break;
 		}
 		testCommonRecycleLBDesc (t4Encap.tf, hd[0]);
 	}

 	if (newStatus == PA_TEST_FAILED)
 		t4Cleanup (&t4Encap, newStatus);  /* No return */

    /* Add virtual links */
 	for (i = 0; i < T4_NUM_LOCAL_VL_HANDLES; i++)  {
        /* Create the virtual link */
        if((paret = Pa_addVirtualLink(t4Encap.tf->passHandle, /* PA Handle */
                            &(t4Encap.vlHandles[i].paHandle), /* Virtual link handle table*/
                            pa_VIRTUAL_LNK_TYPE_OUTER_IP      /* Virtual link type is outer ip */
                            )) != pa_OK)
        {
            System_printf ("%s: (%s:%d): Command Pa_addVirtualLink failed with error code %d\n", tfName, __FILE__, __LINE__, paret);
            newStatus = PA_TEST_FAILED;
        }
        else
        {
            t4Encap.vlHandles[i].state = T4_HANDLE_ACTIVE;
        }
 	}

 	if (newStatus == PA_TEST_FAILED)
 		t4Cleanup (&t4Encap, newStatus);  /* No return */

	/* Burst in the next set of command packets. These packets are used as for the outer header in nested
	 * packets. It will include several configurations that are invalid. These are detected and
	 * verified by the swinfo0 ID */
	n = sizeof (t4OuterIpInfo) / sizeof (t4IpPaSetup_t);
  /* Clear the previous command acks */
  clearPrevCmdAck(t4OuterIpInfo, n);
	newStatus = t4OpenIp (&t4Encap, t4OuterIpInfo, n, t4Encap.l2Handles, "Outer IP headers");
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */

	/* Repeat for Inner IP packets (which are linked to outer IP packets) */
	n = sizeof (t4InnerIpInfo) / sizeof (t4IpPaSetup_t);
  /* Clear the previous command acks */
  clearPrevCmdAck(t4InnerIpInfo, n); 
	newStatus = t4OpenIp (&t4Encap, t4InnerIpInfo, n, t4Encap.l3Handles, "Inner IP headers");
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */

	/* TBD: One more time for tripple layered IP headers */
	n = sizeof (t4InnerInnerIpInfo) / sizeof (t4IpPaSetup_t);
  /* Clear the previous command acks */
  clearPrevCmdAck(t4InnerInnerIpInfo, n);   
	newStatus = t4OpenIp (&t4Encap, t4InnerInnerIpInfo, n, t4Encap.l3Handles, "Inner IP headers");
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */

#ifndef NSS_GEN2
#ifndef __LINUX_USER_SPACE
  /* Check the replace API for PA */
	n = sizeof (t4ReplaceOuterIpInfo) / sizeof (t4RepIpPaSetup_t);
	newStatus = t4ReplaceIp (&t4Encap, t4ReplaceOuterIpInfo, t4OuterIpInfo, n, "Replace Outer IP headers");
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */
	System_flush();
#endif
#endif

    /* NAT-T Configuration */
	newStatus = t4NatTConfiguration (&t4Encap, 1);
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t4Encap.tf, t4Encap.pat, tfName, &paTestL4ExpectedStats, t4Encap.tf->QLinkedBuf1,
	                       t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QGen[Q_CMD_REPLY], TRUE);

	if (newStatus == PA_TEST_FAILED)  {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t4Cleanup (&t4Encap, newStatus);  /* No return */
	}

	/* Fire in each of the data packets multiple times */
	n = sizeof (t4PktInfo) / sizeof (pktTestInfo_t);
	for (j = 0; j < T4_NUM_PACKET_ITERATIONS; j++)  {
		for (i = 0; i < n; i++)  {
			hd[0] = t4FormPktDataDescr (&t4Encap, i);
			if (hd[0] == NULL)  {
				System_printf ("%s (%s:%d): Failed to get free descriptor\n", tfName, __FILE__, __LINE__);
				t4Cleanup (&t4Encap, PA_TEST_FAILED);
			}

			Qmss_queuePush (t4Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], t4PktInfo[i].pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
			testCommonIncStats (t4PktInfo[i].statsMap, &paTestL4ExpectedStats);

			if ((t4PktInfo[i].idx & T4_PACKET_DEST_MASK) == T4_PACKET_L3_MATCH_VALID)  {
				if (commonFifoPushElement (&fifo, (unsigned int)i) < 0)  {
					System_printf ("%s (%s:%d): Test failed - fifo is full\n", tfName, __FILE__, __LINE__);
					t4Cleanup (&t4Encap, PA_TEST_FAILED);
				}
			}
		}

		if (t4RxPkts (&t4Encap, &fifo))
			t4Cleanup (&t4Encap, PA_TEST_FAILED);
	}

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t4Encap.tf, t4Encap.pat, tfName, &paTestL4ExpectedStats, t4Encap.tf->QLinkedBuf1,
	                                   t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t4Cleanup (&t4Encap, newStatus);
    }

    /* QoS packet testing */
	/* Run packets through the system. the complete set of packets is run through multiple times. */
	n = sizeof (t4QoSPktInfo) / sizeof (pktTestInfo_t);
	for (j = 0; j < T4_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < n; i++ )  {

			hd[0] = t4FormQoSPacket (t4Encap.tf, t4Encap.pat, i);

			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T4 QoS packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();
                t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
                break;
			}

			/* Increment any expected stats */
			testCommonIncStats (t4QoSPktInfo[i].statsMap, &paTestL4ExpectedStats);

			Qmss_queuePush (t4Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
            utilCycleDelay (5000);
		}

		if (t4ReceiveQoSPkts (t4Encap.tf, t4Encap.pat, n))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t4ReceiveQoSPktstimeout\n", tfName, __FILE__, __LINE__);
            System_flush();
            t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
			break;
        }
	}

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t4Encap.tf, t4Encap.pat, tfName, &paTestL4ExpectedStats, t4Encap.tf->QLinkedBuf1,
	                                   t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t4Cleanup (&t4Encap, newStatus);
    }

    /* If packet testing */
	/* Run packets through the system. the complete set of packets is run through multiple times. */
	n = sizeof (t4IfPktInfo) / sizeof (pktTestInfo_t);
	for (j = 0; j < T4_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < n; i++ )  {

			hd[0] = t4FormIfPacket (t4Encap.tf, t4Encap.pat, i);

			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T4 If packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();
                t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
                break;
			}

			/* Increment any expected stats */
			testCommonIncStats (t4IfPktInfo[i].statsMap, &paTestL4ExpectedStats);

			Qmss_queuePush (t4Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);

		}

		if (t4ReceiveIfPkts (t4Encap.tf, t4Encap.pat, n))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t4ReceiveIfPktstimeout\n", tfName, __FILE__, __LINE__);
            System_flush();
            t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
			break;
        }
	}

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t4Encap.tf, t4Encap.pat, tfName, &paTestL4ExpectedStats, t4Encap.tf->QLinkedBuf1,
	                                   t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t4Cleanup (&t4Encap, newStatus);
    }

    /* Exception packet testing */
	/* Run packets through the system. the complete set of packets is run through multiple times. */
	n = sizeof (t4ePktInfo) / sizeof (pktTestInfo_t);
	for (j = 0; j < T4_NUM_PACKET_ITERATIONS; j++)   {
        dropCnt = 0;
		for (i = 0; i < n; i++ )  {

			hd[0] = t4FormExpPacket (t4Encap.tf, t4Encap.pat, i, &fDrop);

			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T4 Exception packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();
                t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
                break;
			}

			/* Increment any expected stats */
			testCommonIncStats (t4ePktInfo[i].statsMap, &paTestL4ExpectedStats);

			Qmss_queuePush (t4Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);

            if(fDrop)dropCnt++;
		}

		if (t4ReceiveExpPkts (t4Encap.tf, t4Encap.pat, n - dropCnt))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t4ReceiveExpPk tstimeout\n", tfName, __FILE__, __LINE__);
            System_flush();
            t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
			break;
        }
	}


    /* NAT-T Configuration */
	newStatus = t4NatTConfiguration (&t4Encap, 0);
	if (newStatus == PA_TEST_FAILED)
		t4Cleanup (&t4Encap, newStatus);  /* No return */

	/* Verify and clear the Usr stats */
	newStatus =  testCommonCheckUsrStatsList (t4Encap.tf, t4Encap.pat, tfName, &paTestExpectedUsrStats, T4_NUM_64B_USR_STATS, T4_NUM_USR_STATS, t4UsrStatsTbl,
                                              t4Encap.tf->QLinkedBuf1, t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckUsrStats Failed\n", tfName, __FILE__, __LINE__);
		t4Cleanup (&t4Encap, newStatus);
    }

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t4Encap.tf, t4Encap.pat, tfName, &paTestL4ExpectedStats, t4Encap.tf->QLinkedBuf1,
	                       t4Encap.tf->QGen[Q_CMD_RECYCLE], t4Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t4Cleanup (&t4Encap, newStatus);
    }

    /* Clear all Usr Stats Link */
    newStatus = testCommonUsrStatsConfigReset (t4Encap.tf, t4Encap.pat, tfName, sizeof(t4UsrStatsSetup)/sizeof(pauUsrStatsSetup_t), t4UsrStatsSetup,
                                               T4_CMD_SWINFO0_USR_STATS_CFG_ID, t4Encap.tf->QLinkedBuf3, t4Encap.tf->QGen[Q_CMD_RECYCLE],
                                               t4Encap.tf->QGen[Q_CMD_REPLY]);

	if (newStatus == PA_TEST_FAILED)
    {
	    System_printf ("%s (%s:%d): testCommonUsrStatsConfigReset Failed\n", tfName, __FILE__, __LINE__);
	    t4Cleanup (&t4Encap, newStatus);
    }

    /* Free User-defined statistics */
    if(testCommonFreeUsrStats(t4Encap.tf, tfName, T4_NUM_USR_STATS, t4UsrStatsTbl))
    {
	    System_printf ("%s (%s:%d): testCommonFreeUsrStats Failed\n", tfName, __FILE__, __LINE__);
	    t4Cleanup (&t4Encap, PA_TEST_FAILED);
    }

    /* EQoS packet testing */
    /* Set EQoS Mode Configurations */
	newStatus = t4ConfigEQoSModes (&t4Encap, FALSE);
	if (newStatus == PA_TEST_FAILED)
 		t4Cleanup (&t4Encap, newStatus);

	/* Run packets through the system. the complete set of packets is run through multiple times. */
	n = sizeof (t4eQoSPktTestInfo) / sizeof (ifPktTestInfo_t);
	for (j = 0; j < T4_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < n; i++ )  {

			hd[0] = t4FormEQoSPacket (t4Encap.tf, t4Encap.pat, i);

			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T4 EQoS test packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();
                t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
                break;
			}

			/* Increment any expected stats */
			testCommonIncStats (t4eQoSPktTestInfo[i].pktInfo.statsMap, &paTestL4ExpectedStats);

			Qmss_queuePush (t4Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);

		}

		if (t4ReceiveEQoSPkts (t4Encap.tf, t4Encap.pat, n))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t4ReceiveEQoSpkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();
            t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
			break;
        }
	}

    /* Enable the EQoS mode for Egress traffic */
    newStatus = t4EQoSModeGlobalControl(&t4Encap, TRUE);
	if (newStatus == PA_TEST_FAILED)
 		t4Cleanup (&t4Encap, newStatus);

	/* Run Egress packets through the system. the complete set of packets is run through multiple times. */
	n = sizeof (t4eQoSEgressPktTestInfo) / sizeof (ifPktTestInfo_t);
    numPkts = 0;
	for (j = 0; j < T4_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < n; i++ )  {

			hd[0] = t4FormEQoSEgressPacket (t4Encap.tf, t4Encap.pat, i, &cnt);

			if (hd[0] == NULL)  {
			    System_printf ("%s (%s:%d): T4 EQoS egress test packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();
                t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
                break;
			}

			Qmss_queuePush (t4Encap.tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);

            numPkts += cnt;
		}

		if (t4ReceiveEQoSEgressPkts (t4Encap.tf, t4Encap.pat, numPkts))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t4ReceiveEQoSEgressPkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();
            t4Cleanup (&t4Encap, PA_TEST_FAILED);  /* no return */
			break;
        }
	}

    /* Disable the EQoS mode for Egress traffic */
    newStatus = t4EQoSModeGlobalControl(&t4Encap, FALSE);
	if (newStatus == PA_TEST_FAILED)
 		t4Cleanup (&t4Encap, newStatus);


    /* Clear EQoS Mode Configurations */
	newStatus = t4ConfigEQoSModes (&t4Encap, TRUE);
	if (newStatus == PA_TEST_FAILED)
 		t4Cleanup (&t4Encap, newStatus);

	/* No return from cleanup */
	t4Cleanup (&t4Encap, newStatus);
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif

}






