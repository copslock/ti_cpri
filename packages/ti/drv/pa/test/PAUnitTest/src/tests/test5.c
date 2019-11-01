/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/ 
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

/* This test suite verifies the following PASS features:
 *  - Add/Delete LUT2 (UDP/TCP/GTP-U) entries 
 *  - MAC/IP/UDP(TCP)/GTP-U) classification and routing
 *  - Exception route configuration
 *  - User-defined Statistics operation
 *  - IPSEC NAT-T operation
 *  - Command set operation for MAC header replacement
 *  - LUT2 entry replacement with atomic queue diversion
 *
 * This test invokes the following LLD APIs:
 *  - Pa_addMac
 *  - Pa_addIp
 *  - Pa_addPort
 *  - Pa_delL4Handle
 *  - Pa_delHandle
 *  - Pa_forwardResult
 *  - Pa_control
 *  - Pa_configExceptionRoute
 *  - Pa_configCmdSet
 *  - Pa_configUsrStats
 *  - Pa_formatTxRoute
 *  - Pa_getHandleRefCount
 *  - Pa_requestStats
 *  - Pa_formatStatsReply
 *  - Pa_requestUsrStats
 *  - Pa_formatUsrStatsReply
 *
 * This test verifies PDSP firmware for routing L4 (TCP/UDP) and L5 (GTP-U) packets. 
 * This test has the following sub-tests
 *   TCP/UDP/UDP-lite:
 *   - Test the ability to add 16-bit ports 
 * 	 - Test the ability to delete 16-bit ports
 *   - Test the ability to verify IP header and TCP/UDP checksum
 *   - Test the ability to perform LUT2 entry update with atomic queue diversion
 *   - Test the LLD for the ability to maintain the reference counts
 *   - Test the firmware for the ability to route the TCP/UDP/UDP-lite packets with the specified destination port number
 *   - Test the firmware for the ability to replace variable-length MAC header
 *   GTP-U:
 *   - Test the ability to configure exception routes
 *   - Test the ability to configure command sets
 *   - Test the ability to add 32-bit ports
 *   - Test the ability to delete 32-bit ports
 *   - Test the firmware for the ability to route the GTPU traffic with the specified TEID (Tunnel ID)
 *   - Test the firmware for the ability to detect and route the GTP-U packets with other message types and error conditions
 *   - Test the firmware for the ability to handle the PDU number in the GTPU extension header
 *   - Test the firmware for the ability to remove the parsed header and trail and insert bytes after lookup
 *   User-defined Statistics:
 *   - Test the ability to perform system-level configurations such as the number of command sets and the number of user-defined counters
 *   - Test the ability to configure user-defined statistics (link and type)
 *   - Test the ability to query and format the user-defined statistics
 *   - Test the firmware for the ability to increment the specified user-defined statistics chain upon LUT1 or LUT2 match
 *   Atomic Queue Diversion
 *   - Test the firmware for the ability to forward the queue diversion request to the specified destination queue
 *   - Test the firmware for the ability to hold until the queue diversion operation is complete
 *   NAT-T:
 *   - Test the ability to configure and re-configure IPSEC NAT-T detector
 *   - The the firmware for the ability to ignore IPSEC NAT-T packet if the detector is disabled
 *   - The the firmware for the ability to detect and deliver IPSEC NAT-T packets
 */

 static char *tfName = "paTestL4Routing";
 
 #define T5_NUM_PACKET_ITERATIONS	1  		/* Number of times the packet stream is passed through */
 #define T5_FIRST_DEST_PORT			2100	/* First TCP/UDP destination port used */
 #define T5_FIRST_GTPU_TEID	   0x01000000	/* First GTPU Tunnel ID used */
 #define T5_FIRST_GTPU_TEID2   0x00010000	/* First GTPU Tunnel ID with link used */
 
 /* There is a one to one mapping from destination port number to L3 Handle and L4 Handle */
 #define T5_L2HDL_IDX_FROM_PORT(x)		(((x)-T5_FIRST_DEST_PORT) % T5_NUM_LOCAL_L2_HANDLES)
 #define T5_L3HDL_IDX_FROM_PORT(x)		(((x)-T5_FIRST_DEST_PORT) % T5_NUM_LOCAL_L3_HANDLES)
 #define T5_L3HDL_IDX_FROM_TEID(x, base) (((x)-(base)) % T5_NUM_LOCAL_L3_HANDLES)
 #define T5_L4HDL_IDX_FROM_PORT(x)		((x)-T5_FIRST_DEST_PORT)
 
 /* General purpose queue usage */
 #define Q_CMD_RECYCLE		  0		/* Command descriptors/buffers recycled here after sent to PA */
 #define Q_CMD_REPLY  		  1		/* Replies from PA routed here */
 #define Q_MATCH		  	  2		/* Packets from PA which match a lookup criteria */
 #define Q_NFAIL		      3		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
 #define Q_PARSE_ERR		  4		/* Packets which resulted in a parse error */
 #define Q_DPKT_RECYCLE		  5		/* Data packet recycle queue */
#ifdef  SIMULATOR_SUPPORT
 /* Simulator does not support Queue diversion operation */
 #define Q_MATCH2		      2		/* Packest from PA which match a loop criteria after queue diversion (should be 6) */
#else
 #define Q_MATCH2		      6		/* Packest from PA which match a loop criteria after queue diversion (should be 6) */
#endif 
 #define Q_GTPU_PING_REQ      7     /* GTPU Ping Request queue (message type 1) */ 
 #define Q_GTPU_PING_RESP     8     /* GTPU Ping Response queue (message type 2) */ 
 #define Q_GTPU_ERR_IND       9     /* GTPU Error Indication queue (message type 26) */ 
 #define Q_GTPU_HDR_NOTIFY   10     /* GTPU Supported Header Notification queue (message type 31) */ 
 #define Q_GTPU_END_MARKER   11     /* GTPU End Marker queue (message type 254) */  
 #define Q_GTPU_FAIL         12     /* GTPU Parsing or other errors */
 #define Q_IP_FRAG           13     /* IP Fragmentation */
 #define Q_NAT_T_KEEPALIVE   14     /* NAT-T Keep Alive packet queue */
 #define Q_NAT_T_CTRL        15     /* NAT-T Control packet queue */
 #define Q_NAT_T_DATA        16     /* NAT-T Data packet queue */
 #define Q_NAT_T_FAIL        17     /* NAT-T Error packet queue */ 
 #define Q_GTPU_MATCH_FAIL   18     /* GTPU Match Fail */
 #define Q_EROUTE_FIRST      Q_GTPU_PING_REQ
 #define Q_EROUTE_LAST       Q_GTPU_MATCH_FAIL
 #define Q_BOUNCE_DDR        21     /* Queue Bounce DDR queue */
 #define Q_BOUNCE_MSMC       22     /* Queue Bounce MSMC queue */
 
 
#ifdef  SIMULATOR_SUPPORT
 /* Simulator does not support Queue diversion operation */
 #define Q_DIVERT_MONITORING    TF_PA_QUEUE_POST /* Queue Diversion MONITORING queue */
#else
 #define Q_DIVERT_MONITORING	(TF_FIRST_GEN_QUEUE + 30)	/* Queue Diversion MONITORING queue */
#endif 
 
/* GTPU Packet Index */ 
#define  T5_GTPU_PING_REQ_PKT_INDEX      0
#define  T5_GTPU_PING_RESP_PKT_INDEX     1
#define  T5_GTPU_ERR_IND_PKT_INDEX       2
#define  T5_GTPU_HDR_NOTIFY_PKT_INDEX    3
#define  T5_GTPU_END_MARKER_PKT_INDEX    4
#define  T5_GTPU_FAIL_PKT_INDEX          5
#define  T5_IP_FRAG_PKT_INDEX            6
#define  T5_GTPU_MATCH_FAIL_PKT_INDEX    7
#define  T5_GTPU_FIRST_PKT_INDEX        20


/* NAT_T packet index */
#define  T5_NAT_T_KEEPALIVE_PKT_INDEX   10
#define  T5_NAT_T_CTRL_PKT_INDEX        11
#define  T5_NAT_T_DATA_PKT_INDEX        12
#define  T5_NAT_T_FAIL_PKT_INDEX        19
                                         
#define  T5_MAX_NAT_T_CHAN               (T5_NAT_T_FAIL_PKT_INDEX + 1)
 
#include "test5pkts.h"
 
/* The number of PA L2 and L3 handles maintained by this test */
#define T5_NUM_LOCAL_L2_HANDLES   			(sizeof(t5EthSetup)/sizeof(t5EthSetup_t))
#define T5_NUM_LOCAL_L3_HANDLES				(sizeof(t5IpSetup)/sizeof(t5IpSetup_t))
#define T5_NUM_LOCAL_SPI_L3_HANDLES         (sizeof(t5SpiIpSetup)/sizeof(t5SpiIpSetup_t))
#define T5_NUM_LOCAL_L4_HANDLES	  			100
#define T5_NUM_LOCAL_L5_HANDLES	  			100
#define T5_NUM_GEN_CONFIGS                  10     /* Maxmium number of general configuration commands */
#define T5_MAX_GTPU_CHAN                    (T5_NUM_LOCAL_L5_HANDLES + T5_GTPU_FIRST_PKT_INDEX)
 
 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T5_HANDLE_UNCONFIGURED = 0,
	T5_HANDLE_PENDING_ACK,
	T5_HANDLE_ACTIVE,
	T5_HANDLE_DISABLED
};

#define T5_CMD_SWINFO0_TYPE_MASK	0xffff0000
#define T5_CMD_SWINFO0_ID_MASK		0x0000ffff

/* SWInfo values on command replies */
#define T5_CMD_SWINFO0_ADD_MAC_ID  		0x51110000
#define T5_CMD_SWINFO0_ADD_IP_ID   		0x51120000
#define T5_CMD_SWINFO0_ADD_PORT_ID	   	0x51130000
#define T5_CMD_SWINFO0_ADD_TEID_ID	   	0x51140000
#define T5_CMD_SWINFO0_EROUTE_CFG_ID   	0x51150000
#define T5_CMD_SWINFO0_CMDSET_CFG_ID   	0x51160000
#define T5_CMD_SWINFO0_GLOBAL_CFG_ID   	0x51170000  
#define T5_CMD_SWINFO0_USR_STATS_CFG_ID 0x51180000  
#define T5_CMD_SWINFO0_NAT_T_CFG_ID   	0x51190000  
#define T5_CMD_SWINFO0_GTPU_CFG_ID   	0x511a0000  
#define T5_CMD_SWINFO0_ADD_SPI_IP_ID   	0x511b0000

#define T5_CMD_SWINFO0_DEL_MAC_ID		0x50010000
#define T5_CMD_SWINFO0_DEL_IP_ID		0x50020000
#define T5_CMD_SWINFO0_DEL_PORT_ID		0x50030000
#define T5_CMD_SWINFO0_DEL_TEID_ID	   	0x50040000
#define T5_CMD_SWINFO0_PKT_ID		    0x50050000
#define T5_CMD_SWINFO0_GLOB_PKT_ID 	    0x50060000
#define T5_CMD_SWINFO0_DEL_SPI_IP_ID    0x50070000


 typedef struct t5Handles_s  {
 
  	paHandleL2L3_t  paHandle;     /* The L3 handle returned by the PA LLD */

 	unsigned int	state;		  /* T4_HANDLE_UNCONFIGURED = handle not configured
 						           * T4_HANDLE_PENDING_ACK = handle configured and sent to pa
 						           * T4_HANDLE_ACTIVE = handle creation acknowledged by pa
 						           * T4_HANDLE_DISABLED = handle was created then released */
    unsigned int    linkCnt;                               
 	
 } t5Handles_t;
 
 typedef struct t5HandlesL4_s  {
 	
 	paHandleL4_t   paHandle;
 	
 	unsigned int   state;

 } t5HandlesL4_t;
 
 /* A grouping of run time created grouped together to make cleanup easier on
  * error exit */

 typedef struct t5TestEncap_s  {
 	tFramework_t  *tf;
 	paTest_t      *pat;
 	
 	/* There is one base packet for each L3 table entry */
 	Cppi_HostDesc  *hd[T5_NUM_LOCAL_L3_HANDLES];
 	
 	/* The command to the modify PDSP to add the TCP/UDP checksum and route to PA receive */
 	uint32_t   cmdStack[T5_NUM_LOCAL_L3_HANDLES][(sizeof(pasahoNextRoute_t) + sizeof(pasahoComChkCrc_t)) / sizeof (uint32_t)];
 	
 	/* The +1 is a place holder handle used to pass known invalid configuration into the PA LLD */
 	t5Handles_t    l2Handles[T5_NUM_LOCAL_L2_HANDLES];		/* MAC handles */
 	t5Handles_t    l3Handles[T5_NUM_LOCAL_L3_HANDLES];		/* IP handles  */
 	t5Handles_t    l3SpiHandles[T5_NUM_LOCAL_SPI_L3_HANDLES];		/* SPI IP handles  */  
 	t5HandlesL4_t  l4Handles[T5_NUM_LOCAL_L4_HANDLES+1];	/* UDP/TCP handles */
 	t5HandlesL4_t  l5Handles[T5_NUM_LOCAL_L5_HANDLES+1];	/* GTPU handles */
    unsigned int   genCmdAck[T5_NUM_GEN_CONFIGS];           /* General configurations */
    
 	
 } t5TestEncap_t;
 
static paSysStats_t paTestL4ExpectedStats;    /* Expected stats results */

/* Global configurations */

#define T5_NUM_64B_USR_STATS            64

static paProtocolLimit_t     t5ProtocolLimit = 
        {
            3,      /* Number of VLANs */
            2,      /* Number of IPs */
            3       /* Number of GREs */
        };
        
static paCmdSetConfig_t      t5CmdSetCfg = 
        {
            32      /* Number of command sets */    
        };    
        
static paQueueDivertConfig_t t5QueueDivertCfg =
        {
            Q_DIVERT_MONITORING,    /* Monitoring Queue */
            0                       /* flow Id */
        };    
        
static paUsrStatsConfig_t   t5UsrStatsCfg =
       {
            pa_USR_STATS_MAX_COUNTERS - T5_NUM_64B_USR_STATS,   /* Number of user stats (448)*/
            T5_NUM_64B_USR_STATS                                /* Number of 64-bit user stats */
       };   
       
static  paIpsecNatTConfig_t  t5NatTCfg = 
        {
            pa_IPSEC_NAT_T_CTRL_ENABLE,    /* ctrlBitMap */
            4500                           /* UDP port number */
            
        };
        
static paPacketControlConfig_t  t5PktCtrlCfg = 
        {
            pa_PKT_CTRL_HDR_VERIFY_PPPoE | pa_PKT_CTRL_HDR_VERIFY_IP,  /* ctrlBitMap */
            0,                                                         /* rxPaddingErrStatsIndex */
            0                                                          /* txPaddingStatsIndex */
        };

static	paQueueBounceConfig_t t5QueueBounceCfg =
	        {
	            1,      /* Enable */
                Q_BOUNCE_DDR  + TF_FIRST_GEN_QUEUE,             /* ddrQueueId */
                Q_BOUNCE_MSMC + TF_FIRST_GEN_QUEUE,             /* msmcQueueId */
                TF_PA_TX_QUEUE_BASE,                            /* hwQueueBegin */
                TF_PA_TX_QUEUE_BASE + NSS_NUM_TX_QUEUES - 1,    /* hwQueueEnd */
                {
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Command Return */
                    pa_QUEUE_BOUNCE_OP_DDR,     /* QoS mode */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Capture Capture */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* IP Reassembly-assisted packets */
                    pa_QUEUE_BOUNCE_OP_MSMC     /* All traffics */
                }
	        };

static  paSysConfig_t  t5GlobalCfg = 
        {
            &t5ProtocolLimit,
            NULL,                   /* pOutIpReassmConfig */
            NULL,                   /* pInIpReassmConfig */
            &t5CmdSetCfg,
            &t5UsrStatsCfg,         /* pUsrStatsConfig */
            &t5QueueDivertCfg,
            &t5PktCtrlCfg,          /* pPktControl */
            &t5QueueBounceCfg       /* pQueueBounceConfig */
        };

/* Command set related definitions */
#define T5_CMDSET_1_INDEX         2
#define T5_CMDSET_1_NUM_CMDS      2
#define T5_CMDSET_2_INDEX         4
#define T5_CMDSET_2_NUM_CMDS      3
#define T5_CMDSET_3_INDEX         10
#define T5_CMDSET_3_NUM_CMDS      2
 
/*
 * Command Set to test MAC router
 *
 */         
 
static uint8_t t5MacRouteHdr[] = 
{
    0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
    0x61, 0x62, 0x63, 0x64, 0x65, 0x66,
    0x08, 0x00
}; 
                                    
static  paPatchInfo_t t5PatchCmd1 = 
        {
            pa_PATCH_OP_MAC_HDR,    /* ctrlBitfield */  
            sizeof(t5MacRouteHdr),  /* nPatchBytes */
            sizeof(t5MacRouteHdr),  /* totalPatchSize */
            0,                      /* offset */
            t5MacRouteHdr           /* Pointer to the patch data */
        };
        
static  paCmdVerifyPktErr_t t5VerifyPktErrCmd1 = 
        {
            pa_PKT_ERR_L4_CHECKSUM, /* ctrlBitfield */  
            pa_DEST_HOST,           /* dest */
            0,                      /* flowId */
            0,                      /* queue */
            0                       /* swInfo0 */
        };
        
      
static paCmdInfo_t t5CmdSet1[] =
{
    
    /* Command 0: Insert Bytes */
    {
        pa_CMD_PATCH_DATA,
        {
            {
                0,                            /* ctrlBitfield */
                0,                            /* nPatchBytes */
                0,                            /* totalPatchSize */
                0,                            /* offset */  
                0                             /* Pointer to the patch data */
            }
        }
    },
    
    /* Command 1: Verify Packet Error */
    {
        pa_CMD_VERIFY_PKT_ERROR,
        {
            {
                0,                            /* ctrlBitfield */
                0,                            /* nPatchBytes */
                0,                            /* totalPatchSize */
                0,                            /* offset */  
                0                             /* Pointer to the patch data */
            }
        }
    },
    
    
}; 

static paCmdInfo_t t5CmdSetCmd1 =
    {
        pa_CMD_CMDSET_AND_USR_STATS,
        {
             
            {
                T5_CMDSET_1_INDEX,               /* Command set index (place holder only) */
                0
            }                   
        }    
    };
  


static uint8_t t5GtpuPatchData[2] = { 0, 0 };

static  paPatchInfo_t t5PatchCmd2 = 
        {
            pa_PATCH_OP_INSERT, /* ctrlBitfield */  
            1,                  /* nPatchBytes */
            1,                  /* totalPatchSize */
            0,                  /* offset */
            t5GtpuPatchData     /* Pointer to the patch data */
        };
      
static paCmdInfo_t t5CmdSet2[] =
{
    /* Command 0: Remove header */
    {
        pa_CMD_REMOVE_HEADER,
        { 
            {
                0,                            /* ctrlBitfield */
                0,                            /* dest */
                0,                            /* pktType */
                0,                            /* flowId */  
                0,                            /* queue */
                0,                            /* swInfo0 */
                0,                            /* swInfo1 */
                0                             /* multiRouteIndex */  
            }
        }
    },
    
    /* Command 1: Insert Bytes */
    {
        pa_CMD_PATCH_DATA,
        {
            {
                0,                            /* ctrlBitfield */
                0,                            /* dest */
                0,                            /* pktType */
                0,                            /* flowId */  
                0,                            /* queue */
                0,                            /* swInfo0 */
                0,                            /* swInfo1 */
                0                             /* multiRouteIndex */  
            }
        }
    },
    
    /* Command 2: Remove Tail */
    {
        pa_CMD_REMOVE_TAIL,
        { 
            {
                0,                            /* ctrlBitfield */
                0,                            /* dest */
                0,                            /* pktType */
                0,                            /* flowId */  
                0,                            /* queue */
                0,                            /* swInfo0 */
                0,                            /* swInfo1 */
                0                             /* multiRouteIndex */  
            }
        }
    }
    
}; 

static paCmdInfo_t t5CmdSetCmd2 =
    {
        pa_CMD_CMDSET,
        {
             
            {
                T5_CMDSET_2_INDEX             /* Command set index */
            }                   
        }    
    };
  

static  paPatchInfo_t t5PatchCmd3 = 
        {
            pa_PATCH_OP_DELETE, /* ctrlBitfield */  
            4,                  /* nPatchBytes */
            0,                  /* totalPatchSize */
            0,                  /* offset */
            NULL                /* Pointer to the patch data */
        };
      
static paCmdInfo_t t5CmdSet3[] =
{
    /* Command 0: Delete Bytes */
    {
        pa_CMD_PATCH_DATA,
        {
            {
                0,                            /* ctrlBitfield */
                0,                            /* dest */
                0,                            /* pktType */
                0,                            /* flowId */  
                0,                            /* queue */
                0,                            /* swInfo0 */
                0,                            /* swInfo1 */
                0                             /* multiRouteIndex */  
            }
        }
    },
	/* Command 1: Remove Tail */
    {
        pa_CMD_REMOVE_TAIL,
        { 
            {
                0,                            /* ctrlBitfield */
                0,                            /* dest */
                0,                            /* pktType */
                0,                            /* flowId */  
                0,                            /* queue */
                0,                            /* swInfo0 */
                0,                            /* swInfo1 */
                0                             /* multiRouteIndex */  
            }
        }
    }
}; 

static paCmdInfo_t t5CmdSetCmd3 =
    {
        pa_CMD_CMDSET,
        {
             
            {
                T5_CMDSET_3_INDEX             /* Command set index */
            }                   
        }    
    };

 
static paCmdReply_t cmdReply = {  pa_DEST_HOST,				/* Dest */
 							      0,						/* Reply ID (returned in swinfo0) */
 							   	  0,						/* Queue */
 							      0 };						/* Flow ID */
 							      
static paRouteInfo_t matchRoute[3] = {  {  	pa_DEST_HOST,		/* Dest */
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

#ifdef _TMS320C6X
#pragma DATA_SECTION(t5Encap, ".testPkts") 
#endif
static t5TestEncap_t  t5Encap;


#define T5_NUM_EXCEPTION_ROUTES    12
 static int t5ErouteTypes[] = {
    pa_EROUTE_GTPU_MESSAGE_TYPE_1,
    pa_EROUTE_GTPU_MESSAGE_TYPE_2,
    pa_EROUTE_GTPU_MESSAGE_TYPE_26,
    pa_EROUTE_GTPU_MESSAGE_TYPE_31,
    pa_EROUTE_GTPU_MESSAGE_TYPE_254,
    pa_EROUTE_GTPU_FAIL,
    pa_EROUTE_IP_FRAG,
    pa_EROUTE_NAT_T_KEEPALIVE,
    pa_EROUTE_NAT_T_CTRL,
    pa_EROUTE_NAT_T_DATA,
    pa_EROUTE_NAT_T_FAIL,
    pa_EROUTE_GTPU_MATCH_FAIL
 };
 
 static paRouteInfo_t t5Eroutes[] = {
 
    /* GTPU Ping Request */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_GTPU_PING_REQ + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_GTPU_PING_REQ_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* GTPU Ping Response */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_GTPU_PING_RESP + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_GTPU_PING_RESP_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* GTPU Error Indication  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_GTPU_ERR_IND + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_GTPU_ERR_IND_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* GTPU Supporeted Header Notify */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_GTPU_HDR_NOTIFY + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_GTPU_HDR_NOTIFY_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* GTPU End Marker  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_GTPU_END_MARKER + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_GTPU_END_MARKER_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* GTPU Parsing Failure */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_GTPU_FAIL + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_GTPU_FAIL_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* IP Fragmentation */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_IP_FRAG + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_IP_FRAG_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* NAT-T Keepalive */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_NAT_T_KEEPALIVE + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_NAT_T_KEEPALIVE_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* NAT-T Control  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_NAT_T_CTRL + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_NAT_T_CTRL_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       &t5CmdSetCmd3        /* No commands ( &t5CmdSetCmd3)*/
    },
    
    /* NAT-T Data  */
 	{  pa_DEST_HOST,		/* Dest */
 	   0,					/* Flow ID */
 	   TF_PA_QUEUE_IPSEC,        /* Q_NAT_T_DATA + TF_FIRST_GEN_QUEUE,*/ /* queue */
 	   -1,					/* Multi route */
 	   T5_NAT_T_DATA_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    },
    
    /* NAT-T Fail  */
 	{  pa_DEST_DISCARD,		/* Dest */
 	   0,					/* Flow ID */
 	   Q_NAT_T_FAIL + TF_FIRST_GEN_QUEUE, /* queue */
 	   -1,					/* Multi route */
 	   T5_NAT_T_FAIL_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    }, 
    
    /* GTPU Match Failure */
    {  pa_DEST_HOST,        /* Dest */
       0,                   /* Flow ID */
       Q_GTPU_MATCH_FAIL + TF_FIRST_GEN_QUEUE, /* queue */
       -1,                  /* Multi route */
       T5_GTPU_MATCH_FAIL_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID, /* sw Info 0 */
       0,                   /* sw Info 1 */       
       0,                   /* customType : not used */         
       0,                   /* customIndex: not used */     
       0,                   /* pkyType: for SRIO only */    
       NULL                 /* No commands */
    }
    
 };
 
/*
 * User-defined statistics
 */ 
 
typedef struct t5UsrStatsLnkEntry_s
{
    uint16_t          lnkIndex;    /* next layer counter index */
    paUsrStatsTypes_e cntType;     /**< Counter type (packet counter */
    
} t5UsrStatsLnkEntry_t;


static t5UsrStatsLnkEntry_t t5UsrStatsLnkTbl[pa_USR_STATS_MAX_COUNTERS];
static pauUsrStatsEntry_t   t5UsrStatsTbl[pa_USR_STATS_MAX_COUNTERS];
static int      t5NumUsrStats = 0;

static paUsrStats_t  paTestExpectedUsrStats;

static void t5Cleanup (t5TestEncap_t *tencap, paTestStatus_t status)
{
	int			   i;		
	int  	       cmdDest;
 	uint16_t	   cmdSize;
 	paReturn_t     paret;
 	paTestStatus_t newStatus;
 	Cppi_HostDesc *hd;
 	
 	
	/* Wait a bit for any packets in PA to complete */
	utilCycleDelay (5000);
	
	/* Return the descriptors which are pointing to packets */
	for (i = 0; i < T5_NUM_LOCAL_L3_HANDLES; i++)  {
		if (tencap->hd[i] != NULL)
			Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
	}
	
 	/* Delete active L4 handles */
 	for (i = 0; i < T5_NUM_LOCAL_L4_HANDLES; i++)  {
		
		cmdReply.replyId = T5_CMD_SWINFO0_DEL_PORT_ID + i; 
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l4Handles[i].state == T5_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T5_HANDLE_ACTIVE))  {
			hd = testCommonDelL4Handles (tencap->tf, tencap->l4Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);
					
			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				status = PA_TEST_FAILED;
				continue;
			}
		
			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	        paTestL4ExpectedStats.classify2.nPackets += 1;
            #ifdef NSS_GEN2
            paTestL4ExpectedStats.classify1.nPackets += 1;
		    #endif
            
			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				status = PA_TEST_FAILED;
 			}
 			
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
    
 	/* Delete active L5 handles */
 	for (i = 0; i < T5_NUM_LOCAL_L5_HANDLES; i++)  {
		
		cmdReply.replyId = T5_CMD_SWINFO0_DEL_TEID_ID + i; 
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l5Handles[i].state == T5_HANDLE_PENDING_ACK) || (tencap->l5Handles[i].state == T5_HANDLE_ACTIVE))  {
			hd = testCommonDelL4Handles (tencap->tf, tencap->l5Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);
					
			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				status = PA_TEST_FAILED;
				continue;
			}
		
			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	        paTestL4ExpectedStats.classify2.nPackets += 1;
            #ifdef NSS_GEN2
 	        paTestL4ExpectedStats.classify1.nPackets += 1;
            #endif
		
			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				status = PA_TEST_FAILED;
 			}
 			
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
    
 	
  	/* Delete active L3 Handles */
 	for (i = 0; i < T5_NUM_LOCAL_L3_HANDLES; i++)  {
 		cmdReply.replyId = T5_CMD_SWINFO0_DEL_IP_ID + i; 
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l3Handles[i].state == T5_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T5_HANDLE_ACTIVE))  {
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

  	/* Delete active SPI L3 Handles */
 	for (i = 0; i < T5_NUM_LOCAL_SPI_L3_HANDLES; i++)  {
 		cmdReply.replyId = T5_CMD_SWINFO0_DEL_SPI_IP_ID + i; 
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l3SpiHandles[i].state == T5_HANDLE_PENDING_ACK) || (tencap->l3SpiHandles[i].state == T5_HANDLE_ACTIVE))  {
			hd = testCommonDelHandle (tencap->tf, &tencap->l3SpiHandles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
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
 	
  	/* Delete active L2 Handles */
 	for (i = 0; i < T5_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T5_CMD_SWINFO0_DEL_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l2Handles[i].state == T5_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T5_HANDLE_ACTIVE))  {
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
 			
 			/* Recycle the command packet as well */
 			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			if (hd == NULL)  {
 				System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				continue;
 			}
 			testCommonRecycleLBDesc (tencap->tf, hd);
 			
 			
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

static paTestStatus_t t5DeleteL4 (t5TestEncap_t *tencap)
{
	int			   i;		
	int  	       cmdDest;
 	uint16_t	       cmdSize;
 	paReturn_t     paret;
 	paTestStatus_t status = PA_TEST_PASSED;
 	Cppi_HostDesc *hd;
 	
 	/* Delete active L4 handles */
 	for (i = 0; i < T5_NUM_LOCAL_L4_HANDLES; i++)  {
		
		cmdReply.replyId = T5_CMD_SWINFO0_DEL_PORT_ID + i; 
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l4Handles[i].state == T5_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T5_HANDLE_ACTIVE))  {
			hd = testCommonDelL4Handles (tencap->tf, tencap->l4Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);
					
			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				status = PA_TEST_FAILED;
				continue;
			}
		
			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	        paTestL4ExpectedStats.classify2.nPackets += 1;
            #ifdef NSS_GEN2
 	        paTestL4ExpectedStats.classify1.nPackets += 1;
            #endif
            
		
			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				status = PA_TEST_FAILED;
 			}
 			
 			/* Recycle the command packet as well */
 			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			if (hd == NULL)  {
 				System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				status = PA_TEST_FAILED;
 				continue;
 			}
 			testCommonRecycleLBDesc (tencap->tf, hd);
            
            tencap->l4Handles[i].state = T5_HANDLE_DISABLED;
 		    tencap->l3Handles[T5_L3HDL_IDX_FROM_PORT(T5_FIRST_DEST_PORT+i)].linkCnt--;
 		}
 	}
    
    return (status);
}  

static paTestStatus_t t5DeleteL5 (t5TestEncap_t *tencap, int fLnkEn)
{
	int			   i;		
	int  	       cmdDest;
 	uint16_t	       cmdSize;
 	paReturn_t     paret;
 	paTestStatus_t status = PA_TEST_PASSED;
 	Cppi_HostDesc *hd;
    
 	/* Delete active L5 handles */
 	for (i = 0; i < T5_NUM_LOCAL_L5_HANDLES; i++)  {
		
		cmdReply.replyId = T5_CMD_SWINFO0_DEL_TEID_ID + i; 
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l5Handles[i].state == T5_HANDLE_PENDING_ACK) || (tencap->l5Handles[i].state == T5_HANDLE_ACTIVE))  {
			hd = testCommonDelL4Handles (tencap->tf, tencap->l5Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
					&cmdReply, &cmdDest, &cmdSize, &paret);
					
			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				status = PA_TEST_FAILED;
				continue;
			}
		
			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	        paTestL4ExpectedStats.classify2.nPackets += 1;
            #ifdef NSS_GEN2
 	        paTestL4ExpectedStats.classify1.nPackets += 1;
            #endif
		
			/* Wait for the response */
			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
				status = PA_TEST_FAILED;
 			}
 			
 			/* Recycle the command packet as well */
 			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 			if (hd == NULL)  {
 				System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 				status = PA_TEST_FAILED;
 				continue;
 			}
 			testCommonRecycleLBDesc (tencap->tf, hd);
            tencap->l5Handles[i].state = T5_HANDLE_DISABLED;
            
            if(fLnkEn)
 		        tencap->l3Handles[T5_L3HDL_IDX_FROM_TEID(T5_FIRST_GTPU_TEID+i, T5_FIRST_GTPU_TEID)].linkCnt--;
            

 		}
 	}
    
    return (status);
}  


			
static void t5CmdRep (t5TestEncap_t *tencap)
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
		    
		    swinfoType = swinfo[0] & T5_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T5_CMD_SWINFO0_ID_MASK;
		    
		    paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);

		    
		    switch (swinfoType)  {
		    	
		    	case T5_CMD_SWINFO0_ADD_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T5_HANDLE_ACTIVE;
		    		max = T5_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_addMac";
		    		break;
		    	
		    	case T5_CMD_SWINFO0_ADD_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T5_HANDLE_ACTIVE;
		    		max = T5_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_addIp";
		    		break;
            
          case T5_CMD_SWINFO0_ADD_SPI_IP_ID:
		    		stateP = &tencap->l3SpiHandles[swinfoIdx].state;
		    		stateV = T5_HANDLE_ACTIVE;
		    		max = T5_NUM_LOCAL_SPI_L3_HANDLES;
		    		s = "pa_addIp (SPI)";
		    		break;            
		    	
		    	case T5_CMD_SWINFO0_ADD_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T5_HANDLE_ACTIVE;
		    		max = T5_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_addPort";
		    		break;
                    
		    	case T5_CMD_SWINFO0_ADD_TEID_ID:
		    		stateP = &tencap->l5Handles[swinfoIdx].state;
		    		stateV = T5_HANDLE_ACTIVE;
		    		max = T5_NUM_LOCAL_L5_HANDLES;
		    		s = "pa_addPort (TEID)";
		    		break;
                    
		    		
		    	case T5_CMD_SWINFO0_DEL_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T5_HANDLE_DISABLED;
		    		max = T5_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_delMac";
		    		break;
		    		
		    	case T5_CMD_SWINFO0_DEL_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T5_HANDLE_DISABLED;
		    		max = T5_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_delIp";
		    		break;

		    	case T5_CMD_SWINFO0_DEL_SPI_IP_ID:
		    		stateP = &tencap->l3SpiHandles[swinfoIdx].state;
		    		stateV = T5_HANDLE_DISABLED;
		    		max = T5_NUM_LOCAL_SPI_L3_HANDLES;
		    		s = "pa_delIp (SPI)";
		    		break;            
		    		
		    	case T5_CMD_SWINFO0_DEL_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T5_HANDLE_DISABLED;
		    		max = T5_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_delPort";
		    		break;
                    
                case T5_CMD_SWINFO0_EROUTE_CFG_ID:
                case T5_CMD_SWINFO0_CMDSET_CFG_ID:
                case T5_CMD_SWINFO0_GLOBAL_CFG_ID:
                case T5_CMD_SWINFO0_NAT_T_CFG_ID:
                case T5_CMD_SWINFO0_GTPU_CFG_ID:
                case T5_CMD_SWINFO0_USR_STATS_CFG_ID:
                    stateP = &tencap->genCmdAck[swinfoIdx];
                    stateV = TRUE;
                    max = T5_NUM_GEN_CONFIGS;
                    s = "Pa_configExceptionRoutes/Pa_configCmdSet/Pa_control/Pa_configUsrStats";
                    break;     
		    		
		    	default:
		    		System_printf ("%s (%s:%d): Unknown command ID found in swinfo0 (0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
		    		t5Cleanup (tencap, PA_TEST_FAILED);
		    		break;
		    	
		    }
		 		    
		    /* In this test only valid responses are exected from PA */
		    if (paret != pa_OK)  {
		    	System_printf ("%s (%s:%d): PA command %s returned error code %d, Index = %d\n", tfName, __FILE__, __LINE__, s, paret, swinfoIdx);
		    	t5Cleanup (tencap, PA_TEST_FAILED);
		    }
		    	
		    
		   	if (swinfoIdx >= max)  {
		   		System_printf ("%s (%s:%d): Received command ack (%s) for out of range handle (%d, max = %d)\n",
		   						tfName, __FILE__, __LINE__, s, swinfoIdx, max);
		   		t5Cleanup (tencap, PA_TEST_FAILED);
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
	
}

static paTestStatus_t t5ExceptionRoutes (t5TestEncap_t *tencap)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    paCmdInfo_t     cmdInfo[2];
    
    /* Specify the user statistics associated with the MAC input */
    cmdInfo[0].cmd = pa_CMD_USR_STATS;
    cmdInfo[0].params.usrStats.index = t5NatTKeepAliveUsrStats;
    cmdInfo[1].cmd = pa_CMD_USR_STATS;
    cmdInfo[1].params.usrStats.index = t5NatTErrUsrStats;
    
    t5Eroutes[7].pCmd = &cmdInfo[0]; 
    t5Eroutes[10].pCmd = &cmdInfo[1]; 
    
    /* Issue the exception route command */
    cmdReply.replyId  = T5_CMD_SWINFO0_EROUTE_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
    for(i = 0; i < T5_NUM_EXCEPTION_ROUTES; i++)
        t5Eroutes[i].flowId = tencap->tf->tfFlowNum[0];    
 	hd = testCommonConfigExceptionRoute (tencap->tf, T5_NUM_EXCEPTION_ROUTES, t5ErouteTypes, t5Eroutes,  
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
		t5CmdRep (tencap);
		
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

static paTestStatus_t t5GlobalConfiguration (t5TestEncap_t *tencap)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paCtrlInfo_t    ctrlInfo;
    
    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = t5GlobalCfg;
    cmdReply.replyId  = T5_CMD_SWINFO0_GLOBAL_CFG_ID;
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
		t5CmdRep (tencap);
		
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

static paTestStatus_t t5NatTConfiguration (t5TestEncap_t *tencap, int enable)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paCtrlInfo_t    ctrlInfo;
    
    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_IPSEC_NAT_T_CONFIG;
    ctrlInfo.params.ipsecNatTDetCfg = t5NatTCfg;
    ctrlInfo.params.ipsecNatTDetCfg.ctrlBitMap = enable?pa_IPSEC_NAT_T_CTRL_ENABLE:0;
    cmdReply.replyId  = T5_CMD_SWINFO0_NAT_T_CFG_ID;
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
    #ifdef NSS_GEN2
 	paTestL4ExpectedStats.classify1.nPackets += 1;
    #endif
    
    tencap->genCmdAck[0] = FALSE;
    
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t5CmdRep (tencap);
		
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

static paTestStatus_t t5GtpuConfiguration (t5TestEncap_t *tencap, int linkEn, int routeEn)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paCtrlInfo_t    ctrlInfo;
    
    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_GTPU_CONFIG;
    ctrlInfo.params.gtpuCfg.ctrlBitMap = linkEn?pa_GTPU_CTRL_USE_LINK:0;

    /* Route End Marker as G-PDU flag */
    if (routeEn)
    {
      ctrlInfo.params.gtpuCfg.ctrlBitMap |= pa_GTPU_CTRL_ROUTE_END_MARKER_AS_GPDU;
    }
    
    cmdReply.replyId  = T5_CMD_SWINFO0_GTPU_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
 	hd = testCommonGlobalConfig (tencap->tf, &ctrlInfo,  
 	                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3, 
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);
                                         
   if (hd == NULL)  {
   			
   	 System_printf ("%s: (%s:%d): Failure in GTPU Config command\n", tfName, __FILE__, __LINE__);
   	 return (PA_TEST_FAILED);
   }								 
    
    /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	paTestL4ExpectedStats.classify2.nPackets += 1;
    #ifdef NSS_GEN2
 	paTestL4ExpectedStats.classify1.nPackets += 1;
    #endif
    
    tencap->genCmdAck[0] = FALSE;
    
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t5CmdRep (tencap);
		
		if (tencap->genCmdAck[0])
			break;
		else
			utilCycleDelay (500);
	}
			
	if (i == 100)  {
		System_printf ("%s: (%s:%d): Pa_control (GTPU) commands was not acked\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}


static paTestStatus_t t5CmdSetConfiguration (t5TestEncap_t *tencap)
{
	int 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    uint16_t        cmdSetIndex[] = {T5_CMDSET_1_INDEX, T5_CMDSET_2_INDEX, T5_CMDSET_3_INDEX};
    int             numCmds[] = {T5_CMDSET_1_NUM_CMDS, T5_CMDSET_2_NUM_CMDS, T5_CMDSET_3_NUM_CMDS};
    paCmdInfo_t*    pCmdInfo[] = {t5CmdSet1, t5CmdSet2, t5CmdSet3}; 
    
    
    
    /* Issue the command set command */
    cmdReply.replyId  = T5_CMD_SWINFO0_CMDSET_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
    
    for (j = 0; j < 3; j++)
    {
 	    hd = testCommonConfigCmdSet (tencap->tf, cmdSetIndex[j], numCmds[j], pCmdInfo[j],  
 	                                        tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3, 
 	                                        &cmdReply, &cmdDest, &cmdSize, &paret);
                                         
        if (hd == NULL)  {
   			
   	        System_printf ("%s: (%s:%d): Failure in ConfigCmdSet command\n", tfName, __FILE__, __LINE__);
   	        return (PA_TEST_FAILED);
        }								 
    
        /* Send command */
 	    Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
        tencap->genCmdAck[0] = FALSE;
    
	    /* All the packets should have been acked */
	    for (i = 0; i < 100; i++)  {
		    t5CmdRep (tencap);
		
		    if (tencap->genCmdAck[0])
			    break;
		    else
			    utilCycleDelay (500);
	    }
			
	    if (i == 100)  {
		    System_printf ("%s: (%s:%d): Pa_configCmdSet (%d) commands was not acked\n", tfName, __FILE__, __LINE__, j);
		    return (PA_TEST_FAILED);
	    }
    }
	return (PA_TEST_PASSED);
}


/*
 * Initialize the link table 
 */

static void t5InitUsrStatsLnkTbl (t5UsrStatsLnkEntry_t* pEntry)
{
    int i;
    
    for (i = 0; i < pa_USR_STATS_MAX_COUNTERS; i++)
    {
        pEntry[i].lnkIndex = pa_USR_STATS_LNK_END;
        pEntry[i].cntType = pa_USR_STATS_TYPE_PACKET;    
    }     
}

static void t5UpdateUsrStatsLnkTbl (t5UsrStatsLnkEntry_t* pLnkTbl, paUsrStatsCounterConfig_t* pCntCfg)
{
    int i;
	paUsrStatsCounterEntryConfig_t* cntInfo = pCntCfg->cntInfo;
    int numCnt = pCntCfg->numCnt;	
    
    for (i = 0; i < numCnt; i++)
    {
        pLnkTbl[cntInfo[i].cntIndex].lnkIndex = cntInfo[i].cntLnk;
        pLnkTbl[cntInfo[i].cntIndex].cntType = cntInfo[i].cntType;
        t5UsrStatsTbl[t5NumUsrStats++].cntIndex = cntInfo[i].cntIndex;
    }     
}

/*
 * Update all counters in the link chain
 */
static void t5UpdateUsrStats(paUsrStats_t* pStats,  t5UsrStatsLnkEntry_t* pLnktbl,  uint16_t cntIndex, uint16_t pktSize)
{
    while (cntIndex != pa_USR_STATS_LNK_END)
    {
        if (cntIndex < T5_NUM_64B_USR_STATS)
        {
            pStats->count64[cntIndex] += (pLnktbl[cntIndex].cntType == pa_USR_STATS_TYPE_PACKET)?1:pktSize;
        } 
        else
        {
            pStats->count32[cntIndex - T5_NUM_64B_USR_STATS] += (pLnktbl[cntIndex].cntType == pa_USR_STATS_TYPE_PACKET)?1:pktSize;
        
        }
        
        cntIndex = pLnktbl[cntIndex].lnkIndex;
    }
}

static paTestStatus_t t5ConfigUsrStats (t5TestEncap_t *tencap, int numEntries, pauUsrStatsSetup_t *usrStatsSetup, int clear)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paUsrStatsCounterConfig_t   cntCfg;
    paUsrStatsConfigInfo_t      statsCfgInfo;
	
    statsCfgInfo.pCntCfg = &cntCfg;
    if (!clear)
    {
	    for (i = 0; i < numEntries; i++)  {
		    cmdReply.replyId = T5_CMD_SWINFO0_USR_STATS_CFG_ID + i;
		    cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
        
            memset(&cntCfg, 0, sizeof(cntCfg));
            cntCfg.numCnt = usrStatsSetup[i].nStats;
            cntCfg.cntInfo = usrStatsSetup[i].cntEntryTbl;
		
		    hd = testCommonConfigUsrStats (tencap->tf, &statsCfgInfo, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3, 
 	        	                        &cmdReply, &cmdDest, &cmdSize, &paret);
								 
            if (paret != pa_OK)
            {
                if (paret != usrStatsSetup[i].paret)
                {
			        System_printf ("%s: (%s:%d): configUsrStats command (%d): unexpected err = %d (expected err = %d)\n", tfName, 
                                __FILE__, __LINE__, i, paret, usrStatsSetup[i].paret);
		            return (PA_TEST_FAILED);
			    
                }
                tencap->genCmdAck[i] = TRUE;
                continue;
            }
            else if (hd == NULL)  {
					
			    System_printf ("%s: (%s:%d): Failure in common configUsrStats command, entry number %d\n", tfName, __FILE__, __LINE__, i);
		        return (PA_TEST_FAILED);
		    }	        				 
								 
		    Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
            tencap->genCmdAck[i] = FALSE;
        
            t5UpdateUsrStatsLnkTbl(t5UsrStatsLnkTbl, &cntCfg);
        
 		    t5CmdRep (tencap);
	    }
    }
    else
    {
        numEntries = 1;
	    cmdReply.replyId = T5_CMD_SWINFO0_USR_STATS_CFG_ID;
	    cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
    
        memset(&cntCfg, 0, sizeof(cntCfg));
        cntCfg.ctrlBitfield = pa_USR_STATS_CONFIG_RESET;
        cntCfg.numCnt = 0;
        cntCfg.cntInfo = NULL;
	
	    hd = testCommonConfigUsrStats (tencap->tf, &statsCfgInfo, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3, 
 		                               &cmdReply, &cmdDest, &cmdSize, &paret);
						 
        if (paret != pa_OK)
        {
	        System_printf ("%s: (%s:%d): configUsrStats command (%d): unexpected err = %d\n", tfName, 
                        __FILE__, __LINE__, 0, paret);
	        return (PA_TEST_FAILED);
        }
        else if (hd == NULL)  {
			
	        System_printf ("%s: (%s:%d): Failure in common configUsrStats (clear) command\n", tfName, __FILE__, __LINE__);
	        return (PA_TEST_FAILED);
	    }	        				 
						 
	    Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
        tencap->genCmdAck[0] = FALSE;
    
        t5InitUsrStatsLnkTbl(t5UsrStatsLnkTbl);
    
 	    t5CmdRep (tencap);
    
    }
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t5CmdRep (tencap);
		
		for (j = m = 0; j < numEntries; j++)  {
			if (tencap->genCmdAck[j])
				m += 1;
		}
		
		if (m == numEntries)
			break;
		else
			utilCycleDelay (1000);
	}
			
	if (i == 100)  {
		System_printf ("%s: (%s:%d):  %d (out of %d) pa_configUsrStats commands were acked\n", tfName, __FILE__, __LINE__, m, numEntries);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}

static paTestStatus_t t5OpenL2 (t5TestEncap_t *tencap, t5EthSetup_t *ethSetup, int nL2Handles)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paCmdInfo_t     cmdInfo;
    
    cmdInfo.cmd = pa_CMD_USR_STATS;
    matchRoute[1].pCmd = &cmdInfo; 
    
	
	for (i = 0; i < nL2Handles; i++)  {
		cmdReply.replyId = T5_CMD_SWINFO0_ADD_MAC_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
        
        /* Specify the user statistics associated with the MAC input */
        cmdInfo.params.usrStats.index = t5L2UsrStats[i];
		
		hd = testCommonAddMac (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t5EthSetup[i].ethInfo, &matchRoute[1], &nfailRoute,
 	    	                   &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, 
 	        	               &cmdReply, &cmdDest, &cmdSize, &paret);
								 
		if (hd == NULL)  {
					
			System_printf ("%s: (%s:%d): Failure in common addMac command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}								 
								 
		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l2Handles[i].state = T5_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
 		
 		t5CmdRep (tencap);
	}
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t5CmdRep (tencap);
		
		for (j = m = 0; j < nL2Handles; j++)  {
			if (tencap->l2Handles[j].state == T5_HANDLE_ACTIVE)
				m += 1;
		}
		
		if (m == nL2Handles)
			break;
		else
			utilCycleDelay (1000);
	}
			
	if (i == 100)  {
		System_printf ("%s: (%s:%d):  %d (out of %d) pa_addMac commands were acked\n", tfName, __FILE__, __LINE__, m, nL2Handles);
		return (PA_TEST_FAILED);
	}
	return (PA_TEST_PASSED);
}

static paTestStatus_t t5OpenSpiL3 (t5TestEncap_t *tencap, t5SpiIpSetup_t *ipSetup, int nL3Handles)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
  paRouteInfo_t  spiMatchRoute = {  	pa_DEST_HOST,		/* Dest */
 								       		0,					/* Flow ID */
 								       		0,					/* queue */
 								       	   -1,					/* Multi route */
                                            0,					        /* sw Info 0 */ 								       		
                                            0,                  /* sw Info 1 */       
                                            0,                  /* customType : not used  */         
                                            0,                  /* customIndex: not used  */     
                                            0,                  /* pkyType: for SRIO only */    
                                            NULL};              /* No commands            */;
  paHandleL2L3_t   linkHandle;

  spiMatchRoute.queue  = (Q_NAT_T_DATA + TF_FIRST_GEN_QUEUE);
  spiMatchRoute.flowId = tencap->tf->tfFlowNum[0];
	
	for (i = 0; i < nL3Handles; i++)  {
		cmdReply.replyId = T5_CMD_SWINFO0_ADD_SPI_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

    if (t5SpiIpSetup[i].lHandleIdx == -1)
      linkHandle = NULL;
    else
      linkHandle = tencap->l3Handles[t5SpiIpSetup[i].lHandleIdx].paHandle;

    spiMatchRoute.swInfo0 = (T5_NAT_T_DATA_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID + i);
		
		hd = testCommonAddIp (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t5SpiIpSetup[i].ipInfo, &spiMatchRoute, &nfailRoute,
								 &tencap->l3SpiHandles[i].paHandle, 
								 linkHandle,
								 tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
								 &cmdReply, &cmdDest, &cmdSize, &paret);
								 
								 
		if (hd == NULL)  {
					
			System_printf ("%s: (%s:%d): Failure in common addIp command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}								 
								 
		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3SpiHandles[i].state = T5_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
    if (t5SpiIpSetup[i].lHandleIdx != -1)
      tencap->l3Handles[t5SpiIpSetup[i].lHandleIdx].linkCnt++;
    
 		t5CmdRep (tencap);
	}
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t5CmdRep (tencap);
		
		for (j = m = 0; j < nL3Handles; j++)  {
			if (tencap->l3SpiHandles[j].state == T5_HANDLE_ACTIVE)
				m += 1;
		}
		
		if (m == nL3Handles)
			break;
		else
			utilCycleDelay (1000);
	}
			
	if (i == 100)  {
		System_printf ("%s: (%s:%d):  %d (out of %d) pa_addIp commands were acked\n", tfName, __FILE__, __LINE__, m, nL3Handles);
		return (PA_TEST_FAILED);
	}	
	
	return (PA_TEST_PASSED);
}


static paTestStatus_t t5OpenL3 (t5TestEncap_t *tencap, t5IpSetup_t *ipSetup, int nL3Handles)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
	
	for (i = 0; i < nL3Handles; i++)  {
		cmdReply.replyId = T5_CMD_SWINFO0_ADD_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
		
		hd = testCommonAddIp (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t5IpSetup[i].ipInfo, &matchRoute[2], &nfailRoute,
								 &tencap->l3Handles[i].paHandle, 
								 tencap->l2Handles[t5IpSetup[i].lHandleIdx].paHandle,
								 tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
								 &cmdReply, &cmdDest, &cmdSize, &paret);
								 
								 
		if (hd == NULL)  {
					
			System_printf ("%s: (%s:%d): Failure in common addIp command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}								 
								 
		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[i].state = T5_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
 		tencap->l2Handles[t5IpSetup[i].lHandleIdx].linkCnt++;
 		t5CmdRep (tencap);
	}
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t5CmdRep (tencap);
		
		for (j = m = 0; j < nL3Handles; j++)  {
			if (tencap->l3Handles[j].state == T5_HANDLE_ACTIVE)
				m += 1;
		}
		
		if (m == nL3Handles)
			break;
		else
			utilCycleDelay (1000);
	}
			
	if (i == 100)  {
		System_printf ("%s: (%s:%d):  %d (out of %d) pa_addIp commands were acked\n", tfName, __FILE__, __LINE__, m, nL3Handles);
		return (PA_TEST_FAILED);
	}	
	
	return (PA_TEST_PASSED);
}

static paTestStatus_t t5GenericOpenL4 (t5TestEncap_t *tencap, unsigned int fDestPort, int nL3Handles, int nL4Handles)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    
    matchRoute[0].pCmd = &t5CmdSetCmd1;
	
	for (i = 0; i < nL4Handles; i++)  {
			
		cmdReply.replyId  = T5_CMD_SWINFO0_ADD_PORT_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
        t5CmdSetCmd1.params.cmdSetUsrStats.statsIndex = t5L3UsrStats[T5_L3HDL_IDX_FROM_PORT(T5_FIRST_DEST_PORT+i)];
		
		hd = testCommonAddPort (tencap->tf, pa_LUT2_PORT_SIZE_16, T5_FIRST_DEST_PORT+i, &matchRoute[0], &tencap->l4Handles[i].paHandle, 
								&tencap->l3Handles[T5_L3HDL_IDX_FROM_PORT(T5_FIRST_DEST_PORT+i)].paHandle,
                                tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);
	
								 
								 
		if (hd == NULL)  {
					
			System_printf ("%s: (%s:%d): Failure in common addPort command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}								 
								 
		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l4Handles[i].state = T5_HANDLE_PENDING_ACK;
 		tencap->l3Handles[T5_L3HDL_IDX_FROM_PORT(T5_FIRST_DEST_PORT+i)].linkCnt++;
 	    paTestL4ExpectedStats.classify2.nPackets += 1;
        #ifdef NSS_GEN2
 	    paTestL4ExpectedStats.classify1.nPackets += 1;
        #endif
        
 		utilCycleDelay (600);
 		
 		t5CmdRep (tencap);
	}
	
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t5CmdRep (tencap);
		
		for (j = m = 0; j < nL4Handles; j++)  {
			if (tencap->l4Handles[j].state == T5_HANDLE_ACTIVE)
				m += 1;
		}
		
		if (m == nL4Handles)
			break;
		else
			utilCycleDelay (1000);
	}
			
	if (i == 100)  {
		System_printf ("%s: (%s:%d):  %d (out of %d) pa_addPort commands were acked\n", tfName, __FILE__, __LINE__, m, nL4Handles);
		return (PA_TEST_FAILED);
	}	
	
	return (PA_TEST_PASSED);
}

static paTestStatus_t t5GenericOpenGTPU (t5TestEncap_t *tencap, unsigned int fGTPUTeid,  unsigned int GTPUTeidBase, int nL5Handles, int fLnkEn)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
	
	for (i = 0; i < nL5Handles; i++)  {
			
		cmdReply.replyId  = T5_CMD_SWINFO0_ADD_TEID_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
		
        matchRoute[0].swInfo0 = T5_GTPU_FIRST_PKT_INDEX + T5_CMD_SWINFO0_PKT_ID + i;
        matchRoute[0].pCmd = (i & 1)?&t5CmdSetCmd2:NULL;
		hd = testCommonAddPort (tencap->tf, pa_LUT2_PORT_SIZE_32, fGTPUTeid+i, &matchRoute[0], &tencap->l5Handles[i].paHandle, 
								&tencap->l3Handles[T5_L3HDL_IDX_FROM_TEID(fGTPUTeid+i, GTPUTeidBase)].paHandle,
                                tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);
								 
		if (hd == NULL)  {
					
			System_printf ("%s: (%s:%d): Failure in common addPort (TEID) command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}								 
								 
		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l5Handles[i].state = T5_HANDLE_PENDING_ACK;
 	    paTestL4ExpectedStats.classify2.nPackets += 1;
        #ifdef NSS_GEN2
 	    paTestL4ExpectedStats.classify1.nPackets += 1;
        #endif
        if(fLnkEn)
     		tencap->l3Handles[T5_L3HDL_IDX_FROM_TEID(fGTPUTeid+i, GTPUTeidBase)].linkCnt++;
         		
 		utilCycleDelay (600);
 		
 		t5CmdRep (tencap);
	}
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t5CmdRep (tencap);
		
		for (j = m = 0; j < nL5Handles; j++)  {
			if (tencap->l5Handles[j].state == T5_HANDLE_ACTIVE)
				m += 1;
		}
		
		if (m == nL5Handles)
			break;
		else
			utilCycleDelay (1000);
	}
			
	if (i == 100)  {
		System_printf ("%s: (%s:%d):  %d (out of %d) pa_addPort commands were acked\n", tfName, __FILE__, __LINE__, m, nL5Handles);
		return (PA_TEST_FAILED);
	}	
	
	return (PA_TEST_PASSED);
}

static void t5FormPktDataDescriptors (t5TestEncap_t *tencap, int nL3Handles)
{
	Qmss_Queue      	q;
	int 				i;
	uint16_t  			cmdStackSize;
	pasahoNextRoute_t  *panr;
	paReturn_t      	paret;
	
	
	paTxChksum_t t5pktChksum = {  /* The UDP checksum command */
    
    	0,     	/* Start offset of UDP header */
    	0,     	/* Checksum length (UDP header + payload */
    	0,      /* Offset to checksum location RELATIVE TO THE START OF THE TCP/UDP HEADER */
    	0, 		/* Initial value is IPv4 pseudo header checksum value */
    	1       /* computed value of 0 written as -0 */

	};
	

	
	paRouteInfo_t route = {    	pa_DEST_HOST,		/* Dest */
 								0,					/* Flow ID */
 								0,					/* queue */
 							   -1,					/* Multi route */
 								0,					/* sw Info 0 */
                                0,                  /* sw Info 1 */       
                                0,                  /* customType : not used  */         
                                0,                  /* customIndex: not used  */     
                                0,                  /* pkyType: for SRIO only */    
                                NULL};              /* No commands            */
 								       		
	
	
	/* Attach one free descriptor to each of the packets */
	for (i = 0; i < nL3Handles; i++)  {
		tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);
		
		if (tencap->hd[i] == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}
	
		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);
  		
  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)t5PktInfo[i].pkt, (uint32_t)t5PktInfo[i].pktLen);
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t5PktInfo[i].pktLen);
        
  		/* Create the UDP checksum and routing commands as well and attach them to the descriptor */
    	panr = (pasahoNextRoute_t *)tencap->cmdStack[i];  /* Where the command will be placed */
    	   	
    	/* Route the packet to PA Tx 0 which makes it appear to have arrived over the network */
		route.queue = tencap->tf->QPaTx[TF_PA_Q_INPUT];
        route.flowId = tencap->tf->tfFlowNum[0];
		
		if (t5PktInfo[i].statsMap[2] & (1 << TF_STATS_BM_C2_NUM_UDP))
			t5pktChksum.resultOffset = 6;
		else if (t5PktInfo[i].statsMap[2] & (1 << TF_STATS_BM_C2_NUM_TCP))
			t5pktChksum.resultOffset = 16;
		else  {
			System_printf ("%s (%s:%d): Could not determine if packet was TCP or UDP\n", tfName, __FILE__, __LINE__);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}
		
		t5pktChksum.initialSum  = t5PseudoChksums[i];
		t5pktChksum.lengthBytes = PASAHO_LINFO_READ_END_OFFSET(t5PktInfo[i].info) - PASAHO_LINFO_READ_L4_OFFSET(t5PktInfo[i].info);
		t5pktChksum.startOffset = PASAHO_LINFO_READ_L4_OFFSET(t5PktInfo[i].info);
		
		cmdStackSize = sizeof(tencap->cmdStack[i]);

   		paret = Pa_formatTxRoute (  &t5pktChksum,       /* L4 payload checksum */
                                	NULL,               /* No second checksum   */
                                	&route,             /* Internal routing     */
                                	(Ptr)panr,          /* Command buffer       */
                                	&cmdStackSize);   	/* Command size         */
                                	
        if (paret != pa_OK)  {
        	System_printf ("%s (%s:%d): Pa_formatTxRoute returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        	t5Cleanup (tencap, PA_TEST_FAILED);
        }
        
  		Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)panr, cmdStackSize);
	}
	
}

static Cppi_HostDesc *t5FormGTPUDataPacket (tFramework_t *tf, paTest_t *pat, pktTestInfo_t *pktInfo)
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
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(pktInfo->pkt)), pktInfo->pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, pktInfo->pktLen);
  	
  	return (hd);
}

static Cppi_HostDesc *t5FormNatTDataPacket (tFramework_t *tf, paTest_t *pat, int pktIdx)
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
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t5NatTPktInfo[pktIdx].info.pkt)), t5NatTPktInfo[pktIdx].info.pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t5NatTPktInfo[pktIdx].info.pktLen);
  	
  	return (hd);
}



static int t5RxPkts (t5TestEncap_t *tencap, pauFifo_t *fifo)
{
	Cppi_HostDesc  		*hd;
	pasahoLongInfo_t 	*pinfo;
	uint32_t		      	 infoLen;
	uint32_t				 flags;
	int					 idx;
	unsigned int				 l4Offset;
	uint8_t				*rxPkt;
	unsigned int				 port;
	unsigned int				 eport;
	int					 nQ, i, n;
    int                  msmcBounceCount=0;
    int                  count = 0;

	for (i = 0 ;i < 100; i++) {	
        testCommonRelayQueueBouncePkts (tencap->tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &msmcBounceCount);
    	/* Look for packets in the rx packet queue, verify that the UDP port found matches the next value in
    	 * the fifo, then verify the receive packet information */
    	while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_MATCH]) > 0)  {
    		
    		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
    		if (hd == NULL)  {
    			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
    			return (-1);
    		}
    		
    		/* Get the parse information, make sure there is an L4 offset */
    		if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
    			System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}
    		
    		rxPkt = (uint8_t *)hd->buffPtr;
    		flags = PASAHO_LINFO_READ_HDR_BITMASK(pinfo);
    		l4Offset = PASAHO_LINFO_READ_L4_OFFSET(pinfo);
    
    		
    		if ((flags & (PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_TCP)) == 0)  {
    			System_printf ("%s (%s:%d): Found a packet without any L4 info\n", tfName, __FILE__, __LINE__);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}
    
    		/* Same offset to destination port for both TCP and UDP */
    		port = (rxPkt[l4Offset+2] << 8) | rxPkt[l4Offset+3];
    		
    		if ((port < T5_FIRST_DEST_PORT) || (port >= (T5_FIRST_DEST_PORT + T5_NUM_LOCAL_L4_HANDLES)))  {
    			System_printf ("%s (%s:%d): Found a packet with destination port 0x%04x, valid range is 0x%04x - 0x%04x\n",
    				tfName, __FILE__, __LINE__, port, T5_FIRST_DEST_PORT, T5_FIRST_DEST_PORT + T5_NUM_LOCAL_L4_HANDLES - 1);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}
    		
    		/* Make sure this is the expected packet */
    		eport = commonFifoPopElement (fifo, &nQ);
    		if (nQ <= 0)  {
    			System_printf ("%s (%s:%d): Could not pop an element off of the software tracking fifo (n = %d)\n",
    				tfName, __FILE__, __LINE__, nQ);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}
    		
    		if (eport != port)  {
    			System_printf ("%s (%s:%d): Received packet with destination port 0x%04x, expected 0x%04x\n",
    				tfName, __FILE__, __LINE__, port, eport);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}
    					
    		/* Get the packet index from the destination port */
    		idx = T5_L3HDL_IDX_FROM_PORT(port);
    				
    					
    		if (testCommonComparePktInfo (tfName, t5PktInfo[idx].info, pinfo))  {
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
    		}
            
            /* Verify the replaced MAC header */
            if(memcmp((void*)hd->buffPtr, t5MacRouteHdr, sizeof(t5MacRouteHdr)))
            {
    			System_printf ("%s (%s:%d): MAC header does not match!\n", tfName, __FILE__, __LINE__);
    			testCommonRecycleLBDesc (tencap->tf, hd);
    			return (-1);
            }
    		
    		testCommonRecycleLBDesc (tencap->tf, hd);

            count++;
    	}	
    	
    	
    	/* Since the packets went to the modify PDSP and then back to the QM, a descriptor 
    	 * and linked buffer was required while the packet was in the QM. This will
    	 * be recycled to the default recycle queue */
    	while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
    		
    		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
    		if (hd == NULL)  {
    			System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
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
		
    if(msmcBounceCount != count)
        System_printf("t5RxPkts: receives %d queue bounce packets and %d final packets\n", msmcBounceCount, count);

    return (0);
}

static Cppi_HostDesc *t5GetRxPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    int index;
	
	if (Qmss_getQueueEntryCount(tf->QGen[Q_MATCH]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_MATCH])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			return (NULL);
		}
        
        return (hd);
	}	
		
	if (Qmss_getQueueEntryCount(tf->QGen[Q_NFAIL]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_NFAIL])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			return (NULL);
		}
        
        return (hd);
	}
    
    for (index = Q_EROUTE_FIRST; index <= Q_EROUTE_LAST; index++)
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


/* Search the receive data packet queue for received data packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t5ReceiveGTPUDataPkts (tFramework_t *tf, paTest_t *pat, pktTestInfo_t *pktInfoTbl, int numTblEntries, uint8_t *actualPktCount, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pktTestInfo_t    *tinfo;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;
	int               i, j;
	unsigned int		      chan;
    int               count = 0;
    int               msmcBounceCount=0;
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &msmcBounceCount);
		while ((hd = t5GetRxPkt (tf)) != NULL)  {
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & T5_CMD_SWINFO0_TYPE_MASK) != T5_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & T5_CMD_SWINFO0_ID_MASK;
            count++;
			if (chan <= T5_MAX_GTPU_CHAN)
			  actualPktCount[chan] += 1;
			  
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < numTblEntries; j++)  {
				if (pktInfoTbl[j].idx == chan)  {
					tinfo = &pktInfoTbl[j];
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
		System_printf ("%s (%s:%d): GTPU Error - unable to recover all descriptors with associated buffers\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}
	
    if(msmcBounceCount != count)
        System_printf("t5ReceiveGTPUDataPkts: receives %d queue bounce packets and %d final packets\n", msmcBounceCount, count);

    return (0);

}

/* Search the receive data packet queue for received data packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t5ReceiveNatTDataPkts (tFramework_t *tf, paTest_t *pat, uint8_t *actualPktCount, int expCount)
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
    int               msmcBounceCount=0;
    
    if (!expCount)
    {
        /* Wait for all packets to be processed by PASS even if all packets are expected to be dropped */
        utilCycleDelay (10000);
    }
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (1000);
        testCommonRelayQueueBouncePkts (tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &msmcBounceCount);
		while ((hd = t5GetRxPkt (tf)) != NULL)  {
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & T5_CMD_SWINFO0_TYPE_MASK) != T5_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				return (-1);
			}
			
			chan = *swInfo & T5_CMD_SWINFO0_ID_MASK;
            count++;
			if (chan <= T5_MAX_NAT_T_CHAN)
			  actualPktCount[chan] += 1;
              
            pkt = (uint8_t *)hd->buffPtr;  
			  
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t5NatTPktInfo) / sizeof(pktTestInfo2_t); j++)  {
				if (t5NatTPktInfo[j].info.idx == chan)  {
					tinfo = &t5NatTPktInfo[j + pkt[15]].info;
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
		System_printf ("%s (%s:%d): NAT-T Processing Error - unable to recover all descriptors with associated buffers\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}
	
    if(msmcBounceCount != count)
        System_printf("t2ReceiveNatTDataPkts: receives %d queue bounce packets and %d final packets\n", msmcBounceCount, count);

    return (0);

}


static paTestStatus_t t5VerifyLinkCounters (t5TestEncap_t *tencap, int nL2Handles, int nL3Handles, int nSpiL3Handles)
{
    uint16_t count;
    int i;
    
    for (i = 0; i < nL2Handles; i++)  {
        Pa_getHandleRefCount(t5Encap.tf->passHandle,
                             t5Encap.l2Handles[i].paHandle,
                             &count);
                             
        if (count != (uint16_t)t5Encap.l2Handles[i].linkCnt)
        {
	        System_printf ("%s: (%s:%d) L2 handle[%d]: link count (%d) mismatch. should be %d\n", 
                           tfName, __FILE__, __LINE__, i, count, t5Encap.l2Handles[i].linkCnt);
                           
            System_flush();
	        return (PA_TEST_FAILED);
        }                     
    }
    
    
    for (i = 0; i < nL3Handles; i++)  {
        Pa_getHandleRefCount(t5Encap.tf->passHandle,
                             t5Encap.l3Handles[i].paHandle,
                             &count);
                             
        if (count != (uint16_t)t5Encap.l3Handles[i].linkCnt)
        {
	        System_printf ("%s: (%s:%d) L3 handle[%d]: link count (%d) mismatch. should be %d\n", 
                           tfName, __FILE__, __LINE__, i, count, t5Encap.l3Handles[i].linkCnt);
                           
            System_flush();
	        return (PA_TEST_FAILED);
        }                     
    }

    for (i = 0; i < nSpiL3Handles; i++)  {
        Pa_getHandleRefCount(t5Encap.tf->passHandle,
                             t5Encap.l3SpiHandles[i].paHandle,
                             &count);
                             
        if (count != (uint16_t)t5Encap.l3SpiHandles[i].linkCnt)
        {
	        System_printf ("%s: (%s:%d) L3 handle[%d]: link count (%d) mismatch. should be %d\n", 
                           tfName, __FILE__, __LINE__, i, count, t5Encap.l3SpiHandles[i].linkCnt);
                           
            System_flush();
	        return (PA_TEST_FAILED);
        }                     
    }    

    return (PA_TEST_PASSED);
}

/* 
 * Queue Diversion test supporting functions
 */
 
static paTestStatus_t t5UpdateL4 (t5TestEncap_t *tencap, uint16_t port)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    int             l4Idx = T5_L4HDL_IDX_FROM_PORT(port);
    int             l3Idx = T5_L3HDL_IDX_FROM_PORT(port);
 	//volatile int mdebugWait = 1;
    
    
    matchRoute[0].pCmd = &t5CmdSetCmd1;
    matchRoute[0].queue = (uint16_t) t5Encap.tf->QGen[Q_MATCH2];
    
    t5CmdSetCmd1.params.cmdSetUsrStats.statsIndex = t5L3UsrStats[T5_L3HDL_IDX_FROM_PORT(port)];
			
	cmdReply.replyId  = T5_CMD_SWINFO0_ADD_PORT_ID + l4Idx;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
	
	hd = testCommonAddPort2 (tencap->tf, pa_LUT2_PORT_SIZE_16, port, TRUE, tencap->tf->QGen[Q_MATCH], &matchRoute[0], &tencap->l4Handles[l4Idx].paHandle, 
						     &tencap->l3Handles[l3Idx].paHandle,
                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);
	
							 
							 
	if (hd == NULL)  {
				
		System_printf ("%s: (%s:%d): Failure in common addPort2 command, entry number %d\n", tfName, __FILE__, __LINE__, 0);
		t5Cleanup (tencap, PA_TEST_FAILED);
	}	
    
    /* restore the destination queue */							 
    matchRoute[0].queue = (uint16_t) tencap->tf->QGen[Q_MATCH];
    
	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	tencap->l4Handles[l4Idx].state = T5_HANDLE_PENDING_ACK;
 	paTestL4ExpectedStats.classify2.nPackets += 1;
    #ifdef NSS_GEN2
 	paTestL4ExpectedStats.classify1.nPackets += 1;
    #endif
    
 	utilCycleDelay (600);
 	t5CmdRep (tencap);
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		if (tencap->l4Handles[l4Idx].state == T5_HANDLE_ACTIVE)
            break;
		else
			utilCycleDelay (1000);
            
		t5CmdRep (tencap);
	}
			
	if (i == 100)  {
		System_printf ("%s: (%s:%d):  %d (out of %d) pa_addPort commands were acked\n", tfName, __FILE__, __LINE__, 0, 1);
		return (PA_TEST_FAILED);
	}	
	
	return (PA_TEST_PASSED);
}

static void t5SendDataPackets (t5TestEncap_t *tencap, uint16_t port, uint16_t numPkts, uint16_t startSeqNum)
{
	Cppi_HostDesc  		*hd;
	Qmss_Queue      	q;
	int 				i,j;
    int                 l3Idx = T5_L3HDL_IDX_FROM_PORT(port);
    uint8_t*            pData;
    uint16_t            seqNum = startSeqNum;
    int                 l4Offset = PASAHO_LINFO_READ_L4_OFFSET(t5PktInfo[l3Idx].info);
    
	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
	
	/* Prepare and send packets */
	for (i = 0; i < numPkts; i++)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);
		
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}
	
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(hd), q);
  		
  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(hd), (uint8_t *)t5PktInfo[l3Idx].pkt, (uint32_t)t5PktInfo[l3Idx].pktLen);
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(hd), (uint32_t)t5PktInfo[l3Idx].pktLen);
        Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)(hd), 0);
        
		/* Set the destination TCP/UDP port */
		pData = &t5PktInfo[l3Idx].pkt[l4Offset];
		pData[2] = (port >> 8) & 0xff;
		pData[3] = (port) & 0xff;
        
        /* Set the sequence Number */
        pData[8] = (seqNum >> 8) & 0xFF;
        pData[9] = seqNum & 0xFF;
        seqNum++;
        
        /*
        * Write back the entire cache to make sure that the test packets are updated. 
        * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
        *       the portion of packet which is updated.
        *
        */
#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
        CACHE_wbAllL1d(CACHE_WAIT);
        //CACHE_wbAllL2(CACHE_WAIT);
#endif
#else
        Osal_writeBackCache ((void *)t5PktInfo[l3Idx].pkt, t5PktInfo[l3Idx].pktLen);
#endif
        
		/* Send the data to PDSP0 */
		Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd, t5PktInfo[l3Idx].pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		testCommonIncStats (t5PktInfo[l3Idx].statsMap, &paTestL4ExpectedStats);	
        
        t5UpdateUsrStats(&paTestExpectedUsrStats,  t5UsrStatsLnkTbl,  
                          t5L3UsrStats[l3Idx], 
                          t5PktInfo[l3Idx].pktLen);
                          
        t5UpdateUsrStats(&paTestExpectedUsrStats,  t5UsrStatsLnkTbl,  
                          t5L2UsrStats[T5_L2HDL_IDX_FROM_PORT(port)], 
                          t5PktInfo[l3Idx].pktLen);
        
        /* Wait until the packet is received by PASS */
	    /* Wait for descriptors to return. It is assumed that they are returning in order. Also poll for
	    * received packets */
	    for (j = 0; j < 100; j++)  {
		
		    utilCycleDelay (500);
		    if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) > 0)
			    break;
	    }
	
	    if (j == 100)  {
		    System_printf ("%s (%s:%d): t5SendDataPackets: Timeout waiting for descriptors to PDSP0 to be recycled\n", tfName, __FILE__, __LINE__);
		    t5Cleanup (tencap, PA_TEST_FAILED);
	    }
        
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
			t5Cleanup (tencap, PA_TEST_FAILED);
		}
 		Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
	}
}

static int t5ReceiveDataPacket (t5TestEncap_t *tencap, uint16_t eport, uint16_t numPkts, uint16_t startSeqNum)
{
	Cppi_HostDesc  		*hd;
	pasahoLongInfo_t 	*pinfo;
	uint32_t		     infoLen;
	uint32_t			 flags;
	int					 idx;
	unsigned int		 l4Offset;
	uint8_t				*rxPkt;
	uint16_t			 port;
    uint16_t             eSeqNum = startSeqNum;
    uint16_t             seqNum;
    int                  count = 0;
    int                  i;
    
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (1000);
    
	    /* Look for packets in the rx packet queue, verify that the UDP port found matches the next value in
	    * the fifo, then verify the receive packet information */
	    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_MATCH2]) > 0)  {
		
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH2])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received packet from Q_MATCH2\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }
		
		    /* Get the parse information, make sure there is an L4 offset */
		    if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
			    System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }
		
		    rxPkt = (uint8_t *)hd->buffPtr;
		    flags = PASAHO_LINFO_READ_HDR_BITMASK(pinfo);
		    l4Offset = PASAHO_LINFO_READ_L4_OFFSET(pinfo);

		
		    if ((flags & (PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_TCP)) == 0)  {
			    System_printf ("%s (%s:%d): Found a packet without any L4 info\n", tfName, __FILE__, __LINE__);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }

		    /* Same offset to destination port for both TCP and UDP */
		    port = (rxPkt[l4Offset+2] << 8) | rxPkt[l4Offset+3];
		
		    if ((port < T5_FIRST_DEST_PORT) || (port >= (T5_FIRST_DEST_PORT + T5_NUM_LOCAL_L4_HANDLES)))  {
			    System_printf ("%s (%s:%d): Found a packet with destination port 0x%04x, valid range is 0x%04x - 0x%04x\n",
				    tfName, __FILE__, __LINE__, port, T5_FIRST_DEST_PORT, T5_FIRST_DEST_PORT + T5_NUM_LOCAL_L4_HANDLES - 1);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }
		
		    /* Make sure this is the expected packet */
		    if (eport != port)  {
			    System_printf ("%s (%s:%d): Received packet with destination port 0x%04x, expected 0x%04x\n",
				    tfName, __FILE__, __LINE__, port, eport);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }
        
		    seqNum = (rxPkt[l4Offset+8] << 8) | rxPkt[l4Offset+9];
        
            /* Make sure this is the expected packet */
		    if (eSeqNum != seqNum)  {
			    System_printf ("%s (%s:%d): Received packet with seqNum %d, expected %d\n",
				    tfName, __FILE__, __LINE__, seqNum, eSeqNum);
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }
					
		    /* Get the packet index from the destination port */
		    idx = T5_L3HDL_IDX_FROM_PORT(port);
					
		    if (testCommonComparePktInfo (tfName, t5PktInfo[idx].info, pinfo))  {
			    testCommonRecycleLBDesc (tencap->tf, hd);
			    return (-1);
		    }
        
		    testCommonRecycleLBDesc (tencap->tf, hd);
        
            count++;
            eSeqNum++;
	    }
        
        if(count >= numPkts)
            break;
	}    
    
	if (i == 100)  {
		System_printf ("%s (%s:%d): t5ReceiveDataPacket - unable to receive all packets\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}
    	
	
	/* Since the command response pkt will sent to QMSS and then PDSP5 and then to the QM, a descriptor 
	 * and linked buffer was required while the command response was in the QM. This will
	 * be recycled to the default recycle queue */
	while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
		
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
			return (-1);
		}
		
		testCommonRecycleLBDesc (tencap->tf, hd);
	}

	return (0);
}

#ifdef __LINUX_USER_SPACE
static int setupPktTestInfo2(pktTestInfo2_t* testInfoPkt, int count, char* tfname)
{
	int i,pktSz;
	uint8_t *pkt;
	int returnVal = 0;	

	for (i = 0; i < count; i++)  {
		/* Store the original information */
        pkt   = testInfoPkt[i].info.pkt;
		pktSz = testInfoPkt[i].info.pktLen;
        /* Allocate memory for the Packet buffers from Framework, to control phy2Virt and Virt2Phy */
        testInfoPkt[i].info.pkt = (uint8_t *)fw_memAlloc(pktSz, CACHE_LINESZ);
        if(testInfoPkt[i].info.pkt == NULL) {
  	        printf ("%s: memAlloc failed for pkt %d\n", tfname, i);
  	        returnVal = -1;
        }
		/* Restore the original pkt information in the allocated memory */
        memcpy(testInfoPkt[i].info.pkt, pkt, pktSz);
    }

	return (returnVal);
}
#else
static int setupPktTestInfo2(pktTestInfo2_t* testInfoPkt, int count, char* tfname)
{
  return 0;
}

#endif

void t5ClearL2State(t5EthSetup_t* ethSetup, int num)
{
  int i;

  for ( i = 0; i < num ; i++ )
  {
    ethSetup->acked = FALSE;
    ethSetup++;
  }
}

void t5ClearL3State(t5IpSetup_t* ipSetup, int num)
{
  int i;

  for ( i = 0; i < num ; i++ )
  {
    ipSetup->acked = FALSE;
    ipSetup++;
  }
}

void t5ClearSpiL3State(t5SpiIpSetup_t* spiIpSetup, int num)
{
  int i;

  for ( i = 0; i < num ; i++ )
  {
    spiIpSetup->acked = FALSE;
    spiIpSetup++;
  }
}

#ifdef __LINUX_USER_SPACE
void* paTestL4Routing (void *args)
{
 	void  *a0 = (void *)((paTestArgs_t *)args)->tf;
 	void  *a1 = (void *)((paTestArgs_t *)args)->pat;
#else
void paTestL4Routing (UArg a0, UArg a1)
{	
#endif
 	int				i, j, k, n;
 	paTestStatus_t  newStatus = PA_TEST_PASSED;
 	uint8_t			*pdata;
    Cppi_HostDesc  *hd[sizeof(t5GTPUPktInfo) / sizeof(pktTestInfo_t)];
 	unsigned int			fifoData[(2*T5_NUM_LOCAL_L3_HANDLES)+1];
 	pauFifo_t       fifo =  { 0, 0, (2*T5_NUM_LOCAL_L3_HANDLES)+1, NULL };
 	uint8_t			expectedPktCount[T5_MAX_GTPU_CHAN];
 	uint8_t			actualPktCount[T5_MAX_GTPU_CHAN];
    int             numUsrStatsEntries;
    int             bounceCount = 0;
    
 	//volatile int mdebugWait = 1;
    #ifndef  SIMULATOR_SUPPORT
    int32_t           result;
    #endif
 	
 	/* Initialize the test state */
    numUsrStatsEntries = sizeof(t5UsrStatsSetup)/sizeof(pauUsrStatsSetup_t);
    
    t5NumUsrStats = 0;
	fifo.data = fifoData;
 	memset (&t5Encap, 0, sizeof(t5Encap));
 	t5Encap.tf  = (tFramework_t *)a0;
 	t5Encap.pat = (paTest_t *)a1;
 	for (i = 0; i < T5_NUM_LOCAL_L4_HANDLES; i++)
 		t5Encap.l4Handles[i].state = T5_HANDLE_UNCONFIGURED; 		
 	for (i = 0; i < T5_NUM_LOCAL_L3_HANDLES; i++)
 		t5Encap.l3Handles[i].state = T5_HANDLE_UNCONFIGURED;	
 	for (i = 0; i < T5_NUM_LOCAL_L2_HANDLES; i++)
 		t5Encap.l2Handles[i].state = T5_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T5_NUM_LOCAL_L3_HANDLES; i++)
 		t5Encap.hd[i] = NULL;	
 		
    i = setupPktTestInfo(t5PktInfo, (sizeof(t5PktInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
        t5Encap.pat->testStatus = PA_TEST_FAILED;		
   	    Task_exit();
    }
	
    i = setupPktTestInfo(t5GTPUPktInfo, (sizeof(t5GTPUPktInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
        t5Encap.pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

    i = setupPktTestInfo(t5GTPUPktInfo1, (sizeof(t5GTPUPktInfo1) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
        t5Encap.pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

    i = setupPktTestInfo2(t5NatTPktInfo, (sizeof(t5NatTPktInfo) / sizeof(pktTestInfo2_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
        t5Encap.pat->testStatus = PA_TEST_FAILED;		
   	    Task_exit();
    }

    i = setupPktTestInfo(t5GTPUPktInfo2, (sizeof(t5GTPUPktInfo2) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
        t5Encap.pat->testStatus = PA_TEST_FAILED;	
   	    Task_exit();
    }

    /* Runtime initial values */
    matchRoute[0].queue = (uint16_t) t5Encap.tf->QGen[Q_MATCH];
	matchRoute[0].flowId= t5Encap.tf->tfFlowNum[0];
    nfailRoute.queue    = (uint16_t) t5Encap.tf->QGen[Q_NFAIL];
    cmdReply.queue      = (uint16_t) t5Encap.tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId     = t5Encap.tf->tfFlowNum[0];
    t5QueueDivertCfg.destFlowId = t5Encap.tf->tfFlowNum[0];
    
    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL4ExpectedStats, 0, sizeof(paTestL4ExpectedStats));
    memset (&paTestExpectedUsrStats, 0, sizeof(paTestExpectedUsrStats));
    memset (expectedPktCount, 0, sizeof(expectedPktCount));
    memset (actualPktCount, 0, sizeof(actualPktCount));
    
    /* Initialize the user-defined statistics link table */
    t5InitUsrStatsLnkTbl (t5UsrStatsLnkTbl);
    
    /* 
     * Local initialization 
     */
    t5VerifyPktErrCmd1.queue = t5Encap.tf->QGen[Q_MATCH]; 
    t5VerifyPktErrCmd1.flowId= t5Encap.tf->tfFlowNum[0]; 
    t5CmdSet1[0].params.patch  = t5PatchCmd1; 
    t5CmdSet1[1].params.verifyPktErr = t5VerifyPktErrCmd1; 
    t5CmdSetCmd1.params.cmdSetUsrStats.setIndex = T5_CMDSET_1_INDEX;  
    t5CmdSet2[1].params.patch  = t5PatchCmd2;  
    t5CmdSetCmd2.params.cmdSet.index = T5_CMDSET_2_INDEX; 
    t5CmdSet3[0].params.patch  = t5PatchCmd3;  
    t5CmdSetCmd3.params.cmdSet.index = T5_CMDSET_3_INDEX;  
     
    
    /* Global Configuration */
	newStatus = t5GlobalConfiguration (&t5Encap);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
    
    /* Exception Route Configurations */
	newStatus = t5ExceptionRoutes (&t5Encap);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
        
    /* Command Set Configuration */
	newStatus = t5CmdSetConfiguration (&t5Encap);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
        
    /* Usr Stats configuration */
    newStatus = t5ConfigUsrStats (&t5Encap, numUsrStatsEntries, t5UsrStatsSetup, FALSE);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
        
    /* Enable QMSS queue diversion support */
#ifndef  SIMULATOR_SUPPORT
    /* Program the accumulator */
    if ((result = Qmss_programDiversionQueue (Qmss_PdspId_PDSP1, (Qmss_QueueHnd) Q_DIVERT_MONITORING, t5Encap.tf->QPaTx[TF_PA_Q_POST])) != QMSS_ACC_SOK)
    {
        System_printf ("Error Programming accumulator for queue diversion operation with error code : %d\n", result);
		t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* No return */
    }
#endif    
	/* Burst in the L2 configuraton */
    t5ClearL2State(&t5EthSetup[0], T5_NUM_LOCAL_L2_HANDLES);
	newStatus = t5OpenL2 (&t5Encap, t5EthSetup, T5_NUM_LOCAL_L2_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
	
	/* Burst in the L3 configuration */
    t5ClearL3State(&t5IpSetup[0], T5_NUM_LOCAL_L3_HANDLES);
	newStatus = t5OpenL3 (&t5Encap, t5IpSetup, T5_NUM_LOCAL_L3_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);

	/* Burst in the SPI with Link L3 configuration */
    t5ClearSpiL3State(&t5SpiIpSetup[0], T5_NUM_LOCAL_SPI_L3_HANDLES);
	newStatus = t5OpenSpiL3 (&t5Encap, t5SpiIpSetup, T5_NUM_LOCAL_SPI_L3_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  
        
	/* Completely fill the L4 table. There is a one to one mapping of UDP destination port
	 * values to the IP entries in the lookup table. */
	newStatus = t5GenericOpenL4 (&t5Encap, T5_FIRST_DEST_PORT, T5_NUM_LOCAL_L3_HANDLES, T5_NUM_LOCAL_L4_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);
        
	/* Completely fill the L4 table. There is a one to one mapping of GTPU Tunnel ID
	 * values to the IP entries in the lookup table. */
    #ifdef NSS_GEN2 
	newStatus = t5GenericOpenGTPU (&t5Encap, T5_FIRST_GTPU_TEID, T5_FIRST_GTPU_TEID, T5_NUM_LOCAL_L5_HANDLES, TRUE);
    #else
	newStatus = t5GenericOpenGTPU (&t5Encap, T5_FIRST_GTPU_TEID, T5_FIRST_GTPU_TEID, T5_NUM_LOCAL_L5_HANDLES, FALSE);
    #endif
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);
        
    /* Verify the reference counters */    
    newStatus = t5VerifyLinkCounters(&t5Encap, T5_NUM_LOCAL_L2_HANDLES, T5_NUM_LOCAL_L3_HANDLES, T5_NUM_LOCAL_SPI_L3_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);
        
	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t5Encap.tf, t5Encap.pat, tfName, &paTestL4ExpectedStats, t5Encap.tf->QLinkedBuf1, 
	                                   t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);  /* No return */
    }    
        
	/* Create one descriptor for each IP table entry. The descriptor will remain
	 * with the packet throughout the test */
	t5FormPktDataDescriptors (&t5Encap, T5_NUM_LOCAL_L3_HANDLES);
       
	/* Send the packets. Use the modify PDSP to generate the TCP/UDP checksums. Send 
	 * each packet type to each of the opened TCP/UDP ports */
	 
	for (k = 0; k < T5_NUM_PACKET_ITERATIONS; k++)  {
	 
		for (j = 0; j < T5_NUM_LOCAL_L4_HANDLES; j += T5_NUM_LOCAL_L3_HANDLES)  {
		
			for (i = n = 0; (i < T5_NUM_LOCAL_L3_HANDLES) && ((i+j) < T5_NUM_LOCAL_L4_HANDLES); i++)  {
							
				
				/* Set the destination TCP/UDP port */
				pdata = t5PktInfo[i].pkt;
				pdata[(PASAHO_LINFO_READ_L4_OFFSET(t5PktInfo[i].info)) + 2] = ((T5_FIRST_DEST_PORT + i + j) >> 8) & 0xff;
				pdata[(PASAHO_LINFO_READ_L4_OFFSET(t5PktInfo[i].info)) + 3] = (T5_FIRST_DEST_PORT + i + j) & 0xff;
                
                /*
                * Write back the entire cache to make sure that the test packets are updated. 
                * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
                *       the portion of packet which is updated.
                *
                */
#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
                CACHE_wbAllL1d(CACHE_WAIT);
                //CACHE_wbAllL2(CACHE_WAIT);
#endif
#else
                Osal_writeBackCache ((void *)t5PktInfo[i].pkt, t5PktInfo[i].pktLen);
#endif
                
				/* Send the data to the modify PDSP */
                //if(j == 12)mdebugHaltPdsp(4);
				Qmss_queuePush (t5Encap.tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)t5Encap.hd[i], t5PktInfo[i].pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
                //Qmss_queuePush (t5Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)t5Encap.hd[i], t5PktInfo[i].pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
                //if(j == 12)while (mdebugWait);

                /* Allow some time for the packet to be processed */
				utilCycleDelay (1000);                
				n += 1;
				t5Encap.hd[i] = NULL;
				
				testCommonIncStats (t5PktInfo[i].statsMap, &paTestL4ExpectedStats);	
				
				if (t5PktInfo[i].idx & T5_PKTINFO_IDX_MATCH_FLAG)  {
				
					if (commonFifoPushElement (&fifo, (unsigned int)(T5_FIRST_DEST_PORT + i + j)) < 0)  {
						System_printf ("%s (%s:%d): Test failed - fifo is full\n", tfName, __FILE__, __LINE__);
						t5Cleanup (&t5Encap, PA_TEST_FAILED);
					}
				}
                
                t5UpdateUsrStats(&paTestExpectedUsrStats,  t5UsrStatsLnkTbl,  
                                  t5L3UsrStats[T5_L3HDL_IDX_FROM_PORT((T5_FIRST_DEST_PORT + i + j))], 
                                  t5PktInfo[i].pktLen);
                                  
                t5UpdateUsrStats(&paTestExpectedUsrStats,  t5UsrStatsLnkTbl,  
                                  t5L2UsrStats[T5_L2HDL_IDX_FROM_PORT((T5_FIRST_DEST_PORT + i + j))], 
                                  t5PktInfo[i].pktLen);
			}

			
			/* Wait for descriptors to return. It is assumed that they are returning in order. Also poll for
			 * received packets */
			for (i = 0; i < 100; i++)  {

				utilCycleDelay (1000);
				if ((Qmss_getQueueEntryCount (t5Encap.tf->QGen[Q_DPKT_RECYCLE])) >= n)
					break;
			}
			
			if (i == 100)  {
				System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
				t5Cleanup (&t5Encap, PA_TEST_FAILED);
			}
            
			/* Check for received packets */
			if (t5RxPkts (&t5Encap, &fifo))
				t5Cleanup (&t5Encap, PA_TEST_FAILED);

			/* Recycle the descriptors */
			for (i = 0; i < n; i++) {
				t5Encap.hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (t5Encap.tf->QGen[Q_DPKT_RECYCLE])) & ~15);
				if (t5Encap.hd[i] == NULL)  {
					System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
					t5Cleanup (&t5Encap, PA_TEST_FAILED);
				}
			}
		}
	}
	
	/* Verify and clear the stats */
	newStatus =  testCommonCheckUsrStatsList (t5Encap.tf, t5Encap.pat, tfName, &paTestExpectedUsrStats, T5_NUM_64B_USR_STATS, t5NumUsrStats, t5UsrStatsTbl,
                                              t5Encap.tf->QLinkedBuf1, t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckUsrStatsList Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);
    }
    
	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t5Encap.tf, t5Encap.pat, tfName, &paTestL4ExpectedStats, t5Encap.tf->QLinkedBuf1, 
	                                   t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);  /* No return */
    }    
    
    /* LUT2 Queue Diversion Test */
    t5SendDataPackets(&t5Encap, T5_FIRST_DEST_PORT, 10, 0);

    /* Give some time for all the packets to get through the system */
    utilCycleDelay (5000);

    testCommonRelayQueueBouncePkts (t5Encap.tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &bounceCount);

    if(bounceCount != 10)
        System_printf("LUT2 Queue Diversion Test (Phase 1): receives %d queue bounce packets\n", bounceCount);

    newStatus = t5UpdateL4(&t5Encap, T5_FIRST_DEST_PORT);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);

#ifndef  SIMULATOR_SUPPORT
    /* All packets should have diverted from Q_MATCH to Q_MATCH2, otherwise test fails */
    if (Qmss_getQueueEntryCount(t5Encap.tf->QGen[Q_MATCH]) > 0)
    {
       System_printf(" Did not divert all packets from Queue diversion \n");
       System_flush();
       t5Cleanup(&t5Encap, PA_TEST_FAILED);
    }
#endif
    t5SendDataPackets(&t5Encap, T5_FIRST_DEST_PORT, 10, 10);
    
	/* Give some time for all the packets to get through the system */
	utilCycleDelay (4000);
    
    bounceCount = 0;
    testCommonRelayQueueBouncePkts (t5Encap.tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, NULL, &bounceCount);
    if(bounceCount != 10)
        System_printf("LUT2 Queue Diversion Test (Phase 2): receives %d queue bounce packets\n", bounceCount);

    if (t5ReceiveDataPacket(&t5Encap, T5_FIRST_DEST_PORT, 20, 0))
    {
		System_printf ("%s (%s:%d): L4 Queue Diversion test fails\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, PA_TEST_FAILED);
    }
    
	/* Verify and clear the User stats */
	newStatus =  testCommonCheckUsrStatsList (t5Encap.tf, t5Encap.pat, tfName, &paTestExpectedUsrStats, T5_NUM_64B_USR_STATS, t5NumUsrStats, t5UsrStatsTbl,
                                              t5Encap.tf->QLinkedBuf1, t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
    
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): UDP: testCommonCheckUsrStatsList Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);
    }
    
	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t5Encap.tf, t5Encap.pat, tfName, &paTestL4ExpectedStats, t5Encap.tf->QLinkedBuf1, 
	                                   t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);  /* No return */
    }    
    
    
 	/* Delete active L4 handles */
    if (t5DeleteL4 (&t5Encap) != PA_TEST_PASSED)
    {
		System_printf ("%s (%s:%d): L4 Packet reception complete, but L4 deletion fails\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, PA_TEST_FAILED);
    }
    
	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t5Encap.tf, t5Encap.pat, tfName, &paTestL4ExpectedStats, t5Encap.tf->QLinkedBuf1, 
	                                   t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);  /* No return */
    }    
    
    /* NAT-T packet testing prior to NAT-T configuration: all packets should be dropped due to LUT2 match failure*/
	/* Run packets through the system. the complete set of packets is run through three times. */
	for (j = 0; j < T5_NUM_PACKET_ITERATIONS; j++)   {
		for (i = 0; i < sizeof(t5NatTPktInfo) / sizeof(pktTestInfo2_t); i++ )  {
		
			hd[i] = t5FormNatTDataPacket (t5Encap.tf, t5Encap.pat, i);
			
			if (hd[i] == NULL)  {
			    System_printf ("%s (%s:%d): T5 NatT packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
                t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
                break;
			}
			
			/* Increment any expected stats */
			testCommonIncStats (t5NatTPktInfo[i].info.statsMap, &paTestL4ExpectedStats);	
 		    paTestL4ExpectedStats.classify2.nSilentDiscard += 1;
            
            /* Update User Statistics */
            t5UpdateUsrStats(&paTestExpectedUsrStats,  t5UsrStatsLnkTbl, t5L2UsrStats[0], t5NatTPktInfo[i].info.pktLen);  
			
		}
			
		for (i = 0; i < sizeof(t5NatTPktInfo) / sizeof(pktTestInfo2_t); i++)
        {
			Qmss_queuePush (t5Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[i], hd[i]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
	        utilCycleDelay (5000);
            
        }    
			
		if (t5ReceiveNatTDataPkts (t5Encap.tf, t5Encap.pat, actualPktCount, 0))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t5ReceiveNatTDataPkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
            t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
			break;
        }    
	}
        
    /* NAT-T Configuration */
	newStatus = t5NatTConfiguration (&t5Encap, 1);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
        
        
    /* NAT-T packet testing after NAT-T configuration */
	/* Run packets through the system. the complete set of packets is run through three times. */
	for (j = 0; j < T5_NUM_PACKET_ITERATIONS; j++)   {
        int count = 0;
		for (i = 0; i < sizeof(t5NatTPktInfo) / sizeof(pktTestInfo2_t); i++ )  {
		
			hd[i] = t5FormNatTDataPacket (t5Encap.tf, t5Encap.pat, i);
			
			if (hd[i] == NULL)  {
			    System_printf ("%s (%s:%d): T5 NatT packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
                t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
                break;
			}
			
			/* Inc the count if the packet is passed back to the host */
  			if (t5NatTPktInfo[i].info.idx < T5_MAX_NAT_T_CHAN)
            {
                if (t5NatTPktInfo[i].info.idx == T5_NAT_T_KEEPALIVE_PKT_INDEX)
                {
                    /* Update User Statistics */
                    t5UpdateUsrStats(&paTestExpectedUsrStats, t5UsrStatsLnkTbl, t5NatTKeepAliveUsrStats, t5NatTPktInfo[i].info.pktLen);  
                }
            
                if (t5NatTPktInfo[i].info.idx == T5_NAT_T_FAIL_PKT_INDEX)
                {
                    /* Update User Statistics */
                    t5UpdateUsrStats(&paTestExpectedUsrStats, t5UsrStatsLnkTbl, t5NatTErrUsrStats, t5NatTPktInfo[i].info.pktLen);  
 		            paTestL4ExpectedStats.classify2.nSilentDiscard += 1;
                }
                else
                {
	  			    expectedPktCount[t5NatTPktInfo[i].info.idx] += 1;
                    count ++;
                }
                
            }
            
			/* Increment any expected stats */
			testCommonIncStats (t5NatTPktInfo[i].info.statsMap, &paTestL4ExpectedStats);
			testCommonIncStats (t5NatTPktInfo[i].statsMap, &paTestL4ExpectedStats);	
      
            /* Update User Statistics */
            t5UpdateUsrStats(&paTestExpectedUsrStats,  t5UsrStatsLnkTbl, t5L2UsrStats[0], t5NatTPktInfo[i].info.pktLen);  
			
		}
			
		for (i = 0; i < sizeof(t5NatTPktInfo) / sizeof(pktTestInfo2_t); i++)
        {
            //mdebugHaltPdsp(4);
			Qmss_queuePush (t5Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[i], hd[i]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
	        utilCycleDelay (5000);
            //while (mdebugWait);
        }    
			
		if (t5ReceiveNatTDataPkts (t5Encap.tf, t5Encap.pat, actualPktCount, count))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t5ReceiveNatTDataPkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
            t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
			break;
        }    
	}
	
	/* Verify that the expected and actual received packet counts match */
	for (i = 0; i < T5_MAX_NAT_T_CHAN; i++)  {
		if (expectedPktCount[i] != actualPktCount[i])  {
			System_printf ("%s (%s:%d): Packet count mismatch for entry %d - expected %d, found %d\n", tfName,
						   __FILE__, __LINE__, i, expectedPktCount[i], actualPktCount[i]);
            System_flush();               
		    t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
		}
	}
    
	/* Verify and clear the stats */
	newStatus =  testCommonCheckUsrStatsList (t5Encap.tf, t5Encap.pat, tfName, &paTestExpectedUsrStats, T5_NUM_64B_USR_STATS, t5NumUsrStats, t5UsrStatsTbl,
                                              t5Encap.tf->QLinkedBuf1, t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckUsrStatsList Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);
    }
    
	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t5Encap.tf, t5Encap.pat, tfName, &paTestL4ExpectedStats, t5Encap.tf->QLinkedBuf1, 
	                                   t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);  /* No return */
    }    
    
    /* Clear all Usr Stats Link */
    newStatus = testCommonUsrStatsConfigReset (t5Encap.tf, t5Encap.pat, tfName, numUsrStatsEntries, t5UsrStatsSetup,
                                               T5_CMD_SWINFO0_USR_STATS_CFG_ID, t5Encap.tf->QLinkedBuf3, t5Encap.tf->QGen[Q_CMD_RECYCLE], 
                                               t5Encap.tf->QGen[Q_CMD_REPLY]);
    
	if (newStatus == PA_TEST_FAILED)
    {
	    System_printf ("%s (%s:%d): testCommonUsrStatsConfigReset Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, newStatus);  /* No return */
    }   
    
    /* GTPU packet testing */
	/* Run packets through the system. the complete set of packets is run through multiple times. */
	for (j = 0; j < T5_NUM_PACKET_ITERATIONS; j++)   {
        int count = 0;
		for (i = 0; i < sizeof(t5GTPUPktInfo) / sizeof(pktTestInfo_t); i++ )  {
		
			hd[i] = t5FormGTPUDataPacket (t5Encap.tf, t5Encap.pat, &t5GTPUPktInfo[i]);
			
			if (hd[i] == NULL)  {
			    System_printf ("%s (%s:%d): T5 GTPU packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
                t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
                break;
			}
			
			/* Inc the count if the packet is passed back to the host */
  			if (t5GTPUPktInfo[i].idx < T5_MAX_GTPU_CHAN)
            {
	  			expectedPktCount[t5GTPUPktInfo[i].idx] += 1;
                count ++;
            }
			/* Increment any expected stats */
			testCommonIncStats (t5GTPUPktInfo[i].statsMap, &paTestL4ExpectedStats);	
			
		}
			
		for (i = 0; i < sizeof(t5GTPUPktInfo) / sizeof(pktTestInfo_t); i++)
        {
            //mdebugHaltPdsp(4);
			Qmss_queuePush (t5Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[i], hd[i]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
	        utilCycleDelay (5000);
            //while (mdebugWait);
        }    
				
		if (t5ReceiveGTPUDataPkts (t5Encap.tf, t5Encap.pat, t5GTPUPktInfo, sizeof(t5GTPUPktInfo)/sizeof(pktTestInfo_t), actualPktCount, count))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t5ReceiveGTPUDataPkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
            t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
			break;
        }    
	}
	
	
	/* Verify that the expected and actual received packet counts match */
	for (i = 0; i < T5_MAX_GTPU_CHAN; i++)  {
		if (expectedPktCount[i] != actualPktCount[i])  {
			System_printf ("%s (%s:%d): Packet count mismatch for entry %d - expected %d, found %d\n", tfName,
						   __FILE__, __LINE__, i, expectedPktCount[i], actualPktCount[i]);
            System_flush();               
		    t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
		}
	}
	
	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t5Encap.tf, t5Encap.pat, tfName, &paTestL4ExpectedStats, t5Encap.tf->QLinkedBuf1, 
	                       t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
                           
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
    }    

  /* GTPU Configuration (Route End Marker message same as G-PDU) */
  /* Note: For NetCP1.5 device, the GTPU link is always enabled, this congiuration will be ignored by PASS */
	newStatus = t5GtpuConfiguration (&t5Encap, 0, 1);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
        
  /* GTPU packet testing2 */
	/* Run packets through the system. the complete set of packets is run through three times. */
	for (j = 0; j < T5_NUM_PACKET_ITERATIONS; j++)   {
        int count = 0;
		for (i = 0; i < sizeof(t5GTPUPktInfo1) / sizeof(pktTestInfo_t); i++ )  {
		
			hd[i] = t5FormGTPUDataPacket (t5Encap.tf, t5Encap.pat, &t5GTPUPktInfo1[i]);
			
			if (hd[i] == NULL)  {
			    System_printf ("%s (%s:%d): T5 GTPU packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
                t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
                break;
			}
			
			/* Inc the count if the packet is passed back to the host */
  			if (t5GTPUPktInfo1[i].idx < T5_MAX_GTPU_CHAN)
            {
	  			expectedPktCount[t5GTPUPktInfo1[i].idx] += 1;
                count ++;
            }
			/* Increment any expected stats */
			testCommonIncStats (t5GTPUPktInfo2[i].statsMap, &paTestL4ExpectedStats);	
			
		}
			
		for (i = 0; i < sizeof(t5GTPUPktInfo1) / sizeof(pktTestInfo_t); i++)
        {
            //mdebugHaltPdsp(4);
			Qmss_queuePush (t5Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[i], hd[i]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
	        utilCycleDelay (5000);            
            //while (mdebugWait);
        }    
				
		if (t5ReceiveGTPUDataPkts (t5Encap.tf, t5Encap.pat, t5GTPUPktInfo1, sizeof(t5GTPUPktInfo1)/sizeof(pktTestInfo_t), actualPktCount, count))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t5ReceiveGTPUDataPkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
            t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
			break;
        }    
	}

   	/* Delete active L5 handles */
    #ifdef NSS_GEN2
    if (t5DeleteL5 (&t5Encap, TRUE) != PA_TEST_PASSED)
    #else
    if (t5DeleteL5 (&t5Encap, FALSE) != PA_TEST_PASSED)
    #endif
    {
		System_printf ("%s (%s:%d): L5 Packet reception #1 complete, but L5 deletion fails\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, PA_TEST_FAILED);
    }
    
    /* GTPU Configuration (Enable Link) */
    /* Note: For NetCP1.5 device, the GTPU link is always enabled, this congiuration will be ignored by PASS */
	newStatus = t5GtpuConfiguration (&t5Encap, 1, 0);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
        
	/* Completely fill the L4 table. There is a one to one mapping of GTPU Tunnel ID
	 * values to the IP entries in the lookup table. */
	newStatus = t5GenericOpenGTPU (&t5Encap, T5_FIRST_GTPU_TEID2, T5_FIRST_GTPU_TEID2, T5_NUM_LOCAL_L5_HANDLES, TRUE);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);
        
    /* Verify the reference counters */    
    newStatus = t5VerifyLinkCounters(&t5Encap, T5_NUM_LOCAL_L2_HANDLES, T5_NUM_LOCAL_L3_HANDLES, T5_NUM_LOCAL_SPI_L3_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);
        
    /* GTPU packet testing2 */
	/* Run packets through the system. the complete set of packets is run through three times. */
	for (j = 0; j < T5_NUM_PACKET_ITERATIONS; j++)   {
        int count = 0;
		for (i = 0; i < sizeof(t5GTPUPktInfo2) / sizeof(pktTestInfo_t); i++ )  {
		
			hd[i] = t5FormGTPUDataPacket (t5Encap.tf, t5Encap.pat, &t5GTPUPktInfo2[i]);
			
			if (hd[i] == NULL)  {
			    System_printf ("%s (%s:%d): T5 GTPU packet sent:  run out of buffers\n", tfName, __FILE__, __LINE__);
                System_flush();  
                t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
                break;
			}
			
			/* Inc the count if the packet is passed back to the host */
  			if (t5GTPUPktInfo2[i].idx < T5_MAX_GTPU_CHAN)
            {
	  			expectedPktCount[t5GTPUPktInfo2[i].idx] += 1;
                count ++;
            }
			/* Increment any expected stats */
			testCommonIncStats (t5GTPUPktInfo2[i].statsMap, &paTestL4ExpectedStats);	
			
		}
			
		for (i = 0; i < sizeof(t5GTPUPktInfo2) / sizeof(pktTestInfo_t); i++)
        {
            //mdebugHaltPdsp(4);
			Qmss_queuePush (t5Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[i], hd[i]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
            //while (mdebugWait);
        }    
				
		if (t5ReceiveGTPUDataPkts (t5Encap.tf, t5Encap.pat, t5GTPUPktInfo2, sizeof(t5GTPUPktInfo2)/sizeof(pktTestInfo_t), actualPktCount, count))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t5ReceiveGTPUDataPkts timeout\n", tfName, __FILE__, __LINE__);
            System_flush();  
            t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
			break;
        }    
	}
	
	/* Verify that the expected and actual received packet counts match */
	for (i = 0; i < T5_MAX_GTPU_CHAN; i++)  {
		if (expectedPktCount[i] != actualPktCount[i])  {
			System_printf ("%s (%s:%d): Packet count mismatch for entry %d - expected %d, found %d\n", tfName,
						   __FILE__, __LINE__, i, expectedPktCount[i], actualPktCount[i]);
            System_flush();               
		    t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
		}
	}
        
 	/* Delete active L5 handles */
    if (t5DeleteL5 (&t5Encap, TRUE) != PA_TEST_PASSED)
    {
		System_printf ("%s (%s:%d): L5 Packet reception #2 complete, but L5 deletion fails\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, PA_TEST_FAILED);
    }
        
    /* GTPU Configuration (Disable Link, Disable Route of msg254 same as msg255) */
	newStatus = t5GtpuConfiguration (&t5Encap, 0, 0);
	if (newStatus == PA_TEST_FAILED)
		t5Cleanup (&t5Encap, newStatus);  /* No return */
        
	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t5Encap.tf, t5Encap.pat, tfName, &paTestL4ExpectedStats, t5Encap.tf->QLinkedBuf1, 
	                                   t5Encap.tf->QGen[Q_CMD_RECYCLE], t5Encap.tf->QGen[Q_CMD_REPLY], TRUE);
                           
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t5Cleanup (&t5Encap, PA_TEST_FAILED);  /* no return */
    }    
    
	/* No return from cleanup */
	t5Cleanup (&t5Encap, newStatus);
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
 		
}
 
