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

/* IPv4 Fragmentation and PASS-assisted IP or RA-based Reassembly test
 * This test tests the LLD and firmware the ability to perform IPv4 fragmentation
 * on egress packets per IP fragmentation command.
 * In PASS-assisted IP Reassembly test, it also tests the LLD and firmware the ability to
 * perform the related system configuration and  the ability to detect and forward the IP
 * fragments and packets of any active traffic flow to host. An IP reassembly sample code
 * is used to perform IP reassembly and forward the reassembled packets back to PASS for
 * further classification.
 * In RA-based IP Reassembly test, it will test the LLD and firmware the ability to configure
 * the RA module and PASS RA path to forward both outer IP and inner IP packets to the
 * RA module. The RA module will forward the non-fragmented and reassembled IP packets back
 * to PASS for further classification without host intervention.
 *
 * This test invokes the following LLD APIs:
 *  - Pa_addMac
 *  - Pa_addIp
 *  - Pa_addPort
 *  - Pa_delL4Handle
 *  - Pa_delHandle
 *  - Pa_control
 *  - Pa_forwardResult
 *  - Pa_allocUsrStats
 *  - Pa_requestStatsList
 *  - Pa_freeUsrStats
 *  - Pa_formatStatsReply
 *  - Pa_formatUsrStatsReply
 *  - Pa_queryRaStats
 *  - Pa_formatTxCmd
 *
 * The test procedure is described below:
 *  - Call Pa_control to enable the PASS-assisted IP Reassembly and/or RA-based IP Reassembly support
 *  - Call Pa_addMac, Pa_addIp and Pa_addPort to setup the receiving paths
 *  - Send variable-length packets to PASS command queue for UDP checksum calculation
 *    and IP fragmentation if the payload size is larger than the desired MTU size.
 *  - Both fragmented and non-fragmented packets will be delivered to PASS input queue for
 *    classification
 *  - If the RA-Based IP Reassembly is disabled, the IP fragments and the non-fragmented packet within
 *    an active traffic flow are detected and delivered to the specified host queue for reassembly
 *  - Both reassembled and pass-through packets are sent back to the corresponding PDSP for
 *    continuous parsing and classification
 *  - If the RA-based IP Reassembly is enabled, all the IP packets and fragments are delivered to the
 *    RA module and reassembled IP packets are sent back to PASS for further classification. Therefore,
 *    two steps above will be skipped.
 *  - Packets are delivered to the matching queue upon the final UDP lookup
 *
 * This test has the following sub-tests
 *   Tx Command (IP Fragmentation, Checksum calculation and Routing):
 *   - Test the ability to format a set of tx commands
 *   - Test the firmware ability to calculate and verify IPv4 and UDP checksum
 * 	 - Test the firmware ability to perform IPv4 fragmentation
 *   - Test the firmware ability to route the fragments and packets to their destination queues or RA
 *   PASS-assisted IPv4 Reassembly
 *   - Test the ability to configure PASS-assisted IP Reassembly
 *   - Test the ability to configure RA-based IP Reassembly
 *   - Test the firmware for the ability to detect IP fragments
 *   - Test the firmware for the ability to allocate and free traffic flows
 *   - Test the firmware for the ability to identify active traffic flow per packet and forward the packet
 *     to the configured host queue
 *   - Test the firmware for the ability to process the reassembled and pass-through packets
 */
 #include "reassemLib.h"

 static char *tfName = "paTestIPv4FragReassem";

 #ifdef NSS_GEN2
 #define PA_USE_HW_RA               /* Use Hradware Reassembly Engine */
 #endif

 #define T10_NUM_PACKET_GROUPS	10 	/* Number of packet groups where each group contains packets with specified payload size */

 /* General purpose queue usage */
 #define Q_CMD_RECYCLE		  0		/* Command descriptors/buffers recycled here after sent to PA */
 #define Q_CMD_REPLY  		  1		/* Replies from PA routed here */
 #define Q_MATCH		  	  2		/* Packets from PA which match a lookup criteria */
 #define Q_NFAIL		      3		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
 #define Q_PARSE_ERR		  4		/* Packets which resulted in a parse error */
 #define Q_DPKT_RECYCLE		  5		/* Data packet recycle queue */
 #define Q_IP_REASSM1		  6		/* IP Reassembly Input Queue 1 (Outer IP) */
 #define Q_IP_REASSM2		  7		/* IP Reassembly Input Queue 2 (Inner IP) */

 #define Q_IP_FRAG           13     /* IP Fragmentation */
 #define Q_BOUNCE_DDR        21     /* Queue Bounce DDR queue */
 #define Q_BOUNCE_MSMC       22     /* Queue Bounce MSMC queue */

#include "test10pkts.h"

/* The number of PA L2 and L3 handles maintained by this test */
#define T10_NUM_LOCAL_L2_HANDLES   			(sizeof(t10EthSetup)/sizeof(t10EthSetup_t))
#define T10_NUM_LOCAL_L3_HANDLES			(sizeof(t10IpSetup)/sizeof(t10IpSetup_t))
#define T10_NUM_LOCAL_L4_HANDLES	  		(sizeof(t10UdpSetup)/sizeof(t10UdpSetup_t))
#define T10_NUM_GEN_CONFIGS                  10     /* Maxmium number of general configuration commands */

 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T10_HANDLE_UNCONFIGURED = 0,
	T10_HANDLE_PENDING_ACK,
	T10_HANDLE_ACTIVE,
	T10_HANDLE_DISABLED
};

#define T10_CMD_SWINFO0_TYPE_MASK	        0xffff0000
#define T10_CMD_SWINFO0_ID_MASK		        0x0000ffff

/* SWInfo values on command replies */
#define T10_CMD_SWINFO0_ADD_MAC_ID  		0xa1110000
#define T10_CMD_SWINFO0_ADD_IP_ID   		0xa1120000
#define T10_CMD_SWINFO0_ADD_PORT_ID	   	    0xa1130000
#define T10_CMD_SWINFO0_GLOBAL_CFG_ID   	0xa1140000
#define T10_CMD_SWINFO0_USR_STATS_CFG_ID    0xa1150000
#define T10_CMD_SWINFO0_DEL_MAC_ID		    0xa0010000
#define T10_CMD_SWINFO0_DEL_IP_ID		    0xa0020000
#define T10_CMD_SWINFO0_DEL_PORT_ID		    0xa0030000

 typedef struct t10Handles_s  {

  	paHandleL2L3_t  paHandle;     /* The L3 handle returned by the PA LLD */

 	unsigned int	state;		  /* T4_HANDLE_UNCONFIGURED = handle not configured
 								   * T4_HANDLE_PENDING_ACK = handle configured and sent to pa
 								   * T4_HANDLE_ACTIVE = handle creation acknowledged by pa
 								   * T4_HANDLE_DISABLED = handle was created then released */
    unsigned int            linkCnt;

 } t10Handles_t;

 typedef struct t10HandlesL4_s  {

 	paHandleL4_t   paHandle;

 	unsigned int   state;

 } t10HandlesL4_t;

 /* A grouping of run time created grouped together to make cleanup easier on
  * error exit */

 typedef struct t10TestEncap_s  {
 	tFramework_t  *tf;
 	paTest_t      *pat;

 	/* There is one base packet for each L4 table entry */
 	Cppi_HostDesc  *hd[T10_NUM_LOCAL_L4_HANDLES];

 	/* The command to the modify PDSP to add the TCP/UDP checksum and route to PA receive */
 	uint32_t   cmdStack[T10_NUM_LOCAL_L4_HANDLES][(2 * sizeof(pasahoNextRoute_t) + sizeof(pasahoComChkCrc_t) + sizeof(pasahoIpFrag_t) + sizeof(pasahoComBlindPatch_t) + (2 * sizeof(pasahoPatchMsgLen_t))) / sizeof (uint32_t)];

 	/* The +1 is a place holder handle used to pass known invalid configuration into the PA LLD */
 	t10Handles_t    l2Handles[T10_NUM_LOCAL_L2_HANDLES];		/* MAC handles */
 	t10Handles_t    l3Handles[T10_NUM_LOCAL_L3_HANDLES];		/* IP handles  */
 	t10HandlesL4_t  l4Handles[T10_NUM_LOCAL_L4_HANDLES+1];	    /* UDP/TCP handles */
    unsigned int    genCmdAck[T10_NUM_GEN_CONFIGS];             /* General configurations */

 } t10TestEncap_t;

static paSysStats_t paTestL4ExpectedStats;    /* Expected stats results */

#ifdef PA_USE_HW_RA
static paRaStats_t paTestRaStats;
#endif

#define T10_NUM_USR_STATS                   4
#define T10_USR_STATS_ID_TX_PKTS            0
#define T10_USR_STATS_ID_TX_BYTES           1
#define T10_USR_STATS_ID_L2_PADDING_ERR     2
#define T10_USR_STATS_ID_L2_TX_PADDING      3

static  pauUsrStatsEntry_t t10UsrStatsTbl[T10_NUM_USR_STATS] =
{
    /* index, lnkId, fAlloc, f64b, fByteCnt */
    {0, T10_USR_STATS_ID_TX_BYTES, TRUE,   FALSE, FALSE},      /* Tx packet count */
    {0, PAU_USR_STATS_NO_LINK,     TRUE,   TRUE,  TRUE},       /* Tx byte count */
    {0, PAU_USR_STATS_NO_LINK,     TRUE,   FALSE, FALSE},      /* L2 padding error count */
    {0, PAU_USR_STATS_NO_LINK,     TRUE,   FALSE, FALSE},      /* L2 padding count */
};

/* Global configurations */
#define T10_NUM_64B_USR_STATS            64

static paIpReassmConfig_t   t10OutIpReassmCfg =
        {
            2,      /* Number of traffic Flow */
            0,      /* CPPI Flow */
            TF_FIRST_GEN_QUEUE + Q_IP_REASSM1   /* destination queue */
        };

static paIpReassmConfig_t   t10InIpReassmCfg =
        {
            2,      /* Number of traffic Flow */
            0,      /* CPPI Flow */
            TF_FIRST_GEN_QUEUE + Q_IP_REASSM2   /* destination queue */
        };

static paUsrStatsConfig_t   t10UsrStatsCfg =
       {
            pa_USR_STATS_MAX_COUNTERS - T10_NUM_64B_USR_STATS,   /* Number of user stats (448)*/
            T10_NUM_64B_USR_STATS                                /* Number of 64-bit user stats */
       };

static paPacketControlConfig_t  t10PktCtrlCfg =
        {
            0,                  /* ctrlBitMap */
            0,                  /* rxPaddingErrStatsIndex */
            0                   /* txPaddingStatsIndex */
        };

static	paQueueBounceConfig_t t10QueueBounceCfg =
	        {
	            1,      /* Enable */
                Q_BOUNCE_DDR  + TF_FIRST_GEN_QUEUE,             /* ddrQueueId */
                Q_BOUNCE_MSMC + TF_FIRST_GEN_QUEUE,             /* msmcQueueId */
                TF_PA_TX_QUEUE_BASE,                            /* hwQueueBegin */
                TF_PA_TX_QUEUE_BASE + NSS_NUM_TX_QUEUES - 1,    /* hwQueueEnd */
                {
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Command Return */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* QoS mode */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Capture Capture */
                    pa_QUEUE_BOUNCE_OP_DDR,     /* IP Reassembly-assisted packets */
                    pa_QUEUE_BOUNCE_OP_NONE     /* All traffics */
                }
	        };

#ifdef NSS_GEN2

static  paRaGroupConfig_t t10OutRaGroupCfg =
        {
        #ifdef PA_USE_HW_RA
            pa_RA_CTRL_ENABLE
            #ifdef NETSS_INTERNAL_PKTDMA
            | pa_RA_CTRL_USE_LOCAL_DMA
            #endif
            ,
            //pa_RA_CTRL_TO_QUEUE,                /* ctrlBitMap */
        #else
            0,
        #endif
            0,                                  /* Flow Id (4) */
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

static  paRaGroupConfig_t t10InRaGroupCfg =
        {
        #ifdef PA_USE_HW_RA
            pa_RA_CTRL_ENABLE
            #ifdef NETSS_INTERNAL_PKTDMA
            | pa_RA_CTRL_USE_LOCAL_DMA
            #endif
            ,
            //pa_RA_CTRL_TO_QUEUE,                /* ctrlBitMap */

        #else
            0,
        #endif
            0,                                  /* Flow Id (5) */
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

#endif

static  paSysConfig_t  t10GlobalCfg =
        {
            NULL,               /* pProtoLimit */
            &t10OutIpReassmCfg, /* pOutIpReassmConfig */
            &t10InIpReassmCfg,  /* pInIpReassmConfig */
            NULL,               /* pCmdSetConfig */
            &t10UsrStatsCfg,    /* pUsrStatsConfig */
            NULL,               /* pQueueDivertConfig */
            NULL,               /* pPktControl */
            &t10QueueBounceCfg, /* pQueueBounceConfig */
            NULL,               /* pOutAclConfig */
            NULL,               /* pInAclConfig */
            #ifdef NSS_GEN2
            &t10OutRaGroupCfg,  /* pOutIpRaGroupConfig */
            &t10InRaGroupCfg    /* pInIpRaGroupConfig */
            #else
            NULL,               /* pOutIpRaGroupConfig */
            NULL                /* pInIpRaGroupConfig */
            #endif
        };


static  paSysConfig_t  t10GlobalCfg2 =
        {
            NULL,               /* pProtoLimit */
            NULL,               /* pOutIpReassmConfig */
            NULL,               /* pInIpReassmConfig */
            NULL,               /* pCmdSetConfig */
            NULL,               /* pUsrStatsConfig */
            NULL,               /* pQueueDivertConfig */
            &t10PktCtrlCfg,     /* pPktControl */
            NULL,               /* pQueueBounceConfig */
            NULL,               /* pOutAclConfig */
            NULL,               /* pInAclConfig */
            NULL,               /* pOutIpRaGroupConfig */
            NULL                /* pInIpRaGroupConfig */
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


/*
 * User-definded Statitics Map
 *
 * Group 1: counter 0,  packet Counter (Rx Padding Error) no link
 *          counter 1,  packet Counter (Tx Padding) no link
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (t10UsrStatsGroup1, ".testPkts")
#endif

static paUsrStatsCounterEntryConfig_t t10UsrStatsGroup1[T10_NUM_USR_STATS];

#ifdef _TMS320C6X
#pragma DATA_SECTION(t10UsrStatsSetup, ".testPkts")
#endif
static pauUsrStatsSetup_t  t10UsrStatsSetup[] = {
    /* entry 0 */
    {
        sizeof(t10UsrStatsGroup1)/sizeof(paUsrStatsCounterEntryConfig_t),    /* number of entries */
        t10UsrStatsGroup1,                                                   /* counter Info table */
        pa_OK                                                               /* Expected return value */
    }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(t10Encap, ".testPkts")
#endif
static t10TestEncap_t  t10Encap;
static paUsrStats_t    paTestExpectedUsrStats;

static void t10Cleanup (t10TestEncap_t *tencap, paTestStatus_t status)
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
	for (i = 0; i < T10_NUM_LOCAL_L4_HANDLES; i++)  {
		if (tencap->hd[i] != NULL)
			Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
	}

 	/* Delete active L4 handles */
 	for (i = 0; i < T10_NUM_LOCAL_L4_HANDLES; i++)  {

		cmdReply.replyId = T10_CMD_SWINFO0_DEL_PORT_ID + i;
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l4Handles[i].state == T10_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T10_HANDLE_ACTIVE))  {
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

  	/* Delete active L3 Handles */
 	for (i = 0; i < T10_NUM_LOCAL_L3_HANDLES; i++)  {
 		cmdReply.replyId = T10_CMD_SWINFO0_DEL_IP_ID + i;
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l3Handles[i].state == T10_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T10_HANDLE_ACTIVE))  {
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

  	/* Delete active L2 Handles */
 	for (i = 0; i < T10_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T10_CMD_SWINFO0_DEL_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l2Handles[i].state == T10_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T10_HANDLE_ACTIVE))  {
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

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_IP_REASSM1]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_IP_REASSM1])) & ~15);
 		testCommonRecycleLBDesc (tencap->tf, hd);
 	}

 	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_IP_REASSM2]) > 0)  {
 		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_IP_REASSM2])) & ~15);
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

static void t10CmdRep (t10TestEncap_t *tencap)
{
	Cppi_HostDesc  *hd;
	uint32_t         *swinfo;
	paReturn_t      paret;
	uint32_t			swinfoType;
	uint32_t			swinfoIdx;
	paEntryHandle_t reth;
    int32_t			    htype;
    int32_t             cmdDest;
    char		   *s;
    uint32_t		   *stateP;
    uint32_t			stateV;
    int32_t				max;

	while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_CMD_REPLY]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_REPLY])) & ~15);

		    Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);

		    swinfoType = swinfo[0] & T10_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T10_CMD_SWINFO0_ID_MASK;

		    paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);


		    switch (swinfoType)  {

		    	case T10_CMD_SWINFO0_ADD_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T10_HANDLE_ACTIVE;
		    		max = T10_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_addMac";
		    		break;

		    	case T10_CMD_SWINFO0_ADD_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T10_HANDLE_ACTIVE;
		    		max = T10_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_addIp";
		    		break;

		    	case T10_CMD_SWINFO0_ADD_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T10_HANDLE_ACTIVE;
		    		max = T10_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_addPort";
		    		break;

		    	case T10_CMD_SWINFO0_DEL_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T10_HANDLE_DISABLED;
		    		max = T10_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_delMac";
		    		break;

		    	case T10_CMD_SWINFO0_DEL_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T10_HANDLE_DISABLED;
		    		max = T10_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_delIp";
		    		break;

		    	case T10_CMD_SWINFO0_DEL_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T10_HANDLE_DISABLED;
		    		max = T10_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_delPort";
		    		break;

                case T10_CMD_SWINFO0_GLOBAL_CFG_ID:
                    stateP = &tencap->genCmdAck[swinfoIdx];
                    stateV = TRUE;
                    max = T10_NUM_GEN_CONFIGS;
                    s = "Pa_control";
                    break;

		    	default:
		    		System_printf ("%s (%s:%d): Unknown command ID found in swinfo0 (0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
		    		t10Cleanup (tencap, PA_TEST_FAILED);
		    		break;

		    }

		    /* In this test only valid responses are exected from PA */
		    if (paret != pa_OK)  {
		    	System_printf ("%s (%s:%d): PA command %s returned error code %d, Index = %d\n", tfName, __FILE__, __LINE__, s, paret, swinfoIdx);
		    	t10Cleanup (tencap, PA_TEST_FAILED);
		    }


		   	if (swinfoIdx >= max)  {
		   		System_printf ("%s (%s:%d): Received command ack (%s) for out of range handle (%d, max = %d)\n",
		   						tfName, __FILE__, __LINE__, s, swinfoIdx, max);
		   		t10Cleanup (tencap, PA_TEST_FAILED);
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

static paTestStatus_t t10GlobalConfiguration (t10TestEncap_t *tencap, paSysConfig_t *pCfg)
{
	int32_t 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int32_t  			cmdDest;
	uint16_t			cmdSize;
    paCtrlInfo_t    ctrlInfo;

    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = *pCfg;
    cmdReply.replyId  = T10_CMD_SWINFO0_GLOBAL_CFG_ID;
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
		t10CmdRep (tencap);

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

static paTestStatus_t t10OpenL2 (t10TestEncap_t *tencap, t10EthSetup_t *ethSetup, int nL2Handles)
{
	int32_t 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int32_t  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL2Handles; i++)  {
		cmdReply.replyId = T10_CMD_SWINFO0_ADD_MAC_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddMac (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t10EthSetup[i].ethInfo, &matchRoute[1], &nfailRoute,
 	    	                   &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
 	        	               &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addMac command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t10Cleanup (tencap, PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l2Handles[i].state = T10_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

 		t10CmdRep (tencap);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t10CmdRep (tencap);

		for (j = m = 0; j < nL2Handles; j++)  {
			if (tencap->l2Handles[j].state == T10_HANDLE_ACTIVE)
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

static paTestStatus_t t10OpenL3 (t10TestEncap_t *tencap, t10IpSetup_t *ipSetup, int nL3Handles)
{
	int32_t 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int32_t  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL3Handles; i++)  {
		cmdReply.replyId = T10_CMD_SWINFO0_ADD_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddIp (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t10IpSetup[i].ipInfo,
                              t10IpSetup[i].nextLut1?&matchRoute[1]:&matchRoute[2],
                              &nfailRoute,
							  &tencap->l3Handles[i].paHandle,
							  t10IpSetup[i].innerIp?tencap->l3Handles[t10IpSetup[i].lHandleIdx].paHandle:
                                                    tencap->l2Handles[t10IpSetup[i].lHandleIdx].paHandle,
							  tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
							  &cmdReply, &cmdDest, &cmdSize, &paret);


		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addIp command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t10Cleanup (tencap, PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[i].state = T10_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    t10CmdRep (tencap);
            if (tencap->l3Handles[i].state == T10_HANDLE_ACTIVE)
            {
                t10IpSetup[i].acked = TRUE;
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

/* Simple User-Statistics routine: It does not process link and counter type */
static void t10UpdateUsrStats(paUsrStats_t* pStats, uint16_t cntIndex, uint32_t size)
{
    if (cntIndex < T10_NUM_64B_USR_STATS)
    {
        pStats->count64[cntIndex] += (uint64_t)size ;
    }
    else
    {
        pStats->count32[cntIndex - T10_NUM_64B_USR_STATS] += size;
    }
}

static paTestStatus_t t10OpenL4 (t10TestEncap_t *tencap, t10UdpSetup_t *udpSetup, int nL4Handles)
{
	int32_t 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int32_t  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL4Handles; i++)  {

		cmdReply.replyId  = T10_CMD_SWINFO0_ADD_PORT_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
        matchRoute[0].swInfo0 = T10_SWINFO0_PKT_ID + i;

		hd = testCommonAddPort (tencap->tf, pa_LUT2_PORT_SIZE_16, udpSetup[i].port, &matchRoute[0], &tencap->l4Handles[i].paHandle,
								&tencap->l3Handles[udpSetup[i].lHandleIdx].paHandle,
                                tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addPort command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t10Cleanup (tencap, PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		paTestL4ExpectedStats.classify2.nPackets += 1;
        #ifdef NSS_GEN2
 		paTestL4ExpectedStats.classify1.nPackets += 1;
        #endif
 		tencap->l4Handles[i].state = T10_HANDLE_PENDING_ACK;
 		utilCycleDelay (600);

 		t10CmdRep (tencap);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t10CmdRep (tencap);

		for (j = m = 0; j < nL4Handles; j++)  {
			if (tencap->l4Handles[j].state == T10_HANDLE_ACTIVE)
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

int t10NumTxPadding = 0;

static void t10UpdatesStats(paSysStats_t *stats, int pktIndex, uint16_t pktLen, uint16_t mtuSize, uint16_t ipOffset, int innerIp, int first)
{

    int numFrags;
    uint16_t fragSize, lastFragSize;
    uint16_t payloadSize;

    static int fFrags[T10_NUM_LOCAL_L4_HANDLES] = {FALSE, FALSE, FALSE, FALSE};


    /* Reset the fragement flag when the first packet in this packet stream is generated */
    if(first)fFrags[pktIndex] = FALSE;

    payloadSize = (pktLen - ipOffset) - 20;
    lastFragSize = mtuSize - 20;
    fragSize = lastFragSize & 0xFFF8;

    numFrags = 1;
    while (payloadSize >= lastFragSize)
    {
        numFrags += 1;
        payloadSize -= fragSize;

        if ((fragSize + ipOffset) < (60 - 20))
        {
            /* padding is required */
            t10UpdateUsrStats(&paTestExpectedUsrStats, t10UsrStatsTbl[T10_USR_STATS_ID_L2_TX_PADDING].cntIndex, 1);
            t10NumTxPadding++;
        }
    }

    if ((payloadSize + ipOffset) < (60 - 20))
    {
        /* padding is required */
        t10UpdateUsrStats(&paTestExpectedUsrStats, t10UsrStatsTbl[T10_USR_STATS_ID_L2_TX_PADDING].cntIndex, 1);
        t10NumTxPadding++;
    }

    if (numFrags > 1)
    {
        fFrags[pktIndex] = TRUE;
	    stats->classify1.nTxIpFrag += numFrags;
	    stats->classify1.nIpFrag   += numFrags;
    }

    /* MAC update due to fragments */
	stats->classify1.nPackets += (numFrags - 1);
    stats->classify1.nTableMatch += (numFrags - 1);

    /* Outer IP updates due to fragments */
    if (innerIp)
    {
	    stats->classify1.nPackets += (numFrags - 1);
        stats->classify1.nTableMatch += (numFrags - 1);
		stats->classify1.nIpv4Packets += (numFrags - 1);
    }

    /* IP Reassembly updates */
    if (fFrags[pktIndex])
    {
	    stats->classify1.nPackets += numFrags;
    }
}
#ifndef PA_USE_HW_RA
#ifndef SIMULATOR_SUPPORT
static void t10IpRreassemTimeoutUpdatesStats(paSysStats_t *stats, uint32_t count)
{
    stats->classify1.nIpv4Packets -= count;
    stats->classify1.nTableMatch -= count;
    stats->classify2.nUdp -= count;
	stats->classify2.nPackets -= count;

}
#endif
#endif

#if 0

/* Padding check for IP fragments only */
static int t10TxPaddingCheck(Cppi_HostDesc *hd, int* cnt)
{
	pasahoLongInfo_t 	*pInfo;
	uint32_t	      	 infoLen;
    uint16_t             ipOffset;
    uint16_t             fragOffset;
    uint16_t             pktLen, ipLen;
	uint8_t*    		 ipHdr;

	/* Get the parse information */
	if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pInfo, &infoLen) != CPPI_SOK)  {
		System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
		return (-1);
	}

    /* Extract IP related information. */
    ipOffset = PASAHO_LINFO_READ_START_OFFSET(pInfo);
  	pktLen = Cppi_getPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd);

    ipHdr = (uint8_t *)(hd->buffPtr + ipOffset);

    fragOffset = (ipHdr[6] << 8) + ipHdr[7];
    fragOffset &= 0x3fff;

    if (!fragOffset)
    {
        /* It is a non-fragmented packet, padding check should be done later */
        return 0;
    }

    ipLen = (ipHdr[2] << 8) + ipHdr[3];

    if ((ipLen + ipOffset) < 60)
    {
        /* padding is required */
        t10UpdateUsrStats(&paTestExpectedUsrStats, t10UsrStatsTbl[T10_USR_STATS_ID_L2_TX_PADDING].cntIndex, 1);
        *cnt += 1;
        if(pktLen != 60)
            return -1;
    }

    return 0;

}

/* Padding check for fragments packet at the final matched location */
static int t10TxPaddingCheck2(Cppi_HostDesc *hd, int* cnt)
{
	pasahoLongInfo_t 	*pInfo;
	uint32_t	      	 infoLen;
    uint16_t             endOffset;
    uint16_t             pktLen;

	/* Get the parse information */
	if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pInfo, &infoLen) != CPPI_SOK)  {
		System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
		return (-1);
	}

    /* Extract IP related information. */
    endOffset = PASAHO_LINFO_READ_END_OFFSET(pInfo);
  	pktLen = Cppi_getPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd);

    if (endOffset < 60)
    {
        /* padding is required */
        t10UpdateUsrStats(&paTestExpectedUsrStats, t10UsrStatsTbl[T10_USR_STATS_ID_L2_TX_PADDING].cntIndex, 1);
        *cnt += 1;
        if(pktLen != 60)
            return -1;
    }

    return 0;

}

#endif


static paTxChksum_t t10pktChksumIp = {  /* The IP checksum command */

    	0,     	/* Start offset of IP header */
       20,     	/* Checksum length  */
       10,      /* Offset to checksum location RELATIVE TO THE START OF THE TCP/UDP HEADER */
    	0, 		/* Initial value is IP header checksum value */
    	1       /* computed value of 0 written as -0 */

	};

static paTxChksum_t t10pktChksumUdp = {  /* The UDP checksum command */

    	0,     	/* Start offset of UDP header */
    	0,     	/* Checksum length (UDP header + payload */
    	6,      /* Offset to checksum location RELATIVE TO THE START OF THE TCP/UDP HEADER */
    	0, 		/* Initial value is IPv4 pseudo header checksum value */
    	1       /* computed value of 0 written as -0 */

	};


static	paCmdNextRoute_t t10pktRoute1 = { pa_NEXT_ROUTE_PARAM_PRESENT |
                                          pa_NEXT_ROUTE_PROC_USR_STATS,  /* ctrlBitfield */
                                          pa_DEST_HOST,	       /* Dest */
                                          0,                   /* pkyType: for SRIO only */
 								          0,				   /* Flow ID */
 								          TF_PA_QUEUE_TXCMD,   /* queue */
 								          0,				   /* sw Info 0 */
                                          0,                   /* sw Info 1 */
 							              0,  				    /* Multi route */
                                          0};                   /* User-defined stats index */
#if 1
static	paCmdNextRoute_t t10pktRoute2 = { pa_NEXT_ROUTE_PARAM_PRESENT |   /* ctrlBitfield */
                                          pa_NEXT_ROUTE_PROC_NEXT_CMD,
                                          pa_DEST_HOST,		  /* Dest */
                                          0,                  /* pkyType: for SRIO only */
 								          0,				  /* Flow ID */
 								          TF_PA_QUEUE_INPUT,  /* queue */
 								          0x12345678,		  /* sw Info 0 */
                                          0x87654321,         /* sw Info 1 */
 							              0,  				  /* Multi route */
                                          0};                 /* User-defined stats index */
#else
static	paCmdNextRoute_t t10pktRoute2 = { pa_NEXT_ROUTE_PARAM_PRESENT,    /* ctrlBitfield */
                                          pa_DEST_HOST,		  /* Dest */
                                          0,                  /* pkyType: for SRIO only */
 								          0,				  /* Flow ID */
 								          TF_PA_QUEUE_INPUT,  /* queue */
 								          0x12345678,		  /* sw Info 0 */
                                          0x87654321,         /* sw Info 1 */
 							              0,  				  /* Multi route */
                                          0};                 /* User-defined stats index */
#endif

static  paCmdIpFrag_t t10pktIpFrag =  {
                                       0,               /* ipOffset */
                                       256              /* mtuSize */
                                      };

static paPatchInfo_t t10pktPatch = {  /* The Blind patch command */

    	                                    0,     	/* ctrlBitfield (Overwrite) */
    	                                    8,     	/* The number of bytes to be patched */
    	                                    8,      /* The number of patch bytes in the patch command */
    	                                    0, 		/* offset */
    	                                    0       /* Pointer to the patch data */

	                               };

static uint16_t t10MtuSize[T10_NUM_LOCAL_L4_HANDLES] = {38, 200, 512, 1500};

static uint16_t t10pktPayloadSize[T10_NUM_LOCAL_L4_HANDLES][10] =
{
  {52,     8,   60,  82, 100, 120,  80,  60,   40,   20},  /* MTU size =  38  */
  {60,   300,  500, 120, 400, 250, 100, 500,   80,  300},  /* MTU size =  200 */
  {100,  500,  550,  60, 900, 600, 400, 850, 1400, 1000},  /* MTU size =  512 */
  {3000, 1500, 1550, 260, 900, 600, 400, 850, 1400, 2500},  /* MTU size = 1500 */
};

#ifndef __LINUX_USER_SPACE
static uint8_t t10pktBuf[T10_NUM_LOCAL_L4_HANDLES][T10_PKTBUF_SIZE];
#else
static uint8_t* t10pktBuf[T10_NUM_LOCAL_L4_HANDLES];
#endif

static void t10SendDataPkts (t10TestEncap_t *tencap, int nL4Handles, int group, int fFirst)
{
	Cppi_HostDesc  		*hd;
	Qmss_Queue      	q;
	int 				i;
	uint16_t  			cmdStackSize;
	paReturn_t      	paret;
    uint8_t             *buf;
    uint16_t            payloadLen;
    paCmdInfo_t        cmdInfo[8];
    paTxChksum_t       *pTxChksumIp  = &cmdInfo[0].params.chksum;
    paTxChksum_t       *pTxChksumUdp = &cmdInfo[1].params.chksum;
    paPatchInfo_t      *pTxPatch = &cmdInfo[7].params.patch;
    int                ipOffset, ipOffset2, udpOffset;
 	//volatile int mdebugWait = 1;

    /* Group 1: IP/UDP checksum */
    cmdInfo[0].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[0].params.chksum = t10pktChksumIp;
    cmdInfo[1].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[1].params.chksum = t10pktChksumUdp;
    cmdInfo[2].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[2].params.route = t10pktRoute1;
    /* Group 2: IP fragmentation plus Blind Patch (Simulating AH haeder patch) */
    /* cmdInfo[3].cmd = pa_CMD_PATCH_MSG_LEN or pa_CMD_NONE */
    cmdInfo[3].params.patchMsgLen.msgLenSize = 2;
    cmdInfo[3].params.patchMsgLen.offset = 14 + 4;
    cmdInfo[3].params.patchMsgLen.msgLen = 2;
    cmdInfo[4].params.patchMsgLen.msgLenSize = 2;
    cmdInfo[4].params.patchMsgLen.offset = 14 + 4;
    cmdInfo[4].params.patchMsgLen.msgLen = 2;
    cmdInfo[5].cmd = pa_CMD_IP_FRAGMENT;
    cmdInfo[5].params.ipFrag= t10pktIpFrag;
    cmdInfo[6].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[6].params.route = t10pktRoute2;
    cmdInfo[7].cmd = pa_CMD_PATCH_DATA;
    cmdInfo[7].params.patch = t10pktPatch;


	/* Attach one free descriptor to each of the packets */
	for (i = 0; i < nL4Handles; i++)  {
		tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);

		if (tencap->hd[i] == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
			t10Cleanup (tencap, PA_TEST_FAILED);
		}

		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);

        buf = t10pktBuf[i];
        payloadLen = t10pktPayloadSize[i][group];

  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)buf), (uint32_t)(t10PktInfo[i].pktLen + payloadLen));
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t10PktInfo[i].pktLen + payloadLen);

        memcpy(buf, t10PktInfo[i].pkt, t10PktInfo[i].pktLen);
        testGenPayload(PAU_PAYLOAD_INC8, 0, payloadLen, &buf[t10PktInfo[i].pktLen]);

        t10UpdateUsrStats(&paTestExpectedUsrStats, t10UsrStatsTbl[T10_USR_STATS_ID_TX_PKTS].cntIndex, 1);
        t10UpdateUsrStats(&paTestExpectedUsrStats, t10UsrStatsTbl[T10_USR_STATS_ID_TX_BYTES].cntIndex,
                          (uint32_t)t10PktInfo[i].pktLen + payloadLen);

        /* Update the packet header and command based on the payload size */
        udpOffset = PASAHO_LINFO_READ_L4_OFFSET(t10PktInfo[i].info);
        ipOffset =  PASAHO_LINFO_READ_L3_OFFSET(t10PktInfo[i].info);
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
        //utilUpdateIpChksums(&buf[ipOffset2]);
        pTxChksumIp ->startOffset = ipOffset2;

        /* Update UDP checksum command */
		pTxChksumUdp->startOffset = udpOffset;
		pTxChksumUdp->lengthBytes = payloadLen + 8;
		pTxChksumUdp->initialSum  = utilGetIpPsudoChkSum(&buf[ipOffset2], payloadLen + 8, 0x11);

        /* Add Message Length Patch command if PPPoE header is included */
        cmdInfo[4].cmd = cmdInfo[3].cmd =  (ipOffset > 14)?pa_CMD_PATCH_MSG_LEN:pa_CMD_NONE;

        cmdInfo[5].params.ipFrag.ipOffset = ipOffset2;
		cmdInfo[5].params.ipFrag.mtuSize = t10MtuSize[i];
        pTxPatch->offset = udpOffset + 8;
        pTxPatch->patchData = &buf[udpOffset + 8];  /* patch with the same data */

		cmdStackSize = sizeof(tencap->cmdStack[i]);

   		paret = Pa_formatTxCmd (    8,
                                	cmdInfo,
                                	0,
                                	(Ptr)&tencap->cmdStack[i],    /* Command buffer       */
                                	&cmdStackSize);   	          /* Command size         */

        if (paret != pa_OK)  {
        	System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        	t10Cleanup (tencap, PA_TEST_FAILED);
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
		Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)tencap->hd[i], (uint32_t)t10PktInfo[i].pktLen + payloadLen, TF_SIZE_DESC, Qmss_Location_TAIL);

        //while (mdebugWait);

		testCommonIncStats (t10PktInfo[i].statsMap, &paTestL4ExpectedStats);
        t10UpdatesStats(&paTestL4ExpectedStats, i, t10PktInfo[i].pktLen + payloadLen,  t10MtuSize[i], ipOffset2, ipOffset != ipOffset2, fFirst);

   	    utilCycleDelay (20000);

	}

   /* Wait for descriptors to return. It is assumed that they are returning in order. */
   for (i = 0; i < 100; i++)  {

   	if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= nL4Handles)
   		break;

   	 utilCycleDelay (500);
   }

   if (i == 100)  {
   	    System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
   	    t10Cleanup (tencap, PA_TEST_FAILED);
   }

   /* Recycle the descriptors */
   for (i = 0; i < nL4Handles; i++) {
   	    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
   	    if (tencap->hd[i] == NULL)  {
   		    System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
   		    t10Cleanup (tencap, PA_TEST_FAILED);
   	    }

		Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
        tencap->hd[i] = NULL;

   }

	/* Since the packets went to the modify PDSP and then back to the QM, a descriptor
	 * and linked buffer was required while the packet was in the QM. This will
	 * be recycled to the default recycle queue */
	while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
		Cppi_HostDesc *hdNext;
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
			break;
		}

        do
        {
            /* Goto the next descriptor. */
            hdNext = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc*)hd);
		    testCommonRecycleLBDesc (tencap->tf, hd);
        } while((hd = hdNext) != NULL);
	}

}
#ifndef PA_USE_HW_RA

#define MAX_RX_FRAGS_PER_CALL       20

static int t10RxFrags(t10TestEncap_t *tencap, Qmss_QueueHnd inQ, Qmss_QueueHnd outQ, int* pExpCnt)
{
	Cppi_HostDesc*       hd[MAX_RX_FRAGS_PER_CALL];
    int                  fragCnt = 0;
    int                  i, j, mid;
    Qmss_Queue           q;

    q.qMgr = 0;
	q.qNum = TF_DEF_RET_Q;

	/* Process the received fragments */
	while ((Qmss_getQueueEntryCount(inQ) > 0) && (fragCnt < MAX_RX_FRAGS_PER_CALL))  {

		hd[fragCnt]  = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (inQ)) & ~15);
		if (hd[fragCnt] == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet from queue %d\n", tfName, __FILE__, __LINE__, inQ);
			return (-1);
		}

        #if 0
        if(t10TxPaddingCheck(hd[fragCnt], &t10NumTxPadding)) {
			System_printf ("%s (%s:%d): Tx padding check failed for received packet from queue %d\n", tfName, __FILE__, __LINE__, inQ);
			return (-1);
        }
        #endif

        /* All the fragments should be cleaned up after reassembled */
        Cppi_setReturnPolicy (Cppi_DescType_HOST, (Cppi_Desc *) hd[fragCnt], Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET);
        Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd[fragCnt], q);

        fragCnt++;

	}

    /*
     * Send fragments to the reassembly function in alternative order to verify the the IP reassembly route support
     * multiple reassembly context simultaneouely.
     */

    mid = (fragCnt + 1)/2;

    for ( i = 0, j = mid; i < mid; i++, j++)
    {
        if (paEx_reassemLibProc(hd[i], outQ))
        {
			System_printf ("%s (%s:%d): sample_procPkts(%d) return error!\n", tfName, i, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tencap->tf, hd[i]);
			return (-1);
        }

        #ifndef  SIMULATOR_SUPPORT
        /* There is a pending simulator timing prroblem. To be enabled after the simulator bug is fixed */
        if ((j == (fragCnt - 1)) && (i == (mid -1)))
        {
            uint32_t timeoutCnt;
            /*
             * Test IP Reassembly timeout:
             * This fragment (packet) should be the last one within its group
             * If it is an IP fragments, the paEx_reassemLibTimerTick() will flush out the remaining fragments
             * If it is the case, update the corresponding statistics and *pExpCnt
             */
            paIPReassemblyStats_t stats;

            paEx_reassemLibQueryStats(&stats, FALSE);
            timeoutCnt = stats.reassemblyTimeout;

            /* Verify the Reassembly timeout operation */
            paEx_reassemLibTimerTick(5001);

            /* adjust the expected statistics */
            paEx_reassemLibQueryStats(&stats, FALSE);
            timeoutCnt = stats.reassemblyTimeout - timeoutCnt;

            if (timeoutCnt)
            {
                t10IpRreassemTimeoutUpdatesStats(&paTestL4ExpectedStats, timeoutCnt);
                *pExpCnt -= timeoutCnt;
            }

            if (paEx_reassemLibProc(hd[j], outQ))
            {
			    System_printf ("%s (%s:%d): sample_procPkts(%d) return error!\n", tfName, j, __FILE__, __LINE__);
			    testCommonRecycleLBDesc (tencap->tf, hd[j]);
			    return (-1);
            }

            if (timeoutCnt)
            {
                paEx_reassemLibTimerTick(5001);  /* kick out the last fragments */
                paTestL4ExpectedStats.classify1.nPackets++; /* There is an extra null packet entering PDSP1/PDSP2 */
            }

        }
        else
        #endif
        if (j < fragCnt)
        {
            if (paEx_reassemLibProc(hd[j], outQ))
            {
			    System_printf ("%s (%s:%d): sample_procPkts(%d) return error!\n", tfName, j, __FILE__, __LINE__);
			    testCommonRecycleLBDesc (tencap->tf, hd[j]);
			    return (-1);
            }
        }
    }

	/* Since the packets went to the modify PDSP and then back to the QM, a descriptor
	 * and linked buffer was required while the packet was in the QM. This will
	 * be recycled to the default recycle queue */
	while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
		hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
		if (hd[0] == NULL)  {
			System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
			return (-1);
		}

        testCommonRecycleLBDesc (tencap->tf, hd[0]);

	}

    return (0);
}
#endif

/* Search the receive data packet queue for received data packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t10ReceiveDataPkts (t10TestEncap_t *tencap, int expCount)
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
#ifndef PA_USE_HW_RA
    int               ddrBounceCount=0;
#endif

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (5000);

#ifndef PA_USE_HW_RA
        testCommonRelayQueueBouncePkts (tencap->tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, &ddrBounceCount, NULL);

        /* Process OutIp fragments */
        if (t10RxFrags(tencap, tencap->tf->QGen[Q_IP_REASSM1], tencap->tf->QPaTx[TF_PA_Q_OUTER_IP], &expCount))
        {
            return (-1);

        }

        /* Process Inner fragments */
        if (t10RxFrags(tencap, tencap->tf->QGen[Q_IP_REASSM2], tencap->tf->QPaTx[TF_PA_Q_INNER_IP], &expCount))
        {
            return (-1);
        }

#endif
	    /* Look for packets in the rx packet queue, verify that the UDP port found matches the next value in
	    * the fifo, then verify the receive packet information */
	    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_MATCH]) > 0)  {

		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
		    if (hd == NULL)  {
			    System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
		    }

            #if 0
            if(t10TxPaddingCheck2(hd, &t10NumTxPadding)) {
			    System_printf ("%s (%s:%d): Tx padding check failed for reassembled packet\n", tfName, __FILE__, __LINE__);
			    return (-1);
            }
            #endif

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((*swInfo & T10_CMD_SWINFO0_TYPE_MASK) != T10_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

			pktId = *swInfo & T10_CMD_SWINFO0_ID_MASK;

		    if (pktId >= T10_NUM_LOCAL_L4_HANDLES)  {
			    System_printf ("%s (%s:%d): Found a packet with unexpected packet id %d (max = %d)\n",
				    tfName, __FILE__, __LINE__, pktId, T10_NUM_LOCAL_L4_HANDLES - 1);
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


			if (testCommonComparePktInfo (tfName, t10PktInfo[pktId].info, pinfo))  {
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
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
			return (-1);
		}

        testCommonRecycleLBDesc (tencap->tf, hd);

	}

#ifndef PA_USE_HW_RA
    if(ddrBounceCount != count)
        System_printf("t10ReceiveDataPkts: receives %d queue bounce packets and %d final reassembled packets\n", ddrBounceCount, count);
#endif
	return (0);

}

int paT10NumTestPkts = T10_NUM_LOCAL_L4_HANDLES;

static void clearEthPrevCmdAck(t10EthSetup_t *setUp, int size)
{
  int i;
  for (i = 0 ; i < size ; i ++)
  {
     setUp->acked = FALSE;
     setUp++;
  }
}


static void clearIpPrevCmdAck(t10IpSetup_t *setUp, int size)
{
  int i;
  for (i = 0 ; i < size ; i ++)
  {
     setUp->acked = FALSE;
     setUp++;
  }
}

static void clearUdpPrevCmdAck(t10UdpSetup_t *setUp, int size)
{
  int i;
  for (i = 0 ; i < size ; i ++)
  {
     setUp->acked = FALSE;
     setUp++;
  }
}

#ifdef __LINUX_USER_SPACE
void* paTestIPv4FragReassem (void *args)
{
 	void  *a0 = (void *)((paTestArgs_t *)args)->tf;
 	void  *a1 = (void *)((paTestArgs_t *)args)->pat;
#else
void paTestIPv4FragReassem (UArg a0, UArg a1)
{
#endif
 	int				i,k;
 	paTestStatus_t  newStatus;
 	//volatile int mdebugWait = 1;
    paIPReassemblyConfig_t ipReassemCfg;

 	/* Initialize the test state */
 	memset (&t10Encap, 0, sizeof(t10Encap));
 	t10Encap.tf  = (tFramework_t *)a0;
 	t10Encap.pat = (paTest_t *)a1;
 	for (i = 0; i < T10_NUM_LOCAL_L4_HANDLES; i++)
 		t10Encap.l4Handles[i].state = T10_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T10_NUM_LOCAL_L3_HANDLES; i++)
 		t10Encap.l3Handles[i].state = T10_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T10_NUM_LOCAL_L2_HANDLES; i++)
 		t10Encap.l2Handles[i].state = T10_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T10_NUM_LOCAL_L4_HANDLES; i++)
 		t10Encap.hd[i] = NULL;

#ifdef __LINUX_USER_SPACE
    for (i = 0; i < T10_NUM_LOCAL_L4_HANDLES; i++)  {
        /* Allocate memory for the Packet buffers */
        t10pktBuf[i] = (uint8_t *)fw_memAlloc(T10_PKTBUF_SIZE, CACHE_LINESZ);
        if(t10pktBuf[i] == NULL) {
  	        printf ("%s: memAlloc failed for pkt %d\n", tfName, i);
 		    t10Encap.pat->testStatus = PA_TEST_FAILED;
  	        return (void *)0;
        }
    }
#endif
    /* Runtime initial values */
    matchRoute[0].queue = (uint16_t) t10Encap.tf->QGen[Q_MATCH];
    matchRoute[0].flowId = t10Encap.tf->tfFlowNum[0];
    nfailRoute.queue    = (uint16_t) t10Encap.tf->QGen[Q_NFAIL];
    cmdReply.queue      = (uint16_t) t10Encap.tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId     = t10Encap.tf->tfFlowNum[0];
    t10pktRoute1.flowId = t10Encap.tf->tfFlowNum[0];
    t10pktRoute2.flowId = t10Encap.tf->tfFlowNum[0];
    t10OutIpReassmCfg.destFlowId = t10Encap.tf->tfFlowNum[0];
    t10InIpReassmCfg.destFlowId = t10Encap.tf->tfFlowNum[0];

#ifdef NSS_GEN2
    #ifndef NETSS_INTERNAL_PKTDMA
    t10OutRaGroupCfg.flowId = t10Encap.tf->tfFlowNum1;
    t10InRaGroupCfg.flowId = t10Encap.tf->tfFlowNum2;
    #else
    t10OutRaGroupCfg.flowId = t10Encap.tf->tfLocFlowNum;
    t10InRaGroupCfg.flowId = t10Encap.tf->tfLocFlowNum;
    #endif
#endif

    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL4ExpectedStats, 0, sizeof(paTestL4ExpectedStats));
    memset (&paTestExpectedUsrStats, 0, sizeof(paTestExpectedUsrStats));

    /* Initialize the reassembly control blocks */
    memset(&ipReassemCfg, 0, sizeof(paIPReassemblyConfig_t));
    ipReassemCfg.timeout = 5000;
    ipReassemCfg.descSize = TF_SIZE_DESC;
    ipReassemCfg.numReassemblyContexts = 10;

    if (paEx_reassemLibInit(&ipReassemCfg))
    {
	    System_printf ("%s (%s:%d):  sample_initIPReassembly fail\n", tfName, __FILE__, __LINE__);
	    t10Cleanup (&t10Encap, PA_TEST_FAILED);
    }

    /* Global Configuration */
	newStatus = t10GlobalConfiguration (&t10Encap, &t10GlobalCfg);
	if (newStatus == PA_TEST_FAILED)
		t10Cleanup (&t10Encap, newStatus);  /* No return */

    /* Allocate User-defined statistics */
    if(testCommonAllocUsrStats(t10Encap.tf, tfName, T10_NUM_USR_STATS, t10UsrStatsTbl, t10UsrStatsGroup1))
    {
        newStatus = PA_TEST_FAILED;
        t10Cleanup (&t10Encap, newStatus);  /* No return */
    }

    /* Global Configuration 2 */
    t10PktCtrlCfg.rxPaddingErrStatsIndex = t10UsrStatsTbl[T10_USR_STATS_ID_L2_PADDING_ERR].cntIndex;
    t10PktCtrlCfg.txPaddingStatsIndex    = t10UsrStatsTbl[T10_USR_STATS_ID_L2_TX_PADDING].cntIndex;
	newStatus = t10GlobalConfiguration (&t10Encap, &t10GlobalCfg2);
	if (newStatus == PA_TEST_FAILED)
		t10Cleanup (&t10Encap, newStatus);  /* No return */

    /* Usr Stats configuration */
    newStatus = testCommonUsrStatsSetup (t10Encap.tf, t10Encap.pat, tfName, sizeof(t10UsrStatsSetup)/sizeof(pauUsrStatsSetup_t), t10UsrStatsSetup,
                                         T10_CMD_SWINFO0_USR_STATS_CFG_ID, t10Encap.tf->QLinkedBuf3, t10Encap.tf->QGen[Q_CMD_RECYCLE],
                                         t10Encap.tf->QGen[Q_CMD_REPLY], FALSE);
	if (newStatus == PA_TEST_FAILED)
		t10Cleanup (&t10Encap, newStatus);  /* No return */

	/* Burst in the L2 configuraton */
    clearEthPrevCmdAck(&t10EthSetup[0], T10_NUM_LOCAL_L2_HANDLES);
	newStatus = t10OpenL2 (&t10Encap, t10EthSetup, T10_NUM_LOCAL_L2_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t10Cleanup (&t10Encap, newStatus);  /* No return */

	/* Burst in the L3 configuration */
    clearIpPrevCmdAck(&t10IpSetup[0], T10_NUM_LOCAL_L3_HANDLES);
	newStatus = t10OpenL3 (&t10Encap, t10IpSetup, T10_NUM_LOCAL_L3_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t10Cleanup (&t10Encap, newStatus);

	/* Burst in the L3 configuration */
    clearUdpPrevCmdAck(&t10UdpSetup[0], T10_NUM_LOCAL_L4_HANDLES);
	newStatus = t10OpenL4 (&t10Encap, t10UdpSetup, T10_NUM_LOCAL_L4_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t10Cleanup (&t10Encap, newStatus);

	/* Send the packets. Use the modify PDSP to generate the TCP/UDP checksums.
	 * Use the modify PDSP to genertae IP fragments if necessary */
    t10pktRoute1.statsIndex = t10UsrStatsTbl[T10_USR_STATS_ID_TX_PKTS].cntIndex;
	for (k = 0; k < T10_NUM_PACKET_GROUPS; k+=1)  {

        /*
         * There are 4 streams of variable-size packets
         * Send multiple test groups at a time to verify the followings:
         *   - PASS can maintain multiple traffic flows
         *   - PASS will forward both fragments and non-fragmented packets to the host
         *     when the traffic flow is active
         */
        for (i = 0; i < 1; i++)
        {
            t10SendDataPkts (&t10Encap, paT10NumTestPkts, k + i, i == 0);
        }

        if (t10ReceiveDataPkts (&t10Encap, paT10NumTestPkts * i))
        {
		    System_printf ("%s (%s:%d):  Receive packets fail\n", tfName, __FILE__, __LINE__);
		    t10Cleanup (&t10Encap, PA_TEST_FAILED);
        }
	}


	/* Verify and clear the Usr stats */
	newStatus =  testCommonCheckUsrStatsList (t10Encap.tf, t10Encap.pat, tfName, &paTestExpectedUsrStats, T10_NUM_64B_USR_STATS, T10_NUM_USR_STATS, t10UsrStatsTbl,

                                              t10Encap.tf->QLinkedBuf1, t10Encap.tf->QGen[Q_CMD_RECYCLE], t10Encap.tf->QGen[Q_CMD_REPLY], TRUE);
    #ifndef PA_USE_HW_RA
    /* Note: Expected tx padding count can not be verified with Hardware RA module */
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckUsrStats Failed\n", tfName, __FILE__, __LINE__);
		t10Cleanup (&t10Encap, newStatus);
    }
    #endif

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t10Encap.tf, t10Encap.pat, tfName, &paTestL4ExpectedStats, t10Encap.tf->QLinkedBuf1,
	                                   t10Encap.tf->QGen[Q_CMD_RECYCLE], t10Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
	    System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t10Cleanup (&t10Encap, newStatus);
    }

    #ifdef PA_USE_HW_RA

    /* Display RA statistics */
    Pa_queryRaStats (t10Encap.tf->passHandle, TRUE, &paTestRaStats);
    testCommonDispRaStats (tfName, &paTestRaStats);

    #endif

    /* Clear all Usr Stats Link */
    newStatus = testCommonUsrStatsConfigReset (t10Encap.tf, t10Encap.pat, tfName, sizeof(t10UsrStatsSetup)/sizeof(pauUsrStatsSetup_t), t10UsrStatsSetup,
                                               T10_CMD_SWINFO0_USR_STATS_CFG_ID, t10Encap.tf->QLinkedBuf3, t10Encap.tf->QGen[Q_CMD_RECYCLE],
                                               t10Encap.tf->QGen[Q_CMD_REPLY]);

	if (newStatus == PA_TEST_FAILED)
    {
	    System_printf ("%s (%s:%d): testCommonUsrStatsConfigReset Failed\n", tfName, __FILE__, __LINE__);
	    t10Cleanup (&t10Encap, newStatus);
    }

    /* Free User-defined statistics */
    if(testCommonFreeUsrStats(t10Encap.tf, tfName, T10_NUM_USR_STATS, t10UsrStatsTbl))
    {
        newStatus = PA_TEST_FAILED;
    }

	/* No return from cleanup */
	t10Cleanup (&t10Encap, PA_TEST_PASSED);

#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
}

