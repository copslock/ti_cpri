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

/* IPv6 Fragmentation and PASS-assisted or RA-based IP Reassembly test
 * This test tests the LLD and firmware the ability to perform IPv6 fragmentation
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
 *  - Pa_requestStats
 *  - Pa_formatStatsReply
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
 *   - Test the firmware ability to calculate and verify UDP checksum
 * 	 - Test the firmware ability to perform IPv6 fragmentation
 *   - Test the firmware ability to route the fragments and packets to their destination queues or RA
 *   PASS-assisted IPv6 Reassembly
 *   - Test the ability to configure PASS-assisted IP Reassembly
 *   - Test the ability to configure RA-based IP Reassembly
 *   - Test the firmware for the ability to detect IP fragments
 *   - Test the firmware for the ability to allocate and free traffic flows
 *   - Test the firmware for the ability to identify active traffic flow per packet and forward the packet
 *     to the configured host queue
 *   - Test the firmware for the ability to process the reassembled and pass-through packets
 */
 #include "reassemLib.h"

 static char *tfName = "paTestIPv6FragReassem";

 #ifdef NSS_GEN2
 #define PA_USE_HW_RA               /* Use Hradware Reassembly Engine */
 #endif
 #define T11_NUM_PACKET_GROUPS	10 	/* Number of packet groups where each group contains packets with specified payload size */

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

#include "test11pkts.h"

/* The number of PA L2 and L3 handles maintained by this test */
#define T11_NUM_LOCAL_L2_HANDLES   			(sizeof(t11EthSetup)/sizeof(t11EthSetup_t))
#define T11_NUM_LOCAL_L3_HANDLES			(sizeof(t11IpSetup)/sizeof(t11IpSetup_t))
#define T11_NUM_LOCAL_L4_HANDLES	  		(sizeof(t11UdpSetup)/sizeof(t11UdpSetup_t))
#define T11_NUM_GEN_CONFIGS                  10     /* Maxmium number of general configuration commands */

 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T11_HANDLE_UNCONFIGURED = 0,
	T11_HANDLE_PENDING_ACK,
	T11_HANDLE_ACTIVE,
	T11_HANDLE_DISABLED
};

#define T11_CMD_SWINFO0_TYPE_MASK	        0xffff0000
#define T11_CMD_SWINFO0_ID_MASK		        0x0000ffff

/* SWInfo values on command replies */
#define T11_CMD_SWINFO0_ADD_MAC_ID  		0xa1110000
#define T11_CMD_SWINFO0_ADD_IP_ID   		0xa1120000
#define T11_CMD_SWINFO0_ADD_PORT_ID	   	    0xa1130000
#define T11_CMD_SWINFO0_GLOBAL_CFG_ID   	0xa1140000
#define T11_CMD_SWINFO0_USR_STATS_CFG_ID    0xa1150000
#define T11_CMD_SWINFO0_DEL_MAC_ID		    0xa0010000
#define T11_CMD_SWINFO0_DEL_IP_ID		    0xa0020000
#define T11_CMD_SWINFO0_DEL_PORT_ID		    0xa0030000

 typedef struct t11Handles_s  {

  	paHandleL2L3_t  paHandle;     /* The L3 handle returned by the PA LLD */

 	unsigned int	state;		  /* T4_HANDLE_UNCONFIGURED = handle not configured
 								   * T4_HANDLE_PENDING_ACK = handle configured and sent to pa
 								   * T4_HANDLE_ACTIVE = handle creation acknowledged by pa
 								   * T4_HANDLE_DISABLED = handle was created then released */
    unsigned int            linkCnt;

 } t11Handles_t;

 typedef struct t11HandlesL4_s  {

 	paHandleL4_t   paHandle;

 	unsigned int 		   state;

 } t11HandlesL4_t;

 /* A grouping of run time created grouped together to make cleanup easier on
  * error exit */

 typedef struct t11TestEncap_s  {
 	tFramework_t  *tf;
 	paTest_t      *pat;

 	/* There is one base packet for each L4 table entry */
 	Cppi_HostDesc  *hd[T11_NUM_LOCAL_L4_HANDLES];

 	/* The command to the modify PDSP to add the TCP/UDP checksum and route to PA receive */
 	uint32_t   cmdStack[T11_NUM_LOCAL_L4_HANDLES][(2 * sizeof(pasahoNextRoute_t) + sizeof(pasahoComChkCrc_t) + sizeof(pasahoIpFrag_t) + sizeof(pasahoComBlindPatch_t) + (2 * sizeof(pasahoPatchMsgLen_t))) / sizeof (uint32_t)];

 	/* The +1 is a place holder handle used to pass known invalid configuration into the PA LLD */
 	t11Handles_t    l2Handles[T11_NUM_LOCAL_L2_HANDLES];		/* MAC handles */
 	t11Handles_t    l3Handles[T11_NUM_LOCAL_L3_HANDLES];		/* IP handles  */
 	t11HandlesL4_t  l4Handles[T11_NUM_LOCAL_L4_HANDLES+1];	    /* UDP/TCP handles */
    unsigned int            genCmdAck[T11_NUM_GEN_CONFIGS];             /* General configurations */


 } t11TestEncap_t;

static paSysStats_t paTestL4ExpectedStats;    /* Expected stats results */

#ifdef PA_USE_HW_RA
static paRaStats_t paTestRaStats;
#endif

static paIpReassmConfig_t   t11OutIpReassmCfg =
        {
            2,      /* Number of traffic Flow */
            0,      /* CPPI Flow */
            PA_BOUNCE_QUEUE_DDR(TF_FIRST_GEN_QUEUE + Q_IP_REASSM1)   /* destination queue */
        };

static paIpReassmConfig_t   t11InIpReassmCfg =
        {
            2,      /* Number of traffic Flow */
            0,      /* CPPI Flow */
            PA_BOUNCE_QUEUE_DDR(TF_FIRST_GEN_QUEUE + Q_IP_REASSM2)   /* destination queue */
        };

static	paQueueBounceConfig_t t11QueueBounceCfg =
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
                    pa_QUEUE_BOUNCE_OP_MSMC,     /* IP Reassembly-assisted packets */
                    pa_QUEUE_BOUNCE_OP_NONE     /* All traffics */
                }
	        };

static  paRaGroupConfig_t t11OutRaGroupCfg =
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
            4,                                  /* Flow Id */
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

static  paRaGroupConfig_t t11InRaGroupCfg =
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
            5,                                  /* Flow Id */
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


static  paSysConfig_t  t11GlobalCfg =
        {
            NULL,               /* pProtoLimit */
            &t11OutIpReassmCfg, /* pOutIpReassmConfig */
            &t11InIpReassmCfg,  /* pInIpReassmConfig */
            NULL,               /* pCmdSetConfig */
            NULL,               /* pUsrStatsConfig */
            NULL,               /* pQueueDivertConfig */
            NULL,               /* pPktControl */
            &t11QueueBounceCfg, /* pQueueBounceConfig */
            NULL,               /* pOutAclConfig */
            NULL,               /* pInAclConfig */
            &t11OutRaGroupCfg,     /* pOutIpRaGroupConfig */
            &t11InRaGroupCfg      /* pInIpRaGroupConfig */
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
#pragma DATA_SECTION(t11Encap, ".testPkts")
#endif
static t11TestEncap_t  t11Encap;

static void t11Cleanup (t11TestEncap_t *tencap, paTestStatus_t status)
{
	int32_t			   i;
	int32_t  	       cmdDest;
 	uint16_t	       cmdSize;
 	paReturn_t     paret;
 	paTestStatus_t newStatus;
 	Cppi_HostDesc *hd;

	/* Wait a bit for any packets in PA to complete */
	utilCycleDelay (5000);

	/* Return the descriptors which are pointing to packets */
	for (i = 0; i < T11_NUM_LOCAL_L4_HANDLES; i++)  {
		if (tencap->hd[i] != NULL)
			Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
	}

 	/* Delete active L4 handles */
 	for (i = 0; i < T11_NUM_LOCAL_L4_HANDLES; i++)  {

		cmdReply.replyId = T11_CMD_SWINFO0_DEL_PORT_ID + i;
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l4Handles[i].state == T11_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T11_HANDLE_ACTIVE))  {
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
 	for (i = 0; i < T11_NUM_LOCAL_L3_HANDLES; i++)  {
 		cmdReply.replyId = T11_CMD_SWINFO0_DEL_IP_ID + i;
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l3Handles[i].state == T11_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T11_HANDLE_ACTIVE))  {
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
 	for (i = 0; i < T11_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T11_CMD_SWINFO0_DEL_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l2Handles[i].state == T11_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T11_HANDLE_ACTIVE))  {
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

static void t11CmdRep (t11TestEncap_t *tencap)
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

		    swinfoType = swinfo[0] & T11_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T11_CMD_SWINFO0_ID_MASK;

		    paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);


		    switch (swinfoType)  {

		    	case T11_CMD_SWINFO0_ADD_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T11_HANDLE_ACTIVE;
		    		max = T11_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_addMac";
		    		break;

		    	case T11_CMD_SWINFO0_ADD_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T11_HANDLE_ACTIVE;
		    		max = T11_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_addIp";
		    		break;

		    	case T11_CMD_SWINFO0_ADD_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T11_HANDLE_ACTIVE;
		    		max = T11_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_addPort";
		    		break;

		    	case T11_CMD_SWINFO0_DEL_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T11_HANDLE_DISABLED;
		    		max = T11_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_delMac";
		    		break;

		    	case T11_CMD_SWINFO0_DEL_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T11_HANDLE_DISABLED;
		    		max = T11_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_delIp";
		    		break;

		    	case T11_CMD_SWINFO0_DEL_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T11_HANDLE_DISABLED;
		    		max = T11_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_delPort";
		    		break;

                case T11_CMD_SWINFO0_GLOBAL_CFG_ID:
                    stateP = &tencap->genCmdAck[swinfoIdx];
                    stateV = TRUE;
                    max = T11_NUM_GEN_CONFIGS;
                    s = "Pa_control";
                    break;

		    	default:
		    		System_printf ("%s (%s:%d): Unknown command ID found in swinfo0 (0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
		    		t11Cleanup (tencap, PA_TEST_FAILED);
		    		break;

		    }

		    /* In this test only valid responses are exected from PA */
		    if (paret != pa_OK)  {
		    	System_printf ("%s (%s:%d): PA command %s returned error code %d, Index = %d\n", tfName, __FILE__, __LINE__, s, paret, swinfoIdx);
		    	t11Cleanup (tencap, PA_TEST_FAILED);
		    }


		   	if (swinfoIdx >= max)  {
		   		System_printf ("%s (%s:%d): Received command ack (%s) for out of range handle (%d, max = %d)\n",
		   						tfName, __FILE__, __LINE__, s, swinfoIdx, max);
		   		t11Cleanup (tencap, PA_TEST_FAILED);
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

static paTestStatus_t t11GlobalConfiguration (t11TestEncap_t *tencap)
{
	int32_t 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int32_t  			cmdDest;
	uint16_t			cmdSize;
    paCtrlInfo_t    ctrlInfo;

    /* Issue the command set command */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = t11GlobalCfg;
    cmdReply.replyId  = T11_CMD_SWINFO0_GLOBAL_CFG_ID;
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
		t11CmdRep (tencap);

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

static paTestStatus_t t11OpenL2 (t11TestEncap_t *tencap, t11EthSetup_t *ethSetup, int nL2Handles)
{
	int32_t 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int32_t  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL2Handles; i++)  {
		cmdReply.replyId = T11_CMD_SWINFO0_ADD_MAC_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddMac (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t11EthSetup[i].ethInfo, &matchRoute[1], &nfailRoute,
 	    	                   &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
 	        	               &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addMac command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t11Cleanup (tencap, PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l2Handles[i].state = T11_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

 		t11CmdRep (tencap);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t11CmdRep (tencap);

		for (j = m = 0; j < nL2Handles; j++)  {
			if (tencap->l2Handles[j].state == T11_HANDLE_ACTIVE)
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

static paTestStatus_t t11OpenL3 (t11TestEncap_t *tencap, t11IpSetup_t *ipSetup, int nL3Handles)
{
	int32_t 			i, j;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int32_t  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL3Handles; i++)  {
		cmdReply.replyId = T11_CMD_SWINFO0_ADD_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

		hd = testCommonAddIp (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t11IpSetup[i].ipInfo,
                              t11IpSetup[i].nextLut1?&matchRoute[1]:&matchRoute[2],
                              &nfailRoute,
							  &tencap->l3Handles[i].paHandle,
							  t11IpSetup[i].innerIp?tencap->l3Handles[t11IpSetup[i].lHandleIdx].paHandle:
                                                    tencap->l2Handles[t11IpSetup[i].lHandleIdx].paHandle,
							  tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
							  &cmdReply, &cmdDest, &cmdSize, &paret);


		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addIp command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t11Cleanup (tencap, PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[i].state = T11_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

	    for (j = 0; j < 100; j++)  {
            utilCycleDelay (1000);
		    t11CmdRep (tencap);
            if (tencap->l3Handles[i].state == T11_HANDLE_ACTIVE)
            {
                t11IpSetup[i].acked = TRUE;
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

static paTestStatus_t t11OpenL4 (t11TestEncap_t *tencap, t11UdpSetup_t *udpSetup, int nL4Handles)
{
	int32_t 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int32_t  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < nL4Handles; i++)  {

		cmdReply.replyId  = T11_CMD_SWINFO0_ADD_PORT_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
        matchRoute[0].swInfo0 = T11_SWINFO0_PKT_ID + i;

		hd = testCommonAddPort (tencap->tf, pa_LUT2_PORT_SIZE_16, udpSetup[i].port, &matchRoute[0], &tencap->l4Handles[i].paHandle,
								&tencap->l3Handles[udpSetup[i].lHandleIdx].paHandle,
                                tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			System_printf ("%s: (%s:%d): Failure in common addPort command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t11Cleanup (tencap, PA_TEST_FAILED);
		}

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		paTestL4ExpectedStats.classify2.nPackets += 1;
        #ifdef NSS_GEN2
        paTestL4ExpectedStats.classify1.nPackets += 1;
        #endif
 		tencap->l4Handles[i].state = T11_HANDLE_PENDING_ACK;
 		utilCycleDelay (600);

 		t11CmdRep (tencap);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t11CmdRep (tencap);

		for (j = m = 0; j < nL4Handles; j++)  {
			if (tencap->l4Handles[j].state == T11_HANDLE_ACTIVE)
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


static void t11UpdatesStats(paSysStats_t *stats, int pktIndex, uint16_t pktLen, uint16_t mtuSize, uint16_t ipOffset, uint16_t l4Offset, int innerIp, int first)
{

    int32_t numFrags;
    uint16_t ipHdrLen;
    uint16_t fragSize;
    uint16_t payloadSize;

    static int32_t fFrags[T11_NUM_LOCAL_L4_HANDLES] = {FALSE, FALSE, FALSE, FALSE};


    /* Reset the fragement flag when the first packet in this packet stream is generated */
    if(first)fFrags[pktIndex] = FALSE;

    mtuSize &= 0xFFF8;

    payloadSize = (pktLen - l4Offset);
    if (innerIp)
    {
        fragSize = mtuSize - 48;
    }
    else
    {
        ipHdrLen = (l4Offset - ipOffset) + 8;
        fragSize = mtuSize - ipHdrLen;
    }

    numFrags = 1;

    if ((fragSize + 8) < payloadSize)
    {
        /* note: the condition for non-fragmented packet should not include the fragment header */
        while (payloadSize > fragSize)
        {
            numFrags++;
            payloadSize -= fragSize;
        }
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
		stats->classify1.nIpv6Packets += (numFrags - 1);
    }

    /* IP Reassembly updates */
    if (fFrags[pktIndex])
    {
	    stats->classify1.nPackets += numFrags;
    }
}
#ifndef PA_USE_HW_RA
#ifndef SIMULATOR_SUPPORT
static void t11IpRreassemTimeoutUpdatesStats(paSysStats_t *stats, uint32_t count)
{
    stats->classify1.nIpv6Packets -= count;
    stats->classify1.nTableMatch -= count;
    stats->classify2.nUdp -= count;
	stats->classify2.nPackets -= count;

}
#endif
#endif


static paTxChksum_t t11pktChksum = {  /* The UDP checksum command */

    	0,     	/* Start offset of UDP header */
    	0,     	/* Checksum length (UDP header + payload */
    	6,      /* Offset to checksum location RELATIVE TO THE START OF THE TCP/UDP HEADER */
    	0, 		/* Initial value is IPv6 pseudo header checksum value */
    	1       /* computed value of 0 written as -0 */

	};

static	paCmdNextRoute_t t11pktRoute1 = { pa_NEXT_ROUTE_PARAM_PRESENT,  /* ctrlBitfield */
                                          pa_DEST_HOST,	       /* Dest */
                                          0,                   /* pkyType: for SRIO only */
 								          0,				   /* Flow ID */
 								          TF_PA_QUEUE_TXCMD,   /* queue */
 								          0,				   /* sw Info 0 */
                                          0,                   /* sw Info 1 */
 							              0};				   /* Multi route */

#if 1
static	paCmdNextRoute_t t11pktRoute2 = { pa_NEXT_ROUTE_PARAM_PRESENT |   /* ctrlBitfield */
                                          pa_NEXT_ROUTE_PROC_NEXT_CMD,
                                          pa_DEST_HOST,		  /* Dest */
                                          0,                  /* pkyType: for SRIO only */
 								          0,				  /* Flow ID */
 								          TF_PA_QUEUE_INPUT,  /* queue */
 								          0x12345678,		  /* sw Info 0 */
                                          0x87654321,         /* sw Info 1 */
 							              0};				  /* Multi route */
#else
static	paCmdNextRoute_t t11pktRoute2 = { pa_NEXT_ROUTE_PARAM_PRESENT,    /* ctrlBitfield */
                                          pa_DEST_HOST,		  /* Dest */
                                          0,                  /* pkyType: for SRIO only */
 								          0,				  /* Flow ID */
 								          TF_PA_QUEUE_INPUT,  /* queue */
 								          0x12345678,		  /* sw Info 0 */
                                          0x87654321,         /* sw Info 1 */
 							              0};				  /* Multi route */
#endif

static  paCmdIpFrag_t t11pktIpFrag =  {
                                       14,               /* ipOffset */
                                       256              /* mtuSize */
                                      };

static paPatchInfo_t t11pktPatch = {  /* The Blind patch command */

    	                                    0,     	/* ctrlBitfield (Overwrite) */
    	                                    8,     	/* The number of bytes to be patched */
    	                                    8,      /* The number of patch bytes in the patch command */
    	                                    0, 		/* offset */
    	                                    0       /* Pointer to the patch data */

	                               };


static uint16_t t11MtuSize[T11_NUM_LOCAL_L4_HANDLES] = {
                                                        80,       /* 80 */
                                                        200,
                                                        512,
                                                        1500};

static uint16_t t11pktPayloadSize[T11_NUM_LOCAL_L4_HANDLES][10] =
{
    {32,   40,  60, 80,  100, 120,  80, 60,  40,   20},    /* MTU size = 80   */
    {60,  300, 500, 120, 400, 250, 100, 500, 80,   300},   /* MTU size = 200 */
    {100, 500, 550, 60,  900, 600, 400, 850, 1500, 1000},  /* MTU size = 512 */
    {3000, 1500, 1550, 260, 900, 600, 400, 850, 1400, 2500},  /* MTU size = 1500 */
};

#ifndef __LINUX_USER_SPACE
static uint8_t t11pktBuf[T11_NUM_LOCAL_L4_HANDLES][T11_PKTBUF_SIZE];
#else
static uint8_t* t11pktBuf[T11_NUM_LOCAL_L4_HANDLES];
#endif

static void t11SendDataPkts (t11TestEncap_t *tencap, int nL4Handles, int group, int fFirst)
{
	Cppi_HostDesc  		*hd;
	Qmss_Queue      	q;
	int32_t 				i;
	uint16_t  			cmdStackSize, ipPayloadLen;
	paReturn_t      	paret;
    uint8_t              *buf;
    uint16_t             payloadLen;
    paCmdInfo_t        cmdInfo[7];
    paTxChksum_t       *pTxChksum = &cmdInfo[0].params.chksum;
    paPatchInfo_t      *pTxPatch = &cmdInfo[6].params.patch;
    int32_t                ipOffset, ipOffset2, udpOffset, fInnerIp = FALSE;
 	//volatile int mdebugWait = 1;

    /* Group 1: UDP checksum */
    cmdInfo[0].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[0].params.chksum = t11pktChksum;
    cmdInfo[1].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[1].params.route = t11pktRoute1;
    /* Group 2: IP fragmentation plus Blind Patch (Simulating AH header patch) */
    /* cmdInfo[2].cmd = pa_CMD_PATCH_MSG_LEN or pa_CMD_NONE */
    cmdInfo[2].params.patchMsgLen.msgLenSize = 2;
    cmdInfo[2].params.patchMsgLen.offset = 14 + 4;
    cmdInfo[2].params.patchMsgLen.msgLen = 2;
    cmdInfo[3].params.patchMsgLen.msgLenSize = 2;
    cmdInfo[3].params.patchMsgLen.offset = 14 + 4;
    cmdInfo[3].params.patchMsgLen.msgLen = 2;
    cmdInfo[4].cmd = pa_CMD_IP_FRAGMENT;
    cmdInfo[4].params.ipFrag= t11pktIpFrag;
    cmdInfo[5].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[5].params.route = t11pktRoute2;
    cmdInfo[6].cmd = pa_CMD_PATCH_DATA;
    cmdInfo[6].params.patch = t11pktPatch;

	/* Attach one free descriptor to each of the packets */
	for (i = 0; i < nL4Handles; i++)  {
		tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);

        if (tencap->hd[i] == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
			t11Cleanup (tencap, PA_TEST_FAILED);
		}

		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);

        buf = t11pktBuf[i];
        payloadLen = t11pktPayloadSize[i][group];

  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)buf), (uint32_t)(t11PktInfo[i].pktLen + payloadLen));
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t11PktInfo[i].pktLen + payloadLen);

        memcpy(buf, t11PktInfo[i].pkt, t11PktInfo[i].pktLen);
        testGenPayload(PAU_PAYLOAD_INC8, 0, payloadLen, &buf[t11PktInfo[i].pktLen]);

        /* Update the packet header and command based on the payload size */
        udpOffset = PASAHO_LINFO_READ_L4_OFFSET(t11PktInfo[i].info);
        ipOffset =  PASAHO_LINFO_READ_L3_OFFSET(t11PktInfo[i].info);
        ipOffset2 = udpOffset - 40;
        ipPayloadLen = (udpOffset - ipOffset) - 40 + payloadLen + 8;

        /*
         * For our test case, the following assumptions are made
         * 1. The total szie of extension headers of outer IP is less than 40 bytes
         * 2. There is no extension header for outer IP
         */
        if ((ipOffset2 - ipOffset) >= 40)
        {
            /* It is IP over IP packet */
            buf[ipOffset2 + 4] = (payloadLen + 8) >> 8;
            buf[ipOffset2 + 5] = (payloadLen + 8) & 0xFF;
            buf[ipOffset2 + 3] = (uint8_t)group;
            fInnerIp = TRUE;
        }

        buf[ipOffset + 4] = (ipPayloadLen) >> 8;
        buf[ipOffset + 5] = (ipPayloadLen) & 0xFF;
        buf[ipOffset + 3] = (uint8_t)group;
        buf[udpOffset + 4] =  (payloadLen + 8) >> 8;
        buf[udpOffset + 5] =  (payloadLen + 8) & 0xFF;

		pTxChksum->startOffset = udpOffset;
		pTxChksum->lengthBytes = payloadLen + 8;
        if (fInnerIp)
        {
		    pTxChksum->initialSum  = utilGetIpPsudoChkSum(&buf[ipOffset2], payloadLen + 8, 0x11);
        }
        else
        {
		    pTxChksum->initialSum  = utilGetIpPsudoChkSum(&buf[ipOffset], payloadLen + 8, 0x11);
        }


        /* Add Message Length Patch command if PPPoE header is included */
        cmdInfo[2].cmd = cmdInfo[3].cmd =  (ipOffset > 14)?pa_CMD_PATCH_MSG_LEN:pa_CMD_NONE;

        cmdInfo[4].params.ipFrag.ipOffset = (fInnerIp)?ipOffset2:ipOffset;
        cmdInfo[4].params.ipFrag.mtuSize = t11MtuSize[i];
        pTxPatch->offset = udpOffset + 8;
        pTxPatch->patchData = &buf[udpOffset + 8];  /* patch with the same data */

        cmdStackSize = sizeof(tencap->cmdStack[i]);

		paret = Pa_formatTxCmd (    7,
									&cmdInfo[0],
									0,
									(Ptr)&tencap->cmdStack[i],    /* Command buffer       */
									&cmdStackSize);   	          /* Command size         */

		if (paret != pa_OK)  {
			System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
			t11Cleanup (tencap, PA_TEST_FAILED);
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
		Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)tencap->hd[i], (uint32_t)t11PktInfo[i].pktLen + payloadLen, TF_SIZE_DESC, Qmss_Location_TAIL);
        //while (mdebugWait);

		testCommonIncStats (t11PktInfo[i].statsMap, &paTestL4ExpectedStats);
        t11UpdatesStats(&paTestL4ExpectedStats, i, t11PktInfo[i].pktLen + payloadLen,  t11MtuSize[i], ipOffset, udpOffset, fInnerIp, fFirst);

	}

   /* Wait for descriptors to return. It is assumed that they are returning in order. */
   for (i = 0; i < 100; i++)  {

   	if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= nL4Handles)
   		break;

   	 utilCycleDelay (500);
   }

   if (i == 100)  {
   	    System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
   	    t11Cleanup (tencap, PA_TEST_FAILED);
   }

   /* Recycle the descriptors */
   for (i = 0; i < nL4Handles; i++) {
   	    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
   	    if (tencap->hd[i] == NULL)  {
   		    System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
   		    t11Cleanup (tencap, PA_TEST_FAILED);
   	    }

		Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
        tencap->hd[i] = NULL;

   }

	/* Since the packets went to the modify PDSP and then back to the QM, a descriptor
	 * and linked buffer was required while the packet was in the QM. This will
	 * be recycled to the default recycle queue */
	while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {

		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
			break;
		}

		testCommonRecycleLBDesc (tencap->tf, hd);
	}
}

static void t11SendFragDataPkts (t11TestEncap_t *tencap, int nL4Handles)
{
  Qmss_Queue      	 q;
  int32_t 				   i;
  uint8_t           *buf;
  uint16_t           payloadLen;

  /* Attach one free descriptor to each of the packets */
  for (i = 0; i < nL4Handles; i++)  {
       tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);

       if (tencap->hd[i] == NULL)  {
          System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
          t11Cleanup (tencap, PA_TEST_FAILED);
	   }

		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);

        buf = t11pktBuf[i];
        payloadLen = t11MalFPktInfo[i].pktLen;

  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)buf), (uint32_t)(payloadLen));
        Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t) payloadLen);
        Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), 0);

        memcpy(buf, t11MalFPktInfo[i].pkt, t11MalFPktInfo[i].pktLen);

		/* Send the data to the PDSP0 */
        //mdebugHaltPdsp(5);
		Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_INPUT], (Ptr)tencap->hd[i], (uint32_t)t11PktInfo[i].pktLen + payloadLen, TF_SIZE_DESC, Qmss_Location_TAIL);
        //while (mdebugWait);
 		testCommonIncStats (t11MalFPktInfo[i].statsMap, &paTestL4ExpectedStats);
	}

   /* Wait for descriptors to return. It is assumed that they are returning in order. */
   for (i = 0; i < 100; i++)  {

   	if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= nL4Handles)
   		break;

   	 utilCycleDelay (500);
   }

   if (i == 100)  {
   	    System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
   	    t11Cleanup (tencap, PA_TEST_FAILED);
   }

   /* Recycle the descriptors */
   for (i = 0; i < nL4Handles; i++) {
   	    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
   	    if (tencap->hd[i] == NULL)  {
   		    System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
   		    t11Cleanup (tencap, PA_TEST_FAILED);
   	    }

		Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
        tencap->hd[i] = NULL;

   }
}

#ifndef PA_USE_HW_RA

#define MAX_RX_FRAGS_PER_CALL       20

static int t11RxFrags(t11TestEncap_t *tencap, Qmss_QueueHnd inQ, Qmss_QueueHnd outQ, int* pExpCnt)
{
	Cppi_HostDesc*       hd[MAX_RX_FRAGS_PER_CALL];
    int32_t                  fragCnt = 0;
    int32_t                  i, j, mid;
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
                t11IpRreassemTimeoutUpdatesStats(&paTestL4ExpectedStats, timeoutCnt);
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
static int t11ReceiveDataPkts (t11TestEncap_t *tencap, int expCount)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;
	int32_t               i,j;
    int32_t               pktId = 0;
	uint32_t			  flags;
    uint32_t              eflags;
    int32_t               count = 0;
#ifndef PA_USE_HW_RA
    int                   ddrBounceCount=0;
#endif

	for (i = 0; i < 100; i++)  {

        utilCycleDelay (5000);
#ifndef PA_USE_HW_RA
        testCommonRelayQueueBouncePkts (tencap->tf, tfName, Q_BOUNCE_DDR, Q_BOUNCE_MSMC, &ddrBounceCount, NULL);

        /* Process OutIp fragments */
        if (t11RxFrags(tencap, tencap->tf->QGen[Q_IP_REASSM1], tencap->tf->QPaTx[TF_PA_Q_OUTER_IP], &expCount))
        {
            return (-1);

        }


        /* Process Inner fragments */
        if (t11RxFrags(tencap, tencap->tf->QGen[Q_IP_REASSM2], tencap->tf->QPaTx[TF_PA_Q_INNER_IP], &expCount))
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

			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

			if ((*swInfo & T11_CMD_SWINFO0_TYPE_MASK) != T11_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}

			pktId = *swInfo & T11_CMD_SWINFO0_ID_MASK;

		    if (pktId >= T11_NUM_LOCAL_L4_HANDLES)  {
			    System_printf ("%s (%s:%d): Found a packet with unexpected packet id %d (max = %d)\n",
				    tfName, __FILE__, __LINE__, pktId, T11_NUM_LOCAL_L4_HANDLES - 1);
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

			/* locate the associated test information based on the channel value */
			for (j = 0;  j < sizeof(t11PktInfo) / sizeof(pktTestInfo_t); j++)  {
				if (t11PktInfo[j].idx == (T11_SWINFO0_PKT_ID | pktId))  {
					break;
				}
			}

			if (j == sizeof(t11PktInfo) / sizeof(pktTestInfo_t) )  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for packet ID %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, pktId);
				testCommonRecycleLBDesc (tencap->tf, hd);
				return (-1);
			}



			if (testCommonComparePktInfo (tfName, t11PktInfo[j].info, pinfo))  {
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
        System_printf("t11ReceiveDataPkts: receives %d queue bounce packets and %d final reassembled packets\n", ddrBounceCount, count);
#endif
	return (0);
}


static void clearEthPrevCmdAck(t11EthSetup_t *setUp, int size)
{
  int i;
  for (i = 0 ; i < size ; i ++)
  {
     setUp->acked = FALSE;
     setUp++;
  }
}


static void clearIpPrevCmdAck(t11IpSetup_t *setUp, int size)
{
  int i;
  for (i = 0 ; i < size ; i ++)
  {
     setUp->acked = FALSE;
     setUp++;
  }
}

static void clearUdpPrevCmdAck(t11UdpSetup_t *setUp, int size)
{
  int i;
  for (i = 0 ; i < size ; i ++)
  {
     setUp->acked = FALSE;
     setUp++;
  }
}

//int paT11NumTestPkts = T11_NUM_LOCAL_L4_HANDLES;
#ifndef SIMULATOR_SUPPORT
int paT11NumTestPkts = 4;
#else
int paT11NumTestPkts = 2;
#endif
#ifdef __LINUX_USER_SPACE
void* paTestIPv6FragReassem (void *args)
{
 	void  *a0 = (void *)((paTestArgs_t *)args)->tf;
 	void  *a1 = (void *)((paTestArgs_t *)args)->pat;
#else
void paTestIPv6FragReassem (UArg a0, UArg a1)
{
#endif
 	int				i,k;
 	paTestStatus_t  newStatus;
 	//volatile int mdebugWait = 1;
    paIPReassemblyConfig_t ipReassemCfg;

 	/* Initialize the test state */
 	memset (&t11Encap, 0, sizeof(t11Encap));
 	t11Encap.tf  = (tFramework_t *)a0;
 	t11Encap.pat = (paTest_t *)a1;
 	for (i = 0; i < T11_NUM_LOCAL_L4_HANDLES; i++)
 		t11Encap.l4Handles[i].state = T11_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T11_NUM_LOCAL_L3_HANDLES; i++)
 		t11Encap.l3Handles[i].state = T11_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T11_NUM_LOCAL_L2_HANDLES; i++)
 		t11Encap.l2Handles[i].state = T11_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T11_NUM_LOCAL_L4_HANDLES; i++)
 		t11Encap.hd[i] = NULL;

#ifdef __LINUX_USER_SPACE
    for (i = 0; i < T11_NUM_LOCAL_L4_HANDLES; i++)  {
        /* Allocate memory for the Packet buffers */
        t11pktBuf[i] = (uint8_t *)fw_memAlloc(T11_PKTBUF_SIZE, CACHE_LINESZ);
        if(t11pktBuf[i] == NULL) {
  	        printf ("%s: memAlloc failed for pkt %d\n", tfName, i);
 		    t11Encap.pat->testStatus = PA_TEST_FAILED;
  	        return (void *)0;
        }
    }
#endif
    /* Runtime initial values */
    matchRoute[0].queue = (uint16_t) t11Encap.tf->QGen[Q_MATCH];
    matchRoute[0].flowId = t11Encap.tf->tfFlowNum[0];
    nfailRoute.queue    = (uint16_t) t11Encap.tf->QGen[Q_NFAIL];
    cmdReply.queue      = (uint16_t) t11Encap.tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId     = t11Encap.tf->tfFlowNum[0];
    t11pktRoute1.flowId = t11Encap.tf->tfFlowNum[0];
    t11pktRoute2.flowId = t11Encap.tf->tfFlowNum[0];
    t11OutIpReassmCfg.destFlowId = t11Encap.tf->tfFlowNum[0];
    t11InIpReassmCfg.destFlowId = t11Encap.tf->tfFlowNum[0];

#ifdef NSS_GEN2
    #ifndef NETSS_INTERNAL_PKTDMA
    t11OutRaGroupCfg.flowId = t11Encap.tf->tfFlowNum1;
    t11InRaGroupCfg.flowId = t11Encap.tf->tfFlowNum2;
    #else
    t11OutRaGroupCfg.flowId = t11Encap.tf->tfLocFlowNum;
    t11InRaGroupCfg.flowId = t11Encap.tf->tfLocFlowNum;
    #endif
#endif


    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL4ExpectedStats, 0, sizeof(paTestL4ExpectedStats));

    /* Initialize the reassembly control blocks */
    memset(&ipReassemCfg, 0, sizeof(paIPReassemblyConfig_t));
    ipReassemCfg.timeout = 5000;
    ipReassemCfg.descSize = TF_SIZE_DESC;
    ipReassemCfg.numReassemblyContexts = 10;

    if (paEx_reassemLibInit(&ipReassemCfg))
    {
	    System_printf ("%s (%s:%d):  sample_initIPReassembly fail\n", tfName, __FILE__, __LINE__);
	    t11Cleanup (&t11Encap, PA_TEST_FAILED);
    }

    /* Global Configuration */
	newStatus = t11GlobalConfiguration (&t11Encap);
	if (newStatus == PA_TEST_FAILED)
		t11Cleanup (&t11Encap, newStatus);  /* No return */

	/* Burst in the L2 configuraton */
    clearEthPrevCmdAck(t11EthSetup, T11_NUM_LOCAL_L2_HANDLES);
	newStatus = t11OpenL2 (&t11Encap, t11EthSetup, T11_NUM_LOCAL_L2_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t11Cleanup (&t11Encap, newStatus);  /* No return */

	/* Burst in the L3 configuration */
    clearIpPrevCmdAck(t11IpSetup, T11_NUM_LOCAL_L3_HANDLES);
	newStatus = t11OpenL3 (&t11Encap, t11IpSetup, T11_NUM_LOCAL_L3_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t11Cleanup (&t11Encap, newStatus);

	/* Burst in the L4 configuration */
    clearUdpPrevCmdAck(t11UdpSetup, T11_NUM_LOCAL_L4_HANDLES);
	newStatus = t11OpenL4 (&t11Encap, t11UdpSetup, T11_NUM_LOCAL_L4_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t11Cleanup (&t11Encap, newStatus);

  /* Send the malformed packet test. The full fragmented packet 
   * needs to be pushed to Mac Look up pdsp - this should not affect the regular tests */
    t11SendFragDataPkts (&t11Encap, sizeof (t11MalFPktInfo)/sizeof (pktTestInfo_t));

	/* Send the packets. Use the modify PDSP to generate the TCP/UDP checksums.
	 * Use the modify PDSP to genertae IP fragments if necessary */
	for (k = 0; k < T11_NUM_PACKET_GROUPS; k+=1)
    //for (k = 0; k < 1; k+=1)
    {
        /*
         * There are 4 streams of variable-size packets
         * Send multiple test groups at a time to verify the followings:
         *   - PASS can maintain multiple traffic flows
         *   - PASS will forward both fragments and non-fragmented packets to the host
         *     when the traffic flow is active
         */
        // k = 0;
        for (i = 0; i < 1; i++)
        {
            t11SendDataPkts (&t11Encap, paT11NumTestPkts, k + i, i == 0);
        }

        if (t11ReceiveDataPkts (&t11Encap, paT11NumTestPkts * i))
        {
		    System_printf ("%s (%s:%d):  Receive packets fail\n", tfName, __FILE__, __LINE__);
		    t11Cleanup (&t11Encap, PA_TEST_FAILED);
        }
	}

    #ifdef PA_USE_HW_RA

    Pa_queryRaStats (t11Encap.tf->passHandle, TRUE, &paTestRaStats);
    testCommonDispRaStats (tfName, &paTestRaStats);

    #endif

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t11Encap.tf, t11Encap.pat, tfName, &paTestL4ExpectedStats, t11Encap.tf->QLinkedBuf1,
	                                   t11Encap.tf->QGen[Q_CMD_RECYCLE], t11Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
	    System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t11Cleanup (&t11Encap, newStatus);
    }

	/* No return from cleanup */
	t11Cleanup (&t11Encap, PA_TEST_PASSED);

#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
}



