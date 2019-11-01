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

/* Add/Delete ACL and ACL Filtering test
 * This test test the LLD Pa_addAcl, Pa_delAclHandle and Pa_queryAclStats, as well as the
 * PDSP firmware for filtering IP packets. This test has the following sub-tests
 *   - Test the LLD and firmware for the ability to insert ACL entries in order
 *   - Test the LLD to detect duplicate entry
 *   - Test the LLD when entering more entries then configured for
 *   - Test the LLD for global configuration
 *   - Test the firmware for the ability to take a burst of Pa_AddAcl commands
 *   - Test the firmware for the ability to detect IP header errors
 *   - Test the firmware for the ability to handle both outer and inner ACL rules
 *   - Test the firmware for the ability to take a burst of data packets
 *   - Test the LLD/firmware for the ability to delete ACL entries
 */

static char *tfName = "paTestACL";

#define T12_NUM_PACKET_ITERATIONS	1  		/* Number of times the packet stream is passed through */

#define T12_EXPTPKT_FIFO_SIZE		20		/* Must hold the number of packets in transit in PA at one time */

/* General purpose queue usage */
#define Q_CMD_RECYCLE	 0		/* Command descriptors/buffers recycled here after sent to PA */
#define Q_CMD_REPLY  	 1		/* Replies from PA routed here */
#define Q_MATCH		  	 2		/* Packets from PA which match a lookup criteria */
#define Q_NFAIL		     3		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
#define Q_PARSE_ERR		 4		/* Packets which resulted in a parse error */


/* The number of PA L2 and L3 handles maintained by this test */
#define T12_NUM_LOCAL_L2_HANDLES   			2
#define T12_NUM_LOCAL_L3_OUTER_IP_HANDLES	64
#define T12_NUM_LOCAL_L3_INNER_IP_HANDLES	62
#define T12_NUM_LOCAL_L3_OUTER_AND_INNER1_IP_HANDLE  (T12_NUM_LOCAL_L3_OUTER_IP_HANDLES + T12_NUM_LOCAL_L3_INNER_IP_HANDLES)
#define T12_NUM_LOCAL_L3_HANDLES				(T12_NUM_LOCAL_L3_OUTER_IP_HANDLES+T12_NUM_LOCAL_L3_INNER_IP_HANDLES+2)
#define T12_NUM_LOCAL_L4_HANDLES	  		8
#define T12_NUM_LOCAL_OUTER_ACL_HANDLES	    64
#define T12_NUM_LOCAL_INNER_ACL_HANDLES	    64
#define T12_NUM_LOCAL_ACL_HANDLES	  		(T12_NUM_LOCAL_OUTER_ACL_HANDLES + T12_NUM_LOCAL_INNER_ACL_HANDLES)


typedef struct t12IpPaSetup_s  {
	int        seqId;		/* Sequential enumeration to identify */
	int		   handleIdx;	/* Local handle index. Specifies which local handle corresponds to this entry */
    int        lutInst;     /* Specify which LUT1 (0-2) should be used */
	int		   lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	int		   routeIdx;	/* Which match route index to use, 0 or 1 */
	paIpInfo_t ipInfo;		/* PA IP configuration structure */
	paReturn_t ret;			/* Expected return code from pa_addIp command */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */

} t12IpPaSetup_t;


typedef struct t12AclPaSetup_s  {
	int        seqId;		/* Sequential enumeration to identify */
	int		   handleIdx;	/* Local handle index. Specifies which local handle corresponds to this entry */
    int        aclInst;     /* Specify which ACL LUT1 (0-1) should be used */
	int		   lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	int		   nHandleIdx;  /* Next ACL handle Index (This entry should be inserted in front of the rule specified by this one*/
	int		   action;	    /* ACL action per match */
	paAclInfo_t aclInfo;	/* PA ACL configuration structure */
	paReturn_t ret;			/* Expected return code from pa_addIp command */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */
} t12AclPaSetup_t;

#include "test12pkts.h"

/* The total number of buffers with linked descriptors */
#define TOTAL_BUFS   (TF_LINKED_BUF_Q1_NBUFS + TF_LINKED_BUF_Q2_NBUFS + TF_LINKED_BUF_Q3_NBUFS)

 /* Commands to the PA are verified through the value in swinfo0.
  * The 16 ms bits are used as verification, the 16 lbs are for local handle id */
#define T12_CMD_SWINFO0_ADD_MAC_ID  	0x11100000  /* Identifies add mac command */
#define T12_CMD_SWINFO0_DEL_MAC_ID  	0x11110000  /* Identifies del mac command */
#define T12_CMD_SWINFO0_ADD_IP_ID		0x22200000  /* Identifies the add IP command */
#define T12_CMD_SWINFO0_DEL_IP_ID		0x22210000  /* Identifies the del IP command */
#define T12_CMD_SWINFO0_ADD_PORT_ID		0x22220000  /* Identifies the add port command */
#define T12_CMD_SWINFO0_DEL_PORT_ID		0x22230000  /* Identifies the del port command */
#define T12_CMD_SWINFO0_ADD_ACL_ID		0x22240000  /* Identifies the add ACL command */
#define T12_CMD_SWINFO0_DEL_ACL_ID		0x22250000  /* Identifies the del ACL command */
#define T12_CMD_SWINFO0_STATS_REQ_ID	0x33330000	/* Identifies the req stats command */
#define T12_CMD_SWINFO0_CRC_CFG_ID		0x44400000  /* Identifies the CRC config command */
#define T12_CMD_SWINFO0_GLOBAL_CFG_ID   0x44410000  /* Identifies the Global config command */
#define T12_CMD_SWINFO0_PKT_ID			0x55550000  /* Identifies the packet as a data packet */

#define T12_CMD_SWINFO0_TYPE_MASK		0xffff0000  /* Mask for the command type */
#define T12_CMD_SWINFO0_ID_MASK			0x0000ffff  /* Mask for the local ID */

 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T12_HANDLE_UNCONFIGURED = 0,
	T12_HANDLE_PENDING_ACK,
	T12_HANDLE_ACTIVE,
	T12_HANDLE_DISABLED
};

typedef struct t12Handles_s  {

 	paHandleL2L3_t  paHandle;     /* The L3 handle returned by the PA LLD */

	unsigned int	state;		  /* T12_HANDLE_UNCONFIGURED = handle not configured
								   * T12_HANDLE_PENDING_ACK = handle configured and sent to pa
								   * T12_HANDLE_ACTIVE = handle creation acknowledged by pa
								   * T12_HANDLE_DISABLED = handle was created then released */

} t12Handles_t;

typedef struct t12HandlesL4_s  {

	paHandleL4_t   paHandle;

	unsigned int   state;

} t12HandlesL4_t;

typedef struct t12HandlesAcl_s  {

 	paHandleAcl_t   paHandle;     /* The ACL handle returned by the PA LLD */

	unsigned int	state;

} t12HandlesAcl_t;

 /* A grouping of run time created grouped together to make cleanup easier on
  * error exit */
 typedef struct t12TestEncap_s  {
 	tFramework_t  *tf;
 	paTest_t      *pat;

 	/* The +1 is a place holder handle used to pass known invalid configuration into the PA LLD */
 	t12Handles_t    l2Handles[T12_NUM_LOCAL_L2_HANDLES+1];		/* MAC handles */
 	t12Handles_t    l3Handles[T12_NUM_LOCAL_L3_HANDLES+1];		/* IP handles  */
 	t12HandlesL4_t  l4Handles[T12_NUM_LOCAL_L4_HANDLES+1];		/* UDP/TCP handles */
    t12HandlesAcl_t aclHandles[T12_NUM_LOCAL_ACL_HANDLES+1];	/* ACL handles */

 } t12TestEncap_t;

static paSysStats_t paTestL4ExpectedStats;    /* Expected stats results */

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

 								    {  		pa_DEST_CONTINUE_PARSE_LUT2,/* Dest */
 								       		0,							/* Flow ID */
 								       		0,							/* queue */
 								           -1,							/* Multi route */
 								        	0,							/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL},                      /* No commands            */


 								  	{		pa_DEST_CONTINUE_PARSE_LUT1,/* Dest */
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


/* CRC Configuration of CRC-32C for SCTP */
static paCrcConfig_t   t12CrcCfg = {
                                    pa_CRC_CONFIG_INVERSE_RESULT,       /* ctrlBitfield */
                                    pa_CRC_SIZE_32,
                                    0x1EDC6F41,                         /* polynomial */
                                    0xFFFFFFFF                          /* initValue */
                                  };


#ifdef _TMS320C6X
#pragma DATA_SECTION(t12Encap, ".testPkts")
#endif
static t12TestEncap_t  t12Encap;

void t12Cleanup (t12TestEncap_t *tencap, paTestStatus_t testStatus)
{
 	int 	       i;
 	int  	       cmdDest;
 	uint16_t	   cmdSize;
 	paReturn_t     paret;
 	Cppi_HostDesc *hd;
 	paTestStatus_t newStatus;

 	/* Delete active L4 handles */
 	for (i = 0; i < T12_NUM_LOCAL_L4_HANDLES; i++)  {

		cmdReply.replyId = T12_CMD_SWINFO0_DEL_PORT_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l4Handles[i].state == T12_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T12_HANDLE_ACTIVE))  {
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
 	for (i = 0; i < T12_NUM_LOCAL_L3_HANDLES; i++)  {
 		cmdReply.replyId = T12_CMD_SWINFO0_DEL_IP_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l3Handles[i].state == T12_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T12_HANDLE_ACTIVE))  {
			hd = testCommonDelHandle (tencap->tf, &tencap->l3Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
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


 	/* Delete active L2 Handles */
 	for (i = 0; i < T12_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T12_CMD_SWINFO0_DEL_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l2Handles[i].state == T12_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T12_HANDLE_ACTIVE))  {
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

 	/* Delete active ACL Handles */
 	for (i = 0; i < T12_NUM_LOCAL_ACL_HANDLES; i++)  {
 		cmdReply.replyId = T12_CMD_SWINFO0_DEL_ACL_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->aclHandles[i].state == T12_HANDLE_PENDING_ACK) || (tencap->aclHandles[i].state == T12_HANDLE_ACTIVE))  {
			hd = testCommonDelAclHandle (tencap->tf, &tencap->aclHandles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
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


 	newStatus = testCommonCheckStats (tencap->tf, tencap->pat, tfName, &paTestL4ExpectedStats, tencap->tf->QLinkedBuf1,
	                       tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
		testStatus = PA_TEST_FAILED;

	/* Return result */
    tencap->pat->testStatus = testStatus;

    /* Return */
    Task_exit();
}


static void t12DispAclStats(t12TestEncap_t *tencap)
{
    int i;
    paReturn_t paret;
    paAclStats_t aclStats;

 	for (i = 0; i < T12_NUM_LOCAL_ACL_HANDLES; i++)  {

		if ( tencap->aclHandles[i].state == T12_HANDLE_ACTIVE)  {

            paret = Pa_queryAclStats(tencap->tf->passHandle, tencap->aclHandles[i].paHandle, 0,  &aclStats);

			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): Pa_queryAclStats returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				continue;
			}

            System_printf ("ACL Stats (%d): nMatchPackets = %d,  nMatchBytes = %d\n", i, aclStats.nMatchPackets, aclStats.nMatchBytes);

 		}
 	}

    System_flush();
}


/* Check for pa lld return errors. Exit the test on any error condition */
static void t12HandleError (t12TestEncap_t *tencap, paReturn_t paret, Cppi_HostDesc *hd, int line)
{

	if (paret == pa_OK)
		return;

	System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, line, paret);

	if ((hd != NULL) && testCommonRecycleLBDesc (tencap->tf, hd))
		System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);

    t12Cleanup (tencap, PA_TEST_FAILED);  /* No return */

}


 /* Look for replies to add Ip commands and verify the results */
void t12L3CmdRep (t12TestEncap_t *tencap, t12IpPaSetup_t *ipSetup)
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

		    swinfoType = swinfo[0] & T12_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T12_CMD_SWINFO0_ID_MASK;

            if (swinfoType != T12_CMD_SWINFO0_ADD_IP_ID)  {
                System_printf ("%s (%s:%d): found packet in command reply queue without add IP swinfo type (found 0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            if (swinfoIdx >= sizeof(t12OuterIpInfo)/sizeof(t12IpPaSetup_t))  {
            	System_printf ("%s (%s:%d): found packet in command reply queue with invalid index (found 0x%08x)\n", tfName, __FILE__, __LINE__, swinfoIdx);
            	testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            ipSetup[swinfoIdx].acked = TRUE;
            paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);

            if (paret != ipSetup[swinfoIdx].ret)  {
                System_printf ("%s (%s:%d): Pa_forwardResult returned %d, expected %d\n", tfName, __FILE__, __LINE__, paret, ipSetup[swinfoIdx].ret);
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            testCommonRecycleLBDesc (tencap->tf, hd);

            tencap->l3Handles[ipSetup[swinfoIdx].handleIdx].state = T12_HANDLE_ACTIVE;

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

 /* Look for replies to add Ip commands and verify the results */
void t12AclCmdRep (t12TestEncap_t *tencap, t12AclPaSetup_t *aclSetup, int maxEntries)
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

		    swinfoType = swinfo[0] & T12_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T12_CMD_SWINFO0_ID_MASK;

            if (swinfoType != T12_CMD_SWINFO0_ADD_ACL_ID)  {
                System_printf ("%s (%s:%d): found packet in command reply queue without add IP swinfo type (found 0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            if (swinfoIdx >= maxEntries)  {
            	System_printf ("%s (%s:%d): found packet in command reply queue with invalid index (found 0x%08x)\n", tfName, __FILE__, __LINE__, swinfoIdx);
            	testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            aclSetup[swinfoIdx].acked = TRUE;
            paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);

            if (paret != aclSetup[swinfoIdx].ret)  {
                System_printf ("%s (%s:%d): Pa_forwardResult returned %d, expected %d\n", tfName, __FILE__, __LINE__, paret, aclSetup[swinfoIdx].ret);
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            testCommonRecycleLBDesc (tencap->tf, hd);

            tencap->aclHandles[aclSetup[swinfoIdx].handleIdx].state = T12_HANDLE_ACTIVE;

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



/*
 * Utility function to derive the LUT1 index from the local index
 * It is used to verify the Pa_addIP() with application specific index
 * The index should be consistent with the system allocated LUT1 index
 */
static int t12GetLUT1Index(int handleIndex)
{
    if (handleIndex >= T12_NUM_LOCAL_L3_HANDLES)
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
}


static paTestStatus_t t12OpenIp (t12TestEncap_t  *tencap, t12IpPaSetup_t *ipSetup, int n, t12Handles_t *linkedHandles, char *id)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paRouteInfo_t  *pNfailRoute;

	for (i = 0; i < n; i++)  {
		//cmdReply.replyId = T12_CMD_SWINFO0_ADD_IP_ID + ipSetup[i].seqId;
        cmdReply.replyId = T12_CMD_SWINFO0_ADD_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

        pNfailRoute = (ipSetup[i].routeIdx == 1)?&nfailRoute1:&nfailRoute;
		hd = testCommonAddIp2 (tencap->tf, ipSetup[i].lutInst, t12GetLUT1Index(ipSetup[i].handleIdx), &ipSetup[i].ipInfo, &matchRoute[ipSetup[i].routeIdx], pNfailRoute,
							   &tencap->l3Handles[ipSetup[i].handleIdx].paHandle,
							   linkedHandles[ipSetup[i].lHandleIdx].paHandle,
							   tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf2,
							   &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			/* It's not a failure if the return code from pa was expected */
			if (paret == ipSetup[i].ret)  {
				ipSetup[i].acked = TRUE;
				continue;
			}

			System_printf ("%s: (%s:%d): Failure in common addIp command, %s entry number %d\n", tfName, __FILE__, __LINE__, id, ipSetup[i].seqId);
			t12HandleError (tencap, paret, hd, __LINE__);
		}
        else if (paret == pa_DUP_ENTRY) {
            /* The ack will be handle for the first entry */
            ipSetup[i].acked = TRUE;
        }

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[ipSetup[i].handleIdx].state = T12_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

 		t12L3CmdRep (tencap, ipSetup);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t12L3CmdRep (tencap, ipSetup);

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


static paTestStatus_t t12OpenAcl (t12TestEncap_t  *tencap, t12AclPaSetup_t *aclSetup, int n, t12Handles_t *linkedHandles, t12HandlesAcl_t *nextHandles,  char *id)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;

	for (i = 0; i < n; i++)  {
        cmdReply.replyId = T12_CMD_SWINFO0_ADD_ACL_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

        if (aclSetup[i].nHandleIdx != -1)
        {

            for (j = 0; j < 100; j++)
            {
                /* wait for previous entry to be completed */
			    utilCycleDelay (1000);

 		        t12AclCmdRep (tencap, aclSetup, n);

                if(aclSetup[aclSetup[i].nHandleIdx % T12_NUM_LOCAL_OUTER_ACL_HANDLES].acked)
                    break;
            }


	        if (j == 100)  {
		        System_printf ("%s: (%s:%d): addAcl commands for entry %d was not acked (%s)\n", tfName, __FILE__, __LINE__, aclSetup[i].nHandleIdx, id);
		        return (PA_TEST_FAILED);
	        }
        }

    hd = testCommonAddAcl (tencap->tf, aclSetup[i].aclInst, aclSetup[i].action, &aclSetup[i].aclInfo,
							   &tencap->aclHandles[aclSetup[i].handleIdx].paHandle,
							   (aclSetup[i].lHandleIdx != -1)?linkedHandles[aclSetup[i].lHandleIdx].paHandle:NULL,
                               (aclSetup[i].nHandleIdx != -1)?nextHandles[aclSetup[i].nHandleIdx].paHandle:NULL,
							   tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf2,
							   &cmdReply, &cmdDest, &cmdSize, &paret);

		if (hd == NULL)  {

			/* It's not a failure if the return code from pa was expected */
			if (paret == aclSetup[i].ret)  {
				aclSetup[i].acked = TRUE;
				continue;
			}

			System_printf ("%s: (%s:%d): Failure in common addAcl command, %s entry number %d\n", tfName, __FILE__, __LINE__, id, i);
			t12HandleError (tencap, paret, hd, __LINE__);
		}
        else if (paret == pa_DUP_ENTRY) {
            /* The ack will be handle for the first entry */
            aclSetup[i].acked = TRUE;
        }

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->aclHandles[aclSetup[i].handleIdx].state = T12_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

 		t12AclCmdRep (tencap, aclSetup, n);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t12AclCmdRep (tencap, aclSetup, n);

		for (j = m = 0; j < n; j++)  {
			if (aclSetup[j].acked == TRUE)
				m += 1;
		}

		if (m == n)
			break;
		else
			utilCycleDelay (1000);
	}

	if (i == 100)  {
		System_printf ("%s: (%s:%d): Command %d (out of %d) addAcl commands were acked (%s)\n", tfName, __FILE__, __LINE__, m, n, id);
		return (PA_TEST_FAILED);
	}

	return (PA_TEST_PASSED);

}


static Cppi_HostDesc  *t12FormPktDataDescr (t12TestEncap_t *tencap, int idx)
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
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)t12PktInfo[idx].pkt), t12PktInfo[idx].pktLen);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t12PktInfo[idx].pktLen);

    return (hd);
}


/* Look for packets in the receive queue, verify the match */
int t12RxPkts (t12TestEncap_t *tencap, pauFifo_t *fifo)
{
	Cppi_HostDesc    *hd;
	unsigned int			  idx;
	int				  n;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;

	/* Look for packets in the receive queue */
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

		if (testCommonComparePktInfo (tfName, t12PktInfo[idx].info, pinfo))  {
			testCommonRecycleLBDesc (tencap->tf, hd);
			return (-1);
		}

		testCommonRecycleLBDesc (tencap->tf, hd);
	}

	return (0);

}

void t12ClrIpAck(t12IpPaSetup_t *ipSetup, int size)
{
  int i;

  for ( i = 0; i < size; i++ ) 
  {
    ipSetup->acked = FALSE;
    ipSetup ++;
  }
}

void t12ClrAclInfo(t12AclPaSetup_t *aclSetup, int size)
{
  int i;

  for ( i = 0; i < size; i++ ) 
  {
    aclSetup->acked = FALSE;
    aclSetup ++;
  }
}

#ifdef __LINUX_USER_SPACE
void* paTestACL (void *args)
{
 	void  *a0 = (void *)((paTestArgs_t *)args)->tf;
 	void  *a1 = (void *)((paTestArgs_t *)args)->pat;
#else
void paTestACL (UArg a0, UArg a1)
{
#endif
 	Cppi_HostDesc  *hd[10];
 	paReturn_t      paret;
 	int				i, j, n;
 	int  			cmdDest;
 	uint16_t		cmdSize;
 	paTestStatus_t  testStatus = PA_TEST_PASSED;
 	paTestStatus_t  newStatus;

 	unsigned int			fifoData[T12_EXPTPKT_FIFO_SIZE];
 	pauFifo_t       fifo =  { 0, 0, T12_EXPTPKT_FIFO_SIZE, NULL };

 	volatile int mdebugWait = 1;

	/* Initialize the test state */
	fifo.data = fifoData;
 	memset (&t12Encap, 0, sizeof(t12Encap));
 	t12Encap.tf  = (tFramework_t *)a0;
 	t12Encap.pat = (paTest_t *)a1;
 	for (i = 0; i < T12_NUM_LOCAL_L4_HANDLES; i++)
 		t12Encap.l4Handles[i].state = T12_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T12_NUM_LOCAL_L3_HANDLES; i++)
 		t12Encap.l3Handles[i].state = T12_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T12_NUM_LOCAL_L2_HANDLES; i++)
 		t12Encap.l2Handles[i].state = T12_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T12_NUM_LOCAL_ACL_HANDLES; i++)
 		t12Encap.aclHandles[i].state = T12_HANDLE_UNCONFIGURED;


    i = setupPktTestInfo(t12PktInfo, (sizeof(t12PktInfo) / sizeof(pktTestInfo_t)), tfName );
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(t12PktInfo): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    t12Encap.pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

    /* Runtime initial values */
    matchRoute[0].queue = (uint16_t) t12Encap.tf->QGen[Q_MATCH];
	matchRoute[0].flowId = t12Encap.tf->tfFlowNum[0];
    nfailRoute.queue    = (uint16_t) t12Encap.tf->QGen[Q_NFAIL];
    nfailRoute1.queue   = (uint16_t) t12Encap.tf->QGen[Q_MATCH];
	nfailRoute1.flowId  = t12Encap.tf->tfFlowNum[0];
    cmdReply.queue      = (uint16_t) t12Encap.tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId     = t12Encap.tf->tfFlowNum[0];

    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL4ExpectedStats, 0, sizeof(paTestL4ExpectedStats));

    /*
     * Configure the CRC engine for SCTP CRC-32C checksum
     * The CRC-engine connected to PDSP2 should be configured since the SCTP is within the
     * inner IP paylaod which is parased and lookup at PDSP2
     */
    hd[0] = testCommonConfigCrcEngine(t12Encap.tf, 2, &t12CrcCfg,  t12Encap.tf->QGen[Q_CMD_RECYCLE], t12Encap.tf->QLinkedBuf3,
                                      &cmdReply, &cmdDest, &cmdSize, &paret);

    t12HandleError (&t12Encap, paret, hd[0], __LINE__); /* Will not return on error */


 	/* Add two mac entries into the table. All the test packets will match one of these */
 	for (i = 0; i < T12_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T12_CMD_SWINFO0_ADD_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue = t12Encap.tf->QGen[Q_CMD_REPLY];

 		hd[i] = testCommonAddMac (t12Encap.tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t12EthInfo[i], &matchRoute[2], &nfailRoute,
 	    	                    &t12Encap.l2Handles[i].paHandle, t12Encap.tf->QGen[Q_CMD_RECYCLE], t12Encap.tf->QLinkedBuf1,
 	        	                &cmdReply, &cmdDest, &cmdSize, &paret);

 	    t12HandleError (&t12Encap, paret, hd[i], __LINE__); /* Will not return on error */

 	}


 	/* Send the commands to PA. Each will result in 1 packet in classify1 */
 	for (i = 0; i < T12_NUM_LOCAL_L2_HANDLES; i++)  {
 		Qmss_queuePush (t12Encap.tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		t12Encap.l2Handles[i].state = T12_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
 	}

 	/* Wait for the responses. They will be received in the order in which they were sent */
 	newStatus = testStatus;
 	for (i = 0; i < T12_NUM_LOCAL_L2_HANDLES; i++)  {

		if (testCommonWaitCmdReply (t12Encap.tf, t12Encap.pat, tfName, cmdReply.queue, T12_CMD_SWINFO0_ADD_MAC_ID + i, __LINE__)) {
			System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
			newStatus = PA_TEST_FAILED;
            break;
 		}

 		/* Recycle the command packet as well */
 		hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (t12Encap.tf->QGen[Q_CMD_RECYCLE])) & ~15);
 		if (hd[0] == NULL)  {
 			System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 			newStatus = PA_TEST_FAILED;
            break;
 		}
 		testCommonRecycleLBDesc (t12Encap.tf, hd[0]);
 	}

 	if (newStatus == PA_TEST_FAILED)
 		t12Cleanup (&t12Encap, newStatus);  /* No return */


	/* Configure Outer IP (which are linked to MAC) */
	n = sizeof (t12OuterIpInfo) / sizeof (t12IpPaSetup_t);
    t12ClrIpAck(&t12OuterIpInfo[0], n);
	newStatus = t12OpenIp (&t12Encap, t12OuterIpInfo, n, t12Encap.l2Handles, "Outer IP headers");
	if (newStatus == PA_TEST_FAILED)
		t12Cleanup (&t12Encap, newStatus);  /* No return */

	/* Repeat for Inner IP  (which are linked to outer IP packets) */
	n = sizeof (t12InnerIpInfo) / sizeof (t12IpPaSetup_t);
    t12ClrIpAck(&t12InnerIpInfo[0], n);
	newStatus = t12OpenIp (&t12Encap, t12InnerIpInfo, n, t12Encap.l3Handles, "Inner IP headers");
	if (newStatus == PA_TEST_FAILED)
		t12Cleanup (&t12Encap, newStatus);  /* No return */

	/* Configure Outer ACL entries (which may be linked to inner IP) */
	n = sizeof (t12OuterAclInfo) / sizeof (t12AclPaSetup_t);
  /* Clear the ACL info set from previous run */
    t12ClrAclInfo(&t12OuterAclInfo[0], n);
	newStatus = t12OpenAcl (&t12Encap, t12OuterAclInfo, n, t12Encap.l2Handles, t12Encap.aclHandles, "Outer ACL setup");
	if (newStatus == PA_TEST_FAILED)
		t12Cleanup (&t12Encap, newStatus);  /* No return */

	/* Repeat for Inner ACL entries packets (which may be linked to outer IP) */
	n = sizeof (t12InnerAclInfo) / sizeof (t12AclPaSetup_t);
  /* Clear the ACL info set from previous run */
    t12ClrAclInfo(&t12InnerAclInfo[0], n);
	newStatus = t12OpenAcl (&t12Encap, t12InnerAclInfo, n, t12Encap.l3Handles, t12Encap.aclHandles, "Inner ACL setup");
	if (newStatus == PA_TEST_FAILED)
		t12Cleanup (&t12Encap, newStatus);  /* No return */

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t12Encap.tf, t12Encap.pat, tfName, &paTestL4ExpectedStats, t12Encap.tf->QLinkedBuf1,
	                       t12Encap.tf->QGen[Q_CMD_RECYCLE], t12Encap.tf->QGen[Q_CMD_REPLY], TRUE);

	if (newStatus == PA_TEST_FAILED)  {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t12Cleanup (&t12Encap, newStatus);  /* No return */
	}

	/* Fire in each of the data packets multiple times */
	n = sizeof (t12PktInfo) / sizeof (pktTestInfo_t);
	for (j = 0; j < T12_NUM_PACKET_ITERATIONS; j++)  {
		for (i = 0; i < n; i++)  {
			hd[0] = t12FormPktDataDescr (&t12Encap, i);
			if (hd[0] == NULL)  {
				System_printf ("%s (%s:%d): Failed to get free descriptor\n", tfName, __FILE__, __LINE__);
				t12Cleanup (&t12Encap, PA_TEST_FAILED);
			}

			Qmss_queuePush (t12Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], t12PktInfo[i].pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
			testCommonIncStats (t12PktInfo[i].statsMap, &paTestL4ExpectedStats);

			if ((t12PktInfo[i].idx & T12_PACKET_DEST_MASK) == T12_PACKET_L3_MATCH_VALID)  {
				if (commonFifoPushElement (&fifo, (unsigned int)i) < 0)  {
					System_printf ("%s (%s:%d): Test failed - fifo is full\n", tfName, __FILE__, __LINE__);
					t12Cleanup (&t12Encap, PA_TEST_FAILED);
				}
			}
		}

        /* Delay to allow packets go through the PASS */
        utilCycleDelay (4000);

		if (t12RxPkts (&t12Encap, &fifo))
			t12Cleanup (&t12Encap, PA_TEST_FAILED);
	}

	/* Give some delay for all packets to pass through the system */
	utilCycleDelay (4000);
	if (t12RxPkts (&t12Encap, &fifo))
		t12Cleanup (&t12Encap, PA_TEST_FAILED);

	/* The packet index fifo must be empty */
	n = commonFifoGetCount (&fifo);
	if (n != 0)  {
		System_printf ("%s (%s:%d): Packet reception complete but packet index fifo not empty\n", tfName, __FILE__, __LINE__);
		t12Cleanup (&t12Encap, PA_TEST_FAILED);
	}

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t12Encap.tf, t12Encap.pat, tfName, &paTestL4ExpectedStats, t12Encap.tf->QLinkedBuf1,
	                       t12Encap.tf->QGen[Q_CMD_RECYCLE], t12Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t12Cleanup (&t12Encap, newStatus);
    }

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t12Encap.tf, t12Encap.pat, tfName, &paTestL4ExpectedStats, t12Encap.tf->QLinkedBuf1,
	                       t12Encap.tf->QGen[Q_CMD_RECYCLE], t12Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	    t12Cleanup (&t12Encap, newStatus);
    }

    /* Display ACL statistics */
    t12DispAclStats(&t12Encap);

	/* No return from cleanup */
	t12Cleanup (&t12Encap, newStatus);

#ifdef __LINUX_USER_SPACE
paTestACL_end:
    return (void *)0;
#endif

}






