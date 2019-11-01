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

/* Add/Delete ACL and ACL Filtering along with Rescore test
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

static char *tfName = "paTestACLRescore";

#define T15_NUM_PACKET_ITERATIONS	1  		/* Number of times the packet stream is passed through */

#define T15_EXPTPKT_FIFO_SIZE		20		/* Must hold the number of packets in transit in PA at one time */

#define T15_RESCORE_TRIGGER      1

/* General purpose queue usage */
#define Q_CMD_RECYCLE	 0		/* Command descriptors/buffers recycled here after sent to PA */
#define Q_CMD_REPLY  	 1		/* Replies from PA routed here */
#define Q_MATCH		  	 2		/* Packets from PA which match a lookup criteria */
#define Q_NFAIL		     3		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
#define Q_PARSE_ERR		 4		/* Packets which resulted in a parse error */


/* The number of PA L2 and L3 handles maintained by this test */
#define T15_NUM_LOCAL_L2_HANDLES   			              sizeof(t15EthInfo)/ sizeof (paEthInfo_t)
#define T15_NUM_LOCAL_L3_OUTER_IP_HANDLES	            sizeof(t15OuterIpInfo)/ sizeof (t15IpPaSetup_t)
#define T15_NUM_LOCAL_L3_INNER_IP_HANDLES	            sizeof(t15InnerIpInfo)/ sizeof (t15IpPaSetup_t)
#define T15_NUM_LOCAL_L3_HANDLES				              (T15_NUM_LOCAL_L3_OUTER_IP_HANDLES+T15_NUM_LOCAL_L3_INNER_IP_HANDLES+2)
#define T15_NUM_LOCAL_OUTER_ACL_HANDLES	              48
#define T15_NUM_LOCAL_INNER_ACL_HANDLES	              48
#define T15_NUM_LOCAL_ACL_HANDLES	  		              (T15_NUM_LOCAL_OUTER_ACL_HANDLES + T15_NUM_LOCAL_INNER_ACL_HANDLES + 1)

/* ACL entries are configured for UDP ports */
#define T15_OUTER_ACL_UDP_DST_PORT_ENTRY0             0x0555
#define T15_INNER_ACL_UDP_DST_PORT_ENTRY0             (T15_OUTER_ACL_UDP_DST_PORT_ENTRY0 + 0x100)
#define T15_NUM_ENTRIES_FOR_RESCORE                   7
#define T15_OUTER_ACL_RESCORE_TRIG_START              (T15_NUM_LOCAL_OUTER_ACL_HANDLES/2)
#define T15_OUTER_ACL_RESCORE_TRIG_END                (T15_OUTER_ACL_RESCORE_TRIG_START + T15_NUM_ENTRIES_FOR_RESCORE)
#define T15_INNER_ACL_RESCORE_TRIG_START              (T15_NUM_LOCAL_INNER_ACL_HANDLES/2)
#define T15_INNER_ACL_RESCORE_TRIG_END                (T15_INNER_ACL_RESCORE_TRIG_START + T15_NUM_ENTRIES_FOR_RESCORE)
#define T15_INNER_ACL_RESCORE_TRIG_START              (T15_NUM_LOCAL_INNER_ACL_HANDLES/2)



typedef struct t15IpPaSetup_s  {
	int         seqId;		/* Sequential enumeration to identify */
	int		      handleIdx;	/* Local handle index. Specifies which local handle corresponds to this entry */
  int         lutInst;     /* Specify which LUT1 (0-2) should be used */
	int		      lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	int		      routeIdx;	/* Which match route index to use, 0 or 1 */
	paIpInfo_t  ipInfo;		/* PA IP configuration structure */
	paReturn_t  ret;			/* Expected return code from pa_addIp command */
	Bool	      acked;		/* Set to TRUE when the reply to the command is received */

} t15IpPaSetup_t;


typedef struct t15AclPaSetup_s  {
	int         seqId;		/* Sequential enumeration to identify */
	int		      handleIdx;	/* Local handle index. Specifies which local handle corresponds to this entry */
  int         aclInst;     /* Specify which ACL LUT1 (0-1) should be used */
	int		      lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	int		      nHandleIdx;  /* Next ACL handle Index (This entry should be inserted in front of the rule specified by this one*/
	int		      action;	    /* ACL action per match */
	paAclInfo_t aclInfo;	/* PA ACL configuration structure */
	paReturn_t  ret;			/* Expected return code from pa_addIp command */
	Bool	      acked;		/* Set to TRUE when the reply to the command is received */
} t15AclPaSetup_t;

#include "test15pkts.h"

/* The total number of buffers with linked descriptors */
#define TOTAL_BUFS   (TF_LINKED_BUF_Q1_NBUFS + TF_LINKED_BUF_Q2_NBUFS + TF_LINKED_BUF_Q3_NBUFS)

 /* Commands to the PA are verified through the value in swinfo0.
  * The 16 ms bits are used as verification, the 16 lbs are for local handle id */
#define T15_CMD_SWINFO0_ADD_MAC_ID  	0x11100000  /* Identifies add mac command */
#define T15_CMD_SWINFO0_DEL_MAC_ID  	0x11110000  /* Identifies del mac command */
#define T15_CMD_SWINFO0_ADD_IP_ID		  0x22200000  /* Identifies the add IP command */
#define T15_CMD_SWINFO0_DEL_IP_ID		  0x22210000  /* Identifies the del IP command */
#define T15_CMD_SWINFO0_ADD_PORT_ID		0x22220000  /* Identifies the add port command */
#define T15_CMD_SWINFO0_DEL_PORT_ID		0x22230000  /* Identifies the del port command */
#define T15_CMD_SWINFO0_ADD_ACL_ID		0x22240000  /* Identifies the add ACL command */
#define T15_CMD_SWINFO0_DEL_ACL_ID		0x22250000  /* Identifies the del ACL command */
#define T15_CMD_SWINFO0_STATS_REQ_ID	0x33330000	/* Identifies the req stats command */
#define T15_CMD_SWINFO0_CRC_CFG_ID		0x44400000  /* Identifies the CRC config command */
#define T15_CMD_SWINFO0_GLOBAL_CFG_ID 0x44410000  /* Identifies the Global config command */
#define T15_CMD_SWINFO0_PKT_ID			  0x55550000  /* Identifies the packet as a data packet */

#define T15_CMD_SWINFO0_TYPE_MASK		  0xffff0000  /* Mask for the command type */
#define T15_CMD_SWINFO0_ID_MASK			  0x0000ffff  /* Mask for the local ID */

 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T15_HANDLE_UNCONFIGURED = 0,
	T15_HANDLE_PENDING_ACK,
	T15_HANDLE_ACTIVE,
	T15_HANDLE_DISABLED
};

typedef struct t15Handles_s  {

 	paHandleL2L3_t  paHandle;     /* The L3 handle returned by the PA LLD */

	unsigned int	state;		  /* T15_HANDLE_UNCONFIGURED = handle not configured
								   * T15_HANDLE_PENDING_ACK = handle configured and sent to pa
								   * T15_HANDLE_ACTIVE = handle creation acknowledged by pa
								   * T15_HANDLE_DISABLED = handle was created then released */
  Bool	        rsvd;
} t15Handles_t;

typedef struct t15HandlesL4_s  {

	paHandleL4_t   paHandle;

	unsigned int   state;

  Bool	         rsvd;
} t15HandlesL4_t;

typedef struct t15HandlesAcl_s  {

 	paHandleAcl_t   paHandle;     /* The ACL handle returned by the PA LLD */

	unsigned int	state;
	Bool	        acked;		/* Set to TRUE when the reply to the command is received */

} t15HandlesAcl_t;

 /* A grouping of run time created grouped together to make cleanup easier on
  * error exit */
 typedef struct t15TestEncap_s  {
 	tFramework_t  *tf;
 	paTest_t      *pat;

 	/* The +1 is a place holder handle used to pass known invalid configuration into the PA LLD */
 	t15Handles_t    l2Handles[T15_NUM_LOCAL_L2_HANDLES+1];		/* MAC handles */
 	t15Handles_t    l3Handles[T15_NUM_LOCAL_L3_HANDLES+1];		/* IP handles  */
  t15HandlesAcl_t aclHandles[T15_NUM_LOCAL_ACL_HANDLES+1];	/* ACL handles */
  uint8_t         aclMatchBitMap[T15_NUM_LOCAL_OUTER_ACL_HANDLES >> 3];

 } t15TestEncap_t;

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
static paCrcConfig_t   t15CrcCfg = {
                                    pa_CRC_CONFIG_INVERSE_RESULT,       /* ctrlBitfield */
                                    pa_CRC_SIZE_32,
                                    0x1EDC6F41,                         /* polynomial */
                                    0xFFFFFFFF                          /* initValue */
                                  };


#ifdef _TMS320C6X
#pragma DATA_SECTION(t15Encap, ".testPkts")
#endif
static t15TestEncap_t  t15Encap;

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
#pragma DATA_SECTION(t15pktBuf, ".testPkts")
#pragma DATA_ALIGN (t15pktBuf, 128)
static uint8_t  t15pktBuf[T15_NUM_LOCAL_OUTER_ACL_HANDLES][512];
#else
static uint8_t  t15pktBuf[T15_NUM_LOCAL_OUTER_ACL_HANDLES][512] __attribute__ ((aligned (128)));
#endif
#else
static uint8_t *t15pktBuf[T15_NUM_LOCAL_OUTER_ACL_HANDLES];
#endif

void t15Cleanup (t15TestEncap_t *tencap, paTestStatus_t testStatus)
{
 	int 	       i;
 	int  	       cmdDest;
 	uint16_t	   cmdSize;
 	paReturn_t     paret;
 	Cppi_HostDesc *hd;
 	paTestStatus_t newStatus;

 	/* Delete active L3 Handles */
 	for (i = 0; i < T15_NUM_LOCAL_L3_HANDLES; i++)  {
 		cmdReply.replyId = T15_CMD_SWINFO0_DEL_IP_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l3Handles[i].state == T15_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T15_HANDLE_ACTIVE))  {
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
 	for (i = 0; i < T15_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T15_CMD_SWINFO0_DEL_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->l2Handles[i].state == T15_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T15_HANDLE_ACTIVE))  {
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
 	for (i = 0; i < T15_NUM_LOCAL_ACL_HANDLES; i++)  {
 		cmdReply.replyId = T15_CMD_SWINFO0_DEL_ACL_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

		if ((tencap->aclHandles[i].state == T15_HANDLE_PENDING_ACK) || (tencap->aclHandles[i].state == T15_HANDLE_ACTIVE))  {
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

static void t15InitMatchPktIndex(t15TestEncap_t *tencap)
{
  int i;

  for ( i = 0; i < sizeof(tencap->aclMatchBitMap); i++)
    tencap->aclMatchBitMap[i] = 0;

  return;
}

static void t15SetMatchPktIndex(t15TestEncap_t *tencap, int index)
{
  int  idx = index >> 3;
  int  bitpos = index % 8;

  if (idx >= (T15_NUM_LOCAL_ACL_HANDLES >> 3) )
    return;

  tencap->aclMatchBitMap[idx] |= (1 << bitpos);

  return;
}

#define T15_DISP_ACL_ENTRY_DETAILS 0

static paTestStatus_t t15DispAclStats(t15TestEncap_t *tencap, int aclInst, int numHandles)
{
    int                i;
    paReturn_t         paret;
    paAclStats_t       aclStats;
    paTestStatus_t     newStatus = PA_TEST_PASSED;
    t15HandlesAcl_t   *aclHandles;

    if (aclInst == pa_ACL_INST_OUTER_IP)
      aclHandles = &tencap->aclHandles[0];
    else
      aclHandles = &tencap->aclHandles[T15_NUM_LOCAL_OUTER_ACL_HANDLES];

    /* Zero out the expected match pkts */
    t15InitMatchPktIndex(&t15Encap);

   	for (i = 0; i < numHandles; i++)  {

  		if ( aclHandles[i].state == T15_HANDLE_ACTIVE)  {
        paret = Pa_queryAclStats(tencap->tf->passHandle, aclHandles[i].paHandle, 0,  &aclStats);
  			if (paret != pa_OK)  {
  				System_printf ("%s (%s:%d): Pa_queryAclStats returned error code %d\n", tfName, __FILE__, __LINE__, paret);
  				continue;
  			}

        if (aclStats.nMatchPackets == 0)
           System_printf ("ACL Stats (%d): nMatchPackets = %d,  nMatchBytes = %d\n", i, aclStats.nMatchPackets, aclStats.nMatchBytes);
        else
          t15SetMatchPktIndex(tencap, i);

        System_flush();
   		}
      else {
        System_printf ("ACL handle should be active: unexpected state for aclHandles[%d].state %d \n", i, aclHandles[i].state);
      }
   	}

#if T15_DISP_ACL_ENTRY_DETAILS
    /* Compare ACL bit map, every entry should have one packet */
    for ( i = 0 ; i < (numHandles >> 3); i++) {
      System_printf("aclMatchBitMap[%d]: %d \n", i, t15Encap.aclMatchBitMap[i]);
    }
#else
    /* Compare ACL bit map, every entry should have one packet */
    for ( i = 0 ; i < (numHandles >> 3); i++) {
       if (t15Encap.aclMatchBitMap[i] != 0xFF) {
         newStatus = PA_TEST_FAILED;
         break;
       }
    }
#endif
    System_flush();
    return (newStatus);
}


/* Check for pa lld return errors. Exit the test on any error condition */
static void t15HandleError (t15TestEncap_t *tencap, paReturn_t paret, Cppi_HostDesc *hd, int line)
{

	if (paret == pa_OK)
		return;

	System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, line, paret);

	if ((hd != NULL) && testCommonRecycleLBDesc (tencap->tf, hd))
		System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);

    t15Cleanup (tencap, PA_TEST_FAILED);  /* No return */

}


 /* Look for replies to add Ip commands and verify the results */
void t15L3CmdRep (t15TestEncap_t *tencap, t15IpPaSetup_t *ipSetup)
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

		    swinfoType = swinfo[0] & T15_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T15_CMD_SWINFO0_ID_MASK;

            if (swinfoType != T15_CMD_SWINFO0_ADD_IP_ID)  {
                System_printf ("%s (%s:%d): found packet in command reply queue without add IP swinfo type (found 0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            if (swinfoIdx >= sizeof(t15OuterIpInfo)/sizeof(t15IpPaSetup_t))  {
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

            tencap->l3Handles[ipSetup[swinfoIdx].handleIdx].state = T15_HANDLE_ACTIVE;

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
void t15AclCmdRep (t15TestEncap_t *tencap, t15AclPaSetup_t *aclSetup, int maxEntries)
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

		    swinfoType = swinfo[0] & T15_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T15_CMD_SWINFO0_ID_MASK;

            if (swinfoType != T15_CMD_SWINFO0_ADD_ACL_ID)  {
                System_printf ("%s (%s:%d): found packet in command reply queue without add IP swinfo type (found 0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            if (swinfoIdx >= maxEntries)  {
            	System_printf ("%s (%s:%d): found packet in command reply queue with invalid index (found 0x%08x)\n", tfName, __FILE__, __LINE__, swinfoIdx);
            	testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            aclSetup[swinfoIdx].acked = tencap->aclHandles[aclSetup[swinfoIdx].handleIdx].acked = TRUE;
            paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);

            if (paret != aclSetup[swinfoIdx].ret)  {
                System_printf ("%s (%s:%d): Pa_forwardResult returned %d, expected %d\n", tfName, __FILE__, __LINE__, paret, aclSetup[swinfoIdx].ret);
                testCommonRecycleLBDesc (tencap->tf, hd);
                continue;
            }

            testCommonRecycleLBDesc (tencap->tf, hd);

            tencap->aclHandles[aclSetup[swinfoIdx].handleIdx].state = T15_HANDLE_ACTIVE;

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
static int t15GetLUT1Index(int handleIndex)
{
    if (handleIndex >= T15_NUM_LOCAL_L3_HANDLES)
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


static paTestStatus_t t15OpenIp (t15TestEncap_t  *tencap, t15IpPaSetup_t *ipSetup, int n, t15Handles_t *linkedHandles, char *id)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    paRouteInfo_t  *pNfailRoute;

	for (i = 0; i < n; i++)  {
		//cmdReply.replyId = T15_CMD_SWINFO0_ADD_IP_ID + ipSetup[i].seqId;
        cmdReply.replyId = T15_CMD_SWINFO0_ADD_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

        pNfailRoute = (ipSetup[i].routeIdx == 2)?&nfailRoute1:&nfailRoute;
		hd = testCommonAddIp2 (tencap->tf, ipSetup[i].lutInst, t15GetLUT1Index(ipSetup[i].handleIdx), &ipSetup[i].ipInfo, &matchRoute[ipSetup[i].routeIdx], pNfailRoute,
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
			t15HandleError (tencap, paret, hd, __LINE__);
		}
        else if (paret == pa_DUP_ENTRY) {
            /* The ack will be handle for the first entry */
            ipSetup[i].acked = TRUE;
        }

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[ipSetup[i].handleIdx].state = T15_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

 		t15L3CmdRep (tencap, ipSetup);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t15L3CmdRep (tencap, ipSetup);

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

static Cppi_HostDesc  *t15FormPktDataDescr (t15TestEncap_t *tencap, int idx)
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
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)t15PktInfo[idx].pkt), t15PktInfo[idx].pktLen);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t15PktInfo[idx].pktLen);

    return (hd);
}

static paAclInfo_t aclInfoDummy =  {
                              pa_ACL_INFO_VALID_DST_PORT,                               /* validbitMap */
                              0,                                                    /* ctrlFlag */
                              0,                                                    /* ctrlFlagMask */
                              pa_IPV4,  /* IP Type */
                              { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
                              { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
                              { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
                              { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
                              0,      /* Protocol */
                              0,      /* DSCP */
                              0,          /* srcPortbegin */
                              0,          /* srcPort end */
                              0x0786,     /* destPortbegin */
                              0x0786,     /* destPort end */
                            };

static paTestStatus_t t15OpenAcl (t15TestEncap_t  *tencap, t15AclPaSetup_t *aclSetup, int n, t15Handles_t *linkedHandles, t15HandlesAcl_t *nextHandles,  char *id)
{
	int 			      i, j, m, index;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			      cmdDest;
	uint16_t			  cmdSize;

	for (i = 0; i < n; i++)  {
    cmdReply.replyId = T15_CMD_SWINFO0_ADD_ACL_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

    if (aclSetup[i].nHandleIdx != -1) {
      for (j = 0; j < 100; j++) {
        /* wait for previous entry to be completed */
	      utilCycleDelay (1000);
        t15AclCmdRep (tencap, aclSetup, n);

        if (aclSetup[i].aclInst == pa_ACL_INST_OUTER_IP) {
          index = aclSetup[i].nHandleIdx % T15_NUM_LOCAL_OUTER_ACL_HANDLES;
          if(tencap->aclHandles[index].acked)
            break;
        }
        else {
          index  = aclSetup[i].nHandleIdx % T15_NUM_LOCAL_INNER_ACL_HANDLES;
          index += T15_NUM_LOCAL_OUTER_ACL_HANDLES;
          if(tencap->aclHandles[index].acked)
            break;
        }
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
        tencap->aclHandles[aclSetup[i].handleIdx].acked = TRUE;
				continue;
			}

			System_printf ("%s: (%s:%d): Failure in common addAcl command \n", tfName, __FILE__, __LINE__, id);
			t15HandleError (tencap, paret, hd, __LINE__);
		}
    else if (paret == pa_DUP_ENTRY) {
       /* The ack will be handle for the first entry */
       aclSetup[i].acked = TRUE;
       tencap->aclHandles[aclSetup[i].handleIdx].acked = TRUE;
    }

		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

    /* When the rescore command is triggered, fire another one to test the LLD for getting ACL Busy */
#if 1
    /* Fire another Entry (dummy) just after rescore is triggered */
    if (aclSetup[i].aclInfo.dstPortEnd == (T15_OUTER_ACL_UDP_DST_PORT_ENTRY0 + T15_OUTER_ACL_RESCORE_TRIG_START)) {

    		hd = testCommonAddAcl (tencap->tf, pa_ACL_INST_OUTER_IP, pa_ACL_ACTION_PERMIT, &aclInfoDummy,
    							   &tencap->aclHandles[T15_NUM_LOCAL_ACL_HANDLES-1].paHandle,
    							   NULL,  NULL,
    							   tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf2,
    							   &cmdReply, &cmdDest, &cmdSize, &paret);

        if (paret == pa_ACL_BUSY)
        {
          /* No action during Busy, this is just a check that LLD is returning Busy signal for the application 
           * Uncomment below System_printf() to see BUSY signal indications getting printed in the unit test
           */
          // System_printf("  Pa_addAcl returned ACL HW busy indication due to ACL busy indication \n");
          // System_flush();
        }
        else if ((hd == NULL) ||
                 (paret != pa_OK)) {
          System_printf ("%s: (%s:%d): Failure in common addAcl command \n", tfName, __FILE__, __LINE__, id);
          t15HandleError (tencap, paret, hd, __LINE__);
        }
        else {
          tencap->aclHandles[T15_NUM_LOCAL_ACL_HANDLES-1].state = T15_HANDLE_PENDING_ACK;
          /* Restore the descriptor and return it */
          hd->buffLen = hd->origBufferLen;
          Qmss_queuePush (tencap->tf->QLinkedBuf2, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
        }
    }
#endif

 		tencap->aclHandles[aclSetup[i].handleIdx].state = T15_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;

 		t15AclCmdRep (tencap, aclSetup, n);
	}

	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t15AclCmdRep (tencap, aclSetup, n);

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

static void t15InsertAclEntries(int aclInst, int udpPortStart, int rescoreStart, int rescoreEnd, int numHandles)
{
  int i;
  paTestStatus_t  newStatus;
  t15AclPaSetup_t aclPaSetup;

  for ( i = 0; i < numHandles; i++)  {
    aclPaSetup                      = t15AclInfoTemplate[0];
    aclPaSetup.acked                = FALSE;
    aclPaSetup.aclInst              = aclInst;
    aclPaSetup.aclInfo.dstPortBegin = udpPortStart;
    aclPaSetup.aclInfo.dstPortEnd   = udpPortStart + i;
    aclPaSetup.seqId                = i;
    aclPaSetup.handleIdx            = i;
    if (aclInst != pa_ACL_INST_OUTER_IP) {
      aclPaSetup.handleIdx        += T15_NUM_LOCAL_OUTER_ACL_HANDLES;
    }

#if T15_RESCORE_TRIGGER
    if ( (i >= rescoreStart) ) {

        if (i  < rescoreEnd )  {
         /* Insert them later to initiate rescore operations */
         continue;
        }
    }
#endif
    if ( aclInst == pa_ACL_INST_OUTER_IP )
      newStatus = t15OpenAcl (&t15Encap, &aclPaSetup, 1, t15Encap.l2Handles, t15Encap.aclHandles, "Outer ACL setup");
    else
      newStatus = t15OpenAcl (&t15Encap, &aclPaSetup, 1, t15Encap.l3Handles, t15Encap.aclHandles, "Inner ACL setup");

	  if (newStatus == PA_TEST_FAILED)
		  t15Cleanup (&t15Encap, newStatus);  /* No return */
  }
  return;
}

static void t15TrigAclRescoreFireDataPkts(int aclInst, int udpPortBase, int rescoreStart, int rescoreEnd,int numHandles)
{
  t15AclPaSetup_t aclPaSetup;
  int             i, nextAclIndex;
 	Cppi_HostDesc  *hd[T15_NUM_LOCAL_ACL_HANDLES];
  paTestStatus_t  newStatus;
  uint8_t        *buf;


  for ( i = 0; i < numHandles; i++)  {
    t15UdpHdr_t  *udpHdr;
    uint8_t       udpOffset;
    uint16_t      port;
    memset(&t15PktInfo[i], 0, sizeof (pktTestInfo_t) );
    buf    = t15pktBuf[i];

    if ( aclInst == pa_ACL_INST_OUTER_IP ) {
      memcpy(buf, pkt0Template, sizeof (pkt0Template));
      udpOffset           = PASAHO_LINFO_READ_START_OFFSET(&pkt0TemplateInfo);
      t15PktInfo[i].pktLen = sizeof (pkt0Template);
      t15PktInfo[i].info   = &pkt0TemplateInfo;
    }
    else  {
      memcpy(buf, pkt1Template, sizeof (pkt1Template));
      udpOffset           = PASAHO_LINFO_READ_START_OFFSET(&pkt1TemplateInfo);
      t15PktInfo[i].pktLen = sizeof (pkt1Template);
      t15PktInfo[i].info   = &pkt1TemplateInfo;
      t15PktInfo[i].statsMap[2] = (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4);  	/* Inner IP match */
    }
    udpHdr              = (t15UdpHdr_t *) (&buf[udpOffset]);
    port                = udpPortBase + i;
    udpHdr->dstPort[1]  = (port >> 0) & 0x00FF; // T15_SWIZ(port);
    udpHdr->dstPort[0]  = (port >> 8) & 0x00FF;
    t15PktInfo[i].pkt   = buf;
    t15PktInfo[i].statsMap[0] = (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH); /* MAC match */
    t15PktInfo[i].statsMap[1] = (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4);  	/* Outer IP match */
  }

	/* Prepare to Fire in each of the data packets */
  for (i = 0; i < numHandles; i++)  {
    hd[i] = t15FormPktDataDescr (&t15Encap, i);
    if (hd[i] == NULL)  {
      System_printf ("%s (%s:%d): Failed to get free descriptor\n", tfName, __FILE__, __LINE__);
			t15Cleanup (&t15Encap, PA_TEST_FAILED);
    }
    testCommonIncStats (t15PktInfo[i].statsMap, &paTestL4ExpectedStats);
	}

#if T15_RESCORE_TRIGGER
  /* Simulate the Rescore operations */
  nextAclIndex = rescoreEnd;
  for ( i = (rescoreEnd-1); i >= rescoreStart; i--)  {
    aclPaSetup                      = t15AclInfoTemplate[0];
    aclPaSetup.aclInst              = aclInst;
    aclPaSetup.aclInfo.dstPortBegin = udpPortBase;
    aclPaSetup.aclInfo.dstPortEnd   = udpPortBase + i;
    aclPaSetup.seqId                = i;
    aclPaSetup.handleIdx            = i;
    aclPaSetup.nHandleIdx           = nextAclIndex;
    if (aclInst != pa_ACL_INST_OUTER_IP) {
      aclPaSetup.handleIdx        += T15_NUM_LOCAL_OUTER_ACL_HANDLES;
      aclPaSetup.nHandleIdx       += T15_NUM_LOCAL_OUTER_ACL_HANDLES;
    }

    if ( aclInst == pa_ACL_INST_OUTER_IP ) {
      newStatus = t15OpenAcl (&t15Encap, &aclPaSetup, 1, t15Encap.l2Handles, t15Encap.aclHandles, "Outer ACL setup");
    }
    else {
      newStatus = t15OpenAcl (&t15Encap, &aclPaSetup, 1, t15Encap.l3Handles, t15Encap.aclHandles, "Inner ACL setup");
    }

	  if (newStatus == PA_TEST_FAILED)
		  t15Cleanup (&t15Encap, newStatus);  /* No return */
    /* Make this entry as next ACL entry, so that the last entry triggers the rescore, followed by packets fired */
    nextAclIndex = i;
  }
#endif
	/* Fire in each of the data packets multiple times */
  for (i = 0; i < numHandles; i++)  {
    Qmss_queuePush (t15Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[i], t15PktInfo[i].pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
	}

  return;
}

/* Look for packets in the receive queue, verify the match */
int t15RxPkts (t15TestEncap_t *tencap, int outer)
{
	Cppi_HostDesc    *hd;
	pasahoLongInfo_t *pinfo, *expInfo;
	uint32_t		      infoLen;

	/* Look for packets in the receive queue */
	while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_MATCH]) > 0)  {

		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor off the receive packet queue\n", tfName, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tencap->tf, hd);
			return (-1);
		}

		/* Verify the parse information is correct */
		if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
			System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tencap->tf, hd);
			return (-1);
		}

    if (outer) {
       expInfo = &pkt0TemplateInfo;
    }
    else {
       expInfo = &pkt1TemplateInfo;
    }

		if (testCommonComparePktInfo (tfName, expInfo, pinfo))  {
			testCommonRecycleLBDesc (tencap->tf, hd);
			return (-1);
		}

		testCommonRecycleLBDesc (tencap->tf, hd);
	}

	return (0);

}

void t15ClrIpAck(t15IpPaSetup_t *ipSetup, int size)
{
  int i;

  for ( i = 0; i < size; i++ ) 
  {
    ipSetup->acked = FALSE;
    ipSetup ++;
  }
}

#ifdef __LINUX_USER_SPACE
void* paTestACLRescore (void *args)
{
 	void  *a0 = (void *)((paTestArgs_t *)args)->tf;
 	void  *a1 = (void *)((paTestArgs_t *)args)->pat;
#else
void paTestACLRescore (UArg a0, UArg a1)
{
#endif
   	Cppi_HostDesc  *hd[T15_NUM_LOCAL_OUTER_ACL_HANDLES];
   	paReturn_t      paret;
   	int				      i, n;
   	int  			      cmdDest;
   	uint16_t		    cmdSize;
   	paTestStatus_t  testStatus = PA_TEST_PASSED;
   	paTestStatus_t  newStatus;
   	volatile int    mdebugWait = 1;

  	/* Initialize the test state */
   	memset (&t15Encap, 0, sizeof(t15Encap));
   	t15Encap.tf  = (tFramework_t *)a0;
   	t15Encap.pat = (paTest_t *)a1;

   	for (i = 0; i < T15_NUM_LOCAL_L3_HANDLES; i++)
   		t15Encap.l3Handles[i].state = T15_HANDLE_UNCONFIGURED;
   	for (i = 0; i < T15_NUM_LOCAL_L2_HANDLES; i++)
   		t15Encap.l2Handles[i].state = T15_HANDLE_UNCONFIGURED;
   	for (i = 0; i < T15_NUM_LOCAL_ACL_HANDLES; i++) {
   		t15Encap.aclHandles[i].state = T15_HANDLE_UNCONFIGURED;
      t15Encap.aclHandles[i].acked = FALSE;
    }

#ifdef __LINUX_USER_SPACE
        for (i = 0; i < T15_NUM_LOCAL_OUTER_ACL_HANDLES; i++)  {
            /* Allocate memory for the Packet buffers */
            t15pktBuf[i] = (uint8_t *)fw_memAlloc(512, CACHE_LINESZ);
            if(t15pktBuf[i] == NULL) {
                printf ("%s: memAlloc failed for pkt %d\n", tfName, i);
            t15Encap.pat->testStatus = PA_TEST_FAILED;
                return (void *)0;
            }
        }
#endif

    /* Runtime initial values */
    matchRoute[0].queue   = (uint16_t) t15Encap.tf->QGen[Q_MATCH];
	  matchRoute[0].flowId  = t15Encap.tf->tfFlowNum[0];

    nfailRoute1.queue     = (uint16_t) t15Encap.tf->QGen[Q_MATCH];
	  nfailRoute1.flowId    = t15Encap.tf->tfFlowNum[0];

    nfailRoute.queue      = (uint16_t) t15Encap.tf->QGen[Q_NFAIL];
	  nfailRoute.flowId     = t15Encap.tf->tfFlowNum[0];

    cmdReply.queue        = (uint16_t) t15Encap.tf->QGen[Q_CMD_REPLY];
	  cmdReply.flowId       = t15Encap.tf->tfFlowNum[0];

    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL4ExpectedStats, 0, sizeof(paTestL4ExpectedStats));

    /*
     * Configure the CRC engine for SCTP CRC-32C checksum
     * The CRC-engine connected to PDSP2 should be configured since the SCTP is within the
     * inner IP paylaod which is parased and lookup at PDSP2
     */
    hd[0] = testCommonConfigCrcEngine(t15Encap.tf, 2, &t15CrcCfg,  t15Encap.tf->QGen[Q_CMD_RECYCLE], t15Encap.tf->QLinkedBuf3,
                                      &cmdReply, &cmdDest, &cmdSize, &paret);

    t15HandleError (&t15Encap, paret, hd[0], __LINE__); /* Will not return on error */


   	/* Add two mac entries into the table. All the test packets will match one of these */
   	for (i = 0; i < T15_NUM_LOCAL_L2_HANDLES; i++)  {
   		cmdReply.replyId = T15_CMD_SWINFO0_ADD_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
   		cmdReply.queue = t15Encap.tf->QGen[Q_CMD_REPLY];

   		hd[i] = testCommonAddMac (t15Encap.tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t15EthInfo[i], &matchRoute[2], &nfailRoute,
   	    	                    &t15Encap.l2Handles[i].paHandle, t15Encap.tf->QGen[Q_CMD_RECYCLE], t15Encap.tf->QLinkedBuf1,
   	        	                &cmdReply, &cmdDest, &cmdSize, &paret);

   	    t15HandleError (&t15Encap, paret, hd[i], __LINE__); /* Will not return on error */

   	}


   	/* Send the commands to PA. Each will result in 1 packet in classify1 */
   	for (i = 0; i < T15_NUM_LOCAL_L2_HANDLES; i++)  {
   		Qmss_queuePush (t15Encap.tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
   		t15Encap.l2Handles[i].state = T15_HANDLE_PENDING_ACK;
   		paTestL4ExpectedStats.classify1.nPackets += 1;
   	}

   	/* Wait for the responses. They will be received in the order in which they were sent */
   	newStatus = testStatus;
   	for (i = 0; i < T15_NUM_LOCAL_L2_HANDLES; i++)  {

  		if (testCommonWaitCmdReply (t15Encap.tf, t15Encap.pat, tfName, cmdReply.queue, T15_CMD_SWINFO0_ADD_MAC_ID + i, __LINE__)) {
  			System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
  			newStatus = PA_TEST_FAILED;
              break;
   		}

   		/* Recycle the command packet as well */
   		hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (t15Encap.tf->QGen[Q_CMD_RECYCLE])) & ~15);
   		if (hd[0] == NULL)  {
   			System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
   			newStatus = PA_TEST_FAILED;
              break;
   		}
   		testCommonRecycleLBDesc (t15Encap.tf, hd[0]);
   	}

 	if (newStatus == PA_TEST_FAILED)
 		t15Cleanup (&t15Encap, newStatus);  /* No return */


  	/* Configure Outer IP (which are linked to MAC) */
  	n = sizeof (t15OuterIpInfo) / sizeof (t15IpPaSetup_t);
    t15ClrIpAck(&t15OuterIpInfo[0], n);
  	newStatus = t15OpenIp (&t15Encap, t15OuterIpInfo, n, t15Encap.l2Handles, "Outer IP headers");
  	if (newStatus == PA_TEST_FAILED)
  		t15Cleanup (&t15Encap, newStatus);  /* No return */

  	/* Repeat for Inner IP  (which are linked to outer IP packets) */
  	n = sizeof (t15InnerIpInfo) / sizeof (t15IpPaSetup_t);
    t15ClrIpAck(&t15InnerIpInfo[0], n);
  	newStatus = t15OpenIp (&t15Encap, t15InnerIpInfo, n, t15Encap.l3Handles, "Inner IP headers");
  	if (newStatus == PA_TEST_FAILED)
  		t15Cleanup (&t15Encap, newStatus);  /* No return */


  	/* Configure Outer ACL entries (which may be linked to inner IP) */
    t15InsertAclEntries( pa_ACL_INST_OUTER_IP, T15_OUTER_ACL_UDP_DST_PORT_ENTRY0,           \
                         T15_OUTER_ACL_RESCORE_TRIG_START, T15_OUTER_ACL_RESCORE_TRIG_END,  \
                         T15_NUM_LOCAL_OUTER_ACL_HANDLES);

  	/* Verify and clear the stats */
  	newStatus =  testCommonCheckStats (t15Encap.tf, t15Encap.pat, tfName, &paTestL4ExpectedStats, t15Encap.tf->QLinkedBuf1,
  	                       t15Encap.tf->QGen[Q_CMD_RECYCLE], t15Encap.tf->QGen[Q_CMD_REPLY], TRUE);

  	if (newStatus == PA_TEST_FAILED)  {
  		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
  		t15Cleanup (&t15Encap, newStatus);  /* No return */
  	}

    /* Prepare packets associated for each ACL entry */
    t15TrigAclRescoreFireDataPkts( pa_ACL_INST_OUTER_IP, T15_OUTER_ACL_UDP_DST_PORT_ENTRY0,           \
                                   T15_OUTER_ACL_RESCORE_TRIG_START, T15_OUTER_ACL_RESCORE_TRIG_END,  \
                                   T15_NUM_LOCAL_OUTER_ACL_HANDLES);

  	/* Give some delay for all packets to pass through the system */
  	utilCycleDelay (100000);
  	if (t15RxPkts (&t15Encap, 1))
  		t15Cleanup (&t15Encap, PA_TEST_FAILED);

  	/* Verify and clear the stats */
  	newStatus =  testCommonCheckStats (t15Encap.tf, t15Encap.pat, tfName, &paTestL4ExpectedStats, t15Encap.tf->QLinkedBuf1,
  	                       t15Encap.tf->QGen[Q_CMD_RECYCLE], t15Encap.tf->QGen[Q_CMD_REPLY], TRUE);
  	if (newStatus == PA_TEST_FAILED)
      {
  		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
  	    t15Cleanup (&t15Encap, newStatus);
      }

    /* Display ACL statistics */
    newStatus = t15DispAclStats(&t15Encap, pa_ACL_INST_OUTER_IP, T15_NUM_LOCAL_OUTER_ACL_HANDLES);

  	if (newStatus == PA_TEST_FAILED)
    {
  		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
     	/* No return from cleanup */
  	  t15Cleanup (&t15Encap, newStatus);
    }

    /* Repeat for Inner ACL */
  	/* Configure Inner ACL entries (which may be linked to inner IP) */
    t15InsertAclEntries( pa_ACL_INST_INNER_IP, T15_INNER_ACL_UDP_DST_PORT_ENTRY0,           \
                         T15_INNER_ACL_RESCORE_TRIG_START, T15_INNER_ACL_RESCORE_TRIG_END,  \
                         T15_NUM_LOCAL_INNER_ACL_HANDLES);

  	/* Verify and clear the stats */
  	newStatus =  testCommonCheckStats (t15Encap.tf, t15Encap.pat, tfName, &paTestL4ExpectedStats, t15Encap.tf->QLinkedBuf1,
  	                       t15Encap.tf->QGen[Q_CMD_RECYCLE], t15Encap.tf->QGen[Q_CMD_REPLY], TRUE);

  	if (newStatus == PA_TEST_FAILED)  {
  		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
  		t15Cleanup (&t15Encap, newStatus);  /* No return */
  	}

    /* Prepare packets associated for each ACL entry */
    t15TrigAclRescoreFireDataPkts( pa_ACL_INST_INNER_IP, T15_INNER_ACL_UDP_DST_PORT_ENTRY0,           \
                                   T15_INNER_ACL_RESCORE_TRIG_START, T15_INNER_ACL_RESCORE_TRIG_END,  \
                                   T15_NUM_LOCAL_INNER_ACL_HANDLES);

  	/* Give some delay for all packets to pass through the system */
  	utilCycleDelay (100000);
  	if (t15RxPkts (&t15Encap, 0))
  		t15Cleanup (&t15Encap, PA_TEST_FAILED);

  	/* Verify and clear the stats */
  	newStatus =  testCommonCheckStats (t15Encap.tf, t15Encap.pat, tfName, &paTestL4ExpectedStats, t15Encap.tf->QLinkedBuf1,
  	                       t15Encap.tf->QGen[Q_CMD_RECYCLE], t15Encap.tf->QGen[Q_CMD_REPLY], TRUE);
  	if (newStatus == PA_TEST_FAILED)
      {
  		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
  	    t15Cleanup (&t15Encap, newStatus);
      }

    /* Display ACL statistics */
    newStatus = t15DispAclStats(&t15Encap, pa_ACL_INST_INNER_IP, T15_NUM_LOCAL_INNER_ACL_HANDLES);

  	if (newStatus == PA_TEST_FAILED)
    {
  		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
    }

  	/* No return from cleanup */
  	t15Cleanup (&t15Encap, newStatus);

#ifdef __LINUX_USER_SPACE
      return (void *)0;
#endif

}






