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

/* This test suite verifies the following PASS features:
 *  - Add/Delete LUT2 (UDP/TCP/GTP-U) entries 
 *  - LUT2 FULL test
 *
 * This test invokes the following LLD APIs:
 *  - Pa_addMac
 *  - Pa_addIp
 *  - Pa_addPort
 *  - Pa_delL4Handle
 *  - Pa_delHandle
 *  - Pa_forwardResult
 *  - Pa_control
 *  - Pa_formatTxRoute
 *  - Pa_requestStats
 *  - Pa_formatStatsReply
 *  - Pa_requestUsrStats
 *  - Pa_formatUsrStatsReply
 *
 * This test verifies PDSP firmware for checking LUT2 full signal after all LUT2 entries are utilized. 
 */

 static char *tfName = "paTestLUT2Full";
 
 #define T16_NUM_PACKET_ITERATIONS	1  		/* Number of times the packet stream is passed through */
 #define T16_FIRST_DEST_PORT			  0	    /* First TCP/UDP destination port used */

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

#include "test16pkts.h"
 
/* The number of PA L2 and L3 handles maintained by this test */
#define T16_NUM_LOCAL_L2_HANDLES   			(sizeof(t16EthSetup)/sizeof(t16EthSetup_t))
#define T16_NUM_LOCAL_L3_HANDLES				(sizeof(t16IpSetup)/sizeof(t16IpSetup_t))
#define T16_NUM_LOCAL_L4_HANDLES	  	     T16_SOC_MAX_LUT2_SIZE
 
 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	T16_HANDLE_UNCONFIGURED = 0,
	T16_HANDLE_PENDING_ACK,
	T16_HANDLE_ACTIVE,
	T16_HANDLE_DISABLED
};

#define T16_CMD_SWINFO0_TYPE_MASK	0xffff0000
#define T16_CMD_SWINFO0_ID_MASK		0x0000ffff

/* SWInfo values on command replies */
#define T16_CMD_SWINFO0_ADD_MAC_ID  		0x51110000
#define T16_CMD_SWINFO0_ADD_IP_ID   		0x51120000
#define T16_CMD_SWINFO0_ADD_PORT_ID	   	0x51130000
#define T16_CMD_SWINFO0_ADD_TEID_ID	   	0x51140000
#define T16_CMD_SWINFO0_EROUTE_CFG_ID   	0x51150000
#define T16_CMD_SWINFO0_CMDSET_CFG_ID   	0x51160000
#define T16_CMD_SWINFO0_GLOBAL_CFG_ID   	0x51170000  
#define T16_CMD_SWINFO0_USR_STATS_CFG_ID 0x51180000  
#define T16_CMD_SWINFO0_NAT_T_CFG_ID   	0x51190000  
#define T16_CMD_SWINFO0_GTPU_CFG_ID   	0x511a0000  
#define T16_CMD_SWINFO0_ADD_SPI_IP_ID   	0x511b0000

#define T16_CMD_SWINFO0_DEL_MAC_ID		0x50010000
#define T16_CMD_SWINFO0_DEL_IP_ID		0x50020000
#define T16_CMD_SWINFO0_DEL_PORT_ID		0x50030000
#define T16_CMD_SWINFO0_DEL_TEID_ID	   	0x50040000
#define T16_CMD_SWINFO0_PKT_ID		    0x50050000
#define T16_CMD_SWINFO0_GLOB_PKT_ID 	    0x50060000
#define T16_CMD_SWINFO0_DEL_SPI_IP_ID    0x50070000


 typedef struct t16Handles_s  {
 
  	paHandleL2L3_t  paHandle;     /* The L3 handle returned by the PA LLD */

 	unsigned int	state;		  /* T4_HANDLE_UNCONFIGURED = handle not configured
 						           * T4_HANDLE_PENDING_ACK = handle configured and sent to pa
 						           * T4_HANDLE_ACTIVE = handle creation acknowledged by pa
 						           * T4_HANDLE_DISABLED = handle was created then released */
    unsigned int    linkCnt;                               
 	
 } t16Handles_t;
 
 typedef struct t16HandlesL4_s  {
 	
 	paHandleL4_t   paHandle;
 	
 	unsigned int   state;

 } t16HandlesL4_t;
 
 /* A grouping of run time created grouped together to make cleanup easier on
  * error exit */

 typedef struct t16TestEncap_s  {
 	tFramework_t  *tf;
 	paTest_t      *pat;
 	
 	/* There is one base packet for each L3 table entry */
 	Cppi_HostDesc  *hd[T16_NUM_LOCAL_L3_HANDLES];
 	
 	/* The command to the modify PDSP to add the TCP/UDP checksum and route to PA receive */
 	uint32_t   cmdStack[T16_NUM_LOCAL_L3_HANDLES][(sizeof(pasahoNextRoute_t) + sizeof(pasahoComChkCrc_t)) / sizeof (uint32_t)];
 	
 	/* The +1 is a place holder handle used to pass known invalid configuration into the PA LLD */
 	t16Handles_t    l2Handles[T16_NUM_LOCAL_L2_HANDLES];		/* MAC handles */
 	t16Handles_t    l3Handles[T16_NUM_LOCAL_L3_HANDLES];		/* IP handles  */
 	t16HandlesL4_t  l4Handles[T16_NUM_LOCAL_L4_HANDLES+1];	/* UDP/TCP handles */
 } t16TestEncap_t;
 
static paSysStats_t paTestL4ExpectedStats;    /* Expected stats results */

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

#pragma DATA_SECTION(t16Encap, ".testPkts") 
static t16TestEncap_t  t16Encap;

static void t16Cleanup (t16TestEncap_t *tencap, paTestStatus_t status)
{
	int			     i, retval;		
	int  	       cmdDest;
 	uint16_t	   cmdSize;
 	paReturn_t     paret;
 	paTestStatus_t newStatus;
 	Cppi_HostDesc *hd;
 	
 	
	/* Wait a bit for any packets in PA to complete */
	utilCycleDelay (5000);
	
	/* Return the descriptors which are pointing to packets */
	for (i = 0; i < T16_NUM_LOCAL_L3_HANDLES; i++)  {
		if (tencap->hd[i] != NULL)
			Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
	}

 	System_flush ();
	
 	/* Delete active L4 handles */
 	for (i = 0; i < T16_NUM_LOCAL_L4_HANDLES; i++)  {
		
		cmdReply.replyId = T16_CMD_SWINFO0_DEL_PORT_ID + i; 
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l4Handles[i].state == T16_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T16_HANDLE_ACTIVE))  {
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
        	utilCycleDelay (100000);
            retval = testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__);

	        if (retval != 0)
            {
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

  	System_flush ();    
  	/* Delete active L3 Handles */
 	for (i = 0; i < T16_NUM_LOCAL_L3_HANDLES; i++)  {
 		cmdReply.replyId = T16_CMD_SWINFO0_DEL_IP_ID + i; 
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l3Handles[i].state == T16_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T16_HANDLE_ACTIVE))  {
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
  System_flush ();

  	/* Delete active L2 Handles */
 	for (i = 0; i < T16_NUM_LOCAL_L2_HANDLES; i++)  {
 		cmdReply.replyId = T16_CMD_SWINFO0_DEL_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];
		
		if ((tencap->l2Handles[i].state == T16_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T16_HANDLE_ACTIVE))  {
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
 	  	System_flush ();
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

static paReturn_t t16CmdRep (t16TestEncap_t *tencap)
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
		    
		    swinfoType = swinfo[0] & T16_CMD_SWINFO0_TYPE_MASK;
		    swinfoIdx  = swinfo[0] & T16_CMD_SWINFO0_ID_MASK;
		    
		    paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);

		    
		    switch (swinfoType)  {
		    	
		    	case T16_CMD_SWINFO0_ADD_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T16_HANDLE_ACTIVE;
		    		max = T16_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_addMac";
		    		break;
		    	
		    	case T16_CMD_SWINFO0_ADD_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T16_HANDLE_ACTIVE;
		    		max = T16_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_addIp";
		    		break;
            
		    	case T16_CMD_SWINFO0_ADD_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T16_HANDLE_ACTIVE;
		    		max = T16_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_addPort";
		    		break;
                    
		    	case T16_CMD_SWINFO0_DEL_MAC_ID:
		    		stateP = &tencap->l2Handles[swinfoIdx].state;
		    		stateV = T16_HANDLE_DISABLED;
		    		max = T16_NUM_LOCAL_L2_HANDLES;
		    		s = "pa_delMac";
		    		break;
		    		
		    	case T16_CMD_SWINFO0_DEL_IP_ID:
		    		stateP = &tencap->l3Handles[swinfoIdx].state;
		    		stateV = T16_HANDLE_DISABLED;
		    		max = T16_NUM_LOCAL_L3_HANDLES;
		    		s = "pa_delIp";
		    		break;

		    	case T16_CMD_SWINFO0_DEL_PORT_ID:
		    		stateP = &tencap->l4Handles[swinfoIdx].state;
		    		stateV = T16_HANDLE_DISABLED;
		    		max = T16_NUM_LOCAL_L4_HANDLES;
		    		s = "pa_delPort";
		    		break;
                    
		    	default:
		    		System_printf ("%s (%s:%d): Unknown command ID found in swinfo0 (0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
		    		t16Cleanup (tencap, PA_TEST_FAILED);
		    		break;
		    	
		    }
		 		    
		    /* In this test only valid responses are exected from PA */
		    if (paret != pa_OK)  {

		    	if (paret != pa_LUT2_TABLE_FULL) {
			    	System_printf ("%s (%s:%d): PA command %s returned error code %d, Index = %d\n", tfName, __FILE__, __LINE__, s, paret, swinfoIdx);
		    		t16Cleanup (tencap, PA_TEST_FAILED);
		    	}
		    }
		    	
		    
		   	if (swinfoIdx >= max)  {
		   		System_printf ("%s (%s:%d): Received command ack (%s) for out of range handle (%d, max = %d)\n",
		   						tfName, __FILE__, __LINE__, s, swinfoIdx, max);
		   		t16Cleanup (tencap, PA_TEST_FAILED);
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
	
    return (paret);
}

static paTestStatus_t t16OpenL2 (t16TestEncap_t *tencap, t16EthSetup_t *ethSetup, int nL2Handles)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
	
	for (i = 0; i < nL2Handles; i++)  {
		cmdReply.replyId = T16_CMD_SWINFO0_ADD_MAC_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
        
        /* Specify the user statistics associated with the MAC input */
        //cmdInfo.params.usrStats.index = t16L2UsrStats[i];
		
		hd = testCommonAddMac (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&t16EthSetup[i].ethInfo, &matchRoute[1], &nfailRoute,
 	    	                   &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, 
 	        	               &cmdReply, &cmdDest, &cmdSize, &paret);
								 
		if (hd == NULL)  {
					
			System_printf ("%s: (%s:%d): Failure in common addMac command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t16Cleanup (tencap, PA_TEST_FAILED);
		}								 
								 
		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l2Handles[i].state = T16_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
 		
 		t16CmdRep (tencap);
	}
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t16CmdRep (tencap);
		
		for (j = m = 0; j < nL2Handles; j++)  {
			if (tencap->l2Handles[j].state == T16_HANDLE_ACTIVE)
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

static paTestStatus_t t16OpenL3 (t16TestEncap_t *tencap, t16IpSetup_t *ipSetup, int nL3Handles)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
	
	for (i = 0; i < nL3Handles; i++)  {
		cmdReply.replyId = T16_CMD_SWINFO0_ADD_IP_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
		
		hd = testCommonAddIp (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t16IpSetup[i].ipInfo, &matchRoute[2], &nfailRoute,
								 &tencap->l3Handles[i].paHandle, 
								 tencap->l2Handles[t16IpSetup[i].lHandleIdx].paHandle,
								 tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
								 &cmdReply, &cmdDest, &cmdSize, &paret);
								 
								 
		if (hd == NULL)  {
					
			System_printf ("%s: (%s:%d): Failure in common addIp command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t16Cleanup (tencap, PA_TEST_FAILED);
		}								 
								 
		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l3Handles[i].state = T16_HANDLE_PENDING_ACK;
 		paTestL4ExpectedStats.classify1.nPackets += 1;
 		tencap->l2Handles[t16IpSetup[i].lHandleIdx].linkCnt++;
 		t16CmdRep (tencap);
	}
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t16CmdRep (tencap);
		
		for (j = m = 0; j < nL3Handles; j++)  {
			if (tencap->l3Handles[j].state == T16_HANDLE_ACTIVE)
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

static paTestStatus_t t16GenericOpenL4 (t16TestEncap_t *tencap, unsigned int fDestPort, int nL3Handles, int nL4Handles)
{
	int 			i, j, m;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
    
	for (i = nL4Handles-1; i >= 0; i--)  {
			
		cmdReply.replyId  = T16_CMD_SWINFO0_ADD_PORT_ID + i;
		cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
		
		hd = testCommonAddPort (tencap->tf, pa_LUT2_PORT_SIZE_16, T16_FIRST_DEST_PORT+i, &matchRoute[0], &tencap->l4Handles[i].paHandle, 
								&tencap->l3Handles[(T16_FIRST_DEST_PORT+i) % T16_NUM_LOCAL_L3_HANDLES].paHandle,
                                tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);
	
								 
								 
		if (hd == NULL)  {
					
			System_printf ("%s: (%s:%d): Failure in common addPort command, entry number %d\n", tfName, __FILE__, __LINE__, i);
			t16Cleanup (tencap, PA_TEST_FAILED);
		}								 
								 
		Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		tencap->l4Handles[i].state = T16_HANDLE_PENDING_ACK;
 		tencap->l3Handles[(T16_FIRST_DEST_PORT+i) % T16_NUM_LOCAL_L3_HANDLES].linkCnt++;
 	    paTestL4ExpectedStats.classify2.nPackets += 1;
        #ifdef NSS_GEN2
 	    paTestL4ExpectedStats.classify1.nPackets += 1;
        #endif
        
 		utilCycleDelay (100000);
 		
 		t16CmdRep (tencap);
	}
	
	
	/* All the packets should have been acked */
	for (i = 0; i < 100; i++)  {
		t16CmdRep (tencap);
		
		for (j = m = 0; j < nL4Handles; j++)  {
			if (tencap->l4Handles[j].state == T16_HANDLE_ACTIVE)
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

	/* Now attempt to add one more entry in LUT2, should expect LUT2 FULL signal indication from the hardware */
paReturn_t t16_expect_lut2_full(t16TestEncap_t *tencap, unsigned int fDestPort)
{
    paReturn_t     paret;
    int            cmdDest;
	uint16_t	   cmdSize;
	Cppi_HostDesc *hd;

    cmdReply.replyId  = T16_CMD_SWINFO0_ADD_PORT_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

	hd = testCommonAddPort (tencap->tf, pa_LUT2_PORT_SIZE_16, fDestPort, &matchRoute[0], &tencap->l4Handles[0].paHandle,
							&tencap->l3Handles[0].paHandle,
                            tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);

	if (hd == NULL)  {

		System_printf ("%s: (%s:%d): Failure in common addPort command, entry port number %d\n", tfName, __FILE__, __LINE__, fDestPort);
		t16Cleanup (tencap, PA_TEST_FAILED);
	}

	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    paTestL4ExpectedStats.classify2.nPackets += 1;

    /* Wait a bit for any packets in PA to complete */
	utilCycleDelay (100000);
	return(t16CmdRep (tencap));
}
void paTestLUT2Full (UArg a0, UArg a1)
{	
 	int				      i;
 	paTestStatus_t  newStatus = PA_TEST_PASSED;
 	paReturn_t            paret;
   
 	memset (&t16Encap, 0, sizeof(t16Encap));
 	t16Encap.tf  = (tFramework_t *)a0;
 	t16Encap.pat = (paTest_t *)a1;
 	for (i = 0; i < T16_NUM_LOCAL_L4_HANDLES; i++)
 		t16Encap.l4Handles[i].state = T16_HANDLE_UNCONFIGURED; 		
 	for (i = 0; i < T16_NUM_LOCAL_L3_HANDLES; i++)
 		t16Encap.l3Handles[i].state = T16_HANDLE_UNCONFIGURED;	
 	for (i = 0; i < T16_NUM_LOCAL_L2_HANDLES; i++)
 		t16Encap.l2Handles[i].state = T16_HANDLE_UNCONFIGURED;
 	for (i = 0; i < T16_NUM_LOCAL_L3_HANDLES; i++)
 		t16Encap.hd[i] = NULL;	
 		
  /* Runtime initial values */
  matchRoute[0].queue = (uint16_t) t16Encap.tf->QGen[Q_MATCH];
  matchRoute[0].flowId= t16Encap.tf->tfFlowNum[0];
  nfailRoute.queue    = (uint16_t) t16Encap.tf->QGen[Q_NFAIL];
  cmdReply.queue      = (uint16_t) t16Encap.tf->QGen[Q_CMD_REPLY];
  cmdReply.flowId     = t16Encap.tf->tfFlowNum[0];
    
  /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
  memset (&paTestL4ExpectedStats, 0, sizeof(paTestL4ExpectedStats));
    
   
  /* Use default Global Configuration */
  /* Burst in the L2 configuraton */
	newStatus = t16OpenL2 (&t16Encap, t16EthSetup, T16_NUM_LOCAL_L2_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t16Cleanup (&t16Encap, newStatus);  /* No return */
	
  	System_flush ();

	/* Burst in the L3 configuration */
	newStatus = t16OpenL3 (&t16Encap, t16IpSetup, T16_NUM_LOCAL_L3_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t16Cleanup (&t16Encap, newStatus);

  	System_flush ();

	/* Completely fill the L4 table. There is a one to one mapping of UDP destination port
	 * values to the IP entries in the lookup table. */
	newStatus = t16GenericOpenL4 (&t16Encap, T16_FIRST_DEST_PORT, T16_NUM_LOCAL_L3_HANDLES, T16_NUM_LOCAL_L4_HANDLES);
	if (newStatus == PA_TEST_FAILED)
		t16Cleanup (&t16Encap, newStatus);
        
    /* Now expect LUT2 FULL here */
	paret = t16_expect_lut2_full(&t16Encap, T16_FIRST_DEST_PORT+300);

	if (paret != pa_LUT2_TABLE_FULL) {
		t16Cleanup(&t16Encap, PA_TEST_FAILED);
	}

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t16Encap.tf, t16Encap.pat, tfName, &paTestL4ExpectedStats, t16Encap.tf->QLinkedBuf1,
	                                   t16Encap.tf->QGen[Q_CMD_RECYCLE], t16Encap.tf->QGen[Q_CMD_REPLY], TRUE);
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t16Cleanup (&t16Encap, newStatus);  /* No return */
    }    
        
    
	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t16Encap.tf, t16Encap.pat, tfName, &paTestL4ExpectedStats, t16Encap.tf->QLinkedBuf1,
	                                   t16Encap.tf->QGen[Q_CMD_RECYCLE], t16Encap.tf->QGen[Q_CMD_REPLY], TRUE);
                           
	if (newStatus == PA_TEST_FAILED)
    {
		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
		t16Cleanup (&t16Encap, PA_TEST_FAILED);  /* no return */
    }    
    
	/* No return from cleanup */
	t16Cleanup (&t16Encap, newStatus);
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
 		
}
 
