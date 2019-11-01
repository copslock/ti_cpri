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
#include "test9pkts.h"

#ifdef __LINUX_USER_SPACE
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif

/* Add/Delete SRIO and PDSP routing test
 * This test tests the LLD Pa_addSrio and Pa_delHandle functions, as well as the
 * PDSP firmware for routing SRIO packets. This test has the following sub-tests
 *   - Test the LLD for the ability to determine if a new entry invalidates a previous entry
 *   - Test the LLD to enter increasingly strict match requirements (destination Id, source Id, mailbox ...)
 *   - Test the LLD when entering more entries then configured for
 *   - Test the firmware for the ability to take a burst of Pa_addSrio commands
 *   - Test the firmware for the routing of matches and next step fail routes
 *   - Test the firmware for the ability to take a burst of data packets
 *   - Test the LLD/firmware for the ability to delete entries
 */

 static char *tfName = "paTestSrioRouting";
 
 /* General purpose queue usage */
 #define Q_CMD_RECYCLE		  0		/* Command descriptors/buffers recycled here after sent to PA */
 #define Q_CMD_REPLY  		  1		/* Replies from PA routed here */
 #define Q_MATCH		  	  2		/* Packets from PA which match a lookup criteria */
 #define Q_NFAIL		      3		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
 #define Q_PARSE_ERR		  4		/* Packets which resulted in a parse error */
 #define Q_MULTI_0			  5		/* Multi route queue 0 */
 #define Q_MULTI_1			  6	    /* Multi route queue 1 */
 #define Q_MULTI_2			  7		/* Multi route queue 2 */
 #define Q_MULTI_3			  8		/* Multi route queue 3 */
 #define Q_MULTI_4			  9		/* Multi route queue 4 */
 #define Q_MULTI_5			 10		/* Multi route queue 5 */
 #define Q_MULTI_6           11		/* Multi route queue 6 */
 #define Q_MULTI_7			 12		/* Multi route queue 7 */
 
/* The number of PA handles maintained by this test */
#define T9_NUM_LOCAL_HANDLES	64

/* The total number of buffers with linked descriptors */
#define TOTAL_BUFS   (TF_LINKED_BUF_Q1_NBUFS + TF_LINKED_BUF_Q2_NBUFS + TF_LINKED_BUF_Q3_NBUFS)
 
 /* Commands to the PA are verified through the value in swinfo0.
  * The 16 ms bits are used as verification, the 16 lbs are for local handle id */
#define T9_CMD_SWINFO0_ADD_ID  		0x11110000  /* Identifies add mac command */
#define T9_CMD_SWINFO0_DEL_ID  		0x22220000  /* Identifies del mac command */
#define T9_CMD_SWINFO0_STATS_REQ_ID	0x33330000	/* Identifies the req stats command */
#define T9_CMD_SWINFO0_PKT_ID		0x55550000  /* Identifies the packet as a data packet */

#define T9_NUM_PACKET_ITERATIONS        1    /* 3 */

 
static paSysStats_t paTestL2ExpectedStats;  /* Expected stats results */
 
 /* 32 L2 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
	L2_HANDLE_UNCONFIGURED = 0,
	L2_HANDLE_PENDING_ACK,
	L2_HANDLE_ACTIVE,
	L2_HANDLE_DISABLED
};

 typedef struct t9Handles_s  {
 
  	paHandleL2L3_t  paHandle;     /* The handle returned by the PA LLD */

 	unsigned int			state;		  /* L2_HANDLE_UNCONFIGURED = handle not configured
 								   * L2_HANDLE_PENDING_ACK = handle configured and sent to pa
 								   * L2_HANDLE_ACTIVE = handle creation acknowledged by pa */
 	
 } t9Handles_t;
 
 
 /* Static test configuration - routing of matching packets to the same queue and 
  * distinguished by swinfo0 */
 typedef struct t9SrioAndRoute_s  {
 	
 	paSrioInfo_t srio;
 	uint32_t     swinfo0;
    uint16_t     nextHdrOffset;
    uint8_t      nextHdr;
    
 } t9SrioAndRoute_t;
 
static t9SrioAndRoute_t  t9SrioAndSwinfo[] =  {
	
	{ 
      /* Entry 0: Route on dest Id only */
      {  
         pa_SRIO_INFO_VALID_DEST_ID,                 /* valid bits */
         0,                                          /* srcId */
         0x1234,                                     /* destId */
         pa_SRIO_TRANSPORT_TYPE_1,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_11,                            /* message type */    
         {  
            0,                                       /* mbox */
            0                                        /* letter */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID,                         /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 1: Route on dest/src Id only */
      {  
         pa_SRIO_INFO_VALID_ID,                      /* valid bits */
         0x5678,                                     /* srcId */
         0x1234,                                     /* destId */
         pa_SRIO_TRANSPORT_TYPE_1,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_11,                            /* message type */    
         {  
            0,                                       /* mbox */
            0                                        /* letter */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 1,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 2: Route on dest/src Id and mbox */
      {  
         pa_SRIO_INFO_VALID_ID |                     /* valid bits */
         pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX,
         0x5678,                                     /* srcId */
         0x1234,                                     /* destId */
         pa_SRIO_TRANSPORT_TYPE_1,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_11,                            /* message type */    
         {  
            2,                                       /* mbox */
            0                                        /* letter */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 2,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 3: Route on dest/src Id, mbox  and letter */
      {  
         pa_SRIO_INFO_VALID_ID                |      /* valid bits */
         pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX |
         pa_SRIO_INFO_VALID_TYPE_INFO_LETTER,
         0x5678,                                     /* srcId */
         0x1234,                                     /* destId */
         pa_SRIO_TRANSPORT_TYPE_1,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_11,                            /* message type */    
         {  
            2,                                       /* mbox */
            1                                        /* letter */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 3,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 4: Route on dest/src Id, mbox, letter and priority */
      {  
         pa_SRIO_INFO_VALID_ID                |      /* valid bits */
         pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX |
         pa_SRIO_INFO_VALID_TYPE_INFO_LETTER  |
         pa_SRIO_INFO_VALID_PRI,
         0x5678,                                     /* srcId */
         0x1234,                                     /* destId */
         pa_SRIO_TRANSPORT_TYPE_1,                   /* transport type */ 
         0,                                          /* completion code */
         2,                                          /* priority */
         pa_SRIO_TYPE_11,                            /* message type */    
         {  
            2,                                       /* mbox */
            1                                        /* letter */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 4,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
    
	{ 
      /* Entry 5: Route on dest Id only */
      {  
         pa_SRIO_INFO_VALID_DEST_ID,                 /* valid bits */
         0,                                          /* srcId */
         0x12,                                       /* destId */
         pa_SRIO_TRANSPORT_TYPE_0,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_9,                             /* message type */    
         {  
            0,                                       /* stream Id */
            0                                        /* class of service */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 5,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 6: Route on dest/src Id only */
      {  
         pa_SRIO_INFO_VALID_ID,                      /* valid bits */
         0x56,                                       /* srcId */
         0x12,                                       /* destId */
         pa_SRIO_TRANSPORT_TYPE_0,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_9,                             /* message type */    
         {  
            0,                                       /* stream Id */
            0                                        /* class of service */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 6,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 7: Route on dest/src Id and stream Id */
      {  
         pa_SRIO_INFO_VALID_ID |                     /* valid bits */
         pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID,
         0x56,                                       /* srcId */
         0x12,                                       /* destId */
         pa_SRIO_TRANSPORT_TYPE_0,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_9,                             /* message type */    
         {  
            0xbabe,                                  /* stream Id */
            0                                        /* class of service */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 7,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 8: Route on dest/src Id, stream Id and class of service */
      {  
         pa_SRIO_INFO_VALID_ID                 |     /* valid bits */
         pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID |
         pa_SRIO_INFO_VALID_TYPE_INFO_COS,
         0x56,                                       /* srcId */
         0x12,                                       /* destId */
         pa_SRIO_TRANSPORT_TYPE_0,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_9,                             /* message type */    
         {  
            0xbabe,                                  /* stream Id */
            2                                        /* class of service */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 8,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 9: Route on dest/src Id, stream Id, class of service and priority */
      {  
         pa_SRIO_INFO_VALID_ID                 |     /* valid bits */
         pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID |
         pa_SRIO_INFO_VALID_TYPE_INFO_COS      |
         pa_SRIO_INFO_VALID_PRI,
         0x56,                                       /* srcId */
         0x12,                                       /* destId */
         pa_SRIO_TRANSPORT_TYPE_0,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_9,                             /* message type */    
         {  
            0xbabe,                                  /* stream Id */
            2                                        /* class of service */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 9,                     /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
	{ 
      /* Entry 10: Route on letter only */
      {  
         pa_SRIO_INFO_VALID_TYPE_INFO_LETTER,        /* valid bits */
         0,                                          /* srcId */
         0,                                          /* destId */
         pa_SRIO_TRANSPORT_TYPE_1,                   /* transport type */ 
         0,                                          /* completion code */
         2,                                          /* priority */
         pa_SRIO_TYPE_11,                            /* message type */    
         {  
            0,                                       /* mbox */
            3                                        /* letter */
         }  	
      },   
      T9_CMD_SWINFO0_PKT_ID + 10,                    /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    },
    
 };
 
/*
 *  Use both pre-determined and system-allocated LUT1 entry index 
 *  Note: The pre-determined LUT1 entry index should be consistent with the system-allocated one.
 *        It is not recommended to mix those two modes in the application.
 */ 
#ifndef __LINUX_USER_SPACE 
const int  t9SrioIndex[] =  {
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 0 */
                62,                             /* entry 1 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 2 */
                60,                             /* entry 3 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 4 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 5 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 6 */
                56,                             /* entry 7 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 8 */
                54,                             /* entry 9 */
                40                              /* entry 10 */
};                
#else
const int  t9SrioIndex[] =  {
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 0 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 1 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 2 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 3 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 4 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 5 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 6 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 7 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 8 */
                pa_LUT1_INDEX_NOT_SPECIFIED,    /* entry 9 */
                pa_LUT1_INDEX_NOT_SPECIFIED     /* entry 10 */
};                
#endif
 
static t9SrioAndRoute_t  t9SrioAndSwinfoFail = 
    {
      /* This would steal matches from entry 4 if allowed */
      /* This entry is not passed to PA */
      /* !!! No PA Entry !!!! */
      {  
         pa_SRIO_INFO_VALID_SRC_ID            |      /* valid bits */
         pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX |
         pa_SRIO_INFO_VALID_TYPE_INFO_LETTER  |
         pa_SRIO_INFO_VALID_PRI,
         0x5678,                                     /* srcId */
         0x1234,                                     /* destId */
         pa_SRIO_TRANSPORT_TYPE_1,                   /* transport type */ 
         0,                                          /* completion code */
         2,                                          /* priority */
         pa_SRIO_TYPE_11,                            /* message type */    
         {  
            2,                                       /* mbox */
            1                                        /* letter */
         }  	
      },   
      0xDEADBEEF,                                    /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    };

 
/* A single non-const entry is used to test handling entering too many table elements
 * as well as delete and add */
static t9SrioAndRoute_t  t9VarSrioAndRoute =  
	{ 
      /* Entry 0: Route on dest Id only */
      {  
         pa_SRIO_INFO_VALID_DEST_ID,                 /* valid bits */
         0,                                          /* srcId */
         0x2000,                                     /* destId */
         pa_SRIO_TRANSPORT_TYPE_1,                   /* transport type */ 
         0,                                          /* completion code */
         0,                                          /* priority */
         pa_SRIO_TYPE_11,                            /* message type */    
         {  
            0,                                       /* mbox */
            0                                        /* letter */
         }  	
      },   
      0,                                             /* swInfo0 */
      0,                                             /* next hedaer offset */
      pa_HDR_TYPE_UNKNOWN                            /* next header type */
    };

									        
/* Prototype required due to circular function calling */									        
static paTestStatus_t t9CheckStats (tFramework_t *tf, paTest_t *pat, Bool clear, t9Handles_t *l2Handles);			


static Cppi_HostDesc *formDataPacket (tFramework_t *tf, paTest_t *pat, int pktIdx, uint8_t *expectedPktCount)
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
  	
  	/* Attach the data and set the length */
    Cppi_setPacketType(Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t) t9PktTestInfo[pktIdx].pktType);
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)t9PktTestInfo[pktIdx].psInfo, 8);
  	Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(t9PktTestInfo[pktIdx].pkt)), t9PktTestInfo[pktIdx].pktLen);
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t9PktTestInfo[pktIdx].pktLen);
  	
  	return (hd);
}

/* setup the Test Pkt for pktdma */
int t9SetupPktTestInfo(pktTest9Info_t* testInfoPkt, int count, char* tfname)
{
	int returnVal = 0;
#ifdef __LINUX_USER_SPACE
	int i,pktSz;
	uint8_t *pkt;
	for (i = 0; i < count; i++)  {
		/* Store the original information */
        pkt   = testInfoPkt[i].pkt;
		pktSz = testInfoPkt[i].pktLen;
        /* Allocate memory for the Packet buffers from Framework, to control phy2Virt and Virt2Phy */
        testInfoPkt[i].pkt = (uint8_t *)fw_memAlloc(pktSz, CACHE_LINESZ);
        if(testInfoPkt[i].pkt == NULL) {
  	        printf ("%s: memAlloc failed for pkt %d\n", tfname, i);
  	        returnVal = -1;
        }
		/* Restore the original pkt information in the allocated memory */
        memcpy(testInfoPkt[i].pkt, pkt, pktSz);
    }
#endif

	return (returnVal);
}

								        
/* Recycle delete commands and command recycles  */
static int t9StateDel (tFramework_t *tf, paTest_t *pat, t9Handles_t *l2Handles)
{
	Cppi_HostDesc  *hd;
	paEntryHandle_t reth;
	paReturn_t      paret;
	int				htype;
	int  			cmdDest;
	int 			i;
		
	/* Don't send the command until half of the rx buffers are available. The command replies will always be
	 * sourced from QLinkedBuf1 and arrive in Q_CMD_REPLY. Delay the command until there are enough buffers available
	 * to send and leave a 50% overhead.*/
	for (i = 0; (i < 100) && (Qmss_getQueueEntryCount (tf->QLinkedBuf1) < (TF_LINKED_BUF_Q1_NBUFS >> 1)); i++)  {		
		if ((Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) + Qmss_getQueueEntryCount(tf->QLinkedBuf1)) >= TF_LINKED_BUF_Q1_NBUFS)
		  break;
	}
			
	while (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) > 0)  {
				
		/* Recycle the command descriptor/buffer */
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_REPLY])) & ~15);
		if ((hd->softwareInfo0 & 0xffff0000u) != T9_CMD_SWINFO0_DEL_ID)  {
			System_printf ("%s (%s:%d): Found packet in PA command reply queue without delete ID (found 0x%08x)", tfName, __FILE__, __LINE__, hd->softwareInfo0);
			return (-1);
		}
				
		/* Send the reply back to PA to let the driver know the command has been accepted */
		paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
				
		if (paret != pa_OK)  {
			System_printf ("%s (%s:%d): paForwardResult returned error %d in response to pa_DelHandle reply from PA for handle #%d\n", tfName, __FILE__, __LINE__, hd->softwareInfo0 & 0xffff);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			return (-1);
		}
			
		l2Handles[hd->softwareInfo0 & 0xffff].state = L2_HANDLE_DISABLED;
				
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			return (-1);
		}
			
	}
		
    utilCycleDelay (100);
				
	/* The command recycle descriptor/buffer */
	while (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_RECYCLE]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			return (-1);
		}
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Timeout waiting for free descriptor/buffer queue to fill to halfway point\n", tfName, __FILE__, __LINE__);
		return (-1);
	}
	
	return (0);
		
}
														        									        									
	
static void paTestL2RecoverAndExit (tFramework_t *tf, paTest_t *pat, t9Handles_t *l2Handles, paTestStatus_t status, Bool doStats)
{
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
 	uint16_t			cmdSize;
	int 			i, j, m;
	volatile int    mdebugWait = 0;
	
	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
	cmdReply.queue   = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId  = tf->tfFlowNum[0];

	
	/* Delete all the handles */
	for (i = 0; i < T9_NUM_LOCAL_HANDLES; i++)  {
		/* do not attempt to delete if handle is null */
		if (l2Handles[i].state == L2_HANDLE_UNCONFIGURED)
			continue;

		cmdReply.replyId = T9_CMD_SWINFO0_DEL_ID + i;
		hd = testCommonDelHandle (tf, &l2Handles[i].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, &cmdReply, &cmdDest, &cmdSize, &paret);
		
		if (paret != pa_OK)  {
			System_printf ("%s (%s:%d): PA LLD returned error code %d on handle deletion(%d)\n", tfName, __FILE__, __LINE__, paret, i);
			status = PA_TEST_FAILED;
			break;
		}
		
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): No descriptor available for del handle command\n", tfName, __FILE__, __LINE__);
			status = PA_TEST_FAILED;
			break;
		}
		
		/* Wait to send the command until half of the rx buffers are available */
		if (t9StateDel (tf, pat, l2Handles))  {
			status = PA_TEST_FAILED;
			break;
		}
		
		/* mdebugHaltPdsp (0); */
		/* Send the command to the PA */
		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
		paTestL2ExpectedStats.classify1.nPackets += 1;
		
		while (mdebugWait);
	}
	
	/* Give some time for remaining commands to complete */
	for (i = 0; i < 100; i++)  {
		
		for (j = m = 0; j < T9_NUM_LOCAL_HANDLES; j++) {
            /* don't get the state accumulation for unconfigured L2 Handles*/
			if (l2Handles[j].state == L2_HANDLE_UNCONFIGURED)
				continue;
			
			if (l2Handles[j].state != L2_HANDLE_DISABLED)
				m += 1;
	    }
				
		if (m)  {
			if (t9StateDel (tf, pat, l2Handles))  {
				status = PA_TEST_FAILED;
				break;
			}
			utilCycleDelay (100);
		} else
			break;
		
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Unable to delete all L2 handles. %d remain undeleted\n", tfName, __FILE__, __LINE__, m);
		status = PA_TEST_FAILED;
	}
	
	/* Verify the stats are as expected */
	if (t9CheckStats (tf, pat, TRUE, l2Handles) == PA_TEST_FAILED)
	  status = PA_TEST_FAILED;
	
	/* Test result */
	pat->testStatus = status;
	
	/* Return */
	Task_exit();
}		
				
/* Look for command replies from PA */					     
static void t9L2CmdRep (tFramework_t *tf, paTest_t *pat, t9Handles_t *localHandles)
{
	Cppi_HostDesc  *hd;
	uint32_t         *swInfo0;
	uint32_t 			swInfoCmd;
	unsigned int			lid;
	paReturn_t      paret;
	paEntryHandle_t reth;
	int				htype;
	int				cmdDest;
	
	while (Qmss_getQueueEntryCount ((tf->QGen)[Q_CMD_REPLY]) > 0)  {
		
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop ((tf->QGen[Q_CMD_REPLY]))) & ~15);
		if (Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **) &swInfo0) == CPPI_EPIB_NOT_PRESENT)  {
			System_printf ("%s (%s:%d): Found descriptor in PA command reply queue without EIPB present, failing\n", tfName, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		swInfoCmd = (*swInfo0 & 0xffff0000u); 
		/* Verify expected value in swinfo0 16 msbs */
		if ( (swInfoCmd != T9_CMD_SWINFO0_ADD_ID) && (swInfoCmd != T9_CMD_SWINFO0_DEL_ID) ) {
			System_printf ("%s (%s:%d): Found descriptor in PA command reply queue without command reply swinfo0\n", tfName, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		/* Extract the local instance value */
		lid = *swInfo0 & 0xffffu;
		if (lid >= T9_NUM_LOCAL_HANDLES)  {
			System_printf ("%s (%s:%d): Received PA command reply for out of range local handle %d (max %d)\n", tfName, __FILE__, __LINE__, lid, T9_NUM_LOCAL_HANDLES);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		/* Send the reply back to PA to let the driver know the command has been accepted */
		paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
		if (paret != pa_OK)  {
			System_printf ("%s (%s:%d): paForwardResult returned error %d in response to paAddMac reply from PA\n", tfName, __FILE__, __LINE__,paret);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		/* Make sure the handle returned by PA matches the local copy */
		if (localHandles[lid].paHandle != reth.l2l3Handle)  {
			System_printf ("%s (%s:%d): paForwardResult returned handle (0x%08x) that did match internal table value (0x%08x)\n", tfName, __FILE__, __LINE__, (uint32_t)(localHandles[lid].paHandle), (uint32_t) reth.l2l3Handle);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		/* Recycle the descriptor and buffer */
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
		if (swInfoCmd == T9_CMD_SWINFO0_ADD_ID)
			localHandles[lid].state = L2_HANDLE_ACTIVE;
		else
			localHandles[lid].state = L2_HANDLE_UNCONFIGURED;
	}
	
	while (Qmss_getQueueEntryCount ((tf->QGen)[Q_CMD_RECYCLE]) > 0)  {
		
		/* Recycle the command descriptor/buffer */
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop ((tf->QGen[Q_CMD_RECYCLE]))) & ~15);
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);
			paTestL2RecoverAndExit (tf, pat, localHandles, PA_TEST_FAILED, TRUE);  /* No Return */
		}
		
	}
			
		
}
								     
									     
								     
									     
static void paL2HandleError (tFramework_t *tf, paTest_t *pat, t9Handles_t *l2Handles, paReturn_t paret, Cppi_HostDesc *hd)
{	 	  
    /* Check paret before the descriptor. If paret indicates failure the descriptor will be NULL */                    
 	if (paret != pa_OK)  {
 		System_printf ("%s (%s:%d): testCommonAddSrio failed, PA LLD error code = %d\n", tfName, __FILE__, __LINE__, paret);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
 	
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): testCommonAddSrio failed due to unavailable free linked buffer descriptor\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No return */
 	}
}


/* Check the stats */
static paTestStatus_t t9CheckStats (tFramework_t *tf, paTest_t *pat, Bool clear, t9Handles_t *l2Handles)
{ 	
 	paSysStats_t   paStats1;
    paSysStats_t  *paStats = &paStats1; 
	paTestStatus_t  status;
    #ifndef NSS_GEN2
	Cppi_HostDesc  *hd;
	uint8_t		   *bp;
 	unsigned int	blen;
	int             i;

	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
 	cmdReply.queue   = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId  = tf->tfFlowNum[0];
 	
 	/* Check the PA stats to make sure they are set as expected */
 	cmdReply.replyId = T9_CMD_SWINFO0_STATS_REQ_ID;
 	if (testCommonRequestPaStats (tfName, tf, clear, tf->QLinkedBuf1, tf->QGen[Q_CMD_RECYCLE],  &cmdReply))  {
 		System_printf ("%s (%s:%d): testCommonRequestPaStats failed\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
 	}
 	
 	/* Wait for the stats reply */
	for (i = 0; i < 100; i++)  {
		utilCycleDelay (1000);
		if (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) > 0)
			break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Did not find response from PA to stats request command\n", tfName, __FILE__, __LINE__);
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
	
	/* Recycle the descriptor/buffer returned from the stats request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from stats request\n", tfName, __FILE__, __LINE__);
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
	
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats request\n", tfName, __FILE__, __LINE__);
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
 		
 	/* Format the stats response and compare to the expected results */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_REPLY])) & ~15);
	Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bp, &blen);
	paStats = (paSysStats_t *)Pa_formatStatsReply (tf->passHandle, (paCmd_t)bp);
    
#else

	if (testCommonQueryPaStats (tfName, tf, TRUE, paStats))  {
		System_printf ("%s (%s:%d): testCommonQueryPaStats f\ailed\n", tfName, __FILE__, __LINE__);
		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, FALSE);  /* No Return */
	}
#endif
      
    if (testCommonCompareStats (tfName, (paSysStats_t *)&paTestL2ExpectedStats, paStats))
    	status = PA_TEST_FAILED;
    else
    	status = PA_TEST_PASSED;
           
    #ifndef NSS_GEN2	  
     /* Recycle the descriptor and associated buffer back to queue from which it came */
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tfName, __FILE__, __LINE__);
		status = PA_TEST_FAILED;
	}
	#endif
	return (status);
}
	
	
/* Search the receive data packet queue for received data packets. Remain in 
 * this function until all buffers are restored to their respective queues */
static int t9ReceiveDataPkts (tFramework_t *tf, paTest_t *pat, uint8_t *actualPktCount, int expCnt)
{
	Cppi_HostDesc    *hd;
	uint32_t		     *swInfo;
	pktTest9Info_t    *tinfo;
	pasahoLongInfo_t *pinfo;
	uint32_t		      infoLen;
	int               i, j;
	unsigned int		      chan;
    int               cnt = 0;
	
	for (i = 0; i < 100; i++)  {
		
        utilCycleDelay (10000);
		while (Qmss_getQueueEntryCount (tf->QGen[Q_MATCH]) > 0) {
			
			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_MATCH])) & ~15);
			if (hd == NULL)  {
				System_printf ("%s (%s:%d): Popped a NULL descriptor off of match queue\n", tfName, __FILE__, __LINE__);
				return (-1);
			}
			
			/* Verify swInfo0 for packet match and packet ID number */
			Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
			
			if ((*swInfo & 0xffff0000) != T9_CMD_SWINFO0_PKT_ID)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
							   tfName, __FILE__, __LINE__, *swInfo);
			 	testCommonRecycleLBDesc (tf, hd);
				break;
			}
			
			chan = *swInfo & 0xffff;
			if (chan < T9_NUM_LOCAL_HANDLES)
            {
			  actualPktCount[chan] += 1;
              cnt++;
            }  
			  
			/* locate the associated test information based on the channel value */
			for (j = 0, tinfo = NULL; j < sizeof(t9PktTestInfo) / sizeof(pktTest9Info_t); j++)  {
				if (t9PktTestInfo[j].idx == chan)  {
					tinfo = &t9PktTestInfo[j];
					break;
				}
			}
			
			if (tinfo == NULL)  {
				System_printf ("%s (%s:%d): Found a packet in the receive packet queue for channel %d, but found no matching packet info\n",
								tfName, __FILE__, __LINE__, chan);
				testCommonRecycleLBDesc (tf, hd);
				break;
			}
				
			/* Verify the parse information is correct */
			if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
				System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
				testCommonRecycleLBDesc (tf, hd);
				break;
			}
			
			if (testCommonComparePktInfo (tfName, tinfo->info, pinfo))  {
				testCommonRecycleLBDesc (tf, hd);
				break; // skip to the end
			}
			
			/* Return the descriptor/buffer */
			testCommonRecycleLBDesc (tf, hd);								
			
		}
		
        if(cnt >= expCnt)
            break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Error - unable to recover all descriptors with associated buffers\n", tfName, __FILE__, __LINE__);
        System_flush();
		return (-1);
	}
	
	return (0);

}
	
#ifdef __LINUX_USER_SPACE
void* paTestSrioRouting (void *args)
{
 	tFramework_t  *tf  = ((paTestArgs_t *)args)->tf;
 	paTest_t      *pat = ((paTestArgs_t *)args)->pat;
#else
void paTestSrioRouting (UArg a0, UArg a1)
{
 	tFramework_t   *tf  = (tFramework_t *)a0;
 	paTest_t       *pat = (paTest_t *)a1;
#endif
 	Cppi_HostDesc  *hd[8];
 	paReturn_t      paret;
 	t9Handles_t     l2Handles[T9_NUM_LOCAL_HANDLES];
    #ifdef PASS_LUT_LIMIT_TEST 
 	paHandleL2L3_t  fHandle;
    #endif
 	int				i, j, k, l;
 	int				state;
 	int				count;
 	int  			cmdDest;
 	Bool			halt;
 	uint16_t			cmdSize;
 	paTestStatus_t  testStatus = PA_TEST_PASSED;
 	uint8_t			expectedPktCount[T9_NUM_LOCAL_HANDLES];
 	uint8_t			actualPktCount[T9_NUM_LOCAL_HANDLES];

 	
 	volatile int mdebugWait = 1;
 	
 	paRouteInfo_t   matchRoute = {  pa_DEST_HOST,		/* Dest */
 								    0,					/* Flow ID */
 								    0,					/* queue */
 								    -1,					/* Multi route */
 								    0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */       
                                    0,                  /* customType : not used */         
                                    0,                  /* customIndex: not used */     
                                    0,                  /* pkyType: for SRIO only */    
                                    NULL};              /* No commands */
 								    
 	paRouteInfo_t   nfailRoute = {  pa_DEST_HOST,		/* Dest */
 									0,					/* Flow ID */
 									0,					/* queue */
 									-1,					/* Multi route */
 									0,					/* sw Info 0 */
                                    0,                  /* sw Info 1 */       
                                    0,                  /* customType : not used */         
                                    0,                  /* customIndex: not used */     
                                    0,                  /* pkyType: for SRIO only */    
                                    NULL};              /* No commands */
 									
 	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 		
    i = t9SetupPktTestInfo(t9PktTestInfo, (sizeof(t9PktTestInfo) / sizeof(pktTest9Info_t)), tfName);
    if (i < 0) {
    	System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }

    /* Runtime initial values */
    matchRoute.queue = (uint16_t) tf->QGen[Q_MATCH];
    matchRoute.flowId = tf->tfFlowNum[0];
    nfailRoute.queue = (uint16_t) tf->QGen[Q_NFAIL];
    nfailRoute.flowId = tf->tfFlowNum[0];
    cmdReply.queue   = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId  = tf->tfFlowNum[0];
    
    /* Zero out the l2Handle array and packet counts */
    memset (l2Handles, 0, sizeof(l2Handles));
    memset (expectedPktCount, 0, sizeof(expectedPktCount));
    memset (actualPktCount, 0, sizeof(actualPktCount));
    
    /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
    memset (&paTestL2ExpectedStats, 0, sizeof(paTestL2ExpectedStats));
 								  
 	/* Initialize the first entry in the table */
 	cmdReply.replyId = T9_CMD_SWINFO0_ADD_ID + 0;  /* T9_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
 	cmdReply.queue = tf->QGen[Q_CMD_REPLY];
 	matchRoute.swInfo0 = nfailRoute.swInfo0 = t9SrioAndSwinfo[0].swinfo0;
    
 	hd[0] = testCommonAddSrio (tf, t9SrioIndex[0], &t9SrioAndSwinfo[0].srio, t9SrioAndSwinfo[0].nextHdr, t9SrioAndSwinfo[0].nextHdrOffset, 
                              &matchRoute, &nfailRoute, &l2Handles[0].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                          &cmdReply, &cmdDest, &cmdSize, &paret);
 	                        
 	                        
 	paL2HandleError (tf, pat, l2Handles, paret, hd[0]);  /* Will not return on error */
 	
 	/* Send the command to PA. This will result in 1 packet in classify1 */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[0], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	l2Handles[0].state = L2_HANDLE_PENDING_ACK;
 	paTestL2ExpectedStats.classify1.nPackets += 1;
 	 	
 	/* Wait for a PA reply */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t9L2CmdRep (tf, pat, l2Handles);
 		if (l2Handles[0].state == L2_HANDLE_ACTIVE)
 			break;
 	}
 	
 	if (i == 100)  {
 		System_printf ("%s (%s:%d): Reply to paAddSrio not found\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* no return */
 	}
 	
 	
 	/* Add the next 6 entries in a burst. Take the memory from linked buffer descriptor area two
 	 * since the responses will come from area one. These are entries 1-6 in the local handle table */
 	for (i = 1; i < 7; i++)  {
 		cmdReply.replyId = T9_CMD_SWINFO0_ADD_ID + i;
 		matchRoute.swInfo0 = nfailRoute.swInfo0 = t9SrioAndSwinfo[i].swinfo0;
 		hd[i] = testCommonAddSrio (tf, t9SrioIndex[i+1], &t9SrioAndSwinfo[i].srio, t9SrioAndSwinfo[i].nextHdr, t9SrioAndSwinfo[i].nextHdrOffset,
                                   &matchRoute, &nfailRoute, &l2Handles[i].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                               &cmdReply, &cmdDest, &cmdSize, &paret);
 	    paL2HandleError (tf, pat, l2Handles, paret, hd[i]);  /* Will not return on error */
 	}
 	
 	
 	/* Send all the commands at once to test the ability to handle back to back commands */
 	for (i = 1; i < 7; i++)  {
 		/* if (mdebugWait) mdebugHaltPdsp(0); */ 
 		
 		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		
 		/* while (mdebugWait); */
 		
 		l2Handles[i].state = L2_HANDLE_PENDING_ACK;
 		paTestL2ExpectedStats.classify1.nPackets += 1;
 		
 	}
 	
 	
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t9L2CmdRep (tf, pat, l2Handles);
 		
 		state = 1;
 		for (j = 1; j < 7; j++)  {
 			if (l2Handles[j].state == L2_HANDLE_PENDING_ACK)
 				state = 0;
 		}
 		
 		if (state == 1)
 			break;
 	}
 	
 	if ((i == 100) && (state == 0))  {
 		System_printf ("%s: (%s:%d): Burst of 6 addSrio commands did not result in 6 acks from PA\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);
 	}

 	/* The next entry should result in the PA generating an error */
 	cmdReply.replyId = T9_CMD_SWINFO0_ADD_ID + 7;  /* T9_CMD_SWINFO0_ADD_ID identfies command, 16 LS bits identify the local handle number */
 	matchRoute.swInfo0 = nfailRoute.swInfo0 = t9SrioAndSwinfoFail.swinfo0;
 	hd[0] = testCommonAddSrio (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t9SrioAndSwinfoFail.srio, t9SrioAndSwinfoFail.nextHdr, t9SrioAndSwinfoFail.nextHdrOffset, 
                               &matchRoute, &nfailRoute, &l2Handles[7].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
 	                           &cmdReply, &cmdDest, &cmdSize, &paret);
 	                        
 	                         	/* If the descriptor came back non-null, restore it and return it */
 	if (hd[0] != NULL)  {
 		hd[0]->buffLen = hd[0]->origBufferLen;
 		Qmss_queuePush (tf->QLinkedBuf2, (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}
 	                  
 	/* The test won't exit on this failure since no commands have been sent to PA */                  
 	if (paret != pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT)  {
 		System_printf ("%s (%s:%d): function paAddSrio did not detect an invalid entry order\n", tfName, __FILE__, __LINE__);
 		testStatus = PA_TEST_FAILED;
 	}
 		
 		
    /* Rapid fire the next 4 entries from the const table to the PA */
    /* Add the next 4 entries in a burst. Take the memory from linked buffer descriptor area two
 	 * since the responses will come from area one. These are entries 1-6 in the local handle table */
 	for (i = 7; i < 11; i++)  {
 		cmdReply.replyId = T9_CMD_SWINFO0_ADD_ID + i;
 		matchRoute.swInfo0 = nfailRoute.swInfo0 = t9SrioAndSwinfo[i].swinfo0;
 		hd[i-7] = testCommonAddSrio (tf, t9SrioIndex[i], &t9SrioAndSwinfo[i].srio, t9SrioAndSwinfo[i].nextHdr, t9SrioAndSwinfo[i].nextHdrOffset,
                                   &matchRoute, &nfailRoute, &l2Handles[i].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
 	                        &cmdReply, &cmdDest, &cmdSize, &paret);
 	    paL2HandleError (tf, pat, l2Handles, paret, hd[i-7]);  /* Will not return on error */
 	}
 	
 	/* Send all the commands at once to test the ability to handle back to back commands */
 	for (i = 7; i < 11; i++)  {
 		Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[i-7], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 		l2Handles[i].state = L2_HANDLE_PENDING_ACK;
 		paTestL2ExpectedStats.classify1.nPackets += 1;
 	}
 	
 	/* Wait for the PA to generate all the responses */
 	for (i = 0; i < 100; i++)  {
 		utilCycleDelay (1000);
 		t9L2CmdRep (tf, pat, l2Handles);
 		
 		state = 1;
 		for (j = 7; j < 11; j++)  {
 			if (l2Handles[j].state == L2_HANDLE_PENDING_ACK)
 				state = 0;
 		}
 		
 		if (state == 1)
 			break;
 	}
 	
 	 if (i == 100)  {
 		System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
 	}
 	
#ifndef __LINUX_USER_SPACE 	
 	/* 11 Entries into the table have been made. Make an additional 52 entries in 13 batches of 4.
 	 * These entries do not test any particular routing, simply table addition and overflow */
 	for (i = k = 0; i < 13; i++, k += 4)  {
 		
 		for (j = 0; j < 4; j++)  {
 			cmdReply.replyId = T9_CMD_SWINFO0_ADD_ID + 11 + k + j;
 			t9VarSrioAndRoute.srio.destId = 0x2000 + k + j;
 			matchRoute.swInfo0 = nfailRoute.swInfo0 = 0x5555000b + k + j + 11;
 			hd[j] = testCommonAddSrio (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t9VarSrioAndRoute.srio, t9VarSrioAndRoute.nextHdr, t9VarSrioAndRoute.nextHdrOffset,
                                      &matchRoute, &nfailRoute, &l2Handles[k+j+11].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
 	                        		  &cmdReply, &cmdDest, &cmdSize, &paret);
 	    	paL2HandleError (tf, pat, l2Handles, paret, hd[j]);  /* Will not return on error */
 		}
 		
 		/* Send all the commands at once to test the ability to handle back to back commands */
 		for (j = 0; j < 4; j++)  {
 			Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[j], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 			l2Handles[k+j+11].state = L2_HANDLE_PENDING_ACK;
 			paTestL2ExpectedStats.classify1.nPackets += 1;
 		}
 
 		/* Wait for the PA to generate all the responses */
 		for (l = 0; l < 100; l++)  {
 			utilCycleDelay (1000);
 			t9L2CmdRep (tf, pat, l2Handles);
 		
 			state = 1;
 			for (j = 0; j < 4; j++)  {
 				if (l2Handles[11 + k + j].state == L2_HANDLE_PENDING_ACK)
 					state = 0;
 			}
 		
 			if (state == 1)
 				break;
 		}
 		
 		if (l == 100)  {
 			System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
 			paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
 		}
 	}
 	
 	/* There is one place left in the table. Try to make two more entries. The first one should succeed
 	 * but the next one should be rejected by the PA lld */
 	cmdReply.replyId = T9_CMD_SWINFO0_ADD_ID + 11 + k;
 	t9VarSrioAndRoute.srio.destId = 0x2000 + k + 11;
 	matchRoute.swInfo0 = nfailRoute.swInfo0 = T9_CMD_SWINFO0_ADD_ID + k + 11;
 	hd[0] = testCommonAddSrio (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t9VarSrioAndRoute.srio, t9VarSrioAndRoute.nextHdr, t9VarSrioAndRoute.nextHdrOffset,
                              &matchRoute, &nfailRoute, &l2Handles[k+11].paHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                          &cmdReply, &cmdDest, &cmdSize, &paret);
 	paL2HandleError (tf, pat, l2Handles, paret, hd[0]);  /* Will not return on error */
 	
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd[0], cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
 	l2Handles[k+11].state = L2_HANDLE_PENDING_ACK;
 	paTestL2ExpectedStats.classify1.nPackets += 1;
 
 	
 	/* Wait for the PA to generate the response */
 	for (l = 0; l < 100; l++)  {
 		utilCycleDelay (1000);
 		t9L2CmdRep (tf, pat, l2Handles);

 		if (l2Handles[11+k].state == L2_HANDLE_ACTIVE)
 			break;
 	}
 	
 	if (l == 100)  {
 		System_printf ("%s (%s:%d): Failed to find responses from PA\n", tfName, __FILE__, __LINE__);
 		paTestL2RecoverAndExit (tf, pat, l2Handles, PA_TEST_FAILED, TRUE);  /* No Return */
	}
    
    #ifdef PASS_LUT_LIMIT_TEST 
 
 	/* The next add mac command should fail */
 	k += 1;
 	t9VarSrioAndRoute.srio.destId = 0x2000 + k + 11;
 	matchRoute.swInfo0 = nfailRoute.swInfo0 = T9_CMD_SWINFO0_ADD_ID + k + 11;
 	hd[0] = testCommonAddSrio (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &t9VarSrioAndRoute.srio, t9VarSrioAndRoute.nextHdr, t9VarSrioAndRoute.nextHdrOffset,
                              &matchRoute, &nfailRoute, &fHandle, tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, 
 	                          &cmdReply, &cmdDest, &cmdSize, &paret);
 		                          
 	/* If the descriptor came back non-null, restore it and return it */
 	if (hd[0] != NULL)  {
 		hd[0]->buffLen = hd[0]->origBufferLen;
 		Qmss_queuePush (tf->QLinkedBuf2, (Ptr)hd[0], hd[0]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
 	}
 	
 	/* The test will continue on failure since the PA sub-system has not been changed */
 	if (paret != pa_HANDLE_TABLE_FULL)  {
 		System_printf ("%s (%s:%d): function paAddSrio did not detect a full handle table\n", tfName, __FILE__, __LINE__);
 		testStatus = PA_TEST_FAILED;
 	}
    
    #endif
 	
 	/* Check and clear the stats */
 	testStatus = t9CheckStats (tf, pat, TRUE, l2Handles);
	memset (&paTestL2ExpectedStats, 0, sizeof(paTestL2ExpectedStats));
	
	if (testStatus != PA_TEST_PASSED)
		paTestL2RecoverAndExit (tf, pat, l2Handles, testStatus, TRUE);
#endif	

	/* Run packets through the system. the complete set of packets is run through three times. */
	for (j = 0, halt = FALSE; (j < T9_NUM_PACKET_ITERATIONS) && (halt == FALSE); j++)   {
		
        int numPkts = 0;
        
		for (i = 0; (i < sizeof(t9PktTestInfo) / sizeof(pktTest9Info_t)) && (halt == FALSE);  )  {
			
			/* Form up to 8 data packets to send */
			for (k = 0; ((k < 8) && (i < sizeof(t9PktTestInfo) / sizeof(pktTest9Info_t))); k++)  {
				hd[k] = formDataPacket (tf, pat, i, expectedPktCount);
				
				if (hd[k] == NULL)  {
					halt = TRUE;
					break;
				}
				
				/* Inc the count if the packet is passed back to the host */
  				if (t9PktTestInfo[i].idx >= 0)
                {
	  				expectedPktCount[t9PktTestInfo[i].idx] += 1;
                    numPkts++;
                }    
		
				/* Increment any expected stats */
				testCommonIncStats (t9PktTestInfo[i].statsMap, &paTestL2ExpectedStats);	
				
				/* Proceed to the next packet */
				i += 1;  				

			}
			
//            mdebugWait = 1;
// 		    if (mdebugWait) mdebugHaltPdsp(0);  
				
			for (l = 0; l < k; l++)
				Qmss_queuePush (tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[l], hd[l]->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
                
//	        while (mdebugWait);
				
		}
		
		/* Wait for all descriptors associated with received packets to be recycled. The descriptors
		 * had to go somewhere so wait until the total count is accounted for */
		for (l = 0, count = 0; ((l < 100) && (count != TOTAL_BUFS)); l++)
        {
            utilCycleDelay (100);
			count = Qmss_getQueueEntryCount(tf->QGen[Q_MATCH]) + Qmss_getQueueEntryCount(tf->QGen[Q_NFAIL]) + 
			        Qmss_getQueueEntryCount(tf->QGen[Q_PARSE_ERR]) + Qmss_getQueueEntryCount(tf->QLinkedBuf1) + 
			        Qmss_getQueueEntryCount(tf->QLinkedBuf2) + Qmss_getQueueEntryCount(tf->QLinkedBuf3);
        }            
			
		if (l == 100)  {
			System_printf ("%s (%s:%d): Timeout waiting for packets from PA\n", tfName, __FILE__, __LINE__);
			testStatus = PA_TEST_FAILED;
			break;
		}
						
		if (t9ReceiveDataPkts (tf, pat, actualPktCount, numPkts))
        {
            /* Error Handling */
			System_printf ("%s (%s:%d): t9ReceiveDataPkts timeout %d\n", tfName,
						   __FILE__, __LINE__);
            System_flush();               
			break;
        }    
	}
	
	/* Verify that the expected and actual received packet counts match */
	for (i = 0; i < T9_NUM_LOCAL_HANDLES; i++)  {
		if (expectedPktCount[i] != actualPktCount[i])  {
			System_printf ("%s (%s:%d): Packet count mismatch for entry %d - expected %d, found %d\n", tfName,
						   __FILE__, __LINE__, i, expectedPktCount[i], actualPktCount[i]);
            System_flush();               
			testStatus = PA_TEST_FAILED;
		}
	}

	/* Clean up and return */
 	paTestL2RecoverAndExit (tf, pat, l2Handles, testStatus, TRUE);
 	
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
}
 		
 
 		 
 
 	
 	


