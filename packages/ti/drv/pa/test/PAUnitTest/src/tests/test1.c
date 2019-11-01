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

/* Unconfigured PA test.
 * This test sends a valid data packet to each of the PASS processing stage before any configuration
 * has been provided. The PA should simply silently discard the packets
 * 
 * a0 is a pointer to the test framework structure
 * a1 is a pointer to the paTest_t structure for this test, as initialized in testMain.
 * 
 * General purpose Queue assignment 
 * 0 - Recycle queue for the data packets
 * 1 - Recycle queue for the request stats request
 * 2 - Destination queue for the request stats reply
 * 
 */
 
 static char *tfName = "paTestUnconfigured";
 
 #define Q_DATA_RECYCLE		  0
 #define Q_REQ_STATS_RECYCLE  1
 #define Q_STATS_REPLY		  2
 
#ifdef NSS_GEN2
/* PASS Gen2 */
#define PAU_NUM_PROC_STAGES    9
#define PAU_NUM_C1_PKTS        5
#define PAU_NUM_C1_NO_MATCHES  1
#define PAU_NUM_C1_DISCARDS    4
#define PAU_NUM_C1_INV_CTRLS   0
#define PAU_NUM_C2_PKTS        0
#define PAU_NUM_C2_DISCARDS    0
#define PAU_NUM_C2_INV_CTRLS   0
#define PAU_NUM_BAD_CMDS       4
 
#else
/* PASS Gen1 */
#define PAU_NUM_PROC_STAGES    6
#ifdef __LINUX_USER_SPACE
#define PAU_NUM_C1_PKTS        2
#define PAU_NUM_C1_NO_MATCHES  1
#define PAU_NUM_C1_DISCARDS    1
#define PAU_NUM_C1_INV_CTRLS   1
#else
#define PAU_NUM_C1_PKTS        3
#define PAU_NUM_C1_NO_MATCHES  1
#define PAU_NUM_C1_DISCARDS    1
#define PAU_NUM_C1_INV_CTRLS   2
#endif
#define PAU_NUM_C2_PKTS        1
#define PAU_NUM_C2_DISCARDS    1
#define PAU_NUM_C2_INV_CTRLS   1
#define PAU_NUM_BAD_CMDS       2
#endif 
 
#ifdef _TMS320C6X
#pragma DATA_ALIGN(paTucPktData, 16)
const unsigned char paTucPktData[] = {
#else
const unsigned char paTucPktData[] __attribute__ ((aligned (16))) = {
#endif
                            0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,                      /* Dest MAC */
                            0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,                      /* Src MAC  */
                            0x08, 0x00,                                              /* Ethertype = IPv4 */
                            0x45, 0x00, 0x00, 0x6c,                                  /* IP version, services, total length */
                            0x00, 0x00, 0x00, 0x00,                                  /* IP ID, flags, fragment offset */
                            0x05, 0x11, 0xa5, 0x97,                                  /* IP ttl, protocol (UDP), header checksum */
                            0x9e, 0xda, 0x6d, 0x0a,                                  /* Source IP address */
                            0x01, 0x02, 0x03, 0x04,                                  /* Destination IP address */
                            0x12, 0x34, 0x05, 0x55,                                  /* UDP source port, dest port */
                            0x00, 0x58, 0xe1, 0x98,                                  /* UDP len, UDP checksum */
                            0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,          /* 80 bytes of payload data */
                            0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
                            0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
                            0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
                            0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
                            0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61,
                            0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
                            0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
                            0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
                            0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81  };
                            
unsigned char* paTucPkt;

const paSysStats_t paTucExpectedStats  =  {
			
					 //		1,		/* Clear flag */
							
						{   PAU_NUM_C1_PKTS, 	    /* Classify 1 nPackets */
							0,			            /* Classify 1 nIpv4Packets */
							0,			            /* Classify 1 nIpv4PacketsInner*/
							0,			            /* Classify 1 nIpv6Packets */
							0,			            /* Classify 1 nIpv6PacketsInner */
							0,			            /* Classify 1 nCustomPackets */
 							0,			            /* Classify 1 nSrioPackets */
							0,			            /* Classify 1 nLlcSnapFail */
							0,			            /* Classify 1 nTableMatch */
							PAU_NUM_C1_NO_MATCHES,	/* Classify 1 nNoTableMatch */
							0,			            /* Classify 1 nIpFrag */
							0,			            /* Classify 1 nIpDepthOverflow */ 
							0,			            /* Classify 1 nVlanDepthOverflow */
							0,			            /* Classify 1 nGreDepthOverflow */
							0,			            /* Classify 1 nMplsPackets */
							0,			            /* Classify 1 nParseFail */
							0,			            /* Classify 1 nInvalidIPv6Opt */
							0,			            /* Classify 1 nTxIpFrag */
							PAU_NUM_C1_DISCARDS,	/* Classify 1 nSilentDiscard */
							PAU_NUM_C1_INV_CTRLS,   /* Classify 1 nInvalidControl */
							0,			            /* Classify 1 nInvalidState */
							0  },		            /* Classify 1 nSystemFail */
							                        
						{	PAU_NUM_C2_PKTS, 		/* Classify 2 nPackets */
							0,			            /* Classify 2 nUdp */
							0,			            /* Classify 2 nTcp */
							0,			            /* Classify 2 nCustom */
							0,			            /* Classify 2 nCommandFail */
							0,			            /* Classify 2 nInvalidComReplyDest */
							PAU_NUM_C2_DISCARDS,	/* Classify 2 nSilentDiscard */
							PAU_NUM_C2_INV_CTRLS },	/* Classify 2 nInvalidControl */
							
						{   PAU_NUM_BAD_CMDS  },   	/* Modify nCommandFail */
						
						{   0  }		            /* Common nIdAllocationFail */
						
			}; 
 
void paTestUcRecoverAndExit (tFramework_t *tf, paTest_t *pat, paTestStatus_t testResult)
{
	Cppi_HostDesc *hd;
	
	/* Set the result code */
	pat->testStatus = testResult;
	
	/* Restore the packet recycle queues */
	while (Qmss_getQueueEntryCount ((tf->QGen)[Q_DATA_RECYCLE]))  {
		
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_DATA_RECYCLE])) & ~15);
		
		hd->buffPtr = hd->origBuffPtr;
		hd->buffLen = hd->origBufferLen;
		
		Qmss_queuePushDesc (tf->QfreeDesc, (Ptr)hd);
	}
	

	
	/* Return */
	Task_exit();
}

#ifdef __LINUX_USER_SPACE
void* paTestUnconfigured (void *args)
{
 	tFramework_t  *tf  = ((paTestArgs_t *)args)->tf;
 	paTest_t      *pat = ((paTestArgs_t *)args)->pat;
#else
 void paTestUnconfigured (UArg a0, UArg a1)
 {
 	tFramework_t  *tf  = (tFramework_t *)a0;
 	paTest_t      *pat = (paTest_t *)a1;
#endif
 	Cppi_HostDesc *hd;
#ifndef  NSS_GEN2   
 	uint8_t		  *bp;
 	uint32_t	   blen;
 	paCmdReply_t   reply;
#endif    
 	paSysStats_t   paStats1;
    paSysStats_t  *paStats = &paStats1; 
 	Qmss_Queue     q;
 	paTestStatus_t result;
 	int  i;
 	unsigned int len;
    int numPkts = 0;
 	
 	//volatile int mdebugWait = 1;

#ifdef __LINUX_USER_SPACE
    /* Allocate memory for the Packet buffers */
    paTucPkt = (uint8_t *)fw_memAlloc(sizeof(paTucPktData), CACHE_LINESZ);
    if(paTucPkt == NULL) {
  	    printf ("%s: memAlloc failed for pkt %d\n", tfName, i);
 		pat->testStatus = PA_TEST_FAILED;
  	    return (void *)0;
    }
    memcpy(paTucPkt, paTucPktData, sizeof(paTucPktData));
#else    
    paTucPkt = (unsigned char*)paTucPktData;	
#endif
 	
	/* Send the data packets to the PA */
	for (i = 0; i < PAU_NUM_PROC_STAGES; i++)
    {
    
#if defined(__LINUX_USER_SPACE) && !defined(NSS_GEN2)
       /* Linux has added an unconditional match IP entry, skip LUT1_1 */
       if ( i == 1)
            continue;
#endif    
    
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Queue pop failed for free descriptor queue (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
			paTestUcRecoverAndExit (tf, pat, PA_TEST_FAILED);
		}
		
		/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tf->QGen[Q_DATA_RECYCLE];
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
		
		/* Attach the data and set the length */
#ifdef PA_SIM_BUG_4BYTES
  		len = (sizeof(paTucPktData)+3)& ~3;
#else
		len = sizeof(paTucPktData);
#endif
        Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc *)hd, (Ptr)utilgAddr((uint32_t)paTucPkt), len);
  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (Ptr)utilgAddr((uint32_t)paTucPkt), len);
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, len);
  	    /* Make sure there is no control info.  */
  	    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
  		
  		Qmss_queuePush (tf->QPaTx[i + TF_PA_Q_CONFIG_BASE], (Ptr)utilgAddr((uint32_t)hd), len, TF_SIZE_DESC, Qmss_Location_TAIL);
        
        numPkts++;
  	
	}
	
	/* Wait for all descriptors to pop up in the recycle queue */
	for (i = 0; i < 100; i++)  {
		utilCycleDelay (1000);
		if (Qmss_getQueueEntryCount (tf->QGen[Q_DATA_RECYCLE]) >= numPkts)
			break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Did not find expected recycled descriptors in queue %d\n", tfName, __FILE__, __LINE__, tf->QGen[Q_DATA_RECYCLE]);
		paTestUcRecoverAndExit (tf, pat, PA_TEST_FAILED);
	}
    
    /* Wait for state update to complete */
	utilCycleDelay (10000);
    
#ifndef  NSS_GEN2   
	
	/* Request stats from the PA */
	reply.dest    = pa_DEST_HOST;
	reply.replyId = 0;
	reply.queue   = tf->QGen[Q_STATS_REPLY];
	reply.flowId  = tf->tfFlowNum[0];
	
	if (testCommonRequestPaStats (tfName, tf, TRUE, tf->QLinkedBuf1, tf->QGen[Q_REQ_STATS_RECYCLE], &reply))  {
		System_printf ("%s (%s:%d): testCommonRequestPaStats failed\n", tfName, __FILE__, __LINE__);
		paTestUcRecoverAndExit (tf, pat, PA_TEST_FAILED);
	}
	
	/* Wait for the stats reply */
	for (i = 0; i < 100; i++)  {
		utilCycleDelay (1000);
		if (Qmss_getQueueEntryCount (tf->QGen[Q_STATS_REPLY]) > 0)
			break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Did not find response from PA to stats request command\n", tfName, __FILE__, __LINE__);
		paTestUcRecoverAndExit (tf, pat, PA_TEST_FAILED);
	}
	
	/* Recycle the descriptor/buffer returned from the stats request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_REQ_STATS_RECYCLE])) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from stats request\n", tfName, __FILE__, __LINE__);
		paTestUcRecoverAndExit (tf, pat, PA_TEST_FAILED);
	}
	
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats request\n", tfName, __FILE__, __LINE__);
		paTestUcRecoverAndExit (tf, pat, PA_TEST_FAILED);
	}
	
	/* Format the stats response and compare to the expected results */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_STATS_REPLY])) & ~15);
	Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bp, &blen);
	paStats = Pa_formatStatsReply (tf->passHandle, (paCmd_t)bp);
    
#else

	if (testCommonQueryPaStats (tfName, tf, TRUE, paStats))  {
		System_printf ("%s (%s:%d): testCommonQueryPaStats failed\n", tfName, __FILE__, __LINE__);
		paTestUcRecoverAndExit (tf, pat, PA_TEST_FAILED);
	}
    
#endif    
	  
    if (testCommonCompareStats (tfName, (paSysStats_t *)&paTucExpectedStats, paStats))
    	result = PA_TEST_FAILED;
    else
    	result = PA_TEST_PASSED;   
    	  
    #ifndef  NSS_GEN2      
     	/* Recycle the descriptor and associated buffer back to queue from which it came */
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tfName, __FILE__, __LINE__);
		result = PA_TEST_FAILED;
	}
    #endif
	
	paTestUcRecoverAndExit (tf, pat, result);
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
                        
 }
	

	
	
  		
		
		
			
		




