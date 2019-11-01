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

/* Custom LUT1/LUT2 Lookup test
 * This test tests the PA LLD custom configuration functions, as well as the
 * PDSP firmware custom lookup handling.
 * This test also includes a 16-bit CRC test vector verification with varied
 * CRC payload formats. 
 * This test invokes the following LLD APIs
 *  - Pa_setCustomLUT1
 *  - Pa_addCustomLUT1
 *  - Pa_setCustomLUT2
 *  - Pa_addCustomLUT2
 *  - Pa_configCmdSet
 *  - Pa_configCrcEngine
 *  - Pa_addMac
 *  - Pa_addIp
 *  - Pa_addPort
 *  - Pa_delHandle
 *  - Pa_delL4Handle
 *  - Pa_forwardResult
 *  - Pa_requestStats
 *  - Pa_formatStatsReply
 * This test has the following sub-tests
 *  - Test the ability to configure multiple custom LUT1 sets
 *  - Test the ability to add custom LUT1 entries
 *  - Test the ability to configure multiple custom LUT2 sets
 *  - Test the ability to add custom LUT2 entries
 *  - Test the ability to configure command sets
 *  - Test the LLD for the ability to detect configuration error when custom LUT1, LUT2 and command set APIs are invoked
 *  - Test the firmware for the ability to route the custom LUT1 traffic in addition to the standard IP traffic
 *  - Test the firmware for the ability to route the custom LUT2 traffic in addition to the standard UDP traffic
 *  - Test the firmware for the ability to copy the packet data to the PS info section in the packet descriptor after lookup
 *  - Test the firmware for the ability to perform CRC verification with various CRC payload formats.
 */

static char *tfName = "paTestCustomLookup";
static paSysStats_t t7stats;
 
#define T7_NUM_PACKET_ITERATIONS	1  		/* Number of times the packet stream is passed through */
#define T7_UDP_BYTE_PKT_ID			10		/* The byte that contains the packet ID */

 
/* General purpose queue usage */
#define Q_MATCH		  	  0		/* Packets from PA which match a lookup criteria */
#define Q_NFAIL		      1		/* Packets from PA which matches a mac lookup, but failed an L3 lookup */
#define Q_PARSE_ERR		  2		/* Packets which resulted in a parse error */
#define Q_DPKT_RECYCLE	  3		/* Data packet recycle queue */
#define Q_UDP_MATCH		  4     /* Where a UDP (not custom L4) is routed */
#define Q_PAYLOAD_MATCH	  5     /* For payload packet only */
 
 
#include "test7pkts.h"

static paCmdReply_t cmdReply = {  pa_DEST_HOST,				/* Dest */
 							      0,						/* Reply ID (returned in swinfo0) */
 							   	  0,						/* Queue */
 							      0 };						/* Flow ID */
                                  
/* CRC Configuration of CRC for WCDMA FP */
static paCrcConfig_t   t7CrcCfg = {
                                    0,                      /* ctrlBitfield (left shift and no insert */
                                    pa_CRC_SIZE_16,
                                    0x80050000,             /* polynomial */
                                    0x0                     /* initValue */
                                  };     
                                  
/* Global configurations */
static paCmdSetConfig_t      t7CmdSetCfg = 
        {
            64      /* Number of command sets */    
        };    
        
static  paSysConfig_t  t7GlobalCfg = 
        {
            NULL,                   /* pProtoLimit */ 
            NULL,                   /* pOutIpReassmConfig */
            NULL,                   /* pInIpReassmConfig */
            &t7CmdSetCfg,
            NULL,                   /* pUsrStatsConfig */
            NULL,                   /* pQueueDivertConfig */
            NULL,                   /* pPktVerify */
            NULL                    /* pQueueBounceConfig */
        };

Cppi_HostDesc *t7GetRxPkt (tFramework_t *tf)
{
	Cppi_HostDesc *hd = NULL;
    
    if (Qmss_getQueueEntryCount(tf->QGen[Q_PAYLOAD_MATCH]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_PAYLOAD_MATCH])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			return (NULL);
		}
        
        /* just drop the payload packet */
		testCommonRecycleLBDesc (tf, hd);
        hd = NULL;
	} 
	
	if (Qmss_getQueueEntryCount(tf->QGen[Q_MATCH]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_MATCH])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			return (NULL);
		}
		
	} else if (Qmss_getQueueEntryCount(tf->QGen[Q_UDP_MATCH]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_UDP_MATCH])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			return (NULL);
		}
		
	} else if (Qmss_getQueueEntryCount(tf->QGen[Q_NFAIL]) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_NFAIL])) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
			return (NULL);
		}
	} 
     	
	return (hd);
}
		

int t7RxPackets (tFramework_t *tf, int counts[])
{
	Cppi_HostDesc    *hd;
	uint8_t		     *rxPkt;
	pasahoLongInfo_t *pinfo;
	int32_t			 *swinfo;
	unsigned int			  l4Offset;
	uint32_t		      infoLen;
	uint32_t			  flags;
    unsigned int              eflags;
	int				  idx;
	int				  i;
	
	while ((hd = t7GetRxPkt (tf)) != NULL)  {
		
		/* Get the parse information, make sure there is an L4 offset */
		if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
			System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
			testCommonRecycleLBDesc (tf, hd);
			return (-1);
		}
		
		flags = PASAHO_LINFO_READ_HDR_BITMASK(pinfo);
		
		if ((flags & PASAHO_HDR_BITMASK_UDP) == 0)  {
            /* UDP offset is not valid for Custom Lookup */
            l4Offset = PASAHO_LINFO_READ_START_OFFSET(pinfo);
		}
        else
        {
		    l4Offset = PASAHO_LINFO_READ_L4_OFFSET(pinfo);
        }
		
		rxPkt = (uint8_t *)hd->buffPtr;	
		/* 8 bytes for UDP header */
		idx = rxPkt[8 + l4Offset + T7_UDP_BYTE_PKT_ID];
		
		if ((idx < 0) || (idx >= sizeof(t7PktInfo) / sizeof(pktTestInfo_t)))  {
			System_printf ("%s (%s:%d): The packet index byte in the packet had an invalid value (%d)\n",
							tfName, __FILE__, __LINE__, idx);
			testCommonRecycleLBDesc (tf, hd);
			return (-1);
		}
		
		/* Verify that the returned info matches what was expectd */
		testCommonComparePktInfo (tfName, t7PktInfo[idx].info, pinfo);
		
		/* Verify that the value in swinfo0 matches the expected value */
		Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);
		if (t7PktInfo[idx].idx != swinfo[0])
			System_printf ("%s (%s:%d): Packet with index %d returned swinfo0 0x%08x, expected 0x%08x\n",
				tfName, __FILE__, __LINE__, idx, swinfo[0], t7PktInfo[idx].idx);
		
        
        eflags = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc *)hd) & 0xf;


        if (eflags != 0)  {
            System_printf ("%s (%s:%d): Packet with index %d returned with error flags = 0x%02x\n", tfName, __FILE__, __LINE__, idx, eflags);
        }
		
		testCommonRecycleLBDesc (tf, hd);
		
		/* Subtract the packet from the expected packet count */
		if (counts[idx] <= 0)
			System_printf ("%s (%s:%d): Packet index %d had an expected count of 0, but found another packet\n",
						   tfName, __FILE__, __LINE__, idx);
		else		
			counts[idx] -= 1;
			
	}
	
	/* Return the number of packets still expected */
	for (i = idx = 0; i < sizeof(t7PktInfo) / sizeof(pktTestInfo_t); i++)
		idx += counts[i];
		
	return (idx);
}
	
#ifdef __LINUX_USER_SPACE
void* paTestCustom (void *args)
{
 	tFramework_t  *tf  = ((paTestArgs_t *)args)->tf;
 	paTest_t      *pat = ((paTestArgs_t *)args)->pat;
#else
void paTestCustom (UArg a0, UArg a1)
{	
	tFramework_t *tf  = (tFramework_t *)a0;
	paTest_t     *pat = (paTest_t *)a1;
#endif
 	Cppi_HostDesc  *hd;
    paCtrlInfo_t    ctrlInfo;
 	paReturn_t      paret;
 	int  			cmdDest;
 	uint16_t		cmdSize;
    int             crcEngInst;
	
	paTestStatus_t tstat = PA_TEST_PASSED;
	
	int i;
	int counts[sizeof(t7PktInfo) / sizeof(pktTestInfo_t)];
	
	//volatile int mdebugWait = 1;

	cmdReply.flowId  = tf->tfFlowNum[0];
    
	/* Setup the Test Information for pktDma */
    i = setupPktTestInfo(t7PktInfo, sizeof(t7PktInfo) / sizeof(pktTestInfo_t), tfName);
	if (i < 0) {
		System_printf ("%s: (%s:%d): setupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
   	    pat->testStatus = PA_TEST_FAILED;
		Task_exit();
    } 		
	
	memset (&t7stats, 0, sizeof(paSysStats_t));
	memset (counts, 0, sizeof(counts));
    
    /* Issue the global configuration command  */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = t7GlobalCfg;
    cmdReply.replyId  = T7_CMD_SWINFO0_GLOBAL_CFG_ID;
	cmdReply.queue    = tf->QCommonCmdRep;
 	hd = testCommonGlobalConfig (tf, &ctrlInfo, tf->QLinkedBuf3, tf->QLinkedBuf3, 
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);
                                         
    if (hd == NULL)  {
   			
   	  System_printf ("%s: (%s:%d): testCommonGlobalConfig(): no buffer \n", tfName, __FILE__, __LINE__);
		 pat->testStatus = PA_TEST_FAILED;
		 Task_exit();
    }								 
    
    /* Send command */
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    
	if (testCommonWaitCmdReply (tf, pat, tfName, cmdReply.queue, T7_CMD_SWINFO0_GLOBAL_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): testCommonGlobalConfig\n", tfName, __FILE__, __LINE__);
		pat->testStatus = PA_TEST_FAILED;
		Task_exit();
 	}
    
    
    /* 
     * Command set initialization 
     */
    t7CmdSet4[0].params.copy  = t7CopyHdr;
    t7CmdSet4[1].params.split = t7SplitOP;
    t7CmdSet4[2].params.patch = t7Patch; 
    t7CmdSet4[3].params.crcOp = t7CrcOP; 
    t7CmdSet4[4].params.copy  = t7CopyTail;  
    t7CmdSetCmd4.params.cmdSet.index = 4;  
    
    t7CmdSet11[0].params.crcOp = t7CrcOP_1; 
    t7CmdSetCmd11.params.cmdSet.index = 11;  
    
    t7CmdSet22[0].params.split = t7SplitOP_2;
    t7CmdSet22[1].params.patch = t7Patch_2; 
    t7CmdSet22[2].params.crcOp = t7CrcOP_2; 
    t7CmdSetCmd22.params.cmdSet.index = 22; 
     
    t7CmdSet44[0].params.crcOp = t7CrcOP_4; 
    t7CmdSetCmd44.params.cmdSet.index = 44; 
     
    t7CmdSet55[0].params.crcOp = t7CrcOP_5; 
    t7CmdSetCmd55.params.cmdSet.index = 55; 
	
	if (testCommonSetupTest (tf, &t7stats, &t7ATestSetup, tfName, __FILE__, __LINE__))  {
		System_printf ("%s (%s:%d): Test setup failed\n", tfName, __FILE__, __LINE__);
		pat->testStatus = PA_TEST_FAILED;
		Task_exit();
	}
    
    /* 
     * Configure the CRC engine for Custom CRC checksum 
     * The CRC-engine connected to PDSP4 should be configured to perform CRC validation within the command set
     */
     
    #ifndef NSS_GEN2  
 	cmdReply.replyId = T7_CMD_SWINFO0_CRC_CFG_ID;  
 	cmdReply.queue = tf->QCommonCmdRep;
    crcEngInst = 4;
    #else
    crcEngInst = pa_CRC_INST_5_0;
    #endif
    
    hd = testCommonConfigCrcEngine(tf, crcEngInst, &t7CrcCfg,  tf->QLinkedBuf3, tf->QLinkedBuf3,
                                   &cmdReply, &cmdDest, &cmdSize, &paret);
         
    if (paret != pa_OK)
    {
		System_printf ("%s (%s:%d): testCommonConfigCrcEngine failed with error code = %d\n", tfName, __FILE__, __LINE__, paret);
		if(hd)testCommonRecycleLBDesc (tf, hd);
		pat->testStatus = PA_TEST_FAILED;
		Task_exit();
    }
    
    #ifndef NSS_GEN2                                
                                   
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
                            
	if (testCommonWaitCmdReply (tf, pat, tfName, cmdReply.queue, T7_CMD_SWINFO0_CRC_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): testCommonConfigCrcEngine failed\n", tfName, __FILE__, __LINE__);
		pat->testStatus = PA_TEST_FAILED;
		Task_exit();
 	}
    
    #endif
    
    /* Check Command Statistics */
	if (testCommonCheckStats (tf, pat, tfName, &t7stats, tf->QLinkedBuf1, tf->QLinkedBuf1, tf->QCommonCmdRep, TRUE) != PA_TEST_PASSED)  {
		System_printf ("%s (%s:%d): testCommonCheckStats failed\n", tfName, __FILE__, __LINE__);
		tstat = PA_TEST_FAILED;
	}

	/* Add a packet index byte into the UDP data, recompute the UDP checksum */
	for (i = 0; i < sizeof(t7PktInfo) / sizeof(pktTestInfo_t); i++)  {

		if (t7PktInfo[i].idx != 0)  {
			counts[i] = T7_NUM_PACKET_ITERATIONS;
			/* 8 bytes for UDP header size */
			t7PktInfo[i].pkt[8 + T7_UDP_BYTE_PKT_ID + TF_GET_UDP_OFFSET(t7PktInfo[i].info)] = i;
		}
		
		utilCompUdpChksums (&t7PktInfo[i], TRUE);
		utilCompIpChksums (&t7PktInfo[i], TRUE);
	}
	
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
#endif
    
    //mdebugHaltPdsp(4); 
		
	/* Send the packets. The max number of packets that can be sent is limited by the number
	 * of free buffers since all sends are done at one time with no buffer recycling */
	for (i = 0; i < T7_NUM_PACKET_ITERATIONS; i++)
		testCommonSendPackets (tf, tfName, &t7stats, t7PktInfo, sizeof(t7PktInfo) / sizeof(pktTestInfo_t), 0);
		
    //mdebugWait = 1;     
		
	//while (mdebugWait);
	/* utilCycleDelay (5000); */
	
	/* Verify packets as they are received */
	for (i = 0; i < 100; i++)  {
		if (t7RxPackets (tf, counts) == 0)
			break;
		else
			utilCycleDelay (1000);
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%n): Not all expected received packets found\n", tfName, __FILE__, __LINE__);
		tstat = PA_TEST_FAILED;
	}
		
	if (testCommonCheckStats (tf, pat, tfName, &t7stats, tf->QLinkedBuf1, tf->QLinkedBuf1, tf->QCommonCmdRep, TRUE) != PA_TEST_PASSED)  {
		System_printf ("%s (%s:%d): testCommonCheckStats failed\n", tfName, __FILE__, __LINE__);
		tstat = PA_TEST_FAILED;
	}
		
	if (testCommonTeardownTest (tf, &t7stats, &t7ATestSetup, tfName, __FILE__, __LINE__))  {
		System_printf ("%s (%s:%d): Test teardown failed\n", tfName, __FILE__, __LINE__);
		pat->testStatus = PA_TEST_FAILED;
		Task_exit();
	}	
	
	if (testCommonCheckStats (tf, pat, tfName, &t7stats, tf->QLinkedBuf1, tf->QLinkedBuf1, tf->QCommonCmdRep, TRUE) != PA_TEST_PASSED)  {
		System_printf ("%s (%s:%d): testCommonCheckStats failed\n", tfName, __FILE__, __LINE__);
		tstat = PA_TEST_FAILED;
	}
	
	pat->testStatus = tstat;
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#else
	Task_exit ();
#endif
	

}	




