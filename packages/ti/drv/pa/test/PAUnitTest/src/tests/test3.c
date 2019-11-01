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
#include "test3pkts.h"

#ifdef __LINUX_USER_SPACE
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif


/* Format tx route and checksum verification test
 * This test tests the LLD Pa_formatTxRoute and Pa_formatTxCmd APIs, as well as the
 * checksum verification for IPv4 header, UDP, UDP-lite and TCP headers on reception.
 * This test also tests the transmit timestamp report and timestamp insertion of incoming
 * packets
 * 
 * This test covers the following sub-tests:
 *  - Test the LLD for formatting the checksum and routing commands
 *  - Test the LLD for formatting the CRC, routing and report timestamp commands
 *  - Test the firmware for correct packet routing after processing the checksum command
 *  - Test the PA sub-system for correct IPv4 header checksum calculation and verification
 *  - Test the PA sub-system for correct TCP/UDP/UDP-lite payload checksum calculation
 *  - Test the firmware for the ability to process the report timestamp command
 */

static char *tfName = "paTestTxFormatRoute";

#define PA_T3_SHOW_TIMESTAMP

/* General purpose queue usage */
#define Q_CMD_RECYCLE	    0	/* Command descriptors/buffers recycle */
#define Q_CMD_REPLY			1   /* Replies from PA */
#define Q_MATCH				2	/* Packets from PA which match a lookup */
#define Q_NFAIL             3   /* Packet from PA which failed a lookup */
#define Q_TIMESTAMP         4   /* Packet from PA which contains the tx timestamp */
#define Q_CRC_OPERATION     5   /* Packet from PA which contains the result of CRC operation */
#define Q_DPKT_RECYCLE      6   /* Data packet recycle queue */

/* Handles managed by this test */
#define TEST_NUM_L2_HANDLES     1
#define TEST_NUM_L3_HANDLES     1  /* 2 */
#define TEST_NUM_L4_HANDLES     1  /* 2 */


/* Command reply SW Info identifiers */
#define TEST_SWINFO0_CMD_ID     0x12340000
#define TEST_SWINFO0_STATS_REQ  0x12340001
#define TEST_SWINFO0_TIMESTAMP  0x12340002

static paSysStats_t paTestTxFmtExptStats;    /* Expected stats results */
static uint32_t  paT3txCmdBuf[4][128/sizeof(uint32_t)];
static uint16_t  paT3cmdStackSize[4];

/*
 * Format Tx commands for the following two test cases
 * - Compute CRC with CRC specified offset
 * - Compute CRC with follow Payload operations
*/
static paTestStatus_t testCrcOpTxCmds (tFramework_t *tf, paTest_t *pat, uint8_t *buf)
{
  	Cppi_HostDesc		*hd = NULL;
   	Qmss_Queue       q;    
    paCmdInfo_t      cmdInfo[2];
    paReturn_t       paret;
    int              i, j, nCmds = 0;    
    
    paCmdNextRoute_t routeCmdHost = {
                                    0,              /*  ctrlBitfield */
                                    pa_DEST_HOST,   /* Route - host       */
                                    0,              /* pktType don't care */
                                    0,              /* flow I             */
                                    TF_FIRST_GEN_QUEUE + Q_CRC_OPERATION,   /* Queue                */
                                    0,              /* SWInfo 0 (not used) */
                                    0x00000000,     /* SWInfo 1 (not used) */       
                                    0               /* multiRouteIndex (not used) */    
                                 }; 

    /* Clear up all the command buffers */
    memset(paT3txCmdBuf, 0, sizeof(paT3txCmdBuf));
    routeCmdHost.flowId = tf->tfFlowNum[0];

    cmdInfo[0].cmd = pa_CMD_CRC_OP;
    nCmds++;

    cmdInfo[1].cmd = pa_CMD_NEXT_ROUTE;
    nCmds++;    
    cmdInfo[1].params.route = routeCmdHost;   

    for ( i = 0; i < T3_NUM_CRC_OP_CMD_TO_TEST; i ++) 
    {
        hd  = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
    
    		if (hd == NULL)  
        {
			      System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
			      return (PA_TEST_FAILED);
        }

		    /* Setup the return for the descriptor */
  		  q.qMgr = 0;
  		  q.qNum = tf->QGen[Q_DPKT_RECYCLE];
  		  Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q); 

        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(hd), (uint8_t *)utilgAddr((uint32_t)buf), (uint32_t) T3_CRC_TEST_PKTLEN);
        Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(hd), (uint32_t)T3_CRC_TEST_PKTLEN);

        memcpy(buf, crcTestPayload0, T3_CRC_TEST_PKTLEN);
        cmdInfo[0].params.crcOp = crcTestInfo[i];


        paret = Pa_formatTxCmd (    nCmds,
                                    cmdInfo,
                                    0,             
                                    (Ptr)&paT3txCmdBuf[0][0], /* Command buffer       */
                                    &paT3cmdStackSize[0]);    /* Command size         */        

        if (paret != pa_OK)
        {
      	    System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
            return (PA_TEST_FAILED);
        }

    		Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)(hd), (uint8_t *)&paT3txCmdBuf[0][0], paT3cmdStackSize[0]);  

#ifndef __LINUX_USER_SPACE
        /*
        * Write back the entire cache to make sure that the test packets are updated. 
        * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
        *       the portion of packet which is updated.
        *
        */
#ifdef _TMS320C6X
        CACHE_wbAllL1d(CACHE_WAIT);
        //CACHE_wbAllL2(CACHE_WAIT);
#endif
#endif       
        Qmss_queuePush (tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)hd, (uint32_t)T3_CRC_TEST_PKTLEN, TF_SIZE_DESC, Qmss_Location_TAIL);

        /* give some time for the Tx PDSP to process the command */

       /* Wait for descriptors to return. It is assumed that they are returning in order. */        
        for ( j = 0; j < 100; j++ )
        {
            utilCycleDelay (20000);
            if ((Qmss_getQueueEntryCount (tf->QGen[Q_DPKT_RECYCLE])) >= 1)
                break;
        } 

        if ( j == 100) 
        {
            System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
            return (PA_TEST_FAILED);
        }
        else 
        {
            /* Push the descriptor to the free queue */
   	        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_DPKT_RECYCLE])) & ~15);
   	        if (hd == NULL)  {
   		          System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
   		          return (PA_TEST_FAILED);
            }
         		Qmss_queuePushDesc (tf->QfreeDesc, (Ptr) hd);
            hd = NULL;             
   	    }   

        for (j = 0; j < 100; j++)  
        {      
            uint8_t *pktChk;
            uint16_t crcOutput;
            int      done = 0;
            utilCycleDelay (5000);            
          
            /* Look for packets in the rx packet queue, verify that the UDP port found matches the next value in
             * the fifo, then verify the receive packet information */
            while (Qmss_getQueueEntryCount(tf->QGen[Q_CRC_OPERATION]) > 0)  
            {      
                hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CRC_OPERATION])) & ~15);
                if (hd == NULL)  {
                  System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
                  return (PA_TEST_FAILED);
                }   		    
                pktChk = (uint8_t *) hd->buffPtr;
                pktChk += (hd->buffLen - T3_CRC_TEST_CRC_SIZE);
    
                /* Inspect the CRC computed */
                crcOutput = (pktChk[0] << 8 )+ pktChk[1];
                if ( crcOutput != T3_CRC_TEST_EXPECTED_CRC )
                  return (PA_TEST_FAILED);
                
                /* Return the descriptor/buffer */
                testCommonRecycleLBDesc (tf, hd);
                done = 1;
            }    
            if (done)
              break;
        }
    }

    return (PA_TEST_PASSED);
}


/*
 * Format Tx commands for the following four test cases
 * - SRTP over IPSEC ESP
 * - SRTP over IPSEC AH
 * - Inner IP/UDP Checksum plus IPSEC ESP
 * - Inner IP/UDP Checksum plus IPSEC AH
 */
static paTestStatus_t testFormatTxCmds  (tFramework_t *tf, paTest_t *pat)
{
 
    paCmdInfo_t cmdInfo[10];
    paReturn_t  paret;
    
    
    paCmdNextRoute_t routeCmdSa = {
                                    0,              /*  ctrlBitfield */
                                    pa_DEST_SASS,   /* Route - host       */
                                    0,              /* pktType don't care */
                                    0,              /* flow Id              */
                                    TF_PA_QUEUE_SASS, 	/* Queue                */
                                    0x12345678,     /* SWInfo 0             */
                                    0x0c000000,     /* SWInfo 1 */       
                                    0               /* multiRouteIndex (not used) */
    
                                 };   
                                 
    paCmdNextRoute_t routeCmdEth = {
                                    0,              /*  ctrlBitfield */
                                    pa_DEST_HOST,   /* Route - host       */
                                    pa_EMAC_PORT_1, /* pktType don't care */
                                    0,              /* flow Id              */
                                    TF_FIRST_GEN_QUEUE + 20,  			/* Queue                */
                                    0x87654321,     /* SWInfo 0             */
                                    0x00000000,     /* SWInfo 1 */       
                                    0               /* multiRouteIndex (not used) */
    
                                 };   
                                 
                                
    /* IP header checksum configuration */
    static paTxChksum_t ipChksum = {
        50,     /* Start offset of IP header */
        20,     /* Length of IP header */
        10,     /* Offset to checksum location RELATIVE TO THE START OF THE IP HEADER */
        0,      /* Initial sum */
        1       /* computed value of 0 written as -0 */
    };


    /* The UDP checksum */
    static paTxChksum_t udpChksum = {
        70,     /* Start offset of UDP header */
        508,     /* Checksum length (UDP payload + UDP checksum) */
        6,      /* Offset to checksum location RELATIVE TO THE START OF THE UDP HEADER */
        0x1054, /* Initial value is IPv4 pseudo header checksum value */
        1       /* computed value of 0 written as -0 */
    };
                                
                                
    /* clear up all command buffers */
    memset(paT3txCmdBuf, 0, sizeof(paT3txCmdBuf));
    routeCmdSa.flowId = routeCmdEth.flowId = tf->tfFlowNum[0];
    
    /* Case 1: SRTP over IPSEC ESP */                
    
    /* Commad 0: SA_PAYLOAD */
    cmdInfo[0].cmd = pa_CMD_SA_PAYLOAD;
    cmdInfo[0].params.payload.offset  = 78;
    cmdInfo[0].params.payload.len     = 500;
    cmdInfo[0].params.payload.supData = 0xbabeface;
    
    /* Command 1: IP checksum */
    cmdInfo[1].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[1].params.chksum = ipChksum;
    
    /* Command 2: UDP checksum */
    cmdInfo[2].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[2].params.chksum = udpChksum;
    
    /* Command 3: Next route */
    cmdInfo[3].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[3].params.route = routeCmdSa;
    
    /* Command 4: SA_PAYLOAD */
    cmdInfo[4].cmd = pa_CMD_SA_PAYLOAD;
    cmdInfo[4].params.payload.offset  = 34;
    cmdInfo[4].params.payload.len     = 560;
    cmdInfo[4].params.payload.supData = 0xbaddbeef;
                                           
    /* Command 5: IP Fragment */
    cmdInfo[5].cmd = pa_CMD_IP_FRAGMENT;
    cmdInfo[5].params.ipFrag.ipOffset  = 34;
    cmdInfo[5].params.ipFrag.mtuSize   = 256;
    
    /* Command 6: Next route */
    cmdInfo[6].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[6].params.route = routeCmdEth;
    
    paret = Pa_formatTxCmd (  7,        /* nCmd */
                              cmdInfo,  /* command info */
                              0,        /* offset */
                              (Ptr)&paT3txCmdBuf[0][0], /* Command buffer       */
                              &paT3cmdStackSize[0]);    /* Command size         */
                              
    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }
    
    
    /* Case 2: SRTP over IPSEC AH */                
    
    /* Commad 0: SA_PAYLOAD */
    cmdInfo[0].cmd = pa_CMD_SA_PAYLOAD;
    cmdInfo[0].params.payload.offset  = 82;
    cmdInfo[0].params.payload.len     = 500;
    cmdInfo[0].params.payload.supData = 0xbabeface;
    
    /* Command 1: IP checksum */
    cmdInfo[1].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[1].params.chksum = ipChksum;
    cmdInfo[1].params.chksum.startOffset = 54;
    
    /* Command 2: UDP checksum */
    cmdInfo[2].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[2].params.chksum = udpChksum;
    cmdInfo[1].params.chksum.startOffset = 74;
    
    /* Command 3: Next route */
    cmdInfo[3].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[3].params.route = routeCmdSa;
    
    /* Command 4: SA_PAYLOAD */
    cmdInfo[4].cmd = pa_CMD_SA_PAYLOAD;
    cmdInfo[4].params.payload.offset  = 14;
    cmdInfo[4].params.payload.len     = 568;
    cmdInfo[4].params.payload.supData = 0xbaddbeef;
                                           
    /* Command 5: IP Fragment */
    cmdInfo[5].cmd = pa_CMD_IP_FRAGMENT;
    cmdInfo[5].params.ipFrag.ipOffset  = 34;
    cmdInfo[5].params.ipFrag.mtuSize   = 256;
    
    /* Command 6: Next route */
    cmdInfo[6].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[6].params.route = routeCmdEth;
    cmdInfo[6].params.route.ctrlBitfield = pa_NEXT_ROUTE_PROC_NEXT_CMD;
    
    /* Command 7: Patch Command */
    cmdInfo[7].cmd = pa_CMD_PATCH_DATA;
    cmdInfo[7].params.patch.ctrlBitfield = 0;
    cmdInfo[7].params.patch.nPatchBytes = 8;
    cmdInfo[7].params.patch.offset = 56;
    cmdInfo[7].params.patch.totalPatchSize = 16;
    cmdInfo[7].params.patch.patchData = NULL;
    
    
    paret = Pa_formatTxCmd (  8,        /* nCmd */
                              cmdInfo,  /* command info */
                              0,        /* offset */
                              (Ptr)&paT3txCmdBuf[1][0], /* Command buffer       */
                              &paT3cmdStackSize[1]);    /* Command size         */
                              
    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }
    
    
    /* Case 3: IP/UDP checksum plus IPSEC ESP */                
    
    
    /* Command 1: IP checksum */
    cmdInfo[0].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[0].params.chksum = ipChksum;
    
    /* Command 2: UDP checksum */
    cmdInfo[1].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[1].params.chksum = udpChksum;
    
    /* Command 3: Next route */
    cmdInfo[2].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[2].params.route = routeCmdSa;
    
    /* Command 4: SA_PAYLOAD */
    cmdInfo[3].cmd = pa_CMD_SA_PAYLOAD;
    cmdInfo[3].params.payload.offset  = 34;
    cmdInfo[3].params.payload.len     = 560;
    cmdInfo[3].params.payload.supData = 0xbaddbeef;
                                           
    /* Command 5: IP Fragment */
    cmdInfo[4].cmd = pa_CMD_IP_FRAGMENT;
    cmdInfo[4].params.ipFrag.ipOffset  = 34;
    cmdInfo[4].params.ipFrag.mtuSize   = 256;
    
    /* Command 6: Next route */
    cmdInfo[5].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[5].params.route = routeCmdEth;
    
    paret = Pa_formatTxCmd (  6,        /* nCmd */
                              cmdInfo,  /* command info */
                              0,        /* offset */
                              (Ptr)&paT3txCmdBuf[2][0], /* Command buffer       */
                              &paT3cmdStackSize[2]);    /* Command size         */
                              
    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }
    
    
    /* Case 4:  IP/UDP checksum plus IPSEC AH */                
    
    /* Command 1: IP checksum */
    cmdInfo[0].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[0].params.chksum = ipChksum;
    cmdInfo[0].params.chksum.startOffset = 54;
    
    /* Command 2: UDP checksum */
    cmdInfo[1].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[1].params.chksum = udpChksum;
    cmdInfo[1].params.chksum.startOffset = 74;
    
    /* Command 3: Next route */
    cmdInfo[2].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[2].params.route = routeCmdSa;
    
    /* Command 4: SA_PAYLOAD */
    cmdInfo[3].cmd = pa_CMD_SA_PAYLOAD;
    cmdInfo[3].params.payload.offset  = 14;
    cmdInfo[3].params.payload.len     = 568;
    cmdInfo[3].params.payload.supData = 0xbaddbeef;
                                           
    /* Command 5: IP Fragment */
    cmdInfo[4].cmd = pa_CMD_IP_FRAGMENT;
    cmdInfo[4].params.ipFrag.ipOffset  = 34;
    cmdInfo[4].params.ipFrag.mtuSize   = 256;
    
    /* Command 6: Next route */
    cmdInfo[5].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[5].params.route = routeCmdEth;
    cmdInfo[5].params.route.ctrlBitfield = pa_NEXT_ROUTE_PROC_NEXT_CMD;
    
    /* Command 7: Patch Command */
    cmdInfo[6].cmd = pa_CMD_PATCH_DATA;
    cmdInfo[6].params.patch.ctrlBitfield = 0;
    cmdInfo[6].params.patch.nPatchBytes = 8;
    cmdInfo[6].params.patch.offset = 56;
    cmdInfo[6].params.patch.totalPatchSize = 16;
    cmdInfo[6].params.patch.patchData = NULL;
    
    
    paret = Pa_formatTxCmd (  7,        /* nCmd */
                              cmdInfo,  /* command info */
                              0,        /* offset */
                              (Ptr)&paT3txCmdBuf[3][0], /* Command buffer       */
                              &paT3cmdStackSize[3]);    /* Command size         */
                              
    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }
    
    return (PA_TEST_PASSED);
    
}

/* The IPv4 packet is sent five times. The first time with no
 * checksum calculation. This packet should return with the IP header 
 * and udp checksum flag showing invalid checksums. This is followed
 * by a command to do the IPv4 header checksum only, then the
 * UDP checksum only, and then both checksums. */
#define CMD_BUF_SIZE  (sizeof(pasahoNextRoute_t) + (3 * sizeof(pasahoComChkCrc_t)) + sizeof(pasahoReportTimestamp_t))
#ifdef __LINUX_USER_SPACE
static uint8_t* paT3pkt[5] = {NULL, NULL, NULL, NULL, NULL};
#else
static uint8_t paT3pkt0[sizeof(t3pkt)];
static uint8_t paT3pkt1[sizeof(t3pkt)];
static uint8_t paT3pkt2[sizeof(t3pkt)];
static uint8_t paT3pkt3[sizeof(t3pkt)];
static uint8_t paT3pkt4[sizeof(t3pkt)];
static uint8_t* paT3pkt[5] = {paT3pkt0, paT3pkt1, paT3pkt2, paT3pkt3, paT3pkt4};
#endif
static uint16_t paT3pktSize[5];
static paTestStatus_t testSendIpv4 (tFramework_t *tf, paTest_t *pat)
{
    Cppi_HostDesc *hd;

    uint32_t   cmdStack[5][(sizeof(pasahoNextRoute_t) + (3 * sizeof(pasahoComChkCrc_t)) + sizeof(pasahoReportTimestamp_t)) / sizeof (uint32_t)];
    uint16_t   cmdStackSize[5] = {CMD_BUF_SIZE, CMD_BUF_SIZE, CMD_BUF_SIZE, CMD_BUF_SIZE, CMD_BUF_SIZE};
    uint32_t   rxTimestamp[5], txTimestamp[5];
    uint32_t   rxTimestampMSW[5], txTimestampMSW[5];
    paTimestamp_t timestamp[5];
    paCmdInfo_t cmdInfo[5];
	uint32_t        *swinfo;
    int      i;
    uint16_t   len;
	//volatile int    mdebugWait = 0;
    Qmss_Queue   q;
    uint16_t       tsQueue;

    /* We stored upper 32 bit of RX timestamp in pinfo->word6 */
    pasahoLongInfo_t *pinfo;
    uint32_t		  infoLen;

    /* Because the UDP checksum must be zero, it will not be checked by default 
     * but only when the checksum is generated */
    unsigned int errFlagsExptd[5] = { 0xc, 0x4, 0x8, 0x0, 0x0 }; 
    unsigned int eflags;
    uint8_t psFlags;
#ifdef NSS_GEN2    
    Cppi_DescTag      tag;
#endif 
      
    paReturn_t paret;

    pasahoNextRoute_t  *panr;
    pasahoReportTimestamp_t    *pats;


    paRouteInfo_t  route =  {  pa_DEST_HOST,  /* Route - host         */
                               0,             /* flow Id              */
                               0,  			  /* Queue                */
                               -1,            /* Multi route disabled */
                               0,             /* SWInfo 0             */
                               0,             /* SWInfo 1 */       
                               0,             /* customType : not used */         
                               0,             /* customIndex: not used */     
                               0,             /* pkyType: for SRIO only */    
                               NULL};         /* No commands */
                               
    paCmdNextRoute_t routeCmd = {
                                    0,              /*  ctrlBitfield */
                                    pa_DEST_HOST,   /* Route - host       */
                                    0,              /* pktType don't care */
                                    0,              /* flow Id              */
                                    0,  			/* Queue                */
                                    0,              /* SWInfo 0             */
                                    0,              /* SWInfo 1 */       
                                    0               /* multiRouteIndex (not used) */
    
                                };                              

                        

    /* Create the command stack for the four packet sends */
    memset(cmdStack, 0, sizeof(cmdStack));
    memset(cmdInfo, 0, sizeof(cmdInfo));
    
	/* Route the packet to PA Tx 0 which makes it appear to have arrived over the network */
	route.queue = tf->QPaTx[TF_PA_Q_INPUT];
    route.flowId = tf->tfFlowNum[0];
	routeCmd.queue = tf->QPaTx[TF_PA_Q_INPUT];
    routeCmd.flowId = tf->tfFlowNum[0];
    tsQueue = tf->QGen[Q_TIMESTAMP];
    
    /* Packet 1 - send to PA, route back to QM into PA 0 as a received packet */
    /* report timestamp */
    memcpy(paT3pkt[0], t3pkt, sizeof(t3pkt));
    paT3pktSize[0] = sizeof(t3pkt);
    /* corrupt UDP checksum */
    #ifdef PASS_TEST_NEXT_FAIL_ROUTE 
    paT3pkt[0][36] = 0x55; /* Trigger NextFail route */
    #endif
    *(paT3pkt[0] + 40) = 0xbe;
    *(paT3pkt[0] + 41) = 0xef;
    pats = (pasahoReportTimestamp_t *)&cmdStack[0][0];
    PASAHO_SET_CMDID(pats, PASAHO_PAMOD_REPORT_TIMESTAMP);
    PASAHO_SET_REPORT_FLOW(pats, tf->tfFlowNum[0]);
    PASAHO_SET_REPORT_QUEUE(pats,tsQueue);
    pats->swInfo0 = TEST_SWINFO0_TIMESTAMP;
    
    panr = (pasahoNextRoute_t *)&cmdStack[0][sizeof(pasahoReportTimestamp_t)/sizeof(uint32_t)];

    paret = Pa_formatTxRoute (  NULL,                /* IPv4 header checksum */
                                NULL,                /* No second checksum   */
                                &route,              /* Internal routing     */
                                (Ptr)panr,           /* Command buffer       */
                                &cmdStackSize[0]);   /* Command size         */

    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxRoute returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }

    cmdStackSize[0] += sizeof(pasahoReportTimestamp_t);

    /* Packet 2 - send to PA for IPv4 header checksum calculation, then back to
     * QM in the PA 0 out queue */
    /* insert timestamp */
	memcpy(paT3pkt[1], t3pkt, sizeof(t3pkt));
    paT3pktSize[1] = sizeof(t3pkt);
    *(paT3pkt[1] + 40) = 0xbe;
	*(paT3pkt[1] + 41) = 0xef;

    pats = (pasahoReportTimestamp_t *)&cmdStack[1][0];
    PASAHO_SET_CMDID(pats, PASAHO_PAMOD_REPORT_TIMESTAMP);
    PASAHO_SET_REPORT_FLOW(pats, tf->tfFlowNum[0]);
    PASAHO_SET_REPORT_QUEUE(pats,tsQueue);
    pats->swInfo0 = TEST_SWINFO0_TIMESTAMP;
     
    panr = (pasahoNextRoute_t *)&cmdStack[1][sizeof(pasahoReportTimestamp_t)/sizeof(uint32_t)];

    paret = Pa_formatTxRoute (  &t3pktIpChksum,      /* IPv4 header checksum */
                                NULL,                /* No second checksum   */
                                &route,              /* Internal routing     */
                                (Ptr)panr,           /* Command buffer       */
                                &cmdStackSize[1]);   /* Command size         */


    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxRoute returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }
    
    cmdStackSize[1] += sizeof(pasahoReportTimestamp_t);
    
    /* Packet 3 - send to PA for UDP checksum calculation, then back to QM
     * in the PA 0 out queue */
    /* insert timestamp */
	memcpy(paT3pkt[2], t3pkt, sizeof(t3pkt));
    paT3pktSize[2] = sizeof(t3pkt);
    pats = (pasahoReportTimestamp_t *)&cmdStack[2][0];
    PASAHO_SET_CMDID(pats, PASAHO_PAMOD_REPORT_TIMESTAMP);
    PASAHO_SET_REPORT_FLOW(pats, tf->tfFlowNum[0]);
    PASAHO_SET_REPORT_QUEUE(pats,tsQueue);
    pats->swInfo0 = TEST_SWINFO0_TIMESTAMP;
    
    panr = (pasahoNextRoute_t *)&cmdStack[2][sizeof(pasahoReportTimestamp_t)/sizeof(uint32_t)];

    paret = Pa_formatTxRoute (  &t3pktUdpChksum,     /* IPv4 header checksum */
                                NULL,                /* No second checksum   */
                                &route,              /* Internal routing     */
                                (Ptr)panr,           /* Command buffer       */
                                &cmdStackSize[2]);   /* Command size         */

    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxRoute returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }

    cmdStackSize[2] += sizeof(pasahoReportTimestamp_t);
    
    /* Packet 4 - both checksums, then back to QM for output */
    /* Commad 0: insert timestamp */
	memcpy(paT3pkt[3], t3pkt, sizeof(t3pkt));
    paT3pktSize[3] = sizeof(t3pkt);
    cmdInfo[0].cmd = pa_CMD_REPORT_TX_TIMESTAMP;
    cmdInfo[0].params.txTs.destQueue = tsQueue;
    cmdInfo[0].params.txTs.flowId    = tf->tfFlowNum[0];
    cmdInfo[0].params.txTs.swInfo0   = TEST_SWINFO0_TIMESTAMP;
    
    /* Command 1: IP checksum */
    cmdInfo[1].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[1].params.chksum = t3pktIpChksum;
    
    /* Command 2: UDP checksum */
    //cmdInfo[2].cmd = pa_CMD_TX_CHECKSUM;
    //cmdInfo[2].params.chksum = t3pktUdpChksum;
    
    /* Command 3: CRC */
    cmdInfo[2].cmd = pa_CMD_CRC_OP;
    cmdInfo[2].params.crcOp = t3pktCrc;
    
    /* Command 3: Next route */
    cmdInfo[3].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[3].params.route = routeCmd;
    
    paret = Pa_formatTxCmd (  4,        /* nCmd */
                              cmdInfo,  /* command info */
                              0,        /* offset */
                              (Ptr)&cmdStack[3][0], /* Command buffer       */
                              &cmdStackSize[3]);    /* Command size         */
                              
    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }
    
    
    /* Packet 5 (short packet )- one checksums, Tx padding required then back to QM for output */
    /* Commad 0: insert timestamp */
	memcpy(paT3pkt[4], t3pkt, sizeof(t3pkt));
    paT3pktSize[4] = 50;  /* The packet is truncated to 50 to test padding */
    /* adjust IP and UDP length */
    *(paT3pkt[4] + T3_PKT_OFFSET_IP_LEN  + 0) = 0;
    *(paT3pkt[4] + T3_PKT_OFFSET_IP_LEN  + 1) = 20 + 8 + 8;
    *(paT3pkt[4] + T3_PKT_OFFSET_UDP_LEN + 0) = 0;
    *(paT3pkt[4] + T3_PKT_OFFSET_UDP_LEN + 1) = 8 + 8;
    cmdInfo[0].cmd = pa_CMD_REPORT_TX_TIMESTAMP;
    cmdInfo[0].params.txTs.destQueue = tsQueue;
    cmdInfo[0].params.txTs.flowId    = tf->tfFlowNum[0];
    cmdInfo[0].params.txTs.swInfo0   = TEST_SWINFO0_TIMESTAMP;
    
    /* Command 1: IP checksum */
    cmdInfo[1].cmd = pa_CMD_TX_CHECKSUM;
    cmdInfo[1].params.chksum = t3pktIpChksum;
    
    /* Command 2: Next route */
    cmdInfo[2].cmd = pa_CMD_NEXT_ROUTE;
    cmdInfo[2].params.route = routeCmd;
    cmdInfo[2].params.route.ctrlBitfield |= pa_NEXT_ROUTE_TX_L2_PADDING;
    
    paret = Pa_formatTxCmd (  3,        /* nCmd */
                              cmdInfo,  /* command info */
                              0,        /* offset */
                              (Ptr)&cmdStack[4][0], /* Command buffer       */
                              &cmdStackSize[4]);    /* Command size         */
                              
    if (paret != pa_OK)  {
        System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
        return (PA_TEST_FAILED);
    }
    

    /* Send the packets. Each packet transmission will result in the popping of two free
     * descriptors/buffers since the packet loops back into the QM twice - once with
     * the packet and checksums (and this arrives in Input Q for transmit into PA) and
     * once again with the resulting packet. The descriptors popped off for inter-pa 
     * transmit will wind up in the default return Q */

    /* Recycle the free descriptors right back to the free descriptor queue */
    q.qMgr = 0;
    q.qNum = tf->QfreeDesc;

    for (i = 0; i < 5; i++)   {

        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
        if (hd == NULL)  {
            System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
            return (PA_TEST_FAILED);
        }

        /* Setup the return for the descriptor */
        Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

        /* Attach the data and set the length */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)paT3pkt[i]), paT3pktSize[i]);
        Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, paT3pktSize[i]);

        /* Attach the command in PS data */
        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)cmdStack[i], cmdStackSize[i]);

        /* Send the packet to PDSP 5 */
        //mdebugWait = 1;
        //if(i == 3)
  		//    mdebugHaltPdsp(5);  

        Qmss_queuePush (tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)hd, sizeof(t3pkt), TF_SIZE_DESC, Qmss_Location_TAIL);
        paTestTxFmtExptStats.classify1.nPackets     += 2;     /* Passes once through mac, once through IP */
        paTestTxFmtExptStats.classify1.nTableMatch  += 2;
        #ifdef NSS_GEN2
        paTestTxFmtExptStats.classify1.nPackets     += 1;     /* Pass Ingress4 PDSP0 */
        #endif
        paTestTxFmtExptStats.classify2.nPackets     += 1;     
        paTestTxFmtExptStats.classify2.nUdp         += 1;
        paTestTxFmtExptStats.classify1.nIpv4Packets += 1;
        
        //if(i == 3)
        //    while(mdebugWait);
            
        //utilCycleDelay (3000);

    }

    /* Wait for 5 packets to arrive in the match queue */
    for (i = 0; (i < 100) && (Qmss_getQueueEntryCount(tf->QGen[Q_MATCH]) < 5); i++) 
        utilCycleDelay (5000);

    if (i == 100)  {
        System_printf ("%s (%s:%d): Did not find 5 packets in the match queue\n", tfName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }

    /* Verify the checksum fields in the CPPI descriptor. These must arrive in order */
    for (i = 0; i < 5; i++)  {
    
        int numErr;
        
        if (numErr = Qmss_getQueueEntryCount(tf->QGen[Q_NFAIL]))
        {
            while (numErr--)
            {
                hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_NFAIL])) & ~15);
                if (hd == NULL)  {
                    System_printf ("%s (%s:%d): Error - popped a NULL descriptor\n", tfName, __FILE__, __LINE__);
                    return (PA_TEST_FAILED);
                }
                testCommonRecycleLBDesc (tf, hd); 
            }
        }
    
        /* record the current timestamp */
        Pa_getTimestamp(tf->passHandle, &timestamp[i]);
        
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_MATCH])) & ~15);
        if (hd == NULL)  {
            System_printf ("%s (%s:%d): Error - popped a NULL descriptor\n", tfName, __FILE__, __LINE__);
            return (PA_TEST_FAILED);
        }
        
        #ifndef NSS_GEN2
        psFlags = Cppi_getPSFlags (Cppi_DescType_HOST, (Cppi_Desc *)hd);
        #else
        tag = Cppi_getTag(Cppi_DescType_HOST, (Cppi_Desc *)hd); 
        psFlags = tag.destTagLo;
        #endif
        
        eflags = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc *)hd) & 0xf;
        Cppi_getTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)hd, &rxTimestamp[i]);

        /* Verify the parse information is correct */
        if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
        	System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
        	return (PA_TEST_FAILED);
        }
        rxTimestampMSW[i] = PASAHO_LINFO_READ_TSTAMP_MSB(pinfo);
        testCommonRecycleLBDesc (tf, hd); 

        if (eflags != errFlagsExptd[i])  {
            System_printf ("%s (%s:%d): Expected error flags = %d, found %d\n", tfName, __FILE__, __LINE__, errFlagsExptd[i], eflags);
            //return (PA_TEST_FAILED);
        }
        
        if (psFlags != pa_EMAC_PORT_0)
        {
            System_printf ("%s (%s:%d): Packet with index %d returned with unexpected PS flags = 0x%02x\n", tfName, __FILE__, __LINE__, i, psFlags);
            //return (PA_TEST_FAILED);
        }

        /* Also verify the tx timestamp report packet */
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_TIMESTAMP])) & ~15);
        if (hd == NULL)  {
            System_printf ("%s (%s:%d): Did not find Tx timestamp report packet\n", tfName, __FILE__, __LINE__);
            return (PA_TEST_FAILED);
        }
        
        /* verify the swInfo for the received packet */
        Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);
        
        if (swinfo[0] != TEST_SWINFO0_TIMESTAMP)  {
            System_printf ("%s (%s:%d): Expected timestamp swInfo = 0x%08x, found 0x%08x\n", tfName, __FILE__, __LINE__, TEST_SWINFO0_TIMESTAMP,swinfo[0]);
            return (PA_TEST_FAILED);
        }
        
        Cppi_getTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)hd, &txTimestamp[i]);
        txTimestampMSW[i] = swinfo[1];
        
#ifndef NSS_GEN2        
        if (txTimestamp[i] > rxTimestamp[i])
        {
            System_printf ("%s (%s:%d): Rx timestamp 0x%08x is ealier than Tx timestamp 0x%08x\n", tfName, __FILE__, __LINE__, rxTimestamp[i],txTimestamp[i]);
            return (PA_TEST_FAILED);
        }
#endif        
        
        testCommonRecycleLBDesc (tf, hd);
        

    }
    
#if defined(PA_T3_SHOW_TIMESTAMP) && !defined(NSS_GEN2)        
    for (i = 0; i < 5; i++)  {
        System_printf ("pkt %d: System Timestamp 0x%04x%08x%04x; Rx timestamp 0x%08x%08x; Tx timestamp 0x%08x%08x\n", i, timestamp[i].hi_hi, timestamp[i].hi, timestamp[i].lo, rxTimestampMSW[i], rxTimestamp[i], txTimestampMSW[i], txTimestamp[i]);
    	System_flush();
    }    
#endif        
    
    /* Test the packet adjustment macros. The packet payload is reduced by 40 bytes,
     * the IP and UDP lengths are changed, and the packet checksum commands are updated to
     * reflect the new length. */
    /* IPv4 length */
    len = ((uint16_t)(t3pkt[T3_PKT_OFFSET_IP_LEN+0]) << 8) | t3pkt[T3_PKT_OFFSET_IP_LEN+1];
    len = len - 40;
    t3pkt[T3_PKT_OFFSET_IP_LEN+0] = len >> 8;
    t3pkt[T3_PKT_OFFSET_IP_LEN+1] = len & 0xff;
    
    /* UDP length */
    len = ((uint16_t)(t3pkt[T3_PKT_OFFSET_UDP_LEN+0]) << 8) | t3pkt[T3_PKT_OFFSET_UDP_LEN+1];
    len = len - 40;
    t3pkt[T3_PKT_OFFSET_UDP_LEN+0] = len >> 8;
    t3pkt[T3_PKT_OFFSET_UDP_LEN+1] = len & 0xff;
    
    pa_SET_TX_CHKSUM_LENGTH (&cmdStack[3][sizeof(pasahoReportTimestamp_t)/sizeof(uint32_t)], 1, len);                   /* Update the UDP checksum length */
    pa_SET_TX_CHKSUM_INITVAL(&cmdStack[3][sizeof(pasahoReportTimestamp_t)/sizeof(uint32_t)], 1, testCommonOnesCompAdd(len, T3_PKT_PSEUDO_HDR_CHKSUM_SANS_LEN));
    
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

	/* Send the packet for checksum generation, loop it back and verify the checksum */
    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
    if (hd == NULL)  {
    	System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
        return (PA_TEST_FAILED);
    }

    /* Setup the return for the descriptor */
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

    /* Attach the data and set the length */
#ifdef __LINUX_USER_SPACE
    {
        memcpy (paT3pkt[3], t3pkt, sizeof(t3pkt));
        Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc *)hd, paT3pkt[3], sizeof(t3pkt) - 40);
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, paT3pkt[3], sizeof(t3pkt) - 40);
    }
#else
    Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)t3pkt), sizeof(t3pkt) - 40);
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)t3pkt), sizeof(t3pkt) - 40);
#endif
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, sizeof(t3pkt) - 40);

    /* Attach the command in PS data */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)cmdStack[3], cmdStackSize[3]);

    /* Send the packet to PDSP 5 */
    Qmss_queuePush (tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)hd, sizeof(t3pkt), TF_SIZE_DESC, Qmss_Location_TAIL);
    paTestTxFmtExptStats.classify1.nPackets     += 2;     /* Passes once through mac, once through IP */
    paTestTxFmtExptStats.classify1.nTableMatch  += 2;
    #ifdef NSS_GEN2
    paTestTxFmtExptStats.classify1.nPackets     += 1;     /* Pass through Ingress4 PDSP0 */
    #endif
    paTestTxFmtExptStats.classify2.nPackets     += 1;     
    paTestTxFmtExptStats.classify2.nUdp         += 1;
    paTestTxFmtExptStats.classify1.nIpv4Packets += 1;
    
    
 /* Wait for the packet to arrive in the match queue */
    for (i = 0; (i < 100) && (Qmss_getQueueEntryCount(tf->QGen[Q_MATCH]) < 1); i++) 
        utilCycleDelay (100);

    if (i == 100)  {
        System_printf ("%s (%s:%d): Did not find the length modified packet in the match queue\n");
        return (PA_TEST_FAILED);
    }

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_MATCH])) & ~15);
    if (hd == NULL)  {
        System_printf ("%s (%s:%d): Error - popped a NULL descriptor\n", tfName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }

    eflags = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc *)hd) & 0xf;

    testCommonRecycleLBDesc (tf, hd); 

    if (eflags != 0)  {
        System_printf ("%s (%s:%d): Expected error flags = 0, found %d\n", tfName, __FILE__, __LINE__, eflags);
        return (PA_TEST_FAILED);
    }

    /* Also verify the tx timestamp report packet */
    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_TIMESTAMP])) & ~15);
    if (hd == NULL)  {
        System_printf ("%s (%s:%d): Did not find Tx timestamp report packet\n", tfName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }
    
    /* verify the swInfo for the received packet */
    Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);
    
    if (swinfo[0] != TEST_SWINFO0_TIMESTAMP)  {
        System_printf ("%s (%s:%d): Expected timestamp swInfo = 0x%08x, found 0x%08x\n", tfName, __FILE__, __LINE__, TEST_SWINFO0_TIMESTAMP, swinfo[0]);
        return (PA_TEST_FAILED);
    }
    
    testCommonRecycleLBDesc (tf, hd);

    return (PA_TEST_PASSED);

}    

static int testWaitCmdReply (tFramework_t *tf, paTest_t *pat, int Qcmd, uint32_t swinfo0, int line)
{
	Cppi_HostDesc *hd;
	uint32_t        *swinfo;
    paEntryHandle_t	reth;
    paReturn_t      paret;
    int			    htype;
    int             cmdDest;
	int 		    i;
	
	for (i = 0; i < 100; i++)  {
		if (Qmss_getQueueEntryCount(Qcmd) > 0)  {
			
			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (Qcmd)) & ~15);

            Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);
            
            if (swinfo[0] != swinfo0)  {
                System_printf ("%s (%s:%d): found packet in command reply queue without reply (found 0x%08x)\n", tfName, __FILE__, line, swinfo[0]);
                testCommonRecycleLBDesc (tf, hd);
                continue;
            }

            paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
            if (paret != pa_OK)  
                System_printf ("%s (%s:%d): Pa_forwardResult returned error %d\n", tfName, __FILE__, line, paret);
            
            testCommonRecycleLBDesc (tf, hd); 
                
            return (0);

        }  else  {
            
          utilCycleDelay (1000);

        }
        
	    while (Qmss_getQueueEntryCount ((tf->QGen)[Q_CMD_RECYCLE]) > 0)  {
		    /* Recycle the command descriptor/buffer */
		    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop ((tf->QGen[Q_CMD_RECYCLE]))) & ~15);
		    testCommonRecycleLBDesc (tf, hd);  
	    }            
	}
	
	return (-1);

}	


/* Check the stats  */
static paTestStatus_t t3CheckStats (tFramework_t *tf, paTest_t *pat, Bool clear)
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
 	cmdReply.replyId = TEST_SWINFO0_STATS_REQ;
 	if (testCommonRequestPaStats (tfName, tf, clear, tf->QLinkedBuf1, tf->QGen[Q_CMD_RECYCLE],  &cmdReply))  {
 		System_printf ("%s (%s:%d): testCommonRequestPaStats failed\n", tfName, __FILE__, __LINE__);
 		return (PA_TEST_FAILED);
 	}
    
 	/* Wait for the stats reply */
	for (i = 0; i < 100; i++)  {
		utilCycleDelay (1000);
		if (Qmss_getQueueEntryCount (tf->QGen[Q_CMD_REPLY]) > 0)
			break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Did not find response from PA to stats request command\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}

	/* Recycle the descriptor/buffer returned from the stats request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from stats request\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats request\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
 		
 	/* Format the stats response and compare to the expected results */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_REPLY])) & ~15);
	Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bp, &blen);
	paStats = (paSysStats_t *)Pa_formatStatsReply (tf->passHandle, (paCmd_t)bp);
	  
    #else

	if (testCommonQueryPaStats (tfName, tf, TRUE, paStats))  {
		System_printf ("%s (%s:%d): testCommonQueryPaStats f\ailed\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
    #endif
      
    if (testCommonCompareStats (tfName, (paSysStats_t *)&paTestTxFmtExptStats, paStats))
    	status = PA_TEST_FAILED;
    else
    	status = PA_TEST_PASSED;   
    	  
    if (clear)
        memset (&paTestTxFmtExptStats, 0, sizeof(paTestTxFmtExptStats));
    	  
    #ifndef NSS_GEN2      
     /* Recycle the descriptor and associated buffer back to queue from which it came */
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
    #endif
	
	return (status);
}

static void testCleanup (tFramework_t *tf, paTest_t *pat, paHandleL2L3_t *l2Handles, paHandleL2L3_t *l3Handles, paHandleL4_t *l4Handles, paTestStatus_t testStatus)
{
    Cppi_HostDesc *hd;
    paReturn_t     paret;
    int            cmdDest;
    uint16_t         cmdSize;
    int            cmdCount  = 0;
    int            i;


    paCmdReply_t   reply  =  { pa_DEST_HOST,           /* Destination */
                               TEST_SWINFO0_CMD_ID,    /* SW Info 0   */
                               0,  					   /* Reply queue */
                               0  };                   /* Flow ID     */


	reply.queue = tf->QGen[Q_CMD_REPLY];
	reply.flowId  = tf->tfFlowNum[0];

    /* Close any open handles */
    for (i = 0; i < TEST_NUM_L2_HANDLES; i++)  {
        if (l2Handles[i] != 0)  {
            hd = testCommonDelHandle (tf, &l2Handles[i], tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, &reply, &cmdDest, &cmdSize, &paret);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): testCommonDelHandle returned NULL pointer, paret = %d\n", tfName, __FILE__, __LINE__, paret);
                testStatus = PA_TEST_FAILED;
                continue;
            }

            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): testCommonDelHandle return PA error %d\n", tfName, __FILE__, __LINE__, paret);
                testStatus = PA_TEST_FAILED;
                continue;
            }

            Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
            paTestTxFmtExptStats.classify1.nPackets += 1;
            cmdCount += 1;

        }
    }

    for (i = 0; i < TEST_NUM_L3_HANDLES; i++)  {
        if (l3Handles[i] != 0)  {
            hd = testCommonDelHandle (tf, &l3Handles[i], tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, &reply, &cmdDest, &cmdSize, &paret);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): testCommonDelHandle returned NULL pointer, paret = %d\n", tfName, __FILE__, __LINE__, paret);
                testStatus = PA_TEST_FAILED;
                continue;
            }

            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): testCommonDelHandle return PA error %d\n", tfName, __FILE__, __LINE__, paret);
                testStatus = PA_TEST_FAILED;
                continue;
            }

            Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
            paTestTxFmtExptStats.classify1.nPackets += 1;
            cmdCount += 1;

        }
    }

    for (i = 0; i < TEST_NUM_L4_HANDLES; i++)  {
        if ((l4Handles[i][0] != 0) || (l4Handles[i][1] != 0))  {
            hd = testCommonDelL4Handles (tf, l4Handles[i], tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2, &reply, &cmdDest, &cmdSize, &paret);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): testCommonDelHandle returned NULL pointer, paret = %d\n", tfName, __FILE__, __LINE__, paret);
                testStatus = PA_TEST_FAILED;
                continue;
            }

            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): testCommonDelHandle return PA error %d\n", tfName, __FILE__, __LINE__, paret);
                testStatus = PA_TEST_FAILED;
                continue;
            }

            Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
            paTestTxFmtExptStats.classify2.nPackets += 1;
            #ifdef NSS_GEN2
            paTestTxFmtExptStats.classify1.nPackets += 1;
            #endif
            cmdCount += 1;

        }
    }


    /* Wait for responses from the PA subsystem */
    for (i = 0; i < cmdCount; i++)  {
    	if (testWaitCmdReply (tf, pat, tf->QGen[Q_CMD_REPLY], TEST_SWINFO0_CMD_ID, __LINE__))  {
    		System_printf ("%s (%s:%d): Missing replies to PA commands (expected in queue %d)\n", tfName, __FILE__, __LINE__, tf->QGen[Q_CMD_REPLY]);	
    		testStatus = PA_TEST_FAILED;
    	}
    }
    		

    /* Restore any data on queues used */
    while (Qmss_getQueueEntryCount(tf->QGen[Q_CMD_RECYCLE]) > 0) {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
        testCommonRecycleLBDesc (tf, hd);
    }

    while (Qmss_getQueueEntryCount(tf->QGen[Q_MATCH]) > 0)  {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
        testCommonRecycleLBDesc (tf, hd);
    }

    while (Qmss_getQueueEntryCount(tf->QGen[Q_NFAIL]) > 0)  {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
        testCommonRecycleLBDesc (tf, hd);
    }
            
    if (t3CheckStats (tf, pat, TRUE) == PA_TEST_FAILED)
		testStatus = PA_TEST_FAILED;
            
    /* Return result */                
    pat->testStatus = testStatus;
    
    /* Return */
    Task_exit();
}            

        
static void testHandlePaError (tFramework_t *tf, paTest_t *pat, paReturn_t paret, paHandleL2L3_t *l2Handles, paHandleL2L3_t *l3Handles, paHandleL4_t *l4Handles, Cppi_HostDesc  *hd, int line)
{
	if (paret == pa_OK)
		return;
		
	System_printf ("%s (%s:%d): Error - PA LLD returned error code %d\n", tfName, __FILE__, line, paret); 
	if(hd)testCommonRecycleLBDesc (tf, hd);
	testCleanup (tf, pat, l2Handles, l3Handles, l4Handles, PA_TEST_FAILED);  /* no return */
}      

#ifdef __LINUX_USER_SPACE
void* paTestTxFmtRt (void *args)
{
 	tFramework_t  *tf  = ((paTestArgs_t *)args)->tf;
 	paTest_t      *pat = ((paTestArgs_t *)args)->pat;
#else
void paTestTxFmtRt (UArg a0, UArg a1)
{
    tFramework_t   *tf  = (tFramework_t *)a0;
    paTest_t       *pat = (paTest_t *)a1;
#endif
    Cppi_HostDesc  *hd;
    paReturn_t      paret;

    paTestStatus_t  testStatus = PA_TEST_PASSED;

    int     i;
    int     cmdDest;
    uint16_t  cmdSize;


    paHandleL2L3_t  l2Handles[TEST_NUM_L2_HANDLES] = { 0 };
    paHandleL2L3_t  l3Handles[TEST_NUM_L3_HANDLES] = { 0 /*,  0  */};
    paHandleL4_t    l4Handles[TEST_NUM_L4_HANDLES] = {  {0,0} /*,  {0,0} */ };


    paRouteInfo_t  macMatchRoute =  {  pa_DEST_CONTINUE_PARSE_LUT1, /* Dest - keep parsing    */
                                       0,                           /* Flow ID                */
                                       0,                           /* Queue                  */
                                       -1,                          /* Multi route disabled   */
                                       0,                           /* SW Info 0              */
                                       0,                           /* SW Info 1              */       
                                       0,                           /* customType : pa_CUSTOM_TYPE_NONE  */         
                                       0,                           /* customIndex: not used  */     
                                       0,                           /* pkyType: for SRIO only */    
                                       NULL};                       /* No commands            */

    paRouteInfo_t  ipMatchRoute  =  {  pa_DEST_CONTINUE_PARSE_LUT2, /* Dest - keep parsing    */
                                       0,                           /* Flow ID                */
                                       0,                           /* Queue                  */
                                       -1,                          /* Multi route disabled   */
                                       0,                           /* SW Info 0              */
                                       0,                           /* SW Info 1              */       
                                       0,                           /* customType : pa_CUSTOM_TYPE_NONE  */         
                                       0,                           /* customIndex: not used  */     
                                       0,                           /* pkyType: for SRIO only */    
                                       NULL};                       /* No commands            */


    paRouteInfo_t  udpMatchRoute =  {  pa_DEST_HOST,                /* Dest - send to host    */
                                       0,                           /* Flow ID                */
                                       0,                           /* Queue, filled in below */
                                       -1,                          /* Multi route disabled   */
                                       0,                           /* SW Info 0              */
                                       0,                           /* SW Info 1              */       
                                       0,                           /* customType : not used  */         
                                       0,                           /* customIndex: not used  */     
                                       pa_EMAC_PORT_0,              /* pkyType(SRIO)/EMAC port number (ETH/HOST)*/    
                                       NULL};                       /* No commands            */

    paRouteInfo_t  nfailRoute    =  {  pa_DEST_HOST,                /* Dest - send to host    */
                                       0,                           /* Flow ID                */
                                       0,                           /* Queue, filled in below */
                                       -1,                          /* Multi route disabled   */
                                       0xbabeface,                  /* SW Info 0              */
                                       0,                           /* SW Info 1              */       
                                       0,                           /* customType : not used  */         
                                       0,                           /* customIndex: not used  */     
                                       0,                           /* pkyType: for SRIO only */    
                                       NULL};                       /* No commands            */
                                       
    paRouteInfo_t  nfailRouteMac    =  {  pa_DEST_DISCARD,          /* Dest - send to host    */
                                       0,                           /* Flow ID                */
                                       0,                           /* Queue, filled in below */
                                       -1,                          /* Multi route disabled   */
                                       0xbabeface,                  /* SW Info 0              */
                                       0,                           /* SW Info 1              */       
                                       0,                           /* customType : not used  */         
                                       0,                           /* customIndex: not used  */     
                                       0,                           /* pkyType: for SRIO only */    
                                       NULL};                       /* No commands            */
                                       

    paCmdReply_t cmdReply        =  {  pa_DEST_HOST,                /* Dest - send to host    */
                                       0,                           /* Reply ID (SWInfo 0)    */
                                       0,                           /* Queue                  */
                                       0,  };                       /* Flow ID                */
                                
#ifdef __LINUX_USER_SPACE
    for (i = 0; (i < (sizeof(paT3pkt)/sizeof(paT3pkt[0])) ); i++)  {

        /* Allocate memory for the Packet buffers */
        paT3pkt[i] = (uint8_t *)fw_memAlloc(sizeof(t3pkt), CACHE_LINESZ);
        if(paT3pkt[i] == NULL) {
  	        printf ("%s: memAlloc failed for pkt %d\n", tfName, i);
 		    pat->testStatus = PA_TEST_FAILED;
  	        return (void *)0;
        }
    }
#endif


    /* Runtime initial values */
    udpMatchRoute.queue = (uint16_t) tf->QGen[Q_MATCH];
	udpMatchRoute.flowId= tf->tfFlowNum[0];
    nfailRoute.queue    = (uint16_t) tf->QGen[Q_NFAIL];
	nfailRoute.flowId   = tf->tfFlowNum[0];
    cmdReply.queue      = (uint16_t) tf->QGen[Q_CMD_REPLY];
	cmdReply.flowId     = tf->tfFlowNum[0];

    /* set the t3 command stack size */
    paT3cmdStackSize[0] = paT3cmdStackSize[1] = paT3cmdStackSize[2] = paT3cmdStackSize[3] = 128;

    /* set the template packet */
    memcpy (&t3pkt[0], &t3pkt_template[0], sizeof(t3pkt_template));

    /* Zero the expected stats. They are updated as the test progresses */
    memset (&paTestTxFmtExptStats, 0, sizeof(paTestTxFmtExptStats));


    /* Make table entries for the MAC/IP and L4 port numbers. The packets that are
     * generated are routed back to the PA to be received. This will verify the
     * receive checksum generation as well */

    /* L2 (MAC) entries */
    for (i = 0; i < TEST_NUM_L2_HANDLES; i++)  {

        cmdReply.replyId = TEST_SWINFO0_CMD_ID;
        cmdReply.queue   = tf->QGen[Q_CMD_REPLY];
        hd = testCommonAddMac (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &testPaEthInfo[i], &macMatchRoute, &nfailRouteMac,
                               &l2Handles[i], tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf2,
                               &cmdReply, &cmdDest, &cmdSize, &paret);
        

        testHandlePaError (tf, pat, paret, l2Handles, l3Handles, l4Handles, hd, __LINE__); /* no return on error */


        /* Send the command to the PA */
        Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
        paTestTxFmtExptStats.classify1.nPackets += 1;

        /* Wait for a PA reply, and recycle the command descriptor/buffer */
        if (testWaitCmdReply (tf, pat, tf->QGen[Q_CMD_REPLY], TEST_SWINFO0_CMD_ID, __LINE__))
            testCleanup (tf, pat, l2Handles, l3Handles, l4Handles, PA_TEST_FAILED);  /* no return */

    }


    /* L3 (IP) entries. In this test all IPs link to MAC index 0 */
    cmdReply.replyId = TEST_SWINFO0_CMD_ID;
    cmdReply.queue   = tf->QGen[Q_CMD_REPLY];

    for (i = 0; i < TEST_NUM_L3_HANDLES; i++)  {

        hd = testCommonAddIp (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &testPaIpInfo[i], &ipMatchRoute, &nfailRoute,
                              &l3Handles[i], l2Handles[0], tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf1,
                              &cmdReply, &cmdDest, &cmdSize, &paret);

        testHandlePaError (tf, pat, paret, l2Handles, l3Handles, l4Handles, hd, __LINE__);  /* no return on error */

        /* Send the command to the PA */
        Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
        paTestTxFmtExptStats.classify1.nPackets += 1;

        /* Wait for a PA reply, and recycle the command descriptor/buffer */
        if (testWaitCmdReply (tf, pat, tf->QGen[Q_CMD_REPLY], TEST_SWINFO0_CMD_ID, __LINE__))
            testCleanup (tf, pat, l2Handles, l3Handles, l4Handles, PA_TEST_FAILED);

    }


    /* L4 (UDP/TCP) entries. There is a one to one link between ports and linked IP addresses */
    for (i = 0; i < TEST_NUM_L4_HANDLES; i++)  {

        hd = testCommonAddPort (tf, pa_LUT2_PORT_SIZE_16, (uint32_t)testPaPortInfo[i], &udpMatchRoute, &(l4Handles[i]), &l3Handles[i],
                                tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf1, &cmdReply, &cmdDest,
                                &cmdSize, &paret);
                               

        testHandlePaError (tf, pat, paret, l2Handles, l3Handles, l4Handles, hd, __LINE__);

        /* Send the command to the PA */
        Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
        paTestTxFmtExptStats.classify2.nPackets += 1;
        #ifdef NSS_GEN2
        paTestTxFmtExptStats.classify1.nPackets += 1;
        #endif
        
        /* Wait for a PA reply, and recycle the command descriptor/buffer */
        if (testWaitCmdReply (tf, pat, tf->QGen[Q_CMD_REPLY], TEST_SWINFO0_CMD_ID, __LINE__))
            testCleanup (tf, pat, l2Handles, l3Handles, l4Handles, PA_TEST_FAILED);

    }

     /* 
      * Configure the CRC engine for CRC checksum 
      * The CRC-engine connected to Egress0-PDSP1 should be configured since the CRC command will be issued
      * at Egress0-PDSP1
      */
     
    #ifndef NSS_GEN2 
 	cmdReply.replyId = TEST_SWINFO0_CMD_ID;
    cmdReply.queue   = tf->QGen[Q_CMD_REPLY];
    
    #endif
    hd = testCommonConfigCrcEngine(tf, pa_CRC_INST_6_1 , &t3CrcCfg,  tf->QGen[Q_CMD_RECYCLE], tf->QLinkedBuf3,
                                      &cmdReply, &cmdDest, &cmdSize, &paret);

    testHandlePaError (tf, pat, paret, l2Handles, l3Handles, l4Handles, hd, __LINE__);  /* no return on error */
        
    #ifndef NSS_GEN2     
    
 	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
                            
    /* Wait for a PA reply, and recycle the command descriptor/buffer */
        if (testWaitCmdReply (tf, pat, tf->QGen[Q_CMD_REPLY], TEST_SWINFO0_CMD_ID, __LINE__))
        {
		    System_printf ("%s (%s:%d): testCommonConfigCrcEngine failed\n", tfName, __FILE__, __LINE__);
            testCleanup (tf, pat, l2Handles, l3Handles, l4Handles, PA_TEST_FAILED);
        }
    
    #endif
    
    /* Pass the IPv4 packet, verify returned packets */
    testStatus = testSendIpv4 (tf, pat); 

	if (t3CheckStats (tf, pat, TRUE) == PA_TEST_FAILED)
		testStatus = PA_TEST_FAILED;

    /* Verify Tx command formatting operation */
    testStatus = testFormatTxCmds (tf, pat); 

	if (t3CheckStats (tf, pat, TRUE) == PA_TEST_FAILED)
		testStatus = PA_TEST_FAILED;

    /* Verify Tx command CRC operation */
    testStatus = testCrcOpTxCmds (tf, pat, paT3pkt[0]); 

    /* Close all handles and exit */
    testCleanup (tf, pat, l2Handles, l3Handles, l4Handles, testStatus);
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#endif
}







