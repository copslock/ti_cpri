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
#include "test6pkts.h"

#ifdef __LINUX_USER_SPACE
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif


/* Blind patch and routing test
 * This test tests the LLD Pa_formatRoutePatch and macros for creating patch and routing 
 * command sequences as well as the PDSP firmware for correct operation in executing patch 
 * command and subsequent routing. This test has the following sub-tests:
 * - Test the LLD for formatting the patch and routing commands 
 * - Test the firmware for correct packet routing after packet patching
 */
 
static char *tfName = "paTestPatchRoute";

/* General purpose queue usage */
#define Q_CMD_RECYCLE		  0		/* Command descriptors/buffers recycled here after sent to PA */
#define Q_RESULT			  1		/* Where patched/routed packets are sent  */
#define Q_CMD_REPLY		      2     /* Used for stats reply */

#define N_TEST_PKTS   (sizeof (test6PktInfo) / sizeof (test6PktInfo_t))

static paSysStats_t paTestPatchExpectedStats; 

/* setup the Test Pkt for pktdma */
int t6SetupPktTestInfo(test6PktInfo_t* testInfoPkt, int count, char* tfname)
{
	int returnVal = 0;
#ifdef __LINUX_USER_SPACE
	int i,pktSz;
	uint8_t *pkt;
	for (i = 0; i < count; i++)  {
		/* Store the original information */
        pkt   = testInfoPkt[i].pkt;
		pktSz = testInfoPkt[i].pktSize;
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
 
#ifdef __LINUX_USER_SPACE
void* paTestPatchRoute (void *args)
{
 	tFramework_t  *tf  = ((paTestArgs_t *)args)->tf;
 	paTest_t      *pat = ((paTestArgs_t *)args)->pat;
#else
void paTestPatchRoute (UArg a0, UArg a1)
{	
	tFramework_t *tf  = (tFramework_t *)a0;
 	paTest_t     *pat = (paTest_t *)a1;
#endif
 	uint8_t        *p8;
    paCmdInfo_t  cmdInfo[TEST6_MAX_PATCH+1];
 	
	int           i, j, k;
	int           nq;
  	unsigned int		  fifoData[N_TEST_PKTS+1];
 	pauFifo_t     fifo =  { 0, 0, N_TEST_PKTS+1, NULL };
 	paReturn_t    paret;
 	paPatchInfo_t patch;
 	uint16_t 		  size;
 	int			  offset;
 	int			  packetRetCount = 0;
 	uint32_t        plen;
 	uint8_t		  v;
 	unsigned int		  pnum;
 	
 	paTestStatus_t testStatus = PA_TEST_PASSED;
 	
 	Cppi_HostDesc *hd;
 	Qmss_Queue     q;
 	
 	//volatile int mdebugWait = 1;
 	
 	/* Where to send the packets after the modify if complete */
 	paRouteInfo_t  route =  {  pa_DEST_HOST,  /* Route - host         */
                               0,             /* flow Id              */
                               0,  			  /* Queue                */
                               -1,            /* Multi route disabled */
                               0,             /* SWInfo 0             */
                               0,             /* sw Info 1 */       
                               0,             /* customType : not used  */         
                               0,             /* customIndex: not used  */     
                               pa_EMAC_PORT_0,/* pkyType(SRIO)/EMAC port number (ETH/HOST)*/    
                               NULL};         /* No commands            */
                               
    paCmdNextRoute_t routeCmd = {
                                    0,              /*  ctrlBitfield */
                                    pa_DEST_HOST,   /* Route - host         */
                                    pa_EMAC_PORT_0, /* pktType_emacCtrl     */
                                    0,              /* flow Id              */
                                    0,  			/* Queue                */
                                    0,              /* SWInfo 0             */
                                    0,              /* SWInfo 1 */       
                                    0               /* multiRouteIndex (not used) */
    
                                };                              

    i = t6SetupPktTestInfo(test6PktInfo, (sizeof(test6PktInfo) / sizeof(test6PktInfo_t)), tfName);
    if (i < 0) {
    	System_printf ("%s: (%s:%d): t6SetupPktTestInfo(): setup Pkt Failed \n", tfName, __FILE__, __LINE__);
	    pat->testStatus = PA_TEST_FAILED;
   	    Task_exit();
    }    

 	
	fifo.data = fifoData;
	route.queue = tf->QGen[Q_RESULT];
    route.flowId = tf->tfFlowNum[0];
    routeCmd.queue = route.queue;
    routeCmd.flowId = tf->tfFlowNum[0];
    
	
	memset (&paTestPatchExpectedStats, 0, sizeof (paSysStats_t));
	
    /* Recycle the free descriptor to a queue so the code can easily tell the packet has been sent */
    q.qMgr = 0;
    q.qNum = tf->QGen[Q_CMD_RECYCLE];
	
	
	/* Form the commands and send the packets */
	for (i = 0; i < N_TEST_PKTS; i++)  {
		
		memset (test6PktInfo[i].cmdBuf, 0, TEST6_CMD_SIZE);
		size = TEST6_CMD_SIZE;
		
		/* If there is only one patch command, use the PA utility function to build the command */
		if ((test6PktInfo[i].patch[1] == NULL) && (test6PktInfo[i].patch[2] == NULL))  {
			
			patch.nPatchBytes    = test6PktInfo[i].patch[0]->len;
			patch.totalPatchSize = pa_MAX_PATCH_BYTES;
			patch.offset         = test6PktInfo[i].patch[0]->offset;
			patch.ctrlBitfield   = (test6PktInfo[i].patch[0]->overwrite)?0:pa_PATCH_OP_INSERT;
			patch.patchData      = test6PktInfo[i].patch[0]->bytes;
			
			paret = Pa_formatRoutePatch (&route, &patch, test6PktInfo[i].cmdBuf, &size);
			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): Pa_formatRoutePatch returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				testStatus = PA_TEST_FAILED;
				continue;
			}
			
			/* Offset marks the end of the command info used */
			offset = size;

		
		}  else  {
			
			/* Manually form the command if there are more then then one patch. In this case 
			 * put the route information after all patches */
			for (j = 0; j < TEST6_MAX_PATCH; j++)  {
                if(test6PktInfo[i].patch[j] == NULL)
                    break;
                cmdInfo[j].cmd = pa_CMD_PATCH_DATA;
                cmdInfo[j].params.patch.ctrlBitfield = test6PktInfo[i].patch[j]->overwrite?0:pa_PATCH_OP_INSERT;
                cmdInfo[j].params.patch.ctrlBitfield |= (test6PktInfo[i].patch[j]->delete?pa_PATCH_OP_DELETE:0);
                cmdInfo[j].params.patch.nPatchBytes = test6PktInfo[i].patch[j]->len;
                cmdInfo[j].params.patch.offset = test6PktInfo[i].patch[j]->offset;
                if (test6PktInfo[i].patch[j]->delete)
                {
                    cmdInfo[j].params.patch.patchData = NULL;
                    cmdInfo[j].params.patch.totalPatchSize = 0;
                }
                else
                {
                    cmdInfo[j].params.patch.totalPatchSize = (test6PktInfo[i].patch[j]->len + 3) & ~3;
                    cmdInfo[j].params.patch.patchData = test6PktInfo[i].patch[j]->bytes;
                }    
			}
			
			/* Add in the final route */
            cmdInfo[j].cmd = pa_CMD_NEXT_ROUTE;
            cmdInfo[j].params.route = routeCmd;
            
			paret = Pa_formatTxCmd (j+1, cmdInfo, 0, test6PktInfo[i].cmdBuf, &size);
			if (paret != pa_OK)  {
				System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
				testStatus = PA_TEST_FAILED;
				continue;
			}
            
			/* Offset marks the end of the command info used */
			offset = size;
		}
		
		/* Use a free descriptor to send the packet to PA */
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
        if (hd == NULL)  {
            System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tf->QfreeDesc);
            testStatus = PA_TEST_FAILED;
            break;
        }

        /* Setup the return for the descriptor */
        Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

        /* Attach the data and set the length */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)test6PktInfo[i].pkt), test6PktInfo[i].pktSize);
        Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, test6PktInfo[i].pktSize);

        /* Attach the command in PS data */
        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)test6PktInfo[i].cmdBuf, offset);
        
        //mdebugHaltPdsp (5);
        /* Send the packet to PDSP 5 */
        Qmss_queuePush (tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)hd, test6PktInfo[i].pktSize, TF_SIZE_DESC, Qmss_Location_TAIL);
        //while (mdebugWait);
    
        
        /* Keep count if the packet should return. */
        if (test6PktInfo[i].success == TRUE)  {
			packetRetCount += 1;
			if (commonFifoPushElement (&fifo, (unsigned int)i) < 0)  {
				System_printf ("%s (%s:%d): Test failed - fifo is full\n", tfName, __FILE__, __LINE__);
				testStatus = PA_TEST_FAILED;
				break;
			}
        }  else  {
        	paTestPatchExpectedStats.modify.nCommandFail += 1;
        }
    }
    
    /* Wait for all the packet descriptors to recycle */
    for (i = 0; i < 100; i++)  {
    	
    	if (Qmss_getQueueEntryCount(tf->QGen[Q_CMD_RECYCLE]) == N_TEST_PKTS)
    		break;
    	else
    		utilCycleDelay (500);
    }
    
    if (i == 100)  {
    	System_printf ("%s (%s:%d): Failed to find all command descriptors in the recycle queue\n", tfName, __FILE__, __LINE__);
    	testStatus = PA_TEST_FAILED;
    }
    
    while (Qmss_getQueueEntryCount(tf->QGen[Q_CMD_RECYCLE]) > 0)  {
    	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_CMD_RECYCLE])) & ~15);
    	if (hd == NULL)  {
    		System_printf ("%s (%s:%d): Failed to pop descriptor\n", tfName, __FILE__, __LINE__);
    		testStatus = PA_TEST_FAILED;
    	}
    	Qmss_queuePushDesc (tf->QfreeDesc, (Ptr) hd);
    }
    	    	
    
    /* Verify the patch as packets arrive */
    for (i = 0; (i < 100) && (packetRetCount > 0); i++)  {
    	
        uint8_t psFlags;
#ifdef NSS_GEN2    
        Cppi_DescTag tag;
#endif 
        
    	if (Qmss_getQueueEntryCount(tf->QGen[Q_RESULT]) == 0)  {
    		utilCycleDelay (500);
    		continue;
    	}
    	
    	while (Qmss_getQueueEntryCount(tf->QGen[Q_RESULT]) > 0)  {
    		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[Q_RESULT])) & ~15);
			if (hd == NULL)  {
				System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
				testStatus = PA_TEST_FAILED;
				continue;
			} 
			
			packetRetCount -= 1;
            
            #ifndef NSS_GEN2
            psFlags = Cppi_getPSFlags (Cppi_DescType_HOST, (Cppi_Desc *)hd);
            #else
            tag = Cppi_getTag(Cppi_DescType_HOST, (Cppi_Desc *)hd); 
            psFlags = tag.destTagLo;
            #endif
            
            if (psFlags != pa_EMAC_PORT_0)
            {
                System_printf ("%s (%s:%d): Packet with index %d returned with expected PS flags = 0x%02x\n", tfName, __FILE__, __LINE__, i, psFlags);
            }
            
			/* The packets should arrive in order */
			Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *)hd, &p8, &plen);
			
			pnum = commonFifoPopElement (&fifo, &nq);
			if (nq <= 0)  {
				System_printf ("%s (%s:%d): Could not pop an element off of the software tracking fifo (n = %d)\n",
								tfName, __FILE__, __LINE__, nq);
				testCommonRecycleLBDesc (tf, hd);
				testStatus = PA_TEST_FAILED;
				continue;
			}
		
			/* Check if the packet contains sequential data */
			for (j = 1, v = p8[0]; j < plen; j++)  {
				v = (v + 1) & 0xff;
				if (v != p8[j])  {
					System_printf ("%s (%s:%d): packet %d, byte %d: Expected byte value 0x%08x, found 0x%08x\n",
								   tfName, __FILE__, __LINE__, pnum, j, v, p8[j]);
					testStatus = PA_TEST_FAILED;
					testCommonRecycleLBDesc (tf, hd);
					break;
				}
			}
		
			/* Make sure the packet length matches the expected length */
			k = test6PktInfo[pnum].pktSize;
			for (j = 0; j < TEST6_MAX_PATCH; j++)  {
				if (test6PktInfo[pnum].patch[j] != NULL)  {
					if (test6PktInfo[pnum].patch[j]->overwrite == FALSE)
						k = k + test6PktInfo[pnum].patch[j]->len;
					if (test6PktInfo[pnum].patch[j]->delete == TRUE)
						k = k - test6PktInfo[pnum].patch[j]->len;
				}
			}
			if (k != plen)  {
				System_printf ("%s (%s:%d): expected packet %d to have return length of %d, found %d\n",
							   tfName, __FILE__, __LINE__, pnum, k, plen);
				testStatus = PA_TEST_FAILED;
				testCommonRecycleLBDesc (tf, hd);
			} 
			
			testCommonRecycleLBDesc (tf, hd);	
    	}
    }
		
										          
	if (packetRetCount > 0)  {
		System_printf ("%s (%s:%d): Missing %d packets\n", tfName, __FILE__, __LINE__, packetRetCount);
		testStatus = PA_TEST_FAILED;
	}		
	
	if (testCommonCheckStats (tf, pat, tfName, &paTestPatchExpectedStats, tf->QLinkedBuf1, 
	                       			  tf->QGen[Q_CMD_RECYCLE], tf->QGen[Q_CMD_REPLY], TRUE) == PA_TEST_FAILED)
		testStatus = PA_TEST_FAILED;	
			
		/* Return result */                
    pat->testStatus = testStatus; 
    
    /* Return */
#ifdef __LINUX_USER_SPACE
    return (void *)0;
#else
    Task_exit();
#endif
   
}
