/*
 *
 * Copyright (C) 2010-2014 Texas Instruments Incorporated - http://www.ti.com/ 
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



/* Common test utilities */
#include "../pautest.h"

#define TEST_COMMON_STATS_REQ_ID	0x12123487

volatile int pdsp_halt = 0;

#define TF_MAX_USR_STATS    64
/* Request stats from the PA */
int testCommonRequestPaStats (char *fname, tFramework_t *tf, Bool reset, int32_t QSource, int32_t QRecycle,  paCmdReply_t *reply)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	paReturn_t     paret;
	uint16_t       cmdSize;
	int			   cmdDest;
	uint32_t	   pktcmd = PASAHO_PACFG_CMD;
	volatile int   mdebugWait = 0;
	
	/* Pop a descriptor with an associated buffer to use for the command request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QSource)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop descriptor from queue %d\n", fname, __FILE__, __LINE__, QSource);
		return (-1);
	}
	
	/* Make sure there is a buffer attached and get the size */
	if (hd->buffPtr == (uint32_t)NULL)  {
		System_printf ("%s (%s:%d): Descriptor from queue %d had no associated buffer\n", fname, __FILE__, __LINE__, QSource);
		return (-1);
	}
	cmdSize = hd->origBufferLen;
	
	paret = Pa_requestStats (tf->passHandle, reset, (paCmd_t)hd->buffPtr, &cmdSize, reply, &cmdDest);

	
	if (paret != pa_OK)  {
		System_printf ("%s (%s:%d): Pa_requestStats returned error code %d\n", fname, __FILE__, __LINE__, paret);
		Qmss_queuePushDesc (QSource, (Ptr)hd);
		return (-1);
	}
	
	
	/* Set the packet length and the buffer length. The buffer length MUST NOT
	 * exceed the packet length */
	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, cmdSize);
  	hd->buffLen = cmdSize;
	
	/* Mark the packet as a configuration packet and setup for the descriptor return */
  	Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&pktcmd, 4);
  	
  	q.qMgr = 0;
  	q.qNum = QRecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);	
  	
	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
	while (mdebugWait);
	return (0);
}

#ifdef NSS_GEN2
/* Query stats from the PA */
int testCommonQueryPaStats (char *tName, tFramework_t *tf, Bool clear, paSysStats_t *paStats)
{
	paReturn_t     paret;
    
	paret = Pa_querySysStats (tf->passHandle, clear, paStats);
	
	if (paret != pa_OK)  {
		System_printf ("%s (%s:%d): Pa_querySysStats returned error code %d\n", tName, __FILE__, __LINE__, paret);
		return (-1);
	}
    
	return (0);
}
#endif

/* Compare stats and report any differences */
int testCommonCompareStats (char *fname, paSysStats_t *expected, paSysStats_t *actual)
{
	int retval = 0;
	
    
    #ifndef NSS_GEN2
	if (expected->classify1.nPackets != actual->classify1.nPackets)  {
		System_printf ("%s: Stat classify1.nPackets expected %d, found %d\n", fname, expected->classify1.nPackets, actual->classify1.nPackets);
		retval = 1;
	}
    #endif
    
  	if (expected->classify1.nIpv4Packets != actual->classify1.nIpv4Packets)  {
  		System_printf ("%s: Stat classify1.nIpv4Packets expected %d, found %d\n", fname, expected->classify1.nIpv4Packets, actual->classify1.nIpv4Packets);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nIpv6Packets != actual->classify1.nIpv6Packets)  {
  		System_printf ("%s: Stat classify1.nIpv6Packets expected %d, found %d\n", fname, expected->classify1.nIpv6Packets, actual->classify1.nIpv6Packets);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nCustomPackets != actual->classify1.nCustomPackets)  {
  		System_printf ("%s: Stat classify1.nCustomPackets expected %d, found %d\n", fname, expected->classify1.nCustomPackets, actual->classify1.nCustomPackets);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nSrioPackets != actual->classify1.nSrioPackets)  {
  		System_printf ("%s: Stat classify1.nSrioPackets expected %d, found %d\n", fname, expected->classify1.nSrioPackets, actual->classify1.nSrioPackets);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nLlcSnapFail != actual->classify1.nLlcSnapFail)  {
  		System_printf ("%s: Stat classify1.nLlcSnapFail expected %d, found %d\n", fname, expected->classify1.nLlcSnapFail, actual->classify1.nLlcSnapFail);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nTableMatch != actual->classify1.nTableMatch)  {
  		System_printf ("%s: Stat classify1.nTableMatch expected %d, found %d\n", fname, expected->classify1.nTableMatch, actual->classify1.nTableMatch);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nNoTableMatch != actual->classify1.nNoTableMatch)  {
  		System_printf ("%s: Stat classify1.nNoTableMatch expected %d, found %d\n", fname, expected->classify1.nNoTableMatch, actual->classify1.nNoTableMatch);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nIpFrag != actual->classify1.nIpFrag)  {
  		System_printf ("%s: Stat classify1.nIpFrag expected %d, found %d\n", fname, expected->classify1.nIpFrag, actual->classify1.nIpFrag);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nIpDepthOverflow != actual->classify1.nIpDepthOverflow)  {
  		System_printf ("%s: Stat classify1.nIpDepthOverflow expected %d, found %d\n", fname, expected->classify1.nIpDepthOverflow, actual->classify1.nIpDepthOverflow);
  		retval = 1;
  	}
  	
 	if (expected->classify1.nVlanDepthOverflow != actual->classify1.nVlanDepthOverflow)  {
 		System_printf ("%s: Stat classify1.nVlanDepthOverflow expected %d, found %d\n", fname, expected->classify1.nVlanDepthOverflow, actual->classify1.nVlanDepthOverflow);
 		retval = 1;
 	}
 	
  	if (expected->classify1.nGreDepthOverflow != actual->classify1.nGreDepthOverflow)  {
  		System_printf ("%s: Stat classify1.nGreDepthOverflow expected %d, found %d\n", fname, expected->classify1.nGreDepthOverflow, actual->classify1.nGreDepthOverflow);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nMplsPackets != actual->classify1.nMplsPackets)  {
  		System_printf ("%s: Stat classify1.nMplsPackets expected %d, found %d\n", fname, expected->classify1.nMplsPackets, actual->classify1.nMplsPackets);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nParseFail != actual->classify1.nParseFail)  {
  		System_printf ("%s: Stat classify1.nParseFail expected %d, found %d\n", fname, expected->classify1.nParseFail, actual->classify1.nParseFail);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nInvalidIPv6Opt != actual->classify1.nInvalidIPv6Opt)  {
  		System_printf ("%s: Stat classify1.nInvalidIPv6Opt expected %d, found %d\n", fname, expected->classify1.nInvalidIPv6Opt, actual->classify1.nInvalidIPv6Opt);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nTxIpFrag != actual->classify1.nTxIpFrag)  {
  		System_printf ("%s: Stat classify1.nTxIpFrag expected %d, found %d\n", fname, expected->classify1.nTxIpFrag, actual->classify1.nTxIpFrag);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nSilentDiscard != actual->classify1.nSilentDiscard)  {
  		System_printf ("%s: Stat classify1.nSilentDiscard expected %d, found %d\n", fname, expected->classify1.nSilentDiscard, actual->classify1.nSilentDiscard);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nInvalidControl != actual->classify1.nInvalidControl)  {
  		System_printf ("%s: Stat classify1.nInvalidControl expected %d, found %d\n", fname, expected->classify1.nInvalidControl, actual->classify1.nInvalidControl);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nInvalidState != actual->classify1.nInvalidState)  {
  		System_printf ("%s: Stat classify1.nInvalidState expected %d, found %d\n", fname, expected->classify1.nInvalidState, actual->classify1.nInvalidState);
  		retval = 1;
  	}
  	
  	if (expected->classify1.nSystemFail != actual->classify1.nSystemFail)  {
  		System_printf ("%s: Stat classify1.nSystemFail expected %d, found %d\n", fname, expected->classify1.nSystemFail, actual->classify1.nSystemFail);
  		retval = 1;
  	}
  	
  	System_flush ();
  	
  	
	if (expected->classify2.nPackets != actual->classify2.nPackets)  {
		System_printf ("%s: Stat classify2.nPackets expected %d, found %d\n", fname, expected->classify2.nPackets, actual->classify2.nPackets);
		retval = 1;
	}
  	
  	
  	if (expected->classify2.nUdp != actual->classify2.nUdp)  {
  		System_printf ("%s: Stat classify2.nUdp expected %d, found %d\n", fname, expected->classify2.nUdp, actual->classify2.nUdp);
  		retval = 1;
  	}
  	
  	if (expected->classify2.nTcp != actual->classify2.nTcp)  {
  		System_printf ("%s: Stat classify2.nTcp expected %d, found %d\n", fname, expected->classify2.nTcp, actual->classify2.nTcp);
  		retval = 1;
  	}
  	
  	if (expected->classify2.nCustom != actual->classify2.nCustom)  {
  		System_printf ("%s: Stat classify2.nCustom expected %d, found %d\n", fname, expected->classify2.nCustom, actual->classify2.nCustom);
  		retval = 1;
  	}
  	
  	
  	if (expected->classify2.nSilentDiscard != actual->classify2.nSilentDiscard)  {
  		System_printf ("%s: Stat classify2.nSilentDiscard expected %d, found %d\n", fname, expected->classify2.nSilentDiscard, actual->classify2.nSilentDiscard);
  		retval = 1;
  	}
  	
  	if (expected->classify2.nInvalidControl != actual->classify2.nInvalidControl)  {
  		System_printf ("%s: Stat classify2.nInvalidControl expected %d, found %d\n", fname, expected->classify2.nInvalidControl, actual->classify2.nInvalidControl);
  		retval = 1;
  	}
  	
  	if (expected->modify.nCommandFail != actual->modify.nCommandFail)  {
  		System_printf ("%s: Stat modify.nCommandFail expected %d, found %d\n", fname, expected->modify.nCommandFail, actual->modify.nCommandFail);
  		retval = 1;
  	}
  	
  	System_flush ();
    
#ifdef __LINUX_USER_SPACE
    return (0);
#else
  	return (retval);
#endif
}

/* Compare RA stats and report any differences */
int testCommonCompareRaStats (char *fname, paRaStats_t *expected, paRaStats_t *actual)
{
	int retval = 0;
    int i;
    
    for ( i = 0; i < pa_RA_NUM_GROUPS; i++)
    {
        if (expected->group[i].nReasmPackets != actual->group[i].nReasmPackets)
        {
		    System_printf ("%s RA (%d): Stat nReasmPackets expected %d, found %d\n", fname, i, expected->group[i].nReasmPackets, actual->group[i].nReasmPackets);
            retval = -1;
        }                                     
    
        if (expected->group[i].nFrags != actual->group[i].nFrags)
        {
		    System_printf ("%s RA (%d): Stat nFrags expected %d, found %d\n", fname, i, expected->group[i].nFrags, actual->group[i].nFrags);
            retval = -1;
        }
        
        if (expected->group[i].nPackets != actual->group[i].nPackets)
        {
		    System_printf ("%s RA (%d): Stat nPackets expected %d, found %d\n", fname, i, expected->group[i].nPackets, actual->group[i].nPackets);
            retval = -1;
        }

        if (expected->group[i].nCxtTOwSOP != actual->group[i].nCxtTOwSOP)
        {
		    System_printf ("%s RA (%d): Stat nCxtTOwSOP expected %d, found %d\n", fname, i, expected->group[i].nCxtTOwSOP, actual->group[i].nCxtTOwSOP);
            retval = -1;
        }
        
        if (expected->group[i].nCxtTOwSOPBytes != actual->group[i].nCxtTOwSOPBytes)
        {
		    System_printf ("%s RA (%d): Stat nCxtTOwSOPBytes expected %d, found %d\n", fname, i, expected->group[i].nCxtTOwSOPBytes, actual->group[i].nCxtTOwSOPBytes);
            retval = -1;
        }
        
        if (expected->group[i].nCxtTOwoSOP != actual->group[i].nCxtTOwoSOP)
        {
		    System_printf ("%s RA (%d): Stat nCxtTOwoSOP expected %d, found %d\n", fname, i, expected->group[i].nCxtTOwoSOP, actual->group[i].nCxtTOwoSOP);
            retval = -1;
        } 
        
        if (expected->group[i].nCxtTOwoSOPBytes != actual->group[i].nCxtTOwoSOPBytes)
        {
		    System_printf ("%s RA (%d): Stat nCxtTOwoSOPBytes expected %d, found %d\n", fname, i, expected->group[i].nCxtTOwoSOPBytes, actual->group[i].nCxtTOwoSOPBytes);
            retval = -1;
        }      
        
        if (expected->group[i].nZeroByte != actual->group[i].nZeroByte)
        {
		    System_printf ("%s RA (%d): Stat nZeroByte expected %d, found %d\n", fname, i, expected->group[i].nZeroByte, actual->group[i].nZeroByte);
            retval = -1;
        }
        
        if (expected->group[i].nIpv6Overlap != actual->group[i].nIpv6Overlap)
        {
		    System_printf ("%s RA (%d): Stat nIpv6Overlap expected %d, found %d\n", fname, i, expected->group[i].nIpv6Overlap, actual->group[i].nIpv6Overlap);
            retval = -1;
        }
        
        if (expected->group[i].nIpv6OverlapBytes != actual->group[i].nIpv6OverlapBytes)
        {
		    System_printf ("%s RA (%d): Stat nIpv6OverlapBytes expected %d, found %d\n", fname, i, expected->group[i].nIpv6OverlapBytes, actual->group[i].nIpv6OverlapBytes);
            retval = -1;
        }
        
        if (expected->group[i].nLargePackets != actual->group[i].nLargePackets)
        {
		    System_printf ("%s RA (%d): Stat nLargePackets expected %d, found %d\n", fname, i, expected->group[i].nLargePackets, actual->group[i].nLargePackets);
            retval = -1;
        }
        
        if (expected->group[i].nIpv4TcpErr != actual->group[i].nIpv4TcpErr)
        {
		    System_printf ("%s RA (%d): Stat nIpv4TcpErr expected %d, found %d\n", fname, i, expected->group[i].nIpv4TcpErr, actual->group[i].nIpv4TcpErr);
            retval = -1;
        }
        
        if (expected->group[i].nFragLenErr != actual->group[i].nFragLenErr)
        {
		    System_printf ("%s RA (%d): Stat nFragLenErr expected %d, found %d\n", fname, i, expected->group[i].nFragLenErr, actual->group[i].nFragLenErr);
            retval = -1;
        }
        
        if (expected->group[i].nIpv4IllegalIHL != actual->group[i].nIpv4IllegalIHL)
        {
		    System_printf ("%s RA (%d): Stat nIpv4IllegalIHL expected %d, found %d\n", fname, i, expected->group[i].nIpv4IllegalIHL, actual->group[i].nIpv4IllegalIHL);
            retval = -1;
        }
        
        if (expected->group[i].nSmallFragments != actual->group[i].nSmallFragments)
        {
		    System_printf ("%s RA (%d): Stat nSmallFragments expected %d, found %d\n", fname, i, expected->group[i].nSmallFragments, actual->group[i].nSmallFragments);
            retval = -1;
        }
        
        if (expected->group[i].nIllegalFragLen != actual->group[i].nIllegalFragLen)
        {
		    System_printf ("%s RA (%d): Stat nIllegalFragLen expected %d, found %d\n", fname, i, expected->group[i].nIllegalFragLen, actual->group[i].nIllegalFragLen);
            retval = -1;
        }
        
        if (expected->group[i].nCxtCompletedDiscard != actual->group[i].nCxtCompletedDiscard)
        {
		    System_printf ("%s RA (%d): Stat nCxtCompletedDiscard expected %d, found %d\n", fname, i, expected->group[i].nCxtCompletedDiscard, actual->group[i].nCxtCompletedDiscard);
            retval = -1;
        }
        
        if (expected->group[i].nCxtCompletedDiscardBytes != actual->group[i].nCxtCompletedDiscardBytes)
        {
		    System_printf ("%s RA (%d): Stat nCxtCompletedDiscardBytes expected %d, found %d\n", fname, i, expected->group[i].nCxtCompletedDiscardBytes, actual->group[i].nCxtCompletedDiscardBytes);
            retval = -1;
        }
    }    
  	
  	System_flush ();
    
  	return (retval);
}

void testCommonDispRaStats (char *fname, paRaStats_t *pStats)
{
    int i;
    
    for ( i = 0; i < pa_RA_NUM_GROUPS; i++)
    {
        System_printf ("%s RA (%d) Statistics:\n", fname, i);
        System_printf ("------------------------------------------------\n");
		System_printf ("nReasmPackets             =  0x%08x\n", pStats->group[i].nReasmPackets);
		System_printf ("nFrags                    =  0x%08x\n", pStats->group[i].nFrags);
		System_printf ("nPackets                  =  0x%08x\n", pStats->group[i].nPackets);
		System_printf ("nCxtTOwSOP                =  0x%08x\n", pStats->group[i].nCxtTOwSOP);
		System_printf ("nCxtTOwSOPBytes           =  0x%08x\n", pStats->group[i].nCxtTOwSOPBytes);
		System_printf ("nCxtTOwoSOP               =  0x%08x\n", pStats->group[i].nCxtTOwoSOP);
		System_printf ("nCxtTOwoSOPBytes          =  0x%08x\n", pStats->group[i].nCxtTOwoSOPBytes);
		System_printf ("nZeroByte                 =  0x%08x\n", pStats->group[i].nZeroByte);
		System_printf ("nIpv6Overlap              =  0x%08x\n", pStats->group[i].nIpv6Overlap);
		System_printf ("nIpv6OverlapBytes         =  0x%08x\n", pStats->group[i].nIpv6OverlapBytes);
		System_printf ("nLargePackets             =  0x%08x\n", pStats->group[i].nLargePackets);
		System_printf ("nIpv4TcpErr               =  0x%08x\n", pStats->group[i].nIpv4TcpErr);
		System_printf ("nFragLenErr               =  0x%08x\n", pStats->group[i].nFragLenErr);
		System_printf ("nIpv4IllegalIHL           =  0x%08x\n", pStats->group[i].nIpv4IllegalIHL);
		System_printf ("nSmallFragments           =  0x%08x\n", pStats->group[i].nSmallFragments);
		System_printf ("nIllegalFragLen           =  0x%08x\n", pStats->group[i].nIllegalFragLen);
		System_printf ("nCxtCompletedDiscard      =  0x%08x\n", pStats->group[i].nCxtCompletedDiscard);
		System_printf ("nCxtCompletedDiscardBytes =  0x%08x\n", pStats->group[i].nCxtCompletedDiscardBytes);
     	System_flush ();
    }
}    

/* Request User stats from the PA */
int testCommonRequestUsrStats (char *fname, tFramework_t *tf, Bool reset, int32_t QSource, int32_t QRecycle,  paCmdReply_t *reply, paUsrStats_t  *usrStats)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	paReturn_t     paret;
	uint16_t       cmdSize;
	int			   cmdDest;
	uint32_t	   pktcmd = PASAHO_PACFG_CMD;
	//volatile int   mdebugWait = 0;
	
	/* Pop a descriptor with an associated buffer to use for the command request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QSource)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop descriptor from queue %d\n", fname, __FILE__, __LINE__, QSource);
		return (-1);
	}
	
	/* Make sure there is a buffer attached and get the size */
	if (hd->buffPtr == (uint32_t)NULL)  {
		System_printf ("%s (%s:%d): Descriptor from queue %d had no associated buffer\n", fname, __FILE__, __LINE__, QSource);
		return (-1);
	}
	cmdSize = hd->origBufferLen;
	
	paret = Pa_requestUsrStats (tf->passHandle, reset, (paCmd_t)hd->buffPtr, &cmdSize, reply, &cmdDest, usrStats);
	
	if (paret != pa_OK)  {
		System_printf ("%s (%s:%d): Pa_requestUsrStats returned error code %d\n", fname, __FILE__, __LINE__, paret);
		Qmss_queuePushDesc (QSource, (Ptr)hd);
		return (-1);
	}
	
    if (cmdSize)
    {
	    /* Set the packet length and the buffer length. The buffer length MUST NOT
	    * exceed the packet length */
	    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, cmdSize);
  	    hd->buffLen = cmdSize;
	
	    /* Mark the packet as a configuration packet and setup for the descriptor return */
  	    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&pktcmd, 4);
  	
  	    q.qMgr = 0;
  	    q.qNum = QRecycle;
  	    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);	
  	
	    Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    }
    else
    {
    
        /* Recycle the descriptor and associated buffer back to queue from which it came */
	    if (testCommonRecycleLBDesc (tf, hd))  {
		    System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", fname, __FILE__, __LINE__);
		    return (-1);
	    }
    
    }
	return (0);

}

/* Request User stats from the PA */
int testCommonRequestUsrStatsList (char *fname, tFramework_t *tf, Bool reset, int numCnts, uint16_t *usrCnt, int32_t QSource, int32_t QRecycle,  paCmdReply_t *reply, paUsrStats_t  *usrStats)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	paReturn_t     paret;
	uint16_t       cmdSize;
	int			   cmdDest;
	uint32_t	   pktcmd = PASAHO_PACFG_CMD;
	//volatile int   mdebugWait = 0;
	
	/* Pop a descriptor with an associated buffer to use for the command request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QSource)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Failed to pop descriptor from queue %d\n", fname, __FILE__, __LINE__, QSource);
		return (-1);
	}
	
	/* Make sure there is a buffer attached and get the size */
	if (hd->buffPtr == (uint32_t)NULL)  {
		System_printf ("%s (%s:%d): Descriptor from queue %d had no associated buffer\n", fname, __FILE__, __LINE__, QSource);
		return (-1);
	}
	cmdSize = hd->origBufferLen;
	
	paret = Pa_requestUsrStatsList (tf->passHandle, reset, numCnts, usrCnt, (paCmd_t)hd->buffPtr, &cmdSize, reply, &cmdDest, usrStats);
	
	if (paret != pa_OK)  {
		System_printf ("%s (%s:%d): Pa_requestUsrStats returned error code %d\n", fname, __FILE__, __LINE__, paret);
		Qmss_queuePushDesc (QSource, (Ptr)hd);
		return (-1);
	}
	
    if (cmdSize)
    {
	    /* Set the packet length and the buffer length. The buffer length MUST NOT
	    * exceed the packet length */
	    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, cmdSize);
  	    hd->buffLen = cmdSize;
	
	    /* Mark the packet as a configuration packet and setup for the descriptor return */
  	    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&pktcmd, 4);
  	
  	    q.qMgr = 0;
  	    q.qNum = (uint16_t)QRecycle;
  	    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);	
  	
	    Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
    }
    else
    {
    
        /* Recycle the descriptor and associated buffer back to queue from which it came */
	    if (testCommonRecycleLBDesc (tf, hd))  {
		    System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", fname, __FILE__, __LINE__);
		    return (-1);
	    }
    
    }
	return (0);

}

/* Compare stats and report any differences */
int testCommonCompareUsrStats (char *fname, paUsrStats_t *expected, paUsrStats_t *actual)
{
	int retval = 0;
    int i;
    
    for (i = 0; i < pa_USR_STATS_MAX_64B_COUNTERS; i++)
    {
	    if (expected->count64[i] != actual->count64[i])  {
		    System_printf ("%s: Usr Stat count64[%d] expected %d, found %d\n", fname, i, expected->count64[i], actual->count64[i]);
		    retval = 1;
	    }
    }
    
    for (i = 0; i < pa_USR_STATS_MAX_32B_COUNTERS; i++)
    {
	    if (expected->count32[i] != actual->count32[i])  {
		    System_printf ("%s: Usr Stat count32[%d] expected %d, found %d\n", fname, i, expected->count32[i], actual->count32[i]);
		    retval = 1;
	    }
    }
  	
  	System_flush ();
  	return (retval);
}

/* Compare stats and report any differences */
int testCommonCompareUsrStatsList (char *fname, paUsrStats_t *expected, paUsrStats_t *actual, int num64bStats, int numCnts, uint16_t *usrCnt)
{
	int retval = 0;
    int i, index;
    
    for (i = 0; i < numCnts; i++)
    {
        index = usrCnt[i];
        
        if (index < num64bStats)
        {
            /* 64-bit counters */
	        if (expected->count64[index] != actual->count64[index])  {
		        System_printf ("%s: Usr Stat count64[%d] expected %u, found %u\n", fname, index, (uint32_t)expected->count64[index], (uint32_t)actual->count64[index]);
		        retval = 1;
	        }
        }
        else
        {
            /* 32-bit counters */
            index -= num64bStats;
	        if (expected->count32[index] != actual->count32[index])  {
		        System_printf ("%s: Usr Stat count32[%d] expected %u, found %u\n", fname, index, expected->count32[index], actual->count32[index]);
		        retval = 1;
	        }    
	    }
    }
    
  	System_flush ();
  	return (retval);
}
#ifdef NSS_GEN2  
/* Compare long packet information during EOAM mode on EOAM packets */
int testCommonComparePktInfoEoamMode (char *tfName, pasahoLongInfo_t *expected, pasahoLongInfo_t *actual, int check)
{
	int retval = 0;

    Bool fCustom = PASAHO_LINFO_READ_HDR_BITMASK(expected) & PASAHO_HDR_BITMASK_CUSTOM;

  /* For next fail packets, there is no meaning to check the EOAM specific information */
  if (check) {
    
    if (PASAHO_LINFO_READ_EOAM_PKT_MEG_LEVEL(expected) != PASAHO_LINFO_READ_EOAM_PKT_MEG_LEVEL(actual))  {
  		System_printf ("%s (%s:%d): expected MEG level = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_EOAM_PKT_MEG_LEVEL(expected), PASAHO_LINFO_READ_EOAM_PKT_MEG_LEVEL(actual));
  		retval = 1;    
    }

    if (PASAHO_LINFO_READ_EOAM_PKT_OPCODE (expected) != PASAHO_LINFO_READ_EOAM_PKT_OPCODE(actual))  {
  		System_printf ("%s (%s:%d): expected Opcode = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_EOAM_PKT_OPCODE(expected), PASAHO_LINFO_READ_EOAM_PKT_OPCODE(actual));
  		retval = 1;    
    } 

    if (PASAHO_LINFO_READ_EOAM_TF_MATCH_CNT(expected) != PASAHO_LINFO_READ_EOAM_TF_MATCH_CNT(actual))  {
      System_printf ("%s (%s:%d): expected target flow match count = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_EOAM_TF_MATCH_CNT(expected), PASAHO_LINFO_READ_EOAM_TF_MATCH_CNT(actual));
      retval = 1;    
    } 
  }
	
	if (PASAHO_LINFO_READ_START_OFFSET(expected) != PASAHO_LINFO_READ_START_OFFSET(actual))  {
		System_printf ("%s (%s:%d): expected start offset = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_START_OFFSET(expected), PASAHO_LINFO_READ_START_OFFSET(actual));
		retval = 1;
	}
	
	if ((PASAHO_LINFO_READ_END_OFFSET(expected)) &&
        (PASAHO_LINFO_READ_END_OFFSET(expected) != PASAHO_LINFO_READ_END_OFFSET(actual)))  {
		System_printf ("%s (%s:%d): expected end offset = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_END_OFFSET(expected), PASAHO_LINFO_READ_END_OFFSET(actual));
		retval = 1;
	}
	
	if (!fCustom && (PASAHO_LINFO_READ_EIDX(expected) != PASAHO_LINFO_READ_EIDX(actual)))  {
		System_printf ("%s (%s:%d): expected error index = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_EIDX(expected), PASAHO_LINFO_READ_EIDX(actual));
		retval = 1;
	}
	
	if (PASAHO_LINFO_READ_PMATCH(expected) != PASAHO_LINFO_READ_PMATCH(actual))  {
		System_printf ("%s (%s:%d): expected pmatch = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_PMATCH(expected), PASAHO_LINFO_READ_PMATCH(actual));
		retval = 1;
	}
	
	if (PASAHO_LINFO_READ_HDR_BITMASK(expected) != PASAHO_LINFO_READ_HDR_BITMASK(actual))  {
		System_printf ("%s (%s:%d): expected header bitmask = 0x%04x, found 0x%04x\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_HDR_BITMASK(expected), PASAHO_LINFO_READ_HDR_BITMASK(actual));
		retval = 1;
	}
    
	if ((PASAHO_LINFO_READ_HDR_BITMASK2(expected)) &&
        (PASAHO_LINFO_READ_HDR_BITMASK2(expected) != PASAHO_LINFO_READ_HDR_BITMASK2(actual))) {
		System_printf ("%s (%s:%d): expected header bitmask2 = 0x%02x, found 0x%02x\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_HDR_BITMASK2(expected), PASAHO_LINFO_READ_HDR_BITMASK2(actual));
		retval = 1;
	}
	
	if (PASAHO_LINFO_READ_VLAN_COUNT(expected) != PASAHO_LINFO_READ_VLAN_COUNT(actual))  {
		System_printf ("%s (%s:%d): expected vlan count = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_VLAN_COUNT(expected), PASAHO_LINFO_READ_VLAN_COUNT(actual));
		retval = 1;
	}

	System_flush ();
	
#ifdef __LINUX_USER_SPACE
    return (0);
#else	
	return (retval);
#endif
}
#endif

/* Compare long packet information */
int testCommonComparePktInfo (char *tfName, pasahoLongInfo_t *expected, pasahoLongInfo_t *actual)
{
	int retval = 0;

    Bool fCustom = PASAHO_LINFO_READ_HDR_BITMASK(expected) & PASAHO_HDR_BITMASK_CUSTOM;
	
	if (PASAHO_LINFO_READ_START_OFFSET(expected) != PASAHO_LINFO_READ_START_OFFSET(actual))  {
		System_printf ("%s (%s:%d): expected start offset = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_START_OFFSET(expected), PASAHO_LINFO_READ_START_OFFSET(actual));
		retval = 1;
	}
	
	if ((PASAHO_LINFO_READ_END_OFFSET(expected)) &&
        (PASAHO_LINFO_READ_END_OFFSET(expected) != PASAHO_LINFO_READ_END_OFFSET(actual)))  {
		System_printf ("%s (%s:%d): expected end offset = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_END_OFFSET(expected), PASAHO_LINFO_READ_END_OFFSET(actual));
		retval = 1;
	}
	
	if (!fCustom && (PASAHO_LINFO_READ_EIDX(expected) != PASAHO_LINFO_READ_EIDX(actual)))  {
		System_printf ("%s (%s:%d): expected error index = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_EIDX(expected), PASAHO_LINFO_READ_EIDX(actual));
		retval = 1;
	}
	
	if (PASAHO_LINFO_READ_PMATCH(expected) != PASAHO_LINFO_READ_PMATCH(actual))  {
		System_printf ("%s (%s:%d): expected pmatch = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_PMATCH(expected), PASAHO_LINFO_READ_PMATCH(actual));
		retval = 1;
	}
	
	if (PASAHO_LINFO_READ_HDR_BITMASK(expected) != PASAHO_LINFO_READ_HDR_BITMASK(actual))  {
		System_printf ("%s (%s:%d): expected header bitmask = 0x%04x, found 0x%04x\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_HDR_BITMASK(expected), PASAHO_LINFO_READ_HDR_BITMASK(actual));
		retval = 1;
	}
    
	if ((PASAHO_LINFO_READ_HDR_BITMASK2(expected)) &&
        (PASAHO_LINFO_READ_HDR_BITMASK2(expected) != PASAHO_LINFO_READ_HDR_BITMASK2(actual))) {
		System_printf ("%s (%s:%d): expected header bitmask2 = 0x%02x, found 0x%02x\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_HDR_BITMASK2(expected), PASAHO_LINFO_READ_HDR_BITMASK2(actual));
		retval = 1;
	}
	
	if (PASAHO_LINFO_READ_VLAN_COUNT(expected) != PASAHO_LINFO_READ_VLAN_COUNT(actual))  {
		System_printf ("%s (%s:%d): expected vlan count = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_VLAN_COUNT(expected), PASAHO_LINFO_READ_VLAN_COUNT(actual));
		retval = 1;
	}
	
	if (PASAHO_LINFO_READ_IP_COUNT(expected) != PASAHO_LINFO_READ_IP_COUNT(actual))  {
		System_printf ("%s (%s:%d): expected IP count = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_IP_COUNT(expected), PASAHO_LINFO_READ_IP_COUNT(actual));
		retval = 1;
	}
	
	if (PASAHO_LINFO_READ_GRE_COUNT(expected) != PASAHO_LINFO_READ_GRE_COUNT(actual))  {
		System_printf ("%s (%s:%d): expected GRE count = %d, found %d\n", tfName, __FILE__, __LINE__, PASAHO_LINFO_READ_GRE_COUNT(expected), PASAHO_LINFO_READ_GRE_COUNT(actual));
		retval = 1;
	}
	
	System_flush ();
	
#ifdef __LINUX_USER_SPACE
    return (0);
#else	
	return (retval);
#endif
}

/* Increment expected stats based on an input bit map */
void testCommonIncStats (paStatsBmap_t *map,  paSysStats_t *stats)	
{
	int i;
	
	/* There are always three maps in the map list because stats can be incremented up to 3 times */
	for (i = 0; i < 3; i++)  {
		
		if (map[i] & (1 << TF_STATS_BM_C1_NUM_PACKETS))
			stats->classify1.nPackets += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_TABLE_MATCH))
			stats->classify1.nTableMatch += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_NO_TABLE_MATCH))
			stats->classify1.nNoTableMatch += 1;
            
		if (map[i] & (1 << TF_STATS_BM_C1_IP_FRAG))
			stats->classify1.nIpFrag += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_TX_IP_FRAG))
			stats->classify1.nTxIpFrag += 1;
            
		if (map[i] & (1 << TF_STATS_BM_C1_VLAN_OVERFLOW))
			stats->classify1.nVlanDepthOverflow += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_NUM_MPLS))
			stats->classify1.nMplsPackets += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_PARSE_FAIL))
			stats->classify1.nParseFail += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_SILENT_DISCARD))
			stats->classify1.nSilentDiscard += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_NUM_IPV4))
			stats->classify1.nIpv4Packets += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_NUM_IPV6))
			stats->classify1.nIpv6Packets += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C1_NUM_CUSTOM))
			stats->classify1.nCustomPackets += 1;
            
		if (map[i] & (1 << TF_STATS_BM_C1_NUM_SRIO))
			stats->classify1.nSrioPackets += 1;
            
		if (map[i] & (1 << TF_STATS_BM_C2_NUM_PACKETS))
			stats->classify2.nPackets += 1;
            
		if (map[i] & (1 << TF_STATS_BM_C2_NUM_UDP))
			stats->classify2.nUdp += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C2_NUM_TCP))
			stats->classify2.nTcp += 1;
			
		if (map[i] & (1 << TF_STATS_BM_C2_NUM_CUSTOM))
			stats->classify2.nCustom += 1;
			
	}
	
}
			

/* Reset a buffer descriptor with linked buffer and place on the correct free buffer with 
 * descriptor queue */
int testCommonRecycleLBDesc (tFramework_t *tf, Cppi_HostDesc *hd)
{
    Qmss_Queue q;
    Cppi_HostDesc *nextHdr;
    
    do
    {
	    hd->buffLen = hd->origBufferLen;
	    hd->buffPtr = hd->origBuffPtr;
        nextHdr = (Cppi_HostDesc *)hd->nextBDPtr;
        hd->nextBDPtr = 0;
        
        Cppi_setReturnPolicy (Cppi_DescType_HOST, (Cppi_Desc *) hd, Cppi_ReturnPolicy_RETURN_BUFFER);
        q.qMgr = 0;

	    if (hd->buffLen == TF_LINKED_BUF_Q1_BUF_SIZE)  {
            q.qNum = TF_LINKED_BUF_Q1;
            Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
		    Qmss_queuePush (tf->QLinkedBuf1, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);

	    }  else if (hd->buffLen == TF_LINKED_BUF_Q2_BUF_SIZE)  {
            q.qNum = TF_LINKED_BUF_Q2;
            Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
		    Qmss_queuePush (tf->QLinkedBuf2, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL); 

	    }  else if (hd->buffLen == TF_LINKED_BUF_Q3_BUF_SIZE)  {
            q.qNum = TF_LINKED_BUF_Q3;
            Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
		    Qmss_queuePush (tf->QLinkedBuf3, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);

	    }  else
		    return (-1);
    
    } while ((hd = nextHdr));
        
		
	return (0);
}

/* Provide an add srio configuration to the PA sub-system */
Cppi_HostDesc *testCommonAddSrio (tFramework_t *tf, int index, paSrioInfo_t *srioInfo, uint16_t nextHdr, uint16_t nextHdrOffset,
                                 paRouteInfo_t *matchRoute, paRouteInfo_t *nfailRoute, paHandleL2L3_t *l2handle, int32_t Qrecycle, int32_t QCmdMem, 
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_addSrio (tf->passHandle,
                         index,
					     srioInfo,
                         nextHdr,
                         nextHdrOffset,
					     matchRoute,
					     nfailRoute,
					     l2handle,
					     (paCmd_t) hd->buffPtr,
					     cmdSize,
					     repInfo,
					     cmdDest);
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}


/* Provide an add mac configuration to the PA sub-system */
Cppi_HostDesc *testCommonAddMac (tFramework_t *tf, int index, paEthInfo_t *ethInfo, paRouteInfo_t *matchRoute, paRouteInfo_t *nfailRoute, 
 	                       		 paHandleL2L3_t *l2handle, int32_t Qrecycle, int32_t QCmdMem, 
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_addMac (tf->passHandle,
                        index,
					    ethInfo,
					    matchRoute,
					    nfailRoute,
					    l2handle,
					    (paCmd_t) hd->buffPtr,
					    cmdSize,
					    repInfo,
					    cmdDest);
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}

/********************************************************************
 * FUNCTION PURPOSE: Determine if a uint8_t array is identically 0
 ********************************************************************
 * DESCRIPTION: Returns TRUE if every element in a uint8_t is zero
 ********************************************************************/
static uint16_t util_uint8_zero (uint8_t *v, int n)
{
  int i;

  for (i = 0; i < n; i++)
    if (v[i] != 0)
      return (FALSE);

  return (TRUE);

}

static void test_convert_ipInfo(paIpInfo_t *pIpInfo, paIpInfo2_t *pIpInfo2)
{
    memset(pIpInfo2, 0, sizeof(paIpInfo2_t));
    
    if (pIpInfo->ipType == pa_IPV4)
    {
        if (!util_uint8_zero(&pIpInfo->src.ipv4[0], 4))
        {
            memcpy(&pIpInfo2->src.ipv4[0], &pIpInfo->src.ipv4[0], 4);
            pIpInfo2->validBitMap |= pa_IP_INFO_VALID_SRC;
        }
        
        if (!util_uint8_zero(&pIpInfo->dst.ipv4[0], 4))
        {
            memcpy(&pIpInfo2->dst.ipv4[0], &pIpInfo->dst.ipv4[0], 4);
            pIpInfo2->validBitMap |= pa_IP_INFO_VALID_DST;
        }
    }
    else
    {
    
        if (!util_uint8_zero(&pIpInfo->src.ipv6[0], 16))
        {
            memcpy(&pIpInfo2->src.ipv6[0], &pIpInfo->src.ipv6[0], 16);
            pIpInfo2->validBitMap |= pa_IP_INFO_VALID_SRC;
        }
        
        if (!util_uint8_zero(&pIpInfo->dst.ipv6[0], 16))
        {
            memcpy(&pIpInfo2->dst.ipv6[0], &pIpInfo->dst.ipv6[0], 16);
            pIpInfo2->validBitMap |= pa_IP_INFO_VALID_DST;
        }
    }
    
    if (pIpInfo2->validBitMap & (pa_IP_INFO_VALID_SRC | pa_IP_INFO_VALID_DST))
    {
        pIpInfo2->ipType = pIpInfo->ipType;
    }
    
    if (pIpInfo->spi)
    {
        pIpInfo2->spi = pIpInfo->spi;
        pIpInfo2->validBitMap |= pa_IP_INFO_VALID_SPI;
    }
    
    if (pIpInfo->flow)
    {
        pIpInfo2->flow = pIpInfo->flow;
        pIpInfo2->validBitMap |= pa_IP_INFO_VALID_FLOW;
    }
    
    if (pIpInfo->greProto)
    {
        pIpInfo2->greProto = pIpInfo->greProto;
        pIpInfo2->validBitMap |= pa_IP_INFO_VALID_GREPROTO;
    }
    
    if (pIpInfo->proto)
    {
        pIpInfo2->proto = pIpInfo->proto;
        pIpInfo2->validBitMap |= pa_IP_INFO_VALID_PROTO;
    }
    
    if (pIpInfo->sctpPort)
    {
        pIpInfo2->sctpPort = pIpInfo->sctpPort;
        pIpInfo2->validBitMap |= pa_IP_INFO_VALID_SCTPPORT;
    }
    
    if (pIpInfo->tosCare)
    {
        pIpInfo2->tos = pIpInfo->tos;
        pIpInfo2->validBitMap |= pa_IP_INFO_VALID_TOS;
    }
    
}

/*************************************************************************
 * FUNCTION PURPOSE: Convert ethInfo 
 ************************************************************************* 
 * DESCRIPTION: Convert old style of ETH information to new data structure
 *************************************************************************/
static void test_convert_ethInfo(paEthInfo_t *pEthInfo, paEthInfo2_t *pEthInfo2)
{
    memset(pEthInfo2, 0, sizeof(paEthInfo2_t));
    
    if (!util_uint8_zero(&pEthInfo->src[0], pa_MAC_ADDR_SIZE))
    {
        memcpy(&pEthInfo2->src[0], &pEthInfo->src[0], pa_MAC_ADDR_SIZE);
        pEthInfo2->validBitMap |= pa_ETH_INFO_VALID_SRC;
    }
    
    if (!util_uint8_zero(&pEthInfo->dst[0], pa_MAC_ADDR_SIZE))
    {
        memcpy(&pEthInfo2->dst[0], &pEthInfo->dst[0], pa_MAC_ADDR_SIZE);
        pEthInfo2->validBitMap |= pa_ETH_INFO_VALID_DST;
    }
    
    if (pEthInfo->vlan)
    {
        pEthInfo2->vlan = pEthInfo->vlan;
        pEthInfo2->validBitMap |= pa_ETH_INFO_VALID_VLAN;
    }
    
    if (pEthInfo->ethertype)
    {
        pEthInfo2->ethertype = pEthInfo->ethertype;
        pEthInfo2->validBitMap |= pa_ETH_INFO_VALID_ETHERTYPE;
    }
    
    if (pEthInfo->mplsTag)
    {
        pEthInfo2->mplsTag = pEthInfo->mplsTag;
        pEthInfo2->validBitMap |= pa_ETH_INFO_VALID_MPLSTAG;
    }
    
    if (pEthInfo->inport)
    {
        pEthInfo2->inport = pEthInfo->inport;
        pEthInfo2->validBitMap |= pa_ETH_INFO_VALID_INPORT;
    }
}


/* Provide an add mac configuration to the PA sub-system */
Cppi_HostDesc *testCommonAddMac2 (tFramework_t *tf, int index, paEthInfo_t *ethInfo, paRouteInfo2_t *matchRoute, paRouteInfo2_t *nfailRoute, 
                                  paHandleL2L3_t nextLink, 
 	                       		  paHandleL2L3_t *l2handle, int32_t Qrecycle, int32_t QCmdMem, 
 	                        	  paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, uint32_t ctrlBitMap, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
    
    /* Prepare parameters for Pa_addIp2 */ 
    paParamDesc params;
    paEthInfo2_t ethInfo2;

    /* Initialize backwards compatible structs */ 
    memset(&params, 0, sizeof(paParamDesc));
    test_convert_ethInfo(ethInfo, &ethInfo2); 

    params.lutInst = pa_LUT_INST_NOT_SPECIFIED;
    params.index = index;
    params.prevLink = NULL;
    params.nextLink = nextLink;
    params.routeInfo = matchRoute;
    params.nextRtFail = nfailRoute;
  
    if(index != pa_LUT1_INDEX_NOT_SPECIFIED)
        params.validBitMap |= pa_PARAM_VALID_INDEX;
    
    if(nextLink)
        params.validBitMap |= pa_PARAM_VALID_NEXTLINK;

    if (ctrlBitMap)
        params.validBitMap |= pa_PARAM_VALID_CTRLBITMAP;

    /* Replace command is provided */
    if (ctrlBitMap == pa_PARAM_CTRL_REPLACE) {
      params.ctrlBitMap |= pa_PARAM_CTRL_REPLACE;
    }
    
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
    
	
	*paret = Pa_addMac2 (tf->passHandle,
					     &ethInfo2,
                         &params,
					     l2handle,
					     (paCmd_t) hd->buffPtr,
					     cmdSize,
					     repInfo,
					     cmdDest);
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}


Cppi_HostDesc *testCommonAddIp2 (tFramework_t *tf, 
                                int lutInst,int index, 
                                paIpInfo_t *ipInfo, 
                                paRouteInfo_t *matchRoute, 
                                paRouteInfo_t *nfailRoute, 
 	                       		    paHandleL2L3_t *l3handle, 
                                paHandleL2L3_t linkedL2Handle, 
                                int32_t Qrecycle, 
                                int32_t QCmdMem, 
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_addIp (tf->passHandle,
                       lutInst,
                       index,
					   ipInfo,
					   linkedL2Handle,
					   matchRoute,
					   nfailRoute,
					   l3handle,
					   (paCmd_t) hd->buffPtr,
					   cmdSize,
					   repInfo,
					   cmdDest);
					   
    /* Restore the descriptor and return it on PA failure */
	if ((*paret != pa_OK) && (*paret != pa_DUP_ENTRY)) {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}


Cppi_HostDesc *testCommonAddIp3 (tFramework_t *tf, 
                                int lutInst,int index, 
                                paIpInfo_t *ipInfo, 
                                paRouteInfo2_t *matchRoute, 
                                paRouteInfo2_t *nfailRoute, 
                                paHandleL2L3_t *l3handle, 
                                paHandleL2L3_t prevLink, 
                                paHandleL2L3_t nextLink, 
                                int32_t Qrecycle, 
                                int32_t QCmdMem, 
                                paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, uint32_t ctrlBitMap, paReturn_t *paret)
{
  Cppi_HostDesc *hd;
  Qmss_Queue     q;
  uint32_t       psCmd;
  
  /* Pop a descriptor with a linked buffer to create the command */
  hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
  if (hd == NULL)
    return (NULL);
    
    
  /* Prepare parameters for Pa_addIp2 */ 
  paParamDesc params;
  paIpInfo2_t ipInfo2;

  /* Initialize backwards compatible structs */ 
  memset(&params, 0, sizeof(paParamDesc));
  test_convert_ipInfo(ipInfo, &ipInfo2); 

  params.lutInst = lutInst;
  params.index = index;
  params.prevLink = prevLink;
  params.nextLink = nextLink;
  params.routeInfo = matchRoute;
  params.nextRtFail = nfailRoute;
  
  if(lutInst != pa_LUT_INST_NOT_SPECIFIED)
    params.validBitMap |= pa_PARAM_VALID_LUTINST;
    
  if(index != pa_LUT1_INDEX_NOT_SPECIFIED)
    params.validBitMap |= pa_PARAM_VALID_INDEX;
    
  if(prevLink)
    params.validBitMap |= pa_PARAM_VALID_PREVLINK;
    
  if(nextLink)
    params.validBitMap |= pa_PARAM_VALID_NEXTLINK;
    
  if (ctrlBitMap)
    params.validBitMap |= pa_PARAM_VALID_CTRLBITMAP;

  /* Replace command is provided */
  if (ctrlBitMap == pa_PARAM_CTRL_REPLACE) {
    params.ctrlBitMap |= pa_PARAM_CTRL_REPLACE;
  }    

  *cmdSize = hd->origBufferLen;
  
  *paret = Pa_addIp2 (tf->passHandle,
                       &ipInfo2,
                       &params,
                       l3handle,
                       (paCmd_t) hd->buffPtr,
                       cmdSize,
                       repInfo,
                       cmdDest);
             
    /* Restore the descriptor and return it on PA failure */
  if ((*paret != pa_OK) && (*paret != pa_DUP_ENTRY)) {
    hd->buffLen = hd->origBufferLen;
    Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
    return (NULL);
  }
  
#ifdef PA_SIM_BUG_4BYTES
    *cmdSize = (*cmdSize+3)& ~3;
#endif

  /* Setup the return for the descriptor */
    q.qMgr = 0;
    q.qNum = Qrecycle;
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
    
    /* Mark the packet as a configuration packet */
    psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
    
    hd->buffLen = *cmdSize;
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
    
    return (hd);
}

Cppi_HostDesc *testCommonAddIp (tFramework_t *tf, int index, paIpInfo_t *ipInfo, paRouteInfo_t *matchRoute, paRouteInfo_t *nfailRoute, 
 	                       	    paHandleL2L3_t *l3handle, paHandleL2L3_t linkedL2Handle, int32_t Qrecycle, int32_t QCmdMem, 
 	                            paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{

    return(testCommonAddIp2(tf, pa_LUT_INST_NOT_SPECIFIED, index, ipInfo, matchRoute, nfailRoute, l3handle, linkedL2Handle,
                            Qrecycle, QCmdMem, repInfo, cmdDest, cmdSize, paret));   
}

Cppi_HostDesc *testCommonAddPort (tFramework_t *tf, int portSize, uint32_t destPort, paRouteInfo_t *matchRoute, paHandleL4_t *l4handle, paHandleL2L3_t *l3handle,
                                  int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;	                          
                                
    *paret =   Pa_addPort ( tf->passHandle, (uint16_t)portSize, destPort, *l3handle, FALSE, pa_PARAMS_NOT_SPECIFIED, matchRoute, *l4handle, (paCmd_t) hd->buffPtr, cmdSize, repInfo, cmdDest);
    
                    
     /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}  

Cppi_HostDesc *testCommonAddPort2 (tFramework_t *tf, int portSize, uint32_t destPort, uint16_t fReplace, uint16_t divertQ, paRouteInfo_t *matchRoute, paHandleL4_t *l4handle, paHandleL2L3_t *l3handle,
                                   int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;	                          
                                
    *paret =   Pa_addPort ( tf->passHandle, (uint16_t)portSize, destPort, *l3handle, fReplace, divertQ, matchRoute, *l4handle, (paCmd_t) hd->buffPtr, cmdSize, repInfo, cmdDest);
                    
     /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}  

#ifdef NSS_GEN2
Cppi_HostDesc *testCommonAddPort3 (tFramework_t *tf, int portSize, uint32_t destPort, uint16_t fReplace, uint16_t divertQ, paRouteInfo2_t *matchRoute, paHandleL4_t *l4handle, paHandleL2L3_t *l3handle,
                                   int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
    paLut2ParamDesc params;
    
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
    
    /* prepare the parameters for addPort2 */
    memset(&params, 0, sizeof(paLut2ParamDesc));
    
    if (fReplace)
    {
        params.validBitMap |= pa_LUT2_PARAM_VALID_CTRL_BITMAP;
        params.ctrlFlags |= pa_LUT2_INFO_CONTROL_FLAG_REPLACE;
    }
    
    if (divertQ != pa_PARAMS_NOT_SPECIFIED)
    {
        params.validBitMap |= pa_LUT2_PARAM_VALID_DIVERTQ;
        params.divertQ = divertQ;
    }
                                
    *paret =   Pa_addPort2 ( tf->passHandle, (uint16_t)portSize, destPort, *l3handle, &params, matchRoute, *l4handle, (paCmd_t) hd->buffPtr, cmdSize, repInfo, cmdDest);
                    
     /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}  

#endif


Cppi_HostDesc *testCommonAddCustomL3 (tFramework_t *tf, uint16_t customIndex, int lutInst, uint8_t match[], paRouteInfo_t *matchRoute, paRouteInfo_t *nfailRoute,
                                      paHandleL2L3_t *l3handle, paHandleL2L3_t linkedL2Handle,
									  int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_addCustomLUT1 (tf->passHandle, customIndex, lutInst, pa_LUT1_INDEX_NOT_SPECIFIED, match, linkedL2Handle, matchRoute, nfailRoute, 
                               l3handle, (paCmd_t) hd->buffPtr, cmdSize, repInfo, cmdDest);	     

     /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}

Cppi_HostDesc *testCommonAddCl3Config (tFramework_t *tf, uint16_t customIndex, uint16_t offset, uint16_t nextHdr, uint16_t nextHdrOffset, uint8_t masks[], 
									   int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;	     
		
	*paret = Pa_setCustomLUT1 (tf->passHandle, customIndex, offset, nextHdr, nextHdrOffset, masks, (paCmd_t) hd->buffPtr, cmdSize, repInfo, cmdDest);

    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}    
                         

Cppi_HostDesc *testCommonAddCustomL4 (tFramework_t *tf, uint16_t customIndex, uint8_t match[], paRouteInfo_t *matchRoute, paHandleL4_t l4handle, paHandleL2L3_t l3handle,
									  int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_addCustomLUT2 (tf->passHandle, customIndex,  match, l3handle, FALSE, pa_PARAMS_NOT_SPECIFIED, matchRoute, l4handle, (paCmd_t) hd->buffPtr, cmdSize, repInfo, cmdDest);	     
			
							

     /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}

Cppi_HostDesc *testCommonAddCl4Config (tFramework_t *tf, Bool handleLink, uint16_t customIndex, int16_t customHdrSize, uint16_t offsets[], uint8_t masks[], 
										int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;	     
		
	*paret = Pa_setCustomLUT2 (tf->passHandle, customIndex, handleLink, customHdrSize, offsets, masks, 0, (paCmd_t) hd->buffPtr, cmdSize, repInfo, cmdDest);

    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
} 

#ifdef NSS_GEN2
Cppi_HostDesc *testCommonAddEoam(tFramework_t     *tf, 
                                 paEthInfo2_t     *ethInfo, 
                                 paEoamFlowInfo_t *eoamInfo,  
                                 paHandleEoam_t   *eoamHandle,
 	                       		     int32_t           Qrecycle, 
 	                       		     int32_t           QCmdMem, 
 	                       		     paCmdReply_t     *repInfo, 
 	                       		     int              *cmdDest, 
 	                       		     uint16_t         *cmdSize, 
 	                       		     paReturn_t       *paret)
{
    Cppi_HostDesc *hd;
    Qmss_Queue     q;
    uint32_t       psCmd;
    
    /* Pop a descriptor with a linked buffer to create the command */
    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
    if (hd == NULL)
      return (NULL);
      
    *cmdSize = hd->origBufferLen;
    
    *paret = Pa_addEoamFlow (tf->passHandle,
                             ethInfo,
                             eoamInfo,
                             eoamHandle,
                   (paCmd_t) hd->buffPtr,
                             cmdSize,
                             repInfo,
                             cmdDest);
               
      /* Restore the descriptor and return it on PA failure */
    if ((*paret != pa_OK)) {
      hd->buffLen = hd->origBufferLen;
      Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
      return (NULL);
    }
    
#ifdef PA_SIM_BUG_4BYTES
      *cmdSize = (*cmdSize+3)& ~3;
#endif
  
    /* Setup the return for the descriptor */
      q.qMgr = 0;
      q.qNum = Qrecycle;
      Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
      
      /* Mark the packet as a configuration packet */
      psCmd = PASAHO_PACFG_CMD;
      Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
      
      hd->buffLen = *cmdSize;
      Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
      
      return (hd);
}

Cppi_HostDesc *testCommonAddAcl (tFramework_t *tf, int aclInst,int action, paAclInfo_t *aclInfo,  
 	                       		 paHandleAcl_t *aclhandle, paHandleL2L3_t linkedHandle, paHandleAcl_t nextHandle, 
                                 int32_t Qrecycle, int32_t QCmdMem, 
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
    Cppi_HostDesc *hd;
    Qmss_Queue     q;
    uint32_t       psCmd;
    uint32_t       timeToNextCall = 0;

    /* Pop a descriptor with a linked buffer to create the command */
    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
    if (hd == NULL)
      return (NULL);

    *cmdSize = hd->origBufferLen;

    *paret = Pa_addAcl (tf->passHandle,
                         aclInst,
                         action,
                         aclInfo,
                         linkedHandle,
                         nextHandle,
                         aclhandle,
                        (paCmd_t) hd->buffPtr,
                         cmdSize,
                         repInfo,
                         cmdDest,
                         &timeToNextCall);

    /* Application can initiate task sleep OR switch task for anything else to come back to this 
     * again for the next try */
     if (*paret == pa_ACL_BUSY) {
     }

    /* Restore the descriptor and return it on PA failure */
	if ((*paret != pa_OK)) {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}              

Cppi_HostDesc *testCommonAddFc  (tFramework_t *tf, int index, paEfOpInfo_t *efInfo, paFcInfo_t *fcInfo,  
 	                       		 paHandleFc_t *fchandle, int32_t Qrecycle, int32_t QCmdMem, 
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{

	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_addFc (tf->passHandle,
                       index,
                       efInfo,
					   fcInfo,
					   fchandle,
					   (paCmd_t) hd->buffPtr,
					   cmdSize,
					   repInfo,
					   cmdDest);
					   
    /* Restore the descriptor and return it on PA failure */
	if ((*paret != pa_OK)) {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}               

#endif                  

/* Provide an Multi-route configuration to the PA sub-system */
Cppi_HostDesc *testCommonConfigMultiRoute (tFramework_t *tf, paMultiRouteModes_e mode, uint16_t index, uint16_t nRoute, paMultiRouteEntry_t *routeEntry, 
 	                       		           int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_configMultiRoute (tf->passHandle,
                                  mode,
                                  index,
					              nRoute,
					              routeEntry,
					              (paCmd_t) hd->buffPtr,
					              cmdSize,
					              repInfo,
					              cmdDest);
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
	/* indicate the use of the multi route use set in the test framework */
	tf->mrUseList[index].indication = TF_PA_RIDX_USED;
  	
  	return (hd);
}

/* Provide an Exception Route configuration to the PA sub-system */
Cppi_HostDesc *testCommonConfigExceptionRoute (tFramework_t *tf, int nRoute, int *routeTypes, paRouteInfo_t *eRoutes, 
 	                       		               int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret, int fEflow)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t	   psCmd;
	uint32_t       index; 
    PaExc_Log_t   *excUseList = (fEflow)?tf->efExcUseList:tf->excUseList;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
    #ifdef NSS_GEN2
    if (fEflow)
    {
	    *paret = Pa_configEflowExceptionRoute (tf->passHandle,
					                           nRoute,
                                               routeTypes,
					                           eRoutes,
					                           (paCmd_t) hd->buffPtr,
					                           cmdSize,
					                           repInfo,
					                           cmdDest);
    
    
    }
    else
    {
	    *paret = Pa_configExceptionRoute (tf->passHandle,
					                    nRoute,
                                        routeTypes,
					                    eRoutes,
					                    (paCmd_t) hd->buffPtr,
					                    cmdSize,
					                    repInfo,
					                    cmdDest);
    }               
    #else
    
	    *paret = Pa_configExceptionRoute (tf->passHandle,
					                    nRoute,
                                        routeTypes,
					                    eRoutes,
					                    (paCmd_t) hd->buffPtr,
					                    cmdSize,
					                    repInfo,
					                    cmdDest);
    
    #endif                    
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);

	/* indicate the use of the command set in the test framework */
	for (index = 0; index < nRoute; index++)
		excUseList[routeTypes[index]].indication = TF_PA_RIDX_USED;	
  	
  	return (hd);
}

/* Provide an Exception Route configuration to the PA sub-system */
Cppi_HostDesc *testCommonConfigExceptionRoute2 (tFramework_t *tf, int nRoute, int *routeTypes, paRouteInfo2_t *eRoutes, 
 	                       		                int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t	   psCmd;
	uint32_t       index; 
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_configExceptionRoute2 (tf->passHandle,
					                   nRoute,
                                       routeTypes,
					                   eRoutes,
					                   (paCmd_t) hd->buffPtr,
					                   cmdSize,
					                   repInfo,
					                   cmdDest);
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);

	/* indicate the use of the command set in the test framework */
	for (index = 0; index < nRoute; index++)
		tf->excUseList[routeTypes[index]].indication = TF_PA_RIDX_USED;	
  	
  	return (hd);
}

/* Provide an Default Route configuration to the PA sub-system */
Cppi_HostDesc *testCommonConfigDefaultRoute  (tFramework_t *tf, int numPorts, paDefRouteConfig_t *defRouteCfg,
 	                       		              int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
    paCtrlInfo_t        ctrl; 
    Cppi_HostDesc      *hd; 	

    /* Clear the local valirables */
    memset (&ctrl, 0, sizeof(paCtrlInfo_t));

    ctrl.code                             = pa_CONTROL_EMAC_PORT_CONFIG;
    ctrl.params.emacPortCfg.cfgType       = pa_EMAC_PORT_CFG_DEFAULT_ROUTE;
    ctrl.params.emacPortCfg.numEntries    = (uint8_t)numPorts;
    ctrl.params.emacPortCfg.u.defRouteCfg = defRouteCfg;
    
    hd = testCommonGlobalConfig(tf, &ctrl, Qrecycle, QCmdMem, repInfo, cmdDest, cmdSize, paret);
  	return (hd);
}

/* Provide an EQoS Mode configuration to the PA sub-system */
Cppi_HostDesc *testCommonConfigEQoSMode  (tFramework_t *tf, int numPorts, paEQosModeConfig_t *eQoSModeCfg,
 	                       		         int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
    paCtrlInfo_t        ctrl; 
    Cppi_HostDesc      *hd; 	

    /* Clear the local valirables */
    memset (&ctrl, 0, sizeof(paCtrlInfo_t));

    ctrl.code                             = pa_CONTROL_EMAC_PORT_CONFIG;
    ctrl.params.emacPortCfg.cfgType       = pa_EMAC_PORT_CFG_EQoS_MODE;
    ctrl.params.emacPortCfg.numEntries    = (uint8_t)numPorts;
    ctrl.params.emacPortCfg.u.eQoSModeCfg = eQoSModeCfg;
    
    hd = testCommonGlobalConfig(tf, &ctrl, Qrecycle, QCmdMem, repInfo, cmdDest, cmdSize, paret);
  	return (hd);
}

/* Provide a Cmd Set configuration to the PA sub-system */
Cppi_HostDesc *testCommonConfigCmdSet (tFramework_t *tf, uint16_t index, int nCmd, paCmdInfo_t *cmdInfo, 
 	                       		       int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_configCmdSet (tf->passHandle,
                              index,
					          nCmd,
                              cmdInfo,
					          (paCmd_t) hd->buffPtr,
					          cmdSize,
					          repInfo,
					          cmdDest);
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);

	/* indicate the use of the command set in the test framework */
	tf->cmdSetUseList[index].indication = TF_PA_RIDX_USED;
  	
  	return (hd);
}

/* Look for command replies from PA - simplified for test framework clean up*/
void testCommonCmdRep (tFramework_t *tf, int *cmdReply, int32_t QCmdResp, int32_t QRet, int expReplyId)
{
	Cppi_HostDesc    *hd;
	uint32_t         *swInfo0;
	uint32_t 		 swInfoCmd;

	while (Qmss_getQueueEntryCount (QCmdResp) > 0)  {
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop ((tf->QCommonCmdRep))) & ~15);
		if (Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **) &swInfo0) == CPPI_EPIB_NOT_PRESENT)  {
			System_printf ("testCommonCmdRep: (%s:%d): Found descriptor in PA command reply queue without EIPB present, failing\n", __FILE__, __LINE__);
			testCommonRecycleLBDesc (tf, hd);  /* Ignore return code */
		}
		swInfoCmd = (*swInfo0 & 0xffff0000u);

		/* Verify expected value in swinfo0 16 msbs */
		if (swInfoCmd == expReplyId) {
			/* Recycle the descriptor and buffer */
			if (testCommonRecycleLBDesc (tf, hd))  {
				System_printf ("testCommonCmdRep: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", __FILE__, __LINE__);
			}
			*cmdReply = TRUE;
		}
	}

	while (Qmss_getQueueEntryCount (QRet) > 0)  {

		/* Recycle the command descriptor/buffer */
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QDefRet)) & ~15);
		if (testCommonRecycleLBDesc (tf, hd))  {
			System_printf ("testCommonCmdRep: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n",__FILE__, __LINE__);
		}
	}
}

int testCommonSetGblConfig(	paCtrlInfo_t ctrlInfo, uint32_t replyId)
{
	Cppi_HostDesc  *hd;
 	paCmdReply_t   cmdReply = {pa_DEST_HOST,			    /* Dest */
 							   0,	                        /* Reply ID (returned in swinfo0) */
 							   0,						    /* Queue */
 							   0 };						    /* Flow ID */
                               
	int 		   cmdDest, i;
	uint16_t 	   cmdSize;
	paReturn_t     paret;
	int            fCmdReply;
	tFramework_t   *tf = &tFramework;

	/* Issue the command set command */
    cmdReply.replyId  = replyId;
	cmdReply.queue    = tf->QCommonCmdRep;
	cmdReply.flowId   = tf->tfFlowNum[0];
 	hd = testCommonGlobalConfig (tf, &ctrlInfo,
 			                     tf->QDefRet, tf->QLinkedBuf3,
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);

    if ( (hd == NULL) ||
         (paret != pa_OK) ) {
            System_printf ("testCommonSetGblConfig: (%s:%d): Failure in Setting to Default Global Config\n", __FILE__, __LINE__);
            return (-1);
    }

    if (pdsp_halt)
    	mdebugHaltPdsp(cmdDest);

    /* Send command */
	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

    /* All the packets should have been acked */
	fCmdReply = FALSE;
    for (i = 0; i < 100; i++)  {
    
		  utilCycleDelay (1000);
          
	      testCommonCmdRep (tf, &fCmdReply, tf->QCommonCmdRep, tf->QDefRet, cmdReply.replyId);

	      if (fCmdReply == TRUE)
	         break;
    }

    if (i == 100)  {
	     System_printf ("(%s:%d): Set Global Config commands was not acked\n", __FILE__, __LINE__);
	     return (-1);
    }

	return (0);

}

int testCommonSetDefaultGlobalConfig (void)
{
	paCtrlInfo_t   ctrlInfo;

	/* Set the default values, taken from pa.h */
	paProtocolLimit_t     paDefProtocolLimit =
	        {
	        	pa_PROTOCOL_LIMIT_NUM_VLANS_DEF,      /* Number of VLANs */
	        	pa_PROTOCOL_LIMIT_NUM_IP_DEF,         /* Number of IPs */
	        	pa_PROTOCOL_LIMIT_NUM_GRE_DEF         /* Number of GREs */
	        };

	paCmdSetConfig_t      paDefCmdSetCfg =
	        {
	            64      /* Number of command sets */
	        };

	paUsrStatsConfig_t   paDefUsrStatsCfg =
	       {
	            (pa_USR_STATS_MAX_COUNTERS - 64), /* Number of user stats   */
	            64                                /* Number of 64-bit user stats */
	       };

	paQueueDivertConfig_t paDefQueueDivertCfg =
	        {
	            0,      /* Monitoring Queue */
	            0       /* flow Id */
	        };

	paPacketControlConfig_t  paDefPktCtrlCfg =
	        {
	            pa_PKT_CTRL_HDR_VERIFY_IP,  /* ctrlBitMap */
	            64,  /* rxPaddingErrStatsIndex */
	            65   /* txPaddingStatsIndex */
	        };

	paQueueBounceConfig_t paDefQueueBounceCfg =
	        {
	            0,      /* Enable */
                0,      /* ddrQueueId */
                0,      /* msmcQueueId */
                0,      /* hwQueueBegin */
                0,      /* hwQueueEnd */
                {
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Command Return */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* EQoS mode */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* Capture Capture */
                    pa_QUEUE_BOUNCE_OP_NONE,    /* IP Reassembly-assisted packets */
                    pa_QUEUE_BOUNCE_OP_NONE     /* All traffics */
                }
	        };

	paIpReassmConfig_t       paDefReassmConfig =
   	        {
   	            0,  /* numTrafficFlow */
   	    	    0,  /* destFlowId */
   	    	    0   /* destQueue */
	        };

    pa802p1agDetConfig_t   pa802p1agDetCfg =
            {
                0,  /* ctrlBitMap */    
            };
            
	paIpsecNatTConfig_t  paDefNatTCfg =
	        {
	            0,    /* ctrlBitMap */
	            0     /* UDP port number */
	        };
            
            
    #ifdef NSS_GEN2        
    paAclConfig_t  paAclCfg =
            {
                pa_ACL_ACTION_PERMIT,       /* action */
                0,                          /* destFlowId */
                0                           /* destQueue */
            };
            
    paRaGroupConfig_t paRaGroupCfg =
        {
            0,                              /* ctrlBitMap */
            0,                              /* Flow Id */
            {                               /* TimeoutER */
                0,                          /* dest */
                0,                          /* flowId */
                0                           /* queue */
            },       
        
            {                               /* CritErrER */
                0,                          /* dest */
                0,                          /* flowId */
                0                           /* queue */
            },       
  
            {                               /* genErrER */
                0,                          /* dest */
                0,                          /* flowId */
                0                           /* queue */
            }       
        
        };       
    #endif    
            
	paSysConfig_t  paDefGlobalCfg;
    
    memset(&paDefGlobalCfg, 0, sizeof(paSysConfig_t));

	paDefGlobalCfg.pCmdSetConfig       = &paDefCmdSetCfg;
	paDefGlobalCfg.pInIpReassmConfig   = &paDefReassmConfig;
	paDefGlobalCfg.pOutIpReassmConfig  = &paDefReassmConfig;
	paDefGlobalCfg.pPktControl         = &paDefPktCtrlCfg;
	paDefGlobalCfg.pProtoLimit         = &paDefProtocolLimit;
	paDefGlobalCfg.pQueueDivertConfig  = &paDefQueueDivertCfg;
	paDefGlobalCfg.pUsrStatsConfig     = &paDefUsrStatsCfg;
	paDefGlobalCfg.pQueueBounceConfig  = &paDefQueueBounceCfg;
#ifdef NSS_GEN2
    paDefGlobalCfg.pOutAclConfig       = &paAclCfg;
    paDefGlobalCfg.pOutAclConfig->insertMode = pa_ACL_INSERT_RANDOM;
    paDefGlobalCfg.pInAclConfig        = &paAclCfg;
    paDefGlobalCfg.pInAclConfig->insertMode = pa_ACL_INSERT_RANDOM;    
    paDefGlobalCfg.pOutIpRaGroupConfig = &paRaGroupCfg;
    paDefGlobalCfg.pInIpRaGroupConfig  = &paRaGroupCfg;
#endif

	/* set System Global default configuration */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = paDefGlobalCfg;
	if (testCommonSetGblConfig(ctrlInfo, TF_COMMON_CMD_ID_DEFGBLCFG)) {
		return (-1);
	}
    
    /* Disable 802.1ag detector */
    ctrlInfo.code = pa_CONTROL_802_1ag_CONFIG;
    ctrlInfo.params.pa802p1agDetCfg = pa802p1agDetCfg;
	if (testCommonSetGblConfig(ctrlInfo, TF_COMMON_CMD_ID_DEF_802P1AG_CFG)) {
		return (-1);
	}

	/* set nat-t global default configuration */
    ctrlInfo.code = pa_CONTROL_IPSEC_NAT_T_CONFIG;
    ctrlInfo.params.ipsecNatTDetCfg = paDefNatTCfg;
	if (testCommonSetGblConfig(ctrlInfo, TF_COMMON_CMD_ID_DEF_NAT_T_CFG)) {
		return (-1);
	}
    
    /* GTPU configuration is no longer required */
    
    /* RA configuration: TBD */

	return (0);
}

/* Reset all the multi-route groups in PASS */
int testCommonMultiRouteRecover(void)
{
	Cppi_HostDesc  *hd;
 	paCmdReply_t   cmdReply = { pa_DEST_HOST,			            /* Dest */
 							    TF_COMMON_CMD_ID_MULTIROUTE_CFG,	/* Reply ID (returned in swinfo0) */
 							    0,						            /* Queue */
 							    0 }; 					            /* Flow ID */
                                
	int 		   cmdDest, j, i;
	uint16_t 	   cmdSize;
	paReturn_t     paret;
	int            fCmdReply;
	tFramework_t   *tf=&tFramework;

	cmdReply.queue = tf->QCommonCmdRep;
	cmdReply.flowId= tf->tfFlowNum[0];

	for (j = 0; j < TF_PA_NUM_MRENTRIES; j++) {

		/* Check if the multi route is used for that index */
	    if (tf->mrUseList[j].indication == TF_PA_RIDX_USED ) {
 		    hd = testCommonConfigMultiRoute (tf, pa_MULTI_ROUTE_MODE_RESET, j, 0,  
 	                                        NULL, tf->QDefRet, tf->QLinkedBuf3, 
 	                                        &cmdReply, &cmdDest, &cmdSize, &paret);			

            if ( (hd == NULL) ||
    	         (paret != pa_OK) ) {
	                System_printf ("testCommonConfigMultiRoute: (%s:%d): Failure in Multi Route Config command\n", __FILE__, __LINE__);
	   		        return (-1);
            }

            /* Send command */
     	    Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

     	    fCmdReply = FALSE;
   	        /* All the packets should have been acked */
   	        for (i = 0; i < 100; i++)  {
            
   			    utilCycleDelay (1000);
                
   		        testCommonCmdRep (tf, &fCmdReply, tf->QCommonCmdRep, tf->QDefRet, cmdReply.replyId);

   		        if (fCmdReply == TRUE)
   			        break;
   	        }

   	        if (i == 100)  {
   		        System_printf ("(%s:%d): Pa_configMultiRoute (%d) commands was not acked\n", __FILE__, __LINE__, j);
   		        return (-1);
   	        }
            
   	        /* Set the multi-route group index as not used for the next test case */
   	        tf->mrUseList[j].indication = TF_PA_RIDX_NOT_USED;
	    }
    }

	return (0);

}

/* Reset all the commandSets in PASS */
int testCommonCmdSetRecover(void)
{
	Cppi_HostDesc  *hd;
 	paCmdReply_t   cmdReply = {pa_DEST_HOST,			    /* Dest */
 							   TF_COMMON_CMD_ID_ADD_CMDSET,	/* Reply ID (returned in swinfo0) */
 							   0,						    /* Queue */
 							   0 };						    /* Flow ID */
	int 		   cmdDest, cmdSetIndex, i;
	uint16_t 	   cmdSize;
	paReturn_t     paret;
	int            fCmdReply;
	tFramework_t   *tf = &tFramework;

	cmdReply.queue = tf->QCommonCmdRep;
	cmdReply.flowId= tf->tfFlowNum[0];

	for (cmdSetIndex = 0; cmdSetIndex < TF_PA_NUM_CMDSETS; cmdSetIndex++) {

		/* Check if the command set is used for that index */
	    if (tf->cmdSetUseList[cmdSetIndex].indication == TF_PA_RIDX_USED ) {
	        hd = testCommonConfigCmdSet (tf, cmdSetIndex, 0, NULL,
	                                     tf->QDefRet, tf->QLinkedBuf3,
	                                     &cmdReply, &cmdDest, &cmdSize, &paret);

            if ( (hd == NULL) ||
    	         (paret != pa_OK) ) {
	                System_printf ("testCommonCmdSetRecover: (%s:%d): Failure in ConfigCmdSet command\n", __FILE__, __LINE__);
	   		        return (-1);
            }

            /* Send command */
     	    Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

     	    fCmdReply = FALSE;
   	        /* All the packets should have been acked */
   	        for (i = 0; i < 100; i++)  {
            
   			    utilCycleDelay (1000);
                
   		        testCommonCmdRep (tf, &fCmdReply, tf->QCommonCmdRep, tf->QDefRet, cmdReply.replyId);

   		        if (fCmdReply == TRUE)
   			        break;
   	        }

   	        if (i == 100)  {
   		        System_printf ("(%s:%d): Pa_configCmdSet (%d) commands was not acked\n", __FILE__, __LINE__, cmdSetIndex);
   		        return (-1);
   	        }
   	        /* Set the command set index as not used for the next test case */
   	        tf->cmdSetUseList[cmdSetIndex].indication = TF_PA_RIDX_NOT_USED;
	    }
    }

	return (0);
}

/* Reset all the exceptions in PASS */
int testExceptionSetRecover (void) {
	Cppi_HostDesc  *hd;
 	paCmdReply_t   cmdReply = {pa_DEST_HOST,			    /* Dest */
 							   TF_COMMON_CMD_ID_EXCEPTION_CFG,	/* Reply ID (returned in swinfo0) */
 							   0,						    /* Queue */
 							   0 };						    /* Flow ID */
	int 		   cmdDest, excUseIndex, i;
	uint16_t 	   cmdSize;
	paReturn_t     paret;
	int            fCmdReply;
    paRouteInfo_t  paRouteInfo[TF_PA_NUM_EXCEPTION];
    /* Default operation routing */
 	paRouteInfo_t  paDefRouteInfo;
	int            numExceptions = 0;
	int erouteTypes[TF_PA_NUM_EXCEPTION];
	tFramework_t *tf = &tFramework;
	
	memset((void *) &paRouteInfo[0], 0, sizeof (paRouteInfo) );
	memset((void *) &paDefRouteInfo, 0, sizeof (paDefRouteInfo) );
	paDefRouteInfo.dest = pa_DEST_DISCARD;

	cmdReply.queue = tf->QCommonCmdRep;	
	cmdReply.flowId= tf->tfFlowNum[0];
    
    /* Ingress Exception */

	for (excUseIndex = 0; excUseIndex < TF_PA_NUM_EXCEPTION; excUseIndex++) {

		/* Check if the command set is used for that index */
	    if (tf->excUseList[excUseIndex].indication == TF_PA_RIDX_USED ) {
			memcpy ((void *)&paRouteInfo[numExceptions], (void *)&paDefRouteInfo, sizeof (paRouteInfo_t));
			erouteTypes[numExceptions] = excUseIndex;
			numExceptions ++;
	    }
    }	

	if (numExceptions){
	    hd = testCommonConfigExceptionRoute (tf, numExceptions, erouteTypes, paRouteInfo,
	                                         tf->QDefRet, tf->QLinkedBuf3,
	                                         &cmdReply, &cmdDest, &cmdSize, &paret, FALSE);

        if ( (hd == NULL) ||
    	     (paret != pa_OK) ) {
	            System_printf ("testExceptionSetRecover: (%s:%d): Failure in Removing Exception Route\n", __FILE__, __LINE__);
	            return(-1);
        }

        /* Send command */
        Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

        fCmdReply = FALSE;
   	    /* All the packets should have been acked */
   	    for (i = 0; i < 100; i++)  {
   			utilCycleDelay (1000);
   		    testCommonCmdRep (tf, &fCmdReply, tf->QCommonCmdRep, tf->QDefRet, cmdReply.replyId);
	        if (fCmdReply == TRUE)
   			    break;
   	    }

   	    if (i == 100)  {
   		    System_printf ("testExceptionSetRecover: (%s:%d): Pa_configExceptionRoute, exceptions routes are not cleared\n", __FILE__, __LINE__);
   		    return (-1);
   	    }				
	}
    
#ifdef NSS_GEN2
    /* Egress Flow Exception */
    numExceptions = 0;
	for (excUseIndex = 0; excUseIndex < TF_PA_NUM_EFLOW_EXCEPTION; excUseIndex++) {

		/* Check if the command set is used for that index */
	    if (tf->efExcUseList[excUseIndex].indication == TF_PA_RIDX_USED ) {
			memcpy ((void *)&paRouteInfo[numExceptions], (void *)&paDefRouteInfo, sizeof (paRouteInfo_t));
			erouteTypes[numExceptions] = excUseIndex;
			numExceptions ++;
	    }
    }	

	if (numExceptions){
	    hd = testCommonConfigExceptionRoute (tf, numExceptions, erouteTypes, paRouteInfo,
	                                         tf->QDefRet, tf->QLinkedBuf3,
	                                         &cmdReply, &cmdDest, &cmdSize, &paret, TRUE);

        if ( (hd == NULL) ||
    	     (paret != pa_OK) ) {
	            System_printf ("testExceptionSetRecover: (%s:%d): Failure in Removing Exception Route\n", __FILE__, __LINE__);
	            return(-1);
        }

        /* Send command */
        Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

        fCmdReply = FALSE;
   	    /* All the packets should have been acked */
   	    for (i = 0; i < 100; i++)  {
   			utilCycleDelay (1000);
   		    testCommonCmdRep (tf, &fCmdReply, tf->QCommonCmdRep, tf->QDefRet, cmdReply.replyId);
	        if (fCmdReply == TRUE)
   			    break;
   	    }

   	    if (i == 100)  {
   		    System_printf ("testExceptionSetRecover: (%s:%d): Pa_configExceptionRoute, exceptions routes are not cleared\n", __FILE__, __LINE__);
   		    return (-1);
   	    }				
	}

#endif
	return (0);
}

/* Provide a Usr Stats configuration to the PA subsystem */
Cppi_HostDesc *testCommonConfigUsrStats (tFramework_t *tf, paUsrStatsConfigInfo_t *cfgInfo, 
 	                       		         int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
    *paret = pa_OK;
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
    {
		return (NULL);
    }    
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_configUsrStats (tf->passHandle,
					            cfgInfo,
					            (paCmd_t) hd->buffPtr,
					            cmdSize,
					            repInfo,
					            cmdDest);
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}
                                       
                                       


/* Provide a CRC Engine configuration to the PA sub-system */
Cppi_HostDesc *testCommonConfigCrcEngine (tFramework_t *tf, uint16_t index, paCrcConfig_t *cfgInfo, 
 	                       		          int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
    paCmd_t       cmdBuf = NULL;
	Cppi_HostDesc *hd = NULL;
    
#ifndef NSS_GEN2 
	Qmss_Queue     q;
	uint32_t	   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
    cmdBuf = (paCmd_t)hd->buffPtr;
    
#endif    
	
	*paret = Pa_configCrcEngine (tf->passHandle,
                                 index,
					             cfgInfo,
					             cmdBuf,
					             cmdSize,
					             repInfo,
					             cmdDest);
	
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
        #ifndef NSS_GEN2 
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
        #endif
		return (NULL);
	}
    
#ifndef NSS_GEN2 
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
#endif    
  	return (hd);
}

/* Provide a Global configuration to the PA sub-system */
Cppi_HostDesc *testCommonGlobalConfig (tFramework_t *tf, paCtrlInfo_t *cfgInfo, 
 	                       		       int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t	   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_control (tf->passHandle,
					     cfgInfo,
					     (paCmd_t) hd->buffPtr,
					     cmdSize,
					     repInfo,
					     cmdDest);
			
    /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}


Cppi_HostDesc *testCommonDelHandle (tFramework_t *tf, paHandleL2L3_t *paHdl, int Qrecycle, int QCmdMem, paCmdReply_t *cmdReply, 
									int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_delHandle (tf->passHandle, paHdl, (paCmd_t)hd->buffPtr, cmdSize, cmdReply, cmdDest);
	
	 /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}

Cppi_HostDesc *testCommonDelL4Handles (tFramework_t *tf, paHandleL4_t l4Handle, int Qrecycle, int QCmdMem, paCmdReply_t *reply, 
									   int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_delL4Handle (tf->passHandle, l4Handle, (paCmd_t)hd->buffPtr, cmdSize, reply, cmdDest);
	
 #ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}     

#ifdef NSS_GEN2

Cppi_HostDesc *testCommonDelEoamHandle (tFramework_t *tf, paHandleEoam_t *paHdl, int Qrecycle, int QCmdMem, paCmdReply_t *cmdReply, 
									   int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_delEoamHandle (tf->passHandle, paHdl, (paCmd_t)hd->buffPtr, cmdSize, cmdReply, cmdDest);
	
	 /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}

Cppi_HostDesc *testCommonDelAclHandle (tFramework_t *tf, paHandleAcl_t *paHdl, int Qrecycle, int QCmdMem, paCmdReply_t *cmdReply, 
									   int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_delAclHandle (tf->passHandle, paHdl, (paCmd_t)hd->buffPtr, cmdSize, cmdReply, cmdDest);
	
	 /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}

Cppi_HostDesc *testCommonDelFcHandle (tFramework_t *tf, paHandleFc_t *paHdl, int Qrecycle, int QCmdMem, paCmdReply_t *cmdReply, 
									  int *cmdDest, uint16_t *cmdSize, paReturn_t *paret)
{
	Cppi_HostDesc *hd;
	Qmss_Queue     q;
	uint32_t		   psCmd;
	
	/* Pop a descriptor with a linked buffer to create the command */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (QCmdMem)) & ~15);
	if (hd == NULL)
		return (NULL);
		
	*cmdSize = hd->origBufferLen;
	
	*paret = Pa_delFcHandle (tf->passHandle, paHdl, (paCmd_t)hd->buffPtr, cmdSize, cmdReply, cmdDest);
	
	 /* Restore the descriptor and return it on PA failure */
	if (*paret != pa_OK)  {
		hd->buffLen = hd->origBufferLen;
		Qmss_queuePush (QCmdMem, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
		return (NULL);
	}
	
#ifdef PA_SIM_BUG_4BYTES
  	*cmdSize = (*cmdSize+3)& ~3;
#endif

	/* Setup the return for the descriptor */
  	q.qMgr = 0;
  	q.qNum = Qrecycle;
  	Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
  	
  	/* Mark the packet as a configuration packet */
  	psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);
  	
    hd->buffLen = *cmdSize;
  	Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, *cmdSize);
  	
  	return (hd);
}

#endif


uint16_t testCommonOnesCompAdd (uint16_t v1, uint16_t v2)
{
	uint32_t v3;
	v3 = v1 + v2;
	v3 = (v3 & 0xffff) + (v3 >> 16);
	v3 = (v3 & 0xffff) + (v3 >> 16);
	return ((uint16_t)v3);
}
	     	
	
int numCmdError = 0;         	

int testCommonWaitCmdReply (tFramework_t *tf, paTest_t *pat, char *tname, int Qcmd, uint32_t swinfo0, int line)
{
	Cppi_HostDesc *hd;
	uint32_t        *swinfo;
    paEntryHandle_t	reth;
    paReturn_t     paret;
    int			   htype;
    int            cmdDest;
	int 		   i;
	
	/* Wait a bit for any packets in PA to complete */
	utilCycleDelay (5000);

	for (i = 0; i < 100; i++)  {
		if (Qmss_getQueueEntryCount(Qcmd) > 0)  {
			
			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (Qcmd)) & ~15);

            Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);
            
            if (swinfo[0] != swinfo0)  {
                System_printf ("%s (%s:%d): found packet in command reply queue with swifo[0] = 0x%08x (expected 0x%08x)\n", tname, __FILE__, line, swinfo[0], swinfo0);
                testCommonRecycleLBDesc (tf, hd);
                continue;
            }

            paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
            if (paret != pa_OK)
            {  
                System_printf ("%s (%s:%d): Pa_forwardResult returned error %d\n", tname, __FILE__, line, paret);
                numCmdError++;
            }    
            
            testCommonRecycleLBDesc (tf, hd); 
                
            return (0);

        }  else  {
            
          utilCycleDelay (1000);

        }
	}
	
	return (-1);

}	


/* Request the stats, compare to expected value */
paTestStatus_t testCommonCheckStats (tFramework_t *tf, paTest_t *pat, char *tName, paSysStats_t *expected, int32_t qSource, int32_t qCmdRecycle, int32_t qReply, Bool clear)
{ 	
	paSysStats_t    paStats1;
	paSysStats_t    *paStats = &paStats1;
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
 							   
 	cmdReply.queue  = (uint16_t) qReply;
	cmdReply.flowId = tf->tfFlowNum[0];
 	/* Check the PA stats to make sure they are set as expected */
 	cmdReply.replyId = TEST_COMMON_STATS_REQ_ID;
 	if (testCommonRequestPaStats (tName, tf, clear, qSource, qCmdRecycle,  &cmdReply))  {
 		System_printf ("%s (%s:%d): testCommonRequestPaStats failed\n", tName, __FILE__, __LINE__);
 		return (PA_TEST_FAILED);
 	}
 	
 	/* Wait for the stats reply */
	for (i = 0; i < 100; i++)  {
		utilCycleDelay (1000);
		if (Qmss_getQueueEntryCount (qReply) > 0)
			break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Did not find response from PA to stats request command\n", tName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
 	
	/* Recycle the descriptor/buffer returned from the stats request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (qCmdRecycle)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from stats request\n", tName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
	
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats request\n", tName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}

 		
 	/* Format the stats response and compare to the expected results */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (qReply)) & ~15);
	Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bp, &blen);
	paStats = Pa_formatStatsReply (tf->passHandle, (paCmd_t)bp);
    
    #else
    
	paReturn_t      paret;
    
	paret = Pa_querySysStats (tf->passHandle, clear, paStats);

	
	if (paret != pa_OK)  {
		System_printf ("%s (%s:%d): Pa_querySysStats returned error code %d\n", tName, __FILE__, __LINE__, paret);
		return (PA_TEST_FAILED);
	}
    
    #endif
	  
    if (testCommonCompareStats (tName, expected, paStats))
    	status = PA_TEST_FAILED;
    else
    	status = PA_TEST_PASSED;   
    	
    if (clear)
        memset (expected, 0, sizeof(paSysStats_t));
    	  
    #ifndef NSS_GEN2      
     /* Recycle the descriptor and associated buffer back to queue from which it came */
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
	}
    #endif
	
	return (status);
}	     	

/* Clear PASS system statistics */
int testCommonClearPaStats (void)
{ 	
    char *tName = "testCommonClearPaStats";
	tFramework_t   *tf = &tFramework;
    #ifndef NSS_GEN2 
	Cppi_HostDesc  *hd;
	int             i;

	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
	cmdReply.queue  = tf->QCommonCmdRep;
	cmdReply.flowId = tf->tfFlowNum[0];

 	/* Check the PA stats to make sure they are set as expected */
 	cmdReply.replyId = TEST_COMMON_STATS_REQ_ID;
 	if (testCommonRequestPaStats (tName, tf, TRUE, tf->QLinkedBuf1, tf->QDefRet,  &cmdReply))  {
 		System_printf ("%s (%s:%d): testCommonRequestPaStats failed\n", tName, __FILE__, __LINE__);
 		return (-1);
 	}
 	
 	/* Wait for the stats reply */
	for (i = 0; i < 100; i++)  {
		utilCycleDelay (1000);
		if (Qmss_getQueueEntryCount (tf->QCommonCmdRep) > 0)
			break;
	}
	
	if (i == 100)  {
		System_printf ("%s (%s:%d): Did not find response from PA to stats request command\n", tName, __FILE__, __LINE__);
		return (-1);
	}
 	
	/* Recycle the descriptor/buffer returned from the stats request */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QDefRet)) & ~15);
	if (hd == NULL)  {
		System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from stats request\n", tName, __FILE__, __LINE__);
		return (-1);
	}
	
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats request\n", tName, __FILE__, __LINE__);
		return (-1);
	}
 		
 	/* Format the stats response and compare to the expected results */
	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QCommonCmdRep)) & ~15);
    
     /* Recycle the descriptor and associated buffer back to queue from which it came */
	if (testCommonRecycleLBDesc (tf, hd))  {
		System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tName, __FILE__, __LINE__);
		return (-1);
	}
    #else
	paSysStats_t    paStats;
	paReturn_t      paret;
    
	paret = Pa_querySysStats (tf->passHandle, TRUE, &paStats);
	
	if (paret != pa_OK)  {
		System_printf ("%s (%s:%d): Pa_querySysStats returned error code %d\n", tName, __FILE__, __LINE__, paret);
		return (-1);
	}
    
    #endif
	  
	return (0);
}	     	


static paUsrStats_t  pauUsrStats;

/* Request the stats, compare to expected value */
paTestStatus_t testCommonCheckUsrStats (tFramework_t *tf, paTest_t *pat, char *tName, paUsrStats_t *expected, int32_t qSource, int32_t qCmdRecycle, int32_t qReply, Bool clear)
{ 	
	Cppi_HostDesc  *hd;
	paUsrStats_t   *usrStats = &pauUsrStats;
	int             i;
	paTestStatus_t  status;

	paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
 	cmdReply.queue   = (uint16_t) qReply;
	cmdReply.flowId  = tf->tfFlowNum[0];
    
    memset(usrStats, 0, sizeof(paUsrStats_t)); 
 	
 	/* Check the PA stats to make sure they are set as expected */
 	cmdReply.replyId = TEST_COMMON_STATS_REQ_ID;
 	if (testCommonRequestUsrStats (tName, tf, clear, qSource, qCmdRecycle, &cmdReply, usrStats))  {
 		System_printf ("%s (%s:%d): testCommonRequestUsrStats failed\n", tName, __FILE__, __LINE__);
 		return (PA_TEST_FAILED);
 	}
 	
    if (clear)
    {
 	    /* Wait for the stats reply */
	    for (i = 0; i < 100; i++)  {
		    utilCycleDelay (1000);
		    if (Qmss_getQueueEntryCount (qReply) > 0)
			    break;
	    }
	
	    if (i == 100)  {
		    System_printf ("%s (%s:%d): Did not find response from PA to stats request command\n", tName, __FILE__, __LINE__);
		    return (PA_TEST_FAILED);
	    }
 	
	    /* Recycle the descriptor/buffer returned from the stats request */
	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (qCmdRecycle)) & ~15);
	    if (hd == NULL)  {
		    System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from stats request\n", tName, __FILE__, __LINE__);
		    return (PA_TEST_FAILED);
	    }
	
	    if (testCommonRecycleLBDesc (tf, hd))  {
		    System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats request\n", tName, __FILE__, __LINE__);
		    return (PA_TEST_FAILED);
	    }

 	    /* Format the stats response and compare to the expected results */
	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (qReply)) & ~15);
        
        /* Recycle the descriptor and associated buffer back to queue from which it came */
	    if (testCommonRecycleLBDesc (tf, hd))  {
		    System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tName, __FILE__, __LINE__);
		    return (PA_TEST_FAILED);
	    }
    }
	  
    if (testCommonCompareUsrStats (tName, expected, usrStats))
    	status = PA_TEST_FAILED;
    else
    	status = PA_TEST_PASSED;   
    	
    if (clear)
        memset (expected, 0, sizeof(paUsrStats_t));
	
	return (status);
}	     	

paTestStatus_t testCommonPrintDbgInfo(tFramework_t *tf, paTest_t *pat, char *tName, paSnapShotDebugInfo_t* paDbgInfo, uint32_t debugType)
{
    int i;
    /* Print the Debug Information */
    switch (debugType)
    {
    case pa_DBG_INFO_TYPE_REASSEMBLY_ENABLE:
        System_printf ("Number of Active Traffic flows: Outer = %d, Inner = %d\n", \
                paDbgInfo->u.reassemContext.outer.numActiveTF, \
                paDbgInfo->u.reassemContext.inner.numActiveTF);
        System_printf ("Destination Queue (flowId): Outer = %d (%d), Inner = %d (%d)\n", \
                paDbgInfo->u.reassemContext.outer.queue, paDbgInfo->u.reassemContext.outer.flowId, \
                paDbgInfo->u.reassemContext.inner.queue, paDbgInfo->u.reassemContext.inner.flowId);

        System_printf ("Traffic Flow: Outer IP reassembly Context Info: \n");
        for (i = 0; i < paDbgInfo->u.reassemContext.outer.numActiveTF; i++)
        {
             System_printf ("    Active Index =%d\n", paDbgInfo->u.reassemContext.outer.traffic_flow[i].index);
             System_printf ("    Src Ip  = 0x%x\n", paDbgInfo->u.reassemContext.outer.traffic_flow[i].srcIp);
             System_printf ("    Dst Ip  = 0x%x\n", paDbgInfo->u.reassemContext.outer.traffic_flow[i].dstIp);
             System_printf ("    Protocol Field in IP Header  = 0x%x\n", paDbgInfo->u.reassemContext.outer.traffic_flow[i].proto);
             System_printf ("    Number of pending fragments and non-fragmented packets Count  = %d\n\n", paDbgInfo->u.reassemContext.outer.traffic_flow[i].count);
        }

        System_printf ("Traffic Flow: Inner IP reassembly Context Info: \n");
        for (i = 0; i < paDbgInfo->u.reassemContext.inner.numActiveTF; i++)
        {
             System_printf ("    Active Index =%d\n", paDbgInfo->u.reassemContext.inner.traffic_flow[i].index);
             System_printf ("    Src Ip  = 0x%x\n", paDbgInfo->u.reassemContext.inner.traffic_flow[i].srcIp);
             System_printf ("    Dst Ip  = 0x%x\n", paDbgInfo->u.reassemContext.inner.traffic_flow[i].dstIp);
             System_printf ("    Protocol Field in IP Header  = 0x%x\n", paDbgInfo->u.reassemContext.inner.traffic_flow[i].proto);
             System_printf ("    Number of pending fragments and non-fragmented packets Count  = %d\n\n", paDbgInfo->u.reassemContext.inner.traffic_flow[i].count);
        }
        break;
    default:
        break;
    }

    return (PA_TEST_PASSED);
}
/* Request and comapre a list of user-defined statistics and clear the list of stats optionally */
paTestStatus_t testCommonCheckUsrStatsList (tFramework_t *tf, paTest_t *pat, char *tName, paUsrStats_t *expected,
                                            int num64bStats, int numCnts, pauUsrStatsEntry_t *usrStatsTbl,
                                            int32_t qSource, int32_t qCmdRecycle, int32_t qReply, Bool clear)
{
    Cppi_HostDesc  *hd;
    uint8_t        *bp;
    paUsrStats_t   *usrStats = &pauUsrStats;
    uint16_t        usrCnt[TF_MAX_USR_STATS];
    uint32_t    	blen;
    int             i;
    paTestStatus_t  status;
    paReturn_t      paret;
    uint16_t        index;


    paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
 							   0,						/* Reply ID (returned in swinfo0) */
 							   0,						/* Queue */
 							   0 };						/* Flow ID */
 							   
 	cmdReply.queue   = (uint16_t) qReply;

    memset(usrStats, 0, sizeof(paUsrStats_t));

    if(numCnts > TF_MAX_USR_STATS)
    {
	    System_printf ("%s (%s:%d): testCommonCheckUsrStatsList: numCnts (%d) exceeds the limit (%d)\n",
	  	    			tName, __FILE__, __LINE__, numCnts, TF_MAX_USR_STATS);

        return (PA_TEST_FAILED);
    }

    for (i = 0; i < numCnts; i++)
        usrCnt[i] = usrStatsTbl[i].cntIndex;


    /* Check the PA stats to make sure they are set as expected */
    cmdReply.replyId = TEST_COMMON_STATS_REQ_ID;
    cmdReply.flowId  = tf->tfFlowNum[0];
    if (testCommonRequestUsrStatsList (tName, tf, clear, numCnts, usrCnt, qSource, qCmdRecycle, &cmdReply, NULL))  {
        System_printf ("%s (%s:%d): testCommonRequestPaStatsList failed\n", tName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }

    /* Wait for the stats reply */
    for (i = 0; i < 100; i++)  {
        utilCycleDelay (1000);
        if (Qmss_getQueueEntryCount (qReply) > 0)
            break;
    }

    if (i == 100)  {
        System_printf ("%s (%s:%d): Did not find response from PA to user stats request command\n", tName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }

    /* Recycle the descriptor/buffer returned from the stats request */
    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (qCmdRecycle)) & ~15);
    if (hd == NULL)  {
        System_printf ("%s (%s:%d): Did not find returned descriptor/buffer from user stats request\n", tName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }

    if (testCommonRecycleLBDesc (tf, hd))  {
        System_printf ("%s (%s:%d): Failed to find original free buffer Q for user stats request\n", tName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }

    /* Format the stats response and compare to the expected results */
    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (qReply)) & ~15);
    Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bp, &blen);
    if ((paret = Pa_formatUsrStatsReply (tf->passHandle, (paCmd_t)bp, usrStats)) != pa_OK)
    {
        System_printf ("%s (%s:%d): Pa_formatUsrStatsReply return error code = %d!\n", tName, __FILE__, __LINE__, paret);
        testCommonRecycleLBDesc (tf, hd);\
        return (PA_TEST_FAILED);
    }
    
    /* Recycle the descriptor and associated buffer back to queue from which it came */
    if (testCommonRecycleLBDesc (tf, hd))  {
        System_printf ("%s (%s:%d): Failed to find original free buffer Q for stats response\n", tName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }

    if (testCommonCompareUsrStatsList (tName, expected, usrStats, num64bStats, numCnts, usrCnt))
        status = PA_TEST_FAILED;
    else
        status = PA_TEST_PASSED;

    if (clear)
    {
        for (i = 0; i < numCnts; i++)
        {
            index = usrCnt[i];
            if (index < num64bStats)
            {
                /* 64-bit counters */
                expected->count64[index] = 0;
            }
            else
            {
                /* 32-bit counters */
                index -= num64bStats;
                expected->count32[index] = 0;
            }
        }
    }

    return (status);
}



int commonFifoPushElement (pauFifo_t *fifo, unsigned int elem)
{
	int nextIn;
	
	nextIn = fifo->inIdx + 1;
	if (nextIn >= fifo->size)
		nextIn = 0;
	
	/* Check if the fifo is full */
	if (nextIn == fifo->outIdx)
		return (-1);
		
	/* Add the element */
	fifo->data[fifo->inIdx] = elem;
	fifo->inIdx = nextIn;
	
	return (0);
}

int commonFifoGetCount (pauFifo_t *fifo)
{
	if (fifo->inIdx >= fifo->outIdx)
		return (fifo->inIdx - fifo->outIdx);
	else
		return (fifo->size + fifo->inIdx - fifo->outIdx);
}

unsigned int commonFifoPopElement (pauFifo_t *fifo, int *numInQBeforePop)
{
	unsigned int v;
	
	*numInQBeforePop = commonFifoGetCount (fifo);
	if (*numInQBeforePop == 0)
		return (0);
		
	v = fifo->data[fifo->outIdx];
	fifo->outIdx = fifo->outIdx + 1;
	if (fifo->outIdx >= fifo->size)
		fifo->outIdx = 0;
		
	return (v);
}
		
	
int testCommonVerifyCmdResult (tFramework_t *tf, paSysStats_t *stats, char *tfName, char *fname, int fline, char *tid, Cppi_HostDesc *hd, 
								paReturn_t exp, paReturn_t act, pauTestSetupStatus_t *status, int cmdDest, int cmdSize)
{
	
	if (act != exp)  {
		System_printf ("%s (%s:%d - from %s:%d): %s returned %d, expected %d\n", tfName, __FILE__, __LINE__, fname, fline, tid,
						act, exp);
	
		if (hd != NULL)  {
			if (testCommonRecycleLBDesc (tf, hd))  {
				System_printf ("%s (%s:%d - from %s:%d): Failed to find original free buffer Q for stats request\n", tfName, 
						__FILE__, __LINE__, fname, fline);
			}
		}
		
		/* Don't change the status of the entry */
		return (-1);
		
	}  else  {
		
		if (act == pa_OK)  {
			if (hd == NULL)  {
				System_printf ("%s (%s:%d - from %s:%d): %s returned OK but no buffer descriptor found\n",
						tfName, __FILE__, __LINE__, fname, fline), tid;
				return (-1);
			}
			
			Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
			*status = PAU_TEST_SETUP_STATUS_CMD_SENT;
			if ((cmdDest == pa_CMD_TX_DEST_0) || (cmdDest == pa_CMD_TX_DEST_1) || (cmdDest == pa_CMD_TX_DEST_2))
				stats->classify1.nPackets += 1;
			else if ((cmdDest == pa_CMD_TX_DEST_3))
            {
                #ifndef NSS_GEN2
				stats->classify2.nPackets += 1;
                #else
				stats->classify1.nPackets += 1;
                #endif
            }    
            #ifdef NSS_GEN2
			else if ((cmdDest == pa_CMD_TX_DEST_4))
            {
				stats->classify2.nPackets += 1;
				stats->classify1.nPackets += 1;
            }    
            #endif
			return (0);
		
		}  else  {
			
			/* The descriptor was already returned if the pa result code was not OK */
			*status = PAU_TEST_SETUP_STATUS_CMD_NO_CMD;  /* Command is complete */
		
			return (0);
		}
	}
	
}
	


int testCommonMacSetup (tFramework_t *tf, paSysStats_t *stats, pauTestMacSetup_t *maci, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 	   cmdSize;
	paReturn_t     paret;
	
	/* Set the reply ID to indicate the type and index of the setup */
	rep->replyId = TF_COMMON_CMD_ID_ADD_MAC + idx;
	
	hd = testCommonAddMac (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &maci->ethInfo, &maci->matchRoute, &maci->nFail, &maci->handle, 
						   maci->bufferQ, maci->bufferQ, rep, &cmdDest, &cmdSize, &paret);
						   
	return (testCommonVerifyCmdResult (tf, stats, tfName, fname, fline, "Pa_addMac", hd, maci->paret, paret, &maci->status, cmdDest, cmdSize));
}
						   
		

int testCommonIpSetup (tFramework_t *tf, paSysStats_t *stats, pauTestIpSetup_t *ipi, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	paHandleL2L3_t linkHandle = NULL;
	
	/* Set the reply ID to indicate the type and index of the reply */
	rep->replyId = TF_COMMON_CMD_ID_ADD_IP + idx;
	
	
	/* Verify a valid previous link */
	if ((ipi->l2Link != NULL) && (ipi->l3Link == NULL))  {
		if (ipi->l2Link->status != PAU_TEST_SETUP_STATUS_CMD_REPLIED)  {
			System_printf ("%s: (%s:%d - from %s:%d): IP linked to L2 handle that is not in CMD_REPLIED state\n",
							tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		linkHandle = ipi->l2Link->handle;
		
	} else if ((ipi->l2Link == NULL) && (ipi->l3Link != NULL))  {
		if (ipi->l3Link->status != PAU_TEST_SETUP_STATUS_CMD_REPLIED)  {
			System_printf ("%s: (%s:%d - from %s:%d): IP linked to L3 handle that is not in CMD_REPLIED state\n",
							tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		linkHandle = ipi->l3Link->handle;
		
	}  else if ((ipi->l2Link != NULL) && (ipi->l3Link != NULL))  {
		System_printf ("%s: (%s:%d - from %s:%d): Invalid IP configuration - both L2 and L3 links specified\n",
					tfName, __FILE__, __LINE__, fname, fline);
		return (-1);
	}
	
	
	hd = testCommonAddIp (tf, pa_LUT1_INDEX_NOT_SPECIFIED, &ipi->ipInfo, &ipi->matchRoute, &ipi->nFail, &ipi->handle, linkHandle, ipi->bufferQ,
						  ipi->bufferQ, rep, &cmdDest, &cmdSize, &paret);
						  
	return (testCommonVerifyCmdResult (tf, stats, tfName, fname, fline, "Pa_addIp", hd, ipi->paret, paret, &ipi->status, cmdDest, cmdSize));
						  
}


int testCommonL4Setup (tFramework_t *tf, paSysStats_t *stats, pauTestL4Setup_t *l4i, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	paHandleL2L3_t linkHandle = NULL;
	
	/* Set the reply ID to indicate the type and index of the reply */
	rep->replyId = TF_COMMON_CMD_ID_ADD_PORT + idx;
	
	/* Verify that the linked handle is valid */
	if (l4i->l3Link != NULL)  {
		if (l4i->l3Link->status != PAU_TEST_SETUP_STATUS_CMD_REPLIED)  {
			System_printf ("%s: (%s:%d - from %s:%d): IP linked to UDP handle that is not in CMD_REPLIED state\n",
							tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		linkHandle = l4i->l3Link->handle;
	}	

	hd = testCommonAddPort (tf, pa_LUT2_PORT_SIZE_16, (uint32_t)l4i->destPort, &l4i->matchRoute, &l4i->handle, &linkHandle, l4i->bufferQ, l4i->bufferQ,
							rep, &cmdDest, &cmdSize, &paret);
	
	return (testCommonVerifyCmdResult (tf, stats, tfName, fname, fline, "Pa_addPort", hd, l4i->paret, paret, &l4i->status, cmdDest, cmdSize));
	
	
}

int testCommonCl3Config (tFramework_t *tf, paSysStats_t *stats, pauTestCl3Config_t *l3cfg, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	
	rep->replyId = TF_COMMON_CMD_ID_CFG_CL3 + idx;
	
	
	hd = testCommonAddCl3Config (tf, l3cfg->custIndex, l3cfg->offset, l3cfg->nextHdr, l3cfg->nextHdrOffset, l3cfg->byteMasks, l3cfg->bufferQ,
								 l3cfg->bufferQ, rep, &cmdDest, &cmdSize, &paret);
								
	return (testCommonVerifyCmdResult (tf, stats, tfName, fname, fline, "Pa_setCustomLUT1", hd, l3cfg->paret, paret, &l3cfg->status, cmdDest, cmdSize));

}	

int testCommonCl3Setup (tFramework_t *tf, paSysStats_t *stats, pauTestCl3Setup_t *l3s, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx)
{
	Cppi_HostDesc *hd;
    int            lutInst = pa_LUT_INST_NOT_SPECIFIED;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	paHandleL2L3_t linkHandle = NULL;
	
	rep->replyId = TF_COMMON_CMD_ID_ADD_CL3 + idx;
	
	/* Verify that the linked handle is valid */
	/* Verify a valid previous link */
	if ((l3s->l2Link != NULL) && (l3s->l3Link == NULL))  {
		if (l3s->l2Link->status != PAU_TEST_SETUP_STATUS_CMD_REPLIED)  {
			System_printf ("%s: (%s:%d - from %s:%d): IP linked to L2 handle that is not in CMD_REPLIED state\n",
							tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		linkHandle = l3s->l2Link->handle;
        lutInst = pa_LUT1_INST_1;
		
	} else if ((l3s->l2Link == NULL) && (l3s->l3Link != NULL))  {
		if (l3s->l3Link->status != PAU_TEST_SETUP_STATUS_CMD_REPLIED)  {
			System_printf ("%s: (%s:%d - from %s:%d): IP linked to L3 handle that is not in CMD_REPLIED state\n",
							tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		linkHandle = l3s->l3Link->handle;
        lutInst = pa_LUT1_INST_2;
		
	}  else if ((l3s->l2Link != NULL) && (l3s->l3Link != NULL))  {
		System_printf ("%s: (%s:%d - from %s:%d): Invalid IP configuration - both L2 and L3 links specified\n",
					tfName, __FILE__, __LINE__, fname, fline);
		return (-1);
	}
    
	hd = testCommonAddCustomL3 (tf, l3s->custIndex, lutInst, l3s->match, &l3s->matchRoute, &l3s->nFail, &l3s->handle,  linkHandle, l3s->bufferQ, l3s->bufferQ, rep,
								&cmdDest, &cmdSize, &paret);
	
	return (testCommonVerifyCmdResult (tf, stats, tfName, fname, fline, "Pa_addCustomLUT1", hd, l3s->paret, paret, &l3s->status, cmdDest, cmdSize));

}



	
int testCommonCl4Config (tFramework_t *tf, paSysStats_t *stats, pauTestCl4Config_t *l4cfg, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	
	rep->replyId = TF_COMMON_CMD_ID_CFG_CL4 + idx;
	
	
	hd = testCommonAddCl4Config (tf, l4cfg->handleLink, l4cfg->custIndex, l4cfg->custHdrSize, l4cfg->byteOffsets, l4cfg->byteMasks, l4cfg->bufferQ,
								l4cfg->bufferQ, rep, &cmdDest, &cmdSize, &paret);
								
	return (testCommonVerifyCmdResult (tf, stats, tfName, fname, fline, "Pa_setCustomLUT2", hd, l4cfg->paret, paret, &l4cfg->status, cmdDest, cmdSize));

}	

int testCommonCl4Setup (tFramework_t *tf, paSysStats_t *stats, pauTestCl4Setup_t *l4s, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	paHandleL2L3_t linkHandle = NULL;
	
	rep->replyId = TF_COMMON_CMD_ID_ADD_CL4 + idx;
	
	/* Verify that the linked handle is valid */
	if (l4s->l3Link != NULL)  {
		if (l4s->l3Link->status != PAU_TEST_SETUP_STATUS_CMD_REPLIED) {
			System_printf ("%s: (%s:%d - from %s:%d): IP linked to Custom L4 handle that is not in CMD_REPLIED state\n",
							tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		linkHandle = l4s->l3Link->handle;
		
	}	
	
	hd = testCommonAddCustomL4 (tf, l4s->custIndex, l4s->match, &l4s->matchRoute, l4s->handle,  linkHandle, l4s->bufferQ, l4s->bufferQ, rep,
								&cmdDest, &cmdSize, &paret);
	
	return (testCommonVerifyCmdResult (tf, stats, tfName, fname, fline, "Pa_addCustomLUT2", hd, l4s->paret, paret, &l4s->status, cmdDest, cmdSize));

}


int testCommonCmdSetSetup (tFramework_t *tf, paSysStats_t *stats, pauTestCmdSetSetup_t *cmdSet, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	
	rep->replyId = TF_COMMON_CMD_ID_ADD_CMDSET + idx;
	
	hd =  testCommonConfigCmdSet(tf, cmdSet->index, cmdSet->nCmds, cmdSet->cmdInfo, cmdSet->bufferQ, cmdSet->bufferQ, rep,
								 &cmdDest, &cmdSize, &paret);
	
	return (testCommonVerifyCmdResult (tf, stats, tfName, fname, fline, "Pa_configCmdSet", hd, cmdSet->paret, paret, &cmdSet->status, cmdDest, cmdSize));

}

int testCommonCloseL4 (tFramework_t *tf, paSysStats_t *stats, paHandleL4_t handle, pauTestSetupStatus_t *status, paCmdReply_t *rep, int Q, char *tfName, char *fname, int fline)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	
	hd = testCommonDelL4Handles (tf, handle, Q, Q, rep, &cmdDest, &cmdSize, &paret);
	
	if ((hd == NULL) || paret != pa_OK)  {
		System_printf ("%s (%s:%d from %s:%d): close L4 handle failed, paret = %d\n", tfName, __FILE__, __LINE__, fname, fline, paret);
		*status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;  /* Command not yet sent */		
		return (-1);
	}
	
	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
	stats->classify2.nPackets += 1;
    #ifdef NSS_GEN2
	stats->classify1.nPackets += 1;
    #endif
	*status = PAU_TEST_SETUP_STATUS_CMD_SENT;  /* Command sent */			
	return (0);
}


int testCommonCloseL2L3 (tFramework_t *tf, paSysStats_t *stats, paHandleL2L3_t *handle, pauTestSetupStatus_t *status, paCmdReply_t *rep, int Q, char *tfName, char *fname, int fline)
{
	Cppi_HostDesc *hd;
	int 		   cmdDest;
	uint16_t 		   cmdSize;
	paReturn_t     paret;
	
	hd = testCommonDelHandle (tf, handle, Q, Q, rep, &cmdDest, &cmdSize, &paret);
	
	if ((hd == NULL) || paret != pa_OK)  {
		System_printf ("%s (%s:%d from %s:%d): close L2/L3 handle failed, paret = %d\n", tfName, __FILE__, __LINE__, fname, fline, paret);
		*status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;  /* Command not yet sent */				
		return (-1);
	}
	
	stats->classify1.nPackets += 1;
	
	Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
	*status = PAU_TEST_SETUP_STATUS_CMD_SENT;  /* Command sent */			
	
	return (0);
}
	
	
		
	
	
/* Examine the reply queue and change command state */
void testCommonCheckComplete (tFramework_t *tf, pauTestSetup_t *setup, char *tfName, char *fname, int fline)
{
	Cppi_HostDesc  *hd;
	uint32_t 		   *swinfo;
	uint32_t			swinfoType;
	uint32_t			swinfoChan;
	paReturn_t      paret;
	paEntryHandle_t	reth;
    int			    htype;
    int             cmdDest;
	
	pauTestSetupStatus_t *status;
	int					  maxChan;
	char				 *tid;
	
	while ((hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QCommonCmdRep)) & ~15)) != NULL)  {
		
		Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);
		swinfoType = swinfo[0] & 0xffff0000;
		swinfoChan = swinfo[0] & 0x0000ffff;
		
		switch (swinfoType)  {
			case TF_COMMON_CMD_ID_ADD_MAC:  status = &setup->macs[swinfoChan].status;    maxChan = setup->nMac;     tid = "Add Mac";        break;
			case TF_COMMON_CMD_ID_ADD_IP:   status = &setup->ips[swinfoChan].status;     maxChan = setup->nIps;     tid = "Add Ip";         break;
			case TF_COMMON_CMD_ID_ADD_PORT: status = &setup->l4s[swinfoChan].status;     maxChan = setup->nL4s;     tid = "Add L4";         break;
			case TF_COMMON_CMD_ID_CFG_CL3:  status = &setup->cl3cfgs[swinfoChan].status; maxChan = setup->nCl3Cfgs; tid = "CL3 config";     break;
			case TF_COMMON_CMD_ID_ADD_CL3:  status = &setup->cl3s[swinfoChan].status;    maxChan = setup->nCl3s;    tid = "Add Cl3";        break;
			case TF_COMMON_CMD_ID_DEL_CL3:  status = &setup->cl3s[swinfoChan].status;    maxChan = setup->nCl3s;    tid = "Del Cl3";		break;
			case TF_COMMON_CMD_ID_CFG_CL4:  status = &setup->cl4cfgs[swinfoChan].status; maxChan = setup->nCl4Cfgs; tid = "CL4 config";     break;
			case TF_COMMON_CMD_ID_ADD_CL4:  status = &setup->cl4s[swinfoChan].status;    maxChan = setup->nCl4s;    tid = "Add Cl4";        break;
			case TF_COMMON_CMD_ID_DEL_CL4:  status = &setup->cl4s[swinfoChan].status;    maxChan = setup->nCl4s;    tid = "Del Cl4";		break;
			case TF_COMMON_CMD_ID_DEL_PORT: status = &setup->l4s[swinfoChan].status;     maxChan = setup->nL4s;     tid = "Del L4";         break;
			case TF_COMMON_CMD_ID_DEL_IP:   status = &setup->ips[swinfoChan].status;     maxChan = setup->nIps;     tid = "Del Ip";   		break;
			case TF_COMMON_CMD_ID_DEL_MAC:  status = &setup->macs[swinfoChan].status;    maxChan = setup->nMac;     tid = "Del Mac";		break;
			case TF_COMMON_CMD_ID_ADD_CMDSET:  status = &setup->cmdSet[swinfoChan].status;    maxChan = setup->nCmdSets;    tid = "AddCmd Set";        break;
						
			default:
				System_printf ("%s: (%s:%d - from %s:%d): Found descriptor in common command rep queue with unknown swinfo0 = 0x%08x\n",
						tfName, __FILE__, __LINE__, fname, fline, swinfo[0]);
				testCommonRecycleLBDesc (tf, hd);
				continue;
		}
		
		if (swinfoChan >= maxChan)  {
			System_printf ("%s: (%s:%d - from %s:%d): Found channel %d in %s setup, but max value is %d\n",
							tfName, __FILE__, __LINE__, fname, fline, swinfoChan, tid, maxChan);
			testCommonRecycleLBDesc (tf, hd);
			continue;
		}
	
		*status = PAU_TEST_SETUP_STATUS_CMD_REPLIED;
		
		if (swinfoType != TF_COMMON_CMD_ID_CFG_CL4)  {
			paret = Pa_forwardResult (tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);
			
			if (paret != pa_OK)  {
				System_printf ("%s: (%s:%d - from %s:%d): Pa_forwardResult returned error code %d\n",
								tfName, __FILE__, __LINE__, fname, fline, paret);
				*status = PAU_TEST_SETUP_STATUS_CMD_REP_ERR;
				
			}
		}
		
		testCommonRecycleLBDesc (tf, hd);
	}
	
}		
		


/* Returns TRUE if all entries are complete */
Bool testCommonWaitComplete (tFramework_t *tf, pauTestSetup_t *setup, char *tfName, char *fname, int fline)
{
	int  i, j;
	Bool done;
	
	for (i = 0, done = FALSE; (i < 100) && (done == FALSE); i++)  {
    
		utilCycleDelay (5000);
        
        /* Process command response packet */
		testCommonCheckComplete (tf, setup, tfName, fname, fline);
    
        /* Verify whether all command responses are received */
		done = TRUE;
		
		for (j = 0; (j < setup->nMac) && (done == TRUE); j++)  
			if (setup->macs[j].status == PAU_TEST_SETUP_STATUS_CMD_SENT)  
				done = FALSE;
		
		for (j = 0; (j < setup->nIps) && (done == TRUE); j++)
			if (setup->ips[j].status == PAU_TEST_SETUP_STATUS_CMD_SENT)
				done = FALSE;
				
		for (j = 0; (j < setup->nL4s) && (done == TRUE); j++)
			if (setup->l4s[j].status == PAU_TEST_SETUP_STATUS_CMD_SENT)
				done = FALSE;
				
		for (j = 0; (j < setup->nCl3Cfgs) && (done == TRUE); j++)
			if (setup->cl3cfgs[j].status == PAU_TEST_SETUP_STATUS_CMD_SENT)
				done = FALSE;
				
		for (j = 0; (j < setup->nCl3s) && (done == TRUE); j++)
			if (setup->cl3s[j].status == PAU_TEST_SETUP_STATUS_CMD_SENT)
				done = FALSE;
                
		for (j = 0; (j < setup->nCl4Cfgs) && (done == TRUE); j++)
			if (setup->cl4cfgs[j].status == PAU_TEST_SETUP_STATUS_CMD_SENT)
				done = FALSE;
				
		for (j = 0; (j < setup->nCl4s) && (done == TRUE); j++)
			if (setup->cl4s[j].status == PAU_TEST_SETUP_STATUS_CMD_SENT)
				done = FALSE;
				
	}
	
	return (done);
}


/* Wait for a specific command to complete */
int testCommonCheckWait (tFramework_t *tf, pauTestSetup_t *setup, char *tfName, char *fname, int fline, pauTestSetupStatus_t *status)
{
	int i;
	
	for (i = 0; i < 100; i++)  {
		testCommonCheckComplete (tf, setup, tfName, fname, fline);
		if (*status != PAU_TEST_SETUP_STATUS_CMD_SENT)
			break;
		else
			utilCycleDelay (1000);
	}
	
	if (i == 100)
		return (-1);
		
	return (0);
	
}


/* Search the Queue Bounce queue for received packets. */
void testCommonRelayQueueBouncePkts (tFramework_t *tf, char *tfName, int ddrQIdx, int msmcQIdx, int* ddrCnt, int* msmcCnt)
{
	Cppi_HostDesc    *hd;
	uint32_t		 *swInfo;
    int              ddrCount = 0, msmcCount = 0;
    uint32_t         queueId;

    /*
     * Look for entries in the Queue bounce DDR queue
     */
	while (Qmss_getQueueEntryCount(tf->QGen[ddrQIdx]) > 0)  {

	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[ddrQIdx])) & ~15);
	    if (hd == NULL)  {
		    System_printf ("%s (%s:%d): Failed to pop a Queue Bounce DDR queue packet\n", tfName, __FILE__, __LINE__);
		    break;
	    }

		Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
        queueId = swInfo[1] &0x3FFF;
		Qmss_queuePushDescSize (Qmss_getHandleFromQID(queueId), (Ptr)hd, TF_SIZE_DESC);

        ddrCount++;

	}

    /*
     * Look for entries in the Queue bounce DDR queue
     */
	while (Qmss_getQueueEntryCount(tf->QGen[msmcQIdx]) > 0)  {

	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QGen[msmcQIdx])) & ~15);
	    if (hd == NULL)  {
		    System_printf ("%s (%s:%d): Failed to pop a Queue Bounce MSMC queue packet\n", tfName, __FILE__, __LINE__);
		    break;
	    }

		Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);
        queueId = swInfo[1] &0x3FFF;
		Qmss_queuePushDescSize (Qmss_getHandleFromQID(queueId), (Ptr)hd, TF_SIZE_DESC);

        msmcCount++;

	}

    if(ddrCnt)*ddrCnt += ddrCount;
    if(msmcCnt)*msmcCnt += msmcCount;
}
		
		
/* Setup PA for a test */
int testCommonSetupTest (tFramework_t *tf, paSysStats_t *stats, pauTestSetup_t *setup, char *tfName, char *fname, int fline)
{
	int i;
	paCmdReply_t cmdReply = {  pa_DEST_HOST,						/* Dest */
 							      0,								/* Reply ID (returned in swinfo0) */
 							   	  0,								/* Queue, set by common setup */
 							      0 };		 						/* Flow ID */
 							     
	cmdReply.queue = tf->QCommonCmdRep; 							  
	cmdReply.flowId= tf->tfFlowNum[0];
	
	/* Initialize the test state */
	for (i = 0; i < setup->nMac; i++)
		setup->macs[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
	
	for (i = 0; i < setup->nIps; i++)
		setup->ips[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
		
	for (i = 0; i < setup->nL4s; i++)
		setup->l4s[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
		
	for (i = 0; i < setup->nCl3Cfgs; i++)
		setup->cl3cfgs[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
		
	for (i = 0; i < setup->nCl3s; i++)
		setup->cl3s[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
        
	for (i = 0; i < setup->nCl4Cfgs; i++)
		setup->cl4cfgs[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
		
	for (i = 0; i < setup->nCl4s; i++)
		setup->cl4s[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
	
	/* MAC setup */
	for (i = 0; i < setup->nMac; i++)  {
        setup->macs[i].nFail.flowId = tf->tfFlowNum[0];
		if (testCommonMacSetup (tf, stats, &(setup->macs[i]), &cmdReply, tfName, fname, fline, i))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses (MAC) not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->macs[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->macs[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): MAC setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else 
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
	}
						
			
	
	/* IP Setup */
	for (i = 0; i < setup->nIps; i++)  {
        setup->ips[i].nFail.flowId = tf->tfFlowNum[0];
		if (testCommonIpSetup (tf, stats, &(setup->ips[i]), &cmdReply, tfName, fname, fline, i))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses (IP) not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->ips[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->ips[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): IP setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else 
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
	}
	
	/* Custom L3 config */
	for (i = 0; i < setup->nCl3Cfgs; i++)  {
		if (testCommonCl3Config (tf, stats, &(setup->cl3cfgs[i]), &cmdReply, tfName, fname, fline, i))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responces (L3 Custom Config) not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->cl3cfgs[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->cl3cfgs[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): Custom L3 config failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
			
	}
	
	/* Custom L3 setup */
	for (i = 0; i < setup->nCl3s; i++)  {
        setup->cl3s[i].matchRoute.flowId = tf->tfFlowNum[0];
        setup->cl3s[i].nFail.flowId = tf->tfFlowNum[0];
		if (testCommonCl3Setup (tf, stats, &setup->cl3s[i], &cmdReply, tfName, fname, fline, i))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses (L3 Custom Setup) not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->cl3s[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->cl3s[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): Custom L3 setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
	}
    
	/* L4 Setup */
	for (i = 0; i < setup->nL4s; i++)  {
        setup->l4s[i].matchRoute.flowId = tf->tfFlowNum[0];
		if (testCommonL4Setup (tf, stats, &(setup->l4s[i]), &cmdReply, tfName, fname, fline, i))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses (L4) not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->l4s[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->l4s[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): L4 setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
            
        /* Wait a while for the next L4 entry */
        utilCycleDelay (1000);    
	}
    
	/* Custom L4 config */
	for (i = 0; i < setup->nCl4Cfgs; i++)  {
		if (testCommonCl4Config (tf, stats, &(setup->cl4cfgs[i]), &cmdReply, tfName, fname, fline, i))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responces (Custom L4 Config) not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->cl4cfgs[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->cl4cfgs[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): Custom L4 config failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
			
	}
	
	/* Custom L4 setup */
	for (i = 0; i < setup->nCl4s; i++)  {
        setup->cl4s[i].matchRoute.flowId = tf->tfFlowNum[0];
		if (testCommonCl4Setup (tf, stats, &setup->cl4s[i], &cmdReply, tfName, fname, fline, i))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses (Custom L4) not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->cl4s[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->cl4s[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): Custom L4 setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
            
        /* Wait a while for the next L4 entry */
        utilCycleDelay (1000);    
            
	}
    
	/* Command Set setup */
	for (i = 0; i < setup->nCmdSets; i++)  {
		if (testCommonCmdSetSetup (tf, stats, &setup->cmdSet[i], &cmdReply, tfName, fname, fline, i))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses (Command Set Configuration) not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->cmdSet[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->cmdSet[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): Command Set setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
            
        /* Wait a while for the next L4 entry */
        utilCycleDelay (1000);    
            
	}

	if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
		System_printf ("%s: (%s:%d - called from %s:%d): PA responses not complete\n", tfName, __FILE__, __LINE__, fname, fline);
	
	
	return (0);
}
	
/* Teardown a test which was setup using testCommonSetupTest */
int testCommonTeardownTest (tFramework_t *tf, paSysStats_t *stats, pauTestSetup_t *setup, char *tfName, char *fname, int fline)
{
	int i;
	paCmdReply_t cmdReply = {  pa_DEST_HOST,						/* Dest */
 							      0,								/* Reply ID (returned in swinfo0) */
 							   	  0,								/* Queue, set by common setup */
 							      0 };		 						/* Flow ID */
 							     
	cmdReply.queue = tf->QCommonCmdRep; 							  
	cmdReply.flowId= tf->tfFlowNum[0];
	
	/* Only teardown channels that were active */
	for (i = 0; i < setup->nMac; i++)
		if (setup->macs[i].status == PAU_TEST_SETUP_STATUS_CMD_REPLIED)
			setup->macs[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
	
	for (i = 0; i < setup->nIps; i++)
		if (setup->ips[i].status == PAU_TEST_SETUP_STATUS_CMD_REPLIED)
			setup->ips[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
		
	for (i = 0; i < setup->nCl3s; i++)
		if (setup->cl3s[i].status == PAU_TEST_SETUP_STATUS_CMD_REPLIED)
			setup->cl3s[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
            
	for (i = 0; i < setup->nL4s; i++)
		if (setup->l4s[i].status == PAU_TEST_SETUP_STATUS_CMD_REPLIED)
			setup->l4s[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
		
	/* There is no teardown for custom L4 config */
		
	for (i = 0; i < setup->nCl4s; i++)
		if (setup->cl4s[i].status == PAU_TEST_SETUP_STATUS_CMD_REPLIED)
			setup->cl4s[i].status = PAU_TEST_SETUP_STATUS_CMD_NOT_SENT;
			
	/* Custom L4 teardown */
	for (i = 0; i < setup->nCl4s; i++)  {
		cmdReply.replyId = TF_COMMON_CMD_ID_DEL_CL4 + i;
		if (testCommonCloseL4 (tf, stats, setup->cl4s[i].handle, &setup->cl4s[i].status, &cmdReply, setup->cl4s[i].bufferQ, tfName, fname, fline))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->cl4s[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->cl4s[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): Custom L4 setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
	}
	
	/* L4 Teardown */
	for (i = 0; i < setup->nL4s; i++)  {
		cmdReply.replyId = TF_COMMON_CMD_ID_DEL_PORT + i;
		if (testCommonCloseL4 (tf, stats, setup->l4s[i].handle, &setup->l4s[i].status, &cmdReply, setup->l4s[i].bufferQ, tfName, fname, fline))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->l4s[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->l4s[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): L4 setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
	}
	
	/* IP Teardown */
	for (i = 0; i < setup->nIps; i++)  {
		cmdReply.replyId = TF_COMMON_CMD_ID_DEL_IP + i;
		if (testCommonCloseL2L3 (tf, stats, &setup->ips[i].handle, &setup->ips[i].status, &cmdReply, setup->ips[i].bufferQ, tfName, fname, fline))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->ips[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->ips[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): IP setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else 
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
	}
	
    
	/* Custom L3 teardown */
	for (i = 0; i < setup->nCl3s; i++)  {
		cmdReply.replyId = TF_COMMON_CMD_ID_DEL_CL3 + i;
		if (testCommonCloseL2L3 (tf, stats, &setup->cl3s[i].handle, &setup->cl3s[i].status, &cmdReply, setup->cl3s[i].bufferQ, tfName, fname, fline))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->cl3s[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->cl3s[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): Custom L3 teardown failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
	}
    
			
	/* MAC teardown */
	for (i = 0; i < setup->nMac; i++)  {
        if(setup->macs[i].paret != pa_OK)continue;
		cmdReply.replyId = TF_COMMON_CMD_ID_DEL_MAC + i;
		if (testCommonCloseL2L3 (tf, stats, &setup->macs[i].handle, &setup->macs[i].status, &cmdReply, setup->macs[i].bufferQ, tfName, fname, fline))  {
			if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
				System_printf ("%s: (%s:%d - called from %s:%d): PA responses not complete\n", tfName, __FILE__, __LINE__, fname, fline);
			return (-1);
		}
		
		if (setup->macs[i].waitDone == TRUE)  {
			if (testCommonCheckWait (tf, setup, tfName, fname, fline, &setup->macs[i].status) == -1)
				System_printf ("%s: (%s:%d - called from %s:%d): MAC setup failed waiting for item %d to complete\n",
								tfName, __FILE__, __LINE__, fname, fline, i);
		}  else 
			testCommonCheckComplete (tf, setup, tfName, fname, fline);
	}
	
	if (testCommonWaitComplete (tf, setup, tfName, fname, fline) == FALSE)
		System_printf ("%s: (%s:%d - called from %s:%d): PA responses not complete\n", tfName, __FILE__, __LINE__, fname, fline);
	
	
	return (0);
	
}
	

	
/* Packets are taken off of the free descriptor Q and automatically returned after the transfer */
int testCommonSendPackets (tFramework_t *tf, char *tfName, paSysStats_t *stats, pktTestInfo_t *pktInfo, int nPackets, int dest)
{
	Cppi_HostDesc  *hd;
	Qmss_Queue      q;
	int i;
	
	
	for (i = 0; i < nPackets; i++)  {
		
		hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tf->QfreeDesc)) & ~15);
		if (hd == NULL)  {
			System_printf ("%s (%s:%d): Failed to pop a descriptor off free descriptor queue %d\n", 
							tfName, __FILE__, __LINE__, tf->QfreeDesc);
			return (-1);
		}
		
			/* Setup the return for the descriptor */
  		q.qMgr = 0;
  		q.qNum = tf->QfreeDesc;
  		Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
        
  	    /* Make sure there is no control info.  */
  	    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
  		
  		Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, pktInfo[i].pkt, (uint32_t)pktInfo[i].pktLen);
  		Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint32_t)pktInfo[i].pktLen);
  		
  		Qmss_queuePush (tf->QPaTx[dest + TF_PA_Q_CONFIG_BASE], (Ptr)hd, pktInfo[i].pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
  		
  		testCommonIncStats (pktInfo[i].statsMap, stats);
	}
	
	
	return (0);
}
	

int testCommonAllocUsrStats (tFramework_t *tf, char *tName, int numCnts, pauUsrStatsEntry_t *usrStatsTbl, paUsrStatsCounterEntryConfig_t *usrStatsCfgTbl)
{
    paUsrStatsAlloc_t usrStatsAlloc[TF_MAX_USR_STATS];
	paReturn_t      paret = pa_OK;
    int i, numAlloc;

    if (numCnts > TF_MAX_USR_STATS)
    {
	    System_printf ("%s (%s:%d): testCommonAllocUsrStats: numCnts (%d) exceeds the limit (%d)\n", 
	  	    			tName, __FILE__, __LINE__, numCnts, TF_MAX_USR_STATS);
    
        return (-1);
    }    
        
     /* Construct the user-defined statistics allocation table */
     memset(usrStatsAlloc, 0, sizeof(paUsrStatsAlloc_t)*numCnts);
     for (i = 0;  i < numCnts; i++)
     {
        if (!usrStatsTbl[i].fAlloc)
        {
            usrStatsAlloc[i].ctrlBitfield = pa_USR_STATS_ALLOC_CNT_PRESENT;
            usrStatsAlloc[i].cntIndex = usrStatsTbl[i].cntIndex;
        }
     
        if(usrStatsTbl[i].f64b)
            usrStatsAlloc[i].ctrlBitfield |= pa_USR_STATS_ALLOC_64B_CNT;
     }   
     
     numAlloc = numCnts;
     
     paret = Pa_allocUsrStats(tf->passHandle, &numAlloc, usrStatsAlloc);
     
     if ((paret != pa_OK) || (numAlloc < numCnts))
     {
	    System_printf ("%s (%s:%d): testCommonAllocUsrStats: Pa_allocUsrStats returns %d with %d user-defined stats allocated out of %d\n", 
	  	    			tName, __FILE__, __LINE__, paret, numAlloc, numCnts);
        return (-1);
     }
     
     /* Construct the usrStats Configuration table */
     for ( i = 0; i < numCnts; i++)
     {
        usrStatsCfgTbl[i].cntIndex = usrStatsAlloc[i].cntIndex;
        if (usrStatsTbl[i].fAlloc)
            usrStatsTbl[i].cntIndex = usrStatsAlloc[i].cntIndex;
        if(usrStatsTbl[i].lnkId != PAU_USR_STATS_NO_LINK)
            usrStatsCfgTbl[i].cntLnk = usrStatsAlloc[usrStatsTbl[i].lnkId].cntIndex;
        else
            usrStatsCfgTbl[i].cntLnk = pa_USR_STATS_LNK_END;
        usrStatsCfgTbl[i].cntType = usrStatsTbl[i].fByteCnt?pa_USR_STATS_TYPE_BYTE:pa_USR_STATS_TYPE_PACKET;              
     }
     
     return (0);   
}

int testCommonFreeUsrStats (tFramework_t *tf, char *tName, int numCnts, pauUsrStatsEntry_t *usrStatsTbl)
{
    uint16_t        usrStats[TF_MAX_USR_STATS];
	paReturn_t      paret = pa_OK;
    int i, numFree;

    if(numCnts > TF_MAX_USR_STATS)
    {
	    System_printf ("%s (%s:%d): testCommonFreeUsrStats: numCnts (%d) exceeds the limit (%d)\n", 
	  	    			tName, __FILE__, __LINE__, numCnts, TF_MAX_USR_STATS);
    
        return (-1);
    }    
        
     /* Construct the user-defined statistics allocation table */
     for (i = 0, numFree = 0;  i < numCnts; i++)
     {
        if (usrStatsTbl[i].fAlloc)
        {
            usrStats[numFree++] = usrStatsTbl[i].cntIndex;
        }
     }   
     
     paret = Pa_freeUsrStats(tf->passHandle, numFree, usrStats);
     
     if (paret != pa_OK)
     {
	    System_printf ("%s (%s:%d): testCommonAFreeUsrStats: Pa_freeUsrStats returns %d\n", 
	  	    			tName, __FILE__, __LINE__, paret);
        return (-1);
     }
     
     return (0);   
}

paTestStatus_t testCommonUsrStatsSetup (tFramework_t *tf, paTest_t *pat, char *tName, int numStatsGroup, pauUsrStatsSetup_t *usrStatsSetup, 
                                        uint32_t cmdId, int32_t qSource, int32_t qCmdRecycle, int32_t qCmdReply, Bool clear)
{
	int 			i;
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t		cmdSize;
    paUsrStatsCounterConfig_t   cntCfg;
    paUsrStatsConfigInfo_t      statsCfgInfo;
    
    paCmdReply_t cmdReply = {  pa_DEST_HOST,			/* Dest */
					           0,						/* Reply ID (returned in swinfo0) */
					   	       0,						/* Queue */
					           0 };						/* Flow ID */

	cmdReply.flowId= tf->tfFlowNum[0];

    statsCfgInfo.pCntCfg = &cntCfg;
    if (!clear)
    {
	    for (i = 0; i < numStatsGroup; i++)  {
		    cmdReply.replyId = cmdId + i;
		    cmdReply.queue   = (uint16_t)qCmdReply;
        
            memset(&cntCfg, 0, sizeof(cntCfg));
            cntCfg.numCnt = usrStatsSetup[i].nStats;
            cntCfg.cntInfo = usrStatsSetup[i].cntEntryTbl;
		
		    hd = testCommonConfigUsrStats (tf, &statsCfgInfo, qCmdRecycle, qSource, 
 	        	                           &cmdReply, &cmdDest, &cmdSize, &paret);
                                           
            if (paret != pa_OK)
            {
                if (paret != usrStatsSetup[i].paret)
                {
			        System_printf ("%s: (%s:%d): configUsrStats command (%d): unexpected err = %d (expected err = %d)\n", tName, 
                                __FILE__, __LINE__, i, paret, usrStatsSetup[i].paret);
		            return (PA_TEST_FAILED);
			    
                }
                continue;
            }
            else if (hd == NULL)  {
					
			    System_printf ("%s: (%s:%d): Failure in common configUsrStats command, entry number %d\n", tName, __FILE__, __LINE__, i);
		        return (PA_TEST_FAILED);
		    }	        				 
                                           
		    Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
                                           
	        if (testCommonWaitCmdReply (tf, pat, tName, cmdReply.queue, cmdId + i, __LINE__)) {
		        System_printf ("%s (%s:%d): testCommoncofigUsrStats failed\n", tName, __FILE__, __LINE__);
		        return (PA_TEST_FAILED);
 	        }
            
 	        /* Recycle the command packet as well */
 	        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (qCmdRecycle) & ~15));
 	        if (hd == NULL)  {
 		        System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tName, __FILE__, __LINE__);
 		        return(PA_TEST_FAILED);
 	        }
 	        testCommonRecycleLBDesc (tf, hd);
	    }
    }
    else
    {
		cmdReply.replyId = cmdId;
		cmdReply.queue   = (uint16_t)qCmdReply;
    
        memset(&cntCfg, 0, sizeof(cntCfg));
        cntCfg.ctrlBitfield = pa_USR_STATS_CONFIG_RESET;
        cntCfg.numCnt = 0;
        cntCfg.cntInfo = NULL;
	
	    hd = testCommonConfigUsrStats (tf, &statsCfgInfo, qCmdRecycle, qSource,
 		                               &cmdReply, &cmdDest, &cmdSize, &paret);
						 
        if (paret != pa_OK)
        {
	        System_printf ("%s: (%s:%d): configUsrStats command (%d): unexpected err = %d\n", tName, 
                        __FILE__, __LINE__, 0, paret);
	        return (PA_TEST_FAILED);
        }
        else if (hd == NULL)  {
			
	        System_printf ("%s: (%s:%d): Failure in common configUsrStats (clear) command\n", tName, __FILE__, __LINE__);
	        return (PA_TEST_FAILED);
	    }	        				 
						 
	    Qmss_queuePush (tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
        
	    if (testCommonWaitCmdReply (tf, pat, tName, cmdReply.queue, cmdId, __LINE__)) {
		    System_printf ("%s (%s:%d): testCommonConfigUsrStats failed \n", tName, __FILE__, __LINE__);
		    return (PA_TEST_FAILED);
 	    }
        
 	    /* Recycle the command packet as well */
 	    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (qCmdRecycle)) & ~15);
 	    if (hd == NULL)  {
 		    System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tName, __FILE__, __LINE__);
 		    return(PA_TEST_FAILED);
 	    }
 	    testCommonRecycleLBDesc (tf, hd);
    }
	
	return (PA_TEST_PASSED);
}

paTestStatus_t testCommonUsrStatsConfigReset (tFramework_t *tf, paTest_t *pat, char *tName, int numStatsGroup, pauUsrStatsSetup_t *usrStatsSetup, uint32_t cmdId, int32_t qSource, int32_t qCmdRecycle, int32_t qCmdReply)
{
    int i, j, k = 0;
    paUsrStatsCounterConfig_t   cntCfg;
    paTestStatus_t              ret;
    paUsrStatsTypes_e           prevVal[pa_USR_STATS_MAX_COUNTERS];
    
	for (i = 0, k = 0; i < numStatsGroup; i++)  {
    
        cntCfg.numCnt = usrStatsSetup[i].nStats;
        cntCfg.cntInfo = usrStatsSetup[i].cntEntryTbl;
        
        for (j = 0; j < cntCfg.numCnt; j++) {
            prevVal[k++]              = cntCfg.cntInfo[j].cntType;
            cntCfg.cntInfo[j].cntType = pa_USR_STATS_TYPE_DISABLE;    
        }
	}
    
    ret = testCommonUsrStatsSetup(tf, pat, tName, numStatsGroup, usrStatsSetup, cmdId, qSource, qCmdRecycle, qCmdReply, TRUE);

    /* Now restore the original control type for restartability */
   	for (i = 0, k = 0; i < numStatsGroup; i++)  {
        cntCfg.numCnt  = usrStatsSetup[i].nStats;
        cntCfg.cntInfo = usrStatsSetup[i].cntEntryTbl;

        for (j = 0; j < cntCfg.numCnt; j++) {
          cntCfg.cntInfo[j].cntType = prevVal[k++];
        }
    }

    return (ret);
}

/* Function used for debugging firmware */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_pa_ss.h>

#ifndef __LINUX_USER_SPACE
void mdebugHaltPdsp (Int pdspNum)
{

#ifndef NSS_GEN2

    CSL_Pa_ssRegs *passRegs;
#ifdef __LINUX_USER_SPACE
    passRegs = (CSL_Pa_ssRegs *)fw_passCfgVaddr;
#else
    passRegs = (CSL_Pa_ssRegs *)CSL_NETCP_CFG_REGS;
#endif
	passRegs->PDSP_CTLSTAT[pdspNum].PDSP_CONTROL &= ~(CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK);
#endif

}

void mdebugRunPdsp (int pdspNum)
{
    //CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)CSL_PA_SS_CFG_REGS; 
	//passRegs->PDSP_CTLSTAT[pdspNum].PDSP_CONTROL |= (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK);

}
#else
#include "fw_test.h"
void mdebugHaltPdsp (int pdspNum)
{
    //CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)fw_passCfgVaddr; 
	//passRegs->PDSP_CTLSTAT[pdspNum].PDSP_CONTROL &= ~(CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK);

}

void mdebugRunPdsp (int pdspNum)
{
    //CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)fw_passCfgVaddr; 
	//passRegs->PDSP_CTLSTAT[pdspNum].PDSP_CONTROL |= (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK);

}
#endif
  	
		
	

