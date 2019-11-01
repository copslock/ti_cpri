/**
 *   @file  reassemLib.c
 *
 *   @brief   
 *      The file contains the PASS-assisted IP Reassembly sample code
 *      which demonstrates how to interact with the NetCP to perform 
 *      IPv4 reassembly operation. 
 *
 *      The sample code implements a simplified version of IP reassembly algorithm which supports non-overlapping segments only. The sample 
 *      code performs the following tasks:
 *      @li Maintain the IP reassembly contexts consist of source IP, destination IP, IP identification, protocol, fragments count and the 
 *          corresponding traffic flow id.
 *      @li Forward the non-fragmented IP packet with its flow id and count = 1 to PA PDSP queue. This avoids reordering the non-fragmented packets.
 *      @li For IPSEC inner IP fragments, call SA LLD to perform the post-decryption operation including padding check and IPSEC header 
 *          and authentication tag removal.
 *      @li Forward the reassembled IP packet with its flow id and fragments count to PA PDSP queue.
 *      @li Send a null packet with its flow id and fragments count to PA PDSP queue if the fragments are discarded due to timeout or other error.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009-2013 Texas Instruments, Inc.
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
 *  \par
*/

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <reassemLib.h>

/**************************************************************************
 ***************** IP Version 4 Protocol Definitions **********************
 **************************************************************************/
/* IPV4 byte offsets to fields */
#define IPV4_OFFSET_VER_HLEN       0
#define IPV4_OFFSET_TOS            1
#define IPV4_OFFSET_LEN            2
#define IPV4_OFFSET_ID             4
#define IPV4_OFFSET_FLAGS_FRAGO    6
#define IPV4_OFFSET_TTL            8
#define IPV4_OFFSET_PROTO          9
#define IPV4_OFFSET_HDR_CHKSUM     10
#define IPV4_OFFSET_SRC_ADDR       12
#define IPV4_OFFSET_DEST_ADDR      16

/* IPV4 definitions */
#define IPV4_VER_MASK              0xF0
#define IPV4_VER_VALUE             0x40
#define IPV4_HLEN_MASK             0x0F
#define IPV4_HLEN_SHIFT            0
#define IPV4_FRAGO_MASK            0x1FFF
#define IPV4_FLAGS_MF_MASK         0x2000
#define IPV4_FLAGS_DF_MASK         0x4000
#define IPV4_FRAG_DET_MASK         (IPV4_FLAGS_MF_MASK | \
                                    IPV4_FRAGO_MASK)

#define IPV4_HDR_MIN_SIZE          20

#define IPV4_FRAG_TIME	           30000    /* fragment default lifetime in milliseconds */

/* IP Macros */
#define IPV4_READ_VER(x)                ((x[IPV4_OFFSET_VER_HLEN] & IPV4_VER_MASK)  >> 4)
#define IPV4_READ_IHL(x)                ((x[IPV4_OFFSET_VER_HLEN] & IPV4_HLEN_MASK) << 2)
#define IPV4_READ_PROTO(x)              x[IPV4_OFFSET_PROTO]
#define IPV4_READ_LEN(x)                ((x[IPV4_OFFSET_LEN] << 8) + x[IPV4_OFFSET_LEN+1])
#define IPV4_WRITE_LEN(x, len)          x[IPV4_OFFSET_LEN] = (len) >> 8,               \
                                        x[IPV4_OFFSET_LEN + 1] = (len) & 0xFF
#define IPV4_READ_ID(x)                 ((x[IPV4_OFFSET_ID] << 8) + x[IPV4_OFFSET_ID+1])
#define IPV4_WRITE_CKSUM(x, cksum)      x[IPV4_OFFSET_HDR_CHKSUM] = (cksum) >> 8,     \
                                        x[IPV4_OFFSET_HDR_CHKSUM + 1] = (cksum) & 0xFF
#define IPV4_READ_FLAGS_FRAGO(x)        ((x[IPV4_OFFSET_FLAGS_FRAGO] << 8) + x[IPV4_OFFSET_FLAGS_FRAGO + 1])
#define IPV4_WRITE_FLAGS_FRAGO(x, v)    x[IPV4_OFFSET_FLAGS_FRAGO] = (v) >> 8,     \
                                        x[IPV4_OFFSET_FLAGS_FRAGO+1] = (v) & 0xFF

#define IPV4_READ_SRC_ADDR(x)           (x[IPV4_OFFSET_SRC_ADDR] << 24) | (x[IPV4_OFFSET_SRC_ADDR+1] << 16) | (x[IPV4_OFFSET_SRC_ADDR+2] << 8) | x[IPV4_OFFSET_SRC_ADDR+3]
#define IPV4_READ_DEST_ADDR(x)          (x[IPV4_OFFSET_DEST_ADDR] << 24) | (x[IPV4_OFFSET_DEST_ADDR+1] << 16) | (x[IPV4_OFFSET_DEST_ADDR+2] << 8) | x[IPV4_OFFSET_DEST_ADDR+3]

#define FRAG_FIRST                  0x1
#define FRAG_MIDDLE                 0x2
#define FRAG_LAST                   0x4


/* Protocol field values (IPV4) / next header (IPV6) */
#define IP_PROTO_NEXT_IPV6_HOP_BY_HOP    0   /* IPv6 extension header - hop by hop */
#define IP_PROTO_NEXT_IPV6_ROUTE        43   /* IPv6 extension header - route      */
#define IP_PROTO_NEXT_IPV6_FRAG         44   /* IPv6 extension header - fragmentation */
#define IP_PROTO_NEXT_IPV6_NO_NEXT      59   /* IPv6 extention header - no next header */     
#define IP_PROTO_NEXT_IPV6_DEST_OPT     60   /* IPv6 extension header - destination options */


/** 
 * @brief IP Reassembly Context
 */
typedef struct paIPReassemblyCxt_s
{
    /* IP reassembly context identifier */
    uint8_t         protocol;       
    uint32_t        id;
    paEx_IPAddr        srcAddr;
    paEx_IPAddr        destAddr;
    Qmss_QueueHnd   destQ;              /* destination queue */
    
    /* PASS Traffic flow related variables */
    uint16_t        tfId;               /* Traffic Flow Id */
    uint16_t        fragCnt;            /* number of fragments so far */
     
    /* IP Reassembly parameters */ 
    uint16_t        originalLen;
    uint16_t        detectedLen;
    
    uint32_t        timeout;
    
    Cppi_HostDesc*  pHostDesc;          /* pointer to the first descriptor */
    
    struct paIPReassemblyCxt_s  *next;  /* pointer to the next context in the list */      
} paIPReassemblyCxt_t;

/** 
 * @brief IP Reassembly Master Control Block
 */
typedef struct paIPReassemblyMCB_s
{
    paIPReassemblyCxt_t *freeList;      /* IP Reassembly Block Free List */
    paIPReassemblyCxt_t *activeList;    /* IP Reassembly Block Active List */
    
    /* configuration */
    uint16_t     descSize;              /* CPPI descriptor size */
    uint16_t     timeout;               /* Reassembly context timeout in seconds */
    
    /* Statistics */
    paIPReassemblyStats_t  stats;          
    
} paIPReassemblyMCB_t;

/** 
 * @brief The number of IP reassembly context blocks supported
 */
#define MAX_IP_REASSEMBLY_CONTEXTS      100

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/
/* IP Reassembly Information Blocks. */
paIPReassemblyCxt_t     IPReassemblyInfo[MAX_IP_REASSEMBLY_CONTEXTS];

/* IP Reassembly Control Block */
paIPReassemblyMCB_t     paIPReassemblyMCB;

/**************************************************************************
 ************************** Extern Variables ******************************
 **************************************************************************/

/**************************************************************************
 **********************  LIST Utility Functions ***************************
 **************************************************************************/
/** 
 *  @b Description
 *  @n  
 *      The function is called to add a entry to the beginning of the list.
 *
 *  @param[in]  pplist
 *      This is the pointer to the list to which the node is to be added. 
 *  @param[in]  entry
 *      This is the node which is to be added.
 *
 *  @retval
 *      Not Applicable
 */
static void listInsert(paIPReassemblyCxt_t **pplist,  paIPReassemblyCxt_t *entry)
{
    paIPReassemblyCxt_t *temp = *pplist;
    
    entry->next = temp;
    *pplist = entry;
}


/**
 *  @b Description
 *  @n  
 *      The function is called to remove the specified node from the list. 
 *
 *  @param[in]  pplist
 *      This is the pointer to the list from where node will be removed.
 *  @param[in]  entry
 *      This is the node which is to be removed.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  -1
 */
static int listRemove (paIPReassemblyCxt_t **pplist,  paIPReassemblyCxt_t *entry)
{
    paIPReassemblyCxt_t *next = *pplist;
    paIPReassemblyCxt_t *prev = NULL;

    
    Bool found = paEX_FALSE;

    /* Are there any nodes in the list? */
    if (next == NULL)
		return -1;

    while (next)
    {
        if (next == entry)
        {
            found = paEX_TRUE;
            
            if (prev)
            {
                prev->next = next->next;
            }
            else
            {
                /* It is the fisrt entry in the list */
                *pplist = next->next;
            }
            
            break;
        }
        
        prev = next;
        next = next->next;
    }

    return (found ? 0 : -1);
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to get an entry from the beginning of the list.
 *
 *  @param[in]  pplist
 *      This is the pointer to the list from where the first node is returned.
 *
 *  @retval
 *      Pointer to the first node
 *       
 */
static  paIPReassemblyCxt_t* listGet(paIPReassemblyCxt_t **pplist)
{
    paIPReassemblyCxt_t *next = *pplist;
    
    if (next != NULL)
    {
        *pplist = next->next;
    }
    
    return (next);
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to find the entry of the specific IP reassembly block 
 *
 *  @param[in]  pplist
 *      This is the pointer to the list to ne searched.
 *  @param[in]  protocol
 *      This is the protocol at the IP header.
 *  @param[in]  id
 *      This is the identification at the IP header.
 *  @param[in]  saddr
 *      This is the source IP address.
 *  @param[in]  daddr
 *      This is the destination IP address.
 *  @param[in]  destQ
 *      Destination queue of the reassembled or non-fragment packets
 *
 *  @retval
 *      Pointer to the IP Reassembly block which matches all the input criteria
 */
static  paIPReassemblyCxt_t* listFind(
    paIPReassemblyCxt_t** pplist, 
    uint8_t                 protocol, 
    uint32_t                id, 
    paEx_IPAddr*           saddr, 
    paEx_IPAddr*           daddr, 
    int32_t                 destQ
)
{
    paIPReassemblyCxt_t *next = *pplist;
    
     if (saddr->ver == paEx_IPVersion_IPV4)
    {
        while (next != NULL)
        {
            if ((next->id == id)                                          &&
               (next->srcAddr.addr.ipv4.u.a32  == saddr->addr.ipv4.u.a32)  &&
               (next->destAddr.addr.ipv4.u.a32 == daddr->addr.ipv4.u.a32)  &&
               (next->protocol == protocol)                               &&
               (next->destQ == destQ))
            {
                /* entry is found */
                return(next);
            }   
            
            /* check the next entry in the list */
            next = next->next;
        }
    }
    else
    {
        while (next != NULL)
        {
            if ((next->id == id)                                                  &&
               (next->srcAddr.addr.ipv6.u.a32[0] == saddr->addr.ipv6.u.a32[0])    &&
               (next->srcAddr.addr.ipv6.u.a32[1] == saddr->addr.ipv6.u.a32[1])    &&
               (next->srcAddr.addr.ipv6.u.a32[2] == saddr->addr.ipv6.u.a32[2])    &&
               (next->srcAddr.addr.ipv6.u.a32[3] == saddr->addr.ipv6.u.a32[3])    &&
               (next->destAddr.addr.ipv6.u.a32[0] == daddr->addr.ipv6.u.a32[0])   &&
               (next->destAddr.addr.ipv6.u.a32[1] == daddr->addr.ipv6.u.a32[1])   &&
               (next->destAddr.addr.ipv6.u.a32[2] == daddr->addr.ipv6.u.a32[2])   &&
               (next->destAddr.addr.ipv6.u.a32[3] == daddr->addr.ipv6.u.a32[3])   &&
               (next->protocol == protocol)                                       &&
               (next->destQ    == destQ))
            {
                /* entry is found */
                return(next);
            }   
            
            /* check the next entry in the list */
            next = next->next;
        }
    }
    
    return (next);
}

/**************************************************************************
 ***************IP Checksum  Utility Functions ****************************
 **************************************************************************/
/** 
 *  @b Description
 *  @n  
 *      The function return one's complement sum of two input values 
 *
 *  @param[in]  v1
 *      Input value1
 *  @param[in]  v2
 *      Input value2
 *  @retval
 *      One's complement sum of two input values
 */
static uint16_t utilOnesComplementAdd (uint16_t v1, uint16_t v2)
{
  uint32_t result;

  result = (uint32_t)v1 + (uint32_t)v2;
  result = (result >> 16) + (result & 0xffff);
  result = (result >> 16) + (result & 0xffff);

  return ((uint16_t)result);
}

/** 
 *  @b Description
 *  @n  
 *      The function return one's complement sum of input array 
 *
 *  @param[in]  p
 *      pointer to the input 16-bit data array
 *  @param[in]  nwords
 *      Size of the input array
 *  @retval
 *      One's complement sum of input array
 */
static uint16_t utilOnesCompChkSum (uint8_t *p, int nwords)
{
  uint16_t chksum = 0;
  uint16_t v;
  uint32_t i;
  uint32_t j;

  for (i = j = 0; i < nwords; i++, j+=2)  {
    v = (p[j] << 8) | p[j+1];
    chksum = utilOnesComplementAdd (chksum, v);
  }

  return (chksum);

} 

/** 
 *  @b Description
 *  @n  
 *      The function is used to calculate, update and return IPv4 header checksum
 *
 *  @param[in]  ipHdr
 *      pointer to the IPv4 header
 *  @retval
 *      IPv4 checksum
 */
static uint16_t utilUpdateIpChksums (uint8_t  *ipHdr)
{
	uint16_t  sum;
	
	/* reset the checksum field to zero */
    IPV4_WRITE_CKSUM(ipHdr, 0);
    
	sum = ~utilOnesCompChkSum (ipHdr, 10);
	
    IPV4_WRITE_CKSUM(ipHdr, sum);
    
	return (sum);
}

/**************************************************************************
 ************** Host Descriptor Processing  Utility Functions *************
 **************************************************************************/
/**
 *  @b Description
 *  @n  
 *      The function recycles the descriptor chain.
 *
 *  @param[in]  pHostDesc
 *      Descriptor which holds the received packet
 *
 *  @retval
 *      Not Applicable.
 */
static void utilRecycleDesc(Cppi_HostDesc* pHostDesc)
{
    Qmss_Queue   qmssQueue;

    /* Cycle through all the linked descriptors and clean them out. */
    while (pHostDesc != NULL)
    {
        /* Get the return queue from the descriptor. */
        qmssQueue = Cppi_getReturnQueue (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc);

        /* Push the descriptor to the specified return queue. */
        Qmss_queuePushDesc(Qmss_getQueueHandle(qmssQueue), (void*)pHostDesc);

        /* Now get the next chained descriptor. */
        pHostDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc);
    }
    return;
}

#ifdef _INCLUDE_SA
/**************************************************************************
 ******************* IPESC Processing Utility Functions *******************
 **************************************************************************/
/**
 *  @b Description
 *  @n  
 *      The function extracts the SA LLD channel handle from the packet descriptor.It is up to the 
 *      application to provide a mechanism to derive the channel handle from the CPPI swInfo[2].
 *      For example, we can configure the swInfo1 at Sa_DestInfo_t to be the desired SA channel handle.
 *
 *  @param[in]  pHostDesc
 *      Descriptor which holds the received packet
 *
 *  @retval
 *      SA LLD channel handle
 *
 */
static Sa_ChanHandle ipsecGetChanHndl(Cppi_HostDesc *pHostDesc)
{
	uint32_t         *swinfo;

	Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t **)&swinfo);

    return((Sa_ChanHandle)swInfo[1]);
}

/**
 *  @b Description
 *  @n  
 *      The function invoke the SA LLD sendData API to performs IPSEC post-decryption processing 
 *
 *  @param[in]  pHostDesc
 *      Descriptor which holds the received packet
 *
 *  @param[in]  pInfo
 *      Pointer to the PA packet info data
 *  
 *  @param[out] pOffsetAdjust
 *      IP header offset adjustment due to the IPSEC post-decryption processing 
 *
 *  @retval
 *      Not Applicable.
 *
 */
static void ipsecPostProcessing(Cppi_HostDesc    *pHostDesc, 
                                pasahoLongInfo_t *pInfo, 
                                unit16_t         *pOffsetAdjust)
{
    Sa_PktInfo_t  pktInfo;
    Sa_PktDesc_t* pPktDesc = &pktInfo.pktDesc;
    uint16_t ipOffset = PASAHO_LINFO_READ_L3_OFFSET(pInfo);
    uint32_t origHdrLoc;
    Sa_ChanHandle saHndl;
    
    void*     segments[2];
    uint16_t  segUsedSizes[2];
    uint16_t  segAllocSizes[2];
    
    /* Initialize the SA LLD packet descriptor */
    pktDesc->nSegments = 1;
    pktDesc->segments = segments;
    pktDesc->segUsedSizes = segUsedSizes;
    pktDesc->segAllocSizes = segAllocSizes;
    
    /* Populate the SA LLD packet descriptor based on the input packet */
    pPktDesc->size = pHostDesc->buffLen;
    pPktDesc->segments[0] = (void *)pHostDesc->buffPtr; 
    pPktDesc->segUsedSizes[0] = pPktDesc->size;
    pPktDesc->segAllocSizes[0] = pHostDesc->origBufferLen;
    origHdrLoc = (uint32_t)pPktDesc->segments[0];
    pktInfo.validBitMap = 0;
    
    pPktDesc->payloadOffset = ipOffset;       
    pPktDesc->payloadLen    = pPktDesc->size - pPktDesc->payloadOffset;
        
    /* Call SA LLD API to perform Protocol Specific Operation */
    saHndl = ipsecGetChanHndl(pHostDesc);
    Sa_chanReceiveData(saHndl, &pktInfo);
    
    /* Adjust per payload updates */
    *pOffsetAdjust = (uint16_t)((uint32_t)pPktDesc->segments[0] - origHdrLoc);
    
    hd->buffPtr = (uint32_t)pPktDesc->segments[0];
    hd->buffLen = pPktDesc->payloadOffset + pPktDesc->payloadLen;
    
    PASAHO_LINFO_SET_START_OFFSET(pInfo, PASAHO_LINFO_READ_START_OFFSET(pInfo) - *pOffsetAdjust);
    
    PASAHO_LINFO_CLR_IPSEC(pInfo);
    
}

#endif

/**
 *  @b Description
 *  @n  
 *      The function remove the IP reassembly Block from the active list, clean up its cotents and
 *      insert it into the freeList.  
 *
 *  @retval
 *      Not Applicable
 *
 */
static void paEx_initReassemblyBlock(paIPReassemblyCxt_t *pReassemblyInfo)
{
    listRemove(&paIPReassemblyMCB.activeList, pReassemblyInfo); 
    memset(pReassemblyInfo, 0, sizeof(paIPReassemblyCxt_t));
    listInsert(&paIPReassemblyMCB.freeList, pReassemblyInfo);
}

/**
 *  @b Description
 *  @n  
 *      The function initializes the IP reassembly Master Control Block and resources. It should
 *      be invoked before the PASS-assisted reassembly operation is enabled.
 *
 *  @param[in]  pConfig
 *      Pointer to the IP Reassembly configuration structure
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  @note This fuction may be modified to allocate the IP reassembly blocks from memory heap and/or
 *        performs other OS related functions
 */
int paEx_reassemLibInit(paIPReassemblyConfig_t *pConfig)
{
    int i;
    
    if (pConfig->numReassemblyContexts > MAX_IP_REASSEMBLY_CONTEXTS)
    {
        return (-1);
    }
    
    /* 
     * Call dynamic memory allocation function to allocate the IP reassembly 
     * blocks if desired.
     */
    
    /* Initialize the IP Reassembly Info Blocks. */
    memset ((void*)&IPReassemblyInfo, 0, sizeof(IPReassemblyInfo));
    memset ((void*)&paIPReassemblyMCB, 0, sizeof(paIPReassemblyMCB_t));
    
    /* Initialize the freeList */
    for (i = 0; i < pConfig->numReassemblyContexts; i++)
    {
        listInsert(&paIPReassemblyMCB.freeList, &IPReassemblyInfo[i]);
    }
    
    paIPReassemblyMCB.descSize = pConfig->descSize;
    paIPReassemblyMCB.timeout  = pConfig->timeout;
    
    return 0;
}


/**
 *  @b Description
 *  @n  
 *      The function runs through the reassembly information blocks and
 *      times out entries if they have expired.
 *      It should be invoked by the application every second at least.
 *
 *  @param[in]  timeElapsed
 *      The elapsed time since the previous call
 *
 *
 *  @retval
 *      Not Applicable.
 */
void paEx_reassemLibTimerTick(uint32_t timeElapsed)
{

    paIPReassemblyCxt_t *next = paIPReassemblyMCB.activeList;
    paIPReassemblyCxt_t *prev = NULL;
    
    while (next)
    {
        if (next->timeout <= timeElapsed)
        {
            
            paIPReassemblyCxt_t* pReassemblyInfo = next;
            if (pReassemblyInfo->tfId != PA_INV_TF_INDEX)
            {
	            Cppi_HostDesc  		*hd = pReassemblyInfo->pHostDesc;
	            pasahoLongInfo_t 	*pInfo;
	            uint32_t        	 infoLen;
            
                /* Prepare and send a null packet */
		        /* Get and update the packet context */
		        Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pInfo, &infoLen);
                PASAHO_LINFO_SET_FRANCNT(pInfo, next->fragCnt);
                PASAHO_LINFO_SET_NULL_PKT_IND(pInfo, 1);
                
                /* send the packet out */
  		        Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);
  	            Qmss_queuePush (next->destQ, hd, 0, paIPReassemblyMCB.descSize, Qmss_Location_TAIL);
            
            }
            else
            {
                /* There is no associated traffic flow, just free the descriptors */
                utilRecycleDesc(pReassemblyInfo->pHostDesc);
            }
            
            paIPReassemblyMCB.stats.reassemblyTimeout++;
            
            if (prev == NULL)
            {
                /* This is the first entry in the activeList */
                paIPReassemblyMCB.activeList = pReassemblyInfo->next;
            }
            else
            {
                /* There is still entry in the list */
                prev->next = next->next;
            }
            next = next->next;
            
            
            /* free this entry */
            memset(pReassemblyInfo, 0, sizeof(paIPReassemblyCxt_t));
            listInsert(&paIPReassemblyMCB.freeList, pReassemblyInfo);
        } 
        else
        {            
            next->timeout -= timeElapsed;
            /* move to next entry in the list */            
            prev = next; 
            next = next->next; 
        }
    }
}

/**
 *  @b Description
 *  @n  
 *      The function returns the IP reassembly statistics to user buffer and reset
 *      all statistics if doClear is set
 *  @param[in/out]  pStats
 *      The pointer to the Reassembly statistics
 *  @param[in]      doClear
 *      if set, reset all statistics after query
 *
 *  @retval
 *      Not Applicable.
 */
void paEx_reassemLibQueryStats(paIPReassemblyStats_t *pStats, int doClear)
{
    memcpy(pStats, &paIPReassemblyMCB.stats, sizeof(paIPReassemblyStats_t));
    if(doClear)
        memset(&paIPReassemblyMCB.stats, 0, sizeof(paIPReassemblyStats_t));
}

/**
 *  @b Description
 *  @n  
 *      The function returns the IP reassembly block for this IP fragment.
 *      It search the active list at first.
 *      If there is no active reassembly block matching this IP fragement, a 
 *      new control block will be allocated.
 *      If there is none found the function returns NULL.
 *
 *  @param[in]  ipHdr
 *      Pointer to the IP header
 *  @param[in]  destQ
 *      Destination queue of the reassembled or non-fragment packets
 *
 *  @retval
 *      Success -   IP Reassembly block to be used.
 *  @retval
 *      Error   -   NULL
 */
static paIPReassemblyCxt_t* paEx_getIPReassemblyBlock(uint8_t *ipHdr, paEx_IPVersion ipVer, uint16_t fragHdrOffset, int32_t destQ)
{
    paIPReassemblyCxt_t *pIPReassemblyInfo;
    paEx_IPAddr saddr; 
    paEx_IPAddr daddr; 
    uint8_t  protocol;
    uint32_t id;
    //uint32_t saddr = IPV4_READ_SRC_ADDR(ipHdr);
    //uint32_t daddr = IPV4_READ_DEST_ADDR(ipHdr);
    
    /* Get the pointer to the IP header. */
    if (ipVer == paEx_IPVersion_IPV4)
    {
        paEx_IPHeader* ipv4Hdr;

        /* Extract the reassembly context information from the IP header. */
        ipv4Hdr                = (paEx_IPHeader*)(ipHdr);
        protocol               = IPV4_READ_PROTO(ipHdr);
        id                     = IPV4_READ_ID(ipHdr);
        saddr.ver              = paEx_IPVersion_IPV4;
        saddr.addr.ipv4.u.a32  = (ipv4Hdr->IPSrc[0] << 24) | (ipv4Hdr->IPSrc[1] << 16) | (ipv4Hdr->IPSrc[2] << 8) | ipv4Hdr->IPSrc[3];
        daddr.ver              = paEx_IPVersion_IPV4;
        daddr.addr.ipv4.u.a32  = (ipv4Hdr->IPDst[0] << 24) | (ipv4Hdr->IPDst[1] << 16) | (ipv4Hdr->IPDst[2] << 8) | ipv4Hdr->IPDst[3];
    }
    else
    {
        paEx_IPv6Header*     ipv6Hdr;
        paEx_IPv6FragHeader* ipv6FragHdr;
        uint32_t              index;

        /* Extract the reassembly context information from the IP header. */
        ipv6Hdr      = (paEx_IPv6Header*)(ipHdr);
        ipv6FragHdr  = (paEx_IPv6FragHeader*)(ipHdr + fragHdrOffset);
        protocol     = ipv6FragHdr->NextHeader;
        id           = (ipv6FragHdr->FragId[0] << 24) | (ipv6FragHdr->FragId[1] << 16) | (ipv6FragHdr->FragId[2] << 8) | ipv6FragHdr->FragId[3];
        saddr.ver    = paEx_IPVersion_IPV6;
        for (index = 0; index < 16; index++)
            saddr.addr.ipv6.u.a8[index] = ipv6Hdr->SrcAddr.u.a8[index];  
        daddr.ver    = paEx_IPVersion_IPV6;
        for (index = 0; index < 16; index++)
            daddr.addr.ipv6.u.a8[index] = ipv6Hdr->DstAddr.u.a8[index];
    }

    /* Search the active list */
    if((pIPReassemblyInfo = listFind(&paIPReassemblyMCB.activeList, protocol, id, &saddr, &daddr, destQ)))
        return(pIPReassemblyInfo);
    
    /* There is no associated reassembly block, allocate a new block from free list */
    if ((pIPReassemblyInfo = listGet(&paIPReassemblyMCB.freeList)))
    {
        /* Initialize the new reassembly block */
        pIPReassemblyInfo->protocol = protocol;
        pIPReassemblyInfo->id = id;
        memcpy (&pIPReassemblyInfo->srcAddr, &saddr, sizeof (paEx_IPAddr));
        memcpy (&pIPReassemblyInfo->destAddr, &daddr, sizeof (paEx_IPAddr));
        //pIPReassemblyInfo->srcAddr = saddr;
        //pIPReassemblyInfo->destAddr = daddr;
        pIPReassemblyInfo->destQ  = destQ;
        
        pIPReassemblyInfo->tfId = PA_INV_TF_INDEX;
        pIPReassemblyInfo->timeout = paIPReassemblyMCB.timeout;
        
        /* Insert the new block into the active list */
        listInsert(&paIPReassemblyMCB.activeList, pIPReassemblyInfo);
    }

    return pIPReassemblyInfo; 
}

/**
 *  @b Description
 *  @n  
 *      The function performs IP reassembly operation 
 *
 *  @param[in]  pHostDesc
 *      Descriptor which holds the received packet
 *  @param[in]  tfIndex
 *      PASS Traffic flow index
 *  @param[in]  destQ
 *      Destination queue of the reassembled or non-fragment packets
 *  @param[in]  ipOffset
 *      Offset to the IPv4 header
 *  @param[in]  fragOffset
 *      fragment Offset in the IPv4 header
 *  @param[out]  ppReassemblyBlock
 *      pointer to the IP Reassembly block pointer. (set to NULL if still waiting for fragments)
 *
 *  @retval
 *      0       - Success
 *  @retval
 *      <0      - Error
 */
 
int paEx_ipReassembly (
    Cppi_HostDesc*          pHostDesc, 
    uint16_t                tfIndex,
    int32_t                 destQ,
    uint16_t                ipOffset, 
    uint16_t                fragOffset,
    uint16_t                nextHdrOffset,
    uint16_t                fragHdrOffset,
    paEx_IPVersion          ipVer,
    paIPReassemblyCxt_t**   ppReassemblyBlock
)
{
    paIPReassemblyCxt_t* pIPReassemblyBlock;
    uint32_t            dataLen, ipHdrLen;
    uint32_t            fragmentType;
    uint8_t*            pFragmentData;
    uint8_t*            ipHdr = (uint8_t *)(pHostDesc->buffPtr + ipOffset);

    /* Initialize the pointer to the Reassembly block to NULL */
    *ppReassemblyBlock = NULL; 

    /* Get the reassembly block. */
    pIPReassemblyBlock = paEx_getIPReassemblyBlock(ipHdr, ipVer, fragHdrOffset, destQ);
    if (pIPReassemblyBlock == NULL)
        return -1;

    /* Increment the number of fragments received. */
    paIPReassemblyMCB.stats.fragmentsRxed++;
    
    /* Update the traffic flow Index and fragment count */
    if (pIPReassemblyBlock->tfId == PA_INV_TF_INDEX)
    {
        pIPReassemblyBlock->tfId = tfIndex;    
    }
    else if (pIPReassemblyBlock->tfId != tfIndex)
    {
        return (-1);
    }
    
    if(pIPReassemblyBlock->tfId != PA_INV_TF_INDEX)
        pIPReassemblyBlock->fragCnt++;

    if (ipVer == paEx_IPVersion_IPV4)
    {
        /* Extract the IP lenth including the IP header */
        dataLen = IPV4_READ_LEN(ipHdr);
        ipHdrLen = IPV4_READ_IHL(ipHdr);

        /* Determine if the packet is FIRST, MIDDLE or LAST Fragment. */
        if ((fragOffset & IPV4_FLAGS_MF_MASK) == 0)
            fragmentType = FRAG_LAST;
        else if ((fragOffset & IPV4_FRAGO_MASK) == 0)
            fragmentType = FRAG_FIRST;
        else
            fragmentType = FRAG_MIDDLE;

        /* Get the fragmentation offset now. We can remove the flags. */
        fragOffset = (fragOffset & IPV4_FRAGO_MASK) << 3;

        /* The LAST Fragment is used to determine the length of the original packet. */
        if (fragmentType == FRAG_LAST)
            pIPReassemblyBlock->originalLen = dataLen + fragOffset;

        /* Is this the first fragment? */
        if (fragmentType == FRAG_FIRST)
        {
            /* The first fragment needs to include the complete header. */
            /* remove the potential L2 padding */
            if(pHostDesc->buffLen > (ipOffset + dataLen))
                pHostDesc->buffLen = ipOffset + dataLen;    
        }
        else
        {
            /* Middle and Last Fragments need to skip the IPv4 header */
            pFragmentData = ((uint8_t*)(ipHdr)) + ipHdrLen;
            dataLen         = dataLen - ipHdrLen;
            
            /* Modify the data buffers in the descriptors. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, pFragmentData, dataLen);
            
        }
    }
    else
    {
        dataLen  = pktRead16bits(ipHdr, PA_FIELDOFFSET(paEx_IPv6Header, PayloadLength));
        dataLen -= (fragHdrOffset - IPv6HDR_SIZE + IPV6_FRAGHDR_SIZE);
        /* Determine if the packet is FIRST, MIDDLE or LAST Fragment. */
        if ((fragOffset & IPV6_FLAGS_MF_MASK) == 0)
            fragmentType = FRAG_LAST;
        else if ((fragOffset & IPV6_FRAGO_MASK) == 0)
            fragmentType = FRAG_FIRST;
        else
            fragmentType = FRAG_MIDDLE;

        /* Get the fragmentation offset now. We can remove the flags. */
        fragOffset = fragOffset & IPV6_FRAGO_MASK;

        /* The LAST Fragment is used to determine the length of the original packet. */
        if (fragmentType == FRAG_LAST)
            pIPReassemblyBlock->originalLen = dataLen + fragOffset;

        /* Is this the first fragment? */
        if (fragmentType == FRAG_FIRST)
        {
            uint8_t* ipv6FragHdr;
            uint8_t  tmpNextHdr;
            /* The first fragment needs to include all the headers except the
             * fragment extension header. */
            pFragmentData = (uint8_t*)pHostDesc->buffPtr + IPV6_FRAGHDR_SIZE;
            /* Update the next header field of the fixed IP header. */
            ipv6FragHdr   = ((uint8_t*)ipHdr + fragHdrOffset);
            
            tmpNextHdr = pktRead8bits(ipv6FragHdr, PA_FIELDOFFSET(paEx_IPv6FragHeader, NextHeader));
            pktWrite8bits(ipHdr, nextHdrOffset, tmpNextHdr);

            /* Shift the headers to overwrite the fragment header. */
            memmove ((void*) pFragmentData, (void*)pHostDesc->buffPtr, ipOffset + fragHdrOffset);

            /* Modify the data buffer pointer and length in the descriptor. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, pFragmentData, dataLen + ipOffset + fragHdrOffset);
        }
        else
        {
            /* Middle and Last Fragments need to skip the IP fixed header
             * and fragment headers. */
            pFragmentData = ((uint8_t*)(ipHdr)) + fragHdrOffset + IPV6_FRAGHDR_SIZE;
            
            /* Modify the data buffers in the descriptors. */
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, pFragmentData, dataLen);
        }
    }

    /* Set the fragmentation offset into the descriptor. */
    Cppi_setSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, fragOffset);


    /* TODO: We need to override the descriptor return queue also at this point in time. */

    /* Ensure that the BD is inserted into the correct location in the list. */
    if (pIPReassemblyBlock->pHostDesc == NULL)
    {
        /* LIST is empty. So we remember the current descriptor. */
        pIPReassemblyBlock->pHostDesc = pHostDesc;
    }
    else
    {
        Cppi_HostDesc*  pListHostDesc;
        Cppi_HostDesc*  pPrevListHostDesc;
        uint16_t        listFragOffset;

        /* Start from the head of the descriptor chain */
        pListHostDesc = pIPReassemblyBlock->pHostDesc;

        /* Set the previous list descriptor to NULL. */
        pPrevListHostDesc = NULL;

        /* Cycle through the chained packets to determine where we can place the received packet */
        while (pListHostDesc != NULL)
        {
            /* Get the fragmentation offset for the descriptor. */
            listFragOffset = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)pListHostDesc);

            /* Does our fragment come before this? */
            if (fragOffset > listFragOffset)
            {
                /* NO. Remember this descriptor */
                pPrevListHostDesc = pListHostDesc;

                /* Goto the next descriptor. */
                pListHostDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc*)pListHostDesc);
            }
            else
            {
                /* YES. We can now insert the descriptor. */
                break;
            }
        }

        /* Are we inserting at the head? */
        if (pPrevListHostDesc == NULL)
        {
            /* YES. So change the current head in the reassembly block. */
            pIPReassemblyBlock->pHostDesc = pHostDesc;

            /* Ensure that the current descriptor points to the next. */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, (Cppi_Desc*)pListHostDesc);
        }
        else
        {
            /* NO. Change the links. The previous BD points to the current BD. */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pPrevListHostDesc, (Cppi_Desc*)pHostDesc);

            /* The current BD points to the next BD */
            Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pHostDesc, (Cppi_Desc*)pListHostDesc);
        }
    }

    /* We have detected another fragment. Account for the length of this fragment. */
    pIPReassemblyBlock->detectedLen = pIPReassemblyBlock->detectedLen + dataLen;

    /* Check if the reassembly is complete? */
    if (pIPReassemblyBlock->originalLen == pIPReassemblyBlock->detectedLen)
    {
        /* Increment the number of reassembled packets. */
        paIPReassemblyMCB.stats.reassembledPkts++;

        /* Return the head of the reassembled packet. Reassembly is complete. */
        *ppReassemblyBlock = pIPReassemblyBlock;
    }

    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function processes ingress packets from the PASS-assisted reassembly input queue.
 *      It performs the following tasks:
 *      @li Forward the non-fragmented IP packet with its flow id and count = 1 to destation queue. This avoids reordering 
 *          the non-fragmented packets.
 *      @li For IPSEC inner IP fragments, call SA LLD to perform the post-decryption operation including padding check and IPSEC header 
 *          and authentication tag removal.
 *      @li Invoke the IP reassembly function
 *      @li Forward the reassembled IP packet with its flow id and fragments count to the destination queue.
 * 
 *
 *  @param[in]  pHostDesc
 *      Descriptor which holds the received packet
 *  @param[in]  destQ
 *      PASS-assisted IP reassembly destination queue
 *
 *  @retval
 *      0       - Success
 *  @retval
 *      <0      - Error
 */
int paEx_reassemLibProc(Cppi_HostDesc *pHostDesc, Qmss_QueueHnd destQ)
{
	pasahoLongInfo_t 	*pInfo;
    paIPReassemblyCxt_t*  pReassemblyInfo;
    Cppi_HostDesc*       reassembledHd;
	uint32_t	      	 infoLen;
    uint16_t             ipOffset; 
    uint16_t             fragOffset;
    uint16_t             pktLen;
	uint8_t*             ipHdr;
    uint16_t             tmp;
	paEx_IPVersion       ipVer;
    uint8_t*             ipv6FragHdr;
    uint16_t             fragHdrOffset;     /* IPv6 only: From IP header */
    uint16_t             nextHdrOffset;      /* IPv6 only: From IP header */
	/* Process the packet from the PASS-assisted reassembly input queue */
	/* Get the parse information */
	if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)pHostDesc, (uint8_t **)&pInfo, &infoLen) != CPPI_SOK)  {
		//System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
		utilRecycleDesc (pHostDesc);
		return (-1);
	}
        
    /* Extract the fragmentation offset & flags to host order before we proceed. */
    ipOffset = PASAHO_LINFO_READ_START_OFFSET(pInfo);
    ipHdr = (uint8_t *)(pHostDesc->buffPtr + ipOffset);
    tmp = pktRead8bits(ipHdr,0) & IPV4_VER_MASK; 
    if (tmp == IPV4_VER_VALUE)
    {
        /* This is an IPv4 packet. */
        ipVer = paEx_IPVersion_IPV4;

        fragOffset = IPV4_READ_FLAGS_FRAGO(ipHdr);
        fragOffset &= IPV4_FRAG_DET_MASK;
    }
    else if (tmp == IPV6_VER_VALUE)
    {
        int fSearchDone = paEX_FALSE;
        int tmpNextHdr, tmpHdrLen;
        
        /* This is an IPv6 packet */
        ipVer   =   paEx_IPVersion_IPV6;
        
        /* Extract the fragmentation offset & flags to host order before we proceed */
        fragHdrOffset = IPv6HDR_SIZE;
        nextHdrOffset = PA_FIELDOFFSET(paEx_IPv6Header, NextHeader);
        tmpNextHdr = pktRead8bits(ipHdr, nextHdrOffset);
        
        while (!fSearchDone)
        {
            switch (tmpNextHdr)
            {
                case IP_PROTO_NEXT_IPV6_FRAG:
                    ipv6FragHdr = (uint8_t *)ipHdr + fragHdrOffset;
                    fragOffset = pktRead16bits(ipv6FragHdr, PA_FIELDOFFSET(paEx_IPv6FragHeader, FragOffset));
                    fSearchDone = paEX_TRUE;
                    break;
                    
                case IP_PROTO_NEXT_IPV6_HOP_BY_HOP:
                case IP_PROTO_NEXT_IPV6_ROUTE:
                case IP_PROTO_NEXT_IPV6_DEST_OPT:
                    tmpNextHdr = pktRead8bits(ipHdr, fragHdrOffset);
                    tmpHdrLen = pktRead8bits(ipHdr, fragHdrOffset + 1);
                    tmpHdrLen = (tmpHdrLen + 1) << 3;
                    nextHdrOffset = fragHdrOffset;
                    fragHdrOffset += tmpHdrLen;
                    break;
                    
                case IP_PROTO_NEXT_IPV6_NO_NEXT:
                default:
                    /* It is not a fragmented packet */
                    fragOffset = 0;
                    fSearchDone = paEX_TRUE;
                    break;
            }    
        }
        
    }
    else
    {
        /* This is an invalid version. */
        return -1;
    }

       
    if (!fragOffset)
    {
        /* It is a non-fragmented packet, forward it immediately */
        Qmss_queuePush (destQ, pHostDesc, pHostDesc->buffLen, paIPReassemblyMCB.descSize, Qmss_Location_TAIL);
    }
    else
    {
#ifdef _INCLUDE_SA
    
        /* 
         * Is it an IPSEC ESP/AH packet?
         * It is required to perform IPSEC post-processing for IPSEC-decrypted fragments
         */
        
        if (PASAHO_LINFO_IS_IPSEC(pInfo))
        {
            /* Increment the number of fragments decrypted. */
            paIPReassemblyMCB.fragmentsDecrypted++;
            
            ipsecPostProcessing(hd, pInfo, &offsetAdjust); 
            
            ipOffset -= offsetAdjust;   
        } 
        
#endif        
    
        if (paEx_ipReassembly(pHostDesc, 
                                PASAHO_LINFO_READ_TFINDEX(pInfo),
                                destQ,
                                ipOffset,
                                fragOffset,
                                nextHdrOffset,
                                fragHdrOffset,
                                ipVer,
                                &pReassemblyInfo))
        {
            /* Error Processing */
            return -1;
        }                       
                               
         
        if(pReassemblyInfo)                    
        {
            /* Reassembly packet is ready */  
            reassembledHd = pReassemblyInfo->pHostDesc;        
            ipHdr = (uint8_t *)(reassembledHd->buffPtr + ipOffset);
            pktLen = pReassemblyInfo->originalLen + ipOffset;
            if (ipVer == paEx_IPVersion_IPV4) {
                IPV4_WRITE_LEN(ipHdr, pReassemblyInfo->originalLen);
                IPV4_WRITE_FLAGS_FRAGO(ipHdr, 0);
                utilUpdateIpChksums(ipHdr);
            } 
            else
            {
                pktWrite16bits(ipHdr, PA_FIELDOFFSET(paEx_IPv6Header, PayloadLength), pReassemblyInfo->originalLen + fragHdrOffset - IPv6HDR_SIZE);
                pktLen += fragHdrOffset;
            }

            /* Get the parse information */
            Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)reassembledHd, (uint8_t **)&pInfo, &infoLen);
            PASAHO_LINFO_SET_FRANCNT(pInfo, pReassemblyInfo->fragCnt);
            //pktLen = pReassemblyInfo->originalLen + ipOffset;
            
            /* send the packet out */
            Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)reassembledHd, pktLen);
  	        Qmss_queuePush (destQ, reassembledHd, pktLen, paIPReassemblyMCB.descSize, Qmss_Location_TAIL);
        
            /* re-initialize the reassembly block */
            paEx_initReassemblyBlock(pReassemblyInfo);
        }                   
                           
    }

    return (0);
}

/**
 *  @b Description
 *  @n  
 *      The function clear the IP reassembly Master Control Block and release all resources. It may
 *      be invoked if the PASS-assisted reassembly operation is no longer needed.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  @note This fuction may be modified to free the IP reassembly blocks from memory heap and/or
 *        performs other OS related functions
 */
int paEx_reassemLibDelete(void)
{
    return 0;
}

