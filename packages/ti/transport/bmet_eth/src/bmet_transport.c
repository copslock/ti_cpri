/**
 *   @file  bmet_transport.c
 *
 *   @brief
 *      BMET transport mechanism for small cell.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */
/**
 *   @file  bmet_transport.c
 *
 *   @brief   
 *      This is a very slim ethernet transport framework library implementation.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012, Texas Instruments, Inc.
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
 

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/* BMET channel Includes */
#include <ti/transport/bmet_eth/bmet_transport.h>
#include <ti/transport/bmet_eth/bmet_eth_osal.h>

/* QM include files */

/* Logger */
 
/* Debug */
//#define NETCP_DEBUG_PRINT 
#undef NETCP_DEBUG_PRINT
/***************** global variables used only in core0 ***************************/
/**
 *  @b Description
 *  @n
 *      Utility function which converts a local address to global.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Global Address
 */
static inline uint32_t l2_global_address (uint32_t addr)
{
	uint32_t corenum;

	/* Get the core number. */
	corenum = CSL_chipReadReg(CSL_CHIP_DNUM);

	/* Compute the global address. */
	if ((addr >= 0x800000) && (addr < 0x900000))
		return (addr + (0x10000000 + (corenum*0x1000000)));
	else
		return addr;
}

static Cppi_HostDesc* Get_CppiDesc(Qmss_QueueHnd gPoolQHnd, uint32_t descsize)
{
  Cppi_HostDesc* pCppiDesc = NULL;
  uint8_t*     ptrDataBuffer;
  uint32_t     dataLen;  

  /* Get a free descriptor from the global free queue we setup 
   * during initialization.
   */
  pCppiDesc = Qmss_queuePop (gPoolQHnd);

  /* Did we get a packet. */ 
  if (pCppiDesc != NULL)
  {

    /* Yes. Invalidate the packet. */
    BMET_ETH_osalBeginPktAccess (pCppiDesc, descsize);
    
    /* Get the original buffer information. */
    Cppi_getOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)pCppiDesc,
                             (uint8_t**)&ptrDataBuffer, &dataLen);

    /* Ensure that the original buffer and buffer address is one and the same. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pCppiDesc, (uint8_t*)ptrDataBuffer, NULL);

    /* Kill any other links to the packets. */
    Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pCppiDesc, NULL);

    /* Setup the packet length  */
    Cppi_setPacketLen(Cppi_DescType_HOST, (Cppi_Desc*)pCppiDesc, NULL);
  }

  return (pCppiDesc);
  
}


static Cppi_HostDesc* Link_CppiDesc(Cppi_HostDesc* pPkt1, Cppi_HostDesc* pPkt2, Cppi_HostDesc* pLastPkt)
{
  Cppi_HostDesc*  ptrPrevDesc = NULL;
  Cppi_HostDesc*  ptrDesc;
  uint32_t        packetLength;

  /* Validations: Ensure that the packets passed to be merged are correct */
  if ((pPkt1 == NULL) || (pPkt2 == NULL))
    return NULL;

  /* Is the last packet in the chain specified? */
  if (pLastPkt == NULL)
  {
    /* NO. Get the pointer to the descriptor. */
    ptrDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc *)pPkt1);
    while (ptrDesc != NULL)
    {
      /* Store the previous descriptor. */
      ptrPrevDesc = ptrDesc;
      /* Get the pointer to the next descriptor. */
      ptrDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc *)ptrDesc);
    }
  }
  else
  {
     /* YES. Get the last element in the chain. */
     ptrPrevDesc = (Cppi_HostDesc*)pLastPkt;
  }

  /* Link the last buffer descriptor of packet1 with the packet2. If packet1 had only 1 
   * descriptor we link the head descriptor of packet1 with packet2 else we link the
   * last descriptor from packet1 to the head descriptor of packet2 */
  if (ptrPrevDesc == NULL)
    Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pPkt1, (Cppi_Desc*)pPkt2);
  else
    Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrPrevDesc, (Cppi_Desc*)pPkt2);

  /* Compute the total packet length after merging the packets. */
  packetLength = Cppi_getPacketLen(Cppi_DescType_HOST, (Cppi_Desc*)pPkt1) + 
                 Cppi_getPacketLen(Cppi_DescType_HOST, (Cppi_Desc*)pPkt2);

  /* Ensure that the packet length is properly setup in the packet1 to account for the total length */
  Cppi_setPacketLen(Cppi_DescType_HOST, (Cppi_Desc*)pPkt1, packetLength);

  /* Reset the packet length in the packet2 because this has now been merged and the 
   * length has already been accounted for */
  Cppi_setPacketLen(Cppi_DescType_HOST, (Cppi_Desc*)pPkt2, 0);

  /* Return the merged packet descriptor */
  return (pPkt1);
}
    
/* This function initializes the descriptors
   used by the netcp transport mechanism*/
static bmetStatus_e bmetDescInit(bmetInst_t* bmetInst, uint32_t numDesc, uint32_t sgmii_port_num, uint32_t pktHdr_Size)
{
  uint32_t i, num_allocated;
  Qmss_QueueHnd gQueueHnd           = bmetInst->system_free_queue_hnd; 
  Qmss_QueueHnd retQueueHnd         = bmetInst->gTxReturnQHnd; 
  uint32_t payloadSize              = bmetInst->payload_size;
  uint8_t *pktHeader_ptr            = (uint8_t *) l2_global_address((uint32_t)&bmetInst->bmetPktHeader[0]);
  uint32_t pktHeader_size           = pktHdr_Size;

  Cppi_HostDesc*   pd; /* packet descriptor */
  Cppi_HostDesc*   bd; /* buffer descriptor */

  /* Since there are two descriptors per buffer send, inc is by 2 */
  num_allocated = 0;
  for(i=0; i< numDesc/2;i++)
  {
    /* Get bufferless descriptors from global system queue (one PD and one BD) */
    pd = (Cppi_HostDesc*)Get_CppiDesc(gQueueHnd, bmetInst->host_desc_size);
    if (pd == NULL)
    {
      continue;
    }
    num_allocated++;    

    /* Set swInfo with the index of this descriptor in the array so that it is 
       easy to manage the info about this descriptor in the array
     */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc*)pd, (uint32_t)i);

    /* Set desc addr of this descriptor in the array so that it is 
       easy to manage the info about this descriptor in the array
     */
    Cppi_setSoftwareInfo1(Cppi_DescType_HOST, (Cppi_Desc*)pd, (uint32_t)pd);

    /* Set the data buffer to point to the UDP header & lengths appropriately. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pd, 
                  (uint8_t *)l2_global_address((uint32_t)pktHeader_ptr), pktHeader_size);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pd, pktHeader_size);
    
    bd = (Cppi_HostDesc*)Get_CppiDesc(gQueueHnd, bmetInst->host_desc_size);
    if (bd == NULL)
    {
      continue;
    }
    num_allocated++;    

    /* Set desc addr of this descriptor in the array so that it is 
       easy to manage the info about this descriptor in the array
     */
    Cppi_setSoftwareInfo1(Cppi_DescType_HOST, (Cppi_Desc*)bd, (uint32_t)bd);

    /* Set the data buffer (NULL as there is no log buffer attached yet) & lengths appropriately. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)bd, NULL, payloadSize);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)bd,  payloadSize);
    
    /* merge PD->BD */
    pd = (Cppi_HostDesc* )Link_CppiDesc(pd, bd, NULL);

    /* Use SGMII Port 1 for slow path traffic to send out */
    Cppi_setPSFlags (Cppi_DescType_HOST, (Cppi_Desc *)pd, (1 << sgmii_port_num));

    /* Set the return queue in the packet to be the transmit completion queue. */
    Cppi_setReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)pd, Qmss_getQueueNumber(retQueueHnd));

    BMET_ETH_osalEndPktAccess((void*) pd, bmetInst->host_desc_size);
    BMET_ETH_osalEndPktAccess((void*) bd, bmetInst->host_desc_size);

    /* fill out descriptor list */
    bmetInst->bmetDescList[i].packetDesc = pd;
    bmetInst->bmetDescList[i].bufferDesc = bd;
    bmetInst->bmetDescList[i].free       = TRUE;
  }

  if (num_allocated != numDesc)
    return (BMET_DESC_ALOCFAIL);
  else
    return (BMET_SUCCESS);
}

/* This function sends the next pending packet for core*/
int32_t bmet_get_status(transport_HANDLE handle)
{
  Cppi_HostDesc*   pd;
  Cppi_HostDesc*   bd;
  uint32_t         pdIndex;
  int32_t          identity = -1;
  Qmss_QueueHnd    gTxReturnQHnd;
  bmetInst_t* bmetInst  = (bmetInst_t *) handle;

  gTxReturnQHnd = bmetInst->gTxReturnQHnd;

  while (1)
  {
    /* Pop off a packet. */
    pd = (Cppi_HostDesc*)(Qmss_queuePop(gTxReturnQHnd));

    if (pd == NULL)
      break;

    /* Yes. Invalidate the packet. */
    BMET_ETH_osalBeginPktAccess (pd, bmetInst->host_desc_size);

    /* Find from which consumer this packet was sent */
    /* Software info in PD descriptor is intended to hold the index in the PD array for this PD*/
    pdIndex = pd->softwareInfo0;

    if (pdIndex >= bmetInst->numTrasportDesc)
        return (BMET_DESC_INDEX_EXCEED_FAIL);

    /* Software info in BD descriptor is intended to hold the core number */
	bd = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc *)pd);
  
    identity = bd->softwareInfo0;

	/* Check error condition. Check if the atached BD is the same one that was shipped out */
    if (bd == NULL)
    {
      return (BMET_ATTCHED_BD_NULL);
    }

	if((uint32_t)bd != (uint32_t)bmetInst->bmetDescList[pdIndex].bufferDesc)
    {
      return (BMET_ATTCHED_BD_NOT_MATCHED);
    }

    /* Clear up the software information for BD which represents coreNum. */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc*)bd, 0);

    /* Check for error condition. This packet should have been marked as used until now. 
       If it is free this is a error condition.
    */

    if(bmetInst->bmetDescList[pdIndex].free == TRUE)
    {
      /* Error! This packet should have been marked as used, but it is free */
      return (BMET_DESC_FOUND_FREE_BEFORE_FREE);
    }

    /* Set PD to free */
    bmetInst->bmetDescList[pdIndex].free = TRUE;
    
    bmetInst->bmetCh_stats.pktsReturnCount++;


   }

   return(identity);
}

/* This function sends the next pending packet for core*/
int32_t bmet_send(transport_HANDLE handle, uint8_t* logBuf, int32_t identity)
{
  Cppi_HostDesc*   pd;
  Cppi_HostDesc*   bd;
  bmetDescInfo_t   *bmetDesc;
  uint32_t         logBufSize;
  int32_t          stats, i, flag;
  bmetInst_t* bmetInst = (bmetInst_t *) handle;

  /* Check if there is a descriptor to be freed, if yes free it */
  stats = bmet_get_status(handle);
  if ((stats == BMET_DESC_INDEX_EXCEED_FAIL)        || 
      (stats == BMET_DESC_FOUND_FREE_BEFORE_FREE)   || 
      (stats == BMET_ATTCHED_BD_NOT_MATCHED)        || 
      (stats == BMET_ATTCHED_BD_NULL) )
  {
    BMET_ETH_osalException(bmetInst->moduleId, stats);  
    return (stats);
  }

  bmetInst->desc_array_index.numFreeDesc = 0;
  for (i=0; i< (bmetInst->numTrasportDesc); i++)
  {
    bmetDesc = &bmetInst->bmetDescList[i];

    if (bmetDesc->free == TRUE) 
    {
      bmetInst->desc_array_index.numFreeDesc++;
    }
  }

   /* Point to next free descriptor index for next calls in the entire ring*/
  flag = 0;
  for (i=0; i< (bmetInst->numTrasportDesc); i++)
  {
    bmetDesc = &bmetInst->bmetDescList[i];

    if (bmetDesc->free == TRUE) 
    {
      bmetInst->desc_array_index.nextFreeDesc = i;
      flag = 1;
      break;
    }
  }

  if (flag == 0)
  {
    bmetInst->bmetCh_stats.noPdAvailableCount++;    
    return (BMET_DESC_NOTAVAILABLE);
  }

  logBufSize = bmetInst->payload_size;

  BMET_ETH_osalEndMemAccess(logBuf, logBufSize);

  /* Get free descriptor */
  bmetDesc = &bmetInst->bmetDescList[bmetInst->desc_array_index.nextFreeDesc];
  pd = bmetDesc->packetDesc;
  bd = bmetDesc->bufferDesc;

  /* Set PD to used. This can be removed later. It is an extra control element to catch errors.*/
  bmetDesc->free = FALSE;

  /* Attach log buffer to the Buffer descriptor BD */
  Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)bd, (uint8_t *)l2_global_address((uint32_t)logBuf), logBufSize);
  
  /* SWInfo in BD records the consumer ID number. This is used when the packet is recycled back to know
     who needs to recycle a log buffer.
  */
  Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc*)bd, identity);

  BMET_ETH_osalEndPktAccess((void*) pd, bmetInst->host_desc_size);
  BMET_ETH_osalEndPktAccess((void*) bd, bmetInst->host_desc_size);

  /* Push the packet on the Ethernet Transmit queue */
  Qmss_queuePushDescSize(bmetInst->ethTxQueueHnd, pd, bmetInst->host_desc_size);
  
  /* Update stats */
  bmetInst->bmetCh_stats.pktsSentCount++;

  return(BMET_SUCCESS);
}

/* BMET Ch Destroy API */
bmetStatus_e bmet_destroy(transport_HANDLE handle)
{
  bmetInst_t* bmetInst = (bmetInst_t*) handle;
  uint32_t count, i;
  Cppi_HostDesc*   pd;
  Cppi_HostDesc*   bd;
  bmetDescInfo_t   *bmetDesc;

  /* Return back all the descriptors to the system */
  count = Qmss_getQueueEntryCount (bmetInst->gTxReturnQHnd);
  for (i = 0; i < count; i++)
  {
    /* Pop off a packet. */
    pd = (Cppi_HostDesc*)(Qmss_queuePop(bmetInst->gTxReturnQHnd));
	bd = (Cppi_HostDesc*) Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc *)pd);

    /* Not the descriptor that was sent before, possible corruption */
    if ( (pd != (Cppi_HostDesc*)Cppi_getSoftwareInfo1(Cppi_DescType_HOST, (Cppi_Desc *)pd)) |
         (bd != (Cppi_HostDesc*)Cppi_getSoftwareInfo1(Cppi_DescType_HOST, (Cppi_Desc *)bd)) )
      break;

    /* return the packet to the free queue - recycle */
    Qmss_queuePushDescSize (bmetInst->system_free_queue_hnd,  pd,   bmetInst->host_desc_size);
    /* loose the count in the instance */
   // netcpInst->numTrasportDesc--;

    /* return the packet to the free queue - recycle */
    Qmss_queuePushDescSize (bmetInst->system_free_queue_hnd,  bd,   bmetInst->host_desc_size);
    /* loose the count in the instance */
    // netcpInst->numTrasportDesc--;
  }

  /* Return the remaining descriptors to system queue, where it was taken from */
  for (i = 0; i < bmetInst->numTrasportDesc; i++)
  {
	 bmetDesc = &bmetInst->bmetDescList[i];
	 /* If the descriptor is not free, it should have been recycled earlier */
	 if (bmetDesc->free == FALSE)
		 continue;

	  pd = bmetDesc->packetDesc;
	  bd = bmetDesc->bufferDesc;
	  /* return the packet to the free queue - recycle */
	  Qmss_queuePushDescSize (bmetInst->system_free_queue_hnd,  pd,   bmetInst->host_desc_size);
	  /* return the packet to the free queue - recycle */
	  Qmss_queuePushDescSize (bmetInst->system_free_queue_hnd,  bd,   bmetInst->host_desc_size);
  }

  BMET_ETH_osalMemFree((void*) bmetInst, BMET_INST_SIZE);

  return (BMET_SUCCESS);
}

static void bmet_IPChecksum(Bmet_IPHeader* ptr_iphdr)
{
    int32_t   tmp1;
    uint16_t  *pw;
    uint32_t  TSum = 0;

    /* Get header size in 4 byte chunks */
    tmp1 = ptr_iphdr->VerLen & 0xF;

    /* Checksum field is NULL in checksum calculations */
    ptr_iphdr->Checksum = 0;

    /* Checksum the header */
    pw = (uint16_t *)ptr_iphdr;
    do {
        TSum += (uint32_t)*pw++;
        TSum += (uint32_t)*pw++;
    } while( --tmp1 );
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = ~TSum;

    /* Note checksum is Net/Host byte order independent */
    ptr_iphdr->Checksum = (uint16_t)TSum;
    return;
}

int32_t bmetBuildNetPktHeader(uint8_t * pkt, bmetChConfig_t bmetConfig, uint32_t pktHdrSize)
{
  Bmet_EthHeader*        ptrEthHeader;
  Bmet_VLANHeader*       ptrVLANHeader;
  Bmet_IPHeader*         ptrIPHeader;
  Bmet_UDPHeader*        ptrUDPHeader;
  int32_t                i;
  uint32_t               packetLen;
  uint16_t               localUdpPort=bmetConfig.local_udp_port;
  uint16_t               remoteUdpPort=bmetConfig.remote_udp_port;
  
 
  /* Get the packet length of the Payload. For UIA transport, payload is always MTU size.*/
  packetLen = bmetConfig.payload_size;

  /* Get the pointer to the various headers. */
  ptrEthHeader = (Bmet_EthHeader*)pkt;
  if (bmetConfig.send_vlan_header == BMET_PUT_VLAN_HEADER)
  {
    ptrVLANHeader= (Bmet_VLANHeader*)((uint8_t*)ptrEthHeader + BMET_ETHHDR_SIZE);
    ptrIPHeader  = (Bmet_IPHeader*)((uint8_t*)ptrVLANHeader  + BMET_VLANHDR_SIZE);
  }
  else
  {
    ptrIPHeader  = (Bmet_IPHeader*)((uint8_t*)ptrEthHeader  + BMET_ETHHDR_SIZE);
  }

  ptrUDPHeader = (Bmet_UDPHeader*)((uint8_t*)ptrIPHeader  + BMET_IPHDR_SIZE);
  
  /* Populate the UDP Header: The destination port should be the input parameter remoteUdpPort 
   * and the source port is the input parameter localUdpPort. */
  ptrUDPHeader->DstPort   = htons(remoteUdpPort);
  ptrUDPHeader->SrcPort   = htons(localUdpPort);
  ptrUDPHeader->Length    = htons(BMET_UDPHDR_SIZE + packetLen);
  ptrUDPHeader->UDPChecksum = 0x0000;

  /* The packet length now includes the UDP Header */
  packetLen = packetLen + BMET_UDPHDR_SIZE;

  /* Populate the IP Header*/
  ptrIPHeader->VerLen   = 0x45;
  ptrIPHeader->Tos      = 0;
  ptrIPHeader->Id       = 0x2e54;
  ptrIPHeader->FlagOff  = 0x0;
  ptrIPHeader->Ttl      = 128;
  ptrIPHeader->Protocol   = BMET_IPPROTO_UDP;
  ptrIPHeader->IPSrc[0]   = bmetConfig.IPSrc[0];
  ptrIPHeader->IPSrc[1]   = bmetConfig.IPSrc[1];
  ptrIPHeader->IPSrc[2]   = bmetConfig.IPSrc[2];
  ptrIPHeader->IPSrc[3]   = bmetConfig.IPSrc[3];
  ptrIPHeader->IPDst[0]   = bmetConfig.IPDst[0];
  ptrIPHeader->IPDst[1]   = bmetConfig.IPDst[1];
  ptrIPHeader->IPDst[2]   = bmetConfig.IPDst[2];
  ptrIPHeader->IPDst[3]   = bmetConfig.IPDst[3];

  /* The packet length now includes the IP Header also. */
  packetLen = packetLen + BMET_IPHDR_SIZE;

  /* Setup the total length in the packet. */
  ptrIPHeader->TotalLen   = htons(packetLen);

  /* Setup the IP Header checksum. */
  bmet_IPChecksum (ptrIPHeader);

  /* Populate the MAC Header. 
   * - Setup the destination and source MAC Address 
   * - This is a VLAN Packet. */
  for (i = 0; i < 6; i++)
  {
    ptrEthHeader->SrcMac[i] = bmetConfig.MacSrc[i];
    ptrEthHeader->DstMac[i] = bmetConfig.MacDst[i];

  }

  /* The packet length now includes the VLAN Header also. */
  if (bmetConfig.send_vlan_header == BMET_PUT_VLAN_HEADER)
  {
    /* Populate the VLAN header with VLAN ID 2. All outgoing packets on the 
     * SCBP need to be tagged with VLAN Identifier 2. */
    ptrVLANHeader->tci      = htons(2);
    ptrVLANHeader->protocol = htons(BMET_ETH_IP);
    packetLen               = packetLen + BMET_VLANHDR_SIZE;
    ptrEthHeader->Type      = htons(BMET_ETH_VLAN);
  }
  else
  {
    ptrEthHeader->Type = htons(BMET_ETH_IP);
  }

  BMET_ETH_osalEndMemAccess(pkt, packetLen);

  return 0;
}

/* Uia Trasport Consumer Create API */
transport_HANDLE bmet_create
(
  bmetChConfig_t  bmetConfig
)
{
  uint8_t           isAllocated;
  uint32_t          pktHdrSize;
  bmetInst_t*       bmetInst;
  transport_HANDLE  transport_handle;

  transport_handle = BMET_ETH_osalMemAlloc(BMET_INST_SIZE, BMET_INST_ALIGN);

  if (transport_handle == NULL)
  {
    return (NULL);
  }

  bmetInst = (bmetInst_t*)transport_handle;

  /* Increment the consumer count status attached to that contract for next consumer */
  /* Create the netcpInst values */

  memset(bmetInst, 0, sizeof(bmetInst_t));

  if (bmetConfig.num_trasport_desc > BMET_MAX_NUM_TRANSPORT_DESC)
  {
    BMET_ETH_osalMemFree((void*) transport_handle, BMET_INST_SIZE);
    BMET_ETH_osalException(bmetInst->moduleId, BMET_UNSUPPORTED);    
    return (NULL);
  }

  bmetInst->host_desc_size          = bmetConfig.desc_size;
  bmetInst->numTrasportDesc         = bmetConfig.num_trasport_desc>>1; /* Internal operations */
  bmetInst->payload_size            = bmetConfig.payload_size;
  bmetInst->system_free_queue_hnd   = bmetConfig.global_free_queue_hnd;
  bmetInst->moduleId                = bmetConfig.moduleId;

 bmetInst->ethTxQueueHnd     = Qmss_queueOpen(Qmss_QueueType_PASS_QUEUE, bmetConfig.eth_tx_queue_num, &isAllocated);
  if(bmetInst->ethTxQueueHnd == NULL)
  {
    BMET_ETH_osalMemFree((void*) transport_handle, BMET_INST_SIZE);
    BMET_ETH_osalException(bmetInst->moduleId, BMET_QUEUEOPEN_FAIL);
    return (NULL);
  }

  /* Initialize indexes for descriptor array */  
  bmetInst->desc_array_index.numFreeDesc        = bmetInst->numTrasportDesc;
  bmetInst->desc_array_index.nextFreeDesc       = 0;

  /* Open the Tx Completion Queue for descriptors received back from
     the switch
  */
  bmetInst->gTxReturnQHnd       = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);

  bmetInst->bmetCh_stats.returnQueueNum = bmetInst->gTxReturnQHnd;
  
  if(bmetInst->gTxReturnQHnd == NULL)
  {
    BMET_ETH_osalException(bmetInst->moduleId, BMET_QUEUEOPEN_FAIL);
    return (NULL);
  }
  /* Clear the return queue for any old information */
  Qmss_queueEmpty(bmetInst->gTxReturnQHnd);

  if (bmetConfig.send_vlan_header == BMET_PUT_VLAN_HEADER)
  {
    pktHdrSize  = BMET_PKT_HEADER_SIZE;
  }
  else
  {
    pktHdrSize  = BMET_PKT_HEADER_SIZE-BMET_VLANHDR_SIZE;
  }
  
  if ((bmetDescInit(bmetInst, bmetConfig.num_trasport_desc, bmetConfig.sgmii_send_port_num, pktHdrSize)) != BMET_SUCCESS)
  {
    BMET_ETH_osalMemFree((void*) transport_handle, BMET_INST_SIZE);
    BMET_ETH_osalException(bmetInst->moduleId, BMET_DESC_ALOCFAIL);    
    return (NULL);
  }
  bmetBuildNetPktHeader(&bmetInst->bmetPktHeader[0], bmetConfig, pktHdrSize);
  return (transport_handle);

}

/* Nothing past this point */
