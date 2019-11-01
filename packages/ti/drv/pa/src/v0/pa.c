/******************************************************************************
 * FILE PURPOSE:  Main Routines for PA LLD
 ******************************************************************************
 * FILE NAME:   pa.c  
 *
 * DESCRIPTION: Main PA LLD functions 
 *
 * FUNCTION               DESCRIPTION
 * ========               ===========
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2009-2014
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

#if defined(_LITTLE_ENDIAN)
  #define SWIZ(x)  (sizeof((x)) == 1 ? (x) : (sizeof((x)) == 2 ? swiz16((x)) : (sizeof((x)) == 4 ? swiz32((x)) : 0)))
#else
  #define SWIZ(x)  (x)
  #define swizFcmd(x)
  #define swizStatsRep(x)
#endif

#include <string.h>
#include <stddef.h>
#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pa_osal.h>
#include <ti/drv/pa/pasahost.h>
#include "paloc.h"
#include "pafrm.h"
#include <ti/csl/cslr_pa_ss.h>
#include <ti/drv/rm/rm_services.h>

#define pa_mac_zero(x)      pa_uint8_zero((uint8_t *)(x), 6)
#define pa_ip_zero(x)       pa_uint8_zero((uint8_t *)((x).ipv6), 16)

/* Global Variable which describes the PA LLD Version Information */
const char paLldVersionStr[] = PA_LLD_VERSION_STR ":" __DATE__  ":" __TIME__;

/* RM DTS resource names */
const char rmLut[RM_NAME_MAX_CHARS] = "pa-lut";
const char rmFirmware[RM_NAME_MAX_CHARS] = "pa-firmware";
const char *rmUsrStats[2] = { "pa-32bUsrStats",
                              "pa-64bUsrStats"};

/* Next header conversion table */
const uint8_t pa_next_hdr_tbl[] = {
    PAFRM_HDR_MAC,
    PAFRM_HDR_IPv4,
    PAFRM_HDR_IPv6,
    PAFRM_HDR_CUSTOM_C1,
    PAFRM_HDR_UDP,
    PAFRM_HDR_UDP_LITE,
    PAFRM_HDR_TCP,                                    
    PAFRM_HDR_CUSTOM_C2,
    PAFRM_HDR_UNKNOWN
}; 

/* local utility functions */
int pa_rmService (Rm_ServiceType type, const char *resName, int32_t *resNum, int32_t *resCnt);
 
#if defined(_LITTLE_ENDIAN)

/*********************************************************************
 * FUNCTION PURPOSE: Swizzling
 *********************************************************************
 * DESCRIPTION: The PA sub-system requires all multi-byte fields in
 *              big endian format.
 *********************************************************************/
static inline uint16_t swiz16(uint16_t x)
{
  return ((x >> 8) | (x << 8));
}

static inline uint32_t swiz32 (uint32_t x)
{
  return (((x) >> 24) | (((x) >> 8) & 0xff00L) | (((x) << 8) & 0xff0000L) | ((x) << 24));
}

static void swizFcmd (pafrmCommand_t *fcmd)
{
  fcmd->commandResult =  SWIZ(fcmd->commandResult);
  fcmd->comId         =  SWIZ(fcmd->comId);
  fcmd->retContext    =  SWIZ(fcmd->retContext);
  fcmd->replyQueue    =  SWIZ(fcmd->replyQueue);
}

static void swizStatsRep (paSysStats_t *s)
{
  s->classify1.nPackets             = SWIZ(s->classify1.nPackets);
  s->classify1.nIpv4Packets         = SWIZ(s->classify1.nIpv4Packets);
  s->classify1.nIpv4PacketsInner    = SWIZ(s->classify1.nIpv4PacketsInner);
  s->classify1.nIpv6Packets         = SWIZ(s->classify1.nIpv6Packets);
  s->classify1.nIpv6PacketsInner    = SWIZ(s->classify1.nIpv6PacketsInner);
  s->classify1.nCustomPackets       = SWIZ(s->classify1.nCustomPackets);
  s->classify1.nSrioPackets         = SWIZ(s->classify1.nSrioPackets);
  s->classify1.nLlcSnapFail         = SWIZ(s->classify1.nLlcSnapFail);
  s->classify1.nTableMatch          = SWIZ(s->classify1.nTableMatch);
  s->classify1.nNoTableMatch        = SWIZ(s->classify1.nNoTableMatch);
  s->classify1.nIpFrag              = SWIZ(s->classify1.nIpFrag);
  s->classify1.nIpDepthOverflow     = SWIZ(s->classify1.nIpDepthOverflow); 
  s->classify1.nVlanDepthOverflow   = SWIZ(s->classify1.nVlanDepthOverflow);
  s->classify1.nGreDepthOverflow    = SWIZ(s->classify1.nGreDepthOverflow);
  s->classify1.nMplsPackets         = SWIZ(s->classify1.nMplsPackets);
  s->classify1.nParseFail           = SWIZ(s->classify1.nParseFail);
  s->classify1.nInvalidIPv6Opt      = SWIZ(s->classify1.nInvalidIPv6Opt);
  s->classify1.nTxIpFrag            = SWIZ(s->classify1.nTxIpFrag);
  s->classify1.nSilentDiscard       = SWIZ(s->classify1.nSilentDiscard);
  s->classify1.nInvalidControl      = SWIZ(s->classify1.nInvalidControl);
  s->classify1.nInvalidState        = SWIZ(s->classify1.nInvalidState);
  s->classify1.nSystemFail          = SWIZ(s->classify1.nSystemFail);

  s->classify2.nPackets             = SWIZ(s->classify2.nPackets);
  s->classify2.nUdp                 = SWIZ(s->classify2.nUdp);
  s->classify2.nTcp                 = SWIZ(s->classify2.nTcp);
  s->classify2.nCustom              = SWIZ(s->classify2.nCustom);
  s->classify2.nSilentDiscard       = SWIZ(s->classify2.nSilentDiscard);
  s->classify2.nInvalidControl      = SWIZ(s->classify2.nInvalidControl);

  s->modify.nCommandFail = SWIZ(s->modify.nCommandFail);


}
#endif

/***************************************************************************
 * FUNCTION PURPOSE: snap shot the reassembly control block structure
 ***************************************************************************
 * DESCRIPTION: reads the PDSP scratch memory for reassembly control block
 ***************************************************************************/
static uint32_t pa_read_reassem_control_blk(uint32_t *addr, pa_trafficFlow_t *tf)
{
  uint32_t *ptr, reg;

  ptr = addr;

  /* read 32 bits at a time */
  reg = *ptr ++;
  tf->numTF       = PASAHO_READ_BITFIELD(reg, 24, 8);
  tf->numActiveTF = PASAHO_READ_BITFIELD(reg, 16, 8);

  reg = *ptr ++;
  tf->queue       = PASAHO_READ_BITFIELD(reg, 16, 16);
  tf->flowId      = PASAHO_READ_BITFIELD(reg, 8,  8);

  /* Read the traffic flow bit map information */
  reg = *ptr;

  return (reg);
}

/***************************************************************************
 * FUNCTION PURPOSE: snap shot the reassembly control block structure
 ***************************************************************************
 * DESCRIPTION: reads the PDSP scratch memory for reassembly traffic flow
 ***************************************************************************/
static void pa_read_reassem_traffic_flow(uint32_t *addr, pa_ReassemblyFlow_t *tf, uint32_t bitMap)
{
  uint32_t reg;
  int      bit, index = 31;
  Pa_IpTrafficFlow_t tfTbl[32];

  /* snap shot the pdsp scratch memory to stack */
  memcpy (&tfTbl[0], addr, sizeof (tfTbl));

  while (bitMap)
  {
      /* Check if next index is active or not */
      bit = bitMap & (uint32_t) 0x80000000;

      bitMap = bitMap << 1;

      /* This link is not active */
      if (bit == 0) {
         index --;
         continue;
      }

      /* read the information */
      reg = tfTbl[index].word0;
      tf->index       = index;
      tf->count       = PASAHO_READ_BITFIELD(reg, 24, 8);
      tf->proto       = PASAHO_READ_BITFIELD(reg, 16, 8);
      tf->srcIp       = tfTbl[index].word1;
      tf->dstIp       = tfTbl[index].word2;

      /* Point to next element in the traffic flow debug Info structure */
      tf ++;

      /* Check for next index */
      index --;
  }

  return;
}

/********************************************************************
 * FUNCTION PURPOSE: Determine if a uint8_t array is identically 0
 ********************************************************************
 * DESCRIPTION: Returns TRUE if every element in a uint8_t is zero
 ********************************************************************/
static uint16_t pa_uint8_zero (uint8_t *v, int n)
{
  int i;

  for (i = 0; i < n; i++)
    if (v[i] != 0)
      return (FALSE);

  return (TRUE);

}

/********************************************************************
 * FUNCTION PURPOSE: Determine if two uint8_t arrays are identical
 ********************************************************************
 * DESCRIPTION: Returns TRUE if two uint8_t arrays are equal
 ********************************************************************/
static uint16_t pa_uint8_match (uint8_t *v1, uint8_t *v2, int n)
{
  int i;

  for (i = 0; i < n; i++)
    if (v1[i] != v2[i])
      return (FALSE);

  return (TRUE);

}

/********************************************************************
 * FUNCTION PURPOSE: Determine if two MAC addresses table match
 ********************************************************************
 * DESCRIPTION: Returns TRUE if the MAC addresses table match.
 *              If the first mac address is 0 then this entry in
 *              the table is a don't care which would be a table
 *              match
 ********************************************************************/
static uint16_t pa_mac_match (paMacAddr_t mac1, paMacAddr_t mac2)
{
  /* Mac1 is all zero indicates a don't care, which is a match */
  if (pa_mac_zero (mac1))
    return (TRUE);

  /* Compare the mac addresses */
  return (pa_uint8_match ((uint8_t *)mac1, (uint8_t *)mac2, 6));

}

/***********************************************************************
 * FUNCTION PURPOSE: Determine if two IP addresses table match
 ***********************************************************************
 * DESCRIPTION: Returns TRUE if the IP addresses table match.
 *              If the first IP address is 0 then this entry in 
 *              the table is a don't care which would always be a
 *              table match
 ***********************************************************************/
static uint16_t pa_ip_match (paIpAddr_t ip1,  paIpAddr_t ip2)
{
  /* Ip1 ll zero indicates a don't care, which is a match */
    if (pa_ip_zero (ip1))
      return (TRUE);

  /* Compare the addresses. For IPV4 addresses the 12 msbs will
   * be zero. */
  return (pa_uint8_match ((uint8_t *)(ip1.ipv6), (uint8_t *)(ip2.ipv6),  16));

}

/***********************************************************************
 * FUNCTION PURPOSE: Determine if two IP TOS fields table match
 ***********************************************************************
 * DESCRIPTION: Returns TRUE if the two TOS fields table match.
 *              If the first TOS is a don't care (TRUE), then this is
 *              a table match 
 ************************************************************************/
static uint16_t pa_tos_match (uint8_t tos1, uint8_t tos2)
{
  if (tos1 == tos2)
    return (TRUE);

  return (FALSE);

}


/**********************************************************************
 * FUNCTION PURPOSE: Check for table match on uint8_t
 **********************************************************************
 * DESCRIPTION: Returns TRUE if two uint8_t values will table match.
 *              If the first value is 0 then in the table it is a 
 *              don't care, which is an automatic match.
 **********************************************************************/
static uint16_t pa_u8_match (uint8_t v1, uint8_t v2)
{
  /* If v1 is 0 it is a don't care, which is a match */
  if (v1 == 0)
    return (TRUE);

  if (v1 == v2)
    return (TRUE);

  return (FALSE);

}

/**********************************************************************
 * FUNCTION PURPOSE: Check for table match on uint16_t
 **********************************************************************
 * DESCRIPTION: Returns TRUE if two uint16_t values will table match.
 *              If the first value is 0 then in the table it is a 
 *              don't care, which is an automatic match.
 **********************************************************************/
static uint16_t pa_u16_match (uint16_t v1,  uint16_t v2)
{
  /* If v1 is 0 it is a don't care, which is a match */
  if (v1 == 0)
    return (TRUE);

  if (v1 == v2)
    return (TRUE);

  return (FALSE);

}

/*************************************************************************
 * FUNCTION PURPOSE: Check for table match on uint32_t
 ************************************************************************* 
 * DESCRIPTION: Returns TRUE if two uint32_t values will table match
 *************************************************************************/
static uint16_t pa_u32_match (uint32_t v1,  uint32_t v2)
{
  /* If v1 is 0 it is a don't care, which is a match */
  if (v1 == 0)
    return (TRUE);

  if (v1 == v2)
    return (TRUE);

  return (FALSE);

}

#define PA_INDEX_ALLOCATION_ERROR               -1

/*************************************************************************
 * FUNCTION PURPOSE: Bitmap-based resource allocation  
 *************************************************************************
 * DESCRIPTION: Search and find the first available free index in a bitmap 
 *              allocation table. 
 *				Return PA_INDEX_ALLOCATION_ERROR if nothing is available
 *
 ************************************************************************/
static int pa_bitmap_alloc(uint32_t *allocBitMap, int startIdx, int endIdx)
{
    int tblIdx = startIdx >> 5, bitIdx = startIdx % 32;
    int endTblIdx = endIdx >> 5, endBitIdx;
    uint32_t bitMask;
    uint16_t index;
  
    /* For each bit field word */
    for (;tblIdx <= endTblIdx; tblIdx++)
    {
	    /* Check if block has any free entries */
	    if (!~allocBitMap[tblIdx])
        {
		    continue;
        }  
          
	    /* For each bit in the word */
        if (tblIdx == endTblIdx)
        {
            endBitIdx = endIdx % 32;
        }    
        else
        {
            endBitIdx = 32;  
        }      
            
	    for(; bitIdx < endBitIdx; bitIdx++)
	    {
		    bitMask = 0x1 << bitIdx;
      
		    if(!(allocBitMap[tblIdx] & bitMask))
		    {
			    allocBitMap[tblIdx] |= bitMask;
                index = (tblIdx << 5) +bitIdx;
                return (index); 
		    }
	    }
    }
    
    return PA_INDEX_ALLOCATION_ERROR;
}

/*************************************************************************
 * FUNCTION PURPOSE: Bitmap-based resource Update  
 *************************************************************************
 * DESCRIPTION: Update the specified resource index in a bitmap 
 *              allocation table. 
 *
 ************************************************************************/
static void pa_bitmap_update(uint32_t *allocBitMap, int tblSize, int index, int set)
{
    int tblIdx = index >> 5, bitIdx = index % 32;
  
    if (tblIdx < tblSize)
    {
        if(set)
            allocBitMap[tblIdx] |=  (0x1 << bitIdx);
        
        else
            allocBitMap[tblIdx] &= ~(0x1 << bitIdx);
    }
}

/*************************************************************************
 * FUNCTION PURPOSE: Bitmap-based resource verify  
 *************************************************************************
 * DESCRIPTION: Verify whether the specified resource index is set in a 
 *              bitmap allocation table. 
 * return: TRUE  if the corresponding bit is set
 *         FALSE if the corresponding bit is not set
 *
 ************************************************************************/
static int pa_bitmap_verify(uint32_t *allocBitMap, int tblSize, int index)
{
    int tblIdx = index >> 5, bitIdx = index % 32;
    int ret = FALSE;
  
    if (tblIdx < tblSize)
    {
        if(allocBitMap[tblIdx] & (0x1 << bitIdx))
            ret = TRUE;
    }
    
    return (ret);
}

/*******************************************************************************************
 * FUNCTION PURPOSE: Verify  user-defined statistics
 *******************************************************************************************
 * DESCRIPTION: This function is called to verify whether a user-defined statistics is vaild.
 *              If it is not valid and the RM is not used, just mark it to be valid since
 *              the application is not required to call Pa_allocUsrStats for backward
 *              compatibility 
 *
 *  return -1: if the usr-defined statistics is not valid
 *          0: otherwise
 *               
 ********************************************************************************************/
static int  pa_verify_usr_stats(paInst_t *paInst, int32_t cntIndex, int fCache)
{
    uint16_t    numCounters, num64bCounters; 
    int         rmInst;
    uint32_t    mtCsKey; 
    int32_t     rmCntIndex;
    int         ret = 0;

    /* Refresh PA Instance */
    if (fCache)
    {
        Pa_osalMtCsEnter(&mtCsKey);
        Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
    }
    num64bCounters = paInst->cfg.usrStatsConfig.num64bCounters;
    numCounters    = paInst->cfg.usrStatsConfig.numCounters;

    if (cntIndex > numCounters)
    {
        ret = -1;
    } 
    else if (!pa_bitmap_verify(paInst->usrStatsAllocBitmap, PA_USR_STATS_BITMAP_SIZE, cntIndex) && paLObj.cfg.rmServiceHandle)
    {

        if (cntIndex < num64bCounters)
        {
            rmInst = 1;
            rmCntIndex = cntIndex;
        }
        else
        {
            rmInst = 0;
            rmCntIndex = cntIndex - num64bCounters;
        }

        if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmUsrStats[rmInst], &rmCntIndex, NULL))
        {
            ret = -1;
        }
        else
        {
            /* The counter is valid, release the resource to simulate permission check */
            pa_rmService (Rm_service_RESOURCE_FREE, rmUsrStats[rmInst], &rmCntIndex, NULL);
        }
    }

    if (!ret)
    {
        /* Mark the bit in the allocation bitMap */ 
        pa_bitmap_update(paInst->usrStatsAllocBitmap, PA_USR_STATS_BITMAP_SIZE, cntIndex, TRUE);
    
    }

    if (fCache)
    {
        Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
        Pa_osalMtCsExit(mtCsKey);  
    }
    return (ret);
} /* pa_verify_usr_stats */


/*******************************************************************************************
 * FUNCTION PURPOSE: Derive the destination queue Id
 *******************************************************************************************
 * DESCRIPTION: This function is used to derive the destination queue Id with the embedded
 *              control bits based on the input queue Id, the Queue Bounce configuration
 *              and the traffic class associated with the originated API.
 *
 *  return queueId with embedded control bits
 *
 ********************************************************************************************/
static uint16_t  pa_convert_queue_id(paInst_t *paInst, int routingClass, uint16_t queue)
{
    paQueueBounceConfig_t* pQueueBounceCfg = &paInst->cfg.queueBounceConfig;
    uint16_t queueBounceCtrlType = queue >> pa_QUEUE_BOUNCE_CTRL_LOC;
    uint16_t queueNum = queue & pa_QUEUE_BOUNCE_QUEUE_MASK;

    if (!pQueueBounceCfg->enable)
    {
        return(queueNum);
    }

    if ((queueNum >= pQueueBounceCfg->hwQueueBegin) &&
       (queueNum <= pQueueBounceCfg->hwQueueEnd))
    {
        /* It is a hardware queue, queue bounce is not required */
        return(queueNum);
    }

    if (queueBounceCtrlType == pa_QUEUE_BOUNCE_CTRL_DEFAULT)
    {
        /*
        * Error check is not required here because routingClass is set by internal
        * function and the range check of default operation mode has been performed
        * at Queue Bounce Configuration time
        */
        uint16_t defOp = pQueueBounceCfg->defOp[routingClass];

        return(queueNum | (defOp << pa_QUEUE_BOUNCE_CTRL_LOC));
    }
    else if ((queueBounceCtrlType == pa_QUEUE_BOUNCE_CTRL_DDR) ||
             (queueBounceCtrlType == pa_QUEUE_BOUNCE_CTRL_MSMC))
    {
        return(queue);
    }
    else /* pa_QUEUE_BOUNCE_CTRL_NONE */
    {
        return(queueNum);
    }
}

/*************************************************************************
 * FUNCTION PURPOSE: Convert RouteInfo 
 ************************************************************************* 
 * DESCRIPTION: Convert old style of routing information to new data structure
 *************************************************************************/
static void pa_convert_routeInfo_to_routeInfo2(paRouteInfo_t *pRouteInfo, paRouteInfo2_t *pRouteInfo2)
{
    memset(pRouteInfo2, 0, sizeof(paRouteInfo2_t));
    
    /* Note the first portion of paRouteInfo2_t matches paRouteInfo_t exactly */
    memcpy(&pRouteInfo2->dest, pRouteInfo, sizeof(paRouteInfo_t));
    
    if(pRouteInfo->mRouteIndex != pa_NO_MULTI_ROUTE)pRouteInfo2->validBitMap |= pa_ROUTE_INFO_VALID_MROUTEINDEX;
    if(pRouteInfo->pktType_emacCtrl)pRouteInfo2->validBitMap |= pa_ROUTE_INFO_VALID_PKTTYPE_EMAC; 
    if(pRouteInfo->pCmd)pRouteInfo2->validBitMap |= pa_ROUTE_INFO_VALID_PCMD;
}

/*************************************************************************
 * FUNCTION PURPOSE: Convert ipInfo 
 ************************************************************************* 
 * DESCRIPTION: Convert old style of IP information to new data structure
 *************************************************************************/
static void pa_convert_ipInfo_to_ipInfo2(paIpInfo_t *pIpInfo, paIpInfo2_t *pIpInfo2)
{
    memset(pIpInfo2, 0, sizeof(paIpInfo2_t));
    
    if (pIpInfo->ipType == pa_IPV4)
    {
        if (!pa_uint8_zero(&pIpInfo->src.ipv4[0], 4))
        {
            memcpy(&pIpInfo2->src.ipv4[0], &pIpInfo->src.ipv4[0], 4);
            pIpInfo2->validBitMap |= pa_IP_INFO_VALID_SRC;
        }
        
        if (!pa_uint8_zero(&pIpInfo->dst.ipv4[0], 4))
        {
            memcpy(&pIpInfo2->dst.ipv4[0], &pIpInfo->dst.ipv4[0], 4);
            pIpInfo2->validBitMap |= pa_IP_INFO_VALID_DST;
        }
    }
    else
    {
    
        if (!pa_uint8_zero(&pIpInfo->src.ipv6[0], 16))
        {
            memcpy(&pIpInfo2->src.ipv6[0], &pIpInfo->src.ipv6[0], 16);
            pIpInfo2->validBitMap |= pa_IP_INFO_VALID_SRC;
        }
        
        if (!pa_uint8_zero(&pIpInfo->dst.ipv6[0], 16))
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
static void pa_convert_ethInfo_to_ethInfo2(paEthInfo_t *pEthInfo, paEthInfo2_t *pEthInfo2)
{
    memset(pEthInfo2, 0, sizeof(paEthInfo2_t));
    
    if (!pa_uint8_zero(&pEthInfo->src[0], pa_MAC_ADDR_SIZE))
    {
        memcpy(&pEthInfo2->src[0], &pEthInfo->src[0], pa_MAC_ADDR_SIZE);
        pEthInfo2->validBitMap |= pa_ETH_INFO_VALID_SRC;
    }
    
    if (!pa_uint8_zero(&pEthInfo->dst[0], pa_MAC_ADDR_SIZE))
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


/*************************************************************************
 * FUNCTION PURPOSE: Format API2 common parameters 
 ************************************************************************* 
 * DESCRIPTION: Format API2 parameter structure from API parameters
 *************************************************************************/
static void pa_format_paramDesc(paParamDesc *params,
                                int lutInst,
                                int index,
                                paHandleL2L3_t prevLink,
                                paHandleL2L3_t nextLink,
                                paRouteInfo2_t *routeInfo,
                                paRouteInfo2_t *nextRtFai
                                )
{
   
    memset(params, 0, sizeof(paParamDesc));
    
    params->lutInst = lutInst;
    params->index = index;
    params->prevLink = prevLink;
    params->nextLink = nextLink;
    params->routeInfo = routeInfo;
    params->nextRtFail = nextRtFai;
    
    if(lutInst != pa_LUT_INST_NOT_SPECIFIED)
        params->validBitMap |= pa_PARAM_VALID_LUTINST;
   
    if(index != pa_LUT1_INDEX_NOT_SPECIFIED)
        params->validBitMap |= pa_PARAM_VALID_INDEX;
   
    if(prevLink)
        params->validBitMap |= pa_PARAM_VALID_PREVLINK;
   
    if(nextLink)
        params->validBitMap |= pa_PARAM_VALID_NEXTLINK;
}

/*************************************************************************
 * FUNCTION PURPOSE: Format LUT2 API2 common parameters 
 ************************************************************************* 
 * DESCRIPTION: Format LUT2 API2 parameter structure from API parameters
 *************************************************************************/
static void pa_format_lut2ParamDesc(paLut2ParamDesc *params,
                                    int fReplace,
                                    uint16_t divertQ)
{
   
    memset(params, 0, sizeof(paLut2ParamDesc));
    
    if (fReplace)
    {
        params->validBitMap |= pa_LUT2_PARAM_VALID_CTRL_BITMAP;
        params->ctrlFlags |= pa_LUT2_INFO_CONTROL_FLAG_REPLACE;
    }
    
    if (divertQ != pa_PARAMS_NOT_SPECIFIED)
    {
        params->validBitMap |= pa_LUT2_PARAM_VALID_DIVERTQ;
        params->divertQ = divertQ;
    }
}

/*************************************************************************
 * FUNCTION PURPOSE: Convert Routing Info2
 ************************************************************************* 
 * DESCRIPTION: Convert the application routing information into the
 *              firmware forward information
 * Returns FALSE if error occurs
 *************************************************************************/
static paReturn_t pa_conv_routing_info2 (paInst_t *paInst, pafrmForward_t *fwdInfo, paRouteInfo2_t *routeInfo, int cmdDest, uint16_t failRoute, int routingClass)
{

  /*
   * The command location is the same in all destination modes except SRIO.
   * Initialize pCmd here to save code space.
   *
   * CmdSet is supported in all destination modes except CONTINUE_PARSE
   * Initialize fcmdSetNotSupport to FALSE as default
   */
                                      
  uint8_t psFlags = 0;                                    
  uint8_t *pCmd = fwdInfo->u.host.cmd;
  int     fcmdSetNotSupport = FALSE;
  uint16_t queue = routeInfo->queue;
  
  if ((routeInfo->dest == pa_DEST_HOST) || (routeInfo->dest == pa_DEST_EMAC)) {
    if (routeInfo->validBitMap & pa_ROUTE_INFO_VALID_PKTTYPE_EMAC) 
    {
      psFlags  = (routeInfo->pktType_emacCtrl & pa_EMAC_CTRL_CRC_DISABLE)?
                  PAFRM_ETH_PS_FLAGS_DISABLE_CRC:0;
      psFlags |= ((routeInfo->pktType_emacCtrl & pa_EMAC_CTRL_PORT_MASK) << PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
    }
  }
  
  if (routeInfo->dest == pa_DEST_HOST)  {
    fwdInfo->forwardType    = PAFRM_FORWARD_TYPE_HOST;
    fwdInfo->u.host.context = SWIZ(routeInfo->swInfo0);
    fwdInfo->u.host.psFlags = SWIZ(psFlags);
    
    if (routeInfo->validBitMap & pa_ROUTE_INFO_VALID_PRIORITY_TYPE)
    {
      if(routeInfo->priorityType == pa_ROUTE_PRIORITY_VLAN)
      {
        fwdInfo->u.host.ctrlBitMap   |= PAFRM_ROUTING_PRIORITY_VLAN_ENABLE;
        routingClass = pa_QUEUE_BOUNCE_ROUTING_CLASS_QoS;
      }
      else if(routeInfo->priorityType == pa_ROUTE_PRIORITY_DSCP)
      {
        fwdInfo->u.host.ctrlBitMap   |= PAFRM_ROUTING_PRIORITY_DSCP_ENABLE;
        routingClass = pa_QUEUE_BOUNCE_ROUTING_CLASS_QoS;
      }
      else if (routeInfo->priorityType ==  pa_ROUTE_INTF)
      {
        fwdInfo->u.host.ctrlBitMap   |= PAFRM_ROUTING_IF_DEST_SELECT_ENABLE;
      }
      else if (routeInfo->priorityType ==  pa_ROUTE_INTF_W_FLOW)
      {
        fwdInfo->u.host.ctrlBitMap   |= (PAFRM_ROUTING_IF_DEST_SELECT_ENABLE |
                                         PAFRM_ROUTING_FLOW_IF_BASE_ENABLE);
      }
      else if (routeInfo->priorityType == pa_ROUTE_EQoS_MODE)
      {
        /* There must be valid Egress port in this mode */
        if ((!(routeInfo->validBitMap & pa_ROUTE_INFO_VALID_PKTTYPE_EMAC)) ||
           (!(routeInfo->pktType_emacCtrl & pa_EMAC_CTRL_PORT_MASK)))
        {
          return (pa_ERR_CONFIG);
        }
           
        routingClass = pa_QUEUE_BOUNCE_ROUTING_CLASS_QoS;
        fwdInfo->u.host.ctrlBitMap   |= (PAFRM_ROUTING_IF_DEST_SELECT_ENABLE |
                                         PAFRM_ROUTING_FLOW_EQOS_IF_ENABLE);
      }
      else
      {
        return (pa_ERR_CONFIG);
      }
    }

    queue = pa_convert_queue_id(paInst, routingClass, queue);

    if (routeInfo->validBitMap & pa_ROUTE_INFO_VALID_MROUTEINDEX)  {
      if (routeInfo->mRouteIndex >= pa_MAX_MULTI_ROUTE_SETS) {
        return (pa_ERR_CONFIG);
      }
      fwdInfo->u.host.ctrlBitMap   |= PAFRM_MULTIROUTE_ENABLE;
      fwdInfo->u.host.multiIdx     = (uint8_t)routeInfo->mRouteIndex;
      fwdInfo->u.host.paPdspRouter = PAFRM_DEST_PA_M_0;
    }
    
  } else if (routeInfo->dest == pa_DEST_DISCARD)  {
    fwdInfo->forwardType = PAFRM_FORWARD_TYPE_DISCARD;

  } else if (routeInfo->dest == pa_DEST_EMAC)  {
    fwdInfo->forwardType        = PAFRM_FORWARD_TYPE_ETH;
    fwdInfo->u.eth.psFlags      = SWIZ(psFlags);
  
  } else if (failRoute) {
    
    return (pa_ERR_CONFIG);

  } else if (((routeInfo->dest == pa_DEST_CONTINUE_PARSE_LUT1) && (routeInfo->customType != pa_CUSTOM_TYPE_LUT2))|| 
             ((routeInfo->dest == pa_DEST_CONTINUE_PARSE_LUT2) && (routeInfo->customType != pa_CUSTOM_TYPE_LUT1)))  {
             
    /* Custom Error check */
    if (((routeInfo->customType == pa_CUSTOM_TYPE_LUT1) && (routeInfo->customIndex >= pa_MAX_CUSTOM_TYPES_LUT1)) || 
        ((routeInfo->customType == pa_CUSTOM_TYPE_LUT2) && (routeInfo->customIndex >= pa_MAX_CUSTOM_TYPES_LUT2)) ) {
       return(pa_ERR_CONFIG); 
    }    
             
    fwdInfo->forwardType = PAFRM_FORWARD_TYPE_PA;
    fwdInfo->u.pa.customType = (uint8_t)routeInfo->customType;
    fwdInfo->u.pa.customIdx = SWIZ(routeInfo->customIndex); 

    if (routeInfo->dest == pa_DEST_CONTINUE_PARSE_LUT2) {
        
       fwdInfo->u.pa.paDest = PAFRM_DEST_PA_C2; 
        
    }
    else {
      /*
       * cmdDest is provided by calling function 
       * There is no need to check error case 
       */
      fwdInfo->u.pa.paDest = (cmdDest == pa_CMD_TX_DEST_0)?PAFRM_DEST_PA_C1_1:PAFRM_DEST_PA_C1_2; 
    }
    
    fcmdSetNotSupport = TRUE;

  } else if (routeInfo->dest == pa_DEST_CASCADED_FORWARDING_LUT1) {
    fwdInfo->forwardType = PAFRM_FORWARD_TYPE_PA;
    fwdInfo->u.pa.paDest = (cmdDest == pa_CMD_TX_DEST_0)?PAFRM_DEST_PA_C1_1:PAFRM_DEST_PA_C1_2;
    fwdInfo->u.pa.ctrlBitMap   |= PAFRM_CASCADED_FORWARDING;
    fcmdSetNotSupport = TRUE;
  
  } else if (routeInfo->dest == pa_DEST_SASS)  {
    fwdInfo->forwardType        = PAFRM_FORWARD_TYPE_SA;
    fwdInfo->u.sa.swInfo0       = SWIZ(routeInfo->swInfo0);
    fwdInfo->u.sa.swInfo1       = SWIZ(routeInfo->swInfo1);
    
  }  else if (routeInfo->dest == pa_DEST_SRIO)  {
    fwdInfo->forwardType        = PAFRM_FORWARD_TYPE_SRIO;
    fwdInfo->u.srio.psInfo0     = SWIZ(routeInfo->swInfo0);
    fwdInfo->u.srio.psInfo1     = SWIZ(routeInfo->swInfo1);
    fwdInfo->u.srio.pktType     = SWIZ(routeInfo->pktType_emacCtrl);
    pCmd = NULL;
  
  } else  {

    return (pa_ERR_CONFIG);
  }
  
  fwdInfo->flowId = routeInfo->flowId;
  fwdInfo->queue = SWIZ(queue);

  if (pCmd && (routeInfo->validBitMap & pa_ROUTE_INFO_VALID_PCMD))  {
  
    paCmdInfo_t* paCmd = routeInfo->pCmd;
    paPatchInfo_t       *pPatchInfo;
    paCmdSet_t          *pCmdSet;
    paCmdUsrStats_t     *pUsrStats;
    paCmdSetUsrStats_t  *pCmdSetUsrStats; 
    
    switch (paCmd->cmd)
    {
      case pa_CMD_PATCH_DATA:
        pPatchInfo = &paCmd->params.patch;
        
        if((pPatchInfo->nPatchBytes > 2) || (!(pPatchInfo->ctrlBitfield & pa_PATCH_OP_INSERT)) || (pPatchInfo->patchData == NULL))
          return (pa_ERR_CONFIG); 
          
        pCmd[0] = PAFRM_RX_CMD_PATCH_DATA;
        pCmd[1] = pPatchInfo->nPatchBytes;
        pCmd[2] = pPatchInfo->patchData[0];
        pCmd[3] = pPatchInfo->patchData[1];
        break;
      
      case pa_CMD_CMDSET:
        pCmdSet = &paCmd->params.cmdSet;
        if(fcmdSetNotSupport || (pCmdSet->index >= paInst->cfg.cmdSetConfig.numCmdSets))
          return (pa_ERR_CONFIG); 
        pCmd[0] = PAFRM_RX_CMD_CMDSET;
        pCmd[1] = (uint8_t)pCmdSet->index; 
        break;
        
      case pa_CMD_USR_STATS:
        pUsrStats = &paCmd->params.usrStats;
        if(pUsrStats->index >= paInst->cfg.usrStatsConfig.numCounters)
          return (pa_ERR_CONFIG); 
          
        if(pa_verify_usr_stats(paInst, (int32_t)pUsrStats->index, FALSE))
          return (pa_RESOURCE_USE_DENIED);                      
          
        pCmd[0] = PAFRM_RX_CMD_USR_STATS;
        pCmd[1] = 4; 
        pCmd[2] = pUsrStats->index >> 8;
        pCmd[3] = pUsrStats->index & 0xFF;
        break;
        
      case pa_CMD_CMDSET_AND_USR_STATS:
        pCmdSetUsrStats = &paCmd->params.cmdSetUsrStats;
        if(fcmdSetNotSupport || 
           (pCmdSetUsrStats->setIndex >= paInst->cfg.cmdSetConfig.numCmdSets) || 
           (pCmdSetUsrStats->statsIndex >= paInst->cfg.usrStatsConfig.numCounters))
          return (pa_ERR_CONFIG); 
          
        if(pa_verify_usr_stats(paInst, (int32_t)pCmdSetUsrStats->statsIndex, FALSE))
          return (pa_RESOURCE_USE_DENIED);                      
          
        pCmd[0] = PAFRM_RX_CMD_CMDSET_USR_STATS;
        pCmd[1] = (uint8_t)pCmdSetUsrStats->setIndex; 
        pCmd[2] = pCmdSetUsrStats->statsIndex >> 8;
        pCmd[3] = pCmdSetUsrStats->statsIndex & 0xFF;
        
        break;
      
      default:
        return(pa_ERR_CONFIG);
    }    
  
  }
  return (pa_OK);
} /* pa_conv_routing_info2 */

/*************************************************************************
 * FUNCTION PURPOSE: Wrapper for pa_conv_routing_info
 ************************************************************************* 
 * DESCRIPTION: Convert the application routing information into the
 *              firmware forward information
 * Returns FALSE if error occurs
 *************************************************************************/
static paReturn_t pa_conv_routing_info (paInst_t *paInst, pafrmForward_t *fwdInfo, paRouteInfo_t *routeInfo, int cmdDest, uint16_t failRoute, int routingClass)
{
  paRouteInfo2_t routeInfo2; 
  memset(&routeInfo2, 0, sizeof(paRouteInfo2_t));
  routeInfo2.dest     = routeInfo->dest;
  routeInfo2.flowId   = routeInfo->flowId;
  routeInfo2.queue    = routeInfo->queue;
  
  if (routeInfo->mRouteIndex != pa_NO_MULTI_ROUTE)
  {
    routeInfo2.validBitMap |= pa_ROUTE_INFO_VALID_MROUTEINDEX;
    routeInfo2.mRouteIndex = routeInfo->mRouteIndex;
  }

  routeInfo2.swInfo0           = routeInfo->swInfo0;
  routeInfo2.swInfo1           = routeInfo->swInfo1;
  routeInfo2.customType        = routeInfo->customType;
  routeInfo2.customIndex       = routeInfo->customIndex;
  
  if (routeInfo->pktType_emacCtrl)
  {
    routeInfo2.validBitMap |= pa_ROUTE_INFO_VALID_PKTTYPE_EMAC;
    routeInfo2.pktType_emacCtrl  = routeInfo->pktType_emacCtrl;
  }
  
  if (routeInfo->pCmd)
  {
    routeInfo2.validBitMap |= pa_ROUTE_INFO_VALID_PCMD;
    routeInfo2.pCmd         = routeInfo->pCmd;
  }
  
  return pa_conv_routing_info2(paInst, fwdInfo, &routeInfo2, cmdDest, failRoute, routingClass);
}

/***************************************************************************
 * FUNCTION PURPOSE: setup the firmware command for emac port configuration
 ***************************************************************************
 * DESCRIPTION: setup the firmware command for emac port configuration
 ***************************************************************************/
static paReturn_t pa_set_emac_port_cfg_frm_cmd(
                                         paInst_t                  *paInst,
                                         pafrmCommandSysConfigPa_t *sysCfgPtr, 
                                         paEmacPortConfig_t        *emacPortCfg,
                                         int                       *cmdDest
                                        )
{
  pafrmCommandSysConfigPa_t* ccfg = sysCfgPtr;
  int                        i;
  uint8_t                    numEntries = emacPortCfg->numEntries;

  /* error checking */
  if (numEntries > pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES_GEN1)
    return (pa_ERR_CONFIG);

  switch (emacPortCfg->cfgType)
  {
    case pa_EMAC_PORT_CFG_PKT_CAPTURE:
    {
      pafrmPktCapCfg_t      *pktCapfrmCfg;
      paPktCaptureConfig_t  *pktCapCfg    = emacPortCfg->u.pktCapCfg;
      uint32_t               direction    = pktCapCfg->ctrlBitMap & pa_PKT_CLONE_INGRESS;
      uint32_t               enable       = pktCapCfg->ctrlBitMap & pa_PKT_CLONE_ENABLE;
      uint32_t               swInfo0;
      uint16_t               queue;
      uint8_t                capturePort, flow, ctrlBitMap;

      if (direction) /* ingress capture command */
      {
        ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_IGRESS_PCAP;  
        pktCapfrmCfg = &ccfg->u.igressPktCapCfg;        
       
      }
      else /* egress capture command */
      {
        ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_EGRESS_PCAP;
        pktCapfrmCfg = &ccfg->u.egressPktCapCfg;          
      }

      /* Clear the command structure */
      memset(pktCapfrmCfg, 0, sizeof (pafrmPktCapCfg_t));

      /* record number of entries configured in the command */
      pktCapfrmCfg->numPorts = numEntries;      

      for (i = 0; i < numEntries; i++)
      {
        /* Obtain the information from the configuration */
        capturePort = pktCapCfg[i].portToBeCaptured;
        queue       = pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_CAPTURE, pktCapCfg[i].queue);
        flow        = pktCapCfg[i].flowId;
        swInfo0     = pktCapCfg[i].swInfo0;


        /* report error when capture port is invalid */
        if (capturePort >  pa_EMAC_PORT_3)
          return (pa_ERR_CONFIG);
        
        /* report error when capture port is 0 (host port) for ingress capture */
        if (direction && (capturePort == pa_EMAC_PORT_NOT_SPECIFIED))
          return (pa_ERR_CONFIG);

        ctrlBitMap    = PAFRM_PKT_CAP_HOST;
      
        if (enable)
        {
          ctrlBitMap |= PAFRM_PKT_CAP_ENABLE;        
        }
        else
        {
          ctrlBitMap &= ~PAFRM_PKT_CAP_ENABLE;                
        }

        /* translate to firmware global configuration command*/
        pktCapfrmCfg->pafrmPktcCapCfg[i].ctrlBitmap    = SWIZ(ctrlBitMap);
        pktCapfrmCfg->pafrmPktcCapCfg[i].capturePort   = SWIZ(capturePort);
        pktCapfrmCfg->pafrmPktcCapCfg[i].queue         = SWIZ(queue);
        pktCapfrmCfg->pafrmPktcCapCfg[i].emacport_flow = SWIZ(flow);
        pktCapfrmCfg->pafrmPktcCapCfg[i].context       = SWIZ(swInfo0);        
      }
      
      /* packet capture at command */
      *cmdDest = pa_CMD_TX_DEST_0; 
      break;
    }
    case pa_EMAC_PORT_CFG_MIRROR:
    {
      pafrmPktCapCfg_t      *pktCapfrmCfg;
      paPortMirrorConfig_t  *pMirrorCfg   = emacPortCfg->u.mirrorCfg;
      uint32_t               direction    = pMirrorCfg->ctrlBitMap & pa_PKT_CLONE_INGRESS;
      uint32_t               enable       = pMirrorCfg->ctrlBitMap & pa_PKT_CLONE_ENABLE;
      uint8_t                port, mirrorPort, ctrlBitMap;

      if (direction) /* ingress capture command */
      {
        ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_IGRESS_PCAP;  
        pktCapfrmCfg = &ccfg->u.igressPktCapCfg;        
           
      }
      else /* egress capture command */
      {
        ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_EGRESS_PCAP;
        pktCapfrmCfg = &ccfg->u.egressPktCapCfg;          
      }

      /* Clear the command structure */
      memset(pktCapfrmCfg, 0, sizeof (pafrmPktCapCfg_t));
    
      /* record number of entries configured in the command */
      pktCapfrmCfg->numPorts = numEntries;      
    
      for (i = 0; i < numEntries; i++)
      {
        /* Obtain the information from the configuration */
        port        = pMirrorCfg[i].portToBeMirrored;
        mirrorPort  = pMirrorCfg[i].mirrorPort;
    
        /* report error when capture port is 0 (host port) */
        if ((port == pa_EMAC_PORT_NOT_SPECIFIED) ||
            (port >  pa_EMAC_PORT_3))
          return (pa_ERR_CONFIG);
    
        ctrlBitMap   = 0;
          
        if (enable)
        {
          ctrlBitMap |= PAFRM_PKT_CAP_ENABLE;        
        }
        else
        {
          ctrlBitMap &= ~PAFRM_PKT_CAP_ENABLE;                
        }

        /* translate to firmware global configuration command*/
        pktCapfrmCfg->pafrmPktcCapCfg[i].ctrlBitmap    = ctrlBitMap;
        pktCapfrmCfg->pafrmPktcCapCfg[i].capturePort   = port;
        pktCapfrmCfg->pafrmPktcCapCfg[i].emacport_flow = mirrorPort << 4;
      }
          
      /* packet capture at command */
      *cmdDest = pa_CMD_TX_DEST_0; 
      break;
    }
    case pa_EMAC_PORT_CFG_DEFAULT_ROUTE:
    {
      pafrmDefRouteCfg_t    *pDefRoutefrmCfg = &ccfg->u.defRouteCfg;
      paDefRouteConfig_t    *pDefRouteCfg = emacPortCfg->u.defRouteCfg;
      uint8_t                port, ctrlBitMap;
      paReturn_t             retCode;
      
      if(numEntries > PAFRM_MAX_EMAC_PORT)
        return (pa_ERR_CONFIG);

      ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_DEFAULT_ROUTE;

      /* Record number of entries */
      pDefRoutefrmCfg->numPorts = numEntries;
    
      for (i = 0; i < numEntries; i++)
      {
        /* Obtain the information from the configuration */
        port        = pDefRouteCfg[i].port;            
        
        /* report error when capture port is 0 (host port) */
        if ((port == pa_EMAC_PORT_NOT_SPECIFIED) ||
            (port >  pa_EMAC_PORT_3))
        {
          return (pa_ERR_CONFIG);
        }

        /* get to zero base now */
        port--;

        /* Get the control bit map */
        ctrlBitMap = (uint8_t)pDefRouteCfg[i].ctrlBitMap;
        
        /* translate to firmware global configuration command*/
        pDefRoutefrmCfg->routeCfg[i].ctrlBitmap    = SWIZ(ctrlBitMap);
        pDefRoutefrmCfg->routeCfg[i].port          = SWIZ(port);

        /* store multi cast default route information if enabled */
        if (ctrlBitMap & pa_EMAC_IF_DEFAULT_ROUTE_MC_ENABLE)
        {
          retCode = pa_conv_routing_info2 (paInst, &pDefRoutefrmCfg->routeCfg[i].dRoute[pa_DROUTE_MULTICAST], &pDefRouteCfg[i].dRouteInfo[pa_DROUTE_MULTICAST], pa_CMD_TX_DEST_5, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
          /* Error checking */
          if (retCode != pa_OK)
            return (retCode);
        }
        /* Store broad cast default route information if enabled */
        if (ctrlBitMap & pa_EMAC_IF_DEFAULT_ROUTE_BC_ENABLE)
        {
          retCode = pa_conv_routing_info2 (paInst, &pDefRoutefrmCfg->routeCfg[i].dRoute[pa_DROUTE_BROADCAST], &pDefRouteCfg[i].dRouteInfo[pa_DROUTE_BROADCAST], pa_CMD_TX_DEST_5, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
          /* Error checking */
          if (retCode != pa_OK)
            return (retCode);        
        }

        if (ctrlBitMap & pa_EMAC_IF_DEFAULT_ROUTE_UC_ENABLE)
        {
          retCode = pa_conv_routing_info2 (paInst, &pDefRoutefrmCfg->routeCfg[i].dRoute[pa_DROUTE_NO_MATCH], &pDefRouteCfg[i].dRouteInfo[pa_DROUTE_NO_MATCH], pa_CMD_TX_DEST_5, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
          /* Error checking */
          if (retCode != pa_OK)
            return (retCode);        
        }            
      }
              
      /* packet capture at command, can be any pdsp, 5 is used as it is lightly loaded */
      *cmdDest = pa_CMD_TX_DEST_5; 
      break;
    }
    case pa_EMAC_PORT_CFG_EQoS_MODE:
    {
      pafrmEQosModeConfig_t  *pafrmEQoSCfg = &ccfg->u.eqosCfg;
      paEQosModeConfig_t     *paEQosCfg    = emacPortCfg->u.eQoSModeCfg;
      uint8_t                port, ctrlBitMap, mapSize;
      paRouteOffset_t        *mapTbl, *frmMapTbl;
      int                    j;

      if(numEntries > PAFRM_MAX_EMAC_PORT)
        return (pa_ERR_CONFIG);

      ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_EQoS_MODE;

      /* Record global egress default priority */
      pafrmEQoSCfg->numPorts = numEntries;
      for (i = 0; i < numEntries; i++)
      {
        /* Obtain the information from the configuration */
        port        = paEQosCfg[i].port;            
        
        /* report error when capture port is 0 (host port) */
        if ((port == pa_EMAC_PORT_NOT_SPECIFIED) ||
            (port >  pa_EMAC_PORT_3))
        {
          return (pa_ERR_CONFIG);
        }

        /* get to zero base now */
        port--;

        /* Get the control bit map */
        ctrlBitMap = (uint8_t)paEQosCfg[i].ctrlBitMap;
        
        /* translate to firmware global configuration command*/
        pafrmEQoSCfg->eQoSCfg[i].ctrlBitMap    = SWIZ(ctrlBitMap);
        pafrmEQoSCfg->eQoSCfg[i].port          = SWIZ(port);
        pafrmEQoSCfg->eQoSCfg[i].flowBase      = SWIZ(paEQosCfg[i].flowBase);
        pafrmEQoSCfg->eQoSCfg[i].queueBase     = SWIZ(paEQosCfg[i].queueBase);
        pafrmEQoSCfg->eQoSCfg[i].ingressDefPri = SWIZ(paEQosCfg[i].ingressDefPri);
        pafrmEQoSCfg->eQoSCfg[i].vlanId        = SWIZ(paEQosCfg[i].vlanId);

        mapTbl  = &paEQosCfg[i].pbitMap[0];
        frmMapTbl = &pafrmEQoSCfg->eQoSCfg[i].pbitMap[0];          
        mapSize = 8;        

        for (j = 0; j < mapSize; j++)
        {
          frmMapTbl[j].flowOffset  = SWIZ(mapTbl[j].flowOffset);
          frmMapTbl[j].queueOffset = SWIZ(mapTbl[j].queueOffset);          
        }

        mapTbl    = &paEQosCfg[i].dscpMap[0];
        frmMapTbl = &pafrmEQoSCfg->eQoSCfg[i].dscpMap[0];
        mapSize = 64;
        
        for (j = 0; j < mapSize; j++)
        {
          frmMapTbl[j].flowOffset  = SWIZ(mapTbl[j].flowOffset);
          frmMapTbl[j].queueOffset = SWIZ(mapTbl[j].queueOffset);          
        }        
        
      }          
      /* packet capture at command, can be any pdsp, 4 is used as it is lightly loaded */
      *cmdDest = pa_CMD_TX_DEST_4; 
      break;
    }            
    default:
      return (pa_ERR_CONFIG);      
    }

  return (pa_OK);
}

/*************************************************************************
 * FUNCTION PURPOSE: Format Firmware Command Header
 ************************************************************************* 
 * DESCRIPTION: Clear and construct the firmware command header
 * Returns pointer to the firmware command
 *************************************************************************/
static pafrmCommand_t* pa_format_fcmd_header (paInst_t *paInst, void *pCmd, paCmdReply_t *reply, uint8_t cmd, uint16_t comId, uint16_t cmdSize)
{
  pafrmCommand_t *fcmd = (pafrmCommand_t *) pCmd;
  
  memset(fcmd, 0, cmdSize);

  fcmd->command       = SWIZ(cmd);
  fcmd->magic         = PAFRM_CONFIG_COMMAND_SEC_BYTE;
  fcmd->comId         = SWIZ(comId);
  fcmd->retContext    = SWIZ(reply->replyId);
  fcmd->replyQueue    = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_CMD_RET, reply->queue));
  fcmd->flowId        = SWIZ(reply->flowId);
  
  return(fcmd);
}  

/*************************************************************************
 * FUNCTION PURPOSE: Send a service request to RM
 ************************************************************************* 
 * DESCRIPTION: Sends a service request to RM and waits for response to
 *              provide to PA
 *************************************************************************/ 
int pa_rmService (Rm_ServiceType type, const char *resName, int32_t *resNum, int32_t *resCnt)
{
  Rm_ServiceHandle   *rmService = (Rm_ServiceHandle *)paLObj.cfg.rmServiceHandle;
  Rm_ServiceReqInfo   rmServiceReq;
  Rm_ServiceRespInfo  rmServiceResp;   
  
  memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
  memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));
  
  rmServiceReq.type = type;
  rmServiceReq.resourceName = resName;
  if (*resNum >= 0) {
    rmServiceReq.resourceBase = *resNum;
  }
  else {
    rmServiceReq.resourceBase = RM_RESOURCE_BASE_UNSPECIFIED;
  }
  rmServiceReq.resourceLength = 1;
  /* RM will block until resource is returned since callback is NULL */
  rmServiceReq.callback.serviceCallback = NULL;
  rmService->Rm_serviceHandler(rmService->rmHandle, &rmServiceReq, &rmServiceResp);
  if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
      (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC)) {
    /* Only return value through resNum if the service type is
     * ALLOCATE... */
    if ((type == Rm_service_RESOURCE_ALLOCATE_INIT) ||
        (type == Rm_service_RESOURCE_ALLOCATE_USE)) {
      *resNum = rmServiceResp.resourceBase;
    } 
    
    if(resCnt)
      *resCnt = rmServiceResp.instAllocCount;
          
    return (1);
  }
  return (0);
}

/****************************************************************************
 * FUNCTION PURPOSE: Add a MAC address to the lookup table
 ****************************************************************************
 * DESCRIPTION: A MAC address is added to the L2 table.
 *              All L2 lookups are handled through PDSP0
 ****************************************************************************/
paReturn_t Pa_addMac  (  Pa_Handle          iHandle,
                         int                index,
                         paEthInfo_t       *ethInfo,
                         paRouteInfo_t     *routeInfo,
                         paRouteInfo_t     *nextRtFail,
                         paHandleL2L3_t    *handle,
                         paCmd_t            cmd,
                         uint16_t          *cmdSize,
                         paCmdReply_t      *reply,
                         int               *cmdDest )
{

  paParamDesc params;
  paEthInfo2_t ethInfo2;
  paRouteInfo2_t matchRoute2;
  paRouteInfo2_t nfailRoute2;
  paReturn_t  ret;
  
  if((ethInfo == NULL) || (routeInfo == NULL) || (nextRtFail == NULL) || (handle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  pa_convert_ethInfo_to_ethInfo2(ethInfo, &ethInfo2);
  pa_convert_routeInfo_to_routeInfo2(routeInfo,  &matchRoute2);
  pa_convert_routeInfo_to_routeInfo2(nextRtFail, &nfailRoute2);
  pa_format_paramDesc(&params, pa_LUT_INST_NOT_SPECIFIED, index, NULL, NULL, &matchRoute2, &nfailRoute2);
  
  ret = Pa_addMac2 (iHandle,
                    &ethInfo2,
                    &params,
                    handle,
                    cmd,
                    cmdSize,
                    reply,
                    cmdDest);

  return (ret);

} /* Pa_addMac */

/****************************************************************************
 * FUNCTION PURPOSE: Add a MAC address to the lookup table
 ****************************************************************************
 * DESCRIPTION: A MAC address is added to the L2 table.
 *              All L2 lookups are handled through PDSP0
 ****************************************************************************/
paReturn_t Pa_addMac2  (  Pa_Handle    iHandle,
                          paEthInfo2_t *ethInfo,    /**<  Value @ref paEthInfo2_t */
                          paParamDesc  *params,
                          paLnkHandle_t*handle,     /**<  Pointer to the returned L2 handle */
                          paCmd_t      cmd,
                          uint16_t     *cmdSize,
                          paCmdReply_t *reply,
                          int          *cmdDest
                       )
{
  /* The entry is created in the stack and then copied to the table. This allows
   * for a comparison with other table entries to verify there is not an identical
   * entry or an entry that would supercede this one */

  paInst_t               *paInst    = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL2Entry_t            *l2Table;
  paL2Entry_t             l2Entry;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  int                     i;
  uint16_t                csize;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey;  
  paVirtualLnk_t          *vlnkTable;

  /* Param parsing */ 
  paLnkHandle_t   nextLink;
  uint8_t         lut1Index;
  paRouteInfo2_t  *routeInfo;
  paRouteInfo2_t  *nextRtFail;

  if((ethInfo == NULL) || (params == NULL) || (params->routeInfo == NULL) || (params->nextRtFail == NULL) || (handle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  nextLink = (params->validBitMap & pa_PARAM_VALID_NEXTLINK)?((paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,params->nextLink)):NULL;
  lut1Index = (params->validBitMap & pa_PARAM_VALID_INDEX)?(uint16_t)params->index:PAFRM_LUT1_INDEX_LAST_FREE;
  routeInfo = params->routeInfo;
  nextRtFail = params->nextRtFail;

  if (paLObj.cfg.rmServiceHandle) {
    int32_t dstPdsp = pa_CMD_TX_DEST_0;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &dstPdsp, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }

	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &dstPdsp, NULL)) {
      return pa_RESOURCE_FREE_DENIED;
    }
  }
 
  /* Sanity chek the LUT1 index */
  if (lut1Index > PAFRM_HW_LUT1_ENTRIES) {
    return(pa_INVALID_LUT1_INDEX);
  }

  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+sizeof(pafrmCommandAddLut1_t)-sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  /* Return the actual size of the buffer used */
  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);
  
  /* Form the table entry. Status is not required */
  memset (&l2Entry, 0, sizeof(paL2Entry_t));
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_DST)
    memcpy (l2Entry.cfg.mac.dstMac,  ethInfo->dst,  sizeof(paMacAddr_t));
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_SRC)
    memcpy (l2Entry.cfg.mac.srcMac,  ethInfo->src,  sizeof(paMacAddr_t));
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN)
    l2Entry.cfg.mac.vlan       =  ethInfo->vlan;
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_ETHERTYPE)
    l2Entry.cfg.mac.ethertype  =  ethInfo->ethertype;
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_MPLSTAG)
    l2Entry.cfg.mac.mplsTag    =  ethInfo->mplsTag;
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_INPORT)
    l2Entry.cfg.mac.inport     =  ethInfo->inport;
    
  /* The MPLS label is restricted to 20 bits */
  if (l2Entry.cfg.mac.mplsTag & ~PA_MPLS_LABEL_MASK)  
     return(pa_INVALID_MPLS_LABEL);
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  l2Table	= (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);	

  /* Signal the application that a table modification will be done */
  Pa_osalBeginMemAccess ((void*) l2Table, 
					   paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);

  /* Replace the index might have been requested 
   * Check if that is the case
   */
  if (params->validBitMap & pa_PARAM_VALID_CTRLBITMAP) {
    /* Check if replace command is set */
    if (params->ctrlBitMap & pa_PARAM_CTRL_REPLACE) {        
      paL2Entry_t *l2EntryRep = (paL2Entry_t *) *handle;        
      /* If the replace index entry provided is not active,
       * flag the error to application 
       * Also, The replace index requested can not be 
       * different from original lut index 
       */        
      if (l2EntryRep->hdr.status != PA_TBL_STAT_ACTIVE )
      {
        ret = pa_INVALID_INPUT_HANDLE;
        goto Pa_addMac2_end;          
      }

      /* Specified the lut1Index for replacement */
      if (lut1Index != PAFRM_LUT1_INDEX_LAST_FREE) {
        if (lut1Index != l2EntryRep->hdr.lutIdx) {
          ret = pa_INVALID_INPUT_HANDLE;
          goto Pa_addMac2_end;
        }
      }
      else { /* did not specify the lut1 index for replacement */
        lut1Index = l2EntryRep->hdr.lutIdx;
      }

      /* No need to find the free entry as it is replace command */
      i = l2EntryRep->hdr.tableIdx;

      /* update the l3Table with the entry to be replaced */
      goto Pa_addMac2_send_cmd;
    }
  }

  /* Look for an identical entry in the table. If one is found, return it */
  /* perform entry check only if the LUT1 index is not specified by user */
  if (lut1Index == PAFRM_LUT1_INDEX_LAST_FREE) {
    for (i = 0; i < paInst->l2n;  i++)  {

      if ( ((l2Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (l2Table[i].hdr.status == PA_TBL_STAT_ACTIVE)             ) &&
           (l2Table[i].hdr.subType == PA_TABLE_ENTRY_SUBTYPE_MAC )) {

        if (!memcmp (&(l2Table[i].cfg), &(l2Entry.cfg), sizeof(paL2InCfg_t)))  {
          /* Identical entry identified */
          if (l2Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) {
            *handle = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l2Table[i]);
            ret = pa_INVALID_DUP_ENTRY;
            goto Pa_addMac2_end;
          }                     
          else {
              /*  
               * Identical entry is identified, reuse the same table entry
               * Keep the entry status since it simply replaces the original entry
               * and the user may decide not to forward the packet
               */ 
            lut1Index = l2Table[i].hdr.lutIdx;
            ret = pa_DUP_ENTRY;   
            goto Pa_addMac2_send_cmd;
          }                  
        }
      }
    }


    /* Check to see if this entry is more general then one already in the table.
     * If so then this entry would 'steal' packets from that entry.
     */

    for (i = 0; i < paInst->l2n; i++)  {

      if ( (l2Table[i].hdr.status != PA_TBL_STAT_PENDING_SUBS_REPLY)  &&
           (l2Table[i].hdr.status != PA_TBL_STAT_ACTIVE)              )
        continue;
        
      if ( l2Table[i].hdr.subType != PA_TABLE_ENTRY_SUBTYPE_MAC )  
        continue;

      if (!pa_mac_match (l2Entry.cfg.mac.dstMac, l2Table[i].cfg.mac.dstMac))
        continue;

      if (!pa_mac_match (l2Entry.cfg.mac.srcMac, l2Table[i].cfg.mac.srcMac))
        continue;

      if (!pa_u16_match (l2Entry.cfg.mac.vlan, l2Table[i].cfg.mac.vlan))
        continue;

      if (!pa_u16_match (l2Entry.cfg.mac.ethertype, l2Table[i].cfg.mac.ethertype))
        continue;

      if (!pa_u32_match (l2Entry.cfg.mac.mplsTag, l2Table[i].cfg.mac.mplsTag))
        continue;
        
      if (!pa_u16_match (l2Entry.cfg.mac.inport, l2Table[i].cfg.mac.inport))
        continue;
        

      /* This is a more general entry then one already in the table */
      ret = pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT;
      goto Pa_addMac2_end;
    }
  }


  /* Find a free entry in the table */
  for (i = 0; i < paInst->l2n; i++)  {

    if (l2Table[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;

  }

  if (i == paInst->l2n)  {
    ret = pa_HANDLE_TABLE_FULL;
    goto Pa_addMac2_end;
  }
  
Pa_addMac2_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *handle = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l2Table[i]);

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_L2  | i, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  al1->index         = lut1Index;
  
  /* When next link is set check to see if the link is a virtual link 
   * Then set the flag to notify firmware that a virtual link will be used 
   * for the next stage. */
  if (nextLink != NULL) {
    /* nextLink must be a virtual link */
    if (!paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)
    {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addMac2_end;
    }
                           
    vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);
    Pa_osalBeginMemAccess ((void *) vlnkTable,
					   paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);

    if ((((paVirtualLnk_t *)nextLink)->type != PA_TABLE_ENTRY_TYPE_VL) ||
        (((paVirtualLnk_t *)nextLink)->status != PA_TBL_STAT_ACTIVE))
    {
      ret = pa_INVALID_INPUT_HANDLE;
      Pa_osalEndMemAccess ((void *) vlnkTable,
					 paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
      goto Pa_addMac2_end;
    }
    al1->type         = PAFRM_COM_ADD_LUT1_VLINK;
    al1->vLinkNum     = (uint8_t) (((paVirtualLnk_t *)nextLink)->tableIdx);
	  Pa_osalEndMemAccess ((void *) vlnkTable, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  }  else {
    al1->type         = PAFRM_COM_ADD_LUT1_STANDARD;
  }
  
  /* 
   * Workaround: ARM tool inconsistent issue which triggered Alignment Trap warning from User space
   * per two consecutive 6-byte memcpy operations:
   * memcpy (al1->u.ethIp.dmac, l2Entry.cfg.mac.dstMac, sizeof(paMacAddr_t));
   * memcpy (al1->u.ethIp.smac, l2Entry.cfg.mac.srcMac, sizeof(paMacAddr_t));
   */
  memcpy (al1->u.ethIp.dmac, l2Entry.cfg.mac.dstMac, sizeof(paMacAddr_t)*2);

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_DST)
  {
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_DMAC;
  }  

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_SRC)
  {
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_SMAC;
  }  

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_ETHERTYPE)
  {
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_ETYPE;
    al1->u.ethIp.etype = SWIZ(l2Entry.cfg.mac.ethertype);
  }
  
  if (ethInfo->validBitMap & pa_ETH_INFO_VALID_MPLSTAG) 
  {
    al1->u.ethIp.matchFlags |= PAFRM_LUT1_MATCH_MPLS;
    al1->u.ethIp.pm.mpls     = SWIZ(l2Entry.cfg.mac.mplsTag);
  }

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN)
  {
    al1->u.ethIp.matchFlags |= PAFRM_LUT1_MATCH_VLAN;
    al1->u.ethIp.vlan  = SWIZ(l2Entry.cfg.mac.vlan);
  }  
    
  if (ethInfo->validBitMap & pa_ETH_INFO_VALID_INPORT)
  {
    al1->u.ethIp.matchFlags |= PAFRM_LUT1_MATCH_PORT;
    al1->u.ethIp.inport = (uint8_t)l2Entry.cfg.mac.inport;
  }

  al1->u.ethIp.key        |= PAFRM_LUT1_KEY_MAC;
  al1->u.ethIp.key         = SWIZ(al1->u.ethIp.key);
  al1->u.ethIp.matchFlags |= PAFRM_LUT1_CUSTOM_MATCH_KEY;
  al1->u.ethIp.matchFlags  = SWIZ(al1->u.ethIp.matchFlags);

  /* Forwarding information */
  ret1 = pa_conv_routing_info2(paInst, &al1->match, routeInfo, pa_CMD_TX_DEST_0, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addMac2_end;
  }

  /* Next fail information */
  ret1 = pa_conv_routing_info2(paInst, &al1->nextFail, nextRtFail, pa_CMD_TX_DEST_0, TRUE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addMac2_end;
  }

  if (ret != pa_DUP_ENTRY)
  {
    /* Initialze, the table entry, add the status and pdsp NUM */
    l2Entry.hdr.type     = PA_TABLE_ENTRY_TYPE_L2;
    l2Entry.hdr.subType  = PA_TABLE_ENTRY_SUBTYPE_MAC;
    l2Entry.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
    l2Entry.hdr.pdspNum  = PASS_PDSP0;
    l2Entry.hdr.lutIdx   = -1;
    l2Entry.hdr.tableIdx = i;

    memcpy (&l2Table[i], &l2Entry, sizeof(paL2Entry_t));
  }

  /* The destination for this command must be PDSP 0 */
  *cmdDest = pa_CMD_TX_DEST_0;
  
Pa_addMac2_end:
  Pa_osalEndMemAccess ((void *) l2Table,
					 paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  
  
  return (ret);

} /* Pa_addMac2 */

/****************************************************************************
 * FUNCTION PURPOSE: Add a SRIO entry to the lookup table
 ****************************************************************************
 * DESCRIPTION: A SRIO entry is added to the L2 table. (LUT1_0)
 *              All L2 lookups are handled through PDSP0
 ****************************************************************************/
paReturn_t Pa_addSrio (  Pa_Handle         iHandle,
                         int               index,
                         paSrioInfo_t      *srioInfo,
                         uint16_t          nextHdr,
                         uint16_t          nextHdrOffset,
                         paRouteInfo_t     *routeInfo,
                         paRouteInfo_t     *nextRtFail,
                         paHandleL2L3_t    *handle,
                         paCmd_t           cmd,
                         uint16_t          *cmdSize,
                         paCmdReply_t      *reply,
                         int               *cmdDest)
{
  /* The entry is created in the stack and then copied to the table. This allows
   * for a comparison with other table entries to verify there is not an identical
   * entry or an entry that would supercede this one */

  paInst_t               *paInst    = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL2Entry_t            *l2Table;

  paL2Entry_t             l2Entry;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  int                     i;
  uint16_t                csize;
  uint8_t                 lut1Index = (index == pa_LUT1_INDEX_NOT_SPECIFIED)?PAFRM_LUT1_INDEX_LAST_FREE:(uint8_t)index;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey;   
       
  if((srioInfo == NULL) || (routeInfo == NULL) || (nextRtFail == NULL) || (handle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  if (paLObj.cfg.rmServiceHandle) {
    int32_t dstPdsp = pa_CMD_TX_DEST_0;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &dstPdsp, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }
	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &dstPdsp, NULL)) {
      return pa_RESOURCE_FREE_DENIED;
    }	
  }

  /* Sanity chek the LUT1 index */
  if (lut1Index > PAFRM_HW_LUT1_ENTRIES) {
    return(pa_INVALID_LUT1_INDEX);
  }
  
  if (nextHdr > pa_HDR_TYPE_UNKNOWN)
    return ( pa_ERR_CONFIG ); 

  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+sizeof(pafrmCommandAddLut1_t)-sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  /* Return the actual size of the buffer used */
  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);
  
  /* Form the table entry. Status is not required */
  memset(&l2Entry, 0, sizeof(paL2Entry_t));
  
  l2Entry.cfg.srio.validBitMap = srioInfo->validBitMap;
  
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_ID)   {
  
    if((srioInfo->tt != pa_SRIO_TRANSPORT_TYPE_0) && (srioInfo->tt != pa_SRIO_TRANSPORT_TYPE_1))
      return(pa_ERR_CONFIG);
      
    l2Entry.cfg.srio.tt       = (uint8_t) srioInfo->tt; 
      
    if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_SRC_ID) {
        l2Entry.cfg.srio.srcId = (srioInfo->tt == pa_SRIO_TRANSPORT_TYPE_0)?
                                 srioInfo->srcId & 0x00FF: srioInfo->srcId;   
    }  
    
    if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_DEST_ID) {
        l2Entry.cfg.srio.destId = (srioInfo->tt == pa_SRIO_TRANSPORT_TYPE_0)?
                                  srioInfo->destId & 0x00FF: srioInfo->destId;   
    }  
  }
  
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_CC)
    l2Entry.cfg.srio.cc       =  (uint8_t) srioInfo->cc;
    
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_PRI)
    l2Entry.cfg.srio.pri       = (uint8_t) srioInfo->pri;
    
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO)   {
  
    if((srioInfo->msgType != pa_SRIO_TYPE_9) && (srioInfo->msgType != pa_SRIO_TYPE_11))
      return(pa_ERR_CONFIG);
      
    l2Entry.cfg.srio.msgType  = (uint8_t) srioInfo->msgType; 
      
    if (srioInfo->msgType == pa_SRIO_TYPE_9) {
    
        if(srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID)
          l2Entry.cfg.srio.typeParam1 = srioInfo->typeInfo.type9.streamId;
          
        if(srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_COS)
          l2Entry.cfg.srio.typeParam2 = (uint8_t)srioInfo->typeInfo.type9.cos;
          
    }  
    
    if (srioInfo->msgType == pa_SRIO_TYPE_11) {
    
    
        if(srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX)
          l2Entry.cfg.srio.typeParam1 = srioInfo->typeInfo.type11.mbox;
    
        if(srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_LETTER)
          l2Entry.cfg.srio.typeParam2 = (uint8_t)srioInfo->typeInfo.type11.letter;
          
    }  
  }
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  l2Table	= (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);	

  /* Signal the application that a table modification will be done */
  Pa_osalBeginMemAccess ((void*) l2Table, 
	      			     paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  /* Look for an identical entry in the table. If one is found, return it */
  /* perform entry check only if the LUT1 index is not specified by user */
  if (lut1Index == PAFRM_LUT1_INDEX_LAST_FREE) {
    for (i = 0; i < paInst->l2n;  i++)  {

      if ( ((l2Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (l2Table[i].hdr.status == PA_TBL_STAT_ACTIVE)             ) &&
           (l2Table[i].hdr.subType == PA_TABLE_ENTRY_SUBYTPE_SRIO )) {

        if (!memcmp (&(l2Table[i].cfg), &(l2Entry.cfg), sizeof(paL2InCfg_t)))  {

          /* Identical entry identified */
          if (l2Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) {
            *handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l2Table[i]);
            ret = pa_INVALID_DUP_ENTRY;
            goto Pa_addSrio_end;                 
          }                     
          else {
              /*  
               * Identical entry is identified, reuse the same table entry
               * Keep the entry status since it simply replaces the original entry
               * and the user may decide not to forward the packet
               */ 
            lut1Index = l2Table[i].hdr.lutIdx;
            ret = pa_DUP_ENTRY;   
            goto Pa_addSrio_send_cmd;
          }                  
        }
      }
    }


    /* Check to see if this entry is more general then one already in the table.
     * If so then this entry would 'steal' packets from that entry.
     */

    for (i = 0; i < paInst->l2n; i++)  {

      if ( (l2Table[i].hdr.status != PA_TBL_STAT_PENDING_SUBS_REPLY)  &&
           (l2Table[i].hdr.status != PA_TBL_STAT_ACTIVE)              )
        continue;
        
      if ( l2Table[i].hdr.subType != PA_TABLE_ENTRY_SUBYTPE_SRIO )  
        continue;
        
      if (l2Entry.cfg.srio.validBitMap & pa_SRIO_INFO_VALID_ID) {
         if(!(l2Table[i].cfg.srio.validBitMap & pa_SRIO_INFO_VALID_ID))
           continue;
         else if (l2Table[i].cfg.srio.tt != l2Entry.cfg.srio.tt)
           continue;  
      }
        
      if (l2Entry.cfg.srio.validBitMap & pa_SRIO_INFO_VALID_SRC_ID) {
         if(!(l2Table[i].cfg.srio.validBitMap & pa_SRIO_INFO_VALID_SRC_ID))
           continue;
         else if (l2Table[i].cfg.srio.srcId != l2Entry.cfg.srio.srcId)
           continue;  
      }
      
      if (l2Entry.cfg.srio.validBitMap & pa_SRIO_INFO_VALID_DEST_ID) {
         if(!(l2Table[i].cfg.srio.validBitMap & pa_SRIO_INFO_VALID_DEST_ID))
           continue;
         else if (l2Table[i].cfg.srio.destId != l2Entry.cfg.srio.destId)
           continue;  
      }
      
      if (l2Entry.cfg.srio.validBitMap & pa_SRIO_INFO_VALID_CC) {
         if(!(l2Table[i].cfg.srio.validBitMap & pa_SRIO_INFO_VALID_CC))
           continue;
         else if (l2Table[i].cfg.srio.cc != l2Entry.cfg.srio.cc)
           continue;  
      }
      
      if (l2Entry.cfg.srio.validBitMap & pa_SRIO_INFO_VALID_PRI) {
         if(!(l2Table[i].cfg.srio.validBitMap & pa_SRIO_INFO_VALID_PRI))
           continue;
         else if (l2Table[i].cfg.srio.pri != l2Entry.cfg.srio.pri)
           continue;  
      }
      
      if (l2Entry.cfg.srio.validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO) {
         if(!(l2Table[i].cfg.srio.validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO))
           continue;
         else if (l2Table[i].cfg.srio.msgType != l2Entry.cfg.srio.msgType)
           continue;  
      }
      
      
      /* Note: It is the same aspa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX */ 
      if (l2Entry.cfg.srio.validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID) {
         if(!(l2Table[i].cfg.srio.validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID))
           continue;
         else if (l2Table[i].cfg.srio.typeParam1 != l2Entry.cfg.srio.typeParam1)
           continue;  
      }
      
      /* Note: It is the same aspa_SRIO_INFO_VALID_TYPE_INFO_LETTER */ 
      if (l2Entry.cfg.srio.validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_COS) {
         if(!(l2Table[i].cfg.srio.validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_COS))
           continue;
         else if (l2Table[i].cfg.srio.typeParam2 != l2Entry.cfg.srio.typeParam2)
           continue;  
      }
      
        
      /* This is a more specific entry then one already in the table */
      ret = pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT;                     
      goto Pa_addSrio_end;                 
    }
  }

  /* Find a free entry in the table */
  for (i = 0; i < paInst->l2n; i++)  {

    if (l2Table[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;

  }

  if (i == paInst->l2n)  {
    ret = pa_HANDLE_TABLE_FULL;
    goto Pa_addSrio_end;                 
  }
  
Pa_addSrio_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l2Table[i]);

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_L2  | i, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  al1->index         = SWIZ(lut1Index);
  al1->type          = PAFRM_COM_ADD_LUT1_SRIO;

  /* Form the matchflags and the key */
  al1->u.ethIp.matchFlags = PAFRM_LUT1_CUSTOM_MATCH_KEY;
  al1->u.srio.key         = PAFRM_LUT1_SRIO_KEY_SRIO;
                           
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_SRC_ID) {
    al1->u.srio.srcId        = SWIZ(l2Entry.cfg.srio.srcId);
    al1->u.srio.matchFlags  |= PAFRM_LUT1_SRIO_MATCH_SRCID; 
  }
  
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_DEST_ID) {
    al1->u.srio.destId       = SWIZ(l2Entry.cfg.srio.destId);
    al1->u.srio.matchFlags  |= PAFRM_LUT1_SRIO_MATCH_DESTID; 
  }
  
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_PRI) {
    al1->u.srio.pri          = SWIZ(l2Entry.cfg.srio.pri);
    al1->u.srio.matchFlags  |= PAFRM_LUT1_SRIO_MATCH_PRI; 
  }
  
  /* pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX */
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID) {
    al1->u.srio.typeParam1   = SWIZ(l2Entry.cfg.srio.typeParam1);
    al1->u.srio.matchFlags  |= PAFRM_LUT1_SRIO_MATCH_TYPEPARAM1; 
  }
  
  /* pa_SRIO_INFO_VALID_TYPE_INFO_LETTER */
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_COS) {
    al1->u.srio.typeParam2   = SWIZ(l2Entry.cfg.srio.typeParam2);
    al1->u.srio.matchFlags  |= PAFRM_LUT1_SRIO_MATCH_TYPEPARAM2; 
  }
  
  if(srioInfo->tt ==  pa_SRIO_TRANSPORT_TYPE_0)
    al1->u.srio.key |= PAFRM_LUT1_SRIO_KEY_TRANSPORT_8;
  else
    al1->u.srio.key |= PAFRM_LUT1_SRIO_KEY_TRANSPORT_16;
    
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO)
  {  
    if(srioInfo->msgType == pa_SRIO_TYPE_9)
        al1->u.srio.msgType = PAFRM_LUT1_SRIO_MSG_TYPE_9;
    else  
        al1->u.srio.msgType = PAFRM_LUT1_SRIO_MSG_TYPE_11;
        
    al1->u.srio.msgType     = SWIZ(al1->u.srio.msgType);    
    al1->u.srio.matchFlags  |= PAFRM_LUT1_SRIO_MATCH_MSGTYPE;        
  }  
  
  al1->u.srio.nextHdr       = SWIZ(pa_next_hdr_tbl[nextHdr]);
  al1->u.srio.nextHdrOffset = SWIZ(nextHdrOffset);
  al1->u.srio.matchFlags    = SWIZ(al1->u.srio.matchFlags);
  al1->u.srio.key           = SWIZ(al1->u.srio.key);

  /* Forwarding information */
  ret1 = pa_conv_routing_info(paInst, &al1->match, routeInfo, pa_CMD_TX_DEST_0, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1; 
    goto Pa_addSrio_end;                 
  }

  /* Next fail information */
  ret1 = pa_conv_routing_info(paInst, &al1->nextFail, nextRtFail, pa_CMD_TX_DEST_0, TRUE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret != pa_OK) {
    ret = ret1;
    goto Pa_addSrio_end;                 
     
  }

  if (ret != pa_DUP_ENTRY) {
     /* Initialze, the table entry, add the status and pdsp NUM */
     l2Entry.hdr.type     = PA_TABLE_ENTRY_TYPE_L2;
     l2Entry.hdr.subType  = PA_TABLE_ENTRY_SUBYTPE_SRIO;
     l2Entry.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
     l2Entry.hdr.pdspNum  = PASS_PDSP0;
     l2Entry.hdr.lutIdx   = -1;
     l2Entry.hdr.tableIdx = i;

     memcpy (&l2Table[i], &l2Entry, sizeof(paL2Entry_t));
  }

  /* The destination for this command must be PDSP 0 */
  *cmdDest = pa_CMD_TX_DEST_0;
  
Pa_addSrio_end:  
  Pa_osalEndMemAccess ((void *) l2Table,
					 paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  
  
  return (ret);

} /* Pa_addSrio */

/*******************************************************************************************
 * FUNCTION PURPOSE: Add an ACL entry to the ACL table
 *******************************************************************************************
 * DESCRIPTION: The ACL entry is verified and added to the ACL table. 
 *              The following LUT1 are used to store ACL related entries at NetCP 1.5:
 *                  - Ingess 0, PDSP1, LUT1_1: Outer IP
 *                  - Ingres 3, PDSP0, LUT1_5: Inner IP
 *                
 ********************************************************************************************/
paReturn_t  Pa_addAcl  (Pa_Handle          iHandle,
                        int                aclInst,
                        int                aclAction,
                        paAclInfo_t       *aclInfo,
                        paHandleL2L3_t     prevLink,
                        paHandleAcl_t      nextEntry,
                        paHandleAcl_t     *retHandle,
                        paCmd_t            cmd,
                        uint16_t          *cmdSize,
                        paCmdReply_t      *reply,
                        int               *cmdDest,
                        uint32_t          *timeToNextCall)
{
  return (pa_API_UNSUPPORTED);
} /* Pa_addAcl */

/*******************************************************************************************
 * FUNCTION PURPOSE: Add a Flow Cache entry to the FC table
 *******************************************************************************************
 * DESCRIPTION: The Flow Cache entry is verified and added to the FC table. 
 *              The following LUT1 are used to store Flwo Cache entries at NetCP 1.5:
 *                  - Egress 0, PDSP0, LUT1_INST_5_0
 *                
 ********************************************************************************************/
paReturn_t  Pa_addFc   (Pa_Handle          iHandle,
                        int                index,
                        paEfOpInfo_t      *efOpInfo,
                        paFcInfo_t        *fcInfo,
                        paHandleFc_t      *retHandle,
                        paCmd_t            cmd,
                        uint16_t          *cmdSize,
                        paCmdReply_t      *reply,
                        int               *cmdDest )
{
  return (pa_API_UNSUPPORTED);
} /* Pa_addFc */

/***************************************************************************************
 * FUNCTION PURPOSE: Delete an L2/L3 handle
 ***************************************************************************************
 * DESCRIPTION: The handle is deleted. Dependent handles are left intact
 ***************************************************************************************/
paReturn_t Pa_delHandle (Pa_Handle       iHandle,
                         paHandleL2L3_t  *handle, 
                         paCmd_t         cmd,
                         uint16_t        *cmdSize,
                         paCmdReply_t    *reply,
                         int             *cmdDest )

{
  paInst_t              *paInst  = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL3Entry_t           *l3Table;
  paL2L3Header_t        *hdr     = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,*handle);
  paL3Entry_t           *l3e     = NULL;
  pafrmCommand_t        *fcmd;
  pafrmCommandDelLut1_t *del;
  uint16_t              csize, comId;
  paReturn_t            ret = pa_OK;
  uint32_t              mtCsKey;  
  paL2Entry_t 		   *l2Table;
  paVirtualLnk_t       *vlnkTable;
  vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);  
  
  if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+sizeof(pafrmCommandDelLut1_t)-sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if ((reply->dest != pa_DEST_HOST) && (reply->dest != pa_DEST_DISCARD))
    return (pa_INVALID_CMD_REPLY_DEST);
    
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  l2Table   = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);;

  /* Inform the host a table entry will be changed. Need to do both
   * since the table must be read before determining what type of
   * handle it is */
  Pa_osalBeginMemAccess ((void*)l2Table,
					   paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalBeginMemAccess ((void*)l3Table,
					   paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)						
    Pa_osalBeginMemAccess ((void *) vlnkTable,
						 paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);

  /* Basic sanity check. The base address of the table is not verified, however */
  if ((hdr == NULL) || ((hdr->type != PA_TABLE_ENTRY_TYPE_L2) && (hdr->type != PA_TABLE_ENTRY_TYPE_L3))) {
    ret = pa_INVALID_INPUT_HANDLE;
    goto Pa_delHandle_end;
  }

  if (hdr->status == PA_TBL_STAT_INACTIVE)  {
    ret = pa_HANDLE_INACTIVE;
    goto Pa_delHandle_end;
  }

  if (hdr->type == PA_TABLE_ENTRY_TYPE_L2)
    comId = PA_COMID_L2 | hdr->tableIdx;
  else {
    comId = PA_COMID_L3 | hdr->tableIdx;
    l3e = &l3Table[hdr->tableIdx];
    
    if (l3e->pHandle) {
      /* note: The L2L3 header and the virtual link header are compatible */
      paL2L3Header_t *hdr1 = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3e->pHandle);
      if (hdr1->lnkCnt > 0) 
      {
        hdr1->lnkCnt--;
      }
      else
      {
        ret = pa_WARN_LNK_CNT_UNSYNC;
      }
    }
    
  }
  
  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_DEL_LUT1, comId, csize);
  
  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  del = (pafrmCommandDelLut1_t *)&(fcmd->cmd);

  del->index = hdr->lutIdx;
  del->index = SWIZ(del->index);

  /* The command must be sent to the PDSP that owns the lut */
  *cmdDest = hdr->pdspNum + pa_CMD_TX_DEST_0;

  /* Mark the entry as disabled */
  hdr->status = PA_TBL_STAT_INACTIVE;
  
Pa_delHandle_end:
  
  /* Release the table */
  Pa_osalEndMemAccess ((void*)l2Table,
					 paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalEndMemAccess ((void*)l3Table,
					 paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)						
    Pa_osalEndMemAccess ((void *)vlnkTable,
					  paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  /* clear the handle */
  *handle = NULL;

  return (ret);

} /* Pa_delHandle */

/*******************************************************************************************
 * FUNCTION PURPOSE: Return LUT1 information from the L2L3Handle
 *******************************************************************************************
 * DESCRIPTION: The function returns the PDSPID and the lut1 index from the LUT1 handle. 
 *
 ********************************************************************************************/
paReturn_t Pa_getLUT1Info ( Pa_Handle       iHandle,
                            paHandleL2L3_t  l2l3handle,
                            paLUT1Info_t    *lut1Info)
{
  paL2L3Header_t  *hdr = (paL2L3Header_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l2l3handle);
  paReturn_t      ret = pa_OK;

  /* Basic sanity check. The base address of the table is not verified, however */
  if ((hdr == NULL) || ((hdr->type != PA_TABLE_ENTRY_TYPE_L2) && (hdr->type != PA_TABLE_ENTRY_TYPE_L3))) {
    ret = pa_INVALID_INPUT_HANDLE;
  }
  else {
    lut1Info->lut1Inst  = hdr->pdspNum;
    lut1Info->lut1Index = hdr->lutIdx;
  }

  return (ret);
}

/***************************************************************************************
 * FUNCTION PURPOSE: Delete an ACL handle
 ***************************************************************************************
 * DESCRIPTION: The handle is deleted. Dependent handles are left intact
 ***************************************************************************************/
paReturn_t Pa_delAclHandle (Pa_Handle       iHandle,
                            paHandleAcl_t   *handle, 
                            paCmd_t         cmd,
                            uint16_t        *cmdSize,
                            paCmdReply_t    *reply,
                            int             *cmdDest )
{
  return (pa_API_UNSUPPORTED);
} /* Pa_delAclHandle */

/***************************************************************************************
 * FUNCTION PURPOSE: Delete a FC handle
 ***************************************************************************************
 * DESCRIPTION: The handle is deleted. Dependent handles are left intact
 ***************************************************************************************/
paReturn_t Pa_delFcHandle (Pa_Handle       iHandle,
                           paHandleFc_t    *handle, 
                           paCmd_t         cmd,
                           uint16_t        *cmdSize,
                           paCmdReply_t    *reply,
                           int             *cmdDest )
{
  return (pa_API_UNSUPPORTED);
} /* Pa_delFcHandle */

/*******************************************************************************************
 * FUNCTION PURPOSE: Return number of linking channels
 *******************************************************************************************
 * DESCRIPTION: The function returns the number of channels linked to the LUT1 handle. 
 *
 ********************************************************************************************/
paReturn_t Pa_getHandleRefCount ( Pa_Handle       iHandle,
                                  paHandleL2L3_t  l2l3handle,
                                  uint16_t        *refCount )
{
  paInst_t			    *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL2L3Header_t        *hdr     = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l2l3handle);
  paReturn_t            ret = pa_OK;
  uint32_t              mtCsKey;  
  paL2Entry_t 		   *l2Table   = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);
  paL3Entry_t          *l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  paVirtualLnk_t       *vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);

  if(refCount == NULL)
    return(pa_INVALID_INPUT_POINTER);

  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  /* Inform the host a table entry will be changed. Need to do both
   * since the table must be read before determining what type of
   * handle it is */
  
  Pa_osalBeginMemAccess ((void *) l2Table,
                         paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalBeginMemAccess ((void*) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)                       
    Pa_osalBeginMemAccess ((void *) vlnkTable,
                            paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);

  if ((hdr == NULL) || ((hdr->type != PA_TABLE_ENTRY_TYPE_L2) && (hdr->type != PA_TABLE_ENTRY_TYPE_L3) && (hdr->type != PA_TABLE_ENTRY_TYPE_VL) )) {
    *refCount = 0;                     
    ret = pa_INVALID_INPUT_HANDLE;
  }
  else
  {
    *refCount = hdr->lnkCnt;
  }
  /* Release the table */
  Pa_osalEndMemAccess ((void*)l2Table,
                       paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalEndMemAccess ((void*)l3Table,
                       paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)                       
    Pa_osalEndMemAccess ((void*)vlnkTable,
                        paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);

} /* Pa_getHandleRefCount */

/*******************************************************************************************
 * FUNCTION PURPOSE: Allocate a VirtualLink in the virtual link table.
 *******************************************************************************************
 * DESCRIPTION: Allocate a VirtualLink in the virtual link table.
 ********************************************************************************************/
paReturn_t Pa_addVirtualLink(Pa_Handle       iHandle,
                             paLnkHandle_t   *vlinkHdl,
                                int8_t lnkType)
{
  paInst_t			  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  int               i;
  paVirtualLnk_t    *vlnkTable;
  paReturn_t        ret = pa_OK;
  
  if(vlinkHdl == NULL)
    return(pa_INVALID_INPUT_POINTER);

  if(lnkType > pa_VIRTUAL_LNK_TYPE_INNER_IP)
    return (pa_ERR_CONFIG);

  vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);
  Pa_osalBeginMemAccess ((void *) vlnkTable,
					   paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  // Look for free entries in the virtual link table
  for (i = 0; i < paInst->nMaxVlnk; i++)
  {
    if (vlnkTable[i].status == PA_TBL_STAT_INACTIVE)
    {
      vlnkTable[i].status   = PA_TBL_STAT_ACTIVE;
      vlnkTable[i].subType  = PA_TABLE_ENTRY_SUBTYPE_VLINK_BASE + lnkType;
	  /* Update the return value to be offset instead of absolute address to support multi process */
      *vlinkHdl          = (paLnkHandle_t)	  pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&vlnkTable[i]);
      ret = pa_OK;
  	  Pa_osalEndMemAccess ((void *) vlnkTable,
							 paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
      return ret;
    }
  }
  ret = pa_VIRTUAL_LINK_TABLE_FULL;
  Pa_osalEndMemAccess ((void *) vlnkTable,
					   paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  return ret;

}

/*******************************************************************************************
 * FUNCTION PURPOSE: Delete a VirtualLink in the virtual link table.
 *******************************************************************************************
 * DESCRIPTION: Delete a VirtualLink in the virtual link table.
 ********************************************************************************************/
paReturn_t Pa_delVirtualLink(Pa_Handle       iHandle,
                             paLnkHandle_t   *vlinkHdl
                            )
{
  paInst_t		    *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paVirtualLnk_t    *vlnk;
  paReturn_t        ret = pa_INVALID_INPUT_HANDLE;
  paVirtualLnk_t    *vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);

  if(vlinkHdl == NULL)
    return(pa_INVALID_INPUT_POINTER);
    
  vlnk = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, *vlinkHdl);  

  Pa_osalBeginMemAccess ((void *) vlnkTable,
						 paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  // Look for free entries in the virtual link table
  if (vlnk->status == PA_TBL_STAT_ACTIVE)
  {
      vlnk->status = PA_TBL_STAT_INACTIVE;
      vlnk->lnkCnt = 0;
      ret =  pa_OK;
      *vlinkHdl = NULL;  /* Clear the handle */
  }
	Pa_osalEndMemAccess ((void *) vlnkTable,
						 paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  return ret;

}

/*******************************************************************************************
 * FUNCTION PURPOSE: Allocates a set of user-defined statistics
 *******************************************************************************************
 * DESCRIPTION: This function is called to request a number of 32-bit or 64-bit user-defined 
 *              statistics. If there are not enough user-defined statistics available, It
 *              shall return pa_RESOURCE_USE_DENIED
 ********************************************************************************************/
paReturn_t Pa_allocUsrStats(Pa_Handle           iHandle,
                            int                 *pNumCnt,
                            paUsrStatsAlloc_t   *cntList
                            )
{
    paInst_t          *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
    uint16_t          numCounters, num64bCounters; 
    int               startIdx, endIdx, i, rmInst;
    int               numCnt;
    int32_t           cntIndex, rmCntIndex;
    paReturn_t        ret = pa_OK;
    uint32_t          mtCsKey; 
    
    if(pNumCnt == NULL)
        return(pa_INVALID_INPUT_POINTER);
        
    numCnt = *pNumCnt;    

    /* Refresh PA Instance  */
    Pa_osalMtCsEnter(&mtCsKey);
    Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

    num64bCounters = paInst->cfg.usrStatsConfig.num64bCounters;
    numCounters    = paInst->cfg.usrStatsConfig.numCounters;

    if ((cntList == NULL) || (numCnt > numCounters))
    {
        *pNumCnt = 0;
        ret = pa_ERR_CONFIG;
    } 
    else
    {
        for (i = 0; i < numCnt; i++)
        {
            paUsrStatsAlloc_t *pStatsAlloc = &cntList[i];

            if (pStatsAlloc->ctrlBitfield & pa_USR_STATS_ALLOC_64B_CNT)
            {
                /* 64-bit counter */
                startIdx = 0; 
                endIdx = num64bCounters - 1;
                rmInst = 1;
            }
            else
            {
                /* 32-bit counter */
                startIdx = num64bCounters;
                endIdx = numCounters - 1;
                rmInst = 0;
            }

            if (pStatsAlloc->ctrlBitfield & pa_USR_STATS_ALLOC_CNT_PRESENT)
            {

                cntIndex = pStatsAlloc->cntIndex;

                if ((cntIndex < startIdx) || (cntIndex > endIdx))
                {
                    /* Index is invalid */
                    ret = pa_RESOURCE_USE_DENIED;
                    break;
                }

                /* Counter verification */
                if (paLObj.cfg.rmServiceHandle)
                {
                    rmCntIndex = cntIndex - startIdx;
                    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmUsrStats[rmInst], &rmCntIndex, NULL))
                    {
                        ret = pa_RESOURCE_USE_DENIED;
                        break;
                    }
                    else
                    {
                        /* The counter is valid, release the resource to simulate permission check */
                        pa_rmService (Rm_service_RESOURCE_FREE, rmUsrStats[rmInst], &rmCntIndex, NULL);
                    }
                }

                /* Index is valid, mark the allocation bitMap */
                pa_bitmap_update(paInst->usrStatsAllocBitmap, PA_USR_STATS_BITMAP_SIZE, cntIndex, TRUE);

            }
            else
            {
                /* Counter allocation */
                if (paLObj.cfg.rmServiceHandle)
                {
                    rmCntIndex = RM_RESOURCE_BASE_UNSPECIFIED;
                    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmUsrStats[rmInst], &rmCntIndex, NULL))
                    {
                        ret = pa_RESOURCE_USE_DENIED;
                        break;
                    }

                    cntIndex = rmCntIndex + startIdx;

                    if (cntIndex > endIdx)
                    {
                        /*
                         * The allocated counter is within the resoure range, but it exceeds the local
                         * limit. Free the resource and return error
                         */
                        pa_rmService (Rm_service_RESOURCE_FREE, rmUsrStats[rmInst], &rmCntIndex, NULL);
                        ret = pa_RESOURCE_USE_DENIED;
                        break;

                    }

                    /* Counter is available and so mark the allocation bitMap */
                    pa_bitmap_update(paInst->usrStatsAllocBitmap, PA_USR_STATS_BITMAP_SIZE, cntIndex, TRUE);
                }
                else
                {
                    cntIndex = pa_bitmap_alloc(paInst->usrStatsAllocBitmap, startIdx, endIdx);
                    if (cntIndex == PA_INDEX_ALLOCATION_ERROR)
                    {
                        /* There are no more available counters */
                        ret = pa_RESOURCE_USE_DENIED;
                        break;
                    }
                }

                /* Record the allocated counter */
                pStatsAlloc->cntIndex = (uint16_t)cntIndex;
            }
        }

        *pNumCnt = i;
    }

    Pa_osalEndMemAccess (paInst, sizeof(paInst_t));

    Pa_osalMtCsExit(mtCsKey);  

    return (ret);
} /* Pa_allocUsrStats */

/*******************************************************************************************
 * FUNCTION PURPOSE: Free a set of user-defined statistics
 *******************************************************************************************
 * DESCRIPTION: This function is called to release a set of user-defined statistics. 
 ********************************************************************************************/
paReturn_t Pa_freeUsrStats(Pa_Handle       iHandle,
                           int             numCnt,
                           uint16_t*       cntList
                          )
{
    paInst_t   *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
    uint16_t    numCounters, num64bCounters; 
    int         i, rmInst;
    int32_t     resCnt, rmCntIndex;
    uint32_t    mtCsKey; 
    paReturn_t  ret = pa_OK;

    /* Refresh PA Instance */
    Pa_osalMtCsEnter(&mtCsKey);
    Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

    num64bCounters = paInst->cfg.usrStatsConfig.num64bCounters;
    numCounters    = paInst->cfg.usrStatsConfig.numCounters;

    if ((numCnt > numCounters) || (cntList == NULL))
    {
        ret = pa_ERR_CONFIG;
    }
    else
    {
        for ( i = 0; i < numCnt; i++)
        {
            int32_t cntIndex = cntList[i];

            if (cntIndex < num64bCounters)
            {
                rmInst = 1;
                rmCntIndex = cntIndex;
            }
            else
            {
                rmInst = 0;
                rmCntIndex = cntIndex - num64bCounters;
            }

            if (cntIndex >= numCounters)
            {
                /* invalid  counter */
                ret = pa_ERR_CONFIG;
                break;
            }
            else if (paLObj.cfg.rmServiceHandle)
            {
                if (!pa_rmService (Rm_service_RESOURCE_FREE, rmUsrStats[rmInst], &rmCntIndex, &resCnt))
                {
                    ret = pa_RESOURCE_USE_DENIED;
                    break;
                }

                /* clear the bit the allocation bitMap only if the resource counter reach 0 */
                if(resCnt == 0)
                    pa_bitmap_update(paInst->usrStatsAllocBitmap, PA_USR_STATS_BITMAP_SIZE, cntIndex, FALSE);
            }
            else
            {
                /* Clear the bit in the allocation bitMap */
                pa_bitmap_update(paInst->usrStatsAllocBitmap, PA_USR_STATS_BITMAP_SIZE, cntIndex, FALSE);
            }
        }
    }

    Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
    Pa_osalMtCsExit(mtCsKey);

    return (ret);
} /* Pa_freeUsrStats */

/*******************************************************************************************
 * FUNCTION PURPOSE: Add an IP address to the lookup table
 *******************************************************************************************
 * DESCRIPTION: The IP address is verified and added to the table. 
 *
 *              If the IP address is linked to a MAC address then the entry is sent to PDSP1
 *              If the entry is not linked to a MAC address then the entry is sent to PDSP1
 *
 *              If the IP address is linked to another IP address the entry is sent to PDSP2
 *
 ********************************************************************************************/
paReturn_t  Pa_addIp  ( Pa_Handle       iHandle,
                        int             lutInst,
                        int             index, 
                        paIpInfo_t     *ipInfo,
                        paHandleL2L3_t  prevLink,
                        paRouteInfo_t  *routeInfo,
                        paRouteInfo_t  *nextRtFail,
                        paHandleL2L3_t *retHandle,
                        paCmd_t         cmd,
                        uint16_t       *cmdSize,
                        paCmdReply_t   *reply,
                        int            *cmdDest )
{

  paParamDesc params;
  paIpInfo2_t ipInfo2;
  paRouteInfo2_t matchRoute2;
  paRouteInfo2_t nfailRoute2;
  paReturn_t  ret;
  
  if((ipInfo == NULL) || (routeInfo == NULL) || (nextRtFail == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  pa_convert_ipInfo_to_ipInfo2(ipInfo, &ipInfo2);
  pa_convert_routeInfo_to_routeInfo2(routeInfo,  &matchRoute2);
  pa_convert_routeInfo_to_routeInfo2(nextRtFail, &nfailRoute2);
  pa_format_paramDesc(&params, lutInst, index, prevLink, NULL, &matchRoute2, &nfailRoute2);
  
  ret = Pa_addIp2 (iHandle,
                   &ipInfo2,
                   &params,
                   retHandle,
                   cmd,
                   cmdSize,
                   reply,
                   cmdDest);

  return (ret);
  
} /* Pa_addIp */

/*******************************************************************************************
 * FUNCTION PURPOSE: Add an IP address to the lookup table
 *******************************************************************************************
 * DESCRIPTION: The IP address is verified and added to the table. 
 *
 *              If the IP address is linked to a MAC address then the entry is sent to PDSP1
 *              If the entry is not linked to a MAC address then the entry is sent to PDSP1
 *
 *              If the IP address is linked to another IP address the entry is sent to PDSP2
 *
 ********************************************************************************************/
paReturn_t  Pa_addIp2  (Pa_Handle       iHandle,
                        paIpInfo2_t    *ipInfo,
                        paParamDesc    *params,
                        paLnkHandle_t  *retHandle,
                        paCmd_t         cmd,
                        uint16_t       *cmdSize,
                        paCmdReply_t   *reply,
                        int            *cmdDest )
{

  /* The entry is created in the stack and then copied to the table. This allows
   * for a comparison with other table entries to verify there is not an identical
   * entry or an entry that would supercede this one */
  paInst_t				*paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL3Entry_t            *l3Table;
  
  paL3Entry_t             l3Entry;
  paL2L3Header_t         *hdr = NULL;
  paVirtualLnk_t         *vhdr = NULL;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  int                     i;
  uint16_t                csize;
  uint16_t                l2Release;
  int                     fNonIpSpi;  

  paL2Entry_t 		 *l2Table	= (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);
  paVirtualLnk_t     *vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);
  /* Param parsing */ 
  int             lutInst;
  paLnkHandle_t   prevLink;
  paLnkHandle_t   nextLink;
  uint8_t         lut1Index;
  paRouteInfo2_t  *routeInfo;
  paRouteInfo2_t  *nextRtFail;
  
  paReturn_t      ret = pa_OK;
  paReturn_t      ret1;
  uint8_t         numNextLayers = 0;   /* Specify the number of next matching layers */
  uint32_t        mtCsKey;  
  uint8_t         virtualLinkFlag;  

  if((ipInfo == NULL) || (params == NULL) || (params->routeInfo == NULL) || (params->nextRtFail == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  lutInst = (params->validBitMap & pa_PARAM_VALID_LUTINST)?params->lutInst:pa_LUT_INST_NOT_SPECIFIED;
  hdr = prevLink = (params->validBitMap & pa_PARAM_VALID_PREVLINK)?((paLnkHandle_t)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,params->prevLink)):NULL; 
  vhdr = nextLink = (params->validBitMap & pa_PARAM_VALID_NEXTLINK)?((paLnkHandle_t)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,params->nextLink)):NULL;
  lut1Index = (params->validBitMap & pa_PARAM_VALID_INDEX)?(uint16_t)params->index:PAFRM_LUT1_INDEX_LAST_FREE;
  routeInfo = params->routeInfo;
  nextRtFail = params->nextRtFail;
  
  /* Sanity check the LUT1 index */
  if (lut1Index > PAFRM_HW_LUT1_ENTRIES) {
    return(pa_INVALID_LUT1_INDEX);
  }
  
  /* Sanity check the LUT1 instance */
  if ((lutInst != pa_LUT_INST_NOT_SPECIFIED) && (lutInst > pa_LUT1_INST_MAX)) {
    return(pa_INVALID_LUT_INST);
  }
  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+sizeof(pafrmCommandAddLut1_t)-sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  /* Return the actual size of the buffer used */
  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);

  /* It is not valid to have a GRE and SPI and SCTP in the search */
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_SPI) numNextLayers++;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_GREPROTO) numNextLayers++;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_SCTPPORT) numNextLayers++;
  if (numNextLayers > 1)
    return (pa_ERR_CONFIG);

  /* Create the entry on the stack, and zero the entire entry. This will
   * allow simple compare of entries using memcmp even though some fields
   * will be unused, or there will be gaps between fields */
  memset (&l3Entry, 0, sizeof(paL3Entry_t));

  /* For IPv4, copy the 4 address bytes to the last 4 bytes in an IPv6 sized byte array */
  if (ipInfo->validBitMap & (pa_IP_INFO_VALID_SRC | pa_IP_INFO_VALID_DST))
  { 
    if (ipInfo->ipType == pa_IPV4)  {
        
        if (ipInfo->validBitMap & pa_IP_INFO_VALID_SRC)
            memcpy (&(l3Entry.u.ipInfo.src.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                    ipInfo->src.ipv4, sizeof(paIpv4Addr_t));

        if (ipInfo->validBitMap & pa_IP_INFO_VALID_DST)
            memcpy (&(l3Entry.u.ipInfo.dst.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                    ipInfo->dst.ipv4, sizeof(paIpv4Addr_t));

    }  else if (ipInfo->ipType == pa_IPV6)  {
        if (ipInfo->validBitMap & pa_IP_INFO_VALID_SRC)
            memcpy (&l3Entry.u.ipInfo.src.ipv6, ipInfo->src.ipv6, sizeof(paIpv6Addr_t));
            
        if (ipInfo->validBitMap & pa_IP_INFO_VALID_DST)
            memcpy (&l3Entry.u.ipInfo.dst.ipv6, ipInfo->dst.ipv6, sizeof(paIpv6Addr_t));

    }  else
        return (pa_ERR_CONFIG);
        
    l3Entry.u.ipInfo.ipType   =  ipInfo->ipType;
        
  }  
  l3Entry.u.ipInfo.validBitMap  =  ipInfo->validBitMap;
  
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_SPI)l3Entry.u.ipInfo.spi = ipInfo->spi;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_FLOW)l3Entry.u.ipInfo.flow = ipInfo->flow;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_GREPROTO)l3Entry.u.ipInfo.greProto = ipInfo->greProto;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_PROTO)l3Entry.u.ipInfo.proto = ipInfo->proto;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_TOS)l3Entry.u.ipInfo.tos = ipInfo->tos;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_SCTPPORT)l3Entry.u.ipInfo.sctpPort = ipInfo->sctpPort;
  
  /* The flow label is restricted to 20 bits */
  if (l3Entry.u.ipInfo.flow & ~PA_IP_FLOW_MASK)  {
    return( pa_INVALID_IP_FLOW);
  }
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  /* Signal the application that a table modification will be done. */
  l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  Pa_osalBeginMemAccess ((void*)l3Table,
					   paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
                         
  l2Release = FALSE;
  virtualLinkFlag = FALSE;


  /* If there is a link to the l2 table then that table must be
   * protected as well. Of course the previous link could be an l3
   * link as well as an l2 link. If so then the l2 protection is released */
  if (prevLink != NULL)  {
    /* If the prevLink is a virtual link, then we need to protect the
     * virtual link table, then the l2 protection is released
     * */
    if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)                       
        Pa_osalBeginMemAccess ((void *) vlnkTable,
                               paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
     
    vhdr = (paVirtualLnk_t *)prevLink;
    if (vhdr->type == PA_TABLE_ENTRY_TYPE_VL)
    {
      virtualLinkFlag = TRUE;
      if(vhdr->status != PA_TBL_STAT_ACTIVE){
        ret = pa_INVALID_INPUT_HANDLE;
        goto Pa_addIp2_end;
      }
    }
    else{
      if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)
        Pa_osalEndMemAccess ((void*) vlnkTable,
                            paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
      l2Release = TRUE;
      Pa_osalBeginMemAccess ((void *) l2Table,
                             paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);

      hdr = (paL2L3Header_t *)prevLink;
      /* Must be linked to a complete entry */
      if (hdr->status != PA_TBL_STAT_ACTIVE)  {
        ret = pa_INVALID_INPUT_HANDLE;
        goto Pa_addIp2_end;
      }

      if (hdr->type == PA_TABLE_ENTRY_TYPE_L3)  {
        Pa_osalEndMemAccess ((void *) l2Table,
                             paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
        l2Release = FALSE;
      }  else if (hdr->type != PA_TABLE_ENTRY_TYPE_L2)  {
        ret = pa_INVALID_INPUT_HANDLE;
        goto Pa_addIp2_end;
      }
    }

  }

  /* Replace the index might have been requested 
   * Check if that is the case
   */
  if (params->validBitMap & pa_PARAM_VALID_CTRLBITMAP) {
    /* Check if replace command is set */
    if (params->ctrlBitMap & pa_PARAM_CTRL_REPLACE) {        
      paL3Entry_t *l3EntryRep = (paL3Entry_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, *retHandle);
      /* If the replace index entry provided is not active,
       * flag the error to application 
       * Also, The replace index requested can not be 
       * different from original lut index 
       */        
      if (l3EntryRep->hdr.status != PA_TBL_STAT_ACTIVE )
      {
        ret = pa_HANDLE_INACTIVE;
        goto Pa_addIp2_end;          
      }

      /* Specified the lut1Index for replacement */
      if (lut1Index != PAFRM_LUT1_INDEX_LAST_FREE) {
        if (lut1Index != l3EntryRep->hdr.lutIdx) {
          ret = pa_INVALID_INPUT_HANDLE;
          goto Pa_addIp2_end;
        }
      }
      else { /* did not specify the lut1 index for replacement */
        lut1Index = l3EntryRep->hdr.lutIdx;
      }

      /* Decrement the link count associated with prevLink handle if there is a prevLink */
      if (l3EntryRep->pHandle) {
        paL3Entry_t *prevl3Entry = (paL3Entry_t *) l3EntryRep->pHandle;        
        prevl3Entry->hdr.lnkCnt --;
      }

      /* No need to find the free entry as it is replace command */
      i = l3EntryRep->hdr.tableIdx;

      /* update the l3Table with the entry to be replaced */
      goto Pa_addIp2_send_cmd;
    }
  }

  /* Look for an identical entry in the table. If one is found, return it */
  /* perform entry check only if the LUT1 index is not specified by user */
  if (lut1Index == PAFRM_LUT1_INDEX_LAST_FREE) {
    for (i = 0; i < paInst->l3n; i++)  {

      if ( ((l3Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (l3Table[i].hdr.status == PA_TBL_STAT_ACTIVE)               ) &&
            (l3Table[i].hdr.subType == PA_TABLE_ENTRY_SUBYTPE_IP))    {

        if (!memcmp (&l3Table[i].u.ipInfo, &l3Entry.u.ipInfo, sizeof(paIpInfo_t)))  { 
	      paLnkHandle_t pHandle_loc = NULL;
          if (l3Table[i].pHandle)
		  	pHandle_loc  = (paLnkHandle_t)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3Table[i].pHandle);

          if (pHandle_loc == prevLink)  {

            if (l3Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) {
              *retHandle = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&(l3Table[i]));
              ret = pa_INVALID_DUP_ENTRY;
              goto Pa_addIp2_end;
                                   
            }                     
            else {
              /*  
               * Identical entry is identified, reuse the same table entry
               * Keep the entry status since it simply replaces the original entry
               * and the user may decide not to forward the packet
               */ 
              lut1Index = l3Table[i].hdr.lutIdx;
              ret = pa_DUP_ENTRY;   
              goto Pa_addIp2_send_cmd;
            }                  
          }
        }
      }
    }

    /* Check to see if this entry is more general than one already in the table.
     * If so then this entry would 'steal' packets from that entry.
     * The order of the args to each of these function is important.
     */

    for (i = 0; i < paInst->l3n; i++)  {
      paLnkHandle_t pHandle_loc = NULL;
      paLnkHandle_t nHandle_loc = NULL;
	  if (l3Table[i].pHandle)
	  	pHandle_loc  = (paLnkHandle_t)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3Table[i].pHandle);
	  if (l3Table[i].nHandle)
	  	nHandle_loc  = (paLnkHandle_t)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3Table[i].nHandle);	  

      if ( (l3Table[i].hdr.status != PA_TBL_STAT_PENDING_SUBS_REPLY)  &&
           (l3Table[i].hdr.status != PA_TBL_STAT_ACTIVE)              )
        continue;
        
      if(l3Table[i].hdr.subType != PA_TABLE_ENTRY_SUBYTPE_IP)  
        continue;

      if (prevLink != pHandle_loc)
        continue;

      if (nextLink != nHandle_loc)
        continue;

      if (!pa_ip_match (l3Entry.u.ipInfo.src,  l3Table[i].u.ipInfo.src))
        continue;

      if (!pa_ip_match (l3Entry.u.ipInfo.dst,  l3Table[i].u.ipInfo.dst))
        continue;

      if (!pa_u32_match (l3Entry.u.ipInfo.spi,  l3Table[i].u.ipInfo.spi))
        continue;

      if (!pa_u32_match (l3Entry.u.ipInfo.flow, l3Table[i].u.ipInfo.flow))
        continue;

      if (!pa_u16_match (l3Entry.u.ipInfo.greProto, l3Table[i].u.ipInfo.greProto))
        continue;

      if (!pa_u8_match (l3Entry.u.ipInfo.proto, l3Table[i].u.ipInfo.proto))
        continue;

      if(l3Entry.u.ipInfo.validBitMap & pa_IP_INFO_VALID_TOS)
        if (!pa_tos_match (l3Entry.u.ipInfo.tos, l3Table[i].u.ipInfo.tos))
          continue;

      if (!pa_u16_match (l3Entry.u.ipInfo.sctpPort, l3Table[i].u.ipInfo.sctpPort))
        continue;

      /* This is a more specific entry than one already in the table */
      ret = pa_INVALID_TABLE_MORE_SPECIFIC_ENTRY_PRESENT;
      goto Pa_addIp2_end;

    }
  }
  
  /* Find a free entry in the table */
  for (i = 0; i < paInst->l3n; i++)  {

    if (l3Table[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;
  }

  if (i == paInst->l3n)  {
    ret = pa_HANDLE_TABLE_FULL;
    goto Pa_addIp2_end;
  }
  
Pa_addIp2_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *retHandle = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l3Table[i]);

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_L3  | i, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  al1->index         = SWIZ(lut1Index);

  /* When next link is set check to see if the link is a virtual link 
   * Then set the flag to notify firmware that a virtual link will be used 
   * for the next stage. */
  if (nextLink != NULL) {
    if (((paVirtualLnk_t *)nextLink)->type != PA_TABLE_ENTRY_TYPE_VL)
    {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addIp2_end;
    }
    al1->type         = PAFRM_COM_ADD_LUT1_VLINK;
    al1->vLinkNum     = (uint8_t)(((paVirtualLnk_t *)nextLink)->tableIdx);
  }  else {
    al1->type         = PAFRM_COM_ADD_LUT1_STANDARD;
  }

  /* Ethertype holds the linked PDSP ID */
  fNonIpSpi = pa_ip_zero (l3Entry.u.ipInfo.src) && pa_ip_zero (l3Entry.u.ipInfo.dst) && (ipInfo->validBitMap & pa_IP_INFO_VALID_SPI);   
  /* vlan holds the linked PDSP LUT index */
  if (prevLink != NULL)  {
    if (virtualLinkFlag){
      al1->u.ethIp.etype = SWIZ((uint16_t)0x3);
      al1->u.ethIp.vlan  = SWIZ((uint16_t)vhdr->tableIdx);
      //vhdr->lnkCnt++;
      if (ipInfo->validBitMap & fNonIpSpi) {
        ret = pa_ERR_CONFIG;
        goto Pa_addIp2_end;
      }
    } else { 
      al1->u.ethIp.etype = SWIZ((uint16_t)hdr->pdspNum);
      al1->u.ethIp.vlan  = SWIZ((uint16_t)hdr->lutIdx);
    }
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_LINK;
  }

  /* The IP type has already been validated as either IPv4 or IPv6. IPv4
   * addresses have been pre-padded with 0 to be the same size as IPv6 */

  if (ipInfo->validBitMap & pa_IP_INFO_VALID_SRC)
  {
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_SIP;
    memcpy (al1->u.ethIp.srcIp, &l3Entry.u.ipInfo.src, sizeof(paIpv6Addr_t));
  }  

  if (ipInfo->validBitMap & pa_IP_INFO_VALID_DST)
  {
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_DIP;
    memcpy (al1->u.ethIp.dstIp, &l3Entry.u.ipInfo.dst, sizeof(paIpv6Addr_t));
  }  
  
  if (al1->u.ethIp.matchFlags & (PAFRM_LUT1_MATCH_SIP|PAFRM_LUT1_MATCH_DIP))
  {
    al1->u.ethIp.matchFlags |= PAFRM_LUT1_MATCH_KEY;
    
    /* The validity of ipType was already checked */
    if (l3Entry.u.ipInfo.ipType == pa_IPV4)
        al1->u.ethIp.key  |=  PAFRM_LUT1_KEY_IPV4;
    else
        al1->u.ethIp.key  |=  PAFRM_LUT1_KEY_IPV6;
    
  }

  if (ipInfo->validBitMap & pa_IP_INFO_VALID_SPI)
  {
    al1->u.ethIp.spi        =  SWIZ(l3Entry.u.ipInfo.spi);
    al1->u.ethIp.inport     =  PAFRM_LUT1_SPI;
  }
  else if (ipInfo->validBitMap & pa_IP_INFO_VALID_GREPROTO)
  {
    al1->u.ethIp.spi        =  SWIZ((uint32_t)l3Entry.u.ipInfo.greProto);
    al1->u.ethIp.inport     =  PAFRM_LUT1_GRE;
  }
  else if (ipInfo->validBitMap & pa_IP_INFO_VALID_SCTPPORT)  
  {
    al1->u.ethIp.spi        =   SWIZ((uint32_t)l3Entry.u.ipInfo.sctpPort);
    al1->u.ethIp.inport     =  PAFRM_LUT1_SCTP;
  }
  
  if (ipInfo->validBitMap & (pa_IP_INFO_VALID_SPI | pa_IP_INFO_VALID_GREPROTO | pa_IP_INFO_VALID_SCTPPORT))
  {
    al1->u.ethIp.matchFlags |=  (PAFRM_LUT1_MATCH_SPI_GRE_SCTP|PAFRM_LUT1_MATCH_PORT);
    al1->u.ethIp.inport     =   SWIZ(al1->u.ethIp.inport);
  }
  
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_FLOW)
  {
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_FLOW;
    al1->u.ethIp.flow        =  SWIZ(l3Entry.u.ipInfo.flow);
  }  
  
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_PROTO)
  {
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_PROTO;
    al1->u.ethIp.protoNext = l3Entry.u.ipInfo.proto;
  }  
  
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_TOS)
  {
    al1->u.ethIp.matchFlags |=  PAFRM_LUT1_MATCH_TOS;
    al1->u.ethIp.tosTclass = l3Entry.u.ipInfo.tos;
  }
    
  al1->u.ethIp.matchFlags = SWIZ(al1->u.ethIp.matchFlags);  
  al1->u.ethIp.key        = SWIZ(al1->u.ethIp.key); 
  
  ret1 = pa_conv_routing_info2(paInst, &al1->match, routeInfo, *cmdDest, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addIp2_end;
  }

  /* Next fail information */
  ret1 = pa_conv_routing_info2(paInst, &al1->nextFail, nextRtFail, *cmdDest, TRUE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addIp2_end;
  }
  
  /* find the command destination */
  if (lutInst != pa_LUT_INST_NOT_SPECIFIED) {
    switch (lutInst)
    {
        case pa_LUT1_INST_0:
            l3Entry.hdr.pdspNum = PASS_PDSP0;
            *cmdDest            = pa_CMD_TX_DEST_0;
            break;
            
        case pa_LUT1_INST_1:
            l3Entry.hdr.pdspNum = PASS_PDSP1;
            *cmdDest            = pa_CMD_TX_DEST_1;
            break;
            
        case pa_LUT1_INST_2:
            l3Entry.hdr.pdspNum = PASS_PDSP2;
            *cmdDest            = pa_CMD_TX_DEST_2;
            break;
    }
  
  }
  else if (prevLink == NULL)  {

    l3Entry.hdr.pdspNum = PASS_PDSP1;
    *cmdDest            = pa_CMD_TX_DEST_1;

  }  else  {
     
    if (!virtualLinkFlag)
    {
        if ( (hdr->type == PA_TABLE_ENTRY_TYPE_L2) ||
             (ipInfo->validBitMap & pa_IP_INFO_VALID_SPI) ) {
            l3Entry.hdr.pdspNum = PASS_PDSP1;
            *cmdDest            = pa_CMD_TX_DEST_1;
        }  else  {
            l3Entry.hdr.pdspNum = PASS_PDSP2;
            *cmdDest            = pa_CMD_TX_DEST_2;
        }
    }
    else
    {
        if (vhdr->subType == PA_TABLE_ENTRY_SUBTYPE_VLINK_MAC)  {
            l3Entry.hdr.pdspNum = PASS_PDSP1;
            *cmdDest            = pa_CMD_TX_DEST_1;
        }  else  {
            l3Entry.hdr.pdspNum = PASS_PDSP2;
            *cmdDest            = pa_CMD_TX_DEST_2;
        }
    }

  }

  if (paLObj.cfg.rmServiceHandle) {    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, (int32_t *)cmdDest, NULL)) {
      ret = pa_RESOURCE_USE_DENIED;
      goto Pa_addIp2_end;
    }
	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, (int32_t *) cmdDest, NULL)) {
      ret = pa_RESOURCE_FREE_DENIED;
	  goto Pa_addIp2_end;
    }	
  }  

  if (ret != pa_DUP_ENTRY)
  {
    /* Add the status and pdsp NUM */
    l3Entry.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
    if (prevLink != NULL)
      l3Entry.pHandle      = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr, prevLink);
    if (nextLink != NULL)
      l3Entry.nHandle      = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr, nextLink);
    l3Entry.hdr.type     = PA_TABLE_ENTRY_TYPE_L3;
    l3Entry.hdr.subType  = PA_TABLE_ENTRY_SUBYTPE_IP;
    l3Entry.hdr.tableIdx = i;
    l3Entry.hdr.lutIdx   = -1;
  
    memcpy (&l3Table[i], &l3Entry, sizeof(paL3Entry_t));
  }
  
  
Pa_addIp2_end:  

  if (l2Release) Pa_osalEndMemAccess ((void *) l2Table,
                                      paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  if (virtualLinkFlag) Pa_osalEndMemAccess ((void *)vlnkTable,
                                            paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);

  Pa_osalEndMemAccess ((void *) l3Table,
                       paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);

} /* Pa_addIp2 */

/*******************************************************************************************
 * FUNCTION PURPOSE: Add an EOAM entry to the Ethernet OAM table
 *******************************************************************************************
 * DESCRIPTION: The EOAM entry is verified and added to the EOAM table. 
 *              The following LUT1 are used to store EOAM related entries at NetCP 1.5:
 *                  - Ingess 0, PDSP1, LUT1_1
 *              This API is not supported for NetCP 1.0
 *                
 ********************************************************************************************/
  paReturn_t Pa_addEoamFlow  (Pa_Handle         iHandle,
                              paEthInfo2_t      *ethInfo,    
                              paEoamFlowInfo_t  *eoamInfo,   
                              paLnkHandle_t     *handle,     
                              paCmd_t           cmd,
                              uint16_t          *cmdSize,
                              paCmdReply_t      *reply,
                              int               *cmdDest
                             )
{
  return (pa_API_UNSUPPORTED);
}
/****************************************************************************
 * FUNCTION PURPOSE: Add a destination TCP/UDP/GTPU port to the lookup
 ****************************************************************************
 * DESCRIPTION: The destination port is added to the lookup criteria.
 ****************************************************************************/
paReturn_t  Pa_addPort ( Pa_Handle       iHandle,
                         int             portSize,
                         uint32_t        destPort,
                         paHandleL2L3_t  linkHandle,
                         uint16_t        fReplace, 
                         uint16_t        divertQ,
                         paRouteInfo_t  *routeInfo,
                         paHandleL4_t    retHandle,
                         paCmd_t         cmd,
                         uint16_t       *cmdSize,
                         paCmdReply_t   *reply,
                         int            *cmdDest )

{
  paLut2ParamDesc params;
  paRouteInfo2_t  matchRoute;
  paReturn_t  ret;
  
  if((routeInfo == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  pa_convert_routeInfo_to_routeInfo2(routeInfo,  &matchRoute);
  pa_format_lut2ParamDesc(&params, fReplace, divertQ);
  
  ret = Pa_addPort2(iHandle, portSize, destPort, linkHandle, &params, &matchRoute,
                    retHandle, cmd, cmdSize, reply, cmdDest);
                    
  return (ret);                  
  
}  /* Pa_addPort */

/****************************************************************************
 * FUNCTION PURPOSE: Add a destination TCP/UDP port to the lookup
 ****************************************************************************
 * DESCRIPTION: The 2nd generation add Port function
 ****************************************************************************/
paReturn_t  Pa_addPort2 (Pa_Handle        iHandle,
                         int              portSize,
                         uint32_t         destPort,
                         paHandleL2L3_t   linkHandle, 
                         paLut2ParamDesc *params,
                         paRouteInfo2_t  *routeInfo,
                         paHandleL4_t     retHandle,
                         paCmd_t          cmd,
                         uint16_t        *cmdSize,
                         paCmdReply_t    *reply,
                         int             *cmdDest )
{

  paInst_t              *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  pafrmCommand_t        *fcmd;
  pafrmCommandAddLut2_t *al2;
  paL2L3Header_t        *hdr;
  paVirtualLnk_t        *vhdr;
  paL4Entry_t           *hl4;
  uint16_t               csize;
  paReturn_t             ret = pa_OK;
  paReturn_t             ret1;
  uint32_t               mtCsKey;  
  int                    fReplace;
  uint8_t                virtualLinkFlag = FALSE; 
  paL3Entry_t           *l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  paVirtualLnk_t        *vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);

  if((params == NULL) || (routeInfo == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
    
  fReplace =  (params->validBitMap & pa_LUT2_PARAM_VALID_CTRL_BITMAP) &&
              (params->ctrlFlags & pa_LUT2_INFO_CONTROL_FLAG_REPLACE);    

  /* Future enhancement for RM protection of LUT2 entries */
  #if 0
  if (paLObj.cfg.rmServiceHandle) {
    int32_t dstPdsp = pa_CMD_TX_DEST_3;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &dstPdsp, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }
  }
  #endif
  
  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandAddLut2_t) - sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);
  
  *cmdSize = csize;

  /* The reply generated by the PASS must go to the host */
  if  (reply->dest != pa_DEST_HOST) 
    return (pa_INVALID_CMD_REPLY_DEST);
    
  /* Sanity check: 16-bit or 32-bit ports only */
  if ((portSize != pa_LUT2_PORT_SIZE_16) && (portSize != pa_LUT2_PORT_SIZE_32))
    return ( pa_ERR_CONFIG );
    
  if((!fReplace) && (params->validBitMap & pa_LUT2_PARAM_VALID_DIVERTQ))
    return ( pa_ERR_CONFIG );  
    
  /* Information from the linked handle */
  hdr = (paL2L3Header_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,linkHandle);
  vhdr = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, linkHandle);

  /* Initialize the return handle */
  hl4 = (paL4Entry_t *)    retHandle;
  memset (hl4, 0, sizeof(paL4Entry_t));
  hl4->lnkType = PA_TABLE_ENTRY_TYPE_NONE;
  hl4->subType = (portSize == pa_LUT2_PORT_SIZE_16)?PA_TABLE_ENTRY_SUBYTPE_PORT16:
                                                    PA_TABLE_ENTRY_SUBYTPE_PORT32;
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Verify the link */
  Pa_osalBeginMemAccess ((void *) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
                         
  /* Check to see whether we are using virtual link */
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)
    Pa_osalBeginMemAccess ((void *) vlnkTable,
                           paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  if (vhdr && vhdr->type == PA_TABLE_ENTRY_TYPE_VL)
  {
      virtualLinkFlag = TRUE;
  }
  
  if((hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16) || PA_TEST_STATE_GTPU_LINK(paInst))
  {
    /* L3 or virual link is required */
    if ((hdr == NULL) ||
       (hdr->status != PA_TBL_STAT_ACTIVE) ||
       ((hdr->type != PA_TABLE_ENTRY_TYPE_L3) && (hdr->type != PA_TABLE_ENTRY_TYPE_VL)))
    {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addPort_end;
    }
    else
    {
      hl4->lnkType = hdr->type;
      hl4->lnkTableIdx = hdr->tableIdx;
      if (hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16)
      {
        hl4->u.portInfo.lnkP  = hdr->pdspNum;
        hl4->u.portInfo.lnkId = hdr->lutIdx;
      }
    }   
  }

  if (hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16) {
    hl4->u.portInfo.destPort = (uint16_t)destPort;
  }
  else {
    if (PA_TEST_STATE_GTPU_LINK(paInst))
        hl4->u.port = (destPort << 8) | PAFRM_FORM_CUSTOM_L4_LINK(hdr->pdspNum, hdr->lutIdx);
    else
        hl4->u.port = destPort;
        
  }

  /* Create the command */   
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT2, 0, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al2 = (pafrmCommandAddLut2_t *) &(fcmd->cmd);

  al2->type  = PAFRM_COM_ADD_LUT2_STANDARD;
  al2->ctrlBitMap |= (fReplace)?PAFRM_COM_LUT2_CTRL_REPLACE:0;
  if (virtualLinkFlag){
    // populate the virtual link index
    al2->lnkTableIdx = hl4->lnkTableIdx;
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_LINK | PAFRM_COM_LUT2_CTRL_VLINK;
  } else {
    if (hl4->lnkType == PA_TABLE_ENTRY_TYPE_L3) {
      al2->lnkTableIdx = hl4->lnkTableIdx;
      al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_LINK;
    }
  }
  
  /* Forwarding information */
  ret1 = pa_conv_routing_info2(paInst, &al2->match, routeInfo, pa_CMD_TX_DEST_3, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addPort_end;
  }
  
  if (hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16) {
  
    al2->u.port.data[1] = (uint8_t) (destPort >> 8);
    al2->u.port.data[2] = destPort & 0xFF;
    al2->u.port.data[3] = PAFRM_FORM_CUSTOM_L4_LINK(hl4->u.portInfo.lnkP, hl4->u.portInfo.lnkId);
    
    if (destPort == PAFRM_DEST_PORT_GTP)
    {
        if (paInst->n2152Entries <= 0)
        {
            al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_GTPU;
            paInst->n2152Entries = 1;
        }  
        else if (!fReplace)
        {
            paInst->n2152Entries++;
        }  
    }    
    
  } else  {
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_PORT32;
    al2->u.port.data[0] = (uint8_t) (hl4->u.port >> 24);
    al2->u.port.data[1] = (uint8_t) (hl4->u.port >> 16);
    al2->u.port.data[2] = (uint8_t) (hl4->u.port >> 8);
    al2->u.port.data[3] = hl4->u.port & 0xFF;;
  }
  
  /* Queue Divert information */
  if ((params->validBitMap & pa_LUT2_PARAM_VALID_DIVERTQ)) {
    al2->qDivert.srcQ  = SWIZ(params->divertQ);
    al2->qDivert.destQ = SWIZ(routeInfo->queue);
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_QUEUE_DIVERT;
  }
  
  al2->ctrlBitMap   = SWIZ(al2->ctrlBitMap);  
  al2->lnkTableIdx  = SWIZ(al2->lnkTableIdx);
     
  /* Update the link counter */
  if (((hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16) || PA_TEST_STATE_GTPU_LINK(paInst)) &&
      (!fReplace)){
    hdr->lnkCnt++;
  }
  
  /* Only one valid destination for a LUT2 add */
  *cmdDest = pa_CMD_TX_DEST_3;
  
Pa_addPort_end:  
/* Check to see whether we are using virtual link */
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)
    Pa_osalEndMemAccess ((void *) vlnkTable,
                         paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  Pa_osalEndMemAccess ((void*)l3Table,
                       paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);

} /* Pa_addPort */

/***************************************************************************************
 * FUNCTION PURPOSE: Configure the PA L3 (LUT1)  custom lookup
 ***************************************************************************************
 * DESCRIPTION: The per-system L3 (LUT1)custom configuration parameters are setup
 ***************************************************************************************/
paReturn_t Pa_setCustomLUT1 ( Pa_Handle       iHandle,
                              uint16_t        custIndex,
                              uint16_t        parseByteOffset,
                              uint16_t        nextHdr,
                              uint16_t        nextHdrOffset,
                              uint8_t         byteMasks[pa_NUM_BYTES_CUSTOM_LUT1],
                              paCmd_t         cmd,
                              uint16_t       *cmdSize,
                              paCmdReply_t   *reply,
                              int            *cmdDest )
{
  uint16_t                csize;
  pafrmCommand_t         *fcmd;
  pafrmCommandSysConfigPa_t *ccfg;
  paInst_t              *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  uint32_t               mtCsKey;
  
  if((byteMasks == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }
  
  /* Sanity check: custom index range check */
  if (custIndex >= pa_MAX_CUSTOM_TYPES_LUT1)
    return ( pa_ERR_CONFIG );
    
  if (nextHdr > pa_HDR_TYPE_UNKNOWN)
    return ( pa_ERR_CONFIG ); 

  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t) + sizeof(pafrmC1Custom_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  /* Return the actual size of the buffer used */
  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);

  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  ccfg = (pafrmCommandSysConfigPa_t *) &(fcmd->cmd);

  ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_CUSTOM_LUT1;
  ccfg->u.customC1Config.idx    = (uint8_t)custIndex;
  ccfg->u.customC1Config.offset = SWIZ(parseByteOffset);
  ccfg->u.customC1Config.nextHdr = SWIZ(pa_next_hdr_tbl[nextHdr]);
  ccfg->u.customC1Config.nextHdrOffset = SWIZ(nextHdrOffset);
  
  memcpy (ccfg->u.customC1Config.bitMask, byteMasks, sizeof(ccfg->u.customC1Config.bitMask));

  /* Any PDSP can be used to handle the command. PDSP5 is chosen
   * since it is typically lightly loaded */
  *cmdDest = pa_CMD_TX_DEST_5;

  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);

  return (pa_OK);

} /* Pa_setCustomLUT1 */
                            
/*********************************************************************
 * FUNCTION PURPOSE: Add a custom L3 (LUT1) lookup to the tables
 *********************************************************************
 * DESCRIPTION: The entry is added to the lookup table (LUT1)
 *********************************************************************/
paReturn_t Pa_addCustomLUT1 ( Pa_Handle       iHandle,
                              uint16_t        custIndex,
                              int             lutInst,
                              int             index,
                              uint8_t         match[pa_NUM_BYTES_CUSTOM_LUT1],
                              paHandleL2L3_t  prevLink,
                              paRouteInfo_t  *routeInfo,
                              paRouteInfo_t  *nextRtFail,
                              paHandleL2L3_t *retHandle,
                              paCmd_t         cmd,
                              uint16_t       *cmdSize,
                              paCmdReply_t   *reply,
                              int            *cmdDest )
{
  /* The entry is created in the stack and then copied to the table. This allows
   * for a comparison with other table entries to verify there is not an identical
   * entry or an entry that would supercede this one */
  paInst_t				  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL3Entry_t          *l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  paL3Entry_t             l3Entry;
  paL2L3Header_t         *hdr = NULL;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  int                     i;
  uint16_t                csize;
  uint16_t                l2Release;
  uint8_t                 lut1Index = (index == pa_LUT1_INDEX_NOT_SPECIFIED)?PAFRM_LUT1_INDEX_LAST_FREE:(uint8_t)index;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey;          
  paL2Entry_t 		   *l2Table   = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);
  
  if((match == NULL) || (routeInfo == NULL) || (nextRtFail == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }
  
  /* Sanity check: custom index range check */
  if (custIndex >= pa_MAX_CUSTOM_TYPES_LUT1)
    return ( pa_ERR_CONFIG );

  /* Sanity check the LUT1 instance */
  if ((lutInst != pa_LUT_INST_NOT_SPECIFIED) && (lutInst > pa_LUT1_INST_MAX)) {
    return(pa_INVALID_LUT_INST);
  }
  
  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+sizeof(pafrmCommandAddLut1_t)-sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  /* Return the actual size of the buffer used */
  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);

  /* Create the entry on the stack, and zero the entire entry. This will
   * allow simple compare of entries using memcmp even though some fields
   * will be unused, or there will be gaps between fields */
  memset (&l3Entry, 0, sizeof(paL3Entry_t));

  /* Copy the custom information into the proposed table entry */
  memcpy (l3Entry.u.customInfo.match, match, sizeof(l3Entry.u.customInfo.match));
  l3Entry.u.customInfo.customIdx = (uint8_t)custIndex;
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Signal the application that a table modification will be done. */
  Pa_osalBeginMemAccess ((void *) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  l2Release = FALSE;

  /* If there is a link to the l2 table then that table must be
   * protected as well. Of course the previous link could be an l3
   * link as well as an l2 link. If so then the l2 protection is released */
  if (prevLink != NULL)  {
    l2Release = TRUE;
    Pa_osalBeginMemAccess ((void *)l2Table,
                           paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);

	hdr = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, prevLink);

    /* Must be linked to a complete entry */
    if (hdr->status != PA_TBL_STAT_ACTIVE)  {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addCustomLUT1_end;
    }

    if (hdr->type == PA_TABLE_ENTRY_TYPE_L3) {
      Pa_osalEndMemAccess ((void *) l2Table,
                           paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
      l2Release = FALSE;
    }  else if (hdr->type != PA_TABLE_ENTRY_TYPE_L2)  {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addCustomLUT1_end;
    }

  }

  /* Look for an identical entry in the table. If one is found, return it */
  /* perform entry check only if the LUT1 index is not specified by user */
  if (lut1Index == PAFRM_LUT1_INDEX_LAST_FREE) {
    for (i = 0; i < paInst->l3n; i++)  {

      if ( ((l3Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (l3Table[i].hdr.status == PA_TBL_STAT_ACTIVE)               ) &&
            (l3Table[i].hdr.subType == PA_TABLE_ENTRY_SUBYTPE_CUSTOM)        )    {

        if (!memcmp (&l3Table[i].u.customInfo, &l3Entry.u.customInfo, sizeof(l3Entry.u.customInfo)))  {
          if (l3Table[i].pHandle == prevLink)  {

            if (l3Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) {
              *retHandle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&(l3Table[i]));
              ret = pa_INVALID_DUP_ENTRY;   
              goto Pa_addCustomLUT1_end;
                                
            }                     
            else {
              /*  
               * Identical entry is identified, reuse the same table entry
               * Keep the entry status since it simply replaces the original entry
               * and the user may decide not to forward the packet
               */ 
              lut1Index = l3Table[i].hdr.lutIdx;
              ret = pa_DUP_ENTRY;   
              goto Pa_addCustomLut1_send_cmd;
            }                  
          }
        }
      }
    }
  }

  /* Find a free entry in the table */
  for (i = 0; i < paInst->l3n; i++)  {

    if (l3Table[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;

  }

  if (i == paInst->l3n)  {
    ret = pa_HANDLE_TABLE_FULL;
    goto Pa_addCustomLUT1_end;
  }
  
Pa_addCustomLut1_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *retHandle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l3Table[i]);

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_L3  | i, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  al1->index         = SWIZ(lut1Index);
  al1->type          = PAFRM_COM_ADD_LUT1_CUSTOM;

  /* Ethertype holds the linked PDSP ID */
  /* vlan holds the linked PDSP LUT index */
  if (prevLink != NULL)  {
    al1->u.custom.etype = SWIZ((uint16_t)hdr->pdspNum);
    al1->u.custom.vlan  = SWIZ((uint16_t)hdr->lutIdx);
  }

  memcpy (al1->u.custom.matchValues, &l3Entry.u.customInfo.match, sizeof(l3Entry.u.customInfo.match));

  /* Form the matchflags and the key */
  al1->u.custom.matchFlags = PAFRM_LUT1_CUSTOM_MATCH_MATCH | PAFRM_LUT1_CUSTOM_MATCH_KEY ;
  al1->u.custom.key        = PAFRM_LUT1_CUSTOM_KEY_CUSTOM + PAFRM_LUT1_CUSTOM_KEY_INDEX(custIndex);

  if (prevLink != NULL) 
    al1->u.custom.matchFlags |=  (PAFRM_LUT1_CUSTOM_MATCH_ETYPE | PAFRM_LUT1_CUSTOM_MATCH_VLAN);
    
  al1->u.custom.matchFlags = SWIZ(al1->u.custom.matchFlags); 
  al1->u.custom.key        = SWIZ(al1->u.custom.key);  
  
  /* Forwarding information */
  ret1 = pa_conv_routing_info(paInst, &al1->match, routeInfo, *cmdDest, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addCustomLUT1_end;
  }

  /* Next fail information */
  ret1 = pa_conv_routing_info(paInst, &al1->nextFail, nextRtFail, *cmdDest, TRUE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addCustomLUT1_end;
  }

  /* find the command destination */
  if (lutInst != pa_LUT_INST_NOT_SPECIFIED) {
    switch (lutInst)
    {
        case pa_LUT1_INST_0:
            l3Entry.hdr.pdspNum = PASS_PDSP0;
            *cmdDest            = pa_CMD_TX_DEST_0;
            break;
            
        case pa_LUT1_INST_1:
            l3Entry.hdr.pdspNum = PASS_PDSP1;
            *cmdDest            = pa_CMD_TX_DEST_1;
            break;
            
        case pa_LUT1_INST_2:
            l3Entry.hdr.pdspNum = PASS_PDSP2;
            *cmdDest            = pa_CMD_TX_DEST_2;
            break;
    }
  
  }
  else if (prevLink == NULL)  {

    l3Entry.hdr.pdspNum = PASS_PDSP1;
    *cmdDest            = pa_CMD_TX_DEST_1;

  }  else  {

    if (hdr->type == PA_TABLE_ENTRY_TYPE_L2)  {
      l3Entry.hdr.pdspNum = PASS_PDSP1;
      *cmdDest            = pa_CMD_TX_DEST_1;
    }  else  {
      l3Entry.hdr.pdspNum = PASS_PDSP2;
      *cmdDest            = pa_CMD_TX_DEST_2;
    }

    if (paLObj.cfg.rmServiceHandle) {      
      if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, (int32_t *) cmdDest, NULL)) {
        ret = pa_RESOURCE_USE_DENIED;
        goto Pa_addCustomLUT1_end;
      }
	  /* we use RM only for permission check, so freeing up immediately */
      if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, (int32_t *) cmdDest, NULL)) {
        ret = pa_RESOURCE_FREE_DENIED;
        goto Pa_addCustomLUT1_end;
      }	  
    }    
  }

  if (ret != pa_DUP_ENTRY)
  {
    /* Add the status and pdsp NUM */
    l3Entry.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
    l3Entry.pHandle      = (paLnkHandle_t) prevLink;
    l3Entry.hdr.type     = PA_TABLE_ENTRY_TYPE_L3;
    l3Entry.hdr.subType  = PA_TABLE_ENTRY_SUBYTPE_CUSTOM;
    l3Entry.hdr.tableIdx = i;
    l3Entry.hdr.lutIdx   = -1;
  
    memcpy (&l3Table[i], &l3Entry, sizeof(paL3Entry_t));
  }
  
Pa_addCustomLUT1_end:  

  if (l2Release) Pa_osalEndMemAccess ((void *) l2Table,
                                      paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalEndMemAccess ((void *) l3Table,
                       paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);
} /* Pa_addCustomLUT1 */

/***************************************************************************************
 * FUNCTION PURPOSE: Configure the PA LUT2 custom lookup
 ***************************************************************************************
 * DESCRIPTION: The per-system L4 custom configuration parameters are setup
 ***************************************************************************************/
paReturn_t Pa_setCustomLUT2 ( Pa_Handle       iHandle,
                              uint16_t        custIndex,
                              uint16_t        handleLink,
                              uint16_t        custHdrSize,
                              uint16_t        byteOffsets[pa_NUM_BYTES_CUSTOM_LUT2],
                              uint8_t         byteMasks[pa_NUM_BYTES_CUSTOM_LUT2],
                              uint8_t         setMask,
                              paCmd_t         cmd,
                              uint16_t       *cmdSize,
                              paCmdReply_t   *reply,
                              int            *cmdDest )
{
  paInst_t              *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  uint32_t               mtCsKey;
  uint16_t                csize;
  pafrmCommand_t         *fcmd;
  pafrmCommandSysConfigPa_t *ccfg;
  int                     i;
  
  if((byteOffsets == NULL) || (byteMasks == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }
  
  /* Sanity check: custom index range check */
  if (custIndex >= pa_MAX_CUSTOM_TYPES_LUT2)
    return ( pa_ERR_CONFIG );

  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t) + sizeof(pafrmC2Custom_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  /* Return the actual size of the buffer used */
  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);

  /* The byte offsets must be in increasing order */
  for (i = 0; i < pa_NUM_BYTES_CUSTOM_LUT2-1; i++)
    if (byteOffsets[i] >= byteOffsets[i+1])
      return (pa_ERR_CONFIG);

  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0, csize);
  
  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  ccfg = (pafrmCommandSysConfigPa_t *) &(fcmd->cmd);

  ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_CUSTOM_LUT2;
  ccfg->u.customC2Config.idx         = (uint8_t)custIndex;
  ccfg->u.customC2Config.hdrSize     = (uint8_t)custHdrSize;
  ccfg->u.customC2Config.bitSet      = SWIZ(setMask);
  ccfg->u.customC2Config.ctrlBitMap  = (handleLink)?PAFRM_C2_CUSTOM_CTRL_USE_LINK:0;
  ccfg->u.customC2Config.ctrlBitMap  = SWIZ(ccfg->u.customC2Config.ctrlBitMap); 
    for(i = 0; i < 4; i++)
    ccfg->u.customC2Config.offset[i] = SWIZ(byteOffsets[i]);
  memcpy (ccfg->u.customC2Config.bitMask, byteMasks,   sizeof(ccfg->u.customC2Config.bitMask));

  /* Only PDSP0 or PDSP3 can be used due to constant register configuration */
  *cmdDest = pa_CMD_TX_DEST_0;

  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);

  return (pa_OK);

} /* Pa_setCustomLUT2 */

/*********************************************************************
 * FUNCTION PURPOSE: Add a custom LUT2 lookup to the tables
 *********************************************************************
 * DESCRIPTION: The custom entry is added to the lookup table (LUT2)
 *********************************************************************/
paReturn_t Pa_addCustomLUT2 ( Pa_Handle       iHandle,
                              uint16_t        custIndex,
                              uint8_t         match[pa_NUM_BYTES_CUSTOM_LUT2],
                              paHandleL2L3_t  prevLink,
                              uint16_t        fReplace, 
                              uint16_t        divertQ,
                              paRouteInfo_t  *routeInfo,
                              paHandleL4_t    retHandle,
                              paCmd_t         cmd,
                              uint16_t       *cmdSize,
                              paCmdReply_t   *reply,
                              int            *cmdDest )
{
  paInst_t			  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut2_t  *al2;
  paL2L3Header_t         *hdr = NULL;
  paL4Entry_t            *hl4;
  uint16_t                csize;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey;        
  paL3Entry_t          *l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);

  if((match == NULL) || (routeInfo == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }
  
  #if 0
  /* Future enhancement for RM protection of LUT2 entries */
  if (paLObj.cfg.rmServiceHandle) {
    int32_t dstPdsp = pa_CMD_TX_DEST_3;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &dstPdsp, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }
	/* we use RM only for permission check, so freeing up immediately */
    if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &dstPdsp, NULL)) {
       return  pa_RESOURCE_FREE_DENIED;
    }	
  }
  #endif
  
  /* Sanity check: custom index range check */
  if (custIndex >= pa_MAX_CUSTOM_TYPES_LUT2)
    return ( pa_ERR_CONFIG );
    
  if((!fReplace) && (divertQ != pa_PARAMS_NOT_SPECIFIED))
    return ( pa_ERR_CONFIG );  

  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandAddLut2_t) - sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);
  
  *cmdSize = csize;

  /* The reply generated by the PASS must go to the host */
  if  (reply->dest != pa_DEST_HOST) 
    return (pa_INVALID_CMD_REPLY_DEST);

  /* Information from the linked handle */
  if (prevLink != NULL)
  	hdr = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, prevLink);

  /* Initialize the return handle */
  hl4 = (paL4Entry_t *)    retHandle;
  memset (hl4, 0, sizeof(paL4Entry_t));
  hl4->lnkType = PA_TABLE_ENTRY_TYPE_NONE;
  hl4->subType = PA_TABLE_ENTRY_SUBYTPE_CUSTOM;
  hl4->customIndex = (uint8_t)custIndex;
  
  /* Copy all the match bytes even if the last one is used for link */
  memcpy (hl4->u.customInfo, match, sizeof(hl4->u.customInfo));

  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  Pa_osalBeginMemAccess ((void *) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);

  /* Verify the link */
  if (hdr != NULL)  {
    if ( (hdr->type   != PA_TABLE_ENTRY_TYPE_L3) ||
         (hdr->status == PA_TBL_STAT_INACTIVE)     )  {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addCustomLUT2_end;
    }
    
    hl4->lnkType = PA_TABLE_ENTRY_TYPE_L3;
    hl4->lnkTableIdx = hdr->tableIdx;
    hl4->u.customInfo[3] = PAFRM_FORM_CUSTOM_L4_LINK(hdr->pdspNum, hdr->lutIdx);
  }
  
  /* Create the command */   
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT2, 0, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al2 = (pafrmCommandAddLut2_t *) &(fcmd->cmd);

  al2->type  = PAFRM_COM_ADD_LUT2_CUSTOM;
  al2->index = (uint8_t)custIndex;
  al2->ctrlBitMap |= (fReplace)?PAFRM_COM_LUT2_CTRL_REPLACE:0;
  
  if (hl4->lnkType == PA_TABLE_ENTRY_TYPE_L3) {
    al2->lnkTableIdx = hl4->lnkTableIdx;
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_LINK;
  }

  memcpy (al2->u.custom.match, hl4->u.customInfo, sizeof (al2->u.custom.match));
  
  /* Queue Divert information */
  if (divertQ != pa_PARAMS_NOT_SPECIFIED) {
    al2->qDivert.srcQ  = SWIZ(divertQ);
    al2->qDivert.destQ = SWIZ(routeInfo->queue);
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_QUEUE_DIVERT;
  }
  
  al2->ctrlBitMap   = SWIZ(al2->ctrlBitMap);  
  al2->lnkTableIdx  = SWIZ(al2->lnkTableIdx);

  /* Forwarding information */
  ret1 = pa_conv_routing_info(paInst, &al2->match, routeInfo, pa_CMD_TX_DEST_3, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addCustomLUT2_end;
  }
  
  /* Update the link counter */
  if (hdr != NULL) {
    hdr->lnkCnt++;
  }
  
  /* Only one valid destination for a LUT2 add */
  *cmdDest = pa_CMD_TX_DEST_3;
  
Pa_addCustomLUT2_end:  

  Pa_osalEndMemAccess ((void *) l3Table,
                       paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);

} /* Pa_addCustomLUT2 */

/***************************************************************************************
 * FUNCTION PURPOSE: Delete an L4(LUT2) handle
 ***************************************************************************************
 * DESCRIPTION: The handle is deleted. Dependent handles are left intact
 ***************************************************************************************/
paReturn_t Pa_delL4Handle (Pa_Handle      iHandle,
                           paHandleL4_t   handle, 
                           paCmd_t        cmd,
                           uint16_t      *cmdSize,
                           paCmdReply_t  *reply,
                           int           *cmdDest )

{
  paInst_t			  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL4Entry_t           *hl4;
  paL3Entry_t          *l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  paL3Entry_t           *l3e = NULL;
  paVirtualLnk_t       *vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);
  paL2L3Header_t        *hdr;
  paVirtualLnk_t        *vhdr;
  pafrmCommand_t        *fcmd;
  pafrmCommandDelLut2_t *dl2;
  uint16_t               csize;
  paReturn_t             ret = pa_OK;
  uint32_t               mtCsKey;  
  
  if((handle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandDelLut2_t) - sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;
  
  /* The reply generated by the PASS must go to the host */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);

  /* Sanity check the handle */
  hl4 = (paL4Entry_t *)handle;
  if ((hl4->lnkType != PA_TABLE_ENTRY_TYPE_L3) && 
      (hl4->lnkType != PA_TABLE_ENTRY_TYPE_VL) &&
      (hl4->lnkType != PA_TABLE_ENTRY_TYPE_NONE))
    return (pa_INVALID_INPUT_HANDLE);

  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  if (hl4->lnkType == PA_TABLE_ENTRY_TYPE_L3) {
  
    /* Inform the host a table entry will be changed */ 
    Pa_osalBeginMemAccess ((void *) l3Table,
                           paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
                           
      l3e = &l3Table[hl4->lnkTableIdx];
      hdr = &(l3e->hdr);
      if (hdr->lnkCnt > 0)
      {
        hdr->lnkCnt--; 
      }
      else
      {
        ret = pa_WARN_LNK_CNT_UNSYNC;  
      }                        

    Pa_osalEndMemAccess ((void *) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  }
  else if (hl4->lnkType == PA_TABLE_ENTRY_TYPE_VL)  {
  
    /* Inform the host a table entry will be changed */ 
    Pa_osalBeginMemAccess ((void *) vlnkTable,
                           paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
                        
                           
      vhdr = &vlnkTable[hl4->lnkTableIdx];
      if (vhdr->lnkCnt > 0)
      {
        vhdr->lnkCnt--; 
      }
      else
      {
        ret = pa_WARN_LNK_CNT_UNSYNC;  
      }                        

    Pa_osalEndMemAccess ((void *) vlnkTable,
                         paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  }

  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);


  /* Create the command */   
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_DEL_LUT2, 0, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  dl2 = (pafrmCommandDelLut2_t *) &(fcmd->cmd);
  dl2->type = PAFRM_COM_DEL_LUT2_STANDARD;
  
  if (hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16) {
  
    dl2->u.port.data[1] = (uint8_t) (hl4->u.portInfo.destPort >> 8);
    dl2->u.port.data[2] = hl4->u.portInfo.destPort & 0xFF;
    dl2->u.port.data[3] = PAFRM_FORM_CUSTOM_L4_LINK(hl4->u.portInfo.lnkP, hl4->u.portInfo.lnkId);
    
    if (hl4->u.portInfo.destPort == PAFRM_DEST_PORT_GTP)
    {
        Pa_osalMtCsEnter(&mtCsKey);
        Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
        
        if (paInst->n2152Entries == 1)
        {
            dl2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_GTPU;
            paInst->n2152Entries = 0;
        }  
        else 
        {
            if(paInst->n2152Entries >  1)
                paInst->n2152Entries--;
        }  
        
        Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
        Pa_osalMtCsExit(mtCsKey);  
        
    }
    
  } else if (hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT32){
  
    dl2->u.port.data[0] = (uint8_t) (hl4->u.port >> 24);
    dl2->u.port.data[1] = (uint8_t) (hl4->u.port >> 16);
    dl2->u.port.data[2] = (uint8_t) (hl4->u.port >> 8);
    dl2->u.port.data[3] = hl4->u.port & 0xFF;;
    
  } else {

    dl2->type = PAFRM_COM_DEL_LUT2_CUSTOM;
    dl2->index =  hl4->customIndex;
    memcpy (dl2->u.custom.match, hl4->u.customInfo, sizeof(dl2->u.custom.match));
  }
  dl2->index        = SWIZ(dl2->index);
  dl2->ctrlBitMap   = SWIZ(dl2->ctrlBitMap);
  
  /* Only one valid destination for a LUT2 add */
  *cmdDest = pa_CMD_TX_DEST_3;
  
  /* clear the L4 handle */
  memset(hl4, 0, sizeof(paL4Entry_t));

  return (ret);

} /* Pa_delL4Handle */

/******************************************************************************************************
 * FUNCTION PUROPSE: Process result from PASS subsystem
 ******************************************************************************************************
 * DESCRIPTION: The sub-system responded to a command, the result is verified
 ******************************************************************************************************/
paReturn_t Pa_forwardResult (Pa_Handle iHandle, void *vresult, paEntryHandle_t *retHandle, int *handleType, int *cmdDest)
{
  paInst_t              *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL2Entry_t           *l2Table;
  paL3Entry_t           *l3Table;
  pafrmCommand_t        *fcmd;
  pafrmCommandAddLut1_t *al1;
  pafrmCommandAddLut2_t *al2;
  paL2Entry_t           *l2e;
  paL3Entry_t           *l3e = NULL;
  paL2L3Header_t        *hdr;
  paL4Entry_t           *pL4Entry;
  int8_t                 origStatus;
  paReturn_t             ret = pa_OK;
  uint32_t               mtCsKey;  
  paVirtualLnk_t        *vlnkTable;

  if ((vresult == NULL) || (retHandle == NULL) || (handleType == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
    
  pL4Entry = (paL4Entry_t *)&retHandle->l4Handle;  
    
  /* The buffer contains the complete formatted command */
  fcmd = (pafrmCommand_t *)vresult;
  swizFcmd(fcmd);

  memset(retHandle, 0, sizeof(paEntryHandle_t));
  *handleType = pa_INVALID_HANDLE;
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  l2Table    = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);
  l3Table    = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  vlnkTable  = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);
  
  Pa_osalBeginMemAccess ((void *) l2Table,
                         paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalBeginMemAccess ((void *) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)                       
    Pa_osalBeginMemAccess ( (void *) vlnkTable,
                           paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);

  if (fcmd->command == PAFRM_CONFIG_COMMAND_ADDREP_LUT1)  {

    al1 = (pafrmCommandAddLut1_t *)&(fcmd->cmd);

    if ((fcmd->comId & PA_COMID_L_MASK) == PA_COMID_L2)  {

      l2e = &l2Table[fcmd->comId & PA_COMID_IDX_MASK];
      hdr = &(l2e->hdr);
      retHandle->l2l3Handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,l2e);
      *handleType           =  pa_L2_HANDLE;

    }  else  {

      l3e = &l3Table[fcmd->comId & PA_COMID_IDX_MASK];
      hdr = &(l3e->hdr);
      retHandle->l2l3Handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,l3e);
      *handleType           =  pa_L3_HANDLE;
    }

    origStatus = hdr->status;

    if (fcmd->commandResult != PAFRM_COMMAND_RESULT_SUCCESS)  {
      hdr->status = PA_TBL_STAT_INACTIVE;
     
      if (origStatus == PA_TBL_STAT_ACTIVE)
      {
        /* Duplicate entry, remove the link */
        if (l3e && l3e->pHandle)
        {
            hdr = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3e->pHandle);
            hdr->lnkCnt--;
        }
      }
      
      ret = pa_LUT_ENTRY_FAILED;

    }  else  {

      origStatus  = hdr->status;
      hdr->status = PA_TBL_STAT_ACTIVE;
      hdr->lutIdx = SWIZ(al1->index);
      
      if (l3e && l3e->pHandle && (origStatus != PA_TBL_STAT_ACTIVE)) {
        hdr = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3e->pHandle);
        hdr->lnkCnt++;
      }

    }

  }  else if (  (fcmd->command == PAFRM_CONFIG_COMMAND_ADDREP_LUT2)  ||
                (fcmd->command == PAFRM_CONFIG_COMMAND_DEL_LUT2)      )   {

    al2 = (pafrmCommandAddLut2_t *)&(fcmd->cmd);

    /* The handle must be recreated */
    pL4Entry->lnkType =  PA_TABLE_ENTRY_TYPE_NONE;
    
    if (SWIZ(al2->ctrlBitMap) & PAFRM_COM_LUT2_CTRL_LINK) {
      pL4Entry->lnkType =  PA_TABLE_ENTRY_TYPE_L3;
      pL4Entry->lnkTableIdx = SWIZ(al2->lnkTableIdx);
      if(SWIZ(al2->ctrlBitMap) & PAFRM_COM_LUT2_CTRL_VLINK)
        pL4Entry->lnkType =  PA_TABLE_ENTRY_TYPE_VL;
    }
    
    if (SWIZ(al2->type) == PAFRM_COM_ADD_LUT2_STANDARD) {
      if (SWIZ(al2->ctrlBitMap) & PAFRM_COM_LUT2_CTRL_PORT32) {
        pL4Entry->subType = PA_TABLE_ENTRY_SUBYTPE_PORT32;
        pL4Entry->u.port = (al2->u.port.data[0] << 24) +
                           (al2->u.port.data[1] << 16) +
                           (al2->u.port.data[2] << 8) +
                           (al2->u.port.data[3]); 
      
      }
      else {
        pL4Entry->subType = PA_TABLE_ENTRY_SUBYTPE_PORT16;
        pL4Entry->u.portInfo.destPort = (al2->u.port.data[1] << 8) + al2->u.port.data[2];
        pL4Entry->u.portInfo.lnkP  =  PAFRM_GET_PDSPID_FROM_LINK(al2->u.port.data[3]);
        pL4Entry->u.portInfo.lnkId =  PAFRM_GET_LUTIDX_FROM_LINK(al2->u.port.data[3]);
      }
    
    }
    else {
      /* Custom type */
      pL4Entry->subType = PA_TABLE_ENTRY_SUBYTPE_CUSTOM;
      pL4Entry->customIndex = SWIZ(al2->index);
      memcpy (pL4Entry->u.customInfo, al2->u.custom.match, sizeof (al2->u.custom.match));
    }

    *handleType         = pa_L4_HANDLE;

    /* The LUT2 can be busy with a different add. There is only room
     * to store one add while one is in progress */
    if (fcmd->commandResult == PAFRM_COMMAND_RESULT_LUT2_ADD_BUSY)  {
      *cmdDest = pa_CMD_TX_DEST_3;
      ret = pa_RESUBMIT_COMMAND;
      /* Swizzle back the header */
      fcmd->commandResult = 0U;
      swizFcmd(fcmd);
    }
    else if (fcmd->commandResult == PAFRM_COMMAND_RESULT_LUT2_FULL) {
      ret = pa_LUT2_TABLE_FULL;
    }

  }  else  {

    /* For other commands the Sub-System return value was not really required */

  }
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)
    Pa_osalEndMemAccess ((void *)vlnkTable,
                         paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  Pa_osalEndMemAccess ((void*)l3Table,
                       paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  Pa_osalEndMemAccess ((void*)l2Table,
                       paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);  

}  /* Pa_forwardResult */
        
/************************************************************************************
 * FUNCTION PURPOSE: Configure sub-system routing of exception conditions
 ************************************************************************************
 * DESCRIPTION: Packets which satifies the exception conditions can be routed to the 
 *              host 
 ************************************************************************************/
paReturn_t Pa_configExceptionRoute2(Pa_Handle       iHandle,
                                    int             nRoute,
                                    int            *routeTypes,
                                    paRouteInfo2_t *eRoutes,
                                    paCmd_t         cmd,
                                    uint16_t       *cmdSize,
                                    paCmdReply_t   *reply,
                                    int            *cmdDest)
{
  paInst_t			        *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  pafrmCommand_t            *fcmd;
  pafrmCommandSysConfigPa_t *cpa;
  int                       i;
  uint16_t                  csize;
  paReturn_t                retCode;
  uint32_t                  mtCsKey;  
  
  if((routeTypes == NULL) || (eRoutes == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t) + sizeof(pafrmComEroute_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* The destination for the command must be host or discard */
  if (  (reply->dest != pa_DEST_HOST)     &&
        (reply->dest != pa_DEST_DISCARD)  )
    return (pa_INVALID_CMD_REPLY_DEST);

  /* Make sure there is at least one route */
  if ((nRoute <= 0) || (nRoute > pa_EROUTE_MAX))
    return (pa_ERR_CONFIG);

  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0, csize);

  /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  cpa = (pafrmCommandSysConfigPa_t *)&(fcmd->cmd);
  
  cpa->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_EROUTE;

  cpa->u.eroute.routeBitMap = 0;
  
  for (i = 0; i < nRoute; i++)  {

    if (  (eRoutes[i].dest != pa_DEST_HOST)    &&
          (eRoutes[i].dest != pa_DEST_EMAC)    &&
          (eRoutes[i].dest != pa_DEST_DISCARD)  )
    {      
      retCode = pa_ERR_CONFIG;
      break;
    }
      
    if ( routeTypes[i] >= pa_EROUTE_MAX  )
    {
      retCode = pa_ERR_CONFIG;
      break;  
    }  

    cpa->u.eroute.routeBitMap |=  (1 << routeTypes[i]);
    
    retCode =  pa_conv_routing_info2 (paInst, &cpa->u.eroute.eRoute[routeTypes[i]], &eRoutes[i], pa_CMD_TX_DEST_5, FALSE, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
    
    if(retCode != pa_OK)
      break;
  }
  
  cpa->u.eroute.routeBitMap = SWIZ(cpa->u.eroute.routeBitMap);

  /* Destination can be any PDSP, but 4 is used since it is lightly loaded */
  *cmdDest = pa_CMD_TX_DEST_4;
  
  if (retCode != pa_OK)
  {
    /* No need to send the command packets */
    *cmdSize = 0;
  }  
  
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);

  return (retCode);

} /* Pa_configExceptionRoute2 */

/************************************************************************************
 * FUNCTION PURPOSE: Configure sub-system routing of exception conditions
 ************************************************************************************
 * DESCRIPTION: Packets which satifies the exception conditions can be routed to the 
 *              host 
 ************************************************************************************/
paReturn_t Pa_configExceptionRoute(Pa_Handle       iHandle,
                                   int             nRoute,
                                   int            *routeTypes,
                                   paRouteInfo_t  *eRoutes,
                                   paCmd_t         cmd,
                                   uint16_t       *cmdSize,
                                   paCmdReply_t   *reply,
                                   int            *cmdDest)
{
  int               i;
  paRouteInfo2_t    routeInfo2[pa_EROUTE_MAX];
                   
  
  /* Make sure there is at least one route */
  if ((nRoute <= 0) || (nRoute > pa_EROUTE_MAX))
    return (pa_ERR_CONFIG);
    
  if((routeTypes == NULL) || (eRoutes == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
    
  for ( i = 0; i < nRoute; i++)
  {
    pa_convert_routeInfo_to_routeInfo2(&eRoutes[i], &routeInfo2[i]);
  }  
  
  return(Pa_configExceptionRoute2(iHandle, nRoute, routeTypes, routeInfo2, 
                                  cmd, cmdSize, reply, cmdDest));

} /* Pa_configExceptionRoute */

/************************************************************************************
 * FUNCTION PURPOSE: Configure multi-route groups
 ************************************************************************************
 * DESCRIPTION: Configures the multi-route group which consists of multiple
 *              packet routing entries
 ************************************************************************************/
paReturn_t Pa_configMultiRoute (Pa_Handle               iHandle,
                                paMultiRouteModes_e     mode,
                                uint16_t                index,
                                uint16_t                nRoute,
                                paMultiRouteEntry_t    *routeEntry,
                                paCmd_t                 cmd,
                                uint16_t               *cmdSize,
                                paCmdReply_t           *reply,
                                int                    *cmdDest)
{
  paInst_t              *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  uint32_t               mtCsKey;
  pafrmCommand_t           *fcmd;
  pafrmCommandMultiRoute_t *mr;
  int                       i;
  uint16_t                  csize;

  if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandMultiRoute_t) - sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* The destination for the command must be host or discard */
  if (  (reply->dest != pa_DEST_HOST)     &&
        (reply->dest != pa_DEST_DISCARD)  )
    return (pa_INVALID_CMD_REPLY_DEST);

  /* Make sure there is at least one entry  */
  if (mode == pa_MULTI_ROUTE_MODE_CONFIG) {
  	if ((nRoute == 0) || (nRoute > pa_MAX_MULTI_ROUTE_ENTRIES))
    return (pa_ERR_CONFIG);
  }
    
  /* Sanity check: multi-route set index range and operation mode */
  if(index >= pa_MAX_MULTI_ROUTE_SETS)
    return (pa_ERR_CONFIG);  
    
  if((mode != pa_MULTI_ROUTE_MODE_CONFIG) && (mode != pa_MULTI_ROUTE_MODE_RESET))
    return (pa_ERR_CONFIG);  

  if ((mode == pa_MULTI_ROUTE_MODE_CONFIG) && (routeEntry == NULL))
    return (pa_ERR_CONFIG);

  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_MULTI_ROUTE, 0, csize);

  /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  mr = (pafrmCommandMultiRoute_t *)&(fcmd->cmd);
  
  mr->idx = (uint8_t)index;
  
  if (mode == pa_MULTI_ROUTE_MODE_CONFIG) {
    mr->mode = PAFRM_COMMAND_MULTI_ROUTE_MODE_ADD;
    mr->nRoutes = (uint8_t)nRoute;
    
    for (i = 0; i < nRoute; i++)  {
      mr->quFl[i].ctrlFlags = routeEntry[i].ctrlBitfield | PAFRM_MULTI_RUOTE_CTRL_ACTIVE;
      mr->quFl[i].ctrlFlags = SWIZ(mr->quFl[i].ctrlFlags);
      mr->quFl[i].flowId = SWIZ(routeEntry[i].flowId);
      mr->quFl[i].queue = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC, routeEntry[i].queue));
      mr->quFl[i].swInfo0 = SWIZ(routeEntry[i].swInfo0);
    }
  }
  else {
    mr->mode = PAFRM_COMMAND_MULTI_ROUTE_MODE_DEL;
  }                                                

  /* Destination should be either PDSP4 or PDSP5 where multiple routes is executed */
  *cmdDest = pa_CMD_TX_DEST_5;

  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);

  return (pa_OK);

} /* Pa_configMultiRoute */

/************************************************************************************
 * FUNCTION PURPOSE: Initialize the CRC table
 ************************************************************************************
 * DESCRIPTION: Initialize the CRC table based on the 32-bit polynomial 
 ************************************************************************************/
static void pa_init_crc_table4(uint32_t polynomial, uint32_t *crc_table4)
{
  uint32_t  i,bit;
  
  /* 16 values representing all possible 4-bit values */
  for(i=0; i<PAFRM_CRC_TABLE_SIZE; i++) {
    crc_table4[i]=i<<28;
    for (bit=0; bit<4; bit++) {
      /* If shifting out a zero, then just shift */
      if( !(crc_table4[i] & 0x80000000) )
        crc_table4[i] = (crc_table4[i] << 1);
      /* Else add in the polynomial as well */
      else
        crc_table4[i] = (crc_table4[i] << 1) ^ polynomial;
    }
    crc_table4[i] = SWIZ(crc_table4[i]);
  }
}

/************************************************************************************
 * FUNCTION PURPOSE: Configure CRC engine
 ************************************************************************************
 * DESCRIPTION: This function is used to configure the specified CRC engine by 
 *   formating the required CRC configuration command packet.
 ************************************************************************************/
paReturn_t Pa_configCrcEngine (Pa_Handle       iHandle,
                               uint16_t        index,
                               paCrcConfig_t  *cfgInfo,
                               paCmd_t         cmd,
                               uint16_t       *cmdSize,
                               paCmdReply_t   *reply,
                               int            *cmdDest)
{
  paInst_t                 *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  uint32_t                  mtCsKey;
  pafrmCommand_t           *fcmd;
  pafrmCommandConfigCRC_t  *ccrc;
  uint16_t                  csize;

  if((cfgInfo == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
   
  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandConfigCRC_t) - sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* The destination for the command must be host or discard */
  if (  (reply->dest != pa_DEST_HOST)     &&
        (reply->dest != pa_DEST_DISCARD)  )
    return (pa_INVALID_CMD_REPLY_DEST);

  /* Sanity check: there is only 6 CRC engines */
  if(index >= PAFRM_NUM_CRC_ENGINES)
    return (pa_ERR_CONFIG);  
    
  if(cfgInfo->size > pa_CRC_SIZE_32)
    return (pa_ERR_CONFIG);   
    
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_CRC_ENGINE, 0, csize);

  /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  ccrc = (pafrmCommandConfigCRC_t *)&(fcmd->cmd);
  
  ccrc->ctrlBitMap = (uint8_t) cfgInfo->size;
  ccrc->ctrlBitMap |= (cfgInfo->ctrlBitfield & pa_CRC_CONFIG_RIGHT_SHIFT)?PAFRM_CRC_CTRL_RIGHT_SHIFT:
                                                                          PAFRM_CRC_CTRL_LEFT_SHIFT;
  ccrc->ctrlBitMap |= (cfgInfo->ctrlBitfield & pa_CRC_CONFIG_INVERSE_RESULT)? PAFRM_CRC_CTRL_INV_RESULT:0;
                                                                          
  ccrc->ctrlBitMap = SWIZ(ccrc->ctrlBitMap);
  ccrc->initVal = SWIZ(cfgInfo->initValue);
  
  pa_init_crc_table4(cfgInfo->polynomial, ccrc->crcTbl);

  /* Destination is the PDSP corresping to the CRC engine index */
  *cmdDest = (int)index;

  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);

  return (pa_OK);

} /* Pa_configCrcEngine */

/************************************************************************************
 * FUNCTION PURPOSE: Configure Command Set
 ************************************************************************************
 * DESCRIPTION: This function is used to configure the command set which consists 
 *   of a list of commands 
 ************************************************************************************/
paReturn_t Pa_configCmdSet (Pa_Handle       iHandle,
                            uint16_t        index,
                            int             nCmd,
                            paCmdInfo_t    *cmdInfo,
                            paCmd_t         cmd,
                            uint16_t       *cmdSize,
                            paCmdReply_t   *reply,
                            int            *cmdDest)
{
  paInst_t			  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  pafrmCommand_t       *fcmd;
  pafrmCommandCmdSet_t *cmdSet;
  uint16_t              csize;
  uint8_t               buf[PAFRM_MAX_CMD_SET_SIZE + 20];
  int                   i;
  uint8_t               maxCmdSets;
  uint16_t              offset, cmdOffset;
  uint16_t              maxCmdSize, maxUsrCounters; 
  uint32_t              mtCsKey;  
  
  if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  if((nCmd > 0) && (cmdInfo == NULL))
    return (pa_ERR_CONFIG);  
  
  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandCmdSet_t) - sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* The destination for the command must be host or discard */
  if (  (reply->dest != pa_DEST_HOST)     &&
        (reply->dest != pa_DEST_DISCARD)  )
    return (pa_INVALID_CMD_REPLY_DEST);
    
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  maxCmdSets     = paInst->cfg.cmdSetConfig.numCmdSets;
  maxUsrCounters = paInst->cfg.usrStatsConfig.numCounters;
  
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  /* Sanity check: command set index range check */
  if (index >= maxCmdSets)
  {
    return (pa_ERR_CONFIG); 
  }   
    
  maxCmdSize = (maxCmdSets == 32)?128 - offsetof(pafrmCommandCmdSet_t, cmd):
                                  64 - offsetof(pafrmCommandCmdSet_t, cmd);  
    
  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_CMD_SET, 0, csize);

  /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  cmdSet = (pafrmCommandCmdSet_t *)&(fcmd->cmd);
  
  cmdSet->idx = (uint8_t)index;
  cmdSet->numCmd = (uint8_t)nCmd;
  
  offset = 0;
  cmdOffset = 0;
  
  for (i = 0; i < nCmd; i++) {
    pafrmRxCmdHdr_t *pCmdHdr = (pafrmRxCmdHdr_t *)&buf[cmdOffset];
    
    switch (cmdInfo[i].cmd)
    {
      
      case pa_CMD_NEXT_ROUTE:
        {
          paCmdNextRoute_t* route = &cmdInfo[i].params.route;
          pafrmRxCmdNextRoute_t* nr = (pafrmRxCmdNextRoute_t *)&buf[cmdOffset + sizeof(pafrmRxCmdHdr_t)];
          
          /* 
           * All the routing information should be in the packet already 
           */
          if(route->ctrlBitfield & pa_NEXT_ROUTE_PARAM_PRESENT)
            return (pa_ERR_CONFIG);
            
          if (route->ctrlBitfield & pa_NEXT_ROUTE_PROC_MULTI_ROUTE) {
            if(route->multiRouteIndex >= pa_MAX_MULTI_ROUTE_SETS)
              return (pa_ERR_CONFIG);  
            nr->ctrlFlags |= PAFRM_RX_NEXT_ROUTE_CTRL_MULTI_ROUTE;
            nr->multiRouteIndex = (uint8_t)route->multiRouteIndex;
          }
          
          if (route->dest == pa_DEST_EMAC)
          {
            nr->ctrlFlags |= PAFRM_RX_NEXT_ROUTE_CTRL_EMAC_ROUTE;
            if (route->pktType_emacCtrl)
            {
                nr->ctrlFlags |= PAFRM_RX_NEXT_ROUTE_CTRL_PSFLAGS_VALID;
                nr->psFlags = ((route->pktType_emacCtrl & pa_EMAC_CTRL_PORT_MASK) << PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
                nr->psFlags |= ((route->pktType_emacCtrl & pa_EMAC_CTRL_CRC_DISABLE)? PAFRM_ETH_PS_FLAGS_DISABLE_CRC:0);                
            }
          }
          
          pCmdHdr->cmd = PAFRM_RX_CMD_NEXT_ROUTE;
          pCmdHdr->len = sizeof(pafrmRxCmdNextRoute_t) + sizeof(pafrmRxCmdHdr_t);

        }
        break;
      
      case pa_CMD_CRC_OP:
        {
          paCmdCrcOp_t  *crc = &cmdInfo[i].params.crcOp;
          pafrmRxCmdCrcOp_t *ptx = (pafrmRxCmdCrcOp_t *)&buf[cmdOffset  + sizeof(pafrmRxCmdHdr_t)];
          
          if (!(crc->ctrlBitfield & pa_CRC_OP_CRC_VALIDATE))
          {
            /* 
             * Support CRC verification only in rx direction
             */
            return (pa_ERR_CONFIG); 
          
          } 
          
          if (!(crc->ctrlBitfield & pa_CRC_OP_CRC_FRAME_TYPE))
          {
            /*
             * It is not possible to verify the start offset with variable-length header
             */
            if(crc->startOffset < offset)
                return(pa_ERR_CONFIG);
                
            offset = crc->startOffset;
               
          }
          
          ptx->ctrlFlags = (crc->ctrlBitfield & pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD)?PAFRM_RX_CRC_OP_CTRL_CRC_FOLLOW_PAYLOAD:0;
          ptx->ctrlFlags |= (crc->ctrlBitfield & pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER)?PAFRM_RX_CRC_OP_CTRL_PAYLOAD_LENGTH_IN_HEADER:0;
          ptx->ctrlFlags |= (crc->ctrlBitfield & pa_CRC_OP_PAYLOAD_LENGTH_OFFSET_IS_NEGATIVE)?PAFRM_RX_CRC_OP_CTRL_LEN_OFFSET_NEGATIVE:0;
          if (crc->ctrlBitfield & pa_CRC_OP_CRC_FRAME_TYPE)
          {
            ptx->ctrlFlags |= PAFRM_RX_CRC_OP_CTRL_FRAME_TYPE_INCLUDED;
            
            /* Make sure that the frame type is supported */
            if(crc->frameType > pa_CRC_OP_FRAME_TYPE_MAX)
                return(pa_ERR_CONFIG);
                
            ptx->ctrlFlags |= crc->frameType;    
          }
          ptx->ctrlFlags = SWIZ(ptx->ctrlFlags);
          ptx->lenAdjust = (uint8_t)crc->lenAdjust;
          ptx->startOffset = SWIZ(crc->startOffset);
          ptx->lenOffset = SWIZ(crc->lenOffset);
          ptx->lenMask = SWIZ(crc->lenMask);
          ptx->crcOffset = SWIZ(crc->crcOffset);
          ptx->len = SWIZ(crc->len);
          
          pCmdHdr->cmd = PAFRM_RX_CMD_CRC_OP;
          pCmdHdr->len = sizeof(pafrmRxCmdCrcOp_t) + sizeof(pafrmRxCmdHdr_t);
          
        }
        break;
        
      case pa_CMD_COPY_DATA_TO_PSINFO:
        {
          paCmdCopy_t* pCopy = &cmdInfo[i].params.copy;
          pafrmRxCmdCopy_t *pCmdCopy = (pafrmRxCmdCopy_t *)&buf[cmdOffset  + sizeof(pafrmRxCmdHdr_t)];
          
          if((pCopy->numBytes + pCopy->destOffset) > 32)
            return(pa_ERR_CONFIG);
          
          pCmdCopy->ctrlFlags  = (pCopy->ctrlBitfield & pa_COPY_OP_FROM_END)?PAFRM_RX_COPY_CTRL_FROM_END:0;
          pCmdCopy->ctrlFlags  = SWIZ(pCmdCopy->ctrlFlags);
          pCmdCopy->srcOffset  = (uint8_t)pCopy->srcOffset;
          pCmdCopy->destOffset = (uint8_t)pCopy->destOffset;
          pCmdCopy->numBytes   = (uint8_t)pCopy->numBytes;
          
          pCmdHdr->cmd = PAFRM_RX_CMD_COPY_DATA;
          pCmdHdr->len = sizeof(pafrmRxCmdCopy_t) + sizeof(pafrmRxCmdHdr_t);

        }
        break;
      
      case pa_CMD_PATCH_DATA:
        {
          paPatchInfo_t* patch = &cmdInfo[i].params.patch;
          pafrmRxCmdPatch_t *pCmdPatch = (pafrmRxCmdPatch_t *)&buf[cmdOffset  + sizeof(pafrmRxCmdHdr_t)];
          int fData = !(patch->ctrlBitfield & pa_PATCH_OP_DELETE);
          
          if((patch->offset < offset)  || (patch->nPatchBytes > pa_MAX_RX_PATCH_BYTES) || 
             (fData && (patch->patchData == NULL)))
            return(pa_ERR_CONFIG);
            
          pCmdPatch->ctrlFlags = (uint8_t)patch->ctrlBitfield;
          pCmdPatch->offset    = (uint8_t)patch->offset;
          pCmdPatch->numBypes  = (uint8_t)patch->nPatchBytes; 
          
          if(fData)
            memcpy(&pCmdPatch->data, patch->patchData, patch->nPatchBytes);
          
          pCmdHdr->cmd = PAFRM_RX_CMD_PATCH_DATA;
          pCmdHdr->len = sizeof(pafrmRxCmdPatch_t) + sizeof(pafrmRxCmdHdr_t);
          
          if((patch->nPatchBytes > 4) && fData)
            pCmdHdr->len += (((patch->nPatchBytes - 4) + 3)/4)*4; 
          
          offset = patch->offset;

        }
        break;
        
      case pa_CMD_SPLIT:
        {
          paCmdSplitOp_t      *split = &cmdInfo[i].params.split;
          pafrmRxCmdSplitOp_t *pCmdSplit = (pafrmRxCmdSplitOp_t *)&buf[cmdOffset + sizeof(pafrmRxCmdHdr_t)];
                
          pCmdHdr->cmd = PAFRM_RX_CMD_SPLIT;
          pCmdHdr->len = sizeof(pafrmRxCmdSplitOp_t) + sizeof(pafrmRxCmdHdr_t);
          
          pCmdSplit->ctrlFlags = 0;
          if (split->ctrlBitfield & pa_SPLIT_OP_FRAME_TYPE)
          {
            pCmdSplit->ctrlFlags |= PAFRM_RX_SPLIT_OP_CTRL_FRAME_TYPE_INCLUDED;
            
            /* Make sure that the frame type is supported */
            if(split->frameType > pa_CRC_OP_FRAME_TYPE_MAX)
                return(pa_ERR_CONFIG);
                
            pCmdSplit->frameType = (uint8_t)split->frameType;    
          }
          pCmdSplit->ctrlFlags = SWIZ(pCmdSplit->ctrlFlags);
          pCmdSplit->flowId    = (uint8_t)split->flowId;
          pCmdSplit->destQueue = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC, split->destQueue));
          pCmdSplit->startOffset = (uint8_t)split->startOffset;
          
        }
        break;
        
      
      case pa_CMD_REMOVE_HEADER:
        {
          /* It must be the first command before packet data has been used */
          if(offset)
            return (pa_ERR_CONFIG);
            
          pCmdHdr->cmd = PAFRM_RX_CMD_REMOVE_HDR;
          pCmdHdr->len = sizeof(pafrmRxCmdHdr_t);
            
        }
        break;
        
      case pa_CMD_REMOVE_TAIL:
        {
          
          pCmdHdr->cmd = PAFRM_RX_CMD_REMOVE_TAIL;
          pCmdHdr->len = sizeof(pafrmRxCmdHdr_t);
            
        }
        break;
        
      
      case pa_CMD_MULTI_ROUTE:
        {
          paCmdMultiRoute_t      *mRoute = &cmdInfo[i].params.mRoute;
          pafrmRxCmdMultiRoute_t *pCmdMroute = (pafrmRxCmdMultiRoute_t *)pCmdHdr;
          
          if(mRoute->index >= pa_MAX_MULTI_ROUTE_SETS)
            return (pa_ERR_CONFIG);  
          
          pCmdMroute->cmd = PAFRM_RX_CMD_MULTI_ROUTE;
          pCmdMroute->len = sizeof(pafrmRxCmdMultiRoute_t);
          pCmdMroute->index = (uint8_t)mRoute->index;
        
        }
        break;
        
      case pa_CMD_USR_STATS:
        {
          paCmdUsrStats_t      *usrStats = &cmdInfo[i].params.usrStats;
          pafrmRxCmdUsrStats_t *pCmdUsrStats = (pafrmRxCmdUsrStats_t *)pCmdHdr;
          
          if(usrStats->index >= maxUsrCounters)
            return (pa_ERR_CONFIG);  
            
          if(pa_verify_usr_stats(paInst, (int32_t)usrStats->index, TRUE))
            return (pa_RESOURCE_USE_DENIED);                      
          
          pCmdUsrStats->cmd = PAFRM_RX_CMD_USR_STATS;
          pCmdUsrStats->len = sizeof(pafrmRxCmdUsrStats_t);
          pCmdUsrStats->index = SWIZ(usrStats->index);
        
        }
        break;
        
      case pa_CMD_VERIFY_PKT_ERROR:
        {
          paCmdVerifyPktErr_t      *verifyPktErr = &cmdInfo[i].params.verifyPktErr;
          pafrmRxCmdVerifyPktErr_t *pCmdVerifyPktErr = (pafrmRxCmdVerifyPktErr_t *)&buf[cmdOffset  + sizeof(pafrmRxCmdHdr_t)];
          
          pCmdHdr->cmd = PAFRM_RX_CMD_VERIFY_PKT_ERROR;
          pCmdHdr->len = sizeof(pafrmRxCmdVerifyPktErr_t) + sizeof(pafrmRxCmdHdr_t);
          
          switch (verifyPktErr->dest)
          {
            case pa_DEST_DISCARD:
                pCmdVerifyPktErr->forwardType = PAFRM_FORWARD_TYPE_DISCARD;
                break;
                
            case pa_DEST_HOST:
                pCmdVerifyPktErr->forwardType = PAFRM_FORWARD_TYPE_HOST;   
                pCmdVerifyPktErr->flowId  = SWIZ(verifyPktErr->flowId);
                pCmdVerifyPktErr->queue   = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC, verifyPktErr->queue));
                pCmdVerifyPktErr->swInfo0 = SWIZ(verifyPktErr->swInfo0);
                break;
                    
            default:
                return (pa_ERR_CONFIG);  
          }
          
          pCmdVerifyPktErr->errMask = (verifyPktErr->errorBitfield & pa_PKT_ERR_IP_CHECKSUM)?PAFRM_RX_PKT_ERR_IP_CHECKSUM:0;
          pCmdVerifyPktErr->errMask |= (verifyPktErr->errorBitfield & pa_PKT_ERR_L4_CHECKSUM)?PAFRM_RX_PKT_ERR_L4_CHECKSUM:0;
          pCmdVerifyPktErr->errMask |= (verifyPktErr->errorBitfield & pa_PKT_ERR_CRC)?PAFRM_RX_PKT_ERR_CRC:0;
          pCmdVerifyPktErr->errMask = SWIZ(pCmdVerifyPktErr->errMask);
          pCmdVerifyPktErr->forwardType = SWIZ(pCmdVerifyPktErr->forwardType);
        
        }
        break;
      
      default:
        return (pa_ERR_CONFIG);
    }
  
    cmdOffset += pCmdHdr->len;
    pCmdHdr->len = SWIZ(pCmdHdr->len);
    
    if(cmdOffset > maxCmdSize)
      return (pa_CMDSET_TOO_BIG);
  }
  
  /* copy the constructed commands */
  memcpy((uint8_t *)cmdSet->cmd, buf, maxCmdSize);

  /* Destination should be either PDSP4 or PDSP5 where cmdSet is executed */
  *cmdDest = pa_CMD_TX_DEST_5;

  return (pa_OK);

} /* Pa_configCmdSet */

/************************************************************************************
 * FUNCTION PURPOSE: Verify link table of the user-defined statistics
 ************************************************************************************
 * DESCRIPTION: The function verify the link table by detecting close loop which means
 *              the same counter appears more than once in its link chain
 * 
 * Return: TRUE:  if no loop detected
 *         FALSW: if loop is detected
 *
 * Note: It is not necessary to perform range check in this function since the link
 *       counter index has been verified by the caller 
 ************************************************************************************/
 static int pa_verify_usr_stats_lnk_tbl(uint16_t numStats, paUsrStatsLnkEntry_t* lnktbl)
 {
    uint32_t bitMask[(pa_USR_STATS_MAX_COUNTERS + 31)>>5];
    int bitIndex, bitOffset;
    int cntIndex, index;
    
    for (index = 0; index < numStats; index++)
    {
        /* clear the  bitMask array */
        memset(bitMask, 0, sizeof(bitMask));
        
        cntIndex = index;
        
        while (lnktbl[cntIndex].lnkIndex != PA_USR_STATS_LNK_END)
        {
            /* Can not link to itself */
            if(lnktbl[cntIndex].lnkIndex == cntIndex)
                return (FALSE);
        
            /* Is the same counter used? */
            bitIndex = cntIndex >> 5; 
            bitOffset = cntIndex & 0x1F;
            
            if (bitMask[bitIndex] & (1 << bitOffset))
            {
                return FALSE;
            }
            
            bitMask[bitIndex] |= (1 << bitOffset);
            
            cntIndex = lnktbl[cntIndex].lnkIndex;
        
        }
    }
    
    return (TRUE);
 
 }

/************************************************************************************
 * FUNCTION PURPOSE: Configure User-defined Statistics
 ************************************************************************************
 * DESCRIPTION: This function is used to perform the counter configuration of 
 *              the multi-level hierarchical user-defined statistics which consists 
 *              of up to 64 64-bit counters and up to 192 32-bit counters.
 ************************************************************************************/
paReturn_t Pa_configUsrStats (Pa_Handle                iHandle,
                              paUsrStatsConfigInfo_t  *cfgInfo,
                              paCmd_t                  cmd,
                              uint16_t                *cmdSize,
                              paCmdReply_t            *reply,
                              int                     *cmdDest)
{
  paInst_t			  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paUsrStatsCounterConfig_t* pCntCfg;
  paUsrStatsCounterEntryConfig_t *cntInfo;
  pafrmCommand_t           *fcmd;
  pafrmUsrStatsCntCfg_t    *cntCfg;
  pafrmUsrStatsEntry_t     *pEntry;
  int                       i;
  uint16_t                  csize, maxUsrCounters;
  paReturn_t                errCode = pa_OK;
  paUsrStatsLnkEntry_t      usrStatsLnkTbl[pa_USR_STATS_MAX_COUNTERS];      /* local copy of the user stats link table */
  uint32_t                  mtCsKey;  
  int                       fLnkTblRelease = FALSE;
  paUsrStatsLnkEntry_t *uslnkTable = (paUsrStatsLnkEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_USR_STATS_LNK_TABLE].base);    
  
  if((cfgInfo == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  maxUsrCounters = paInst->cfg.usrStatsConfig.numCounters;
  
  /* Sanity Check  */
  if ((cfgInfo == NULL) || ((pCntCfg = cfgInfo->pCntCfg) == NULL) || ((pCntCfg->numCnt > 0) && (pCntCfg->cntInfo == NULL)) ||(pCntCfg->numCnt > maxUsrCounters))
  {
    errCode = pa_ERR_CONFIG;
  }
  else
  {
    /* Verify that there is enough room to create the command */
    csize = sizeof(pafrmCommand_t) + sizeof(pafrmUsrStatsCntCfg_t) - sizeof(uint32_t) + sizeof(pafrmUsrStatsEntry_t) * pCntCfg->numCnt;
    if (*cmdSize < csize)
    {
        errCode = pa_INSUFFICIENT_CMD_BUFFER_SIZE;
    }
    else
    {
        *cmdSize = csize;
        /* The destination for the command must be host or discard */
        if (  (reply->dest != pa_DEST_HOST)     &&
            (reply->dest != pa_DEST_DISCARD)  )
            errCode =  pa_INVALID_CMD_REPLY_DEST;
    }

  }  
  
  if (errCode != pa_OK)
  {
    goto Pa_configUsrStats_end;
  }  
    
  Pa_osalBeginMemAccess ((void *) uslnkTable,
                         paInst->paBufs[PA_BUFFNUM_USR_STATS_LNK_TABLE].size);
                         
  fLnkTblRelease = TRUE;                        
                         
  memcpy(usrStatsLnkTbl, uslnkTable, paInst->nUsrStats * sizeof(paUsrStatsLnkEntry_t));                       
    
  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_USR_STATS, 0, csize);

  /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  cntCfg = (pafrmUsrStatsCntCfg_t *)&(fcmd->cmd);
  
  if (pCntCfg->ctrlBitfield & pa_USR_STATS_CONFIG_RESET)
  {
    cntCfg->ctrlFlags |= PAFRM_USR_STATS_CFG_CLR_ALL;
    for (i = 0; i < paInst->nUsrStats; i++)  {
        usrStatsLnkTbl[i].lnkIndex = PA_USR_STATS_LNK_END;
    }
  }  
    
  cntCfg->ctrlFlags = SWIZ(cntCfg->ctrlFlags);
  cntCfg->nEntries  = SWIZ(pCntCfg->numCnt);
  
  if (pCntCfg->numCnt)
  {
    pEntry = (pafrmUsrStatsEntry_t *)((uint8_t *)&(fcmd->cmd) + sizeof(pafrmUsrStatsCntCfg_t));
    cntInfo = pCntCfg->cntInfo;
  
    for (i = 0; i < pCntCfg->numCnt; i++)  {
    
        if ((cntInfo[i].cntIndex >= maxUsrCounters)  ||
            ((cntInfo[i].cntLnk != pa_USR_STATS_LNK_END) && (cntInfo[i].cntLnk >= maxUsrCounters)))
        {
            errCode = pa_ERR_CONFIG;
            break;
        }
        else if (pa_verify_usr_stats(paInst, (int32_t)cntInfo[i].cntIndex, FALSE)) 
        {
            errCode = pa_RESOURCE_USE_DENIED;
            break;
        }
        else
        {
            pEntry[i].index = SWIZ(cntInfo[i].cntIndex);
            usrStatsLnkTbl[cntInfo[i].cntIndex].lnkIndex =
            pEntry[i].lnkIndex = (cntInfo[i].cntLnk == pa_USR_STATS_LNK_END)?PAFRM_USR_STATS_LNK_END:cntInfo[i].cntLnk;
            if(cntInfo[i].cntType == pa_USR_STATS_TYPE_BYTE)
                pEntry[i].lnkIndex |= PAFRM_USR_STATS_BYTE_CNT;
            else if (cntInfo[i].cntType == pa_USR_STATS_TYPE_DISABLE) 
            {
                pEntry[i].lnkIndex = PAFRM_USR_STATS_LNK_END | PAFRM_USR_STATS_DISABLE;
                usrStatsLnkTbl[cntInfo[i].cntIndex].lnkIndex = PAFRM_USR_STATS_LNK_END;
            }   
            pEntry[i].lnkIndex = SWIZ(pEntry[i].lnkIndex);    
        }
        
    }                                      
  
    /*  Verify the updated link table  */
    if((errCode == pa_OK) && (!pa_verify_usr_stats_lnk_tbl(paInst->nUsrStats, usrStatsLnkTbl)))
        errCode = pa_ERR_CONFIG;  
  } 
  
  /* All user-defined statistics related configuration should occur at PDSP4 */
  *cmdDest = pa_CMD_TX_DEST_4;
            
  if (errCode == pa_OK) 
  {
    memcpy(uslnkTable, usrStatsLnkTbl, paInst->nUsrStats * sizeof(paUsrStatsLnkEntry_t)); 
  } 
  else
  {
    /* No need to send the command */
    *cmdSize = 0;
  }
  
Pa_configUsrStats_end:  
  if (fLnkTblRelease)   
  {     
    Pa_osalEndMemAccess (uslnkTable,
                         paInst->paBufs[PA_BUFFNUM_USR_STATS_LNK_TABLE].size);
  }                     
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);
  
  return (errCode);

} /* Pa_configUsrStats */                             

/************************************************************************************
 * FUNCTION PURPOSE: System-level control and configuration
 ************************************************************************************
 * DESCRIPTION: Perform system-level control configuration by updating the local
 *              instance and/or format the global configuration command packet
 *              host 
 ************************************************************************************/
paReturn_t Pa_control (Pa_Handle      iHandle, 
                       paCtrlInfo_t  *ctrl, 
                       paCmd_t        cmd,
                       uint16_t       *cmdSize,
                       paCmdReply_t   *reply,
                       int            *cmdDest)
{
  paInst_t			     *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paSysConfig_t          *cfg = &ctrl->params.sysCfg;
  pafrmCommand_t         *fcmd;
  uint16_t               csize;
  paReturn_t             ret = pa_OK;
  uint32_t               mtCsKey;  
  
  if((ctrl == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  if((ctrl->code > pa_CONTROL_MAX_CONFIG_GEN1))
    return (pa_ERR_CONFIG);
    
  /* The destination for the command must be host or discard */
  if (  (reply->dest != pa_DEST_HOST)     &&
        (reply->dest != pa_DEST_DISCARD)  )
    return (pa_INVALID_CMD_REPLY_DEST);

  if (ctrl->code == pa_CONTROL_SYS_CONFIG)
  {
    pafrmCommandConfigPa_t *cpa;
  
    /* Verify that there is enough room to create the command */
    csize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandConfigPa_t) - sizeof(uint32_t);
    
    if (*cmdSize < csize)
        return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

    *cmdSize = csize;

    Pa_osalMtCsEnter(&mtCsKey);
    Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

    /* Create the command */
    fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_CONFIG_PA, 0, csize);

    /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
    * must be considered */
    if (reply->dest == pa_DEST_HOST)
        fcmd->replyDest = PAFRM_DEST_PKTDMA;
    else
        fcmd->replyDest = PAFRM_DEST_DISCARD;
    
    cpa = (pafrmCommandConfigPa_t *)&(fcmd->cmd);
  
    /* Protocol Limitations */
    if (cfg->pProtoLimit)
    {
        paProtocolLimit_t* pPLcfg = cfg->pProtoLimit;
    
        /* range check */
        if ((pPLcfg->vlanMax <= pa_PROTOCOL_LIMIT_NUM_VLANS_MAX)  &&
            (pPLcfg->ipMax <=  pa_PROTOCOL_LIMIT_NUM_IP_MAX)      &&
            (pPLcfg->greMax <= pa_PROTOCOL_LIMIT_NUM_GRE_MAX))
        { 
            paInst->cfg.protoLimit = *pPLcfg;
            cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_MAX_COUNTS;
            cpa->maxCounts.vlanMaxCount = SWIZ(pPLcfg->vlanMax);
            cpa->maxCounts.ipMaxCount   = SWIZ(pPLcfg->ipMax);
            cpa->maxCounts.greMaxCount  = SWIZ(pPLcfg->greMax);
        }
        else
        {
            ret = pa_ERR_CONFIG;    
        }
  
    }
  
    /* Queue Bounce configuration */
    if (cfg->pQueueBounceConfig)
    {
        int i;
        paQueueBounceConfig_t* pQueueBounceCfg = cfg->pQueueBounceConfig;

        paInst->cfg.queueBounceConfig = *pQueueBounceCfg;

        for (i = 0; i < PA_MAX_QUEUE_BOUNCE_ROUTING_CLASSES; i++)
        {
            if (pQueueBounceCfg->defOp[i] > pa_QUEUE_BOUNCE_OP_MAX)
            {
                paInst->cfg.queueBounceConfig.enable = FALSE;
                ret = pa_ERR_CONFIG;
                break;
            }
        }

        if (ret == pa_OK)
        {
            cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_QUEUE_BOUNCE;
            cpa->queueBounce.ddrQueue     = SWIZ(pQueueBounceCfg->ddrQueueId);
            cpa->queueBounce.msmcQueue    = SWIZ(pQueueBounceCfg->msmcQueueId);
        }

        /* the return queue may be updated per this configuration */
        fcmd->replyQueue = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_CMD_RET, reply->queue));
    }

    /* Outer IP configuration */
    if (cfg->pOutIpReassmConfig)
    {
        paIpReassmConfig_t* pIpReassmCfg = cfg->pOutIpReassmConfig;
    
        /* range check */
        if (pIpReassmCfg->numTrafficFlow > pa_MAX_IP_REASM_TRAFFIC_FLOWS) 
        {  
            ret = pa_ERR_CONFIG;
        }
        else
        {
            paInst->cfg.outIpReassmConfig = *pIpReassmCfg;
            cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_OUT_IP_REASSEM;
            cpa->outIpReasm.numTrafficFlow = SWIZ(pIpReassmCfg->numTrafficFlow);
            cpa->outIpReasm.destFlowId     = SWIZ(pIpReassmCfg->destFlowId);
            cpa->outIpReasm.destQueue      = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_IP_REASSEMBLY, pIpReassmCfg->destQueue));
        }
    }
  
    /* Inner IP configuration */
    if (cfg->pInIpReassmConfig)
    {
        paIpReassmConfig_t* pIpReassmCfg = cfg->pInIpReassmConfig;
    
        /* range check */
        if (pIpReassmCfg->numTrafficFlow > pa_MAX_IP_REASM_TRAFFIC_FLOWS) 
        {  
            ret = pa_ERR_CONFIG;
        }
        else
        {
            paInst->cfg.inIpReassmConfig = *pIpReassmCfg;
            cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_IN_IP_REASSEM;
            cpa->inIpReasm.numTrafficFlow = SWIZ(pIpReassmCfg->numTrafficFlow);
            cpa->inIpReasm.destFlowId     = SWIZ(pIpReassmCfg->destFlowId);
            cpa->inIpReasm.destQueue      = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_IP_REASSEMBLY, pIpReassmCfg->destQueue));
        }
    }
  
    /* Command Set Configurations */
    if (cfg->pCmdSetConfig)
    {
        paCmdSetConfig_t* pCmdSetCfg = cfg->pCmdSetConfig;
    
        /* range check */
        if ((pCmdSetCfg->numCmdSets != 32) &&
            (pCmdSetCfg->numCmdSets != 64))
        {  
            ret = pa_ERR_CONFIG;
        }
        else
        {
            paInst->cfg.cmdSetConfig = *pCmdSetCfg;
            cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_CMDSET;
            if (pCmdSetCfg->numCmdSets == 32)
            {
                cpa->cmdSet.numCmdSets = 32;
                cpa->cmdSet.size = 128;
            }
            else
            {
                cpa->cmdSet.numCmdSets = 64;
                cpa->cmdSet.size = 64;
            }
            cpa->cmdSet.numCmdSets = SWIZ(cpa->cmdSet.numCmdSets);
            cpa->cmdSet.size       = SWIZ(cpa->cmdSet.size);
        }
    }
  
    /* User-defined statistics configuration*/
    if (cfg->pUsrStatsConfig)
    {
        paUsrStatsConfig_t* pUsrStatsCfg = cfg->pUsrStatsConfig;
    
        /* range check */
        if ((pUsrStatsCfg->numCounters > paInst->nUsrStats)            || 
            (pUsrStatsCfg->num64bCounters > pUsrStatsCfg->numCounters) ||
            ((pUsrStatsCfg->num64bCounters*8 + (pUsrStatsCfg->numCounters - pUsrStatsCfg->num64bCounters)*4) > pa_USR_STATS_MAX_64B_COUNTERS*8))
        {  
            ret = pa_ERR_CONFIG;
        }
        else
        {
            paInst->cfg.usrStatsConfig = *pUsrStatsCfg;
            cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_USR_STATS;
            cpa->usrStats.numCounters = SWIZ(pUsrStatsCfg->numCounters);
            cpa->usrStats.num64bCounters = SWIZ(pUsrStatsCfg->num64bCounters);
        }
    }
  
    /* Queue Divert configuration */
    if (cfg->pQueueDivertConfig)
    {
        paQueueDivertConfig_t* pQueueDevertCfg = cfg->pQueueDivertConfig;
    
        paInst->cfg.queueDivertConfig = *pQueueDevertCfg;
        cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_QUEUE_DIVERT;
        cpa->queueDivert.destFlowId     = SWIZ(pQueueDevertCfg->destFlowId);
        cpa->queueDivert.destQueue      = SWIZ(pQueueDevertCfg->destQueue);
    }
    
    /* Packet Verification configuration */
    if ( (cfg->pPktControl) ||
         (cfg->pPktControl2) )
    {
        paPacketControlConfig_t*  pPktControlCfg  = cfg->pPktControl;
        paPacketControl2Config_t* pPktControlCfg2 = cfg->pPktControl2;
        paPacketControl2Config_t  pktCtrlCfg2;
        
        cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_PKT_CTRL;

        if (cfg->pPktControl) 
        {
            /* All bits are set as valid */
            pktCtrlCfg2.validBitMap = (
                            pa_PKT_CTRL2_VALID_PPPoE_HDR_CHECK    | \
                            pa_PKT_CTRL2_VALID_IP_HDR_CHECK       | \
                            pa_PKT_CTRL2_VALID_MAC_PADDING_CHECK  | \
                            pa_PKT_CTRL2_VALID_IP_FRAGS_TO_EROUTE | \
                            pa_PKT_CTRL2_VALID_L3_OFFSET          | \
                            pa_PKT_CTRL2_VALID_PADDING_STATS_INDEX
                          );
             pktCtrlCfg2.ctrlBitMap             = pPktControlCfg->ctrlBitMap;
             pktCtrlCfg2.rxPaddingErrStatsIndex = pPktControlCfg->rxPaddingErrStatsIndex;
             pktCtrlCfg2.txPaddingStatsIndex    = pPktControlCfg->txPaddingStatsIndex;
        }
        else
        {
             memcpy(&pktCtrlCfg2, pPktControlCfg2, sizeof (paPacketControl2Config_t) );
        }

        cpa->pktCtrl.ctrlBitMap  =  SWIZ(pktCtrlCfg2.ctrlBitMap);
        cpa->pktCtrl.validBitMap =  SWIZ(pktCtrlCfg2.validBitMap); 

        if (pktCtrlCfg2.validBitMap & pa_PKT_CTRL2_VALID_PADDING_STATS_INDEX) 
        {     
            if(pa_verify_usr_stats(paInst, (int32_t)pktCtrlCfg2.rxPaddingErrStatsIndex, FALSE))
                ret = pa_RESOURCE_USE_DENIED;
            else                          
                cpa->pktCtrl.rxPaddingErrCntIndex = SWIZ(pktCtrlCfg2.rxPaddingErrStatsIndex);
                
            if(pa_verify_usr_stats(paInst, (int32_t)pktCtrlCfg2.txPaddingStatsIndex, FALSE))
                ret = pa_RESOURCE_USE_DENIED;
            else    
                cpa->pktCtrl.txPaddingCntIndex = SWIZ(pktCtrlCfg2.txPaddingStatsIndex);
        }
        
        if ((pktCtrlCfg2.validBitMap & pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_EQoS_MODE) &&
            (pktCtrlCfg2.ctrlBitMap & pa_PKT_CTRL_EMAC_IF_EGRESS_EQoS_MODE))
        {
            cpa->pktCtrl.egressDefPri = (uint8_t)pktCtrlCfg2.egressDefPri;
        }
        
    }

    /* Destination can be any PDSP, but 4 is used since it is lightly loaded */
    *cmdDest = pa_CMD_TX_DEST_4;

    if ((!cpa->validFlag) || (ret != pa_OK))
    {
        /* There is no command to be sent */
        *cmdSize = 0;
        ret = pa_ERR_CONFIG;
    }
  
    cpa->validFlag = SWIZ(cpa->validFlag);
  
    Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
    Pa_osalMtCsExit(mtCsKey);
  }
  else
  {
    pafrmCommandSysConfigPa_t *ccfg;
  
    /* Verify that there is enough room to create the command */
    csize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t);

    if (ctrl->code == pa_CONTROL_EMAC_PORT_CONFIG)
         csize += sizeof (pafrmEQosModeConfig_t);
    else
         csize += (ctrl->code == pa_CONTROL_802_1ag_CONFIG)?sizeof(pafrm802p1agDet_t):sizeof(pafrmIpsecNatTDet_t);		

    if (*cmdSize < csize)
        return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

    *cmdSize = csize;

    Pa_osalMtCsEnter(&mtCsKey);
    Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

    /* Create the command */
    fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0, csize);

    /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
     * must be considered */
    if (reply->dest == pa_DEST_HOST)
        fcmd->replyDest = PAFRM_DEST_PKTDMA;
    else
        fcmd->replyDest = PAFRM_DEST_DISCARD;
    
    ccfg = (pafrmCommandSysConfigPa_t *) &(fcmd->cmd);


	switch (ctrl->code) 
    {
        case pa_CONTROL_802_1ag_CONFIG:
        {
    		pa802p1agDetConfig_t  *pa802p1agDet = &ctrl->params.pa802p1agDetCfg;
            ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_802_1AG;
            ccfg->u.pa802p1agDet.ctrlBitMap   = (pa802p1agDet->ctrlBitMap & pa_802_1ag_DETECT_ENABLE)?PAFRM_802_1ag_DET_ENABLE:0;
            ccfg->u.pa802p1agDet.ctrlBitMap  |= (pa802p1agDet->ctrlBitMap & pa_802_1ag_DETECT_STANDARD)?PAFRM_802_1ag_DET_STANDARD:0;
            ccfg->u.pa802p1agDet.ctrlBitMap  =  SWIZ(ccfg->u.pa802p1agDet.ctrlBitMap);
            /* 802.1ag detector is at PDSP0 only */
            *cmdDest = pa_CMD_TX_DEST_0;
  		  break;
        }
		
        case pa_CONTROL_IPSEC_NAT_T_CONFIG:
  	  	{
            paIpsecNatTConfig_t *ipsecNatTDet = &ctrl->params.ipsecNatTDetCfg;
          
            ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_IPSEC_NAT_T;
            ccfg->u.ipsecNatTDet.ctrlBitMap = (ipsecNatTDet->ctrlBitMap & pa_IPSEC_NAT_T_CTRL_ENABLE)?PAFRM_IPSEC_NAT_T_DET_ENABLE:0;
            ccfg->u.ipsecNatTDet.ctrlBitMap = SWIZ(ccfg->u.ipsecNatTDet.ctrlBitMap);
            ccfg->u.ipsecNatTDet.udpPort    = SWIZ(ipsecNatTDet->udpPort);
      
            /* IPSEC NAT-T detector is at PDSP3 only */
            *cmdDest = pa_CMD_TX_DEST_3;
  		  break;
        }
  		
        case pa_CONTROL_GTPU_CONFIG:
  	  	{
            paGtpuConfig_t *gtpuCfg = &ctrl->params.gtpuCfg;
          
            ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_GTPU;
          
            if (gtpuCfg->ctrlBitMap & pa_GTPU_CTRL_USE_LINK)
            {
              PA_SET_STATE_GTPU_LINK(paInst, 1);
              ccfg->u.gtpuCfg.ctrlBitMap = (uint8_t)PAFRM_GTPU_CTRL_USE_LINK;
            }
            else
            {
              PA_SET_STATE_GTPU_LINK(paInst, 0);
              ccfg->u.gtpuCfg.ctrlBitMap = (uint8_t)0;
            }

            if (gtpuCfg->ctrlBitMap & pa_GTPU_CTRL_ROUTE_END_MARKER_AS_GPDU)
            {
              ccfg->u.gtpuCfg.ctrlBitMap |= (uint8_t)PAFRM_GTPU_CTRL_ROUTE_END_MARKER_AS_GPDU;            
            }         
                    
            /* GTP-U classification is at PDSP3 only */
            *cmdDest = pa_CMD_TX_DEST_3;
      
            break;
        }
        case pa_CONTROL_EMAC_PORT_CONFIG:
        {
            paEmacPortConfig_t     *portCfg = &ctrl->params.emacPortCfg;
            ret = pa_set_emac_port_cfg_frm_cmd(paInst, ccfg, portCfg, cmdDest);
            break;
        }

        default:
          ret = pa_ERR_CONFIG;
          break;       
      }

    Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
    Pa_osalMtCsExit(mtCsKey);

  }
  
  return (ret);

} /* Pa_control */

/********************************************************************************
 * FUNCTION PURPOSE: Request Sub-System statistics
 ********************************************************************************
 * DESCRIPTION: The command to request statistics is created
 ********************************************************************************/
static paReturn_t pa_format_stats_request  (paInst_t      *paInst, 
                                            uint16_t       doClear, 
                                            uint16_t       doRequest,
                                            uint16_t       nCnt,
                                            uint16_t      *cntIndex, 
                                            paCmd_t        cmd, 
                                            uint16_t      *cmdSize, 
                                            paCmdReply_t  *reply, 
                                            int           *cmdDest,
                                            uint8_t       statsType)
{

  pafrmCommand_t          *fcmd;
  pafrmCommandReqStats_t  *rstat;
  uint16_t                csize;
  
  if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }

  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandReqStats_t) - sizeof(uint32_t);
  
  if ((nCnt != pa_USR_STATS_CLEAR_ALL) && doClear)
    csize += sizeof(pafrmUsrStatsClrStats_t);
    
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* Verify the reply destination */
  if (reply->dest != pa_DEST_HOST)
    return (pa_ERR_CONFIG);

  /* Form the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_REQ_STATS, 0, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  rstat = (pafrmCommandReqStats_t *)&(fcmd->cmd);

  rstat->ctrlBitMap = doClear?PAFRM_STATS_CTRL_CLR:0;
  rstat->ctrlBitMap |= (doRequest?PAFRM_STATS_CTRL_REQ:0);
  
  rstat->type  = statsType;
  rstat->numCnt = SWIZ(nCnt);
  
  if ((nCnt != pa_USR_STATS_CLEAR_ALL) && doClear)
  {
    int i;
    pafrmUsrStatsClrStats_t *cstat = (pafrmUsrStatsClrStats_t *)((uint8_t *)rstat + sizeof(pafrmCommandReqStats_t));
    
    for ( i = 0; i < nCnt; i++)
    {
      if (pa_verify_usr_stats(paInst, (int32_t)cntIndex[i], TRUE))
      {
        /* No need to send the command packets */
        *cmdSize = 0;
        *cmdDest = pa_CMD_TX_DEST_4;
        return (pa_RESOURCE_USE_DENIED);   
      }                     
      
      cstat->bitMap[cntIndex[i] >> 5] |= (1 << (cntIndex[i] % 32)); 
    }
    
    for ( i = 0; i < PA_USR_STATS_BITMAP_SIZE; i++)
      cstat->bitMap[i] = SWIZ(cstat->bitMap[i]);
  }

  /* The usr-stats reset command must go to PDSP4 */
  *cmdDest = pa_CMD_TX_DEST_4;

  return (pa_OK);

}  

/********************************************************************************
 * FUNCTION PURPOSE: Request Sub-System statistics
 ********************************************************************************
 * DESCRIPTION: The command to request statistics is created
 ********************************************************************************/
paReturn_t Pa_requestStats (Pa_Handle      handle,
                            uint16_t       doClear, 
                            paCmd_t        cmd, 
                            uint16_t      *cmdSize, 
                            paCmdReply_t  *reply, 
                            int           *cmdDest)
{

  paInst_t  *paInst =  (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, handle);

  return(pa_format_stats_request(paInst, doClear, TRUE, pa_USR_STATS_CLEAR_ALL, NULL, 
                                 cmd, cmdSize, reply, cmdDest, PAFRM_STATS_TYPE_SYS));

}  /* Pa_requestStats */

/********************************************************************************
 * FUNCTION PURPOSE: Request User-defined statistics List
 ********************************************************************************
 * DESCRIPTION: Query a set of User-defined statistics 
 ********************************************************************************/
paReturn_t Pa_requestUsrStatsList (Pa_Handle      handle,
                                   uint16_t       doClear,
                                   uint16_t       nCnt,
                                   uint16_t      *cntIndex, 
                                   paCmd_t        cmd, 
                                   uint16_t      *cmdSize, 
                                   paCmdReply_t  *reply, 
                                   int           *cmdDest,
                                   paUsrStats_t  *pUsrStats)
{

  paInst_t      *paInst =  (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, handle);
  uint16_t       numCounters;
  uint32_t       mtCsKey;  
  uint16_t       doRequest = FALSE;
  
  if((cmdSize == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }
  
    /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr == (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
  
  if((nCnt != pa_USR_STATS_CLEAR_ALL) && (cntIndex == NULL))
    return (pa_ERR_CONFIG);
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  numCounters    = paInst->cfg.usrStatsConfig.numCounters;
  
  if (nCnt > numCounters)
  {
    Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
    Pa_osalMtCsExit(mtCsKey);
    return (pa_ERR_CONFIG);  
  }
  
  if (pUsrStats)
  {
    CSL_Pa_ssRegs *passRegs;
    uint16_t       num64bCounters, num32bCounters;
    uint32_t      *pCounter;
    int            i;
  
    /* non-autonomous reading */
    num64bCounters = paInst->cfg.usrStatsConfig.num64bCounters;
    num32bCounters = numCounters - num64bCounters;
    passRegs = (CSL_Pa_ssRegs *)paLObj.cfg.baseAddr;
    pCounter = (uint32_t *)&passRegs->PDSP_SRAM[PAFRM_USR_STATS_SRAM_INDEX].PDSP_RAM[PAFRM_USR_STATS_OFFSET];
  
    /* Format and copy 64-bit counters */
    for (i = 0; i < num64bCounters; i++, pCounter += 2)
    {
        pUsrStats->count64[i] = (((uint64_t)pCounter[0]) << 32) + (uint64_t)pCounter[1]; 
    }
  
    /* Format and copy 32-bit counters */
    for (i = 0; i < num32bCounters; i++)
    {
        pUsrStats->count32[i] = pCounter[i]; 
    }
  }
  else
  {
    doRequest = TRUE;
  }
  
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);
  
  if (doClear || doRequest)
  {
    return(pa_format_stats_request(paInst, doClear, doRequest, nCnt, cntIndex, cmd, cmdSize, reply, cmdDest, PAFRM_STATS_TYPE_USR));
  }
  else
  {
    *cmdSize = 0;
    *cmdDest = pa_CMD_TX_DEST_4;

    return (pa_OK);
  }

}  /* Pa_requestUsrStatsList */

/********************************************************************************
 * FUNCTION PURPOSE: Request User-defined statistics
 ********************************************************************************
 * DESCRIPTION: The command to request User-defined statistics is created
 ********************************************************************************/
paReturn_t Pa_requestUsrStats (Pa_Handle      handle,
                               uint16_t       doClear, 
                               paCmd_t        cmd, 
                               uint16_t      *cmdSize, 
                               paCmdReply_t  *reply, 
                               int           *cmdDest,
                               paUsrStats_t  *pUsrStats)
{
    return(Pa_requestUsrStatsList (handle, doClear, pa_USR_STATS_CLEAR_ALL, NULL, 
                                   cmd, cmdSize, reply, cmdDest, pUsrStats));
}  /* Pa_requestUsrStats */

/****************************************************************************
 * FUNCTION PURPOSE: Format the repy generated to a stats request
 ****************************************************************************
 * DESCRIPTION: The stats request reply is swizzled and a pointer to the
 *              stats is returned
 ****************************************************************************/
paSysStats_t* Pa_formatStatsReply (Pa_Handle  handle,
                                   paCmd_t    cmd)
{
  pafrmCommand_t *fcmd;
  pafrmCommandReqStats_t *rstat;
  
  paSysStats_t   *statsRep;
  
  if(cmd == NULL)
    return (NULL);
    
  fcmd     = (pafrmCommand_t *)cmd;
  swizFcmd(fcmd);
  
  rstat = (pafrmCommandReqStats_t *)&(fcmd->cmd);

  /* Verify the command was valid */
  if ((fcmd->command != PAFRM_CONFIG_COMMAND_REQ_STATS) ||
      (rstat->type != PAFRM_STATS_TYPE_SYS))
    return (NULL);
    
  statsRep = (paSysStats_t *)((uint8_t*)&(fcmd->cmd) + sizeof(pafrmCommandReqStats_t));

  swizStatsRep (statsRep);

  return (statsRep);

} /* Pa_formatStatsReply */

/****************************************************************************
 * FUNCTION PURPOSE: Format the repy generated to a user stats request
 ****************************************************************************
 * DESCRIPTION: The stats request reply is swizzled and a pointer to the
 *              stats is returned
 ****************************************************************************/
paReturn_t Pa_formatUsrStatsReply (Pa_Handle      handle,
                                   paCmd_t        cmd,
                                   paUsrStats_t  *pUsrStats)
{
  pafrmCommand_t *fcmd;
  pafrmCommandReqStats_t *rstat;
  
  if ((cmd == NULL) || (pUsrStats == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  fcmd     = (pafrmCommand_t *)cmd;
  swizFcmd(fcmd);
  
  rstat = (pafrmCommandReqStats_t *)&(fcmd->cmd);

  /* Verify the command was valid */
  if ((fcmd->command != PAFRM_CONFIG_COMMAND_REQ_STATS) ||
      (rstat->type != PAFRM_STATS_TYPE_USR))             
    return (pa_ERR_CONFIG);
    
  if (rstat->ctrlBitMap & PAFRM_STATS_CTRL_REQ)
  {
    paInst_t      *paInst =  (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, handle);
    uint16_t       numCounters, num64bCounters, num32bCounters;
    uint32_t      *pCounter;
    int            i;
    uint32_t       mtCsKey;  
  
    if(pUsrStats == NULL)
      return (pa_ERR_CONFIG);          
  
    /* format the statistics */
    Pa_osalMtCsEnter(&mtCsKey);
    Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
    
    numCounters    = paInst->cfg.usrStatsConfig.numCounters;
    num64bCounters = paInst->cfg.usrStatsConfig.num64bCounters;
    
    Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
    Pa_osalMtCsExit(mtCsKey);
    
    num32bCounters = numCounters - num64bCounters;
    pCounter = (uint32_t *)((uint8_t*)&(fcmd->cmd) + sizeof(pafrmCommandReqStats_t));
    
  
    /* Format and copy 64-bit counters */
    for (i = 0; i < num64bCounters; i++, pCounter += 2)
    {
        pUsrStats->count64[i] = (((uint64_t)SWIZ(pCounter[0])) << 32) + (uint64_t)SWIZ(pCounter[1]); 
    }
  
    /* Format and copy 32-bit counters */
    for (i = 0; i < num32bCounters; i++)
    {
        pUsrStats->count32[i] = SWIZ(pCounter[i]); 
    }
  
  }  

  return (pa_OK);

} /* Pa_formatUsrStatsReply */

/********************************************************************************
 * FUNCTION PURPOSE: Query Sub-System statistics
 ********************************************************************************
 * DESCRIPTION: This API queries sub-system statistics
 ********************************************************************************/
paReturn_t Pa_querySysStats   (Pa_Handle      handle,
                               uint16_t       doClear, 
                               paSysStats_t  *pSysStats)
{
  return (pa_API_UNSUPPORTED);
}  /* Pa_querySysStats */

/********************************************************************************
 * FUNCTION PURPOSE: Query RA statistics
 ********************************************************************************
 * DESCRIPTION: This API queries RA statistics
 ********************************************************************************/
paReturn_t Pa_queryRaStats   (Pa_Handle      handle,
                              uint16_t       doClear, 
                              paRaStats_t   *pRaStats)
{
  return (pa_API_UNSUPPORTED);
}  /* Pa_queryRaStats */

/********************************************************************************
 * FUNCTION PURPOSE: Query ACL statistics
 ********************************************************************************
 * DESCRIPTION: This API queries ACL statistics
 ********************************************************************************/
paReturn_t Pa_queryAclStats (Pa_Handle      handle,
                             paHandleAcl_t  aclHandle,
                             uint16_t       doClear, 
                             paAclStats_t  *pAclStats)
{
  return (pa_API_UNSUPPORTED);
} /* Pa_queryAclStats */                            

/********************************************************************************
 * FUNCTION PURPOSE: Query Flow Cache statistics
 ********************************************************************************
 * DESCRIPTION: This API queries Flow Cache statistics
 ********************************************************************************/
paReturn_t Pa_queryFcStats (Pa_Handle      handle,
                            paHandleFc_t   fcHandle,
                            uint16_t       doClear, 
                            paFcStats_t    *pFcStats)
{
  return (pa_API_UNSUPPORTED);
} /* Pa_queryFcStats */                            

/************************************************************************************
 * FUNCTION PURPOSE: Create a Tx checksum command
 ************************************************************************************
 * DESCRIPTION: The command is formatted
 ************************************************************************************/
static void pa_tx_chk (paTxChksum_t *chk, pasahoComChkCrc_t *ptx)
{
  PASAHO_SET_CMDID             (ptx, PASAHO_PAMOD_CMPT_CHKSUM);
  PASAHO_CHKCRC_SET_START      (ptx, chk->startOffset);
  PASAHO_CHKCRC_SET_LEN        (ptx, chk->lengthBytes);
  PASAHO_CHKCRC_SET_RESULT_OFF (ptx, chk->resultOffset);
  PASAHO_CHKCRC_SET_INITVAL    (ptx, chk->initialSum);

  if (chk->negative0)
     PASAHO_CHKCRC_SET_NEG0 (ptx, 1);
  else
     PASAHO_CHKCRC_SET_NEG0 (ptx, 0);

} 

/************************************************************************************
 * FUNCTION PURPOSE: Convert routing info to next Route command
 ************************************************************************************
 * DESCRIPTION: Convert routing info to next Route command
 ************************************************************************************/
static void pa_conv_routeInfo_nRoute (paRouteInfo_t *route, paCmdNextRoute_t *nrCmd, uint16_t nextCmd)
{
    nrCmd->ctrlBitfield = (nextCmd)?pa_NEXT_ROUTE_PROC_NEXT_CMD:0;
    nrCmd->dest = route->dest;
    nrCmd->pktType_emacCtrl = route->pktType_emacCtrl;
    nrCmd->flowId = route->flowId;
    nrCmd->queue = route->queue;
    nrCmd->swInfo0 = route->swInfo0;
    nrCmd->swInfo1 = route->swInfo1;
    
    if (route->mRouteIndex != pa_NO_MULTI_ROUTE) {
      nrCmd->ctrlBitfield |= pa_NEXT_ROUTE_PROC_MULTI_ROUTE;
      nrCmd->multiRouteIndex = (uint16_t)route->mRouteIndex;    
    }

} 

/************************************************************************************
 * FUNCTION PURPOSE: Format Tx routing / checksum information
 ************************************************************************************
 * DESCRIPTION: The command header is created for computing checksums and 
 *              routing packets through the transmit sub-system
 ************************************************************************************/
paReturn_t Pa_formatTxRoute  (paTxChksum_t  *chk0,
                              paTxChksum_t  *chk1,
                              paRouteInfo_t *route,
                              void*          cmdBuffer,
                              uint16_t      *cmdSize )
{

  paCmdInfo_t  cmdInfo[3];
  int          nCmd = 1;
  paCmdInfo_t* pCmdInfo = &cmdInfo[0];
  
  if ((route == NULL) || (cmdBuffer == NULL) || (cmdSize == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  memset(cmdInfo, 0, sizeof(cmdInfo));
  
  if (chk0) {
    pCmdInfo->cmd = pa_CMD_TX_CHECKSUM;
    pCmdInfo->params.chksum = *chk0;  
    nCmd++;
    pCmdInfo++;
  }
  
  if (chk1) {
    pCmdInfo->cmd = pa_CMD_TX_CHECKSUM;
    pCmdInfo->params.chksum = *chk1;  
    nCmd++;
    pCmdInfo++;
  }
  
  pCmdInfo->cmd = pa_CMD_NEXT_ROUTE;
  pa_conv_routeInfo_nRoute(route, &pCmdInfo->params.route, FALSE);
  
  return(Pa_formatTxCmd(nCmd, &cmdInfo[0], 0, cmdBuffer, cmdSize));
 
} /* Pa_formatTxRoute */

/************************************************************************************
 * FUNCTION PURPOSE: Format blind patch / Tx routing command
 ************************************************************************************
 * DESCRIPTION: The command header is created for routing (delayed) followed
 *              by a blind patch.
 ************************************************************************************/
paReturn_t Pa_formatRoutePatch (paRouteInfo_t *route,
                                paPatchInfo_t *patch,
                                void*          cmdBuffer,
                                uint16_t      *cmdSize)
{

  paCmdInfo_t  cmdInfo[2];
  
  if ((route == NULL) || (patch == NULL) || (cmdBuffer == NULL) || (cmdSize == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  memset(cmdInfo, 0, sizeof(cmdInfo));
  cmdInfo[1].cmd = pa_CMD_PATCH_DATA;
  cmdInfo[1].params.patch = *patch;
  
  cmdInfo[0].cmd = pa_CMD_NEXT_ROUTE;
  pa_conv_routeInfo_nRoute(route, &cmdInfo[0].params.route, TRUE);
  
  return(Pa_formatTxCmd(2, &cmdInfo[0], 0, cmdBuffer, cmdSize));

} /* Pa_formatRoutePatch */

/************************************************************************************
 * FUNCTION PURPOSE: Format Tx command set 
 ************************************************************************************
 * DESCRIPTION: This function formats a list of commands to be executed on the packets 
 *          to be transmitted over the network
 ************************************************************************************/
paReturn_t Pa_formatTxCmd (int             nCmd,
                           paCmdInfo_t    *cmdInfo,
                           uint16_t        offset,
                           void           *cmdBuffer,
                           uint16_t       *cmdSize)
{
  pasahoComChkCrc_t *ptx;
  pasahoNextRoute_t *nr;
  pasahoComBlindPatch_t *bpCmd;
  pasahoShortInfo_t *sInfo;
  pasahoCmdInfo_t   *paCmdInfo;
  uint16_t           pdest;
  uint16_t           cmdOffset = offset;
  uint8_t            buf[128], nrCmdSize; 
  uint8_t           *pCmd = (uint8_t *)cmdBuffer;
  int                index;  
  int                fAlignDetect = FALSE;      /* Indicate whetehr we need to perform  8-byte alignmnet check for command blocl forwarding to SASS */
  int                fNRPlusPatch = FALSE;      /* Indicate that the patch Command following the next commmand, therefore, its szie should be included */
  int                cmdBlockSize = 0; 
  int                bufAddr = (int)&buf[0];        
  
  if (((uint32_t)cmdBuffer & 0x3) || (offset & 0x3) || (offset > PAFRM_MAX_CMD_SET_SIZE)) 
    return (pa_ERR_CONFIG);
    
  if ((cmdInfo == NULL) || (cmdBuffer == NULL) || (cmdSize == NULL))
    return(pa_INVALID_INPUT_POINTER);

  if(nCmd <= 0)return (pa_ERR_CONFIG);  
  
  memset(buf, 0, 128);
  
  for (index = 0; index < nCmd; index++)  {
    switch (cmdInfo[index].cmd)
    {
    
      case pa_CMD_NONE:
        {
          /* Insert a dummy command */
          paCmdInfo = (pasahoCmdInfo_t *)(bufAddr + cmdOffset);
          
          PASAHO_SET_CMDID (paCmdInfo, PASAHO_PAMOD_DUMMY);
          cmdOffset += 4;
          
          if(fAlignDetect)
            cmdBlockSize+=4; 
        }
        break;         
      
      case pa_CMD_NEXT_ROUTE:
        {
          paCmdNextRoute_t* route = &cmdInfo[index].params.route;
          nr = (pasahoNextRoute_t *)(bufAddr + cmdOffset);
          nrCmdSize = sizeof(pasahoNextRoute_t) - sizeof(nr->word1);  
          
          if ((route->ctrlBitfield & pa_NEXT_ROUTE_PROC_NEXT_CMD) && (cmdInfo[index+1].cmd != pa_CMD_PATCH_DATA)) {
            /* Only support NextRoute plus Patch command */
            return(pa_ERR_CONFIG);
          }
          
          /*
           * Detect whether we need to insert a dummy command in front of the next route command 
           * to maintain 8-byte alignment requirement for SASS.
           */
          if (fAlignDetect)
          {
            cmdBlockSize += nrCmdSize;
            
            if ((route->dest == pa_DEST_SRIO) ||
                (route->ctrlBitfield & pa_NEXT_ROUTE_PROC_USR_STATS) ||
                ((route->dest != pa_DEST_SASS) && 
                 ((route->pktType_emacCtrl) || (route->ctrlBitfield & pa_NEXT_ROUTE_TX_L2_PADDING))))
            {
                cmdBlockSize += sizeof(nr->word1);
            }
          
            if (route->ctrlBitfield & pa_NEXT_ROUTE_PROC_NEXT_CMD)
            {
                paPatchInfo_t* patch = &cmdInfo[index+1].params.patch;
                
                cmdBlockSize += (patch->totalPatchSize + 4);
                fNRPlusPatch = TRUE;
            }
            
            if (cmdBlockSize % 8)
            {
                /* Insert dummy command */
                paCmdInfo = (pasahoCmdInfo_t *)(bufAddr + cmdOffset);
                PASAHO_SET_CMDID (paCmdInfo, PASAHO_PAMOD_DUMMY);
                cmdOffset += 4;
                nr = (pasahoNextRoute_t *)(bufAddr + cmdOffset);
            }
          
            cmdBlockSize = 0;
          } 
          
          /* Make sure the destination is valid */
          switch (route->dest)  {

            case pa_DEST_SASS:   
              pdest = PAFRM_DEST_PKTDMA;
              break;

            case pa_DEST_HOST:   
            case pa_DEST_EMAC:   
              pdest = (route->dest == pa_DEST_EMAC)?PAFRM_DEST_ETH:PAFRM_DEST_PKTDMA;
              if ((route->pktType_emacCtrl) || (route->ctrlBitfield & pa_NEXT_ROUTE_TX_L2_PADDING) || (route->dest == pa_DEST_EMAC))
              {
                uint8_t psFlags;
                PASAHO_SET_E (nr, 1);
                psFlags = (route->pktType_emacCtrl & pa_EMAC_CTRL_CRC_DISABLE)?
                          PAFRM_ETH_PS_FLAGS_DISABLE_CRC:0;
                psFlags |= ((route->pktType_emacCtrl & pa_EMAC_CTRL_PORT_MASK) << PAFRM_ETH_PS_FLAGS_PORT_SHIFT);  
                PASAHO_SET_PKTTYPE  (nr, psFlags); 
                if (route->ctrlBitfield & pa_NEXT_ROUTE_TX_L2_PADDING)
                {
                    PASAHO_SET_TX_PADDING (nr, 1);
                }    
                
                nrCmdSize += sizeof(nr->word1);
              } 
              break;
    
            case pa_DEST_SRIO:   
              pdest = PAFRM_DEST_SRIO;
              PASAHO_SET_E (nr, 1);
              PASAHO_SET_PKTTYPE  (nr, route->pktType_emacCtrl); 
              nrCmdSize += sizeof(nr->word1);
              break;

            default:  return (pa_ERR_CONFIG);
          }

          PASAHO_SET_CMDID (nr, PASAHO_PAMOD_NROUTE);
          PASAHO_SET_DEST  (nr, pdest);
          PASAHO_SET_FLOW  (nr, route->flowId);
          PASAHO_SET_QUEUE (nr, route->queue);
          if (route->ctrlBitfield & pa_NEXT_ROUTE_PROC_NEXT_CMD) {
            PASAHO_SET_N  (nr, 1);
          } 
          
          if (route->ctrlBitfield & pa_NEXT_ROUTE_PROC_USR_STATS) {
            PASAHO_SET_E (nr, 1);
            PASAHO_SET_TX_STATS(nr,1);
            PASAHO_SET_USR_STATS_INDEX(nr, route->statsIndex);
            nrCmdSize = sizeof(pasahoNextRoute_t);
          }
           
          nr->swInfo0 = route->swInfo0;
          nr->swInfo1 = route->swInfo1;
          
          cmdOffset += nrCmdSize;
        
        }
        break;
      
      case pa_CMD_CRC_OP:
        {
          paCmdCrcOp_t  *crc = &cmdInfo[index].params.crcOp;
          uint8_t ctrl;
          
          ptx    = (pasahoComChkCrc_t *)(bufAddr + cmdOffset);
          
          if (crc->ctrlBitfield & ( pa_CRC_OP_CRC_VALIDATE | pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER | pa_CRC_OP_CRC_FRAME_TYPE))
          {
            /* 
             * Support CRC calculation only in tx direction
             * length information should be available in the tx direction
             */
            return (pa_ERR_CONFIG); 
          
          }  
          
          ctrl = (crc->ctrlBitfield & pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD)?0:PAFRM_CRC_FLAG_CRC_OFFSET_VALID;
          
          PASAHO_SET_CMDID             (ptx, PASAHO_PAMOD_CMPT_CRC);
          PASAHO_CHKCRC_SET_START      (ptx, crc->startOffset);
          PASAHO_CHKCRC_SET_LEN        (ptx, crc->len);
          PASAHO_CHKCRC_SET_CTRL       (ptx, ctrl);
          PASAHO_CHKCRC_SET_RESULT_OFF (ptx, crc->crcOffset);
          
          cmdOffset += sizeof (pasahoComChkCrc_t);
          
          if(fAlignDetect)
            cmdBlockSize+=sizeof(pasahoComChkCrc_t); 
          
        }
        break;
      
      
      case pa_CMD_PATCH_DATA:
        {
          paPatchInfo_t* patch = &cmdInfo[index].params.patch;
          
          bpCmd = (pasahoComBlindPatch_t *) (bufAddr + cmdOffset); 
          
          /* Verify that the patch information is consisent */
          if ( (patch->nPatchBytes > pa_MAX_PATCH_BYTES)    ||
               (patch->totalPatchSize & 0x03))  
              return (pa_ERR_CONFIG);

          PASAHO_SET_CMDID                (bpCmd, PASAHO_PAMOD_PATCH);
          PASAHO_BPATCH_SET_PATCH_NBYTES  (bpCmd, patch->nPatchBytes);
          PASAHO_BPATCH_SET_PATCH_CMDSIZE (bpCmd, (patch->totalPatchSize + 4) >> 2);  /* Add the 4 byte command header */
          
          if(patch->ctrlBitfield & pa_PATCH_OP_INSERT)
            PASAHO_BPATCH_SET_OVERWRITE     (bpCmd,0);
          else
            PASAHO_BPATCH_SET_OVERWRITE     (bpCmd,1);
            
          if (patch->ctrlBitfield & pa_PATCH_OP_DELETE)
          {
            PASAHO_BPATCH_SET_OVERWRITE     (bpCmd,1);
            PASAHO_BPATCH_SET_DELETE        (bpCmd,1);
          }
            
            
          PASAHO_BPATCH_SET_OFFSET        (bpCmd, patch->offset);
          
          cmdOffset += 4;

          if (patch->patchData && patch->totalPatchSize)
          {
            int i, n;
            n = patch->nPatchBytes;
            for (i = 0; i < n; i++)
                PASAHO_BPATCH_SET_PATCH_BYTE(bpCmd, i, patch->patchData[i]);
                
            cmdOffset += patch->totalPatchSize;    
          }
          
          /* 
           * The patch command may be appended to the next route command and it is already included in the cmdBlockSize calcualtion
           */
          
          if(fAlignDetect && !fNRPlusPatch)
            cmdBlockSize+=(patch->totalPatchSize + 4); 
            
          fNRPlusPatch = FALSE;  
          
        }
        break;
      
      
      case pa_CMD_TX_CHECKSUM:
        {
          paTxChksum_t  *chk = &cmdInfo[index].params.chksum;
          
          ptx    = (pasahoComChkCrc_t *)(bufAddr + cmdOffset);
          pa_tx_chk (chk, ptx);
          cmdOffset += sizeof (pasahoComChkCrc_t);
          if(fAlignDetect)
            cmdBlockSize+=sizeof(pasahoComChkCrc_t); 
          
        }
        break;
      
      
      case pa_CMD_REPORT_TX_TIMESTAMP:
        {
          paCmdTxTimestamp_t       *txTs = &cmdInfo[index].params.txTs;
          pasahoReportTimestamp_t  *rtInfo = (pasahoReportTimestamp_t *)(bufAddr + cmdOffset);
          
          PASAHO_SET_CMDID (rtInfo, PASAHO_PAMOD_REPORT_TIMESTAMP);
          PASAHO_SET_REPORT_FLOW(rtInfo, (uint8_t)txTs->flowId);
          PASAHO_SET_REPORT_QUEUE(rtInfo, txTs->destQueue);
          rtInfo->swInfo0 = txTs->swInfo0;
          cmdOffset += sizeof(pasahoReportTimestamp_t);
          if(fAlignDetect)
            cmdBlockSize+=sizeof(pasahoReportTimestamp_t); 
            
        }
        break;
      
      case pa_CMD_IP_FRAGMENT:
        {
          paCmdIpFrag_t   *ipFrag = &cmdInfo[index].params.ipFrag;
          pasahoIpFrag_t  *ipFragInfo = (pasahoIpFrag_t *)(bufAddr + cmdOffset);
          
          PASAHO_SET_CMDID (ipFragInfo, PASAHO_PAMOD_IP_FRAGMENT);
          PASAHO_SET_SUB_CODE_IP_FRAG(ipFragInfo);
          PASAHO_SET_IP_OFFSET(ipFragInfo, (uint8_t)ipFrag->ipOffset);
          PASAHO_SET_MTU_SIZE(ipFragInfo, ipFrag->mtuSize);
          cmdOffset += sizeof(pasahoIpFrag_t);
          if(fAlignDetect)
            cmdBlockSize+=sizeof(pasahoIpFrag_t); 
        }
        break;
        
      case pa_CMD_PATCH_MSG_LEN:
        {
          paPatchMsgLenInfo_t  *patchMsgLen = &cmdInfo[index].params.patchMsgLen;
          pasahoPatchMsgLen_t  *fwPatchMsgLen = (pasahoPatchMsgLen_t *)(bufAddr + cmdOffset);
          
          PASAHO_SET_CMDID (fwPatchMsgLen, PASAHO_PAMOD_PATCH_MSG_LEN);
          PASAHO_SET_SUB_CODE_PATCH_MSG_LEN(fwPatchMsgLen);
          PASAHO_SET_MSGLEN_SIZE(fwPatchMsgLen, (patchMsgLen->msgLenSize != 2));
          PASAHO_SET_MSGLEN_OFFSET(fwPatchMsgLen, patchMsgLen->offset);
          PASAHO_SET_MSGLEN(fwPatchMsgLen, patchMsgLen->msgLen);
          cmdOffset += sizeof(pasahoPatchMsgLen_t);
          if(fAlignDetect)
            cmdBlockSize+=sizeof(pasahoPatchMsgLen_t); 
        }
        break;
        
      case pa_CMD_EMAC_CRC_VERIFY:
        {
          paCmdEmacCrcVerify_t   *emacCrc = &cmdInfo[index].params.emacCrc;
          pasahoEmacCrcVerify_t  *emacCrcInfo = (pasahoEmacCrcVerify_t *)(bufAddr + cmdOffset);
          
          if (nCmd == 1)
          {
            PASAHO_SET_CMDID (emacCrcInfo, PASAHO_PAMOD_EMAC_CRC_VERIFY);
            PASAHO_SET_SUB_CODE_EMAC_CRC_VERIFY(emacCrcInfo);
            PASAHO_SET_EMACPORT(emacCrcInfo, (uint8_t)emacCrc->emacPort);
            cmdOffset += sizeof(pasahoEmacCrcVerify_t);
            if(fAlignDetect)
                cmdBlockSize+=sizeof(pasahoEmacCrcVerify_t); 
          }  
          else
          {
              /* This command does not work with any other commands */
              return (pa_ERR_CONFIG);
          }
        }
        break;
      
      case pa_CMD_SA_PAYLOAD:
        {
          paPayloadInfo_t* pInfo = &cmdInfo[index].params.payload;
        
          sInfo = (pasahoShortInfo_t *)(bufAddr + cmdOffset);
          PASAHO_SET_CMDID (sInfo, PASAHO_SA_SHORT_INFO);
          PASAHO_SINFO_SET_PAYLOAD_OFFSET(sInfo, pInfo->offset);
          PASAHO_SINFO_SET_PAYLOAD_LENGTH(sInfo, pInfo->len);
          sInfo->word1 = pInfo->supData;
          
          cmdOffset += sizeof(pasahoShortInfo_t);
          
          /* This is the first and only command processed by SASS, turn on alignment detector */
          fAlignDetect = TRUE;
          cmdBlockSize = 0;
           
        }
        break;
      
      default:
        return (pa_ERR_CONFIG);
    }
    
    if((cmdOffset > 128) || (cmdOffset > *cmdSize))
      return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);
  }
  
  memcpy(pCmd + offset, &buf[offset], cmdOffset - offset);
  *cmdSize = cmdOffset;
  
  return (pa_OK);
  
} /* Pa_formatTxCmd */ 

/****************************************************************************************
 * FUNCTION PURPOSE: Enable a PDSP
 ****************************************************************************************
 * DESCRIPTION:  The PDSP is put into the run state if it is currently in the reset state.
 *               If taken out of reset the function waits for the PDSP to indicate 
 *               that it is ready. 
 *
 *               Return value:
 *                  PA_PDSP_ALREADY_ACTIVE - the PDSP was already out of reset
 *                  PA_PDSP_RESET_RELEASED - the PDSP was in reset and successfully taken out of reset
 *                  PA_PDSP_NO_RESTART - the PDSP was taken out of reset but timed out on the handshake
 ****************************************************************************************/

#define PA_PDSP_ALREADY_ACTIVE      0
#define PA_PDSP_RESET_RELEASED      1
#define PA_PDSP_NO_RESTART          2

static int pa_pdsp_run (volatile CSL_Pa_ssPdsp_ctlstatRegs *pdsp, volatile CSL_Pa_ssMailboxRegs *mbox)
{
  int i;

  /* Check for enabled PDSP */
  if ((pdsp->PDSP_CONTROL & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK) == CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)
    return (PA_PDSP_ALREADY_ACTIVE);

  /* Clear the mailbox */
  mbox->MBOX_SLOT[0] = 0;

  /* Enable the PDSP */
  pdsp->PDSP_CONTROL |= (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK |
                         CSL_PA_SS_PDSP_CONTROL_SOFT_RST_N_MASK);

  /* Wait for the mailbox to become non-zero */
  for (i = 0; i < PA_MAX_PDSP_ENABLE_LOOP_COUNT; i++)  {
    if (mbox->MBOX_SLOT[0] != 0)  {
      return (PA_PDSP_RESET_RELEASED);
    }
  }

  return (PA_PDSP_NO_RESTART);

} /* pa_pdsp_run */

/*****************************************************************************************
 * FUNCTION PURPOSE: Set the state of the Sub-System
 *****************************************************************************************
 * DESCRIPTION: The Sub-System state can be set or queried.
 *****************************************************************************************/
paSSstate_t Pa_resetControl (Pa_Handle iHandle, paSSstate_t newState)
{

  int    i;
  paInst_t *paInst                          = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);  
  CSL_Pa_ssRegs *passRegs;
  paSSstate_t  ret = pa_STATE_INVALID_REQUEST;
  uint32_t     mtCsKey;  

  /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);

  /* Check permission */
  if ( (newState == pa_STATE_RESET) ||
  	   (newState == pa_STATE_ENABLE) )
  {
	  if (paLObj.cfg.rmServiceHandle) {
  		  int32_t fw = 0;

		  if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmFirmware, &fw, NULL)) {
			return pa_RESOURCE_USE_DENIED;

		  }
		  if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_INIT, rmFirmware, &fw, NULL)) {
			/* Use but don't init - Check firmware revision */
			  
			/* if check reveals different revision - return PA_FIRMWARE_REVISION_DIFFERENCE; */
		  }    
		  /* we use RM only for permission check, so freeing up immediately */
		  
		  /* Release firmware "use" */	  
		  if (!pa_rmService (Rm_service_RESOURCE_FREE, rmFirmware, &fw, NULL)) {
			return pa_RESOURCE_USE_DENIED;
		  }   
		  
		  /* Release firmware "init" */ 	
		  if (!pa_rmService (Rm_service_RESOURCE_FREE, rmFirmware, &fw, NULL)) {
			return pa_RESOURCE_INIT_DENIED;
		  } 
	  }
  }
	  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  passRegs = (CSL_Pa_ssRegs *)paLObj.cfg.baseAddr;
  
  if (newState == pa_STATE_RESET)  {

    /* Put each of the PDSPs into reset (PC = 0)*/
    for (i = 0; i < 6; i++)  {
      passRegs->PDSP_CTLSTAT[i].PDSP_CONTROL = 0;
    }

    /* Reset packet ID */
    passRegs->PKT_ID.PKT_ID_SOFT_RESET = 1;

    /* Reset LUT2 */
    passRegs->LUT2.LUT2_SOFT_RESET = 1;

    /* Reset statistics */
    passRegs->STATS.STATS_SOFT_RESET = 1;

    /* Reset timers */
    for (i = 0; i < 6; i++)
      passRegs->PDSP_TIMER[i].TIMER_CNTRL_REG = 0;

    ret = pa_STATE_RESET;

  }

  else if (newState == pa_STATE_ENABLE)  {
    uint16_t         doGlobalReset = TRUE;
    int          res;

    ret = pa_STATE_ENABLE;
     
    /* Do nothing if a pdsp is already out of reset. If any PDSPs are out of reset
     * a global init is not performed */
    for (i = 0; i < 6; i++)  {

      res = pa_pdsp_run (&(passRegs->PDSP_CTLSTAT[i]), &(passRegs->MAILBOX[i]));
      if (res == PA_PDSP_ALREADY_ACTIVE)
        doGlobalReset = FALSE;
    
      if (res == PA_PDSP_NO_RESTART)  {
        ret = pa_STATE_ENABLE_FAILED;
        doGlobalReset = FALSE;
      }

    }

    /* If global reset is required any PDSP can do it */
    if (doGlobalReset)  {

      passRegs->MAILBOX[0].MBOX_SLOT[1] = 1;   /* Tell PDSP0 to do global init */
      passRegs->MAILBOX[0].MBOX_SLOT[0] = 0;   /* Let PDSP0 go */

      while (passRegs->MAILBOX[0].MBOX_SLOT[1] != 0);

      for (i = 1; i < 6; i++)
        passRegs->MAILBOX[i].MBOX_SLOT[0] = 0;   /* Let PDSP go */

    }  else  {

      for (i = 0; i < 6; i++)
        passRegs->MAILBOX[i].MBOX_SLOT[0] = 0;   /* Let PDSP go */

    }

  }

  else if (newState == pa_STATE_QUERY)  {

    uint16_t enable  = FALSE;
    uint16_t reset   = FALSE;

    for (i = 0; i < 6; i++)  {
      if ( (passRegs->PDSP_CTLSTAT[i].PDSP_CONTROL & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK) == 
                     (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK))
        enable = TRUE;
      else
        reset = TRUE;
    }

    if ( (reset == TRUE) && (enable == TRUE) )
      ret = pa_STATE_INCONSISTENT;
    else if (reset == TRUE)
      ret = pa_STATE_RESET;
    else
      ret = pa_STATE_ENABLE;

  }
  
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);

  return (ret);

} /* Pa_resetControl */

/*****************************************************************************************
 * FUNCTION PURPOSE: Configures the PA timer which is used to generate 32-bit timestamp
 *****************************************************************************************
 * DESCRIPTION: This function is used to configure the 16-bit timer reserved for 32-bit 
 * system timestamp.
 *****************************************************************************************/
paReturn_t Pa_configTimestamp (Pa_Handle             iHandle,
                               paTimestampConfig_t  *cfgInfo)
{
   
  paInst_t 		*paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  CSL_Pa_ssRegs *passRegs;
  uint32_t      mtCsKey;  
  
  /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);

  if (cfgInfo == NULL)
      return (pa_ERR_CONFIG); 
      
  if (cfgInfo->enable) {
    if ( (cfgInfo->factor < pa_TIMESTAMP_SCALER_FACTOR_2) || 
        (cfgInfo->factor > pa_TIMESTAMP_SCALER_FACTOR_8192)) {
        return (pa_ERR_CONFIG);
    }
  }      
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
      
  passRegs = (CSL_Pa_ssRegs *)paLObj.cfg.baseAddr;
  if (cfgInfo->enable) {
    
    passRegs->PDSP_TIMER[0].TIMER_LOAD_REG  = 0xFFFF;
    passRegs->PDSP_TIMER[0].TIMER_CNTRL_REG = CSL_PA_SS_TIMER_CNTRL_REG_GO_MASK       |  
                                              CSL_PA_SS_TIMER_CNTRL_REG_MODE_MASK     |
                                              (cfgInfo->factor << CSL_PA_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT)    |
                                              CSL_PA_SS_TIMER_CNTRL_REG_PSE_MASK;
  } 
  else  {
    passRegs->PDSP_TIMER[0].TIMER_LOAD_REG = 0;
    passRegs->PDSP_TIMER[0].TIMER_CNTRL_REG = 0;
  }
  
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);
    
  return (pa_OK);

} /* Pa_configTimestamp */

/*****************************************************************************************
 * FUNCTION PURPOSE: Return the 48-bit system timestamp
 *****************************************************************************************
 * DESCRIPTION: This function is used to inquery the 48-bit system timestamp.
 *****************************************************************************************/
paReturn_t Pa_getTimestamp  (Pa_Handle            iHandle, 
                             paTimestamp_t        *pTimestamp)
{
   
  paInst_t 			 *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  CSL_Pa_ssRegs *passRegs;
  uint32_t mtCsKey;

  /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
  
  if (pTimestamp == NULL)
    return(pa_INVALID_INPUT_POINTER);
      
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  
  passRegs = (CSL_Pa_ssRegs *)paLObj.cfg.baseAddr;
  pTimestamp->lo = 0x0000FFFF - passRegs->PDSP_TIMER[0].TIMER_VALUE_REG;
  pTimestamp->hi = passRegs->PDSP_SRAM[PAFRM_SYS_TIMESTAMP_SRAM_INDEX].PDSP_RAM[PAFRM_SYS_TIMESTAMP_OFFSET];
  pTimestamp->hi_hi = (uint16_t) passRegs->PDSP_SRAM[PAFRM_SYS_TIMESTAMP_SRAM_INDEX].PDSP_RAM[PAFRM_SYS_TIMESTAMP_OFFSET_HI];
      
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);
  
  return (pa_OK);

} /* Pa_getTimestamp */

/* Moved the constant defintions outside this file *
 * This helps to generate the bin files required for
 * linux */
extern  const uint32_t pap_pdsp_const_reg_map[6][4];

#define PA_PDSP_CONST_REG_INDEX_C25_C24     0
#define PA_PDSP_CONST_REG_INDEX_C27_C26     1
#define PA_PDSP_CONST_REG_INDEX_C29_C28     2
#define PA_PDSP_CONST_REG_INDEX_C31_C30     3

/***********************************************************************************************
 * FUNCTION PURPOSE: Download a PDSP image
 ***********************************************************************************************
 * DESCRIPTION: A PDSP image is downloaded. The PDSP remains in reset.
 ***********************************************************************************************/
paReturn_t Pa_downloadImage (Pa_Handle iHandle, int modId, void* image, int sizeBytes)
{

  paInst_t              *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  CSL_Pa_ssRegs *passRegs;
  paReturn_t ret = pa_OK;
  uint32_t mtCsKey;
  volatile uint32_t *src, *dst;
  uint32_t i;

  /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);

  if(image == NULL)
    return(pa_INVALID_INPUT_POINTER);

  /* Verify the specified PDSP is valid */
  if ((modId < 0)  || (modId >= 6))
    return (pa_ERR_CONFIG);

  /* Verify the image size is in range */
  if ((sizeBytes < 0)  || (sizeBytes >= 8192))
    return (pa_ERR_CONFIG);
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  if (paLObj.cfg.rmServiceHandle) {
    int32_t fw = 0;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmFirmware, &fw, NULL)) {
      ret = pa_RESOURCE_USE_DENIED;
      goto Pa_downloadImage_end;
    }
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_INIT, rmFirmware, &fw, NULL)) {
      /* Use but don't init - Check firmware revision */
        
      /* if check reveals different revision - return PA_FIRMWARE_REVISION_DIFFERENCE; */
    }    
    /* we use RM only for permission check, so freeing up immediately */

    /* Release firmware "use" */
    if (!pa_rmService (Rm_service_RESOURCE_FREE, rmFirmware, &fw, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }	

    /* Release firmware "init" */	  
    if (!pa_rmService (Rm_service_RESOURCE_FREE, rmFirmware, &fw, NULL)) {
      return pa_RESOURCE_INIT_DENIED;
    }   
	
  }
 
  passRegs = (CSL_Pa_ssRegs *)paLObj.cfg.baseAddr;

  /* Make sure the PDSP is disabled */
  if ( (passRegs->PDSP_CTLSTAT[modId].PDSP_CONTROL & CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)  ==
                                     (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK)   )
  {
    ret = pa_SYSTEM_STATE_INVALID;
    goto Pa_downloadImage_end;
  }  

  /* Copy the image */
  src = (uint32_t *) image;
  dst = (uint32_t *) passRegs->PDSP_IRAM[modId].PDSP_RAM;
  for ( i = 0; i < (sizeBytes/4); i++)
  {
      dst[i] = src[i];
  }
  
  /* Initialize the programmable constant registers C24-31 */
  passRegs->PDSP_CTLSTAT[modId].PDSP_CONSTANT_TABLE_BLOCK_INDEX_0 = pap_pdsp_const_reg_map[modId][PA_PDSP_CONST_REG_INDEX_C25_C24];
  passRegs->PDSP_CTLSTAT[modId].PDSP_CONSTANT_TABLE_BLOCK_INDEX_1 = pap_pdsp_const_reg_map[modId][PA_PDSP_CONST_REG_INDEX_C27_C26];
  passRegs->PDSP_CTLSTAT[modId].PDSP_CONSTANT_TABLE_PROG_PTR_0    = pap_pdsp_const_reg_map[modId][PA_PDSP_CONST_REG_INDEX_C29_C28];
  passRegs->PDSP_CTLSTAT[modId].PDSP_CONSTANT_TABLE_PROG_PTR_1    = pap_pdsp_const_reg_map[modId][PA_PDSP_CONST_REG_INDEX_C31_C30];


Pa_downloadImage_end:
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  
  
  return (ret);

} /* Pa_downloadImage */

/***********************************************************************************************
 * FUNCTION PURPOSE: Inquire PDSP Version Number
 ***********************************************************************************************
 * DESCRIPTION: Extract the PA PDSP  version number.
 ***********************************************************************************************/
paReturn_t Pa_getPDSPVersion (Pa_Handle iHandle, int modId, uint32_t *pVersion)
{
  paInst_t			  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  CSL_Pa_ssRegs *passRegs;
  uint32_t mtCsKey;      

    /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
  
  if(pVersion == NULL)
    return(pa_INVALID_INPUT_POINTER);
  
  /* Verify the specified PDSP is valid */
  if ((modId < 0)  || (modId >= 6))
    return (pa_ERR_CONFIG);
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  passRegs = (CSL_Pa_ssRegs *)paLObj.cfg.baseAddr;
  *pVersion = passRegs->PDSP_SRAM[PAFRM_PDSP_VERSION_SRAM_INDEX].PDSP_RAM[PAFRM_PDSP_VERSION_OFFSET(modId)];
  
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  
  
  return (pa_OK);

} /* Pa_getPDSPVersion */


/***********************************************************************************************
 * FUNCTION PURPOSE: Inquire Version Number
 ***********************************************************************************************
 * DESCRIPTION: Return the PA LLD version number.
 ***********************************************************************************************/
uint32_t Pa_getVersion (void)
{
    return PA_LLD_VERSION_ID;
} /* Pa_getVersion */

/***********************************************************************************************
 * FUNCTION PURPOSE: Inquire Version String
 ***********************************************************************************************
 * DESCRIPTION: Return the PA LLD version string.
 ***********************************************************************************************/
const char* Pa_getVersionStr (void)
{
    return paLldVersionStr;
} /* Pa_getVersionStr */

/***********************************************************************************************
 * FUNCTION PURPOSE: Get debug information snap shot from PA
 ***********************************************************************************************
 * DESCRIPTION: Return the PA internal debug information.
 ***********************************************************************************************/
paReturn_t Pa_getDbgpInfo(Pa_Handle iHandle, paSnapShotDebugInfo_t *dbgInfo)
{
  CSL_Pa_ssRegs         *passRegs;
  paReturn_t             ret = pa_OK;
  uint32_t               *addr;
  uint32_t               bitmap;

  /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);

  if(dbgInfo == NULL)
    return(pa_INVALID_INPUT_POINTER);

  passRegs = (CSL_Pa_ssRegs *)paLObj.cfg.baseAddr;

  /* Read the reassembly debug snap shot from firmware and report to the caller */
  switch (dbgInfo->debugInfoType)
  {
      case pa_DBG_INFO_TYPE_REASSEMBLY_ENABLE:
            addr	   = (uint32_t *)&passRegs->PDSP_SRAM[PAFRM_OUT_REASSEM_CTRL_BLK_INDEX].PDSP_RAM[PAFRM_OUT_REASSEM_CTRL_BLK_OFFSET];
            bitmap = pa_read_reassem_control_blk(addr, &dbgInfo->u.reassemContext.outer);

            /* Set the traffic flow array to inactive state */
            memset (&dbgInfo->u.reassemContext.outer.traffic_flow[0], 0xFF, sizeof (pa_ReassemblyFlow_t) * 32 );
            addr       = (uint32_t *)&passRegs->PDSP_SRAM[PAFRM_OUT_REASSEM_TRAFFIC_FLOW_INDEX].PDSP_RAM[PAFRM_OUT_REASSEM_TRAFFIC_FLOW_OFFSET];
            pa_read_reassem_traffic_flow (addr, &dbgInfo->u.reassemContext.outer.traffic_flow[0], bitmap);

            addr       = (uint32_t *)&passRegs->PDSP_SRAM[PAFRM_IN_REASSEM_CTRL_BLK_INDEX].PDSP_RAM[PAFRM_IN_REASSEM_CTRL_BLK_OFFSET];
            bitmap = pa_read_reassem_control_blk(addr, &dbgInfo->u.reassemContext.inner);

            memset (&dbgInfo->u.reassemContext.inner.traffic_flow[0], 0xFF, sizeof (pa_ReassemblyFlow_t) * 32 );
            addr       = (uint32_t *)&passRegs->PDSP_SRAM[PAFRM_IN_REASSEM_TRAFFIC_FLOW_INDEX].PDSP_RAM[PAFRM_IN_REASSEM_TRAFFIC_FLOW_OFFSET];
            pa_read_reassem_traffic_flow (addr, &dbgInfo->u.reassemContext.inner.traffic_flow[0], bitmap);

            break;
      default:
            ret = pa_ERR_CONFIG;
            break;
  }

  return (ret);

}
/***********************************************************************************************
 * FUNCTION PURPOSE: Get virtual link ID information 
 ***********************************************************************************************
 * DESCRIPTION: Return the PA virtual link ID information.
 ***********************************************************************************************/

paReturn_t Pa_getVirtualLinkId(Pa_Handle       iHandle,
                               paLnkHandle_t   vlinkHdl,
                               int8_t* lnkId)
{
  paVirtualLnk_t    *vlnk;  
  paReturn_t        ret = pa_INVALID_INPUT_HANDLE;
 
  vlnk = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, vlinkHdl);  
  
  Pa_osalBeginMemAccess ((void *) vlnk, sizeof (paVirtualLnk_t));
 
  if (vlnk->status == PA_TBL_STAT_ACTIVE)
  {
       *lnkId = vlnk->tableIdx;
        ret = pa_OK;
  }
  Pa_osalEndMemAccess ((void *) vlnk, sizeof (paVirtualLnk_t));
 
return (ret);
}

/* Nothing past this point */
