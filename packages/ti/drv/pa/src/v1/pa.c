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
#endif

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

/* PDSP/Cluster Mapping */
const paPdspMap_t pa_pdsp_map[PASS_NUM_PDSPS] = {
    {(uint8_t) PASS_CLUSTER_INGRESS0, 0, 0x3f04},
    {(uint8_t) PASS_CLUSTER_INGRESS0, 1, 0x3f04},
    {(uint8_t) PASS_CLUSTER_INGRESS1, 0, 0x3f04},
    {(uint8_t) PASS_CLUSTER_INGRESS1, 1, 0x3f04},
    {(uint8_t) PASS_CLUSTER_INGRESS2, 0, 0x1f04},
    {(uint8_t) PASS_CLUSTER_INGRESS3, 0, 0x1f04},
    {(uint8_t) PASS_CLUSTER_INGRESS4, 0, 0x3f04},
    {(uint8_t) PASS_CLUSTER_INGRESS4, 1, 0x3f04},
    {(uint8_t) PASS_CLUSTER_POST, 0, 0x3f04},
    {(uint8_t) PASS_CLUSTER_POST, 1, 0x3f04},
    {(uint8_t) PASS_CLUSTER_EGRESS0, 0, 0x1f04},
    {(uint8_t) PASS_CLUSTER_EGRESS0, 1, 0x1f04},
    {(uint8_t) PASS_CLUSTER_EGRESS0, 2, 0x1f04},
    {(uint8_t) PASS_CLUSTER_EGRESS1, 0, 0x0f04},
    {(uint8_t) PASS_CLUSTER_EGRESS2, 0, 0x0f04}
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

/*************************************************************************
 * FUNCTION PURPOSE: Find IP Subnet size
 ************************************************************************* 
 * DESCRIPTION: Returns number of 1 at the IP subnet mask 
 *              
 * Assumption: valid subnet mask array: there are continuos 1 from MSB until
 *             the first 0 is found
 *************************************************************************/
static uint8_t pa_find_ip_subnet_size (uint8_t mask[],  int size)
{
  uint8_t subnetSize = size * 8;
  uint8_t byte;
  int i = size -1;  
  
  /* find the first non-zero entry */
  while ((mask[i] == 0 ) && (i >= 0))
  {
   subnetSize -= 8;
   i--; 
  }
  
  if (i >= 0)
  {
    byte = mask[i];
    while (!(byte & 1))
    {
        subnetSize--;
        byte >>= 1;
        if(!byte)break;
    } 
  }
  
  return(subnetSize);

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
 * FUNCTION PURPOSE: Convert Flow Cache Routing Info
 ************************************************************************* 
 * DESCRIPTION: Convert the flow cache routing information into the
 *              firmware forward information
 * Returns FALSE if error occurs
 *************************************************************************/
static paReturn_t pa_conv_fc_routing_info (pafrmForward_t *fwdInfo, paEfOpInfo_t *efInfo)
{
  if(efInfo == NULL)
    return(pa_ERR_CONFIG);

  fwdInfo->forwardType       = PAFRM_FORWARD_TYPE_EFLOW;
  
  if (efInfo->ctrlFlags & pa_EF_OP_CONTROL_FLAG_FC_LOOKUP)
  {
    /* Trigger Flow cache operation */
    fwdInfo->u.ef.ctrlFlags = PAFRM_EF_CTRL_FC_LOOKUP;
  }
  else
  {
    /* Use Egress Flow records directly */
    fwdInfo->u.ef.validBitMap  = (uint8_t)efInfo->validBitMap;
    fwdInfo->u.ef.lvl1RecIndex = (uint8_t)efInfo->lvl1Index;
    fwdInfo->u.ef.lvl2RecIndex = (uint8_t)efInfo->lvl2Index;
    fwdInfo->u.ef.lvl3RecIndex = (uint8_t)efInfo->lvl3Index;
    fwdInfo->u.ef.lvl4RecIndex = (uint8_t)efInfo->lvl4Index;
  }  
  
  return (pa_OK);
} /* pa_conv_fc_routing_info */

/*************************************************************************
 * FUNCTION PURPOSE: Convert ipInfo 
 ************************************************************************* 
 * DESCRIPTION: Convert old style of IP information to new data structure
 *************************************************************************/
static void pa_convert_ipInfo_to_ipInfo2(paIpInfo_t *pIpInfo, paIpInfo2_t *pIpInfo2)
{
    memset(pIpInfo2, 0, sizeof(paIpInfo2_t));
    pIpInfo2->ipType = pIpInfo->ipType;
    
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
static paReturn_t pa_conv_routing_info2 (paInst_t *paInst, pafrmForward_t *fwdInfo, paRouteInfo2_t *routeInfo, int cmdDest, uint16_t failRoute, uint16_t destPdsp, uint8_t paFlags, int routingClass)
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
                    PAFRM_ETH_PS_FLAGS_CRC_PRESENT:0;
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
           
        fwdInfo->u.host.ctrlBitMap   |= (PAFRM_ROUTING_IF_DEST_SELECT_ENABLE |
                                         PAFRM_ROUTING_FLOW_EQOS_IF_ENABLE);
        routingClass = pa_QUEUE_BOUNCE_ROUTING_CLASS_QoS;
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
    
    if (routeInfo->validBitMap & pa_ROUTE_INFO_VALID_CTRLBITMAP)
    {
        if ((routeInfo->ctrlBitMap & pa_ROUTE_INFO_L2_PKT_CAPTURE) &&
           (routeInfo->dest == pa_DEST_CONTINUE_PARSE_LUT1))
        {
            fwdInfo->u.pa.context = SWIZ(routeInfo->swInfo0);
            paFlags |= PAFRM_PA_L2_CAPTURE;
        }
    }
             
    fwdInfo->forwardType = PAFRM_FORWARD_TYPE_PA;
    fwdInfo->u.pa.customType = (uint8_t)routeInfo->customType;
    fwdInfo->u.pa.customIdx = SWIZ(routeInfo->customIndex); 
    fwdInfo->u.pa.flags = SWIZ(paFlags);

    if (routeInfo->dest == pa_DEST_CONTINUE_PARSE_LUT2) {
        
       fwdInfo->u.pa.paDest = PAFRM_DEST_INGRESS4; 
        
    }
    else {
      /*
       * cmdDest is provided by calling function 
       * There is no need to check error case 
       */
      if (cmdDest == pa_CMD_TX_DEST_0)
      {
        /* Layer 2 entry */
        fwdInfo->u.pa.paDest = PAFRM_DEST_INGRESS1;
      }
      else if (cmdDest == pa_CMD_TX_DEST_1) 
      {
        fwdInfo->u.pa.paDest = (destPdsp == 0)?PAFRM_DEST_INGRESS1:PAFRM_DEST_INGRESS3;
        if(routeInfo->customType == pa_CUSTOM_TYPE_LUT1)
            fwdInfo->u.pa.paDest = PAFRM_DEST_INGRESS3; 
        
      }
      else if (cmdDest == pa_CMD_TX_DEST_3) 
      {
        fwdInfo->u.pa.paDest = PAFRM_DEST_INGRESS4;
      }
      else
      {
        return(pa_ERR_CONFIG); 
      }
    }
    
    fcmdSetNotSupport = TRUE;

  } else if (routeInfo->dest == pa_DEST_CASCADED_FORWARDING_LUT1) {
    fwdInfo->forwardType = PAFRM_FORWARD_TYPE_PA;
    fwdInfo->u.pa.paDest = (cmdDest == pa_CMD_TX_DEST_0)?PAFRM_DEST_INGRESS1:PAFRM_DEST_INGRESS4;
    if (routeInfo->validBitMap & pa_ROUTE_INFO_VALID_CTRLBITMAP)
    {
        if (routeInfo->ctrlBitMap & pa_ROUTE_INFO_L2_PKT_CAPTURE) 
        {
            fwdInfo->u.pa.context = SWIZ(routeInfo->swInfo0);
            paFlags |= PAFRM_PA_L2_CAPTURE;
        }
    }
    
    paFlags |= PAFRM_CASCADED_FORWARDING;
    fwdInfo->u.pa.flags = SWIZ(paFlags);
    fcmdSetNotSupport = TRUE;
  
  }  else if (routeInfo->dest == pa_DEST_SASS)  {
    fwdInfo->forwardType        = PAFRM_FORWARD_TYPE_SA;
    fwdInfo->u.sa.swInfo0       = SWIZ(routeInfo->swInfo0);
    fwdInfo->u.sa.swInfo1       = SWIZ(routeInfo->swInfo1);
    
  }  else if (routeInfo->dest == pa_DEST_SASS_LOC_DMA)  {
    fwdInfo->forwardType        = PAFRM_FORWARD_TYPE_SA | PAFRM_FORWARD_CONTROL_USE_LOC_DMA;
    fwdInfo->u.sa.swInfo0       = SWIZ(routeInfo->swInfo0);
    fwdInfo->u.sa.swInfo1       = SWIZ(routeInfo->swInfo1);
    
  }  else if ((routeInfo->dest == pa_DEST_RES_1) || (routeInfo->dest == pa_DEST_RES_2)) {
    fwdInfo->forwardType        = PAFRM_FORWARD_TYPE_SA_DIRECT;
    fwdInfo->flowId             = (routeInfo->dest == pa_DEST_RES_1)?PAFRM_DEST_ACE0:PAFRM_DEST_ACE1; 
    fwdInfo->u.sa.swInfo0       = SWIZ(routeInfo->swInfo0);
    fwdInfo->u.sa.swInfo1       = SWIZ(routeInfo->swInfo1);
    
  }  else if (routeInfo->dest == pa_DEST_SRIO)  {
    fwdInfo->forwardType        = PAFRM_FORWARD_TYPE_SRIO;
    fwdInfo->u.srio.psInfo0     = SWIZ(routeInfo->swInfo0);
    fwdInfo->u.srio.psInfo1     = SWIZ(routeInfo->swInfo1);
    fwdInfo->u.srio.pktType     = SWIZ(routeInfo->pktType_emacCtrl);
    pCmd = NULL;
  
  }  else if (routeInfo->dest == pa_DEST_EFLOW)  {
  
    paReturn_t ret;
  
    ret = pa_conv_fc_routing_info(fwdInfo, routeInfo->efOpInfo);
    if(ret != pa_OK)
        return(ret);
        
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
 * FUNCTION PURPOSE: Convert Routing Info 
 ************************************************************************* 
 * DESCRIPTION: Convert the application routing information into the
 *              firmware forward information
 * Returns FALSE if error occurs
 *************************************************************************/
static paReturn_t pa_conv_routing_info (paInst_t *paInst, pafrmForward_t *fwdInfo, paRouteInfo_t *routeInfo, int cmdDest, uint16_t failRoute, uint16_t destPdsp, int routingClass)
{
  paRouteInfo2_t routeInfo2; 
  
  pa_convert_routeInfo_to_routeInfo2(routeInfo, &routeInfo2);
  
  return(pa_conv_routing_info2(paInst, fwdInfo, &routeInfo2, cmdDest, failRoute, destPdsp, 0, routingClass));
    
} /* pa_conv_routing_info */

/***************************************************************************
 * FUNCTION PURPOSE: setup the firmware command for emac port configuration
 ***************************************************************************
 * DESCRIPTION: setup the firmware command for emac port configuration
 ***************************************************************************/
static paReturn_t pa_control_get_cmd_size(paCtrlInfo_t *ctrl, uint16_t *pCmdSize)
{
    paReturn_t ret = pa_OK;

    switch (ctrl->code)
    {
        case pa_CONTROL_SYS_CONFIG:
            *pCmdSize = sizeof(pafrmCommand_t) + sizeof(pafrmCommandConfigPa_t) - sizeof(uint32_t);
            break;
            
        case pa_CONTROL_802_1ag_CONFIG:
            *pCmdSize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t) 
                        + sizeof(pafrm802p1agDet_t);
            break;
            
        case pa_CONTROL_IPSEC_NAT_T_CONFIG:
            *pCmdSize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t) 
                        + sizeof(pafrmIpsecNatTDet_t);
            break;
            
        case pa_CONTROL_GTPU_CONFIG:
            *pCmdSize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t) 
                        + sizeof(pafrmGtpuCfg_t);
            break;
            
        case pa_CONTROL_EMAC_PORT_CONFIG:
        {
            paEmacPortConfig_t  *portCfg = &ctrl->params.emacPortCfg;
            uint8_t             numEntries = portCfg->numEntries;
            
            if (portCfg->cfgType == pa_EMAC_PORT_CFG_DEFAULT_ROUTE)
            {
                *pCmdSize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t)
                            + offsetof(pafrmDefRouteCfg_t, routeCfg) + sizeof(pafrmDefRouteInfo_t)*numEntries;
            }
            else if (portCfg->cfgType == pa_EMAC_PORT_CFG_EQoS_MODE)
            {
                *pCmdSize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t)
                            + offsetof(pafrmEQosModeConfig_t, eQoSCfg) + sizeof(pafrmEQosModeIf_t)*numEntries;
            }
            else
            {
                /* pa_EMAC_PORT_CFG_PKT_CAPTURE or pa_EMAC_PORT_CFG_MIRROR */
                *pCmdSize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t)
                            + offsetof(pafrmPktCapCfg_t, pktCapCfg) + sizeof(pafrmPktCapIfCfg_t)*numEntries;
            }
            break;
        }    
        case pa_CONTROL_RA_CONFIG:
            *pCmdSize = 0;
            break;

        case pa_CONTROL_TIME_OFFSET_CONFIG:
              *pCmdSize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t) 
                        + sizeof(pafrmSetTOffsetCfg_t);
              break;

        default:
            ret = pa_ERR_CONFIG;
            break;
    }
    
    return(ret);    
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
  if (numEntries > pa_MAX_NUM_EMAC_PORT_CONFIG_ENTRIES_GEN2)
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
        if (capturePort >  pa_EMAC_PORT_7)
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
        pktCapfrmCfg->pktCapCfg[i].ctrlBitmap    = SWIZ(ctrlBitMap);
        pktCapfrmCfg->pktCapCfg[i].capturePort   = SWIZ(capturePort);
        pktCapfrmCfg->pktCapCfg[i].queue         = SWIZ(queue);
        pktCapfrmCfg->pktCapCfg[i].emacport_flow = SWIZ(flow);
        pktCapfrmCfg->pktCapCfg[i].context       = SWIZ(swInfo0);        
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
            (port >  pa_EMAC_PORT_7))
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
        pktCapfrmCfg->pktCapCfg[i].ctrlBitmap    = ctrlBitMap;
        pktCapfrmCfg->pktCapCfg[i].capturePort   = port;
        pktCapfrmCfg->pktCapCfg[i].emacport_flow = mirrorPort;
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
            (port >  pa_EMAC_PORT_7))
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
          retCode = pa_conv_routing_info2 (paInst, &pDefRoutefrmCfg->routeCfg[i].dRoute[pa_DROUTE_MULTICAST], &pDefRouteCfg[i].dRouteInfo[pa_DROUTE_MULTICAST], pa_CMD_TX_DEST_0, TRUE, 0, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
          /* Error checking */
          if (retCode != pa_OK)
            return (retCode);
        }
        /* Store broad cast default route information if enabled */
        if (ctrlBitMap & pa_EMAC_IF_DEFAULT_ROUTE_BC_ENABLE)
        {
          retCode = pa_conv_routing_info2 (paInst, &pDefRoutefrmCfg->routeCfg[i].dRoute[pa_DROUTE_BROADCAST], &pDefRouteCfg[i].dRouteInfo[pa_DROUTE_BROADCAST], pa_CMD_TX_DEST_0, TRUE, 0, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
          /* Error checking */
          if (retCode != pa_OK)
            return (retCode);        
        }

        if (ctrlBitMap & pa_EMAC_IF_DEFAULT_ROUTE_UC_ENABLE)
        {
          retCode = pa_conv_routing_info2 (paInst, &pDefRoutefrmCfg->routeCfg[i].dRoute[pa_DROUTE_NO_MATCH], &pDefRouteCfg[i].dRouteInfo[pa_DROUTE_NO_MATCH], pa_CMD_TX_DEST_0, TRUE, 0, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
          /* Error checking */
          if (retCode != pa_OK)
            return (retCode);        
        }            
      }
              
      /* Default route command, can be any pdsp, POST PDSP0 is used as it is lightly loaded */
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
            (port >  pa_EMAC_PORT_7))
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
           
      /* Default route command, can be any pdsp, POST PDSP0 is used as it is lightly loaded */
      *cmdDest = pa_CMD_TX_DEST_5; 
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
static pafrmCommand_t* pa_format_fcmd_header (paInst_t *paInst, void *pCmd, paCmdReply_t *reply, uint8_t cmd, uint16_t comId, uint8_t firstPdsp, uint16_t cmdSize)
{
  pafrmCommand_t *fcmd = (pafrmCommand_t *) pCmd;
  
  memset(fcmd, 0, cmdSize);

  fcmd->status        = PAFRM_CFG_CMD_STATUS_PROC;
  fcmd->pdspIndex     = SWIZ(firstPdsp);
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

uint32_t pa_clr_stats[2];
/********************************************************************************
 * FUNCTION PURPOSE: Clear ACL statistics
 ********************************************************************************
 * DESCRIPTION: This function clear ACL statistics
 ********************************************************************************/
static void pa_clear_acl_stats (paAclEntry_t *aclEntry)
{
  volatile    uint32_t *pSrcCnt;
  pSrcCnt = (uint32_t *) &paLObj.pSysRegs->COLLECT_STATS[PASS_ACL_STATS_BASE_ENTRY(aclEntry->hdr.tableIdx)];
  pa_clr_stats[0] = pSrcCnt[0];
  pa_clr_stats[1] = pSrcCnt[1];
  
} /* pa_clear_acl_stats */  

/********************************************************************************
 * FUNCTION PURPOSE: Clear FC statistics
 ********************************************************************************
 * DESCRIPTION: This function clear ACL statistics
 ********************************************************************************/
static void pa_clear_fc_stats (paFcEntry_t *fcEntry)
{
  volatile    uint32_t *pSrcCnt;
  pSrcCnt = (uint32_t *) &paLObj.pSysRegs->COLLECT_STATS[PASS_FC_STATS_BASE_ENTRY(fcEntry->hdr.tableIdx)];
  pa_clr_stats[0] = pSrcCnt[0];
  
} /* pa_clear_fc_stats */ 



/********************************************************************************
 * FUNCTION PURPOSE: ACL Set Score
 ********************************************************************************
 * DESCRIPTION: This function sets the score operations
 ********************************************************************************/
static paReturn_t pa_acl_set_score(uint16_t score_hi, 
                                   uint16_t score_lo, 
                                   uint16_t *score,
                                   uint8_t  *trigRescore, 
                                   int      mode)
{
  paReturn_t ret = pa_OK;
  *trigRescore = PA_ACL_RESCORE_NO_TRIGGER;
 
  switch (mode) {
    case pa_ACL_INSERT_TOP:
      if ((score_hi - score_lo) > PA_ACL_IDEAL_SCORE_DISTANCE) {
        *score = score_lo + PA_ACL_IDEAL_SCORE_DISTANCE;
      }
      else {
        *score = (score_hi + score_lo) >> 1;
      }
      break;

    case pa_ACL_INSERT_BOTTOM:
      if ((score_hi - score_lo) > PA_ACL_IDEAL_SCORE_DISTANCE) {
        *score = score_hi - PA_ACL_IDEAL_SCORE_DISTANCE;
      }
      else {
        *score = (score_hi + score_lo) >> 1;
      }      
      break;

    case pa_ACL_INSERT_RANDOM:
      if ((score_hi - score_lo) > PA_ACL_IDEAL_SCORE_DISTANCE) {
        if (score_hi <= PA_ACL_MID_SCORE_LIST) {
          *score = score_hi - PA_ACL_IDEAL_SCORE_DISTANCE;
        }
        else {
          *score = score_lo + PA_ACL_IDEAL_SCORE_DISTANCE;         
        }
      }
      else {
        *score = (score_hi + score_lo) >> 1;
      }       
      break;

    default:
       ret = pa_ERR_CONFIG;
  }

  if (ret == pa_OK) {
    if ( ( abs((int)(score_hi - *score)) <= PA_ACL_MIN_RESCORE_DISTANCE ) ||
         ( abs((int)(score_lo - *score)) <= PA_ACL_MIN_RESCORE_DISTANCE ) ) {
      *trigRescore = PA_ACL_RESCORE_TRIGGER;       
    }
  }

  return (ret);
}

/********************************************************************************
 * FUNCTION PURPOSE: ACL Entry Allocate
 ********************************************************************************
 * DESCRIPTION: This function does ACL entry allocate operations
 ********************************************************************************/
static paAclLinkListElem_t* pa_acl_list_elem_alloc(paAclLinkListElem_t  *list, int listSize)
{
  int i;

  for (i = 0; i < listSize; i++)  {
    if (list[i].ctrlBitMap & PA_ACL_LINK_ELEM_ACTIVE_ENTRY)
      continue;
    else
      break;
  }

  if (i == listSize)
    return (NULL);
  else
    return (&list[i]);    
}

/********************************************************************************
 * FUNCTION PURPOSE: ACL Entry Free
 ********************************************************************************
 * DESCRIPTION: This function does ACL entry Free operations
 ********************************************************************************/
static void pa_acl_list_elem_free(paAclLinkListElem_t  *list)
{
  /* Clear the control bit indicating it is inactive and available for next use */
  list->ctrlBitMap &= ~PA_ACL_LINK_ELEM_ACTIVE_ENTRY;
  memset(list, 0, sizeof(paAclLinkListElem_t));
  return;
}

/********************************************************************************
 * FUNCTION PURPOSE: ACL Entry Insert
 ********************************************************************************
 * DESCRIPTION: This function does ACL entry insert operations
 ********************************************************************************/
 static uint16_t pa_acl_init_score(int insertMode)
{
  if (insertMode == pa_ACL_INSERT_TOP)
    return 0x100;
  else if (insertMode == pa_ACL_INSERT_BOTTOM)
    return 0xff00;
  else
    return 0x8000;
}

/********************************************************************************
 * FUNCTION PURPOSE: ACL Entry Insert
 ********************************************************************************
 * DESCRIPTION: This function does ACL entry insert operations
 ********************************************************************************/
static paReturn_t pa_insert_entry( paAclLinkListElem_t  *list, 
                                   int                   listSize,
                                   paAclLinkListInfo_t  *info,
                                   int                   insertMode, 
                                   paHandleAcl_t         nextEntry, 
                                   uint32_t              tableIdx,
                                   uint16_t             *priority, 
                                   paHandleAcl_t         aclHandle)
{
  paAclLinkListElem_t *listEntry, *cur, *prev;
  uint16_t             score_hi, score_lo;
  uint8_t              trigRescore = PA_ACL_RESCORE_NO_TRIGGER;

  listEntry = pa_acl_list_elem_alloc(list, listSize);

  /* listEntry can not be NULL, if this happens, the system is in corrupted state, and 
   * may not be able to recover from it */
  if (listEntry == NULL)
    return (pa_HANDLE_TABLE_FULL); 

  /* Initialize the list entry allocated */
  listEntry->ctrlBitMap   = PA_ACL_LINK_ELEM_ACTIVE_ENTRY;
  listEntry->elemIdx      = tableIdx;
  listEntry->aclHandle    = aclHandle;
  listEntry->scorePending = 0;

  if (info->numEntries == 0)  {
    /* continue initialize the list entries 
     * This is the first entry, set the score based on the insert mode and
     * insert this element as the first entry at the ACL List
     */
    listEntry->prev         = NULL;
    listEntry->next         = NULL;
    listEntry->score        = pa_acl_init_score(insertMode);
    /* This is the first entry, both head and tail point to same entry */    
    info->head = info->tail = listEntry;    
  }
  else if (nextEntry == NULL) {
    /* continue initialize the list entries 
     * This is insert at bottom of the last entry 
     * Find the score of the last entry as score_hi, 
     * set the score_lo = 0. Set the score using
     * the high and low values. 
     * Insert this element as the last entry at the ACL list
     */
    listEntry->prev         = info->tail;
    listEntry->next         = NULL;

    score_hi                = info->tail->score;
    score_lo                = 0;
    pa_acl_set_score(score_hi, score_lo, &listEntry->score, &trigRescore, insertMode);

    /* insert this as the last entry in the ACL list */
    info->tail->next        = listEntry;
    
    /* update the new tail */
    info->tail              = listEntry;
  }
  else {
    /* continue initialize the list entries:
     * Insert this element in front of the ACL element with its
     * achHandle equal to nextEntry, find score_lo as the score of that entry and 
     * score_hi as the score of the previous entry. Set the score, using the high
     * and low values.
     */

    cur = info->head;
    
    while (cur != NULL) {
      /* found the next entry */
      if (nextEntry == cur->aclHandle) {
        break;
      }
      /* Check the next element */
      cur = cur->next;
    }

    /* Error handling */
    if (cur == NULL)
      return (pa_ERR_CONFIG);

    /* Insert the new element in front of the ACL element with its AclHandle equal to nextEntry */
    prev                    = cur->prev;

    listEntry->next         = cur;
    listEntry->prev         = prev;
    cur->prev               = listEntry;

    /* Set the scores */
    score_lo                = cur->score;    
    if (prev != NULL) {
      prev->next            = listEntry;      
      score_hi              = prev->score;
    }
    else {
      score_hi              = 0xFFFF;
      /* update head  */ 
      info->head            = listEntry;
    }
    
    pa_acl_set_score(score_hi, score_lo, &listEntry->score, &trigRescore, insertMode);

  }

  /* Update the manual score */
  *priority = listEntry->score;
 
  if (trigRescore == PA_ACL_RESCORE_TRIGGER) {
    info->ctrlBitMap |= PA_ACL_LINK_INFO_RESCORING_ACTIVE;
  }
    
  info->numEntries++;

  return (pa_OK);
}

/********************************************************************************
 * FUNCTION PURPOSE: ACL Entry remove
 ********************************************************************************
 * DESCRIPTION: This function does ACL entry remove operations
 ********************************************************************************/
static paReturn_t pa_remove_entry(paAclLinkListInfo_t  *info, paHandleAcl_t aclHandle)
{
  paAclLinkListElem_t *delNode, *prev, *next;

  delNode  = info->head;

  while (delNode != NULL)  {
    if (delNode->aclHandle == aclHandle)
      break;
    delNode = delNode->next;
  }

  prev = delNode->prev;
  next = delNode->next;

  if (prev != NULL)
    prev->next = delNode->next;
  else
    info->head = next; /* update head to next link as it is head deletion */
  
  if (next != NULL)
    next->prev = prev;
  else
    info->tail = prev; /* update tail to prev link as it is tail deletion */

  /* now clear the delNode, free it by making as inactive */
  pa_acl_list_elem_free(delNode);
  
  /* Update the entries */
  info->numEntries--;

  return(pa_OK);
}

/********************************************************************************
 * FUNCTION PURPOSE: get ACL rescore list header address
 ********************************************************************************
 * DESCRIPTION: This function returns the base address of the rescore list header
 ********************************************************************************/
paAclRescoreHdr_t* pa_get_rescore_hdr_addr(int aclInst)
{
  CSL_Pa_clRegs         *paClRegs;
  
  if (aclInst == pa_ACL_INST_OUTER_IP) {
    paClRegs = (CSL_Pa_clRegs *)paLObj.pClRegs[PASS_CLUSTER_INGRESS0];
    return ((paAclRescoreHdr_t *) &paClRegs->PDSP_SRAM[PAFRM_OUT_ACL_RESCORE_HDR_OFFSET]);    
  }
  else {
    paClRegs = (CSL_Pa_clRegs *)paLObj.pClRegs[PASS_CLUSTER_INGRESS3];    
    return ((paAclRescoreHdr_t *) &paClRegs->PDSP_SRAM[PAFRM_IN_ACL_RESCORE_HDR_OFFSET]);    
  }
}

/********************************************************************************
 * FUNCTION PURPOSE: get ACL rescore list data address
 ********************************************************************************
 * DESCRIPTION: This function returns the base address of the rescore list
 ********************************************************************************/
paAclRescoreList_t* pa_get_rescore_table_addr(int aclInst)
{
  CSL_Pa_clRegs         *paClRegs;
  
  if (aclInst == pa_ACL_INST_OUTER_IP) {
    paClRegs = (CSL_Pa_clRegs *)paLObj.pClRegs[PASS_CLUSTER_INGRESS0];
    return ((paAclRescoreList_t *) &paClRegs->PDSP_SRAM[PAFRM_OUT_ACL_RESCORE_DATA_OFFSET]);    
  }
  else {
    paClRegs = (CSL_Pa_clRegs *)paLObj.pClRegs[PASS_CLUSTER_INGRESS3];    
    return ((paAclRescoreList_t *) &paClRegs->PDSP_SRAM[PAFRM_IN_ACL_RESCORE_DATA_OFFSET]);    
  }
}

/********************************************************************************
 * FUNCTION PURPOSE: ACL rescore list generation
 ********************************************************************************
 * DESCRIPTION: This function generates the ACL rescore list
 ********************************************************************************/
static paReturn_t pa_gen_rescore_list( paAclEntry_t         *tbl, 
                                       paAclLinkListInfo_t  *info, 
                                       paAclRescoreList_t   *rescoreList, 
                                       paAclRescoreHdr_t    *rescHdr,
                                       int                   mode,
                                       uint16_t             *pendingIndex)
{
  paAclEntry_t           *aclTable = tbl;
  paAclEntry_t           *aclEntry;
  paL2L3Header_t         *hdr;        /* ACL header */
  paAclLinkListElem_t    *cur, *prev, *next;
  int                     rFlag, numE = info->numEntries, numEntries = 0;
  uint16_t                prevScore, nextScore;
  paReturn_t              ret = pa_OK;
  uint32_t                temp;

  /* MSB bit is set to indicate rescore command for firmware */
  *pendingIndex = PA_ACL_FW_RESCORE_LIST_GENERATED;  

  /* Update the scorePending for all the entries that need re-score */
  if (mode == pa_ACL_INSERT_TOP) {   
    info->head->scorePending = (numE * PA_ACL_IDEAL_SCORE_DISTANCE);  
  }
  else if (mode == pa_ACL_INSERT_BOTTOM) {
    info->head->scorePending = PA_ACL_HIGH_SCORE_LIST; 
  } 
  else {
    info->head->scorePending = PA_ACL_MID_SCORE_LIST + ((numE >> 1) * PA_ACL_IDEAL_SCORE_DISTANCE);  
  }

  cur   = info->head;
  /* Clear the rescoring schedule bit */
  cur->ctrlBitMap &= ~PA_ACL_LINK_ELEM_RESCORE_COMPLETE;  
  
  /* Track and update the final scores  */
  while (cur->next != NULL) {      
    /* Next update */  
    cur  = cur->next;      
    prev = cur->prev;     
    cur->scorePending = prev->scorePending - PA_ACL_IDEAL_SCORE_DISTANCE;  
    /* Clear the rescoring schedule bit */
    cur->ctrlBitMap &= ~PA_ACL_LINK_ELEM_RESCORE_COMPLETE;    
  }

  /* Do rescore operation until all the elements are scheduled */
  while ( numEntries < numE ) {
    /* start from the beginning till end */
    cur = info->head; 
    while (cur != NULL ) {
      /* Try next, if rescore schedule is complete for this score */
      if (cur->ctrlBitMap & PA_ACL_LINK_ELEM_RESCORE_COMPLETE) {
        cur     = cur->next;
        continue;
      }

      prev      = cur->prev;
      /* Limit the previous score */
      if (prev != NULL) {
        prevScore = prev->score;
      }
      else {
        prevScore = PA_ACL_HIGH_SCORE_LIST;
      }
      
      next      = cur->next;      
      /* Limit the next score */
      if (next != NULL) {
        nextScore = next->score;
      }
      else {
        nextScore = PA_ACL_LOW_SCORE_LIST;
      }  

      /* Update the AclEntry */
      aclEntry = &aclTable[cur->elemIdx];
      hdr      = &aclEntry->hdr;
      /* Check if rescore can be allowed at this time on this entry */
      if ( (cur->scorePending < prevScore) &&
           (cur->scorePending > nextScore) )  {
        rFlag = 1;
      }
      else if ( (cur->scorePending == PA_ACL_HIGH_SCORE_LIST) || 
                (cur->scorePending == PA_ACL_LOW_SCORE_LIST) ){
        rFlag = 1;       
      }
      else {
        rFlag = 0;
      }

      if (rFlag == 1)  {
         /* Allowed, update the score during border cases also */
         cur->ctrlBitMap |= PA_ACL_LINK_ELEM_RESCORE_COMPLETE; 

         if (cur->score != cur->scorePending)  {
           cur->score = cur->scorePending;

           /* Record the pending LUT1 index offset update */
           if ( cur->elemIdx == info->lastIdx) {
             *pendingIndex  |= numEntries << 2;
             /* MSB bit is set to indicate rescore command for firmware */
             *pendingIndex |= PA_ACL_FW_RESCORE_PEND_LUT1_IDX;             
           }

           /* Record the sequence */
           temp = (hdr->lutIdx << 16) + cur->score;   
           rescoreList->idx_score[numEntries++] = (temp); 
         }
      }

      /* check the next score */
      cur = next;
    }  
  }
  /* Record number of rescore elements and offsets in the rescore hdr for scratch memory 
   * store the final offset as (numEntries * 4), to help firmware to goto next entry 
   * without shifting to save a cycle
   */
  rescHdr->final_cur      = numEntries << 18;

  return (ret);
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
  uint16_t                priority, bitMask = 0;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey, CBWords0, CBWords1; 
  paVirtualLnk_t          *vlnkTable;
   
  /* Param parsing */ 
  paVirtualLnk_t  *nextLink;
  uint16_t        lut1Index;
  paRouteInfo2_t  *routeInfo;
  paRouteInfo2_t  *nextRtFail;
  
  if((ethInfo == NULL) || (params == NULL) || (params->routeInfo == NULL) || (params->nextRtFail == NULL) || (handle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  nextLink = (params->validBitMap & pa_PARAM_VALID_NEXTLINK)?((paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,params->nextLink)):NULL;
  lut1Index = (params->validBitMap & pa_PARAM_VALID_INDEX)?(uint16_t)params->index:PAFRM_LUT1_INDEX_LAST_FREE;
  routeInfo = params->routeInfo;
  nextRtFail = params->nextRtFail;

  if (paLObj.cfg.rmServiceHandle) {
    int32_t lutInst = pa_LUT1_INST_0_0;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &lutInst, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }

	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &lutInst, NULL)) {
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
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN_PRI)
    l2Entry.cfg.mac.vlanPri    =  ethInfo->vlanPri;
    
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
    for (i = 0; i < paInst->nL2;  i++)  {

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

  }


  /* Find a free entry in the table */
  for (i = 0; i < paInst->nL2; i++)  {

    if (l2Table[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;

  }

  if (i == paInst->nL2)  {
    ret = pa_HANDLE_TABLE_FULL;
    goto Pa_addMac2_end;
  }
  
Pa_addMac2_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *handle = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l2Table[i]);

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_L2  | i, 0, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  al1->index    = SWIZ(lut1Index);
  
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
  
    if ((nextLink->hdr.type != PA_TABLE_ENTRY_TYPE_VL) ||
        (nextLink->hdr.status != PA_TBL_STAT_ACTIVE)   ||
        (nextLink->hdr.subType != PA_TABLE_ENTRY_SUBTYPE_VLINK_MAC))
    {
      ret = pa_INVALID_INPUT_HANDLE;
      Pa_osalEndMemAccess ((void *) vlnkTable,
                           paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
      goto Pa_addMac2_end;
    }
    
    al1->type         = PAFRM_COM_ADD_LUT1_VLINK;
    al1->vLinkNum     = SWIZ(nextLink->hdr.tableIdx);
	Pa_osalEndMemAccess ((void *) vlnkTable,
                        paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  }  else {
    al1->type         = PAFRM_COM_ADD_LUT1_STANDARD;
  }
    
  CBWords0      = PAFRM_LUT1_CLASS_STANDARD << PAFRM_LUT1_CLASS_SHIFT;
  CBWords1      = PAFRM_LUT1_VALID_PKTTYPE;
  priority      = 0; 
  
  al1->u.mac.pktType     = PAFRM_L2_PKT_TYPE_MAC;

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_DST)
  {
    memcpy (al1->u.mac.dmac, l2Entry.cfg.mac.dstMac, sizeof(paMacAddr_t));
    CBWords0 |=  PAFRM_LUT1_VALID_DMAC_ALL;
    priority += 10;
  }  

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_SRC)
  {
    memcpy (al1->u.mac.smac, l2Entry.cfg.mac.srcMac, sizeof(paMacAddr_t));
    CBWords0 |=  PAFRM_LUT1_VALID_SMAC;
    priority += 10;
  }  

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_ETHERTYPE)
  {
    al1->u.mac.etherType = SWIZ(l2Entry.cfg.mac.ethertype);
    CBWords0 |=  PAFRM_LUT1_VALID_ETHERTYPE;
    priority += 10;
    
  }
  
  if (ethInfo->validBitMap & pa_ETH_INFO_VALID_MPLSTAG) 
  {
    al1->u.mac.mpls     = SWIZ(l2Entry.cfg.mac.mplsTag);
    CBWords0 |=  PAFRM_LUT1_VALID_MPLS;
    priority += 10;
  }

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN)
  {
    al1->u.mac.vlanId1  = SWIZ(l2Entry.cfg.mac.vlan);
    CBWords1 |=  PAFRM_LUT1_VALID_VLANID1;
    priority += 10;
    
  }  
    
  if (ethInfo->validBitMap & pa_ETH_INFO_VALID_INPORT)
  {
    al1->u.mac.inport  = SWIZ(l2Entry.cfg.mac.inport);
    CBWords1 |=  PAFRM_LUT1_VALID_INPORT;
    priority += 10;
  }

  if (ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN_PRI)
  {
    al1->u.mac.vlanPri1 = SWIZ(l2Entry.cfg.mac.vlanPri);
    CBWords1 |=  PAFRM_LUT1_VALID_VLAN_PRI1;
    priority += 10;
  }  
  
  al1->CBWords0 = SWIZ(CBWords0);
  al1->CBWords1 = SWIZ(CBWords1); 
  al1->priority = SWIZ(priority);
  al1->bitMask  = SWIZ(bitMask);

  /* Forwarding information */
  ret1 = pa_conv_routing_info2(paInst, &al1->match, routeInfo, pa_CMD_TX_DEST_0, FALSE, 0, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addMac2_end;
  }

  /* Next fail information */
  ret1 = pa_conv_routing_info2(paInst, &al1->nextFail, nextRtFail, pa_CMD_TX_DEST_0, TRUE, 0, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
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
    l2Entry.hdr.pdspNum  = PASS_INGRESS0_PDSP0;
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
  uint16_t                lut1Index = (index == pa_LUT1_INDEX_NOT_SPECIFIED)?PAFRM_LUT1_INDEX_LAST_FREE:(uint16_t)index;
  uint16_t                priority, bitMask;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey, CBWords0, CBWords1;  
  
  if((srioInfo == NULL) || (routeInfo == NULL) || (nextRtFail == NULL) || (handle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  if (paLObj.cfg.rmServiceHandle) {
    int32_t lutInst = pa_LUT1_INST_0_0;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &lutInst, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }
	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &lutInst, NULL)) {
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
    for (i = 0; i < paInst->nL2;  i++)  {

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

    for (i = 0; i < paInst->nL2; i++)  {

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
  for (i = 0; i < paInst->nL2; i++)  {

    if (l2Table[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;

  }

  if (i == paInst->nL2)  {
    ret = pa_HANDLE_TABLE_FULL;
    goto Pa_addSrio_end;                 
  }
  
Pa_addSrio_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l2Table[i]);

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_L2  | i, 0, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  al1->index    = SWIZ(lut1Index);
  al1->type     = PAFRM_COM_ADD_LUT1_SRIO;
  CBWords0      = PAFRM_LUT1_CLASS_STANDARD << PAFRM_LUT1_CLASS_SHIFT;
  CBWords1      = PAFRM_LUT1_VALID_PKTTYPE;
  priority      = 0;
  bitMask       = 0; 
  
  al1->u.mac.pktType     = PAFRM_L2_PKT_TYPE_SRIO;

  /* Form the matchflags and the key */
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_SRC_ID) {
    al1->u.srio.srcId = SWIZ(l2Entry.cfg.srio.srcId);
    CBWords1 |= PAFRM_LUT1_VALID_SRIO_SRC_ID;
    priority += 10;
  }
  
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_DEST_ID) {
    al1->u.srio.dstId = SWIZ(l2Entry.cfg.srio.destId);
    CBWords1 |= PAFRM_LUT1_VALID_SRIO_DST_ID;
    priority += 10;
  }
  
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_PRI) {
    al1->u.srio.pri          = SWIZ((uint16_t)l2Entry.cfg.srio.pri);
    CBWords1 |= PAFRM_LUT1_VALID_SRIO_PRI;
    priority += 10;
  }
  
  /* pa_SRIO_INFO_VALID_TYPE_INFO_MAILBOX */
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_STREAMID) {
    al1->u.srio.typeParam1   = SWIZ(l2Entry.cfg.srio.typeParam1);
    CBWords0 |= PAFRM_LUT1_VALID_SRIO_TYPE_PARAM1;
    priority += 10;
  }
  
  /* pa_SRIO_INFO_VALID_TYPE_INFO_LETTER */
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO_COS) {
    al1->u.srio.typeParam2   = SWIZ(l2Entry.cfg.srio.typeParam2);
    CBWords0 |= PAFRM_LUT1_VALID_SRIO_TYPE_PARAM2;
    bitMask  |= 0xff;
    priority += 10;
  }
  
  if(srioInfo->tt ==  pa_SRIO_TRANSPORT_TYPE_0)
    al1->u.srio.pktFlags |= PAFRM_SRIO_FLAG_PORT8;
    
  if (srioInfo->validBitMap & pa_SRIO_INFO_VALID_TYPE_INFO)
  {  
    if(srioInfo->msgType == pa_SRIO_TYPE_9)
        al1->u.srio.pktFlags = PAFRM_SRIO_FLAG_TYPE9;
    else  
        al1->u.srio.pktFlags = PAFRM_SRIO_FLAG_TYPE11;
  }
   
  CBWords0 |= PAFRM_LUT1_VALID_PKTFLAGS;
  bitMask  |= (al1->u.srio.pktFlags << 8);
  
  al1->u.srio.nextHdr       = SWIZ(pa_next_hdr_tbl[nextHdr]);
  al1->u.srio.nextHdrOffset = SWIZ(nextHdrOffset);
  
  al1->CBWords0 = SWIZ(CBWords0);
  al1->CBWords1 = SWIZ(CBWords1); 
  al1->priority = SWIZ(priority);
  al1->bitMask  = SWIZ(bitMask);

  /* Forwarding information */
  ret1 = pa_conv_routing_info(paInst, &al1->match, routeInfo, pa_CMD_TX_DEST_0, FALSE, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1; 
    goto Pa_addSrio_end;                 
  }

  /* Next fail information */
  ret1 = pa_conv_routing_info(paInst, &al1->nextFail, nextRtFail, pa_CMD_TX_DEST_0, TRUE, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
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
  paL2L3Header_t        *hdr1;
  paL3Entry_t           *l3e     = NULL;
  pafrmCommand_t        *fcmd;
  pafrmCommandDelLut1_t *del;
  pafrmCommandCmdHdr_t  *cmdHdr;
  uint16_t              csize, comId;
  paReturn_t            ret = pa_OK;
  uint32_t              mtCsKey;  
  int                   fDelLnk = FALSE;
  uint8_t               pdsp = 0;
  paL2Entry_t 		   *l2Table;
  paVirtualLnk_t       *vlnkTable;
  vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);  
  
  if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+2*sizeof(pafrmCommandDelLut1_t)+2*sizeof(pafrmCommandCmdHdr_t)-sizeof(uint32_t);
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
  l2Table   = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);

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
  {
    comId = PA_COMID_L2 | hdr->tableIdx;
  }  
  else {
    comId = PA_COMID_L3 | hdr->tableIdx;
    l3e = &l3Table[hdr->tableIdx];
    
    if (l3e->pHandle) {
      /* note: The L2L3 header and the virtual link header are compatible */
      hdr1 = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3e->pHandle);
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
  
  /* Find command destination and PDSP number */
  switch (hdr->pdspNum)
  {
    case PASS_INGRESS0_PDSP0:
        *cmdDest = pa_CMD_TX_DEST_0;
        break;
        
    case PASS_INGRESS1_PDSP0:
        *cmdDest = pa_CMD_TX_DEST_1;
        break;
        
    case PASS_INGRESS1_PDSP1:
        *cmdDest = pa_CMD_TX_DEST_1;
        /*
         * hdr->type == PA_TABLE_ENTRY_TYPE_L3 and l3e->pHandle != NULL; Therefore Hdr1 exists
         */
        if ((hdr->type != PA_TABLE_ENTRY_TYPE_L3) || (hdr->subType != PA_TABLE_ENTRY_SUBYTPE_IPSEC) || (!l3e->pHandle))
        {
            ret = pa_INVALID_INPUT_HANDLE;
            goto Pa_delHandle_end;
        } 
        if ((hdr1->lnkCnt == 0) && (hdr1->subType == PA_TABLE_ENTRY_SUBYTPE_IP_IPSEC))
        {
            /* It it time to delete the IP entey which is associated with the IPSEC entry */
            fDelLnk = TRUE;   
        }
        else
        {
            pdsp = 1;
        }
        break;
    
    case PASS_INGRESS2_PDSP0:
        *cmdDest = pa_CMD_TX_DEST_2;
        break;
        
    case PASS_INGRESS4_PDSP0:
        *cmdDest = pa_CMD_TX_DEST_4;
        break;
        
    default:
        ret = pa_ERR_CONFIG;
        goto Pa_delHandle_end;
  }
  
  /* Create the command */
  if (fDelLnk)
  {
    fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_MULTI_CMDS, 0, pdsp, csize);
    cmdHdr = (pafrmCommandCmdHdr_t *) &(fcmd->cmd);
    cmdHdr->command = PAFRM_CONFIG_COMMAND_DEL_LUT1;
    cmdHdr->offset =  sizeof(pafrmCommand_t)+sizeof(pafrmCommandDelLut1_t)+sizeof(pafrmCommandCmdHdr_t)-sizeof(uint32_t);
    cmdHdr->offset = SWIZ(cmdHdr->offset);
    cmdHdr->comId = PA_COMID_L3  | hdr1->tableIdx;
    cmdHdr->comId = SWIZ(cmdHdr->comId );
    del = (pafrmCommandDelLut1_t *)((uint8_t *)cmdHdr + sizeof(pafrmCommandCmdHdr_t));
    del->index = SWIZ((uint16_t)hdr1->lutIdx);
  }
  else
  {
    fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_DEL_LUT1, comId, pdsp, csize);
    del = (pafrmCommandDelLut1_t *)&(fcmd->cmd);
    del->index = SWIZ((uint16_t)hdr->lutIdx);
  }  
  
  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  /* Mark the entry as disabled */
  hdr->status = PA_TBL_STAT_INACTIVE;
  
  if (fDelLnk)
  {
    /* Processing the second command */
    cmdHdr = (pafrmCommandCmdHdr_t *) ((uint8_t *)del + sizeof(pafrmCommandDelLut1_t));
    cmdHdr->command = PAFRM_CONFIG_COMMAND_DEL_LUT1;
    cmdHdr->offset = 0;
    cmdHdr->comId = PA_COMID_L3  | hdr->tableIdx;
    cmdHdr->comId = SWIZ(cmdHdr->comId);
    
    del = (pafrmCommandDelLut1_t *)((uint8_t *)cmdHdr + sizeof(pafrmCommandCmdHdr_t));
    del->index = SWIZ((uint16_t)hdr->lutIdx);
    
    /* Mark the link as inactive */
    hdr1->status = PA_TBL_STAT_INACTIVE;
  }
  
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

  /* To get pass warnings of un-used variables */
  iHandle               = iHandle;
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
  paInst_t              *paInst  = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL2L3Header_t        *hdr     = (paL2L3Header_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, *handle);
  pafrmCommand_t        *fcmd;
  pafrmCommandDelLut1_t *del;
  uint16_t              csize, comId;
  paReturn_t            ret = pa_OK;
  uint32_t              mtCsKey;  
  uint8_t               pdsp = 0;
  paAclEntry_t          *aclTable = (paAclEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_ACL_TABLE].base);
  paAclLinkListInfo_t   *listInfoTable;  
  
  if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+ sizeof(pafrmCommandDelLut1_t) -sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if ((reply->dest != pa_DEST_HOST) && (reply->dest != pa_DEST_DISCARD))
    return (pa_INVALID_CMD_REPLY_DEST);
    
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Inform the host a table entry will be changed. */
  Pa_osalBeginMemAccess (aclTable,
                         paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);

  /* Basic sanity check. The base address of the table is not verified, however */
  if ((hdr == NULL) || (hdr->type != PA_TABLE_ENTRY_TYPE_ACL)) {
    ret = pa_INVALID_INPUT_HANDLE;
    goto Pa_delAclHandle_end;
  }

  if (hdr->status == PA_TBL_STAT_INACTIVE)  {
    ret = pa_HANDLE_INACTIVE;
    goto Pa_delAclHandle_end;
  }

  comId = PA_COMID_ACL | hdr->tableIdx;
  
  /* Find command destination and PDSP number */
  switch (hdr->pdspNum)
  {
    case PASS_INGRESS0_PDSP1:
        *cmdDest = pa_CMD_TX_DEST_0;
        pdsp = 1;
        listInfoTable = (paAclLinkListInfo_t *) PA_ACL_LINK_LIST_INFO_TABLE(1, paInst->nAcl, aclTable);
        break;
        
    case PASS_INGRESS3_PDSP0:
        *cmdDest = pa_CMD_TX_DEST_3;
        pdsp = 0;
        listInfoTable = (paAclLinkListInfo_t *) PA_ACL_LINK_LIST_INFO_TABLE(0, paInst->nAcl, aclTable);        
        break;
        
    default:
        ret = pa_ERR_CONFIG;
        goto Pa_delAclHandle_end;
  }
  
  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_DEL_LUT1, comId, pdsp, csize);
  del = (pafrmCommandDelLut1_t *)&(fcmd->cmd);
  del->index = SWIZ((uint16_t) hdr->lutIdx);
  
  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  /* Mark the entry as disabled */
  hdr->status = PA_TBL_STAT_INACTIVE;

  /* Remove this element from ACL list */
  pa_remove_entry(listInfoTable, *handle);
  
Pa_delAclHandle_end:
  
  /* Release the table */
  Pa_osalEndMemAccess (aclTable,
                       paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  /* clear the handle */
  *handle = NULL;

  return (ret);

} /* Pa_delAclHandle */

/***************************************************************************************
 * FUNCTION PURPOSE: Delete an EOAM handle
 ***************************************************************************************
 * DESCRIPTION: The handle is deleted. Dependent handles are left intact
 ***************************************************************************************/
paReturn_t Pa_delEoamHandle (Pa_Handle       iHandle,
                            paHandleEoam_t  *handle, 
                            paCmd_t         cmd,
                            uint16_t        *cmdSize,
                            paCmdReply_t    *reply,
                            int             *cmdDest )
{
  paInst_t              *paInst  = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL2L3Header_t        *hdr     = (paL2L3Header_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, *handle);
  pafrmCommand_t        *fcmd;
  pafrmCommandDelLut1_t *del;
  uint16_t              csize, comId;
  paReturn_t            ret = pa_OK;
  uint32_t              mtCsKey;  
  uint8_t               pdsp = 0;
  paEoamEntry_t         *eoamTable;
  
  
  if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+ sizeof(pafrmCommandDelLut1_t) -sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if ((reply->dest != pa_DEST_HOST) && (reply->dest != pa_DEST_DISCARD))
    return (pa_INVALID_CMD_REPLY_DEST);
    
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  eoamTable = (paEoamEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].base);  

  /* Inform the host a table entry will be changed. */
  Pa_osalBeginMemAccess (eoamTable,
                         paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);

  /* Basic sanity check. The base address of the table is not verified, however */
  if ((hdr == NULL) || (hdr->type != PA_TABLE_ENTRY_TYPE_EOAM)) {
    ret = pa_INVALID_INPUT_HANDLE;
    goto Pa_delEoamHandle_end;
  }

  if (hdr->status == PA_TBL_STAT_INACTIVE)  {
    ret = pa_HANDLE_INACTIVE;
    goto Pa_delEoamHandle_end;
  }

  comId = PA_COMID_EOAM | hdr->tableIdx;
  
  /* Find command destination and PDSP number */
  switch (hdr->pdspNum)
  {
    case PASS_INGRESS0_PDSP1:
        *cmdDest = pa_CMD_TX_DEST_0;
        pdsp = 1;
        break;
        
    default:
        ret = pa_ERR_CONFIG;
        goto Pa_delEoamHandle_end;
  }
  
  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_DEL_LUT1, comId, pdsp, csize);
  del = (pafrmCommandDelLut1_t *)&(fcmd->cmd);
  del->index = SWIZ((uint16_t) hdr->lutIdx);
  
  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  /* Mark the entry as disabled */
  hdr->status = PA_TBL_STAT_INACTIVE;
  
Pa_delEoamHandle_end:
  
  /* Release the table */
  Pa_osalEndMemAccess (eoamTable,
                       paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  /* clear the handle */
  *handle = NULL;

  return (ret);

} /* Pa_delEoamHandle */


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
  paInst_t              *paInst  = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL2L3Header_t        *hdr     = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, *handle);
  pafrmCommand_t        *fcmd;
  pafrmCommandDelLut1_t *del;
  uint16_t              csize, comId;
  paReturn_t            ret = pa_OK;
  uint32_t              mtCsKey;  
  paFcEntry_t           *fcTable = (paFcEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_FC_TABLE].base);;
  
  if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
  
  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+ sizeof(pafrmCommandDelLut1_t) -sizeof(uint32_t);
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* Sanity check the command reply information.  */
  if ((reply->dest != pa_DEST_HOST) && (reply->dest != pa_DEST_DISCARD))
    return (pa_INVALID_CMD_REPLY_DEST);
    
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Inform the host a table entry will be changed. */
  Pa_osalBeginMemAccess (fcTable,
                         paInst->paBufs[PA_BUFFNUM_FC_TABLE].size);

  /* Basic sanity check. The base address of the table is not verified, however */
  if ((hdr == NULL) || (hdr->type != PA_TABLE_ENTRY_TYPE_FC)) {
    ret = pa_INVALID_INPUT_HANDLE;
    goto Pa_delFcHandle_end;
  }

  if (hdr->status == PA_TBL_STAT_INACTIVE)  {
    ret = pa_HANDLE_INACTIVE;
    goto Pa_delFcHandle_end;
  }

  comId = PA_COMID_FC | hdr->tableIdx;
  
  /* Set command destination and PDSP number */
  *cmdDest = pa_CMD_TX_DEST_6;
  
  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_DEL_LUT1, comId, 0, csize);
  del = (pafrmCommandDelLut1_t *)&(fcmd->cmd);
  del->index = SWIZ((uint16_t) hdr->lutIdx);
  
  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  /* Mark the entry as disabled */
  hdr->status = PA_TBL_STAT_INACTIVE;
  
Pa_delFcHandle_end:
  
  /* Release the table */
  Pa_osalEndMemAccess (fcTable,
                       paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  /* clear the handle */
  *handle = NULL;

  return (ret);

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
  paInst_t		    *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  int               i;
  paVirtualLnk_t    *vlnkTable;
  paL2L3Header_t    *hdr;
  
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
    if (vlnkTable[i].hdr.status == PA_TBL_STAT_INACTIVE)
    {
      hdr          = (paL2L3Header_t *)&vlnkTable[i].hdr;
      hdr->status   = PA_TBL_STAT_ACTIVE;
      hdr->subType  = PA_TABLE_ENTRY_SUBTYPE_VLINK_BASE + lnkType;
      
      *vlinkHdl          = (paLnkHandle_t)	  pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&vlnkTable[i]);
      
  	  Pa_osalEndMemAccess ((void *) vlnkTable,
                           paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
      return (pa_OK);
    }
  }
  
  Pa_osalEndMemAccess ((void *) vlnkTable,
                       paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
                       
  return (pa_VIRTUAL_LINK_TABLE_FULL);

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
  if (vlnk->hdr.status == PA_TBL_STAT_ACTIVE)
  {
      vlnk->hdr.status = PA_TBL_STAT_INACTIVE;
      vlnk->hdr.lnkCnt = 0;
      vlnk->hdr.subType = 0;
      *vlinkHdl = NULL;  /* Clear the handle */
      ret =  pa_OK;
      
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

static paRouteInfo2_t  pauForwardLut1Route =  { 0,                           /* validBitmap */ 
                                                pa_DEST_CONTINUE_PARSE_LUT1, /* Dest */
						                                    0,	           			         /* Flow ID */
						                                    0,					                 /* queue */
						                                   -1,					                 /* Multi route */
						                                    0,					                 /* sw Info 0 */
							                                  0,  				                 /* sw Info 1 */
                                                0,                           /* customType : not used */         
                                                0,                           /* customIndex: not used */     
                                                0,                           /* pkyType: for SRIO only */    
                                                NULL,                        /* No commands */
                                                0,                           /* Priority */
                                                NULL,                        /* efOpInfo */
                                                0                            /* ctrlBitMap */
                                              };                       
                                               
                                   
static paRouteInfo2_t   pauFailRoute = { 0,                     /* validBitmap */
                                         pa_DEST_DISCARD,       /* Dest */
							             0,					    /* Flow ID */
							             0,					    /* queue */
							             -1,					/* Multi route */
							             0,					    /* sw Info 0 */
							             0,  				    /* sw Info 1 */
                                         0,                     /* customType : not used */         
                                         0,                     /* customIndex: not used */     
                                         0,                     /* pkyType: for SRIO only */    
                                         NULL,                  /* No commands */
                                         0,                     /* Priority */
                                         NULL,                  /* efOpInfo */
                                         0                      /* ctrlBitMap */                                         
                                         };                       
                                         
/*******************************************************************************************
 * FUNCTION PURPOSE: Add an IP address to the lookup table
 *******************************************************************************************
 * DESCRIPTION: The IP address is verified and added to the LUT1 table. 
 *              The following LUT1 are used to store IP related entries at NetCP 1.5:
 *                  - Ingess 1, PDSP0, LUT1_2: Outer IP
 *                  - Ingres 1, PDSP1, LUT1_3: First layer of IPSEC  (ESP or AH)
 *                  - Ingres 2, PDSP0, LUT1_4: Second layer of IPSEC (for ESP over AH only)
 *                  - Ingres 4, PDSP0, LUT1_6: Inner IP
 *                
 *              If the required IP entry is not linked or linked to an L2 entry, then this entry 
 *              is sent to Ingress 1 PDSP0 for LUT1_2. If the IP entry contains IPSEC SPI field,
 *              then a corresponding entry will be added to Ingress1 PDSP1 for LUT1_3 by creating
 *              the configuration packet with multiple LUT1 commands.
 *
 *              If the required IP entey conatins SPI only and is linked to an IPSEC entry, then this
 *              entry is sent to Ingress 2 PDSP0 for LUT1_4.
 *  
 *              If the required IP entry does not conatin SPI and is linked to another L3 entry, then 
 *              the entry is sent to Ingress 4 PDSP0 for LUT1_6.
 *              Note: Inner IP with SPI is not supported.
 *
 *  Note: The firewall entries are stored at Ingress 0, PDSP1, LUT1_1 and Ingress 3, PDSP0 LUT1_5 
 *        and those entries will be processed by API (TBD)
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
 * DESCRIPTION: The 2nd generation addIp function. 
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
  paL3Entry_t             l3Entry, l3Entry2;
  int                     fl3Entry2 = FALSE;
  int                     fNonIp;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  pafrmCommandCmdHdr_t   *cmdHdr;
  int                     i,j;
  uint16_t                csize;
  uint16_t                l2Release;
  uint16_t                lut1Index2 = PAFRM_LUT1_INDEX_LAST_FREE; 
  paL2Entry_t 		 *l2Table	= (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);
  paVirtualLnk_t     *vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);
  /* Param parsing */ 
  int             lutInst;
  paLnkHandle_t   prevLink;
  paLnkHandle_t   nextLink;
  uint16_t        lut1Index;
  paRouteInfo2_t  *routeInfo;
  paRouteInfo2_t  *nextRtFail;
  paL2L3Header_t  *hdr;
  paVirtualLnk_t  *vhdr;
  uint16_t        priority, srcVC = 0, vLinkNum = 0;
  paReturn_t      ret = pa_OK;
  paReturn_t      ret1;
  uint32_t        mtCsKey, CBWords0, CBWords1;
  uint8_t         numNextLayers = 0;   /* Specify the number of next matching layers */
  paRouteInfo2_t  *mRoute, *nfRoute;
  int             fIpEntryFound = FALSE, fIpsecEntryFound = FALSE;
  uint8_t         firstPdsp = 0;
  
  
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
  if ((lutInst != pa_LUT_INST_NOT_SPECIFIED) && ((lutInst != pa_LUT1_INST_1) && (lutInst != pa_LUT1_INST_2))) {
    return(pa_INVALID_LUT_INST);
  }
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
    
  }
  
  l3Entry.u.ipInfo.ipType   =  ipInfo->ipType;
  l3Entry.u.ipInfo.validBitMap  =  ipInfo->validBitMap;
  
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_SPI)l3Entry.u.ipInfo.spi = ipInfo->spi;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_FLOW)l3Entry.u.ipInfo.flow = ipInfo->flow;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_GREPROTO)l3Entry.u.ipInfo.greProto = ipInfo->greProto;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_PROTO)l3Entry.u.ipInfo.proto = ipInfo->proto;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_TOS)l3Entry.u.ipInfo.tos = ipInfo->tos;
  if (ipInfo->validBitMap & pa_IP_INFO_VALID_SCTPPORT)l3Entry.u.ipInfo.sctpPort = ipInfo->sctpPort;
  
  /* The flow label is restricted to 20 bits */
  if (l3Entry.u.ipInfo.flow & ~PA_IP_FLOW_MASK)  
    return (pa_INVALID_IP_FLOW);
  
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  /* Signal the application that a table modification will be done. */
  l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  Pa_osalBeginMemAccess ((void*)l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
                         
  /* The prevLink or nextLink may be a virtual link, there we need to protect the
   * virtual link table
   */
                         
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)                       
      Pa_osalBeginMemAccess ((void *) vlnkTable,
                             paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
                         
  l2Release = FALSE;


  /* If there is a link to the l2 table then that table must be
   * protected as well. Of course the previous link could be an l3 or virtual
   * link as well as an l2 link. If so then the l2 protection is released */
  if (prevLink != NULL)  {
    
    if (hdr->type == PA_TABLE_ENTRY_TYPE_VL)
    {
      if(hdr->status != PA_TBL_STAT_ACTIVE){
        ret = pa_INVALID_INPUT_HANDLE;
        goto Pa_addIp2_end;
      }
      srcVC = hdr->tableIdx;
    }
    else{
      /* L2 or L3 Link */
      l2Release = TRUE;
      Pa_osalBeginMemAccess ((void *) l2Table,
                            paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);

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
      
      srcVC = PAFRM_MK_SRC_VC(hdr->pdspNum, hdr->lutIdx);
    }  
  }
  
  /* When next link is set check to see if the link is a valid virtual link */
  if (nextLink != NULL){
    
    if ((vhdr->hdr.type != PA_TABLE_ENTRY_TYPE_VL) || (vhdr->hdr.status != PA_TBL_STAT_ACTIVE))
    {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addIp2_end;
    }
    
    vLinkNum = vhdr->hdr.tableIdx;
  }  
    
  fNonIp = pa_ip_zero (l3Entry.u.ipInfo.src) && pa_ip_zero (l3Entry.u.ipInfo.dst) && (ipInfo->validBitMap & pa_IP_INFO_VALID_SPI);  
    
  /* find the command destination and related information */
  if (lutInst != pa_LUT_INST_NOT_SPECIFIED) {
    switch (lutInst)
    {
        case pa_LUT1_INST_1:
            l3Entry.hdr.pdspNum = PASS_INGRESS1_PDSP0;
            *cmdDest            = pa_CMD_TX_DEST_1;
            if (fNonIp && ipInfo->spi)
            {
               /* It must be an IPSEC ESP entry which is linked to the IPSEC AH entry 
                * or en IPSEC omly entry link to outer IP entry */
               if ((prevLink == NULL) || ((hdr->subType != PA_TABLE_ENTRY_SUBYTPE_IPSEC) && 
                                          (hdr->subType != PA_TABLE_ENTRY_SUBYTPE_IP)    &&
                                          (hdr->subType != PA_TABLE_ENTRY_SUBYTPE_IP_IPSEC)))
               {
                    ret = pa_ERR_CONFIG;
                    goto Pa_addIp2_end;
               } 
               
               if (hdr->subType != PA_TABLE_ENTRY_SUBYTPE_IPSEC)
               {
                   /* IPSEC only entry */
                   l3Entry.hdr.pdspNum = PASS_INGRESS1_PDSP1;
                   firstPdsp = 1;
                   lutInst = pa_LUT1_INST_1_1;
               }
               else
               {
                    /* IPSEC AH over ESP */
                    l3Entry.hdr.pdspNum = PASS_INGRESS2_PDSP0;
                    *cmdDest            = pa_CMD_TX_DEST_2;
                    lutInst = pa_LUT1_INST_2_0;
               }
            }
            else if (ipInfo->spi)
            {
                /* The entry contains both IP and IPSEC info, need to create two entries for IP and corresonding IPSEC */
                /* The IP entry must specify the protocol type so that it csn be distinguished  from IP only entry */
                if ((ipInfo->proto != IP_PROTO_ESP) && (ipInfo->proto != IP_PROTO_AUTH))
                {
                    ret = pa_ERR_CONFIG;
                    goto Pa_addIp2_end;
                }
                memset (&l3Entry2, 0, sizeof(paL3Entry_t));
                l3Entry2.hdr.pdspNum = PASS_INGRESS1_PDSP1;
                lutInst = pa_LUT1_INST_1_1;
                l3Entry2.u.ipInfo.spi = ipInfo->spi;
                l3Entry.u.ipInfo.spi  = 0;
                fl3Entry2 = TRUE;
                
            }
            break;
            
        case pa_LUT1_INST_2:
            if (ipInfo->spi)
            {
                ret = pa_ERR_CONFIG;
                goto Pa_addIp2_end;
            }
            l3Entry.hdr.pdspNum = PASS_INGRESS4_PDSP0;
            lutInst = pa_LUT1_INST_4_0;
            *cmdDest            = pa_CMD_TX_DEST_4;
            break;
            
        /* No default check since lutInst is already verified */     
    }
  
  }
  else if ((prevLink == NULL) || 
           (hdr->type == PA_TABLE_ENTRY_TYPE_L2) || 
           ((hdr->type == PA_TABLE_ENTRY_TYPE_VL) && (hdr->subType == PA_TABLE_ENTRY_SUBTYPE_VLINK_MAC))) {

    l3Entry.hdr.pdspNum = PASS_INGRESS1_PDSP0;
    *cmdDest            = pa_CMD_TX_DEST_1;
    lutInst = pa_LUT1_INST_1_0;
    if (ipInfo->spi)
    {
        /* The entry contains both IP and IPSEC info, need to create two entries (IP and corresonding IPSEC) */
        /* The IP entry must specify the protocol type so that it csn be distinguished  from IP only entry */
        if ((ipInfo->proto != IP_PROTO_ESP) && (ipInfo->proto != IP_PROTO_AUTH))
        {
            ret = pa_ERR_CONFIG;
            goto Pa_addIp2_end;
        }
        memset (&l3Entry2, 0, sizeof(paL3Entry_t));
        l3Entry2.hdr.pdspNum = PASS_INGRESS1_PDSP1;
        l3Entry2.u.ipInfo.spi  = ipInfo->spi;
        l3Entry.u.ipInfo.spi  = 0;
        fl3Entry2 = TRUE;
    }

  }  else  {

    /* It will either Inner IP or ESP/AH over IP */
    if ((hdr->subType == PA_TABLE_ENTRY_SUBYTPE_IPSEC) ||
        (hdr->subType == PA_TABLE_ENTRY_SUBTYPE_VLINK_OUTER_IP)) {
       if (fNonIp)
       {
            /* It must be an IPSEC ESP over AH traffic */
            l3Entry.hdr.pdspNum = PASS_INGRESS2_PDSP0;
            *cmdDest            = pa_CMD_TX_DEST_2;
            lutInst = pa_LUT1_INST_2_0;
       }
       else
       {
            l3Entry.hdr.pdspNum = PASS_INGRESS4_PDSP0;
            *cmdDest            = pa_CMD_TX_DEST_4;
            lutInst = pa_LUT1_INST_4_0;
       }
    }  else  {
       if (fNonIp && ipInfo->spi)
       {
            /* IP/IPSEC */
            firstPdsp = 1;
            l3Entry.hdr.pdspNum = PASS_INGRESS1_PDSP1;
            *cmdDest            = pa_CMD_TX_DEST_1;
            lutInst = pa_LUT1_INST_1_1;
       }
       else if (!fNonIp && (ipInfo->spi == 0))
       {
            l3Entry.hdr.pdspNum = PASS_INGRESS4_PDSP0;
            *cmdDest            = pa_CMD_TX_DEST_4;
            lutInst = pa_LUT1_INST_4_0;
       }
       else
       {
            /* There should be no IP/IPSEC traffic over IP */
            ret = pa_ERR_CONFIG;
            goto Pa_addIp2_end;
       }
    }

  }
  
  if (paLObj.cfg.rmServiceHandle) {    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, (int32_t *)&lutInst, NULL)) {
      ret = pa_RESOURCE_USE_DENIED;
      goto Pa_addIp2_end;
    }
	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, (int32_t *) &lutInst, NULL)) {
      ret = pa_RESOURCE_FREE_DENIED;
	  goto Pa_addIp2_end;
    }	
  }  
  
  /* Sanity check the command buffer */
  csize = fl3Entry2? sizeof(pafrmCommand_t)+2*sizeof(pafrmCommandAddLut1_t)+2*sizeof(pafrmCommandCmdHdr_t)-sizeof(uint32_t):
                     sizeof(pafrmCommand_t)+sizeof(pafrmCommandAddLut1_t)-sizeof(uint32_t); 
  if (*cmdSize < csize)
  {
    ret = pa_INSUFFICIENT_CMD_BUFFER_SIZE;
    goto Pa_addIp2_end;
  }  

  /* Return the actual size of the buffer used */
  *cmdSize = csize;

  /* Replace the index might have been requested 
   * Check if that is the case
   */
  if (params->validBitMap & pa_PARAM_VALID_CTRLBITMAP) {
    /* Check if replace command is set */
    if (params->ctrlBitMap & pa_PARAM_CTRL_REPLACE) {        
      ret = pa_API_UNSUPPORTED;
      goto Pa_addIp2_end;          
    }
  }

  /* Look for an identical entry in the table. If one is found, return it */
  /* perform entry check only if the LUT1 index is not specified by user */
  /* TBD: we may not be able to support user-sepcified index for IPSEC use case since two differnt indexes 
          are required at two LUT1 table */
  if (lut1Index == PAFRM_LUT1_INDEX_LAST_FREE) {
    for (i = 0; i < paInst->nL3; i++)  {

      if ( ((l3Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (l3Table[i].hdr.status == PA_TBL_STAT_ACTIVE)               ) &&
           ((l3Table[i].hdr.subType == PA_TABLE_ENTRY_SUBYTPE_IP)      ||
            (l3Table[i].hdr.subType == PA_TABLE_ENTRY_SUBYTPE_IP_IPSEC)||
            (l3Table[i].hdr.subType == PA_TABLE_ENTRY_SUBYTPE_IPSEC)    ))  {

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
              fIpEntryFound = TRUE;
              if (!fl3Entry2)
              {
                ret = pa_DUP_ENTRY; 
                goto Pa_addIp2_send_cmd;
              }
              else
                break;  
            }                  
          }
        }
      }
    }  
  }  
  /* Looks for IPSEC entries only if lut1Index is found */
  if (fl3Entry2 && (lut1Index != PAFRM_LUT1_INDEX_LAST_FREE))
  {
    for (j = 0; j < paInst->nL3; j++)  {
    
        if (((l3Table[j].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (l3Table[j].hdr.status == PA_TBL_STAT_ACTIVE)               ) &&
            (l3Table[j].hdr.subType == PA_TABLE_ENTRY_SUBYTPE_IPSEC      ))  {

            if (!memcmp (&l3Table[j].u.ipInfo, &l3Entry2.u.ipInfo, sizeof(paIpInfo_t)))  { 
	            paLnkHandle_t pHandle_loc = NULL;
                if (l3Table[j].pHandle)
		  	        pHandle_loc  = (paLnkHandle_t)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3Table[j].pHandle);
                if (pHandle_loc == &l3Table[i])  {

                    if (l3Table[j].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) {
                        *retHandle = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&(l3Table[j]));
                        ret = pa_INVALID_DUP_ENTRY;
                        goto Pa_addIp2_end;
                               
                    }                     
                    else {
                    /*  
                    * Identical entry is identified, reuse the same table entry
                    * Keep the entry status since it simply replaces the original entry
                    * and the user may decide not to forward the packet
                    */ 
                        lut1Index2 = l3Table[j].hdr.lutIdx;
                        fIpsecEntryFound = TRUE;
                        ret = pa_DUP_ENTRY;   
                        goto Pa_addIp2_send_cmd;
                    }                  
                }
            }
        }
    }
  }
  
  /* Find a free entry in the table */
  if (!fIpEntryFound)
  {
    for (i = 0; i < paInst->nL3; i++)  {

        if (l3Table[i].hdr.status == PA_TBL_STAT_INACTIVE)
        break;
    }
  
    if (i == paInst->nL3)  {
        ret = pa_HANDLE_TABLE_FULL;
        goto Pa_addIp2_end;
    }
  }
  
  /* Find a free entry in the table */
  if (fl3Entry2 && (!fIpsecEntryFound))
  {
    for (j = i+1; j < paInst->nL3; j++)  {

        if (l3Table[j].hdr.status == PA_TBL_STAT_INACTIVE)
        break;
    }

    if (j == paInst->nL3)  {
        ret = pa_HANDLE_TABLE_FULL;
        goto Pa_addIp2_end;
    }
  }
  
Pa_addIp2_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *retHandle = fl3Entry2? (paHandleL2L3_t)&l3Table[j]:&l3Table[i];
  *retHandle = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,*retHandle);

  /* Create the command */                            
  if (fl3Entry2)
  {
    fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_MULTI_CMDS, 0, 0, csize);
    cmdHdr = (pafrmCommandCmdHdr_t *) &(fcmd->cmd);
    cmdHdr->command = PAFRM_CONFIG_COMMAND_ADDREP_LUT1;
    cmdHdr->offset =  sizeof(pafrmCommand_t)+sizeof(pafrmCommandAddLut1_t)+sizeof(pafrmCommandCmdHdr_t)-sizeof(uint32_t);
    cmdHdr->offset = SWIZ(cmdHdr->offset);
    cmdHdr->comId = PA_COMID_L3  | i;
    cmdHdr->comId = SWIZ(cmdHdr->comId );
    al1 =   (pafrmCommandAddLut1_t *)((uint8_t *)cmdHdr + sizeof(pafrmCommandCmdHdr_t));
   
  }
  else
  {
    fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_L3  | i, firstPdsp, csize);
    al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);
  } 

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;
      
  priority      = 0; 
  
  al1->index = SWIZ(lut1Index);

  /* When next link is set 
   * Then set the flag to notify firmware that a virtual link will be used 
   * for the next stage. */
  if ((!fl3Entry2) && (nextLink != NULL)){
    al1->type         = PAFRM_COM_ADD_LUT1_VLINK;
    al1->vLinkNum     = SWIZ(vLinkNum);
  }  else {
    al1->type         = PAFRM_COM_ADD_LUT1_STANDARD;
  }
  
  if (fNonIp)
  {
    /* IPSEC only entry */
    CBWords0 = (uint32_t)PAFRM_LUT1_CLASS_IPSEC << PAFRM_LUT1_CLASS_SHIFT;
    CBWords0 |= (PAFRM_LUT1_VALID_IPSEC_SPI | PAFRM_LUT1_VALID_IPSEC_PKTTYPE);     
    CBWords1  = PAFRM_LUT1_VALID_IPSEC_SRC_VC;
    
    al1->u.ipsec.pktType = PAFRM_L3_PKT_TYPE_IPSEC;
    al1->u.ipsec.srcVC = SWIZ(srcVC);
    al1->u.ipsec.spi   = SWIZ(l3Entry.u.ipInfo.spi);
    priority = 30;
  }
  else if (l3Entry.u.ipInfo.ipType == pa_IPV4)
  {    
    /* IPV4 entry */    
    CBWords0 = (uint32_t)(PAFRM_LUT1_CLASS_IPV4 << PAFRM_LUT1_CLASS_SHIFT) | PAFRM_LUT1_VALID_IPV4_PKTFLAGS;
    CBWords1 = PAFRM_LUT1_VALID_IPV4_PKTTYPE;

    al1->u.ipv4.pktType   = PAFRM_L3_PKT_TYPE_IP;
    al1->u.ipv4.pktFlags  = PAFRM_IP_FLAG_V4;
    
    if (prevLink != NULL)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_SRC_VC;
        al1->u.ipv4.srcVC = SWIZ(srcVC);
        priority += 10;
    }

    /* The IP type has already been validated as either IPv4 or IPv6. IPv4
     * addresses have been pre-padded with 0 to be the same size as IPv6 */
    if (ipInfo->validBitMap & pa_IP_INFO_VALID_SRC)
    {
      CBWords0 |= (32 << PAFRM_LUT1_IPV4_SUBNET_SIP_SHIFT);
      memcpy(&al1->u.ipv4.srcIp,  &(l3Entry.u.ipInfo.src.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
      priority += 10;
    }  

    if (ipInfo->validBitMap & pa_IP_INFO_VALID_DST)
    {
      CBWords0 |= (32 << PAFRM_LUT1_IPV4_SUBNET_DIP_SHIFT);
      memcpy(&al1->u.ipv4.dstIp,  &(l3Entry.u.ipInfo.dst.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
      priority += 10;
    }  

    if (ipInfo->validBitMap & pa_IP_INFO_VALID_GREPROTO)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_DST_PORT;
        
        al1->u.ipv4.dstPort = SWIZ(l3Entry.u.ipInfo.greProto);
        al1->u.ipv4.pktFlags |= PAFRM_IP_FLAG_GRE;
        priority += 15;
    }
    else if (ipInfo->validBitMap & pa_IP_INFO_VALID_SCTPPORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_DST_PORT;
        
        al1->u.ipv4.dstPort = SWIZ(l3Entry.u.ipInfo.sctpPort);
        al1->u.ipv4.pktFlags |= PAFRM_IP_FLAG_SCTP;
        priority += 15;
    }
    
    if (ipInfo->validBitMap & pa_IP_INFO_VALID_PROTO)
    {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_PROTO;
        al1->u.ipv4.protocol = SWIZ(l3Entry.u.ipInfo.proto);
        priority += 10;
    }  

    if (ipInfo->validBitMap & pa_IP_INFO_VALID_TOS)
    {
        CBWords0 |= PAFRM_LUT1_VALID_IPV4_DSCP;
        al1->u.ipv4.dscp = SWIZ(l3Entry.u.ipInfo.tos);
        priority += 10;
    }  
    
    if (fl3Entry2)
    {
        CBWords0 |= PAFRM_LUT1_VALID_IPV4_PKTFLAGS;
        al1->u.ipv4.pktFlags |= PAFRM_IP_FLAG_IPSEC;
    }
    
    al1->u.ipv4.pktFlags = SWIZ(al1->u.ipv4.pktFlags);
    al1->bitMask =  al1->u.ipv4.pktFlags;
  }
  else
  {    
    /* IPV6 entry */    
    CBWords0 = (uint32_t)(PAFRM_LUT1_CLASS_IPV6 << PAFRM_LUT1_CLASS_SHIFT) | PAFRM_LUT1_VALID_IPV6_PKTFLAGS;
    CBWords1 = PAFRM_LUT1_VALID_IPV6_PKTTYPE;

    al1->u.ipv6.pktType     = PAFRM_L3_PKT_TYPE_IP;

    /* Ethertype holds the linked PDSP ID */
    /* vlan holds the linked PDSP LUT index */
    if (prevLink != NULL)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_SRC_VC;
        al1->u.ipv6.srcVC = SWIZ(srcVC);
    }

    /* The IP type has already been validated as either IPv4 or IPv6. IPv4
     * addresses have been pre-padded with 0 to be t`he same size as IPv6 */

    if (ipInfo->validBitMap & pa_IP_INFO_VALID_SRC)
    {
      CBWords0 |= (128 << PAFRM_LUT1_IPV6_SUBNET_SIP_SHIFT);
      memcpy(&al1->u.ipv6.srcIp0,  &(l3Entry.u.ipInfo.src.ipv6), sizeof(paIpv6Addr_t));
      priority += 10;
    }  

    if (ipInfo->validBitMap & pa_IP_INFO_VALID_DST)
    {
      CBWords0 |= (128 << PAFRM_LUT1_IPV6_SUBNET_DIP_SHIFT);
      memcpy(&al1->u.ipv6.dstIp0,  &(l3Entry.u.ipInfo.dst.ipv6), sizeof(paIpv6Addr_t));
      priority += 10;
    }  

    if (ipInfo->validBitMap & pa_IP_INFO_VALID_GREPROTO)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_DST_PORT;
        
        al1->u.ipv6.dstPort = SWIZ(l3Entry.u.ipInfo.greProto);
        al1->u.ipv6.pktFlags = PAFRM_IP_FLAG_GRE;
        priority += 15;
    }
    else if (ipInfo->validBitMap & pa_IP_INFO_VALID_SCTPPORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_DST_PORT;
        
        al1->u.ipv6.dstPort = SWIZ(l3Entry.u.ipInfo.sctpPort);
        al1->u.ipv6.pktFlags = PAFRM_IP_FLAG_SCTP;
        priority += 15;
    }
    
    if (ipInfo->validBitMap & pa_IP_INFO_VALID_PROTO)
    {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_PROTO;
        al1->u.ipv6.protocol = SWIZ(l3Entry.u.ipInfo.proto);
        priority += 10;
    }  

    if (ipInfo->validBitMap & pa_IP_INFO_VALID_TOS)
    {
        CBWords0 |= PAFRM_LUT1_VALID_IPV6_DSCP;
        al1->u.ipv6.dscp = SWIZ(l3Entry.u.ipInfo.tos);
        priority += 10;
    }  
    
    if (ipInfo->validBitMap & pa_IP_INFO_VALID_FLOW)
    {
        CBWords0 |= PAFRM_LUT1_VALID_IPV6_FLOWLABEL;
        al1->u.ipv6.flowLabel = SWIZ(l3Entry.u.ipInfo.flow);
        priority += 10;
    } 
    
    if (fl3Entry2)
    {
        al1->u.ipv6.pktFlags |= PAFRM_IP_FLAG_IPSEC;
    }
    
    al1->bitMask =  al1->u.ipv6.pktFlags | PAFRM_IP_FLAG_IP_TYPE;
    al1->u.ipv6.pktFlags = SWIZ(al1->u.ipv6.pktFlags);
    al1->bitMask =  SWIZ(al1->bitMask);
  }
  
  al1->CBWords0 = SWIZ(CBWords0);
  al1->CBWords1 = SWIZ(CBWords1); 
  al1->priority = SWIZ(priority);  
  
  if (fl3Entry2)
  {
    mRoute  = (paRouteInfo2_t *)&pauForwardLut1Route;
    nfRoute = (paRouteInfo2_t *)&pauFailRoute;
  }
  else
  {
    mRoute = routeInfo;
    nfRoute = nextRtFail;
  }
  
  ret1 = pa_conv_routing_info2(paInst, &al1->match, mRoute, *cmdDest, FALSE, firstPdsp, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
      ret = ret1;
      goto Pa_addIp2_end;
  }

  /* Next fail information */
  ret1 = pa_conv_routing_info2(paInst, &al1->nextFail, nfRoute, *cmdDest, TRUE, firstPdsp, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
      ret = ret1;
      goto Pa_addIp2_end;
  }

  if (!fIpEntryFound)
  {
    /* Add the status and pdsp NUM */
    l3Entry.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
    if (prevLink != NULL)
      l3Entry.pHandle    = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr, prevLink);
    l3Entry.hdr.type     = PA_TABLE_ENTRY_TYPE_L3;
    if (fl3Entry2)
    {
        l3Entry.hdr.subType = PA_TABLE_ENTRY_SUBYTPE_IP_IPSEC;
    }
    else
    {
        l3Entry.hdr.subType = fNonIp?PA_TABLE_ENTRY_SUBYTPE_IPSEC:
                                     PA_TABLE_ENTRY_SUBYTPE_IP; 
        if (nextLink != NULL)
            l3Entry.nHandle      = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr, nextLink);
    }              
                      
    l3Entry.hdr.tableIdx = i;
    l3Entry.hdr.lutIdx   = -1;
  
    memcpy (&l3Table[i], &l3Entry, sizeof(paL3Entry_t));
  }
  
  if (fl3Entry2)
  {
    /* Processing the second command */
    cmdHdr = (pafrmCommandCmdHdr_t *) ((uint8_t *)al1 + sizeof(pafrmCommandAddLut1_t));
    cmdHdr->command = PAFRM_CONFIG_COMMAND_ADDREP_LUT1;
    cmdHdr->offset = 0;
    cmdHdr->comId = PA_COMID_L3  | j;
    cmdHdr->comId = SWIZ(cmdHdr->comId );
    
    al1 =   (pafrmCommandAddLut1_t *)((uint8_t *)cmdHdr + sizeof(pafrmCommandCmdHdr_t));
    
    al1->index = SWIZ(lut1Index2);
    
    /* When next link is set 
     * Then set the flag to notify firmware that a virtual link will be used 
     * for the next stage. */
    if (nextLink != NULL){
      al1->type         = PAFRM_COM_ADD_LUT1_VLINK;
      al1->vLinkNum     = SWIZ(vLinkNum);
    }  else {
      al1->type         = PAFRM_COM_ADD_LUT1_STANDARD;
    }
    
    /* IPSEC only entry */
    CBWords0 = (uint32_t)PAFRM_LUT1_CLASS_IPSEC << PAFRM_LUT1_CLASS_SHIFT;
    CBWords0 |= (PAFRM_LUT1_VALID_IPSEC_SPI | PAFRM_LUT1_VALID_IPSEC_PKTTYPE);     
    CBWords1  = PAFRM_LUT1_VALID_IPSEC_SRC_VC;
    
    al1->u.ipsec.pktType = PAFRM_L3_PKT_TYPE_IPSEC;
    al1->u.ipsec.srcVC = PAFRM_MK_SRC_VC(l3Entry.hdr.pdspNum, 0);
    al1->u.ipsec.srcVC = SWIZ(al1->u.ipsec.srcVC);
    al1->u.ipsec.spi   = SWIZ(l3Entry2.u.ipInfo.spi);
    priority = 30;
    
    al1->CBWords0 = SWIZ(CBWords0);
    al1->CBWords1 = SWIZ(CBWords1); 
    al1->priority = SWIZ(priority);  
    
    ret1 = pa_conv_routing_info2(paInst, &al1->match, routeInfo, *cmdDest, FALSE, 1, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
    if (ret1 != pa_OK) {
        ret = ret1;
        goto Pa_addIp2_end;
    }

    /* Next fail information */
    ret1 = pa_conv_routing_info2(paInst, &al1->nextFail, nextRtFail, *cmdDest, TRUE, 1, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
    if (ret1 != pa_OK) {
        ret = ret1;
        goto Pa_addIp2_end;
    }
    
    if (!fIpsecEntryFound)
    {
      /* Add the status and pdsp NUM */
      l3Entry2.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
      l3Entry2.pHandle      = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&(l3Table[i]));
      if (nextLink != NULL)
          l3Entry2.nHandle  = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr, nextLink);
      l3Entry2.hdr.type     = PA_TABLE_ENTRY_TYPE_L3;
      l3Entry2.hdr.subType  = PA_TABLE_ENTRY_SUBYTPE_IPSEC;
      l3Entry2.hdr.tableIdx = j;
      l3Entry2.hdr.lutIdx   = -1;
  
      memcpy (&l3Table[j], &l3Entry2, sizeof(paL3Entry_t));
    }
  
  }
  
Pa_addIp2_end:  

  if (l2Release) Pa_osalEndMemAccess ((void *) l2Table,
                                      paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)
    Pa_osalEndMemAccess((void *)vlnkTable,
                         paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  Pa_osalEndMemAccess ((void *) l3Table,
                       paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);

} /* Pa_addIp2 */

/*******************************************************************************************
 * FUNCTION PURPOSE: Add an Ethernet OAM entry to the Ethernet OAM table
 *******************************************************************************************
 * DESCRIPTION: The Ethernet OAM entry is verified and added to the EOAM table. 
 *              The following LUT1 are used to store EOAM related entries at NetCP 1.5:
 *                  - Ingess 0, PDSP1, LUT1_1
 *              This API is not supported for NetCP 1.0
 *                
 ********************************************************************************************/
  paReturn_t Pa_addEoamFlow  (Pa_Handle         iHandle,
                              paEthInfo2_t      *ethInfo,    
                              paEoamFlowInfo_t  *eoamInfo,   
                              paHandleEoam_t    *handle,     
                              paCmd_t           cmd,
                              uint16_t          *cmdSize,
                              paCmdReply_t      *reply,
                              int               *cmdDest
                             )
{
  /* The entry is created in the stack and then copied to the table. This allows
   * for a comparison with other table entries to verify there is not an identical
   * entry or an entry that would supercede this one */

  paInst_t               *paInst    = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paEoamEntry_t          *eoamTable;

  paEoamEntry_t           eoamEntry;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  int                     i;
  uint16_t                csize;
  uint16_t                priority, bitMask = 0;
  paReturn_t              ret = pa_OK;
  uint32_t                mtCsKey, CBWords0, CBWords1; 
  uint16_t                lut1Index = PAFRM_LUT1_INDEX_LAST_FREE;
  
  if((ethInfo == NULL) || (eoamInfo == NULL) || (handle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  /* Check if global eoam configuration is done before adding any entry to the table */
  if (PA_GET_STATE_EOAM(paInst) == 0)
     return (pa_LUT_ENTRY_FAILED);

  if (paLObj.cfg.rmServiceHandle) {
    int32_t lutInst  = pa_LUT1_INST_0_1;    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &lutInst, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }

	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &lutInst, NULL)) {
      return pa_RESOURCE_FREE_DENIED;
    }
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
  memset (&eoamEntry, 0, sizeof(paL2Entry_t));
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_DST)
    memcpy (eoamEntry.cfg.mac.dstMac,  ethInfo->dst,  sizeof(paMacAddr_t));
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_SRC)
    memcpy (eoamEntry.cfg.mac.srcMac,  ethInfo->src,  sizeof(paMacAddr_t));
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN)
    eoamEntry.cfg.mac.vlan     =  ethInfo->vlan;
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_ETHERTYPE)
    eoamEntry.cfg.mac.ethertype  =  ethInfo->ethertype;
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_MPLSTAG)
    eoamEntry.cfg.mac.mplsTag    =  ethInfo->mplsTag;
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_INPORT)
    eoamEntry.cfg.mac.inport     =  ethInfo->inport;
  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN_PRI)
    eoamEntry.cfg.mac.vlanPri    =  ethInfo->vlanPri;
    
  /* The MPLS label is restricted to 20 bits */
  if (eoamEntry.cfg.mac.mplsTag & ~PA_MPLS_LABEL_MASK)  
     return(pa_INVALID_MPLS_LABEL);
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  eoamTable	= (paEoamEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].base);	

  /* Signal the application that a table modification will be done */
  Pa_osalBeginMemAccess ((void*) eoamTable, 
					   paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);

  /* Look for an identical entry in the table. If one is found, return it */
  /* perform entry check only if the LUT1 index is not specified by user */
  if (lut1Index == PAFRM_LUT1_INDEX_LAST_FREE) {
    for (i = 0; i < paInst->nEoam;  i++)  {

      if ( ((eoamTable[i].hdr.status  == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (eoamTable[i].hdr.status  == PA_TBL_STAT_ACTIVE)             ) &&
            (eoamTable[i].hdr.subType == PA_TABLE_ENTRY_SUBTYPE_MAC )) {

        if (!memcmp (&(eoamTable[i].cfg), &(eoamEntry.cfg), sizeof(paL2InCfg_t)))  {
          /* Identical entry identified */
          if (eoamTable[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) {
            *handle = (paHandleEoam_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&eoamTable[i]);
            ret = pa_INVALID_DUP_ENTRY;
            goto Pa_addEoamFlow_end;
          }                     
          else {
              /*  
               * Identical entry is identified, reuse the same table entry
               * Keep the entry status since it simply replaces the original entry
               * and the user may decide not to forward the packet
               */ 
            lut1Index = eoamTable[i].hdr.lutIdx;
            ret = pa_DUP_ENTRY;   
            goto Pa_addEoamFlow_send_cmd;
          }                  
        }
      }
    }
  }

  /* Find a free entry in the table */
  for (i = 0; i < paInst->nEoam; i++)  {

    if (eoamTable[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;

  }

  if (i == paInst->nEoam)  {
    ret = pa_HANDLE_TABLE_FULL;
    goto Pa_addEoamFlow_end;
  }
  
Pa_addEoamFlow_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *handle = (paHandleEoam_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&eoamTable[i]);

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_EOAM  | i, 1, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);
  
  al1->index    = SWIZ(lut1Index);  
  al1->type     = PAFRM_COM_ADD_LUT1_STANDARD;
    
  CBWords0      = PAFRM_LUT1_CLASS_STANDARD << PAFRM_LUT1_CLASS_SHIFT;
  CBWords1      = PAFRM_LUT1_VALID_PKTTYPE;
  priority      = 0; 
  
  al1->u.mac.pktType     = PAFRM_L2_PKT_TYPE_MAC;

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_DST)
  {
    memcpy (al1->u.mac.dmac, eoamEntry.cfg.mac.dstMac, sizeof(paMacAddr_t));
    CBWords0 |=  PAFRM_LUT1_VALID_DMAC_ALL;
    priority += 10;
  }  

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_SRC)
  {
    memcpy (al1->u.mac.smac, eoamEntry.cfg.mac.srcMac, sizeof(paMacAddr_t));
    CBWords0 |=  PAFRM_LUT1_VALID_SMAC;
    priority += 10;
  }  

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_ETHERTYPE)
  {
    al1->u.mac.etherType = SWIZ(eoamEntry.cfg.mac.ethertype);
    CBWords0 |=  PAFRM_LUT1_VALID_ETHERTYPE;
    priority += 10;
    
  }
  
  if (ethInfo->validBitMap & pa_ETH_INFO_VALID_MPLSTAG) 
  {
    al1->u.mac.mpls     = SWIZ(eoamEntry.cfg.mac.mplsTag);
    CBWords0 |=  PAFRM_LUT1_VALID_MPLS;
    priority += 10;
  }

  if(ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN)
  {
    al1->u.mac.vlanId1  = SWIZ(eoamEntry.cfg.mac.vlan);
    CBWords1 |=  PAFRM_LUT1_VALID_VLANID1;
    priority += 10;
    
  }  
    
  if (ethInfo->validBitMap & pa_ETH_INFO_VALID_INPORT)
  {
    al1->u.mac.inport = SWIZ(eoamEntry.cfg.mac.inport);
    CBWords1 |=  PAFRM_LUT1_VALID_INPORT;
    priority += 10;
  }

  if (ethInfo->validBitMap & pa_ETH_INFO_VALID_VLAN_PRI)
  {
    al1->u.mac.vlanPri1 = SWIZ(eoamEntry.cfg.mac.vlanPri);
    CBWords1 |=  PAFRM_LUT1_VALID_VLAN_PRI1;
    priority += 10;
  }  
  
  al1->CBWords0   = SWIZ(CBWords0);
  al1->CBWords1   = SWIZ(CBWords1); 
  al1->priority   = SWIZ(priority);
  al1->bitMask    = SWIZ(bitMask);

  /* Forwarding information */
  al1->match.forwardType        = PAFRM_FORWARD_TYPE_EOAM;
  al1->match.flowId             = SWIZ(eoamInfo->flowId);
  al1->match.queue              = SWIZ(eoamInfo->destQueue); 
  al1->match.u.eoam.context     = SWIZ(eoamInfo->swInfo0);
  al1->match.u.eoam.megLevel    = SWIZ(eoamInfo->megLevel);
  al1->match.u.eoam.statsIndex  = SWIZ(eoamInfo->statsIndex);

  if (ret != pa_DUP_ENTRY)
  {
    /* Initialze, the table entry, add the status and pdsp NUM */
    eoamEntry.hdr.type     = PA_TABLE_ENTRY_TYPE_EOAM;
    eoamEntry.hdr.subType  = PA_TABLE_ENTRY_SUBTYPE_MAC;
    eoamEntry.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
    eoamEntry.hdr.pdspNum  = PASS_INGRESS0_PDSP1;
    eoamEntry.hdr.lutIdx   = -1;
    eoamEntry.hdr.tableIdx = i;

    memcpy (&eoamTable[i], &eoamEntry, sizeof(paL2Entry_t));
  }

  /* The destination for this command must be PDSP 1 */
  *cmdDest = pa_CMD_TX_DEST_0;
  
Pa_addEoamFlow_end:
  Pa_osalEndMemAccess ((void *) eoamTable,
                       paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  
  
  return (ret);

}

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

  /* The entry is created in the stack and then copied to the table. This allows
   * for a comparison with other table entries to verify there is not an identical
   * entry */
  paInst_t               *paInst   = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paAclEntry_t           *aclTable = (paAclEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_ACL_TABLE].base);
  paAclEntry_t            aclEntry;
  paL2L3Header_t         *hdr = NULL;
  paL2L3Header_t         *hdr1;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  int                     i, insertMode;
  uint16_t                csize;
  uint16_t                priority, l2Release, l3Release;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey, CBWords0, CBWords1, remainingEntries;
  paAclRescoreHdr_t      *rescoreHdr;
  paRouteInfo2_t         *mRoute, *nfRoute;
  uint8_t                 firstPdsp, paFlags;
  uint8_t                 subnetSize;
  paL2Entry_t 		     *l2Table	      = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);
  paL3Entry_t 		     *l3Table	      = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  paAclLinkListElem_t  *listElemTable = (paAclLinkListElem_t *) PA_ACL_LINK_LIST_ELEM_TABLE(paInst->nAcl, aclTable);
  paAclLinkListInfo_t  *listInfoTable;
  paAclRescoreList_t   *rescoreMem;
  
  if((aclInfo == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }

  /* ACL Outer IP resource can not be used when EOAM is there in the system */
  if ( (aclInst == pa_ACL_INST_OUTER_IP) &&
       (PA_GET_STATE_EOAM(paInst) == 1) )
  {
    /* LUT1 resource for Outer ACL can not be used as it is used for EOAM classification */
    return (pa_RESOURCE_USE_DENIED);
  }
  
  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+sizeof(pafrmCommandAddLut1_t)-sizeof(uint32_t); 
  if (*cmdSize < csize)
  {
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);
  }  

  /* Return the actual size of the buffer used */
  *cmdSize = csize;
  
  /* Sanity check the ACL instance,  ACL action */
  if ((aclInst != pa_ACL_INST_OUTER_IP) && (aclInst != pa_ACL_INST_INNER_IP)) {
    return(pa_INVALID_LUT_INST);
  }
  
  if ((aclAction != pa_ACL_ACTION_PERMIT) && (aclAction != pa_ACL_ACTION_DENY)) {
    return(pa_INVALID_ACL_ACTION);
  }
  
  /* Sanity check the command reply information.  */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);
    
  prevLink  = (prevLink)?((paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,prevLink)):NULL;
  nextEntry = (nextEntry)?((paAclEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,nextEntry)):NULL;
  
  /* Create the entry on the stack, and zero the entire entry. This will
   * allow simple compare of entries using memcmp even though some fields
   * will be unused, or there will be gaps between fields */
  memset (&aclEntry, 0, sizeof(paAclEntry_t));
  
  aclEntry.aclInfo.validBitMap = aclInfo->validBitMap;
  aclEntry.aclInfo.ipType      = aclInfo->ipType;
  
  /* For IPv4, copy the 4 address bytes to the last 4 bytes in an IPv6 sized byte array */
  if (aclInfo->ipType == pa_IPV4)  {
    if(aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_IP)
        memcpy (&(aclEntry.aclInfo.srcIp.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                aclInfo->srcIp.ipv4, sizeof(paIpv4Addr_t));
                
    if(aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_IP_MASK)
        memcpy (&(aclEntry.aclInfo.srcIpMask.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                aclInfo->srcIpMask.ipv4, sizeof(paIpv4Addr_t));
                
    if(aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_IP)
        memcpy (&(aclEntry.aclInfo.dstIp.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                aclInfo->dstIp.ipv4, sizeof(paIpv4Addr_t));
                
    if(aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_IP_MASK)
        memcpy (&(aclEntry.aclInfo.dstIpMask.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                aclInfo->dstIpMask.ipv4, sizeof(paIpv4Addr_t));

  }  else if (aclInfo->ipType == pa_IPV6)  {
    if(aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_IP)
        memcpy (&aclEntry.aclInfo.srcIp.ipv6, aclInfo->srcIp.ipv6, sizeof(paIpv6Addr_t));
        
    if(aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_IP_MASK)
        memcpy (&aclEntry.aclInfo.srcIpMask.ipv6, aclInfo->srcIpMask.ipv6, sizeof(paIpv6Addr_t));
        
    if(aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_IP)
        memcpy (&aclEntry.aclInfo.dstIp.ipv6, aclInfo->dstIp.ipv6, sizeof(paIpv6Addr_t));
        
    if(aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_IP_MASK)
        memcpy (&aclEntry.aclInfo.dstIpMask.ipv6, aclInfo->dstIpMask.ipv6, sizeof(paIpv6Addr_t));
  }  else
    return (pa_ERR_CONFIG);
    
  if (aclInfo->validBitMap & pa_ACL_INFO_VALID_CTRL_FLAG)
  {
    aclEntry.aclInfo.ctrlFlag       =  aclInfo->ctrlFlag & aclInfo->ctrlFlagMask;
    aclEntry.aclInfo.ctrlFlagMask   =  aclInfo->ctrlFlagMask;
  }
  
  if (aclInfo->validBitMap & pa_ACL_INFO_VALID_PROTO)
  {
    aclEntry.aclInfo.proto      =  aclInfo->proto;
  }
  
  if (aclInfo->validBitMap & pa_ACL_INFO_VALID_DSCP)
  {
    aclEntry.aclInfo.dscp       =  aclInfo->dscp;
  }
  
  if (aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_PORT)
  {
    aclEntry.aclInfo.srcPortBegin   =  aclInfo->srcPortBegin;
    aclEntry.aclInfo.srcPortEnd     =  aclInfo->srcPortEnd;
    
    if(aclInfo->srcPortEnd < aclInfo->srcPortBegin)
        return (pa_ERR_CONFIG);
  }
  
  if (aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_PORT)
  {
    aclEntry.aclInfo.dstPortBegin   =  aclInfo->dstPortBegin;
    aclEntry.aclInfo.dstPortEnd     =  aclInfo->dstPortEnd;
    
    if(aclInfo->dstPortEnd < aclInfo->dstPortBegin)
        return (pa_ERR_CONFIG);
  }
    
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  /* Signal the application that a table modification will be done. */
  Pa_osalBeginMemAccess (aclTable,
                         paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);
                         
  l2Release = l3Release = FALSE;

  /* If there is a link to the l2 or l3 table then that table must be
   * protected as well.  */
  if (prevLink != NULL)  {

    Pa_osalBeginMemAccess (l2Table,
                           paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
    Pa_osalBeginMemAccess (l3Table,
                           paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);

    l2Release = l3Release = TRUE;
    
    hdr = (paL2L3Header_t *)prevLink;
    /* Must be linked to a complete entry */
    if (hdr->status != PA_TBL_STAT_ACTIVE)  {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addAcl_end;
    }

    if (hdr->type == PA_TABLE_ENTRY_TYPE_L3)  {
      Pa_osalEndMemAccess (l2Table,
                           paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
      l2Release = FALSE;
    }  else if (hdr->type == PA_TABLE_ENTRY_TYPE_L2)  {
      Pa_osalEndMemAccess (l3Table,
                           paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
      l3Release = FALSE;
    } 
    else
    {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addAcl_end;
    }
  }
  
  /* If there is a relative index */
  if (nextEntry != NULL)
  {
    hdr1 = (paL2L3Header_t *)nextEntry;
    
    /* Must be linked to a complete entry */
    if (hdr1->status != PA_TBL_STAT_ACTIVE)  {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addAcl_end;
    }
    
    //priority = hdr1->lutIdx; No longer applicable as ACL entry add is now manual mode 
  }
  else
  {
    //priority = 0xFFFF;  /* insert the entry to the bottom of ACL table */
  }
    
  /* find the command destination and related information */
  if (aclInst == pa_ACL_INST_OUTER_IP)
  {
    aclEntry.hdr.pdspNum = PASS_INGRESS0_PDSP1;
    *cmdDest             = pa_CMD_TX_DEST_0;
    firstPdsp            = 1;
    listInfoTable        = (paAclLinkListInfo_t *) PA_ACL_LINK_LIST_INFO_TABLE(1, paInst->nAcl, aclTable);  
    insertMode           = paInst->cfg.outAclConfig.insertMode;
    rescoreMem           = pa_get_rescore_table_addr(pa_ACL_INST_OUTER_IP);    
    rescoreHdr           = pa_get_rescore_hdr_addr(pa_ACL_INST_OUTER_IP);     
  }
  else
  {
    /* aclInst == pa_CAL_INST_1 */
    aclEntry.hdr.pdspNum = PASS_INGRESS3_PDSP0;
    *cmdDest             = pa_CMD_TX_DEST_3;
    firstPdsp            = 0;
    listInfoTable        = (paAclLinkListInfo_t *) PA_ACL_LINK_LIST_INFO_TABLE(0, paInst->nAcl, aclTable);      
    insertMode           = paInst->cfg.inAclConfig.insertMode;    
    rescoreMem           = pa_get_rescore_table_addr(pa_ACL_INST_INNER_IP);     
    rescoreHdr           = pa_get_rescore_hdr_addr(pa_ACL_INST_INNER_IP);    
  }

  /* Update time to next call this API, for every ACL add entry */
  if (listInfoTable->ctrlBitMap == PA_ACL_LINK_INFO_RESCORING_ACTIVE )  {
    remainingEntries = ( (rescoreHdr->final_cur >> 16) & 0xFFFF) - (rescoreHdr->final_cur & 0xFFFF);
    *timeToNextCall  = remainingEntries >> 2; /* time in micro sends */
  }
  else {
    *timeToNextCall = remainingEntries = 0;
  }

  if (remainingEntries == 0) {
    /* Clear the rescore active state as rescore is complete */
    listInfoTable->ctrlBitMap &= ~PA_ACL_LINK_INFO_RESCORING_ACTIVE;
  }
  else {
    /* Still rescore operation is going on */
    ret = pa_ACL_BUSY;  
	  goto Pa_addAcl_end;    
  }
  
  if (paLObj.cfg.rmServiceHandle) {    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, (int32_t *) &aclInst, NULL)) {
      ret = pa_RESOURCE_USE_DENIED;
      goto Pa_addAcl_end;
    }
	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, (int32_t *) &aclInst, NULL)) {
      ret = pa_RESOURCE_FREE_DENIED;
	  goto Pa_addAcl_end;
    }	
  }  
  
  /* Look for an identical entry in the table. If one is found, return it */
  for (i = 0; i < paInst->nAcl; i++)  {

    if ( (aclTable[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
         (aclTable[i].hdr.status == PA_TBL_STAT_ACTIVE)               )   {
         
      /* If the LUT1 location is different, then the entry is considered different */
      if(aclEntry.hdr.pdspNum != aclTable[i].hdr.pdspNum)
        continue;  

      if (!memcmp (&aclTable[i].aclInfo, &aclEntry.aclInfo, sizeof(paAclInfo_t)))  { 
	    paLnkHandle_t pHandle_loc = NULL;
        if (aclTable[i].pHandle)
			pHandle_loc  = (paLnkHandle_t)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,aclTable[i].pHandle);

        if (pHandle_loc == prevLink)  {
            *retHandle = (paHandleAcl_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&(aclTable[i]));
            ret = pa_INVALID_DUP_ACL_ENTRY;
            goto Pa_addAcl_end;
        }
      }
    }
  }  
  
  
  /* Find a free entry in the table */
  for (i = 0; i < paInst->nAcl; i++)  {

      if (aclTable[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;
  }
  
  if (i == paInst->nAcl)  {
      ret = pa_HANDLE_TABLE_FULL;
      goto Pa_addAcl_end;
  }

  /* Record the last ACL table entry, which may still have its lut Index unknown */
  listInfoTable->lastIdx = i;
  
  /* The handle is just a pointer to the table entry */
  *retHandle =  (paHandleAcl_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&(aclTable[i]));

  /* Create the command */                            
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_ACL  | i, firstPdsp, csize);
  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;
  
  al1->index = PASS_ACL_STATS_BASE_ENTRY(i); /* Used for stats index */
  al1->index = SWIZ(al1->index);
  al1->type  = PAFRM_COM_ADD_LUT1_STANDARD;
      
  if (aclEntry.aclInfo.ipType == pa_IPV4)
  {    
    /* IPV4 entry */    
    CBWords0 = (uint32_t)(PAFRM_LUT1_CLASS_IPV4 << PAFRM_LUT1_CLASS_SHIFT) | PAFRM_LUT1_VALID_IPV4_PKTFLAGS;
    CBWords1 = PAFRM_LUT1_VALID_IPV4_PKTTYPE;

    al1->u.ipv4.pktType   = PAFRM_L3_PKT_TYPE_FIREWALL;
    al1->u.ipv4.pktFlags  = PAFRM_IP_FLAG_V4;
    al1->bitMask |= PAFRM_IP_FLAG_IP_TYPE;
    
    if (prevLink != NULL)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_SRC_VC;
        al1->u.ipv4.srcVC = PAFRM_MK_SRC_VC(hdr->pdspNum, hdr->lutIdx);
        al1->u.ipv4.srcVC = SWIZ(al1->u.ipv4.srcVC);
    }

    /* The IP type has already been validated as either IPv4 or IPv6. IPv4
     * addresses have been pre-padded with 0 to be the same size as IPv6 */

    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_IP)
    {
      if(aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_IP_MASK)
        subnetSize = pa_find_ip_subnet_size (&(aclEntry.aclInfo.srcIpMask.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
      else
        subnetSize = 32;
          
      CBWords0 |= (subnetSize << PAFRM_LUT1_IPV4_SUBNET_SIP_SHIFT);
      memcpy(&al1->u.ipv4.srcIp,  &(aclEntry.aclInfo.srcIp.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
    }  

    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_IP)
    {
      if(aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_IP_MASK)
        subnetSize = pa_find_ip_subnet_size (&(aclEntry.aclInfo.dstIpMask.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
      else
        subnetSize = 32;
    
      CBWords0 |= (subnetSize << PAFRM_LUT1_IPV4_SUBNET_DIP_SHIFT);
      memcpy(&al1->u.ipv4.dstIp,  &(aclEntry.aclInfo.dstIp.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
    }  

    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_PROTO)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_PROTO;
        
        al1->u.ipv4.protocol = SWIZ(aclEntry.aclInfo.proto);
    }
    
    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_DSCP)
    {
        CBWords0 |= PAFRM_LUT1_VALID_IPV4_DSCP;
        al1->u.ipv4.dscp = SWIZ(aclEntry.aclInfo.dscp);
    }  
    
    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_PORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_SRC_PORT;
        if (aclEntry.aclInfo.srcPortBegin != aclEntry.aclInfo.srcPortEnd)
        {
            CBWords0 |= PAFRM_LUT1_IPV4_SRC_PORT_RANGE; 
            al1->range0Hi = SWIZ(aclEntry.aclInfo.srcPortEnd);
        }       
        al1->u.ipv4.srcPort = SWIZ(aclEntry.aclInfo.srcPortBegin);
    }
    
    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_PORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_DST_PORT;
        if (aclEntry.aclInfo.dstPortBegin != aclEntry.aclInfo.dstPortEnd)
        {
            CBWords0 |= PAFRM_LUT1_IPV4_DST_PORT_RANGE; 
            al1->range1Hi = SWIZ(aclEntry.aclInfo.dstPortEnd);
        }       
        al1->u.ipv4.dstPort = SWIZ(aclEntry.aclInfo.dstPortBegin);
    }
    
    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_CTRL_FLAG)  {
        if(aclEntry.aclInfo.ctrlFlagMask & pa_ACL_INFO_CONTROL_FLAG_FRAG)
            al1->bitMask |= PAFRM_IP_FLAG_FRAG;
            
        if(aclEntry.aclInfo.ctrlFlagMask & pa_ACL_INFO_CONTROL_FLAG_CONTAIN_L4)
            al1->bitMask |= PAFRM_IP_FLAG_CONTAIN_L4;
            
        if(aclEntry.aclInfo.ctrlFlag & pa_ACL_INFO_CONTROL_FLAG_FRAG)
            al1->u.ipv4.pktFlags |= PAFRM_IP_FLAG_FRAG;
            
        if(aclEntry.aclInfo.ctrlFlag & pa_ACL_INFO_CONTROL_FLAG_CONTAIN_L4)
            al1->u.ipv4.pktFlags |= PAFRM_IP_FLAG_CONTAIN_L4;
    }
    
    al1->u.ipv4.pktFlags = SWIZ(al1->u.ipv4.pktFlags);
  }
  else
  {    
    /* IPV6 entry */    
    CBWords0 = (uint32_t)(PAFRM_LUT1_CLASS_IPV6 << PAFRM_LUT1_CLASS_SHIFT) | PAFRM_LUT1_VALID_IPV6_PKTFLAGS;
    CBWords1 = PAFRM_LUT1_VALID_IPV6_PKTTYPE;

    al1->u.ipv6.pktType     = PAFRM_L3_PKT_TYPE_FIREWALL;
    
    if (prevLink != NULL)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_SRC_VC;
        al1->u.ipv6.srcVC = PAFRM_MK_SRC_VC(hdr->pdspNum, hdr->lutIdx);
        al1->u.ipv6.srcVC = SWIZ(al1->u.ipv6.srcVC);
    }

    /* The IP type has already been validated as either IPv4 or IPv6. IPv4
     * addresses have been pre-padded with 0 to be the same size as IPv6 */

    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_IP)
    {
      if(aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_IP_MASK)
        subnetSize = pa_find_ip_subnet_size (&(aclEntry.aclInfo.srcIpMask.ipv6[0]), sizeof(paIpv6Addr_t));
      else
        subnetSize = 128;
          
      CBWords0 |= (subnetSize << PAFRM_LUT1_IPV6_SUBNET_SIP_SHIFT);
      memcpy(&al1->u.ipv6.srcIp0,  &(aclEntry.aclInfo.srcIp.ipv6[0]), sizeof(paIpv6Addr_t));
    }  

    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_IP)
    {
      if(aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_IP_MASK)
        subnetSize = pa_find_ip_subnet_size (&(aclEntry.aclInfo.dstIpMask.ipv6[0]), sizeof(paIpv6Addr_t));
      else
        subnetSize = 128;
    
      CBWords0 |= (subnetSize << PAFRM_LUT1_IPV6_SUBNET_DIP_SHIFT);
      memcpy(&al1->u.ipv6.dstIp0,  &(aclEntry.aclInfo.dstIp.ipv6[0]), sizeof(paIpv6Addr_t));
    }  

    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_PROTO)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_PROTO;
        al1->u.ipv6.protocol = SWIZ(aclEntry.aclInfo.proto);
    }
    
    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_DSCP)  {
        CBWords0 |= PAFRM_LUT1_VALID_IPV6_DSCP;
        al1->u.ipv6.dscp = SWIZ(aclEntry.aclInfo.dscp);
    }  
    
    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_SRC_PORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_SRC_PORT;
        if (aclEntry.aclInfo.srcPortBegin != aclEntry.aclInfo.srcPortEnd)
        {
            CBWords0 |= PAFRM_LUT1_IPV6_SRC_PORT_RANGE; 
            al1->range0Hi = SWIZ(aclEntry.aclInfo.srcPortEnd);
        }       
        al1->u.ipv6.srcPort = SWIZ(aclEntry.aclInfo.srcPortBegin);
    }
    
    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_DST_PORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_DST_PORT;
        if (aclEntry.aclInfo.dstPortBegin != aclEntry.aclInfo.dstPortEnd)
        {
            CBWords0 |= PAFRM_LUT1_IPV6_DST_PORT_RANGE; 
            al1->range1Hi = SWIZ(aclEntry.aclInfo.dstPortEnd);
        }       
        al1->u.ipv6.dstPort = SWIZ(aclEntry.aclInfo.dstPortBegin);
    }
    
    if (aclInfo->validBitMap & pa_ACL_INFO_VALID_CTRL_FLAG)  {
        
        if(aclEntry.aclInfo.ctrlFlagMask & pa_ACL_INFO_CONTROL_FLAG_FRAG)
            al1->bitMask |= PAFRM_IP_FLAG_FRAG;
            
        if(aclEntry.aclInfo.ctrlFlagMask & pa_ACL_INFO_CONTROL_FLAG_CONTAIN_L4)
            al1->bitMask |= PAFRM_IP_FLAG_CONTAIN_L4;
            
        if(aclEntry.aclInfo.ctrlFlag & pa_ACL_INFO_CONTROL_FLAG_FRAG)
            al1->u.ipv6.pktFlags |= PAFRM_IP_FLAG_FRAG;
            
        if(aclEntry.aclInfo.ctrlFlag & pa_ACL_INFO_CONTROL_FLAG_CONTAIN_L4)
            al1->u.ipv6.pktFlags |= PAFRM_IP_FLAG_CONTAIN_L4;
        
    }
    
    al1->u.ipv6.pktFlags = SWIZ(al1->u.ipv6.pktFlags);
    
  }

  /* Insert the entry to the Link List */
  ret1 = pa_insert_entry(listElemTable, paInst->nAcl, listInfoTable, insertMode,
                         nextEntry?(paHandleAcl_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,nextEntry):0,
                         i, &priority, *retHandle);

  if (ret1 != pa_OK) {
      ret = ret1;
      goto Pa_addAcl_end;
  }

  /* If inserting an ACL entry triggers rescoring, indicate it via rescore command */
  if (listInfoTable->ctrlBitMap & PA_ACL_LINK_INFO_RESCORING_ACTIVE)  {

    uint16_t pendingIndex;
   
    /* Generate the rescore list in scratch memory */ 
    ret1 = pa_gen_rescore_list(aclTable, listInfoTable, rescoreMem, rescoreHdr, insertMode, &pendingIndex);

    /* Update the pending offset in the command */
    al1->vLinkNum = SWIZ(pendingIndex);

    if (ret1 != pa_OK) {
        ret = ret1;
        goto Pa_addAcl_end;
    }    
  }

  al1->CBWords0 = SWIZ(CBWords0);
  al1->CBWords1 = SWIZ(CBWords1); 
  al1->bitMask =  SWIZ(al1->bitMask);
  al1->priority = SWIZ(priority);  
  
  mRoute = &pauForwardLut1Route;
  paFlags = 0;
  
  if (aclAction == pa_ACL_ACTION_MARK)
  {
    paFlags = PAFRM_PA_CTRL_PKT_MARK;  
  }
  else if (aclAction == pa_ACL_ACTION_DENY)
  {
    if ((aclEntry.aclInfo.ctrlFlag & pa_ACL_INFO_CONTROL_FLAG_CONTAIN_L4) &&
        (aclInfo->validBitMap & (pa_ACL_INFO_VALID_DST_PORT | pa_ACL_INFO_VALID_SRC_PORT)))
    {
        /* 
         * Special case: If we only drop the first fragment containing the matched L4 information, the other fragments will be
         *               eventually dropped by the reassembly engine after a long timeout. To avoid this long timeout, we
         *               can simulate the state-full firewall operation by dropping the (reassembled) packet after the reassembly
         *               module.         
         */
        paFlags = PAFRM_PA_CTRL_PKT_DROP;
    }
    else
    {
        mRoute = &pauFailRoute;
    }
  }
  
  nfRoute = &pauFailRoute;
  
  ret1 = pa_conv_routing_info2(paInst, &al1->match, mRoute, *cmdDest, FALSE, firstPdsp, paFlags, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
      ret = ret1;
      goto Pa_addAcl_end;
  }
  
  /* Next fail information */
  ret1 = pa_conv_routing_info2(paInst, &al1->nextFail, nfRoute, *cmdDest, TRUE, firstPdsp, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
      ret = ret1;
      goto Pa_addAcl_end;
  }
  
  /* Add the status and pdsp NUM */
  aclEntry.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
  if (prevLink != NULL)
    aclEntry.pHandle    = (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr, prevLink);
  aclEntry.hdr.type     = PA_TABLE_ENTRY_TYPE_ACL;
  aclEntry.hdr.subType  = PA_TABLE_ENTRY_SUBYTPE_L3L4;   
  aclEntry.hdr.tableIdx = i;
  aclEntry.hdr.lutIdx   = -1;

  /* store in the aclTable */
  memcpy (&aclTable[i], &aclEntry, sizeof(paAclEntry_t));
  
  /* clear ACL stats */
  pa_clear_acl_stats(&aclTable[i]);
  
Pa_addAcl_end:  

  if (l2Release) Pa_osalEndMemAccess (l2Table,
                                      paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  if (l3Release) Pa_osalEndMemAccess (l3Table,
                                      paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  Pa_osalEndMemAccess (aclTable,
                       paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);

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

  /* The entry is created in the stack and then copied to the table. This allows
   * for a comparison with other table entries to verify there is not an identical
   * entry */
  paInst_t               *paInst   = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paFcEntry_t            *fcTable  = (paFcEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_FC_TABLE].base);
  paFcEntry_t            fcEntry;
  paFcEntry_t*           pRepEntry;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  int                     i;
  uint16_t                lut1Index = (index == pa_LUT1_INDEX_NOT_SPECIFIED)?PAFRM_LUT1_INDEX_LAST_FREE:(uint16_t)index;
  uint16_t                csize;
  uint16_t                priority;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey, CBWords0, CBWords1;
  uint8_t                 subnetSize;
  
  if((efOpInfo == NULL) || (fcInfo == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }
  
  if (lut1Index > PAFRM_HW_LUT1_ENTRIES) {
    return(pa_INVALID_LUT1_INDEX);
  }
  
  pRepEntry = (paFcEntry_t*) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, *retHandle);
  
  /* Sanity check the command buffer */
  csize = sizeof(pafrmCommand_t)+sizeof(pafrmCommandAddLut1_t)-sizeof(uint32_t); 
  if (*cmdSize < csize)
  {
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);
  }  

  /* Return the actual size of the buffer used */
  *cmdSize = csize;
  /* Sanity check the command reply information.  */
  if (reply->dest != pa_DEST_HOST)
    return (pa_INVALID_CMD_REPLY_DEST);
    
  if (paLObj.cfg.rmServiceHandle) {
    int32_t lutInst = pa_LUT1_INST_5_0;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &lutInst, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }

	/* we use RM only for permission check, so freeing up immediately */
	if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &lutInst, NULL)) {
      return pa_RESOURCE_FREE_DENIED;
    }
  }

  /* Create the entry on the stack, and zero the entire entry. This will
   * allow simple compare of entries using memcmp even though some fields
   * will be unused, or there will be gaps between fields */
  memset (&fcEntry, 0, sizeof(paFcEntry_t));
  
  fcEntry.fcInfo.validBitMap = fcInfo->validBitMap;
  fcEntry.fcInfo.ipType      = fcInfo->ipType;
  
  /* For IPv4, copy the 4 address bytes to the last 4 bytes in an IPv6 sized byte array */
  if (fcInfo->ipType == pa_IPV4)  {
    if(fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_IP)
        memcpy (&(fcEntry.fcInfo.srcIp.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                fcInfo->srcIp.ipv4, sizeof(paIpv4Addr_t));
                
    if(fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_IP_MASK)
        memcpy (&(fcEntry.fcInfo.srcIpMask.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                fcInfo->srcIpMask.ipv4, sizeof(paIpv4Addr_t));
                
    if(fcInfo->validBitMap & pa_FC_INFO_VALID_DST_IP)
        memcpy (&(fcEntry.fcInfo.dstIp.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                fcInfo->dstIp.ipv4, sizeof(paIpv4Addr_t));
                
    if(fcInfo->validBitMap & pa_FC_INFO_VALID_DST_IP_MASK)
        memcpy (&(fcEntry.fcInfo.dstIpMask.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), 
                fcInfo->dstIpMask.ipv4, sizeof(paIpv4Addr_t));

  }  else if (fcInfo->ipType == pa_IPV6)  {
    if(fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_IP)
        memcpy (&fcEntry.fcInfo.srcIp.ipv6, fcInfo->srcIp.ipv6, sizeof(paIpv6Addr_t));
        
    if(fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_IP_MASK)
        memcpy (&fcEntry.fcInfo.srcIpMask.ipv6, fcInfo->srcIpMask.ipv6, sizeof(paIpv6Addr_t));
        
    if(fcInfo->validBitMap & pa_FC_INFO_VALID_DST_IP)
        memcpy (&fcEntry.fcInfo.dstIp.ipv6, fcInfo->dstIp.ipv6, sizeof(paIpv6Addr_t));
        
    if(fcInfo->validBitMap & pa_FC_INFO_VALID_DST_IP_MASK)
        memcpy (&fcEntry.fcInfo.dstIpMask.ipv6, fcInfo->dstIpMask.ipv6, sizeof(paIpv6Addr_t));
  }  else
    return (pa_ERR_CONFIG);
    
  if (fcInfo->validBitMap & pa_FC_INFO_VALID_CTRL_FLAG)
  {
    fcEntry.fcInfo.ctrlFlag       =  fcInfo->ctrlFlag & fcInfo->ctrlFlagMask;
    fcEntry.fcInfo.ctrlFlagMask   =  fcInfo->ctrlFlagMask;
  }
  
  if (fcInfo->validBitMap & pa_FC_INFO_VALID_PROTO)
  {
    fcEntry.fcInfo.proto      =  fcInfo->proto;
  }
  
  if (fcInfo->validBitMap & pa_FC_INFO_VALID_DSCP)
  {
    fcEntry.fcInfo.dscp       =  fcInfo->dscp;
  }
  
  if (fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_PORT)
  {
    fcEntry.fcInfo.srcPortBegin   =  fcInfo->srcPortBegin;
    fcEntry.fcInfo.srcPortEnd     =  fcInfo->srcPortEnd;
    
    if(fcInfo->srcPortEnd < fcInfo->srcPortBegin)
        return (pa_ERR_CONFIG);
  }
  
  if (fcInfo->validBitMap & pa_FC_INFO_VALID_DST_PORT)
  {
    fcEntry.fcInfo.dstPortBegin   =  fcInfo->dstPortBegin;
    fcEntry.fcInfo.dstPortEnd     =  fcInfo->dstPortEnd;
    
    if(fcInfo->dstPortEnd < fcInfo->dstPortBegin)
        return (pa_ERR_CONFIG);
  }
    
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  /* Signal the application that a table modification will be done. */
  Pa_osalBeginMemAccess (fcTable,
                         paInst->paBufs[PA_BUFFNUM_FC_TABLE].size);
                         
  /* Set the command destination and related information */
  fcEntry.hdr.pdspNum = PASS_EGRESS0_PDSP0;
  *cmdDest            = pa_CMD_TX_DEST_6;
  
  /* Look for an identical entry or replaced entry in the table. If one is found, return it */
  /* perform entry check only if the LUT1 index is not specified by user */
  if (lut1Index == PAFRM_LUT1_INDEX_LAST_FREE) {
    for (i = 0; i < paInst->nFc; i++)  {

        if (pRepEntry == &fcTable[i])
        {
            if (fcTable[i].hdr.status == PA_TBL_STAT_ACTIVE)
            {
                /* The application intends to replace this entry */
                memcpy(&fcTable[i].fcInfo, &fcEntry.fcInfo, sizeof(paFcInfo_t));
                lut1Index = fcTable[i].hdr.lutIdx;
                goto Pa_addFc_send_cmd;
            }
            else if (fcTable[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY)
            {
                ret = pa_PENDING_FC_ENTRY;
                goto Pa_addFc_end;
            }
        }

        if ((fcTable[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (fcTable[i].hdr.status == PA_TBL_STAT_ACTIVE)               )   {

            if (!memcmp (&fcTable[i].fcInfo, &fcEntry.fcInfo, sizeof(paFcInfo_t)))  { 

               if (fcTable[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) {
                    *retHandle = (paHandleFc_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&(fcTable[i]));
                    ret = pa_INVALID_DUP_ENTRY;
                    goto Pa_addFc_end;
                }                     
                else {
                   /*  
                    * Identical entry is identified, reuse the same table entry
                    * Keep the entry status since it simply replaces the original entry
                    * and the user may decide not to forward the packet
                    */ 
                    ret = pa_DUP_ENTRY; 
                    lut1Index = fcTable[i].hdr.lutIdx;
                    goto Pa_addFc_send_cmd;  
                }                  
            }
        }
    
    }  
  }
  /* Find a free entry in the table */
  for (i = 0; i < paInst->nFc; i++)  {

      if (fcTable[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;
  }
  
  if (i == paInst->nFc)  {
      ret = pa_HANDLE_TABLE_FULL;
      goto Pa_addFc_end;
  }
  
Pa_addFc_send_cmd:  
  
  /* The handle is just a pointer to the table entry */
  *retHandle =  (paHandleFc_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&(fcTable[i]));

  /* Create the command */                            
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_FC  | i, 0, csize);
  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;
  
  al1->index = SWIZ(lut1Index);
  al1->type  = PAFRM_COM_ADD_LUT1_STANDARD;
  al1->statsIndex = PASS_FC_STATS_BASE_ENTRY(i); /* Used for stats index */
  al1->statsIndex = SWIZ(al1->statsIndex);
  priority   = 0; 
      
  if (fcEntry.fcInfo.ipType == pa_IPV4)
  {    
    /* IPV4 entry */    
    CBWords0 = (uint32_t)(PAFRM_LUT1_CLASS_IPV4 << PAFRM_LUT1_CLASS_SHIFT) | PAFRM_LUT1_VALID_IPV4_PKTFLAGS;
    CBWords1 = PAFRM_LUT1_VALID_IPV4_PKTTYPE;

    al1->u.ipv4.pktType   = PAFRM_L3_PKT_TYPE_FC;
    al1->u.ipv4.pktFlags  = PAFRM_IP_FLAG_V4;
    al1->bitMask |= PAFRM_IP_FLAG_IP_TYPE;
    
    /* The IP type has already been validated as either IPv4 or IPv6. IPv4
     * addresses have been pre-padded with 0 to be the same size as IPv6 */

    if (fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_IP)
    {
      if(fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_IP_MASK)
        subnetSize = pa_find_ip_subnet_size (&(fcEntry.fcInfo.srcIpMask.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
      else
        subnetSize = 32;
          
      CBWords0 |= (subnetSize << PAFRM_LUT1_IPV4_SUBNET_SIP_SHIFT);
      priority += 10;
      memcpy(&al1->u.ipv4.srcIp,  &(fcEntry.fcInfo.srcIp.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
    }  

    if (fcInfo->validBitMap & pa_FC_INFO_VALID_DST_IP)
    {
      if(fcInfo->validBitMap & pa_FC_INFO_VALID_DST_IP_MASK)
        subnetSize = pa_find_ip_subnet_size (&(fcEntry.fcInfo.dstIpMask.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
      else
        subnetSize = 32;
    
      CBWords0 |= (subnetSize << PAFRM_LUT1_IPV4_SUBNET_DIP_SHIFT);
      priority += 10;
      memcpy(&al1->u.ipv4.dstIp,  &(fcEntry.fcInfo.dstIp.ipv6[sizeof(paIpv6Addr_t)-sizeof(paIpv4Addr_t)]), sizeof(paIpv4Addr_t));
    }  

    if (fcInfo->validBitMap & pa_FC_INFO_VALID_PROTO)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_PROTO;
        priority += 10;
        
        al1->u.ipv4.protocol = SWIZ(fcEntry.fcInfo.proto);
    }
    
    if (fcInfo->validBitMap & pa_FC_INFO_VALID_DSCP)
    {
        CBWords0 |= PAFRM_LUT1_VALID_IPV4_DSCP;
        priority += 10;
        al1->u.ipv4.dscp = SWIZ(fcEntry.fcInfo.dscp);
    }  
    
    if (fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_PORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_SRC_PORT;
        if (fcEntry.fcInfo.srcPortBegin != fcEntry.fcInfo.srcPortEnd)
        {
            CBWords0 |= PAFRM_LUT1_IPV4_SRC_PORT_RANGE; 
            al1->range0Hi = SWIZ(fcEntry.fcInfo.srcPortEnd);
            priority += 10;
        }       
        al1->u.ipv4.srcPort = SWIZ(fcEntry.fcInfo.srcPortBegin);
        priority += 10;
        
    }
    
    if (fcInfo->validBitMap & pa_FC_INFO_VALID_DST_PORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV4_DST_PORT;
        if (fcEntry.fcInfo.dstPortBegin != fcEntry.fcInfo.dstPortEnd)
        {
            CBWords0 |= PAFRM_LUT1_IPV4_DST_PORT_RANGE; 
            al1->range1Hi = SWIZ(fcEntry.fcInfo.dstPortEnd);
            priority += 10;
        }       
        al1->u.ipv4.dstPort = SWIZ(fcEntry.fcInfo.dstPortBegin);
        priority += 10;
        
    }
    
    if (fcInfo->validBitMap & pa_FC_INFO_VALID_CTRL_FLAG)  {
        priority += 10;
        if(fcEntry.fcInfo.ctrlFlagMask & pa_FC_INFO_CONTROL_FLAG_FRAG)
            al1->bitMask |= PAFRM_IP_FLAG_FRAG;
            
        if(fcEntry.fcInfo.ctrlFlagMask & pa_FC_INFO_CONTROL_FLAG_CONTAIN_L4)
            al1->bitMask |= PAFRM_IP_FLAG_CONTAIN_L4;
            
        if(fcEntry.fcInfo.ctrlFlag & pa_FC_INFO_CONTROL_FLAG_FRAG)
            al1->u.ipv4.pktFlags |= PAFRM_IP_FLAG_FRAG;
            
        if(fcEntry.fcInfo.ctrlFlag & pa_FC_INFO_CONTROL_FLAG_CONTAIN_L4)
            al1->u.ipv4.pktFlags |= PAFRM_IP_FLAG_CONTAIN_L4;
    }
    
    al1->u.ipv4.pktFlags = SWIZ(al1->u.ipv4.pktFlags);
  }
  else
  {    
    /* IPV6 entry */    
    CBWords0 = (uint32_t)(PAFRM_LUT1_CLASS_IPV6 << PAFRM_LUT1_CLASS_SHIFT) | PAFRM_LUT1_VALID_IPV6_PKTFLAGS;
    CBWords1 = PAFRM_LUT1_VALID_IPV6_PKTTYPE;

    al1->u.ipv6.pktType     = PAFRM_L3_PKT_TYPE_FC;
    
    /* The IP type has already been validated as either IPv4 or IPv6. IPv4
     * addresses have been pre-padded with 0 to be the same size as IPv6 */

    if (fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_IP)
    {
      if(fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_IP_MASK)
        subnetSize = pa_find_ip_subnet_size (&(fcEntry.fcInfo.srcIpMask.ipv6[0]), sizeof(paIpv6Addr_t));
      else
        subnetSize = 128;
          
      CBWords0 |= (subnetSize << PAFRM_LUT1_IPV6_SUBNET_SIP_SHIFT);
      priority += 10;
      memcpy(&al1->u.ipv6.srcIp0,  &(fcEntry.fcInfo.srcIp.ipv6[0]), sizeof(paIpv6Addr_t));
    }  

    if (fcInfo->validBitMap & pa_FC_INFO_VALID_DST_IP)
    {
      if(fcInfo->validBitMap & pa_FC_INFO_VALID_DST_IP_MASK)
        subnetSize = pa_find_ip_subnet_size (&(fcEntry.fcInfo.dstIpMask.ipv6[0]), sizeof(paIpv6Addr_t));
      else
        subnetSize = 128;
    
      CBWords0 |= (subnetSize << PAFRM_LUT1_IPV6_SUBNET_DIP_SHIFT);
      priority += 10;
      memcpy(&al1->u.ipv6.dstIp0,  &(fcEntry.fcInfo.dstIp.ipv6[0]), sizeof(paIpv6Addr_t));
    }  

    if (fcInfo->validBitMap & pa_FC_INFO_VALID_PROTO)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_PROTO;
        priority += 10;
        al1->u.ipv6.protocol = SWIZ(fcEntry.fcInfo.proto);
    }
    
    if (fcInfo->validBitMap & pa_FC_INFO_VALID_DSCP)  {
        CBWords0 |= PAFRM_LUT1_VALID_IPV6_DSCP;
        priority += 10;
        al1->u.ipv6.dscp = SWIZ(fcEntry.fcInfo.dscp);
    }  
    
    if (fcInfo->validBitMap & pa_FC_INFO_VALID_SRC_PORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_SRC_PORT;
        if (fcEntry.fcInfo.srcPortBegin != fcEntry.fcInfo.srcPortEnd)
        {
            CBWords0 |= PAFRM_LUT1_IPV6_SRC_PORT_RANGE; 
            priority += 10;
            al1->range0Hi = SWIZ(fcEntry.fcInfo.srcPortEnd);
        }       
        priority += 10;
        al1->u.ipv6.srcPort = SWIZ(fcEntry.fcInfo.srcPortBegin);
    }
    
    if (fcInfo->validBitMap & pa_FC_INFO_VALID_DST_PORT)  {
        CBWords1 |= PAFRM_LUT1_VALID_IPV6_DST_PORT;
        if (fcEntry.fcInfo.dstPortBegin != fcEntry.fcInfo.dstPortEnd)
        {
            CBWords0 |= PAFRM_LUT1_IPV6_DST_PORT_RANGE; 
            priority += 10;
            al1->range1Hi = SWIZ(fcEntry.fcInfo.dstPortEnd);
        }       
        priority += 10;
        al1->u.ipv6.dstPort = SWIZ(fcEntry.fcInfo.dstPortBegin);
    }
    
    if (fcInfo->validBitMap & pa_FC_INFO_VALID_CTRL_FLAG)  {
        
        priority += 10;
        if(fcEntry.fcInfo.ctrlFlagMask & pa_FC_INFO_CONTROL_FLAG_FRAG)
            al1->bitMask |= PAFRM_IP_FLAG_FRAG;
            
        if(fcEntry.fcInfo.ctrlFlagMask & pa_FC_INFO_CONTROL_FLAG_CONTAIN_L4)
            al1->bitMask |= PAFRM_IP_FLAG_CONTAIN_L4;
            
        if(fcEntry.fcInfo.ctrlFlag & pa_FC_INFO_CONTROL_FLAG_FRAG)
            al1->u.ipv6.pktFlags |= PAFRM_IP_FLAG_FRAG;
            
        if(fcEntry.fcInfo.ctrlFlag & pa_FC_INFO_CONTROL_FLAG_CONTAIN_L4)
            al1->u.ipv6.pktFlags |= PAFRM_IP_FLAG_CONTAIN_L4;
        
    }
    
    al1->u.ipv6.pktFlags = SWIZ(al1->u.ipv6.pktFlags);
    
  }
  
  al1->CBWords0 = SWIZ(CBWords0);
  al1->CBWords1 = SWIZ(CBWords1); 
  al1->bitMask =  SWIZ(al1->bitMask);
  al1->priority = SWIZ(priority);  
  
  ret1 = pa_conv_fc_routing_info(&al1->match, efOpInfo);
  if (ret1 != pa_OK) {
      ret = ret1;
      goto Pa_addFc_end;
  }
  
  /* Add the status and pdsp NUM */
  if (fcTable[i].hdr.status != PA_TBL_STAT_ACTIVE) {
    fcEntry.hdr.status   = PA_TBL_STAT_PENDING_SUBS_REPLY;
    fcEntry.hdr.type     = PA_TABLE_ENTRY_TYPE_FC;
    fcEntry.hdr.subType  = PA_TABLE_ENTRY_SUBYTPE_L3L4;   
    fcEntry.hdr.tableIdx = i;
    fcEntry.hdr.lutIdx   = -1;
    memcpy (&fcTable[i], &fcEntry, sizeof(paFcEntry_t));
    /* clear FC stats */
    pa_clear_fc_stats(&fcTable[i]);
  }
  
Pa_addFc_end:  

  Pa_osalEndMemAccess (fcTable,
                       paInst->paBufs[PA_BUFFNUM_FC_TABLE].size);
                       
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (ret);

} /* Pa_addFc */

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
  paL4Entry_t           *hl4;
  uint16_t               csize, srcVC;
  paReturn_t             ret = pa_OK;
  paReturn_t             ret1;
  uint32_t               mtCsKey;
  int                    fReplace;
  paL3Entry_t           *l3Table   = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  paVirtualLnk_t        *vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);
  
  if((params == NULL) || (routeInfo == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  fReplace =  (params->validBitMap & pa_LUT2_PARAM_VALID_CTRL_BITMAP) &&
              (params->ctrlFlags & pa_LUT2_INFO_CONTROL_FLAG_REPLACE);
  
  /* Future enhancement for RM protection of LUT2 entries */
  #if 0
  if (paLObj.cfg.rmServiceHandle) {
    int32_t lutInst = pa_CMD_TX_DEST_4 - pa_CMD_TX_DEST_0 ;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &lutInst, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }
    
	/* we use RM only for permission check, so freeing up immediately */
    if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &lutInst, NULL)) {
       return  pa_RESOURCE_FREE_DENIED;
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

  /* Initialize the return handle */
  hl4 = (paL4Entry_t *) retHandle;
  memset (hl4, 0, sizeof(paL4Entry_t));
  hl4->lnkType = PA_TABLE_ENTRY_TYPE_NONE;
  hl4->subType = (portSize == pa_LUT2_PORT_SIZE_16)?PA_TABLE_ENTRY_SUBYTPE_PORT16:
                                                    PA_TABLE_ENTRY_SUBYTPE_PORT32;
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  /* Verify the link */
  Pa_osalBeginMemAccess ((void *) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
                         
  /* Check to see whether we are using virtual link */
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)
    Pa_osalBeginMemAccess ((void *) vlnkTable,
                           paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
                           
  if ((hdr == NULL) || (hdr->status == PA_TBL_STAT_INACTIVE) || 
      ((hdr->type != PA_TABLE_ENTRY_TYPE_L3) && (hdr->type != PA_TABLE_ENTRY_TYPE_VL)))  {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addPort_end;
  }
                           
  if (hdr->type == PA_TABLE_ENTRY_TYPE_VL)
  {
      srcVC = hdr->tableIdx;
  }
  else
  {
      srcVC = PAFRM_MK_SRC_VC(hdr->pdspNum, hdr->lutIdx);
  }

  hl4->lnkType = hdr->type;
  hl4->lnkTableIdx = hdr->tableIdx;
  hl4->lnk = srcVC;

  if (hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16) {
    hl4->u.port16 = (uint16_t)destPort;
  }
  else {
    hl4->u.port32 = destPort;
  }

  /* Create the command */   
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT2, 0, 1, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al2 = (pafrmCommandAddLut2_t *) &(fcmd->cmd);

  al2->type  = PAFRM_COM_ADD_LUT2_STANDARD;
  al2->ctrlBitMap |= (fReplace)?PAFRM_COM_LUT2_CTRL_REPLACE:0;
  al2->inkTableIdx = SWIZ(hl4->lnkTableIdx);
  al2->srcVC       = SWIZ(hl4->lnk);
  
  if (hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16) {
  
    al2->u.port.portNum =  SWIZ((uint32_t)hl4->u.port16);
    al2->l4Type  = PAFRM_L4_PKT_TYPE_PORT16;
    
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
    al2->u.port.portNum =  SWIZ(hl4->u.port32);
    al2->l4Type = PAFRM_L4_PKT_TYPE_PORT32;
  }
  
  /* Queue Divert information */
  if ((params->validBitMap & pa_LUT2_PARAM_VALID_DIVERTQ)) {
    al2->qDivert.srcQ  = SWIZ(params->divertQ);
    al2->qDivert.destQ = SWIZ(routeInfo->queue);
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_QUEUE_DIVERT;
  }
  
  /* L4 Link information */
  if (hl4->lnkType == PA_TABLE_ENTRY_TYPE_VL) {
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_LINK | PAFRM_COM_LUT2_CTRL_VLINK;
  } else if (hl4->lnkType == PA_TABLE_ENTRY_TYPE_L3) {
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_LINK;
  }
  
  al2->ctrlBitMap   = SWIZ(al2->ctrlBitMap);  
     
  /* Forwarding information */
  ret1 = pa_conv_routing_info2(paInst, &al2->match, routeInfo, pa_CMD_TX_DEST_4, FALSE, 1, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addPort_end;
  }
  
  /* Update the link counter */
  if (!fReplace)
  {
    hdr->lnkCnt++;
  }
  
  /* Only one valid destination for a LUT2 add */
  *cmdDest = pa_CMD_TX_DEST_4;
  
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

} /* Pa_addPort2 */

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
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0, 0, csize);

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

  /* Any PDSP can be used to handle the command. POST PDSP0 is chosen
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
  paInst_t				 *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paL3Entry_t            *l3Table= (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  paL3Entry_t             l3Entry;
  paL2L3Header_t         *hdr = NULL;
  pafrmCommand_t         *fcmd;
  pafrmCommandAddLut1_t  *al1;
  int                     i;
  uint16_t                csize;
  uint16_t                l2Release;
  uint16_t                lut1Index = (index == pa_LUT1_INDEX_NOT_SPECIFIED)?PAFRM_LUT1_INDEX_LAST_FREE:(uint16_t)index;
  paReturn_t              ret = pa_OK;
  paReturn_t              ret1;
  uint32_t                mtCsKey, CBWords0, CBWords1;  
  paL2Entry_t 		     *l2Table   = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);
  
  if((match == NULL) || (routeInfo == NULL) || (nextRtFail == NULL) || (retHandle == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
  {
    return (pa_INVALID_INPUT_POINTER);
  }
  
  /* Sanity check: custom index range check */
  if (custIndex >= pa_MAX_CUSTOM_TYPES_LUT1)
    return ( pa_ERR_CONFIG );

  /* Sanity check the LUT1 instance */
  if ((lutInst != pa_LUT_INST_NOT_SPECIFIED) && ((lutInst != pa_LUT1_INST_1) && (lutInst != pa_LUT1_INST_2))) {
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
    for (i = 0; i < paInst->nL3; i++)  {

      if ( ((l3Table[i].hdr.status == PA_TBL_STAT_PENDING_SUBS_REPLY) ||
            (l3Table[i].hdr.status == PA_TBL_STAT_ACTIVE)               ) &&
            (l3Table[i].hdr.subType == PA_TABLE_ENTRY_SUBYTPE_CUSTOM)        )    {

        if (!memcmp (&l3Table[i].u.customInfo, &l3Entry.u.customInfo, sizeof(l3Entry.u.customInfo)))  {
	      paLnkHandle_t pHandle_loc = NULL;
          if (l3Table[i].pHandle)
		  	pHandle_loc  = (paLnkHandle_t)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3Table[i].pHandle);

          if (pHandle_loc == prevLink)  {

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
  for (i = 0; i < paInst->nL3; i++)  {

    if (l3Table[i].hdr.status == PA_TBL_STAT_INACTIVE)
      break;

  }

  if (i == paInst->nL3)  {
    ret = pa_HANDLE_TABLE_FULL;
    goto Pa_addCustomLUT1_end;
  }
  
Pa_addCustomLut1_send_cmd:
  
  /* The handle is just a pointer to the table entry */
  *retHandle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,&l3Table[i]);

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT1, PA_COMID_L3 | i, 0, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al1  = (pafrmCommandAddLut1_t *) &(fcmd->cmd);

  al1->index    = SWIZ(lut1Index);
  al1->type     = PAFRM_COM_ADD_LUT1_CUSTOM;
  CBWords0      = PAFRM_LUT1_CLASS_STANDARD << PAFRM_LUT1_CLASS_SHIFT;
  CBWords1      = PAFRM_LUT1_VALID_PKTTYPE;  
  
  al1->u.custom.pktType = PAFRM_L2_PKT_TYPE_CUSTOM + custIndex; 

  /* Ethertype holds the linked PDSP ID */
  /* vlan holds the linked PDSP LUT index */
  if (prevLink != NULL)  {
    al1->u.custom.srcVC = PAFRM_MK_SRC_VC(hdr->pdspNum, hdr->lutIdx);
    al1->u.custom.srcVC = SWIZ(al1->u.custom.srcVC);
    CBWords1 |= PAFRM_LUT1_VALID_SRC_VC;
  }
  
  memcpy (al1->u.custom.match, &l3Entry.u.customInfo.match, sizeof(l3Entry.u.customInfo.match));
  CBWords0 |= PAFRM_LUT1_VALID_CUSTOM_LUT1_0;
  CBWords1 |= PAFRM_LUT1_VALID_CUSTOM_LUT1_1;

  al1->CBWords0 = SWIZ(CBWords0);
  al1->CBWords1 = SWIZ(CBWords1);     

  /* find the command destination */
  if (lutInst != pa_LUT_INST_NOT_SPECIFIED) {
    switch (lutInst)
    {
        case pa_LUT1_INST_1:
            l3Entry.hdr.pdspNum = PASS_INGRESS1_PDSP0;
            *cmdDest            = pa_CMD_TX_DEST_1;
            lutInst             = pa_LUT1_INST_1_0;
            break;
            
        case pa_LUT1_INST_2:
            l3Entry.hdr.pdspNum = PASS_INGRESS4_PDSP0;
            *cmdDest            = pa_CMD_TX_DEST_4;
            lutInst             = pa_LUT1_INST_4_0;
            break;
    }
  
  }
  else if (prevLink == NULL)  {

    l3Entry.hdr.pdspNum = PASS_INGRESS1_PDSP0;
    *cmdDest            = pa_CMD_TX_DEST_1;
    lutInst             = pa_LUT1_INST_1_0;
    

  }  else  {

    if (hdr->type == PA_TABLE_ENTRY_TYPE_L2)  {
      l3Entry.hdr.pdspNum = PASS_INGRESS1_PDSP0;
      *cmdDest            = pa_CMD_TX_DEST_1;
      lutInst             = pa_LUT1_INST_1_0;
    }  else  {
      l3Entry.hdr.pdspNum = PASS_INGRESS4_PDSP0;
      *cmdDest            = pa_CMD_TX_DEST_4;
      lutInst             = pa_LUT1_INST_4_0;
    }

  }
  
  if (paLObj.cfg.rmServiceHandle) {      
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, (int32_t *) &lutInst, NULL)) {
      ret = pa_RESOURCE_USE_DENIED;
      goto Pa_addCustomLUT1_end;
    }
	  /* we use RM only for permission check, so freeing up immediately */
    if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, (int32_t *) &lutInst, NULL)) {
      ret = pa_RESOURCE_FREE_DENIED;
      goto Pa_addCustomLUT1_end;
    }	  
  }    
  
  /* Forwarding information */
  ret1 = pa_conv_routing_info(paInst, &al1->match, routeInfo, *cmdDest, FALSE, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addCustomLUT1_end;
  }

  /* Next fail information */
  ret1 = pa_conv_routing_info(paInst, &al1->nextFail, nextRtFail, *cmdDest, TRUE, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addCustomLUT1_end;
  }
  
  if (ret != pa_DUP_ENTRY)
  {
    /* Add the status and pdsp NUM */
    l3Entry.hdr.status  = PA_TBL_STAT_PENDING_SUBS_REPLY;
    if (prevLink != NULL)
      l3Entry.pHandle    = (paLnkHandle_t) prevLink;
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
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0, 1, csize);
  
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

  /* LUT2 resides at Ingress4 PDSP1 */
  *cmdDest = pa_CMD_TX_DEST_4;

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
  
  /* Future enhancement for RM protection of LUT2 entries */
  #if 0
  if (paLObj.cfg.rmServiceHandle) {
    int32_t lutInst = pa_CMD_TX_DEST_4 - pa_CMD_TX_DEST_0;
    
    if (!pa_rmService (Rm_service_RESOURCE_ALLOCATE_USE, rmLut, &lutInst, NULL)) {
      return pa_RESOURCE_USE_DENIED;
    }
    
	/* we use RM only for permission check, so freeing up immediately */
    if (!pa_rmService (Rm_service_RESOURCE_FREE, rmLut, &lutInst, NULL)) {
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
  hl4->subType = PA_TABLE_ENTRY_SUBYTPE_CUSTOM;
  hl4->customIndex = (uint8_t)custIndex;
  
  /* Copy all the match bytes even if the last one is used for link */
  memcpy (hl4->u.customInfo, match, sizeof(hl4->u.customInfo));

  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  Pa_osalBeginMemAccess ((void *) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);

  /* Verify the link */
  if ((hdr == NULL) || ((hdr->type != PA_TABLE_ENTRY_TYPE_L3) || (hdr->status == PA_TBL_STAT_INACTIVE)))  {
      ret = pa_INVALID_INPUT_HANDLE;
      goto Pa_addCustomLUT2_end;
  }

  hl4->lnkType = PA_TABLE_ENTRY_TYPE_L3;
  hl4->lnkTableIdx = hdr->tableIdx;
  hl4->lnk = PAFRM_MK_SRC_VC(hdr->pdspNum, hdr->lutIdx);
  
  /* Create the command */   
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_ADDREP_LUT2, 0, 1, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  al2 = (pafrmCommandAddLut2_t *) &(fcmd->cmd);

  al2->type  = PAFRM_COM_ADD_LUT2_CUSTOM;
  al2->l4Type  = PAFRM_L4_PKT_TYPE_CUSTOM;
  
  al2->index = (uint8_t)custIndex;
  al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_LINK;
  al2->ctrlBitMap |= (fReplace)?PAFRM_COM_LUT2_CTRL_REPLACE:0;
  
  al2->inkTableIdx = SWIZ(hl4->lnkTableIdx);
  al2->srcVC       = SWIZ(hl4->lnk);
  
  memcpy (al2->u.custom.match, hl4->u.customInfo, sizeof (al2->u.custom.match));
  
  /* Queue Divert information */
  if (divertQ != pa_PARAMS_NOT_SPECIFIED) {
    al2->qDivert.srcQ  = SWIZ(divertQ);
    al2->qDivert.destQ = SWIZ(routeInfo->queue);
    al2->ctrlBitMap |= PAFRM_COM_LUT2_CTRL_QUEUE_DIVERT;
  }
  
  al2->ctrlBitMap   = SWIZ(al2->ctrlBitMap);  

  /* Forwarding information */
  ret1 = pa_conv_routing_info(paInst, &al2->match, routeInfo, pa_CMD_TX_DEST_4, FALSE, 1, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
  if (ret1 != pa_OK) {
    ret = ret1;
    goto Pa_addCustomLUT2_end;
  }
  
  /* Update the link counter */
  if (hdr != NULL) {
    hdr->lnkCnt++;
  }
  
  /* Only one valid destination for a LUT2 add */
  *cmdDest = pa_CMD_TX_DEST_4;
  
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
      if (vhdr->hdr.lnkCnt > 0)
      {
        vhdr->hdr.lnkCnt--; 
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
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_DEL_LUT2, 0, 1, csize);

  /* Validity of the destination was already checked (HOST), so no other cases 
   * must be considered */
  fcmd->replyDest = PAFRM_DEST_PKTDMA;

  dl2 = (pafrmCommandDelLut2_t *) &(fcmd->cmd);
  dl2->type = PAFRM_COM_DEL_LUT2_STANDARD;
  dl2->srcVC  = SWIZ(hl4->lnk);
  dl2->lnkTableIdx = SWIZ(hl4->lnkTableIdx);
  
  if (hl4->subType == PA_TABLE_ENTRY_SUBYTPE_PORT16) {
  
    dl2->l4Type =  PAFRM_L4_PKT_TYPE_PORT16;
    dl2->u.port.portNum = SWIZ((uint32_t)hl4->u.port16);
    
    if (hl4->u.port16 == PAFRM_DEST_PORT_GTP)
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
    dl2->l4Type =  PAFRM_L4_PKT_TYPE_PORT32;
    dl2->u.port.portNum = SWIZ(hl4->u.port32);
    
  } else {

    dl2->type = PAFRM_COM_DEL_LUT2_CUSTOM;
    dl2->index =  hl4->customIndex;
    dl2->l4Type =  PAFRM_L4_PKT_TYPE_CUSTOM;
    memcpy (dl2->u.custom.match, hl4->u.customInfo, sizeof(dl2->u.custom.match));
  }
  dl2->index        = SWIZ(dl2->index);
  dl2->ctrlBitMap   = SWIZ(dl2->ctrlBitMap);
  
  /* Only one valid destination for a LUT2 add */
  *cmdDest = pa_CMD_TX_DEST_4;
  
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
  paEoamEntry_t         *eoamTable;  
  paL3Entry_t           *l3Table;
  paAclEntry_t          *aclTable = NULL;
  paFcEntry_t           *fcTable = NULL;
  pafrmCommand_t        *fcmd;
  pafrmCommandAddLut1_t *al1, *al1_2;
  pafrmCommandAddLut2_t *al2;
  pafrmCommandCmdHdr_t  *cmdHdr, *cmdHdr2;
  paL2Entry_t           *l2e;
  paL3Entry_t           *l3e = NULL, *l3e2 = NULL;
  paAclEntry_t          *acle = NULL;
  paFcEntry_t           *fce = NULL;
  paEoamEntry_t         *eoame = NULL;
  paL2L3Header_t        *hdr, *hdr2;
  paL4Entry_t           *pL4Entry;
  int8_t                 origStatus, origStatus2;
  paReturn_t             ret = pa_OK;
  uint32_t               mtCsKey;  
  paVirtualLnk_t        *vlnkTable = NULL;
  
  if ((vresult == NULL) || (retHandle == NULL) || (handleType == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);
    
  pL4Entry = (paL4Entry_t *)&retHandle->l4Handle;    

  /* The buffer contains the complete formatted command */
  fcmd = (pafrmCommand_t *)vresult;
  swizFcmd(fcmd);

  memset(retHandle, 0, sizeof(paEntryHandle_t));
  *handleType = pa_INVALID_HANDLE;
  
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  l2Table    = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);
  l3Table    = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);
  
  Pa_osalBeginMemAccess ((void *) l2Table,
                         paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalBeginMemAccess ((void *) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  if (paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)   
  {                    
    vlnkTable  = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);
    Pa_osalBeginMemAccess ( (void *) vlnkTable,
                           paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  } 

  if (paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size)
  {
    eoamTable  = (paEoamEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].base);  
    Pa_osalBeginMemAccess ((void *) eoamTable,
                           paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);    
  }  
  if (paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size)
  {
    aclTable  = (paAclEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_ACL_TABLE].base);
    Pa_osalBeginMemAccess ( (void *) aclTable,
                           paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);
                           
  }
  if (paInst->paBufs[PA_BUFFNUM_FC_TABLE].size)
  {
    fcTable  = (paFcEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_FC_TABLE].base);
    Pa_osalBeginMemAccess ( (void *) fcTable,
                           paInst->paBufs[PA_BUFFNUM_FC_TABLE].size);
  }
  
  if (fcmd->command == PAFRM_CONFIG_COMMAND_ADDREP_LUT1)  {

    al1 = (pafrmCommandAddLut1_t *)&(fcmd->cmd);

    if ((fcmd->comId & PA_COMID_L_MASK) == PA_COMID_L2)  {

      l2e = &l2Table[fcmd->comId & PA_COMID_IDX_MASK];
      hdr = &(l2e->hdr);
      retHandle->l2l3Handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,l2e);
      *handleType           =  pa_L2_HANDLE;

    }  else if ((fcmd->comId & PA_COMID_L_MASK) == PA_COMID_L3) {

      l3e = &l3Table[fcmd->comId & PA_COMID_IDX_MASK];
      hdr = &(l3e->hdr);
      retHandle->l2l3Handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,l3e);
      *handleType           =  pa_L3_HANDLE;
    } else if ((fcmd->comId & PA_COMID_L_MASK) == PA_COMID_ACL) {
    
      if (NULL == aclTable)
      {
        ret = pa_INVALID_INPUT_HANDLE;
        goto Pa_forwardResult_end;
      }
      acle = &aclTable[fcmd->comId & PA_COMID_IDX_MASK];
      hdr = &(acle->hdr);
      retHandle->l2l3Handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,acle);
      *handleType           =  pa_ACL_HANDLE;
    } 
    else if ((fcmd->comId & PA_COMID_L_MASK) == PA_COMID_EOAM) {
      if (NULL == eoamTable)
      {
        ret = pa_INVALID_INPUT_HANDLE;
        goto Pa_forwardResult_end;
      }  
      eoame = &eoamTable[fcmd->comId & PA_COMID_IDX_MASK];
      hdr = &(eoame->hdr);
      retHandle->l2l3Handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,eoame);
      *handleType           =  pa_EOAM_HANDLE;      
    } else {
    
      if (NULL == fcTable)
      {
        ret = pa_INVALID_INPUT_HANDLE;
        goto Pa_forwardResult_end;
      }
      
      fce = &fcTable[fcmd->comId & PA_COMID_IDX_MASK];
      hdr = &(fce->hdr);
      retHandle->l2l3Handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,fce);
      *handleType           =  pa_FC_HANDLE;
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
      if(SWIZ(al2->ctrlBitMap) & PAFRM_COM_LUT2_CTRL_VLINK)
        pL4Entry->lnkType =  PA_TABLE_ENTRY_TYPE_VL;
    }
    
    pL4Entry->lnkTableIdx = SWIZ(al2->inkTableIdx);
    pL4Entry->lnk = SWIZ(al2->srcVC);
    
    if (SWIZ(al2->type) == PAFRM_COM_ADD_LUT2_STANDARD) {
      if (SWIZ(al2->l4Type) == PAFRM_L4_PKT_TYPE_PORT32) {
        pL4Entry->subType = PA_TABLE_ENTRY_SUBYTPE_PORT32;
        pL4Entry->u.port32 = SWIZ(al2->u.port.portNum);
      }
      else {
        pL4Entry->subType = PA_TABLE_ENTRY_SUBYTPE_PORT16;
        pL4Entry->u.port16 = (uint16_t)SWIZ(al2->u.port.portNum);
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
      *cmdDest = pa_CMD_TX_DEST_4;
      ret = pa_RESUBMIT_COMMAND;
      /* Swizzle back the header */
      fcmd->commandResult = 0U;
      swizFcmd(fcmd);
    }
    else if (fcmd->commandResult == PAFRM_COMMAND_RESULT_LUT2_FULL) {
      ret = pa_LUT2_TABLE_FULL;
    }    
  }
  else if (fcmd->command == PAFRM_CONFIG_COMMAND_MULTI_CMDS)  {

    cmdHdr = (pafrmCommandCmdHdr_t *) &(fcmd->cmd);
    
    /* Just handle IPSEC entry */
    if (cmdHdr->command == PAFRM_CONFIG_COMMAND_ADDREP_LUT1)
    {
    
        cmdHdr->offset = SWIZ(cmdHdr->offset);
        cmdHdr->comId  = SWIZ(cmdHdr->comId);
        
        cmdHdr2 = (pafrmCommandCmdHdr_t *)((uint8_t*)fcmd + cmdHdr->offset);
        cmdHdr2->comId  = SWIZ(cmdHdr2->comId);

        al1 = (pafrmCommandAddLut1_t *)((uint8_t*)cmdHdr + sizeof(pafrmCommandCmdHdr_t));
        al1_2 = (pafrmCommandAddLut1_t *)((uint8_t*)cmdHdr2 + sizeof(pafrmCommandCmdHdr_t));
        
        l3e2 = &l3Table[cmdHdr2->comId & PA_COMID_IDX_MASK];
        hdr2 = &(l3e2->hdr);
        retHandle->l2l3Handle = (paHandleL2L3_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,l3e2);
        *handleType           =  pa_L3_HANDLE;
        
        l3e = &l3Table[cmdHdr->comId & PA_COMID_IDX_MASK];
        hdr = &(l3e->hdr);
        
        if (l3e2->pHandle != (paLnkHandle_t)pa_CONV_BASE_TO_OFFSET(paLObj.cfg.instPoolBaseAddr,l3e))
        {
            ret = pa_LUT_ENTRY_FAILED;
        }
        else
        {
            origStatus = hdr->status;
            origStatus2 = hdr2->status;

            if (fcmd->commandResult != PAFRM_COMMAND_RESULT_SUCCESS)  {
                #if 0 /* TBD */
                //hdr2->status = PA_TBL_STAT_INACTIVE;
     
                if (origStatus == PA_TBL_STAT_ACTIVE)
                {
                    /* Duplicate entry, remove the link */
                    if (l3e && l3e->pHandle)
                    {
                        hdr = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3e->pHandle);
                        hdr->lnkCnt--;
                    }
                }
                #endif
      
                ret = pa_LUT_ENTRY_FAILED;

            }  else  {

                hdr2->status = hdr->status = PA_TBL_STAT_ACTIVE;
                hdr->lutIdx = SWIZ(al1->index);
                hdr2->lutIdx = SWIZ(al1_2->index);
                
                if (origStatus2 != PA_TBL_STAT_ACTIVE)
                {
                    hdr->lnkCnt++;
                }
      
                if (l3e && l3e->pHandle && (origStatus != PA_TBL_STAT_ACTIVE)) {
                    hdr = (paL2L3Header_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr,l3e->pHandle);
                    hdr->lnkCnt++;
                }
            }
        }
    }

  }   
  
  else  {

    /* For other commands the Sub-System return value was not really required */

  }
  
Pa_forwardResult_end:
  
  if(paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size)
    Pa_osalEndMemAccess ((void *)vlnkTable,
                         paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  Pa_osalEndMemAccess ((void*)l2Table,
                       paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
  Pa_osalEndMemAccess ((void*)l3Table,
                       paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);
  if (paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size)
  {
    Pa_osalEndMemAccess ((void *)aclTable,
                         paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);
  }
  if (paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size)
  {
    Pa_osalEndMemAccess ((void *)eoamTable,
                         paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);
  }  
  if (paInst->paBufs[PA_BUFFNUM_FC_TABLE].size)
  {
    Pa_osalEndMemAccess ((void *)fcTable,
                         paInst->paBufs[PA_BUFFNUM_FC_TABLE].size);
  }
  
                       
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
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0, 0, csize);

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
    
    retCode =  pa_conv_routing_info2 (paInst, &cpa->u.eroute.eRoute[routeTypes[i]], &eRoutes[i], pa_CMD_TX_DEST_5, FALSE, 0, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
    
    if(retCode != pa_OK)
      break;
  }
  
  cpa->u.eroute.routeBitMap = SWIZ(cpa->u.eroute.routeBitMap);

  /* Destination can be any PDSP in ingress path, but Post PDSP0 is used since it is lightly loaded */
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
paReturn_t Pa_configExceptionRouteCommon(Pa_Handle       iHandle,
                                         int             nRoute,
                                         int            *routeTypes,
                                         paRouteInfo_t  *eRoutes,
                                         paCmd_t         cmd,
                                         uint16_t       *cmdSize,
                                         paCmdReply_t   *reply,
                                         int            *cmdDest,
                                         int            dest,
                                         int            maxEroutes)
{
  paInst_t			        *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  pafrmCommand_t            *fcmd;
  pafrmCommandSysConfigPa_t *cpa;
  int                       i;
  uint16_t                  csize;
  paReturn_t                retCode;
  
  if((routeTypes == NULL) || (eRoutes == NULL) || (cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
    return(pa_INVALID_INPUT_POINTER);

  /* Verify that there is enough room to create the command */
  csize = sizeof(pafrmCommand_t) + offsetof(pafrmCommandSysConfigPa_t, u) - sizeof(uint32_t) + 
          offsetof(pafrmComEroute_t, eRoute) + maxEroutes*sizeof(pafrmForward_t);
          
  if (*cmdSize < csize)
    return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

  *cmdSize = csize;

  /* The destination for the command must be host or discard */
  if (  (reply->dest != pa_DEST_HOST)     &&
        (reply->dest != pa_DEST_DISCARD)  )
    return (pa_INVALID_CMD_REPLY_DEST);

  /* Make sure there is at least one route */
  if (nRoute <= 0)
    return (pa_ERR_CONFIG);

  /* Refresh PA Instance for read only */
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));

  /* Create the command */
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0, 0, csize);

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
      return (pa_ERR_CONFIG);
      
    if ( routeTypes[i] >= maxEroutes  )
      return (pa_ERR_CONFIG);  

    cpa->u.eroute.routeBitMap |=  (1 << routeTypes[i]);
    
    retCode =  pa_conv_routing_info (paInst, &cpa->u.eroute.eRoute[routeTypes[i]], &eRoutes[i], dest, FALSE, 0, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC);
    if(retCode != pa_OK)
        return(retCode);
    
  }
  
  cpa->u.eroute.routeBitMap = SWIZ(cpa->u.eroute.routeBitMap);

  /* Destination can be any PDSP, but POST PDSP0  is used since it is lightly loaded */
  *cmdDest = dest;

  return (pa_OK);

} /* Pa_configExceptionRouteCommon */

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
    return(Pa_configExceptionRouteCommon(iHandle, nRoute, routeTypes, eRoutes, cmd, cmdSize, reply,
                                         cmdDest, pa_CMD_TX_DEST_5, pa_EROUTE_MAX));


} /* Pa_configExceptionRoute */
/************************************************************************************
 * FUNCTION PURPOSE: Configure sub-system routing of exception conditions
 ************************************************************************************
 * DESCRIPTION: Packets which satifies the exception conditions can be routed to the 
 *              host 
 ************************************************************************************/
paReturn_t Pa_configEflowExceptionRoute(Pa_Handle       iHandle,
                                        int             nRoute,
                                        int            *routeTypes,
                                        paRouteInfo_t  *eRoutes,
                                        paCmd_t         cmd,
                                        uint16_t       *cmdSize,
                                        paCmdReply_t   *reply,
                                        int            *cmdDest)
{
    return(Pa_configExceptionRouteCommon(iHandle, nRoute, routeTypes, eRoutes, cmd, cmdSize, reply,
                                         cmdDest, pa_CMD_TX_DEST_7, pa_EFLOW_EROUTE_MAX));

} /* Pa_configEflowExceptionRoute */

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
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_MULTI_ROUTE, 0, 1, csize);

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

  /* Multi-route processing PDSP: POST PDSP1 */
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
    //crc_table4[i] = SWIZ(crc_table4[i]);
  }
}

/************************************************************************************
 * FUNCTION PURPOSE: Configure CRC engine
 ************************************************************************************
 * DESCRIPTION: This function is used to configure the specified CRC engine by 
 *   formating the required CRC configuration command packet.
 ************************************************************************************/
paReturn_t Pa_configCrcEngine (Pa_Handle       iHandle,
                               uint16_t        inst,
                               paCrcConfig_t  *cfgInfo,
                               paCmd_t         cmd,
                               uint16_t       *cmdSize,
                               paCmdReply_t   *reply,
                               int            *cmdDest)
{
  CSL_Pa_pcheckRecipeRegs     *pCheckRecipeRegs;
  uint32_t mtCsKey;
  uint32_t crcTbl[16];
  int i;
  
  if((cfgInfo == NULL) || (cmdSize == NULL)) 
    return(pa_INVALID_INPUT_POINTER);
   
  /* TBD: To be updated for NetCP 1.5 architecture, there are 4 recipes for each CRC engine */
  *cmdSize = 0;
  
  if(cfgInfo->size > pa_CRC_SIZE_32)
    return (pa_ERR_CONFIG);   

  switch (inst)
  {
    case pa_CRC_INST_0_0:
        pCheckRecipeRegs = &paLObj.pPpuRegs[PASS_INGRESS0_PDSP1]->PCHECK.RECIPE[0];
        break;
        
    case pa_CRC_INST_1_0:
        pCheckRecipeRegs = &paLObj.pPpuRegs[PASS_INGRESS1_PDSP1]->PCHECK.RECIPE[0];
        break;
        
    case pa_CRC_INST_4_0:
        pCheckRecipeRegs = &paLObj.pPpuRegs[PASS_INGRESS4_PDSP1]->PCHECK.RECIPE[0];
        break;
        
    case pa_CRC_INST_5_0:
        pCheckRecipeRegs = &paLObj.pPpuRegs[PASS_POST_PDSP1]->PCHECK.RECIPE[0];
        break;
        
    case pa_CRC_INST_6_0:
        pCheckRecipeRegs = &paLObj.pPpuRegs[PASS_EGRESS0_PDSP1]->PCHECK.RECIPE[0];
        break;
        
    case pa_CRC_INST_6_1:
        pCheckRecipeRegs = &paLObj.pPpuRegs[PASS_EGRESS0_PDSP2]->PCHECK.RECIPE[0];
        break;
        
     default:
        return (pa_ERR_CONFIG);  
  }
  
  Pa_osalMtCsEnter(&mtCsKey);
  
  pCheckRecipeRegs->CONTROL;
  
  pCheckRecipeRegs->CONTROL  = (cfgInfo->ctrlBitfield & pa_CRC_CONFIG_RIGHT_SHIFT)?CSL_PA_PCHECK_CONTROL_RSHIFT_MASK:0;
  pCheckRecipeRegs->CONTROL |= (cfgInfo->ctrlBitfield & pa_CRC_CONFIG_INVERSE_RESULT)? CSL_PA_PCHECK_CONTROL_FINAL_NOT_MASK:0;
                                                                          
  pa_init_crc_table4(cfgInfo->polynomial, crcTbl);
  for(i = 0; i < (PAFRM_CRC_TABLE_SIZE - 1); i++)
    pCheckRecipeRegs->TABLE[i] = crcTbl[i+1];  
  
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
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_CMD_SET, 0, 0, csize);

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
          ptx->crcSize = (uint8_t)crc->crcSize;
          ptx->lenOffset = (uint8_t)crc->lenOffset;
          ptx->lenMask = SWIZ(crc->lenMask);
          ptx->crcOffset = SWIZ(crc->crcOffset);
          ptx->len = SWIZ(crc->len);
          ptx->initVal = SWIZ(crc->initValue);
          
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

  /* Command set processing PDSP: Post PDSP0 */
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
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_USR_STATS, 0, 0, csize);

  /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
   * must be considered */
  if (reply->dest == pa_DEST_HOST)
    fcmd->replyDest = PAFRM_DEST_PKTDMA;
  else
    fcmd->replyDest = PAFRM_DEST_DISCARD;

  cntCfg = (pafrmUsrStatsCntCfg_t *)&(fcmd->cmd);
  
  if(pCntCfg->ctrlBitfield & pa_USR_STATS_CONFIG_RESET)
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
  /* All user-defined statistics related configuration should occur at Post PDSP0 (User-defined statistic processing PDSP */
  *cmdDest = pa_CMD_TX_DEST_5;
            
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
 * FUNCTION PURPOSE: Update Egress Flow record 
 ************************************************************************************
 * DESCRIPTION: Issue Egress Flow record update command to PASS 
 ************************************************************************************/
static paReturn_t pa_update_ef_rec (volatile pafrmHostCfgCommand_t *pCfgCmd,
                                    volatile uint32_t  *pRecBuf,
                                    paEfRec_t *pRec,
                                    uint8_t   *record,
                                    int       recSize)
{
    uint32_t    mtCsKey; 
    uint32_t    cmd;
    uint8_t    *pSrc = record;
    int i;
    paReturn_t ret = pa_OK;                                               
    
    
    cmd = PAFRM_HOST_FORMAT_COMMAND(PAFRM_HOST_CMD_CFG_EF_RECORD, pRec->type, pRec->index);
     
    /* Refresh PA Instance for read only */
    Pa_osalMtCsEnter(&mtCsKey);
    
    /* Update the record */
    for (i = 0; i < recSize/4; i++, pSrc+= 4)
    {
        pRecBuf[i] =  PALLD_MK_UINT32_FROM_8ARRAY(pSrc);
    }
    
    /* issue the command */
    pCfgCmd->resp = PAFRM_HOST_CMD_RESP_OK;
    pCfgCmd->cmd = cmd;
    
    /* Wait for command to be executed */
    while (pCfgCmd->cmd) { };
    
    if(pCfgCmd->resp != PAFRM_HOST_CMD_RESP_OK)
        ret = pa_EF_REC_CONFIG_ERR; 
     
    Pa_osalMtCsExit(mtCsKey);
    
    return(ret);
}    

/************************************************************************************
 * FUNCTION PURPOSE: Configure Egress Flow Type One Record 
 ************************************************************************************
 * DESCRIPTION: This function is used to configure egress flow type one record 
 ************************************************************************************/
static paReturn_t pa_config_ef_rec1 (paInst_t  *paInst, paEfRec_t *pRec)
{
    paEfRecLevel1_t *pRec1 = &pRec->u.level1;
    uint8_t cfgData[PAFRM_EF_REC1_SIZE];
    uint16_t ctrlFlags = 0;
    int ipAddrSize = (pRec1->ipType == pa_IPV4)?pa_IPV4_ADDR_SIZE:pa_IPV6_ADDR_SIZE;
    volatile pafrmHostCfgCommand_t *pCfgCmd = (volatile pafrmHostCfgCommand_t *)
                                              &paLObj.pClRegs[PASS_CLUSTER_EF_REC1]->PDSP_SRAM[PAFRM_EF_REC1_CONFIG_CMD_OFFSET];
    volatile uint32_t              *pRecBuf = (volatile uint32_t *)
                                              &paLObj.pClRegs[PASS_CLUSTER_EF_REC1]->PDSP_SRAM[PAFRM_EF_REC1_CONFIG_BUF_OFFSET];
                                              
    paReturn_t ret;                                               
    
    /* Sanity check */
    if(pRec->index >= pa_MAX_EF_LVL1_RECORDS)
        return (pa_INVALID_EF_REC_INDEX); 
        
    memset(cfgData, 0,  PAFRM_EF_REC1_SIZE);   
        
    if(pRec->ctrlBitMap & pa_EF_RECORD_CONTROL_ENABLE)
    {
        /* Construct the EFlow record */  
        ctrlFlags = PAFRM_EF_REC1_VALID;
          
        if (pRec1->validBitMap & pa_EF_LVL1_RECORD_VALID_IP_SRC)
        {
            ctrlFlags |= PAFRM_EF_REC1_FLAG_IP_SRC_ADDR;
            memcpy(&cfgData[offsetof(paFrmEfRec1_t, srcIp)], pRec1->src.ipv6, ipAddrSize);
        }
        
        if (pRec1->validBitMap & pa_EF_LVL1_RECORD_VALID_IP_DST)
        {
            ctrlFlags |= PAFRM_EF_REC1_FLAG_IP_DST_ADDR;
            memcpy(&cfgData[offsetof(paFrmEfRec1_t, dstIp)], pRec1->dst.ipv6, ipAddrSize);
        }
        
        if (pRec1->validBitMap & pa_EF_LVL1_RECORD_VALID_FLOW_LABEL)
        {
            ctrlFlags |= PAFRM_EF_REC1_FLAG_IP_FLOW_LABEL;
            cfgData[offsetof(paFrmEfRec1_t, flowLablelHi)] = (pRec1->flow >> 16) & 0x0F;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec1_t, flowLablelLo), (pRec1->flow & 0xFFFF)); 
        }
        
        if (pRec1->validBitMap & pa_EF_LVL1_RECORD_VALID_TOS_CLASS)
        {
            ctrlFlags |= PAFRM_EF_REC1_FLAG_IP_TOS_CLASS;
            cfgData[offsetof(paFrmEfRec1_t, tos)] = pRec1->tos;
        }
        
        if (pRec1->validBitMap & pa_EF_LVL1_RECORD_VALID_IP_MTU)
        {
            ctrlFlags |= PAFRM_EF_REC1_FLAG_IP_MTU;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec1_t, mtu), pRec1->mtu); 
        }
        
        if (pRec1->validBitMap & pa_EF_LVL1_RECORD_VALID_SRC_PORT)
        {
            ctrlFlags |= PAFRM_EF_REC1_FLAG_L4_SRC_PORT;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec1_t, srcPort), pRec1->srcPort); 
        }
        
        if (pRec1->validBitMap & pa_EF_LVL1_RECORD_VALID_DST_PORT)
        {
            ctrlFlags |= PAFRM_EF_REC1_FLAG_L4_DST_PORT;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec1_t, dstPort), pRec1->dstPort); 
        }
        
        if (pRec1->validBitMap & pa_EF_LVL1_RECORD_VALID_CTRL_FLAGS)
        {
            if(pRec1->ctrlBitMap & pa_EF_LVL1_RECORD_CONTROL_FLAG_IP_CKSUM)
                ctrlFlags |= PAFRM_EF_REC1_FLAG_IPV4_CKSUM;    
        
            if(pRec1->ctrlBitMap & pa_EF_LVL1_RECORD_CONTROL_FLAG_L4_CKSUM)
                ctrlFlags |= PAFRM_EF_REC1_FLAG_L4_CKSUM;    
        
            if(pRec1->ctrlBitMap & pa_EF_LVL1_RECORD_CONTROL_FLAG_TTL_DEC)
                ctrlFlags |= PAFRM_EF_REC1_FLAG_IP_TTL_UPDATE;
                
            if(pRec1->ctrlBitMap & pa_EF_LVL1_RECORD_CONTROL_REMOVE_OUTER_IP_HDR_TRAIL)
                ctrlFlags |= PAFRM_EF_REC1_FLAG_STRIP_OUTER_IP; 
                
            if(pRec1->ctrlBitMap & pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_TCP_CTRL)
                ctrlFlags |= PAFRM_EF_REC1_FLAG_EXP_TCP_CTRL;    
                    
            if(pRec1->ctrlBitMap & pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_OPTION)
                ctrlFlags |= PAFRM_EF_REC1_FLAG_EXP_IP_OPTION;    
                    
            if(pRec1->ctrlBitMap & pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT)
                ctrlFlags |= PAFRM_EF_REC1_FLAG_EXP_IP_FRAGMENT;    
                    
            if(pRec1->ctrlBitMap & pa_EF_LVL1_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE)
                ctrlFlags |= PAFRM_EF_REC1_FLAG_EXP_IP_EXPIRE; 
                
        }
        
        pktWrite16bits_m(cfgData, offsetof(paFrmEfRec1_t, ctrlFlags), ctrlFlags); 
        
    }
    
    ret = pa_update_ef_rec(pCfgCmd, pRecBuf, pRec, cfgData, PAFRM_EF_REC1_SIZE);
    
    return (ret);
}

/************************************************************************************
 * FUNCTION PURPOSE: Configure Egress Flow Type Two Record 
 ************************************************************************************
 * DESCRIPTION: This function is used to configure egress flow type two record 
 ************************************************************************************/
static paReturn_t pa_config_ef_rec2 (paInst_t  *paInst, paEfRec_t *pRec)
{
    paEfRecLevel2_t *pRec2 = &pRec->u.level2;
    uint8_t cfgData[PAFRM_EF_REC2_SIZE];
    uint16_t ctrlFlags = 0;
    volatile pafrmHostCfgCommand_t *pCfgCmd = (volatile pafrmHostCfgCommand_t *)
                                              &paLObj.pClRegs[PASS_CLUSTER_EF_REC2]->PDSP_SRAM[PAFRM_EF_REC2_CONFIG_CMD_OFFSET];
    volatile uint32_t              *pRecBuf = (volatile uint32_t *)
                                              &paLObj.pClRegs[PASS_CLUSTER_EF_REC2]->PDSP_SRAM[PAFRM_EF_REC2_CONFIG_BUF_OFFSET];
                                              
    paReturn_t ret;                                               
    
    /* Sanity check */
    if(pRec->index >= pa_MAX_EF_LVL2_RECORDS)
        return (pa_INVALID_EF_REC_INDEX); 
        
    memset(cfgData, 0,  PAFRM_EF_REC2_SIZE);   
        
    if(pRec->ctrlBitMap & pa_EF_RECORD_CONTROL_ENABLE)
    {
        /* Construct the EFlow record */  
        ctrlFlags = PAFRM_EF_REC2_VALID;
          
        if (pRec2->validBitMap & pa_EF_LVL2_RECORD_VALID_IP_MTU)
        {
            ctrlFlags |= PAFRM_EF_REC2_FLAG_IP_MTU;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec2_t, mtu), pRec2->mtu); 
        }
        
        if (pRec2->ipHdrSize && pRec2->ipHdr)
        {
            if ((pRec2->ipHdrSize > pa_MAX_EF_REC_IP_HDR_LEN) || (pRec2->ipHdrSize < 20))
            {
                return(pa_ERR_CONFIG);
            }
            
            cfgData[offsetof(paFrmEfRec2_t, l3HdrSize)] = pRec2->ipHdrSize;
            memcpy(&cfgData[offsetof(paFrmEfRec2_t, l3Hdr)], pRec2->ipHdr, pRec2->ipHdrSize);
        }
        
        if (pRec2->validBitMap & pa_EF_LVL2_RECORD_VALID_IPSEC)
        {
            ctrlFlags |= PAFRM_EF_REC2_FLAG_IPSEC_PROC;
            ctrlFlags |= (pRec2->ipsec.ipsecProto == pa_IPSEC_PROTO_AH)?PAFRM_EF_REC2_FLAG_IPSEC_AH:0;
            
            if(pRec2->ipsec.ctrlBitMap & pa_EF_RECORD_IPSEC_USE_LOC_DMA)
                 ctrlFlags |= PAFRM_EF_REC2_FLAG_LOC_DMA;    
            
            cfgData[offsetof(paFrmEfRec2_t, ivSize)] = pRec2->ipsec.ivSize;
            cfgData[offsetof(paFrmEfRec2_t, icvSize)] = pRec2->ipsec.macSize;
            cfgData[offsetof(paFrmEfRec2_t, encBlkSize)] = pRec2->ipsec.encBlkSize;
            cfgData[offsetof(paFrmEfRec2_t, flowId)] = pRec2->ipsec.flowId;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec2_t, queueId), pRec2->ipsec.queueId); 
            pktWrite32bits_m(cfgData, offsetof(paFrmEfRec2_t, spi), pRec2->ipsec.spi); 
            pktWrite32bits_m(cfgData, offsetof(paFrmEfRec2_t, swInfo0), pRec2->ipsec.saInfo0); 
            pktWrite32bits_m(cfgData, offsetof(paFrmEfRec2_t, swInfo1), pRec2->ipsec.saInfo1); 
            
        }
        
        if (pRec2->validBitMap & pa_EF_LVL2_RECORD_VALID_CTRL_FLAGS)
        {
            
            if (pRec2->ctrlBitMap & pa_EF_LVL2_RECORD_CONTROL_SINGLE_IP)
            {
                if (pRec2->ipHdrSize)
                {
                    return(pa_ERR_CONFIG);
                }
            
                ctrlFlags |= PAFRM_EF_REC2_FLAG_SINGLE_IP;   
            }     
        
            if (pRec2->ctrlBitMap & pa_EF_LVL2_RECORD_CONTROL_INSERT_IPSEC_HDR_TRAIL)
            {
                if (!(pRec2->validBitMap & pa_EF_LVL2_RECORD_VALID_IPSEC))
                    return(pa_ERR_CONFIG);
                
                ctrlFlags |= PAFRM_EF_REC2_FLAG_INS_IPSEC_HDR;
                ctrlFlags |= (pRec2->ipsec.ipsecProto == pa_IPSEC_PROTO_ESP)?PAFRM_EF_REC2_FLAG_INS_ESP_TRAIL:0;
            }       
        
            if (pRec2->ctrlBitMap & pa_EF_LVL2_RECORD_CONTROL_FLAG_TTL_DEC)
            {
                if (pRec2->ctrlBitMap & pa_EF_LVL2_RECORD_CONTROL_SINGLE_IP)
                {
                    return(pa_ERR_CONFIG);
                }
            
                ctrlFlags |= PAFRM_EF_REC2_FLAG_IP_TTL_UPDATE;
            }    
                
            if(pRec2->ctrlBitMap & pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_OPTION)
                ctrlFlags |= PAFRM_EF_REC2_FLAG_EXP_IP_OPTION;    
                    
            if(pRec2->ctrlBitMap & pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_FRAGMENT)
                ctrlFlags |= PAFRM_EF_REC2_FLAG_EXP_IP_FRAGMENT;    
                    
            if(pRec2->ctrlBitMap & pa_EF_LVL2_RECORD_CONTROL_FLAG_EXP_IP_EXPIRE)
                ctrlFlags |= PAFRM_EF_REC2_FLAG_EXP_IP_EXPIRE;    
                
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec2_t, ctrlFlags), ctrlFlags); 
                
        }
    }
    
    ret = pa_update_ef_rec(pCfgCmd, pRecBuf, pRec, cfgData, PAFRM_EF_REC2_SIZE);
    
    return (ret);
}

/************************************************************************************
 * FUNCTION PURPOSE: Configure Egress Flow Type Three Record 
 ************************************************************************************
 * DESCRIPTION: This function is used to configure egress flow type three record 
 ************************************************************************************/
static paReturn_t pa_config_ef_rec3 (paInst_t  *paInst, paEfRec_t *pRec)
{
    paEfRecLevel3_t *pRec3 = &pRec->u.level3;
    uint8_t cfgData[PAFRM_EF_REC3_SIZE];
    uint16_t ctrlFlags = 0;
    volatile pafrmHostCfgCommand_t *pCfgCmd = (volatile pafrmHostCfgCommand_t *)
                                              &paLObj.pClRegs[PASS_CLUSTER_EF_REC3]->PDSP_SRAM[PAFRM_EF_REC3_CONFIG_CMD_OFFSET];
    volatile uint32_t              *pRecBuf = (volatile uint32_t *)
                                              &paLObj.pClRegs[PASS_CLUSTER_EF_REC3]->PDSP_SRAM[PAFRM_EF_REC3_CONFIG_BUF_OFFSET];
    paReturn_t ret;                                               
    
    /* Sanity check */
    if(pRec->index >= pa_MAX_EF_LVL3_RECORDS)
        return (pa_INVALID_EF_REC_INDEX); 
        
    memset(cfgData, 0,  PAFRM_EF_REC3_SIZE);   
        
    if(pRec->ctrlBitMap & pa_EF_RECORD_CONTROL_ENABLE)
    {
        /* Construct the EFlow record */  
        ctrlFlags = PAFRM_EF_REC3_VALID;
          
        if (pRec3->validBitMap & pa_EF_LVL3_RECORD_VALID_IP_MTU)
        {
            ctrlFlags |= PAFRM_EF_REC3_FLAG_IP_MTU;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec3Ah_t, mtu), pRec3->mtu); 
        }
        
        if (pRec3->validBitMap & pa_EF_LVL3_RECORD_VALID_IPSEC)
        {
            /* IPSEC AH parameters */
            ctrlFlags |= PAFRM_EF_REC3_FLAG_IPSEC_AH;
            
            if(pRec3->ipsec.ctrlBitMap & pa_EF_RECORD_IPSEC_USE_LOC_DMA)
                 ctrlFlags |= PAFRM_EF_REC3_FLAG_LOC_DMA;    
            
            cfgData[offsetof(paFrmEfRec3Ah_t, ivSize)] = pRec3->ipsec.ivSize;
            cfgData[offsetof(paFrmEfRec3Ah_t, icvSize)] = pRec3->ipsec.macSize;
            cfgData[offsetof(paFrmEfRec3Ah_t, flowId)] = pRec3->ipsec.flowId;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec3Ah_t, queueId), pRec3->ipsec.queueId); 
            pktWrite32bits_m(cfgData, offsetof(paFrmEfRec3Ah_t, spi), pRec3->ipsec.spi); 
            pktWrite32bits_m(cfgData, offsetof(paFrmEfRec3Ah_t, swInfo0), pRec3->ipsec.saInfo0); 
            pktWrite32bits_m(cfgData, offsetof(paFrmEfRec3Ah_t, swInfo1), pRec3->ipsec.saInfo1); 
            
        }
        else
        {
            /* IPSEC NAT-T Parameters */
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec3Nat_t, srcPort), pRec3->srcPort); 
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec3Nat_t, dstPort), pRec3->dstPort); 
            
        }
        
        if (pRec3->validBitMap & pa_EF_LVL3_RECORD_VALID_CTRL_FLAGS)
        {
            if(pRec3->ctrlBitMap & pa_EF_LVL3_RECORD_CONTROL_REPLACE_HDR)
                ctrlFlags |= PAFRM_EF_REC3_FLAG_REP_HDR;    
        }
        
        pktWrite16bits_m(cfgData, offsetof(paFrmEfRec3Ah_t, ctrlFlags), ctrlFlags); 
        
    }
    
    ret = pa_update_ef_rec(pCfgCmd, pRecBuf, pRec, cfgData, PAFRM_EF_REC3_SIZE);
    
    return (ret);
}

/************************************************************************************
 * FUNCTION PURPOSE: Configure Egress Flow Type Four Record 
 ************************************************************************************
 * DESCRIPTION: This function is used to configure egress flow type four record 
 ************************************************************************************/
static paReturn_t pa_config_ef_rec4 (paInst_t  *paInst, paEfRec_t *pRec)
{
    paEfRecLevel4_t *pRec4 = &pRec->u.level4;
    uint8_t cfgData[PAFRM_EF_REC4_SIZE];
    uint16_t ctrlFlags = 0;
    volatile pafrmHostCfgCommand_t *pCfgCmd = (volatile pafrmHostCfgCommand_t *)
                                              &paLObj.pClRegs[PASS_CLUSTER_EF_REC4]->PDSP_SRAM[PAFRM_EF_REC4_CONFIG_CMD_OFFSET];
    volatile uint32_t              *pRecBuf = (volatile uint32_t *)
                                              &paLObj.pClRegs[PASS_CLUSTER_EF_REC4]->PDSP_SRAM[PAFRM_EF_REC4_CONFIG_BUF_OFFSET];
                                              
    paReturn_t ret;                                               
    
    /* Sanity check */
    if(pRec->index >= pa_MAX_EF_LVL4_RECORDS)
        return (pa_INVALID_EF_REC_INDEX); 
        
    memset(cfgData, 0,  PAFRM_EF_REC4_SIZE);   
        
    if(pRec->ctrlBitMap & pa_EF_RECORD_CONTROL_ENABLE)
    {
        /* Construct the EFlow record */  
        ctrlFlags = PAFRM_EF_REC4_VALID;
          
        if (pRec4->validBitMap & pa_EF_LVL4_RECORD_VALID_802_3)
        {
            ctrlFlags |= PAFRM_EF_REC4_FLAG_802_3;
            cfgData[offsetof(paFrmEfRec4_t, l2LenOffset)] = (uint8_t) pRec4->lenOffset802p3;
        }
        
        if (pRec4->validBitMap & pa_EF_LVL4_RECORD_VALID_PPPoE)
        {
            ctrlFlags |= PAFRM_EF_REC4_FLAG_PPPoE;
            cfgData[offsetof(paFrmEfRec4_t, pppoeOffset)] = (uint8_t) pRec4->lenOffsetPPPoE;
        }                                                   
        
        if (pRec4->validBitMap & pa_EF_LVL4_RECORD_VALID_VLAN1)
        {
            ctrlFlags |= PAFRM_EF_REC4_FLAG_VLAN1;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec4_t, vlan1), pRec4->vlan1); 
        }
        
        if (pRec4->validBitMap & pa_EF_LVL4_RECORD_VALID_VLAN2)
        {
            ctrlFlags |= PAFRM_EF_REC4_FLAG_VLAN2;
            pktWrite16bits_m(cfgData, offsetof(paFrmEfRec4_t, vlan2), pRec4->vlan2); 
        }
        
        if (pRec4->validBitMap & pa_EF_LVL4_RECORD_VALID_MIN_PKTSIZE)
        {
            ctrlFlags |= PAFRM_EF_REC4_FLAG_TX_PADDING;
            cfgData[offsetof(paFrmEfRec4_t, minPktSize)] = pRec4->minPktSize; 
        }                                                   
        
        if (pRec4->l2HdrSize && pRec4->l2Hdr)
        {
            if (pRec4->l2HdrSize > pa_MAX_EF_REC_L2_HDR_LEN)
            {
                return(pa_ERR_CONFIG);
            }
            
            cfgData[offsetof(paFrmEfRec4_t, l2HdrSize)] = pRec4->l2HdrSize;
            memcpy(&cfgData[offsetof(paFrmEfRec4_t, l2Hdr)], pRec4->l2Hdr, pRec4->l2HdrSize);
        }
        
        /* Destination related comfiguration */
        cfgData[offsetof(paFrmEfRec4_t, pktType_psFlags)] = pRec4->pktType_emacCtrl;
        
        switch (pRec4->dest)
        {
            case pa_DEST_EMAC:
                cfgData[offsetof(paFrmEfRec4_t, destType)] = PAFRM_FORWARD_TYPE_ETH;
                break;
            
            case pa_DEST_SRIO:
                pktWrite32bits_m(cfgData, offsetof(paFrmEfRec4_t, swInfo1), pRec4->swInfo1); 
                /* pass through */
            
            case pa_DEST_HOST:
                cfgData[offsetof(paFrmEfRec4_t, flowId)] = pRec4->flowId;
                pktWrite16bits_m(cfgData, offsetof(paFrmEfRec4_t, queueId), pRec4->queueId); 
                pktWrite32bits_m(cfgData, offsetof(paFrmEfRec4_t, swInfo0), pRec4->swInfo0); 
                cfgData[offsetof(paFrmEfRec4_t, destType)] = (pRec4->dest == pa_DEST_SRIO)?
                                                             PAFRM_FORWARD_TYPE_SRIO:PAFRM_FORWARD_TYPE_HOST;
                break;
            
            default:
                return(pa_ERR_CONFIG);
        }        
        
        if (pRec4->validBitMap & pa_EF_LVL4_RECORD_VALID_CTRL_FLAGS)
        {
        }
        
        if (pRec4->validBitMap & pa_REF_LVL4_RECORD_VALID_ROUTE_PRIORITY_TYPE)
        {
            if(pRec4->priIfType == pa_ROUTE_PRIORITY_VLAN)
            {
                ctrlFlags |= PAFRM_EF_REC4_ROUTE_PRIORITY_VLAN;
            }
            else if(pRec4->priIfType == pa_ROUTE_PRIORITY_DSCP)
            {
                ctrlFlags |= PAFRM_EF_REC4_ROUTE_PRIORITY_DSCP;
            }
        }
        
        pktWrite16bits_m(cfgData, offsetof(paFrmEfRec4_t, ctrlFlags), ctrlFlags); 
        
    }
    
    ret = pa_update_ef_rec(pCfgCmd, pRecBuf, pRec, cfgData, PAFRM_EF_REC4_SIZE);
    
    return (ret);
}


/************************************************************************************
 * FUNCTION PURPOSE: Configure Egress Flow records
 ************************************************************************************
 * DESCRIPTION: This function is used to configure on eor multiple egress flow records 
 ************************************************************************************/
paReturn_t Pa_configEflowRecords (Pa_Handle       iHandle,
                                  int             nRecords,
                                  int            *nRecProc,    
                                  paEfRec_t      *records
                                  )
{
  paInst_t  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  int i;
  paReturn_t ret = pa_OK;
  
  if (((nRecords > 0 ) && (records == NULL)) || (nRecProc == NULL))
  {
    return(pa_ERR_CONFIG);
  }
  
  for ( i = 0; i < nRecords; i++)
  {
    switch (records[i].type)
    {
        case pa_EFLOW_REC_TYPE_LVL1:
            ret = pa_config_ef_rec1(paInst, &records[i]);
            break;
            
        case pa_EFLOW_REC_TYPE_LVL2:
            ret = pa_config_ef_rec2(paInst, &records[i]);
            break;

        case pa_EFLOW_REC_TYPE_LVL3:
            ret = pa_config_ef_rec3(paInst, &records[i]);
            break;
            
        case pa_EFLOW_REC_TYPE_LVL4:
            ret = pa_config_ef_rec4(paInst, &records[i]);
            break;
            
        default:
            ret = pa_ERR_CONFIG;
            break;    
    }
    
    if (ret != pa_OK)
    {
        *nRecProc = i;
        return(ret);
    }
  
  }
  
   *nRecProc = i;
  return(pa_OK);

}  /* Pa_configEflowRecords */                                

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
  paInst_t			  *paInst = (paInst_t *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, iHandle);
  paSysConfig_t       *cfg = &ctrl->params.sysCfg;
  pafrmCommand_t      *fcmd;
  uint16_t            csize;
  paReturn_t          ret = pa_OK;
  uint32_t            mtCsKey;  

  if(ctrl == NULL)return(pa_INVALID_INPUT_POINTER);
  
  if((ctrl->code > pa_CONTROL_MAX_CONFIG_GEN2))
    return (pa_ERR_CONFIG);
    
  ret = pa_control_get_cmd_size(ctrl, &csize);
  
  if(ret != pa_OK)
    return(ret);
    
  /* The destination for the command must be host or discard */
  if (csize)
  {
    if((cmd == NULL) || (cmdSize == NULL) || (reply == NULL) || (cmdDest == NULL))
        return(pa_INVALID_INPUT_POINTER);
  
    if ((reply->dest != pa_DEST_HOST)     &&
        (reply->dest != pa_DEST_DISCARD))
        return (pa_INVALID_CMD_REPLY_DEST);
    
    /* Verify that there is enough room to create the command */
    if (*cmdSize < csize)
        return (pa_INSUFFICIENT_CMD_BUFFER_SIZE);

    *cmdSize = csize;
  }

  if (ctrl->code == pa_CONTROL_SYS_CONFIG)
  {
    pafrmCommandConfigPa_t *cpa;

    Pa_osalMtCsEnter(&mtCsKey);
    Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

    /* Create the command */
    fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_CONFIG_PA, 0, 0, csize);

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
        paPacketControlConfig_t* pPktControlCfg = cfg->pPktControl;
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
    
    /* Outer IP ACL configuration */
    if (cfg->pOutAclConfig)
    {
        paAclConfig_t* pAclCfg = cfg->pOutAclConfig;

        /* ACL config is not permitted when EOAM is enabled */
        if (PA_GET_STATE_EOAM(paInst) == 1)
        {
          ret = pa_ERR_CONFIG;
        }

        /* ACL insert mode config check, if not configured, default to random insert */
        if ( (pAclCfg->insertMode < pa_ACL_INSERT_RANDOM) ||
             (pAclCfg->insertMode > pa_ACL_INSERT_TOP) )
        {
             pAclCfg->insertMode = pa_ACL_INSERT_RANDOM;
        }
        
        /* range check */
        if (pAclCfg->action >  pa_ACL_ACTION_MAX) 
        {  
            ret = pa_ERR_CONFIG;
        }
        else
        {
            paInst->cfg.outAclConfig = *pAclCfg;
            
            cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_OUT_IP_ACL;
            
            switch (pAclCfg->action)
            {
                case pa_ACL_ACTION_DENY:
                    cpa->outAcl.action = (uint8_t)PAFRM_ACL_ACTION_DROP;
                    break;
                    
                case pa_ACL_ACTION_PERMIT:
                    cpa->outAcl.action = (uint8_t)PAFRM_ACL_ACTION_FORWARD;
                    break;    
                     
                case pa_ACL_ACTION_HOST:
                    cpa->outAcl.action = (uint8_t)PAFRM_ACL_ACTION_HOST;
                    break;    
                    
                /* deafult: (range check performed) */    
            
            }
            cpa->outAcl.destFlowId     = SWIZ(pAclCfg->destFlowId);
            cpa->outAcl.destQueue      = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC, pAclCfg->destQueue));
        }
    }
    
    /* Inner IP ACL configuration */
    if (cfg->pInAclConfig)
    {
        paAclConfig_t* pAclCfg = cfg->pInAclConfig;

        /* ACL insert mode config check, if not configured, default to random insert */
        if ( (pAclCfg->insertMode < pa_ACL_INSERT_RANDOM) ||
             (pAclCfg->insertMode > pa_ACL_INSERT_TOP) )
        {
             pAclCfg->insertMode = pa_ACL_INSERT_RANDOM;
        }
        
        /* range check */
        if (pAclCfg->action >  pa_ACL_ACTION_MAX) 
        {  
            ret = pa_ERR_CONFIG;
        }
        else
        {
            paInst->cfg.inAclConfig = *pAclCfg;
            
            cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_OUT_IP_ACL;
            
            switch (pAclCfg->action)
            {
                case pa_ACL_ACTION_DENY:
                    cpa->inAcl.action = (uint8_t)PAFRM_ACL_ACTION_DROP;
                    break;
                    
                case pa_ACL_ACTION_PERMIT:
                    cpa->inAcl.action = (uint8_t)PAFRM_ACL_ACTION_FORWARD;
                    break;    
                     
                case pa_ACL_ACTION_HOST:
                    cpa->inAcl.action = (uint8_t)PAFRM_ACL_ACTION_HOST;
                    break;    
                    
                /* deafult: (range check performed) */    
            
            }
            cpa->inAcl.destFlowId     = SWIZ(pAclCfg->destFlowId);
            cpa->inAcl.destQueue      = SWIZ(pa_convert_queue_id(paInst, pa_QUEUE_BOUNCE_ROUTING_CLASS_MISC, pAclCfg->destQueue));
        }
    }
    
    /* Outer IP RA configuration */
    if (cfg->pOutIpRaGroupConfig)
    {
        paRaGroupConfig_t* pRaGroupCfg = cfg->pOutIpRaGroupConfig;
        
        paInst->cfg.outIpRaGroupConfig = *pRaGroupCfg;
        
        cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_OUT_IP_RA;
        
        cpa->outRa.ctrlBitMap |= (pRaGroupCfg->ctrlBitMap & pa_RA_CTRL_ENABLE)?PAFRM_RA_CONTROL_EN:0;
        cpa->outRa.ctrlBitMap |= (pRaGroupCfg->ctrlBitMap & pa_RA_CTRL_USE_LOCAL_DMA)?PAFRM_RA_CONTROL_USE_LOC_DMA:0;
        cpa->outRa.ctrlBitMap |= (pRaGroupCfg->ctrlBitMap & pa_RA_CTRL_TO_QUEUE)?PAFRM_RA_CONTROL_TO_QUEUE:0;
        cpa->outRa.flowId = pRaGroupCfg->flowId;
        cpa->outRa.ctrlBitMap = SWIZ(cpa->outRa.ctrlBitMap);
        cpa->outRa.flowId = SWIZ(cpa->outRa.flowId);
        
        pa_ra_group_cfg(pRaGroupCfg, &paLObj.pSysRegs->RA.FLOW_OVERRIDE[0]);        
    
    }
    
    /* Inner IP RA configuration */
    if (cfg->pInIpRaGroupConfig)
    {
        paRaGroupConfig_t* pRaGroupCfg = cfg->pInIpRaGroupConfig;
        
        paInst->cfg.inIpRaGroupConfig = *pRaGroupCfg;
        
        cpa->validFlag |= PAFRM_COMMAND_CONFIG_VALID_IN_IP_RA;
        
        cpa->inRa.ctrlBitMap |= (pRaGroupCfg->ctrlBitMap & pa_RA_CTRL_ENABLE)?PAFRM_RA_CONTROL_EN:0;
        cpa->inRa.ctrlBitMap |= (pRaGroupCfg->ctrlBitMap & pa_RA_CTRL_USE_LOCAL_DMA)?PAFRM_RA_CONTROL_USE_LOC_DMA:0;
        cpa->inRa.ctrlBitMap |= (pRaGroupCfg->ctrlBitMap & pa_RA_CTRL_TO_QUEUE)?PAFRM_RA_CONTROL_TO_QUEUE:0;
        cpa->inRa.flowId = pRaGroupCfg->flowId;
        cpa->inRa.ctrlBitMap = SWIZ(cpa->inRa.ctrlBitMap);
        cpa->inRa.flowId = SWIZ(cpa->inRa.flowId);
        
        pa_ra_group_cfg(pRaGroupCfg, &paLObj.pSysRegs->RA.FLOW_OVERRIDE[1]);        
    
    }

    /* EOAM Global Configuration */
    if (cfg->pEoamConfig)
    {
        paEoamGlobalConfig_t* pEoamCfg = cfg->pEoamConfig;      
        paInst->cfg.eOamConfig = *pEoamCfg;

        /* Check if EOAM table is allocated for global configuration */
        if (paInst->nEoam <= 0)
          return (pa_ERR_CONFIG);

        if (pEoamCfg->enable) 
        {
          PA_SET_STATE_EOAM(paInst, 1); /* EOAM is now enabled */
          cpa->eoam.ctrlBitMap |= PAFRM_EOAM_CONTROL_EN;
        }
        else
        {
          PA_SET_STATE_EOAM(paInst, 0); /* EOAM is now disabled */ 
          cpa->eoam.ctrlBitMap &= ~PAFRM_EOAM_CONTROL_EN;          
        }
        
        cpa->validFlag       |= PAFRM_COMMAND_CONFIG_VALID_EOAM;
        cpa->eoam.ctrlBitMap  = SWIZ(cpa->eoam.ctrlBitMap);        

        if (pEoamCfg->enable)
        {
         /*
          * Num Ro  SYS CLK (MHz) PA CLK (MHz)  1 PA Timer Tick (ns)  1 Ro Timer Tick (ns)  Num Ro * 1 Ro Over Timer Tick
          * 1       1000          333.3333333    6                    393216                 393216
          * 7       1050          350            5.714285714          374491.4286            2621440
          * 328     1049.6        349.8666667    5.716463415          374634.1463            122880000
          */
        
          switch (pEoamCfg->freq)
          {
              case pa_INPUT_FREQ_1000MHZ: 
                {
                  cpa->eoam.mul        = pa_1588_TIME_CONV_MUL_FOR_INPUT_FREQ_1000MHZ;
                  cpa->eoam.nsNumRoAcc = 393216 ;
                  cpa->eoam.nsRoAcc    = 393216;
                  cpa->eoam.numRo      = 1;
                  break;
                }
              case pa_INPUT_FREQ_1050MHZ:
                {
                  cpa->eoam.mul        = pa_1588_TIME_CONV_MUL_FOR_INPUT_FREQ_1050MHZ;
                  cpa->eoam.nsNumRoAcc = 2621440;
                  cpa->eoam.nsRoAcc    = 374491;
                  cpa->eoam.numRo      = 7; 
                  break;
                }
              case pa_INPUT_FREQ_1049p6MHZ:
                {
                  cpa->eoam.mul        = pa_1588_TIME_CONV_MUL_FOR_INPUT_FREQ_1049p6MHZ;
                  cpa->eoam.nsNumRoAcc = 122880000;
                  cpa->eoam.nsRoAcc    = 374634;
                  cpa->eoam.numRo      = 328;              
                  break;
                }
              default:
                return (pa_ERR_CONFIG);        
          }
        

          cpa->eoam.mul        = SWIZ(cpa->eoam.mul);              
          cpa->eoam.nsNumRoAcc = SWIZ(cpa->eoam.nsNumRoAcc);              
          cpa->eoam.nsRoAcc    = SWIZ(cpa->eoam.nsRoAcc);              
          cpa->eoam.numRo      = SWIZ(cpa->eoam.numRo);

          if (pEoamCfg->validBitMap & pa_EOAM_VALID_STATS_CTRL)
          {  
            paEoamTfExcCtrlConfig_t   *eoamStatsCtrlCfg = &pEoamCfg->statsCtrl;
            int   i, numProtos = eoamStatsCtrlCfg->numProtoExcl;

            if (numProtos > pa_MAX_ETH_PROTO_EOAM_EXCLUSION)
               return (pa_ERR_CONFIG);

            if (numProtos)
            {
              for ( i = 0; i < numProtos; i++ )
              {
                cpa->eoam.exclEthTypes[i] = SWIZ(eoamStatsCtrlCfg->exclEthTypes[i]);
              }
            }
          } 
        }
    }
  
    /* Destination can be any PDSP, but POST PDSP0 is used since it is lightly loaded */
    *cmdDest = pa_CMD_TX_DEST_5;

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

    Pa_osalMtCsEnter(&mtCsKey);
    Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
    if (csize)
    {
        /* Create the command */
        fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_SYS_CONFIG, 0,
                                    ((ctrl->code == pa_CONTROL_802_1ag_CONFIG) || (ctrl->code == pa_CONTROL_EMAC_PORT_CONFIG))?0:1, csize);

        /* Validity of the destination was already checked (HOST, DISCARD), so no other cases 
        * must be considered */
        if (reply->dest == pa_DEST_HOST)
            fcmd->replyDest = PAFRM_DEST_PKTDMA;
        else
            fcmd->replyDest = PAFRM_DEST_DISCARD;
    
        ccfg = (pafrmCommandSysConfigPa_t *) &(fcmd->cmd);
    }
    
    switch (ctrl->code) 
    {
        case pa_CONTROL_802_1ag_CONFIG:
        {
            pa802p1agDetConfig_t  *pa802p1agDet = &ctrl->params.pa802p1agDetCfg;
            
            ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_802_1AG;
            ccfg->u.pa802p1agDet.ctrlBitMap   = (pa802p1agDet->ctrlBitMap & pa_802_1ag_DETECT_ENABLE)?PAFRM_802_1ag_DET_ENABLE:0;
            ccfg->u.pa802p1agDet.ctrlBitMap  |= (pa802p1agDet->ctrlBitMap & pa_802_1ag_DETECT_STANDARD)?PAFRM_802_1ag_DET_STANDARD:0;
            ccfg->u.pa802p1agDet.ctrlBitMap  =  SWIZ(ccfg->u.pa802p1agDet.ctrlBitMap);
        
            /* 802.1ag detector is at Ingress0 PDSP0 only */
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
      
            /* IPSEC NAT-T detector is at Ingress4 PDSP1 or Ingress1 PDSP1 */
            *cmdDest = (ipsecNatTDet->ctrlBitMap & pa_IPSEC_NAT_T_CTRL_LOC_LUT1)?pa_CMD_TX_DEST_1:pa_CMD_TX_DEST_4;
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
          
            /* GTP-U classification is at Ingress4 PDSP1 only */
            *cmdDest = pa_CMD_TX_DEST_4;
    
            break;
        }
        
        case pa_CONTROL_EMAC_PORT_CONFIG:
        {
            paEmacPortConfig_t     *portCfg = &ctrl->params.emacPortCfg;
            ret = pa_set_emac_port_cfg_frm_cmd(paInst, ccfg, portCfg, cmdDest);
            if (ret != pa_OK)
            {
                *cmdSize = 0;
                return(ret);
            }
            
            break;
        }
        
        case pa_CONTROL_RA_CONFIG:
        {
            Pa_osalMtCsEnter(&mtCsKey);
            Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
        
            pa_ra_global_cfg(&ctrl->params.raCfg, paLObj.pSysRegs);
    
            break;
        }

        case pa_CONTROL_TIME_OFFSET_CONFIG:
        {
          paSetTimeOffset_t  *tsCfg        = &ctrl->params.tOffsetCfg;

          ccfg->cfgCode = PAFRM_SYSTEM_CONFIG_CODE_TIME_OFFSET_CFG;
          /* time offset correction configuration, can be any PDSP */
          ccfg->u.timeOffsetCfg.offset_sec = tsCfg->offset_sec;
          ccfg->u.timeOffsetCfg.offset_ns  = tsCfg->offset_ns;
          
          ccfg->u.timeOffsetCfg.offset_sec = SWIZ(ccfg->u.timeOffsetCfg.offset_sec);
          ccfg->u.timeOffsetCfg.offset_ns  = SWIZ(ccfg->u.timeOffsetCfg.offset_ns);          
          /* Command proc at Ingress 4 PDSP1 */
          *cmdDest = pa_CMD_TX_DEST_4;
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

  pafrmCommand_t         *fcmd;
  pafrmCommandReqStats_t *rstat;
  uint16_t                csize;

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
  fcmd = pa_format_fcmd_header (paInst, cmd, reply, PAFRM_CONFIG_COMMAND_REQ_STATS, 0, 0, csize);

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
        *cmdDest = pa_CMD_TX_DEST_5;
        return (pa_RESOURCE_USE_DENIED);   
      }                     
      
      cstat->bitMap[cntIndex[i] >> 5] |= (1 << (cntIndex[i] % 32)); 
    }
    
    for ( i = 0; i < PA_USR_STATS_BITMAP_SIZE; i++)
      cstat->bitMap[i] = SWIZ(cstat->bitMap[i]);
  }

  /* Usr-defined statistics processing PDSP: Post PDSP0*/
  *cmdDest = pa_CMD_TX_DEST_5;

  return (pa_OK);

}  

/********************************************************************************
 * FUNCTION PURPOSE: Request Sub-System statistics
 ********************************************************************************
 * DESCRIPTION: This API is not supported at PASS Gen2
 ********************************************************************************/
paReturn_t Pa_requestStats (Pa_Handle      handle,
                            uint16_t       doClear, 
                            paCmd_t        cmd, 
                            uint16_t      *cmdSize, 
                            paCmdReply_t  *reply, 
                            int           *cmdDest)
{

  return (pa_API_UNSUPPORTED);

}  /* Pa_requestStats */

/********************************************************************************
 * FUNCTION PURPOSE: Query Sub-System statistics
 ********************************************************************************
 * DESCRIPTION: This API queries sub-system statistics
 ********************************************************************************/
paReturn_t Pa_querySysStats   (Pa_Handle      handle,
                               uint16_t       doClear, 
                               paSysStats_t  *pSysStats)
{
  uint32_t      *pDstCnt = (uint32_t *)pSysStats;
  volatile uint32_t *pSrcCnt;
  uint32_t       i;
  
  if (pSysStats == NULL)
    return(pa_INVALID_INPUT_POINTER);
  
  pSrcCnt = doClear? (uint32_t *) &paLObj.pSysRegs->COLLECT_STATS[PASS_SYS_STATS_BASE]:(uint32_t *)&paLObj.pSysRegs->QUERY_STATS[PASS_SYS_STATS_BASE];
  
  /* Format and copy system counters */
  for (i = 0; i < sizeof(paSysStats_t)/sizeof(uint32_t); i++)
  {
      pDstCnt[i] = pSrcCnt[i];
  }
  
  return (pa_OK);

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
  uint32_t      *pDstCnt[pa_RA_NUM_GROUPS];
  volatile uint32_t *pSrcCnt[pa_RA_NUM_GROUPS];
  uint32_t       i,j;
  
  if (pRaStats == NULL)
    return(pa_INVALID_INPUT_POINTER);
  
  for(i = 0; i < pa_RA_NUM_GROUPS; i++)
    pSrcCnt[i] = doClear? (uint32_t *) &paLObj.pSysRegs->COLLECT_STATS[PASS_RA_GROUP_STATS_BASE(i)]:(uint32_t *) &paLObj.pSysRegs->QUERY_STATS[PASS_RA_GROUP_STATS_BASE(i)];
  
  /* Format and copy system counters */
  for (i = 0; i < pa_RA_NUM_GROUPS; i++)
  {
    pDstCnt[i] = (uint32_t *)&pRaStats->group[i];
    for (j = 0; j < sizeof(paRaGroupStats_t)/sizeof(uint32_t); j++)
    {
        pDstCnt[i][j] = pSrcCnt[i][j];
    }
  }
  
  return (pa_OK);

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

  paAclEntry_t  *aclEntry = (paAclEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, aclHandle);
                         
  volatile    uint32_t *pSrcCnt;
  
  if (pAclStats == NULL)
    return(pa_INVALID_INPUT_POINTER);
  
  /* Refresh ACL Instance for read only */
  Pa_osalBeginMemAccess (aclEntry, sizeof(paL2L3Header_t));
  
  pSrcCnt = doClear? (uint32_t *) &paLObj.pSysRegs->COLLECT_STATS[PASS_ACL_STATS_BASE_ENTRY(aclEntry->hdr.tableIdx)]:
                     (uint32_t *) &paLObj.pSysRegs->QUERY_STATS[PASS_ACL_STATS_BASE_ENTRY(aclEntry->hdr.tableIdx)];
  
  Pa_osalEndMemAccess (aclEntry, sizeof(paL2L3Header_t));
  
  pAclStats->nMatchPackets = pSrcCnt[0];
  pAclStats->nMatchBytes   = pSrcCnt[1];
  
  return (pa_OK);

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

  paFcEntry_t   *fcEntry = (paFcEntry_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, fcHandle);
                         
  volatile    uint32_t *pSrcCnt;
  
  if (pFcStats == NULL)
    return(pa_INVALID_INPUT_POINTER);
  
  /* Refresh PA and ACL Instance for read only */
  Pa_osalBeginMemAccess (fcEntry, sizeof(paL2L3Header_t));
  
  pSrcCnt = doClear? (uint32_t *) &paLObj.pSysRegs->COLLECT_STATS[PASS_FC_STATS_BASE_ENTRY(fcEntry->hdr.tableIdx)]:
                     (uint32_t *) &paLObj.pSysRegs->QUERY_STATS[PASS_FC_STATS_BASE_ENTRY(fcEntry->hdr.tableIdx)];
  
  Pa_osalEndMemAccess (fcEntry, sizeof(paL2L3Header_t));
  
  pFcStats->nMatchPackets = pSrcCnt[0];
  
  return (pa_OK);

} /* Pa_queryFcStats */                            

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
    CSL_Pa_clRegs *pClRegs;
    uint16_t       num64bCounters, num32bCounters;
    uint32_t      *pCounter;
    int            i;
  
    /* non-autonomous reading */
    num64bCounters = paInst->cfg.usrStatsConfig.num64bCounters;
    num32bCounters = numCounters - num64bCounters;
    pClRegs = paLObj.pClRegs[PASS_CLUSTER_POST];
    pCounter = (uint32_t *)&pClRegs->PDSP_SRAM[PAFRM_USR_STATS_OFFSET];
  
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
    *cmdDest = pa_CMD_TX_DEST_5;

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
 * DESCRIPTION: This API is not supported at PASS Gen2
 ****************************************************************************/
paSysStats_t* Pa_formatStatsReply (Pa_Handle  handle,
                                   paCmd_t    cmd)
{
  return (NULL);

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
  int                fProcessAh   = FALSE;      /* Indicate that We are processing AH for SA */
  int                i, start;
  int                cmdBlockSize = 0;
  int                bufAddr = (int) &buf[0];

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
            int        totalPatchSize;
            cmdBlockSize += nrCmdSize;
            
            if ((route->dest == pa_DEST_SRIO) ||
                (route->ctrlBitfield & pa_NEXT_ROUTE_PROC_USR_STATS) ||
                ((route->dest != pa_DEST_SASS) && 
                 ((route->pktType_emacCtrl) || (route->ctrlBitfield & (pa_NEXT_ROUTE_TX_L2_PADDING | pa_NEXT_ROUTE_RPT_TX_TIMESTAMP)))))
            {
                cmdBlockSize += sizeof(nr->word1);
            }

            if (route->ctrlBitfield & pa_NEXT_ROUTE_PROC_NEXT_CMD)
            {
                paPatchInfo_t* patch = &cmdInfo[index+1].params.patch;

                totalPatchSize = patch->totalPatchSize;
                cmdBlockSize += 4;

                /* Detect whether we are processing AH for SA LLD */

                /* Below conditions needs to be met to declare AH process
                 *  (a) The command next to NEXT ROUTE command should be patch command (already detected at the beginning)
                 *  (b) The patch command's totalPatchSize should be zero or patchData == NULL
                 *  (c) There should not be any further commands after patch command, (patch command should be last command)
                 *  (d) Declare AH process flag to TRUE
                 */

                if ( ((totalPatchSize == 0) || (patch->patchData == NULL)) &&
                     (index == (nCmd -2))
                   )
                {
                    fProcessAh = TRUE;
                }
            }

            /* By default any commands sent to SA should be 8 byte aligned.
             * When processing AH, SA LLD requires 16 byte align requirements
             */
            if (fProcessAh == TRUE)
            {
                start = (cmdBlockSize % 16);
            }
            else {
                
                cmdBlockSize += totalPatchSize;
                start = (cmdBlockSize % 8);
            }

            for ( i = start; i > 0; i-=4)
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

            case pa_DEST_SASS_LOC_DMA:   
              pdest = PAFRM_DEST_PKTDMA_LOC;
              break;

            case pa_DEST_RES_1:
              pdest = PAFRM_DEST_NR_ACE0;
              break;

            case pa_DEST_RES_2:
              pdest = PAFRM_DEST_NR_ACE1;
              break;

            case pa_DEST_HOST:
            case pa_DEST_EMAC:
              pdest = (route->dest == pa_DEST_EMAC)?PAFRM_DEST_ETH:PAFRM_DEST_PKTDMA;
              if ((route->pktType_emacCtrl) || 
                  (route->ctrlBitfield & (pa_NEXT_ROUTE_TX_L2_PADDING | pa_NEXT_ROUTE_RPT_TX_TIMESTAMP)) || 
                  (route->dest == pa_DEST_EMAC))
              {
                uint8_t psFlags;
                PASAHO_SET_E (nr, 1);
                psFlags = (route->pktType_emacCtrl & pa_EMAC_CTRL_CRC_DISABLE)?
                          PAFRM_ETH_PS_FLAGS_CRC_PRESENT:0;
                psFlags |= ((route->pktType_emacCtrl & pa_EMAC_CTRL_PORT_MASK) << PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
                PASAHO_SET_PKTTYPE  (nr, psFlags);
                if (route->ctrlBitfield & pa_NEXT_ROUTE_TX_L2_PADDING)
                {
                    PASAHO_SET_TX_PADDING (nr, 1);
                }
                
                if (route->ctrlBitfield & pa_NEXT_ROUTE_RPT_TX_TIMESTAMP)
                {
                    PASAHO_SET_RPT_TX_TIMESTAMP (nr, 1);
                }

                nrCmdSize += sizeof(nr->word1);
              }
              break;

            case pa_DEST_SRIO:
              pdest = PAFRM_DEST_NR_SRIO;
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
          PASAHO_CHKCRC_SET_CTRL       (ptx, ctrl);
          PASAHO_CHKCRC_SET_START      (ptx, crc->startOffset);
          PASAHO_CHKCRC_SET_CRCSIZE    (ptx, crc->crcSize);
          PASAHO_CHKCRC_SET_LEN        (ptx, crc->len);
          PASAHO_CHKCRC_SET_RESULT_OFF (ptx, crc->crcOffset);
          PASAHO_CHKCRC_SET_INITVAL32  (ptx, crc->initValue);

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

          if(fAlignDetect && !fProcessAh)
            cmdBlockSize+=(patch->totalPatchSize + 4);
            
          fProcessAh = FALSE;

        }
        break;


      case pa_CMD_TX_CHECKSUM:
        {
          paTxChksum_t  *chk = &cmdInfo[index].params.chksum;

          ptx    = (pasahoComChkCrc_t *)(bufAddr + cmdOffset);
          PASAHO_SET_CMDID             (ptx, PASAHO_PAMOD_CMPT_CHKSUM);
          PASAHO_CHKCRC_SET_START      (ptx, chk->startOffset);
          PASAHO_CHKCRC_SET_LEN        (ptx, chk->lengthBytes);
          PASAHO_CHKCRC_SET_RESULT_OFF (ptx, chk->resultOffset);
          PASAHO_CHKCRC_SET_INITVAL    (ptx, chk->initialSum);

          if (chk->negative0)
              PASAHO_CHKCRC_SET_NEG0 (ptx, 1);
          else
              PASAHO_CHKCRC_SET_NEG0 (ptx, 0);

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

      case pa_CMD_EF_OP:
        {
          paCmdEfOp_t  *efOp = &cmdInfo[index].params.efOp;
          pasahoComEfOp_t *pEfCmd = (pasahoComEfOp_t *)(bufAddr + cmdOffset);
          uint16_t ctrl = 0;

          if (nCmd != 1)
          {
            /* This command can not be combined with any other command */
            return(pa_ERR_CONFIG); 
          }

          if (efOp->ctrlBitfield & pa_EF_OP_CMD_FC_LOOKUP)
          {
            ctrl |= PASAHO_HDR_EF_OP_CTRL_FC;
          }
          else
          {
            if (efOp->ctrlBitfield & pa_EF_OP_CMD_VALID_LVL1)
            {
                ctrl |= PASAHO_HDR_EF_OP_CTRL_LVL1_REC;
                PASAHO_EF_OP_SET_LVL1_REC(pEfCmd, (uint8_t)efOp->lvl1Index);
            } 

            if (efOp->ctrlBitfield & pa_EF_OP_CMD_VALID_LVL2)
            {
                ctrl |= PASAHO_HDR_EF_OP_CTRL_LVL2_REC;
                PASAHO_EF_OP_SET_LVL2_REC(pEfCmd, (uint8_t)efOp->lvl2Index);
            }

            if (efOp->ctrlBitfield & pa_EF_OP_CMD_VALID_LVL3)
            {
                ctrl |= PASAHO_HDR_EF_OP_CTRL_LVL3_REC;
                PASAHO_EF_OP_SET_LVL3_REC(pEfCmd, (uint8_t)efOp->lvl3Index);
            }

            if (efOp->ctrlBitfield & pa_EF_OP_CMD_VALID_LVL4)
            {
                ctrl |= PASAHO_HDR_EF_OP_CTRL_LVL4_REC;
                PASAHO_EF_OP_SET_LVL4_REC(pEfCmd, (uint8_t)efOp->lvl4Index);
            }

          } 

          PASAHO_SET_CMDID (pEfCmd, PASAHO_PAMOD_EF_OP);
          PASAHO_EF_OP_SET_CTRL(pEfCmd, ctrl);
          PASAHO_EF_OP_SET_L2_OFFSET(pEfCmd, efOp->l2Offset);
          PASAHO_EF_OP_SET_L3_OFFSET(pEfCmd, efOp->l3Offset);
          PASAHO_EF_OP_SET_L3_OFFSET2(pEfCmd, efOp->l3Offset2);
          PASAHO_EF_OP_SET_IPSEC_OFFSET(pEfCmd, efOp->ipsecOffset);
          PASAHO_EF_OP_SET_END_OFFSET(pEfCmd, efOp->endOffset);
          cmdOffset += sizeof (pasahoComEfOp_t);

        }
        break;

      case pa_CMD_PATCH_TIME:
        {          
          paPatchTime_t  *insTime            = &cmdInfo[index].params.patchTime;
          pasahoInsMsgTime_t *fwInsMsgTime = (pasahoInsMsgTime_t *)(bufAddr + cmdOffset);
          PASAHO_SET_CMDID (fwInsMsgTime, PASAHO_PAMOD_PATCH_MSG_TIME);
          PASAHO_SET_SUB_CODE_INS_MSG_TIME(fwInsMsgTime);     
          PASAHO_SET_INS_OFFSET_MSG_TIME(fwInsMsgTime, insTime->startOffset);
          cmdOffset += sizeof (pasahoInsMsgTime_t);
          if(fAlignDetect)
            cmdBlockSize+=sizeof(pasahoInsMsgTime_t);           
        }
        break;
      case pa_CMD_PATCH_COUNT:
        {          
          paPatchCount_t  *insCount          = &cmdInfo[index].params.patchCount;
          pasahoInsMsgCount_t *fwInsMsgCnt = (pasahoInsMsgCount_t *)(bufAddr + cmdOffset);
          PASAHO_SET_CMDID (fwInsMsgCnt, PASAHO_PAMOD_PATCH_MSG_COUNT);
          PASAHO_SET_SUB_CODE_INS_MSG_COUNT(fwInsMsgCnt);
          PASAHO_SET_INS_OFFSET_MSG_COUNT(fwInsMsgCnt, insCount->startOffset);
          PASAHO_SET_COUNTER_INDEX_FOR_MSG_COUNT(fwInsMsgCnt, insCount->countIndex);
          cmdOffset += sizeof (pasahoInsMsgCount_t);
          if(fAlignDetect)
            cmdBlockSize+=sizeof(pasahoInsMsgCount_t);           
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

int      debugPdspMaxWait = 0;
uint32_t debugPdspAddr = 0;

//static int pa_pdsp_run (CSL_Pa_pdspControl_statusRegs *pdsp, CSL_Pa_ssMailboxRegs volatile *mbox) {
int pa_pdsp_run (CSL_Pa_pdspControl_statusRegs *pdsp, volatile CSL_Pa_ssMailboxRegs *mbox){
  int i;

  /* Check for enabled PDSP */
  if ( (pdsp->CONTROL & CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK) == CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK)
    return (PA_PDSP_ALREADY_ACTIVE);

  /* Clear the mailbox */
  mbox->MBOX_SLOT[0] = 0;

  /* Enable the PDSP */
  pdsp->CONTROL |= (CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK |
                    CSL_PA_PDSP_CONTROL_SOFT_RST_N_MASK);
            
  /* Wait for the mailbox to become non-zero */
  for (i = 0; i < PA_MAX_PDSP_ENABLE_LOOP_COUNT; i++)  {
    if (mbox->MBOX_SLOT[0] != 0)  {
        if ( i > debugPdspMaxWait)
        {
            debugPdspMaxWait = i;
            debugPdspAddr = (uint32_t)mbox;
        }
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
	  
  Pa_osalMtCsEnter(&mtCsKey);
  
  if (newState == pa_STATE_RESET)  {

    /* Put each of the PDSPs into reset (PC = 0) and reset timers*/
    for (i = 0; i < PASS_NUM_PDSPS; i++)  {
      paLObj.pPpuRegs[i]->PDSP_CONTROL_STATUS.CONTROL = 0;
      paLObj.pPpuRegs[i]->CP_TIMER.TIMER_CNTRL_REG = 0;
    }

    /* Reset LUT2 */
    paLObj.pPpuRegs[PASS_INGRESS4_PDSP1]->LUT2.CLR_TABLE = CSL_PA_LUT2_CLR_TABLE_GO_MASK;

    /* Reset statistics */
    //passRegs->STATS.STATS_SOFT_RESET = 1;

    ret = pa_STATE_RESET;

  }

  else if (newState == pa_STATE_ENABLE)  {
    int          res;

    ret = pa_STATE_ENABLE;
     
    /* Do nothing if a pdsp is already out of reset. If any PDSPs are out of reset
     * a global init is not performed */
    for (i = 0; i < PASS_NUM_PDSPS; i++)  {

      res = pa_pdsp_run (&(paLObj.pPpuRegs[i]->PDSP_CONTROL_STATUS), &(paLObj.pSysRegs->MAILBOX[i]));
    
      if (res == PA_PDSP_NO_RESTART)  {
        ret = pa_STATE_ENABLE_FAILED;
      }

    }

    /* Start all PDSP can do it */
    for (i = 0; i < PASS_NUM_PDSPS; i++)
      paLObj.pSysRegs->MAILBOX[i].MBOX_SLOT[0] = 0;   /* Let PDSP go */

  }

  else if (newState == pa_STATE_QUERY)  {

    uint16_t enable  = FALSE;
    uint16_t reset   = FALSE;

    for (i = 0; i < PASS_NUM_PDSPS; i++)  {
      if ( (paLObj.pPpuRegs[i]->PDSP_CONTROL_STATUS.CONTROL & CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK) == 
                     (CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK))
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
   
  CSL_Pa_ppuRegs *pPpuRegs;
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

  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  
  pPpuRegs = paLObj.pPpuRegs[PASS_INGRESS0_PDSP0];
     
  if (cfgInfo->enable) {
    
    pPpuRegs->CP_TIMER.TIMER_LOAD_REG  = 0xFFFF;
    pPpuRegs->CP_TIMER.TIMER_CNTRL_REG = CSL_CP_TIMER16_TIMER_CNTRL_REG_GO_MASK       |  
                                         CSL_CP_TIMER16_TIMER_CNTRL_REG_MODE_MASK     |
                                         (cfgInfo->factor << CSL_CP_TIMER16_TIMER_CNTRL_REG_PRESCALE_SHIFT)    |
                                         CSL_CP_TIMER16_TIMER_CNTRL_REG_PSE_MASK;
  } 
  else  {
    pPpuRegs->CP_TIMER.TIMER_LOAD_REG = 0;
    pPpuRegs->CP_TIMER.TIMER_CNTRL_REG = 0;
  }
  
  Pa_osalMtCsExit(mtCsKey);
    
  return (pa_OK);

} /* Pa_configTimestamp */

/*****************************************************************************************
 * FUNCTION PURPOSE: Return the 64-bit system timestamp
 *****************************************************************************************
 * DESCRIPTION: This function is used to inquery the 48-bit system timestamp.
 *****************************************************************************************/
paReturn_t Pa_getTimestamp  (Pa_Handle            iHandle, 
                             paTimestamp_t        *pTimestamp)
{
   
  CSL_Pa_ppuRegs *pPpuRegs;
  uint32_t mtCsKey;

  /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
  
  if (pTimestamp == NULL)
    return(pa_INVALID_INPUT_POINTER);
      
  Pa_osalMtCsEnter(&mtCsKey);
  
  pPpuRegs = paLObj.pPpuRegs[PASS_INGRESS0_PDSP0];
  
  pTimestamp->lo = 0x0000FFFF - pPpuRegs->CP_TIMER.TIMER_VALUE_REG;
  pTimestamp->hi = paLObj.pSysRegs->SRAM[PAFRM_SYS_TIMESTAMP_OFFSET];
  pTimestamp->hi_hi = (uint16_t) paLObj.pSysRegs->SRAM[PAFRM_SYS_TIMESTAMP_OFFSET_HI];
      
  Pa_osalMtCsExit(mtCsKey);
  
  return (pa_OK);

} /* Pa_getTimestamp */

/* Moved the constant defintions outside this file *
 * This helps to generate the bin files required for
 * linux */
extern const uint32_t pap_pdsp_const_reg_map[PASS_NUM_PDSPS][32];

/***********************************************************************************************
 * FUNCTION PURPOSE: Download a PDSP image
 ***********************************************************************************************
 * DESCRIPTION: A PDSP image is downloaded. The PDSP remains in reset.
 ***********************************************************************************************/
paReturn_t Pa_downloadImage (Pa_Handle iHandle, int modId, void* image, int sizeBytes)
{

  CSL_Pa_ppuRegs *pPpuRegs;
  paReturn_t ret = pa_OK;
  uint32_t mtCsKey;
  int i;
  volatile uint32_t *image_word;

  /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
  
  if(image == NULL)
    return(pa_INVALID_INPUT_POINTER);

  /* Verify the specified PDSP is valid */
  if ((modId < 0)  || (modId >= PASS_NUM_PDSPS))
    return (pa_ERR_CONFIG);

  /* Verify the image size is in range */
  if ((sizeBytes < 0)  || (sizeBytes > 0x3000))
    return (pa_ERR_CONFIG);
  
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);

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
 
  pPpuRegs = paLObj.pPpuRegs[modId];

  /* Make sure the PDSP is disabled */
  if ( (pPpuRegs->PDSP_CONTROL_STATUS.CONTROL & CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK)  ==
                                               (CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK)   )
  {
    ret = pa_SYSTEM_STATE_INVALID;
    goto Pa_downloadImage_end;
  }  

  /* Copy the image */
  image_word = (uint32_t *) image;
  for ( i = 0; i < (sizeBytes/4); i++)
  {
      pPpuRegs->PDSP_IRAM[i] = image_word[i];
  }

  /* Initialize the programmable constant registers */
  for(i = 0; i < 32; i++)
    pPpuRegs->PDSP_DEBUG.ICTE[i] = pap_pdsp_const_reg_map[modId][i];
  
Pa_downloadImage_end:
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
  CSL_Pa_clRegs *pClRegs;

    /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
  
  if(pVersion == NULL)
    return(pa_INVALID_INPUT_POINTER);
  
  /* Verify the specified PDSP is valid */
  if ((modId < 0)  || (modId >= PASS_NUM_PDSPS))
    return (pa_ERR_CONFIG);
  
  pClRegs  = paLObj.pClRegs[pa_pdsp_map[modId].clNum];
  *pVersion = pClRegs->PDSP_SRAM[PAFRM_PDSP_VERSION_OFFSET(pa_pdsp_map[modId].verBase, 
                                                           pa_pdsp_map[modId].pdspNum)];
  
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
  CSL_Pa_clRegs         *paClOutRegs, *paClInRegs;
  paReturn_t             ret = pa_OK;
  uint32_t              *addr;
  uint32_t               bitmap;

  /* Check for PA Base address null configurations */
  if (paLObj.cfg.baseAddr ==  (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
  
  if(dbgInfo == NULL)
    return(pa_INVALID_INPUT_POINTER);

  paClOutRegs = (CSL_Pa_clRegs *)paLObj.pClRegs[PASS_CLUSTER_INGRESS1];
  paClInRegs  = (CSL_Pa_clRegs *)paLObj.pClRegs[PASS_CLUSTER_INGRESS4];

  /* Read the reassembly debug snap shot from firmware and report to the caller */
  switch (dbgInfo->debugInfoType)
  {
      case pa_DBG_INFO_TYPE_REASSEMBLY_ENABLE:
          addr   = (uint32_t *)&paClOutRegs->PDSP_SRAM[PAFRM_OUT_REASSEM_CTRL_BLK_OFFSET];
          bitmap = pa_read_reassem_control_blk(addr, &dbgInfo->u.reassemContext.outer);

          /* Set the traffic flow array to inactive state */
          memset (&dbgInfo->u.reassemContext.outer.traffic_flow[0], 0xFF, sizeof (pa_ReassemblyFlow_t) * 32 );
          addr   = (uint32_t *)&paClOutRegs->PDSP_SRAM[PAFRM_OUT_REASSEM_TRAFFIC_FLOW_OFFSET];
          pa_read_reassem_traffic_flow (addr, &dbgInfo->u.reassemContext.outer.traffic_flow[0], bitmap);

          addr   = (uint32_t *)&paClInRegs->PDSP_SRAM[PAFRM_IN_REASSEM_CTRL_BLK_OFFSET];
          bitmap = pa_read_reassem_control_blk(addr, &dbgInfo->u.reassemContext.inner);

          memset (&dbgInfo->u.reassemContext.inner.traffic_flow[0], 0xFF, sizeof (pa_ReassemblyFlow_t) * 32 );
          addr   = (uint32_t *)&paClInRegs->PDSP_SRAM[PAFRM_IN_REASSEM_TRAFFIC_FLOW_OFFSET];
          pa_read_reassem_traffic_flow (addr, &dbgInfo->u.reassemContext.inner.traffic_flow[0], bitmap);
        break;
    default:
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
  paReturn_t         ret = pa_INVALID_INPUT_HANDLE;
    
  vlnk = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, vlinkHdl);  

  Pa_osalBeginMemAccess ((void *) vlnk, sizeof (paVirtualLnk_t));
  // Look for free entries in the virtual link table
  if (vlnk->hdr.status == PA_TBL_STAT_ACTIVE)
  {
    *lnkId = vlnk->hdr.tableIdx;
    ret = pa_OK;
  }
  Pa_osalEndMemAccess ((void *) vlnk, sizeof (paVirtualLnk_t));

  return (ret);
}

/* Nothing past this point */
