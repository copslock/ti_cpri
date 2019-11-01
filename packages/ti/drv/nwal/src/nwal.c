/******************************************************************************
 * FILE PURPOSE:  Main Routines for NWAL LLD
 ******************************************************************************
 * FILE NAME:   nwal.c
 *
 * DESCRIPTION: NWAL LLD run time API and internal functions
 *              which is not SA LLD dependent
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2012
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

#include <string.h>
#include "nwal_util.h"
#include "nwal_osal.h"
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/csl/csl_qm_queue.h>
#include <stdio.h>

extern nssGlobalConfigParams_t nssGblCfgParams;
extern nwalProcessContext_t     nwalProcessCtx;

/* The define identifies code in NWAL where PS command related adjustments
 * are currently being done. In future this should move to PALLD or
 * alternatively need to be done in a different way
#define NWAL_PA_PS_CMD_WROUND 1
#define NWAL_PA_PS_CMD_WROUND2 1
 */
/* Debug define  for routing packet to host from NetCP during transmit
 * #define NWAL_NETCP_TX_PKT_DEBUG 1
 *#
 *
 */

#ifdef NWAL_DUMP_PKT
 /****************************************************************************/
static void  nwal_dump_buf_32bit
(
    uint8_t*                      buf,
    uint32_t                      buf_length
)
{
    uint8_t                       count = 0;
    uint16_t                      dump_size;
    uint32_t*                     tmp_buf;
    uint8_t                       row_count;

    dump_size = buf_length/4 ;

    tmp_buf = (uint32_t *)(buf);

    printf("NWAL *:  - 32 bit word hex Length: %d Start \n",buf_length);
    do
    {
        row_count = (dump_size - count);

        if(row_count == 0)
        {
            break;
        }

        if(row_count > 4)
        {
            row_count = 4;
        }

        switch (row_count)
        {
            case 4:
            {
                printf("NWAL *:%02d : %08x    %08x    %08x    %08x \n",
                      count,tmp_buf[0],tmp_buf[1],tmp_buf[2],tmp_buf[3]);
                break;
            }
            case 3:
            {
                printf("NWAL *: %02d : %08x    %08x    %08x \n",
                      count,tmp_buf[0],tmp_buf[1],tmp_buf[2]);
                break;
            }

            case 2:
            {
                printf("NWAL *: %02d : %08x    %08x \n",
                      count,tmp_buf[0],tmp_buf[1]);
                break;
            }

            case 1:
            {
                printf("NWAL *: %02d : %08x \n",
                      count,tmp_buf[0]);
                break;
            }

            default:
            {
                /* Should never reach here */
                printf("NWAL *: Internal Error in nwal_dump_buf_32bit().Row Count: %d \n",
                    row_count);
                return;
            }
        }

        tmp_buf = tmp_buf + row_count;
        count = count +  row_count;

    }while(count < dump_size);

    printf("NWAL *:  - Byte hex Dump End \n");

}
/****************************************************************************
 * FUNCTION:    nwal_dump_buf
 ****************************************************************************
 * DESCRIPTION: This function will dump the contents of buffer
 *
 ****************************************************************************/
static void  nwal_dump_buf
(
    uint8_t*                      buf,
    uint32_t                      buf_length
)
{
    uint8_t                       count = 0;
    uint16_t                      dump_size;
    uint8_t*                     tmp_buf;
    uint8_t                       row_count;


    dump_size = buf_length ;

    tmp_buf = (uint8_t *)(buf);

    printf("NWAL *:  - 8 bit word hex Length: %d Start \n",buf_length);
    do
    {
        row_count = (dump_size - count);

        if(row_count == 0)
        {
            break;
        }

        if(row_count > 4)
        {
            row_count = 4;
        }

        switch (row_count)
        {
            case 4:
            {
                printf("NWAL *:%02d : %02x    %02x    %02x    %02x \n",
                      count,tmp_buf[0],tmp_buf[1],tmp_buf[2],tmp_buf[3]);
                break;
            }
            case 3:
            {
                printf("NWAL *: %02d : %02x    %02x    %02x \n",
                      count,tmp_buf[0],tmp_buf[1],tmp_buf[2]);
                break;
            }

            case 2:
            {
                printf("NWAL *: %02d : %02x    %02x \n",
                      count,tmp_buf[0],tmp_buf[1]);
                break;
            }

            case 1:
            {
                printf("NWAL *: %02d : %02x \n",
                      count,tmp_buf[0]);
                break;
            }

            default:
            {
                /* Should never reach here */
                printf("NWAL *: Internal Error in nwal_dump_buf().Row Count: %d \n",
                    row_count);
                return;
            }
        }

        tmp_buf = tmp_buf + row_count;
        count = count +  row_count;

    }while(count < dump_size);

    printf("NWAL *:  - Byte hex Dump End \n");

}
#endif
/********************************************************************
 * FUNCTION PURPOSE: Compares two uint8_t array for being identical
 ********************************************************************
 * DESCRIPTION: Returns nwal_TRUE if both arrays match
 ********************************************************************/
nwal_Bool_t nwalCompareByteArray (uint8_t* v1,uint8_t* v2, int n)
{
  int i;

  for (i = 0; i < n; i++)
    if (v1[i] != v2[i])
      return (nwal_FALSE);

  return (nwal_TRUE);

}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_prepMacHdr Prepare MAC header
 *                    Supported modes:DIX,DIX with VLAN,
 *                    802.3 and 802.3 with VLAN
 *                    Return count of the header bytes
 ********************************************************************
 ********************************************************************/
uint8_t nwal_prepMacHdr(uint8_t*                    pHdrBase,
                        uint16_t                    bufLen,
                        nwalMacOpt_t*               pMacOpt,
                        uint16_t                    ethProtoType,
                        uint8_t*                    pSrc,
                        uint8_t*                    pDst)
{
    uint8_t*    pTmpBuf;
    uint8_t     hdrLen = NWAL_ETH_DIX_HDR_LEN;
    uint16_t    tmp;
    uint8_t     vlanPrio = 0;

    pTmpBuf = pHdrBase + NWAL_ETH_OFFSET_DEST_MAC;
    memcpy(pTmpBuf,pDst,NWAL_ETH_MAC_ADDR_LEN);

    pTmpBuf = pHdrBase + NWAL_ETH_OFFSET_SRC_MAC;
    memcpy(pTmpBuf,pSrc,NWAL_ETH_MAC_ADDR_LEN);

    pTmpBuf = pHdrBase + NWAL_ETH_VLAN_TAG;
    if((pMacOpt->validParams & NWAL_MAC_OPT_VALID_PARAM_VLAN_ID)
        ==NWAL_MAC_OPT_VALID_PARAM_VLAN_ID)
    {
        if((pMacOpt->validParams & NWAL_MAC_OPT_VALID_PARAM_VLAN_PRIO)
            == NWAL_MAC_OPT_VALID_PARAM_VLAN_PRIO)
        {
            vlanPrio = pMacOpt->vlanPrio;
        }

        /* Update Ethertype to VLAN tag */
         nwalWrite16bits_m(pTmpBuf,0,NWAL_ETH_TYPE_VLAN_TAG);
         pTmpBuf++;
         pTmpBuf++;

         tmp = (((vlanPrio & NWAL_ETH_VLAN_PRIO_MASK) <<
                  NWAL_ETH_VLAN_PRIO_SHIFT) |
                ((0x00 & NWAL_ETH_VLAN_RIF_MASK) << NWAL_ETH_VLAN_RIF_SHIFT) |
                (pMacOpt->vlanId & NWAL_ETH_VLAN_ID_MASK));
         nwalWrite16bits_m(pTmpBuf,0,tmp);

         hdrLen += NWAL_ETH_VLAN_HDR_SIZE;
    }

    if(pMacOpt->frameFormat == NWAL_MAC_OPT_FRAME_FORMAT_802_3)
    {
        /* Frame length */
        nwalWrite16bits_m(pTmpBuf,0,0);
        pTmpBuf++;
        pTmpBuf++;

        /* LLC Header */
        nwalWrite16bits_m(pTmpBuf,0,0xAAAA);
        pTmpBuf++;
        pTmpBuf++;

        nwalWrite16bits_m(pTmpBuf,0,0x0300);
        pTmpBuf++;
        pTmpBuf++;

        /* Org Code */
        nwalWrite16bits_m(pTmpBuf,0,0);
        pTmpBuf++;
        pTmpBuf++;
        hdrLen += NWAL_ETH_802_3_HDR_SIZE;
    }

    /* Packet Type */
    nwalWrite16bits_m(pTmpBuf,0,ethProtoType);

    return(hdrLen);
}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_updateIpInitCksum Prepare Initial IP
 *                    checksum. If IP Length is passed it would be updated
 *                    before computing checksum
 ********************************************************************
 ********************************************************************/
static inline void nwal_updateIpInitLen(uint8_t *pIpheader,
                                        uint16_t ipLength)
{
      pIpheader[NWAL_IPV4_OFFSET_LEN] = (ipLength >> 8) & 0xff;
      pIpheader[NWAL_IPV4_OFFSET_LEN+1] = (ipLength & 0xff);
}

/********************************************************************
 *  FUNCTION PURPOSE: Ones complement addition utility
 ********************************************************************
 ********************************************************************/
static inline uint16_t nwal_utilOnesComplementAdd (uint16_t v1, uint16_t v2)
{
  uint32_t result;

  result = (uint32_t)v1 + (uint32_t)v2;
  result = (result >> 16) + (result & 0xffff);

  return ((uint16_t)result);
}

/********************************************************************
 *  FUNCTION PURPOSE: Ones complement checksum utility
 ********************************************************************
 ********************************************************************/
static inline uint16_t nwal_utilOnesCompChkSum (uint8_t  *p, uint32_t nwords)
{
  uint32_t chksum = 0;
  uint16_t v;
  uint32_t i;
  uint32_t j;


  for (i = j = 0; i < nwords; i++, j+=2)  {
    v = (p[j] << 8) | p[j+1];
    chksum += v;
  }

  chksum = (chksum >> 16) + (chksum & 0xffff);
  chksum = (chksum >> 16) + (chksum & 0xffff);

  return ((uint16_t)chksum);
} /* utilOnesCompChkSum */

/********************************************************************
 *  FUNCTION PURPOSE: Update ones complement checksum with additional
 *                    init checksum
 ********************************************************************
 ********************************************************************/
static inline uint16_t nwal_utilUpdOnesCompChkSum ( uint8_t     *p,
                                                    uint16_t    initChkSum,
                                                    uint32_t    nwords)
{
  uint16_t chksum = 0;
  uint16_t v;
  uint32_t i;
  uint32_t j;

  chksum = initChkSum;

  for (i = j = 0; i < nwords; i++, j+=2)  {
    v = (p[j] << 8) | p[j+1];
    chksum = nwal_utilOnesComplementAdd (chksum, v);
  }
  return (chksum);
} /* utilOnesCompChkSum */

/**************************************************************************************
 * FUNCTION PURPOSE: Compute ipv4 psudo checksum
 **************************************************************************************
 * DESCRIPTION: Compute ipv4 psudo checksum
 **************************************************************************************/
static inline uint16_t nwal_utilGetIpv4PsudoChkSum (uint8_t *data, uint16_t payloadLen)
{
  uint16_t psudo_chksum;

  psudo_chksum =
      nwal_utilOnesCompChkSum (&data[NWAL_IPV4_OFFSET_SRC_ADDR], 4);
  psudo_chksum =
      nwal_utilOnesComplementAdd(psudo_chksum,
                                 (uint16_t) data[NWAL_IPV4_OFFSET_PROTO]);
  psudo_chksum =
      nwal_utilOnesComplementAdd(psudo_chksum, payloadLen);

  return (psudo_chksum);

} /* utilGetIpv4PsudoChkSum */

/**************************************************************************************
 * FUNCTION PURPOSE: Compute ipv6 psudo checksum
 **************************************************************************************
 * DESCRIPTION: Compute ipv6 psudo checksum
 **************************************************************************************/
static inline uint16_t nwal_utilGetIpv6PsudoChkSum (uint8_t *data, uint16_t payloadLen)
{
  uint16_t psudo_chksum;

  psudo_chksum =
      nwal_utilOnesCompChkSum (&data[NWAL_IPV6_OFFSET_SRC_ADDR], 16);
  psudo_chksum =
      nwal_utilOnesComplementAdd(psudo_chksum, payloadLen);
  psudo_chksum =
      nwal_utilOnesComplementAdd(psudo_chksum, 0);
  psudo_chksum =
      nwal_utilOnesComplementAdd(psudo_chksum,
                                 (uint16_t) data[NWAL_IPV6_OFFSET_NEXT_HDR]);

  return (psudo_chksum);
} /* utilGetIpv6PsudoChkSum */

/**************************************************************************************
 * FUNCTION PURPOSE: Compute ip psudo checksum
 **************************************************************************************
 * DESCRIPTION: Compute ip psudo checksum
 **************************************************************************************/
uint16_t nwal_utilGetIpPsudoChkSum (uint8_t *ip, uint16_t payloadLen)
{
  uint16_t psudo_chksum;

  if ((ip[0] & 0xF0) == 0x40)
  {
      psudo_chksum = nwal_utilGetIpv4PsudoChkSum(ip, payloadLen);
  }
  else
  {
      psudo_chksum = nwal_utilGetIpv6PsudoChkSum(ip, payloadLen);
  }
  return (psudo_chksum);
} /* utilGetIpPsudoChkSum */

#ifdef NWAL_LIB_ENABLE_PROFILE
unsigned int nwal_IpCheckSum_prof_sum = 0;
unsigned int nwal_IpCheckSum_prof_count= 0;
unsigned int nwal_UdpCheckSum_prof_sum = 0;
unsigned int nwal_UdpCheckSum_prof_count = 0;
#endif

/**************************************************************************************
 * FUNCTION PURPOSE: Compute and insert IP Header checksum
 **************************************************************************************
 * DESCRIPTION: Compute and insert IP Header checksum
 **************************************************************************************/
void nwal_utilCompIpChksums (uint8_t *           pIpheader)
{
    uint16_t    sum;
#ifdef NWAL_LIB_ENABLE_PROFILE
    unsigned int            countBegin,countEnd;
#endif
#ifdef NWAL_LIB_ENABLE_PROFILE
    countBegin = nwal_read_clock();
#endif
    /* reset the checksum field to zero */
    pIpheader[NWAL_IPV4_OFFSET_HDR_CHKSUM] = 0;
    pIpheader[NWAL_IPV4_OFFSET_HDR_CHKSUM+1] = 0;
    sum = ~nwal_utilOnesCompChkSum (pIpheader, NWAL_IPV4_HDR_LEN_BYTES/2);

    pIpheader[NWAL_IPV4_OFFSET_HDR_CHKSUM] = sum >> 8;
    pIpheader[NWAL_IPV4_OFFSET_HDR_CHKSUM+1] = sum & 0x00ff;

#ifdef NWAL_LIB_ENABLE_PROFILE
    countEnd = nwal_read_clock();
    nwal_IpCheckSum_prof_sum += (countEnd - countBegin);
    nwal_IpCheckSum_prof_count++;
#endif
    return;
}

/**************************************************************************************
 * FUNCTION PURPOSE: Compute and insert UDP checksum
 **************************************************************************************
 * DESCRIPTION: Compute and insert UDP checksum
 *              Currently assumes all payload being in one
 **************************************************************************************/
void nwal_utilCompUdpChksums (nwalTxPktInfo_t*  pPktInfo,
                              uint8_t *         pUdpHeader,
                              Cppi_HostDesc*    pPloadDesc)
{
    int32_t             pktLen;
    uint16_t            sum;
    Cppi_HostDesc*      ptrDesc;
    uint8_t*            pPloadBuffPtr;
    uint32_t            ploadBuffLen;
#ifdef NWAL_LIB_ENABLE_PROFILE
    unsigned int            countBegin,countEnd;
#endif
#ifdef NWAL_LIB_ENABLE_PROFILE
    countBegin = nwal_read_clock();
#endif

    pktLen =
    ((((int32_t)pUdpHeader[NWAL_UDP_OFFSET_LEN] << 8) |
       (int32_t)pUdpHeader[NWAL_UDP_OFFSET_LEN+1]) + 1) & ~1;

    /* replace the checksum with the pseudo header checksum */
    pUdpHeader[NWAL_UDP_OFFSET_CHKSUM] =
                            pPktInfo->pseudoHdrChecksum >> 8;
    pUdpHeader[NWAL_UDP_OFFSET_CHKSUM+1] =
                            pPktInfo->pseudoHdrChecksum & 0xff;


    sum = nwal_utilOnesCompChkSum (pUdpHeader,
                                   NWAL_UDP_HDR_LEN_BYTES>>1);
    ptrDesc = pPloadDesc;

    while (ptrDesc != NULL)
    {
        Cppi_getData (Cppi_DescType_HOST,
                      (Cppi_Desc*)ptrDesc,
                      &pPloadBuffPtr,
                      &ploadBuffLen);
        sum = nwal_utilUpdOnesCompChkSum(pPloadBuffPtr,
                                         sum,
                                         ploadBuffLen);
        ptrDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST,
                                                (Cppi_Desc *)ptrDesc);
    }

    if (pktLen & 1)
    {
        sum = nwal_utilOnesComplementAdd(sum,
                                         pPloadBuffPtr[ploadBuffLen-1] << 8);
    }
    sum = ~sum;

    if(sum == 0)
       sum = 0xFFFF;


    pUdpHeader[NWAL_UDP_OFFSET_CHKSUM] = sum >> 8;
    pUdpHeader[NWAL_UDP_OFFSET_CHKSUM+1] = sum & 0x00ff;


#ifdef NWAL_LIB_ENABLE_PROFILE
    countEnd = nwal_read_clock();
    nwal_UdpCheckSum_prof_sum += (countEnd - countBegin);
    nwal_UdpCheckSum_prof_count++;
#endif
    return;
}


/********************************************************************
 *  FUNCTION PURPOSE: nwal_prepIpv6Hdr Prepare IPv6 header
 *                    Optional headers are not handled
 ********************************************************************
 ********************************************************************/
nwal_Bool_t nwal_prepIpv6Hdr( uint8_t*  pHdrBase,
                              uint16_t  bufLen,
                              uint8_t   trafficClass,
                              uint32_t  flow,
                              uint8_t   proto,
                              uint8_t   hopLimit,
                              uint8_t*  pSrc,
                              uint8_t*  pDst)
{
    uint16_t    tmp1;
    uint16_t    tmp2;
    uint8_t*    pTmpBuf;

    if(bufLen< NWAL_IPV6_HDR_LEN_BYTES)
    {
        /* Error */
        return nwal_FALSE;
    }

    memset(pHdrBase,0,NWAL_IPV6_HDR_LEN_BYTES);

    /* Version, Traffic Class and Flow Label Hi */
    tmp1 =
        ((NWAL_IPV6_VERSION & NWAL_IPV6_VER_MASK) << NWAL_IPV6_VER_SHIFT);

    tmp2 = (trafficClass & NWAL_IPV6_TRAFFIC_CLASS_MASK);
    tmp1 |= (tmp2 << NWAL_IPV6_TRAFFIC_CLASS_SHIFT);
    tmp1 |= ((flow & NWAL_IPV6_FLOW_LABEL_HI_MASK) >>
                  NWAL_IPV6_FLOW_LABEL_HI_SHIFT);
    nwalWrite16bits_m(pHdrBase,NWAL_IPV6_OFFSET_VER_CLASS_FLOW_HI,tmp1);

    pTmpBuf = pHdrBase + NWAL_IPV6_OFFSET_NEXT_HDR;
    *pTmpBuf = proto;

    pTmpBuf = pHdrBase + NWAL_IPV6_OFFSET_HOP_LIMIT;
    *pTmpBuf = hopLimit;

    pTmpBuf = pHdrBase + NWAL_IPV6_OFFSET_SRC_ADDR;
    memcpy(pTmpBuf,pSrc,NWAL_IPV6_ADDR_NUM_BYTES);

    pTmpBuf = pHdrBase + NWAL_IPV6_OFFSET_DST_ADDR;
    memcpy(pTmpBuf,pDst,NWAL_IPV6_ADDR_NUM_BYTES);

    return nwal_TRUE;
}


/********************************************************************
 *  FUNCTION PURPOSE: nwal_prepIpv4Hdr Prepare IPv4 header
 *                    Optional headers are not handled
 ********************************************************************
 ********************************************************************/
nwal_Bool_t nwal_prepIpv4Hdr( uint8_t*  pHdrBase,
                              uint16_t  bufLen,
                              uint8_t   tos,
                              uint8_t   ttl,
                              uint8_t   proto,
                              uint8_t*  pSrc,
                              uint8_t*  pDst)
{
    uint8_t*    pTmpBuf = pHdrBase;

    if(bufLen < NWAL_IPV4_HDR_LEN_BYTES)
    {
        return nwal_FALSE;
    }
    memset(pTmpBuf,0,NWAL_IPV4_HDR_LEN_BYTES);

    pTmpBuf = pHdrBase + NWAL_IPV4_OFFSET_VER_HLEN;
    *pTmpBuf = NWAL_IPV4_VER_HDR_LEN_VAL;

    pTmpBuf = pHdrBase + NWAL_IPV4_OFFSET_TOS;
    *pTmpBuf = tos;

    /* Length to be updated external to this function */

    pTmpBuf = pHdrBase + NWAL_IPV4_OFFSET_TTL;
    *pTmpBuf = ttl;

    pTmpBuf = pHdrBase + NWAL_IPV4_OFFSET_PROTO;
    *pTmpBuf = proto;

    /* CHKSUM to be updated external to this function */
    /* Copy the source Address */
    pTmpBuf = pHdrBase + NWAL_IPV4_OFFSET_SRC_ADDR;
    memcpy(pTmpBuf,pSrc,NWAL_IPV4_ADDR_LEN);

    /* Copy the destination Address */
    pTmpBuf = pHdrBase + NWAL_IPV4_OFFSET_DST_ADDR;
    memcpy(pTmpBuf,pDst,NWAL_IPV4_ADDR_LEN);

    return nwal_TRUE;
}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_prepUdpHdr Prepare UDP header
 ********************************************************************
 ********************************************************************/
nwal_Bool_t nwal_prepUdpHdr ( uint8_t* pHdrBase,
                              uint16_t bufLen,
                              uint16_t srcPort,
                              uint16_t dstPort)
{
    if(bufLen < NWAL_UDP_HDR_LEN_BYTES)
    {
        return nwal_FALSE;
    }

    nwalWrite16bits_m(pHdrBase,
                      NWAL_UDP_OFFSET_SRC_PORT,
                      srcPort);
    nwalWrite16bits_m(pHdrBase,
                      NWAL_UDP_OFFSET_DEST_PORT,
                      dstPort);
    nwalWrite16bits_m(pHdrBase,
                      NWAL_UDP_OFFSET_LEN,
                      0);  /* Length to be updated separately */
    nwalWrite16bits_m(pHdrBase,
                      NWAL_UDP_OFFSET_CHKSUM,
                      0);/* Pseudo Header checksum*/
    return nwal_TRUE;
}

/********************************************************************
 *  FUNCTION PURPOSE: Get next instance. Generic function to
 *                    get next MAC/IP/IPSec/UDP Instance
 *                    Function would need to be multicore safe and
 *                    handle cache alignment.
 *                    Calling function would need to ensure that next
 *                    instance is available and not out of bound
 *
 ********************************************************************
 ********************************************************************/
void* nwal_getNextInst( void*        pInst,
                        uint32_t     instSize)
{
    uint32_t                instAdjSize;
    uint8_t*                pMem = pInst;

    instAdjSize = nwal_round_size(NWAL_CACHE_LINE_SIZE, 1, instSize);
    pMem = pMem + instAdjSize;
    return(pMem);
}


/********************************************************************
 *  FUNCTION PURPOSE: Get a Free NWAL instance. Generic function to
 *                    get MAC/IP/IPSec/UDP Instance
 *                    Calling function would need to be ensure multicore
 *                    protection
 *
 ********************************************************************
 ********************************************************************/
nwal_Handle nwal_getInst(   void*        pInstBlock,
                            uint32_t     maxNumInstances,
                            uint32_t     instSize)
{

    uint16_t                count;
    nwalHandleHdr_t*        pHandleHdr;
    uint8_t*                pMem;
    NWAL_STATE_COUNT_T      state;
    uint32_t                instAdjSize;

    instAdjSize = nwal_round_size(NWAL_CACHE_LINE_SIZE, 1, instSize);

    pMem = pInstBlock;

    /* Retrieve next available IPSec Channel */
    for(count=0;count < maxNumInstances;count++)
    {
        pHandleHdr = (nwalHandleHdr_t*)(pMem);
        NWAL_osalInvalidateCache(pMem,instAdjSize);
        state = NWAL_GET_STATE(pHandleHdr->stateCount);
        if(state == NWAL_STATE_INACTIVE)
        {
            /* Set count to 1 */
            pHandleHdr->stateCount =
                (NWAL_SET_STATE(pHandleHdr->stateCount,
                                NWAL_STATE_CFG_IN_PROGRESS) | 1);
            NWAL_osalWriteBackCache(pMem,instAdjSize);
            //printf("nwal_getInst: pMem(as address): 0x%x, maxNumInstances: %d\n", pMem, maxNumInstances);
            return((nwal_Handle )pMem);
        }

        pMem = pMem + instAdjSize;
        
    }

    return NULL;
}

/********************************************************************
 *  FUNCTION PURPOSE: Free a NWAL instance. Generic function to
 *                    fredd MAC/IP/IPSec/UDP Instance
 *                    Function would need to be multicore safe and
 *                    handle cache alignment.
 *
 ********************************************************************
 ********************************************************************/
void  nwal_freeInst(nwal_Handle*    pHandle,
                    uint32_t        instSize)
{

    uint32_t            key;
    nwalHandleHdr_t     handleHdr;

    memcpy(&handleHdr,(nwalHandleHdr_t *)pHandle,sizeof(nwalHandleHdr_t));

    NWAL_osalCsEnter(&key);

    /* Invalidation not done as it is write only */
    memset(pHandle,0,instSize);
    handleHdr.stateCount =
                NWAL_SET_STATE(handleHdr.stateCount,NWAL_STATE_INACTIVE);
    memcpy((nwalHandleHdr_t *)pHandle,&handleHdr,sizeof(nwalHandleHdr_t));
    NWAL_osalWriteBackCache(pHandle,instSize);

    NWAL_osalCsExit(key);
}
/********************************************************************
 *  FUNCTION PURPOSE: nwal_saveTransInfo: Store the transaction
 *                    Information details from app
 *
 ********************************************************************
 ********************************************************************/
void nwal_saveTransInfo (nwalIntTransInfo_t*    pIntTransInfo,
                         nwal_TransID_t         transId)
{
    pIntTransInfo->transId =  transId;
}

/********************************************************************
 * FUNCTION PURPOSE: Set Transaction details for the callback
 ********************************************************************
 * DESCRIPTION: Set Transaction details so that application call back
 *              can be initiated when reply is received from PA
 ********************************************************************/
void nwal_setTransInfo
(
    nwalIntTransInfo_t* pTransInfo,
    uint16_t            transType,
    void*               nwalHandle
)
{
    pTransInfo->handleHdr.handleId =
        (NWAL_HANDLE_TYPE_TRANS << NWAL_HANDLE_TYPE_SHIFT);
    pTransInfo->handleHdr.handleId |= transType;
    pTransInfo->nwalHandle  = nwalHandle;
}


/********************************************************************
 * FUNCTION PURPOSE: Retrieve the linked Buffer Q based on the correct
 *                   size
 ********************************************************************
 * DESCRIPTION: Retrieve the linked Buffer Q based on the correct
 *              size
 ********************************************************************/
nwalBufPool_t* nwal_getLinkedBufQ
(
    nwalBufPool_t*          pLinkBuf,
    uint16_t                bufSize,
    uint16_t                numQ
)
{
    uint16_t              count = 0;
    while(count < numQ)
    {
        if(pLinkBuf->bufSize >= bufSize)
        {
            return(pLinkBuf);
        }
        count++;
        pLinkBuf++;
    }
    return NULL;
}

/********************************************************************
 * FUNCTION PURPOSE: Prepare Command Buffer for LLD
 ********************************************************************
 * DESCRIPTION: Prepare the command buffer for LLD
 ********************************************************************/
nwal_RetValue nwal_prepCmdBuf
(
    nwalGlobContext_t*      pIhandle,
    uint16_t*               pCmdSize,
    paCmdReply_t*           pCmdReply,
    nwalIntTransInfo_t*     pTransInfo,
    Cppi_HostDesc**         ppHd
)
{
    nwalLocContext_t*       pLocContext;
    Ti_Pkt*                 pPkt;
    uint8_t*                pDataBuffer;
    uint32_t                origBufSize;
    nwalBufPool_t*          pLinkBufTxQ;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    pLinkBufTxQ = nwal_getLinkedBufQ(pLocContext->cfg.txCtlPool.bufPool,
                                       *pCmdSize,
                                       pLocContext->cfg.txCtlPool.numBufPools);
    if(pLinkBufTxQ  == NULL)
        return (nwal_ERR_NO_FREE_BUF);

    /* Allocate a packet from heap. The API also does invalidation of packet
     * By default packets will be returned to the return queue for packet lib
     * since no cloning or merge is being done
     */
    pPkt = Pktlib_allocPacket(pLinkBufTxQ->heapHandle,*pCmdSize);
    if (pPkt == NULL)
        return (nwal_ERR_NO_FREE_CMD_DESC);

    *ppHd = Pktlib_getDescFromPacket(pPkt);
    Cppi_getOriginalBufInfo(Cppi_DescType_HOST,
                            (Cppi_Desc *)*ppHd,
                            (uint8_t**)&pDataBuffer,
                            &origBufSize);
    /* Needed this conversion to adapt between different type between PA
     * and CPPI Since size is not expected to be more than 16 bit so no
     * side effect expected
     */
    *pCmdSize = (uint16_t)origBufSize;

    pCmdReply->dest = pa_DEST_HOST;
    pCmdReply->replyId = (uint32_t)(pTransInfo);
    pCmdReply->queue = pLocContext->rxCtlQ;
    pCmdReply->flowId = (uint8_t)(pLocContext->rxCtlFlowId);

    return (nwal_OK);
}

/********************************************************************
 * FUNCTION PURPOSE: Transmit command buffer to PA
 ********************************************************************
 * DESCRIPTION: Transmit the command buffer for PA subsystem
 ********************************************************************/
nwal_RetValue nwal_txCmdBuf
(
    nwalGlobContext_t*      pIhandle,
    uint16_t                cmdSize,
    int                     cmdDest,
    Cppi_HostDesc*          pHd
)
{
    uint32_t                psCmd;

    /* Mark the packet as a configuration packet */
    psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST,
                    (Cppi_Desc *)pHd,
                    (uint8_t *)&psCmd,
                    4);

    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHd, cmdSize);
    pHd->buffLen = cmdSize;

    /* Update the cache */
    NWAL_osalWriteBackCache(pHd,NWAL_DESC_SIZE);
    NWAL_osalWriteBackCache((void *)pHd->buffPtr,cmdSize);


    /* Send the command to PA. This will result in 1 packet in classify1 */
    Qmss_queuePushDescSize(nwalProcessCtx.pSharedMemBase->txQ[cmdDest],
    //pIhandle->txQ[cmdDest],
                            (Ptr)pHd,
                            NWAL_DESC_SIZE);

    return (nwal_OK);

}


/********************************************************************
 * FUNCTION PURPOSE: Common Function to delete L2/L3/L4 handle
 ********************************************************************
 * DESCRIPTION: Deletes Mac node from the list
 ********************************************************************/
nwal_RetValue nwalDelPaHandle (nwalGlobContext_t*       pIhandle,
                               nwalIntTransInfo_t*      pTransInfo,
                               paHandleL4_t             paL4Handle,
                               paHandleL2L3_t*          paL2L3Handle)
{
    Cppi_HostDesc*      pHd;
    paReturn_t          netCPRet;
    paCmdReply_t        cmdReply;
    uint16_t            cmdSize = pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES;
    int                 cmdDest;
    nwal_RetValue       retVal;
    nwalLocContext_t*   pLocContext;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    retVal = nwal_prepCmdBuf(nwalProcessCtx.pSharedMemBase,
                             &cmdSize,
                             &cmdReply,
                             pTransInfo,
                             &pHd);
    if(retVal != nwal_OK)
    {
        return retVal;
    }

    if(paL2L3Handle != NULL)
    {
        /* Layer 2/3 Handle */
        netCPRet =  Pa_delHandle (nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                               paL2L3Handle,
                               (paCmd_t) pHd->buffPtr,
                               &cmdSize,
                               &cmdReply,
                               &cmdDest);
    }
    else
    {
        netCPRet =  Pa_delL4Handle (nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                                 paL4Handle,
                                (paCmd_t) pHd->buffPtr,
                                 &cmdSize,
                                 &cmdReply,
                                 &cmdDest);
    }

    if(netCPRet != pa_OK)
    {
        pLocContext->extErr = netCPRet;
        return nwal_ERR_PA;
    }

    retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                            cmdSize,
                            cmdDest,
                            pHd);
    return (retVal);

}

/********************************************************************
 * FUNCTION PURPOSE: Get Match Route and Fail Route Info
 ********************************************************************
 * DESCRIPTION: Get Match Route and Fail Route Info. Fail Route is optional
 ********************************************************************/
nwal_RetValue nwal_convRouteInfo(nwalGlobContext_t*      pNwalHandle,
                                 nwalLocContext_t*       pLocContext,
                                 nwalLocRouteInfo_t*        pRouteInfo,
                                 int                     dest,
                                 uint32_t                swInfo,
                                 paRouteInfo_t*          pMatchRoute,
                                 paRouteInfo_t*          pFailRoute,
                                 paRouteInfo2_t*         pMatchRoute2,
                                 paRouteInfo2_t*         pFailRoute2)
{
    if(pMatchRoute2 == NULL)
        return(nwal_ERR_INVALID_PARAM);
    else
    {
        memset(pMatchRoute2,0,sizeof(paRouteInfo2_t));
        pMatchRoute2->mRouteIndex = pa_NO_MULTI_ROUTE;
        pMatchRoute2->swInfo0 = swInfo;
    }

    if(pMatchRoute)
    {
        memset(pMatchRoute,0,sizeof(paRouteInfo_t));
    }
    if(pFailRoute)
    {
        memset(pFailRoute,0,sizeof(paRouteInfo_t));
    }

    if(pFailRoute2)
    {
        memset(pFailRoute2,0,sizeof(paRouteInfo2_t));
    }

    if(pRouteInfo->flowId == CPPI_PARAM_NOT_SPECIFIED)
    {
        pRouteInfo->flowId = pLocContext->rxPktFlowId;
    }

    if(pRouteInfo->rxPktQ == QMSS_PARAM_NOT_SPECIFIED)
    {
        pRouteInfo->rxPktQ = nwalProcessCtx.pSharedMemBase->cfg.rxDefPktQ;
    }

    switch(pRouteInfo->matchAction)
    {
        case NWAL_MATCH_ACTION_HOST:
        {
            pMatchRoute2->dest = pa_DEST_HOST;
            pMatchRoute2->mRouteIndex = pa_NO_MULTI_ROUTE;
            pMatchRoute2->swInfo0 = swInfo;
            pMatchRoute2->queue = pRouteInfo->rxPktQ;
            pMatchRoute2->flowId = pRouteInfo->flowId;

            nwalUpdateRoutePriority(pRouteInfo->routeType, pMatchRoute2);
            if(pRouteInfo->routeType == NWAL_ROUTE_PKTTYPE_EQOS)
            {
                pMatchRoute2->pktType_emacCtrl = (pRouteInfo->egress_switch_port |
                                                  pa_EMAC_CTRL_CRC_DISABLE);
            }
        break;
        }
        case NWAL_MATCH_ACTION_DISCARD:
        {
            pMatchRoute2->dest = pa_DEST_DISCARD;
            break;
        }
        case NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE:
        {
            pMatchRoute2->dest = dest;
            break;
        }
        case NWAL_MATCH_ACTION_EMAC:
             pMatchRoute2->dest = pa_DEST_EMAC;
             pMatchRoute2->pktType_emacCtrl = pRouteInfo->egress_switch_port;
            break;
        default:
            return(nwal_ERR_INVALID_PARAM);
    }
    if(pMatchRoute)
    {
         pMatchRoute->dest = pMatchRoute2->dest;
         pMatchRoute->flowId = pMatchRoute2->flowId;
         pMatchRoute->mRouteIndex = pMatchRoute2->mRouteIndex;
         pMatchRoute->queue = pMatchRoute2->queue;
         pMatchRoute->swInfo0 = pMatchRoute2->swInfo0;
    }

    if(pFailRoute2)
    {
        if(pRouteInfo->failAction == NWAL_NEXT_ROUTE_FAIL_ACTION_HOST)
        {
            pFailRoute2->dest = pa_DEST_HOST;
            pFailRoute2->mRouteIndex = pa_NO_MULTI_ROUTE;
            pFailRoute2->swInfo0 = swInfo;
            pFailRoute2->queue = pRouteInfo->rxPktQ;
            pFailRoute2->flowId = pRouteInfo->flowId;
            nwalUpdateRoutePriority(pRouteInfo->routeType, pFailRoute2);
            if(pRouteInfo->routeType == NWAL_ROUTE_PKTTYPE_EQOS)
            {
                pFailRoute2->pktType_emacCtrl = pRouteInfo->egress_switch_port;
            }

        }
        else
        {
            pFailRoute2->dest = pa_DEST_DISCARD;
        }
        if(pFailRoute)
        {
            pFailRoute->dest = pFailRoute2->dest;
            pFailRoute->mRouteIndex = pFailRoute2->mRouteIndex;
            pFailRoute->swInfo0 = pFailRoute2->swInfo0;
            pFailRoute->queue = pFailRoute2->queue;
            pFailRoute->flowId = pFailRoute2->flowId;
        }
    }
 
    return(nwal_OK);
}
/********************************************************************
 * FUNCTION PURPOSE: Configure PA for IP Address
 ********************************************************************
 * DESCRIPTION: Function to configure IP address at PA
 ********************************************************************/
nwal_RetValue nwal_configIP
(
   nwalGlobContext_t*       pIhandle,
   nwalIpInfo_t*            pIpInfo,
   nwalLocRouteInfo_t*         pRouteInfo
)
{
    Cppi_HostDesc*      pHd;
    paReturn_t          netCPRet;
    paRouteInfo2_t      matchRoute2,failRoute2;
    paCmdReply_t        cmdReply;
    uint16_t            cmdSize = pa_ADD_IP_MIN_CMD_BUF_SIZE_BYTES;
    int                 cmdDest;
    nwalHandleHdr_t*    pHandleHdr;
    nwalIpInfo_t*       pTunIPInfo;
    nwalMacInfo_t*      pMacInfo;
    nwal_RetValue       retVal;
    nwalLocContext_t*   pLocContext;
    paParamDesc         paramDesc;
    nwal_Handle         prevHandleAddr;


    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    memset(&paramDesc,0,sizeof(paParamDesc));


    prevHandleAddr = (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                    pIpInfo->prevHandle));

    if(prevHandleAddr != nwal_HANDLE_INVALID)
    {
        
        pHandleHdr = (nwalHandleHdr_t *)prevHandleAddr;
        if(pHandleHdr->handleId == NWAL_HANDLE_MAC_INST)
        {
            pMacInfo = (nwalMacInfo_t *)(prevHandleAddr);
            paramDesc.prevLink = pMacInfo->paMacHandle;
            
        }
        else if(pHandleHdr->handleId == NWAL_HANDLE_IP_INST)
        {
            pTunIPInfo = (nwalIpInfo_t *)(prevHandleAddr);
            paramDesc.prevLink = pTunIPInfo->paIpHandle;
        }
        else if(pHandleHdr->handleId == NWAL_HANDLE_IPSEC_INST)
        {
            nwalIpSecInfo_t*    pIpSecInfo;
            pIpSecInfo = (nwalIpSecInfo_t *)(prevHandleAddr);
            paramDesc.prevLink = pIpSecInfo->paIpHandle;
        }
        else
        {
            /* Should never reach here */
            return nwal_ERR_INVALID_HANDLE;
        }
        if(paramDesc.prevLink)
            paramDesc.validBitMap |= pa_PARAM_VALID_PREVLINK;
    }

    retVal = nwal_prepCmdBuf(nwalProcessCtx.pSharedMemBase,
                             &cmdSize,
                             &cmdReply,
                             &pIpInfo->transInfo,
                             &pHd);
    if(retVal != nwal_OK)
    {
        goto ERR_nwal_configIP;
    }
    
     /* Get the Match and Route Info based on Application configuration
     */
    retVal = nwal_convRouteInfo(nwalProcessCtx.pSharedMemBase,
                                pLocContext,
                                pRouteInfo,
                                pa_DEST_CONTINUE_PARSE_LUT2,
                                (uint32_t)pIpInfo->transInfo.appId,
                                NULL,
                                NULL,
                                &matchRoute2,
                                &failRoute2);


    if(retVal != nwal_OK)
    {
        goto ERR_nwal_configIP;
    }

    paramDesc.routeInfo = &matchRoute2;
    paramDesc.nextRtFail = &failRoute2;
    paramDesc.index = pa_LUT1_INDEX_NOT_SPECIFIED;
    paramDesc.lutInst = pa_LUT_INST_NOT_SPECIFIED;

    netCPRet =  Pa_addIp2(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                          &pIpInfo->paIpInfo,
                          &paramDesc,
                          &pIpInfo->paIpHandle,
                          (paCmd_t) pHd->buffPtr,
                          &cmdSize,
                          &cmdReply,
                          &cmdDest);

    if(netCPRet != pa_OK)
    {
        pLocContext->extErr = netCPRet;
        retVal = nwal_ERR_PA;
        goto ERR_nwal_configIP;
    }

    if (cmdDest > NSS_NUM_TX_PKTDMA_CHANNELS)
    {
        return nwal_ERR_INVALID_CMD_DEST;
    }

    retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                            cmdSize,
                            cmdDest,
                            pHd);
    if(retVal != nwal_OK)
    {
        goto ERR_nwal_configIP;
    }

    nwal_setTransInfo(&pIpInfo->transInfo,
                      NWAL_HANDLE_ID_TRANS_ADD_IP,
                      pIpInfo);
    /*update global info; need to protect from multicore*/
    pLocContext->numPendPAReq++;

    pIpInfo->appId = pIpInfo->transInfo.appId;
    pIpInfo->handleHdr.handleId = NWAL_HANDLE_IP_INST;

    NWAL_osalWriteBackCache(pIpInfo,sizeof(nwalIpInfo_t));
    NWAL_osalWriteBackCache(pLocContext,sizeof(nwalLocContext_t));

    return (nwal_OK);

ERR_nwal_configIP:
    return retVal;

}


/********************************************************************
 * FUNCTION PURPOSE: Configure PA for local L4 LUT2 entry/ port
 ********************************************************************
 * DESCRIPTION: Function to configure L4 LUT2 entry
 ********************************************************************/
nwal_RetValue nwal_configPort
(
   nwalGlobContext_t*       pIhandle,
   nwalPortInfo_t           *pPortInfo,
   paHandleL2L3_t           prevLink,
   nwalLocRouteInfo_t*         pRouteInfo
)
{
    Cppi_HostDesc       *pHd;
    paReturn_t          netCPRet;
    paRouteInfo_t       matchRoute;
    paRouteInfo2_t      matchRoute2;
    paCmdReply_t        cmdReply;
    uint16_t            cmdSize = pa_ADD_PORT_MIN_CMD_BUF_SIZE_BYTES;
    int                 cmdDest;
    nwal_RetValue       retVal;
    int                 portSize = pa_LUT2_PORT_SIZE_16;
    uint32_t            destPort;
    nwalLocContext_t*   pLocContext;
    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    if(pRouteInfo->matchAction == NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE)
    {
        /* No next route available. Packet either need to be
         * discarded or terminated to host
         */
        return(nwal_ERR_INVALID_PARAM);
    }

    retVal = nwal_prepCmdBuf(nwalProcessCtx.pSharedMemBase,
                             &cmdSize,
                             &cmdReply,
                             &pPortInfo->transInfo,
                             &pHd);
    if(retVal != nwal_OK)
    {
        goto ERR_nwal_configPort;
    }

    /* Get the Match and Fail Route Info based on Application configuration
     */
    pRouteInfo->failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_DISCARD;

    retVal = nwal_convRouteInfo(nwalProcessCtx.pSharedMemBase,
                                pLocContext,
                                pRouteInfo,
                                pa_DEST_HOST,
                                (uint32_t)pPortInfo->transInfo.appId,
                                &matchRoute,
                                NULL,
                                &matchRoute2,
                                NULL);
    if(retVal != nwal_OK)
    {
        goto ERR_nwal_configPort;
    }

    if(pPortInfo->proto == NWAL_APP_PLOAD_PROTO_GTPU)
    {
        portSize = pa_LUT2_PORT_SIZE_32;
        destPort = pPortInfo->rxConnCfg.appProto.gtpTeid;
    }
    else
    {
        destPort = pPortInfo->rxConnCfg.appProto.udpPort;
    }

    netCPRet =  Pa_addPort(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                        portSize,
                        destPort,
                        prevLink,
                        0,
                        pa_PARAMS_NOT_SPECIFIED,
                        &matchRoute,
                        pPortInfo->paPortHandle,
                        (paCmd_t) pHd->buffPtr,
                        &cmdSize,
                        &cmdReply,
                        &cmdDest);
    if(netCPRet != pa_OK)
    {
        pLocContext->extErr = netCPRet;
        retVal = nwal_ERR_PA;
        goto ERR_nwal_configPort;
    }

    if (cmdDest > NSS_NUM_TX_PKTDMA_CHANNELS)
    {
        retVal = nwal_ERR_INVALID_CMD_DEST;
        goto ERR_nwal_configPort;
    }

    retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                            cmdSize,
                            cmdDest,
                            pHd);

     if(retVal != nwal_OK)
    {
        goto ERR_nwal_configPort;
    }

     nwal_setTransInfo(&pPortInfo->transInfo,
                       NWAL_HANDLE_ID_TRANS_ADD_PORT,
                       pPortInfo);

    pPortInfo->handleHdr.handleId = NWAL_HANDLE_PORT_INST;
    pPortInfo->appId = pPortInfo->transInfo.appId;

    /*update global info; need to protect from multicore*/
    pLocContext->numPendPAReq++;

    NWAL_osalWriteBackCache(pPortInfo,sizeof(nwalPortInfo_t));
    NWAL_osalWriteBackCache(pLocContext,sizeof(nwalLocContext_t));
    return (nwal_OK);

ERR_nwal_configPort:
    nwal_freeInst((nwal_Handle*)pPortInfo,sizeof(nwalPortInfo_t));
    return(retVal);

}

/********************************************************************
 * FUNCTION PURPOSE: nwal_retBuf: Return Buffer and descriptor back
 *********************************************************************
 * DESCRIPTION: Return Buffer and descriptor back
 ********************************************************************/
nwal_Bool_t nwal_retBuf
(
    nwalGlobContext_t*      pIhandle,
    nwal_Bool_t             isCtl,
    nwal_Bool_t             isRx,
    Cppi_HostDesc*          pHd
)
{
    nwalLocContext_t*   pLocContext;
    Ti_Pkt*             pPkt;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_FALSE);
    }

    if(isRx)
    {
        pPkt = Pktlib_getPacketFromDesc(pHd);
        Pktlib_freePacket(pPkt);
        return nwal_TRUE;
    }

    /* TX buffer will be freed by Hardware.
     */
    return nwal_TRUE;
}


/********************************************************************
 * FUNCTION PURPOSE: Invalidate cache for dependent handles
 ********************************************************************
 * DESCRIPTION: Invalidate cache for dependent handles
 ********************************************************************/
void nwal_InvPreHnd(nwal_Handle*    pHandle)
{
    nwalHandleHdr_t*    pHndlHdr = (nwalHandleHdr_t*)pHandle;
    NWAL_osalInvalidateCache(pHndlHdr,sizeof(nwalHandleHdr_t));

    while(((nwal_Handle)pHndlHdr != nwal_HANDLE_INVALID) &&
           (pHndlHdr->handleId != NWAL_HANDLE_MAC_INST))
    {
        /* Invalidate all previous handles */
        switch(pHndlHdr->handleId)
        {
            case NWAL_HANDLE_PORT_INST:
            {
                nwalPortInfo_t* pPortInfo = (nwalPortInfo_t*)pHndlHdr;
                NWAL_osalInvalidateCache(pPortInfo,sizeof(nwalPortInfo_t));
                if(pPortInfo->pL2L3HdrInfo)
                {
                    pHndlHdr =
                        (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                        pPortInfo->pL2L3HdrInfo));
                }
                else if (pPortInfo->rxConnCfg.inHandle)
                {
                    pHndlHdr =
                        (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                        pPortInfo->rxConnCfg.inHandle));
                }
                break;
            }

            case NWAL_HANDLE_L2_L3_HDR_INST:
            {
                nwalL2L3HdrInfo_t*  pL2L3HdrInfo =
                                        (nwalL2L3HdrInfo_t*)pHndlHdr;
                nwalIpSecInfo_t*    pIpSecInfo;

                NWAL_osalInvalidateCache(pL2L3HdrInfo,
                                         sizeof(nwalL2L3HdrInfo_t)
                                         );
                pHndlHdr =
                    (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                      pL2L3HdrInfo->inHandle));

                if(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                        pL2L3HdrInfo->outHandle))
                {
                    nwalHandleHdr_t*    pHndlHdr2 =
                                        (nwalHandleHdr_t*)
                                        (NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                        pL2L3HdrInfo->outHandle));
                    if(pHndlHdr2->handleId != NWAL_HANDLE_IPSEC_INST)
                    {
                        /* Unexpected Error */
                        while(1);
                    }

                    pIpSecInfo = 
                        (nwalIpSecInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                    pL2L3HdrInfo->outHandle));
                    NWAL_osalInvalidateCache(pIpSecInfo,
                                             sizeof(nwalIpSecInfo_t));
                }

                break;
            }

            case NWAL_HANDLE_IP_INST:
            {
                nwalIpInfo_t*  pIpInfo = (nwalIpInfo_t*)pHndlHdr;
                NWAL_osalInvalidateCache(pIpInfo,sizeof(nwalIpInfo_t));
                pHndlHdr = 
                          (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                           pIpInfo->prevHandle));
                break;
            }

            case NWAL_HANDLE_IPSEC_INST:
            {
                nwalIpSecInfo_t*  pIpSecInfo = (nwalIpSecInfo_t*)pHndlHdr;
                NWAL_osalInvalidateCache(pIpSecInfo,sizeof(nwalIpSecInfo_t));
                pHndlHdr = 
                               (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                 pIpSecInfo->prevHandle));
                break;
            }
        }
    }
    NWAL_osalInvalidateCache(pHndlHdr,sizeof(nwalMacInfo_t));
}



/********************************************************************
 * FUNCTION PURPOSE: Convert MAC configuration to PA config structure
 ********************************************************************
 * DESCRIPTION: Convert MAC configuration to PA config structure
 ********************************************************************/
nwal_RetValue nwal_convMacParam(nwalMacParam_t*        pParam,
                                paEthInfo2_t*          pPaEthInfo2)
{
    static unsigned char allMmac[]={0,0,0,0,0,0};

    /* Store the MAC configuration for PA for later lookup, only set
        pa_ETH_INFO_VALID_DST for non-zero MAC address*/
    if(memcmp(pParam->macAddr, allMmac, sizeof(nwalMacAddr_t)))
    {
        pPaEthInfo2->validBitMap |= pa_ETH_INFO_VALID_DST;
    }

    memcpy(pPaEthInfo2->dst,pParam->macAddr,sizeof(nwalMacAddr_t));

    if((pParam->validParams & NWAL_SET_MAC_VALID_PARAM_VLAN_ID) ==
        NWAL_SET_MAC_VALID_PARAM_VLAN_ID)
    {
        pPaEthInfo2->vlan = pParam->vlanId;
        pPaEthInfo2->validBitMap |= pa_ETH_INFO_VALID_VLAN;
    }

    if((pParam->validParams & NWAL_SET_MAC_VALID_PARAM_IFNUM) ==
        NWAL_SET_MAC_VALID_PARAM_IFNUM)
    {
        pPaEthInfo2->inport = pParam->ifNum;
        pPaEthInfo2->validBitMap |= pa_ETH_INFO_VALID_INPORT;
    }
    if((pParam->validParams & NWAL_SET_MAC_VALID_PARAM_REMOTE_MAC) ==
        NWAL_SET_MAC_VALID_PARAM_REMOTE_MAC)
    {
        memcpy(pPaEthInfo2->src,pParam->remMacAddr,sizeof(nwalMacAddr_t));
        pPaEthInfo2->validBitMap |= pa_ETH_INFO_VALID_SRC;
    }
    if((pParam->validParams & NWAL_SET_MAC_VALID_PARAM_ETHER_TYPE) ==
        NWAL_SET_MAC_VALID_PARAM_ETHER_TYPE)
    {
        pPaEthInfo2->ethertype = pParam->etherType;
        pPaEthInfo2->validBitMap |= pa_ETH_INFO_VALID_ETHERTYPE;
    }
    
    return(nwal_OK);
}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_setMacIface() API
 ********************************************************************
 * DESCRIPTION: nwal_setMacIface() API Implementation
 ********************************************************************/
nwal_RetValue nwal_setMacIface   (nwal_Inst             nwalInst,
                                  nwal_TransID_t        transId,
                                  nwal_AppId            appId,
                                  nwalMacParam_t*       pParam,
                                  nwal_Handle*          pIfHandle)
{
    Cppi_HostDesc*      pHd;
    paReturn_t          netCPRet;
    paRouteInfo2_t      matchRoute2,failRoute2;
    paParamDesc         paramDesc;
    paCmdReply_t        cmdReply;
    uint16_t            cmdSize = pa_ADD_MAC_MIN_CMD_BUF_SIZE_BYTES;
    int                 cmdDest;
    nwal_RetValue       retVal;
    nwal_Handle         tmpHandle;
    nwalMacInfo_t*      pMacInfo;
    nwalLocContext_t*   pLocContext;
    uint32_t            key;
    nwalLocRouteInfo_t     routeInfo;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    /* Get a Free MAC Instance */
    NWAL_osalCsEnter(&key);
    tmpHandle = 
        nwal_getInst((nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                             nwalProcessCtx.pSharedMemBase->pMacInfo)),
                             nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxMacAddress,
                             sizeof(nwalMacInfo_t));
    if(tmpHandle == NULL)
    {
        NWAL_osalCsExit(key);
        return nwal_ERR_RES_UNAVAILABLE;
    }


    memset(&paramDesc,0,sizeof(paParamDesc));
    pMacInfo = (nwalMacInfo_t *)(tmpHandle);
    memset(pMacInfo,0,sizeof(nwalMacInfo_t));

    retVal= nwal_convMacParam(pParam,&pMacInfo->paEthInfo);
    if(retVal != nwal_OK)
    {
        NWAL_osalCsExit(key);
        return(retVal);
    }

    nwal_saveTransInfo(&pMacInfo->transInfo,transId);
    pMacInfo->transInfo.appId = appId;

    retVal = nwal_prepCmdBuf(nwalProcessCtx.pSharedMemBase,
                             &cmdSize,
                             &cmdReply,
                             &pMacInfo->transInfo,
                             &pHd);
    if(retVal != nwal_OK)
    {
        goto ERR_nwal_setMacIface;
    }
    memset(&routeInfo,0,sizeof(nwalLocRouteInfo_t));

    if ((pParam->validParams & NWAL_SET_MAC_VALID_PARAM_ROUTE_TYPE) ==
            (NWAL_SET_MAC_VALID_PARAM_ROUTE_TYPE))
    {
        routeInfo.routeType = pParam->routeType;
    }
    routeInfo.flowId = pParam->appRxPktFlowId;
    routeInfo.rxPktQ = pParam->appRxPktQueue;
    routeInfo.matchAction = pParam->matchAction;
    routeInfo.failAction = pParam->failAction;
    routeInfo.egress_switch_port = pParam->egress_switch_port;
    
    /* Get the Match and Route Info based on Application configuration
     */
    retVal = nwal_convRouteInfo(nwalProcessCtx.pSharedMemBase,
                                pLocContext,
                                &routeInfo,
                                pa_DEST_CONTINUE_PARSE_LUT1,
                                (uint32_t)appId,
                                NULL,
                                NULL,
                                &matchRoute2,
                                &failRoute2);

    paramDesc.routeInfo = &matchRoute2;
    paramDesc.nextRtFail = &failRoute2;
    paramDesc.index = pa_LUT1_INDEX_NOT_SPECIFIED;
    paramDesc.lutInst = pa_LUT_INST_NOT_SPECIFIED;

    if(retVal != nwal_OK)
    {
        goto ERR_nwal_setMacIface;
    }

    netCPRet = Pa_addMac2(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                          &pMacInfo->paEthInfo,
                          &paramDesc,
                          &pMacInfo->paMacHandle,
                          (paCmd_t) pHd->buffPtr,
                          &cmdSize,
                          &cmdReply,
                          &cmdDest);

    if(netCPRet != pa_OK)
    {
        pLocContext->extErr = netCPRet;
        retVal = nwal_ERR_PA;
        goto ERR_nwal_setMacIface;
    }

    retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                            cmdSize,
                            cmdDest,
                            pHd);
    if(retVal != nwal_OK)
    {
        goto ERR_nwal_setMacIface;
    }

    /*update global info; need to protect from multicore*/
    pLocContext->numPendPAReq++;

    pMacInfo->appId = appId;
    pMacInfo->handleHdr.handleId = NWAL_HANDLE_MAC_INST;

    nwal_setTransInfo(&pMacInfo->transInfo,
                      NWAL_HANDLE_ID_TRANS_ADD_MAC,
                      pMacInfo);


    /* Set the handle as Inprogress. The handle will be set to Active after
     * getting the response.
     */
    pMacInfo->handleHdr.stateCount =
            NWAL_SET_STATE(pMacInfo->handleHdr.stateCount,
                           NWAL_STATE_CFG_IN_PROGRESS);

    *pIfHandle = (nwal_Handle*)NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                             pMacInfo);

    if(transId == NWAL_TRANSID_SPIN_WAIT)
    {
        /* Block until response is received from NetCP */
        retVal =
            nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                          NULL,
                          NULL,
                          &pMacInfo->transInfo.handleHdr);
    }

    NWAL_osalWriteBackCache(pMacInfo,sizeof(nwalMacInfo_t));
    NWAL_osalWriteBackCache(pLocContext,sizeof(nwalLocContext_t));
    NWAL_osalCsExit(key);
    return (retVal);

ERR_nwal_setMacIface:
    NWAL_osalCsExit(key);
    nwal_freeInst((nwal_Handle*)pMacInfo,sizeof(nwalMacInfo_t));
    return (retVal);

}
/********************************************************************
 *  FUNCTION PURPOSE: nwal_delMacIface() API
 ********************************************************************
 * DESCRIPTION: nwal_delMacIface() API Implementation
 ********************************************************************/
nwal_RetValue nwal_delMacIface   (nwal_Inst                 nwalInst,
                                  nwal_TransID_t            transId,
                                  nwal_Handle               pIfHandle)
{
    nwal_RetValue        retVal;

    nwalMacInfo_t*   pMacInfo =
            (nwalMacInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
            pIfHandle));

    nwal_saveTransInfo(&pMacInfo->transInfo,transId);
    nwal_setTransInfo(&pMacInfo->transInfo,
                      NWAL_HANDLE_ID_TRANS_DEL_MAC,
                      pMacInfo);

    retVal = nwalDelPaHandle(nwalProcessCtx.pSharedMemBase,
                             &pMacInfo->transInfo,
                             0,
                             &pMacInfo->paMacHandle);
    if(retVal!= nwal_OK)
    {
        return(retVal);
    }

    if(transId == NWAL_TRANSID_SPIN_WAIT)
    {
        /* Block until response is received from NetCP */
        retVal =
            nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                          NULL,
                          NULL,
                          &pMacInfo->transInfo.handleHdr);
    }
    NWAL_osalWriteBackCache(pMacInfo,sizeof(nwalMacInfo_t));
    return (retVal);
}
/********************************************************************
 *  FUNCTION PURPOSE: nwal_getMacIface() API
 ********************************************************************
 * DESCRIPTION: nwal_getMacIface() API Implementation
 ********************************************************************/
nwal_Bool_t nwal_getMacIface(   nwal_Inst           nwalInst,
                                nwalMacParam_t*     pParam,
                                nwal_Handle*        pIfHandle)
{
    nwalMacInfo_t*      pMacInfo;
    uint8_t             count=0;
    nwal_Bool_t         found = nwal_FALSE;
    paEthInfo2_t         paEthInfo;
    uint32_t            key;
    NWAL_STATE_COUNT_T  state;

    *pIfHandle = NULL;
    memset(&paEthInfo,0,sizeof(paEthInfo2_t));
    nwal_convMacParam(pParam,&paEthInfo);

    pMacInfo =  (nwalMacInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                nwalProcessCtx.pSharedMemBase->pMacInfo)),
    NWAL_osalCsEnter(&key);
    while(count < nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxMacAddress)
    {
        NWAL_osalInvalidateCache(pMacInfo,sizeof(nwalMacInfo_t));
        state = NWAL_GET_STATE(pMacInfo->handleHdr.stateCount);
        if((state == NWAL_STATE_ACTIVE) &&
            (nwalCompareByteArray((uint8_t *)&paEthInfo,
                                (uint8_t *)&pMacInfo->paEthInfo,
                                sizeof(paEthInfo2_t))))
        {
            found = nwal_TRUE;
            *pIfHandle = 
                (nwal_Handle*)NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                pMacInfo);
            break;
        }
        pMacInfo = nwal_getNextInst(pMacInfo,sizeof(nwalMacInfo_t));
        count++;
    }
    NWAL_osalCsExit(key);
    return(found);
}
/********************************************************************
 *  FUNCTION PURPOSE: Function to convert IP param from Application to
 *                    PA LLD structure
 ********************************************************************
 * DESCRIPTION: Function to convert IP param from Application to
 *              PA LLD structure
 ********************************************************************/
void nwal_convIpParam(nwal_IpType           ipType,
                      nwalIpAddr_t*         pDst,
                      nwalIpAddr_t*         pSrc,
                      nwalIpOpt_t*          pIpOpt,
                      paIpInfo2_t*          pPaIpInfo)
{
    /* Store the PA IP configuration */
    pPaIpInfo->ipType = ipType;
    if(pPaIpInfo->ipType  == pa_IPV4)
    {
        pPaIpInfo->validBitMap |= pa_IP_INFO_VALID_DST;
        memcpy(pPaIpInfo->dst.ipv4,&pDst->ipv4,NWAL_IPV4_ADDR_SIZE);
        if(pSrc)
        {
            pPaIpInfo->validBitMap |= pa_IP_INFO_VALID_SRC;
            memcpy(pPaIpInfo->src.ipv4,&pSrc->ipv4,NWAL_IPV4_ADDR_SIZE);
            
        }
    }
    else if(pPaIpInfo->ipType == pa_IPV6)
    {
        pPaIpInfo->validBitMap |= pa_IP_INFO_VALID_DST;
        memcpy(pPaIpInfo->dst.ipv6,&pDst->ipv6,NWAL_IPV6_ADDR_SIZE);
        if(pSrc)
        {
            pPaIpInfo->validBitMap |= pa_IP_INFO_VALID_SRC;
            memcpy(pPaIpInfo->src.ipv6,&pSrc->ipv6,NWAL_IPV6_ADDR_SIZE);
        }
    }

    if((pIpOpt->validParams & NWAL_IP_OPT_VALID_PARAMS_L4_PROTO) ==
        NWAL_IP_OPT_VALID_PARAMS_L4_PROTO)
    {
        pPaIpInfo->validBitMap |= pa_IP_INFO_VALID_PROTO;
        pPaIpInfo->proto = pIpOpt->proto;
    }

    if((pIpOpt->validParams & NWAL_IP_OPT_VALID_PARAMS_TOS) ==
        NWAL_IP_OPT_VALID_PARAMS_TOS)
    {
        pPaIpInfo->validBitMap |= pa_IP_INFO_VALID_TOS;
        pPaIpInfo->tos = pIpOpt->tos;
    }

    if((pIpOpt->validParams & NWAL_IP_OPT_VALID_PARAMS_FLOW_LABEL) ==
        NWAL_IP_OPT_VALID_PARAMS_FLOW_LABEL)
    {
        pPaIpInfo->validBitMap |= pa_IP_INFO_VALID_FLOW;
        pPaIpInfo->flow = pIpOpt->flowLabel;
    }
}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_setIPAddr() API Implementation
 ********************************************************************
 * DESCRIPTION: nwal_setIPAddr() API Implementation
 ********************************************************************/
nwal_RetValue nwal_setIPAddr  ( nwal_Inst           nwalInst,
                                nwal_TransID_t      transId,
                                nwal_AppId          appId,
                                nwal_Handle         ifHandle,
                                nwalIpParam_t*      pParam,
                                nwal_Handle*        pIpHandle)
{
    nwalIpInfo_t*       pNwalIpInfo;
    nwal_RetValue       nwalRetVal;
    nwalHandleHdr_t*    pHandleHdr;
    NWAL_STATE_COUNT_T  state;
    nwal_Handle         tmpHandle;
    uint32_t            key;
    nwalLocRouteInfo_t     routeInfo;
    nwal_Handle         ifHandleAddr;
    static unsigned char zeroIP6[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    if(ifHandle != nwal_HANDLE_INVALID)
    {
        ifHandleAddr =
            (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                            ifHandle));
        pHandleHdr = (nwalHandleHdr_t *)ifHandleAddr;
        /* Get the state of the upper layer handle */
        state = NWAL_GET_STATE(pHandleHdr->stateCount);
        if(state != NWAL_STATE_ACTIVE)
        {
            return nwal_ERR_INVALID_STATE;
        }

        nwal_InvPreHnd(ifHandleAddr);
    }

    NWAL_osalCsEnter(&key);
    tmpHandle = nwal_getInst((nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                nwalProcessCtx.pSharedMemBase->pIPInfo)),
                                nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxIpAddress,
                                sizeof(nwalIpInfo_t));
    if(tmpHandle == NULL)
    {
        NWAL_osalCsExit(key);
        return nwal_ERR_RES_UNAVAILABLE;
    }

    pNwalIpInfo = (nwalIpInfo_t *)tmpHandle;
    memset(pNwalIpInfo,0,sizeof(nwalIpInfo_t));
    
    pNwalIpInfo->prevHandle = ifHandle;

    if (pParam->ipType == nwal_IPV4)
    {
        if(!(memcmp(&pParam->remIpAddr, zeroIP6, sizeof(nwalIpv4Addr_t))))
        {
            pParam->validParams = 
                            pParam->validParams &~NWAL_SET_IP_VALID_PARAM_REMOTE_IP;
        }
    }
    else
    {
        if(!(memcmp(&pParam->remIpAddr, zeroIP6, sizeof(nwalIpv6Addr_t))))
        {
            pParam->validParams = 
                            pParam->validParams &~NWAL_SET_IP_VALID_PARAM_REMOTE_IP;
        }
    }
    if((pParam->validParams & NWAL_SET_IP_VALID_PARAM_REMOTE_IP) ==
        (NWAL_SET_IP_VALID_PARAM_REMOTE_IP))
    {
        /* Store the PA IP configuration, remote IP present */
        nwal_convIpParam(pParam->ipType,
                     &pParam->locIpAddr,
                     &pParam->remIpAddr,
                     &pParam->ipOpt,
                     &pNwalIpInfo->paIpInfo);
    }
    else
    {
        /* Store the PA IP configuration, remote IP not present */
        nwal_convIpParam(pParam->ipType,
                     &pParam->locIpAddr,
                     NULL,
                     &pParam->ipOpt,
                     &pNwalIpInfo->paIpInfo);
    }
    nwal_saveTransInfo(&pNwalIpInfo->transInfo,transId);
    pNwalIpInfo->transInfo.appId = appId;

    memset(&routeInfo, 0, sizeof(nwalLocRouteInfo_t));


    if ((pParam->validParams & NWAL_SET_IP_VALID_PARAM_ROUTE_TYPE) ==
            (NWAL_SET_IP_VALID_PARAM_ROUTE_TYPE))
    {
        routeInfo.routeType = pParam->routeType;
    }


    routeInfo.flowId = pParam->appRxPktFlowId;
    routeInfo.rxPktQ = pParam->appRxPktQueue;
    routeInfo.matchAction = pParam->matchAction;
    routeInfo.failAction = pParam->failAction;
    
    nwalRetVal = nwal_configIP(nwalProcessCtx.pSharedMemBase,
                               pNwalIpInfo,
                               &routeInfo);

    //*pIpHandle = pNwalIpInfo;
    *pIpHandle = (nwal_Handle*)NWAL_CONV_ADDRESS_TO_OFFSET((uint32_t)nwalProcessCtx.pSharedMemBase,
                             (uint32_t)pNwalIpInfo);

    if(nwalRetVal!= nwal_OK)
    {
        NWAL_osalCsExit(key);
        nwal_freeInst((nwal_Handle*)pNwalIpInfo,sizeof(nwalIpInfo_t));
        return(nwalRetVal);
    }

    if(transId == NWAL_TRANSID_SPIN_WAIT)
    {
        /* Block until response is received from NetCP */
        nwalRetVal =
            nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                          NULL,
                          NULL,
                          &pNwalIpInfo->transInfo.handleHdr);
    }
    NWAL_osalCsExit(key);
    return(nwalRetVal);

}
/********************************************************************
 *  FUNCTION PURPOSE: nwal_delIPAddr() API Implementation
 ********************************************************************
 * DESCRIPTION: nwal_setIPAddr() API Implementation
 ********************************************************************/
nwal_RetValue nwal_delIPAddr  ( nwal_Inst               nwalInst,
                                nwal_TransID_t          transId,
                                nwal_Handle             pIpHandle)
{
    nwalIpInfo_t*       pIpInfo =
        (nwalIpInfo_t *)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                   pIpHandle));
    nwal_RetValue       retVal;

    nwal_saveTransInfo(&pIpInfo->transInfo,transId);
    nwal_setTransInfo(&pIpInfo->transInfo,
                      NWAL_HANDLE_ID_TRANS_DEL_IP,
                      pIpInfo);

    retVal = nwalDelPaHandle(nwalProcessCtx.pSharedMemBase,
                             &pIpInfo->transInfo,
                             0,
                             &pIpInfo->paIpHandle);
    if(retVal!= nwal_OK)
    {
        nwal_debug_bk();
        return(retVal);
    }

    if(transId == NWAL_TRANSID_SPIN_WAIT)
    {
        /* Block until response is received from NetCP */
        retVal =
            nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                          NULL,
                          NULL,
                          &pIpInfo->transInfo.handleHdr);
    }
    NWAL_osalWriteBackCache(pIpInfo,sizeof(nwalIpInfo_t));
    if(retVal!= nwal_OK)
    {
        nwal_debug_bk();
        return(retVal);
    }
    return(retVal);

}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_getIPAddr() API Implementation
 ********************************************************************
 * DESCRIPTION: nwal_getIPAddr() API Implementation
 ********************************************************************/
nwal_Bool_t nwal_getIPAddr ( nwal_Inst          nwalInst,
                             nwalIpParam_t*     pParam,
                             nwal_Handle        prevHandle,
                             nwal_Handle*       pIpHandle)
{
    nwalIpInfo_t*       pNwalIpInfo;
    uint8_t             count=0;
    nwal_Bool_t         found = nwal_FALSE;
    paIpInfo2_t          paIpInfo;
    uint32_t            key;
    NWAL_STATE_COUNT_T  state;

    *pIpHandle = NULL;

    memset(&paIpInfo,0,sizeof(paIpInfo2_t));
    nwal_convIpParam(pParam->ipType,
                     &pParam->locIpAddr,
                     NULL,
                     &pParam->ipOpt,
                     &paIpInfo);

    pNwalIpInfo =
        (nwalIpInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                 nwalProcessCtx.pSharedMemBase->pIPInfo));
    NWAL_osalCsEnter(&key);
    while(count < nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxIpAddress)
    {
        NWAL_osalInvalidateCache(pNwalIpInfo,sizeof(nwalIpInfo_t));
        state = NWAL_GET_STATE(pNwalIpInfo->handleHdr.stateCount);
        if((state == NWAL_STATE_ACTIVE) &&
           (nwalCompareByteArray((uint8_t *)&paIpInfo,
                                (uint8_t *)&pNwalIpInfo->paIpInfo,
                                sizeof(paIpInfo2_t))))
        {
            if(prevHandle == pNwalIpInfo->prevHandle)
            {
                
                found = nwal_TRUE;
                *pIpHandle =
                    (nwal_Handle*)NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                    pNwalIpInfo);
                break;
            }
        }
        pNwalIpInfo = nwal_getNextInst(pNwalIpInfo,sizeof(nwalIpInfo_t));
        count++;
    }
    NWAL_osalCsExit(key);
    return(found);
}

/********************************************************************
 *  FUNCTION PURPOSE: Implementation for lookup of existing L2L3 handle
 ********************************************************************
 * DESCRIPTION: Implementation for lookup of existing L2L3 Hdr handle
 ********************************************************************/
nwal_Bool_t nwal_lkupL2L3Hdr( nwal_Inst               nwalInst,
                              nwalL2L3HdrParam_t*     pL2L3HdrParam,
                              nwalL2L3HdrInfo_t**     ppL2L3HdrInfo)
{
    uint8_t                 count=0;
    uint8_t*                pHdr;
    nwal_Handle outHandleAddrParam = NULL;
    nwal_Handle inHandleAddrParam = NULL;
    nwal_Handle outHandleAddrInfo = NULL;;
    nwal_Handle inHandleAddrInfo = NULL;;
    nwalL2L3HdrInfo_t*      pL2L3HdrInfo =
        (nwalL2L3HdrInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
        nwalProcessCtx.pSharedMemBase->pL2L3HdrInfo));

    while(count < nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxL2L3Hdr)
    {
        NWAL_osalInvalidateCache(pL2L3HdrInfo,sizeof(nwalL2L3HdrInfo_t));

        if(pL2L3HdrParam->outHandle)
            outHandleAddrParam =
                (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                            pL2L3HdrParam->outHandle));

        if(pL2L3HdrParam->inHandle)
            inHandleAddrParam =
                (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                            pL2L3HdrParam->inHandle));

        if(pL2L3HdrInfo->outHandle)
            outHandleAddrInfo =
                (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                            pL2L3HdrInfo->outHandle));

        if(pL2L3HdrInfo->inHandle)
            inHandleAddrInfo =
                (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                            pL2L3HdrInfo->inHandle));

        if((outHandleAddrInfo == outHandleAddrParam) &&
           (inHandleAddrInfo ==  inHandleAddrParam))
        {
            pHdr = pL2L3HdrInfo->hdrBuf;
            pHdr = pHdr + NWAL_ETH_OFFSET_DEST_MAC;
            /* Check match for remote MAC Address */
            if(nwalCompareByteArray((uint8_t *)pL2L3HdrParam->remMacAddr,
                                    pHdr,NWAL_MAC_ADDR_SIZE))
            {
                /* Destination MAC address matched.. Check for Destination
                 * IP Address
                 */
                pHdr = pL2L3HdrInfo->hdrBuf + pL2L3HdrInfo->ipOffset;
                if(pL2L3HdrInfo->ipVer == pa_IPV4)
                {
                    /* Check match for remote IP Address */
                    pHdr = pHdr + NWAL_IPV4_OFFSET_DST_ADDR;
                    //ipLen = NWAL_IPV4_ADDR_SIZE;
                    if(nwalCompareByteArray((uint8_t *)pL2L3HdrParam->remIpAddr.ipv6,
                                        pHdr,
                                        NWAL_IPV4_ADDR_SIZE))
                    {
                        *ppL2L3HdrInfo = 
                            (nwalL2L3HdrInfo_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                        pL2L3HdrInfo));
                        return(nwal_TRUE);
                    }
                }else if(pL2L3HdrInfo->ipVer == pa_IPV6)
                {
                    /* Check match for remote IP Address */
                    pHdr = pHdr + NWAL_IPV6_OFFSET_DST_ADDR;
                    //ipLen = NWAL_IPV6_ADDR_SIZE;
                    if(nwalCompareByteArray((uint8_t *)pL2L3HdrParam->remIpAddr.ipv6,
                                        pHdr,
                                        NWAL_IPV6_ADDR_SIZE))
                    {
                        *ppL2L3HdrInfo =
                              (nwalL2L3HdrInfo_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                                pL2L3HdrInfo));
                        return(nwal_TRUE);
                    }
                }
            }
        }

        pL2L3HdrInfo =
            nwal_getNextInst(pL2L3HdrInfo,sizeof(nwalL2L3HdrInfo_t));
        count++;
    }

    return(nwal_FALSE);
}
/********************************************************************
 *  FUNCTION PURPOSE: Create L2L3 handle containing MAC/[IPSec]/IP/UDP
 *                    header
 ********************************************************************
 * DESCRIPTION:  Create L2L3 handle containing MAC/[IPSec]/IP/UDP
 *                    header
 ********************************************************************/
nwal_RetValue nwal_createL2L3Hdr(nwal_Inst               nwalInst,
                                 nwalL2L3HdrParam_t*     pL2L3HdrParam,
                                 nwalL2L3HdrInfo_t**     ppL2L3HdrInfo)
{
    nwalIpInfo_t*           pIpInfo = NULL;
    nwalIpSecInfo_t*        pIpSecInfoIn = NULL;
    nwalIpSecInfo_t*        pIpSecInfoOut = NULL;
    nwalMacInfo_t*          pMacInfo = NULL;
    nwal_Handle             tmpHandle;
    nwalL2L3HdrInfo_t*      pL2L3HdrInfo;
    nwalHandleHdr_t*        pHndlHdr;
    uint8_t                 hdrLen,tmpLen;
    uint8_t*                pTmpHdr;
    uint8_t                 proto = 0;
    uint8_t                 tos = 0;
    uint32_t                flowLabel = 0;
    uint16_t                state,count;
    uint32_t                key;
    uint16_t                ethProtoType = NWAL_ETH_TYPE_IP;
    nwal_RetValue           retVal= nwal_OK;
    nwalMacAddr_t*          pRemMacAddr = &pL2L3HdrParam->remMacAddr;

    NWAL_osalCsEnter(&key);
    if(nwal_lkupL2L3Hdr(nwalProcessCtx.pSharedMemBase,pL2L3HdrParam,ppL2L3HdrInfo))
    {
        pHndlHdr = 
              (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                          *ppL2L3HdrInfo));
        state = NWAL_GET_STATE(pHndlHdr->stateCount);
        count = (pHndlHdr->stateCount & ~NWAL_STATE_MASK);
        count++;
        pHndlHdr->stateCount =
            (((state << NWAL_STATE_SHIFT) & NWAL_STATE_MASK) | count);

        /* ppL2L3HdrInfo is offset */
        //pL2L3HdrInfo = *ppL2L3HdrInfo;
//            (nwalL2L3HdrInfo_t*)NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
//                                                   *ppL2L3HdrInfo);
        NWAL_osalCsExit(key);
        return nwal_OK;
    }
    else
    {
        /* Get a Free MAC Instance */
        tmpHandle = nwal_getInst((nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS((uint32_t)nwalProcessCtx.pSharedMemBase,
                                (uint32_t)nwalProcessCtx.pSharedMemBase->pL2L3HdrInfo)),
                                 nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxL2L3Hdr,
                                 sizeof(nwalL2L3HdrInfo_t));

        if(tmpHandle == NULL)
        {
            NWAL_osalCsExit(key);
            return nwal_ERR_L2L3_UNAVAILABLE;
        }
        /* right now, tmpHandle is address */
        pL2L3HdrInfo = (nwalL2L3HdrInfo_t*)(tmpHandle);
    }

    /* inHandle and outHandle are offsets */
    pL2L3HdrInfo->inHandle = pL2L3HdrParam->inHandle;
    pL2L3HdrInfo->outHandle = pL2L3HdrParam->outHandle;
    pL2L3HdrInfo->handleHdr.handleId = NWAL_HANDLE_L2_L3_HDR_INST;

    pHndlHdr = 
         (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                        pL2L3HdrInfo->inHandle));

    /* Get all upper layer information for the header */
    if(pHndlHdr->handleId != NWAL_HANDLE_IP_INST)
    {
        /* Currently not supported */
        retVal = nwal_ERR_INVALID_HANDLE;
        goto ERR_nwal_createL2L3Hdr;
    }
    pIpInfo = 
        (nwalIpInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                   pL2L3HdrInfo->inHandle));
    pHndlHdr = 
         (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                       pIpInfo->prevHandle));
    if(pIpInfo->paIpInfo.ipType == pa_IPV6)
    {
        ethProtoType = NWAL_ETH_TYPE_IPv6;
    }

    if(pHndlHdr->handleId == NWAL_HANDLE_IPSEC_INST)
    {
        pIpSecInfoIn = 
            (nwalIpSecInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                       pIpInfo->prevHandle));
        /* Check for valid output IPSec Handle */
        if(pL2L3HdrInfo->outHandle == NULL)
        {
            retVal = nwal_ERR_INVALID_HANDLE;
            goto ERR_nwal_createL2L3Hdr;
        }
        pHndlHdr =
             (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                        pL2L3HdrInfo->outHandle));
        if(pHndlHdr->handleId == NWAL_HANDLE_IPSEC_INST)
        {
            //pIpSecInfoOut = (nwalIpSecInfo_t*)(pL2L3HdrInfo->outHandle);
             pIpSecInfoOut = (nwalIpSecInfo_t*)pHndlHdr;
            /* Override destination MAC address to the destination of Tunnel
             * This is to handle the case if tunnel is not terminating to
             * final endpoint
             */
            pRemMacAddr = &pIpSecInfoOut->remMacAddr;
        }
        else
        {
            retVal = nwal_ERR_INVALID_HANDLE;
            goto ERR_nwal_createL2L3Hdr;
        }

        /* Reset handle header to previous for inbound to search for
         * MAC handle
         */
        pHndlHdr = 
        (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                       pIpSecInfoIn->prevHandle));
    }

    if(pHndlHdr->handleId != NWAL_HANDLE_MAC_INST)
    {
        retVal = nwal_ERR_INVALID_HANDLE;
        goto ERR_nwal_createL2L3Hdr;
    }
    pMacInfo = (nwalMacInfo_t *)pHndlHdr;

    /* Initialize the Header */
    hdrLen = NWAL_MAX_L2_L3_HDR_BUF_SIZE;
    memset(pL2L3HdrInfo->hdrBuf,0,hdrLen);
    /* Prepare the MAC Header */
    tmpLen = nwal_prepMacHdr( pL2L3HdrInfo->hdrBuf,
                              hdrLen,
                              &pL2L3HdrParam->macOpt,
                              ethProtoType,
                              pMacInfo->paEthInfo.dst,
                              *pRemMacAddr);
    hdrLen -= tmpLen;
    pTmpHdr = pL2L3HdrInfo->hdrBuf + tmpLen;
    if((pIpSecInfoIn) && (pIpSecInfoIn->saMode == nwal_SA_MODE_TUNNEL))
    {
        pL2L3HdrInfo->outerIpOffset = tmpLen;
        pL2L3HdrInfo->outerIpVer = pIpSecInfoIn->paIpInfo.ipType;
         proto = NWAL_IP_PROTO_IPV4;
        if(pIpInfo->paIpInfo.ipType == pa_IPV6)
        {
            proto = NWAL_IP_PROTO_IPV6;
        }

        if(pIpSecInfoIn->paIpInfo.ipType == pa_IPV6)
        {
            if(!(nwal_prepIpv6Hdr(pTmpHdr,
                                     hdrLen,
                                     pIpSecInfoIn->paIpInfo.tos,
                                     pIpSecInfoIn->paIpInfo.flow,
                                     proto,
                                     nwalProcessCtx.pSharedMemBase->cfg.hopLimit,
                                     pIpSecInfoIn->paIpInfo.dst.ipv6,
                                     pIpSecInfoOut->paIpInfo.dst.ipv6)))
            {
                retVal = nwal_ERR_NO_FREE_BUF;
                goto ERR_nwal_createL2L3Hdr;
            }
            tmpLen =  tmpLen + NWAL_IPV6_HDR_LEN_BYTES;
            hdrLen -= NWAL_IPV6_HDR_LEN_BYTES;
            pTmpHdr += NWAL_IPV6_HDR_LEN_BYTES;
        }
        else
        {
            if(!(nwal_prepIpv4Hdr(pTmpHdr,
                                     hdrLen,
                                     pIpSecInfoIn->paIpInfo.tos,
                                     nwalProcessCtx.pSharedMemBase->cfg.hopLimit,
                                     proto,
                                     pIpSecInfoIn->paIpInfo.dst.ipv4,
                                     pIpSecInfoOut->paIpInfo.dst.ipv4)))
            {
                retVal = nwal_ERR_NO_FREE_BUF;
                goto ERR_nwal_createL2L3Hdr;
            }
            /* Initialize the IP header checksum for Outer IP as checksum
             * will be updated by software: NWAL
             */
            nwal_updateIpInitLen(pTmpHdr,
                                 ((NWAL_IPV4_HDR_LEN_BYTES*2)+
                                   NWAL_UDP_HDR_LEN_BYTES));

            tmpLen =  tmpLen + NWAL_IPV4_HDR_LEN_BYTES;
            hdrLen -= NWAL_IPV4_HDR_LEN_BYTES;
            pTmpHdr += NWAL_IPV4_HDR_LEN_BYTES;
        }
    }

    pL2L3HdrInfo->ipOffset = tmpLen;
    pL2L3HdrInfo->ipVer = pIpInfo->paIpInfo.ipType;

    if(pL2L3HdrInfo->ipVer != pL2L3HdrParam->ipType)
    {
        retVal = nwal_ERR_INVALID_PARAM;
        goto ERR_nwal_createL2L3Hdr;
    }

    proto = NWAL_IPV4_UDP;
    if((pL2L3HdrParam->ipOpt.validParams & NWAL_IP_OPT_VALID_PARAMS_L4_PROTO) ==
       NWAL_IP_OPT_VALID_PARAMS_L4_PROTO)
    {
        proto = pL2L3HdrParam->ipOpt.proto;
        if(pL2L3HdrParam->ipOpt.proto != NWAL_IPV4_UDP)
        {
            retVal = nwal_ERR_INVALID_PARAM;
            goto ERR_nwal_createL2L3Hdr;
        }
    }

    if((pL2L3HdrParam->ipOpt.validParams & NWAL_IP_OPT_VALID_PARAMS_TOS) ==
       NWAL_IP_OPT_VALID_PARAMS_TOS)
    {
        tos = pL2L3HdrParam->ipOpt.tos;
    }

    if((pL2L3HdrParam->ipOpt.validParams &
        NWAL_IP_OPT_VALID_PARAMS_FLOW_LABEL) ==
       NWAL_IP_OPT_VALID_PARAMS_FLOW_LABEL)
    {
        flowLabel = pL2L3HdrParam->ipOpt.flowLabel;
    }

    if(pIpInfo->paIpInfo.ipType == pa_IPV6)
    {
        if(!(nwal_prepIpv6Hdr(pTmpHdr,
                                 hdrLen,
                                 tos,
                                 flowLabel,
                                 proto,
                                 nwalProcessCtx.pSharedMemBase->cfg.hopLimit,
                                 pIpInfo->paIpInfo.dst.ipv6,
                                 pL2L3HdrParam->remIpAddr.ipv6)))
        {
            retVal = nwal_ERR_NO_FREE_BUF;
            goto ERR_nwal_createL2L3Hdr;
        }
        tmpLen =  tmpLen + NWAL_IPV6_HDR_LEN_BYTES;
        hdrLen -= NWAL_IPV6_HDR_LEN_BYTES;
        pTmpHdr += NWAL_IPV6_HDR_LEN_BYTES;
    }
    else
    {
        if(!(nwal_prepIpv4Hdr(pTmpHdr,
                                 hdrLen,
                                 tos,
                                 nwalProcessCtx.pSharedMemBase->cfg.hopLimit,
                                 proto,
                                 pIpInfo->paIpInfo.dst.ipv4,
                                 pL2L3HdrParam->remIpAddr.ipv4)))
        {
            retVal = nwal_ERR_NO_FREE_BUF;
            goto ERR_nwal_createL2L3Hdr;
        }


        /* Initialize the IP header checksum */
        nwal_updateIpInitLen(pTmpHdr,
                             (NWAL_IPV4_HDR_LEN_BYTES +
                              NWAL_UDP_HDR_LEN_BYTES));

        tmpLen =  tmpLen + NWAL_IPV4_HDR_LEN_BYTES;
        hdrLen -= NWAL_IPV4_HDR_LEN_BYTES;
        pTmpHdr += NWAL_IPV4_HDR_LEN_BYTES;
    }

    pL2L3HdrInfo->hdrLen = tmpLen;
    pL2L3HdrInfo->handleHdr.stateCount =
                NWAL_SET_STATE(pL2L3HdrInfo->handleHdr.stateCount,
                               NWAL_STATE_ACTIVE);
    *ppL2L3HdrInfo = pL2L3HdrInfo;
    NWAL_osalWriteBackCache(pL2L3HdrInfo,sizeof(nwalL2L3HdrInfo_t));
    NWAL_osalCsExit(key);
    *ppL2L3HdrInfo = 
        (nwalL2L3HdrInfo_t*)NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                               pL2L3HdrInfo);
    
    return(nwal_OK);

ERR_nwal_createL2L3Hdr:
    NWAL_osalCsExit(key);
    nwal_freeInst((nwal_Handle*)pL2L3HdrInfo,sizeof(nwalL2L3HdrInfo_t));
    return(retVal);
}

/********************************************************************
 *  FUNCTION PURPOSE: Delete L2L3 handle containing MAC/[IPSec]/IP/UDP
 *                    header
 ********************************************************************
 * DESCRIPTION:  Delete L2L3 handle containing MAC/[IPSec]/IP/UDP
 *               header
 ********************************************************************/
nwal_RetValue nwal_deleteL2L3Hdr(nwal_Inst               nwalInst,
                                 nwal_Handle             l2l3HdrHandle)
{
    nwalHandleHdr_t*        pHandleHdr = 
            (nwalHandleHdr_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                           l2l3HdrHandle));
    uint16_t                state,count;
    uint32_t                key;

    NWAL_osalCsEnter(&key);
    state = NWAL_GET_STATE(pHandleHdr->stateCount);
    count = (pHandleHdr->stateCount & ~NWAL_STATE_MASK);
    count--;

    if(!count)
    {
        pHandleHdr->stateCount =
            NWAL_SET_STATE(pHandleHdr->stateCount,NWAL_STATE_INACTIVE);
    }
    else
    {
        pHandleHdr->stateCount =
            (((state << NWAL_STATE_SHIFT) & NWAL_STATE_MASK) | count);
    }

    NWAL_osalCsExit(key);
    return nwal_OK;
}
/********************************************************************
 *  FUNCTION PURPOSE: nwal_addConn() API
 ********************************************************************
 * DESCRIPTION:  nwal_addConn() API
 ********************************************************************/
nwal_RetValue nwal_addConn( nwal_Inst               nwalInst,
                            nwal_TransID_t          transId,
                            nwal_AppId              appId,
                            nwal_appProtoType_t     proto,
                            nwalRxConnCfg_t*        pRxConnCfg,
                            nwalTxConnCfg_t*        pTxConnCfg,
                            nwal_Handle*            pNwalConHandle)
{
    nwalPortInfo_t*     pPortInfo;
    paHandleL2L3_t      prevLink = NULL;
    nwal_RetValue       retVal;
    //nwalGlobContext_t*  pNwalHandle  = (nwalGlobContext_t*)nwalInst;
    nwal_Handle         tmpHandle;
    nwalHandleHdr_t *   pHndlHdr;
    nwalIpInfo_t*       pTmpIPInfo;
    nwalHandleHdr_t     tmpHdr;
    nwalLocContext_t*   pLocContext;
    uint32_t            key;
    uint32_t            instAdjSize;
    nwalLocRouteInfo_t     routeInfo;
    nwal_Handle         rxConnInHandleAddr;
    uint16_t cacheLineSize = NWAL_CACHE_LINE_SIZE;
    uint8_t*    pLocContextTmp;
    if(!pRxConnCfg)
    {
        /* Local configuration is mandatory */
        return(nwal_ERR_INVALID_PARAM);
    }
    NWAL_osalInvalidateCache(nwalProcessCtx.pSharedMemBase, sizeof(nwalGlobContext_t));
    if(pRxConnCfg->rxCoreId >= nwalProcessCtx.pSharedMemBase->memSizeInfo.nProc)
    {
        return (nwal_ERR_INVALID_PROC_ID);
    }
    instAdjSize = nwal_round_size(cacheLineSize, 1, sizeof(nwalLocContext_t));
    //pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    pLocContextTmp = (uint8_t*)((uint32_t)nwalProcessCtx.pLocalCtxInstPool + (instAdjSize * pRxConnCfg->rxCoreId));
    pLocContext = (nwalLocContext_t*)pLocContextTmp;

    /* Check the state of the core terminating the packet */
    NWAL_osalInvalidateCache(pLocContext,sizeof(nwalLocContext_t));
    if(pLocContext->state != NWAL_LOC_INSTANCE_ACTIVE)
    {
        return(nwal_ERR_INVALID_STATE);
    }

    NWAL_osalCsEnter(&key);
    tmpHandle = nwal_getInst((void*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                nwalProcessCtx.pSharedMemBase->pPortInfo)),
                                nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxL4Ports,
                                sizeof(nwalPortInfo_t));
    /* Since there is no lookup for connection. multicore protection
     * not needed for full update
     */
    NWAL_osalCsExit(key);
    if(tmpHandle == NULL)
    {
        return nwal_ERR_PORT_UNAVAILABLE;
    }
    pPortInfo = (nwalPortInfo_t*)(tmpHandle);
    tmpHdr = pPortInfo->handleHdr;
    memset(pPortInfo,0,sizeof(nwalPortInfo_t));
    pPortInfo->handleHdr = tmpHdr;

    /* Store the required parameters for later use */
    nwal_saveTransInfo(&pPortInfo->transInfo,transId);
    pPortInfo->transInfo.appId = appId;
    pPortInfo->isRemActive =  nwal_FALSE;
    pPortInfo->proto = proto;

    /* Add entry for LUT2 rule based on local configuration */
    memcpy(&pPortInfo->rxConnCfg,pRxConnCfg,sizeof(nwalRxConnCfg_t));
    if(pTxConnCfg != NULL)    {
        memcpy(&pPortInfo->txConnCfg,pTxConnCfg,sizeof(nwalTxConnCfg_t));
        retVal = nwal_cfgConn((nwal_Inst)nwalProcessCtx.pSharedMemBase,
                              (nwal_Handle)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,pPortInfo)),
                              pTxConnCfg);
        if(retVal != nwal_OK)
        {
            goto ERR_nwal_addConn;
        }
    }
    /* inHandle is offset, needs to change to address*/
    rxConnInHandleAddr =
            (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                        pRxConnCfg->inHandle));

    
    if(pRxConnCfg->inHandle != nwal_HANDLE_INVALID)
    {
        rxConnInHandleAddr =
            (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                        pRxConnCfg->inHandle));
        /* Get the PA handle for previous link */
        pHndlHdr = (nwalHandleHdr_t *)(rxConnInHandleAddr);
        if(pHndlHdr->handleId != NWAL_HANDLE_IP_INST)
        {
            retVal = nwal_ERR_INVALID_PARAM;
            goto ERR_nwal_addConn;
        }
        pTmpIPInfo = (nwalIpInfo_t *)(rxConnInHandleAddr);
        prevLink = pTmpIPInfo->paIpHandle;
        
    }

    memset(&routeInfo, 0, sizeof(nwalLocRouteInfo_t));
    if ((pRxConnCfg->validParams & NWAL_SET_CONN_VALID_PARAM_ROUTE_TYPE) ==
            (NWAL_SET_CONN_VALID_PARAM_ROUTE_TYPE))
    {
        routeInfo.routeType = pRxConnCfg->routeType;
    }

    if(pRxConnCfg->appRxPktQueue == QMSS_PARAM_NOT_SPECIFIED)
    {
        routeInfo.rxPktQ = pLocContext->rxL4PktQ;
    }
    else
    {
        routeInfo.rxPktQ = pRxConnCfg->appRxPktQueue;
    }

    if(pRxConnCfg->appRxPktFlowId == CPPI_PARAM_NOT_SPECIFIED)
    {
        routeInfo.flowId = pLocContext->rxPktFlowId;
    }
    else
    {
        routeInfo.flowId = pRxConnCfg->appRxPktFlowId;
    }

    routeInfo.matchAction = pRxConnCfg->matchAction;
    retVal = nwal_configPort(nwalProcessCtx.pSharedMemBase,
                             pPortInfo,
                             prevLink,
                             &routeInfo);
    if(retVal != nwal_OK)
    {
        if(pPortInfo->pL2L3HdrInfo)
        {
            nwal_deleteL2L3Hdr(nwalProcessCtx.pSharedMemBase,pPortInfo->pL2L3HdrInfo);
        }
        goto ERR_nwal_addConn;
    }

    *pNwalConHandle =
        (nwal_Handle*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                   pPortInfo));
    if(transId == NWAL_TRANSID_SPIN_WAIT)
    {
        /* Block until response is received from NetCP */
        retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                               NULL,
                               NULL,
                               &pPortInfo->transInfo.handleHdr);
    }
    NWAL_osalWriteBackCache(pPortInfo, sizeof(nwalPortInfo_t));
    return(retVal);

ERR_nwal_addConn:
    nwal_freeInst((nwal_Handle*)pPortInfo,sizeof(nwalPortInfo_t));
    return retVal;
}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_cfgConn() API
 ********************************************************************
 * DESCRIPTION:  nwal_cfgConn() API
 ********************************************************************/
nwal_RetValue nwal_cfgConn(     nwal_Inst           nwalInst,
                                nwal_Handle         pNwalConHandle,
                                nwalTxConnCfg_t*    pTxConnCfg)
{
    nwalPortInfo_t*     pPortInfo;
    nwalL2L3HdrParam_t  l2l3HdrParam;
    nwal_RetValue       retVal;
    uint16_t            srcPort,dstPort;

     pPortInfo =
        (nwalPortInfo_t *)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                   pNwalConHandle));
    if(pTxConnCfg != NULL)
    {
        memcpy(&pPortInfo->txConnCfg,pTxConnCfg,sizeof(nwalTxConnCfg_t));

        /* Create L2/L3 header for connection. Routine will first check if
         * there isalready similar L2/L3 header available before creating a
         * new one
         */
        memset(&l2l3HdrParam,0,sizeof(nwalL2L3HdrParam_t));

        l2l3HdrParam.inHandle = pPortInfo->rxConnCfg.inHandle;
        l2l3HdrParam.outHandle = pPortInfo->txConnCfg.outHandle;

        memcpy(l2l3HdrParam.remMacAddr,
               pTxConnCfg->remMacAddr,
               sizeof(nwalMacAddr_t));
        l2l3HdrParam.macOpt = pTxConnCfg->macOpt;

        l2l3HdrParam.ipType = pTxConnCfg->ipType;
        memcpy(&l2l3HdrParam.remIpAddr,
               &pTxConnCfg->remIpAddr,
               sizeof(nwalIpAddr_t));
        l2l3HdrParam.ipOpt = pTxConnCfg->ipOpt;
        retVal = nwal_createL2L3Hdr((nwal_Inst)nwalProcessCtx.pSharedMemBase,
                                    &l2l3HdrParam,
                                    &pPortInfo->pL2L3HdrInfo);
        if(retVal != nwal_OK)
        {
            return (retVal);
        }

        /* Create UDP header for the connection */
        memset(pPortInfo->hdrBuf,0,NWAL_UDP_HDR_LEN_BYTES);
        if(pPortInfo->proto == NWAL_APP_PLOAD_PROTO_GTPU)
        {
            srcPort = NWAL_GTPU_UDP_PORT;
            dstPort = NWAL_GTPU_UDP_PORT;
        }
        else if(pPortInfo->proto == NWAL_APP_PLOAD_PROTO_UDP)
        {
            srcPort = pPortInfo->rxConnCfg.appProto.udpPort;
            dstPort = pTxConnCfg->appProto.udpPort;
        }
        else /* Default NWAL_APP_PLOAD_16_BIT_PORT */
        {
            srcPort = pPortInfo->rxConnCfg.appProto.port;
            dstPort = pTxConnCfg->appProto.port;
        }
        nwal_prepUdpHdr(pPortInfo->hdrBuf,
                        NWAL_UDP_HDR_LEN_BYTES,
                        srcPort,
                        dstPort);


        pPortInfo->isRemActive =  nwal_TRUE;
    }

    return(nwal_OK);
}

/********************************************************************
 *  FUNCTION PURPOSE: nwal_delConn() API
 ********************************************************************
 * DESCRIPTION:  nwal_delConn() API
 ********************************************************************/
nwal_RetValue nwal_delConn( nwal_Inst               nwalInst ,
                            nwal_TransID_t          transId,
                            nwal_Handle             pNwalConHandle)
{
    nwal_RetValue        retVal;
    nwalPortInfo_t*      pPortInfo;

    pPortInfo =
        (nwalPortInfo_t *)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                   pNwalConHandle));
    nwal_saveTransInfo(&pPortInfo->transInfo,transId);

    nwal_setTransInfo(&pPortInfo->transInfo,
                      NWAL_HANDLE_ID_TRANS_DEL_PORT,
                      pPortInfo);
    retVal = nwalDelPaHandle(nwalProcessCtx.pSharedMemBase,
                             &pPortInfo->transInfo,
                             pPortInfo->paPortHandle,
                             0);
    if(retVal!= nwal_OK)
    {
        return(retVal);
    }

    if(transId == NWAL_TRANSID_SPIN_WAIT)
    {
        /* Block until response is received from NetCP */
        retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                               NULL,
                               NULL,
                               &pPortInfo->transInfo.handleHdr);
    }
    NWAL_osalWriteBackCache(pPortInfo,sizeof(nwalPortInfo_t));
    return(retVal);
}

/********************************************************************
 *  FUNCTION PURPOSE: API to nwal_refreshConn() for a core
 ********************************************************************
 * DESCRIPTION:  nwal_refreshConn() API
 ********************************************************************/
nwal_RetValue nwal_refreshConn( nwal_Inst               nwalInst,
                                nwal_Handle             nwalHandle)
{
    NWAL_osalInvalidateCache(nwalProcessCtx.pSharedMemBase, sizeof(nwalGlobContext_t));
    nwal_Handle nwalHandleAddr = 
                    (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                nwalHandle));

    nwal_InvPreHnd(nwalHandleAddr);
    return(nwal_OK);

}

/********************************************************************
 *  FUNCTION PURPOSE: Function for enabling debugging in the case
 *                    of unexpected error
 ********************************************************************
 * DESCRIPTION:  Function for enabling debugging in the case
 *               of unexpected error
 ********************************************************************/
void nwal_debug_bk()
{
    volatile    uint16_t  debug =1;
    while(debug);
}

/********************************************************************
 * FUNCTION PURPOSE: Receive and process command Response
 ********************************************************************
 * DESCRIPTION: Process command Response
 ********************************************************************/
static inline void nwal_resubmitMsg  (nwalGlobContext_t*    pIhandle,
                                      nwalLocContext_t*     pLocContext,
                                      int                   cmdDest,
                                      Cppi_HostDesc*        pHd)
{
    uint32_t            psCmd;
    Qmss_Queue          q;
    nwalBufPool_t*      pLinkBufRxQ = NULL;
    uint8_t*            pDataBuffer;
    uint32_t            origBuffLen;

    /* Get Linked Buffer q for the RX */
    Cppi_getOriginalBufInfo(Cppi_DescType_HOST,
                            (Cppi_Desc *)pHd,
                            (uint8_t**)&pDataBuffer,
                            &origBuffLen);
    pLinkBufRxQ = nwal_getLinkedBufQ(pLocContext->cfg.rxCtlPool.bufPool,
                                     origBuffLen,
                                     pLocContext->cfg.rxCtlPool.numBufPools);

    q.qMgr = 0;
    q.qNum = Pktlib_getInternalHeapQueue(pLinkBufRxQ->heapHandle);
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)pHd, q);

    /* Mark packet as a configuration packet */
    psCmd = PASAHO_PACFG_CMD;
    Cppi_setPSData (Cppi_DescType_HOST,
                    (Cppi_Desc *)pHd,
                    (uint8_t *)&psCmd,
                    4);

    /* Update Cache */
    NWAL_osalWriteBackCache((void *)pHd->buffPtr,pHd->buffLen);
    NWAL_osalWriteBackCache(pHd,NWAL_DESC_SIZE);

    /* Resend command to PA. */
    Qmss_queuePushDescSize(pIhandle->txQ[cmdDest],
                            (Ptr)pHd,
                            NWAL_DESC_SIZE);

    /*update global stats; need to be multicore protected*/
    pLocContext->stats.numResubmits++;
    return;
}

/********************************************************************
 * FUNCTION PURPOSE: Do additional sanity check and update internal
 *                   state of NWAL for incoming control response
 ********************************************************************
 * DESCRIPTION: Updated internal state for control Response
 ********************************************************************/
static inline void nwal_procCtlResp  (nwalGlobContext_t*    pIhandle,
                                      nwalLocContext_t*     pLocContext,
                                      nwalIntTransInfo_t*   pTransInfo,
                                      NWAL_HANDLE_TYPE_ID_T handleID,
                                      Cppi_HostDesc*        pHd,
                                      paEntryHandle_t*      pRetHandle,
                                      NWAL_STATE_COUNT_T    state,
                                      nwal_RetValue         nwalRet,
                                      nwal_CmdCallBack*     pCmdCallBack)
{
    nwal_Bool_t         transIsValid = nwal_TRUE;
    uint16_t            instSize=0;
    nwal_TransID_t      transId = pTransInfo->transId;
    nwal_AppId          appId = pTransInfo->appId;

    switch(handleID)
    {
        case NWAL_HANDLE_ID_TRANS_ADD_MAC:
            {
                nwalMacInfo_t       *pMacInfo;

                instSize = sizeof(nwalMacInfo_t);
                pMacInfo = (nwalMacInfo_t *)
                           (pTransInfo->nwalHandle);

                if(pMacInfo->paMacHandle != pRetHandle->l2l3Handle)
                {
                    pLocContext->stats.errHandleMismatch++;
                    nwal_debug_bk();
                }

                /* Update the state */
                pMacInfo->handleHdr.stateCount =
                    NWAL_SET_STATE(pMacInfo->handleHdr.stateCount,state);
                break;
            }
        case NWAL_HANDLE_ID_TRANS_DEL_MAC:
            {
                instSize = sizeof(nwalMacInfo_t);
                nwal_freeInst(pTransInfo->nwalHandle,
                              sizeof(nwalMacInfo_t));
                break;
            }
        case NWAL_HANDLE_ID_TRANS_ADD_IP:
            {
                nwalIpInfo_t       *pIpInfo;

                instSize = sizeof(nwalIpInfo_t);
                pIpInfo = (nwalIpInfo_t *)
                           (pTransInfo->nwalHandle);

                if(pIpInfo->paIpHandle != pRetHandle->l2l3Handle)
                {
                    pLocContext->stats.errHandleMismatch++;
                    nwal_debug_bk();
                }

                /* Update the state */
                pIpInfo->handleHdr.stateCount =
                    NWAL_SET_STATE(pIpInfo->handleHdr.stateCount,state);

                break;
            }

        case NWAL_HANDLE_ID_TRANS_DEL_IP:
            {
                instSize = sizeof(nwalIpInfo_t);
                nwal_freeInst(pTransInfo->nwalHandle,
                              sizeof(nwalIpInfo_t));
                break;
            }

        case NWAL_HANDLE_ID_TRANS_ADD_IPSEC:
            {
                nwalIpSecInfo_t       *pIpSecInfo;

                instSize = sizeof(nwalIpSecInfo_t);
                pIpSecInfo = (nwalIpSecInfo_t *)
                             (pTransInfo->nwalHandle);

                if(pIpSecInfo->paIpHandle != pRetHandle->l2l3Handle)
                {
                    pLocContext->stats.errHandleMismatch++;
                    nwal_debug_bk();
                }


                /* Update the state */
                pIpSecInfo->hdr.handleHdr.stateCount =
                    NWAL_SET_STATE(pIpSecInfo->handleHdr.stateCount,state);
                break;
            }

#ifdef NWAL_ENABLE_SA
        case NWAL_HANDLE_ID_TRANS_DEL_IPSEC:
            {
                instSize = sizeof(nwalIpSecInfo_t);
                /* Free the SA Channel and IPSEC Instance */
                nwal_freeSaChan(pIhandle,
                                (nwalIpSecInfo_t*)(pTransInfo->nwalHandle));
                break;
            }
#endif
        case NWAL_HANDLE_ID_TRANS_ADD_PORT:
            {
                nwalPortInfo_t       *pPortInfo;

                instSize = sizeof(nwalPortInfo_t);
                pPortInfo = (nwalPortInfo_t *)
                           (pTransInfo->nwalHandle);
                if(memcmp(pPortInfo->paPortHandle,pRetHandle->l4Handle,sizeof(paHandleL4_t)))
                {
                    pLocContext->stats.errHandleMismatch++;
                }

                /* Update the state */
                pPortInfo->handleHdr.stateCount =
                    NWAL_SET_STATE(pPortInfo->handleHdr.stateCount,state);
                break;
            }
        case NWAL_HANDLE_ID_TRANS_DEL_PORT:
            {
                nwalPortInfo_t       *pPortInfo;

                instSize = sizeof(nwalPortInfo_t);
                pPortInfo = (nwalPortInfo_t *)
                           (pTransInfo->nwalHandle);
                if(pPortInfo->pL2L3HdrInfo)
                {
                    nwal_deleteL2L3Hdr(pTransInfo->nwalHandle,
                                       pPortInfo->pL2L3HdrInfo);
                }
                nwal_freeInst(pTransInfo->nwalHandle,
                              sizeof(nwalPortInfo_t));
                break;
            }
        case NWAL_HANDLE_ID_TRANS_PA_GLOB_CFG:
        {
            /* No Action Required */
            return;
        }

        default:
            pLocContext->stats.inActiveTrans++;
            nwal_debug_bk();
            transIsValid = nwal_FALSE;
            break;
    }

    if(transIsValid)
    {
        if((pCmdCallBack) &&
            (transId != NWAL_TRANSID_SPIN_WAIT))
        {
            pCmdCallBack(appId,
                         transId,
                         nwalRet);
        }
    }
    /* Update the state information for instance */
    NWAL_osalWriteBackCache((void *)pTransInfo->nwalHandle,instSize);
    return;
}


/********************************************************************
 * FUNCTION PURPOSE: Receive and process command Response
 ********************************************************************
 * DESCRIPTION: Process command Response
 ********************************************************************/
nwal_RetValue nwal_pollCtlQ  (nwalGlobContext_t*      pIhandle,
                              nwal_CmdCallBack*       pCmdCallBack,
                              nwal_CmdPaStatsReply*   pPaStatsCallBack,
                              nwalHandleHdr_t*        pRefHandleHdr)
{

    paReturn_t          netCPRet=pa_OK;
    int                 handleType,cmdDest;
    nwal_RetValue       nwalRet = nwal_OK;
    NWAL_STATE_COUNT_T  state;
    paSysStats_t*       stats;
    nwalHandleHdr_t*    pHandleHdr = NULL;
    Cppi_HostDesc*      pHd;
    uint32_t*           pSwInfo0;
    nwalLocContext_t*   pLocContext;
    paEntryHandle_t     retHandle;
    nwalIntTransInfo_t* pTransInfo;
#ifdef NWAL_DUMP_PKT
    uint8_t *           pTmpBuf;
    uint32_t            tmpLen;
#endif

    pLocContext = nwal_getLocContext(pIhandle);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    /* Set default callback if not passed by Application */
    if(pPaStatsCallBack == NULL)
    {
        pPaStatsCallBack = pLocContext->cfg.pPaStatsCallBack;
    }

    if(pCmdCallBack == NULL)
    {
        pCmdCallBack = pLocContext->cfg.pCmdCallBack;
    }

    while(1)
    {
        while(Qmss_getQueueEntryCount (pLocContext->rxCtlQ) > 0)
        {
            /* Process all pending packets from the control Queue */
            pHd =
                (Cppi_HostDesc *)
                QMSS_DESC_PTR(Qmss_queuePop (pLocContext->rxCtlQ));
            if(pHd == NULL)
            {
                /* Unexpected */
                return(nwal_ERR_QMSS);
            }

            /* Invalidate cache before accessing contents */
            NWAL_osalInvalidateCache(pHd,NWAL_CACHE_LINE_SIZE);
            NWAL_osalInvalidateCache((void *)pHd->buffPtr,pHd->buffLen);

            /* Sanity check on descriptor for the command response */
            if (Cppi_getSoftwareInfo (Cppi_DescType_HOST,
                                     (Cppi_Desc *)pHd,
                                     (uint8_t **) &pSwInfo0) ==
                                     CPPI_EPIB_NOT_PRESENT)
            {
                /* Error case. Not expected to reach here */
                //nwal_debug_bk();
                pLocContext->stats.errNonEpibPkts++;
                return (nwal_ERR_CPPI);
            }

            /* Check from handle for the response type received from PASS */
            pHandleHdr = (nwalHandleHdr_t *)(*pSwInfo0);
            pTransInfo = (nwalIntTransInfo_t *)(*pSwInfo0);
            if((pHandleHdr->handleId & NWAL_HANDLE_ID_MASK) ==
                NWAL_HANDLE_ID_TRANS_PA_STAT)
            {
                /* PA Response stats handling */
                stats = (paSysStats_t *)
                         Pa_formatStatsReply (pIhandle->memSizeInfo.pahandle,
                                              (paCmd_t)pHd->buffPtr);
                if(stats == NULL)
                {
                    /* Sanity check failed. Unexpected */
                    return (nwal_ERR_PA);
                }

                /* Store the stats locally and pass stats to application
                 * through call back
                 */
                memcpy(&pLocContext->paSysStats,stats,sizeof(paSysStats_t));
                if((pPaStatsCallBack != NULL) &&
                   (pTransInfo->transId != NWAL_TRANSID_SPIN_WAIT))
                {
                    pPaStatsCallBack(pTransInfo->appId,
                                     pTransInfo->transId,
                                     &pLocContext->paSysStats);
                }
                /* Return the Descriptor back to the appropriate RX linked
                 * buffer Q
                 */
                nwal_retBuf(pIhandle,nwal_TRUE,nwal_TRUE,pHd);

                return(nwalRet);
            }

#ifdef NWAL_DUMP_PKT
            printf("NWAL:Command Response from PA \n");
            pTmpBuf = nwalCppi_getPSData(Cppi_DescType_HOST,
                                         Cppi_PSLoc_PS_IN_DESC,
                                        (Cppi_Desc *)pHd,
                                        &tmpLen);
            printf("NWAL:LOG Dump of PS Data BEGIN \n");
            nwal_dump_buf_32bit(pTmpBuf,tmpLen);
            printf("NWAL:LOG Dump of PS Data END \n");

            printf("NWAL:LOG Dump of PKT BEGIN \n");
            nwal_dump_buf((uint8_t *)pHd->buffPtr,pHd->buffLen);
            printf("NWAL:LOG Dump of PKT Data END \n");
#endif
            /* Forward the result to LLD for all other control responses
             * to check result for the request
             */
            netCPRet = Pa_forwardResult(pIhandle->memSizeInfo.pahandle,
                                        (void *)pHd->buffPtr,
                                        &retHandle,
                                        &handleType,
                                        &cmdDest);
            if(netCPRet != pa_OK)
            {
                if(netCPRet  == pa_RESUBMIT_COMMAND)
                {
                    /* PA had requested for resubmit of command.This is
                     * because of receiving burst of L4 port additions and
                     * PA can only buffer one additional request while it is
                     * processing a request
                     */
                    nwal_resubmitMsg(pIhandle,pLocContext,cmdDest,pHd);

                    /* Wait for additional response processing to next
                     * poll cycle
                     */
                    return(nwalRet);

                }else
                {
                    nwalRet = nwal_ERR_PA;
                    state = NWAL_STATE_INACTIVE;
                    pLocContext->extErr = netCPRet;
                }
            }
            else
            {
                state = NWAL_STATE_ACTIVE;

                /* Update stats local to proc */
                pLocContext->numPendPAReq--;
            }

            nwal_procCtlResp(pIhandle,
                             pLocContext,
                             pTransInfo,
                             (pHandleHdr->handleId & NWAL_HANDLE_ID_MASK),
                             pHd,
                             &retHandle,
                             state,
                             nwalRet,
                             pCmdCallBack);


            /* Return the Descriptor back to the appropriate RX
             * linked buffer Q
             */
            nwal_retBuf(pIhandle,nwal_TRUE,nwal_TRUE,pHd);
        }

        if(pRefHandleHdr == NULL)
        {
            /* Non Blocking API call */
            return(nwalRet);
        }

        if((pRefHandleHdr) && (pRefHandleHdr == pHandleHdr))
        {
            /* Internal Blocking API use case */
            return(nwalRet);
        }
    }
 }

 /********************************************************************
 * FUNCTION PURPOSE: API for polling packets or control message from
 *                   network
 ********************************************************************
 * DESCRIPTION: API for polling packets or control message from
 *              network
 ********************************************************************/
void nwal_pollCtl(  nwal_Inst                nwalInst,
                    nwal_CmdCallBack*        pCmdCallBack,
                    nwal_CmdPaStatsReply*    pPaStatsCallBack)
{
    nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                  pCmdCallBack,
                  pPaStatsCallBack,
                  NULL);
}

/********************************************************************
 * FUNCTION PURPOSE: Packet processing from network
 ********************************************************************
 * DESCRIPTION: Packet processing from network
 ********************************************************************/
static inline uint16_t nwal_procPkt(    nwalGlobContext_t*      pIhandle,
                                        Qmss_QueueHnd           rxQ,
                                        uint32_t                appCookie,
                                        uint8_t                 maxPkts,
                                        Qmss_QueueHnd           appRxPktQueue,
                                        nwal_rxPktCallBack*     pRxPktCallBack)
{
    pasahoLongInfo_t*   pinfo = NULL;
    Cppi_HostDesc*      pHd;
    uint16_t            count = 0;
    uint8_t             count2;
    nwalLocContext_t*   pLocContext = NULL;
    nwal_Bool_t         freePkt[NWAL_MAX_RX_PKT_THRESHOLD] = {0};
    nwalRxPktInfo_t     rxPktInfo[NWAL_MAX_RX_PKT_THRESHOLD];
    uint32_t            tmpFlag;
    uint8_t             txQIndex;

#ifndef __ARMv7
    if(!(Qmss_getQueueEntryCount (rxQ)))
    {
        return(0);
    }
#endif

    /* In the case of ARM architecture avoiding reading the count to
     * reduce the  cycle cost of reading QMSS registers
     */
    if(!pRxPktCallBack)
    {
        pLocContext = nwal_getLocContext(pIhandle);
        if(pLocContext == NULL)
        {
            //nwal_debug_bk();
            return(count);
        }
        pRxPktCallBack = pLocContext->cfg.pRxPktCallBack;
    }

    while (count < maxPkts )
    {
        pHd = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop (rxQ));
        if(pHd == NULL) break;

        NWAL_osalInvalidateCache(pHd,NWAL_CACHE_LINE_SIZE);
        NWAL_osalInvalidateCache((void *)pHd->buffPtr,pHd->buffLen);

        rxPktInfo[count].pPkt = Pktlib_getPacketFromDesc(pHd);

        if(nwal_mGetAppidFmPkt(rxPktInfo[count].pPkt,&rxPktInfo[count].appId) != nwal_TRUE)
         {
            /* Error case. Should never reach here. Block for futher
             * debugging */
            //nwal_debug_bk();
            break;
        }
        /* Always set MAC packet to unknown for now. Net CP support would
         * be needed
         */
        rxPktInfo[count].rxFlag1 = NWAL_RX_FLAG1_META_DATA_VALID;
        rxPktInfo[count].startOffset = 0;
        rxPktInfo[count].pktLen = Pktlib_getPacketLen(rxPktInfo[count].pPkt);


        /* Get the extended information*/
        pinfo =  NULL;
        pinfo = nwal_mGetProtoInfo(rxPktInfo[count].pPkt);
        if(pinfo)
        {
            nwal_rxFlag1_t  tmpFlag = 0;

            if((nwal_mIsFragmentPkt(pinfo)) && (pLocContext->pRxReassemProc))
            {
                txQIndex = NSS_PA_QUEUE_OUTER_IP_INDEX;

                /* Check Outer vs Inner IP */
                if(PASAHO_LINFO_READ_IP_COUNT(pinfo) > 1)
                {
                    /* Inner IP Fragmentation */
                     txQIndex = NSS_PA_QUEUE_INNER_IP_INDEX;
                }
                rxPktInfo[count].rxFlag1 |= NWAL_RX_IP_FRAGMENT_PKT;
                pLocContext->pRxReassemProc(rxPktInfo[count].pPkt,
                     QMSS_PASS_QUEUE_BASE + txQIndex);
                /* Move to next Pop */
                continue;
            }

            rxPktInfo[count].l3OffBytes = nwal_mGetL3OffBytes(pinfo);
            rxPktInfo[count].l4OffBytes = nwal_mGetL4Offset(pinfo);
            if(rxPktInfo[count].l4OffBytes)
            {
                rxPktInfo[count].l4ProtoType = nwal_mGetL4ProtoType(pinfo);
            }

            rxPktInfo[count].ploadOffBytes = nwal_mGetPloadOffBytes(pinfo);
            rxPktInfo[count].ploadLen = nwal_mGetPloadLen(pinfo);
            tmpFlag = (nwal_mGetCryptoStatus(pinfo));
            rxPktInfo[count].rxFlag1 |= tmpFlag;


            rxPktInfo[count].pktType = NWAL_MAC_PKT_UNICAST;
            if(nwal_mIsMacBroadcast(pinfo))
            {
                rxPktInfo[count].pktType = NWAL_MAC_PKT_BROADCAST;
            }
            else if(nwal_mIsMacMulticast(pinfo))
            {
                rxPktInfo[count].pktType = NWAL_MAC_PKT_MULTICAST;
            }

            if(nwal_mIsFragmentPkt(pinfo))
            {
                rxPktInfo[count].rxFlag1 |= NWAL_RX_IP_FRAGMENT_PKT;
            }
            rxPktInfo[count].enetPort = nwal_mGetRxEmacPort(pinfo);
        }
        else
        {
             rxPktInfo[count].pktType = NWAL_MAC_PKT_UNKNOWN;
             rxPktInfo[count].enetPort = NWAL_ENET_PORT_UNKNOWN;
        }

        tmpFlag = 0x0;
        if(rxPktInfo[count].l3OffBytes)
        {
            if(nwal_mIsL3CksumStatusPass(rxPktInfo[count].pPkt))
            {
                tmpFlag = NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_ACK;
            }
            else
            {
                tmpFlag = NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_NACK;
            }
        }
        else
        {
            tmpFlag = NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_UNKNOWN;
        }

        rxPktInfo[count].rxFlag1  |=
            ((tmpFlag << NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_SHIFT) &
              NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_MASK);

        if(rxPktInfo[count].ploadOffBytes)
        {
            if(nwal_mIsL4CksumStatusPass(rxPktInfo[count].pPkt))
            {

                tmpFlag = NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_ACK;
            }
            else
            {
                tmpFlag = NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_NACK;
            }
        }
        else
        {
            tmpFlag = NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_UNKNOWN;
        }
        rxPktInfo[count].rxFlag1  |=
             ((tmpFlag << NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_SHIFT) &
               NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_MASK);
        freePkt[count] = nwal_FALSE;
        count++;
    }

    if(count && pRxPktCallBack)
    {
        /* count is already incremented to reflect 1 based */
        if (pRxPktCallBack)
        {
            pRxPktCallBack(appCookie,
                           count,
                           rxPktInfo,
                           NWAL_getTimeStamp(),
                        freePkt);
        }
        for(count2 = 0;count2 < count;count2++)
        {
            if(freePkt[count2] == nwal_TRUE)
            {
                /* Free the packet  */
                Pktlib_freePacket(rxPktInfo[count2].pPkt);
            }
        }
    }
    return(count);

}
/********************************************************************
 * FUNCTION PURPOSE: API for polling packets from network
 ********************************************************************
 * DESCRIPTION: API for polling packets from network
 ********************************************************************/
uint16_t nwal_pollPkt(nwal_Inst             nwalInst,
                      nwal_pollPktQCtl      pktQCtl,
                      uint32_t              appCookie,
                      uint16_t              maxPkts,
                      Qmss_QueueHnd         appRxPktQueue,
                      nwal_rxPktCallBack*   pRxPktCallBack)
{
    nwalLocContext_t*   pLocContext;
    uint16_t            pktCount= 0;
    uint16_t            count;

    if((pktQCtl & nwal_POLL_DEFAULT_PER_PROC_PKT_Q) ==
        nwal_POLL_DEFAULT_PER_PROC_PKT_Q)
    {
        pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
        if(pLocContext == NULL)
        {
            nwal_debug_bk();
            return(pktCount);
        }
        count = 
        nwal_procPkt(nwalProcessCtx.pSharedMemBase,
                         pLocContext->rxL4PktQ,
                         appCookie,
                         maxPkts,
                         appRxPktQueue,
                         pRxPktCallBack);
        pktCount += count;
    }

    if(((pktQCtl & nwal_POLL_APP_MANAGED_PKT_Q) ==
         nwal_POLL_APP_MANAGED_PKT_Q) && (pktCount < maxPkts))
    {
        count =
            nwal_procPkt(nwalProcessCtx.pSharedMemBase,
                         appRxPktQueue,
                         appCookie,
                         maxPkts,
                         appRxPktQueue,
                         pRxPktCallBack);
        pktCount += count;
    }

    if(((pktQCtl & nwal_POLL_DEFAULT_GLOB_PKT_Q) ==
         nwal_POLL_DEFAULT_GLOB_PKT_Q)  && (pktCount < maxPkts))
    {
        if(nwalProcessCtx.pSharedMemBase->cfg.rxDefPktQ)
        {
             count =
                 nwal_procPkt(nwalProcessCtx.pSharedMemBase,
                              nwalProcessCtx.pSharedMemBase->cfg.rxDefPktQ,
                              appCookie,
                              maxPkts,
                              appRxPktQueue,
                              pRxPktCallBack);
             pktCount += count;
        }
    }
    return(pktCount);
}

/********************************************************************
 * FUNCTION PURPOSE: API to transmit raw packet with prefilled header
 *                   to network.
 ********************************************************************
 * DESCRIPTION: API to transmit raw packet with prefilled header
 *              to network.
 ********************************************************************/
nwal_RetValue nwal_sendRaw (nwal_Inst         nwalInst,
                            nwal_Bool_t       lpbackPass,
                            uint16_t          bufLen,
                            uint8_t*          pBuf)
{
    Cppi_HostDesc*      pHd;
    nwalBufPool_t*      pLinkBufTxQ;
    char                psFlags = 0x0;
    nwalLocContext_t *  pLocContext;
    Ti_Pkt*             pPkt;
    uint8_t*            pDataBuf;
    uint32_t            dataLen;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    pLinkBufTxQ = nwal_getLinkedBufQ(pLocContext->cfg.txPktPool.bufPool,
                                     bufLen,
                                     pLocContext->cfg.txPktPool.numBufPools);
    if(pLinkBufTxQ ==NULL)
        return (nwal_ERR_NO_FREE_BUF);

    /* Allocate a packet from the heap. Return Queue is already set to
     * default free queue by packet Lib
     */
    pPkt = Pktlib_allocPacket(pLinkBufTxQ->heapHandle,bufLen);
    if (pPkt == NULL)
        return (nwal_ERR_NO_FREE_CMD_DESC);

    pHd = Pktlib_getDescFromPacket(pPkt);

    /* Make sure there is no control info.  */
      Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)pHd, 0);


    /* Memory copy the buffer from application.
     */
    Pktlib_getDataBuffer(pPkt,&pDataBuf,&dataLen);
    if(dataLen < bufLen)
    {
        return (nwal_ERR_PKT_LIB);
    }
    memcpy(pDataBuf,pBuf,bufLen);
    /* Write back the cache */
    NWAL_osalWriteBackCache((void *)pDataBuf,bufLen);
    Pktlib_setDataBufferLen(pPkt,bufLen);

    /* TX to send packet out. EMAC switch will redirect to appropriate
     * port based on ALE
     */
    Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)pHd, psFlags);
#ifndef __ARMv7
    Pktlib_writebackPkt(pPkt);
#endif

    if(lpbackPass == nwal_TRUE)
    {
        /* Send the packet out to transmit Q*/
        Qmss_queuePushDescSize(nwalProcessCtx.pSharedMemBase->txQ[NSS_PA_QUEUE_INPUT_INDEX],
                                pHd,
                                NWAL_DESC_SIZE);
    }
    else
    {
        Qmss_queuePushDescSize(nwalProcessCtx.pSharedMemBase->txQ[NSS_CPSW_QUEUE_ETH_INDEX],
                                pHd,
                                NWAL_DESC_SIZE);


        
    }

    return nwal_OK;

}

/********************************************************************
 * FUNCTION PURPOSE: Prepare the Protocol Specific command label 
 ********************************************************************
 * DESCRIPTION: Prepare the Protocol Specific command label
 ********************************************************************/
static inline nwal_RetValue
           nwal_prepPSCmdLabel( nwalGlobContext_t*        pIhandle,
                                uint32_t                  saSwInfo0,
                                uint32_t                  saSwInfo1,
                                nwalTxPktInfo_t*          pPktInfo,
                                nwalTxPSCmdInfo_t*        pTxPSCmdInfo)
{
    uint16_t                cmdOffset = 0;
    uint16_t                cmdBlockSize = 0;
    pasahoComChkCrc_t*      ptx;
    pasahoNextRoute_t*      nr;
    pasahoCmdInfo_t*        paCmdInfo;
    uint16_t                 txQ;
    uint16_t                alignDetect = nwal_FALSE;
    uint8_t*                pPsDataBuf;
    uint16_t                ipSecAhMode = nwal_FALSE;
    pasahoPatchMsgLen_t*    pPatchMsgLen;
    pasahoIpFrag_t*         pIpFragInfo;
    nwalLocContext_t*       pLocContext;


    if(pTxPSCmdInfo == NULL)
    {
        return(nwal_ERR_INVALID_PARAM);
    }
    pLocContext = nwal_getLocContext(pIhandle);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    pPsDataBuf = (uint8_t *)pTxPSCmdInfo->psCmdLabel;

    if(pPktInfo->lpbackPass == nwal_TRUE)
    {
        /* Default TX Queue Index */
        txQ= QMSS_PASS_QUEUE_BASE +
            NSS_PA_QUEUE_INPUT_INDEX;
    }
    else
    {
        txQ= QMSS_PASS_QUEUE_BASE +
             NSS_CPSW_QUEUE_ETH_INDEX;
    }

    /* Initialize command Info */
    memset(pTxPSCmdInfo,0,sizeof(nwalTxPSCmdInfo_t));

    if((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_AH_CRYPTO) ==
        NWAL_TX_FLAG1_DO_IPSEC_AH_CRYPTO)
     {
        ipSecAhMode = nwal_TRUE;
     }

    if((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPV4_CHKSUM) ==
        NWAL_TX_FLAG1_DO_IPV4_CHKSUM)
    {
        /* Command Prep for IP Header checksum update */
        ptx = (pasahoComChkCrc_t *)&pPsDataBuf[cmdOffset];
        PASAHO_SET_CMDID(ptx, PASAHO_PAMOD_CMPT_CHKSUM);
        PASAHO_CHKCRC_SET_START      (ptx, pPktInfo->ipOffBytes);
        PASAHO_CHKCRC_SET_LEN        (ptx, NWAL_IPV4_HDR_LEN_BYTES);
        PASAHO_CHKCRC_SET_RESULT_OFF (ptx, NWAL_IPV4_OFFSET_HDR_CHKSUM);
        PASAHO_CHKCRC_SET_INITVAL    (ptx, 0);
        PASAHO_CHKCRC_SET_NEG0 (ptx, 1);
        cmdOffset += sizeof (pasahoComChkCrc_t);
    }

    if(((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_UDP_CHKSUM) ==
        NWAL_TX_FLAG1_DO_UDP_CHKSUM) ||
       ((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_TCP_CHKSUM) ==
       NWAL_TX_FLAG1_DO_TCP_CHKSUM))
    {
        /* Command Prep for UDP checksum update */
        ptx = (pasahoComChkCrc_t *)&pPsDataBuf[cmdOffset];
        PASAHO_SET_CMDID(ptx, PASAHO_PAMOD_CMPT_CHKSUM);
        PASAHO_CHKCRC_SET_START      (ptx, pPktInfo->l4OffBytes);
        PASAHO_CHKCRC_SET_LEN (ptx,
                               (pPktInfo->l4HdrLen + pPktInfo->ploadLen));

        /* Offset to checksum location RELATIVE TO THE START OF
         * THE L4 HEADER
         */
        if((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_UDP_CHKSUM) ==
            NWAL_TX_FLAG1_DO_UDP_CHKSUM)
        {
            PASAHO_CHKCRC_SET_RESULT_OFF (ptx, NWAL_UDP_OFFSET_CHKSUM);
        }
        else if ((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_TCP_CHKSUM) ==
                  NWAL_TX_FLAG1_DO_TCP_CHKSUM)
        {
            PASAHO_CHKCRC_SET_RESULT_OFF (ptx, NWAL_TX_FLAG1_DO_TCP_CHKSUM);
        }

        PASAHO_CHKCRC_SET_INITVAL (ptx, pPktInfo->pseudoHdrChecksum);
        PASAHO_CHKCRC_SET_NEG0 (ptx, 1);

        pTxPSCmdInfo->udpChkOff = cmdOffset;
        cmdOffset += sizeof (pasahoComChkCrc_t);
    }

    if(cmdOffset)
    {
        /* Set TX Queue to PDSP5 for checksum computation */
        /* Set TX Queue to PDSP6 for checksum computation, NETCP 1.5 */
        txQ = QMSS_PASS_QUEUE_BASE +
              NSS_PA_QUEUE_TXCMD_INDEX;
    }
    if(pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_CRYPTO_MASK)
    {
        /* Update Crypto command in PS Data */
        pasahoShortInfo_t *sInfo;
        if(cmdOffset)
        {
            /* Set next route so that packet gets transferred to SA */
            nr = (pasahoNextRoute_t *)&pPsDataBuf[cmdOffset];
            nr->word0 = 0;
            nr->word1 = 0;
            nr->swInfo0 = saSwInfo0;
            nr->swInfo1 = saSwInfo1;
            PASAHO_SET_CMDID (nr, PASAHO_PAMOD_NROUTE);
            PASAHO_SET_DEST  (nr, PASAHO_NR_DEST_PKTDMA);
            PASAHO_SET_FLOW  (nr, pIhandle->rxPaSaFlowId);
#ifdef NWAL_NETCP_TX_PKT_DEBUG1
            PASAHO_SET_QUEUE (nr, pIhandle->cfg.rxDefPktQ);
#else
            PASAHO_SET_QUEUE (nr,
                          (QMSS_PASS_QUEUE_BASE +
                           NSS_SA_QUEUE_SASS_INDEX));
#endif
            /* TBD No multiroute settings */
            pTxPSCmdInfo->pa2saNextRoute = cmdOffset;

            cmdOffset += sizeof(pasahoNextRoute_t) - sizeof(nr->word1);
        }
        else
        {
            /* Send the packet directly to SA for Encryption */
            txQ = QMSS_PASS_QUEUE_BASE + NSS_SA_QUEUE_SASS_INDEX;
        }

        sInfo = (pasahoShortInfo_t *)&pPsDataBuf[cmdOffset];
        sInfo->word0 = 0;
        sInfo->word1 = 0; /* supData is init to 0 */
        PASAHO_SET_CMDID (sInfo, PASAHO_SA_SHORT_INFO);
        PASAHO_SINFO_SET_PAYLOAD_OFFSET(sInfo, pPktInfo->saOffBytes);
        PASAHO_SINFO_SET_PAYLOAD_LENGTH(sInfo, pPktInfo->saPayloadLen);

        pTxPSCmdInfo->saShortInfoOff = cmdOffset;

        cmdOffset += sizeof(pasahoShortInfo_t);

        /* Make sure the PS command size once received at SA is multiple of
         * 8 bytes because of NetCP /PKTDMA requirement
         */
        cmdBlockSize  = sizeof(pasahoShortInfo_t);
        alignDetect = nwal_TRUE;

    }

     if(alignDetect)
     {
         cmdBlockSize += sizeof(pasahoNextRoute_t) - sizeof(nr->word1);
         if(pPktInfo->lpbackPass == nwal_FALSE)
         {
            cmdBlockSize += sizeof(nr->word1);
         }
         if(ipSecAhMode == nwal_TRUE)
         {
            /* IP Sec AH mode. Include patch comman block in the same
             * block
             */
            cmdBlockSize += 4;
         }
         /* Make sure next command at SA starts at 8 byte boundary */
         if(cmdBlockSize % 8)
         {
             /* Add a Dummy word for alignment */
            paCmdInfo = (pasahoCmdInfo_t *)&pPsDataBuf[cmdOffset];
            paCmdInfo->word0 = 0;
            PASAHO_SET_CMDID (paCmdInfo, PASAHO_PAMOD_DUMMY);
            cmdOffset += sizeof(uint32_t);
            cmdBlockSize += sizeof(uint32_t);
         }
         cmdBlockSize = 0;
     }

    if((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_UPDATE_ETHER_LEN) ==
        NWAL_TX_FLAG1_DO_UPDATE_ETHER_LEN)
    {
        if(!cmdOffset)
        {
            /* Set TX Queue to PDSP5 for checksum computation, NETCP 1.0 */
            /* Set TX Queue to PDSP6 for checksum computation, NETCP 1.5 */
            txQ = QMSS_PASS_QUEUE_BASE +
                  NSS_PA_QUEUE_TXCMD_INDEX;
        }
        pPatchMsgLen = (pasahoPatchMsgLen_t*)&pPsDataBuf[cmdOffset];
        PASAHO_SET_CMDID (pPatchMsgLen, PASAHO_PAMOD_PATCH_MSG_LEN);
        PASAHO_SET_SUB_CODE_PATCH_MSG_LEN(pPatchMsgLen);
        PASAHO_SET_MSGLEN_SIZE(pPatchMsgLen, 0);
        PASAHO_SET_MSGLEN_OFFSET(pPatchMsgLen, pPktInfo->etherLenOffBytes);
        PASAHO_SET_MSGLEN(pPatchMsgLen, 2);
        cmdOffset += sizeof(pasahoPatchMsgLen_t);
    }

    if(pPktInfo->mtuSize)
    {
        if(!cmdOffset)
        {
        /* Set TX Queue to PDSP5 for fragmentation , NETCP 1.0 */
        /* Set TX Queue to PDSP6 for fragmentation , NETCP 1.5 */
            txQ = QMSS_PASS_QUEUE_BASE +
                  NSS_PA_QUEUE_TXCMD_INDEX;
        }
        else
        {
            if(!(pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_CRYPTO_MASK))
            {
                /* There is a command preceding IP fragmentation
                 * Resend the command to PDSP5
                 */
                nr = (pasahoNextRoute_t *)&pPsDataBuf[cmdOffset];
                nr->word0 = 0;
                nr->word1 = 0;
                nr->swInfo0 = 0;
                nr->swInfo1 = 0;
                PASAHO_SET_CMDID (nr, PASAHO_PAMOD_NROUTE);
                PASAHO_SET_DEST  (nr, PASAHO_NR_DEST_PKTDMA);
                PASAHO_SET_FLOW  (nr, (uint8_t)pLocContext->rxPktFlowId);
                PASAHO_SET_QUEUE (nr,
                                  (QMSS_PASS_QUEUE_BASE +
                                   NSS_PA_QUEUE_TXCMD_INDEX));
                cmdOffset += sizeof(pasahoNextRoute_t) - sizeof(nr->word1);
                cmdBlockSize += sizeof(pasahoNextRoute_t) - sizeof(nr->word1);
            }
       }
        /* Update command for fragmentation */
        pIpFragInfo = (pasahoIpFrag_t* )&pPsDataBuf[cmdOffset];
        PASAHO_SET_CMDID (pIpFragInfo, PASAHO_PAMOD_IP_FRAGMENT);
        PASAHO_SET_SUB_CODE_IP_FRAG(pIpFragInfo);
        PASAHO_SET_IP_OFFSET(pIpFragInfo, (uint8_t)pPktInfo->ipOffBytes);
        PASAHO_SET_MTU_SIZE(pIpFragInfo, pPktInfo->mtuSize);
        cmdOffset += sizeof(pasahoIpFrag_t);
    }

    if((cmdOffset) ||
       (pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_CRYPTO_MASK))
    {
        /* In the case of IPSEC packet from SA will always be redirected
         * to PDSP-5
         * Add the next route to handle further redirection of packet
         */

        /* At a minimum there is one command. Set the final route details
         * for packet to be sent out
         */
        /* Set next route so that packet gets transferred to SA */

        nr = (pasahoNextRoute_t *)&pPsDataBuf[cmdOffset];
        nr->word0 = 0;
        nr->word1 = 0;
        nr->swInfo0 = 0;
        nr->swInfo1 = 0;


        PASAHO_SET_CMDID (nr, PASAHO_PAMOD_NROUTE);

        if(pPktInfo->lpbackPass == nwal_TRUE)
        {
             PASAHO_SET_DEST  (nr, PASAHO_NR_DEST_PKTDMA);
             PASAHO_SET_FLOW  (nr, (uint8_t)pLocContext->rxPktFlowId);

             /* Enable command for PDSP firmware to insert padding bytes (0)
              * if size is less than 60 bytes.
              * This action will be done by default if packets are sent through
              * PASS
              */
             PASAHO_SET_TX_PADDING (nr, 1);

#ifdef NWAL_NETCP_TX_PKT_DEBUG
            PASAHO_SET_QUEUE (nr, pIhandle->cfg.rxDefPktQ);
#else
            PASAHO_SET_QUEUE (nr,
                              QMSS_PASS_QUEUE_BASE +
                              NSS_PA_QUEUE_INPUT_INDEX);
#endif
        }
        else
        {
             uint8_t psFlags =0;
             if(pPktInfo->enetPort)
             {
                 PASAHO_SET_E (nr, 1);
                 psFlags =
                     ((pPktInfo->enetPort & pa_EMAC_CTRL_PORT_MASK) <<
                       NWAL_PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
                 PASAHO_SET_PKTTYPE  (nr, psFlags);
             }

             PASAHO_SET_DEST  (nr, PASAHO_NR_DEST_ETH);

             /* Enable command for PDSP firmware to insert padding bytes (0)
              * if size is less than 60 bytes
              * This action will be done by default if packets are sent through
              * PASS
              */
             PASAHO_SET_TX_PADDING (nr, 1);
        }

         pTxPSCmdInfo->enetPortNextRoute = cmdOffset;

         if(ipSecAhMode == nwal_TRUE)
         {
            /* Indicate in PS command to process the next patch command before
             * processing the next route
             */
            PASAHO_SET_N  (nr, 1);
         }

         /* Update command offset for the length */
         cmdOffset += sizeof(pasahoNextRoute_t) - sizeof(nr->word1);
         cmdBlockSize += sizeof(pasahoNextRoute_t) - sizeof(nr->word1);
         if(pPktInfo->lpbackPass == nwal_FALSE)
         {
            cmdOffset += sizeof(nr->word1);
            cmdBlockSize += sizeof(nr->word1);
         }

         if(ipSecAhMode == nwal_TRUE)
         {
             pasahoComBlindPatch_t *bpCmd;

             bpCmd = (pasahoComBlindPatch_t *) &pPsDataBuf[cmdOffset];
             if(pPktInfo->saAhMacSize > NWAL_IPSEC_AH_MAX_AUTH_TAG_BYTES)
             {
                return(nwal_ERR_INVALID_PARAM);
             }

             PASAHO_SET_CMDID(bpCmd, PASAHO_PAMOD_PATCH);
             PASAHO_BPATCH_SET_PATCH_NBYTES  (bpCmd, pPktInfo->saAhMacSize);
             PASAHO_BPATCH_SET_PATCH_CMDSIZE (bpCmd,
                                             (NWAL_IPSEC_AH_MAX_AUTH_TAG_BYTES +
                                              4) >> 2);

             /* Use Patch replace option */
             PASAHO_BPATCH_SET_OVERWRITE     (bpCmd,1);
             PASAHO_BPATCH_SET_OFFSET        (bpCmd, pPktInfo->saAhIcvOffBytes);

             pTxPSCmdInfo->ahPathCmdOff = cmdOffset;
             cmdOffset += 4;
         }
    }

    pTxPSCmdInfo->txQueue = txQ;
    pTxPSCmdInfo->psCmdLabelLen = cmdOffset;
    if(pTxPSCmdInfo->psCmdLabelLen > NWAL_MAX_PS_COMMAND_SIZE)
    {
        return nwal_ERR_RES_UNAVAILABLE;
    }

#ifdef NWAL_LIB_ENABLE_PROFILE
   // nwal_prof5 = nwal_read_clock();
#endif

    return nwal_OK;
}


#if 0
/* TBD: Will be enabled once PA format TX checksum command can provide
 * command offset details
 */

#define NWAL_MAX_PS_CMD_SIZE (2 *sizeof(pasahoNextRoute_t)) + \
                              (2 * sizeof(pasahoComChkCrc_t))+ \
                               sizeof(paPayloadInfo_t) + \
                               (2* sizeof(paCmdInfo_t))
#define NWAL_MAX_NUM_CMDS   7

/********************************************************************
 * FUNCTION PURPOSE: Update PS Info for the packet being transmitted
 ********************************************************************
 * DESCRIPTION: Update PS Info for the packet being transmitted
 ********************************************************************/
static inline nwal_RetValue nwal_updatePSInfo(
                                nwalGlobContext_t*        pIhandle,
                                uint32_t*                 pPsDataBuf,
                                uint32_t                  saSwInfo0,
                                uint32_t                  saSwInfo1,
                                nwalTxPktInfo_t*          pPktInfo,
                                uint32_t*                 pCmdSize,
                                Qmss_QueueHnd*            pTxQHnd,
                                nwalTxPSCmdInfo_t*        pTxPSCmdInfo)


{
    uint16_t            cmdSize = NWAL_MAX_PS_CMD_SIZE;
    paReturn_t          netCPRet;
    uint8_t             cmdIdx = 0;
    paCmdInfo_t         cmdInfo[NWAL_MAX_NUM_CMDS];
    uint8_t             txQIndex;


    if(pPktInfo->lpbackPass == nwal_TRUE)
    {
        /* Default TX Queue Index */
        txQIndex = NWAL_TX_QUEUE_INDEX_PDSP0;
    }
    else
    {
        txQIndex = NWAL_TX_QUEUE_INDEX_ENET;
    }

    if((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPV4_CHKSUM) ==
        NWAL_TX_FLAG1_DO_IPV4_CHKSUM)
    {
        cmdInfo[cmdIdx].cmd = pa_CMD_TX_CHECKSUM;
        cmdInfo[cmdIdx].params.chksum.startOffset = pPktInfo->ipOffBytes;

        /* Length of the data for checksum computation */
        cmdInfo[cmdIdx].params.chksum.lengthBytes =
            NWAL_IPV4_HDR_LEN_BYTES;
        cmdInfo[cmdIdx].params.chksum.resultOffset =
            NWAL_IPV4_OFFSET_HDR_CHKSUM;
        cmdInfo[cmdIdx].params.chksum.initialSum = 0;
        cmdInfo[cmdIdx].params.chksum.negative0 = 1;
        cmdIdx++;
    }



    if(((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_UDP_CHKSUM) ==
        NWAL_TX_FLAG1_DO_UDP_CHKSUM) ||
       ((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_TCP_CHKSUM) ==
       NWAL_TX_FLAG1_DO_TCP_CHKSUM))
    {
        /* UDP checksum update */
        /* Start Offset for the checksum computation */
        cmdInfo[cmdIdx].cmd = pa_CMD_TX_CHECKSUM;
        cmdInfo[cmdIdx].params.chksum.startOffset = pPktInfo->l4OffBytes;

        /* Length of the data for checksum computation */
        cmdInfo[cmdIdx].params.chksum.lengthBytes = (pPktInfo->l4HdrLen +
                                                     pPktInfo->ploadLen);

        /* Offset to checksum location RELATIVE TO THE START OF
         * THE L4 HEADER
         */
        if((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_UDP_CHKSUM) ==
            NWAL_TX_FLAG1_DO_UDP_CHKSUM)
        {
            cmdInfo[cmdIdx].params.chksum.resultOffset =
                NWAL_UDP_OFFSET_CHKSUM;
        }else if((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_TCP_CHKSUM) ==
                  NWAL_TX_FLAG1_DO_TCP_CHKSUM)
        {
            cmdInfo[cmdIdx].params.chksum.resultOffset =
                NWAL_TCP_OFFSET_CHKSUM;
        }

        cmdInfo[cmdIdx].params.chksum.initialSum =
            pPktInfo->pseudoHdrChecksum;

        /* TRUE: Computed value of 0 written as -0 for both TCP and UDP*/
        cmdInfo[cmdIdx].params.chksum.negative0 = 1;

        cmdIdx++;
    }

    if(cmdIdx)
    {
        /* Set TX Queue to PDSP5 for checksum computation, NETCP 1.0 */
        /* Set TX Queue to PDSP6 for checksum computation, NETCP 1.5 */
        txQIndex = NWAL_TX_QUEUE_INDEX_TXCMD;
    }

    if(pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_CRYPTO_MASK)
    {
        if(cmdIdx)
        {
            cmdInfo[cmdIdx].cmd = pa_CMD_NEXT_ROUTE;
            /* Set next route so that packet gets transferred to SA */
            cmdInfo[cmdIdx].params.route.ctrlBitfield =
                pa_NEXT_ROUTE_PARAM_PRESENT;
            cmdInfo[cmdIdx].params.route.dest = pa_DEST_SASS;
            cmdInfo[cmdIdx].params.route.flowId = pIhandle->rxPaSaFlowId;
            cmdInfo[cmdIdx].params.route.queue =
                pIhandle->txQMap[NWAL_TX_QUEUE_INDEX_SASS];
            cmdInfo[cmdIdx].params.route.swInfo0 = saSwInfo0;
            cmdInfo[cmdIdx].params.route.swInfo1 = saSwInfo1;
            cmdInfo[cmdIdx].params.route.multiRouteIndex = 0;
            cmdIdx++;
        }
        else
        {
            /* Send the packet directly to SA for Encryption */
            txQIndex = NWAL_TX_QUEUE_INDEX_SASS;
        }

        cmdInfo[cmdIdx].cmd = pa_CMD_SA_PAYLOAD;
        cmdInfo[cmdIdx].params.payload.offset = pPktInfo->saOffBytes;
        cmdInfo[cmdIdx].params.payload.len = pPktInfo->saPayloadLen;
        cmdInfo[cmdIdx].params.payload.supData = 0;
        cmdIdx++;

#ifdef NWAL_PA_PS_CMD_WROUND
        /* Insert a blank command for aligning to 8 byte boundary.*/
        cmdInfo[cmdIdx].cmd = pa_CMD_NONE;
        cmdIdx++;
#endif
    }

    if((cmdIdx) ||
       (pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_CRYPTO_MASK))
    {
        /* In the case of IPSEC packet from SA will always be redirected to
         * PDSP-5 Add the next route to handle further redirection of packet
         */

        /* At a minimum there is one command. Set the final route details
         * for packet to be sent out
         */
        cmdInfo[cmdIdx].cmd = pa_CMD_NEXT_ROUTE;

        if(pPktInfo->lpbackPass == nwal_TRUE)
        {
            nwalLocContext_t*   pLocContext;
            pLocContext = nwal_getLocContext(pIhandle);
            if(pLocContext == NULL)
            {
                return(nwal_ERR_INVALID_PROC_ID);
            }

            /* Loopback to PA after checksum offload */
            cmdInfo[cmdIdx].params.route.ctrlBitfield =
                pa_NEXT_ROUTE_PARAM_PRESENT;
            cmdInfo[cmdIdx].params.route.dest = pa_DEST_HOST;
            cmdInfo[cmdIdx].params.route.flowId =
                (uint8_t)pLocContext->rxPktFlowId;
            cmdInfo[cmdIdx].params.route.queue =
                pIhandle->txQ[NWAL_TX_QUEUE_INDEX_PDSP0];
            cmdInfo[cmdIdx].params.route.multiRouteIndex = 0;
        }
        else
        {
            cmdInfo[cmdIdx].params.route.dest = pa_DEST_EMAC;
            cmdInfo[cmdIdx].params.route.pktType_emacCtrl =
                pPktInfo->enetPort;
        }

        cmdIdx++;
        netCPRet = Pa_formatTxCmd (cmdIdx,   /* nCmd */
                                   cmdInfo,  /* command info */
                                   0,        /* offset */
                                   (Ptr)pPsDataBuf, /* Command buffer   */
                                   &cmdSize);    /* Command size        */
        if(netCPRet != pa_OK)
        {
            return(nwal_ERR_PA);
        }
    }
    *pTxQHnd = pIhandle->txQ[txQIndex];
    return nwal_OK;
}

#endif

/* Enable Software checksum inside NWAL module instead of NetCP offload */
//#define NWAL_ENABLE_SW_CHECKSUM
/********************************************************************
 * FUNCTION PURPOSE: API to update protocol header to the packet
 ********************************************************************
 * DESCRIPTION: API to update protocol header to the packet
 ********************************************************************/
nwal_RetValue nwal_updateProtoHdr  (nwal_Inst         nwalInst,
                                    nwal_Handle       nwalHandle,
                                    nwalTxPktInfo_t*  pPktInfo)
{
    nwalBufPool_t*      pLinkBufTxQ;
    nwalPortInfo_t*     pPortInfo;
    uint8_t             hdrLen;
    uint8_t             ipHdrLen = 0;
#ifdef NWAL_ENABLE_SA
    uint8_t             macHdrLen = 0;
#endif
    uint8_t*            pIpheader;
    uint8_t*            pUdpHeader;
    uint8_t*            pOrigBuffPtr;
    uint32_t            origBuffLen;
    Ti_Pkt*             pHdrPkt;
    Cppi_HostDesc*      pHdrDesc;
    Ti_Pkt*             pPloadPkt = pPktInfo->pPkt;
    uint8_t*            pPloadBuffPtr;
    uint32_t            ploadBuffLen;
    Ti_Pkt*             pPkt= pPktInfo->pPkt; /* By default.
                                               * Will be updated in the case
                                               * of  header insertion
                                               */
    nwalLocContext_t*   pLocContext = NULL;
    nwalL2L3HdrInfo_t*  pL2L3HdrInfoAddr = NULL;

    nwal_Handle nwalHandleAddr = 
                    (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                nwalHandle));
    if((((nwalHandleHdr_t *)(nwalHandleAddr))->handleId & NWAL_HANDLE_ID_MASK) !=
        NWAL_HANDLE_ID_INST_PORT)
    {
        return (nwal_ERR_INVALID_HANDLE);
    }
    pPortInfo = (nwalPortInfo_t* )nwalHandleAddr;

    if(!pPortInfo->isRemActive)
    {
        return (nwal_ERR_INVALID_STATE);
    }

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }
    pL2L3HdrInfoAddr = 
        (nwalL2L3HdrInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase, 
                                                   pPortInfo->pL2L3HdrInfo));
    hdrLen = pL2L3HdrInfoAddr->hdrLen + NWAL_UDP_HDR_LEN_BYTES;
    pLinkBufTxQ = nwal_getLinkedBufQ(pLocContext->cfg.txPktPool.bufPool,
                                     hdrLen,
                                     pLocContext->cfg.txPktPool.numBufPools);
    if(pLinkBufTxQ ==NULL)
        return (nwal_ERR_NO_FREE_BUF);

    Pktlib_getDataBuffer(pPloadPkt,&pPloadBuffPtr,&ploadBuffLen);

    pHdrPkt = Pktlib_allocPacket(pLinkBufTxQ->heapHandle,hdrLen);
    if (pHdrPkt == NULL)
        return (nwal_ERR_NO_FREE_CMD_DESC);

    pHdrDesc = Pktlib_getDescFromPacket(pHdrPkt);

    /* Setup the return policy to return all segments in the descriptor */
    Cppi_setReturnPolicy (Cppi_DescType_HOST,
                          (Cppi_Desc *)pHdrDesc,
                          Cppi_ReturnPolicy_RETURN_BUFFER);

    /* Reset the buffer pointer to orig in the case of packets w/o IPSec.
     * Update the length to actual header
     */
    Cppi_getOriginalBufInfo(Cppi_DescType_HOST,
                            (Cppi_Desc *)pHdrDesc,
                            &pOrigBuffPtr,
                            &origBuffLen);
    if(origBuffLen < hdrLen)
    {
        return (nwal_ERR_PKT_LIB);
    }
    if(pL2L3HdrInfoAddr->outHandle)
    {
        /* Move the buffer pointer to take care of additional offset needed
         * by SA LLD
         */
        pOrigBuffPtr = pOrigBuffPtr + NWAL_SA_PKT_HDR_MARGIN;
    }

    /* Memory copy the static header */
    memcpy((uint8_t *)pOrigBuffPtr,
           (pL2L3HdrInfoAddr->hdrBuf),
            pL2L3HdrInfoAddr->hdrLen);

    pUdpHeader = (uint8_t *)pOrigBuffPtr + pL2L3HdrInfoAddr->hdrLen;
    memcpy((uint8_t *)pUdpHeader,pPortInfo->hdrBuf,NWAL_UDP_HDR_LEN_BYTES);
#ifdef NWAL_ENABLE_SA
    if(pL2L3HdrInfoAddr->outerIpOffset)
    {
        macHdrLen = pL2L3HdrInfoAddr->outerIpOffset;
    }
    else
    {
        macHdrLen = pL2L3HdrInfoAddr->ipOffset;
    }
#endif
    if(pL2L3HdrInfoAddr->outerIpOffset)
    {
        pIpheader = pOrigBuffPtr + pL2L3HdrInfoAddr->outerIpOffset;
        if(pL2L3HdrInfoAddr->outerIpVer == nwal_IPV4)
        {
            /* Update the IP ID */
            nwal_updateIpIDLen(pIpheader,
                               &pL2L3HdrInfoAddr->ipId,
                               nwal_FALSE,
                               ploadBuffLen);
            /* Checksum will be updated by the SA LLD */
            ipHdrLen = NWAL_IPV4_HDR_LEN_BYTES;
        }
        else
        {
            /* Update the payload length */
            nwal_updateIpv6PloadLen(pIpheader,
                                    (ploadBuffLen + NWAL_UDP_HDR_LEN_BYTES +
                                    (pL2L3HdrInfoAddr->hdrLen -
                                     pL2L3HdrInfoAddr->ipOffset)));
            ipHdrLen = NWAL_IPV6_HDR_LEN_BYTES;
        }
    }

    pIpheader = (uint8_t *)pOrigBuffPtr + pL2L3HdrInfoAddr->ipOffset;
        pPktInfo->pseudoHdrChecksum =
            nwal_utilGetIpPsudoChkSum(pIpheader,
                                      (ploadBuffLen + NWAL_UDP_HDR_LEN_BYTES));
        pPktInfo->ipOffBytes = pL2L3HdrInfoAddr->ipOffset;

    if(pL2L3HdrInfoAddr->ipVer == nwal_IPV4)
    {
        /* Update the IP ID */
        nwal_updateIpIDLen(pIpheader,
                           &pL2L3HdrInfoAddr->ipId,
                           nwal_TRUE,
                           ploadBuffLen);
        if(!ipHdrLen)
        {
            ipHdrLen = NWAL_IPV4_HDR_LEN_BYTES;
        }
#ifndef NWAL_ENABLE_SW_CHECKSUM
        pPktInfo->txFlag1 |= NWAL_TX_FLAG1_DO_IPV4_CHKSUM;
#else
        nwal_utilCompIpChksums(pIpheader);
#endif
    }
    else
    {
        /* Update the payload length */
        nwal_updateIpv6PloadLen(pIpheader,
                               (ploadBuffLen + NWAL_UDP_HDR_LEN_BYTES));
        if(!ipHdrLen)
        {
            ipHdrLen = NWAL_IPV6_HDR_LEN_BYTES;
        }
    }

    /* Update UDP Length */
    nwalWrite16bits_m(pUdpHeader,
                      NWAL_UDP_OFFSET_LEN,
                      (ploadBuffLen + NWAL_UDP_HDR_LEN_BYTES));

    /* Offload UDP checksum to PA */
    pPktInfo->startOffset = 0;
    pPktInfo->l4OffBytes = (uint8_t *)pUdpHeader - (uint8_t *)pOrigBuffPtr;
    pPktInfo->l4HdrLen = NWAL_UDP_HDR_LEN_BYTES;
    pPktInfo->ploadLen = ploadBuffLen;
#ifndef NWAL_ENABLE_SW_CHECKSUM
    pPktInfo->txFlag1 |= NWAL_TX_FLAG1_DO_UDP_CHKSUM;
#else
    nwal_utilCompUdpChksums(pPktInfo,
                            pUdpHeader,
                            Pktlib_getDescFromPacket(pPloadPkt));
#endif

    if(pL2L3HdrInfoAddr->outHandle)
    {
#ifdef NWAL_ENABLE_SA
        uint32_t            swInfo0 = 0;
        uint32_t            swInfo1 = 0;
        nwal_RetValue       nwalRetVal;
#ifdef NWAL_DUMP_PKT
    printf("NWAL:LOG Dump of PKT BEGIN BEFORE IPSEC Header Insertion \n");
    nwal_dump_buf(pOrigBuffPtr,
                 (pL2L3HdrInfoAddr->hdrLen + NWAL_UDP_HDR_LEN_BYTES));
    printf("NWAL:LOG Dump of PKT Data END \n");
#endif
        /* IPSec Channel */
        nwalRetVal = nwal_InsIpSec(nwalProcessCtx.pSharedMemBase,
                                   pHdrDesc,
                                   pL2L3HdrInfoAddr,
                                   (nwalIpSecInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                pL2L3HdrInfoAddr->outHandle)),
                                   pOrigBuffPtr,
                                   macHdrLen,
                                   ipHdrLen,
                                   pPktInfo,
                                   &swInfo0,
                                   &swInfo1);
        if(nwalRetVal != nwal_OK)
        {
            Pktlib_freePacket(pHdrPkt);
            return(nwalRetVal);
        }
        pPkt = pHdrPkt;
#ifdef NWAL_DUMP_PKT
    printf("NWAL:LOG Dump of PKT BEGIN AFTER IPSEC Header Insertion \n");
    nwal_dump_buf((uint8_t *)pHdrDesc->buffPtr,pHdrDesc->buffLen);
    printf("NWAL:LOG Dump of PKT Data END \n");
#endif
#else
        return (nwal_ERR_SA_NOT_ENABLED);
#endif
    }
    /*** IP Sec Insert **/
    else
    {
        pPkt = Pktlib_packetMerge(pHdrPkt,pPloadPkt,NULL);
        Cppi_setData(Cppi_DescType_HOST,
                     (Cppi_Desc *)pHdrDesc,
                     pOrigBuffPtr,
                     hdrLen);
    }

#ifndef __ARMv7
    /* Update the Cache. TBD: Include below in packet Lib*/
    NWAL_osalWriteBackCache((void *)pOrigBuffPtr,hdrLen);
    NWAL_osalWriteBackCache((void *)pPloadBuffPtr,ploadBuffLen);


    /* Descriptor size for payload descriptor set to minimum cache line size */
    Pktlib_writebackPkt(pPloadPkt);
    Pktlib_writebackPkt(pPkt);
#endif
    pPktInfo->pPkt = pPkt;
    pPktInfo->mtuSize = pPortInfo->txConnCfg.mtuSize;

    return nwal_OK;

}


/********************************************************************
 * FUNCTION PURPOSE: API for transmitting packet out to network
 ********************************************************************
 * DESCRIPTION: API for transmitting packet out to network
 ********************************************************************/
nwal_RetValue nwal_send  (nwal_Inst         nwalInst,
                          nwal_Handle       nwalHandle,
                          nwalTxPktInfo_t*  pPktInfo)
{
    nwal_RetValue       nwalRetVal;
    uint32_t            swInfo0 = 0;
    uint32_t            swInfo1 = 0;
    nwalTxPSCmdInfo_t   txPsCmdInfo;
    Cppi_HostDesc*      pPktDesc;

#ifdef NWAL_DUMP_PKT
    uint8_t *           pTmpBuf;
    uint32_t            tmpLen;
#endif


    if(pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_CRYPTO_MASK)
    {
#ifdef NWAL_ENABLE_SA
        nwalRetVal = nwal_getSaSwInfo((nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                                  nwalHandle)),
                                      &swInfo0,
                                      &swInfo1);
        if(nwalRetVal != nwal_OK)
        {
            return nwalRetVal;
        }
#else
        return (nwal_ERR_SA_NOT_ENABLED);
#endif
    }

        nwalRetVal = nwal_prepPSCmdLabel(nwalProcessCtx.pSharedMemBase,
                                         swInfo0,
                                         swInfo1,
                                         pPktInfo,
                                         &txPsCmdInfo);
        if(nwalRetVal != nwal_OK)
        {
            return nwalRetVal;
        }
    
        if(((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_ESP_CRYPTO) ==
                 NWAL_TX_FLAG1_DO_IPSEC_ESP_CRYPTO))
        {
            if(((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_UDP_CHKSUM) ==
                 NWAL_TX_FLAG1_DO_UDP_CHKSUM) ||
                ((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_TCP_CHKSUM) ==
                  NWAL_TX_FLAG1_DO_TCP_CHKSUM))
            {
            nwal_mCmdSetL4CkSumCrypPort(pPktInfo->pPkt,
                                        &txPsCmdInfo,
                                        pPktInfo->l4OffBytes,
                                        (pPktInfo->l4HdrLen +
                                        pPktInfo->ploadLen),
                                        pPktInfo->pseudoHdrChecksum,
                                        pPktInfo->saOffBytes,
                                        pPktInfo->saPayloadLen,
                                        swInfo0,
                                        swInfo1,
                                        pPktInfo->enetPort);
                goto nwal_send_tx_pkt;
            }
    
        nwal_mCmdSetCrypPort(pPktInfo->pPkt,
                             &txPsCmdInfo,
                             pPktInfo->saOffBytes,
                             pPktInfo->saPayloadLen,
                             swInfo0,
                             swInfo1,
                             pPktInfo->enetPort);
            goto nwal_send_tx_pkt;
        }
    
        if(((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_IPSEC_AH_CRYPTO) ==
                 NWAL_TX_FLAG1_DO_IPSEC_AH_CRYPTO))
        {
            if(((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_UDP_CHKSUM) ==
                 NWAL_TX_FLAG1_DO_UDP_CHKSUM) ||
                ((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_TCP_CHKSUM) ==
                  NWAL_TX_FLAG1_DO_TCP_CHKSUM))
            {
            nwal_mCmdSetL4CkSumAHCrypPort(pPktInfo->pPkt,
                                            &txPsCmdInfo,
                                            pPktInfo->l4OffBytes,
                                            (pPktInfo->l4HdrLen +
                                             pPktInfo->ploadLen),
                                            pPktInfo->pseudoHdrChecksum,
                                            pPktInfo->saOffBytes,
                                            pPktInfo->saPayloadLen,
                                            swInfo0,
                                            swInfo1,
                                            pPktInfo->saAhIcvOffBytes,
                                            pPktInfo->saAhMacSize,
                                            pPktInfo->enetPort);
                goto nwal_send_tx_pkt;
            }
    
        nwal_mCmdSetAHCrypPort(pPktInfo->pPkt,
                                 &txPsCmdInfo,
                                 pPktInfo->saOffBytes,
                                 pPktInfo->saPayloadLen,
                                 swInfo0,
                                 swInfo1,
                                 pPktInfo->saAhIcvOffBytes,
                                 pPktInfo->saAhMacSize,
                                 pPktInfo->enetPort);
            goto nwal_send_tx_pkt;
        }
    
        if(((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_UDP_CHKSUM) ==
             NWAL_TX_FLAG1_DO_UDP_CHKSUM) ||
           ((pPktInfo->txFlag1 & NWAL_TX_FLAG1_DO_TCP_CHKSUM) ==
             NWAL_TX_FLAG1_DO_TCP_CHKSUM))
        {
        nwal_mCmdSetL4CkSumPort( pPktInfo->pPkt,
                                 &txPsCmdInfo,
                                 pPktInfo->l4OffBytes,
                                 (pPktInfo->l4HdrLen + pPktInfo->ploadLen),
                                 pPktInfo->pseudoHdrChecksum,
                                 pPktInfo->enetPort);
            goto nwal_send_tx_pkt;
        }

    nwal_mCmdSetPort(pPktInfo->pPkt,
                     &txPsCmdInfo,
                     pPktInfo->enetPort);

nwal_send_tx_pkt:
#ifndef __ARMv7
    Pktlib_writebackPkt(pPktInfo->pPkt);
#endif
    pPktDesc = Pktlib_getDescFromPacket(pPktInfo->pPkt);

#ifdef NWAL_DUMP_PKT
    pTmpBuf = nwalCppi_getPSData(Cppi_DescType_HOST,
                                 Cppi_PSLoc_PS_IN_DESC,
                                (Cppi_Desc *)pPktDesc,
                                &tmpLen);
    printf("NWAL:LOG Dump of Descriptor Begin TX Queue :0x%x\n",txPsCmdInfo.txQueue);
    nwal_dump_buf((uint8_t *)pPktDesc,NWAL_DESC_SIZE);
    printf("NWAL:LOG Dump of Descriptor END \n");
    printf("NWAL:LOG Dump of PS Data BEGIN \n");
    nwal_dump_buf_32bit(pTmpBuf,tmpLen);
    printf("NWAL:LOG Dump of PS Data END \n");

    Pktlib_getDataBuffer(pPktInfo->pPkt,&pTmpBuf,&tmpLen);
    printf("NWAL:LOG Dump of PKT BEGIN \n");
    nwal_dump_buf(pTmpBuf,tmpLen);
    printf("NWAL:LOG Dump of PKT Data END \n");
#endif
    /* Send the packet out to transmit Q*/
    Qmss_queuePushDescSize (txPsCmdInfo.txQueue,
                            pPktDesc,
                            NWAL_DESC_SIZE);
    return nwal_OK;
}

/********************************************************************
 * FUNCTION PURPOSE: API to Initialize PS command info
 ********************************************************************
 * DESCRIPTION: API to Initialize PS command info
 ********************************************************************/
nwal_RetValue nwal_initPSCmdInfo  (nwal_Inst            nwalInst,
                                   nwalTxPktInfo_t*     pPktInfo,
                                   nwalTxPSCmdInfo_t*   pTxPSCmdInfo)
{
    /* SA swinfo & MAC Size by default will be set to 0. To be reset during run time
     */

    return(nwal_prepPSCmdLabel(nwalProcessCtx.pSharedMemBase,
                                0,
                                0,
                                pPktInfo,
                                pTxPSCmdInfo));
}

/********************************************************************
 * FUNCTION PURPOSE: API to querry statistics from PA
 ********************************************************************
 * DESCRIPTION: API to querry statistics from PA
 ********************************************************************/
nwal_RetValue nwal_getPAStats    (nwal_Inst             nwalInst,
                                  nwal_TransID_t        transId,
                                  paSysStats_t*         pPaStats,
                                  nwal_Bool_t           doClear)
{

    Cppi_HostDesc*      pHd;
    paReturn_t          netCPRet;
    paCmdReply_t        cmdReply;
    uint16_t            cmdSize = pa_REQUEST_STATS_MIN_CMD_BUF_SIZE_BYTES;
 
    nwal_RetValue       retVal;
    nwalLocContext_t*   pLocContext;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    if((transId == NWAL_TRANSID_SPIN_WAIT) && (!pPaStats))
    {
        return (nwal_ERR_INVALID_PARAM);
    }


    if (nssGblCfgParams.layout.fNssGen2)
    {
        netCPRet = Pa_querySysStats (nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                             doClear, 
                             &pLocContext->paSysStats);
        if (netCPRet == pa_OK)
        {
            memcpy(pPaStats,&pLocContext->paSysStats,sizeof(paSysStats_t));
        }
        else
        {
            pLocContext->extErr = netCPRet;
            return nwal_ERR_PA;
        }
    }
    else
    {
        int                 cmdDest;
        pLocContext->transInfo.transId = transId;
        retVal = nwal_prepCmdBuf(nwalProcessCtx.pSharedMemBase,
                                 &cmdSize,
                                 &cmdReply,
                                 &pLocContext->transInfo,
                                 &pHd);
        if(retVal != nwal_OK)
        {
            return retVal;
        }
        netCPRet = Pa_requestStats (nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                              doClear,
                             (paCmd_t) pHd->buffPtr,
                             &cmdSize,
                             &cmdReply,
                             &cmdDest);
        if (netCPRet == pa_OK)
        {

            retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                                    cmdSize,
                                    cmdDest,
                                    pHd);
            if(retVal != nwal_OK)
            {
                return (retVal);
            }
            /*update global info; need to protect from multicore*/
            pLocContext->numPendPAReq++;
        
            nwal_setTransInfo(&pLocContext->transInfo,
                              NWAL_HANDLE_ID_TRANS_PA_STAT,
                              0);
        
            if(transId == NWAL_TRANSID_SPIN_WAIT)
            {
                /* Block for response */
                retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                                       NULL,
                                       NULL,
                                       &pLocContext->transInfo.handleHdr);
        
                /* Copy the PA Stats received */
                memcpy(pPaStats,&pLocContext->paSysStats,sizeof(paSysStats_t));
            }
        }
        else
        {
            pLocContext->extErr = netCPRet;
            return nwal_ERR_PA;
        }
    }

    NWAL_osalWriteBackCache(pLocContext,sizeof(nwalLocContext_t));
    return (nwal_OK);
}

/********************************************************************
 * FUNCTION PURPOSE: API to retrieve local channel context
 ********************************************************************
 * DESCRIPTION: API to retrieve local channel context
 ********************************************************************/
nwal_RetValue nwal_getChanCxtInfo(nwal_Inst              nwalInst,
                                  nwal_Handle            nwalHandle,
                                  nwalChanCxtInfo_t*     pInfo)
{
    nwalHandleHdr_t*    pHndlHdr = (nwalHandleHdr_t*)nwalHandle;
    memset(pInfo,0,sizeof(nwalChanCxtInfo_t));

    switch(pHndlHdr->handleId)
    {
        case NWAL_HANDLE_PORT_INST:
        {
            nwalPortInfo_t* pPortInfo = (nwalPortInfo_t*)pHndlHdr;
            NWAL_osalInvalidateCache(pPortInfo,sizeof(nwalPortInfo_t));
            pInfo->validBitMap |= NWAL_CTRL_CXT_VALID_PA_L4_HANDLE;
            memcpy(pInfo->paL4Handle,pPortInfo->paPortHandle,sizeof(paHandleL4_t));
            break;
        }
        case NWAL_HANDLE_IP_INST:
        {
            nwalIpInfo_t*  pIpInfo = (nwalIpInfo_t*)pHndlHdr;
            pInfo->validBitMap |= NWAL_CTRL_CXT_VALID_PA_INNER_IP_HANDLE;
            pInfo->paInnerIpHandle = pIpInfo->paIpHandle;
            break;
        }
#ifdef NWAL_ENABLE_SA
        case NWAL_HANDLE_IPSEC_INST:
        {
            nwalIpSecInfo_t*  pIpSecInfo = (nwalIpSecInfo_t*)pHndlHdr;
            pInfo->validBitMap |= NWAL_CTRL_CXT_VALID_PA_OUTER_IP_HANDLE;
            pInfo->paOuterIpHandle = pIpSecInfo->paIpHandle;

            pInfo->validBitMap |= NWAL_CTRL_CXT_VALID_IPSEC_SA_HANDLE;
            pInfo->saChanHandle = pIpSecInfo->hdr.saChanHandle;
            NWAL_osalInvalidateCache(pIpSecInfo,sizeof(nwalIpSecInfo_t));
            pHndlHdr = (nwalHandleHdr_t*)pIpSecInfo->prevHandle;
            break;
        }

        case NWAL_HANDLE_DM_INST:
        {
            nwalDmSaInfo_t* pDmSaInfo = (nwalDmSaInfo_t*)pHndlHdr;
            pInfo->validBitMap |= NWAL_CTRL_CXT_VALID_DM_SA_HANDLE;
            pInfo->saChanHandle = pDmSaInfo->hdr.saChanHandle;
            break;
        }
#endif
    }
    return (nwal_OK);
}
