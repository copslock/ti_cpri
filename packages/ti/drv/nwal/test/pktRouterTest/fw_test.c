/******************************************************************************
 * FILE PURPOSE:  File containing NWAL test routines
 ******************************************************************************
 * FILE NAME:   fw_test.c
 *
 * DESCRIPTION: File containing NWAL test routines for just PKT Send/Receive
 *              from slave devices. All NETCP configuration being done from
 *              master device
 *
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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
#include "fw_test.h"
#include "fw_rm.h"
#include <ti/csl/csl_cpsw.h>

void testNwalPoll (void );

uint32_t                test_nwal_ip_id  = 0xfffffffe;
uint32_t                rcvd_AppID = 0;
uint32_t                nwalPktLoopBack=nwal_FALSE;
//#define NWAL_PKT_LOOPBACK
testNwLocContext_t      testNwLocContext;
#pragma DATA_SECTION (testNwGlobContext, ".testNwGlobContext");
testNwGlobContext_t     testNwGlobContext;

unsigned long   cumCycles =0;
unsigned long   startProfile =0;
unsigned long   numCycleMeas =0;
unsigned long   avgCycleMeas =0;

static inline fw_startProfile()
{
   startProfile = fw_read_clock();
}
static inline fw_endProfile()
{
    unsigned long   curClock = fw_read_clock();
    cumCycles += (curClock -  startProfile);
    numCycleMeas++;
    avgCycleMeas = cumCycles/numCycleMeas;
}

#define TEST_NWAL_EXTRA_DEBUG   1

/* Protocol header offset information */
#define NWAL_TEST_ETH_OFFSET_DEST_MAC           0
#define NWAL_TEST_ETH_OFFSET_SRC_MAC            6
#define NWAL_TEST_ETH_OFFSET_TYPE_LEN           12
#define NWAL_TEST_ETH_VLAN_TAG                  12
#define NWAL_TEST_ETH_TYPE_VLAN_TAG             0x8100
#define NWAL_TEST_ETH_TYPE_IP                   0x0800
#define NWAL_TEST_ETH_TYPE_IPv6                 0x86DD
#define NWAL_TEST_ETH_MAC_ADDR_LEN              6
#define NWAL_TEST_ETH_DIX_HDR_LEN               14

#define NWAL_TEST_IPV4_OFFSET_SRC_ADDR          (12)
#define NWAL_TEST_IPV4_OFFSET_DST_ADDR          (16)
#define NWAL_TEST_IPV4_UDP                      0x11

/* UDP byte offsets to fields */
#define NWAL_TEST_UDP_OFFSET_SRC_PORT           (0)
#define NWAL_TEST_UDP_OFFSET_DEST_PORT          (2)
#define NWAL_TEST_UDP_OFFSET_LEN                (4)
#define NWAL_TEST_UDP_OFFSET_CHKSUM             (6)
/* UDP definitions */
#define NWAL_TEST_UDP_HDR_LEN_BYTES             (4*2)

extern cregister    volatile unsigned int DNUM;
extern cregister    volatile unsigned int TSCL;
volatile uint32_t clearPAStats = 0;
volatile uint32_t getPAStats = 0;
#ifdef C66x_MASTER
 /* All packets in this test use a single destination address ethernet
 * routing for the L2 lookup in PA 
 */
static nwalMacParam_t   nwalMacParam= {
    0,      /* validParams */
    0,      /* ifNum */
    0,      /* vlanId      */
    { 0x20, 0x21, 0x22, 0x23, 0x24, 0x25 },      /* Local mac */
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Remote mac */
    NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
    NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
    CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
    QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
    0
};

static nwalIpParam_t    nwalIpParam= {
    0,
    pa_IPV4,      /* IP Type */
    { 192, 168, 16, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Dest IP */ 
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },       /* Src IP */ 
    { 0x1,17,0,0},/* IP Options */
    NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
    NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
    CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
    QMSS_PARAM_NOT_SPECIFIED,                     /* Use default queue configured to NWAL if packet is routed to host */
    0
};

static nwalRxConnCfg_t nwalRxConnCfg = {
    0,      /* inHandle */
    0,      /* valid params */
    0x0, /* appProto */
    0, /* rxCoreId */
    NWAL_MATCH_ACTION_HOST,                      /* Terminate at host in case of match */
    CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
    QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
    0                                            /* route type */
};
static nwalTxConnCfg_t nwalTxConnCfg = {
    0,      /* outHandle */
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },      /* remMacAddr: Same as local*/
    { 0,0,0x2,0},   /* mac options */
    0,
    pa_IPV4,      /* IP Type */
    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,  /* Destination IP Address */
    {0x1,17,0,0},/* IP Options */
    0x0555 /* appProto */
};


nwal_Handle     nwalMacHandle;
nwal_Handle     nwalIpHandle;
nwal_Handle     nwalUdpHandle[CPU_NUM_REM_FAST_PATH_CORES + 1];


static uint8_t testPayload[] = {
  /* Payload */
  0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
  0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
  0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
  0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
  0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
  0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61,
  0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
  0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
  0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
  0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81

};
#define TEST_PAYLOAD_LEN            80


/*****************************************************************************
 * FUNCTION PURPOSE: Test for nwal_Send
 *****************************************************************************
 * DESCRIPTION: The function will be called to send packets out
 *****************************************************************************/
nwal_Bool_t  testNwalSendDummyPkt (nwal_Handle  handle)
{
     nwal_RetValue      retValue;
     /* Share the same memory buffer pool for the TX packet */
     nwalMbufPool_t*    pMbufPool;
     uint8_t            count = 0;
     nwalBufPool_t*     pBufPool;
     Ti_Pkt*            pPkt;
     uint8_t*           pDataBuffer;
     uint32_t           dataLen;
     nwalTxPktInfo_t    txPktInfo;

     pMbufPool = &testNwLocContext.nwalLocCfg.txPktPool;
     pBufPool = pMbufPool->bufPool;
     while(count < pMbufPool->numBufPools)
     {
         if(pBufPool->bufSize >= TEST_PAYLOAD_LEN)
         {
             break;
         }

         pBufPool++;
         count++;
     }
     if(count == pMbufPool->numBufPools)
     {
         return nwal_FALSE;
     }

     pPkt = Pktlib_allocPacket(pBufPool->heapHandle,TEST_PAYLOAD_LEN);
     if (pPkt == NULL)
            return (FALSE);

     Pktlib_getDataBuffer(pPkt,&pDataBuffer,&dataLen);
     if(dataLen < TEST_PAYLOAD_LEN)
     {
         /*  Unexpected */
         while(1);
     }

    /* Memory copy the Payload
     */
    memcpy(pDataBuffer,
           testPayload,
           TEST_PAYLOAD_LEN);
           
    Osal_writeBackCache(pDataBuffer,TEST_PAYLOAD_LEN);
    Pktlib_setDataBufferLen(pPkt,TEST_PAYLOAD_LEN);
    memset(&txPktInfo,0,sizeof(nwalTxPktInfo_t));
    txPktInfo.pPkt = pPkt;
    txPktInfo.lpbackPass = nwal_FALSE;

    /* Update the L2/L3/L4 protocol header */
    retValue = nwal_updateProtoHdr(testNwGlobContext.nwalInstHandle,
                                   handle,
                                   &txPktInfo);
    if(retValue != nwal_OK)
    {
        return nwal_FALSE;
    }
    retValue = nwal_send(testNwGlobContext.nwalInstHandle,
                         handle,
                         &txPktInfo);
    if(retValue != nwal_OK)
    {
        return nwal_FALSE;
    }
    
    return nwal_TRUE;
}
/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC at NWAL
 *****************************************************************************
 * DESCRIPTION: The function will be called to configure MAC instance
 *****************************************************************************/
nwal_Handle  testAddMac    (nwalMacParam_t     *pMacInfo)
{
    nwal_RetValue       retValue;
    nwal_Handle         macHandle;
    
    retValue = nwal_setMacIface(  testNwGlobContext.nwalInstHandle,
                                  NWAL_TRANSID_SPIN_WAIT,
                                  0,
                                  pMacInfo,
                                  &macHandle);
    if(retValue !=  nwal_OK)
    {
        System_printf("ERROR: nwal_setMacIface returned Error Code %d\n",
                    retValue);
        nwal_SystemFlush();
        return NULL;
    }
    nwal_SystemFlush();
#ifndef __LINUX_USER_SPACE
    /* Update Switch ALE for the MAC Address so that packet will be routed to PA
     * In the case of NWAL running in Linux User space below code is currently disabled as switch
     * access is limited currently in Kernel. Workaround would be in DSP to send out first packet
     * after adding a MAC entry
     */
    if(testNwSwUpdateMacAddr(pMacInfo->macAddr))
    {
        System_printf("CPSW ALE table updated for the destination MAC address \n");
        nwal_SystemFlush();
    }
#endif
    return(macHandle);
}


/*****************************************************************************
 * FUNCTION PURPOSE: Configure IP at NWAL
 *****************************************************************************
 * DESCRIPTION: The function will be called to configure IP instance
 *****************************************************************************/
nwal_Handle  testAddIp  (nwalIpParam_t* pIpParam,
                         nwal_Handle    macHandle)
{
    nwal_RetValue           retValue;
    nwal_Handle             ipHandle;
    retValue = nwal_setIPAddr(   testNwGlobContext.nwalInstHandle,
                                  NWAL_TRANSID_SPIN_WAIT,
                                  0,
                                  macHandle,
                                  pIpParam,
                                  &ipHandle);
    if(retValue !=  nwal_OK)
    {
        System_printf("ERROR: nwal_setIPAddr returned Error Code %d\n",
                    retValue);
        nwal_SystemFlush();
        return NULL;
    }
    nwal_SystemFlush();
    return(ipHandle);
}
/*****************************************************************************
 * FUNCTION PURPOSE: Add MAC/IP/UDP remote connection for Remote Fast Path DSP
 *                   cores on SOC
 *****************************************************************************
 * DESCRIPTION: Add MAC/IP/UDP remote connection for Remote Fast Path DSP
 *              cores on SOC
 *****************************************************************************/
nwal_Bool_t testAddRemFPL4()
{
    uint8_t                 count;
    nwal_RetValue           retValue;

    nwalRxConnCfg.inHandle = nwalIpHandle;

    /* Add UDP entry for Remote Core */
    nwalRxConnCfg.rxCoreId = 0;
    for(count = 0;count < CPU_NUM_REM_FAST_PATH_CORES; count++)
    {
        nwalRxConnCfg.appProto.udpPort =
                                TEST_NWAL_BASE_REM_FP_UDP_PORT + count;
        nwalRxConnCfg.appRxPktQueue =
                                TEST_NWAL_BASE_REM_FP_RX_PKT_QUEUE  + count;
        System_printf("CORE: %d Adding Remote Fast Path Port config,Terminating Core:%d UDP Port:%d Queue:%d\n",
                       DNUM,
                       count,
                       nwalRxConnCfg.appProto.udpPort,
                       nwalRxConnCfg.appRxPktQueue);

        retValue = nwal_addConn(testNwGlobContext.nwalInstHandle,
                                NWAL_TRANSID_SPIN_WAIT,
                                (nwal_AppId)TEST_NWAL_APP_ID_DEFAULT,
                                NWAL_APP_PLOAD_PROTO_UDP,
                                &nwalRxConnCfg,
                                &nwalTxConnCfg,
                                &nwalUdpHandle[count]);
        if(retValue !=  nwal_OK)
        {
            System_printf("CORE: %d ERROR: nwal_addConn returned Error Code %d for remote FP Connection \n",
                           DNUM,retValue);
            nwal_SystemFlush();
            while(1);
        }
        #if 1
        if(testNwalSendDummyPkt(nwalUdpHandle[count]) != nwal_TRUE)
        {
            while(1);
        }
        #endif
    }
    nwalRxConnCfg.appRxPktQueue = QMSS_PARAM_NOT_SPECIFIED;
    nwal_SystemFlush();
    return nwal_TRUE;
}
#endif
/********************************************************************
 *  FUNCTION PURPOSE: Ones complement addition utility
 ********************************************************************
 ********************************************************************/
uint16_t test_utilOnesComplementAdd (uint16_t v1, uint16_t v2)
{
  uint32_t result;

  result = (uint32_t)v1 + (uint32_t)v2;
  result = (result >> 16) + (result & 0xffff);
  result = (result >> 16) + (result & 0xffff);

  return ((uint16_t)result);
}

/********************************************************************
 *  FUNCTION PURPOSE: Ones complement checksum utility
 ********************************************************************
 ********************************************************************/
 uint16_t test_utilOnesCompChkSum (uint8_t *p, uint32_t nwords)
{
  uint16_t chksum = 0;
  uint16_t v;
  uint32_t i;
  uint32_t j;

  for (i = j = 0; i < nwords; i++, j+=2)  {
    v = (p[j] << 8) | p[j+1];
    chksum = test_utilOnesComplementAdd (chksum, v);
  }
  return (chksum);
} /* utilOnesCompChkSum */


/*******************************************************************************
 * FUNCTION PURPOSE: Compute ipv4 psudo checksum
 *******************************************************************************
 * DESCRIPTION: Compute ipv4 psudo checksum
 ******************************************************************************/
uint16_t test_utilGetIpv4PsudoChkSum (uint8_t *data, uint16_t payloadLen)
{
  uint16_t psudo_chksum;

  psudo_chksum = test_utilOnesCompChkSum (&data[12], 4);
  psudo_chksum = test_utilOnesComplementAdd(psudo_chksum, (uint16_t) data[9]);
  psudo_chksum = test_utilOnesComplementAdd(psudo_chksum, payloadLen);

  return (psudo_chksum);

} /* utilGetIpv4PsudoChkSum */

/******************************************************************************
 * FUNCTION PURPOSE: Reads 16 bit value into 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Reads 16 bit value into 16 bit word.  No assumptions
 *
 * void testNwalRead16bits_m (
 *    uint8_t *base,    - Base of byte array
 *    uint16_t byteOffset, - byte offset to write; assumed to be even
 *    uint16_t val)        - 16 bit val
 *
 *****************************************************************************/
static inline uint16_t testNwalRead16bits_m (uint8_t *base,
                                             uint16_t byteOffset)
{
  char *wptr = ((char *)base + byteOffset);
  uint16_t  retVal;
  retVal = (((uint16_t)wptr[0]) << 8) | (wptr[1] & 0xFF);
  return retVal;
#if 0
  *pVal = wptr[0];
  *pVal = (*pVal << 8);
  *pVal = (*pVal | wptr[1]);
#endif

} /* testNwalRead16bits_m */


/******************************************************************************
 * FUNCTION PURPOSE: Bit packing to 16 bit value
 ******************************************************************************
 * DESCRIPTION: Bit packing to 16 bit value
 *
 *****************************************************************************/
static inline void testNwalWrite16bits_m (uint8_t *base,
                                          uint16_t byteOffset,
                                          uint16_t val)
{
  char *wptr = ((char *)base + byteOffset);

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  wptr[0] = (char)(val>>8);
  wptr[1] = (char)(val & 0xff);

} /* testNwalWrite16bits_m */

#ifdef TEST_NWAL_EXTRA_DEBUG
/****************************************************************************
 * FUNCTION:    fw_nwal_dump_buf
 ****************************************************************************
 * DESCRIPTION: This function will dump the contents of buffer
 *
 ****************************************************************************/
static void  fw_nwal_dump_buf
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
                printf("NWAL *: Internal Error in fw_nwal_dump_buf().Row Count: %d \n",
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
/*****************************************************************************
 * FUNCTION PURPOSE: Test for nwal_Send with full packet header + payload
 *****************************************************************************
 * DESCRIPTION: The function will be called to send packets out
 *****************************************************************************/
static inline void  testNwalFlipAndSendMacIpUdpPkt (nwalRxPktInfo_t*      pRxPktInfo,
                                                    uint8_t*              pDataBuff,
                                                    uint32_t              dataLen,
                                                    nwal_Bool_t*          pTxStatus)
{
    nwalMacAddr_t       tmpMac[NWAL_MAC_ADDR_SIZE];
    nwalIpAddr_t        tmpIpAddr;
    uint16_t            tmpVal;
    nwalTxPktInfo_t     txPktInfo;
#if 0
    uint16_t            l4HdrLen = 0;
    uint8_t*            pIpHdr;
#endif
    Cppi_HostDesc*      pPktDesc;
    int                 len;
    uint8_t*            pTmpBuf;
    uint32_t            tmpLen;
    nwal_RetValue       retValue;
    uint16_t            srcPort;
    uint16_t            dstPort;
    
#ifdef C66x_MASTER
#ifdef NWAL_TEST_DESC_GLOB_MEM
    Osal_invalidateCache((void *)pRxPktInfo->pPkt,NWAL_DESC_SIZE);
#endif
#else
    Osal_invalidateCache((void *)pRxPktInfo->pPkt,NWAL_DESC_SIZE);
#endif
    Pktlib_getDataBuffer(pRxPktInfo->pPkt,&pTmpBuf,&tmpLen);
#ifdef C66x_MASTER
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
    Osal_invalidateCache((void *)pTmpBuf,tmpLen);
#endif
#else
    Osal_invalidateCache((void *)pTmpBuf,tmpLen);
#endif
#ifdef TEST_NWAL_EXTRA_DEBUG
    printf("NWAL:LOG Dump of PKT BEGIN Before Modification Begin\n");
    fw_nwal_dump_buf(pTmpBuf,tmpLen);
    printf("NWAL:LOG Dump of PKT Data Before Modification END \n");
#endif
    *pTxStatus = nwal_FALSE;

    /* Flip the MAC source and Destination */
    memcpy((uint8_t*)tmpMac,
           &pDataBuff[NWAL_TEST_ETH_OFFSET_DEST_MAC],
           NWAL_MAC_ADDR_SIZE);
    memcpy(&pDataBuff[NWAL_TEST_ETH_OFFSET_DEST_MAC],
           &pDataBuff[NWAL_TEST_ETH_OFFSET_SRC_MAC],
           NWAL_MAC_ADDR_SIZE);
    memcpy(&pDataBuff[NWAL_TEST_ETH_OFFSET_SRC_MAC],
           (uint8_t*)tmpMac,
           NWAL_MAC_ADDR_SIZE);
    tmpVal = testNwalRead16bits_m(pDataBuff,
                                  NWAL_TEST_ETH_VLAN_TAG);
    if(tmpVal == NWAL_TEST_ETH_TYPE_IP)
    {
        /* Flip IPv4 Source and Destination */
        memcpy(&tmpIpAddr.ipv4,
               &pDataBuff[pRxPktInfo->l3OffBytes + NWAL_TEST_IPV4_OFFSET_SRC_ADDR],
               NWAL_IPV4_ADDR_SIZE);

        memcpy(&pDataBuff[pRxPktInfo->l3OffBytes + NWAL_TEST_IPV4_OFFSET_SRC_ADDR],
               &pDataBuff[pRxPktInfo->l3OffBytes + NWAL_TEST_IPV4_OFFSET_DST_ADDR],
               NWAL_IPV4_ADDR_SIZE);

        memcpy(&pDataBuff[pRxPktInfo->l3OffBytes + NWAL_TEST_IPV4_OFFSET_DST_ADDR],
               &tmpIpAddr.ipv4,
               NWAL_IPV4_ADDR_SIZE);
        /* TBD: Leave checksum unchanged */
    }
    else
    {
        testNwLocContext.numUnhandledEthType++;
        return;
    }

    if(pRxPktInfo->l4ProtoType == NWAL_TEST_IPV4_UDP)
    {
        /* Flip source and destination UDP port */
        srcPort = testNwalRead16bits_m(pDataBuff,
                             (pRxPktInfo->l4OffBytes + NWAL_TEST_UDP_OFFSET_SRC_PORT));
        dstPort = testNwalRead16bits_m(pDataBuff,
                             (pRxPktInfo->l4OffBytes + NWAL_TEST_UDP_OFFSET_DEST_PORT));

        testNwalWrite16bits_m(pDataBuff,
                              (pRxPktInfo->l4OffBytes + NWAL_TEST_UDP_OFFSET_SRC_PORT),
                              dstPort);
        testNwalWrite16bits_m(pDataBuff,
                              (pRxPktInfo->l4OffBytes + NWAL_TEST_UDP_OFFSET_DEST_PORT),
                              srcPort);
#if 0
        /* TBD: Leave checksum unchanged */
        l4HdrLen = NWAL_TEST_UDP_HDR_LEN_BYTES;
#endif
    }
    else
    {
        testNwLocContext.numUnhandledL4Type++;
        return;
    }

    memset(&txPktInfo,0,sizeof(txPktInfo));
    txPktInfo.pPkt = pRxPktInfo->pPkt;

    txPktInfo.txFlag1 = (NWAL_TX_FLAG1_META_DATA_VALID);
    txPktInfo.lpbackPass = nwalPktLoopBack;
    txPktInfo.enetPort = pRxPktInfo->enetPort;
#if 0
    /* Would need to configure packet flow ID for enabling any
     * checksum offload to PA
     */
    txPktInfo.txFlag1 = 
                (NWAL_TX_FLAG1_META_DATA_VALID|
                 NWAL_TX_FLAG1_DO_IPV4_CHKSUM|
                 NWAL_TX_FLAG1_DO_UDP_CHKSUM);
    txPktInfo.startOffset = pRxPktInfo->startOffset;
    txPktInfo.ipOffBytes = pRxPktInfo->l3OffBytes;
    txPktInfo.l4OffBytes = pRxPktInfo->l4OffBytes;
    txPktInfo.l4HdrLen=l4HdrLen;
    txPktInfo.ploadLen = pRxPktInfo->ploadLen;
    pIpHdr = pDataBuff + txPktInfo.ipOffBytes;
    txPktInfo.pseudoHdrChecksum =
        test_utilGetIpv4PsudoChkSum(pIpHdr,
                                    (txPktInfo.ploadLen+8));
#endif
    pPktDesc = Pktlib_getDescFromPacket(txPktInfo.pPkt);

    /* Make sure there is no control info.  */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc, 0);
    len = Pktlib_getPacketLen(txPktInfo.pPkt);
    if(nwalPktLoopBack != nwal_TRUE)
    {
        len = len-4;
    }

    Cppi_setData (Cppi_DescType_HOST, 
                  (Cppi_Desc *) pPktDesc, 
                  pDataBuff,
                  len);
    Pktlib_setPacketLen(txPktInfo.pPkt,
                        len);
    if(Pktlib_getNextPacket(txPktInfo.pPkt) != 0) 
    {
        printf(" ERROR!!! RX Multi Segment Descriptor Length update not being handled");
    }
#ifdef C66x_MASTER
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
    /* Write back the cache */
    Osal_writeBackCache((void *)pDataBuff,len);
#endif
#ifdef NWAL_TEST_DESC_GLOB_MEM
    Osal_writeBackCache((void *)txPktInfo.pPkt,NWAL_DESC_SIZE);
#endif
#else
     Osal_writeBackCache((void *)pDataBuff,len);
     Osal_writeBackCache((void *)txPktInfo.pPkt,NWAL_DESC_SIZE);
#endif

    retValue = nwal_send(testNwGlobContext.nwalInstHandle,
                             nwal_HANDLE_INVALID,
                             &txPktInfo);
    if(retValue != nwal_OK)
    {
        return;
    }
    testNwLocContext.numPktSent++;
    *pTxStatus = nwal_TRUE;
}


/*****************************************************************************
 * FUNCTION PURPOSE: NWAL callback for all packets from Network
 *****************************************************************************
 * DESCRIPTION: The function will create NWAL instance which is prerequiite for
 *              any call to NWAL
 *****************************************************************************/
void testNWALRxPktCallback     (uint32_t            appCookie,
                                uint16_t            numPkts,
                                nwalRxPktInfo_t*    pPktInfo,
                                uint64_t            timestamp,
                                nwal_Bool_t*        pFreePkt)
{
    uint8_t*          pUdpHeader;
    uint16_t          destPort;
    uint8_t*          pDataBuffer;
    uint8_t           count;
    uint32_t          dataLen;
    uint32_t          errFlag;
    nwal_Bool_t       txStatus=nwal_FALSE;
    fw_startProfile();
    for(count=0;count<numPkts;count++)
    {
        pFreePkt[count] = nwal_FALSE;

        Pktlib_getDataBuffer(pPktInfo[count].pPkt,&pDataBuffer,&dataLen);
        rcvd_AppID = (uint32_t)(pPktInfo[count].appId);

        if((uint32_t)(pPktInfo[count].appId) == TEST_NWAL_APP_ID_MAC)
        {
            testNwLocContext.numL2PktsRecvd++;
        }
        else if((uint32_t)(pPktInfo[count].appId) == test_nwal_ip_id)
        {
            testNwLocContext.numL3PktsRecvd++;
        }
        else
        {
            pUdpHeader = pDataBuffer + pPktInfo[count].l4OffBytes + 2;
            destPort = testNwalRead16bits_m(pUdpHeader,0);
            if(destPort != (TEST_NWAL_BASE_REM_FP_UDP_PORT + DNUM))
            {
                /* Payload comparison failed */
                System_printf("\nCORE: %d ERROR:Unexpected packet received on Dest Port 0x%x \n",
                                DNUM,destPort);
                nwal_SystemFlush();
                pFreePkt[count] = nwal_TRUE;
                continue;
            }

            /* Check for any errors reported by NETCP */
            errFlag =
                ((pPktInfo[count].rxFlag1 & NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_MASK) >>
                  NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_SHIFT);
            if(errFlag == NWAL_RX_FLAG1_IPV4_CHKSUM_VERIFY_NACK)
            {
                /* Payload comparison failed */
                System_printf("CORE: %d ERROR: Packet with incorrect IPv4 Checksum received \n",
                               DNUM);
                nwal_SystemFlush();
#ifdef TEST_NWAL_EXTRA_DEBUG
                fw_nwal_dump_buf(pDataBuffer,dataLen);
#endif
                pFreePkt[count] = nwal_TRUE;
                continue;
                //while(1);
            }

            errFlag =
                ((pPktInfo[count].rxFlag1 & NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_MASK) >>
                  NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_SHIFT);
            if(errFlag == NWAL_RX_FLAG1_L4_CHKSUM_VERIFY_NACK)
            {
                /* Payload comparison failed */
                System_printf("CORE: %d ERROR: Packet with incorrect L4 Checksum received \n",
                               DNUM);
#ifdef TEST_NWAL_EXTRA_DEBUG
                fw_nwal_dump_buf(pDataBuffer,dataLen);
#endif
                nwal_SystemFlush();
                pFreePkt[count] = nwal_TRUE;
                continue;
                //while(1);
            }

            testNwLocContext.numL4PktsRecvd++;
            testNwalFlipAndSendMacIpUdpPkt(&pPktInfo[count],
                                           pDataBuffer,
                                           dataLen,
                                           &txStatus);
            if(txStatus == nwal_FALSE)
            {
                pFreePkt[count] = nwal_TRUE;
                continue;
            }
        }
    }
    fw_endProfile();
    return;
}
#define PA_VERSION_INCOMP_WAROUND

void testNWALCmdDispPaStats( )
{
    nwal_SystemFlush();
    System_printf("--- PA STATS BEGIN --- \n");


    System_printf("C1 number of packets:           %d\n", testNwGlobContext.paStats.classify1.nPackets);
    System_printf("C1 number IPv4 packets:         %d\n", testNwGlobContext.paStats.classify1.nIpv4Packets);
#ifndef PA_VERSION_INCOMP_WAROUND
    System_printf("C1 number Inner IPv4 packets:   %d\n", testNwGlobContext.paStats.classify1.nIpv4PacketsInner);
#endif
    System_printf("C1 number IPv6 packets:         %d\n", testNwGlobContext.paStats.classify1.nIpv6Packets);
#ifndef PA_VERSION_INCOMP_WAROUND
    System_printf("C1 number Inner IPv6 packets:   %d\n", testNwGlobContext.paStats.classify1.nIpv6PacketsInner);
#endif
    System_printf("C1 number custom packets:       %d\n", testNwGlobContext.paStats.classify1.nCustomPackets);
    System_printf("C1 number SRIO packets:         %d\n", testNwGlobContext.paStats.classify1.nSrioPackets);
    System_printf("C1 number llc/snap fail:        %d\n", testNwGlobContext.paStats.classify1.nLlcSnapFail);
    System_printf("C1 number table matched:        %d\n", testNwGlobContext.paStats.classify1.nTableMatch);
    System_printf("C1 number failed table matched: %d\n", testNwGlobContext.paStats.classify1.nNoTableMatch);
    System_printf("C1 number IP frags:             %d\n", testNwGlobContext.paStats.classify1.nIpFrag);
    System_printf("C1 number IP depth overflow:    %d\n", testNwGlobContext.paStats.classify1.nIpDepthOverflow);
    System_printf("C1 number vlan depth overflow:  %d\n", testNwGlobContext.paStats.classify1.nVlanDepthOverflow);
    System_printf("C1 number gre depth overflow:   %d\n", testNwGlobContext.paStats.classify1.nGreDepthOverflow);
    System_printf("C1 number mpls packets:         %d\n", testNwGlobContext.paStats.classify1.nMplsPackets);
    System_printf("C1 number of parse fail:        %d\n", testNwGlobContext.paStats.classify1.nParseFail);
    System_printf("C1 number invalid IPv6 opts:    %d\n", testNwGlobContext.paStats.classify1.nInvalidIPv6Opt);
    System_printf("C1 number TX IP Fragment Pkt:   %d\n", testNwGlobContext.paStats.classify1.nTxIpFrag);
    System_printf("C1 number of silent discard:    %d\n", testNwGlobContext.paStats.classify1.nSilentDiscard);
    System_printf("C1 number of invalid control:   %d\n", testNwGlobContext.paStats.classify1.nInvalidControl);
    System_printf("C1 number of invalid states:    %d\n", testNwGlobContext.paStats.classify1.nInvalidState);
    System_printf("C1 number of system fails:      %d\n\n", testNwGlobContext.paStats.classify1.nSystemFail);
    nwal_SystemFlush();

    System_printf("C2 number of Packets:             %d\n", testNwGlobContext.paStats.classify2.nPackets);
    System_printf("C2 number of UDP packets:         %d\n", testNwGlobContext.paStats.classify2.nUdp);
    System_printf("C2 number of TCP packets:         %d\n", testNwGlobContext.paStats.classify2.nTcp);
    System_printf("C2 number of custom packets:      %d\n", testNwGlobContext.paStats.classify2.nCustom);
    System_printf("C2 Reserved 3:                    %d\n", testNwGlobContext.paStats.classify2.reserved3);
    System_printf("C2 Reserved 4:                    %d\n", testNwGlobContext.paStats.classify2.reserved4);
    System_printf("C2 number of silent discard:      %d\n", testNwGlobContext.paStats.classify2.nSilentDiscard);
    System_printf("C2 number of invalid control:     %d\n\n", testNwGlobContext.paStats.classify2.nInvalidControl);
    nwal_SystemFlush();


    System_printf("Modify number of command file: %d\n\n", testNwGlobContext.paStats.modify.nCommandFail);

    System_printf("Reserved 5: %d\n\n", testNwGlobContext.paStats.common.reserved5);

    System_printf("--- PA STATS END --- \n");
}

/*****************************************************************************
 * FUNCTION PURPOSE: Callback for  statistics from PA
 *****************************************************************************
 * DESCRIPTION: The function will be called as callback once PA statistics is
 *              is received from NetCP
 *****************************************************************************/

void testNWALCmdPaStatsReply (  nwal_AppId       appHandle,
                                nwal_TransID_t   trans_id,
                                paSysStats_t     *stats)
{


        memcpy(&testNwGlobContext.paStats,stats,sizeof(paSysStats_t));
        testNWALCmdDispPaStats();
}
uint32_t poll_stats=0;
#if 0
CSL_CPSW_5GF_STATS  stats[2];
#endif
/*****************************************************************************
 * FUNCTION PURPOSE: Polls different RX queues for the packets or command
 *                   responses
 *****************************************************************************
 * DESCRIPTION: The function will be called for polling messages/packets
 *****************************************************************************/
void testNwalPoll (void )
{
    uint16_t        numRxPkts;

    while(1)
    {
        if(clearPAStats || getPAStats)
        {
            memset(&testNwGlobContext.paStats,0,sizeof(paSysStats_t));
            nwal_getPAStats(testNwGlobContext.nwalInstHandle,
                            NWAL_TRANSID_SPIN_WAIT,
                            &testNwGlobContext.paStats,
                            clearPAStats);
            clearPAStats = 0;
            getPAStats = 0;
        }
        numRxPkts = nwal_pollPkt( testNwGlobContext.nwalInstHandle,
                                     nwal_POLL_APP_MANAGED_PKT_Q,
                                     0,
                                     NWAL_MAX_RX_PKT_THRESHOLD,
                                     testNwLocContext.pktQHandle,
                                     NULL);

#ifdef TEST_NWAL_EXTRA_DEBUG
        if(numRxPkts)
        {
            System_printf("Core: %d Received %d number of packets from network \n",
                           DNUM,numRxPkts);
            System_printf("Core: %d numPktSent: %d numL2PktsRecvd:%d numL3PktsRecvd:%d,numL4PktsRecvd:%d\n",
                           DNUM,testNwLocContext.numPktSent,testNwLocContext.numL2PktsRecvd,
                           testNwLocContext.numL3PktsRecvd,testNwLocContext.numL4PktsRecvd);

            System_printf("Core: %d ERR Stats txErrDrop: %d numUnhandledEthType:%d numUnhandledL4Type:%d,invL3Offset:%d\n",
                           DNUM,testNwLocContext.txErrDrop,testNwLocContext.numUnhandledEthType,
                           testNwLocContext.numUnhandledL4Type,testNwLocContext.invL3Offset);

            nwal_SystemFlush();
        }
#endif
    }
}
//#define TEST_ENABLE_EXTENDED_CHECK
/********************************************************************
 * FUNCTION PURPOSE: Packet processing from network
 ********************************************************************
 * DESCRIPTION: Packet processing from network
 ********************************************************************/
testRouteTask(Qmss_QueueHnd   rxQ)
{
    pasahoLongInfo_t*   pinfo = NULL;
    Cppi_HostDesc*      pHd;
    nwalMacAddr_t       tmpMac[NWAL_MAC_ADDR_SIZE];
    nwalIpAddr_t        tmpIpAddr;
    uint16_t            srcPort;
    uint16_t            dstPort;
    uint8_t*            pDataBuff;
    uint32_t            dataLen;
    uint16_t            tmpVal;

    while(1)
    {
        if(poll_stats)
        {
            poll_stats=0;
#if 0
            CSL_CPSW_nGF_getStats(stats);
            if(stats[0].RxCRCErrors || stats[1].RxCRCErrors)
            {
                System_printf("ERROR Received packets with CRC errors : Port 1 CRC error count: %d, Port 2 CRC error count:%d\n", 
                               stats[0].RxCRCErrors,
                               stats[1].RxCRCErrors);
                System_flush();
                while(1);
            }
#endif
        }

        if(!(Qmss_getQueueEntryCount (rxQ)))
        {
            continue;
        } 
        fw_startProfile();
        pHd = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop (rxQ));
        if(pHd == NULL) continue;
#ifdef C66x_MASTER
#ifdef NWAL_TEST_DESC_GLOB_MEM 
        /* Invalidate the cache */
        Osal_invalidateCache(pHd,NWAL_CACHE_LINE_SIZE);
#endif
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
        Osal_invalidateCache((void *)pHd->buffPtr,pHd->buffLen);
#endif
#else
     Osal_invalidateCache(pHd,NWAL_CACHE_LINE_SIZE);
     Osal_invalidateCache((void *)pHd->buffPtr,pHd->buffLen);
#endif


        Cppi_getData (Cppi_DescType_HOST, 
                     (Cppi_Desc*)pHd, 
                     &pDataBuff, 
                     &dataLen);

        /* Get the extended information*/
        pinfo =  NULL;
        pinfo = nwal_mGetProtoInfo((Ti_Pkt*)pHd);
        fw_startProfile();
        
        /* TBD: Error Checks for IP and L4 checksum */
        /* Flip the MAC source and Destination */
        memcpy((uint8_t*)tmpMac,
               &pDataBuff[NWAL_TEST_ETH_OFFSET_DEST_MAC],
               NWAL_MAC_ADDR_SIZE);
        memcpy(&pDataBuff[NWAL_TEST_ETH_OFFSET_DEST_MAC],
               &pDataBuff[NWAL_TEST_ETH_OFFSET_SRC_MAC],
               NWAL_MAC_ADDR_SIZE);
        memcpy(&pDataBuff[NWAL_TEST_ETH_OFFSET_SRC_MAC],
               (uint8_t*)tmpMac,
               NWAL_MAC_ADDR_SIZE);
        tmpVal = testNwalRead16bits_m(pDataBuff,
                                      NWAL_TEST_ETH_VLAN_TAG);
#ifdef TEST_ENABLE_EXTENDED_CHECK
        if(tmpVal == NWAL_TEST_ETH_TYPE_IP)
        {
#endif
            /* Flip IPv4 Source and Destination */
            memcpy(&tmpIpAddr.ipv4,
                   &pDataBuff[nwal_mGetL3OffBytes(pinfo) + NWAL_TEST_IPV4_OFFSET_SRC_ADDR],
                   NWAL_IPV4_ADDR_SIZE);

            memcpy(&pDataBuff[nwal_mGetL3OffBytes(pinfo) + NWAL_TEST_IPV4_OFFSET_SRC_ADDR],
                   &pDataBuff[nwal_mGetL3OffBytes(pinfo) + NWAL_TEST_IPV4_OFFSET_DST_ADDR],
                   NWAL_IPV4_ADDR_SIZE);

            memcpy(&pDataBuff[nwal_mGetL3OffBytes(pinfo) + NWAL_TEST_IPV4_OFFSET_DST_ADDR],
                   &tmpIpAddr.ipv4,
                   NWAL_IPV4_ADDR_SIZE);

            /* TBD: Leave checksum unchanged */
#ifdef TEST_ENABLE_EXTENDED_CHECK
        }
        else
        {
            testNwLocContext.numUnhandledEthType++;
            return;
        }
#endif        
        /* TBD: Always assumed to have PS Info */
#ifdef TEST_ENABLE_EXTENDED_CHECK
#if defined(DEVICE_K2L) || defined(DEVICE_K2E) || defined(SOC_K2E) || defined(SOC_K2L)
        if(nwal_mGetL4ProtoType2(pinfo) == NWAL_TEST_IPV4_UDP)
#else
        if(nwal_mGetL4ProtoType(pinfo) == NWAL_TEST_IPV4_UDP)
#endif
        {
#endif

            /* Flip source and destination UDP port */
            srcPort = testNwalRead16bits_m(pDataBuff,
                                 (nwal_mGetL4Offset(pinfo) + NWAL_TEST_UDP_OFFSET_SRC_PORT));
            dstPort = testNwalRead16bits_m(pDataBuff,
                                 (nwal_mGetL4Offset(pinfo) + NWAL_TEST_UDP_OFFSET_DEST_PORT));

            testNwalWrite16bits_m(pDataBuff,
                                  (nwal_mGetL4Offset(pinfo) + NWAL_TEST_UDP_OFFSET_SRC_PORT),
                                  dstPort);
            testNwalWrite16bits_m(pDataBuff,
                                  (nwal_mGetL4Offset(pinfo) + NWAL_TEST_UDP_OFFSET_DEST_PORT),
                                  srcPort);

#ifdef TEST_ENABLE_EXTENDED_CHECK
        }
        else
        {
            testNwLocContext.numUnhandledL4Type++;
            while(1);
        }
#endif
        
         /* Make sure there is no control info.  */
         Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)pHd, 0);
#ifndef NWAL_PKT_LOOPBACK
        /* Remove the 4 CRC bytes from the switch */
        dataLen = dataLen -4;
#endif
        Cppi_setData (Cppi_DescType_HOST,
                      (Cppi_Desc *) pHd, 
                      pDataBuff,
                      dataLen); 
        Cppi_setPacketLen (Cppi_DescType_HOST, 
                          (Cppi_Desc*)pHd, 
                          dataLen);

        /* TBD: Assumes single segment */
        /* TBD: Assumes single segment */
        nwal_mCmdSetPort((Ti_Pkt*)pHd,
                          &testNwLocContext.txPsCmdInfo,
                          nwal_mGetRxEmacPort(pinfo));

#ifdef C66x_MASTER
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
        /* Write back the cache */
        Osal_writeBackCache((void *)pDataBuff,dataLen);
#endif
#ifdef NWAL_TEST_DESC_GLOB_MEM
        Osal_writeBackCache((void *)pHd,NWAL_DESC_SIZE);
#endif
#else
    Osal_writeBackCache((void *)pDataBuff,dataLen);
    Osal_writeBackCache((void *)pHd,NWAL_DESC_SIZE);
#endif
         /* Send the packet out to transmit Q*/
        Qmss_queuePushDescSize (testNwLocContext.txPsCmdInfo.txQueue,
                                pHd,
                                NWAL_DESC_SIZE);
#ifdef TEST_NWAL_EXTRA_DEBUG        
        System_printf("Core: %d Received 1 packets from network \n",
                       DNUM);
        System_printf("Core: %d numPktSent: %d numL2PktsRecvd:%d numL3PktsRecvd:%d,numL4PktsRecvd:%d\n",
                       DNUM,testNwLocContext.numPktSent,testNwLocContext.numL2PktsRecvd,
                       testNwLocContext.numL3PktsRecvd,testNwLocContext.numL4PktsRecvd);

        System_printf("Core: %d ERR Stats txErrDrop: %d numUnhandledEthType:%d numUnhandledL4Type:%d,invL3Offset:%d\n",
                       DNUM,testNwLocContext.txErrDrop,testNwLocContext.numUnhandledEthType,
                       testNwLocContext.numUnhandledL4Type,testNwLocContext.invL3Offset);

        nwal_SystemFlush();
#endif
        fw_endProfile();
        
    }
}

/*****************************************************************************
 * FUNCTION PURPOSE:Top level function being executed in the thread for NWAL
 *                  test
 *****************************************************************************
 * DESCRIPTION: The function will be called to send packets out
 *****************************************************************************/
#ifdef __LINUX_USER_SPACE
 void  nwalTest(int id)
#else
void  nwalTest(UArg a0, UArg a1)
#endif
{
    uint8_t   isAlloc;
    nwalTxPktInfo_t    txPktInfo;

    if(DNUM)
    {
        while(1)
        {
          Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
          /* Invalidate the Global context */
          if(testNwGlobContext.state == TEST_NW_CXT_GLOB_ACTIVE)
          {
              break;
          }
        }
    }

    /* Open the general purpose queue for polling the packets */

    /* Open next available queue from the pool */
    testNwLocContext.pktQHandle =
        Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                        (TEST_NWAL_BASE_REM_FP_RX_PKT_QUEUE+DNUM),
                         &isAlloc);
    if(testNwLocContext.pktQHandle < 0)
    {
        return;
    }
#ifdef C66x_MASTER
    if(!DNUM)
    {
        nwalMacHandle = testAddMac(&nwalMacParam);
        if(nwalMacHandle == NULL)
        {
            System_printf("CORE: %d Error Creating MAC handle \n",DNUM);
            nwal_SystemFlush();
            while(1);
        }

        nwalIpHandle = testAddIp(&nwalIpParam,nwalMacHandle);
        if(nwalIpHandle == NULL)
        {
            System_printf("CORE: %d Error Creating IP handle \n",DNUM);
            nwal_SystemFlush();
            while(1);
        }

        /* Create UDP Fast Path termination from NetCP */
        if(testAddRemFPL4() != nwal_TRUE)
        {
            System_printf("CORE: %d Error adding remote FP L4 connections \n",DNUM);
            nwal_SystemFlush();
            while(1);
        }
    }
#endif
    /* Create a default PS command label with no NetCP offload */
    memset(&txPktInfo,0,sizeof(nwalTxPktInfo_t));
    txPktInfo.txFlag1 = (NWAL_TX_FLAG1_META_DATA_VALID);
    txPktInfo.lpbackPass = nwalPktLoopBack;
    txPktInfo.enetPort = 1;/* By default */
    if(nwal_initPSCmdInfo(testNwGlobContext.nwalInstHandle,
                          &txPktInfo,
                          &testNwLocContext.txPsCmdInfo) != nwal_OK)
    {
        while(1);
    }
#if 0
    testNwalPoll();
#else
    testRouteTask(testNwLocContext.pktQHandle);
#endif
    nwal_SystemFlush();
}
