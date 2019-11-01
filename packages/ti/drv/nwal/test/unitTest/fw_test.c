/******************************************************************************
 * FILE PURPOSE:  File containing NWAL test routines
 ******************************************************************************
 * FILE NAME:   fw_test.c
 *
 * DESCRIPTION: File containing NWAL test routines
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2015
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
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_pa_ss.h>

void testNwalPoll (void );


/* Memory for below definition to be uncached global visible across all cores */
#pragma DATA_SECTION (testNwGlobContext, ".testNwGlobContext");
#pragma DATA_ALIGN(testNwGlobContext, 128)
testNwGlobContext_t     testNwGlobContext;

/* Will get loaded to local memory by default */
#ifdef __ARMv7
__thread testNwLocContext_t     testNwLocContext;
extern nwalLocCfg_t             nwalLocCfg;
#else
testNwLocContext_t              testNwLocContext;
#endif

#ifdef __ARMv7
__thread int core_id;
#define DNUM    core_id
#else
extern cregister  volatile unsigned int DNUM;
#endif

volatile uint32_t clearPAStats = 0;
volatile uint32_t getPAStats = 0;
Ti_Pkt*     pDmEncPkt = NULL;
uint32_t    nwalPktLoopBack=1;
uint32_t    testCount = 0;
uint32_t    passCount = 0;
uint32_t    rcvd_AppID = 0;

/*****************************************************************************
 * DEFINITION: Test packet for NWAL
 *****************************************************************************/
#pragma DATA_SECTION(testPkt, ".testPkts")
#pragma DATA_ALIGN(testPkt, 128)

static uint8_t testPkt[] = {

  /* MAC header */
  0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
  0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
  0x08, 0x00,

  /* IP header */
  0x45, 0x00,
  0x00, 0x6c,  /* Length (including this header) */
  0x00, 0x00, 0x00, 0x00, 0x01, 0x11,
  0xb1, 0x76,  /* Header checksum */
  0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04,

  /* UDP header */
  0x05, 0x55, 0x05, 0x55,
  0x00, 0x58,  /* Length, including this header */
  0x0, 0x0,  /* Header checksum. Disabled for Raw packets */

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

static uint8_t testPktQos[] = {

  /* MAC header */
  0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
  0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
  0x81, 0x00, 0x88, 0x88, /*802.1 Q tag, vlan priority of 4 */
  0x08, 0x00,

  /* IP header */
  0x45, 0x0c,           /* DSCP value of 3 */
  0x00, 0x6c,  /* Length (including this header) */
  0x00, 0x00, 0x00, 0x00, 0x01, 0x11,
  0xb1, 0x76,  /* Header checksum */
  0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04,

  /* UDP header */
  0x05, 0x55, 0x05, 0x55,
  0x00, 0x58,  /* Length, including this header */
  0x0, 0x0,  /* Header checksum. Disabled for Raw packets */

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

#define TEST_PKT_IP_OFFSET_BYTES        14
#define TEST_PKT_UDP_OFFSET_BYTES       34
#define TEST_PKT_PLOAD_OFFSET_BYTES     42
#define TEST_PKT_UDP_HDR_LEN            8
/* Offsets to length fields */
#define TEST_PKT_OFFSET_IP_LEN      16
#define TEST_PKT_OFFSET_UDP_LEN     38

#define TEST_PKT_LEN                TEST_PAYLOAD_LEN + TEST_PKT_PLOAD_OFFSET_BYTES

/* Workaround for PA LLD version incompatibility */
#define PA_VERSION_INCOMP_WAROUND 1
#define TEST_NWAL_MAX_RETRY_SLAVE_CORE_STATE    10

/* All packets in this test use a single destination address ethernet
 * routing for the L2 lookup in PA */
static nwalMacParam_t   nwalMacParam[]= {
    0,      /* validParams */
    0,      /* ifNum */
    0,      /* vlanId      */
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Local mac */
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Remote mac */
    NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
    NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
    CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
    QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
    0
};

static nwalIpParam_t    nwalIpParam[]= {
    0,
    pa_IPV4,      /* IP Type */
    { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Dest IP */
    { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /*Source IP */
    { 0x1,17,0,0},/* IP Options */
    NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
    NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
    CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
    QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
    0
};

static nwalIpParam_t    nwalIpv6Param[]= {
    0,
    pa_IPV6,      /* IP Type */
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 },  /* Dest IP */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /*Source IP */
    { 0x1,17,0,0},/* IP Options */
    NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
    NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
    CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
    QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
    0
};


static nwalRxConnCfg_t nwalRxConnCfg = {
    0,      /* inHandle */
    0,
    0x0555, /* appProto */
    0, /* rxCoreId */
    NWAL_MATCH_ACTION_HOST,                      /* Terminate at host in case of match */
    CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
    QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
    0
};
static nwalTxConnCfg_t nwalTxConnCfg = {
    0,      /* outHandle */
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
    { 0,0,0x2,0},   /* mac options */
    0,
    pa_IPV4,      /* IP Type */
    1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,  /* Destination IP Address */
    0x1,17,0,0,/* IP Options */
    0x0555 /* appProto */
};

static nwalTxConnCfg_t nwalTxConnIpv6Cfg = {
    0,      /* outHandle */
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
    { 0,0,0x2,0},   /* mac options */
    0,
    pa_IPV6,      /* IP Type */
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 ,  /* Destination IP Address */
    0x1,17,0,0,/* IP Options */
    0x0555 /* appProto */
};

#ifdef NWAL_ENABLE_SA
static uint8_t nwalAuthKey[36] =
        {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
         0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
         0x20, 0x21, 0x22, 0x23 };
static uint8_t nwalEncrKey[36] =
        {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
         0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
         0x30, 0x31, 0x32, 0x33 };


static nwalSaIpSecParam_t   nwalSaIpSecESPParam[]={
    {   /* Inbound */
        0,/* validParams */
        nwal_SA_MODE_TUNNEL,
        64,/* replayWindow */
        NWAL_SA_DIR_INBOUND,
        0,
        0,
        NWAL_SA_AALG_HMAC_SHA1,
        NWAL_SA_EALG_AES_CTR,
        { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
        12, /* macSize */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
    },
    {   /* Outbound */
        0,/* validParams */
        nwal_SA_MODE_TUNNEL,
        64,/* replayWindow */
        NWAL_SA_DIR_OUTBOUND,
        0,
        0,
        NWAL_SA_AALG_HMAC_SHA1,
        NWAL_SA_EALG_AES_CTR,
        { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
        12, /* macSize */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
    },
};
static nwalSaIpSecParam_t   nwalSaIpSecAHParam[]={
    {   /* Inbound */
        0,/* validParams */
        nwal_SA_MODE_TUNNEL,
        64,/* replayWindow */
        NWAL_SA_DIR_INBOUND,
        0,
        0,
        NWAL_SA_AALG_HMAC_SHA1,
        NWAL_SA_EALG_NULL,
        { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
        12, /* macSize */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
    },
    {   /* Outbound */
        0,/* validParams */
        nwal_SA_MODE_TUNNEL,
        64,/* replayWindow */
        NWAL_SA_DIR_OUTBOUND,
        0,
        0,
        NWAL_SA_AALG_HMAC_SHA1,
        NWAL_SA_EALG_NULL,
        { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
        12, /* macSize */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
        },
};
static nwalSaIpSecParam_t   nwalSaIpSecTransESPParam[]={
    {   /* Inbound */
        0,/* validParams */
        nwal_SA_MODE_TRANSPORT,
        64,/* replayWindow */
        NWAL_SA_DIR_INBOUND,
        0,
        0,
        NWAL_SA_AALG_HMAC_SHA1,
        NWAL_SA_EALG_AES_CTR,
        { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
        12, /* macSize */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
    },
    {   /* Outbound */
        0,/* validParams */
        nwal_SA_MODE_TRANSPORT,
        64,/* replayWindow */
        NWAL_SA_DIR_OUTBOUND,
        0,
        0,
        NWAL_SA_AALG_HMAC_SHA1,
        NWAL_SA_EALG_AES_CTR,
        { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
        12, /* macSize */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
        },
};
static nwalSaIpSecParam_t   nwalSaIpSecTransAHParam[]={
    {   /* Inbound */
        0,/* validParams NWAL_SA_INFO_VALID_PARAM_ESN */
        nwal_SA_MODE_TRANSPORT,
        64,/* replayWindow */
        NWAL_SA_DIR_INBOUND,
        0,
        0,
        NWAL_SA_AALG_AES_XCBC, /*NWAL_SA_AALG_HMAC_MD5 ,NWAL_SA_AALG_HMAC_SHA1 */
        NWAL_SA_EALG_NULL,
        { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
        12, /* macSize */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
},
    {   /* Outbound */
        0,/* validParams NWAL_SA_INFO_VALID_PARAM_ESN */
        nwal_SA_MODE_TRANSPORT,
        64,/* replayWindow */
        NWAL_SA_DIR_OUTBOUND,
        0,
        0,
        NWAL_SA_AALG_AES_XCBC,/* NWAL_SA_AALG_HMAC_MD5 ,NWAL_SA_AALG_HMAC_SHA1 */
        NWAL_SA_EALG_NULL,
        { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* remMacAddr: Same as local*/
        12, /* macSize */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
        },
};
static nwalSecKeyParams_t nwalSaDmKeyParam ={
    16, /* encKeySize: 16 bytes Encryption Key and 4 bytes Salt : 24 bytes:NWAL_SA_EALG_3DES_CBC and 0 bytes Salt*/
    20, /* macKeySize: 16 bytes NWAL_SA_AALG_HMAC_MD5 */
    NULL,
    NULL,
};
static nwalSecKeyParams_t nwalSaEspKeyParam ={
    20, /* encKeySize: 16 bytes Encryption Key and 4 bytes Salt : 24 bytes:NWAL_SA_EALG_DES_CBC and 0 bytes Salt*/
    20, /* macKeySize: 16 bytes NWAL_SA_AALG_HMAC_MD5 */
    NULL,
    NULL,
};

static nwalSecKeyParams_t nwalSaAHKeyParam ={
    0, /* encKeySize: 16 bytes Encryption Key and 4 bytes Salt : 24 bytes:NWAL_SA_EALG_DES_CBC and 0 bytes Salt*/
    20, /* macKeySize: 16 bytes NWAL_SA_AALG_HMAC_MD5 */
    NULL,
    NULL,
};
static nwalSecKeyParams_t nwalSaTransAHKeyParam ={
    0, /* encKeySize: 16 bytes Encryption Key and 4 bytes Salt : 24 bytes:NWAL_SA_EALG_DES_CBC and 0 bytes Salt*/
    16, /* macKeySize: 16 bytes NWAL_SA_AALG_HMAC_MD5,20:NWAL_SA_AALG_HMAC_SHA1  */
    NULL,
    NULL,
};
static nwalSaIpSecId_t nwalSaIpSecESPId ={
    0x44444444,
    { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Src IP */
    { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Dst IP */
    nwal_IpSecProtoESP
};

static nwalSaIpSecId_t nwalSaIpSecAHId ={
    0x55555555,
    { 1, 2, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Src IP */
    { 1, 2, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Dst IP */
    nwal_IpSecProtoAH
};

static nwalSaIpSecId_t nwalSaIpv6IpSecESPId ={
    0x66666666,
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 },  /* Src IP */
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 },  /* Dst IP */
    nwal_IpSecProtoESP
};

static nwalSaIpSecId_t nwalSaIpv6IpSecAHId ={
    0x77777777,
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 },  /* Src IP */
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 },  /* Dst IP */
    nwal_IpSecProtoAH
};
static nwalSaIpSecId_t nwalSaIpSecTransportESPId ={
    0x88888888,
    { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Src IP */
    { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Dst IP */
    nwal_IpSecProtoESP
};

static nwalSaIpSecId_t nwalSaIpSecTransportAHId ={
    0x99999999,
    { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Src IP */
    { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Dst IP */
    nwal_IpSecProtoAH
};

static nwalSecPolParams_t  nwalPolParams[2]={
    {
        0,  /* handle */
        0,  /* valid params */
        NWAL_SA_DIR_INBOUND,
        4,      /* IP Type */
        1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,  /* dst */
        1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,  /* src */
        {0x1,17,0,0 },/* IP Options */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
    },
    {
        0,  /* handle */
        0,  /* valid params */
        NWAL_SA_DIR_OUTBOUND,
        4,      /* IP Type */
        1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,  /* dst */
        1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,  /* src */
        { 0x1,17,0,0 },/* IP Options */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
    },
};

static nwalSecPolParams_t  nwalPolIPv6Params[2]={
    {
        0,  /* handle */
        0,  /* valid params */
        NWAL_SA_DIR_INBOUND,
        6,      /* IP Type */
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 ,  /* dst */
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 ,  /* src */
        {0x1,17,0,0 },/* IP Options */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
    },
    {
        0,  /* handle */
        0,  /* valid params */
        NWAL_SA_DIR_OUTBOUND,
        6,      /* IP Type */
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 ,  /* dst */
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,14, 15, 16 ,  /* src */
        { 0x1,17,0,0 },/* IP Options */
        NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE,       /* Continue parsing to next route for match */
        NWAL_NEXT_ROUTE_FAIL_ACTION_HOST,            /* For next route fail action by default is route to host */
        CPPI_PARAM_NOT_SPECIFIED,                    /* Use default flow configured to NWAL  if packet is routed to host */
        QMSS_PARAM_NOT_SPECIFIED,                    /* Use default queue configured to NWAL if packet is routed to host */
        0
    },
};

static nwalDmSAParams_t   nwalDmSaParam[]={
    {   /* Decryption Channel */
        NWAL_DM_CHAN_DECRYPT,
        64,/* replayWindow */
        NWAL_SA_AALG_HMAC_SHA1,
        NWAL_SA_EALG_AES_CBC,
        12, /* macSize */
        0, /* aadSize */
        nwal_FALSE, /* enc1st */
    },
    {     /* Encryption Channel */
        NWAL_DM_CHAN_ENCRYPT,
        64,/* replayWindow */
        NWAL_SA_AALG_HMAC_SHA1,
        NWAL_SA_EALG_AES_CBC,
        12, /* macSize */
        0, /* aadSize */
        nwal_TRUE, /* enc1st */
    },
};
#endif /* # NWAL_ENABLE_SA */



#ifndef __ARMv7
extern cregister volatile unsigned int TSCL;
#endif

#ifdef __ARMv7
__thread nwal_Handle         nwalMacHandle;
__thread nwal_Handle         nwalIpHandle;
__thread nwal_Handle         nwalIpv6Handle;
__thread nwal_Handle         udpHandle;
__thread nwal_Handle         nwalIpv6UdpHandle;
__thread nwal_Handle         nwalSecAssocESPInHandle;
__thread nwal_Handle         nwalSecAssocESPOutHandle;
__thread nwal_Handle         nwalESPPolInHandle;
__thread nwal_Handle         nwalESPPolOutHandle;
__thread nwal_Handle         saESPUdpHandle;

__thread nwal_Handle         nwalSecAssocAHInHandle;
__thread nwal_Handle         nwalSecAssocAHOutHandle;
__thread nwal_Handle         nwalAHPolInHandle;
__thread nwal_Handle         nwalAHPolOutHandle;
__thread nwal_Handle         saAHUdpHandle;

__thread nwal_Handle         nwalIpv6SecAssocESPInHandle;
__thread nwal_Handle         nwalIPv6SecAssocESPOutHandle;
__thread nwal_Handle         nwalIpv6ESPPolInHandle;
__thread nwal_Handle         nwalIpv6ESPPolOutHandle;
__thread nwal_Handle         saIpv6ESPUdpHandle;

__thread nwal_Handle         nwalIpv6SecAssocAHInHandle;
__thread nwal_Handle         nwalIpv6SecAssocAHOutHandle;
__thread nwal_Handle         nwalIpv6AHPolInHandle;
__thread nwal_Handle         nwalIpv6AHPolOutHandle;
__thread nwal_Handle         saIpv6AHUdpHandle;

__thread nwal_Handle         nwalSecAssocTransESPInHandle;
__thread nwal_Handle         nwalSecAssocTransESPOutHandle;
__thread nwal_Handle         nwalESPTransPolInHandle;
__thread nwal_Handle         nwalESPTransPolOutHandle;
__thread nwal_Handle         saESPTransUdpHandle;

__thread nwal_Handle         nwalSecAssocTransAHInHandle;
__thread nwal_Handle         nwalSecAssocTransAHOutHandle;
__thread nwal_Handle         nwalAHTransPolInHandle;
__thread nwal_Handle         nwalAHTransPolOutHandle;
__thread nwal_Handle         saAHTransUdpHandle;

__thread nwal_Handle         nwalDmEncSaHandle;
__thread nwal_Handle         nwalDmDecSaHandle;
__thread uint32_t dmAuthTag[32];

__thread unsigned int fw_nwal_pollPkt_sum = 0;
__thread unsigned int fw_nwal_PollCount_sum = 0;
__thread unsigned int fw_nwal_PollPktCount_sum = 0;
__thread unsigned int fw_nwal_PollPktCB_sum = 0;

__thread unsigned int fw_dm_send_count=0;
__thread unsigned int fw_dm_poll_count=0;

__thread unsigned int fw_nwal_sendDM_start,fw_nwal_sendDM_end;
__thread unsigned int fw_nwal_sendDM_prof;
#else
nwal_Handle         nwalMacHandle;
nwal_Handle         nwalIpHandle;
nwal_Handle         nwalIpv6Handle;
nwal_Handle         udpHandle;
nwal_Handle         nwalIpv6UdpHandle;

nwal_Handle         nwalSecAssocESPInHandle;
nwal_Handle         nwalSecAssocESPOutHandle;
nwal_Handle         nwalESPPolInHandle;
nwal_Handle         nwalESPPolOutHandle;
nwal_Handle         saESPUdpHandle;

nwal_Handle         nwalSecAssocAHInHandle;
nwal_Handle         nwalSecAssocAHOutHandle;
nwal_Handle         nwalAHPolInHandle;
nwal_Handle         nwalAHPolOutHandle;
nwal_Handle         saAHUdpHandle;

nwal_Handle         nwalIpv6SecAssocESPInHandle;
nwal_Handle         nwalIPv6SecAssocESPOutHandle;
nwal_Handle         nwalIpv6ESPPolInHandle;
nwal_Handle         nwalIpv6ESPPolOutHandle;
nwal_Handle         saIpv6ESPUdpHandle;

nwal_Handle         nwalIpv6SecAssocAHInHandle;
nwal_Handle         nwalIpv6SecAssocAHOutHandle;
nwal_Handle         nwalIpv6AHPolInHandle;
nwal_Handle         nwalIpv6AHPolOutHandle;
nwal_Handle         saIpv6AHUdpHandle;

nwal_Handle         nwalSecAssocTransESPInHandle;
nwal_Handle         nwalSecAssocTransESPOutHandle;
nwal_Handle         nwalESPTransPolInHandle;
nwal_Handle         nwalESPTransPolOutHandle;
nwal_Handle         saESPTransUdpHandle;

nwal_Handle         nwalSecAssocTransAHInHandle;
nwal_Handle         nwalSecAssocTransAHOutHandle;
nwal_Handle         nwalAHTransPolInHandle;
nwal_Handle         nwalAHTransPolOutHandle;
nwal_Handle         saAHTransUdpHandle;

nwal_Handle         nwalDmEncSaHandle;
nwal_Handle         nwalDmDecSaHandle;

uint32_t dmAuthTag[32];
unsigned int fw_nwal_pollPkt_sum = 0;
unsigned int fw_nwal_PollCount_sum = 0;
unsigned int fw_nwal_PollPktCount_sum = 0;
unsigned int fw_nwal_PollPktCB_sum = 0;

unsigned int fw_dm_send_count=0;
unsigned int fw_dm_poll_count=0;

unsigned int fw_nwal_sendDM_start,fw_nwal_sendDM_end;
unsigned int fw_nwal_sendDM_prof;
#endif

#define NWAL_LIB_ENABLE_PROFILE
#ifdef NWAL_LIB_ENABLE_PROFILE
extern unsigned int nwal_sendDM_prof1_sum;
extern unsigned int nwal_sendDM_prof2_sum;
extern unsigned int nwal_sendDM_prof3_sum;
extern unsigned int nwal_sendDM_prof4_sum;
extern unsigned int nwal_sendDM_prof5_sum;
extern unsigned int nwal_sendDM_sum;
extern unsigned int nwal_sendDM_count_sum ;
#endif


//#define TEST_NWAL_DUMP_PKT  1
#ifdef TEST_NWAL_DUMP_PKT
 /****************************************************************************/
static void  fw_nwal_dump_buf_32bit
(
    uint8_t*                      buf,
    uint32_t                      buf_length
)
{
    uint8_t                       count = 0;
    uint16_t                      dump_size;
    uint32_t*                     tmp_buf;
    uint8_t                       row_count;

    //if(first > 20) return;

    //first++;

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
                printf("NWAL *: Internal Error in fw_nwal_dump_buf_32bit().Row Count: %d \n",
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
 *  FUNCTION PURPOSE: Internal NWAL function. Currently not explosed
 *                    through API
 ********************************************************************
 ********************************************************************/
#ifdef NWAL_ENABLE_SA
nwal_RetValue nwal_getSecAssocStats(nwal_Inst          nwalInst,
                                    nwal_Handle        nwalSecAssocHandle,
                                    Sa_IpsecStats_t*   pSaIpsecStats);
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


/**************************************************************************************
 * FUNCTION PURPOSE: Compute ipv4 psudo checksum
 **************************************************************************************
 * DESCRIPTION: Compute ipv4 psudo checksum
 **************************************************************************************/
uint16_t test_utilGetIpv4PsudoChkSum (uint8_t *data, uint16_t payloadLen)
{
  uint16_t psudo_chksum;

  psudo_chksum = test_utilOnesCompChkSum (&data[12], 4);
  psudo_chksum = test_utilOnesComplementAdd(psudo_chksum, (uint16_t) data[9]);
  psudo_chksum = test_utilOnesComplementAdd(psudo_chksum, payloadLen);

  return (psudo_chksum);

} /* utilGetIpv4PsudoChkSum */

/********************************************************************
 *  FUNCTION PURPOSE: Print Test Case
 ********************************************************************
 ********************************************************************/
void fw_logTest (nwal_Bool_t begin,
                 nwal_Bool_t fail,
                 const char* logString)
{
    if(begin)
    {
        testCount++;
        System_printf("CORE:%d NWAL UNIT TEST:%d ***** %s BEGIN *****\n",DNUM,testCount,logString);
    }
    else
    {
        if(fail)
        {
            System_printf("CORE:%d NWAL UNIT TEST:%d ***** %s END STATUS: FAILED *****\n\n",
                           DNUM,testCount,logString);
        }
        else
        {
            passCount++;
            System_printf("CORE:%d NWAL UNIT TEST:%d ***** %s END STATUS: PASSED *****\n\n",
                           DNUM,testCount,logString);
        }
    }
    nwal_SystemFlush();
}

/*****************************************************************************
 * Function: Utility function a cycle clock
 ****************************************************************************/
#ifndef __ARMv7 /* Remove this ifdef if you want to use PMU (__asm__ below) */
static uint32_t utilReadTime32(void)
{
    uint32_t timeVal;

#if defined (_TMS320C6X)
    timeVal = TSCL;
#else
    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(timeVal));
#endif
    return timeVal;
}
#endif

/*****************************************************************************
 * FUNCTION PURPOSE: Delay function
 *****************************************************************************
 * DESCRIPTION: The function will create NWAL instance which is prerequiite for
 *              any call to NWAL
 *****************************************************************************/
void utilCycleDelay (int iCount)
{
  if (iCount <= 0)
    return;

  /* If there is need for ARM-RTOS, adjust #ifdef to use the PMU
   * via the __asm__ in utilReadTime32 and use same code as
   * c66 */
#ifndef __ARMv7
  {
    uint32_t start = utilReadTime32();
    uint32_t count = (uint32_t)iCount;

    while ((utilReadTime32() - start) < count);
  }
#else
  {
    uint32_t sat;
    /* This code is for user-mode where PMU is inaccessible */
    for (sat=0; sat<(uint32_t)iCount; sat++);
  }
#endif
}

volatile uint8_t* pSrc;
volatile uint8_t* pDst;
volatile uint16_t count=0;

/*****************************************************************************
 * FUNCTION PURPOSE: Delay function
 *****************************************************************************
 * DESCRIPTION: The function will create NWAL instance which is prerequiite for
 *              any call to NWAL
 *****************************************************************************/
nwal_Bool_t testNwalBufCmp (uint8_t* pOrigSrc, uint8_t* pOrigDst,uint16_t len)
{
  pSrc = pOrigSrc;
  pDst = pOrigDst;
  while(count < len)
  {
      if(*pSrc != *pDst)
      {
          return nwal_FALSE;
      }

      pSrc++;
      pDst++;
      count++;
  }
  count=0;
  return nwal_TRUE;
}

/******************************************************************************
 * FUNCTION PURPOSE: Write 16 bit value into 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Writes 16 bit value into 16 bit word.  No assumptions
 *
 * void nwalWrite16bits_m (
 *    uint8_t *base,    - Base of byte array
 *    uint16_t byteOffset, - byte offset to write; assumed to be even
 *    uint16_t val)        - 16 bit val
 *
 *****************************************************************************/
void testNwalRead16bits_m (uint8_t *base,
                           uint16_t byteOffset,
                           uint16_t* pVal)
{
  char *wptr = ((char *)base + byteOffset);

  *pVal = wptr[0];
  *pVal = (*pVal << 8);
  *pVal = (*pVal | wptr[1]);

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

void testNwalInitPollProfile ()
{
    Osal_cache_op_measure_reset();
    fw_nwal_pollPkt_sum = 0;
    fw_nwal_PollCount_sum = 0;
    fw_nwal_PollPktCount_sum = 0;
    fw_nwal_PollPktCB_sum = 0;
}

void testNwalDispCacheCycles (unsigned int numPkts)
{
#ifdef NWAL_ENABLE_PROFILE
    unsigned int        sumCacheCycles=0;
    unsigned long long  numCacheOps=0;
    sumCacheCycles =Osal_cache_op_measure(&numCacheOps);

    if(numCacheOps && numPkts)
    {
        System_printf("numCacheOps=%d Cumulative Cache Cycle: %d  Cache Cycles /Ops=%d Cache Cycles /Pkt = %d \n",
                       numCacheOps,sumCacheCycles, sumCacheCycles/numCacheOps,
                       numCacheOps? (sumCacheCycles/(numPkts)) : 0);
    }
#endif
    return;
}

void testNwalDispPollProfile ()
{
#ifdef NWAL_ENABLE_PROFILE
    unsigned int        sumCacheCycles=0;
    unsigned long long  numCacheOps=0;
    unsigned int        pollCyclesWoCache;
    sumCacheCycles =Osal_cache_op_measure(&numCacheOps);
    pollCyclesWoCache = ((fw_nwal_pollPkt_sum-fw_nwal_PollPktCB_sum) - sumCacheCycles);

    System_printf("CORE: %d Profile: nwal_poll() Cumulative nwal_poll w/0 cache Cycles %d,Count # of Polls:%d, Count # of packets:%d \n",
                   DNUM,pollCyclesWoCache,fw_nwal_PollCount_sum,fw_nwal_PollPktCount_sum);

    if(fw_nwal_PollCount_sum    &&
       fw_nwal_PollPktCount_sum &&
       fw_nwal_PollPktCount_sum)
    {
        System_printf("CORE: %d Profile Contdd.: Cycles per Poll %d,Per Pkt %d,Callback Per Pkt:%d \n",
                   DNUM,((pollCyclesWoCache)/fw_nwal_PollCount_sum),
                   ((pollCyclesWoCache)/fw_nwal_PollPktCount_sum),
                   fw_nwal_PollPktCB_sum/fw_nwal_PollPktCount_sum);
    }
    testNwalDispCacheCycles(TEST_MAX_BURST);
    nwal_SystemFlush();
#endif
    return;
}


void  fw_nwal_dump_buf
(
    uint8_t*                      buf,
    uint32_t                      buf_length
)
{
    uint8_t                       count = 0;
    uint16_t                      dump_size;
    uint8_t*                     tmp_buf;
    uint8_t                       row_count;
    static uint8_t                first = 0;

    if(first > 2) return;

    first++;

    dump_size = buf_length ;

    tmp_buf = (uint8_t *)(buf);

    printf("NWAL *:  - 16 bit word hex Length: %d Start \n",buf_length);
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
    uint8_t*          pBuf;
    uint8_t           count;
    uint32_t          dataLen;
    uint32_t          errFlag;
    unsigned int      fw_nwal_CB_start = 0;
    unsigned int      fw_nwal_CB_end = 0;
#if TEST_NWAL_DUMP_PKT
            uint32_t            tmpLen;
            uint8_t*            pTmpBuf;
#endif

    fw_nwal_CB_start = fw_read_clock();
    for(count=0;count<numPkts;count++)
    {
        pFreePkt[count] = nwal_TRUE;

        Pktlib_getDataBuffer(pPktInfo[count].pPkt,&pDataBuffer,&dataLen);
   
#if TEST_NWAL_DUMP_PKT
        pTmpBuf = nwalCppi_getPSData(Cppi_DescType_HOST,
                             Cppi_PSLoc_PS_IN_DESC,
                            (Cppi_Desc *)pPktInfo[count].pPkt,
                            &tmpLen);
        printf("NWAL-TEST-RX:LOG Dump of Descriptor BEGIN Address: 0x%x \n",pPktInfo[count].pPkt);
        fw_nwal_dump_buf_32bit((uint8_t *)pPktInfo[count].pPkt,128);
        printf("NWAL-TEST-RX:LOG Dump of Descriptor END \n");

        printf("NWAL-TEST-RX:LOG Dump of PS Data BEGIN \n");
        fw_nwal_dump_buf_32bit(pTmpBuf,tmpLen);
        printf("NWAL-TEST-RX:LOG Dump of PS Data END \n");
        
        printf("NWAL-TEST-RX:LOG Dump of PKT BEGIN \n");
        fw_nwal_dump_buf((uint8_t *)pDataBuffer,dataLen);
        printf("NWAL-TEST-RX:LOG Dump of PKT Data END \n");
#endif
        rcvd_AppID = (uint32_t)(pPktInfo[count].appId);
        if((uint32_t)(pPktInfo[count].appId) == TEST_NWAL_APP_ID_EXCEPTION)
        {
            if((pPktInfo[count].rxFlag1 & NWAL_RX_IP_FRAGMENT_PKT) ==
                NWAL_RX_IP_FRAGMENT_PKT)
            {
                testNwLocContext.numFragPktsRecvd++;
            }
        }
        else if((uint32_t)(pPktInfo[count].appId) == TEST_NWAL_APP_ID_MAC)
        {
            testNwLocContext.numL2PktsRecvd++;
        }
        else if((uint32_t)(pPktInfo[count].appId) == TEST_NWAL_APP_ID_IP)
        {
            testNwLocContext.numL3PktsRecvd++;
        }
        else
        {
            pUdpHeader = pDataBuffer + pPktInfo[count].l4OffBytes + 2;
            testNwalRead16bits_m(pUdpHeader,0,&destPort);
            if((destPort != (TEST_NWAL_BASE_UDP_PORT + DNUM)) &&
               (destPort != (TEST_NWAL_BASE_REM_UDP_PORT + DNUM)) &&
               (destPort != (TEST_NWAL_BASE_AH_UDP_PORT + DNUM)) &&
               (destPort != (TEST_NWAL_BASE_IPV6_ESP_UDP_PORT + DNUM)) &&
               (destPort != (TEST_NWAL_BASE_IPV6_AH_UDP_PORT + DNUM))&&
               (destPort != (TEST_NWAL_BASE_AH_TRANS_UDP_PORT + DNUM))&&
               (destPort != (TEST_NWAL_BASE_ESP_TRANS_UDP_PORT + DNUM)))
            {
                /* Payload comparison failed */
                System_printf("\nCORE: %d ERROR:Unexpected packet received on Dest Port 0x%x \n",
                                DNUM,destPort);
                nwal_SystemFlush();
                goto testNWALRxPktCallbackUpdateBench;
            }

            /* Compare the contents of the packet received */
            pBuf = pDataBuffer + pPktInfo[count].ploadOffBytes;
            if(pPktInfo[count].ploadLen != TEST_PAYLOAD_LEN)
            {
                System_printf("\nCORE: %d Payload length mismatch 0x%x \n",
                                DNUM,destPort);
                nwal_SystemFlush();
                /* Length check failed */
                //while(1);
            }
            if(!(testNwalBufCmp(pBuf,testPayload,TEST_PAYLOAD_LEN)))
            {
                /* Payload comparison failed */
                System_printf("CORE: %d ERROR: Mismatch for the payload received 0x%x\n",
                        DNUM);
                nwal_SystemFlush();
               // while(1);
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
                fw_nwal_dump_buf(pDataBuffer,dataLen);
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
                fw_nwal_dump_buf(pDataBuffer,dataLen);
                nwal_SystemFlush();
                //while(1);
            }

            testNwLocContext.numL4PktsRecvd++;
        }
    }

testNWALRxPktCallbackUpdateBench:
    fw_nwal_CB_end = fw_read_clock();
    fw_nwal_PollPktCB_sum += (fw_nwal_CB_end - fw_nwal_CB_start);
}
/*****************************************************************************
 * FUNCTION PURPOSE: NWAL callback for all Data mode Payload
 *****************************************************************************
 * DESCRIPTION: Callback for receiving Data mode Payload from NetCP after Crypto
 *****************************************************************************/
void testNWALRxDmBack( uint32_t                appCookie,
                       uint16_t                numPkts,
                       nwalDmRxPayloadInfo_t*  pDmRxPayloadInfo,
                       nwal_Bool_t*            pFreePkt)
{
#ifdef NWAL_ENABLE_SA
    uint8_t*                pDataBuffer;
    uint8_t                 count;
    uint32_t                dataLen;
    nwalDmTxPayloadInfo_t   payloadInfo;
    uint8_t                 iv[16]={ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    nwal_RetValue           retValue;

    for(count=0;count<numPkts;count++)
    {

#ifndef TEST_NWAL_DM_NO_LOOPBACK
        Pktlib_getDataBuffer(pDmRxPayloadInfo[count].pPkt,&pDataBuffer,&dataLen);

        if((uint32_t)(pDmRxPayloadInfo[count].appId) == TEST_NWAL_APP_ID_DM_ENC)
        {

                testNwGlobContext.numDMEncRespRecvd++;
                if((uint32_t)pDmRxPayloadInfo[count].appCtxId != TEST_NWAL_CTX_ID_DM_ENC)
                {
                    System_printf("ERROR!!! Incorrect appCtxId,. Expected:0x%x,Received:0x%x \n",
                                   TEST_NWAL_CTX_ID_DM_ENC,pDmRxPayloadInfo[count].appCtxId);
                }
                memcpy(dmAuthTag,
                       pDmRxPayloadInfo[count].pAuthTag,
                       pDmRxPayloadInfo[count].authTagLen);
                pDmEncPkt = pDmRxPayloadInfo[count].pPkt;
                memset(&payloadInfo,0,sizeof(nwalDmTxPayloadInfo_t));
                //memset(iv,0,sizeof(iv));

                payloadInfo.pPkt = pDmRxPayloadInfo[count].pPkt;
                pFreePkt[count] = nwal_FALSE;
                payloadInfo.encOffset = 0;
                payloadInfo.authOffset = 0;
                payloadInfo.encSize = dataLen;
                payloadInfo.authSize = dataLen;
                payloadInfo.pEncIV = NULL;
                if(nwalDmSaParam[1].cipherMode != NWAL_SA_EALG_NULL)
                {
                    payloadInfo.pEncIV = iv;
                }
                payloadInfo.pEncIV = iv;
                payloadInfo.pAuthIV = NULL;
                payloadInfo.aadSize = 0;
                payloadInfo.pAad = NULL;
                payloadInfo.appCtxId = (nwal_AppId)TEST_NWAL_CTX_ID_DM_DEC;

                fw_nwal_sendDM_start = fw_read_clock();
                retValue = nwal_sendDM(testNwGlobContext.nwalInstHandle,
                                       nwalDmDecSaHandle,
                                       &payloadInfo);

                fw_nwal_sendDM_end = fw_read_clock();
                fw_nwal_sendDM_prof += (fw_nwal_sendDM_end-fw_nwal_sendDM_start);
                fw_dm_send_count++;

                if(retValue != nwal_OK)
                {
                    System_printf("CORE: %d nwal_sendDM (Decryption) FAILED,  \n",DNUM);
                    return;
                }


                break;
        }
        else if((uint32_t)(pDmRxPayloadInfo[count].appId) == TEST_NWAL_APP_ID_DM_DEC)
        {
                if(dataLen != TEST_PAYLOAD_LEN)
                {
                    /* Error */
                    while(1);
                }
                if((uint32_t)pDmRxPayloadInfo[count].appCtxId != TEST_NWAL_CTX_ID_DM_DEC)
                {
                    System_printf("ERROR!!! Incorrect appCtxId,. Expected:0x%x,Received:0x%x \n",
                                   TEST_NWAL_CTX_ID_DM_DEC,pDmRxPayloadInfo[count].appCtxId);
                }

                 if(!(testNwalBufCmp(pDataBuffer,testPayload,TEST_PAYLOAD_LEN)))
                 {
                    /* Payload comparison failed */
                    System_printf("CORE: %d ERROR: Mismatch for the payload received from SA DM Decrypt Channel\n",
                                   DNUM);
                    nwal_SystemFlush();
                    while(1);
                 }
                 if(!(testNwalBufCmp((uint8_t*)dmAuthTag,
                                     (uint8_t*)pDmRxPayloadInfo[count].pAuthTag,
                                     pDmRxPayloadInfo[count].authTagLen)))
                 {
                    uint8_t count =0;
                    /* Authentication Tag comparison failed */
                    System_printf("CORE: %d ERROR: Mismatch for the Authentication Tag received from SA \n",
                                   DNUM);
                    System_printf("Tag after Encrypt: Length:%d",
                                   pDmRxPayloadInfo[count].authTagLen);
                    for(count=0;count < pDmRxPayloadInfo[count].authTagLen;count++)
                    {
                        System_printf("0x%x",dmAuthTag[count]);
                    }

                    System_printf("Tag after Decrypt: Length:%d",
                                   pDmRxPayloadInfo[count].authTagLen);
                    for(count=0;count < pDmRxPayloadInfo[count].authTagLen;count++)
                    {
                        System_printf("0x%x",pDmRxPayloadInfo[count].pAuthTag[count]);
                    }
                    System_printf("\n");
                    nwal_SystemFlush();
                    while(1);
                 }
                testNwGlobContext.numDMDecRespRecvd++;
                pFreePkt[count] = nwal_TRUE;
                break;
        }else
        {
                /* Unexpected. Block */
                while(1);

        }
#else
        testNwGlobContext.numDMEncRespRecvd++;
        pFreePkt[count] = nwal_TRUE;
#endif
    }
#endif
}

/*****************************************************************************
 * FUNCTION PURPOSE: Callback function from NWAL for config request
 *****************************************************************************
 * DESCRIPTION: The function will be called by the NWAL for the configuration
 *              request
 *****************************************************************************/
void testNWALCmdCallBack (nwal_AppId        appHandle,
                          uint16_t          trans_id,
                          nwal_RetValue     ret)
{

    if(ret != nwal_OK)
    {
        System_printf("ERROR: testNWALCmdCallBack returned Error Code %d\n",
                    ret);
        nwal_SystemFlush();
        testNwLocContext.numCmdFail++;
    }
    else
    {
        testNwLocContext.numCmdPass++;
        switch(testNwLocContext.transInfos[trans_id].transType)
        {
            case TEST_NWAL_HANDLE_TRANS_MAC:
            {
                if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_OPEN_PENDING)
                {
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_OPEN;
                }
                else if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_CLOSE_PENDING)
                {
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_IDLE;
                }
                break;
            }
            case TEST_NWAL_HANDLE_TRANS_IP:
            {
                if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_OPEN_PENDING)
                {
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_OPEN;
                }
                else if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_CLOSE_PENDING)
                {
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_IDLE;
                }
                break;
            }
            case TEST_NWAL_HANDLE_TRANS_PORT:
            {
                if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_OPEN_PENDING)
                {
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_OPEN;
                }
                else if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_CLOSE_PENDING)
                {
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_IDLE;
                }
                break;
            }
            case TEST_NWAL_HANDLE_TRANS_SEC_ASSOC:
            {
                if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_OPEN_PENDING)
                {
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_OPEN;
                }
                else if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_CLOSE_PENDING)
                {
                    System_printf("Set Security Assoc  Close ACK received for trans_id: %d\n",
                                testNwLocContext.transInfos[trans_id].transType,trans_id);
                    nwal_SystemFlush();
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_IDLE;
                }
                break;
            }
            case TEST_NWAL_HANDLE_TRANS_SEC_POLICY:
            {
                if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_OPEN_PENDING)
                {
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_OPEN;
                }
                else if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_CLOSE_PENDING)
                {
                    System_printf("Set Security Policy  Close ACK received for trans_id: %d\n",
                                testNwLocContext.transInfos[trans_id].transType,trans_id);
                    nwal_SystemFlush();
                    testNwLocContext.transInfos[trans_id].state =TEST_NWAL_HANDLE_STATE_IDLE;
                }
                break;
            }
            default:
            {
                System_printf("ERROR: Invalid transaction type %d for trans_id: %d\n",
                    testNwLocContext.transInfos[trans_id].transType,trans_id);
                nwal_SystemFlush();
                break;
            }

        }
    }

    if(trans_id != NWAL_TRANSID_SPIN_WAIT)
        testNwLocContext.numPendingCfg--;

    if(testNwLocContext.transInfos[trans_id].state == TEST_NWAL_HANDLE_STATE_IDLE)
    {
        testNwLocContext.transInfos[trans_id].inUse = nwal_FALSE;
        testNwLocContext.transInfos[trans_id].pRemoteConn = NULL;
    }
}

/*****************************************************************************
 * FUNCTION PURPOSE: Display statistics from PA
 *****************************************************************************
 * DESCRIPTION: The function will be called to collect statistics from PA
 *****************************************************************************/

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
        testNwLocContext.numPendingCfg--;
}


/*****************************************************************************
 * FUNCTION PURPOSE: Get Free Transaction Info for configuration request
 *****************************************************************************
 * DESCRIPTION: The function will be called for receiving free transaction info
 *****************************************************************************/
testNwalTransInfo_t * testGetFreeTransInfo (nwal_TransID_t   *pTransId)
{

    uint16_t           count=0;

    while(1)
    {
        count=0;
        while(count < TEST_MAX_NUM_TRANS)
        {
            if((testNwLocContext.transInfos[count].inUse) != nwal_TRUE)
            {
                testNwLocContext.transInfos[count].inUse = nwal_TRUE;
                *pTransId = count;
                return(&testNwLocContext.transInfos[count]);
            }
            count++;
        }

        /* Poll NWAL for the command response */
        if(testNwLocContext.numPendingCfg)
        {
            nwal_pollCtl(testNwGlobContext.nwalInstHandle,NULL,NULL);
        }
    }
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC at NWAL
 *****************************************************************************
 * DESCRIPTION: The function will be called to configure MAC instance
 *****************************************************************************/
nwal_Handle  testAddMac    (nwalMacParam_t     *pMacInfo)
{
    nwal_RetValue       retValue;
    testNwalTransInfo_t *pTransInfo;
    nwal_TransID_t            trans_id;


    pTransInfo = testGetFreeTransInfo(&trans_id);
    pTransInfo->transType = TEST_NWAL_HANDLE_TRANS_MAC;
    retValue = nwal_setMacIface(  testNwGlobContext.nwalInstHandle,
                                  trans_id,
                                  (nwal_AppId)TEST_NWAL_APP_ID_MAC,
                                  pMacInfo,
                                  &pTransInfo->handle);
    if(retValue !=  nwal_OK)
    {
        System_printf("ERROR: nwal_setMacIface returned Error Code %d\n",
                    retValue);
        nwal_SystemFlush();
        return NULL;
    }
    pTransInfo->inUse = nwal_FALSE;
    nwal_SystemFlush();

    if(trans_id != NWAL_TRANSID_SPIN_WAIT)
    {
        testNwLocContext.numPendingCfg++;
        while(testNwLocContext.numPendingCfg)
        {
            nwal_pollCtl(testNwGlobContext.nwalInstHandle,NULL,NULL);
        }
    }


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
    return(pTransInfo->handle);
}


/*****************************************************************************
 * FUNCTION PURPOSE: Configure IP at NWAL
 *****************************************************************************
 * DESCRIPTION: The function will be called to configure MAC instance
 *****************************************************************************/
nwal_Handle  testAddIp  (nwalIpParam_t* pIpParam,
                         nwal_Handle    macHandle)
{
    nwal_RetValue           retValue;
    testNwalTransInfo_t*    pTransInfo;
    nwal_TransID_t                trans_id;

    pTransInfo = testGetFreeTransInfo(&trans_id);
    pTransInfo->transType = TEST_NWAL_HANDLE_TRANS_IP;

    retValue = nwal_setIPAddr(    testNwGlobContext.nwalInstHandle,
                                  trans_id,
                                  (nwal_AppId)TEST_NWAL_APP_ID_IP,
                                  macHandle,
                                  pIpParam,
                                  &pTransInfo->handle);
    if(retValue !=  nwal_OK)
    {
        System_printf("ERROR: nwal_assignIPAddr returned Error Code %d\n",
                    retValue);
        nwal_SystemFlush();
        return NULL;
    }
    pTransInfo->inUse = nwal_FALSE;
    nwal_SystemFlush();
    if(trans_id != NWAL_TRANSID_SPIN_WAIT)
    {
        testNwLocContext.numPendingCfg++;
        while(testNwLocContext.numPendingCfg)
        {
            nwal_pollCtl(testNwGlobContext.nwalInstHandle,NULL,NULL);
        }
    }
    return(pTransInfo->handle);
}


/*****************************************************************************
 * FUNCTION PURPOSE: Configure UDP at NWAL
 *****************************************************************************
 * DESCRIPTION: The function will be called to configure MAC instance
 *****************************************************************************/
nwal_Handle testConfigUdp   (nwal_Handle      inHandle,
                             nwal_Handle      outHandle,
                             uint32_t         udpPort,
                             uint8_t          rxCoreId,
                             nwal_Bool_t      isIpSec,
                             nwal_Bool_t      isIpv6)
{
    nwal_RetValue           retValue;
    testNwalTransInfo_t     *pTransInfo;
    nwal_TransID_t          trans_id;
    testNwalRemoteConn_t*   pRemConn=NULL;
    uint16_t                retryCount=0;
    nwalTxConnCfg_t*        pTxConnCfg = &nwalTxConnCfg;

    if((rxCoreId != DNUM)&& (isIpv6))
    {
        System_printf("ERROR: Remote configuration for IPv6 not Handled currently \n");
        while(1);
    }
    if(isIpv6)
    {
        pTxConnCfg = &nwalTxConnIpv6Cfg;
    }

    pTransInfo = testGetFreeTransInfo(&trans_id);

    nwalRxConnCfg.inHandle = inHandle;
    nwalRxConnCfg.appProto.udpPort = udpPort;
    nwalRxConnCfg.rxCoreId =  rxCoreId;

    pTxConnCfg->outHandle = outHandle;
    pTxConnCfg->appProto.udpPort = udpPort;

    System_printf("CORE: %d Adding Port config,  port: %d \n",DNUM,nwalRxConnCfg.appProto.udpPort);

    nwal_SystemFlush();

    while(1)
    {
        retValue = nwal_addConn(testNwGlobContext.nwalInstHandle,
                               trans_id,
                               (nwal_AppId)TEST_NWAL_APP_ID_DEFAULT,
                               NWAL_APP_PLOAD_PROTO_UDP,
                               &nwalRxConnCfg,
                               pTxConnCfg,
                               &pTransInfo->handle);
        if(retValue == nwal_ERR_INVALID_STATE)
        {
            utilCycleDelay(1000);
            retryCount++;
            if(retryCount > TEST_NWAL_MAX_RETRY_SLAVE_CORE_STATE)
            {
                System_printf("CORE: %d Remote Core %d Retried for Max Limit. Switching to single core %d \n",
                                DNUM,rxCoreId,TEST_NWAL_MAX_RETRY_SLAVE_CORE_STATE);
                nwal_SystemFlush();
                return(nwal_OK);
            }
            continue;
        }

        if(retValue ==  nwal_OK)
        {
            /* Success */
            break;
        }
        System_printf("CORE: %d ERROR: nwal_addConn returned Error Code %d \n",
                    DNUM,retValue);
        nwal_SystemFlush();
        pTransInfo->inUse = nwal_FALSE;
        while(1);

    }


    pTransInfo->transType = TEST_NWAL_HANDLE_TRANS_PORT;
    pTransInfo->state = TEST_NWAL_HANDLE_STATE_OPEN_PENDING;
    nwal_SystemFlush();
    if(trans_id != NWAL_TRANSID_SPIN_WAIT)
    {
        testNwLocContext.numPendingCfg++;
        while(testNwLocContext.numPendingCfg)
        {
            nwal_pollCtl(testNwGlobContext.nwalInstHandle,NULL,NULL);
        }
    }
    if(rxCoreId != DNUM)
    {
        System_printf("CORE: %d Remote CORE: %d Port 0x%x Config Complete \n",
                       DNUM,rxCoreId,nwalRxConnCfg.appProto.udpPort);
#ifdef NWAL_ENABLE_SA
        if(isIpSec)
        {
            pRemConn = &testNwGlobContext.remIPSecUDPConn[rxCoreId-1];
        }
        else
        {
            pRemConn = &testNwGlobContext.remUDPConn[rxCoreId-1];
        }
#else
        pRemConn = &testNwGlobContext.remUDPConn[rxCoreId-1];
#endif
        pRemConn->handle= pTransInfo->handle;
        pRemConn->udpPort = udpPort;
        pRemConn->state = TEST_NWAL_HANDLE_STATE_OPEN;

        Osal_writeBackCache(&testNwGlobContext,sizeof(testNwGlobContext));

    }
    return(pTransInfo->handle);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Add MAC/IP/UDP remote connection from master core
 *****************************************************************************
 * DESCRIPTION: Add MAC/IP/UDP remote connection from master core
 *****************************************************************************/
nwal_Bool_t testAddRemL4 ()
{
    uint8_t     count;
    /* Add UDP entry for Remote Core, for ARM, currently support 1 remote core*/
#ifdef __ARMv7
    for(count = 1;count < CPU_NCORES-2; count++)
#else
    for(count = 1;count < CPU_NCORES; count++)
#endif
    {
        nwal_Handle tmpHandle;
        tmpHandle = testConfigUdp(nwalIpHandle,
                                  NULL,
                                 (TEST_NWAL_BASE_REM_UDP_PORT + count),
                                 count,
                                 nwal_FALSE,
                                 nwal_FALSE);
        if(tmpHandle == NULL)
        {
            return nwal_FALSE;
        }
    }

    return nwal_TRUE;
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
    nwal_Handle             fpUdpHandle;

    nwalRxConnCfg.inHandle = nwalIpHandle;
    nwalRxConnCfg.rxCoreId = 0;

    /* Add UDP entry for Remote Core */
    for(count = 0;count < CPU_NUM_REM_FAST_PATH_CORES; count++)
    {
        nwalRxConnCfg.appProto.udpPort =
                                TEST_NWAL_BASE_REM_FP_UDP_PORT + count;
        nwalRxConnCfg.appRxPktQueue =
                                TEST_NWAL_BASE_REM_FP_RX_PKT_QUEUE  + count;
        System_printf("CORE: %d Adding Remote Fast Path Port config,  port: %d Queue:%d\n",
                       DNUM,
                       nwalRxConnCfg.appProto.udpPort,
                       nwalRxConnCfg.appRxPktQueue);

        retValue = nwal_addConn(testNwGlobContext.nwalInstHandle,
                                NWAL_TRANSID_SPIN_WAIT,
                                (nwal_AppId)TEST_NWAL_APP_ID_DEFAULT,
                                NWAL_APP_PLOAD_PROTO_UDP,
                                &nwalRxConnCfg,
                                NULL,
                                &fpUdpHandle);
        if(retValue !=  nwal_OK)
        {
            System_printf("CORE: %d ERROR: nwal_addConn returned Error Code %d for remote FP Connection \n",
                           DNUM,retValue);
            nwal_SystemFlush();
            while(1);
        }
    }
    nwalRxConnCfg.appRxPktQueue = QMSS_PARAM_NOT_SPECIFIED;
    return nwal_TRUE;
}
/*****************************************************************************
 * FUNCTION PURPOSE: Test for nwal_Send with full packet header + payload
 *****************************************************************************
 * DESCRIPTION: The function will be called to send packets out
 *****************************************************************************/
nwal_Bool_t  testNwalFastSendPkt (nwal_txFlag1_t    txFlag,
                                  nwal_Bool_t       isFrag,
                                  nwal_Bool_t       isQos)
{
     nwalMbufPool_t*    pMbufPool;  /* Share the same memory buffer pool for the TX packet */
     uint8_t            count = 0,countTx=0;
     nwalBufPool_t*     pBufPool;
     Ti_Pkt*            pPkt;
     uint8_t*           pDataBuffer;
     uint32_t           dataLen;
     nwalTxPktInfo_t    txPktInfo;
     uint8_t*           pIpHdr;
#ifdef NWAL_ENABLE_PROFILE
     unsigned int       fw_nwal_send_prep_start,fw_nwal_send_prep_end;
     unsigned int       fw_nwal_send_end,fw_nwal_initPSCmd_end;
     unsigned int       fw_nwal_send_prep_prof = 0;
     unsigned int       fw_nwal_initPSCmd_prep_prof = 0;
     unsigned int       fw_nwal_initPSCmd_prof = 0;
     unsigned int       fw_nwal_setPSCmd_prof = 0;
     unsigned int       fw_nwal_send_prof = 0;
     unsigned int       sumCacheCycles=0;
     unsigned int       numCacheOps=0;
#endif
     nwalTxPSCmdInfo_t  txPsCmdInfo;
     Cppi_HostDesc*     pPktDesc;

#ifdef NWAL_ENABLE_PROFILE
     Osal_cache_op_measure_reset();
     fw_nwal_send_prep_start = fw_read_clock();
#endif

     memset(&txPktInfo,0,sizeof(nwalTxPktInfo_t));
     if(nwalPktLoopBack)
     {
         txPktInfo.lpbackPass = nwal_TRUE;
     }
     else
     {
         txPktInfo.lpbackPass = nwal_FALSE;
     }
     txPktInfo.txFlag1 = txFlag;
     txPktInfo.startOffset = 0;
     txPktInfo.ipOffBytes = TEST_PKT_IP_OFFSET_BYTES;
     txPktInfo.l4OffBytes = TEST_PKT_UDP_OFFSET_BYTES;
     txPktInfo.l4HdrLen = TEST_PKT_UDP_HDR_LEN;
     txPktInfo.ploadLen = TEST_PAYLOAD_LEN;
     if(isQos)
     {
        txPktInfo.ipOffBytes = TEST_PKT_IP_OFFSET_BYTES + 4;
        txPktInfo.l4OffBytes = TEST_PKT_UDP_OFFSET_BYTES + 4;
        pIpHdr = testPktQos + txPktInfo.ipOffBytes;
     }
     else
     {
        pIpHdr = testPkt + txPktInfo.ipOffBytes;
    }
     if(isFrag == nwal_TRUE)
     {
        txPktInfo.mtuSize = TEST_MAX_MTU_SIZE;
     }
     txPktInfo.pseudoHdrChecksum =
        test_utilGetIpv4PsudoChkSum(pIpHdr,(TEST_PAYLOAD_LEN+TEST_PKT_UDP_HDR_LEN));
#ifdef NWAL_ENABLE_PROFILE
     fw_nwal_send_prep_end = fw_read_clock();
#endif

     if(nwal_initPSCmdInfo(testNwGlobContext.nwalInstHandle,
                           &txPktInfo,
                           &txPsCmdInfo) != nwal_OK)
     {
        printf("nwal_initPSCmdInfo() ERROR \n");
     }

#ifdef NWAL_ENABLE_PROFILE
     fw_nwal_initPSCmd_end = fw_read_clock();

     fw_nwal_initPSCmd_prep_prof = fw_nwal_send_prep_end - fw_nwal_send_prep_start;
     fw_nwal_initPSCmd_prof = fw_nwal_initPSCmd_end - fw_nwal_send_prep_end;

     System_printf("Profile: NWAL FAST Send () Cycles Breakdown:Init PS Cmd Prep: %d Init NetCP Cmd %d,\n",
                   fw_nwal_initPSCmd_prep_prof,fw_nwal_initPSCmd_prof);

     testNwalDispCacheCycles(TEST_MAX_BURST);
     Osal_cache_op_measure_reset();
#endif

     while(countTx < TEST_MAX_BURST)
     {
#ifdef NWAL_ENABLE_PROFILE
         fw_nwal_send_prep_start = fw_read_clock();
#endif
         pMbufPool = &testNwLocContext.nwalLocCfg.txPktPool;
         pBufPool = pMbufPool->bufPool;
         while(count < pMbufPool->numBufPools)
         {
             if(pBufPool->bufSize >= TEST_PKT_LEN)
             {
                 break;
             }

             pBufPool++;
             count++;
         }
         if(count == pMbufPool->numBufPools)
         {
             System_printf("PktLib Allocation Out of Buf Pools \n");
             return nwal_FALSE;
         }

         pPkt = Pktlib_allocPacket(pBufPool->heapHandle,TEST_PKT_LEN);
         if (pPkt == NULL)
         {
                System_printf("PktLib Allocation failure \n");
                return (FALSE);
         }

         txPktInfo.pPkt = pPkt;

         Pktlib_getDataBuffer(pPkt,&pDataBuffer,&dataLen);
         if(dataLen < TEST_PKT_LEN)
         {
             /*  Unexpected */
             while(1);
         }

        /* Memory copy the Payload
         */
     if(isQos)
     {
        memcpy(pDataBuffer, testPktQos, TEST_PKT_LEN);
     }
     else
     {
        memcpy(pDataBuffer, testPkt, TEST_PKT_LEN);
     }
        /* Reset the IP Header Checksum */
        pDataBuffer[24] = 0;
        pDataBuffer[25] = 0;


        Pktlib_setDataBufferLen(pPkt,TEST_PKT_LEN);
        Osal_writeBackCache(pDataBuffer,TEST_PAYLOAD_LEN);

#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_send_prep_end = fw_read_clock();
#endif
         pPktDesc = Pktlib_getDescFromPacket(txPktInfo.pPkt);

        if((txFlag & (NWAL_TX_FLAG1_DO_IPV4_CHKSUM| NWAL_TX_FLAG1_DO_UDP_CHKSUM)) ==
           (NWAL_TX_FLAG1_DO_IPV4_CHKSUM| NWAL_TX_FLAG1_DO_UDP_CHKSUM))
        {
#if TEST_NWAL_DUMP_PKT
            uint32_t            tmpLen;
            uint8_t*            pTmpBuf;
#endif
            nwal_mCmdSetL4CkSumPort(txPktInfo.pPkt,
                                    &txPsCmdInfo,
                                    txPktInfo.l4OffBytes,
                                    (txPktInfo.l4HdrLen + txPktInfo.ploadLen),
                                    txPktInfo.pseudoHdrChecksum,
                                    txPktInfo.enetPort);
#if TEST_NWAL_DUMP_PKT
            pTmpBuf = nwalCppi_getPSData(Cppi_DescType_HOST,
                                 Cppi_PSLoc_PS_IN_DESC,
                                (Cppi_Desc *)pPktDesc,
                                &tmpLen);

            printf("NWAL-TEST-TX:LOG Dump of PS Data BEGIN \n");
            fw_nwal_dump_buf_32bit(pTmpBuf,tmpLen);
            printf("NWAL-TEST-TX:LOG Dump of PS Data END \n");
            
            printf("NWAL-TEST-TX:LOG Dump of PKT BEGIN \n");
            fw_nwal_dump_buf((uint8_t *)pPktDesc->buffPtr,pPktDesc->buffLen);
            printf("NWAL-TEST-TX:LOG Dump of PKT Data END \n");
#endif

        }

        else
        {
            uint8_t*    pPsDataBuf;

            nwal_mCmdUpdatePSData(pPkt,&txPsCmdInfo,&pPktDesc,&pPsDataBuf);
        }
#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_initPSCmd_end = fw_read_clock();
#endif

#ifndef __ARMv7
        Pktlib_writebackPkt(txPktInfo.pPkt);
#endif

        /* Send the packet out to transmit Q*/
        Qmss_queuePushDescSize (txPsCmdInfo.txQueue,
                                pPktDesc,
                                NWAL_DESC_SIZE);

#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_send_end = fw_read_clock();

        fw_nwal_send_prep_prof +=(fw_nwal_send_prep_end - fw_nwal_send_prep_start);
        fw_nwal_setPSCmd_prof += (fw_nwal_initPSCmd_end - fw_nwal_send_prep_end);
        fw_nwal_send_prof += (fw_nwal_send_end - fw_nwal_initPSCmd_end);
#endif
        countTx++;
    }

#ifdef NWAL_ENABLE_PROFILE

    System_printf("Profile:Pkt Alloc Cost:%d Per Pkt Cmd Update:%d, QMSS Push:%d\n",
                   fw_nwal_send_prep_prof/countTx,fw_nwal_setPSCmd_prof/countTx,fw_nwal_send_prof/countTx);

    testNwalDispCacheCycles(TEST_MAX_BURST);
#endif
    return nwal_TRUE;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC classification to terminate packet to host
 *****************************************************************************
 * DESCRIPTION:Verify packet reception from a MAC classification being terminated
 *             at host
 *****************************************************************************/
nwal_Bool_t testMacMatchRoute (nwal_Bool_t remMac)
{
    nwalMacParam_t      macParam;
    nwal_Handle         macHandle;

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC Classification Match Test");
    memcpy(&macParam,nwalMacParam,sizeof(nwalMacParam_t));

    macParam.matchAction = NWAL_MATCH_ACTION_HOST;
    macParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */
    if (remMac)
        macParam.validParams |= NWAL_SET_MAC_VALID_PARAM_REMOTE_MAC;
    
    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          &macParam,
                          &macHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        macHandle = testAddMac(&macParam);
        if(macHandle == NULL)
        {
            System_printf("CORE: %d testMacMatchRoute() Failed \n",DNUM);
            nwal_SystemFlush();
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Classification Match Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d testMacMatchRoute() Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Send packet and check for reception of packet at host */
    if(testNwalFastSendPkt(NWAL_TX_FLAG1_DO_IPV4_CHKSUM,nwal_FALSE,nwal_FALSE) != nwal_TRUE)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Classification Match Test");
        return nwal_FALSE;
    }
    testNwLocContext.numL2PktsSent += TEST_MAX_BURST;
    System_printf("CORE: %d Polling for L2 packets %d \n",
                   DNUM,
                   TEST_MAX_BURST);

    testNwalInitPollProfile();


    testNwalPoll();
    while(testNwLocContext.numL2PktsSent != testNwLocContext.numL2PktsRecvd)
    {
        nwal_SystemFlush();
        testNwalPoll();

        /* Check for the packet received */
        utilCycleDelay(20000);

    }

    System_printf("CORE: %d MAC Classification Match Num L2 Pkts Sent:%d, Num L2 Pkts Received %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL2PktsSent,
                    testNwLocContext.numL2PktsRecvd,
                    rcvd_AppID);


    testNwalDispPollProfile();

    /* Delete the MAC handle as test passed */
    if(nwal_delMacIface(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         macHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting MAC Iface \n",DNUM);
        nwal_SystemFlush();
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC Classification Match Test");
    return nwal_TRUE;
}


/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC classification to terminate packet to host
 *****************************************************************************
 * DESCRIPTION:Verify packet reception from a MAC classification being terminated
 *             at host
 *****************************************************************************/
nwal_Bool_t testMacMatchRouteVLANPriority (nwal_Bool_t remMac)
{
    nwalMacParam_t      macParam;
    nwal_Handle         macHandle;

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC Classification with VLAN Priority Routing Test");
    memcpy(&macParam,nwalMacParam,sizeof(nwalMacParam_t));

    macParam.matchAction = NWAL_MATCH_ACTION_HOST;
    macParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */

    macParam.validParams |= NWAL_SET_MAC_VALID_PARAM_ROUTE_TYPE;
    macParam.routeType = NWAL_ROUTE_VLAN_PRIORITY;
    macParam.appRxPktQueue= testNwGlobContext.qosQ[0];
   
    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          &macParam,
                          &macHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        macHandle = testAddMac(&macParam);
        if(macHandle == NULL)
        {
            System_printf("CORE: %d testMacMatchRoute() Failed \n",DNUM);
            nwal_SystemFlush();
            fw_logTest(nwal_FALSE,nwal_TRUE,
                        "NWAL MAC Classification with VLAN Priority Routing Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d testMacMatchRouteVLANPriority() Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Send packet and check for reception of packet at host */
    if(testNwalFastSendPkt(NWAL_TX_FLAG1_DO_IPV4_CHKSUM,nwal_FALSE,nwal_TRUE) != nwal_TRUE)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Classification Match Test");
        return nwal_FALSE;
    }
    testNwLocContext.numL2PktsSent += TEST_MAX_BURST;
    System_printf("CORE: %d Polling for L2 packets %d \n",
                   DNUM,
                   TEST_MAX_BURST);

    testNwalInitPollProfile();


    testNwalPoll();
    while(testNwLocContext.numL2PktsSent != testNwLocContext.numL2PktsRecvd)
    {
        nwal_SystemFlush();
        testNwalPoll();

        /* Check for the packet received */
        utilCycleDelay(20000);

    }

    System_printf("CORE: %d MAC Classification Match Num L2 Pkts Sent:%d, Num L2 Pkts Received %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL2PktsSent,
                    testNwLocContext.numL2PktsRecvd,
                    rcvd_AppID);


    testNwalDispPollProfile();

    /* Delete the MAC handle as test passed */
    if(nwal_delMacIface(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         macHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting MAC Iface \n",DNUM);
        nwal_SystemFlush();
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC Classification with VLAN Priority Routing Test");
    return nwal_TRUE;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC classification to terminate packet to host through Fail route
 *****************************************************************************
 * DESCRIPTION:Verify packet reception from a MAC classification being terminated
 *             at host from fail route
 *****************************************************************************/
nwal_Bool_t testMacFailRoute ()
{
    nwalMacParam_t      macParam;
    nwal_Handle         macHandle;

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC Classification Fail Test");
    memcpy(&macParam,nwalMacParam,sizeof(nwalMacParam_t));

    /* Set to default match and Fail Action */
    macParam.matchAction = NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE;
    macParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */

    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          &macParam,
                          &macHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        macHandle = testAddMac(&macParam);
        if(macHandle == NULL)
        {
            System_printf("CORE: %d testMacMatchRoute() Failed \n",DNUM);
            nwal_SystemFlush();
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Classification Fail Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d testMacFailRoute() Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Send packet and check for reception of packet at host */
    if(testNwalFastSendPkt((NWAL_TX_FLAG1_DO_IPV4_CHKSUM | NWAL_TX_FLAG1_DO_UDP_CHKSUM),
                           nwal_FALSE, nwal_FALSE) != nwal_TRUE)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Classification Fail Test");
        return nwal_FALSE;
    }
    testNwLocContext.numL2PktsSent += TEST_MAX_BURST;
    System_printf("CORE: %d POLLING for number of L2 packets  %d \n",
                    DNUM,
                    testNwLocContext.numL2PktsSent);

    testNwalInitPollProfile();

    testNwalPoll();
    while(testNwLocContext.numL2PktsSent != testNwLocContext.numL2PktsRecvd)
    {

        nwal_SystemFlush();
        testNwalPoll();
        /* Check for the packet received */
        utilCycleDelay(20000);

    }

    System_printf("CORE: %d MAC Classification Next Route Num L2 Pkts Sent:%d, Num L2 Pkts Received %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL2PktsSent,
                    testNwLocContext.numL2PktsRecvd,
                    rcvd_AppID);


    testNwalDispPollProfile();

    /* Delete the MAC handle as test passed */
    if(nwal_delMacIface(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         macHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting MAC Iface \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Classification Fail Test");
        return nwal_FALSE;
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC Classification Fail Test");
    return nwal_TRUE;
}
/*****************************************************************************
 * FUNCTION PURPOSE: Configure IP classification to terminate packet to
 *                   host through Match route for IP rule without MAC entry
 *****************************************************************************
 * DESCRIPTION:Verify packet reception from a IP classification being terminated
 *             at host from Match route for IP rule
 *****************************************************************************/
nwal_Bool_t testBypassMacIpMatchRoute ()
{
    nwalIpParam_t       ipParam;
    nwal_Handle         ipHandle;
    nwalMacParam_t      macParam;
    nwal_Handle         macHandle;

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC Bypass IP Classification Match Test");

    /* Add a default MAC rule to redirect all packets to IP classification */
    memset(&macParam,0,sizeof(nwalMacParam_t));

    /* Set to default match and Fail Action */
    macParam.matchAction = NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE;
    macParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */
    macParam.appRxPktFlowId = CPPI_PARAM_NOT_SPECIFIED;
    macParam.appRxPktQueue = QMSS_PARAM_NOT_SPECIFIED;

    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          &macParam,
                          &macHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        macHandle = testAddMac(&macParam);
        if(macHandle == NULL)
        {
            System_printf("CORE: %d testAddMac() Failed \n",DNUM);
            nwal_SystemFlush();
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Classification Fail Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d testBypassMacIpMatchRoute() Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

     memcpy(&ipParam,nwalIpParam,sizeof(nwalIpParam_t));
    /* Set to default match and Fail Action */
    ipParam.matchAction = NWAL_MATCH_ACTION_HOST;
    ipParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */
    /* Check if IP address is already configured */
    if(!nwal_getIPAddr(testNwGlobContext.nwalInstHandle,
                       &ipParam,
                       nwal_HANDLE_INVALID,
                       &ipHandle))
    {
        /* Not found */
        System_printf("CORE: %d Adding MAC Bypass IP entry - SRC=[%d, %d, %d, %d] \n",
                    DNUM,
                    nwalIpParam->locIpAddr.ipv4[0],
                    nwalIpParam->locIpAddr.ipv4[1],
                    nwalIpParam->locIpAddr.ipv4[2],
                    nwalIpParam->locIpAddr.ipv4[3]);
        nwal_SystemFlush();


        ipHandle = testAddIp(&ipParam,nwal_HANDLE_INVALID);
        if(ipHandle == NULL)
        {
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Bypass IP Classification Match Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing IP handle\n",DNUM);
        nwal_SystemFlush();
    }


    /* Send packet and check for reception of packet at host */
    if(testNwalFastSendPkt((NWAL_TX_FLAG1_DO_IPV4_CHKSUM | NWAL_TX_FLAG1_DO_UDP_CHKSUM),
                           nwal_FALSE, nwal_FALSE) != nwal_TRUE)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Bypass IP Classification Match Test");
        return nwal_FALSE;
    }
    testNwLocContext.numL3PktsSent += TEST_MAX_BURST;
    System_printf("CORE: %d Polling for L3 packets %d \n",
                   DNUM,
                   TEST_MAX_BURST);

    testNwalInitPollProfile();
    testNwalPoll();
    while(testNwLocContext.numL3PktsSent != testNwLocContext.numL3PktsRecvd)
    {

        nwal_SystemFlush();
        testNwalPoll();

        /* Check for the packet received */
        utilCycleDelay(20000);

    }

    System_printf("CORE: %d MAC Byass + IP Classification Match Num L3 Pkts Sent:%d, Num L3 Pkts Received %d, AppID: 0x%x \n",
                    DNUM,
                    testNwLocContext.numL3PktsSent,
                    testNwLocContext.numL3PktsRecvd,
                    rcvd_AppID);

    testNwalDispPollProfile();

    if(nwal_delIPAddr(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         ipHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting IP Handle \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Bypass IP Classification Match Test");
        return nwal_FALSE;
    }

    /* Delete the MAC handle as test passed */
    if(nwal_delMacIface(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         macHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting MAC Iface \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC Bypass IP Classification Match Test");
        return nwal_FALSE;
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC Bypass IP Classification Match Test");
    return nwal_TRUE;
}
/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC/IP classification to terminate packet to
 *                   host through Match route for IP rule
 *****************************************************************************
 * DESCRIPTION:Verify packet reception from a IP classification being terminated
 *             at host from Match route for IP rule
 *****************************************************************************/
nwal_Bool_t testIpMatchRoute (nwal_Bool_t remIp)
{
    nwalMacParam_t      macParam;
    nwalIpParam_t       ipParam;
    nwal_Handle         macHandle;
    nwal_Handle         ipHandle;

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IP Classification Match Test");
    memcpy(&macParam,nwalMacParam,sizeof(nwalMacParam_t));
    memcpy(&ipParam,nwalIpParam,sizeof(nwalIpParam_t));

    /* Set to default match and Fail Action */
    macParam.matchAction = NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE;
    macParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */

    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          &macParam,
                          &macHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        macHandle = testAddMac(&macParam);
        if(macHandle == NULL)
        {
            System_printf("CORE: %d testMacMatchRoute() Failed \n",DNUM);
            nwal_SystemFlush();
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Match Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d testMacFailRoute() Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Set to default match and Fail Action */
    ipParam.matchAction = NWAL_MATCH_ACTION_HOST;
    ipParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */
    if(remIp)
        ipParam.validParams |= NWAL_SET_IP_VALID_PARAM_REMOTE_IP;

    /* Check if IP address is already configured */
    if(!nwal_getIPAddr(testNwGlobContext.nwalInstHandle,
                       &ipParam,
                       macHandle,
                       &ipHandle))
    {
        /* Not found */
        System_printf("CORE: %d Adding IP entry - SRC=[%d, %d, %d, %d] \n",
                    DNUM,
                    nwalIpParam->locIpAddr.ipv4[0],
                    nwalIpParam->locIpAddr.ipv4[1],
                    nwalIpParam->locIpAddr.ipv4[2],
                    nwalIpParam->locIpAddr.ipv4[3]);
        nwal_SystemFlush();


        ipHandle = testAddIp(&ipParam,macHandle);
        if(ipHandle == NULL)
        {
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Match Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing IP handle\n",DNUM);
        nwal_SystemFlush();
    }


    /* Send packet and check for reception of packet at host */
    if(testNwalFastSendPkt((NWAL_TX_FLAG1_DO_IPV4_CHKSUM | NWAL_TX_FLAG1_DO_UDP_CHKSUM),
                            nwal_FALSE,nwal_FALSE) != nwal_TRUE)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Match Test");
        return nwal_FALSE;
    }
    testNwLocContext.numL3PktsSent += TEST_MAX_BURST;
    System_printf("CORE: %d Polling for L3 packets %d \n",
                   DNUM,
                   TEST_MAX_BURST);

    testNwalInitPollProfile();
    testNwalPoll();
    while(testNwLocContext.numL3PktsSent != testNwLocContext.numL3PktsRecvd)
    {

        nwal_SystemFlush();
        testNwalPoll();

        /* Check for the packet received */
        utilCycleDelay(20000);

    }

    System_printf("CORE: %d IP Classification Match  Num L3 Pkts Sent:%d, Num L3 Pkts Received %d, AppID: 0x%x \n",
                    DNUM,
                    testNwLocContext.numL3PktsSent,
                    testNwLocContext.numL3PktsRecvd,
                    rcvd_AppID);

    testNwalDispPollProfile();

    if(nwal_delIPAddr(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         ipHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting IP Handle \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Match Test");
        return nwal_FALSE;
    }

    /* Delete the MAC handle as test passed */
    if(nwal_delMacIface(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         macHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting MAC Iface \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Match Test");
        return nwal_FALSE;
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IP Classification Match Test");
    return nwal_TRUE;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC/IP classification to terminate packet to
 *                   host through Match route for IP rule, with DSCP priority
 *                   routing configured
 *****************************************************************************
 * DESCRIPTION:Verify packet reception from a IP classification being terminated
 *             at host from Match route for IP rule, with DSCP priority
 *             routing configured
 *****************************************************************************/
nwal_Bool_t testIpMatchRouteDSCPPriority()
{
    nwalMacParam_t      macParam;
    nwalIpParam_t       ipParam;
    nwal_Handle         macHandle;
    nwal_Handle         ipHandle;

    fw_logTest(nwal_TRUE,nwal_FALSE,
    "NWAL MAC/IP Classification with DSCP Priority Routing Test");
    memcpy(&macParam,nwalMacParam,sizeof(nwalMacParam_t));
    memcpy(&ipParam,nwalIpParam,sizeof(nwalIpParam_t));

    /* Set to default match and Fail Action */
    macParam.matchAction = NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE;
    macParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */

    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          &macParam,
                          &macHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        macHandle = testAddMac(&macParam);
        if(macHandle == NULL)
        {
            System_printf("CORE: %d testIpMatchRouteDSCPPriority() Failed \n",DNUM);
            nwal_SystemFlush();
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Match Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d testIpMatchRouteDSCPPriority() Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Set to default match and Fail Action */
    ipParam.matchAction = NWAL_MATCH_ACTION_HOST;
    ipParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */
        ipParam.validParams |= NWAL_SET_IP_VALID_PARAM_ROUTE_TYPE;
        ipParam.routeType = NWAL_ROUTE_DSCP_PRIORITY;
        ipParam.appRxPktQueue = testNwGlobContext.qosQ[0];

        
    /* Check if IP address is already configured */
    if(!nwal_getIPAddr(testNwGlobContext.nwalInstHandle,
                       &ipParam,
                       macHandle,
                       &ipHandle))
    {
        /* Not found */
        System_printf("CORE: %d Adding IP entry - SRC=[%d, %d, %d, %d] \n",
                    DNUM,
                    nwalIpParam->locIpAddr.ipv4[0],
                    nwalIpParam->locIpAddr.ipv4[1],
                    nwalIpParam->locIpAddr.ipv4[2],
                    nwalIpParam->locIpAddr.ipv4[3]);
        nwal_SystemFlush();


        ipHandle = testAddIp(&ipParam,macHandle);
        if(ipHandle == NULL)
        {
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Match Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing IP handle\n",DNUM);
        nwal_SystemFlush();
    }


    /* Send packet and check for reception of packet at host */
    if(testNwalFastSendPkt((NWAL_TX_FLAG1_DO_IPV4_CHKSUM | NWAL_TX_FLAG1_DO_UDP_CHKSUM),
                                nwal_FALSE,nwal_TRUE) != nwal_TRUE)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification with DSCP Priority Routing Test");
        return nwal_FALSE;
    }
    testNwLocContext.numL3PktsSent += TEST_MAX_BURST;
    System_printf("CORE: %d Polling for L3 packets %d \n",
                   DNUM,
                   TEST_MAX_BURST);

    testNwalInitPollProfile();
    //while(1);
    testNwalPoll();
    while(testNwLocContext.numL3PktsSent != testNwLocContext.numL3PktsRecvd)
    {

        nwal_SystemFlush();
        testNwalPoll();

        /* Check for the packet received */
        utilCycleDelay(20000);

    }

    System_printf("CORE: %d IP Classification Match  Num L3 Pkts Sent:%d, Num L3 Pkts Received %d, AppID: 0x%x \n",
                    DNUM,
                    testNwLocContext.numL3PktsSent,
                    testNwLocContext.numL3PktsRecvd,
                    rcvd_AppID);

    testNwalDispPollProfile();

    if(nwal_delIPAddr(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         ipHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting IP Handle \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,
            "NWAL MAC/IP Classification with DSCP Priority Routing Test");
        return nwal_FALSE;
    }

    /* Delete the MAC handle as test passed */
    if(nwal_delMacIface(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         macHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting MAC Iface \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,
            "NWAL MAC/IP Classification with DSCP Priority Routing Test");
        return nwal_FALSE;
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,
        "NWAL MAC/IP Classification with DSCP Priority Routing Test");
    return nwal_TRUE;
}



/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC/IP classification to terminate packet to
 *                   host through Fail route for IP rule
 *****************************************************************************
 * DESCRIPTION:Verify packet reception from a IP classification being terminated
 *             at host from fail route for IP rule
 *****************************************************************************/
nwal_Bool_t testIpFailRoute ()
{
    nwalMacParam_t      macParam;
    nwalIpParam_t       ipParam;
    nwal_Handle         macHandle;
    nwal_Handle         ipHandle;

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IP Classification Fail Test");
    memcpy(&macParam,nwalMacParam,sizeof(nwalMacParam_t));
    memcpy(&ipParam,nwalIpParam,sizeof(nwalIpParam_t));

    /* Set to default match and Fail Action */
    macParam.matchAction = NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE;
    macParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */

    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          &macParam,
                          &macHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        macHandle = testAddMac(&macParam);
        if(macHandle == NULL)
        {
            System_printf("CORE: %d testMacMatchRoute() Failed \n",DNUM);
            nwal_SystemFlush();
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Fail Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d testMacFailRoute() Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Set to default match and Fail Action */
    ipParam.matchAction = NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE;;
    ipParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */
    /* Check if IP address is already configured */
    if(!nwal_getIPAddr(testNwGlobContext.nwalInstHandle,
                       &ipParam,
                       macHandle,
                       &ipHandle))
    {
        /* Not found */
        System_printf("CORE: %d Adding IP entry - SRC=[%d, %d, %d, %d] \n",
                    DNUM,
                    nwalIpParam->locIpAddr.ipv4[0],
                    nwalIpParam->locIpAddr.ipv4[1],
                    nwalIpParam->locIpAddr.ipv4[2],
                    nwalIpParam->locIpAddr.ipv4[3]);
        nwal_SystemFlush();


        ipHandle = testAddIp(&ipParam,macHandle);
        if(ipHandle == NULL)
        {

            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Fail Test");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing IP handle\n",DNUM);
        nwal_SystemFlush();
    }


    /* Send packet and check for reception of packet at host */
    if(testNwalFastSendPkt((NWAL_TX_FLAG1_DO_IPV4_CHKSUM | NWAL_TX_FLAG1_DO_UDP_CHKSUM),
                            nwal_FALSE, nwal_FALSE) != nwal_TRUE)
    {
        return nwal_FALSE;
    }
    testNwLocContext.numL3PktsSent += TEST_MAX_BURST;
    System_printf("CORE: %d Polling for L3 packets %d \n",
                   DNUM,
                   TEST_MAX_BURST);

    testNwalInitPollProfile();
    testNwalPoll();
    while(testNwLocContext.numL3PktsSent != testNwLocContext.numL3PktsRecvd)
    {
        nwal_SystemFlush();
        testNwalPoll();

        /* Check for the packet received */
        utilCycleDelay(20000);

    }

    System_printf("CORE: %d IP Classification Next Route test Num L3 Pkts Sent:%d, Num L3 Pkts Received %d, AppId: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL3PktsSent,
                    testNwLocContext.numL3PktsRecvd,
                    rcvd_AppID);


    testNwalDispPollProfile();


    if(nwal_delIPAddr(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         ipHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting IP Handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Delete the MAC handle as test passed */
    if(nwal_delMacIface(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         macHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting MAC Iface \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP Classification Fail Test");
        return nwal_FALSE;
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IP Classification Fail Test");
    return nwal_TRUE;
}

void mdebugHaltPdsp (int32_t pdspNum)
{
#if !defined(DEVICE_K2L) && !defined(DEVICE_K2E) && !defined(SOC_K2L) && !defined(SOC_K2E) 
    CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)CSL_NETCP_CFG_REGS; 
            passRegs->PDSP_CTLSTAT[pdspNum].PDSP_CONTROL &= ~(CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK);
#endif
}

void mdebugRunPdsp (int32_t pdspNum)
{
#if !defined(DEVICE_K2L) && !defined(DEVICE_K2E) && !defined(SOC_K2L) && !defined(SOC_K2E) 
    CSL_Pa_ssRegs *passRegs = (CSL_Pa_ssRegs *)CSL_NETCP_CFG_REGS; 
           passRegs->PDSP_CTLSTAT[pdspNum].PDSP_CONTROL |= (CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK);
#endif
}


/*****************************************************************************
 * FUNCTION PURPOSE: Test IPv4 Fragmentation and Receipt of fragmented packets
 *                   Higher level software is expected to do reassembly
 *****************************************************************************
 * DESCRIPTION:Test IPv4 Fragmentation and Receipt of fragmented packets
 *             Higher level software is expected to do reassembly
 *****************************************************************************/
nwal_Bool_t testFragIpException ()
{
    nwalMacParam_t      macParam;
    nwalIpParam_t       ipParam;
    nwal_Handle         macHandle;
    nwal_Handle         ipHandle;
    nwalCtlInfo_t       nwalCtlInfo;
    nwal_RetValue       nwalRetVal;

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL Exception Packet Test Fragment Pkt Terminate");
    memcpy(&macParam,nwalMacParam,sizeof(nwalMacParam_t));
    memcpy(&ipParam,nwalIpParam,sizeof(nwalIpParam_t));

    /* Set to default match and Fail Action */
    macParam.matchAction = NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE;
    macParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */
    /* Enable NWAL control to enable receipt of fragmented packets */
    /* Terminate all exception packets through the default global packet queue */
    memset(&nwalCtlInfo,0,sizeof(nwalCtlInfo));
    nwalCtlInfo.pktCtl = NWAL_CTRL_CFG_ALL_EXCEPTIONS;
    nwalCtlInfo.appId = (nwal_AppId)TEST_NWAL_APP_ID_EXCEPTION;
    nwalCtlInfo.matchAction = NWAL_MATCH_ACTION_HOST;
    nwalCtlInfo.appRxPktFlowId = NWAL_FLOW_NOT_SPECIFIED;
    nwalCtlInfo.appRxPktQueue = NWAL_QUEUE_NOT_SPECIFIED;
    nwalRetVal = nwal_control(testNwGlobContext.nwalInstHandle,&nwalCtlInfo);
    if(nwalRetVal != nwal_OK)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Exception Packet Test Fragment Pkt Terminate ");
        return nwal_FALSE;
    }
    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          &macParam,
                          &macHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        macHandle = testAddMac(&macParam);
        if(macHandle == NULL)
        {
            System_printf("CORE: %d testMacMatchRoute() Failed \n",DNUM);
            nwal_SystemFlush();
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Exception Packet Test Fragment Pkt Terminate ");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d testMacFailRoute() Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Set to default match and Fail Action */
    ipParam.matchAction = NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE;;
    ipParam.failAction = NWAL_NEXT_ROUTE_FAIL_ACTION_HOST; /* Not applicable as matchAction
                                                             * will take precedence
                                                             */
    /* Check if IP address is already configured */
    if(!nwal_getIPAddr(testNwGlobContext.nwalInstHandle,
                       &ipParam,
                       macHandle,
                       &ipHandle))
    {
        /* Not found */
        System_printf("CORE: %d Adding IP entry - SRC=[%d, %d, %d, %d] \n",
                    DNUM,
                    nwalIpParam->locIpAddr.ipv4[0],
                    nwalIpParam->locIpAddr.ipv4[1],
                    nwalIpParam->locIpAddr.ipv4[2],
                    nwalIpParam->locIpAddr.ipv4[3]);
        nwal_SystemFlush();


        ipHandle = testAddIp(&ipParam,macHandle);
        if(ipHandle == NULL)
        {

            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Exception Packet Test Fragment Pkt Terminatet ");
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing IP handle\n",DNUM);
        nwal_SystemFlush();
    }

     //mdebugHaltPdsp(5);
     //mdebugRunPdsp(5);

    /* Send packet and check for reception of packet at host */
    if(testNwalFastSendPkt((NWAL_TX_FLAG1_DO_IPV4_CHKSUM | NWAL_TX_FLAG1_DO_UDP_CHKSUM),
                            nwal_TRUE,nwal_FALSE) != nwal_TRUE)
    {
        return nwal_FALSE;
    }
    testNwLocContext.numFragPktsRecvd = 0;
    testNwLocContext.numFragPktsSent = (TEST_MAX_BURST*2);
    System_printf("CORE: %d Polling for L3 packets %d \n",
                   DNUM,
                   TEST_MAX_BURST);

    testNwalInitPollProfile();
    testNwalPoll();
    while(testNwLocContext.numFragPktsSent != testNwLocContext.numFragPktsRecvd)
    {
        nwal_SystemFlush();
        testNwalPoll();

        /* Check for the packet received */
        utilCycleDelay(20000);

    }

    System_printf("CORE: %d IPv4 Fragement test Num Fragment Pkts Sent:%d, Num Fragment Pkts Received %d, AppId: 0x%x\n",
                    DNUM,
                    testNwLocContext.numFragPktsSent,
                    testNwLocContext.numFragPktsRecvd,
                    rcvd_AppID);


    testNwalDispPollProfile();
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL Exception Packet Test Fragment Pkt Terminate ");

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL Exception Packet Test Fragment Pkt Discard");
    nwalCtlInfo.matchAction = NWAL_MATCH_ACTION_DISCARD;
    nwalRetVal = nwal_control(testNwGlobContext.nwalInstHandle,&nwalCtlInfo);
    if(nwalRetVal != nwal_OK)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Exception Packet Test Fragment Pkt Discard");
        return nwal_FALSE;
    }
    /* Send packet and validate if packets are dropped at PA */
    if(testNwalFastSendPkt((NWAL_TX_FLAG1_DO_IPV4_CHKSUM | NWAL_TX_FLAG1_DO_UDP_CHKSUM),
                            nwal_TRUE, nwal_FALSE) != nwal_TRUE)
    {
        return nwal_FALSE;
    }
    testNwLocContext.numFragPktsRecvd = 0;
    testNwLocContext.numFragPktsSent = (TEST_MAX_BURST*2);
    System_printf("CORE: %d Polling for Exception Fragmented packets %d \n",
                   DNUM,
                   TEST_MAX_BURST);
    utilCycleDelay(20000);
    testNwalInitPollProfile();
    testNwalPoll();
    if(testNwLocContext.numFragPktsRecvd)
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Exception Packet Test Fragment Pkt Discard ");
        nwal_SystemFlush();
        while(1);
    }

    if(nwal_delIPAddr(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         ipHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting IP Handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Delete the MAC handle as test passed */
    if(nwal_delMacIface(testNwGlobContext.nwalInstHandle,
                         NWAL_TRANSID_SPIN_WAIT,
                         macHandle) != nwal_OK)
    {
        System_printf("CORE: %d Error deleting MAC Iface \n",DNUM);
        nwal_SystemFlush();
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Exception Packet Test Fragment Pkt Discard ");
        return nwal_FALSE;
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL Exception Packet Test Fragment Pkt Discard ");
    return nwal_TRUE;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC/IP/UDP based on the NET_ENCAP
 *****************************************************************************
 * DESCRIPTION: The function will be called along with the NET_ENCAP configuration
 *              at NEU for adding L2/L3/L4 classification at NWAL
 *****************************************************************************/
nwal_Bool_t testConfigMacIpUdp ()
{
    System_printf("**** NWAL MAC/IP/UDP Classification Match Config Begin ****\n");
    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          nwalMacParam,
                          &nwalMacHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        nwalMacHandle = testAddMac(nwalMacParam);
        if(nwalMacHandle == NULL)
        {
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Check if IP address is already configured */
    if(!nwal_getIPAddr(testNwGlobContext.nwalInstHandle,
                       nwalIpParam,
                       nwalMacHandle,
                       &nwalIpHandle))
    {
        /* Not found */
        System_printf("CORE: %d Adding IP entry - SRC=[%d, %d, %d, %d] \n",
                    DNUM,
                    nwalIpParam->locIpAddr.ipv4[0],
                    nwalIpParam->locIpAddr.ipv4[1],
                    nwalIpParam->locIpAddr.ipv4[2],
                    nwalIpParam->locIpAddr.ipv4[3]);
        nwal_SystemFlush();


        nwalIpHandle = testAddIp(nwalIpParam,
                                 nwalMacHandle);
        if(nwalIpHandle == NULL)
        {
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing IP handle\n",DNUM);
        nwal_SystemFlush();
    }
#ifndef NWAL_ENABLE_SA
    /* Set the global state to indicate that global common resource allocation is complete */
    testNwGlobContext.state = TEST_NW_CXT_GLOB_RES_ALLOC_COMPLETE;
    Osal_writeBackCache(&testNwGlobContext,sizeof(testNwGlobContext));
#endif

    /* Add UDP entry for Local Core*/
    nwal_SystemFlush();
    udpHandle = testConfigUdp(nwalIpHandle,
                              NULL,
                             (TEST_NWAL_BASE_UDP_PORT + DNUM),
                             DNUM,
                             nwal_FALSE,
                             nwal_FALSE);
    if(udpHandle == NULL)
    {
        System_printf("CORE %d: Error adding UDP entry: 0x%x \n", DNUM,(TEST_NWAL_BASE_UDP_PORT+DNUM));
        nwal_SystemFlush();
        return nwal_FALSE;
    }
    if(!DNUM)
    {
        testAddRemL4();
    }
    System_printf("**** NWAL MAC/IP/UDP Classification Match Config End ****\n\n");
    return nwal_TRUE;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC/IPv6/UDP
 *****************************************************************************
 * DESCRIPTION: The function will be called along with the NET_ENCAP configuration
 *              at NEU for adding L2/L3[IPv6/L4 classification at NWAL
 *****************************************************************************/
nwal_Bool_t testConfigMacIpv6Udp ()
{
    System_printf("**** NWAL MAC/IPv6/UDP Classification Match Config Begin ****\n");
    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          nwalMacParam,
                          &nwalMacHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        nwalMacHandle = testAddMac(nwalMacParam);
        if(nwalMacHandle == NULL)
        {
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Check if IP address is already configured */
    if(!nwal_getIPAddr(testNwGlobContext.nwalInstHandle,
                       nwalIpv6Param,
                       nwalMacHandle,
                       &nwalIpv6Handle))
    {
        /* Not found */
        uint8_t     count;
        System_printf("CORE: %d Adding IP entry - SRC=[",DNUM);
        for(count=0;count < NWAL_IPV6_ADDR_SIZE;count++)
        {
            System_printf(" %d ",nwalIpv6Param->locIpAddr.ipv6[count]);
        }
        System_printf("] \n");
        nwal_SystemFlush();


        nwalIpv6Handle = testAddIp(nwalIpv6Param,
                                   nwalMacHandle);
        if(nwalIpv6Handle == NULL)
        {
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using Existing IPv6 handle\n",DNUM);
        nwal_SystemFlush();
    }
    /* Add UDP entry for Local Core*/
    nwal_SystemFlush();
    nwalIpv6UdpHandle = testConfigUdp(nwalIpv6Handle,
                                      NULL,
                                      (TEST_NWAL_BASE_UDP_PORT + DNUM),
                                      DNUM,
                                      nwal_FALSE,
                                      nwal_TRUE);
    if(nwalIpv6UdpHandle == NULL)
    {
        System_printf("CORE %d: Error adding IPv6/UDP entry: 0x%x \n",
                       DNUM,
                       (TEST_NWAL_BASE_UDP_PORT+DNUM));
        nwal_SystemFlush();
        return nwal_FALSE;
    }
    System_printf("**** NWAL MAC/IPv6/UDP Classification Match Config End ****\n\n");
    return nwal_TRUE;
}

#ifdef NWAL_ENABLE_SA
/*****************************************************************************
 * FUNCTION PURPOSE: Create SA
 *****************************************************************************
 * DESCRIPTION: The function will be called to create Security Association for IPSec
 *****************************************************************************/
nwal_Handle testSetAssoc(nwal_Handle            macHandle,
                         nwal_IpType            ipType,
                         nwal_SaDir             ipSecDir,
                         nwalSaIpSecId_t*       pSaIpSecId,
                         nwalSecKeyParams_t*    pSaKeyParam,
                         nwalSaIpSecParam_t*    pSaIpSecParam)
{
    nwal_RetValue           retValue;
    testNwalTransInfo_t*    pTransInfo;
    nwal_TransID_t          trans_id;
    nwalCreateSAParams_t    createParam;
    nwal_Handle             tmpIpSecHandle;
    uint32_t                swInfo0;
    uint32_t                swInfo1;

    if(nwal_getSecAssoc(testNwGlobContext.nwalInstHandle,
                        pSaIpSecId,
                        ipSecDir,
                        &tmpIpSecHandle,
                        &swInfo0,
                        &swInfo1) == nwal_TRUE)
    {
        System_printf("CORE: %d Using pre existing Security Association Direction %d \n",DNUM,ipSecDir);
        nwal_SystemFlush();
        return(tmpIpSecHandle);
    }

    pTransInfo = testGetFreeTransInfo(&trans_id);
    System_printf("CORE: %d Creating Security Association Direction %d,  \n",DNUM,ipSecDir);
    nwal_SystemFlush();
    memset(&createParam,0,sizeof(createParam));

    createParam.h.macHandle = macHandle;
    createParam.ipType = ipType;
    memcpy(&createParam.saIpSecParam,pSaIpSecParam,sizeof(nwalSaIpSecParam_t));

    memcpy(&createParam.keyParam,pSaKeyParam,sizeof(nwalSecKeyParams_t));
    createParam.keyParam.pEncKey = nwalEncrKey;
    createParam.keyParam.pAuthKey = nwalAuthKey;

    pTransInfo->transType = TEST_NWAL_HANDLE_TRANS_SEC_ASSOC;

    if(trans_id != NWAL_TRANSID_SPIN_WAIT)
    {
        testNwLocContext.numPendingCfg++;
    }

    retValue = nwal_setSecAssoc(testNwGlobContext.nwalInstHandle,
                                trans_id,
                                (nwal_AppId)TEST_NWAL_APP_ID_DEFAULT,
                                pSaIpSecId,
                                &createParam,
                                &pTransInfo->handle);
    System_printf("CORE: %d Set Security Assoc called with  trans_id: %d Direction %d \n",DNUM,trans_id,ipSecDir);
    nwal_SystemFlush();
    if(retValue !=  nwal_OK)
    {
        if(retValue == nwal_TRANS_COMPLETE)
        {
            /* API call successful */
            testNwLocContext.numPendingCfg--;
            pTransInfo->inUse = nwal_FALSE;
            goto testSetAssoc_getSecAssoc;
        }

        System_printf("CORE: %d ERROR: nwal_setSecAssoc returned Error Code %d \n",
                    DNUM,retValue);
        nwal_SystemFlush();
        if(trans_id != NWAL_TRANSID_SPIN_WAIT)
        {
            testNwLocContext.numPendingCfg--;
        }
        pTransInfo->inUse = nwal_FALSE;
        return nwal_FALSE;
    }
    else
    {
        while(testNwLocContext.numPendingCfg)
        {
            nwal_pollCtl(testNwGlobContext.nwalInstHandle,NULL,NULL);
        }
    }
testSetAssoc_getSecAssoc:
    if(nwal_getSecAssoc(testNwGlobContext.nwalInstHandle,
                        pSaIpSecId,
                        ipSecDir,
                        &tmpIpSecHandle,
                        &swInfo0,
                        &swInfo1) == nwal_TRUE)
    {
        System_printf("CORE: %d SA Channel Created.Direction:%d swInfo0: 0x%x swInfo1:0x%x\n",
                       DNUM,ipSecDir,swInfo0,swInfo1);
        nwal_SystemFlush();
    }
    else
    {

        System_printf("CORE: %d ERROR Unexpected Failure from nwal_getSecAssoc %d \n",DNUM,ipSecDir);
        while(1);
    }
    return(pTransInfo->handle);
}


/*****************************************************************************
 * FUNCTION PURPOSE: Add SP
 *****************************************************************************
 * DESCRIPTION: The function will be called to configure  Policy
 *****************************************************************************/
nwal_Handle testAddSP(nwal_Handle           nwalSecAssocHandle,
                      nwalSecPolParams_t*   pNwalPolParams)
{
    nwal_RetValue           retValue;
    testNwalTransInfo_t*    pTransInfo;
    nwal_TransID_t          trans_id;
    nwal_Handle             tmpPolicyHandle;

    pTransInfo = testGetFreeTransInfo(&trans_id);
    pNwalPolParams->handle = nwalSecAssocHandle;

    if(nwal_getSecPolicy(testNwGlobContext.nwalInstHandle,
                        pNwalPolParams,
                        &tmpPolicyHandle) == nwal_TRUE)
    {
        System_printf("CORE: %d Using pre existing Security Policy  \n",DNUM);
        nwal_SystemFlush();
        return(tmpPolicyHandle);
    }


    System_printf("CORE: %d Creating Security Policy,  \n",DNUM);
    nwal_SystemFlush();

    pTransInfo->transType = TEST_NWAL_HANDLE_TRANS_SEC_POLICY;
    if(trans_id != NWAL_TRANSID_SPIN_WAIT)
    {
        testNwLocContext.numPendingCfg++;
    }

    retValue = nwal_setSecPolicy(testNwGlobContext.nwalInstHandle,
                                 trans_id,
                                 (nwal_AppId)TEST_NWAL_APP_ID_DEFAULT,
                                 pNwalPolParams,
                                 &pTransInfo->handle);
    System_printf("CORE: %d Set Security Policy called with  trans_id: %d \n",DNUM,trans_id);
    nwal_SystemFlush();
    if(retValue !=  nwal_OK)
    {
        if(retValue == nwal_TRANS_COMPLETE)
        {
            /* API call successful */
            testNwLocContext.numPendingCfg--;
            pTransInfo->inUse = nwal_FALSE;
            return(pTransInfo->handle);
        }
        else
        {

            System_printf("CORE: %d ERROR: nwal_setSecPolicy returned Error Code %d \n",
                        DNUM,retValue);
            nwal_SystemFlush();
            if(trans_id != NWAL_TRANSID_SPIN_WAIT)
            {
                testNwLocContext.numPendingCfg--;
            }
            pTransInfo->inUse = nwal_FALSE;
            return nwal_FALSE;
        }
    }

    while(testNwLocContext.numPendingCfg)
    {
        nwal_pollCtl(testNwGlobContext.nwalInstHandle,NULL,NULL);
    }
    return(pTransInfo->handle);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Add MAC/IPSec/IP/UDP remote connection from master core
 *****************************************************************************
 * DESCRIPTION: Add MAC/IPSec/IP/UDP remote connection from master core
 *****************************************************************************/
nwal_Bool_t testAddRemIPSecESPL4 ()
{
    uint8_t     count;
    /* Add UDP entry for Remote Core, for ARM, currently support 1 remote core*/

#ifdef __ARMv7
    for(count = 1;count < CPU_NCORES-2; count++)
#else
    for(count = 1;count < CPU_NCORES; count++)
#endif
    {
        nwal_Handle tmpHandle;
        tmpHandle = testConfigUdp(nwalESPPolInHandle,
                                  nwalESPPolOutHandle,
                                  (TEST_NWAL_BASE_REM_UDP_PORT + count),
                                  count,
                                  nwal_TRUE,
                                  nwal_FALSE);
        if(tmpHandle == NULL)
        {
            System_printf("CORE %d: Error adding Remote IPSEC/IP/UDP entry: 0x%x for Remote CORE %d \n",
                            DNUM,
                            (TEST_NWAL_BASE_REM_UDP_PORT+DNUM),count);
            nwal_SystemFlush();
            return nwal_FALSE;
        }
    }

    return nwal_TRUE;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC/IPSec/IP/UDP
 *****************************************************************************
 * DESCRIPTION:
 *****************************************************************************/
nwal_Bool_t testConfigMacIpSecIpUdp ()
{
    nwal_RetValue           retValue;
    uint8_t                 count=0;

    if(!DNUM)
    {
        System_printf("**** NWAL Null MAC/IPSec/IP/UDP Classification Match Create/Delete Test Begin ****\n");
        for(count=0;count< (TEST_MAX_NUM_IPSEC_CHANNELS + 5);count++)
        {
            /* Test IPSec ESP Tunnel with NULL MAC Handle */
            /* Only configuration is being tested. Data packet TX and RX
             * not being tested as currently unit test uses SA LLD to prepare
             * IPSec protocol header
             */
             /* Create IPSec ESP Tunnel SA Channels */
            nwalSecAssocESPInHandle = testSetAssoc(nwal_HANDLE_INVALID,
                                                   nwal_IPV4,
                                                   NWAL_SA_DIR_OUTBOUND,
                                                   &nwalSaIpSecESPId,
                                                   &nwalSaEspKeyParam,
                                                   &nwalSaIpSecESPParam[1]);
            if(nwalSecAssocESPInHandle == NULL)
            {
                while(1);
            }

            nwalESPPolInHandle = testAddSP(nwalSecAssocESPInHandle,
                                           &nwalPolParams[0]);
            if(nwalESPPolInHandle == NULL)
            {
                while(1);
            }

            retValue = nwal_delSecPolicy(testNwGlobContext.nwalInstHandle,
                                         NWAL_TRANSID_SPIN_WAIT,
                                         nwalESPPolInHandle);
            if((retValue !=  nwal_OK) && (retValue != nwal_TRANS_COMPLETE))
            {
                System_printf("CORE: %d DEL Security Policy Failed !!!Error:%d \n",DNUM,retValue);
                return nwal_FALSE;
            }
            System_printf("CORE: %d Security Policy Deleted \n",DNUM);

            retValue = nwal_delSecAssoc(testNwGlobContext.nwalInstHandle,
                                        NWAL_TRANSID_SPIN_WAIT,
                                        nwalSecAssocESPInHandle);
            if((retValue !=  nwal_OK) && (retValue != nwal_TRANS_COMPLETE))
            {
                System_printf("CORE: %d DEL Security Assoc Failed !!! \n",DNUM);
                return nwal_FALSE;
            }
            System_printf("CORE: %d SA Deleted \n",DNUM);
            utilCycleDelay(10000);
        }
        System_printf("**** NWAL Null MAC/IPSec/IP/UDP Classification Match Config End ****\n");
    }

    System_printf("**** NWAL MAC/IPSec/IP/UDP Classification Match Config Begin ****\n");
    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          nwalMacParam,
                          &nwalMacHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        nwalMacHandle = testAddMac(nwalMacParam);
        if(nwalMacHandle == NULL)
        {
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Create IPSec ESP Tunnel SA Channels */
    nwalSecAssocESPInHandle = testSetAssoc(nwalMacHandle,
                                           nwal_IPV4,
                                           NWAL_SA_DIR_INBOUND,
                                           &nwalSaIpSecESPId,
                                           &nwalSaEspKeyParam,
                                           &nwalSaIpSecESPParam[0]);
    if(nwalSecAssocESPInHandle == NULL)
    {
        while(1);
    }

    nwalESPPolInHandle = testAddSP(nwalSecAssocESPInHandle,
                                   &nwalPolParams[0]);
    if(nwalESPPolInHandle == NULL)
    {
        while(1);
    }

    nwalSecAssocESPOutHandle = testSetAssoc(nwalMacHandle,
                                            nwal_IPV4,
                                            NWAL_SA_DIR_OUTBOUND,
                                            &nwalSaIpSecESPId,
                                            &nwalSaEspKeyParam,
                                            &nwalSaIpSecESPParam[1]);
    if(nwalSecAssocESPOutHandle == NULL)
    {
        while(1);
    }

    nwalESPPolOutHandle = testAddSP(nwalSecAssocESPOutHandle,
                                    &nwalPolParams[1]);
    if(nwalESPPolOutHandle == NULL)
    {
        while(1);
    }

    /* Add UDP entry */
    System_printf("CORE: %d Adding UDP entry for MAC/IPSec[ESP]/IP config \n", DNUM);
    nwal_SystemFlush();
    saESPUdpHandle = testConfigUdp(nwalESPPolInHandle,
                                   nwalESPPolOutHandle,
                                   (TEST_NWAL_BASE_UDP_PORT + DNUM),
                                   DNUM,
                                   nwal_TRUE,
                                   nwal_FALSE);
    if(saESPUdpHandle == NULL)
    {
        System_printf("CORE %d: Error adding UDP entry: 0x%x \n", DNUM,(TEST_NWAL_BASE_UDP_PORT+DNUM));
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    if(!DNUM)
    {
        testAddRemIPSecESPL4();
    }
    System_printf("**** NWAL MAC/IPSec[ESP]/IP/UDP Classification Match Config End ****\n\n");

    System_printf("**** NWAL MAC/IPSec[ESP]-Transport/UDP Classification Match Config BEGIN ****\n\n");
    /* Create IPSec ESP Transport SA Channels */
    nwalSecAssocTransESPInHandle =
                             testSetAssoc(nwalMacHandle,
                                          nwal_IPV4,
                                          NWAL_SA_DIR_INBOUND,
                                          &nwalSaIpSecTransportESPId,
                                          &nwalSaEspKeyParam,
                                          &nwalSaIpSecTransESPParam[0]);
    if(nwalSecAssocTransESPInHandle == NULL)
    {
        while(1);
    }

    nwalESPTransPolInHandle = testAddSP(nwalSecAssocTransESPInHandle,
                                  &nwalPolParams[0]);
    if(nwalESPTransPolInHandle == NULL)
    {
        while(1);
    }

    nwalSecAssocTransESPOutHandle =
                             testSetAssoc(nwalMacHandle,
                                          nwal_IPV4,
                                          NWAL_SA_DIR_OUTBOUND,
                                          &nwalSaIpSecTransportESPId,
                                          &nwalSaEspKeyParam,
                                          &nwalSaIpSecTransESPParam[1]);
    if(nwalSecAssocTransESPOutHandle == NULL)
    {
        while(1);
    }

    nwalESPTransPolOutHandle = testAddSP(nwalSecAssocTransESPOutHandle,
                                   &nwalPolParams[1]);
    if(nwalESPTransPolOutHandle == NULL)
    {
        while(1);
    }

    /* Add UDP entry */
    System_printf("CORE: %d Adding UDP entry for MAC/IPSec-Trans config \n", DNUM);
    nwal_SystemFlush();
    saESPTransUdpHandle = testConfigUdp(nwalESPTransPolInHandle,
                                  nwalESPTransPolOutHandle,
                                  (TEST_NWAL_BASE_ESP_TRANS_UDP_PORT + DNUM),
                                  DNUM,
                                  nwal_TRUE,
                                  nwal_FALSE);
    if(saESPTransUdpHandle == NULL)
    {
        System_printf("CORE %d: Error adding UDP entry: 0x%x \n", DNUM,(TEST_NWAL_BASE_AH_UDP_PORT+DNUM));
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    System_printf("**** NWAL MAC/IPSec ESP Transport/UDP Classification Match Config End ****\n\n");

    System_printf("**** NWAL MAC/IPSec[AH]-Transport/UDP Classification Match Config BEGIN ****\n\n");
    /* Create IPSec ESP Transport SA Channels */
    nwalSecAssocTransAHInHandle =
                             testSetAssoc(nwalMacHandle,
                                          nwal_IPV4,
                                          NWAL_SA_DIR_INBOUND,
                                          &nwalSaIpSecTransportAHId,
                                          &nwalSaTransAHKeyParam,
                                          &nwalSaIpSecTransAHParam[0]);
    if(nwalSecAssocTransAHInHandle == NULL)
    {
        while(1);
    }

    nwalAHTransPolInHandle = testAddSP(nwalSecAssocTransAHInHandle,
                                       &nwalPolParams[0]);
    if(nwalAHTransPolInHandle == NULL)
    {
        while(1);
    }

    nwalSecAssocTransAHOutHandle =
                             testSetAssoc(nwalMacHandle,
                                          nwal_IPV4,
                                          NWAL_SA_DIR_OUTBOUND,
                                          &nwalSaIpSecTransportAHId,
                                          &nwalSaTransAHKeyParam,
                                          &nwalSaIpSecTransAHParam[1]);
    if(nwalSecAssocTransAHOutHandle == NULL)
    {
        while(1);
    }

    nwalAHTransPolOutHandle = testAddSP(nwalSecAssocTransAHOutHandle,
                                         &nwalPolParams[1]);
    if(nwalAHTransPolOutHandle == NULL)
    {
        while(1);
    }

    /* Add UDP entry */
    System_printf("CORE: %d Adding UDP entry for MAC/IPSec-Trans config \n", DNUM);
    nwal_SystemFlush();
    saAHTransUdpHandle = testConfigUdp(nwalAHTransPolInHandle,
                                      nwalAHTransPolOutHandle,
                                     (TEST_NWAL_BASE_AH_TRANS_UDP_PORT + DNUM),
                                      DNUM,
                                      nwal_TRUE,
                                      nwal_FALSE);
    if(saAHTransUdpHandle == NULL)
    {
        System_printf("CORE %d: Error adding UDP entry: 0x%x \n", DNUM,(TEST_NWAL_BASE_AH_TRANS_UDP_PORT+DNUM));
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    System_printf("**** NWAL MAC/IPSec AH Transport/UDP Classification Match Config End ****\n\n");

    System_printf("**** NWAL MAC/IPSec AH /IP/UDP Classification Match Config Begin  ****\n\n");
    /* Create IPSec AH Tunnel SA Channels */
    nwalSecAssocAHInHandle = testSetAssoc(nwalMacHandle,
                                          nwal_IPV4,
                                          NWAL_SA_DIR_INBOUND,
                                          &nwalSaIpSecAHId,
                                          &nwalSaAHKeyParam,
                                          &nwalSaIpSecAHParam[0]);
    if(nwalSecAssocAHInHandle == NULL)
    {
        while(1);
    }

    nwalAHPolInHandle = testAddSP(nwalSecAssocAHInHandle,
                                  &nwalPolParams[0]);
    if(nwalAHPolInHandle == NULL)
    {
        while(1);
    }

    nwalSecAssocAHOutHandle = testSetAssoc(nwalMacHandle,
                                           nwal_IPV4,
                                           NWAL_SA_DIR_OUTBOUND,
                                           &nwalSaIpSecAHId,
                                           &nwalSaAHKeyParam,
                                           &nwalSaIpSecAHParam[1]);
    if(nwalSecAssocAHOutHandle == NULL)
    {
        while(1);
    }

    nwalAHPolOutHandle = testAddSP(nwalSecAssocAHOutHandle,
                                   &nwalPolParams[1]);
    if(nwalAHPolOutHandle == NULL)
    {
        while(1);
    }

    /* Set the global state to indicate that global common resource allocation is complete */
    testNwGlobContext.state = TEST_NW_CXT_GLOB_RES_ALLOC_COMPLETE;
    Osal_writeBackCache(&testNwGlobContext,sizeof(testNwGlobContext));

    /* Add UDP entry */
    System_printf("CORE: %d Adding UDP entry for MAC/IPSec[AH]/IP config \n", DNUM);
    nwal_SystemFlush();
    saAHUdpHandle = testConfigUdp(nwalAHPolInHandle,
                                  nwalAHPolOutHandle,
                                  (TEST_NWAL_BASE_AH_UDP_PORT + DNUM),
                                  DNUM,
                                  nwal_TRUE,
                                  nwal_FALSE);
    if(saAHUdpHandle == NULL)
    {
        System_printf("CORE %d: Error adding UDP entry: 0x%x \n", DNUM,(TEST_NWAL_BASE_AH_UDP_PORT+DNUM));
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    System_printf("**** NWAL MAC/IPSec[AH]/IP/UDP Classification Match Config End ****\n\n");

    return nwal_TRUE;
}
/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC/IPv6/IPSec/IPv6/UDP
 *****************************************************************************
 * DESCRIPTION:
 *****************************************************************************/
nwal_Bool_t testConfigMacIpv6IpSecIpv6Udp ()
{

    System_printf("**** NWAL MAC/IPv6/IPSec/IPv6/UDP Classification Match Config Begin ****\n");
    /* Check if NWAL MAC instance is already created */
    if(!nwal_getMacIface(testNwGlobContext.nwalInstHandle,
                          nwalMacParam,
                          &nwalMacHandle))
    {
        /* Not found */
        System_printf("CORE %d: Adding MAC entry \n",DNUM);
        nwal_SystemFlush();
        nwalMacHandle = testAddMac(nwalMacParam);
        if(nwalMacHandle == NULL)
        {
            return nwal_FALSE;
        }
    }
    else
    {
        System_printf("CORE: %d Using existing MAC handle \n",DNUM);
        nwal_SystemFlush();
    }

    /* Create IPSec ESP Tunnel SA Channels */
    nwalIpv6SecAssocESPInHandle = testSetAssoc(nwalMacHandle,
                                               nwal_IPV6,
                                               NWAL_SA_DIR_INBOUND,
                                               &nwalSaIpv6IpSecESPId,
                                               &nwalSaEspKeyParam,
                                               &nwalSaIpSecESPParam[0]);
    if(nwalIpv6SecAssocESPInHandle == NULL)
    {
        while(1);
    }

    nwalIpv6ESPPolInHandle = testAddSP(nwalIpv6SecAssocESPInHandle,
                                       &nwalPolIPv6Params[0]);
    if(nwalIpv6ESPPolInHandle == NULL)
    {
        while(1);
    }

    nwalIPv6SecAssocESPOutHandle = testSetAssoc(nwalMacHandle,
                                                nwal_IPV6,
                                                NWAL_SA_DIR_OUTBOUND,
                                                &nwalSaIpv6IpSecESPId,
                                                &nwalSaEspKeyParam,
                                                &nwalSaIpSecESPParam[1]);
    if(nwalIPv6SecAssocESPOutHandle == NULL)
    {
        while(1);
    }

    nwalIpv6ESPPolOutHandle = testAddSP(nwalIPv6SecAssocESPOutHandle,
                                        &nwalPolIPv6Params[1]);
    if(nwalIpv6ESPPolOutHandle == NULL)
    {
        while(1);
    }

    /* Add UDP entry */
    System_printf("CORE: %d Adding UDP Port:%d entry for MAC/IPv6/IPSec[ESP]/IPv6 config \n",
                   DNUM,
                   (TEST_NWAL_BASE_IPV6_ESP_UDP_PORT + DNUM));
    nwal_SystemFlush();
    saIpv6ESPUdpHandle = testConfigUdp(nwalIpv6ESPPolInHandle,
                                       nwalIpv6ESPPolOutHandle,
                                       (TEST_NWAL_BASE_IPV6_ESP_UDP_PORT + DNUM),
                                       DNUM,
                                       nwal_TRUE,
                                       nwal_TRUE);
    if(saIpv6ESPUdpHandle == NULL)
    {
        System_printf("CORE %d: Error adding UDP entry: 0x%x \n",
                       DNUM,
                       (TEST_NWAL_BASE_IPV6_ESP_UDP_PORT+DNUM));
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    System_printf("*******NWAL MAC/IPv6/IPSec[ESP]/IPv6/UDP Classification Match Config End *******\n\n");

    /* Create IPSec AH Tunnel SA Channels */
    nwalIpv6SecAssocAHInHandle = testSetAssoc(nwalMacHandle,
                                              nwal_IPV6,
                                              NWAL_SA_DIR_INBOUND,
                                              &nwalSaIpv6IpSecAHId,
                                              &nwalSaAHKeyParam,
                                              &nwalSaIpSecAHParam[0]);
    if(nwalIpv6SecAssocAHInHandle == NULL)
    {
        while(1);
    }

    nwalIpv6AHPolInHandle = testAddSP(nwalIpv6SecAssocAHInHandle,
                                      &nwalPolIPv6Params[0]);
    if(nwalIpv6AHPolInHandle == NULL)
    {
        while(1);
    }

    nwalIpv6SecAssocAHOutHandle = testSetAssoc(nwalMacHandle,
                                               nwal_IPV6,
                                               NWAL_SA_DIR_OUTBOUND,
                                               &nwalSaIpv6IpSecAHId,
                                               &nwalSaAHKeyParam,
                                               &nwalSaIpSecAHParam[1]);
    if(nwalIpv6SecAssocAHOutHandle == NULL)
    {
        while(1);
    }

    nwalIpv6AHPolOutHandle = testAddSP(nwalIpv6SecAssocAHOutHandle,
                                       &nwalPolIPv6Params[1]);
    if(nwalIpv6AHPolOutHandle == NULL)
    {
        while(1);
    }


    /* Add UDP entry */
    System_printf("CORE: %d Adding UDP Port:%d entry for MAC/IPSec[AH]/IP config \n",
                   DNUM,
                   (TEST_NWAL_BASE_IPV6_AH_UDP_PORT + DNUM));
    nwal_SystemFlush();
    saIpv6AHUdpHandle = testConfigUdp(nwalIpv6AHPolInHandle,
                                      nwalIpv6AHPolOutHandle,
                                      (TEST_NWAL_BASE_IPV6_AH_UDP_PORT + DNUM),
                                      DNUM,
                                      nwal_TRUE,
                                      nwal_TRUE);
    if(saIpv6AHUdpHandle == NULL)
    {
        System_printf("CORE %d: Error adding UDP entry: 0x%x \n",
                       DNUM,
                       (TEST_NWAL_BASE_IPV6_AH_UDP_PORT+DNUM));
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    System_printf("**** NWAL MAC/IPv6/IPSec[AH]/IPv6/UDP Classification Match Config End ****\n\n");

    return nwal_TRUE;
}


/*****************************************************************************
 * FUNCTION PURPOSE: Data Mode Channel Stats
 *****************************************************************************
 * DESCRIPTION: Data Mode Channel Stats
 *****************************************************************************/
nwal_Bool_t  testGetDataModeStats (nwal_Handle  handle)
{
    Sa_DataModeStats_t  dmStats;
    nwal_RetValue       retVal;

    memset(&dmStats,0,sizeof(Sa_DataModeStats_t));

    retVal = nwal_getDataModeStats(testNwGlobContext.nwalInstHandle,
                                   handle,
                                   &dmStats);
    if(retVal != nwal_OK)
    {
        System_printf("CORE: %d Error getting Data Mode Stats: Ret Status: %d \n",
                       retVal);
        return(nwal_FALSE);
    }
    System_printf("Data Mode Stats pktHi:0x%x,pktLo:0x%x \n",
                   dmStats.pktHi,dmStats.pktLo);
    System_printf("------------- Data Mode Stats END ----------------------------\n\n");
    return(nwal_TRUE);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure MAC/IPSec/IP/UDP
 *****************************************************************************
 * DESCRIPTION:
 *****************************************************************************/
nwal_Bool_t testConfigDmSa ()
{
    nwal_RetValue           retValue;
    nwalCreateDmSAParams_t  dmSaParam;
    nwalDmTxPayloadInfo_t   payloadInfo;
    Ti_Pkt*                 pPkt;
    nwalMbufPool_t*         pMbufPool;  /* Share the same memory buffer pool for the TX packet */
    uint8_t                 count = 0;
    nwalBufPool_t*          pBufPool;
    uint8_t*                pDataBuffer;
    uint32_t                dataLen;
    uint8_t                 iv[16]={ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL Sideband IPSec Data Mode Test");
    System_printf("CORE: %d Creating Data Mode Security Association (Encryption),  \n",DNUM);
    System_printf("CORE: %d Encryption Mode:%d, Authentication Alg:%d, Mac Size:%d \n",
                   DNUM,nwalDmSaParam[0].cipherMode,nwalDmSaParam[0].authMode,
                   nwalDmSaParam[0].macSize);
    System_printf("CORE: %d Encryption Key Sz:%d, Authentication Key Sz:%d \n",
                   DNUM,nwalSaDmKeyParam.encKeySize,
                   nwalSaDmKeyParam.macKeySize);
    nwal_SystemFlush();
    memset(&dmSaParam,0,sizeof(nwalCreateDmSAParams_t));
    memcpy(&dmSaParam.dmSaParam,&nwalDmSaParam[0],sizeof(nwalDmSAParams_t));
    memcpy(&dmSaParam.keyParam,&nwalSaDmKeyParam,sizeof(nwalSecKeyParams_t));
    dmSaParam.keyParam.pEncKey = nwalEncrKey;
    dmSaParam.keyParam.pAuthKey = nwalAuthKey;
    retValue = nwal_setDMSecAssoc(testNwGlobContext.nwalInstHandle,
                                  (nwal_AppId)TEST_NWAL_APP_ID_DM_DEC,
                                  &dmSaParam,
                                  &nwalDmDecSaHandle);
    if(retValue != nwal_OK)
    {
        System_printf("CORE: %d Data Mode Security Association (Encryption) FAILED,  \n",DNUM);
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Sideband IPSec Data Mode Test");
        return nwal_FALSE;
    }

    System_printf("CORE: %d Creating Data Mode Security Association (Decryption),  \n",DNUM);
    System_printf("CORE: %d Decryption Mode:%d, Authentication Alg:%d, Mac Size:%d \n",
                   DNUM,nwalDmSaParam[1].cipherMode,nwalDmSaParam[1].authMode,
                   nwalDmSaParam[1].macSize);
    System_printf("CORE: %d Decryption Key Sz:%d, Authentication Key Sz:%d \n",
                   DNUM,nwalSaDmKeyParam.encKeySize,
                   nwalSaDmKeyParam.macKeySize);
    nwal_SystemFlush();
    memset(&dmSaParam,0,sizeof(nwalCreateDmSAParams_t));
    memcpy(&dmSaParam.dmSaParam,&nwalDmSaParam[1],sizeof(nwalDmSAParams_t));
    memcpy(&dmSaParam.keyParam,&nwalSaDmKeyParam,sizeof(nwalSecKeyParams_t));
    dmSaParam.keyParam.pEncKey = nwalEncrKey;
    dmSaParam.keyParam.pAuthKey = nwalAuthKey;
    retValue = nwal_setDMSecAssoc(testNwGlobContext.nwalInstHandle,
                                  (nwal_AppId)TEST_NWAL_APP_ID_DM_ENC,
                                  &dmSaParam,
                                  &nwalDmEncSaHandle);
    if(retValue != nwal_OK)
    {
        System_printf("CORE: %d Data Mode Security Association (Decryption) FAILED,  \n",DNUM);
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Sideband IPSec Data Mode Test");
        return nwal_FALSE;
    }

    /* Send packet to encryption -> decryption and verifiy the content */
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
        System_printf("CORE: %d Could not allocate buffer count:%d num Pools: %d,  \n",DNUM,count,pMbufPool->numBufPools);
        return nwal_FALSE;
    }
    Osal_cache_op_measure_reset();
#ifdef NWAL_LIB_ENABLE_PROFILE
    nwal_sendDM_prof1_sum = 0;
    nwal_sendDM_prof2_sum = 0;
    nwal_sendDM_prof3_sum = 0;
    nwal_sendDM_prof4_sum = 0;
    nwal_sendDM_prof5_sum = 0;
    nwal_sendDM_sum = 0;
    nwal_sendDM_count_sum = 0;
#endif
    for(count=0;count < TEST_MAX_BURST;count++)
    {

        pPkt = Pktlib_allocPacket(pBufPool->heapHandle,TEST_PAYLOAD_LEN);
        if (pPkt == NULL)
        {
            System_printf("CORE: %d PKTLIB Allocation failed count:%d,  \n",DNUM,count);
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Sideband IPSec Data Mode Test");
            return nwal_FALSE;
        }

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

        Pktlib_setDataBufferLen(pPkt,TEST_PAYLOAD_LEN);
#ifndef __ARMv7
        Osal_writeBackCache(pDataBuffer,TEST_PAYLOAD_LEN);
#endif
        memset(&payloadInfo,0,sizeof(nwalDmTxPayloadInfo_t));
        //memset(iv,0,sizeof(iv));

        payloadInfo.pPkt = pPkt;
        payloadInfo.encOffset = 0;
        payloadInfo.authOffset = 0;
        payloadInfo.encSize = TEST_PAYLOAD_LEN;
        payloadInfo.authSize = TEST_PAYLOAD_LEN;
        payloadInfo.pEncIV = NULL;
        if(nwalDmSaParam[1].cipherMode != NWAL_SA_EALG_NULL)
        {
            payloadInfo.pEncIV = iv;
        }
        payloadInfo.pAuthIV = NULL;
        payloadInfo.aadSize = 0;
        payloadInfo.pAad = NULL;
        payloadInfo.appCtxId = (nwal_AppId)TEST_NWAL_CTX_ID_DM_ENC;

        fw_nwal_sendDM_start = fw_read_clock();
        retValue = nwal_sendDM(testNwGlobContext.nwalInstHandle,
                               nwalDmEncSaHandle,
                               &payloadInfo);

        fw_nwal_sendDM_end = fw_read_clock();
        fw_nwal_sendDM_prof += (fw_nwal_sendDM_end-fw_nwal_sendDM_start);
        fw_dm_send_count++;

        if(retValue != nwal_OK)
        {
            System_printf("CORE: %d nwal_sendDM (Encryption) FAILED,  \n",DNUM);
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Sideband IPSec Data Mode Test");
            return nwal_FALSE;
        }
        testNwGlobContext.numDMEncReqSent++;
    }

    System_printf("nwal_sendDM()Cycle Cost at App:%d \n",fw_nwal_sendDM_prof/fw_dm_send_count);
    testNwalDispCacheCycles(TEST_MAX_BURST);
    Osal_cache_op_measure_reset();

    utilCycleDelay(20000);
    nwal_pollDm(testNwGlobContext.nwalInstHandle,
                     nwal_POLL_DM_DEF_SB_SA_Q,
                     0,
                     1,
                     QMSS_PARAM_NOT_SPECIFIED,
                     NULL);

    fw_dm_poll_count++;
    while(testNwGlobContext.numDMEncReqSent != testNwGlobContext.numDMEncRespRecvd)
    {
        nwal_SystemFlush();
        nwal_pollDm(testNwGlobContext.nwalInstHandle,
                     nwal_POLL_DM_DEF_SB_SA_Q,
                     0,
                     1,
                     QMSS_PARAM_NOT_SPECIFIED,
                     NULL);

    fw_dm_poll_count++;
        /* Check for the packet received */
        utilCycleDelay(20000);
    }

#ifndef TEST_NWAL_DM_NO_LOOPBACK
    utilCycleDelay(20000);
    nwal_pollDm(testNwGlobContext.nwalInstHandle,
                     nwal_POLL_DM_DEF_SB_SA_Q,
                     0,
                     1,
                     QMSS_PARAM_NOT_SPECIFIED,
                     NULL);
    fw_dm_poll_count++;


    while(testNwGlobContext.numDMEncReqSent != testNwGlobContext.numDMDecRespRecvd)
    {
        nwal_SystemFlush();
        nwal_pollDm(testNwGlobContext.nwalInstHandle,
                     nwal_POLL_DM_DEF_SB_SA_Q,
                     0,
                     1,
                     QMSS_PARAM_NOT_SPECIFIED,
                     NULL);

        fw_dm_poll_count++;

        /* Check for the packet received */
        utilCycleDelay(20000);
    }
#endif

#ifdef NWAL_ENABLE_PROFILE
    System_printf("nwal_sendDM()-2 Cycle Cost at App:%d \n",fw_nwal_sendDM_prof/fw_dm_send_count);
    testNwalDispCacheCycles(TEST_MAX_BURST*2);
#endif
#ifdef NWAL_LIB_ENABLE_PROFILE
    System_printf("***** NWAL Data Mode Send Cycle Breakdown *****\n");
    if(nwal_sendDM_count_sum)
    {
        System_printf("Per DM Send:%d,Number of DM Send:%d, nwal_sendDM_prof1_sum:%d,nwal_sendDM_prof2_sum:%d \n ",
                       nwal_sendDM_sum/nwal_sendDM_count_sum,
                       nwal_sendDM_count_sum,
                       nwal_sendDM_prof1_sum/nwal_sendDM_count_sum,
                       nwal_sendDM_prof2_sum/nwal_sendDM_count_sum);
        System_printf("nwal_sendDM_prof3_sum:%d,nwal_sendDM_prof4_sum:%d,nwal_sendDM_prof5_sum:%d  \n",
                       nwal_sendDM_prof3_sum/nwal_sendDM_count_sum,
                       nwal_sendDM_prof4_sum/nwal_sendDM_count_sum,
                       nwal_sendDM_prof5_sum/nwal_sendDM_count_sum);
    }
#endif
    System_printf("------------- Data Mode Encode Channel Stats BEGIN ----------------\n");
    testGetDataModeStats(nwalDmEncSaHandle);
    System_printf("------------- Data Mode Decode Channel Stats BEGIN ----------------\n");
    testGetDataModeStats(nwalDmDecSaHandle);
    /* Delete the DM SA Channels */
    retValue = nwal_delDMSecAssoc(testNwGlobContext.nwalInstHandle,
                                  nwalDmEncSaHandle);
    if(retValue != nwal_OK)
    {
        System_printf("CORE: %d Deletion of Data Mode Security Association (Encryption) FAILED,  \n",DNUM);
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Sideband IPSec Data Mode Test");
        return nwal_FALSE;
    }

    retValue = nwal_delDMSecAssoc(testNwGlobContext.nwalInstHandle,
                                  nwalDmDecSaHandle);
    if(retValue != nwal_OK)
    {
        System_printf("CORE: %d Deletion of Data Mode Security Association (Decryption) FAILED,  \n",DNUM);
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Sideband IPSec Data Mode Test");
        return nwal_FALSE;
    }

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL Sideband IPSec Data Mode Test");
    return(nwal_TRUE);
}

/*****************************************************************************
 * FUNCTION PURPOSE: IPSec Channel Stats
 *****************************************************************************
 * DESCRIPTION: IPSec Channel Stats
 *****************************************************************************/
nwal_Bool_t  testGetIpSecStats (nwal_Handle  handle)
{
    Sa_IpsecStats_t ipSecStats;
    nwal_RetValue   retVal;

    memset(&ipSecStats,0,sizeof(Sa_IpsecStats_t));

    retVal = nwal_getSecAssocStats(testNwGlobContext.nwalInstHandle,handle,&ipSecStats);
    if(retVal != nwal_OK)
    {
        System_printf("CORE: %d Error getting IP Sec Stats: Ret Status: %d \n",
                       retVal);
        return(nwal_FALSE);
    }

    if((ipSecStats.pktEncHi) ||(ipSecStats.pktEncLo))
    {
        System_printf("------------- IPSec TX (Encryption Channel) Stats BEGIN --\n");
    }
    else
    {
        System_printf("------------- IPSec RX (Decryption Channel) Stats BEGIN --\n");
    }
    System_printf("IPSec replayOld:0x%x,replayDup:0x%x,authFail:0x%x \n",
                   ipSecStats.replayOld,ipSecStats.replayDup,ipSecStats.authFail);
    System_printf("IPSec txESN:0x%x,txSN:0x%x,rxESN:0x%x \n",
                   ipSecStats.txESN,ipSecStats.txSN,ipSecStats.rxESN);
    System_printf("IPSec pktEncHi:0x%x,pktEncLo:0x%x,pktDecHi:0x%x,pktDecLo:0x%x \n",
                   ipSecStats.pktEncHi,ipSecStats.pktEncLo,ipSecStats.pktDecHi,
                   ipSecStats.pktDecLo);
    System_printf("IPSec txByteCountHi:0x%x,txByteCountLo:0x%x,rxByteCountHi:0x%x,rxByteCountLo:0x%x \n",
                   ipSecStats.txByteCountHi,ipSecStats.txByteCountLo,
                   ipSecStats.rxByteCountHi,ipSecStats.rxByteCountLo);
    System_printf("------------- IPSec Stats END ----------------------------\n\n");
    return(nwal_TRUE);
}
/*****************************************************************************
 * FUNCTION PURPOSE: SA System Stats
 *****************************************************************************
 * DESCRIPTION: SA System Stats
 *****************************************************************************/
nwal_Bool_t  testGetSaSysStats ()
{
    Sa_SysStats_t   saSysStats;
    nwal_RetValue   retVal;

    memset(&saSysStats,0,sizeof(Sa_SysStats_t));

    retVal = nwal_getSASysStats(testNwGlobContext.nwalInstHandle,&saSysStats);
    if(retVal != nwal_OK)
    {
        System_printf("CORE: %d Error getting SA Sys Stats: Ret Status: %d \n",
                       retVal);
        return(nwal_FALSE);
    }

    System_printf("------------- SA System Stats BEGIN ---------------------------\n");
//    System_printf("SA System Stats errNoMem:0x%x,errCtx:0x%x,errEngine:0x%x,errProto:0x%x \n",
//                   saSysStats.errNoMem,saSysStats.errCtx,saSysStats.errEngine,saSysStats.errProto);
    System_printf("------------- SA System Stats END ----------------------------\n\n");
    return(nwal_TRUE);
}

#endif
nwal_Bool_t  testPktSendOut (uint16_t  dest_port)
{
    nwal_RetValue   retValue;
    uint8_t         countTx=0;
#ifdef NWAL_ENABLE_PROFILE
    unsigned int    fw_nwal_send_start,fw_nwal_send_end;
    unsigned int    fw_nwal_send_prof = 0;
#endif
    uint8_t count=0;

    /* Update the packet with correct UDP port */
    testNwalWrite16bits_m(testPkt,(TEST_PKT_UDP_OFFSET_BYTES+2),dest_port);
#ifdef NWAL_ENABLE_PROFILE
    Osal_cache_op_measure_reset();
#endif

    while(countTx < TEST_MAX_BURST)
    {

#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_send_start = fw_read_clock();
#endif
        retValue = nwal_sendRaw(testNwGlobContext.nwalInstHandle,
                                nwal_TRUE,
                                TEST_PKT_LEN,
                                testPkt);
#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_send_end = fw_read_clock();
        fw_nwal_send_prof += (fw_nwal_send_end - fw_nwal_send_start);
#endif

        if(retValue != nwal_OK)
        {
          System_printf("ERROR: nwal_sendRaw returned Error Code %d \n",
                      retValue);
          nwal_SystemFlush();
          testNwLocContext.TxErrDrop += 1;
          return nwal_FALSE;
        }
        else
        {
          testNwLocContext.numL4PktsSent++;
        }

        countTx++;
    }
#ifdef NWAL_ENABLE_PROFILE
    System_printf("\n Profile: nwal_sendRaw() Full Pkt Cycle Cost at App:%d \n",
                   fw_nwal_send_prof/countTx);
    testNwalDispCacheCycles(TEST_MAX_BURST);
#endif

    utilCycleDelay(20000);
    System_printf("CORE: %d Polling for RAW L4  %d packets\n",
                   DNUM,
                   TEST_MAX_BURST);

    testNwalInitPollProfile();
    testNwalPoll();

    
    while(testNwLocContext.numL4PktsSent != testNwLocContext.numL4PktsRecvd)
    {

        int32_t     userInput;
        if(count == 100)
        {
            System_printf("CORE: %d Enter -1 to continue polling -2 to get PA stats \n",DNUM);
            nwal_SystemFlush();
            scanf("%d",&userInput);
            if(userInput == 2)
            {
                nwal_getPAStats(testNwGlobContext.nwalInstHandle,
                            NWAL_TRANSID_SPIN_WAIT,
                            &testNwGlobContext.paStats,
                            0);
                testNWALCmdDispPaStats();
            }
            count = 0;
        }
        nwal_SystemFlush();

        testNwalPoll();

        /* Check for the packet received */
        utilCycleDelay(20000);
        count++;

    }
    System_printf("CORE: %d SEND RAW packets sent %d  number L4 packet recvd: %d, App ID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL4PktsSent,
                    testNwLocContext.numL4PktsRecvd,
                    rcvd_AppID);
    testNwalDispPollProfile();

    return nwal_TRUE;
}

#ifdef NWAL_LIB_ENABLE_PROFILE
extern unsigned int nwal_IpCheckSum_prof_sum;
extern unsigned int nwal_IpCheckSum_prof_count;
extern unsigned int nwal_UdpCheckSum_prof_sum;
extern unsigned int nwal_UdpCheckSum_prof_count;
#endif


/*****************************************************************************
 * FUNCTION PURPOSE: Test for nwal_Send
 *****************************************************************************
 * DESCRIPTION: The function will be called to send packets out
 *****************************************************************************/
nwal_Bool_t  testNwalSend (nwal_Handle  handle,
                           nwal_Bool_t  isSA,
                           nwal_Handle  inHandle,
                           nwal_Handle  outHandle)
{
     nwal_RetValue      retValue;
     /* Share the same memory buffer pool for the TX packet */
     nwalMbufPool_t*    pMbufPool;
     uint8_t            count = 0,countTx=0;
     nwalBufPool_t*     pBufPool;
     Ti_Pkt*            pPkt;
     uint8_t*           pDataBuffer;
     uint32_t           dataLen;
     nwalTxPktInfo_t    txPktInfo;
#ifdef NWAL_ENABLE_PROFILE
     unsigned int       fw_nwal_send_prep_start,fw_nwal_send_prep_end,fw_nwal_hdr_prep_end;
     unsigned int       fw_nwal_send_start,fw_nwal_send_end;
     unsigned int       fw_nwal_send_prep_prof = 0;
     unsigned int       fw_nwal_protoHdr_prof = 0;
     unsigned int       fw_nwal_send_prof = 0;

     Osal_cache_op_measure_reset();
#endif

#ifdef NWAL_LIB_ENABLE_PROFILE
    nwal_IpCheckSum_prof_sum = 0;
    nwal_IpCheckSum_prof_count = 0;
    nwal_UdpCheckSum_prof_sum = 0;
    nwal_UdpCheckSum_prof_count = 0;
#endif

     while(countTx < TEST_MAX_BURST)
     {
#ifdef NWAL_ENABLE_PROFILE
         fw_nwal_send_prep_start = fw_read_clock();
#endif
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

#ifndef __ARMv7
        Osal_writeBackCache(pDataBuffer,TEST_PAYLOAD_LEN);
#endif
        Pktlib_setDataBufferLen(pPkt,TEST_PAYLOAD_LEN);
        memset(&txPktInfo,0,sizeof(nwalTxPktInfo_t));
        txPktInfo.pPkt = pPkt;
        txPktInfo.lpbackPass = nwal_TRUE;

#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_send_prep_end = fw_read_clock();
#endif

        /* Update the L2/L3/L4 protocol header */
        retValue = nwal_updateProtoHdr(testNwGlobContext.nwalInstHandle,
                                       handle,
                                       &txPktInfo);
#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_hdr_prep_end = fw_read_clock();
#endif
        if(retValue != nwal_OK)
        {
            return nwal_FALSE;
        }

#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_send_start = fw_read_clock();
#endif
        retValue = nwal_send(testNwGlobContext.nwalInstHandle,
                             handle,
                             &txPktInfo);
#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_send_end = fw_read_clock();
#endif
        if(retValue != nwal_OK)
        {
            return nwal_FALSE;
        }
#ifdef NWAL_ENABLE_PROFILE
        fw_nwal_send_prep_prof +=(fw_nwal_send_prep_end - fw_nwal_send_prep_start);
        fw_nwal_protoHdr_prof +=(fw_nwal_hdr_prep_end - fw_nwal_send_prep_end);
        fw_nwal_send_prof += (fw_nwal_send_end - fw_nwal_send_start);
#endif
        testNwLocContext.numL4PktsSent++;
        countTx++;
     }

#ifdef NWAL_ENABLE_PROFILE
     System_printf("Profile: nwal_send() Full Pkt Cycle Cost at App:Prep %d, ProtoHdr:%d, Send:%d \n",
                   fw_nwal_send_prep_prof/countTx,
                   fw_nwal_protoHdr_prof/countTx,
                   fw_nwal_send_prof/countTx);
     testNwalDispCacheCycles(TEST_MAX_BURST);
#endif

    System_printf("CORE: %d Polling for %d packets\n",
                   DNUM,
                   TEST_MAX_BURST);
    testNwalInitPollProfile();
    utilCycleDelay(20000);
    testNwalPoll();

#ifdef NWAL_ENABLE_SA
    if(isSA)
    {
        testGetIpSecStats(inHandle);
        testGetIpSecStats(outHandle);
        testGetSaSysStats();
    }
#endif
    while(testNwLocContext.numL4PktsSent != testNwLocContext.numL4PktsRecvd)
    {
        nwal_SystemFlush();
        testNwalPoll();
        System_printf("CORE %d Polling for MAC/IP/UDP[Local] %d packets\n",
                       DNUM,
                       (testNwLocContext.numL4PktsSent-testNwLocContext.numL4PktsRecvd));
        /* Check for the packet received */
        utilCycleDelay(20000);
    }
    testNwalDispPollProfile();
#ifdef NWAL_LIB_ENABLE_PROFILE
    if(nwal_IpCheckSum_prof_count)
    {
       System_printf(" NWAL IP Checksum Profile for Payload Size : %d\n",TEST_PAYLOAD_LEN);
        System_printf("NWAL LIB profile:nwal_IpCheckSum Per Packet:%d nwal_IpCheckSum count:%d \n",
                           (nwal_IpCheckSum_prof_sum)/nwal_IpCheckSum_prof_count,
                           nwal_IpCheckSum_prof_count);
    }
    if(nwal_UdpCheckSum_prof_sum)
    {
         System_printf(" NWAL UDP Checksum Profile for Payload Size : %d\n",TEST_PAYLOAD_LEN);
         System_printf("NWAL LIB profile:nwal_UdpCheckSum Per Packet:%d nwal_UdpCheckSum count:%d \n",
                           (nwal_UdpCheckSum_prof_sum)/nwal_UdpCheckSum_prof_count,
                           nwal_UdpCheckSum_prof_count);
    }
#endif
     return nwal_TRUE;
}
/*****************************************************************************
 * FUNCTION PURPOSE: Test for NWAL configuration of L2/L3/L4 and packet send and receive
 *****************************************************************************
 * DESCRIPTION: The function will be called to send packets out
 *****************************************************************************/
nwal_Bool_t  nwalTestPktSendRecv ()
{
    nwal_RetValue       retVal;
    nwalGlobCxtInfo_t   nwalGlobCxt;
    uint8_t             count;
    uint32_t            lockKey;

    System_printf("\n\n CORE: %d ------------- NWAL MODULE UNIT TEST BEGIN ------------- \n",DNUM);
    retVal =
    nwal_getGlobCxtInfo(testNwGlobContext.nwalInstHandle,
                        &nwalGlobCxt);
    if(retVal == nwal_OK)
    {
        System_printf("*******NWAL Global Context Info Dump Begin *******\n");
        System_printf("rxPaSaFlowId:%d,rxSaPaFlowId:%d rxDefPktQ:0x%x \n",
                       nwalGlobCxt.rxPaSaFlowId,nwalGlobCxt.rxSaPaFlowId,
                        nwalGlobCxt.rxDefPktQ);
        System_printf("defFlowQ:0x%x,passCppiHandle:0x%x extErr:%d \n",
                       nwalGlobCxt.defFlowQ,nwalGlobCxt.passCppiHandle,
                       nwalGlobCxt.extErr);
        for(count=0;count < nwalGlobCxt.numPaPDSPs;count++)
        {
            System_printf("NetCP PASS PDSP - %d Version:0x%x \n",
                           count,nwalGlobCxt.pdspVer[count]);
        }
        System_printf("*******NWAL Global Context Info Dump End *******\n\n");
    }
    if(!DNUM)
    {
#ifdef NWAL_ENABLE_SA
        if(testConfigDmSa() == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testConfigDmSa() returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }
#endif

        if(testMacMatchRoute(FALSE) == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testMacMatchRoute returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }

        if(testMacMatchRoute(TRUE) == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testMacMatchRoute with remMac returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }
        if(testMacMatchRouteVLANPriority(FALSE) == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testIpMatchRoute with VLAN Prioirity returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }
#ifndef __LINUX_USER_SPACE
        if(testMacFailRoute() == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testMacMatchRoute returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }
#endif

        if(testIpMatchRoute(FALSE) == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testIpMatchRoute returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }
        if(testIpMatchRoute(TRUE) == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testIpMatchRoute with remIp returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }

        if(testIpMatchRouteDSCPPriority() == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testIpMatchRoute with DSCP Priority returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }

        if(testBypassMacIpMatchRoute() == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testBypassMacIpMatchRoute returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }
        if(testIpFailRoute() == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testIpFailRoute returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }      
        if(testFragIpException() == nwal_FALSE)
        {
            System_printf("CORE:%d ERROR: testFragIpException returned Error\n",DNUM);
            nwal_SystemFlush();
            return nwal_FALSE;
        }
        
        nwal_SystemFlush();

    }

    /* Open MAC/IP/UDP Handle */
    if(testConfigMacIpUdp() == nwal_FALSE)
    {
        System_printf("CORE:%d ERROR: testConfigMacIpUdp returned Error\n",DNUM);
        nwal_SystemFlush();
        return nwal_FALSE;
    }
    if(testConfigMacIpv6Udp() == nwal_FALSE)
    {
        System_printf("CORE:%d ERROR: testConfigMacIpUdp returned Error\n",DNUM);
        nwal_SystemFlush();
        return nwal_FALSE;
    }
#ifdef NWAL_ENABLE_SA
    if(testConfigMacIpv6IpSecIpv6Udp() == nwal_FALSE)
    {
        System_printf("CORE:%d ERROR: testConfigMacIpv6IpSecIpv6Udp returned Error\n",DNUM);
        nwal_SystemFlush();
        return nwal_FALSE;
    }
    
    if(testConfigMacIpSecIpUdp() == nwal_FALSE)
    {
        System_printf("CORE:%d ERROR: testConfigMacIpSecIpUdp returned Error\n",DNUM);
        nwal_SystemFlush();
        return nwal_FALSE;
    }
#endif
    
    /* Check for all command response being received */
    if(testNwLocContext.numPendingCfg)
    {
        System_printf("CORE: %d ERROR: Timeout waiting for ACK from NWAL. Num of pending config %d \n",
                    DNUM,
                    testNwLocContext.numPendingCfg);
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IP/UDP Raw Packet Test");
    if(!testPktSendOut(TEST_NWAL_BASE_UDP_PORT + DNUM))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP/UDP Raw Packet Test");
        return nwal_FALSE;
    }
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IP/UDP Raw Packet Test");

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IP/UDP Fast Send Test");
    if(!testNwalFastSendPkt((NWAL_TX_FLAG1_DO_IPV4_CHKSUM |
                             NWAL_TX_FLAG1_DO_UDP_CHKSUM),nwal_FALSE, nwal_FALSE))

    {
        fw_logTest(nwal_TRUE,nwal_TRUE,"NWAL MAC/IP/UDP Fast Send Test");
        return nwal_FALSE;
    }
    testNwLocContext.numL4PktsSent += TEST_MAX_BURST;
    System_printf("CORE: %d Polling for UDP %d packets\n",
                   DNUM,
                   TEST_MAX_BURST);
    testNwalInitPollProfile();
    utilCycleDelay(20000);
    testNwalPoll();


    while(testNwLocContext.numL4PktsSent != testNwLocContext.numL4PktsRecvd)
    {
        nwal_SystemFlush();
        testNwalPoll();
        System_printf("CORE %d Polling for UDP %d packets\n",
                       DNUM,
                       (testNwLocContext.numL4PktsSent-testNwLocContext.numL4PktsRecvd));
        /* Check for the packet received */
        utilCycleDelay(20000);
    }

    System_printf("CORE: %d UDP packets sent %d  number L4 packet recvd: %d, AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL4PktsSent,
                    testNwLocContext.numL4PktsRecvd,
                    rcvd_AppID);
    testNwalDispPollProfile();
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IP/UDP Fast Send Test");


    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IP/UDP Full Send with proto hdr Test");
    if(!testNwalSend(udpHandle,
                     nwal_FALSE,
                     nwal_HANDLE_INVALID,
                     nwal_HANDLE_INVALID))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP/UDP Full Send with proto hdr Test");
        return nwal_FALSE;
    }
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IP/UDP Full Send with proto hdr Test");

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IPv6/UDP Full Send with proto hdr Test");
    if(!testNwalSend(nwalIpv6UdpHandle,
                     nwal_FALSE,
                     nwal_HANDLE_INVALID,
                     nwal_HANDLE_INVALID))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IPv6/UDP Full Send with proto hdr Test");
        return nwal_FALSE;
    }
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IPv6/UDP Full Send with proto hdr Test");

#ifdef NWAL_ENABLE_SA

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IPSec[ESP]/IP/UDP Full Proto Hdr Send Test");
    if(!testNwalSend(saESPUdpHandle,
                     nwal_TRUE,
                     nwalSecAssocESPInHandle,
                     nwalSecAssocESPOutHandle))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IPSec[ESP]/IP/UDP Full Proto Hdr Send Test");
        return nwal_FALSE;
    }

    System_printf("CORE: %d MAC/IPSec[ESP]/IP/UDP[Local] packets sent %d  number L4 packet recvd: %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL4PktsSent,
                    testNwLocContext.numL4PktsRecvd,
                    rcvd_AppID);

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IPSec[ESP]/IP/UDP Full Proto Hdr Send Test");

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IPSec[ESP]-Transport/UDP Full Proto Hdr Send Test");
    if(!testNwalSend(saESPTransUdpHandle,
                     nwal_TRUE,
                     nwalSecAssocTransESPInHandle,
                     nwalSecAssocTransESPOutHandle))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IPSec[ESP]-Transport/UDP Full Proto Hdr Send Test");
        return nwal_FALSE;
    }

    System_printf("CORE: %d NWAL MAC/IPSec[ESP]-Transport/UDP packets sent %d  number L4 packet recvd: %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL4PktsSent,
                    testNwLocContext.numL4PktsRecvd,
                    rcvd_AppID);

    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IPSec[ESP]-Transport/UDP Full Proto Hdr Send Test");

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IPSec[AH]/IP/UDP Full Proto Hdr Send Test");
    if(!testNwalSend(saAHUdpHandle,
                     nwal_TRUE,
                     nwalSecAssocAHInHandle,
                     nwalSecAssocAHOutHandle))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IPSec[AH]/IP/UDP Full Proto Hdr Send Test");
        return nwal_FALSE;
    }

    System_printf("CORE: %d MAC/IPSec[AH]/IP/UDP[Local] packets sent %d  number L4 packet recvd: %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL4PktsSent,
                    testNwLocContext.numL4PktsRecvd,
                    rcvd_AppID);
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IPSec[AH]/IP/UDP Full Proto Hdr Send Test");

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IPSec[AH]-Transport/UDP Full Proto Hdr Send Test");
    if(!testNwalSend(saAHTransUdpHandle,
                     nwal_TRUE,
                     nwalSecAssocTransAHInHandle,
                     nwalSecAssocTransAHOutHandle))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IPSec[AH]-Transport/UDP Full Proto Hdr Send Test");
        return nwal_FALSE;
    }

    System_printf("CORE: %d NWAL MAC/IPSec[AH]-Transport/UDP packets sent %d  number L4 packet recvd: %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL4PktsSent,
                    testNwLocContext.numL4PktsRecvd,
                    rcvd_AppID);
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IPSec[AH]-Transport/UDP Full Proto Hdr Send Test");


    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IPv6/IPSec[ESP]/IPv6/UDP Full Proto Hdr Send Test");
    if(!testNwalSend(saIpv6ESPUdpHandle,
                     nwal_TRUE,
                     nwalIpv6SecAssocESPInHandle,
                     nwalIPv6SecAssocESPOutHandle))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IPv6/IPSec[ESP]/IPv6/UDP Full Proto Hdr Send Test");
        return nwal_FALSE;
    }

    System_printf("CORE: %d NWAL MAC/IPv6/IPSec[ESP]/IPv6/UDP [Local] packets sent %d  number L4 packet recvd: %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL4PktsSent,
                    testNwLocContext.numL4PktsRecvd,
                    rcvd_AppID);
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IPv6/IPSec[ESP]/IPv6/UDP Full Proto Hdr Send Test");

    fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IPv6/IPSec[AH]/IPv6/UDP Full Proto Hdr Send Test");
    if(!testNwalSend(saIpv6AHUdpHandle,
                     nwal_TRUE,
                     nwalIpv6SecAssocAHInHandle,
                     nwalIpv6SecAssocAHOutHandle))
    {
        fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IPv6/IPSec[AH]/IPv6/UDP Full Proto Hdr Send Test");
        return nwal_FALSE;
    }

    System_printf("CORE: %d MAC/IPv6/IPSec[AH]/IPv6/UDP[Local] packets sent %d  number L4 packet recvd: %d AppID: 0x%x\n",
                    DNUM,
                    testNwLocContext.numL4PktsSent,
                    testNwLocContext.numL4PktsRecvd,
                    rcvd_AppID);
    fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IPv6/IPSec[AH]/IPv6/UDP Full Proto Hdr Send Test");
#endif

    if(DNUM)
    {
        /* For all cores other than Core 0 block until remote core is created by Core -0 */

        /* Verify Packets TX and RX on the UDP port created at remote Core 0 */
        /* Block until Core 0 had configured the UDP port */
        while(testNwGlobContext.remUDPConn[DNUM-1].state != TEST_NWAL_HANDLE_STATE_OPEN)
        {
            Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
        }
        /* Refresh the connection created remotely */
        retVal = nwal_refreshConn(testNwGlobContext.nwalInstHandle,testNwGlobContext.remUDPConn[DNUM-1].handle);
        if(retVal != nwal_OK)
        {
            System_printf("CORE: %d nwal_refreshConn() Failed for MAC/IP/UDP \n",
                            DNUM);
        }

        fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IP/UDP[Remote] Full Proto Hdr Send Test");
        /* Test TX and RX for the port */
         if(!testNwalSend(testNwGlobContext.remUDPConn[DNUM-1].handle,
                         nwal_FALSE,
                         nwal_HANDLE_INVALID,
                         nwal_HANDLE_INVALID))
        {
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IP/UDP[Remote] Full Proto Hdr Send Test");
            return nwal_FALSE;
        }
        fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IP/UDP[Remote] Full Proto Hdr Send Test");

#ifdef NWAL_ENABLE_SA
        /* Verify Packets TX and RX on the UDP port created at remote Core 0 */
        /* Block until Core 0 had configured the UDP port */
        while(testNwGlobContext.remIPSecUDPConn[DNUM-1].state != TEST_NWAL_HANDLE_STATE_OPEN)
        {
            Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
        }
        /* Refresh the connection created remotely */
        retVal = nwal_refreshConn(testNwGlobContext.nwalInstHandle,testNwGlobContext.remIPSecUDPConn[DNUM-1].handle);
        if(retVal != nwal_OK)
        {
            System_printf("CORE: %d nwal_refreshConn() Failed for MAC/IPSec/IP/UDP \n",
                            DNUM);
        }

        fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL MAC/IPSec/IP/UDP[Remote] Full Proto Hdr Send Test");
        /* Test TX and RX for the port */
         if(!testNwalSend(testNwGlobContext.remIPSecUDPConn[DNUM-1].handle,
                         nwal_FALSE,
                         nwal_HANDLE_INVALID,
                         nwal_HANDLE_INVALID))
        {
            fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL MAC/IPSec/IP/UDP[Remote] Full Proto Hdr Send Test");
            nwal_SystemFlush();
            return nwal_FALSE;
        }
        fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL MAC/IPSec/IP/UDP[Remote] Full Proto Hdr Send Test");
#endif
    }

    /* Check for the all command response being received*/
    utilCycleDelay(1000);

     /* Check for all command response being received */
    if(testNwLocContext.numPendingCfg)
    {
        System_printf("CORE: %d ERROR: Timeout waiting for ACK from NWAL. Num of pending config %d, AppID: 0x%x \n",
                    DNUM,testNwLocContext.numPendingCfg,
                    rcvd_AppID);
        nwal_SystemFlush();
        return nwal_FALSE;
    }
    if(!DNUM)
    {
        int32_t     userInput;
        System_printf("CORE: %d **** BASIC UNIT TEST SUMMARY: PASSED:%d,FAILED:%d,TOTAL:%d **** \n\n",
                       DNUM,
                       passCount,
                       (testCount - passCount),
                       testCount);

        utilCycleDelay(20000);
        while(testNwGlobContext.numCoresTested != (testNwGlobContext.numCoresStarted -1))
        {
            Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
#ifdef __ARMv7
          pthread_yield();
#endif
        }
#ifdef __ARMv7
        /* Test for remote Fast Path DSP cores */
        System_printf("CORE: %d Enter 1 to continue testing Fast path DSP cores or any other input to Exit \n",DNUM);
        nwal_SystemFlush();
        scanf("%d",&userInput);
        if(userInput == 1)
        {
            uint16_t  dest_port;
            nwal_getPAStats(testNwGlobContext.nwalInstHandle,
                            NWAL_TRANSID_SPIN_WAIT,
                            &testNwGlobContext.paStats,
                            0);
            testNWALCmdDispPaStats();
            testAddRemFPL4();
            fw_logTest(nwal_TRUE,nwal_FALSE,"NWAL Remote Fast Path Core PKT TX/RX Test");
            for(count =0;count < CPU_NUM_REM_FAST_PATH_CORES;count++)
            {
                dest_port = (TEST_NWAL_BASE_REM_FP_UDP_PORT+count);
                if(testPktSendOut(dest_port) != nwal_TRUE)
                {
                    fw_logTest(nwal_FALSE,nwal_TRUE,"NWAL Remote Fast Path Core PKT TX/RX Test");
                    nwal_SystemFlush();
                    return nwal_FALSE;
                }
                System_printf("CORE: %d TX/RX test for Fast Path Core %d /Max Fast Path Core:%d Successful\n",
                               DNUM,
                               (count+1),
                               TEST_NWAL_BASE_REM_FP_UDP_PORT);
                nwal_SystemFlush();
            }
            fw_logTest(nwal_FALSE,nwal_FALSE,"NWAL Remote Fast Path Core PKT TX/RX Test");
        }
#endif
        memset(&testNwGlobContext.paStats,0,sizeof(paSysStats_t));
        nwal_getPAStats(testNwGlobContext.nwalInstHandle,
                        NWAL_TRANSID_SPIN_WAIT,
                        &testNwGlobContext.paStats,
                        0);
        testNWALCmdDispPaStats();
    }

    Osal_nwalCsEnter(&lockKey);
    Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
    testNwGlobContext.numCoresTested++;
    Osal_writeBackCache(&testNwGlobContext,sizeof(testNwGlobContext));
    Osal_nwalCsExit(lockKey);

    System_printf("CORE: %d **** UNIT TEST SUMMARY: PASSED:%d,FAILED:%d,TOTAL:%d **** \n",
                   DNUM,
                   passCount,
                   (testCount - passCount),
                   testCount);
    System_printf("CORE: %d **** NWAL MODULE UNIT TEST END. **** \n",DNUM);
    nwal_SystemFlush();


    return(nwal_TRUE);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Test for NWAL configuration of L2/L3/L4 and packet send and receive
 *****************************************************************************
 * DESCRIPTION: The function will be called to send packets out
 *****************************************************************************/
#ifdef __LINUX_USER_SPACE
 void  nwalTest(int id)
#else
void  nwalTest(UArg a0, UArg a1)
#endif
{
#ifdef __ARMv7
    uint16_t            count=0;
    uint32_t            lockKey;
    /* Common Initialization for all cores */
    while(count < TEST_MAX_NUM_TRANS)
    {
        testNwLocContext.transInfos[count].transId = count;
        count++;
    }
    core_id = id;
    memcpy(&testNwLocContext.nwalLocCfg,&nwalLocCfg,sizeof(nwalLocCfg_t));
    testNwLocContext.state = TEST_NW_CXT_LOC_ACTIVE;
    Osal_nwalCsEnter(&lockKey);
    Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
    testNwGlobContext.numCoresStarted++;
    Osal_writeBackCache(&testNwGlobContext,sizeof(testNwGlobContext));
    Osal_nwalCsExit(lockKey);
#endif

    if(DNUM)
    {
        while(1)
        {
          Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
          /* Invalidate the Global context */
          if(testNwGlobContext.state == TEST_NW_CXT_GLOB_RES_ALLOC_COMPLETE)
          {
              break;
          }
#ifdef __ARMv7
          pthread_yield();
#endif
        }
    }
    nwalTestPktSendRecv();
    nwal_SystemFlush();

}


/*****************************************************************************
 * FUNCTION PURPOSE: Polls different RX queues for the packets or command responses
 *****************************************************************************
 * DESCRIPTION: The function will be called for polling messages/packets
 *****************************************************************************/
void testNwalPoll (void )
{
    uint16_t        numRxPkts;
    unsigned int    fw_nwal_pollPkt_start,fw_nwal_pollPkt_end;
    int qosQNum;
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

    if((testNwLocContext.state & TEST_NW_CXT_LOC_ACTIVE) != TEST_NW_CXT_LOC_ACTIVE)
    {
        /* Resource not yet initialized.
         * Return
         */
        return;
    }

    if(!DNUM)
    {
       fw_nwal_pollPkt_start = fw_read_clock();
        /* Poll for common L2/L3 packets */
       numRxPkts = nwal_pollPkt( testNwGlobContext.nwalInstHandle,
                                 nwal_POLL_DEFAULT_GLOB_PKT_Q,
                                 0,
                                 NWAL_MAX_RX_PKT_THRESHOLD,
                                 QMSS_PARAM_NOT_SPECIFIED,
                                 NULL);
       fw_nwal_pollPkt_end =fw_read_clock();
       fw_nwal_pollPkt_sum += (fw_nwal_pollPkt_end - fw_nwal_pollPkt_start);
       fw_nwal_PollCount_sum++;
       fw_nwal_PollPktCount_sum += numRxPkts;
    }

    fw_nwal_pollPkt_start = fw_read_clock();
    numRxPkts = nwal_pollPkt(testNwGlobContext.nwalInstHandle,
                             nwal_POLL_DEFAULT_PER_PROC_PKT_Q,
                             0,
                             NWAL_MAX_RX_PKT_THRESHOLD,
                             QMSS_PARAM_NOT_SPECIFIED,
                             NULL);
    fw_nwal_pollPkt_end = fw_read_clock();
    fw_nwal_pollPkt_sum += (fw_nwal_pollPkt_end - fw_nwal_pollPkt_start);
    fw_nwal_PollCount_sum++;
    fw_nwal_PollPktCount_sum += numRxPkts;

    for(qosQNum=0; qosQNum< 5;qosQNum++)
    {
       fw_nwal_pollPkt_start = fw_read_clock();
       numRxPkts = nwal_pollPkt(testNwGlobContext.nwalInstHandle,
                                 nwal_POLL_APP_MANAGED_PKT_Q,
                                 0,
                                 NWAL_MAX_RX_PKT_THRESHOLD,
                                 testNwGlobContext.qosQ[qosQNum],
                                 NULL);
        fw_nwal_pollPkt_end = fw_read_clock();
        fw_nwal_pollPkt_sum += (fw_nwal_pollPkt_end - fw_nwal_pollPkt_start);
        fw_nwal_PollCount_sum++;
        fw_nwal_PollPktCount_sum += numRxPkts;
    }
}
