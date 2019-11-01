/*
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
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




#ifdef _TMS320C6X
#pragma DATA_SECTION(t3pkt, ".testPkts")
#pragma DATA_ALIGN(t3pkt, 8)
static uint8_t t3pkt[256];
#pragma DATA_SECTION(t3pkt_template, ".testPkts")
#pragma DATA_ALIGN(t3pkt_template, 8)
static uint8_t t3pkt_template[] = {
#else
static uint8_t t3pkt[256] __attribute__ ((aligned (8)));
static uint8_t t3pkt_template[] __attribute__ ((aligned (8))) = {
#endif
  /* MAC header */
  0x00, 0x01, 0x02, 0x03, 0x04, 0xaa,
  0x00, 0xe0, 0xa6, 0x66, 0x57, 0x04,
  0x08, 0x00,

  /* IP header */
  0x45, 0x00,
  0x00, 0x6c,  /* Length (including this header) */
  0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
  0x00, 0x00,  /* Header checksum */
  0x9e, 0xda, 0x6d, 0x0a, 0x01, 0x02, 0x03, 0x04,

  /* UDP header */
  0x12, 0x34, 0x05, 0x55,
  0x00, 0x58,  /* Length, including this header */
  0x00, 0x00,  /* Header checksum */

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

/* Offsets to length fields */
#define T3_PKT_OFFSET_IP_LEN		16
#define T3_PKT_OFFSET_UDP_LEN       38

/* The pseudo header checksum of the packet except for the 16 bit length */
#define T3_PKT_PSEUDO_HDR_CHKSUM_SANS_LEN  0x0ffc

/* IP header checksum configuration */
static paTxChksum_t t3pktIpChksum = {

    14,     /* Start offset of IP header */
    20,     /* Length of IP header */
    10,     /* Offset to checksum location RELATIVE TO THE START OF THE IP HEADER */
    0,      /* Initial sum */
    1       /* computed value of 0 written as -0 */

};


/* The UDP checksum */
static paTxChksum_t t3pktUdpChksum = {

    34,     /* Start offset of UDP header */
    88,     /* Checksum length (UDP payload + UDP checksum) */
    6,      /* Offset to checksum location RELATIVE TO THE START OF THE UDP HEADER */
    0x1054, /* Initial value is IPv4 pseudo header checksum value */
    1       /* computed value of 0 written as -0 */

};

/* CRC */
static paCmdCrcOp_t t3pktCrc = {

    0,     /* ctrlbit */
    42,     /* startoffset */
    78,      /* len */
    0,
    0,
    0,
    78, /* crcoffset */
    2,     /* crcSize */
    0      /* frametype */

};

static paCrcConfig_t   t3CrcCfg = {
                                    0,         /* ctrlBitfield */
                                    pa_CRC_SIZE_16,
                                    0x80050000,                    /* polynomial */
                                    0                              /* initValue */
                                  };




/* All packets in this test use a single destination address ethernet
 * routing for the L2 lookup in PA */
static paEthInfo_t testPaEthInfo[] =  {

  { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Dest mac */
    0,      /* Vlan      */
    0,      /* ethertype */
    0,      /* mpls tag  */
    0       /* input EMAC port */
    }

  };

/* IP lookup criteria. One for each IP packet. Only one distinguishing value
 * is really required, but it doesn't hurt to put more information */
static paIpInfo_t testPaIpInfo[] =  {

   {  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Src IP */
      { 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* Dest IP */
      0,        /* SPI */
      0,        /* flow */
      pa_IPV4,  /* IP type */
      0,        /* GRE proto */
      17,       /* Protocol type = UDP */
      0,        /* TOS */
      0,        /* TOS care */
      0         /* SCTP port */
   }
};


/* Port matching information */
uint16_t  testPaPortInfo[] = {

    0x0555

};

#define T3_NUM_CRC_OP_CMD_TO_TEST     2

#ifdef _TMS320C6X
#pragma DATA_SECTION (crcTestPayload0, ".testPkts")
#endif
static uint8_t crcTestPayload0[] =
{
     0x08 ,0x00 ,0x27 ,0x68 ,0xa6 ,0xee ,0xb4 ,0x99,
     0x4c ,0x91 ,0x2a ,0xe4 ,0x08 ,0x00 ,0x45 ,0xb8,
     0x00 ,0x36 ,0xfb ,0xb0 ,0x00 ,0x00 ,0x80 ,0x11,
     0xf4 ,0x46 ,0xc0 ,0xa8 ,0x64 ,0xaa ,0xc0 ,0xa8,
     0x64 ,0x0c ,0xd2 ,0xf3 ,0xa8 ,0xb2 ,0x00 ,0x22,
     0x00 ,0x00 ,0x84 ,0xe6 ,0x01 ,0x10 ,0x20 ,0x02,
     0x0f ,0xe0 ,0x00 ,0x00 ,0x00 ,0x04 ,0x18 ,0x30,
     0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x84 ,0x08 ,0x30,
     0x32 ,0x00 ,0xBE ,0xEF
}; // expected crc 0x03a8

#define T3_CRC_TEST_EXPECTED_CRC  0x03A8
#define T3_CRC_TEST_CRC_SIZE      2
#define T3_CRC_TEST_PKTLEN        ( sizeof (crcTestPayload0) )
#define T3_CRC_TEST_START_OFFSET  45
#define T3_CRC_TEST_LEN           ( T3_CRC_TEST_PKTLEN - T3_CRC_TEST_START_OFFSET - T3_CRC_TEST_CRC_SIZE )

#ifdef _TMS320C6X
#pragma DATA_SECTION (crcTestPayload0, ".testPkts")
#endif
static paCmdCrcOp_t crcTestInfo[T3_NUM_CRC_OP_CMD_TO_TEST] =
{
      {
        pa_CRC_OP_CRC_RESULT_FOLLOW_PAYLOAD, /* Ctrl Bit Filed */
        T3_CRC_TEST_START_OFFSET,/* startOffset */
        T3_CRC_TEST_LEN, /* len =  */
        0, /* lenOffset - not used */
        0, /* lenMask - not used */
        0, /* lenAdjust - not used */
        1000, /* crcOffset - to be upated later */
        T3_CRC_TEST_CRC_SIZE, /* CrcSize */
        0, /* Framttype - not used */
        0  /* initVal */
      },
      {
        0, /* Ctrl Bit Filed */
        T3_CRC_TEST_START_OFFSET, /* startOffset */
        T3_CRC_TEST_LEN, /* len  */
        0, /* lenOffset - not used */
        0, /* lenMask - not used */
        0, /* lenAdjust - not used */
        T3_CRC_TEST_LEN, /* crcOffset */
        T3_CRC_TEST_CRC_SIZE, /* CrcSize */
        0, /* Framttype - not used */
        0  /* initVal */
      }
};


/* Nothing past this point */
