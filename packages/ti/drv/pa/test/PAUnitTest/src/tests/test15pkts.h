/*
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef TEST15PKTS_H_
#define TEST15PKTS_H_

/* Valid rx MAC addresses used during the test */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t15EthInfo, ".testPkts")
#endif
static paEthInfo_t t15EthInfo[] =  {

    {
    	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 	{ 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
    },

     {
    	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 	{ 0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09 },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
    }


};


/* Outer IP configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t15OuterIpInfo, ".testPkts")
#endif
static t15IpPaSetup_t t15OuterIpInfo[] =  {

	/* ------- Entry 0 ----------- */
	{
		0,			/* Sequential ID */
		0,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		1,			/* L2 Handle Index */
		0,			/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},

	/* ------- Entry 1 ----------- */
	{
		1,			/* Sequential ID */
		1,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED, /* LUT1 Instance */
		1,			/* L2 Handle Index */
    2,                        /* Route Index - continue parse lut1 */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 204, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},
};

/* The following configurations link to an IP header (nested inner IP) */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t15InnerIpInfo, ".testPkts")
#endif
static t15IpPaSetup_t t15InnerIpInfo[] =  {

	/* ------- Entry 0 -----------
	 * match IP dest only  */
	{
		0,												/* Sequential ID */
		T15_NUM_LOCAL_L3_OUTER_IP_HANDLES + 0,			/* Local Handle Index */
        pa_LUT_INST_NOT_SPECIFIED,                      /* LUT1 Instance */
		1,												/* L3 Outer IP Handle Index */
                                                        /* Note: It is entry 23 */
		0,												/* Route Index - route to host */
		{
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  	 /* IP Source address */
			{ 10, 11, 12, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI  */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE		/* Set to true when the command is acked */
	},
};

/* Udp Header */
typedef struct t15UdpHdr_s {
  uint8_t srcPort[2];
  uint8_t dstPort[2];
  uint16_t len;
  uint16_t checksum;
} t15UdpHdr_t;

/* IP packet template
 * mac dest = 00:e0:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.204 (Outer IP Info 1)
 * UDP (destination port = variable)
 * Permitted by inner ACL Rules
 * Designed to match outer IP configuration 1, Inner Ip Configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0Template, ".testPkts")
#endif
static uint8_t pkt0Template[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x10,
	0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x1d, 0xe7, 0x01, 0x02, 0x03, 0x04, 0xc8, 0xc9,
	0xca, 0xcb, 0x12, 0x34, 0x05, 0x55, 0x00, 0x58,
	0x9a, 0xe6, 0x2d, 0xcf, 0x46, 0x29, 0x04, 0xb4,
	0x78, 0xd8, 0x68, 0xa7, 0xff, 0x3f, 0x2b, 0xf1,
	0xfc, 0xd9, 0x7a, 0x96, 0x09, 0x2c, 0xa5, 0x57,
	0x74, 0x64, 0xc4, 0xaf, 0x15, 0x28, 0xa4, 0xe9,
	0x57, 0xdb, 0x5e, 0x20, 0xfb, 0x38, 0xa8, 0x4e,
	0xa6, 0x14, 0x93, 0x25, 0x56, 0x24, 0x44, 0xdf,
	0x59, 0x8d, 0x43, 0x7b, 0xbe, 0x90, 0x16, 0x89,
	0x9d, 0x7e, 0x77, 0xc6, 0x2f, 0x26, 0x98, 0x88,
	0xf5, 0xb4, 0x30, 0xd4, 0x34, 0x9d, 0x3a, 0x0d,
	0x0f, 0xbd, 0x2f, 0xa1, 0xf7, 0x0f, 0xd9, 0x68,
	0xf4, 0xd9 };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0TemplateInfo, ".testPkts")
#endif
static pasahoLongInfo_t pkt0TemplateInfo = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(122,0,0,PASAHO_HDR_UDP),     /* end offset = 106, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

/* Ip Over Ip packet template
 * mac dest = 00:e0:a6:66:57:09  (MAC Info 1)
 * out ip dest = 200.201.202.204 (out IP info 1)
 * inner ip dest = 10.11.12.13 (inner IP info none)
 * UDP (destination port = variable)
 * Permitted by outer ACL NoMatch and inner ACL Rules
 * Designed to match inner IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1Template, ".testPkts")
#endif
static uint8_t pkt1Template[] = {
	0x00, 0xe0, 0xa6, 0x66, 0x57, 0x09, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x19, 0x07, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0xcc, 0x45, 0x00, 0x00, 0x6c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x93, 0x85, 0x9e, 0xda,
	0x6d, 0x0a, 0x0a, 0x0b, 0x0c, 0x0d, 0xaa, 0xbb,
	0x05, 0x55, 0x00, 0x58, 0xeb, 0xc7, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
	0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
	0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
	0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
	0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63 };


#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1TemplateInfo, ".testPkts")
#endif
static pasahoLongInfo_t pkt1TemplateInfo = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,54),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 54 (UDP Header) */
	TF_FORM_PKT_INFO_WORD1(142,0,0,PASAHO_HDR_UDP),     /* end offset = 142, errIdx, portNum = 0, nextHdr = UDP */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)

};

/* Outer ACL configuration */
#ifdef _TMS320C6X
#pragma DATA_SECTION (t15AclInfoTemplate, ".testPkts")
#endif
static t15AclPaSetup_t t15AclInfoTemplate[] =  {

	/* ------- Entry 0 ----------- */
	{
		0,			/* Sequential ID */
		0,			/* Local Handle Index */
    -1,     /* ACL LUT Instance */
		1,			/* L2/L3 Handle Index */
        -1,         /* next ACL handle index */
		pa_ACL_ACTION_PERMIT,			/* ACL actions */
		{
            pa_ACL_INFO_VALID_DST_PORT,                               /* validbitMap */
            0,                                                    /* ctrlFlag */
            0,                                                    /* ctrlFlagMask */
			pa_IPV4,	/* IP Type */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address mask */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address mask*/
			0,			/* Protocol */
			0,			/* DSCP */
            0,          /* srcPortbegin */
            0,          /* srcPort end */
            0x0555,     /* destPortbegin */
            0x0555,     /* destPort end */
		},
		pa_OK,		/* PA LLD return value expected */
		FALSE,    /* ACK */
	},
};

/* Bits 14 and 15 are used to tell where the packet is expected to wind up */
#define T15_PACKET_L3_MATCH_VALID	(0 << 14) 		/* Packet will match (handle index in bits 13:0) */
#define T15_PACKET_NFAIL			(1 << 14)     	/* Packet will arrive in the next fail queue  */
#define T15_PACKET_DISCARD			(2 << 14)		/* Packet is discarded by PA */
#define T15_PACKET_DEST_MASK		(3 << 14)
#define T15_PACKET_INDEX_MASK		0x3fff

#ifdef _TMS320C6X
#pragma DATA_SECTION (t15PktInfo, ".testPkts")
#endif
static pktTestInfo_t t15PktInfo[64];

#endif /*TEST15PKTS_H_*/
