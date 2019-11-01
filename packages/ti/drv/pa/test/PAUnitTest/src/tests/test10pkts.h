/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/
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



#ifndef TEST10PKTS_H_
#define TEST10PKTS_H_


/* Valid rx MAC addresses used during the test */
typedef struct t10EthSetup  {
	paEthInfo_t  ethInfo;		/* PA Ethernet configuration structure */
	Bool	     acked;			/* Set to TRUE when the reply to the command is received */
} t10EthSetup_t;


#ifdef _TMS320C6X
#pragma DATA_SECTION (t10EthSetup, ".testPkts")
#endif
static t10EthSetup_t t10EthSetup[] =  {

    {
      {	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },      /* Source mac is dont care */
   	 	{ 0x00, 0x01, 0x02, 0x03, 0x04, 0xaa },      /* Dest mac */
    	0,      /* Vlan      */
    	0,      /* ethertype */
    	0,		/* mpls tag  */
        0       /* input EMAC port */
      },
      FALSE  }
};

typedef struct t10IpSetup_s  {
    Bool       innerIp;     /* Inner IP link to MAC */
    Bool       nextLut1;    /* Next route is LUT1 */
	int		   lHandleIdx;  /* Linked handle (to previous L2 or L3 layer) */
	paIpInfo_t ipInfo;		/* PA IP configuration structure */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */

} t10IpSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t10IpSetup, ".testPkts")
#endif
static t10IpSetup_t  t10IpSetup[] = {
	/* IP Entry 0 */
	{
        FALSE,  /* Outer IP */
        FALSE,  /* Next Route LUT2 */
		0,	/* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 1 */
	{
        0,     /* Outer IP */
        TRUE,  /* Next Route LUT1 */
        0,	/* Linked to dest mac index 0 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	},

	/* IP Entry 2 */
	{
        TRUE,   /* Inner IP */
        FALSE,  /* Next Route LUT2 */
        1,	    /* Linked to outer IP  index 1 */
		{	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Source address */
			{ 200, 201, 202, 102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  /* IP Destination address */
			0,			/* SPI */
			0,			/* Flow */
			pa_IPV4,	/* IP Type */
			0,			/* GRE Protocol */
			0,			/* Protocol */
			0,			/* TOS */
			0,			/* TOS Care flag */
            0           /* SCTP port */
		},
		FALSE
	}
};

typedef struct t10UdpSetup_s  {
	int		   lHandleIdx;  /* Linked handle (to previous L3 layer) */
	uint16_t   port;		/* destination port number */
	Bool	   acked;		/* Set to TRUE when the reply to the command is received */
} t10UdpSetup_t;

#ifdef _TMS320C6X
#pragma DATA_SECTION(t10UdpSetup, ".testPkts")
#endif
static t10UdpSetup_t  t10UdpSetup[] = {
	/* UDP Entry 0 */
	{
		0,	    /* Linked to dest ip index 0 */
        0x8000, /* destination port number */
		FALSE
	},

	/* UDP Entry 1 */
	{
		0,	    /* Linked to dest ip index 0 */
        0x8002, /* destination port number */
		FALSE
	},

	/* UDP Entry 2 */
	{
		2,	    /* Linked to dest ip index 2 */
        0x9000, /* destination port number */
		FALSE
	},

	/* UDP Entry 3 */
	{
		2,	    /* Linked to dest ip index 2 */
        0x9002, /* destination port number */
		FALSE
	}

};

#ifndef  SIMULATOR_SUPPORT

/* packet 0
 * mac dest = 00:01:02:03:04:aa
 * PPPoE header for IPv4
 * ip src  = 158.218.109.11
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0, ".testPkts")
#endif
static uint8_t pkt0[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x88, 0x64, 0x11, 0x00,
	0x12, 0x34, 0x00, 0x00, 0x00, 0x21, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x00, 0x00, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x80, 0x00, 0x00, 0x00,
	0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt0Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,50),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(22,42,50,0),    	            /* L3 offset = 22, l4Offset = 42, l5Offset = 50, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4| PASAHO_HDR_BITMASK_PPPoE | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,50),		/* cmd len = 24, start offset = 50 (UDP Payload) */
	TF_FORM_PKT_INFO_WORD1(0,0,1,0,0,0),    /* end offset = various(NA), pmatch set */
	TF_FORM_PKT_INFO_WORD2(22,42,50,0),		/* L3 offset = 22, l4Offset = 42, l5Offset = 50, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#else


/*
 * The keystone2 simulator does not support the message length update required by PPPoE processing.
 * Remove PPPoE header until the enhanced simulator is available.
 *
 */

/* packet 0
 * mac dest = 00:01:02:03:04:aa
 * PPPoE header for IPv4
 * ip src  = 158.218.109.11
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0, ".testPkts")
#endif
static uint8_t pkt0[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x00, 0x00, 0x9e, 0xda, 0x6d, 0x0b, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x80, 0x00, 0x00, 0x00,
	0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt0Info, ".testPkts")
#endif

#ifdef NSS_GEN2
static pasahoLongInfo_t pkt0Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset, l4Offset, l5Offset, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4| PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt0Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (UDP Payload) */
	TF_FORM_PKT_INFO_WORD1(0,0,1,0,0,0),    /* end offset = various(NA), pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif
#endif

/* packet 1
 * mac dest = 00:01:02:03:04:aa
 * ip src  = 158.218.109.12
 * ip dest = 200.201.202.100
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1, ".testPkts")
#endif
static uint8_t pkt1[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x00, 0x00, 0x9e, 0xda, 0x6d, 0x0c, 0xc8, 0xc9,
	0xca, 0x64, 0xaa, 0xbb, 0x80, 0x02, 0x00, 0x00,
	0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt1Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt1Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,42),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),    	            /* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t pkt1Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,42),		/* cmd len = 24, start offset = 42 (Payload) */
	TF_FORM_PKT_INFO_WORD1(0,0,1,0,0,0),    /* end offset = various (NA), pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,34,42,0),		/* L3 offset = 14, l4Offset = 34, l5Offset = 42, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,1,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 2
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.32
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.33
 * Inner ip dest = 200.201.202.102
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2, ".testPkts")
#endif
static uint8_t pkt2[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x00, 0x00, 0x9e, 0xda, 0x6d, 0x20, 0xc8, 0xc9,
	0xca, 0x65, 0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x00, 0x00, 0x9e, 0xda,
    0x6d, 0x21, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x00, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt2Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,62),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt2Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,62),		/* cmd len = 24, start offset = 62 (Payload) */
	TF_FORM_PKT_INFO_WORD1(0,0,1,0,0,0),    /* end offset = various (NA), pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),		/* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,2,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

/* packet 3
 * mac dest = 00:01:02:03:04:aa
 * Outer ip src  = 158.218.109.48
 * Outer ip dest = 200.201.202.101
 * Inner ip src  = 158.218.109.49
 * Inner ip dest = 200.201.202.102
 * Designed to match IP configuration 0 */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3, ".testPkts")
#endif
static uint8_t pkt3[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0xaa, 0x00, 0xe0,
	0xa6, 0x66, 0x57, 0x04, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04,
	0x00, 0x00, 0x9e, 0xda, 0x6d, 0x30, 0xc8, 0xc9,
	0xca, 0x65, 0x45, 0x00, 0x00, 0x62, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x11, 0x00, 0x00, 0x9e, 0xda,
    0x6d, 0x31, 0xc8, 0xc9, 0xca, 0x66, 0xaa, 0xbb,
    0x90, 0x02, 0x00, 0x00, 0x00, 0x00
    };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt3Info, ".testPkts")
#endif
#ifdef NSS_GEN2
static pasahoLongInfo_t pkt3Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,62),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset */
	TF_FORM_PKT_INFO_WORD1(0,0,0,PASAHO_HDR_UNKNOWN),   /* end offset = various(NA), errIdx, portNum = 0, nextHdr */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),    	            /* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4 | PASAHO_HDR_BITMASK_UDP),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 2),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 2 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t pkt3Info = {

	TF_FORM_PKT_INFO_WORD0(0,24,62),		/* cmd len = 24, start offset = 62 (Payload) */
	TF_FORM_PKT_INFO_WORD1(0,0,1,0,0,0),    /* end offset = various(NA), pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,54,62,0),		/* L3 offset = 14, l4Offset = 54, l5Offset = 62, ahEspOffset = 0 */

	/* bitmap, next header = Unkown, vlan count = 0, ip count = 1, gre count = 0, frag = 0, ip route options = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP | PASAHO_HDR_BITMASK_UDP),PASAHO_HDR_UNKNOWN,0,2,0,0,0,0),
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#define T10_SWINFO0_PKT_ID                 0xAAAA0000
#define T10_PKTBUF_SIZE                    3500

#ifdef _TMS320C6X
#pragma DATA_SECTION (t10PktInfo, ".testPkts")
#endif
static pktTestInfo_t t10PktInfo[] =  {

	/* Packet 0 */
	{
		(uint8_t *)pkt0,
		(pasahoLongInfo_t *)&pkt0Info,
		sizeof(pkt0),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T10_SWINFO0_PKT_ID | 0		/* Packet will be matched by packet index 0 */
	},

	/* Packet 1 */
	{
		(uint8_t *)pkt1,
		(pasahoLongInfo_t *)&pkt1Info,
		sizeof(pkt1),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP) }, 								  		/* UDP match */
		T10_SWINFO0_PKT_ID | 1		/* Packet will be matched by packet index 1 */
	},

	/* Packet 2 */
	{
		(uint8_t *)pkt2,
		(pasahoLongInfo_t *)&pkt2Info,
		sizeof(pkt2),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)},	/* IP/UDP match */															  		/* UDP match */
		T10_SWINFO0_PKT_ID | 2		/* Packet will be matched by packet index 2 */
	},

	/* Packet 3 */
	{
		(uint8_t *)pkt3,
		(pasahoLongInfo_t *)&pkt3Info,
		sizeof(pkt3),
		{ (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),  									/* MAC match */
		  (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),  	/* IP match */
		  (1 << TF_STATS_BM_C2_NUM_PACKETS) | (1 << TF_STATS_BM_C2_NUM_UDP)     | (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4)},	/* IP/UDP match */															  		/* UDP match */
		T10_SWINFO0_PKT_ID | 3		/* Packet will be matched by packet index 3 */
	}

};

#endif /*TEST10PKTS_H_*/
