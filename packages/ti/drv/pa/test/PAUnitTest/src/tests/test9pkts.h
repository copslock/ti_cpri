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



/* Packet and associated structures */
typedef struct pktTest9Info_s  {
	uint8_t 		     *pkt;
	pasahoLongInfo_t *info;
    uint32_t           *psInfo;
    int               pktType;
	int			      pktLen;
	paStatsBmap_t     statsMap[3];  /* Bit map of which stats to increment. Some stats must be incremented 3 times */
	int    		      idx;		    /* Used to increment the test tracking - tells the test to look for
						             * a packet with this index */
} pktTest9Info_t;


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt2, ".testPkts")
#pragma DATA_ALIGN(pkt2, 8)
static uint8_t pkt2[] = {
#else
static uint8_t pkt2[] __attribute__ ((aligned (8))) = {
#endif
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,
	0x08, 0x00, 0x45, 0x00, 0x00, 0x5c, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x11, 0x75, 0x4e, 0x0a, 0x0b,
	0x0c, 0x0d, 0x14, 0x15, 0x16, 0x17, 0x11, 0x11,
	0x22, 0x22, 0x00, 0x48, 0x61, 0x89, 0x32, 0x33,
	0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
	0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
	0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
	0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
	0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b,
	0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
	0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71
};

#ifdef NSS_GEN2
static pasahoLongInfo_t pkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,0),		  /* cmd len = 24, pmatch = 1, frag = 0, start offset = 0 */
	TF_FORM_PKT_INFO_WORD1(0,0,0,0),          /* end offset = 0, errIdx, portNum = 0, nextHdr = don't care */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	  /* L3 offset = 0, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0,
	                       0, 0),             /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),    /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)           	  /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,0),		      /* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,1,0,0,0),    /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		  /* L3 offset = 0 */
	TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),  /* vlan count = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

static uint32_t psInfo2[2] = {
    TF_FORM_SRIO_PSINFO_WORD0(0x5678, 0x1234),
    TF_FORM_SRIO_PSINFO_WORD1_TYPE11(0, 0, pa_SRIO_TRANSPORT_TYPE_1, 0, 2)
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt5, ".testPkts")
#pragma DATA_ALIGN(pkt5, 8)
static uint8_t pkt5[] = {
#else
static uint8_t pkt5[] __attribute__ ((aligned (8))) = {
#endif
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,
	0x88, 0x47, 0x12, 0x34, 0x51, 0x64, 0x45, 0x00,
	0x00, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x11,
	0x75, 0x4e, 0x0a, 0x0b, 0x0c, 0x0d, 0x14, 0x15,
	0x16, 0x17, 0x11, 0x11, 0x22, 0x22, 0x00, 0x48,
	0x61, 0x89, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
	0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
	0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
	0x70, 0x71

};

#ifdef NSS_GEN2
static pasahoLongInfo_t pkt5Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,0),	   /* cmd len = 24, pmatch = 1, frag = 0, start offset = 0 */
	TF_FORM_PKT_INFO_WORD1(114,0,0,0),     /* end offset = 114, errIdx, portNum = 0, nextHdr = don't care */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),       /* L3 offset = 0, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0,
	                       0, 0),          /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0), /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)              /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt5Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,0),		    /* cmd len = 24, start offset = 0 */
	TF_FORM_PKT_INFO_WORD1(114,0,1,0,0,0),  /* end offset = 114, errIdx = 0, pmatch = 1, c2c(custom) = 0, l1PdspId = 0, l1Idx = don't care */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 0 */
	TF_FORM_PKT_INFO_WORD3((0),
	                        0,0,0,0,0,0,0),  /* bitmap, nextHdr = ipv4, vlan Count = 0,
											  * ipCount = 0, gre count = 0, frag = 0,
											  * ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

static uint32_t psInfo5[2] = {
    TF_FORM_SRIO_PSINFO_WORD0(0x66, 0x12),
    TF_FORM_SRIO_PSINFO_WORD1_TYPE9(0, 0, pa_SRIO_TRANSPORT_TYPE_0, 0xface, 5)
};


/* Packet 10 is an ARP request over SRIO */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt10, ".testPkts")
#pragma DATA_ALIGN(pkt10, 8)
static uint8_t pkt10[] = {
#else
static uint8_t pkt10[] __attribute__ ((aligned (8))) = {
#endif
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   /* Dest MAC */
	0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x01,	  /* Src MAC */
	0x08, 0x06,							  /* Ethertype = ARP */
	0x00, 0x01,							  /* Hardware type = ethernet */
	0x08, 0x00,							  /* Protocol type = IP */
	0x06, 0x04,							  /* Hardware size = 6, protocol size = 4 */
	0x00, 0x01,							  /* Opcode = request */
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06,	  /* Sender MAC */
	0x10, 0x11, 0x12, 0x13,				  /* Sender IP */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   /* Target MAC */
	0x14, 0x15, 0x16, 0x17,				  /* Target IP */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,	  /* Trailer */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};


#ifdef NSS_GEN2
static pasahoLongInfo_t pkt10Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,0),	   /* cmd len = 24, pmatch = 1, frag = 0, start offset = 0 */
	TF_FORM_PKT_INFO_WORD1(60,0,0,0),      /* end offset = 60, errIdx, portNum = 0, nextHdr = don't care */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),       /* L3 offset = 0, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3(0,
	                       0, 0),          /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0), /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)              /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt10Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,0),        /* cmd len = 24, start offset = 0 */
    TF_FORM_PKT_INFO_WORD1(60,0,1,0,0,0),   /* end offset = 60, errIdx = 0, pmatch = 1, c2c = 0, l1PdspId = 0, l1idx = dont care */
    TF_FORM_PKT_INFO_WORD2(0,0,0,0),
    TF_FORM_PKT_INFO_WORD3(0,0,0,0,0,0,0,0),
    TF_FORM_PKT_INFO_WORD4(0)
};
#endif

static uint32_t psInfo10[2] = {
    TF_FORM_SRIO_PSINFO_WORD0(0x6666, 0x7777),
    TF_FORM_SRIO_PSINFO_WORD1_TYPE11(0, 0, pa_SRIO_TRANSPORT_TYPE_1, 3, 5)
};

pktTest9Info_t  t9PktTestInfo[] =  {


	{ (uint8_t *)pkt2,  (pasahoLongInfo_t *)&pkt2Info, psInfo2, PA_PKT_TYPE_SRIO_TYPE_11, sizeof(pkt2),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_SRIO),
		              0, 0, 2 },

	{ (uint8_t *)pkt5,  (pasahoLongInfo_t *)&pkt5Info, psInfo5, PA_PKT_TYPE_SRIO_TYPE_9, sizeof(pkt5),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_SRIO),
		              0, 0, 5 },

    { (uint8_t *)pkt10,  (pasahoLongInfo_t *)&pkt10Info, psInfo10, PA_PKT_TYPE_SRIO_TYPE_11, sizeof(pkt10),
                      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_SRIO),
                      0, 0, 10 }

};





