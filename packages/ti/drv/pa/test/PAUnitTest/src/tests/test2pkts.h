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

/* Xerox Proprietary packet with etherType = 0x801  */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt0, ".testPkts")
#pragma DATA_ALIGN(pkt0, 8)
static const uint8_t pkt0[] = {
#else
static const uint8_t pkt0[] __attribute__ ((aligned (8))) = {
#endif
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x01, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x08, 0x01, 0x45, 0x00,
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

static const pasahoLongInfo_t pkt0Info =  {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 255),                    /* bitmap, pdspNum = 0, liIndex = 255 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};

#else

static const pasahoLongInfo_t pkt0Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106, errIdx = 0, pmatch = 0, c2c(custom) = 0  */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),  /* bitmap, nextHdr = unknown, vlan Count = 0,
															   * ipCount = 1, gre count = 0, frag = 0,
															   * ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif

/* 802.3 packet */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt1, ".testPkts")
#pragma DATA_ALIGN(pkt1, 8)
static const uint8_t pkt1[] = {
#else
static const uint8_t pkt1[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x00, 0x64, 0xaa, 0xaa,
    0x03, 0x11, 0x22, 0x33,
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
/* Note: The output MAC header is replaced by a 14-byte header */
static const pasahoLongInfo_t pkt1Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_IPv4),    /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = Ipv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_802_3),
	                        0, 255),                    /* bitmap, pdspNum = 0, liIndex = 255 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

#else
static const pasahoLongInfo_t pkt1Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (22) */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106 (114), pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),PASAHO_HDR_IPv4,0,0,0,0,PASAHO_HDR_BITMASK2_802_3,0),
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt2, ".testPkts")
#pragma DATA_ALIGN(pkt2, 8)
static const uint8_t pkt2[] = {
#else
static const uint8_t pkt2[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0xaa, 0xbb,
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

/* Note: The output MAC header is replaced by a 14-byte header */
#ifdef NSS_GEN2
static const pasahoLongInfo_t pkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_IPv4),    /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 255),                    /* bitmap, pdspNum = 0, liIndex = 255 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};

#else
static const pasahoLongInfo_t pkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (18) */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106 (110), pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14 (18)*/
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),0,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt5, ".testPkts")
#pragma DATA_ALIGN(pkt5, 8)
static const uint8_t pkt5[] = {
#else
static const uint8_t pkt5[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0xaa, 0xbb,
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

/* Note: The output MAC header is replaced by a 14-byte header */
#ifdef NSS_GEN2
static const pasahoLongInfo_t pkt5Info =  {

	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_IPv4),    /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN | PASAHO_HDR_BITMASK_MPLS),
	                        1, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};

#else
/* Note: The output MAC header is replaced by a 14-byte header */
static const pasahoLongInfo_t pkt5Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14(22) *//* MAC header is replaced by a 14-byte header */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,1,0),  /* end offset = 106 (114), errIdx = 0, pmatch = 1, c2c(custom) = 0, l1PdspId = 1, l1Idx = don't care */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14 (22) */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN | PASAHO_HDR_BITMASK_MPLS),
	                        PASAHO_HDR_IPv4,1,0,0,0,0,0),  /* bitmap, nextHdr = ipv4, vlan Count = 0,
																							  * ipCount = 1, gre count = 0, frag = 0,
																							  * ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif

/* Packet 10 is an ARP request */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt10, ".testPkts")
#pragma DATA_ALIGN(pkt10, 8)
static const uint8_t pkt10[] = {
#else
static const uint8_t pkt10[] __attribute__ ((aligned (8))) = {
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
/* Note: The output MAC header is replaced by a 14-byte header */
static const pasahoLongInfo_t pkt10Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(60,0,0,0),                   /* end offset = 60, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		            /* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t pkt10Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,14),        /* cmd len = 24, start offset = 14 */
    TF_FORM_PKT_INFO_WORD1(60,0,1,0,0,0),   /* end offset = 60, errIdx = 0, pmatch = 1, c2c = 0, l1PdspId = 0, l1idx = dont care */
    TF_FORM_PKT_INFO_WORD2(14,0,0,0),       /* L3 offset = 14 */
    TF_FORM_PKT_INFO_WORD3(PASAHO_HDR_BITMASK_MAC,0,0,0,0,0,0,0),
    TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt11, ".testPkts")
#pragma DATA_ALIGN(pkt11, 8)
static const uint8_t pkt11[] = {
#else
static const uint8_t pkt11[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x55, 0x66, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,
	0x88, 0x64, 0x11, 0x00, 0x12, 0x34, 0x00, 0x5e,
    0x00, 0x21, 0x45, 0x00, 0x00, 0x5c, 0x00, 0x00,
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

/* Note: The output MAC header is replaced by a 14-byte header */
#ifdef NSS_GEN2
static const pasahoLongInfo_t pkt11Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,0),                  /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(26,0,0,0),		            /* L3 offset = 26  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN | PASAHO_HDR_BITMASK_PPPoE),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t pkt11Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 (26) */  /* MAC header plus VLAN plus PPPoE header */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106 (118), pmatch set */
	TF_FORM_PKT_INFO_WORD2(26,0,0,0),		/* L3 offset = 26 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),0,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt12, ".testPkts")
#pragma DATA_ALIGN(pkt12, 8)
static const uint8_t pkt12[] = {
#else
static const uint8_t pkt12[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0xff, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x04, 0x81, 0x00, 0x85, 0x55,
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
static const pasahoLongInfo_t pkt12Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(110,0,0,0),                  /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		            /* L3 offset = 18  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t pkt12Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,1,0,0,0),  /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(22,0,0,0),		/* L3 offset = 22 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),0,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt14, ".testPkts")
#pragma DATA_ALIGN(pkt14, 8)
static const uint8_t pkt14[] = {
#else
static const uint8_t pkt14[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0xff, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x06, 0x81, 0x00, 0xc7, 0x77,
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
static const pasahoLongInfo_t pkt14Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(110,0,0,0),                  /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		            /* L3 offset = 18  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t pkt14Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,1,0,0,0),  /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		/* L3 offset = 22 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),0,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt16, ".testPkts")
#pragma DATA_ALIGN(pkt16, 8)
static const uint8_t pkt16[] = {
#else
static const uint8_t pkt16[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0xff, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x07, 0x81, 0x00, 0x85, 0x55,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t pkt16Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(110,0,0,0),                  /* end offset = 110, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		            /* L3 offset = 18  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t pkt16Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,1,0,0,0),  /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		/* L3 offset = 22 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),0,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_ALIGN(pkt17, 8)
static const uint8_t pkt17[] = {
#else
static const uint8_t pkt17[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0xff, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x08,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t pkt17Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,0),                  /* end offset, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		            /* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t pkt17Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),0,0,0,0,0,0,0),  /* vlan count = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt18, ".testPkts")
#pragma DATA_ALIGN(pkt18, 8)
static const uint8_t pkt18[] = {
#else
static const uint8_t pkt18[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0xff, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x09, 0x81, 0x00, 0xc8, 0x88,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t pkt18Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(110,0,0,0),                  /* end offset = 110, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		            /* L3 offset = 18  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t pkt18Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,1,0,0,0),  /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		/* L3 offset = 22 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),0,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt19, ".testPkts")
#pragma DATA_ALIGN(pkt19, 8)
static const uint8_t pkt19[] = {
#else
static const uint8_t pkt19[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x55, 0x88, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x0a,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t pkt19Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,0),                  /* end offset, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		            /* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t pkt19Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),0,0,1,0,0,0,0),  /* vlan count = 0, ip Count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


pktTestInfo_t  t2PktTestInfo[] =  {

	{ (uint8_t *)pkt1,  (pasahoLongInfo_t *)&pkt1Info, sizeof(pkt1),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		              0, 0, 1 },

	{ (uint8_t *)pkt2,  (pasahoLongInfo_t *)&pkt2Info, sizeof(pkt2),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		              0, 0, 2 },

	{ (uint8_t *)pkt5,  (pasahoLongInfo_t *)&pkt5Info, sizeof(pkt5),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_MPLS),
		              0, 0, 5 },

    { (uint8_t *)pkt10,  (pasahoLongInfo_t *)&pkt10Info, sizeof(pkt10),
                      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
                      0, 0, 10 },

	{ (uint8_t *)pkt11,  (pasahoLongInfo_t *)&pkt11Info, sizeof(pkt11),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		              0, 0, 11 },

	{ (uint8_t *)pkt0,  (pasahoLongInfo_t *)&pkt0Info, sizeof(pkt0),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
                      (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),
		              0, 0 },

};


pktTestInfo_t  t2QoSPktTestInfo[] =  {

	{ (uint8_t *)pkt12,  (pasahoLongInfo_t *)&pkt12Info, sizeof(pkt12),
		               (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		               0, 0, 12 },

	{ (uint8_t *)pkt14,  (pasahoLongInfo_t *)&pkt14Info, sizeof(pkt14),
		               (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		               0, 0, 14 },
};

pktTestInfo_t  t2IfPktTestInfo[] =  {

	{ (uint8_t *)pkt16,  (pasahoLongInfo_t *)&pkt16Info, sizeof(pkt16),
		               (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		               0, 0, 16 },

	{ (uint8_t *)pkt17,  (pasahoLongInfo_t *)&pkt17Info, sizeof(pkt17),
		               (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		               0, 0, 17 },

	{ (uint8_t *)pkt18,  (pasahoLongInfo_t *)&pkt18Info, sizeof(pkt18),
		               (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		               0, 0, 18 },

    #ifndef __LINUX_USER_SPACE
    /*
     * Forward to PDSP1 to verify nextFail route
     * For non-user mode only since Linux add an unconditional match entry at LUT1_1
     */
	{ (uint8_t *)pkt19,  (pasahoLongInfo_t *)&pkt19Info, sizeof(pkt19),
		               (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
                       (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NUM_IPV4) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),
		                0, 19 }
    #endif
};


/* 802.1ag packet draft */
#ifdef _TMS320C6X
#pragma DATA_SECTION(e802p1agPkt1, ".testPkts")
#pragma DATA_ALIGN(e802p1agPkt1, 8)
static const uint8_t e802p1agPkt1[] = {
#else
static const uint8_t e802p1agPkt1[] __attribute__ ((aligned (8))) = {
#endif
	0x01, 0x80, 0xc2, 0x00, 0x00, 0x32, 0x00, 0xbb,      /* store packet index at location 6 */
	0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x88,
	0x89, 0x02, 0x45, 0x00, 0x00, 0x5c, 0x00, 0x00,
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
static const pasahoLongInfo_t e802p1agPkt1Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,18),		        /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
	TF_FORM_PKT_INFO_WORD1(110,pa_EROUTE_802_1ag,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	            /* L3 offset = 26  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t e802p1agPkt1Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,pa_EROUTE_802_1ag,0,0,0,0),  /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 14 (18)*/
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            PASAHO_HDR_UNKNOWN,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif



/* 802.1ag packet standard  */
#ifdef _TMS320C6X
#pragma DATA_SECTION(e802p1agPkt2, ".testPkts")
#pragma DATA_ALIGN(e802p1agPkt2, 8)
static const uint8_t e802p1agPkt2[] = {
#else
static const uint8_t e802p1agPkt2[] __attribute__ ((aligned (8))) = {
#endif
	0x01, 0x80, 0xc2, 0xba, 0xbe, 0xff, 0x01, 0xbb,     /* store packet index at location 6 */
	0xcc, 0xdd, 0xee, 0xff, 0x89, 0x02, 0x45, 0x00,
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
static const pasahoLongInfo_t e802p1agPkt2Info =  {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,14),		        /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18)*/
	TF_FORM_PKT_INFO_WORD1(106,pa_EROUTE_802_1ag,0,PASAHO_HDR_UNKNOWN),  /* end offset = 106, errIdx, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),    	            /* L3 offset = 26  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)

};
#else
static const pasahoLongInfo_t e802p1agPkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,pa_EROUTE_802_1ag,0,0,0,0),  /* end offset = 106, errIdx = 0, pmatch = 0, c2c(custom) = 0  */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),  /* bitmap, nextHdr = unknown, vlan Count = 0,
															   * ipCount = 1, gre count = 0, frag = 0,
															   * ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

pktTestInfo_t t2E8021agPktTestInfo[3][2] =  {

    {
	    { (uint8_t *)e802p1agPkt1,  (pasahoLongInfo_t *)&e802p1agPkt1Info, sizeof(e802p1agPkt1),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS),
		                0, 0, T2_802_1AG_FAIL_PKT_INDEX },

	    { (uint8_t *)e802p1agPkt2,  (pasahoLongInfo_t *)&e802p1agPkt2Info, sizeof(e802p1agPkt2),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS),
		                0, 0, T2_802_1AG_FAIL_PKT_INDEX }
    },

    {
	    { (uint8_t *)e802p1agPkt1,  (pasahoLongInfo_t *)&e802p1agPkt1Info, sizeof(e802p1agPkt1),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS),
		                0, 0, T2_802_1AG_PKT_INDEX },

	    { (uint8_t *)e802p1agPkt2,  (pasahoLongInfo_t *)&e802p1agPkt2Info, sizeof(e802p1agPkt2),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS),
		                0, 0, T2_802_1AG_FAIL_PKT_INDEX }
    },

    {
	    { (uint8_t *)e802p1agPkt1,  (pasahoLongInfo_t *)&e802p1agPkt1Info, sizeof(e802p1agPkt1),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS),
		                0, 0, T2_802_1AG_PKT_INDEX },

	    { (uint8_t *)e802p1agPkt2,  (pasahoLongInfo_t *)&e802p1agPkt2Info, sizeof(e802p1agPkt2),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS),
		                0, 0, T2_802_1AG_PKT_INDEX }
    }

};

/* Xerox Proprietary packet with etherType = 0x801  */
#ifdef _TMS320C6X
#pragma DATA_SECTION(ifPkt0, ".testPkts")
#pragma DATA_ALIGN(ifPkt0, 8)
/* multicast packet */
static const uint8_t ifPkt0[] = {
#else
static const uint8_t ifPkt0[] __attribute__ ((aligned (8))) = {
#endif
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x01, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x08, 0x01, 0x45, 0x00,
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
	0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                              /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2

static const pasahoLongInfo_t ifPkt0Info =  {

	TF_FORM_PKT_INFO_WORD0(0,24,0,0,14),		        /* cmd len = 24, pmatch = 0, frag = 0, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = unknown */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		            /* L3 offset = 0  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};

#else

static const pasahoLongInfo_t ifPkt0Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,0,0,0),  /* end offset = 106, errIdx = 0, pmatch = 0, c2c(custom) = 0  */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 0  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),  /* bitmap, nextHdr = unknown, vlan Count = 0,
															   * ipCount = 1, gre count = 0, frag = 0,
															   * ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};

#endif


/* If Broadcast */
#ifdef _TMS320C6X
#pragma DATA_SECTION(ifPkt1, ".testPkts")
#pragma DATA_ALIGN(ifPkt1, 8)
static const uint8_t ifPkt1[] = {
#else
static const uint8_t ifPkt1[] __attribute__ ((aligned (8))) = {
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
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xba, 0xbe, 0xfa, 0xce                /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t ifPkt1Info = {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,14),		        /* cmd len = 24, pmatch = 0, frag = 0, start offset = 14 (22)*/
	TF_FORM_PKT_INFO_WORD1(60,0,0,0),                   /* end offset = 60, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		            /* L3 offset  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t ifPkt1Info = {
    TF_FORM_PKT_INFO_WORD0(0,24,14),        /* cmd len = 24, start offset = 14 */
    TF_FORM_PKT_INFO_WORD1(60,0,0,0,0,0),   /* end offset = 60, errIdx = 0, pmatch = 1, c2c = 0, l1PdspId = 0, l1idx = dont care */
    TF_FORM_PKT_INFO_WORD2(0,0,0,0),        /* L3 offset */
    TF_FORM_PKT_INFO_WORD3(PASAHO_HDR_BITMASK_MAC,0,0,0,0,0,0,0),
    TF_FORM_PKT_INFO_WORD4(0)
};
#endif


#ifdef _TMS320C6X
#pragma DATA_SECTION(ifPkt2, ".testPkts")
#pragma DATA_ALIGN(ifPkt2, 8)
/* Unmatched Unicast packet in default route tests */
static const uint8_t ifPkt2[] = {
#else
static const uint8_t ifPkt2[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x55, 0x88, 0x0e, 0xb0,
	0x01, 0x02, 0x03, 0x80,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t ifPkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,14),		        /* cmd len = 24, pmatch = 0, frag = 0, start offset*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,0),                  /* end offset, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		            /* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t ifPkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,0,0,0),  /* end offset = 106, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 14 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),0,0,0,0,0,0,0),  /* vlan count = 0, ip Count = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

ifPktTestInfo_t  t2DRpktTestInfo[] =  {

    {
	    { (uint8_t *)ifPkt0,  (pasahoLongInfo_t *)&ifPkt0Info, sizeof(ifPkt0),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),
		                0, 0, T2_DR_MC_PKT_INDEX},
        pa_EMAC_PORT_1,
        0, 0, 0
    },

    {
	    { (uint8_t *)ifPkt1,  (pasahoLongInfo_t *)&ifPkt1Info, sizeof(ifPkt1),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),
		                0, 0, T2_DR_BC_PKT_INDEX},
        pa_EMAC_PORT_1,
        0, 0, 0
    },

    {
	    { (uint8_t *)ifPkt2,  (pasahoLongInfo_t *)&ifPkt2Info, sizeof(ifPkt2),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),
		                0, 0, T2_DR_UC_PKT_INDEX},
        pa_EMAC_PORT_1,
        0, 0, 0
    },

    {
	    { (uint8_t *)ifPkt0,  (pasahoLongInfo_t *)&ifPkt0Info, sizeof(ifPkt0),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS),
		                0, 0, T2_DR_PC_MC_PKT_INDEX},
        pa_EMAC_PORT_3,
        0, 0, 0
    },

    {
	    { (uint8_t *)ifPkt1,  (pasahoLongInfo_t *)&ifPkt1Info, sizeof(ifPkt1),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS),
		                0, 0, T2_DR_PC_BC_PKT_INDEX},
        pa_EMAC_PORT_3,
        0, 0, 0
    },

};

#ifdef _TMS320C6X
#pragma DATA_SECTION(eQoSpkt0, ".testPkts")
#pragma DATA_ALIGN(eQoSpkt0, 8)
/*
 * DP-Bit mode with VLAN PBit = 4 ==> offset = 1
 * byte[5]: packet info offset
 */
static const uint8_t eQoSpkt0[] = {
#else
static const uint8_t eQoSpkt0[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0x00, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x10, 0x81, 0x00, 0x85, 0x55,
	0x08, 0x00, 0x45, 0x04, 0x00, 0x5c, 0x00, 0x00,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t eQoSpkt0Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 18)*/
	TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_IPv4),    /* end offset = 110, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0), 		            /* L3 offset = 0  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t eQoSpkt0Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,1,0,0,0),  /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            PASAHO_HDR_IPv4,1,0,0,0,0,0), /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(eQoSpkt1, ".testPkts")
#pragma DATA_ALIGN(eQoSpkt1, 8)
/*
 * DP-Bit mode with IP dscp = 2 ==> offset = 8
 * byte[5]: packet info offset
 */
static const uint8_t eQoSpkt1[] = {
#else
static const uint8_t eQoSpkt1[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0x01, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x10,
	0x08, 0x00, 0x45, 0x08, 0x00, 0x5c, 0x00, 0x00,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t eQoSpkt1Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_IPv4),    /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0), 		            /* L3 offset = 0  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t eQoSpkt1Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
                            PASAHO_HDR_IPv4,0,0,0,0,0,0), /* vlan count = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(eQoSpkt2, ".testPkts")
#pragma DATA_ALIGN(eQoSpkt2, 8)
/*
 * DP-Bit mode: untagged, non-IP packet from EMAC PORT 0:  defPri = 0 ==> offset = 0
 * byte[5]: packet info offset
 */
static const uint8_t eQoSpkt2[] = {
#else
static const uint8_t eQoSpkt2[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0x02, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x10,
	0x08, 0x02, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t eQoSpkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0), 		            /* L3 offset = 0  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t eQoSpkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
                            PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),  /* vlan count = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(eQoSpkt3, ".testPkts")
#pragma DATA_ALIGN(eQoSpkt3, 8)
/*
 * DP-Bit mode with override set = 2 From EMAC port 1: defPri 7 ==> offset 5
 * byte[5]: packet info offset
 */
static const uint8_t eQoSpkt3[] = {
#else
static const uint8_t eQoSpkt3[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0x00, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x11,
	0x08, 0x00, 0x45, 0x08, 0x00, 0x5c, 0x00, 0x00,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t eQoSpkt3Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_IPv4),    /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0), 		            /* L3 offset = 0  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t eQoSpkt3Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
                            PASAHO_HDR_IPv4,0,0,0,0,0,0), /* vlan count = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(eQoSpkt4, ".testPkts")
#pragma DATA_ALIGN(eQoSpkt4, 8)
/*
 * DSCP mode with LAN (ignored) IPv6 with DSCP = 0x11 ==> offset = 14
 * byte[5]: packet info offset
 */
static const uint8_t eQoSpkt4[] = {
#else
static const uint8_t eQoSpkt4[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0x00, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x12, 0x81, 0x00, 0x45, 0x55,
	0x86, 0xdd, 0x64, 0x41, 0x01, 0x01, 0x00, 0x48,
	0x11, 0xFF, 0x20, 0x02,	0x9e, 0xda, 0x6d, 0x30,
    0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x00, 0x02, 0x04, 0x08, 0x10, 0x12,
    0x14, 0x18, 0x20, 0x22, 0x24, 0x28, 0x30, 0x32,
    0x34, 0x39, 0x11, 0x11, 0x22, 0x22, 0x00, 0x48,
    0x61, 0x89, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
    0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
    0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
    0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t eQoSpkt4Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 18)*/
	TF_FORM_PKT_INFO_WORD1(130,0,0,PASAHO_HDR_IPv6),    /* end offset = 130, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0), 		            /* L3 offset = 0  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t eQoSpkt4Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(130,0,1,0,0,0),  /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
                            PASAHO_HDR_IPv6,1,0,0,0,0,0), /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(eQoSpkt5, ".testPkts")
#pragma DATA_ALIGN(eQoSpkt5, 8)
/*
 * DSCP mode: untagged, non-IP packet from EMAC PORT 2:  defPri = 2 ==> offset = 6
 * byte[5]: packet info offset
 */
static const uint8_t eQoSpkt5[] = {
#else
static const uint8_t eQoSpkt5[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x22, 0x33, 0x44, 0x88, 0x01, 0x0e, 0xa0,
	0x01, 0x02, 0x03, 0x12,
	0x08, 0x03, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
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
	0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
    0xba, 0xbe, 0xfa, 0xce                             /* simulate CRC which will be removed by PDSP0 */
};

#ifdef NSS_GEN2
static const pasahoLongInfo_t eQoSpkt5Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,14),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 14)*/
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_UNKNOWN), /* end offset = 106, errIdx = 0, portNum = 0, nextHdr = 0 (don't care) */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0), 		            /* L3 offset = 0  */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)
};
#else
static const pasahoLongInfo_t eQoSpkt5Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,0),  /* end offset = 106, pmatch set */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
                            PASAHO_HDR_UNKNOWN,0,0,0,0,0,0),  /* vlan count = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

ifPktTestInfo_t  t2eQoSPktTestInfo[] =  {

    {
	    { (uint8_t *)eQoSpkt0,  (pasahoLongInfo_t *)&eQoSpkt0Info, sizeof(eQoSpkt0),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		                0, 0, 20},
        pa_EMAC_PORT_0,
        0, 0, 0
    },

    {
	    { (uint8_t *)eQoSpkt1,  (pasahoLongInfo_t *)&eQoSpkt1Info, sizeof(eQoSpkt1),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		                0, 0, 20},
        pa_EMAC_PORT_0,
        0, 0, 0
    },

    {
	    { (uint8_t *)eQoSpkt2,  (pasahoLongInfo_t *)&eQoSpkt2Info, sizeof(eQoSpkt2),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		                0, 0, 20},
        pa_EMAC_PORT_0,
        0, 0, 0
    },

    {
	    { (uint8_t *)eQoSpkt3,  (pasahoLongInfo_t *)&eQoSpkt3Info, sizeof(eQoSpkt3),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		                0, 0, 21},
        pa_EMAC_PORT_1,
        0, 0, 0
    },

    {
	    { (uint8_t *)eQoSpkt4,  (pasahoLongInfo_t *)&eQoSpkt4Info, sizeof(eQoSpkt4),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		                0, 0, 22},
        pa_EMAC_PORT_2,
        0, 0, 0
    },

    {
	    { (uint8_t *)eQoSpkt5,  (pasahoLongInfo_t *)&eQoSpkt5Info, sizeof(eQoSpkt5),
		                (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		                0, 0, 22},
        pa_EMAC_PORT_2,
        0, 0, 0
    },

};









