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


/*
 * Note: NSS_GEN2: There is no distinguish between MAC and IP broadcast (multi-cast) packet.
 *       Exception routes pa_EROUTE_MAC_BROADCAST and pa_EROUTE_MAC_MULTICAST are not supported
 */
#ifdef NSS_GEN2
#define T8_MAC_BROADCAST_PKT_INDEX      T8_IP_BROADCAST_PKT_INDEX
#define T8_MAC_MULTICAST_PKT_INDEX      T8_IP_MULTICAST_PKT_INDEX
#else
#define T8_MAC_BROADCAST_PKT_INDEX      10
#define T8_MAC_MULTICAST_PKT_INDEX      11
#endif
#define T8_IP_BROADCAST_PKT_INDEX       12
#define T8_IP_MULTICAST_PKT_INDEX       13


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt1, ".testPkts")
#pragma DATA_ALIGN(pkt1, 8)
static uint8_t pkt1[] = {
#else
static uint8_t pkt1[] __attribute__ ((aligned (8))) = {
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

#ifdef NSS_GEN2
static pasahoLongInfo_t pkt1Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,0,0),                  /* end offset = 142, errIdx, portNum = 0, nextHdr = Don't care */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),    	            /* L3 offset = 18, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt1Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 20, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,1,0,0,0),  /* end offset = 110, pmatch set */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		/* L3 offset = 18 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),0,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt2, ".testPkts")
#pragma DATA_ALIGN(pkt2, 8)
static uint8_t pkt2[] = {
#else
static uint8_t pkt2[] __attribute__ ((aligned (8))) = {
#endif
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x81, 0x00, 0x08, 0x00,
	0x45, 0x00,
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
static pasahoLongInfo_t pkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(108,0,0,PASAHO_HDR_IPv4),    /* end offset = 108, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),    	            /* L3 offset = 18, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC|PASAHO_HDR_BITMASK_VLAN),
	                        1, 0),                      /* bitmap, pdspNum = 1, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt2Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 20, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(108,0,1,0,1,0),  /* end offset = 108, errIdx = 0, pmatch = 1, c2c(custom) = 0, l1PdspId = 1, l1Idx = don't care */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		/* L3 offset = 18 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        PASAHO_HDR_IPv4,1,0,0,0,0,0),  /* bitmap, nextHdr = ipv4, vlan Count = 1,
																							  * ipCount = 0, gre count = 0, frag = 0,
																							  * ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt3, ".testPkts")
#pragma DATA_ALIGN(pkt3, 8)
static uint8_t pkt3[] = {
#else
static uint8_t pkt3[] __attribute__ ((aligned (8))) = {
#endif
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x08, 0x00, 0x45, 0x00,
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
static pasahoLongInfo_t pkt3Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,14),		        /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_IPv4),    /* end offset = 106, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 18, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t pkt3Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,0,0,0),  /* end offset = 106, errIdx = 0, pmatch = 0, c2c(custom) = 0, l1PdspId = 0, l1Idx = don't care */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        PASAHO_HDR_IPv4,0,0,0,0,0,0),  /* bitmap, nextHdr = ipv4, vlan Count = 0,
															* ipCount = 0, gre count = 0, frag = 0,
															* ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt4, ".testPkts")
#pragma DATA_ALIGN(pkt4, 8)
static uint8_t pkt4[] = {
#else
static uint8_t pkt4[] __attribute__ ((aligned (8))) = {
#endif
	0x01, 0x00, 0x1e, 0x44, 0x55, 0x66, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x08, 0x00, 0x45, 0x00,
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
static pasahoLongInfo_t pkt4Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,0,0,14),		        /* cmd len = 24, pmatch = 0, frag = 0, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_IPv4),    /* end offset = 106, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 18, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t pkt4Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,14),		/* cmd len = 24, start offset = 14 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,0,0,0),  /* end offset = 106, errIdx = 0, pmatch = 0, c2c(custom) = 0, l1PdspId = 0, l1Idx = don't care */
	TF_FORM_PKT_INFO_WORD2(0,0,0,0),		/* L3 offset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC),
	                        PASAHO_HDR_IPv4,0,0,0,0,0,0),  /* bitmap, nextHdr = ipv4, vlan Count = 0,
															* ipCount = 0, gre count = 0, frag = 0,
															* ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt5, ".testPkts")
#pragma DATA_ALIGN(pkt5, 8)
static uint8_t pkt5[] = {
#else
static uint8_t pkt5[] __attribute__ ((aligned (8))) = {
#endif
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1a, 0x1b,
	0x1c, 0x1d, 0x1e, 0x1f, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x5c, 0x00, 0x01, 0x00, 0x00, 0x05, 0x11,
	0x75, 0x4e, 0x0a, 0x0b, 0x0c, 0x0d, 0xff, 0xff,
	0xff, 0xff, 0x11, 0x11, 0x22, 0x22, 0x00, 0x48,
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
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_UDP),     /* end offset = 108, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 18, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 59),                      /* bitmap, pdspNum = 0, liIndex = 59 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt5Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,59), /* end offset = 106, errIdx = 0, pmatch = 1, c2c(custom) = 0, l1PdspId = 0, l1Idx = don't care */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),
	                        PASAHO_HDR_UDP,0,1,0,0,0,0),  /* bitmap, nextHdr = udp, vlan Count = 0,
															* ipCount = 1, gre count = 0, frag = 0,
															* ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt6, ".testPkts")
#pragma DATA_ALIGN(pkt6, 8)
static uint8_t pkt6[] = {
#else
static uint8_t pkt6[] __attribute__ ((aligned (8))) = {
#endif
	0x01, 0x00, 0x1e, 0x14, 0x15, 0x16, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0x08, 0x00, 0x45, 0x00,
	0x00, 0x5c, 0x00, 0x01, 0x00, 0x00, 0x05, 0x11,
	0x75, 0x4e, 0x0a, 0x0b, 0x0c, 0x0d, 0xe1, 0x15,
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
static pasahoLongInfo_t pkt6Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 */
	TF_FORM_PKT_INFO_WORD1(106,0,0,PASAHO_HDR_UDP),     /* end offset = 108, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 18, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 58),                      /* bitmap, pdspNum = 0, liIndex = 58 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */
};
#else
static pasahoLongInfo_t pkt6Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 */
	TF_FORM_PKT_INFO_WORD1(106,0,1,0,0,58),  /* end offset = 106, errIdx = 0, pmatch = 1, c2c(custom) = 0, l1PdspId = 0, l1Idx = don't care */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),
	                        PASAHO_HDR_UDP,0,1,0,0,0,0),  /* bitmap, nextHdr = UDP, vlan Count = 0,
															* ipCount = 1, gre count = 0, frag = 0,
															* ip route = 0, multi route = 0 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt7, ".testPkts")
#pragma DATA_ALIGN(pkt7, 8)
static uint8_t pkt7[] = {
#else
static uint8_t pkt7[] __attribute__ ((aligned (8))) = {
#endif
	0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0xaa, 0xbb,
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
static pasahoLongInfo_t pkt7Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,18),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,0,0),                  /* end offset = 108, errIdx, portNum = 0, nextHdr = Don't care */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),    	            /* L3 offset = 18, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 59 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 1, 0, 0),              /* VC, pri, vlan Count = 1, greCount = 0, ipCount = 0 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t pkt7Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,18),		/* cmd len = 24, start offset = 18 */
	TF_FORM_PKT_INFO_WORD1(110,0,0,0,0,0),  /* end offset = 110, pmatch clear */
	TF_FORM_PKT_INFO_WORD2(18,0,0,0),		/* L3 offset = 18 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_VLAN),0,1,0,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt8, ".testPkts")
#pragma DATA_ALIGN(pkt8, 8)
static uint8_t pkt8[] = {
#else
static uint8_t pkt8[] __attribute__ ((aligned (8))) = {
#endif
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff,
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
static pasahoLongInfo_t pkt8Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,1,0,34),		        /* cmd len = 24, pmatch = 1, frag = 0, start offset = 34 */
	TF_FORM_PKT_INFO_WORD1(110,0,0,PASAHO_HDR_UDP),     /* end offset = 108, errIdx, portNum = 0, nextHdr = IPv4 */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),    	            /* L3 offset = 18, l4Offset = 0, l5Offset = 0, ahEspOffset = 0 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IPv4),
	                        0, 0),                      /* bitmap, pdspNum = 0, liIndex = 0 */
    TF_FORM_PKT_INFO_WORD4(0, 0, 0, 0, 1),              /* VC, pri, vlan Count = 0, greCount = 0, ipCount = 1 */
	TF_FORM_PKT_INFO_WORD5(0)           	            /* Pseudo header checksum */

};
#else
static pasahoLongInfo_t pkt8Info =  {
	TF_FORM_PKT_INFO_WORD0(0,24,34),		/* cmd len = 24, start offset = 34 */
	TF_FORM_PKT_INFO_WORD1(110,0,1,0,0,0),  /* end offset = 106, pmatch set */
	TF_FORM_PKT_INFO_WORD2(14,0,0,0),		/* L3 offset = 14 */
	TF_FORM_PKT_INFO_WORD3((PASAHO_HDR_BITMASK_MAC | PASAHO_HDR_BITMASK_IP),PASAHO_HDR_UDP,0,1,0,0,0,0),  /* vlan count = 1 */
	TF_FORM_PKT_INFO_WORD4(0)
};
#endif


typedef struct pktTest8Info_s  {
	uint8_t 		     *pkt;
	pasahoLongInfo_t *info;
	int			      pktLen;
	paStatsBmap_t     statsMap[3];  /* Bit map of which stats to increment. Some stats must be incremented 3 times */
	int    		      idx;		    /* Used to increment the test tracking - tells the test to look for pkt Info */
    int               multiCount;   /* Number of multiple entries */
} pktTest8Info_t;


pktTest8Info_t  t8PktTestInfo[] =  {

	{ (uint8_t *)pkt1,  (pasahoLongInfo_t *)&pkt1Info, sizeof(pkt1),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		              0, 0, 1, 1 },

	{ (uint8_t *)pkt2,  (pasahoLongInfo_t *)&pkt2Info, sizeof(pkt2),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		              0, 0, 2, 4 },
	{ (uint8_t *)pkt3,  (pasahoLongInfo_t *)&pkt3Info, sizeof(pkt3),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),
		              0, 0, T8_MAC_BROADCAST_PKT_INDEX, 8 },

	{ (uint8_t *)pkt4,  (pasahoLongInfo_t *)&pkt4Info, sizeof(pkt4),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH),
		               0, 0, T8_MAC_MULTICAST_PKT_INDEX, 4 },

	{ (uint8_t *)pkt5,  (pasahoLongInfo_t *)&pkt5Info, sizeof(pkt5),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),
		               0, T8_IP_BROADCAST_PKT_INDEX, 8 },

	{ (uint8_t *)pkt6,  (pasahoLongInfo_t *)&pkt6Info, sizeof(pkt6),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),
		               0, T8_IP_MULTICAST_PKT_INDEX, 4 },

	{ (uint8_t *)pkt7,  (pasahoLongInfo_t *)&pkt7Info, sizeof(pkt7),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH) | (1 << TF_STATS_BM_C1_SILENT_DISCARD),
		              0, 0, -1, 0 },

	{ (uint8_t *)pkt8,  (pasahoLongInfo_t *)&pkt8Info, sizeof(pkt8),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_TABLE_MATCH) | (1 << TF_STATS_BM_C1_SILENT_DISCARD),
		              (1 << TF_STATS_BM_C1_NUM_PACKETS) | (1 << TF_STATS_BM_C1_NO_TABLE_MATCH) | (1 << TF_STATS_BM_C1_NUM_IPV4),
		              0, -1, 0 },

};





