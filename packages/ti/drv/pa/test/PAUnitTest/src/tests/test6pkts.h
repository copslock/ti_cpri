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



#ifndef TEST6PKTS_H_
#define TEST6PKTS_H_

#define TEST6_CMD_SIZE		64
#define TEST6_MAX_PATCH	     5

typedef struct test6Patch_s  {
	int len;
	int offset;
	Bool overwrite;
    Bool delete;
	uint8_t bytes[16];
} test6Patch_t;


typedef struct test6PktInfo_s  {
   uint8_t        *pkt;
   int           pktSize;
   uint8_t		*cmdBuf;
   test6Patch_t *patch[TEST6_MAX_PATCH];  /* NULL for entries without patch info */
   Bool          success;   			  /* True if the patch should succeed */
} test6PktInfo_t;

/* After patch all packets should be an increasing sequence of bytes */


/* Packet 0
 * Patch 4 bytes starting at byte 4
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt0, ".testPkts")
#endif
static uint8_t pkt0[] =  {
	0x00, 0x01, 0x02, 0x03,
	0x00, 0x00, 0x00, 0x00,
	0x08, 0x09, 0x0a, 0x0b
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt0Patch, ".testPkts")
#endif
static test6Patch_t  pkt0Patch = {
	4,
	4,
	TRUE,
    FALSE,
	{ 0x04, 0x05, 0x06, 0x07,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt0Cmd, ".testPkts")
#endif
static uint8_t pkt0Cmd[TEST6_CMD_SIZE];


/* Packet 1
 * patch 1 byte starting at byte 10 */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt1, ".testPkts")
#endif
static uint8_t pkt1[] = {
	0x10, 0x11, 0x12, 0x13,
	0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x00, 0x1b,
	0x1c, 0x1d
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt1Patcha, ".testPkts")
#endif
static test6Patch_t pkt1Patcha = {
	1,
	10,
	TRUE,
    FALSE,
	{ 0x1a, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00, }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt1Patchb, ".testPkts")
#endif
static test6Patch_t pkt1Patchb = {
	2,
	12,
	TRUE,
    TRUE,
	{ 0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00, }
};


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt1Cmd, ".testPkts")
#endif
static uint8_t pkt1Cmd[TEST6_CMD_SIZE];

/* Packet 2
 * patch 16 bytes starting at byte 5 */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt2, ".testPkts")
#endif
static uint8_t pkt2[] = {
	0x20, 0x21, 0x22, 0x23,
	0x24, 0x35
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt2Patch, ".testPkts")
#endif
static test6Patch_t pkt2Patch = {
	16,
	5,
	FALSE,
    FALSE,
	{  0x25, 0x26, 0x27, 0x28,
	   0x29, 0x2a, 0x2b, 0x2c,
	   0x2d, 0x2e, 0x2f, 0x30,
	   0x31, 0x32, 0x33, 0x34 }
};


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt2Cmd, ".testPkts")
#endif
static uint8_t pkt2Cmd[TEST6_CMD_SIZE];

/* Packet 3
 * patch 3 bytes starting at byte 6
 * patch 9 bytes starting at bye 15
 * delete 2 bytes starting at byte 24
 */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt3, ".testPkts")
#endif
static uint8_t pkt3[] = {
	0x30, 0x31, 0x32, 0x33,
	0x34, 0x35, 0x00, 0x00,
	0x00, 0x39, 0x3a, 0x3b,
	0x3c, 0x3d, 0x3e, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x48, 0x49
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt3Patcha, ".testPkts")
#endif
static test6Patch_t pkt3Patcha = {
	3,
	6,
	TRUE,
    FALSE,
	{  0x36, 0x37, 0x38, 0x00,
	   0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00,
	   0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt3Patchb, ".testPkts")
#endif
static test6Patch_t pkt3Patchb = {
	9,
	15,
	TRUE,
    FALSE,
	{	0x3f, 0x40, 0x41, 0x42,
		0x43, 0x44, 0x45, 0x46,
		0x47, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt3Cmd, ".testPkts")
#endif
static uint8_t pkt3Cmd[TEST6_CMD_SIZE];

/* Packet 4
 * Three patches */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt4, ".testPkts")
#endif
static uint8_t pkt4[] = {
	0x40, 0x41, 0x00, 0x43,
	0x44, 0x45, 0x00, 0x47,
	0x48, 0x49, 0x00, 0x4b
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt4Patcha, ".testPkts")
#endif
static test6Patch_t pkt4Patcha = {
	1,
	2,
	TRUE,
    FALSE,
	{ 0x42, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt4Patchb, ".testPkts")
#endif
static test6Patch_t pkt4Patchb = {
	1,
	6,
	TRUE,
    FALSE,
	{ 0x46, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt4Patchc, ".testPkts")
#endif
static test6Patch_t pkt4Patchc = {
	1,
	10,
	TRUE,
    FALSE,
	{ 0x4a, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt4Cmd, ".testPkts")
#endif
static uint8_t pkt4Cmd[TEST6_CMD_SIZE];


/* Packet 5
 * Four patches mixed insert and overrwrite.
 * The first byte is patched as an insert,
 * and a tail patch is added */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5, ".testPkts")
#endif
static uint8_t pkt5[] = {
	      0x51, 0x52, 0x53,
	0x54, 0x00, 0x56,
	0x58, 0x59, 0x5a, 0x5b,
	0x5c, 0x5d };

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Patcha, ".testPkts")
#endif
static test6Patch_t  pkt5Patcha = {
	1,
	0,
	FALSE,
    FALSE,
	{ 0x50, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Patchb, ".testPkts")
#endif
static test6Patch_t pkt5Patchb = {
	1,
	4,
	TRUE,
    FALSE,
	{ 0x55, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Patchc, ".testPkts")
#endif
static test6Patch_t pkt5Patchc = {
	1,
	6,
	FALSE,
    FALSE,
	{ 0x57, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt5Patchd, ".testPkts")
#endif
static test6Patch_t	 pkt5Patchd = {
	2,
	12,
	FALSE,
    FALSE,
	{ 0x5e, 0x5f, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};


#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt5Cmd, ".testPkts")
#endif
static uint8_t pkt5Cmd[TEST6_CMD_SIZE];

/* Packet 6 should be rejected in the PA because it has too
 * many patch requests */
#ifdef _TMS320C6X
#pragma DATA_SECTION (pkt6, ".testPkts")
#endif
static uint8_t pkt6[] = {
	0x60, 0x61, 0x62, 0x00,
	0x64, 0x65, 0x66, 0x00,
	0x68, 0x69, 0x6a, 0x00,
	0x6c, 0x6d, 0x6e, 0x00,
	0x70, 0x71, 0x72, 0x00
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt6Patcha, ".testPkts")
#endif
static test6Patch_t pkt6Patcha = {
	1,
	3,
	TRUE,
    FALSE,
	{ 0x63, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt6Patchb, ".testPkts")
#endif
static test6Patch_t pkt6Patchb = {
	1,
	7,
	TRUE,
    FALSE,
	{ 0x67, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt6Patchc, ".testPkts")
#endif
static test6Patch_t pkt6Patchc = {
	1,
	11,
	TRUE,
    FALSE,
	{ 0x6b, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt6Patchd, ".testPkts")
#endif
static test6Patch_t pkt6Patchd = {
	1,
	15,
	TRUE,
    FALSE,
	{ 0x6f, 0x00, 0x00, 0x00,
 	  0x00, 0x00, 0x00, 0x00,
 	  0x00, 0x00, 0x00, 0x00,
 	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt6Patche, ".testPkts")
#endif
static test6Patch_t pkt6Patche = {
	1,
	19,
	TRUE,
    FALSE,
	{ 0x73, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt6Cmd, ".testPkts")
#endif
static uint8_t pkt6Cmd[TEST6_CMD_SIZE];

/* Packet 7 should be rejected because the two patches
 * are out of order */
#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt7, ".testPkts")
#endif
static uint8_t pkt7[] = {
	0x70, 0x00, 0x00, 0x73,
	0x74, 0x00, 0x00, 0x77
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt7Patcha, ".testPkts")
#endif
static test6Patch_t pkt7Patcha = {
	2,
	5,
	TRUE,
    FALSE,
	{ 0x75, 0x76, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt7Patchb, ".testPkts")
#endif
static test6Patch_t pkt7Patchb = {
	2,
	2,
	TRUE,
    FALSE,
	{ 0x71, 0x72, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00 }
};

#ifdef _TMS320C6X
#pragma DATA_SECTION(pkt7Cmd, ".testPkts")
#endif
static uint8_t pkt7Cmd[TEST6_CMD_SIZE];



#ifdef _TMS320C6X
#pragma DATA_SECTION(test6PktInfo, ".testPkts")
#endif
static test6PktInfo_t test6PktInfo[] =  {

	{  pkt0,
	   sizeof(pkt0),
	   pkt0Cmd,
	   { &pkt0Patch, NULL, NULL, NULL, NULL },
	   TRUE  },

	{  pkt1,
	   sizeof(pkt1),
	   pkt1Cmd,
	   { &pkt1Patcha, &pkt1Patchb, NULL, NULL, NULL },
	   TRUE  },

	{  pkt2,
	   sizeof(pkt2),
	   pkt2Cmd,
	   { &pkt2Patch, NULL, NULL, NULL, NULL },
	   TRUE },


	{  pkt3,
	   sizeof(pkt3),
	   pkt3Cmd,
	   { &pkt3Patcha, &pkt3Patchb, NULL, NULL, NULL },
	   TRUE },

	{  pkt4,
	   sizeof(pkt4),
	   pkt4Cmd,
	   { &pkt4Patcha, &pkt4Patchb, &pkt4Patchc, NULL, NULL },
	   TRUE },

	{  pkt5,
	   sizeof(pkt5),
	   pkt5Cmd,
	   { &pkt5Patcha, &pkt5Patchb, &pkt5Patchc, &pkt5Patchd, NULL },
	   TRUE },


	{  pkt6,
	   sizeof(pkt6),
	   pkt6Cmd,
	   { &pkt6Patcha, &pkt6Patchb, &pkt6Patchc, &pkt6Patchd, &pkt6Patche },
	   FALSE },

	{  pkt7,
	   sizeof(pkt7),
	   pkt7Cmd,
	   { &pkt7Patcha, &pkt7Patchb, NULL, NULL, NULL },
	   FALSE }

};




#endif /*TEST6PKTS_H_*/
