/** 
 *   @file  bcp_lte_ul.c
 *
 *   @brief  
 *      Runs LTE Uplink (UL) test case using BCP driver APIs.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
#include "bcp_example.h"
#include "bcp_lte.h"

#define     TX_DATA_BUFFER_SIZE             2048 * 2
#define     RX_DATA_BUFFER_SIZE             2048
#define     TX_NUM_DESC                     2
#define     RX_NUM_DESC                     (BCP_EXAMPLE_NUM_HOST_DESC/4 - TX_NUM_DESC)
#define     RX_Q_NUM                        900 + CSL_chipReadReg (CSL_CHIP_DNUM)
#define     RX_FLOW_ID                      0 + CSL_chipReadReg (CSL_CHIP_DNUM)

extern UInt32   totalNumTestsPass, totalNumTestsFail;

#pragma DATA_SECTION (tbSize, ".testData");
static UInt32   tbSize;
#pragma DATA_SECTION (numSymbPerSumbfrm, ".testData");
static UInt8    numSymbPerSumbfrm;
#pragma DATA_SECTION (modulation, ".testData");
static UInt8    modulation;
#pragma DATA_SECTION (numSubcarrier, ".testData");
static UInt16   numSubcarrier;
#pragma DATA_SECTION (numRiSubcs, ".testData");
static UInt16   numRiSubcs;
#pragma DATA_SECTION (numAckSubcs, ".testData");
static UInt16   numAckSubcs;
#pragma DATA_SECTION (numCqiSubcs, ".testData");
static UInt16   numCqiSubcs;
#pragma DATA_SECTION (rnti, ".testData");
static UInt16   rnti;
#pragma DATA_SECTION (cellID, ".testData");
static UInt16   cellID;
#pragma DATA_SECTION (numAckBits, ".testData");
static UInt8    numAckBits;
#pragma DATA_SECTION (ackBeta, ".testData");
static UInt8    ackBeta;
#pragma DATA_SECTION (numRiBits, ".testData");
static UInt8    numRiBits;
#pragma DATA_SECTION (riBeta, ".testData");
static UInt8    riBeta;
#pragma DATA_SECTION (numCqiBits, ".testData");
static UInt8    numCqiBits;
#pragma DATA_SECTION (cqiBeta, ".testData");
static UInt8    cqiBeta;
#pragma DATA_SECTION (ns, ".testData");
static UInt8    ns;
#pragma DATA_SECTION (rms, ".testData");
static UInt16   rms;
#pragma DATA_SECTION (snrEven, ".testData");
#pragma DATA_SECTION (snrOdd, ".testData");
static float    snrEven, snrOdd;
#pragma DATA_SECTION (scale1, ".testData");
#pragma DATA_SECTION (scale2, ".testData");
static Int32    scale1, scale2;
#pragma DATA_SECTION (rvIndex, ".testData");
static UInt8    rvIndex;
#pragma DATA_SECTION (numTrans, ".testData");
static UInt8    numTrans;
#pragma DATA_SECTION (subfrmIdx, ".testData");
static UInt8    subfrmIdx;
#pragma DATA_SECTION (soundFlag, ".testData");
static UInt8    soundFlag;

#pragma DATA_SECTION (numPrbInPusch, ".testData");
static UInt8 numPrbInPusch[45]={1, 2, 3, 4, 5, 6, 8, 9, 10,12,15,16, 18,20,24,25, 27,30,32,36, 40,45,48,50, 54,60,64,72, 75,80,
81,90,96,100,108,120,128,144,150,160,162,180,192,200,216};


/* Turbo Code Internal Interleaver Parameters */
#pragma DATA_SECTION (TurboInterTable, ".testData");
static const UInt16 TurboInterTable[189][3] = {
/*K=Data Size  f1      f2 */     
    {0	    , 0	     ,     0  },
    {40	    , 3	     ,    10  },
    {48	    , 7	     ,    12  },
    {56	    , 19	 ,    42  },
    {64	    , 7	     ,    16  },
    {72	    , 7	     ,    18  },
    {80	    , 11	 ,    20  },
    {88	    , 5	     ,    22  },
    {96	    , 11	 ,    24  },
    {104	, 7	     ,    26  },
    {112	, 41	 ,    84  },
    {120	, 103	 ,    90  },
    {128	, 15	 ,    32  },
    {136	, 9	     ,    34  },
    {144	, 17	 ,    108 },
    {152	, 9	     ,    38  },
    {160	, 21	 ,    120 },
    {168	, 101	 ,    84  },
    {176	, 21	 ,    44  },
    {184	, 57	 ,    46  },
    {192	, 23	 ,    48  },
    {200	, 13	 ,    50  },
    {208	, 27	 ,    52  },
    {216	, 11	 ,    36  },
    {224	, 27	 ,    56  },
    {232	, 85	 ,    58  },
    {240	, 29	 ,    60  },
    {248	, 33	 ,    62  },
    {256	, 15	 ,    32  },
    {264	, 17	 ,    198 },
    {272	, 33	 ,    68  },
    {280	, 103	 ,    210 },
    {288	, 19	 ,    36  },
    {296	, 19	 ,    74  },
    {304	, 37	 ,    76  },
    {312	, 19	 ,    78  },
    {320	, 21	 ,    120 },
    {328	, 21	 ,    82  },
    {336	, 115	 ,    84  },
    {344	, 193	 ,    86  },
    {352	, 21	 ,    44  },
    {360	, 133	 ,    90  },
    {368	, 81	 ,    46  },
    {376	, 45	 ,    94  },
    {384	, 23	 ,    48  },
    {392	, 243	 ,    98  },
    {400	, 151	 ,    40  },
    {408	, 155	 ,    102 },
    {416	, 25	 ,    52  },
    {424	, 51	 ,    106 },
    {432	, 47	 ,    72  },
    {440	, 91	 ,    110 },
    {448	, 29	 ,    168 },
    {456	, 29	 ,    114 },
    {464	, 247	 ,    58  },
    {472	, 29	 ,    118 },
    {480	, 89	 ,    180 },
    {488	, 91	 ,    122 },
    {496	, 157	 ,    62  },
    {504	, 55	 ,    84  },
    {512	, 31	 ,    64  },
    {528	, 17	 ,    66  },
    {544	, 35	 ,    68  },
    {560	, 227	 ,    420 },
    {576	, 65	 ,    96  },
    {592	, 19	 ,    74  },
    {608	, 37	 ,    76  },
    {624	, 41	 ,    234 },
    {640	, 39	 ,    80  },
    {656	, 185	 ,    82  },
    {672	, 43	 ,    252 },
    {688	, 21	 ,    86  },
    {704	, 155	 ,    44  },
    {720	, 79	 ,    120 },
    {736	, 139	 ,    92  },
    {752	, 23	 ,    94  },
    {768	, 217	 ,    48  },
    {784	, 25	 ,    98  },
    {800	, 17	 ,    80  },
    {816	, 127	 ,    102 },
    {832	, 25	 ,    52  },
    {848	, 239	 ,    106 },
    {864	, 17	 ,    48  },
    {880	, 137	 ,    110 },
    {896	, 215	 ,    112 },
    {912	, 29	 ,    114 },
    {928	, 15	 ,    58  },
    {944	, 147	 ,    118 },
    {960	, 29	 ,    60  },
    {976	, 59	 ,    122 },
    {992	, 65	 ,    124 },
    {1008	, 55	 ,    84  },
    {1024	, 31	 ,    64  },
    {1056	, 17	 ,    66  },
    {1088	, 171	 ,    204 },
    {1120	, 67	 ,    140 },
    {1152	, 35	 ,    72  },
    {1184	, 19	 ,    74  },
    {1216	, 39	 ,    76  },
    {1248	, 19	 ,    78  },
    {1280	, 199	 ,    240 },
    {1312	, 21	 ,    82  },
    {1344	, 211	 ,    252 },
    {1376	, 21	 ,    86  },
    {1408	, 43	 ,    88  },
    {1440	, 149	 ,    60  },
    {1472	, 45	 ,    92  },
    {1504	, 49	 ,    846 },
    {1536	, 71	 ,    48  },
    {1568	, 13	 ,    28  },
    {1600	, 17	 ,    80  },
    {1632	, 25	 ,    102 },
    {1664	, 183	 ,    104 },
    {1696	, 55	 ,    954 },
    {1728	, 127	 ,    96  },
    {1760	, 27	 ,    110 },
    {1792	, 29	 ,    112 },
    {1824	, 29	 ,    114 },
    {1856	, 57	 ,    116 },
    {1888	, 45	 ,    354 },
    {1920	, 31	 ,    120 },
    {1952	, 59	 ,    610 },
    {1984	, 185	 ,    124 },
    {2016	, 113	 ,    420 },
    {2048	, 31	 ,    64  },
    {2112	, 17	 ,    66  },
    {2176	, 171	 ,    136 },
    {2240	, 209	 ,    420 },
    {2304	, 253	 ,    216 },
    {2368	, 367	 ,    444 },
    {2432	, 265	 ,    456 },
    {2496	, 181	 ,    468 },
    {2560	, 39	 ,    80  },
    {2624	, 27	 ,    164 },
    {2688	, 127	 ,    504 },
    {2752	, 143	 ,    172 },
    {2816	, 43	 ,    88  },
    {2880	, 29	 ,    300 },
    {2944	, 45	 ,    92  },
    {3008	, 157	 ,    188 },
    {3072	, 47	 ,    96  },
    {3136	, 13	 ,    28  },
    {3200	, 111	 ,    240 },
    {3264	, 443	 ,    204 },
    {3328	, 51	 ,    104 },
    {3392	, 51	 ,    212 },
    {3456	, 451	 ,    192 },
    {3520	, 257	 ,    220 },
    {3584	, 57	 ,    336 },
    {3648	, 313	 ,    228 },
    {3712	, 271	 ,    232 },
    {3776	, 179	 ,    236 },
    {3840	, 331	 ,    120 },
    {3904	, 363	 ,    244 },
    {3968	, 375	 ,    248 },
    {4032	, 127	 ,    168 },
    {4096	, 31	 ,    64  },
    {4160	, 33	 ,    130 },
    {4224	, 43	 ,    264 },
    {4288	, 33	 ,    134 },
    {4352	, 477	 ,    408 },
    {4416	, 35	 ,    138 },
    {4480	, 233	 ,    280 },
    {4544	, 357	 ,    142 },
    {4608	, 337	 ,    480 },
    {4672	, 37	 ,    146 },
    {4736	, 71	 ,    444 },
    {4800	, 71	 ,    120 },
    {4864	, 37	 ,    152 },
    {4928	, 39	 ,    462 },
    {4992	, 127	 ,    234 },
    {5056	, 39	 ,    158 },
    {5120	, 39	 ,    80  },
    {5184	, 31	 ,    96  },
    {5248	, 113	 ,    902 },
    {5312	, 41	 ,    166 },
    {5376	, 251	 ,    336 },
    {5440	, 43	 ,    170 },
    {5504	, 21	 ,    86  },
    {5568	, 43	 ,    174 },
    {5632	, 45	 ,    176 },
    {5696	, 45	 ,    178 },
    {5760	, 161	 ,    120 },
    {5824	, 89	 ,    182 },
    {5888	, 323	 ,    184 },
    {5952	, 47	 ,    186 },
    {6016	, 23	 ,    94  },
    {6080	, 47	 ,    190 },
    {6144	, 263	 ,    480 } 
};                           

#pragma DATA_SECTION (BETAOFFSET_ACK_INDEX, ".testData");
float BETAOFFSET_ACK_INDEX[16]={2.0f, 2.5f, 3.125f, 4.0f, 5.0f, 6.25f, 8.0f, 10.0f, 12.625f, 15.875f, 20.0f, 31.0f,  50.0f, 80.0f, 126.0f, 0.f};

#pragma DATA_SECTION (BETAOFFSET_RI_INDEX, ".testData");
float BETAOFFSET_RI_INDEX[16]={1.25f, 1.625f, 2.0f, 2.5f, 3.125f, 4.0f, 5.0f, 6.250f, 8.0f, 10.0f, 12.625f, 15.875f, 20.0f, 0.f,0.f,0.f};

#pragma DATA_SECTION (BETAOFFSET_CQI_INDEX, ".testData");
float BETAOFFSET_CQI_INDEX[16]=	{0.f, 0.f, 1.125f, 1.25f,
		1.375f, 1.625f,1.75f,2.0f, 
		2.25f, 2.5f, 2.875f, 3.125f,
		3.5f,4.0f,5.0f, 6.25f};

#pragma DATA_SECTION (harqInput, ".testData");
static UInt32 harqInput [132];

#pragma DATA_SECTION (harqOutput, ".testData");
static UInt32 harqOutput [132];

/* Reference Output Data for the test */
/* Output packet 1 payload - CQI bits */
#define LTE_UL_OUTPUT_PKT_1_WRD_SIZE  5

#pragma DATA_SECTION (lte_ul_output_packet_1, ".testData");
static UInt32 lte_ul_output_packet_1[LTE_UL_OUTPUT_PKT_1_WRD_SIZE] = {
#ifndef SIMULATOR_SUPPORT
/* The packet header (first 4 words) generated by BCP is pre-swapped 
 * inside RD and is produced as 8 bit byte data. In this application we 
 * are comparing packet header as 32 bit data entities and not 
 * 8 bit byte data. Hence, we need to maintain endian dependent 
 * version of packet header. The RD output data on other hand is 
 * produced as 32 bit data as programmed in TM flow table, and hence 
 * no endian specific conversion required here.
 */
#ifdef  xdc_target__bigEndian                   
  0x00000400, 0x00000000, 0x00000000, 0xffff0000, 0xf2fcf4fc
#else
  0x00000004, 0x00000000, 0x00000000, 0xffff0000, 0xf2fcf4fc
#endif
#else
/* Simulator bug workaround: BCP TM module endian handling not correct. */
  0x00000004, 0x00000000, 0x00000000, 0xffff0000, 0xf2fcf4fc
#endif
};

/* Output packet 2 payload */
#define LTE_UL_OUTPUT_PKT_2_WRD_SIZE  140

#pragma DATA_SECTION (lte_ul_output_packet_2, ".testData");
static UInt32 lte_ul_output_packet_2[LTE_UL_OUTPUT_PKT_2_WRD_SIZE] = {
#ifndef SIMULATOR_SUPPORT
/* The packet header (first 4 words) generated by BCP is pre-swapped 
 * inside RD and is produced as 8 bit byte data. In this application we 
 * are comparing packet header as 32 bit data entities and not 
 * 8 bit byte data. Hence, we need to maintain endian dependent 
 * version of packet header. The RD output data on other hand is 
 * produced as 32 bit data as programmed in TM flow table, and hence 
 * no endian specific conversion required here.
 */
#ifdef  xdc_target__bigEndian                   
  0x0000a800, 0x00000000, 0x00000000, 0x00000000,
#else
  0x000000a8, 0x00000000, 0x00000000, 0x00000000,
#endif
#else
/* Simulator bug workaround: BCP TM module endian handling not correct. */
  0x000000a8, 0x00000000, 0x00000000, 0x00000000,
#endif
  0x09fd09fd, 0xfcfb02fd, 0x02060001, 0x0efff002,
  0x05edfd07, 0xfcfb00f9, 0xf4fe01f1, 0xfe0403f6,
  0x04fe1507, 0xfb140c02, 0xffebfff9, 0xf2fff90a,
  0x0cfffcf7, 0x00f100f3, 0x04fe0605, 0xf603f805,
  0x0c040004, 0xff0df7f9, 0xffed00ff, 0xfcfc0306,
  0x02fbfbfa, 0x010afd0e, 0xfcf9fefa, 0x0efefdfc,
  0x010603f7, 0x0017ec05, 0x07f40604, 0xfff8f8f8,
  0xf003fc03, 0x010701eb, 0x040f04f4, 0xf203ff0f,
  0xf9010104, 0x05fef7f7, 0x00030d04, 0x02fe01fe,
  0x03000700, 0xfd0908f8, 0xf9f8fcfa, 0x05fdfcf5,
  0xf2fc0207, 0x03ecf508, 0xf8ef1202, 0x00000000,
  0xfcff04f9, 0x050609ff, 0xfef90ff4, 0x05fd08fd,
  0xfc0703fa, 0x01fa0bff, 0xf7fb0204, 0x07ed04fc,
  0x04f7fdf0, 0xfffe02f8, 0x07f40bf0, 0x04fffcfe,
  0x08040b04, 0x07fe0d02, 0xec0306ff, 0xf711fb0f,
  0xf4080b00, 0x0c04f5f2, 0xf80506ff, 0x030109fa,
  0xfe0801f7, 0xfe07f705, 0x06030efc, 0xf5fff4fd,
  0xfefff90b, 0xf5fbfd07, 0xf5f8f30c, 0xf504fcfc,
  0xfaf9fefd, 0x01090af3, 0xfff7f5fd, 0x07040106,
  0xf2f7fd13, 0x0cfc0c00, 0xff010afa, 0x0200f8f7,
  0x01fdf908, 0xfff4f6f8, 0x04f5f6fe, 0xf50a0502,
  0xfef904fd, 0xf7080604, 0xf1f9ebf5, 0x00000000,
  0x020505f4, 0xf1030202, 0xfb07f408, 0x0504ff01,
  0x0709fe0d, 0xecffffec, 0x08fa00fd, 0xf909f006,
  0xf706ff02, 0x07f9ecfe, 0xffedfafd, 0x03fa0409,
  0xf8f305ff, 0xf30d000b, 0xfffa02e8, 0xe1edff07,
  0x0908100a, 0x07fa020d, 0xf901f5fc, 0xfeff0202,
  0x01f7fafc, 0xf8fcfa05, 0xf8f803ff, 0xf704fafb,
  0x04f40bfa, 0x03f5f30c, 0xf9f6ee08, 0xff01050c,
  0xfe040104, 0x04f9f3fc, 0x01f60301, 0x08f8000d,
  0x0d0f01fc, 0xfe0702fd, 0x04f80f04, 0x03010605,
  0x04fc080a, 0xf4fc06fd, 0x050afffe, 0x18fd03fa,
  0xfbf20405, 0x02fcfefa, 0x0202ebfd, 0x00000000,
  0x727f656e, 0x564d4336, 0x7268677f, 0x6a52636d};

#pragma DATA_SECTION (lte_ul_output_mem, ".testData");
static UInt32 lte_ul_output_mem[132] = {
0x09fd09fd,
0xfcfb02fd,
0x02060001,
0x0efff002,
0x05edfd07,
0xfcfb00f9,
0xf4fe01f1,
0xfe0403f6,
0x04fe1507,
0xfb140c02,
0xffebfff9,
0xf2fff90a,
0x0cfffcf7,
0x00f100f3,
0x04fe0605,
0xf603f805,
0x0c040004,
0xff0df7f9,
0xffed00ff,
0xfcfc0306,
0x02fbfbfa,
0x010afd0e,
0xfcf9fefa,
0x0efefdfc,
0x010603f7,
0x0017ec05,
0x07f40604,
0xfff8f8f8,
0xf003fc03,
0x010701eb,
0x040f04f4,
0xf203ff0f,
0xf9010104,
0x05fef7f7,
0x00030d04,
0x02fe01fe,
0x03000700,
0xfd0908f8,
0xf9f8fcfa,
0x05fdfcf5,
0xf2fc0207,
0x03ecf508,
0xf8ef1202,
0x00000000,
0xfcff04f9,
0x050609ff,
0xfef90ff4,
0x05fd08fd,
0xfc0703fa,
0x01fa0bff,
0xf7fb0204,
0x07ed04fc,
0x04f7fdf0,
0xfffe02f8,
0x07f40bf0,
0x04fffcfe,
0x08040b04,
0x07fe0d02,
0xec0306ff,
0xf711fb0f,
0xf4080b00,
0x0c04f5f2,
0xf80506ff,
0x030109fa,
0xfe0801f7,
0xfe07f705,
0x06030efc,
0xf5fff4fd,
0xfefff90b,
0xf5fbfd07,
0xf5f8f30c,
0xf504fcfc,
0xfaf9fefd,
0x01090af3, 
0xfff7f5fd,
0x07040106,
0xf2f7fd13,
0x0cfc0c00,
0xff010afa,
0x0200f8f7,
0x01fdf908,
0xfff4f6f8,
0x04f5f6fe,
0xf50a0502,
0xfef904fd,
0xf7080604,
0xf1f9ebf5,
0x00000000,
0x020505f4,
0xf1030202,
0xfb07f408,
0x0504ff01,
0x0709fe0d,
0xecffffec,
0x08fa00fd,
0xf909f006,
0xf706ff02,
0x07f9ecfe,
0xffedfafd,
0x03fa0409,
0xf8f305ff,
0xf30d000b,
0xfffa02e8,
0xd9edff07,
0x0908100a,
0x07fa020d,
0xf901f5fc,
0xfeff0202,
0x01f7fafc,
0xf8fcfa05,
0xf8f803ff,
0xf704fafb,
0x04f40bfa,
0x03f5f30c,
0xf9f6ee08,
0xff01050c,
0xfe040104,
0x04f9f3fc,
0x01f60301,
0x08f8000d,
0x0d0f01fc,
0xfe0702fd,
0x04f80f04,
0x03010605,
0x04fc080a,
0xf4fc06fd,
0x050afffe,
0x18fd03fa,
0xfbf20405,
0x02fcfefa,
0x0202ebfd,
0x00000000};


/** ============================================================================
 *   @n@b read_bcp_config
 *
 *   @b Description
 *   @n Utility function that reads the input test vector params from a given
 *      file.
 *
 *   @param[in]  
 *   @n fp              Test configuration file handle.
 * 
 *   @return        
 *   @n None.
 * =============================================================================
 */
static Void read_bcp_config(FILE* fp)
{
	Char            lineBuf[300];
	Char*           ptrBuf;
	Int32           i, j;
	
    while (fgets (lineBuf, 300, fp) != NULL)
	{
        if ((ptrBuf = strstr(lineBuf, "="))!=NULL)
		{
			ptrBuf ++;
           
			if (strstr(lineBuf,"numOFDM")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				numSymbPerSumbfrm = i;
				continue;
			}

			if (strstr(lineBuf,"SoundEn")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				soundFlag = i;
				continue;
			}

			if (strstr(lineBuf,"TBS")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				tbSize = i;
				continue;
			}
		
			if (strstr(lineBuf,"RNTI")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				rnti = (UInt8) i;
				continue;

			}

			if (strstr(lineBuf,"CellId")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				cellID = (UInt8) i;
				continue;

			}

			if (strstr(lineBuf,"Alloc")!=NULL)
			{
				sscanf(ptrBuf,"%d:%d", &i, &j);
				numSubcarrier = (j - i + 1) * 12;
				continue;

			}

			if (strstr(lineBuf,"ModSch")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				modulation = i;
				continue;
			}


			if (strstr(lineBuf,"AckBits")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				numAckBits = i;
				continue;
			}

			if (strstr(lineBuf,"McsACKoff")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				ackBeta = (UInt8) i;
				continue;
			}

			if (strstr(lineBuf,"RiBits")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				numRiBits = (UInt16) i;
				continue;
			}

			if (strstr(lineBuf,"McsRIoff")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				riBeta = (UInt8) i;
				continue;
			}

			if (strstr(lineBuf,"CqiBits")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				numCqiBits = (UInt8) i;
				continue;
			}

			if (strstr(lineBuf,"McsCQIoff")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				cqiBeta = (UInt8) i;
				continue;
			}

			if (strstr(lineBuf,"numTrans")!=NULL)
			{
				sscanf(ptrBuf,"%d", &i);
				numTrans = (UInt8) i;
				continue;
			}
		}
	}

    return;
}

/** ============================================================================
 *   @n@b compute_cbparams_ref
 *
 *   @b Description
 *   @n Given a small subset of 3GPP Params, this function calculates and maps
 *      them out to actual BCP H/w parameters.
 *
 *   @param[in]  
 *   @n tbSize          Transport block size
 * 
 *   @param[out]  
 *   @n pCodeBlkParams  LTE input parameters calculated for this test.
 * 
 *   @return        
 *   @n None.
 * =============================================================================
 */
static Void compute_cbparams_ref (UInt32 tbSize, BcpTest_LteCBParams* pCodeBlkParams)
{
	UInt32          numCodeBlocks;
	UInt32          outputbitsNoFiller;
	UInt32          firstSegmtSize;
	UInt32          secondSegmtSize;
	UInt32          firstSegCount;
	UInt32          secondSegCount;
	Int16           idx;
	UInt8           crcOrder = 24;
	UInt8           delaK;
	UInt16          numFillerBits;
	UInt32          numInputBits = tbSize + 24;  // tb size + 24 bits crc
		
	/* calculate the number of code blocks to be generated from imput bit stream */
	if( numInputBits <= MAX_CODE_BLOCK_SIZE )
	{
		numCodeBlocks = 1;
		outputbitsNoFiller = numInputBits;
	}
	else
	{ 
      	numCodeBlocks = (UInt32)ceil((double)numInputBits/( MAX_CODE_BLOCK_SIZE - crcOrder));
		outputbitsNoFiller = numInputBits + ( numCodeBlocks * crcOrder) ;
	}
	
	/* calculate number of bits in each block only when numCodeBlocks!=0 */
	if( numCodeBlocks != 0)
	{

		/* K+ = is minimum K in 3GPP TS 36.212 table 5.1.3-3 C * K >= B' */
		for(idx=0; idx < MAX_BLOCK_INDEX; idx++)
		{
			firstSegmtSize = TurboInterTable[idx][0];
			
			if( (numCodeBlocks * firstSegmtSize) >= outputbitsNoFiller)
			{
				firstSegCount = 1;
				break;
			}
		}
		pCodeBlkParams->f1Kp = TurboInterTable[idx][1];
		pCodeBlkParams->f2Kp = TurboInterTable[idx][2];

		if(numCodeBlocks == 1)
		{
			secondSegmtSize	= 0;
			secondSegCount  = 0;
			/* no kminues */
			pCodeBlkParams->f1Km = 0;
			pCodeBlkParams->f2Km = 0;
		}
		else if(numCodeBlocks > 1)
		{
			for(idx = MAX_BLOCK_INDEX - 1; idx >=0; idx--)
			{
				secondSegmtSize = TurboInterTable[idx][0];
				if( secondSegmtSize < firstSegmtSize )
				{
					break;
				}
			}
			pCodeBlkParams->f1Km = TurboInterTable[idx][1];
			pCodeBlkParams->f2Km = TurboInterTable[idx][2];
	
			delaK = firstSegmtSize - secondSegmtSize;
			secondSegCount = ( numCodeBlocks * firstSegmtSize ) - outputbitsNoFiller;
			secondSegCount = (UInt32)floor( secondSegCount / delaK );
	
			firstSegCount =  numCodeBlocks - secondSegCount;

		}
		
		numFillerBits = firstSegCount * firstSegmtSize + secondSegCount * secondSegmtSize - outputbitsNoFiller;
    
		/* Update the code block parameters with values calculated */ 
		pCodeBlkParams->numCodeBks          =	numCodeBlocks;
		pCodeBlkParams->outputbitsNoFiller  =   outputbitsNoFiller;
		pCodeBlkParams->numCodeBksKp        =   firstSegCount;
		pCodeBlkParams->codeBkSizeKp        =   firstSegmtSize;
		pCodeBlkParams->numCodeBksKm        =   secondSegCount;
		pCodeBlkParams->codeBkSizeKm        =   secondSegmtSize;
		pCodeBlkParams->numFillerBits       =   numFillerBits;
	}

    return;
}

/** ============================================================================
 *   @n@b compute_uciBits
 *
 *   @b Description
 *   @n Computes UCI parameters for the test. 
 * =============================================================================
 */
Void compute_uciBits
(
    BcpTest_LteCBParams*    pCodeBlkParams,
    UInt16                  numPrb,
    UInt8                   numOFDMsym,
    UInt8                   numAckBits,
	UInt8                   ackIdx,
	UInt8                   numRiBits,
    UInt8                   riIdx,
	UInt8                   numCqiBits,
	UInt8                   cqiIdx,
	UInt16*                 pNumRiSubcs,
	UInt16*                 pNnumAckSubcs,
	UInt16*                 pNumCqiSubcs
)
{
	UInt32          sumKr, temp1;
	float           temp;
	UInt16          cqiL;
	
	sumKr = pCodeBlkParams->codeBkSizeKm * pCodeBlkParams->numCodeBksKm + pCodeBlkParams->numCodeBksKp * pCodeBlkParams->codeBkSizeKp;
					
	temp = numAckBits * numPrb * 12 * numOFDMsym * BETAOFFSET_ACK_INDEX[ackIdx];
	temp = (float) ceil(temp /(float) sumKr);
	temp1= (UInt32) temp;
							
	if (temp1 < (UInt32)(4 * numPrb * 12))
		* pNnumAckSubcs = temp1;
	else
		* pNnumAckSubcs = 4 * numPrb * 12;

	temp = numRiBits * numPrb * 12 * numOFDMsym * BETAOFFSET_RI_INDEX[riIdx];
	temp = (float) ceil(temp /(float) sumKr);
	temp1= (UInt32) temp;
	if (temp1 < (UInt32)(4 * numPrb * 12))
		*pNumRiSubcs = temp1;
	else
		*pNumRiSubcs = 4 * numPrb * 12 ;
	

//	if (numCqiBits <= 11)
		cqiL  = 0;  
//	else cqiL = 8;

	temp = (numCqiBits + cqiL) * numPrb * 12 * numOFDMsym * BETAOFFSET_CQI_INDEX[cqiIdx] ;
	temp = (float) ceil(temp /(float) sumKr);
	temp1= (UInt32) temp;

	if (temp1 < (numPrb * 12  * numOFDMsym  - (*pNumRiSubcs)) )
		* pNumCqiSubcs  = temp1;
	else
		* pNumCqiSubcs  = numPrb * 12 * numOFDMsym  - (*pNumRiSubcs);
}

/** ============================================================================
 *   @n@b compute_rmGamma_ref
 *
 *   @b Description
 *   @n Computes Rate matching parameters for the test. 
 * =============================================================================
 */
static Void compute_rmGamma_ref
(
    UInt32                      numChannelBits,
    UInt8                       mod, 
    UInt8                       numLayers,
    BcpTest_LteCBParams*        pCodeBlkParams, 
    UInt8                       rvIndex,
    float                       ratioKm, //for PDSCH only, between 1 to 3
    float                       ratioKp,
    UInt32                      Nir,  
    UInt8                       testFlag, 
    BcpTest_RateMatchParams*    pRmParams
)
{
	UInt16 Gprime;
    UInt32 Gamma, E0, E1;   
	UInt32 G; 
	UInt8 modulationType , blkMap, numCodeBks;
	UInt16 numRows;
	UInt32 vBufferSize;
	
	G = numChannelBits;
	modulationType = mod;
	blkMap = numLayers;
	numCodeBks = pCodeBlkParams->numCodeBks;

               
	Gprime = G / (blkMap * modulationType);

    Gamma = Gprime % numCodeBks ;      
   
    //E0 = blkMap * modulationType * ((UInt32)floor( Gprime / numCodeBks));   
    //E1 = blkMap * modulationType * ((UInt32)ceil( Gprime / numCodeBks ));       

	E0 = blkMap *modulationType * (Gprime /numCodeBks);
	if (Gprime % numCodeBks==0)
		E1 = 0;
	else
		E1 = blkMap *modulationType * (Gprime /numCodeBks+1);

	
       
	pRmParams->Gamma   = Gamma;
	pRmParams->E0      = E0;
	pRmParams->E1      = E1;

	if (testFlag==LTE_PDSCH) //PDSCH
	{
		if(pRmParams->E0 > 64000) //satuarate to maximum 
			pRmParams->E0 = 64000/modulationType * modulationType;
		if (pRmParams->E1 > 64000)
			pRmParams->E1 = 64000/modulationType * modulationType;;

	}

	if (pCodeBlkParams->numCodeBksKm)
	{
		numRows = ((pCodeBlkParams->codeBkSizeKm + 4) + 31)/32;
		vBufferSize = numRows * 32 * 3;

		if (testFlag==LTE_PDSCH) //PDSCH
		{
			if (ratioKm != 0)
				pRmParams->NcbKm = (UInt32) (ratioKm * numRows * 32.f);
			else //Nir is the input
			{
				if (Nir/numCodeBks < vBufferSize)
					pRmParams->NcbKm =Nir/numCodeBks;
				else
					pRmParams->NcbKm = vBufferSize;
			}
		}
		else //PUSCH_SIC
			pRmParams->NcbKm = vBufferSize;
	
		if (pRmParams->NcbKm == numRows * 32)
			pRmParams->NcbKm ++; //not in the end of systematic bits

		pRmParams->rvKmCol = (UInt32) (2 * ceil(pRmParams->NcbKm/(8.f * numRows)) * rvIndex + 2);
		pRmParams->NcbKmCol = ( pRmParams->NcbKm - numRows * 32 - 1)/( 2 * numRows);
		pRmParams->NcbKmRow = ( pRmParams->NcbKm - numRows * 32 - 1)% (2 * numRows);

		if (pRmParams->rvKmCol > 31)
		{
			pRmParams->rvKm = (pRmParams->rvKmCol - 32)/ 2 + 32;

		}
		else
			pRmParams->rvKm = pRmParams->rvKmCol;

	}
	else
	{
		pRmParams->NcbKm = 0;
		pRmParams->rvKm = 0;
		pRmParams->rvKmCol = 0;
		pRmParams->NcbKmCol = 0;
		pRmParams->NcbKmRow = 0;
	}


	if (pCodeBlkParams->numCodeBksKp)
	{
		numRows = ((pCodeBlkParams->codeBkSizeKp + 4) + 31)/32;
		vBufferSize = numRows * 32 * 3;

		if (testFlag==LTE_PDSCH) //PDSCH
		{
			if (ratioKp != 0)
				pRmParams->NcbKp = (UInt32) (ratioKp * numRows * 32.f);
			else
			{
				if (Nir/numCodeBks < vBufferSize)
					pRmParams->NcbKp =Nir/numCodeBks;
				else
					pRmParams->NcbKp = vBufferSize;
			}
		}

		else
			pRmParams->NcbKp = vBufferSize;

		if (pRmParams->NcbKp == numRows * 32)
			pRmParams->NcbKp++;
	
		pRmParams->rvKpCol = (UInt32) (2 * ceil(pRmParams->NcbKp/(8.f * numRows)) * rvIndex + 2);
		pRmParams->NcbKpCol = ( pRmParams->NcbKp - numRows * 32 - 1)/( 2 * numRows);
		pRmParams->NcbKpRow = ( pRmParams->NcbKp - numRows * 32 - 1)% (2 * numRows);

		if (pRmParams->rvKpCol > 31)
		{
			pRmParams->rvKp = (pRmParams->rvKpCol - 32)/ 2 + 32;

		}
		else
			pRmParams->rvKp = pRmParams->rvKpCol;
	}
	else
	{
		pRmParams->NcbKp = 0;
		pRmParams->rvKp = 0;
		pRmParams->rvKpCol = 0;
		pRmParams->NcbKpCol = 0;
		pRmParams->NcbKpRow = 0;

	}

    return;
}

/** ============================================================================
 *   @n@b prepare_sslhdr_cfg
 *
 *   @b Description
 *   @n Sets up the Soft Slicer (SSL) header for the test based on inputs passed.
 * =============================================================================
 */
static Void prepare_sslhdr_cfg
(
    Bcp_SslHdr_LteCfg*  pSslHdrCfg, 
    Bcp_RadioStd        radioStd,
    UInt32              Rprime, //number of subcarrier
    UInt8               mod, 
    UInt8               numOFDMSymPerSubfrm, //lte only
    UInt8               numLayers,
    UInt16              rms,
    UInt8               Qfmt,
    UInt32              cinit,   //lte scrambling only
    UInt16              numRi,
    UInt16              numAck,
    Int32               scaleEven,    //noise scale for even slot
    Int32               scaleOdd      //noise scale for odd slot
)
{
	UInt8               idx;
	UInt32              Rprime1;
	UInt8               prbList[45]={1, 2, 3, 4, 5, 6, 8, 9,10,12,
									15,16,18,20,24,25,27,30,32,36,
									40,45,48,50,54,60,64,72,75,80,
									81,90,96,100,108,120,128,144,150,160,
									162,180,192,200,216};

    /* Setup the SSL header as per params passed */
    memset (pSslHdrCfg, 0, sizeof (Bcp_SslHdr_LteCfg));        
    pSslHdrCfg->local_hdr_len = 15;

    if (radioStd == Bcp_RadioStd_LTE)
    {
        pSslHdrCfg->modeSelCfg.split_mode_en       =   numLayers-1;
        pSslHdrCfg->modeSelCfg.jack_bit            =   0;
        pSslHdrCfg->modeSelCfg.lte_descrambler_en  =   1;
        pSslHdrCfg->modeSelCfg.mod_type_sel        =   (Bcp_ModulationType) (mod/2);
        pSslHdrCfg->modeSelCfg.cmux_ln             =   (Bcp_CmuxLength) (numOFDMSymPerSubfrm - 9);
        pSslHdrCfg->modeSelCfg.q_format            =   (Bcp_QFormat) (Qfmt);
		Rprime1 = 12 * prbList[Rprime] * numLayers;
        pSslHdrCfg->modeSelCfg.b_matrix_sel        =   gind_row_index(Rprime1 * mod);
        pSslHdrCfg->modeSelCfg.rmux_ln_index       =   Rprime; 
        pSslHdrCfg->ri_ln               =   numRi;
        pSslHdrCfg->ack_ln              =   numAck;
        pSslHdrCfg->cinit_p2            =   cinit; 
    }
    else
    {
        pSslHdrCfg->modeSelCfg.split_mode_en       =   0;
        pSslHdrCfg->modeSelCfg.jack_bit            =   0;
        pSslHdrCfg->modeSelCfg.lte_descrambler_en  =   0;
        pSslHdrCfg->modeSelCfg.mod_type_sel        =   (Bcp_ModulationType) (mod/2);
        pSslHdrCfg->modeSelCfg.cmux_ln             =   (Bcp_CmuxLength) (0);
        pSslHdrCfg->modeSelCfg.q_format            =   (Bcp_QFormat) (Qfmt);
        pSslHdrCfg->modeSelCfg.b_matrix_sel        =   0;
        pSslHdrCfg->modeSelCfg.rmux_ln_index       =   0;
        pSslHdrCfg->ri_ln               =   0;
        pSslHdrCfg->ack_ln              =   0;
        pSslHdrCfg->cinit_p2            =   0;
    }

    if (mod == 1 || mod == 2)   // BPSK or QPSK
        pSslHdrCfg->uva                 =   ((rms * 23170 + 0x4000) >> 15);
    else if (mod == 4)  // 16 QAM
        pSslHdrCfg->uva                 =   ((rms * 10362 + 0x4000) >> 15);
    else if (mod == 6)  // 64 QAM
        pSslHdrCfg->uva                 =   ((rms * 5056 + 0x4000) >> 15);
    else    // 256 QAM
        pSslHdrCfg->uva                 =   ((rms * 2513 + 0x4000) >> 15);

    if (radioStd == Bcp_RadioStd_LTE)
    {
		for(idx = 0; idx < (numOFDMSymPerSubfrm + 1)/2; idx++)
		{
            pSslHdrCfg->scale_c0 [idx] = scaleEven; 
		}

		for(idx = (numOFDMSymPerSubfrm + 1)/2; idx < numOFDMSymPerSubfrm; idx++)
		{
            pSslHdrCfg->scale_c0 [idx] = scaleOdd; 
		}

		if (numLayers == 2)
		{
			for(idx = 0; idx<(numOFDMSymPerSubfrm + 1)/2; idx++)
			{
                pSslHdrCfg->scale_c1 [idx] = scaleEven; 
			}

			for(idx = (numOFDMSymPerSubfrm + 1)/2; idx<numOFDMSymPerSubfrm; idx++)
			{
                pSslHdrCfg->scale_c1 [idx] = scaleOdd; 
			}
		}
    }

    return;
}

/** ============================================================================
 *   @n@b prepare_lte_rdhdr_cfg
 *
 *   @b Description
 *   @n Sets up the LTE Rate Dematching (RD) header for the test. 
 * =============================================================================
 */
static Void prepare_lte_rdhdr_cfg 
(
    Bcp_RdHdr_LteCfg*           pRdHdrCfg,
    BcpTest_LteCBParams*        pCodeBlkParams,
    BcpTest_RateMatchParams*    pRmParams,
    UInt8                       numCqiPayload,
    UInt16                      numCqiOutBits,
    UInt8                       uciFlag,
    UInt8                       harqFlag,
    UInt8                       cqiPassThroughFlag
)
{
    /* Setup the LTE Rate dematching header as per inputs */
    memset (pRdHdrCfg, 0, sizeof (Bcp_RdHdr_LteCfg));        
    pRdHdrCfg->local_hdr_len                =   7;        

    if (!pCodeBlkParams->numCodeBksKm)
    {
        pRdHdrCfg->num_code_blocks_c1       =   pCodeBlkParams->numCodeBksKp;        
        pRdHdrCfg->block_size_k1            =   pCodeBlkParams->codeBkSizeKp;        
        pRdHdrCfg->num_code_blocks_c2       =   0;        
        pRdHdrCfg->block_size_k2            =   0;        
    }
    else
    {
        pRdHdrCfg->num_code_blocks_c1       =   pCodeBlkParams->numCodeBksKm;        
        pRdHdrCfg->block_size_k1            =   pCodeBlkParams->codeBkSizeKm;        
        pRdHdrCfg->num_code_blocks_c2       =   pCodeBlkParams->numCodeBksKp;        
        pRdHdrCfg->block_size_k2            =   pCodeBlkParams->codeBkSizeKp;        
    }
    pRdHdrCfg->num_code_blocks_ce1          =   pCodeBlkParams->numCodeBks - pRmParams->Gamma;
    pRdHdrCfg->block_size_e1                =   pRmParams->E0; 
    pRdHdrCfg->num_code_blocks_ce2          =   pRmParams->Gamma;
    pRdHdrCfg->block_size_e2                =   pRmParams->E1;

    if (uciFlag)
    {
        if (cqiPassThroughFlag)
            pRdHdrCfg->block_size_outq_cqi  =   numCqiOutBits;              
        else
            pRdHdrCfg->block_size_outq_cqi  =   numCqiPayload;              
        pRdHdrCfg->block_size_inq_cqi       =   numCqiOutBits;              
        pRdHdrCfg->cqi_pass_through         =   cqiPassThroughFlag;
    }
    else
    {
        pRdHdrCfg->block_size_outq_cqi      =   0;              
        pRdHdrCfg->block_size_inq_cqi       =   0;              
        pRdHdrCfg->cqi_pass_through         =   1;
    }
    pRdHdrCfg->rv_start_column              =   pRmParams->rvKp; 
    pRdHdrCfg->num_filler_bits_f            =   pCodeBlkParams->numFillerBits;

	if (harqFlag)
        pRdHdrCfg->enable_harq_input        =   1;
	else
        pRdHdrCfg->enable_harq_input        =   0;

    pRdHdrCfg->enable_harq_output           =   1;
    pRdHdrCfg->harq_input_address           =   convert_coreLocalToGlobalAddr ((UInt32)harqInput);
    pRdHdrCfg->harq_output_address          =   convert_coreLocalToGlobalAddr ((UInt32)harqOutput);
    pRdHdrCfg->init_cb_flowId               =   0;
    pRdHdrCfg->flowId_hi                    =   0;
    pRdHdrCfg->flowId_cqi_offset            =   0;
    pRdHdrCfg->tcp3d_scale_factor           =   16;
    pRdHdrCfg->tcp3d_dyn_range              =   0;

    return;
}	

/** ============================================================================
 *   @n@b prepare_tmhdr_cfg
 *
 *   @b Description
 *   @n Sets up the Traffic Manager header for the test. 
 *
 *   @param[out]  
 *   @n pTmHdrCfg   TM Header configuration thus populated for the test.
 * 
 *   @return        
 *   @n None.
 * =============================================================================
 */
static Void prepare_tmhdr_cfg (Bcp_TmHdrCfg* pTmHdrCfg)
{
    pTmHdrCfg->ps_data_size     =   0;
    pTmHdrCfg->info_data_size   =   0;

    return;
}

/** ============================================================================
 *   @n@b add_bcp_config_data
 *
 *   @b Description
 *   @n Given a data buffer, this API sets up the packet contents for the test.
 *      It adds all the BCP configuration params (header) and the payload to the 
 *      data buffer. On success, returns the number of bytes of configuration 
 *      and payload data added to the data buffer passed. 
 *
 *   @param[in]  
 *   @n hBcp        BCP driver handle
 *
 *   @param[in]  
 *   @n pDataBuffer Data Buffer handle to which the payload and BCP configuration
 *                  params need to be added.
 * 
 *   @return        Int32
 *   @n >0      -   Number of bytes of data (payload + header configuration) added 
 *                  to the data buffer.
 *
 *   @n -1      -   Error populating the data buffer.
 * =============================================================================
 */
static Int32 add_bcp_config_data (Bcp_DrvHandle hBcp, UInt8*  pDataBuffer)
{
	FILE*                       pTestCfgFile;
	FILE*                       pTestSnrCfgFile;
    Char                        lineBuf [256];
	Char*                       ptrBuf;
    UInt32                      dataBufferLen, tmpLen, temp, harq_address, num_words;
    BcpTest_LteCBParams*        pCodeBlkParams;
    BcpTest_RateMatchParams*    pRmParams;
    UInt32                      lteChanType, cInit;
    Bcp_RadioStd                radioStd;
    Bcp_GlobalHdrCfg            bcpGlblHdrCfg;
    Bcp_TmHdrCfg                tmHdrCfg;
    Bcp_SslHdr_LteCfg           sslHdrCfg;
    Bcp_RdHdr_LteCfg            rdHdrCfg;
	float                       noiseVar;
	UInt32                      numChannelBits, readTemp;
	UInt8                       numCqiPayload;
	UInt8                       uciFlag, harqFlag,   cqiPassThroughFlag, index;
	UInt16                      numCqiOutBits, cellIDTemp;
	UInt8                       rvOrder[4]={0,2,1,3};
    UInt8*  					pStartDataBuffer;

    /* Allocate space for test configuration */
    if ((pCodeBlkParams = Bcp_osalMalloc (sizeof (BcpTest_LteCBParams), FALSE)) == NULL)
    {
#ifdef BCP_EXAMPLE_DEBUG
        System_printf ("Failed to allocate memory for holding test configuration.\n");
#endif
        return -1;
    }
    memset (pCodeBlkParams, 0, sizeof (BcpTest_LteCBParams));

    if ((pRmParams = Bcp_osalMalloc (sizeof (BcpTest_RateMatchParams), FALSE)) == NULL)
    {
#ifdef BCP_EXAMPLE_DEBUG
        System_printf ("Failed to allocate memory for holding test configuration.\n");
#endif
        return -1;
    }
    memset (pRmParams, 0, sizeof (BcpTest_RateMatchParams));
    
    /* Get the test configuration from the file. */
#ifdef BCP_EXAMPLE_DEBUG
    System_printf ("Reading test configuration ... \n");
#endif

    if ((pTestCfgFile = fopen("..\\..\\testcases\\lte\\pusch_paramters_UV620.txt","r")) == NULL)
    {
#ifdef BCP_EXAMPLE_DEBUG
        System_printf ("Failed to open test configuration file: ..\..\testcases\lte\pusch_paramters_UV620.txt\n");
#endif
        return -1;
    }
    read_bcp_config(pTestCfgFile);
    fclose (pTestCfgFile);

    /* Get the code block params */
    soundFlag = 0;
    compute_cbparams_ref (tbSize, pCodeBlkParams);

    if (numCqiBits)
        uciFlag = 1;
    else
        uciFlag = 0;
    cellIDTemp = cellID;

    if ((pTestSnrCfgFile = fopen("..\\..\\testcases\\lte\\pusch_paramters_UV620_RxSNR.txt","r")) == NULL)
    {
#ifdef BCP_EXAMPLE_DEBUG
        System_printf ("Failed to open test configuration Rx SNR file: ..\..\testcases\lte\pusch_paramters_UV620_RxSNR.txt\n");
#endif
        return -1;
    }

    /* To begin with, lets test LTE PUSCH channel */
    radioStd    =   Bcp_RadioStd_LTE; 
    lteChanType =   LTE_PUSCH;
    
    /* Initialize our data buffer length running counter */
    dataBufferLen   =   0;

    for (index = 0; index < numTrans; index ++)
    {
        fgets(lineBuf,256,pTestSnrCfgFile);
        sscanf(lineBuf,"%f", &snrEven);
        ptrBuf = strstr(lineBuf,"frame:");
        ptrBuf += 7;
        sscanf(ptrBuf,"%d", &readTemp);
        ptrBuf = strstr(lineBuf,"subframe:");
        ptrBuf += 10;
        sscanf(ptrBuf,"%d", &readTemp);
        subfrmIdx = (UInt8)readTemp;

        fgets(lineBuf,256,pTestSnrCfgFile);
        sscanf(lineBuf,"%f", &snrOdd);

        if (index == 0)
            continue;                

        if (soundFlag)
        {
            if (index) //second one sounding is enable
                numSymbPerSumbfrm--;
        }

        if (numCqiBits < 12)
        {
            compute_uciBits(pCodeBlkParams,
                            numSubcarrier/12,
						    numSymbPerSumbfrm,
						    numAckBits,
					        ackBeta,
				            numRiBits,
						    riBeta,
							numCqiBits,
							cqiBeta,
							&numRiSubcs,
							&numAckSubcs,
							&numCqiSubcs);
            numCqiPayload = numCqiBits;
			cqiPassThroughFlag = 1;
        }
        else
        {
            compute_uciBits(pCodeBlkParams,
                            numSubcarrier/12,
							numSymbPerSumbfrm,
							numAckBits,
							ackBeta,
							numRiBits,
							riBeta,
							numCqiBits + 8,
							cqiBeta,
						    &numRiSubcs,
							&numAckSubcs,
							&numCqiSubcs);
            numCqiPayload = numCqiBits + 8;
            cqiPassThroughFlag = 0;
        }
        
        numChannelBits = numSubcarrier * numSymbPerSumbfrm * modulation;
        numChannelBits -= (numRiSubcs * modulation);
        numChannelBits -= (numCqiSubcs * modulation);
        numCqiOutBits = numCqiSubcs *modulation;
        rvIndex = rvOrder[index];

        compute_rmGamma_ref (numChannelBits,
                            modulation,
                            1, 
                            pCodeBlkParams,
                            rvIndex,
                            0, 
                            0,
                            0,
                            lteChanType,
                            pRmParams);

        if (index == 0)
            harqFlag  = 0; 
        else
            harqFlag  = 1; 

        ns = subfrmIdx * 2;
        rms = 1024; //convert the data into fix point
		
        /* snrEven is 1/noiseVar (one-dimension) */
        noiseVar = (float) snrEven/rms/rms/2.f;  
        scale1 = * ((Int32 *) &noiseVar);

        noiseVar = (float) snrOdd/rms/rms/2.f;
        scale2 = * ((Int32 *) &noiseVar);

        cInit = (rnti * (1<<14)) + ((ns / 2) * (1<<9)) + cellIDTemp;

        /* Start adding BCP Packet headers based on the test configuration we read. */
        tmpLen			=	0;

        /* Save the start pointer and set running pointer to CRC header */
        pStartDataBuffer = pDataBuffer;
        pDataBuffer 	+=	8;
        dataBufferLen	+=	8;

        /* Header 2: SSL Header */
        {
            UInt8 uu;
			UInt8 allocIdx;

			for (uu = 0; uu < 25; uu++)
			{
				if (numPrbInPusch[uu] == numSubcarrier/12)
				{
					allocIdx = uu;
					break;
				}
            }

            prepare_sslhdr_cfg (&sslHdrCfg, radioStd, allocIdx, modulation, numSymbPerSumbfrm, 1, rms, 2, 
                                cInit, numRiSubcs, numAckSubcs, scale1, scale2);
        }
        if (Bcp_addLTE_SSLHeader (&sslHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("Failed to add SSL Header to packet \n");
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;

        /* Header 3: Rate de-matching header */
        prepare_lte_rdhdr_cfg (&rdHdrCfg, pCodeBlkParams, pRmParams, numCqiPayload, numCqiOutBits, uciFlag, harqFlag, cqiPassThroughFlag);
        if (Bcp_addLTE_RDHeader (&rdHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("Failed to add lte rate modulation header to packet \n");            
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;

        /* Also, initialize the memory block from which RD will read HARQ Input Data */
        if ((pTestCfgFile = fopen("..\\..\\testcases\\lte\\pusch_rdMemIn_0004.dat","r")) == NULL)
        {
#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("Cannot open HARQ data input file: ..\..\testcases\lte\pusch_rdMemIn_0004.dat\n");
#endif
            return -1;
        }
	    fscanf(pTestCfgFile, "0x%x\n", &harq_address);
        fscanf(pTestCfgFile, "0x%x\n", &num_words);
        harq_address	= convert_coreLocalToGlobalAddr ((UInt32)harqInput);
        temp = 0;
#ifndef SIMULATOR_SUPPORT
        read_harq_data_from_file (pTestCfgFile, (UInt8*)(harq_address), &temp);
#else
        /* Simulator bug workaround: BCP TM module endian handling not correct. */
        read_data_from_file (pTestCfgFile, (UInt8*)(harq_address), &temp);
#endif
        if (temp != num_words * 4)
        {
#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("Error initializing HARQ Input \n");
#endif
            fclose (pTestCfgFile);
            return -1;
        }
        fclose (pTestCfgFile);

        /* Header 4: Traffic Manager header */
        prepare_tmhdr_cfg (&tmHdrCfg);
        if (Bcp_addTMHeader (&tmHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("Failed to add Traffic Manager header to packet \n");            
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;

        /* Header 1: Global Header */
        bcpGlblHdrCfg.pkt_type          =   Bcp_PacketType_Normal;
        bcpGlblHdrCfg.flush             =   0;
        bcpGlblHdrCfg.drop              =   0;
        bcpGlblHdrCfg.halt              =   0;
        bcpGlblHdrCfg.radio_standard    =   radioStd;
        bcpGlblHdrCfg.hdr_end_ptr       =   ((dataBufferLen + 3) >> 2); // 2 + 16 + 8 + 2
        bcpGlblHdrCfg.flow_id           =   RX_FLOW_ID;
        bcpGlblHdrCfg.destn_tag         =   0xDEAD;
        if (Bcp_addGlobalHeader (&bcpGlblHdrCfg, pStartDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("Failed to add Global Header to packet \n");
#endif
            return -1;
        }

        /* No Info words/PS to be added for now */

    	/* Check and Add padding to align data on 128 bits (16 bytes) */
        tmpLen = (dataBufferLen % 16)? (16 - (dataBufferLen % 16)): 0;

    	/* Add padding to align data on 128 bits */
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;

        /* Finally add the data to the packet in the same order as the headers */
        if ((pTestCfgFile = fopen("..\\..\\testcases\\lte\\pusch_sslIn_0004.dat","r")) == NULL)
        {
#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("Cannot open data input file: ..\..\testcases\lte\pusch_sslIn_0004.dat\n");
#endif
            return -1;
        }
        read_data_from_file (pTestCfgFile, pDataBuffer, &dataBufferLen);
        fclose (pTestCfgFile);
    }
       
    fclose (pTestSnrCfgFile);

    Bcp_osalFree (pRmParams, sizeof (BcpTest_RateMatchParams), FALSE);
    Bcp_osalFree (pCodeBlkParams, sizeof (BcpTest_LteCBParams), FALSE);
       
    /* Successfully read the test configuration */        
    return dataBufferLen;
}

/** ============================================================================
 *   @n@b test_lte_ul
 *
 *   @b Description
 *   @n Sets up the parameters required and runs an LTE Uplink (UL) test case.
 *
 *   @param[in]  
 *   @n hBcp        BCP driver handle
 *
 *   @param[in]  
 *   @n hGlblFDQ    Global free descriptor queue handle. The test will allocate all its
 *                  free descriptors from this queue for its run and release them
 *                  back once done to this queue.
 * 
 *   @return
 *   @n None
 *
 * =============================================================================
 */
Void test_lte_ul (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblFDQ)
{
    Bcp_TxCfg           txCfg;
    Bcp_TxHandle        hTx = NULL;
    Bcp_RxCfg           rxCfg;
    Bcp_RxHandle        hRx = NULL;
    Qmss_QueueHnd       hTxFDQ, hRxFDQ;
    UInt8               rxFlowId, rxSrcId;
    UInt32              i, dataBufferLen, rxDataBufferLen, rxPsInfoLen, rxDataTotalLen, numRxPackets, testFail = 0, numTestPkts;
    UInt16              rxDestnTag;
    Cppi_Desc*          pCppiDesc;
    UInt8*              pDataBuffer;
    UInt8*              pRxDataBuffer;
    UInt8*              pRxPsInfo;
    Bcp_DrvBufferHandle hRxDrvBuffer;
    Bcp_DrvBufferHandle hVoid;
    Bcp_DrvBufferHandle hTmp;
    Int32               dataBufferLenUsed;
    Qmss_Queue          tmpQ;

    /* Setup Rx side:
     *  -   Open BCP Rx queue on which BCP results are to be received
     *  -   Setup a Rx FDQ, Rx flow to receive data
     *  -   Set the BCP Tunnel Rx endpoint configuration to NULL since we
     *      are processing BCP packets locally.
     */
    if (allocate_fdq (hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, 0, &hRxFDQ, NULL) < 0)
    {
        System_printf ("Error opening Tx FDQ \n");            
        return;
    }
    else
    {
        System_printf ("Rx FDQ %d successfully setup with %d descriptors\n", hRxFDQ, RX_NUM_DESC);
    }
    tmpQ = Qmss_getQueueNumber(hRxFDQ);
    
    rxCfg.rxQNum                        =   RX_Q_NUM;       
    rxCfg.bUseInterrupts                =   0;              // Use polling 
    memset (&rxCfg.flowCfg, 0, sizeof(Cppi_RxFlowCfg));
    rxCfg.flowCfg.flowIdNum             =   RX_FLOW_ID;  
    rxCfg.flowCfg.rx_dest_qmgr          =   0;    
    rxCfg.flowCfg.rx_dest_qnum          =   RX_Q_NUM;  
    rxCfg.flowCfg.rx_desc_type          =   Cppi_DescType_HOST; 
    rxCfg.flowCfg.rx_ps_location        =   Cppi_PSLoc_PS_IN_DESC;  
    rxCfg.flowCfg.rx_psinfo_present     =   0;           
    rxCfg.flowCfg.rx_error_handling     =   0;              //  Drop the packet, do not retry on starvation
    rxCfg.flowCfg.rx_einfo_present      =   0;              //  By default no EPIB info
    
    rxCfg.flowCfg.rx_dest_tag_lo_sel    =   4;              //  Pick dest tag 7:0 bits from the PD dest_tag        
    rxCfg.flowCfg.rx_dest_tag_hi_sel    =   5;              //  Pick the dest tag 15:8 bits from the PD dest_tag
    rxCfg.flowCfg.rx_src_tag_lo_sel     =   2;              //  Pick the src tag 7:0 bits from the PD flow_id 7:0
    rxCfg.flowCfg.rx_src_tag_hi_sel     =   4;              //  Pick the src tag 15:8 bits from the PD src_tag 7:0 

    rxCfg.flowCfg.rx_size_thresh0_en    =   0;    
    rxCfg.flowCfg.rx_size_thresh1_en    =   0;    
    rxCfg.flowCfg.rx_size_thresh2_en    =   0;    

    rxCfg.flowCfg.rx_size_thresh0       =   0x0;
    rxCfg.flowCfg.rx_size_thresh1       =   0x0;
    rxCfg.flowCfg.rx_size_thresh2       =   0x0;

    rxCfg.flowCfg.rx_fdq0_sz0_qmgr      =   tmpQ.qMgr;
    rxCfg.flowCfg.rx_fdq0_sz0_qnum      =   tmpQ.qNum; 
    rxCfg.flowCfg.rx_fdq0_sz1_qnum      =   0x0; 
    rxCfg.flowCfg.rx_fdq0_sz1_qmgr      =   0x0;
    rxCfg.flowCfg.rx_fdq0_sz2_qnum      =   0x0;
    rxCfg.flowCfg.rx_fdq0_sz2_qmgr      =   0x0;
    rxCfg.flowCfg.rx_fdq0_sz3_qnum      =   0x0;
    rxCfg.flowCfg.rx_fdq0_sz3_qmgr      =   0x0;

    rxCfg.flowCfg.rx_fdq1_qnum          =   tmpQ.qNum; 
    rxCfg.flowCfg.rx_fdq1_qmgr          =   tmpQ.qMgr;
    rxCfg.flowCfg.rx_fdq2_qnum          =   tmpQ.qNum; 
    rxCfg.flowCfg.rx_fdq2_qmgr          =   tmpQ.qMgr;
    rxCfg.flowCfg.rx_fdq3_qnum          =   tmpQ.qNum; 
    rxCfg.flowCfg.rx_fdq3_qmgr          =   tmpQ.qMgr;

    /* Setup TM flow entry configuration for BCP */
#ifdef SIMULATOR_SUPPORT
    /* Simulator bug workaround: BCP TM module endian handling not correct. */
    rxCfg.tmFlowCfg.endian_in           =   Bcp_EndianFormat_128;
    rxCfg.tmFlowCfg.endian_out          =   Bcp_EndianFormat_128;
#else
    rxCfg.tmFlowCfg.endian_in           =   Bcp_EndianFormat_32;
    rxCfg.tmFlowCfg.endian_out          =   Bcp_EndianFormat_32;
#endif
    rxCfg.tmFlowCfg.format_in           =   Bcp_DataFormat_NoChange;
    rxCfg.tmFlowCfg.pkt_type            =   0;      // No special pkt assignments
    rxCfg.tmFlowCfg.dsp_int_sel         =   0;      // Interrupt core 0
    rxCfg.tmFlowCfg.format_out          =   Bcp_DataFormat_NoChange;
    rxCfg.tmFlowCfg.qfifo_out           =   RX_FLOW_ID;      // Use Rx QFIFO 2
    rxCfg.tmFlowCfg.ps_flags            =   0;      // No PS for now

    if ((hRx = Bcp_rxOpen (hBcp, &rxCfg, NULL)) == NULL)
    {
        System_printf ("BCP Rx Open failed \n");
		goto cleanup_and_return;
    }
    else
    {
        System_printf ("Flow %d opened to send data to RxQ: %d \n", RX_FLOW_ID, RX_Q_NUM);
    }

    /* Setup Tx side:
     *  -   Open BCP Tx queue using which data would be sent 
     *  -   Setup a Tx FDQ and initialize it with some Tx FDs
     *  -   Set the BCP Tunnel Tx endpoint configuration to NULL since we
     *      are processing BCP packets locally.
     */
    txCfg.txQNum    =   Bcp_QueueId_0;
    if ((hTx = Bcp_txOpen (hBcp, &txCfg, NULL)) == NULL)
    {
        System_printf ("BCP Tx Open failed \n");
        return;
    }

    if (allocate_fdq (hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, 0, &hTxFDQ, NULL) < 0)
    {
        System_printf ("Error opening Tx FDQ \n");            
        return;
    }
    else
    {
        System_printf ("Tx FDQ %d successfully setup with %d descriptors\n", hTxFDQ, TX_NUM_DESC);
    }

    /* Build and Send a packet with LTE UL parameters for BCP Processing */
    for (numTestPkts = 0; numTestPkts < BCP_TEST_NUM_PACKETS; numTestPkts ++)
    {
        if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hTxFDQ)) == NULL)
        {
#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("Out of Tx FDs! \n");
#endif
            testFail ++;
		    goto cleanup_and_return;
        }
        pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
        Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        
	    memset (pDataBuffer, 0, dataBufferLen); 

        /* Read test configuration */
        if ((dataBufferLenUsed = add_bcp_config_data (hBcp, pDataBuffer)) <= 0)
        {
            System_printf ("Error reading test vectors/populating packet \n");
            testFail ++;
            goto cleanup_and_return;
        }

        /* Setup the data length in the descriptor */
        Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLenUsed);
        Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLenUsed);

#ifdef BCP_EXAMPLE_DEBUG
   	    System_printf ("Sending a packet of len %d to BCP ...\n", dataBufferLenUsed);
#endif

        /* Since BCP is local to the device, set destination address to NULL */
        Bcp_send (hTx, (Bcp_DrvBufferHandle) pCppiDesc, BCP_EXAMPLE_SIZE_HOST_DESC, NULL);

        /* Wait on data to be received from BCP and validate it */
        for (i = 0; i < 400; i ++);
        while ((numRxPackets = Bcp_rxGetNumOutputEntries (hRx)) == 0);

#ifdef BCP_EXAMPLE_DEBUG
        System_printf ("Got %d packet(s) from BCP \n", Bcp_rxGetNumOutputEntries (hRx));
#endif
    
        for (i = 0; i < numRxPackets; i ++)
        {
            /* Data could arrive scattered across multiple linked descriptors.
             * Collect data from all linked descriptors and validate it.
             */
            rxDataTotalLen  =   0;

            Bcp_recv (hRx,
                    &hRxDrvBuffer,
                    &pRxDataBuffer,
                    &rxDataBufferLen,
                    &pRxPsInfo,
                    &rxPsInfoLen,
                    &rxFlowId,
                    &rxSrcId,
                    &rxDestnTag);            

            if (i == 0)
            {
                if (validate_rxdata ((UInt8 *)lte_ul_output_packet_1, 
                                    LTE_UL_OUTPUT_PKT_1_WRD_SIZE*4, 
                                    pRxDataBuffer, 
                                    rxDataBufferLen, 
                                    rxDataTotalLen) != 0)
                    testFail ++;                    
            }
            else
            {
                if (validate_rxdata ((UInt8 *)lte_ul_output_packet_2, 
                                    LTE_UL_OUTPUT_PKT_2_WRD_SIZE*4, 
                                    pRxDataBuffer, 
                                    rxDataBufferLen, 
                                    rxDataTotalLen) != 0)
                    testFail ++;                    
            }

            rxDataTotalLen  +=  rxDataBufferLen;

            /* Check if there are any descriptors linked to this Rx desc */
            while (hRxDrvBuffer)
            {
                /* Save original Rx desc handle. */
                hTmp = hRxDrvBuffer;

                if ((hRxDrvBuffer = Cppi_getNextBD (Cppi_getDescType (hRxDrvBuffer), hRxDrvBuffer)))
                {
                    Bcp_rxProcessDesc  (hRx,
                                        hRxDrvBuffer,
                                        &hVoid,
                                        &pRxDataBuffer,
                                        &rxDataBufferLen,
                                        &pRxPsInfo,
                                        &rxPsInfoLen,
                                        &rxFlowId,
                                        &rxSrcId,
                                        &rxDestnTag);            

                    if (i == 0)
                    {
                        if (validate_rxdata ((UInt8 *)lte_ul_output_packet_1, 
                                            LTE_UL_OUTPUT_PKT_1_WRD_SIZE*4, 
                                            pRxDataBuffer, 
                                            rxDataBufferLen, 
                                            rxDataTotalLen) != 0)
                            testFail ++;                            
                    }
                    else
                    {
                        if (validate_rxdata ((UInt8 *)lte_ul_output_packet_2, 
                                            LTE_UL_OUTPUT_PKT_2_WRD_SIZE*4, 
                                            pRxDataBuffer, 
                                            rxDataBufferLen,
                                            rxDataTotalLen) != 0)
                            testFail ++;                           
                        
                    }

                    rxDataTotalLen  +=  rxDataBufferLen;
                }
        
                Bcp_rxFreeRecvBuffer (hRx, hTmp, BCP_EXAMPLE_SIZE_HOST_DESC);
            }

#ifdef BCP_EXAMPLE_DEBUG
            System_printf ("[Pkt %d]: Total Len: %d DestnTag: 0x%x SrcId: 0x%x (testFail: %d)\n", i, rxDataTotalLen, rxDestnTag, rxSrcId, testFail);
#endif

            if ((i == 0 && rxDataTotalLen != LTE_UL_OUTPUT_PKT_1_WRD_SIZE * 4) || 
                (i == 1 && rxDataTotalLen != LTE_UL_OUTPUT_PKT_2_WRD_SIZE * 4))
                testFail ++;
        }
    
        /* Compare HARQ output */
#ifndef SIMULATOR_SUPPORT
        if (validate_harqoutput (lte_ul_output_mem, 132 * 4, harqOutput, 132 * 4, 0) != 0)
#else
        /* Simulator bug workaround: BCP TM module endian handling not correct. */
        if (validate_rxdata ((UInt8 *)lte_ul_output_mem, 132 * 4, (UInt8 *)harqOutput, 132 * 4, 0) != 0)
#endif
            testFail ++;
    
    }

cleanup_and_return:
    if (testFail > 0)
    {
        System_printf ("LTE UL Test:    FAILED\n");                
        totalNumTestsFail ++;
    }
    else
    {
        System_printf ("LTE UL Test:    PASS\n");                
        totalNumTestsPass ++;
    }    

    if (hRx)
        Bcp_rxClose (hRx);

    deallocate_fdq (hRxFDQ, hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, NULL);

    if (hTx)
        Bcp_txClose (hTx);

    deallocate_fdq (hTxFDQ, hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, NULL);

    return;
}
