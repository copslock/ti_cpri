/** 
 *   @file  test_wcdma_ul.c
 *
 *   @brief  
 *      Runs WCDMA Uplink (UL) (HSUPA FDD) test case using BCP driver APIs.
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
#include "bcp_test.h"
#include "bcp_test_wcdma.h"

/*  Enable this compilation flag to turn on BCP Tx Queue Monitoring. 
 *
 *  The application opens a "History queue" - a general prupose queue in 
 *  addition to Tx/Rx FDQs to buffer BCP requests on Tx end until they are 
 *  acknowledged by the receiver end. All BCP Tx descriptors are configured with
 *  this history queue as Tx completion queue. In case of any errors in BCP 
 *  processing, the application re-transmits BCP packets from the history queue. 
 *  Once done all packets from history queue are restored back to Tx FDQ for cleanup 
 *  on exit.
 */
#undef     MAINTAIN_DESC_HISTORY

/* If BCP Tx queue monitoring is required, we need N * 2 number of
 * free descriptors at any point of time to be able to send/receive
 * N packets.
 */
#ifdef MAINTAIN_DESC_HISTORY
#define     TX_NUM_DESC                     (BCP_TEST_NUM_PACKETS * 2)
#else
#define     TX_NUM_DESC                     (BCP_TEST_NUM_PACKETS)
#endif
#define     TX_DATA_BUFFER_SIZE             4096 

#define     RX_NUM_DESC                     (BCP_TEST_NUM_HOST_DESC/4 - TX_NUM_DESC)
#define     RX_DATA_BUFFER_SIZE             2048

#define     RX_Q_NUM                        900 + CSL_chipReadReg (CSL_CHIP_DNUM)
#define     RX_FLOW_ID                      0 + CSL_chipReadReg (CSL_CHIP_DNUM)

extern UInt32   totalNumTestsPass, totalNumTestsFail;

#pragma DATA_SECTION (testVectorHsupa, ".testData");
static const UInt32 testVectorHsupa[1][9] = 
{   /* TTI mode: 0-2ms, 1-10ms */
    /*TTItype, startTTIN, numTTItoRun, numHARQ, TBsize, maxNumElement_Set0, partialSF, qFmt, maxNumReTrans */    
{    0,         0,          1,        8,      354,          6,              2,       3,         5        }
};

#pragma DATA_SECTION (testVectorHsupaPLnonmax, ".testData");
static const float testVectorHsupaPLnonmax[1] = {0.68};

/* 3GPP 25.211 sec5.2.1.3 table 5B for TTI = 2ms */
/* all posible SF, modulation and Ndata table 3GPP 25.212 sec 4.8.4.1 */
/* index                              1   2   3   4   5   6    7    8    9    10    11*/ 	
#pragma DATA_SECTION (tab_Ndata_TTI2ms, ".testData");
static const Int32 tab_Ndata_TTI2ms[11] = { 30, 60,120,240,480,960,1920,3840,7680,11520,23040};

#pragma DATA_SECTION (tab_Ndata_TTI10ms, ".testData");
static const Int32 tab_Ndata_TTI10ms[11]= { 30*5, 60*5,120*5,240*5,480*5,960*5,1920*5,3840*5,7680*5,11520*5,23040*5};

#pragma DATA_SECTION (tab_SF, ".testData");
static const Int32 tab_SF          [11] = {256,128, 64, 32, 16,  8,   4,   4,   2,    6,    6};//6 is 4 and 2

#pragma DATA_SECTION (tab_Num_E_DPDCH, ".testData");
static const Int32 tab_Num_E_DPDCH [11] = {  1,  1,  1,  1,  1,  1,   1,   2,   2,    4,    4};

#pragma DATA_SECTION (tab_Mod_scheme, ".testData");
static const Int32 tab_Mod_scheme  [11] = {  1,  1,  1,  1,  1,  1,   1,   1,   1,    1,    2};//1:BPSK, 2:4PAM

/* RV value to s r table 3GPP 25.212 sec 4.8.4.3 table 15D */
/*           RVindex =  0  1  2  3 */
#pragma DATA_SECTION (tab_s, ".testData");
static const Int32 tab_s[4] = {1, 0, 1, 0};

#pragma DATA_SECTION (tab_r, ".testData");
static const Int32 tab_r[4] = {0, 0, 1, 1};

/* Reference output data */
#define WCDMA_UL_OUTPUT_PKT_1_WRD_SIZE  296

#pragma DATA_SECTION (wcdma_ul_output_packet_1, ".testData");
static UInt32 wcdma_ul_output_packet_1[WCDMA_UL_OUTPUT_PKT_1_WRD_SIZE] = {
#ifndef SIMULATOR_SUPPORT
/* The packet header (first 4 words) generated by BCP is pre-swapped 
 * inside RD and is produced as 8 bit byte data. In this application we 
 * are comparing packet header as 32 bit data entities and not 
 * 8 bit byte data. Hence, we need to maintain endian dependent 
 * version of packet header. The RD output data on other hand is 
 * produced as 32 bit data as programmed in TM flow table, and hence 
 * no endian specific conversion required here.
 */
#ifndef xdc_target__bigEndian
  0x0000017e, 0x00000000, 0x00000000, 0x00000000,
#else
  0x00007e01, 0x00000000, 0x00000000, 0x00000000,
#endif
#else
/* Simulator bug workaround: BCP TM module endian handling not correct. */
  0x0000017e, 0x00000000, 0x00000000, 0x00000000,
#endif
  0x0e1f15e1, 0x1fe101e5, 0x0de7111e, 0x1c0dede7,
  0xec001fe1, 0xe100131f, 0x0300fd0c, 0x1f00e1ef,
  0x0106e106, 0x1fe1e100, 0xfbe1e100, 0x10e1e100,
  0x151fe109, 0x1fe11fe3, 0xfdee0106, 0x0e03130b,
  0x1f001ff8, 0xf400e11f, 0xf500181f, 0xe1001fff,
  0xe11f1f00, 0xe1e71f00, 0x15e10c00, 0xe1e10500,
  0x02ebe11f, 0xe11f1fe1, 0x0e1f02e1, 0x02e1e11f,
  0x0600e510, 0x00e604fb, 0x00e1e11f, 0x000e1f1f,
  0xe107ed00, 0x0e18f500, 0xe1e1001f, 0xfafa00e1,
  0x1f1f1c1f, 0x1fe11d1f, 0xf91f06f4, 0x09f3e104,
  0x00e1eae1, 0x00e1e1e1, 0x001f1fe1, 0x00e1e103,
  0xe1e100eb, 0x1de1001f, 0x0d150011, 0xe1f80018,
  0xe1e1e11f, 0x08fae117, 0x1f15e1e1, 0x1f141014,
  0x00f51fe1, 0x00f21ff0, 0x001fe1eb, 0xfcf91f13,
  0x1fe800e1, 0x1f1900e1, 0x1fe10013, 0x0f00ea1f,
  0xe1f1e11f, 0x11e1ef1f, 0xede11fe1, 0xe1e11f1f,
  0xef171f00, 0xe110e100, 0x1f1ffd00, 0xe11f1c00,
  0xe100e11f, 0xf800f517, 0xe100191f, 0x1f00e918,
  0xfe1f1fe6, 0xe11f1ce1, 0xe1071f1f, 0x1f03fbf7,
  0x111f0e00, 0x121fe100, 0xf4e11f00, 0x1fe10700,
  0x1800e4e1, 0xf400f216, 0xe1001d1f, 0x1f001ffc,
  0xe8f01f1f, 0x1fe1e1ec, 0x13111fe1, 0x191ce1e1,
  0x1f0ae100, 0xe1ee00e1, 0xe1e100f1, 0xe117001f,
  0x001fe117, 0x001fe1f5, 0x00e11f1e, 0x001f1f1f,
  0xf5e11f1f, 0x1f1fe11f, 0xe1ffe11b, 0x1f1ee11f,
  0x1ff400e6, 0x04e1001f, 0x1fe100e2, 0x07e1001f,
  0x00141ce1, 0x0019e1e1, 0x001ff0fa, 0x00141fec,
  0xe11fe1e8, 0xf61fe11f, 0xe111e31f, 0x1fe1e7e1,
  0xe61f00fe, 0x1fe1001c, 0xec00e11f, 0x0800e10e,
  0x0011e1fa, 0x00e107e1, 0x1fe104ed, 0xf503f700,
  0xe1e1f8e1, 0xe11fe1ea, 0x19e1e1ff, 0xf9fde103,
  0xf400e111, 0x1f00e113, 0xff001f1f, 0xe1000bf4,
  0x1de11f00, 0xe81c0500, 0x16ff1f00, 0xe1e11f00,
  0xe1e1f2e1, 0xfbe1ff18, 0x0b1ffae1, 0xe11feae1,
  0x06001f0b, 0x1f00ec04, 0xe900e1e1, 0xe100e20b,
  0xe104e500, 0xfe1cf600, 0x1fe10100, 0x1be1e800,
  0xe11fe1e1, 0x1f11f1f4, 0xf5e11de1, 0xfcec1f15,
  0x001ff3e1, 0x001fe1f1, 0x00e1e1e1, 0x00e11fe1,
  0x19fb001f, 0xe119001f, 0x1fe100e1, 0x1f1f00f8,
  0xe1191fe3, 0xe10dfc0b, 0xe1e10bf3, 0xe1e11ff4,
  0x00e11f1f, 0x00e11f1f, 0x00e11f1f, 0x001e1f05,
  0xe11f0011, 0xfce100e1, 0xe8e1001e, 0x1ff4001f,
  0xed1fe11f, 0x1fe1e1e1, 0x03e7e11f, 0xe1e11fe6,
  0x001f031f, 0xeee1021f, 0xeaedfc00, 0x1f1fe400,
  0xe11f001f, 0x1f00f8f3, 0x0300e1e1, 0x1f00101f,
  0x011f1fe7, 0xe11f1fe1, 0x1f1f18f8, 0x1f1f1fe1,
  0xe6e1e100, 0xeee31f00, 0x0bee0500, 0xe11fe100,
  0xea00e11f, 0x1c001fe1, 0x1700191f, 0xe700f5e1,
  0x1ff11f1f, 0x18e4e3e3, 0xeb111f1f, 0x1fe11fe1,
  0xe9f71800, 0xe11c1100, 0xe1e10900, 0xff1f00e1,
  0x130003e1, 0x0e0014eb, 0x1f00e1e1, 0x00e11fe2,
  0x1f1fe7e6, 0xfb191bef, 0x0ee11ff0, 0x1f1f1ffc,
  0xfeea00fa, 0xede3000e, 0xe1f1001f, 0xe1e10008,
  0x00180712, 0x00ed1ce1, 0x00e1131a, 0x001f1fe1,
  0x1fe1f91f, 0x1f18e11c, 0xeffd1fe7, 0x1ff71f0a,
  0x041f00e9, 0xe1ed001f, 0xe1e1001f, 0xe1e100e4,
  0x0007f212, 0x001fe114, 0x0011e6e1, 0x001f01e1,
  0xec1ff914, 0x1c1feae1, 0x091f0f1a, 0x1f18e11f,
  0xe1000a1f, 0x0100e8e1, 0xe100131f, 0x1c00f51f,
  0xe3e2e113, 0xe3f11f00, 0x1feb1d00, 0xe9ed1f00,
  0xf40de11f, 0xe11b1f17, 0x1f081f1f, 0x1fe10c1f,
  0x1f00e414, 0xe1001be1, 0xe1001f1f, 0x0d001fe4,
  0x1f1f1f00, 0x1412e100, 0x1feae100, 0x1f1f0f00,
  0xfcec1f12, 0xeaf9e1e9, 0x19e11fe1, 0xe41d0b0b,
  0xf100e1e1, 0xeb00031f, 0x00ea1f15, 0x001f1ff7,
  0xe1ff1f00, 0x1f1f1000, 0xe1e1001f, 0xe11c001f,
  0x0a130804, 0xf91f0a04, 0xe11f1fe1, 0xe1e1130d,
  0x00e1121f, 0x00fd15e4, 0x00e60312, 0x001f1ff2,
  0x1fe100ef, 0xee1f001f, 0xe11f00ea, 0xfa1f000b,
  0xe1060ee1, 0xe9e11feb, 0x1208121f, 0x0000e11f,
  0x001f0f1f, 0x001c1fef, 0x0be6e1e1, 0x00000000,
  0xf71100e1, 0x0f1f00e4, 0x0e1f00f9, 0x00000000,
  0x80806a7f, 0x80808080, 0x8080707f, 0x80808080};

/** ============================================================================
 *   @n@b computeParams_secRateMatch
 *
 *   @b Description
 *   @n Utility function used by WCDMA 3GPP --> BCP Params mapper API 
 *      @a computeParams_hsupa() to calculate 2nd loop Rate matching params.
 *
 *   @param[in]  
 *   @n Nedata          Number of rate matching output bits
 * 
 *   @param[in]  
 *   @n Nsys            Number of 2nd loop rate matching input systematic bits
 * 
 *   @param[in]  
 *   @n Np1             Number of 2nd loop rate matching input parity1 bits
 * 
 *   @param[in]  
 *   @n Np2             Number of 2nd loop rate matching input parity1 bits 
 *
 *   @param[in]  
 *   @n r               
 * 
 *   @param[in]  
 *   @n s               
 * 
 *   @param[in]  
 *   @n rmax            
 * 
 *   @param[out]  
 *   @n pEinit          Init value for rate matching loop
 * 
 *   @param[out]  
 *   @n pEminus         Minus value for rate matching loop
 * 
 *   @param[out]  
 *   @n pEplus          Plus value for rate matching loop
 * 
 *   @param[out]  
 *   @n pNtsys          Number of systematic bits for the loop
 * 
 *   @param[out]  
 *   @n pPunctureFlag   Indicates if puncturing is to be performed
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
static Void computeParams_secRateMatch
( 
    Int32               Nedata, 
	Int32               Nsys,  
    Int32               Np1,   
    Int32               Np2,   
	Int32               r,
	Int32               s,
	Int32               rmax,
	Int32               *pEinit,
	Int32               *pEminus,
	Int32               *pEplus,
	Int32               *pNtsys,
	Int32               *pPunctureFlag
)
{
	Int32               einit[3];
	Int32               eminus[3];
	Int32               eplus[3];
	Int32               punctureFlag;
	Int32               Ntsys;
	Int32               Ntp1;
	Int32               Ntp2;
	Int32               temp_int32;
	double              temp_double;
	Int32               Xi[3];
	UInt32              i;
	
	if (Nedata <= Nsys+Np1+Np2)//puncturing case
	{
		punctureFlag = 1;
	}
	else if (Nedata > Nsys+Np1+Np2) //repeatition case
	{
		punctureFlag = 0;
	}

	/* get einit, eplus and eminus for the second RM based on 3GPP 25.212 section 4.5.4.3 */
	if (punctureFlag == 1)//puncturing case
	{
		if (s==1) //systematic bits are prioritised 
		{
			if (Nsys < Nedata)
				Ntsys = Nsys;
			else
				Ntsys = Nedata;
		}
        else if (s==0) //systematic bits are not prioritised 
		{
			temp_int32 = (Int32)Nedata-(Np1+Np2);
			if (temp_int32>0)
				Ntsys = temp_int32;
			else
				Ntsys = 0;
		}
	}
	else // repeatition case
	{
		temp_double = (double)Nsys*Nedata/(Nsys+2*Np1);
		Ntsys = (Int32)floor(temp_double); 
	}

	temp_double = (double)(Nedata-Ntsys)/2;
	Ntp1 = (Int32)floor(temp_double);
	Ntp2 = (Int32)ceil (temp_double);

	/* 3GPP 25.212 section 4.5.4.3 table 10 */	
	Xi[0] = Nsys;
	Xi[1] = Np1;
	Xi[2] = Np2;

	/* get eplus */
	eplus[0] = Nsys;
	eplus[1] = 2*Np1;
	eplus[2] = Np2;

	/* get eminus */
	temp_int32 = (Int32)Nsys-Ntsys;
	eminus[0]  = (Int32)abs(temp_int32);

    temp_int32 = (Int32)Np1-Ntp1;
	eminus[1]  = (Int32)2*abs(temp_int32);

	temp_int32 = (Int32)Np2-Ntp2;
	eminus[2]  = (Int32)abs(temp_int32);

	/* get einit */
	for (i = 0; i < 3; i++)
	{
		if (punctureFlag == 1)//puncturing case	
			temp_double = (double)r*eplus[i]/rmax;
		else//repeating case
			temp_double = (double)(s+2*r)*eplus[i]/(2*rmax);

		temp_int32 = (Int32)(Xi[i]-(Int32)floor(temp_double)-1);

		if (eplus[i] == 0)
		{
			einit[i] = temp_int32 + 1;
		}
		else
		{
			while (temp_int32 < 0)
			{
				temp_int32 += eplus[i];
			}
			einit[i] = (temp_int32%eplus[i]) + 1;
		}
	}

	/* output */
	for (i = 0; i < 3; i++)
	{
		pEinit[i]  = einit[i];
		pEminus[i] = eminus[i];
		pEplus[i]  = eplus[i];
	}

	*pNtsys = Ntsys;
	*pPunctureFlag = punctureFlag;
}

/** ============================================================================
 *   @n@b computeParams_hsupa
 *
 *   @b Description
 *   @n Utility function to map WCDMA 3GPP --> BCP H/w Params.
 *
 *   @param[out]  
 *   @n pHsupaParams    HSUPA test configuration.
 * =============================================================================
 */
static Void computeParams_hsupa (BcpTest_HsupaParams* pHsupaParams)
{
	Int32  TBsize; 
	Int32  modScheme;//0:BPSK, 1:4PAM 
	Int32  maxSet0; 
	Int32  nPhyCh;
	Int32  RVindex; 
	Int32  B;
	double temp_double;
	double PLnonmax;
	double PLmodswitch;
	double PLmax;
	Int32  SF;
	Int32  C;
	Int32  K;
	Int32  Ne;
	Int32  Nsys;
	Int32  Np1;
	Int32  Np2;
	Int32  rmax;
	Int32  s;
    Int32  r; 
	Int32  Nedata;
	Int32  Ntsys;
	Int32  eplus[3];
	Int32  eminus[3];
	Int32  einit[3];
	Int32  punctureFlag;
	Int32  SET1_index[11];
	Int32  SET2_index[11];
	Int32  i, j;
	Int32  num_ele_SET1;
	Int32  num_ele_SET2;
	Int32  RSN;
	Int32  Narq;
	Int32  TTIN;
	Int32  einit1_p1;
	Int32  einit1_p2;
	Int32  eplus1_p1;
	Int32  eplus1_p2;
	Int32  eminus1_p1;
	Int32  eminus1_p2;
	Int32* tab_Ndata;

	pHsupaParams->valid = 1;

	/* root parameters */		
	TBsize   = (Int32) pHsupaParams->TBsize; 
	maxSet0  = (Int32) pHsupaParams->maxSet0;//1 to 11
	RSN      = (Int32) pHsupaParams->RSN;
	Narq     = (Int32) pHsupaParams->Narq;
	TTIN     = (Int32) pHsupaParams->TTIN;
	PLnonmax = (double)pHsupaParams->PLnonmax;

	/* constant */
	PLmodswitch = 0.468;
	PLmax = 0.44;
	if (maxSet0 > 9)
		PLmax = 0.33;

	if (pHsupaParams->ttiType == 0)
		tab_Ndata = (Int32 *)tab_Ndata_TTI2ms;
	else
		tab_Ndata = (Int32 *)tab_Ndata_TTI10ms;

	/* start calculate */
	B = TBsize + 24;

	temp_double = (double)B/5114;
    C = (Int32)ceil(temp_double);

    if (B < 40)
	{
		K = 40;
	}
	else
	{
		temp_double = (double)B/C;
		K = (Int32)ceil(temp_double);
	}
		
    /* the number of turbo encoder output bit */ 
	/* the number of rate matching input bit  */ 
	Ne = C*(3*K+12);
	
	/* 3GPP25.212 section 4.8.4.1 */
	/* determine SF, modulation, nPhyCh and the number of rate matching output bit */
	/* initlization to zore */
	for (i = 0; i < 11; i++)
	{
		SET1_index[i] = 0;
		SET2_index[i] = 0;
	}

	/* get SET1 */
	j = 0;
	for (i = 0; i < maxSet0; i++)
	{
		if (tab_Ndata[i] >= Ne)
		{
			SET1_index[j] = i;
			j = j+1;
		}
	}
	num_ele_SET1 = j;

	/* if SET1 is not empty and the smallest element of SET1 requires just one E-DPDCH */
	if ((num_ele_SET1 > 0) && (tab_Num_E_DPDCH[SET1_index[0]]==1)) 
	{
		    Nedata = tab_Ndata[SET1_index[0]];
	}
	else /* else of SET1 is not empty and the smallest element of SET1 requires just one E-DPDCH */
	{
		/* get SET2 */
		j = 0;
		for (i = 0; i < maxSet0; i++)
		{
			if((double)tab_Ndata[i] >= (double)PLnonmax*Ne)
			{
				SET2_index[j] = i;
				j= j+1;
			}
		}
		num_ele_SET2 = j;
    
		/* SET2 is not empty */
		if(num_ele_SET2 > 0)
		{
			i = 0;
			Nedata = tab_Ndata[SET2_index[i]]; 
			while ((Nedata<tab_Ndata[SET2_index[num_ele_SET2-1]])&&(tab_Num_E_DPDCH[SET2_index[i+1]]==1))
			{
				i = i+1;
				Nedata = tab_Ndata[SET2_index[i]]; 
			}
        
			temp_double = (double)Nedata/Ne;
			if ((Nedata == tab_Ndata[10])&&((double)temp_double>= (double)PLmodswitch))
				Nedata = tab_Ndata[9];
        
			temp_double = (double)Nedata/Ne;
			if ((Nedata == tab_Ndata[9])&&((double)temp_double<(double)PLmodswitch))
				Nedata = tab_Ndata[maxSet0-1];  
		}   
		else /* else of SET2 is not empty */
		{
			Nedata = tab_Ndata[maxSet0-1]; 
			if (((double)Nedata - (double)PLmax*Ne) < 0) 
			{
#ifdef BCP_TEST_DEBUG                    
				Bcp_osalLog("Nedata-PLmax*Ne is negative, this is a invalid test case\n");
#endif
				pHsupaParams->valid = 0;
				return;
			}
				
		} /* end of SET2 is not empty */
	}/* end if SET1 is not empty and the smallest element of SET1 requires just one E-DPDCH */

	for(i = 0; i < maxSet0; i++)
	{
		if (Nedata == tab_Ndata[i])
		{
			nPhyCh    = tab_Num_E_DPDCH[i];
			modScheme = tab_Mod_scheme[i];
			SF        = tab_SF[i];  
		}
	}

	/* the 1st RM is transparent */
	Nsys = Ne/3;
	Np1  = Ne/3;
	Np2  = Ne/3;
	rmax = 2;

	/* */
	einit1_p1 = Np1;//any value greater than 0
	einit1_p2 = Np2;//any value greater than 0

	eplus1_p1 = 2*Np1;//it is not be used actually
	eplus1_p2 = 1*Np2;//it is not be used actually

	eminus1_p1 = 0;//must be 0
	eminus1_p2 = 0;//must be 0
	/* */

	/* get RV index (s and r) from RSN 3GPP 25.212 4.9.2.2 */
	/* RSN on E-DPCCH is only 2 bit and saturates at 3. But it is possible to */
	/* have more transmissions, in this case also the CFN is used to calculate */
	/* the current transmission, see 3GPP TS25.212, section 4.9.2.2 */ 
//if RVidxType == 0% if only RV index 0 used independently of the RSN %
//    RVindex = 0;
//else %RV index according to RSN%
//    if TTI == 0%2ms%
//        Narq = 8;
//        TTIN = 5*CFN + subfrmIdx;
//    else %10ms %
//        Narq = 4;
//        TTIN = CFN;
//    end

    if (RSN > 3)
        RSN = 3;//RSN on E-DPCCH is only 2 bit and saturates at 3 

	temp_double = (double)TTIN/Narq;
    if (((double)Nsys/Nedata) < 0.5)
	{
        if (RSN == 0)RVindex = 0;
        if (RSN == 1)RVindex = 2;
        if (RSN == 2)RVindex = 0;
        if (RSN == 3)RVindex = (((Int32)floor(temp_double))%2)*2;
	}
    else
	{
        if (RSN == 0)RVindex = 0;
        if (RSN == 1)RVindex = 3;
        if (RSN == 2)RVindex = 2;
        if (RSN == 3)RVindex = ((Int32)floor(temp_double))%4;
    }
//end

	s = tab_s[RVindex];
	r = tab_r[RVindex];

	/* the 2nd RM */
	computeParams_secRateMatch(Nedata,//number of rate matching output bit 
							  Nsys,  //number of 2nd rate matching input sys bits
							  Np1,   //number of 2nd rate matching input par1 bits
							  Np2,   //number of 2nd rate matching input par2 bits
							  r,
							  s,
							  rmax,
							  (Int32*)einit,
							  (Int32*)eminus,
							  (Int32*)eplus,
							  (Int32*)&Ntsys,
							  (Int32*)&punctureFlag);

	/* output parameters */
	pHsupaParams->numCodeBks    = C;
	pHsupaParams->codeBkSizeK   = K;
    pHsupaParams->numFillerBits = C*K-B;
	pHsupaParams->Ne            = Ne;
	pHsupaParams->modScheme     = modScheme;
	pHsupaParams->nPhyCh        = nPhyCh;
	//pHsupaParams->SF            = SF;
	pHsupaParams->Nedata        = Nedata;
    pHsupaParams->einit_sys     = einit[0];
    pHsupaParams->eplus_sys     = eplus[0];
	pHsupaParams->eminus_sys    = eminus[0];
	pHsupaParams->einit_p1      = einit[1];
    pHsupaParams->eplus_p1      = eplus[1]; 
	pHsupaParams->eminus_p1     = eminus[1]; 
	pHsupaParams->einit_p2      = einit[2]; 
	pHsupaParams->eplus_p2      = eplus[2]; 
	pHsupaParams->eminus_p2     = eminus[2]; 
	/**/
	pHsupaParams->einit1_p1     = einit1_p1;
	pHsupaParams->eplus1_p1     = eplus1_p1;
	pHsupaParams->eminus1_p1    = eminus1_p1;
	pHsupaParams->einit1_p2     = einit1_p2;
	pHsupaParams->eplus1_p2     = eplus1_p2;
	pHsupaParams->eminus1_p2    = eminus1_p2;
	/**/
	pHsupaParams->punctureFlag  = punctureFlag;

	if (nPhyCh == 4)
	{
		pHsupaParams->hsupaPhyCh[0].SF = 2;
		pHsupaParams->hsupaPhyCh[1].SF = 2;
		pHsupaParams->hsupaPhyCh[2].SF = 4;
		pHsupaParams->hsupaPhyCh[3].SF = 4;

#if 0
		if (commonPartialSF != 0xFF)
		{
			pHsupaParams->hsupaPhyCh[0].SFradio = pHsupaParams->hsupaPhyCh[0].SF/commonPartialSF;
			pHsupaParams->hsupaPhyCh[1].SFradio = pHsupaParams->hsupaPhyCh[1].SF/commonPartialSF;
			pHsupaParams->hsupaPhyCh[2].SFradio = pHsupaParams->hsupaPhyCh[2].SF/commonPartialSF;
			pHsupaParams->hsupaPhyCh[3].SFradio = pHsupaParams->hsupaPhyCh[3].SF/commonPartialSF;

			pHsupaParams->hsupaPhyCh[0].partialSF = commonPartialSF;
			pHsupaParams->hsupaPhyCh[1].partialSF = commonPartialSF;
			pHsupaParams->hsupaPhyCh[2].partialSF = commonPartialSF;
			pHsupaParams->hsupaPhyCh[3].partialSF = commonPartialSF;
		}

		if (commonSFradio != 0xFF)
		{
			pHsupaParams->hsupaPhyCh[0].partialSF = pHsupaParams->hsupaPhyCh[0].SF/commonSFradio;
			pHsupaParams->hsupaPhyCh[1].partialSF = pHsupaParams->hsupaPhyCh[1].SF/commonSFradio;
			pHsupaParams->hsupaPhyCh[2].partialSF = pHsupaParams->hsupaPhyCh[2].SF/commonSFradio;
			pHsupaParams->hsupaPhyCh[3].partialSF = pHsupaParams->hsupaPhyCh[3].SF/commonSFradio;

			pHsupaParams->hsupaPhyCh[0].SFradio = commonSFradio;
			pHsupaParams->hsupaPhyCh[1].SFradio = commonSFradio;
			pHsupaParams->hsupaPhyCh[2].SFradio = commonSFradio;
			pHsupaParams->hsupaPhyCh[3].SFradio = commonSFradio;
		}
#endif

		if(modScheme == 1)//BPSK
		{
			pHsupaParams->hsupaPhyCh[0].Ndata = 3840;//N2
			pHsupaParams->hsupaPhyCh[1].Ndata = 3840;//N2
			pHsupaParams->hsupaPhyCh[2].Ndata = 1920;//N4
			pHsupaParams->hsupaPhyCh[3].Ndata = 1920;//N4
		}
		else//modScheme == 2,4PAM
		{
			pHsupaParams->hsupaPhyCh[0].Ndata = 7680;//M2
			pHsupaParams->hsupaPhyCh[1].Ndata = 7680;//M2
			pHsupaParams->hsupaPhyCh[2].Ndata = 3840;//M4
			pHsupaParams->hsupaPhyCh[3].Ndata = 3840;//M4
		}
	}
	else //nPhyCh == 1 or nPhyCh == 2 
	{
		for (i = 0; i < nPhyCh; i++)
		{
			pHsupaParams->hsupaPhyCh[i].SF      = SF;

#if 0
			if (commonPartialSF != 0xFF)
			{
				pHsupaParams->hsupaPhyCh[i].SFradio = SF/commonPartialSF;
				pHsupaParams->hsupaPhyCh[i].partialSF = commonPartialSF;
			}

			if (commonSFradio != 0xFF)
			{
				pHsupaParams->hsupaPhyCh[i].partialSF = SF/commonSFradio;
				pHsupaParams->hsupaPhyCh[i].SFradio = commonSFradio;
			}
#endif

			pHsupaParams->hsupaPhyCh[i].Ndata   = Nedata/nPhyCh;
		}
	}

#if 0
	if (SF < commonPartialSF) pHsupaParams->valid = 0;//SF must be greater then or equal to partialSF, otherwise this test case is invalid
#endif
		
}

/** ============================================================================
 *   @n@b computeParams_hsupa_partialSF
 *
 *   @b Description
 *   @n Utility function.
 *
 *   @param[out]  
 *   @n pHsupaParams    HSUPA test configuration.
 * =============================================================================
 */
static Void computeParams_hsupa_partialSF (BcpTest_HsupaParams*    pHsupaParams)
{
	Uint32 i;

	if (pHsupaParams->commonPartialSF != 0xFF)
	{
		for (i = 0; i < pHsupaParams->nPhyCh; i++)
		{
			pHsupaParams->hsupaPhyCh[i].SFradio   = pHsupaParams->hsupaPhyCh[i].SF/pHsupaParams->commonPartialSF;
			pHsupaParams->hsupaPhyCh[i].partialSF = pHsupaParams->commonPartialSF;
		}
	}

	if (pHsupaParams->commonSFradio != 0xFF)
	{
		for (i = 0; i < pHsupaParams->nPhyCh; i++)
		{
			pHsupaParams->hsupaPhyCh[i].partialSF = pHsupaParams->hsupaPhyCh[i].SF/pHsupaParams->commonSFradio;
			pHsupaParams->hsupaPhyCh[i].SFradio   = pHsupaParams->commonSFradio;
		}
	}
}

/** ============================================================================
 *   @n@b prepare_corhdr_cfg
 *
 *   @b Description
 *   @n Sets up Correlation engine header configuration as per inputs provided.
 *
 *   @param[out]  
 *   @n pCorHdrCfg      Correlation header thus built
 *
 *   @param[in]  
 *   @n pHsupaParams    HSUPA test configuration.
 *
 *   @param[in]  
 *   @n channelType     Channel type.
 * =============================================================================
 */
static Void prepare_corhdr_cfg 
(
    Bcp_CorHdrCfg*          pCorHdrCfg, 
    BcpTest_HsupaParams*    pHsupaParams, 
    UInt8                   channelType
)
{
	UInt32                  i;
	UInt32                  SFradio, SFradio1;

    memset (pCorHdrCfg, 0, sizeof (Bcp_CorHdrCfg));

    /* Setup the correlation header as per inputs specified */
    pCorHdrCfg->local_hdr_len       =   2;
    pCorHdrCfg->pucch_despread_sel  =   Bcp_CorPucchDespread_Sel_DESPREAD;

	for (i = 0; i < pHsupaParams->nPhyCh; i++)
	{
		SFradio = pHsupaParams->hsupaPhyCh[i].SFradio;
		if (SFradio ==   1) SFradio1 = 0; 
		if (SFradio ==   2) SFradio1 = 1; 
		if (SFradio ==   4) SFradio1 = 2; 
		if (SFradio ==   8) SFradio1 = 3; 
		if (SFradio ==  16) SFradio1 = 4; 
		if (SFradio ==  32) SFradio1 = 5; 
		if (SFradio ==  64) SFradio1 = 6; 
		if (SFradio == 128) SFradio1 = 7; 
        pCorHdrCfg->block_params[i].sf_ratio        =   SFradio1;
	}
	
	for (i = 0; i < pHsupaParams->nPhyCh; i++)
        pCorHdrCfg->block_params[i].despreading_length  =   2560*3/pHsupaParams->hsupaPhyCh[i].partialSF;

	if (channelType == WCDMAFDD_HSUPA)//real
        pCorHdrCfg->despread_flag_cplx  =   Bcp_CorDespread_Cplx_16REAL;
	else//complex
        pCorHdrCfg->despread_flag_cplx  =   Bcp_CorDespread_Cplx_32COMPLEX;

    return;
}

/** ============================================================================
 *   @n@b prepare_fdd_sslhdr_cfg
 *
 *   @b Description
 *   @n Sets up Soft Slicer (SSL) engine header configuration as per inputs provided.
 *
 *   @param[out]  
 *   @n pSslHdrCfg      WCDMA FDD SSL header thus built
 *
 *   @param[in]  
 *   @n pHsupaParams    HSUPA test configuration.
 *
 *   @param[in]  
 *   @n channelType     Channel type.
 * =============================================================================
 */
static Void prepare_fdd_sslhdr_cfg 
(
    Bcp_SslHdr_WcdmaFddCfg*     pSslHdrCfg, 
    BcpTest_HsupaParams*        pHsupaParams, 
    UInt8                       channelType
)
{
	UInt32                      i;
	const float                 a[2]= {1.0, 0.4472};
	float                       unit_ssl;
	Int16                       unit_ssl16;
	float                       noiseVar_ssl;
	Int32                       noiseVar_ssl32;
	Int32                       noiseVar2_ssl32;
	UInt32                      numSegment;
	Int32                       noiseVar2_ssl;

    memset (pSslHdrCfg, 0, sizeof (Bcp_SslHdr_WcdmaFddCfg));

    /* Setup the WCDMA FDD SSL Header as per inputs passed */
    pSslHdrCfg->local_hdr_len               =   15;
    pSslHdrCfg->modeSelCfg.fdd_tdd_sel      =   Bcp_SslFddTdd_Sel_FDD;
    pSslHdrCfg->modeSelCfg.jack_bit         =   0;
    pSslHdrCfg->modeSelCfg.tti_2ms_10ms_sel =   (Bcp_Tti_Sel) pHsupaParams->ttiType; 

	if (pHsupaParams->modScheme == 1)
        pSslHdrCfg->modeSelCfg.mod_type_sel =   Bcp_ModulationType_BPSK;
	else if (pHsupaParams->modScheme == 2)
        pSslHdrCfg->modeSelCfg.mod_type_sel =   Bcp_ModulationType_4PAM;
	else
    {
#ifdef BCP_TEST_DEBUG
		Bcp_osalLog("Wrong HSUPA modulation mode!\n");
#endif
        return;
    }

    pSslHdrCfg->modeSelCfg.q_format         =   (Bcp_QFormat) pHsupaParams->qFmt;

    pSslHdrCfg->modeSelCfg.wcmda_num_phy_ch =   pHsupaParams->nPhyCh;

    if (pHsupaParams->ttiType == Bcp_Tti_Sel_2ms)
	{
		if (pHsupaParams->nPhyCh == 4) 
		{
			numSegment = 12;

			/* unit */
			unit_ssl    = (float)pHsupaParams->hsupaPhyCh[0].modRms * a[pHsupaParams->modScheme-1];
			unit_ssl16  = (Uint16) (unit_ssl + 0.5);//float represent into Int16 
			for (i = 0; i < 6; i++)
                pSslHdrCfg->uva[i]  =   unit_ssl16;

			unit_ssl    = (float)pHsupaParams->hsupaPhyCh[2].modRms * a[pHsupaParams->modScheme-1];
			unit_ssl16  = (Uint16) (unit_ssl + 0.5);//float represent into Int16 
			for (i = 6; i < 12; i++)
                pSslHdrCfg->uva[i]  =   unit_ssl16;

			if (pHsupaParams->hsupaPhyCh[0].noiseVar == 0)
			{
				noiseVar_ssl32 = 0x7F800000;
			}
			else
			{
				noiseVar_ssl = (float)(1/(pHsupaParams->hsupaPhyCh[0].noiseVar*2));
				noiseVar_ssl32 = (*((Int32*)&noiseVar_ssl));//float represent into Int32 
			}

			if (pHsupaParams->hsupaPhyCh[2].noiseVar == 0)
			{
				noiseVar2_ssl32 = 0x7F800000;
			}
			else
			{
				noiseVar2_ssl = (float)(1/(pHsupaParams->hsupaPhyCh[2].noiseVar*2));
				noiseVar2_ssl32 = (*((Int32*)&noiseVar_ssl));//float represent into Int32 
			}

			for (i = 0; i < 6; i++)
                pSslHdrCfg->scale_c0 [i]    =   noiseVar_ssl32;                   
			for (i = 6; i < 12; i++)
                pSslHdrCfg->scale_c0 [i]    =   noiseVar2_ssl32;                   
		}
		else //if((pHsupaParams->nPhyCh == 1)||(pHsupaParams->nPhyCh == 2))
		{
			numSegment = 6;

			/* unit */
			unit_ssl    = (float)pHsupaParams->hsupaPhyCh[0].modRms * a[pHsupaParams->modScheme-1];
			unit_ssl16  = (Uint16) (unit_ssl + 0.5);//float represent into Int16
			
			for (i = 0; i < 6; i++)
                pSslHdrCfg->uva[i]  =   unit_ssl16;

			if (pHsupaParams->hsupaPhyCh[0].noiseVar == 0)
			{
				noiseVar_ssl32 = 0x7F800000;
			}
			else
			{
				noiseVar_ssl = (float)(1/(pHsupaParams->hsupaPhyCh[0].noiseVar*2));
				noiseVar_ssl32 = (*((Int32*)&noiseVar_ssl));//float represent into Int32 
			}

			for (i = 0; i < 6; i++)
                pSslHdrCfg->scale_c0 [i]    =   noiseVar_ssl32;                   
		}
	}
	else if (pHsupaParams->ttiType == Bcp_Tti_Sel_10ms)
	{
		if (pHsupaParams->nPhyCh == 4) 
			numSegment = 10;
		else//if((pHsupaParams->nPhyCh == 1)||(pHsupaParams->nPhyCh == 2))
			numSegment = 5;
	}
	else
	{
#ifdef BCP_TEST_DEBUG
		Bcp_osalLog("wrong TTI type!\n");
#endif
        return;
	}

    pSslHdrCfg->wcdma_symb_seq      =   (1280/pHsupaParams->hsupaPhyCh[0].SF);
    

    ((void)numSegment);
    ((void)noiseVar2_ssl);

    return;
}

/** ============================================================================
 *   @n@b prepare_dnthdr_cfg
 *
 *   @b Description
 *   @n Sets up De-interleaver (DNT) engine header configuration as per inputs provided.
 *
 *   @param[out]  
 *   @n pDntHdrCfg      DNT header thus built
 *
 *   @param[in]  
 *   @n pHsupaParams    HSUPA test configuration.
 *
 *   @param[in]  
 *   @n channelType     Channel type.
 * =============================================================================
 */
static Void prepare_dnthdr_cfg
(
    Bcp_DntHdrCfg*              pDntHdrCfg, 
    BcpTest_HsupaParams*        pHsupaParams, 
    UInt8                       channelType
)
{
	UInt32      i;

    memset (pDntHdrCfg, 0, sizeof (Bcp_DntHdrCfg));

    /* Setup DNT Header as per inputs specified. */
    pDntHdrCfg->local_hdr_len   =   8;

	for (i = 0; i < pHsupaParams->nPhyCh; i++)
        pDntHdrCfg->tblCfg[i].num_r2_length         =   (UInt32)floor((double)pHsupaParams->hsupaPhyCh[i].Ndata/pHsupaParams->modScheme/30);

	for (i = pHsupaParams->nPhyCh; i < 6; i++)
        pDntHdrCfg->tblCfg[i].num_r2_length         =   0;

	for (i = 0; i < 6; i++)
        pDntHdrCfg->tblCfg[i].num_dummy             =   0;

    pDntHdrCfg->num_frame_count                     =   pHsupaParams->nPhyCh;

	for (i = 0; i < pHsupaParams->nPhyCh; i++)
        pDntHdrCfg->tblCfg[i].num_data_format_in    =   8 * pHsupaParams->modScheme;

	for (i = pHsupaParams->nPhyCh; i< 6; i++)
        pDntHdrCfg->tblCfg[i].num_data_format_in    =   0;

    pDntHdrCfg->num_data_value                      =   pHsupaParams->Nedata;
    pDntHdrCfg->num_constellation                   =   0;
    pDntHdrCfg->num_descramble                      =   0;

    return;
}

/** ============================================================================
 *   @n@b prepare_rdhdr_cfg
 *
 *   @b Description
 *   @n Sets up Rate dematching (RD) engine header configuration as per inputs provided.
 *
 *   @param[out]  
 *   @n pRdHdrCfg       RD header thus built
 *
 *   @param[in]  
 *   @n pHsupaParams    HSUPA test configuration.
 *
 *   @param[in]  
 *   @n channelType     Channel type.
 * =============================================================================
 */
static Void prepare_rdhdr_cfg
(
    Bcp_RdHdr_xCdmaCfg*         pRdHdrCfg, 
    BcpTest_HsupaParams*        pHsupaParams, 
    UInt8                       channelType
)
{
    memset (pRdHdrCfg, 0, sizeof (Bcp_RdHdr_xCdmaCfg));        

    /* Setup the RD header as per inputs passed */
    pRdHdrCfg->local_hdr_len    =   22;

	/* systematic */
    pRdHdrCfg->sys0_len         =   pHsupaParams->Ne/3;
    pRdHdrCfg->sys0_init2       =   pHsupaParams->einit_sys;
    pRdHdrCfg->sys0_minus2      =   pHsupaParams->eminus_sys;
    pRdHdrCfg->sys0_plus2       =   pHsupaParams->eplus_sys;
    pRdHdrCfg->sys0_alpha       =   0;
    pRdHdrCfg->sys0_beta        =   0;
    pRdHdrCfg->sys0_puncture    =   pHsupaParams->punctureFlag;
    pRdHdrCfg->sys0_turbo       =   3;

	/* Setup Parity1 */
    pRdHdrCfg->p0_par1_len      =   pHsupaParams->Ne/3;
    pRdHdrCfg->p0_par1_init1    =   pHsupaParams->Ne/3;
    pRdHdrCfg->p0_par1_minus1   =   0;
    pRdHdrCfg->p0_par1_plus1    =   pHsupaParams->Ne/3;
    pRdHdrCfg->p0_par1_init2    =   pHsupaParams->einit_p1;
    pRdHdrCfg->p0_par1_minus2   =   pHsupaParams->eminus_p1;
    pRdHdrCfg->p0_par1_plus2    =   pHsupaParams->eplus_p1;

	/* Setup Parity2 */
    pRdHdrCfg->p0_par2_len      =   pHsupaParams->Ne/3;
    pRdHdrCfg->p0_par2_init1    =   pHsupaParams->Ne/3;
    pRdHdrCfg->p0_par2_minus1   =   0;
    pRdHdrCfg->p0_par2_plus1    =   pHsupaParams->Ne/3;
    pRdHdrCfg->p0_par2_init2    =   pHsupaParams->einit_p2;
    pRdHdrCfg->p0_par2_minus2   =   pHsupaParams->eminus_p2;
    pRdHdrCfg->p0_par2_plus2    =   pHsupaParams->eplus_p2;

    pRdHdrCfg->flow_id_init     =   0;
    pRdHdrCfg->flow_id_max      =   0;

	if (channelType == WCDMAFDD_HSUPA)  //HSUPA no collection table, this two parameters not used,just set to 0 
	{
        pRdHdrCfg->fdd          =   1;
        pRdHdrCfg->collect_cols =   0;
        pRdHdrCfg->collect_rows =   0;
	}
    pRdHdrCfg->turbo_length     =   pHsupaParams->Ne/3/pHsupaParams->numCodeBks;
    pRdHdrCfg->turbo_count      =   pHsupaParams->numCodeBks;
    pRdHdrCfg->tcp3d_dyn_range  =   0;
    pRdHdrCfg->tcp3d_scale_factor   =   16;
	if (pHsupaParams->RSN == 0)
        pRdHdrCfg->en_harq_in   =   0;            
	else
        pRdHdrCfg->en_harq_in   =   1;            
    pRdHdrCfg->en_harq_out      =   1;            
    pRdHdrCfg->harq_in_addr     =   0xC000000;            
    pRdHdrCfg->harq_out_addr    =   0xC000000;            

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
 *   @n@b add_test_config_data
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
static Int32 add_test_config_data (Bcp_DrvHandle hBcp, UInt8*  pDataBuffer)
{
    FILE*                       pTestCfgFile;        
    BcpTest_HsupaParams         hsupaParams;
    UInt32                      dataBufferLen, tmpLen, ttiNum, RSN, temp;
    UInt8                       wcdmaChanType;
    Bcp_RadioStd                radioStd;
    Bcp_GlobalHdrCfg            bcpGlblHdrCfg;
    Bcp_TmHdrCfg                tmHdrCfg;
    Bcp_CorHdrCfg               corHdrCfg;
    Bcp_SslHdr_WcdmaFddCfg      sslHdrCfg;
    Bcp_DntHdrCfg               dntHdrCfg;
    Bcp_RdHdr_xCdmaCfg          rdHdrCfg;
	float                       c_float;
    UInt8*  					pStartDataBuffer;

#ifdef BCP_TEST_DEBUG
    Bcp_osalLog ("Reading test configuration ... \n");
#endif

    /* Get the HSUPA input test vector params */
    hsupaParams.ttiType         =   testVectorHsupa[0][0]; 
	hsupaParams.startTTIN       =   testVectorHsupa[0][1]; 
	hsupaParams.numTTItoRun     =   testVectorHsupa[0][2]; 
	hsupaParams.Narq            =   testVectorHsupa[0][3]; 
	hsupaParams.TBsize          =   testVectorHsupa[0][4];  
	hsupaParams.maxSet0         =   testVectorHsupa[0][5]; 
	hsupaParams.commonSFradio   =   testVectorHsupa[0][6];
	hsupaParams.commonPartialSF =   0xFF;
	hsupaParams.qFmt            =   testVectorHsupa[0][7]; 
	hsupaParams.maxNumReTrans   =   testVectorHsupa[0][8]; 
	hsupaParams.PLnonmax        =   testVectorHsupaPLnonmax[0]; 
	hsupaParams.edchCategory    =   7; 

    for (ttiNum = hsupaParams.startTTIN; ttiNum < (hsupaParams.startTTIN + hsupaParams.numTTItoRun); ttiNum ++)
    {
        RSN  = (UInt32)(ttiNum/hsupaParams.Narq);
							
        if (RSN >= 3)
            hsupaParams.RSN     =   3;
        else
            hsupaParams.RSN     =   RSN;

        hsupaParams.TTIN        =   ttiNum;

        // Read 'a' value 
		if ((pTestCfgFile = fopen ("..\\..\\testcases\\wcdma\\ST_WCDMA_FDDHSUPA_0001_E_a_val_RX.dat", "r")) ==NULL)
		{
#ifdef BCP_TEST_DEBUG
			Bcp_osalLog("Error opening test config file:  ..\..\testcases\wcdma\ST_WCDMA_FDDHSUPA_0001_E_a_val_RX.dat\n");
#endif
            return -1;
		}

        if (fscanf(pTestCfgFile, "%d", &temp) != EOF)
        {
            hsupaParams.hsupaPhyCh[0].modRms = temp;
            hsupaParams.hsupaPhyCh[1].modRms = hsupaParams.hsupaPhyCh[0].modRms;
        }
        else
        {
#ifdef BCP_TEST_DEBUG
			Bcp_osalLog("Error reading test config file \n");
#endif
            fclose (pTestCfgFile);
            return -1;
        }

        fclose (pTestCfgFile);

		if ((pTestCfgFile = fopen ("..\\..\\testcases\\wcdma\\ST_WCDMA_FDDHSUPA_0001_E_a2_val_RX.dat", "r")) ==NULL)
		{
#ifdef BCP_TEST_DEBUG
			Bcp_osalLog("Error opening test config file: ..\..\testcases\wcdma\ST_WCDMA_FDDHSUPA_0001_E_a2_val_RX.dat\n");
#endif
            return -1;
		}
        if (fscanf(pTestCfgFile, "%d", &temp) != EOF)
        {
            hsupaParams.hsupaPhyCh[2].modRms = temp;
            hsupaParams.hsupaPhyCh[3].modRms = hsupaParams.hsupaPhyCh[2].modRms;
        }
        else
        {
#ifdef BCP_TEST_DEBUG
			Bcp_osalLog("Error reading test config file \n");
#endif
            fclose (pTestCfgFile);
            return -1;
        }

        fclose (pTestCfgFile);

        // Read 'c' value 
		if ((pTestCfgFile = fopen ("..\\..\\testcases\\wcdma\\ST_WCDMA_FDDHSUPA_0001_E_c_val_RX.dat", "r")) ==NULL)
		{
#ifdef BCP_TEST_DEBUG
			Bcp_osalLog("Error opening test config file: ..\..\testcases\wcdma\ST_WCDMA_FDDHSUPA_0001_E_c_val_RX.dat\n");
#endif
            return -1;
		}

        if (fscanf(pTestCfgFile, "%f", &c_float) != EOF)
        {
            hsupaParams.hsupaPhyCh[0].noiseVar = (double)1/(2*(double)c_float);
            hsupaParams.hsupaPhyCh[1].noiseVar = hsupaParams.hsupaPhyCh[0].noiseVar;
        }
        else
        {
#ifdef BCP_TEST_DEBUG
			Bcp_osalLog("Error reading test config file \n");
#endif
            fclose (pTestCfgFile);
            return -1;
        }

        fclose (pTestCfgFile);

        if ((pTestCfgFile = fopen ("..\\..\\testcases\\wcdma\\ST_WCDMA_FDDHSUPA_0001_E_c2_val_RX.dat", "r")) ==NULL)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog("Error opening test config file: ..\..\testcases\wcdma\ST_WCDMA_FDDHSUPA_0001_E_c2_val_RX.dat\n");
#endif
            return -1;
        }

        if (fscanf(pTestCfgFile, "%f", &c_float) != EOF)
        {
            hsupaParams.hsupaPhyCh[2].noiseVar = (double)1/(2*(double)c_float);
            hsupaParams.hsupaPhyCh[3].noiseVar = hsupaParams.hsupaPhyCh[2].noiseVar; 
        }
        else
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog("Error reading test config file \n");
#endif
            fclose (pTestCfgFile);
            return -1;
        }

        fclose (pTestCfgFile);
            
        /* Generate rest of the parameters */
        computeParams_hsupa (&hsupaParams);
        computeParams_hsupa_partialSF (&hsupaParams);

        radioStd        =   Bcp_RadioStd_WCDMA_R99; 
        wcdmaChanType   =   WCDMAFDD_HSUPA;

        /* Start adding BCP Packet headers based on the test configuration we read. */

        /* Initialize our data buffer length running counter */
        dataBufferLen   =   0;
        tmpLen			=	0;

        /* Save the start pointer and set running pointer to CRC header */
        pStartDataBuffer = pDataBuffer;
        pDataBuffer 	+=	8;
        dataBufferLen	+=	8;

        /* Header 2: Correlation Header */
        prepare_corhdr_cfg (&corHdrCfg, &hsupaParams, wcdmaChanType);
        if (Bcp_addCorrelationHeader (&corHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Failed to add Correlation header to packet \n");            
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;

        /* Header 3: Soft Slicer (SSL) Header */
        prepare_fdd_sslhdr_cfg (&sslHdrCfg, &hsupaParams, wcdmaChanType);
        if (Bcp_addWCDMAFDD_SSLHeader (&sslHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Failed to add SSL header to packet \n");            
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;

        /* Header 4: Deinterleaver Header */
        prepare_dnthdr_cfg (&dntHdrCfg, &hsupaParams, wcdmaChanType);
        if (Bcp_addDeinterleaverHeader (&dntHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Failed to add Deinterleaver header to packet \n");            
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;

        /* Header 5: Rate dematching Header */
        prepare_rdhdr_cfg (&rdHdrCfg, &hsupaParams, wcdmaChanType);
        if (Bcp_addxCdma_RDHeader (&rdHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Failed to add Rate dematching header to packet \n");            
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;

        /* Header 6: Traffic Manager header */
        prepare_tmhdr_cfg (&tmHdrCfg);
        if (Bcp_addTMHeader (&tmHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Failed to add Traffic Manager header to packet \n");            
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
        bcpGlblHdrCfg.hdr_end_ptr       =   ((dataBufferLen + 3) >> 2);
        bcpGlblHdrCfg.flow_id           =   RX_FLOW_ID;
        bcpGlblHdrCfg.destn_tag         =   0xDEAD;
        if (Bcp_addGlobalHeader (&bcpGlblHdrCfg, pStartDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
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
        if ((pTestCfgFile = fopen("..\\..\\testcases\\wcdma\\bcp_fdd_hsupa_input.dat","r")) == NULL)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Cannot open data input file: ..\..\testcases\wcdma\bcp_fdd_hsupa_input.dat\n");
#endif
            return -1;
        }
        read_data_from_file (pTestCfgFile, pDataBuffer, &dataBufferLen);
        fclose (pTestCfgFile);
    }
       
    /* Successfully read the test configuration */        
    return dataBufferLen;
}


/** ============================================================================
 *   @n@b test_wcdma_ul
 *
 *   @b Description
 *   @n Sets up the parameters required and runs an WCDMA Uplink (UL) test case.
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
Void test_wcdma_ul (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblFDQ)
{
    Bcp_TxCfg           txCfg;
    Bcp_TxHandle        hTx = NULL;
    Bcp_RxCfg           rxCfg;
    Bcp_RxHandle        hRx = NULL;
    Qmss_QueueHnd       hTxFDQ = NULL, hRxFDQ = NULL, hHistQ = NULL;
    UInt8               rxFlowId, rxSrcId, numPkts;
    UInt32              i, dataBufferLen, rxDataBufferLen, rxPsInfoLen, rxDataTotalLen, numRxPackets, testFail = 0;
    UInt16              rxDestnTag;
    Cppi_Desc*          pCppiDesc;
    UInt8*              pDataBuffer;
    UInt8*              pRxDataBuffer;
    UInt8*              pRxPsInfo;
    Bcp_DrvBufferHandle hRxDrvBuffer;
    Bcp_DrvBufferHandle hVoid;
    Bcp_DrvBufferHandle hTmp;
    UInt8               bAllocHistQ;
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
        Bcp_osalLog ("Error opening Tx FDQ \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("Rx FDQ %d successfully setup with %d descriptors\n", hRxFDQ, RX_NUM_DESC);
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
    rxCfg.tmFlowCfg.qfifo_out           =   RX_FLOW_ID;      
    rxCfg.tmFlowCfg.ps_flags            =   0;      // No PS for now

    if ((hRx = Bcp_rxOpen (hBcp, &rxCfg, NULL)) == NULL)
    {
        Bcp_osalLog ("BCP Rx Open failed \n");
		goto cleanup_and_return;
    }
    else
    {
        Bcp_osalLog ("Flow %d opened to send data to RxQ: %d \n", RX_FLOW_ID, RX_Q_NUM);
    }

    /* Setup Tx side:
     *  -   Open BCP Tx queue using which data would be sent 
     *
     *  -   Setup a Tx FDQ and initialize it with some Tx FDs
     *
     *  -   Setup a History queue if requested. History queue is a general 
     *      purpose queue used by the application to buffer the BCP requests
     *      (Tx descriptors) until the packets are received correctly through BCP 
     *      at the receiver end. In case of BCP errors, the application can retransmit
     *      the packet from the history queue if needed. 
     *
     *  -   Set the BCP Tunnel Tx endpoint configuration to NULL since we
     *      are processing BCP packets locally.
     */
    txCfg.txQNum    =   Bcp_QueueId_0;
    if ((hTx = Bcp_txOpen (hBcp, &txCfg, NULL)) == NULL)
    {
        Bcp_osalLog ("BCP Tx Open failed \n");
        return;
    }

#ifdef MAINTAIN_DESC_HISTORY
    /* Maintain BCP Tx packets history */
    bAllocHistQ     =   1;
#else
    /* Do not maintain any BCP Tx packets history */
    bAllocHistQ     =   0;
#endif
    if (allocate_fdq (hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, bAllocHistQ, &hTxFDQ, &hHistQ) < 0)
    {
        Bcp_osalLog ("Error opening Tx FDQ/History queue \n");            
        goto cleanup_and_return;
    }
    else
    {
        Bcp_osalLog ("Tx FDQ setup:          %d with %d Free descriptors\n", hTxFDQ, TX_NUM_DESC);
    }

#ifdef MAINTAIN_DESC_HISTORY
    Bcp_osalLog ("History Queue setup:   %d \n", hHistQ);
    Bcp_osalLog ("\nSetup Complete ... Start Send \n\n");
#endif
    
    /* Build and Send a packet with WCDMA UL parameters for BCP Processing */
    for (numPkts = 0; numPkts < BCP_TEST_NUM_PACKETS; numPkts ++)
    {
        if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hTxFDQ)) == NULL)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Out of Tx FDs! \n");
#endif
            testFail ++;
		    goto cleanup_and_return;
        }
        pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
        Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        
	    memset (pDataBuffer, 0, dataBufferLen); 

        /* Read test configuration */
        if ((dataBufferLenUsed = add_test_config_data (hBcp, pDataBuffer)) <= 0)
        {
            Bcp_osalLog ("Error reading test vectors/populating packet \n");
            testFail ++;
            goto cleanup_and_return;
        }

#ifdef BCP_TEST_DEBUG
   	    Bcp_osalLog ("Sending packet #%d: 0x%p of len %d to BCP ...\n", numPkts, pCppiDesc, dataBufferLenUsed);
#endif

        /* Setup the data length in the descriptor */
        Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLenUsed);
        Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLenUsed);

        /* Since BCP is local to the device, set destination address to NULL */
        Bcp_send (hTx, (Bcp_DrvBufferHandle) pCppiDesc, BCP_TEST_SIZE_HOST_DESC, NULL);
    }

#ifdef MAINTAIN_DESC_HISTORY
    Bcp_osalLog ("\nSend Complete ... Sent %d packets \n\n", numPkts);
    
rx_packets:    

    Bcp_osalLog ("Wait on Results from BCP ... \n");
#endif

    /* Wait on data to be received from BCP and validate it */
    for (i = 0; i < 100; i ++);
    while ((numRxPackets = Bcp_rxGetNumOutputEntries (hRx)) != BCP_TEST_NUM_PACKETS);

#ifdef BCP_TEST_DEBUG
    Bcp_osalLog ("Got %d packet(s) from BCP \n", Bcp_rxGetNumOutputEntries (hRx));
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

        if (validate_rxdata ((UInt8 *)wcdma_ul_output_packet_1, 
                            WCDMA_UL_OUTPUT_PKT_1_WRD_SIZE*4, 
                            pRxDataBuffer, 
                            rxDataBufferLen, 
                            rxDataTotalLen) != 0)
            testFail ++;                

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

                if (validate_rxdata ((UInt8 *)wcdma_ul_output_packet_1, 
                                    WCDMA_UL_OUTPUT_PKT_1_WRD_SIZE*4, 
                                    pRxDataBuffer, 
                                    rxDataBufferLen, 
                                    rxDataTotalLen) != 0)
                    testFail ++;                        

                rxDataTotalLen  +=  rxDataBufferLen;

            }
        
            Bcp_rxFreeRecvBuffer (hRx, hTmp, BCP_TEST_SIZE_HOST_DESC);
        }

#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("[Pkt %d]: Total Len: %d DestnTag: 0x%x SrcId: 0x%x (testFail: %d)\n", i, rxDataTotalLen, rxDestnTag, rxSrcId, testFail);
#endif

        if (rxDataTotalLen != WCDMA_UL_OUTPUT_PKT_1_WRD_SIZE * 4)
            testFail ++;                
    }

#ifdef MAINTAIN_DESC_HISTORY
    if (numRxPackets == numPkts)
    {
        /* All packets processed and received correctly through BCP.
         * Recycle all "N" packets from History queue to TxFDQ.
         */
        Bcp_osalLog ("\nTx/Rx Complete ...Cleanup History Queue\n\n");
        for (i = 0; i < numPkts; i ++)
        {
            if ((pCppiDesc = Qmss_queuePop (hHistQ)) == NULL)
            {
                break;                
            }

            Bcp_osalLog ("Recycling Tx packet #%d: 0x%p from History queue: %d to Tx FDQ: %d \n", 
                         numPkts, pCppiDesc, hHistQ, hTxFDQ);

            /* Recycle descriptors from History queue back to Tx FDQ */
            Qmss_queuePushDescSize (hTxFDQ, pCppiDesc, 256);
        }
    }
    else
    {
        Bcp_osalLog ("\nRx Failed ... Retransmit packets from History Queue\n\n");
        /* Not all packets received from BCP. Retransmit the packets from
         * History queue again.
         *
         * Retransmission can be done 2 ways:
         *  (1) Retransmit only packets for which results werent received 
         *      from BCP. Tx/Rx Packets can be easily matched using the 
         *      "destn_id" field.
         *
         *  (2) Blindly retransmit all "N" packets from History queue.
         *
         *  For simplicity sake, we here are implementing option (2).
         */
        for (numPkts = 0; numPkts < BCP_TEST_NUM_PACKETS; numPkts ++)
        {
            if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hHistQ)) == NULL)
            {
                Bcp_osalLog ("No descriptors in History queue! \n");
		        goto cleanup_and_return;
            }
            pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
            Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);

   	        Bcp_osalLog ("Retransmitting #%d: 0x%p packet of length %d to BCP ...\n", numPkts, pCppiDesc, dataBufferLen);
        
            /* Setup the data length in the descriptor */
            Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLen);
            Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLen);

            /* Since BCP is local to the device, set destination address to NULL */
            Bcp_send (hTx, (Bcp_DrvBufferHandle) pCppiDesc, BCP_TEST_SIZE_HOST_DESC, NULL);
        }

        Bcp_osalLog ("\nRetransmission Complete ... Sent %d packets \n\n", numPkts);

        /* Process received packets */
        goto rx_packets;
    }
#endif
        
cleanup_and_return:
    if (testFail > 0 || i != BCP_TEST_NUM_PACKETS)
    {
        Bcp_osalLog ("WCDMA UL Test:    FAILED\n");                
        totalNumTestsFail ++;
    }
    else
    {
        Bcp_osalLog ("WCDMA UL Test:    PASS\n");                
        totalNumTestsPass ++;
    }    

    if (hRx)
        Bcp_rxClose (hRx);

    deallocate_fdq (hRxFDQ, hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, NULL);

    if (hTx)
        Bcp_txClose (hTx);

#ifdef MAINTAIN_DESC_HISTORY
    if (hTxFDQ)
        deallocate_fdq (hTxFDQ, hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, hHistQ);
#else
    if (hTxFDQ)
        deallocate_fdq (hTxFDQ, hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, NULL);
#endif

    return;
}
