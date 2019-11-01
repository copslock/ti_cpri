/** 
 *   @file  test_wcdma_rel99.c
 *
 *   @brief  
 *      Runs WCDMA Rel-99 Uplink/Downlink test case using BCP driver APIs.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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
#include <math.h>
#include <stdlib.h>

#define     TX_DATA_BUFFER_SIZE             2048 
#define     RX_DATA_BUFFER_SIZE             2048
#define     TX_NUM_DESC                     16
#define     RX_NUM_DESC                     ((BCP_TEST_NUM_HOST_DESC/4) - TX_NUM_DESC)
#define     RX_Q_NUM                        900 + CSL_chipReadReg (CSL_CHIP_DNUM)
#define     RX_FLOW_ID                      0 + CSL_chipReadReg (CSL_CHIP_DNUM)

extern UInt32   totalNumTestsPass, totalNumTestsFail;

/* Reference output data */
#define R99_DL_OUTPUT_PKT_1_11_WRD_SIZE     4
#pragma DATA_SECTION (r99_dl_output_packet_1_11, ".testData");
static UInt32 r99_dl_output_packet_1_11[R99_DL_OUTPUT_PKT_1_11_WRD_SIZE] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000};

#define R99_DL_OUTPUT_PKT_12_WRD_SIZE       32
#pragma DATA_SECTION (r99_dl_output_packet_12, ".testData");
static UInt32 r99_dl_output_packet_12[R99_DL_OUTPUT_PKT_12_WRD_SIZE] = {
    0xfffecfff, 0x1ffc0ffc, 0xe7ffafff, 0x07fe03ff,
    0x8bffecff, 0x01ff80ff, 0xc4fffd3f, 0xc07ff03f,
    0xf0ffff7f, 0xf01ff80f, 0xfdf7fe6f, 0xfc07fe07,
    0xff6ffff1, 0xff01ffc0, 0xfff47ffc, 0xffc07fe0,
    0xfff6ffff, 0x3ff01ff8, 0x7fff07ff, 0x0ffc07fe,
    0xbfffd3ff, 0x03ff01ff, 0x98ffc4ff, 0x80ffc07f,
    0xe27ff35f, 0xe07ff01f, 0xfd1fff97, 0xf80ffe07,
    0xff8bffe3, 0xfe03ff01, 0xff81ffd4, 0xff80ffc0};

#define R99_DL_OUTPUT_PKT_13_WRD_SIZE       32
#pragma DATA_SECTION (r99_dl_output_packet_13, ".testData");
static UInt32 r99_dl_output_packet_13[R99_DL_OUTPUT_PKT_13_WRD_SIZE] = {
    0xfffe2ffe, 0x1ffc0ffc, 0xc7fe0fff, 0x07fe03ff,
    0x95ffa6ff, 0x01ff80ff, 0xeffffbff, 0xc07ff03f,
    0xf85ff8ff, 0xf01ff80f, 0xfd87feb7, 0xfc07fe07,
    0xffb5ffe2, 0xff01ffc0, 0xfff47fe1, 0xffc07fe0,
    0xbff8dff8, 0x3ff01ff8, 0x7ffd5ffe, 0x0ffc07fe,
    0xebfffdff, 0x03ff01ff, 0x8dffd9ff, 0x80ffc07f,
    0xf77ff03f, 0xe07ff01f, 0xfe1fff97, 0xf80ffe07,
    0xff9bff0b, 0xfe03ff01, 0xff9cffec, 0xff80ffc0};

#define R99_DL_OUTPUT_PKT_14_WRD_SIZE       32
#pragma DATA_SECTION (r99_dl_output_packet_14, ".testData");
static UInt32 r99_dl_output_packet_14[R99_DL_OUTPUT_PKT_14_WRD_SIZE] = {
    0x7ffd3fff, 0x1ffc0ffc, 0xe7ff9bff, 0x07fe03ff,
    0x5dffbdff, 0x01ff80ff, 0xdd7ff47f, 0xc07ff03f,
    0xf93ff8af, 0xf01ff80f, 0xfe7ffe4f, 0xfc07fe07,
    0xffd5ffd9, 0xff01ffc0, 0xffdeffe4, 0xffc07fe0,
    0xbff9bffd, 0x3ff01ff8, 0x0ffc7fff, 0x0ffc07fe,
    0x2fff7bff, 0x03ff01ff, 0xf6fff2ff, 0x80ffc07f,
    0xf97ffa1f, 0xe07ff01f, 0xffaffeef, 0xf80ffe07,
    0xfed3ff93, 0xfe03ff01, 0xffb7ffd8, 0xff80ffc0};

#define R99_DL_OUTPUT_PKT_15_WRD_SIZE       32
#pragma DATA_SECTION (r99_dl_output_packet_15, ".testData");
static UInt32 r99_dl_output_packet_15[R99_DL_OUTPUT_PKT_15_WRD_SIZE] = {
    0x1ffd5ffe, 0x1ffc0ffc, 0x9ffe53ff, 0x07fe03ff,
    0x3bfff7ff, 0x01ff80ff, 0xc0fff13f, 0xc07ff03f,
    0xf95ffdbf, 0xf01ff80f, 0xfdcffe27, 0xfc07fe07,
    0xfff3ffcb, 0xff01ffc0, 0xfff17fec, 0xffc07fe0,
    0x7ff8dffa, 0x3ff01ff8, 0x9ffd6fff, 0x0ffc07fe,
    0xdfff4bff, 0x03ff01ff, 0x8fffcf7f, 0x80ffc07f,
    0xe37ff41f, 0xe07ff01f, 0xff8ffef7, 0xf80ffe07,
    0xff4fff97, 0xfe03ff01, 0xffc1ffec, 0xff80ffc0};

#define R99_UL_OUTPUT_PKT_1_WRD_SIZE  36

#pragma DATA_SECTION (r99_ul_output_packet_1, ".testData");
static UInt32 r99_ul_output_packet_1[R99_UL_OUTPUT_PKT_1_WRD_SIZE] = {
    0xe4ffcc78, 0xd1cede24, 0x122033e0, 0x27b4bf4f,
    0x3f7fa707, 0x465747d1, 0x0b0112fb, 0xbcd629e7,
    0x061034dc, 0xf5e11d60, 0x7ffd19f9, 0xb713d237,
    0xe0391fff, 0x11fa1a02, 0x9412270b, 0xe7d3e408,
    0x103e3ffc, 0x29c1e73d, 0x10e0b7b2, 0x00171deb,
    0xc72af802, 0xd41ebc10, 0xe7a0040f, 0x18d613df,
    0xeb14f712, 0x41022021, 0x0534e027, 0xfacdfd12,
    0x060be60c, 0x4af30ca9, 0xf850f2e3, 0x333bfbec,
    0x000000d2, 0x00000000, 0x00000000, 0x00000000};

/** ============================================================================
 *   @n@b read_dl_test_config
 *
 *   @b Description
 *   @n Utility function that reads the input test vector params from a given
 *      file.
 *
 *   @param[in]  
 *   @n fp              Test configuration file handle.
 * 
 *   @param[in]  
 *   @n pR99DlParams    Rel 99 DL Input parameters read for this test
 * 
 *   @return        
 *   @n None.
 * =============================================================================
 */
static Void read_dl_test_config(FILE* filePtr, BcpTest_R99DlParams* pR99DlParams)
{
	Char                    lineBuf[300];
	Int32                   i,ii,iii; 
	BcpTest_R99DlTrch*      pR99DlTrch;

	fgets(lineBuf,300,filePtr);//skip one line

	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[7],"%d",&i);
	pR99DlParams->ChType = i;

	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[9],"%d",&i);
	pR99DlParams->numPhyCh = i;

	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[11],"%d",&i);
	pR99DlParams->slotFormat = i;

	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[11],"%d",&i);
	pR99DlParams->dtxPosType = i;
		
	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[9],"%d",&i);
	pR99DlParams->usedTFCI = i;
		
	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[8],"%d",&i);
	pR99DlParams->numTrch = i;

	for ( ii = 0; ii < pR99DlParams->numTrch; ii++)
	{
		pR99DlTrch = &pR99DlParams->r99DlTrch[ii];

		fgets(lineBuf,300,filePtr);//skip one line

		fgets(lineBuf,300,filePtr);//numRadioFrmPerTti
		sscanf(&lineBuf[18],"%d",&i);
		pR99DlTrch->numRadioFrmPerTti = i;

		fgets(lineBuf,300,filePtr);//CRCsize
		sscanf(&lineBuf[8],"%d",&i);
		pR99DlTrch->CRCsize = i;

		fgets(lineBuf,300,filePtr);//turboFlag
		sscanf(&lineBuf[10],"%d",&i);
		pR99DlTrch->turboFlag = i;

		fgets(lineBuf,300,filePtr);//halfCodeRateFlag
		sscanf(&lineBuf[17],"%d",&i);
		pR99DlTrch->halfCodeRateFlag = i;
			
		fgets(lineBuf,300,filePtr);//RMattribute
		sscanf(&lineBuf[12],"%d",&i);
		pR99DlTrch->RMattribute = i;

		fgets(lineBuf,300,filePtr);//numTF
		sscanf(&lineBuf[6],"%d",&i);
		pR99DlTrch->numTF = i;
			
		for (iii = 0; iii < pR99DlTrch->numTF; iii++)
		{
			fgets(lineBuf,300,filePtr);//TBsize
			sscanf(&lineBuf[10],"%d",&i);
			pR99DlTrch->TF[iii].TBsize = i;

			fgets(lineBuf,300,filePtr);//numTB
			sscanf(&lineBuf[9],"%d",&i);
			pR99DlTrch->TF[iii].numTB= i;		
		}
	}//end of TrCH

	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[9],"%d",&i);
	pR99DlParams->TFCSsize = i;

	for ( ii = 0; ii < pR99DlParams->TFCSsize; ii++)
	{
		for (iii = 0; iii < pR99DlParams->numTrch; iii++)
		{
			fgets(lineBuf,300,filePtr);
			sscanf(&lineBuf[15],"%d",&i);
			pR99DlParams->TFCSTab[ii][iii] = i;
		}
	}

	for ( ii = 0; ii < pR99DlParams->numTrch; ii++)
	{
		pR99DlTrch = &pR99DlParams->r99DlTrch[ii];

		fgets(lineBuf,300,filePtr);
		sscanf(&lineBuf[12],"%d",&i);
		pR99DlTrch->deltaNiltti = i;
	}//end of TrCH

    return;
}

/** ============================================================================
 *   @n@b read_ul_test_config
 *
 *   @b Description
 *   @n Utility function that reads the input test vector params from a given
 *      file.
 *
 *   @param[in]  
 *   @n fp              Test configuration file handle.
 * 
 *   @param[in]  
 *   @n pR99UlParams    Rel 99 UL Input parameters read for this test
 * 
 *   @return        
 *   @n None.
 * =============================================================================
 */
static Void read_ul_test_config(FILE* filePtr, BcpTest_R99UlParams* pR99UlParams)
{
	Char                lineBuf[300];
	UInt32              i,ii,iii; 
	BcpTest_R99UlTrch*  pR99UlTrch;
	float               f;

	//fgets(lineBuf,300,filePtr);
	//sscanf(&lineBuf[9],"%d",&i);
	//pR99UlParams->usedTFCI = i;

	fgets(lineBuf,300,filePtr);//skip one line

	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[6],"%d",&i);
	pR99UlParams->partialSF = i;

	if (pR99UlParams->partialSF == 256) 
		pR99UlParams->maxSet0 = 1;
	else if (pR99UlParams->partialSF == 128)
		pR99UlParams->maxSet0 = 2;
	else if (pR99UlParams->partialSF == 64)
		pR99UlParams->maxSet0 = 3;
	else if (pR99UlParams->partialSF == 32)
		pR99UlParams->maxSet0 = 4;
	else if (pR99UlParams->partialSF == 16)
		pR99UlParams->maxSet0 = 5;
	else if (pR99UlParams->partialSF == 8)
		pR99UlParams->maxSet0 = 6;
	else if (pR99UlParams->partialSF == 4)
		pR99UlParams->maxSet0 = 7;
	else
    {
#ifdef BCP_TEST_DEBUG
		Bcp_osalLog("wrong minSF!\n");
#endif
        return;
    }
	
	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[3],"%f",&f);
	pR99UlParams->PL = f;

	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[5],"%d",&i);
	pR99UlParams->qFmt = i;
		
	fgets(lineBuf,300,filePtr);
	sscanf(&lineBuf[8],"%d",&i);
	pR99UlParams->numTrch = i;

	for ( ii = 0; ii < pR99UlParams->numTrch; ii++)
	{
		pR99UlTrch = &pR99UlParams->r99UlTrch[ii];

		fgets(lineBuf,300,filePtr);//skip one line

		fgets(lineBuf,300,filePtr);//numRadioFrmPerTti
		sscanf(&lineBuf[18],"%d",&i);
		pR99UlTrch->numRadioFrmPerTti = i;

		fgets(lineBuf,300,filePtr);//CRCsize
		sscanf(&lineBuf[8],"%d",&i);
		pR99UlTrch->CRCsize = i;

		fgets(lineBuf,300,filePtr);//turboFlag
		sscanf(&lineBuf[10],"%d",&i);
		pR99UlTrch->turboFlag = i;

		fgets(lineBuf,300,filePtr);//halfCodeRateFlag
		sscanf(&lineBuf[17],"%d",&i);
		pR99UlTrch->halfCodeRateFlag = i;
			
		fgets(lineBuf,300,filePtr);//RMattribute
		sscanf(&lineBuf[12],"%d",&i);
		pR99UlTrch->RMattribute = i;

		fgets(lineBuf,300,filePtr);//usedTF
		sscanf(&lineBuf[7],"%d",&i);
		pR99UlTrch->usedTF = i;

		fgets(lineBuf,300,filePtr);//numTF
		sscanf(&lineBuf[6],"%d",&i);
		pR99UlTrch->numTF = i;
			
		for (iii = 0; iii < pR99UlTrch->numTF; iii++)
		{
			fgets(lineBuf,300,filePtr);//TBsize
			sscanf(&lineBuf[10],"%d",&i);
			pR99UlTrch->TF[iii].TBsize = i;

			fgets(lineBuf,300,filePtr);//numTB
			sscanf(&lineBuf[9],"%d",&i);
			pR99UlTrch->TF[iii].numTB= i;		
		}

	}//end of TrCH

	fgets(lineBuf,300,filePtr);//radioFrmIdx
	sscanf(&lineBuf[12],"%d",&i);
	pR99UlParams->radioFrmIdx = i;

	fgets(lineBuf,300,filePtr);//Ntr
	sscanf(&lineBuf[4],"%d",&i);
	pR99UlParams->Ntr = i;

    return;
}

/** ============================================================================
 *   @n@b computeParams_r99Dl
 *
 *   @b Description
 *   @n Calculates rest of the test parameters based on test inputs
 *
 *   @param[out]  
 *   @n pR99DlParams    Rel 99 DL Input parameters read for this test
 * 
 *   @return        
 *   @n None.
 * =============================================================================
 */
Void computeParams_r99Dl (BcpTest_R99DlParams * pR99DlParams)
{
	BcpTest_R99DlTrch* pR99DlTrch;
	Int32  TBsize;  
	Int32  B;
	double temp_double;
	Int32  C;
	Int32  K;
	Int32  Z;
	UInt32 numTB;
	UInt32 turboFlag;
	UInt32 halfCodeRateFlag;
	UInt32 CRCsize;
	UInt32 trchIdx;
	UInt32 tfIdx;
	BcpTest_R99DlTF* pR99DlTF;
	UInt32 numRadioFrmPerTti;
	UInt32 maxNiltti;
	UInt32 Niltti;
	UInt32 Nil;
	UInt32 maxSum;
	UInt32 tfcIdx;
	UInt32 sum;
	Int32 temp_int32;
	UInt32 D;
	Int32  deltaN;
	UInt32 totalSum;
	UInt32 rmZ[FDD_R99_DL_MAX_NUM_TRCH+1];
	Int32 deltaNij;
	Int32 deltaNiltti;
	UInt32 Xi;
	UInt32 einit_sys;
	UInt32 eplus_sys;
	UInt32 eminus_sys;
	UInt32 einit_p1;
	UInt32 eplus_p1;
	Int32 eminus_p1;
	UInt32 einit_p2;
	UInt32 eplus_p2;
	Int32 eminus_p2;
	UInt32 einit;
	UInt32 eplus;
	Int32  eminus;
	Int32 a_p1;
	Int32 a_p2;
	Int32 deltaNi;
	Int32 a;
	Int32 deltaNistar;
	Int32 deltaNimax;
	UInt32 Nmax;
	Int32 deltaNib_p1;
	Int32 deltaNib_p2;

	float  temp_float;
	UInt32 dioLen;
	UInt32 dioAddr;

	UInt32 rfIdx;
	UInt32 maxNumBitsRmOut;
	UInt32 numBitsRmOut;
	UInt32 b;


	/** calculate Nitti for all the TrCh all the TF **/
	/* per TrCh */
	for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
	{
		pR99DlTrch        = &pR99DlParams->r99DlTrch[trchIdx];

		CRCsize           = pR99DlTrch->CRCsize;
		numRadioFrmPerTti = pR99DlTrch->numRadioFrmPerTti;
		turboFlag         = pR99DlTrch->turboFlag;
		halfCodeRateFlag  = pR99DlTrch->halfCodeRateFlag;

		if (turboFlag == 0)//CC
			Z = 504;
		else if (turboFlag == 1)//TC
			Z = 5114;
		else
        {
#ifdef BCP_TEST_DEBUG
			Bcp_osalLog("Worng codingScheme!\n");
#endif
            return;
        }

		/* per TF */
		for (tfIdx = 0; tfIdx < pR99DlTrch->numTF; tfIdx++)
		{
			pR99DlTF = &(pR99DlTrch->TF[tfIdx]);

			TBsize = pR99DlTF->TBsize;
			numTB  = pR99DlTF->numTB;

			B = (TBsize + CRCsize)*numTB;

			temp_double = (double)B/Z;
			C = (Int32)ceil(temp_double);

			if (B == 0)
			{
				K      = 0;
				Niltti = 0;
				Nil    = 0;
			}
			else
			{
				if ((B < 40)&&(turboFlag == 1))
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
				if (turboFlag == 0)//CC
				{
					if (halfCodeRateFlag == 1)//1/2CC
						Niltti = C*(2*K+16);
					else if (halfCodeRateFlag == 0)//1/3CC 
						Niltti = C*(3*K+24);
					else
                    {
#ifdef BCP_TEST_DEBUG
						Bcp_osalLog("Worng codingScheme!\n");
#endif
                        return;
                    }
				}
				else if (turboFlag == 1)//TC
					Niltti = C*(3*K+12);
				else
                {
#ifdef BCP_TEST_DEBUG
					Bcp_osalLog("Worng codingScheme!\n");
#endif
                    return;
                }

				Nil = FDD_R99_DL_MAX_RF*Niltti/numRadioFrmPerTti;
			}

			/* output TF parameters */
			pR99DlTF->numCodeBks    = C;
			pR99DlTF->codeBkSizeK   = K;
			pR99DlTF->numFillerBits = C*K-B;
			pR99DlTF->Niltti        = Niltti;
			pR99DlTF->Nil           = Nil;
		}/* end of per TF */
	}//end of TrCh

	/* flexible positions */
	if (pR99DlParams->dtxPosType == 0)//0:flexible, 1:fixed
	{	
		/** calculate max of sum of all TrCh RM*Ni **/
		maxSum = 0;
		for (tfcIdx = 0; tfcIdx < pR99DlParams->TFCSsize; tfcIdx++)
		{
			sum = 0;

			for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
			{
				pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];

				tfIdx      = pR99DlParams->TFCSTab[tfcIdx][trchIdx];

				sum += pR99DlTrch->RMattribute * pR99DlTrch->TF[tfIdx].Nil;
			}

			pR99DlParams->totalSumTFCSTab[tfcIdx] = sum;

			if (sum > maxSum) maxSum = sum;

		}//end of TFCS

		for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];

			/* per TF */
			for (tfIdx = 0; tfIdx < pR99DlTrch->numTF; tfIdx++)
			{
				pR99DlTF = &(pR99DlTrch->TF[tfIdx]);

				temp_int32 = (Int32)ceil((double)((double)pR99DlParams->Ndata*pR99DlTrch->RMattribute*pR99DlTF->Niltti)/((double)pR99DlTrch->numRadioFrmPerTti*maxSum/FDD_R99_DL_MAX_RF));
				pR99DlTF->deltaNiltti = pR99DlTrch->numRadioFrmPerTti * temp_int32 - pR99DlTF->Niltti;
			}
		}

		/* for all TFC in TFCS */
		for (tfcIdx = 0; tfcIdx < pR99DlParams->TFCSsize; tfcIdx++)
		{
			/* calculate the current TFC's D */
			D = 0;
			for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
			{
				pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];
				tfIdx = pR99DlParams->TFCSTab[tfcIdx][trchIdx];
				D += (pR99DlTrch->TF[tfIdx].Niltti + pR99DlTrch->TF[tfIdx].deltaNiltti)/pR99DlTrch->numRadioFrmPerTti;
			}

			if (D > pR99DlParams->Ndata)
			{
				totalSum = pR99DlParams->totalSumTFCSTab[tfcIdx];

				rmZ[0] = 0;

				/* for all TrCh */
				sum = 0;
				for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
				{
					pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];
					tfIdx = pR99DlParams->TFCSTab[tfcIdx][trchIdx];

					/* calculate deltaNij */
					sum += pR99DlTrch->RMattribute * pR99DlTrch->TF[tfIdx].Nil;
					rmZ[trchIdx+1] = (UInt32)floor((double)sum * pR99DlParams->Ndata/totalSum);   
					deltaNij = (Int32)FDD_R99_DL_MAX_RF*rmZ[trchIdx+1] - FDD_R99_DL_MAX_RF*rmZ[trchIdx] - pR99DlTrch->TF[tfIdx].Nil; 

					/* calculate deltaN */
					deltaN = pR99DlTrch->numRadioFrmPerTti * deltaNij/FDD_R99_DL_MAX_RF;
					
					if (pR99DlTrch->TF[tfIdx].deltaNiltti > deltaN)
						pR99DlTrch->TF[tfIdx].deltaNiltti = deltaN;
				}
			}/* end of if (D > pR99DlParams->Ndata)*/
		}/* end of for all TFC in TFCS */

		/** ouput TRCH parameters **/
		for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];
			
			pR99DlTF = &(pR99DlTrch->TF[pR99DlTrch->TFI]);

			pR99DlTrch->TBsize        = pR99DlTF->TBsize;
			pR99DlTrch->numTB         = pR99DlTF->numTB;
			pR99DlTrch->numCodeBks    = pR99DlTF->numCodeBks;
			pR99DlTrch->codeBkSizeK   = pR99DlTF->codeBkSizeK;
			pR99DlTrch->numFillerBits = pR99DlTF->numFillerBits;
			pR99DlTrch->Niltti        = pR99DlTF->Niltti;
			//pR99DlTrch->deltaNiltti   = pR99DlTF->deltaNiltti;//read from XML file
		}

        /** calculate einit, eplus and eminus **/
		for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];
			//tfIdx      = pR99DlTrch->TFI;

			deltaNiltti = pR99DlTrch->deltaNiltti;
			Niltti      = pR99DlTrch->Niltti;

			if (deltaNiltti == 0)
			{
				pR99DlTrch->numBitsRmOut = Niltti;
				pR99DlTrch->punctureFlag = 0;
				pR99DlTrch->turboFlagBcpConfig = pR99DlTrch->turboFlag;

				/* transporatation */
				if(pR99DlTrch->turboFlag == 1)
				{
					Xi         = Niltti/3;

					pR99DlTrch->tc_einit_sys      = Xi;
					pR99DlTrch->tc_eplus_sys      = Xi;
					pR99DlTrch->tc_eminus_sys      = 0;

					pR99DlTrch->tc_einit_p1      = Xi;
					pR99DlTrch->tc_eplus_p1      = 2*Xi;
					pR99DlTrch->tc_eminus_p1      = 0;

					pR99DlTrch->tc_einit_p2      = Xi;
					pR99DlTrch->tc_eplus_p2      = Xi;
					pR99DlTrch->tc_eminus_p2      = 0;			
				}
				else
				{
					Xi                       = Niltti;

					pR99DlTrch->cc_einit     = 1;
					pR99DlTrch->cc_eplus     = 2*Niltti;
					pR99DlTrch->cc_eminus    = 0;
				}
			}
			else //	if (deltaNiltti != 0)
			{
				pR99DlTrch->numBitsRmOut              = Niltti + deltaNiltti;

				if((pR99DlTrch->turboFlag == 1)&&(deltaNiltti < 0))//turbo puncture
				{
				
					Xi         = Niltti/3;

					/* sys */
					/* always transportaion */
					/* As long as e_init>0, e_minus=0; e_plus=any_value; this is equivalent to transparent (no puncture/no repeat) operation */
					/* set the value based on 25.212 Sec4.2.7.2.1.4 */
					pR99DlTrch->tc_einit_sys  = Xi;//any value greater than 0
					pR99DlTrch->tc_eplus_sys  = Xi;//it is not be used actually
					pR99DlTrch->tc_eminus_sys = 0;//must be 0
		
					for (b = 2; b <= 3; b++)
					{
						if (b == 2)
						{
							a = 2;
							deltaNi = (Int32)floor((float)deltaNiltti/2);
						}
						else
						{
							a = 1;
							deltaNi = (Int32)ceil((float)deltaNiltti/2);
						}

						einit  = Xi;
						eplus  = a*Xi;
						eminus = a*abs(deltaNi);

						if (b == 2)
						{
							pR99DlTrch->tc_einit_p1      = einit;
							pR99DlTrch->tc_eplus_p1      = eplus;
							pR99DlTrch->tc_eminus_p1     = eminus;
						}
						else
						{
							pR99DlTrch->tc_einit_p2      = einit;
							pR99DlTrch->tc_eplus_p2      = eplus;
							pR99DlTrch->tc_eminus_p2     = eminus;
						}
					}
				
					pR99DlTrch->punctureFlag = 1;
					pR99DlTrch->turboFlagBcpConfig = 1;
				}
				else//TC 25.212 sec 4.2.7.2.1.4
				{
					deltaNi = deltaNiltti;

					a = 2;

					Xi = Niltti;

					einit = 1;
					eplus = a*Niltti;
				
					if (deltaNi<0)
						eminus = a*(-1)*deltaNi;
					else
						eminus = a*deltaNi;

					pR99DlTrch->cc_einit     = einit;
					pR99DlTrch->cc_eplus     = eplus;
					pR99DlTrch->cc_eminus    = eminus;
				

					if (pR99DlTrch->turboFlag == 1)
					{
						pR99DlTrch->punctureFlag = 0; 
					}
					else
					{
						if (deltaNi < 0) 
							pR99DlTrch->punctureFlag = 1;
						else
							pR99DlTrch->punctureFlag = 0; 
					}

					pR99DlTrch->turboFlagBcpConfig = 0;
				}
			}//	end of if (deltaNiltti != 0)
			
			pR99DlTrch->numBits1DTXOut            = pR99DlTrch->numBitsRmOut;//flexible pos no 1st DTX
			pR99DlTrch->numBits1IntOutPerRadioFrm = pR99DlTrch->numBits1DTXOut/pR99DlTrch->numRadioFrmPerTti;
		
		}//end of Trch
	}//end of flexible positions
	else//fixed positions
	{
		/** ouput TRCH parameters **/
		for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];
			
			pR99DlTF = &(pR99DlTrch->TF[pR99DlTrch->TFI]);

			pR99DlTrch->TBsize        = pR99DlTF->TBsize;
			pR99DlTrch->numTB         = pR99DlTF->numTB;
			pR99DlTrch->numCodeBks    = pR99DlTF->numCodeBks;
			pR99DlTrch->codeBkSizeK   = pR99DlTF->codeBkSizeK;
			pR99DlTrch->numFillerBits = pR99DlTF->numFillerBits;
			pR99DlTrch->Niltti        = pR99DlTF->Niltti;
		}

		/* calculate Ni* for all TrCH */
		for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];

			maxNiltti = 0;

			for (tfcIdx = 0; tfcIdx < pR99DlParams->TFCSsize; tfcIdx++)
			{
				tfIdx  = pR99DlParams->TFCSTab[tfcIdx][trchIdx];

				if (pR99DlTrch->TF[tfIdx].Niltti > maxNiltti)
					maxNiltti = pR99DlTrch->TF[tfIdx].Niltti;
			}//end of TFC

			pR99DlTrch->Nistar = FDD_R99_DL_MAX_RF*maxNiltti/pR99DlTrch->numRadioFrmPerTti;

		}//end of Trch

		/* calculate deltaNistar for all TrCH */
		totalSum = 0;
		for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];
			totalSum  += pR99DlTrch->RMattribute * pR99DlTrch->Nistar;
		}

		rmZ[0] = 0;
		sum = 0;
		/* for all TrCh */
		for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];

			/* calculate deltaNi* */
			sum += pR99DlTrch->RMattribute * pR99DlTrch->Nistar;
			rmZ[trchIdx+1] = (UInt32)floor((double)sum * pR99DlParams->Ndata/totalSum);   
			deltaNistar = (Int32)FDD_R99_DL_MAX_RF*rmZ[trchIdx+1] - FDD_R99_DL_MAX_RF*rmZ[trchIdx] - pR99DlTrch->Nistar; 

			/* calculate deltaNimax */
			deltaNimax = (Int32)pR99DlTrch->numRadioFrmPerTti * deltaNistar/FDD_R99_DL_MAX_RF;

			//pR99DlTrch->deltaNistar = deltaNistar/MAX_F;
			pR99DlTrch->deltaNimax  = deltaNimax;
		}

		/* calculate einit, eplus, eminus */
		for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];
			deltaNimax    = pR99DlTrch->deltaNimax;
			Niltti        = pR99DlTrch->Niltti;

			if (deltaNimax == 0)
			{ 
				//for (tfIdx = 0; tfIdx < pR99DlTrch->numTF; tfIdx++)
				//	pR99DlTrch->TF[tfIdx].deltaNiltti = 0;

				//tfIdx = pR99DlTrch->TFI;
				//deltaNiltti = pR99DlTrch->TF[tfIdx].deltaNiltti;
				deltaNiltti = 0;

				pR99DlTrch->numBitsRmOut = Niltti;
				pR99DlTrch->punctureFlag = 0;
				pR99DlTrch->turboFlagBcpConfig = pR99DlTrch->turboFlag;

				//pR99DlTrch->numBitsRmOut = Niltti;
				//pR99DlTrch->punctureFlag = 0;

				/* transporatation */
				if(pR99DlTrch->turboFlag == 1)
				{
					Xi         = Niltti/3;

					pR99DlTrch->tc_einit_sys      = Xi;
					pR99DlTrch->tc_eplus_sys      = Xi;
					pR99DlTrch->tc_eminus_sys      = 0;

					pR99DlTrch->tc_einit_p1      = Xi;
					pR99DlTrch->tc_eplus_p1      = 2*Xi;
					pR99DlTrch->tc_eminus_p1      = 0;

					pR99DlTrch->tc_einit_p2      = Xi;
					pR99DlTrch->tc_eplus_p2      = Xi;
					pR99DlTrch->tc_eminus_p2      = 0;			
				}
				else
				{
					Xi         = Niltti;
					pR99DlTrch->cc_einit     = 1;
					pR99DlTrch->cc_eplus     = 2*Niltti;
					pR99DlTrch->cc_eminus    = 0;
				}

				/* find the max Niltti it is the 1DTX output bits */
				maxNumBitsRmOut = 0;
				for (tfIdx = 0; tfIdx < pR99DlTrch->numTF; tfIdx++)
				{
					numBitsRmOut = pR99DlTrch->TF[tfIdx].Niltti;
					if (numBitsRmOut > maxNumBitsRmOut)
						maxNumBitsRmOut = numBitsRmOut;
				}
				pR99DlTrch->numBits1DTXOut = maxNumBitsRmOut;
			}
			else//deltaNimax != 0
			{
				Nmax = 0;
				for (tfIdx = 0; tfIdx < pR99DlTrch->numTF; tfIdx++)
				{
					if (pR99DlTrch->TF[tfIdx].Niltti > Nmax)
						Nmax = pR99DlTrch->TF[tfIdx].Niltti;
				}

				if ((pR99DlTrch->turboFlag == 1)&&(deltaNimax < 0))//turbo puncture
				{
					pR99DlTrch->turboFlagBcpConfig = 1;
					pR99DlTrch->punctureFlag = 1;

					Xi   = Niltti/3;
					Nmax = Nmax/3;

					/* sys */
					/* always transportaion */
					/* As long as e_init>0, e_minus=0; e_plus=any_value; this is equivalent to transparent (no puncture/no repeat) operation */
					/* set the value based on 25.212 Sec4.2.7.2.1.4 */
					einit_sys  = Xi;//any value greater than 0
					eplus_sys  = Xi;//it is not be used actually
					eminus_sys = 0;//must be 0
					
					/* par1 */
					a_p1        = 2;
					deltaNib_p1 = floor(deltaNimax/2);

					einit_p1    = Nmax;
					eplus_p1    = a_p1*Nmax;
					if (deltaNib_p1<0)
						eminus_p1 = a_p1*(-1)*deltaNib_p1;
					else	
						eminus_p1 = a_p1*deltaNib_p1;
					
					/* par2 */
					a_p2        = 1;
					deltaNib_p2 = ceil(deltaNimax/2);

					einit_p2    = Nmax;
					eplus_p2    = a_p2*Nmax;
					if (deltaNib_p2<0)
						eminus_p2 = a_p2*(-1)*deltaNib_p2;
					else
						eminus_p2 = a_p2*deltaNib_p2;

					pR99DlTrch->tc_einit_sys      = einit_sys;
					pR99DlTrch->tc_eplus_sys      = eplus_sys;
					pR99DlTrch->tc_eminus_sys      = eminus_sys;
					pR99DlTrch->tc_einit_p1      = einit_p1;
					pR99DlTrch->tc_eplus_p1      = eplus_p1;
					pR99DlTrch->tc_eminus_p1      = eminus_p1;
					pR99DlTrch->tc_einit_p2      = einit_p2;
					pR99DlTrch->tc_eplus_p2      = eplus_p2;
					pR99DlTrch->tc_eminus_p2      = eminus_p2;

					/* calculate numBits1DTXOut */
					/* calculate deltaNiltti for all TF */
					maxNumBitsRmOut = 0;
					for (tfIdx = 0; tfIdx < pR99DlTrch->numTF; tfIdx++)
					{
						Niltti = pR99DlTrch->TF[tfIdx].Niltti;
						Xi     = Niltti/3;
					
						deltaNiltti = (Int32)floor((float)(deltaNi)*(deltaNi)*Xi/Nmax+0.5)-(Int32)floor((float)abs(deltaNi*deltaNi*deltaNi)*Xi/Nmax);

						pR99DlTrch->TF[tfIdx].deltaNiltti = deltaNiltti;
						numBitsRmOut = Niltti+deltaNiltti;
						if (numBitsRmOut > maxNumBitsRmOut)
							maxNumBitsRmOut = numBitsRmOut;
					}
					pR99DlTrch->numBits1DTXOut = maxNumBitsRmOut;

					/* calculate number of RM output bits */
					tfIdx = pR99DlTrch->TFI;
					pR99DlTrch->numBitsRmOut = pR99DlTrch->TF[tfIdx].Niltti + pR99DlTrch->TF[tfIdx].deltaNiltti;
					pR99DlTrch->deltaNiltti  = pR99DlTrch->TF[tfIdx].deltaNiltti;
				}
				else//CC and TC repeat
				{
					/* calculate einit, eplus and eminus for used TF */
					deltaNi = deltaNimax;
					a = 2;
					einit  = 1;
					eplus  = a*Nmax;
					eminus = a*abs(deltaNi);

					pR99DlTrch->cc_einit     = einit;
					pR99DlTrch->cc_eplus     = eplus;
					pR99DlTrch->cc_eminus    = eminus;

					/* calculate deltaNiltti for all TF */
					maxNumBitsRmOut = 0;
					for (tfIdx = 0; tfIdx < pR99DlTrch->numTF; tfIdx++)
					{
						Niltti = pR99DlTrch->TF[tfIdx].Niltti;
						Xi     = Niltti;
					
						if (deltaNi >= 0)
							deltaNiltti = (Int32)ceil((float)abs(deltaNi)*Xi/Nmax);
						else
							deltaNiltti = (-1)*(Int32)ceil((float)abs(deltaNi)*Xi/Nmax);

						pR99DlTrch->TF[tfIdx].deltaNiltti = deltaNiltti;
						numBitsRmOut = Niltti+deltaNiltti;
						if (numBitsRmOut > maxNumBitsRmOut)
							maxNumBitsRmOut = numBitsRmOut;
					}
					pR99DlTrch->numBits1DTXOut = maxNumBitsRmOut;

					/* calculate number of RM output bits */
					tfIdx = pR99DlTrch->TFI;
					pR99DlTrch->numBitsRmOut = pR99DlTrch->TF[tfIdx].Niltti + pR99DlTrch->TF[tfIdx].deltaNiltti;
					pR99DlTrch->deltaNiltti  = pR99DlTrch->TF[tfIdx].deltaNiltti;

					if (pR99DlTrch->turboFlag == 1)
					{
						pR99DlTrch->punctureFlag = 0; 
					}
					else
					{
						if (deltaNi < 0) 
							pR99DlTrch->punctureFlag = 1;
						else
							pR99DlTrch->punctureFlag = 0; 
					}

					pR99DlTrch->turboFlagBcpConfig = 0;
				}
			}

			pR99DlTrch->numBits1IntOutPerRadioFrm = pR99DlTrch->numBits1DTXOut/pR99DlTrch->numRadioFrmPerTti;
		}//end of Trch

	}//end of fixed position

	pR99DlParams->numBitsTrchConcatOut = 0;
	for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++) 
		pR99DlParams->numBitsTrchConcatOut += pR99DlParams->r99DlTrch[trchIdx].numBits1IntOutPerRadioFrm;
	
	dioAddr = 0x0C000000;
	for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++) 
	{
		pR99DlTrch = &pR99DlParams->r99DlTrch[trchIdx];

		dioLen  = pR99DlTrch->numBits1IntOutPerRadioFrm*2;//number of real bits in one RF * 2bit/real bit
		temp_float = (float)dioLen/128;//1 INT output each RF on 128 bit bound
		dioLen  = (UInt32)ceil(temp_float);//number of 128-bit words in one RF
		pR99DlTrch->dioNumBytesPerRadioFrm = dioLen*16;//number of bytes
		pR99DlTrch->dioNumBytesPerTti = pR99DlTrch->dioNumBytesPerRadioFrm * pR99DlTrch->numRadioFrmPerTti;

		//pR99DlParams->dioCountPerRadioFrm[trchIdx] = pR99DlTrch->dioCountPerRadioFrm;

		//pR99DlParams->trchConcaNumBitsPerRadioFrm[trchIdx] = pR99DlTrch->num1IntOutBitsPerRadioFrm;

		for (rfIdx = 0; rfIdx < FDD_R99_DL_NUM_RF_TO_RUN; rfIdx++)
		{
			pR99DlTrch->dioAddrPerRadioFrm[rfIdx] = dioAddr;
			dioAddr += pR99DlTrch->dioNumBytesPerRadioFrm;

			//pR99DlParams->dioAddrPerRadioFrm[rfIdx][trchIdx] = pR99DlTrch->dioAddrPerRadioFrm[rfIdx];
		}
	}
}

UInt32 computeGCD (UInt32 a, UInt32 b)
{
	UInt32 c;

	if (a < b)
	{
		c = a;            
        a = b;
        b = c;
	}

	c = a%b;

	if (c == 0)
	{
	}
	else
	{
		a = b;
        b = c; 
	}

	return b;
}


Void computeParams_r99ul(BcpTest_R99UlParams* pR99UlParams)
{
	Int32  TBsize; 
	Int32  maxSet0; 
	Int32  nPhyCh; 
	Int32  B;
	double temp_double;
	double PL;
	Int32  SF;
	Int32  C;
	Int32  K;
	Int32  Ne;
	Int32  Ndata;
	Int32  Ndataj;
	Int32  eplus;
	Int32  eminus;
	Int32  einit;
	Int32  SET1_index[11];
	Int32  SET2_index[11];
	Int32  i, j;
	Int32  num_ele_SET1;
	Int32  num_ele_SET2;
	UInt32 trchIdx;
	BcpTest_R99UlTrch* pR99UlTrch;
	Int32  CRCsize;
	Int32  numRadioFrmPerTti;
	UInt32 turboFlag;
	UInt32 halfCodeRateFlag;
	UInt32 Z;
	UInt32 tfIdx;
	BcpTest_R99UlTF* pR99UlTF;
	UInt32 numTB;
	UInt32 Nil;
	UInt32 sum;
	UInt32 minRM;
	UInt32 rmZ[FDD_R99_UL_MAX_NUM_TRCH+1];
	Int32 deltaNij;
	Int32 Nij;
	Int32 Fi;
	Int32 R;
	Int32 q;
	float qPrime;
	Int32 qAbs;
	Int32 gcd;
	Int32 x;
	Int32 temp, temp1;
	Int32 sIdx;
	Int32 sValue;
	Int32 S[8];
	const Int32* P; 
	Int32 a;
	Int32 Xi;
	Int32 deltaNi;
	Int32 b;
	Int32 r;
	UInt32 Ntr;

	
	/*          maxSet0(1 to 12)        1    2    3     4     5     6     7       8       9      10      11      12*/ 	
	const Int32 tab_Ndata    [12] = { 150, 300, 600, 1200, 2400, 4800, 9600, 9600*2, 9600*3, 9600*4, 9600*5, 9600*6};
	const Int32 tab_SF       [12] = { 256, 128,  64,   32,   16,    8,    4,      4,      4,      4,      4,      4};
	const Int32 tab_Num_PhyCh[12] = {   1,   1,   1,    1,    1,    1,    1,      2,      3,      4,      5,      6};

	const Int32 P1[1] = {0};
	const Int32 P2[2] = {0,1};
	const Int32 P4[4] = {0,2,1,3};
	const Int32 P8[8] = {0,4,2,6,1,5,3,7};

	maxSet0 =         pR99UlParams->maxSet0;
	PL      = (double)pR99UlParams->PL;
	Ntr     =         pR99UlParams->Ntr;

	/** calculate Nij for all the TrCh **/
	for (trchIdx = 0; trchIdx < pR99UlParams->numTrch; trchIdx++)
	{
		pR99UlTrch        = &pR99UlParams->r99UlTrch[trchIdx];

		CRCsize           = pR99UlTrch->CRCsize;
		numRadioFrmPerTti = pR99UlTrch->numRadioFrmPerTti;
		turboFlag         = pR99UlTrch->turboFlag;
		halfCodeRateFlag  = pR99UlTrch->halfCodeRateFlag;

		if (turboFlag == 0)//CC
			Z = 504;
		else if (turboFlag == 1)//TC
			Z = 5114;
		else
        {
#ifdef BCP_TEST_DEBUG
			Bcp_osalLog("Wrong codingScheme!\n");
#endif
            return;
        }

		tfIdx = pR99UlTrch->usedTF;
		pR99UlTF = &(pR99UlTrch->TF[tfIdx]);

		TBsize = pR99UlTF->TBsize;
		numTB  = pR99UlTF->numTB;

		B = (TBsize + CRCsize)*numTB;

		temp_double = (double)B/Z;
		C = (Int32)ceil(temp_double);

		if (B == 0)
		{
			K      = 0;
			Nil    = 0;
		}
		else
		{
			if ((B < 40)&&(turboFlag == 1))
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
			if (turboFlag == 0)//CC
			{
				if (halfCodeRateFlag == 1)//1/2CC
					Ne = C*(2*K+16);
				else if (halfCodeRateFlag == 0)//1/3CC 
					Ne = C*(3*K+24);
				else
                {
#ifdef BCP_TEST_DEBUG
					Bcp_osalLog("Worng codingScheme!\n");
#endif
                    return;
                }
			}
			else if (turboFlag == 1)//TC
				Ne = C*(3*K+12);
			else
            {
#ifdef BCP_TEST_DEBUG
				Bcp_osalLog("Worng codingScheme!\n");
#endif
                return;
            }

			/* radio frame equalisation */
			temp_double = (double)Ne/numRadioFrmPerTti;
			Nil = (UInt32)ceil(temp_double);
		}

		pR99UlTrch->TBsize        = TBsize;
		pR99UlTrch->numTB         = numTB;
		pR99UlTrch->numCodeBks    = C;
		pR99UlTrch->codeBkSizeK   = K;
		pR99UlTrch->numFillerBits = C*K-B;
		pR99UlTrch->Nij           = Nil;
	}//end of TrCh

	/** calculate the sum of all TrCh RM*Ni for the used TFC only **/
	sum = 0;
	for (trchIdx = 0; trchIdx < pR99UlParams->numTrch; trchIdx++)
	{
		pR99UlTrch = &pR99UlParams->r99UlTrch[trchIdx];

		sum += pR99UlTrch->RMattribute * pR99UlTrch->Nij;
	}
	pR99UlParams->totalSumRMxNmj = sum;

	/* initlization to zore */
	for (i = 0; i < 12; i++)
	{
		SET1_index[i] = 0;
		SET2_index[i] = 0;
	}

	minRM = 0xffffff;
	for (trchIdx = 0; trchIdx < pR99UlParams->numTrch; trchIdx++)
	{
		pR99UlTrch = &pR99UlParams->r99UlTrch[trchIdx];

		if (pR99UlTrch->RMattribute < minRM )
			minRM = pR99UlTrch->RMattribute;
	}

	/* get SET1 */
	j = 0;
	for (i = 0; i < maxSet0; i++)
	{
		if ((minRM * tab_Ndata[i] / 15 * Ntr) >= pR99UlParams->totalSumRMxNmj)
		{
			SET1_index[j] = i;
			j = j+1;
		}
	}
	num_ele_SET1 = j;

	/* if SET1 is not empty and the smallest element of SET1 requires just one E-DPDCH */
	if ((num_ele_SET1 > 0) && (tab_Num_PhyCh[SET1_index[0]]==1)) 
	{
		    Ndataj = tab_Ndata[SET1_index[0]]/ 15 * Ntr;
	}
	else /* else of SET1 is not empty and the smallest element of SET1 requires just one E-DPDCH */
	{
		/* get SET2 */
		j = 0;
		for (i = 0; i < maxSet0; i++)
		{
			if((double)(minRM *tab_Ndata[i]/ 15 * Ntr) >= (double)PL*pR99UlParams->totalSumRMxNmj)
			{
				SET2_index[j] = i;
				j= j+1;
			}
		}
		num_ele_SET2 = j;
    
		i = 0;
		Ndata = tab_Ndata[SET2_index[i]]/ 15 * Ntr; 
		while ((Ndata<(tab_Ndata[SET2_index[num_ele_SET2-1]]/ 15 * Ntr))&&(tab_Num_PhyCh[SET2_index[i+1]]==tab_Num_PhyCh[SET2_index[i]]))
		{
			i = i+1;
			Ndata = tab_Ndata[SET2_index[i]]/ 15 * Ntr; 
		}

		Ndataj = Ndata;

	}/* end if SET1 is not empty and the smallest element of SET1 requires just one E-DPDCH */

	for(i = 0; i < maxSet0; i++)
	{
		if (Ndataj == tab_Ndata[i]/ 15 * Ntr)
		{
			nPhyCh    = tab_Num_PhyCh[i];
			SF        = tab_SF[i];  
		}
	}

	pR99UlParams->SF     = SF;
	pR99UlParams->nPhyCh = nPhyCh;
	pR99UlParams->Ndataj = Ndataj;//in a radio frame for all physical channels
	pR99UlParams->SFradio= SF/pR99UlParams->partialSF;

	/** calculate deltaNij, only calculate for the used TFCI */
	rmZ[0] = 0;
	sum = 0;
	/* calculate deltaNij for all TrCh */
	for (trchIdx = 0; trchIdx < pR99UlParams->numTrch; trchIdx++)
	{
		pR99UlTrch = &pR99UlParams->r99UlTrch[trchIdx];

		/* calculate deltaNi* */
		sum += pR99UlTrch->RMattribute * pR99UlTrch->Nij;
		rmZ[trchIdx+1] = (UInt32)floor((double)sum * pR99UlParams->Ndataj/pR99UlParams->totalSumRMxNmj);   
		deltaNij = (Int32)rmZ[trchIdx+1] - rmZ[trchIdx] - pR99UlTrch->Nij;

		pR99UlTrch->deltaNij = deltaNij ; 
	}

	/** calculate einit, eplus and eminus for all Trch **/
	for (trchIdx = 0; trchIdx < pR99UlParams->numTrch; trchIdx++)
	{
		pR99UlTrch = &pR99UlParams->r99UlTrch[trchIdx];

		deltaNij = pR99UlTrch->deltaNij;
		Nij      = pR99UlTrch->Nij; 
		Fi       = pR99UlTrch->numRadioFrmPerTti;
		turboFlag= pR99UlTrch->turboFlag;

		if (Fi == 1)
			P = P1;
		else if (Fi == 2)
			P = P2;
		else if (Fi == 4)
			P = P4;
		else
			P = P8;

		if (deltaNij == 0)
		{	
			/* transparent */
			pR99UlTrch->punctureFlag = 0;

			if(turboFlag == 1)
			{
				Xi         = Nij/3;

				pR99UlTrch->tc_einit_sys      = Xi;
				pR99UlTrch->tc_eplus_sys      = Xi;
				pR99UlTrch->tc_eminus_sys      = 0;

				pR99UlTrch->tc_einit_p1      = Xi;
				pR99UlTrch->tc_eplus_p1      = 2*Xi;
				pR99UlTrch->tc_eminus_p1      = 0;

				pR99UlTrch->tc_einit_p2      = Xi;
				pR99UlTrch->tc_eplus_p2      = Xi;
				pR99UlTrch->tc_eminus_p2      = 0;			
			}
			else
			{
				Xi         = Nij;
				pR99UlTrch->cc_einit     = 1;
				pR99UlTrch->cc_eplus     = 2*Xi;
				pR99UlTrch->cc_eminus    = 0;
			}
		}
		else
		{
			if((turboFlag == 1)&&(deltaNij < 0))
			{
				pR99UlTrch->punctureFlag = 1;
				pR99UlTrch->turboFlagBcpConfig = 1;

				Xi         = (Int32)floor((double)Nij/3);
		
				/* sys */
				/* always transportaion */
				/* As long as e_init>0, e_minus=0; e_plus=any_value; this is equivalent to transparent (no puncture/no repeat) operation */
				/* set the value based on 25.212 Sec4.2.7.2.1.4 */
				pR99UlTrch->tc_einit_sys  = Xi;//any value greater than 0
				pR99UlTrch->tc_eplus_sys  = Xi;//it is not be used actually
				pR99UlTrch->tc_eminus_sys = 0;//must be 0

				/* parity1 and parity2 */
				for (b = 2; b <= 3; b++ )
				{
					if (b == 2) 
					{
						a = 2;
						deltaNi = (Int32)floor((double)deltaNij/2);
					}
					else
					{
						a = 1;
						deltaNi = (Int32)ceil((double)deltaNij/2);
					}
					
					temp = (Int32)abs(deltaNi);
					q = (Int32)floor((double)Xi/temp);

					if ( q <= 2)
					{
						for (r = 0; r <= Fi-1; r++)
						{
							sIdx    = (3*r+b-1)%Fi;
							sValue  = r%2;
							S[sIdx] = sValue;
						}
					}
					else
					{
						if (q%2 == 0)//q is even
						{
							/* greatest common divisor */
							gcd = computeGCD (q, Fi);
						
					        qPrime = (float)(q - (float)gcd/Fi);
						}
				        else//q is odd
						{
					        qPrime = q;
						}

						for (x = 0; x <= Fi-1; x++)
						{
							
							//temp = x*qPrime;
							temp1 = (Int32)ceil((float)x*qPrime);
							r = (Int32)temp1%Fi;
							
							if (Fi == 1)
								sValue = temp1;
							else if (Fi == 2)
								sValue = temp1>>1;
							else if (Fi == 4)
								sValue = temp1>>2;
							else
								sValue = temp1>>3;

							temp = (Int32)3*r+b-1;
							sIdx = (Int32)temp%Fi;
							
							S[sIdx] = sValue;
						}
					}
					temp = a*S[P[pR99UlParams->radioFrmIdx%Fi]]*abs(deltaNi)+Xi;
					einit = (temp)%(a*Xi);
					if (einit == 0) 
						einit = a*Xi;

					eplus = a*Xi;

					eminus = a*abs(deltaNi);

					if (b == 2) 
					{
						pR99UlTrch->tc_einit_p1  = einit;
						pR99UlTrch->tc_eminus_p1 = eminus;
						pR99UlTrch->tc_eplus_p1  = eplus;
					}
					else
					{
						pR99UlTrch->tc_einit_p2  = einit;
						pR99UlTrch->tc_eminus_p2 = eminus;
						pR99UlTrch->tc_eplus_p2  = eplus;
					}
				}
			}
			else
			{
				pR99UlTrch->turboFlagBcpConfig = 0;

				if (deltaNij)
				{
					R = deltaNij%Nij;
				}
				else
				{
					R = deltaNij;
					while (R < 0)
						R += Nij;
				}

				if ((R != 0)&&(2*R <= Nij))
				{
					q = (Int32)ceil((double)Nij/R);
				}
				else
				{
					q = (Int32) ceil((double)Nij/(R-Nij));
				}

				if (q%2 == 0)//q is even
				{
					qAbs = abs(q);

					/* greatest common divisor */
					gcd = computeGCD (qAbs, Fi);
				
					qPrime = (float)(q + (float)gcd/Fi);
				}
				else//q is odd
				{
					qPrime = q;
				}

				for (x = 0; x <= Fi-1; x++)
				{
					temp = (Int32)floor(x*qPrime);
					temp1 = abs(temp);	
					
					if (Fi == 1)
						sValue = temp1;
					else if (Fi == 2)
						sValue = temp1>>1;
					else if (Fi == 4)
						sValue = temp1>>2;
					else
						sValue = temp1>>3;

					sIdx = temp1%Fi;

					S[sIdx] = sValue;
				}

				deltaNi = deltaNij;
				a  = 2;
				Xi = Nij;

				temp1 = abs(deltaNi);
				temp = a*S[P[pR99UlParams->radioFrmIdx%Fi]]*temp1+1;
				einit = (temp)%(a*Nij);
				eplus = a*Nij;
				eminus = a*temp1;

				pR99UlTrch->cc_einit  = einit;
				pR99UlTrch->cc_eminus = eminus;
				pR99UlTrch->cc_eplus  = eplus;
				pR99UlTrch->Xi        = Xi;

				if (turboFlag == 0)
				{
					if (deltaNij < 0)
						pR99UlTrch->punctureFlag = 1;
					else
						pR99UlTrch->punctureFlag = 0;
				}
				else
				{
					pR99UlTrch->punctureFlag = 0;
				}
			}
		}//end of pR99UlTrch->deltaNij != 0
	}//end of TrCH
}

/** ============================================================================
 *   @n@b prepare_wcdma_enchdr_cfg
 *
 *   @b Description
 *   @n Sets up the Encoder engine header for the test. 
 *
 *   @param[out]  
 *   @n pEncHdrCfg      Encoder Header configuration thus populated for the test.
 * 
 *   @param[in]  
 *   @n pR99DlParams    R99 DL parameters calculated for this test.
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
static Void prepare_wcdma_enchdr_cfg
(
    Bcp_EncHdrCfg*              pEncHdrCfg, 
    UInt32                      codeBkSizeK,
	UInt32                      numCodeBks,
    UInt32                      turboFlag,
    UInt32                      codeRateFlag
)
{
    UInt32                      i;        

    memset (pEncHdrCfg, 0, sizeof (Bcp_EncHdrCfg));

    /* Setup the Encoder header as per inputs passed. */        
    pEncHdrCfg->local_hdr_len                   =   1;    
    pEncHdrCfg->turbo_conv_sel                  =   turboFlag;

    if (numCodeBks)
    {
        pEncHdrCfg->blockCfg[0].block_size      =   codeBkSizeK;
        pEncHdrCfg->blockCfg[0].num_code_blks   =   numCodeBks;
    }
    else
    {
        pEncHdrCfg->blockCfg[0].block_size      =   0;
        pEncHdrCfg->blockCfg[0].num_code_blks   =   0;
    }
    pEncHdrCfg->blockCfg[1].block_size          =   0;
    pEncHdrCfg->blockCfg[1].num_code_blks       =   0;

    pEncHdrCfg->blockCfg[2].block_size          =   0;
    pEncHdrCfg->blockCfg[2].num_code_blks       =   0;

    pEncHdrCfg->scr_crc_en                      =   0;  //Dont add CRC

    if (turboFlag == 1)
        pEncHdrCfg->code_rate_flag              =   0;
    else
        pEncHdrCfg->code_rate_flag              =   codeRateFlag;

    for (i = 0; i < 3; i ++)
    {
        pEncHdrCfg->blockCfg[i].intvpar0        =   0;
        pEncHdrCfg->blockCfg[i].intvpar1        =   0;
        pEncHdrCfg->blockCfg[i].intvpar2        =   0;
        pEncHdrCfg->blockCfg[i].intvpar3        =   0;
    }

    return;        
}

/** ============================================================================
 *   @n@b prepare_wcdma_rmhdr_cfg
 *
 *   @b Description
 *   @n Sets up the Rate Modulation engine header for the test. 
 *
 *   @param[out]  
 *   @n pRmHdrCfg       Rate Modulation Header configuration thus populated for the test.
 * 
 *   @param[in]  
 *   @n xCdmaChanType   WCDMA channel type.
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
static Void prepare_wcdma_rmhdr_cfg
(
    Bcp_RmHdr_xCdmaCfg*         pRmHdrCfg,
    UInt8                       xCdmaChanType,
    UInt32                      Ne,
    UInt32                      einit_sys,
    UInt32                      eminus_sys,
    UInt32                      eplus_sys,
    UInt32                      einit1_p1,
    UInt32                      eminus1_p1,
    UInt32                      eplus1_p1,
    UInt32                      einit_p1,
    UInt32                      eminus_p1,
    UInt32                      eplus_p1,
    UInt32                      einit1_p2,
    UInt32                      eminus1_p2,
    UInt32                      eplus1_p2,
    UInt32                      einit_p2,
    UInt32                      eminus_p2,
    UInt32                      eplus_p2,
    UInt32                      punctureFlag,
    UInt32                      Ncol,
    UInt32                      Nrow,
    UInt8                       turboFlag,
    UInt8                       halfRateFlag,
    UInt8                       scrFlag
)
{
    UInt32                      i;        

    /* Setup the xCDMA Rate Modulation header as per inputs passed. */        
    pRmHdrCfg->local_hdr_len        =   6;
    pRmHdrCfg->input_order          =   1;
	if (xCdmaChanType == WCDMAFDD_REL99DL)
        pRmHdrCfg->half_rate        =   halfRateFlag;
    else
        pRmHdrCfg->half_rate        =   0;

    if (xCdmaChanType == WCDMAFDD_HSDPA || xCdmaChanType == TDSCDMA_HSUPAPIC)
    {
        pRmHdrCfg->collect_cols     =   Ncol;
        pRmHdrCfg->collect_rows     =   Nrow;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC || xCdmaChanType == WCDMAFDD_REL99DL)
    {
        pRmHdrCfg->collect_cols     =   0;
        pRmHdrCfg->collect_rows     =   0;
    }

    if (scrFlag == 0)
        pRmHdrCfg->num_scram        =   0;
    else if (scrFlag == 1)
        pRmHdrCfg->num_scram        =   256;

    /* Setup Systematic channel */
    pRmHdrCfg->sys0_len             =   Ne/3;
    pRmHdrCfg->sys0_init2           =   einit_sys;
    pRmHdrCfg->sys0_minus2          =   eminus_sys;
    pRmHdrCfg->sys0_plus2           =   eplus_sys;
    pRmHdrCfg->sys0_alpha           =   0;
    pRmHdrCfg->sys0_beta            =   0;
    pRmHdrCfg->sys0_puncture        =   punctureFlag;

    if (xCdmaChanType == WCDMAFDD_HSDPA || xCdmaChanType == TDSCDMA_HSUPAPIC)
        pRmHdrCfg->sys0_turbo       =   3;
	else if (xCdmaChanType == WCDMAFDD_HSUPAPIC)
        pRmHdrCfg->sys0_turbo       =   2;
	else if (xCdmaChanType == WCDMAFDD_REL99DL)
	{
		/*  For convolution code:   ConvTurbo set to 0
		 *  For turbo code:         ConvTurbo set to 1 with ParityParameter set 1 or 
         *                          ConvTurbo set to 2 with ParityParameter set 2  
         */ 
        if (turboFlag == 0)//CC
            pRmHdrCfg->sys0_turbo   =   0;
		else if (turboFlag == 1)//TC
            pRmHdrCfg->sys0_turbo   =   2;
	}

    /* Transport channels 0-4 not used */
    for (i = 0; i < 5; i ++)
    {
        pRmHdrCfg->channelCfg[i].sys_len        =   0;            
        pRmHdrCfg->channelCfg[i].sys_init2      =   0;            
        pRmHdrCfg->channelCfg[i].sys_minus2     =   0;            
        pRmHdrCfg->channelCfg[i].sys_plus2      =   0;            
        pRmHdrCfg->channelCfg[i].sys_aplha      =   0;            
        pRmHdrCfg->channelCfg[i].sys_beta       =   0;            
        pRmHdrCfg->channelCfg[i].sys_puncture   =   0;            
        pRmHdrCfg->channelCfg[i].sys_turbo      =   0;            
    }

    /* Setup Parity 1 */
    if (xCdmaChanType == WCDMAFDD_HSDPA || xCdmaChanType == TDSCDMA_HSUPAPIC)
    {
        pRmHdrCfg->p0_par1_len      =   Ne/3;
        pRmHdrCfg->p0_par1_init1    =   einit1_p1;
        pRmHdrCfg->p0_par1_minus1   =   eminus1_p1;
        pRmHdrCfg->p0_par1_plus1    =   eplus1_p1;
        pRmHdrCfg->p0_par1_init2    =   einit_p1;
        pRmHdrCfg->p0_par1_minus2   =   eminus_p1;
        pRmHdrCfg->p0_par1_plus2    =   eplus_p1;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC || xCdmaChanType == WCDMAFDD_REL99DL)
    {
        pRmHdrCfg->p0_par1_len      =   0;
        pRmHdrCfg->p0_par1_init1    =   0;
        pRmHdrCfg->p0_par1_minus1   =   0;
        pRmHdrCfg->p0_par1_plus1    =   0;
        pRmHdrCfg->p0_par1_init2    =   0;
        pRmHdrCfg->p0_par1_minus2   =   0;
        pRmHdrCfg->p0_par1_plus2    =   0;
    }

    /* Setup Parity 2 */
    if (xCdmaChanType == WCDMAFDD_HSDPA || xCdmaChanType == TDSCDMA_HSUPAPIC)
    {
        pRmHdrCfg->p0_par2_len      =   Ne/3;
        pRmHdrCfg->p0_par2_init1    =   einit1_p2;
        pRmHdrCfg->p0_par2_minus1   =   eminus1_p2;
        pRmHdrCfg->p0_par2_plus1    =   eplus1_p2;
        pRmHdrCfg->p0_par2_init2    =   einit_p2;
        pRmHdrCfg->p0_par2_minus2   =   eminus_p2;
        pRmHdrCfg->p0_par2_plus2    =   eplus_p2;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC || xCdmaChanType == WCDMAFDD_REL99DL)
    {
        pRmHdrCfg->p0_par2_len      =   0;
        pRmHdrCfg->p0_par2_init1    =   0;
        pRmHdrCfg->p0_par2_minus1   =   0;
        pRmHdrCfg->p0_par2_plus1    =   0;
        pRmHdrCfg->p0_par2_init2    =   0;
        pRmHdrCfg->p0_par2_minus2   =   0;
        pRmHdrCfg->p0_par2_plus2    =   0;
    }

    if (xCdmaChanType == WCDMAFDD_HSDPA || xCdmaChanType == TDSCDMA_HSUPAPIC)
    {
        pRmHdrCfg->p1_par1_len      =   0;
        pRmHdrCfg->p1_par1_init2    =   0;
        pRmHdrCfg->p1_par1_minus2   =   0;
        pRmHdrCfg->p1_par1_plus2    =   0;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC)
    {
        pRmHdrCfg->p1_par1_len      =   Ne/3;
        pRmHdrCfg->p1_par1_init2    =   einit_p1;
        pRmHdrCfg->p1_par1_minus2   =   eminus_p1;
        pRmHdrCfg->p1_par1_plus2    =   eplus_p1;
    }
    else if (xCdmaChanType == WCDMAFDD_REL99DL)
    {
        if (turboFlag == 1)
        {
            pRmHdrCfg->p1_par1_len      =   Ne/3;
            pRmHdrCfg->p1_par1_init2    =   einit_p1;
            pRmHdrCfg->p1_par1_minus2   =   eminus_p1;
            pRmHdrCfg->p1_par1_plus2    =   eplus_p1;
        }
        else
        {
            pRmHdrCfg->p1_par1_len      =   0;
            pRmHdrCfg->p1_par1_init2    =   0;
            pRmHdrCfg->p1_par1_minus2   =   0;
            pRmHdrCfg->p1_par1_plus2    =   0;
        }
    }

    if (xCdmaChanType == WCDMAFDD_HSDPA || xCdmaChanType == TDSCDMA_HSUPAPIC)
    {
        pRmHdrCfg->p1_par2_len      =   0;
        pRmHdrCfg->p1_par2_init2    =   0;
        pRmHdrCfg->p1_par2_minus2   =   0;
        pRmHdrCfg->p1_par2_plus2    =   0;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC)
    {
        pRmHdrCfg->p1_par2_len      =   Ne/3;
        pRmHdrCfg->p1_par2_init2    =   einit_p2;
        pRmHdrCfg->p1_par2_minus2   =   eminus_p2;
        pRmHdrCfg->p1_par2_plus2    =   eplus_p2;
    }
    else if (xCdmaChanType == WCDMAFDD_REL99DL)
    {
        if (turboFlag == 1)
        {
            pRmHdrCfg->p1_par2_len      =   Ne/3;
            pRmHdrCfg->p1_par2_init2    =   einit_p2;
            pRmHdrCfg->p1_par2_minus2   =   eminus_p2;
            pRmHdrCfg->p1_par2_plus2    =   eplus_p2;
        }
        else
        {
            pRmHdrCfg->p1_par2_len      =   0;
            pRmHdrCfg->p1_par2_init2    =   0;
            pRmHdrCfg->p1_par2_minus2   =   0;
            pRmHdrCfg->p1_par2_plus2    =   0;
        }
    }

    return;        
}

/** ============================================================================
 *   @n@b prepare_rel99_inthdr_cfg
 *
 *   @b Description
 *   @n Sets up the Interleaver engine header for the test. 
 *
 *   @param[out]  
 *   @n pIntHdrCfg      Interleaver Header configuration thus populated for the test.
 * 
 *   @param[in]  
 *   @n xCdmaChanType   WCDMA channel type.
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
static Void prepare_rel99_inthdr_cfg 
(
    Bcp_IntHdrCfg*      pIntHdrCfg,
    UInt8               xCdmaChanType,
    UInt32              numDataValues,
    UInt32              numDTXValues,
    UInt32              numDataFormatIn,
    UInt32              numIntWays,
    UInt32              numR2Length,
    UInt32              numDummy,
    UInt32              numDataFormatOut
)
{
    UInt32                      i;        

    /* Setup the interleaver header as per inputs passed */
    pIntHdrCfg->local_hdr_len       =   3;
    pIntHdrCfg->num_add_dtx         =   numDTXValues;
    for (i = 0; i < 6; i ++)
    {
        if (i == 0)
            pIntHdrCfg->tblCfg[i].num_data_format_in    =   numDataFormatIn;
        else
            pIntHdrCfg->tblCfg[i].num_data_format_in    =   0;
    }
    
    pIntHdrCfg->num_data_format_out     =   numDataFormatOut;

    if (xCdmaChanType == TDSCDMA_REL99DL)
    {
        pIntHdrCfg->flag_in_order           =   1;  // input is from encoder
        pIntHdrCfg->flag_half_rate          =   0;
    }
    else
    {
        pIntHdrCfg->flag_in_order           =   0;  // input is from rate matching engine
        pIntHdrCfg->flag_half_rate          =   0;
    }

    pIntHdrCfg->num_constellation           =   0;
    pIntHdrCfg->num_int_ways                =   numIntWays;
        
    for (i = 0; i < 6; i ++)
    {
        if (i == 0)
        {
            pIntHdrCfg->tblCfg[i].num_r2_length =   numR2Length;
            pIntHdrCfg->tblCfg[i].num_dummy     =   numDummy;
        }
        else
            pIntHdrCfg->tblCfg[i].num_r2_length =   0;
            pIntHdrCfg->tblCfg[i].num_dummy     =   0;
    }

    pIntHdrCfg->num_frame_count             =   1;

    return;        
}

/** ============================================================================
 *   @n@b prepare_diohdr_cfg
 *
 *   @b Description
 *   @n Sets up the DIO header for the test. 
 *
 *   @param[out]  
 *   @n pDioHdrCfg      DIO Header configuration thus populated for the test.
 * 
 *   @param[in]  
 *   @n bIsDioRead      Indicates DIO Read/Write operation. Set to 1 for Read
 *                      and 0 for Write
 * 
 *   @param[in]  
 *   @n blkCnt          Number of DIO block transfers being specified. Valid
 *                      values are 1-6
 * 
 *   @param[in]  
 *   @n dioAddr         List of DIO Addresses
 * 
 *   @param[in]  
 *   @n dioByteCnt      Byte count for each of DIO operations
 * 
 *   @return        
 *   @n None.
 * =============================================================================
 */
static Void prepare_diohdr_cfg (Bcp_DioHdrCfg* pDioHdrCfg, UInt8 bIsDioRead, UInt32 blkCnt, UInt32* dioAddr, UInt32* dioByteCnt)
{
    UInt32          i;

    /* Initialize the DIO header */        
    memset (pDioHdrCfg, 0, sizeof (Bcp_DioHdrCfg));

    /* Setup the DIO header as per inputs passed */
    pDioHdrCfg->dio_endian       =   Bcp_EndianFormat_128;
    pDioHdrCfg->dio_rd_wr        =   bIsDioRead;
    pDioHdrCfg->dio_blk_cnt      =   blkCnt;

    for (i = 0; i < blkCnt; i++)
    {
        pDioHdrCfg->dio_dmablk_cfg[i].dio_addr   =   dioAddr[i];
        pDioHdrCfg->dio_dmablk_cfg[i].dio_cnt    =   dioByteCnt[i];

        pDioHdrCfg->local_hdr_len                +=  2;
    }

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
    /* No PS/Info data for now */        
    pTmHdrCfg->ps_data_size     =   0;
    pTmHdrCfg->info_data_size   =   0;

    return;
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
 *   @n pR99UlParams    R99 UL test configuration.
 *
 *   @param[in]  
 *   @n channelType     Channel type.
 * =============================================================================
 */
static Void prepare_corhdr_cfg 
(
    Bcp_CorHdrCfg*          pCorHdrCfg, 
    BcpTest_R99UlParams*    pR99UlParams, 
    UInt8                   channelType
)
{
	UInt32                  SFradio[6]; 
	UInt32                  numLength[6]; 
	UInt32                  i;

    memset (pCorHdrCfg, 0, sizeof (Bcp_CorHdrCfg));

    /* Setup the correlation header as per inputs specified */
    pCorHdrCfg->local_hdr_len       =   2;
    pCorHdrCfg->pucch_despread_sel  =   Bcp_CorPucchDespread_Sel_DESPREAD;

	for (i = 0; i < 6; i++)
	{
		SFradio[i] = 0;
		numLength[i] = 0;
	}

	/* final despreading */
	if (pR99UlParams->SFradio ==   1) SFradio[0] = 0; 
	if (pR99UlParams->SFradio ==   2) SFradio[0] = 1; 
	if (pR99UlParams->SFradio ==   4) SFradio[0] = 2; 
	if (pR99UlParams->SFradio ==   8) SFradio[0] = 3; 
	if (pR99UlParams->SFradio ==  16) SFradio[0] = 4; 
	if (pR99UlParams->SFradio ==  32) SFradio[0] = 5; 
	if (pR99UlParams->SFradio ==  64) SFradio[0] = 6; 
	if (pR99UlParams->SFradio == 128) SFradio[0] = 7; 

	for (i = 0; i < pR99UlParams->nPhyCh; i++)
	{
		SFradio[i] = SFradio[0];
		numLength[i] = 2560*pR99UlParams->Ntr/pR99UlParams->partialSF;
	}

	for (i = 0; i < pR99UlParams->nPhyCh; i++)
        pCorHdrCfg->block_params[i].sf_ratio        =   SFradio [i];
	for (i = pR99UlParams->nPhyCh; i < 6; i++)
        pCorHdrCfg->block_params[i].sf_ratio        =   0;
	
	for (i = 0; i < pR99UlParams->nPhyCh; i++)
        pCorHdrCfg->block_params[i].despreading_length  =   numLength [i];
	for (i = pR99UlParams->nPhyCh; i < 6; i++)
        pCorHdrCfg->block_params[i].despreading_length  =   0;

    pCorHdrCfg->despread_flag_cplx  =   Bcp_CorDespread_Cplx_16REAL;

    return;
}

/** ============================================================================
 *   @n@b prepare_fddrel99_sslhdr_cfg
 *
 *   @b Description
 *   @n Sets up Soft Slicer (SSL) engine header configuration as per inputs provided.
 *
 *   @param[out]  
 *   @n pSslHdrCfg      WCDMA Rel 99 FDD SSL header thus built
 *
 *   @param[in]  
 *   @n pR99UlParams    Rel 99 UL test configuration.
 *
 *   @param[in]  
 *   @n channelType     Channel type.
 * =============================================================================
 */
static Void prepare_fddrel99_sslhdr_cfg 
(
    Bcp_SslHdr_WcdmaFddCfg*     pSslHdrCfg, 
    BcpTest_R99UlParams*        pR99UlParams, 
    UInt8                       channelType
)
{
	UInt32                      i;
	float                       unit_ssl;
	Int16                       unit_ssl16;
	float                       noiseVar_ssl;
	Int32                       noiseVar_ssl32;
	UInt32                      numSegment;

    memset (pSslHdrCfg, 0, sizeof (Bcp_SslHdr_WcdmaFddCfg));

    /* Setup the WCDMA FDD SSL Header as per inputs passed */
    pSslHdrCfg->local_hdr_len               =   9;
    pSslHdrCfg->modeSelCfg.fdd_tdd_sel      =   Bcp_SslFddTdd_Sel_FDD;
    pSslHdrCfg->modeSelCfg.jack_bit         =   0;
    pSslHdrCfg->modeSelCfg.tti_2ms_10ms_sel =   Bcp_Tti_Sel_10ms; 
    pSslHdrCfg->modeSelCfg.mod_type_sel     =   Bcp_ModulationType_BPSK;

    pSslHdrCfg->modeSelCfg.q_format         =   (Bcp_QFormat) pR99UlParams->qFmt;

    pSslHdrCfg->modeSelCfg.wcmda_num_phy_ch =   pR99UlParams->nPhyCh;

	numSegment = 5;

	/* unit */
	for (i = 0; i < numSegment; i++)
	{
		unit_ssl    = (float)pR99UlParams->modRms;
		unit_ssl16  = (UInt16) (unit_ssl + 0.5);//float represent into Int16 
        pSslHdrCfg->uva[i]                  =   unit_ssl16;
	}

	/* noise var */
	for (i = 0; i < numSegment; i++)
	{
		if (pR99UlParams->noiseVar == 0)
		{
			noiseVar_ssl32 = 0x7F800000;
		}
		else
		{
			noiseVar_ssl = (float)(1/(pR99UlParams->noiseVar*2));
			noiseVar_ssl32 = (*((Int32*)&noiseVar_ssl));//float represent into Int32 
		}
        pSslHdrCfg->scale_c0 [i]    =   noiseVar_ssl32;                   
	}
	
    pSslHdrCfg->wcdma_symb_seq      =   (7680*pR99UlParams->nPhyCh/pR99UlParams->SF/15*pR99UlParams->Ntr);

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
 *   @n pR99UlParams    Rel 99 UL test configuration.
 *
 *   @param[in]  
 *   @n channelType     Channel type.
 * =============================================================================
 */
static Void prepare_dnthdr_cfg
(
    Bcp_DntHdrCfg*              pDntHdrCfg, 
    UInt32                      nPhyCh,
    UInt32                      numDataValues,
    UInt32                      numR2Length,
    UInt32                      enumDataFormat,
    UInt32                      numDummy    
)
{
	UInt32      i;

    memset (pDntHdrCfg, 0, sizeof (Bcp_DntHdrCfg));

    /* Setup DNT Header as per inputs specified. */
    pDntHdrCfg->local_hdr_len   =   3;

	for (i = 0; i < nPhyCh; i++)
        pDntHdrCfg->tblCfg[i].num_r2_length         =   numR2Length;

	for (i = nPhyCh; i < 6; i++)
        pDntHdrCfg->tblCfg[i].num_r2_length         =   0;

	for (i = 0; i < nPhyCh; i++)
        pDntHdrCfg->tblCfg[i].num_dummy             =   numDummy;

	for (i = nPhyCh; i < 6; i++)
        pDntHdrCfg->tblCfg[i].num_dummy             =   0;

    pDntHdrCfg->num_frame_count                     =   nPhyCh;

	for (i = 0; i < nPhyCh; i++)
        pDntHdrCfg->tblCfg[i].num_data_format_in    =   enumDataFormat;

	for (i = nPhyCh; i< 6; i++)
        pDntHdrCfg->tblCfg[i].num_data_format_in    =   0;

    pDntHdrCfg->num_data_value                      =   numDataValues;
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
 *   @n pR99UlParams    Rel-99 UL test configuration.
 *
 *   @param[in]  
 *   @n channelType     Channel type.
 * =============================================================================
 */
static Void prepare_rdhdr_cfg
(
    Bcp_RdHdr_xCdmaCfg*         pRdHdrCfg, 
    BcpTest_R99UlParams*        pR99UlParams, 
    UInt8                       channelType
)
{
	UInt32                      trchIdx;
	BcpTest_R99UlTrch*          pR99UlTrch;
	UInt32                      turboCount;
	UInt32                      turboTrChIdx[2];
	UInt32                      betaTabIdx;
	UInt32                      alpha;
	UInt32                      numNonEmptyTrch;
	/* 25.212 table 6 */
	static Int32 betaTable[4][8] = {
		{0,-1,-1,-1,-1,-1,-1,-1},
		{0, 1,-1,-1,-1,-1,-1,-1},
		{0, 1, 2, 0,-1,-1,-1,-1},//same as 3GPP spec
		{0, 1, 2, 0, 1, 2, 0, 1}
	};

    memset (pRdHdrCfg, 0, sizeof (Bcp_RdHdr_xCdmaCfg));        

    /* Setup the RD header as per inputs passed */
    pRdHdrCfg->local_hdr_len    =   30;

	turboCount = 0;
	numNonEmptyTrch = 0;
	for (trchIdx = 0; trchIdx < pR99UlParams->numTrch; trchIdx ++)
	{
		pR99UlTrch        = &pR99UlParams->r99UlTrch[trchIdx];

		if (pR99UlTrch->numTB > 0)
		{
		if (pR99UlTrch->turboFlagBcpConfig == 1)//turbo puncture
		{
			if (pR99UlTrch->numRadioFrmPerTti == 1) 
			{
				betaTabIdx = 0;
				alpha = 0;
			}
			else if (pR99UlTrch->numRadioFrmPerTti == 2)
			{
				betaTabIdx = 1;
				alpha = 1;
			}
			else if (pR99UlTrch->numRadioFrmPerTti == 4)
			{
				betaTabIdx = 2;
				alpha = 0;
			}
			else if (pR99UlTrch->numRadioFrmPerTti == 8) 
			{
				betaTabIdx = 3;
				alpha = 1;
			}

            if (trchIdx == 0)
            {
                pRdHdrCfg->sys0_len         =   pR99UlTrch->Nij/3;
                pRdHdrCfg->sys0_init2       =   pR99UlTrch->tc_einit_sys;
                pRdHdrCfg->sys0_minus2      =   pR99UlTrch->tc_eminus_sys;
                pRdHdrCfg->sys0_plus2       =   pR99UlTrch->tc_eplus_sys;
                pRdHdrCfg->sys0_alpha       =   alpha;
                pRdHdrCfg->sys0_beta        =   betaTable[betaTabIdx][pR99UlParams->radioFrmIdx%pR99UlTrch->numRadioFrmPerTti];
                pRdHdrCfg->sys0_puncture    =   pR99UlTrch->punctureFlag;
                pRdHdrCfg->sys0_turbo       =   2-turboCount;
            }
            else
            {
                pRdHdrCfg->channelCfg[trchIdx-1].sys_len        =   pR99UlTrch->Nij/3;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_init2      =   pR99UlTrch->tc_einit_sys;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_minus2     =   pR99UlTrch->tc_eminus_sys;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_plus2      =   pR99UlTrch->tc_eplus_sys;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_aplha      =   alpha;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_beta       =   betaTable[betaTabIdx][pR99UlParams->radioFrmIdx%pR99UlTrch->numRadioFrmPerTti];
                pRdHdrCfg->channelCfg[trchIdx-1].sys_puncture   =   pR99UlTrch->punctureFlag;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_turbo      =   2-turboCount;
            }
			
			turboTrChIdx[turboCount++] = trchIdx;
		}
		else//CC and turbo repeat
		{
            if (trchIdx == 0)
            {
                pRdHdrCfg->sys0_len         =   pR99UlTrch->Nij;
                pRdHdrCfg->sys0_init2       =   pR99UlTrch->cc_einit;
                pRdHdrCfg->sys0_minus2      =   pR99UlTrch->cc_eminus;
                pRdHdrCfg->sys0_plus2       =   pR99UlTrch->cc_eplus;
                pRdHdrCfg->sys0_alpha       =   0;
                pRdHdrCfg->sys0_beta        =   0;
                pRdHdrCfg->sys0_puncture    =   pR99UlTrch->punctureFlag;
                pRdHdrCfg->sys0_turbo       =   0;
            }
            else
            {
                pRdHdrCfg->channelCfg[trchIdx-1].sys_len        =   pR99UlTrch->Nij;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_init2      =   pR99UlTrch->cc_einit;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_minus2     =   pR99UlTrch->cc_eminus;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_plus2      =   pR99UlTrch->cc_eplus;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_aplha      =   0;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_beta       =   0;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_puncture   =   pR99UlTrch->punctureFlag;
                pRdHdrCfg->channelCfg[trchIdx-1].sys_turbo      =   0;
            }
		}
		numNonEmptyTrch++;
		}
	}
	
	for (trchIdx = numNonEmptyTrch; trchIdx < 6; trchIdx ++)
	{
		/* no use */
        pRdHdrCfg->channelCfg[trchIdx-1].sys_len        =   0;
        pRdHdrCfg->channelCfg[trchIdx-1].sys_init2      =   0;
        pRdHdrCfg->channelCfg[trchIdx-1].sys_minus2     =   0;
        pRdHdrCfg->channelCfg[trchIdx-1].sys_plus2      =   0;
        pRdHdrCfg->channelCfg[trchIdx-1].sys_aplha      =   0;        
        pRdHdrCfg->channelCfg[trchIdx-1].sys_beta       =   0;
        pRdHdrCfg->channelCfg[trchIdx-1].sys_puncture   =   0;
        pRdHdrCfg->channelCfg[trchIdx-1].sys_turbo      =   0;
	}

	if (turboCount == 0)
	{
		/* the first turbo puncture TrCH parity 1 */
        pRdHdrCfg->p0_par1_len      =   0;
        pRdHdrCfg->p0_par1_init1    =   0;
        pRdHdrCfg->p0_par1_minus1   =   0;
        pRdHdrCfg->p0_par1_plus1    =   0;
        pRdHdrCfg->p0_par1_init2    =   0;
        pRdHdrCfg->p0_par1_minus2   =   0;
        pRdHdrCfg->p0_par1_plus2    =   0;

		/* the first turbo puncture TrCH parity 2 */
        pRdHdrCfg->p0_par2_len      =   0;
        pRdHdrCfg->p0_par2_init1    =   0;
        pRdHdrCfg->p0_par2_minus1   =   0;
        pRdHdrCfg->p0_par2_plus1    =   0;
        pRdHdrCfg->p0_par2_init2    =   0;
        pRdHdrCfg->p0_par2_minus2   =   0;
        pRdHdrCfg->p0_par2_plus2    =   0;

		/* the second turbo puncture TrCH parity 1 */
        pRdHdrCfg->p1_par1_len      =   0;
        pRdHdrCfg->p1_par1_init2    =   0;
        pRdHdrCfg->p1_par1_minus2   =   0;
        pRdHdrCfg->p1_par1_plus2    =   0;

		/* the second turbo puncture TrCH parity 2 */
        pRdHdrCfg->p1_par2_len      =   0;
        pRdHdrCfg->p1_par2_init2    =   0;
        pRdHdrCfg->p1_par2_minus2   =   0;
        pRdHdrCfg->p1_par2_plus2    =   0;
	}
	else if (turboCount == 1)
	{
		/* the first turbo puncture TrCH parity 1 */
        pRdHdrCfg->p0_par1_len      =   0;
        pRdHdrCfg->p0_par1_init1    =   0;
        pRdHdrCfg->p0_par1_minus1   =   0;
        pRdHdrCfg->p0_par1_plus1    =   0;
        pRdHdrCfg->p0_par1_init2    =   0;
        pRdHdrCfg->p0_par1_minus2   =   0;
        pRdHdrCfg->p0_par1_plus2    =   0;

		/* the first turbo puncture TrCH parity 2 */
        pRdHdrCfg->p0_par2_len      =   0;
        pRdHdrCfg->p0_par2_init1    =   0;
        pRdHdrCfg->p0_par2_minus1   =   0;
        pRdHdrCfg->p0_par2_plus1    =   0;
        pRdHdrCfg->p0_par2_init2    =   0;
        pRdHdrCfg->p0_par2_minus2   =   0;
        pRdHdrCfg->p0_par2_plus2    =   0;

		trchIdx    = turboTrChIdx[0];
		pR99UlTrch = &pR99UlParams->r99UlTrch[trchIdx];

		/* the second turbo puncture TrCH parity 1 */
        pRdHdrCfg->p1_par1_len      =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p1_par1_init2    =   pR99UlTrch->tc_einit_p1;
        pRdHdrCfg->p1_par1_minus2   =   pR99UlTrch->tc_eminus_p1;
        pRdHdrCfg->p1_par1_plus2    =   pR99UlTrch->tc_eplus_p1;

		/* the second turbo puncture TrCH parity 2 */
        pRdHdrCfg->p1_par2_len      =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p1_par2_init2    =   pR99UlTrch->tc_einit_p2;
        pRdHdrCfg->p1_par2_minus2   =   pR99UlTrch->tc_eminus_p2;
        pRdHdrCfg->p1_par2_plus2    =   pR99UlTrch->tc_eplus_p2;
	}
	else if (turboCount == 2)
	{
		trchIdx    = turboTrChIdx[1];
		pR99UlTrch = &pR99UlParams->r99UlTrch[trchIdx];
	
		/* the first turbo puncture TrCH parity 1 */
        pRdHdrCfg->p0_par1_len      =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p0_par1_init1    =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p0_par1_minus1   =   0;
        pRdHdrCfg->p0_par1_plus1    =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p0_par1_init2    =   pR99UlTrch->tc_einit_p1;
        pRdHdrCfg->p0_par1_minus2   =   pR99UlTrch->tc_eminus_p1;
        pRdHdrCfg->p0_par1_plus2    =   pR99UlTrch->tc_eplus_p1;

		/* the first turbo puncture TrCH parity 2 */
        pRdHdrCfg->p0_par2_len      =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p0_par2_init1    =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p0_par2_minus1   =   0;
        pRdHdrCfg->p0_par2_plus1    =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p0_par2_init2    =   pR99UlTrch->tc_einit_p2;
        pRdHdrCfg->p0_par2_minus2   =   pR99UlTrch->tc_eminus_p2;
        pRdHdrCfg->p0_par2_plus2    =   pR99UlTrch->tc_eplus_p2;

		trchIdx    = turboTrChIdx[0];
		pR99UlTrch = &pR99UlParams->r99UlTrch[trchIdx];

		/* the second turbo puncture TrCH parity 1 */
        pRdHdrCfg->p1_par1_len      =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p1_par1_init2    =   pR99UlTrch->tc_einit_p1;
        pRdHdrCfg->p1_par1_minus2   =   pR99UlTrch->tc_eminus_p1;
        pRdHdrCfg->p1_par1_plus2    =   pR99UlTrch->tc_eplus_p1;

		/* the second turbo puncture TrCH parity 2 */
        pRdHdrCfg->p1_par2_len      =   pR99UlTrch->Nij/3;
        pRdHdrCfg->p1_par2_init2    =   pR99UlTrch->tc_einit_p2;
        pRdHdrCfg->p1_par2_minus2   =   pR99UlTrch->tc_eminus_p2;
        pRdHdrCfg->p1_par2_plus2    =   pR99UlTrch->tc_eplus_p2;
	}

    pRdHdrCfg->flow_id_init         =   0;
    pRdHdrCfg->flow_id_max          =   0;
    pRdHdrCfg->fdd                  =   1;
    pRdHdrCfg->collect_cols         =   0;
    pRdHdrCfg->collect_rows         =   1;
    pRdHdrCfg->turbo_length         =   0;
    pRdHdrCfg->turbo_count          =   0;
    pRdHdrCfg->tcp3d_dyn_range      =   1;
    pRdHdrCfg->tcp3d_scale_factor   =   16;
    pRdHdrCfg->en_harq_in           =   0;            
    pRdHdrCfg->en_harq_out          =   0;            
    pRdHdrCfg->harq_in_addr         =   0;            
    pRdHdrCfg->harq_out_addr        =   0;            

    return;
}


/** ============================================================================
 *   @n@b add_dl_test_config_data
 *
 *   @b Description
 *   @n Given a Tx FDQ handle, this API sets up the DL parameters in CPPI 
 *      packet for test. It adds all the BCP configuration params and the payload 
 *      to each packet. On success, returns 0 and returns -1 to indicate error.
 *
 *   @param[in]  
 *   @n hBcp        BCP driver handle
 *
 *   @param[in]  
 *   @n hTxFDQ      Tx FDQ handle
 * 
 *   @return        Int32
 *   @n 0       -   Success.
 *
 *   @n -1      -   Error setting up/sending Rel 99 DL packet.
 * =============================================================================
 */
static Int32 add_dl_test_config_data (Bcp_DrvHandle hBcp, Qmss_QueueHnd hTxFDQ, Bcp_TxHandle hTx)
{
    FILE*                       pTestCfgFile;        
    BcpTest_R99DlParams*        pR99DlParams;
    UInt32                      dataBufferLen, tmpLen, tmpVal, i;
    UInt8                       wcdmaChanType;
    Bcp_RadioStd                radioStd;
    Bcp_GlobalHdrCfg            bcpGlblHdrCfg;
    Bcp_CrcHdrCfg               crcHdrCfg;
    Bcp_EncHdrCfg               encHdrCfg;
    Bcp_RmHdr_xCdmaCfg          wcdmaRmHdrCfg;
    Bcp_IntHdrCfg               intHdrCfg;
    Bcp_TmHdrCfg                tmHdrCfg;
	UInt32                      trchIdx;
	UInt32                      realTrchIdx;
	BcpTest_R99DlTrch*          pR99DlTrch;
	UInt32                      rfIdx, ttiIdx;
	UInt32                      dioAddrArray[6];
	UInt32                      dioCountArray[6];
	UInt32                      trchLenArray[6];
	UInt8                       flushFlag;
	UInt32                      numDTX;
	UInt32                      R2lenght;
	UInt32                      numDummy;
	UInt32                      lastNonEmptyTrCH;
    Bcp_DioHdrCfg               dioHdrCfg;
    Cppi_Desc*                  pCppiDesc;
    UInt8*                      pDataBuffer;
    Int32                       temp = 0, ret;
    UInt8*  					pStartDataBuffer;

	/* 25.211 table 11 slot format 0, 1, 2,   3,  4   5   6  7   8   9  10  11  12   13   14   15    16 */
	const UInt32 Ndata_tab[17] =  {4, 2, 16, 14, 14, 12, 10, 8, 34, 32, 30, 28, 60, 140, 288, 608, 1248};

    if ((pR99DlParams = Bcp_osalMalloc (sizeof (BcpTest_R99DlParams), FALSE)) == NULL)
    {
#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Error allocating memory for R99 DL Param Info \n");
#endif
        return -1;
    }
    memset (pR99DlParams, 0, sizeof (BcpTest_R99DlParams));

#ifdef BCP_TEST_DEBUG
    Bcp_osalLog ("Reading test configuration ... \n");
#endif

    /* Get the input test vector params */
    if ((pTestCfgFile = fopen ("..\\..\\testcases\\wcdma\\ST_WCDMA_FDDR99DL_00003.txt", "r")) == NULL)
    {
#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Error opening test configuration file: ..\..\testcases\wcdma\ST_WCDMA_FDDR99DL_00003.txt\n");
#endif
        return -1;
    }
    read_dl_test_config (pTestCfgFile, pR99DlParams);
    fclose (pTestCfgFile);
    
    /* Generate rest of the parameters */
    if (pR99DlParams->ChType == 1)
        pR99DlParams->Ndata = pR99DlParams->numPhyCh*15*72;
    else
        pR99DlParams->Ndata = pR99DlParams->numPhyCh*15*Ndata_tab[pR99DlParams->slotFormat];//numPhyCh*15*(Ndata1+Ndata2)

    for ( i = 0; i < pR99DlParams->numTrch; i++)
    {
        pR99DlParams->r99DlTrch[i].TFI         = pR99DlParams->TFCSTab[pR99DlParams->usedTFCI][i];
        pR99DlParams->r99DlTrch[i].numTTItoRun = FDD_R99_DL_NUM_RF_TO_RUN/pR99DlParams->r99DlTrch[i].numRadioFrmPerTti;
    }

    pR99DlParams->TACformatFlag = 1; // Use packed TAC format
		
    /* calculate rest of parameters */
    computeParams_r99Dl(pR99DlParams);

    if (pR99DlParams->numTrch >= 7)//14 TrCH 1 data lenght is not correct
        return -1;            

    radioStd        =   Bcp_RadioStd_WCDMA_R99; 
    wcdmaChanType   =   WCDMAFDD_REL99DL;

    /* Start adding BCP Packet headers based on the test configuration we read. */
	for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
	{
		pR99DlTrch  =   &(pR99DlParams->r99DlTrch[trchIdx]);
		if (pR99DlTrch->numTB) 
            lastNonEmptyTrCH = trchIdx;
	}

    /* Finally add the data to the packet in the same order as the headers */
    if ((pTestCfgFile = fopen("..\\..\\testcases\\wcdma\\bcp_fddr99dl_input.dat","r")) == NULL)
    {
#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Cannot open data input file: ..\..\testcases\wcdma\bcp_fddr99dl_input.dat\n");
#endif
        return -1;
    }
    
	/* TTI packets */
	for (trchIdx = 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
	{
		pR99DlTrch = &(pR99DlParams->r99DlTrch[trchIdx]);

		if (pR99DlTrch->numTB)
		{
			for (ttiIdx = 0; ttiIdx < pR99DlTrch->numTTItoRun; ttiIdx++)
			{
				if ((trchIdx == lastNonEmptyTrCH) && (ttiIdx == pR99DlTrch->numTTItoRun-1))
					flushFlag = 1;
				else
					flushFlag = 0;

                /* Build and Send a packet with WCDMA Rel-99 DL parameters for BCP Processing */
                if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hTxFDQ)) == NULL)
                {
#ifdef BCP_TEST_DEBUG
                    Bcp_osalLog ("Out of Tx FDs! \n");
#endif
		            return -1;
                }
                pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
                Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        
	            memset (pDataBuffer, 0, dataBufferLen); 

                /* Initialize our data buffer length running counter */
                dataBufferLen   =   0;
                tmpLen			=	0;
                
                /* Save the start pointer and set running pointer to CRC header */
                pStartDataBuffer = pDataBuffer;
                pDataBuffer 	+=	8;
                dataBufferLen	+=	8;

                /* Header 2: CRC Header */
                prepare_crchdr_cfg (&crcHdrCfg, radioStd, pR99DlTrch->TBsize, pR99DlTrch->numFillerBits, 0, 0, 0, 
                                    pR99DlTrch->CRCsize, pR99DlTrch->numTB, 1, 0, &tmpVal);
                if (Bcp_addCRCHeader (&crcHdrCfg, pDataBuffer, &tmpLen) < 0)
                {
#ifdef BCP_TEST_DEBUG
                    Bcp_osalLog ("Failed to add CRC Header to packet \n");
#endif
                    return -1;
                }
                pDataBuffer 	+=	tmpLen;
                dataBufferLen	+=	tmpLen;
                tmpLen			=	0;
               
                /* Header 3: Encoder Header */
                prepare_wcdma_enchdr_cfg (&encHdrCfg, pR99DlTrch->codeBkSizeK, pR99DlTrch->numCodeBks, 
                                            pR99DlTrch->turboFlag, pR99DlTrch->halfCodeRateFlag);
                if (Bcp_addEncoderHeader (&encHdrCfg, pDataBuffer, &tmpLen) < 0)
                {
#ifdef BCP_TEST_DEBUG
                    Bcp_osalLog ("Failed to add Encoder Header to packet \n");            
#endif
                    return -1;
                }
                pDataBuffer 	+=	tmpLen;
                dataBufferLen	+=	tmpLen;
                tmpLen			=	0;

				/* rate match */
				if (pR99DlTrch->turboFlag == 1)//TC puncture
				{
                    /* Header 4: Rate matching header */
                    prepare_wcdma_rmhdr_cfg (&wcdmaRmHdrCfg, wcdmaChanType, 
                                            pR99DlTrch->Niltti,
						                    pR99DlTrch->tc_einit_sys,
						                    pR99DlTrch->tc_eminus_sys,
						                    pR99DlTrch->tc_eplus_sys,
						                    0,
						                    0,
                                            0,
						                    pR99DlTrch->tc_einit_p1,
						                    pR99DlTrch->tc_eminus_p1,
						                    pR99DlTrch->tc_eplus_p1,
						                    0,
						                    0,
						                    0,
						                    pR99DlTrch->tc_einit_p2,
						                    pR99DlTrch->tc_eminus_p2,
						                    pR99DlTrch->tc_eplus_p2,
						                    pR99DlTrch->punctureFlag,
						                    0,
						                    0,
						                    pR99DlTrch->turboFlagBcpConfig,
						                    pR99DlTrch->halfCodeRateFlag,
						                    0);
				}
				else//CC and TC repeat
				{
					prepare_wcdma_rmhdr_cfg(&wcdmaRmHdrCfg, wcdmaChanType,
						                    pR99DlTrch->Niltti*3,
						                    pR99DlTrch->cc_einit,
						                    pR99DlTrch->cc_eminus,
						                    pR99DlTrch->cc_eplus,
						                    0,
                                            0,
						                    0,
						                    0,
						                    0,
						                    0,
						                    0,
						                    0,
						                    0,
						                    0,
						                    0,
						                    0,
						                    pR99DlTrch->punctureFlag,
						                    0,
						                    0,
						                    pR99DlTrch->turboFlagBcpConfig,
						                    pR99DlTrch->halfCodeRateFlag,
						                    0);
				}
                if (Bcp_addxCdma_RMHeader (&wcdmaRmHdrCfg, pDataBuffer, &tmpLen) < 0)
                {
#ifdef BCP_TEST_DEBUG
                    Bcp_osalLog ("Failed to add WCDMA rate modulation header to packet \n");            
#endif
                    return -1;
                }
                pDataBuffer 	+=	tmpLen;
                dataBufferLen	+=	tmpLen;
                tmpLen			=	0;

                /* Header 5: 1st Interleaver header */
                prepare_rel99_inthdr_cfg (&intHdrCfg, wcdmaChanType,
						            pR99DlTrch->numBitsRmOut,
                                    pR99DlTrch->numBits1DTXOut - pR99DlTrch->numBitsRmOut,//numDTXValues,
						            1,//B1B
						            pR99DlTrch->numRadioFrmPerTti,
						            pR99DlTrch->numBits1IntOutPerRadioFrm,
						            0,//numDummy always 0
						            0); //B2B
                if (Bcp_addInterleaverHeader (&intHdrCfg, pDataBuffer, &tmpLen) < 0)
                {
#ifdef BCP_TEST_DEBUG
                    Bcp_osalLog ("Failed to add Interleaver header to packet \n");            
#endif
                    return -1;
                }
                pDataBuffer 	+=	tmpLen;
                dataBufferLen	+=	tmpLen;
                tmpLen			=	0;
                
                /* Header 6: DIO header with "write" request */
                prepare_diohdr_cfg (&dioHdrCfg,
                                    0,  // write operation
                                    1,  // number of DIO blocks
						            (UInt32 *)&pR99DlTrch->dioAddrPerRadioFrm[ttiIdx*pR99DlTrch->numRadioFrmPerTti],
                                    (UInt32 *)&pR99DlTrch->dioNumBytesPerTti); 
                if (Bcp_addDIOHeader (&dioHdrCfg, pDataBuffer, &tmpLen) < 0)
                {
#ifdef BCP_TEST_DEBUG
                    Bcp_osalLog ("Failed to add DIO header to packet \n");            
#endif
                    return -1;
                }
                pDataBuffer 	+=	tmpLen;
                dataBufferLen	+=	tmpLen;
                tmpLen			=	0;

                /* Header 7: Traffic Manager header */
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
                bcpGlblHdrCfg.flush             =   flushFlag;
                bcpGlblHdrCfg.drop              =   0;
                bcpGlblHdrCfg.halt              =   0;
                bcpGlblHdrCfg.radio_standard    =   radioStd;
                bcpGlblHdrCfg.hdr_end_ptr       =   ((dataBufferLen + 3) >> 2); // 2 + 4 + 2 + 7 + 4 + 3 + 2 + 0 words pad
                bcpGlblHdrCfg.flow_id           =   RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM);
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

	            while ((ret = fscanf(pTestCfgFile, "0x%x[^\n]", &temp)) != EOF)
                {
                    getc (pTestCfgFile);
                    if (ret == 0)
                        break;                       
                    *(UInt32 *)pDataBuffer  =   temp;
                    dataBufferLen 		    +=  4;
                    pDataBuffer				+=	4;
	            }

#ifdef BCP_TEST_DEBUG
                Bcp_osalLog ("Sending a packet of len: %d to BCP ...\n", dataBufferLen);
#endif

                /* Setup the data length in the descriptor */
                Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLen);
                Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLen);

                /* Since BCP is local to the device, set destination address to NULL */
                Bcp_send (hTx, (Bcp_DrvBufferHandle) pCppiDesc, BCP_TEST_SIZE_HOST_DESC, NULL);
			}//end of TTI
		}//end if (pR99DlTrch->numTB)
		else
		{
			//in fixed pos, a empty TrCH is in the CCTrCH if numBits1IntOutPerRadioFrm > 0, 
            //we need to prepare a zero block in memory using DIO, this just for test.
			//if if numBits1IntOutPerRadioFrm = 0, the empty TrCH is not in CCtrCH, we don't need to prepare zero block
			if (pR99DlParams->dtxPosType == 1 && pR99DlTrch->numBits1IntOutPerRadioFrm > 0)
			{
				for (rfIdx = 0; rfIdx < FDD_R99_DL_NUM_RF_TO_RUN; rfIdx++)
				{
                    if ((trchIdx == pR99DlParams->numTrch - 1) && (rfIdx == FDD_R99_DL_NUM_RF_TO_RUN - 1))
							flushFlag = 1;
						else
							flushFlag = 1;

                    /* Build and Send a packet with WCDMA Rel-99 DL parameters for BCP Processing */
                    if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hTxFDQ)) == NULL)
                    {
#ifdef BCP_TEST_DEBUG
                        Bcp_osalLog ("Out of Tx FDs! \n");
#endif
		                return -1;
                    }
                    pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
                    Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        
	                memset (pDataBuffer, 0, dataBufferLen); 

                    /* Initialize our data buffer length running counter */
                    dataBufferLen   =   0;
                    tmpLen			=	0;
                
                    /* Save the start pointer and set running pointer to CRC header */
                    pStartDataBuffer = pDataBuffer;
                    pDataBuffer 	+=	8;
                    dataBufferLen	+=	8;

                    /* Header 2: DIO Header */
                    prepare_diohdr_cfg (&dioHdrCfg,
                                        0,  // write operation
                                        1,  // number of DIO blocks
						                (UInt32 *)&pR99DlTrch->dioAddrPerRadioFrm[rfIdx],
                                        (UInt32 *)&pR99DlTrch->dioNumBytesPerRadioFrm); 
                    if (Bcp_addDIOHeader (&dioHdrCfg, pDataBuffer, &tmpLen) < 0)
                    {
#ifdef BCP_TEST_DEBUG
                        Bcp_osalLog ("Failed to add DIO header to packet \n");            
#endif
                        return -1;
                    }
                    pDataBuffer 	+=	tmpLen;
                    dataBufferLen	+=	tmpLen;
                    tmpLen			=	0;

                    /* Header 3: Traffic Manager header */
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
                    bcpGlblHdrCfg.flush             =   flushFlag;
                    bcpGlblHdrCfg.drop              =   0;
                    bcpGlblHdrCfg.halt              =   0;
                    bcpGlblHdrCfg.radio_standard    =   radioStd;
                    bcpGlblHdrCfg.hdr_end_ptr       =   ((dataBufferLen + 3) >> 2); // 2 + 3 + 2 + 1 word pad
                    bcpGlblHdrCfg.flow_id           =   RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM);
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

	                while ((ret = fscanf(pTestCfgFile, "0x%x[^\n]", &temp)) != EOF)
                    {
                        getc (pTestCfgFile);
                        if (ret == 0)
                            break;                       
                        *(UInt32 *)pDataBuffer  =   temp;
                        dataBufferLen 		    +=  4;
                        pDataBuffer				+=	4;
	                }
#ifdef BCP_TEST_DEBUG
                    Bcp_osalLog ("Sending a packet of len: %d to BCP ...\n", dataBufferLen);
#endif
                    /* Setup the data length in the descriptor */
                    Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLen);
                    Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLen);

                    /* Since BCP is local to the device, set destination address to NULL */
                    Bcp_send (hTx, (Bcp_DrvBufferHandle) pCppiDesc, BCP_TEST_SIZE_HOST_DESC, NULL);
				}
			}
		}
	}//end of TrCH

	/* radio frame packets */
	for (rfIdx = 0; rfIdx < FDD_R99_DL_NUM_RF_TO_RUN; rfIdx++)
	{
        UInt32          dioHdrLength;
            
        /* Build and Send a packet with WCDMA Rel-99 DL parameters for BCP Processing */
        if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hTxFDQ)) == NULL)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Out of Tx FDs! \n");
#endif
		    return -1;
        }
        pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
        Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        
	    memset (pDataBuffer, 0, dataBufferLen); 

		if (rfIdx == FDD_R99_DL_NUM_RF_TO_RUN - 1)
			flushFlag = 1;
		else
			flushFlag = 0;

		/* DIO read: Trch Concatenate, all Trch in current RF */
		realTrchIdx     =   0;
        dioHdrLength    =   1; // DIO Header word 0
		for (trchIdx= 0; trchIdx < pR99DlParams->numTrch; trchIdx++ )
		{
			pR99DlTrch = &(pR99DlParams->r99DlTrch[trchIdx]);

			if (((pR99DlParams->dtxPosType == 1) && (pR99DlTrch->numBits1IntOutPerRadioFrm > 0)) ||
                ((pR99DlParams->dtxPosType == 0) && (pR99DlTrch->numTB > 0)))
			{
				dioAddrArray[realTrchIdx]  = pR99DlTrch->dioAddrPerRadioFrm[rfIdx];
				dioCountArray[realTrchIdx] = pR99DlTrch->dioNumBytesPerRadioFrm;
				realTrchIdx++;
                dioHdrLength += 2;
			}
		}

        /* Initialize our data buffer length running counter */
        dataBufferLen   =   tmpLen = 0;

        /* Save the start pointer and set running pointer to CRC header */
        pStartDataBuffer = pDataBuffer;
        pDataBuffer 	+=	8;
        dataBufferLen	+=	8;

        /* Header 2: DIO header with "read" request */
        prepare_diohdr_cfg (&dioHdrCfg,
                            1,  // read operation
                            realTrchIdx,  // number of DIO blocks
						    dioAddrArray,
                            dioCountArray); 
        if (Bcp_addDIOHeader (&dioHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Failed to add DIO header to packet \n");            
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;

		/* TrCH concatenate */
		realTrchIdx = 0;
		for (trchIdx= 0; trchIdx < pR99DlParams->numTrch; trchIdx++)
		{
			pR99DlTrch = &(pR99DlParams->r99DlTrch[trchIdx]);

			if (((pR99DlParams->dtxPosType == 1) && (pR99DlTrch->numBits1IntOutPerRadioFrm > 0)) ||
                ((pR99DlParams->dtxPosType == 0) && (pR99DlTrch->numTB > 0)))
			{
				trchLenArray[realTrchIdx] = pR99DlTrch->numBits1IntOutPerRadioFrm * 2;//2bit per DTX symbol
				realTrchIdx++;
			}
		}
        /* Header 2: CRC Header */
        prepare_crchdr_cfg (&crcHdrCfg, radioStd, pR99DlTrch->TBsize, pR99DlTrch->numFillerBits, 0, 0, 0, 
                            pR99DlTrch->CRCsize, pR99DlTrch->numTB, 2, realTrchIdx, trchLenArray);
        if (Bcp_addCRCHeader (&crcHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Failed to add CRC Header to packet \n");
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;
               
        /* Header 3: 2ndst Interleaver header */
		numDTX   = pR99DlParams->Ndata - pR99DlParams->numBitsTrchConcatOut;
		R2lenght = (UInt32)ceil((double)pR99DlParams->Ndata/30);
		numDummy = R2lenght*30 - numDTX - pR99DlParams->numBitsTrchConcatOut;

        prepare_rel99_inthdr_cfg (&intHdrCfg, wcdmaChanType,
                                  pR99DlParams->numBitsTrchConcatOut,
                                  numDTX,//numDTXValues,
						          0,//B2B
						          30,
						          R2lenght,
						          numDummy,
						          7-pR99DlParams->TACformatFlag*2); // 5: packed TAC format, 7: unpacked
        if (Bcp_addInterleaverHeader (&intHdrCfg, pDataBuffer, &tmpLen) < 0)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Failed to add Interleaver header to packet \n");            
#endif
            return -1;
        }
        pDataBuffer 	+=	tmpLen;
        dataBufferLen	+=	tmpLen;
        tmpLen			=	0;
                
        /* Header 4: Traffic Manager header */
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
        bcpGlblHdrCfg.flush             =   flushFlag;
        bcpGlblHdrCfg.drop              =   0;
        bcpGlblHdrCfg.halt              =   0;
        bcpGlblHdrCfg.radio_standard    =   radioStd;
        bcpGlblHdrCfg.hdr_end_ptr       =   ((dataBufferLen + 3) >> 2); // 2 + dioHdrLength + 4 + 9 + 2 +  words pad
        bcpGlblHdrCfg.flow_id           =   RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM);
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

#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Sending a packet of len: %d to BCP ...\n", dataBufferLen);
#endif

        /* Setup the data length in the descriptor */
        Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLen);
        Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLen);

        /* Since BCP is local to the device, set destination address to NULL */
        Bcp_send (hTx, (Bcp_DrvBufferHandle) pCppiDesc, BCP_TEST_SIZE_HOST_DESC, NULL);
	}//end of radio frame

    Bcp_osalFree (pR99DlParams, sizeof (BcpTest_R99DlParams), FALSE);

    fclose (pTestCfgFile);

    return 0;
}

/** ============================================================================
 *   @n@b test_wcdma_rel99_dl
 *
 *   @b Description
 *   @n Sets up the parameters required and runs a WCDMA Rel-99 DL test case.
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
Void test_wcdma_rel99_dl (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblFDQ)
{
    Bcp_TxCfg           txCfg;
    Bcp_TxHandle        hTx = NULL;
    Bcp_RxCfg           rxCfg;
    Bcp_RxHandle        hRx = NULL;
    Qmss_QueueHnd       hTxFDQ, hRxFDQ;
    UInt8               rxFlowId, rxSrcId, i;
    UInt32              rxDataBufferLen, rxPsInfoLen, rxDataTotalLen, testFail = 0, numTestPkts, refLen;
    UInt16              rxDestnTag;
    UInt8               *pRxDataBuffer, *pRxPsInfo, *refOutputBuf;
    Bcp_DrvBufferHandle hRxDrvBuffer;
    Bcp_DrvBufferHandle hVoid;
    Bcp_DrvBufferHandle hTmp;
    Qmss_Queue          tmpQ;
    
    /* Setup Rx side:
     *  -   Open BCP Rx queue on which BCP results are to be received
     *  -   Setup a Rx FDQ, CPPI Rx flow and TM flow to receive data
     *  -   Set the BCP Tunnel Rx endpoint configuration to NULL since we
     *      are processing BCP packets locally.
     */
    if (allocate_fdq (hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, 0, &hRxFDQ, NULL) < 0)
    {
        Bcp_osalLog ("Error opening Rx FDQ \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("Rx FDQ %d successfully setup with %d descriptors\n", hRxFDQ, RX_NUM_DESC);
    }
    tmpQ = Qmss_getQueueNumber(hRxFDQ);
    
    rxCfg.rxQNum                        =   RX_Q_NUM + CSL_chipReadReg (CSL_CHIP_DNUM);       
    rxCfg.bUseInterrupts                =   0;              // Use polling 
    memset (&rxCfg.flowCfg, 0, sizeof(Cppi_RxFlowCfg));
    rxCfg.flowCfg.flowIdNum             =   RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM);    
    rxCfg.flowCfg.rx_dest_qmgr          =   0;    
    rxCfg.flowCfg.rx_dest_qnum          =   RX_Q_NUM + CSL_chipReadReg (CSL_CHIP_DNUM);  
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
    rxCfg.flowCfg.rx_fdq2_qmgr          =   0;
    rxCfg.flowCfg.rx_fdq3_qnum          =   tmpQ.qNum;   
    rxCfg.flowCfg.rx_fdq3_qmgr          =   tmpQ.qMgr;

    /* Setup the corresponding TM flow entry configuration */
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
    rxCfg.tmFlowCfg.qfifo_out           =   RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM);      // Use Rx QFIFO1
    rxCfg.tmFlowCfg.ps_flags            =   0;      // No PS for now

    if ((hRx = Bcp_rxOpen (hBcp, &rxCfg, NULL)) == NULL)
    {
        Bcp_osalLog ("BCP Rx Open failed \n");
		goto cleanup_and_return;
    }
    else
    {
        Bcp_osalLog ("Flow %d opened to send data to RxQ: %d \n", RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM), RX_Q_NUM + CSL_chipReadReg (CSL_CHIP_DNUM));
    }

    /* Setup Tx side:
     *  -   Open BCP Tx queue using which data would be sent 
     *  -   Setup a Tx FDQ and initialize it with some Tx FDs
     *  -   Set the BCP Tunnel Tx endpoint configuration to NULL since we
     *      are processing BCP packets locally.
     */
    txCfg.txQNum    =   (Bcp_QueueId) (Bcp_QueueId_0 + CSL_chipReadReg (CSL_CHIP_DNUM));
    if ((hTx = Bcp_txOpen (hBcp, &txCfg, NULL)) == NULL)
    {
        Bcp_osalLog ("BCP Tx Open failed \n");
        return;
    }

    if (allocate_fdq (hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, 0, &hTxFDQ, NULL) < 0)
    {
        Bcp_osalLog ("Error opening Tx FDQ \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("Tx FDQ %d successfully setup with %d descriptors\n", hTxFDQ, TX_NUM_DESC);
    }

    for (numTestPkts = 0; numTestPkts < BCP_TEST_NUM_PACKETS; numTestPkts ++)
    {
        /* Read test configuration */
        if (add_dl_test_config_data (hBcp, hTxFDQ, hTx) < 0)
        {
            Bcp_osalLog ("Error building/sending R99 DL packets \n");
            testFail ++;
            goto cleanup_and_return;
        }

        /* Wait on data to be received from BCP and validate it. Poll on Rx queue for results. */
        while (Bcp_rxGetNumOutputEntries (hRx) == 0); 
        
        i = 0;
        while (1)
        {
            /* Data could arrive scattered across multiple linked descriptors.
             * Collect data from all linked descriptors and validate it.
             */
            rxDataTotalLen  =   0;

            if (Bcp_recv (hRx,
                    &hRxDrvBuffer,
                    &pRxDataBuffer,
                    &rxDataBufferLen,
                    &pRxPsInfo,
                    &rxPsInfoLen,
                    &rxFlowId,
                    &rxSrcId,
                    &rxDestnTag) == BCP_RETVAL_ENO_RESULT)
                break;                    

            if (i < 11)
            {
                refLen = R99_DL_OUTPUT_PKT_1_11_WRD_SIZE;
                refOutputBuf = (UInt8*)r99_dl_output_packet_1_11;
            }
            else if (i == 11)
            {
                /* Ignore the Dont care bits for the radio frame output */
                refLen = (R99_DL_OUTPUT_PKT_12_WRD_SIZE - 2);
                rxDataBufferLen -=  (2 * 4);
                refOutputBuf = (UInt8*)r99_dl_output_packet_12;
            }
            else if (i == 12)
            {
                /* Ignore the Dont care bits for the radio frame output */
                refLen = (R99_DL_OUTPUT_PKT_13_WRD_SIZE - 2);
                rxDataBufferLen -=  (2 * 4);
                refOutputBuf = (UInt8*)r99_dl_output_packet_13;
            }
            else if (i == 13) 
            {
                /* Ignore the Dont care bits for the radio frame output */
                refLen = (R99_DL_OUTPUT_PKT_14_WRD_SIZE - 2);
                rxDataBufferLen -=  (2 * 4);
                refOutputBuf = (UInt8*)r99_dl_output_packet_14;
            }
            else if (i == 14) 
            {
                /* Ignore the Dont care bits for the radio frame output */
                refLen = (R99_DL_OUTPUT_PKT_15_WRD_SIZE - 2);
                rxDataBufferLen -=  (2 * 4);
                refOutputBuf = (UInt8*)r99_dl_output_packet_15;
            }
            else
                goto cleanup_and_return;


            if (validate_rxdata ((UInt8 *)refOutputBuf, 
                                refLen*4, 
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
       
                    if (validate_rxdata ((UInt8 *)refOutputBuf, 
                                        refLen*4, 
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
            
            i++;
        }

#ifdef BCP_TEST_DEBUG
	    Bcp_osalLog ("Got %d packet(s) from BCP \n", i);
#endif
    }

cleanup_and_return:
    if (testFail > 0)
    {
        Bcp_osalLog ("WCDMA Rel-99 DL Test:    FAILED\n");                
        totalNumTestsFail ++;
    }
    else
    {
        Bcp_osalLog ("WCDMA Rel-99 DL Test:    PASS\n");                
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

/** ============================================================================
 *   @n@b add_ul_test_config_data
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
static Int32 add_ul_test_config_data (Bcp_DrvHandle hBcp, UInt8*  pDataBuffer)
{
    FILE*                       pTestCfgFile;        
    BcpTest_R99UlParams*        pR99UlParams;
    UInt32                      dataBufferLen, tmpLen, numR2Length, numDummy, rfIdx;
    UInt8                       wcdmaChanType;
    Bcp_RadioStd                radioStd;
    Bcp_GlobalHdrCfg            bcpGlblHdrCfg;
    Bcp_TmHdrCfg                tmHdrCfg;
    Bcp_CorHdrCfg               corHdrCfg;
    Bcp_SslHdr_WcdmaFddCfg      sslHdrCfg;
    Bcp_DntHdrCfg               dntHdrCfg;
    Bcp_RdHdr_xCdmaCfg          rdHdrCfg;
	float                       temp_float;
    UInt32                      temp_uint32;
    UInt8*  					pStartDataBuffer;

    if ((pR99UlParams = Bcp_osalMalloc (sizeof (BcpTest_R99UlParams), FALSE)) == NULL)
    {
#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Error allocating memory for R99 UL Param Info \n");
#endif
        return -1;
    }
    memset (pR99UlParams, 0, sizeof (BcpTest_R99UlParams));

#ifdef BCP_TEST_DEBUG
    Bcp_osalLog ("Reading test configuration ... \n");
#endif

    /* Get the Rel-99 UL input test vector params */
    if ((pTestCfgFile = fopen ("..\\..\\testcases\\wcdma\\ST_WCDMA_FDDR99UL_00001.txt", "r")) ==NULL)
    {
#ifdef BCP_TEST_DEBUG
	    Bcp_osalLog("Error opening test config file: ..\..\testcases\wcdma\ST_WCDMA_FDDR99UL_00001.txt\n");
#endif
        return -1;
    }
    read_ul_test_config (pTestCfgFile, pR99UlParams);
    fclose (pTestCfgFile);

    pR99UlParams->numRadioFmttoRun = 1;

    for (rfIdx = 0; rfIdx < pR99UlParams->numRadioFmttoRun; rfIdx++)
    {
        /* calculate rest of parameters */
        computeParams_r99ul (pR99UlParams);

        /* open ssl unit file */
        if ((pTestCfgFile = fopen ("..\\..\\testcases\\wcdma\\ST_WCDMA_FDDR99UL_00001_a_val_RX.dat", "r")) == NULL)
	    {
#ifdef BCP_TEST_DEBUG
	        Bcp_osalLog("Error opening file ..\..\testcases\wcdma\ST_WCDMA_FDDR99UL_0001_a_val_RX.dat\n");
#endif
            return -1;
        }
       
        /* read in ssl unit */
        if (fscanf(pTestCfgFile, "%d", &temp_uint32) != EOF)
            pR99UlParams->modRms = temp_uint32;
        else
        {
#ifdef BCP_TEST_DEBUG
		    Bcp_osalLog("ssl unit Error!\n");
#endif
            fclose (pTestCfgFile);
            return -1;
        }
        fclose (pTestCfgFile);

        /* open ssl noiseVar file */
        if ((pTestCfgFile = fopen ("..\\..\\testcases\\wcdma\\ST_WCDMA_FDDR99UL_00001_c_val_RX.dat", "r")) == NULL)
	    {
#ifdef BCP_TEST_DEBUG
	        Bcp_osalLog("Error opening file ..\..\testcases\wcdma\ST_WCDMA_FDDR99UL_00001_c_val_RX.dat\n");
#endif
            return -1;
        }
       
        /* read in ssl unit */
        if (fscanf(pTestCfgFile, "%f", &temp_float) != EOF)
            pR99UlParams->noiseVar = (double)1/(2*(double)temp_float);
        else
        {
#ifdef BCP_TEST_DEBUG
		    Bcp_osalLog("ssl noiseVar Error!\n");
#endif
            fclose (pTestCfgFile);
            return -1;
        }
        fclose (pTestCfgFile);

        radioStd        =   Bcp_RadioStd_WCDMA_R99; 
        wcdmaChanType   =   WCDMAFDD_REL99UL;

        /* Start adding BCP Packet headers based on the test configuration we read. */

        /* Initialize our data buffer length running counter */
        dataBufferLen   =   0;
        tmpLen			=	0;

        /* Save the start pointer and set running pointer to CRC header */
        pStartDataBuffer = pDataBuffer;
        pDataBuffer 	+=	8;
        dataBufferLen	+=	8;

        /* Header 2: Correlation Header */
        prepare_corhdr_cfg (&corHdrCfg, pR99UlParams, wcdmaChanType);
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
        prepare_fddrel99_sslhdr_cfg (&sslHdrCfg, pR99UlParams, wcdmaChanType);
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
	    numR2Length = (UInt32)ceil((double)pR99UlParams->Ndataj/30);
        numDummy    = numR2Length*30-pR99UlParams->Ndataj;
        prepare_dnthdr_cfg (&dntHdrCfg, 
                            pR99UlParams->nPhyCh,
                            pR99UlParams->Ndataj,
                            numR2Length,
                            8,
                            numDummy);
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
        prepare_rdhdr_cfg (&rdHdrCfg, pR99UlParams, wcdmaChanType);
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
        bcpGlblHdrCfg.hdr_end_ptr       =   ((dataBufferLen + 3) >> 2); // 2 + 3 + 10 + 4 + 31 + 2 + 0 pad
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
        if ((pTestCfgFile = fopen("..\\..\\testcases\\wcdma\\bcp_fddr99ul_input.dat","r")) == NULL)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Cannot open data input file: ..\..\testcases\wcdma\bcp_fddr99ul_input.dat \n");
#endif
            return -1;
        }
        read_data_from_file (pTestCfgFile, pDataBuffer, &dataBufferLen);
        fclose (pTestCfgFile);
    }

    Bcp_osalFree (pR99UlParams, sizeof (BcpTest_R99UlParams), FALSE);

    /* Successfully read the test configuration */        
    return dataBufferLen;
}


/** ============================================================================
 *   @n@b test_wcdma_rel99_ul
 *
 *   @b Description
 *   @n Sets up the parameters required and runs a WCDMA Rel-99 UL test case.
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
Void test_wcdma_rel99_ul (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblFDQ)
{
    Bcp_TxCfg           txCfg;
    Bcp_TxHandle        hTx = NULL;
    Bcp_RxCfg           rxCfg;
    Bcp_RxHandle        hRx = NULL;
    Qmss_QueueHnd       hTxFDQ, hRxFDQ;
    UInt8               rxFlowId, rxSrcId, i;
    UInt32              rxDataBufferLen, rxPsInfoLen, rxDataTotalLen, testFail = 0, numTestPkts, dataBufferLen;
    UInt16              rxDestnTag;
    UInt8*              pRxDataBuffer;
    UInt8*              pRxPsInfo;
    Bcp_DrvBufferHandle hRxDrvBuffer;
    Bcp_DrvBufferHandle hVoid;
    Bcp_DrvBufferHandle hTmp;
    Cppi_Desc*          pCppiDesc;
    UInt8*              pDataBuffer;
    Int32               dataBufferLenUsed;
    Qmss_Queue          tmpQ;

    /* Setup Rx side:
     *  -   Open BCP Rx queue on which BCP results are to be received
     *  -   Setup a Rx FDQ, CPPI Rx flow and TM flow to receive data
     *  -   Set the BCP Tunnel Rx endpoint configuration to NULL since we
     *      are processing BCP packets locally.
     */
    if (allocate_fdq (hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, 0, &hRxFDQ, NULL) < 0)
    {
        Bcp_osalLog ("Error opening Rx FDQ \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("Rx FDQ %d successfully setup with %d descriptors\n", hRxFDQ, RX_NUM_DESC);
    }
    tmpQ = Qmss_getQueueNumber(hRxFDQ);
    rxCfg.rxQNum                        =   RX_Q_NUM + CSL_chipReadReg (CSL_CHIP_DNUM);       
    rxCfg.bUseInterrupts                =   0;              // Use polling 
    memset (&rxCfg.flowCfg, 0, sizeof(Cppi_RxFlowCfg));
    rxCfg.flowCfg.flowIdNum             =   RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM);    
    rxCfg.flowCfg.rx_dest_qmgr          =   0;    
    rxCfg.flowCfg.rx_dest_qnum          =   RX_Q_NUM + CSL_chipReadReg (CSL_CHIP_DNUM);  
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

    /* Setup the corresponding TM flow entry configuration */
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
    rxCfg.tmFlowCfg.qfifo_out           =   RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM);      // Use Rx QFIFO1
    rxCfg.tmFlowCfg.ps_flags            =   0;      // No PS for now

    if ((hRx = Bcp_rxOpen (hBcp, &rxCfg, NULL)) == NULL)
    {
        Bcp_osalLog ("BCP Rx Open failed \n");
		goto cleanup_and_return;
    }
    else
    {
        Bcp_osalLog ("Flow %d opened to send data to RxQ: %d \n", RX_FLOW_ID + CSL_chipReadReg (CSL_CHIP_DNUM), RX_Q_NUM + CSL_chipReadReg (CSL_CHIP_DNUM));
    }

    /* Setup Tx side:
     *  -   Open BCP Tx queue using which data would be sent 
     *  -   Setup a Tx FDQ and initialize it with some Tx FDs
     *  -   Set the BCP Tunnel Tx endpoint configuration to NULL since we
     *      are processing BCP packets locally.
     */
    txCfg.txQNum    =   (Bcp_QueueId) (Bcp_QueueId_0 + CSL_chipReadReg (CSL_CHIP_DNUM));
    if ((hTx = Bcp_txOpen (hBcp, &txCfg, NULL)) == NULL)
    {
        Bcp_osalLog ("BCP Tx Open failed \n");
        return;
    }

    if (allocate_fdq (hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, 0, &hTxFDQ, NULL) < 0)
    {
        Bcp_osalLog ("Error opening Tx FDQ \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("Tx FDQ %d successfully setup with %d descriptors\n", hTxFDQ, TX_NUM_DESC);
    }

    for (numTestPkts = 0; numTestPkts < BCP_TEST_NUM_PACKETS; numTestPkts ++)
    {
        /* Get a descriptor for putting together a R99 UL packet */
        if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hTxFDQ)) == NULL)
        {
#ifdef BCP_TEST_DEBUG
            Bcp_osalLog ("Out of Tx FDs! \n");
#endif
            goto cleanup_and_return;
        }
        pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
        Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        
        memset (pDataBuffer, 0, dataBufferLen); 
            
        /* Read test configuration */
        if ((dataBufferLenUsed = add_ul_test_config_data (hBcp, pDataBuffer)) <= 0)
        {
            Bcp_osalLog ("Error reading test vectors/populating R99 UL packet \n");
            testFail ++;
            goto cleanup_and_return;
        }

#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Sending a packet of len: %d to BCP ...\n", dataBufferLenUsed);
#endif

        /* Setup the data length in the descriptor */
        Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLenUsed);
        Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLenUsed);

        /* Since BCP is local to the device, set destination address to NULL */
        Bcp_send (hTx, (Bcp_DrvBufferHandle) pCppiDesc, BCP_TEST_SIZE_HOST_DESC, NULL);

        /* Wait on data to be received from BCP and validate it. Poll on Rx queue for results. */
        while (Bcp_rxGetNumOutputEntries (hRx) == 0); 

#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Got %d packet(s) from BCP \n", Bcp_rxGetNumOutputEntries (hRx));
#endif
   
        for (i = 0; i < Bcp_rxGetNumOutputEntries (hRx); i ++)
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

            if (validate_rxdata ((UInt8 *)r99_ul_output_packet_1, 
                                R99_UL_OUTPUT_PKT_1_WRD_SIZE*4, 
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
       
                    if (validate_rxdata ((UInt8 *)r99_ul_output_packet_1, 
                                        R99_UL_OUTPUT_PKT_1_WRD_SIZE*4, 
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

            if (rxDataTotalLen != R99_UL_OUTPUT_PKT_1_WRD_SIZE * 4)
                testFail ++;                    
        }
    }

cleanup_and_return:
    if (testFail > 0)
    {
        Bcp_osalLog ("WCDMA Rel-99 UL Test:    FAILED\n");                
        totalNumTestsFail ++;
    }
    else
    {
        Bcp_osalLog ("WCDMA Rel-99 UL Test:    PASS\n");                
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
