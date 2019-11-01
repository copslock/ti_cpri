/** 
 *   @file  bcp_wcdma.h
 *
 *   @brief  
 *      Header file with data structures and definitions used by WCDMA tests.
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
#ifndef _BCP_WCDMA_H_
#define _BCP_WCDMA_H_

/* xCDMA Channel Types */
#define WCDMAFDD_HSDPA         0
#define WCDMAFDD_HSUPA         1
#define WCDMAFDD_HSUPAPIC      2
#define WCDMAFDD_REL99DL       3
#define WCDMAFDD_REL99UL       4
#define WCDMAFDD_REL99ULPIC    5

#define TDSCDMA_HSDPA          6
#define TDSCDMA_HSUPA          7
#define TDSCDMA_HSUPAPIC       8 
#define TDSCDMA_REL99DL        9
#define TDSCDMA_REL99UL       10
#define TDSCDMA_REL99ULPIC    11

#define FDD_R99_DL_MAX_TFCS_SIZE 150
#define FDD_R99_DL_MAX_NUM_TRCH  7
#define FDD_R99_DL_MAX_NUM_TF    6
#define FDD_R99_DL_NUM_RF_TO_RUN 4

#define FDD_R99_UL_MAX_TFCS_SIZE 66
#define FDD_R99_UL_MAX_NUM_TRCH  6
#define FDD_R99_UL_MAX_NUM_TF    9
#define FDD_R99_UL_MAX_NUM_PHYCH 6

#define FDD_R99_DL_MAX_RF        8

typedef struct _BcpTest_HsdpaParams 
{
	/* root */
	UInt32 TBsize;
	UInt32 modScheme; 
	UInt32 Nir; 
	UInt32 nPhyCh; 
	UInt32 RVindex;

	/* drived */
	UInt32 numCodeBks;
    UInt32 codeBkSizeK;
	UInt32 numFillerBits;
	UInt32 Ne;    //number of rate matching input bits
	UInt32 Nedata;//number of rate matching output bits
    UInt32 einit_sys;
    UInt32 eplus_sys;
	UInt32 eminus_sys;
	UInt32 einit1_p1;
	UInt32 eplus1_p1;
	UInt32 eminus1_p1;
	UInt32 einit1_p2;
	UInt32 eplus1_p2;
	UInt32 eminus1_p2;
	UInt32 einit_p1;
    UInt32 eplus_p1; 
	UInt32 eminus_p1; 
	UInt32 einit_p2; 
	UInt32 eplus_p2; 
	UInt32 eminus_p2; 
	UInt32 Ncol;
	UInt32 Nrow;
	UInt32 Nr;
	UInt32 Nc;
	UInt32 punctureFlag;
	UInt32 b;
	UInt32 valid;//0: not valid, 1: valid

	/* TAC format */
	UInt32 TACformatFlag;
} BcpTest_HsdpaParams;

typedef struct _BcpTest_Hsupa_PhyChParams 
{
	UInt32      SF;
	UInt32      partialSF;
	UInt32      SFradio;
	UInt32      Ndata;
	UInt16      modRms;
	double      noiseVar;
	UInt8*      pIntOut;//8-bit byte per binary bits
	Int16*      pModOut;//8-bit byte per binary bits
	Int16*      pSpreadOut;//8-bit byte per binary bits
	Int16*      pChannelOut;//8-bit byte per binary bits
	Int16*      pPartialDeSpreadOut;//16-bit real value
	Int16*      pFinalDeSpreadOut;//16-bit real value
	Int8*       pSoftSlicerOut;
	Int8*       pRateDeMatchOut;

} BcpTest_Hsupa_PhyChParams;

typedef struct _BcpTest_HsupaParams 
{
	UInt32      ttiType;//0:2msTTI, 1-10msTTI
	UInt32      startTTIN;
	UInt32      numTTItoRun;
	UInt32      maxNumReTrans;

	/* root, parameter for test */
	UInt32      TBsize;
	UInt32      maxSet0;      
	UInt32      Narq;
	double      PLnonmax;
	UInt32      commonPartialSF;
	UInt32      commonSFradio;
    
	//UInt16    modRms;
	UInt16      snr;
	//double    noiseVar;
    //
	/* root, paramters changed per TTI (packet)*/
	UInt32      RSN;
	UInt32      TTIN;
	UInt32      awgnFlag;
    UInt16      qFmt;

	/* derived */
	UInt32      edchCategory;
	UInt32      modScheme;//1:BPSK, 2:4PAM 
	UInt32      nPhyCh;
	BcpTest_Hsupa_PhyChParams   hsupaPhyCh[4]; 
	//UInt32    SF;
	UInt32      RVindex;
	UInt32      numCodeBks;
    UInt32      codeBkSizeK;
	UInt32      numFillerBits;
	UInt32      Ne;    //number of rate matching input bits
	UInt32      Nedata;//number of rate matching output bits
    UInt32      einit_sys;
    UInt32      eplus_sys;
	UInt32      eminus_sys;
	/**/
	UInt32      einit1_p1;
	UInt32      eplus1_p1;
	UInt32      eminus1_p1;
	UInt32      einit1_p2;
	UInt32      eplus1_p2;
	UInt32      eminus1_p2;
	/**/
	UInt32      einit_p1;
    UInt32      eplus_p1; 
	UInt32      eminus_p1; 
	UInt32      einit_p2; 
	UInt32      eplus_p2; 
	UInt32      eminus_p2; 
	UInt32      punctureFlag;
	UInt32      valid;//0: not valid, 1: valid
	float**     pRateDeMatchOut1;
	float**     pRateDeMatchOut2;
	float**     pLlrHistoryTi;
} BcpTest_HsupaParams;

/* R99 DL TF */
typedef struct _BcpTest_R99DlTF
{
		UInt32 TBsize;
		UInt32 numTB;
		UInt32 codeBkSizeK;
		UInt32 numCodeBks;
		UInt32 numFillerBits;

		UInt32 Niltti;//numBits in TTI before RM on TrCh i TF l	
		Int32  deltaNiltti;	

		UInt32 Nil;//only used in flexible pos
} BcpTest_R99DlTF; 

/* R99 DL TrCH */
typedef struct _BcpTest_R99DlTrch 
{
		UInt32 numTTItoRun;
		UInt32 numRadioFrmPerTti;
		UInt32 CRCsize;
		UInt32 RMattribute;
		UInt32 turboFlag;
		UInt32 turboFlagBcpConfig;
		UInt32 halfCodeRateFlag;
		
		/**/
		UInt32 TBsize;
		UInt32 numTB;
		UInt32 codeBkSizeK;
		UInt32 numCodeBks;
		UInt32 numFillerBits;
		
		/**/
		UInt32 Niltti;
		Int32  deltaNiltti;

		/**/
		UInt32 Xi;
		
		UInt32 cc_einit;
		UInt32 cc_eplus;
		UInt32 cc_eminus;

		UInt32 tc_einit_sys;
		UInt32 tc_eplus_sys;
		UInt32 tc_eminus_sys;

		UInt32 tc_einit_p1;
		UInt32 tc_eplus_p1;
		UInt32 tc_eminus_p1;

		UInt32 tc_einit_p2; 
		UInt32 tc_eplus_p2; 
		UInt32 tc_eminus_p2; 

		UInt32 punctureFlag;

		/**/
		UInt32 numBitsRmOut;             //number of real bits
		UInt32 numBits1DTXOut;           //number of real bits
		UInt32 numBits1IntOutPerRadioFrm;//number of real bits

		/**/	
		UInt32   TFI;
		UInt32   numTF;
		BcpTest_R99DlTF TF[FDD_R99_DL_MAX_NUM_TF];
		
		/**/
		UInt32 Nistar;     //intermediate variable, only used in fixed pos
		Int32  deltaNistar;//intermediate variable, only used in fixed pos
		Int32  deltaNimax; //intermediate variable, only used in fixed pos
		
		/* used by DIO module write which is one TrCH per TTI operation */
		UInt32 dioAddrPerRadioFrm[FDD_R99_DL_NUM_RF_TO_RUN];//must be in 128-bit bundery
		UInt32 dioNumBytesPerRadioFrm;//number of bytes
		UInt32 dioNumBytesPerTti;//number of bytes
		
} BcpTest_R99DlTrch; 

/* R99 DL */
typedef struct _BcpTest_R99DlParams
{
	UInt32     numPhyCh;
	UInt32     slotFormat;//0:data, 1: sccpch
	UInt32     ChType;//0:data, 1: s-ccpch
	UInt32     dtxPosType;//0:flexible, 1:fixed
	UInt32     Ndata;//numPhyCh*15*(Ndata1+Ndata2)
	UInt32     numTrch; 
	BcpTest_R99DlTrch r99DlTrch[FDD_R99_DL_MAX_NUM_TRCH];
	
	UInt32     TFCSsize;
	UInt32     TFCSTab[FDD_R99_DL_MAX_TFCS_SIZE][FDD_R99_DL_MAX_NUM_TRCH];//TFCS
	UInt32     totalSumTFCSTab[FDD_R99_DL_MAX_TFCS_SIZE];
	UInt32     usedTFCI;//used TFC

	UInt32     numBitsTrchConcatOut;//number of read bits, not include DTX, 
	
	/* used by DIO module read which is all TrCH per radio frame operation */
	UInt32     dioAddrPerRadioFrm[FDD_R99_DL_NUM_RF_TO_RUN][6];//[RF][TrCh]

	UInt32     TACformatFlag;
	
} BcpTest_R99DlParams;

/* R99 UL TF */
typedef struct _BcpTest_R99UlTF
{
		UInt32 TBsize;
		UInt32 numTB;
} BcpTest_R99UlTF; 

/* R99 UL TrCH */
typedef struct _BcpTest_R99UlTrch
{
		UInt32 numRadioFrmPerTti;
		UInt32 CRCsize;
		UInt32 RMattribute;
		UInt32 turboFlag;
		UInt32 turboFlagBcpConfig;
		UInt32 halfCodeRateFlag;
		
		UInt32 TBsize;
		UInt32 numTB;
		UInt32 codeBkSizeK;
		UInt32 numCodeBks;
		UInt32 numFillerBits;
	
		/**/
		UInt32 cc_einit;
		UInt32 cc_eplus;
		UInt32 cc_eminus;

		UInt32 tc_einit_sys;
		UInt32 tc_eplus_sys;
		UInt32 tc_eminus_sys;
		/**/
		UInt32 tc_einit_p1;
		UInt32 tc_eplus_p1;
		UInt32 tc_eminus_p1;
		/**/
		UInt32 tc_einit_p2; 
		UInt32 tc_eplus_p2; 
		UInt32 tc_eminus_p2; 

		UInt32 punctureFlag;

		UInt32 Xi; 
	
		UInt32   numTF;
		BcpTest_R99UlTF TF[FDD_R99_UL_MAX_NUM_TF];

		UInt32 Nij;
		Int32  deltaNij;

		UInt32 usedTF;
		
} BcpTest_R99UlTrch; 

/* R99 UL */
typedef struct _BcpTest_R99UlParams 
{
	UInt32      numRadioFmttoRun;
	UInt32      radioFrmIdx;
	UInt32      qFmt;
	float       PL;
	UInt32      maxSet0;
	UInt32      Ndataj;
	UInt32      SF;

	UInt32      totalSumRMxNmj;

	UInt32      numTrch; 
	BcpTest_R99UlTrch  r99UlTrch[FDD_R99_UL_MAX_NUM_TRCH];

	UInt32      nPhyCh;

	//UInt32      usedTFCI;
	//UInt32      TFCSsize;
	//UInt32      TFCSTab[FDD_R99_UL_MAX_TFCS_SIZE][FDD_R99_UL_MAX_NUM_TRCH];//TFCS
	//UInt32      usedTFC[FDD_R99_UL_MAX_NUM_TRCH];

	UInt32      partialSF;
	UInt32      SFradio;
	UInt16      modRms;
	double      noiseVar;
	UInt8     * pIntOut;//8-bit byte per binary bits
	Int16     * pModOut;//8-bit byte per binary bits
	Int16     * pSpreadOut;//8-bit byte per binary bits
	Int16     * pChannelOut;//8-bit byte per binary bits
	Int16     * pPartialDeSpreadOut;//16-bit real value
	Int16     * pFinalDeSpreadOut;//16-bit real value
	Int8      * pSoftSlicerOut;
	Int8      * pRateDeMatchOut;

	UInt32    Ntr;
} BcpTest_R99UlParams;

#endif /* _BCP_WCDMA_H_ */
