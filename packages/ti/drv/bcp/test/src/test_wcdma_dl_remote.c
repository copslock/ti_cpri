/** 
 *   @file  test_wcdma_dl.c
 *
 *   @brief  
 *      Runs WCDMA Downlink (DL) test case using BCP driver APIs.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this driver has been supplied.
 *  ===========================================================================
 *  \par
 */
#include "bcp_test.h"
#include "bcp_test_wcdma.h"

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>
#include <ti/csl/csl_srioAuxPhyLayer.h>

#define     TX_DATA_BUFFER_SIZE             3072 
#define     RX_DATA_BUFFER_SIZE             2048
#define     TX_NUM_DESC                     8
#define     RX_NUM_DESC                     8
#define     RX_Q_NUM                        900
#define     RX_FLOW_ID                      0
#define     BCP_TX_QUEUE(i)                 (QMSS_BCP_QUEUE_BASE + i)
#define     SRIO_TX_QUEUE(i)                (QMSS_SRIO_QUEUE_BASE + i)

extern const UInt32 DEVICE_ID1_16BIT;
extern const UInt32 DEVICE_ID1_8BIT;
extern const UInt32 DEVICE_ID_BCP_16BIT;
extern const UInt32 DEVICE_ID_BCP_8BIT;
extern UInt32   totalNumTestsPass, totalNumTestsFail;
extern Int32 srio_freeRecvBuffer (Void* hVoid, Void* hDrvBuffer);

/* Input test vector */
#pragma DATA_SECTION (testVectorHsdpa, ".testData");
static const UInt32 testVectorHsdpa [1][5] = { /* modScheme 0:QPSK, 1:16QAM, 2:64QAM */
    /*TBsize	modScheme	    Nir    nPhyCh    RVindex*/     
    { 19904,        1,         28800,    11,        0   } 
};  

/* Output Reference Data */
#define WCDMA_DL_OUTPUT_PKT_1_WRD_SIZE  660

#pragma DATA_SECTION (wcdma_dl_out_packet_1, ".testData");
static UInt32 wcdma_dl_out_packet_1[WCDMA_DL_OUTPUT_PKT_1_WRD_SIZE] = {
0x0b11d405,0x023b7515,0xdbd8d4ab,0x4b4bd5b0,
0x6c563dfe,0xcaa3f738,0xa67e1624,0x39f7d54f,
0x8fc41a45,0xe555602f,0x98234ae8,0x9e41fc66,
0x4d48b778,0x1cbc1c38,0x7f975df5,0xebc5288c,
0x6154814e,0x36823aa5,0xa1fdadd9,0x6412c616,
0x47703ec7,0x31059356,0x4cb0d247,0x368368bb,
0x09fc48c0,0xbc0d0c28,0x01a32772,0x0b6c1343,
0xbf0ee34b,0xadc2a144,0x0daf9506,0xbf2f42ea,
0x79d43a9e,0x85b2d179,0x0975ba4b,0x40be03e1,
0x482c95d9,0xdae85fbc,0x52e94515,0x9a4bb810,
0x361108d4,0xc2c2074b,0xf5cde3a3,0x65426e7c,
0x2e202a29,0x152b2074,0xa35b9012,0x11ad4fe5,
0xab5f23f9,0xc0738bf8,0x48fddd6e,0x1b7bc734,
0x9df5fd3c,0x5c7672ae,0xfb9dd1f4,0x13038a1c,
0x19840e1c,0xdc03077e,0xac4c54a1,0xf7313cc7,
0x9f364c1c,0xc56a9d2e,0x159d0542,0x55506ffb,
0x78a0bdfd,0x428a7f60,0x0140b9a3,0x4bed4304,
0xdddbda96,0x4766fa21,0xa1340b96,0xdee93da9,
0xa5a06628,0xc0485444,0x75c4cd88,0xcae5d523,
0xb443c96e,0x6e976aac,0x9c17e7c8,0x015d2b09,
0x94a99af9,0x7c726c3a,0x0616fdb4,0x96c5de8a,
0x39e3eb6c,0x91e1756a,0x11c0f5e9,0xeb31a4c8,
0x80c0b5ed,0xe8a9029d,0xdf7538ef,0x9acd09c7,
0x76269550,0x2ec6a35a,0xcc5cfd5c,0xb013cd8a,
0x714ab460,0x678569cc,0x16bd4501,0xbfc7284a,
0x232fb7dc,0x6b093b40,0xf9dd8b07,0x2a68b007,
0x362ffbfb,0xf2c74485,0x347abcca,0xecfac4f2,
0x804f882d,0x9d3e038b,0x5f8adea4,0x554f77cd,
0xbb5f2cbd,0x8458daf1,0x534e8590,0x30ebfdfa,
0x73ae6cc2,0xc4d3dff5,0xa09d8d8e,0x33a3e425,
0xb70a2641,0x5fc4dda5,0x4e8abe39,0xa5b69069,
0xc7fdde2e,0x5975a033,0xfab77422,0x44ba4583,
0xef10db32,0x5b092b77,0x8d54c5dd,0xf082d05e,
0x9d8708c2,0x236a2be4,0x4abf315e,0xcf6a2f24,
0x83d3ee79,0x5192d336,0x7bc7917c,0xe82ae07a,
0xce43a746,0x358cea4d,0xadbbf762,0xf67e1529,
0x4d35d500,0x0c647121,0x067987f1,0x01675586,
0x370d49ef,0x2d4809ef,0x5f7cdf09,0x8abeb245,
0xe441d8eb,0xe49d6c52,0x2c97054f,0xb90a67c3,
0xe01326d7,0x8904ebe9,0x3422b9c1,0x9021a8e3,
0x8ca30de8,0xb2aa22aa,0xe86d3ad0,0x100762f9,
0xa04d59f4,0xdc27527b,0x8cb53a52,0xb512b98e,
0xedd046c9,0x5add56d4,0x7227cc04,0x15fc6835,
0x7b9d1855,0x417433a0,0xbe10dd78,0xac73ec2a,
0x0a05a27f,0x987f7eb0,0x9f09bb52,0xfcdc7025,
0x47f78b1a,0xf3b187df,0xf78656f8,0x109fea86,
0xe5b0afd1,0x0c686a4f,0xeee48783,0x04d6af3c,
0x1597bfea,0x8df1aad3,0x51437920,0x8d80ec34,
0x774a9198,0xd00b3e08,0x0051c204,0xc5de2e58,
0xd0a33d88,0x0dbb8bbd,0xc41bb37e,0xc30f79f6,
0x8a8ead2e,0xdf4f16cd,0xac56e31a,0xa11345f9,
0x438250bf,0x62e267dd,0xf7c90bae,0x7a21a2b7,
0xd0ec0de6,0xa385a168,0x75a45761,0x685dae36,
0x0876b552,0xae631ffa,0xf38eeef3,0x88710938,
0xb2677a7b,0xd962db43,0xdb2cb1cc,0x44911a73,
0x2ee22970,0x25181e5b,0xc916bfb8,0xb1512cd1,
0xd64d0414,0xfcd245ed,0x2148857b,0x70436c82,
0x43a2839e,0x3edd4f03,0xcbf3c7fe,0x3b554400,
0x98525a1b,0x1fcbce80,0x5d2e3e2c,0x6cba6306,
0x5cad235c,0x478edd21,0x389da68e,0x93b16844,
0x156bcb87,0x3520306c,0x18bafe1f,0x5c1db8b5,
0x6cfdab5a,0x42ba5278,0xf4caae03,0x7784c119,
0x7acb7877,0xef0dcc3e,0xc261f0ca,0xb995e2d6,
0x632d5d29,0xb976a8c0,0xc03752f0,0xe48fb443,
0x0b32d8a5,0xa7123501,0x8a06110c,0x0f715fb2,
0xeecff07f,0xabccc554,0x60404f1d,0xcaae86cc,
0x3dbb608d,0xfae1932c,0x5577d2a3,0xaaa872b0,
0x069a9ceb,0x5d067f74,0x518fbffa,0xbd1b865f,
0xc6da5da3,0xaeeb76d2,0x20277eda,0x0cde9d6b,
0x19f0b162,0x37063e9b,0xc9d321d9,0x846fc94a,
0xd5a5589e,0x449b9d52,0x6c278993,0xb6f4c740,
0x639cc5d2,0xc5c45043,0x1364e083,0xd7e0c60b,
0x25ceed82,0xb3cfe57e,0xd2c14987,0xb87881ef,
0xbbd10816,0xfc09136a,0xfe0fc804,0x579861d9,
0xe47b6e7e,0x6e56e6a8,0x63597a8b,0x2bb656e0,
0x49838d6c,0xf87488d2,0x2e6b03c4,0x0e61c93a,
0x2e4849ee,0xdf094559,0x2d3954fb,0x974d0a69,
0x4867f73b,0x03c95ead,0xfd896310,0x2795f124,
0x02efd6d1,0x03c57d47,0x1ab3e0f0,0xd2202afd,
0x43d03737,0x89f007b5,0x0bf7d829,0xb7c2a49a,
0xd0f666a5,0x4aed8f5b,0xc3d3d119,0xed044e05,
0x3b406af3,0x5790d325,0x0539989d,0xc1935088,
0xfeb4ee3e,0x2127afe4,0xd651aa12,0xf077d788,
0x688977b0,0x67733fbe,0xefc7a68c,0x2502dea7,
0x1ce330eb,0xb0376050,0x95554b9c,0xd0840203,
0xec82afae,0xaf94f6f1,0x4374b273,0xacc013b3,
0x172249c5,0x7ae1b263,0x343f79b4,0x6f1f981f,
0xde7c479d,0x5ea4ed0a,0x760f8354,0x7c4faeb4,
0x3f15d4bd,0xf89a302e,0x7bba3719,0x57549d8f,
0x4df422ca,0x371374eb,0x173743fb,0xf30df611,
0x67aebab5,0x6b332e4e,0xcab8fca0,0xc6f23ab0,
0x86886c54,0x8af0e236,0x56b1e8c9,0xd5a29212,
0xa900dfb0,0xa6e529ee,0xeab68dfb,0x2b51e7ab,
0xa0231d5a,0xfd91a619,0xcff548e1,0x0668f879,
0xe027a185,0x4179fa7b,0xf2af98a4,0x94b5f9a6,
0x2c5e2068,0xa65ca76a,0xd2f576e7,0xcb5dd94c,
0xf0bd3681,0xb28b5262,0x500c6c29,0x50d7777e,
0x5c848f2d,0x2296822b,0x514466bd,0xe9d82a73,
0x7fbb8c1d,0xbc7241ef,0xe7f64a5b,0x780948fb,
0x08ab5bb4,0x21c41778,0x6f57db19,0xad26065e,
0xe68274c1,0x7c3e7159,0x47e1038a,0xd5f3cfb9,
0xd18adf03,0xdf1f9d0e,0x624cdc27,0x59e352e3,
0xfcac8850,0xa8e90d6a,0x3cf3f5ad,0xfbf2c729,
0x64f5f296,0x2507066d,0x4979a8c4,0x3066d4af,
0x4506ccb2,0x289877c8,0xcaecb1b7,0xf57e9a44,
0x05012cef,0x4ba23455,0xff936f79,0xbd897b74,
0x756fc386,0xdca2619d,0x87464903,0x3dd29f5d,
0xdec4ef53,0xcfb5b4d2,0xb6c953c1,0x5103207c,
0xa80c5d4d,0xe2558bb2,0x7c864228,0x7e0a8175,
0xf8400ff0,0xf193a5f6,0xa38ff58d,0xdbe49d18,
0xb5223f27,0xea73b3ab,0xbabb4090,0x154574c7,
0xce2ee913,0xc9ce27d8,0xb39f9dfc,0xf38e0d84,
0x346ae61d,0xfb53236b,0x9abaecf5,0xede34182,
0x47197893,0x4f797e25,0xdbf54c99,0x284f15e6,
0x671641d9,0xef848723,0x06eddd31,0xf5df3b10,
0xe5f78eb5,0x155d478e,0x23630e3c,0xdce6b5bf,
0x7e248575,0x2cca1a91,0xdef37e3d,0x1fdedcd6,
0xb586d73d,0xacf0b678,0xbbdcace8,0x391d9804,
0xe9082aa3,0xf1ec8985,0xe16a73db,0xf688959f,
0xe74d22c6,0x459e46e1,0xb0247dda,0x9fd11e05,
0x0b5def2d,0xecf32dc2,0xa384d385,0x510fe884,
0x2d3f5227,0xff850179,0xe313112d,0x7fb97ad5,
0x8ff85e62,0x40690bf2,0x2739fb6e,0x7346da23,
0xac066fb1,0xf6916c7d,0xf4431640,0xa1d99245,
0x7266d171,0xf47e2f3b,0xb0fceeb8,0x4c031b76,
0x3d5f0934,0xb09d4f09,0x6c5b57be,0xea82efe9,
0xc8cf0ab2,0x610e3f7f,0xb485af08,0xa9ed95af,
0x8153a5e3,0x2a8af54e,0xf19373d3,0x59144aaa,
0xa02cfaee,0x5aee05a0,0x8e5730ac,0xbccd5d33,
0x9d05320c,0x1a9e8407,0xe947c8d9,0x00fe3183,
0x5880ac93,0x68fda70d,0x611d7e00,0x986a5dd6,
0x03cee2db,0x96a46f44,0x93158028,0xc4f0b2ff,
0x23b8783e,0xc914b503,0x32fdc2cf,0xad20c2ba,
0x47c73ea8,0x840e0efe,0x6cb4ca6d,0xc94d1934,
0x2ba1b6e2,0xda4428b4,0x644ff43c,0xa1f9ed81,
0xbdde80c8,0xb738efde,0xcd097f06,0xc14c72d6,
0x9ca6cc9a,0x83536f1d,0x38894e1f,0x0acd57f9,
0xd5f1c2cd,0x043f91fe,0x3af792d0,0x0a70c96d,
0x79aecf54,0x5f8596cf,0x33e13d34,0x70dfecc6,
0xeea62734,0x6975223b,0xe583ce5b,0x196c3390,
0xf9ae3c7b,0x4541c644,0x927bcd5e,0x59f9ab76,
0x25fa1cf8,0x664398f2,0x7d814b42,0xaac5533f,
0x2b6120f8,0xa91cd863,0x4b7da0d1,0x9a894f62,
0x5b22bf7d,0x6b762af4,0xd8cdb535,0x98f23dd8,
0xf947261b,0x0102563e,0x0c2458a8,0x5dec7030,
0xbe08e482,0xec1b4c83,0x1ae2fadf,0x40ca332b,
0x288bb8c2,0x2645a069,0x93bf5db1,0x7cb5f150,
0xe0e8c8bc,0xd74ce29b,0xf7f65cca,0x1e0a69bf,
0x07111fac,0x5c66e829,0x3f3b1c40,0xdbcd4176,
0xe141d347,0x24da2e9a,0x69a0268b,0xbbbe01d5,
0xaf9d9fbd,0x7207639c,0x981071fb,0xe0c1757c,
0xfe64fd5f,0x4856eb87,0xbd56a8fd,0x1f919215,
0x26afd1e8,0x5deae9a3,0xa4b25dd8,0x8b8f171d,
0x621b869d,0x16efc050,0x2b8b47f2,0x3c93e9c5,
0x73e5d8be,0xde031aaa,0x78c381a8,0xa53c135c,
0xe8dc64e2,0x41fcd33d,0xf799aca4,0x6a9793fd,
0xecf5f535,0xbd8d85bc,0xa862009b,0x759c20b9,
0xbc245324,0xf86109fa,0x03c0b6ab,0x875f3ba6,
0x4bd1fc10,0x0d85b87a,0xb6ab752e,0xa35a39cb,
0x6dae94f1,0x613bb3bc,0x4df5c36c,0xd32da5f8,
0xbf20c81d,0xe5ec998f,0x69e0f30a,0xfbadf35c,
0x56749b45,0x1819e000,0x3e4385b8,0xaccb194a,
0xc081b1ae,0xa7ec3c2c,0x2a49a670,0xb05148e4,
0x749a730a,0x5a590b2a,0xe0de46f8,0x8aaae429,
0xfc0574ac,0x51c6eaf5,0x2ccedddc,0x77e426ec};


/** ============================================================================
 *   @n@b computeParams_secRateMatch
 *
 *   @b Description
 *   @n Utility function used by WCDMA 3GPP --> BCP Params mapper API 
 *      @a computeParams_hsdpa() to calculate 2nd loop Rate matching params.
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
	Uint32              i;
	
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
 *   @n@b computeParams_hsdpa
 *
 *   @b Description
 *   @n Given a small subset of 3GPP params, this function calculates and maps 
 *      out the params based on 3GPP specs to actual BCP H/w Parameters.
 *
 *   @param[out]  
 *   @n pHsdpaParams    HSDPA parameters calculated for this test.
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
static Void computeParams_hsdpa (BcpTest_HsdpaParams* pHsdpaParams)
{
	Int32  B;
	double temp_double;
	Int32  C;
	Int32  K;
	Int32  Ne;
	Int32  deltaNtti;
	Int32  Nsys;
	Int32  Np1;
	Int32  Np2;
	Int32  einit1_p1;
	Int32  einit1_p2;
	Int32  eplus1_p1;
	Int32  eplus1_p2;
	Int32  eminus1_p1;
	Int32  eminus1_p2;
	Int32  a_p1;
	Int32  a_p2;
	Int32  deltaNi_p1;
	Int32  deltaNi_p2;
	Int32  rmax;
    Int32* tab_s;
	Int32* tab_r;
	Int32* tab_b;
	Int32  Ndata;
	Int32  s;
    Int32  r; 
	Int32  Nedata;
	Int32  Ntsys;
	Int32  Xi_p1;
	Int32  Xi_p2;
	Int32  eplus[3];
	Int32  eminus[3];
	Int32  einit[3];
	Int32  Ncol;
	Int32  Nrow;
	Int32  Nr;
	Int32  Nc;
	Int32  punctureFlag;
	
	/* RV value to s R table 3GPP 25.212 sec 4.6.2.1 table 12 */
	/* RV value                           0  1  2  3  4  5  6  7 */
	const Int32 tab_s_16QAM_64QAM[8]  = {1, 0, 1, 0, 1, 1, 1, 1};
	const Int32 tab_R_16QAM_64QAM[8]  = {0, 0, 1, 1, 0, 0, 0, 1};
	const Int32 tab_b_16QAM_64QAM[8]  = {0, 0, 1, 1, 1, 2, 3, 0};

	/* RV value to s R table 3GPP 25.212 sec 4.6.2.1 table 13 */
	/* RV value             0  1  2  3  4  5  6  7 */
	const Int32 tab_s_QPSK[8]  = {1, 0, 1, 0, 1, 0, 1, 0};
	const Int32 tab_r_QPSK[8]  = {0, 0, 1, 1, 2, 2, 3, 3};
	const Int32 tab_b_QPSK[8]  = {0, 0, 0, 0, 0, 0, 0, 0};//QPSK is transparent

	/* start calculate */
	B = pHsdpaParams->TBsize + 24;

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

	/* the first stage rate matching */
	/* 3GPP 25.212 sec4.5.4.2 */
	Nsys = Ne/3;// systematic stream is transparent always %

	if (Nsys > pHsdpaParams->Nir)
	{
		pHsdpaParams->valid = 0;//this test case is not valid
	}
	else
	{
		pHsdpaParams->valid = 1;//this test case is valid

		deltaNtti = pHsdpaParams->Nir - Ne;

		a_p1 = 2;
		a_p2 = 1;

		if (deltaNtti >= 0) //two parity streams transparent
		{
			Np1  = Ne/3;
			Np2  = Ne/3;

			/* As long as e_init>0, e_minus=0; e_plus=any_value; this is equivalent to transparent (no puncture/no repeat) operation */
			/* set the value based on 25.212 Sec4.2.7.2.1.4 */
			einit1_p1 = Np1;//any value greater than 0
			einit1_p2 = Np2;//any value greater than 0

			eplus1_p1 = a_p1*Np1;//it is not be used actually
			eplus1_p2 = a_p2*Np2;//it is not be used actually

			eminus1_p1 = 0;//must be 0
			eminus1_p2 = 0;//must be 0
		}
		else //two parity streams puncturing
		{ 
			//3GPP 25.212 sec 4.2.7.2.2.3 
			temp_double = (double)deltaNtti/2;
			deltaNi_p1 = (Int32)floor(temp_double);
			deltaNi_p2 = (Int32)ceil (temp_double);
    
			Xi_p1 = Ne/3;
			Xi_p2 = Ne/3;
    
			einit1_p1 = Xi_p1;// innital parity 1 
			einit1_p2 = Xi_p2;//innital parity 2 

			eplus1_p1 = a_p1*Xi_p1;
			eplus1_p2 = a_p2*Xi_p2;

			eminus1_p1 = a_p1*abs(deltaNi_p1);
			eminus1_p2 = a_p2*abs(deltaNi_p2);
    
			Np1  = Ne/3 + deltaNi_p1;
			Np2  = Ne/3 + deltaNi_p2;
		}

		/* the second stage rate matching */
		/* parameters in 3GPP 25.212 section 4.5.4.3 */
		if (pHsdpaParams->modScheme == 0) //QPSK 
		{
			rmax  = 4;
			tab_s = (Int32*)tab_s_QPSK;
			tab_r = (Int32*)tab_r_QPSK;
			tab_b = (Int32*)tab_b_QPSK;
			Ndata = 320;
		}
		else // 16QAM and 64QAM 
		{
			rmax  = 2;
			tab_s = (Int32*)tab_s_16QAM_64QAM;
			tab_r = (Int32*)tab_R_16QAM_64QAM;
			tab_b = (Int32*)tab_b_16QAM_64QAM;
			if (pHsdpaParams->modScheme == 1)//16QAM
				Ndata = 640;
			else//64QAM
				Ndata = 960;
		}

		s = tab_s[pHsdpaParams->RVindex];
		r = tab_r[pHsdpaParams->RVindex];

		/* the number of rate matching output bit  */ 
		Nedata = pHsdpaParams->nPhyCh*3*Ndata;

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

		Nrow = (pHsdpaParams->modScheme+1)*2;
		Ncol = Nedata/Nrow;
		temp_double = (double)Ntsys/Ncol;
		Nr   = (Int32)floor(temp_double);
		Nc   = Ntsys-Nr*Ncol;

		/* output parameters */
		pHsdpaParams->numCodeBks    = C;
		pHsdpaParams->codeBkSizeK   = K;
		pHsdpaParams->numFillerBits = C*K-B;
		pHsdpaParams->Ne            = Ne;
		pHsdpaParams->Nedata        = Nedata;
		pHsdpaParams->einit_sys     = einit[0];
		pHsdpaParams->eplus_sys     = eplus[0];
		pHsdpaParams->eminus_sys    = eminus[0];
		pHsdpaParams->einit1_p1     = einit1_p1;
		pHsdpaParams->eplus1_p1     = eplus1_p1;
		pHsdpaParams->eminus1_p1    = eminus1_p1;
		pHsdpaParams->einit1_p2     = einit1_p2;
		pHsdpaParams->eplus1_p2     = eplus1_p2;
		pHsdpaParams->eminus1_p2    = eminus1_p2;
		pHsdpaParams->einit_p1      = einit[1];
		pHsdpaParams->eplus_p1      = eplus[1]; 
		pHsdpaParams->eminus_p1     = eminus[1]; 
		pHsdpaParams->einit_p2      = einit[2]; 
		pHsdpaParams->eplus_p2      = eplus[2]; 
		pHsdpaParams->eminus_p2     = eminus[2]; 
		pHsdpaParams->Ncol          = Ncol;
		pHsdpaParams->Nrow          = Nrow;
		pHsdpaParams->Nr            = Nr;
		pHsdpaParams->Nc            = Nc;
		pHsdpaParams->punctureFlag  = punctureFlag;
		pHsdpaParams->b             = tab_b[pHsdpaParams->RVindex];
	}

    return;
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
 *   @n pHsdpaParams    HSDPA parameters calculated for this test.
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
static Void prepare_wcdma_enchdr_cfg
(
    Bcp_EncHdrCfg*              pEncHdrCfg, 
    BcpTest_HsdpaParams*        pHsdpaParams,
    UInt32                      turboFlag,
    UInt32                      codeRateFlag
)
{
    UInt32                      i;        

    /* Setup the Encoder header as per inputs passed. */        
    pEncHdrCfg->local_hdr_len                   =   1;    
    pEncHdrCfg->turbo_conv_sel                  =   turboFlag;

    if (pHsdpaParams->numCodeBks)
    {
        pEncHdrCfg->blockCfg[0].block_size      =   pHsdpaParams->codeBkSizeK;
        pEncHdrCfg->blockCfg[0].num_code_blks   =   pHsdpaParams->numCodeBks;
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
 *   @param[in]  
 *   @n pHsdpaParams    HSDPA parameters calculated for this test.
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
static Void prepare_wcdma_rmhdr_cfg
(
    Bcp_RmHdr_xCdmaCfg*         pRmHdrCfg,
    UInt8                       xCdmaChanType,
    BcpTest_HsdpaParams*        pHsdpaParams
)
{
    UInt32                      i;        

    /* Setup the xCDMA Rate Modulation header as per inputs passed. */        
    pRmHdrCfg->local_hdr_len        =   20;
    pRmHdrCfg->input_order          =   1;
    pRmHdrCfg->half_rate            =   0;
    if (xCdmaChanType == WCDMAFDD_HSDPA)
    {
        pRmHdrCfg->collect_cols     =   pHsdpaParams->Ncol;
        pRmHdrCfg->collect_rows     =   pHsdpaParams->Nrow;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC)
    {
        pRmHdrCfg->collect_cols     =   0;
        pRmHdrCfg->collect_rows     =   0;
    }
    pRmHdrCfg->num_scram            =   0;

    /* Setup Systematic channel */
    pRmHdrCfg->sys0_len             =   pHsdpaParams->Ne/3;
    pRmHdrCfg->sys0_init2           =   pHsdpaParams->einit_sys;
    pRmHdrCfg->sys0_minus2          =   pHsdpaParams->eminus_sys;
    pRmHdrCfg->sys0_plus2           =   pHsdpaParams->eplus_sys;
    pRmHdrCfg->sys0_alpha           =   0;
    pRmHdrCfg->sys0_beta            =   0;
    pRmHdrCfg->sys0_puncture        =   pHsdpaParams->punctureFlag;

    if (xCdmaChanType == WCDMAFDD_HSDPA)
        pRmHdrCfg->sys0_turbo       =   3;
    else
        pRmHdrCfg->sys0_turbo       =   2;

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
    if (xCdmaChanType == WCDMAFDD_HSDPA)
    {
        pRmHdrCfg->p0_par1_len      =   pHsdpaParams->Ne/3;
        pRmHdrCfg->p0_par1_init1    =   pHsdpaParams->einit1_p1;
        pRmHdrCfg->p0_par1_minus1   =   pHsdpaParams->eminus1_p1;
        pRmHdrCfg->p0_par1_plus1    =   pHsdpaParams->eplus1_p1;
        pRmHdrCfg->p0_par1_init2    =   pHsdpaParams->einit_p1;
        pRmHdrCfg->p0_par1_minus2   =   pHsdpaParams->eminus_p1;
        pRmHdrCfg->p0_par1_plus2    =   pHsdpaParams->eplus_p1;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC)
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
    if (xCdmaChanType == WCDMAFDD_HSDPA)
    {
        pRmHdrCfg->p0_par2_len      =   pHsdpaParams->Ne/3;
        pRmHdrCfg->p0_par2_init1    =   pHsdpaParams->einit1_p2;
        pRmHdrCfg->p0_par2_minus1   =   pHsdpaParams->eminus1_p2;
        pRmHdrCfg->p0_par2_plus1    =   pHsdpaParams->eplus1_p2;
        pRmHdrCfg->p0_par2_init2    =   pHsdpaParams->einit_p2;
        pRmHdrCfg->p0_par2_minus2   =   pHsdpaParams->eminus_p2;
        pRmHdrCfg->p0_par2_plus2    =   pHsdpaParams->eplus_p2;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC)
    {
        pRmHdrCfg->p0_par2_len      =   0;
        pRmHdrCfg->p0_par2_init1    =   0;
        pRmHdrCfg->p0_par2_minus1   =   0;
        pRmHdrCfg->p0_par2_plus1    =   0;
        pRmHdrCfg->p0_par2_init2    =   0;
        pRmHdrCfg->p0_par2_minus2   =   0;
        pRmHdrCfg->p0_par2_plus2    =   0;
    }

    if (xCdmaChanType == WCDMAFDD_HSDPA)
    {
        pRmHdrCfg->p1_par1_len      =   0;
        pRmHdrCfg->p1_par1_init2    =   0;
        pRmHdrCfg->p1_par1_minus2   =   0;
        pRmHdrCfg->p1_par1_plus2    =   0;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC)
    {
        pRmHdrCfg->p1_par1_len      =   pHsdpaParams->Ne/3;
        pRmHdrCfg->p1_par1_init2    =   pHsdpaParams->einit_p1;
        pRmHdrCfg->p1_par1_minus2   =   pHsdpaParams->eminus_p1;
        pRmHdrCfg->p1_par1_plus2    =   pHsdpaParams->eplus_p1;
    }

    if (xCdmaChanType == WCDMAFDD_HSDPA)
    {
        pRmHdrCfg->p1_par2_len      =   0;
        pRmHdrCfg->p1_par2_init2    =   0;
        pRmHdrCfg->p1_par2_minus2   =   0;
        pRmHdrCfg->p1_par2_plus2    =   0;
    }
    else if (xCdmaChanType == WCDMAFDD_HSUPAPIC)
    {
        pRmHdrCfg->p1_par2_len      =   pHsdpaParams->Ne/3;
        pRmHdrCfg->p1_par2_init2    =   pHsdpaParams->einit_p2;
        pRmHdrCfg->p1_par2_minus2   =   pHsdpaParams->eminus_p2;
        pRmHdrCfg->p1_par2_plus2    =   pHsdpaParams->eplus_p2;
    }

    return;        
}

/** ============================================================================
 *   @n@b prepare_inthdr_cfg
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
 *   @param[in]  
 *   @n pHsdpaParams    HSDPA parameters calculated for this test.
 * 
 *   @return        
 *   @n None
 * =============================================================================
 */
static Void prepare_inthdr_cfg 
(
    Bcp_IntHdrCfg*              pIntHdrCfg,
    UInt8                       xCdmaChanType,
    BcpTest_HsdpaParams*        pHsdpaParams
)
{
    UInt32                      i;        

    /* Setup the interleaver header as per inputs passed */
    pIntHdrCfg->local_hdr_len       =   3;
    pIntHdrCfg->num_constellation   =   pHsdpaParams->b;
    pIntHdrCfg->flag_half_rate      =   0;

    if (xCdmaChanType == WCDMAFDD_HSDPA)
        pIntHdrCfg->num_data_format_out =   7 - pHsdpaParams->TACformatFlag * 2;
    else
        pIntHdrCfg->num_data_format_out =   0;
    pIntHdrCfg->num_add_dtx         =   0;
    
    for (i = 0; i < 6; i ++)
    {
        if (i == 0)
            pIntHdrCfg->tblCfg[i].num_data_format_in    =   (pHsdpaParams->modScheme + 1) * 2;
        else
            pIntHdrCfg->tblCfg[i].num_data_format_in    =   0;
    }

    if (xCdmaChanType == TDSCDMA_REL99DL)
    {
        pIntHdrCfg->flag_in_order                       =   1;  // input is from encoder
    }
    else
    {
        pIntHdrCfg->flag_in_order                       =   0;  // input is from rate matching engine
    }
	
    if (xCdmaChanType == WCDMAFDD_HSDPA)
    {
        pIntHdrCfg->num_int_ways                        =   30;
    
        for (i = 0; i < 6; i ++)
        {
            if (i == 0)
                pIntHdrCfg->tblCfg[i].num_r2_length     =   32;
            else
                pIntHdrCfg->tblCfg[i].num_r2_length     =   0;
            pIntHdrCfg->tblCfg[i].num_dummy             =   0;
        }
    }
    else
    {
        pIntHdrCfg->num_int_ways                        =   0;
    
        for (i = 0; i < 6; i ++)
        {
            pIntHdrCfg->tblCfg[i].num_r2_length         =   0;
            pIntHdrCfg->tblCfg[i].num_dummy             =   0;
        }
    }

    pIntHdrCfg->num_frame_count                     =   pHsdpaParams->nPhyCh;

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
    /* To be able to receive BCP processed packets from BCP on 
     * remote device back on this device without any DSP intervention on
     * remote device, we'll have to send the return path (BCP --> this device via SRIO) 
     * in Tx packet as PS Info. 
     *
     * PS Info of BCP output packet would contain SRIO return path. The SRIO return path
     * information is 8 bytes long.
     */
    pTmHdrCfg->ps_data_size     =   2;
    pTmHdrCfg->info_data_size   =   0;

    return;
}

/** ============================================================================
 *   @n@b add_test_config_data
 *
 *   @b Description
 *   @n Given a data buffer, this API sets up the packet contents for the test.
 *      It adds all the BCP configuration params and the payload to the data buffer.
 *      On success, returns the number of bytes of configuration and payload data
 *      added to the data buffer passed. 
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
    BcpTest_HsdpaParams         hsdpaParams;
    UInt32                      dataBufferLen, tmpLen, tmpVal;
    UInt8                       wcdmaChanType;
    Bcp_RadioStd                radioStd;
    Bcp_GlobalHdrCfg            bcpGlblHdrCfg;
    Bcp_CrcHdrCfg               crcHdrCfg;
    Bcp_EncHdrCfg               encHdrCfg;
    Bcp_RmHdr_xCdmaCfg          wcdmaRmHdrCfg;
    Bcp_IntHdrCfg               intHdrCfg;
    Bcp_TmHdrCfg                tmHdrCfg;
    Srio_Type9AddrInfo*         ptr_addr9Info;
    Srio_Type9AddrInfo*         ptr_localAddr9Info;
    Srio_SockAddrInfo           from;
    Srio_SockAddrInfo           to;

#ifdef BCP_TEST_DEBUG
    Bcp_osalLog ("Reading test configuration ... \n");
#endif

    /* Get the HSDPA input test vector params */
    hsdpaParams.TBsize      =   testVectorHsdpa[0][0]; 
    hsdpaParams.modScheme   =   testVectorHsdpa[0][1];/* 0:QPSK, 1:16QAM, 2:64QAM */ 
    hsdpaParams.Nir         =   testVectorHsdpa[0][2];  
    hsdpaParams.nPhyCh      =   testVectorHsdpa[0][3]; 
    hsdpaParams.RVindex     =   testVectorHsdpa[0][4]; 
    hsdpaParams.TACformatFlag = 1;//1 TAC format, 0 unpacked format
    
    /* Generate rest of the parameters */
    computeParams_hsdpa ((BcpTest_HsdpaParams *) &hsdpaParams);

    radioStd        =   Bcp_RadioStd_WCDMA_R99; 
    wcdmaChanType   =   WCDMAFDD_HSDPA;

    /* Start adding BCP Packet headers based on the test configuration we read. */

    /* Initialize our data buffer length running counter */
    dataBufferLen   =   0;
    tmpLen			=	0;

    /* Header 1: Global Header */
    bcpGlblHdrCfg.pkt_type          =   Bcp_PacketType_Normal;
    bcpGlblHdrCfg.flush             =   0;
    bcpGlblHdrCfg.drop              =   0;
    bcpGlblHdrCfg.halt              =   0;
    bcpGlblHdrCfg.radio_standard    =   radioStd;
    bcpGlblHdrCfg.hdr_end_ptr       =   37; // 2 + 4 + 2 + 21 + 4 + 2 + 2 words PS
    bcpGlblHdrCfg.flow_id           =   RX_FLOW_ID;
    bcpGlblHdrCfg.destn_tag         =   0xDEAD;
    if (Bcp_addGlobalHeader (&bcpGlblHdrCfg, pDataBuffer, &tmpLen) < 0)
    {
#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Failed to add Global Header to packet \n");
#endif
        return -1;
    }
    pDataBuffer 	+=	tmpLen;
    dataBufferLen	+=	tmpLen;
    tmpLen			=	0;

    /* Header 2: CRC Header */
    prepare_crchdr_cfg (&crcHdrCfg, radioStd, hsdpaParams.TBsize, hsdpaParams.numFillerBits, 0, 0, 1, 24, 1, 1, 0, &tmpVal);
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
    prepare_wcdma_enchdr_cfg (&encHdrCfg, &hsdpaParams, 1, 0);
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

    /* Header 4: Rate matching header */
    prepare_wcdma_rmhdr_cfg (&wcdmaRmHdrCfg, wcdmaChanType, &hsdpaParams);
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

    /* Header 5: Interleaver header */
    prepare_inthdr_cfg (&intHdrCfg, wcdmaChanType, &hsdpaParams);
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

    /* Add SRIO return path for BCP results in the PS info field of Tx packets.
     * The return path is used by BCP on TN to return BCP output packets back
     * to this device.
     *
     * Return path would be: From BCP --> to this device
     */
    from.type9.tt       = TRUE;                 /* We are using 16 bit identifiers. */
    from.type9.id       = DEVICE_ID_BCP_16BIT;  /* BCP Source Identifier    */
    from.type9.streamId = 1;                    /* BCP Stream Identifier    */
    from.type9.cos      = CSL_chipReadReg (CSL_CHIP_DNUM);  /* Class of Service */
   
    to.type9.tt         = TRUE;                 /* We are using 16 bit identifiers. */
    to.type9.id         = DEVICE_ID1_16BIT;     /* Current device Source Identifier */
    to.type9.streamId   = 0;                    /* Current device Stream Identifier */
    to.type9.cos        = CSL_chipReadReg (CSL_CHIP_DNUM);  /* Class of Service     */
    
    ptr_addr9Info = (Srio_Type9AddrInfo*)&to;
 
    /* Get the address information to which the socket is bound. */
    ptr_localAddr9Info = (Srio_Type9AddrInfo *)&from;

    /* Create the SRIO Protocol Specific Information. */
    CSL_SRIO_SetType9PSInfo((UInt32 *) (pDataBuffer), ptr_addr9Info->id, ptr_localAddr9Info->id, 
                             ptr_addr9Info->cos, 0x1, ptr_addr9Info->tt, ptr_addr9Info->streamId);
    pDataBuffer 	+=	8;
    dataBufferLen	+=	8;
	                             
	/* Add padding to align data start on 128 bits */    
    pDataBuffer 	+=	12;
    dataBufferLen	+=	12;
    
    /* Finally add the data to the packet in the same order as the headers */
    if ((pTestCfgFile = fopen("..\\..\\wcdma\\bcp_fdd_hsdpa_input.dat","r")) == NULL)
    {
#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("Cannot open data input file:  ..\..\wcdma\bcp_fdd_hsdpa_input.dat\n");
#endif
        return -1;
    }
    read_data_from_file (pTestCfgFile, pDataBuffer, &dataBufferLen);
    fclose (pTestCfgFile);

    /* Successfully read the test configuration */        
    return dataBufferLen;
}

/** ============================================================================
 *   @n@b my_freeRecvBuffer
 *
 *   @b Description
 *   @n SRIO receive buffer cleanup API 
 *
 *   @param[in]  
 *   @n hSrioBuf    SRIO driver buffer handle
 *
 *   @return        None
 * =============================================================================
 */
Void my_freeRecvBuffer (Srio_DrvBuffer hSrioBuf)
{
   	srio_freeRecvBuffer (NULL, hSrioBuf); 
	return;
}


/** ============================================================================
 *   @n@b run_bcp_client
 *
 *   @b Description
 *   @n Sets up the parameters required to run an WCDMA Downlink (DL) test case 
 *      using the BCP on remote device. Uses SRIO to send requests and
 *      receive results from remote BCP.
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
Void run_bcp_client (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblFDQ)
{
    Bcp_TxHandle        hTx = NULL;
    Bcp_RxHandle        hRx = NULL;
    Qmss_QueueHnd       hSrioTxFDQ;
    Qmss_QueueHnd       hSrioRxFDQ;
    UInt32              dataBufferLen;
    Int32               dataBufferLenUsed;
    Cppi_Desc*          pCppiDesc;
    UInt8*              pDataBuffer;
    Srio_SockAddrInfo   to;
    Srio_DrvConfig      srioDrvCfg;
    UInt8               rxFlowId, rxSrcId;
    UInt32              i, rxDataBufferLen, rxPsInfoLen, rxDataTotalLen, testFail = 0;
    UInt16              rxDestnTag;
    UInt8*              pRxDataBuffer;
    UInt8*              pRxPsInfo;
    Cppi_DescTag        cppiTagInfo;
    Void*               hTmp;
    Qmss_Queue          tmpQ;

    /* Setup BCP Rx configuration. 
     *  -   Nothing to setup here since BCP flows are not accessible
     *      on remote device.
     *  -   Just setup transport Rx endpoint configuration instead
     *      to receive all output from BCP onto a Rx queue.
     */

    /* Setup BCP Transport Rx endpoint configuration.
     *
     * The transport we are using here is SRIO. 
     *  -   Setup a Rx FDQ for SRIO on this device 
     *  -   Setup a SRIO Rx flow to receive all output from BCP via SRIO and redirect
     *      them onto a general purpose Rx queue. The application will poll
     *      on this Rx queue for results, and will validate them.
     */
    if (allocate_fdq (hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, 0, &hSrioRxFDQ, NULL) < 0)
    {
        Bcp_osalLog ("Error opening Rx FDQ for SRIO \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("SRIO Rx FDQ %d successfully setup with %d descriptors\n", hSrioRxFDQ, RX_NUM_DESC);
    }
    tmpQ = Qmss_getQueueNumber(hSrioRxFDQ);
    
    /* Initialize the SRIO Driver Configuration. */
    memset ((Void *)&srioDrvCfg, 0, sizeof(Srio_DrvConfig));

    /* Use application managed flow configuration */
    srioDrvCfg.bAppManagedConfig                        =   TRUE;
    srioDrvCfg.u.appManagedCfg.bIsRxFlowCfgValid        =   TRUE;

    /* Configure the Receive Flow */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.flowIdNum          = -1;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_qnum       = RX_Q_NUM; /* Receive BCP output on Rx Q */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_qmgr       = 0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_sop_offset      = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_ps_location     = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_desc_type       = 0x1; /* Host Descriptor. */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_error_handling  = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_psinfo_present  = 0x0; /* No PS Information */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_einfo_present   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo     = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi     = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo      = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi      = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo_sel = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi_sel = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo_sel  = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi_sel  = 0x0;

    /* Disable Receive size thresholds. */
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0_en = 0x0;
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1_en = 0x0;
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2_en = 0x0;

	/* Use the Application Receive Free Queue for picking all descriptors. */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qnum       = tmpQ.qNum;  
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qmgr       = tmpQ.qMgr;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qnum       = tmpQ.qNum;  
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qmgr       = tmpQ.qMgr;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qnum       = tmpQ.qNum;  
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qmgr       = tmpQ.qMgr;

    /* Use the Receive Queue for picking the SOP packet also. */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qnum   = tmpQ.qNum;  
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qmgr   = tmpQ.qMgr;

    /* There are no size thresholds configured. */
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0    = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1    = 0x0;
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2    = 0x0;

    /* The other threshold queues do not need to be configured */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qnum   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qmgr   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qnum   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qmgr   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qnum   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qmgr   = 0x0;

    /* Polling Mode: So dont program the accumulator. */
	srioDrvCfg.u.appManagedCfg.bIsAccumlatorCfgValid        = 0;

    /* Populate the rest of the configuration. */
    srioDrvCfg.u.appManagedCfg.rawRxFreeDrvBuffer           = my_freeRecvBuffer;

    /* No BCP Rx configuration. Setup only BCP Rx endpoint */
    if ((hRx = Bcp_rxOpen (hBcp, NULL, &srioDrvCfg)) == NULL)
    {
        Bcp_osalLog ("BCP Rx Open failed \n");
		goto cleanup_and_return;
    }
    else
    {
        Bcp_osalLog ("SRIO Flow opened to send data to RxQ: %d \n", RX_Q_NUM);
    }

    /* Setup BCP Tx configuration.
     *  -   BCP is not local to the device. Hence cannot configure BCP Tx
     *      queue directly. So no need for setting up BCP Tx configuration.
     *  -   Setup only BCP Tx transport tunnel to be able to send packets to
     *      BCP from this device via SRIO
     */

    /* Setup BCP Tx endpoint configuration 
     * -    Open SRIO Tx queue. We'll use it to send BCP requests to
     *      BCP on TN.
     */
    /* Initialize the SRIO Driver Configuration. */
    memset ((Void *)&srioDrvCfg, 0, sizeof(Srio_DrvConfig));

    /* Enable application manged configuration. Disable Rx flow
     * configuration, since all we need here is SRIO Tx queue to be
     * setup.
     */
    srioDrvCfg.bAppManagedConfig                        =   TRUE;
    srioDrvCfg.u.appManagedCfg.bIsRxFlowCfgValid        =   FALSE;
	srioDrvCfg.u.appManagedCfg.bIsAccumlatorCfgValid    =   FALSE;
    if ((hTx = Bcp_txOpen (hBcp, NULL, &srioDrvCfg)) == NULL)
    {
        Bcp_osalLog ("Error opening BCP Tx object \n");
        return;
    }

    /* Allocate a Tx FDQ for sending BCP requests. */
    if (allocate_fdq (hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, 0, &hSrioTxFDQ, NULL) < 0)
    {
        Bcp_osalLog ("Error opening Tx FDQ \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("SRIO Tx FDQ %d successfully setup with %d descriptors\n", hSrioTxFDQ, TX_NUM_DESC);
        Bcp_osalLog ("\nReady to send packets to BCP over SRIO ... \n");
    }

    /* Build and Send a packet with WCDMA DL parameters for BCP Processing */
    if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hSrioTxFDQ)) == NULL)
    {
        Bcp_osalLog ("Out of Tx FDs! \n");
		goto cleanup_and_return;
    }
    pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
    Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        
    /* Initialize data buffer we add BCP request data */
	memset (pDataBuffer, 0, dataBufferLen); 

    /* Read test configuration and build BCP request packet */
    if ((dataBufferLenUsed = add_test_config_data (hBcp, pDataBuffer)) <= 0)
    {
        Bcp_osalLog ("Error reading test vectors/populating BCP packet \n");
        goto cleanup_and_return;
    }

    /* Populate the destination information where the BCP request packet must be sent to. */
    to.type9.tt         = TRUE;                 /* We are using 16 bit identifiers.          */
    to.type9.id         = DEVICE_ID_BCP_16BIT;  /* BCP device Identifier    */
    to.type9.streamId   = 1;                    /* BCP device Stream Identifier */
    to.type9.cos        = CSL_chipReadReg (CSL_CHIP_DNUM);  /* Class of Service */

   	Bcp_osalLog ("Sending a packet of len %d to BCP over SRIO ...\n", dataBufferLenUsed);

    /* Setup the data length in the descriptor */
    Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLenUsed);
    Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, dataBufferLenUsed);
    
    Bcp_send (hTx, pCppiDesc, BCP_TEST_SIZE_HOST_DESC, &to);
    
    Bcp_osalLog ("Waiting on output from BCP ...\n");

    /* Wait on data to be received from BCP and validate it */
    while (Qmss_getQueueEntryCount (RX_Q_NUM) == 0) { Task_sleep (2); }

    Bcp_osalLog ("Got %d output packet(s) from BCP \n", Qmss_getQueueEntryCount (RX_Q_NUM));

    Bcp_osalLog ("Validating BCP output ... \n");

    i = 0;
    while (Qmss_getQueueEntryCount (RX_Q_NUM) > 0)
    {
        /* Data could arrive scattered across multiple linked descriptors.
         * Collect data from all linked descriptors and validate it.
         */
        rxDataTotalLen  =   0;

        if ((pCppiDesc = Qmss_queuePop (RX_Q_NUM)) != NULL)
        {
            pCppiDesc = (Cppi_Desc *)QMSS_DESC_PTR (pCppiDesc);                

            /* Get the source and destination tag information */
            cppiTagInfo         =   Cppi_getTag (Cppi_DescType_HOST, pCppiDesc);
            rxSrcId             =   cppiTagInfo.srcTagHi;
            rxFlowId            =   cppiTagInfo.srcTagLo;
            rxDestnTag          =   ((cppiTagInfo.destTagHi << 8) | (cppiTagInfo.destTagLo));

            /* Get Data buffer containing the output and its length */
            Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pRxDataBuffer, &rxDataBufferLen);

            /* Get PS data buffer and its length from the descriptor */
            Cppi_getPSData (Cppi_DescType_HOST, (Cppi_PSLoc)0x0, pCppiDesc, &pRxPsInfo, &rxPsInfoLen);
        }

        /* Validate BCP output against expected output */
        if (validate_rxdata ((UInt8 *)wcdma_dl_out_packet_1, 
                WCDMA_DL_OUTPUT_PKT_1_WRD_SIZE*4, 
                pRxDataBuffer, 
                rxDataBufferLen, 
                rxDataTotalLen) != 0)
                testFail ++;

        rxDataTotalLen  +=  rxDataBufferLen;

        /* Check if there are any descriptors linked to this Rx desc */
        while (pCppiDesc)
        {
            /* Save original Rx desc handle. */
            hTmp = pCppiDesc;

            if ((pCppiDesc = Cppi_getNextBD (Cppi_getDescType (pCppiDesc), pCppiDesc)))
            {
                pCppiDesc = (Cppi_Desc *)QMSS_DESC_PTR (pCppiDesc);                

                /* Get the source and destination tag information */
                cppiTagInfo         =   Cppi_getTag (Cppi_DescType_HOST, pCppiDesc);
                rxSrcId             =   cppiTagInfo.srcTagHi;
                rxFlowId            =   cppiTagInfo.srcTagLo;
                rxDestnTag          =   ((cppiTagInfo.destTagHi << 8) | (cppiTagInfo.destTagLo));

                /* Get Data buffer containing the output and its length */
                Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pRxDataBuffer, &rxDataBufferLen);

                /* Get PS data buffer and its length from the descriptor */
                Cppi_getPSData (Cppi_DescType_HOST, (Cppi_PSLoc)0x0, pCppiDesc, &pRxPsInfo, &rxPsInfoLen);
       
                if (validate_rxdata ((UInt8 *)wcdma_dl_out_packet_1, 
                                    WCDMA_DL_OUTPUT_PKT_1_WRD_SIZE*4, 
                                    pRxDataBuffer, 
                                    rxDataBufferLen, 
                                    rxDataTotalLen) != 0)
                    testFail ++;                        

                rxDataTotalLen  +=  rxDataBufferLen;
            }
        
            Bcp_rxFreeRecvBuffer (hRx, hTmp, BCP_TEST_SIZE_HOST_DESC);
        }

        Bcp_osalLog ("[Pkt %d]: Total Len: %d DestnTag: 0x%x SrcId: 0x%x FlowId: 0x%x\n", i, rxDataTotalLen, rxDestnTag, rxSrcId, rxFlowId);

        if (rxDataTotalLen != WCDMA_DL_OUTPUT_PKT_1_WRD_SIZE * 4)
        {
            testFail ++;
        }

        /* Increment number of packets received. */
        i ++;
    }

    if (testFail > 0)
    {
        Bcp_osalLog ("WCDMA DL Test:    FAILED\n");                
        totalNumTestsFail ++;
    }
    else
    {
        Bcp_osalLog ("WCDMA DL Test:    PASS\n");                
        totalNumTestsPass ++;
    }
    
    Bcp_osalLog ("BCP Remote transfer successful!\n");
cleanup_and_return:
    if (hTx)
        Bcp_txClose (hTx);

    deallocate_fdq (hSrioTxFDQ, hGlblFDQ, TX_NUM_DESC, TX_DATA_BUFFER_SIZE, NULL);

    if (hRx)
        Bcp_rxClose (hRx);            

    deallocate_fdq (hSrioRxFDQ, hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, NULL);

    return;
}


/** ============================================================================
 *   @n@b run_bcp_server
 *
 *   @b Description
 *   @n Sets up the BCP/SRIO on BCP local device to be able to receive and process
 *      BCP requests. All processed packets from BCP (BCP output) are routed back
 *      to a remote device based on return route programmed on the BCP request packet.
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
Void run_bcp_server (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblFDQ)
{
    Qmss_QueueHnd       hSrioRxFDQ;
    Bcp_TxCfg           txCfg;
    Bcp_TxHandle        hTx = NULL;
    Bcp_RxCfg           rxCfg;
    Bcp_RxHandle        hRx = NULL;
    Qmss_QueueHnd       hRxFDQ;
    Srio_DrvConfig      srioDrvCfg;
    Qmss_Queue          tmpQ;

    /* Setup BCP Rx configuration:
     *  -   Open Rx queue on to which BCP results are to be forwarded to
     *  -   If running on simulator, we route all BCP output to a general
     *      purpose Rx queue (due to Sim packet type assignment bug) and
     *      then put the correct packet type on the packet and send it out
     *      via SRIO.
     *  -   If running on Si, we route all BCP output directly to SRIO to
     *      be sent out to a remote device.
     *  -   Setup a BCP Rx FDQ, Rx flow to receive data
     *  -   No need to setup BCP transport Rx endpoint configuration since
     *      BCP is local to device and we can receive data from BCP directly.
     */
    if (allocate_fdq (hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, 0, &hRxFDQ, NULL) < 0)
    {
        Bcp_osalLog ("Error opening Rx FDQ \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("BCP Rx FDQ %d successfully setup with %d descriptors\n", hRxFDQ, RX_NUM_DESC);
    }
    tmpQ = Qmss_getQueueNumber(hRxFDQ);
    rxCfg.rxQNum                        =   SRIO_TX_QUEUE(0);   // Send BCP output to SRIO Tx queue to be sent out to Ny.
    rxCfg.bUseInterrupts                =   0;              // Use polling 
    memset (&rxCfg.flowCfg, 0, sizeof(Cppi_RxFlowCfg));
    rxCfg.flowCfg.flowIdNum             =   RX_FLOW_ID;  
    rxCfg.flowCfg.rx_dest_qmgr          =   0;    
    rxCfg.flowCfg.rx_dest_qnum          =   rxCfg.rxQNum;  
    rxCfg.flowCfg.rx_desc_type          =   Cppi_DescType_HOST; 
    rxCfg.flowCfg.rx_ps_location        =   Cppi_PSLoc_PS_IN_DESC;  // SRIO header to be added as PS to be present in desc
    rxCfg.flowCfg.rx_psinfo_present     =   1;              //  Enable PS info. Will be used for sending SRIO header to SRIO
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
    rxCfg.tmFlowCfg.pkt_type            =   30;     // Set Rx packet type to be 30 for SRIO Type 9
    rxCfg.tmFlowCfg.dsp_int_sel         =   0;      // Interrupt core 0
    rxCfg.tmFlowCfg.format_out          =   Bcp_DataFormat_NoChange;
    rxCfg.tmFlowCfg.qfifo_out           =   RX_FLOW_ID;      // Use Rx QFIFO 3
    rxCfg.tmFlowCfg.ps_flags            =   0;      // No PS flags for now

    /* Setup BCP Rx. No transport setup required since BCP is local to device. */
    if ((hRx = Bcp_rxOpen (hBcp, &rxCfg, NULL)) == NULL)
    {
        Bcp_osalLog ("BCP Rx Open failed \n");
		goto cleanup_and_return;
    }
    else
    {
        Bcp_osalLog ("BCP Flow %d opened to send data to RxQ: %d \n", RX_FLOW_ID, rxCfg.rxQNum);
    }

    /* Setup BCP Tx side:
     *  -   Open BCP Tx queue for remote BCP request processing.
     *  -   Setup BCP transport Tx endpoint configuration.
     */
    txCfg.txQNum    =   Bcp_QueueId_0;

    /* Setup BCP Transport Tx endpoint configuration.
     *
     * The transport we are using here is SRIO. 
     *  -   Setup SRIO to redirect all packets it receives to BCP Tx queue we are using.
     *  -   Setup a Rx FDQ for SRIO on this device 
     *  -   Setup a SRIO Rx flow to redirect all traffic received to BCP Tx queue
     */
    if (allocate_fdq (hGlblFDQ, RX_NUM_DESC, TX_DATA_BUFFER_SIZE, 0, &hSrioRxFDQ, NULL) < 0)
    {
        Bcp_osalLog ("Error opening Rx FDQ for SRIO \n");            
        return;
    }
    else
    {
        Bcp_osalLog ("SRIO Rx FDQ %d successfully setup with %d descriptors\n", hSrioRxFDQ, RX_NUM_DESC);
    }
    tmpQ = Qmss_getQueueNumber(hSrioRxFDQ);
    /* Initialize the SRIO Driver Configuration. */
    memset ((Void *)&srioDrvCfg, 0, sizeof(Srio_DrvConfig));

    /* Setup the SRIO Application Managed Configuration. */
    srioDrvCfg.bAppManagedConfig                            = TRUE;

    /* Enable flow configuration  */
    srioDrvCfg.u.appManagedCfg.bIsRxFlowCfgValid            = 1;

    /* Configure the Receive Flow */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.flowIdNum          = -1;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_qnum       = BCP_TX_QUEUE(txCfg.txQNum); // Send SRIO packets to BCP
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_qmgr       = 0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_sop_offset      = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_ps_location     = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_desc_type       = 0x1; /* Host Descriptor. */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_error_handling  = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_psinfo_present  = 0x0; /* No PS Information */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_einfo_present   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo     = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi     = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo      = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi      = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo_sel = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi_sel = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo_sel  = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi_sel  = 0x0;

    /* Disable Receive size thresholds. */
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0_en = 0x0;
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1_en = 0x0;
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2_en = 0x0;

	/* Use the Application Receive Free Queue for picking all descriptors. */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qnum       = tmpQ.qNum; 
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qmgr       = tmpQ.qMgr;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qnum       = tmpQ.qNum; 
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qmgr       = tmpQ.qMgr;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qnum       = tmpQ.qNum; 
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qmgr       = tmpQ.qMgr;

    /* Use the Receive Queue for picking the SOP packet also. */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qnum   = tmpQ.qNum; 
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qmgr   = tmpQ.qMgr;

    /* There are no size thresholds configured. */
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0    = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1    = 0x0;
    srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2    = 0x0;

    /* The other threshold queues do not need to be configured */
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qnum   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qmgr   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qnum   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qmgr   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qnum   = 0x0;
	srioDrvCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qmgr   = 0x0;

    /* Polling Mode: So dont program the accumulator. */
	srioDrvCfg.u.appManagedCfg.bIsAccumlatorCfgValid        = 0;

    /* Populate the rest of the configuration. */
    srioDrvCfg.u.appManagedCfg.rawRxFreeDrvBuffer           = my_freeRecvBuffer;

    /* Setup BCP Tx queue. Setup also transport Tx endpoint configuration to forward all packets received on SRIO to BCP Tx queue.*/
    if ((hTx = Bcp_txOpen (hBcp, &txCfg, &srioDrvCfg)) == NULL)
    {
        Bcp_osalLog ("BCP Tx Open failed \n");
        return;
    }
    else
    {
        Bcp_osalLog ("BCP Tx Queue %d succesfully setup. Waiting for BCP requests ...\n", txCfg.txQNum);
    }

    /* Do nothing. Wait forever for BCP requests. */
    while (1)
    {
        Task_sleep (20);
    }

cleanup_and_return:
    if (hRx)
        Bcp_rxClose (hRx);

    deallocate_fdq (hRxFDQ, hGlblFDQ, RX_NUM_DESC, RX_DATA_BUFFER_SIZE, NULL);

    if (hTx)
        Bcp_txClose (hTx);

    deallocate_fdq (hSrioRxFDQ, hGlblFDQ, RX_NUM_DESC, TX_DATA_BUFFER_SIZE, NULL);
	
    return;
}
