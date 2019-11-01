

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <math.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include "cslUtils.h"

#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif


#define 	MAX_SLOT				400
#define 	SLOT_NUM_FIRST_PUSH		40
#define 	SLOT_NUM_DATA_CHECK		130

#define     NBCHANNELPERLINK	2
#if LTE_RATE == 20
#define     NBCHANNELMAXPERLINK	2
#elif LTE_RATE == 10
#define     NBCHANNELMAXPERLINK	4
#elif LTE_RATE == 5
#define     NBCHANNELMAXPERLINK	8
#elif LTE_RATE == 1
#define     NBCHANNELMAXPERLINK	32
#endif
#define     NBCHANNELMAX		(NBCHANNELPERLINK*2) 		                                    // 2-link max for this test
#if LTE_RATE == 1
#define     NBSYMBOL			1																// 1 jumbo packet per slot
#else
#define     NBSYMBOL			AIF2_LTE_SYMBOL_NUM											// Number of symbols per slot
#endif
#if LTE_RATE == 20
#define		LTESYMBOLSIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL1_SIZE)*4 + 16)    // 8848: FFT size + CP first
#define 	LTESYMBOL2SIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL_SIZE)*4 + 16)     // 8784: FFT size + CP 2nd-6th
#elif LTE_RATE == 10
#define		LTESYMBOLSIZE		((AIF2_LTE10_FFT_SIZE+AIF2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((AIF2_LTE10_FFT_SIZE+AIF2_LTE10_CYPRENORMAL_SIZE)*4 + 16)
#elif LTE_RATE == 5
#define		LTESYMBOLSIZE		((AIF2_LTE5_FFT_SIZE+AIF2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((AIF2_LTE5_FFT_SIZE+AIF2_LTE5_CYPRENORMAL_SIZE)*4 + 16)
#elif LTE_RATE == 1
#define		LTESYMBOLSIZE		((((AIF2_LTE1P4_FFT_SIZE+AIF2_LTE1P4_CYPRENORMAL1_SIZE)+(6*(AIF2_LTE1P4_FFT_SIZE+AIF2_LTE1P4_CYPRENORMAL_SIZE)))*4) + 16)
#endif

#define		NBCHANNELINIT		1


//Value for Sin calculation
#define     Q_FACTOR           	 	8191
#if LTE_RATE == 20
#define     SAMP_FREQ_FLOAT     	30.72
#elif LTE_RATE == 10
#define     SAMP_FREQ_FLOAT     	15.36
#elif LTE_RATE == 5
#define     SAMP_FREQ_FLOAT     	7.68
#elif LTE_RATE == 1
#define     SAMP_FREQ_FLOAT     	1.92
#endif
#define     PI                  	3.14159265358979323846

#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4 || EVM_TYPE == 5 || EVM_TYPE == 1 || EVM_TYPE == 6
#define     DESCMSM				1															// Lyrtech has no stable DDR, so using MSM and reduing the number of mono desc
#define		NBDESCMAX			128                                                         // Descriptor count rounded to the next power of 2
																							// 7 sym * 2 AxCs * 2 pingpong * 2 directions + 16 control packets
#else
#define     DESCMSM				0															// mono desc region going to DDR
#define		NBDESCMAX			128                                                         // Descriptor count rounded to the next power of 2
																							// 14 sym * 2 AxCs * 2 pingpong * 2 directions * 2 links
#endif

#define MONO_RX_FDQ            2024
#define MONO_RX_Q              1024
#define MONO_RX_FDQ_CW         2048
#define MONO_RX_Q_CW           1048


typedef struct {
	int16_t re;
	int16_t im;
	} Complex16;

extern volatile unsigned int testcheck;

extern volatile uint32_t aif2SymbolEgressCount[MAX_SLOT];
extern volatile uint32_t aif2SymbolIngressCount[MAX_SLOT];
extern volatile uint32_t getCpuTimestamp[MAX_SLOT];
extern volatile uint32_t getPhyTimerFrames[MAX_SLOT];
extern volatile uint32_t getPhyTimerClk[MAX_SLOT];
extern volatile uint32_t getRxPktCnt[MAX_SLOT];
extern volatile uint32_t symbolNum[MAX_SLOT][7];
extern unsigned char   saveDesc[2][14][32];
extern volatile uint32_t rxPktCnt[NBCHANNELPERLINK * 2];
extern volatile uint32_t symbolcount;
extern volatile uint32_t slotcount;



extern void getcomplex(Complex16* payload,uint32_t index, uint32_t tx);
extern void slotRecycling(AIF_PktDmaConfigHandle hPktDma, uint8_t rxBuffer[][LTESYMBOLSIZE-16], uint32_t nbchanneltotal, uint32_t firstChan, uint32_t activeLink);
extern void slotPushing(AIF_PktDmaConfigHandle hPktDma, uint32_t nbchanneltotal, uint32_t firstChan, Complex16 firstSample[][NBSYMBOL*2], Cppi_MonolithicDesc* symbolPkt[][NBSYMBOL*2]);
extern void load_data(AIF_PktDmaConfigHandle hPktDma, uint32_t nbchanneltotal, uint32_t firstChan, Complex16 firstSample[][NBSYMBOL*2], Cppi_MonolithicDesc* symbolPkt[][NBSYMBOL*2]);
extern void lteFinalCheck(AIF_PktDmaConfigHandle hPktDma, uint8_t rxBuffer[][LTESYMBOLSIZE-16], uint32_t nbchanneltotal, uint32_t firstChan, uint32_t activeLink);


