/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_cgem.h>

#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>
#include <ti/drv/iqn2/iqn2_osal.h>

#include <ti/drv/iqn2/iqn2ver.h>
#include <ti/drv/iqn2/include/IQN2_defs.h>
#include <ti/drv/iqn2/include/IQN2_runtime.h>

#define __IQN2_INIT_C
#include <ti/drv/iqn2/include/IQN2_init.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(IQN2_initHw, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_startHw, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_initDio, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_getVersion, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_getVersionStr, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_getIngressRadioStandardId, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_getEgressRadioStandardId, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_getEgressRadioTimerId, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_getIngressRadioTimerId, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_initRmAilSetupParams, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_initRadioTimer, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_initAt2Event, ".text:iqn2Init");

#pragma CODE_SECTION(initEgrGroupInfo, ".text:iqn2Init");
#pragma CODE_SECTION(initIngrGroupInfo, ".text:iqn2Init");
#pragma CODE_SECTION(addAxCtoEgrGroupTable, ".text:iqn2Init");
#pragma CODE_SECTION(addAxCtoIngrGroupTable, ".text:iqn2Init");
#pragma CODE_SECTION(getIngrGroupId, ".text:iqn2Init");
#pragma CODE_SECTION(getEgrGroupId, ".text:iqn2Init");
#pragma CODE_SECTION(getMaxEgrGroupId, ".text:iqn2Init");
#pragma CODE_SECTION(getMaxIngrGroupId, ".text:iqn2Init");

#pragma DATA_SECTION(iqn2Setup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(aid2Setup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ail0Setup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ail1Setup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ailEgrSetup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ailIgrSetup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ailPeSetup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ailPdSetup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ailUatSetup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ailPhySetup, ".far:iqn2Init_dat");

#pragma DATA_SECTION(topSetup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(iqs2Setup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(dio2Setup, ".far:iqn2Init_dat");
#pragma DATA_SECTION(at2Setup, ".far:iqn2Init_dat");

#pragma DATA_SECTION(Iqn2Obj, ".far:iqn2Init_dat");
#pragma DATA_SECTION(Iqn2Context, ".far:iqn2Init_dat");

#pragma DATA_SECTION(ingrRadStdInfo, ".far:iqn2Init_dat");
#pragma DATA_SECTION(egrRadStdInfo, ".far:iqn2Init_dat");
#pragma DATA_SECTION(egrGroupInfo, ".far:iqn2Init_dat");
#pragma DATA_SECTION(ingrGroupInfo, ".far:iqn2Init_dat");

#pragma DATA_SECTION(FrameMsgTot, ".far:iqn2Init_dat");
#pragma DATA_SECTION(FrameMsg1, ".far:iqn2Init_dat");
#pragma DATA_SECTION(FrameMsg, ".far:iqn2Init_dat");
#pragma DATA_SECTION(FrameMsg2, ".far:iqn2Init_dat");
#endif

#ifdef _TMS320C6X
#ifdef _LITTLE_ENDIAN
#define DEVICE_LE
#endif
#else
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define DEVICE_LE
#endif
#endif

typedef struct {
    uint32_t    indexSc;
    uint32_t    indexTc;
    uint32_t    symbolTc;
} radioStandardInfo_t;

typedef struct {
    uint32_t        samplingRate;
    IQN2_LteCpType  cpType;
    uint32_t        startAxC;
    uint32_t        endAxC;
    uint32_t        isPopulated; //0 if not, 1 if yes
    uint32_t        isLte;   //0 if not, 1 if yes
} baseGroupInfo_t;

typedef struct {
    baseGroupInfo_t g;
    uint32_t cellId;
} egrGroupInfo_t;

#define ingrGroupInfo_t egrGroupInfo_t

static Iqn2Fl_Setup               iqn2Setup;//Iqn2 HW setup																		

static Iqn2Fl_Aid2Setup           aid2Setup; // AID setup
static Iqn2Fl_AilSetup            ail0Setup; // Setup for AIL0
static Iqn2Fl_AilSetup            ail1Setup; // Setup for AIL1
static Iqn2Fl_AilEgrSetup         ailEgrSetup[IQN2FL_AIL_MAX]; // Egress setup
static Iqn2Fl_AilIgrSetup         ailIgrSetup[IQN2FL_AIL_MAX]; // Ingress setup
static Iqn2Fl_AilPeSetup          ailPeSetup[IQN2FL_AIL_MAX]; // PE setup
static Iqn2Fl_AilPdSetup          ailPdSetup[IQN2FL_AIL_MAX]; // PD setup
static Iqn2Fl_AilUatSetup         ailUatSetup[IQN2FL_AIL_MAX];//uAT setup
static Iqn2Fl_AilPhySetup         ailPhySetup[IQN2FL_AIL_MAX];//Phy setup

static Iqn2Fl_TopSetup            topSetup; // Top registers setup
static Iqn2Fl_Iqs2Setup           iqs2Setup; // IQS2 setup
static Iqn2Fl_Dio2Setup           dio2Setup; // DIO2 setup
static Iqn2Fl_At2Setup            at2Setup; // AT2 setup

static Iqn2Fl_Obj                 Iqn2Obj;
static Iqn2Fl_Context             Iqn2Context;

static radioStandardInfo_t        ingrRadStdInfo[IQN2_MAX_NUM_RADIO_STANDARD];
static radioStandardInfo_t        egrRadStdInfo[IQN2_MAX_NUM_RADIO_STANDARD];
static egrGroupInfo_t             egrGroupInfo[IQN2_MAX_NUM_RADIO_STANDARD];
static ingrGroupInfo_t            ingrGroupInfo[IQN2_MAX_NUM_RADIO_STANDARD];

static uint32_t                   FrameMsgTot[IQN2_MAX_NUM_RADIO_STANDARD];
static uint32_t                   FrameMsg1[IQN2_MAX_NUM_RADIO_STANDARD];
static uint32_t                   FrameMsg[IQN2_MAX_NUM_RADIO_STANDARD];
static uint32_t                   FrameMsg2[IQN2_MAX_NUM_RADIO_STANDARD];

uint32_t IQN2_getVersion (void)
{
    return IQN2_LLD_VERSION_ID;
}

const char  iqn2LldVersionStr[] = IQN2_LLD_VERSION_STR ":" __DATE__  ":" __TIME__;

const char* IQN2_getVersionStr (void)
{
    return iqn2LldVersionStr;
}

//Initialize IQN2 DIO buffers				//FIXME need update for lte dio mode
void
IQN2_initDio(
		IQN2_ConfigHandle  hIqn2
)
{
	uint32_t i,j, num_channel_pe, num_channel_pd, num_AxC_pe, num_AxC_pd;
	
	num_channel_pe = 0;
	num_channel_pd = 0;
	for (i=0; i<IQN2_MAX_NUM_DIO_ENGINE; i++)
	{	
		num_channel_pe += hIqn2->dioConfig[i].numPeDBCH;
		num_channel_pd += hIqn2->dioConfig[i].numPdDBCH;
	}
	
	num_AxC_pe = 0;
	num_AxC_pd = 0;
	if( num_channel_pe == 0)
	{
		for (j=0; j<IQN2_MAX_NUM_AIL; j++)
		{
			num_AxC_pe += hIqn2->ailConfig[j].numWcdmaPeAxC;	
		}
		num_AxC_pe += hIqn2->aidConfig.numWcdmaEgressAxC;
		for (i=0;i<=(num_AxC_pe/16);i++)
		{
		    hIqn2->dioConfig[i].offsetPeDBCH = i*16;
		    if(i == num_AxC_pe/16)
				hIqn2->dioConfig[i].numPeDBCH = num_AxC_pe%16;
			else
				hIqn2->dioConfig[i].numPeDBCH = 16;
		}
	} else {
		num_AxC_pe = num_channel_pe;
	}
	
	if( num_channel_pd == 0)
	{
		for (j=0; j<IQN2_MAX_NUM_AIL; j++)
		{
			num_AxC_pd += hIqn2->ailConfig[j].numWcdmaPdAxC;	
		}
		num_AxC_pd += hIqn2->aidConfig.numWcdmaIngressAxC;
		for (i=0;i<=(num_AxC_pd/16);i++)
		{
			hIqn2->dioConfig[i].offsetPdDBCH = i*16;
			if(i == num_AxC_pd/16)
				hIqn2->dioConfig[i].numPdDBCH = num_AxC_pd%16;
			else
				hIqn2->dioConfig[i].numPdDBCH = 16;
		}
	} else {
		num_AxC_pd = num_channel_pd;
	}

    for (i=0; i<IQN2_MAX_NUM_DIO_ENGINE; i++)
    {
        if(hIqn2->dioConfig[i].numPdDBCH != 0)
            hIqn2->dioConfig[i].dioEnable = 1;
    }
} 

static void initEgrGroupInfo(void)
{
    memset(egrGroupInfo, 0, IQN2_MAX_NUM_RADIO_STANDARD * sizeof(egrGroupInfo_t));
}

static void initIngrGroupInfo(ingrGroupInfo_t *ingrGroupInfoTable, uint32_t tableSize)
{
    memset(ingrGroupInfoTable, 0, tableSize * sizeof(ingrGroupInfo_t));
}

 //returns groupId
static uint32_t addAxCtoEgrGroupTable(uint32_t AxC, float samplingRate, IQN2_LteCpType cpType,
                                    uint32_t cellId, uint32_t isLte, uint32_t *newEntry)
{
    uint32_t i, found = 0, id;
    egrGroupInfo_t *g;

    for(i = 0; i < IQN2_MAX_NUM_RADIO_STANDARD; i++) {
        g = &egrGroupInfo[i];
        if (g->g.isPopulated) {
            //check if entries match
            if ((samplingRate == g->g.samplingRate) && (cpType == g->g.cpType) && (cellId == g->cellId) && (isLte == g->g.isLte) && (AxC == (g->g.endAxC+1))) {
                //matched, add AxC
                if (AxC < g->g.startAxC) {
                    g->g.startAxC = AxC;
                }
                if (AxC > g->g.endAxC) {
                    g->g.endAxC = AxC;
                }
                *newEntry = 0;
                return(i);
            }
        }
    }

    //find the first unpopulated group and populate it
    for(i = 0; i < IQN2_MAX_NUM_RADIO_STANDARD; i++) {
        g = &egrGroupInfo[i];
        if (! g->g.isPopulated) {
            found = 1;
            id = i;
            break;
        }
    }
    if (! found) {  //error
        return(IQN2_MAX_NUM_RADIO_STANDARD); //return invalid groupId
    }

    //populate
    g->g.samplingRate = samplingRate;
    if (isLte == 0)
    {
        g->g.isLte = 0;
    } else {
        g->g.isLte = 1;
        g->cellId = cellId;
        g->g.cpType = cpType;
    }
    g->g.startAxC = AxC;
    g->g.endAxC   = AxC;
    g->g.isPopulated = 1;
    *newEntry = 1;
    return(id);
}

//returns groupId
static uint32_t addAxCtoIngrGroupTable(ingrGroupInfo_t *ingrGroupInfoTable, uint32_t tableSize, uint32_t AxC,
                        float samplingRate, IQN2_LteCpType cpType, uint32_t cellId, uint32_t isLte, uint32_t *newEntry)
{
    uint32_t i, found = 0, id;
    ingrGroupInfo_t *g;

    for(i = 0; i < tableSize; i++) {
        g = &ingrGroupInfoTable[i];
        if (g->g.isPopulated) {
            //check if entries match
            if ((samplingRate == g->g.samplingRate) && (cpType == g->g.cpType) && (isLte == g->g.isLte) && (cellId == g->cellId) && (AxC == (g->g.endAxC+1))) {
                //matched, add AxC
                if (AxC < g->g.startAxC) {
                    g->g.startAxC = AxC;
                }
                if (AxC > g->g.endAxC) {
                    g->g.endAxC = AxC;
                }
                *newEntry = 0;
                return(i);
            }
        }
    }

    //find the first unpopulated group and populate it
    for(i = 0; i < tableSize; i++) {
        g = &ingrGroupInfoTable[i];
        if (! g->g.isPopulated) {
            found = 1;
            id = i;
            break;
        }
    }
    if (! found) {  //error
        return(tableSize); //invalid group Id
    }
    //populate
    g->g.samplingRate = samplingRate;
    if(isLte == 0)
    {
        g->g.isLte = 0;
    } else {
        g->g.isLte = 1;
        g->g.cpType = cpType;
        g->cellId = cellId;
    }
    g->g.startAxC = AxC;
    g->g.endAxC   = AxC;
    g->g.isPopulated = 1;
    *newEntry = 1;
    return(id);
}

static uint32_t getIngrGroupId(uint32_t AxC)
{
    uint32_t i;

    for(i = 0; i < IQN2_MAX_NUM_RADIO_STANDARD; i++) {
        if ((AxC >= ingrGroupInfo[i].g.startAxC) && (AxC <= ingrGroupInfo[i].g.endAxC)) {
            return(i);
        }
    }
    return(IQN2_MAX_NUM_RADIO_STANDARD); //invalid group
}

uint32_t
IQN2_getIngressRadioStandardId(
        uint32_t AxC
)
{
    return(getIngrGroupId(AxC));
}

static uint32_t getEgrGroupId(uint32_t AxC)
{
    uint32_t i;

    for(i = 0; i < IQN2_MAX_NUM_RADIO_STANDARD; i++) {
        if ((AxC >= egrGroupInfo[i].g.startAxC) && (AxC <= egrGroupInfo[i].g.endAxC)) {
            return(i);
        }
    }
    return(IQN2_MAX_NUM_RADIO_STANDARD); //invalid group
}

uint32_t
IQN2_getEgressRadioStandardId(
        uint32_t AxC
)
{
    return(getEgrGroupId(AxC));
}

static uint32_t getMaxEgrGroupId()
{
    uint32_t i, max=0;

    for(i = 0; i < IQN2_MAX_NUM_RADIO_STANDARD; i++) {
        if (egrGroupInfo[i].g.isPopulated) {
            max = i+1;
        }
    }
    return(max);
}

static uint32_t getMaxIngrGroupId()
{
    uint32_t i, max=0;

    for(i = 0; i < IQN2_MAX_NUM_RADIO_STANDARD; i++) {
        if (ingrGroupInfo[i].g.isPopulated) {
            max = i+1;
        }
    }
    return(max);
}

// Egress radio timer id is equal to Egress radio standard id
uint32_t
IQN2_getEgressRadioTimerId(
        uint32_t AxC
)
{
    return(getEgrGroupId(AxC));
}

// Ingress radio timer id is equal to Ingress radio standard id + max number of radio standards
// It is expected that DL application and UL application use a different radio timer
uint32_t
IQN2_getIngressRadioTimerId(
        uint32_t AxC
)
{
    uint32_t radioId;
    radioId = getMaxEgrGroupId() + getIngrGroupId(AxC);
    if (radioId<IQN2_MAX_NUM_RADT) {
        return(radioId);
    } else {
        return(IQN2_MAX_NUM_RADT); // invalid radio timer ID
    }
}

// Returns frame strobe enum for this radio timer id
Iqn2Fl_AtEvtStrobe
IQN2_getRadioTimerFrameStrobe(
        uint32_t radioId
)
{
    return ((Iqn2Fl_AtEvtStrobe)radioId);
}

// Returns symbol strobe enum for this radio timer id
Iqn2Fl_AtEvtStrobe
IQN2_getRadioTimerSymbolStrobe(
        uint32_t radioId
)
{
    return ((Iqn2Fl_AtEvtStrobe)(IQN2FL_RADT0_SYMBOL+radioId));
}

static uint32_t
getSampleRate(
		IQN2_SampleRate enumRate
)
{
	if	(enumRate == IQN2_SRATE_1P92MHZ)
		return (1920);
	else if (enumRate == IQN2_SRATE_3P84MHZ)
		return (3840);
	else if (enumRate == IQN2_SRATE_7P68MHZ)
		return (7680);
	else if (enumRate == IQN2_SRATE_15P36MHZ)
		return (15360);
	else if (enumRate == IQN2_SRATE_23P04MHZ)
		return (23040);
	else if (enumRate == IQN2_SRATE_30P72MHZ)
		return (30720);
	else if (enumRate == IQN2_SRATE_61P44MHZ)
		return (61440);
	else if (enumRate == IQN2_SRATE_92P16MHZ)
		return (92160);
	else if (enumRate == IQN2_SRATE_122P88MHZ)
		return (122880);
	else if (enumRate == IQN2_SRATE_184P32MHZ)
		return (184320);
	else
		return (30720);
}

/* Setup IQN2 FL structure given user configuration*/
void
IQN2_initHw(
		IQN2_ConfigHandle  hIqn2,
	    Iqn2Fl_InitCfg*    hIqn2BaseAddr
)
{
	Iqn2Fl_Status iqn2Status;
	uint32_t	    i,j,k;
	uint32_t	dbmxPe[IQN2_MAX_NUM_RADIO_STANDARD];
	uint32_t    kncPe[IQN2_MAX_NUM_RADIO_STANDARD];
	uint32_t    dbmxPeTotal;
	uint32_t	dbmxPd[IQN2_MAX_NUM_RADIO_STANDARD];
	uint32_t    kncPd[IQN2_MAX_NUM_RADIO_STANDARD];
	uint32_t    dbmxPdTotal;
	uint32_t 	src[IQN2_MAX_NUM_DIO_ENGINE][16];
	uint32_t    dest[IQN2_MAX_NUM_DIO_ENGINE][16];
	uint32_t    maxcontainers = 0;
	uint32_t    containercount = 0;
	uint32_t    dataWidth = 0;
	uint8_t     frameIndexSc  = 0;
	uint32_t    enableDio = 0;
	IQN2_LteCpType  lteCpType;
	uint32_t     groupId, cellId, isLte, isNewGroup;
	uint32_t    offsetdbmx;
	uint32_t    frameLength=0;

   	memset(&iqn2Setup, 0, sizeof(iqn2Setup));
	memset(&aid2Setup, 0, sizeof(aid2Setup));
	memset(&ail0Setup, 0, sizeof(ail0Setup));
	memset(&ail1Setup, 0, sizeof(ail1Setup));
	memset(&ailEgrSetup[0], 0, sizeof(ailEgrSetup));
	memset(&ailIgrSetup[0], 0, sizeof(ailIgrSetup));
	memset(&ailPeSetup[0], 0, sizeof(ailPeSetup));
	memset(&ailPdSetup[0], 0, sizeof(ailPdSetup));
	memset(&ailUatSetup[0], 0, sizeof(ailUatSetup));
    memset(&ailPhySetup[0], 0, sizeof(ailPhySetup));

	memset(&topSetup, 0, sizeof(topSetup));
	memset(&iqs2Setup, 0, sizeof(iqs2Setup));
	memset(&dio2Setup, 0, sizeof(dio2Setup));
	memset(&at2Setup, 0, sizeof(at2Setup));

	/* Initialize CSL library, this step is required - Does not do anything at the moment! */
	Iqn2Fl_init(&Iqn2Context);
	
	hIqn2->hFl = Iqn2Fl_open(&Iqn2Obj, CSL_IQN, hIqn2BaseAddr, &iqn2Status);  //FIXME (CSL_IQN is set instead of previously 0)
	if ((hIqn2->hFl == NULL) || (iqn2Status != IQN2FL_SOK))
	{
	   Iqn2_osalLog ("\nError opening CSL_IQN");
	   exit(1);
	}
	
	hIqn2->hIqn2Setup = &iqn2Setup;

	iqn2Setup.ailSetup[IQN2FL_AIL_0] = &ail0Setup;
	iqn2Setup.ailSetup[IQN2FL_AIL_1] = &ail1Setup;
	iqn2Setup.topSetup = &topSetup;
	iqn2Setup.iqs2Setup = &iqs2Setup;
	iqn2Setup.dio2Setup = &dio2Setup;
	iqn2Setup.at2Setup = &at2Setup;
	iqn2Setup.aid2Setup = &aid2Setup;

    #define MBSFN_FACTOR 2
	initEgrGroupInfo();
	frameIndexSc = 0;
	for (k=0; k<IQN2_MAX_NUM_AIL; k++)
	{
	    /* WCDMA AxCs */
	    for (i=hIqn2->ailConfig[k].firstWcdmaAxC;i<(hIqn2->ailConfig[k].numWcdmaPeAxC + hIqn2->ailConfig[k].firstWcdmaAxC);i++)
        {
	    	if(hIqn2->AxCconfig[i].sampleRate != IQN2_SRATE_CUSTOM)
	    		hIqn2->AxCconfig[i].customSampleRate = getSampleRate(hIqn2->AxCconfig[i].sampleRate);
	        lteCpType = hIqn2->AxCconfig[i].lteCpType;
	        cellId = hIqn2->AxCconfig[i].egressLTEcellId;
            isLte = 0;
            if ((groupId = addAxCtoEgrGroupTable(i, hIqn2->AxCconfig[i].customSampleRate, lteCpType, cellId, isLte, &isNewGroup))
                 == IQN2_MAX_NUM_RADIO_STANDARD) {
                Iqn2_osalLog("Fatal Error: addAxCtoEgrGroupTable returned bad ID\n");
            } else {
                if (isNewGroup) {
                    egrRadStdInfo[groupId].indexSc     = frameIndexSc;//start index
                    egrRadStdInfo[groupId].indexTc     = frameIndexSc;//terminal index
                    egrRadStdInfo[groupId].symbolTc    = IQN2_SLOT_COUNT_TC_WCDMA_FDD;//15 slots for WCDMA
                    frameIndexSc += 1;
                }
            }
        }

	    /* LTE AxCs */
	    for (i=hIqn2->ailConfig[k].firstLteAxC;i<(hIqn2->ailConfig[k].numLtePeAxC + hIqn2->ailConfig[k].firstLteAxC);i++)
        {
	    	if(hIqn2->AxCconfig[i].sampleRate != IQN2_SRATE_CUSTOM)
	    		hIqn2->AxCconfig[i].customSampleRate = getSampleRate(hIqn2->AxCconfig[i].sampleRate);
	        lteCpType = hIqn2->AxCconfig[i].lteCpType;
            cellId = hIqn2->AxCconfig[i].egressLTEcellId;
            isLte = 1;
            if ((groupId = addAxCtoEgrGroupTable(i, hIqn2->AxCconfig[i].customSampleRate, lteCpType, cellId, isLte, &isNewGroup))
                 == IQN2_MAX_NUM_RADIO_STANDARD) {
                Iqn2_osalLog("Fatal Error: addAxCtoEgrGroupTable returned bad ID\n");
            } else {
                if (isNewGroup) {
                    if (lteCpType == IQN2_LTE_CPTYPE_NORMAL) {
                        egrRadStdInfo[groupId].indexSc     = frameIndexSc;//start index
                        egrRadStdInfo[groupId].indexTc     = frameIndexSc + ((MBSFN_FACTOR * IQN2_LTE_SYMBOL_NUM) - 1);//terminal index
                        egrRadStdInfo[groupId].symbolTc    = IQN2_LTE_FRAME_SYMBOL_NUM-1;//140 symbols for LTE
                        frameIndexSc += (MBSFN_FACTOR * IQN2_LTE_SYMBOL_NUM);
                    }
                    else { // Extended CP
                        egrRadStdInfo[groupId].indexSc  = frameIndexSc;//start index
                        egrRadStdInfo[groupId].indexTc  = frameIndexSc + (IQN2_LTE_SYMBOL_NUM_EXT_CP-1);//terminal index
                        egrRadStdInfo[groupId].symbolTc = IQN2_LTE_FRAME_SYMBOL_NUM_EXT_CP-1;//120 symbols for LTE with Extended CP
                        frameIndexSc += IQN2_LTE_SYMBOL_NUM_EXT_CP;
                    }
                }
            }
        }
	}

    for (i=hIqn2->aidConfig.firstLteAxC;i<(hIqn2->aidConfig.numLteEgressAxC + hIqn2->aidConfig.firstLteAxC);i++)
    {
    	if(hIqn2->AxCconfig[i].sampleRate != IQN2_SRATE_CUSTOM)
    		hIqn2->AxCconfig[i].customSampleRate = getSampleRate(hIqn2->AxCconfig[i].sampleRate);
        lteCpType = hIqn2->AxCconfig[i].lteCpType;
        cellId = hIqn2->AxCconfig[i].egressLTEcellId;
        isLte = 1;
        if ((groupId = addAxCtoEgrGroupTable(i, hIqn2->AxCconfig[i].customSampleRate, lteCpType, cellId, isLte, &isNewGroup))
             == IQN2_MAX_NUM_RADIO_STANDARD) {
            Iqn2_osalLog("Fatal Error: addAxCtoEgrGroupTable returned bad ID\n");
        } else {
            if (isNewGroup) {
				if (lteCpType == IQN2_LTE_CPTYPE_NORMAL) {
					egrRadStdInfo[groupId].indexSc     = frameIndexSc;//start index
					egrRadStdInfo[groupId].indexTc     = frameIndexSc + ((MBSFN_FACTOR * IQN2_LTE_SYMBOL_NUM) - 1);//terminal index
					egrRadStdInfo[groupId].symbolTc    = IQN2_LTE_FRAME_SYMBOL_NUM-1;//140 symbols for LTE
					frameIndexSc += (MBSFN_FACTOR * IQN2_LTE_SYMBOL_NUM);
				} else if (lteCpType == IQN2_LTE_CPTYPE_EXTENDED) { // Extended CP
					egrRadStdInfo[groupId].indexSc  = frameIndexSc;//start index
					egrRadStdInfo[groupId].indexTc  = frameIndexSc + (IQN2_LTE_SYMBOL_NUM_EXT_CP-1);//terminal index
					egrRadStdInfo[groupId].symbolTc = IQN2_LTE_FRAME_SYMBOL_NUM_EXT_CP-1;//120 symbols for LTE with extended CP
					frameIndexSc += IQN2_LTE_SYMBOL_NUM_EXT_CP;
                } else {
                    egrRadStdInfo[groupId].indexSc  = frameIndexSc;//start index
                    egrRadStdInfo[groupId].indexTc  = frameIndexSc;//terminal index
                    if(((hIqn2->AxCconfig[i].customSampleRate*10) % hIqn2->AxCconfig[i].packetSize) != 0)
                    {
                        Iqn2_osalLog("Fatal Error: the sampling rate is not a multiple of the packet size \n");
                    } else if (((hIqn2->AxCconfig[i].customSampleRate*10) / hIqn2->AxCconfig[i].packetSize) > 255) {
                        Iqn2_osalLog("Fatal Error: too many symbols in one radio radio, max is 255. \n");
                    } else {
                        egrRadStdInfo[groupId].symbolTc = ((hIqn2->AxCconfig[i].customSampleRate*10) / hIqn2->AxCconfig[i].packetSize) -1;//nb symbol custom: nbSymbol = samplingRate / packetSize
                    }
                    frameIndexSc ++;
                }
            }
        }
    }

    for (i=hIqn2->aidConfig.firstWcdmaAxC;i<(hIqn2->aidConfig.numWcdmaEgressAxC + hIqn2->aidConfig.firstWcdmaAxC);i++)
    {
    	if(hIqn2->AxCconfig[i].sampleRate != IQN2_SRATE_CUSTOM)
    		hIqn2->AxCconfig[i].customSampleRate = getSampleRate(hIqn2->AxCconfig[i].sampleRate);
        lteCpType = hIqn2->AxCconfig[i].lteCpType;
        cellId = hIqn2->AxCconfig[i].egressLTEcellId;
        isLte = 0;
        if ((groupId = addAxCtoEgrGroupTable(i, hIqn2->AxCconfig[i].customSampleRate, lteCpType, cellId, isLte, &isNewGroup))
             == IQN2_MAX_NUM_RADIO_STANDARD) {
            Iqn2_osalLog("Fatal Error: addAxCtoEgrGroupTable returned bad ID\n");
        } else {
            if (isNewGroup) {
                egrRadStdInfo[groupId].indexSc     = frameIndexSc;//start index
                egrRadStdInfo[groupId].indexTc     = frameIndexSc;//terminal index
                egrRadStdInfo[groupId].symbolTc    = IQN2_SLOT_COUNT_TC_WCDMA_FDD;//15 slots for WCDMA
                frameIndexSc += 1;
            }
        }
    }

	initIngrGroupInfo(&ingrGroupInfo[0], IQN2_MAX_NUM_RADIO_STANDARD);
	frameIndexSc = 0;
    for (k=0; k<IQN2_MAX_NUM_AIL; k++)
    {
        for (i=hIqn2->ailConfig[k].firstWcdmaAxC;i<(hIqn2->ailConfig[k].numWcdmaPdAxC+hIqn2->ailConfig[k].firstWcdmaAxC);i++)
        {
            lteCpType = hIqn2->AxCconfig[i].lteCpType;
            cellId = hIqn2->AxCconfig[i].ingressLTEcellId;
            isLte = 0;
            if ((groupId = addAxCtoIngrGroupTable(&ingrGroupInfo[0], IQN2_MAX_NUM_RADIO_STANDARD, i, hIqn2->AxCconfig[i].customSampleRate, lteCpType, cellId, isLte, &isNewGroup))
                 == IQN2_MAX_NUM_RADIO_STANDARD) {
                Iqn2_osalLog("Fatal Error: addAxCtoIngrGroupTable returned bad ID\n");
            } else {
                if (isNewGroup) {
                    ingrRadStdInfo[groupId].indexSc     = frameIndexSc;//start index
                    ingrRadStdInfo[groupId].indexTc     = frameIndexSc;//terminal index
                    ingrRadStdInfo[groupId].symbolTc    = IQN2_SLOT_COUNT_TC_WCDMA_FDD;//15 slots for Wcdma
                    frameIndexSc += 1;
                }
            }
        }
        for (i=hIqn2->ailConfig[k].firstLteAxC;i<hIqn2->ailConfig[k].numLtePdAxC + hIqn2->ailConfig[k].firstLteAxC;i++)
        {
            lteCpType = hIqn2->AxCconfig[i].lteCpType;
            cellId = hIqn2->AxCconfig[i].ingressLTEcellId;
            isLte = 1;
            if ((groupId = addAxCtoIngrGroupTable(&ingrGroupInfo[0], IQN2_MAX_NUM_RADIO_STANDARD, i, hIqn2->AxCconfig[i].customSampleRate, lteCpType, cellId, isLte, &isNewGroup))
                 == IQN2_MAX_NUM_RADIO_STANDARD) {
                Iqn2_osalLog("Fatal Error: addAxCtoIngrGroupTable returned bad ID\n");
            } else {
                if (isNewGroup) {
                    if (lteCpType == IQN2_LTE_CPTYPE_NORMAL) {
                        ingrRadStdInfo[groupId].indexSc     = frameIndexSc;//start index
                        ingrRadStdInfo[groupId].indexTc     = frameIndexSc + (IQN2_LTE_SYMBOL_NUM-1);//terminal index
                        ingrRadStdInfo[groupId].symbolTc    = IQN2_LTE_FRAME_SYMBOL_NUM-1;//140 symbols for LTE
                        frameIndexSc += IQN2_LTE_SYMBOL_NUM;
                    }
                    else { // Extended CP
                        ingrRadStdInfo[groupId].indexSc  = frameIndexSc;//start index
                        ingrRadStdInfo[groupId].indexTc  = frameIndexSc + (IQN2_LTE_SYMBOL_NUM_EXT_CP-1);//terminal index
                        ingrRadStdInfo[groupId].symbolTc = IQN2_LTE_FRAME_SYMBOL_NUM_EXT_CP-1;//120 symbols for LTE with extended CP
                        frameIndexSc += IQN2_LTE_SYMBOL_NUM_EXT_CP;
                    }
                }
            }
        }
	}

    for (i=hIqn2->aidConfig.firstLteAxC;i<hIqn2->aidConfig.numLteIngressAxC + hIqn2->aidConfig.firstLteAxC;i++)
    {
        lteCpType = hIqn2->AxCconfig[i].lteCpType;
        cellId = hIqn2->AxCconfig[i].ingressLTEcellId;
        isLte = 1;
        if ((groupId = addAxCtoIngrGroupTable(&ingrGroupInfo[0], IQN2_MAX_NUM_RADIO_STANDARD, i, hIqn2->AxCconfig[i].customSampleRate, lteCpType, cellId, isLte, &isNewGroup))
             == IQN2_MAX_NUM_RADIO_STANDARD) {
            Iqn2_osalLog("Fatal Error: addAxCtoIngrGroupTable returned bad ID\n");
        } else {
            if (isNewGroup) {
				if (lteCpType == IQN2_LTE_CPTYPE_NORMAL) {
					ingrRadStdInfo[groupId].indexSc     = frameIndexSc;//start index
					ingrRadStdInfo[groupId].indexTc     = frameIndexSc + (IQN2_LTE_SYMBOL_NUM-1);//terminal index
					ingrRadStdInfo[groupId].symbolTc    = IQN2_LTE_FRAME_SYMBOL_NUM-1;//140 symbols for LTE
					frameIndexSc += IQN2_LTE_SYMBOL_NUM;
				} else if (lteCpType == IQN2_LTE_CPTYPE_EXTENDED) { // Extended CP
					ingrRadStdInfo[groupId].indexSc  = frameIndexSc;//start index
					ingrRadStdInfo[groupId].indexTc  = frameIndexSc + (IQN2_LTE_SYMBOL_NUM_EXT_CP-1);//terminal index
					ingrRadStdInfo[groupId].symbolTc = IQN2_LTE_FRAME_SYMBOL_NUM_EXT_CP-1;//120 symbols for LTE with extended CP
					frameIndexSc += IQN2_LTE_SYMBOL_NUM_EXT_CP;
				} else {
				    ingrRadStdInfo[groupId].indexSc  = frameIndexSc;//start index
                    ingrRadStdInfo[groupId].indexTc  = frameIndexSc;//terminal index
                    if(((hIqn2->AxCconfig[i].customSampleRate*10) % hIqn2->AxCconfig[i].packetSize) != 0)
                    {
                        Iqn2_osalLog("Fatal Error: the sampling rate is not a multiple of the packet size \n");
                    } else if (((hIqn2->AxCconfig[i].customSampleRate*10) / hIqn2->AxCconfig[i].packetSize) > 255) {
                        Iqn2_osalLog("Fatal Error: too many packets in one 10ms radio frame, max is 255.\n");
                    } else {
                        ingrRadStdInfo[groupId].symbolTc = ((hIqn2->AxCconfig[i].customSampleRate*10) / hIqn2->AxCconfig[i].packetSize) -1;//nb symbol custom: nbSymbol = samplingRate / packetSize
                    }
                    frameIndexSc ++;
				}
            }
        }
    }

    for (i=hIqn2->aidConfig.firstWcdmaAxC;i<(hIqn2->aidConfig.numWcdmaIngressAxC+hIqn2->aidConfig.firstWcdmaAxC);i++)
    {
        lteCpType = hIqn2->AxCconfig[i].lteCpType;
        cellId = hIqn2->AxCconfig[i].ingressLTEcellId;
        isLte = 0;
        if ((groupId = addAxCtoIngrGroupTable(&ingrGroupInfo[0], IQN2_MAX_NUM_RADIO_STANDARD, i, hIqn2->AxCconfig[i].customSampleRate, lteCpType, cellId, isLte, &isNewGroup))
             == IQN2_MAX_NUM_RADIO_STANDARD) {
            Iqn2_osalLog("Fatal Error: addAxCtoIngrGroupTable returned bad ID\n");
        } else {
            if (isNewGroup) {
                ingrRadStdInfo[groupId].indexSc     = frameIndexSc;//start index
                ingrRadStdInfo[groupId].indexTc     = frameIndexSc;//terminal index
                ingrRadStdInfo[groupId].symbolTc    = IQN2_SLOT_COUNT_TC_WCDMA_FDD;//15 slots for Wcdma
                frameIndexSc += 1;
            }
        }
    }

    for(i=0; i<IQN2_MAX_NUM_RADIO_STANDARD; i++)
    {
        if (ingrGroupInfo[i].g.isLte)
        {
            if (ingrGroupInfo[i].g.cpType == IQN2_LTE_CPTYPE_NONE)
            {
                FrameMsg1[i] = ingrGroupInfo[i].g.samplingRate * 10 / (ingrRadStdInfo[i].symbolTc+1);
                FrameMsg[i] = ingrGroupInfo[i].g.samplingRate * 10 / (ingrRadStdInfo[i].symbolTc+1);
                FrameMsgTot[i] = ingrGroupInfo[i].g.samplingRate * 10 / (ingrRadStdInfo[i].symbolTc+1);
            } else {
                FrameMsg1[i] = ingrGroupInfo[i].g.samplingRate * (IQN2_LTE20_FFT_SIZE + IQN2_LTE20_CYPRENORMAL1_SIZE) / IQN2_LTE20_SAMPLE_RATE;
                FrameMsg[i] = ingrGroupInfo[i].g.samplingRate * (IQN2_LTE20_FFT_SIZE + IQN2_LTE20_CYPRENORMAL_SIZE) / IQN2_LTE20_SAMPLE_RATE;
                FrameMsg2[i] = ingrGroupInfo[i].g.samplingRate * (IQN2_LTE20_FFT_SIZE + IQN2_LTE20_CYPREEXTENDED_SIZE) / IQN2_LTE20_SAMPLE_RATE;
                FrameMsgTot[i] = FrameMsg1[i] + (6 * FrameMsg[i]);
            }
        } else {
            FrameMsg[i] = 2560;
            FrameMsgTot[i] = FrameMsg[i];
        }
    }


   	/************** IQN2 TOP area Setup  ***************************************************/
    topSetup.top_vc_sys_sts_cfg.sysclk_sel   = 0;//use SERDES lane0 clk is used for sys_clk
    if (hIqn2->aidConfig.aidEnable) {
        topSetup.top_vc_sys_sts_cfg.at_dfe_clk_sel = 1;
    } else {
        topSetup.top_vc_sys_sts_cfg.at_dfe_clk_sel = 0;
    }
	topSetup.top_psr_cfg.bw_limit = 0; //use 1/16 VBUS BW for packet flushing

    for (j=0; j<IQN2_MAX_NUM_AIL; j++)
    {
        for(i=0;i<(hIqn2->ailConfig[j].numLtePeAxC);i++){
            topSetup.top_psr_cfg.drop_pkt[i] = 1;
            topSetup.top_psr_cfg.pack_ps_data[i] = 1;//pack ps data for pktDMA chan
        }
    }


    for(i=0;i<hIqn2->aidConfig.numLteEgressAxC;i++){ //FIXME lteDIO
        topSetup.top_psr_cfg.drop_pkt[i] = 1;//1;
        topSetup.top_psr_cfg.pack_ps_data[i] = 1;//pack ps data for pktDMA chan
    }

    for(i=hIqn2->aidConfig.firstCtlChannel;i<hIqn2->aidConfig.numCtlChannel + hIqn2->aidConfig.firstCtlChannel;i++){
    	topSetup.top_psr_cfg.drop_pkt[i] = 1;
    	topSetup.top_psr_cfg.arb_priority[i] = 2;
    	topSetup.top_psr_cfg.pack_ps_data[i] = 0;
    }

    if (hIqn2->aidConfig.aidEnable)
        topSetup.top_vc_sys_sts_cfg.at_dfe_clk_sel = 1;//use DFE PLL output clk for sub system sys_clk

	
	/************** IQS Setup  ********************************************************************************/
    //SoC Egress
    for (j=0; j<IQN2_MAX_NUM_AIL; j++)
    {
        for(i=0;i<hIqn2->ailConfig[j].numWcdmaPeAxC;i++){
            iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].chan = hIqn2->ailConfig[j].firstWcdmaAxC + i;//DIO chan 0 ~ 15 is mapped to AIL0 chan 0 ~ 15
            iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].dest = (Iqn2Fl_EgressChanDest) ((j+1)*2);    //IQN2FL_AIL0_AXC; //destination is AIL0 AxC
            iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].psi_pri = 1;
            iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].arb_pri = 1;
        }

        for(i=0;i<hIqn2->ailConfig[j].numLtePeAxC;i++){
            iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].chan = hIqn2->ailConfig[j].firstLteAxC + i;//pktDMA chan 0 ~ 3 is mapped to AIL0 chan 0 ~ 3
            iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].dest = (Iqn2Fl_EgressChanDest) ((j+1)*2);  //IQN2FL_AIL0_AXC; //destination is AIL0 AxC
        }

        for(i=0;i<hIqn2->ailConfig[j].numCtlChannel;i++){
            iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[hIqn2->ailConfig[j].firstCtlChannel + i].chan = i;//pktDMA chan 0 ~ 3 is mapped to AIL0 chan 0 ~ 3
            iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[hIqn2->ailConfig[j].firstCtlChannel + i].dest = (Iqn2Fl_EgressChanDest) (((j+1)*2)+1);   //IQN2FL_AIL0_CTL; //destination is AIL0 AxC
        }
    }
	
    for(i=0;i<hIqn2->aidConfig.numWcdmaEgressAxC;i++){
        iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].chan = hIqn2->aidConfig.firstWcdmaAxC + i;//DIO chan 0 ~ 15 is mapped to AID chan 0 ~ 15
        iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].dest = IQN2FL_AID2_AXC;    //IQN2FL_AID_AXC; //destination is AID AxC
        iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].psi_pri = 1;
        iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].arb_pri = 1;
    }

	for(i=0;i<hIqn2->aidConfig.numLteEgressAxC;i++){
		if(hIqn2->aidConfig.lteDio==1)
		{
			iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].chan = hIqn2->aidConfig.firstLteAxC + i;//DIO chan 0 ~ 15 is mapped to AID chan 0 ~ 15
			iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].dest = IQN2FL_AID2_AXC;    //IQN2FL_AID_AXC; //destination is AID AxC
			iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].psi_pri = 1;
			iqs2Setup.iqs2_egress_chan_cfg.egr_dio2_cfg[i].arb_pri = 1;
		} else {
			iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].chan = hIqn2->aidConfig.firstLteAxC + i;//pktDMA chan 0 ~ 3 is mapped to AID chan 0 ~ 3
			iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].dest = IQN2FL_AID2_AXC; //destination is AID AxC
			iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].arb_pri = 0;
			iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[i].psi_pri = 0;
		}
	}

	for(i=0;i<hIqn2->aidConfig.numCtlChannel;i++){
	    iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[hIqn2->aidConfig.firstCtlChannel + i].chan = i;//pktDMA chan 4 is mapped to AID CTL chan 0
	    iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[hIqn2->aidConfig.firstCtlChannel + i].dest = IQN2FL_AID2_CTL; //destination is AID CTL
	    iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[hIqn2->aidConfig.firstCtlChannel + i].arb_pri = 2;
	    iqs2Setup.iqs2_egress_chan_cfg.egr_pktdma_cfg[hIqn2->aidConfig.firstCtlChannel + i].psi_pri = 2;
	}
	
	//SoC Ingress
    for (j=0; j<IQN2_MAX_NUM_AIL; j++)
    {
        for(i=0;i<hIqn2->ailConfig[j].numLtePdAxC;i++){
            if (j==0)
            {
                iqs2Setup.iqs2_ingress_chan_cfg.ail0_axc_lut_cfg[i + hIqn2->ailConfig[j].firstLteAxC].chan = i;//AIL0 chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
                iqs2Setup.iqs2_ingress_chan_cfg.ail0_axc_lut_cfg[i + hIqn2->ailConfig[j].firstLteAxC].dest = IQN2FL_PKTDMA;//destination is PktDMA
            } else if (j==1) {
                iqs2Setup.iqs2_ingress_chan_cfg.ail1_axc_lut_cfg[i + hIqn2->ailConfig[j].firstLteAxC].chan = i;//AIL0 chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
                iqs2Setup.iqs2_ingress_chan_cfg.ail1_axc_lut_cfg[i + hIqn2->ailConfig[j].firstLteAxC].dest = IQN2FL_PKTDMA;//destination is PktDMA
            }
        }

        for(i=0;i<hIqn2->ailConfig[j].numCtlChannel;i++){
            if (j==0)
            {
                iqs2Setup.iqs2_ingress_chan_cfg.ail0_ctl_lut_cfg[i].chan = i + hIqn2->ailConfig[j].firstCtlChannel;//AIL0 chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
                iqs2Setup.iqs2_ingress_chan_cfg.ail0_ctl_lut_cfg[i].dest = IQN2FL_PKTDMA;//destination is PktDMA
            } else if (j==1) {
                iqs2Setup.iqs2_ingress_chan_cfg.ail1_ctl_lut_cfg[i].chan = i + hIqn2->ailConfig[j].firstCtlChannel;//AIL0 chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
                iqs2Setup.iqs2_ingress_chan_cfg.ail1_ctl_lut_cfg[i].dest = IQN2FL_PKTDMA;//destination is PktDMA
            }
        }

        for(i=0;i<hIqn2->ailConfig[j].numWcdmaPdAxC;i++){
            if (j==0)
            {
                iqs2Setup.iqs2_ingress_chan_cfg.ail0_axc_lut_cfg[i + hIqn2->ailConfig[j].firstWcdmaAxC].chan = i;//AIL0 chan 0 ~ 15 is mapped to DIO chan 0 ~ 15
                iqs2Setup.iqs2_ingress_chan_cfg.ail0_axc_lut_cfg[i + hIqn2->ailConfig[j].firstWcdmaAxC].dest = IQN2FL_DIO2;//destination is DIO
            } else if(j==1){
                iqs2Setup.iqs2_ingress_chan_cfg.ail1_axc_lut_cfg[i + hIqn2->ailConfig[j].firstWcdmaAxC].chan = i;//AIL0 chan 0 ~ 15 is mapped to DIO chan 0 ~ 15
                iqs2Setup.iqs2_ingress_chan_cfg.ail1_axc_lut_cfg[i + hIqn2->ailConfig[j].firstWcdmaAxC].dest = IQN2FL_DIO2;//destination is DIO
            }
        }
    }

    for(i=0;i<hIqn2->aidConfig.numWcdmaIngressAxC;i++){
        iqs2Setup.iqs2_ingress_chan_cfg.aid2_axc_lut_cfg[i + hIqn2->aidConfig.firstWcdmaAxC].chan = i;//AID chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
        iqs2Setup.iqs2_ingress_chan_cfg.aid2_axc_lut_cfg[i + hIqn2->aidConfig.firstWcdmaAxC].dest = IQN2FL_DIO2;//destination is PktDMA
    }
	
	for(i=0;i<hIqn2->aidConfig.numLteIngressAxC;i++){
		if(hIqn2->aidConfig.lteDio==1)
		{
			iqs2Setup.iqs2_ingress_chan_cfg.aid2_axc_lut_cfg[i + hIqn2->aidConfig.firstLteAxC].chan = i;//AID chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
			iqs2Setup.iqs2_ingress_chan_cfg.aid2_axc_lut_cfg[i + hIqn2->aidConfig.firstLteAxC].dest = IQN2FL_DIO2;//destination is PktDMA
		} else {
			iqs2Setup.iqs2_ingress_chan_cfg.aid2_axc_lut_cfg[i + hIqn2->aidConfig.firstLteAxC].chan = i;//AID chan 0 ~ 3 is mapped to PktDMA chan 0 ~ 3
			iqs2Setup.iqs2_ingress_chan_cfg.aid2_axc_lut_cfg[i + hIqn2->aidConfig.firstLteAxC].dest = IQN2FL_PKTDMA;//destination is PktDMA
		}
	}
	
	for(i=0;i<hIqn2->aidConfig.numCtlChannel;i++){
		iqs2Setup.iqs2_ingress_chan_cfg.aid2_ctl_lut_cfg[i].chan = i + hIqn2->aidConfig.firstCtlChannel;//AID CTL chan 0 is mapped to PktDMA chan 4
		iqs2Setup.iqs2_ingress_chan_cfg.aid2_ctl_lut_cfg[i].dest = IQN2FL_PKTDMA;//destination is PktDMA
	}

	iqs2Setup.iqs2_ingress_cfg.aid2_axc_cfg_pri = 0;
	iqs2Setup.iqs2_ingress_cfg.aid2_ctl_cfg_pri = 2;
	//iqs2Setup.iqs2_ingress_cfg.pktdma_cfg_pb_sel = 1;
	iqs2Setup.iqs2_ingress_cfg.aid2_axc_cfg_allow_pushback = 0;
	iqs2Setup.iqs2_ingress_cfg.aid2_ctl_cfg_allow_pushback = 1;

	if (hIqn2->aidConfig.aidEnable == 0) { // AIL
		iqs2Setup.iqs2_ingress_cfg.pktdma_cfg_pb_sel = 1;//pb_at_3qtr_full
	} else { // AID
        iqs2Setup.iqs2_ingress_cfg.pktdma_cfg_pb_sel = 0;//pb_at_half_full
	}


	/**************************************************************************************/
    /************** DIO Setup  ***********************************************************/
	/**************************************************************************************/
    for (j=0;j<IQN2_MAX_NUM_DIO_ENGINE;j++)
    {
        if(hIqn2->dioConfig[j].dioEnable)
            enableDio = 1;
    }
    if (enableDio)
    {
        /********* DIO Core setup ***************************/
#ifdef DEVICE_LE
        dio2Setup.dio2_global_cfg_rsa_big_endian  = 0;//little endian order for UL RSA data
#else
        dio2Setup.dio2_global_cfg_rsa_big_endian  = 1;//little endian order for UL RSA data
#endif
        dio2Setup.dio2_global_cfg_vbusm_priority = 0; //highest priority (0 = highest .. 7 = lowest)

        //DIO Egress core
        for (j=0;j<IQN2_MAX_NUM_DIO_ENGINE;j++)
        {
            if(hIqn2->dioConfig[j].numPeDBCH != 0)
            {
                dio2Setup.dio2_core_egress.bcn_table_sel[j] = IQN2FL_TABLE_A;//Table A
                dio2Setup.dio2_core_egress.dma_num_axc[j] = hIqn2->dioConfig[j].numPeDBCH - 1;//Set N-1

                if 	(hIqn2->dioConfig[j].egRsaOn == 1)
                {
                    dio2Setup.dio2_core_egress.dma_cfg0[j].dma_num_qwd = IQN2FL_2QW;//2 QWD for UL
                    dio2Setup.dio2_core_egress.dma_cfg0[j].dma_brst_ln = IQN2FL_2QW;//2 QWD per burst
                    dio2Setup.dio2_core_egress.dma_cfg0[j].rsa_cnvrt_en = 1;
                } else if (hIqn2->dioConfig[j].lteMode == 1){
                	dio2Setup.dio2_core_egress.dma_cfg0[j].dma_num_qwd = IQN2FL_4QW;//1 QWD for DL
					dio2Setup.dio2_core_egress.dma_cfg0[j].dma_brst_ln = IQN2FL_4QW;//1 QWD per burst
					dio2Setup.dio2_core_egress.dma_cfg0[j].rsa_cnvrt_en = 0;
                } else {
                    dio2Setup.dio2_core_egress.dma_cfg0[j].dma_num_qwd = IQN2FL_1QW;//1 QWD for DL
                    dio2Setup.dio2_core_egress.dma_cfg0[j].dma_brst_ln = IQN2FL_1QW;//1 QWD per burst
                    dio2Setup.dio2_core_egress.dma_cfg0[j].rsa_cnvrt_en = 0;
                }
                dio2Setup.dio2_core_egress.dma_cfg0[j].dma_eng_en = 1;
                if (hIqn2->dioConfig[j].usedWithTAC == 0)
                {
                    dio2Setup.dio2_core_egress.dma_cfg0[j].dma_num_blks = hIqn2->dioConfig[j].outNumBlock - 1;//DIO_NUM_BLOCK -1;//Set N-1
                    if (hIqn2->dioConfig[j].lteMode == 1){
                    	dio2Setup.dio2_core_egress.dma_cfg1_dma_blk_addr_stride[j] = 4;
                    } else {
                    	dio2Setup.dio2_core_egress.dma_cfg1_dma_blk_addr_stride[j] = (dio2Setup.dio2_core_egress.dma_cfg0[j].dma_num_qwd)+1;
                    }
                } else {
                    if (hIqn2->dioConfig[j].outNumBlock <= 64)
                        dio2Setup.dio2_core_egress.dma_cfg0[j].dma_num_blks = hIqn2->dioConfig[j].outNumBlock - 1;//DIO_NUM_BLOCK -1;//Set N-1
                    else
                        dio2Setup.dio2_core_egress.dma_cfg0[j].dma_num_blks = 64 - 1;//DIO_NUM_BLOCK -1;//Set N-1
                    dio2Setup.dio2_core_egress.dma_cfg1_dma_blk_addr_stride[j] = 0;
                }
                for(i=0;i<hIqn2->dioConfig[j].numPeDBCH;i++)
                    dio2Setup.dio2_e_aog_ram_mmr_axc_off_cfg_4samp_offset[i] = 0;//similar to aif2 dio offset
                for(i=0;i<hIqn2->dioConfig[j].numPeDBCH;i++){
                    src[j][i]  = ((uint32_t) hIqn2->dioConfig[j].out[i]) >> 4;
                    if (hIqn2->dioConfig[j].usedWithTAC == 0)
                        dio2Setup.dio2_e_dbcntx_ram_mmr[j].dma_vbus_base_addr_axc[i] = src[j][i];//DL has 1QWD offset for each AxC
                    else
                        dio2Setup.dio2_e_dbcntx_ram_mmr[j].dma_vbus_base_addr_axc[i] = (0x23200000 + ((i + hIqn2->dioConfig[j].offsetPeDBCH)*((dio2Setup.dio2_core_egress.dma_cfg0[j].dma_num_qwd)+1)*16))>>4;//DL has 1QWD offset for each AxC//DL has 1QWD offset for each AxC
                    dio2Setup.dio2_e_dbcntx_ram_mmr[j].ch_id[i] = i + hIqn2->dioConfig[j].offsetPeDBCH;
                    dio2Setup.dio2_e_dbcntx_ram_mmr[j].ch_en[i] = 1;
                }
            }
        }

        //DIO Ingress core
        for (j=0;j<IQN2_MAX_NUM_DIO_ENGINE;j++)
        {
            if(hIqn2->dioConfig[j].numPdDBCH != 0)
            {
                dio2Setup.dio2_core_ingress.bcn_table_sel[j] = IQN2FL_TABLE_A;//Table A
                dio2Setup.dio2_core_ingress.dma_num_axc[j] = hIqn2->dioConfig[j].numPdDBCH - 1;//Set N-1

                if 	(hIqn2->dioConfig[j].rsaOn == 1)
                {
                    dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_num_qwd = IQN2FL_2QW;//2 QWD for UL
                    dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_brst_ln = IQN2FL_2QW;//2 QWD per burst
                    dio2Setup.dio2_core_ingress.dma_cfg0[j].rsa_cnvrt_en = 1;
                } else if (hIqn2->dioConfig[j].lteMode == 1){
                	dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_num_qwd = IQN2FL_4QW;//1 QWD for DL
					dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_brst_ln = IQN2FL_4QW;//1 QWD per burst
					dio2Setup.dio2_core_ingress.dma_cfg0[j].rsa_cnvrt_en = 0;
                } else {
                    dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_num_qwd = IQN2FL_1QW;//1 QWD for DL
                    dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_brst_ln = IQN2FL_1QW;//1 QWD per burst
                    dio2Setup.dio2_core_ingress.dma_cfg0[j].rsa_cnvrt_en = 0;
                }
                dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_eng_en = 1;

                if (hIqn2->dioConfig[j].usedWithRAC == 0){
                    dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_num_blks = hIqn2->dioConfig[j].inNumBlock - 1; //DIO_NUM_BLOCK -1;//Set N-1
                    if (hIqn2->dioConfig[j].lteMode == 1){
                    	dio2Setup.dio2_core_ingress.dma_cfg1_dma_blk_addr_stride[j] = 4;//1qwd per AxC
                    } else {
                    	dio2Setup.dio2_core_ingress.dma_cfg1_dma_blk_addr_stride[j] = (dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_num_qwd)+1;//1qwd per AxC
                    }
                } else {
                    // The fact that the RAC FE is mapped with a jump of 0x800 every 8 chips with 4 blocks every 32-chip period
                    //  -> Block Stride is 0x80
                    //  -> Burst Stried is 4
                    dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_num_blks = 4 - 1; //Set N-1
                    dio2Setup.dio2_core_ingress.dma_cfg1_dma_blk_addr_stride[j] = 0x80;//1qwd per AxC
                }
                for(i=0;i<hIqn2->dioConfig[j].numPdDBCH;i++)
                    dio2Setup.dio2_i_axc_off_mmr_cfg_4samp_offset[i] = 0;//similar to aif2 dio offset
                for(i=0;i<hIqn2->dioConfig[j].numPdDBCH;i++){
                    dest[j][i] = ((uint32_t) hIqn2->dioConfig[j].in[i]) >> 4;
                    if (hIqn2->dioConfig[j].usedWithRAC == 0)
                    {
                        dio2Setup.dio2_i_dbcntx_ram_mmr[j].dma_vbus_base_addr_axc[i] = dest[j][i];//DL has 1QWD offset for each AxC
                    } else {
                        dio2Setup.dio2_i_dbcntx_ram_mmr[j].dma_vbus_base_addr_axc[i] = (0x23C00000 + ((i + hIqn2->dioConfig[j].offsetPdDBCH)*((dio2Setup.dio2_core_ingress.dma_cfg0[j].dma_num_qwd)+1)*16))>>4;//DL has 1QWD offset for each AxC
                    }
                    dio2Setup.dio2_i_dbcntx_ram_mmr[j].ch_id[i] = i + hIqn2->dioConfig[j].offsetPdDBCH;
                    dio2Setup.dio2_i_dbcntx_ram_mmr[j].ch_en[i] = 1;
                }
            }
        }


        /** DIO SI setup. Be careful! DIO SI has reversed order of Egress/Ingress when compared to SoC level order ***/

        //DIO SI ingress for SoC level egress operation (matched with AIL egress)
        for(j=0; j<IQN2_MAX_NUM_DIO_ENGINE;j++)
        {
            for(i = 0; i < hIqn2->dioConfig[j].numPeDBCH; i++){
                dio2Setup.dio2_ife_chan_cfg_grp[i+hIqn2->dioConfig[j].offsetPeDBCH].chan_en = 1;
                dio2Setup.dio2_ife_chan_cfg_grp[i+hIqn2->dioConfig[j].offsetPeDBCH].chan_radio_sel = (Iqn2Fl_ChanRadioSel) IQN2_getEgressRadioStandardId(i + hIqn2->dioConfig[j].firstAxC);
                dio2Setup.dio2_ife_chan_cfg_grp[i+hIqn2->dioConfig[j].offsetPeDBCH].chan_axc_offset = 0;//fine AxC offset within QWD level. normally set to zero
            }
        }

        for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
        {
            if((egrGroupInfo[i].g.isPopulated == 1) && ((egrGroupInfo[i].g.isLte == 0) || ((egrGroupInfo[i].g.isLte==1)&&(hIqn2->aidConfig.lteDio==1))))
            {
                dio2Setup.dio2_ife_radio_std_grp.ife_frm_tc_cfg[i].sym_tc = egrRadStdInfo[i].symbolTc;//15 WCDMA slots. Set N-1
                dio2Setup.dio2_ife_radio_std_grp.ife_frm_tc_cfg[i].index_sc = egrRadStdInfo[i].indexSc;
                dio2Setup.dio2_ife_radio_std_grp.ife_frm_tc_cfg[i].index_tc = egrRadStdInfo[i].indexTc;
                if(egrGroupInfo[i].g.isLte == 0)
                {
                	dio2Setup.ife_frm_samp_tc_cfg_samp_tc[egrRadStdInfo[i].indexSc] = FrameMsg[i] - 1;//Set N-1
                } else {
					dio2Setup.ife_frm_samp_tc_cfg_samp_tc[egrRadStdInfo[i].indexSc] = FrameMsg1[i] - 1;//Set N-1
					for(j=1;j<(IQN2_LTE_SYMBOL_NUM);j++){
						dio2Setup.ife_frm_samp_tc_cfg_samp_tc[egrRadStdInfo[i].indexSc+j] = FrameMsg[i]- 1;//frame message terminal count for other 6 normal cyclic prefix LTE symbols
					}
					for(j = IQN2_LTE_SYMBOL_NUM; j < (MBSFN_FACTOR * IQN2_LTE_SYMBOL_NUM); j++) {
						dio2Setup.ife_frm_samp_tc_cfg_samp_tc[egrRadStdInfo[i].indexSc + j] =
						dio2Setup.ife_frm_samp_tc_cfg_samp_tc[egrRadStdInfo[i].indexSc + (j - IQN2_LTE_SYMBOL_NUM)];
					}
                }
            }
        }

        dio2Setup.dio2_iq_idc_rate_ctl_cfg_rate = 0xF;//set idc rate control to max as a default		//FIXME need to look at that value

        //DIO SI egress for SoC level ingress operation (Matched with AIL ingress)
        for(j=0; j<IQN2_MAX_NUM_DIO_ENGINE;j++)
        {
            for(i = 0; i < hIqn2->dioConfig[j].numPdDBCH; i++){
                dio2Setup.dio2_efe_cfg_grp.chan_en[i+hIqn2->dioConfig[j].offsetPdDBCH] = 1;
                dio2Setup.dio2_efe_cfg_grp.chan_radio_sel[i+hIqn2->dioConfig[j].offsetPdDBCH] = (Iqn2Fl_ChanRadioSel) IQN2_getIngressRadioStandardId(i + hIqn2->dioConfig[j].firstAxC);//use radio standard 0 for WCDMA
                dio2Setup.efe_chan_axc_offset_cfg[i+hIqn2->dioConfig[j].offsetPdDBCH] = 0;//for CPRI,this should be matched with AIL PD AxC offset
            }
        }

        for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
        {
			if((ingrGroupInfo[i].g.isPopulated == 1) && ((ingrGroupInfo[i].g.isLte == 0) || ((ingrGroupInfo[i].g.isLte==1)&&(hIqn2->aidConfig.lteDio==1))))
            {
                dio2Setup.dio2_efe_radio_std_grp.efe_frm_tc_cfg[i].sym_tc = ingrRadStdInfo[i].symbolTc;//15 WCDMA slots. Set N-1
                dio2Setup.dio2_efe_radio_std_grp.efe_frm_tc_cfg[i].index_sc = ingrRadStdInfo[i].indexSc;
                dio2Setup.dio2_efe_radio_std_grp.efe_frm_tc_cfg[i].index_tc = ingrRadStdInfo[i].indexTc;
                if(ingrGroupInfo[i].g.isLte == 0)
                {
                	dio2Setup.efe_frm_samp_tc_cfg[ingrRadStdInfo[i].indexSc] = FrameMsg[i] - 1;//Set N-1
                } else {
                	dio2Setup.efe_frm_samp_tc_cfg[ingrRadStdInfo[i].indexSc] = FrameMsg1[i] - 1;//Set N-1
                	for(j=1;j<(IQN2_LTE_SYMBOL_NUM);j++){
                		dio2Setup.efe_frm_samp_tc_cfg[ingrRadStdInfo[i].indexSc+j] = FrameMsg[i] - 1;//Set N-1
                	}
                }
            }
        }

        for(j=0; j<IQN2_MAX_NUM_DIO_ENGINE;j++)
        {
            for(i=0;i<hIqn2->dioConfig[j].numPdDBCH;i++){
                dio2Setup.efe_chan_tdm_lut_cfg_chan_idx_cfg[i+hIqn2->dioConfig[j].offsetPdDBCH] = i+hIqn2->dioConfig[j].offsetPdDBCH;
                dio2Setup.efe_chan_tdm_lut_cfg_chan_idx_en_cfg[i+hIqn2->dioConfig[j].offsetPdDBCH] = 1;
            }
            if (hIqn2->dioConfig[j].numPdDBCH != 0)
            {
                dio2Setup.efe_rad_std_sch_cfg_tdm_start[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = hIqn2->dioConfig[j].offsetPdDBCH;
                dio2Setup.efe_rad_std_sch_cfg_tdm_len[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = hIqn2->dioConfig[j].numPdDBCH;
                dio2Setup.efe_rad_std_sch_cfg_tdm_en[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = 1;
            }
        }

        /******* DIO uAT setup **********************************/
        for (j=0;j<IQN2_MAX_NUM_DIO_ENGINE;j++)
        {
            if(hIqn2->dioConfig[j].numPeDBCH != 0)
            {
                //DIO Core Egress RADT0 with event16 (Used for Soc level Egress operation)
                if ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76))
                {
                    dio2Setup.uat_dio_egr_radt_tc_cfg_val[j] = IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER;//CPRI frame length with 245.76 MHz clock
                } else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64){
                	dio2Setup.uat_dio_egr_radt_tc_cfg_val[j] = IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER;
                } else if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI){  //OBSAI
                    dio2Setup.uat_dio_egr_radt_tc_cfg_val[j] = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;//OBSAI frame length with 307.20 MHz clock
                }
                if(hIqn2->dioConfig[j].lteMode == 1)
                {
                	dio2Setup.uat_evt_clk_cnt_tc_cfg_val[16+j] = (((dio2Setup.uat_dio_egr_radt_tc_cfg_val[j]+1)/FrameMsgTot[IQN2_getEgressRadioStandardId(hIqn2->dioConfig[j].firstAxC)]/20) * 16) - 1;//16 sample event for Lte
                } else {
					if (hIqn2->dioConfig[j].egRsaOn == 1)
						dio2Setup.uat_evt_clk_cnt_tc_cfg_val[16+j] = (((dio2Setup.uat_dio_egr_radt_tc_cfg_val[j]+1)/FrameMsgTot[IQN2_getEgressRadioStandardId(hIqn2->dioConfig[j].firstAxC)]/15) * 8) - 1;//8 sample event for UL
					else
						dio2Setup.uat_evt_clk_cnt_tc_cfg_val[16+j] = (((dio2Setup.uat_dio_egr_radt_tc_cfg_val[j]+1)/FrameMsgTot[IQN2_getEgressRadioStandardId(hIqn2->dioConfig[j].firstAxC)]/15) * 4) - 1;//4 sample event for DL
                }
                dio2Setup.uat_dio_egr_radt_offset_cfg_val[j] = 0;//not used for DIAG_SYNC test mode
                dio2Setup.uat_evt_radt_cmp_cfg_val[16+j] = 0;//No initial delay

            }
        }

        //DIO SI Ingress RADT is not used (SoC level DIO egress doesn't require event)

        //DIO SI Egress RADT0 with event0 (Used for SoC level Ingress operation)
        for (j=0;j<IQN2_MAX_NUM_DIO_ENGINE;j++)
        {
            if(hIqn2->dioConfig[j].numPdDBCH != 0)
            {
				if ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76))
				{
					dio2Setup.uat_egr_radt_tc_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER;//CPRI frame length with 245.76 MHz clock
				} else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64){
					dio2Setup.uat_egr_radt_tc_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER;//DFE frame length with 368.64 MHz clock
				} else if(hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI){		//OBSAI
					dio2Setup.uat_egr_radt_tc_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;//OBSAI frame length with 307.20 MHz clock
				}
				dio2Setup.uat_egr_radt_offset_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = 0;//not used for DIAG_SYNC test mode
				if (hIqn2->aidConfig.aidEnable)
					dio2Setup.uat_evt_radt_cmp_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = hIqn2->aidConfig.siDelay + hIqn2->AxCconfig[hIqn2->dioConfig[j].firstAxC].ingressAxCOffset + 500;//(AID2 SI offset) + 500 (ingress pipe delay)
				else
					dio2Setup.uat_evt_radt_cmp_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = hIqn2->ailConfig[0].pe2Offset + 500;//pi + 500 (ingress pipe delay)

				if(hIqn2->dioConfig[j].lteMode == 1){
					dio2Setup.uat_evt_clk_cnt_tc_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = ((dio2Setup.uat_egr_radt_tc_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)]+1) / FrameMsgTot[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] / 20 * 4) - 1;//4 sample event
				} else {
					dio2Setup.uat_evt_clk_cnt_tc_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] = ((dio2Setup.uat_egr_radt_tc_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)]+1) / FrameMsgTot[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] / 15 * 4) - 1;//4 sample event
				}
            }
        }

        for (j=0;j<IQN2_MAX_NUM_DIO_ENGINE;j++)
        {
            if(hIqn2->dioConfig[j].numPdDBCH != 0)
            {
                //DIO Core Ingress RADT0 with event19 (Used for Soc level Ingress operation)
                if ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76)) {
                    dio2Setup.uat_dio_ing_radt_tc_cfg_val[j] = IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER;//CPRI frame length with 245.76 MHz clock
                } else if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI) {
                	dio2Setup.uat_dio_ing_radt_tc_cfg_val[j] = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;//OBSAI frame length with 307.20 MHz clock
                } else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64) {
                	dio2Setup.uat_dio_ing_radt_tc_cfg_val[j] = IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER;//DFE frame length with 368.64 MHz clock
                }

                if (hIqn2->dioConfig[j].lteMode == 1) {
                	dio2Setup.uat_evt_clk_cnt_tc_cfg_val[19+j] = (((dio2Setup.uat_dio_ing_radt_tc_cfg_val[j]+1)/FrameMsgTot[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)]/20) * 16) - 1;
                	dio2Setup.uat_evt_radt_cmp_cfg_val[19+j] = dio2Setup.uat_evt_radt_cmp_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] + (dio2Setup.uat_evt_clk_cnt_tc_cfg_val[19+j]);//DIO SI delay + n chip delay
                } else {
					if (hIqn2->dioConfig[j].rsaOn == 1)
					{
						dio2Setup.uat_evt_clk_cnt_tc_cfg_val[19+j] = (((dio2Setup.uat_dio_ing_radt_tc_cfg_val[j]+1)/FrameMsgTot[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)]/15) * 8) - 1;
						dio2Setup.uat_evt_radt_cmp_cfg_val[19+j] = dio2Setup.uat_evt_radt_cmp_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] + 480;//DIO SI delay + 480 = 1230 clock delay for 8 chip DMA
					} else {
						dio2Setup.uat_evt_clk_cnt_tc_cfg_val[19+j] = (((dio2Setup.uat_dio_ing_radt_tc_cfg_val[j]+1)/FrameMsgTot[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)]/15) * 4) - 1;
						dio2Setup.uat_evt_radt_cmp_cfg_val[19+j] = dio2Setup.uat_evt_radt_cmp_cfg_val[IQN2_getIngressRadioStandardId(hIqn2->dioConfig[j].firstAxC)] + 240;//DIO SI delay + 240 = 1400 clock delay for 4 chip DMA
					}
                }

                dio2Setup.uat_dio_ing_radt_offset_cfg_val[j] = 0;//not used for DIAG_SYNC test mode
            }
        }
    }

	/**************************************************************************************/
	/************** AIL0 Setup  ***********************************************************/
	/**************************************************************************************/
    for (k=0; k<IQN2_MAX_NUM_AIL; k++)
    {
        if (hIqn2->ailConfig[k].ailEnable == 1)
        {
            if (k == 0)
            {
                ail0Setup.ailInstNum   = IQN2FL_AIL_0;
                ail0Setup.pAilEgrSetup = &ailEgrSetup[k];
                ail0Setup.pAilIgrSetup = &ailIgrSetup[k];
                ail0Setup.pAilPeSetup  = &ailPeSetup[k];
                ail0Setup.pAilPdSetup  = &ailPdSetup[k];
                ail0Setup.pAilUatSetup = &ailUatSetup[k];
                ail0Setup.pAilPhySetup = &ailPhySetup[k];
            } else if (k == 1){
                ail1Setup.ailInstNum   = IQN2FL_AIL_1;
                ail1Setup.pAilEgrSetup = &ailEgrSetup[k];
                ail1Setup.pAilIgrSetup = &ailIgrSetup[k];
                ail1Setup.pAilPeSetup  = &ailPeSetup[k];
                ail1Setup.pAilPdSetup  = &ailPdSetup[k];
                ail1Setup.pAilUatSetup = &ailUatSetup[k];
                ail1Setup.pAilPhySetup = &ailPhySetup[k];
            }


            uint32_t linkRate = 0;

            memset(&dbmxPe[0], 0, sizeof(dbmxPe));
            memset(&dbmxPd[0], 0, sizeof(dbmxPd));
            memset(&kncPe[0], 0, sizeof(kncPe));
            memset(&kncPd[0], 0, sizeof(kncPd));

            if (hIqn2->ailConfig[k].linkRate == IQN2FL_LINK_RATE_2x)
                linkRate = 2;
            else if (hIqn2->ailConfig[k].linkRate == IQN2FL_LINK_RATE_4x)
                linkRate = 4;
            else if (hIqn2->ailConfig[k].linkRate == IQN2FL_LINK_RATE_8x)
                linkRate = 8;

            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                if (egrGroupInfo[i].g.isPopulated)
                {
                    if (egrGroupInfo[i].g.isLte == 0)
                    {
                        dbmxPe[i] = hIqn2->ailConfig[k].numWcdmaPeAxC;
                        kncPe[i] = hIqn2->ailConfig[k].numWcdmaPeAxC;
                    } else {
                        for (j=hIqn2->ailConfig[k].firstLteAxC;j<(hIqn2->ailConfig[k].numLtePeAxC+hIqn2->ailConfig[k].firstLteAxC);j++)
                        {
                            if(i == getEgrGroupId(j))
                            {
                                if(hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
                                {
                                    if(hIqn2->AxCconfig[j].cpriPackMode == IQN2_LTE_CPRI_0b1)
                                    {
                                        dbmxPe[i] += 1;
                                        kncPe[i] += 1;
                                    } else if (hIqn2->AxCconfig[j].cpriPackMode == IQN2_LTE_CPRI_6b6){
                                        dbmxPe[i] += 1 * hIqn2->AxCconfig[j].cpriPackMode;
                                        kncPe[i] += 8;
                                    } else {
                                        dbmxPe[i] += 1 * hIqn2->AxCconfig[j].cpriPackMode;
                                        kncPe[i] += 1 * hIqn2->AxCconfig[j].cpriPackMode;
                                    }
                                } else {
                                    dbmxPe[i] += 1;
                                    kncPe[i]  += 1;
                                }
                            }
                        }
                    }
                }
                if (ingrGroupInfo[i].g.isPopulated)
                {
                    if (ingrGroupInfo[i].g.isLte == 0)
                    {
                        dbmxPd[i] = hIqn2->ailConfig[k].numWcdmaPdAxC;
                        kncPd[i] = hIqn2->ailConfig[k].numWcdmaPdAxC;
                    } else {
                        for (j=hIqn2->ailConfig[k].firstLteAxC;j<(hIqn2->ailConfig[k].numLtePdAxC+hIqn2->ailConfig[k].firstLteAxC);j++)
                        {

                            if (i == getIngrGroupId(j))
                            {
                                if(hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
                                {
                                    if(hIqn2->AxCconfig[j].cpriPackMode == IQN2_LTE_CPRI_0b1)
                                    {
                                        dbmxPd[i] += 1;
                                        kncPd[i]  += 1;
                                    } else if (hIqn2->AxCconfig[j].cpriPackMode == IQN2_LTE_CPRI_6b6) {
                                        dbmxPd[i] += 1 * hIqn2->AxCconfig[j].cpriPackMode;
                                        kncPd[i]  += 8;
                                    } else {
                                        dbmxPd[i] += 1 * hIqn2->AxCconfig[j].cpriPackMode;
                                        kncPd[i]  += 1 * hIqn2->AxCconfig[j].cpriPackMode;
                                    }
                                } else {
                                    dbmxPd[i] += 1;
                                    kncPd[i]  += 1;
                                }
                            }
                        }
                    }
                }
            }

            dbmxPeTotal = 0;
            dbmxPdTotal = 0;
            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                dbmxPeTotal += dbmxPe[i];
                dbmxPdTotal += dbmxPd[i];
            }

            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                if(ingrGroupInfo[i].g.isPopulated)
                {
                    if((hIqn2->AxCconfig[ingrGroupInfo[i].g.startAxC].inboundDataWidth == IQN2FL_DATA_WIDTH_8_BIT) || (hIqn2->AxCconfig[ingrGroupInfo[i].g.startAxC].inboundDataWidth == IQN2FL_DATA_WIDTH_16_BIT))
                        dataWidth = 16;
                    else
                        dataWidth = 15;
                    containercount += dbmxPd[i]*15;
                }
            }
            maxcontainers = dbmxPdTotal + (((60 * linkRate) - containercount) / dataWidth);

            /*********** Phy configuration **************************/
            ailPhySetup[k].ail_phy_glb_cfg.obsai_cpri = hIqn2->protocol; //CPRI or OBSAI
            ailPhySetup[k].ail_phy_glb_cfg.link_rate = hIqn2->ailConfig[k].linkRate; //IQN2FL_LINK_RATE_4x; //4x link rate

            ailPhySetup[k].ail_phy_rt_cfg.config = 2;//transmit mode
            ailPhySetup[k].ail_phy_rt_cfg.em_en = 0;//empty msg disable

            ailPhySetup[k].ail_phy_ci_co_lut_cfg.phy_ci_lut_cfg_sel = 0;//Select table A
            ailPhySetup[k].ail_phy_ci_co_lut_cfg.phy_co_lut_cfg_sel = 0;//Select table A

            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                if (egrGroupInfo[i].g.isPopulated)
                {
                    if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
                    {
                        ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_count = dbmxPe[i] - 1;
                        ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_type = hIqn2->AxCconfig[(egrGroupInfo[i].g.startAxC)].outboundDataWidth;//15 bit sample
                    } else {
                        if (egrGroupInfo[i].g.isLte)
                            ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_count = (dbmxPe[i] * linkRate) - 1;//sample count (n-1)	 define
                        else
                            ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_count = dbmxPe[i] - 1;
                        ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_type = hIqn2->AxCconfig[(egrGroupInfo[i].g.startAxC)].outboundDataWidth;//15 bit sample
                    }
                    ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_last = 0;//sample last
                }
            }
            if(getMaxEgrGroupId()!=0)
                ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[(getMaxEgrGroupId()-1)].smpl_last = 1;//sample last

            if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
            {
                ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[(getMaxEgrGroupId()-1)].smpl_count += maxcontainers - dbmxPeTotal;
            }

            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                if (ingrGroupInfo[i].g.isPopulated)
                {
                    if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
                    {
                        ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_count = dbmxPd[i] - 1;
                        ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_type = hIqn2->AxCconfig[(ingrGroupInfo[i].g.startAxC)].inboundDataWidth;//15 bit sample
                    } else {
                        if(ingrGroupInfo[i].g.isLte)
                            ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_count = (dbmxPd[i] * linkRate) - 1;//sample count (n-1)
                        else
                            ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_count = dbmxPd[i] - 1;//sample count (n-1)
                        ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_type = hIqn2->AxCconfig[(ingrGroupInfo[i].g.startAxC)].inboundDataWidth;//15 bit sample
                    }
                    ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_last = 0;//sample last
                }
            }
            if(getMaxIngrGroupId()!=0)
                ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[(getMaxIngrGroupId()-1)].smpl_last = 1;//sample last
            if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
            {
                ailPhySetup[k].ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[(getMaxIngrGroupId()-1)].smpl_count += maxcontainers - dbmxPdTotal;
            }

            ailPhySetup[k].ail_phy_tm_regs.phy_tm_cfg_en = 1;//TM enable
            if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
            {
                ailPhySetup[k].ail_phy_tm_regs.phy_tm_cpri_scr_ctrl_cfg_seed_value = 0;//zero means scrambler disabled
                ailPhySetup[k].ail_phy_tm_regs.phy_tm_cpri_ptrp_cfg_ptr_p = 20;//Set CPRI pointer p
                ailPhySetup[k].ail_phy_tm_regs.phy_tm_cpri_version_cfg_prot_vers = 0x1;//Set CPRI version
            } else {		//OBSAI
                ailPhySetup[k].ail_phy_tm_regs.phy_tm_ctrl_cfg_los_en = 0;//Loss of Signal disabled
                if (hIqn2->ailConfig[k].numLtePeAxC == 0)
                {
                    ailPhySetup[k].ail_phy_tm_regs.phy_tm_scr_ctrl_cfg_seed_value = 3;//Set OBSAI SCR seed value
                    ailPhySetup[k].ail_phy_tm_regs.phy_tm_scr_ctrl_cfg_scr_en = 0;//OBSAI SCR disabled
                } else {
                    ailPhySetup[k].ail_phy_tm_regs.phy_tm_scr_ctrl_cfg_seed_value = 1;//Set OBSAI SCR seed value
                    ailPhySetup[k].ail_phy_tm_regs.phy_tm_scr_ctrl_cfg_scr_en = 1;//OBSAI SCR disabled
                }
            }

            ailPhySetup[k].ail_phy_rm_regs.phy_rm_cfg_rx_en = 1;//RM enable
            if ((hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI) && (hIqn2->ailConfig[k].numLtePeAxC != 0))
                ailPhySetup[k].ail_phy_rm_regs.phy_rm_dscr_ctrl_cfg_scr_en = 1;//SCR disable  //FIXME for LTE obsai
            else
                ailPhySetup[k].ail_phy_rm_regs.phy_rm_dscr_ctrl_cfg_scr_en = 0;//SCR disable  //FIXME for LTE obsai
            ailPhySetup[k].ail_phy_rm_regs.phy_rm_lcv_ctrl_cfg_en               = FALSE;
            ailPhySetup[k].ail_phy_rm_regs.phy_rm_lcv_ctrl_cfg_los_det_thold    = IQN2_AIL_RM_LOS_DET_THOLD;
            ailPhySetup[k].ail_phy_rm_regs.phy_rm_fsm_sync_cfg_sync_t           = IQN2_AIL_RM_SYNC_THOLD;
            ailPhySetup[k].ail_phy_rm_regs.phy_rm_fsm_sync_cfg_frame_sync_t     = IQN2_AIL_RM_FRAME_SYNC_THOLD;
            ailPhySetup[k].ail_phy_rm_regs.phy_rm_fsm_unsync_cfg_unsync_t       = IQN2_AIL_RM_UNSYNC_THOLD;
            ailPhySetup[k].ail_phy_rm_regs.phy_rm_fsm_unsync_cfg_frame_unsync_t = IQN2_AIL_RM_FRAME_UNSYNC_THOLD;

            /************ PE, PD configuration ***************************************/

            //PE configuration (most PE related configuration is in SI Egress)
            for(i=0;i<(hIqn2->ailConfig[k].numWcdmaPeAxC + hIqn2->ailConfig[k].numLtePeAxC);i++)
                ailPeSetup[k].ail_pe_common.ail_pe_common_chan_cfg[i].rt_ctl = IQN2FL_INSERTPE;//PE insert mode

            //PD configuration
            if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
            {
                for(i=0;i<hIqn2->ailConfig[k].numWcdmaPdAxC;i++){//WCDMA
                    ailPdSetup[k].ail_pd_common[i + hIqn2->ailConfig[k].firstWcdmaAxC].rad_std = getEgrGroupId(i + hIqn2->ailConfig[k].firstWcdmaAxC);;
                    ailPdSetup[k].ail_pd_common[i + hIqn2->ailConfig[k].firstWcdmaAxC].axc_offset = hIqn2->AxCconfig[i + hIqn2->ailConfig[k].firstWcdmaAxC].ingressAxCOffset;//ingress AxC offset
                }

                for(i=0;i<hIqn2->ailConfig[k].numLtePdAxC;i++){//LTE
                    ailPdSetup[k].ail_pd_common[i + hIqn2->ailConfig[k].firstLteAxC].rad_std = getEgrGroupId(i + hIqn2->ailConfig[k].firstLteAxC);
                    ailPdSetup[k].ail_pd_common[i + hIqn2->ailConfig[k].firstLteAxC].axc_offset = hIqn2->AxCconfig[i + hIqn2->ailConfig[k].firstLteAxC].ingressAxCOffset;//ingress AxC offset
                }

                offsetdbmx = 0;
                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    for(j=0;j<kncPd[i];j++){//LTE
                        if(ingrGroupInfo[i].g.isLte)
                            ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_pd_cpri_axc0_cfg[offsetdbmx].cont_lut_grp = getIngrGroupId((j/(kncPd[i]/(ingrGroupInfo[i].g.endAxC - ingrGroupInfo[i].g.startAxC))) + ingrGroupInfo[i].g.startAxC);
                        else
                            ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_pd_cpri_axc0_cfg[offsetdbmx].cont_lut_grp = getIngrGroupId(j+ingrGroupInfo[i].g.startAxC);
                        ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_pd_cpri_axc0_cfg[offsetdbmx].cont_lut_en = 1;
                        offsetdbmx++;
                    }
                }

                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    if (ingrGroupInfo[i].g.isPopulated)
                    {
                        if(ingrGroupInfo[i].g.isLte)
                        {
                            if(hIqn2->AxCconfig[ingrGroupInfo[i].g.startAxC].cpriPackMode == IQN2_LTE_CPRI_0b1)
                                ailPdSetup[k].ail_pd_cpri_axc_cfg.bub_fsm_cfg_knc[i] = (kncPd[i]*2) - 1;//set n-1
                            else
                                ailPdSetup[k].ail_pd_cpri_axc_cfg.bub_fsm_cfg_knc[i] = kncPd[i] - 1;//set n-1
                        } else {
                            ailPdSetup[k].ail_pd_cpri_axc_cfg.bub_fsm_cfg_knc[i] = kncPd[i] -1;
                        }
                    }

                    if (kncPd[i] != dbmxPd[i])
                        ailPdSetup[k].ail_pd_cpri_axc_cfg.bub_fsm2_cfg_gap_int[i] = kncPd[i] / (kncPd[i] - dbmxPd[i]);//zero means no stuffing samples
                    else
                        ailPdSetup[k].ail_pd_cpri_axc_cfg.bub_fsm2_cfg_gap_int[i] = 0;//zero means no stuffing samples
                }

                offsetdbmx = 0;
                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    if (ingrGroupInfo[i].g.isPopulated)
                    {
                        if(ingrGroupInfo[i].g.isLte)
                        {
                            if(hIqn2->AxCconfig[ingrGroupInfo[i].g.startAxC].cpriPackMode == IQN2_LTE_CPRI_0b1)
                                ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[i].ncont = (dbmxPd[i]*2) - 1;//set n-1
                            else
                                ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[i].ncont = dbmxPd[i] - 1;//set n-1
                        } else {
                            ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[i].ncont = dbmxPd[i] - 1;//set n-1
                        }
                        ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_pd_cpri_tdm_fsm_cfg[i].start_lut = offsetdbmx;
                        ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd_cfg_en[i] = 1;			//enable the associated radio standard
                        ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd1_cfg_bfrm_offset[i] = 0;//CPRI basic frame offset
                        ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd1_cfg_hfrm_offset[i] = 0;//CPRI hyper frame offset
                        ailPdSetup[k].ail_pd_cpri_axc_cfg.ail_iq_pd_cpri_axc_radstd_cfg.radstd2_cfg_bfrm_num[i] = 38399;//set n-1 basic frame number in RAD frame
                        offsetdbmx += dbmxPd[i];
                    }
                }

                offsetdbmx = 0;
                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    if (ingrGroupInfo[i].g.isPopulated)
                    {
                        if(ingrGroupInfo[i].g.isLte)
                        {
                            if(hIqn2->AxCconfig[ingrGroupInfo[i].g.startAxC].cpriPackMode == IQN2_LTE_CPRI_0b1)
                            {
                                for(j=0;j<dbmxPd[i]*2;j++){//LTE
                                    ailPdSetup[k].ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_axc[offsetdbmx] = j % dbmxPd[i] + ingrGroupInfo[i].g.startAxC;
                                    ailPdSetup[k].ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_en[offsetdbmx] = j / dbmxPd[i];
                                    offsetdbmx++;
                                }
                            } else {
                                for(j=0;j<dbmxPd[i];j++){//LTE
                                    ailPdSetup[k].ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_axc[offsetdbmx] = j/hIqn2->AxCconfig[ingrGroupInfo[i].g.startAxC].cpriPackMode + ingrGroupInfo[i].g.startAxC;
                                    ailPdSetup[k].ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_en[offsetdbmx] = 1;
                                    offsetdbmx++;
                                }
                            }
                        } else {
                            for(j=0;j<dbmxPd[i];j++)//LTE
                            {
                                ailPdSetup[k].ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_axc[offsetdbmx] = j + ingrGroupInfo[i].g.startAxC;
                                ailPdSetup[k].ail_pd_cpri_axc_cfg.axc_tdm_lut_cfg_en[offsetdbmx] = 1;
                                offsetdbmx++;
                            }
                        }
                    }
                }

            } else {  //OBSAI

                for(i=0;i<hIqn2->ailConfig[k].numWcdmaPeAxC;i++){//WCDMA
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].adr = i;
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].ts_adr = 0;
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].ts_frmt = 1;//Normal TS
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].ts_mask = 0;//No mask
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].typ = 0x2;//WCDMA
                }

                for(i=0;i<hIqn2->ailConfig[k].numLtePeAxC;i++){//LTE
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].adr = i;
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].ts_adr = 0;
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].ts_frmt = 1;//Normal TS
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].ts_mask = 0;//No mask
                    ailPeSetup[k].ail_pe_obsai_hdr_lut[i].typ = 0xE;//WCDMA
                }

                for(i=0;i<hIqn2->ailConfig[k].numWcdmaPdAxC;i++){//WCDMA
                    ailPdSetup[k].ail_pd_common[i + hIqn2->ailConfig[k].firstWcdmaAxC].rad_std = getEgrGroupId(i + hIqn2->ailConfig[k].firstWcdmaAxC);
                    ailPdSetup[k].ail_pd_common[i + hIqn2->ailConfig[k].firstWcdmaAxC].axc_offset = hIqn2->AxCconfig[i + hIqn2->ailConfig[k].firstWcdmaAxC].ingressAxCOffset;//ingress AxC offset
                }

                for(i=0;i<hIqn2->ailConfig[k].numLtePdAxC;i++){//LTE
                    ailPdSetup[k].ail_pd_common[i + hIqn2->ailConfig[k].firstLteAxC].rad_std = getEgrGroupId(i + hIqn2->ailConfig[k].firstLteAxC);
                    ailPdSetup[k].ail_pd_common[i + hIqn2->ailConfig[k].firstLteAxC].axc_offset = hIqn2->AxCconfig[i + hIqn2->ailConfig[k].firstLteAxC].ingressAxCOffset;//ingress AxC offset
                }

                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    if (ingrGroupInfo[i].g.isPopulated)
                    {
                        ailPdSetup[k].ail_pd_obsai_cfg.frm_tc_cfg[i].index_sc  = ingrRadStdInfo[i].indexSc;//start index
                        ailPdSetup[k].ail_pd_obsai_cfg.frm_tc_cfg[i].index_tc  = ingrRadStdInfo[i].indexTc;//terminal index
                        ailPdSetup[k].ail_pd_obsai_cfg.frm_tc_cfg[i].sym_tc    = ingrRadStdInfo[i].symbolTc;//number of symbols per frame

                        ailPdSetup[k].ail_pd_obsai_cfg.radstd_cfg_axcoffset_win[i] = 400;
                        ailPdSetup[k].ail_pd_obsai_cfg.radt_cfg_tc[i] = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;//OBSAI RADT terminal count

                        if(ingrGroupInfo[i].g.isLte)
                        {
                            if (ingrGroupInfo[i].g.cpType == IQN2_LTE_CPTYPE_NORMAL) {
                                //frame message terminal count for first normal cyclic prefix symbol
                                ailPdSetup[k].ail_pd_obsai_frm_msg_tc_cfg_tc[ingrRadStdInfo[i].indexSc] = (FrameMsg1[i]/ IQN2_NUM_WORDS_PER_QWORD) - 1;
                                //frame message terminal count for other 6 normal cyclic prefix LTE symbols
                                for(j=1;j<(IQN2_LTE_SYMBOL_NUM);j++){
                                    ailPdSetup[k].ail_pd_obsai_frm_msg_tc_cfg_tc[ingrRadStdInfo[i].indexSc + j] = (FrameMsg[i] /IQN2_NUM_WORDS_PER_QWORD) - 1;
                                }
                            } else { //extended
                                //frame message terminal count for all 6 extended cyclic prefix LTE symbols
                                for(j=0 ;j<(IQN2_LTE_SYMBOL_NUM_EXT_CP);j++){
                                    ailPdSetup[k].ail_pd_obsai_frm_msg_tc_cfg_tc[ingrRadStdInfo[i].indexSc + j] = (FrameMsg2[i] /IQN2_NUM_WORDS_PER_QWORD) - 1;
                                }
                            }
                        } else {
                            ailPdSetup[k].ail_pd_obsai_frm_msg_tc_cfg_tc[ingrRadStdInfo[i].indexSc] = (FrameMsg[i] / IQN2_NUM_WORDS_PER_QWORD) - 1;
                        }
                    }
                }

                for(i=0;i<hIqn2->ailConfig[k].numWcdmaPdAxC;i++){
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.chan_cfg[i].gsm_ul = 0;//non GSM data
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_adr = i;
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_mask = 0;
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_ts = 0;
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_type = 2;//WCDMA
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_en = 1;
                }

                for(i=0;i<hIqn2->ailConfig[k].numLtePdAxC;i++){//LTE
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.chan_cfg[i].gsm_ul = 0;//non GSM data
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_adr = i;
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_mask = 0;
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_ts = 0;
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_type = 14;//LTE
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.route_cfg[i].chan_en = 1;
                }

                //WCDMA type lut setup
                if (hIqn2->ailConfig[k].numWcdmaPdAxC != 0)
                {
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.type_lut_cfg[2].ts_format = 1;//normal TS
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.type_lut_cfg[2].obsai_pkt_en = 0;//AxC data
                }

                if (hIqn2->ailConfig[k].numLtePdAxC != 0)
                {
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.type_lut_cfg[14].ts_format = 1;//normal TS
                    ailPdSetup[k].ail_pd_obsai_cfg.lut_cfg.type_lut_cfg[14].obsai_pkt_en = 0;//AxC data
                }

            }

            /************ AIL0 SI configuration ***************************************/
            //SI Egress (include major PE configuration)
            for(i=0;i<hIqn2->ailConfig[k].numWcdmaPeAxC;i++){//WCDMA
                ailEgrSetup[k].ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i + hIqn2->ailConfig[k].firstWcdmaAxC].chan_radio_sel = (Iqn2Fl_ChanRadioSel) getEgrGroupId(i + hIqn2->ailConfig[k].firstWcdmaAxC);     //assign to one of the 8 radio channel
                ailEgrSetup[k].ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i + hIqn2->ailConfig[k].firstWcdmaAxC].axc_fine_offset =0;//fine AxC offset. normally set zero
                ailEgrSetup[k].ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i + hIqn2->ailConfig[k].firstWcdmaAxC].chan_en =1;
                if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI){
                    ailEgrSetup[k].ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i + hIqn2->ailConfig[k].firstWcdmaAxC].chan_obsai_ctl = IQN2FL_CHAN_OBSAI_AXC;//AxC traffic
                    ailEgrSetup[k].ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i + hIqn2->ailConfig[k].firstWcdmaAxC].chan_enet_ctl = IQN2FL_CHAN_ENET_CTL_NON_ENET;
                }
            }

            for(i=0;i<hIqn2->ailConfig[k].numLtePeAxC;i++){//LTE
                ailEgrSetup[k].ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i + hIqn2->ailConfig[k].firstLteAxC].chan_radio_sel = (Iqn2Fl_ChanRadioSel) getEgrGroupId(i + hIqn2->ailConfig[k].firstLteAxC);     //assign to one of the 8 radio channel
                ailEgrSetup[k].ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i + hIqn2->ailConfig[k].firstLteAxC].axc_fine_offset =0;//fine AxC offset. normally set zero
                ailEgrSetup[k].ail_si_iq_efe_config_group.ail_si_iq_efe_chan_config[i + hIqn2->ailConfig[k].firstLteAxC].chan_en =1;
            }

            /* LTE setup*/

            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                if (egrGroupInfo[i].g.isPopulated)
                {
                    ailEgrSetup[k].ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[i].index_sc = egrRadStdInfo[i].indexSc;//start index
                    ailEgrSetup[k].ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[i].index_tc = egrRadStdInfo[i].indexTc;//terminal index
                    ailEgrSetup[k].ail_si_iq_efe_radio_std_group.ail_iq_efe_frm_tc_cfg[i].sym_tc = egrRadStdInfo[i].symbolTc;//number of symbols per frame

                    if(egrGroupInfo[i].g.isLte)
                    {
                        if (egrGroupInfo[i].g.cpType == IQN2_LTE_CPTYPE_NORMAL) {
                            //frame message terminal count for first normal cyclic prefix symbol
                            ailEgrSetup[k].ail_iq_efe_frm_samp_tc_mmr_ram[egrRadStdInfo[i].indexSc] = FrameMsg1[i]- 1;//frame message terminal count for first normal cyclic prefix symbol
                            //frame message terminal count for other 6 normal cyclic prefix LTE symbols
                            for(j=1;j<(IQN2_LTE_SYMBOL_NUM);j++){
                                ailEgrSetup[k].ail_iq_efe_frm_samp_tc_mmr_ram[egrRadStdInfo[i].indexSc + j] = FrameMsg[i]- 1;//frame message terminal count for other 6 normal cyclic prefix LTE symbols
                            }
                            //Copy 1st slot into 2nd slot for MBSFN
                            for(j = IQN2_LTE_SYMBOL_NUM; j < (MBSFN_FACTOR * IQN2_LTE_SYMBOL_NUM); j++) {
                                ailEgrSetup[k].ail_iq_efe_frm_samp_tc_mmr_ram[egrRadStdInfo[i].indexSc + j] =
                                ailEgrSetup[k].ail_iq_efe_frm_samp_tc_mmr_ram[egrRadStdInfo[i].indexSc + (j - IQN2_LTE_SYMBOL_NUM)];
                            }
                        } else { //extended
                            //frame message terminal count for all 6 extended cyclic prefix LTE symbols
                            for(j=0 ;j<(IQN2_LTE_SYMBOL_NUM_EXT_CP);j++){
                                ailEgrSetup[k].ail_iq_efe_frm_samp_tc_mmr_ram[egrRadStdInfo[i].indexSc + j] = FrameMsg2[i]- 1;//frame message terminal count for first normal cyclic prefix symbol
                            }
                        }
                    } else {
                        ailEgrSetup[k].ail_iq_efe_frm_samp_tc_mmr_ram[egrRadStdInfo[i].indexSc] = FrameMsg[i]- 1;
                    }
                }
            }

            for(i=0;i<(hIqn2->ailConfig[k].numWcdmaPeAxC + hIqn2->ailConfig[k].numLtePeAxC);i++){
                if ((hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI) && (hIqn2->ailConfig[k].numLtePeAxC == 0))
                    ailEgrSetup[k].ail_iq_efe_chan_axc_offset[i] = hIqn2->AxCconfig[i].egressAxCOffset;//egress AxC offset (normally matched with PE_STB)
                else 		//OBSAI
                    ailEgrSetup[k].ail_iq_efe_chan_axc_offset[i] = hIqn2->AxCconfig[i].egressAxCOffset;//egress AxC offset
            }

            offsetdbmx = 0;
            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                if (egrGroupInfo[i].g.isPopulated)
                {
                    if(egrGroupInfo[i].g.isLte)
                    {
                        if(hIqn2->AxCconfig[egrGroupInfo[i].g.startAxC].cpriPackMode == IQN2_LTE_CPRI_0b1)
                        {
                            for(j=0;j<dbmxPe[i]*2;j++){//LTE
                                ailEgrSetup[k].ail_iq_pe_axc_tdm_lut_cfg[offsetdbmx].axc = (j % dbmxPe[i]) + egrGroupInfo[i].g.startAxC;
                                ailEgrSetup[k].ail_iq_pe_axc_tdm_lut_cfg[offsetdbmx].en = j / dbmxPe[i];
                                offsetdbmx++;
                            }
                        } else {
                            for(j=0;j<dbmxPe[i];j++){//LTE
                                if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)
                                    ailEgrSetup[k].ail_iq_pe_axc_tdm_lut_cfg[offsetdbmx].axc = j + egrGroupInfo[i].g.startAxC;
                                else
                                    ailEgrSetup[k].ail_iq_pe_axc_tdm_lut_cfg[offsetdbmx].axc = (j / hIqn2->AxCconfig[egrGroupInfo[i].g.startAxC].cpriPackMode) + egrGroupInfo[i].g.startAxC;
                                ailEgrSetup[k].ail_iq_pe_axc_tdm_lut_cfg[offsetdbmx].en = 1;
                                offsetdbmx++;
                            }
                        }
                    } else {
                        for(j=0;j<dbmxPe[i];j++){//WCDMA
                            ailEgrSetup[k].ail_iq_pe_axc_tdm_lut_cfg[offsetdbmx].axc = j + egrGroupInfo[i].g.startAxC;
                            ailEgrSetup[k].ail_iq_pe_axc_tdm_lut_cfg[offsetdbmx].en = 1;
                            offsetdbmx++;
                        }
                    }
                }
            }

            ailEgrSetup[k].phy_en = 1;

            if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
            {
                ailEgrSetup[k].ail_si_iq_e_sch_cpri.axc_cont_tc = maxcontainers - 1;//(n-1)total number of containers in basic frame for AIL0
                offsetdbmx = 0;
                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    if (egrGroupInfo[i].g.isPopulated)
                    {
                        if((hIqn2->AxCconfig[egrGroupInfo[i].g.startAxC].cpriPackMode == IQN2_LTE_CPRI_0b1) && (egrGroupInfo[i].g.isLte == 1)){
                            ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_bub_fsm_cfg.knc[i] = (kncPe[i]*2) - 1;//set n-1
                            ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_tdm_fsm_cfg.ncont[i] = (dbmxPe[i]*2) - 1;//set n-1
                        } else {
                            ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_bub_fsm_cfg.knc[i] = kncPe[i] - 1;//set n-1
                            ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_tdm_fsm_cfg.ncont[i] = dbmxPe[i] - 1;//set n-1
                        }
                        if (kncPe[i] != dbmxPe[i])
                            ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_bub_fsm_cfg.gap_int[i] = kncPe[i] / (kncPe[i] - dbmxPe[i]);//If there are any bubbles present, bub_gap indicates space between bubbles. Program BUB_GAP_INT .GE. KNC for no bubbles.
                        else
                            ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_bub_fsm_cfg.gap_int[i] = 38400;//If there are any bubbles present, bub_gap indicates space between bubbles. Program BUB_GAP_INT .GE. KNC for no bubbles.

                        ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_tdm_fsm_cfg.lutstrt[i] = offsetdbmx;//lut array start position

                        ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd_cfg_en[i]= 1;
                        ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd1_cfg_bfrm_offset[i]= 0;//basic frame offset
                        ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd1_cfg_hfrm_offset[i]= 0;//hyper frame offset
                        ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_radstd_cfg.radstd2_cfg_bfrm_num[i]= 38399;//38400 basic frames in radio frame
                        offsetdbmx += dbmxPe[i];
                    }
                }

                offsetdbmx = 0;
                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    if (egrGroupInfo[i].g.isPopulated)
                    {
                        for(j=0;j<kncPe[i];j++){
                            ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_cont_cfg.lut_grp[offsetdbmx] = i;
                            ailEgrSetup[k].ail_si_iq_e_sch_cpri.ail_iq_pe_cpri_cont_cfg.lut_en[offsetdbmx] = 1;
                            if((hIqn2->AxCconfig[egrGroupInfo[i].g.startAxC].cpriPackMode == IQN2_LTE_CPRI_0b1) && (egrGroupInfo[i].g.isLte == 1))
                            {
#ifdef DEVICE_LE
                                ailEgrSetup[k].ail_iq_edc_reg_grp.dat_swap[offsetdbmx] = 3;//word swap for liitle endian
#else
                                ailEgrSetup[k].ail_iq_edc_reg_grp.dat_swap[offsetdbmx] = 0;//no swap for big endian
#endif
                            }
                            offsetdbmx++;
                        }
                    }
                }
            } else { 		//OBSAI

                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    if (egrGroupInfo[i].g.isPopulated)
                    {
                        ailEgrSetup[k].ail_iq_pe_obsai_modtxrule_cfg[i].rule_mod = 0;
                        ailEgrSetup[k].ail_iq_pe_obsai_modtxrule_cfg[i].rule_ctl_msg = 0;//AxC msg
                        ailEgrSetup[k].ail_iq_pe_obsai_modtxrule_cfg[i].rule_index = 0;
                        ailEgrSetup[k].ail_iq_pe_obsai_modtxrule_cfg[i].rule_en = 1;

                        ailEgrSetup[k].ail_si_iq_e_obsai_dbm_rule[i].dbm_1mult = 0;//set n-1
                        ailEgrSetup[k].ail_si_iq_e_obsai_dbm_rule[i].dbm_x = dbmxPe[i] - 1;//total 16 AxCs
                        ailEgrSetup[k].ail_si_iq_e_obsai_dbm_rule[i].dbm_radstd = i;//radio standard 0
                        ailEgrSetup[k].ail_si_iq_e_obsai_dbm_rule[i].dbm_xbubble = 0;//set n-1
                        ailEgrSetup[k].ail_si_iq_e_obsai_dbm_rule[i].dbm_lutstrt = 0;//start from TDM lut 0
                        ailEgrSetup[k].ail_si_iq_e_obsai_dbm_rule[i].dbm_1size = 0;
                        ailEgrSetup[k].ail_si_iq_e_obsai_dbm_rule[i].dbm_2size = 0;
                        ailEgrSetup[k].ail_si_iq_e_obsai_dbm_rule[i].dbm_en = 1;
                    }
                }
            }

            ailEgrSetup[k].ail_ectl_reg_grp.rate = 7;//ctl channel rate control to max as a default	//setting to 50% BW to match the AID2 setting

            //SI Ingress
            for(i=0;i<hIqn2->ailConfig[k].numWcdmaPdAxC;i++){//WCDMA
                ailIgrSetup[k].ail_iq_ife_chan_config_group[hIqn2->ailConfig[k].firstWcdmaAxC + i].chan_radio_sel = (Iqn2Fl_ChanRadioSel) getIngrGroupId(hIqn2->ailConfig[k].firstWcdmaAxC + i);
                ailIgrSetup[k].ail_iq_ife_chan_config_group[hIqn2->ailConfig[k].firstWcdmaAxC + i].chan_en = 1;
                ailIgrSetup[k].ail_iq_ife_chan_config_group[hIqn2->ailConfig[k].firstWcdmaAxC + i].chan_tdd_frc_off = IQN2FL_CHAN_NO_FRC_OFF_SYM;  //FIXME check this value
                if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)
                {
                    ailIgrSetup[k].ail_iq_ife_chan_config_group[hIqn2->ailConfig[k].firstWcdmaAxC + i].chan_obsai_ctl = IQN2FL_CHAN_OBSAI_AXC;//AxC mode
                    ailIgrSetup[k].ail_iq_ife_chan_config_group[hIqn2->ailConfig[k].firstWcdmaAxC + i].chan_enet_ctl = IQN2FL_CHAN_ENET_CTL_NON_ENET;
                }
            }

            for(i=0;i<hIqn2->ailConfig[k].numLtePdAxC;i++){//LTE
                ailIgrSetup[k].ail_iq_ife_chan_config_group[hIqn2->ailConfig[k].firstLteAxC + i].chan_radio_sel = (Iqn2Fl_ChanRadioSel) getIngrGroupId(hIqn2->ailConfig[k].firstLteAxC + i);
                ailIgrSetup[k].ail_iq_ife_chan_config_group[hIqn2->ailConfig[k].firstLteAxC + i].chan_en = 1;
                ailIgrSetup[k].ail_iq_ife_chan_config_group[hIqn2->ailConfig[k].firstLteAxC + i].chan_tdd_frc_off = IQN2FL_CHAN_NO_FRC_OFF_SYM;
                if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)
                {
                    ailIgrSetup[k].ail_iq_ife_chan_config_group[i].chan_obsai_ctl = IQN2FL_CHAN_OBSAI_AXC;//AxC mode
                }
            }

            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                if (ingrGroupInfo[i].g.isPopulated)
                {
                    ailIgrSetup[k].ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[i].index_sc = ingrRadStdInfo[i].indexSc;
                    ailIgrSetup[k].ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[i].index_tc = ingrRadStdInfo[i].indexTc;
                    ailIgrSetup[k].ail_iq_ife_radio_std_group.ail_iq_ife_frm_tc_cfg[i].sym_tc = ingrRadStdInfo[i].symbolTc;//140 LTE symbols

                    if(ingrGroupInfo[i].g.isLte)
                    {
                        if (ingrGroupInfo[i].g.cpType == IQN2_LTE_CPTYPE_NORMAL) {
                            ailIgrSetup[k].samp_tc[ingrRadStdInfo[i].indexSc] = FrameMsg1[i] - 1;//2208 samples for first 20MHz symbol
                            for(j=1;j<(IQN2_LTE_SYMBOL_NUM);j++){
                                ailIgrSetup[k].samp_tc[ingrRadStdInfo[i].indexSc + j] = FrameMsg[i]- 1;//frame message terminal count for other 6 normal cyclic prefix LTE symbols
                            }
                        } else {
                            for(j=0 ;j<(IQN2_LTE_SYMBOL_NUM_EXT_CP);j++){
                                ailIgrSetup[k].samp_tc[ingrRadStdInfo[i].indexSc + j] = FrameMsg2[i]- 1;
                            }
                        }
                    } else {
                        ailIgrSetup[k].samp_tc[ingrRadStdInfo[i].indexSc] = FrameMsg[i]- 1;//frame message terminal count for other 6 normal cyclic prefix LTE symbols
                    }
                }
            }

            offsetdbmx = 0;
            for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
            {
                if (ingrGroupInfo[i].g.isPopulated)
                {
                    if((hIqn2->AxCconfig[ingrGroupInfo[i].g.startAxC].cpriPackMode == IQN2_LTE_CPRI_0b1) && (ingrGroupInfo[i].g.isLte == 1))
                    {
                        for(j=0;j<dbmxPd[i];j++)
                        {
#ifdef DEVICE_LE
                            ailIgrSetup[k].ail_iq_idc_ch_cfg_grp[offsetdbmx].dat_swap = 3;//word swap for endian change required for LTE1.4MHz symbol
#else
                            ailIgrSetup[k].ail_iq_idc_ch_cfg_grp[offsetdbmx].dat_swap = 0;//no swap for BIG endian.
#endif
                            offsetdbmx++;
                        }
                    }
                }
            }

            ailIgrSetup[k].rate_idc = 15;//rate control to max as a default
            ailIgrSetup[k].rate_ictl = 0;//ctl channel rate control to 0 to overcome a potential issue in IQN2 with pktdma back pressure

        /************ AIL0 CPRI control channel configuration ***************************************/
            if(hIqn2->ailConfig[k].numCtlChannel != 0)
            {
                //Egress CW setup
                for (i=0;i<hIqn2->ailConfig[k].numCtlChannel;i++)
                {
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].delin_sel = 2;//Hyperframe delimiter
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].byte_en = 0xFFFF;//enable whole BW
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].rt_ctl = 1;//insert from PE
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].hf_lut_en = 0;//hf lut is not enabled
                }

                for(i=20;i<64;i++){//CW LUT from pointer p to the 64th row
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i].cw_chan = 0;
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i].cw_en = 1;
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i+64].cw_chan = 0;
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i+64].cw_en = 1;
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i+128].cw_chan = 0;
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i+128].cw_en = 1;
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i+192].cw_chan = 0;
                    ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i+192].cw_en = 1;
                }

                //52th row is reserved area for PORT ID, so it should be disabled
                ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[52].cw_en = 0;
                ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[116].cw_en = 0;
                ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[180].cw_en = 0;
                ailPeSetup[k].ail_pe_cpri_cw.ail_pe_cpri_cw_lut[244].cw_en = 0;

                for (i=0;i<hIqn2->ailConfig[k].numCtlChannel;i++)
                {
#ifdef DEVICE_LE
                    ailEgrSetup[k].ail_ectl_reg_grp.dat_swap[i] = 3;//word swap for little endian
#else
                    ailEgrSetup[k].ail_ectl_reg_grp.dat_swap[i] = 0;//no swap for big endian
#endif
                    ailEgrSetup[k].ail_ectl_pkt_if[i].chan_en = 1;
                }

                //Ingress CW setup
                for (i=0;i<hIqn2->ailConfig[k].numCtlChannel;i++)
                {
                    ailPdSetup[k].ail_pd_cpri_cw_cfg.chan_cfg[i].delin_sel = 2;//Hyperframe delimiter
                    ailPdSetup[k].ail_pd_cpri_cw_cfg.chan_cfg[i].byte_en = 0xFFFF;//enable whole BW
                    ailPdSetup[k].ail_pd_cpri_cw_cfg.chan_cfg[i].hf_lut_en = 0;//hf lut is not used
                    ailPdSetup[k].ail_pd_cpri_cw_cfg.chan_cfg[i].chan_en = 1;//enable CW channel 0
                }

                for(i=20;i<64;i++){//CW LUT from pointer p to the 64th row
                  ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_chan[i] = 0;
                  ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_en[i] = 1;
                  ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_chan[i+64] = 0;
                  ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_en[i+64] = 1;
                  ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_chan[i+128] = 0;
                  ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_en[i+128] = 1;
                  ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_chan[i+192] = 0;
                  ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_en[i+192] = 1;
                }

                //52th row is reserved area for PORT ID, so it should be disabled
                ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_en[52] = 0;
                ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_en[116] = 0;
                ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_en[180] = 0;
                ailPdSetup[k].ail_pd_cpri_cw_cfg.lut_cfg_cw_en[244] = 0;

                for (i=0;i<hIqn2->ailConfig[k].numCtlChannel;i++)
                {
#ifdef DEVICE_LE
                    ailIgrSetup[k].ail_ictl_idc_if.ictl_chan_cfg[i].dat_swap = 3;//word swap for little endian
#else
                    ailIgrSetup[k].ail_ictl_idc_if.ictl_chan_cfg[i].dat_swap = 0;//no swap for big endian
#endif
                    ailIgrSetup[k].ail_ictl_pkt_if_chan_en[i] = 1;
                }
            }
            /************ AIL0 uAT configuration ***************************************/
            if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
                ailUatSetup[k].ail_uat_bcn_tc_cfg_val = IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER;
            else
                ailUatSetup[k].ail_uat_bcn_tc_cfg_val = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;
            ailUatSetup[k].ail_uat_bcn_offset_cfg_val = 0;

            ailUatSetup[k].ail_uat_pe_fb_cfg_val = hIqn2->ailConfig[k].pe2Offset;//PE event
            ailUatSetup[k].ail_uat_rt_fb_cfg_val = hIqn2->ailConfig[k].pe2Offset + 60;//RT event (between 310 ~ 360)
            ailUatSetup[k].ail_uat_tm_fb_cfg_val = hIqn2->ailConfig[k].deltaOffset;//Delta (80 clock from PE)
            ailUatSetup[k].ail_uat_pimin_cfg_val = hIqn2->ailConfig[k].piMin;//PI MIN
            ailUatSetup[k].ail_uat_pimax_cfg_val = hIqn2->ailConfig[k].piMin + 50;//PI MAX

            if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)
            {
                for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
                {
                    if (egrGroupInfo[i].g.isPopulated)
                    {
                        ailUatSetup[k].ail_uat_egr_radt_tc_cfg_val[i] = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;
                        ailUatSetup[k].ail_uat_egr_radt_offset_cfg_val[i] = 0;
                    }
                    if (ingrGroupInfo[i].g.isPopulated)
                    {
                        ailUatSetup[k].ail_uat_ing_radt_tc_cfg_val[i] = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;
                        ailUatSetup[k].ail_uat_ing_radt_offset_cfg_val[i] = 0;
                    }
                }
            }
        }
    }
	/**************************************************************************************/
	/************** AID Setup  ***********************************************************/
	/**************************************************************************************/
	if (hIqn2->aidConfig.aidEnable == 1)
	{
		/** AID SI setup ***/
		//AID SI egress
	    for(i=0;i<hIqn2->aidConfig.numWcdmaEgressAxC;i++){
            aid2Setup.aid2_iq_efe_cfg_grp.chan_en[i + hIqn2->aidConfig.firstWcdmaAxC] = 1;
            aid2Setup.aid2_iq_efe_cfg_grp.chan_radio_sel[i + hIqn2->aidConfig.firstWcdmaAxC] =(Iqn2Fl_ChanRadioSel) getEgrGroupId(i + hIqn2->aidConfig.firstWcdmaAxC);//use radio standard 0 for LTE
            aid2Setup.efe_axc_offset[i + hIqn2->aidConfig.firstWcdmaAxC] = 0; //hIqn2->AxCconfig[i + hIqn2->aidConfig.firstWcdmaAxC].egressAxCOffset;
        }

	    for(i=0;i<hIqn2->aidConfig.numLteEgressAxC;i++){
			aid2Setup.aid2_iq_efe_cfg_grp.chan_en[i + hIqn2->aidConfig.firstLteAxC] = 1;
			aid2Setup.aid2_iq_efe_cfg_grp.chan_radio_sel[i + hIqn2->aidConfig.firstLteAxC] =(Iqn2Fl_ChanRadioSel) getEgrGroupId(i + hIqn2->aidConfig.firstLteAxC);//use radio standard 0 for LTE
			aid2Setup.efe_axc_offset[i + hIqn2->aidConfig.firstLteAxC] = 0; //hIqn2->AxCconfig[i + hIqn2->aidConfig.firstLteAxC].egressAxCOffset;
		}
	
		
        for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
        {
            if (egrGroupInfo[i].g.isPopulated)
            {
                aid2Setup.aid2_iq_efe_radio_std_grp.sym_tc[i] = egrRadStdInfo[i].symbolTc;  //140 LTE symbols Set n-1
                aid2Setup.aid2_iq_efe_radio_std_grp.index_sc[i] = egrRadStdInfo[i].indexSc;
                aid2Setup.aid2_iq_efe_radio_std_grp.index_tc[i] = egrRadStdInfo[i].indexTc;

                if(egrGroupInfo[i].g.isLte)
                {
					if(egrGroupInfo[i].g.cpType == IQN2_LTE_CPTYPE_NORMAL)
					{
						aid2Setup.efe_samp_tc[egrRadStdInfo[i].indexSc] = FrameMsg1[i] - 1;//2208 samples for first 20MHz symbol
						for(j=1;j<(IQN2_LTE_SYMBOL_NUM);j++){
							aid2Setup.efe_samp_tc[egrRadStdInfo[i].indexSc + j] = FrameMsg[i]- 1;//frame message terminal count for other 6 normal cyclic prefix LTE symbols
						}

						for(j = IQN2_LTE_SYMBOL_NUM; j < (MBSFN_FACTOR * IQN2_LTE_SYMBOL_NUM); j++) {
							aid2Setup.efe_samp_tc[egrRadStdInfo[i].indexSc + j] =
							aid2Setup.efe_samp_tc[egrRadStdInfo[i].indexSc + (j - IQN2_LTE_SYMBOL_NUM)];
						}

					} else if (egrGroupInfo[i].g.cpType == IQN2_LTE_CPTYPE_EXTENDED) {
						for(j=0 ;j<(IQN2_LTE_SYMBOL_NUM_EXT_CP);j++){
							aid2Setup.efe_samp_tc[egrRadStdInfo[i].indexSc + j] = FrameMsg2[i]- 1;
						}
					} else {
					    aid2Setup.efe_samp_tc[egrRadStdInfo[i].indexSc] = FrameMsg1[i]- 1;
					}
                } else {
                    aid2Setup.efe_samp_tc[egrRadStdInfo[i].indexSc] = FrameMsg[i]- 1;
                }
            }
        }

        for(i=hIqn2->aidConfig.firstWcdmaAxC;i<hIqn2->aidConfig.numWcdmaEgressAxC + hIqn2->aidConfig.firstWcdmaAxC;i++){//EFE TDM look up table setup
            aid2Setup.efe_chan_index_cfg[i] = i;
            aid2Setup.efe_chan_index_en_cfg[i] = 1;
#ifdef DEVICE_LE												//FIXME lteDio maybe needed for lte too
            if (hIqn2->dioConfig[i/16].egRsaOn == 1)
            {
            	aid2Setup.edc_ch_cfg_dat_swap[i] = 0;//no swap
            	aid2Setup.edc_ch_cfg_iq_order[i] = 2;//swap I and Q
            } else {
            	aid2Setup.edc_ch_cfg_dat_swap[i] = 3;//word swap
            	aid2Setup.edc_ch_cfg_iq_order[i] = 3;//swap I and Q
            }
#else
            if (hIqn2->dioConfig[i/16].egRsaOn == 1)
            {
            	aid2Setup.edc_ch_cfg_dat_swap[i] = 1;//byte swap (to be verified)
            } else {
            	aid2Setup.edc_ch_cfg_dat_swap[i] = 0;//no swap for big endian - The internal format of the data before the data swapper is big endian since the IFE is natively big endian.
            }

#endif
        }

		for(i=hIqn2->aidConfig.firstLteAxC;i<hIqn2->aidConfig.numLteEgressAxC + hIqn2->aidConfig.firstLteAxC;i++){//EFE TDM look up table setup
			aid2Setup.efe_chan_index_cfg[i] = i;
			aid2Setup.efe_chan_index_en_cfg[i] = 1;
#ifdef DEVICE_LE
			aid2Setup.edc_ch_cfg_dat_swap[i] = 3;//word swap
#else
			aid2Setup.edc_ch_cfg_dat_swap[i] = 0;//no swap for big endian - The internal format of the data before the data swapper is big endian since the IFE is natively big endian.
#endif
		}

        for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
        {
            if (egrGroupInfo[i].g.isPopulated)
            {
                aid2Setup.efe_tdm_start[i] = egrGroupInfo[i].g.startAxC;
                aid2Setup.efe_tdm_len[i] = egrGroupInfo[i].g.endAxC - egrGroupInfo[i].g.startAxC + 1;
                aid2Setup.efe_tdm_en[i] = 1;
            }
        }

		//AID SI ingress
        for(i=hIqn2->aidConfig.firstWcdmaAxC;i<hIqn2->aidConfig.numWcdmaIngressAxC + hIqn2->aidConfig.firstWcdmaAxC;i++){
            aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_en = 1;
            aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_radio_sel = (Iqn2Fl_ChanRadioSel) getIngrGroupId(i);//use radio standard 0 for LTE
            aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_axc_offset = 0;//hIqn2->AxCconfig[i].ingressAxCOffset;  //fine AxC offset within QWD level. normally set to zero
#ifdef DEVICE_LE
            if (hIqn2->dioConfig[i/16].rsaOn == 1)
            {
            	aid2Setup.aid2_iq_idc_chan_cfg_grp[i].dat_swap = 0;//no swap
            	aid2Setup.aid2_iq_idc_chan_cfg_grp[i].iq_order = 2;//I and Q swap
            } else {
            	aid2Setup.aid2_iq_idc_chan_cfg_grp[i].dat_swap = 3;//word swap
            	aid2Setup.aid2_iq_idc_chan_cfg_grp[i].iq_order = 3;//I and Q swap
            }
#else
            if (hIqn2->dioConfig[i/16].rsaOn == 1)
            {
            	aid2Setup.aid2_iq_idc_chan_cfg_grp[i].dat_swap = 1;//byte swap (to be verified)
            } else {
            	aid2Setup.aid2_iq_idc_chan_cfg_grp[i].dat_swap = 0;//no swap for big endian - The internal format of the data before the data swapper is big endian since the IFE is natively big endian.
            }			
#endif
        }

		for(i=hIqn2->aidConfig.firstLteAxC;i<hIqn2->aidConfig.numLteIngressAxC + hIqn2->aidConfig.firstLteAxC;i++){
			aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_en = 1;
			aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_radio_sel = (Iqn2Fl_ChanRadioSel) getIngrGroupId(i);//use radio standard 0 for LTE
			aid2Setup.aid2_iq_ife_chan_cfg_grp[i].chan_axc_offset = 0; //hIqn2->AxCconfig[i].ingressAxCOffset;  //fine AxC offset within QWD level. normally set to zero
#ifdef DEVICE_LE
			aid2Setup.aid2_iq_idc_chan_cfg_grp[i].dat_swap = 3;//word swap
#else
			aid2Setup.aid2_iq_idc_chan_cfg_grp[i].dat_swap = 0;//no swap for big endian - The internal format of the data before the data swapper is big endian since the IFE is natively big endian.
#endif
		}
		
        for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
        {
            if (ingrGroupInfo[i].g.isPopulated)
            {
                aid2Setup.aid2_iq_ife_radio_std_grp.sym_tc[i] = ingrRadStdInfo[i].symbolTc;  //140 LTE symbols Set n-1
                aid2Setup.aid2_iq_ife_radio_std_grp.index_sc[i] = ingrRadStdInfo[i].indexSc;
                aid2Setup.aid2_iq_ife_radio_std_grp.index_tc[i] = ingrRadStdInfo[i].indexTc;

                if (ingrGroupInfo[i].g.isLte)
                {
					if (ingrRadStdInfo[i].symbolTc == (IQN2_LTE_FRAME_SYMBOL_NUM-1)) {
						aid2Setup.ife_samp_tc[ingrRadStdInfo[i].indexSc] = FrameMsg1[i] - 1;//2208 samples for first 20MHz symbol
						for(j=1;j<(IQN2_LTE_SYMBOL_NUM);j++){
							aid2Setup.ife_samp_tc[ingrRadStdInfo[i].indexSc + j] = FrameMsg[i]- 1;//frame message terminal count for other 6 normal cyclic prefix LTE symbols
						}
					} else if (ingrRadStdInfo[i].symbolTc == (IQN2_LTE_FRAME_SYMBOL_NUM_EXT_CP - 1)){
						for(j=0 ;j<(IQN2_LTE_SYMBOL_NUM_EXT_CP);j++){
							aid2Setup.ife_samp_tc[ingrRadStdInfo[i].indexSc + j] = FrameMsg2[i]- 1;
						}
					} else {
					    aid2Setup.ife_samp_tc[ingrRadStdInfo[i].indexSc] = FrameMsg1[i]- 1;
					}
                } else {
                    aid2Setup.ife_samp_tc[ingrRadStdInfo[i].indexSc] = FrameMsg[i]- 1;
                }
            }
        }
		
        aid2Setup.idc_rate_ctl_cfg_rate = 0xF; //0xF; //max BW.

		//AID CTL channel setup
		for (i=0;i<hIqn2->aidConfig.numCtlChannel;i++)
		{
			aid2Setup.ectl_chan_en[i] = 1;
			aid2Setup.ectl_channel[i] = 0;//ctl channel DB threshold. 0 means any data in the buffer
            aid2Setup.ictl_pkt_if_chan_en[i] = 1;
#ifdef DEVICE_LE
            aid2Setup.ectl_ch_cfg_dat_swap[i] = 3;//word swap
            aid2Setup.aid2_ictl_idc_if_ch_cfg[i].dat_swap = 3;//word swap
#else
            aid2Setup.ectl_ch_cfg_dat_swap[i] = 0;//no swap for big endian - The internal format of the data before the data swapper is big endian since the IFE is natively big endian.
            aid2Setup.aid2_ictl_idc_if_ch_cfg[i].dat_swap = 0;//no swap
#endif


		}

		aid2Setup.ectl_rate_ctl_cfg = 7;//suppress eCTLBW to 50 %
		aid2Setup.ictl_rate_ctl_cfg_rate = 0;//suppress iCTL BW (this is needed to work-around a potential back pressure issue not handled in iqn2 and coming from pktdma)
			
		/******* AID uAT setup **********************************/
		if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76)
		    frameLength = IQN2_DFE_245_76_CLOCK_COUNT_TC_PHY_TIMER;
		else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64)
		    frameLength = IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER;

        for (i=0;i<IQN2_MAX_NUM_RADIO_STANDARD;i++)
        {
            if (egrGroupInfo[i].g.isPopulated)
            {
                //AID SI Egress RADT0 with event0
                aid2Setup.uat_egr_radt_tc_cfg_val[i] = frameLength;//dfe frame length with 245.76 MHz clock
                aid2Setup.uat_egr_radt_offset_cfg_val[i] = 0;//not used for DIAG_SYNC test mode
                aid2Setup.uat_evt_radt_cmp_cfg_val[i] = hIqn2->aidConfig.siDelay + hIqn2->AxCconfig[egrGroupInfo[i].g.startAxC].egressAxCOffset;//200 clock delay for AID SI event start

                if (egrGroupInfo[i].g.isLte)
                {
                    if (egrGroupInfo[i].g.cpType == IQN2_LTE_CPTYPE_NONE)
                    {
                        aid2Setup.uat_evt_clk_cnt_tc_cfg_val[i] = ((frameLength+1) / FrameMsg1[i] / (egrRadStdInfo[i].symbolTc+1) * 4) - 1;//4 sample event (LTE20MHz)
                    } else {
                        aid2Setup.uat_evt_clk_cnt_tc_cfg_val[i] = ((frameLength+1) / FrameMsgTot[i] / 20 * 4) - 1;//4 sample event (LTE20MHz)
                    }
                } else {
                    aid2Setup.uat_evt_clk_cnt_tc_cfg_val[i] = ((frameLength+1) / FrameMsgTot[i] / 15 * 4) - 1;//
                }
            }
            if (ingrGroupInfo[i].g.isPopulated)
            {
                //AID SI Ingress RADT0 with event8
                aid2Setup.uat_ing_radt_tc_cfg_val[i] = frameLength;//dfe frame length with 245.76 MHz clock
                aid2Setup.uat_ing_radt_offset_cfg_val[i] = 0;//not used for DIAG_SYNC test mode
                aid2Setup.uat_evt_radt_cmp_cfg_val[8 + i] = hIqn2->aidConfig.siDelay + 8 + hIqn2->AxCconfig[ingrGroupInfo[i].g.startAxC].ingressAxCOffset;// radio delay for AID SI event start (+8 is a temporaly fix for dfe exception in LTE)
                if (ingrGroupInfo[i].g.isLte)
                {
                    if (ingrGroupInfo[i].g.cpType == IQN2_LTE_CPTYPE_NONE)
                    {
                        aid2Setup.uat_evt_clk_cnt_tc_cfg_val[8 + i] = ((frameLength+1) / FrameMsg1[i] / (ingrRadStdInfo[i].symbolTc+1) * 4) - 1;//4 sample event (LTE20MHz)
                    } else {
                        aid2Setup.uat_evt_clk_cnt_tc_cfg_val[8 + i] = ((frameLength+1) / FrameMsgTot[i] / 20 * 4) - 1;//4 sample event (LTE20MHz)
                    }
                } else {
                    aid2Setup.uat_evt_clk_cnt_tc_cfg_val[8 + i] = ((frameLength+1) / FrameMsgTot[i] / 15 * 4) - 1;//4 sample event (LTE20MHz)
                }
            }

        }
	} else {
		iqn2Setup.aid2Setup = NULL;
	}
	
	/************** AT Setup  **************************/
    //BCN timer
	if ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76)){
		at2Setup.at2_bcn_offset_cfg_val = IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER - 255;//BCN starts 255 clock before frame boundary
		at2Setup.at2_bcn_clkcnt_tc_cfg_val = IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER;	//IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER
	} else if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI) { 		//OBSAI
		at2Setup.at2_bcn_offset_cfg_val = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER - 255;//BCN starts 255 clock before frame boundary
		at2Setup.at2_bcn_clkcnt_tc_cfg_val = IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER;	//IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER
	} else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64) {
	    at2Setup.at2_bcn_offset_cfg_val = IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER - 255;//BCN starts 255 clock before frame boundary
        at2Setup.at2_bcn_clkcnt_tc_cfg_val = IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER;   //IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER
	}
	at2Setup.at2_bcn_frm_init_lsbs_cfg_wr_val = 0;                                  //BCN frame init value
	at2Setup.at2_bcn_frame_tc_lsb_cfg_val     = IQN2_FRAME_COUNT_TC_PHY_TIMER;      //BCN frame terminal count

	/**************** RADTs **************************/
	// Populate RADT structure based on num of egress radio standards used
	for (i=0; i<getMaxEgrGroupId(); i++)
	{
	    hIqn2->radTimerConfig[i].mode = IQN2_RADT_EGR_MODE;
	}

} //eof IQN2_initHw


void
IQN2_initRmAilSetupParams (
   Iqn2Fl_AilInstance    ailNum,
   int32_t               bEnableLcvControl,
   uint16_t              losDetThreshold,
   uint16_t              SyncThreshold,
   uint16_t              FrameSyncThreshold,
   uint16_t              UnsyncThreshold,
   uint16_t              FrameUnsyncThreshold
)
{
     ailPhySetup[ailNum].ail_phy_rm_regs.phy_rm_lcv_ctrl_cfg_en               = bEnableLcvControl;
     ailPhySetup[ailNum].ail_phy_rm_regs.phy_rm_lcv_ctrl_cfg_los_det_thold    = losDetThreshold;
     ailPhySetup[ailNum].ail_phy_rm_regs.phy_rm_fsm_sync_cfg_sync_t           = SyncThreshold;
     ailPhySetup[ailNum].ail_phy_rm_regs.phy_rm_fsm_sync_cfg_frame_sync_t     = FrameSyncThreshold;
     ailPhySetup[ailNum].ail_phy_rm_regs.phy_rm_fsm_unsync_cfg_unsync_t       = UnsyncThreshold;
     ailPhySetup[ailNum].ail_phy_rm_regs.phy_rm_fsm_unsync_cfg_frame_unsync_t = FrameUnsyncThreshold;
}

void
IQN2_initRadioTimer(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,k,z;
    uint32_t    lut_index;
    uint32_t    clockCountTc[7];
    baseGroupInfo_t *grpInfo;
    radioStandardInfo_t *radtInfo;
    uint32_t         grpId;

  lut_index = 0;
  for (i=0; i<IQN2_MAX_NUM_RADT; i++)
  {
      if ((hIqn2->radTimerConfig[i].mode != IQN2_RADT_UNUSED_MODE) &&
          (((hIqn2->radTimerConfig[i].mode == IQN2_RADT_EGR_MODE) && (egrGroupInfo[i].g.isPopulated)) ||
           ((hIqn2->radTimerConfig[i].mode == IQN2_RADT_ING_MODE) && (ingrGroupInfo[i-getMaxEgrGroupId()].g.isPopulated))
          )
         )
      {
          if (hIqn2->radTimerConfig[i].mode == IQN2_RADT_EGR_MODE) {
              grpId = i;
              grpInfo = &egrGroupInfo[grpId].g;
              radtInfo = &egrRadStdInfo[grpId];
          } else if (hIqn2->radTimerConfig[i].mode == IQN2_RADT_ING_MODE) {
              grpId = i-getMaxEgrGroupId();
              grpInfo = &ingrGroupInfo[grpId].g;
              radtInfo = &ingrRadStdInfo[grpId];
          }

          if (hIqn2->radTimerConfig[i].userSpecified == 0) {
              at2Setup.at2_radt_init_1_cfg_frm_init_lsb[i]  = 1;
              at2Setup.at2_radt_init_2_cfg_frm_init_msb[i]  = 0;
              at2Setup.at2_radt_1_cfg_frm_tc_lsb[i]         = 4096 - 1;
              at2Setup.at2_radt_4_cfg_bcn_sync_cmp[i]       = 0;

          } else {
              at2Setup.at2_radt_init_1_cfg_frm_init_lsb[i]  = hIqn2->radTimerConfig[i].initFrameLsbNum;
              at2Setup.at2_radt_init_2_cfg_frm_init_msb[i]  = hIqn2->radTimerConfig[i].initFrameMsbNum;
              at2Setup.at2_radt_1_cfg_frm_tc_lsb[i]         = hIqn2->radTimerConfig[i].frameTerminalCount - 1;
              at2Setup.at2_radt_4_cfg_bcn_sync_cmp[i]       = hIqn2->radTimerConfig[i].bcnCompValue;
          }
          if (grpInfo->isLte)
          {
              if ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76))
              {
                  if(grpInfo->cpType == IQN2_LTE_CPTYPE_NONE)
                      at2Setup.at2_radt_0_cfg_clkcnt_tc[i] = ((IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1) / FrameMsg1[grpId] / (radtInfo->symbolTc+1)) - 1;
                  else
                      at2Setup.at2_radt_0_cfg_clkcnt_tc[i] = ((IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1) / FrameMsgTot[grpId] / 20) - 1;// 8 clock counts in one sample for LTE20   CPRI
              } else if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI) {
                      at2Setup.at2_radt_0_cfg_clkcnt_tc[i] = ((IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1) / FrameMsgTot[grpId] / 20) - 1;//10 clock counts in one sample for LTE20   OBSAI
              } else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64){
                  if(grpInfo->cpType == IQN2_LTE_CPTYPE_NONE)
                      at2Setup.at2_radt_0_cfg_clkcnt_tc[i] = ((IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER+1) / FrameMsg1[grpId] / (radtInfo->symbolTc+1)) - 1;//12 clock counts in one sample for LTE20   DFE@368.64
                  else
                      at2Setup.at2_radt_0_cfg_clkcnt_tc[i] = ((IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER+1) / FrameMsgTot[grpId] / 20) - 1;//12 clock counts in one sample for LTE20   DFE@368.64
              }
          } else { // WCDMA
              if ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76))
                  at2Setup.at2_radt_0_cfg_clkcnt_tc[i] = ((IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1) / FrameMsgTot[grpId] / 15) - 1;//8 clock count in one sample for LTE20MHz
              else if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)
                  at2Setup.at2_radt_0_cfg_clkcnt_tc[i] = ((IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1) / FrameMsgTot[grpId] / 15) - 1;//8 clock count in one sample for LTE20MHz
              else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64)
                  at2Setup.at2_radt_0_cfg_clkcnt_tc[i] = ((IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER+1) / FrameMsgTot[grpId] / 15) - 1;//8 clock count in one sample for LTE20MHz
          }
          at2Setup.at2_radt_3_cfg_lut_index_strt[i]    = lut_index;
          if (grpInfo->isLte)
          {
              if (hIqn2->radTimerConfig[i].userSpecified == 0) {
                  if(grpInfo->cpType == IQN2_LTE_CPTYPE_NONE)
                  {
                      at2Setup.at2_radt_0_cfg_lutindex_tc[i]    = lut_index;
                      at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[lut_index] = FrameMsg1[grpId] - 1;
                      lut_index ++;
                      at2Setup.at2_radt_0_cfg_symb_tc[i] = (radtInfo->symbolTc);//140 symbols in frame
                  } else {
                      at2Setup.at2_radt_0_cfg_lutindex_tc[i]    = lut_index + IQN2_LTE_SYMBOL_NUM - 1;
                      at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[lut_index] = FrameMsg1[grpId] - 1;
                      for(j=(1+lut_index);j<((IQN2_LTE_SYMBOL_NUM+lut_index));j++){
                          at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[j] = FrameMsg[grpId] - 1;
                      }
                      lut_index += IQN2_LTE_SYMBOL_NUM;
                      at2Setup.at2_radt_0_cfg_symb_tc[i] = 139;//140 symbols in frame
                  }
              } else {
                  if (hIqn2->radTimerConfig[i].lte.cpType == IQN2_LTE_CPTYPE_NORMAL) {
                      at2Setup.at2_radt_0_cfg_lutindex_tc[i]    = lut_index + IQN2_LTE_SYMBOL_NUM - 1;
                  } else if (hIqn2->radTimerConfig[i].lte.cpType == IQN2_LTE_CPTYPE_EXTENDED){
                      at2Setup.at2_radt_0_cfg_lutindex_tc[i]    = lut_index + IQN2_LTE_SYMBOL_NUM_EXT_CP - 1;
                  } else {
                      at2Setup.at2_radt_0_cfg_lutindex_tc[i]    = lut_index;
                  }
                  at2Setup.at2_radt_0_cfg_symb_tc[i] = hIqn2->radTimerConfig[i].lte.numSymbolStrobesForFrameStrobe - 1;

                  if (hIqn2->radTimerConfig[0].lte.cpType == IQN2_LTE_CPTYPE_NORMAL) {
                      clockCountTc[0] = FrameMsg1[grpId];
                      for (z=1;z<IQN2_LTE_SYMBOL_NUM;z++) {
                          clockCountTc[z] = FrameMsg[grpId];
                      }

                      k = 0;
                      for (z=lut_index;z<(IQN2_LTE_SYMBOL_NUM + lut_index);z++) {
                          for (j=0;j<hIqn2->radTimerConfig[i].lte.numSymbolsForSymbolStrobe;j++) {
                              at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[z] += clockCountTc[k++];
                              if (k==IQN2_LTE_SYMBOL_NUM) k=0;
                          }
                          at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[z] -= 1;
                      }
                      lut_index += IQN2_LTE_SYMBOL_NUM;
                  } else if (hIqn2->radTimerConfig[i].lte.cpType == IQN2_LTE_CPTYPE_EXTENDED){ // Extended CP
                      for (z=lut_index;z<(IQN2_LTE_SYMBOL_NUM_EXT_CP + lut_index);z++) {
                        at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[z] = (hIqn2->radTimerConfig[i].lte.numSymbolsForSymbolStrobe*FrameMsg2[grpId])-1;
                      }
                      lut_index += IQN2_LTE_SYMBOL_NUM_EXT_CP;
                  } else {
                      at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[lut_index] = (hIqn2->radTimerConfig[i].lte.numSymbolsForSymbolStrobe*FrameMsg1[grpId])-1;
                      lut_index ++;
                  }

              }
          } else // WCDMA
          {
              at2Setup.at2_radt_0_cfg_lutindex_tc[i]                  = lut_index;
              at2Setup.at2_radt_sym_lut_ram_cfg_symbcnt_tc[lut_index] = FrameMsg[grpId] - 1;;//2560 chips in slot
              lut_index++;
              if (hIqn2->radTimerConfig[i].userSpecified == 0) {
                  at2Setup.at2_radt_0_cfg_symb_tc[i]                      = 14; //15 slots in frame
              } else {
                  at2Setup.at2_radt_0_cfg_symb_tc[i] = hIqn2->radTimerConfig[i].wcdma.numSlotStrobesForFrameStrobe - 1;
              }
          }
      }
  }
}

void
IQN2_initAt2Event(
        IQN2_ConfigHandle    hIqn2,
        IQN2_At2EventHandle  hAt2Evt
)
{
    at2Setup.at2_events_24array.at2_events_24array_offset_cfg_val[hAt2Evt->EventSelect]          = hAt2Evt->EventOffset;
    at2Setup.at2_events_24array.at2_events_24array_offset_cfg_evt_strb_sel[hAt2Evt->EventSelect] = hAt2Evt->EvtStrobeSel;
    if (hAt2Evt->EventModulo != -1) {
        at2Setup.at2_events_24array.at2_events_24array_mod_tc_cfg_val[hAt2Evt->EventSelect]      = hAt2Evt->EventModulo;
    } else {
        at2Setup.at2_events_24array.at2_events_24array_mod_tc_cfg_val[hAt2Evt->EventSelect]      = (uint32_t)CSL_IQN_AT2_AT2_EVENT_MOD_TC_CFG_VAL_MASK; //set max value which means no additional events between strobe
    }
    if (hAt2Evt->EventMaskLsb != 0) {
        at2Setup.at2_events_24array.at2_events_24array_mask_lsbs_cfg_val[hAt2Evt->EventSelect] = hAt2Evt->EventMaskLsb;
    } else {
        at2Setup.at2_events_24array.at2_events_24array_mask_lsbs_cfg_val[hAt2Evt->EventSelect] = CSL_IQN_AT2_AT2_EVENT_MASK_LSBS_CFG_VAL_MASK;
    }
    if (hAt2Evt->EventMaskMsb != 0) {
        at2Setup.at2_events_24array.at2_events_24array_mask_msbs_cfg_val[hAt2Evt->EventSelect] = hAt2Evt->EventMaskMsb;
    } else {
        at2Setup.at2_events_24array.at2_events_24array_mask_msbs_cfg_val[hAt2Evt->EventSelect] = CSL_IQN_AT2_AT2_EVENT_MASK_MSBS_CFG_VAL_MASK;
    }

    hIqn2->at2EventConfig[hAt2Evt->EventSelect].EventSelect  = hAt2Evt->EventSelect;
    hIqn2->at2EventConfig[hAt2Evt->EventSelect].EventOffset  = hAt2Evt->EventOffset;
    hIqn2->at2EventConfig[hAt2Evt->EventSelect].EvtStrobeSel = hAt2Evt->EvtStrobeSel;
    hIqn2->at2EventConfig[hAt2Evt->EventSelect].EventModulo =  hAt2Evt->EventModulo;
    hIqn2->at2EventConfig[hAt2Evt->EventSelect].EventMaskLsb  = hAt2Evt->EventMaskLsb;
    hIqn2->at2EventConfig[hAt2Evt->EventSelect].EventMaskMsb  = hAt2Evt->EventMaskMsb;
}

void
IQN2_initNanoSecsToByteClocks(
        IQN2_ConfigHandle    hIqn2,
        IQN2_At2EventHandle  hAt2Evt
)
{
    float ratio = 0;

    if((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76))
        ratio = (float)((float)(IQN2_CLOCK_COUNT_TC_CPRI + 1) / (float)1000000);
    else if(hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)
        ratio = (float)((float)(IQN2_CLOCK_COUNT_TC_OBSAI + 1) / (float)1000000);
    else if(hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64)
        ratio = (float)((float)(IQN2_CLOCK_COUNT_TC_368_64 + 1) / (float)1000000);

    hAt2Evt->EventOffset = (uint32_t)((float)hAt2Evt->EventOffset * ratio);
    hAt2Evt->EventModulo = ((uint32_t)((float)hAt2Evt->EventModulo * ratio)) - 1;
}

void
IQN2_initWcdmaChipsToByteClocks(
        IQN2_ConfigHandle    hIqn2,
        IQN2_At2EventHandle  hAt2Evt
)
{
    uint32_t clockCount = 0;

    if((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76))
        clockCount = 64;
    else if(hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)
        clockCount = 80;
    else if(hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64)
        clockCount = 96;

    hAt2Evt->EventOffset = (uint32_t)(hAt2Evt->EventOffset*clockCount);
    hAt2Evt->EventModulo = ((uint32_t)(hAt2Evt->EventModulo*clockCount) - 1);
}

void
IQN2_initEgrDioEvent(
		IQN2_ConfigHandle    hIqn2,
		IQN2_DioEventHandle  hEgrDioEvent,
		uint32_t             dioNum
)
{
    if (hIqn2)
    {
    	dio2Setup.uat_dio_egr_radt_tc_cfg_val[dioNum]      = hEgrDioEvent->FrameTerminalCount;
    	dio2Setup.uat_evt_clk_cnt_tc_cfg_val[16+dioNum]    = hEgrDioEvent->EventModulo;
    	dio2Setup.uat_evt_radt_cmp_cfg_val[16+dioNum]      = hEgrDioEvent->DioFrameEventOffset;
    }
}

void
IQN2_initIngrDioEvent(
		IQN2_ConfigHandle    hIqn2,
		IQN2_DioEventHandle  hIngrDioEvent,
		uint32_t             dioNum
)
{
    if (hIqn2)
    {
    	dio2Setup.uat_dio_ing_radt_tc_cfg_val[dioNum]      = hIngrDioEvent->FrameTerminalCount;
    	dio2Setup.uat_evt_clk_cnt_tc_cfg_val[19+dioNum]    = hIngrDioEvent->EventModulo;
    	dio2Setup.uat_evt_radt_cmp_cfg_val[19+dioNum]      = hIngrDioEvent->DioFrameEventOffset;
    }
}


void
IQN2_initIngrDioAxCOffset(
		IQN2_ConfigHandle    hIqn2,
		uint32_t             dioNum,
		uint32_t             axcNum,
		uint8_t              offset
)
{
    dio2Setup.dio2_i_axc_off_mmr_cfg_4samp_offset[axcNum + hIqn2->dioConfig[dioNum].offsetPdDBCH] = offset;//similar to aif2 dio offset
}

void
IQN2_initAilPiMax(
		IQN2_ConfigHandle    hIqn2,
		Iqn2Fl_AilInstance   ailNum,
		uint32_t             piMax
)
{
	if (hIqn2->ailConfig[ailNum].piMin < piMax)
	{
		ailUatSetup[ailNum].ail_uat_pimax_cfg_val = piMax;
	}
}

/* Start IQN2 given this user configuration*/
void IQN2_startHw(
		IQN2_ConfigHandle  hIqn2
)
{
	Iqn2Fl_Status iqn2Status;
	Iqn2Fl_UatCfg uat_cfg;
	uint32_t       ctrlArg, j;
	uint8_t 		wcdmaOn = 0;

	/****** Do IQN2 HW setup (set all MMRs above) **********************************************************************/
	iqn2Status = Iqn2Fl_hwSetup(hIqn2->hFl, hIqn2->hIqn2Setup);
	for (j=0; j<IQN2_MAX_NUM_AIL; j++)
	{
        if( hIqn2->ailConfig[j].numWcdmaPeAxC != 0)
            wcdmaOn = 1;

        if (iqn2Status != IQN2FL_SOK)Iqn2_osalLog ("\nIQN2 HW setup error 0x%x",iqn2Status);

        ctrlArg = 0xAAAAAAAA;

        hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance) j;  //IQN2FL_AIL_0; // Using AIL_0 instance

        //global enable of AIL0 (Egress, Ingress)
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
        // }

        if( hIqn2->ailConfig[j].numWcdmaPeAxC != 0)
        {
            //global enable of DIO SI (Egress, Ingress)
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
            //global enable of DIO engine Core (Egress, Ingress)
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
        }

        if(wcdmaOn)
        {
            //Enable DIO uAT with DIAG_SYNC
            if (hIqn2->timerSyncSource == IQN2_DIAG_SW_SYNC)
            {
                //Enable DIO uAT with DIAG_SYNC
                uat_cfg.diag_sync = 1;
            } else {
                //Enable DIO uAT with uAT free run. make sync with AT in run time
                uat_cfg.diag_sync = 0;
                // DIO module Re-synchronization restriction - enable only after SW sync process is complete
                IQN2_dio2DisableIngressChannels(hIqn2);
                IQN2_dio2DisableEngines(hIqn2);
            }
            uat_cfg.uat_run = 1;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_UAT_GEN_CTL_UAT_CFG, (void *)&uat_cfg);
        }

        if(hIqn2->ailConfig[j].numCtlChannel != 0) {
            if (ailIgrSetup[j].rate_ictl == 0) {
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_ICTL_RATE_CTL_CFG_REG, (void *)&ailIgrSetup[j].rate_ictl);
            }
        }


        //Enable AIL0 uAT with DIAG_SYNC
        if (hIqn2->timerSyncSource == IQN2_DIAG_SW_SYNC)
        {
            //Enable DIO uAT with DIAG_SYNC
            uat_cfg.diag_sync = 1;
        } else {
            //Enable DIO uAT with uAT free run. make sync with AT in run time
            uat_cfg.diag_sync = 0;
        }
        uat_cfg.uat_run = 1;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_UAT_GEN_CTL_UAT_CFG_REG, (void *)&uat_cfg);
	}
	
	if (hIqn2->aidConfig.aidEnable == 1)
	{
		ctrlArg = 0x55555555;
		//global enable of AID (Egress, Ingress)
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
	
		if (hIqn2->aidConfig.numCtlChannel != 0)
		{
			//global enable of AID CTL (Egress, Ingress)
			Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
			Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_SET, (void *)&ctrlArg);

			//apply ictl rate control setting if explicitely set to 0
			if (aid2Setup.ictl_rate_ctl_cfg_rate == 0) {
			    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ICTL_RATE_CTL_CFG_REG, (void *)&aid2Setup.ictl_rate_ctl_cfg_rate);
			}
		}

		if ((hIqn2->aidConfig.numWcdmaEgressAxC != 0) || ((hIqn2->aidConfig.numLteEgressAxC != 0) && (hIqn2->aidConfig.lteDio)))
		{
		    //global enable of DIO SI (Egress, Ingress)
		    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
		    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
		    //global enable of DIO engine Core (Egress, Ingress)
		    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
		    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_SET, (void *)&ctrlArg);

		    //Enable DIO uAT with DIAG_SYNC
            if (hIqn2->timerSyncSource == IQN2_DIAG_SW_SYNC)
            {
                //Enable DIO uAT with DIAG_SYNC
                uat_cfg.diag_sync = 1;
            } else {
                //Enable DIO uAT with uAT free run. make sync with AT in run time
                uat_cfg.diag_sync = 0;
            }
		    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_UAT_GEN_CTL_UAT_CFG, (void *)&uat_cfg);
		}

        if (hIqn2->timerSyncSource == IQN2_DIAG_SW_SYNC)
        {
            //Enable AID uAT with DIAG_SYNC
            uat_cfg.diag_sync = 1;
        } else {
            //Enable AID uAT with uAT free run. make sync with AT in run time
            uat_cfg.diag_sync = 0;
        }
		uat_cfg.uat_run = 1;
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_UAT_GEN_CTL_UAT_CFG, (void *)&uat_cfg);
	}

	ctrlArg = 0x00000000;
	for (j=0; j<IQN2_MAX_NUM_RADT; j++)
	{
	      if (hIqn2->radTimerConfig[j].mode != IQN2_RADT_UNUSED_MODE) {
	          ctrlArg |= (1<<j);
	      }
	}
	Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN, (void *)&ctrlArg);
 
} //eof IQN2_startHw

// SW control which starts the BCN Timer
// For RADTs, once enabled in IQN2_startHw(), they will start running once the compare value equals the BCN timer value
void IQN2_runBcnTimer(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t       ctrlArg;

    ctrlArg = 0x1;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RUN_BCN, (void *)&ctrlArg);

}

////////////////////

