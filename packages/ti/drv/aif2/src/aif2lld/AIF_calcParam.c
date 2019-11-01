/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/

#include <ti/csl/csl.h>
#include <ti/drv/aif2/aif2fl.h>
#include <ti/csl/soc.h>
#include <ti/drv/aif2/AIF_defs.h>

#define __AIF_CALCPARAM_C
// Bit count over 32 bits
#ifdef _TMS320C6X
#define _bitc32(a) _dotpu4(_bitc4(a),0x01010101)
#else
static uint32_t _bitc32(uint32_t i)
{
    i = i - ((i >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}
#endif

#include <ti/drv/aif2/AIF_calcParam.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(AIF_calcParameters, ".text:aifDriver");
#pragma CODE_SECTION(AIF_calcAifTimingForTxNode, ".text:aifDriver");

#pragma CODE_SECTION(calcAifLinkCommonParameters, ".text:aifDriver");
#pragma CODE_SECTION(calcAifPackMode, ".text:aifDriver");
#pragma CODE_SECTION(calcAifPeOffsetParam, ".text:aifDriver");
#pragma CODE_SECTION(setBufDepth, ".text:aifDriver");
#pragma CODE_SECTION(getNextEnabledLinkCfg, ".text:aifDriver");
#pragma CODE_SECTION(calcAifAxCParam, ".text:aifDriver");
#pragma CODE_SECTION(calcAifCpri8WordOffsetParam, ".text:aifDriver");
#pragma CODE_SECTION(calcAifTimingForRxNode, ".text:aifDriver");
#pragma CODE_SECTION(calcAifLinkParameters, ".text:aifDriver");
#endif

//Calculate some common AIF Parameters according above pre-set paremeters
static void
calcAifLinkCommonParameters(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex    linkIndex
    )
{
AIF_LinkConfigHandle aifConfig = &(hAif->linkConfig[linkIndex]);

	//calculate stream number and byte clock speed according protocol and link rate
	if ((aifConfig->numPeAxC == 0) && (aifConfig->RtEnabled == 0))
	{
		if (hAif->mode == AIF_WCDMA_MODE)
		{
            aifConfig->numPeAxC= aifConfig->linkRate*4;
		}
		if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE)){
			if (aifConfig->sampleRate == AIF_SRATE_1P92MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/1920;
			if (aifConfig->sampleRate == AIF_SRATE_7P68MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/7680;
			if (aifConfig->sampleRate  == AIF_SRATE_15P36MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/15360;
			if (aifConfig->sampleRate  == AIF_SRATE_23P04MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/30720; // same as LTE20
			if (aifConfig->sampleRate  == AIF_SRATE_30P72MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/30720;
			if (aifConfig->sampleRate  == AIF_SRATE_61P44MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/61440;
		}
		if (hAif->mode == AIF_LTE_WCDMA_MODE)
		{
			if (aifConfig->mode == WCDMA)
			{
				aifConfig->numPeAxC= aifConfig->linkRate*4;
			} else {
				if (aifConfig->sampleRate == AIF_SRATE_1P92MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/1920;
				if (aifConfig->sampleRate == AIF_SRATE_7P68MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/7680;
				if (aifConfig->sampleRate  == AIF_SRATE_15P36MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/15360;
				if (aifConfig->sampleRate  == AIF_SRATE_23P04MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/30720;  // same as LTE20
				if (aifConfig->sampleRate  == AIF_SRATE_30P72MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/30720;
				if (aifConfig->sampleRate  == AIF_SRATE_61P44MHZ) aifConfig->numPeAxC= aifConfig->linkRate*4*256*15/61440;
			}
		}

		if(AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol)
		{
			/*CPRI byte clock speed is 61.44MHz x link rate*/
			aifConfig->byteClockInKHz= 61440* aifConfig->linkRate;

			//for CPRI, if the data with is 8 or 16 bits, the support stream number should minus 1
			if((AIF2FL_DATA_WIDTH_16_BIT==aifConfig->inboundDataWidth)||
				(AIF2FL_DATA_WIDTH_8_BIT==aifConfig->inboundDataWidth))
			{
				if (aifConfig->linkRate==8) aifConfig->numPeAxC-= 2;
				else aifConfig->numPeAxC-= 1;
			}
		}
		else
		{
			/*OBSAI byte clock speed is 76.8MHz x link rate*/
			aifConfig->byteClockInKHz= 76800* aifConfig->linkRate;
		}
	} else {
		if(AIF2FL_LINK_PROTOCOL_CPRI==hAif->protocol)
			aifConfig->byteClockInKHz= 61440* aifConfig->linkRate;
		else
			/*OBSAI byte clock speed is 76.8MHz x link rate*/
			aifConfig->byteClockInKHz= 76800* aifConfig->linkRate;
	}

	if (hAif->mode == AIF_GENERICPACKET_MODE) aifConfig->numPeAxC= 1;
	if (aifConfig->numPdAxC == 0) aifConfig->numPdAxC = aifConfig->numPeAxC;

}

static void
calcAifPackMode(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex   linkIndex
	)
{
	uint32_t i;
	AIF_LinkConfigHandle aifConfig = &(hAif->linkConfig[linkIndex]);
	if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)){
		if (aifConfig->cpriPackMode == AIF2_LTE_CPRI_1b1)
			aifConfig->maskPattern[0]= 1;
		else if (aifConfig->cpriPackMode == AIF2_LTE_CPRI_2b2)
			aifConfig->maskPattern[0]= 3;
		else if (aifConfig->cpriPackMode == AIF2_LTE_CPRI_4b4)
			aifConfig->maskPattern[0]= 15;
		else if (aifConfig->cpriPackMode == AIF2_LTE_CPRI_8b8)
			aifConfig->maskPattern[0]= 255;
		for (i=1;i<aifConfig->numPeAxC;i++)
		{
			aifConfig->maskPattern[i]=aifConfig->maskPattern[0]<< (_bitc32(aifConfig->maskPattern[0]) * i);
		}
	}
}

static uint32_t
calcAifPackModeAxC(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex   linkIndex,
        uint32_t 				offsetAxC
	)
{
	uint32_t i;
	int32_t offsetBitC;
	AIF_LinkConfigHandle aifConfig = &(hAif->linkConfig[linkIndex]);
	offsetBitC = 0;
	for (i=0;i<aifConfig->numPeAxC;i++)
	{
		if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE) || (hAif->mode == AIF_LTE_WCDMA_MODE)){
			if (hAif->AxCconfig[i+offsetAxC].cpriPackMode == AIF2_LTE_CPRI_1b1)
			{
				hAif->AxCconfig[i+offsetAxC].maskPattern= 1 << offsetBitC;
				offsetBitC += (_bitc32(1));
			} else if (hAif->AxCconfig[i+offsetAxC].cpriPackMode == AIF2_LTE_CPRI_2b2) {
				hAif->AxCconfig[i+offsetAxC].maskPattern= 3 << offsetBitC;
				offsetBitC += (_bitc32(3));
			} else if (hAif->AxCconfig[i+offsetAxC].cpriPackMode == AIF2_LTE_CPRI_4b4) {
				hAif->AxCconfig[i+offsetAxC].maskPattern= 15 << offsetBitC;
				offsetBitC += (_bitc32(15));
			} else if (hAif->AxCconfig[i+offsetAxC].cpriPackMode == AIF2_LTE_CPRI_8b8) {
				hAif->AxCconfig[i+offsetAxC].maskPattern= 255 << offsetBitC;
				offsetBitC += (_bitc32(255));
			}
		}
	}
	return (i+offsetAxC);
}

//Calculate timing for the first Tx Node. For this example, it is F1 TX
static void 
calcAifPeOffsetParam(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex   linkIndex
)
{
	AIF_LinkConfigHandle aifConfig = &(hAif->linkConfig[linkIndex]);
	//AT link setup to be adjusted
	if ((aifConfig->pe2Offset == 0) && (aifConfig->txWait == 0))
	{
		if (aifConfig->comType == AIF2_2_AIF2 || aifConfig->comType == AIF2_LOOPBACK) {
			if ((hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE))
			{
				if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
				{
					if (aifConfig->outboundDataType == AIF2FL_LINK_DATA_TYPE_RSA) // Uplink
					{
						aifConfig->pe2Offset = AIF2_LTE_OBSAI_UL_Pe2OffSet;
					} else { // Downlink
						aifConfig->pe2Offset = AIF2_LTE_OBSAI_DL_Pe2OffSet;
					}
				} else {
					if (aifConfig->outboundDataType == AIF2FL_LINK_DATA_TYPE_RSA) // Uplink
					{
						aifConfig->pe2Offset = AIF2_LTE_CPRI_UL_Pe2OffSet;
					} else { // Downlink
						aifConfig->pe2Offset = AIF2_LTE_CPRI_DL_Pe2OffSet;
					}
				}
			} else if (hAif->mode == AIF_LTE_WCDMA_MODE) {
					aifConfig->pe2Offset = AIF2_LTE_WCDMA_OBSAI_DL_Pe2OffSet;

			} else {
				if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
				{
					if (aifConfig->outboundDataType == AIF2FL_LINK_DATA_TYPE_RSA) // Uplink
					{
						aifConfig->pe2Offset = AIF2_WCDMA_OBSAI_UL_Pe2OffSet;
					} else { // Downlink
						aifConfig->pe2Offset = AIF2_WCDMA_OBSAI_DL_Pe2OffSet;
					}
				} else {
					if (hAif->mode == AIF_GENERICPACKET_MODE){
						aifConfig->pe2Offset = 310;
					} else {
						if (aifConfig->outboundDataType == AIF2FL_LINK_DATA_TYPE_RSA) // Uplink
						{
							aifConfig->pe2Offset = AIF2_WCDMA_CPRI_UL_Pe2OffSet;
						} else { // Downlink
							aifConfig->pe2Offset = AIF2_WCDMA_CPRI_DL_Pe2OffSet;
						}
					}
				}
			}
		} else { // AIF1 to AIF2 com TBD
			if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
			{
				aifConfig->pe2Offset = 610; // TO BE FIXED
			} else {
				aifConfig->pe2Offset = 490; // TO BE FIXED
			}
		}
	}
}

static void setBufDepth(Aif2Fl_DbFifoDepth qwSize, Aif2Fl_DbFifoDepth *bufPtr)
{
	if (*bufPtr == 0) {
		*bufPtr = qwSize;
	}
}

static AIF_LinkConfigHandle	getNextEnabledLinkCfg(AIF_ConfigHandle  hAif, AIF_LinkConfigHandle linkCfgPtr)
{
	while (linkCfgPtr <= &hAif->linkConfig[AIF_MAX_NUM_LINKS]) {
		if (linkCfgPtr->linkEnable == 1) {
			return(linkCfgPtr);
		}
		linkCfgPtr++;
	}
	return(NULL);
}

//Calculate the BufDepth for default value.
//Note: The AIF LLD associates the DB and pktDMA channels to the AxCs in the natural order 
//      of the DB channels, the AxCs in the natural order of the natural
//      order of the links that are enabled. Natural order means starting from
//      0 towards the maximum. E.g if link 2 and 5 are enabled each with number of AxCs
//      of 1 and 2 respectively, then the mapping of DB channel to AxCs will be as follows:
//      link 2, AxC0 will be DB chan 0
//      link 5, AxC0 will be DB chan 1
//      link 5, AxC1 will be DB chan 2
//      This allocation order is reflected in the code below to set the defaults
//      and where user is providing his own numbers, it is with the assumption that 
//      the user knows the mapping order and will populated the entries accordingly
//      in the 128 entry arrays of the egress and ingress depths.
//      This routine will need to be changed if in future LLD provides ability
//      to do arbitrary mappings.
static void
calcAifAxCParam(
        AIF_ConfigHandle    hAif
)
{
	AIF_LinkConfigHandle linkCfgPtr = &hAif->linkConfig[0];
	uint32_t eDbIndx = 0;
	uint32_t iDbIndx = 0;
	uint32_t iEgr, iIngr;
	Aif2Fl_DbFifoDepth defaultQwSize = AIF2FL_DB_FIFO_DEPTH_QW8;
	uint32_t isAifModeFDDorTDD;
	
	isAifModeFDDorTDD = (hAif->mode == AIF_LTE_FDD_MODE) || (hAif->mode == AIF_LTE_TDD_MODE);
	
	//if ((hAif->mode == AIF_GENERICPACKET_MODE) || isAifModeFDDorTDD)
	{
		while ((linkCfgPtr = getNextEnabledLinkCfg(hAif, linkCfgPtr)) != NULL) {
			if ((isAifModeFDDorTDD) || (hAif->mode == AIF_GENERICPACKET_MODE)) {
				if (linkCfgPtr->sampleRate == AIF_SRATE_1P92MHZ) {
					defaultQwSize = AIF2FL_DB_FIFO_DEPTH_QW8;
				}
				else {
					defaultQwSize = AIF2FL_DB_FIFO_DEPTH_QW64;
				}
			}
			for (iEgr = 0; iEgr < linkCfgPtr->numPeAxC; iEgr++) {
				setBufDepth(defaultQwSize, &hAif->AxCconfig[eDbIndx].egressBufDepth);
				//Calculate default value for AxCOffset.
				if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
				{
					if (hAif->AxCconfig[eDbIndx].peAxCOffset == 0)
					{
						hAif->AxCconfig[eDbIndx].peAxCOffset = linkCfgPtr->pe2Offset + 1;
					}
				} else { // CPRI
					if (hAif->AxCconfig[eDbIndx].peAxCOffset == 0)
					{
						hAif->AxCconfig[eDbIndx].peAxCOffset = linkCfgPtr->pe2Offset;
					}
				}
				eDbIndx++;
			}
			for (iIngr = 0; iIngr < linkCfgPtr->numPdAxC; iIngr++) {
				setBufDepth(defaultQwSize, &hAif->AxCconfig[iDbIndx].ingressBufDepth);
				//Calculate default value for AxCOffset.
				if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
				{
					if (hAif->AxCconfig[iDbIndx].pdAxCOffset == 0)
					{
						hAif->AxCconfig[iDbIndx].pdAxCOffset = linkCfgPtr->pe2Offset + 1;
					}
				}
				iDbIndx++;
			}
			linkCfgPtr++;
		}
	}
}

//Calculate the BufDepth for default value.
static void
calcAifCpri8WordOffsetParam(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex   linkIndex
)
{
	uint32_t i;

	AIF_LinkConfigHandle aifConfig = &(hAif->linkConfig[linkIndex]);
	if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
	{
		for(i=0;i<128;i++)
		{
			if (aifConfig->cpri8WordOffset[i] == 0)
				aifConfig->cpri8WordOffset[i] = 0;
		}
	}

}

//Calculate timing for the first Tx Node. For this example, it is F1 TX
uint32_t
AIF_calcAifTimingForTxNode(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex   linkIndex
)
{
AIF_LinkConfigHandle aifConfig = &(hAif->linkConfig[linkIndex]);

	//AT link setup to be adjusted
	if (aifConfig->comType == AIF2_2_AIF2 || aifConfig->comType == AIF2_LOOPBACK) {
		if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
		{
			if (aifConfig->txWait != 0) {
				aifConfig->pe2Offset = (aifConfig->txWait * (AIF2_CLOCK_COUNT_TC_OBSAI + 1) / 1000000) + 10; // converting ns to byte clocks
			}
			aifConfig->deltaOffset = aifConfig->pe2Offset + 60 + 80 * (aifConfig->nodeTx);
		} else {
			if (aifConfig->txWait != 0) {
				aifConfig->pe2Offset = (aifConfig->txWait * (AIF2_CLOCK_COUNT_TC_CPRI + 1) / 1000000) + 10; // converting ns to byte clocks
			}
			aifConfig->deltaOffset = aifConfig->pe2Offset + 60 + 64 * (aifConfig->nodeTx);
		}
	}
	return(aifConfig->deltaOffset);
}

//Calculate timing for the first Rx Node on daisy Chain. For this example, it is F2 RX
static void 
calcAifTimingForRxNode(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex   linkIndex
    )
{
AIF_LinkConfigHandle aifConfig = &(hAif->linkConfig[linkIndex]);

	//AT link setup to be adjusted
	if (aifConfig->comType == AIF2_2_AIF2 || aifConfig->comType == AIF2_LOOPBACK) {
		if(AIF2FL_LINK_PROTOCOL_OBSAI==hAif->protocol)
		{
			if (aifConfig->piMin == 0) aifConfig->piMin = aifConfig->deltaOffset + 80 * (aifConfig->nodeRx);
		} else {
			if (aifConfig->piMin == 0) aifConfig->piMin = aifConfig->deltaOffset + 64 * (aifConfig->nodeRx);
		}
	}
}


static void 
calcAifLinkParameters(
        AIF_ConfigHandle    hAif,
        Aif2Fl_LinkIndex    linkIndex
    )
{
	calcAifPeOffsetParam(hAif,linkIndex);
	calcAifCpri8WordOffsetParam(hAif,linkIndex);
    // Compute timing parameters for TI EVMs
	AIF_calcAifTimingForTxNode(hAif,linkIndex);
	calcAifTimingForRxNode(hAif,linkIndex);
}


//Calculate some other AIF Parameters according to the pre-set paremeters in AIF2_ConfigObj
void
AIF_calcParameters(
    AIF_ConfigHandle    hAif
)
{
	int32_t i;
	uint32_t j=0;
	for(i=0; i<AIF_MAX_NUM_LINKS; i++)
	{
		if(0==hAif->linkConfig[i].linkEnable)
			continue;

		calcAifLinkCommonParameters(hAif, (Aif2Fl_LinkIndex)i);
		calcAifPackMode(hAif, (Aif2Fl_LinkIndex)i);
		if (hAif->linkConfig[i].multiRate == 1)
			j = calcAifPackModeAxC(hAif, (Aif2Fl_LinkIndex)i, j);
		calcAifLinkParameters(hAif, (Aif2Fl_LinkIndex)i);
	}
	calcAifAxCParam(hAif);
}


////////////////////
