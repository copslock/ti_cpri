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

#include <ti/csl/csl.h>
#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/csl/soc.h>
#include <ti/drv/iqn2/include/IQN2_defs.h>

#define __IQN2_CALCPARAM_C
// Bit count over 32 bits
#ifdef _TMS320C6X
//#define _bitc32(a) _dotpu4(_bitc4(a),0x01010101)
#else
/*static uint32_t _bitc32(uint32_t i)
{
    i = i - ((i >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}*/
#endif

#include <ti/drv/iqn2/include/IQN2_calcParam.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(calcIqn2AilCommonParameters, ".text:iqn2CalcParam"); 
#pragma CODE_SECTION(calcIqn2PeOffsetParam, ".text:iqn2CalcParam");
#pragma CODE_SECTION(IQN2_calcAilTimingForTxNode, ".text:iqn2CalcParam");
#pragma CODE_SECTION(calcIqn2TimingForRxNode, ".text:iqn2CalcParam");
#pragma CODE_SECTION(calcIqn2AilParameters, ".text:iqn2CalcParam");
#pragma CODE_SECTION(IQN2_calcParameters, ".text:iqn2CalcParam");
#endif

uint32_t delayRx = 0;


//Calculate some common IQN2 Parameters according above pre-set paremeters
static void
calcIqn2AilCommonParameters(
        IQN2_ConfigHandle    hIqn2,
        Iqn2Fl_AilInstance   ailIndex
    )
{
IQN2_AilConfigHandle iqn2Config = &(hIqn2->ailConfig[ailIndex]);

	if(IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
		iqn2Config->byteClockInKHz= 61440* iqn2Config->linkRate;
	} else {
		/*OBSAI byte clock speed is 76.8MHz x link rate*/
		iqn2Config->byteClockInKHz= 76800* iqn2Config->linkRate;
	}
}

//Calculate timing for the first Tx Node. For this example, it is F1 TX
static void 
calcIqn2PeOffsetParam(
        IQN2_ConfigHandle    hIqn2,
        Iqn2Fl_AilInstance   ailIndex
)
{
	IQN2_AilConfigHandle iqn2Config = &(hIqn2->ailConfig[ailIndex]);
	//AT link setup to be adjusted
	if ((iqn2Config->pe2Offset == 0) && (iqn2Config->txWait == 0))
	{
		if (iqn2Config->comType == IQN2_COM_NO_LOOPBACK || iqn2Config->comType == IQN2_COM_SD_LOOPBACK) {
				if(IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol)
				{
						iqn2Config->pe2Offset = IQN2_LTE_OBSAI_DL_Pe2OffSet;
				} else {
						iqn2Config->pe2Offset = IQN2_LTE_CPRI_DL_Pe2OffSet;
				}
		} else { // IQN21 to IQN2 com TBD
			if(IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol)
			{
				iqn2Config->pe2Offset = 610; // TO BE FIXED
			} else {
				iqn2Config->pe2Offset = 490; // TO BE FIXED
			}
		}
	}

}

//Calculate timing for the first Tx Node. For this example, it is F1 TX
uint32_t
IQN2_calcAilTimingForTxNode(
        IQN2_ConfigHandle    hIqn2,
        Iqn2Fl_AilInstance   ailIndex
)
{
IQN2_AilConfigHandle iqn2Config = &(hIqn2->ailConfig[ailIndex]);

	//AT link setup to be adjusted
	if (iqn2Config->comType == IQN2_COM_NO_LOOPBACK || iqn2Config->comType == IQN2_COM_SD_LOOPBACK) {
		if(IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol)
		{
			if (iqn2Config->txWait != 0) {
			    iqn2Config->pe2Offset = (iqn2Config->txWait * (IQN2_CLOCK_COUNT_TC_OBSAI + 1) / 1000000) + 10; // converting ns to byte clocks
			}
			iqn2Config->deltaOffset = iqn2Config->pe2Offset + 60 + 80 * (iqn2Config->nodeTx);
			iqn2Config->piMin = iqn2Config->deltaOffset + 80 * (iqn2Config->nodeRx);
		} else {
			if (iqn2Config->txWait != 0) {
			    iqn2Config->pe2Offset = (iqn2Config->txWait * (IQN2_CLOCK_COUNT_TC_CPRI + 1) / 1000000) + 10; // converting ns to byte clocks
			}
			iqn2Config->deltaOffset = iqn2Config->pe2Offset + 60 + 64 * (iqn2Config->nodeTx);
			iqn2Config->piMin = iqn2Config->deltaOffset + 64 * (iqn2Config->nodeRx);
		}
	}
	return(iqn2Config->deltaOffset);
}

//Calculate timing for the first Rx Node on daisy Chain. For this example, it is F2 RX
static void 
calcIqn2TimingForRxNode(
        IQN2_ConfigHandle    hIqn2,
        Iqn2Fl_AilInstance   ailIndex
    )
{
IQN2_AilConfigHandle iqn2Config = &(hIqn2->ailConfig[ailIndex]);

	//AT link setup to be adjusted
	if(IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol)
	{
		if (iqn2Config->piMin == 0) iqn2Config->piMin = iqn2Config->deltaOffset + 80 * (iqn2Config->nodeRx);
	} else {
		if (iqn2Config->piMin == 0) iqn2Config->piMin = iqn2Config->deltaOffset + 64 * (iqn2Config->nodeRx);
	}
}


static void 
calcIqn2AilParameters(
        IQN2_ConfigHandle    hIqn2,
        Iqn2Fl_AilInstance   ailIndex
    )
{
	calcIqn2PeOffsetParam(hIqn2,ailIndex);
    // Compute timing parameters for TI EVMs
	IQN2_calcAilTimingForTxNode(hIqn2,ailIndex);
	calcIqn2TimingForRxNode(hIqn2,ailIndex);
}

static void
calcIqn2AidParameters(
        IQN2_ConfigHandle    hIqn2
    )
{
    if (hIqn2->aidConfig.siDelay == 0)
        hIqn2->aidConfig.siDelay = 200; //default value for AID2 SI delay (4 samples)
}

//Calculate IQN2 timing and delay parameters according to the pre-set paremeters in IQN2_ConfigObj
void
IQN2_calcParameters(
    IQN2_ConfigHandle    hIqn2
)
{
	int32_t i;
	for(i=0; i<IQN2FL_AIL_MAX; i++)
	{
		if(0==hIqn2->ailConfig[i].ailEnable)
			continue;

		calcIqn2AilCommonParameters(hIqn2, (Iqn2Fl_AilInstance)i);
		calcIqn2AilParameters(hIqn2, (Iqn2Fl_AilInstance)i);
	}
	if(hIqn2->aidConfig.aidEnable)
	    calcIqn2AidParameters(hIqn2);
}




////////////////////
