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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <math.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include "cslUtils.h"
#include "mnavUtils.h"

#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif

#include "checkRfUtils.h"
#include "lteUtilsMulti.h"

volatile uint32_t packets_already_pushed = 0;

volatile uint32_t iqn2SymbolEgressCount[MAX_SLOT];
volatile uint32_t iqn2SymbolIngressCount[MAX_SLOT];
volatile uint32_t iqn2CtlIngressCount[MAX_SLOT];
volatile uint32_t iqn2CtlIngressPktStatus[MAX_SLOT];
volatile uint32_t iqn2RxPktStatus0[MAX_SLOT];
volatile uint32_t iqn2RxPktStatus1[MAX_SLOT];
volatile uint32_t getCpuTimestamp[MAX_SLOT];
volatile uint32_t getRadt0Frames[MAX_SLOT];
volatile uint32_t getRadt0Symbols[MAX_SLOT];
volatile uint32_t getRadt1Frames[MAX_SLOT];
volatile uint32_t getRadt1Symbols[MAX_SLOT];
volatile uint32_t getBCNFrame[MAX_SLOT];
volatile uint32_t getRxPktCnt[MAX_SLOT];
volatile uint32_t getAil0TmStatus[MAX_SLOT];
volatile uint32_t symbolNum[MAX_SLOT][7];
volatile uint32_t rxPktCnt[8 * 2];
volatile uint32_t symbolcount = 0;
volatile uint32_t symbolNb = 0;
volatile uint32_t slotcount = 0;
Cppi_MonolithicDesc* symbolPkt[8 * 2][NBSYMBOL*2];

extern volatile unsigned int         testcheck;

extern Uint32 *p0_2_tone_axc1_in;
extern Uint32 *p0_2_tone_axc2_in;
extern Uint32 *p0_2_tone_axc3_in;
extern Uint32 *p0_2_tone_axc4_in;

//////////////////////////////////////////////////////////////////////////////////////////////////
//							Swi dedicated for packet pushing									//
//////////////////////////////////////////////////////////////////////////////////////////////////
void slotRecycling(PktDmaConfigHandle hPktDma)
{
	int32_t                i,chan;
	uint32_t               rxSymCnt, payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;
	uint32_t 				nbchanneltotal, firstChan;

	nbchanneltotal = hPktDma->numAxC;
	firstChan = hPktDma->firstAxC;


#if NEVER_END_TEST == 0
	/*
	 * Rx packet recycling until final check, we need available descriptors in Rx FDQs when actual data is pushed
	 */
	if (slotcount < SLOT_NUM_DATA_CHECK){
#endif
	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
	{
		rxSymCnt = Qmss_getQueueEntryCount(hPktDma->rxQAxC[chan]);

		for(i=0;i<rxSymCnt;i++)
		{
			ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQAxC[chan]));
			if (ptrMonoDesc == NULL)
				printf("Rx descriptor NULL: descriptor %d, chan %d, slotcount %d \n", i, chan, slotcount);
			// Invalidate symbol packet header and get the updated payload address + length
			UTILS_cacheInvalidate((void *)ptrMonoDesc, 16);
			Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			//check payload length for first symbol
			if (payloadLen != (hPktDma->txDescSizeAxC[chan]-16) && payloadLen != (hPktDma->txDesc2SizeAxC[chan]-16))
					printf("unexpected payload length:%d, %d\n",(hPktDma->txDescSizeAxC[chan]-16),payloadLen);
			Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			symbolNum [slotcount][i] = ((payloadPtr[0] >> 7) & 0xFF);//check symbol number into PS field
			// recycle symbol packet on rx free queue
			Qmss_queuePushDesc(hPktDma->rxFqAxC[chan], (uint32_t*)ptrMonoDesc);
			rxPktCnt[chan]++;
		}
	}
#if NEVER_END_TEST == 0
	}
#endif

}

void symbolRecycling(PktDmaConfigHandle hPktDma)
{
	int32_t                i,chan;
	uint32_t               rxSymCnt, payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;
	uint32_t 				nbchanneltotal, firstChan;

	nbchanneltotal = hPktDma->numAxC;
	firstChan = hPktDma->firstAxC;


#if NEVER_END_TEST == 0
	/*
	 * Rx packet recycling until final check, we need available descriptors in Rx FDQs when actual data is pushed
	 */
	if (symbolcount < SYMBOL_NUM_DATA_CHECK){
#endif
	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
	{
		rxSymCnt = Qmss_getQueueEntryCount(hPktDma->rxQAxC[chan]);

		for(i=0;i<rxSymCnt;i++)
		{
			ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQAxC[chan]));
			if (ptrMonoDesc == NULL)
				printf("Rx descriptor NULL: descriptor %d, chan %d, symbolcount %d \n", i, chan, symbolcount);
			// Invalidate symbol packet header and get the updated payload address + length
			UTILS_cacheInvalidate((void *)ptrMonoDesc, 16);
			Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
		    //check payload length for first symbol
			if (payloadLen != (hPktDma->txDescSizeAxC[chan]-16) && payloadLen != (hPktDma->txDesc2SizeAxC[chan]-16))
                printf("unexpected payload length:%d, %d\n",(hPktDma->txDescSizeAxC[chan]-16),payloadLen);
			Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			symbolNum [slotcount][i] = ((payloadPtr[0] >> 7) & 0xFF);//check symbol number into PS field
			// recycle symbol packet on rx free queue
			Qmss_queuePushDesc(hPktDma->rxFqAxC[chan], (uint32_t*)ptrMonoDesc);
			rxPktCnt[chan]++;
		}
	}
#if NEVER_END_TEST == 0
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////							Swi dedicated for getting timings (core 0)							//
////////////////////////////////////////////////////////////////////////////////////////////////////
void slotCount(Iqn2Fl_Handle   hIqn2Fl)
{
	slotcount++;
	getCpuTimestamp[slotcount] 	= TSCL;
	getRadt1Frames[slotcount]   = hIqn2Fl->regs->At2.AT2_RADT[1].AT2_RADT_1_STS;
	getRadt1Symbols[slotcount]  = CSL_FEXT(hIqn2Fl->regs->At2.AT2_RADT[1].AT2_RADT_0_STS,IQN_AT2_AT2_RADT_0_STS_RADT_SYMCNT_VAL);
	getRadt0Frames[slotcount]   = hIqn2Fl->regs->At2.AT2_RADT[0].AT2_RADT_1_STS;
	getRadt0Symbols[slotcount]  = CSL_FEXT(hIqn2Fl->regs->At2.AT2_RADT[0].AT2_RADT_0_STS,IQN_AT2_AT2_RADT_0_STS_RADT_SYMCNT_VAL);
	getBCNFrame[slotcount]      = hIqn2Fl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
	getAil0TmStatus[slotcount]      = CSL_FEXT(hIqn2Fl->regs->Ail[0].AIL_PHY_TM.AIL_PHY_TM_STS,IQN_AIL_AIL_PHY_TM_STS_FRM_STATE) ;

}

void symbolCount(Iqn2Fl_Handle   hIqn2Fl)
{

	symbolcount++;

	getCpuTimestamp[symbolcount] 	   = TSCL;
	getRadt1Frames[symbolcount]   = hIqn2Fl->regs->At2.AT2_RADT[1].AT2_RADT_1_STS;
	getRadt0Frames[symbolcount]   = hIqn2Fl->regs->At2.AT2_RADT[0].AT2_RADT_1_STS;
	getBCNFrame[symbolcount]      = hIqn2Fl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;

}

////////////////////////////////////////////////////////////////////////////////////////////////////
////							Swi dedicated for getting egress and ingress packet count			//
////////////////////////////////////////////////////////////////////////////////////////////////////
void packetCount(Iqn2Fl_Handle   hIqn2Fl, uint32_t aidEnable)
{
	if (aidEnable == 0)
	{
		iqn2SymbolEgressCount[slotcount]      = hIqn2Fl->regs->Ail[0].AIL_IQ_EDC_REGISTER_GROUP.AIL_IQ_EDC_EOP_CNTR_STS;
		iqn2SymbolIngressCount[slotcount]     = hIqn2Fl->regs->Ail[0].AIL_IQ_INGRESS_VBUS_MMR_GROUP.AIL_IQ_IDC_EOP_CNTR_STS;
	} else {
		iqn2SymbolEgressCount[slotcount]      = hIqn2Fl->regs->Aid2.AID2_IQ_EDC_REGISTER_GROUP.AID2_IQ_EDC_EOP_CNTR_STS;
		iqn2SymbolIngressCount[slotcount]     = hIqn2Fl->regs->Aid2.AID2_IQ_INGRESS_VBUS_MMR_GROUP.AID2_IQ_IDC_EOP_CNTR_STS;
		iqn2CtlIngressCount[slotcount]        = hIqn2Fl->regs->Aid2.AID2_CTL_INGRESS_VBUS_MMR_GROUP.AID2_ICTL_EOP_CNTR_STS;
		iqn2CtlIngressPktStatus[slotcount]    = hIqn2Fl->regs->Aid2.AID2_ICTL_IDC_IF.AID2_ICTL_INPKT_STS;
	}
    iqn2RxPktStatus0[slotcount] = hIqn2Fl->regs->Top.VC_CDMA_STATUS.VC_CDMA_RSTAT_L_PKT_STS;
    iqn2RxPktStatus1[slotcount] = hIqn2Fl->regs->Top.VC_CDMA_STATUS.VC_CDMA_RSTAT_H_PKT_STS;
}

void symbolPacketCount(Iqn2Fl_Handle   hIqn2Fl, uint32_t aidEnable)
{
	if (aidEnable == 0)
	{
		iqn2SymbolEgressCount[symbolcount] = hIqn2Fl->regs->Ail[1].AIL_IQ_EDC_REGISTER_GROUP.AIL_IQ_EDC_EOP_CNTR_STS;
		iqn2SymbolIngressCount[symbolcount]  = hIqn2Fl->regs->Ail[1].AIL_IQ_INGRESS_VBUS_MMR_GROUP.AIL_IQ_IDC_EOP_CNTR_STS;
	} else {
		iqn2SymbolEgressCount[symbolcount] = hIqn2Fl->regs->Aid2.AID2_IQ_EDC_REGISTER_GROUP.AID2_IQ_EDC_EOP_CNTR_STS;
		iqn2SymbolIngressCount[symbolcount] = hIqn2Fl->regs->Aid2.AID2_IQ_INGRESS_VBUS_MMR_GROUP.AID2_IQ_IDC_EOP_CNTR_STS;
		iqn2CtlIngressCount[symbolcount]    =  hIqn2Fl->regs->Aid2.AID2_CTL_INGRESS_VBUS_MMR_GROUP.AID2_ICTL_EOP_CNTR_STS;
	}
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////
////							Swi dedicated for packet pushing									//
////////////////////////////////////////////////////////////////////////////////////////////////////
void slotPushing(PktDmaConfigHandle hPktDma)
{
	int32_t                i,chan;
	uint32_t               payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;
//	Complex16			*checkPtr;

	uint32_t 				nbchanneltotal, firstChan;


	nbchanneltotal = hPktDma->numAxC;
	firstChan = hPktDma->firstAxC;

	/*
	 * In this Isr, at this slot, push all s7 ymbols for all AxCs, links at once
	 * It assumes that all symbolPkt were first popped from the FDQs, and properly populated.
	 * AIF2 will returned them in FDQs following the actual transmission via AIF2 protocol encoder.
	 * Note that AIF2 Rx PktDMA channels will starve for data checking since we don't perform
	 * Rx packet recycling at runtime. We don't keep pushing packets beyond a certain slot.
	 */
#if NEVER_END_TEST == 0
	if((slotcount >= (SLOT_NUM_FIRST_PUSH)) && (slotcount < SLOT_NUM_DATA_CHECK)){
	#endif
	for(i=symbolNb;i<(symbolNb + NBSYMBOL);i++)
	{
		for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
		{
			// pop from AxC Tx free descriptor queue
			ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[chan]));
			if (ptrMonoDesc == NULL)
				printf("Tx descriptor NULL: descriptor %d, chan %d, slotcount %d \n", i, chan, slotcount);
			// Recreate PS data
			UTILS_cacheInvalidate((void *)ptrMonoDesc, 16);
			Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			payloadPtr[0] = (uint32_t )(0x00008000 + chan + (i << 7));//add symbol number into PS field
			UTILS_cacheWriteBack((void *)ptrMonoDesc, 16); // writeback the updated PS data in MSM
			Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			Qmss_queuePushDesc((hPktDma->txQAxC[chan]), (uint32_t*)ptrMonoDesc);
		}
	}
	symbolNb += NBSYMBOL;
#if LTE_RATE == 1
	if (symbolcount == 20) symbolcount = 0;
#else
	if (symbolNb == IQN2_LTE_FRAME_SYMBOL_NUM) symbolNb = 0;
#endif
#if NEVER_END_TEST == 0
	}
#endif
	// Make sure no overflow occurs on the monitoring arrays
	if (slotcount == (MAX_SLOT-1)) slotcount = 0;
}

void symbolPushing(PktDmaConfigHandle hPktDma)
{
	int32_t                i,chan;
	uint32_t               payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;
//	Complex16			*checkPtr;

	uint32_t 				nbchanneltotal, firstChan;


	nbchanneltotal = hPktDma->numAxC;
	firstChan = hPktDma->firstAxC;

	/*
	 * In this Isr, at this slot, push all s7 ymbols for all AxCs, links at once
	 * It assumes that all symbolPkt were first popped from the FDQs, and properly populated.
	 * AIF2 will returned them in FDQs following the actual transmission via AIF2 protocol encoder.
	 * Note that AIF2 Rx PktDMA channels will starve for data checking since we don't perform
	 * Rx packet recycling at runtime. We don't keep pushing packets beyond a certain slot.
	 */
#if NEVER_END_TEST == 0
	if((symbolcount >= (SYMBOL_NUM_FIRST_PUSH)) && (symbolcount < SYMBOL_NUM_DATA_CHECK)){
#endif
	for(i=symbolNb;i<(symbolNb + 1);i++)
	{
		for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
		{
			// pop from AxC Tx free descriptor queue
			ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[chan]));
			if (ptrMonoDesc == NULL)
				printf("Tx descriptor NULL: descriptor %d, chan %d, slotcount %d \n", i, chan, symbolcount);
			// Recreate PS data
			UTILS_cacheInvalidate((void *)ptrMonoDesc, 16);
			Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			payloadPtr[0] = (uint32_t )(0x00008000 + chan + (i << 7));//add symbol number into PS field
			UTILS_cacheWriteBack((void *)ptrMonoDesc, 16); // writeback the updated PS data in MSM
			Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			Qmss_queuePushDesc((hPktDma->txQAxC[chan]), (uint32_t*)ptrMonoDesc);
		}
	}
	symbolNb ++;
#if LTE_RATE == 1
	if (symbolNb == 20) symbolcount = 0;
#else
	if (symbolNb == IQN2_LTE_FRAME_SYMBOL_NUM) symbolNb = 0;
#endif
#if NEVER_END_TEST == 0
	}
#endif
	// Make sure no overflow occurs on the monitoring arrays
	if (symbolcount == (MAX_SLOT-1)) symbolcount = 0;
}


		//////////////////////////////////////////////////////////////////////////////////////////////////
		//										PRE-LOAD of PACKETS										//
		//////////////////////////////////////////////////////////////////////////////////////////////////
void load_data(PktDmaConfigHandle hPktDma)
{
	uint32_t chan, idx,idx1, idx2, payloadLen;
	Cppi_MonolithicDesc *mono_pkt;
    Qmss_Queue 			 descQueue;
    Cppi_DescTag 		 descTag;
#ifdef DFE_CTL
    Complex16			*payloadPtr;
#else
    uint32_t            *payloadPtr;
#endif
    uint32_t				*ctrPayloadPtr;
    Qmss_Queue           queueInfo;
    uint32_t 				nbchanneltotal, firstChan, nbDesc;

    memset((void*)&iqn2SymbolEgressCount[0], 0, sizeof(iqn2SymbolEgressCount));
    memset((void*)&iqn2SymbolIngressCount[0], 0, sizeof(iqn2SymbolEgressCount));
    memset((void*)&getCpuTimestamp[0], 0, sizeof(getCpuTimestamp));
    memset((void*)&getRadt0Frames[0], 0, sizeof(getRadt0Frames));
    memset((void*)&getRadt1Frames[0], 0, sizeof(getRadt1Frames));
    memset((void*)&getBCNFrame[0], 0, sizeof(getBCNFrame));
    memset((void*)&getRxPktCnt[0], 0, sizeof(getRxPktCnt));
    memset((void*)&symbolNum[0][0], 0, sizeof(symbolNum));
    memset((void*)&rxPktCnt[0], 0, sizeof(rxPktCnt));

    nbchanneltotal = hPktDma->numAxC;
    firstChan = hPktDma->firstAxC;

	for(chan = firstChan;chan < nbchanneltotal+firstChan;chan++)
	{
		idx1=0;
		nbDesc = Qmss_getQueueEntryCount(hPktDma->txFqAxC[chan]);
		for(idx = 0;idx < nbDesc;idx++)
		{
			mono_pkt = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[chan]));
            if (mono_pkt == NULL)
                printf("Tx free descriptor NULL: descriptor %d, chan %d\n", idx, chan);

			//Create Mono packet
			Cppi_setDescType((Cppi_Desc *)mono_pkt,Cppi_DescType_MONOLITHIC);
			Cppi_setPacketType(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
			Cppi_setDataOffset(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,16);

			if((idx%NBSYMBOL) == 0) {
				Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,(hPktDma->txDescSizeAxC[chan]-16));
			} else {
				Cppi_setPacketLen(Cppi_DescType_MONOLITHIC, (Cppi_Desc *)mono_pkt,(hPktDma->txDesc2SizeAxC[chan]-16));
			}

			Cppi_setPSFlags (Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,1);
			Cppi_setPSLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,4);

			queueInfo = Qmss_getQueueNumber(hPktDma->txFqAxC[chan]);
			descQueue.qMgr = queueInfo.qMgr;
			descQueue.qNum = queueInfo.qNum;
			Cppi_setReturnQueue(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,descQueue);

			descTag.destTagHi = 0;					descTag.destTagLo = 0;
			descTag.srcTagHi  = 0;					descTag.srcTagLo  = 0;
			Cppi_setTag(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,&descTag);

			// Get payload address
			Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);

			if((idx%NBSYMBOL) == 0)
			{
				 //payload data setup(first symbol)
				for (idx2 = 0; idx2 < (hPktDma->txDescSizeAxC[chan]-16)/4; idx2 ++)
				{
#if DFE_JESD_LOOPBACK == 1
					getcomplex(&payloadPtr[idx2],idx1,chan, 30.72);
#else
					payloadPtr[idx2]=(idx2/4) + (idx << 16);
#endif
					idx1++;
				}
			} else {
				//payload data setup(other six symbols)
				for (idx2 = 0; idx2 < (hPktDma->txDesc2SizeAxC[chan]-16)/4; idx2 ++)
				{
#if DFE_JESD_LOOPBACK == 1
					getcomplex(&payloadPtr[idx2],idx1,chan, 30.72);
#else
					payloadPtr[idx2]=(idx2/4) + (idx << 16);
#endif
					idx1++;
				}
			}
			//Create PS data
			Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
			*((uint32_t*)payloadPtr) = (uint32_t )(0x00008000 + chan + (idx << 7));//add symbol number into PS field

			//Tx queue push will be done in slot ISR at the pace of 7 symbols / AxCs
			UTILS_cacheWriteBackInvalidate((void*)mono_pkt, hPktDma->txDescSizeAxC[chan]);
			Qmss_queuePushDesc((hPktDma->txFqAxC[chan]), (uint32_t*)mono_pkt);
		}
	}


	if (hPktDma->psMsgEnable == 1)
	{
		nbchanneltotal = hPktDma->numCtrl;
		firstChan = 0;
		if(hPktDma->ctrlDescType == Cppi_DescType_MONOLITHIC)
		{
			for(chan = firstChan;chan < nbchanneltotal+firstChan;chan++)
			{
				nbDesc = Qmss_getQueueEntryCount(hPktDma->txFqCtrl[chan]);
				for(idx = 0;idx < nbDesc;idx++)
				{
					mono_pkt = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqCtrl[chan]));
					if (mono_pkt == NULL)
						printf("Tx free control descriptor NULL: descriptor %d, chan %d \n", idx, chan);
					queueInfo = Qmss_getQueueNumber(hPktDma->txFqCtrl[chan]);
					descQueue.qMgr = queueInfo.qMgr;
					descQueue.qNum = queueInfo.qNum;
					Cppi_setReturnQueue(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,descQueue);
					// Get payload address
					Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&ctrPayloadPtr, &payloadLen);

					 //payload data setup(first symbol)
					for (idx2 = 0; idx2 < (hPktDma->txDescSizeCtrl[chan]-16)/4; idx2 ++)
					{
						ctrPayloadPtr[idx2]=(idx2/4);
						idx1++;
					}

					//Tx queue push will be done in slot ISR at the pace of 7 symbols / AxCs
					UTILS_cacheWriteBackInvalidate((void*)mono_pkt, hPktDma->txDescSizeCtrl[chan]);
					Qmss_queuePushDesc((hPktDma->txFqCtrl[chan]), (uint32_t*)mono_pkt);
				}
			}
		}
	}
}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// 											FINAL CHECK																   //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void lteFinalCheck(PktDmaConfigHandle hPktDma)
{
	uint32_t  monoRxCount, monoTxCount, rx_count;
	uint32_t  chan, i, idx, idx2, value;
	uint32_t  testpass;
	uint32_t *payloadPtr;
    uint32_t  nbchanneltotal, firstChan;

    nbchanneltotal = hPktDma->numAxC;
    firstChan = hPktDma->firstAxC;

    printf(" \n");

	/* Check received symbol packet data */
	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
	{
		monoRxCount = Qmss_getQueueEntryCount(hPktDma->rxQAxC[chan]); // get number of received packed on first channel
		monoTxCount = Qmss_getQueueEntryCount(hPktDma->txFqAxC[chan]); // channel 0 only
		printf("Number of monolithic packets received in RX queue channel %d: %d\n", chan, monoRxCount);
		printf("Number of monolithic packets in TX free queue for channel %d: %d\n", chan, monoTxCount);
	}

	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
	{
		testpass = 1;
		rx_count = Qmss_getQueueEntryCount((hPktDma->rxQAxC[chan]));
		if (rx_count == 0) testpass =0;

		for (idx = 0; idx < rx_count; idx ++)
		{
			symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQAxC[chan]));
            if (symbolPkt[chan][idx] == NULL)
                printf("Tx free descriptor NULL: descriptor %d, chan %d\n", idx, chan);
			UTILS_cacheInvalidate((void*)symbolPkt[chan][idx], hPktDma->txDescSizeAxC[chan]);
			payloadPtr = (uint32_t *)symbolPkt[chan][idx];
			payloadPtr += 4; //skip pkt header and PS field (16 bytes)
			testpass = 1;
#if DFE_BB_LOOPBACK == 1
			if((idx%NBSYMBOL) == 0)
			{
				for (idx2 = 0; idx2 < (hPktDma->txDescSizeAxC[chan]-16)/4; idx2 ++)
				{
					if(payloadPtr[idx2]!=((idx2/4)+ (idx << 16)))
						testpass = 0;
				}

			} else {
				for (idx2 = 0; idx2 < (hPktDma->txDesc2SizeAxC[chan]-16)/4; idx2 ++)
				{
					if(payloadPtr[idx2]!=((idx2/4)+ (idx << 16)))
						testpass = 0;
				}
			}
#endif
			Qmss_queuePushDesc(hPktDma->rxFqAxC[chan], (uint32_t*) symbolPkt[chan][idx]);

		}
		printf(" Test a) Monolithic Packet Data Recv: on chan: %d are :%d \n", chan, rx_count);
	}
#if DFE_BB_LOOPBACK == 1
		if (testpass == 1) {
			printf(" Test a) Monolithic Packet Data Send/Recv: PASS on channel: %d \n", chan);
		} else {
			printf(" Test a) Monolithic Packet Data Send/Recv: FAIL\n");
			testcheck++;
		}
#endif

		/* read the descriptor counts of the Monolithic queues. */
		value =0;
		for (i =firstChan ; i< nbchanneltotal+firstChan; i++) {
			value += Qmss_getQueueEntryCount(hPktDma->txQAxC[i]);
		}
		if (value != 0) printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d FAIL\n",value);
		else printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d PASS\n",value);
		value =0;
		for (i =firstChan ; i< nbchanneltotal+firstChan; i++) {
			value += Qmss_getQueueEntryCount(hPktDma->txFqAxC[i]); // channel0
		}
		if (value != (nbchanneltotal)*NBSYMBOL*2) printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
		else printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d PASS\n",value);
		value =0;
		for (i = firstChan ; i< nbchanneltotal+firstChan; i++) {
			value += Qmss_getQueueEntryCount(hPktDma->rxQAxC[i]);
		}
		if (value != 0) printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d FAIL\n",value);
		else printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d PASS\n",value);
		value =0;
		for (i = firstChan ; i< nbchanneltotal+firstChan; i++) {
			value += Qmss_getQueueEntryCount(hPktDma->rxFqAxC[i]);
		}
		if (value != (nbchanneltotal)*NBSYMBOL*2) printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
		else printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);
}

Cppi_MonolithicDesc* symbolPktCtl[4][NUMDESCCTL];
void pktDmaCtrlFinalCheck(PktDmaConfigHandle hPktDma)
{
	uint32_t 			monoRxCount, monoTxCount, rx_count, testpass;
    uint32_t 			nbchanneltotal, firstChan, chan, idx, idx2, value, i;
	uint32_t            *payloadPtr;


    nbchanneltotal = hPktDma->numCtrl;
    firstChan = 0;
	/* Check received symbol packet data */
	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
	{
		monoRxCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[chan]); // get number of received packed on first channel
		monoTxCount = Qmss_getQueueEntryCount(hPktDma->txFqCtrl[chan]); // channel 0 only
		printf(" \n Number of control packets received in RX queue channel %d: %d\n", chan, monoRxCount);
		printf(" Number of control packets in TX free queue for channel %d: %d\n", chan, monoTxCount);
	}

	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
		{
			testpass = 1;
			rx_count = Qmss_getQueueEntryCount((hPktDma->rxQCtrl[chan]));
			if (rx_count == 0) testpass =0;

			for (idx = 0; idx < rx_count; idx ++)
			{
				symbolPktCtl[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[chan]));
	            if (symbolPktCtl[chan][idx] == NULL)
	                printf("Rx descriptor NULL: descriptor %d, chan %d\n", idx, chan);
				CACHE_invL1d((void *)symbolPktCtl[chan][idx], hPktDma->txDescSizeAxC[chan], CACHE_WAIT);
				payloadPtr = (uint32_t *)symbolPktCtl[chan][idx];
				payloadPtr += 4; //skip pkt header and PS field (16 bytes)
				testpass = 1;

				for (idx2 = 0; idx2 < (720-16)/4; idx2 ++)
				{
					if(payloadPtr[idx2]!=(idx2/4))
						testpass = 0;
				}
				Qmss_queuePushDesc(hPktDma->rxFqCtrl[chan], (uint32_t*) symbolPktCtl[chan][idx]);

			}
			printf(" Test a) Control Packet Data Recv: on chan: %d are :%d \n", chan, rx_count);
		}

			if (testpass == 1) {
				printf(" Test a) Control Packet Send/Recv: PASS on channel: %d \n", chan);
			} else {
				printf(" Test a) Control Packet Send/Recv: FAIL\n");
			}

			/* read the descriptor counts of the Monolithic queues. */
			value =0;
			for (i =firstChan ; i< nbchanneltotal+firstChan; i++) {
				value += Qmss_getQueueEntryCount(hPktDma->txQCtrl[i]);
			}
			if (value != 0) printf(" Test b1) Control Packet Tx Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b1) Control Packet Tx Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i =firstChan ; i< nbchanneltotal+firstChan; i++) {
				value += Qmss_getQueueEntryCount(hPktDma->txFqCtrl[i]); // channel0
			}
			if (value != (nbchanneltotal)*NUMDESCCTL) printf(" Test b2) Control Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b2) Control Packet Tx Complete Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i = firstChan ; i< nbchanneltotal+firstChan; i++) {
				value += Qmss_getQueueEntryCount(hPktDma->rxQCtrl[i]);
			}
			if (value != 0) printf(" Test b3) Control Packet Rx Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b3) Control Packet Rx Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i = firstChan ; i< nbchanneltotal+firstChan; i++) {
				value += Qmss_getQueueEntryCount(hPktDma->rxFqCtrl[i]);
			}
			if (value != (nbchanneltotal)*NUMDESCCTL) printf(" Test b4) Control Packet Rx Free Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b4) Control Packet Rx Free Descriptor Counts:%d PASS\n",value);

}

