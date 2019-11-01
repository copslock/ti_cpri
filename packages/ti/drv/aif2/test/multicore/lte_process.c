#ifdef LTE_MODE

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

#include "lte_process.h"

volatile uint32_t packets_already_pushed = 0;

volatile uint32_t aif2SymbolEgressCount[MAX_SLOT];
volatile uint32_t aif2SymbolIngressCount[MAX_SLOT];
volatile uint32_t getCpuTimestamp[MAX_SLOT];
volatile uint32_t getPhyTimerFrames[MAX_SLOT];
volatile uint32_t getPhyTimerClk[MAX_SLOT];
volatile uint32_t getRxPktCnt[MAX_SLOT];
volatile uint32_t symbolNum[MAX_SLOT][7];
//unsigned char   saveDesc[2][14][32];
volatile uint32_t rxPktCnt[NBCHANNELPERLINK * 2];
volatile uint32_t symbolcount=0;
volatile uint32_t slotcount=0;



void getcomplex(Complex16* payload,uint32_t index, uint32_t tx)
{
	float freq;
	Complex16 x1;
	Complex16 y;
//		if (tx == 0)
//		{
			// 1 MHz signal
                freq = 1;
                x1.re = (int16_t) ( (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) ( (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                y = x1;
//		} else if (tx == 1){
//
//                Complex16         x2;
//                // 1 MHz signal
//                freq = 1;
//                x1.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
//                x1.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
//                // 2 MHz signal
//                freq = 2;
//                x2.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
//                x2.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
//                // dual tone signal
//                y.re = x1.re + x2.re;
//                y.im = x1.im + x2.im;
//		}
                payload->re = y.re;
                payload->im = y.im;
}



//////////////////////////////////////////////////////////////////////////////////////////////////
//							Swi dedicated for packet pushing									//
//////////////////////////////////////////////////////////////////////////////////////////////////
void slotRecycling(AIF_PktDmaConfigHandle hPktDma, uint8_t rxBuffer[][LTESYMBOLSIZE-16], uint32_t nbchanneltotal, uint32_t firstChan, uint32_t activeLink)
{
	int32_t                i,chan;
	uint32_t               rxSymCnt, payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;

	slotcount++;

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
			// Invalidate symbol packet header and get the updated payload address + length
			CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
			Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);

#if LTE_RATE == 1
			if (payloadLen != ((LTESYMBOLSIZE-16)*NBCHANNELINIT))
						System_printf("unexpected payload length:%d, %d\n",((LTESYMBOLSIZE-16)*NBCHANNELINIT),payloadLen);
#else
			if((rxPktCnt[chan]%NBSYMBOL) == 0)
			{
				 //check payload length for first symbol

				if (payloadLen != ((LTESYMBOLSIZE-16)*NBCHANNELINIT))
					System_printf("unexpected payload length:%d, %d\n",((LTESYMBOLSIZE-16)*NBCHANNELINIT),payloadLen);
			} else {
				//check payload length for the other 6 symbols
				if (payloadLen != ((LTESYMBOL2SIZE-16)*NBCHANNELINIT))
					System_printf("unexpected payload length:%d, %d\n",((LTESYMBOL2SIZE-16)*NBCHANNELINIT),payloadLen);
			}
#endif

			CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
			Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			symbolNum [slotcount][i] = ((payloadPtr[0] >> 7) & 0xFF);//check symbol number into PS field
			CACHE_wbL1d((void *)payloadPtr, 4, CACHE_WAIT); // writeback the updated PS data in MSM

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
//void slotCount()
//{
//	slotcount++;
//	getCpuTimestamp[slotcount] 	   = TSCL;
//	getPhyTimerFrames[slotcount]   = aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
//	getPhyTimerClk[slotcount]      = aifObj.hFl->regs->AT_PHYT_CLKCNT_VALUE;
//
//	if (slotcount == (SLOT_NUM_FIRST_PUSH+1)) {
//		AIF_enableException(&aifObj);
//	}
//
//	if (slotcount == (SLOT_NUM_DATA_CHECK-1)) UTILS_aif2ExceptIntDisable();
//
//	aif2SymbolIngressCount[slotcount] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
//	aif2SymbolEgressCount[slotcount]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
//
//
//		// Make sure no overflow occurs on the monitoring arrays
//	if (slotcount == (MAX_SLOT-1)) slotcount = 0;
//}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////
////							Swi dedicated for packet pushing									//
////////////////////////////////////////////////////////////////////////////////////////////////////
void slotPushing(AIF_PktDmaConfigHandle hPktDma, uint32_t nbchanneltotal, uint32_t firstChan, Complex16 firstSample[][NBSYMBOL*2], Cppi_MonolithicDesc* symbolPkt[][NBSYMBOL*2])
{
	int32_t                i,chan;
	uint32_t               payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;
	Complex16			*checkPtr;

	/*
	 * In this Isr, at this slot, push all s7 ymbols for all AxCs, links at once
	 * It assumes that all symbolPkt were first popped from the FDQs, and properly populated.
	 * AIF2 will returned them in FDQs following the actual transmission via AIF2 protocol encoder.
	 * Note that AIF2 Rx PktDMA channels will starve for data checking since we don't perform
	 * Rx packet recycling at runtime. We don't keep pushing packets beyond a certain slot.
	 */
	if (!packets_already_pushed) {
		/* The first two slots we push the packets that have been initialized and are stored in symbolPkt[][]  */
		if((slotcount == SLOT_NUM_FIRST_PUSH) || (slotcount == (SLOT_NUM_FIRST_PUSH+1)))
		{
			for(i=symbolcount;i<(symbolcount + NBSYMBOL);i++)
			{
				for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
				{
					Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)symbolPkt[chan][i%(2*NBSYMBOL)], (uint8_t**)&payloadPtr, &payloadLen);
					checkPtr = (Complex16*)payloadPtr;
					if ((checkPtr->re != firstSample[chan][i%(2*NBSYMBOL)].re) || (checkPtr->im != firstSample[chan][i%(2*NBSYMBOL)].im)) {
						System_printf("unexpected first sample in this packet\n");
					}
					Qmss_queuePushDesc((hPktDma->txQAxC[chan]), (uint32_t*)symbolPkt[chan][i%(2*NBSYMBOL)]);
				}
			}
			symbolcount += NBSYMBOL;
#if LTE_RATE == 1
			if (symbolcount == 20) symbolcount = 0;
#else
			if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
#endif
			if (slotcount == (SLOT_NUM_FIRST_PUSH+1)) {
				packets_already_pushed = 1;
			}
		}
	} else {
#if NEVER_END_TEST == 0
		if((slotcount > (SLOT_NUM_FIRST_PUSH+1)) && (slotcount < SLOT_NUM_DATA_CHECK)){
#endif
		for(i=symbolcount;i<(symbolcount + NBSYMBOL);i++)
		{
			for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
			{
				// pop from AxC Tx free descriptor queue
				ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[chan]));
				// Recreate PS data
				CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
				Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
				payloadPtr[0] = (uint32_t )(0x00008000 + chan + (i << 7));//add symbol number into PS field
				CACHE_wbL1d((void *)payloadPtr, 4, CACHE_WAIT); // writeback the updated PS data in MSM
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
				checkPtr = (Complex16*)payloadPtr;
				if ((checkPtr->re != firstSample[chan][i%(2*NBSYMBOL)].re) || (checkPtr->im != firstSample[chan][i%(2*NBSYMBOL)].im)) {
					System_printf("unexpected first sample in this packet\n");
				}
//				memcpy(&saveDesc[chan][i%(2*NBSYMBOL)][0],ptrMonoDesc,32);
				Qmss_queuePushDesc((hPktDma->txQAxC[chan]), (uint32_t*)ptrMonoDesc);
			}
		}
		symbolcount += NBSYMBOL;
#if LTE_RATE == 1
		if (symbolcount == 20) symbolcount = 0;
#else
		if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
#endif
#if NEVER_END_TEST == 0
		}
#endif
	}
	// Make sure no overflow occurs on the monitoring arrays
	if (slotcount == (MAX_SLOT-1)) slotcount = 0;
}



		//////////////////////////////////////////////////////////////////////////////////////////////////
		//										PRE-LOAD of PACKETS										//
		//////////////////////////////////////////////////////////////////////////////////////////////////
void load_data(AIF_PktDmaConfigHandle hPktDma, uint32_t nbchanneltotal, uint32_t firstChan, Complex16 firstSample[][NBSYMBOL*2], Cppi_MonolithicDesc* symbolPkt[][NBSYMBOL*2])
{
	uint32_t chan, idx,idx1, idx2, payloadLen;
	Cppi_MonolithicDesc *mono_pkt;
    Qmss_Queue 			 descQueue;
    Cppi_DescTag 		 descTag;
    Complex16            *payloadPtr;
    Qmss_Queue           queueInfo;

	for(chan = firstChan;chan < nbchanneltotal+firstChan;chan++)
	{
		idx1=0;
		for(idx = 0;idx < (NBSYMBOL*2);idx++)
		{
			symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[chan]));
			mono_pkt = symbolPkt[chan][idx];

			//Create Mono packet
			Cppi_setDescType((Cppi_Desc *)mono_pkt,Cppi_DescType_MONOLITHIC);
			Cppi_setPacketType(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
			Cppi_setDataOffset(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,16);

#if LTE_RATE == 1
			Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,(LTESYMBOLSIZE-16));
#else
			if((idx%NBSYMBOL) == 0) {
				Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,(LTESYMBOLSIZE-16));
			} else {
				Cppi_setPacketLen(Cppi_DescType_MONOLITHIC, (Cppi_Desc *)mono_pkt,(LTESYMBOL2SIZE-16));
			}
#endif

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

#if LTE_RATE == 1
			 //payload data setup(first symbol)
			for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
			{
				getcomplex(&payloadPtr[idx2],idx1,chan);
				idx1++;
			}
#else
			if((idx%NBSYMBOL) == 0)
			{
				 //payload data setup(first symbol)
				for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
				{
					getcomplex(&payloadPtr[idx2],idx1,chan);
					idx1++;
				}
			} else {
				//payload data setup(other six symbols)
				for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
				{
					getcomplex(&payloadPtr[idx2],idx1,chan);
					idx1++;
				}
			}
#endif
			firstSample[chan][idx].re =  payloadPtr[0].re;
			firstSample[chan][idx].im =  payloadPtr[0].im;

			//Create PS data
			Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
			*((uint32_t*)payloadPtr) = (uint32_t )(0x00008000 + chan + (idx << 7));//add symbol number into PS field

			//Tx queue push will be done in slot ISR at the pace of 7 symbols / AxCs
			CACHE_wbInvL1d((void *)mono_pkt, LTESYMBOLSIZE, CACHE_WAIT);
		}
	}
}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// 											FINAL CHECK																   //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void lteFinalCheck(AIF_PktDmaConfigHandle hPktDma, uint8_t rxBuffer[][LTESYMBOLSIZE-16], uint32_t nbchanneltotal, uint32_t firstChan, uint32_t activeLink)
{
	uint32_t monoRxCount, monoTxCount, rx_count;
	uint32_t chan, i, idx, idx1, idx2, value;
	uint32_t testpass;

	Cppi_MonolithicDesc* symbolPkt[NBCHANNELMAXPERLINK][NBSYMBOL*2];

	Complex16            *payloadPtr;
	Complex16			 sampleCheck;

	/* Check received symbol packet data */
	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
	{
		monoRxCount = Qmss_getQueueEntryCount(hPktDma->rxQAxC[chan]); // get number of received packed on first channel
		monoTxCount = Qmss_getQueueEntryCount(hPktDma->txFqAxC[chan]); // channel 0 only
		System_printf(" \nNumber of monolithic packets received in RX queue channel %d: %d\n", chan, monoRxCount);
		System_printf(" Number of monolithic packets in TX free queue for channel %d: %d\n", chan, monoTxCount);
	}

	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
	{
		idx1 = 0;
		testpass = 1;
		rx_count = Qmss_getQueueEntryCount((hPktDma->rxQAxC[chan]));
		if (rx_count == 0) testpass =0;

		for (idx = 0; idx < NBSYMBOL*2; idx ++)
		{
			symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQAxC[chan]));
			payloadPtr = (Complex16 *)symbolPkt[chan][idx];
			payloadPtr += 4; //skip pkt header and PS field (16 bytes)
			testpass = 1;
#if LTE_RATE == 1
			for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
			{
				getcomplex(&sampleCheck,idx1,chan);
				if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
				{
					testpass = 0; testcheck++;
				}
				idx1++;
			}
#else
			if((idx%NBSYMBOL) == 0)
			{
				for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
				{
					getcomplex(&sampleCheck,idx1,chan);
					if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
					{
						testpass = 0; testcheck++;
					}
					idx1++;
				}

			} else {
				for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
				{
					getcomplex(&sampleCheck,idx1,chan);
						if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
					{
						testpass = 0; testcheck++;
					}
					idx1++;
				}
			}
#endif
			Qmss_queuePushDesc(hPktDma->rxFqAxC[chan], (uint32_t*) symbolPkt[chan][idx]);

		}
		System_printf(" Test a) Monolithic Packet Data Recv: on chan: %d are :%d \n", chan, rx_count);
	}

		if (testpass == 1) {
			System_printf(" Test a) Monolithic Packet Data Send/Recv: PASS on channel: %d \n", chan);
			//testcheck = 0;
		} else {
			System_printf(" Test a) Monolithic Packet Data Send/Recv: FAIL\n");
		}

		/* read the descriptor counts of the Monolithic queues. */
		value =0;
		for (i =firstChan ; i< nbchanneltotal+firstChan; i++) {
			value += Qmss_getQueueEntryCount(hPktDma->txQAxC[i]);
		}
		if (value != 0) System_printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d FAIL\n",value);
		else System_printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d PASS\n",value);
		value =0;
		for (i =firstChan ; i< nbchanneltotal+firstChan; i++) {
			value += Qmss_getQueueEntryCount(hPktDma->txFqAxC[i]); // channel0
		}
		if (value != (nbchanneltotal)*NBSYMBOL*2) System_printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
		else System_printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d PASS\n",value);
		value =0;
		for (i = firstChan ; i< nbchanneltotal+firstChan; i++) {
			value += Qmss_getQueueEntryCount(hPktDma->rxQAxC[i]);
		}
		if (value != 0) System_printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d FAIL\n",value);
		else System_printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d PASS\n",value);
		value =0;
		for (i = firstChan ; i< nbchanneltotal+firstChan; i++) {
			value += Qmss_getQueueEntryCount(hPktDma->rxFqAxC[i]);
		}
		if (value != (nbchanneltotal)*NBSYMBOL*2) System_printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
		else System_printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);

		System_printf("\nEnding test\n");
}

#endif
