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

#include <ti/drv/iqn2tests/testIQN2LLD_aid/src/edmaUtils.h>
#include <ti/drv/iqn2tests/testIQN2LLD_aid/src/mathUtils.h>
#include "ltePlaybackUtils.h"
#include "checkRfUtils.h"

#include <ti/drv/iqn2tests/testIQN2LLD_aid/src/DSP_fft16x16.h>
#include <ti/drv/iqn2tests/testIQN2LLD_aid/src/gen_twiddle_fft16x16.h>
#include <ti/drv/iqn2tests/testIQN2LLD_aid/src/gen_twiddle_fft16x16.c>

#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
volatile uint32_t packets_already_pushed = 1;
#else
volatile uint32_t packets_already_pushed = 0;
#endif

volatile uint32_t iqn2SymbolEgressCount[MAX_SLOT];
volatile uint32_t iqn2SymbolIngressCount[MAX_SLOT];
volatile uint32_t getCpuTimestamp[MAX_SLOT];
volatile uint32_t getRadt0Frames[MAX_SLOT];
volatile uint32_t getRadt0Symbols[MAX_SLOT];
volatile uint32_t getRadt1Frames[MAX_SLOT];
volatile uint32_t getRadt1Symbols[MAX_SLOT];
volatile uint32_t getBCNFrame[MAX_SLOT];
volatile uint32_t getRxPktCnt[MAX_SLOT];
volatile uint32_t symbolNum[MAX_SLOT][7];
volatile uint32_t rxPktCnt[8 * 2];
volatile uint32_t rxCnt[8 * 2][MAX_SLOT];
volatile uint32_t symbolcount = 0;
volatile uint32_t dbgslotcount = 0;
volatile uint32_t symbolNb = 0;
volatile uint32_t slotcount = 0;
volatile uint32_t rxSymNum;

extern int        PEAKDEBUG;

Cppi_MonolithicDesc* symbolPkt[8 * 2][IQN2_LTE_FRAME_SYMBOL_NUM];
/* Storing first sample of each packet to check packet sequence at runtime */
Complex16 firstSample[8*2][IQN2_LTE_FRAME_SYMBOL_NUM];

#pragma DATA_SECTION(rxSamples,".iqn2descddr");
uint32_t  rxSamples[4][140][SYMBOLSIZE20];

#pragma DATA_SECTION(testInfo,".iqn2captcmd");
test_info_t	testInfo = 	{
		0x50CCA06F, 		//magic;
		0x00000001, 		//revision;
		0 ,//SPECIAL_SEQUENCE, 	//type;
		#ifdef LOOPBACK
		1, 					//loopback;
		#else
		0, 					//no loopback;
		#endif
		-1, 				//test_case;
		0, 					//capture_requested;
		1, 					//never_end set by default;
		1,					//use_cio
		0, 					//error_condition
		0,					//force_stop
		0                  //alive_test
};

/*
 * Buffers storing TX and RX symbols for debug and verification
 */
#pragma DATA_SECTION(rxSlots,".iqn2rxcapture");
lte_slot_t  rxSlots[NBCHANNEL][20]; // can capture up to 10ms per channel

uint32_t  capture_ongoing   = 0;
uint32_t  capture_AxC       = 0;
uint32_t  *captPtr;

uint8_t   rxBuffer[(NBCHANNEL)+1][SYMBOLSIZE20-16];  // MUST BE IN LL2 to enable HW cache coherency
uint8_t   rxCheck[NBCHANNEL][NBSYMBOL];               // Keeps track of current check of all LTE symbols on a slot per AxC
uint16_t  rxRuntimeFail[NBCHANNEL][NBSYMBOL];         // Keeps track of the number of failures per LTE symbols on a slot per AxC
uint16_t  rxRuntimeSuccess=0;                         // Counter


/*
 *  Variables used fft software
 */
#pragma DATA_ALIGN(twiddleFacts, 8);
int16_t   twiddleFacts[2*N + 2*PAD];


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


	/*
	 * Rx packet recycling until final check, we need available descriptors in Rx FDQs when actual data is pushed
	 */

	if ((testInfo.never_end == 0) && (slotcount == (SLOT_NUM_DATA_CHECK - (hPktDma->numAxC * 20 + 20))))
	    testInfo.capture_requested = 1;

	if ((slotcount < SLOT_NUM_DATA_CHECK) || (testInfo.never_end == 1))
	{
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
                // Store Rx symbol number
                rxSymNum = *(((uint32_t*)ptrMonoDesc) + 3);
                rxSymNum = (rxSymNum>>7)&0x000000FF;

                if((rxPktCnt[chan]%NBSYMBOL) == 0)
                {
                    //check payload length for first symbol
                    if (payloadLen != ((hPktDma->rxDescSizeAxC[chan]-16))) {
                        UTILS_iqn2ExceptIntDisable();
                        slotcount = SLOT_NUM_DATA_CHECK  + 1;
                        testInfo.never_end = 0;
                        testInfo.error_condition = -1;
                        printf("FATAL: unexpected payload length:%d, %d\n",((hPktDma->rxDescSizeAxC[chan]-16)),payloadLen);
                    }
                } else {
                    //check payload length for the other 6 symbols
                    if (payloadLen != ((hPktDma->txDesc2SizeAxC[chan]-16))) {
                        UTILS_iqn2ExceptIntDisable();
                        slotcount = SLOT_NUM_DATA_CHECK  + 1;
                        testInfo.never_end = 0;
                        testInfo.error_condition = -1;
                        printf("FATAL: unexpected payload length:%d, %d\n",((hPktDma->txDesc2SizeAxC[chan]-16)),payloadLen);
                    }
                }

                if ((capture_ongoing == 1) && (capture_AxC == chan)) {
                    if((rxPktCnt[chan]%NBSYMBOL) == 0)
                    {
                        UTILS_triggerEdmaSampleCapt((rxPktCnt[chan]%NBSYMBOL),(uint32_t *)captPtr ,(uint32_t *)payloadPtr, (hPktDma->rxDescSizeAxC[chan]-16));
                        captPtr += ((hPktDma->rxDescSizeAxC[chan] - 16) / 4);
                    } else {
                        UTILS_triggerEdmaSampleCapt((rxPktCnt[chan]%NBSYMBOL),(uint32_t *)captPtr ,(uint32_t *)payloadPtr, (hPktDma->txDesc2SizeAxC[chan]-16));
                        captPtr += ((hPktDma->txDesc2SizeAxC[chan] - 16) / 4);
                    }
                }

                // recycle symbol packet on rx free queue
                Qmss_queuePushDesc(hPktDma->rxFqAxC[chan], (uint32_t*)ptrMonoDesc);
                rxPktCnt[chan]++;
                // handle 32-bit wrap here
                if (rxPktCnt[chan] == 0xFFFFFFFC) rxPktCnt[chan] = 0;
                rxCnt[chan][dbgslotcount]++;
                if (rxPktCnt[chan] == 4294967292) rxPktCnt[chan] = 0; //secure the wrap of rxPktCnt to a multiple of 7
            }
        }
	}

    if ((slotcount < (SLOT_NUM_DATA_CHECK-1)) || (testInfo.never_end == 1))
    {
        if (rxSymNum == (IQN2_LTE_FRAME_SYMBOL_NUM - 1))
        {
            if ((testInfo.capture_requested==1) && (capture_ongoing==1)) {
                capture_AxC++;
                if (capture_AxC == hPktDma->numAxC)
                {
                    capture_ongoing   = 0;
                    testInfo.capture_requested = 0;
                } else {
                    captPtr = (uint32_t*) rxPtr[capture_AxC];
                }
            }
            if ((testInfo.capture_requested==1) && (capture_ongoing==0)) {
                capture_ongoing   = 1;
                capture_AxC       = 0;
                captPtr = (uint32_t*) rxPtr[capture_AxC];
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////							Swi dedicated for getting timings (core 0)							//
////////////////////////////////////////////////////////////////////////////////////////////////////
void slotCount(Iqn2Fl_Handle   hIqn2Fl)
{
	slotcount++;dbgslotcount++;
	getCpuTimestamp[dbgslotcount] 	= TSCL;
	getRadt1Frames[dbgslotcount]   = hIqn2Fl->regs->At2.AT2_RADT[1].AT2_RADT_1_STS;
	getRadt1Symbols[dbgslotcount]  = CSL_FEXT(hIqn2Fl->regs->At2.AT2_RADT[1].AT2_RADT_0_STS,IQN_AT2_AT2_RADT_0_STS_RADT_SYMCNT_VAL);
	getRadt0Frames[dbgslotcount]   = hIqn2Fl->regs->At2.AT2_RADT[0].AT2_RADT_1_STS;
	getRadt0Symbols[dbgslotcount]  = CSL_FEXT(hIqn2Fl->regs->At2.AT2_RADT[0].AT2_RADT_0_STS,IQN_AT2_AT2_RADT_0_STS_RADT_SYMCNT_VAL);
	getBCNFrame[dbgslotcount]      = hIqn2Fl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;

	if(testInfo.alive_test == 1)
	    testInfo.alive_test = 0;

}

////////////////////////////////////////////////////////////////////////////////////////////////////
////							Swi dedicated for getting egress and ingress packet count			//
////////////////////////////////////////////////////////////////////////////////////////////////////
void packetCount(Iqn2Fl_Handle   hIqn2Fl, uint32_t aidEnable)
{
	if (aidEnable == 0)
	{
		iqn2SymbolEgressCount[dbgslotcount] = hIqn2Fl->regs->Ail[0].AIL_IQ_EDC_REGISTER_GROUP.AIL_IQ_EDC_EOP_CNTR_STS;
		iqn2SymbolIngressCount[dbgslotcount]  = hIqn2Fl->regs->Ail[0].AIL_IQ_INGRESS_VBUS_MMR_GROUP.AIL_IQ_IDC_EOP_CNTR_STS;
	} else {
		iqn2SymbolEgressCount[dbgslotcount] = hIqn2Fl->regs->Aid2.AID2_IQ_EDC_REGISTER_GROUP.AID2_IQ_EDC_EOP_CNTR_STS;
		iqn2SymbolIngressCount[dbgslotcount] = hIqn2Fl->regs->Aid2.AID2_IQ_INGRESS_VBUS_MMR_GROUP.AID2_IQ_IDC_EOP_CNTR_STS;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////							Swi dedicated for packet pushing									//
////////////////////////////////////////////////////////////////////////////////////////////////////
void slotPushing(PktDmaConfigHandle hPktDma)
{
	int32_t              i,j,chan;
	uint32_t             payloadLen;
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

	if((slotcount >= (SLOT_NUM_FIRST_PUSH)) && ((slotcount < (SLOT_NUM_DATA_CHECK+1)) || (testInfo.never_end == 1))){

        for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
        {
            for(i=symbolNb;i<(symbolNb + NBSYMBOL);i++)
            {
                j = (i+NBSYMBOL)%140;
                symbolPkt[chan][j] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[chan]));

				if (symbolPkt[chan][i] == NULL)
					printf("Tx descriptor NULL: descriptor %d, chan %d, slotcount %d \n", i, chan, slotcount);
				// Recreate PS data
				UTILS_cacheInvalidate((void *)symbolPkt[chan][i], 16);
				Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)symbolPkt[chan][i], (uint8_t**)&payloadPtr, &payloadLen);
				payloadPtr[0] = (uint32_t )(0x00008000 + chan + (i << 7));//add symbol number into PS field
				UTILS_cacheWriteBack((void *)symbolPkt[chan][i], 16); // writeback the updated PS data in MSM
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)symbolPkt[chan][i], (uint8_t**)&payloadPtr, &payloadLen);
//				checkPtr = (Complex16*)payloadPtr;

//				if ((checkPtr->re != firstSample[chan][i].re) || (checkPtr->im != firstSample[chan][i].im)) {
//					UTILS_iqn2ExceptIntDisable();
//					slotcount = SLOT_NUM_DATA_CHECK  + 1;
//					testInfo.never_end = 0;
//					testInfo.error_condition = -1;
//					printf("FATAL: unexpected first sample in this packet\n");
//				}

                Qmss_queuePushDesc((hPktDma->txQAxC[chan]), (uint32_t*)symbolPkt[chan][i]);

                Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)symbolPkt[chan][j], (uint8_t**)&payloadPtr, &payloadLen);
                if((j%NBSYMBOL) == 0)
                {
                    UTILS_triggerEdmaSampleCapt(((j%NBSYMBOL) + ((chan+1)*NBSYMBOL)),(uint32_t *)payloadPtr,(uint32_t *)txPtr[chan], (hPktDma->txDescSizeAxC[chan]-16));
                    txPtr[chan] += ((hPktDma->txDescSizeAxC[chan] - 16) / 4);
                } else {
                    UTILS_triggerEdmaSampleCapt(((j%NBSYMBOL) + ((chan+1)*NBSYMBOL)),(uint32_t *)payloadPtr ,(uint32_t *)txPtr[chan], (hPktDma->txDesc2SizeAxC[chan]-16));
                    txPtr[chan] += ((hPktDma->txDesc2SizeAxC[chan] - 16) / 4);
                }
            }
		}

        if (j == (IQN2_LTE_FRAME_SYMBOL_NUM - 1)) resetAxCPtr();

		symbolNb += NBSYMBOL;
		if (symbolNb == IQN2_LTE_FRAME_SYMBOL_NUM) symbolNb = 0;
		if (slotcount == (SLOT_NUM_DATA_CHECK-1)) UTILS_iqn2ExceptIntDisable();
	}

	if (slotcount == (SLOT_NUM_FIRST_PUSH - 1))
    {
        for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
        {
            for (i=0;i<NBSYMBOL;i++)
            {
                symbolPkt[chan][i] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[chan]));
            }
        }
    }

	// Make sure no overflow occurs on the monitoring arrays
	if (dbgslotcount == (MAX_SLOT-1)) dbgslotcount = 0;
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
    Complex16           *payloadPtr;
    Qmss_Queue           queueInfo;
    uint32_t 			nbchanneltotal, firstChan, nbDesc;
    uint32_t            fftSize;

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
    resetAxCPtr();
	for(chan = firstChan;chan < nbchanneltotal+firstChan;chan++)
	{
        if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE20_FFT_SIZE+IQN2_LTE20_CYPRENORMAL1_SIZE)*4 + 16))
        {
            fftSize = IQN2_LTE20_FFT_SIZE;
        } else if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE10_FFT_SIZE+IQN2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)) {
            fftSize = IQN2_LTE10_FFT_SIZE;
        } else if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)) {
            fftSize = IQN2_LTE5_FFT_SIZE;
        }

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
                    getTMdata(&payloadPtr[idx2],chan);
                    idx1++;
                }
            } else {
                //payload data setup(other six symbols)
                for (idx2 = 0; idx2 < (hPktDma->txDesc2SizeAxC[chan]-16)/4; idx2 ++)
                {
                    getTMdata(&payloadPtr[idx2],chan);
                    idx1++;
                }
            }
            firstSample[chan][idx].re =  payloadPtr[0].re;
            firstSample[chan][idx].im =  payloadPtr[0].im;

            //Create PS data
            Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
            *((uint32_t*)payloadPtr) = (uint32_t )(0x00008000 + chan + (idx << 7));//add symbol number into PS field

            //Tx queue push will be done in slot ISR at the pace of 7 symbols / AxCs
            UTILS_cacheWriteBackInvalidate((void*)mono_pkt, hPktDma->txDescSizeAxC[chan]);
            Qmss_queuePushDesc((hPktDma->txFqAxC[chan]), (uint32_t*)mono_pkt);
        }
	}
	resetAxCPtr();

#if SPECIAL_SEQUENCE == 0
            gen_twiddle_fft16x16(&twiddleFacts[PAD], fftSize);
            memset(rxCheck,0x00,sizeof(rxCheck));
            memset(rxRuntimeFail,0x00,sizeof(rxRuntimeFail));
#endif
}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// 											FINAL CHECK																   //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void lteFinalCheck(PktDmaConfigHandle hPktDma)
{
	uint32_t  monoRxCount, monoTxCount, rx_count;
	uint32_t  chan, i, idx, value;
	uint32_t  testpass;
    uint32_t  nbchanneltotal, firstChan;
    uint32_t  cpSize, cp2Size, fftSize;
    Complex16            *payloadPtr;
    float     samplFreq;
#if (DFE_BB_LOOPBACK == 1) && (SPECIAL_SEQUENCE == 0)
    Complex16            sampleCheck;
    uint32_t             idx1, idx2;
#endif

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

	testpass = 1;
	for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
	{
        if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE20_FFT_SIZE+IQN2_LTE20_CYPRENORMAL1_SIZE)*4 + 16))
        {
            samplFreq = 30.72;
            cpSize    = IQN2_LTE20_CYPRENORMAL1_SIZE;
            cp2Size   = IQN2_LTE20_CYPRENORMAL_SIZE;
            fftSize   = IQN2_LTE20_FFT_SIZE;
        } else if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE10_FFT_SIZE+IQN2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)) {
            samplFreq = 15.36;
            cpSize    = IQN2_LTE10_CYPRENORMAL1_SIZE;
            cp2Size   = IQN2_LTE10_CYPRENORMAL_SIZE;
            fftSize   = IQN2_LTE10_FFT_SIZE;
        } else if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)) {
            samplFreq = 7.68;
            cpSize    = IQN2_LTE5_CYPRENORMAL1_SIZE;
            cp2Size   = IQN2_LTE5_CYPRENORMAL_SIZE;
            fftSize   = IQN2_LTE5_FFT_SIZE;
        }

		PEAKDEBUG = 1;
#if (DFE_BB_LOOPBACK == 1) && (SPECIAL_SEQUENCE == 0)
		idx1 = 0;
#endif
		rx_count = Qmss_getQueueEntryCount((hPktDma->rxQAxC[chan]));
		if (rx_count == 0)  {testpass =0;testcheck++;}

		for (idx = 0; idx<rx_count; idx++)
		{
		    symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQAxC[chan]));
		    Qmss_queuePushDesc(hPktDma->rxFqAxC[chan], (uint32_t*) symbolPkt[chan][idx]);
		}
		payloadPtr =  rxPtr[chan];
        UTILS_cacheInvalidate((void *)payloadPtr, 1228800);
		for (idx = 0; idx < 140; idx ++)
		{
			if((idx%NBSYMBOL) == 0)
			{
//				UTILS_cacheInvalidate((void *)payloadPtr, hPktDma->txDescSizeAxC[chan]);
				memcpy((void*)&rxBuffer[chan][0],(void*)payloadPtr, (hPktDma->txDescSizeAxC[chan]-16));

#if (DFE_BB_LOOPBACK == 1) && (SPECIAL_SEQUENCE == 0)
                for (idx2 = 0; idx2 < (hPktDma->txDescSizeAxC[chan]-16)/4; idx2 ++)
                {
                    getcomplex(&sampleCheck,idx1,chan,samplFreq);
                    if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
                    {
                        testpass = 0; testcheck++;
                    }
                    idx1++;
                }
#endif
                payloadPtr += (hPktDma->txDescSizeAxC[chan]-16)/4;

				memcpy((void*)&rxSamples[chan][idx][0],(void*)rxBuffer[chan], (hPktDma->txDescSizeAxC[chan]-16));
#if SPECIAL_SEQUENCE == 0
				DSP_fft16x16(&twiddleFacts[PAD], fftSize, (int16_t*)&rxBuffer[chan][cpSize*4], (int16_t*)&rxBuffer[(NBCHANNEL)][0]);
				if((chan == 0) || (chan == 2)||(chan == 4) || (chan == 6))
				{
					if (!checkSingleTone((int16_t*)&rxBuffer[(NBCHANNEL)][0],fftSize,(fftSize/samplFreq*FREQ),0)) {
						testpass = 0;testcheck++;
						rxCheck[chan][0] = 0;
						rxRuntimeFail[chan][0]++;
					}
				} else {
					if (!checkDualTone((int16_t*)&rxBuffer[(NBCHANNEL)][0],fftSize,(fftSize/samplFreq*FREQ), (fftSize/samplFreq*FREQ*2),1)) {
						testpass = 0;testcheck++;
						rxCheck[chan][0] = 0;
						rxRuntimeFail[chan][0]++;
					}
				}
#endif
			} else {
//				UTILS_cacheInvalidate((void *)payloadPtr, hPktDma->txDesc2SizeAxC[chan]);
				memcpy((void*)&rxBuffer[chan][0],(void*)payloadPtr,(hPktDma->txDesc2SizeAxC[chan]-16));

#if (DFE_BB_LOOPBACK == 1) && (SPECIAL_SEQUENCE == 0)
//                payloadPtr = (Complex16 *)rxBuffer[chan];
                for (idx2 = 0; idx2 < (hPktDma->txDesc2SizeAxC[chan]-16)/4; idx2 ++)
                {
                    getcomplex(&sampleCheck,idx1,chan,samplFreq);
                    if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
                    {
                        testpass = 0; testcheck++;
                    }
                    idx1++;
                }
#endif
                payloadPtr += (hPktDma->txDesc2SizeAxC[chan]-16)/4;

				memcpy((void*)&rxSamples[chan][idx][0],(void*)rxBuffer[chan],hPktDma->txDesc2SizeAxC[chan]-16);
#if SPECIAL_SEQUENCE == 0
				DSP_fft16x16(&twiddleFacts[PAD], fftSize, (int16_t*)&rxBuffer[chan][cp2Size*4], (int16_t*)&rxBuffer[(NBCHANNEL)][0]);
				if((chan == 0) || (chan == 2)||(chan == 4) || (chan == 6))
				{
					if (!checkSingleTone((int16_t*)&rxBuffer[(NBCHANNEL)][0],fftSize,(fftSize/samplFreq*FREQ),0)) {
						testpass = 0;testcheck++;
						rxCheck[chan][idx%NBSYMBOL] = 0;
						rxRuntimeFail[chan][idx%NBSYMBOL]++;
					}
				} else {
					if (!checkDualTone((int16_t*)&rxBuffer[(NBCHANNEL)][0],fftSize,(fftSize/samplFreq*FREQ), (fftSize/samplFreq*FREQ*2),1)) {
						testpass = 0;testcheck++;
						rxCheck[chan][idx%NBSYMBOL] = 0;
						rxRuntimeFail[chan][idx%NBSYMBOL]++;
					}
				}
#endif
			}
		}
		printf(" Test a) Monolithic Packet Data Recv: on chan: %d are :%d \n", chan, rx_count);
	}

		if (testpass == 1) {
			printf(" Test a) Monolithic Packet Data Send/Recv: PASS on channel: %d \n", chan);
		} else {
			printf(" Test a) Monolithic Packet Data Send/Recv: FAIL\n");
			for(chan = firstChan; chan < nbchanneltotal+firstChan; chan++)
			{
				for (idx = 0; idx < NBSYMBOL; idx ++)
				{
					if (rxRuntimeFail[chan][idx]) {
						printf("Rx data check failed on Antenna%d Symbol%d\n",chan,idx);
					}
				}
			}
		}

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
		if (value != (nbchanneltotal)*14) printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
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
		if (value != (nbchanneltotal)*14) printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
		else printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);
		
		
}

void UTILS_genDummyInitData(PktDmaConfigHandle hPktDma)
{
    uint32_t chan, idx, idx2;
    uint32_t *bufPtr;

    resetAxCPtr();

    for (chan=0;chan<hPktDma->numAxC;chan++)
    {
        bufPtr = (uint32_t*) txPtr[chan];

        for(idx = 0;idx < 140;idx++)
        {
            if((idx%NBSYMBOL) == 0)
            {
                 //payload data setup(first symbol)
                for (idx2 = 0; idx2 < ((hPktDma->txDescSizeAxC[chan] - 16) / 4); idx2 ++)
                {
                    *bufPtr = (idx2/4)+(idx<<16);
                    bufPtr++;
                }
            } else {
                //payload data setup(other six symbols)
                for (idx2 = 0; idx2 < ((hPktDma->txDesc2SizeAxC[chan] - 16) / 4); idx2 ++)
                {
                    *bufPtr = (idx2/4)+(idx<<16);
                    bufPtr++;
                }
            }
        }
    }
}

void UTILS_genSingleDualTone(PktDmaConfigHandle hPktDma)
{
    uint32_t chan, idx, idx1, idx2;
    Complex16 *bufPtr;
    float sampleFreq;

    resetAxCPtr();

    for (chan=0;chan<hPktDma->numAxC;chan++)
    {
        idx1 = 0;
        bufPtr = txPtr[chan];

        if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE20_FFT_SIZE+IQN2_LTE20_CYPRENORMAL1_SIZE)*4 + 16))
        {
            sampleFreq = 30.72;
        } else if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE10_FFT_SIZE+IQN2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)) {
            sampleFreq = 15.36;
        } else if (hPktDma->txDescSizeAxC[chan] == (uint32_t)((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)) {
            sampleFreq = 7.68;
        }

        for(idx = 0;idx < 140;idx++)
        {
            if((idx%NBSYMBOL) == 0)
            {
                 //payload data setup(first symbol)
                for (idx2 = 0; idx2 < ((hPktDma->txDescSizeAxC[chan] - 16) / 4); idx2 ++)
                {
                    getcomplex(bufPtr,idx1,chan, sampleFreq);
                    idx1++;
                    bufPtr++;
                }
            } else {
                //payload data setup(other six symbols)
                for (idx2 = 0; idx2 < ((hPktDma->txDesc2SizeAxC[chan] - 16) / 4); idx2 ++)
                {
                    getcomplex(bufPtr,idx1,chan, sampleFreq);
                    idx1++;
                    bufPtr++;
                }
            }
        }
    }
}


