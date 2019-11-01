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

#ifdef RUNTIME

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_xmcAux.h>

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/aif2/device/aif2_device.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>

#include <ti/drv/aif2/test/utils/cslUtils.h>
#include <ti/drv/aif2/test/utils/mnavUtils.h>

#define     NB_LINKS              1
#define     LTE_RATE              20
#define     LTESYMBOLSIZE       ((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL1_SIZE)*4 + 16)    // 8848: FFT size + CP first
#define     LTESYMBOL2SIZE      ((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL_SIZE)*4 + 16)     // 8784: FFT size + CP 2nd-6th
#define     FFT_SIZE              AIF2_LTE20_FFT_SIZE
#define     CYPRENORMAL1_SIZE     AIF2_LTE20_CYPRENORMAL1_SIZE
#define     CYPRENORMAL_SIZE      AIF2_LTE20_CYPRENORMAL_SIZE
#define     NBSYMBOL              AIF2_LTE_SYMBOL_NUM                                             // Number of symbols per slot
#define     NUM_AXCS_LTE          2
#if LTE_RATE == 20
#define     NBDESCMAX            (64*NB_LINKS)          // Descriptor count rounded to the next power of 2
#elif LTE_RATE == 10
#define     NBDESCMAX            (128*NB_LINKS)          // Descriptor count rounded to the next power of 2
#elif LTE_RATE == 5
#define     NBDESCMAX            (256*NB_LINKS)          // Descriptor count rounded to the next power of 2
#elif LTE_RATE == 40
#define     NBDESCMAX            (32*NB_LINKS)          // Descriptor count rounded to the next power of 2
#endif
#define     SLOT_NUM_FIRST_PUSH 100                     // The first push needs to happen on the last slot number before frame boundary
#define     SLOT_NUM_DATA_CHECK 202//7502                    // test stops after this amount of Lte slots
#define     MAX_DEBUG_SLOT      400                     // For debug, size of monitoring arrays
#define     MONO_RX_FDQ         2012                    // Define Rx queue base index for ingress traffic and the associated Rx free descriptor queue base index
#define     MONO_RX_Q           1000

//Users should use 16 bytes aligned data for IQN2 and PktDMA test
#ifdef _TMS320C6X
#pragma DATA_SECTION(mono_region,".aifdescmsm")//use MSMC memory for test
#pragma DATA_ALIGN (mono_region, 64)
#endif
uint8_t   mono_region[NBDESCMAX * ((LTESYMBOLSIZE/64)+1) * 64]; // aligned on 64 bytes and multiple of 64 bytes for L1 caching

/*
 * Define an array that can contain all descriptor addresses.
 * This way, all packets can be popped from the FDQ and prepared at once for the purpose of this test.
 */
Cppi_MonolithicDesc* symbolPkt[NUM_AXCS_LTE][NBSYMBOL*2];

/*
 * Define Rx flows for AIF2 ingress traffic
 * The flow will tell AIF2 from which FDQ to pop new symbol descriptors
 */
Cppi_RxFlowCfg rxFlow[NUM_AXCS_LTE];
Cppi_RxFlowCfg rxFlowCtrl[4];

/* AIF2 FL for runtime access */
AIF_PktDmaConfigObj  pktDmaObj;
AIF_CfgObj           aifCfgObj; // for test only
Aif2Fl_Obj          aifObj;
Aif2Fl_Context      aif2Context;
Aif2Fl_Param        aif2Param;
Aif2Fl_Handle       hAifFl;

/* PKTDMA */

volatile unsigned int slotcount = 0;
volatile unsigned int dbgslotcount = 0;
volatile unsigned int symbolcount = 0;
volatile unsigned int testcheck = 1;    // reports pass fail at the end of the test
volatile unsigned int rxSymNum;
volatile unsigned int rxPktCnt[NUM_AXCS_LTE];
volatile unsigned int packets_already_pushed = 0;
volatile unsigned int aif2SymbolEgressCount[MAX_DEBUG_SLOT];
volatile unsigned int aif2SymbolIngressCount[MAX_DEBUG_SLOT];
volatile unsigned int getCpuTimestamp[MAX_DEBUG_SLOT];
volatile unsigned int getPhyTimerFrames[MAX_DEBUG_SLOT];
volatile unsigned int getPhyTimerClk[MAX_DEBUG_SLOT];

void slotIsr();
void init_data(AIF_PktDmaConfigObj* hPktDma);
void check_data(AIF_PktDmaConfigObj* hPktDma);

int main(void)
{

    uint32_t   chan, idx;
    Aif2Fl_Status status;

    /* Make shared memory (MSM) non cacheable for the purpose of testing */
    CSL_XMC_invalidatePrefetchBuffer();
    CACHE_setMemRegionInfo(12,1,0); // MAR12 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(13,1,0); // MAR13 - cacheable (always), not prefetchable

    CACHE_setL1DSize (CACHE_L1_32KCACHE);
    CACHE_setL1PSize (CACHE_L1_32KCACHE);

    AIF_enable(); // need to init PktDMA part of AIF2

    printf("\nStarting LTE runtime program\n");
    memset(&pktDmaObj, 0, sizeof(pktDmaObj));
    // Initialize CSL library, this step is required
    status = Aif2Fl_init(&aif2Context);
    if (status != AIF2FL_SOK)
        printf("bad context initialization \n");

    hAifFl = Aif2Fl_open(&aifObj, 0, &aif2Param, &status);

    // AIF2 LLD programs AT event 5 with period 0.5ms in case of LTE
    aif2evt5_userIsr = slotIsr;

    // initialization function for the interrupt controllers (NO_AIF_LLD)
	UTILS_aifIntcSetup();

	// PktDma parameters
    pktDmaObj.firstAxC	= 0;
	pktDmaObj.numAxC    = NUM_AXCS_LTE;
	pktDmaObj.mode      = AIF_LTE_FDD_MODE;
    chan = 0;
	for (idx=0 ; idx<NUM_AXCS_LTE ; idx++)
	{
		pktDmaObj.txRegionAxC[idx]   = UTILS_getMemRegionNum(mono_region);
		pktDmaObj.txNumDescAxC[idx]  = NBSYMBOL*2; // double num of Pkts
		pktDmaObj.txDescSizeAxC[idx] = LTESYMBOLSIZE;
		pktDmaObj.rxRegionAxC[idx]   = UTILS_getMemRegionNum(mono_region);
		pktDmaObj.rxNumDescAxC[idx]  = NBSYMBOL*2; // double num of Pkts
		pktDmaObj.rxDescSizeAxC[idx] = LTESYMBOLSIZE;
		memset(&rxFlow[idx], 0, sizeof(Cppi_RxFlowCfg));
		rxFlow[idx].rx_dest_qnum     = MONO_RX_Q+idx;
		rxFlow[idx].rx_fdq0_sz0_qnum = MONO_RX_FDQ+idx;
		rxFlow[idx].rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // MONO
		rxFlow[idx].rx_fdq1_qnum     = MONO_RX_FDQ+idx;
		rxFlow[idx].rx_fdq2_qnum     = MONO_RX_FDQ+idx;
		rxFlow[idx].rx_fdq3_qnum     = MONO_RX_FDQ+idx;
		rxFlow[idx].rx_sop_offset    = 12+4;   // desc header size for Monolithic packet with PS
        rxFlow[idx].rx_psinfo_present = 1;

		pktDmaObj.hRxFlowAxC[idx]   = &rxFlow[idx];
		pktDmaObj.hRxFlowCtrl[idx]  = NULL;
		chan++;
	}

	memset(mono_region, 0, sizeof(mono_region));

	// initialization function for qmss and cppi low-level drivers
	UTILS_initQmss((uint32_t*)mono_region, NBDESCMAX, ((LTESYMBOLSIZE/64)+1) * 64, 0, NULL);

	UTILS_initPktDma(&pktDmaObj);
	aifCfgObj.coreCfg[0].linkEnable[0] = 1;
	aifCfgObj.coreCfg[0].numAxC[0]     = NUM_AXCS_LTE;
	UTILS_initCppiChannel(&aifCfgObj, &pktDmaObj);

	init_data(&pktDmaObj);

	printf("\nReady for LTE20MHz 4 AxCs test\n");

    slotcount = 0;
    dbgslotcount = 0;

    /****** wait for data transfer completion.***********************/
    while(1)
    {
        asm (" NOP 5 ");
        asm (" NOP 5 ");

        if(slotcount >= SLOT_NUM_DATA_CHECK+2)
         {
            CSR&= 0xFFFFFFFE;
            Aif2Fl_reset(hAifFl);
            break;
         }
    }

    check_data(&pktDmaObj);

    printf("\nEnding LTE20MHz 4AxCs test\n");
    UTILS_resetQmss(&pktDmaObj);
#ifdef USERM
    UTILS_resetRm();
#endif

    return(0);

}

void init_data(AIF_PktDmaConfigObj* hPktDma)
{
    uint32_t              chan, idx, idx2, payloadLen;
    uint32_t              *payloadPtr;
    Cppi_MonolithicDesc *mono_pkt;
    Qmss_Queue           descQueue;
    Cppi_DescTag         descTag;
    Qmss_Queue           queueInfo;
    /*
     *  LTE Tx packet symbol initialization at once
     */
    for(chan = 0;chan < NUM_AXCS_LTE;chan++)
    {
        for(idx = 0;idx < (NBSYMBOL*2);idx++)
        {
            symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->txFqAxC[chan]));
            mono_pkt = symbolPkt[chan][idx];

            //Create Mono packet
            Cppi_setDescType((Cppi_Desc *)mono_pkt,Cppi_DescType_MONOLITHIC);
            Cppi_setPacketType(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
            Cppi_setDataOffset(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,16);

            if((idx%NBSYMBOL) == 0) {
                Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,(LTESYMBOLSIZE-16));
            } else {
                Cppi_setPacketLen(Cppi_DescType_MONOLITHIC, (Cppi_Desc *)mono_pkt,(LTESYMBOL2SIZE-16));
            }

            Cppi_setPSFlags (Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,1);
            Cppi_setPSLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,4);

            queueInfo = Qmss_getQueueNumber(hPktDma->txFqAxC[chan]);
            descQueue.qMgr = queueInfo.qMgr;
            descQueue.qNum = queueInfo.qNum;
            Cppi_setReturnQueue(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,descQueue);

            descTag.destTagHi = 0;                  descTag.destTagLo = 0;
            descTag.srcTagHi  = 0;                  descTag.srcTagLo  = 0;
            Cppi_setTag(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,&descTag);

            // Get payload address
            Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
            if((idx%NBSYMBOL) == 0)
            {
                 //payload data setup(first symbol)
                for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++) payloadPtr[idx2] = (chan << 24) +  (idx << 16) + idx2;
            } else {
                //payload data setup(other six symbols)
                for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++) payloadPtr[idx2] = (chan << 24) + (idx << 16) + idx2;
            }

            //Create PS data
            Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
            payloadPtr[0] = (uint32_t )(0x00008000 + chan + (idx << 7));//add symbol number into PS field

            //Tx queue push will be done in subframe ISR at once
        }
    }
    /*
     * Perform block writeback on L1D to make sure all symbol packets are coherent with MSM
     */
    for(idx = 0;idx < NBDESCMAX;idx++) {
        CACHE_wbInvL1d((void *)(mono_region+(idx*LTESYMBOLSIZE)), LTESYMBOLSIZE, CACHE_WAIT);
    }
}

void slotIsr()
{
    int32_t                i,chan;
    uint32_t               rxSymCnt, payloadLen, *payloadPtr;
    Cppi_MonolithicDesc *ptrMonoDesc;

    slotcount++;dbgslotcount++;
    getCpuTimestamp[dbgslotcount]     = TSCL;
    getPhyTimerFrames[dbgslotcount]   = hAifFl->regs->AT_PHYT_FRM_VALUE_LSBS;
    getPhyTimerClk[dbgslotcount]      = hAifFl->regs->AT_PHYT_CLKCNT_VALUE;

    /*
     * Rx packet recycling until final check, we need available descriptors in Rx FDQs when actual data is pushed
     */
    if (slotcount < SLOT_NUM_DATA_CHECK)
    {
        for(chan = 0; chan < NUM_AXCS_LTE; chan++)
        {
            rxSymCnt = Qmss_getQueueEntryCount(pktDmaObj.rxQAxC[chan]);

            for(i=0;i<rxSymCnt;i++)
            {
                  ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(pktDmaObj.rxQAxC[chan]));
                  // Invalidate symbol packet header and get the updated payload address + length
                  CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
                  Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);

                  // Store Rx symbol number
                  rxSymNum = *(((uint32_t*)ptrMonoDesc) + 3);
                  rxSymNum = (rxSymNum>>7)&0x000000FF;

                  if((rxPktCnt[chan]%NBSYMBOL) == 0)
                  {
                        //check payload length for first symbol
                        if (payloadLen != (LTESYMBOLSIZE-16)) {
                            slotcount = SLOT_NUM_DATA_CHECK  + 1;
                            printf("FATAL: unexpected payload length:%d, %d\n",(LTESYMBOLSIZE-16),payloadLen);
                        }

                  } else {
                        //check payload length for the other 6 symbols
                        if (payloadLen != (LTESYMBOL2SIZE-16)) {
                            slotcount = SLOT_NUM_DATA_CHECK  + 1;
                            printf("FATAL: unexpected payload length:%d, %d\n",(LTESYMBOL2SIZE-16),payloadLen);
                        }
                  }
                  // recycle symbol packet on rx free queue
                  Qmss_queuePushDesc(pktDmaObj.rxFqAxC[chan], (uint32_t*)ptrMonoDesc);
                  rxPktCnt[chan]++;
            }
        }
    }

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
                for(chan = 0; chan < NUM_AXCS_LTE; chan++)
                {
                    Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)symbolPkt[chan][i], (uint8_t**)&payloadPtr, &payloadLen);
                    Qmss_queuePushDesc((pktDmaObj.txQAxC[chan]), (uint32_t*)symbolPkt[chan][i]);
                }
            }
            symbolcount += NBSYMBOL;
            if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
            if (slotcount == (SLOT_NUM_FIRST_PUSH+1)) {
                packets_already_pushed = 1;
            }
        }
    } else {
        if((slotcount > (SLOT_NUM_FIRST_PUSH+1)) && (slotcount < SLOT_NUM_DATA_CHECK))
        {
            for(i=symbolcount;i<(symbolcount + NBSYMBOL);i++)
            {
                for(chan = 0; chan < NUM_AXCS_LTE; chan++)
                {
                    // pop from AxC Tx free descriptor queue
                    ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(pktDmaObj.txFqAxC[chan]));
                    // Recreate PS data
                    CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
                    Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
                    payloadPtr[0] = (uint32_t )(0x00008000 + chan + (i << 7));//add symbol number into PS field
                    CACHE_wbL1d((void *)payloadPtr, 4, CACHE_WAIT); // writeback the updated PS data in MSM
                    Qmss_queuePushDesc((pktDmaObj.txQAxC[chan]), (uint32_t*)ptrMonoDesc);
                }
            }
            symbolcount += NBSYMBOL;
            if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
        }
    }

    /* PktDMA activity monitoring */
    aif2SymbolEgressCount[dbgslotcount]  = hAifFl->regs->DB_EDB_EOP_CNT;
    aif2SymbolIngressCount[dbgslotcount] = hAifFl->regs->AD_ISCH_EOP_CNT;
    // Make sure no overflow occurs on the monitoring arrays
    if (dbgslotcount == (MAX_DEBUG_SLOT-1)) dbgslotcount = 0;
}


void check_data(AIF_PktDmaConfigObj* hPktDma)
{
    uint32_t monoRxCount, monoTxCount, testpass, firsterror, rx_count, idx, idx2, chan, value, i, *payloadPtr;

    monoRxCount = Qmss_getQueueEntryCount(hPktDma->rxQAxC[0]); // get number of received packed on first channel
    monoTxCount = Qmss_getQueueEntryCount(hPktDma->txFqAxC[0]); // channel 0 only
    printf(" Number of monolithic packets received in RX queue channel0: %d\n", monoRxCount);
    printf(" Number of monolithic packets in TX free queue for channel0: %d\n", monoTxCount);
    monoRxCount = Qmss_getQueueEntryCount(hPktDma->rxQAxC[1]);
    monoTxCount = Qmss_getQueueEntryCount(hPktDma->txFqAxC[1]);
    printf(" Number of monolithic packets received in RX queue channel1: %d\n", monoRxCount);
    printf(" Number of monolithic packets in TX free queue for channel1: %d\n", monoTxCount);

    /* Check received symbol packet data */
    for(chan =0; chan < NUM_AXCS_LTE; chan++)
    {
        testpass = 1;
        firsterror = 0;
        rx_count = Qmss_getQueueEntryCount((hPktDma->rxQAxC[chan]));
        if (rx_count == 0) testpass =0;
        for (idx = 0; idx < rx_count; idx ++)
        {
            symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQAxC[chan]));
            payloadPtr = (uint32_t *)symbolPkt[chan][idx];
            payloadPtr += 4; //skip pkt header and PS field (16 bytes)
            testpass = 1;
            if((idx%NBSYMBOL) == 0)
            {

                CACHE_invL1d((void *)payloadPtr, LTESYMBOLSIZE-16, CACHE_WAIT);
                for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
                {
                    if (payloadPtr[idx2] != (chan << 24) +  (idx << 16) + idx2)
                    {
                        if (testpass == 1 ) firsterror =idx2;
                            testpass = 0; testcheck++;
                    }
                }
            } else {

                CACHE_invL1d((void *)(payloadPtr), LTESYMBOL2SIZE-16, CACHE_WAIT);
                for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
                {
                    if (payloadPtr[idx2] != (chan << 24) + (idx << 16) + idx2)
                    {
                        if (testpass == 1 ) firsterror =idx2;
                            testpass = 0; testcheck++;
                    }
                }
            }
            if (testpass == 0)
            {
                printf(" Test a) Packet Payload: FAIL on chan:%d\t symbol:%d\t received:%d\t expected:%d\t position:%d \n", \
                         chan , idx, payloadPtr[firsterror],(chan << 24) +  (idx << 16) + firsterror, firsterror);
                firsterror =0;
                testcheck++;
            }
            Qmss_queuePushDesc(hPktDma->rxFqAxC[chan], (uint32_t*) symbolPkt[chan][idx]);

        }
        printf(" Test a) Monolithic Packet Data Recv: on chan: %d are :%d \n", chan, rx_count);
    }

    if (testpass == 1) {
        printf(" Test a) Monolithic Packet Data Send/Recv: PASS on channel: %d \n", chan);

    } else {
        printf(" Test a) Monolithic Packet Data Send/Recv: FAIL\n");
    }

    /* read the descriptor counts of the Monolithic queues. */
    value =0;
    for (i =0 ; i< NUM_AXCS_LTE; i++) {
        value += Qmss_getQueueEntryCount(hPktDma->txQAxC[i]);
    }
    if (value != 0) printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d FAIL\n",value);
    else printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d PASS\n",value);
    value =0;
    for (i =0 ; i< NUM_AXCS_LTE; i++) {
        value += Qmss_getQueueEntryCount(hPktDma->txFqAxC[i]); // channel0
    }
    if (value != NUM_AXCS_LTE*NBSYMBOL*2) printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
    else printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d PASS\n",value);
    value =0;
    for (i =0 ; i< NUM_AXCS_LTE; i++) {
        value += Qmss_getQueueEntryCount(hPktDma->rxQAxC[i]);
    }
    if (value != 0) printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d FAIL\n",value);
    else printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d PASS\n",value);
    value =0;
    for (i =0 ; i< NUM_AXCS_LTE; i++) {
        value += Qmss_getQueueEntryCount(hPktDma->rxFqAxC[i]);
    }
    if (value != NUM_AXCS_LTE*NBSYMBOL*2) printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
    else printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);

    if (testcheck == 1) {
       testcheck = 0;
       printf("All tests have passed\n");
    } else {
       printf("Some tests have failed\n");
    }
}


#endif

//////////////////////////////
