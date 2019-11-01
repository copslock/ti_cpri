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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>

#include <ti/csl/csl.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/cslr_tmr.h>
#include <ti/csl/csl_tmrAux.h>

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>

#if EVM_TYPE == 0
#include <EVM.h>
#endif

#include <cslUtils.h>
#include <mnavUtils.h>

#define     TRIG_TIMER          0

#define     TEST_NUM            2
#define     TEST_ALL            0

#define     _AIF2FL_DUMP      0   // Enable the DUMP of the AIF configuration, for debug purposes

// pre-proc constant: EVM_TYPE -> 0 = Lyrtech board, 1 = Advantech board DSP_1, 2 = Advantech setup DSP_2
#define     CLKIN1_INPUT_KHZ    122880  // on Lyrtech  EVM , the Frequency is based on an external 122.88 MHz clock
#if EVM_TYPE == 0
#define     SYSCLK_INPUT_KHZ    153600  // on Lyrtech EVM , the Frequency is based on an external 153.6 MHz clock
#else
#define     SYSCLK_INPUT_KHZ    122880  // on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#endif
#define     PLLC_PREDIV_CLK   	1       // PLLC_PREDIV_CLK - 1 set to register
#define     PLLC_PLLM_CLK     	8       // PLLC_PLLM_CLK - 1 set to register (983MHz CPU clock)

volatile unsigned int verbose       = 1;
volatile unsigned int NB_ITERATIONS = 1;
volatile unsigned int ntest         = 0;
volatile unsigned int testcheck     = 1;
#ifdef LOOPBACK
volatile unsigned int intLoopback   = 1;
volatile unsigned int swSync        = 1;
#else
volatile unsigned int intLoopback   = 0;
volatile unsigned int swSync        = 0;
#endif

volatile unsigned char testEnable[TEST_NUM] =
{  // Each of those need to be run individually
#if EVM_TYPE !=6
	1,		// CPRI 4x for GENERIC PACKET - Link 0
#else
	0,
#endif
	1		// CPRI 4x for GENERIC PACKET - Link 0 and Link 1

};

typedef struct {
        // Test name
        const unsigned char name[32];
        //AIF link enable or disable, 1 or 0
        uint32_t                   linkEnable[AIF_MAX_NUM_LINKS];
        //AIF link rate (1=1x; 2=2x; 4= 4x).
        uint32_t                   linkRate[AIF_MAX_NUM_LINKS];
        //AIF link data type for outbound burst traffic.
        Aif2Fl_LinkDataType     outboundDataType[AIF_MAX_NUM_LINKS];
        //AIF link data width for outbound burst traffic.
        Aif2Fl_DataWidth        outboundDataWidth[AIF_MAX_NUM_LINKS];
        //AIF link data type for inbound burst traffic.
        Aif2Fl_LinkDataType     inboundDataType[AIF_MAX_NUM_LINKS];
        //AIF link data width for inbound burst traffic.
        Aif2Fl_DataWidth        inboundDataWidth[AIF_MAX_NUM_LINKS];
        //AIF link DIO engine used
        Aif2Fl_DioEngineIndex   dioEngine[AIF_MAX_NUM_LINKS];
        } TestObj;


//////////////////////
// Global variables
AIF_ConfigObj               aifObj;
AIF_ConfigHandle            hConfigAif = &aifObj;

volatile TestObj                     testObjTab[TEST_NUM] = {
	{//1st Test - CPRI 4x for GENERIC PACKET - Link 0
	  "CPRI_4X_GP_L0", // test name
	 // link0          link1          link2          link3          link4          link5
	  {0,             0,             0,             0,             1,             0            },  // link enable
	  {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	  {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
	  {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
	},
	{//1st Test - CPRI 4x for GENERIC PACKET - Link 0 and Link 1
      "CPRI_4X_GP_L0&L1", // test name
   // link0          link1          link2          link3          link4          link5
     {1,             1,             0,             0,             0,             0            },  // link enable
     {4,             4,             4,             4,             4,             4            },  // link rate
     {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
     {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
     {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
     {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
     {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
    }
   };


/* Define queues for common FDQs */
#define HOST_TX_COMPLETE_Q     2000
#define HOST_RX_FDQ            2006

/* These are for the AIF test */
#define HOST_RX_Q              1000
#define HOST_TX_Q              512

#define NUM_AXC                2
#define BUFF_LEN               512
#define NUM_PKTDESC               16
//Users should use 16 bytes aligned data for Aif2 and PktDMA test
//#pragma DATA_SECTION(mono_region_test,".extData_sect")//use DDR3 memory for test mode
//Users should use 16 bytes aligned data for Aif2 and pkt dma test
#pragma DATA_ALIGN (host_region, 2*NUM_PKTDESC)
uint8_t   host_region[4 * NUM_PKTDESC * NUM_AXC * 64];//32 64 byte descriptors
#pragma DATA_ALIGN (txBuffers, 2*NUM_PKTDESC)
uint32_t   txBuffers[2*NUM_PKTDESC * (BUFF_LEN/4)];
#pragma DATA_ALIGN (rxBuffers, 2*NUM_PKTDESC)
uint32_t   rxBuffers[2*NUM_PKTDESC * (BUFF_LEN/4)];
#pragma DATA_ALIGN (rxBuffers1, 2*NUM_PKTDESC)
uint32_t   rxBuffers1[2*NUM_PKTDESC * (BUFF_LEN/4)];
uint32_t  tmp0[NUM_PKTDESC*NUM_AXC], tmp1[NUM_PKTDESC*NUM_AXC];


volatile unsigned int int4_result;
volatile uint32_t aif2SymbolEgressCount[100];
volatile uint32_t aif2SymbolIngressCount[100];

uint32_t printexceptions = 1;

Cppi_RxFlowCfg rxFlowCpriGP[AIF_MAX_NUM_LINKS];

#define SF_NUM_FIRST_PUSH	30
#define SF_NUM_DATA_CHECK	130

#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif

void my_interrupt_for_Lte_test(){
    int i, idx, j;
    uint32_t chan;
    uint32_t rxSymCnt;
    Cppi_HostDesc *host_pkt;
    Cppi_HostDesc *host_pkt1;

    if (int4_result <SF_NUM_FIRST_PUSH)
    {
        chan = 0;
    	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
    	{
    		if (testObjTab[ntest].linkEnable[i]){
    			rxSymCnt = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]);
				for (idx = 0; idx < rxSymCnt; idx ++)
				{
					host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(HOST_RX_Q+chan));
					host_pkt1 = (Cppi_HostDesc*)Cppi_getNextBD (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt);

					Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*) host_pkt);
					Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*) host_pkt1);
    			}
				chan++;
    		}
    	}
    }

    if(int4_result == SF_NUM_FIRST_PUSH){
        chan = 0;
		for (i=0;i<AIF_MAX_NUM_LINKS;i++)
		{
			if (testObjTab[ntest].linkEnable[i]){
                for(j=0;j<NUM_PKTDESC;j++){
                    Qmss_queuePushDesc(aifObj.pktDmaConfig.txQAxC[chan], (uint32_t*) tmp0[j+(NUM_PKTDESC*chan)]);
                }
                chan++;
			}
		}
    }
	aif2SymbolEgressCount[int4_result]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
	aif2SymbolIngressCount[int4_result] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
    int4_result++;
}

int main(void)
{
    uint32_t  idx, idx2, i;
    uint32_t  chan;
    uint16_t  testpass = 0;
    uint16_t myboard	= EVM_TYPE;
	uint32_t  hostRxCount;
    uint32_t *temp;
    uint32_t  rx_count, value;
    Cppi_HostDesc *host_pkt;
    Cppi_HostDesc *host_pkt1;
    Cppi_DescTag   descTag;

    fsevt1_userIsr = my_interrupt_for_Lte_test;
    printf("Beginning AIF2 CPRI GENERIC PACKET TEST:\n");
    UTILS_waitForHw(100000);
    
    if (myboard == 0)  UTILS_configMasterSlave();
    else DSP_procId = EVM_TYPE ; //in case advantech it could be board 1 or 2
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    DSP_procId = 1;
#endif

   UTILS_resetTimer(TRIG_TIMER);
   UTILS_initTimer(TRIG_TIMER);

   fsevt1_userIsr = my_interrupt_for_Lte_test;

	// Take AIF out of power saver
	AIF_enable();

    for(ntest=0;ntest<TEST_NUM;ntest++)
    {
 	    if (testEnable[ntest] == 1 )
 	    {
 	    	int4_result = 0;
			// General parameters
 	    	memset(&aifObj, 0, sizeof(aifObj));

 	    	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
			aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
			aifObj.pktdmaOrDioEngine  = AIF2FL_CPPI;
			aifObj.mode               = AIF_GENERICPACKET_MODE;
			if (swSync == 0)
				aifObj.aif2TimerSyncSource= AIF2FL_CHIP_INPUT_SYNC;
			else
				aifObj.aif2TimerSyncSource= AIF2FL_SW_SYNC;

			if (aifObj.mode == AIF_GENERICPACKET_MODE) {
			    chan = 0;
				for (i=0;i<AIF_MAX_NUM_LINKS;i++)
				{
					if (testObjTab[ntest].linkEnable[i]){
						// PktDma parameters for control words (CPRI control word FastC&M)
						aifObj.pktDmaConfig.txRegionAxC[chan]   = UTILS_getMemRegionNum(host_region);
						aifObj.pktDmaConfig.txNumDescAxC[chan]  = 2*NUM_PKTDESC;
						aifObj.pktDmaConfig.txDescSizeAxC[chan] = BUFF_LEN;
						aifObj.pktDmaConfig.rxRegionAxC[chan]   = UTILS_getMemRegionNum(host_region);
						aifObj.pktDmaConfig.rxNumDescAxC[chan]  = 2*NUM_PKTDESC;
						aifObj.pktDmaConfig.rxDescSizeAxC[chan] = BUFF_LEN;

						memset(&rxFlowCpriGP[i], 0, sizeof(Cppi_RxFlowCfg));
						rxFlowCpriGP[i].rx_dest_qnum     = HOST_RX_Q+chan;
						rxFlowCpriGP[i].rx_fdq0_sz0_qnum = HOST_RX_FDQ+chan;
						rxFlowCpriGP[i].rx_desc_type     = (uint8_t)Cppi_DescType_HOST;    // HOST
						rxFlowCpriGP[i].rx_fdq1_qnum     = HOST_RX_FDQ+chan;
						rxFlowCpriGP[i].rx_fdq2_qnum     = HOST_RX_FDQ+chan;
						rxFlowCpriGP[i].rx_fdq3_qnum     = HOST_RX_FDQ+chan;

						aifObj.pktDmaConfig.hRxFlowAxC[chan]    = &rxFlowCpriGP[i];
						chan++;
					} else {
						aifObj.pktDmaConfig.hRxFlowAxC[i]    = NULL;
					}
				}
				aifObj.pktDmaConfig.txDataBuff[0]     = UTILS_local2GlobalAddr(&txBuffers[0]);
				aifObj.pktDmaConfig.rxDataBuff[0]     = UTILS_local2GlobalAddr(&rxBuffers[0]);
				aifObj.pktDmaConfig.txDataBuff[1]     = UTILS_local2GlobalAddr(&txBuffers[0]);
				aifObj.pktDmaConfig.rxDataBuff[1]     = UTILS_local2GlobalAddr(&rxBuffers1[0]);
				aifObj.pktDmaConfig.hRxFlowCtrl[0]    = NULL;
				aifObj.pktDmaConfig.hRxFlowCtrl[1]    = NULL;
				aifObj.pktDmaConfig.hRxFlowCtrl[2]    = NULL;
				aifObj.pktDmaConfig.hRxFlowCtrl[3]    = NULL;
			}
			memset(host_region, 0, 4 * NUM_PKTDESC * NUM_AXC * 64);

			// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
		    for (i=0;i<8192;i++){
		    	Qmss_queueEmpty(i);
		    }
			UTILS_doCleanup(&aifObj,TRIG_TIMER);

        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{
				 aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
				 aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[0].linkRate[i];
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_7P68MHZ;
				 aifObj.linkConfig[i].outboundDataType   = testObjTab[ntest].outboundDataType[i];
				 aifObj.linkConfig[i].outboundDataWidth  = testObjTab[ntest].outboundDataWidth[i];
				 aifObj.linkConfig[i].inboundDataType    = testObjTab[ntest].inboundDataType[i];
				 aifObj.linkConfig[i].inboundDataWidth   = testObjTab[ntest].inboundDataWidth[i];
				 aifObj.linkConfig[i].psMsgEnable        = 0;
				 if (intLoopback == 1 ) aifObj.linkConfig[i].comType            = AIF2_LOOPBACK;
				 else	 aifObj.linkConfig[i].comType            = AIF2_2_AIF2;
				 aifObj.linkConfig[i].dioEngine          = testObjTab[ntest].dioEngine[i]; //NA for pkDMA
            }

        	printf("test: %s\n", testObjTab[ntest].name);
	        if (DSP_procId == 1)
				UTILS_printLinkConfig(&aifObj);

			// initialization function for the AT timer and EDMA3 ISRs
			UTILS_aifIntcSetup();

			// leave some time - might not be necessary in the end
			UTILS_waitForHw(1000);
			// initialization function for the AIF2 H/W of the TMS320C6670
			AIF_calcParameters(&aifObj);
			// initialization function for qmss and cppi low-level drivers
			UTILS_initQmss((uint32_t*)host_region, 4 * NUM_PKTDESC * NUM_AXC, 64, 0, NULL);
			// initialization function for the AIF2 H/W of the TMS320C6670
			AIF_initPktDma(&aifObj);
			AIF_initHw(&aifObj);

			chan = 0;
			for (i=0;i<AIF_MAX_NUM_LINKS;i++)
			{
				if (testObjTab[ntest].linkEnable[i]){
					for(idx =0; idx < NUM_PKTDESC; idx++){  //push 8 host linked packets into Tx queue for test
						//Create Host packet Tx descriptor #2 with 2 linked Host Buf desc.

						host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqAxC[chan]));
						host_pkt1 = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqAxC[chan]));
						tmp0[idx+(chan*NUM_PKTDESC)] = (uint32_t) host_pkt;
						tmp1[idx+(chan*NUM_PKTDESC)] = (uint32_t) host_pkt1;
						host_pkt->buffLen = BUFF_LEN;
						Cppi_setPSLocation(Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (Cppi_PSLoc) 1);
						//host_pkt->psv_word_count = 0;
						Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, BUFF_LEN * 2);
						Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (Cppi_Desc*)host_pkt1);
						descTag.srcTagLo = 0;
						Cppi_setTag (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, &descTag);
						Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (uint8_t **)&temp,(uint32_t*) value);
						for (idx2 = 0; idx2 < (BUFF_LEN/4); idx2 ++) temp[idx2] = idx2 + 0x10000;
						host_pkt1->buffLen = BUFF_LEN;
						Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt1, NULL);
						Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt1, (uint8_t **)&temp,(uint32_t*) value);
						for (idx2 = 0; idx2 < (BUFF_LEN/4); idx2 ++) temp[idx2] = idx2 + 0x20000;

						tmp0[idx+(chan*NUM_PKTDESC)] |= 0x00000003;//set DESC_SIZE to 3 for 64 byte host descriptors
						tmp1[idx+(chan*NUM_PKTDESC)] |= 0x00000003;//set DESC_SIZE to 3 for 64 byte host descriptors
						//Tx decriptors will be pushed in ISR for this example
					}
					chan++;
				}
			}
			memset(rxBuffers, 0xFF, sizeof(rxBuffers));
			//memset(txBuffers, 0xAA, sizeof(rxBuffers));

#if _AIF2FL_DUMP == 1
            if (DSP_procId == 1)
            {
               FILE *fout;
               fout = fopen("aif2fl_dump.txt","w");
               if (fout)
               {
                   dump_Aif2Fl_Setup(fout,aifObj.hAif2Setup);
                   fclose(fout);
               }

            }
#endif

		// start AIF2 HW
			AIF_startHw(&aifObj);
			// let DSP2 go behond this point and start timer 0 on DSP1 for external Fsync
			int4_result =0;
			UTILS_initTimer(TRIG_TIMER);
			if (DSP_procId == 1){
				UTILS_triggerFsync(&aifObj);
			}

			if (printexceptions == 1) {
				while (aifFsyncEventCount[1] <= 2);
				AIF_enableException(&aifObj);
			}
    /*****************************************************************
    * Enable AIF and wait for completion.
	*/
		     /*****************************************************************
		    * Enable AIF2 and wait for completion.
		    */
		    while(1)
		    {
		        asm (" NOP 9 ");
		        asm (" NOP 9 ");
		        if(int4_result == SF_NUM_FIRST_PUSH+2)//Wait two phy frame time
		        {
		            break;
		        }

		    }
			if (printexceptions == 1) {
				// disable interrupts before checking for data
				CSR&= 0xFFFFFFFE;
			}

			chan = 0;
			for (i=0;i<AIF_MAX_NUM_LINKS;i++)
			{
				hostRxCount = 0;  // descriptor count for host RX queue
				if (aifObj.linkConfig[i].linkEnable)
				{
					while (hostRxCount < NUM_PKTDESC)
					{
						// Get current descriptor count for host RX queue
						hostRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]);
						if (hostRxCount != 0)
						printf(" Number of monolithic packets received in RX queue: %d\n", hostRxCount);
					}
				}
			}
		   /*****************************************************************
		    * Compare the data in the destination buffers.
		    */

		    /* Compare the Monolithic packet data */
		    testpass = 1;
		    chan = 0;
			for (i=0;i<AIF_MAX_NUM_LINKS;i++)
			{
				if (aifObj.linkConfig[i].linkEnable)
				{
					rx_count = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]);
					for (idx = 0; idx < rx_count; idx ++)
					{
						host_pkt = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(HOST_RX_Q+chan));
						Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt, (uint8_t **)&temp,(uint32_t*) value);
						for (idx2 = 0; idx2 < (BUFF_LEN/4); idx2 ++)if (temp[idx2] != (idx2 + 0x10000)) testpass = 0;
						host_pkt1 = (Cppi_HostDesc*)Cppi_getNextBD (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt);
						Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)host_pkt1, (uint8_t **)&temp,(uint32_t*) value);
						for (idx2 = 0; idx2 < (BUFF_LEN/4); idx2 ++)if (temp[idx2] != (idx2 + 0x20000)) testpass = 0;

						Qmss_queuePushDesc(HOST_RX_FDQ+chan, (uint32_t*) host_pkt);
						Qmss_queuePushDesc(HOST_RX_FDQ+chan, (uint32_t*) host_pkt1);
					}
					chan++;
				}
			}

			for (idx2 = 0; idx2 < (BUFF_LEN/4); idx2 ++)if (rxBuffers[idx2] != (idx2 + 0x10000)) testpass = 0;
			if (ntest==1)
				for (idx2 = 0; idx2 < (BUFF_LEN/4); idx2 ++)if (rxBuffers1[idx2] != (idx2 + 0x10000)) testpass = 0;

		    if (testpass == 1) {
		        //testcheck = 0;
			    printf(" ----------- Test a) Host Packet Data Send/Recv: PASS ------------ \n");
		    } else {
		    	printf(" Test a) Host Packet Data Send/Recv: FAIL\n");
		        testcheck++;
		    }

		    chan = 0;
			for (i=0;i<AIF_MAX_NUM_LINKS;i++)
			{
				if (aifObj.linkConfig[i].linkEnable)
				{
					printf(" Test for Link %d \n",i);
					/* read the descriptor counts of the Host queues. */
					value = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txQAxC[chan]);
					if (value != 0) printf(" Test b1) Host Packet Tx Descriptor Counts:%d FAIL\n",value);
					else printf(" Test b1) Host Packet Tx Descriptor Counts:%d PASS\n",value);

					value = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[chan]);
					if (value != 2*NUM_PKTDESC) printf(" Test b2) Host Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
					else printf(" Test b2) Host Packet Tx Complete Descriptor Counts:%d PASS\n",value);

					value = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]);
					if (value != 0) printf(" Test b3) Host Packet Rx Descriptor Counts:%d FAIL\n",value);
					else printf(" Test b3) Host Packet Rx Descriptor Counts:%d PASS\n",value);

					value = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxFqAxC[chan]);
					if (value != 2*NUM_PKTDESC) printf(" Test b4) Host Packet Rx Free Descriptor Counts:%d FAIL\n",value);
					else printf(" Test b4) Host Packet Rx Free Descriptor Counts:%d PASS\n",value);

					chan++;
				}
			}

			UTILS_doCleanup(&aifObj,TRIG_TIMER);

		    if (printexceptions == 1) {
		    	AIF_printException(&aifObj);
		    }


		    printf("\nEnding AIF2 CPRI Generic packet  test\n");

		}
    }

    if (testcheck == 1) {
       testcheck = 0;
       printf("All tests have passed\n");
    } else {
       printf("Some tests have failed\n");
    }

 	return (0);
}

