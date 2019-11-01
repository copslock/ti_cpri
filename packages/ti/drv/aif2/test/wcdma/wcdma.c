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

#include "cslUtils.h"
#include "mnavUtils.h"


#define     TEST_NUM            10

#define     TEST_ALL            0

#define     TRIG_TIMER          0

// pre-proc constant: EVM_TYPE -> 0 = Lyrtech board, 1 = Advantech board DSP_1, 2 = Advantech setup DSP_2
#define     CLKIN1_INPUT_KHZ    122880  // on Lyrtech  EVM , the Frequency is based on an external 122.88 MHz clock
#if EVM_TYPE == 0
#define     SYSCLK_INPUT_KHZ    153600  // on Lyrtech EVM , the Frequency is based on an external 153.6 MHz clock
#else
#define     SYSCLK_INPUT_KHZ    122880  // on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#endif
#define     PLLC_PREDIV_CLK   	1       // PLLC_PREDIV_CLK - 1 set to register
#define     PLLC_PLLM_CLK     	8       // PLLC_PLLM_CLK - 1 set to register (983MHz CPU clock)

volatile unsigned int verbose = 1;
volatile unsigned int NB_ITERATIONS = 1;
volatile unsigned int ntest=0;
volatile unsigned int testcheck     = 1;
#ifdef LOOPBACK
volatile unsigned int intLoopback = 1;
volatile unsigned int swSync = 1;
#else
volatile unsigned int intLoopback = 0;
volatile unsigned int swSync = 0;
#endif

volatile unsigned char testEnable[TEST_NUM] =
    {  // Each of those need to be run individually
        1,     	//1st Test - CPRI 4x for DL WCDMA - Link 0
        1,     	//2nd Test - CPRI 4x for UL WCDMA - Link 0
        1,	   	//3rd Test - CPRI 4x for UL WCDMA - Link 0,1,2,3 - Dio 0,1
        1,		//4 Test - CPRI 4x for DL WCDMA - Link 0,1,2,3 - Dio 0,1,2
        1,		//5 Test - OBSAI 4x for UL WCDMA - Link 0
        1,		//6 Test - OBSAI 4x for UL WCDMA - Link 0,1,2,3 - DIO 0,1
        1,		//7 Test - OBSAI 4x for DL WCDMA - Link 0
        1,		//8 Test - OBSAI 4x for DL WCDMA - Link 0,1,2,3 - Dio 1,2
        1,		//9 Test - CPRI 4x for DL WCDMA - Link 0 15bits
        1		//10 Test - CPRI 4x for DL WCDMA - Link 0 with terminated AxC
    };

typedef struct {
        // Test name
        const unsigned char name[32];
        //AIF protcole used
        Aif2Fl_LinkProtocol     protocol;
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
        //AIF number of links per DIO engine
        uint32_t					 nbLinkDio[AIF2_MAX_NUM_DIO_ENGINE];
        } TestObj;


//////////////////////
// Global variables
AIF_ConfigObj               aifObj;
AIF_ConfigHandle            hConfigAif = &aifObj;
AIF_DataTraceObj			aif2TraceObj;

volatile TestObj                     testObjTab[TEST_NUM] = {
	{//1st Test - CPRI 4x for DL WCDMA - Link 0
	  "CPRI one link 4x DL", // test name
	  AIF2FL_LINK_PROTOCOL_CPRI,
   // link0          link1          link2          link3          link4          link5
	 {1,             0,             0,             0,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
	 {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
	 {1,             0,             0},
	},
	{//2nd Test - CPRI 4x for UL WCDMA - Link 0
	  "CPRI one link 4x UL", // test name
	  AIF2FL_LINK_PROTOCOL_OBSAI,
	 // link0          link1          link2          link3          link4          link5
	 {1,             0,             0,             0,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // outboundDataType
	 {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8 },  // outboundDataWidth
	 {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // inboundDataType
	 {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8 },  // oinboundDataWidth
	 {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_2}, //Dio engine
	 {1,             0,             0},                                                           //link enable per DIO engine
	},
	{//3rd Test - CPRI 4x for UL WCDMA - Link 0,1,2,3 - Dio 0,1
      "CPRI Multi link 4x UL", // test name
      AIF2FL_LINK_PROTOCOL_CPRI,
   // link0          link1          link2          link3          link4          link5
     {1,             1,             1,             1,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // outboundDataType
	 {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8 },  // outboundDataWidth
	 {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // inboundDataType
	 {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8 },  // oinboundDataWidth
     {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
     {2,             2,             0},                                                           //link enable per DIO engine
    },
	{//4 Test - CPRI 4x for DL WCDMA - Link 0,1,2,3 - Dio 0,1,2
      "CPRI Multi link 4x DL", // test name
      AIF2FL_LINK_PROTOCOL_CPRI,
   // link0          link1          link2          link3          link4          link5
     {1,             1,             1,             1,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
     {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_2, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
     {2,             1,             1},
    },
	{//5 Test - OBSAI 4x for UL WCDMA - Link 0
      "OBSAI one link 4x UL", // test name
      AIF2FL_LINK_PROTOCOL_OBSAI,
   // link0          link1          link2          link3          link4          link5
     {0,             1,             0,             0,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // outboundDataType
	 {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8 },  // outboundDataWidth
	 {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // inboundDataType
	 {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8 },  // oinboundDataWidth
     {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_2, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
     {1,             0,             0},
    },
	{//6 Test - OBSAI 4x for UL WCDMA - Link 0,1,2,3 - DIO 0,1
      "OBSAI Multi link 4x UL", // test name
      AIF2FL_LINK_PROTOCOL_OBSAI,
   // link0          link1          link2          link3          link4          link5
     {1,             1,             1,             1,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // outboundDataType
	 {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8 },  // outboundDataWidth
	 {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // inboundDataType
	 {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8 },  // oinboundDataWidth
     {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
     {2,             2,             0},
    },
	{//7 Test - OBSAI 4x for DL WCDMA - Link 0
      "OBSAI one link 4x DL", // test name
      AIF2FL_LINK_PROTOCOL_OBSAI,
   // link0          link1          link2          link3          link4          link5
     {1,             0,             0,             0,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
     {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_2, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
     {1,             0,             0},
    },
	{//8 Test - OBSAI 4x for DL WCDMA - Link 0,1,2,3 - Dio 1,2
      "OBSAI Multi link 4x DL", // test name
      AIF2FL_LINK_PROTOCOL_OBSAI,
   // link0          link1          link2          link3          link4          link5
     {1,             1,             1,             1,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
     {AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_2, AIF2FL_DIO_ENGINE_2, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
     {0,             2,             2},
    },
	{//9 Test - CPRI 4x for DL WCDMA - Link 0 15bits
      "CPRI One link 4x DL 15 bits", // test name
      AIF2FL_LINK_PROTOCOL_CPRI,
   // link0          link1          link2          link3          link4          link5
     {1,             0,             0,             0,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	 {AIF2FL_DATA_WIDTH_15_BIT, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	 {AIF2FL_DATA_WIDTH_15_BIT, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
     {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_1, AIF2FL_DIO_ENGINE_2, AIF2FL_DIO_ENGINE_2, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
     {1,             0,             0},
    },
	{//10 Test - CPRI 4x for DL WCDMA - Link 0
	  "CPRI one link 4x DL", // test name
	  AIF2FL_LINK_PROTOCOL_CPRI,
   // link0          link1          link2          link3          link4          link5
	 {1,             0,             0,             0,             0,             0            },  // link enable
	 {4,             4,             4,             4,             4,             4            },  // link rate
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	 {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	 {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
	 {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
	 {1,             0,             0},
	}
   };

// Users should use 16 bytes aligned(Quad word) data for Aif2 and PktDMA data flow
// DIO programming for WDMA UL:
//       wrap 1 = 2 Qwords * NUM_AxC = 32 bytes * NUM_AxC
//       wrap 2 = NumBlocks * wrap1 = 4 * 32 bytes * NUM_AxC
// So NumBlocks = 4, hence dio_result = 32 words * NUM_AxC
#define NUM_AxC_MAX (2*16)                                  //for this test purpose, we use maximum 2 links per DIO engine. If you want you can use up to 4 on 1 dio so 4*15
#pragma DATA_ALIGN (dio_data, 16)
uint32_t  dio_data[AIF2_MAX_NUM_DIO_ENGINE][NUM_AxC_MAX * 32];
#pragma DATA_ALIGN (dio_result, 16)
uint32_t  dio_result[AIF2_MAX_NUM_DIO_ENGINE][NUM_AxC_MAX * 32];                //NUM_AxC channels * 1 (RAC works at 4-chip pace) * 16 (bytes or 1QW every 4 chips) * 8 blocks

// For OBSAI, test data trace
#pragma DATA_SECTION(trace_data,".aif2tracedata")
#pragma DATA_ALIGN (trace_data, 16)
uint32_t  trace_data[16 * 2000];
#pragma DATA_SECTION(trace_framingdata,".aif2tracedata")
#pragma DATA_ALIGN (trace_framingdata, 16)
uint32_t  trace_framingdata[4 * 2000];


#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
#define FRAME_NUM_CHECK  10
#else
#define FRAME_NUM_CHECK  100
#endif

int main(void)
{
	uint32_t  numAxC[AIF2_MAX_NUM_DIO_ENGINE];
    uint32_t  idx, idx2, axc =0, i, numQw, j;
    uint16_t  testpass = 0;
    uint16_t  myboard	= EVM_TYPE; //lyrtech board
    uint32_t	dataPtr, firstActiveLink;


    printf("Beginning AIF2 WCDMA test:\n");
    UTILS_waitForHw(100000);
    
    if (myboard == 0)  UTILS_configMasterSlave();
    else DSP_procId = EVM_TYPE ; //in case advantech it could be board 1 or 2

#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    DSP_procId = 1;
#endif
    
	// Take AIF out of power saver
	AIF_enable();

    for(ntest=0;ntest<TEST_NUM;ntest++)
    {
    	UTILS_waitForHw(100000);
    	firstActiveLink = 0;
 	    if (testEnable[ntest] == 1 )
 	    {
			// General parameters
			memset(&aifObj, 0, sizeof(aifObj));
			aifObj.aif2ClkSpeedKhz   = (uint32_t)SYSCLK_INPUT_KHZ;
			aifObj.protocol           = testObjTab[ntest].protocol;
			aifObj.pktdmaOrDioEngine  = AIF2FL_DIO;
			aifObj.mode               = AIF_WCDMA_MODE;
			if (swSync == 0)
				aifObj.aif2TimerSyncSource= AIF2FL_CHIP_INPUT_SYNC;
			else
				aifObj.aif2TimerSyncSource= AIF2FL_SW_SYNC;

			for (i=0;i<AIF2_MAX_NUM_DIO_ENGINE;i++)
			{
				// Dio parameters for engine 0
				aifObj.dioConfig[i].out          = UTILS_local2GlobalAddr(&dio_data[i][0]);
				aifObj.dioConfig[i].in           = UTILS_local2GlobalAddr(&dio_result[i][0]);

				if (testObjTab[ntest].inboundDataType[0] == AIF2FL_LINK_DATA_TYPE_RSA){
					aifObj.dioConfig[i].inNumBlock	 = 4;
					aifObj.dioConfig[i].outNumBlock  = 4;
					numQw = 2;
				} else {
					aifObj.dioConfig[i].inNumBlock	 = 8;
					aifObj.dioConfig[i].outNumBlock  = 8;
					numQw = 1;
				}
				if (testObjTab[ntest].protocol == AIF2FL_LINK_PROTOCOL_CPRI)
					numAxC[i] = (testObjTab[ntest].nbLinkDio[i] * 15);  //15 for CPRI, 16 for WCDMA.
				else
					numAxC[i] = (testObjTab[ntest].nbLinkDio[i] * 16);  //15 for CPRI, 16 for WCDMA.
			}

			// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
			UTILS_doCleanup(&aifObj,TRIG_TIMER);

        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{
				aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
				aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[ntest].linkRate[i];
				aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_3P84MHZ;
				aifObj.linkConfig[i].outboundDataType   = testObjTab[ntest].outboundDataType[i];
				aifObj.linkConfig[i].outboundDataWidth  = testObjTab[ntest].outboundDataWidth[i];
				aifObj.linkConfig[i].inboundDataType    = testObjTab[ntest].inboundDataType[i];
				aifObj.linkConfig[i].inboundDataWidth   = testObjTab[ntest].inboundDataWidth[i];
				aifObj.linkConfig[i].psMsgEnable        = 0;
				if (intLoopback == 1 ) aifObj.linkConfig[i].comType            = AIF2_LOOPBACK;
				else	 aifObj.linkConfig[i].comType            = AIF2_2_AIF2;
				aifObj.linkConfig[i].dioEngine          = testObjTab[ntest].dioEngine[i];
				if (ntest == 9)
				{
					aifObj.linkConfig[i].firstPdAxC 	= 2;
					aifObj.linkConfig[i].numPdAxC 		= 2;
				}
            }

        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{
        		if (aifObj.linkConfig[i].linkEnable) {
        			firstActiveLink = i;
        			break;
        		}
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
			UTILS_initQmss((uint32_t*)NULL, 0, 0, 0, NULL);
			// initialization function for the AIF2 H/W of the TMS320C6670

			AIF_initDio(&aifObj);
			AIF_initPktDma(&aifObj);
			AIF_initHw(&aifObj);


			memset(dio_result, 0xFF, sizeof(dio_result));
			memset(dio_data, 0x00, sizeof(dio_data));
			for (i=0;i<AIF2_MAX_NUM_DIO_ENGINE;i++)
			{
				//bit 31 ~ 24 : Symbol number bit 23~ 16 : AXC number bit15 ~ 0 :sample count
				for (idx = 0; idx < aifObj.dioConfig[i].inNumBlock; idx++) {
					for (j=0; j<numAxC[i]; j++)
					{
						for (idx2 = 0; idx2 < 4 * numQw; idx2++)
						{
							dio_data[i][(4 * numQw * numAxC[i] * idx) + (4 * numQw * axc) + idx2] = idx2;
							dio_data[i][(4 * numQw * numAxC[i] * idx) + (4 * numQw * axc) + idx2] |= axc << 12;
							dio_data[i][(4 * numQw * numAxC[i] * idx) + (4 * numQw * axc) + idx2] |= idx << 20;
							dio_data[i][(4 * numQw * numAxC[i] * idx) + (4 * numQw * axc) + idx2] |= i << 28;
						}
						axc++;
					}
					axc = 0;
				}
			}

			// start AIF2 HW now that the DIO out buffer is pre-filled
			AIF_startHw(&aifObj);

			// let DSP2 go behond this point and start timer 0 on DSP1 for external Fsync
			UTILS_initTimer(TRIG_TIMER);
			if (DSP_procId == 1){
				UTILS_triggerFsync(&aifObj);
			}

			while (aifFsyncEventCount[1] <= 1);
			AIF_enableException(&aifObj);

			if (aifObj.protocol == AIF2FL_LINK_PROTOCOL_OBSAI)
			{
				aif2TraceObj.linkIndex        			= (Aif2Fl_LinkIndex)firstActiveLink;
				aif2TraceObj.dataCaptureEnable			= 1;
				aif2TraceObj.framingDataCaptureEnable	= 1;
				aif2TraceObj.syncCaptureEnable			= 1;
				aif2TraceObj.dataAddress				= &trace_data[0];
				aif2TraceObj.framingDataAddress			= &trace_framingdata[0];
				aif2TraceObj.dtWrap 					= 2000 - 1;
				memset(trace_data, 0xFF, sizeof(trace_data));
				memset(trace_framingdata, 0xFF, sizeof(trace_framingdata));
				AIF_enableDataTrace(&aifObj,&aif2TraceObj);
			}

			// wait for few Fsync before comparing data
			while (aifFsyncEventCount[1] <= FRAME_NUM_CHECK) {
				asm("    IDLE");
			}

			if (aifObj.protocol == AIF2FL_LINK_PROTOCOL_OBSAI)
			{
				AIF_disableDataTrace(&aifObj);
				if ((trace_data[0] == 0xFFFFFFFF)||(trace_framingdata[0] == 0xFFFFFFFF)) {
					testpass = 1;
				}

			}

			if (ntest != 9)
			{
				for (i=0;i<AIF2_MAX_NUM_DIO_ENGINE;i++)
				{
				/* Compare the DIO loopback data */
					if (numAxC[i] != 0)
						testpass |= memcmp(&dio_data[i][0], &dio_result[i][0], sizeof (dio_result[i][0]));
				}
			} else {
				axc = 0;
				for (i=0;i<AIF2_MAX_NUM_DIO_ENGINE;i++)
				{
					if (numAxC[i] != 0)
					{
						//bit 31 ~ 24 : Symbol number bit 23~ 16 : AXC number bit15 ~ 0 :sample count
						for (idx = 0; idx < aifObj.dioConfig[i].inNumBlock; idx++) {
							for (j=0; j<aifObj.linkConfig[0].numPdAxC; j++)
							{
								for (idx2 = 0; idx2 < 4 * numQw; idx2++)
								{
									dataPtr = idx2;
									dataPtr |= (axc + aifObj.linkConfig[0].firstPdAxC) << 12;
									dataPtr |= idx << 20;
									dataPtr |= i << 28;
									if (dataPtr != dio_result[i][(4 * numQw * aifObj.linkConfig[0].numPdAxC * idx) + (4 * numQw * axc) + idx2])
										testpass = 1;
								}
								axc++;
							}
							axc = 0;
						}
					}
				}
			}

			UTILS_doCleanup(&aifObj,TRIG_TIMER);

			if (testpass != 0) {
				printf(" Test %s on dsp %d: FAIL\n", testObjTab[ntest].name, DSP_procId);
				AIF_printException(&aifObj);
				testpass = 0; testcheck++;
			}
			else { if (DSP_procId == 1) printf("\n Test %s on dsp %d: SUCCESS\n \n", testObjTab[ntest].name, DSP_procId); }
		}
//#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
#if (defined(DEVICE_K2K) || defined(DEVICE_K2H))
 	   //if (ntest==2) ntest = 10;
#endif
	}

    if (testcheck == 1) {
       testcheck = 0;
       printf("All tests have passed\n");
    } else {
       printf("Some tests have failed\n");
    }

	return (0);
    
}
////////////

