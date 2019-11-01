/**
 *   @file  resmgr_cfg.c
 *
 *   @brief
 *      Resource Manager Configuration for entire system.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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
 
#include <ti/drv/aif2/aif2.h>

#define DATA_TYPE_DL    AIF2FL_LINK_DATA_TYPE_NORMAL
#define DATA_TYPE_UL    AIF2FL_LINK_DATA_TYPE_RSA
#define DATA_WIDTH_16   AIF2FL_DATA_WIDTH_16_BIT
#define DATA_WIDTH_15   AIF2FL_DATA_WIDTH_15_BIT
#define DATA_WIDTH_8    AIF2FL_DATA_WIDTH_8_BIT
#define DATA_WIDTH_7    AIF2FL_DATA_WIDTH_7_BIT
#define DIO_0           AIF2FL_DIO_ENGINE_0


/* All configuration requests are placed in the configuration memory section. */

/** 
 * @brief 
 *  Sample System Configuration requested.
 *
 * @details
 *  This is a sample platform configuration request for all 4 cores.
 */
AIF_CfgObj    gAifCfgLocalObj =
{

	  "LTE_20MHZ_RF_SINGLE_TONE", // test name
	  AIF2FL_LINK_PROTOCOL_CPRI,
	  AIF_LTE_FDD_MODE,
	  AIF2FL_CPPI,
	  {
			  /* CORE 0 (TI initialization): Requested Configuration. */
		  {
			  AIF_LTE_FDD_MODE,
			 // link0          link1          link2          link3          link4          link5
			  {0,             0,             0,             1,             0,             0            },  // link enable
			  {4,             4,             4,             4,             4,             4            },  // link rate
			  {0,             0,             0,             1,             0,             0            },  // num AxC per core
			  {0,             0,             0,             0,             0,             0            },  // first AxC Index
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
			  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
			  {LTE,           LTE,           LTE,           LTE,           LTE,           LTE},            // Only for Dual Mode
			  0,																						   // CW enable
			  310,																						   // peAxCOffset
			  0,																						   // pdAxCOffset
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // egressBufDepth
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // ingressBufDepth
		  },
		  	  /* CORE 1 (TI initialization): Requested Configuration. */
		  {
			  AIF_LTE_FDD_MODE,
			 // link0          link1          link2          link3          link4          link5
			  {0,             0,             0,             1,             0,             0            },  // link enable
			  {4,             4,             4,             4,             4,             4            },  // link rate
			  {0,             0,             0,             1,             0,             0            },  // num AxC per core
			  {0,             0,             0,             1,             0,             0            },  // first AxC Index
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
			  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
			  {LTE,           LTE,           LTE,           LTE,           LTE,           LTE},            // Only for Dual Mode
			  0,																						   // CW enable
			  310,																						   // peAxCOffset
			  0,																						   // pdAxCOffset
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // egressBufDepth
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // ingressBufDepth
		  },
		  	  /* CORE 2 (TI initialization): Requested Configuration. */
		  {
			  AIF_LTE_FDD_MODE,
			 // link0          link1          link2          link3          link4          link5
			  {0,             0,             0,             1,             0,             0            },  // link enable
			  {4,             4,             4,             4,             4,             4            },  // link rate
			  {0,             0,             0,             1,             0,             0            },  // num AxC per core
			  {0,             0,             0,             2,             0,             0            },  // first AxC Index
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
			  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
			  {LTE,           LTE,           LTE,           LTE,           LTE,           LTE},            // Only for Dual Mode
			  0,																						   // CW enable
			  310,																						   // peAxCOffset
			  0,																						   // pdAxCOffset
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // egressBufDepth
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // ingressBufDepth
		  },
		  	  /* CORE 3 (TI initialization): Requested Configuration. */
		  {
			  AIF_LTE_FDD_MODE,
			 // link0          link1          link2          link3          link4          link5
			  {0,             0,             0,             1,             0,             0            },  // link enable
			  {4,             4,             4,             4,             4,             4            },  // link rate
			  {0,             0,             0,             1,             0,             0            },  // num AxC per core
			  {0,             0,             0,             3,             0,             0            },  // first AxC Index
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
			  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
			  {LTE,           LTE,           LTE,           LTE,           LTE,           LTE},            // Only for Dual Mode
			  0,																						   // CW enable
			  310,																						   // peAxCOffset
			  0,																						   // pdAxCOffset
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // egressBufDepth
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // ingressBufDepth
		  }
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
		  ,
	  	  /* CORE 4 (TI initialization): Requested Configuration. */
		  {
			  AIF_LTE_FDD_MODE,
			 // link0          link1          link2          link3          link4          link5
			  {0,             0,             0,             1,             0,             0            },  // link enable
			  {4,             4,             4,             4,             4,             4            },  // link rate
			  {0,             0,             0,             1,             0,             0            },  // num AxC per core
			  {0,             0,             0,             4,             0,             0            },  // first AxC Index
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
			  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
			  {LTE,           LTE,           LTE,           LTE,           LTE,           LTE},            // Only for Dual Mode
			  0,																						   // CW enable
			  310,																						   // peAxCOffset
			  0,																						   // pdAxCOffset
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // egressBufDepth
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // ingressBufDepth
		  },
	  	  /* CORE 5 (TI initialization): Requested Configuration. */
		  {
			  AIF_LTE_FDD_MODE,
			 // link0          link1          link2          link3          link4          link5
			  {0,             0,             0,             1,             0,             0            },  // link enable
			  {4,             4,             4,             4,             4,             4            },  // link rate
			  {0,             0,             0,             1,             0,             0            },  // num AxC per core
			  {0,             0,             0,             5,             0,             0            },  // first AxC Index
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
			  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
			  {LTE,           LTE,           LTE,           LTE,           LTE,           LTE},            // Only for Dual Mode
			  0,																						   // CW enable
			  310,																						   // peAxCOffset
			  0,																						   // pdAxCOffset
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // egressBufDepth
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // ingressBufDepth
		  },
	  	  /* CORE 6 (TI initialization): Requested Configuration. */
		  {
			  AIF_LTE_FDD_MODE,
			 // link0          link1          link2          link3          link4          link5
			  {0,             0,             0,             1,             0,             0            },  // link enable
			  {4,             4,             4,             4,             4,             4            },  // link rate
			  {0,             0,             0,             1,             0,             0            },  // num AxC per core
			  {0,             0,             0,             6,             0,             0            },  // first AxC Index
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
			  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
			  {LTE,           LTE,           LTE,           LTE,           LTE,           LTE},            // Only for Dual Mode
			  0,																						   // CW enable
			  310,																						   // peAxCOffset
			  0,																						   // pdAxCOffset
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // egressBufDepth
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // ingressBufDepth
		  },
	  	  /* CORE 7 (TI initialization): Requested Configuration. */
		  {
			  AIF_LTE_FDD_MODE,
			 // link0          link1          link2          link3          link4          link5
			  {0,             0,             0,             1,             0,             0            },  // link enable
			  {4,             4,             4,             4,             4,             4            },  // link rate
			  {0,             0,             0,             1,             0,             0            },  // num AxC per core
			  {0,             0,             0,             7,             0,             0            },  // first AxC Index
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
			  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
			  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
			  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
			  {LTE,           LTE,           LTE,           LTE,           LTE,           LTE},            // Only for Dual Mode
			  0,																						   // CW enable
			  310,																						   // peAxCOffset
			  0,																						   // pdAxCOffset
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // egressBufDepth
			  AIF2FL_DB_FIFO_DEPTH_QW8,																   // ingressBufDepth
		  }
#endif
	  }
};

/* Resource Manager configuration. Initialized on SoC Init core (Core0) with the previous local object. Used here to check
 * resource allocation status for this core */
#pragma DATA_SECTION(gAifCfgObj, ".cfgAifSection");
AIF_CfgObj   gAifCfgObj;

/////////////////////////////////

