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

/** @defgroup IQN2_API IQN2
 *
 * @section Introduction
 *
 * @subsection xxx Purpose and Scope
 * The IQN2 module is dedicated to the definition of generic elements for
 *  the C66x KeyStone IQN2 module and dedicated to the H/W initialization of the
 *  IQN2 module given the IQN2 object configuration parameters from the application user.
 *
 *
 *
 */

/** @file IQN2_defs.h
 *
 *  @brief Header file for driver of IQN2
 *
 *  Description
 *  - The different symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */

/**
 * @defgroup IQN2_DATASTRUCT IQN2 Data Structures
 * @ingroup IQN2_API
 */
/**
 * @defgroup IQN2_ENUM  IQN2 Enumerated Data Types
 * @ingroup IQN2_API
 */
/**
 * @defgroup IQN2_FUNCTION  IQN2 Functions
 * @ingroup IQN2_API
 */


#ifndef __IQN2_DEFS_H
#define __IQN2_DEFS_H

#ifdef _TMS320C6X
#include <c6x.h>
#endif

#include <ti/csl/csl.h>
#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2_types.h>


#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************\
* IQN2 global macro declarations
\**************************************************************************/

/** @addtogroup IQN2_ENUM
*
* @{ */

/** Maximum number of IQN2 AIL */
#define IQN2_MAX_NUM_AIL 						IQN2FL_AIL_MAX
/** Maximum number of IQN2 AxC */
#define IQN2_MAX_NUM_AXC                        48
/** Maximum number of IQN2 RADTs */
#define IQN2_MAX_NUM_RADT                       8
/** Maximum number of IQN2 Radio standards in each direction */
#define IQN2_MAX_NUM_RADIO_STANDARD             8
/** Maximum number of IQN2 AT2 events */
#define IQN2_MAX_NUM_AT2_EVENT                  24
/** Max number of Dio engine */
#define IQN2_MAX_NUM_DIO_ENGINE                 3
/** Max number of CPRI control word sub-streams*/
#define IQN2_CPRI_MAX_CW_SUBSTREAM              4
/** Frame terminal count for Wcdma Fdd */
#define IQN2_FRAME_COUNT_TC_WCDMA_FDD   		4095
/** Slot terminal count for Wcdma Fdd */
#define IQN2_SLOT_COUNT_TC_WCDMA_FDD    	    14
 /** Frame terminal count for BCN timer */
#define IQN2_FRAME_COUNT_TC_PHY_TIMER  		    4095
/** Number of clock counts in a radio frame for Obsai minus 1 */
#define IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER     3071999
/** Number of clock counts in a radio frame for Cpri minus 1 */
#define IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER	    2457599
/** Number of clock counts in a radio frame for DFE@245.76MHz minus 1 */
#define IQN2_DFE_245_76_CLOCK_COUNT_TC_PHY_TIMER  IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER
/** Number of clock counts in a radio frame for DFE@368.64MHz minus */
#define IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER  3686399
/** Byte clock count TC for CPRI/DFE245.76 and 1ms time minus 1 */
#define IQN2_CLOCK_COUNT_TC_CPRI                245759
/** Byte clock count TC for OBSAI and 1ms time minus 1 */
#define IQN2_CLOCK_COUNT_TC_OBSAI               307199
/** Byte clock count TC for DFE368.64 and 1ms time minus 1 */
#define IQN2_CLOCK_COUNT_TC_368_64              368639
/** LTE parameters: number of normal cyclic prefix symbols per slot */
#define IQN2_LTE_SYMBOL_NUM					    7
/** LTE parameters: number of normal cyclic prefix symbols per radio frame */
#define IQN2_LTE_FRAME_SYMBOL_NUM			    140
/** LTE parameters: number of extended cyclic prefix symbols per slot */
#define IQN2_LTE_SYMBOL_NUM_EXT_CP              6
/** LTE parameters: number of extended cyclic prefix symbols per radio frame */
#define IQN2_LTE_FRAME_SYMBOL_NUM_EXT_CP        120
/** LTE parameters: LTE 20 framing value */
#define IQN2_LTE20_SAMPLE_RATE					30720
/** LTE parameters: number of samples per symbol at 184.32MHz sample rate */
#define IQN2_LTE120_FFT_SIZE                    12288
/** LTE parameters: number of samples per symbol at 122.88MHz sample rate */
#define IQN2_LTE80_FFT_SIZE                     8192
/** LTE parameters: number of samples per symbol at 92.16MHz sample rate */
#define IQN2_LTE60_FFT_SIZE					    6144
/** LTE parameters: number of samples per symbol at 61.44MHz sample rate */
#define IQN2_LTE40_FFT_SIZE					    4096
/** LTE parameters: number of samples per symbol at 30.72MHz sample rate */
#define IQN2_LTE20_FFT_SIZE					    2048
/** LTE parameters: number of samples per symbol at 23.04MHz sample rate */
#define IQN2_LTE15_FFT_SIZE					    1536
/** LTE parameters: number of samples per symbol at 15.36MHz sample rate */
#define IQN2_LTE10_FFT_SIZE					    1024
/** LTE parameters: number of samples per symbol at 7.68MHz sample rate */
#define IQN2_LTE5_FFT_SIZE					    512
/** LTE parameters: number of samples per symbol at 3.84MHz sample rate */
#define IQN2_LTE3_FFT_SIZE					    256
/** LTE parameters: number of samples per symbol at 1.92MHz sample rate */
#define IQN2_LTE1P4_FFT_SIZE				    128
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 184.32MHz sample rate */
#define IQN2_LTE120_CYPRENORMAL1_SIZE           960
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 184.32MHz sample rate */
#define IQN2_LTE120_CYPRENORMAL_SIZE            864
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 184.32MHz sample rate */
#define IQN2_LTE120_CYPREEXTENDED_SIZE          3072
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 122.88MHz sample rate */
#define IQN2_LTE80_CYPRENORMAL1_SIZE            640
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 122.88MHz sample rate */
#define IQN2_LTE80_CYPRENORMAL_SIZE             576
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 122.88MHz sample rate */
#define IQN2_LTE80_CYPREEXTENDED_SIZE           2048
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 92.16MHz sample rate */
#define IQN2_LTE60_CYPRENORMAL1_SIZE		    480
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 92.16MHz sample rate */
#define IQN2_LTE60_CYPRENORMAL_SIZE			    432
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 92.16MHz sample rate */
#define IQN2_LTE60_CYPREEXTENDED_SIZE           1536
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 61.44MHz sample rate */
#define IQN2_LTE40_CYPRENORMAL1_SIZE		    320
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 61.44MHz sample rate */
#define IQN2_LTE40_CYPRENORMAL_SIZE			    288
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 61.44MHz sample rate */
#define IQN2_LTE40_CYPREEXTENDED_SIZE           1024
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 30.72MHz sample rate */
#define IQN2_LTE20_CYPRENORMAL1_SIZE		    160
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 30.72MHz sample rate */
#define IQN2_LTE20_CYPRENORMAL_SIZE			    144
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 30.72MHz sample rate */
#define IQN2_LTE20_CYPREEXTENDED_SIZE           512
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 23.04MHz sample rate */
#define IQN2_LTE15_CYPRENORMAL1_SIZE		    120
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 23.04MHz sample rate */
#define IQN2_LTE15_CYPRENORMAL_SIZE			    108
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 23.04MHz sample rate */
#define IQN2_LTE15_CYPREEXTENDED_SIZE           384
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 15.36MHz sample rate */
#define IQN2_LTE10_CYPRENORMAL1_SIZE		    80
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 15.36MHz sample rate */
#define IQN2_LTE10_CYPRENORMAL_SIZE			    72
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 15.36MHz sample rate */
#define IQN2_LTE10_CYPREEXTENDED_SIZE           256
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 7.68MHz sample rate */
#define IQN2_LTE5_CYPRENORMAL1_SIZE			    40
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 7.68MHz sample rate */
#define IQN2_LTE5_CYPRENORMAL_SIZE			    36
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 7.68MHz sample rate */
#define IQN2_LTE5_CYPREEXTENDED_SIZE            128
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 3.84MHz sample rate */
#define IQN2_LTE3_CYPRENORMAL1_SIZE			    20
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 3.84MHz sample rate */
#define IQN2_LTE3_CYPRENORMAL_SIZE			    18
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 3.84MHz sample rate */
#define IQN2_LTE3_CYPREEXTENDED_SIZE            64
/** LTE parameters: number of samples per normal cyclic prefix for first symbols in a slot at 1.92MHz sample rate */
#define IQN2_LTE1P4_CYPRENORMAL1_SIZE		    10
/** LTE parameters: number of samples per normal cyclic prefix for other than first symbols in a slot at 1.92MHz sample rate */
#define IQN2_LTE1P4_CYPRENORMAL_SIZE		    9
/** LTE parameters: number of samples per extended cyclic prefix for all symbols in a slot at 1.92MHz sample rate */
#define IQN2_LTE1P4_CYPREEXTENDED_SIZE          32
/** IQN2 UMTS chip speed in KHz */
#define IQN2_UMTS_CHIP_SPEED_3840K              3840
/** IQN2 UMTS sample speed in KHz */
#define IQN2_UMTS_SAMPLE_SPEED_KHZ              (IQN2_UMTS_CHIP_SPEED_3840K*8)
/** IQN2 AIL RM configuration: Controls the counting of LCV Errors in the RM */
#define IQN2_AIL_RM_LOS_DET_THOLD               0//16
/** IQN2 AIL RM configuration: Threshold value for consecutive valid blocks of bytes which result in state ST1 */
#define IQN2_AIL_RM_SYNC_THOLD                  2//75
/** IQN2 AIL RM configuration: Threshold value for consecutive valid message groups which result in state ST3 */
#define IQN2_AIL_RM_FRAME_SYNC_THOLD            2//75
/** IQN2 AIL RM configuration: Threshold value for consecutive invalid blocks of bytes which result in state ST0 */
#define IQN2_AIL_RM_UNSYNC_THOLD                2//5
/** IQN2 AIL RM configuration: Threshold value for consecutive invalid message groups which result in state ST1 */
#define IQN2_AIL_RM_FRAME_UNSYNC_THOLD          2//5
/** IQN2 number of 32-bit words per quad-word */
#define  IQN2_NUM_WORDS_PER_QWORD	            4
/** IQN2 AIL timing delay between protocol encoder and tx mac for CPRI */
//#define IQN2_CPRI_PE_TO_TM_DELAY 	28		//FIXME
#define IQN2_CPRI_PE_TO_TM_DELAY 	            30	//sys_clk
/** IQN2 AIL timing delay between protocol encoder and tx mac for OBSAI */
//#define IQN2_OBSAI_PE_TO_TM_DELAY 	22 	//FIXME
#define IQN2_OBSAI_PE_TO_TM_DELAY 	            58 //sys_clk
/** IQN2 AIL PE_STB (Pe2Offset) for WCDMA OBSAI DL */
#define IQN2_WCDMA_OBSAI_DL_Pe2OffSet		    610  //  TAC requires about 240 byte clock for processing and DIO DMA requires 256 ~ 300 byte clock to transfer 4 chip data
/** IQN2 AIL PE_STB (Pe2Offset) for WCDMA OBSAI UL */
#define IQN2_WCDMA_OBSAI_UL_Pe2OffSet		    610  //  TAC requires about 240 byte clock for processing and DIO DMA requires 256 ~ 300 byte clock to transfer 4 chip data
/** IQN2 AIL PE_STB (Pe2Offset) for WCDMA CPRI UL */
#define IQN2_WCDMA_CPRI_UL_Pe2OffSet		    490
/** IQN2 AIL PE_STB (Pe2Offset) for WCDMA CPRI DL */
#define IQN2_WCDMA_CPRI_DL_Pe2OffSet		    490
/** IQN2 AIL PE_STB (Pe2Offset) for LTE OBSAI UL */
#define IQN2_LTE_OBSAI_UL_Pe2OffSet		        310
/** IQN2 AIL PE_STB (Pe2Offset) for LTE OBSAI DL */
#define IQN2_LTE_OBSAI_DL_Pe2OffSet		        310
/** IQN2 AIL PE_STB (Pe2Offset) for LTE CPRI UL */
#define IQN2_LTE_CPRI_UL_Pe2OffSet		        310
/** IQN2 AIL PE_STB (Pe2Offset) for LTE CPRI DL */
#define IQN2_LTE_CPRI_DL_Pe2OffSet		        310
/** IQN2 AIL PE_STB (Pe2Offset) for LTE-WCDMA OBSAI DL */
#define IQN2_LTE_WCDMA_OBSAI_DL_Pe2OffSet       310
/** IQN2 AIL PE_STB (Pe2Offset) for LTE-WCDMA OBSAI UL */
#define IQN2_LTE_WCDMA_OBSAI_UL_Pe2OffSet       310
/** IQN2 SerDes processing delay (Tx mac, line and Rx mac) */
//#define IQN2_SERDES_PROCESSING_DELAY 10  //FIXME
#define IQN2_SERDES_PROCESSING_DELAY            12 //in loopback
/** IQN2 redirection delay between Rx and Tx mac for CPRI */
#define IQN2_CPRI_RM_TO_TM_DELAY 	            60
/** IQN2 redirection delay between Rx and Tx mac for OBSAI */
#define IQN2_OBSAI_RM_TO_TM_DELAY 	            38

/**
* @brief This enum describes the IQN2 Sampling rate per link
*/
typedef enum {
			/**  Sampling rate 1.92MHz*/
			IQN2_SRATE_1P92MHZ = 0, \
			/**  Sampling rate 3.84MHz*/
			IQN2_SRATE_3P84MHZ , \
			/**  Sampling rate 7.68MHz*/
			IQN2_SRATE_7P68MHZ , \
			/**  Sampling rate 15.36MHz*/
			IQN2_SRATE_15P36MHZ , \
			/**  Sampling rate 23.04MHz*/
			IQN2_SRATE_23P04MHZ , \
			/**  Sampling rate 30.72MHz*/
			IQN2_SRATE_30P72MHZ , \
			/**  Sampling rate 61.44MHz*/
			IQN2_SRATE_61P44MHZ , \
			/**  Sampling rate 61.44MHz*/
			IQN2_SRATE_92P16MHZ , \
			/**  Sampling rate 122.88MHz*/
			IQN2_SRATE_122P88MHZ , \
			/**  Sampling rate 184.32MHz*/
			IQN2_SRATE_184P32MHZ , \
			/**  Sampling rate custom*/
			IQN2_SRATE_CUSTOM , \
			IQN2_SRATE_MAX
        }IQN2_SampleRate;

/**
 * @brief This enum describes the Cyclic Prefix type for LTE
*/
typedef enum {
            IQN2_LTE_CPTYPE_NORMAL = 0,
            IQN2_LTE_CPTYPE_EXTENDED = 1,
            IQN2_LTE_CPTYPE_NONE = 2
        } IQN2_LteCpType;
			
/**
 *  @brief This enum describes the IQN2 SerDes communication mode
 */
typedef enum {  
            /** Communication with an external device */
            IQN2_COM_NO_LOOPBACK = 0, \
            /** Communication using K-II SerDes loopback */
            IQN2_COM_SD_LOOPBACK = 1, \
            /** number of IQN2 communication type */
            IQN2_COM_MAX
        } IQN2_ComMode;
			
/**
 * @brief This enum describes the IQN2 pack mode for AIL CPRI in order to support LTE
 * CPRI flexible antenna carrier packing.
 * As examples:
 * -  LTE FDD 20 MHz test case, IQN2_LTE_CPRI_8b8 format looks like:{Control, AxC0, AxC0, AxC0,
 * AxC0, AxC0, AxC0, AxC0, AxC0, AxC1, AxC1, AxC1, AxC1, AxC1, AxC1, AxC1, AxC1}
 * -  LTE FDD 20 MHz test case, IQN2_LTE_CPRI_1b1 format looks like: {Control, AxC0, AxC1,
 * AxC0, AxC1, AxC0, AxC1, AxC0, AxC1, AxC0, AxC1, AxC0, AxC1, AxC0, AxC1, AxC0, AxC1}
*/
typedef enum {
                /** Pack mode for Lte 1.4MHz bandwidth */
                IQN2_LTE_CPRI_0b1 = 0,
                /** Pack mode for all Lte bandwidths */
                IQN2_LTE_CPRI_1b1 = 1,
                /** Pack mode for Lte 5MHz bandwidth  */
                IQN2_LTE_CPRI_2b2 = 2,
                /** Pack mode for Lte 10MHz bandwidth  */
                IQN2_LTE_CPRI_4b4 = 4,
                /** Pack mode for Lte 15MHz bandwidth */
                IQN2_LTE_CPRI_6b6 = 6,
                /** Pack mode for Lte 20MHz bandwidth  */
                IQN2_LTE_CPRI_8b8 = 8
            } IQN2_CpriPackingMode;


/**
 * @brief This enum describes the IQN2 synchronization source to detect in order for the
 * software application to perform timing adjustment on BCN offset register.
 * For development purposes, software sync aka diag sync can be setup.
*/
typedef enum {
                /** Diagnostic sync input from Software (Valid when no external sync is required)  */
                IQN2_DIAG_SW_SYNC = 0,
                /** RP1 FCB packet (Only valid with using OBSAI RP1 Sync Interface) */
                IQN2_RP1_SYNC = 1,
                /** Rad sync input pin (Only valid when using external sync pin) */
                IQN2_RAD_SYNC = 2,
                /** Phy sync input pin (Only valid when using external sync pin)  */
                IQN2_PHY_SYNC = 3,
                /** Pa_tscomp sync input from NETCP PA (Valid when using IEEE1588 sync)  */
                IQN2_PA_TSCOMP_SYNC = 4
            } IQN2_SyncSource;

/**
 * @brief This enum describes the IQN2 radio timer application mode, egress, ingress or unused.
*/
typedef enum {
                /** Unused radio timer  */
                IQN2_RADT_UNUSED_MODE = 0,
                /** Radio timer used for an egress application (for instance downlink)  */
                IQN2_RADT_EGR_MODE = 1,
                /** Radio timer used for an ingress application (for instance uplink)  */
                IQN2_RADT_ING_MODE = 2
            } IQN2_RadTimerMode;

/**
 * @brief This enum is used to select an AT2 Event Number [0:23]
*/
typedef enum
{
                /** Selects AT2 event 0 */
               IQN2_AT2_EVENT_0 = 0,
               /** Selects AT2 event 1 */
               IQN2_AT2_EVENT_1,
               /** Selects AT2 event 2 */
               IQN2_AT2_EVENT_2,
               /** Selects AT2 event 3 */
               IQN2_AT2_EVENT_3,
               /** Selects AT2 event 4 */
               IQN2_AT2_EVENT_4,
               /** Selects AT2 event 5 */
               IQN2_AT2_EVENT_5,
               /** Selects AT2 event 6 */
               IQN2_AT2_EVENT_6,
               /** Selects AT2 event 7 */
               IQN2_AT2_EVENT_7,
               /** Selects AT2 event 8, dedicated to TAC */
               IQN2_AT2_EVENT_8,
               /** Selects AT2 event 9, dedicated to RAC_A */
               IQN2_AT2_EVENT_9,
               /** Selects AT2 event 10 */
               IQN2_AT2_EVENT_10,
               /** Selects AT2 event 11 */
               IQN2_AT2_EVENT_11,
               /** Selects AT2 event 12 */
               IQN2_AT2_EVENT_12,
               /** Selects AT2 event 13 */
               IQN2_AT2_EVENT_13,
               /** Selects AT2 event 14 */
               IQN2_AT2_EVENT_14,
               /** Selects AT2 event 15 */
               IQN2_AT2_EVENT_15,
               /** Selects AT2 event 16 */
               IQN2_AT2_EVENT_16,
               /** Selects AT2 event 17 */
               IQN2_AT2_EVENT_17,
               /** Selects AT2 event 18 */
               IQN2_AT2_EVENT_18,
               /** Selects AT2 event 19 */
               IQN2_AT2_EVENT_19,
               /** Selects AT2  event 20 */
               IQN2_AT2_EVENT_20,
               /** Selects AT2 event 21 */
               IQN2_AT2_EVENT_21,
               /** Selects AT2 event 22 */
               IQN2_AT2_EVENT_22,
               /** Selects AT2 event 23 */
               IQN2_AT2_EVENT_23,
               /** *Number of AT2 events */
               IQN2_AT2_EVENT_MAX
            } IQN2_At2EventIndex;

/**
 * @brief This enum describes the LTE TDD UL-DL configuration.
 * 00b = DL, 11b=UL, 01b=S
 * Words are right justified with SF0 in LSB.
 *  SF	0	 1	 2	 3	 4	 5	 6	 7	 8	 9
 *	0	D	 S	 U	 U	 U	 D	 S	 U	 U	 U , 11 11 11 01 00 11 11 11 01 00, 0xFD3F4
 *	1	D	 S	 U	 U	 D	 D	 S	 U	 U	 D , 00 11 11 01 00 00 11 11 01 00, 0x3D0F4
 *	2	D	 S	 U	 D	 D	 D	 S	 U	 D	 D , 00 00 11 01 00 00 00 11 01 00, 0x0D034
 *	3	D	 S	 U	 U	 U	 D	 D	 D	 D	 D , 00 00 00 00 00 11 11 11 01 00, 0x003F4
 *	4	D	 S	 U	 U	 D	 D	 D	 D	 D	 D , 00 00 00 00 00 00 11 11 01 00, 0x000F4
 *	5	D	 S	 U	 D	 D	 D	 D	 D	 D	 D , 00 00 00 00 00 00 00 11 01 00, 0x00034
 *	6	D	 S	 U	 U	 U	 D	 S	 U	 U	 D , 00 11 11 01 00 11 11 11 01 00, 0x3D3F4
 *  0xF3FCF
*/
typedef enum {
	IQN2_LTETDD_ULDL_CFG0 = 0xFD3F4,  //0xF3FCF,//
	IQN2_LTETDD_ULDL_CFG1 = 0x3D0F4,
	IQN2_LTETDD_ULDL_CFG2 = 0x0D034,
	IQN2_LTETDD_ULDL_CFG3 = 0x003F4,
	IQN2_LTETDD_ULDL_CFG4 = 0x000F4,
	IQN2_LTETDD_ULDL_CFG5 = 0x00034,
	IQN2_LTETDD_ULDL_CFG6 = 0x3D3F4
} IQN2_LteTddUlDlCfg;

/**
 * @brief This enum describes the LTE TDD Special SF configuration for normal cyclic prefix.
 * UL symbol = 11b, Dl symbol = 00b, GP symbol = 01
 * Words are right justified with DwPTS in LSB.
 * SSF	DwPTS	GP	UlPTS
 * 0	3		10	1		, 11 01 01 01 01 01 01 01 01 01 01 00 00 00, 0xD555540
 * 1	9		4	1		, 11 01 01 01 01 00 00 00 00 00 00 00 00 00, 0xD540000
 * 2	10		3	1		, 11 01 01 01 00 00 00 00 00 00 00 00 00 00, 0xD500000
 * 3	11		2	1		, 11 01 01 00 00 00 00 00 00 00 00 00 00 00, 0xD400000
 * 4	12		1	1		, 11 01 00 00 00 00 00 00 00 00 00 00 00 00, 0xD000000
 * 5	3		9	2		, 11 11 01 01 01 01 01 01 01 01 01 00 00 00, 0xF555540
 * 6	9		3	2		, 11 11 01 01 01 00 00 00 00 00 00 00 00 00, 0xF540000
 * 7	10		2	2		, 11 11 01 01 00 00 00 00 00 00 00 00 00 00, 0xF500000
 * 8	11		1	2		, 11 11 01 00 00 00 00 00 00 00 00 00 00 00, 0xF400000
 *
*/
typedef enum {
	IQN2_LTETDD_SSF_NCP_CFG0 = 0xD555540,
	IQN2_LTETDD_SSF_NCP_CFG1 = 0xD540000,
	IQN2_LTETDD_SSF_NCP_CFG2 = 0xD500000,
	IQN2_LTETDD_SSF_NCP_CFG3 = 0xD400000,
	IQN2_LTETDD_SSF_NCP_CFG4 = 0xD000000,
	IQN2_LTETDD_SSF_NCP_CFG5 = 0xF555540,
	IQN2_LTETDD_SSF_NCP_CFG6 = 0xF540000,
	IQN2_LTETDD_SSF_NCP_CFG7 = 0xF500000,
	IQN2_LTETDD_SSF_NCP_CFG8 = 0xF400000
} IQN2_LteTddSsfNcpCfg;

/**
@} */

/** @addtogroup IQN2_DATASTRUCT
*
* @{ */

/**
 * @brief This structure contains specific parameters for a
 * given IQN2 antenna container (AxC).
 */
 typedef struct IQN2_AxCObj {
                    /** Holds the Egress AxC offset value for this AxC */
                    uint32_t                    egressAxCOffset;
                    /** Holds the Ingress AxC offset value for this AxC */
                    uint32_t                    ingressAxCOffset;
                    /** Holds the LTE cyclic prefix type for this AxC */
                    IQN2_LteCpType              lteCpType;
                    /** Used for LTE for group creation to facilitate MBSFN operation. Normally an egress group
                        is required to be created for a unique combination of sampling rate and cpType. But for different cellIds,
                        independence is required, so different groups will be created and assigned for different cell Ids even
                        if their sampling rates and cptypes are identical. */
                    uint32_t                    egressLTEcellId;
                    /** Used for LTE for group creation. Normally an ingress group
                        is required to be created for a unique combination of sampling rate and cpType. But for different cellIds,
                        independence is required, so different groups will be created and assigned for different cell Ids even
                        if their sampling rates and cptypes are identical. */
                    uint32_t                    ingressLTEcellId;
                    /** Holds the Sampling Rate for this AxC */
                    IQN2_SampleRate    			sampleRate;
                    /** Holds a custom Sampling Rate for this AxC if sampleRate=IQN2_SRATE_CUSTOM,
 					    it is expressed in KHz
						The DFE PLL clock (2457600 or 3686400) needs to be a multiple of the custom sampling rate
					*/
                    uint32_t           			customSampleRate;
                    /** Holds a packet size for this AxC if sampleRate=IQN2_SRATE_CUSTOM, customSampleRate is set,
                        and lteCpType=IQN2_CPTYPE_NONE
                        it is expressed in number of samples (IQ)
                        The associated custom sampling rate over 10ms needs to be an integer  multiple of the custom packet size
                        Example: 92160*10/4096=225 for a sampling rate of 92.16 and a packet size of 4096 IQ samples.
                    */
                    uint32_t                    packetSize;
                    /** Holds the CPRI pack mode for this AxC */
                    IQN2_CpriPackingMode        cpriPackMode;
                    /** Holds the inbound data width for this AxC */
                    Iqn2Fl_DataWidth            inboundDataWidth;
                    /** Holds the outbound data width for this AxC */
                    Iqn2Fl_DataWidth            outboundDataWidth;
                    } IQN2_AxCObj, *IQN2_AxCHandle;

/**
 * @brief This structure contains the parameters for the
 * LTE specific radio timer configuration.
 */
 typedef struct IQN2_LteRadTimerObj {
                    /** Holds the cyclic prefix type */
                    IQN2_LteCpType             cpType;
                    /** Determines the TC array values and number of entries (lutindex). The valid range for this is 1 through 7 or 6 depending on CP type. */
                    uint32_t                   numSymbolsForSymbolStrobe;
                    /** Determines the start index of entries (lutindex). The valid range for this is 0 through 256. */
                    uint32_t                   startSymbolsForSymbolStrobe;
                    /** Determines the symbolTC, so number of symbols per frame */
                    uint32_t                   numSymbolStrobesForFrameStrobe;
                    } IQN2_LteRadTimerObj, *IQN2_LteRadTimerHandle;

/**
 * @brief This structure contains the parameters for the
 * WCDMA specific radio timer configuration.
 */
 typedef struct IQN2_WcdmaRadTimerObj {
                    /** Determines the slotTC, so number of slots per frame - can be used to adjust the frame time to WCDMA TTI bigger than 10ms */
                    uint32_t                   numSlotStrobesForFrameStrobe;
                    } IQN2_WcdmaRadTimerObj, *IQN2_WcdmaRadTimerHandle;

/**
 * @brief This structure contains the parameters for the
 * IQN2 radio timer configuration.
 */
 typedef struct IQN2_RadTimerObj {
                    /** Holds this radio timer mode */
                    IQN2_RadTimerMode         mode;
                    /** Flag to indicate this configuration is to override the LLD default one */
                    uint32_t                  userSpecified;
                    /** Holds the init frame lsb number value for this radio timer */
                    uint32_t                  initFrameLsbNum;
                    /** Holds the init frame msb number value for this radio timer */
                    uint32_t                  initFrameMsbNum;
                    /** Determines this RAD timer frame terminal count */
                    uint32_t                  frameTerminalCount;
                    /** BCN offset comparator value
                     *  BCN compare value for synchronization to start the RADT
                     */
                    uint32_t                  bcnCompValue;
                    /** Holds LTE specific parameters for radio timer terminal count values  */
                    IQN2_LteRadTimerObj       lte;
                    /** Holds LTE specific parameters for radio timer terminal count values  */
                    IQN2_WcdmaRadTimerObj     wcdma;
                    } IQN2_RadTimerObj, *IQN2_RadTimerHandle;


/**
 * @brief This structure is used for configuring the parameters of IQN2 AT2 events
 */
typedef struct IQN2_At2EventObj{
                   /** Select Event number (0-23) */
                    IQN2_At2EventIndex      EventSelect;
                   /** Event_offset in Byte Clocks after the start of frame/sym strobes,
                    *  the sys_event generator count out OFFSET clocks then fires once.
                    *  LLD provides a Nanosecs to Byte Clocks conversion function:
                    *  LLD provides a Wcdma chips to Byte Clocks conversion function:
                    */
                   int32_t                  EventOffset;

                   /** Event strobe selection
                     *  Selects which RADT counter and start-of-symbol vs. start-of-frame.
                     *     RADT0_frame (0) = Use RADT Frame to trigger the event
                     *     RADT1_frame (1) = Use RADT Frame to trigger the event
                     *     RADT2_frame (2) = Use RADT Frame to trigger the event
                     *     RADT3_frame (3) = Use RADT Frame to trigger the event
                     *     RADT4_frame (4) = Use RADT Frame to trigger the event
                     *     RADT5_frame (5) = Use RADT Frame to trigger the event
                     *     RADT6_frame (6) = Use RADT Frame to trigger the event
                     *     RADT7_frame (7) = Use RADT Frame to trigger the event
                     *     RADT0_symb (8) = Use RADT Symbol to trigger the event
                     *     RADT1_symb (9) = Use RADT Symbol to trigger the event
                     *     RADT2_symb (10) = Use RADT Symbol to trigger the event
                     *     RADT3_symb (11) = Use RADT Symbol to trigger the event
                     *     RADT4_symb (12) = Use RADT Symbol to trigger the event
                     *     RADT5_symb (13) = Use RADT Symbol to trigger the event
                     *     RADT6_symb (14) = Use RADT Symbol to trigger the event
                     *     RADT7_symb (15) = Use RADT Symbol to trigger the event
                     *     BCN_frame (16) = Use BCN Frame to trigger the event
                     *     EVT_nrest (17) = Use Next higher event to trigger this event.
                     */
                   Iqn2Fl_AtEvtStrobe       EvtStrobeSel;

                    /** Event modulo  in Byte Clocks
                     *  Periodic system event rate.
                     *  After the event has fired once, the event will re-fire
                     *  every MOD clocks. The Nanosecs to Byte Clocks conversion is handled by the LLD
                     *  If set to (-1), setting max value which means no additional events between strobes.
                     *  LLD provides a Nanosecs to Byte Clocks conversion function:
                     *  LLD provides a Wcdma chips to Byte Clocks conversion function:
                     */
                   int32_t                  EventModulo;
                   /** Programmably mask GSM system events. The system event
                    *  generator maintains a count which indexes this mask LUT starting
                    *  with the LSB. When set, the bit suppresses the system event.
                    *  This mechanism supports irregular patterns of system events.
                    *  After the table is exhausted, after 64 events, the events are
                    *  always enabled. The LUT is recycled. Event Mask LSBs.
                    *  Usage:
                    *   - 1 per bit enables that event time.
                    *   - if all 0s, enabling all event time (LLD convention), 0xFFFFFFFF
                    *
                    */
                   uint32_t                 EventMaskLsb;

                   /** Programmably mask GSM system events.
                    *  Event Mask MSBs.
                    *  Usage:
                    *   - 1 per bit enables that event time.
                    *   - if all 0, enabling all event time (LLD convention), 0xFFFFFFFF
                    */
                   uint32_t                 EventMaskMsb;
                } IQN2_At2EventObj, *IQN2_At2EventHandle;


/**
 * @brief This structure is used for configuring the parameters of IQN2 DIO events
 */
typedef struct IQN2_DioEventObj{
				    /** Select UAT DIO RADT terminal count */
					uint32_t      FrameTerminalCount;
					/** Select UAT RADT event clock counter terminal count */
					uint16_t      EventModulo;
					/** Select UAT RADT event compare . When compare value equals RADT count, frame rate event is generated */
					uint32_t      DioFrameEventOffset;
				} IQN2_DioEventObj, *IQN2_DioEventHandle;

/**
 * @brief This structure contains the parameters for the
 * initialization of a IQN2 AIL
 */
 typedef struct IQN2_AilConfigObj {
                    /** Selects Ail to enable (0=disable; 1=enable). */
                    uint32_t                   ailEnable;
                    /** Selects IQN2 link rate (2=2x; 4= 4x ; 5= 5x (CPRI only); 8= 8x). */
                    Iqn2Fl_LinkRate            linkRate;
                    /** Selects IQN2 communication mode */
                    IQN2_ComMode               comType;
                    /** Holds PE2 offset (PE_STB) (set by the library)*/
                    uint32_t                   pe2Offset;
                    /** Holds Delta Offset (set by the library)*/
                    uint32_t                   deltaOffset;
                    /** Holds Pi min (can be user defined or set by the library)*/
                    uint32_t                   piMin;
                    /** NodeTx parameter allows inserting a certain delay based on the place of the KeyStone SoC in the antenna daisy chain.
                     * For a direct communication between two devices, the nodeTx value needs to be set to �0�. For the 1st retransmission node,
                     * set it to 1, and so on
                     */
                    uint32_t                   nodeTx;
                    /** NodeRx parameter allows inserting a certain delay based on the place of the KeyStone SoC in the antenna daisy chain.
                     * For a direct communication between two devices, the nodeTx value needs to be set to �0�. For the 1st retransmission node,
                     * set it to 1, and so on
                     */
                    uint32_t                   nodeRx;
                    /** Maximum expected wait time in nanosecs for:
                     *  PktDMA mode: popping a descriptor and DMA of IQ sample from application memory to AIL egress SI
                     *  DIO mode: TAC processing time (4 chip) and DMA time from TAC or application memory to AIL egress SI
                     *  This parameter is used by calcAifTimingForTxNode() to compute deltaOffset.
                     *  It actually corresponds to PE_STB value expressed in ns
                     */
                    uint32_t                   txWait;
                    /** Holds the byte clock expressed in KHz. (set by the library)*/
                    uint32_t                   byteClockInKHz;
                    /** Holds the first LTE AxC carried on this Ail.*/
                    uint32_t                   firstLteAxC;
					/** Holds the first WCDMA AxC carried on this Ail.*/
                    uint32_t                   firstWcdmaAxC;
                    /** Holds the first CTL channels carried on this Ail.*/
                    uint32_t                   firstCtlChannel;
                    /** Holds the number of egress LTE AxCs on this Ail.*/
                    uint32_t                   numLtePeAxC;
                    /** Holds the number of egress WCDMA AxCs on this Ail. */
                    uint32_t                   numWcdmaPeAxC;
                    /** Holds the number of ingress LTE AxCs on this Ail.*/
                    uint32_t                   numLtePdAxC;
					/** Holds the number of ingress WCDMA AxCs on this Ail.*/
                    uint32_t                   numWcdmaPdAxC;
                    /** Holds the number of CTL channels on this Ail.*/
                    uint32_t                   numCtlChannel;
                    } IQN2_AilConfigObj, *IQN2_AilConfigHandle;

/**
 * @brief This structure contains the parameters for the
 * initialization of a IQN2 AID
 */
 typedef struct IQN2_AidConfigObj {
                    /** Holds the enable flag for Aid (0=disable; 1=enable). */
                    uint32_t                   aidEnable;
                    /** Holds the first LTE AxC on this Aid.*/
                    uint32_t                   firstLteAxC;
                    /** Holds the first WCDMA AxC on this Aid.*/
                    uint32_t                   firstWcdmaAxC;
                    /** Holds the first CTL channel on this Aid.*/
                    uint32_t                   firstCtlChannel;
                    /** Holds the number of egress LTE AxCs on this Aid.*/
                    uint32_t                   numLteEgressAxC;
                    /** Holds the number of egress WCDMA AxCs on this Aid.*/
                    uint32_t                   numWcdmaEgressAxC;
                    /** Holds the number of ingress LTE AxC on this Aid.*/
                    uint32_t                   numLteIngressAxC;
                    /** Holds the number of ingress WCDMA AxC on this Aid.*/
                    uint32_t                   numWcdmaIngressAxC;
                    /** Holds the number of CTL channels on this Aid.*/
                    uint32_t                   numCtlChannel;
                    /** Holds the AID2 SI delay on this Aid */
                    uint32_t                   siDelay;
                    /** Setup the LTE traffic to use DIO instead of PKTDMA */
                    uint32_t 				   lteDio;
                    } IQN2_AidConfigObj, *IQN2_AidConfigHandle;
					
/**
 * @brief This structure contains the parameters for the
 * initialization of a IQN2 Dio engine
 */
 typedef struct IQN2_DioConfigObj {
                    /** Holds the enable flag for DIO engine (0=disable; 1=enable). */
                    uint32_t                   dioEnable;
                    /** Selects buffer start address for this DIO engine in egress direction */
                    uint32_t*                  out[16];
                    /** Selects number of DMA blocks (wrap2) for this DIO engine in egress direction */
                    uint32_t                   outNumBlock;
                    /** Selects buffer start address for this DIO engine in ingress direction */
                    uint32_t*                  in[16];
                    /** Selects number of DMA blocks (wrap2) for this DIO engine in ingress direction */
                    uint32_t                   inNumBlock;
                    /** Holds offset to the first DB channel associated to this DIO engine on transmit side */
                    uint32_t                   offsetPeDBCH;
                    /** Holds offset to the first DB channel associated to this DIO engine on receive side */
                    uint32_t                   offsetPdDBCH;
                    /** Holds number of DB channels associated to this DIO engine on transmit path(set by the library) */
                    uint32_t                   numPeDBCH;
                    /** Holds number of DB channels associated to this DIO engine on receive path (set by the library) */
                    uint32_t                   numPdDBCH;
                    /** Determine if we use the DIO duplicate feature with the current dio (0= disable, 1= enable).
                     *  Duplicate feature uses dio engine 2, so this feature cannot be used on both DIO 0 and DIO 1 at the same time
                     *  This duplicate feature can also be used for the purpose of debugging DIO traffic going to RAC */
                    uint32_t                   duplicateOnDioEng2;
                    /** When in Wcdma mode, tells whether this DIO engine is used for RAC on ingress side */
                    uint32_t                   usedWithRAC;
                    /** When in Wcdma mode, tells whether this DIO engine is used for TAC on egress side */
                    uint32_t                   usedWithTAC;
					/** Set Ingress UL traffic  */
					uint32_t                   rsaOn;
					/** Set Egress UL traffic  */
					uint32_t                   egRsaOn;
					/** Set if dio is used for LTE or WCDMA */
					uint32_t 				   lteMode;
					/** dio first aid AxC */
                    uint32_t                   firstAxC;
                    } IQN2_DioConfigObj, *IQN2_DioConfigHandle;

/**
 * @brief This structure contains the pointers to the IQN2 TOP exception counters.
 */
  typedef struct IQN2_EeTopCountObj {
                    /** Holds counters for TOP PSR EE INGRESS FLUSH A data information */
                    Iqn2Fl_PsrEeIngFlushA           psrEeIngFlushACnt;
                    /** Holds counters for TOP PSR EE INGRESS FLUSH b data information */
                    Iqn2Fl_PsrEeIngFlushB           psrEeIngFlushBCnt;
                    /** Holds counters for TOP PSR EE EGRESS PROTOCOL ERR A data information */
                    Iqn2Fl_PsrEeEgressProtocolErrA  psrEeEgressProtocolErrACnt;
                    /** Holds counters for TOP PSR EE EGRESS PROTOCOL ERR b data information */
                    Iqn2Fl_PsrEeEgressProtocolErrB  psrEeEgressProtocolErrBCnt;
                    /** Holds counters for TOP PKTDMA EE DESC STARVE data information */
                    Iqn2Fl_PktdmaEeDescStarve       pktdmaEeDescStarveCnt;
  } IQN2_EeTopCountObj, *IQN2_EeTopCountHandle;

/**
* @brief This structure contains the pointers to the IQN2 AT2 exception counters.
*/
  typedef struct IQN2_EeAt2CountObj {
                    /** Holds counters for AT2 EE AT data information */
                    Iqn2Fl_At2EeInfoErr             at2EeInfoErrCnt;
  } IQN2_EeAt2CountObj, *IQN2_EeAt2CountHandle;

/**
* @brief This structure contains the pointers to the IQN2 IQS2 exception counters.
*/
  typedef struct IQN2_EeIqs2CountObj {
                    /** Holds counters for IQS2 EE CHAN ERR data information */
                    Iqn2Fl_Iqs2EeChanErr        iqs2EeChanErrCnt;
                    /** Holds counters for IQS2 EE INGRESS FLUSH ERR data information */
                    Iqn2Fl_Iqs2EeIngFlush       iqs2EeIngFlushCnt;
                    /** Holds counters for IQS2 EE EGRESS FLUSH ERR data information */
                    Iqn2Fl_Iqs2EeEgrFlush       iqs2EeEgrFlushCnt;
  } IQN2_EeIqs2CountObj, *IQN2_EeIqs2CountHandle;

/**
* @brief This structure contains the pointers to the IQN2 AID2 exception counters.
*/
  typedef struct IQN2_EeAid2CountObj {
                    /** Holds counters for AID2 EE SII A ERR data information */
                    Iqn2Fl_Aid2EeSiiA   aid2EeSiiACnt;
                    /** Holds counters for AID2 EE SII B ERR data information */
                    Iqn2Fl_Aid2EeSiiB   aid2EeSiiBCnt;
                    /** Holds counters for AID2 EE SII C ERR data information */
                    Iqn2Fl_Aid2EeSiiC   aid2EeSiiCCnt;
                    /** Holds counters for AID2 EE SII D ERR data information */
                    Iqn2Fl_Aid2EeSiiD   aid2EeSiiDCnt;
                    /** Holds counters for AID2 EE SII E ERR data information */
                    Iqn2Fl_Aid2EeSiiE   aid2EeSiiECnt;
                    /** Holds counters for AID2 EE SII F ERR data information */
                    Iqn2Fl_Aid2EeSiiF   aid2EeSiiFCnt;
                    /** Holds counters for AID2 EE SII G ERR data information */
                    Iqn2Fl_Aid2EeSiiG   aid2EeSiiGCnt;
                    /** Holds counters for AID2 EE SII H ERR data information */
                    Iqn2Fl_Aid2EeSiiH   aid2EeSiiHCnt;
                    /** Holds counters for AID2 EE SIE A ERR data information */
                    Iqn2Fl_Aid2EeSieA   aid2EeSieACnt;
                    /** Holds counters for AID2 EE SIE B ERR data information */
                    Iqn2Fl_Aid2EeSieB   aid2EeSieBCnt;
                    /** Holds counters for AID2 EE SIE C ERR data information */
                    Iqn2Fl_Aid2EeSieC   aid2EeSieCCnt;
                    /** Holds counters for AID2 EE SIE D ERR data information */
                    Iqn2Fl_Aid2EeSieD   aid2EeSieDCnt;
                    /** Holds counters for AID2 EE SIE E ERR data information */
                    Iqn2Fl_Aid2EeSieE   aid2EeSieECnt;
                    /** Holds counters for AID2 EE SIE F ERR data information */
                    Iqn2Fl_Aid2EeSieF   aid2EeSieFCnt;
                    /** Holds counters for AID2 EE SIE G ERR data information */
                    Iqn2Fl_Aid2EeSieG   aid2EeSieGCnt;
                    /** Holds counters for AID2 EE DFE ERR data information */
                    Iqn2Fl_Aid2EeDfe    aid2EeDfeCnt;
  } IQN2_EeAid2CountObj, *IQN2_EeAid2CountHandle;

  /**
   * @brief This structure contains the parameters for the
   * hfn sync state. It could be used to detect when sync is
   * established (syncStatusChange = 1 and hfnsyncState = 1) or when
   * sync is lost (syncStatusChange = 1 and hfnsyncState = 0)
   * The context is in the IQN2_getException processing function where below
   * parameters are updated before clearing the exceptions.
   */
   typedef struct IQN2_AilHfnSyncObj {
                      /** 1 (TRUE) if sync status changed, 0 otherwise It is basically the rm_ee_sync_status_change_err in EE link A */
                      Bool syncStatusChange;
                      /** if statusChanged above is 1, then indicates whether in hfnsync i.e state ST3 (= 1) or not (=0).
                          It is basically the rm_ee_hfnsync_state_err in EE link A */
                      Bool hfnsyncState;
   } IQN2_AilHfnSyncObj;

  /**
  * @brief This structure contains the pointers to the IQN2 AIL exception counters.
  */
  typedef struct IQN2_EeAilCountObj {
                      /** Holds counters for AIL EE SII A ERR data information */
                      Iqn2Fl_AilEeSiiA    ailEeSiiACnt;
                      /** Holds counters for AIL EE SII B ERR data information */
                      Iqn2Fl_AilEeSiiB    ailEeSiiBCnt;
                      /** Holds counters for AIL EE SII C0 ERR data information */
                      Iqn2Fl_AilEeSiiC0   ailEeSiiC0Cnt;
                      /** Holds counters for AIL EE SII C1 ERR data information */
                      Iqn2Fl_AilEeSiiC1   ailEeSiiC1Cnt;
                      /** Holds counters for AIL EE SII D ERR data information */
                      Iqn2Fl_AilEeSiiD    ailEeSiiDCnt;
                      /** Holds counters for AIL EE SII E ERR data information */
                      Iqn2Fl_AilEeSiiE    ailEeSiiECnt;
                      /** Holds counters for AIL EE SII F ERR data information */
                      Iqn2Fl_AilEeSiiF    ailEeSiiFCnt;
                      /** Holds counters for AIL EE SII G0 ERR data information */
                      Iqn2Fl_AilEeSiiG0   ailEeSiiG0Cnt;
                      /** Holds counters for AIL EE SII G1 ERR data information */
                      Iqn2Fl_AilEeSiiG1   ailEeSiiG1Cnt;
                      /** Holds counters for AIL EE SII H ERR data information */
                      Iqn2Fl_AilEeSiiH    ailEeSiiHCnt;
                      /** Holds counters for AIL EE SIE A ERR data information */
                      Iqn2Fl_AilEeSieA    ailEeSieACnt;
                      /** Holds counters for AIL EE SIE B ERR data information */
                      Iqn2Fl_AilEeSieB    ailEeSieBCnt;
                      /** Holds counters for AIL EE SIE C ERR data information */
                      Iqn2Fl_AilEeSieC    ailEeSieCCnt;
                      /** Holds counters for AIL EE SIE D ERR data information */
                      Iqn2Fl_AilEeSieD    ailEeSieDCnt;
                      /** Holds counters for AIL EE SIE E ERR data information */
                      Iqn2Fl_AilEeSieE    ailEeSieECnt;
                      /** Holds counters for AIL EE SIE F0 ERR data information */
                      Iqn2Fl_AilEeSieF0   ailEeSieF0Cnt;
                      /** Holds counters for AIL EE SIE F1 ERR data information */
                      Iqn2Fl_AilEeSieF1   ailEeSieF1Cnt;
                      /** Holds counters for AIL EE SIE G ERR data information */
                      Iqn2Fl_AilEeSieG    ailEeSieGCnt;
                      /** Holds counters for AIL EE RM 0 ERR data information */
                      Iqn2Fl_AilEeRm0     ailEeRm0Cnt;
                      /** Holds counters for AIL EE RM 0 ERR data information */
                      Iqn2Fl_AilEeRtTm0   ailEeRtTm0Cnt;
                      /** Holds counters for AIL EE CI CO 0 ERR data information */
                      Iqn2Fl_AilEeCiCo0   ailEeCiCo0Cnt;
                      /** Holds counters for AIL EE PD 0 ERR data information */
                      Iqn2Fl_AilEePd0     ailEePd0Cnt;
                      /** Holds counters for AIL EE PD 1 ERR data information */
                      Iqn2Fl_AilEePd1     ailEePd1Cnt;
                      /** Holds counters for AIL EE PE 0 ERR data information */
                      Iqn2Fl_AilEePe0     ailEePe0Cnt;
                      /** Holds counters for AIL EE SI 0 ERR data information */
                      Iqn2Fl_AilEeSi0     ailEeSi0Cnt;
                      /** Holds captured Pi if Pi is detected as being out of the window */
                      uint32_t            ailPiCaptured;
                      /** Holds hfnsync information */
                      IQN2_AilHfnSyncObj  ailEeHfnsync;
  } IQN2_EeAilCountObj, *IQN2_EeAilCountHandle;

  /**
  * @brief This structure contains the pointers to the IQN2 DIO2 exception counters.
  */
  typedef struct IQN2_EeDio2CountObj {
                      /** Holds counters for DIO2 EE DMA ING A ERR data information */
                      Iqn2Fl_Dio2EeDmaIngA    dio2EeDmaIngACnt;
                      /** Holds counters for DIO2 EE DMA ING B ERR data information */
                      Iqn2Fl_Dio2EeDmaIngB    dio2EeDmaIngBCnt;
                      /** Holds counters for DIO2 EE DMA EGR A ERR data information */
                      Iqn2Fl_Dio2EeDmaEgrA    dio2EeDmaEgrACnt;
                      /** Holds counters for DIO2 EE DT A ERR data information */
                      Iqn2Fl_Dio2EeDtA        dio2EeDtACnt;
                      /** Holds counters for DIO2 EE SII A ERR data information */
                      Iqn2Fl_Dio2EeSiiA       dio2EeSiiACnt;
                      /** Holds counters for DIO2 EE SII C ERR data information */
                      Iqn2Fl_Dio2EeSiiC       dio2EeSiiCCnt;
                      /** Holds counters for DIO2 EE SII E ERR data information */
                      Iqn2Fl_Dio2EeSiiE       dio2EeSiiECnt;
                      /** Holds counters for DIO2 EE SII G ERR data information */
                      Iqn2Fl_Dio2EeSiiG       dio2EeSiiGCnt;
                      /** Holds counters for DIO2 EE SIE A ERR data information */
                      Iqn2Fl_Dio2EeSieA       dio2EeSieACnt;
                      /** Holds counters for DIO2 EE SIE D ERR data information */
                      Iqn2Fl_Dio2EeSieD       dio2EeSieDCnt;
                      /** Holds counters for DIO2 EE SIE E ERR data information */
                      Iqn2Fl_Dio2EeSieF       dio2EeSieFCnt;
  } IQN2_EeDio2CountObj, *IQN2_EeDio2CountHandle;

/**
 * @brief This structure contains all the IQN2 exception counters.
 */
  typedef struct IQN2_EeCountObj {
					 /** Holds counters for iqn2 EE TOP data information */
                      IQN2_EeTopCountObj    eeTop;
                     /** Holds counters for iqn2 AT2 Interrupt data information */
                      IQN2_EeAt2CountObj    eeAt2;
                      /** Holds counters for iqn2 IQS2 Interrupt data information */
                      IQN2_EeIqs2CountObj   eeIqs2;
                      /** Holds counters for iqn2 AIL Interrupt data information */
                      IQN2_EeAilCountObj    eeAil[IQN2_MAX_NUM_AIL];
                      /** Holds counters for iqn2 AID2 Interrupt data information */
                      IQN2_EeAid2CountObj   eeAid2;
                      /** Holds counters for iqn2 DIO2 Interrupt data information */
                      IQN2_EeDio2CountObj   eeDio2;
                     /** Holds a flag telling whether any of the enabled exceptions occurred, 0 if none, 1 if any */
                     uint32_t			    eeFlag;
 	 	 	 	 	 } IQN2_EeCountObj, *IQN2_EeCountHandle;

/**
* @brief This structure contains the parameters for the
* IQN2 AIL Data Trace setup.
*/
typedef struct IQN2_AilDataTraceObj {
                     uint32_t                placeHolder;
                 } IQN2_AilDataTraceObj, *IQN2_AilDataTraceHandle;


/**
 * @brief This structure contains the parameters for the
 * initialization of a IQN2 HW module of the TMS320TCI6630K2L.
 * it includes configuration objects for all the links.
 */
 typedef struct IQN2_ConfigObj {
                    /** Holds pointer to IQN2 FL object instance. (set by the library) */
                    Iqn2Fl_Handle           hFl;
                    /** Holds pointer to IQN2 configuration structure (set by the library) */
                    Iqn2Fl_Setup           *hIqn2Setup;
                    /** Selects CPRI,OBSAI, or DFE (245.76 or 368.64 Mhz)  for IQN2 */
                    Iqn2Fl_Protocol 	 	protocol;
                    /** Selects the sync source input for the software synchronization process (iqn2 timer offset adjustments) */
                    IQN2_SyncSource         timerSyncSource;
                    /** Holds configuration structure for each AIL  */
                    IQN2_AilConfigObj       ailConfig[IQN2_MAX_NUM_AIL];
					/** Holds configuration structure the AID  */
                    IQN2_AidConfigObj       aidConfig;
					/** Holds configuration structure for Dio engine  */
					IQN2_DioConfigObj		dioConfig[IQN2_MAX_NUM_DIO_ENGINE];
                    /** Holds specific configuration structure for each AxC */
                    IQN2_AxCObj             AxCconfig[IQN2_MAX_NUM_AXC];
                    /** Holds specific configuration structure for each radio timer */
                    IQN2_RadTimerObj        radTimerConfig[IQN2_MAX_NUM_RADT];
                    /** Holds specific configuration structure for each at2 event */
                    IQN2_At2EventObj        at2EventConfig[IQN2_MAX_NUM_AT2_EVENT];
                    /** Exception counters - updated when exception are enabled */
                    IQN2_EeCountObj         iqn2EeCount;
                    } IQN2_ConfigObj, *IQN2_ConfigHandle;
                    
/**
@} */


#ifdef __cplusplus
}
#endif


#endif //__IQN2_DEFS_H

