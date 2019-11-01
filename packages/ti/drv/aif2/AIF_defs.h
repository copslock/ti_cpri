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


                    
/** 
 * @file AIF_defs.h
 *
 * @brief Header file for AIF module definitions
 * 
*/

/** 
 * @mainpage About the AIF2 LLD
 *
 *
 * @section vvv Purpose and Scope
 * This AIF2 library aims at generalizing the configuration of AIF2 for different
 * modes (CPRI/OBSAI/ABTLib/Generic packet, WCDMA/LTE/Dual mode).
 * The AIF2 library makes use of AIF2 Chip Support Library and CPPI/QMSS Low
 * Level Drivers (LLDs)
 * The AIF2 library is delivered as a RTSC package (ti.drv.aif2).
 *
 * @section xxx AIF2 LLD usage details are included in the user's guide
 *   @subsection aaa Configuration structure
 *   AIF_ConfigObj exposes "user-friendly" parameters
 *   to allow users to configure AIF2 with a general understanding of how AIF2 HW works.
 *   Parameters such as the protocol (CPRI, OBSAI), AIF2 input clock speed, or DIO engine/PKTDMA
 *   configurations are expected from the application. AIF2 configuration examples are provided
 *   by the test examples delivered with the library.
 * @section zzz Related TI Documentation and Software Packages
 *   -# DSP HW references:  <a href="http://www.ti.com/product/tms320c6670"> TMS320C6670 Multicore DSP Manual and User's Guide</a>
 *   -# DSP SW references:  <a href="http://processors.wiki.ti.com/index.php/BIOS_MCSDK_2.0_User_Guide#Platform_Development_Kit_.28PDK.29">
 *                          C66x Peripheral Driver Kit (PDK) User's Guide</a>
 *   -# DSP SW download:  <a href="http://www.ti.com/tool/bioslinuxmcsdk"> BIOSMCSDK-C66X</a>
 *   -# EVM HW references:  <a href="http://www.advantech.com/Support/TI-EVM/6670le_sd.aspx"> TMDXEVM6670L/LE documentation</a>
 *
 * @section xyz Terms and Abbreviations
 *   -# CSL:  Chip Support Library
 *   -# PDK:  Peripheral Driver Kit
 *   -# MCSDK:  Multicore Software Development Kit
 *   -# RTSC:  Real-time Software Components
 *   -# API:  Application Programmer Interface
 *   -# AIF2:  Antenna Interface version 2 for KeyStone devices
 *   -# CCS:  Code Composer Studio  
 *
 */

/** @addtogroup AIF_API AIF
 *  The AIF module is dedicated to the definition of generic elements for
 *  the C66x KeyStone AIF2 module and dedicated to the H/W initialization of the
 *  AIF2 module given the AIF2 object configuration parameters from the application user.
 *  @{
 */

/**
 * @defgroup AIF_DATASTRUCT AIF Data Structures
 * @ingroup AIF_API
 */
/**
 * @defgroup AIF_ENUM  AIF Enumerated Data Types
 * @ingroup AIF_API
 */
/**
 * @defgroup AIF_FUNCTION  AIF Functions
 * @ingroup AIF_API
 */


#ifndef __AIF_DEFS_H
#define __AIF_DEFS_H

#ifdef _TMS320C6X
#include <c6x.h>
#endif

#include <ti/csl/csl.h>
#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2_types.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

#ifdef __cplusplus
extern "C" {
#endif

 /** @addtogroup AIF_ENUM
 *
 * @{ */


/** Maximum number of AIF2 links */
#define AIF_MAX_NUM_LINKS 						(AIF2FL_LINK_5 + 1)
/** Maximum number of AIF2 antenna containers */
#define AIF_MAX_NUM_AXC							128
 /** Maximum number of AIF2 radio timers */
#define AIF_MAX_NUM_RADT						1
/** Default CRC8 polynomial */
#define AIF2_CRC8_POLY                			0xF
/** Default CRC8 seed */
#define AIF2_CRC8_SEED                 			0xF
/** Delay between DB and PE for OBSAI */
#define AIF2_DB_PE_DELAY_OBSAI                 	28
/** Delay between DB and PE for CPRI */
#define AIF2_DB_PE_DELAY_CPRI                   0
/** OBSAI header type for control */
#define AIF2_OBSAI_TYPE_CONTROL          		0x00
/** OBSAI header type for Wcdma Fdd */
#define AIF2_OBSAI_TYPE_WCDMA_FDD          		0x02
/** OBSAI header type for Lte  */
#define AIF2_OBSAI_TYPE_LTE                     0x0E
/** OBSAI header type for generic packet */
#define AIF2_OBSAI_TYPE_GENERIC                 0x0F
/** Frame terminal count for Wcdma Fdd */
#define AIF2_FRAME_COUNT_TC_WCDMA_FDD   		4095
/** Slot terminal count for Wcdma Fdd */
#define AIF2_SLOT_COUNT_TC_WCDMA_FDD	    	14
/** 3GPP clock count TC for OBSAI (2x, 4x, 8x)*/
#define AIF2_CLOCK_COUNT_TC_WCDMA_FDD		   	204799
 /** LTE OBSAI first SYMBOL byte clock size for Nomal CP */
#define AIF2_CLOCK_COUNT_TC_FIRST_OFDM_SYM_OBSAI		22080
 /** LTE OBSAI other SYMBOL byte clock size for Nomal CP */
#define AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_OBSAI			21920
 /** LTE OBSAI SYMBOL byte clock size for Extended CP */
#define AIF2_CLOCK_COUNT_TC_EXTCP_OFDM_SYM_OBSAI		25600

/** */
#define AIF2_FRAME_COUNT_TC_PHY_TIMER  		4095
/** */
#define AIF2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER 3071999
// Byte clock count TC for CPRI and 1ms time - 1
#define AIF2_CLOCK_COUNT_TC_CPRI  	245759
 // Byte clock count TC for OBSAI and 1ms time - 1
#define AIF2_CLOCK_COUNT_TC_OBSAI  	307199
/** */
#define AIF2_CLOCK_COUNT_TC_WCDMA_FDD_CPRI	163839
 /** LTE CPRI first SYMBOL byte clock size for Nomal CP */
#define AIF2_CLOCK_COUNT_TC_FIRST_OFDM_SYM_CPRI		17664
 /** LTE CPRI other SYMBOL byte clock size for Nomal CP */
#define AIF2_CLOCK_COUNT_TC_REST_OFDM_SYM_CPRI		17536
 /** LTE CPRI SYMBOL byte clock size for Extended CP */
 #define AIF2_CLOCK_COUNT_TC_EXTCP_OFDM_SYM_CPRI	20480
/** */
#define AIF2_CPRI_CLOCK_COUNT_TC_PHY_TIMER	2457599


#define AIF2_DB_BASE_ADDR_I_FIFO_0           0
#define AIF2_DB_BASE_ADDR_I_FIFO_1           4
#define AIF2_DB_BASE_ADDR_I_FIFO_2           8

#define AIF2_DB_BASE_ADDR_E_FIFO_0           0
#define AIF2_DB_BASE_ADDR_E_FIFO_1           4
#define AIF2_DB_BASE_ADDR_E_FIFO_2           8


#define NB_STREAM 							 16
#define EGRESS_DATA_SIZE					 64	
#define INGRESS_DATA_SIZE					 64	
/** LTE parameters */
#define AIF2_LTE_SYMBOL_NUM					7
#define AIF2_LTE_FRAME_SYMBOL_NUM			140 //7x2 Antennas x 10
#define AIF2_LTE_SYMBOL_NUM_EXT_CP			6
#define AIF2_LTE_FRAME_SYMBOL_NUM_EXT_CP	120 //6x2 Antennas x 10

#define AIF2_LTE20_FFT_SIZE					2048
#define AIF2_LTE15_FFT_SIZE					1536
#define AIF2_LTE10_FFT_SIZE					1024
#define AIF2_LTE5_FFT_SIZE					512
#define AIF2_LTE1P4_FFT_SIZE				128
#define AIF2_LTE40_FFT_SIZE					4096

#define AIF2_LTE20_CYPRENORMAL1_SIZE		160
#define AIF2_LTE20_CYPRENORMAL_SIZE			144
#define AIF2_LTE20_CYPREEXTENDED_SIZE		512
#define AIF2_LTE15_CYPRENORMAL1_SIZE		120
#define AIF2_LTE15_CYPRENORMAL_SIZE			108
#define AIF2_LTE15_CYPREEXTENDED_SIZE		384
#define AIF2_LTE10_CYPRENORMAL1_SIZE		80
#define AIF2_LTE10_CYPRENORMAL_SIZE			72
#define AIF2_LTE10_CYPREEXTENDED_SIZE		256
#define AIF2_LTE5_CYPRENORMAL1_SIZE			40
#define AIF2_LTE5_CYPRENORMAL_SIZE			36
#define AIF2_LTE5_CYPREEXTENDED_SIZE		128
#define AIF2_LTE1P4_CYPRENORMAL1_SIZE		10
#define AIF2_LTE1P4_CYPRENORMAL_SIZE		9
#define AIF2_LTE1P4_CYPREEXTENDED_SIZE		32
#define AIF2_LTE40_CYPRENORMAL1_SIZE		320
#define AIF2_LTE40_CYPRENORMAL_SIZE			288
#define AIF2_LTE40_CYPREEXTENDED_SIZE		1024

/** missing parameters from the AIF2 CSL*/
#define AIF2_BASE_TX_QUE_NUM 		512
/* AIF2 RM configuration */
#define AIF2_RM_LOS_DET_THOLD      10
#define AIF2_RM_SYNC_THOLD         1
#define AIF2_RM_UNSYNC_THOLD       5
/* AIF1 RM configuration */
#define AIF1_RM_LOS_DET_THOLD      255
#define AIF1_RM_SYNC_THOLD         3
#define AIF1_RM_UNSYNC_THOLD       3

/** AIF2 DATA address for ABT usage 4k/link*/
#define AIF2_0_RX_DATA	0x108E0000
#define AIF2_1_RX_DATA	0x108E1000
#define AIF2_2_RX_DATA	0x108E2000
#define AIF2_3_RX_DATA	0x108E3000
#define AIF2_4_RX_DATA	0x108E4000
#define AIF2_5_RX_DATA	0x108E5000
#define AIF2_0_TX_DATA	0x108E6000
#define AIF2_1_TX_DATA	0x108E7000
#define AIF2_2_TX_DATA	0x108E8000
#define AIF2_3_TX_DATA	0x108E9000
#define AIF2_4_TX_DATA	0x108EA000
#define AIF2_5_TX_DATA	0x108EB000
#define AIF2_LINK_BLOCK_SIZE 	0x1000

// INTC0 EVENT input numbers for AIF2
#define AIF2_EVENT0_INTSEL_MAP 102
#define AIF2_EVENT1_INTSEL_MAP 103
#define AIF2_EVENT2_INTSEL_MAP 104
#define AIF2_EVENT3_INTSEL_MAP 105
#define AIF2_EVENT4_INTSEL_MAP 106
#define AIF2_EVENT5_INTSEL_MAP 107
#define AIF2_EVENT6_INTSEL_MAP 108
#define AIF2_EVENT7_INTSEL_MAP 109


/** ABT inbound outbound data type depending on AIF version */
#define AIF_LINK_DATA_TYPE_DL AIF2FL_LINK_DATA_TYPE_NORMAL


/** Max number of AxC for 2X link rate */
#define AIF2_OBSAI_MAX_NUM_AXC_PER_2X_LINK                               (8)
/** Max number of AxC for 4X link rate */
#define AIF2_OBSAI_MAX_NUM_AXC_PER_4X_LINK                               (16)
/** Max number of AxC for 8X link rate */
#define AIF2_OBSAI_MAX_NUM_AXC_PER_8X_LINK                               (32)

/** Max number of AxC per link CPRI 15*/

#define AIF2_CPRI_MAX_NUM_AXC_PER_2X_LINK_7_15_BIT                               (8)

#define AIF2_CPRI_MAX_NUM_AXC_PER_4X_LINK_7_15_BIT                               (16)

#define AIF2_CPRI_MAX_NUM_AXC_PER_5X_LINK_7_15_BIT                               (16)

#define AIF2_CPRI_MAX_NUM_AXC_PER_8X_LINK_7_15_BIT                           	 (32)

/** Max number of AxC per link CPRI 16*/

#define AIF2_CPRI_MAX_NUM_AXC_PER_2X_LINK_8_16_BIT                               (7)

#define AIF2_CPRI_MAX_NUM_AXC_PER_4X_LINK_8_16_BIT                               (15)

#define AIF2_CPRI_MAX_NUM_AXC_PER_5X_LINK_8_16_BIT                               (15)

#define AIF2_CPRI_MAX_NUM_AXC_PER_8X_LINK_8_16_BIT                               (30)

/** Max number of Dio engine */
#define AIF2_MAX_NUM_DIO_ENGINE		3

/** Max number of CPRI control word sub-streams*/
#define AIF2_CPRI_MAX_CW_SUBSTREAM  4

/** */

#define  AIF2_NUM_WORDS_PER_QWORD	4

/** AIF timing delay between protocol encoder and tx mac for CPRI */
#define AIF_CPRI_PE_TO_TM_DELAY 	28
#define AIF2_CPRI_PE_TO_TM_DELAY 	30	//sys_clk
/** AIF timing delay between protocol encoder and tx mac for OBSAI */
#define AIF_OBSAI_PE_TO_TM_DELAY 	22
#define AIF2_OBSAI_PE_TO_TM_DELAY 	58 //sys_clk

#define AIF2_WCDMA_OBSAI_DL_Pe2OffSet		610  //  TAC requires about 240 byte clock for processing and DIO DMA requires 256 ~ 300 byte clock to transfer 4 chip data
#define AIF2_WCDMA_OBSAI_UL_Pe2OffSet		610  //  TAC requires about 240 byte clock for processing and DIO DMA requires 256 ~ 300 byte clock to transfer 4 chip data
#define AIF2_WCDMA_CPRI_UL_Pe2OffSet		490
#define AIF2_WCDMA_CPRI_DL_Pe2OffSet		490
#define AIF2_LTE_OBSAI_UL_Pe2OffSet		    310
#define AIF2_LTE_OBSAI_DL_Pe2OffSet		    310
#define AIF2_LTE_CPRI_UL_Pe2OffSet		    310  //  For LTE you can remove TAC delay. Not enought information on PktDma processing TODO
#define AIF2_LTE_CPRI_DL_Pe2OffSet		    310
#define AIF2_LTE_WCDMA_OBSAI_DL_Pe2OffSet   310
#define AIF2_LTE_WCDMA_OBSAI_UL_Pe2OffSet   310
#define AIF2_OBSAI_AXC_OFFSET_WIN           300

#define AIF2_CPRI_AXC_OFFSET_WIN    0
/** AIF minimum number of bytes that must be written to the tx mac fifo before it starts */
#define AIF_TM_FIFO_FILL_MIN 		4
/** AIF SerDes processing delay (Tx mac, line and Rx mac) */
#define AIF_SERDES_PROCESSING_DELAY 10
#define AIF2_SERDES_PROCESSING_DELAY 12 //in loopback
/** AIF redirection delay between Rx and Tx mac for CPRI */
#define AIF_CPRI_RM_TO_TM_DELAY 	60
/** AIF redirection delay between Rx and Tx mac for OBSAI */
#define AIF_OBSAI_RM_TO_TM_DELAY 	38
/** AIF UMTS chip speed in KHz */
#define AIF_UMTS_CHIP_SPEED_3840K 	3840
/** AIF UMTS sample speed in KHz */
#define AIF_UMTS_SAMPLE_SPEED_KHZ	(AIF_UMTS_CHIP_SPEED_3840K*8)
/** AIF VBUS versus CPU speed ratio */
#define AIF_VBUS_CPU_RATIO          3
/** AIF maximum number of streams */
#define AIF_MAX_NUM_STREAMS         16
/** AIF RAM outbound data offset */
#define AIF_OUTDATA_OFFSET          0x02000000 
/** AIF RAM block size for one link of inbound or outband data */
#define AIF_LINK_BLOCK_SIZE 		0x00400000
/** AIF RAM block size for inbound/outbound CS RAM */
#define AIF_LINK_PHY_BLOCK_SIZE		2048
/** AIF RAM outbound PS FIFO offset */
#define AIF_OUTPSFIFO_OFFSET        0x05000000
/** AIF RAM outbound PS data block */
#define AIF_PS_OUT_DATA_BLOCK_SIZE  0x80000
/** AIF RAM inbound PS FIFO offset */
#define AIF_INPSFIFO_OFFSET         0x04000000
/** AIF RAM inbound PS data block */
#define AIF_PS_IN_DATA_BLOCK_SIZE   0x400000
/** AIF Packet-switched inbound FIFO size, number of  messages the buffer can hold */
#define AIF_PS_IN_FIFO_SIZE          8
/** AIF Packet-switched inbound FIFO depth
 */
#define AIF_PS_IN_FIFO_DEPTH         AIF2FL_INBOUND_PS_FIFO_EVENT_DEPTH_1


/** AIF FSYNC */
/*assign events. Event 0~9, Event 18~29 is mask events; event 10~17 is count based*/
#define 	AIF_FSYNC_EVENT_LINK0_EDMA 			4
#define 	AIF_FSYNC_EVENT_LINK0_EDMA_DONE	    18
#define 	AIF_FSYNC_EVENT_LINK0_PE_SYNC 		24



/** AIF loopback enable */
#define AIF_CFG_BASE                    (0x01f00000u)
#define AIF_PKTDMA_TX_CHAN_REGION       (AIF_CFG_BASE + 0x00016000u)
#define AIF_PKTDMA_RX_CHAN_REGION       (AIF_CFG_BASE + 0x00018000u)

#define PKTDMA_REG_RX_CHAN_CFG_A    0x000
#define PKTDMA_REG_TX_CHAN_CFG_A    0x000
#define PKTDMA_REG_TX_CHAN_CFG_B    0x004
#define AIF_PKTDMA_GBL_CFG_REGION       (AIF_CFG_BASE + 0x00014000u)

/**
 *  @brief This enum describes the AIF mode supported by the AIF2 Lib
 */
typedef enum {  
                /** WCDMA mode */
                AIF_WCDMA_MODE = 0, \
                /** LTE FDD  mode */
                AIF_LTE_FDD_MODE = 1, \
                /** LTE WCDMA DUAL mode */
                AIF_LTE_WCDMA_MODE = 2, \
                /** Generic packet mode */
                AIF_GENERICPACKET_MODE = 3, \
                /** LTE TDD mode */
                AIF_LTE_TDD_MODE = 4, \
                AIF_MODE_MAX } AIF_Mode;

/**
* @brief This enum describes the AIF2 Sampling rate per link
*/
typedef enum {
			/**  Sampling rate 1.92MHz*/
			AIF_SRATE_1P92MHZ = 0, \
			/**  Sampling rate 3.84MHz*/
			AIF_SRATE_3P84MHZ , \
			/**  Sampling rate 7.68MHz*/
			AIF_SRATE_7P68MHZ , \
			/**  Sampling rate 15.36MHz*/
			AIF_SRATE_15P36MHZ , \
			/**  Sampling rate 23.04MHz*/
			AIF_SRATE_23P04MHZ , \
			/**  Sampling rate 30.72MHz*/
			AIF_SRATE_30P72MHZ , \
			/**  Sampling rate 61.44MHz*/
			AIF_SRATE_61P44MHZ , \
			AIF_SRATE_MAX    }AIF_SampleRate;

/**
 *  @brief This enum describes the AIF types supported by ABT 
 */
typedef enum {  
				/** AIF1 to  AIF2 communication */
                AIF1_2_AIF2 = 0, \
                /** AIF2 to  AIF2 communication */
                AIF2_2_AIF2 = 1, \
                /** AIF2 to  AIF2 communication */
                AIF2_LOOPBACK = 2, \
                /** number of AIF communication type */
                AIF_COM_MAX } AIF_Com_Mode;

/**
 * @brief This enum describes the AIF2 pack mode Interleaving usage for CPRI
*/
typedef enum {
	AIF2_LTE_CPRI_1b1 = 1,
	AIF2_LTE_CPRI_2b2 = 2,
	AIF2_LTE_CPRI_4b4 = 4,
	AIF2_LTE_CPRI_8b8 = 8
} AIF2_PackingMode;

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
	AIF2_LTETDD_ULDL_CFG0 = 0xFD3F4,  //0xF3FCF,//
	AIF2_LTETDD_ULDL_CFG1 = 0x3D0F4,
	AIF2_LTETDD_ULDL_CFG2 = 0x0D034,
	AIF2_LTETDD_ULDL_CFG3 = 0x003F4,
	AIF2_LTETDD_ULDL_CFG4 = 0x000F4,
	AIF2_LTETDD_ULDL_CFG5 = 0x00034,
	AIF2_LTETDD_ULDL_CFG6 = 0x3D3F4
} AIF2_LteTddUlDlCfg;

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
	AIF2_LTETDD_SSF_NCP_CFG0 = 0xD555540,
	AIF2_LTETDD_SSF_NCP_CFG1 = 0xD540000,
	AIF2_LTETDD_SSF_NCP_CFG2 = 0xD500000,
	AIF2_LTETDD_SSF_NCP_CFG3 = 0xD400000,
	AIF2_LTETDD_SSF_NCP_CFG4 = 0xD000000,
	AIF2_LTETDD_SSF_NCP_CFG5 = 0xF555540,
	AIF2_LTETDD_SSF_NCP_CFG6 = 0xF540000,
	AIF2_LTETDD_SSF_NCP_CFG7 = 0xF500000,
	AIF2_LTETDD_SSF_NCP_CFG8 = 0xF400000
} AIF2_LteTddSsfNcpCfg;


/**
 * @brief This enum describes the AIF2 link mode for dual mode usage.
*/
typedef enum {
	WCDMA = 0,
	LTE
} AIF2_LinkMode;

/**
 * @brief This enum describes the Cyclic Prefix type for LTE
*/
typedef enum {
	AIF2_LTE_CPTYPE_NORMAL = 0,
	AIF2_LTE_CPTYPE_EXTENDED = 1
} AIF_LteCpType;


/**
@} */

/** @addtogroup AIF_DATASTRUCT
*
* @{ */

/**
 * @brief Specification of AIF2_DeviceCfgBaseAddr
 *
 * The AIF2_DeviceCfgBaseAddr is used to specify device level configuration
 * to the FL.
 */
typedef struct
{
  void *cfgBase;
  void *serDesB4CfgBase;
  void *serDesB8CfgBase;
} AIF_DeviceCfgBaseAddr;

/**
 * @brief Specification of AIF2_DeviceCfg
 *
 * The AIF2_DeviceCfg is used to specify device level configuration
 * to the FL.
 */
#define AIF_MAX_PERIPHS 1 /**< Maximum peripherals (base addresses) supported by FL */
typedef struct
{
  AIF_DeviceCfgBaseAddr bases[AIF_MAX_PERIPHS]; /**< base addreses */
} AIF_DeviceCfg;

/**
 * @brief Specification of AIF2_InitCfg
 *
 * The AIF2_InitCfg is used to specify configuration to the CSL FL.
 */
typedef struct
{
  AIF_DeviceCfg dev; /**< Device Configuration */
} AIF_InitCfg;

/**
 * @brief This structure contains the parameters for the
 * initialization of a AIF2 link of the TMS320C6670.
 * It also contains a reference to the CSL AIF object instance.
 */
 typedef struct AIF_LinkConfigObj {
                    /** Selects link to enable (0=disable; 1=enable). */
                    uint32_t                   linkEnable;
                    /** Selects AIF link rate (2=2x; 4= 4x ; 5= 5x (CPRI only); 8= 8x). */
                    uint32_t                   linkRate;
                    /** Selects AIF2 sample rates */
                    AIF_SampleRate		 	 sampleRate;
                    /** Selects support for AIF PS messages on this link (OBSAI only, 0=disable; 1=enable) */
                    uint32_t                   psMsgEnable;
                    /** Selects the link data type for outbound burst traffic. */
                    Aif2Fl_LinkDataType     outboundDataType;
                    /** Selects the link data width for outbound burst traffic. */
                    Aif2Fl_DataWidth        outboundDataWidth;
                    /** Selects the link data type for inbound burst traffic. */
                    Aif2Fl_LinkDataType     inboundDataType;
                    /** Selects the link data width for inbound burst traffic. */
                    Aif2Fl_DataWidth        inboundDataWidth;
                    /** Selects AIF legacy mode. */
                    AIF_Com_Mode             comType;
                    /** Selects which AIF2 DIO engine to use for this link */
                    Aif2Fl_DioEngineIndex   dioEngine;
                    /** Holds PE2 offset (set by the library)*/
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
                    /** Maximum expected wait time in ns for:
                     *  PktDMA mode: popping a descriptor and DMA of IQ sample from application memory to AIF egress DB
                     *  DIO mode: TAC processing time (4 chip) and DMA time from TAC or application memory to AIF egress DB
                     *  This parameter is used by calcAifTimingForTxNode() to compute deltaOffset.
                     *  It actually corresponds to PE1 offset in AIF2 user's guide expressed in ns
                     */
                    uint32_t                   txWait;
                    /** Holds the byte clock expressed in KHz. (set by the library)*/
                    uint32_t                   byteClockInKHz;
                    /** Holds the number of streams on this link. (set by the library)*/
                    uint32_t                   numPeAxC;
                    /** Holds the first AxC carried on this link. (set by the library)*/
                    uint32_t                   firstPdAxC;
                    /** Holds the number AxC carried on this link. (set by the library)*/
                    uint32_t                   numPdAxC;
                    /** Holds first DB channel transmit path for this link (set by the library)*/
                    uint32_t					 firstPeDBCH;
                    /** Holds first DB channel receive path for this link (set by the library)*/
                    uint32_t					 firstPdDBCH;
                    /** Selects the interleaving pattern mode of AxCs for ingress CPPI for CPRI - LTE only*/
                    AIF2_PackingMode 		 cpriPackMode;
                    /** Selects the AxC's mask pattern mode for CPRI - LTE only*/
                    uint32_t					 maskPattern[16];
                    /** Enable the re-transmission for the associated link*/
                    uint32_t                   RtEnabled;
                    /** select the link (0 to 5) for the re-transmission*/
                    Aif2Fl_LinkIndex        RtLinkRout;
                    /** Choose the mode of the link (LTE or WCDMA). Use for dual mode only.*/
                    AIF2_LinkMode			 mode;
                    /** Specifies the UL/DL configuration in case of LTE TDD for up to 16 AxCs per link.*/
                    AIF2_LteTddUlDlCfg		 lteTddUlDlCfg[16];
                    /** Specifies the Special sub-frame configuration in case of LTE TDD for up to 16 AxCs per link.*/
                    AIF2_LteTddSsfNcpCfg	 lteTddSsfNcpCfg[16];
					/** Hold the Cpri8Word offset value for Ingress AxC per link */
					uint32_t					 cpri8WordOffset[32];
					/** Select wheter this link is doing multi rate or not */
					uint16_t					 multiRate;
                    } AIF_LinkConfigObj, *AIF_LinkConfigHandle;

/**
 * @brief This structure contains the parameters for the
 * initialization of a AIF2 link of the TMS320C6670.
 * It also contains a reference to the CSL AIF object instance.
 */
 typedef struct AIF_DioConfigObj {
                    /** Selects buffer start address for this DIO engine in egress direction */
                    uint32_t*                  out;
                    /** Selects number of DMA blocks (wrap2) for this DIO engine in egress direction */
                    uint32_t                   outNumBlock;
                    /** Selects buffer start address for this DIO engine in ingress direction */
                    uint32_t*                  in;
                    /** Selects number of DMA blocks (wrap2) for this DIO engine in ingress direction */
                    uint32_t                   inNumBlock;
                    /** Holds number link associated to this DIO engine (set by the library) */
                    uint8_t                    numLink;
                    /** Holds first link associated to this DIO engine (set by the library) */
                    uint8_t                    firstLink;
                    /** Holds offset to the first DB channel associated to this DIO engine on transmit side(set by the library) */
                    uint32_t                   offsetPeDBCH;
                    /** Holds offset to the first DB channel associated to this DIO engine on receive side(set by the library) */
                    uint32_t                   offsetPdDBCH;
                    /** Holds number of DB channels associated to this DIO engine on transmit path(set by the library) */
                    uint32_t                   numPeDBCH;
                    /** Holds number of DB channels associated to this DIO engine on receive path (set by the library) */
                    uint32_t                   numPdDBCH;
                    /** Determine if we use the DIO duplicate feature with the current dio (0= disable, 1= enable).
                     *  Duplicate feature uses dio engine 2, so this feature cannot be used on both DIO 0 and DIO 1 at the same time
                     *  This duplicate feature can also be used for the purpose of debugging DIO traffic going to RAC */
                    uint32_t                   duplicateOnDioEng2;
                    /** Specifies the mode of the link (LTE or WCDMA). Use for dual mode only.*/
                    AIF2_LinkMode			   mode;
                    /** When in Wcdma mode, tells whether this DIO engine is used for RAC on ingress side */
                    uint32_t				   usedWithRAC;
                    /** Selects AIF2 sample rates */
					AIF_SampleRate		 	   sampleRate;
                    } AIF_DioConfigObj, *AIF_DioConfigHandle;

/**
 * @brief This structure contains the parameters for the
 * PKTDMA and QMSS setup.
 */
 typedef struct AIF_PktDmaConfigObj {
                    /** Holds AIF2 CPPI handle returned by the CPPI library   (set by the library) */
                    Cppi_Handle              hCppi;
	 	 	 	 	/** */
	 	 	 	 	Qmss_MemRegion           txRegionAxC[124];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                   txNumDescAxC[124];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                   txDescSizeAxC[124];
	 	 	 	 	/** */
	 	 	 	 	Qmss_MemRegion           rxRegionAxC[124];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                   rxNumDescAxC[124];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                   rxDescSizeAxC[124];
                    /** Points to a table describing the Rx flow for each AxC in use  */
                    Cppi_RxFlowCfg*          hRxFlowAxC[124];
                    /** */
                    Qmss_QueueHnd            txFqAxC[124];
                    /** */
                    Qmss_QueueHnd            txQAxC[124];
                    /** */
                    Qmss_QueueHnd            rxFqAxC[124];
                    /** */
                    Qmss_QueueHnd            rxQAxC[124];
                    /** */
                    Cppi_ChHnd               txChAxC[124];
                    /** */
                    Cppi_ChHnd               rxChAxC[124];
	 	 	 	 	/** */
	 	 	 	 	Qmss_MemRegion           txRegionCtrl[4];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                   txNumDescCtrl[4];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                   txDescSizeCtrl[4];
	 	 	 	 	/** */
	 	 	 	 	Qmss_MemRegion           rxRegionCtrl[4];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                   rxNumDescCtrl[4];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                   rxDescSizeCtrl[4];
                    /** Points to a table describing the Rx flow for control channels - only one for now */
                    Cppi_RxFlowCfg*          hRxFlowCtrl[4];
                    /** */
                    Qmss_QueueHnd            txFqCtrl[4];
                    /** */
                    Qmss_QueueHnd            txQCtrl[4];
                    /** */
                    Qmss_QueueHnd            rxFqCtrl[4];
                    /** */
                    Qmss_QueueHnd            rxQCtrl[4];
                    /** */
                    Cppi_ChHnd               txChCtrl[4];
                    /** */
                    Cppi_ChHnd               rxChCtrl[4];
                    /** */
                    uint32_t*                  rxDataBuff[6];
                    /** */
                    uint32_t*                  txDataBuff[6];
					/** */
					Cppi_ChHnd               dioTxChAxC;                 
					/** */
					Cppi_ChHnd               dioRxChAxC;
					/** */
					uint32_t 					 firstAxC;
					/** */
					uint32_t 					 numAxC;
					/** */
					AIF_Mode 				 mode;
					/** */
					uint32_t					 psMsgEnable;
                    } AIF_PktDmaConfigObj, *AIF_PktDmaConfigHandle;

/**
 * @brief This structure contains the parameters for the
 * hfn sync state. It could be used to detect when sync is
 * established (syncStatusChange = 1 and hfnsyncState = 1) or when
 * sync is lost (syncStatusChange = 1 and hfnsyncState = 0)
 * The context is in the AIF_getException processing function where below
 * parameters are updated before clearing the exceptions.
 */
 typedef struct AIF_HfnSyncObj {
					/** 1 (true) if sync status changed, 0 otherwise It is basically the rm_ee_sync_status_change_err in EE link A */
					uint16_t syncStatusChange;
					/** if statusChanged above is 1, then indicates whether in hfnsync i.e state ST3 (= 1) or not (=0).
						It is basically the rm_ee_hfnsync_state_err in EE link A */
 					uint16_t hfnsyncState;
					} AIF_HfnSyncObj;
 
/**
 * @brief This structure contains the pointers to the AIF2 exception counters.
 */
 typedef struct AIF_EeCountObj {
					/** Holds counters for aif2 EE DB Interrupt data information */
                    Aif2Fl_EeDbInt    eeDbIntCnt;
                    /** Holds counters for aif2 EE Link A Interrupt data information */
                    Aif2Fl_EeLinkAInt eeLinkAIntCnt[AIF_MAX_NUM_LINKS];
                    /** Holds counters for aif2 EE Link B Interrupt data information */
                    Aif2Fl_EeLinkBInt eeLinkBIntCnt[AIF_MAX_NUM_LINKS];
                    /** Holds counters for aif2 EE AD Interrupt data information */
                    Aif2Fl_EeAdInt    eeAdIntCnt;
                    /** Holds counters for aif2 EE CD(PKTDMA module) Interrupt data information */
                    Aif2Fl_EeCdInt    eeCdIntCnt;
                    /** Holds counters for aif2 EE SD Interrupt data information */
                    Aif2Fl_EeSdInt    eeSdIntCnt;
                    /** Holds counters for aif2 EE VC Interrupt data information*/
                    Aif2Fl_EeVcInt    eeVcIntCnt;
                    /** Holds counters for aif2 EE AT Interrupt data information */
                    Aif2Fl_EeAtInt    eeAtIntCnt;
                    /** Holds counters for aif2 EE PD Interrupt data information */
                    Aif2Fl_EePdInt    eePdIntCnt;
                    /** Holds counters for aif2 EE PE Interrupt data information */
                    Aif2Fl_EePeInt    eePeIntCnt;
                    /** Holds captured Pi if Pi is detected as being out of the window */
                    uint32_t             piCaptured[AIF_MAX_NUM_LINKS];
                    /** Holds a flag telling whether any of the enabled exceptions occurred, 0 if none, 1 if any */
                    uint32_t			   eeFlag;
					/** Holds hfnsync information */
                    AIF_HfnSyncObj    eeHfnsync[AIF_MAX_NUM_LINKS];
 	 	 	 	 	} AIF_EeCountObj, *AIF_EeCountHandle;


/**
 * @brief This structure contains the parameters for the
 * AIF2 Data Trace setup.
 */
 typedef struct AIF_DataTraceObj {
	 	 	 	 	/** Selects 1 of 6 links of raw, not gated, RM output data used for data trace feature */
	 	 	 	 	Aif2Fl_LinkIndex   linkIndex;
	 	 	 	 	/** Selects whether Trace Data Capture is Enabled */
					uint32_t              dataCaptureEnable;
					/** Selects whether Trace Framing Data Capture is Enabled */
					uint32_t              framingDataCaptureEnable;
					/** Selects whether Data Trace Sync is Enabled */
					uint32_t              syncCaptureEnable;
                    /** Selects buffer start address for Data and Framing Data capture */
                    uint32_t*             dataAddress;
                    /** Selects buffer start address for Data and Framing Data capture */
                    uint32_t*             framingDataAddress;
                    /** Selects Data Trace DMA burst wrap value
                     *  Burst size is 64 bytes for data and 16 bytes for framing data
                     */
                    uint32_t              dtWrap;
					} AIF_DataTraceObj, *AIF_DataTraceHandle;


/**
 * @brief This structure contains the parameters for the
 * LTE specific radio timer configuration.
 */
 typedef struct AIF_LteRadTimerObj {
					/** Holds the cyclic prefix type */
					AIF_LteCpType			 cpType;
					/** Determines the TC array values and number of entries (lutindex). The valid range for this is 1 through 7 or 6 depending on CP type. */
					uint32_t					 numSymbolsForSymbolStrobe;
					/** Determines the symbolTC, so number of symbols per frame */
					uint32_t					 numSymbolStrobesForFrameStrobe;
					} AIF_LteRadTimerObj, *AIF_LteRadTimerHandle;

/**
 * @brief This structure contains the parameters for the
 * AIF2 radio timer configuration.
 */
 typedef struct AIF_RadTimerObj {
					/** Flag to indicate this configuration is to override the LLD default one */
	 	 	 	 	uint16_t 					 userSpecified;
	 	 	 	 	/** Holds the init clock number value for this radio timer */
	 	 	 	 	uint32_t       			 initClockNum;
	 	 	 	 	/** Holds the init symbol number value for this radio timer */
	 	 	 	 	uint8_t         			 initSymbolNum;
	 	 	 	 	/** Holds the init frame lsb number value for this radio timer */
	 	 	 	 	uint32_t       			 initFrameLsbNum;
	 	 	 	 	/** Holds the init frame msb number value for this radio timer */
	 	 	 	 	uint32_t       			 initFrameMsbNum;
	 	 	 	    /** Determines this RAD timer frame terminal count */
	 	 	 	    uint32_t       			 frameTerminalCount;
	 	 	 	 	/** Holds LTE specific parameters for radio timer terminal count values  */
	 	 	 	    AIF_LteRadTimerObj       lte;
					} AIF_RadTimerObj, *AIF_RadTimerHandle;

/**
 * @brief This structure contains specific parameters for a
 * given AIF2 antenna container (AxC).
 */
 typedef struct AIF_AxCObj {
     	 	 	 	/** Holds the Egress Buffer depth for this AxC */
					 Aif2Fl_DbFifoDepth		 egressBufDepth;
					 /** Holds the Ingress Buffer depth for this AxC */
					 Aif2Fl_DbFifoDepth		 ingressBufDepth;
					 /** Holds the PE AxC offset value for this AxC */
					 uint32_t 					 peAxCOffset;
					 /** Holds the PD AxC offset value for this AxC */
					 uint32_t 					 pdAxCOffset;
					 /** Holds the LTE cyclic prefix type for this AxC*/
					 AIF_LteCpType			     lteCpType;
					 /** Used for LTE for group creation to facilitate MBSFN operation. Normally an egress group 
						is required to be created for a unique combination of sampling rate and cpType. But for different cellIds,
						independence is required, so different groups will be created and assigned for different cell Ids even
						if their sampling rates and cptypes are identical. */
					uint32_t egressLTEcellId;
                    /** Selects the interleaving pattern mode of AxCs for ingress CPPI for CPRI - LTE only*/
                    AIF2_PackingMode 		 cpriPackMode;
                    /** Selects the AxC's mask pattern mode for CPRI - LTE only*/
                    uint32_t					 maskPattern;
                    /** Selects AIF2 sample rates */
                    AIF_SampleRate		 	 sampleRate;
					} AIF_AxCObj, *AIF_AxCHandle;

/**
 * @brief This structure contains the parameters for the
 * initialization of a AIF HW module of the TMS320C6670.
 * it includes configuration objects for all the links.
 */
 typedef struct AIF_ConfigObj {
                    /** Holds pointer to Functional Layer AIF object instance. (set by the library) */
                    Aif2Fl_Handle           hFl;
                    /** Holds pointer to AIF2 configuration structure (set by the library) */
                    Aif2Fl_Setup           *hAif2Setup;
                    /** Selects CPRI or OBSAI for AIF. */
                    Aif2Fl_LinkProtocol 	 protocol;
                    /**  Selects the operating mode for this link (WCDMA, LTE, ABT) */
                    AIF_Mode			     mode;
                    /** Selects AIF2 SerDes input clock in KHz. */
                    uint32_t                   aif2ClkSpeedKhz;
                    /** Selects whether to transport Antenna samples with DIO engine or Packet DMA */
                    Aif2Fl_CppiDio 		 pktdmaOrDioEngine;
                    /** Holds configuration structure for each link  */
                    AIF_LinkConfigObj        linkConfig[AIF_MAX_NUM_LINKS];
                    /** Holds configuration structure for each DIO engine */
                    AIF_DioConfigObj         dioConfig[AIF2_MAX_NUM_DIO_ENGINE];
                    /** Holds configuration structure for each PktDMA channel */
                    AIF_PktDmaConfigObj      pktDmaConfig;
                    /** Selects the sync type for the AIF2 AT timer  */
                    Aif2Fl_AtSyncSource     aif2TimerSyncSource;
					/** phyT Compare value if aif2TimerSyncSource was PHYT_CMP_SYNC */
					uint32_t                   phytCompValue;
					/** Auto resync if new frame boundary is received */
					Aif2Fl_AtReSyncMode     autoResyncMode;
                    /** Exception counters - updated when exception are enabled */
                    AIF_EeCountObj           aif2EeCount;
                    /** Use the super packet workaround */
                    uint16_t   					 superPacket;
                    /** Holds specific configuration structure for each AxC */
                    AIF_AxCObj				 AxCconfig[AIF_MAX_NUM_AXC];
                    /** Holds specific configuration structure for each radio timer (RADT only on AIF2) */
					AIF_RadTimerObj       	 radTimerConfig[AIF_MAX_NUM_RADT];
					/** (ARM Linux specific) Holds a pointer to the Device specific configuration used to pass configuration register base address */
					AIF_InitCfg             *hAif2SerDesBaseAddr;
                    } AIF_ConfigObj, *AIF_ConfigHandle;
                    

/**
@} */


#ifdef __cplusplus
}
#endif


#endif //__AIF_DEFS_H

/** @} */ // end of module additions
