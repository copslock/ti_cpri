/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 2009
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


/** @defgroup AIF2FL_API AIF2FL
 *
 * @section Introduction
 *
 * @subsection xxx Purpose and Scope
 * The purpose of this document is to identify a set of common CSL APIs for
 * the AIF2 module across various devices. The CSL developer is expected to
 * refer to this document while designing APIs for these modules. Some of the
 * listed APIs may not be applicable to a given AIF module. While other cases
 * this list of APIs may not be sufficient to cover all the features of a
 * particular AIF2 Module. The CSL developer should use his discretion designing
 * new APIs or extending the existing ones to cover these.
 *
 * @subsection aaa Terms and Abbreviations
 *   -# CSL:  Chip Support Library
 *   -# API:  Application Programmer Interface
 *
 *
 */

/** @file aif2fl.h
 *
 *  @brief Header file for functional layer of AIF2
 *
 *  Description
 *  - The different symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 *  @date    03 Jun, 2009
 *  @author  Albert Bae
 *  @version 0.0.1
 *
 * =============================================================================
 * Revision History
 * ===============
 *  03-Jun-2009   Albert   File Created.
 *  06-June-2015  Seb      File imported in the driver
 *
 */

/**
 * @defgroup AIF2FL_DATASTRUCT AIF2FL Data Structures
 * @ingroup AIF2FL_API
 */
/**
 * @defgroup AIF2FL_ENUM  AIF2FL Enumerated Data Types
 * @ingroup AIF2FL_API
 */
/**
 * @defgroup AIF2FL_FUNCTION  AIF2FL Functions
 * @ingroup AIF2FL_API
 */

#ifndef _AIF2FL_H_
#define _AIF2FL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <ti/csl/csl.h>
#include <ti/csl/cslr_aif2.h>
#include <ti/csl/soc.h>

#if (defined DEVICE_K2H) || (defined DEVICE_K2K)
#ifndef K2
#define K2
#endif
#endif

/**************************************************************************\
* AIF2 global macro declarations
\**************************************************************************/

/** @addtogroup AIF2FL_ENUM
*
* @{ */

/**
 *
 * @brief aif2fl return and error codes
 *
 *  */
typedef enum
{
    /** Action not supported by AIF2FL */
    AIF2FL_NOTSUPPORTED = -9,
    /** Invalid query to AIF2FL */
    AIF2FL_INVQUERY = -8,
    /** Invalid command to AIF2FL */
    AIF2FL_INVCMD = -7,
    /** Invalid parameters passed to AIF2FL */
    AIF2FL_INVPARAMS = -6,
    /** Handle passed to AIF2FL was invalid */
    AIF2FL_BADHANDLE = -5,
    /** Encoutered AIF2 system resource overflow */
    AIF2FL_OVFL = -4,
    /** Unused code */
    AIF2FL_UNUSED = -3,
    /** AIF2 Peripheral resource is already in use */
    AIF2FL_INUSE = -2,
    /** AIF2 Generic Failure */
    AIF2FL_FAIL = -1,
    /** AIF2FL successful return code */
    AIF2FL_SOK = 1
} Aif2Fl_Status;


/**
 * @brief Frame model supported
 *
 * Use this symbol to specify frame mode for AIF2
 *  */
typedef enum 
{  /** Selects the Normal frame mode */
   AIF2FL_FRAME_MODE_NORMAL = 0,
   /** Selects the Short frame mode */
   AIF2FL_FRAME_MODE_SHORT

} Aif2Fl_FrameMode;

/**
 *
 * @brief Link Protocol supported
 *
 * Use this symbol to specify link formats for AIF2
 *  */
typedef enum 
{  /** Selects the CPRI protocol */
   AIF2FL_LINK_PROTOCOL_CPRI = 0,
   /** Selects the OBSAI protocol */
   AIF2FL_LINK_PROTOCOL_OBSAI

} Aif2Fl_LinkProtocol;

/**
 *
 * @brief  data width format supported
 *
 * Use this symbol to specify DL/UL and Generic data formats for AIF2
 *  */
typedef enum 
{
   /** Selects 7bit data width */ 
   AIF2FL_DATA_WIDTH_7_BIT = 0,
   /** Selects 8bit data width */ 
   AIF2FL_DATA_WIDTH_8_BIT,
   /** Selects 15bit data width */ 
   AIF2FL_DATA_WIDTH_15_BIT,
   /** Selects 16bit data width */ 
   AIF2FL_DATA_WIDTH_16_BIT

} Aif2Fl_DataWidth;


/**
 *
 * @brief aif2 link indices supported
 *
 * Use this symbol to specify the aif2 link index
 *  */
typedef enum 
{
   /** Selects link0 */
   AIF2FL_LINK_0  = 0,
   /** Selects link1 */
   AIF2FL_LINK_1,
   /** Selects link2 */
   AIF2FL_LINK_2,
   /** Selects link3 */
   AIF2FL_LINK_3,
   /** Selects link4 */
   AIF2FL_LINK_4,
   /** Selects link5 */
   AIF2FL_LINK_5,
   /** Selects no link */
   AIF2FL_NO_LINK = 0xF

} Aif2Fl_LinkIndex;



/**
 *
 * @brief  link rates supported
 *
 * Use this symbol to specify the  link rate
 *  */
typedef enum 
{
   /** Selects 8X link rate */
   AIF2FL_LINK_RATE_8x =0,
   /** Selects 4X link rate */
   AIF2FL_LINK_RATE_4x,
   /** Selects 2X link rate */
   AIF2FL_LINK_RATE_2x,
   /** Selects 5X link rate only for CPRI*/
   AIF2FL_LINK_RATE_5x
   
} Aif2Fl_LinkRate;



/**
 *
 * @brief TM states
 *
 * Use this symbol to specify the state of the TM state machine
 * */
typedef enum 
{
   /** Selects the TM state Off */	 
   AIF2FL_TM_ST_OFF =  0x1,
   /** Selects the TM state idle */   
   AIF2FL_TM_ST_IDLE =  0x2,
   /** Selects the TM state re-sync */
   AIF2FL_TM_ST_RE_SYNC =  0x4,
   /** Selects the TM state frame sync */
   AIF2FL_TM_ST_FRAME_SYNC =  0x8

} Aif2Fl_TmSyncState;




/**
 *
 * @brief setup Rm fifo threshold word size for reading received data 
 *
 * 
 * */
typedef enum 
{
   /** FIFO starts reading immediately  */	
   AIF2FL_RM_FIFO_THOLD_IMMEDIATELY =  0,
   /** FIFO starts reading after 4 dual words received   */
   AIF2FL_RM_FIFO_THOLD_4DUAL,
   /** FIFO starts reading after 8 dual words received   */
   AIF2FL_RM_FIFO_THOLD_8DUAL,
   /** FIFO starts reading after 16 dual words received   */
   AIF2FL_RM_FIFO_THOLD_16DUAL

} Aif2Fl_RmFifoThold;


/**
 *
 * @brief Suppress error reporting when the receiver state machine is not in state ST3 
 *
 * 
 *  */
typedef enum 
{
   /** Allow all RM error reporting when not in ST3  */	
   AIF2FL_RM_ERROR_ALLOW =  0,
 
   /** Suppress all RM error reporting when not in ST3  */
   AIF2FL_RM_ERROR_SUPPRESS

} Aif2FlRmErrorSuppress;



/**
 *
 * @brief RM sync states
 *
 * Use this symbol to specify the state of the RM state machine
 * */
typedef enum 
{
   /** Selects the RM state 0 */	
   AIF2FL_RM_ST_0 =  8,
   /** Selects the RM state 1 */
   AIF2FL_RM_ST_1 =  4,
   /** Selects the RM state 2 */
   AIF2FL_RM_ST_2 =  2,
   /** Selects the RM state 3 */
   AIF2FL_RM_ST_3 =  1,
   /** Selects the RM state 4 */
   AIF2FL_RM_ST_4 =  16,
   /** Selects the RM state 5 */
   AIF2FL_RM_ST_5 =  32

} Aif2Fl_RmSyncState;


/**
 *
 * @brief Link data type supported
 *
 * Use this symbol to specify the type of data on inbound/outbound link
 * */
typedef enum 
{
   /** Selects the link data type as downlink, generic */
   AIF2FL_LINK_DATA_TYPE_NORMAL = 0,
   /** Selects the link data type as uplink */
   AIF2FL_LINK_DATA_TYPE_RSA

} Aif2Fl_LinkDataType;


/**
 *
 * @brief GSM data type supported
 *
 * Use this symbol to specify the GSM type of data on inbound link
 * */
typedef enum 
{
   /** Not GSM UL  */
   AIF2FL_GSM_DATA_OTHER = 0,
   /** GSM UL, has special OBSAI Time Stamp implications. UL time stamp format msb=1 first four msg  */
   AIF2FL_GSM_DATA_UL

} Aif2Fl_GSMDataFormat;


/**
 *
 * @brief Retransmitter Mode used
 *
 * Use this symbol to specify the retransmitter mode
 * */

typedef enum
{
   /** RT  takes RM input only  */
   AIF2FL_RT_MODE_RETRANSMIT, 
   /** Aggregate or Merge data from PE and CI */ 
   AIF2FL_RT_MODE_AGGREGATE, 
    /** RT takes PE input only*/ 
   AIF2FL_RT_MODE_TRANSMIT  

} Aif2Fl_RtConfig;



/**
 *
 * @brief Sd module index
 *
 * Use this symbol to specify the Sd Rx invert polarity
 * */
typedef enum
{
    /** Selects the Receive pair normal polarity*/
    AIF2FL_SD_RX_NORMAL_POLARITY = 0,
    /** Selects the Receive pair inverted polarity*/
    AIF2FL_SD_RX_INVERTED_POLARITY
    
} Aif2Fl_SdRxInvertPolarity;


/**
 *
 * @brief Sd module rx alignment
 *
 * 
 * */
typedef enum
{
    /** No symbol alignment will be performed whilst this setting is selected, or when switching to this selection from another*/
    AIF2FL_SD_RX_ALIGNMENT_DISABLE = 0,
    /** Symbol alignment will be performed whenever a misaligned comma symbol is received. */
    AIF2FL_SD_RX_COMMA_ALIGNMENT_ENABLE,
    /** The symbol alignment will be adjusted by one bit position when this mode is selected */
    AIF2FL_SD_RX_ALIGNMENT_JOG

}Aif2Fl_SdRxAlign;



/**
 *
 * @brief Sd module rx LOS
 *
 * 
 * */
typedef enum
{
    /** Loss of signal detection disabled*/
    AIF2FL_SD_RX_LOS_DISABLE = 0,
    /** Loss of signal detection enabled. */
    AIF2FL_SD_RX_LOS_ENABLE = 4

}Aif2Fl_SdRxLos;


#ifdef K2
/**
 *
 * @brief Sd sys clock select from either B8 or B4
 *
 * */
typedef enum 
{
    /* Select TX Byte clock from B8, B4 default */
    AIF2FL_SD_BYTECLOCK_FROM_B8_B4_DEFAULT = 0,
    /* Select TX Byte clock from B8 SERDES */
    AIF2FL_SD_BYTECLOCK_FROM_B8,
    /* Select TX Byte clock from B4 SERDES */
    AIF2FL_SD_BYTECLOCK_FROM_B4,
    /* Select TX Byte clock from both B4 and B8 */
    AIF2FL_SD_BYTECLOCK_FROM_B8_B4

}Aif2Fl_SdClockSelect;
#else
/**
 *
 * @brief Sd sys clock select from either B8 or B4
 *
 * */
typedef enum
{
    /* Select TX Byte clock 0 from B8 SERDES */
	AIF2FL_SD_BYTECLOCK_FROM_B8 = 0,
    /* Select TX Byte clock 0 from B4 SERDES */
	AIF2FL_SD_BYTECLOCK_FROM_B4

}Aif2Fl_SdClockSelect;

#endif


/**
 *
 * @brief CRPI Control Word 4B/5B encoding enable
 *
 * 
 * */
typedef enum 
{
   /** User defined data is not extracted from CPRI Control Words   */
   AIF2FL_CW_DELIM_NO_CW = 0,
   /** CW user data is delimitted using 4B/5B encoding where SOP & EOP are defined  */
   AIF2FL_CW_DELIM_4B5B,
   /** CW user data is delimitted by a user defined Null Delimiter (defined in other MMR field)   */
   AIF2FL_CW_DELIM_NULLDELM,
   /** Packet Boundaries are inferred to be on hyper-frame boundaries  */
   AIF2FL_CW_DELIM_HYP_FRM

} Aif2Fl_CpriCwPktDelim;



/**
 *
 * @brief dicates the payload is to be used as AxC (normal) or Packet traffic
 *
 * 
 *  */
typedef enum 
{
   /** Use  AxC data  */
   AIF2FL_PD_DATA_AXC = 0,
   /** Use  Packet data  */
   AIF2FL_PD_DATA_PKT

} Aif2Fl_PdDataMode;

/**
 *
 * @brief select mode between CPPI and DIO
 *
 * 
 * */
typedef enum 
{
   /** Use CPPI mode  */
   AIF2FL_CPPI = 0,
   /** Use DIO for AxC data  */
   AIF2FL_DIO

} Aif2Fl_CppiDio;


/**
 *
 * @brief Report every missed WDog fail, or report only fails of missing EOP
 *
 * 
 * */
typedef enum 
{
   /** WD Report all  */
   AIF2FL_PD_WD_REPORT_ALL = 0,
   /** WD report EOP only  */
   AIF2FL_PD_WD_REPORT_EOP

} Aif2Fl_PdWatchDogReport;


/**
 *
 * @brief PD or PE Obsai time stamp mask supported
 *
 * */
typedef enum 
{
   /** All TS bits are genrated, not inserted */
   AIF2FL_TSTAMP_MASK_FULL_GEN = 0,
   /** 4 lsb bits of TS are inserted, 2 msb are generated */
   AIF2FL_TSTAMP_MASK_4INS_2GEN,
   /** All TS(5:0) bits are inserted */
   AIF2FL_TSTAMP_MASK_FULL_INS

} Aif2Fl_ObsaiTsMask;


/**
 *
 * @brief PD or PE time stamp format supported
 *
 * */
typedef enum 
{
   /** Selects the link time stamp format as no time stamp check */
   AIF2FL_TSTAMP_FORMAT_NO_TS = 0,
   /** Selects the link time stamp format as normal time stamp */
   AIF2FL_TSTAMP_FORMAT_NORM_TS,
   /** Selects the link time stamp format as GSM OBSAI time stamp (UL time stamp for PE)  */
   AIF2FL_TSTAMP_FORMAT_GSM,
   /** Selects the link time stamp format for generic packet (SOP = 10, MOP = 00, EOP = 11) */
   AIF2FL_TSTAMP_FORMAT_GEN_PKT,
   /** Selects the link time stamp format for ethernet */
   AIF2FL_TSTAMP_FORMAT_ETHERNET,
   /** Selects the link time stamp format, which value is checked by PD or PE route  */
   AIF2FL_TSTAMP_FORMAT_ROUTE_CHECK,
    /** Selects the link time stamp format as GSM DL OBSAI time stamp (only used for PE)  */
   AIF2FL_TSTAMP_FORMAT_GSM_DL

} Aif2Fl_TstampFormat;

/**
 *
 * @brief controls how many OBSAI time stamp bits to use in the reception routing.
 *
 * Use this symbol to specify the masking value for time stamp
 * */
typedef enum 
{
   /** Do not use the TS field for routing  */
   AIF2FL_ROUTE_MASK_NONE = 0,
   /** 4 lsb bits: Use TS(3:0)  */
   AIF2FL_ROUTE_MASK_4LSB,
   /** all Use TS(5:0)   */
   AIF2FL_ROUTE_MASK_ALL,
   /** Reserved */
   AIF2FL_ROUTE_MASK_RESERVED

} Aif2Fl_RouteMask;

/**
 *
 * @brief select CPRI AxC data pack type
 *
 *  */
typedef enum 
{
   /** AxC data is 7bit I & 7bit Q (byte packing needed)  */
   AIF2FL_CPRI_7BIT_SAMPLE = 0,
   /** AxC data is 8bit I & 8bit Q (byte aligned)  */
   AIF2FL_CPRI_8BIT_SAMPLE,
   /** AxC data is 15bit I & 15bit Q (byte packing needed) */
   AIF2FL_CPRI_15BIT_SAMPLE,
   /** AxC data is 16bit I & 16bit Q (byte aligned) */
   AIF2FL_CPRI_16BIT_SAMPLE

} Aif2Fl_CpriAxCPack;

/**
 *
 * @brief Controls RT to perform appropriate insterion/aggregation into PHY 
 *
 * Use this symbol to specify RT control for PE
 * */
typedef enum 
{
   /** Retransmit mode */
   AIF2FL_PE_RT_RETRANS = 0,
   /** PE Insert mode */
   AIF2FL_PE_RT_INSERT,
   /** aggregate 16 bit mode */
   AIF2FL_PE_RT_ADD16,
   /** aggregate 8 bit mode  */
   AIF2FL_PE_RT_ADD8

} Aif2Fl_PeRtContol;

/**
 *
 * @brief CRC: length of CRC 
 *
 * Use this symbol to specify crc length for PE,PD
 * */
typedef enum 
{
   /** CRC 32 bits */
   AIF2FL_CRC_32BIT = 0,
   /** CRC 16 bits  */
   AIF2FL_CRC_16BIT,
   /** CRC 8 bits   */
   AIF2FL_CRC_8BIT

} Aif2Fl_CrcLen;



/**
 *
 * @brief DB FIFO bufffer depth
 *
 * Use this symbol to specify the buffer depth for DB FIFO
 * */
typedef enum
{
   /** Selects  FIFO buffer depth to 8 quad words */
   AIF2FL_DB_FIFO_DEPTH_QW8 = 0,
   /** Selects  FIFO buffer depth to 16 quad words */
   AIF2FL_DB_FIFO_DEPTH_QW16,
   /** Selects  FIFO buffer depth to 32 quad words */
   AIF2FL_DB_FIFO_DEPTH_QW32,
   /** Selects  FIFO buffer depth to 64 quad words */
   AIF2FL_DB_FIFO_DEPTH_QW64,
   /** Selects  FIFO buffer depth to 128 quad words */
   AIF2FL_DB_FIFO_DEPTH_QW128,
   /** Selects  FIFO buffer depth to 256 quad words */
   AIF2FL_DB_FIFO_DEPTH_QW256

} Aif2Fl_DbFifoDepth;

/**
 *
 * @brief DB Big endian swapping control
 *
 * 
 * */
typedef enum 
{
   /** Selects DB no swap */
   AIF2FL_DB_NO_SWAP = 0,
   /** Selects DB byte swap */
   AIF2FL_DB_BYTE_SWAP,
   /** Selects DB half word swap */
   AIF2FL_DB_HALF_WORD_SWAP,
   /** Selects DB word swap */
   AIF2FL_DB_WORD_SWAP
   
} Aif2Fl_DbDataSwap;


/**
 *
 * @brief Internally changing IQ data order control 
 *
 * 
 * */
typedef enum 
{
   /** Selects DB IQ data no swap */
   AIF2FL_DB_IQ_NO_SWAP = 0,
   /** Selects DB IQ data no swap1 */
   AIF2FL_DB_IQ_NO_SWAP1,
   /** Selects DB IQ data byte swap */
   AIF2FL_DB_IQ_BYTE_SWAP,
   /** Selects DB IQ data 16 bit swap */
   AIF2FL_DB_IQ_16BIT_SWAP
   
} Aif2Fl_DbIqOrder;


/**
 *
 * @brief DB DIO length type 
 *
 * Use this symbol to specify DIO length type for DB
 * */
typedef enum 
{
   /** Selects DB dio buffer length 128 bytes */
   AIF2FL_DB_DIO_LEN_128 = 0,
   /** Selects DB dio buffer length 256 bytes */
   AIF2FL_DB_DIO_LEN_256

} Aif2Fl_DioLen;


/**
 *
 * @brief DB Packet mode control type 
 *
 * Use this symbol to specify PM control for DB
 * */
typedef enum 
{
   /** Put PM tokens from PE in separate PM Token FIFO */
   AIF2FL_DB_PM_TOKEN_FIFO = 0,
   /** Put PM tokens from PE in Axc Token FIFO to improve CPRI packet performance */
   AIF2FL_DB_AXC_TOKEN_FIFO

} Aif2Fl_DbPmControl;


/**
 *
 * @brief This field tells how the Ingress Scheduler handles packets marked as failed by the PD. 
 *
 * */
typedef enum 
{
   /** Selects AD error data drop */
   AIF2FL_AD_DROP = 0,
   /** Selects AD error data mark */
   AIF2FL_AD_MARK

} Aif2Fl_AdFailMode;

/**
 *
 * @brief AD Ingress Scheduler aribitration priority
 *
 * */
typedef enum 
{
   /** Set dio as high priority  */
   AIF2FL_AD_DIO_PRI = 0,
   /** Set cppi packet  as high priority */
   AIF2FL_AD_PKT_PRI

} Aif2Fl_AdIngrPriority;


/**
 *
 * @brief AD Egress Scheduler aribitration priority
 *
 * */
typedef enum 
{
   /** Set AxC data as high priority  */
   AIF2FL_AD_AXC_PRI = 0,
   /** Set non AxC data as high priority */
   AIF2FL_AD_NON_AXC_PRI

} Aif2Fl_AdEgrPriority;


/**
 *
 * @brief AD DIO Quad word number for each AxC data
 *
 * */
typedef enum 
{
   /** Set AD dio quad word number to 1 quad */
   AIF2FL_AD_1QUAD = 0,
   /** Set AD dio quad word number to 2 quad */
   AIF2FL_AD_2QUAD,
   /** Set AD dio quad word number to 4 quad */
   AIF2FL_AD_4QUAD

} Aif2Fl_AdNumQWord;

/**
 *
 * @brief AD DIO BCN TABLE SEL  type 
 *
 * Use this symbol to specify DIO bcn table selection  type for AD
 * */
typedef enum 
{
   AIF2FL_AD_DIO_BCN_TABLE_0 = 0,
   AIF2FL_AD_DIO_BCN_TABLE_1

} Aif2Fl_AdBcnTable;

/**
 *
 * @brief aif2 dio engine index supported
 *
 * Use this symbol to specify the aif2 dio engine index
 *  */
typedef enum 
{
   /** Selects dio engine 0 */
   AIF2FL_DIO_ENGINE_0  = 0,
   /** Selects dio engine 1 */
   AIF2FL_DIO_ENGINE_1,
   /** Selects dio engine 2 */
   AIF2FL_DIO_ENGINE_2

} Aif2Fl_DioEngineIndex;


/**
 *
 * @brief AT Rad event strobe selection type
 *
 * Use this symbol to specify rad event strobe type for AT
 * */
typedef enum 
{
   /** Selects rad timer symbol strobe */
   AIF2FL_RADT_SYMBOL = 0,
   /** Selects rad timer frame strobe */
   AIF2FL_RADT_FRAME, 
   /** Selects ul rad timer symbol strobe */
   AIF2FL_ULRADT_SYMBOL,
   /** Selects ul rad timer frame strobe */
   AIF2FL_ULRADT_FRAME,
   /** Selects dl rad timer symbol strobe */
   AIF2FL_DLRADT_SYMBOL,
   /** Selects dl rad timer frame strobe */
   AIF2FL_DLRADT_FRAME,
   /** Selects phy timer frame strobe */
   AIF2FL_PHYT_FRAME

} Aif2Fl_AtEvtStrobe;



/**
 *
 * @brief AT Event selection type
 *
 * Use this symbol to specify event type for AT dynamic event setup
 * */
typedef enum 
{
    /** Selects external event 0 */
   AIF2FL_EVENT_0 = 0,
   /** Selects external event 1 */
   AIF2FL_EVENT_1,
   /** Selects external event 2 */
   AIF2FL_EVENT_2,
   /** Selects external event 3 */
   AIF2FL_EVENT_3,
   /** Selects external event 4 */
   AIF2FL_EVENT_4,
   /** Selects external event 5 */
   AIF2FL_EVENT_5,
   /** Selects external event 6 */
   AIF2FL_EVENT_6,
   /** Selects external event 7 */
   AIF2FL_EVENT_7,
   /** Selects special event 8 for TAC */
   AIF2FL_EVENT_8,
   /** Selects special event 9 for RAC A*/
   AIF2FL_EVENT_9,
   /** Selects special event 10 for RAC B*/
   AIF2FL_EVENT_10,
#ifdef K2
   /** Selects special event 11 for RAC C*/
   AIF2FL_EVENT_11,
   /** Selects special event 12 for RAC D*/
   AIF2FL_EVENT_12,
   /** Selects external event 13 */
   AIF2FL_EVENT_13,
   /** Selects external event 14 */
   AIF2FL_EVENT_14,
   /** Selects external event 15 */
   AIF2FL_EVENT_15,
   /** Selects external event 16 */
   AIF2FL_EVENT_16,
   /** Selects external event 17 */
   AIF2FL_EVENT_17,
   /** Selects external event 18 */
   AIF2FL_EVENT_18,
   /** Selects external event 19 */
   AIF2FL_EVENT_19,
   /** Selects external  event 20 */
   AIF2FL_EVENT_20,
   /** Selects external event 21 */
   AIF2FL_EVENT_21,
   /** Selects external event 22 */
   AIF2FL_EVENT_22,
   /** Selects external event 23 */
   AIF2FL_EVENT_23,
#endif
   /** Selects ingress dio event 0 */
   AIF2FL_IN_DIO_EVENT_0,
   /** Selects ingress dio event 1 */
   AIF2FL_IN_DIO_EVENT_1,
   /** Selects ingress dio event 2 */
   AIF2FL_IN_DIO_EVENT_2,
   /** Selects egress dio event 0 */
   AIF2FL_E_DIO_EVENT_0,
    /** Selects egress dio event 1 */
   AIF2FL_E_DIO_EVENT_1,
    /** Selects egress dio event 2 */
   AIF2FL_E_DIO_EVENT_2
} Aif2Fl_AtEventIndex;


/**
 *
 * @brief Timer sync sources in AT
 *
 * Use this symbol to specify timer sync source for AT
 * */
typedef enum 
{
    /** Selects RP1 sync */
   AIF2FL_RP1_SYNC = 0,
   /** Selects chip input sync */
   AIF2FL_CHIP_INPUT_SYNC,
   /** Selects sw debug sync */
   AIF2FL_SW_SYNC,
   /** Selects rm at  sync */
   AIF2FL_RM_AT_SYNC,
   /** Selects phy timer compare sync */
   AIF2FL_PHYT_CMP_SYNC

} Aif2Fl_AtSyncSource;


/**
 *
 * @brief Sync mode for AT
 *
 * Use this symbol to specify sync mode for AT
 * */
typedef enum 
{
   /** Selects non rp1 mode */
   AIF2FL_NON_RP1_MODE = 0,
   /** Selects rp1 mode */
   AIF2FL_RP1_MODE

} Aif2Fl_AtSyncMode;


/**
 *
 * @brief Re-Sync mode for AT
 *
 * Use this symbol to specify re-sync mode for AT
 * 
 * */
typedef enum 
{
   /** Selects at no auto resync mode */
   AIF2FL_NO_AUTO_RESYNC_MODE = 0,
   /** Selects at auto resync mode */
   AIF2FL_AUTO_RESYNC_MODE

} Aif2Fl_AtReSyncMode;

/**
 *
 * @brief select CRC mode on / off
 * 
 *
 *  */
typedef enum 
{
   /** Don't use at crc  */
   AIF2FL_AT_CRC_DONT_USE = 0,
   /** Use at crc  */
   AIF2FL_AT_CRC_USE

} Aif2Fl_AtCrcUse;

/**
 *
 * @brief select CRC flip mode between normal and reverse
 * 
 *
 *  */
typedef enum 
{
   /** use at crc normal mode  */
   AIF2FL_AT_CRC_NORMAL = 0,
   /** use at crc reverse mode  */
   AIF2FL_AT_CRC_REVERSE

} Aif2Fl_AtCrcFlip;

/**
 *
 * @brief select CRC init value
 * 
 *
 * */
typedef enum 
{
   /** use at crc init0  */
   AIF2FL_AT_CRC_INIT0 = 0,
   /** use at crc init1  */
   AIF2FL_AT_CRC_INIT1

} Aif2Fl_AtCrcInitOnes;

/**
 *
 * @brief select CRC invert mode
 * 
 *
 * */
typedef enum 
{
   /** use at crc non invert mode  */
   AIF2FL_AT_CRC_NOINVERT = 0,
   /** use at crc invert mode  */
   AIF2FL_AT_CRC_INVERT

} Aif2Fl_AtCrcInvert;


/**
 *
 * @brief CRC usage in RP1 sync mode for AT
 *
 * Use this symbol to specify CRC usage mode when AT is used in RP1 sync mode 
 * */
typedef enum 
{
   /** use at sync burst on crc fail  */
   AIF2FL_USE_SYNC_BURST_ON_CRC_FAIL = 0,
   /** discard at sync burst on crc fail  */
   AIF2FL_DISCARD_SYNC_BURST_ON_CRC_FAIL

} Aif2Fl_AtRp1CRCUsage;


/**
 *
 * @brief Type field definitions for RP1 sync burst
 *
 * Use this symbol to specify the type field in the RP1 sync burst 
 * */
typedef enum
{
   /** Selects rp1 type not used  */
   AIF2FL_RP1_TYPE_NOT_USED               = 0x00,
   /** Selects rp1 type rp3 frame number  */
   AIF2FL_RP1_TYPE_RP3_FRAME_NUM          = 0x01, 
   /** Selects rp1 type wcdma fdd frame number  */
   AIF2FL_RP1_TYPE_WCDMA_FDD_FRAME_NUM    = 0x02, 
   /** Selects rp1 type gsm edge 1 frame number  */
   AIF2FL_RP1_TYPE_GSM_EDGE_1_FRAME_NUM   = 0x03,
   /** Selects rp1 type gsm edge 2 frame number  */
   AIF2FL_RP1_TYPE_GSM_EDGE_2_FRAME_NUM   = 0x04,
   /** Selects rp1 type gsm edge3 frame number  */
   AIF2FL_RP1_TYPE_GSM_EDGE_3_FRAME_NUM   = 0x05,
   /** Selects rp1 type wcdma tdd frame number  */
   AIF2FL_RP1_TYPE_WCDMA_TDD_FRAME_NUM    = 0x06,
   /** Selects rp1 type cdma 2000 frame number  */
   AIF2FL_RP1_TYPE_CDMA_2000_FRAME_NUM    = 0x07,
   /** Selects rp1 type tdd  */
   AIF2FL_RP1_TYPE_TOD                    = 0x08,
   /** Selects rp1 type reserved first  */
   AIF2FL_RP1_TYPE_RESERVED_FIRST         = 0x09,
   /** Selects rp1 type reserved last */
   AIF2FL_RP1_TYPE_RESERVED_LAST          = 0x7F,
   /** Selects rp1 type spare first  */
   AIF2FL_RP1_TYPE_SPARE_FIRST            = 0x80,
   /** Selects rp1 type spare last  */
   AIF2FL_RP1_TYPE_SPARE_LAST             = 0xFF

} Aif2Fl_AtRp1TypeField;


/**
 *
 * @brief select EE module working function
 *
 * Use this symbol to specify one of the function of EE  
 * */
typedef enum
{
   /** Selects ee interrupt raw status  */
   AIF2FL_EE_INT_RAW_STATUS = 0, 
   /** Selects ee interrupt set  */
   AIF2FL_EE_INT_SET,             
   /** Selects ee interrupt clear  */
   AIF2FL_EE_INT_CLR,         
   /** Selects ee interrupt enabled status ev0  */
   AIF2FL_EE_INT_EN_STATUS_EV0,
   /** Selects ee interrupt enabled status ev1 */
   AIF2FL_EE_INT_EN_STATUS_EV1, 
   /** Selects ee interrupt enable ev0  */
   AIF2FL_EE_INT_EN_EV0,      
   /** Selects ee interrupt   enable ev1*/
   AIF2FL_EE_INT_EN_EV1,    
   /** Selects ee interrupt enable set ev0  */
   AIF2FL_EE_INT_EN_SET_EV0,     
   /** Selects ee interrupt enable set ev1  */
   AIF2FL_EE_INT_EN_SET_EV1,
   /** Selects ee interrupt enable clear ev0  */
   AIF2FL_EE_INT_EN_CLR_EV0,   
   /** Selects ee interrupt enable clear ev1  */
   AIF2FL_EE_INT_EN_CLR_EV1
    
} Aif2Fl_EeArgIndex;


/**
 * This is the set of control commands that are passed to
 * @a Aif2Fl_hwControl(), with an optional argument type-casted to @a void*
 *
 * The arguments, if any, to be passed with each command are specified
 * next to that command.
 */
typedef enum 
{
   /** Starts a Rx link. use hAif2->arg_link to select link (argument type: uint16_t * ) */
   AIF2FL_CMD_ENABLE_DISABLE_RX_LINK = 0,
    
   /** Starts a Tx link. use hAif2->arg_link to select link (argument type: uint16_t * ) */
   AIF2FL_CMD_ENABLE_DISABLE_TX_LINK,

#ifndef K2
   /** Enable loopback mode for specific link. use hAif2->arg_link to select link (argument type: uint16_t * )*/
   AIF2FL_CMD_ENABLE_DISABLE_LINK_LOOPBACK,
#endif

   /** Enable SD B8 PLL (argument type: uint16_t * )*/   
   AIF2FL_CMD_ENABLE_DISABLE_SD_B8_PLL,

   /** Enable SD B4 PLL (argument type: uint16_t * )*/
   AIF2FL_CMD_ENABLE_DISABLE_SD_B4_PLL,

   /** Control Aif2 Emulation (argument type:  Aif2Fl_VcEmu*)  */   
   AIF2FL_CMD_VC_EMU_CONTROL,

#ifndef K2
   /** Select Serdes link Tx test pattern   (argument type:  Aif2Fl_SdTestPattern *, use hAif2->arg_link to select link)  */
  AIF2FL_CMD_SD_LINK_TX_TEST_PATTERN,

  /** Select Serdes link Rx test pattern   (argument type:  Aif2Fl_SdTestPattern *, use hAif2->arg_link to select link)  */
  AIF2FL_CMD_SD_LINK_RX_TEST_PATTERN,
#endif

   /** Force RM Sync State (argument type:  Aif2Fl_RmSyncState *, use hAif2->arg_link to select link)  */   
   AIF2FL_CMD_RM_FORCE_STATE,

    /** TM L1 Inband Control Signal Set (argument type:  uint8_t *, use hAif2->arg_link to select link)  */   
    AIF2FL_CMD_TM_L1_INBAND_SET,

   /** Force TM Flush FIFO (argument type:  uint16_t *)  */   
   AIF2FL_CMD_TM_FLUSH_FIFO,

    /** Force TM Idle state (argument type:  uint16_t *)  */   
   AIF2FL_CMD_TM_IDLE,

    /** Force TM Resync state (argument type:  uint16_t *)  */   
   AIF2FL_CMD_TM_RESYNC,

   /** Dynamic configuration of PD CPRI id lut register (argument type:  Aif2Fl_PdCpriIdLut  *, use hAif2->arg_link to select link)  */   
   AIF2FL_CMD_PD_CPRI_ID_LUT_SETUP,

   /** Dynamic configuration of PD CPRI Control Word lut register (argument type:  Aif2Fl_PdCpriCwLut  *, use hAif2->arg_link to select link)  */   
   AIF2FL_CMD_PD_CPRI_CW_LUT_SETUP,

   /** Dynamic configuration of PD DBMR  (argument type:  Aif2Fl_DualbitMap  *, use hAif2->arg_link to select link)  */   
   AIF2FL_CMD_PD_LINK_DBMR_SETUP,

   /** Dynamic configuration of PD channel config registers (argument type:  Aif2Fl_PdChannelConfig  *)  */   
   AIF2FL_CMD_PD_CH_CONFIG_SETUP,

   /** Dynamic configuration of PE CPRI Control Word lut register (argument type:  Aif2Fl_CpriCwLut  *, use hAif2->arg_link to select link)  */   
   AIF2FL_CMD_PE_CPRI_CW_LUT_SETUP,

   /** Dynamic configuration of PE OBSAI header register (argument type:  Aif2Fl_PeObsaiHeader  *, use hAif2->arg_link to select link)  */   
   AIF2FL_CMD_PE_OBSAI_HEADER_SETUP,

   /** Dynamic configuration of PE DBMR  (argument type:  Aif2Fl_DualbitMap  *)  */   
   AIF2FL_CMD_PE_LINK_DBMR_SETUP,

   /** Dynamic configuration of PE Modulo rule  (argument type:  Aif2Fl_DualbitMap  *)  */   
   AIF2FL_CMD_PE_MODULO_RULE_SETUP,

   /** Dynamic configuration of PE channel config registers (argument type:  Aif2Fl_PeChannelConfig  *)  */   
   AIF2FL_CMD_PE_CH_CONFIG_SETUP,

   /** Dynamic configuration of PE channel rule LUT config registers (argument type:  Aif2Fl_PeChRuleLut  *)  */   
   AIF2FL_CMD_PE_CH_RULE_LUT_SETUP,
   
   /** Enables Trace data and framing data capture (use hAif2->arg_link to select link, argument type: uint16_t *) */ 
   AIF2FL_CMD_ENABLE_DISABLE_LINK_DATA_CAPTURE, // use arg_link index to determine which link are enabled/disabled

   /** Data Trace Synchronization with Frame boundary Enable (argument type: uint16_t *) */
   AIF2FL_CMD_ENABLE_DISABLE_DATA_TRACE_SYNC,

    /** Enables Ingress DB Debug mode (argument type: uint16_t *) */
   AIF2FL_CMD_DB_IN_ENABLE_DISABLE_DEBUG_MODE,
   
   /** Debug data written to bits 128:0 of Ingress DB RAM (argument type: uint32_t *) */
   AIF2FL_CMD_DB_IN_DEBUG_DATA_SETUP, 

   /** Ingress DB debug side band data setup (argument type: Aif2Fl_DbSideDatal *)  */
   AIF2FL_CMD_DB_IN_DEBUG_SIDE_DATA_SETUP, 

   /** Writes the data in the following registers into the Ingress DB and sideband RAMS
   DB_IDB_DEBUG_D0, DB_IDB_DEBUG_D1, DB_IDB_DEBUG_D2, DB_IDB_DEBUG_D3, DB_IDB_DEBUG_SBDN (argument type: uint16_t *)  */
   AIF2FL_CMD_DB_IN_DEBUG_WRITE,

   /** Set Read and Write Address used to access write or read Offset RAM for DB Debug (argument type: uint8_t * arg[0] : write offset addr  arg[1] : read offset addr)  */
   AIF2FL_CMD_DB_IN_DEBUG_OFFSET_ADDR,

   /** Enable or Disable Ingress DB channel to add or remove channel dynamically (argument type: uint32_t *)  */
   AIF2FL_CMD_DB_IN_ENABLE_DISABLE_CHANNEL,

   /** Setup Ingress DB channel to add or remove channel dynamically (argument type: Aif2Fl_DbChannel *)  */
   AIF2FL_CMD_DB_IN_CHANNEL_SETUP,

    /** Enables Egress DB Debug mode (argument type: uint16_t *) */
   AIF2FL_CMD_DB_E_ENABLE_DISABLE_DEBUG_MODE,

   /** Setup Side band data control info like dio and fifo write enable and channel id and dio base address.(argument type: Aif2Fl_DbSideData *) */
   AIF2FL_CMD_DB_E_DEBUG_READ_CONTROL,
   
   /** the value loaded into DB_EDB_DEBUG_RD_CNTL.CH_ID being issued to the AxC Token FIFO.(argument type: uint16_t *) */
   AIF2FL_CMD_DB_E_DEBUG_WRITE_TOKEN,

   /** Read the data in the following registers from the Egress DB and sideband RAMS
   DB_EDB_DEBUG_D0, DB_EDB_DEBUG_D1, DB_EDB_DEBUG_D2, DB_EDB_DEBUG_D3, DB_EDB_DEBUG_SBDN (argument type: uint16_t *)  */
   AIF2FL_CMD_DB_E_DEBUG_READ,

   /** Set Read and Write Address used to access write or read Offset RAM for DB Debug (argument type: uint8_t * arg[0] : write offset addr  arg[1] : read offset addr)  */
   AIF2FL_CMD_DB_E_DEBUG_OFFSET_ADDR,

   /** Enable or Disable Egress DB channel to add or remove channel dynamically (argument type: uint32_t *)  */
   AIF2FL_CMD_DB_E_ENABLE_DISABLE_CHANNEL,

   /** Setup Egress DB channel to add or remove channel dynamically (argument type: Aif2Fl_DbChannel *)  */
   AIF2FL_CMD_DB_E_CHANNEL_SETUP,

   /** Enable or Disable IN Global AD module dynamically (argument type: uint16_t *)  */
   AIF2FL_CMD_AD_IN_ENABLE_DISABLE_GLOBAL,

   /** Enable or Disable E Global AD module dynamically (argument type: uint16_t *)  */
   AIF2FL_CMD_AD_E_ENABLE_DISABLE_GLOBAL,
		
   /** Enable or Disable Global Ingress DIO mode dynamically (argument type: uint16_t *)  */
   AIF2FL_CMD_AD_IN_ENABLE_DISABLE_DIO_GLOBAL,
       
   /** Enable or Disable Global Egress DIO mode dynamically (argument type: uint16_t *)  */
   AIF2FL_CMD_AD_E_ENABLE_DISABLE_DIO_GLOBAL,
   
   /** Change Ingress DIO table selection dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
   AIF2FL_CMD_AD_IN_DIO_TABLE_SELECT,

   /** Change Ingress DIO num of AxC dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
   AIF2FL_CMD_AD_IN_DIO_NUM_AXC_CHANGE,

   /** Change Ingress DIO BCN table dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
   AIF2FL_CMD_AD_IN_DIO_BCN_TABLE_CHANGE,

   /** Change Egress DIO table selection dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
   AIF2FL_CMD_AD_E_DIO_TABLE_SELECT,

   /** Change Egress DIO num of AxC dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
   AIF2FL_CMD_AD_E_DIO_NUM_AXC_CHANGE,

   /** Change Egress DIO BCN table dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
   AIF2FL_CMD_AD_E_DIO_BCN_TABLE_CHANGE,

   /** Set Enable or disable Data trace DMA for data and framing data (argument type: uint16_t *)  */
   AIF2FL_CMD_AD_TRACE_DATA_DMA_CHANNEL_ON_OFF,

   /** Set Trace data base address  (argument type: uint32_t *)  */
   AIF2FL_CMD_AD_TRACE_DATA_BASE_ADDR,

   /** Set trace framing data base address  (argument type: uint32_t *)  */
   AIF2FL_CMD_AD_TRACE_FRAMING_DATA_BASE_ADDR,

   /** Sets the number of burst transfers before the destination address wraps back to the base address (argument type: uint8_t *)  */
   AIF2FL_CMD_AD_TRACE_CPPI_DMA_BURST_WRAP,

   /** Sets AT External Rad timer event dynamically (argument type: Aif2Fl_AtEvent *)  */
   AIF2FL_CMD_AT_EVENT_SETUP,

   /** Sets AT Delta offset  (use hAif2->arg_link to select link   argument type: Aif2Fl_AtEvent *)  */
   AIF2FL_CMD_AT_DELTA_SETUP,

   /** Sets AT Halt timer  (argument type:  uint16_t  *)  */
   AIF2FL_CMD_AT_HALT_TIMER,

   /** Sets AT diable all events for debug purpose (argument type:  uint16_t  *)  */
   AIF2FL_CMD_AT_DISABLE_ALL_EVENTS,
   
   /** Sets AT Arm timer  (argument type:  uint16_t  *)  */
   AIF2FL_CMD_AT_ARM_TIMER,

   /** Sets AT SW debug sync  (argument type:  uint16_t  *)  */
   AIF2FL_CMD_AT_DEBUG_SYNC,

   /** Sets AT radt wcdma clock divider terminal count  (argument type:  uint8_t  *)  */
   AIF2FL_CMD_AT_RAD_WCDMA_DIV,

   /** Sets AT Rad terminal count  (argument type:  Aif2Fl_AtTcObj  *)  */
   AIF2FL_CMD_AT_RAD_TC_SETUP,

   /** Sets AT GSM Tcount  (argument type:  Aif2Fl_AtGsmTCount  *)  */
   AIF2FL_CMD_AT_GSM_TCOUNT_SETUP,

   /** Enable Eight Rad and Six DIO Events  (argument type:  Aif2Fl_AtEventIndex  *)  */
   AIF2FL_CMD_AT_ENABLE_EVENT,

   /** Disable Eight Rad and Six DIO Events  (argument type:  Aif2Fl_AtEventIndex  *)  */
   AIF2FL_CMD_AT_DISABLE_EVENT,

   /** Force set  Eight Rad and Six DIO Events  (argument type:  Aif2Fl_AtEventIndex  *)  */
   AIF2FL_CMD_AT_FORCE_EVENT,

   /** EE End of interrupt vector value setup  (argument type:  uint8_t  *)  */
   AIF2FL_CMD_EE_EOI_SETUP,

   /** EE VB error interrupt set or clear (use hAif2->ee_arg to select between set and clear   argument type:  Aif2Fl_EeAif2IntSetup  *)  */
   AIF2FL_CMD_EE_AIF2_ERROR_INT,

   /** EE DB interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeDbIntSetup  *)  */
   AIF2FL_CMD_EE_DB_INT,
   
   /** EE AD interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeAdIntSetup  *)  */
   AIF2FL_CMD_EE_AD_INT,

   /** EE CD(CDMA module) interrupt set, clear, enable set or clear for EV0 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeCdIntSetup  *)  */
   AIF2FL_CMD_EE_CD_INT,

   /** EE SD interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeSdIntSetup  *)  */
   AIF2FL_CMD_EE_SD_INT,

   /** EE VC interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeVcIntSetup  *)  */
   AIF2FL_CMD_EE_VC_INT,

   /** EE Aif2 run control register setup (argument type:  Aif2Fl_EeAif2RunSetup  *)  */
   AIF2FL_CMD_EE_AIF2_RUN,
		
   /** EE Link A interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeLinkAIntSetup  *)  */
   AIF2FL_CMD_EE_LINKA_INT,

   /** EE Link B  interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeLinkBIntSetup  *)  */
   AIF2FL_CMD_EE_LINKB_INT,

   /** EE AT interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeAtInt  *)  */
   AIF2FL_CMD_EE_AT_INT,

   /** EE PD common interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EePdInt  *)  */
   AIF2FL_CMD_EE_PD_INT,

   /** EE PE common interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EePeInt  *)  */
   AIF2FL_CMD_EE_PE_INT
   
} Aif2Fl_HwControlCmd;

/**
 * This is the set of query commands to get the status of various
 * operations in AIF2
 *
 * The arguments, if any, to be passed with each command are specified
 * next to that command. */                   
typedef enum 
{

 /** Get  the version of the module accessed  @param  (Aif2Fl_PidStatus *) **/
   AIF2FL_QUERY_VERSION =  0,

 /** returns VC Emu status */
   AIF2FL_QUERY_VC_STAT,
   
 /** SERDES Rx Status     @param  (Aif2Fl_SdRxStatus *) */
   AIF2FL_QUERY_SD_RX_LINK_STATUS,

#ifndef K2
 /** SERDES Tx Status     @param  (CSL_Aif2SdTxStatus *) */
   AIF2FL_QUERY_SD_TX_LINK_STATUS,
#endif

 /** Return the status of SERDES B8 PLL lock.   @param (uint8_t *)*/
   AIF2FL_QUERY_SD_B8_PLL_LOCK,

 /** Return the status of SERDES B4 PLL lock.   @param (uint8_t *)*/
   AIF2FL_QUERY_SD_B4_PLL_LOCK,

  /** RM link Status 0. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus0 *)*/
   AIF2FL_QUERY_RM_LINK_STATUS_0,

  /** RM link Status 1. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus1 *)*/
   AIF2FL_QUERY_RM_LINK_STATUS_1,

  /** RM link Status 2. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus2 *)*/
   AIF2FL_QUERY_RM_LINK_STATUS_2,

  /** RM link Status 3. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus3 *)*/
   AIF2FL_QUERY_RM_LINK_STATUS_3,

  /** RM link Status 4. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus4 *)*/
   AIF2FL_QUERY_RM_LINK_STATUS_4,
	
  /** Return TM link CPRI HFN Status. use hAif2->arg_link to choose link.  @param (uint8_t *)*/
   AIF2FL_QUERY_TM_LINK_CPRI_HFN,

  /** TM link Status. use hAif2->arg_link to choose link.  @param (Aif2Fl_TmStatus *)*/
   AIF2FL_QUERY_TM_LINK_STATUS,

  /** RT Internal FIFO depth Status. use hAif2->arg_link to choose link.  @param (uint8_t *)*/
   AIF2FL_QUERY_RT_FIFO_DEPTH_STATUS,
    
  /** RT Header Error Status. use hAif2->arg_link to choose link.  @param (Aif2Fl_RtHeaderStatus *)*/
   AIF2FL_QUERY_RT_HEADER_ERROR_STATUS,

  /** Get  the status of the Retransmitter @param  (Aif2Fl_RtStatus *) **/
   AIF2FL_QUERY_RT_LINK_STATUS,
		
  /** PD 128 Channel Status.  @param (uint32_t *)*/
   AIF2FL_QUERY_PD_CHANNEL_STATUS,

  /** PD Packet Status for 128 channels if it is in or out of packet.  @param (uint32_t *)*/
   AIF2FL_QUERY_PD_PACKET_STATUS,

  /** PE 128 Channel Status.  @param (uint32_t *)*/
   AIF2FL_QUERY_PE_CHANNEL_STATUS,

  /** PE Packet Status for 128 channels if it is in or out of packet.  @param (uint32_t *)*/
   AIF2FL_QUERY_PE_PACKET_STATUS,

 /** Get Write and Read Offset Value at address in DB_IDB_DEBUG_OFS   @param  (uint8_t *) **/
   AIF2FL_QUERY_DB_IN_DEBUG_OFFSET_DATA,  

 /** Get Debug data written to bits 128:0 of Egress DB RAM @param (uint32_t *) **/
   AIF2FL_QUERY_DB_E_DEBUG_DATA, 

 /** Get Egress DB debug side band data setup  @param (Aif2Fl_DbSideData *)  **/
   AIF2FL_QUERY_DB_E_DEBUG_SIDE_DATA, 

 /** Get Write and Read Offset Value at address in DB_EDB_DEBUG_OFS   @param  (uint8_t *) **/
   AIF2FL_QUERY_DB_E_DEBUG_OFFSET_DATA,  

 /** Get Egress EOP count value   @param (uint32_t *) */
   AIF2FL_QUERY_DB_E_EOP_COUNT,

 /** Get Ingress EOP count value   @param (uint32_t *) */
   AIF2FL_QUERY_AD_I_EOP_COUNT,

 /** Get Aif2 timer link Pi captured value. use hAif2->arg_link to choose link   @param (uint32_t *) */
   AIF2FL_QUERY_AT_LINK_PI_CAPTURE,

 /** Get Aif2 timer capture Radt values @param (Aif2Fl_AtCaptRadt  *) */
   AIF2FL_QUERY_AT_RADT_CAPTURE,
		
 /** Get Aif2 timer RP1 type capture value @param  (uint8_t *) **/
   AIF2FL_QUERY_AT_RP1_TYPE_CAPTURE,

 /** Get Aif2 timer RP1 tod capture lsb value @param  (uint32_t *) **/
   AIF2FL_QUERY_AT_RP1_TOD_CAPTURE_LSB,

 /** Get Aif2 timer RP1 tod capture msb value @param  (uint32_t *) **/
   AIF2FL_QUERY_AT_RP1_TOD_CAPTURE_MSB,

 /** Get Aif2 timer RP1 RP3 capture lsb value @param  (uint32_t *) **/
   AIF2FL_QUERY_AT_RP1_RP3_CAPTURE_LSB,

 /** Get Aif2 timer RP1 RP3 capture msb value @param  (uint32_t *) **/
   AIF2FL_QUERY_AT_RP1_RP3_CAPTURE_MSB,

 /** Get Aif2 timer RP1 RAD capture lsb value @param  (uint32_t *) **/
   AIF2FL_QUERY_AT_RP1_RAD_CAPTURE_LSB,

 /** Get Aif2 timer RP1 RAD capture msb value @param  (uint32_t *) **/
   AIF2FL_QUERY_AT_RP1_RAD_CAPTURE_MSB,

 /** Get Aif2 Phy timer clock count value  @param (uint32_t *) */
   AIF2FL_QUERY_AT_PHY_CLOCK_COUNT,

  /** Get Aif2 Phy timer frame count value lsb @param (uint32_t *) */
   AIF2FL_QUERY_AT_PHY_FRAME_COUNT_LSB,

  /** Get Aif2 Phy timer frame count value msb @param (uint32_t *) */
   AIF2FL_QUERY_AT_PHY_FRAME_COUNT_MSB,

  /** Get Aif2 Rad timer clock count value  @param (uint32_t *) */
   AIF2FL_QUERY_AT_RAD_CLOCK_COUNT,

  /** Get Aif2 Rad timer symbol count value  @param (uint8_t *) */
   AIF2FL_QUERY_AT_RAD_SYMBOL_COUNT,

  /** Get Aif2 Rad timer frame count value lsb @param (uint32_t *) */
   AIF2FL_QUERY_AT_RAD_FRAME_COUNT_LSB,

  /** Get Aif2 Rad timer frame count value msb @param (uint32_t *) */
   AIF2FL_QUERY_AT_RAD_FRAME_COUNT_MSB,

  /** Get Aif2 Ul Rad timer clock count value  @param (uint32_t *) */
   AIF2FL_QUERY_AT_ULRAD_CLOCK_COUNT,

  /** Get Aif2 Ul Rad timer symbol count value  @param (uint8_t *) */
   AIF2FL_QUERY_AT_ULRAD_SYMBOL_COUNT,

  /** Get Aif2 Ul Rad timer frame count value lsb @param (uint32_t *) */
   AIF2FL_QUERY_AT_ULRAD_FRAME_COUNT_LSB,

  /** Get Aif2 Ul Rad timer frame count value msb @param (uint32_t *) */
   AIF2FL_QUERY_AT_ULRAD_FRAME_COUNT_MSB,

  /** Get Aif2 Dl Rad timer clock count value  @param (uint32_t *) */
   AIF2FL_QUERY_AT_DLRAD_CLOCK_COUNT,

  /** Get Aif2 Dl Rad timer symbol count value  @param (uint8_t *) */
   AIF2FL_QUERY_AT_DLRAD_SYMBOL_COUNT,

  /** Get Aif2 Dl Rad timer frame count value lsb @param (uint32_t *) */
   AIF2FL_QUERY_AT_DLRAD_FRAME_COUNT_LSB,

  /** Get Aif2 Dl Rad timer frame count value msb @param (uint32_t *) */
   AIF2FL_QUERY_AT_DLRAD_FRAME_COUNT_MSB,

  /** Get Aif2 Rad timer WCDMA chip count value  @param (Aif2Fl_AtWcdmaCount *) */
   AIF2FL_QUERY_AT_RAD_WCDMA_VALUE,
 
  /** Get Aif2 Ul Rad timer WCDMA chip count value  @param (Aif2Fl_AtWcdmaCount *) */
   AIF2FL_QUERY_AT_ULRAD_WCDMA_VALUE,

  /** Get Aif2 Dl Rad timer WCDMA chip count value  @param (Aif2Fl_AtWcdmaCount *) */
   AIF2FL_QUERY_AT_DLRAD_WCDMA_VALUE,

  /** Get Aif2 Rad timer time stamp clock count value  @param (uint32_t *) */
   AIF2FL_QUERY_AT_RAD_TSTAMP_CLOCK_COUNT,

  /** Get Aif2 GSM Tcount  value  @param (Aif2Fl_AtGsmTCount *) */
   AIF2FL_QUERY_AT_GSM_TCOUNT_VALUE,

  /** Get Aif2 EE DB interrupt  status value  @param (Aif2Fl_EeDbInt *) */
   AIF2FL_QUERY_EE_DB_INT_STATUS,

  /** Get Aif2 EE AD interrupt  status value  @param (Aif2Fl_EeAdInt *) */
   AIF2FL_QUERY_EE_AD_INT_STATUS,
        
  /** Get Aif2 EE CD(CDMA) interrupt  status value  @param (Aif2Fl_EeCdInt *) */
   AIF2FL_QUERY_EE_CD_INT_STATUS,
        
  /** Get Aif2 EE SD interrupt  status value  @param (Aif2Fl_EeSdInt *) */
   AIF2FL_QUERY_EE_SD_INT_STATUS,
        
  /** Get Aif2 EE VC interrupt  status value  @param (Aif2Fl_EeVcInt *) */
   AIF2FL_QUERY_EE_VC_INT_STATUS,

  /** Get Aif2 EE AIF2 run status value  @param (Aif2Fl_EeAif2Run *)  */
   AIF2FL_QUERY_EE_AIF2_RUN_STATUS,
      
  /** Get Aif2 EE error or alarm origination  @param (Aif2Fl_EeAif2Run *)  */
   AIF2FL_QUERY_EE_AIF2_ORIGINATION,
       
   /** Get Aif2 EE Link A interrupt  status value  @param (Aif2Fl_EeLinkAInt *) use hAif2->ee_arg to select function and hAif2->arg_link to select link */
   AIF2FL_QUERY_EE_LINKA_INT_STATUS,

  /** Get Aif2 EE Link B interrupt  status value  @param (Aif2Fl_EeLinkBInt *) use hAif2->ee_arg to select function and hAif2->arg_link to select link */
   AIF2FL_QUERY_EE_LINKB_INT_STATUS,

  /** Get Aif2 EE AT interrupt  status value  @param (Aif2Fl_EeAtInt *) use hAif2->ee_arg to select function */
   AIF2FL_QUERY_EE_AT_INT_STATUS,

   /* Get Aif2 EE PD common interrupt  status value  @param (Aif2Fl_EePdInt *) use hAif2->ee_arg to select function */
   AIF2FL_QUERY_EE_PD_INT_STATUS,

   /* Get Aif2 EE PE common interrupt  status value  @param (Aif2Fl_EePeInt *) use hAif2->ee_arg to select function */
   AIF2FL_QUERY_EE_PE_INT_STATUS
} Aif2Fl_HwStatusQuery;
/**
@} */



/** @addtogroup AIF2FL_DATASTRUCT
*
* @{ */

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/

/**
 * @brief pointer to the csl aif2 register global structure
 * */
typedef volatile CSL_Aif2Regs *Aif2Fl_RegsOvly;

#ifdef K2
/**
 * @brief This is a sub-structure in @a CSL_AifCommonLinkSetup. This structure is used for
 * configuring the parameters for Serdes params specific to a link  */
typedef struct
{

    /** Select Rx Align mode */
   Aif2Fl_SdRxAlign                   rxAlign;
   /** Rx loss of Signal */
   Aif2Fl_SdRxLos                      rxLos;

   /** polairty of Rx differential i/p - normal/inverted */
   Aif2Fl_SdRxInvertPolarity      rxPolarity;


} Aif2Fl_SdLinkSetup;

#else

/**
 *
 * @brief Sd link test pattern index
 *
 * */
typedef enum
{
    /** Test mode disabled */
    AIF2FL_SD_TEST_DISABLED = 0,
    /** Alternating 0/1 Pattern. An alternating 0/1 pattern with a period of 2UI  */
    AIF2FL_SD_ALTERNATING_0_1,
    /** Generate or Verify 27 . 1 PRBS. Uses a 7-bit LFSR with feedback polynomial x7 + x6 + 1 */
    AIF2FL_SD_PRBS_7BIT_LFSR,
    /** Generate or Verify 223.1 PRBS. Uses a 23-bit LFSR with feedback polynomial x23 + x18 + 1 */
    AIF2FL_SD_PRBS_23BIT_LFSR,
    /** Generate or Verify 223.1 PRBS. Uses a 23-bit LFSR with feedback polynomial x23 + x18 + 1 */
    AIF2FL_SD_PRBS_31BIT_LFSR

} Aif2Fl_SdTestPattern;

/**
 *
 * @brief Sd module index
 *
 * Use this symbol to specify the Sd Tx polarity
 * */
typedef enum
{
    /** Selects Tx pair normal polarity */
    AIF2FL_SD_TX_PAIR_NORMAL_POLARITY = 0,
    /** Selects Tx pair inverted polarity */
    AIF2FL_SD_TX_PAIR_INVERTED_POLARITY
} Aif2Fl_SdTxInvertPolarity;


/**
 *
 * @brief Sd clock recovery algorithm
 *
 * Use this symbol to specify the Sd Rx clock recovery algorithm
 * */
typedef enum
{
    /** Phase offset tracking upto ¡¾488ppm. Suitable for use in asynchronous systems with low frequency offset */
    AIF2FL_SD_RX_CDR_FIRST_ORDER_THRESH_1 = 0,
    /** Phase offset tracking upto ¡¾325ppm. Suitable for use in synchronous systems. Offers superiour rejection of random jitter, but is
    less responsive to systematic variation such as sinusoidal jitter */
    AIF2FL_SD_RX_CDR_FIRST_ORDER_THRESH_17 = 1,
    /** As per setting 000, but the algorithm is only enabled periodically to reduce power */
    AIF2FL_SD_RX_CDR_FO_PERIODIC_THRESH_1 =  4,
    /** As per setting 001, but the algorithm is only enabled periodically to reduce power*/
    AIF2FL_SD_RX_CDR_FO_PERIODIC_THRESH_17 = 5

}Aif2Fl_SdRxCdrAlg;

/**
 *
 * @brief Sd module index
 *
 * Use this symbol to specify the Serdes Rx termination
 **/

typedef enum
{
    /** This configuration is for DC coupled systems using CML transmitters*/
    AIF2FL_SD_RX_TERM_COMMON_POINT_VDDT = 0,
    /** This configuration is for AC coupled systems. The transmitter has no effect on the receiver common mode*/
    AIF2FL_SD_RX_TERM_COMMON_POINT_0_7 = 1,
    /** This configuration is for DC coupled systems which require the common mode voltage to be determined by the transmitter only.*/
    AIF2FL_SD_RX_TERM_COMMON_POINT_FLOATING = 3

} Aif2Fl_SdRxTerm;

/**
 *
 * @brief Sd module index
 *
 * Use this symbol to specify the Sd Rx adaptive equalizer
 * */
typedef enum
{
    /** Selects the maximum Receiver equalizer */
    AIF2FL_SD_RX_EQ_MAXIMUM = 0,
    /** Selects the adaptive Receiver equalizer*/
    AIF2FL_SD_RX_EQ_ADAPTIVE,
    /** Selects the Precursor equalization analysis*/
    AIF2FL_SD_RX_EQ_PRECURSOR,
    /** Selects the Postcursor equalization analysis*/
    AIF2FL_SD_RX_EQ_POSTCURSOR

} Aif2Fl_SdRxEqConfig;


/**
 *
 * @brief Sd module index
 *
 * Use this symbol to specify the Sd Tx output swing
 * */

typedef enum
{
    /** Selects tx output swing amplitude*/
    AIF2FL_SD_TX_OUTPUT_SWING_0 = 0,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_1,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_2,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_3,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_4,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_5,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_6,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_7,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_8,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_9,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_10,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_11,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_12,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_13,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_14,
    /** Selects tx output swing amplitude */
    AIF2FL_SD_TX_OUTPUT_SWING_15

} Aif2Fl_SdTxOutputSwing;


/**
 *
 * @brief Sd module index
 *
 * Use this symbol to specify the Sd Tx precursor tab weight
 * */
typedef enum
{
    /** Pre-cursor Transmit tap weights to 0 %*/
    AIF2FL_SD_TX_PRE_TAP_WEIGHT_0 = 0,
    /** Pre-cursor Transmit tap weights to  -2.5 %*/
    AIF2FL_SD_TX_PRE_TAP_WEIGHT_1,
    /** Pre-cursor Transmit tap weights to - 5 %*/
    AIF2FL_SD_TX_PRE_TAP_WEIGHT_2,
    /** Pre-cursor Transmit tap weights to -7.5 %*/
    AIF2FL_SD_TX_PRE_TAP_WEIGHT_3,
    /** Pre-cursor Transmit tap weights to -10 %*/
    AIF2FL_SD_TX_PRE_TAP_WEIGHT_4,
    /** Pre-cursor Transmit tap weights to -12.5 %*/
    AIF2FL_SD_TX_PRE_TAP_WEIGHT_5,
    /** Pre-cursor Transmit tap weights to -15 %*/
    AIF2FL_SD_TX_PRE_TAP_WEIGHT_6,
    /** Pre-cursor Transmit tap weights to -17.5 %*/
    AIF2FL_SD_TX_PRE_TAP_WEIGHT_7

} Aif2Fl_SdTxPrecursorTabWeight;


/**
 *
 * @brief Sd module index
 *
 * Use this symbol to specify the Sd Tx post cursor tab weight
 * */
typedef enum
{
    /** Post-cursor Transmit tap weights to 0 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_0 = 0,
    /** Post-cursor Transmit tap weights to  2.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_1,
    /** Post-cursor Transmit tap weights to 5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_2,
    /** Post-cursor Transmit tap weights to 7.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_3,
    /** Post-cursor Transmit tap weights to 10 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_4,
    /** Post-cursor Transmit tap weights to 12.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_5,
    /** Post-cursor Transmit tap weights to 15 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_6,
    /** Post-cursor Transmit tap weights to 17.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_7,
    /** Post-cursor Transmit tap weights to 20 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_8,
    /** Post-cursor Transmit tap weights to  22.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_9,
    /** Post-cursor Transmit tap weights to  25 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_10,
    /** Post-cursor Transmit tap weights to 27.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_11,
    /** Post-cursor Transmit tap weights to 30 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_12,
    /** Post-cursor Transmit tap weights to 32.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_13,
    /** Post-cursor Transmit tap weights to 35 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_14,
    /** Post-cursor Transmit tap weights to 37.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_15,
     /** Post-cursor Transmit tap weights to 0 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_16,
    /** Post-cursor Transmit tap weights to  -2.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_17,
    /** Post-cursor Transmit tap weights to - 5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_18,
    /** Post-cursor Transmit tap weights to -7.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_19,
    /** Post-cursor Transmit tap weights to -10 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_20,
    /** Post-cursor Transmit tap weights to -12.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_21,
    /** Post-cursor Transmit tap weights to -15 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_22,
    /** Post-cursor Transmit tap weights to -17.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_23,
    /** Post-cursor Transmit tap weights to -20 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_24,
    /** Post-cursor Transmit tap weights to  -22.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_25,
    /** Post-cursor Transmit tap weights to - 25 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_26,
    /** Post-cursor Transmit tap weights to -27.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_27,
    /** Post-cursor Transmit tap weights to -30 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_28,
    /** Post-cursor Transmit tap weights to -32.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_29,
    /** Post-cursor Transmit tap weights to -35 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_30,
    /** Post-cursor Transmit tap weights to -37.5 %*/
    AIF2FL_SD_TX_POST_TAP_WEIGHT_31

} Aif2Fl_SdTxPostcursorTabWeight;

/**
 * @brief This is a sub-structure in @a CSL_AifCommonLinkSetup. This structure is used for
 * configuring the parameters for Serdes params specific to a link  */
typedef struct
{

    /** Select Rx Align mode */
	Aif2Fl_SdRxAlign                   rxAlign;
   /** Rx loss of Signal */
	Aif2Fl_SdRxLos                      rxLos;

    /** specifies the clock/data recovery algorithm */
	Aif2Fl_SdRxCdrAlg            rxCdrAlgorithm;

   /** polairty of Rx differential i/p - normal/inverted */
	Aif2Fl_SdRxInvertPolarity      rxInvertPolarity;

   /** specifies the Rx termination options for AC/DC coupled scenarios */
	Aif2Fl_SdRxTerm              rxTermination;

   /** Rx equalizer configuration */
	Aif2Fl_SdRxEqConfig          rxEqualizerConfig;

   /** Hold Equalizer. Holds equalizer in the current state */
   uint16_t                                    bRxEqHold;

   /** Enable Offset Compensation. Enables samplers offset compensation. */
   uint16_t                                   bRxOffsetComp;

    /** Synchronisation Master. Enables the channel as the master lane for synchronization purposes.
    Tie high unless implementing macro to macro synchronization */
   uint16_t                             bEnableTxSyncMater;

   /** Invert Polarity. Inverts the polarity of TXPi and TXNi. */
   Aif2Fl_SdTxInvertPolarity         txInvertPolarity;

   /** Output swing. Selects one of 16 output amplitude settings between 100 and 850 mVdfpp  */
   Aif2Fl_SdTxOutputSwing      txOutputSwing;

   /** Precursor Tap Weight. Selects one of 8 output tap weights for TX waveform conditioning. The settings are from 0 to 17.5% in 2.5% steps  */
   Aif2Fl_SdTxPrecursorTabWeight         txPrecursorTapWeight;

    /** Adjacent post cursor Tap Weight. Selects one of 32 output tap weights for TX waveform conditioning.*/
   Aif2Fl_SdTxPostcursorTabWeight         txPostcursorTapWeight;


   /**  Transmitter pre and post cursor FIR filter Update. Update control of FIR taps weights.
   Fields TWPRE and TWPST1 can be updated when TXBCLK and this input are both high */
   uint16_t                                      bTxFirFilterUpdate;


} Aif2Fl_SdLinkSetup;

#endif

/**
 * @brief This is a sub-structure in @a Aif2Fl_LinkSetup. This structure is used for
 * configuring the parameters of common link index specifies which all link modules are using
 * */
typedef struct
{
   /** Link protocol  OBSAI/CPRI */
   Aif2Fl_LinkProtocol        linkProtocol;

    /** Link rate */      
   Aif2Fl_LinkRate             linkRate;

   /** Data width to be used on the link 7/8/15/16 bits */
   Aif2Fl_DataWidth          IngrDataWidth;
   
   /** Data width to be used on the link 7/8/15/16 bits */
   Aif2Fl_DataWidth          EgrDataWidth;
  
} Aif2Fl_CommonLinkSetup;


/**
 * @brief  This is a sub-structure in @a Aif2Fl_TmLinkSetup. This structure is used for
 * configuring the parameters of the CPRI params relating to TM  */
typedef struct
{
  
   /** CPRI  Link L1 Inband data 9 bits mask Bit 8: LOS_LOSERR_EN, Bit 7: LOS_LOSRX_EN, 
   Bit 6: LOF_LOFERR_EN, Bit 5: LOF_LOFRX_EN, Bit 4: RAI_LOSERR_EN, Bit 3: RAI_LOSRX_EN, 
   Bit 2: RAI_LOFERR_EN, Bit 1: RAI_LOFRX_EN, Bit 0: RAI_RAIRX_EN*/
   uint16_t        L1InbandEn;

   /** RM Link LOS Error source link Selection */
   Aif2Fl_LinkIndex              RmLinkLosError;

  /** RM Link LOF Error source link Selection */
   Aif2Fl_LinkIndex              RmLinkLofError;

   /** RM Link LOS Rx bit source link Selection */
   Aif2Fl_LinkIndex              RmLinkLosRx;

   /** RM Link LOF Rx bit source link Selection */
   Aif2Fl_LinkIndex              RmLinkLofRx;

   /** RM Link RAI Rx bit source link Selection */
   Aif2Fl_LinkIndex              RmLinkRaiRx;

   /** start up info Z.66.0 */
   uint8_t             TxStartup;

   /** transmit poniter p */
   uint8_t             TxPointerP;

    /** protocol version Z.2.0 */
   uint8_t             TxProtocolVer;

} Aif2Fl_CpriTmSetup;

/**
 * @brief  This is a sub-structure in @a Aif2Fl_LinkSetup. This structure is used for
 * configuring the parameters of the TM */

typedef struct
{
    /** uint16_tean indicating if Tm link is to be enabled  */
   uint16_t                        bEnableTmLink;
	
   /** uint16_tean indicating if Loss of Sync on Rm is to be enabled  */
   uint16_t                        bEnableRmLos;

   /** seed value for Tx scramber initialization  */
   uint8_t                        SeedValue;

   /** uint16_tean indicating if Tm Scrambler is to be enabled  */
   uint16_t                        bEnableScrambler;
   
   /** pointer to CPRI setup for TM */
   Aif2Fl_CpriTmSetup         pCpriTmSetup;

} Aif2Fl_TmLinkSetup;

/**
 * @brief  This is a sub-structure in @a Aif2Fl_LinkSetup. This structure is used for
 * configuring the parameters of RM link*/

typedef struct
{
   /** uint16_tean indicating if Rm link is to be enabled  */
   uint16_t                        bEnableRmLink;
   
   /** setup Rm fifo threshold word size for reading received data */
   Aif2Fl_RmFifoThold      RmFifoThold;

   /** Suppress error reporting when the receiver state machine is not in state ST3 */
   Aif2FlRmErrorSuppress  RmErrorSuppress; 

   /** Enables the RM to automatically disable Serdes symbol alignment when the receiver state machine reaches state ST3 */
   uint16_t                        bEnableSdAutoAlign;
   
   /** uint16_tean indicating if Rm Scrambler is to be enabled  */
   uint16_t                        bEnableScrambler;

   /** Enables a state transition from teh ST3 to the ST0 state when lcv_det_thold is met  */
   uint16_t                        bEnableLcvUnsync;

   /** Writing a 1 to the bit will enable the Line Code Violation counter. THis 16 bit counter will saturate when it reaches a value of 0xffff. 
   Writing a 0 to this bit will clear and disable the counter. The current counter value is available as status, lcv_cntr_value */
   uint16_t                        bEnableLcvControl;

   /** Enables the clock detect watch dog timer. */
   uint16_t                        bEnableWatchDog;

    /** Defines the wrap value of the clock detection watchdog circuit. A value of zero disables the clock watchdog timer, Range 0 to 255 */
   uint8_t                        WatchDogWrap;

   /** Enables the clock quality circuit. */
   uint16_t                        bEnableClockQuality;

   /** Defines the wrap value of the clock monitor used to define clock quality. A value of zero disables the clock monitor, Range 0 to 65,535 */
   uint16_t                      ClockMonitorWrap;

   /** Sets 8b10b los detect threshold values in number of Line Code Violations received during a master frame, OBSAI, or during a Hyperframe, 
   CPRI. Writing to this location will automatically clear the num_los counter and num_los_det status bit. Range 0 to 65,535*/
   uint16_t                      losDetThreshold;

    /** Threshold value for consecutive valid blocks of bytes which result in state ST1. Range 0 to 65,535 */
   uint16_t                      SyncThreshold;

    /** Threshold value for consecutive valid message groups which result in state ST3. Range 0 to 65,535*/
   uint16_t                      FrameSyncThreshold;

     /** Threshold value for consecutive invalid blocks of bytes which result in state ST0. Range 0 to 65,535 */
   uint16_t                      UnsyncThreshold;

    /** Threshold value for consecutive invalid message groups which result in state ST1. Range 0 to 65,535 */
   uint16_t                      FrameUnsyncThreshold;
} Aif2Fl_RmLinkSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_LinkSetup. This structure is used for
 * configuring the parameters of Retransmitter */

typedef struct
{
   /** The CI Select bits indicate the selected inbound link for the retransmitted data path */
   Aif2Fl_LinkIndex     CiSelect;

   /** The Empty Message Enable Bit configures the RT block to insert an empty message in the event that a message is discovered 
   to have a code violation in the header. The configuration is valid in OBSAI mode only. */
   uint16_t                        bEnableEmptyMsg;

   /** The RT Configuration field configures the mode of operation of the RT block */
   Aif2Fl_RtConfig       RtConfig;
  

} Aif2Fl_RtLinkSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_Pd(Pe)LinkSetup. This structure is used for
 * configuring the parameters of Pd and Pe dual bit map */
typedef struct
{
   /** CPRI Dual Bit Map FSM (DBMF) channel count, sets the max number of AxC supported in a given Link. i.e. 
   for CPRI 4x LTE15MHz this must be set to 7b'0000001 indicating 2 AxC */
   uint8_t                                         DbmX;
   /** DBMF bubble count where 0:indicates 1 bubble of 1 AxC sample (4bytes).OBSAI DBMF assumes 4 AxC samples as a bubble length. 
   A burst of bubbles is inserted only when dictated to do so by the bit maps. This field gives great flexibility in rate matching  */
   uint8_t                                         DbmXBubble;
   /** DBMF repititions of map1. */
   uint8_t                                         Dbm1Mult;
   /** DBMF number of bits of bit map1 to use. */
   uint8_t                                         Dbm1Size;
   /** DBMF number of bits of bit map2 to use. */
   uint8_t                                         Dbm2Size;
   /** bit-by-bit bit map [99:0]. pd_dbm_1map[0].dbm_1map[0] == bit 0 of 100 bits */
   uint32_t                                       Dbm1Map[4];
   /** bit-by-bit bit map [67:0]. pd_dbm_2map[0].dbm_2map[0] == bit 0 of 68 bits*/
   uint32_t                                       Dbm2Map[3];
   
} Aif2Fl_DualBitMap ;

/**
 * @brief This is a sub-structure in @a Aif2Fl_PdLinkSetup. This structure is used for
 * configuring the parameters of 32 Pd type LUT  */
typedef struct
{
    /** PD OBSAI time stamp format  selection  */
   Aif2Fl_TstampFormat                 ObsaiTsFormat;

   /** PD CRC type selection  */
   Aif2Fl_CrcLen                    PdCrcType;

    /** enable CRC for this link */
   uint16_t                                    bEnableCrc;

    /** OBSAI mode selection between packet and AxC */
   Aif2Fl_PdDataMode                PdObsaiMode;

    /** enable Ethernet strip for this link */
   uint16_t                                    bEnableEnetStrip;
	
   /** enable CRC calculation  for OBSAI header */
   uint16_t                                    bEnableCrcHeader;
   
} Aif2Fl_PdTypeLut ;

/**
 * @brief This is a sub-structure in @a Aif2Fl_LinkSetup. This structure is used for
 * configuring the parameters of protocol decoder */
typedef struct
{
   /** uint16_tean indicating if Pd link is to be enabled  */
   uint16_t                        bEnablePdLink;
 
   /** CPRI: bit3-0 enables stripping Ethernet headers for chan3-0 OBSAI: unused  */
   uint8_t                        CpriEnetStrip;

    /** CRC: programmable polynomial for CRC8 (other polynomials are fixed) */
   uint8_t                        Crc8Poly;

   /** CRC: programmable seed value for CRC8 (other polynomials are fixed) */
   uint8_t                        Crc8Seed;

   /** CPRI Control word pre packing bit order control (used for changing endianness) */
   uint8_t                        PreEncBitmap;

   /** CPRI Control word post packing bit order control (used for changing endianness) */
   uint8_t                        PostEncBitmap;

   Aif2Fl_CpriAxCPack                CpriAxCPack;
   
   /** CRPI Control Word 4B/5B encoding enable for 4 channels */
   Aif2Fl_CpriCwPktDelim           CpriCwPktDelimitor[4];

   /** CPRI Conrol Word Null Delimitor. Used instead of 4B/5B for packet delimitation. Only used when enabled by CpriCwPktDelimitor fields */
   uint16_t                                      CpriCwNullDelimitor;

   /** PD CPRI Pack map structure  */
   uint8_t                                        PdPackDmaCh[4];

  /** enable pd cpri pack  */
   uint16_t                                         bEnablePack[4];

   /** PD CRC  setup structure  */
   Aif2Fl_CrcLen                          PdCpriCrcType[4];

   /** enable pd cpri crc  */
   uint16_t                                         bEnableCpriCrc[4];
   
   /** Dual bit map structure for Pd CPRI dual bit map rule FSM */
   Aif2Fl_DualBitMap                   PdCpriDualBitMap;

   /** 32 Pd Type LUT structure */ 
   Aif2Fl_PdTypeLut                   PdTypeLut[32];

   /** DBMF CPRI Stream LUT: AxC:used to map DBM X count to DMA channel. PKT:2 lsb indicates 0-3 PKT packing circuit (assumed 4B/5B encoding)*/
   uint8_t                                        CpriDmaCh[128];
   
   /** DBMF CPRI Stream LUT: enable-disable of channel, for each value of DBM X count.*/
   uint16_t                                        bEnableCpriX[128];

    /** DBMF CPRI Stream LUT: dicates the cpri payload is to be used as AxC (normal) or Packet traffic */
   uint16_t                                        bEnableCpriPkt[128];

   /** Fine AxC offset. Used in the front end of PD to align word data into QWords. bit [1:0] are offset into a QWord.
   bit[2] give RSA double QWork alignment. Bit[2] and corresponding axc_offset[0] should always be programmed to be identical*/
   uint8_t                                       Cpri8WordOffset[128];
   
   /** All possible CPRI CW per hyperframe are are mapped to one of four(0~3) CPRI CW staging areas*/
   uint8_t                                       CpriCwChannel[256];    

   /** All possible CPRI CW per hyperframe. Dicatates whether the control word should be captured at all*/
   uint16_t                                        bEnableCpriCw[256];

   /** All possible CPRI CW per hyperframe. identifies which control word slot is eop for each stream */
   uint16_t                                        bHyperFrameEop[256];
   
} Aif2Fl_PdLinkSetup;

/**
 * @brief  This is a sub-structure in @a Aif2Fl_LinkSetup. This structure is used for
 * configuring the link parameters of protocol encoder */
typedef struct
{
   /** uint16_tean indicating if Pe link is to be enabled  */
   uint16_t                        bEnablePeLink;
   
   /** For streams that are AxC, use DB as a circular RAM which will feed DIO DMA. (WCDMA use DIO) otherwise, 
   DB is used as a FIFO for AxC traffic and will feed CPPI DMA */
   Aif2Fl_CppiDio            PeCppiDioSel;

   /** AIF2 is tollerant of whole symbols of missing data on AxC-by-AxC basis */
   uint16_t                         TddAxc;

   /** GSM compress mode turn on or off (only used for GSM) */
   uint16_t                         bGsmCompress;
   
   /**  enable or disable OBSAI bubble bandwidth **/
   uint16_t                        bEnObsaiBubbleBW;
   /** delay (in sys_clks) between DB read and PE processing */
   uint8_t                        PeDelay;

   /** CRC: programmable polynomial for CRC8 (other polynomials are fixed) */
   uint8_t                        Crc8Poly;

   /** CRC: programmable starting seed value (CRC16 & CRC32 seed is fixed)*/
   uint8_t                        Crc8Seed;

   /** CPRI Control word pre encoding bit order control (used for changing endianness) */
   uint8_t                        PreEncBitmap;

   /** CPRI Control word post encoding bit order control (used for changing endianness) */
   uint8_t                        PostEncBitmap;
   
   /** Dual bit map structure for Pe CPRI dual bit map rule FSM */
   Aif2Fl_DualBitMap                   PeCpriDualBitMap;

   /**  CRPI: identifies the bit precision of the IQ data. Used to un-packing packet data passed over CPRI AxC slots */
   Aif2Fl_CpriAxCPack                CpriAxCPack;

   /** CPRI Conrol Word Null Delimitor. Used instead of 4B/5B for packet delimitation. Only used when enabled by CpriCwPktDelimitor fields */
   uint16_t                                      CpriCwNullDelimitor;

   /** CRPI Control Word 4B/5B encoding enable for 4 channels */
   Aif2Fl_CpriCwPktDelim           CpriCwPktDelimitor[4];
   
   /** PE CPRI Pack map structure  */
   uint8_t                                        PePackDmaCh[4];

   /** enalbe pe cpri pack */
   uint16_t                                         bEnablePack[4];

   /** All possible CPRI CW per hyperframe are are mapped to one of four(0~3) CPRI CW staging areas*/
   uint8_t                                       CpriCwChannel[256];    

   /** All possible CPRI CW per hyperframe. Dicatates whether the control word should be captured at all*/
   uint16_t                                        bEnableCpriCw[256];

} Aif2Fl_PeLinkSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_LinkSetup. This structure is used for
 * configuring the parameters of aif2 timer module */
typedef struct
{
   /** Maximum pi offset value. max and min pi makes a window for pi offset */
   uint32_t                      PiMax;
   
   /** Minimum pi offset value. max and min pi makes a window for pi offset */
   uint32_t                      PiMin;

   /** Delta event offset index it is calculated based on byte clock*/
   uint32_t                      DeltaOffset;

   /** Control for positive or negative delta for TM delta event. Controls the BFN PHYT frame number passed to the TM when the Delta event occurs*/
   uint16_t                         IsNegativeDelta;

   /** PE1 event offset index it is calculated based on byte clock*/
   uint32_t                      PE1Offset;

   /** PE2 event offset index it is calculated based on byte clock*/
   uint32_t                      PE2Offset;
	
} Aif2Fl_AtLinkSetup;

#ifdef K2
/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of a SD module, the link index specifies which SD module is used
 * links 0-3 use SD module 0, links 4-5 use SD module 1 */

typedef struct
{
   /** Indicating if B8 PLL is to be enabled */
   uint8_t                        bEnablePllB8;
   
   /** Indicating if B4 PLL is to be enabled */
   uint8_t                        bEnablePllB4;


   /**  Tx byte clock from either B8 or B4 SEDES link will be selected as sys_clk once the PLL has acquired lock */
   Aif2Fl_SdClockSelect      SysClockSelect;

   /**  Select if link clock is gated off or on  each uint16_t array is matched with each link */
   uint16_t                                DisableLinkClock[6];
 
} Aif2Fl_SdCommonSetup;
#else
/**
 *
 * @brief Sd module index
 *
 * Use this symbol to specify the Sd PLL multiply factor
 * */
typedef enum
{
    /** Select 4x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_4X = 16,
    /** Select 5x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_5X = 20,
    /** Select 6x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_6X = 24,
    /** Select 8x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_8X = 32,
    /** Select 8.25x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_8_25X = 33,
    /** Select 10x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_10X = 40,
    /** Select 12x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_12X = 48,
    /** Select 12.5x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_12_5X = 50,
    /** Select 15x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_15X = 60,
     /** Select 16x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_16X = 64,
     /** Select 16.5x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_16_5X = 66,
    /** Select 20x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_20X = 80,
     /** Select 22x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_22X = 88,
    /** Select 25x PLL multiply factor */
    AIF2FL_PLL_MUL_FACTOR_25X = 100

}Aif2Fl_PllMpyFactor;

/**
 *
 * @brief Sd pll voltage range
 *
 * Use this symbol to specify the Sd PLL voltage range
 * */
typedef enum
{
    /** Use low voltage for PLL */
    AIF2FL_PLL_VOLTAGE_LOW = 0,
    /** Use high voltage for PLL */
    AIF2FL_PLL_VOLTAGE_HIGH

}Aif2Fl_SdVoltRange;

/**
 *
 * @brief Sd pll sleep
 *
 * Use this symbol to specify the Sd PLL sleep mode
 * */
typedef enum
{
    /** PLL awake  */
    AIF2FL_PLL_AWAKE = 0,
    /** PLL sleep */
    AIF2FL_PLL_SLEEP

}Aif2Fl_SdSleepPll;

/**
 *
 * @brief Sd pll loop bandwidth selection
 *
 * Use this symbol to specify the Sd PLL loop bandwidth
 * */
typedef enum
{
    /** set pll loop bandwidth mid */
    AIF2FL_PLL_LOOP_BAND_MID = 0,
    /** set pll loop bandwidth ultra high */
    AIF2FL_PLL_LOOP_BAND_UHIGH,
    /** set pll loop bandwidth low */
    AIF2FL_PLL_LOOP_BAND_LOW,
    /** set pll loop bandwidth high */
    AIF2FL_PLL_LOOP_BAND_HIGH

}Aif2Fl_SdLoopBandwidth;


/**
 *
 * @brief Sd pll clock bypass selection
 *
 * Use this symbol to specify the Sd PLL clock bypass
 * */
typedef enum
{
    /* Macro operates normally from the PLL. */
    AIF2FL_PLL_CLOCK_NO_BYPASS = 0,
    /* The macro operates functionally at low speed using TESTCLKT and TESTCLKR */
    AIF2FL_PLL_CLOCK_FUNCTIONAL_BYPASS,
    /* The PLL is bypassed by REFCLKP/N */
    AIF2FL_PLL_CLOCK_REFCLK_OBSERVE

}Aif2Fl_SdClockBypass;

/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of a SD module, the link index specifies which SD module is used
 * links 0-3 use SD module 0, links 4-5 use SD module 1 */

typedef struct
{
   /** Boolean indicating if B8 PLL is to be enabled */
   Bool                        bEnablePllB8;

   /** Boolean indicating if B4 PLL is to be enabled */
   Bool                        bEnablePllB4;

   /** PLL mpy setting 4,5,..25 */
   Aif2Fl_PllMpyFactor         pllMpyFactorB8;

  /** PLL mpy setting 4,5,..25 */
   Aif2Fl_PllMpyFactor         pllMpyFactorB4;

   /** PLL volt range setting between low and high */
   Aif2Fl_SdVoltRange         VoltRangeB8;

   /** PLL volt range setting between low and high */
   Aif2Fl_SdVoltRange         VoltRangeB4;

   /** Puts the B8 PLL into sleep state when high */
   Aif2Fl_SdSleepPll         SleepPllB8;

   /** Puts the B4 PLL into sleep state when high */
   Aif2Fl_SdSleepPll         SleepPllB4;

   /** PLL Loop bandwidth setting  */
   Aif2Fl_SdLoopBandwidth     LB_B8;

   /** PLL Loop bandwidth setting  */
   Aif2Fl_SdLoopBandwidth     LB_B4;

    /** PLL clock bypass setting  */
   Aif2Fl_SdClockBypass     CLKBYP_B8;

   /** PLL clock bypass setting  */
   Aif2Fl_SdClockBypass     CLKBYP_B4;

   /**  Tx byte clock from either B8 or B4 SEDES link will be selected as sys_clk once the PLL has acquired lock */
   Aif2Fl_SdClockSelect      SysClockSelect;

   /**  Select if link clock is gated off or on  each uint16_t array is matched with each link */
   Bool                                DisableLinkClock[6];

} Aif2Fl_SdCommonSetup;
#endif

/**
 * @brief  This is a sub-structure in @a Aif2Fl_PdCommonSetup. This structure is used for
 * configuring the routing parameters of protocol decoder  */
typedef struct
{
   /**OBSAI time stamp in header. Used to extend addressing by using the TS field The number of bits of TS to compare is controlled via the ctrl field.*/
   uint8_t                              RouteTs;

   /**OBSAI type field in header */
   uint8_t                              RouteType;

   /**OBSAI address field in header*/
   uint16_t                            RouteAddr;

   /**Link number for which to apply this routing rule*/
   uint8_t                            RouteLink;
   
   /** controls how many OBSAI time stamp bits to use in the reception routing. */
   Aif2Fl_RouteMask             RouteMask;

} Aif2Fl_PdRoute;

/**
 * @brief  This is a sub-structure in @a Aif2Fl_PdCommonSetup. This structure is used for
 * configuring Pd 128 Dma channel configuration */
typedef struct
{
   /**Channel On/Off. Schedules a channel to be turned on or off.*/
   uint16_t                              bChannelEn;

   /**PD data format (Normal or UL RSA)*/
   Aif2Fl_LinkDataType         DataFormat;
   
} Aif2Fl_PdChConfig;

/**
 * @brief  This is a sub-structure in @a Aif2Fl_PdCommonSetup. This structure is used for
 * configuring Pd 128 Dma channel configuration 1 register*/
typedef struct
{
   /**TS: enable OBSAI time stamp watch dog timer (only known use is GSM BB hopping). (Set to 0 for CPRI) */
   uint16_t                              bTsWatchDogEn;

   /**GSM data format (Other or GSM UL)*/
   Aif2Fl_GSMDataFormat         DataFormat;

    /**OBSAI framing counter: Which of 6 framing counter terminal counts should be used for each dma channel. (codes 3'b110 & 3'b111 are reserved)*/
   uint8_t                             FrameCounter;

   /**Offset for DIO DMA mode. Used to align DMA for AxC with different AxC offsets.*/
   uint8_t                             DioOffset;

   /**PD TDD, disables DMA of whole symbols (CPPI packets), Program as 0xffff for most applications Bit mapped, bit0 --> sym0 of framing FSM
   (Symbol count above 15 are always enabled) */
   uint16_t                            TddEnable;
   
} Aif2Fl_PdChConfig1;

/**
 * @brief This is a sub-structure in @a Aif2Fl_Pd(Pe)CommonSetup. This structure is used for
 * configuring the parameters of PD, PE frame counter */
typedef struct
{
   /** OBSAI AxC framing counter. Index Terminal count. */
   uint8_t                              FrameIndexTc;

   /** OBSAI AxC framing counter. Index Start count.*/
   uint8_t                              FrameIndexSc;

   /** OBSAI AxC framing counter. Symbol terminal count */
   uint8_t                              FrameSymbolTc;
	
} Aif2Fl_FrameCounter;

/**
 * @brief  This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of protocol decoder which are common to all links */
typedef struct
{

   uint16_t                                 PdGlobalEnable;
   
   /** For streams that are AxC, use DB as a circular RAM which will feed DIO DMA. (WCDMA use DIO) otherwise, 
   DB is used as a FIFO for AxC traffic and will feed CPPI DMA */
   Aif2Fl_CppiDio            PdCppiDioSel;

   /**Reception timing window width parameter. Indicates the window size for searching the radio frame boundary identified by TS=0*/
   uint16_t                            AxCOffsetWin;
   
   /** TS: OBSAI time stamp watch dog timer count duration. Counts n x 64 sys_clks */
   uint16_t                                TsWatchDogCount;

   /** Controls circuit to simply report or to report and add an EOP if the wdog fails on an expected EOP */
   uint16_t                                WatchDogEopAdd;

   /** Report every missed WDog fail, or report only fails of missing EOP */
   Aif2Fl_PdWatchDogReport           WatchDogReport;

   /** Max expected terminal count value for the RadT timer value. Used for creating a exected timing window */ 
   uint32_t                                PdRadtTC;

   /** Pd frame and symbol terminal counting value to calculate sop and eop of packet */ 
   Aif2Fl_FrameCounter       PdFrameTC[6];

   /** Pd route */
   Aif2Fl_PdRoute               PdRoute[128];
   
   /** Pd 128 Dma channel configuration */
   Aif2Fl_PdChConfig          PdChConfig[128];

   /*** OBSAI: Antenna Carrier offset programmed in 307.2MHz clocks and relative to the Frame Boundary of the RadT AT timer. 
          CPRI: Antenna Carrier Offset programmed in (non-oversampled) samples, relative to the link (PHY) frame boundary */
   uint32_t                               AxCOffset[128];

   /** Pd 128 Dma channel configuration 1 */
   Aif2Fl_PdChConfig1          PdChConfig1[128];

   /** TDD symbol enable/disable from symbol 16 ~ 47  */
   uint32_t                               TddEnable1[128];
   /** TDD symbol enable/disable from symbol 48 ~ 79  */
   uint32_t                               TddEnable2[128];
   /** TDD symbol enable/disable from symbol 80 ~ 111  */
   uint32_t                               TddEnable3[128];
   /** TDD symbol enable/disable from symbol 112 ~ 143  */
   uint32_t                               TddEnable4[128];

   /** Pd Frame message terminal counter for  OBSAI and CPRI  */
   uint16_t                            PdFrameMsgTc[128];

} Aif2Fl_PdCommonSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_Pd(Pe)CommonSetup. This structure is used for
 * configuring the parameters of Modulo Terminal count */
typedef struct
{
   /** Modulo Terminal count of the rule counter. Dictates the period of the rule. Terminal count is the OBSAI Module minus 1 */
   uint16_t                              RuleModulo;

   /** Transmission rule enable */
   uint16_t                               bEnableRule;

    /** Bit 1: Module Rule oppeations on OBSAI control messages   bit0: AxC messages */
   uint16_t                               bRuleObsaiCtlMsg;

   /** Index. Transmission Rule. Index indicates at which count the rule will fire */
   uint16_t                              RuleIndex;

   /** Indicates which link the rule is allocated to */
   uint8_t                              RuleLink;

	
} Aif2Fl_ModuloTc;

/**
 * @brief This is a sub-structure in @a Aif2Fl_PeCommonSetup. This structure is used for
 * PE DMA channel configuration 0 register */
typedef struct
{
   /** CRC: enable CRC check on AxC by AxC basis */
   uint16_t                              bCrcEn;

   /** OBSAI framing counter: Which of 6 framingcounter terminal counts should be used for each dma channel. (codes 3b110 and 3b111 are reserved) */
   uint8_t                              FrameTC;

   /** Controls RT to perform appropriate insterion/aggregation into PHY */
   Aif2Fl_PeRtContol                RtControl;

   /** CRC: length of CRC */
   Aif2Fl_CrcLen                  CrcType;

   /** check if data is ethernet or not */
   uint16_t                                     isEthernet;

    /** calculate CRC 16 over 3 bytes of OBSAI header */
   uint16_t                                     CrcObsaiHeader;
	
} Aif2Fl_PeDmaCh0;

/**
 * @brief This is a sub-structure in @a Aif2Fl_PeCommonSetup. This structure is used for
 * PE input fifo control register  */
typedef struct
{
  
   /** Set water mark per channel. Water mark controls fill level for which lower/higher fetch rate is issued */
   uint8_t                               MFifoWmark;

   /** Set full level per channel. Full level controls halt of data fetch to prevent overflow*/
   uint8_t                               MFifoFullLevel;

    /** Normally set to 0. First non-tdd symbol of frame. Used for startup sync with DMA*/
   uint8_t                              SyncSymbol;
	
} Aif2Fl_PeInFifoControl;

/**
 * @brief  This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of protocol encoder which are common to all links */
typedef struct
{
   /**  Gives Phase alignement relative to Channel Radio Frame Boundary for scheduling DMA. 
   OBSAI: only lsb is used CPRI: all three bits give 8th phase alignment. Phase can be used to adjust DMA according to DMA transfer budget.*/
   uint8_t                                PeTokenPhase;

   /**  bit order for Ethernet preamble and SOF  */
   uint16_t                               EnetHeaderSelect;
   
   /**  DirectIO buffer length, set same as value in DB */
   Aif2Fl_DioLen              GlobalDioLen;
 
   /**  PE globla enable setup  */
   uint16_t                                PeGlobalEnable;
   
   /** PE frame and symbol terminal counting value to calculate sop and eop of packet */ 
   Aif2Fl_FrameCounter       PeFrameTC[6];
   
   /**Enable PE channels one-by-one*/
   uint16_t                      bEnableCh[128];

   /** PE channel configuration 0 */
   Aif2Fl_PeDmaCh0         PeDmaCh0[128];
   
   /** PE input fifo control */
   Aif2Fl_PeInFifoControl        PeInFifo[128];
   
   /** Antanna carrier offset. Programmed in sys-clocks */
   uint32_t                             PeAxcOffset[128];

   /** PE Frame message terminal counter for OBSAI and CPRI    */
   uint16_t                            PeFrameMsgTc[128];

   /** Modulo terminal count  structure for Pe OBSAI WCDMA or LTE */
   Aif2Fl_ModuloTc            PeModuloTc[64];

   /** PE 128 DMA channel OBSAI time stamp value which is inserted into the frame */
   uint8_t                                       PeChObsaiTS[128];

    /** PE 128 DMA channel OBSAI type value which is inserted into the frame */
   uint8_t                                       PeChObsaiType[128];

   /** PE 128 DMA channel OBSAI address value which is inserted into the frame */
   uint16_t                                       PeChObsaiAddr[128];

    /** PE 128 DMA channel OBSAI time stamp mask value which is inserted into the frame */
   Aif2Fl_RouteMask                   PeChObsaiTsMask[128];

   /** PE 128 DMA channel OBSAI time stamp generation algorithm  */
   Aif2Fl_TstampFormat             PeChObsaiTsfomat[128];
   
   /* OBSAI channel (not used for CPRI) is a packet channel not AxC channel   */
   uint16_t                                         PeObsaiPkt[128];

   /* GSM OBSAI Address is taken from CPPI protocol specific bits (SW value passed in data dma) instead of obsai_adr mmr bits */
   uint16_t                                         PeBbHop[128];
   
   /** Dual bit map structure for Pe OBSAI dual bit map rule FSM */
   Aif2Fl_DualBitMap             PeObsaiDualBitMap[64];

   /** DMA channel rule LUT 0 for first 512 rules */
   uint8_t                               ChIndex0[512];
   /** DMA channel rule LUT 0 enable for first 512 rules */
   uint16_t                                bEnableChIndex0[512];
   uint16_t                                CpriPktEn0[512];
   
    /** DMA channel rule LUT 1 for second 512 rules  */
   uint8_t                               ChIndex1[512];
   /** DMA channel rule LUT 1 enable for first 512 rules */
   uint16_t                                bEnableChIndex1[512];
   /** DMA channel rule LUT 1 cpri packet enable for first 512 rules */
   uint16_t                                CpriPktEn1[512];

   /** DMA channel rule LUT 2 for third 512 rules */
   uint8_t                               ChIndex2[512];
   /** DMA channel rule LUT 2 enable for first 512 rules */
   uint16_t                                bEnableChIndex2[512];
   /** DMA channel rule LUT 2 cpri packet enable for first 512 rules */
   uint16_t                                CpriPktEn2[512];
   
    /** DMA channel rule LUT 3 for fourth 512 rules  */
   uint8_t                               ChIndex3[512];
   /** DMA channel rule LUT 3 enable for first 512 rules */
   uint16_t                                bEnableChIndex3[512];
   /** DMA channel rule LUT 3 cpri packet enable for first 512 rules */
   uint16_t                                CpriPktEn3[512];

   /** DMA channel rule LUT 4 for fifth 512 rules */
   uint8_t                               ChIndex4[512];
   /** DMA channel rule LUT 4 enable for first 512 rules */
   uint16_t                                bEnableChIndex4[512];
   /** DMA channel rule LUT 4 cpri packet enable for first 512 rules */
   uint16_t                                CpriPktEn4[512];
   
    /** DMA channel rule LUT 5 for sixth 512 rules  */
   uint8_t                               ChIndex5[512];
   /** DMA channel rule LUT 5 enable for first 512 rules */
   uint16_t                                bEnableChIndex5[512];
   /** DMA channel rule LUT 5 cpri packet enable for first 512 rules */
   uint16_t                                CpriPktEn5[512];

    /** DMA channel rule LUT 6 for seventh 512 rules */
   uint8_t                               ChIndex6[512];
   /** DMA channel rule LUT 6 enable for first 512 rules */
   uint16_t                                bEnableChIndex6[512];
   
    /** DMA channel rule LUT 7 for eighth 512 rules  */
   uint8_t                               ChIndex7[512];
   /** DMA channel rule LUT 7 enable for first 512 rules */
   uint16_t                                bEnableChIndex7[512];

} Aif2Fl_PeCommonSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of ingress Data Buffer */
typedef struct
{

   /** Channel number to be added or changed */
   uint8_t                   ChannelNum;
   
   /** Channel configuration structure for all 128 channels */
   uint8_t                               BaseAddress;

   /** Channel configuration structure for all 128 channels */
   Aif2Fl_DbFifoDepth        BufDepth;

   /** Big endian, little endian  swapping control */
   Aif2Fl_DbDataSwap       DataSwap;

   /** Internally changing IQ data order control */
   Aif2Fl_DbIqOrder           IQOrder;

   /** uint16_tean indicating if the protocol specific data is to be enabled. this is only used for ingress DB */
   uint16_t                               bEnablePsData;

    /** Programmable packet type that is inserted into pkt_type field in CPPI Info Word 0. this is only used for ingress DB  */
   uint8_t                               PacketType;

   /**  Address offset of SOP for DIO channels  */
   uint8_t                               EgressDioOffset;
	
} Aif2Fl_DbChannel;

/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of ingress Data Buffer */
typedef struct
{
   /** uint16_tean indicating if Ingress data buffer is to be enabled */
   uint16_t                               bEnableIngrDb;

   /** DIO buffer length selection between 128 and 256 bytes */
   Aif2Fl_DioLen              DioBufferLen;

   /** uint16_tean indicating if the channel is to be enabled */
   uint16_t                               bEnableChannel[128];

   /** Channel configuration structure for all 128 channels */
   Aif2Fl_DbChannel        IngrDbChannel[128];

} Aif2Fl_IngrDbSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of Egress data buffer */

typedef struct
{
   /** uint16_tean indicating if Ingress data buffer is to be enabled */
   uint16_t                               bEnableEgrDb;

   /** DIO buffer length selection between 128 and 256 bytes */
   Aif2Fl_DioLen              DioBufferLen;

   /** Egress Packet mode control  */
   Aif2Fl_DbPmControl     PmControl;

   /** uint16_tean indicating if the channel is to be enabled */
   uint16_t                               bEnableChannel[128];
   
   /** Channel configuration structure for all 128 channels */
   Aif2Fl_DbChannel        EgrDbChannel[128];
  
} Aif2Fl_EgrDbSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the common parameters of aif2 Dma module */

typedef struct
{
   /**  global enable Ingress DIO module  */
   uint16_t                                 IngrGlobalDioEnable;

   /**  global enable Egress DIO module  */
   uint16_t                                 EgrGlobalDioEnable;
   
   /**  global enable Ingress AD scheduler module  */
   uint16_t                                 IngrGlobalEnable;

   /**  global enable Egress AD module  */
   uint16_t                                 EgrGlobalEnable;
   
   /**   Wrapping count of EOPs sent to the CPPI DMA Controller */
   uint32_t                              IngrEopCount;
   
   /** This field controls how the Ingress Scheduler handles packets marked as failed by the PD*/
   Aif2Fl_AdFailMode           FailMode;

  /** This field controls the Ingress Scheduler aribitration priority between packet and dio */
  Aif2Fl_AdIngrPriority              IngrPriority;

  /**Queue manager number sent to CDMA Scheduling Interface for all egress transactions 0x3 to 0x0*/
  uint8_t                                 Tx_QueManager;

  /**Base egress queue number for AIF2. Channel id of each packet 127-0 added to this base value to generate 
  egress queue number sent to CDMA Scheduling Interface. 0xF80 to 0x000*/
  uint16_t                                 Tx_QueNum;

  /** This field controls the Egress Scheduler aribitration priority between packet and dio */
  Aif2Fl_AdEgrPriority              EgrPriority;

} Aif2Fl_AdCommonSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_AdDioSetup. This structure is used for
 * configuring the parameters of aif2 dma engine */
typedef struct
{
  /** Selects which buffer channel number table for the DMA engine to use */
  Aif2Fl_AdBcnTable                 BcnTableSelect;
  
  /** Sets the number of quad words per AxC */
  Aif2Fl_AdNumQWord              NumQuadWord;

   /** Sets the number of quad words per AxC */
  uint8_t                                        NumAxC;

   /** Sets the maximum DMA burst length */
  Aif2Fl_AdNumQWord              DmaBurstLen;
   
  /** enable or disable Egress DIO RSA data type format  */
  uint16_t                                       bEnEgressRsaFormat;

   /** enable or disable Ingress DIO RSA data type format  */
  uint16_t                                       bEnIngressRsaFormat;

   /** enable or disable DIO DMA channel  */
  uint16_t                                        bEnDmaChannel;

   /** Set the number of data blocks to transfer before wrapping back to dma_base_addr */
  uint16_t                                        DmaNumBlock;

   /** Sets the VBUS destination base address (upper 28 bits of 32 bit data bus). */
  uint32_t                                        DmaBaseAddr;

   /** Sets the DMA burst address stride (in multiples of 0x10). After each DMA burst, the DMA address will increment by this amount */
  uint16_t                                        DmaBurstAddrStride;

    /** Sets the DMA block address stride (in multiples of 0x10). */
  uint16_t                                        DmaBlockAddrStride;

  /** Data Buffer Channel Number for total 64 channels */
  uint8_t                                          DBCN[64];
  
} Aif2Fl_AdDioEngine;

/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of aif2 dma module especially for DIO mode */
typedef struct
{
  /** uint16_tean indicating if each Dio engine is to be enabled or not */
  uint16_t                              IngrDioEngineEnable[3];

  /** uint16_tean indicating if each Dio engine is to be enabled or not */
  uint16_t                              EgrDioEngineEnable[3];
  
  /** setup ingress dio engine */
  Aif2Fl_AdDioEngine     IngrDioEngine[3];

  /** setup egress dio engine */
  Aif2Fl_AdDioEngine     EgrDioEngine[3];

} Aif2Fl_AdDioSetup;

/** 
 * This object is used to query Phy/Rad timer count
 *  and timer init objects
 *
 */
typedef struct 
{
   /** AT event count clock num */
   uint32_t       ClockNum;
   /** AT event count symbol num */
   uint8_t         SymbolNum;
   /** AT event count lut index num */
   uint8_t         LutIndexNum;
   /** AT event count frame lsb num */
   uint32_t       FrameLsbNum;
   /** AT event count frame msb num */
   uint32_t       FrameMsbNum;
   /** FCB - 1 flag for Ul rad or Dl rad timer */
   uint16_t          FcbMinusOne;

} Aif2Fl_AtCountObj;

/** 
 * This object is used to intialise Phy and Rad timers 
 * 
 */
typedef struct
{
  /** AT event phy timer init count pointer */
  Aif2Fl_AtCountObj      *pPhyTimerInit;
  /** AT event rad timer init count pointer */
  Aif2Fl_AtCountObj      *pRadTimerInit;
  /** AT event ul rad timer init count pointer */
  Aif2Fl_AtCountObj      *pUlRadTimerInit;
  /** AT event dl rad timer init count pointer */
  Aif2Fl_AtCountObj      *pDlRadTimerInit;

} Aif2Fl_AtInitObj;

/** 
 * This object is used to setup terminal count value for  Phy and Rad timers 
 * 
 */
typedef struct
{
  /** AT event phy timer terminal count pointer */
  Aif2Fl_AtCountObj      *pPhyTimerTc;
  /** AT event rad timer terminal count pointer */
  Aif2Fl_AtCountObj      *pRadTimerTc;
   /** Rad timer Ram structure for symbol and Lut index  */
  uint32_t                            RadClockCountTc[128];

} Aif2Fl_AtTcObj;


/** @brief This object contains the aif2 GSM  Tcount  information */
typedef struct 
{
   /** GSM T1 count */
   uint16_t                   t1;
   
   /** GSM T2 count   */
   uint8_t                   t2;
   
   /** GSM T3 count  */
   uint8_t                   t3;

} Aif2Fl_AtGsmTCount;



/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the common parameters of aif2 timer module */
typedef struct
{
   /** Phy sync mode selection */
  Aif2Fl_AtSyncSource          PhySyncSel;
   
  /** Rad sync mode selection */
  Aif2Fl_AtSyncSource          RadSyncSel;

   /** Rp1 mode or not */
  Aif2Fl_AtSyncMode            SyncMode;

  /** Auto resync if new frame boundary is received */
  Aif2Fl_AtReSyncMode         AutoResyncMode;

  /** Crc use or not */
  Aif2Fl_AtCrcUse                CrcMode;

   /** Crc flip normal  or reverse */
  Aif2Fl_AtCrcFlip                 CrcFlip;

   /** Crc init ones is init0  or init1 */
  Aif2Fl_AtCrcInitOnes          CrcInitOnes;

  /** Crc invert is applied  or not */
  Aif2Fl_AtCrcInvert             CrcInvert;

  /** RP1 sync_sampl_window value = size - 1*/
  uint8_t                                   SyncSampleWindow;

  /** RP1 RADT Frame Load selection (8bit ~ 40bits) */
  uint8_t                                   Rp1RadtFrameLoad;

  /** RP1 PHYT Frame Load selection (8bit ~ 40bits) */
  uint8_t                                   Rp1PhytFrameLoad;

  /** Phyt compare value when when radt sync is selected to AIF2FL_PHYT_CMP_SYNC */
  uint32_t                                  PhytCompValue;

  /** RP1 RAD Type Select */
  Aif2Fl_AtRp1TypeField              Rp1Type;

  /** WCDMA value Divide terminal count to make WCDMA counter working 3.84 MHz sample rate */
  uint8_t                                    WcdmaDivTC;
  
  /** timer Init structure for Phy and Rad(Ul,Dl) */
  Aif2Fl_AtInitObj                  AtInit;              

   /** timer terminal count and LUT  structure for Phy and Rad(Ul,Dl) */
  Aif2Fl_AtTcObj                   AtTerminalCount; 

   /** T count  structure for GSM */
  Aif2Fl_AtGsmTCount           AtGsmTcount; 

} Aif2Fl_AtCommonSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_AtEventSetup. This structure is used for
 * configuring the parameters of aif2 at event for external Rad events and Internal events for dio */
typedef struct
{
   /** Select Event */
   Aif2Fl_AtEventIndex      EventSelect;
   
   /** Event offset index */
   uint32_t                             EventOffset;

   /** Event strobe selection  */
   Aif2Fl_AtEvtStrobe        EvtStrobeSel;

    /** Event modulo  */
   uint32_t                             EventModulo;

   /** Event Mask LSB for external GSM event */
   uint32_t                             EventMaskLsb;

    /** Event Mask MSB for external GSM event */
   uint32_t                             EventMaskMsb;

   /** frame strobe selection for DIO  */
   Aif2Fl_AtEvtStrobe        DioFrameStrobeSel;

   /** DIO frame Event offset index */
   uint32_t                             DioFrameEventOffset;

} Aif2Fl_AtEvent;



/**
 * @brief This is a sub-structure in @a Aif2Fl_CommonSetup. This structure is used for
 * configuring the parameters of aif2 dma module especially for external Rad events and Internal events for dio */
typedef struct
{
#ifdef K2
   /** Active event flag */
   uint16_t                                bEnableRadEvent[24];
   
    /** Aif2 Timer 19 external event and 5 special events config*/
   Aif2Fl_AtEvent              AtRadEvent[24];
#else
   /** Active event flag */
   uint16_t                                bEnableRadEvent[11];

   /** Aif2 Timer 8 external event and 3 special events config*/
   Aif2Fl_AtEvent              AtRadEvent[11];
#endif
   /** Active ingress dio event flag */
   uint16_t                                bEnableIngrDioEvent[3];
   
   /** Aif2 Timer 3 ingress dio event  config */
   Aif2Fl_AtEvent              AtIngrDioEvent[3];

    /** Active egress dio event flag */
   uint16_t                                bEnableEgrDioEvent[3];

    /** Aif2 Timer 3 egress  event  config */
   Aif2Fl_AtEvent              AtEgrDioEvent[3];

} Aif2Fl_AtEventSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_HwSetup. This structure is used for
 * configuring the parameters global to AIF2 */
typedef struct 
{
   /** Active link flag  */
   uint16_t                     ActiveLink[6];
   
   /**  frame mode : Normal mode , Short frame mode */
   Aif2Fl_FrameMode      frameMode;


} Aif2Fl_GlobalSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_HwSetup. This structure is used for
 * configuring the parameters of a link */
typedef struct 
{

   /** pointer to Sd common setup */
   Aif2Fl_SdCommonSetup      *pSdCommonSetup;

   /** pointer to Protocol decoder common setup */
   Aif2Fl_PdCommonSetup          *pPdCommonSetup;

   /** pointer to protocol encoder common setup */
   Aif2Fl_PeCommonSetup          *pPeCommonSetup;
   
   /** pointer to data buffer setup */
   Aif2Fl_IngrDbSetup              *pIngrDbSetup;

   /** pointer to data buffer setup */
   Aif2Fl_EgrDbSetup              *pEgrDbSetup;

   /** pointer to Aif2 DMA setup */
   Aif2Fl_AdCommonSetup        *pAdCommonSetup;

   /** pointer to Aif2 DMA DIO Engine setup */
   Aif2Fl_AdDioSetup              *pAdDioSetup;

   /** pointer to Aif2 Timer external and internal event  setup */
   Aif2Fl_AtEventSetup              *pAtEventSetup;

   /** pointer to Aif2 Timer common  setup */
   Aif2Fl_AtCommonSetup          *pAtCommonSetup;

} Aif2Fl_CommonSetup;

/**
 * @brief This is a sub-structure in @a Aif2Fl_HwSetup. This structure is used for
 * configuring the parameters of an outbound link */
typedef struct 
{
   /** Link index 0-5  */
   Aif2Fl_LinkIndex           linkIndex;

   /** pointer to Sd link setup */
   Aif2Fl_SdLinkSetup            *pSdLinkSetup;

   /** pointer to link common setup */
   Aif2Fl_CommonLinkSetup       *pComLinkSetup;

   /* pointer to RM link setup */
   Aif2Fl_RmLinkSetup           *pRmLinkSetup;

   /** pointer to TM setup */
   Aif2Fl_TmLinkSetup           *pTmLinkSetup;

    /** pointer to re-transmitter link setup */
   Aif2Fl_RtLinkSetup              *pRtLinkSetup;

   /** pointer to protocol encoder link setup */
   Aif2Fl_PeLinkSetup              *pPeLinkSetup;

   /** pointer to Protocol decoder link setup */
   Aif2Fl_PdLinkSetup              *pPdLinkSetup;

    /** pointer to Aif2 timer link setup (Pi, Delta, PE signal) */
   Aif2Fl_AtLinkSetup              *pAtLinkSetup;

} Aif2Fl_LinkSetup;


/** @brief This object contains the reference to the instance of AIF2 opened
 *  using the @a Aif2Fl_open().
 *
 *  The pointer to this, is passed to all AIF2 CSL APIs.
 */
typedef struct 
{
   /** This is a pointer to the registers of the AIF2 */
   Aif2Fl_RegsOvly         regs;
   
   /** This is the instance of AIF2 being referred to by this object  */
   CSL_InstNum             perNum;

   /** This is the dedicated link number for HW control and get HW status argument*/
   Aif2Fl_LinkIndex       arg_link;//0 ~ 5

   /** This is the dedicated dio engine number for HW control and get HW status argument*/
   Aif2Fl_DioEngineIndex       arg_dioEngine;//0 ~ 2

   /** This is the dedicated ee function index for HW control and get HW status argument*/
   Aif2Fl_EeArgIndex        ee_arg;

} Aif2Fl_Obj;


/** @brief handle pointer to aif2 object
 **/
typedef Aif2Fl_Obj *Aif2Fl_Handle;


/** @brief This will have the base-address information for the peripheral
 *  instance
 */
typedef struct 
{
   /** This is a pointer to the registers of the AIF2 */
    Aif2Fl_RegsOvly   regs;

} Aif2Fl_BaseAddress;



/** @brief Module specific parameters. 
 */
typedef struct
{
   /** Bit mask to be used for selecting link specific parameters. For this 
    *  module, this is not used as there is only one module specific parameter. 
    *  So user need not worry about this.
    */
   CSL_BitMask8   flags;
    
   /**Global setup structure */
   Aif2Fl_GlobalSetup   gSetup;

} Aif2Fl_Param;



/** @brief This is the Setup structure for configuring AIF2 using @a Aif2Fl_hwSetup()
 * function 
 */
typedef struct 
{
   /** Pointer to the global AIF2 setup structure */
   Aif2Fl_GlobalSetup         *globalSetup;
   
   /** Pointer specifying common setup structure */
   Aif2Fl_CommonSetup     *commonSetup;

   /** Pointer to the 6 link setup structure */
   Aif2Fl_LinkSetup            *linkSetup[6];
   
} Aif2Fl_Setup;



/**************************************************************************\
* AIF2 Data structures for HW Control
\**************************************************************************/

/**
 *
 * @brief RM force sync states
 *
 * Use this symbol to specify the state of the RM state machine
 * */
typedef enum 
{
   /** Selects the force RM state 0 */	
   AIF2FL_RM_FORCE_ST0 =  4,
   /** Selects the force RM state 1 */
   AIF2FL_RM_FORCE_ST1 =  5,
   /** Selects the force RM state 2 */
   AIF2FL_RM_FORCE_ST2 =  6,
   /** Selects the force RM state 3 */
   AIF2FL_RM_FORCE_ST3 =  7,
   /** Selects the force RM state 4 */
   AIF2FL_RM_FORCE_ST4 =  2,
   /** Selects the force RM state 5 */
   AIF2FL_RM_FORCE_ST5 =  3

} Aif2Fl_RmForceSyncState;


/** @brief This structure is used for DBMR dynamic configuring parameters of protocol encoder */
typedef struct
{
   /** true for CPRI and false for OBSAI DBMR setup */
   uint16_t                                       isCpri; 
   
   /** dedicated channel number for CPRI DBMr setup 0 ~ 5 */
   uint8_t                                       CpriLinkNum;

   /** dedicated rule number for Obsai DBMR setup 0~ 63 */
   uint8_t                                       RuleNum;
   
   /** Dual bit map structure for Pe dual bit map rule FSM */
   Aif2Fl_DualBitMap                  DualBitMap;
   
} Aif2Fl_PeDbmr;


/** @brief This structure is used for dynamic configuring the obsai header parameters of protocol encoder */
typedef struct
{

   /** dedicated channel number for this setup */
   uint8_t                                       ChannelNum;
   /** PE DMA channel OBSAI time stamp value which is inserted into the frame */
   uint8_t                                       PeChObsaiType;

    /** PE DMA channel OBSAI type value which is inserted into the frame */
   uint8_t                                       PeChObsaiTs;

   /** PE DMA channel OBSAI address value which is inserted into the frame */
   uint16_t                                       PeChObsaiAddr;

    /** PE DMA channel OBSAI time stamp mask value which is inserted into the frame */
   Aif2Fl_ObsaiTsMask                PeChObsaiTsMask;

   /** PE DMA channel OBSAI time stamp generation algorithm  */
   Aif2Fl_TstampFormat             PeChObsaiTsfomat;
   
} Aif2Fl_PeObsaiHeader;


/** @brief  This structure is used for dynamic configuring the Modulo rule parameters of protocol encoder */
typedef struct
{
    /** dedicated rule number for this setup */
   uint16_t                               RuleNum;
	
    /** Modulo terminal count  structure */
   Aif2Fl_ModuloTc            PeModuloTc;

} Aif2Fl_PeModuloRule;


 /** @brief  This structure is used for dynamic configuring the channel parameters of protocol encoder */
typedef struct
{
    /** dedicated channel number for this setup */
   uint8_t                                       ChannelNum;
	
   /**Enable axc channels one-by-one*/
   uint16_t                              bEnableAxC;

   /** PE channel configuration 0 */
   Aif2Fl_PeDmaCh0         PeDmaCh0;
   
   /** PE input fifo configuration  */
   Aif2Fl_PeInFifoControl         PeInFifo;
   
   /** PE AxC offset */
   uint32_t                            AxCOffset;

} Aif2Fl_PeChannelConfig;


/** @brief  This structure is used for dynamic configuring the channel parameters of protocol encoder */
typedef struct
{
    /** dedicated rule number from 0 to 511 */
   uint16_t                               RuleNum;

    /** dedicated lut number from 0 to 7 */
   uint8_t                               LutNum;
	
   /** Primary, DMA channel number, LUT maps rule indexes to DMA channels */
   uint8_t                               ChIndex;

   /** enable pe channel rule index */
   uint16_t                                bEnableChIndex;

   /** enable cpri packet mode for the rule */
   uint16_t                                bCpriPktEnable;

} Aif2Fl_PeChRuleLut;
  

/** @brief This structure is used for dynamic configuring the cpri Id LUT parameters of protocol decoder */
typedef struct
{
   /** dedicated channel number for this setup */
   uint8_t                                       ChannelNum;

   /**DBMF CPRI Stream LUT: AxC:used to map DBM X count to DMA channel. PKT:2 lsb indicates 0-3 PKT packing circuit (assumed 4B/5B encoding)*/
   uint8_t                                        CpriDmaCh;
   
   /** DBMF CPRI Stream LUT: enable-disable of channel, for each value of DBM X count.*/
   uint16_t                                        bEnableCpriX;

    /** DBMF CPRI Stream LUT: dicates the cpri payload is to be used as AxC (normal) or Packet traffic */
   uint16_t                                       bEnableCpriPkt;

   /** Fine AxC offset. Used in the front end of PD to align word data into QWords. bit [1:0] are offset into a QWord.
   bit[2] give RSA double QWork alignment. Bit[2] and corresponding axc_offset[0] should always be programmed to be identical*/
   uint8_t                                       Cpri8WordOffset;
   
} Aif2Fl_PdCpriIdLut;


/** @brief This structure is used for dynamic configuring the cpri 256 control word LUT parameters of protocol decoder */
typedef struct
{
   /** dedicated channel number for this setup */
   uint8_t                                       ChannelNum;

    /** All possible CPRI CW per hyperframe are are mapped to one of four(0~3) CPRI CW staging areas*/
   uint8_t                                       CpriCwChannel;    

   /** All possible CPRI CW per hyperframe. Dicatates whether the control word should be captured at all*/
   uint16_t                                        bEnableCpriCw;

} Aif2Fl_CpriCwLut;


 /** @brief  This structure is used for dynamic configuring the channel parameters of protocol decoder */
typedef struct
{
    /** dedicated channel number for this setup */
   uint8_t                                       ChannelNum;
	
   /** Pd route */
   Aif2Fl_PdRoute               PdRoute;
   
   /** Pd 128 Dma channel configuration */
   Aif2Fl_PdChConfig          PdChConfig;

   /*** OBSAI: Antenna Carrier offset programmed in 307.2MHz clocks and relative to the Frame Boundary of the RadT AT timer. 
          CPRI: Antenna Carrier Offset programmed in (non-oversampled) samples, relative to the link (PHY) frame boundary */
   uint32_t                               AxCOffset;

   /** Pd 128 Dma channel configuration 1 */
   Aif2Fl_PdChConfig1          PdChConfig1;

   /** TDD symbol enable/disable from symbol 16 ~ 47  */
   uint32_t                               TddEnable1;
   /** TDD symbol enable/disable from symbol 48 ~ 79  */
   uint32_t                               TddEnable2;
   /** TDD symbol enable/disable from symbol 80 ~ 111  */
   uint32_t                               TddEnable3;
   /** TDD symbol enable/disable from symbol 112 ~ 143  */
   uint32_t                               TddEnable4;

   /** pd frame message terminal count */
   uint16_t                              PdFrameMsgTc;

} Aif2Fl_PdChannelConfig;

 

/** @brief This object contains the aif2 DB side data information */
typedef struct 
{
   /** DirectIO Ram Buffer Write Enable */
   uint16_t                   bEnDioBufferWrite;

   /** DirectIO Ram Buffer Read Enable for Egress*/
   uint16_t                   bEnDioBufferRead;
   
    /** CPPI FIFO buffer Write Enable */
   uint16_t                   bEnFifoBufferWrite;
   
   /** Start of packet flag */
   uint16_t                   bSop;
   
  /** End of packet flag */
   uint16_t                   bEop;

   /** DMA channel number 127-0 */
   uint8_t                   ChannelId;

   /** Address within a DirectIO Circular Buffer. Each address contains a quad-word and 16 locations */
   uint8_t                   DioAddress;

   /** Transfer Byte Count: Indicates how many valid bytes are transferred in the current data phase. value should be within 0 ~ 16*/
   uint8_t                   xcnt;

   /** AxC symbol number inserted as part of protocol specific data (0x00 - 0xFF) */
   uint8_t                   Symbol;

} Aif2Fl_DbSideData;


/** @brief This object contains the aif2 Vc Emu control data information */
typedef struct 
{
   /** Aif2 Emulation Freerun Enable */
   uint16_t                   Freerun;
   
    /** Aif2 Emulation Soft Enable */
   uint16_t                   Soft;
   
   /** Aif2 Emulation mode RT selection enable */
   uint16_t                   RtSel;

} Aif2Fl_VcEmu;


/** @brief This object contains the aif2 EE Aif2 Error Interrupt data information */
typedef struct 
{
   /**  ee error interrupt */
   uint16_t                   Error_intr;
   /**  ee alarm interrupt */
   uint16_t                   Alarm_intr;
   /**  ee cppi dma  interrupt */
   uint16_t                   Cdma_intr;

} Aif2Fl_EeAif2Int;


/** @brief This object contains the aif2 EE DB Interrupt data information */
typedef struct 
{
   /**  ee db interrupt  db_ee_i_trc_ram_ovfl_err */
   uint16_t      db_ee_i_trc_ram_ovfl_err;
   /**  ee db interrupt  db_ee_i_token_ovfl_err */
   uint16_t      db_ee_i_token_ovfl_err;
   /**  ee db interrupt  db_ee_i_fifo_ovfl_err */
   uint16_t      db_ee_i_fifo_ovfl_err;
   /**  ee db interrupt  db_ee_i_pd2db_full_err */
   uint16_t      db_ee_i_pd2db_full_err;
   /**  ee db interrupt  db_ee_e_ps_axc_err */
   uint16_t      db_ee_e_ps_axc_err;
   /**  ee db interrupt  db_ee_e_cd_data_err */
   uint16_t      db_ee_e_cd_data_err;
   /**  ee db interrupt  db_ee_e_cd_data_type_err */
   uint16_t      db_ee_e_cd_data_type_err;

} Aif2Fl_EeDbInt;


/** @brief This object contains the aif2 EE AD Interrupt data information */
typedef struct 
{
   /** ee ad interrupt ad_ee_i_cd_data_err  */
   uint16_t      ad_ee_i_cd_data_err;
   /** ee ad interrupt ad_ee_e_cd_sch_err  */
   uint16_t      ad_ee_e_cd_sch_err;
   /** ee ad interrupt ad_ee_i_dma_0_err  */
   uint16_t      ad_ee_i_dma_0_err;
   /** ee ad interrupt ad_ee_i_dma_1_err  */
   uint16_t      ad_ee_i_dma_1_err;
   /** ee ad interrupt ad_ee_i_dma_2_err  */
   uint16_t      ad_ee_i_dma_2_err;
   /** ee ad interrupt ad_ee_e_dma_0_err  */
   uint16_t      ad_ee_e_dma_0_err;
   /** ee ad interrupt ad_ee_e_dma_1_err  */
   uint16_t      ad_ee_e_dma_1_err;
   /** ee ad interrupt ad_ee_e_dma_2_err  */
   uint16_t      ad_ee_e_dma_2_err;

} Aif2Fl_EeAdInt;


/** @brief This object contains the aif2 EE CD(PKTDMA module) Interrupt data information */
typedef struct 
{
   /** ee cppi dma interrupt  cd_ee_sop_desc_starve_err */
   uint16_t      cd_ee_sop_desc_starve_err;
   /** ee cppi dma interrupt  cd_ee_mop_desc_starve_err */
   uint16_t      cd_ee_mop_desc_starve_err;

} Aif2Fl_EeCdInt;


/** @brief This object contains the aif2 EE SD Interrupt data information */
typedef struct 
{
   /** ee serdes interrupt sd_ee_stspll_b4_err */
   uint16_t      sd_ee_stspll_b4_err;
   /** ee serdes interrupt sd_ee_stspll_b8_err */
   uint16_t      sd_ee_stspll_b8_err;

} Aif2Fl_EeSdInt;


/** @brief This object contains the aif2 EE VC Interrupt data information */
typedef struct 
{
   /** ee vc interrupt vc_ee_vbus_err */
   uint16_t      vc_ee_vbus_err;
  
} Aif2Fl_EeVcInt;


/** @brief This object contains the EE aif2 run control data information */
typedef struct 
{
   /** ee aif2 run control aif2_phy_run */
   uint16_t      aif2_phy_run;
   /** ee aif2 run control aif2_global_run */
   uint16_t      aif2_global_run;

} Aif2Fl_EeAif2Run;


/** @brief This object contains the aif2 EE Link A Interrupt data information */
typedef struct 
{
   /** ee link a interrupt  rm_ee_sync_status_change_err */
   uint16_t     rm_ee_sync_status_change_err;
   /** ee link a interrupt  rm_ee_num_los_det_err */
   uint16_t     rm_ee_num_los_det_err;
   /** ee link a interrupt  rm_ee_lcv_det_err */
   uint16_t     rm_ee_lcv_det_err;
   /** ee link a interrupt  rm_ee_frame_bndry_det_err */
   uint16_t     rm_ee_frame_bndry_det_err;
   /** ee link a interrupt  rm_ee_block_bndry_det_err */
   uint16_t     rm_ee_block_bndry_det_err;
   /** ee link a interrupt  rm_ee_missing_k28p5_err */
   uint16_t     rm_ee_missing_k28p5_err;
   /** ee link a interrupt  rm_ee_missing_k28p7_err */
   uint16_t     rm_ee_missing_k28p7_err;
   /** ee link a interrupt  rm_ee_k30p7_det_err */
   uint16_t     rm_ee_k30p7_det_err;
   /** ee link a interrupt  rm_ee_loc_det_err */
   uint16_t     rm_ee_loc_det_err;
   /** ee link a interrupt  rm_ee_rx_fifo_ovf_err */
   uint16_t     rm_ee_rx_fifo_ovf_err;
   /** ee link a interrupt  rm_ee_rcvd_los_err */
   uint16_t     rm_ee_rcvd_los_err;
   /** ee link a interrupt  rm_ee_rcvd_lof_err */
   uint16_t     rm_ee_rcvd_lof_err;
   /** ee link a interrupt  rm_ee_rcvd_rai_err */
   uint16_t     rm_ee_rcvd_rai_err;
   /** ee link a interrupt  rm_ee_rcvd_sdi_err */
   uint16_t     rm_ee_rcvd_sdi_err;
   /** ee link a interrupt  rm_ee_los_err */
   uint16_t     rm_ee_los_err;
   /** ee link a interrupt  rm_ee_lof_err */
   uint16_t     rm_ee_lof_err;
   /** ee link a interrupt  rm_ee_hfnsync_state_err */
   uint16_t     rm_ee_hfnsync_state_err;
   /** ee link a interrupt  rm_ee_lof_state_err */
   uint16_t     rm_ee_lof_state_err;
   /** ee link a interrupt  tm_ee_frm_misalign_err */
   uint16_t     tm_ee_frm_misalign_err;
   /** ee link a interrupt  tm_ee_fifo_starve_err */
   uint16_t     tm_ee_fifo_starve_err;

} Aif2Fl_EeLinkAInt;


/** @brief This object contains the aif2 EE Link B Interrupt data information */
typedef struct 
{
   /** ee link b interrupt  pd_ee_eop_err */
   uint16_t     pd_ee_eop_err;
    /** ee link b interrupt  pd_ee_crc_err */
   uint16_t     pd_ee_crc_err;
   /** ee link b interrupt  pd_ee_cpri_frame_err */
   uint16_t     pd_ee_cpri_frame_err;
   /** ee link b interrupt  pd_ee_axc_fail_err */
   uint16_t     pd_ee_axc_fail_err;
   /** ee link b interrupt   pd_ee_sop_err*/
   uint16_t     pd_ee_sop_err;
   /** ee link b interrupt  pd_ee_obsai_frm_err */
   uint16_t     pd_ee_obsai_frm_err;
   /** ee link b interrupt  pd_ee_wr2db_err */
   uint16_t     pd_ee_wr2db_err;
   /** ee link b interrupt   pe_ee_modrule_err*/
   uint16_t     pe_ee_modrule_err;
    /** ee link b interrupt   pe_ee_sym_err*/
   uint16_t     pe_ee_sym_err;
    /** ee link b interrupt   pe_ee_mf_fifo_overflow_err*/
   uint16_t     pe_ee_mf_fifo_overflow_err;
   /** ee link b interrupt   pe_ee_mf_fifo_underflow_err*/
   uint16_t     pe_ee_mf_fifo_underflow_err;
    /** ee link b interrupt   pe_ee_db_starve_err*/
   uint16_t     pe_ee_db_starve_err;
   /** ee link b interrupt   pe_ee_rt_if_err*/
   uint16_t     pe_ee_rt_if_err;
   /** ee link b interrupt   pe_ee_pkt_starve_err*/
   uint16_t     pe_ee_pkt_starve_err;
   /** ee link b interrupt  rt_ee_frm_err */
   uint16_t     rt_ee_frm_err;
   /** ee link b interrupt  rt_ee_ovfl_err */
   uint16_t     rt_ee_ovfl_err;
   /** ee link b interrupt  rt_ee_unfl_err */
   uint16_t     rt_ee_unfl_err;
   /** ee link b interrupt rt_ee_em_err  */
   uint16_t     rt_ee_em_err;
   /** ee link b interrupt  rt_ee_hdr_err */
   uint16_t     rt_ee_hdr_err;

} Aif2Fl_EeLinkBInt;


/** @brief This object contains the aif2 EE AT Interrupt data information */
typedef struct 
{
	/** ee at interrupt  at_ee_rp1_type_sys_rcvd_err */
	uint16_t      at_ee_rp1_type_sys_rcvd_err;
	/** ee at interrupt   at_ee_rp1_type_rp3_rcvd_err*/
	uint16_t      at_ee_rp1_type_rp3_rcvd_err;
	/** ee at interrupt   at_ee_rp1_type_tod_rcvd_err*/
	uint16_t      at_ee_rp1_type_tod_rcvd_err;
	/** ee at interrupt   at_ee_rp1_type_unsel_err*/
	uint16_t      at_ee_rp1_type_unsel_err;
	/** ee at interrupt   at_ee_rp1_type_spare_err*/
	uint16_t      at_ee_rp1_type_spare_err;
	/** ee at interrupt   at_ee_rp1_type_rsvd_err*/
	uint16_t      at_ee_rp1_type_rsvd_err;
	/** ee at interrupt   at_ee_rp1_bit_width_err*/
	uint16_t      at_ee_rp1_bit_width_err;
	/** ee at interrupt  at_ee_rp1_crc_err */
	uint16_t      at_ee_rp1_crc_err;
	/** ee at interrupt   at_ee_rp1_rp3_err*/
	uint16_t      at_ee_rp1_rp3_err;
	/** ee at interrupt   at_ee_rp1_sys_err*/
	uint16_t      at_ee_rp1_sys_err;
	/** ee at interrupt  at_ee_pi0_err */
	uint16_t      at_ee_pi0_err;
	/** ee at interrupt  at_ee_pi1_err */
	uint16_t      at_ee_pi1_err;
	/** ee at interrupt   at_ee_pi2_err*/
	uint16_t      at_ee_pi2_err;
	/** ee at interrupt   at_ee_pi3_err*/
	uint16_t      at_ee_pi3_err;
	/** ee at interrupt   at_ee_pi4_err*/
	uint16_t      at_ee_pi4_err;
	/** ee at interrupt   at_ee_pi5_err*/
	uint16_t      at_ee_pi5_err;
	/** ee at interrupt   at_ee_phyt_sync_err*/
	uint16_t      at_ee_phyt_sync_err;
	/** ee at interrupt   at_ee_radt_sync_err*/
	uint16_t      at_ee_radt_sync_err;

} Aif2Fl_EeAtInt;

/** @brief This object contains the aif2 EE PD Interrupt data information */
typedef struct 
{
	/** ee pd interrupt  pd_ee_ts_wdog_err  */
	uint16_t      pd_ee_ts_wdog_err;

} Aif2Fl_EePdInt;

/** @brief This object contains the aif2 EE PE Interrupt data information */
typedef struct 
{
	/** ee pe interrupt  pe_ee_rd2db_err  */
	uint16_t      pe_ee_rd2db_err;
	/** ee pe interrupt  pe_ee_token_req_ovfl_err  */
	uint16_t      pe_ee_token_req_ovfl_err;
	/** ee pe interrupt  pe_ee_token_wr_err  */
	uint16_t      pe_ee_token_wr_err;
	/** ee pe interrupt  pe_ee_dat_req_ovfl_err  */
	uint16_t      pe_ee_dat_req_ovfl_err;

} Aif2Fl_EePeInt;


/**************************************************************************\
* AIF2 Data structures for Get HW status
\**************************************************************************/

/** @brief This object contains the aif2 PID information */
typedef struct 
{
   /** Major revision (X) code */
   uint8_t                   major;
   
   /** Custom version code  */
   uint8_t                   custom;
   
   /** Minor revision (Y) code */
   uint8_t                   minor;

   /** RTL version R code */
   uint8_t                   RTL;

   /** function code */
   uint16_t                   func;

   /** current scheme */
   uint8_t                   scheme;

} Aif2Fl_PidStatus;


/** @brief This object contains the aif2 SERDES Rx  link status information */
typedef struct 
{
   /** If the alignment feature of the Ser-des is used by hardware, the receiver frame synchronizer must have knowledge 
   that each Ser-des port had completed a requested byte alignment so that the byte alignment control logic can operate. */
   uint16_t                   sdRxSync;
   
   /**The receiver frame synchronizer must have knowledge that each Ser-des port had detected a loss of signal condition 
   so that the receiver can suppress events due to a loss of frame synchronization  */
   uint16_t                   sdRxLosDetect;
   
#ifdef K2
   /**/
   uint16_t                   sdRxAdaptDone;
#else
   /** Offset compensation in progress. Driven high asynchronously during offset compensation*/
   uint16_t                   sdRxOCIP;

   /** Driven high during equalizer analysis if under equalized */
   uint16_t                   sdRxEqUnder;

   /** Driven high during equalizer analysis if over equalized */
   uint16_t                   sdRxEqOver;

   /** The receiver bus bandwidth is fixed in hardware to 20 bits and is not configurable through an MMR.
   However the value that reflects a 20 bit bus width will be read of the configuration register is read. */
   uint8_t                   sdRxBusWidth;

   /** Test Failure. Driven high when an error is encountered during a test sequence executed on an individual channel. Synchronous to RXBCLK */
   uint16_t                   sdRxTestFail;
#endif

} Aif2Fl_SdRxStatus;

#ifndef K2
/** @brief This object contains the aif2 SERDES Tx  link status information */
typedef struct
{

   /** The transmitter bus bandwidth is fixed in hardware to 20 bits and is not configurable through an MMR.
   However the value that reflects a 20 bit bus width will be read of the configuration register is read */
   uint8_t                   sdTxBusWidth;

   /** Test Failure. Driven high when an error is encountered during a test sequence executed on an individual channel. Synchronous to TXBCLK */
   uint16_t                   sdTxTestFail;

} Aif2Fl_SdTxStatus;

#endif


/** @brief This object contains the aif2 Rm link status0  information */
typedef struct 
{
   /** Indicates that status of the Frame Sync state machine */
   Aif2Fl_RmSyncState                   rmSyncStatus;
   
   /** Active when RX state machine is in ST0, inactive otherwise  */
   uint16_t                   rmLos;
   
   /** Detects when num_los counter has reached the programable los_det_thold limit  */
   uint16_t                   rmNumLosDetect;
   
   /** The clock watchdog circuit detected a missing clock */
   uint16_t                   rmLoc;

   /** Active after an RM FIFO overflow */
   uint16_t                   rmFifoOverflow;

} Aif2Fl_RmStatus0;


/** @brief This object contains the aif2 Rm link status1  information */
typedef struct 
{
   
   /** Represents the number of los detect */
   uint16_t                  rmNumLos;
   
   /** Number of Line Code Violations counted since last cleared and enabled. Range 0 to 65,535 */
   uint16_t                  rmLcvCountValue;

} Aif2Fl_RmStatus1;


/** @brief This object contains the aif2 Rm link status2  information */
typedef struct 
{
   
   /** A value that represents the quality, relative frequency, of the received Serdes clock. Range 0 to 65,535 */
   uint16_t                  rmClockQuality;
   
   /** Indicates the captured scrambling code, only when configuration bit scr_en = 1. */
   uint8_t                  rmScrValue;

} Aif2Fl_RmStatus2;


/** @brief This object contains the aif2 Rm link status3  information */
typedef struct 
{
   /** Received hyperframe number, Z.64.0. Range 0 to 149 basic frames */
   uint8_t                   rmHfn;
   
   /** Received Node B Frame number low byte, Z.128.0  */
   uint8_t                   rmBfnHigh;
   
   /** Received Node B Frame number high byte, Z.130.0*/
   uint8_t                   rmBfnLow;
   
   /** Active high status indicates when the receiber FSM is in the HFSYNC state ST3  */
   uint16_t                   rmHfsyncState;

   /** Active high status indicates Loss Of Frame when the receiver FSM is in state ST0 or ST1 */
   uint16_t                   rmLofState;

} Aif2Fl_RmStatus3;


/** @brief This object contains the aif2 Rm link status4  information */
typedef struct 
{
   /** Received protocol version, Z.2.0 */
   uint8_t                   rmL1Version;
   
   /** Received start up information, Z.66.0  */
   uint8_t                   rmL1Startup;
   
   /** Received reset, Z.130.0, b0  */
   uint16_t                   rmL1RST;

   /** Received rai, Z.130.0, b1  */
   uint16_t                   rmL1RAI;

   /** Received sdi, Z.130.0, b2*/
   uint16_t                   rmL1SDI;

   /** Received los, Z.130.0, b3*/
   uint16_t                   rmL1LOS;

   /** Received lof, Z.130.0, b4  */
   uint16_t                   rmL1LOF;

   /** Received Pointer P */
   uint16_t                   rmL1PointerP;

} Aif2Fl_RmStatus4;


/** @brief This object contains the aif2 Tm link status information */
typedef struct 
{
   /** TM Fail condition  */
   uint16_t                   tmFail;
   
   /** Tx FIFO underflow  */
   uint16_t                   tmFifoUnderflow;
   
   /** CO Frame alignment mismatch from transmit FSM */
   uint16_t                   tmFrameMisalign;

   /** Indicates that status of the Frame Sync state machine */
   Aif2Fl_TmSyncState                   tmFrameStatus;


} Aif2Fl_TmStatus;


/** @brief This object contains the aif2 Rt header error status information */
typedef struct 
{
   /** Indicates a header error has occurred */
   uint16_t                   HeaderError;
   
   /** Indicates which dma channel shows the header error  */
   uint8_t                  DmaChannel;
   
} Aif2Fl_RtHeaderStatus;


/** @brief This object contains the aif2 Rt link status information */
typedef struct 
{
   /** Indicates a header error has occurred */
   uint16_t                   rtHeaderError;
   
   /** Indicates an empty message was inserted  */
   uint16_t                   rtEmptyMessage;
   
   /** Indicates an Underflow of the FIFO has occurred*/
   uint16_t                   rtFifoUnderflow;

   /** Indicates an Overflow of the FIFO has occurred */
   uint16_t                   rtFifoOverflow;

   /** Indicates a frame error of the FIFO has occurred */
   uint16_t                   rtFrameError;


} Aif2Fl_RtStatus;


/** @brief This object contains the aif2 radt capture value information */
typedef struct 
{
   /** Capture RADT clock count upon a PHYT frame boundary */
   uint32_t                   clock;
   
   /** Capture RADT symbol count upon a PHYT frame boundary   */
   uint16_t                   symbol;
   
   /** Capture RADT frame count 5lsbs upon a PHYT frame boundary  */
   uint8_t                     frame;

} Aif2Fl_AtCaptRadt;


/** @brief This object contains the aif2 wcdma count value information */
typedef struct 
{
   /** WCDMA chip value */
   uint16_t                   chip;
   
   /** WCDMA slot value  */
   uint8_t                   slot;
   
   /** WCDMA frame vlaue */
   uint16_t                   frame;

} Aif2Fl_AtWcdmaCount;


/** @brief This object contains the aif2 EE error and alarm origination information */
typedef struct 
{
   /** EE origin lk_en_sts_a0 value */
   uint16_t    lk_en_sts_a0;
   /** EE origin lk_en_sts_b0 value */
   uint16_t    lk_en_sts_b0;
   /** EE origin lk_en_sts_a1 value */
   uint16_t    lk_en_sts_a1;
   /** EE origin lk_en_sts_b1 value */
   uint16_t    lk_en_sts_b1;
   /** EE origin lk_en_sts_a2 value */
   uint16_t    lk_en_sts_a2;
   /** EE origin lk_en_sts_b2 value */
   uint16_t    lk_en_sts_b2;
   /** EE origin lk_en_sts_a3 value */
   uint16_t    lk_en_sts_a3;
   /** EE origin lk_en_sts_b3 value */
   uint16_t    lk_en_sts_b3;
   /** EE origin lk_en_sts_a4 value */
   uint16_t    lk_en_sts_a4;
   /** EE origin lk_en_sts_b4 value */
   uint16_t    lk_en_sts_b4;
   /** EE origin lk_en_sts_a5 value */
   uint16_t    lk_en_sts_a5;
   /** EE origin lk_en_sts_b5 value */
   uint16_t    lk_en_sts_b5;
   /** EE origin at_en_sts value */
   uint16_t    at_en_sts;
   /** EE origin sd_en_sts value */
   uint16_t    sd_en_sts;
   /** EE origin db_en_sts value */
   uint16_t    db_en_sts;
   /** EE origin ad_en_sts value */
   uint16_t    ad_en_sts;
   /** EE origin cd_en_sts value */
   uint16_t    cd_en_sts;
   /** EE origin vc_en_sts value */
   uint16_t    vc_en_sts;

} Aif2Fl_EeOrigin;


/** @brief Aif2 context info is a pointer. 
 */
typedef void *Aif2Fl_Context;

/* KeyStone1 to KeyStone2 compatibility*/
#ifndef K2
#define CSL_AIF2_SD_RX_EN_CFG_RXENABLE_MASK (0x00000003u)
#define CSL_AIF2_SD_RX_EN_CFG_RXENABLE_SHIFT (0x00000000u)
#define CSL_AIF2_SD_RX_EN_CFG_RXENABLE_RESETVAL (0x00000000u)
#define CSL_AIF2_SD_RX_EN_CFG_RXENABLE_DISABLE (0x00000000u)
#define CSL_AIF2_SD_RX_EN_CFG_RXENABLE_SLEEP (0x00000001u)
#define CSL_AIF2_SD_RX_EN_CFG_RXENABLE_SNOOZE (0x00000002u)
#define CSL_AIF2_SD_RX_EN_CFG_RXENABLE_ENABLE (0x00000003u)
#define CSL_AIF2_SD_RX_EN_CFG_RESETVAL (0x00000000u)

#define CSL_AIF2_SD_TX_EN_CFG_TXENABLE_MASK (0x00000003u)
#define CSL_AIF2_SD_TX_EN_CFG_TXENABLE_SHIFT (0x00000000u)
#define CSL_AIF2_SD_TX_EN_CFG_TXENABLE_RESETVAL (0x00000000u)
#define CSL_AIF2_SD_TX_EN_CFG_TXENABLE_DISABLE (0x00000000u)
#define CSL_AIF2_SD_TX_EN_CFG_TXENABLE_SLEEP (0x00000001u)
#define CSL_AIF2_SD_TX_EN_CFG_TXENABLE_SNOOZE (0x00000002u)
#define CSL_AIF2_SD_TX_EN_CFG_TXENABLE_ENABLE (0x00000003u)
#define CSL_AIF2_SD_TX_EN_CFG_RESETVAL (0x00000000u)

#define CSL_AIF2_PD_LK_PACK_CPRI_PRE_ENC_BITSWAP_MASK (0x0000000Fu)
#define CSL_AIF2_PD_LK_PACK_CPRI_PRE_ENC_BITSWAP_SHIFT (0x00000000u)
#define CSL_AIF2_PD_LK_PACK_CPRI_PRE_ENC_BITSWAP_RESETVAL (0x00000000u)
#define CSL_AIF2_PD_LK_PACK_CPRI_POST_ENC_BITSWAP_MASK (0x000000F0u)
#define CSL_AIF2_PD_LK_PACK_CPRI_POST_ENC_BITSWAP_SHIFT (0x00000004u)
#define CSL_AIF2_PD_LK_PACK_CPRI_POST_ENC_BITSWAP_RESETVAL (0x00000000u)
#define CSL_AIF2_PD_LK_PACK_CPRI_RESETVAL (0x00000000u)

#define CSL_AIF2_PD_CW_LUT_HYPFM_EOP_MASK (0x00000008u)
#define CSL_AIF2_PD_CW_LUT_HYPFM_EOP_SHIFT (0x00000003u)
#define CSL_AIF2_PD_CW_LUT_HYPFM_EOP_RESETVAL (0x00000000u)
#define CSL_AIF2_PD_CW_LUT_RESETVAL    (0x00000000u)

#define CSL_AIF2_PE_LINK_GSM_COMPRESS_MASK (0x00000008u)
#define CSL_AIF2_PE_LINK_GSM_COMPRESS_SHIFT (0x00000003u)
#define CSL_AIF2_PE_LINK_GSM_COMPRESS_RESETVAL (0x00000000u)
#define CSL_AIF2_PE_LINK_GSM_COMPRESS_GSMC_OFF (0x00000000u)
#define CSL_AIF2_PE_LINK_GSM_COMPRESS_GSMC_ON (0x00000001u)

#endif

/**
@} */

/**************************************************************************\
* AIF2 global function declarations
\**************************************************************************/

/** 
 * @ingroup AIF2FL_FUNCTION
 * @brief Peripheral specific initialization function.
 *
 * This is the peripheral specific intialization function. This function is
 * idempotent in that calling it many times is same as calling it once.
 * This function initializes the CSL data structures, and doesn't touches
 * the hardware.
 *
 * <b> Usage Constraints: </b>
 * This function should be called before using any of the CSL APIs in the AIF2
 * module.
 * 
 * @b Example:
 * @verbatim
 * ...
 * Aif2Fl_Context Aif2Context 
 * 
   if (AIF2FL_SOK != CSL_aifInit(&AifContext) {
       return;
   }
   @endverbatim
 *
 *
 * @return returns the status of the operation
 *
 * @{ */

Aif2Fl_Status Aif2Fl_init(
   /** AIF2 specific context information
    */
   Aif2Fl_Context *pContext
   );


/** 
 * @ingroup AIF2FL_FUNCTION
 * @brief Opens the instance of AIF2 requested.
 *
 *  The open call sets up the data structures for the particular instance of
 *  AIF device. The device can be re-opened anytime after it has been normally
 *  closed if so required. The handle returned by this call is input as an
 *  essential argument for rest of the APIs described for this module.
 *
 *  <b> Usage Constraints: </b>
 *  AIF must be successfully initialized via @a CSL_AIFInit() before calling
 *  this function. Memory for the @a CSL_AifObj must be allocated outside
 *  this call. This object must be retained while usage of this peripheral.
 *
 *  @b Example:
 *  @verbatim

     CSL_AifObj     aifObj;
     Aif2Fl_Status     status;
     CSL_AifParam   aifParam;
     aifParam.linkIndex = AIF2FL_LINK0 ;
      ...
     hAif = CSL_aifOpen(&aifObj,
                        CSL_AIF,
                        &aifParam,
                        &status);
   @endverbatim
 *
 * @return returns a handle @a Aif2Fl_Handle to the requested instance of
 * Aif if the call is successful, otherwise, a @a NULL is returned.
 *
 * @{ */
Aif2Fl_Handle Aif2Fl_open (
   /** Pointer to the object that holds reference to the
    *  instance of AIF2 requested after the call
    */
   Aif2Fl_Obj         *pAif2Obj,
   /** Instance of AIF2 to which a handle is requested
    */
   CSL_InstNum            aif2Num,
   /** Module specific parameters;
    */
   Aif2Fl_Param           *paif2Param,
   /** This returns the status (success/errors) of the call.
    * Could be 'NULL' if the user does not want status information.
    */
   Aif2Fl_Status             *pStatus
   );

/**
 * @ingroup AIF2FL_FUNCTION
 * @brief Gets the instance of AIF2 base address (DSP only).
 *
 *
 * @return returns status and base address to the requested instance of
 * Aif.
 *
 * @{ */
#ifdef _TMS320C6X
Aif2Fl_Status
Aif2Fl_getBaseAddress (
        CSL_InstNum 	        aif2Num,
        Aif2Fl_Param *          paif2Param,
        Aif2Fl_BaseAddress *    pBaseAddress
);
#endif

/**  
 * @ingroup AIF2FL_FUNCTION
 * @brief Reset whole AIF2 module
 *
 *  The open reset whole AIF2 devices and MMRs
 *  The handle returned by this call is input as an
 *  essential argument for rest of the APIs described for this module.
 *
 *  <b> Usage Constraints: </b>
 *  AIF must be successfully initialized via @a Aif2Fl_init(), Aif2Fl_open() before calling
 *  this function. Memory for the @a Aif2Fl_Obj must be allocated outside
 *  this call. This object must be retained while usage of this peripheral.
 *
 *  @b Example:
 *  @verbatim

     Aif2Fl_Handle     haif2;
    
      ...
     status = Aif2Fl_reset(haif2);
     
   @endverbatim
 *
 * @return returns a handle @a Aif2Fl_Handle to the requested instance of
 * Aif2 if the call is successful, otherwise, a @a NULL is returned.
 *
 * @{ */
Aif2Fl_Status Aif2Fl_reset (
   /** Pointer to the object that holds reference to the
    *  instance of AIF2 requested after the call
    */
   Aif2Fl_Handle        hAif2
   );


/**
 * @ingroup AIF2FL_FUNCTION
 *  The Close call releases the resources of the peripheral
 *
 *  <b> Usage Constraints: </b>
 *  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *  in that order before Aif2Fl_close() can be called.
 *
 *  @b Example:
 *  @verbatim

      Aif2Fl_Handle hAif2;
      ...
      Aif2Fl_close(hAif2);
    @endverbatim
 *
 * @return returns the status of the operation (see @a Aif2Fl_Status)
 * Status is:
 *    AIF2FL_SOK, if close function succeeds.
 *    CSL_EAIF2_BADHANDLE, if the handle is not valid.
 *
 * @{ */
Aif2Fl_Status  Aif2Fl_close(
    /** Pointer to the object that holds reference to the
     *  instance of AIF  link requested after the Aif2Fl_open(...) call
    */
    Aif2Fl_Handle         hAif2
);

/**
 * @ingroup AIF2FL_FUNCTION
 * This function initializes the device registers with
 * the appropriate values provided through the HwSetup Data structure.
 *
 * <b> Usage Constraints:</b>
 * Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 * in that order before Aif2Fl_hwSetup() can be called. The user has to
 * allocate space for & fill in the main setup structure appropriately before
 * calling this function
 *
 * @b Example:
 * @verbatim
     Aif2Fl_Handle hAif2;
     Aif2Fl_Status status;
     Aif2Fl_HwSetup  Aif2Setup;
    Aif2Fl_HwGlobalStup  globalSetup = {?;
    Aif2Fl_HwCommonLinkSetup commonSetup = {?;
    Aif2Fl_HwLinkSetup linkSetup[0~5] = {?;
    
    Aif2Setup.globalSetup   = &globalSetup;
    Aif2Setup.commonSetup   = &commonSetup;
    Aif2Setup.linkSetup  = &linkSetup[0~5] ;
    
     ;
     status = Aif2Fl_hwSetup (hAif2, &Aif2Setup);
   @endverbatim
 *
 * @return Returns the status of the setup operation (see @a Aif2Fl_Status)
 * Status is:
 * AIF2FL_SOK - successful completion of the setup
 * CSL_EAIF2_INVALID_PARAMS - hwSetup structure is not initialized.
 * @{ */

 Aif2Fl_Status  Aif2Fl_hwSetup(
    /** Pointer to the object that holds reference to the
     *  instance of AIF2 requested after the call
    */
    Aif2Fl_Handle             hAif2,
    /** Pointer to setup structure which contains the
     *  information to program AIF2 to a useful state
    */
    Aif2Fl_Setup          *aif2Setup
);

/**
  * @ingroup AIF2FL_FUNCTION
  * @brief Controls AIF operation based on the control command
  * @{ */

Aif2Fl_Status  Aif2Fl_hwControl(
   Aif2Fl_Handle                      hAif2,
   Aif2Fl_HwControlCmd                ctrlCmd,
   void                               *arg
);

/**
@} */

/**
  * @ingroup AIF2FL_FUNCTION
  * @brief Get the status of different operations
  * @{ */

Aif2Fl_Status  Aif2Fl_getHwStatus(
   Aif2Fl_Handle                      hAif2,
   Aif2Fl_HwStatusQuery               Query,
   void                               *response
);
/**
@} */


#ifdef __cplusplus
}
#endif

#endif /* _AIF2FL_H_ */


