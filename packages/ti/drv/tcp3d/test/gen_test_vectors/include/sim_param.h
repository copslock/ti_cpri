/*
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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



#ifndef SIM_PARAM_H
#define SIM_PARAM_H
#include "typedefs.h"


/** 
 *  \struct   _TCP3_SIM_PARMS
 * 
 *  \brief   The structure holds main simulation parameters that are read from the configuration file
 * 
 *  \sa    
 *  
 */
typedef struct _TCP3_SIM_PARMS
{
    int32_t CodingStandard;               /**< 0 - UMTS, 1 - LTE, 2 - WiMAX, 3 - HSUPA Split mode  */   
    int32_t frameLenInd;                  /**< For LTE and WiMAX: index of the block size (LTE: 0-187, Wimax: 0-16), For UMTS/HSUPA block size (40-5114) */     
    int32_t MaxNumTurboIterations;        /**< Maximum number of turbo iterations  */   
    int32_t MinNumTurboIterations;        /**< Minimum number of turbo iterations  */   
    int32_t RateTimesOneTwenty;           /**< Currently not used  */              
    int32_t mappingSign;                  /**< Currently hardcoded to 1  */   

    int32_t NumInfoBits;                  /**< Number of info bits */   
    int32_t NumInfoSymb;                  /**< Half of the number of info bits */
    int32_t NumInfoBytes;                 /**< Number of info bytes */
    int32_t NumCodedBits;                 /**< Number of coded bits at the output of turbo encoder */
    int32_t NumTransmitBits;              /**< Number of transmitted bits (after rate matching) */


    int32_t MinNumFecBlocks;              /**< For BER calculation - maximum number of FEC blocks at one SNR point */    
    int32_t MaxNumFecBlocks;              /**< For BER calculation - minimum number of FEC blocks at one SNR point */
    int32_t MinNumFerErrors;              /**< For BER calculation - minimum number of FEC erros at one SNR point  */

    int32_t PrevFerComputed;              /**< FER calculated for the previous point*/
    float PrevFer;                      /**< FER of the previous snr point*/
    int32_t FrameErrorRateLimit;          /**< Exponent value of the minimum frame error rate of the FER curve. For example -4 means stop curve computation when FER=10^-4 is reached. */ 
    float SnrInitValue;                 /**< Initial SNR value in dB  */ 
    float SnrIncrementStep;             /**< SNR increment steps for BER calculation  */ 
    float SnrLimitValue;                /**< Stop BER calculation if snr reaches this value */
    float noiseSigma;                   /**< Noise RMS value */
    float SnrPointValue;                /**< Current SNR value during the calculation of BER/FER */
    int32_t AdaptiveSnrStep;              /**< =1 Enables adaptive SNR step. If the previous_FER/current_FER > 4 SNR step is divided in two */
    int32_t seed;                         /**< Noise seed */
    int32_t NoiseSwitch;                  /**< =1 nose addition to signal in the chanel enabled */
    uint32_t Scr2318ShiftReg;             /**< Initial value of the 23-bit scrambler shift register for info bit generation */


    uint32_t SnrPointCounter;             /**< Counts the SNR points during BER/FER curve calculation */
    uint32_t FecBlockCntrLimit;           /**< Maximum number of FEC blocks at the current SNR point */

    uint32_t AccBitErrors;                /**< Accumulated bit errors at the current SNR point */
    uint32_t AccFrameErrors;              /**< Accumulated frame errors at the current SNR point */
    uint32_t FecBlockCounter;             /**< FEC block counter at the current SNR point */
    uint32_t AccParity0BitErrors;                /**< Accumulated bit errors at the current SNR point */
    uint32_t AccParity1BitErrors;                /**< Accumulated bit errors at the current SNR point */

    int32_t ErrorProcessingOption;        /**< =1 coded error processing, =0 raw error processing */


    int32_t simulationState;              /**< simulaton state */

    int32_t bitWidthInt;                  /**< Input LLR bit width integer part including sign bit */
    int32_t bitWidthFrac;                 /**< Number of fractional bits of input LLR */
    int32_t maxStarEn;                    /**< =1 MaxStar enabled */
    int32_t maxStarThreshold;             /**< MaxStar thershold - width of the step function that approximates MaxStar correction term*/
    int32_t maxStarValue;                 /**< MaxStar value - hight of the step function that approximates MaxStar correction term  */

    int32_t alternateProcInd;             /**< =1 alternates process index (0 or 1) during BER/FER computation */
    int32_t initialProcInd;               /**< (0or 1) initial process index */


    int32_t extrinsicScales[16];          /**< Extrinsic scales for first 16 half iterations in Q5 format */
    int32_t tcp3_SW0_length;              /**< Sliding window 0 length in bits {16,32, 48, 64, 96 128} */
    int32_t tcp3_SNR_stopVal;             /**< SNR threshold in dB used as a stopping criterion (0-20) */
    int32_t tcp3_SNR_Report;              /**< =1 report SNR, =0 do not report SNR */
    int32_t tcp3_stopSel;                 /**< Stopping criteria: =0 Max iter, =1 CRC, =2 or 3  SNR */
    int32_t tcp3_crcSel;                  /**< CRC polynomial selector: =0 gCRC24B, =1 gCRC24A */
    int32_t tcp3_intlvGenEn;              /**< =0 Internal LTE interleaver table generation disabled, =1 enabled */
    int32_t tcp3_intlvLoadSel;            /**< =0 do not load or generate intlv table, =1 load or generate intlv table */
    int32_t tcp3_extrScaleEn;             /**< =0 Extrinsic scale disabled, =1 Extrinsics scale enabled */
    int32_t tcp3_softOutBitFormat;        /**< =0 - Soft output is truncated from 9 to 8-bits for storage in RAM (i.e. LSB truncated), =1 - Soft output is saturated from 9 to 8-bits for storage in RAM.*/
    int32_t tcp3_outBitOrderSel;          /**< =0 - LSB bit first in time, =1 - MSB bit first in time */
    int32_t tcp3_lteCrcInitSel;           /**< =0 Use 0x000000 as initial value, =1 Use 0xffffff as initial value  */
    int32_t tcp3_lteCrcIterPass;          /**< Select number of consecutive LTE CRC matches as stopping criterion =0: 1 match, =1: 2 matches, =2: 3 matches, =3: 4 matches */
    int32_t tcp3_softOutBitsReadEn;       /**< =1 sends out soft output bits */
    int32_t tcp3_outStatusReadEn;         /**< =1 sends out output status */


    /*For device verificaton*/
    int32_t enableTopLvlDeviceVerification; /**< =1 Top level device verification enabled, =0 disabled */
    int32_t enableDeviceVerification;     /**< =1 Device verification enabled, =0 disabled */
    int32_t disableBeliefPropagation;     /**< =1 Belief propagation enabled, =0 disabled */
    int32_t disableAlphaBeliefPropagation;/**< =1 Belief Alpha propagation enabled, =0 disabled */
    int32_t disableBetaBeliefPropagation; /**< =1 Belief Beta propagation enabled, =0 disabled */

    int32_t devVerAlphaBlockNumber;       /**< Alpha block number used for comparison in device verification */
    int32_t devVerBetaBlockNumber;        /**< Beta block number used for comparison in device verification*/
    int32_t devVerExtrBlockNumber;        /**< Extrinsic block number used for comparison in device verification*/
    int32_t devVerSendIntermediteInternalMemories;  /**< =1 send, =0 do not send */
    int32_t devVerUseLinearInterleaver;   /**< =1 interleaver is linear (for testing) =0 regular interleaver*/
    int32_t devVerZeroAprioriInHardDec;   /**< Only for device verification, =1 : soft decision = extrinisc + systematic (no apriori) */

    int32_t saveIntermediateData;         /**< =1 Intermediate data recording to files enabled, =0 disabled */
    int32_t beliefPropWithinTurboIterEnabled; /**< =1 Belief propagation within turbo iteration is enabled, =0 disabled, default = 0 */

    int32_t punctureInterval;             /**< Used for testing: punctures (punctureInterval-1) out of punctureInterval parity bits, =-1 puncturing disabled, default = -1 */
    int32_t enableRateMatching;           /**< =1 Rate matching enabled, =0 disabled, default = 0  */
    float codingRate;                   /**< If rate matching is enabled, coding rate */
    int32_t redundancyVersionNumber;      /**< Reundancy version number for LTE rate matching  */
    int32_t loadInfoBitsFromFile;         /**< =0 info bits generated internally, =1 info bits read from file  */
    int32_t infoBitsFileIncludesCRC;      /**< =1 File with info bits includes crc, =0 crc not included */
    char  infoBitsFileName[128];        /**< File name with the input bits to the encoder, used if loadInfoBitsFromFile=1 or storeInfoBitsToFile=1 */
    int32_t storeCodedBitsToFile;         /**< =1 coded bits stored to file, =0 not stored */
    int32_t storeInfoBitsToFile;          /**< =1 info bits stored to file, =0 not stored */
    char  codedBitsFileName[128];       /**< File name with the coded bits, used if storeCodedBitsToFile=1 */
    int32_t use_tcp3_encoder_c_model;     /**< =1 Use TCP3 encoder C model, =0 use existing encoders within the transmitter */
} TCP3_SIM_PARMS;


/** 
 *  \struct   _TCP3_SIM_PARMS
 * 
 *  \brief   The structure holds all tcp3d control and configuration register parameters
 * 
 *  \sa    
 *  
 */
typedef struct _TCP3_REGS
{
	int32_t mode_sel;                 //TCP3_MODE
	int32_t in_mem_db_en;  
	int32_t itg_en;
	int32_t err_ignore_en;
	int32_t auto_trig_en;
	int32_t lte_crc_init_sel;
	int32_t trig;                     //TRIGGER_REG
	int32_t endian_intr;              //TCP_ENDIAN
	int32_t endian_indata;
	int32_t exe_cmd;                  //TCP3_EXE
	int32_t num_sw0;                  //CFG0 
	int32_t blk_ln;
	int32_t sw1_ln;                   //CFG1
	int32_t sw2_ln_sel;
	int32_t sw0_ln_sel;
	int32_t inter_load_sel;           //CFG2
	int32_t maxst_en;
	int32_t out_flag_en;
	int32_t out_order_sel;
	int32_t ext_scale_en;
	int32_t soft_out_flag_en;
	int32_t soft_out_order_sel;
	int32_t soft_out_fmt;
	int32_t min_itr;
	int32_t max_itr;
	int32_t snr_val;
	int32_t snr_rep;
	int32_t stop_sel;
	int32_t crc_iter_pass;
	int32_t crc_sel;
	int32_t maxst_thold;              //CFG3
	int32_t maxst_value;
	int32_t beta_st0_map0;            //CFG4
	int32_t beta_st1_map0;
	int32_t beta_st2_map0;
	int32_t beta_st3_map0;
	int32_t beta_st4_map0;            //CFG5
	int32_t beta_st5_map0;
	int32_t beta_st6_map0;
	int32_t beta_st7_map0;
	int32_t beta_st0_map1;            //CFG6
	int32_t beta_st1_map1;
	int32_t beta_st2_map1;
	int32_t beta_st3_map1;
	int32_t beta_st4_map1;            //CFG7
	int32_t beta_st5_map1;
	int32_t beta_st6_map1;
	int32_t beta_st7_map1;
	int32_t ext_scale_0;              //CFG8
	int32_t ext_scale_1;
	int32_t ext_scale_2;
	int32_t ext_scale_3;
	int32_t ext_scale_4;              //CFG9
	int32_t ext_scale_5;
	int32_t ext_scale_6;
	int32_t ext_scale_7;
	int32_t ext_scale_8;              //CFG10
	int32_t ext_scale_9;
	int32_t ext_scale_10;
	int32_t ext_scale_11;
	int32_t ext_scale_12;             //CFG11
	int32_t ext_scale_13;
	int32_t ext_scale_14;
	int32_t ext_scale_15;
	int32_t itg_param_0;              //CFG12
	int32_t itg_param_1;
	int32_t itg_param_2;              //CFG13
	int32_t itg_param_3;
	int32_t itg_param_4;              //CFG14
    //Not part of tcp3 registers:
	int32_t proc_id;        
	int32_t ExtndNumInfoBits;
	int32_t NumInfoBits;
    int32_t IntlvLen;
    int32_t SW0_length;
} TCP3_REGS;




typedef struct _CODE_BLOCK_PARMS
{
    TCP3_SIM_PARMS sparms;
    TCP3_REGS reg;
    int8_t sysLLR[8192];
    int8_t par0LLR[8192];
    int8_t par1LLR[8192];
    int8_t tailLLRs[12];
    int16_t interleaver[8192];
    uint32_t ref_infoBits[256];
    uint32_t ref_hardDecisions[256];
    int8_t ref_softDecisions[3][8192];
    uint32_t ref_outStatus[3];

    uint32_t ssi_hardDecisions[256];
    int8_t ssi_softDecisions[3][8192];
    uint32_t ssi_outStatus[3];

    int32_t codeBlockNumber; //block number read from the codeBlockList file

    int32_t transferError;

} CODE_BLOCK_PARMS;


#endif