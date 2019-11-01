/** 
 *   @file  fftc_lld.h
 *
 *   @brief  
 *      Header file with data structure and API declarations for FFTC Low
 *      Level Driver (LLD).
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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

/** @defgroup FFTC_LLD_API FFTC LLD Data Structures & APIs
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *      The FFT Coprocessor (FFTC) is an accelerator that can be used to perform
 *      FFT and IFFT on data. Using the FFTC to perform computations that otherwise
 *      would have been done in software frees up CPU cycles for other tasks. The 
 *      FFTC module has been designed to be compatible with various OFDM based
 *      wireless standards like WiMax and LTE. 
 *
 *      The FFTC hardware provides the following features:
 *          -#  IFFT and FFT.
 *          -#  Sizes
 *              -#  2^a * 3^b for 2 >= a >= 13, 0 >= b >= 1 - maximum 8192.
 *              -#  12 * 2^a * 3^b * 5^c for sizes between 12 and 1296.
 *          -#  LTE 7.5KHz frequency shift.
 *          -#  16 bits I / 16 bits Q input and output.
 *          -#  444 Msubcarriers/sec throughput.
 *          -#  77 dB SNR.
 *          -#  Dynamic and Programmable Scaling modes.
 *          -#  Dynamic scaling mode returns block exponent.
 *          -#  Support for "FFT Shift" (switch left/right halves)
 *          -#  Support for cyclic prefix   (addition/removal)
 *          -#  Ping/Pong input, output buffers.
 *          -#  Input data scaling with shift.
 *          -#  Output data scaling.
 *
 *      This section of the documentation covers the FFTC Low Level Driver (LLD)
 *      APIs that include FFTC MMR access APIs and some utility APIs for formatting
 *      control headers and compiling FFT configuration for the hardware.
 *
 * @subsection References
 *  -#  FFTC User Guide
 */
#ifndef _FFTC_LLD_H_
#define _FFTC_LLD_H_

/* FFTC CSL Register file and CSL definitions include */
#include <ti/csl/csl.h>
#include <ti/csl/cslr_fftc.h>
#include <ti/csl/soc.h>

/**
@defgroup FFTC_LLD_SYMBOL  FFTC LLD Symbols Defined
@ingroup FFTC_LLD_API
*/

/**
@defgroup FFTC_LLD_DATASTRUCT  FFTC LLD Data Structures
@ingroup FFTC_LLD_API
*/

/**
@defgroup FFTC_LLD_FUNCTION  FFTC LLD Functions
@ingroup FFTC_LLD_API
*/

#ifdef __cplusplus
extern "C"
{
#endif

/**
@addtogroup FFTC_LLD_SYMBOL
@{
*/

/** @brief
 * The Maximum number of butterfly stages supported. 
 */	        
#define     FFTC_MAX_NUM_BUTTERFLY_STAGES           (7)        

/** @brief
 * The number of internal buffers used by FFTC for 
 * simultaneous input, output and calculation operations.
 * A complete FFTC Engine status snapshot can hence be obtained 
 * by putting together the status of all the three buffers.
 */
#define     FFTC_NUM_INTERNAL_BUFFERS               (3)                

/** @brief
 * The maximum number of FFT block sizes that can be
 * specified to the FFTC engine hardware.
 */
#define     FFTC_MAX_NUM_BLOCKS                     (128)        

/** @brief
 * The number of Transmit queues dedicated for FFTC from 
 * the CPPI-Queue Manager (QM).
 */
#define     FFTC_MAX_NUM_TXQUEUES                   (4)        

/** @brief
 * Default value for CPPI Destination queue number.
 *
 * Configure the destination CPPI queue number for 
 * any given FFTC queue as this value, and the default CPPI 
 * queue number from the CPPI DMA channel configuration is 
 * used. 
 */        
#define     FFTC_DEF_CPPI_QUEUE_NUM                 (0x3FFF)             

/** @brief
 * Default DFT size.
 *
 * Configure the 'DFT_size' field of the Queue X Control
 * Register to this value to indicate to the engine to
 * use DFT Size list to pick up block sizes.
 */        
#define     FFTC_DEF_DFT_SIZE                       (0x3F)        

/** @brief
 * Maximum number of PS Info words (32 bit words ) supported 
 * by the FFTC engine.
 */        
#define     FFTC_MAX_NUM_PS_WORDS                   (4)                 

/**
@}
*/

/** @addtogroup FFTC_DATASTRUCT
 @{ */

typedef CSL_FftcRegs*           CSL_FftcRegsOvly;           

typedef struct _Fftc_LldObj
{
    /** FFTC Peripheral instance number */
    uint8_t                     instNum;

    /** Register overlay pointer to access MMR for this FFTC instance */
    CSL_FftcRegsOvly            cfgRegs;
} Fftc_LldObj;

/** 
 *  @brief  Fftc_DFTMode
 *
 *          Enumeration for specifying the DFT/IDFT selection.
 */        
typedef enum   
{
    /** IDFT/IFFT mode */        
    Fftc_DFTMode_IDFT                               = 0,
    /** DFT/FFT mode */        
    Fftc_DFTMode_DFT                                = 1
} Fftc_DFTMode;

/** 
 *  @brief  Fftc_ZeroPadMode
 *
 *          Enumeration for specifying the zero pad mode.
 */        
typedef enum   
{
    /** Addition Mode */        
    Fftc_ZeroPadMode_ADD                            = 0,
    /** Multiplication Mode */        
    Fftc_ZeroPadMode_MULTIPLY                       = 1
} Fftc_ZeroPadMode;

/** 
 *  @brief  Fftc_FreqShiftDir
 *
 *          Enumeration for specifying the LTE Frequency
 *          Shift Direction, 0 for -1 and 1 for +1.
 */        
typedef enum   
{
    /** LTE Frequency Shift Direction - Plus */        
    Fftc_FreqShiftDir_PLUS                          = 0,
    /** LTE Frequency Shift Direction - Minus */        
    Fftc_FreqShiftDir_MINUS                         = 1
} Fftc_FreqShiftDir;

/** 
 *  @brief  Fftc_FreqShiftIndex
 *
 *          Enumeration for specifying the LTE Frequency Shift
 *          Table Index, M. There are only 2 valid values in the table,
 *          i.e., 8192 * 2 (16384) and 6144 * 2 (12288). Set to 0
 *          for M = 16384, and 1 for M = 12288.
 */        
typedef enum   
{
    /** LTE Frequency Shift Index - 8192 * 2 */        
    Fftc_FreqShiftIndex_16384                       = 0,
    /** LTE Frequency Shift Index - 6144 * 2 */        
    Fftc_FreqShiftIndex_12288                       = 1
} Fftc_FreqShiftIndex;

/** 
 *  @brief  Fftc_QueueId
 *
 *          Enumeration for specifying the 4 FFTC CPPI/QM Queues.
 */        
typedef enum   
{
    /** FFTC Tx Queue 0  */
    Fftc_QueueId_0                                  = 0,
    /** FFTC Tx Queue 1  */
    Fftc_QueueId_1                                  = 1,
    /** FFTC Tx Queue 2  */
    Fftc_QueueId_2                                  = 2,
    /** FFTC Tx Queue 3  */
    Fftc_QueueId_3                                  = 3
} Fftc_QueueId;

/** 
 *  @brief  Fftc_DestQRegCfg
 *
 *          Structure to specify/hold the CPPI Destination queue
 *          information stored in the FFTC hardware's Queue X 
 *          Destination Queue Register for a given FFTC queue.
 */     
typedef struct _Fftc_DestQRegCfg
{
    /** Boolean Flag, set to 1 to enable FFT shift at output
     *  (reverse left/right halves)
     *
     *  Corresponds to the 'FFT_shift_left_right_output' bit field 
     *  of the FFTC Queue x Destination Queue Register.
     */
    uint8_t                     bOutputFFTShift;        

    /** Boolean Flag, set to 1 to enable FFT shift at input
     *  (reverse left/right halves)
     *
     *  Corresponds to the 'FFT_shift_left_right_input' bit field 
     *  of the FFTC Queue x Destination Queue Register.
     */
    uint8_t                     bInputFFTShift;

    /** Input shift value to use.
     *
     *  Corresponds to the 'FFT_variable_shift_input' bitfield of the FFTC
     *  Queue x Scaling & Shifting Register.
     */
    uint32_t                    inputShiftVal;    

    /** CPPI Destination queue number where the 
     *  FFT computation result needs to be put.
     *
     *  Corresponds to 'dest_queue' bit field in the
     *  FFTC Queue X Destination Queue Register.
     *
     *  MUST be set to '0x3FFF' if using FFTC higher
     *  layer APIs to configure queue.
     */
    uint32_t                    cppiDestQNum;
} Fftc_DestQRegCfg;


/** 
 *  @brief  Fftc_ScalingShiftingRegCfg
 *
 *          Structure to specify/hold the FFTC Queue X Scaling and 
 *          Shifting Register configuration info for a given 
 *          FFTC queue.
 */     
typedef struct _Fftc_ScalingShiftingRegCfg
{
    /** Boolean flag, set to 1 to enable dynamic scaling, 
     *  0 for static scaling.
     *
     *  Corresponds to the 'dynamic_scaling_en' bit field of the
     *  FFTC Queue x Scaling & Shifting Register.
     */
    uint8_t                     bDynamicScaleEnable;    

    /** 8 bit number used as scaling factor, set to 0x80 to 
     *  disable scaling.
     *
     *  Corresponds to the 'output_scaling' bitfield of the FFTC
     *  Queue x Scaling & Shifting Register.
     */
    uint32_t                    outputScaleVal;    

    /** Applicable for static scaling mode only, the shift value 
     *  to use for each stage.
     *
     *  Corresponds to the 'stage_6_scaling' ... 'stage_0_scaling'
     *  bit fields of the FFTC Queue x scaling and shifting register.
     */
    uint32_t                    radixScalingVal [FFTC_MAX_NUM_BUTTERFLY_STAGES];

    /** For static scaling mode only, shift to use at the
     *  output stage.
     *
     *  Corresponds to the 'stage_out_scaling' bit field of the 
     *  FFTC Queue x scaling and shifting register.
     */
    uint32_t                    radixScalingValLast;    

    /** For static scaling mode only, the shift value to use 
     *  when doing an LTE frequency shift. Used only when LTE 
     *  Frequency Shift is enabled.
     *
     *  Corresponds to the 'stage_lte_shift_scaling' bitfield
     *  of the FFTC Queue x scaling and shifting register.
     */
    uint32_t                    freqShiftScaleVal;

} Fftc_ScalingShiftingRegCfg;


/** 
 *  @brief  Fftc_CyclicPrefixRegCfg
 *
 *          Structure to specify/hold the FFTC Queue X Cyclic Prefix 
 *          Register configuration info for a given FFTC queue.
 */     
typedef struct _Fftc_CyclicPrefixRegCfg
{
    /** Boolean Flag, set to 1 to enable Cyclic Prefix
     *  addition. Useful when sending data to antenna
     *  interface (AIF) directly.
     */
    uint8_t                     bCyclicPrefixAddEnable;

    /** Number of samples to use for cyclic prefix addition.
     *
     *  Corresponds to 'cyclic_prefix_addition' bit field of the
     *  FFTC Queue X Cyclic Prefix Register.
     *
     *  If cyclic prefix needs to be added to more than one
     *  block of the packet, then this value MUST be a 
     *  multiple of 4.
     */
    uint32_t                    cyclicPrefixAddNum;

    /** Boolean Flag, set to 1 to program FFTC to ignore
     *  samples in the beginning of a packet data sent to FFTC.
     *  Useful when receiving data from Antenna Interface (AIF).
     *
     *  Corresponds to 'cyclic_prefix_remove_en' bit field of the
     *  FFTC Queue X Cyclic Prefix Register.
     */
    uint8_t                     bCyclicPrefixRemoveEnable;

    /** Number of samples to ignore when Cyclic Prefix 
     *  removal is enabled.
     *
     *  Corresponds to 'cyclic_prefix_remove_offset' bit field of the
     *  FFTC Queue X Cyclic Prefix Register.
     */
    uint32_t                    cyclicPrefixRemoveNum;
    
} Fftc_CyclicPrefixRegCfg;


/** 
 *  @brief  Fftc_ControlRegCfg
 *
 *          Structure to specify/hold the FFTC Queue X Control 
 *          Register configuration info for a given FFTC queue.
 */     
typedef struct _Fftc_ControlRegCfg
{
    /** DFT Block size in bytes - size of the transform 
     * 
     *  Corresponds to the 'DFT_size' bit field of the FFTC 
     *  Queue X Control Register. A list of the DFT block sizes
     *  supported by the FFTC hardware is documented in the 
     *  FFTC User Guide.
     */
    uint32_t                    dftSize;        

    /** DFT/IDFT selection configuration.
     *
     *  Mode can be either 0 for an IFFT/IDFT, 1 for FFT/DFT. 
     * 
     *  Corresponds to the 'DFT_IDFT_select' bit field of the FFTC 
     *  Queue X Control Register.
     */
    Fftc_DFTMode                dftMode;

    /** Not used on FFTC, always set to zero. */
    uint8_t                     bEmulateDSP16x16;

    /** Boolean Flag, set to 1 to enable zero padding. */
    uint8_t                     bZeroPadEnable;

    /** Zero Pad Mode. Mode can be either "add" or "multiply" 
     *
     *  Corresponds to the 'zero_pad_mode' bit field of the FFTC
     *  Queue X Control Register.
     */
    Fftc_ZeroPadMode            zeroPadMode;    

    /** The number of samples to use for zero padding in "Add" mode 
     *  or the multiplication factor by which the oversampling needs
     *  to be done in "Multiply" mode for zero padding.
     *  Setting this to "0" disables zero padding in both modes.
     *
     *  In "add" mode, this value must be a multiple of 4. In "multiply"
     *  mode, this value must be set such that the data length is a
     *  multiple of 4. Please consult the user guide for a table of
     *  legal values.
     *
     *  Corresponds to the 'zero_pad_val' bit field of the FFTC
     *  Queue X Control Register.
     */
    uint32_t                    zeroPadFactor;

    /** Boolean flag, set to 1 to supress FFTC side info such as
     *  block exponent, clipping detection, error and tag being 
     *  output.
     *
     *  Corresponds to the 'supress_side_info' bit field of the FFTC
     *  Queue X Control Register.
     */
    uint8_t                     bSupressSideInfo;
        
    /** Boolean flag, set to 1 to reverse I/Q order assumend on the input.
     *
     *  Corresponds to the 'iq_order' bit field of the FFTC
     *  Queue X Control Register.
     */
    uint8_t                     bIqOrder;

    /** Boolean flag, set to 1 to use 8-bit I/Q format. The order convention
     *  is followed using iq_order.
     *
     *  The 8-bit sample mode has some restrictions
     *  1) Cyclic prefix removal is not supported.
     *  2) DFT size must be a multiple of 8 samples.
     *     DFT sizes NOT supported are  4, 12, 36, 60, 108, 180, 324,
     *                                  972, 540, 300, 900
     *  3) The resulting data length from zero padding must be a multiple of 8.
     *
     *  Corresponds to the 'iq_size' bit field of the FFTC
     *  Queue X Control Register.
     */
    uint8_t                     bIqSize;
        
} Fftc_ControlRegCfg;


/** 
 *  @brief  Fftc_FreqShiftRegCfg
 *
 *          Structure to specify/hold the FFTC Queue X LTE Frequency 
 *          Shift Register configuration info for a given FFTC queue.
 */     
typedef struct _Fftc_FreqShiftRegCfg
{
    /** Boolean Flag, set to 1 to enable LTE Frequency Shifting of 
     *  the sample at input.
     *
     *  Corresponds to the 'lte_freq_shift_en' bit field of the
     *  FFTC Queue X LTE Frequency Shift Register.
     */
    uint8_t                     bFreqShiftEnable;

    /** Twiddle factor to use. Valid values are 6144 * 2 / 8192 * 2 (M)
     *  Set to 0 for M = 16384 and 1 for M = 12288
     *
     *  Corresponds to the 'lte_freq_shift_index' bit field of the
     *  FFTC Queue X LTE Frequency Shift Register.
     */
    Fftc_FreqShiftIndex         freqShiftIndex;

    /** LTE Frequency Shift multiplication factor (a)
     *
     *  Corresponds to the 'lte_freq_shift_factor' bit field of the
     *  FFTC Queue X LTE Frequency Shift Register.
     */
    uint32_t                    freqShiftMultFactor;

    /** LTE Frequency Shift Phase offset (n0)
     *
     *  Corresponds to the 'lte_freq_shift_phase' bit field of the
     *  FFTC Queue X LTE Frequency Shift Register.
     */
    uint32_t                    freqShiftInitPhase;

    /** LTE Frequency Shift Direction (+ :- 0 / - :- 1).
     *
     *  Corresponds to the 'lte_freq_shift_dir' bit field of the
     *  FFTC Queue X LTE Frequency Shift Register.
     */
    Fftc_FreqShiftDir           freqShiftDirection;

} Fftc_FreqShiftRegCfg;


/** 
 *  @brief  Fftc_QLocalCfg
 *
 *          Structure to specify/hold the queue specific configuration 
 *          for a given FFTC queue.
 */        
typedef struct _Fftc_QLocalCfg
{
    /** Destination Queue register configuration instance. */        
    Fftc_DestQRegCfg            destQRegConfig;

    /** Scaling and Shifting register configuration instance. */        
    Fftc_ScalingShiftingRegCfg  scalingShiftingRegConfig;

    /** Cyclic Prefix register configurartion instance. */
    Fftc_CyclicPrefixRegCfg     cyclicPrefixRegConfig;

    /** Control register configuration instance. */
    Fftc_ControlRegCfg          controlRegConfig;

    /** LTE Frequency Shift register configuration instance. */
    Fftc_FreqShiftRegCfg        freqShiftRegConfig;

} Fftc_QLocalCfg;

/** 
 *  @brief  Fftc_GlobalCfg
 *          
 *          FFTC Global configuration structure to be used to
 *          hold/specify the configuration for the FFTC Configuration
 *          Register.
 */        
typedef struct _Fftc_GlobalCfg
{
    /** Flow ID to use for overriding CPPI packets received
     *  from FFTC queue n.
     *
     *  Corresponds to the 'qn_flowid_overwrite' bit field of
     *  the FFTC Configuration Register.
     */
    uint32_t                    queueFlowidOverwrite [FFTC_MAX_NUM_TXQUEUES];        

    /** The maximum amount of time beyond which the
     *  FFTC scheduler must service a queue. Values can
     *  range between "0x1" and "0xff". When set to "0"
     *  the starvation preventing mechanism in the scheduler
     *  will be disabled.
     *
     *  Corresponds to the 'starvation_period' bit field of
     *  the FFTC Configuration Register.
     */
    uint32_t                    starvationPeriodVal;

    /** FFTC Queue priority.
     *  Valid values are between "0" and "3", "0" being
     *  the highest.
     *
     *  Corresponds to the 'queue_n_priority' bit field of
     *  the FFTC Configuration Register where 0 <= n <= 3.
     */    
    uint32_t                    queuePriority [FFTC_MAX_NUM_TXQUEUES];

    /** Boolean flag, set to 1 to disable the FFT calculation. 
     *  Useful for debugging.
     *
     *  Corresponds to the 'FFT_disabled' bit field of the
     *  FFTC Configuration Register.
     */
    uint8_t                     bDisableFFT;

} Fftc_GlobalCfg;


/** 
 *  @brief  Fftc_ControlHdr
 *          
 *          Configuration structure that can be 
 *          used by the application/driver to setup
 *          a FFTC control header using the LLD APIs.
 */    
typedef struct _Fftc_ControlHdr
{
    /** Length of the protocol specific field that 
     *  should be forwarded to the receiver in 32-bit words. 
     *  Valid values can be between "1" and "4".
     */
    uint32_t                    psFieldLen;

    /** DFT size list length in 32-bit words. Can be between 
     *  "1" and "128".
     */
    uint32_t                    dftSizeListLen;

    /** Boolean Flag, set to 1 to indicate that there is
     *  a protocol specific field that must be forwarded to
     *  the receiver.
     */
    uint8_t                     bPSPassThruPresent;

    /** Boolean flag, set to 1 to indicate that the DFT 
     *  size list is configured.
     */
    uint8_t                     bDFTSizeListPresent;

    /** Boolean flag, set to 1 to indicate that the five
     *  local control registers are present, configuration
     *  that follows must be five 32-bit words long only.
     */
    uint8_t                     bLocalConfigPresent;

} Fftc_ControlHdr;


/** 
 *  @brief  Fftc_QLocalCfgParams
 *          
 *          Configuration structure that is used by the FFTC
 *          LLD *internally* to format the FFTC queue configuration
 *          provided by an application/driver to an acceptable
 *          format by the FFT Hardware.
 */
typedef struct _Fftc_QLocalCfgParams
{
    /** Holds the data that needs to be written to
     *  FFTC Queue x Destination Queue Register.
     */
    uint32_t                    queuexDestQ;

    /** Holds the data that needs to be written to
     *  FFTC Queue x Scaling & Shifting Register.
     */
    uint32_t                    queuexScaleShift;

    /** Holds the data that needs to be written to
     *  FFTC Queue x Cyclic Prefix Register.
     */
    uint32_t                    queuexCyclicPrefix;

    /** Holds the data that needs to be written to
     *  FFTC Queue x Control Register.
     */
    uint32_t                    queuexControl;

    /** Holds the data that needs to be written to
     *  FFTC Queue x LTE Frequency Shift Register.
     */
    uint32_t                    queuexLteFreq;        
} Fftc_QLocalCfgParams;

/** 
 *  @brief  Fftc_PeripheralIdParams
 *          
 *          Configuration structure that can be used
 *          to setup the FFTC Peripheral ID Register.
 */
typedef struct _Fftc_PeripheralIdParams
{
    /** Fixed module ID. */        
    uint32_t                    function;

    /** RTL Version Number (R) */
    uint32_t                    rtlVersion;

    /** Major Version Number (X) */
    uint32_t                    majorNum;

    /** Custom Version Number */
    uint32_t                    customNum;

    /** Minor Version Number (Y) */
    uint32_t                    minorNum;
} Fftc_PeripheralIdParams;

/** 
 *  @brief  Fftc_EmulationControlParams
 *          
 *          Configuration structure that can be used to
 *          setup the FFTC Emulation Control Parameters.
 */
typedef struct _Fftc_EmulationControlParams
{
    /** This bit specifies whether the FFTC H/w should
     * monitor the "emususp" or "emususp_rt" signal for
     * emulation suspend.
     * When set to:
     * 0 -  FFTC monitors "emususp" signal (and ignores "emususp_rt")
     * 1 -  FFTC monitors "emususp_rt" signal (and ignores "emususp")
     */
    uint8_t                     bEmuRtSel;

    /** This bit indicates whether FFTC should perform a 
     * hard stop / soft stop when emulation halt signal is asserted.
     * Set to 1 for emulation soft stop and 0 for emulation hard stop.
     */
    uint8_t                     bEmuSoftStop;

    /** This bit controls FFTC's response to Emulation suspend
     * signal that it has been programmed to monitor.
     * When set to:
     * 0 -  FFTC suspends according to mode specified by bEmuSoftStop 
     *      field.
     * 1 -  FFTC ignores suspend signal and operates normally.
     */
    uint8_t                     bEmuFreeRun;

} Fftc_EmulationControlParams;

/** 
 *  @brief  Fftc_ErrorParams
 *          
 *          Configuration structure that can be used to
 *          hold/setup the FFTC Emulation Control Parameters.
 */
typedef struct _Fftc_ErrorParams
{
    /** Interrupt on EOP is triggered when a CPPI descriptor
     *  that has the bit 2 of its PS field set is completed
     *  and written to Rx destination queue of FFTC in QM.
     */
    uint8_t                     bIsIntOnEOP;

    /** Debug Halt is triggered when a CPPI descriptor that
     *  has bit 1 of its PS field set and is received by the
     *  FFTC.
     */
    uint8_t                     bIsDebugHalt;

    /** This bit is set when FFTC detects an error in
     *  configuration of the incoming FFT block.
     */
    uint8_t                     bIsConfigWordError;

    /** This bit is set when FFTC runs out free buffers to use
     *  when trying to send a packet.
     */
    uint8_t                     bIsDescBufferError;

    /** This bit is set when the packet length is not a multiple
     *  of the FFT block length size.
     */
    uint8_t                     bIsEopError;

    /** This bit is set when the length of the configuration 
     *  field in the PS field is not as per the length specified
     *  in the FFTC control header.
     */
    uint8_t                     bIsConfigInvalError;
} Fftc_ErrorParams;

typedef Fftc_DestQRegCfg                            Fftc_DestQStatusReg;
typedef Fftc_ScalingShiftingRegCfg                  Fftc_ScalingShiftingStatusReg;
typedef Fftc_CyclicPrefixRegCfg                     Fftc_CyclicPrefixStatusReg;

/** 
 *  @brief  Fftc_ControlStatusReg
 *
 *          Structure to hold the contents of Block
 *          X Control Status Register.
 */     
typedef struct _Fftc_ControlStatusReg
{
    /** Reflects the DFT size used. In case a DFT list is
     *  used, this field will reflect the size used from the 
     *  DFT list. In case there is only one block, this size
     *  will be same as the size configured in FFTC Queue X
     *  Control register.
     * 
     *  Corresponds to the 'DFT_size_stat' bit field of the Block
     *  X Control Status register.
     */
    uint32_t                    dftSize;        

    /** DFT/IDFT selection status.
     *
     *  Mode can be either 0 for an IFFT/IDFT, 1 for FFT/DFT. 
     * 
     *  Corresponds to the 'DFT_IDFT_select' bit field of the Block
     *  X Control Status register.
     */
    Fftc_DFTMode                dftMode;

    /** Boolean flag, is set to 1 to indicate that FFTC is programmed to
     *  supress FFTC side info such as block exponent, clipping detection, 
     *  error and tag being output.
     *
     *  Corresponds to the 'supress_side_info' bit field of the Block
     *  X Control Status register.
     */
    uint8_t                     bSupressSideInfo;

    /** Indicates the queue number from which this block has 
     *  originated.
     *
     *  Corresponds to the 'input_queue_num' bit of Block X Control
     *  Status Register.
     */
    Fftc_QueueId                inputQNum; 

    /** Boolean flag, indicates if this block was the first block 
     *  of the packet.
     *
     *  Corresponds to the 'SOP' bit of  Block X Control
     *  Status Register.
     */
    uint8_t                     bIsSOP;

    /** Boolean flag, indicates if this block was the last block 
     *  of the packet.
     *
     *  Corresponds to the 'EOP' bit of  Block X Control
     *  Status Register.
     */
    uint8_t                     bIsEOP;

    /** Boolean flag, indicates whether this block for which the status 
     *  is being retrieved contains an error.
     *
     *  Corresponds to the 'block_error' field of the Block X
     *  Control Status Register.
     */
    uint8_t                     bIsBlockError;

    /** The number of samples used for zero padding in "Add" mode 
     *  or the multiplication factor by which the oversampling 
     *  was done in "Multiply" mode for zero padding on this block.
     *  "0" indicates that zero padding was disabled for the block.
     *
     *  Corresponds to the 'zero_pad_val' bit field of the Block X 
     *  Control Status Register.
     */
    uint32_t                    zeroPadFactor;

    /** Zero Pad Mode. Mode can be either "add" or "multiply".
     *
     *  Corresponds to the 'zero_pad_mode' bit field of the Block X
     *  Control Status Register.
     */
    Fftc_ZeroPadMode            zeroPadMode;    
        
} Fftc_ControlStatusReg;

typedef Fftc_FreqShiftRegCfg                        Fftc_FreqShiftStatusReg;


/** 
 *  @brief  Fftc_PktSizeStatusReg
 *
 *          Structure to hold the contents of Block
 *          X Packet Size Status Register.
 */     
typedef struct _Fftc_PktSizeStatusReg
{
    /** The entire packet size for this block.
     *
     *  Corresponds to the 'Packet_size' bit of the
     *  Block X Packet Size Status Register.
     */
    uint32_t                    pktSize;                

}Fftc_PktSizeStatusReg;

/** 
 *  @brief  Fftc_TagStatusReg
 *
 *          Structure to hold the contents of Block
 *          X Tag Status Register.
 */     
typedef struct _Fftc_TagStatusReg
{
    /** Destination tag used to identify packets
     *  for FFTC.
     *
     *  Corresponds to the 'dest_tag' bit of the
     *  Block X Tag Status Register.
     */
    uint32_t                    destTag;                

    /** Flow Id associated with this block.
     *
     *  Corresponds to the 'flow_id' bit of the
     *  Block X Tag Status Register.
     */
    uint32_t                    flowId;                

    /** Source Id associated with this block.
     *
     *  Corresponds to the 'src_id' bit of the
     *  Block X Tag Status Register.
     */
    uint32_t                    srcId;                

}Fftc_TagStatusReg;


/**
@}
*/

extern int32_t Fftc_mapDFTSizeToIndex (
    uint32_t                            dftBlockSize
);

extern int32_t Fftc_compileQueueLocalConfigParams (
    Fftc_QLocalCfg*                     pFFTLocalCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
);

extern int32_t Fftc_recompileQueueLocalDFTParams (
    int32_t                             dftSize, 
    Fftc_DFTMode                        dftMode, 
    uint8_t*                            pData
);

extern int32_t Fftc_recompileQueueLocalCyclicPrefixParams (
    int32_t                             cyclicPrefixLen, 
    uint8_t*                            pData
);

extern int32_t Fftc_createControlHeader (
    Fftc_ControlHdr*                    pFFTCfgCtrlHdr, 
    uint8_t*                            pData,
    uint32_t*                           pLen
);

extern int32_t Fftc_modifyLocalCfgPresentControlHeader (
    int32_t                             bLocalConfigPresent, 
    uint8_t*                            pData
);

extern int32_t Fftc_createDftSizeList (
    uint16_t*                           pDftSizeList, 
    uint32_t                            dftSizeListLen, 
    uint8_t*                            pData, 
    uint32_t*                           pLen
);

extern int32_t Fftc_lldOpen 
(
    uint8_t                             instNum,
    void*                               cfgRegs,
    Fftc_LldObj*                        pFFTCLldObj
);

extern int32_t Fftc_lldClose 
(
    Fftc_LldObj*                        pFFTCLldObj
);

extern int32_t Fftc_readPidReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_PeripheralIdParams*            pPIDCfg
);

extern int32_t Fftc_readGlobalConfigReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_GlobalCfg*                     pFFTGlobalCfg
);

extern int32_t Fftc_writeGlobalConfigReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_GlobalCfg*                     pFFTGlobalCfg
);

extern void Fftc_doSoftwareReset (
    Fftc_LldObj*                        pFFTCLldObj
);

extern void Fftc_doSoftwareContinue (
    Fftc_LldObj*                        pFFTCLldObj
);

extern int32_t Fftc_isHalted (
    Fftc_LldObj*                        pFFTCLldObj
);

extern int32_t Fftc_writeEmulationControlReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_EmulationControlParams*        pEmulationCfg
);

extern int32_t Fftc_readEmulationControlReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_EmulationControlParams*        pEmulationCfg
);

extern int32_t Fftc_writeEoiReg (
    Fftc_LldObj*                        pFFTCLldObj,
    int32_t                             eoiVal
);

extern int32_t Fftc_readEoiReg (
    Fftc_LldObj*                        pFFTCLldObj
);

extern void Fftc_clearQueueClippingDetectReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum
);

extern int32_t Fftc_readQueueClippingDetectReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum
);

extern int32_t Fftc_writeQueueConfigRegs (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum, 
    Fftc_QLocalCfg*                     pFFTLocalCfg
);

extern int32_t Fftc_readQueueConfigRegs (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum, 
    Fftc_QLocalCfg*                     pFFTLocalCfg
);

extern int32_t Fftc_writeDftSizeListGroupReg (
    Fftc_LldObj*                        pFFTCLldObj,
    uint16_t*                           pDftSizeList, 
    uint32_t                            dftSizeListLen
);

extern int32_t Fftc_readDftSizeListGroupReg (
    Fftc_LldObj*                        pFFTCLldObj,
    uint16_t*                           pDftSizeList
);

extern int32_t Fftc_readBlockDestQStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_DestQStatusReg*                pFFTDestQStatus
);

extern int32_t Fftc_readBlockShiftStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_ScalingShiftingStatusReg*      pFFTShiftStatus
);

extern int32_t Fftc_readBlockCyclicPrefixStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_CyclicPrefixStatusReg*         pFFTCyclicStatus
);

extern int32_t Fftc_readBlockControlStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_ControlStatusReg*              pFFTControlStatus
);

extern int32_t Fftc_readBlockFreqShiftStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_FreqShiftStatusReg*            pFFTFreqShiftStatus
);

extern int32_t Fftc_readBlockPktSizeStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_PktSizeStatusReg*              pFFTPktSizeStatus
);

extern int32_t Fftc_readBlockTagStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_TagStatusReg*                  pFFTTagStatus
);

extern int32_t Fftc_readErrorIntRawStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum,                 
    Fftc_ErrorParams*                   pErrorCfg
);

extern int32_t Fftc_clearErrorIntRawStatusReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum,                 
    Fftc_ErrorParams*                   pErrorCfg
);

extern int32_t Fftc_writeErrorIntEnableSetReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum,                 
    Fftc_ErrorParams*                   pErrorCfg
);

extern int32_t Fftc_readErrorIntEnableSetReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum,                 
    Fftc_ErrorParams*                   pErrorCfg
);

extern int32_t Fftc_clearErrorIntEnableReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum,                 
    Fftc_ErrorParams*                   pErrorCfg
);

extern int32_t Fftc_writeHaltOnErrorReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum,                 
    Fftc_ErrorParams*                   pErrorCfg
);

extern int32_t Fftc_readHaltOnErrorReg (
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_QueueId                        qNum,                 
    Fftc_ErrorParams*                   pErrorCfg
);
        
#ifdef __cplusplus
}
#endif

#endif  /* __FFTC_LLD_H__ */
