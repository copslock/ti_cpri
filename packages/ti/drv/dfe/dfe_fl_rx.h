/********************************************************************
* Copyright (C) 2012-2013 Texas Instruments Incorporated.
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

/**
 *  @defgroup DFE_FL_RX_API RX
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_rx.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_RX CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
/* =============================================================================
 * Revision History
 * ===============
 *
 *
 * =============================================================================
 */

/**
 * @defgroup DFE_FL_RX_SYMBOL DFE Rx Symbols
 * @ingroup DFE_FL_RX_API
 */

/**
 * @defgroup DFE_FL_RX_DATASTRUCT DFE Rx Data Structures
 * @ingroup DFE_FL_RX_API
 */

/**
 * @defgroup DFE_FL_RX_ENUM DFE Rx Enumverated Data Types
 * @ingroup DFE_FL_RX_API
 */

/**
 * @defgroup DFE_FL_RX_FUNCTION DFE Rx Functions
 * @ingroup DFE_FL_RX_API
 */

#ifndef _DFE_FL_RX_H_
#define _DFE_FL_RX_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/csl/cslr_dfe_rx.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/drv/dfe/dfe_fl_rxParams.h>

/**
 * @addtogroup DFE_FL_RX_SYMBOL
 * @{
 */
/// RX power meter result power is valid
#define DFE_FL_RX_POWMTR_RESULT_POWER_VALID        0x1u
/// RX power meter result magnitude square is valid
#define DFE_FL_RX_POWMTR_RESULT_MAGSQ_VALID        0x2u
/// RX power meter result histogram count1 is valid
#define DFE_FL_RX_POWMTR_RESULT_HISTCNT1_VALID     0x4u
/// RX power meter result histogram count2 is valid
#define DFE_FL_RX_POWMTR_RESULT_HISTCNT2_VALID     0x8u

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_RX_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
    ///init clock gate, state, and clear data
    DFE_FL_RX_CMD_CFG_INITS,

    /// set signal ganerator sync source
    DFE_FL_RX_CMD_SET_SIGGEN_SSEL,
    /// set checksum sync source
    DFE_FL_RX_CMD_SET_CHKSUM_SSEL,
    /// set signal generator mode
    DFE_FL_RX_CMD_SET_SIGGEN_MODE,
    /// config signal generator ramp
    DFE_FL_RX_CMD_CFG_SIGGEN_RAMP,
    /// config checksum
    DFE_FL_RX_CMD_CFG_CHKSUM,

    /// Set testbus top control    
    DFE_FL_RX_CMD_SET_TOP_TEST_CTRL,
    /// set testbus imbalance control
    DFE_FL_RX_CMD_SET_IMB_TEST_CTRL,
    /// set testbus feagc dc control
    DFE_FL_RX_CMD_SET_FEAGC_DC_TEST_CTRL,

    /// enable power meter interrupt
    DFE_FL_RX_CMD_ENB_POWMTR_INTR,
    /// disable power meter interrupt
    DFE_FL_RX_CMD_DIS_POWMTR_INTR,
    /// force generating power meter interrupt
    DFE_FL_RX_CMD_SET_FORCE_POWMTR_INTR,
    /// clear force generating power meter interrupt
    DFE_FL_RX_CMD_CLR_FORCE_POWMTR_INTR,
    /// clear power meter interrupt status
    DFE_FL_RX_CMD_CLR_POWMTR_INTR_STATUS,
    /// set power meter sync source
    DFE_FL_RX_CMD_SET_POWMTR_SSEL,
    /// config power meter global
    DFE_FL_RX_CMD_CFG_POWMTR_GLOBAL,
    /// set done reading power meter (HW interrupt mode)
    DFE_FL_RX_CMD_SET_POWMTR_HANDSHAKE_DONE,
    /// set read request to power meter (SW handshake mode)
    DFE_FL_RX_CMD_SET_POWMTR_HANDSHAKE_READ_REQ,
    /// config a power meter
    DFE_FL_RX_CMD_CFG_POWMTR,
    /// set one shot of power meter
    DFE_FL_RX_CMD_SET_ONE_SHOT_MODE,
    /// set meter mode of a power meter
    DFE_FL_RX_CMD_SET_METER_MODE,

    /// program switch bypass
    DFE_FL_RX_CMD_SET_SWITCH_BYPASS,
        
    /// set Rx NCO sync source
    DFE_FL_RX_CMD_SET_NCO_SSEL, 
    /// program Rx NCO bypass
    DFE_FL_RX_CMD_SET_NCO_BYPASS,
    /// enable disable Rx NCO dither
    DFE_FL_RX_CMD_ENB_NCO_DITHER,
    /// program Rx NCO frequency 
    DFE_FL_RX_CMD_CFG_NCO_FREQ,

    /// set Rx EQR sync source     
    DFE_FL_RX_CMD_SET_EQR_SSEL,
    /// program Rx EQR bypass
    DFE_FL_RX_CMD_SET_EQR_BYPASS,
    /// program Rx EQR bypass
    DFE_FL_RX_CMD_CFG_EQR_TAPS,
    /// program Rx EQR shift (gain)
    DFE_FL_RX_CMD_SET_EQR_SHIFT,

    /// set DC_GSG sync source
    DFE_FL_RX_CMD_SET_DC_GSG_SSEL,
    /// set DKACC sync source
    DFE_FL_RX_CMD_SET_DKACC_SSEL,
         
    DFE_FL_RX_CMD_MAX_VALUE
} DfeFl_RxHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /// read check sum result
    DFE_FL_RX_QUERY_CHKSUM_RESULT = 0,

    /// get power meter interrupt status
    DFE_FL_RX_QUERY_POWMTR_INTR_STATUS,
    /// get power meter read miss (HW interrupt mode)
    DFE_FL_RX_QUERY_POWMTR_HANDSHAKE_READ_MISS,
    /// get power meter read ACK (SW handshake mode)
    DFE_FL_RX_QUERY_POWMTR_HANDSHAKE_READ_ACK,
    /// read power meter result
    DFE_FL_RX_QUERY_POWMTR_RESULT,
    /// read power meter configuration
    DFE_FL_RX_QUERY_GET_POWMTR,
    
    /// get the NCO bypass
    DFE_FL_RX_QUERY_GET_NCO_BYPASS,

    /// get Rx NCO sync source 
    DFE_FL_RX_QUERY_NCO_SSEL,
     
    /// get Rx EQR shift value
    DFE_FL_RX_QUERY_EQR_SHIFT,
    
    DFE_FL_RX_QUERY_MAX_VALUE
} DfeFl_RxHwStatusQuery;

/** @brief DC Canceller device */
typedef enum
{
    /// DC canceller 0
    DFE_FL_RX_DC_0 = 0 ,
    /// DC canceller 1
    DFE_FL_RX_DC_1,
    /// DC canceller 2
    DFE_FL_RX_DC_2,
    /// DC canceller 3
    DFE_FL_RX_DC_3,
    
    /// DC canceller ALL
    DFE_FL_RX_DC_ALL = 0xF
} DfeFl_RxDc;

/** @brief sig-gen device */
typedef enum
{
    /// signal generator on I bus
    DFE_FL_RX_SIGGEN_I = 0,
        /// signal generator on Q bus
    DFE_FL_RX_SIGGEN_Q
} DfeFl_RxSiggenDev;

/** @brief sig-gen mode enable */
typedef enum
{
    /// signal generator enable mode, normal
    DFE_FL_RX_SIGGEN_MODE_ENABLE_NORMAL = 0 ,
    /// signal generator enable mode, sig-gen
    DFE_FL_RX_SIGGEN_MODE_ENABLE_SIGGEN
} DfeFl_RxSiggenModeEnable;

/** @brief sig-gen mode frame */
typedef enum
{
    /// signal generator frame mode, normal
    DFE_FL_RX_SIGGEN_MODE_FRAME_NORMAL = 0 ,
    /// signal generator frame mode, sig-gen
    DFE_FL_RX_SIGGEN_MODE_FRAME_SIGGEN
} DfeFl_RxSiggenModeFrame;

/** @brief sig-gen mode ramp */
typedef enum
{                                
    /// signal generator ramp mode, LFSR
    DFE_FL_RX_SIGGEN_MODE_RAMP_NORMAL_LFSR = 0 ,
    /// signal generator ramp mode, RAMP
    DFE_FL_RX_SIGGEN_MODE_RAMP_RAMP
} DfeFl_RxSiggenModeRamp;

/** @brief Top Test Ctrl */
typedef enum
{
    /// Rx testbus top select, off (normal mode)
    DFE_FL_RX_TOP_TEST_CTRL_RXN = 0 ,
    /// Rx testbus top select, BDC_F1
    DFE_FL_RX_TOP_TEST_CTRL_BDC_F1,
    /// Rx testbus top select, BDC_F2
    DFE_FL_RX_TOP_TEST_CTRL_BDC_F2,
    /// Rx testbus top select, IMB
    DFE_FL_RX_TOP_TEST_CTRL_IMB,
    /// Rx testbus top select, FEAGC
    DFE_FL_RX_TOP_TEST_CTRL_FEAGC_TEST,
    /// Rx testbus top select, FEAGC_IN
    DFE_FL_RX_TOP_TEST_CTRL_FEAGC_IN,
    /// Rx testbus top select, FEAGC_OUT
    DFE_FL_RX_TOP_TEST_CTRL_FEAGC_OUT,
    /// Rx testbus top select, DVGA_OUT
    DFE_FL_RX_TOP_TEST_CTRL_DVGA_OUT,
    /// Rx testbus top select, real to complex
    DFE_FL_RX_TOP_TEST_CTRL_BDC_R2C,
    /// Rx testbus top select, switch
    DFE_FL_RX_TOP_TEST_CTRL_BDC_SW,
    /// Rx testbus top select, BDC_NCO
    DFE_FL_RX_TOP_TEST_CTRL_BDC_NCO_MIX
} DfeFl_RxTestCtrl;

/** @brief IMB Test Ctrl */
typedef enum
{
    /// IMB testbus select, none
    DFE_FL_RX_IMB_TEST_CTRL_NO_SEL 		= 0x0 ,
    /// IMB testbus select, IMB_INPUT
    DFE_FL_RX_IMB_TEST_CTRL_INPUT			= 0x1,
    /// IMB testbus select, IMB_OUTPUT
    DFE_FL_RX_IMB_TEST_CTRL_OUTPUT			= 0x2,
    /// IMB testbus select, INTERNAL_S
    DFE_FL_RX_IMB_TEST_CTRL_INTERNAL_S		= 0x100,
    /// IMB testbus select, INTERNAL_DCI
    DFE_FL_RX_IMB_TEST_CTRL_INTERNAL_DCI	= 0x101,
    /// IMB testbus select, INTERNAL_DCQ
    DFE_FL_RX_IMB_TEST_CTRL_INTERNAL_DCQ	= 0x102,
    /// IMB testbus select, INTERNAL_RMS
    DFE_FL_RX_IMB_TEST_CTRL_INTERNAL_RMS	= 0x103,
    /// IMB testbus select, INTERNAL_DCMID
    DFE_FL_RX_IMB_TEST_CTRL_INTERNAL_DCMID	= 0x104,
    /// IMB testbus select, INTERNAL_EACC
    DFE_FL_RX_IMB_TEST_CTRL_INTERNAL_EACC	= 0x105,
    /// IMB testbus select, INTERNAL_E
    DFE_FL_RX_IMB_TEST_CTRL_INTERNAL_E		= 0x106,
    /// IMB testbus select, INTERNAL_WACC
    DFE_FL_RX_IMB_TEST_CTRL_INTERNAL_WACC	= 0x107
} DfeFl_RxImbTestCtrl;

/** @brief feagc Test Ctrl */
typedef enum
{
    /// feagc testbus select, lagecy GC5330
    DFE_FL_RX_FEAGC_TEST_CTRL_5330 		= 0x100 ,
    /// feagc testbus select, DCIN_REAL
    DFE_FL_RX_FEAGC_TEST_CTRL_DCIN_REAL	= 0x101,
    /// feagc testbus select, DCIN_IMAG
    DFE_FL_RX_FEAGC_TEST_CTRL_DCIN_IMAG	= 0x102,
    /// feagc testbus select, DCCORR_S0
    DFE_FL_RX_FEAGC_TEST_CTRL_DCCORR_S0	= 0x103,
    /// feagc testbus select, DCCORR_S1
    DFE_FL_RX_FEAGC_TEST_CTRL_DCCORR_S1	= 0x104,
    /// feagc testbus select, DCCORR_S2
    DFE_FL_RX_FEAGC_TEST_CTRL_DCCORR_S2	= 0x105,
    /// feagc testbus select, DCCORR_S3
    DFE_FL_RX_FEAGC_TEST_CTRL_DCCORR_S3	= 0x106,
    /// feagc testbus select, DCOUT
    DFE_FL_RX_FEAGC_TEST_CTRL_DCOUT		= 0x107,
    /// feagc testbus select, NOTCH_REAL
    DFE_FL_RX_FEAGC_TEST_CTRL_NOTCH_REAL	= 0x108,
    /// feagc testbus select, NOTCH_IMAG
    DFE_FL_RX_FEAGC_TEST_CTRL_NOTCH_IMAG	= 0x109,
    /// feagc testbus select, POWERIN
    DFE_FL_RX_FEAGC_TEST_CTRL_POWERIN		= 0x10a,
    /// feagc testbus select, ERRMAPIN
    DFE_FL_RX_FEAGC_TEST_CTRL_ERRMAPIN		= 0x10b,
    /// feagc testbus select, GAINLOOP
    DFE_FL_RX_FEAGC_TEST_CTRL_GAINLOOP		= 0x10c,
    /// feagc testbus select, PEAKCNT
    DFE_FL_RX_FEAGC_TEST_CTRL_PEAKCNT		= 0x10d,
    /// feagc testbus select, PEAKDECT
    DFE_FL_RX_FEAGC_TEST_CTRL_PEAKDECT		= 0x10e,
    /// feagc testbus select, PEAKCNTIN
    DFE_FL_RX_FEAGC_TEST_CTRL_PEAKCNTIN	= 0x10f
} DfeFl_RxFeagcDcTestCtrl;

/** @brief power meter device */
typedef enum
{
    /// RX power meter 0
    DFE_FL_RX_POWMTR_0 = 0,
    /// RX power meter 1
    DFE_FL_RX_POWMTR_1,
    /// RX power meter 2
    DFE_FL_RX_POWMTR_2,
    /// RX power meter 3
    DFE_FL_RX_POWMTR_3
} DfeFl_RxPowmtrDev;

/** @brief power meter read mode */
typedef enum
{
    /// Rx power meter read mode, INTERRUPT
    DFE_FL_RX_POWMTR_READ_MODE_INTERRUPT = 0,
    /// Rx power meter read mode, HANDSHAKE
    DFE_FL_RX_POWMTR_READ_MODE_HANDSHAKE
} DfeFl_RxPowmtrReadMode;

/** @brief Rx NCO device */
typedef enum
{
    /// Rx NCO 0
    DFE_FL_RX_NCO_0 = 0,
    /// Rx NCO 1
    DFE_FL_RX_NCO_1,
    /// Rx NCO 2
    DFE_FL_RX_NCO_2,
    /// Rx NCO 3
    DFE_FL_RX_NCO_3
} DfeFl_RxNcoDev;

/** @brief Rx EQR device */
typedef enum
{    
    /// Rx EQR 0
    DFE_FL_RX_EQR_0 = 0,
    /// Rx EQR 1
    DFE_FL_RX_EQR_1,
    /// Rx EQR 2
    DFE_FL_RX_EQR_2,
    /// Rx EQR 3
    DFE_FL_RX_EQR_3
} DfeFl_RxEqrDev;

/** @brief Rx EQR shift value */
typedef enum
{
    /// Rx EQR shift value, 0.5
    DFE_FL_RX_EQR_SHIFT_0P5 = 0,
    // Rx EQR shift value, 1.0
    DFE_FL_RX_EQR_SHIFT_1X,
    // Rx EQR shift value, 2.0
    DFE_FL_RX_EQR_SHIFT_2X,
    // Rx EQR shift value, 4.0
    DFE_FL_RX_EQR_SHIFT_4X
} DfeFl_RxEqrShift;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_RX_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_SET_SIGGEN_MODE
 */
typedef struct
{
    /// sig-gen device
    uint32_t sigGen;
    /// enable/disble
    uint32_t enable;
    /// frame mode
    uint32_t frame;
    /// ramp mode
    uint32_t ramp; 
    /// frame length           
    uint32_t frameLen;
} DfeFl_RxSiggenMode;

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_CFG_SIGGEN_RAMP
 */
typedef struct
{
    /// sig-gen device
    uint32_t sigGen;
    /// ramp start
    uint32_t rampStart;
    /// ramp stop
    uint32_t rampStop;
    /// ramp increment
    uint32_t rampIncr;
    /// pulse width
    uint32_t pulseWidth;    
} DfeFl_RxSiggenRampConfig;


/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_CFG_CHKSUM
 */
typedef struct
{
    /// checksum mode
    uint32_t chksumMode;
    /// config latency mode
    struct
    {
        /// stable length
        uint32_t stableLen;
        /// signal length
        uint32_t signalLen;
        /// channel select
        uint32_t chanSel;
    } latencyMode;
} DfeFl_RxChksumConfig;

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_CFG_POWMTR_GLOBAL
 */
typedef struct
{
    /// HW interrupt or SW handshake
    uint32_t readMode;
    /// histogram threshold 1
    uint32_t histThresh1;
    /// histogram threshold 2
    uint32_t histThresh2;
} DfeFl_RxPowmtrGlobalConfig;

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_SET_POWMTR_SSEL
 *      DFE_FL_RX_CMD_SET_POWMTR_HANDSHAKE_DONE
 *      DFE_FL_RX_CMD_SET_POWMTR_HANDSHAKE_READ_REQ
 *
 *      DFE_FL_RX_QUERY_POWMTR_INTR_STATUS
 *      DFE_FL_RX_QUERY_POWMTR_HANDSHAKE_READ_MISS
 *      DFE_FL_RX_QUERY_POWMTR_HANDSKAHE_READ_ACK
 *      
 */
typedef struct
{
    /// power meter device
    uint32_t powmtr;
    /// set or get value
    uint32_t data;
} DfeFl_RxPowmtrGeneric; 

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_CFG_POWMTR
 */
typedef struct
{
    /// power meter device
    uint32_t powmtr;
    /// delay from sync
    uint32_t syncDelay;
    /// samples for integration
    uint32_t nSamples;
    /// samples for interval
    uint32_t interval;
} DfeFl_RxPowmtrConfig;

/** @brief argument for runtime control,
 *      DFE_FL_RX_QUERY_POWMTR_RESULT
 */
typedef struct
{
    /// power meter device
    uint32_t      powmtr;
    /// valid bit flag
    uint32_t      valid;
    /// summation of power
    uint64_t  power;
    /// max peak
    uint64_t  magsq;
    /// count of samples that above threshold 1
    uint32_t      histcnt1;
    /// count of samples that above threshold 2
    uint32_t      histcnt2;
} DfeFl_RxPowmtrResult;

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_SET_NCO_SSEL
 *      DFE_FL_RX_QUERY_NCO_SSEL
 */
typedef struct
{
    /// sync select for dither
    uint32_t sselDither;
    /// sync select for phase
    uint32_t sselPhase;
    /// sync select for frequency
    uint32_t sselFreq;
} DfeFl_RxNcoSsel;

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_ENB_NCO_DITHER
 */
typedef struct
{
    /// nco device
    uint32_t nco;
    /// enable or disable
    uint32_t enable;
} DfeFl_RxNcoDitherConfig;

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_CFG_NCO_FREQ
 */
typedef struct
{
    /// nco device
    uint32_t nco;
    /// frequency word    
    uint64_t freqWord;
} DfeFl_RxNcoFreqWord;

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_SET_EQR_SSEL
 *      DFE_FL_RX_CMD_SET_EQR_SHIFT
 */
typedef struct
{
    /// eqr device
    uint32_t eqr;
    /// get or set value    
    uint32_t data;
} DfeFl_RxEqrGeneric;

/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_CFG_EQR_TAPS
 */
typedef struct
{
    /// eqr device
    uint32_t eqr;
    /// eqr taps for ii
    uint32_t iiTaps[DFE_FL_RX_EQR_LEN];    
    /// eqr taps for iq
    uint32_t iqTaps[DFE_FL_RX_EQR_LEN];    
    /// eqr taps for qi
    uint32_t qiTaps[DFE_FL_RX_EQR_LEN];    
    /// eqr taps for qq
    uint32_t qqTaps[DFE_FL_RX_EQR_LEN];    
} DfeFl_RxEqrTapsConfig;


/** @brief argument for runtime control,
 *      DFE_FL_RX_CMD_SET_DKACC_SSEL
 */
typedef struct
{
    /// DC device
    uint32_t dc;
    /// get or set value    
    uint32_t data;
} DfeFl_RxDcGeneric;


/** @brief overlay register pointer to BB instance
 */
typedef CSL_DFE_RX_REGS *DfeFl_RxRegsOvly;

/** @brief a Rx Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle       hDfe;
    
    /// pointer to register base address of a RX instance
    DfeFl_RxRegsOvly   regs;
   
    /// This is the instance of RX being referred to by this object
    DfeFl_InstNum         perNum;
        
} DfeFl_RxObj;

/** @brief handle pointer to BB object
 */
typedef DfeFl_RxObj *DfeFl_RxHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_RX_FUNCTION
 * @{
 */

//DfeFl_Status dfeFl_RxInit();

DfeFl_RxHandle dfeFl_RxOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_RxObj                *pDfeRxObj,
    DfeFl_InstNum                 rxNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_RxClose(DfeFl_RxHandle hDfeRx);

DfeFl_Status  dfeFl_RxHwControl
(
    DfeFl_RxHandle             hDfeRx,
    DfeFl_RxHwControlCmd       ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_RxGetHwStatus
(
    DfeFl_RxHandle             hDfeRx,
    DfeFl_RxHwStatusQuery      queryId,
    void                        *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_RX_H_ */
