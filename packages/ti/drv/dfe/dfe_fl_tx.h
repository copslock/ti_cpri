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
 *  @defgroup DFE_FL_TX_API TX
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_tx.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_TX CSL
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
 * @defgroup DFE_FL_TX_DATASTRUCT DFE Tx Data Structures
 * @ingroup DFE_FL_TX_API
 */

/**
 * @defgroup DFE_FL_TX_ENUM DFE Tx Enumverated Data Types
 * @ingroup DFE_FL_TX_API
 */

/**
 * @defgroup DFE_FL_TX_FUNCTION DFE Tx Functions
 * @ingroup DFE_FL_TX_API
 */

#ifndef _DFE_FL_TX_H_
#define _DFE_FL_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/csl/cslr_dfe_tx.h>
#include <ti/drv/dfe/dfe_fl.h>
//#include <ti/drv/dfe/dfe_fl_txParams.h>

/**
 * @addtogroup DFE_FL_TX_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
	/// configure tx inits, including ssel, init_clk_gate, init_state and clear_data
	DFE_FL_TX_CMD_CFG_INITS,
    /// set testbus select
    DFE_FL_TX_CMD_SET_TESTBUS_SEL,
    /// set check sum ssel
    DFE_FL_TX_CMD_SET_CHKSUM_SSEL,
    /// set siggen mode
    DFE_FL_TX_CMD_SET_SIGGEN_MODE,
    /// configure siggen ramp
    DFE_FL_TX_CMD_CFG_SIGGEN_RAMP,
    /// configure checksum
    DFE_FL_TX_CMD_CFG_CHKSUM,

    /// update resampler coefficients
    DFE_FL_TX_CMD_UPDATE_RSMP_COEFF,
    /// set resampler coefficients ssel
    DFE_FL_TX_CMD_SET_RSMP_CF_SSEL,
    /// set resampler init phase ssel
    DFE_FL_TX_CMD_SET_RSMP_INIT_PH_SSEL,
    /// set resampler antenna phase
    DFE_FL_TX_CMD_SET_RSMP_ANT_PH,
    /// set resampler init phase
    DFE_FL_TX_CMD_SET_RSMP_INIT_PH,
    /// set resampler mode
    DFE_FL_TX_CMD_SET_RSMP_MODE,
    /// set resampler state
    DFE_FL_TX_CMD_SET_RSMP_STATE,

    /// set mixer ssel
    DFE_FL_TX_CMD_SET_MIX_SSEL,
    /// set mixer frequency
    DFE_FL_TX_CMD_SET_MIX_FREQ,
    /// set mixer phase
    DFE_FL_TX_CMD_SET_MIX_PHASE,
    /// set mixer dither enable
    DFE_FL_TX_CMD_SET_MIX_DIT_EN,

    /// mux control for mixer
    DFE_FL_TX_CMD_SET_MC_MIX,
    /// mux control for resampler
    DFE_FL_TX_CMD_SET_MC_RSMP,
    /// mux control for buc
    DFE_FL_TX_CMD_SET_MC_BUC,

    /// clock control for resampler
    DFE_FL_TX_CMD_SET_CC_RSMP,
    /// clock control for buc
    DFE_FL_TX_CMD_SET_CC_BUC,
    /// clock control for mixer
    DFE_FL_TX_CMD_SET_CC_MIX,
    /// clock control for PA protection
    DFE_FL_TX_CMD_SET_CC_PA,

    /// DC offset ssel
    DFE_FL_TX_CMD_SET_DCOFF_SSEL,
    /// set DC offset
    DFE_FL_TX_CMD_SET_DC_OFFSETS,

    /// set add other tx
    DFE_FL_TX_CMD_SET_ADDB_EN,

    /// set buc coefficients
    DFE_FL_TX_CMD_SET_BUC_COEF,
    /// set buc two antenna
    DFE_FL_TX_CMD_SET_BUC_TWO_ANT,
    /// set buc cascade
    DFE_FL_TX_CMD_SET_BUC_CASCADE,
    /// set buc even symmetric
    DFE_FL_TX_CMD_SET_BUC_EVEN,
    /// set buc interpolating
    DFE_FL_TX_CMD_SET_BUC_INTERP,
    /// set buc add mux
    DFE_FL_TX_CMD_SET_BUC_ADD_MUX,
    /// set buc B path enable
    DFE_FL_TX_CMD_SET_BUC_B_PATH_ENABLE,
    /// set buc mux control
    DFE_FL_TX_CMD_SET_BUC_IN_MUX_CTL,

    /// set two antenna mode
    DFE_FL_TX_CMD_SET_PA_TWO_ANT_MODE,
    /// set factory test
    DFE_FL_TX_CMD_SET_PA_FACTORY_TEST,
    /// set PA protection ssel
    DFE_FL_TX_CMD_SET_PA_SSEL,
    /// set PA threshold
    DFE_FL_TX_CMD_SET_PA_TH,
    /// set PA IIR mu
    DFE_FL_TX_CMD_SET_PA_IIR_MU,
    /// set PA IIR threshold select
    DFE_FL_TX_CMD_SET_PA_IIR_TH_SEL,
    /// set PA IIR threshold value
    DFE_FL_TX_CMD_SET_PA_IIR_TH_VAL,
    /// set PA circular clipper threshold
    DFE_FL_TX_CMD_SET_PA_CC_TH,
    /// set PA peak threshold
    DFE_FL_TX_CMD_SET_PA_PEAK_TH,
    /// set PA peak counter threshold
    DFE_FL_TX_CMD_SET_PA_PEAK_CNT_TH,
    /// set PA peak gain threshold
    DFE_FL_TX_CMD_SET_PA_PEAK_GAIN_TH,
    /// set PA CC counter
    DFE_FL_TX_CMD_SET_PA_CC_CNT,
    /// set PA peak counter
    DFE_FL_TX_CMD_SET_PA_PEAK_CNT,
    /// set PA peak gain counter
    DFE_FL_TX_CMD_SET_PA_PEAKGAIN_CNT,
    /// set PA mask
    DFE_FL_TX_CMD_SET_PA_MASK,
    /// set PA interrupt
    DFE_FL_TX_CMD_SET_PA_INTERRUPT,
    /// set PA lutp
    DFE_FL_TX_CMD_SET_PA_LUTP,
    /// set PA lutq
    DFE_FL_TX_CMD_SET_PA_LUTQ,
    /// set PA lutt
    DFE_FL_TX_CMD_SET_PA_LUTT,

    /// set TDD time step
    DFE_FL_TX_CMD_SET_TIME_STEP,
    /// set TDD reset init
    DFE_FL_TX_CMD_SET_RESET_INT,
    /// set TDD period
    DFE_FL_TX_CMD_SET_TDD_PERIOD,
    /// set TDD on and off
    DFE_FL_TX_CMD_SET_TDD_ON_OFF,

    /// enable PA power interrupt
    DFE_FL_TX_CMD_ENB_PAPOWER_INTR,
    /// enable PA peak interrupt
    DFE_FL_TX_CMD_ENB_PAPEAK_INTR,
    /// disable PA power interrupt
    DFE_FL_TX_CMD_DIS_PAPOWER_INTR,
    /// disable PA peak interrupt
    DFE_FL_TX_CMD_DIS_PAPEAK_INTR,
    /// force set PA power interrupt
    DFE_FL_TX_CMD_SET_FORCE_PAPOWER_INTR,
    /// force set PA peak interrupt
    DFE_FL_TX_CMD_SET_FORCE_PAPEAK_INTR,
    /// clear set PA power interrupt
    DFE_FL_TX_CMD_CLR_FORCE_PAPOWER_INTR,
    /// clear set PA peak interrupt
    DFE_FL_TX_CMD_CLR_FORCE_PAPEAK_INTR,
    /// clear PA power interrupt status
    DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS,
    /// clear PA peak interrupt status
    DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS,
    
    /// set DPD bypass
    DFE_FL_TX_CMD_SET_DPD_BYPASS,
    /// set TX clock enable delay
    DFE_FL_TX_CMD_SET_TX_CKEN_DLY,

    DFE_FL_TX_CMD_MAX_VALUE
} DfeFl_TxHwControlCmd;

/** @brief query commands
 */
typedef enum
{
	/// get inits
	DFE_FL_TX_QUERY_GET_INITS = 0,
	/// get test bus select
	DFE_FL_TX_QUERY_GET_TESTBUS_SEL,
	/// get the checksum result
	DFE_FL_TX_QUERY_GET_CHKSUM_RESULT,

	/// get the resampler coefficients
    DFE_FL_TX_QUERY_GET_RSMP_COEFF,
    /// get the resampler coefficients ssel
    DFE_FL_TX_QUERY_GET_RSMP_CF_SSEL,
    /// get the resampler init phase ssel
    DFE_FL_TX_QUERY_GET_RSMP_INIT_PH_SSEL,
    /// get the resampler antenna phase
    DFE_FL_TX_QUERY_GET_RSMP_ANT_PH,
    /// get the resampler init phase
    DFE_FL_TX_QUERY_GET_RSMP_INIT_PH,
    /// get the resampler mode
    DFE_FL_TX_QUERY_GET_RSMP_MODE,
    /// get the resampler state
    DFE_FL_TX_QUERY_GET_RSMP_STATE,

    /// get mixer ssel
    DFE_FL_TX_QUERY_GET_MIX_SSEL,
    /// get mixer frequency
    DFE_FL_TX_QUERY_GET_MIX_FREQ,
    /// get mixer phase
    DFE_FL_TX_QUERY_GET_MIX_PHASE,
    /// get mixer dither enable
    DFE_FL_TX_QUERY_GET_MIX_DIT_EN,

    /// get the mux control for mix
    DFE_FL_TX_QUERY_GET_MC_MIX,
    /// get the mux control for resampler
    DFE_FL_TX_QUERY_GET_MC_RSMP,
    /// get the mux control for buc
    DFE_FL_TX_QUERY_GET_MC_BUC,

    /// get the clock control for resampler
    DFE_FL_TX_QUERY_GET_CC_RSMP,
    /// get the clock control for buc
    DFE_FL_TX_QUERY_GET_CC_BUC,
    /// get the clock control for mixer
    DFE_FL_TX_QUERY_GET_CC_MIX,
    /// get the clock control for PA protection
    DFE_FL_TX_QUERY_GET_CC_PA,

    /// get the dc offset ssel
    DFE_FL_TX_QUERY_GET_DCOFF_SSEL,
    /// get dc offsets
    DFE_FL_TX_QUERY_GET_DC_OFFSETS,
    /// get add another tx
    DFE_FL_TX_QUERY_GET_ADDB_EN,

    /// get buc coeffs
    DFE_FL_TX_QUERY_GET_BUC_COEF,
    /// get buc two antenna
    DFE_FL_TX_QUERY_GET_BUC_TWO_ANT,
    /// get buc cascade
    DFE_FL_TX_QUERY_GET_BUC_CASCADE,
    /// get buc even symmetric
    DFE_FL_TX_QUERY_GET_BUC_EVEN,
    /// get buc interpolation
    DFE_FL_TX_QUERY_GET_BUC_INTERP,
    /// get buc add mux
    DFE_FL_TX_QUERY_GET_BUC_ADD_MUX,
    /// get buc B path enable
    DFE_FL_TX_QUERY_GET_BUC_B_PATH_ENABLE,
    /// get buc mux control
    DFE_FL_TX_QUERY_GET_BUC_IN_MUX_CTL,

    /// get PA two antenna mode
    DFE_FL_TX_QUERY_GET_PA_TWO_ANT_MODE,
    /// get PA factory test
    DFE_FL_TX_QUERY_GET_PA_FACTORY_TEST,
    /// get PA ssel
    DFE_FL_TX_QUERY_GET_PA_SSEL,
    /// get PA threshold
    DFE_FL_TX_QUERY_GET_PA_TH,
    /// get PA IIR mu
    DFE_FL_TX_QUERY_GET_PA_IIR_MU,
    /// get PA IIR threshold select
    DFE_FL_TX_QUERY_GET_PA_IIR_TH_SEL,
    /// get PA IIR threshold value
    DFE_FL_TX_QUERY_GET_PA_IIR_TH_VAL,
    /// get PA circular clipper threshold
    DFE_FL_TX_QUERY_GET_PA_CC_TH,
    /// get PA peak threshold
    DFE_FL_TX_QUERY_GET_PA_PEAK_TH,
    /// get PA peak counter threshold
    DFE_FL_TX_QUERY_GET_PA_PEAK_CNT_TH,
    /// get PA peak gain threshold
    DFE_FL_TX_QUERY_GET_PA_PEAK_GAIN_TH,
    /// get PA CC counter
    DFE_FL_TX_QUERY_GET_PA_CC_CNT,
    /// get PA peak counter
    DFE_FL_TX_QUERY_GET_PA_PEAK_CNT,
    /// get PA peak gain counter
    DFE_FL_TX_QUERY_GET_PA_PEAKGAIN_CNT,
    /// get PA mask
    DFE_FL_TX_QUERY_GET_PA_MASK,
    /// get PA interrupt
    DFE_FL_TX_QUERY_GET_PA_INTERRUPT,
    /// get PA max magnitude since last read, clear after read
    DFE_FL_TX_QUERY_GET_PA_MAXMAG_CLR,
    /// get PA max magnitude since last read, no clear after read
    DFE_FL_TX_QUERY_GET_PA_MAXMAG_NO_CLR,
    /// get PA d50
    DFE_FL_TX_QUERY_GET_PA_D50,
    /// get PA d51
    DFE_FL_TX_QUERY_GET_PA_D51,
    /// get PA lutp
    DFE_FL_TX_QUERY_GET_PA_LUTP,
    /// get PA lutq
    DFE_FL_TX_QUERY_GET_PA_LUTQ,
    /// get PA lutt
    DFE_FL_TX_QUERY_GET_PA_LUTT,

    /// get TDD time step
    DFE_FL_TX_QUERY_GET_TIME_STEP,
    /// get TDD reset init
    DFE_FL_TX_QUERY_GET_RESET_INT,
    /// get TDD period
    DFE_FL_TX_QUERY_GET_TDD_PERIOD,
    /// get TDD on and off
    DFE_FL_TX_QUERY_GET_TDD_ON_OFF,

    /// get PA power interrupt status
    DFE_FL_TX_QUERY_GET_PAPOWER_INTR_STATUS,
    /// get PA peak interrupt status
    DFE_FL_TX_QUERY_GET_PAPEAK_INTR_STATUS,
    /// get DPD bypass
    DFE_FL_TX_QUERY_GET_DPD_BYPASS,
    /// get tx clock enable delay
    DFE_FL_TX_QUERY_GET_TX_CKEN_DLY,

    DFE_FL_TX_QUERY_MAX_VALUE
} DfeFl_TxHwStatusQuery;

/** @brief sig-gen mode frame */
typedef enum
{
    /// siggen mode normal
	DFE_FL_TX_SIGGEN_MODE_ENABLE_NORMAL = 0 ,
	/// siggen mode enable
    DFE_FL_TX_SIGGEN_MODE_ENABLE_SIGGEN
} DfeFl_TxSiggenModeEnable;

/** @brief sig-gen mode frame */
typedef enum
{
    /// siggen frame normal
	DFE_FL_TX_SIGGEN_MODE_FRAME_NORMAL = 0 ,
	/// siggen frame enable
    DFE_FL_TX_SIGGEN_MODE_FRAME_SIGGEN
} DfeFl_TxSiggenModeFrame;

/** @brief sig-gen mode ramp */
typedef enum
{
    /// LFSR
	DFE_FL_TX_SIGGEN_MODE_RAMP_NORMAL_LFSR = 0 ,
	/// RAMP
    DFE_FL_TX_SIGGEN_MODE_RAMP_RAMP
} DfeFl_TxSiggenModeRamp;

/** @brief tx path */
typedef enum
{
    /// tx path A
	DFE_FL_TXA = 0,
	/// tx path B
    DFE_FL_TXB
} DfeFl_TxPath;

/** @brief tx device */
typedef enum
{
    /// TXA path A
	DFE_FL_TXA_PATHA = 0,
	/// TXA path B
    DFE_FL_TXA_PATHB,
    /// TXB path A
    DFE_FL_TXB_PATHA,
    /// TXB path B
    DFE_FL_TXB_PATHB
} DfeFl_TxDev;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_TX_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_TX_CMD_SET_SIGGEN_MODE
 */
typedef struct
{
    /// enable data generation
	uint32_t enable;
	/// enable frame generation
    uint32_t frame;
    /// ramp (1), or LFSR (0)
    uint32_t mode;
    /// seed
    uint32_t seed;
    /// number of clocks per frame minus 1
    uint32_t frameLenM1;
} DfeFl_TxSiggenMode;

/** @brief argument for runtime control,
 *      DFE_FL_TX_CMD_CFG_SIGGEN_RAMP
 */
typedef struct
{
    /// ramp start
    uint32_t rampStart;
    /// ramp stop
    uint32_t rampStop;
    /// ramp slope
    uint32_t rampSlope;
    /// gen timer
    uint32_t genTimer;
} DfeFl_TxSiggenRampConfig;


/** @brief argument for runtime control,
 *      DFE_FL_TX_CMD_CFG_CHKSUM
 */
typedef struct
{
    /// checksum mode
	uint32_t chksumMode;
	/// latency mode config
    struct
    {
        /// stable length
		uint32_t stableLen;
		/// signal length
        uint32_t signalLen;
        /// channel select
        uint32_t chanSel;
    } latencyMode;
} DfeFl_TxChksumConfig;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_RSMP_SSEL
 *    DFE_FL_TX_CMD_SET_MIX_SSEL
 *    DFE_FL_TX_CMD_SET_DCOFF_SSEL
 *    DFE_FL_TX_CMD_SET_PA_SSEL
 */
typedef struct
{
	/// tx path
	DfeFl_TxPath txPath;
	/// ssel
	uint32_t ssel;
} DfeFl_TxSsel;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_TESTBUS_SEL
 */
typedef struct
{
	/// tx path
	DfeFl_TxPath txPath;
	/// select
	uint32_t sel;
} DfeFl_TxTestBusSel;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_RSMP_MODE
 *    DFE_FL_TX_CMD_SET_RSMP_STATE
 *    DFE_FL_TX_CMD_SET_CC_MIX
 *    DFE_FL_TX_CMD_SET_CC_PA
 */
typedef struct
{
	/// tx path
	DfeFl_TxPath txPath;
	/// data
	uint32_t data;
} DfeFl_TxPathData;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_MIX_FREQ
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// lower 32 bits frequency
	uint32_t lower32;
	/// upper 16 bits frequency
	uint32_t upper16;
} DfeFl_TxMixFreq;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_MIX_PHASE
 *    DFE_FL_TX_CMD_SET_RSMP_ANT_PH
 *    DFE_FL_TX_CMD_SET_RSMP_INIT_PH
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// phase
	uint32_t phase;
} DfeFl_TxPhase;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_UPDATE_RSMP_COEFF
 *    DFE_FL_TX_QUERY_RSMP_COEFF
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// number of coeffs
	uint32_t numCoeff;
	/// coeffs
	uint32_t coeff[96];
} DfeFl_TxRsmpCoeff;

/** @brief argument for runtime control,
 * 	  DFE_FL_TX_CMD_SET_MC_MIX
 */
typedef struct
{
	/// tx path
	DfeFl_TxPath txPath;
	/// mix i
	uint32_t mixi;
	/// mix q
	uint32_t mixq;
} DfeFl_TxMcMix;

/** @brief argument for runtime control,
 * 	  DFE_FL_TX_CMD_SET_MC_RSMP
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// resample a
	uint32_t rsmpa;
	/// resample b
	uint32_t rsmpb;
} DfeFl_TxMcRsmp;

/** @brief argument for runtime control,
 * 	  DFE_FL_TX_CMD_SET_MC_BUC
 */
typedef struct
{
	/// tx path
	DfeFl_TxPath txPath;
	/// buc ie
	uint32_t bucie;
	/// buc il
	uint32_t bucil;
	/// buc qe
	uint32_t bucqe;
	/// buc ql
	uint32_t bucql;
} DfeFl_TxMcBuc;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_CC_RSMP
 *    DFE_FL_TX_CMD_SET_CC_BUC
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// data
	uint32_t data;
} DfeFl_TxDevData;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_DC_OFFSETS
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// dc offset I
	uint32_t dcOffsetI;
	/// dc offset Q
	uint32_t dcOffsetQ;
} DfeFl_TxDcOffsets;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_BUC_COEF
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// number of coeffs
	uint32_t numCoeff;
	/// coeffs
	uint32_t coeff[15];
} DfeFl_TxBucCoeff;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_BUC_IN_MUX_CTL
 */
typedef struct
{
	/// tx paht
	DfeFl_TxPath txPath;
	/// in mux ab
	uint32_t in_mux_ab;
	/// in mux ba
	uint32_t in_mux_ba;
	/// in mux bb
	uint32_t in_mux_bb;
} DfeFl_TxBucInMuxCtl;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_PA_TH
 *    DFE_FL_TX_CMD_SET_PA_CC_TH
 *    DFE_FL_TX_CMD_SET_PA_PEAK_TH
 *    DFE_FL_TX_CMD_SET_PA_PEAK_CNT_TH
 *    DFE_FL_TX_CMD_SET_PA_PEAK_GAIN_TH
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// threshold
	uint32_t theshld;
} DfeFl_TxPaTh;

/** @brief argument for runtime control,
 *    DFE_FL_TX_QUERY_PA_CC_CNT
 *    DFE_FL_TX_QUERY_PA_PEAK_CNT
 *    DFE_FL_TX_QUERY_PA_PEAKGAIN_CNT
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// counter
	uint32_t cnt;
} DfeFl_TxPaCnt;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_PA_IIR_MU
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// mu0
	uint32_t mu0;
	/// mu1
	uint32_t mu1;
} DfeFl_TxPaIirMu;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_PA_IIR_TH_SEL
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// th1 select
	uint32_t th1Sel;
	/// th2 select
	uint32_t th2Sel;
	/// th6 select
	uint32_t th6Sel;
} DfeFl_TxPaIirThSel;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_PA_IIR_TH_VAL
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// threshold TH1
	uint32_t TH1;
	/// threshold TH2
	uint32_t TH2;
	/// threshold TH4
	uint32_t TH4;
} DfeFl_TxPaIirThVal;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_PA_MASK
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// mask
	uint32_t mask;
} DfeFl_TxPaMask;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_PA_INTERRUPT
 *    DFE_FL_TX_QUERY_PA_INTERRUPT
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// stop dpd interrupt
	uint32_t stopDpd;
	/// interrupt 1
	uint32_t intrpt1;
	/// interrupt 2
	uint32_t intrpt2;
	/// interrupt 3
	uint32_t intrpt3;
	/// interrupt 6
	uint32_t intrpt6;
	/// shut down interrupt
	uint32_t shutdown;
	/// lower cfr gain interrupt
	uint32_t lowerCfrGain;
} DfeFl_TxPaIntrpt;

/** @brief argument for runtime control,
 *    DFE_FL_TX_QUERY_GET_PAPOWER_INTR_STATUS,
 *    DFE_FL_TX_QUERY_GET_PAPEAK_INTR_STATUS
 */
typedef struct
{
	/// tx device
	DfeFl_TxDev txDev;
	/// status
	uint32_t status;
} DfeFl_TxIntrStatus;

/** @brief argument for runtime control,
 *    DFE_FL_TX_CMD_SET_PA_LUTP
 *    DFE_FL_TX_CMD_SET_PA_LUTQ
 *    DFE_FL_TX_CMD_SET_PA_LUTT
 */
typedef struct
{
	/// tx path
	DfeFl_TxPath txPath;
	/// number of lut
	uint32_t numLut;
	/// pointer to lut data
	uint32_t *lut_data;
} DfeFl_TxPaLut;

/** @brief argument for runtime control,
 *     DFE_FL_TX_CMD_SET_TDD_ON_OFF
 */
typedef struct
{
	/// tx path
	DfeFl_TxPath txPath;
	/// tdd on 0
	uint32_t tdd_on_0;
	/// tdd off 0
	uint32_t tdd_off_0;
	/// tdd on 1
	uint32_t tdd_on_1;
	/// tdd off 1
	uint32_t tdd_off_1;
} DfeFl_TxTddOnOff;

/** @brief overlay register pointer to TX instance
 */
typedef CSL_DFE_TX_REGS *DfeFl_TxRegsOvly;

/** @brief a TX Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle       hDfe;

    /// pointer to register base address of a TX instance
	DfeFl_TxRegsOvly	regs;

    /// This is the instance of TX being referred to by this object
    DfeFl_InstNum         perNum;

} DfeFl_TxObj;

/** @brief handle pointer to TX object
 */
typedef DfeFl_TxObj *DfeFl_TxHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_TX_FUNCTION
 * @{
 */

DfeFl_TxHandle dfeFl_TxOpen
(
	    DfeFl_Handle               hDfe,
	    DfeFl_TxObj                *pDfeTxObj,
	    DfeFl_InstNum                 txNum,
	    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_TxClose(DfeFl_TxHandle hDfeTx);

DfeFl_Status  dfeFl_TxHwControl
(
    DfeFl_TxHandle             hDfeTx,
    DfeFl_TxHwControlCmd       ctrlCmd,
    void                         *arg
);

DfeFl_Status  dfeFl_TxGetHwStatus
(
    DfeFl_TxHandle             hDfeTx,
    DfeFl_TxHwStatusQuery      queryId,
    void                         *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_TX_H_ */
