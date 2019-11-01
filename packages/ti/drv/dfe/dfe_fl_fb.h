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
 *  @defgroup DFE_FL_FB_API FB
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_fb.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_FB CSL
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
 * @defgroup DFE_FL_FB_DATASTRUCT DFE Fb Data Structures
 * @ingroup DFE_FL_FB_API
 */

/**
 * @defgroup DFE_FL_FB_ENUM DFE Fb Enumverated Data Types
 * @ingroup DFE_FL_FB_API
 */

/**
 * @defgroup DFE_FL_FB_FUNCTION DFE Fb Functions
 * @ingroup DFE_FL_FB_API
 */

#ifndef _DFE_FL_FB_H_
#define _DFE_FL_FB_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/csl/cslr_dfe_fb.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/drv/dfe/dfe_fl_fbParams.h>

/**
 * @addtogroup DFE_FL_FB_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
	/// configure FB inits, including ssel, clk_gate, init_state and clear_data
	DFE_FL_FB_CMD_CFG_INITS = 0,
	/// set FB clock enable delay
    DFE_FL_FB_CMD_SET_INITS_CKENDLY,
    /// set top test bus
    DFE_FL_FB_CMD_SET_TESTBUS,
    /// set dc test bus control
    DFE_FL_FB_CMD_SET_DC_TESTBUS,
    /// set signal generator ssel
    DFE_FL_FB_CMD_SET_SIGGEN_SSEL,
    /// set checksum ssel
    DFE_FL_FB_CMD_SET_CHKSUM_SSEL,
    /// set signal generator mode
    DFE_FL_FB_CMD_SET_SIGGEN_MODE,
    /// set signal generator ramp
    DFE_FL_FB_CMD_CFG_SIGGEN_RAMP,
    /// configure checksum
    DFE_FL_FB_CMD_CFG_CHKSUM,

    /// set FB IO control
    DFE_FL_FB_CMD_SET_IO_CTRL,
    /// set FB IO Mux
    DFE_FL_FB_CMD_SET_IO_MUX,

    /// set dc canceller ssel
    DFE_FL_FB_CMD_SET_DKACC_SSEL,
    /// set dc gsg ssel
    DFE_FL_FB_CMD_SET_DC_GSG_SSEL,
    /// set dc global control
    DFE_FL_FB_CMD_SET_DC_GLOBAL,
    /// set dc intervals
    DFE_FL_FB_CMD_SET_DC_INTERVAL,
    /// set dc canceller update delay
    DFE_FL_FB_CMD_SET_DC_DELAY,
    /// set dc canceller shift and mode control
    DFE_FL_FB_CMD_SET_DC_SHIFT_MODE,
    /// set dc init
    DFE_FL_FB_CMD_SET_DC_INIT,

    /// set real-to-complex ssel
    DFE_FL_FB_CMD_SET_R2C_SSEL,
    /// set real-to-complex mode control
    DFE_FL_FB_CMD_SET_R2C_REALIN,
    /// set real-to-complex spectral inversion control
    DFE_FL_FB_CMD_SET_R2C_SPECINV,

    /// set equalizer ssel
    DFE_FL_FB_CMD_SET_EQR_SSEL,
    /// set equalizer taps
    DFE_FL_FB_CMD_SET_EQR_TAPS,

    /// set mixer nco ssel
    DFE_FL_FB_CMD_SET_NCO_SSEL,
    /// set mixer nco bypass
    DFE_FL_FB_CMD_SET_NCO_BYPASS,
    /// set mixer nco frequency
    DFE_FL_FB_CMD_SET_NCO_FREQ,
    /// set mixer nco dither
    DFE_FL_FB_CMD_SET_NCO_DITHER,

    /// set lut bypass
    DFE_FL_FB_CMD_SET_LUT_BYPASS,
    /// set lut memory, dc and slope
    DFE_FL_FB_CMD_SET_LUT_MEM,

    /// set decimation filter rate
    DFE_FL_FB_CMD_SET_DF_RATE,

    /// set gain ssel
    DFE_FL_FB_CMD_SET_GAIN_SSEL,
    /// set gain value
    DFE_FL_FB_CMD_SET_GAIN_VAL,

    
    DFE_FL_FB_CMD_MAX_VALUE
} DfeFl_FbHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /// get FB inits
	DFE_FL_FB_QUERY_GET_INITS = 0,
	/// get FB inits clock enable delay
    DFE_FL_FB_QUERY_GET_INITS_CKENDLY,

    /// get top testbus configure
    DFE_FL_FB_QUERY_GET_TESTBUS,
    /// get the dc testbus configure
    DFE_FL_FB_QUERY_GET_DC_TESTBUS,
    /// get the checksum result
	DFE_FL_FB_QUERY_GET_CHKSUM_RESULT,

	/// get the IO control configure
	DFE_FL_FB_QUERY_GET_IO_CONTROL,

    /// get dc canceller ssel
    DFE_FL_FB_QUERY_GET_DKACC_SSEL,
    /// get dc gsg ssel
    DFE_FL_FB_QUERY_GET_DC_GSG_SSEL,
    /// get dc global control
    DFE_FL_FB_QUERY_GET_DC_GLOBAL,
    /// get dc interval values
    DFE_FL_FB_QUERY_GET_DC_INTERVAL,
    /// get dc canceller update delay
    DFE_FL_FB_QUERY_GET_DC_DELAY,
    /// get dc canceller shift and mode control
    DFE_FL_FB_QUERY_GET_DC_SHIFT_MODE,
    /// get dc init
    DFE_FL_FB_QUERY_GET_DC_INIT,

    /// get real-to-complex ssel
    DFE_FL_FB_QUERY_GET_R2C_SSEL,
    /// get real-to-complex mode control
    DFE_FL_FB_QUERY_GET_R2C_REALIN,
    /// get real-to-complex spectral inversion control
    DFE_FL_FB_QUERY_GET_R2C_SPECINV,

    /// get equalizer ssel
    DFE_FL_FB_QUERY_GET_EQR_SSEL,
    /// get equalizer taps
    DFE_FL_FB_QUERY_GET_EQR_TAPS,

    /// get mixer nco ssel
    DFE_FL_FB_QUERY_GET_NCO_SSEL,
    /// get mixer nco bypass
    DFE_FL_FB_QUERY_GET_NCO_BYPASS,
    /// get mixer nco frequency
    DFE_FL_FB_QUERY_GET_NCO_FREQ,
    /// get mixer nco dither
    DFE_FL_FB_QUERY_GET_NCO_DITHER,

    /// get lut bypass
    DFE_FL_FB_QUERY_GET_LUT_BYPASS,
    /// get lut memory, dc and slope
    DFE_FL_FB_QUERY_GET_LUT_MEM,

    /// get decimation filter rate
    DFE_FL_FB_QUERY_GET_DF_RATE,

    /// get gain ssel
    DFE_FL_FB_QUERY_GET_GAIN_SSEL,
    /// get gain value
    DFE_FL_FB_QUERY_GET_GAIN_VAL,

    DFE_FL_FB_QUERY_MAX_VALUE
} DfeFl_FbHwStatusQuery;

/** @brief sig-gen mode frame */
typedef enum
{
    /// Enable normal mode
	DFE_FL_FB_SIGGEN_MODE_ENABLE_NORMAL = 0 ,
	/// Enable siggen mode
    DFE_FL_FB_SIGGEN_MODE_ENABLE_SIGGEN
} DfeFl_FbSiggenModeEnable;

/** @brief sig-gen mode frame */
typedef enum
{
    /// Frame normal mode
	DFE_FL_FB_SIGGEN_MODE_FRAME_NORMAL = 0 ,
	/// Frame siggen mode
    DFE_FL_FB_SIGGEN_MODE_FRAME_SIGGEN
} DfeFl_FbSiggenModeFrame;

/** @brief sig-gen mode ramp */
typedef enum
{
    /// LFSR
	DFE_FL_FB_SIGGEN_MODE_RAMP_NORMAL_LFSR = 0 ,
	/// RAMP
    DFE_FL_FB_SIGGEN_MODE_RAMP_RAMP
} DfeFl_FbSiggenModeRamp;

/** @brief sig-gen bus */
typedef enum
{
    /// siggen I bus
	DFE_FL_FB_SIGGEN_IBUS = 0 ,
	/// siggen Q bus
    DFE_FL_FB_SIGGEN_QBUS
} DfeFl_FbSiggenBus;

/** @brief fb block */
typedef enum
{
    /// Fb block 0
	DFE_FL_FB_BLOCK0 = 0 ,
	/// Fb block 1
	DFE_FL_FB_BLOCK1,
	/// Fb block 2
    DFE_FL_FB_BLOCK2,
    /// Fb block 3
    DFE_FL_FB_BLOCK3,
    /// Fb block 4
    DFE_FL_FB_BLOCK4
} DfeFl_FbBlk;

/** @brief fb cb mux */
typedef enum
{
	/// Fb CB mux lut out
	DFE_FL_FB_CB_MUX_LUT_OUT = 0,
	/// Fb CB mux gain out
    DFE_FL_FB_CB_MUX_GAIN_OUT
} DfeFl_CbMux;

/** @brief fb io mux config */
typedef enum
{    
    /// Fb IO mux jesdfb0 route0
	DFE_FL_FB_CFG_SEL_JESDFB0_ROUTE0 = 0,
	/// Fb IO mux jesdfb0 route1
    DFE_FL_FB_CFG_SEL_JESDFB0_ROUTE1 = 2,
    /// Fb IO mux jesdfb1 route2
    DFE_FL_FB_CFG_SEL_JESDFB1_ROUTE2 = 4,
    /// Fb IO mux jesdfb1 route3
    DFE_FL_FB_CFG_SEL_JESDFB1_ROUTE3 = 6,
    /// Fb IO mux jesdfb0 route4
    DFE_FL_FB_CFG_SNF_JESDFB0_ROUTE4 = 8,
    /// Fb IO mux jesdfb1 route4
    DFE_FL_FB_CFG_SNF_JESDFB1_ROUTE4 = 12,
    /// Fb arbitor switch
    DFE_FL_FB_CFG_ARB_SWITCH         = 1
} DfeFl_FbIoMux;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_FB_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_SIGGEN_MODE
 */
typedef struct
{
	/// siggen bus
	DfeFl_FbSiggenBus bus;
	/// siggen enable
	uint32_t enable;
	/// siggen frame
    uint32_t frame;
    /// siggen mode
    uint32_t mode;
    /// seed
    uint32_t seed;
    /// frame length in clocks
    uint32_t frameLen;
} DfeFl_FbSiggenMode;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_CFG_SIGGEN_RAMP
 */
typedef struct
{
    /// siggen bus
	DfeFl_FbSiggenBus bus;
	// ramp starting value
    uint32_t rampStart;
    // ramp stop value
    uint32_t rampStop;
    // ramp increment
    uint32_t rampInc;
    // pulse width
    uint32_t pulseWidth;
} DfeFl_FbSiggenRampCfg;


/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_CFG_CHKSUM
 */
typedef struct
{
    /// checksum mode
	uint32_t chksumMode;
    struct
    {
        /// stable length
		uint32_t stableLen;
		/// signal length
        uint32_t signalLen;
        /// channel select
        uint32_t chanSel;
    } latencyMode;
} DfeFl_FbChksumCfg;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_IO_CTRL
 */
typedef struct
{
	/// capture buffer i/q parallel output signal select
	uint32_t cb_output_select;
	/// host configuration select control
	uint32_t host_cfg_select;
	/// configuration select mode control
	uint32_t cfg_select_mode;
} DfeFl_FbIOCtrl;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_DC_GLOBAL
 */
typedef struct
{
	/// control dc integration accumulator on "freeze" signal
	uint32_t dc_frz_rst_int0;
	/// dc canceller freeze control
	uint32_t dc_freeze;
} DfeFl_FbDcGlobal;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_DC_SHIFT_MODE
 */
typedef struct
{
	/// dc shift control
	uint32_t dc_shift;
	/// dc mode control
	uint32_t dc_mode;
} DfeFl_FbDcShiftMode;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_DC_INIT
 */
typedef struct
{
	/// dc init 0
	uint32_t dcinit0;
	/// dc init 1
	uint32_t dcinit1;
} DfeFl_FbDcInit;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_R2C_REALIN
 *      DFE_FL_FB_CMD_SET_R2C_SPECINV
 *      DFE_FL_FB_CMD_SET_NCO_BYPASS
 *      DFE_FL_FB_CMD_SET_NCO_DITHER
 *      DFE_FL_FB_CMD_SET_LUT_BYPASS
 */
typedef struct
{
	/// Fb block id
	DfeFl_FbBlk blk;
	/// data
	uint32_t data;
} DfeFl_FbR2cRealin, DfeFl_FbR2cSpecinv, DfeFl_FbNcoBypass, DfeFl_FbNcoDither, DfeFl_FbLutBypass;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_EQR_TAPS
 */

#define DFE_FL_FB_EQR_TAPS (DFE_FL_FB_EQR_LEN>>1)
typedef struct
{
	/// Fb block id
	DfeFl_FbBlk blk;
	/// number of coeffs
	uint32_t numCoeff;
	/// taps of ii
	uint32_t taps_ii[DFE_FL_FB_EQR_TAPS];
	/// taps of iq
	uint32_t taps_iq[DFE_FL_FB_EQR_TAPS];
	/// taps of qi
	uint32_t taps_qi[DFE_FL_FB_EQR_TAPS];
	/// taps of qq
	uint32_t taps_qq[DFE_FL_FB_EQR_TAPS];
} DfeFl_FbEqrTaps;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_NCO_SSEL
 */
typedef struct
{
	/// ssel to reset the NCO dither generator
	uint32_t dither;
	/// ssel to zero the NCO phase accumulator
	uint32_t phase;
	/// ssel to frequency control words
	uint32_t freq;
} DfeFl_FbNcoSsel;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_NCO_FREQ
 */
typedef struct
{
	/// Fb block id
	DfeFl_FbBlk blk;
	/// frequency word 15:0
	uint32_t freq_word_w0;
	/// frequency word 31:16
	uint32_t freq_word_w1;
	/// frequency word 47:32
	uint32_t freq_word_w2;
} DfeFl_FbNcoFreq;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_GAIN_VAL
 */
typedef struct
{
	/// Fb block id
	DfeFl_FbBlk blk;
	/// real data
	uint32_t real;
	/// image data
	uint32_t imag;
} DfeFl_FbGainVal;

/** @brief argument for runtime control,
 *      DFE_FL_FB_CMD_SET_LUT_MEM
 */
typedef struct
{
	/// number of LUT entries
	uint32_t numEntry;
	/// pointer to dc
	uint32_t *dc;
	/// pointer to slope
	uint32_t *slope;
} DfeFl_FbLut;

/** @brief overlay register pointer to FB instance
 */
typedef CSL_DFE_FB_REGS *DfeFl_FbRegsOvly;

/** @brief a FB Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle       hDfe;

    /// pointer to register base address of a FB instance
	DfeFl_FbRegsOvly	regs;

    /// This is the instance of FB being referred to by this object
    DfeFl_InstNum         perNum;

} DfeFl_FbObj;

/** @brief handle pointer to FB object
 */
typedef DfeFl_FbObj *DfeFl_FbHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_FB_FUNCTION
 * @{
 */

DfeFl_FbHandle dfeFl_FbOpen
(
	    DfeFl_Handle               hDfe,
	    DfeFl_FbObj                *pDfeFbObj,
	    DfeFl_InstNum                 fbNum,
	    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_FbClose(DfeFl_FbHandle hDfeFb);

DfeFl_Status  dfeFl_FbHwControl
(
    DfeFl_FbHandle             hDfeFb,
    DfeFl_FbHwControlCmd       ctrlCmd,
    void                         *arg
);

DfeFl_Status  dfeFl_FbGetHwStatus
(
    DfeFl_FbHandle             hDfeFb,
    DfeFl_FbHwStatusQuery      queryId,
    void                         *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_FB_H_ */
