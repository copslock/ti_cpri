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
 *  @defgroup DFE_FL_DDUC_API DDUC
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_dduc.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_DDUC CSL
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
 * @defgroup DFE_FL_DDUC_DATASTRUCT DFE Dduc Data Structures
 * @ingroup DFE_FL_DDUC_API
 */

/**
 * @defgroup DFE_FL_DDUC_ENUM DFE Dduc Enumverated Data Types
 * @ingroup DFE_FL_DDUC_API
 */

/**
 * @defgroup DFE_FL_DDUC_FUNCTION DFE Dduc Functions
 * @ingroup DFE_FL_DDUC_API
 */
#ifndef _DFE_FL_DDUC_H_
#define _DFE_FL_DDUC_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/csl/cslr_dfe_dduc.h>

/**
 * @addtogroup DFE_FL_DDUC_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
	/// configure CFR inits, including ssel, clk_gate, init_state and clear_data
    DFE_FL_DDUC_CMD_CFG_INITS,
        
    /// set farrow phase ssel
    DFE_FL_DDUC_CMD_SET_FRW_PHASE_SSEL,
    /// set farrow phase
    DFE_FL_DDUC_CMD_CFG_FRW_PHASE,
    /// set farrow checksum
    DFE_FL_DDUC_CMD_SET_FRW_CHKSUM_SSEL,
    /// set farrow signal generator ssel
    DFE_FL_DDUC_CMD_SET_FRW_SIG_SSEL,
    
    /// set hop offset ssel
    DFE_FL_DDUC_CMD_SET_HOP_OFFSET_SSEL,
    /// configure hop offset
    DFE_FL_DDUC_CMD_CFG_HOP_OFFSET,
    /// set hop ssel
    DFE_FL_DDUC_CMD_SET_HOP_SSEL,
    /// set hop frequence
    DFE_FL_DDUC_CMD_CFG_HOP_FRQWORD,
    /// set hop sync delay
    DFE_FL_DDUC_CMD_SET_SYNC_DELAY,

    /// set mix phase ssel
    DFE_FL_DDUC_CMD_SET_MIX_PHASE_SSEL,
    /// set mix NCO ssel
    DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL,
    /// set mix dithering
    DFE_FL_DDUC_CMD_CFG_MIX_DIT,
    /// configure mix mode
    DFE_FL_DDUC_CMD_CFG_MIX_MODE,
    /// set mix phase
    DFE_FL_DDUC_CMD_SET_MIX_PHASE,

    /// set cic flush ssel
    DFE_FL_DDUC_CMD_SET_CIC_FLUSH_SSEL,
    /// configure cic
    DFE_FL_DDUC_CMD_CFG_CIC,

    /// set selector ssel
    DFE_FL_DDUC_CMD_SET_SELECTOR_SSEL,
    /// set mix select
    DFE_FL_DDUC_CMD_SET_MIX_SEL,

    /// set pfir coeff ssel
    DFE_FL_DDUC_CMD_SET_PFIR_COEF_SSEL,
    /// set pfir coeff offset
    DFE_FL_DDUC_CMD_SET_PFIR_COEF_OFFSET,
    /// set pfir fcmux
    DFE_FL_DDUC_CMD_SET_PFIR_FCMUX,
    /// set pfir coeff symmetry
    DFE_FL_DDUC_CMD_SET_PFIR_PCSYM,
    /// set pfir coeff
    DFE_FL_DDUC_CMD_SET_PFIR_COEF,

    /// set dduc testbus mux
    DFE_FL_DDUC_CMD_CFG_TESTBUS_MUX,
    /// set dduc testbus ICG delay
    DFE_FL_DDUC_CMD_CFG_TESTBUS_ICG_DLY,
    /// set dduc testbus bb mux
    DFE_FL_DDUC_CMD_CFG_TESTBUS_BBMUX,

    /// configure dduc tx signal generator mode
    DFE_FL_DDUC_CMD_CFG_TX_SIGGEN_MODE,
    /// configure dduc tx signal generator ramp
    DFE_FL_DDUC_CMD_CFG_TX_SIGGEN_RAMP,
    /// configure dduc rx signal generator mode
    DFE_FL_DDUC_CMD_CFG_RX_SIGGEN_MODE,
    /// configure dduc rx signal generator ramp
    DFE_FL_DDUC_CMD_CFG_RX_SIGGEN_RAMP,
    /// configure dduc tx checksum
    DFE_FL_DDUC_CMD_CFG_TX_CHKSUM,
    /// configure dduc rx checksum
    DFE_FL_DDUC_CMD_CFG_RX_CHKSUM,

    /// enable cic overflow interrupt
    DFE_FL_DDUC_CMD_ENB_CICOV_INTR,
    /// enable hop roll over interrupt
    DFE_FL_DDUC_CMD_ENB_HOPROLLOVER_INTR,
    /// enable hop half way done interrupt
    DFE_FL_DDUC_CMD_ENB_HOPHALFWAY_INTR,
    /// disable cic overflow interrupt
    DFE_FL_DDUC_CMD_DIS_CICOV_INTR,
    /// disable hop roll over interrupt
    DFE_FL_DDUC_CMD_DIS_HOPROLLOVER_INTR,
    /// disable hop half way done interrupt
    DFE_FL_DDUC_CMD_DIS_HOPHALFWAY_INTR,
    /// force set cic overflow interrupt
    DFE_FL_DDUC_CMD_SET_FORCE_CICOV_INTR,
    /// force set hop roll over interrupt
    DFE_FL_DDUC_CMD_SET_FORCE_HOPROLLOVER_INTR,
    /// force set hop half way done interrupt
    DFE_FL_DDUC_CMD_SET_FORCE_HOPHALFWAY_INTR,
    /// clear set cic overflow interrupt
    DFE_FL_DDUC_CMD_CLR_FORCE_CICOV_INTR,
    /// clear set hop roll over interrupt
    DFE_FL_DDUC_CMD_CLR_FORCE_HOPROLLOVER_INTR,
    /// clear set hop half way interrupt
    DFE_FL_DDUC_CMD_CLR_FORCE_HOPHALFWAY_INTR,
    /// clear cic overflow interrupt status
    DFE_FL_DDUC_CMD_CLR_CICOV_INTR_STATUS,
    /// clear hop roll over interrupt status
    DFE_FL_DDUC_CMD_CLR_HOPROLLOVER_INTR_STATUS,
    /// clear hop half way interrupt status
    DFE_FL_DDUC_CMD_CLR_HOPHALFWAY_INTR_STATUS,
    /// set dduc time step
    DFE_FL_DDUC_CMD_SET_TIME_STEP,
    /// set dduc reset init
    DFE_FL_DDUC_CMD_SET_RESET_INT,
    /// set dduc tdd period
    DFE_FL_DDUC_CMD_SET_TDD_PERIOD,
    /// set dduc tdd on and off
    DFE_FL_DDUC_CMD_SET_TDD_ON_OFF,

        
    DFE_FL_DDUC_CMD_MAX_VALUE
} DfeFl_DducHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /// get the dduc direction
	DFE_FL_DDUC_QUERY_DIRECT,

	/// get hop sync delay
	DFE_FL_DDUC_QUERY_SYNC_DELAY,

    /// get farrow phase
    DFE_FL_DDUC_QUERY_GET_FRW_PHASE,

	/// get cic configuration
	DFE_FL_DDUC_QUERY_CIC_CONFIG,

	/// get dduc tx checksum result
	DFE_FL_DDUC_QUERY_TX_CHKSUM_RESULT,
	/// get dduc rx checksum result
	DFE_FL_DDUC_QUERY_RX_CHKSUM_RESULT,

	/// get cic overflow interrupt status
    DFE_FL_DDUC_QUERY_GET_CICOV_INTR_STATUS,
    /// get hop roll over interrupt status
    DFE_FL_DDUC_QUERY_GET_HOPROLLOVER_INTR_STATUS,
    /// get hop half way interrupt status
    DFE_FL_DDUC_QUERY_GET_HOPHALFWAY_INTR_STATUS,
      
    DFE_FL_DDUC_QUERY_MAX_VALUE
} DfeFl_DducHwStatusQuery;


/** @brief DDUC direction
 */ 
typedef enum
{    
    /// dduc direction uplink
	DFE_FL_DDUC_DIR_UL = 0,
	/// dduc direction downlink
    DFE_FL_DDUC_DIR_DL = 1
} DFE_FL_DDUC_DIR;
    
/** @brief mix nco
 */
typedef enum
{
    /// dduc mix nco 0
	DFE_FL_DDUC_MIX_NCO_0 = 0,
	/// dduc mix nco 1
    DFE_FL_DDUC_MIX_NCO_1,
    /// dduc mix nco 2
    DFE_FL_DDUC_MIX_NCO_2,
    /// dduc mix nco 3
    DFE_FL_DDUC_MIX_NCO_3,
    /// dduc mix nco 4
    DFE_FL_DDUC_MIX_NCO_4,
    /// dduc mix nco 5
    DFE_FL_DDUC_MIX_NCO_5,
    /// dduc mix nco 6
    DFE_FL_DDUC_MIX_NCO_6,
    /// dduc mix nco 7
    DFE_FL_DDUC_MIX_NCO_7,
    /// dduc mix nco 8
    DFE_FL_DDUC_MIX_NCO_8,
    /// dduc mix nco 9
    DFE_FL_DDUC_MIX_NCO_9,
    /// dduc mix nco 10
    DFE_FL_DDUC_MIX_NCO_10,
    /// dduc mix nco 11
    DFE_FL_DDUC_MIX_NCO_11,
    /// dduc mix nco 0 to 11
    DFE_FL_DDUC_MIX_NCO_ALL = 0xFFF    
} DfeFl_DducMixNco;

/** @brief mix mode
 */
typedef enum
{
	/// dduc mix real 0
	DFE_FL_DDUC_MIX_rl0 = 0,
	/// dduc mix real 1
    DFE_FL_DDUC_MIX_rl1,
    /// dduc mix real 2
    DFE_FL_DDUC_MIX_rl2,
    /// dduc mix image 0
    DFE_FL_DDUC_MIX_im0,
    /// dduc mix image 1
    DFE_FL_DDUC_MIX_im1,
    /// dduc mix image 2
    DFE_FL_DDUC_MIX_im2

} DfeFl_DducMixMode;

/** @brief pfir idx
 */
typedef enum
{
    /// dduc pfir index 0
	DFE_FL_DDUC_PFIR_IDX_0 = 0,
	/// dduc pfir index 1
    DFE_FL_DDUC_PFIR_IDX_1,
    /// dduc pfir index 2
    DFE_FL_DDUC_PFIR_IDX_2,
    /// dduc pfir index 3
    DFE_FL_DDUC_PFIR_IDX_3,
    /// dduc pfir index 4
    DFE_FL_DDUC_PFIR_IDX_4,
    /// dduc pfir index 5
    DFE_FL_DDUC_PFIR_IDX_5,
    /// dduc pfir index 6
    DFE_FL_DDUC_PFIR_IDX_6,
    /// dduc pfir index 7
    DFE_FL_DDUC_PFIR_IDX_7,
    /// dduc pfir index 8
    DFE_FL_DDUC_PFIR_IDX_8,
    /// dduc pfir index 9
    DFE_FL_DDUC_PFIR_IDX_9,
    /// dduc pfir index 10
    DFE_FL_DDUC_PFIR_IDX_10,
    /// dduc pfir index 11
    DFE_FL_DDUC_PFIR_IDX_11
} DfeFl_DducPfirIdx;

/** @brief selector mix chan or rx
 */
typedef enum
{
	/// dduc selector mix chan
	DFE_FL_DDUC_SELECTOR_MIX_CHAN = 0,
	/// dduc selector mix rx
	DFE_FL_DDUC_SELECTOR_MIX_RX
} DfeFl_DducChanOrRx;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_DDUC_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_CFG_FRW_PHASE
 */
typedef struct
{
    /// farrow phase
	uint32_t phase[12];
} DfeFl_DducFrwPhaseConfig;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_CFG_HOP_OFFSET
 */
typedef struct
{
    /// hop offset
	uint32_t offset[12];
} DfeFl_DducHopOffsetConfig;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_CFG_HOP_FRQWORD
 */
typedef struct
{
    /// hop frequency word
	struct
    {
        /// low part
		uint32_t low;
		/// middle part
        uint32_t mid;
        /// high part
        uint32_t high;
    } frqWord[12];    
} DfeFl_DducHopFrqwordConfig;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL
 *      DFE_FL_DDUC_CMD_SET_MIX_PHASE
 */
typedef struct
{
	/// dduc mix nco index
	DfeFl_DducMixNco iMix;
	/// data
	uint32_t data;
} DfeFl_DducMixNcoSsel, DfeFl_DducMixNcoPhase;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_CFG_MIX_MODE
 */
typedef struct
{
	/// dduc mix mode index
	DfeFl_DducMixMode iMixMode;
	/// data
	uint32_t data;
} DfeFl_DducMixModeCfg;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_CFG_CIC
 */
typedef struct
{
	/// cic rate
	uint32_t rate;
	/// number of real streams per cic minus one
	uint32_t cic_ndata;
	/// auto-flush enable
	uint32_t fl_en;
	/// change cic gain in receive mode
	uint32_t cic_rx_output_scale;
} DfeFl_DducCicCfg;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_SET_MIX_SEL
 */
typedef struct
{
	/// mix index
	uint32_t iMix;
	/// channal id
	uint32_t iChan;
	/// channel or rx
	DfeFl_DducChanOrRx chan_or_rx;
	/// data
	uint32_t data;
} DfeFl_DducMixSel;

/** @brief argument for runtime control,
 *      CSL_defDducSetPfirCoefOffset
 *      DFE_FL_DDUC_CMD_SET_PFIR_FCMUX
 *      DFE_FL_DDUC_CMD_SET_PFIR_PCSYM
 */
typedef struct
{
	/// pfir id
	DfeFl_DducPfirIdx idx;
	/// data
	uint32_t data;
} DfeFl_DducPfirCoefOffset, DfeFl_DducPfirFcMux, DfeFl_DducPfirPcSym;

/** @brief argument for runtime control,
 *    DFE_FL_DDUC_CMD_SET_PFIR_COEF
 */
typedef struct
{
	/// number of pfir coefficients
	uint32_t numCoeff;
	/// pointer to the coefficients
	uint32_t *coeff;
} DfeFl_DducPfirCoeff;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_CFG_TX_SIGGEN_MODE
 *      DFE_FL_DDUC_CMD_CFG_RX_SIGGEN_MODE
 */
typedef struct
{
	/// enable data generation
	uint32_t enable;
    /// enbale frame generation
    uint32_t frame;
    /// ramp (1), or LFSR (0)
    uint32_t mode;
    /// number of clocks per frame minus 1
    uint32_t frameLenM1;
} DfeFl_DducSiggenMode;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_CFG_TX_SIGGEN_RAMP
 *      DFE_FL_DDUC_CMD_CFG_RX_SIGGEN_RAMP
 */
typedef struct
{
    /// ramp start
    uint32_t rampStart;
    /// ramp stop
    uint32_t rampStop;
    /// ramp slope
    uint32_t rampSlope;
    /// pulse width
    uint32_t pulse_width;
} DfeFl_DducSiggenRampConfig;


/** @brief argument for runtime control,
 *      DFE_FL_DDUC_CMD_CFG_TX_CHKSUM
 *      DFE_FL_DDUC_CMD_CFG_RX_CHKSUM
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
} DfeFl_DducChksumConfig;

/** @brief argument for runtime control,
 *      DFE_FL_DDUC_QUERY_GET_CICOV_INTR_STATUS
 */
typedef struct
{
    /// interrupt
    uint32_t intr;
    /// data
    uint32_t data;
} DfeFl_DducCicovIntrStatus;

/** @brief argument for runtime control,
 *     DFE_FL_DDUC_CMD_SET_TDD_ON_OFF
 */
typedef struct
{
	/// tdd on 0
	uint32_t tdd_on_0;
	/// tdd off 0
	uint32_t tdd_off_0;
	/// tdd on 1
	uint32_t tdd_on_1;
	/// tdd off 1
	uint32_t tdd_off_1;
} DfeFl_DducTddOnOff;

/** @brief overlay register pointer to DDUC instance
 */
typedef CSL_DFE_DDUC_REGS *DfeFl_DducRegsOvly;

/** @brief DDUC Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle           hDfe;
    
    /// pointer to register base address of a DDUC instance
    DfeFl_DducRegsOvly     regs;
   
    /// This is the instance of DDUC being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_DducObj;

/** @brief handle pointer to DDUC object
 */
typedef DfeFl_DducObj *DfeFl_DducHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_DDUC_FUNCTION
 * @{
 */

DfeFl_DducHandle dfeFl_DducOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_DducObj              *pDfeDducObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_DducClose(DfeFl_DducHandle hDfeDduc);

DfeFl_Status  dfeFl_DducHwControl
(
    DfeFl_DducHandle           hDfeDduc,
    DfeFl_DducHwControlCmd     ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_DducGetHwStatus
(
    DfeFl_DducHandle           hDfeDduc,
    DfeFl_DducHwStatusQuery    queryId,
    void                        *arg
);

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_DDUC_H_ */
