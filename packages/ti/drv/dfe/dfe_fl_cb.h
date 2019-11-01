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
 *  @defgroup DFE_FL_CB_API CB
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_cb.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_CB CSL
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
 * @defgroup DFE_FL_CB_DATASTRUCT DFE Cb Data Structures
 * @ingroup DFE_FL_CB_API
 */

/**
 * @defgroup DFE_FL_CB_ENUM DFE Cb Enumverated Data Types
 * @ingroup DFE_FL_CB_API
 */

/**
 * @defgroup DFE_FL_CB_FUNCTION DFE Cb Functions
 * @ingroup DFE_FL_CB_API
 */

#ifndef _DFE_FL_CB_H_
#define _DFE_FL_CB_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/csl/cslr_dfe_cb.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/drv/dfe/dfe_fl_cbParams.h>

/**
 * @addtogroup DFE_FL_CB_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
	/// configure CB inits, including ssel, clk_gate, init_state and clear_data
    DFE_FL_CB_CMD_CFG_INITS,

    /// CB_C arm
	DFE_FL_CB_CMD_SET_CBC_ARM,
	/// CB_C force reset
	DFE_FL_CB_CMD_SET_CBC_FORCE_RESET,
	/// CB_F arm
    DFE_FL_CB_CMD_SET_CBF_ARM,
    //DFE_FL_CB_CMD_SET_CBF_FORCE_RESET,
    /// CB_F subsample mode
    DFE_FL_CB_CMD_SET_CBF_SUBSAMPLE,

    /// set Bus control
    DFE_FL_CB_CMD_SET_BUS_CTRL,
    /// set DSP control
    DFE_FL_CB_CMD_SET_DSP_CTRL,
    /// set DPD mode
    DFE_FL_CB_CMD_SET_DPD_MODE,
    /**
     * set the latency (in reference samples)
     * between reference and feedback signals
     */
    DFE_FL_CB_CMD_SET_REF_FB_LATENCY,

    /// CB_F skip chunck setup
    DFE_FL_CB_CMD_SET_SKIP_CHUNK,

    /// CB operation mode setup
    DFE_FL_CB_CMD_SET_CB_BUF_MODE,
    /**
     * CB buffer setup, including node select,
     * bus select, ref_or_fb and not_used setup
     */
    DFE_FL_CB_CMD_SET_CB_MODE_SET,
    /// CB delay from sync
    DFE_FL_CB_CMD_SET_CB_DLY,
    /// CB rate mode (sample/clock)
    DFE_FL_CB_CMD_SET_CB_RATE_MODE,
    /// CB fractional counter
    DFE_FL_CB_CMD_SET_CB_FRAC_CNT,
    /// CB ssel, including frac_cnt_ssel and len_cnt_ssel
    DFE_FL_CB_CMD_SET_CB_SSEL,

    /**
     * CB node configure, including I0bus_sel, Q0bus_sel, I1bus_sel,
     * Q1bus_sel, I0fsdly, Q0fsdly, I1fsdly, Q1fsdly, fsf and fsfm
     */
    DFE_FL_CB_CMD_SET_NODE_CONFIG,

    /**
     * Multi capture control, including enable, number of captures
     * and the chunk size
     */
    DFE_FL_CB_CMD_SET_MULTI_CTRL,
    /// Multi capture buffer timer setup
    DFE_FL_CB_CMD_SET_MULTI_TIMER,

    /**
     * Trigger monitor setting including triger select, multiband,
     * blk0 magsqd select, blk1 magsqd select, blk0_IOC, blk1_IOC
     */
    DFE_FL_CB_CMD_SET_TRIG_SET,
    /**
     * Trigger monitor configure including I0bus_sel, Q0bus_sel,
     * I1bus_sel, Q1bus_sel, I0fsdly, Q0fsdly, I1fsdly, Q1fsdly, fsf and fsfm
     */
    DFE_FL_CB_CMD_SET_TRIG_CONFIG,
    /**
     * Set tirgger monitor threshold, including length, T1 and T2
     */
    DFE_FL_CB_CMD_SET_TRIG_TH,
    /// setup the trigger monitor decoder
    DFE_FL_CB_CMD_SET_TRIG_DECODER,

    /// setup GSG mode
    DFE_FL_CB_CMD_SET_GSG_MODE,
    /// setup GSG delay from sync
    DFE_FL_CB_CMD_SET_GSG_DELAY,
    /// setup GSG timer, from Timer1 to Timer5
    DFE_FL_CB_CMD_SET_GSG_TIMER,
    /// setup GSG ssel
    DFE_FL_CB_CMD_SET_GSG_SSEL,
    /// setup GSG CB_C ref_sel and fb_sel
    DFE_FL_CB_CMD_SET_GSG_C_SEL,
    /// setup GSG CB_F ref_sel and fb_sel
    DFE_FL_CB_CMD_SET_GSG_F_SEL,
    /// setup GSG trigger select
    DFE_FL_CB_CMD_SET_GSG_TRIG_SEL,
    /// setup silent detect, including enable, samples and threshold
    DFE_FL_CB_CMD_SET_SILENT_DETECT,

    /// setup CB_F chunk selection
    DFE_FL_CB_CMD_SET_CBF_CHUNK_SEL,
    /// setup CB_F broken chain detection, including power_threshold and ref_fb_power_ratio
    DFE_FL_CB_CMD_SET_CBF_BROKEN_CHAIN_DETECT,
    /// setup CB_F delta power inlinear
    DFE_FL_CB_CMD_SET_CBF_DELTA_POW,
    /// setup CB_F bad buffer detection
    DFE_FL_CB_CMD_SET_CBF_BADBUFF_DETECT,

    /// setup power monitor sync delay
    DFE_FL_CB_CMD_SET_PM_SYNC_DLY,
    /// setup power monitor integation period
    DFE_FL_CB_CMD_SET_PM_INTG_PD,
    /// configure power monitor, including I0bus_sel, Q0bus_sel, I0fsdly, Q0fsdly, fsf and fsfm
    DFE_FL_CB_CMD_SET_PM_CONFIG,
    /// setup power monitor node select
    DFE_FL_CB_CMD_SET_PM_NODE_SEL,
    /// setup power monitor ssel
    DFE_FL_CB_CMD_SET_PM_SSEL,

    /// setup sourcing control including fsl, size and repeat
    DFE_FL_CB_CMD_SET_SOURCING_CTRL,
    /// setup sourcing node control
    DFE_FL_CB_CMD_SET_SRC_NODE_CTRL,

    /// setup TDD time step
    DFE_FL_CB_CMD_SET_TIME_STEP,
    /// setup TDD reset interval
    DFE_FL_CB_CMD_SET_RESET_INT,
    /// setup TDD period
    DFE_FL_CB_CMD_SET_TDD_PERIOD,
    /// setup TDD on_off, including on_0, off_0, on_1 and off_1
    DFE_FL_CB_CMD_SET_TDD_ON_OFF,

    /// setup CB_C start ssel
    DFE_FL_CB_CMD_SET_CBC_START_SSEL,
    /// setup CB_F start ssel
    DFE_FL_CB_CMD_SET_CBF_START_SSEL,
    /// setup source ssel
    DFE_FL_CB_CMD_SET_SOURCE_SSEL,
    //DFE_FL_CB_CMD_SET_PM_SSEL,

    /// init fractional phase control, including enable and frac_phase
    DFE_FL_CB_CMD_SET_INIT_FRAC_PHASE_CTRL,
    DFE_FL_CB_CMD_SET_INIT_SSEL,
    DFE_FL_CB_CMD_SET_INIT_CLK_GATE,
    DFE_FL_CB_CMD_SET_INIT_STATE,
    DFE_FL_CB_CMD_SET_INIT_CLR_DATA,
    
    /// set the MSB of CB data
    DFE_FL_CB_CMD_SET_CB_MSB,
    /// set the LSB of CB data
    DFE_FL_CB_CMD_SET_CB_LSB,

    DFE_FL_CB_CMD_MAX_VALUE
} DfeFl_CbHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /// Get init fractional phase control, including enable and frac_phase
    DFE_FL_CB_QUERY_GET_INIT_FRAC_PHASE_CTRL,
    /// Get CB init status, including ssel, clk_gate, init_state and clear_data
	DFE_FL_CB_QUERY_GET_INITS,

	/// Get bus control
	DFE_FL_CB_QUERY_GET_BUS_CTRL,
	/// Get DSP control
    DFE_FL_CB_QUERY_GET_DSP_CTRL,
    /// Get DPD mode
    DFE_FL_CB_QUERY_GET_DPD_MODE,
    /// Get the latency (in reference samples)
    DFE_FL_CB_QUERY_GET_REF_FB_LATENCY,

    /// Get CB_F skip chunck
    DFE_FL_CB_QUERY_GET_SKIP_CHUNK,

    /// Get CB operation mode
    DFE_FL_CB_QUERY_GET_CB_BUF_MODE,
    /**
     * CB buffer setup, including node select,
     * bus select, ref_or_fb and not_used setup
     */
    DFE_FL_CB_QUERY_GET_CB_MODE_SET,
    /// Get CB delay from sync
    DFE_FL_CB_QUERY_GET_CB_DLY,
    /// Get rate mode
    DFE_FL_CB_QUERY_GET_CB_RATE_MODE,
    /// Get franctional counter
    DFE_FL_CB_QUERY_GET_CB_FRAC_CNT,
    /// Get CB ssel, including frac_cnt_ssel and len_cnt_ssel
    DFE_FL_CB_QUERY_GET_CB_SSEL,

    /**
	 * Get CB node configure, including I0bus_sel, Q0bus_sel, I1bus_sel,
	 * Q1bus_sel, I0fsdly, Q0fsdly, I1fsdly, Q1fsdly, fsf and fsfm
	 */
    DFE_FL_CB_QUERY_GET_NODE_CONFIG,

    /**
     * Get multi capture control, including enable, number of captures
     * and the chunk size
     */
    DFE_FL_CB_QUERY_GET_MULTI_CTRL,
    /// Get multi capture buffer timer setup
    DFE_FL_CB_QUERY_GET_MULTI_TIMER,

    /**
     * Get trigger monitor setting including triger select, multiband,
     * blk0 magsqd select, blk1 magsqd select, blk0_IOC, blk1_IOC
     */
    DFE_FL_CB_QUERY_GET_TRIG_SET,
    /**
     * Get trigger monitor configure including I0bus_sel, Q0bus_sel,
     * I1bus_sel, Q1bus_sel, I0fsdly, Q0fsdly, I1fsdly, Q1fsdly, fsf and fsfm
     */
    DFE_FL_CB_QUERY_GET_TRIG_CONFIG,
    /**
     * Get tirgger monitor threshold, including length, T1 and T2
     */
    DFE_FL_CB_QUERY_GET_TRIG_TH,
    /// Get the trigger monitor decoder
    DFE_FL_CB_QUERY_GET_TRIG_DECODER,
    /// Get output power of trigger block
    DFE_FL_CB_QUERY_GET_TRIG_OUTPWR,

    /// Get GSG mode
    DFE_FL_CB_QUERY_GET_GSG_MODE,
    /// Get GSG delay from sync
    DFE_FL_CB_QUERY_GET_GSG_DELAY,
    /// Get GSG timer, from Timer1 to Timer5
    DFE_FL_CB_QUERY_GET_GSG_TIMER,
    /// Get GSG ssel
    DFE_FL_CB_QUERY_GET_GSG_SSEL,
    /// Get GSG CB_C ref_sel and fb_sel
    DFE_FL_CB_QUERY_GET_GSG_C_SEL,
    /// Get GSG CB_F ref_sel and fb_sel
    DFE_FL_CB_QUERY_GET_GSG_F_SEL,
    /// Get GSG trigger select
    DFE_FL_CB_QUERY_GET_GSG_TRIG_SEL,
    /// Get silent detect, including enable, samples and threshold
    DFE_FL_CB_QUERY_GET_SILENT_DETECT,

    /// Get CB_F chunk selection
    DFE_FL_CB_QUERY_GET_CBF_CHUNK_SEL,
    /// Get CB_F broken chain detection, including power_threshold and ref_fb_power_ratio
    DFE_FL_CB_QUERY_GET_CBF_BROKEN_CHAIN_DETECT,
    /// Get CB_F delta power inlinear
    DFE_FL_CB_QUERY_GET_CBF_DELTA_POW,
    /// Get CB_F bad buffer detection
    DFE_FL_CB_QUERY_GET_CBF_BADBUFF_DETECT,
    /// Get CB_F maximum reference power
    DFE_FL_CB_QUERY_GET_CBF_MAXREF_POW,

    /// Get power monitor sync delay
    DFE_FL_CB_QUERY_GET_PM_SYNC_DLY,
    /// Get power monitor integation period
    DFE_FL_CB_QUERY_GET_PM_INTG_PD,
    /// Get power monitor, including I0bus_sel, Q0bus_sel, I0fsdly, Q0fsdly, fsf and fsfm
    DFE_FL_CB_QUERY_GET_PM_CONFIG,
    /// Get power monitor node select
    DFE_FL_CB_QUERY_GET_PM_NODE_SEL,
    /// Get power monitor ssel
    DFE_FL_CB_QUERY_GET_PM_SSEL,

    /// Get sourcing control including fsl, size and repeat
    DFE_FL_CB_QUERY_GET_SOURCING_CTRL,
    /// Get sourcing node control
    DFE_FL_CB_QUERY_GET_SRC_NODE_CTRL,

    /// Get TDD time step
    DFE_FL_CB_QUERY_GET_TIME_STEP,
    /// Get TDD reset interval
    DFE_FL_CB_QUERY_GET_RESET_INT,
    /// Get TDD period
    DFE_FL_CB_QUERY_GET_TDD_PERIOD,
    /// Get TDD on_off, including on_0, off_0, on_1 and off_1
    DFE_FL_CB_QUERY_GET_TDD_ON_OFF,

    /// Get CB_C start ssel
    DFE_FL_CB_QUERY_GET_CBC_START_SSEL,
    /// Get CB_F start ssel
    DFE_FL_CB_QUERY_GET_CBF_START_SSEL,
    /// Get sourcing ssel
    DFE_FL_CB_QUERY_GET_SOURCE_SSEL,
    //DFE_FL_CB_QUERY_GET_PM_SSEL,


    /// Get CB_C arm
	DFE_FL_CB_QUERY_GET_CBC_ARM,
	/// Get CB_C force reset
	DFE_FL_CB_QUERY_GET_CBC_FORCE_RESET,
	/// Get CB_F arm
    DFE_FL_CB_QUERY_GET_CBF_ARM,
    //DFE_FL_CB_QUERY_GET_CBF_FORCE_RESET,
    /// Get CB_F subsample mode
    DFE_FL_CB_QUERY_GET_CBF_SUBSAMPLE,

    /// Get CB done fractional couner
    DFE_FL_CB_QUERY_GET_CB_DONE_FRAC_CNT,
    /// Get CB done address
    DFE_FL_CB_QUERY_GET_CB_DONE_ADDR,
    /// Get CB done length counter
    DFE_FL_CB_QUERY_GET_CB_DONE_LEN_CNT,
    /// Get CB buf full flag
    DFE_FL_CB_QUERY_GET_CB_BUF_FULL,
    /// Get CB status, including done_frac_cnt, done_addr, done_len_cnt and buf_full_flag
    DFE_FL_CB_QUERY_GET_CB_STATUS,

//    DFE_FL_CB_QUERY_GET_REF_CHUNK_DONE_ADDR,
//    DFE_FL_CB_QUERY_GET_FB_CHUNK_DONE_ADDR,
    /// Get CB_F chunk done address
    DFE_FL_CB_QUERY_GET_CHUNK_DONE_ADDR,

    /// Get the MSB of CB data
    DFE_FL_CB_QUERY_GET_CB_MSB,
    /// Get the LSB of CB data
    DFE_FL_CB_QUERY_GET_CB_LSB,

    DFE_FL_CB_QUERY_MAX_VALUE
} DfeFl_CbHwStatusQuery;


/** @brief cb buf */
typedef enum
{
	/// CB buf A
    DFE_FL_CBA = 0 ,
    /// CB buf B
    DFE_FL_CBB,
    /// CB buf C
    DFE_FL_CBC,
    /// CB buf D
    DFE_FL_CBD
} DfeFl_CbBuf;

/** @brief cb node */
typedef enum
{
	/// CB node 0
    DFE_FL_CB_NODE0 = 0 ,
    /// CB node 1
    DFE_FL_CB_NODE1,
    /// CB node 2
    DFE_FL_CB_NODE2,
    /// CB node 3
    DFE_FL_CB_NODE3,
    /// CB node 4
    DFE_FL_CB_NODE4,
    /// CB node 5
    DFE_FL_CB_NODE5,
    /// CB node 6
    DFE_FL_CB_NODE6,
    /// CB node 7
    DFE_FL_CB_NODE7,
    /// CB node 8
    DFE_FL_CB_NODE8
} DfeFl_CbNode;

/** @brief cb multi timer */
typedef enum
{
    /// Multi CB Timer 1
	DFE_FL_CB_MULTI_TIMER1 = 0 ,
	/// Multi CB Timer 2
    DFE_FL_CB_MULTI_TIMER2,
    /// Multi CB Timer 3
    DFE_FL_CB_MULTI_TIMER3,
    /// Multi CB Timer 4
    DFE_FL_CB_MULTI_TIMER4,
    /// Multi CB Timer 5
    DFE_FL_CB_MULTI_TIMER5,
    /// Multi CB Timer 6
    DFE_FL_CB_MULTI_TIMER6,
    /// Multi CB Timer 7
    DFE_FL_CB_MULTI_TIMER7,
    /// Multi CB Timer 8
    DFE_FL_CB_MULTI_TIMER8
} DfeFl_CbMultipleTimer;

/** @brief cb trigger */
typedef enum
{
    /// CB trigger monitor A
	DFE_FL_CB_TRIG_A = 0 ,
	/// CB trigger monitor B
    DFE_FL_CB_TRIG_B
} DfeFl_CbTrig;

/** @brief cb trigger block */
typedef enum
{
    /// CB trigger monitor A block 0
	DFE_FL_CB_TRIG_A_BLK0 = 0 ,
	/// CB trigger monitor A block 1
	DFE_FL_CB_TRIG_A_BLK1,
	/// CB trigger monitor B block 0
    DFE_FL_CB_TRIG_B_BLK0,
    /// CB trigger monitor B block 1
    DFE_FL_CB_TRIG_B_BLK1
} DfeFl_CbTrigBlk;

/** @brief cb gsg */
typedef enum
{
    /// CB GSG 0
	DFE_FL_CB_GSG0 = 0 ,
	/// CB GSG 1
    DFE_FL_CB_GSG1,
    /// CB GSG 2
    DFE_FL_CB_GSG2,
    /// CB GSG 3
    DFE_FL_CB_GSG3,
    /// CB GSG 4
    DFE_FL_CB_GSG4,
    /// CB GSG 5
    DFE_FL_CB_GSG5
} DfeFl_CbGsg;

/** @brief cb ant*/
typedef enum
{
    /// CB ANT 0
	DFE_FL_CB_ANT0 = 0 ,
	/// CB ANT 1
    DFE_FL_CB_ANT1,
    /// CB ANT 2
    DFE_FL_CB_ANT2,
    /// CB ANT 3
    DFE_FL_CB_ANT3
} DfeFl_CbAnt;

/** @brief cb chunk */
typedef enum
{
    /// CB Chunk 1
	DFE_FL_CB_CHUNK1 = 0 ,
	/// CB Chunk 2
    DFE_FL_CB_CHUNK2,
    /// CB Chunk 3
    DFE_FL_CB_CHUNK3,
    /// CB Chunk 4
    DFE_FL_CB_CHUNK4,
    /// CB Chunk 5
    DFE_FL_CB_CHUNK5,
    /// CB Chunk 6
    DFE_FL_CB_CHUNK6,
    /// CB Chunk 7
    DFE_FL_CB_CHUNK7,
    /// CB Chunk 8
    DFE_FL_CB_CHUNK8
} DfeFl_CbChunk;

/** @brief cb operation mode */
typedef enum
{
	/// mpu mode
	DFE_FL_CB_MPU = 0,
	/// cb_c static mode
	DFE_FL_CB_C_STATIC,
	/// sourcing mode
	DFE_FL_CB_SRC,
	/// cb_c trigger mode
	DFE_FL_CB_C_TRIGGER,
	/// cb_c sharing mode
	DFE_FL_CB_C_SHARING,
	/// cb_c monitor mode
	DFE_FL_CB_C_MONITOR,
	/// cb_f
	DFE_FL_CB_F
} DfeFl_CbMode;

/** @brief cb reference or feedback */
typedef enum
{
  /// reference
  DFE_FL_CB_REF = 0,
  /// feedback
  DFE_FL_CB_FB
} DfeFl_CbRefOrFb;


/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CB_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_CB_CMD_SET_CBC_ARM
 *      DFE_FL_CB_CMD_SET_CBF_ARM
 */
typedef struct
{
    /// CB sync
	uint32_t sync;
	/// CB done
    uint32_t cbDone;
} DfeFl_CbArm;

/** @brief argument for runtime control,
 *      DFE_FL_CB_CMD_SET_CBC_FORCE_RESET
 *      DFE_FL_CB_CMD_SET_CBF_FORCE_RESET
 */
typedef struct
{
	/// CB force arm reset
    uint32_t force_arm_reset;
    /// CB force done reset
    uint32_t force_done_reset;
} DfeFl_CbForceReset;

/** @brief argument for runtime control,
 *       DFE_FL_CB_CMD_SET_BUS_CTRL
 */
typedef struct
{
	/// no gating select
	uint32_t nogating;
	/// test bus select
	uint32_t tbus_sel;
	/// IQ swap select
	uint32_t IQ_swap;
} DfeFl_CbBusCtrl;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_SKIP_CHUNK
 */
typedef struct
{
	/// read reference skip chunk
	uint32_t readref_skipchunk;
	/// read feedback skip chunk
	uint32_t readfb_skipchunk;
} DfeFl_CbSkipChunk;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_CB_MODE_SET
 */
typedef struct
{
	/// cb buf
	DfeFl_CbBuf cbBuf;
	/// node select
	uint32_t sel;
	/// bus select
	uint32_t busSel;
	/// reference or feedback
	uint32_t ref_or_fb;
	/// not_used
	uint32_t not_used;
} DfeFl_CbModeSet;

/** @brief argument for runtime control,
 * 	   DFE_FL_CB_CMD_SET_CB_BUF_MODE
 * 	   DFE_FL_CB_CMD_SET_CB_DLY
 *     DFE_FL_CB_CMD_SET_CB_RATE_MODE
 *     DFE_FL_CB_CMD_SET_CB_FRAC_CNT
 */
typedef struct
{
	/// cb buf
	DfeFl_CbBuf cbBuf;
	/// data value
	uint32_t data;
} DfeFl_CbDly, DfeFl_CbRateMode, DfeFl_CbBufMode, DfeFl_CbFracCnt, DfeFl_CbDoneInfo;

/** @brief argument for runtime control,
 *      DFE_FL_CB_QUERY_GET_CB_STATUS
 */
typedef struct 
{
	/// cb buf
	DfeFl_CbBuf cbBuf;
	/// done address
	uint32_t       doneAddr;
	/// done length counter
	uint32_t       doneLenCnt;
	/// done fractional counter
	uint32_t       doneFracCnt;
	/// buffer full flag
	uint32_t       bufferFullFlag;
} DfeFl_CbStatus;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_CB_SSEL
 */
typedef struct
{
	/// cb buf
	DfeFl_CbBuf cbBuf;
	/// franctional counter sync select
	uint32_t frac_cnt_ssel;
	/// length counter sync select
	uint32_t len_cnt_ssel;
} DfeFl_CbSsel;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_NODE_CONFIG
 */
typedef struct
{
	/// cb node
	DfeFl_CbNode cbNode;
	/// I0 bus select
	uint32_t I0bus_sel;
	/// Q0 bus select
	uint32_t Q0bus_sel;
	/// I1 bus select
	uint32_t I1bus_sel;
	/// Q1 bus select
	uint32_t Q1bus_sel;
	/// I0 bus data delay relative to frame start
	uint32_t I0fsdly;
	/// Q0 bus data delay relative to frame start
	uint32_t Q0fsdly;
	/// I1 bus data delay relative to frame start
	uint32_t I1fsdly;
	/// Q1 bus data delay relative to frame start
	uint32_t Q1fsdly;
	/// frame strobe format
	uint32_t fsf;
	/// frame strobe format mask
	uint32_t fsfm;
} DfeFl_CbNodeCfg;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_MULTI_CTRL
 */
typedef struct
{
	/// multi cb enable
	uint32_t enable;
	/// number of captures
	uint32_t numCaptures;
	/// the chunk size
	uint32_t chunkSize;
} DfeFl_CbMultiCtrl;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_MULTI_TIMER
 */
typedef struct
{
	/// multi capture timer
	DfeFl_CbMultipleTimer cbMultiTimer;
	/// data
	uint32_t data;
} DfeFl_CbMultiTimer;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_TRIG_SET
 */
typedef struct
{
	/// trigger monitor
	DfeFl_CbTrig cbTrig;
	/// node selection
	uint32_t sel;
	/// multiband mode enable
	uint32_t multiband;
	/// block0 magnitude or magnitude square select
	uint32_t blk0_MagSqd_sel;
	/// block1 magnitude or magnitude square select
	uint32_t blk1_MagSqd_sel;
	/// block0 integrator counter select
	uint32_t blk0_IOC;
	/// block1 integrator counter select
	uint32_t blk1_IOC;
} DfeFl_CbTrigSet;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_TRIG_CONFIG
 */
typedef struct
{
	/// trigger monitor
	DfeFl_CbTrig cbTrig;
	/// I0 bus select
	uint32_t I0bus_sel;
	/// Q0 bus select
	uint32_t Q0bus_sel;
	/// I1 bus select
	uint32_t I1bus_sel;
	/// Q1 bus select
	uint32_t Q1bus_sel;
	/// I0 bus data delay relative to frame start
	uint32_t I0fsdly;
	/// Q0 bus data delay relative to frame start
	uint32_t Q0fsdly;
	/// I1 bus data delay relative to frame start
	uint32_t I1fsdly;
	/// Q1 bus data delay relative to frame start
	uint32_t Q1fsdly;
	/// frame strobe format
	uint32_t fsf;
	/// frame strobe format mask
	uint32_t fsfm;
} DfeFl_CbTrigCfg;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_TRIG_TH
 */
typedef struct
{
	/// trigger monitor block
	DfeFl_CbTrigBlk cbTrigBlk;
	/// window size
	uint32_t length;
	/// threshold 1
	uint32_t T1;
	/// threshold 2
	uint32_t T2;
} DfeFl_CbTrigTh;

/** @brief argument for runtime control,
 *     DFE_FL_CB_QUERY_GET_TRIG_OUTPWR
 */
typedef struct
{
	/// trigger monitor block
	DfeFl_CbTrigBlk cbTrigBlk;
	/// data
	uint32_t data;
} DfeFl_CbTrigOutpwr;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_GSG_MODE
 *     DFE_FL_CB_CMD_SET_GSG_SSEL
 */
typedef struct
{
	/// cb GSG
	DfeFl_CbGsg cbGsg;
	/// data
	uint32_t data;
} DfeFl_CbGsgMode, DfeFl_CbGsgDelay, DfeFl_CbGsgSsel;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_GSG_TIMER
 */
typedef struct
{
	/// cb GSG
	DfeFl_CbGsg cbGsg;
	/// GSG timer1
	uint32_t timer1;
	/// GSG timer2
	uint32_t timer2;
	/// GSG timer3
	uint32_t timer3;
	/// GSG timer4
	uint32_t timer4;
	/// GSG timer5
	uint32_t timer5;
} DfeFl_CbGsgTimer;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_GSG_C_SEL
 */
typedef struct
{
	/// GSG reference select
	uint32_t ref_sel;
	/// GSG feedback select
	uint32_t fb_sel;
} DfeFl_CbGsgCsel;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_GSG_F_SEL
 */
typedef struct
{
	/// cb antenna
	DfeFl_CbAnt cbAnt;
	/// reference selection
	uint32_t ref_sel;
	/// feedback selection
	uint32_t fb_sel;
} DfeFl_CbGsgFsel;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_GSG_TRIG_SEL
 */
typedef struct
{
	/// trigger monitor
	DfeFl_CbTrig cbTrig;
	/// data
	uint32_t data;
} DfeFl_CbGsgTrigSel;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_SILENT_DETECT
 */
typedef struct
{
	/// silent detection enable
	uint32_t enable;
	/// number of samples
	uint32_t samples;
	/// threshold
	uint32_t thresh;
} DfeFl_CbSilentDet;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_CBF_CHUNK_SEL
 *     DFE_FL_CB_CMD_SET_CBF_DELTA_POW
 *     DFE_FL_CB_CMD_SET_CBF_BADBUFF_DETECT
 *     DFE_FL_CB_CMD_SET_PM_SYNC_DLY
 *     DFE_FL_CB_CMD_SET_PM_INTG_PD
 *     DFE_FL_CB_QUERY_GET_CBF_MAXREF_POW
 *     DFE_FL_CB_CMD_SET_REF_FB_LATENCY
 *     DFE_FL_CB_CMD_SET_PM_SSEL
 */
typedef struct
{
	/// cb antenna
	DfeFl_CbAnt cbAnt;
	/// data
	uint32_t data;
} DfeFl_CbfChunkSel, DfeFl_CbfDeltaPow, DfeFl_CbfBadbuffDet, \
  DfeFl_CbPmSyncDly, DfeFl_CbPmIntgPd, DfeFl_CbfMaxrefPow, \
  DfeFl_CbFbLatency, DfeFl_CbPmSsel;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_CBF_BROKEN_CHAIN_DETECT
 */
typedef struct
{
	/// power threshold
	uint32_t powerTH;
	/// the power ration between reference and feedback signals
	uint32_t ref_fb_powerRatio;
} DfeFl_CbfBrokenChainDet;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_PM_CONFIG
 */
typedef struct
{
	/// cb antenna
	DfeFl_CbAnt cbAnt;
	/// I0 bus select
	uint32_t I0bus_sel;
	/// Q0 bus select
	uint32_t Q0bus_sel;
	/// I1 bus select
	/// I0 bus data delay relative to frame start
	uint32_t I0fsdly;
	/// Q0 bus data delay relative to frame start
	uint32_t Q0fsdly;
	/// frame strobe format
	uint32_t fsf;
	/// frame strobe format mask
	uint32_t fsfm;
} DfeFl_CbPmCfg;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_SOURCING_CTRL
 */
typedef struct
{
	/// sourcing mode frame length
	uint32_t fsl;
	/// number of data to be sourced minus 1
	uint32_t size;
	/// repeat source data
	uint32_t repeat;
} DfeFl_CbSrcCtrl;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_SRC_NODE_CTRL
 */
typedef struct
{
	/// source to dduc from bb
	uint32_t bb_to_dduc;
	/// source to cfr from sum
	uint32_t sum_to_cfr;
	/// source to dpd from cdfr
	uint32_t cdfr_to_dpd;
	/// source to tx from dpd
	uint32_t dpd_to_tx;
	/// source to jesd from tx
	uint32_t tx_to_jesd;
	/// source to rx from jesd
	uint32_t jesd_to_rx;
	/// source to fb from jesd
	uint32_t jesd_to_fb;
	/// source to dduc from rx
	uint32_t rx_to_dduc;
	/// source to dduc from fb
	uint32_t fb_to_dduc;
	/// source to bb from dduc
	uint32_t dduc_to_bb;
} DfeFl_CbSrcNodeCtrl;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_TDD_ON_OFF
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
} DfeFl_CbTddOnOff;

/** @brief argument for runtime control,
 *     DFE_FL_CB_CMD_SET_INIT_FRAC_PHASE_CTRL
 */
typedef struct
{
	/// enable franc phase control
	uint32_t enable;
	/// frac phase value
	uint32_t frac_phase;
} DfeFl_CbInitFracPhCtrl;

/** @brief argument for runtime control,
 *     DFE_FL_CB_QUERY_GET_REF_CHUNK_DONE_ADDR
 *     DFE_FL_CB_QUERY_GET_FB_CHUNK_DONE_ADDR
 */
typedef struct
{
	/// cb buf
	DfeFl_CbBuf cbBuf;
	/// cb chunk
	DfeFl_CbChunk cbChunk;
	/// chunk done address
	uint32_t doneAddr;
} DfeFl_CbChunkDoneAddr;

/** @brief dpd complex int
 */
typedef struct
{
	/// real value
	Uint16 real;
	/// image value
	Uint16 imag;

} DfeFl_CbComplexInt;

/** @brief argument for runtime control,
 *     dfeFl_CbQueryCbMSB
 *     dfeFl_CbQueryCbLSB
 */
typedef struct
{
	/// cb buf
	DfeFl_CbBuf cbBuf;
	/// start position
	uint32_t startPos;
	/// size of cb data
	uint32_t size;
	/// pointer to the complex cb data
	DfeFl_CbComplexInt *data;
} DfeFl_CbData;

/** @brief overlay register pointer to CB instance
 */
typedef CSL_DFE_CB_REGS *DfeFl_CbRegsOvly;

/** @brief a CB Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle       hDfe;

    /// pointer to register base address of a CB instance
	DfeFl_CbRegsOvly	regs;

    /// This is the instance of CB being referred to by this object
    DfeFl_InstNum         perNum;

} DfeFl_CbObj;

/** @brief handle pointer to CB object
 */
typedef DfeFl_CbObj *DfeFl_CbHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CB_FUNCTION
 * @{
 */

DfeFl_CbHandle dfeFl_CbOpen
(
	    DfeFl_Handle               hDfe,
	    DfeFl_CbObj                *pDfeCbObj,
	    DfeFl_InstNum                 cbNum,
	    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_CbClose(DfeFl_CbHandle hDfeCb);

DfeFl_Status  dfeFl_CbHwControl
(
    DfeFl_CbHandle             hDfeCb,
    DfeFl_CbHwControlCmd       ctrlCmd,
    void                         *arg
);

DfeFl_Status  dfeFl_CbGetHwStatus
(
    DfeFl_CbHandle             hDfeCb,
    DfeFl_CbHwStatusQuery      queryId,
    void                         *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_CB_H_ */
