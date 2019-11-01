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
 *  @defgroup DFE_FL_DPD_API DPD
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_dpd.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_DPD CSL
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
 * @defgroup DFE_FL_DPD_DATASTRUCT DFE Dpd Data Structures
 * @ingroup DFE_FL_DPD_API
 */

/**
 * @defgroup DFE_FL_DPD_ENUM DFE Dpd Enumverated Data Types
 * @ingroup DFE_FL_DPD_API
 */

/**
 * @defgroup DFE_FL_DPD_FUNCTION DFE Dpd Functions
 * @ingroup DFE_FL_DPD_API
 */

#ifndef _DFE_FL_DPD_H_
#define _DFE_FL_DPD_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/csl/cslr_dfe_dpd.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/drv/dfe/dfe_fl_dpdParams.h>

/**
 * @addtogroup DFE_FL_DPD_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
	/// configure DPD inits, including ssel, clk_gate, init_state and clear_data
	DFE_FL_DPD_CMD_CFG_INITS = 0,
    /// subchip_mode
    DFE_FL_DPD_CMD_UPD_SUBCHIP_MODE,
    /// subsample
    DFE_FL_DPD_CMD_UPD_SUBSAMPLE,
    /// mux square root
    DFE_FL_DPD_CMD_UPD_MUX_SQRT,
    /// mux complex signal
    DFE_FL_DPD_CMD_UPD_MUX_COMPLX,
    /// mux real magnitude
    DFE_FL_DPD_CMD_UPD_MUX_REAL_MAG,
    /// mux dpd output
    DFE_FL_DPD_CMD_UPD_MUX_OUTPUT,
    /// dpdadapt update mode
    DFE_FL_DPD_CMD_UPD_DPDADAPT_MODE,
    /// dpdadapt mux fsync
    DFE_FL_DPD_CMD_UPD_DPDADAPT_FSYNC,
    /// dpdadapt mux csync
    DFE_FL_DPD_CMD_UPD_DPDADAPT_CSYNC,
    /// config f_sync selection for each block
    DFE_FL_DPD_CMD_UPD_BLK_F_SSEL,
    /// config f_sync selection for all blocks
    DFE_FL_DPD_CMD_UPD_SUBCHIP_F_SSEL,
    /// config c_sync selection for each block
    DFE_FL_DPD_CMD_UPD_BLK_C_SSEL,
    /// config c_sync selection for all blocks
    DFE_FL_DPD_CMD_UPD_SUBCHIP_C_SSEL,
    /// diable dpd
    DFE_FL_DPD_CMD_SET_DPD_DISABLE,
    /// config sync selection for syncB
    DFE_FL_DPD_CMD_UPD_SYNCB_SSEL,

    /// update dpd input scale
    DFE_FL_DPD_CMD_UPD_DPDINPUT_SCALE,

    /// test signal generation config
    DFE_FL_DPD_CMD_CFG_TESTGEN,
    /// test signal generation sync selection
    DFE_FL_DPD_CMD_SET_TESTGEN_SSEL,
    /// chksum
    DFE_FL_DPD_CMD_CFG_CHKSUM,
    /// chksum sync selection
    DFE_FL_DPD_CMD_SET_CHKSUM_SSEL,

    /// clk gate delay
    DFE_FL_DPD_CMD_CLK_GATE_DELAY,
    /// test bus control
    DFE_FL_DPD_CMD_TEST_BUS_CTRL,

    /// update mux_blk in each dpd block
    DFE_FL_DPD_CMD_UPD_MUX_BLK,
    /// update mux_blk_row in each dpd row
    DFE_FL_DPD_CMD_UPD_MUX_BLK_ROW,

    /// update lut init for each row
    DFE_FL_DPD_CMD_UPD_ROW_LUT_INIT,
    /// update lut init for all rows of one block
    DFE_FL_DPD_CMD_UPD_BLK_LUT_INIT,
    /// update lut toggle for each row
    DFE_FL_DPD_CMD_UPD_ROW_LUT_TOGGLE,
    /// update lut toggle for all rows of one block
    DFE_FL_DPD_CMD_UPD_BLK_LUT_TOGGLE,
    /// config sync selection for each cell
    DFE_FL_DPD_CMD_UPD_CELL_SYNC,
    /// config sync selection for all cells of one row
    DFE_FL_DPD_CMD_UPD_ROW_SYNC,
    /// config sync selection for all cells of one block
    DFE_FL_DPD_CMD_UPD_BLK_SYNC,

    /// update lut value for each entry
    DFE_FL_DPD_CMD_UPD_ENTRY_LUT,
    /// update lut value for each cell
    DFE_FL_DPD_CMD_UPD_CELL_LUT,
    /// update lut value for each row
    DFE_FL_DPD_CMD_UPD_ROW_LUT,
    /// update lut value for each block
    DFE_FL_DPD_CMD_UPD_BLK_LUT,
    
    DFE_FL_DPD_CMD_MAX_VALUE
} DfeFl_DpdHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /// subchip_mode
    DFE_FL_DPD_QUERY_SUBCHIP_MODE = 0,
    /// subsample
    DFE_FL_DPD_QUERY_SUBSAMPLE,
    /// mux square root
    DFE_FL_DPD_QUERY_MUX_SQRT,
    /// mux complex signal
    DFE_FL_DPD_QUERY_MUX_COMPLX,
    /// mux real magnitude
    DFE_FL_DPD_QUERY_MUX_REAL_MAG,
    /// mux dpd output
    DFE_FL_DPD_QUERY_MUX_OUTPUT,
    /// dpdadapt update mode
    DFE_FL_DPD_QUERY_DPDADAPT_MODE,
    /// dpdadapt mux fsync
    DFE_FL_DPD_QUERY_DPDADAPT_FSYNC,
    /// dpdadapt mux csync
    DFE_FL_DPD_QUERY_DPDADAPT_CSYNC,
    /// f_sync selection for each block
    DFE_FL_DPD_QUERY_BLK_F_SSEL,
    /// f_sync selection for each subchip
    DFE_FL_DPD_QUERY_SUBCHIP_F_SSEL,
    /// c_sync selection for each block
    DFE_FL_DPD_QUERY_BLK_C_SSEL,
    /// c_sync selection for each subchip
    DFE_FL_DPD_QUERY_SUBCHIP_C_SSEL,
    /// diable dpd
    DFE_FL_DPD_QUERY_DPD_DISABLE,
    /// sync selection for syncB
    DFE_FL_DPD_QUERY_SYNCB_SSEL,
    /// inits ssel
    DFE_FL_DPD_QUERY_INITS_SSEL,
    /// init clk gate
    DFE_FL_DPD_QUERY_INIT_CLK_GATE,
    /// init state
    DFE_FL_DPD_QUERY_INIT_STATE,
    /// clear data
    DFE_FL_DPD_QUERY_CLEAR_DATA,

    /// dpd input scale
    DFE_FL_DPD_QUERY_DPDINPUT_SCALE,

    /// test signal generation config
    DFE_FL_DPD_QUERY_TESTGEN_CFG,
    /// test signal generation sync selection
    DFE_FL_DPD_QUERY_TESTGEN_SSEL,
    /// chksum
    DFE_FL_DPD_QUERY_CHKSUM_RESULT,

    /// clk gate delay
    DFE_FL_DPD_QUERY_CLK_GATE_DELAY,
    /// test bus control
    DFE_FL_DPD_QUERY_TEST_BUS_CTRL,

    /// mux_blk in each dpd block
    DFE_FL_DPD_QUERY_MUX_BLK,
    /// mux_blk_row in each dpd row
    DFE_FL_DPD_QUERY_MUX_BLK_ROW,

    /// lut init for each row
    DFE_FL_DPD_QUERY_ROW_LUT_INIT,
    /// lut init for each block
    DFE_FL_DPD_QUERY_BLK_LUT_INIT,
    /// lut toggle for each row
    DFE_FL_DPD_QUERY_ROW_LUT_TOGGLE,
    ///lut toggle for each block
    DFE_FL_DPD_QUERY_BLK_LUT_TOGGLE,
    /// sync selection for each cell
    DFE_FL_DPD_QUERY_CELL_SYNC,
    /// sync selection for each row
    DFE_FL_DPD_QUERY_ROW_SYNC,
    /// sync selection for each blk,
    DFE_FL_DPD_QUERY_BLK_SYNC,
    /// current lut mpu for each cell
    DFE_FL_DPD_QUERY_CELL_CURRENT_LUT_MPU,
    /// current lut mpu for each row
    DFE_FL_DPD_QUERY_ROW_CURRENT_LUT_MPU,
    /// current lut mpu for each blk
    DFE_FL_DPD_QUERY_BLK_CURRENT_LUT_MPU,

    /// lut value for each entry
    DFE_FL_DPD_QUERY_ENTRY_LUT,
    /// lut value for each cell
    DFE_FL_DPD_QUERY_CELL_LUT,
    /// lut value for each row
    DFE_FL_DPD_QUERY_ROW_LUT,
    /// lut value for each block
    DFE_FL_DPD_QUERY_BLK_LUT,
        
    DFE_FL_DPD_QUERY_MAX_VALUE
} DfeFl_DpdHwStatusQuery;

/** @brief DPD Test Signal Generation Device
 */
typedef enum
{
    /// TESTGEN0 for DPD
    DFE_FL_DPD_TESTGEN_0 = 0,
    /// TESTGEN1 for DPD
    DFE_FL_DPD_TESTGEN_1,
    /// TESTGEN2 for DPD
    DFE_FL_DPD_TESTGEN_2,
    /// TESTGEN3 for DPD
    DFE_FL_DPD_TESTGEN_3,
    /// TESTGEN4 for DPD
    DFE_FL_DPD_TESTGEN_4,
    /// TESTGEN5 for DPD
    DFE_FL_DPD_TESTGEN_5,
    /// TESTGEN6 for DPD
    DFE_FL_DPD_TESTGEN_6,
    /// TESTGEN7 for DPD
    DFE_FL_DPD_TESTGEN_7
} DfeFl_DpdTestGenDev;

/** @brief DPD Test Signal Generation ramp mode
 */
typedef enum
{
    /// LFSR
    DFE_FL_DPD_TESTGEN_RAMP_MODE_LFSR = 0,
    /// RAMP
    DFE_FL_DPD_TESTGEN_RAMP_MODE_RAMP = 1
} DfeFl_DpdTestGenRampMode;

/** @brief DPD checksum return mode */
typedef enum
{
    /// return checksum
	DFE_FL_DPD_CHKSUM_MODE_RETURN_CHKSUM = 0,
	/// return latency
    DFE_FL_DPD_CHKSUM_MODE_RETURN_LATENCY
} DfeFl_DpdChksumMode;

/** @brief DPD Block
 */
typedef enum
{
	/// dpd block 0
	DFE_FL_DPD_B0 = 0,
	/// dpd block 1
	DFE_FL_DPD_B1,
	/// dpd block 2
	DFE_FL_DPD_B2,
	/// dpd block 3
	DFE_FL_DPD_B3
}DfeFl_DPDBlk;

/** @brief DPD Row
 */
typedef enum
{
	/// dpd row 0
	DFE_FL_DPD_R0 = 0,
	/// dpd row 1
	DFE_FL_DPD_R1,
	/// dpd row 2
	DFE_FL_DPD_R2,
	/// dpd row 3
	DFE_FL_DPD_R3,
	/// dpd row 4
	DFE_FL_DPD_R4,
	/// dpd row 5
	DFE_FL_DPD_R5
}DfeFl_DPDRow;

/** @brief DPD Cell
 */
typedef enum
{
	/// dpd cell 0
	DFE_FL_DPD_C0 = 0,
	/// dpd cell 1
	DFE_FL_DPD_C1,
	/// dpd cell 2
	DFE_FL_DPD_C2
}DfeFl_DPDCell;

/** @brief mux sqrt type
 */
typedef enum
{
	/// dpd mux sqrt r00
	DFE_FL_DPD_MUX_SQRT_R00 = 0,
	/// dpd mux sqrt r01
	DFE_FL_DPD_MUX_SQRT_R01,
	/// dpd mux sqrt r1
	DFE_FL_DPD_MUX_SQRT_R1,
	/// dpd mux sqrt r2
	DFE_FL_DPD_MUX_SQRT_R2,
	/// dpd mux sqrt r31
	DFE_FL_DPD_MUX_SQRT_R31,
	/// dpd mux sqrt r32
	DFE_FL_DPD_MUX_SQRT_R32,
	/// dpd mux sqrt r33
	DFE_FL_DPD_MUX_SQRT_R33,
	/// dpd mux sqrt magx2
	DFE_FL_DPD_MUX_SQRT_MAGX2,
	/// dpd mux sqrt magx3
	DFE_FL_DPD_MUX_SQRT_MAGX3
} DfeFl_DpdMuxSqrtType;

/** @brief DPD EVEN Or ODD
 */
typedef enum
{
	/// dpd even
	DFE_FL_DPD_EVEN = 0,
	/// dpd odd
	DFE_FL_DPD_ODD
} DfeFl_DPDEvenOdd;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_DPD_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_DPD_CMD_CFG_TESTGEN
 *      DFE_FL_DPD_QUERY_TESTGEN_CFG
 */
typedef struct
{
    /// test gen device
    DfeFl_DpdTestGenDev tgDev;
    /// enable data generation
    uint32_t genData;
    /// enbale frame generation
    uint32_t genFrame;
    /// ramp (1), or LFSR (0)
    DfeFl_DpdTestGenRampMode rampMode;
    /// seed
    uint32_t seed;
    /// number of clocks per frame minus 1
    uint32_t frameLenM1;
    /// ramp starting value
    uint32_t rampStart;
    /// ramp stop value
    uint32_t rampStop;
    /// ramp slop value
    uint32_t slope;
    /// 0 = generate data forever, n = generate data for n clock cycles
    uint32_t genTimer;
    /// number of data bits inverted (read-only)
    uint32_t numDataBits;
} DfeFl_DpdTestGenConfig;

/** @brief argument for runtime control,
 *      DFE_FL_DPD_CMD_SET_TESTGEN_SSEL
 *      DFE_FL_DPD_QUERY_TESTGEN_SSEL
 */
typedef struct
{
    /// test gen device
    DfeFl_DpdTestGenDev tgDev;
    /// sync select
    uint32_t ssel;
} DfeFl_DpdTestGenSsel;

/** @brief argument for runtime control,
 *      DFE_FL_DPD_CMD_CFG_CHKSUM
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
} DfeFl_DpdChksumConfig;

/** @brief argument for runtime control,
 *      DFE_FL_DPD_CMD_UPD_MUX_SQRT
 *      DFE_FL_DPD_QUERY_MUX_SQRT
 */
typedef struct
{
	/// dpd mux sqrt type
	DfeFl_DpdMuxSqrtType Mux;
	/// data
	uint32_t data;
} DfeFl_DpdMuxSqrt;

/** @brief argument for runtime control,
 *      DFE_FL_DPD_CMD_UPD_MUX_COMPLX
 *      DFE_FL_DPD_QUERY_MUX_COMPLX
 *      DFE_FL_DPD_CMD_UPD_MUX_REAL_MAG
 *      DFE_FL_DPD_QUERY_MUX_REAL_MAG
 */
typedef struct
{
	/// dpd block id
	DfeFl_DPDBlk idxBlk;
	/// dpd even or odd
	DfeFl_DPDEvenOdd evenOrOdd;
	/// data
	uint32_t data;
} DfeFl_DpdMuxSignal;

/** @brief argument for runtime control,
 *      DFE_FL_DPD_CMD_UPD_MUX_OUTPUT
 *      DFE_FL_DPD_QUERY_MUX_OUTPUT
 *      DFE_FL_DPD_CMD_UPD_DPDADAPT_MODE
 *      DFE_FL_DPD_QUERY_DPDADAPT_MODE
 *      DFE_FL_DPD_CMD_UPD_DPDADAPT_FSYNC
 *      DFE_FL_DPD_QUERY_DPDADAPT_FSYNC
 *      DFE_FL_DPD_CMD_UPD_DPDADAPT_CSYNC
 *      DFE_FL_DPD_QUERY_DPDADAPT_CSYNC
 */
typedef struct
{
	/// dpd block id
	DfeFl_DPDBlk idxBlk;
	/// data
	uint32_t data;
} DfeFl_DpdBlkCtrl;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_BLK_F_SSEL
 * 		DFE_FL_DPD_CMD_UPD_BLK_C_SSEL
 * 		DFE_FL_DPD_QUERY_BLK_F_SSEL
 * 		DFE_FL_DPD_QUERY_BLK_C_SSEL
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// ssel value
	uint32_t Ssel;
} DfeFl_DpdBlkSsel;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_SUBCHIP_F_SSEL
 * 		DFE_FL_DPD_CMD_UPD_SUBCHIP_C_SSEL
 * 		DFE_FL_DPD_QUERY_SUBCHIP_F_SSEL
 * 		DFE_FL_DPD_QUERY_SUBCHIP_C_SSEL
 */
typedef struct
{
	/// number of blocks
	uint32_t numBlks;
	/// ssel update table
	DfeFl_DpdBlkSsel	BlkSsel[DFE_FL_DPD_NBLK];
} DfeFl_DpdSubchipSsel;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_MUX_BLK
 * 		DFE_FL_DPD_QUERY_MUX_BLK
 */
typedef struct
{
	/// dpd block id
	DfeFl_DPDBlk idxBlk;
	/// mux 2x
	uint32_t mux_2x;
	/// mux dg 2x
	uint32_t mux_dg_2x;
	/// mux dga even
	uint32_t mux_dga_e;
	/// mux dga odd
	uint32_t mux_dga_o;
	/// mux dg even
	uint32_t mux_dg_e;
	/// mux dg odd
	uint32_t mux_dg_o;
	/// mux dgaxo even
	uint32_t mux_dgaxo_e;
	/// mux dgaxo odd
	uint32_t mux_dgaxo_o;
	/// mux dgxo even
	uint32_t mux_dgxo_e;
	/// mux dgxo odd
	uint32_t mux_dgxo_o;
} DfeFl_DpdMuxBlk;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_MUX_BLK_ROW
 * 		DFE_FL_DPD_QUERY_MUX_BLK_ROW
 */
typedef struct
{
	/// dpd block id
	DfeFl_DPDBlk idxBlk;
	/// dpd row id
	DfeFl_DPDRow idxRow;
	/// mux dgaxi
	uint32_t mux_dgaxi;
	/// mux dgxi
	uint32_t mux_dgxi;
	/// mux real
	uint32_t mux_real;
	/// mux complex
	uint32_t mux_complex;
	/// mux daxi
	uint32_t mux_daxi;
	/// mux dxi
	uint32_t mux_dxi;
} DfeFl_DpdMuxBlkRow;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_ROW_LUT_INIT
 * 		DFE_FL_DPD_CMD_UPD_ROW_LUT_TOGGLE
 * 		DFE_FL_DPD_QUERY_ROW_LUT_INIT
 * 		DFE_FL_DPD_QUERY_ROW_LUT_TOGGLE
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// dpd row id
	uint32_t idxRow;
	/// value
	uint32_t data;
} DfeFl_DpdRowLutInit, DfeFl_DpdRowLutToggle;

typedef struct
{
	/// dpd row id
	uint32_t idxRow;
	/// value
	uint32_t data;
} RowLutInit;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_BLK_LUT_INIT
 * 		DFE_FL_DPD_QUERY_BLK_LUT_INIT
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// number of rows
	uint32_t numRows;
	/// values
	RowLutInit LutInit[DFE_FL_DPD_NROW];
} DfeFl_DpdBlkLutInit;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_BLK_LUT_TOGGLE
 * 		DFE_FL_DPD_QUERY_BLK_LUT_TOGGLE
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// number of rows
	uint32_t numRows;
	/// values
	RowLutInit LutToggle[DFE_FL_DPD_NROW];
} DfeFl_DpdBlkLutToggle;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_CELL_SYNC
 * 		DFE_FL_DPD_QUERY_CELL_SYNC
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// dpd row id
	uint32_t idxRow;
	/// dpd cell id
	uint32_t idxCell;
	/// values
	uint32_t Ssel;
} DfeFl_DpdCellSync;

typedef struct
{
	/// dpd cell id
	uint32_t idxCell;
	/// value
	uint32_t data;
} RowData;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_ROW_SYNC
 * 		DFE_FL_DPD_QUERY_ROW_SYNC
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// dpd row id#
	uint32_t idxRow;
	/// number of cells
	uint32_t numCells;
	/// values
	RowData CellSync[DFE_FL_DPD_NCEL];
} DfeFl_DpdRowSync;

typedef struct
{
	/// dpd row id
	uint32_t idxRow;
	/// dpd cell id
	uint32_t idxCell;
	/// value
	uint32_t data;
} BlkData;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_BLK_SYNC
 * 		DFE_FL_DPD_QUERY_BLK_SYNC
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// number of rows
	uint32_t numRows;
	/// values
	BlkData CellSync[DFE_FL_DPD_NROW][DFE_FL_DPD_NCEL];
} DfeFl_DpdBlkSync;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_QUERY_CELL_CURRENT_LUT_MPU
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// dpd row id
	uint32_t idxRow;
	/// dpd cell id
	uint32_t idxCell;
	/// values
	uint32_t CurLutMpu;
} DfeFl_DpdCellCurLutMpu;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_QUERY_ROW_CURRENT_LUT_MPU
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// dpd row id
	uint32_t idxRow;
	/// number of cells
	uint32_t numCells;
	/// values
	RowData CurLutMpu[DFE_FL_DPD_NCEL];
} DfeFl_DpdRowCurLutMpu;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_QUERY_BLK_CURRENT_LUT_MPU
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// number of rows
	uint32_t numRows;
	/// values
	BlkData CurLutMpu[DFE_FL_DPD_NROW][DFE_FL_DPD_NCEL];
} DfeFl_DpdBlkCurLutMpu;

/** @brief dpd complex int
 */
typedef struct
{
	/// real data
	uint16_t real;
	/// image data
	uint16_t imag;

} DfeFl_DpdComplexInt;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_ENTRY_LUT
 * 		DFE_FL_DPD_QUERY_ENTRY_LUT
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// dpd row id
	uint32_t idxRow;
	/// dpd cell id
	uint32_t idxCell;
	/// dpd lut entry id
	uint32_t idxEntry;
	/// lutGain
	DfeFl_DpdComplexInt lutGain;
	/// lutSlope
	DfeFl_DpdComplexInt lutSlope;
} DfeFl_DpdEntryLUT;

typedef struct
{
	/// dpd lut entry id
	uint32_t idxEntry;
	/// lutGain
	DfeFl_DpdComplexInt lutGain;
	/// lutSlope
	DfeFl_DpdComplexInt lutSlope;
} EntryLUTData;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_CELL_LUT
 * 		DFE_FL_DPD_QUERY_CELL_LUT
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// dpd row id
	uint32_t idxRow;
	/// dpd cell id
	uint32_t idxCell;
	/// number of entries
	uint32_t numEntries;
	/// values
	EntryLUTData LUTval[DFE_FL_DPD_MAX_LUT_SIZE];
} DfeFl_DpdCellLUT;

typedef struct
{
	/// dpd cell id
	uint32_t idxCell;
	/// dpd entry id
	uint32_t idxEntry;
	/// lutGain
	DfeFl_DpdComplexInt lutGain;
	/// lutSlope
	DfeFl_DpdComplexInt lutSlope;
} CellLUTData;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_ROW_LUT
 * 		DFE_FL_DPD_QUERY_ROW_LUT
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// dpd row id
	uint32_t idxRow;
	/// number of cells
	uint32_t numCells;
	/// values
	CellLUTData LUTval[DFE_FL_DPD_NCEL][DFE_FL_DPD_MAX_LUT_SIZE];
} DfeFl_DpdRowLUT;

typedef struct
{
	/// dpd row id
	uint32_t idxRow;
	/// dpd cell id
	uint32_t idxCell;
	/// dpd entry id
	uint32_t idxEntry;
	/// lutGain
	DfeFl_DpdComplexInt lutGain;
	/// lutSlope
	DfeFl_DpdComplexInt lutSlope;
} RowLUTData;

/** @brief argument for runtime control
 * 		DFE_FL_DPD_CMD_UPD_BLK_LUT
 * 		DFE_FL_DPD_QUERY_BLK_LUT
 */
typedef struct
{
	/// dpd block id
	uint32_t idxBlk;
	/// number of rows
	uint32_t numRows;
	/// values
	RowLUTData LUTval[DFE_FL_DPD_NROW][DFE_FL_DPD_NCEL][DFE_FL_DPD_MAX_LUT_SIZE];
} DfeFl_DpdBlkLUT;


/** @brief overlay register pointer to DPD instance
 */
typedef CSL_DFE_DPD_REGS *DfeFl_DpdRegsOvly;

/** @brief a DPD Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle       hDfe;

    /// pointer to register base address of a DPD instance
	DfeFl_DpdRegsOvly	regs;

    /// This is the instance of DPD being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_DpdObj;

/** @brief handle pointer to DPD object
 */
typedef DfeFl_DpdObj *DfeFl_DpdHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_DPD_FUNCTION
 * @{
 */

//DfeFl_Status dfeFl_DpdInit();

DfeFl_DpdHandle dfeFl_DpdOpen
(
	    DfeFl_Handle               hDfe,
	    DfeFl_DpdObj               *pDfeDpdObj,
	    DfeFl_InstNum					perNum,
	    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_DpdClose(DfeFl_DpdHandle hDfeDpd);

DfeFl_Status  dfeFl_DpdHwControl
(
    DfeFl_DpdHandle             hDfeDpd,
    DfeFl_DpdHwControlCmd       ctrlCmd,
    void                         *arg
);

DfeFl_Status  dfeFl_DpdGetHwStatus
(
    DfeFl_DpdHandle             hDfeDpd,
    DfeFl_DpdHwStatusQuery      queryId,
    void                         *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_DPD_H_ */
