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
 *  @defgroup DFE_FL_JESD_API JESD
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_jesd.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_JESD CSL
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
 * @defgroup DFE_FL_JESD_SYMBOL DFE Jesd Symbols
 * @ingroup DFE_FL_JESD_API
 */

/**
 * @defgroup DFE_FL_JESD_DATASTRUCT DFE Jesd Data Structures
 * @ingroup DFE_FL_JESD_API
 */

/**
 * @defgroup DFE_FL_JESD_ENUM DFE Jesd Enumverated Data Types
 * @ingroup DFE_FL_JESD_API
 */

/**
 * @defgroup DFE_FL_JESD_FUNCTION DFE Jesd Functions
 * @ingroup DFE_FL_JESD_API
 */

#ifndef _DFE_FL_JESD_H_
#define _DFE_FL_JESD_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/csl/cslr_dfe_jesd.h>

/**
 * @addtogroup DFE_FL_JESD_SYMBOL
 * @{
 */
/// total number of lanes
#define DFE_FL_JESD_NUM_LANE           4
/// total number of links
#define DFE_FL_JESD_NUM_LINK           2
/// number of nibbles per lane
#define DFE_FL_JESD_LANE_NUM_NIBB      4
/// number of time positions or slots
#define DFE_FL_JESD_NIBB_NUM_POS       4

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_JESD_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
    ///////////////////////////////////////////////////////////////////////////////////////
    //
    //      TX CMDs
    //
    ///////////////////////////////////////////////////////////////////////////////////////
    /// JESDTX, init clock gate, state, and clear data
    DFE_FL_JESDTX_CMD_CFG_INITS,
	/// JESDTX, init ssel
	DFE_FL_JESDTX_CMD_CFG_INITSSEL,
	/// JESDTX, init state
	DFE_FL_JESDTX_CMD_CFG_INITSTATE,
    /// JESDTX, clear lane data
    DFE_FL_JESDTX_CMD_CFG_CLRLANE,
    /// JESDTX, config Tx inputs
    DFE_FL_JESDTX_CMD_CFG_TX_INPUTS,    
    //x JESDTX, config test bus
    DFE_FL_JESDTX_CMD_CFG_TEST_BUS,
    /// JESDTX, config test sequence
    DFE_FL_JESDTX_CMD_CFG_TEST_SEQ,    
    /// JESDTX, enable SYNC~ loopback
    DFE_FL_JESDTX_CMD_CFG_SYNCN_LOOPBACK,
    /// JESDTX, invert SYNC~ polarity
    DFE_FL_JESDTX_CMD_CFG_SYNCN_INVERT,
    /// JESDTX, config BB control
    DFE_FL_JESDTX_CMD_CFG_BB_CTRL,
    /// JESDTX, clear BB error
    DFE_FL_JESDTX_CMD_CLR_BB_ERR,
    /// JESDTX, set fifo read delay
    DFE_FL_JESDTX_CMD_SET_FIFO_READ_DLY,    
    /// JESDTX, zero data when fifo error
    DFE_FL_JESDTX_CMD_SET_FIFO_ERROR_ZERO_DATA,
    /// JESDTX, set sysref delay
    DFE_FL_JESDTX_CMD_SET_SYSREF_DLY,
    /// JESDTX, force sysref request
    DFE_FL_JESDTX_CMD_FORCE_SYSREF_REQUEST,
    /// JESDTX, select SigGen sync source
    DFE_FL_JESDTX_CMD_SET_TESTGEN_SSEL,
    /// JESDTX, select checksum sync source
    DFE_FL_JESDTX_CMD_SET_CHKSUM_SSEL,
    /// JESDTX, select init state link sync source
    DFE_FL_JESDTX_CMD_SET_INITSTATELINK_SSEL,
    /// JESDTX, select sysref counter sync source
    DFE_FL_JESDTX_CMD_SET_SYSREF_CNTR_SSEL,
    /// JESDTX, select sysref mode sync source
    DFE_FL_JESDTX_CMD_SET_SYSREF_MODE_SSEL,  
    /// JESDTX, set sysref mode  
    DFE_FL_JESDTX_CMD_SET_SYSREF_MODE,
    /// JESDTX, config SigGen
    DFE_FL_JESDTX_CMD_CFG_TESTGEN,
    /// JESDTX, config checksum
    DFE_FL_JESDTX_CMD_CFG_CHKSUM,    
    /// JESDTX, config clock gate
    DFE_FL_JESDTX_CMD_CFG_CLKGATE,
    /// JESDTX, config lane
    DFE_FL_JESDTX_CMD_CFG_LANE,
    /// JESDTX, config link
    DFE_FL_JESDTX_CMD_CFG_LINK,
    /// JESDTX, clear link error count
    DFE_FL_JESDTX_CMD_CLR_LINK_ERR_CNT,    
    /// JESDTX, enable lane interrupts
    DFE_FL_JESDTX_CMD_ENB_LANE_INTRGRP,
    /// JESDTX, disabel lane interrupts
    DFE_FL_JESDTX_CMD_DIS_LANE_INTRGRP,
    /// JESDTX, clear lane interrupts status
    DFE_FL_JESDTX_CMD_CLR_LANE_INTRGRP_STATUS,
    /// JESDTX, force generating lane interrupts
    DFE_FL_JESDTX_CMD_SET_FORCE_LANE_INTRGRP,
    /// JESDTX, clear force generating lane interrupts
    DFE_FL_JESDTX_CMD_CLR_FORCE_LANE_INTRGRP,
    /// JESDTX, enable sysref interrupts
    DFE_FL_JESDTX_CMD_ENB_SYSREF_INTRGRP,
    /// JESDTX, disable sysref interrupts
    DFE_FL_JESDTX_CMD_DIS_SYSREF_INTRGRP,
    /// JESDTX, clear sysref interrupts status
    DFE_FL_JESDTX_CMD_CLR_SYSREF_INTRGRP_STATUS,
    /// JESDTX, force generating sysref interrupts
    DFE_FL_JESDTX_CMD_SET_FORCE_SYSREF_INTRGRP,
    /// JESDTX, clear force generating sysref interrupts
    DFE_FL_JESDTX_CMD_CLR_FORCE_SYSREF_INTRGRP,
    /// JESDTX, config lane map
    DFE_FL_JESDTX_CMD_CFG_MAP_LANE,
    /// JESDTX, config nibble map
    DFE_FL_JESDTX_CMD_CFG_MAP_NIBB,
    /// JESDTX, nibble test enable
    DFE_FL_JESDTX_CMD_CFG_NIBB_TEST_ENABLE,
    /// JESDTX, nibble test data pattern
    DFE_FL_JESDTX_CMD_CFG_NIBB_TEST_DATA,


    ///////////////////////////////////////////////////////////////////////////////////////
    //
    //      RX CMDs
    //
    ///////////////////////////////////////////////////////////////////////////////////////
    /// JESDRX, init clock gate, state, and clear data
    DFE_FL_JESDRX_CMD_CFG_INITS,
	/// JESDRX, init ssel
	DFE_FL_JESDRX_CMD_CFG_INITSSEL,
	/// JESDRX, init state
	DFE_FL_JESDRX_CMD_CFG_INITSTATE,
    /// JESDRX, clear lane data
    DFE_FL_JESDRX_CMD_CFG_CLRLANE,
    /// JESDRX, select test bus
    DFE_FL_JESDRX_CMD_SET_TESTBUS_SEL,
    /// JESDRX, config test sequence
    DFE_FL_JESDRX_CMD_CFG_TESTSEQ,
    /// JESDRX, config link and lane loopback
    DFE_FL_JESDRX_CMD_CFG_LOOPBACK,
    /// JESDRX, config BB Rx control
    DFE_FL_JESDRX_CMD_CFG_BBRXCTRL,
    /// JESDRX, config FIFO
    DFE_FL_JESDRX_CMD_CFG_FIFO,
    /// JESDRX, config SYNC~ out
    DFE_FL_JESDRX_CMD_CFG_SYNCN_OUT,
    /// JESDRX, config sysref delay
    DFE_FL_JESDRX_CMD_SET_SYSREF_DLY,
    /// JESDRX, force sysref request
    DFE_FL_JESDRX_CMD_FORCE_SYSREF_REQUEST,
    /// JESDRX, select init state link sync source
    DFE_FL_JESDRX_CMD_SET_INITSTATELINK_SSEL,
    /// JESDRX, select checksum sync source 
    DFE_FL_JESDRX_CMD_SET_CHKSUM_SSEL,
    /// JESDRX, select sysref counter sync source
    DFE_FL_JESDRX_CMD_SET_SYSREF_CNTR_SSEL,
    /// JESDRX, select SYNC~ out bus sync source
    DFE_FL_JESDRX_CMD_SET_SYNCN_OUT_BUS_SSEL,
    /// JESDRX, select sysref mode sync source
    DFE_FL_JESDRX_CMD_SET_SYSREF_MODE_SSEL,  
    /// JESDRX, select sysref mode  
    DFE_FL_JESDRX_CMD_SET_SYSREF_MODE,
    /// JESDRX, config checksum
    DFE_FL_JESDRX_CMD_CFG_CHKSUM,        
    /// JESDRX, config link clock gate 
    DFE_FL_JESDRX_CMD_CFG_LINK_CLKGATE,
    /// JESDRX, config path clock gate
    DFE_FL_JESDRX_CMD_CFG_PATH_CLKGATE,
    /// JESDRX, config lane
    DFE_FL_JESDRX_CMD_CFG_LANE,
    /// JESDRX, config link
    DFE_FL_JESDRX_CMD_CFG_LINK,
    /// JESDRX, clear link error count
    DFE_FL_JESDRX_CMD_CLR_LINK_ERR_CNT,    
    /// JESDRX, enable lane interrupts
    DFE_FL_JESDRX_CMD_ENB_LANE_INTRGRP,
    /// JESDRX, disable lane interrupts
    DFE_FL_JESDRX_CMD_DIS_LANE_INTRGRP,
    /// JESDRX, cleare lane interrupts status
    DFE_FL_JESDRX_CMD_CLR_LANE_INTRGRP_STATUS,
    /// JESDRX, force generating lane interrupts
    DFE_FL_JESDRX_CMD_SET_FORCE_LANE_INTRGRP,
    /// JESDRX, clear force generating lane interrupts
    DFE_FL_JESDRX_CMD_CLR_FORCE_LANE_INTRGRP,
    /// JESDRX, enable sysref interrupts
    DFE_FL_JESDRX_CMD_ENB_SYSREF_INTRGRP,
    /// JESDRX, disable sysref interrupts
    DFE_FL_JESDRX_CMD_DIS_SYSREF_INTRGRP,
    /// JESDRX, clear sysref interrupts status
    DFE_FL_JESDRX_CMD_CLR_SYSREF_INTRGRP_STATUS,
    /// JESDRX, force generating sysref interrupts
    DFE_FL_JESDRX_CMD_SET_FORCE_SYSREF_INTRGRP,
    /// JESDRX, clear force generating sysref interrupts
    DFE_FL_JESDRX_CMD_CLR_FORCE_SYSREF_INTRGRP,
    /// JESDRX, config nibble map
    DFE_FL_JESDRX_CMD_CFG_MAP_NIBB,
    
    DFE_FL_JESD_CMD_MAX_VALUE
} DfeFl_JesdHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    ///////////////////////////////////////////////////////////////////////////////////////
    //
    //      TX QUERYs
    //
    ///////////////////////////////////////////////////////////////////////////////////////
	/// JESDTX, get init ssel
	DFE_FL_JESDTX_QUERY_GET_INITSSEL,	
    /// JESDTX, get BB error
    DFE_FL_JESDTX_QUERY_GET_BB_ERR,    
    /// JESDTX, get sync~ loopback enable
    DFE_FL_JESDTX_QUERY_GET_SYNCN_LOOPBACK,
    /// JESDTX, get sysref align count bits
    DFE_FL_JESDTX_QUERY_GET_SYSREF_ALGNCNT,
    /// JESDTX, get lane sync status
    DFE_FL_JESDTX_QUERY_GET_LANE_SYNC_STATE,
    /// JESDTX, get first sync request flag
    DFE_FL_JESDTX_QUERY_GET_FIRST_SYNC_REQUEST,
    /// JESDTX, get checksum result
    DFE_FL_JESDTX_QUERY_CHKSUM_RESULT,    
    /// JESDTX, get lane interrupts status
    DFE_FL_JESDTX_QUERY_GET_LANE_INTRGRP_STATUS,
    /// JESDTX, get sysref interrupts status
    DFE_FL_JESDTX_QUERY_GET_SYSREF_INTRGRP_STATUS,
    /// JESDTX, get link error count
    DFE_FL_JESDTX_QUERY_GET_LINK_ERR_CNT,     
    /// JESDTX, get sysref mode
    DFE_FL_JESDTX_QUERY_GET_SYSREF_MODE,
    /// JESDTX, get lane config
    DFE_FL_JESDTX_QUERY_GET_LANE_CFG,
    
 
    ///////////////////////////////////////////////////////////////////////////////////////
    //
    //      RX QUERYs
    //
    ///////////////////////////////////////////////////////////////////////////////////////
	/// JESDRX, get init ssel
	DFE_FL_JESDRX_QUERY_GET_INITSSEL,	
    /// JESDRX, get sysref align count bits
    DFE_FL_JESDRX_QUERY_GET_SYSREF_ALGNCNT,
    /// JESDRX, get lane sync state (code state and frame state)
    DFE_FL_JESDRX_QUERY_GET_LANE_SYNC_STATE,
    /// JESDRX, get checksum result
    DFE_FL_JESDRX_QUERY_CHKSUM_RESULT,
    /// JESDRX, get lane interrupts status
    DFE_FL_JESDRX_QUERY_GET_LANE_INTRGRP_STATUS,
    /// JESDRX, get sysref interrupts status
    DFE_FL_JESDRX_QUERY_GET_SYSREF_INTRGRP_STATUS,
    /// JESDRX, get link error count
    DFE_FL_JESDRX_QUERY_GET_LINK_ERR_CNT,     
    /// JESDRX, get lane test data
    DFE_FL_JESDRX_QUERY_GET_LANE_TEST_DATA,
    /// JESDRX, get lane config
    DFE_FL_JESDRX_QUERY_GET_LANE_CFG,
    /// JESDRX, get sysref mode
    DFE_FL_JESDRX_QUERY_GET_SYSREF_MODE,
    /// JESDRX, get lane and link loopback config
    DFE_FL_JESDRX_QUERY_GET_LOOPBACK,
     
    DFE_FL_JESD_QUERY_MAX_VALUE
} DfeFl_JesdHwStatusQuery;


/** @brief lanes
 */
typedef enum
{
    /// JESD lane 0
    DFE_FL_JESD_LANE_0 = 0,
    /// JESD lane 1
    DFE_FL_JESD_LANE_1,
    /// JESD lane 2
    DFE_FL_JESD_LANE_2,
    /// JESD lane 3
    DFE_FL_JESD_LANE_3,
    /// JESD lane ALL
    DFE_FL_JESD_LANE_ALL = 0xf
} DfeFl_JesdLane;

/** @brief links
 */
typedef enum
{
    /// JESD link 0
    DFE_FL_JESD_LINK_0 = 0,
    /// JESD link 1
    DFE_FL_JESD_LINK_1,
    
    /// JESD link ALL
    DFE_FL_JESD_LINK_ALL = 0x3
} DfeFl_JesdLink;

/** @brief jesdtx tx path
 */
typedef enum
{
    DFE_FL_JESDTX_TX_0 = 0,
    DFE_FL_JESDTX_TX_1,
    
    DFE_FL_JESDTX_TX_ALL = 0x3
} DfeFl_JesdTxTxPath;

/** @brief jesdtx tx bus
 */
typedef enum
{
    /// JESDTX TX Bus I0
    DFE_FL_JESDTX_TXBUS_I0 = 0,
    /// JESDTX TX Bus Q0
    DFE_FL_JESDTX_TXBUS_Q0,
    /// JESDTX TX Bus I1
    DFE_FL_JESDTX_TXBUS_I1,
    /// JESDTX TX Bus Q1
    DFE_FL_JESDTX_TXBUS_Q1,
    
    /// JESDTX TX Bus ALL
    DFE_FL_JESDTX_TXBUS_ALL = 0xF
} DfeFl_JesdTxTxBus;

/** @brief jesdtx test sequence 
 */
typedef enum
{
    /// JESDTX test seq, disable
    DFE_FL_JESDTX_TEST_SEQ_DIS = 0,
    /// JESDTX test seq, D21_5
    DFE_FL_JESDTX_TEST_SEQ_D21_5,
    /// JESDTX test seq, K28_5
    DFE_FL_JESDTX_TEST_SEQ_K28_5,
    /// JESDTX test seq, ILA
    DFE_FL_JESDTX_TEST_SEQ_ILA,
    /// JESDTX test seq, RPAT
    DFE_FL_JESDTX_TEST_SEQ_RPAT,
    /// JESDTX test seq, JSPAT
    DFE_FL_JESDTX_TEST_SEQ_JSPAT,
    /// JESDTX test seq, K28_7
    DFE_FL_JESDTX_TEST_SEQ_K28_7
        
} DfeFl_JesdTxTestSeq;

/** @brief jesdtx signal generator
 */
typedef enum
{
    /// JESDTX SigGen at TXI0
    DFE_FL_JESDTX_SIGNAL_GEN_TXI0 = 0,
    /// JESDTX SigGen at TXQ0
    DFE_FL_JESDTX_SIGNAL_GEN_TXQ0,
    /// JESDTX SigGen at TXI1
    DFE_FL_JESDTX_SIGNAL_GEN_TXI1,
    /// JESDTX SigGen at TXQ1
    DFE_FL_JESDTX_SIGNAL_GEN_TXQ1,
    
    /// JESDTX SigGen ALL
    DFE_FL_JESDTX_SIGNAL_GEN_ALL = 0xF
} DfeFl_JesdTxSignalGen;

/** @brief jesdtx checksum lane
 */
typedef enum
{
    /// JESDTX CHKSUM at LANE_0
    DFE_FL_JESDTX_CHKSUM_LANE_0 = 0,
    /// JESDTX CHKSUM at LANE_1
    DFE_FL_JESDTX_CHKSUM_LANE_1,
    /// JESDTX CHKSUM at LANE_2
    DFE_FL_JESDTX_CHKSUM_LANE_2,
    /// JESDTX CHKSUM at LANE_3
    DFE_FL_JESDTX_CHKSUM_LANE_3,
    
    /// JESDTX CHKSUM at LANE_ALL
    DFE_FL_JESDTX_CHKSUM_LANE_ALL = 0xF
} DfeFl_JesdTxChksumLane;

/** @brief jesdtx signal gen ramp mode
 */
typedef enum
{
    /// JESDTX SigGen Ramp Mode, LFSR
    DFE_FL_JESDTX_TESTGEN_RAMP_MODE_LFSR = 0,
    /// JESDTX SigGen Ramp Mode, RAMP
    DFE_FL_JESDTX_TESTGEN_RAMP_MODE_RAMP = 1  
} DfeFl_JesdTxTestGenRampMode;

/** @brief jesd checksum return mode */
typedef enum 
{
    /// JESDTX checksum result, CHKSUM
    DFE_FL_JESDTX_CHKSUM_MODE_RETURN_CHKSUM = 0,
    /// JESDTX checksum result, LATENCY
    DFE_FL_JESDTX_CHKSUM_MODE_RETURN_LATENCY
} DfeFl_JesdTxChksumMode;


/** @brief jesdrx rx path
 */
typedef enum
{
    /// JESDRX path RX0_RX
    DFE_FL_JESDRX_RX0_RX = 0,
    /// JESDRX path RX1_FB0
    DFE_FL_JESDRX_RX1_FB0,
    /// JESDRX path RX2_FB1
    DFE_FL_JESDRX_RX2_FB1,
    
    /// JESDRX path ALL
    DFE_FL_JESDRX_RX_ALL = 0x7
} DfeFl_JesdRxRxPath;

/** @brief jesdrx rx bus
 */
typedef enum
{
    /// JESDRX Rx bus RX0I
    DFE_FL_JESDRX_RXBUS_RX0I = 0,
    /// JESDRX Rx bus RX0Q
    DFE_FL_JESDRX_RXBUS_RX0Q,
    /// JESDRX Rx bus RX1I_FB0I
    DFE_FL_JESDRX_RXBUS_RX1I_FB0I,
    /// JESDRX Rx bus RX1Q_FB0Q
    DFE_FL_JESDRX_RXBUS_RX1Q_FB0Q,
    /// JESDRX Rx bus RX2I_FB1I
    DFE_FL_JESDRX_RXBUS_RX2I_FB1I,
    /// JESDRX Rx bus RX2Q_FB1Q
    DFE_FL_JESDRX_RXBUS_RX2Q_FB1Q,
    
    /// JESDRX Rx bus ALL
    DFE_FL_JESDRX_RXBUS_ALL = 0x3F
} DfeFl_JesdRxRxBus;

/** @brief jesdrx sync_n_out sync bus
 */
typedef enum
{
    /// JESDRX SYNC~ out bus 0
    DFE_FL_JESDRX_SYNCN_OUT_BUS_0 = 0,
    /// JESDRX SYNC~ out bus 1
    DFE_FL_JESDRX_SYNCN_OUT_BUS_1,
    
    /// JESDRX SYNC~ out bus ALL
    DFE_FL_JESDRX_SYNCN_OUT_BUS_ALL = 0x3
} DfeFl_JesdRxSyncnOutBus;

/** @brief jesdtx test bus
 */
typedef enum
{
    /// JESD Tx Testbus,  DISABLE
    DFE_FL_JESDTX_TESTBUS_SEL_DISABLE          = 0x00,

    /// JESD Tx Testbus,  LANE0_INPUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE0_INPUT      = 0x01,
    /// JESD Tx Testbus,  LANE0_SCR_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE0_SCR_OUT    = 0x02,
    /// JESD Tx Testbus,  LANE0_ALIGN_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE0_ALIGN_OUT  = 0x03,
    /// JESD Tx Testbus,  LANE0_ENC_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE0_ENC_OUT    = 0x04,
    /// JESD Tx Testbus,  LANE0_OUTPUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE0_OUTPUT     = 0x05,

    /// JESD Tx Testbus,  LANE1_INPUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE1_INPUT      = 0x11,
    /// JESD Tx Testbus,  LANE1_SCR_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE1_SCR_OUT    = 0x12,
    /// JESD Tx Testbus,  LANE1_ALIGN_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE1_ALIGN_OUT  = 0x13,
    /// JESD Tx Testbus,  LANE1_ENC_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE1_ENC_OUT    = 0x14,
    /// JESD Tx Testbus,  LANE1_OUTPUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE1_OUTPUT     = 0x15,
    
    /// JESD Tx Testbus,  LANE2_INPUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE2_INPUT      = 0x21,
    /// JESD Tx Testbus,  LANE2_SCR_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE2_SCR_OUT    = 0x22,
    /// JESD Tx Testbus,  LANE2_ALIGN_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE2_ALIGN_OUT  = 0x23,
    /// JESD Tx Testbus,  LANE2_ENC_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE2_ENC_OUT    = 0x24,
    /// JESD Tx Testbus,  LANE2_OUTPUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE2_OUTPUT     = 0x25,
    
    /// JESD Tx Testbus,  LANE3_INPUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE3_INPUT      = 0x31,
    /// JESD Tx Testbus,  LANE3_SCR_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE3_SCR_OUT    = 0x32,
    /// JESD Tx Testbus,  LANE3_ALIGN_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE3_ALIGN_OUT  = 0x33,
    /// JESD Tx Testbus,  LANE3_ENC_OUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE3_ENC_OUT    = 0x34,
    /// JESD Tx Testbus,  LANE3_OUTPUT
    DFE_FL_JESDTX_TESTBUS_SEL_LANE3_OUTPUT     = 0x35,
    
    /// JESD Tx Testbus,  TX0_MUX_IN
    DFE_FL_JESDTX_TESTBUS_SEL_TX0_MUX_IN       = 0x81,
    /// JESD Tx Testbus,  TX0_TESTGEN
    DFE_FL_JESDTX_TESTBUS_SEL_TX0_TESTGEN      = 0x82,
    /// JESD Tx Testbus,  TX1_MUX_IN
    DFE_FL_JESDTX_TESTBUS_SEL_TX1_MUX_IN       = 0x91,
    /// JESD Tx Testbus,  TX1_TESTGEN
    DFE_FL_JESDTX_TESTBUS_SEL_TX1_TESTGEN      = 0x92
    
} DfeFl_JesdTxTestBusSel;

/** @brief jesdtx test bus
 */
typedef enum
{
    /// JESD Rx Testbus,  DISABLE
    DFE_FL_JESDRX_TESTBUS_SEL_DISABLE          = 0x00,
    
    /// JESD Rx Testbus,  LANE0_INPUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE0_INPUT      = 0x01,
    /// JESD Rx Testbus,  LANE0_DEC_OUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE0_DEC_OUT    = 0x02,
    /// JESD Rx Testbus,  LANE0_CMA_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE0_CMA_ALGN   = 0x03,
    /// JESD Rx Testbus,  LANE0_LANE_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE0_LANE_ALGN  = 0x04,
    /// JESD Rx Testbus,  LANE0_FRM_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE0_FRM_ALGN   = 0x05,   
    /// JESD Rx Testbus,  LANE0_DESCR_OUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE0_DESCR_OUT  = 0x06,
    /// JESD Rx Testbus,  LANE0_OUTPUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE0_OUTPUT     = 0x07,
    
    /// JESD Rx Testbus,  LANE1_INPUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE1_INPUT      = 0x11,
    /// JESD Rx Testbus,  LANE1_DEC_OUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE1_DEC_OUT    = 0x12,
    /// JESD Rx Testbus,  LANE1_CMA_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE1_CMA_ALGN   = 0x13,
    /// JESD Rx Testbus,  LANE1_LANE_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE1_LANE_ALGN  = 0x14,
    /// JESD Rx Testbus,  LANE1_FRM_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE1_FRM_ALGN   = 0x15,   
    /// JESD Rx Testbus,  LANE1_DESCR_OUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE1_DESCR_OUT  = 0x16,
    /// JESD Rx Testbus,  LANE1_OUTPUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE1_OUTPUT     = 0x17,
    
    /// JESD Rx Testbus,  LANE2_INPUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE2_INPUT      = 0x21,
    /// JESD Rx Testbus,  LANE2_DEC_OUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE2_DEC_OUT    = 0x22,
    /// JESD Rx Testbus,  LANE2_CMA_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE2_CMA_ALGN   = 0x23,
    /// JESD Rx Testbus,  LANE2_LANE_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE2_LANE_ALGN  = 0x24,
    /// JESD Rx Testbus,  LANE2_FRM_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE2_FRM_ALGN   = 0x25,   
    /// JESD Rx Testbus,  LANE2_DESCR_OUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE2_DESCR_OUT  = 0x26,
    /// JESD Rx Testbus,  LANE2_OUTPUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE2_OUTPUT     = 0x27,
    
    /// JESD Rx Testbus,  LANE3_INPUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE3_INPUT      = 0x31,
    /// JESD Rx Testbus,  LANE3_DEC_OUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE3_DEC_OUT    = 0x32,
    /// JESD Rx Testbus,  LANE3_CMA_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE3_CMA_ALGN   = 0x33,
    /// JESD Rx Testbus,  LANE3_LANE_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE3_LANE_ALGN  = 0x34,
    /// JESD Rx Testbus,  LANE3_FRM_ALGN
    DFE_FL_JESDRX_TESTBUS_SEL_LANE3_FRM_ALGN   = 0x35,   
    /// JESD Rx Testbus,  LANE3_DESCR_OUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE3_DESCR_OUT  = 0x36,
    /// JESD Rx Testbus,  LANE3_OUTPUT
    DFE_FL_JESDRX_TESTBUS_SEL_LANE3_OUTPUT     = 0x37,
    
    /// JESD Rx Testbus,  RX0_BFR_LPBK
    DFE_FL_JESDRX_TESTBUS_SEL_RX0_BFR_LPBK     = 0x81,
    /// JESD Rx Testbus,  RX0_AFT_LPBK
    DFE_FL_JESDRX_TESTBUS_SEL_RX0_AFT_LPBK     = 0x82,
    /// JESD Rx Testbus,  RX1_BFR_LPBK
    DFE_FL_JESDRX_TESTBUS_SEL_RX1_BFR_LPBK     = 0x91,
    /// JESD Rx Testbus,  RX1_AFT_LPBK
    DFE_FL_JESDRX_TESTBUS_SEL_RX1_AFT_LPBK     = 0x92,
    /// JESD Rx Testbus,  RX2_BFR_LPBK
    DFE_FL_JESDRX_TESTBUS_SEL_RX2_BFR_LPBK     = 0xA1,
    /// JESD Rx Testbus,  RX2_AFT_LPBK
    DFE_FL_JESDRX_TESTBUS_SEL_RX2_AFT_LPBK     = 0xA2
    
}  DfeFl_JesdRxTestBusSel;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_JESD_DATASTRUCT
 * @{
 */

/**
 * @brief inits config, argument for runtime control,
 *  DFE_FL_JESDTX_CMD_CFG_INITS
 *  DFE_FL_JESDRX_CMD_CFG_INITS
 */
typedef struct {
    /// common block inits
    DfeFl_SublkInitsConfig cmn;
    /// clear data lane
    uint32_t clearDataLane[4];
} DfeFl_JesdInitsConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_CLRLANE
 *      DFE_FL_JESDRX_CMD_CFG_CLRLANE
 */
typedef struct
{
    /// lane#, 0 ~ 3
    uint32_t lane;
    /// 1 to clear data; 0 no clearing (normal)
    uint32_t clearData;
} DfeFl_JesdClearDataLaneConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_TX_INPUTS
 */
typedef struct
{
    /// tx path#, 0/1
    uint32_t txPath;
    /// cken delay
    uint32_t ckenDly;
    /// jesdrxmap to jesdtxmap loopback for tx
    uint32_t rxtxLoopbackEnable;
} DfeFl_JesdTxInputsConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_TEST_SEQ
 */
typedef struct
{
    /// lane#, 0 ~ 3 
    uint32_t lane;
    /// test seqeunce
    uint32_t testSeq;
} DfeFl_JesdTxTestSeqConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_SYNCN_LOOPBACK
 *      DFE_FL_JESDTX_QUERY_GET_SYNCN_LOOPBACK
 */
typedef struct
{
    /// link#, 0/1
    uint32_t link;
    /// 1 to enable sync_n loopback; 0 normal
    uint32_t syncnLoopback;
} DfeFl_JesdTxSyncnLoopbackConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_SYNCN_INVERT
 */
typedef struct
{
    /// link#, 0/1
    uint32_t link;
    /// 1 to enable sync_n polarity; 0 not invert
    uint32_t syncnInvert;
} DfeFl_JesdTxSyncnInvertConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_BB_CTRL
 */
typedef struct
{
    /// 0 = disable BB interface, otherwise each bit enables BB input to each lane
    uint32_t laneEnable;
    /// link select for BB interface to mux a multiframe alignment signal to the BB
    uint32_t linkSel;
} DfeFl_JesdTxBbCtrlConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_FORCE_SYSREF_REQUEST
 *      DFE_FL_JESDRX_CMD_FORCE_SYSREF_REQUEST
 */
typedef struct
{
    /// 1 to force sysref request
    uint32_t force;
    /// auto off timer for forced SYSREF request, 0 = disable auto off
    uint32_t autoOffTimer;
} DfeFl_JesdForceSysrefReqConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_QUERY_GET_LANE_SYNC_STATE
 */
typedef struct
{
    /// lane#, 0 ~ 3
    uint32_t lane;
    /// lane sync state
    uint32_t state;
} DfeFl_JesdTxLaneSyncStateQuery;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_QUERY_GET_LANE_SYNC_STATE
 */
typedef struct
{
    /// lane#, 0 ~ 3
    uint32_t lane;
    /// lane code state
    uint32_t codeState;
    /// lane Frame state
    uint32_t frameState;
} DfeFl_JesdRxLaneSyncStateQuery;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_QUERY_GET_FIRST_SYNC_REQUEST
 */
typedef struct
{
    /// link#, 0/1
    uint32_t link;
    /// 1 indicate first sync request come; 0 not
    uint32_t request;
} DfeFl_JesdTxFirstSyncRequestQuery;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_SET_TESTGEN_SSEL
 */
typedef struct
{
    /// SigGen device
    DfeFl_JesdTxSignalGen tgDev;
    /// sync select
    uint32_t ssel;    
} DfeFl_JesdTxTestGenSselConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_SET_CHKSUM_SSEL
 *      DFE_FL_JESDRX_CMD_SET_CHKSUM_SSEL
 */
typedef struct
{
    /// checksum select
    uint32_t chksumLane;
    /// sync select
    uint32_t ssel;    
} DfeFl_JesdChksumSselConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_SET_INITSTATELINK_SSEL
 *      DFE_FL_JESDRX_CMD_SET_INITSTATELINK_SSEL
 */
typedef struct
{
    /// link#, 0/1
    uint32_t link;
    /// sync select
    uint32_t ssel;    
} DfeFl_JesdInitStateLinkSselConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_TESTGEN
 */
typedef struct
{
    /// SigGen device
    DfeFl_JesdTxSignalGen tgDev;
    /// enable data generation
    uint32_t genData;
    /// enbale frame generation
    uint32_t genFrame;
    /// ramp (1), or LFSR (0)
    DfeFl_JesdTxTestGenRampMode rampMode;
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
} DfeFl_JesdTxTestGenConfig;


/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_CHKSUM
 *      DFE_FL_JESDRX_CMD_CFG_CHKSUM
 */
typedef struct
{
    /// checksum device
    uint32_t chksumDev;
    /// checksum result mode
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
} DfeFl_JesdTxChksumConfig, DfeFl_JesdRxChksumConfig;
    
/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_QUERY_CHKSUM_RESULT
 *      DFE_FL_JESDRX_QUERY_CHKSUM_RESULT
 */
typedef struct
{
    /// checksum select
    uint32_t chksumLane;
    /// checksum result
    uint32_t result;
} DfeFl_JesdChksumResultQuery;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_CLKGATE
 *      DFE_FL_JESDRX_CMD_CFG_LINK_CLKGATE
 *      DFE_FL_JESDRX_CMD_CFG_PATH_CLKGATE 
 */
typedef struct
{
    /// link path
    uint32_t linkPath;
    /// clock gate config
    DfeFl_ClockGateConfig cgCfg;
} DfeFl_JesdClockGateConfig;


/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_LANE
 *      DFE_FL_JESDRX_CMD_CFG_LANE
 *      DFE_FL_JESDTX_QUERY_GET_LANE_CFG
 *      DFE_FL_JESDRX_QUERY_GET_LANE_CFG
 */
typedef struct
{
    /// lane#, 0 ~ 3
    uint32_t lane;
    /// 1 enable the lane
    uint32_t laneEnable;
    /// link# assigned
    uint32_t linkAssign;
    /// laneId assigned
    uint32_t laneId;
} DfeFl_JesdLaneConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_LINK
 */
typedef struct
{
    /// link#, 0/`
    uint32_t link;
    /// Device (link) ID
    uint32_t deviceId;
    /// Bank ID � Extension to DID
    uint32_t bankId;
    /// Number of adjustment resolution steps to adjust DAC LMFC. Applies to Subclass 2 operation only.
    uint32_t adjCnt;
    /// Phase adjustment request to DAC. Subclass 2 only.
    uint32_t phyAdj;
    /// Direction to adjust DAC LMFC. 0 � Advance, 1 � Delay. Applies to Subclass 2 operation only.
    uint32_t adjDir;
    /// Number of lanes per converter device (link) minus 1
    uint32_t numLanesM1;
    /// Scrambling enabled
    uint32_t scramble;
    /// Number of octets per frame minus 1
    uint32_t numOctetsPerFrameM1;
    /// Number of frames per multiframe minus 1
    uint32_t numFramesPerMultiframe;
    /// Number of converters per device minus 1
    uint32_t numCnvtsM1;
    /// Converter resolution minus 1
    uint32_t cnvtResolutionM1;
    /// Number of control bits per sample
    uint32_t numCtrlBits;
    /// Total number of bits per sample minus 1
    uint32_t totalBitsPerSample;
    /// Device Subclass Version. 000 � Subclass 0, 001 � Subclass 1, 010 � Subclass 2
    uint32_t subclass;
    /// Number of samples per converter per frame cycle minus 1
    uint32_t numSamplesPerFrameM1;
    /// JESD204 version. 000 � JESD204A, 001 � JESD204B
    uint32_t jesd204Ver;
    /// Number of control words per frame clock period per link
    uint32_t numCtrlWordsPerFrame;
    /// High Density format
    uint32_t hiDesity;
    /// Reserved field 1
    uint32_t rsvd1;
    /// Reserved field 2
    uint32_t rsvd2;
    /// Number of multiframes in the ILA sequence minus 1
    uint32_t ILAnumMultiframesM1;
    /// 1 = receiver does not support lane synchronization (do not send ILA sequence or /A/ multiframe alignment characters)
    uint32_t noLaneSync;
    /// multipoint link enable
    uint32_t mpLinkEnable;
    /// sysref sampling mode. 
    ///  0 = ignore all sysrefs, 
    ///  1 = use all sysrefs, 
    ///  2 = use only next sysref, 
    ///  3 = skip one sysref and then use one (use only next, next sysref), 
    ///  4 = skip one sysref and then use all, 
    ///  5 = skip two sysrefs and then use one, 
    ///  6 = skip two sysrefs and then use all
    uint32_t sysrefMode;
} DfeFl_JesdTxLinkConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_SET_SYSREF_MODE
 *      DFE_FL_JESDRX_CMD_SET_SYSREF_MODE
 *      DFE_FL_JESDTX_QUERY_GET_SYSREF_MODE
 *      DFE_FL_JESDRX_QUERY_GET_SYSREF_MODE
 */
typedef struct
{
    /// link#, 0/1
    uint32_t link;
    /// sysref sampling mode. 
    ///  0 = ignore all sysrefs, 
    ///  1 = use all sysrefs, 
    ///  2 = use only next sysref, 
    ///  3 = skip one sysref and then use one (use only next, next sysref), 
    ///  4 = skip one sysref and then use all, 
    ///  5 = skip two sysrefs and then use one, 
    ///  6 = skip two sysrefs and then use all
    uint32_t mode;
} DfeFl_JesdLinkSysrefModeConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_QUERY_GET_LINK_ERR_CNT
 *      DFE_FL_JESDRX_QUERY_GET_LINK_ERR_CNT
 */
typedef struct
{
    /// link#, 0/1
    uint32_t link;
    /// link error count
    uint32_t errCnt;
} DfeFl_JesdLinkErrCntQuery;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_ENB_LANE_INTRGRP
 *      DFE_FL_JESDTX_CMD_DIS_LANE_INTRGRP
 *      DFE_FL_JESDTX_CMD_CLR_LANE_INTRGRP_STATUS
 *      DFE_FL_JESDTX_CMD_SET_FORCE_LANE_INTRGRP
 *      DFE_FL_JESDTX_CMD_CLR_FORCE_LANE_INTRGRP
 *      DFE_FL_JESDTX_QUERY_GET_LANE_INTRGRP_STATUS
 */
typedef struct
{
    /// lane#, 0 ~ 3
    uint32_t lane;
    /// fifo empty
    uint32_t fifoEmptyIntr;
    /// fifo read error because fifo empty
    uint32_t fifoReadErrIntr;
    /// fifo full
    uint32_t fifoFullIntr;
    /// fifo write error because fifo full
    uint32_t fifoWriteErrIntr;
} DfeFl_JesdTxLaneIntrs;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_ENB_SYSREF_INTRGRP
 *      DFE_FL_JESDTX_CMD_DIS_SYSREF_INTRGRP
 *      DFE_FL_JESDTX_CMD_CLR_SYSREF_INTRGRP_STATUS
 *      DFE_FL_JESDTX_CMD_SET_FORCE_SYSREF_INTRGRP
 *      DFE_FL_JESDTX_CMD_CLR_FORCE_SYSREF_INTRGRP
 *      DFE_FL_JESDTX_QUERY_GET_SYSREF_INTRGRP_STATUS
 *      DFE_FL_JESDRX_CMD_ENB_SYSREF_INTRGRP
 *      DFE_FL_JESDRX_CMD_DIS_SYSREF_INTRGRP
 *      DFE_FL_JESDRX_CMD_CLR_SYSREF_INTRGRP_STATUS
 *      DFE_FL_JESDRX_CMD_SET_FORCE_SYSREF_INTRGRP
 *      DFE_FL_JESDRX_CMD_CLR_FORCE_SYSREF_INTRGRP
 *      DFE_FL_JESDRX_QUERY_GET_SYSREF_INTRGRP_STATUS
 */
typedef struct
{
    /// sysref request assertion
    uint32_t reqAssertIntr;
    /// sysref request de-assertion
    uint32_t reqDeassertIntr;
    /// sysref error for link0
    uint32_t errLink0Intr;
    /// sysref error for link1
    uint32_t errLink1Intr;
} DfeFl_JesdTxSysrefIntrs;

typedef struct
{
    /// sysref request assertion
    uint32_t reqAssertIntr;
    /// sysref request de-assertion
    uint32_t reqDeassertIntr;
    /// sysref error for link0
    uint32_t errLink0Intr;
    /// sysref error for link1
    uint32_t errLink1Intr;
} DfeFl_JesdRxSysrefIntrs;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_MAP_LANE
 */
typedef struct
{
    /// lane# 0 ~ 3
    uint32_t lane;
    
    /// array [nibble][position]
    /// tx bus nibble select
    uint32_t nibbleSel[4][4];
    /// tx bus frame postion
    uint32_t framePos[4][4];
} DfeFl_JesdTxMapLaneConfig;
    
/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_MAP_NIBB
 */
typedef struct
{
    /// tx bus# 0 ~ 3
    uint32_t txBus;
    
    /// array [nibble]
    /// number of framee minus one
    uint32_t numFrameM1[4];
    /// link select
    uint32_t linkSel[4];
} DfeFl_JesdTxMapNibbConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_NIBB_TEST_ENABLE
 */
typedef struct
{
    /// tx bus# 0 ~ 3
    uint32_t txBus;
    
    /// array [nibble]
    uint32_t testPatEnable[4];
} DfeFl_JesdTxNibbTestEnableConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDTX_CMD_CFG_NIBB_TEST_DATA
 */
typedef struct
{
    /// tx bus# 0 ~ 3
    uint32_t txBus;
    
    /// array [nibble][position]
    uint32_t testData[4][4];
} DfeFl_JesdTxNibbTestDataConfig;





/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_CFG_CLRLANE
 */
typedef struct
{
    /// lane# 0 ~ 3
    uint32_t lane;
    /// clear data enable
    uint32_t clearData;
} DfeFl_JesdRxClearLaneConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_CFG_TESTSEQ
 */
typedef struct
{
    /// lane# 0 ~ 3
    uint32_t lane;
    /// test sequence select
    uint32_t testSeqSel;
} DfeFl_JesdRxTestSeqConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_QUERY_GET_LOOPBACK
 */
typedef struct
{
    /// lane0 loopback
    uint32_t lane0;
    /// lane1 loopback
    uint32_t lane1;
    /// lane2 loopback
    uint32_t lane2;
    /// lane3 loopback
    uint32_t lane3;
    /// tx0 to rx0 loopback
    uint32_t tx0_rx0;
    /// tx0 to rx1 loopback
    uint32_t tx0_rx1fb0;
    /// tx0 to rx2 loopback
    uint32_t tx1_rx2fb1;
} DfeFl_JesdRxLoopbackConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_CFG_BBRXCTRL
 */
typedef struct
{
    /// BB out lane select
    uint32_t bbOutLaneSel;
    /// BB out enable
    uint32_t bbOutEna;
    /// force Frame signal high to Rx0
    uint32_t forceFrameRx0;
    /// force Frame signal high to Rx1
    uint32_t forceFrameRx1Fb0;
    /// force Frame signal high to Rx2
    uint32_t forceFrameRx2Fb1;
} DfeFl_JesdRxBbRxCtrlConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_CFG_FIFO
 */
typedef struct
{
    /// FIFO read delay applied to all SERDES RX FIFOs
    uint32_t readDly;
    /// 0 = allow FIFO errors to zero data
    uint32_t disableZeroData;
} DfeFl_JesdRxFifoConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_CFG_SYNCN_OUT
 */
typedef struct
{
    /// SYNC~ output mux select for link 0
    uint32_t selLink0;
    /// SYNC~ output mux select for link 1
    uint32_t selLink1;
    /// enable sync selected by sync_n_out_sync_bus_ssel_0 to be output on SYNC~ 0
    uint32_t syncBusEnable0;
    /// enable sync selected by sync_n_out_sync_bus_ssel_1 to be output on SYNC~ 1
    uint32_t syncBusEnable1;
    /// SYNC~ output polarity invert for link 0, does not apply if sync_bus_ena0 set
    uint32_t invertLink0;
    /// SYNC~ output polarity invert for link 1, does not apply if sync_bus_ena1 set
    uint32_t invertLink1;
} DfeFl_JesdRxSyncnOutConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_SET_SYNCN_OUT_BUS_SSEL
 */
typedef struct
{
    /// sync_n_out sync bus# 0/1
    uint32_t bus;
    /// sync select
    uint32_t ssel;    
} DfeFl_JesdRxSycnOutBusSselConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_CFG_LINK
 */
typedef struct
{
    /// link# 0 ~ 1
    uint32_t link;
    /// Device (link) ID
    uint32_t deviceId;
    /// Bank ID � Extension to DID
    uint32_t bankId;
    /// Number of adjustment resolution steps to adjust DAC LMFC. Applies to Subclass 2 operation only.
    uint32_t adjCnt;
    /// Phase adjustment request to DAC. Subclass 2 only.
    uint32_t phyAdj;
    /// Direction to adjust DAC LMFC. 0 � Advance, 1 � Delay. Applies to Subclass 2 operation only.
    uint32_t adjDir;
    /// Number of lanes per converter device (link) minus 1
    uint32_t numLanesM1;
    /// Scrambling enabled
    uint32_t scramble;
    /// Number of octets per frame minus 1
    uint32_t numOctetsPerFrameM1;
    /// Number of frames per multiframe minus 1
    uint32_t numFramesPerMultiframe;
    /// Number of converters per device minus 1
    uint32_t numCnvtsM1;
    /// Converter resolution minus 1
    uint32_t cnvtResolutionM1;
    /// Number of control bits per sample
    uint32_t numCtrlBits;
    /// Number of control bits per sample
    uint32_t totalBitsPerSample;
    /// Device Subclass Version. 000 � Subclass 0, 001 � Subclass 1, 010 � Subclass 2
    uint32_t subclass;
    /// Number of samples per converter per frame cycle minus 1
    uint32_t numSamplesPerFrameM1;
    /// JESD204 version. 000 � JESD204A, 001 � JESD204B
    uint32_t jesd204Ver;
    /// Number of control words per frame clock period per link
    uint32_t numCtrlWordsPerFrame;
    /// High Density format
    uint32_t hiDesity;
    /// Reserved field 1
    uint32_t rsvd1;
    /// Reserved field 2
    uint32_t rsvd2;
    /// number of frames for RX buffer delay minus 1
    uint32_t rbdM1;
    /// ignore RBD and release buffers as soon as possible (support for Subclass 0)
    uint32_t minLatencyEnable;
    /// 1 = match with specified character to start buffering, 0 = start buffering with first non-/K/
    uint32_t matchSpecific;
    /// 1 = match character is control character (typically 1)
    uint32_t matchCtrl;
    /// specific control character to start buffering (typically /R/ = /K.28.0/ = 0x1C)
    uint32_t matchData;
    /// choose which errors trigger sync request
    uint32_t syncRequestEnable;
    /// choose which errors contribute towards error count and error reporting
    uint32_t errorEnable;
    /// sysref sampling mode. 
    ///  0 = ignore all sysrefs, 
    ///  1 = use all sysrefs, 
    ///  2 = use only next sysref, 
    ///  3 = skip one sysref and then use one (use only next, next sysref), 
    ///  4 = skip one sysref and then use all, 
    ///  5 = skip two sysrefs and then use one, 
    ///  6 = skip two sysrefs and then use all
    uint32_t sysrefMode;
    /// 1 = transmitter does not support lane synchronization. do not check link configuration data.
    uint32_t noLaneSync;
    /// suppress error reporting for subclass 0 operation. errors won't trigger sync_n but will be counted.
    uint32_t disableErrReport;
    /// multipoint link enable
    uint32_t mpLinkEnable;
} DfeFl_JesdRxLinkConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_ENB_LANE_INTRGRP
 *      DFE_FL_JESDRX_CMD_DIS_LANE_INTRGRP
 *      DFE_FL_JESDRX_CMD_CLR_LANE_INTRGRP_STATUS
 *      DFE_FL_JESDRX_CMD_SET_FORCE_LANE_INTRGRP
 *      DFE_FL_JESDRX_CMD_CLR_FORCE_LANE_INTRGRP
 *      DFE_FL_JESDRX_QUERY_GET_LANE_INTRGRP_STATUS
 */
typedef struct
{
    /// lane# 0 ~ 3
    uint32_t lane;
    /// captured interrupt bit for 8b/10b disparity error
    uint32_t decDispErrIntr;
    /// captured interrupt bit for 8b/10b not-in-table code error
    uint32_t decCodeErrIntr;
    /// captured interrupt bit for code synchronization error
    uint32_t codeSyncErrIntr;
    /// captured interrupt bit for elastic buffer match error (first non-/K/ doesn't match match_ctrl and match_data)
    uint32_t bufMatchErrIntr;
    /// captured interrupt bit for elastic buffer overflow error (bad RBD value)
    uint32_t bufOverflowErrIntr;
    /// captured interrupt bit for link configuration error
    uint32_t linkConfigErrIntr;
    /// captured interrupt bit for frame alignment error
    uint32_t frameAlignErrIntr;
    /// captured interrupt bit for multiframe alignment error
    uint32_t multiframeAlignErrIntr;
    /// captured interrupt bit for FIFO empty flag (write 0 to clear)
    uint32_t fifoEmptyIntr;
    /// captured interrupt bit for FIFO read error (write 0 to clear)
    uint32_t fifoReadErrIntr;
    /// captured interrupt bit for FIFO full flag (write 0 to clear)
    uint32_t fifoFullIntr;
    /// captured interrupt bit for FIFO write error (write 0 to clear)
    uint32_t fifoWriteErrIntr;
    /// captured interrupt bit for test sequence verification fail
    uint32_t testSeqErrIntr;
} DfeFl_JesdRxLaneIntrs;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_CMD_CFG_MAP_NIBB
 */
typedef struct
{
    /// rx bus#
    uint32_t rxBus;
    
    /// array [nibble][position]
    /// lane selection
    uint32_t laneSel[4][4];
    /// nibble in the lane
    uint32_t laneNibbSel[4][4];
    /// time slot in the lane
    uint32_t timeSlotSel[4][4];
    /// zero bits out
    uint32_t zeroBits[4][4];
} DfeFl_JesdRxMapNibbConfig;

/** @brief argument for runtime control,
 *      DFE_FL_JESDRX_QUERY_GET_LANE_TEST_DATA
 */
typedef struct
{
    /// lane# 0 ~ 3
    uint32_t lane;
    
    /// array [position]
    uint32_t testData[4];
} DfeFl_JesdRxLaneTestDataQuery;

/** @brief argument for runtime control,
 *
 */
typedef struct
{
	DfeFl_JesdTxLaneIntrs 	jesdTxLane;
	DfeFl_JesdTxSysrefIntrs jesdTxSysref;
	DfeFl_JesdRxLaneIntrs	jesdRxLane;
	DfeFl_JesdRxSysrefIntrs	jesdRxSysref;

} DfeFl_JesdGroupIntrStatus;

/** @brief overlay register pointer to JESD instance
 */
typedef CSL_DFE_JESD_REGS *DfeFl_JesdRegsOvly;

/** @brief JESD Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle           hDfe;
    
    /// pointer to register base address of a JESD instance
    DfeFl_JesdRegsOvly      regs;
   
    /// This is the instance of JESD being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_JesdObj;

/** @brief handle pointer to JESD object
 */
typedef DfeFl_JesdObj *DfeFl_JesdHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_JESD_FUNCTION
 * @{
 */

DfeFl_JesdHandle dfeFl_JesdOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_JesdObj              *pDfeJesdObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_JesdClose(DfeFl_JesdHandle hDfeJesd);

DfeFl_Status  dfeFl_JesdHwControl
(
    DfeFl_JesdHandle            hDfeJesd,
    DfeFl_JesdHwControlCmd      ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_JesdGetHwStatus
(
    DfeFl_JesdHandle            hDfeJesd,
    DfeFl_JesdHwStatusQuery     queryId,
    void                        *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_JESD_H_ */
