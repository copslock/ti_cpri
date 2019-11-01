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
 *  @defgroup DFE_FL_BB_API BB
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_bb.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_BB CSL
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
 * @defgroup DFE_FL_BB_SYMBOL DFE Bb Symbols
 * @ingroup DFE_FL_BB_API
 */

/**
 * @defgroup DFE_FL_BB_DATASTRUCT DFE Bb Data Structures
 * @ingroup DFE_FL_BB_API
 */

/**
 * @defgroup DFE_FL_BB_ENUM DFE Bb Enumverated Data Types
 * @ingroup DFE_FL_BB_API
 */

/**
 * @defgroup DFE_FL_BB_FUNCTION DFE Bb Functions
 * @ingroup DFE_FL_BB_API
 */

#ifndef _DFE_FL_BB_H_
#define _DFE_FL_BB_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/csl/cslr_dfe_bb.h>
#include <ti/drv/dfe/dfe_fl.h>

/**
 * @addtogroup DFE_FL_BB_SYMBOL
 * @{
 */

#define DFE_FL_BB_MAX_AID_STREAM_ID    256
#define DFE_FL_BB_MAX_SLOTS            256   
#define DFE_FL_BB_MAX_DL_XLATES        256
#define DFE_FL_BB_MAX_UL_XLATES        48
/// max supported carrier types
#define DFE_FL_BB_MAX_CARRIER_TYPES    16
#define DFE_FL_BB_AXC_MAX_SLOTS        16
/// max supported BB power meters each direction
#define DFE_FL_BB_MAX_POWMTRS          16
/// max supported AxCs per antenna
#define DFE_FL_BB_ANTENNA_MAX_AXCS     16

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_BB_ENUM
 * @{
 */
/** @brief control commands
 */
typedef enum
{
    /// BB inits
    DFE_FL_BB_CMD_CFG_INITS,
    /// enable aid loopback
    DFE_FL_BB_CMD_ENB_AID_LOOPBACK,
    /// disable aid loopback
    DFE_FL_BB_CMD_DIS_AID_LOOPBACK,
    /// buf loopback config
    DFE_FL_BB_CMD_CFG_LOOPBACK,
    /// capture buffer config
    DFE_FL_BB_CMD_CFG_CAPBUFF,
    /// test signal generation config
    DFE_FL_BB_CMD_CFG_TESTGEN,
    /// test signal generation sync selection
    DFE_FL_BB_CMD_SET_TESTGEN_SSEL,        
    /// config chksum
    DFE_FL_BB_CMD_CFG_CHKSUM,
    /// select checksum sync source
    DFE_FL_BB_CMD_SET_CHKSUM_SSEL,
    /// select carrier type UL sync strobe
    DFE_FL_BB_CMD_CFG_CT_UL_SYNC_STROBE,
    /// set AID UL strobe delay
    DFE_FL_BB_CMD_CFG_AID_ULSTROBE_DLY,
        
    /// config TXIF_AXC
    DFE_FL_BB_CMD_CFG_TXIF_AXC,
    /// config RXIF_AXC
    DFE_FL_BB_CMD_CFG_RXIF_AXC,
    /// config sync selection for tx gain update
    DFE_FL_BB_CMD_SET_TXGAIN_SSEL,
    /// update TX gain of axc
    DFE_FL_BB_CMD_UPD_TXGAIN,
    /// enable txgain update interrupt
    DFE_FL_BB_CMD_ENB_TXGAIN_INTR,
    /// disable txgain update interrupt
    DFE_FL_BB_CMD_DIS_TXGAIN_INTR,
    /// clear txgain update interrupt status
    DFE_FL_BB_CMD_CLR_TXGAIN_INTR_STATUS,
    /// force set txgain update interrupt
    DFE_FL_BB_CMD_SET_FORCE_TXGAIN_INTR,
    /// clear set txgain update interrupt
    DFE_FL_BB_CMD_CLR_FORCE_TXGAIN_INTR,
    
    /// TX TDD timer config
    DFE_FL_BB_CMD_CFG_TXTDD,
    /// TX TDD timer config sync selection
    DFE_FL_BB_CMD_SET_TXTDD_SSEL,
    /// RX TDD timer config
    DFE_FL_BB_CMD_CFG_RXTDD,
    /// RX TDD timer config sync selection
    DFE_FL_BB_CMD_SET_RXTDD_SSEL,
    
    /** TX power meter commands
     */
    /// config a tx power meter
    DFE_FL_BB_CMD_CFG_TXPM,
    /// config a tx power meter ssel
    DFE_FL_BB_CMD_SET_TXPM_SSEL,
    /// disable update of a tx power meter
    DFE_FL_BB_CMD_DIS_TXPM_UPDATE,
    /// enable interrupt of a tx power meter
    DFE_FL_BB_CMD_ENB_TXPM_INTR,
    /// enable interrupt of a tx power meter
    DFE_FL_BB_CMD_DIS_TXPM_INTR,
    /// clear interrupt status of a tx power meter
    DFE_FL_BB_CMD_CLR_TXPM_INTR_STATUS,
    /// force setting interrupt status of a tx power meter
    DFE_FL_BB_CMD_SET_FORCE_TXPM_INTR,
    /// clear forcing interrupt status of a tx power meter
    DFE_FL_BB_CMD_CLR_FORCE_TXPM_INTR,
    /// enable interrupt to the cpp
    DFE_FL_BB_CMD_ENB_TXPM_AUXINTR,
    /// disable interrupt to the cpp
    DFE_FL_BB_CMD_DIS_TXPM_AUXINTR,
    
    /** RX power meter commands
     */
    /// config a BBRX power meter    
    DFE_FL_BB_CMD_CFG_RXPM,
    /// select sync source for BBRX power meter 
    DFE_FL_BB_CMD_SET_RXPM_SSEL,
    /// disable a BBRX power meter update
    DFE_FL_BB_CMD_DIS_RXPM_UPDATE,
    /// enable a BBRX power meter interrupt
    DFE_FL_BB_CMD_ENB_RXPM_INTR,
    /// disable a BBRX power meter interrupt
    DFE_FL_BB_CMD_DIS_RXPM_INTR,
    /// clear status of a BBRX power mter interrupt
    DFE_FL_BB_CMD_CLR_RXPM_INTR_STATUS,
    /// force generating a BBRX power meter interrupt
    DFE_FL_BB_CMD_SET_FORCE_RXPM_INTR,
    /// clear force generating a BBRX power meter interrupt
    DFE_FL_BB_CMD_CLR_FORCE_RXPM_INTR,
    /// enable BBRX aux interrupts
    DFE_FL_BB_CMD_ENB_RXPM_AUXINTR,
    /// disable BBRX aux interrupts
    DFE_FL_BB_CMD_DIS_RXPM_AUXINTR,
    
    /** Antenna Calibration
     */    
    /// config antenna calibration global
    DFE_FL_BB_CMD_CFG_ANTCAL_GLOBAL,
    /// config antenna calibration
    DFE_FL_BB_CMD_CFG_ANTCAL,
    
    
    
    /** Rx gain control
     */
    /// config sync selection for tx gain update
    DFE_FL_BB_CMD_SET_RXGAIN_SSEL,
    /// update RX gain of axc
    DFE_FL_BB_CMD_UPD_RXGAIN,
    /// enable rx gain update interrupt
    DFE_FL_BB_CMD_ENB_RXGAIN_INTR,
    /// disable rx gain update interrupt
    DFE_FL_BB_CMD_DIS_RXGAIN_INTR,
    /// clear rx gain update interrupt status
    DFE_FL_BB_CMD_CLR_RXGAIN_INTR_STATUS,
    /// force setting rx gain update interrupt status
    DFE_FL_BB_CMD_SET_FORCE_RXGAIN_INTR,
    /// clear force setting rx gain update interrupt status
    DFE_FL_BB_CMD_CLR_FORCE_RXGAIN_INTR,

    /** beAGC control
     */
    /// config beAGC globals
    DFE_FL_BB_CMD_CFG_BEAGC_GLOBAL, 
    /// config a beAGC control loop
    DFE_FL_BB_CMD_CFG_BEAGC,
    /// config a beAGC control loop ssel
    DFE_FL_BB_CMD_SET_BEAGC_SSEL,
    
    /** Rx Notch filter
     */
    /// config Rx Notch filter globals
    DFE_FL_BB_CMD_CFG_RXNOTCH_GLOBAL,
    /// Rx Notch ssel
    DFE_FL_BB_CMD_SET_RXNOTCH_SSEL,
    /// config a Rx Notch filter
    DFE_FL_BB_CMD_CFG_RXNOTCH,
        
    /** BB general interrupts
     */
    /// enable a BB general interrupt    
    DFE_FL_BB_CMD_ENB_GENERAL_INTR,
    /// disable a BB general interrupt    
    DFE_FL_BB_CMD_DIS_GENERAL_INTR,
    /// clear status of a BB general interrupt    
    DFE_FL_BB_CMD_CLR_GENERAL_INTR_STATUS,
    /// force generating a BB general interrupt    
    DFE_FL_BB_CMD_SET_FORCE_GENERAL_INTR,
    /// clear force generating a BB general interrupt    
    DFE_FL_BB_CMD_CLR_FORCE_GENERAL_INTR,
    /// enable group of BB general interrupts
    DFE_FL_BB_CMD_ENB_GENERAL_INTRGRP,
    /// disable group of BB general interrupts
    DFE_FL_BB_CMD_DIS_GENERAL_INTRGRP,
    /// clear status of group of BB general interrupts
    DFE_FL_BB_CMD_CLR_GENERAL_INTRGRP_STATUS,
    /// force generating group of BB general interrupts
    DFE_FL_BB_CMD_SET_FORCE_GENERAL_INTRGRP,
    /// clear force generating group of BB general interrupts
    DFE_FL_BB_CMD_CLR_FORCE_GENERAL_INTRGRP,
    
    
    DFE_FL_BB_CMD_MAX_VALUE
} DfeFl_BbHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /// AID loopback config
    DFE_FL_BB_QUERY_AID_LOOPBACK_CFG = 0,
    /// loopback config
    DFE_FL_BB_QUERY_LOOPBACK_CFG,
    /// capture buffer config
    DFE_FL_BB_QUERY_CAPBUFF_CFG,
    /// test signal generation config
    DFE_FL_BB_QUERY_TESTGEN_CFG,
    /// test signal generation sync selection
    DFE_FL_BB_QUERY_TESTGEN_SSEL,    
    /// chksum ssel
    DFE_FL_BB_QUERY_CHKSUM_SSEL,
    /// chksum result
    DFE_FL_BB_QUERY_CHKSUM_RESULT,
    /// get carrier type UL sync strobe
    DFE_FL_BB_QUERY_CT_UL_SYNC_STROBE,
    // get BB AID UL strobe delay
    DFE_FL_BB_QUERY_AID_ULSTROBE_DLY,

    /// TX GAIN upadte status
    DFE_FL_BB_QUERY_TXGAIN_UPDATE_STATUS,    
    /// config sync selection for tx gain update
    DFE_FL_BB_QUERY_TXGAIN_SSEL,
    /// TX gain config
    DFE_FL_BB_QUERY_TXGAIN_INTR_STATUS,
    
    /// TX TDD timer config
    DFE_FL_BB_QUERY_TXTDD_CFG,
    /// TX TDD timer sync selection
    DFE_FL_BB_QUERY_TXTDD_SSEL,
    /// RX TDD timer config
    DFE_FL_BB_QUERY_RXTDD_CFG,
    /// RX TDD timer sync selection
    DFE_FL_BB_QUERY_RXTDD_SSEL,
    
    /** TX power meter queries
     */    
    /// get BBTX power meter config
    DFE_FL_BB_QUERY_TXPM_CFG,
    /// get BBTX power meter sync select
    DFE_FL_BB_QUERY_TXPM_SSEL,
    /// get if BBTX power meter update disabled
    DFE_FL_BB_QUERY_DIS_TXPM_UPDATE,
    /// get status of BBTX power meter
    DFE_FL_BB_QUERY_TXPM_INTR_STATUS,
    /// get result of BBTX power meter
    DFE_FL_BB_QUERY_TXPM_RESULT,
    
    /** RX power meter queries
     */    
    /// get BBRX power meter config
    DFE_FL_BB_QUERY_RXPM_CFG,
    /// get BBRX power meter sync select
    DFE_FL_BB_QUERY_RXPM_SSEL,
    /// get if BBRX power meter update disabled
    DFE_FL_BB_QUERY_DIS_RXPM_UPDATE,
    /// get status of BBRX power meter
    DFE_FL_BB_QUERY_RXPM_INTR_STATUS,
    /// get result of BBRX power meter
    DFE_FL_BB_QUERY_RXPM_RESULT,
    
    /** Antenna Calibration queries
     */
    /// get global config of antenna calibration    
    DFE_FL_BB_QUERY_ANTCAL_GLOBAL_CFG,
    /// get config of antenna calibration    
    DFE_FL_BB_QUERY_ANTCAL_CFG,
    /// get result of antenna calibration    
    DFE_FL_BB_QUERY_ANTCAL_RESULT,
    
    /// RX GAIN upadte status
    DFE_FL_BB_QUERY_RXGAIN_UPDATE_STATUS,    
    /// config sync selection for rx gain update
    DFE_FL_BB_QUERY_RXGAIN_SSEL,
    /// query rx gain update interrupt status
    DFE_FL_BB_QUERY_RXGAIN_INTR_STATUS,
    /// config beAGC globals
    DFE_FL_BB_QUERY_BEAGC_GLOBAL_CFG, 
    /// config a beAGC control loop
    DFE_FL_BB_QUERY_BEAGC_CFG,
    /// config a beAGC control loop ssel
    DFE_FL_BB_QUERY_BEAGC_SSEL,
    
    /** Rx Notch filter
     */
    /// config Rx Notch filter globals
    DFE_FL_BB_QUERY_RXNOTCH_GLOBAL_CFG,
    /// Rx Notch ssel
    DFE_FL_BB_QUERY_RXNOTCH_SSEL,
    /// config a Rx Notch filter
    DFE_FL_BB_QUERY_RXNOTCH_CFG,

    /// get status of BB general interrupt
    DFE_FL_BB_QUERY_GENERAL_INTR_STATUS,
    /// get status of group of BB general interrupts
    DFE_FL_BB_QUERY_GENERAL_INTRGRP_STATUS,

    DFE_FL_BB_QUERY_MAX_VALUE
} DfeFl_BbHwStatusQuery;

/** @brief carrier type */
typedef enum
{
    /// carrier type 0
    DFE_FL_BB_CARRIER_TYPE_0 = 0,
    /// carrier type 1
    DFE_FL_BB_CARRIER_TYPE_1,
    /// carrier type 2
    DFE_FL_BB_CARRIER_TYPE_2,
    /// carrier type 3
    DFE_FL_BB_CARRIER_TYPE_3,
    /// carrier type 4
    DFE_FL_BB_CARRIER_TYPE_4,
    /// carrier type 5
    DFE_FL_BB_CARRIER_TYPE_5,
    /// carrier type 6
    DFE_FL_BB_CARRIER_TYPE_6,
    /// carrier type 7
    DFE_FL_BB_CARRIER_TYPE_7,
    /// carrier type 8
    DFE_FL_BB_CARRIER_TYPE_8,
    /// carrier type 9
    DFE_FL_BB_CARRIER_TYPE_9,
    /// carrier type 10
    DFE_FL_BB_CARRIER_TYPE_10,
    /// carrier type 11
    DFE_FL_BB_CARRIER_TYPE_11,
    /// carrier type 12
    DFE_FL_BB_CARRIER_TYPE_12,
    /// carrier type 13
    DFE_FL_BB_CARRIER_TYPE_13,
    /// carrier type 14
    DFE_FL_BB_CARRIER_TYPE_14,
    /// carrier type 15
    DFE_FL_BB_CARRIER_TYPE_15,
    
    /// carrier type ALL
    DFE_FL_BB_CARRIER_TYPE_ALL = 0xffff
} DfeFl_BbCarrierType;

/** @brief test_cb_control selection */
typedef enum
{
    /// BB testbus probe, disabled (normal)
    DFE_FL_BB_TEST_CB_CTRL_DISABLE     = 0x00,
    /// BB testbus probe, BBTX AID AxC single
    DFE_FL_BB_TEST_CB_CTRL_TX_AID_S    = 0x01,
    /// BB testbus probe, BBTX AID AxC all
    DFE_FL_BB_TEST_CB_CTRL_TX_AID_A    = 0x03,
    /// BB testbus probe, BBTX buffer memory single
    DFE_FL_BB_TEST_CB_CTRL_TX_BUFMEM_S = 0x05,
    /// BB testbus probe, BBTX buffer memory all
    DFE_FL_BB_TEST_CB_CTRL_TX_BUFMEM_A = 0x07,
    /// BB testbus probe, BBTX DDUC interface single
    DFE_FL_BB_TEST_CB_CTRL_TX_DDUCIF_S = 0x09,
    /// BB testbus probe, BBTX DDUC interface all
    DFE_FL_BB_TEST_CB_CTRL_TX_DDUCIF_A = 0x0B,
    /// BB testbus probe, BB JESDTX AxC single
    DFE_FL_BB_TEST_CB_CTRL_JTX_AID_S   = 0x0D,
    /// BB testbus probe, BB JESDTX AxC all
    DFE_FL_BB_TEST_CB_CTRL_JTX_AID_A   = 0x0F,
    
    /// BB testbus probe, BBRX AID AxC single
    DFE_FL_BB_TEST_CB_CTRL_RX_AID_S    = 0x21,
    /// BB testbus probe, BBRX AID AxC all
    DFE_FL_BB_TEST_CB_CTRL_RX_AID_A    = 0x23,
    /// BB testbus probe, BBRX buffer memory single
    DFE_FL_BB_TEST_CB_CTRL_RX_BUFMEM_S = 0x25,
    /// BB testbus probe, BBRX buffer memory all
    DFE_FL_BB_TEST_CB_CTRL_RX_BUFMEM_A = 0x27,
    /// BB testbus probe, BBRX DDUC interface single
    DFE_FL_BB_TEST_CB_CTRL_RX_DDUCIF_S = 0x29,
    /// BB testbus probe, BBRX DDUC interface all
    DFE_FL_BB_TEST_CB_CTRL_RX_DDUCIF_A = 0x2B,
    /// BB testbus probe, BB JESDRX AID AxC single
    DFE_FL_BB_TEST_CB_CTRL_JRX_AID_S   = 0x2D,
    /// BB testbus probe, BB JESDRX AID AxC all
    DFE_FL_BB_TEST_CB_CTRL_JRX_AID_A   = 0x2F
    
} DfeFl_BbTestCbCtrl;

/** @brief BB general interrupt */
typedef enum
{
    /// BB general interrupt, TXPM_LDERR
    DFE_FL_BB_GENERAL_INTR_TXPM_LDERR      = 0,
    /// BB general interrupt, RXPM_LDERR
    DFE_FL_BB_GENERAL_INTR_RXPM_LDERR      = 1,
    /// BB general interrupt, ANTCAL
    DFE_FL_BB_GENERAL_INTR_ANTCAL          = 2,
    /// BB general interrupt, RXNOTCH_DONE
    DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE    = 3,
    /// BB general interrupt, RXNOTCH_ERR
    DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR     = 4,

    /// BB general interrupt, BUFMEM0_OUF (overflow/underflow)
    DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF     = 8,
    /// BB general interrupt, BUFMEM1_OUF (overflow/underflow)
    DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF     = 9,
    /// BB general interrupt, BUFMEM2_OUF (overflow/underflow)
    DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF     = 10,
    /// BB general interrupt, BUFMEM3_OUF (overflow/underflow)
    DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF     = 11,
    /// BB general interrupt, BUFMEM4_OUF (overflow/underflow)
    DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF     = 12,
    /// BB general interrupt, BUFMEM5_OUF (overflow/underflow)
    DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF     = 13,
    /// BB general interrupt, BUFMEM6_OUF (overflow/underflow)
    DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF     = 14,
    /// BB general interrupt, BUFMEM7_OUF (overflow/underflow)
    DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF     = 15,
    /// BB general interrupt, RXAID_SYNCERR
    DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR   = 16,
    /// BB general interrupt, TXAID_UDF (under flow)
    DFE_FL_BB_GENERAL_INTR_TXAID_UDF       = 17,
    /// BB general interrupt, TXAID_OVF (over flow)
    DFE_FL_BB_GENERAL_INTR_TXAID_OVF       = 18,
    /// BB general interrupt, JESDRX_SYNCERR
    DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR  = 19,
    /// BB general interrupt, JESDTX_UDF (under flow)
    DFE_FL_BB_GENERAL_INTR_JESDTX_UDF      = 20,
    /// BB general interrupt, JESDTX_OVF (over flow)
    DFE_FL_BB_GENERAL_INTR_JESDTX_OVF      = 21
    
} DfeFl_BbGeneralIntr;

/** @brief BB Test Signal Generation Device
 */
typedef enum
{
    /// TESTGEN for DDUC0 buffer
    DFE_FL_BB_DDUC_TESTGEN_0 = 0,
    /// TESTGEN for DDUC1 buffer
    DFE_FL_BB_DDUC_TESTGEN_1,
    /// TESTGEN for DDUC2 buffer
    DFE_FL_BB_DDUC_TESTGEN_2,
    /// TESTGEN for DDUC3 buffer
    DFE_FL_BB_DDUC_TESTGEN_3,
    /// TESTGEN for DDUC4 buffer
    DFE_FL_BB_DDUC_TESTGEN_4,
    /// TESTGEN for DDUC5 buffer
    DFE_FL_BB_DDUC_TESTGEN_5,
    /// TESTGEN for DDUC6 buffer
    DFE_FL_BB_DDUC_TESTGEN_6,
    /// TESTGEN for DDUC7 buffer
    DFE_FL_BB_DDUC_TESTGEN_7,

    /// TESTGEN for AID A buffer
    DFE_FL_BB_AID_TESTGEN_A = 8,
    /// TESTGEN for AID B buffer
    DFE_FL_BB_AID_TESTGEN_B = 9,
    
    DFE_FL_BB_MAX_TESTGENS = 10
} DfeFl_BbTestGenDev;

/** @brief BB Test Signal Generation ramp mode
 */
typedef enum
{
    /// LFSR
    DFE_FL_BB_TESTGEN_RAMP_MODE_LFSR = 0,
    /// RAMP
    DFE_FL_BB_TESTGEN_RAMP_MODE_RAMP = 1  
} DfeFl_BbTestGenRampMode;

/** @brief BB checksum device
 */
typedef enum
{
    /// CHKSUM for DDUC0 buffer
    DFE_FL_BB_DDUC_CHKSUM_0 = 0,
    /// CHKSUM for DDUC1 buffer
    DFE_FL_BB_DDUC_CHKSUM_1,
    /// CHKSUM for DDUC2 buffer
    DFE_FL_BB_DDUC_CHKSUM_2,
    /// CHKSUM for DDUC3 buffer
    DFE_FL_BB_DDUC_CHKSUM_3,
    /// CHKSUM for DDUC4 buffer
    DFE_FL_BB_DDUC_CHKSUM_4,
    /// CHKSUM for DDUC5 buffer
    DFE_FL_BB_DDUC_CHKSUM_5,
    /// CHKSUM for DDUC6 buffer
    DFE_FL_BB_DDUC_CHKSUM_6,
    /// CHKSUM for DDUC7 buffer
    DFE_FL_BB_DDUC_CHKSUM_7,

    /// CHKSUM for AID A buffer
    DFE_FL_BB_AID_CHKSUM_A = 8,
    /// CHKSUM for AID B buffer
    DFE_FL_BB_AID_CHKSUM_B = 9,

    DFE_FL_BB_MAX_CHKSUMS = 10
} DfeFl_BbChksumDev;

/** @brief BB checksum return mode */
typedef enum
{
    /// BB checksum return checksum value
    DFE_FL_BB_CHKSUM_MODE_RETURN_CHKSUM = 0,
    /// BB checksum return latency value
    DFE_FL_BB_CHKSUM_MODE_RETURN_LATENCY
} DfeFl_BbChksumMode;

/** @brief data mode for Tx/Rx TDD
 */
typedef enum
{
    /// TX: UL data passthru unchanged
    DFE_FL_BB_TXTDD_DATAMODE_PASSTHRU = 0,
    /// TX: UL data is zeroed and buffer memory stalled
    DFE_FL_BB_TXTDD_DATAMODE_ZEROED = 1,

    /// RX: DL data is zeroed at notch filter input
    DFE_FL_BB_RXTDD_DATAMODE_ZEROED_AT_NOTCHFILETR_INPUT = 0,
    /// RX: DL data is zeroed at formatter
    DFE_FL_BB_RXTDD_DATAMODE_ZEROED_AT_FORMATTER = 1,
    /// RX: DL data is zeroed at input of BB and buffer memory stalled
    DFE_FL_BB_RXTDD_DATAMODE_ZEROED_AT_BUFFER_INPUT = 2
} DfeFl_BbTddDataMode;

/** @brief BB power meter enable mode
 */
typedef enum
{
    /// power meter is off
    DFE_FL_BB_POWMETR_OFF = 0,
    /// run one interval and then stop per sync
    DFE_FL_BB_POWMETR_SINGLE_POWER_MEASUREMENT,
    /// run multi intervals and then stop per sync
    DFE_FL_BB_POWMETR_SINGLE_POWER_UPDATE_INTERVAL,
    /// run continuouslly, restart per sync
    DFE_FL_BB_POWMETR_CONTINUOUS_POWER_MESURE
} DfeFl_BbPowMtrEnable;
/** @brief BB power meter result or output format
 */
typedef enum
{
    /// float format, 10.16e6
    DFE_FL_BB_POWMTR_OUTFMT_FLOAT_10P16E6 = 0,
    /// starting from 0, in 0.1dB unit step
    DFE_FL_BB_POWMTR_OUTFMT_STEP_0P1DB = 2
} DfeFl_BbPowMtrOutFormat;
/** @brief BB power meter input source
 */
typedef enum
{
    /// at BB input
    ///  tx: before tx gain
    ///  rx: before notch filter
    DFE_FL_BB_POWMTR_INSRC_INPUT = 0,
    /// at BB output
    ///  tx: after circular clipper
    ///  rx: after beAGC
    DFE_FL_BB_POWMTR_INSRC_OUTPUT,
    /// at tx gain output
    DFE_FL_BB_POWMTR_INSRC_TX_GAIN_OUTPUT = 2,
    /// at rx notch filter output
    DFE_FL_BB_POWMTR_INSRC_RX_FILTER_OUTPUT = 2
} DfeFl_BbPowMtrInSource;
/** @brief BB power meter tdd mode
 */
typedef enum
{
    /// tdd mode disabled
    DFE_FL_BB_POWMTR_TDDMODE_DISABLED = 0,
    /// txpm: halt when UL period
    DFE_FL_BB_POWMTR_TDDMODE_TX_HALT_UL = 1,
    /// rxpm: halt when DL period
    DFE_FL_BB_POWMTR_TDDMODE_RX_HALT_DL = 1,
    /// txpm: reset when UL period
    DFE_FL_BB_POWMTR_TDDMODE_TX_RESET_UL = 2,
    /// rxpm: reset when DL period
    DFE_FL_BB_POWMTR_TDDMODE_RX_RESET_DL = 2
} DfeFl_BbPowMtrTddMode;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_BB_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_LOOPBACK
 *      DFE_FL_BB_QUERY_LOOPBACK_CFG
 */
typedef struct
{
    /// BB buf loopback, dduc0 to dduc1
    uint32_t duc0ToDdc1;
    /// BB buf loopback, dduc1 to dduc2
    uint32_t duc1ToDdc2;
    /// BB buf loopback, dduc0 to dduc3
    uint32_t duc0ToDdc3;
    /// BB buf loopback, dduc3 to dduc4
    uint32_t duc3ToDdc4;
    /// BB buf loopback, dduc2 to dduc5
    uint32_t duc2ToDdc5;
    /// BB buf loopback, dduc1 to dduc6
    uint32_t duc1ToDdc6;
    /// BB buf loopback, dduc0 to dduc7
    uint32_t duc0ToDdc7;
} DfeFl_BbLoopbackConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_CAPBUFF
 *      DFE_FL_BB_QUERY_CAPBUFF_CFG
 */
typedef struct
{
#ifdef _BIG_ENDIAN
/// BIG ENDIAN format    
    /// rsvd1
    uint32_t rsvd1      : 16;
    /// axc# or buf# for single mode
    uint32_t testCbAxc  : 8;
    /// rsvd0        
    uint32_t rsvd0      : 2;
    /// testbus probe
    uint32_t testCbCtrl : 6;
#else
/// LITTLE ENDIAN format
    /// testbus probe
    uint32_t testCbCtrl : 6;
    /// rsvd0        
    uint32_t rsvd0      : 2;
    /// axc# or buf# for single mode
    uint32_t testCbAxc  : 8;
    /// rsvd1
    uint32_t rsvd1      : 16;
#endif
} DfeFl_BbCapBuffConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_TESTGEN
 *      DFE_FL_BB_QUERY_TESTGEN_CFG
 */
typedef struct
{
    /// test gen device
    DfeFl_BbTestGenDev tgDev;
    /// only valid for AID
    uint32_t testEnable;
    /// enable data generation
    uint32_t genData;
    /// enbale frame generation
    uint32_t genFrame;
    /// ramp (1), or LFSR (0)
    DfeFl_BbTestGenRampMode rampMode;
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
} DfeFl_BbTestGenConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_SET_TESTGEN_SSEL
 *      DFE_FL_BB_QUERY_TESTGEN_SSEL
 */
typedef struct
{
    /// test gen device
    DfeFl_BbTestGenDev tgDev;
    /// sync select
    uint32_t ssel;
} DfeFl_BbTestGenSsel;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_CHKSUM
 */
typedef struct
{
    /// checksum device
    uint32_t chksumDev;
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
} DfeFl_BbChksumConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_SET_CHKSUM_SSEL
 */
typedef struct
{
    /// checksum device
    uint32_t chksumDev;
    /// sync selection
    uint32_t ssel;
} DfeFl_BbChksumSsel;

/** @brief argument for runtime control,
 *      DFE_FL_BB_QUERY_CHKSUM_RESULT
 */
typedef struct
{
    /// checksum device
    uint32_t chksumDev;
    /// result
    uint32_t result;
} DfeFl_BbChksumResult;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_CT_UL_SYNC_STROBE
 *      DFE_FL_BB_QUERY_CT_UL_SYNC_STROBE
 */
typedef struct
{
    /// carrier type
    uint32_t ct;
    /// UL sync strobe
    uint32_t strobe;
} DfeFl_BbCarrierTypeUlSyncStrobeConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_AID_ULSTROBE_DLY
 *      DFE_FL_BB_QUERY_AID_ULSTROBE_DLY
 */
typedef struct
{
    /// carrier type
    uint32_t ct;
    /// UL strobe delay
    uint32_t dly;
} DfeFl_BbAidUlStrobeDelayConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_TXIF_AXC
 */
typedef struct
{
    /// Per antenna carrier index into buffer memory the carrier is assigned to
    uint32_t bufferIndex;
    /// Per antenna carrier buffer the carrier is assigned to
    uint32_t bufferNum;
    /// Per antenna carrier selection of 1 of 16 power meter configurations the carrier is assigned to.
    uint32_t pmConfigSel;
    /// Per antenna carrier enable of power meter function
    uint32_t pmEn;
    /// Per antenna carrier enable of the circular clipper function
    uint32_t clEn;
    /// Per antenna carrier enable of the gain function (otherwise unity gain)
    uint32_t gainEn;
    /// Per antenna carrier enable.  When disabled carrier is ignored
    uint32_t axcValid;
    /// Per antenna carrier 1/T value to be used when circular clipper is enabled.
    uint32_t cl1OverT;
    /// Per antenna carrier antenna calibration select.
    uint32_t antcalSel;
    /// Per antenna carrier antenna calibration enable
    uint32_t antcalEn;
    /// Per antenna carrier enable of autoCP mode.
    uint32_t autocpEn;
    /// Per antenna carrier selection of autoCP timer (one of two choices).
    uint32_t autocpSel;
} DfeFl_BbTxifAxc;
typedef struct
{
    /// total AxCs for array of txifAxc[]
    uint32_t numAxCs;
    struct {
        /// axc id#
        uint32_t  axc;        
        /// per antenna carrier config
        DfeFl_BbTxifAxc txif;
    } txifAxc[DFE_FL_BB_ANTENNA_MAX_AXCS];
} DfeFl_BbTxifAxcConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_RXIF_AXC
 */
typedef struct
{
    /// Per antenna carrier index into buffer memory the carrier is assigned to
    uint32_t bufferIndex;
    /// Per antenna carrier buffer the carrier is assigned to
    uint32_t bufferNum;
    /// Per antenna carrier assignment of the carrier type.
    uint32_t carrierType;
    /// Per antenna carrier selection of AGC mode. 
    uint32_t beagcMode;
    /// Per antenna carrier enable.  When disabled carrier is ignored even if there is a slot assigned to it.
    uint32_t axcValid;
    /// Per antenna carrier selection of 1 of 16 power meter configurations the carrier is assigned to.
    uint32_t pmConfigSel;
    /// Per antenna carrier enable of power meter function
    uint32_t pmEn;
    /// Per antenna carrier selection of input notch filter configuration
    uint32_t notchEn;
    /// Per antenna carrier selection of 16 bit output packing (instead of 32 bit format)
    uint32_t outPacked;
    /// Per antenna carrier selection of number of floating point bits when in float mode (set by fixedorfloat)
    uint32_t outFloatMode;
    /// Per antenna carrier selection of floating point mode for output
    uint32_t fixedOrFloat;
    /// Per antenna carrier selection of number of mantissa bits +1 the output will be rounded to.    
    uint32_t outNumBits;
    /// Selects which of 16 antenna calibration configurations to use for the carrier
    uint32_t antcalSel;
    /// Per antenna carrier enable of antenna calibration noise enable
    uint32_t antcalEn;
    /// Per antenna carrier power backoff value used when in power managed gain control mode
    uint32_t beagcPowerBackoff;

    /// Per antenna carrier force output to zero when tdd is in DL
    uint32_t tdd0;
    /// Per antenna carrier number of t3 intervals to run gain loop.  0=forever
    uint32_t beagcT3ActvCnt;
    /// Per antenna carrier selection of 1 of 8 beagc loop configurations to be used when in beagc mode
    uint32_t beagcConfigSel;
    /// Per antenna carrier t1 interval when in beagc closed loop gain mode
    uint32_t beagcT1Interval;    
    /// Per antenna carrier t2 interval when in beagc closed loop gain mode
    uint32_t beagcT2Interval;
} DfeFl_BbRxifAxc;
typedef struct
{
    /// total AxCs for array of rxifAxc[]
    uint32_t  numAxCs;
    struct {
        /// axc id#
        uint32_t  axc;
        /// per antenna carrier config
        DfeFl_BbRxifAxc rxif;
    } rxifAxc[DFE_FL_BB_ANTENNA_MAX_AXCS];
} DfeFl_BbRxifAxcConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_UPD_TXIF_AXC_GAIN
 */
typedef struct
{
    /// number of AxCs whose gains need updating
    uint32_t  numAxCs;
    /// axc gain update table
    struct
    {
        /// axc id#
        uint32_t  axc;
        /// real part gain word for the axc
        uint32_t  gainI;
        /// image part gain word for the axc
        uint32_t  gainQ;
    } axcGain[DFE_FL_BB_ANTENNA_MAX_AXCS];
} DfeFl_BbTxGainConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_QUERY_TXGAIN_INTR_STATUS
 *      DFE_FL_BB_QUERY_RXGAIN_INTR_STATUS
 *      DFE_FL_BB_QUERY_TXGAIN_UPDATE_STATUS
 *      DFE_FL_BB_QUERY_RXGAIN_UPDATE_STATUS
 *      DFE_FL_BB_CMD_SET_RXGAIN_SSEL
 *      DFE_FL_BB_CMD_SET_TXGAIN_SSEL
 *      DFE_FL_BB_QUERY_TXGAIN_SSEL
 *      DFE_FL_BB_QUERY_RXGAIN_SSEL
 */
typedef struct
{
    /// carrier type
    uint32_t  ct;
    /// set/get value
    uint32_t  data;
} DfeFl_BbTxRxGainCarrierTypeData;


/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_TXTDD
 *      DFE_FL_BB_CMD_CFG_RXTDD
 *      DFE_FL_BB_QUERY_TXTDD_CFG
 *      DFE_FL_BB_QUERY_RXTDD_CFG
 */
typedef struct
{
    /// enable
    uint32_t enable;
    /// data mode
    DfeFl_BbTddDataMode dataMode;
    /// carrier type
    uint32_t carrierType;
    /// delay from sync
    uint32_t syncDly;
    /// DL1 interval
    uint32_t dl1Interval;
    /// UL1 interval
    uint32_t ul1Interval;
    /// DL2 interval
    uint32_t dl2Interval;
    /// UL2 interval
    uint32_t ul2Interval;
    /// DL3 interval
    uint32_t dl3Interval;
    /// UL3 interval
    uint32_t ul3Interval;        
} DfeFl_BbTddConfig;


/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_TXPM
 *      DFE_FL_BB_CMD_CFG_RXPM
 *      DFE_FL_BB_QUERY_TXPM_CFG
 *      DFE_FL_BB_QUERY_RXPM_CFG
 */
typedef struct
{
    /// power meter Id
    uint32_t pmId;
    /// enable power meter function
    DfeFl_BbPowMtrEnable enable;
    /// result output format
    DfeFl_BbPowMtrOutFormat outFormat;
    /// carrier type
    uint32_t countSource;
    /// power meter input source
    DfeFl_BbPowMtrInSource inSource;
    /// tdd mode
    DfeFl_BbPowMtrTddMode tddMode;
    /// delay from sync
    uint32_t syncDly;
    /// meter interval
    uint32_t interval;
    /// integration period
    uint32_t intgPd;
    /// count of measurements, i.e. count of intervals
    uint32_t pwrUpdate;
    /// for RXPM only, max dB value assuming full power of power interval
    uint32_t maxDb;
} DfeFl_BbPowerMeterConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_SET_TXPM_SSEL
 *      DFE_FL_BB_CMD_SET_RXPM_SSEL
 *      DFE_FL_BB_QUERY_TXPM_SSEL
 *      DFE_FL_BB_QUERY_RXPM_SSEL
 */
typedef struct
{
    /// power meter Id
    uint32_t pmId;
    /// sync selection
    uint32_t ssel;
} DfeFl_BbPowerMeterSsel;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_DIS_TXPM_UPDATE
 *      DFE_FL_BB_CMD_DIS_RXPM_UPDATE
 *      DFE_FL_BB_QUERY_DIS_TXPM_UPDATE
 *      DFE_FL_BB_QUERY_DIS_RXPM_UPDATE
 */
typedef struct
{
    /// power meter Id
    uint32_t pmId;
    /// disable update
    uint32_t disableUpdate;
} DfeFl_BbDisablePowMterUpdateConfig;


/** @brief argument for runtime control,
 *      DFE_FL_BB_QUERY_TXPM_INTR_STATUS
 *      DFE_FL_BB_QUERY_RXPM_INTR_STATUS
 */
typedef struct 
{
    /// power meter Id
    uint32_t pmId;
    /// complete status
    uint32_t status;
} DfeFl_BbPowMtrIntrStatus;

/** @brief argument for runtime control,
 *      DFE_FL_BB_QUERY_TXPM_RESULT
 *      DFE_FL_BB_QUERY_RXPM_RESULT
 */
typedef struct
{
    /// power meter Id
    uint32_t pmId;
    /// peak power main value
    uint32_t peakPower;
    /// peak power extended value
    uint32_t peakPower_extend;
    /// RMS power main value
    uint32_t rmsPower;
    /// RMS power extended value
    uint32_t rmsPower_extend;
} DfeFl_BbPowMtrResult;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_ANTCAL_GLOBAL
 *      DFE_FL_BB_QUERY_ANTCAL_GLOBAL_CFG
 */
typedef struct
{
    /// tx carrier type selection
    uint32_t txCarrierTypeSel;
    /// rx carrier type selection
    uint32_t rxCarrierTypeSel;
    /// tx sync selection
    uint32_t txSsel;
    /// rx sync selection
    uint32_t rxSsel;
    /// enable antenna calibartion
    uint32_t enable;
    /// number of samples to collect noise correlation values
    uint32_t interval;
} DfeFl_BbAntCalGlobalConfig;   

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_ANTCAL
 *      DFE_FL_BB_QUERY_ANTCAL_CFG
 */
typedef struct
{
    /// antenna calibration device
    uint32_t antcal;
    /// Antenna Calibration PN sequencer Initial value
    uint32_t pnInit;
    /// Antenna Calibration PN sequencer tap configuration
    uint32_t pnTapConfig;
    /// Antenna Calibration TX noise level
    uint32_t txNoise;
    /// Antenna Calibration RX correlation delay in samples
    uint32_t rxCorrDelay;
    /// Antenna Calibration RX oversampled.  When 1 AxC is 2x oversampled
    uint32_t rxOverSample;
} DfeFl_BbAntCalConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_QUERY_GENERAL_INTR_STATUS
 */
typedef struct
{
    /// general interrupt
    uint32_t intr;
    /// result
    uint32_t result;
} DfeFl_BbGeneralIntrQuery;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_ENB_GENERAL_INTRGRP
 *      DFE_FL_BB_CMD_DIS_GENERAL_INTRGRP
 *      DFE_FL_BB_CMD_CLR_GENERAL_INTRGRP_STATUS
 *      DFE_FL_BB_CMD_SET_FORCE_GENERAL_INTRGRP
 *      DFE_FL_BB_CMD_CLR_FORCE_GENERAL_INTRGRP
 *      DFE_FL_BB_QUERY_GENERAL_INTRGRP_STATUS
 */
typedef struct
{    
    /// BB general interrupt, TXPM_LDERR
    uint32_t txpmLoadErr;
    /// BB general interrupt, RXPM_LDERR
    uint32_t rxpmLoadErr;
    /// BB general interrupt, ANTCAL
    uint32_t antcal;
    /// BB general interrupt, RXNOTCH_DONE
    uint32_t rxNotchDone;
    /// BB general interrupt, RXNOTCH_ERR
    uint32_t rxNotchErr;

    /// BB general interrupt, BUFMEM_OUF (overflow/underflow)
    uint32_t bufErr[8];
    /// BB general interrupt, RXAID_SYNCERR
    uint32_t rxaidSyncErr;
    /// BB general interrupt, TXAID_UDF (under flow)
    uint32_t txaidUnderflow;
    /// BB general interrupt, TXAID_OVF (over flow)
    uint32_t txaidOverflow;
    /// BB general interrupt, JESDRX_SYNCERR
    uint32_t jesdrxSyncErr;
    /// BB general interrupt, JESDTX_UDF (under flow)
    uint32_t jesdtxUnderflow;
    /// BB general interrupt, JESDTX_OVF (over flow)
    uint32_t jesdtxOverflow;
} DfeFl_BbGeneralIntrGroup;


/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_UPD_RXGAIN
 */
typedef struct
{
    /// number of AxCs whose gains need updating
    uint32_t  numAxCs;
    /// axc gain update table
    struct
    {
        /// axc id#
        uint32_t  axc;
        /// integer part gain word for the axc
        uint32_t  gainInteger;
        /// fractional part gain word for the axc
        uint32_t  gainFraction;
    } axcGain[DFE_FL_BB_ANTENNA_MAX_AXCS];
} DfeFl_BbRxGainConfig;


/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_BEAGC_GLOBAL
 *      DFE_FL_BB_QUERY_BEAGC_GLOBAL_CFG
 */
typedef struct
{
    /// When set beagc gain loop emphasizes saturation by incrementing sat counter by 1 when I or Q is sat
    uint32_t loop_config_sat;
    /// TDD timer configuration for beAGC.  0:tdd halt on DL, 1: tdd reset on DL
    uint32_t tdd_config;
} DfeFl_BbBeagcGlobalConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_BEAGC
 *      DFE_FL_BB_QUERY_BEAGC_CFG
 */
typedef struct
{
    /// beagc device id
    uint32_t beagc;
    /// Select which buffer sync is the source of the interval counter for configuration 0 in closed loop mode.
    uint32_t intervalSource;
    /// master t3 interval
    uint32_t t3Interval;
    /// enable/disbale tdd mode
    uint32_t tdd_enable;
    /// beagc control loop configuration threshold value of AGC unsigned
    uint32_t thresh;
    /// beagc control loop configuration zero_mask masks lower 4 bits for zero count, a 0 will mask off zero calculation
    uint32_t zeroMask;
    /// beagc control loop configuration  zero count threshold
    uint32_t zeroCountThresh;
    /// beagc control loop configuration saturation count threshold;
    uint32_t satCountThresh;
    /// beagc control loop configuration shift value for below threshold.  0=shift of 2 ... 15=shift of 17
    uint32_t dBelow;
    /// beagc control loop configuration shift value for above threshold.  0=shift of 2 ... 15=shift of 17
    uint32_t dAbove;
    /// beagc control loop configuration shift value for saturation case.  0=shift of 2 ... 15=shift of 17
    uint32_t dSat;
    /// beagc control loop configuration shift value for zero case.  0=shift of 2 ... 15=shift of 17
    uint32_t dZero;
    /// beagc control loop configuration maximum allowed gain adjustment value.  Adjustment stops at g(k)=G + amax Amax format is signed (1,16,7)
    uint32_t amax;
    /// beagc control loop configuration minimum allowed gain adjustment value.   Adjustment stops at g(k)=G +  amin. Amin format is signed (1,16,7)
    uint32_t amin;
} DfeFl_BbBeagcConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_SET_BEAGC_SSEL
 *      DFE_FL_BB_QUERY_BEAGC_SSEL
 */
typedef struct
{
    /// beagc device id
    uint32_t beagc;
    /// sync selection
    uint32_t ssel;
} DfeFl_BbBeagcSsel;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_RXNOTCH_GLOBAL
 *      DFE_FL_BB_QUERY_RXNOTCH_GLOBAL_CFG
 */
typedef struct
{
    /// carrier type
    uint32_t carrierType;
    /// tdd mode
    uint32_t tddMode;    
} DfeFl_BbRxNotchGlobalConfig;

/** @brief argument for runtime control,
 *      DFE_FL_BB_CMD_CFG_RXNOTCH
 *      DFE_FL_BB_QUERY_RXNOTCH_CFG
 */
typedef struct
{
    /// axc Id
    uint32_t axc;
    /// filter mode
    uint32_t mode;
    /// filter1 tap select
    uint32_t filter1;
    /// filter2 tap select
    uint32_t filter2;
    /// filter3 tap select
    uint32_t filter3;
    /// filter4 tap select
    uint32_t filter4;
    /// Notch filter 0
    uint32_t tap0I;
    uint32_t tap0Q;
    uint32_t tap0Width;
    /// Notch filter 1
    uint32_t tap1I;
    uint32_t tap1Q;
    uint32_t tap1Width;
} DfeFl_BbRxNotch;

/** @brief AID DL translate
 */
typedef struct
{
#ifdef _BIG_ENDIAN
    uint32_t rsvd0        : 22;
    uint32_t strobeType   : 3;
    uint32_t axc          : 7;        
#else    
    uint32_t axc          : 7;        
    uint32_t strobeType   : 3;
    uint32_t rsvd0        : 22;
#endif        
} DfeFl_BbAidDlXlate;

/** @brief AID UL translate
 */
typedef struct
{
#ifdef _BIG_ENDIAN
    uint32_t rsvd1        : 20;
    uint32_t carrierType  : 4;
    uint32_t rsvd0        : 2;
    uint32_t axc          : 6;        
#else    
    uint32_t axc          : 6;        
    uint32_t rsvd0        : 2;
    uint32_t carrierType  : 4;
    uint32_t rsvd1        : 20;
#endif        
} DfeFl_BbAidUlXlate;

/** @brief TXIF slot map
 */
typedef struct
{
#ifdef _BIG_ENDIAN
    uint32_t rsvd0        : 20;    
    uint32_t carrierType  : 4;
    uint32_t unassigned   : 1;
    uint32_t axc          : 7;
#else
    uint32_t axc          : 7;
    uint32_t unassigned   : 1;
    uint32_t carrierType  : 4;
    uint32_t rsvd0        : 20;                
#endif        
} DfeFl_BbTxifSlot;

/** @brief RXIF slot map
 */
typedef struct
{
#ifdef _BIG_ENDIAN
    uint32_t rsvd0        : 25;    
    uint32_t unassigned   : 1;
    uint32_t axc          : 6;
#else
    uint32_t axc          : 6;
    uint32_t unassigned   : 1;
    uint32_t rsvd0        : 25;                
#endif        
} DfeFl_BbRxifSlot;

/** @brief overlay register pointer to BB instance
 */
typedef CSL_DFE_BB_REGS *DfeFl_BbRegsOvly;

/** @brief a BaseBand (BB) Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle       hDfe;
    
    /// pointer to register base address of a BB instance
    DfeFl_BbRegsOvly   regs;
   
    /// This is the instance of BB being referred to by this object
    DfeFl_InstNum      perNum;

} DfeFl_BbObj;

/** @brief handle pointer to BB object
 */
typedef DfeFl_BbObj *DfeFl_BbHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_BB_FUNCTION
 * @{
 */

DfeFl_BbHandle dfeFl_BbOpen
(
    DfeFl_Handle                hDfe,
    DfeFl_BbObj                *pDfeBbObj,
    DfeFl_InstNum               bbNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_BbClose(DfeFl_BbHandle hDfeBb);

DfeFl_Status  dfeFl_BbHwControl
(
    DfeFl_BbHandle             hDfeBb,
    DfeFl_BbHwControlCmd       ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_BbGetHwStatus
(
    DfeFl_BbHandle             hDfeBb,
    DfeFl_BbHwStatusQuery      queryId,
    void                        *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_BB_H_ */
