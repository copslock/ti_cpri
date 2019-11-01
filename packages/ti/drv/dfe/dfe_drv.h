/********************************************************************
 * Copyright (C) 2015 Texas Instruments Incorporated.
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
#ifndef __DFE_DRV_H__
#define __DFE_DRV_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/drv/dfe/dfe_fl_bb.h>
#include <ti/drv/dfe/dfe_fl_dduc.h>
#include <ti/drv/dfe/dfe_fl_summer.h>
#include <ti/drv/dfe/dfe_fl_autocp.h>
#include <ti/drv/dfe/dfe_fl_cfr.h>
#include <ti/drv/dfe/dfe_fl_cdfr.h>
#include <ti/drv/dfe/dfe_fl_dpd.h>
#include <ti/drv/dfe/dfe_fl_dpda.h>
#include <ti/drv/dfe/dfe_fl_tx.h>
#include <ti/drv/dfe/dfe_fl_jesd.h>
#include <ti/drv/dfe/dfe_fl_rx.h>
#include <ti/drv/dfe/dfe_fl_fb.h>
#include <ti/drv/dfe/dfe_fl_cb.h>
#include <ti/drv/dfe/dfe_fl_misc.h>
#include <ti/drv/dfe/dfe_fl_cpp.h>

/**
 * @defgroup DFE_LLD_API DFE LLD
 */
/**
 * @defgroup DFE_LLD_DATASTRUCT DFE LLD Data Structures
 * @ingroup DFE_LLD_API
 */

/**
 * @defgroup DFE_LLD_SYMBOL DFE LLD Symbols Defined
 * @ingroup DFE_LLD_API
 */

/**
 * @defgroup DFE_LLD_FUNCTION DFE LLD Functions
 * @ingroup DFE_LLD_API
 *
 * To properly configure DFE, the following sequence must be followed:
 * 
 * 
 * | Step | Description                                        | DFE LLD API      |
 * | ---- | -------------------------------------------------- | ---------------- |
 * | 1    | Program DFE PLL to correct operation rate          |                  |
 * | 2    | Power up IQN2 and DFE power domains                |                  |
 * | 3    | Program SERDES to corresponding rate               |                  |
 * | 4    | Open DFE LLD                                       | Dfe_open()       |
 * | 5    | Load DFE target configuration                      | Dfe_loadTgtCfg() |
 * |      | Soft Reset DFE peripheral                          | Dfe_softReset()  |
 * |      | Check and wait SERDES PLL_OK                       |                  |
 * | 6    | IQN2 configuration and enable                      |                  |
 * | 7    | Do DFE initialization sequence for transmit path   | Dfe_initTgtTx()  |
 * | 8    | Program analogue front-end which connects with DFE |                  |
 * |      | Check and wait SERDES OK and !LOSS                 |                  |
 * |      | Do DFE initialization sequence for receive path    | Dfe_initTgtRx()  |
 * | 9    | QMSS, CPPI configuration                           |                  |
 * 
 */
 
/** @brief DFE error status code
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef enum
{
    /// no error
    DFE_ERR_NONE = 0,
    /// mmap fail
    DFE_ERR_HW_MMAP,
    /// CSL Hardware control error
    DFE_ERR_HW_CTRL,
    /// CSL hardware query error
    DFE_ERR_HW_QUERY,
    /// invalid parameters
    DFE_ERR_INVALID_PARAMS,
    /// invalid device access
    DFE_ERR_INVALID_DEVICE,
    /// device already opened, cannot open again
    DFE_ERR_ALREADY_OPENED,
    /// an invalid handle value, such as NULL
    DFE_ERR_INVALID_HANDLE,
    /// callback function is NULL
    DFE_ERR_CALLBACKFXN_IS_NULL,
    /// waiting condition not met when timed out
    DFE_ERR_TIMEDOUT,
    /// sync event not come
    DFE_ERR_SYNC_NOT_COME,
    /// SERDES TX not ready
    DFE_ERR_SERDES_TX_NOT_READY,
    /// SERDES RX not ready
    DFE_ERR_SERDES_RX_NOT_READY,
    /// CPP/DMA channel not valid
    DFE_ERR_CPP_DMA_NOT_VALID,
    /// CPP/DMA channel not available
    DFE_ERR_CPP_DMA_NOT_AVAILABLE,
    /// CPP/Descriptor not available
    DFE_ERR_CPP_DESCRIP_NOT_AVAILABLE,
    
    /// DDUC MIXER NCO can not be updated
    DFE_ERR_DDUC_MIXER_NCO,
    /// CFR is busy, the coeffs can not be updated
    DFE_ERR_CFR_BUSY,
//    // FB pre cb output select is not valid
//    DFE_ERR_FB_PRECB_GAIN,
    DFE_ERR_END
} DFE_Err;

/** @brief Generic CPP/DMA Read/Write mode
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef enum
{
    /// Write single 32-bit words to DFE
    DFE_GENERIC_DMA_RW_MODE_WRITE_SINGLE_WORD = 0,
    /// Write multiple 32-bits words to DFE
    DFE_GENERIC_DMA_RW_MODE_WRITE_MULTI_WORDS,
    /// Read from DFE
    DFE_GENERIC_DMA_RW_MODE_READ
} DFE_GenericDmaReadWriteMode;

/** @brief DFE clock speed as configured by the DFE PLL procedure
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef enum
{
    /// Selects the DFE at 245.76 MHz
    DFE_SPEED_245_76 = 0,
    /// Selects the DFE at 368.64 MHz
    DFE_SPEED_368_64,
    /// Selects the DFE at a custom speed
    DFE_SPEED_CUSTOM
} DFE_Speed;

/** @brief (addr, data) register pair
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct
{
    /// offset address from DFE base address
    uint32_t addr;
    /// data to/from addr  
    uint32_t data;
} DFE_RegPair;

/** @brief DFE Device information
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct
{
    /// DFE peripheral ID
    uint32_t pid;
    /// DFE peripheral base address
    void  *baseAddr;
    /// DFE LLD version
    uint32_t version;
} DFE_DevInfo;

/** @brief BBTX power meter configuration
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct
{
    /// Enable power meter function
    DfeFl_BbPowMtrEnable enable;
    /// Output format of the power meter
    DfeFl_BbPowMtrOutFormat outFormat;
    /// DFE carrier type
    uint32_t countSource;
    /// Power meter input source
    DfeFl_BbPowMtrInSource inSource;
    /// TDD mode
    DfeFl_BbPowMtrTddMode tddMode;
    /// Delay from sync expressed in number of samples
    uint32_t syncDly;
    /// Meter interval expressed in number of samples
    uint32_t interval;
    /// Integration period expressed in number of samples
    uint32_t intgPd;
    /// Count of measurements, i.e. count of intervals
    uint32_t powUptIntvl;
} DFE_BbtxPowmtrConfig;

/** @brief BBRX power meter configuration
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct
{
    /// Enable power meter function
    DfeFl_BbPowMtrEnable enable;
    /// Output format of the power meter
    DfeFl_BbPowMtrOutFormat outFormat;
    /// DFE carrier type
    uint32_t countSource;
    /// Power meter input source
    DfeFl_BbPowMtrInSource inSource;
    /// TDD mode
    DfeFl_BbPowMtrTddMode tddMode;
    /// Delay from sync expressed in number of samples
    uint32_t syncDly;
    /// Meter interval expressed in number of samples
    uint32_t interval;
    /// Integration period expressed in number of samples
    uint32_t intgPd;
    /// Count of measurements, i.e. count of intervals
    uint32_t powUptIntvl;
    /// RX Maximum full scale power in dB for the programmed power interval time. 
    // Used by feed forward power update.  
    // Value is in units of 0.05dB resolution
    uint32_t maxdB;
} DFE_BbrxPowmtrConfig;

typedef struct
{
    // square clipper threshold
    uint32_t threshold;
    // clipper counter threshold (C1)
    uint32_t cc_thr;
    // peak threshold (TH0)
    uint32_t TH0;
    // peak counter threshold (C0)
    uint32_t peak_thr;
    // peakgain counter threshold (C2)
    uint32_t peakgain_thr;
} DFE_TxPAprotPeak;

typedef struct
{
    // mu_p for IIR
    uint32_t mu0;
    // mu_q for IIR
    uint32_t mu1;
    // RMS threshold to reduce CFR gain(TH1)
    uint32_t TH1;
    // RMS threshold to shut down (TH2)
    uint32_t TH2;
    // RMS threshold to peak approaching saturation (TH4)
    uint32_t TH4;
    // threshold selection for a1
    uint32_t th1Sel;
    // threshold selection for a2
    uint32_t th2Sel;
    // threshold selection for a6
    uint32_t th6Sel;
} DFE_TxPAprotRms;

typedef struct
{
    // maximum magnitude of D3
    uint32_t mag;
    // IIR output at D50
    uint32_t d50;
    // IIR output at D51
    uint32_t d51;
} DFE_TxPAprotPwrStatus;

// Jesd Tx bus to lane map
typedef struct
{
    // bus#, one of DfeFl_JesdTxTxBus (I0, Q0, I1, Q1)
    uint32_t bus;
    // slot#, 0 ~ 3
    uint32_t busPos;
} DFE_JesdTxLaneMapPos;

// Jesd Tx link status
typedef struct
{
    // first sync request received for the link
    //    0 – not seen first sync request
    //    1 – seen first sync request    
    uint32_t firstSyncRequest[DFE_FL_JESD_NUM_LINK];
    // error count as reported over SYNC~ interface. 
    uint32_t syncErrCount[DFE_FL_JESD_NUM_LINK];
    // SYSREF alignment counter bits
    uint32_t sysrefAlignCount;
    // captured interrupt bit for sysref_request_assert
    //    0 – sysref request not asserted
    //    1 – sysref request asserted
    uint32_t sysrefReqAssert;
    // captured interrupt bit for sysref_request_deassert
    //    0 – sysref request not de-asserted
    //    1 – sysref request de-asserted
    uint32_t sysrefReqDeassert;
    // captured interrupt bit for sysref_err on the link
    //    0 – no sysref error
    //    1 – sysref error
    uint32_t sysrefErr[DFE_FL_JESD_NUM_LINK];
} DFE_JesdTxLinkStatus;

// Jesd Tx lane status
typedef struct
{
    // synchronization state machine status for lane    
    uint32_t syncState[DFE_FL_JESD_NUM_LANE];
    // FIFO status
    // 0 - fifo not empty; 1 - fifo has been empty
    uint32_t fifoEmpty[DFE_FL_JESD_NUM_LANE];
    // 0 - no read error; 1 - fifo read error
    uint32_t fifoReadErr[DFE_FL_JESD_NUM_LANE];
    // 0 - fifo not full; 1 - fifo has been full
    uint32_t fifoFull[DFE_FL_JESD_NUM_LANE];
    // 0 - no write error; 1 - fifo write error
    uint32_t fifoWriteErr[DFE_FL_JESD_NUM_LANE];
} DFE_JesdTxLaneStatus;

// Jesd Rx link status
typedef struct
{
    // error count as reported over SYNC~ interface.
    uint32_t syncErrCount[DFE_FL_JESD_NUM_LINK];
    // SYSREF alignment counter bits
    uint32_t sysrefAlignCount;
    // captured interrupt bit for sysref_request_assert
    //    0 – sysref request not asserted
    //    1 – sysref request asserted
    uint32_t sysrefReqAssert;
    // captured interrupt bit for sysref_request_deassert
    //    0 – sysref request not de-asserted
    //    1 – sysref request de-asserted
    uint32_t sysrefReqDeassert;
    // captured interrupt bit for sysref_err on the link
    //    0 – no sysref error
    //    1 – sysref error
    uint32_t sysrefErr[DFE_FL_JESD_NUM_LINK];
} DFE_JesdRxLinkStatus;

// Jesd Rx lane status
typedef struct
{
    // code group synchronization state machine status for lane        
    uint32_t codeState[DFE_FL_JESD_NUM_LANE];
    // frame synchronization state machine status for lane
    uint32_t frameState[DFE_FL_JESD_NUM_LANE];
    // 0 - no error; 1 - 8B/10B disparity error
    uint32_t decDispErr[DFE_FL_JESD_NUM_LANE];
    // 0 - no error; 1 - 8B/10B not-in-table code error
    uint32_t decCodeErr[DFE_FL_JESD_NUM_LANE];
    // 0 - no error; 1 - code group sync error
    uint32_t codeSyncErr[DFE_FL_JESD_NUM_LANE];
    // 0 - no error; 1 - elastic buffer match error 
    //(first non-/K/ doesn't match match_ctrl and match_data)
    uint32_t bufMatchErr[DFE_FL_JESD_NUM_LANE];
    // 0 - no error; 1 - elastic buffer overflow error (bad RBD value)
    uint32_t bufOverflowErr[DFE_FL_JESD_NUM_LANE];
    // 0 - no error; 1 - link configuration error
    uint32_t linkConfigErr[DFE_FL_JESD_NUM_LANE];
    // 0 - no error; 1 - frame alignment error
    uint32_t frameAlignErr[DFE_FL_JESD_NUM_LANE];
    // 0 - no error; 1 - multiframe alignment error
    uint32_t multiframeAlignErr[DFE_FL_JESD_NUM_LANE];
    // FIFO status
    // 0 - normal; 1 - fifo empty
    uint32_t fifoEmpty[DFE_FL_JESD_NUM_LANE];
    // 0 - normal; 1 - fifo read error
    uint32_t fifoReadErr[DFE_FL_JESD_NUM_LANE];
    // 0 - normal; 1 - fifo full
    uint32_t fifoFull[DFE_FL_JESD_NUM_LANE];
    // 0 - normal; 1 - fifo write error
    uint32_t fifoWriteErr[DFE_FL_JESD_NUM_LANE];
    // 0 - normal; 1 - test sequence verification failed
    uint32_t testSeqErr[DFE_FL_JESD_NUM_LANE];
} DFE_JesdRxLaneStatus;

// Jesd Rx lane to bus map
typedef struct
{
    // Rx lane#, 0 ~ 3
    uint32_t lane;
    // lane time slot, 0 ~ 3
    uint32_t lanePos;
    // if zero data
    uint32_t zeroBits;
} DFE_JesdRxBusMapPos;

// Rx Equalizer Taps
typedef struct
{
    float taps_ii[DFE_FL_RX_EQR_LEN];
    float taps_iq[DFE_FL_RX_EQR_LEN];
    float taps_qi[DFE_FL_RX_EQR_LEN];
    float taps_qq[DFE_FL_RX_EQR_LEN];
} DFE_RxEqrTaps;

// Fb Equalizer Taps
typedef struct
{
    float taps_ii[DFE_FL_FB_EQR_LEN];
    float taps_iq[DFE_FL_FB_EQR_LEN];
    float taps_qi[DFE_FL_FB_EQR_LEN];
    float taps_qq[DFE_FL_FB_EQR_LEN];
} DFE_FbEqrTaps;


// CB buf configuration
typedef struct
{
    // cb buf mode set
    DfeFl_CbModeSet  cbSet;
    // cb buf delay from sync
    uint32_t dly;
    // 0 = 1s/1c mode; 1 = 2s/1c mode
    uint32_t rate_mode;
    // capture buffer A fractional counter length minus 1; range 0-15; value depends on the relative sampling rates for different buffers
    uint32_t frac_cnt;
    // fractional counter sync select
    uint32_t frac_cnt_ssel;
    // length counter sync select
    uint32_t len_cnt_ssel;
    // cb buf length, upto 8192 complex data
    uint32_t length;
} DFE_CbBufCfg;

/**
 * @brief CB data
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct
{
    /// I data
    uint32_t Idata;
    /// Q data
    uint32_t Qdata;
} DFE_CbData;

/**
 * @brief CPP/DMA resource reserved table
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_CppResTbl
{
    /// reserved dma bitmask
    uint32_t dmaRsvd;
    
    /// discrete trigger out
    uint32_t discreteTrig[DFE_FL_CPP_NUM_DISCRETE_TRIGGERS];
    
    /// four 32-bits words, each bit corresponding to one descriptor
    /// reserved descriptor bitmask
    uint32_t descripRsvd[4];
        
} DFE_CppResTbl;

typedef struct _DFE_DpdData
{
    /// lutGain
    DfeFl_DpdComplexInt lutGain;
    /// lutSlope
    DfeFl_DpdComplexInt lutSlope;
} DFE_DpdData;

typedef struct _DFE_DpdCfg
{
    /// subchip mode
    uint32_t subchip_mode;
    /// subsample
    uint32_t subsample;
    /// dpd input scale
    uint32_t dpdInputScale;
    /// x2 sqrt
    uint32_t x2_sqrt;
} DFE_DpdCfg;

/**
 * @brief DDUC Exception Status
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_EeDducStatus
{
    /// Dduc cicov error
    uint32_t cicovErr;
    /// Dduc hop rollover error
    uint32_t hopRolloverErr;
    /// Dduc hop halfway error
    uint32_t hopHalfwayErr;
} DFE_EeDducStatus;

/**
 * @brief CFR PDC Exception Status
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_EeCfrPdcStatus
{
    /// a0s0 error interrupt
    uint32_t a0s0Err;
    /// a0s1 error interrupt
    uint32_t a0s1Err;
    /// a1s0 error interrupt
    uint32_t a1s0Err;
    /// a1s1 error interrupt
    uint32_t a1s1Err;
} DFE_EeCfrPdcStatus;

/**
 * @brief CFR Exception Status
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_EeCfrStatus
{
    /// AxSx error interrupt
    DFE_EeCfrPdcStatus cfrPdc[2];
    /// AGC inout interrupt min in threshold antenna x
    uint32_t cfrAgcMinInThErr[2];
    /// AGC inout interrupt min out threshold antenna x
    uint32_t cfrAgcMinOutThErr[2];
    /// AGC inout interrupt max in threshold antenna x
    uint32_t cfrAgcMaxInThErr[2];
    /// AGC inout interrupt max out threshold antenna x
    uint32_t cfrAgcMaxOutThErr[2];
    /// AGC syncX error interrupt
    uint32_t cfrAgcSyncErr[2][2];
    /// DTH power change error interrupt
    uint32_t cfrDthPwrCngErr[2];
    /// DTH sync0 error interrupt
    uint32_t cfrDthSync0Err[2];
} DFE_EeCfrStatus;

/**
 * @brief JESD TX Lane Exception Status
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_EeJesdTxlaneStatus
{
    /// fifo empty
    uint32_t fifoEmptyIntr;
    /// fifo read error because fifo empty
    uint32_t fifoReadErrIntr;
    /// fifo full
    uint32_t fifoFullIntr;
    /// fifo write error because fifo full
    uint32_t fifoWriteErrIntr;
} DFE_EeJesdTxlaneStatus;

/**
 * @brief JESD RX Lane Exception Status
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_EeJesdRxlaneStatus
{
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
} DFE_EeJesdRxlaneStatus;

/**
 * @brief MISC Exception Status
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_EeMiscCppIntStatus
{
    /// Sync bus captured interrupt bits
    uint32_t                syncSigInt[16];
    /// CPP DMA captured interrupt bits
    uint32_t                cppDmaDoneInt[32];
    /// Other misc captured interrupt bit
    DfeFl_MiscMiscIntrGroup arbGpio;
} DFE_EeMiscCppIntStatus;

/**
 * @brief This structure contains all the DFE exception counters.
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_EeCountObj {
    /** Holds counters for dfe BB error interrupts */
    DfeFl_BbGeneralIntrGroup    bbErr;
    /** Holds counters for dfe Tx power meter error interrupts */
    uint32_t                    bbtxpmErr[16];
    /** Holds counters for dfe Tx gain update error interrupts */
    uint32_t                    bbtxgainErr[16];
    /** Holds counters for dfe Tx power meter error interrupts */
    uint32_t                    bbrxpmErr[16];
    /** Holds counters for dfe Tx gain update error interrupts */
    uint32_t                    bbrxgainErr[16];
    /** Holds counters for dfe DDUC error interrupts */
    DFE_EeDducStatus            dducErr[4];
    /** Holds counters for dfe CFR error interrupts */
    DFE_EeCfrStatus             cfrErr[2];
    /** tx X antenna Y power is approaching saturation error interrupt */
    uint32_t                    txPaPowerErr[2][2];
    /** tx X antenna Y peak is approaching saturation error interrupt */
    uint32_t                    txPaPeakErr[2][2];
    /** Holds counters for dfe Jesd Tx SysRef error interrupts */
    DfeFl_JesdTxSysrefIntrs     jesdTxSysref;
    /** Holds counters for dfe Jesd Rx SysRef  error interrupts */
    DfeFl_JesdRxSysrefIntrs     jesdRxSysref;
    /** Holds counters for dfe Jesd Tx lane error interrupts */
    DFE_EeJesdTxlaneStatus      jesdTxLaneErr[4];
    /** Holds counters for dfe Jesd Rx lane error interrupts */
    DFE_EeJesdRxlaneStatus      jesdRxLaneErr[4];
    /** Holds counters for dfe Dpda error interrupts */
    DfeFl_DpdaIntrStatus        dpdaErr;
    /** Holds counters for dfe Rx Ibpm error interrupts */
    uint32_t                    rxIbpmInt[4];
    /** Holds counters for dfe capture buffer done interrupts */
    uint32_t                    cbDoneInterrupt;
    /** Holds counters for dfe Misc error interrupts */
    DFE_EeMiscCppIntStatus      miscErr;
    /** Holds counters for dfe Misc error interrupts */
    DfeFl_MiscMasterHiPriIntrGroup masterHiPrioErr;
    /** Holds a flag telling whether any of the enabled exceptions occurred, 0 if none, 1 if any */
    uint32_t                    eeFlag;
    /** Holds a flag telling whether any of the enabled information flags occurred, 0 if none, 1 if any */
    uint32_t                    infoFlag;
} DFE_EeCountObj, *DFE_EeCountHandle;

/** @brief DFE device instance context
 * @ingroup DFE_LLD_DATASTRUCT
 */
typedef struct _DFE_Obj
{   
    
    /// DFE CSL context 
    DfeFl_Context                         dfeCtx;
    /// DFE CSL param
    DfeFl_Param                           dfeParam;
    /// DFE clock speed
    DFE_Speed                             dfeSpeed;
    /// DFE custom clock speed in KHz
    uint32_t                              dfeCustomSpeed;
    /// DFE CSL Handle
    DfeFl_Handle                          hDfe;
    /// DFE_BB CSL Handle
    DfeFl_BbHandle                        hDfeBb[DFE_FL_BB_PER_CNT];
    /// DFE_DDUC CSL Handle
    DfeFl_DducHandle                      hDfeDduc[DFE_FL_DDUC_PER_CNT];
    /// DFE_SUMMER CSL Handle
    DfeFl_SummerHandle                    hDfeSummer[DFE_FL_SUMMER_PER_CNT];
    /// DFE_AUTOCP CSL Handle
    DfeFl_AutocpHandle                    hDfeAutocp[DFE_FL_AUTOCP_PER_CNT];
    /// DFE_CFR CSL Handle
    DfeFl_CfrHandle                       hDfeCfr[DFE_FL_CFR_PER_CNT];
    /// DFE_CDFR CSL Handle
    DfeFl_CdfrHandle                      hDfeCdfr[DFE_FL_CDFR_PER_CNT];
    /// DFE_DPD CSL Handle
    DfeFl_DpdHandle                       hDfeDpd[DFE_FL_DPD_PER_CNT];
    /// DFE_DPD disabled flag: 0 on TIC6630K2L devices, 1 on 66AK2L06
    uint32_t                              dpdIsDisabled;
    /// DFE_DPDA CSL Handle
    DfeFl_DpdaHandle                      hDfeDpda[DFE_FL_DPDA_PER_CNT];
    /// DFE_DPDA disabled flag: 0 on TIC6630K2L devices, 1 on 66AK2L06
    uint32_t                              dpdaIsDisabled;
    /// DFE_TX CSL Handle
    DfeFl_TxHandle                        hDfeTx[DFE_FL_TX_PER_CNT];
    /// DFE_RX CSL Handle
    DfeFl_RxHandle                        hDfeRx[DFE_FL_RX_PER_CNT];
    /// DFE_CB CSL Handle
    DfeFl_CbHandle                        hDfeCb[DFE_FL_CB_PER_CNT];
    /// DFE_JESD CSL Handle
    DfeFl_JesdHandle                      hDfeJesd[DFE_FL_JESD_PER_CNT];
    /// DFE_FB CSL Handle
    DfeFl_FbHandle                        hDfeFb[DFE_FL_FB_PER_CNT];
    /// DFE_MISC CSL Handle
    DfeFl_MiscHandle                      hDfeMisc[DFE_FL_MISC_PER_CNT];
    /// DFE_CPP CSL resource manager 
    DfeFl_CppResMgr                       cppResMgr;
    /// BBTX power meter for CPP/DMA
    uint32_t                              bbtxPowmtr;
    /// dma handle for BBTX power meter
    DfeFl_CppDmaHandle                    hDmaBbtxPowmtr;
    /// descriptor handle for BBTX power meter
    DfeFl_CppDescriptorHandle             hDescripBbtxPowmtr;
    /// IQN2 CTL Ingress Channel for BBTX power meter
    uint32_t                              bbtxPowmtrIqnChnl;
    /// BBRX power meter for CPP/DMA
    uint32_t                              bbrxPowmtr;
    /// dma handle for BBRX power meter
    DfeFl_CppDmaHandle                    hDmaBbrxPowmtr;
    /// descriptor handle for BBRX power meter
    DfeFl_CppDescriptorHandle             hDescripBbrxPowmtr;
    /// descriptor handle for BBTX power meter
    uint32_t                              bbrxPowmtrIqnChnl;
    /// flag to read all 18 bit Cb
    uint32_t                              flag_18bit;
    /// dma handle for cb
    DfeFl_CppDmaHandle                    hDmaCb;
    /// descriptor handle for cb
    DfeFl_CppDescriptorHandle             hDescripCb[8];
    /// IQN2 CTL Channel
    uint32_t                              cbIqnChnl;
    /// sync counter ssel
    DfeFl_MiscSyncGenSig                  sync_cnter_ssel;
    /// bbtx siggen ssel
    uint32_t                              bbtx_siggen_ssel;
    /// bbrx checksum ssel
    uint32_t                              bbrx_chksum_ssel;
    /// ulStrobe strobe
    DfeFl_MiscSyncGenSig                  ulStrobe_Sync;
    /// RX IBPM Unity Magnitude Sqaure value (=I^2 + Q^2)
    uint64_t                              rxIbpmUnityMagsq;
    /// generic DMA hndle
    DfeFl_CppDmaHandle                    hDmaGeneric;
    /// IQN2 CTL Egress channel for generic writing DMA
    uint32_t                              genericDmaIqnChnlDl;
    /// IQN2 CTL Ingress channel for generic reading DMA
    uint32_t                              genericDmaIqnChnlUl;
    /// Exception counters - updated when exception are enabled
    DFE_EeCountObj                        dfeEeCount;
    /// DFE CSL object
    DfeFl_Obj                             objDfe;
    /// DFE_BB CSL object
    DfeFl_BbObj                           objDfeBb[DFE_FL_BB_PER_CNT];
    /// DFE_DDUC CSL object
    DfeFl_DducObj                         objDfeDduc[DFE_FL_DDUC_PER_CNT];
    /// DFE_SUMMER CSL object
    DfeFl_SummerObj                       objDfeSummer[DFE_FL_SUMMER_PER_CNT];
    /// DFE_AUTOCP CSL object
    DfeFl_AutocpObj                       objDfeAutocp[DFE_FL_AUTOCP_PER_CNT];
    /// DFE_CFR CSL object
    DfeFl_CfrObj                          objDfeCfr[DFE_FL_CFR_PER_CNT];
    /// DFE_CDFR CSL object
    DfeFl_CdfrObj                         objDfeCdfr[DFE_FL_CDFR_PER_CNT];
    /// DFE_DPD CSL object
    DfeFl_DpdObj                          objDfeDpd[DFE_FL_DPD_PER_CNT];
    /// DFE_DPDA CSL object
    DfeFl_DpdaObj                         objDfeDpda[DFE_FL_DPDA_PER_CNT];
    /// DFE_TX CSL object
    DfeFl_TxObj                           objDfeTx[DFE_FL_TX_PER_CNT];
    /// DFE_RX CSL object
    DfeFl_RxObj                           objDfeRx[DFE_FL_RX_PER_CNT];
    /// DFE_CB CSL object
    DfeFl_CbObj                           objDfeCb[DFE_FL_CB_PER_CNT];
    /// DFE_JESD CSL object
    DfeFl_JesdObj                         objDfeJesd[DFE_FL_JESD_PER_CNT];
    /// DFE_FB CSL object
    DfeFl_FbObj                           objDfeFb[DFE_FL_FB_PER_CNT];
    /// DFE_MISC CSL object
    DfeFl_MiscObj                         objDfeMisc[DFE_FL_MISC_PER_CNT];

} DFE_Obj;
typedef DFE_Obj * DFE_Handle;

// callback context
typedef void * DFE_CallbackContext;

/** @brief Callback function for waiting sync signal
 * @ingroup DFE_LLD_DATASTRUCT
 * 
 *  return DFE_ERR_OK if sync signal come properly
 *  return DFE_ERR_SYNC_NO_COME if sync signal not come before timed out
 */
typedef DFE_Err (*DFE_CallbackWaitSync)
(
    /// callback context
    DFE_CallbackContext cbkCtx, 
    /// DFE device handle
    DFE_Handle hDfe, 
    /// sync signal to be waiting
    DfeFl_MiscSyncGenSig syncSig
);

// Open DFE LLD device.
DFE_Handle Dfe_open
(
    int dfeInst, 
    DFE_Obj *dfeObj, 
    DFE_CppResTbl *dfeResTbl,
    uint32_t baseAddress,
    DFE_Err *err
);

// Close DFE LLD device opened by Dfe_open().
DFE_Err Dfe_close
(
    DFE_Handle hDfe
);

// Load DFE target configuration, i.e. registers contents.
DFE_Err Dfe_loadTgtCfg
(
    DFE_Handle hDfe,
    DFE_RegPair tgtCfgPairs[]
);

// Do soft reset
DFE_Err Dfe_softReset
(
    DFE_Handle hDfe
);

// DFE initialization sequence for tx path, BB => DDUC => CFR => DPD => TX => JESDTX
DFE_Err Dfe_initTgtTx
(
    DFE_Handle hDfe,
    DFE_CallbackContext waitSyncCtx,
    DFE_CallbackWaitSync waitSyncFxn
);

// DFE initialization sequence for tx path, BB <= DDUC <= RX/FB <= JESDRX
DFE_Err Dfe_initTgtRx
(
    DFE_Handle hDfe,
    DFE_CallbackContext waitSyncCtx,
    DFE_CallbackWaitSync waitSyncFxn
);

// Get back DFE device information, such as PID, base address etc.
DFE_Err Dfe_getDevInfo
(
    DFE_Handle hDfe,
    DFE_DevInfo *devInfo
);

// Issue a sync signal.
DFE_Err Dfe_issueSync
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig syncSig,
    uint32_t waitCnt
);

// Get a sync signal status.
DFE_Err Dfe_getSyncStatus
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig syncSig,
    uint32_t *signaled
);

// Program a sync counter
DFE_Err Dfe_progSyncCounter
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenCntr cntr,
    uint32_t delay,
    uint32_t period,
    uint32_t pulseWidth,
    uint32_t repeat,
    uint32_t invert
);

// Issue sync to start the sync counter, and return without waiting.
DFE_Err Dfe_issueSyncStartSyncCounter
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenCntr cntr,
    DfeFl_MiscSyncGenSig ssel
);

// Program BBTX AxCs’ gains.
DFE_Err Dfe_progBbtxGain
(
    DFE_Handle hDfe,
    uint32_t  numAxCs,
    uint32_t  axc[],
    float  gain[]
);

// Issue sync to update BBTX gain
DFE_Err Dfe_issueSyncUpdateBbtxGain
(
    DFE_Handle hDfe,
    uint32_t ct,
    DfeFl_MiscSyncGenSig ssel
);

// Get BBTX gain update complete
DFE_Err Dfe_getBbtxGainUpdateComplete
(
    DFE_Handle hDfe,
    uint32_t ct,
    uint32_t *complete
);

// Clear BBTX gain update complete interrupt status
DFE_Err Dfe_clearBbtxGainUpdateCompleteIntrStatus
(
    DFE_Handle hDfe,
    uint32_t ct
);

// Program BBTX power meter
DFE_Err Dfe_progBbtxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DFE_BbtxPowmtrConfig *mtrCfg
);

// Issue sync to update BBTX power meter
DFE_Err Dfe_issueSyncUpdateBbtxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DfeFl_MiscSyncGenSig ssel
);

// Clear BBTX power meter complete interrupt status
DFE_Err Dfe_clearBbtxPowmtrDoneIntrStatus
(
    DFE_Handle hDfe,
    uint32_t pmId
);

// Get BBTX power meter complete interrupt status
DFE_Err Dfe_getBbtxPowmtrDoneIntrStatus
(
    DFE_Handle hDfe,
    uint32_t pmId,
    uint32_t *complete
);

// Read BBTX power meter via CPU
DFE_Err Dfe_readBbtxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    float *peak,
    float *rms
);

// Open CPP/DMA for BBTX power meters
DFE_Err Dfe_openBbtxPowmtrDma
(
    DFE_Handle hDfe,
    uint32_t cppDmaId,
    uint32_t cppDescripId,
    uint32_t iqnChnl
);

// Close CPP/DMA for BBTX power meters
DFE_Err Dfe_closeBbtxPowmtrDma
(
    DFE_Handle hDfe
);

// Enable CPP/DMA for BBTX power meters
DFE_Err Dfe_enableBbtxPowmtrDma
(
    DFE_Handle hDfe,
    uint32_t pmId
);

// disable CPP/DMA for BBTX power meters
DFE_Err Dfe_disableBbtxPowmtrDma
(
    DFE_Handle hDfe
);

// Program BBRX AxCs’ gains.
DFE_Err Dfe_progBbrxGain
(
    DFE_Handle hDfe,
    uint32_t  numAxCs,
    uint32_t  axc[],
    float  gain[]
);

// Issue sync to update BBRX gain
DFE_Err Dfe_issueSyncUpdateBbrxGain
(
    DFE_Handle hDfe,
    uint32_t ct,
    DfeFl_MiscSyncGenSig ssel
);

// Get BBRX gain update complete
DFE_Err Dfe_getBbrxGainUpdateComplete
(
    DFE_Handle hDfe,
    uint32_t ct,
    uint32_t *complete
);

// Clear BBRX gain update complete interrupt status
DFE_Err Dfe_clearBbrxGainUpdateCompleteIntrStatus
(
    DFE_Handle hDfe,
    uint32_t ct
);

// Program BBRX power meter
DFE_Err Dfe_progBbrxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DFE_BbrxPowmtrConfig *mtrCfg
);

// Issue sync to update BBRX power meter
DFE_Err Dfe_issueSyncUpdateBbrxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DfeFl_MiscSyncGenSig ssel
);

// Clear BBRX power meter complete interrupt status
DFE_Err Dfe_clearBbrxPowmtrDoneIntrStatus
(
    DFE_Handle hDfe,
    uint32_t pmId
);

// Get BBRX power meter complete interrupt status
DFE_Err Dfe_getBbrxPowmtrDoneIntrStatus
(
    DFE_Handle hDfe,
    uint32_t pmId,
    uint32_t *complete
);

// Read BBTX power meter via CPU
DFE_Err Dfe_readBbrxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    float *peak,
    float *rms
);

// Open CPP/DMA for BBRX power meters
DFE_Err Dfe_openBbrxPowmtrDma
(
    DFE_Handle hDfe,
    uint32_t cppDmaId,
    uint32_t cppDescripId,
    uint32_t iqnChnl
);

// Close CPP/DMA for BBRX power meters
DFE_Err Dfe_closeBbrxPowmtrDma
(
    DFE_Handle hDfe
);

// Enable CPP/DMA for BBRX power meters
DFE_Err Dfe_enableBbrxPowmtrDma
(
    DFE_Handle hDfe,
    uint32_t pmId
);

// disable CPP/DMA for BBRX power meters
DFE_Err Dfe_disableBbrxPowmtrDma
(
    DFE_Handle hDfe
);

// enable/disable BB AID loopback
DFE_Err Dfe_enableDisableBbaidLoopback
(
    DFE_Handle hDfe,
    uint32_t enable
);

// Program BB Buf Loopback
DFE_Err Dfe_progBbbufLoopback
(
    DFE_Handle hDfe,
    DfeFl_BbLoopbackConfig *bufLoopback
);

// Set BB AID UL strobe delay
DFE_Err Dfe_setBbaidUlstrobeDelay
(
    DFE_Handle hDfe,
    uint32_t ct,
    uint32_t dly
);

// Program BB Signal Generator
DFE_Err Dfe_progBbsigGenRamp
(
    DFE_Handle hDfe,
    DfeFl_BbTestGenDev sigGenDev, 
    uint32_t enable,
    int16_t startIq[2],
    int16_t stopIq[2],
    int16_t slopeIq[2]
);

// Issue Sync to Update BB SigGen
DFE_Err Dfe_issueSyncUpdateBbsigGen
(
    DFE_Handle hDfe,
    DfeFl_BbTestGenDev sigGenDev, 
    DfeFl_MiscSyncGenSig ssel
);

// Program BB testbus
DFE_Err Dfe_progBbtestbus
(
    DFE_Handle hDfe,
    DfeFl_BbTestCbCtrl testCbCtrl, 
    uint32_t testCbAxc
);

// Program DDUC Mixer NCO frequency
DFE_Err Dfe_progDducMixerNCO
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    float refClock,
    float freq[12]
);

// Issue Sync to update DDUC Mixer NCO frequency
DFE_Err Dfe_issueSyncUpdateDducMixerNCO
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    DfeFl_MiscSyncGenSig ssel
);

// Program DDUC Mixer Phase
DFE_Err Dfe_progDducMixerPhase
(
    DFE_Handle hDfe,
    uint32_t  dducDev,
    float  phase[12]
);

// Issue Sync to update DDUC Mixer phase
DFE_Err Dfe_issueSyncUpdateDducMixerPhase
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    DfeFl_MiscSyncGenSig ssel
);

// Program DDUC Farrow phase
DFE_Err Dfe_progDducFarrowPhase
(
    DFE_Handle hDfe,
    uint32_t  dducDev,
    uint32_t fifo[12],
    float  phase[12]
);

// Issue Sync to update DDUC Farrow phase
DFE_Err Dfe_issueSyncUpdateDducFarrowPhase
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    DfeFl_MiscSyncGenSig ssel
);

// Program Distributor Map
DFE_Err Dfe_progDducDistMap
(
    DFE_Handle hDfe,
    uint32_t  dducDev,
    uint32_t  rxSel[12],
    uint32_t  chanSel[12]
);

// Issue Sync to update Distributor map
DFE_Err Dfe_issueSyncUpdateDducDistMap
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    DfeFl_MiscSyncGenSig ssel
);

// Program Summer shift
DFE_Err Dfe_progSummerShift
(
    DFE_Handle hDfe,
    uint32_t cfrId,
    uint32_t strId,
    int  gain
);

// Program Summer map
DFE_Err Dfe_progSummerMap
(
    DFE_Handle hDfe,
    uint32_t cfrId,
    uint32_t strId,
    uint32_t sumMap[4]
);

// Issue Sync to update Summer map
DFE_Err Dfe_issueSyncUpdateSummerMap
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
);

// Program Cfr coefficients
DFE_Err Dfe_progCfrCoeff
(
    DFE_Handle hDfe,
    uint32_t cfrDev,
    uint32_t numCoeffs,
    uint32_t *cfrCoeff_i,
    uint32_t *cfrCoeff_q
);

// Issue Sync Update Cfr coefficients
DFE_Err Dfe_issueSyncUpdateCfrCoeff
(
    DFE_Handle hDfe,
    uint32_t cfrDev,
    uint32_t coeffType,
    DfeFl_MiscSyncGenSig ssel
);

// Program Cfr preGain
DFE_Err Dfe_progCfrPreGain
(
    DFE_Handle hDfe,
    uint32_t cfrDev,
    DfeFl_CfrPath cfrPath,
    float gain
);

// Issue Sync Update Cfr preGain
DFE_Err Dfe_issueSyncUpdatCfrPreGain
(
    DFE_Handle hDfe,
    uint32_t cfrDev,
    DfeFl_CfrPath cfrPath,
    DfeFl_MiscSyncGenSig ssel
);

// Program Cfr postGain
DFE_Err Dfe_progCfrPostGain
(
    DFE_Handle hDfe,
    uint32_t cfrDev,
    DfeFl_CfrPath cfrPath,
    float gain
);

// Issue Sync Update Cfr postGain
DFE_Err Dfe_issueSyncUpdatCfrPostGain
(
    DFE_Handle hDfe,
    uint32_t cfrDev,
    DfeFl_CfrPath cfrPath,
    DfeFl_MiscSyncGenSig ssel
);

// Program Cfr protection gain
DFE_Err Dfe_progCfrProtGain
(
    DFE_Handle hDfe,
    uint32_t cfrDev,
    DfeFl_CfrPath cfrPath,
    float gain
);

// Program Tx Mixer
DFE_Err Dfe_progTxMixer
(
    DFE_Handle hDfe,
    DfeFl_TxPath txPath,
    float refClock,
    float freq[2]
);

// Issue sync to update Tx Mixer
DFE_Err Dfe_issueSyncUpdateTxMixer
(
    DFE_Handle hDfe,
    DfeFl_TxPath txPath,
    DfeFl_MiscSyncGenSig ssel
);

// Program Tx PA protection
DFE_Err Dfe_progTxPaProtection
(
    DFE_Handle hDfe,
    DfeFl_TxDev txDev,
    DFE_TxPAprotPeak txPAprotPeak,
    DFE_TxPAprotRms txPAprotRms,
    uint32_t mask
);

// Get Tx PA protection interrupt status
DFE_Err Dfe_getTxPAprotIntrStatus
(
    DFE_Handle hDfe,
    DfeFl_TxPaIntrpt *txPAprotIntr
);

// Clear Tx PA protection interrupt status
DFE_Err Dfe_clearTxPAprotIntrStatus
(
    DFE_Handle hDfe,
    DfeFl_TxDev txDev
);

// Read Tx PA protection power status
DFE_Err Dfe_getTxPAprotPwrStatus
(
    DFE_Handle hDfe,
    DfeFl_TxDev txDev,
    uint32_t clrRead,
    DFE_TxPAprotPwrStatus *txPAprotPwrStatus
);


// Get JESDTX lane enable status
DFE_Err Dfe_getJesdTxLaneEnable
(
    DFE_Handle hDfe,
    uint32_t laneEnable[4],
    uint32_t linkAssign[4]
);

// Program JESD Tx bus to lane map.
DFE_Err Dfe_mapJesdTx2Lane
(
    DFE_Handle hDfe,
    uint32_t lane,
    DFE_JesdTxLaneMapPos laneMap[4]
);

// Program JESDTX Signal Generator to produce a ramp.
DFE_Err Dfe_progJesdTxSigGenRamp
(
    DFE_Handle hDfe,
    DfeFl_JesdTxSignalGen sigGenDev, 
    uint32_t enable,
    int32_t start,
    int32_t stop,
    int32_t slope
);
// Issue sync update JESDTX Signal Generator.
DFE_Err Dfe_issueSyncUpdateJesdTxSigGen
(
    DFE_Handle hDfe,
    DfeFl_JesdTxSignalGen sigGenDev, 
    DfeFl_MiscSyncGenSig ssel
);

// Program JESD Tx testbus
DFE_Err Dfe_progJesdTxTestbus
(
    DFE_Handle hDfe,
    DfeFl_JesdTxTestBusSel tp
);

// Get JESD Tx link status
DFE_Err Dfe_getJesdTxLinkStatus
(
    DFE_Handle hDfe,
    DFE_JesdTxLinkStatus *linkStatus
);

// Get JESD Tx lane status
DFE_Err Dfe_getJesdTxLaneStatus
(
    DFE_Handle hDfe,
    DFE_JesdTxLaneStatus *laneStatus
);

// Clear JESD Tx link error
DFE_Err Dfe_clearJesdTxLinkErrors
(
    DFE_Handle hDfe
);

// Clear JESD Tx lane error
DFE_Err Dfe_clearJesdTxLaneErrors
(
    DFE_Handle hDfe
);

// Get JESD RX lane enable status
DFE_Err Dfe_getJesdRxLaneEnable
(
    DFE_Handle hDfe,
    uint32_t laneEnable[4],
    uint32_t linkAssign[4]
);

// program JESD lane to Rx map
DFE_Err Dfe_mapJesdLane2Rx
(
    DFE_Handle hDfe,
    uint32_t rxBus,
    DFE_JesdRxBusMapPos busMap[4]
);

// Program JESD Rx testbus
DFE_Err Dfe_progJesdRxTestbus
(
    DFE_Handle hDfe,
    DfeFl_JesdRxTestBusSel tp
);

// Init JESD tx state machine.
DFE_Err Dfe_initJesdTx
(
    DFE_Handle hDfe
);

// Init JESD rx state machine.
DFE_Err Dfe_initJesdRx
(
    DFE_Handle hDfe
);

// Program JESD loopbacks for sync_n, lanes or links.
DFE_Err Dfe_progJesdLoopback
(
    DFE_Handle hDfe,
    uint32_t lpbkSync[DFE_FL_JESD_NUM_LINK],
    DfeFl_JesdRxLoopbackConfig *lpbkLaneLink
);

// Get current JESD loopbacks for sync_n, lanes or links.
DFE_Err Dfe_getJesdLoopback
(
    DFE_Handle hDfe,
    uint32_t lpbkSync[DFE_FL_JESD_NUM_LINK],
    DfeFl_JesdRxLoopbackConfig *lpbkLaneLink
);

// Get JESD Rx link status
DFE_Err Dfe_getJesdRxLinkStatus
(
    DFE_Handle hDfe,
    DFE_JesdRxLinkStatus *linkStatus
);

// Get JESD Rx lane status
DFE_Err Dfe_getJesdRxLaneStatus
(
    DFE_Handle hDfe,
    DFE_JesdRxLaneStatus *laneStatus
);

// Clear JESD Rx link error
DFE_Err Dfe_clearJesdRxLinkErrors
(
    DFE_Handle hDfe
);

// Clear JESD Rx lane error
DFE_Err Dfe_clearJesdRxLaneErrors
(
    DFE_Handle hDfe
);

// Program RX IBPM global
DFE_Err Dfe_progRxIbpmGlobal
(
    DFE_Handle hDfe,
    DfeFl_RxPowmtrReadMode readMode,
    float histThresh1,
    float histThresh2,
    uint64_t unityMagsq   
);

// Program individual RX IBPM
DFE_Err Dfe_progRxIbpm
(
    DFE_Handle hDfe,
    uint32_t pmId,
    uint32_t oneShot,
    uint32_t meterMode,
    uint32_t syncDelay,
    uint32_t nSamples,
    uint32_t interval
);

// Issue Sync Update Rx Ibpm
DFE_Err Dfe_issueSyncUpdateRxIbpm
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DfeFl_MiscSyncGenSig ssel
);

// Issue Read Request to Rx Ibpm
DFE_Err Dfe_issueRxIbpmReadRequest
(
    DFE_Handle hDfe,
    uint32_t pmId
);

// Get Rx Ibpm read ack
DFE_Err Dfe_getRxIbpmReadAck
(
    DFE_Handle hDfe,
    uint32_t pmId,
    uint32_t *ackRead
);

// Read Rx IBPM result
DFE_Err Dfe_readRxIbpmResult
(
    DFE_Handle hDfe,
    uint32_t pmId,
    float  *power,
    float  *peak,
    uint32_t *histCount1,
    uint32_t *histCount2
);

// Clear Read Request to Rx Ibpm
DFE_Err Dfe_clearRxIbpmReadRequest
(
    DFE_Handle hDfe,
    uint32_t pmId
);

// Program Rx Equalizer
DFE_Err Dfe_progRxEqr
(
    DFE_Handle hDfe,
    uint32_t rxDev,
    uint32_t shift,
    uint32_t numCoeff,
    DFE_RxEqrTaps *RxEqrTaps
);

// Issue Sync to update Rx Equalizer
DFE_Err Dfe_issueSyncUpdateRxEqr
(
    DFE_Handle hDfe,
    uint32_t rxDev,
    DfeFl_MiscSyncGenSig ssel
);

// Program Rx Mixer NCO
DFE_Err Dfe_progRxMixerNCO
(
    DFE_Handle hDfe,
    uint32_t rxDev,
    float refClock,
    float freq
);

// Issue Sync to update Rx Mixer NCO
DFE_Err Dfe_issueSyncUpdateRxMixerNCO
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
);

// Program Rx Testbus
DFE_Err Dfe_progRxTestbus
(
    DFE_Handle hDfe,
    uint32_t top_ctrl,
    uint32_t imb_ctrl,
    uint32_t feagc_dc_ctrl
);

// Program Fb Equalizer
DFE_Err Dfe_progFbEqr
(
    DFE_Handle hDfe,
    DfeFl_FbBlk FbBlkId,
    uint32_t numCoeff,
    DFE_FbEqrTaps *FbEqrTaps
);

// Issue sync update Fb Equalizer
DFE_Err Dfe_issueSyncUpdateFbEqr
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
);

// Program Fb Mixer NCO
DFE_Err Dfe_progFbMixerNCO
(
    DFE_Handle hDfe,
    DfeFl_FbBlk FbBlkId,
    float refClock,
    float freq
);

// Issue sync update Fb Mixer NCO
DFE_Err Dfe_issueSyncUpdateFbMixerNCO
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
);

// Program Fb IO Mux
DFE_Err Dfe_progFbIOMux
(
    DFE_Handle hDfe,
    DfeFl_FbIoMux ioMux
);

// Program Fb pre-cb gain
DFE_Err Dfe_progFbGain
(
    DFE_Handle hDfe,
    DfeFl_FbBlk FbBlkId,
    float FbGain[2]
);

// Issue sync update Fb gain
DFE_Err Dfe_issueSyncUpdateFbGain
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
);

// Program CB node configuration
DFE_Err Dfe_progCbNodecfg
(
    DFE_Handle hDfe,
    DfeFl_CbNodeCfg *nodeCfg
);

// Program CB buf configuration
DFE_Err Dfe_progCbBufcfg
(
    DFE_Handle hDfe,
    DFE_CbBufCfg *bufCfg
);

// Arm CB and Issue Sync
DFE_Err Dfe_armCbIssueSync
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
);

// Get CB done status
DFE_Err Dfe_getCbDoneStatus
(
    DFE_Handle hDfe,
    DfeFl_CbArm *cbDoneStatus
);

// Read Cb Buf and Status
DFE_Err Dfe_readCbBuf
(
    DFE_Handle hDfe,
    DfeFl_CbBuf cbBufId,
    uint32_t cbLength,
    uint32_t flag_18bit,
    DfeFl_CbComplexInt *cbTemp,
    DfeFl_CbStatus *cbStatus,
    DFE_CbData *cbData
);

// Open CB buf DMA for reading CB buf
DFE_Err Dfe_openCbBufDma
(
    DFE_Handle hDfe,
    uint32_t flag_18bit,
    uint32_t cppDmaId,
    uint32_t cppDescripId[8],
    uint32_t iqnChnl
);

// Close CB buf DMA
DFE_Err Dfe_closeCbBufDma
(
    DFE_Handle hDfe
);

// Enable CB buf DMA
DFE_Err Dfe_enableCbBufDma
(
    DFE_Handle hDfe
);

// Disable CB buf DMA
DFE_Err Dfe_disableCbBufDma
(
    DFE_Handle hDfe
);

// Disable All Testbus Probes
DFE_Err Dfe_disableAllTestbus
(
    DFE_Handle hDfe
);

// program pinmux of a DFE GPIO pin
DFE_Err Dfe_progGpioPinMux
(
    DFE_Handle hDfe,
    DfeFl_MiscGpioPin pinId,
    DfeFl_MiscGpioMux muxSel
);

// Set DFE GPIO Sync Out Source
DFE_Err Dfe_setGpioSyncOutSource
(
    DFE_Handle hDfe,
    uint32_t syncoutId,
    DfeFl_MiscSyncGenSig ssel
);

// Set DFE GPIO Bank Output
DFE_Err Dfe_setGpioBankOutput
(
    DFE_Handle hDfe,
    uint32_t bankOutput
);

// Get DFE GPIO Bank Input
DFE_Err Dfe_getGpioBankInput
(
    DFE_Handle hDfe,
    uint32_t *bankInput
);

// Open Cpp/DMA for Generic IO
DFE_Err Dfe_openGenericDma
(
    DFE_Handle hDfe,
    uint32_t cppDmaId,
    uint32_t iqnChnlDl,
    uint32_t iqnChnlUl
);

// Close Cpp/DMA for Generic IO
DFE_Err Dfe_closeGenericDma
(
    DFE_Handle hDfe
);

// Prepare CPP/DMA embeded header
DFE_Err Dfe_prepareGenericDmaHeader
(
    DFE_Handle hDfe,
    uint32_t *header,
    DFE_GenericDmaReadWriteMode rwMode,
    uint32_t offsetAddrInDref,
    uint32_t sizeOrData
);

// Enable toggle for one dpd block
DFE_Err Dfe_enableDpdToggle
(
    DFE_Handle hDfe,
    uint32_t  blkId
);

// Set sync selection for one dpd block
DFE_Err Dfe_setDpdSyncSel
(
    DFE_Handle hDfe,
    uint32_t  blkId,
    uint32_t  synch
);

// Issue sync update lut
DFE_Err Dfe_issueSyncUpdateDpdLut
(
    DFE_Handle hDfe,
    uint32_t numBlks,
    uint32_t blkId[],
    DfeFl_MiscSyncGenSig ssel
);

// Get current lut memory index status for one dpd block
DFE_Err Dfe_getDpdLutIdx
(
    DFE_Handle hDfe,
    uint32_t blkId,
    uint32_t *LutIdx
);

// program lut for one dpd cell
DFE_Err Dfe_progDpdLutTable
(
    DFE_Handle hDfe,
    uint32_t blkId,
    uint32_t rowId,
    uint32_t cellId,
    DFE_DpdData *DpdData
);

// get dpd configuration
DFE_Err Dfe_getDpdCfg
(
    DFE_Handle hDfe,
    DFE_DpdCfg *dpdCfg
);

// start DPDA
DFE_Err Dfe_startDpda(
        DFE_Handle hDfe,
        uint16_t startAddress
);

// load the DPDA image
DFE_Err Dfe_loadDpda(
        DFE_Handle hDfe,
        uint32_t imageSize,
        DFE_RegPair *imagePtr
);

// write DPDA scalar register
DFE_Err Dfe_writeDpdaScalar(
                        DFE_Handle hDfe,
                        uint8_t scalarId,
                        float iScalar,
                        float qScalar
);

// write value to DPDA IG register file
DFE_Err Dfe_writeDpdaIg(
                   DFE_Handle hDfe,
                   uint8_t igId,
                   uint32_t ig
);

// write samples to capture buffer
DFE_Err Dfe_writeDpdaSamples(
                         DFE_Handle hDfe,
                         DfeFl_CbBuf cbBufId,
                         uint8_t fbFlag,
                         uint16_t cbLength,
                         DfeFl_CbComplexInt *cbTemp,
                         DFE_CbData *cbData
);

DFE_Err Dfe_readDpdaScalar(
                      DFE_Handle hDfe,
                      uint8_t scalarId,
                      float *iScalarPtr,
                      float *qScalarPtr
);

DFE_Err Dfe_readDpdaIg(
                  DFE_Handle hDfe,
                  uint8_t igId,
                  uint32_t *igPtr
);

DFE_Err Dfe_readDpdaParams(
                      DFE_Handle hDfe,
                      uint16_t lineId,
                      uint16_t lineNum,
                      float *paramTbl
);

/* Reset DFE Errors and Alarms counters */
void Dfe_resetException(
        DFE_Handle  hDfe
);

/* Capture and copy DFE Errors and Alarms counters */
void Dfe_captureException (
        DFE_Handle  hDfe,
        DFE_EeCountObj *capturePtr
);

/* Enable  DFE Errors and Alarms */
DFE_Err Dfe_enableException(
        DFE_Handle  hDfe
);

/* Disable  DFE Errors and Alarms */
DFE_Err Dfe_disableException(
        DFE_Handle  hDfe
);

/* Get Dfe Errors and Alarms status and clear */
DFE_Err Dfe_getException(
        DFE_Handle  hDfe
);

void Dfe_printException(
        DFE_Handle  hDfe
);

DFE_Err Dfe_enableCbException(
        DFE_Handle  hDfe
);

DFE_Err Dfe_disableCbException(
        DFE_Handle  hDfe
);

DFE_Err Dfe_enableCppDmaDoneException(
        DFE_Handle  hDfe,
        uint32_t    cppDmaChannelNum
);

DFE_Err Dfe_disableCppDmaDoneException(
        DFE_Handle  hDfe,
        uint32_t    cppDmaChannelNum
);

#ifdef __cplusplus
}
#endif

#endif /* __DFE_DRV_H__ */
