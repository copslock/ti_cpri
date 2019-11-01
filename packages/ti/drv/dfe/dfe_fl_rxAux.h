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

/** @file dfe_fl_rxAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_RX CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 *  @date    26 Oct, 2012
 *  @author  Richard Jiang
 *  @version 0.0.1
 */
#ifndef _DFE_FL_RXAUX_H_
#define _DFE_FL_RXAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_rx.h>

/** ============================================================================
 *   @n@b dfeFl_RxConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_INITS_REG_INIT_CLK_GATE
 *       DFE_RX_INITS_REG_INITS_SSEL
 *       DFE_RX_INITS_REG_INIT_STATE
 *       DFE_RX_INITS_REG_CLEAR_DATA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxConfigInits(DfeFl_RxHandle hDfeRx, DfeFl_SublkInitsConfig * arg)
{
    uint32_t data = hDfeRx->regs->inits;
    
    CSL_FINS(data, DFE_RX_INITS_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_RX_INITS_REG_INIT_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_RX_INITS_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_RX_INITS_REG_CLEAR_DATA, arg->clearData);
    
    hDfeRx->regs->inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_RxSetSiggenSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         ssel    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_TEST_SSEL_REG_SIGGEN_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetSiggenSsel(DfeFl_RxHandle hDfeRx, uint32_t ssel)
{
    CSL_FINS(hDfeRx->regs->test_ssel, DFE_RX_TEST_SSEL_REG_SIGGEN_SSEL, ssel);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         ssel    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_TEST_SSEL_REG_CHKSUM_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetChksumSsel(DfeFl_RxHandle hDfeRx, uint32_t ssel)
{
    CSL_FINS(hDfeRx->regs->test_ssel, DFE_RX_TEST_SSEL_REG_CHKSUM_SSEL, ssel);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetSiggenMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_SIGGEN_I_MODE_REG_SIGGEN_I_RAMP
 *       DFE_RX_SIGGEN_I_MODE_REG_SIGGEN_I_FRAME
 *       DFE_RX_SIGGEN_I_MODE_REG_SIGGEN_I_FRAME_LEN
 *       DFE_RX_SIGGEN_I_MODE_REG_SIGGEN_I_ENABLE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetSiggenMode(DfeFl_RxHandle hDfeRx, DfeFl_RxSiggenMode *arg)
{
    volatile uint32_t *regs;
    
    if(arg->sigGen == DFE_FL_RX_SIGGEN_I)
        regs = &hDfeRx->regs->siggen_i_mode;
    else
        regs = &hDfeRx->regs->siggen_q_mode;
        
    regs[0] = CSL_FMK(DFE_RX_SIGGEN_I_MODE_REG_SIGGEN_I_FRAME_LEN, arg->frameLen)
            | CSL_FMK(DFE_RX_SIGGEN_I_MODE_REG_SIGGEN_I_RAMP, arg->ramp)
            | CSL_FMK(DFE_RX_SIGGEN_I_MODE_REG_SIGGEN_I_FRAME, arg->frame)
            | CSL_FMK(DFE_RX_SIGGEN_I_MODE_REG_SIGGEN_I_ENABLE, arg->enable);    
}

/** ============================================================================
 *   @n@b dfeFl_RxConfigSiggenRamp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_SIGGEN_I_RAMP_STOP_W0_REG_SIGGEN_I_RAMP_STOP_15_0
 *       DFE_RX_SIGGEN_I_RAMP_INC_W0_REG_SIGGEN_I_RAMP_INC_15_0
 *       DFE_RX_SIGGEN_I_RAMP_START_W0_REG_SIGGEN_I_RAMP_START_15_0
 *       DFE_RX_SIGGEN_I_RAMP_INC_W1_REG_SIGGEN_I_RAMP_INC_31_16
 *       DFE_RX_SIGGEN_I_RAMP_START_W1_REG_SIGGEN_I_RAMP_START_31_16
 *       DFE_RX_SIGGEN_I_RAMP_STOP_W1_REG_SIGGEN_I_RAMP_STOP_31_16
 *       DFE_RX_SIGGEN_I_PULSE_WIDTH_REG_SIGGEN_I_PULSE_WIDTH
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxConfigSiggenRamp(DfeFl_RxHandle hDfeRx, DfeFl_RxSiggenRampConfig *arg)
{
    volatile uint32_t *regs;
    
    if(arg->sigGen == DFE_FL_RX_SIGGEN_I)
        regs = &hDfeRx->regs->siggen_i_ramp_start_w0;
    else
        regs = &hDfeRx->regs->siggen_q_ramp_start_w0;
    
    // ramp start    
    regs[0] = CSL_FMK(DFE_RX_SIGGEN_I_RAMP_START_W0_REG_SIGGEN_I_RAMP_START_15_0, arg->rampStart);
    regs[1] = CSL_FMK(DFE_RX_SIGGEN_I_RAMP_START_W1_REG_SIGGEN_I_RAMP_START_31_16, arg->rampStart >> 16);
    
    // ramp stop    
    regs[2] = CSL_FMK(DFE_RX_SIGGEN_I_RAMP_STOP_W0_REG_SIGGEN_I_RAMP_STOP_15_0, arg->rampStop);
    regs[3] = CSL_FMK(DFE_RX_SIGGEN_I_RAMP_STOP_W1_REG_SIGGEN_I_RAMP_STOP_31_16, arg->rampStop >> 16);

    // ramp increment    
    regs[4] = CSL_FMK(DFE_RX_SIGGEN_I_RAMP_INC_W0_REG_SIGGEN_I_RAMP_INC_15_0, arg->rampIncr);
    regs[5] = CSL_FMK(DFE_RX_SIGGEN_I_RAMP_INC_W1_REG_SIGGEN_I_RAMP_INC_31_16, arg->rampIncr >> 16);
    
    // pulse width
    regs[6]= CSL_FMK(DFE_RX_SIGGEN_I_PULSE_WIDTH_REG_SIGGEN_I_PULSE_WIDTH, arg->pulseWidth);
}

/** ============================================================================
 *   @n@b dfeFl_RxConfigChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_CHKSUM_CHAN_SEL_REG_LATENCY_MODE_CHAN_SEL
 *       DFE_RX_CHKSUM_SIG_LEN_REG_LATENCY_MODE_SIGNAL_LEN
 *       DFE_RX_CHKSUM_MODE_REG_CHKSUM_MODE
 *       DFE_RX_CHKSUM_MODE_REG_LATENCY_MODE_STABLE_LEN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxConfigChksum(DfeFl_RxHandle hDfeRx, DfeFl_RxChksumConfig *arg)
{
    hDfeRx->regs->chksum_mode = CSL_FMK(DFE_RX_CHKSUM_MODE_REG_CHKSUM_MODE, arg->chksumMode)
                              | CSL_FMK(DFE_RX_CHKSUM_MODE_REG_LATENCY_MODE_STABLE_LEN, arg->latencyMode.stableLen);
    hDfeRx->regs->chksum_sig_len = CSL_FMK(DFE_RX_CHKSUM_SIG_LEN_REG_LATENCY_MODE_SIGNAL_LEN, arg->latencyMode.signalLen);                                  
    hDfeRx->regs->chksum_chan_sel = CSL_FMK(DFE_RX_CHKSUM_CHAN_SEL_REG_LATENCY_MODE_CHAN_SEL, arg->latencyMode.chanSel);                                  
}

/** ============================================================================
 *   @n@b dfeFl_RxQueryChksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxQueryChksumResult(DfeFl_RxHandle hDfeRx, uint32_t *arg)
{
    *arg = (hDfeRx->regs->chksum_w1 << 16) | (hDfeRx->regs->chksum_w0);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetTopTestCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_TESTBUS_REG_TOP_TEST_CTRL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetTopTestCtrl(DfeFl_RxHandle hDfeRx, DfeFl_RxTestCtrl arg)
{
	CSL_FINS(hDfeRx->regs->testbus, DFE_RX_TESTBUS_REG_TOP_TEST_CTRL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetImbTestCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_TESTBUS_REG_IMB_TEST_CTRL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxSetImbTestCtrl(DfeFl_RxHandle hDfeRx, DfeFl_RxImbTestCtrl arg)
{
	CSL_FINS(hDfeRx->regs->testbus, DFE_RX_TESTBUS_REG_IMB_TEST_CTRL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetFeagcDcTestCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_FEAGC_DC_TESTBUS_REG_FEAGC_DC_TEST_CTRL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxSetFeagcDcTestCtrl(DfeFl_RxHandle hDfeRx, DfeFl_RxFeagcDcTestCtrl arg)
{
	CSL_FINS(hDfeRx->regs->feagc_dc_testbus, DFE_RX_FEAGC_DC_TESTBUS_REG_FEAGC_DC_TEST_CTRL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_RxEnablePowmtrInterrupt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxEnablePowmtrInterrupt(DfeFl_RxHandle hDfeRx, uint32_t arg)
{
    CSL_FINSR(hDfeRx->regs->interrupt_mask, arg, arg, 1);
}

/** ============================================================================
 *   @n@b dfeFl_RxDisablePowmtrInterrupt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxDisablePowmtrInterrupt(DfeFl_RxHandle hDfeRx, uint32_t arg)
{
    CSL_FINSR(hDfeRx->regs->interrupt_mask, arg, arg, 0);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetForcePowmtrInterrupt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetForcePowmtrInterrupt(DfeFl_RxHandle hDfeRx, uint32_t arg)
{
    CSL_FINSR(hDfeRx->regs->interrupt_force, arg, arg, 1);
}

/** ============================================================================
 *   @n@b dfeFl_RxClearForcePowmtrInterrupt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxClearForcePowmtrInterrupt(DfeFl_RxHandle hDfeRx, uint32_t arg)
{
    CSL_FINSR(hDfeRx->regs->interrupt_force, arg, arg, 0);
}

/** ============================================================================
 *   @n@b dfeFl_RxClearPowmtrInterruptStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         powmtr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxClearPowmtrInterruptStatus(DfeFl_RxHandle hDfeRx, uint32_t powmtr)
{
    hDfeRx->regs->interrupt_service = ~(1u << powmtr);
}

/** ============================================================================
 *   @n@b dfeFl_RxQueryPowmtrInterruptStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxQueryPowmtrInterruptStatus(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGeneric *arg)
{
    arg->data = CSL_FEXTR(hDfeRx->regs->interrupt_service, arg->powmtr, arg->powmtr);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetPowmtrSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetPowmtrSsel(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGeneric *arg)
{
    uint32_t b = arg->powmtr * 4;
    CSL_FINSR(hDfeRx->regs->pm_ssel, b+3, b, arg->data);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetPowmtrHandshakeDone
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_PM_HANDSHAKE_REG_DONE3
 *       DFE_RX_PM_HANDSHAKE_REG_DONE2
 *       DFE_RX_PM_HANDSHAKE_REG_DONE1
 *       DFE_RX_PM_HANDSHAKE_REG_DONE0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetPowmtrHandshakeDone(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGeneric *arg)
{
    switch(arg->powmtr)
    {
    case DFE_FL_RX_POWMTR_0:
        CSL_FINS(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_DONE0, arg->data);
        break;
    case DFE_FL_RX_POWMTR_1:
    	CSL_FINS(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_DONE1, arg->data);
        break;
    case DFE_FL_RX_POWMTR_2:
    	CSL_FINS(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_DONE2, arg->data);
        break;
    case DFE_FL_RX_POWMTR_3:
    	CSL_FINS(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_DONE3, arg->data);
        break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_RxSetPowmtrHandshakeReadReq
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_PM_HANDSHAKE_REG_READ_REQ0
 *       DFE_RX_PM_HANDSHAKE_REG_READ_REQ1
 *       DFE_RX_PM_HANDSHAKE_REG_READ_REQ2
 *       DFE_RX_PM_HANDSHAKE_REG_READ_REQ3
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetPowmtrHandshakeReadReq(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGeneric *arg)
{
    switch(arg->powmtr)
    {
    case DFE_FL_RX_POWMTR_0:
        CSL_FINS(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_READ_REQ0, arg->data);
        break;
    case DFE_FL_RX_POWMTR_1:
    	CSL_FINS(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_READ_REQ1, arg->data);
        break;
    case DFE_FL_RX_POWMTR_2:
    	CSL_FINS(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_READ_REQ2, arg->data);
        break;
    case DFE_FL_RX_POWMTR_3:
    	CSL_FINS(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_READ_REQ3, arg->data);
        break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_RxGetPowmtrHandshakeReadMiss
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_RX_PM_HANDSHAKE_REG_MISSED0
 *       DFE_RX_PM_HANDSHAKE_REG_MISSED1
 *       DFE_RX_PM_HANDSHAKE_REG_MISSED2
 *       DFE_RX_PM_HANDSHAKE_REG_MISSED3
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxGetPowmtrHandshakeReadMiss(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGeneric *arg)
{
    switch(arg->powmtr)
    {
    case DFE_FL_RX_POWMTR_0:
    	arg->data = CSL_FEXT(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_MISSED0);
        break;
    case DFE_FL_RX_POWMTR_1:
    	arg->data = CSL_FEXT(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_MISSED1);
        break;
    case DFE_FL_RX_POWMTR_2:
    	arg->data = CSL_FEXT(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_MISSED2);
        break;
    case DFE_FL_RX_POWMTR_3:
    	arg->data = CSL_FEXT(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_MISSED3);
        break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_RxGetPowmtrHandshakeReadAck
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_RX_PM_HANDSHAKE_REG_READ_ACK1
 *       DFE_RX_PM_HANDSHAKE_REG_READ_ACK0
 *       DFE_RX_PM_HANDSHAKE_REG_READ_ACK3
 *       DFE_RX_PM_HANDSHAKE_REG_READ_ACK2
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxGetPowmtrHandshakeReadAck(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGeneric *arg)
{
    switch(arg->powmtr)
    {
    case DFE_FL_RX_POWMTR_0:
    	arg->data = CSL_FEXT(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_READ_ACK0);
        break;
    case DFE_FL_RX_POWMTR_1:
    	arg->data = CSL_FEXT(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_READ_ACK1);
        break;
    case DFE_FL_RX_POWMTR_2:
    	arg->data = CSL_FEXT(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_READ_ACK2);
        break;
    case DFE_FL_RX_POWMTR_3:
    	arg->data = CSL_FEXT(hDfeRx->regs->pm_handshake, DFE_RX_PM_HANDSHAKE_REG_READ_ACK3);
        break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_RxConfigPowmtrGlobal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_PM_HIST_ONE_THRESH_W1_REG_HIST_ONE_THRESH_31_16
 *       DFE_RX_PM_HIST_TWO_THRESH_W0_REG_HIST_TWO_THRESH_15_0
 *       DFE_RX_PM_MODE_REG_READ_MODE
 *       DFE_RX_PM_HIST_ONE_THRESH_W0_REG_HIST_ONE_THRESH_15_0
 *       DFE_RX_PM_HIST_TWO_THRESH_W1_REG_HIST_TWO_THRESH_31_16
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxConfigPowmtrGlobal(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGlobalConfig *arg)
{
    CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_READ_MODE, arg->readMode);
    hDfeRx->regs->pm_hist_one_thresh_w0 = CSL_FMK(DFE_RX_PM_HIST_ONE_THRESH_W0_REG_HIST_ONE_THRESH_15_0, arg->histThresh1);
    hDfeRx->regs->pm_hist_one_thresh_w1 = CSL_FMK(DFE_RX_PM_HIST_ONE_THRESH_W1_REG_HIST_ONE_THRESH_31_16, arg->histThresh1 >> 16);
    hDfeRx->regs->pm_hist_two_thresh_w0 = CSL_FMK(DFE_RX_PM_HIST_TWO_THRESH_W0_REG_HIST_TWO_THRESH_15_0, arg->histThresh2);
    hDfeRx->regs->pm_hist_two_thresh_w1 = CSL_FMK(DFE_RX_PM_HIST_TWO_THRESH_W1_REG_HIST_TWO_THRESH_31_16, arg->histThresh2 >> 16);
}

/** ============================================================================
 *   @n@b dfeFl_RxConfigPowmtr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_PM_NSAMP_W0_REG_NSAMP_15_0
 *       DFE_RX_PM_INTERVAL_W1_REG_INTERVAL_31_16
 *       DFE_RX_PM_SYNC_DELAY_REG_SYNC_DELAY
 *       DFE_RX_PM_INTERVAL_W0_REG_INTERVAL_15_0
 *       DFE_RX_PM_NSAMP_W1_REG_NSAMP_31_16
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxConfigPowmtr(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrConfig *arg)
{
    volatile uint32_t *regs_syncDelay = &hDfeRx->regs->pm_sync_delay[arg->powmtr];
    volatile uint32_t *regs_nsamp = &hDfeRx->regs->pm_nsamp[arg->powmtr].w0;
    volatile uint32_t *regs_interval = &hDfeRx->regs->pm_interval[arg->powmtr].w0;
    
    // sync delay
    regs_syncDelay[0] = CSL_FMK(DFE_RX_PM_SYNC_DELAY_REG_SYNC_DELAY, arg->syncDelay);
    // nsamp
    regs_nsamp[0] = CSL_FMK(DFE_RX_PM_NSAMP_W0_REG_NSAMP_15_0, arg->nSamples);
    regs_nsamp[1] = CSL_FMK(DFE_RX_PM_NSAMP_W1_REG_NSAMP_31_16, arg->nSamples >> 16);
    // interval
    regs_interval[0] = CSL_FMK(DFE_RX_PM_INTERVAL_W0_REG_INTERVAL_15_0, arg->interval);
    regs_interval[1] = CSL_FMK(DFE_RX_PM_INTERVAL_W1_REG_INTERVAL_31_16, arg->interval >> 16);    
}

CSL_IDEF_INLINE void
dfeFl_RxGetPowmtr(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrConfig *arg)
{
    volatile uint32_t *regs_syncDelay = &hDfeRx->regs->pm_sync_delay[arg->powmtr];
    volatile uint32_t *regs_nsamp = &hDfeRx->regs->pm_nsamp[arg->powmtr].w0;
    volatile uint32_t *regs_interval = &hDfeRx->regs->pm_interval[arg->powmtr].w0;

    // sync delay
    arg->syncDelay = CSL_FEXT(regs_syncDelay[0], DFE_RX_PM_SYNC_DELAY_REG_SYNC_DELAY);
    // nsamp
    arg->nSamples = CSL_FEXT(regs_nsamp[1], DFE_RX_PM_NSAMP_W1_REG_NSAMP_31_16);
    arg->nSamples = (arg->nSamples << 16) + CSL_FEXT(regs_nsamp[0], DFE_RX_PM_NSAMP_W0_REG_NSAMP_15_0);
    // interval
    arg->interval = CSL_FEXT(regs_interval[1], DFE_RX_PM_INTERVAL_W1_REG_INTERVAL_31_16);
    arg->interval = (arg->interval << 16) + CSL_FEXT(regs_interval[0], DFE_RX_PM_INTERVAL_W0_REG_INTERVAL_15_0);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetOneShotMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_PM_MODE_REG_ONE_SHOT_MODE2
 *       DFE_RX_PM_MODE_REG_ONE_SHOT_MODE3
 *       DFE_RX_PM_MODE_REG_ONE_SHOT_MODE0
 *       DFE_RX_PM_MODE_REG_ONE_SHOT_MODE1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetOneShotMode(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGeneric *arg)
{
    switch(arg->powmtr)
    {
    case DFE_FL_RX_POWMTR_0:
        CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_ONE_SHOT_MODE0, arg->data);
        break;
    case DFE_FL_RX_POWMTR_1:
    	CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_ONE_SHOT_MODE1, arg->data);
        break;
    case DFE_FL_RX_POWMTR_2:
    	CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_ONE_SHOT_MODE2, arg->data);
        break;
    case DFE_FL_RX_POWMTR_3:
    	CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_ONE_SHOT_MODE3, arg->data);
        break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_RxSetMeterMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_PM_MODE_REG_METER_MODE1
 *       DFE_RX_PM_MODE_REG_METER_MODE0
 *       DFE_RX_PM_MODE_REG_METER_MODE3
 *       DFE_RX_PM_MODE_REG_METER_MODE2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxSetMeterMode(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrGeneric *arg)
{
    switch(arg->powmtr)
    {
    case DFE_FL_RX_POWMTR_0:
        CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_METER_MODE0, arg->data);
        break;
    case DFE_FL_RX_POWMTR_1:
    	CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_METER_MODE1, arg->data);
        break;
    case DFE_FL_RX_POWMTR_2:
    	CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_METER_MODE2, arg->data);
        break;
    case DFE_FL_RX_POWMTR_3:
    	CSL_FINS(hDfeRx->regs->pm_mode, DFE_RX_PM_MODE_REG_METER_MODE3, arg->data);
        break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_RxQueryPowmtrResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_RX_PM_POWER_W3_REG_POWER_CNT
 *       DFE_RX_PM_POWER_W3_REG_POWER_59_48
 *       DFE_RX_PM_MAGSQ_W2_REG_MAGSQ_CNT
 *       DFE_RX_PM_HIST_TWO_W1_REG_HIST_TWO_23_16
 *       DFE_RX_PM_HIST_ONE_W1_REG_HIST_ONE_CNT
 *       DFE_RX_PM_HIST_ONE_W1_REG_HIST_ONE_23_16
 *       DFE_RX_PM_HIST_TWO_W1_REG_HIST_TWO_CNT
 *       DFE_RX_PM_MAGSQ_W2_REG_MAGSQ_35_32
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxQueryPowmtrResult(DfeFl_RxHandle hDfeRx, DfeFl_RxPowmtrResult *arg)
{
    volatile uint32_t *regs_power = &hDfeRx->regs->pm_power[arg->powmtr].w0;
    volatile uint32_t *regs_magsq = &hDfeRx->regs->pm_magsq[arg->powmtr].w0;
    volatile uint32_t *regs_hist1 = &hDfeRx->regs->pm_hist_one[arg->powmtr].w0;
    volatile uint32_t *regs_hist2 = &hDfeRx->regs->pm_hist_two[arg->powmtr].w0;
    
    CSL_Uint64  u64Data;
    uint32_t cnt, data;
    
    arg->valid = 0;
    
    // read power
    cnt = CSL_FEXT(regs_power[3], DFE_RX_PM_POWER_W3_REG_POWER_CNT);
    data = regs_power[0] | (regs_power[1] << 16);
    u64Data = regs_power[2] | (CSL_FEXT(regs_power[3], DFE_RX_PM_POWER_W3_REG_POWER_59_48) << 16);
    arg->power = (u64Data << 32) | data;
    if(cnt == CSL_FEXT(regs_power[3], DFE_RX_PM_POWER_W3_REG_POWER_CNT))
    {
        arg->valid |= DFE_FL_RX_POWMTR_RESULT_POWER_VALID;
    }
    // read magsq
    cnt = CSL_FEXT(regs_magsq[2], DFE_RX_PM_MAGSQ_W2_REG_MAGSQ_CNT);
    data = regs_magsq[0] | (regs_magsq[1] << 16);
    u64Data = CSL_FEXT(regs_magsq[2], DFE_RX_PM_MAGSQ_W2_REG_MAGSQ_35_32);
    arg->magsq = (u64Data << 32) | data;
    if(cnt == CSL_FEXT(regs_magsq[2], DFE_RX_PM_MAGSQ_W2_REG_MAGSQ_CNT))
    {
        arg->valid |= DFE_FL_RX_POWMTR_RESULT_MAGSQ_VALID;
    }
    // read hist count 1
    cnt = CSL_FEXT(regs_hist1[1], DFE_RX_PM_HIST_ONE_W1_REG_HIST_ONE_CNT);
    arg->histcnt1 = regs_hist1[0] | (CSL_FEXT(regs_hist1[1], DFE_RX_PM_HIST_ONE_W1_REG_HIST_ONE_23_16) << 16);
    if(cnt == CSL_FEXT(regs_hist1[1], DFE_RX_PM_HIST_ONE_W1_REG_HIST_ONE_CNT))
    {
        arg->valid |= DFE_FL_RX_POWMTR_RESULT_HISTCNT1_VALID;
    }
    // read hist count 2
    cnt = CSL_FEXT(regs_hist2[1], DFE_RX_PM_HIST_TWO_W1_REG_HIST_TWO_CNT);
    arg->histcnt2 = regs_hist2[0] | (CSL_FEXT(regs_hist2[1], DFE_RX_PM_HIST_TWO_W1_REG_HIST_TWO_23_16) << 16);
    if(cnt == CSL_FEXT(regs_hist2[1], DFE_RX_PM_HIST_TWO_W1_REG_HIST_TWO_CNT))
    {
        arg->valid |= DFE_FL_RX_POWMTR_RESULT_HISTCNT2_VALID;
    }
}

/** ============================================================================
 *   @n@b dfeFl_RxSetSwitchBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_BYPASS_REG_SW_BYPASS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetSwitchBypass(DfeFl_RxHandle hDfeRx, uint32_t arg)
{
    CSL_FINS(hDfeRx->regs->bypass, DFE_RX_BYPASS_REG_SW_BYPASS, arg);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetNcoSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_NCO_SSEL_REG_DITHER_SSEL
 *       DFE_RX_NCO_SSEL_REG_FREQ_SSEL
 *       DFE_RX_NCO_SSEL_REG_PHASE_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxSetNcoSsel(DfeFl_RxHandle hDfeRx, DfeFl_RxNcoSsel *arg)
{
    hDfeRx->regs->nco_ssel = CSL_FMK(DFE_RX_NCO_SSEL_REG_DITHER_SSEL, arg->sselDither)
                           | CSL_FMK(DFE_RX_NCO_SSEL_REG_PHASE_SSEL, arg->sselPhase)
                           | CSL_FMK(DFE_RX_NCO_SSEL_REG_FREQ_SSEL, arg->sselFreq);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetNcoBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_BYPASS_REG_NCO_BYPASS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetNcoBypass(DfeFl_RxHandle hDfeRx, uint32_t arg)
{
    CSL_FINS(hDfeRx->regs->bypass, DFE_RX_BYPASS_REG_NCO_BYPASS, arg);
}

/** ============================================================================
 *   @n@b dfeFl_RxGetNcoBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_RX_BYPASS_REG_NCO_BYPASS
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxGetNcoBypass(DfeFl_RxHandle hDfeRx, uint32_t *arg)
{
    *arg = CSL_FEXT(hDfeRx->regs->bypass, DFE_RX_BYPASS_REG_NCO_BYPASS);
}

/** ============================================================================
 *   @n@b dfeFl_RxGetNcoSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_RX_NCO_SSEL_REG_DITHER_SSEL
 *       DFE_RX_NCO_SSEL_REG_FREQ_SSEL
 *       DFE_RX_NCO_SSEL_REG_PHASE_SSEL
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxGetNcoSsel(DfeFl_RxHandle hDfeRx, DfeFl_RxNcoSsel *arg)
{
    uint32_t data = hDfeRx->regs->nco_ssel;
    arg->sselDither = CSL_FEXT(data, DFE_RX_NCO_SSEL_REG_DITHER_SSEL);
    arg->sselPhase = CSL_FEXT(data, DFE_RX_NCO_SSEL_REG_PHASE_SSEL);
    arg->sselFreq = CSL_FEXT(data, DFE_RX_NCO_SSEL_REG_FREQ_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_RxEnableNcoDither
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxEnableNcoDither(DfeFl_RxHandle hDfeRx, DfeFl_RxNcoDitherConfig *arg)
{
    CSL_FINSR(hDfeRx->regs->nco_dither_enable, arg->nco, arg->nco, arg->enable);
}

/** ============================================================================
 *   @n@b dfeFl_RxConfigNcoFreq
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxConfigNcoFreq(DfeFl_RxHandle hDfeRx, DfeFl_RxNcoFreqWord *arg)
{
    volatile uint32_t *regs = &hDfeRx->regs->nco_freq_word[arg->nco].w0;
    
    regs[0] = arg->freqWord & 0xffffu;
    regs[1] = (arg->freqWord >> 16) & 0xffffu;
    regs[2] = (arg->freqWord >> 32) & 0xffffu;    
}

/** ============================================================================
 *   @n@b dfeFl_RxSetEqrSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetEqrSsel(DfeFl_RxHandle hDfeRx, DfeFl_RxEqrGeneric *arg)
{
    uint32_t b = arg->eqr * 4;
    
    CSL_FINSR(hDfeRx->regs->eqr_ssel, b+3, b, arg->data);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetEqrBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_BYPASS_REG_EQR_BYPASS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetEqrBypass(DfeFl_RxHandle hDfeRx, uint32_t arg)
{
    CSL_FINS(hDfeRx->regs->bypass, DFE_RX_BYPASS_REG_EQR_BYPASS, arg);
}

/** ============================================================================
 *   @n@b dfeFl_RxConfigEqrTaps
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_RxConfigEqrTaps(DfeFl_RxHandle hDfeRx, DfeFl_RxEqrTapsConfig *arg)
{
    int i;
    volatile uint32_t *regs;
    
    switch(arg->eqr)
    {
    case DFE_FL_RX_EQR_0:
        regs = &hDfeRx->regs->eqr_taps_ii0[0];
        break;
    case DFE_FL_RX_EQR_1:
        regs = &hDfeRx->regs->eqr_taps_ii1[0];
        break;
    case DFE_FL_RX_EQR_2:
        regs = &hDfeRx->regs->eqr_taps_ii2[0];
        break;
    case DFE_FL_RX_EQR_3:
        regs = &hDfeRx->regs->eqr_taps_ii3[0];
        break;
    default:
    	return;
    }
    
    for(i = 0; i < DFE_FL_RX_EQR_LEN; i++)
    {
        regs[i] = arg->iiTaps[i];
    }
    for(i = 0; i < DFE_FL_RX_EQR_LEN; i++)
    {
        regs[i] = arg->iqTaps[i];
    }
    for(i = 0; i < DFE_FL_RX_EQR_LEN; i++)
    {
        regs[i] = arg->qiTaps[i];
    }
    for(i = 0; i < DFE_FL_RX_EQR_LEN; i++)
    {
        regs[i] = arg->qqTaps[i];
    }
}

/** ============================================================================
 *   @n@b dfeFl_RxSetEqrShift
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetEqrShift(DfeFl_RxHandle hDfeRx, DfeFl_RxEqrGeneric *arg)
{
    uint32_t b = arg->eqr * 4;
    
    CSL_FINSR(hDfeRx->regs->eqr_shift, b+2, b, arg->data);
}

/** ============================================================================
 *   @n@b dfeFl_RxGetEqrShift
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxGetEqrShift(DfeFl_RxHandle hDfeRx, DfeFl_RxEqrGeneric *arg)
{
    uint32_t b = arg->eqr * 4;
    
    arg->data = CSL_FEXTR(hDfeRx->regs->eqr_shift, b+2, b);
}

/** ============================================================================
 *   @n@b dfeFl_RxSetDkaccSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         dkacc    [add content]
         ssel    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_DC_SSEL_REG_DKACC_SSEL2
 *       DFE_RX_DC_SSEL_REG_DKACC_SSEL3
 *       DFE_RX_DC_SSEL_REG_DKACC_SSEL0
 *       DFE_RX_DC_SSEL_REG_DKACC_SSEL1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetDkaccSsel(DfeFl_RxHandle hDfeRx, uint32_t dkacc, uint32_t ssel)
{
    if(dkacc == DFE_FL_RX_DC_ALL)
    {
        hDfeRx->regs->dc_ssel = CSL_FMK(DFE_RX_DC_SSEL_REG_DKACC_SSEL0, ssel) \
                            | CSL_FMK(DFE_RX_DC_SSEL_REG_DKACC_SSEL1, ssel) \
                            | CSL_FMK(DFE_RX_DC_SSEL_REG_DKACC_SSEL2, ssel) \
                            | CSL_FMK(DFE_RX_DC_SSEL_REG_DKACC_SSEL3, ssel);
    }
    else if(dkacc <= DFE_FL_RX_DC_3)
    {
        uint32_t b = dkacc*4;
        CSL_FINSR(hDfeRx->regs->dc_ssel, b+3, b, ssel);
    }        
}

/** ============================================================================
 *   @n@b dfeFl_RxSetDcGsgSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx    [add content]
         ssel    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_RX_GSG_SSEL_REG_DC_GSG_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_RxSetDcGsgSsel(DfeFl_RxHandle hDfeRx, uint32_t ssel)
{
    CSL_FINS(hDfeRx->regs->gsg_ssel, DFE_RX_GSG_SSEL_REG_DC_GSG_SSEL, ssel);
}

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_RXAUX_H_ */
