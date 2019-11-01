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

/** @file dfe_fl_fbAux.h
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
#ifndef _DFE_FL_FBAUX_H_
#define _DFE_FL_FBAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_fb.h>

/** ============================================================================
 *   @n@b dfeFl_FbConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_INITS_REG_INIT_CLK_GATE
 *       DFE_FB_INITS_REG_INITS_SSEL
 *       DFE_FB_INITS_REG_INIT_STATE
 *       DFE_FB_INITS_REG_CLEAR_DATA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_FbConfigInits(DfeFl_FbHandle hDfeFb, DfeFl_SublkInitsConfig * arg)
{
    uint32_t data = hDfeFb->regs->inits;
    
    CSL_FINS(data, DFE_FB_INITS_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_FB_INITS_REG_INIT_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_FB_INITS_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_FB_INITS_REG_CLEAR_DATA, arg->clearData);
    
    hDfeFb->regs->inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_FbGetInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_INITS_REG_INIT_CLK_GATE
 *       DFE_FB_INITS_REG_INITS_SSEL
 *       DFE_FB_INITS_REG_INIT_STATE
 *       DFE_FB_INITS_REG_CLEAR_DATA
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
dfeFl_FbGetInits(DfeFl_FbHandle hDfeFb, DfeFl_SublkInitsConfig * arg)
{
    uint32_t data = hDfeFb->regs->inits;

    arg->ssel = CSL_FEXT(data, DFE_FB_INITS_REG_INITS_SSEL);
    arg->initClkGate = CSL_FEXT(data, DFE_FB_INITS_REG_INIT_CLK_GATE);
    arg->initState = CSL_FEXT(data, DFE_FB_INITS_REG_INIT_STATE);
    arg->clearData = CSL_FEXT(data, DFE_FB_INITS_REG_CLEAR_DATA);

}

/** ============================================================================
 *   @n@b dfeFl_FbSetInitsCkendly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_INITS_REG_CKEN_DLY
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetInitsCkendly(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->inits, DFE_FB_INITS_REG_CKEN_DLY, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetInitsCkendly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_INITS_REG_CKEN_DLY
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
dfeFl_FbGetInitsCkendly(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->inits, DFE_FB_INITS_REG_CKEN_DLY);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetTestbus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_TESTBUS_REG_TOP_TEST_CTRL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetTestbus(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->testbus, DFE_FB_TESTBUS_REG_TOP_TEST_CTRL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetTestbus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_TESTBUS_REG_TOP_TEST_CTRL
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
dfeFl_FbGetTestbus(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->testbus, DFE_FB_TESTBUS_REG_TOP_TEST_CTRL);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetDcTestbus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_TESTBUS_REG_DC_TEST_CTRL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDcTestbus(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->dc_testbus, DFE_FB_DC_TESTBUS_REG_DC_TEST_CTRL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDcTestbus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_TESTBUS_REG_DC_TEST_CTRL
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
dfeFl_FbGetDcTestbus(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->dc_testbus, DFE_FB_DC_TESTBUS_REG_DC_TEST_CTRL);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetSiggenSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_TEST_SSEL_REG_SIGGEN_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetSiggenSsel(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->test_ssel, DFE_FB_TEST_SSEL_REG_SIGGEN_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_TEST_SSEL_REG_CHKSUM_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetChksumSsel(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->test_ssel, DFE_FB_TEST_SSEL_REG_CHKSUM_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetSiggenMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_FRAME
 *       DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_SEED
 *       DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_FRAME_LEN
 *       DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_ENABLE
 *       DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_RAMP
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetSiggenMode(DfeFl_FbHandle hDfeFb, DfeFl_FbSiggenMode *arg)
{
    volatile uint32_t *regs;

    switch(arg->bus)
    {
    case DFE_FL_FB_SIGGEN_IBUS:
    	regs = &hDfeFb->regs->siggen_i_mode;
    	break;
    case DFE_FL_FB_SIGGEN_QBUS:
    	regs = &hDfeFb->regs->siggen_q_mode;
    	break;
    default:
    	return;
    }

    regs[0] = CSL_FMK(DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_FRAME_LEN, arg->frameLen)
    		| CSL_FMK(DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_SEED, arg->seed)
            | CSL_FMK(DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_RAMP, arg->mode)
            | CSL_FMK(DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_FRAME, arg->frame)
            | CSL_FMK(DFE_FB_SIGGEN_I_MODE_REG_SIGGEN_I_ENABLE, arg->enable);
}

/** ============================================================================
 *   @n@b dfeFl_FbCfgSiggenRamp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_SIGGEN_I_RAMP_STOP_W0_REG_SIGGEN_I_RAMP_STOP_15_0
 *       DFE_FB_SIGGEN_I_PULSE_WIDTH_REG_SIGGEN_I_PULSE_WIDTH
 *       DFE_FB_SIGGEN_I_RAMP_INC_W0_REG_SIGGEN_I_RAMP_INC_15_0
 *       DFE_FB_SIGGEN_I_RAMP_START_W1_REG_SIGGEN_I_RAMP_START_31_16
 *       DFE_FB_SIGGEN_I_RAMP_STOP_W1_REG_SIGGEN_I_RAMP_STOP_31_16
 *       DFE_FB_SIGGEN_I_RAMP_INC_W1_REG_SIGGEN_I_RAMP_INC_31_16
 *       DFE_FB_SIGGEN_I_RAMP_START_W0_REG_SIGGEN_I_RAMP_START_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbCfgSiggenRamp(DfeFl_FbHandle hDfeFb, DfeFl_FbSiggenRampCfg *arg)
{
    volatile uint32_t *regs;

    switch(arg->bus)
    {
    case DFE_FL_FB_SIGGEN_IBUS:
    	regs = &hDfeFb->regs->siggen_i_ramp_start_w0;
    	break;
    case DFE_FL_FB_SIGGEN_QBUS:
    	regs = &hDfeFb->regs->siggen_q_ramp_start_w0;
    	break;
    default:
    	return;
    }

    // ramp start
    regs[0] = CSL_FMK(DFE_FB_SIGGEN_I_RAMP_START_W0_REG_SIGGEN_I_RAMP_START_15_0, arg->rampStart);
    regs[1] = CSL_FMK(DFE_FB_SIGGEN_I_RAMP_START_W1_REG_SIGGEN_I_RAMP_START_31_16, arg->rampStart >> 16);

    // ramp stop
    regs[2] = CSL_FMK(DFE_FB_SIGGEN_I_RAMP_STOP_W0_REG_SIGGEN_I_RAMP_STOP_15_0, arg->rampStop);
    regs[3] = CSL_FMK(DFE_FB_SIGGEN_I_RAMP_STOP_W1_REG_SIGGEN_I_RAMP_STOP_31_16, arg->rampStop >> 16);

    // ramp increment
    regs[4] = CSL_FMK(DFE_FB_SIGGEN_I_RAMP_INC_W0_REG_SIGGEN_I_RAMP_INC_15_0, arg->rampInc);
    regs[5] = CSL_FMK(DFE_FB_SIGGEN_I_RAMP_INC_W1_REG_SIGGEN_I_RAMP_INC_31_16, arg->rampInc >> 16);

    // pluse width
    regs[6]= CSL_FMK(DFE_FB_SIGGEN_I_PULSE_WIDTH_REG_SIGGEN_I_PULSE_WIDTH, arg->pulseWidth);
}

/** ============================================================================
 *   @n@b dfeFl_FbCfgChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_CHKSUM_SIG_LEN_REG_LATENCY_MODE_SIGNAL_LEN
 *       DFE_FB_CHKSUM_MODE_REG_CHKSUM_MODE
 *       DFE_FB_CHKSUM_MODE_REG_LATENCY_MODE_STABLE_LEN
 *       DFE_FB_CHKSUM_CHAN_SEL_REG_LATENCY_MODE_CHAN_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbCfgChksum(DfeFl_FbHandle hDfeFb, DfeFl_FbChksumCfg *arg)
{
    hDfeFb->regs->chksum_mode = CSL_FMK(DFE_FB_CHKSUM_MODE_REG_CHKSUM_MODE, arg->chksumMode)
                              | CSL_FMK(DFE_FB_CHKSUM_MODE_REG_LATENCY_MODE_STABLE_LEN, arg->latencyMode.stableLen);
    hDfeFb->regs->chksum_sig_len = CSL_FMK(DFE_FB_CHKSUM_SIG_LEN_REG_LATENCY_MODE_SIGNAL_LEN, arg->latencyMode.signalLen);
    hDfeFb->regs->chksum_chan_sel = CSL_FMK(DFE_FB_CHKSUM_CHAN_SEL_REG_LATENCY_MODE_CHAN_SEL, arg->latencyMode.chanSel);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetChksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_CHKSUM_W1_REG_CHKSUM_31_16
 *       DFE_FB_CHKSUM_W0_REG_CHKSUM_15_0
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
dfeFl_FbGetChksumResult(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
    uint32_t data;

    data = CSL_FEXT(hDfeFb->regs->chksum_w0, DFE_FB_CHKSUM_W0_REG_CHKSUM_15_0);
    data |= CSL_FEXT(hDfeFb->regs->chksum_w1, DFE_FB_CHKSUM_W1_REG_CHKSUM_31_16) << 16;

	*arg = data;
}

/** ============================================================================
 *   @n@b dfeFl_FbSetIOMux
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_IO_CONTROL_REG_CONFIG_SELECT_MODE
 *       DFE_FB_IO_CONTROL_REG_HOST_CONFIG_SELECT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetIOMux(DfeFl_FbHandle hDfeFb, DfeFl_FbIOCtrl *arg)
{
    CSL_FINS(hDfeFb->regs->io_control, DFE_FB_IO_CONTROL_REG_HOST_CONFIG_SELECT, arg->host_cfg_select);
    CSL_FINS(hDfeFb->regs->io_control, DFE_FB_IO_CONTROL_REG_CONFIG_SELECT_MODE, arg->cfg_select_mode);
}



/** ============================================================================
 *   @n@b dfeFl_FbSetIOCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_IO_CONTROL_REG_CB_OUTPUT_SELECT
 *       DFE_FB_IO_CONTROL_REG_CONFIG_SELECT_MODE
 *       DFE_FB_IO_CONTROL_REG_HOST_CONFIG_SELECT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetIOCtrl(DfeFl_FbHandle hDfeFb, DfeFl_FbIOCtrl *arg)
{
    hDfeFb->regs->io_control= CSL_FMK(DFE_FB_IO_CONTROL_REG_CONFIG_SELECT_MODE, arg->cfg_select_mode)
                            | CSL_FMK(DFE_FB_IO_CONTROL_REG_HOST_CONFIG_SELECT, arg->host_cfg_select)
     	 	 	 	 	 	| CSL_FMK(DFE_FB_IO_CONTROL_REG_CB_OUTPUT_SELECT, arg->cb_output_select);
}

/** ============================================================================
 *   @n@b dfeFl_FbQueryGetIOControl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_IO_CONTROL_REG_CB_OUTPUT_SELECT
 *       DFE_FB_IO_CONTROL_REG_CONFIG_SELECT_MODE
 *       DFE_FB_IO_CONTROL_REG_HOST_CONFIG_SELECT
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
dfeFl_FbQueryGetIOControl(DfeFl_FbHandle hDfeFb, DfeFl_FbIOCtrl *arg)
{
	arg->cfg_select_mode = CSL_FEXT(hDfeFb->regs->io_control, DFE_FB_IO_CONTROL_REG_CONFIG_SELECT_MODE);
	arg->host_cfg_select = CSL_FEXT(hDfeFb->regs->io_control, DFE_FB_IO_CONTROL_REG_HOST_CONFIG_SELECT);
	arg->cb_output_select = CSL_FEXT(hDfeFb->regs->io_control, DFE_FB_IO_CONTROL_REG_CB_OUTPUT_SELECT);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetDcSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_SSEL_REG_DKACC_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDcSsel(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->dc_ssel, DFE_FB_DC_SSEL_REG_DKACC_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDcSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_SSEL_REG_DKACC_SSEL
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
dfeFl_FbGetDcSsel(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->dc_ssel, DFE_FB_DC_SSEL_REG_DKACC_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetDcGsgSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_GSG_SSEL_REG_GSG_UPDATE_SYNC_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDcGsgSsel(DfeFl_FbHandle hDfeFb, uint32_t ssel)
{
    CSL_FINS(hDfeFb->regs->dc_gsg_ssel, DFE_FB_DC_GSG_SSEL_REG_GSG_UPDATE_SYNC_SSEL, ssel);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDcGsgSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *   @n  see below,
 *       DFE_FB_DC_GSG_SSEL_REG_GSG_UPDATE_SYNC_SSEL
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
dfeFl_FbGetDcGsgSsel(DfeFl_FbHandle hDfeFb, uint32_t *ssel)
{
    *ssel = CSL_FEXT(hDfeFb->regs->dc_gsg_ssel, DFE_FB_DC_GSG_SSEL_REG_GSG_UPDATE_SYNC_SSEL);
}


/** ============================================================================
 *   @n@b dfeFl_FbSetDcGlobal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_GLOBAL_REG_DC_FRZ_RST_INT0
 *       DFE_FB_DC_GLOBAL_REG_DC_FREEZE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDcGlobal(DfeFl_FbHandle hDfeFb, DfeFl_FbDcGlobal *arg)
{
    hDfeFb->regs->dc_global = CSL_FMK(DFE_FB_DC_GLOBAL_REG_DC_FREEZE, arg->dc_freeze)
                            | CSL_FMK(DFE_FB_DC_GLOBAL_REG_DC_FRZ_RST_INT0, arg->dc_frz_rst_int0);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDcGlobal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_GLOBAL_REG_DC_FRZ_RST_INT0
 *       DFE_FB_DC_GLOBAL_REG_DC_FREEZE
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
dfeFl_FbGetDcGlobal(DfeFl_FbHandle hDfeFb, DfeFl_FbDcGlobal *arg)
{
	arg->dc_freeze = CSL_FEXT(hDfeFb->regs->dc_global, DFE_FB_DC_GLOBAL_REG_DC_FREEZE);
	arg->dc_frz_rst_int0 = CSL_FEXT(hDfeFb->regs->dc_global, DFE_FB_DC_GLOBAL_REG_DC_FRZ_RST_INT0);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetDcInterval
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_INTERVAL_W1_REG_DC_INTERVAL_23_16
 *       DFE_FB_DC_INTERVAL_W0_REG_DC_INTERVAL_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDcInterval(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->dc_interval_w0, DFE_FB_DC_INTERVAL_W0_REG_DC_INTERVAL_15_0, arg);
	CSL_FINS(hDfeFb->regs->dc_interval_w1, DFE_FB_DC_INTERVAL_W1_REG_DC_INTERVAL_23_16, arg >> 16);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDcInterval
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_INTERVAL_W1_REG_DC_INTERVAL_23_16
 *       DFE_FB_DC_INTERVAL_W0_REG_DC_INTERVAL_15_0
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
dfeFl_FbGetDcInterval(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->dc_interval_w1, DFE_FB_DC_INTERVAL_W1_REG_DC_INTERVAL_23_16);
	*arg = (*arg << 16) | CSL_FEXT(hDfeFb->regs->dc_interval_w0, DFE_FB_DC_INTERVAL_W0_REG_DC_INTERVAL_15_0);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetDcDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_UPDATE_DELAY_REG_DC_UPDATE_DELAY
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDcDelay(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->dc_update_delay, DFE_FB_DC_UPDATE_DELAY_REG_DC_UPDATE_DELAY, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDcDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_UPDATE_DELAY_REG_DC_UPDATE_DELAY
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
dfeFl_FbGetDcDelay(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->dc_update_delay, DFE_FB_DC_UPDATE_DELAY_REG_DC_UPDATE_DELAY);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetDcShiftMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_SHIFT_MODE_REG_DC_SHIFT
 *       DFE_FB_DC_SHIFT_MODE_REG_DC_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDcShiftMode(DfeFl_FbHandle hDfeFb, DfeFl_FbDcShiftMode *arg)
{
    hDfeFb->regs->dc_shift_mode = CSL_FMK(DFE_FB_DC_SHIFT_MODE_REG_DC_SHIFT, arg->dc_shift)
                                | CSL_FMK(DFE_FB_DC_SHIFT_MODE_REG_DC_MODE, arg->dc_mode);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDcShiftMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_SHIFT_MODE_REG_DC_SHIFT
 *       DFE_FB_DC_SHIFT_MODE_REG_DC_MODE
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
dfeFl_FbGetDcShiftMode(DfeFl_FbHandle hDfeFb, DfeFl_FbDcShiftMode *arg)
{
	arg->dc_shift = CSL_FEXT(hDfeFb->regs->dc_shift_mode, DFE_FB_DC_SHIFT_MODE_REG_DC_SHIFT);
	arg->dc_mode = CSL_FEXT(hDfeFb->regs->dc_shift_mode, DFE_FB_DC_SHIFT_MODE_REG_DC_MODE);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetDcInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_INIT_REG_DC_INIT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDcInit(DfeFl_FbHandle hDfeFb, DfeFl_FbDcInit *arg)
{
	CSL_FINS(hDfeFb->regs->dc_init[0], DFE_FB_DC_INIT_REG_DC_INIT, arg->dcinit0);
	CSL_FINS(hDfeFb->regs->dc_init[1], DFE_FB_DC_INIT_REG_DC_INIT, arg->dcinit1);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDcInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DC_INIT_REG_DC_INIT
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
dfeFl_FbGetDcInit(DfeFl_FbHandle hDfeFb, DfeFl_FbDcInit *arg)
{
	arg->dcinit0 = CSL_FEXT(hDfeFb->regs->dc_init[0], DFE_FB_DC_INIT_REG_DC_INIT);
	arg->dcinit1 = CSL_FEXT(hDfeFb->regs->dc_init[1], DFE_FB_DC_INIT_REG_DC_INIT);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetR2cSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_R2C_SSEL_REG_PHASE_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetR2cSsel(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->r2c_ssel, DFE_FB_R2C_SSEL_REG_PHASE_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetR2cSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_R2C_SSEL_REG_PHASE_SSEL
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
dfeFl_FbGetR2cSsel(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->r2c_ssel, DFE_FB_R2C_SSEL_REG_PHASE_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetR2cRealin
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN0
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN1
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN2
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN3
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN4
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetR2cRealin(DfeFl_FbHandle hDfeFb, DfeFl_FbR2cRealin *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN0, arg->data);
    	break;
    case DFE_FL_FB_BLOCK1:
    	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN1, arg->data);
    	break;
    case DFE_FL_FB_BLOCK2:
    	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN2, arg->data);
    	break;
    case DFE_FL_FB_BLOCK3:
    	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN3, arg->data);
    	break;
    case DFE_FL_FB_BLOCK4:
       	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN4, arg->data);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbGetR2cRealin
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN0
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN1
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN2
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN3
 *       DFE_FB_R2C_GLOBAL_REG_REAL_IN4
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
dfeFl_FbGetR2cRealin(DfeFl_FbHandle hDfeFb, DfeFl_FbR2cRealin *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN0);
    	break;
    case DFE_FL_FB_BLOCK1:
    	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN1);
    	break;
    case DFE_FL_FB_BLOCK2:
    	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN2);
    	break;
    case DFE_FL_FB_BLOCK3:
    	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN3);
    	break;
    case DFE_FL_FB_BLOCK4:
       	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_REAL_IN4);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbSetR2cSpecinv
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV4
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV0
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV1
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV2
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV3
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetR2cSpecinv(DfeFl_FbHandle hDfeFb, DfeFl_FbR2cSpecinv *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV0, arg->data);
    	break;
    case DFE_FL_FB_BLOCK1:
    	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV1, arg->data);
    	break;
    case DFE_FL_FB_BLOCK2:
    	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV2, arg->data);
    	break;
    case DFE_FL_FB_BLOCK3:
    	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV3, arg->data);
    	break;
    case DFE_FL_FB_BLOCK4:
       	CSL_FINS(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV4, arg->data);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbGetR2cSpecinv
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV4
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV0
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV1
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV2
 *       DFE_FB_R2C_GLOBAL_REG_SPEC_INV3
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
dfeFl_FbGetR2cSpecinv(DfeFl_FbHandle hDfeFb, DfeFl_FbR2cSpecinv *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV0);
    	break;
    case DFE_FL_FB_BLOCK1:
    	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV1);
    	break;
    case DFE_FL_FB_BLOCK2:
    	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV2);
    	break;
    case DFE_FL_FB_BLOCK3:
    	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV3);
    	break;
    case DFE_FL_FB_BLOCK4:
       	arg->data = CSL_FEXT(hDfeFb->regs->r2c_global, DFE_FB_R2C_GLOBAL_REG_SPEC_INV4);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbSetEqrSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_EQR_SSEL_REG_SSEL0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetEqrSsel(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->eqr_ssel, DFE_FB_EQR_SSEL_REG_SSEL0, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetEqrSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_EQR_SSEL_REG_SSEL0
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
dfeFl_FbGetEqrSsel(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->eqr_ssel, DFE_FB_EQR_SSEL_REG_SSEL0);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetEqrTaps
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
         blk    [add content]
         idx    [add content]
         t_ii    [add content]
         t_iq    [add content]
         t_qi    [add content]
         t_qq    [add content]
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
 *       DFE_FB_EQR_TAPS_QI0_REG_TAPS_QI0
 *       DFE_FB_EQR_TAPS_II0_REG_TAPS_II0
 *       DFE_FB_EQR_TAPS_QQ0_REG_TAPS_QQ0
 *       DFE_FB_EQR_TAPS_IQ0_REG_TAPS_IQ0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetEqrTaps(DfeFl_FbHandle hDfeFb, uint32_t blk, uint32_t idx, uint32_t t_ii, uint32_t t_iq, uint32_t t_qi, uint32_t t_qq)
{
	volatile uint32_t *regs;
	uint32_t idx_t = idx;

	switch(blk)
    {
    case DFE_FL_FB_BLOCK0:
    	regs = &hDfeFb->regs->eqr_taps_ii0[0];
    	break;
    case DFE_FL_FB_BLOCK1:
    	regs = &hDfeFb->regs->eqr_taps_ii1[0];
    	break;
    case DFE_FL_FB_BLOCK2:
    	regs = &hDfeFb->regs->eqr_taps_ii2[0];
    	break;
    case DFE_FL_FB_BLOCK3:
    	regs = &hDfeFb->regs->eqr_taps_ii3[0];
    	break;
    case DFE_FL_FB_BLOCK4:
    	regs = &hDfeFb->regs->eqr_taps_ii4[0];
       	break;
    default:
    	return;
    }
	regs[idx_t] = CSL_FMK(DFE_FB_EQR_TAPS_II0_REG_TAPS_II0, t_ii);
	idx_t += DFE_FL_FB_EQR_TAPS;
	regs[idx_t] = CSL_FMK(DFE_FB_EQR_TAPS_IQ0_REG_TAPS_IQ0, t_iq);
	idx_t += DFE_FL_FB_EQR_TAPS;
	regs[idx_t] = CSL_FMK(DFE_FB_EQR_TAPS_QI0_REG_TAPS_QI0, t_qi);
	idx_t += DFE_FL_FB_EQR_TAPS;
	regs[idx_t] = CSL_FMK(DFE_FB_EQR_TAPS_QQ0_REG_TAPS_QQ0, t_qq);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetEqrTaps
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
         blk    [add content]
         idx    [add content]
         t_ii    [add content]
         t_iq    [add content]
         t_qi    [add content]
         t_qq    [add content]
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
 *       DFE_FB_EQR_TAPS_QI0_REG_TAPS_QI0
 *       DFE_FB_EQR_TAPS_II0_REG_TAPS_II0
 *       DFE_FB_EQR_TAPS_QQ0_REG_TAPS_QQ0
 *       DFE_FB_EQR_TAPS_IQ0_REG_TAPS_IQ0
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
dfeFl_FbGetEqrTaps(DfeFl_FbHandle hDfeFb, uint32_t blk, uint32_t idx, uint32_t *t_ii, uint32_t *t_iq, uint32_t *t_qi, uint32_t *t_qq)
{
	volatile uint32_t *regs;
	uint32_t idx_t = idx;

	switch(blk)
    {
    case DFE_FL_FB_BLOCK0:
    	regs = &hDfeFb->regs->eqr_taps_ii0[0];
    	break;
    case DFE_FL_FB_BLOCK1:
    	regs = &hDfeFb->regs->eqr_taps_ii1[0];
    	break;
    case DFE_FL_FB_BLOCK2:
    	regs = &hDfeFb->regs->eqr_taps_ii2[0];
    	break;
    case DFE_FL_FB_BLOCK3:
    	regs = &hDfeFb->regs->eqr_taps_ii3[0];
    	break;
    case DFE_FL_FB_BLOCK4:
    	regs = &hDfeFb->regs->eqr_taps_ii4[0];
       	break;
    default:
    	return;
    }
	*t_ii = CSL_FEXT(regs[idx_t], DFE_FB_EQR_TAPS_II0_REG_TAPS_II0);
	idx_t += DFE_FL_FB_EQR_TAPS;
	*t_iq = CSL_FEXT(regs[idx_t], DFE_FB_EQR_TAPS_IQ0_REG_TAPS_IQ0);
	idx_t += DFE_FL_FB_EQR_TAPS;
	*t_qi = CSL_FEXT(regs[idx_t], DFE_FB_EQR_TAPS_QI0_REG_TAPS_QI0);
	idx_t += DFE_FL_FB_EQR_TAPS;
	*t_qq = CSL_FEXT(regs[idx_t], DFE_FB_EQR_TAPS_QQ0_REG_TAPS_QQ0);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetNcoSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_NCO_SSEL_REG_PHASE_SSEL
 *       DFE_FB_NCO_SSEL_REG_DITHER_SSEL
 *       DFE_FB_NCO_SSEL_REG_FREQ_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetNcoSsel(DfeFl_FbHandle hDfeFb, DfeFl_FbNcoSsel *arg)
{
    hDfeFb->regs->nco_ssel= CSL_FMK(DFE_FB_NCO_SSEL_REG_FREQ_SSEL, arg->freq)
                          | CSL_FMK(DFE_FB_NCO_SSEL_REG_PHASE_SSEL, arg->phase)
     	 	 	 	 	  | CSL_FMK(DFE_FB_NCO_SSEL_REG_DITHER_SSEL, arg->dither);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetNcoSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_NCO_SSEL_REG_PHASE_SSEL
 *       DFE_FB_NCO_SSEL_REG_DITHER_SSEL
 *       DFE_FB_NCO_SSEL_REG_FREQ_SSEL
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
dfeFl_FbGetNcoSsel(DfeFl_FbHandle hDfeFb, DfeFl_FbNcoSsel *arg)
{
	uint32_t data = hDfeFb->regs->nco_ssel;

	arg->freq = CSL_FEXT(data, DFE_FB_NCO_SSEL_REG_FREQ_SSEL);
	arg->phase = CSL_FEXT(data, DFE_FB_NCO_SSEL_REG_PHASE_SSEL);
	arg->dither = CSL_FEXT(data, DFE_FB_NCO_SSEL_REG_DITHER_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetNcoBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_NCO_BYPASS_REG_BYPASS1
 *       DFE_FB_NCO_BYPASS_REG_BYPASS0
 *       DFE_FB_NCO_BYPASS_REG_BYPASS3
 *       DFE_FB_NCO_BYPASS_REG_BYPASS2
 *       DFE_FB_NCO_BYPASS_REG_BYPASS4
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetNcoBypass(DfeFl_FbHandle hDfeFb, DfeFl_FbNcoBypass *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	CSL_FINS(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS0, arg->data);
    	break;
    case DFE_FL_FB_BLOCK1:
    	CSL_FINS(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS1, arg->data);
    	break;
    case DFE_FL_FB_BLOCK2:
    	CSL_FINS(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS2, arg->data);
    	break;
    case DFE_FL_FB_BLOCK3:
    	CSL_FINS(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS3, arg->data);
    	break;
    case DFE_FL_FB_BLOCK4:
       	CSL_FINS(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS4, arg->data);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbGetNcoBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_NCO_BYPASS_REG_BYPASS1
 *       DFE_FB_NCO_BYPASS_REG_BYPASS0
 *       DFE_FB_NCO_BYPASS_REG_BYPASS3
 *       DFE_FB_NCO_BYPASS_REG_BYPASS2
 *       DFE_FB_NCO_BYPASS_REG_BYPASS4
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
dfeFl_FbGetNcoBypass(DfeFl_FbHandle hDfeFb, DfeFl_FbNcoBypass *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	arg->data = CSL_FEXT(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS0);
    	break;
    case DFE_FL_FB_BLOCK1:
    	arg->data = CSL_FEXT(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS1);
    	break;
    case DFE_FL_FB_BLOCK2:
    	arg->data = CSL_FEXT(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS2);
    	break;
    case DFE_FL_FB_BLOCK3:
    	arg->data = CSL_FEXT(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS3);
    	break;
    case DFE_FL_FB_BLOCK4:
       	arg->data = CSL_FEXT(hDfeFb->regs->nco_bypass, DFE_FB_NCO_BYPASS_REG_BYPASS4);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbSetNcoFreq
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16
 *       DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0
 *       DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetNcoFreq(DfeFl_FbHandle hDfeFb, DfeFl_FbNcoFreq *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    {
    	CSL_FINS(hDfeFb->regs->nco_freq_word[0].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0, arg->freq_word_w0);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[0].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16, arg->freq_word_w1);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[0].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32, arg->freq_word_w2);
    	break;
    }
    case DFE_FL_FB_BLOCK1:
    {
    	CSL_FINS(hDfeFb->regs->nco_freq_word[1].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0, arg->freq_word_w0);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[1].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16, arg->freq_word_w1);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[1].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32, arg->freq_word_w2);
    	break;
    }
    case DFE_FL_FB_BLOCK2:
    {
    	CSL_FINS(hDfeFb->regs->nco_freq_word[2].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0, arg->freq_word_w0);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[2].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16, arg->freq_word_w1);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[2].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32, arg->freq_word_w2);
    	break;
    }
    case DFE_FL_FB_BLOCK3:
    {
    	CSL_FINS(hDfeFb->regs->nco_freq_word[3].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0, arg->freq_word_w0);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[3].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16, arg->freq_word_w1);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[3].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32, arg->freq_word_w2);
    	break;
    }
    case DFE_FL_FB_BLOCK4:
    {
    	CSL_FINS(hDfeFb->regs->nco_freq_word[4].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0, arg->freq_word_w0);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[4].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16, arg->freq_word_w1);
    	CSL_FINS(hDfeFb->regs->nco_freq_word[4].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32, arg->freq_word_w2);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbGetNcoFreq
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16
 *       DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0
 *       DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32
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
dfeFl_FbGetNcoFreq(DfeFl_FbHandle hDfeFb, DfeFl_FbNcoFreq *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    {
    	arg->freq_word_w0 = CSL_FEXT(hDfeFb->regs->nco_freq_word[0].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0);
    	arg->freq_word_w1 = CSL_FEXT(hDfeFb->regs->nco_freq_word[0].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16);
    	arg->freq_word_w2 = CSL_FEXT(hDfeFb->regs->nco_freq_word[0].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32);
    	break;
    }
    case DFE_FL_FB_BLOCK1:
    {
    	arg->freq_word_w0 = CSL_FEXT(hDfeFb->regs->nco_freq_word[1].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0);
    	arg->freq_word_w1 = CSL_FEXT(hDfeFb->regs->nco_freq_word[1].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16);
    	arg->freq_word_w2 = CSL_FEXT(hDfeFb->regs->nco_freq_word[1].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32);
    	break;
    }
    case DFE_FL_FB_BLOCK2:
    {
    	arg->freq_word_w0 = CSL_FEXT(hDfeFb->regs->nco_freq_word[2].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0);
    	arg->freq_word_w1 = CSL_FEXT(hDfeFb->regs->nco_freq_word[2].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16);
    	arg->freq_word_w2 = CSL_FEXT(hDfeFb->regs->nco_freq_word[2].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32);
    	break;
    }
    case DFE_FL_FB_BLOCK3:
    {
    	arg->freq_word_w0 = CSL_FEXT(hDfeFb->regs->nco_freq_word[3].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0);
    	arg->freq_word_w1 = CSL_FEXT(hDfeFb->regs->nco_freq_word[3].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16);
    	arg->freq_word_w2 = CSL_FEXT(hDfeFb->regs->nco_freq_word[3].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32);
    	break;
    }
    case DFE_FL_FB_BLOCK4:
    {
    	arg->freq_word_w0 = CSL_FEXT(hDfeFb->regs->nco_freq_word[4].w0, DFE_FB_NCO_FREQ_WORD_W0_REG_FREQ_WORD_15_0);
    	arg->freq_word_w1 = CSL_FEXT(hDfeFb->regs->nco_freq_word[4].w1, DFE_FB_NCO_FREQ_WORD_W1_REG_FREQ_WORD_31_16);
    	arg->freq_word_w2 = CSL_FEXT(hDfeFb->regs->nco_freq_word[4].w2, DFE_FB_NCO_FREQ_WORD_W2_REG_FREQ_WORD_47_32);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbSetNcoDither
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE3
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE2
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE1
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE0
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE4
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetNcoDither(DfeFl_FbHandle hDfeFb, DfeFl_FbNcoDither *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	CSL_FINS(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE0, arg->data);
    	break;
    case DFE_FL_FB_BLOCK1:
    	CSL_FINS(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE1, arg->data);
    	break;
    case DFE_FL_FB_BLOCK2:
    	CSL_FINS(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE2, arg->data);
    	break;
    case DFE_FL_FB_BLOCK3:
    	CSL_FINS(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE3, arg->data);
    	break;
    case DFE_FL_FB_BLOCK4:
       	CSL_FINS(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE4, arg->data);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbGetNcoDither
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE3
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE2
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE1
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE0
 *       DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE4
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
dfeFl_FbGetNcoDither(DfeFl_FbHandle hDfeFb, DfeFl_FbNcoDither *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	arg->data = CSL_FEXT(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE0);
    	break;
    case DFE_FL_FB_BLOCK1:
    	arg->data = CSL_FEXT(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE1);
    	break;
    case DFE_FL_FB_BLOCK2:
    	arg->data = CSL_FEXT(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE2);
    	break;
    case DFE_FL_FB_BLOCK3:
    	arg->data = CSL_FEXT(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE3);
    	break;
    case DFE_FL_FB_BLOCK4:
       	arg->data = CSL_FEXT(hDfeFb->regs->nco_dither_enable, DFE_FB_NCO_DITHER_ENABLE_REG_DITHER_ENABLE4);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbSetLutBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_LUT_BYPASS_REG_BYPASS4
 *       DFE_FB_LUT_BYPASS_REG_BYPASS0
 *       DFE_FB_LUT_BYPASS_REG_BYPASS1
 *       DFE_FB_LUT_BYPASS_REG_BYPASS2
 *       DFE_FB_LUT_BYPASS_REG_BYPASS3
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetLutBypass(DfeFl_FbHandle hDfeFb, DfeFl_FbLutBypass *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	CSL_FINS(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS0, arg->data);
    	break;
    case DFE_FL_FB_BLOCK1:
    	CSL_FINS(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS1, arg->data);
    	break;
    case DFE_FL_FB_BLOCK2:
    	CSL_FINS(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS2, arg->data);
    	break;
    case DFE_FL_FB_BLOCK3:
    	CSL_FINS(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS3, arg->data);
    	break;
    case DFE_FL_FB_BLOCK4:
       	CSL_FINS(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS4, arg->data);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbGetLutBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_LUT_BYPASS_REG_BYPASS4
 *       DFE_FB_LUT_BYPASS_REG_BYPASS0
 *       DFE_FB_LUT_BYPASS_REG_BYPASS1
 *       DFE_FB_LUT_BYPASS_REG_BYPASS2
 *       DFE_FB_LUT_BYPASS_REG_BYPASS3
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
dfeFl_FbGetLutBypass(DfeFl_FbHandle hDfeFb, DfeFl_FbLutBypass *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    	arg->data = CSL_FEXT(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS0);
    	break;
    case DFE_FL_FB_BLOCK1:
    	arg->data = CSL_FEXT(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS1);
    	break;
    case DFE_FL_FB_BLOCK2:
    	arg->data = CSL_FEXT(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS2);
    	break;
    case DFE_FL_FB_BLOCK3:
    	arg->data = CSL_FEXT(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS3);
    	break;
    case DFE_FL_FB_BLOCK4:
       	arg->data = CSL_FEXT(hDfeFb->regs->lut_bypass, DFE_FB_LUT_BYPASS_REG_BYPASS4);
       	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbSetLutMem
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
         idx    [add content]
         dc    [add content]
         slope    [add content]
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
 *       DFE_FB_LUT_MEM_REG_DC
 *       DFE_FB_LUT_MEM_REG_SLOPE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetLutMem(DfeFl_FbHandle hDfeFb, uint32_t idx, uint32_t dc, uint32_t slope)
{
	volatile uint32_t *regs;

	regs = &hDfeFb->regs->lut_mem[0];
	regs[idx] = CSL_FMK(DFE_FB_LUT_MEM_REG_DC, dc)
			  | CSL_FMK(DFE_FB_LUT_MEM_REG_SLOPE, slope);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetLutMem
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
         idx    [add content]
         dc    [add content]
         slope    [add content]
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
 *       DFE_FB_LUT_MEM_REG_DC
 *       DFE_FB_LUT_MEM_REG_SLOPE
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
dfeFl_FbGetLutMem(DfeFl_FbHandle hDfeFb, uint32_t idx, uint32_t *dc, uint32_t *slope)
{
	volatile uint32_t *regs;

	regs = &hDfeFb->regs->lut_mem[0];
	*dc = CSL_FEXT(regs[idx], DFE_FB_LUT_MEM_REG_DC);
	*slope = CSL_FEXT(regs[idx], DFE_FB_LUT_MEM_REG_SLOPE);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetDfRate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DF_DEC_REG_DEC_RATE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetDfRate(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->df_dec, DFE_FB_DF_DEC_REG_DEC_RATE, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetDfRate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_DF_DEC_REG_DEC_RATE
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
dfeFl_FbGetDfRate(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->df_dec, DFE_FB_DF_DEC_REG_DEC_RATE);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetGainSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_GAIN_SSEL_REG_SSEL0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetGainSsel(DfeFl_FbHandle hDfeFb, uint32_t arg)
{
	CSL_FINS(hDfeFb->regs->gain_ssel, DFE_FB_GAIN_SSEL_REG_SSEL0, arg);
}

/** ============================================================================
 *   @n@b dfeFl_FbGetGainSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_GAIN_SSEL_REG_SSEL0
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
dfeFl_FbGetGainSsel(DfeFl_FbHandle hDfeFb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeFb->regs->gain_ssel, DFE_FB_GAIN_SSEL_REG_SSEL0);
}

/** ============================================================================
 *   @n@b dfeFl_FbSetGainVal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG
 *       DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_FbSetGainVal(DfeFl_FbHandle hDfeFb, DfeFl_FbGainVal *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    {
    	CSL_FINS(hDfeFb->regs->gain_gain[0].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL, arg->real);
    	CSL_FINS(hDfeFb->regs->gain_gain[0].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG, arg->imag);
    	break;
    }
    case DFE_FL_FB_BLOCK1:
    {
    	CSL_FINS(hDfeFb->regs->gain_gain[1].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL, arg->real);
    	CSL_FINS(hDfeFb->regs->gain_gain[1].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG, arg->imag);
    	break;
    }
    case DFE_FL_FB_BLOCK2:
    {
    	CSL_FINS(hDfeFb->regs->gain_gain[2].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL, arg->real);
    	CSL_FINS(hDfeFb->regs->gain_gain[2].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG, arg->imag);
    	break;
    }
    case DFE_FL_FB_BLOCK3:
    {
    	CSL_FINS(hDfeFb->regs->gain_gain[3].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL, arg->real);
    	CSL_FINS(hDfeFb->regs->gain_gain[3].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG, arg->imag);
    	break;
    }
    case DFE_FL_FB_BLOCK4:
    {
    	CSL_FINS(hDfeFb->regs->gain_gain[4].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL, arg->real);
    	CSL_FINS(hDfeFb->regs->gain_gain[4].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG, arg->imag);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_FbGetGainVal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeFb    [add content]
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
 *       DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG
 *       DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL
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
dfeFl_FbGetGainVal(DfeFl_FbHandle hDfeFb, DfeFl_FbGainVal *arg)
{
    switch(arg->blk)
    {
    case DFE_FL_FB_BLOCK0:
    {
    	arg->real = CSL_FEXT(hDfeFb->regs->gain_gain[0].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL);
    	arg->imag = CSL_FEXT(hDfeFb->regs->gain_gain[0].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG);
    	break;
    }
    case DFE_FL_FB_BLOCK1:
    {
    	arg->real = CSL_FEXT(hDfeFb->regs->gain_gain[1].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL);
    	arg->imag = CSL_FEXT(hDfeFb->regs->gain_gain[1].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG);
    	break;
    }
    case DFE_FL_FB_BLOCK2:
    {
    	arg->real = CSL_FEXT(hDfeFb->regs->gain_gain[2].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL);
    	arg->imag = CSL_FEXT(hDfeFb->regs->gain_gain[2].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG);
    	break;
    }
    case DFE_FL_FB_BLOCK3:
    {
    	arg->real = CSL_FEXT(hDfeFb->regs->gain_gain[3].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL);
    	arg->imag = CSL_FEXT(hDfeFb->regs->gain_gain[3].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG);
    	break;
    }
    case DFE_FL_FB_BLOCK4:
    {
    	arg->real = CSL_FEXT(hDfeFb->regs->gain_gain[4].real, DFE_FB_GAIN_GAIN_REAL_REG_GAIN_REAL);
    	arg->imag = CSL_FEXT(hDfeFb->regs->gain_gain[4].imag, DFE_FB_GAIN_GAIN_IMAG_REG_GAIN_IMAG);
    	break;
    }
    default:
    	return;
    }
}

#endif /* _DFE_FL_FBAUX_H_ */
