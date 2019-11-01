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

/** @file dfe_fl_bbAux.h
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
#ifndef _DFE_FL_BBAUX_H_
#define _DFE_FL_BBAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_bb.h>

/** ============================================================================
 *   @n@b dfeFl_BbConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG13_REG_INITS_CLK_GATE
 *       DFE_BB_CFG13_REG_CLEAR_DATA
 *       DFE_BB_CFG13_REG_INITS_STATE
 *       DFE_BB_CFG13_REG_INITS_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_BbConfigInits(DfeFl_BbHandle hDfeBb, DfeFl_SublkInitsConfig * arg)
{
    uint32_t data = hDfeBb->regs->cfg13;
    
    CSL_FINS(data, DFE_BB_CFG13_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_BB_CFG13_REG_INITS_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_BB_CFG13_REG_INITS_STATE, arg->initState);
    CSL_FINS(data, DFE_BB_CFG13_REG_CLEAR_DATA, arg->clearData);
    
    hDfeBb->regs->cfg13 = data;
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableAidLoopback
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *   @n  cfg88.bit11
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_BbEnableAidLoopback(DfeFl_BbHandle hDfeBb)
{
    uint32_t data = hDfeBb->regs->cfg88;
    
    data |= 0x800u;
    
    hDfeBb->regs->cfg88 = data;
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableAidLoopback
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *   @n  cfg88.bit11
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_BbDisableAidLoopback(DfeFl_BbHandle hDfeBb)
{
    uint32_t data = hDfeBb->regs->cfg88;
    
    data &= ~0x800u;
    
    hDfeBb->regs->cfg88 = data;
}

/** ============================================================================
 *   @n@b dfeFl_BbGetAidLoopbackStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         status    [add content]
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
dfeFl_BbQueryAidLoopbackConfig(DfeFl_BbHandle hDfeBb, uint32_t *status)
{
    *status = CSL_FEXT(hDfeBb->regs->cfg88, DFE_BB_CFG88_REG_AID_IF_CONFIG);
    *status &= (uint32_t)0x00000800;
    *status >>= 11;
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigLoopback
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbConfigLoopback(DfeFl_BbHandle hDfeBb, DfeFl_BbLoopbackConfig * arg)
{
    uint32_t data=0;

    data = (((arg->duc0ToDdc1) << 1) & 2)
         | (((arg->duc1ToDdc2) << 2) & 4)
         | (((arg->duc0ToDdc3) << 3) & 8)
         | (((arg->duc3ToDdc4) << 4) & 16)
         | (((arg->duc2ToDdc5) << 5) & 32)
         | (((arg->duc1ToDdc6) << 6) & 64)
         | (((arg->duc0ToDdc7) << 7) & 128);

    hDfeBb->regs->cfg2 = data;
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryLoopbackConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbQueryLoopbackConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbLoopbackConfig * arg)
{
    uint32_t data;

    data = hDfeBb->regs->cfg2;
    arg->duc0ToDdc1 = (((data) & 2) >> 1);
    arg->duc1ToDdc2 = (((data) & 4) >> 2);
    arg->duc0ToDdc3 = (((data) & 8) >> 3);
    arg->duc3ToDdc4 = (((data) & 16) >> 4);
    arg->duc2ToDdc5 = (((data) & 32) >> 5);
    arg->duc1ToDdc6 = (((data) & 64) >> 6);
    arg->duc0ToDdc7 = (((data) & 128) >> 7);
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigCapBuff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbConfigCapBuff(DfeFl_BbHandle hDfeBb, DfeFl_BbCapBuffConfig * arg)
{
    hDfeBb->regs->cfg1 = *(uint32_t *)arg;
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryCapBuffConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbQueryCapBuffConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbCapBuffConfig * arg)
{
     *(uint32_t *)arg = hDfeBb->regs->cfg1;
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigTestGen
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_SIGNAL_GEN0_GEN_TIMER_REG_GEN_TIMER
 *       DFE_BB_SIGNAL_GEN0_RAMP_SLOPE_LO_REG_RAMP_SLOPE_15_0
 *       DFE_BB_SIGNAL_GEN0_RAMP_START_HI_REG_RAMP_START_31_16
 *       DFE_BB_SIGNAL_GEN0_RAMP_STOP_LO_REG_RAMP_STOP_15_0
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_RAMP_MODE
 *       DFE_BB_SIGNAL_GEN0_RAMP_SLOPE_HI_REG_RAMP_SLOPE_31_16
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_GEN_DATA
 *       DFE_BB_CK_SUMA_CTRL1_REG_AID_SIG_GEN_TEST_ENABLE
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_SEED
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_FRAME_LEN_M1
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_GEN_FRAME
 *       DFE_BB_SIGNAL_GEN0_RAMP_START_LO_REG_RAMP_START_15_0
 *       DFE_BB_SIGNAL_GEN0_RAMP_STOP_HI_REG_RAMP_STOP_31_16
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_BbConfigTestGen(DfeFl_BbHandle hDfeBb, DfeFl_BbTestGenConfig * arg)
{
    uint32_t data;
    volatile uint32_t *regs;
    
    switch(arg->tgDev)
    {
    // TESTGEN for DDUC0 buffer
    case DFE_FL_BB_DDUC_TESTGEN_0:
        regs = &hDfeBb->regs->gcck_dduc0_general;
        break;
    // TESTGEN for DDUC1 buffer
    case DFE_FL_BB_DDUC_TESTGEN_1:
        regs = &hDfeBb->regs->gcck_dduc1_general;
        break;
    // TESTGEN for DDUC2 buffer
    case DFE_FL_BB_DDUC_TESTGEN_2:
        regs = &hDfeBb->regs->gcck_dduc2_general;
        break;
    // TESTGEN for DDUC3 buffer
    case DFE_FL_BB_DDUC_TESTGEN_3:
        regs = &hDfeBb->regs->gcck_dduc3_general;
        break;
    // TESTGEN for DDUC4 buffer
    case DFE_FL_BB_DDUC_TESTGEN_4:
        regs = &hDfeBb->regs->gcck_dduc4_general;
        break;
    // TESTGEN for DDUC5 buffer
    case DFE_FL_BB_DDUC_TESTGEN_5:
        regs = &hDfeBb->regs->gcck_dduc5_general;
        break;
    // TESTGEN for DDUC6 buffer
    case DFE_FL_BB_DDUC_TESTGEN_6:
        regs = &hDfeBb->regs->gcck_dduc6_general;
        break;
    // TESTGEN for DDUC7 buffer
    case DFE_FL_BB_DDUC_TESTGEN_7:
        regs = &hDfeBb->regs->gcck_dduc7_general;
        break;

    // TESTGEN for AID A buffer
    case DFE_FL_BB_AID_TESTGEN_A:
        regs = &hDfeBb->regs->gcck_aida_general;
        break;
    // TESTGEN for AID B buffer
    case DFE_FL_BB_AID_TESTGEN_B:
        regs = &hDfeBb->regs->gcck_aidb_general;
        break;
    
    default:
        return;
    }
    
    // general
    regs[0] = CSL_FMK(DFE_BB_GCCK_DDUC0_GENERAL_REG_GEN_DATA, arg->genData)
         | CSL_FMK(DFE_BB_GCCK_DDUC0_GENERAL_REG_GEN_FRAME, arg->genFrame)
         | CSL_FMK(DFE_BB_GCCK_DDUC0_GENERAL_REG_RAMP_MODE, arg->rampMode)
         | CSL_FMK(DFE_BB_GCCK_DDUC0_GENERAL_REG_SEED, arg->seed)
         | CSL_FMK(DFE_BB_GCCK_DDUC0_GENERAL_REG_FRAME_LEN_M1, arg->frameLenM1);
    // ramp start     
    regs[1] = CSL_FMK(DFE_BB_SIGNAL_GEN0_RAMP_START_LO_REG_RAMP_START_15_0, arg->rampStart);
    regs[2] = CSL_FMK(DFE_BB_SIGNAL_GEN0_RAMP_START_HI_REG_RAMP_START_31_16, arg->rampStart >> 16);
    // ramp stop     
    regs[3] = CSL_FMK(DFE_BB_SIGNAL_GEN0_RAMP_STOP_LO_REG_RAMP_STOP_15_0, arg->rampStop);
    regs[4] = CSL_FMK(DFE_BB_SIGNAL_GEN0_RAMP_STOP_HI_REG_RAMP_STOP_31_16, arg->rampStop >> 16);
    // ramp slope
    regs[5] = CSL_FMK(DFE_BB_SIGNAL_GEN0_RAMP_SLOPE_LO_REG_RAMP_SLOPE_15_0, arg->slope);
    regs[6] = CSL_FMK(DFE_BB_SIGNAL_GEN0_RAMP_SLOPE_HI_REG_RAMP_SLOPE_31_16, arg->slope >> 16);
    // gen timer
    regs[7] = CSL_FMK(DFE_BB_SIGNAL_GEN0_GEN_TIMER_REG_GEN_TIMER, arg->genTimer);
    
    // ssel
    data = regs[-1];
    if(arg->tgDev == DFE_FL_BB_AID_TESTGEN_A || arg->tgDev == DFE_FL_BB_AID_TESTGEN_B)
    {
        CSL_FINS(data, DFE_BB_CK_SUMA_CTRL1_REG_AID_SIG_GEN_TEST_ENABLE, arg->testEnable);
    }
    regs[-1] = data;

}

/** ============================================================================
 *   @n@b dfeFl_BbQueryTestGenConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_RAMP_MODE
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_GEN_DATA
 *       DFE_BB_CK_SUMA_CTRL1_REG_AID_SIG_GEN_TEST_ENABLE
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_SEED
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_FRAME_LEN_M1
 *       DFE_BB_GCCK_DDUC0_GENERAL_REG_GEN_FRAME
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
dfeFl_BbQueryTestGenConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbTestGenConfig * arg)
{
    uint32_t data;
    volatile uint32_t *regs;
    
    switch(arg->tgDev)
    {
    // TESTGEN for DDUC0 buffer
    case DFE_FL_BB_DDUC_TESTGEN_0:
        regs = &hDfeBb->regs->gcck_dduc0_general;
        break;
    // TESTGEN for DDUC1 buffer
    case DFE_FL_BB_DDUC_TESTGEN_1:
        regs = &hDfeBb->regs->gcck_dduc1_general;
        break;
    // TESTGEN for DDUC2 buffer
    case DFE_FL_BB_DDUC_TESTGEN_2:
        regs = &hDfeBb->regs->gcck_dduc2_general;
        break;
    // TESTGEN for DDUC3 buffer
    case DFE_FL_BB_DDUC_TESTGEN_3:
        regs = &hDfeBb->regs->gcck_dduc3_general;
        break;
    // TESTGEN for DDUC4 buffer
    case DFE_FL_BB_DDUC_TESTGEN_4:
        regs = &hDfeBb->regs->gcck_dduc4_general;
        break;
    // TESTGEN for DDUC5 buffer
    case DFE_FL_BB_DDUC_TESTGEN_5:
        regs = &hDfeBb->regs->gcck_dduc5_general;
        break;
    // TESTGEN for DDUC6 buffer
    case DFE_FL_BB_DDUC_TESTGEN_6:
        regs = &hDfeBb->regs->gcck_dduc6_general;
        break;
    // TESTGEN for DDUC7 buffer
    case DFE_FL_BB_DDUC_TESTGEN_7:
        regs = &hDfeBb->regs->gcck_dduc7_general;
        break;

    // TESTGEN for AID A buffer
    case DFE_FL_BB_AID_TESTGEN_A:
        regs = &hDfeBb->regs->gcck_aida_general;
        break;
    // TESTGEN for AID B buffer
    case DFE_FL_BB_AID_TESTGEN_B:
        regs = &hDfeBb->regs->gcck_aidb_general;
        break;
    
    default:
        return;
    }
    
    // general
    data = regs[0];
    arg->genData = CSL_FEXT(data, DFE_BB_GCCK_DDUC0_GENERAL_REG_GEN_DATA);
    arg->genFrame = CSL_FEXT(data, DFE_BB_GCCK_DDUC0_GENERAL_REG_GEN_FRAME);
    arg->rampMode = (DfeFl_BbTestGenRampMode)CSL_FEXT(data, DFE_BB_GCCK_DDUC0_GENERAL_REG_RAMP_MODE);
    arg->seed = CSL_FEXT(data, DFE_BB_GCCK_DDUC0_GENERAL_REG_SEED);
    arg->frameLenM1 = CSL_FEXT(data, DFE_BB_GCCK_DDUC0_GENERAL_REG_FRAME_LEN_M1);
    // ramp start     
    arg->rampStart = regs[1] | (regs[2] << 16);
    // ramp stop     
    arg->rampStop = regs[3] | (regs[4] << 16);
    // ramp slope
    arg->slope = regs[5] | (regs[6] << 16);
    // gen timer
    arg->genTimer = regs[7];
    
    // ssel
    data = regs[-1];
    if(arg->tgDev == DFE_FL_BB_AID_TESTGEN_A || arg->tgDev == DFE_FL_BB_AID_TESTGEN_B)
    {
        arg->testEnable = CSL_FEXT(data, DFE_BB_CK_SUMA_CTRL1_REG_AID_SIG_GEN_TEST_ENABLE);
    }
    else
        arg->testEnable = 0;
}

/** ============================================================================
 *   @n@b dfeFl_BbSetTestGenSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         tgDev    [add content]
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
 *       DFE_BB_CK_DDUC0_DDUC0_SSEL_REG_SIG_GEN_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_BbSetTestGenSsel(DfeFl_BbHandle hDfeBb, uint32_t tgDev, uint32_t ssel)
{
    uint32_t data;
    volatile uint32_t *regs;
    
    switch(tgDev)
    {
    // TESTGEN for DDUC0 buffer
    case DFE_FL_BB_DDUC_TESTGEN_0:
        regs = &hDfeBb->regs->gcck_dduc0_general;
        break;
    // TESTGEN for DDUC1 buffer
    case DFE_FL_BB_DDUC_TESTGEN_1:
        regs = &hDfeBb->regs->gcck_dduc1_general;
        break;
    // TESTGEN for DDUC2 buffer
    case DFE_FL_BB_DDUC_TESTGEN_2:
        regs = &hDfeBb->regs->gcck_dduc2_general;
        break;
    // TESTGEN for DDUC3 buffer
    case DFE_FL_BB_DDUC_TESTGEN_3:
        regs = &hDfeBb->regs->gcck_dduc3_general;
        break;
    // TESTGEN for DDUC4 buffer
    case DFE_FL_BB_DDUC_TESTGEN_4:
        regs = &hDfeBb->regs->gcck_dduc4_general;
        break;
    // TESTGEN for DDUC5 buffer
    case DFE_FL_BB_DDUC_TESTGEN_5:
        regs = &hDfeBb->regs->gcck_dduc5_general;
        break;
    // TESTGEN for DDUC6 buffer
    case DFE_FL_BB_DDUC_TESTGEN_6:
        regs = &hDfeBb->regs->gcck_dduc6_general;
        break;
    // TESTGEN for DDUC7 buffer
    case DFE_FL_BB_DDUC_TESTGEN_7:
        regs = &hDfeBb->regs->gcck_dduc7_general;
        break;

    // TESTGEN for AID A buffer
    case DFE_FL_BB_AID_TESTGEN_A:
        regs = &hDfeBb->regs->gcck_aida_general;
        break;
    // TESTGEN for AID B buffer
    case DFE_FL_BB_AID_TESTGEN_B:
        regs = &hDfeBb->regs->gcck_aidb_general;
        break;
    
    default:
        return;
    }
        
    // ssel
    data = regs[-1];
    CSL_FINS(data, DFE_BB_CK_DDUC0_DDUC0_SSEL_REG_SIG_GEN_SSEL, ssel);
    regs[-1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_BbGetTestGenSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         tgDev    [add content]
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
 *       DFE_BB_CK_DDUC0_DDUC0_SSEL_REG_SIG_GEN_SSEL
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
dfeFl_BbGetTestGenSsel(DfeFl_BbHandle hDfeBb, uint32_t tgDev, uint32_t *ssel)
{
    uint32_t data;
    volatile uint32_t *regs;
    
    switch(tgDev)
    {
    // TESTGEN for DDUC0 buffer
    case DFE_FL_BB_DDUC_TESTGEN_0:
        regs = &hDfeBb->regs->gcck_dduc0_general;
        break;
    // TESTGEN for DDUC1 buffer
    case DFE_FL_BB_DDUC_TESTGEN_1:
        regs = &hDfeBb->regs->gcck_dduc1_general;
        break;
    // TESTGEN for DDUC2 buffer
    case DFE_FL_BB_DDUC_TESTGEN_2:
        regs = &hDfeBb->regs->gcck_dduc2_general;
        break;
    // TESTGEN for DDUC3 buffer
    case DFE_FL_BB_DDUC_TESTGEN_3:
        regs = &hDfeBb->regs->gcck_dduc3_general;
        break;
    // TESTGEN for DDUC4 buffer
    case DFE_FL_BB_DDUC_TESTGEN_4:
        regs = &hDfeBb->regs->gcck_dduc4_general;
        break;
    // TESTGEN for DDUC5 buffer
    case DFE_FL_BB_DDUC_TESTGEN_5:
        regs = &hDfeBb->regs->gcck_dduc5_general;
        break;
    // TESTGEN for DDUC6 buffer
    case DFE_FL_BB_DDUC_TESTGEN_6:
        regs = &hDfeBb->regs->gcck_dduc6_general;
        break;
    // TESTGEN for DDUC7 buffer
    case DFE_FL_BB_DDUC_TESTGEN_7:
        regs = &hDfeBb->regs->gcck_dduc7_general;
        break;

    // TESTGEN for AID A buffer
    case DFE_FL_BB_AID_TESTGEN_A:
        regs = &hDfeBb->regs->gcck_aida_general;
        break;
    // TESTGEN for AID B buffer
    case DFE_FL_BB_AID_TESTGEN_B:
        regs = &hDfeBb->regs->gcck_aidb_general;
        break;
    
    default:
        return;
    }
        
    // ssel
    data = regs[-1];
    *ssel = CSL_FEXT(data, DFE_BB_CK_DDUC0_DDUC0_SSEL_REG_SIG_GEN_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CK_SUMA_CTRL0_REG_STABLE_LEN
 *       DFE_BB_CK_SUMA_CTRL0_REG_MODE
 *       DFE_BB_CK_SUMA_CHAN_SEL_REG_CHAN_SEL
 *       DFE_BB_CK_SUMA_SIGNAL_LEN_REG_SIGNAL_LEN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigChksum(DfeFl_BbHandle hDfeBb, DfeFl_BbChksumConfig *arg)
{
    volatile uint32_t *regs;
    
    switch(arg->chksumDev)    
    {
    // CHKSUM for DDUC0 buffer
    case DFE_FL_BB_DDUC_CHKSUM_0:
        regs = &hDfeBb->regs->signal_gen0_ctrl0;
        break;
    // CHKSUM for DDUC1 buffer
    case DFE_FL_BB_DDUC_CHKSUM_1:
        regs = &hDfeBb->regs->signal_gen1_ctrl0;
        break;
    // CHKSUM for DDUC2 buffer
    case DFE_FL_BB_DDUC_CHKSUM_2:
        regs = &hDfeBb->regs->signal_gen2_ctrl0;
        break;
    // CHKSUM for DDUC3 buffer
    case DFE_FL_BB_DDUC_CHKSUM_3:
        regs = &hDfeBb->regs->signal_gen3_ctrl0;
        break;
    // CHKSUM for DDUC4 buffer
    case DFE_FL_BB_DDUC_CHKSUM_4:
        regs = &hDfeBb->regs->signal_gen4_ctrl0;
        break;
    // CHKSUM for DDUC5 buffer
    case DFE_FL_BB_DDUC_CHKSUM_5:
        regs = &hDfeBb->regs->signal_gen5_ctrl0;
        break;
    // CHKSUM for DDUC6 buffer
    case DFE_FL_BB_DDUC_CHKSUM_6:
        regs = &hDfeBb->regs->signal_gen6_ctrl0;
        break;
    // CHKSUM for DDUC7 buffer
    case DFE_FL_BB_DDUC_CHKSUM_7:
        regs = &hDfeBb->regs->signal_gen7_ctrl0;
        break;

    // CHKSUM for AID A buffer
    case DFE_FL_BB_AID_CHKSUM_A:
        regs = &hDfeBb->regs->ck_suma_ctrl0;
        break;
    // CHKSUM for AID B buffer
    case DFE_FL_BB_AID_CHKSUM_B:
        regs = &hDfeBb->regs->ck_sumb_ctrl0;
        break;

    default:
    	return;
    }
    
    // ctrl0, stable_len
    regs[0] = CSL_FMK(DFE_BB_CK_SUMA_CTRL0_REG_MODE, arg->chksumMode)
            | CSL_FMK(DFE_BB_CK_SUMA_CTRL0_REG_STABLE_LEN, arg->latencyMode.stableLen);
    
    // signal_len
    regs[1] = CSL_FMK(DFE_BB_CK_SUMA_SIGNAL_LEN_REG_SIGNAL_LEN, arg->latencyMode.signalLen);
    
    // chan_sel
    regs[2] = CSL_FMK(DFE_BB_CK_SUMA_CHAN_SEL_REG_CHAN_SEL, arg->latencyMode.chanSel);            
}

/** ============================================================================
 *   @n@b dfeFl_BbSetChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CK_SUMA_CTRL1_REG_CHKSUM_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetChksumSsel(DfeFl_BbHandle hDfeBb, DfeFl_BbChksumSsel *arg)
{
    volatile uint32_t *regs;
    
    switch(arg->chksumDev)    
    {
    // CHKSUM for DDUC0 buffer
    case DFE_FL_BB_DDUC_CHKSUM_0:
        regs = &hDfeBb->regs->signal_gen0_ctrl0;
        break;
    // CHKSUM for DDUC1 buffer
    case DFE_FL_BB_DDUC_CHKSUM_1:
        regs = &hDfeBb->regs->signal_gen1_ctrl0;
        break;
    // CHKSUM for DDUC2 buffer
    case DFE_FL_BB_DDUC_CHKSUM_2:
        regs = &hDfeBb->regs->signal_gen2_ctrl0;
        break;
    // CHKSUM for DDUC3 buffer
    case DFE_FL_BB_DDUC_CHKSUM_3:
        regs = &hDfeBb->regs->signal_gen3_ctrl0;
        break;
    // CHKSUM for DDUC4 buffer
    case DFE_FL_BB_DDUC_CHKSUM_4:
        regs = &hDfeBb->regs->signal_gen4_ctrl0;
        break;
    // CHKSUM for DDUC5 buffer
    case DFE_FL_BB_DDUC_CHKSUM_5:
        regs = &hDfeBb->regs->signal_gen5_ctrl0;
        break;
    // CHKSUM for DDUC6 buffer
    case DFE_FL_BB_DDUC_CHKSUM_6:
        regs = &hDfeBb->regs->signal_gen6_ctrl0;
        break;
    // CHKSUM for DDUC7 buffer
    case DFE_FL_BB_DDUC_CHKSUM_7:
        regs = &hDfeBb->regs->signal_gen7_ctrl0;
        break;

    // CHKSUM for AID A buffer
    case DFE_FL_BB_AID_CHKSUM_A:
        regs = &hDfeBb->regs->ck_suma_ctrl0;
        break;
    // CHKSUM for AID B buffer
    case DFE_FL_BB_AID_CHKSUM_B:
        regs = &hDfeBb->regs->ck_sumb_ctrl0;
        break;

    default:
    	return;
    }
    
    CSL_FINS(regs[5], DFE_BB_CK_SUMA_CTRL1_REG_CHKSUM_SSEL, arg->ssel);
}

/** ============================================================================
 *   @n@b dfeFl_BbGetChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *   @n  DFE_BB_CK_SUMA_CTRL1_REG_CHKSUM_SSEL
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
dfeFl_BbGetChksumSsel(DfeFl_BbHandle hDfeBb, DfeFl_BbChksumSsel *arg)
{
    volatile uint32_t *regs;

    switch(arg->chksumDev)
    {
    // CHKSUM for DDUC0 buffer
    case DFE_FL_BB_DDUC_CHKSUM_0:
        regs = &hDfeBb->regs->signal_gen0_ctrl0;
        break;
    // CHKSUM for DDUC1 buffer
    case DFE_FL_BB_DDUC_CHKSUM_1:
        regs = &hDfeBb->regs->signal_gen1_ctrl0;
        break;
    // CHKSUM for DDUC2 buffer
    case DFE_FL_BB_DDUC_CHKSUM_2:
        regs = &hDfeBb->regs->signal_gen2_ctrl0;
        break;
    // CHKSUM for DDUC3 buffer
    case DFE_FL_BB_DDUC_CHKSUM_3:
        regs = &hDfeBb->regs->signal_gen3_ctrl0;
        break;
    // CHKSUM for DDUC4 buffer
    case DFE_FL_BB_DDUC_CHKSUM_4:
        regs = &hDfeBb->regs->signal_gen4_ctrl0;
        break;
    // CHKSUM for DDUC5 buffer
    case DFE_FL_BB_DDUC_CHKSUM_5:
        regs = &hDfeBb->regs->signal_gen5_ctrl0;
        break;
    // CHKSUM for DDUC6 buffer
    case DFE_FL_BB_DDUC_CHKSUM_6:
        regs = &hDfeBb->regs->signal_gen6_ctrl0;
        break;
    // CHKSUM for DDUC7 buffer
    case DFE_FL_BB_DDUC_CHKSUM_7:
        regs = &hDfeBb->regs->signal_gen7_ctrl0;
        break;

    // CHKSUM for AID A buffer
    case DFE_FL_BB_AID_CHKSUM_A:
        regs = &hDfeBb->regs->ck_suma_ctrl0;
        break;
    // CHKSUM for AID B buffer
    case DFE_FL_BB_AID_CHKSUM_B:
        regs = &hDfeBb->regs->ck_sumb_ctrl0;
        break;

    default:
    	return;
    }

    arg->ssel = CSL_FEXT(regs[5], DFE_BB_CK_SUMA_CTRL1_REG_CHKSUM_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_BbGetChksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CK_SUMA_RESULT_LO_REG_RESULT_15_0
 *       DFE_BB_CK_SUMA_RESULT_HI_REG_RESULT_31_16
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
dfeFl_BbGetChksumResult(DfeFl_BbHandle hDfeBb, DfeFl_BbChksumResult *arg)
{
    volatile uint32_t *regs;
    
    switch(arg->chksumDev)    
    {
    // CHKSUM for DDUC0 buffer
    case DFE_FL_BB_DDUC_CHKSUM_0:
        regs = &hDfeBb->regs->signal_gen0_ctrl0;
        break;
    // CHKSUM for DDUC1 buffer
    case DFE_FL_BB_DDUC_CHKSUM_1:
        regs = &hDfeBb->regs->signal_gen1_ctrl0;
        break;
    // CHKSUM for DDUC2 buffer
    case DFE_FL_BB_DDUC_CHKSUM_2:
        regs = &hDfeBb->regs->signal_gen2_ctrl0;
        break;
    // CHKSUM for DDUC3 buffer
    case DFE_FL_BB_DDUC_CHKSUM_3:
        regs = &hDfeBb->regs->signal_gen3_ctrl0;
        break;
    // CHKSUM for DDUC4 buffer
    case DFE_FL_BB_DDUC_CHKSUM_4:
        regs = &hDfeBb->regs->signal_gen4_ctrl0;
        break;
    // CHKSUM for DDUC5 buffer
    case DFE_FL_BB_DDUC_CHKSUM_5:
        regs = &hDfeBb->regs->signal_gen5_ctrl0;
        break;
    // CHKSUM for DDUC6 buffer
    case DFE_FL_BB_DDUC_CHKSUM_6:
        regs = &hDfeBb->regs->signal_gen6_ctrl0;
        break;
    // CHKSUM for DDUC7 buffer
    case DFE_FL_BB_DDUC_CHKSUM_7:
        regs = &hDfeBb->regs->signal_gen7_ctrl0;
        break;

    // CHKSUM for AID A buffer
    case DFE_FL_BB_AID_CHKSUM_A:
        regs = &hDfeBb->regs->ck_suma_ctrl0;
        break;
    // CHKSUM for AID B buffer
    case DFE_FL_BB_AID_CHKSUM_B:
        regs = &hDfeBb->regs->ck_sumb_ctrl0;
        break;

    default:
    	return;
    }
    
    arg->result = CSL_FEXT(regs[3], DFE_BB_CK_SUMA_RESULT_LO_REG_RESULT_15_0);
    arg->result |= CSL_FEXT(regs[4], DFE_BB_CK_SUMA_RESULT_HI_REG_RESULT_31_16) << 16;
}

/** ============================================================================
 *   @n@b dfeFl_BbCfgTxifAxc
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         axc    [add content]
         txif    [add content]
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
 *       DFE_BB_BBTXIF_AXC_CONFIG2_REG_AUTOCP_SEL
 *       DFE_BB_BBTXIF_AXC_CONFIG0_REG_GAIN_EN
 *       DFE_BB_BBTXIF_AXC_CONFIG0_REG_BUFFER_INDEX
 *       DFE_BB_BBTXIF_AXC_CONFIG2_REG_ANTCAL_SEL
 *       DFE_BB_BBTXIF_AXC_CONFIG0_REG_CL_EN
 *       DFE_BB_BBTXIF_AXC_CONFIG1_REG_CL_1OVERT
 *       DFE_BB_BBTXIF_AXC_CONFIG0_REG_PM_CONFIG_SEL
 *       DFE_BB_BBTXIF_AXC_CONFIG0_REG_PM_EN
 *       DFE_BB_BBTXIF_AXC_CONFIG2_REG_AUTOCP_EN
 *       DFE_BB_BBTXIF_AXC_CONFIG0_REG_AXC_VALID
 *       DFE_BB_BBTXIF_AXC_CONFIG0_REG_BUFFER_NUM
 *       DFE_BB_BBTXIF_AXC_CONFIG2_REG_ANTCAL_EN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbCfgTxifAxc(DfeFl_BbHandle hDfeBb, uint32_t axc, DfeFl_BbTxifAxc *txif)
{
    hDfeBb->regs->bbtxif_axc[axc].config0 = \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG0_REG_BUFFER_INDEX, txif->bufferIndex)  | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG0_REG_BUFFER_NUM, txif->bufferNum)      | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG0_REG_PM_CONFIG_SEL, txif->pmConfigSel) | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG0_REG_PM_EN, txif->pmEn)                | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG0_REG_CL_EN, txif->clEn)                | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG0_REG_GAIN_EN, txif->gainEn)            | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG0_REG_AXC_VALID, txif->axcValid);
    hDfeBb->regs->bbtxif_axc[axc].config1 = \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG1_REG_CL_1OVERT, txif->cl1OverT);
    hDfeBb->regs->bbtxif_axc[axc].config2 = \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG2_REG_ANTCAL_SEL, txif->antcalSel)       | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG2_REG_ANTCAL_EN, txif->antcalEn)        | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG2_REG_AUTOCP_EN, txif->autocpEn)        | \
        CSL_FMK(DFE_BB_BBTXIF_AXC_CONFIG2_REG_AUTOCP_SEL, txif->autocpSel);
}

/** ============================================================================
 *   @n@b dfeFl_BbCfgRxifAxc
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         axc    [add content]
         rxif    [add content]
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
 *       DFE_BB_BBRXIF_AXC_CONFIG4_REG_BEAGC_T1_INTERVAL_7_0
 *       DFE_BB_BBRXIF_AXC_CONFIG0_REG_BEAGC_MODE
 *       DFE_BB_BBRXIF_AXC_CONFIG0_REG_AXC_VALID
 *       DFE_BB_BBRXIF_AXC_CONFIG5_REG_BEAGC_T1_INTERVAL_23_8
 *       DFE_BB_BBRXIF_AXC_CONFIG7_REG_BEAGC_T2_INTERVAL_23_16
 *       DFE_BB_BBRXIF_AXC_CONFIG2_REG_BEAGC_POWER_BACKOFF
 *       DFE_BB_BBRXIF_AXC_CONFIG1_REG_PM_EN
 *       DFE_BB_BBRXIF_AXC_CONFIG1_REG_OUT_NUM_BITS
 *       DFE_BB_BBRXIF_AXC_CONFIG1_REG_OUT_PACKED
 *       DFE_BB_BBRXIF_AXC_CONFIG1_REG_OUT_FLOAT_MODE
 *       DFE_BB_BBRXIF_AXC_CONFIG1_REG_NOTCH_EN
 *       DFE_BB_BBRXIF_AXC_CONFIG4_REG_BEAGC_T3_ACTV_CNT
 *       DFE_BB_BBRXIF_AXC_CONFIG0_REG_CARRIER_TYPE
 *       DFE_BB_BBRXIF_AXC_CONFIG2_REG_ANT_CAL_SEL
 *       DFE_BB_BBRXIF_AXC_CONFIG2_REG_ANT_CAL_EN
 *       DFE_BB_BBRXIF_AXC_CONFIG0_REG_BUFFER_INDEX
 *       DFE_BB_BBRXIF_AXC_CONFIG1_REG_PM_CONFIG_SEL
 *       DFE_BB_BBRXIF_AXC_CONFIG2_REG_TDD0
 *       DFE_BB_BBRXIF_AXC_CONFIG0_REG_BUFFER_NUM
 *       DFE_BB_BBRXIF_AXC_CONFIG1_REG_FIXEDORFLOAT
 *       DFE_BB_BBRXIF_AXC_CONFIG6_REG_BEAGC_T2_INTERVAL_15_0
 *       DFE_BB_BBRXIF_AXC_CONFIG4_REG_BEAGC_CONFIG_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbCfgRxifAxc(DfeFl_BbHandle hDfeBb, uint32_t axc, DfeFl_BbRxifAxc *rxif)
{
    hDfeBb->regs->bbrxif_axc[axc].config0 = \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG0_REG_BUFFER_INDEX, rxif->bufferIndex)  | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG0_REG_BUFFER_NUM, rxif->bufferNum)      | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG0_REG_CARRIER_TYPE, rxif->carrierType)  | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG0_REG_BEAGC_MODE, rxif->beagcMode)      | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG0_REG_AXC_VALID, rxif->axcValid);
    hDfeBb->regs->bbrxif_axc[axc].config1 = \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG1_REG_PM_CONFIG_SEL, rxif->pmConfigSel) | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG1_REG_PM_EN, rxif->pmEn)                | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG1_REG_NOTCH_EN, rxif->notchEn)          | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG1_REG_OUT_PACKED, rxif->outPacked)      | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG1_REG_OUT_FLOAT_MODE, rxif->outFloatMode) | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG1_REG_FIXEDORFLOAT, rxif->fixedOrFloat) | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG1_REG_OUT_NUM_BITS, rxif->outNumBits);
    hDfeBb->regs->bbrxif_axc[axc].config2 = \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG2_REG_ANT_CAL_SEL, rxif->antcalSel)     | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG2_REG_ANT_CAL_EN, rxif->antcalEn)       | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG2_REG_BEAGC_POWER_BACKOFF, rxif->beagcPowerBackoff) | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG2_REG_TDD0, rxif->tdd0);
    hDfeBb->regs->bbrxif_axc[axc].config4 = \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG4_REG_BEAGC_T3_ACTV_CNT, rxif->beagcT3ActvCnt) | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG4_REG_BEAGC_CONFIG_SEL, rxif->beagcConfigSel) | \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG4_REG_BEAGC_T1_INTERVAL_7_0, rxif->beagcT1Interval);
    hDfeBb->regs->bbrxif_axc[axc].config5 = \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG5_REG_BEAGC_T1_INTERVAL_23_8, rxif->beagcT1Interval >> 8);
    hDfeBb->regs->bbrxif_axc[axc].config6 = \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG6_REG_BEAGC_T2_INTERVAL_15_0, rxif->beagcT2Interval);
    hDfeBb->regs->bbrxif_axc[axc].config7 = \
        CSL_FMK(DFE_BB_BBRXIF_AXC_CONFIG7_REG_BEAGC_T2_INTERVAL_23_16, rxif->beagcT1Interval >> 16);
}

/** ============================================================================
 *   @n@b dfeFl_BbGetTxGainUpdateStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
         status    [add content]
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
dfeFl_BbGetTxGainUpdateStatus(DfeFl_BbHandle hDfeBb, uint32_t ct, uint32_t *status)
{
    *status = CSL_FEXTR(hDfeBb->regs->cfg85, ct, ct);
}

/** ============================================================================
 *   @n@b dfeFl_BbUpdateTxGain
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         axc    [add content]
         gainI    [add content]
         gainQ    [add content]
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
 *       DFE_BB_BBTXGAIN_I_REG_INPHASE
 *       DFE_BB_BBTXGAIN_Q_REG_QUADRATURE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbUpdateTxGain(DfeFl_BbHandle hDfeBb, uint32_t axc, uint32_t gainI, uint32_t gainQ)
{
    hDfeBb->regs->bbtxgain[axc].i = CSL_FMK(DFE_BB_BBTXGAIN_I_REG_INPHASE, gainI);        
    hDfeBb->regs->bbtxgain[axc].q = CSL_FMK(DFE_BB_BBTXGAIN_Q_REG_QUADRATURE, gainQ);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryTxGain
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         axc    [add content]
         gainI    [add content]
         gainQ    [add content]
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
 *       DFE_BB_BBTXGAIN_I_REG_INPHASE
 *       DFE_BB_BBTXGAIN_Q_REG_QUADRATURE
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
dfeFl_BbQueryTxGain(DfeFl_BbHandle hDfeBb, uint32_t axc, uint32_t *gainI, uint32_t *gainQ)
{
    *gainI = CSL_FEXT(hDfeBb->regs->bbtxgain[axc].i, DFE_BB_BBTXGAIN_I_REG_INPHASE);        
    *gainQ = CSL_FEXT(hDfeBb->regs->bbtxgain[axc].q, DFE_BB_BBTXGAIN_Q_REG_QUADRATURE);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetTxGainCtSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetTxGainCtSsel(DfeFl_BbHandle hDfeBb, uint32_t ct, uint32_t ssel)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg60;
    uint32_t r = ct / 4;
    uint32_t b = (ct % 4) * 4;
    
    CSL_FINSR(regs[r], b+3, b, ssel);    
}

/** ============================================================================
 *   @n@b dfeFl_BbGetTxGainCtSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbGetTxGainCtSsel(DfeFl_BbHandle hDfeBb, uint32_t ct, uint32_t *ssel)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg60;
    uint32_t r = ct / 4;
    uint32_t b = (ct % 4) * 4;
    
    *ssel = CSL_FEXTR(regs[r], b+3, b);    
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableTxGainIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbEnableTxGainIntr(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    CSL_FINSR(hDfeBb->regs->intmask2, ct, ct, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableTxGainIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbDisableTxGainIntr(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    CSL_FINSR(hDfeBb->regs->intmask2, ct, ct, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearTxGainIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbClearTxGainIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    hDfeBb->regs->intstatus2 = ~(1u << ct);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryTxGainIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
         status    [add content]
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
dfeFl_BbQueryTxGainIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t ct, uint32_t *status)
{
    *status = CSL_FEXTR(hDfeBb->regs->intstatus2, ct, ct);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetForceTxGainIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbSetForceTxGainIntr(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    CSL_FINSR(hDfeBb->regs->intforce2, ct, ct, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearForceTxGainIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbClearForceTxGainIntr(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    CSL_FINSR(hDfeBb->regs->intforce2, ct, ct, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigTxTdd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG1168_REG_TXTDD_SSEL
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_BB_CFG1173_REG_TXTDD_TIMER_UL1_23_16
 *       DFE_BB_CFG1168_REG_TXTDD_DATAMODE
 *       DFE_BB_CFG1172_REG_TXTDD_TIMER_UL1_15_0
 *       DFE_BB_CFG1170_REG_TXTDD_TIMER_DL1_23_16
 *       DFE_BB_CFG1171_REG_TXTDD_TIMER_DL1_15_0
 *       DFE_BB_CFG1177_REG_TXTDD_TIMER_DL3_15_0
 *       DFE_BB_CFG1176_REG_TXTDD_TIMER_UL2_23_16
 *       DFE_BB_CFG1168_REG_TXTDD_SSEL
 *       DFE_BB_CFG1169_REG_TXTDD_DELAYFROMSYNC_15_0
 *       DFE_BB_CFG1168_REG_TXTDD_CT_TYPE
 *       DFE_BB_CFG1168_REG_TXTDD_EN
 *       DFE_BB_CFG1170_REG_TXTDD_DELAYFROMSYNC_23_16
 *       DFE_BB_CFG1173_REG_TXTDD_TIMER_DL2_23_16
 *       DFE_BB_CFG1175_REG_TXTDD_TIMER_UL2_15_0
 *       DFE_BB_CFG1174_REG_TXTDD_TIMER_DL2_15_0
 *       DFE_BB_CFG1176_REG_TXTDD_TIMER_DL3_23_16
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigTxTdd(DfeFl_BbHandle hDfeBb, DfeFl_BbTddConfig * arg)
{
    uint32_t data;
    
    hDfeBb->regs->cfg1169 = CSL_FMK(DFE_BB_CFG1169_REG_TXTDD_DELAYFROMSYNC_15_0, arg->syncDly);
    hDfeBb->regs->cfg1170 = CSL_FMK(DFE_BB_CFG1170_REG_TXTDD_DELAYFROMSYNC_23_16, arg->syncDly >> 16)
                    | CSL_FMK(DFE_BB_CFG1170_REG_TXTDD_TIMER_DL1_23_16, arg->dl1Interval >> 16);
    hDfeBb->regs->cfg1171 = CSL_FMK(DFE_BB_CFG1171_REG_TXTDD_TIMER_DL1_15_0, arg->dl1Interval);                 
    hDfeBb->regs->cfg1172 = CSL_FMK(DFE_BB_CFG1172_REG_TXTDD_TIMER_UL1_15_0, arg->ul1Interval);   
    hDfeBb->regs->cfg1173 = CSL_FMK(DFE_BB_CFG1173_REG_TXTDD_TIMER_UL1_23_16, arg->ul1Interval >> 16)              
                    | CSL_FMK(DFE_BB_CFG1173_REG_TXTDD_TIMER_DL2_23_16, arg->dl2Interval >> 16);
    hDfeBb->regs->cfg1174 = CSL_FMK(DFE_BB_CFG1174_REG_TXTDD_TIMER_DL2_15_0, arg->dl2Interval);                 
    hDfeBb->regs->cfg1175 = CSL_FMK(DFE_BB_CFG1175_REG_TXTDD_TIMER_UL2_15_0, arg->ul2Interval);   
    hDfeBb->regs->cfg1176 = CSL_FMK(DFE_BB_CFG1176_REG_TXTDD_TIMER_UL2_23_16, arg->ul2Interval >> 16)              
                    | CSL_FMK(DFE_BB_CFG1176_REG_TXTDD_TIMER_DL3_23_16, arg->dl3Interval >> 16);
    hDfeBb->regs->cfg1177 = CSL_FMK(DFE_BB_CFG1177_REG_TXTDD_TIMER_DL3_15_0, arg->dl3Interval); 
    
    // ssel
    data = CSL_FEXT(hDfeBb->regs->cfg1168, DFE_BB_CFG1168_REG_TXTDD_SSEL);
    hDfeBb->regs->cfg1168 = CSL_FMK(DFE_BB_CFG1168_REG_TXTDD_EN, arg->enable)
                    | CSL_FMK(DFE_BB_CFG1168_REG_TXTDD_DATAMODE, arg->dataMode)
                    | CSL_FMK(DFE_BB_CFG1168_REG_TXTDD_CT_TYPE, arg->carrierType)
                    | CSL_FMK(DFE_BB_CFG1168_REG_TXTDD_SSEL, data);
                                    
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryTxTddConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG1173_REG_TXTDD_TIMER_UL1_23_16
 *       DFE_BB_CFG1168_REG_TXTDD_DATAMODE
 *       DFE_BB_CFG1172_REG_TXTDD_TIMER_UL1_15_0
 *       DFE_BB_CFG1170_REG_TXTDD_TIMER_DL1_23_16
 *       DFE_BB_CFG1171_REG_TXTDD_TIMER_DL1_15_0
 *       DFE_BB_CFG1176_REG_TXTDD_TIMER_UL2_23_16
 *       DFE_BB_CFG1169_REG_TXTDD_DELAYFROMSYNC_15_0
 *       DFE_BB_CFG1168_REG_TXTDD_CT_TYPE
 *       DFE_BB_CFG1168_REG_TXTDD_EN
 *       DFE_BB_CFG1170_REG_TXTDD_DELAYFROMSYNC_23_16
 *       DFE_BB_CFG1173_REG_TXTDD_TIMER_DL2_23_16
 *       DFE_BB_CFG1175_REG_TXTDD_TIMER_UL2_15_0
 *       DFE_BB_CFG1176_REG_TXTDD_TIMER_DL3_23_16
 *       DFE_BB_CFG1174_REG_TXTDD_TIMER_DL2_15_0
 *       DFE_BB_CFG1177_REG_TXTDD_TIMER_DL3_15_0
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
dfeFl_BbQueryTxTddConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbTddConfig * arg)
{
    uint32_t data;
    
    arg->syncDly = CSL_FEXT(hDfeBb->regs->cfg1169, DFE_BB_CFG1169_REG_TXTDD_DELAYFROMSYNC_15_0) 
                 |(CSL_FEXT(hDfeBb->regs->cfg1170, DFE_BB_CFG1170_REG_TXTDD_DELAYFROMSYNC_23_16) << 16);
    arg->dl1Interval = CSL_FEXT(hDfeBb->regs->cfg1171, DFE_BB_CFG1171_REG_TXTDD_TIMER_DL1_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1170, DFE_BB_CFG1170_REG_TXTDD_TIMER_DL1_23_16) << 16);
    arg->ul1Interval = CSL_FEXT(hDfeBb->regs->cfg1172, DFE_BB_CFG1172_REG_TXTDD_TIMER_UL1_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1173, DFE_BB_CFG1173_REG_TXTDD_TIMER_UL1_23_16) << 16);                 
    arg->dl2Interval = CSL_FEXT(hDfeBb->regs->cfg1174, DFE_BB_CFG1174_REG_TXTDD_TIMER_DL2_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1173, DFE_BB_CFG1173_REG_TXTDD_TIMER_DL2_23_16) << 16);
    arg->ul2Interval = CSL_FEXT(hDfeBb->regs->cfg1175, DFE_BB_CFG1175_REG_TXTDD_TIMER_UL2_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1176, DFE_BB_CFG1176_REG_TXTDD_TIMER_UL2_23_16) << 16);                 
    arg->dl3Interval = CSL_FEXT(hDfeBb->regs->cfg1177, DFE_BB_CFG1177_REG_TXTDD_TIMER_DL3_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1176, DFE_BB_CFG1176_REG_TXTDD_TIMER_DL3_23_16) << 16);
                    
    data = hDfeBb->regs->cfg1168;
    arg->enable = CSL_FEXT(data, DFE_BB_CFG1168_REG_TXTDD_EN);
    arg->dataMode = (DfeFl_BbTddDataMode)CSL_FEXT(data, DFE_BB_CFG1168_REG_TXTDD_DATAMODE);
    arg->carrierType = CSL_FEXT(data, DFE_BB_CFG1168_REG_TXTDD_CT_TYPE);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetTxTddSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG1168_REG_TXTDD_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetTxTddSsel(DfeFl_BbHandle hDfeBb, uint32_t ssel)
{
    CSL_FINS(hDfeBb->regs->cfg1168, DFE_BB_CFG1168_REG_TXTDD_SSEL, ssel); 
}

/** ============================================================================
 *   @n@b dfeFl_BbGetTxTddSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG1168_REG_TXTDD_SSEL
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
dfeFl_BbGetTxTddSsel(DfeFl_BbHandle hDfeBb, uint32_t *ssel)
{
    *ssel = CSL_FEXT(hDfeBb->regs->cfg1168, DFE_BB_CFG1168_REG_TXTDD_SSEL); 
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigRxTdd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG1184_REG_RXTDD_SSEL
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_BB_CFG1184_REG_RXTDD_SSEL
 *       DFE_BB_CFG1184_REG_RXTDD_DATAMODE
 *       DFE_BB_CFG1188_REG_RXTDD_TIMER_UL1_15_0
 *       DFE_BB_CFG1190_REG_RXTDD_TIMER_DL2_15_0
 *       DFE_BB_CFG1189_REG_RXTDD_TIMER_UL1_23_16
 *       DFE_BB_CFG1193_REG_RXTDD_TIMER_DL3_15_0
 *       DFE_BB_CFG1189_REG_RXTDD_TIMER_DL2_23_16
 *       DFE_BB_CFG1192_REG_RXTDD_TIMER_UL2_23_16
 *       DFE_BB_CFG1186_REG_RXTDD_TIMER_DL1_23_16
 *       DFE_BB_CFG1191_REG_RXTDD_TIMER_UL2_15_0
 *       DFE_BB_CFG1184_REG_RXTDD_CT_TYPE
 *       DFE_BB_CFG1186_REG_RXTDD_DELAYFROMSYNC_23_16
 *       DFE_BB_CFG1184_REG_RXTDD_EN
 *       DFE_BB_CFG1192_REG_RXTDD_TIMER_DL3_23_16
 *       DFE_BB_CFG1187_REG_RXTDD_TIMER_DL1_15_0
 *       DFE_BB_CFG1185_REG_RXTDD_DELAYFROMSYNC_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigRxTdd(DfeFl_BbHandle hDfeBb, DfeFl_BbTddConfig * arg)
{
    uint32_t data;

    hDfeBb->regs->cfg1185 = CSL_FMK(DFE_BB_CFG1185_REG_RXTDD_DELAYFROMSYNC_15_0, arg->syncDly);
    hDfeBb->regs->cfg1186 = CSL_FMK(DFE_BB_CFG1186_REG_RXTDD_DELAYFROMSYNC_23_16, arg->syncDly >> 16)
                    | CSL_FMK(DFE_BB_CFG1186_REG_RXTDD_TIMER_DL1_23_16, arg->dl1Interval >> 16);
    hDfeBb->regs->cfg1187 = CSL_FMK(DFE_BB_CFG1187_REG_RXTDD_TIMER_DL1_15_0, arg->dl1Interval);                 
    hDfeBb->regs->cfg1188 = CSL_FMK(DFE_BB_CFG1188_REG_RXTDD_TIMER_UL1_15_0, arg->ul1Interval);   
    hDfeBb->regs->cfg1189 = CSL_FMK(DFE_BB_CFG1189_REG_RXTDD_TIMER_UL1_23_16, arg->ul1Interval >> 16)              
                    | CSL_FMK(DFE_BB_CFG1189_REG_RXTDD_TIMER_DL2_23_16, arg->dl2Interval >> 16);
    hDfeBb->regs->cfg1190 = CSL_FMK(DFE_BB_CFG1190_REG_RXTDD_TIMER_DL2_15_0, arg->dl2Interval);                 
    hDfeBb->regs->cfg1191 = CSL_FMK(DFE_BB_CFG1191_REG_RXTDD_TIMER_UL2_15_0, arg->ul2Interval);   
    hDfeBb->regs->cfg1192 = CSL_FMK(DFE_BB_CFG1192_REG_RXTDD_TIMER_UL2_23_16, arg->ul2Interval >> 16)              
                    | CSL_FMK(DFE_BB_CFG1192_REG_RXTDD_TIMER_DL3_23_16, arg->dl3Interval >> 16);
    hDfeBb->regs->cfg1193 = CSL_FMK(DFE_BB_CFG1193_REG_RXTDD_TIMER_DL3_15_0, arg->dl3Interval); 
    
    // ssel
    data = CSL_FEXT(hDfeBb->regs->cfg1184, DFE_BB_CFG1184_REG_RXTDD_SSEL); 
    hDfeBb->regs->cfg1184 = CSL_FMK(DFE_BB_CFG1184_REG_RXTDD_EN, arg->enable)
                    | CSL_FMK(DFE_BB_CFG1184_REG_RXTDD_DATAMODE, arg->dataMode)
                    | CSL_FMK(DFE_BB_CFG1184_REG_RXTDD_CT_TYPE, arg->carrierType)
                    | CSL_FMK(DFE_BB_CFG1184_REG_RXTDD_SSEL, data);

                                    
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryRxTddConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG1184_REG_RXTDD_DATAMODE
 *       DFE_BB_CFG1188_REG_RXTDD_TIMER_UL1_15_0
 *       DFE_BB_CFG1190_REG_RXTDD_TIMER_DL2_15_0
 *       DFE_BB_CFG1189_REG_RXTDD_TIMER_UL1_23_16
 *       DFE_BB_CFG1193_REG_RXTDD_TIMER_DL3_15_0
 *       DFE_BB_CFG1189_REG_RXTDD_TIMER_DL2_23_16
 *       DFE_BB_CFG1192_REG_RXTDD_TIMER_UL2_23_16
 *       DFE_BB_CFG1186_REG_RXTDD_TIMER_DL1_23_16
 *       DFE_BB_CFG1191_REG_RXTDD_TIMER_UL2_15_0
 *       DFE_BB_CFG1184_REG_RXTDD_CT_TYPE
 *       DFE_BB_CFG1186_REG_RXTDD_DELAYFROMSYNC_23_16
 *       DFE_BB_CFG1184_REG_RXTDD_EN
 *       DFE_BB_CFG1192_REG_RXTDD_TIMER_DL3_23_16
 *       DFE_BB_CFG1187_REG_RXTDD_TIMER_DL1_15_0
 *       DFE_BB_CFG1185_REG_RXTDD_DELAYFROMSYNC_15_0
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
dfeFl_BbQueryRxTddConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbTddConfig * arg)
{
    uint32_t data;
    
    arg->syncDly = CSL_FEXT(hDfeBb->regs->cfg1185, DFE_BB_CFG1185_REG_RXTDD_DELAYFROMSYNC_15_0) 
                 |(CSL_FEXT(hDfeBb->regs->cfg1186, DFE_BB_CFG1186_REG_RXTDD_DELAYFROMSYNC_23_16) << 16);
    arg->dl1Interval = CSL_FEXT(hDfeBb->regs->cfg1187, DFE_BB_CFG1187_REG_RXTDD_TIMER_DL1_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1186, DFE_BB_CFG1186_REG_RXTDD_TIMER_DL1_23_16) << 16);
    arg->ul1Interval = CSL_FEXT(hDfeBb->regs->cfg1188, DFE_BB_CFG1188_REG_RXTDD_TIMER_UL1_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1189, DFE_BB_CFG1189_REG_RXTDD_TIMER_UL1_23_16) << 16);                 
    arg->dl2Interval = CSL_FEXT(hDfeBb->regs->cfg1190, DFE_BB_CFG1190_REG_RXTDD_TIMER_DL2_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1189, DFE_BB_CFG1189_REG_RXTDD_TIMER_DL2_23_16) << 16);
    arg->ul2Interval = CSL_FEXT(hDfeBb->regs->cfg1191, DFE_BB_CFG1191_REG_RXTDD_TIMER_UL2_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1192, DFE_BB_CFG1192_REG_RXTDD_TIMER_UL2_23_16) << 16);                 
    arg->dl3Interval = CSL_FEXT(hDfeBb->regs->cfg1193, DFE_BB_CFG1193_REG_RXTDD_TIMER_DL3_15_0)
                     |(CSL_FEXT(hDfeBb->regs->cfg1192, DFE_BB_CFG1192_REG_RXTDD_TIMER_DL3_23_16) << 16);
                    
    data = hDfeBb->regs->cfg1184;
    arg->enable = CSL_FEXT(data, DFE_BB_CFG1184_REG_RXTDD_EN);
    arg->dataMode = (DfeFl_BbTddDataMode)CSL_FEXT(data, DFE_BB_CFG1184_REG_RXTDD_DATAMODE);
    arg->carrierType = CSL_FEXT(data, DFE_BB_CFG1184_REG_RXTDD_CT_TYPE);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetRxTddSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG1184_REG_RXTDD_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetRxTddSsel(DfeFl_BbHandle hDfeBb, uint32_t ssel)
{
    CSL_FINS(hDfeBb->regs->cfg1184, DFE_BB_CFG1184_REG_RXTDD_SSEL, ssel); 
}

/** ============================================================================
 *   @n@b dfeFl_BbGetRxTddSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG1184_REG_RXTDD_SSEL
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
dfeFl_BbGetRxTddSsel(DfeFl_BbHandle hDfeBb, uint32_t *ssel)
{
    *ssel = CSL_FEXT(hDfeBb->regs->cfg1184, DFE_BB_CFG1184_REG_RXTDD_SSEL); 
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigTxpm
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_SYNCDLY_TXPM0_REG_SYNC_DLY_15_0
 *       DFE_BB_CFG_TXPM0_REG_IN_SOURCE
 *       DFE_BB_CFG_TXPM0_REG_OUT_FORMAT
 *       DFE_BB_SYNC_PWR_TXPM0_REG_PWR_UPDATE
 *       DFE_BB_PDINTERVAL_HI_TXPM0_REG_INTG_PD_23_16
 *       DFE_BB_CFG_TXPM0_REG_TDDMODE
 *       DFE_BB_INTERVAL_LO_TXPM0_REG_INTERVAL_15_0
 *       DFE_BB_CFG_TXPM0_REG_ENABLE
 *       DFE_BB_PDINTERVAL_HI_TXPM0_REG_INTERVAL_23_16
 *       DFE_BB_SYNC_PWR_TXPM0_REG_SYNC_DLY_23_16
 *       DFE_BB_CFG_TXPM0_REG_COUNT_SOURCE
 *       DFE_BB_PD_LO_TXPM0_REG_INTG_PD_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigTxpm(DfeFl_BbHandle  hDfeBb, DfeFl_BbPowerMeterConfig * arg)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg_txpm0 + arg->pmId * 8;
    
    // enable, out_format, ct, in_source, tdd_mode
    regs[0] = CSL_FMK(DFE_BB_CFG_TXPM0_REG_ENABLE, arg->enable)
            | CSL_FMK(DFE_BB_CFG_TXPM0_REG_OUT_FORMAT, arg->outFormat) 
            | CSL_FMK(DFE_BB_CFG_TXPM0_REG_COUNT_SOURCE, arg->countSource)
            | CSL_FMK(DFE_BB_CFG_TXPM0_REG_IN_SOURCE, arg->inSource)
            | CSL_FMK(DFE_BB_CFG_TXPM0_REG_TDDMODE, arg->tddMode);                    
    // sync dly [15:0]
    regs[1] = CSL_FMK(DFE_BB_SYNCDLY_TXPM0_REG_SYNC_DLY_15_0, arg->syncDly);
    // interval [15:0]
    regs[2] = CSL_FMK(DFE_BB_INTERVAL_LO_TXPM0_REG_INTERVAL_15_0, arg->interval);
    // interval [23:16], intg_pd [23:16]
    regs[3] = CSL_FMK(DFE_BB_PDINTERVAL_HI_TXPM0_REG_INTERVAL_23_16, arg->interval >> 16)
            | CSL_FMK(DFE_BB_PDINTERVAL_HI_TXPM0_REG_INTG_PD_23_16, arg->intgPd >> 16);
    // intg_pd [15:0]
    regs[4] = CSL_FMK(DFE_BB_PD_LO_TXPM0_REG_INTG_PD_15_0, arg->intgPd);
    // pwr_update, sync_dly [23:16]
    regs[5] = CSL_FMK(DFE_BB_SYNC_PWR_TXPM0_REG_PWR_UPDATE, arg->pwrUpdate)
            | CSL_FMK(DFE_BB_SYNC_PWR_TXPM0_REG_SYNC_DLY_23_16, arg->syncDly >> 16);
            
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryTxpmConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_SYNCDLY_TXPM0_REG_SYNC_DLY_15_0
 *       DFE_BB_CFG_TXPM0_REG_IN_SOURCE
 *       DFE_BB_CFG_TXPM0_REG_OUT_FORMAT
 *       DFE_BB_SYNC_PWR_TXPM0_REG_PWR_UPDATE
 *       DFE_BB_PDINTERVAL_HI_TXPM0_REG_INTG_PD_23_16
 *       DFE_BB_CFG_TXPM0_REG_TDDMODE
 *       DFE_BB_INTERVAL_LO_TXPM0_REG_INTERVAL_15_0
 *       DFE_BB_CFG_TXPM0_REG_ENABLE
 *       DFE_BB_PDINTERVAL_HI_TXPM0_REG_INTERVAL_23_16
 *       DFE_BB_SYNC_PWR_TXPM0_REG_SYNC_DLY_23_16
 *       DFE_BB_CFG_TXPM0_REG_COUNT_SOURCE
 *       DFE_BB_PD_LO_TXPM0_REG_INTG_PD_15_0
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
dfeFl_BbQueryTxpmConfig(DfeFl_BbHandle  hDfeBb, DfeFl_BbPowerMeterConfig * arg)
{
    uint32_t data;
    volatile uint32_t *regs = &hDfeBb->regs->cfg_txpm0 + arg->pmId * 8;
    
    // enable, out_format, ct, in_source, tdd_mode
    data = regs[0];
    arg->enable = (DfeFl_BbPowMtrEnable)CSL_FEXT(data, DFE_BB_CFG_TXPM0_REG_ENABLE);
    arg->outFormat = (DfeFl_BbPowMtrOutFormat)CSL_FEXT(data, DFE_BB_CFG_TXPM0_REG_OUT_FORMAT); 
    arg->countSource = CSL_FEXT(data, DFE_BB_CFG_TXPM0_REG_COUNT_SOURCE);
    arg->inSource = (DfeFl_BbPowMtrInSource)CSL_FEXT(data, DFE_BB_CFG_TXPM0_REG_IN_SOURCE);
    arg->tddMode = (DfeFl_BbPowMtrTddMode)CSL_FEXT(data, DFE_BB_CFG_TXPM0_REG_TDDMODE);
    
    // sync dly [15:0]
    data = regs[1];
    arg->syncDly = CSL_FEXT(data, DFE_BB_SYNCDLY_TXPM0_REG_SYNC_DLY_15_0);
    // interval [15:0]
    data = regs[2];
    arg->interval = CSL_FEXT(data, DFE_BB_INTERVAL_LO_TXPM0_REG_INTERVAL_15_0);
    // interval [23:16], intg_pd [23:16]
    data = regs[3];
    arg->interval |= CSL_FEXT(data, DFE_BB_PDINTERVAL_HI_TXPM0_REG_INTERVAL_23_16) << 16;
    arg->intgPd    = CSL_FEXT(data, DFE_BB_PDINTERVAL_HI_TXPM0_REG_INTG_PD_23_16) << 16; 
    // intg_pd [15:0]
    data = regs[4];
    arg->intgPd |= CSL_FEXT(data, DFE_BB_PD_LO_TXPM0_REG_INTG_PD_15_0);
                
    // pwr_update, sync_dly [23:16]    
    data = regs[5];
    arg->pwrUpdate = CSL_FEXT(data, DFE_BB_SYNC_PWR_TXPM0_REG_PWR_UPDATE);
    arg->syncDly  |= CSL_FEXT(data, DFE_BB_SYNC_PWR_TXPM0_REG_SYNC_DLY_23_16) << 16;
}

/** ============================================================================
 *   @n@b dfeFl_BbSetTxpmSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetTxpmSsel(DfeFl_BbHandle  hDfeBb, uint32_t pmId, uint32_t ssel)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg48;
    uint32_t r = pmId / 4;
    uint32_t b = (pmId % 4) * 4;
    
    CSL_FINSR(regs[r], b+3, b, ssel);    
}

/** ============================================================================
 *   @n@b dfeFl_BbGetTxpmSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbGetTxpmSsel(DfeFl_BbHandle  hDfeBb, uint32_t pmId, uint32_t *ssel)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg48;
    uint32_t r = pmId / 4;
    uint32_t b = (pmId % 4) * 4;
    
    *ssel = CSL_FEXTR(regs[r], b+3, b);    
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigRxpm
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG_RXPM0_REG_OUT_FORMAT
 *       DFE_BB_SYNC_PWR_RXPM0_REG_SYNC_DLY_23_16
 *       DFE_BB_PDINTERVAL_HI_RXPM0_REG_INTG_PD_23_16
 *       DFE_BB_PDINTERVAL_HI_RXPM0_REG_INTERVAL_23_16
 *       DFE_BB_INTERVAL_LO_RXPM0_REG_INTERVAL_15_0
 *       DFE_BB_SYNCDLY_RXPM0_REG_SYNC_DLY_15_0
 *       DFE_BB_SYNC_PWR_RXPM0_REG_PWR_UPDATE
 *       DFE_BB_RXPM_MAX_DB0_REG_MAX_DB
 *       DFE_BB_CFG_RXPM0_REG_COUNT_SOURCE
 *       DFE_BB_CFG_RXPM0_REG_TDDMODE
 *       DFE_BB_PD_LO_RXPM0_REG_INTG_PD_15_0
 *       DFE_BB_CFG_RXPM0_REG_IN_SOURCE
 *       DFE_BB_CFG_RXPM0_REG_ENABLE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigRxpm(DfeFl_BbHandle  hDfeBb, DfeFl_BbPowerMeterConfig * arg)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg_rxpm0 + arg->pmId * 8;
    
    // enable, out_format, ct, in_source, tdd_mode
    regs[0] = CSL_FMK(DFE_BB_CFG_RXPM0_REG_ENABLE, arg->enable)
            | CSL_FMK(DFE_BB_CFG_RXPM0_REG_OUT_FORMAT, arg->outFormat) 
            | CSL_FMK(DFE_BB_CFG_RXPM0_REG_COUNT_SOURCE, arg->countSource)
            | CSL_FMK(DFE_BB_CFG_RXPM0_REG_IN_SOURCE, arg->inSource)
            | CSL_FMK(DFE_BB_CFG_RXPM0_REG_TDDMODE, arg->tddMode);                    
    // sync dly [15:0]
    regs[1] = CSL_FMK(DFE_BB_SYNCDLY_RXPM0_REG_SYNC_DLY_15_0, arg->syncDly);
    // interval [15:0]
    regs[2] = CSL_FMK(DFE_BB_INTERVAL_LO_RXPM0_REG_INTERVAL_15_0, arg->interval);
    // interval [23:16], intg_pd [23:16]
    regs[3] = CSL_FMK(DFE_BB_PDINTERVAL_HI_RXPM0_REG_INTERVAL_23_16, arg->interval >> 16)
            | CSL_FMK(DFE_BB_PDINTERVAL_HI_RXPM0_REG_INTG_PD_23_16, arg->intgPd >> 16);
    // intg_pd [15:0]
    regs[4] = CSL_FMK(DFE_BB_PD_LO_RXPM0_REG_INTG_PD_15_0, arg->intgPd);
    // pwr_update, sync_dly [23:16]
    regs[5] = CSL_FMK(DFE_BB_SYNC_PWR_RXPM0_REG_PWR_UPDATE, arg->pwrUpdate)
            | CSL_FMK(DFE_BB_SYNC_PWR_RXPM0_REG_SYNC_DLY_23_16, arg->syncDly >> 16);
    // max_db
    regs[6] = CSL_FMK(DFE_BB_RXPM_MAX_DB0_REG_MAX_DB, arg->maxDb);    
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryRxpmConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG_RXPM0_REG_OUT_FORMAT
 *       DFE_BB_SYNC_PWR_RXPM0_REG_SYNC_DLY_23_16
 *       DFE_BB_PDINTERVAL_HI_RXPM0_REG_INTG_PD_23_16
 *       DFE_BB_PDINTERVAL_HI_RXPM0_REG_INTERVAL_23_16
 *       DFE_BB_INTERVAL_LO_RXPM0_REG_INTERVAL_15_0
 *       DFE_BB_SYNCDLY_RXPM0_REG_SYNC_DLY_15_0
 *       DFE_BB_SYNC_PWR_RXPM0_REG_PWR_UPDATE
 *       DFE_BB_RXPM_MAX_DB0_REG_MAX_DB
 *       DFE_BB_CFG_RXPM0_REG_COUNT_SOURCE
 *       DFE_BB_CFG_RXPM0_REG_TDDMODE
 *       DFE_BB_PD_LO_RXPM0_REG_INTG_PD_15_0
 *       DFE_BB_CFG_RXPM0_REG_IN_SOURCE
 *       DFE_BB_CFG_RXPM0_REG_ENABLE
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
dfeFl_BbQueryRxpmConfig(DfeFl_BbHandle  hDfeBb, DfeFl_BbPowerMeterConfig * arg)
{
    uint32_t data;
    volatile uint32_t *regs = &hDfeBb->regs->cfg_rxpm0 + arg->pmId * 8;
    
    // enable, out_format, ct, in_source, tdd_mode
    data = regs[0];
    arg->enable = (DfeFl_BbPowMtrEnable)CSL_FEXT(data, DFE_BB_CFG_RXPM0_REG_ENABLE);
    arg->outFormat = (DfeFl_BbPowMtrOutFormat)CSL_FEXT(data, DFE_BB_CFG_RXPM0_REG_OUT_FORMAT); 
    arg->countSource = CSL_FEXT(data, DFE_BB_CFG_RXPM0_REG_COUNT_SOURCE);
    arg->inSource = (DfeFl_BbPowMtrInSource)CSL_FEXT(data, DFE_BB_CFG_RXPM0_REG_IN_SOURCE);
    arg->tddMode = (DfeFl_BbPowMtrTddMode)CSL_FEXT(data, DFE_BB_CFG_RXPM0_REG_TDDMODE);
    
    // sync dly [15:0]
    data = regs[1];
    arg->syncDly = CSL_FEXT(data, DFE_BB_SYNCDLY_RXPM0_REG_SYNC_DLY_15_0);
    // interval [15:0]
    data = regs[2];
    arg->interval = CSL_FEXT(data, DFE_BB_INTERVAL_LO_RXPM0_REG_INTERVAL_15_0);
    // interval [23:16], intg_pd [23:16]
    data = regs[3];
    arg->interval |= CSL_FEXT(data, DFE_BB_PDINTERVAL_HI_RXPM0_REG_INTERVAL_23_16) << 16;
    arg->intgPd    = CSL_FEXT(data, DFE_BB_PDINTERVAL_HI_RXPM0_REG_INTG_PD_23_16) << 16; 
    // intg_pd [15:0]
    data = regs[4];
    arg->intgPd |= CSL_FEXT(data, DFE_BB_PD_LO_RXPM0_REG_INTG_PD_15_0);
                
    // pwr_update, sync_dly [23:16]    
    data = regs[5];
    arg->pwrUpdate = CSL_FEXT(data, DFE_BB_SYNC_PWR_RXPM0_REG_PWR_UPDATE);
    arg->syncDly  |= CSL_FEXT(data, DFE_BB_SYNC_PWR_RXPM0_REG_SYNC_DLY_23_16) << 16;
    // max_db
    data = regs[6];
    arg->maxDb = CSL_FEXT(data, DFE_BB_RXPM_MAX_DB0_REG_MAX_DB);    

}

/** ============================================================================
 *   @n@b dfeFl_BbSetRxpmSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetRxpmSsel(DfeFl_BbHandle  hDfeBb, uint32_t pmId, uint32_t ssel)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg52;
    uint32_t r = pmId / 4;
    uint32_t b = (pmId % 4) * 4;
    
    CSL_FINSR(regs[r], b+3, b, ssel);    
}

/** ============================================================================
 *   @n@b dfeFl_BbGetRxpmSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbGetRxpmSsel(DfeFl_BbHandle  hDfeBb, uint32_t pmId, uint32_t *ssel)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg52;
    uint32_t r = pmId / 4;
    uint32_t b = (pmId % 4) * 4;
    
    *ssel = CSL_FEXTR(regs[r], b+3, b);    
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableTxpmUpdate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbDisableTxpmUpdate(DfeFl_BbHandle hDfeBb, DfeFl_BbDisablePowMterUpdateConfig * arg)
{
    CSL_FINSR(hDfeBb->regs->txpm_update_disable, arg->pmId, arg->pmId, arg->disableUpdate);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryDisableTxpmUpdate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbQueryDisableTxpmUpdate(DfeFl_BbHandle hDfeBb, DfeFl_BbDisablePowMterUpdateConfig * arg)
{
    arg->disableUpdate = CSL_FEXTR(hDfeBb->regs->txpm_update_disable, arg->pmId, arg->pmId);
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableRxpmUpdate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbDisableRxpmUpdate(DfeFl_BbHandle hDfeBb, DfeFl_BbDisablePowMterUpdateConfig * arg)
{
    CSL_FINSR(hDfeBb->regs->rxpm_update_disable, arg->pmId, arg->pmId, arg->disableUpdate);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryDisableRxpmUpdate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbQueryDisableRxpmUpdate(DfeFl_BbHandle hDfeBb, DfeFl_BbDisablePowMterUpdateConfig * arg)
{
    arg->disableUpdate = CSL_FEXTR(hDfeBb->regs->rxpm_update_disable, arg->pmId, arg->pmId);
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableTxpmIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbEnableTxpmIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->intmask0, pmId, pmId, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableTxpmIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbDisableTxpmIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->intmask0, pmId, pmId, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableTxpmAuxIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbEnableTxpmAuxIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->txpm_auxint_mask, pmId, pmId, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableTxpmAuxIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbDisableTxpmAuxIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->txpm_auxint_mask, pmId, pmId, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableRxpmIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbEnableRxpmIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->intmask1, pmId, pmId, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableRxpmIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbDisableRxpmIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->intmask1, pmId, pmId, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableRxpmAuxIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbEnableRxpmAuxIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->rxpm_auxint_mask, pmId, pmId, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableRxpmAuxIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbDisableRxpmAuxIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->rxpm_auxint_mask, pmId, pmId, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearTxpmIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbClearTxpmIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    hDfeBb->regs->intstatus0 = ~(1u << pmId);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearRxpmIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbClearRxpmIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    hDfeBb->regs->intstatus1 = ~(1u << pmId);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryTxpmIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
         status    [add content]
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
dfeFl_BbQueryTxpmIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t pmId, uint32_t *status)
{
    *status = CSL_FEXTR(hDfeBb->regs->intstatus0, pmId, pmId);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryRxpmIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
         status    [add content]
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
dfeFl_BbQueryRxpmIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t pmId, uint32_t *status)
{
    *status = CSL_FEXTR(hDfeBb->regs->intstatus1, pmId, pmId);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetForceTxpmIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbSetForceTxpmIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->intforce0, pmId, pmId, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearForceTxpmIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbClearForceTxpmIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->intforce0, pmId, pmId, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetForceRxpmIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbSetForceRxpmIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->intforce1, pmId, pmId, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearForceRxpmIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         pmId    [add content]
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
dfeFl_BbClearForceRxpmIntr(DfeFl_BbHandle hDfeBb, uint32_t pmId)
{
    CSL_FINSR(hDfeBb->regs->intforce1, pmId, pmId, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryTxpmResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbQueryTxpmResult(DfeFl_BbHandle hDfeBb, DfeFl_BbPowMtrResult *arg)
{
	CSL_DFE_BB_BBTXPWRMETER_REGS data;

	data = hDfeBb->regs->bbtxpwrmeter[arg->pmId];
	arg->peakPower = data.r0;
	arg->peakPower_extend = data.r1;
	arg->rmsPower = data.r2;
	arg->rmsPower_extend = data.r3;
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryRxpmResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
dfeFl_BbQueryRxpmResult(DfeFl_BbHandle hDfeBb, DfeFl_BbPowMtrResult *arg)
{
	CSL_DFE_BB_BBRXPWRMETER_REGS data;

	data = hDfeBb->regs->bbrxpwrmeter[arg->pmId];
	arg->peakPower = data.r0;
	arg->peakPower_extend = data.r1;
	arg->rmsPower = data.r2;
	arg->rmsPower_extend = data.r3;
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigAntcalGlobal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG120_REG_ANTCAL_INTERVAL_15_0
 *       DFE_BB_CFG121_REG_ANTCAL_INTERVAL_31_16
 *       DFE_BB_CFG122_REG_ANTCAL_TX_CT_SEL
 *       DFE_BB_CFG122_REG_ANTCAL_RX_SSEL
 *       DFE_BB_CFG123_REG_ANTCAL_EN
 *       DFE_BB_CFG122_REG_ANTCAL_TX_SSEL
 *       DFE_BB_CFG122_REG_ANTCAL_RX_CT_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigAntcalGlobal(DfeFl_BbHandle hDfeBb, DfeFl_BbAntCalGlobalConfig * arg)
{
    hDfeBb->regs->cfg120 = CSL_FMK(DFE_BB_CFG120_REG_ANTCAL_INTERVAL_15_0, arg->interval);
    hDfeBb->regs->cfg121 = CSL_FMK(DFE_BB_CFG121_REG_ANTCAL_INTERVAL_31_16, arg->interval >> 16);
    hDfeBb->regs->cfg122 = CSL_FMK(DFE_BB_CFG122_REG_ANTCAL_TX_CT_SEL, arg->txCarrierTypeSel)
                         | CSL_FMK(DFE_BB_CFG122_REG_ANTCAL_RX_CT_SEL, arg->rxCarrierTypeSel)
                         | CSL_FMK(DFE_BB_CFG122_REG_ANTCAL_TX_SSEL, arg->txSsel)
                         | CSL_FMK(DFE_BB_CFG122_REG_ANTCAL_RX_SSEL, arg->rxSsel);
                         
    hDfeBb->regs->cfg123 = CSL_FMK(DFE_BB_CFG123_REG_ANTCAL_EN, arg->enable);                     
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryAntcalGlobalConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG122_REG_ANTCAL_RX_SSEL
 *       DFE_BB_CFG123_REG_ANTCAL_EN
 *       DFE_BB_CFG122_REG_ANTCAL_TX_CT_SEL
 *       DFE_BB_CFG122_REG_ANTCAL_TX_SSEL
 *       DFE_BB_CFG122_REG_ANTCAL_RX_CT_SEL
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
dfeFl_BbQueryAntcalGlobalConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbAntCalGlobalConfig * arg)
{
    uint32_t data;
    arg->interval = hDfeBb->regs->cfg120 | (hDfeBb->regs->cfg121 << 16);
    
    data = hDfeBb->regs->cfg122;
    arg->txCarrierTypeSel = CSL_FEXT(data, DFE_BB_CFG122_REG_ANTCAL_TX_CT_SEL);
    arg->rxCarrierTypeSel = CSL_FEXT(data, DFE_BB_CFG122_REG_ANTCAL_RX_CT_SEL);
    arg->txSsel = CSL_FEXT(data, DFE_BB_CFG122_REG_ANTCAL_TX_SSEL);
    arg->rxSsel = CSL_FEXT(data, DFE_BB_CFG122_REG_ANTCAL_RX_SSEL);
                         
    data = hDfeBb->regs->cfg123;
    arg->enable = CSL_FEXT(data, DFE_BB_CFG123_REG_ANTCAL_EN);                     
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigAntcal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_ANTCAL_TX_NOISE_REG_ANTCAL_TX_NOISE
 *       DFE_BB_ANTCAL_RX_REG_ANTCAL_RX_CORR_DELAY
 *       DFE_BB_ANTCAL_PN_TAPCONFIG_REG_ANTCAL_PN_TAPCONFIG
 *       DFE_BB_ANTCAL_RX_REG_ANTCAL_RX_OVERSAMPLE
 *       DFE_BB_ANTCAL_PN_INIT_REG_ANTCAL_PN_INIT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigAntcal(DfeFl_BbHandle hDfeBb, DfeFl_BbAntCalConfig * arg)
{
    hDfeBb->regs->antcal[arg->antcal].pn_init = 
        CSL_FMK(DFE_BB_ANTCAL_PN_INIT_REG_ANTCAL_PN_INIT, arg->pnInit);
    hDfeBb->regs->antcal[arg->antcal].pn_tapconfig = 
        CSL_FMK(DFE_BB_ANTCAL_PN_TAPCONFIG_REG_ANTCAL_PN_TAPCONFIG, arg->pnTapConfig);
    hDfeBb->regs->antcal[arg->antcal].tx_noise = 
        CSL_FMK(DFE_BB_ANTCAL_TX_NOISE_REG_ANTCAL_TX_NOISE, arg->txNoise);
    hDfeBb->regs->antcal[arg->antcal].rx \
      = CSL_FMK(DFE_BB_ANTCAL_RX_REG_ANTCAL_RX_CORR_DELAY, arg->rxCorrDelay) 
      | CSL_FMK(DFE_BB_ANTCAL_RX_REG_ANTCAL_RX_OVERSAMPLE, arg->rxOverSample);            
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryAntcalConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_ANTCAL_TX_NOISE_REG_ANTCAL_TX_NOISE
 *       DFE_BB_ANTCAL_RX_REG_ANTCAL_RX_CORR_DELAY
 *       DFE_BB_ANTCAL_PN_TAPCONFIG_REG_ANTCAL_PN_TAPCONFIG
 *       DFE_BB_ANTCAL_RX_REG_ANTCAL_RX_OVERSAMPLE
 *       DFE_BB_ANTCAL_PN_INIT_REG_ANTCAL_PN_INIT
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
dfeFl_BbQueryAntcalConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbAntCalConfig * arg)
{
    uint32_t data;
    
    data = hDfeBb->regs->antcal[arg->antcal].pn_init;
    arg->pnInit = CSL_FEXT(data, DFE_BB_ANTCAL_PN_INIT_REG_ANTCAL_PN_INIT);
    
    data = hDfeBb->regs->antcal[arg->antcal].pn_tapconfig;
    arg->pnTapConfig = CSL_FEXT(data, DFE_BB_ANTCAL_PN_TAPCONFIG_REG_ANTCAL_PN_TAPCONFIG);
    
    data = hDfeBb->regs->antcal[arg->antcal].tx_noise;
    arg->txNoise = CSL_FEXT(data, DFE_BB_ANTCAL_TX_NOISE_REG_ANTCAL_TX_NOISE);
    
    data = hDfeBb->regs->antcal[arg->antcal].rx;
    arg->rxCorrDelay = CSL_FEXT(data, DFE_BB_ANTCAL_RX_REG_ANTCAL_RX_CORR_DELAY); 
    arg->rxOverSample = CSL_FEXT(data, DFE_BB_ANTCAL_RX_REG_ANTCAL_RX_OVERSAMPLE);            
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableGeneralIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intr    [add content]
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
dfeFl_BbEnableGeneralIntr(DfeFl_BbHandle hDfeBb, uint32_t intr)
{
    if(intr < 16)
        CSL_FINSR(hDfeBb->regs->intmask4, intr, intr, 1);
    else
        CSL_FINSR(hDfeBb->regs->intmask4a, intr-16, intr-16, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableGeneralIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intr    [add content]
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
dfeFl_BbDisableGeneralIntr(DfeFl_BbHandle hDfeBb, uint32_t intr)
{
    if(intr < 16)
        CSL_FINSR(hDfeBb->regs->intmask4, intr, intr, 0);
    else
        CSL_FINSR(hDfeBb->regs->intmask4a, intr-16, intr-16, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearGeneralIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intr    [add content]
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
dfeFl_BbClearGeneralIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t intr)
{
    if(intr < 16)
        hDfeBb->regs->intstatus4 = ~(1u << intr);
    else        
        hDfeBb->regs->intstatus4a = ~(1u << (intr - 16));
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryGeneralIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intr    [add content]
         status    [add content]
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
dfeFl_BbQueryGeneralIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t intr, uint32_t *status)
{
    if(intr < 16)    
        *status = CSL_FEXTR(hDfeBb->regs->intstatus4, intr, intr);
    else
        *status = CSL_FEXTR(hDfeBb->regs->intstatus4a, intr-16, intr-16);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetForceGeneralIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intr    [add content]
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
dfeFl_BbSetForceGeneralIntr(DfeFl_BbHandle hDfeBb, uint32_t intr)
{
    if(intr < 16)    
        CSL_FINSR(hDfeBb->regs->intforce4, intr, intr, 1);
    else
        CSL_FINSR(hDfeBb->regs->intforce4a, intr-16, intr-16, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearForceGeneralIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intr    [add content]
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
dfeFl_BbClearForceGeneralIntr(DfeFl_BbHandle hDfeBb, uint32_t intr)
{
    if(intr < 16)    
        CSL_FINSR(hDfeBb->regs->intforce4, intr, intr, 0);
    else
        CSL_FINSR(hDfeBb->regs->intforce4a, intr-16, intr-16, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableGeneralIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intrGrp    [add content]
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
dfeFl_BbEnableGeneralIntrGroup(DfeFl_BbHandle hDfeBb, DfeFl_BbGeneralIntrGroup *intrGrp)
{
    uint32_t data = 0;
    uint32_t data2 = 0;

    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, intrGrp->txpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, intrGrp->rxpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_ANTCAL, DFE_FL_BB_GENERAL_INTR_ANTCAL, intrGrp->antcal);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, intrGrp->rxNotchDone);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, intrGrp->rxNotchErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, intrGrp->bufErr[0]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, intrGrp->bufErr[1]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, intrGrp->bufErr[2]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, intrGrp->bufErr[3]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, intrGrp->bufErr[4]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, intrGrp->bufErr[5]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, intrGrp->bufErr[6]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, intrGrp->bufErr[7]);
            
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, intrGrp->rxaidSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, intrGrp->txaidUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, intrGrp->txaidOverflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, intrGrp->jesdrxSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, intrGrp->jesdtxUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, intrGrp->jesdtxOverflow);

    hDfeBb->regs->intmask4 |= data;
    hDfeBb->regs->intmask4a |= data2;
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableGeneralIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intrGrp    [add content]
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
dfeFl_BbDisableGeneralIntrGroup(DfeFl_BbHandle hDfeBb, DfeFl_BbGeneralIntrGroup *intrGrp)
{
    uint32_t data = 0;
    uint32_t data2 = 0;

    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, intrGrp->txpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, intrGrp->rxpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_ANTCAL, DFE_FL_BB_GENERAL_INTR_ANTCAL, intrGrp->antcal);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, intrGrp->rxNotchDone);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, intrGrp->rxNotchErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, intrGrp->bufErr[0]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, intrGrp->bufErr[1]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, intrGrp->bufErr[2]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, intrGrp->bufErr[3]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, intrGrp->bufErr[4]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, intrGrp->bufErr[5]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, intrGrp->bufErr[6]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, intrGrp->bufErr[7]);
            
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, intrGrp->rxaidSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, intrGrp->txaidUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, intrGrp->txaidOverflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, intrGrp->jesdrxSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, intrGrp->jesdtxUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, intrGrp->jesdtxOverflow);

    hDfeBb->regs->intmask4 &= ~data;
    hDfeBb->regs->intmask4a &= ~data2;
}

/** ============================================================================
 *   @n@b dfeFl_BbClearGeneralIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intrGrp    [add content]
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
dfeFl_BbClearGeneralIntrGroup(DfeFl_BbHandle hDfeBb, DfeFl_BbGeneralIntrGroup *intrGrp)
{
    uint32_t data = 0;
    uint32_t data2 = 0;

    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, intrGrp->txpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, intrGrp->rxpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_ANTCAL, DFE_FL_BB_GENERAL_INTR_ANTCAL, intrGrp->antcal);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, intrGrp->rxNotchDone);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, intrGrp->rxNotchErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, intrGrp->bufErr[0]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, intrGrp->bufErr[1]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, intrGrp->bufErr[2]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, intrGrp->bufErr[3]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, intrGrp->bufErr[4]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, intrGrp->bufErr[5]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, intrGrp->bufErr[6]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, intrGrp->bufErr[7]);
            
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, intrGrp->rxaidSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, intrGrp->txaidUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, intrGrp->txaidOverflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, intrGrp->jesdrxSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, intrGrp->jesdtxUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, intrGrp->jesdtxOverflow);

    hDfeBb->regs->intstatus4 = ~data;
    hDfeBb->regs->intstatus4a = ~data2;
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryGeneralIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intrGrp    [add content]
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
dfeFl_BbQueryGeneralIntrGroup(DfeFl_BbHandle hDfeBb, DfeFl_BbGeneralIntrGroup *intrGrp)
{
    uint32_t data = hDfeBb->regs->intstatus4;
    uint32_t data2 = hDfeBb->regs->intstatus4a;

    intrGrp->txpmLoadErr = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR);
    intrGrp->rxpmLoadErr = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR);
    intrGrp->antcal = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_ANTCAL, DFE_FL_BB_GENERAL_INTR_ANTCAL);
    intrGrp->rxNotchDone = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE);
    intrGrp->rxNotchErr = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR);
    intrGrp->bufErr[0] = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF);
    intrGrp->bufErr[1] = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF);
    intrGrp->bufErr[2] = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF);
    intrGrp->bufErr[3] = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF);
    intrGrp->bufErr[4] = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF);
    intrGrp->bufErr[5] = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF);
    intrGrp->bufErr[6] = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF);
    intrGrp->bufErr[7] = CSL_FEXTR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF);

    intrGrp->rxaidSyncErr = CSL_FEXTR(data2, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16);
    intrGrp->txaidUnderflow = CSL_FEXTR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16);
    intrGrp->txaidOverflow = CSL_FEXTR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16);
    intrGrp->jesdrxSyncErr = CSL_FEXTR(data2, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16);
    intrGrp->jesdtxUnderflow = CSL_FEXTR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16);
    intrGrp->jesdtxOverflow = CSL_FEXTR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16);

}

/** ============================================================================
 *   @n@b dfeFl_BbSetForceGeneralIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intrGrp    [add content]
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
dfeFl_BbSetForceGeneralIntrGroup(DfeFl_BbHandle hDfeBb, DfeFl_BbGeneralIntrGroup *intrGrp)
{
    uint32_t data = 0;
    uint32_t data2 = 0;

    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, intrGrp->txpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, intrGrp->rxpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_ANTCAL, DFE_FL_BB_GENERAL_INTR_ANTCAL, intrGrp->antcal);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, intrGrp->rxNotchDone);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, intrGrp->rxNotchErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, intrGrp->bufErr[0]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, intrGrp->bufErr[1]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, intrGrp->bufErr[2]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, intrGrp->bufErr[3]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, intrGrp->bufErr[4]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, intrGrp->bufErr[5]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, intrGrp->bufErr[6]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, intrGrp->bufErr[7]);
            
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, intrGrp->rxaidSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, intrGrp->txaidUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, intrGrp->txaidOverflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, intrGrp->jesdrxSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, intrGrp->jesdtxUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, intrGrp->jesdtxOverflow);

    hDfeBb->regs->intforce4 |= data;
    hDfeBb->regs->intforce4a |= data2;
}

/** ============================================================================
 *   @n@b dfeFl_BbClearForceGeneralIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         intrGrp    [add content]
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
dfeFl_BbClearForceGeneralIntrGroup(DfeFl_BbHandle hDfeBb, DfeFl_BbGeneralIntrGroup *intrGrp)
{
    uint32_t data = 0;
    uint32_t data2 = 0;

    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, DFE_FL_BB_GENERAL_INTR_TXPM_LDERR, intrGrp->txpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, DFE_FL_BB_GENERAL_INTR_RXPM_LDERR, intrGrp->rxpmLoadErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_ANTCAL, DFE_FL_BB_GENERAL_INTR_ANTCAL, intrGrp->antcal);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, DFE_FL_BB_GENERAL_INTR_RXNOTCH_DONE, intrGrp->rxNotchDone);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, DFE_FL_BB_GENERAL_INTR_RXNOTCH_ERR, intrGrp->rxNotchErr);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM0_OUF, intrGrp->bufErr[0]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM1_OUF, intrGrp->bufErr[1]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM2_OUF, intrGrp->bufErr[2]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM3_OUF, intrGrp->bufErr[3]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM4_OUF, intrGrp->bufErr[4]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM5_OUF, intrGrp->bufErr[5]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM6_OUF, intrGrp->bufErr[6]);
    CSL_FINSR(data, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, DFE_FL_BB_GENERAL_INTR_BUFMEM7_OUF, intrGrp->bufErr[7]);
            
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_RXAID_SYNCERR-16, intrGrp->rxaidSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, DFE_FL_BB_GENERAL_INTR_TXAID_UDF-16, intrGrp->txaidUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, DFE_FL_BB_GENERAL_INTR_TXAID_OVF-16, intrGrp->txaidOverflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, DFE_FL_BB_GENERAL_INTR_JESDRX_SYNCERR-16, intrGrp->jesdrxSyncErr);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_UDF-16, intrGrp->jesdtxUnderflow);
    CSL_FINSR(data2, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, DFE_FL_BB_GENERAL_INTR_JESDTX_OVF-16, intrGrp->jesdtxOverflow);

    hDfeBb->regs->intforce4 &= ~data;
    hDfeBb->regs->intforce4a &= ~data2;
}

/** ============================================================================
 *   @n@b dfeFl_BbGetRxGainUpdateStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
         status    [add content]
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
dfeFl_BbGetRxGainUpdateStatus(DfeFl_BbHandle hDfeBb, uint32_t ct, uint32_t *status)
{
    *status = CSL_FEXTR(hDfeBb->regs->cfg84, ct, ct);
}

/** ============================================================================
 *   @n@b dfeFl_BbUpdateRxGain
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         axc    [add content]
         gainInteger    [add content]
         gainFraction    [add content]
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
 *       DFE_BB_BBRXGAIN_FRACTION_REG_FRACTION
 *       DFE_BB_BBRXGAIN_INTEGER_REG_INTEGER
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbUpdateRxGain(DfeFl_BbHandle hDfeBb, uint32_t axc, uint32_t gainInteger, uint32_t gainFraction)
{
    hDfeBb->regs->bbrxgain[axc].integer = CSL_FMK(DFE_BB_BBRXGAIN_INTEGER_REG_INTEGER, gainInteger);        
    hDfeBb->regs->bbrxgain[axc].fraction = CSL_FMK(DFE_BB_BBRXGAIN_FRACTION_REG_FRACTION, gainFraction);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryRxGain
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         axc    [add content]
         gainInteger    [add content]
         gainFraction    [add content]
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
 *       DFE_BB_BBRXGAIN_FRACTION_REG_FRACTION
 *       DFE_BB_BBRXGAIN_INTEGER_REG_INTEGER
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
dfeFl_BbQueryRxGain(DfeFl_BbHandle hDfeBb, uint32_t axc, uint32_t *gainInteger, uint32_t *gainFraction)
{
    *gainInteger = CSL_FEXT(hDfeBb->regs->bbrxgain[axc].integer, DFE_BB_BBRXGAIN_INTEGER_REG_INTEGER);        
    *gainFraction = CSL_FEXT(hDfeBb->regs->bbrxgain[axc].fraction, DFE_BB_BBRXGAIN_FRACTION_REG_FRACTION);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetRxGainCtSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetRxGainCtSsel(DfeFl_BbHandle hDfeBb, uint32_t ct, uint32_t ssel)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg56;
    uint32_t r = ct / 4;
    uint32_t b = (ct % 4) * 4;
    
    CSL_FINSR(regs[r], b+3, b, ssel);    
}

/** ============================================================================
 *   @n@b dfeFl_BbGetRxGainCtSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbGetRxGainCtSsel(DfeFl_BbHandle hDfeBb, uint32_t ct, uint32_t *ssel)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg56;
    uint32_t r = ct / 4;
    uint32_t b = (ct % 4) * 4;
    
    *ssel = CSL_FEXTR(regs[r], b+3, b);    
}

/** ============================================================================
 *   @n@b dfeFl_BbEnableRxGainIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbEnableRxGainIntr(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    CSL_FINSR(hDfeBb->regs->intmask3, ct, ct, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbDisableRxGainIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbDisableRxGainIntr(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    CSL_FINSR(hDfeBb->regs->intmask3, ct, ct, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearRxGainIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbClearRxGainIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    hDfeBb->regs->intstatus3 = ~(1u << ct);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryRxGainIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
         status    [add content]
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
dfeFl_BbQueryRxGainIntrStatus(DfeFl_BbHandle hDfeBb, uint32_t ct, uint32_t *status)
{
    *status = CSL_FEXTR(hDfeBb->regs->intstatus3, ct, ct);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetForceRxGainIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbSetForceRxGainIntr(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    CSL_FINSR(hDfeBb->regs->intforce3, ct, ct, 1);
}

/** ============================================================================
 *   @n@b dfeFl_BbClearForceRxGainIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         ct    [add content]
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
dfeFl_BbClearForceRxGainIntr(DfeFl_BbHandle hDfeBb, uint32_t ct)
{
    CSL_FINSR(hDfeBb->regs->intforce3, ct, ct, 0);
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigRxNotchGlobal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         carrierType    [add content]
         tddMode    [add content]
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
 *       DFE_BB_CFG119_REG_RXNOTCH_CT
 *       DFE_BB_CFG119_REG_RXNOTCH_TDDMODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigRxNotchGlobal(DfeFl_BbHandle hDfeBb, uint32_t carrierType, uint32_t tddMode)
{
    uint32_t data = hDfeBb->regs->cfg119;
    
    CSL_FINS(data, DFE_BB_CFG119_REG_RXNOTCH_CT, carrierType);
    CSL_FINS(data, DFE_BB_CFG119_REG_RXNOTCH_TDDMODE, tddMode);
    
    hDfeBb->regs->cfg119 = data;
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryRxNotchGlobalConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         carrierType    [add content]
         tddMode    [add content]
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
 *       DFE_BB_CFG119_REG_RXNOTCH_CT
 *       DFE_BB_CFG119_REG_RXNOTCH_TDDMODE
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
dfeFl_BbQueryRxNotchGlobalConfig(DfeFl_BbHandle hDfeBb, uint32_t *carrierType, uint32_t *tddMode)
{
    uint32_t data = hDfeBb->regs->cfg119;
    
    *carrierType = CSL_FEXT(data, DFE_BB_CFG119_REG_RXNOTCH_CT);
    *tddMode = CSL_FEXT(data, DFE_BB_CFG119_REG_RXNOTCH_TDDMODE);
}

/** ============================================================================
 *   @n@b dfeFl_BbSetRxNotchSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG119_REG_RXNOTCH_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetRxNotchSsel(DfeFl_BbHandle hDfeBb, uint32_t ssel)
{
    CSL_FINS(hDfeBb->regs->cfg119, DFE_BB_CFG119_REG_RXNOTCH_SSEL, ssel);
}

/** ============================================================================
 *   @n@b dfeFl_BbGetRxNotchSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
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
 *       DFE_BB_CFG119_REG_RXNOTCH_SSEL
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
dfeFl_BbGetRxNotchSsel(DfeFl_BbHandle hDfeBb, uint32_t *ssel)
{
    *ssel = CSL_FEXT(hDfeBb->regs->cfg119, DFE_BB_CFG119_REG_RXNOTCH_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigRxNotch
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         cfg    [add content]
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
 *       DFE_BB_RXNOTCH_CONFIG2_REG_FILTER3
 *       DFE_BB_RXNOTCH_CONFIG2_REG_FILTER4
 *       DFE_BB_RXNOTCH_CONFIG1_REG_FILTER1
 *       DFE_BB_RXNOTCH_CONFIG1_REG_AXC_MODE
 *       DFE_BB_RXNOTCH_CONFIG1_REG_FILTER2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigRxNotch(DfeFl_BbHandle hDfeBb, DfeFl_BbRxNotch *cfg)
{
    hDfeBb->regs->rxnotch[cfg->axc].config1 \
        = CSL_FMK(DFE_BB_RXNOTCH_CONFIG1_REG_AXC_MODE, cfg->mode)
        | CSL_FMK(DFE_BB_RXNOTCH_CONFIG1_REG_FILTER1, cfg->filter1)
        | CSL_FMK(DFE_BB_RXNOTCH_CONFIG1_REG_FILTER2, cfg->filter2);
        
    hDfeBb->regs->rxnotch[cfg->axc].config2 \
        = CSL_FMK(DFE_BB_RXNOTCH_CONFIG2_REG_FILTER3, cfg->filter3)
        | CSL_FMK(DFE_BB_RXNOTCH_CONFIG2_REG_FILTER4, cfg->filter4);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryRxNotchConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         cfg    [add content]
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
 *       DFE_BB_RXNOTCH_CONFIG2_REG_FILTER3
 *       DFE_BB_RXNOTCH_CONFIG2_REG_FILTER4
 *       DFE_BB_RXNOTCH_CONFIG1_REG_FILTER1
 *       DFE_BB_RXNOTCH_CONFIG1_REG_AXC_MODE
 *       DFE_BB_RXNOTCH_CONFIG1_REG_FILTER2
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
dfeFl_BbQueryRxNotchConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbRxNotch *cfg)
{
    uint32_t data;
    data = hDfeBb->regs->rxnotch[cfg->axc].config1;
    cfg->mode = CSL_FEXT(data, DFE_BB_RXNOTCH_CONFIG1_REG_AXC_MODE);
    cfg->filter1 = CSL_FEXT(data, DFE_BB_RXNOTCH_CONFIG1_REG_FILTER1);
    cfg->filter2  = CSL_FEXT(data, DFE_BB_RXNOTCH_CONFIG1_REG_FILTER2);
        
    data = hDfeBb->regs->rxnotch[cfg->axc].config2;
    cfg->filter3 = CSL_FEXT(data, DFE_BB_RXNOTCH_CONFIG2_REG_FILTER3);
    cfg->filter4 = CSL_FEXT(data, DFE_BB_RXNOTCH_CONFIG2_REG_FILTER4);
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigBeagcGlobal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         cfg    [add content]
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
 *       DFE_BB_CFG47_REG_BEAGC_TDD_CONFIG
 *       DFE_BB_CFG36_REG_BEAGC_LOOP_CONFIG_SAT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigBeagcGlobal(DfeFl_BbHandle hDfeBb, DfeFl_BbBeagcGlobalConfig *cfg)
{
    CSL_FINS(hDfeBb->regs->cfg36, DFE_BB_CFG36_REG_BEAGC_LOOP_CONFIG_SAT, cfg->loop_config_sat);
    CSL_FINS(hDfeBb->regs->cfg47, DFE_BB_CFG47_REG_BEAGC_TDD_CONFIG, cfg->tdd_config);
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryBeagcGlobalConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         cfg    [add content]
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
 *       DFE_BB_CFG47_REG_BEAGC_TDD_CONFIG
 *       DFE_BB_CFG36_REG_BEAGC_LOOP_CONFIG_SAT
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
dfeFl_BbQueryBeagcGlobalConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbBeagcGlobalConfig *cfg)
{
    cfg->loop_config_sat = CSL_FEXT(hDfeBb->regs->cfg36, DFE_BB_CFG36_REG_BEAGC_LOOP_CONFIG_SAT);
    cfg->tdd_config = CSL_FEXT(hDfeBb->regs->cfg47, DFE_BB_CFG47_REG_BEAGC_TDD_CONFIG);
}

/** ============================================================================
 *   @n@b dfeFl_BbConfigBeagc
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         cfg    [add content]
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
 *       DFE_BB_CFG47_REG_BEAGC_TDD_EN
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_BB_BEAGC_CFG2_REG_DZRI
 *       DFE_BB_BEAGC_CFG1_REG_ZERO_THRES
 *       DFE_BB_BEAGC_CFG4_REG_AMAX_23_16
 *       DFE_BB_BEAGC_CFG2_REG_DABV
 *       DFE_BB_CFG39_REG_BEAGC_T3INTERVAL0_15_0
 *       DFE_BB_BEAGC_CFG2_REG_DBLW
 *       DFE_BB_BEAGC_CFG0_REG_THRES
 *       DFE_BB_BEAGC_CFG1_REG_ZERO_MASK
 *       DFE_BB_BEAGC_CFG2_REG_DSAT
 *       DFE_BB_CFG47_REG_BEAGC_TDD_EN
 *       DFE_BB_BEAGC_CFG4_REG_AMIN_23_16
 *       DFE_BB_BEAGC_CFG1_REG_SAT_THRES
 *       DFE_BB_BEAGC_CFG3_REG_AMAX_15_0
 *       DFE_BB_BEAGC_CFG5_REG_AMIN_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbConfigBeagc(DfeFl_BbHandle hDfeBb, DfeFl_BbBeagcConfig *cfg)
{
    uint32_t data;
    uint32_t r;
    uint32_t b;
    volatile uint32_t *regs_intervalSource = &hDfeBb->regs->cfg37;
    volatile uint32_t *regs_t3IntervalLo = &hDfeBb->regs->cfg39;
    volatile uint32_t *regs_t3IntervalHi = &hDfeBb->regs->cfg124;
    volatile CSL_DFE_BB_BEAGC_REGS *regs_cfg = &hDfeBb->regs->beagc[cfg->beagc];
    
    // Select which buffer sync is the source of the interval counter for configuration 0 in closed loop mode.  
    r = cfg->beagc / 4;
    b = (cfg->beagc % 4) * 4;
    CSL_FINSR(regs_intervalSource[r], b+3, b, cfg->intervalSource);
    // master t3 interval
    r = cfg->beagc / 2;
    b = (cfg->beagc % 2) * 8;
    regs_t3IntervalLo[cfg->beagc] = CSL_FMK(DFE_BB_CFG39_REG_BEAGC_T3INTERVAL0_15_0, cfg->t3Interval);
    CSL_FINSR(regs_t3IntervalHi[r], b+7, b, cfg->t3Interval >> 16);
    
    // enable/disbale tdd mode
    data = CSL_FEXT(hDfeBb->regs->cfg47, DFE_BB_CFG47_REG_BEAGC_TDD_EN);
    CSL_FINSR(data, cfg->beagc, cfg->beagc, cfg->tdd_enable);
    CSL_FINS(hDfeBb->regs->cfg47, DFE_BB_CFG47_REG_BEAGC_TDD_EN, data);
    
    // beagc control loop configuration threshold value of AGC unsigned
    CSL_FINS(regs_cfg->cfg0, DFE_BB_BEAGC_CFG0_REG_THRES, cfg->thresh);
    
    // beagc control loop configuration zero_mask masks lower 4 bits for zero count, a 0 will mask off zero calculation
    // beagc control loop configuration  zero count threshold
    // beagc control loop configuration saturation count threshold; 
    regs_cfg->cfg1 \
        = CSL_FMK(DFE_BB_BEAGC_CFG1_REG_ZERO_MASK, cfg->zeroMask)
        | CSL_FMK(DFE_BB_BEAGC_CFG1_REG_ZERO_THRES, cfg->zeroCountThresh)
        | CSL_FMK(DFE_BB_BEAGC_CFG1_REG_SAT_THRES, cfg->satCountThresh);
    // beagc control loop configuration shift value for below threshold.  0=shift of 2 ... 15=shift of 17
    regs_cfg->cfg2 \
        = CSL_FMK(DFE_BB_BEAGC_CFG2_REG_DBLW, cfg->dBelow)
        | CSL_FMK(DFE_BB_BEAGC_CFG2_REG_DABV, cfg->dAbove)
        | CSL_FMK(DFE_BB_BEAGC_CFG2_REG_DSAT, cfg->dSat)
        | CSL_FMK(DFE_BB_BEAGC_CFG2_REG_DZRI, cfg->dZero);
    // beagc control loop configuration maximum allowed gain adjustment value.  Adjustment stops at g(k)=G + amax Amax format is signed (1,16,7)
    regs_cfg->cfg3 = CSL_FMK(DFE_BB_BEAGC_CFG3_REG_AMAX_15_0, cfg->amax);
    
    regs_cfg->cfg4 \
        = CSL_FMK(DFE_BB_BEAGC_CFG4_REG_AMAX_23_16, cfg->amax >> 16)
        | CSL_FMK(DFE_BB_BEAGC_CFG4_REG_AMIN_23_16, cfg->amin >> 16);
    // beagc control loop configuration minimum allowed gain adjustment value.   Adjustment stops at g(k)=G +  amin. Amin format is signed (1,16,7)
    regs_cfg->cfg5 = CSL_FMK(DFE_BB_BEAGC_CFG5_REG_AMIN_15_0, cfg->amin);
    
}

/** ============================================================================
 *   @n@b dfeFl_BbQueryBeagcConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         cfg    [add content]
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
 *       DFE_BB_BEAGC_CFG2_REG_DBLW
 *       DFE_BB_BEAGC_CFG2_REG_DZRI
 *       DFE_BB_CFG47_REG_BEAGC_TDD_EN
 *       DFE_BB_BEAGC_CFG4_REG_AMAX_23_16
 *       DFE_BB_BEAGC_CFG2_REG_DABV
 *       DFE_BB_CFG39_REG_BEAGC_T3INTERVAL0_15_0
 *       DFE_BB_BEAGC_CFG1_REG_ZERO_THRES
 *       DFE_BB_BEAGC_CFG0_REG_THRES
 *       DFE_BB_BEAGC_CFG1_REG_ZERO_MASK
 *       DFE_BB_BEAGC_CFG2_REG_DSAT
 *       DFE_BB_BEAGC_CFG4_REG_AMIN_23_16
 *       DFE_BB_BEAGC_CFG1_REG_SAT_THRES
 *       DFE_BB_BEAGC_CFG3_REG_AMAX_15_0
 *       DFE_BB_BEAGC_CFG5_REG_AMIN_15_0
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
dfeFl_BbQueryBeagcConfig(DfeFl_BbHandle hDfeBb, DfeFl_BbBeagcConfig *cfg)
{
    uint32_t data;
    uint32_t r;
    uint32_t b;
    volatile uint32_t *regs_intervalSource = &hDfeBb->regs->cfg37;
    volatile uint32_t *regs_t3IntervalLo = &hDfeBb->regs->cfg39;
    volatile uint32_t *regs_t3IntervalHi = &hDfeBb->regs->cfg124;
    volatile CSL_DFE_BB_BEAGC_REGS *regs_cfg = &hDfeBb->regs->beagc[cfg->beagc];
    
    // Select which buffer sync is the source of the interval counter for configuration 0 in closed loop mode.  
    r = cfg->beagc / 4;
    b = (cfg->beagc % 4) * 4;
    cfg->intervalSource = CSL_FEXTR(regs_intervalSource[r], b+3, b);
    // master t3 interval
    r = cfg->beagc / 2;
    b = (cfg->beagc % 2) * 8;
    
    cfg->t3Interval \
        = CSL_FEXT(regs_t3IntervalLo[cfg->beagc] , DFE_BB_CFG39_REG_BEAGC_T3INTERVAL0_15_0)
        |(CSL_FEXTR(regs_t3IntervalHi[r], b+7, b) << 16);
    
    // enable/disbale tdd mode
    data = CSL_FEXT(hDfeBb->regs->cfg47, DFE_BB_CFG47_REG_BEAGC_TDD_EN);
    cfg->tdd_enable = CSL_FEXTR(data, cfg->beagc, cfg->beagc);
    
    // beagc control loop configuration threshold value of AGC unsigned
    cfg->thresh = CSL_FEXT(regs_cfg->cfg0, DFE_BB_BEAGC_CFG0_REG_THRES);
    
    // beagc control loop configuration zero_mask masks lower 4 bits for zero count, a 0 will mask off zero calculation
    // beagc control loop configuration  zero count threshold
    // beagc control loop configuration saturation count threshold; 
    data = regs_cfg->cfg1;
    cfg->zeroMask = CSL_FEXT(data, DFE_BB_BEAGC_CFG1_REG_ZERO_MASK);
    cfg->zeroCountThresh = CSL_FEXT(data, DFE_BB_BEAGC_CFG1_REG_ZERO_THRES);
    cfg->satCountThresh = CSL_FEXT(data, DFE_BB_BEAGC_CFG1_REG_SAT_THRES);
    // beagc control loop configuration shift value for below threshold.  0=shift of 2 ... 15=shift of 17
    data = regs_cfg->cfg2;
    cfg->dBelow = CSL_FEXT(data, DFE_BB_BEAGC_CFG2_REG_DBLW);
    cfg->dAbove = CSL_FEXT(data, DFE_BB_BEAGC_CFG2_REG_DABV);
    cfg->dSat = CSL_FEXT(data, DFE_BB_BEAGC_CFG2_REG_DSAT);
    cfg->dZero = CSL_FEXT(data, DFE_BB_BEAGC_CFG2_REG_DZRI);
    
    data = regs_cfg->cfg4;
    cfg->amax = CSL_FEXT(regs_cfg->cfg3, DFE_BB_BEAGC_CFG3_REG_AMAX_15_0);    
    cfg->amax |= CSL_FEXT(data, DFE_BB_BEAGC_CFG4_REG_AMAX_23_16);

    cfg->amin  = CSL_FEXT(data, DFE_BB_BEAGC_CFG4_REG_AMIN_23_16) << 16;
    cfg->amin |= CSL_FEXT(regs_cfg->cfg5, DFE_BB_BEAGC_CFG5_REG_AMIN_15_0);    
}

/** ============================================================================
 *   @n@b dfeFl_BbSetBeagcSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         beagc    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetBeagcSsel(DfeFl_BbHandle hDfeBb, uint32_t beagc, uint32_t ssel)
{
    uint32_t r = beagc / 4;
    uint32_t b = (beagc % 4) * 4;
    volatile uint32_t *regs_ssel = &hDfeBb->regs->cfg82;
    
    CSL_FINSR(regs_ssel[r], b+3, b, ssel);
}

/** ============================================================================
 *   @n@b dfeFl_BbGetBeagcSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         beagc    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbGetBeagcSsel(DfeFl_BbHandle hDfeBb, uint32_t beagc, uint32_t *ssel)
{
    uint32_t r = beagc / 4;
    uint32_t b = (beagc % 4) * 4;
    volatile uint32_t *regs_ssel = &hDfeBb->regs->cfg82;
    
    *ssel = CSL_FEXTR(regs_ssel[r], b+3, b);
}

/** ============================================================================
 *   @n@b dfeFl_GetCarrierTypeInterval
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         carrierType    [add content]
         ctInterval    [add content]
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
dfeFl_GetCarrierTypeInterval(DfeFl_BbHandle hDfeBb, uint32_t carrierType, uint32_t *ctInterval)
{
    volatile uint32_t *regs = &hDfeBb->regs->cfg3;
    
    uint32_t r = carrierType / 2;
    uint32_t b = carrierType % 2;
    
    *ctInterval = CSL_FEXTR(regs[r], b+7, b);
}

/** ============================================================================
 *   @n@b dfeFl_BbGetAidDlXlate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         stream    [add content]
         xlate    [add content]
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
dfeFl_BbGetAidDlXlate(DfeFl_BbHandle hDfeBb, uint32_t stream, DfeFl_BbAidDlXlate *xlate)
{
    uint32_t *data = (uint32_t *)xlate;
    *data = hDfeBb->regs->aid_dl_xlate[stream];
}

/** ============================================================================
 *   @n@b dfeFl_BbGetAidUlXlate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         stream    [add content]
         xlate    [add content]
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
dfeFl_BbGetAidUlXlate(DfeFl_BbHandle hDfeBb, uint32_t stream, DfeFl_BbAidUlXlate *xlate)
{
    uint32_t *data = (uint32_t *)xlate;
    *data = hDfeBb->regs->aid_ul_xlate[stream];
}

/** ============================================================================
 *   @n@b dfeFl_BbGetTxifSlot
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         slot    [add content]
         slotEntry    [add content]
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
dfeFl_BbGetTxifSlot(DfeFl_BbHandle hDfeBb, uint32_t slot, DfeFl_BbTxifSlot *slotEntry)
{
    uint32_t *data = (uint32_t *)slotEntry;
    
    *data = hDfeBb->regs->bbtxif_slot[slot];
}

/** ============================================================================
 *   @n@b dfeFl_BbGetRxifSlot
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         slot    [add content]
         slotEntry    [add content]
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
dfeFl_BbGetRxifSlot(DfeFl_BbHandle hDfeBb, uint32_t slot, DfeFl_BbRxifSlot *slotEntry)
{
    uint32_t *data = (uint32_t *)slotEntry;
    
    *data = hDfeBb->regs->bbrxif_slot[slot];
}

/** ============================================================================
 *   @n@b dfeFl_BbSetCarrierTypeUlSyncStrobe
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         carrierType    [add content]
         syncStrobe    [add content]
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
 *       DFE_BB_CFG92_REG_CT2_UL_SYNC_STROBE
 *       DFE_BB_CFG92_REG_CT1_UL_SYNC_STROBE
 *       DFE_BB_CFG92_REG_CT3_UL_SYNC_STROBE
 *       DFE_BB_CFG92_REG_CT0_UL_SYNC_STROBE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetCarrierTypeUlSyncStrobe(DfeFl_BbHandle hDfeBb, uint32_t carrierType, uint32_t syncStrobe)
{    
    uint32_t r, b;
    volatile uint32_t *regs = &hDfeBb->regs->cfg92;
    
    if(carrierType == DFE_FL_BB_CARRIER_TYPE_ALL)
    {
        r = CSL_FMK(DFE_BB_CFG92_REG_CT0_UL_SYNC_STROBE, syncStrobe) \
          | CSL_FMK(DFE_BB_CFG92_REG_CT1_UL_SYNC_STROBE, syncStrobe) \
          | CSL_FMK(DFE_BB_CFG92_REG_CT2_UL_SYNC_STROBE, syncStrobe) \
          | CSL_FMK(DFE_BB_CFG92_REG_CT3_UL_SYNC_STROBE, syncStrobe);
        
        regs[0] = regs[1] = regs[2] = regs[3] = r;          
    }
    else if(carrierType <= DFE_FL_BB_CARRIER_TYPE_15)
    {
        r = carrierType / 4;
        b = (carrierType % 4) * 4;
        
        CSL_FINSR(regs[r], b+3, b, syncStrobe);
    }                    
}

/** ============================================================================
 *   @n@b dfeFl_BbGetCarrierTypeUlSyncStrobe
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         carrierType    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  [add content]
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
CSL_IDEF_INLINE uint32_t
dfeFl_BbGetCarrierTypeUlSyncStrobe(DfeFl_BbHandle hDfeBb, uint32_t carrierType)
{    
    uint32_t r, b, syncStrobe;
    volatile uint32_t *regs = &hDfeBb->regs->cfg92;
    
    if(carrierType <= DFE_FL_BB_CARRIER_TYPE_15)
    {
        r = carrierType / 4;
        b = (carrierType % 4) * 4;
        
        syncStrobe = CSL_FEXTR(regs[r], b+3, b);
    }
    else
        syncStrobe = 0xdeadbeefu;
    
    return syncStrobe;                    
}

/** ============================================================================
 *   @n@b dfeFl_BbSetAidUlStrobeDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         carrierType    [add content]
         delay    [add content]
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
 *       DFE_BB_CFG96_REG_AID_ULSTROBE_DLY_CT0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_BbSetAidUlStrobeDelay(DfeFl_BbHandle hDfeBb, uint32_t carrierType, uint32_t delay)
{    
    uint32_t r;
    volatile uint32_t *regs = &hDfeBb->regs->cfg96;
    
    if(carrierType == DFE_FL_BB_CARRIER_TYPE_ALL)
    {
        for(r = 0; r < 16; r++)
        {
            regs[r] = CSL_FMK(DFE_BB_CFG96_REG_AID_ULSTROBE_DLY_CT0, delay);
        }
    }
    else if(carrierType <= DFE_FL_BB_CARRIER_TYPE_15)
    {
        r = carrierType;
        
        regs[r] = CSL_FMK(DFE_BB_CFG96_REG_AID_ULSTROBE_DLY_CT0, delay);
    }                    
}

/** ============================================================================
 *   @n@b dfeFl_BbGetAidUlStrobeDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb    [add content]
         carrierType    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  [add content]
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_BB_CFG96_REG_AID_ULSTROBE_DLY_CT0
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
CSL_IDEF_INLINE uint32_t
dfeFl_BbGetAidUlStrobeDelay(DfeFl_BbHandle hDfeBb, uint32_t carrierType)
{    
    uint32_t r, delay;
    volatile uint32_t *regs = &hDfeBb->regs->cfg96;
    
    if(carrierType <= DFE_FL_BB_CARRIER_TYPE_15)
    {
        r = carrierType;
        
        delay = CSL_FEXT(regs[r], DFE_BB_CFG96_REG_AID_ULSTROBE_DLY_CT0);
    }
    else
        delay = 0xdeadbeefu;
    
    return delay;                    
}

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_BBAUX_H_ */
