/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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

/** ============================================================================
 *   @file  iqn2fl_getHwStatusAux.h
 *
 *   @brief  API Auxilary header file for IQN2 get HW status
 *
 */

#ifndef _IQN2FLGETHWSTATUSAUX_H_
#define _IQN2FLGETHWSTATUSAUX_H_
 
#include <ti/drv/iqn2/iqn2fl.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 *  Get Hardware Status Functions of IQN2
 */

/** ============================================================================
 *   @n@b Iqn2Fl_getAt2RadtStatus
 *
 *   @b Description
 *   @n This function returns AT2 RADT status 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2            Handle to the aif2 instance
            pAt2RadtStatus   Pointer to get AT2 RADT status.

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AT2_RADT_0_STS, AT2_RADT_1_STS, AT2_RADT_2_STS, AT2_RADT_3_STS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_At2RadtStatus   at2RadtStatus;
        Iqn2Fl_getAt2RadtStatus (hIqn2,  &at2RadtStatus);
     @endverbatim
 * ===========================================================================
 */
static inline 
void Iqn2Fl_getAt2RadtStatus(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_At2RadtStatus *pAt2RadtStatus
)
{
    uint32_t tmpReg;

    tmpReg = hIqn2->regs->At2.AT2_RADT[pAt2RadtStatus->radt_num].AT2_RADT_0_STS;
    pAt2RadtStatus->radt_sampcnt_val = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_RADT_0_STS_RADT_SAMPCNT_VAL);
    pAt2RadtStatus->radt_symcnt_val = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_RADT_0_STS_RADT_SYMCNT_VAL);

    tmpReg = hIqn2->regs->At2.AT2_RADT[pAt2RadtStatus->radt_num].AT2_RADT_1_STS;
    pAt2RadtStatus->radt_frm_val_lsb = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_RADT_1_STS_RADT_FRM_VAL_LSB);

    tmpReg = hIqn2->regs->At2.AT2_RADT[pAt2RadtStatus->radt_num].AT2_RADT_2_STS;
    pAt2RadtStatus->radt_frm_val_msb = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_RADT_2_STS_RADT_FRM_VAL_MSB);

    tmpReg = hIqn2->regs->At2.AT2_RADT[pAt2RadtStatus->radt_num].AT2_RADT_3_STS;
    pAt2RadtStatus->radt_chip = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_RADT_3_STS_RADT_CHIP);
    pAt2RadtStatus->radt_slot = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_RADT_3_STS_RADT_SLOT);
    pAt2RadtStatus->radt_frm = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_RADT_3_STS_RADT_FRME);
    
    return;
}


#ifdef __cplusplus
}
#endif
#endif /* _IQN2FLGETHWSTATUSAUX_H_ */
