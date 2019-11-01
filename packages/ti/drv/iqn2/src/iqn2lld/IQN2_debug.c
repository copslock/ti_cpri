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

#include <string.h>
#include <stdio.h>

#include <ti/csl/csl.h>
#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>
#include <ti/drv/iqn2/iqn2fl_getHwStatusAux.h>
#include <ti/drv/iqn2/iqn2_osal.h>

#include <ti/drv/iqn2/include/IQN2_defs.h>
#include <ti/drv/iqn2/include/IQN2_runtime.h>

#define __IQN2_DEBUG_C
#include <ti/drv/iqn2/include/IQN2_debug.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(IQN2_enableTopException, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_disableTopException, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_enablePktdmaException, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_disablePktdmaException, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_enableAt2Exception, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_disableAt2Exception, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_enableIqs2Exception, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_disableIqs2Exception, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_enableAid2Exception, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_disableAid2Exception, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_enableDio2Exception, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_disableDio2Exception, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_enableAilException, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_disableAilException, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_enableException, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_disableException, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_getPsrException, ".text:iqn2");
#pragma CODE_SECTION(IQN2_getPktDMAException, ".text:iqn2");
#pragma CODE_SECTION(IQN2_getAt2Exception, ".text:iqn2");
#pragma CODE_SECTION(IQN2_getIqs2Exception, ".text:iqn2");
#pragma CODE_SECTION(IQN2_getAid2Exception, ".text:iqn2");
#pragma CODE_SECTION(IQN2_getDfeException, ".text:iqn2");
#pragma CODE_SECTION(IQN2_getDio2Exception, ".text:iqn2");
#pragma CODE_SECTION(IQN2_getAilException, ".text:iqn2");
#pragma CODE_SECTION(IQN2_getException, ".text:iqn2");
#pragma CODE_SECTION(IQN2_printException, ".text:iqn2");
#pragma CODE_SECTION(IQN2_printStatus, ".text:iqn2");
#pragma CODE_SECTION(IQN2_resetException, ".text:iqn2");
#pragma CODE_SECTION(IQN2_captureException, ".text:iqn2");
#pragma CODE_SECTION(IQN2_enableAilDataTrace, ".text:iqn2");
#pragma CODE_SECTION(IQN2_disableAilDataTrace, ".text:iqn2");
#pragma CODE_SECTION(reset_ExceptionStats, ".text:iqn2");
#endif

#ifdef _TMS320C6X
extern uint32_t _disable_interrupts(void);
extern void _restore_interrupts(uint32_t key);
#endif

/* Reset exception stats */
static void reset_ExceptionStats(
        IQN2_ConfigHandle  hIqn2
)
{
    memset(&hIqn2->iqn2EeCount, (Uint8)0x00, sizeof(IQN2_EeCountObj));
}

void IQN2_resetException(
        IQN2_ConfigHandle  hIqn2
)
{
    reset_ExceptionStats(hIqn2);
    IQN2_resyncProcedureReset();
}

void IQN2_captureException (
        IQN2_ConfigHandle  hIqn2,
        IQN2_EeCountObj *capturePtr
)
{
    memcpy(capturePtr, &hIqn2->iqn2EeCount, sizeof(IQN2_EeCountObj));
}

/* Enable  IQN2 TOP Errors and Alarms */
void IQN2_enableTopException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_PsrEeIngFlushA           psrEeIngFlushA;
Iqn2Fl_PsrEeIngFlushB           psrEeIngFlushB;
Iqn2Fl_PsrEeEgressProtocolErrA  psrEeEgressProtocolErrA;
Iqn2Fl_PsrEeEgressProtocolErrB  psrEeEgressProtocolErrB;
Iqn2Fl_PktdmaEeDescStarve       pktdmaEeDescStarve;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_SET_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_SET_EV0;
   }
   /* TOP errors */
   psrEeIngFlushA.err = 0xFFFFFFFF; // enable errors for all channels
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_A_REGSET, (void *)&psrEeIngFlushA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_A_REGSET, (void *)&psrEeIngFlushA);

   psrEeIngFlushB.err = 0x0000FFFF; // enable errors for all channels
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_B_REGSET, (void *)&psrEeIngFlushB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_B_REGSET, (void *)&psrEeIngFlushB);

   psrEeEgressProtocolErrA.err = 0xFFFFFFFF; // enable errors for all channels
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_A_REGSET, (void *)&psrEeEgressProtocolErrA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_A_REGSET, (void *)&psrEeEgressProtocolErrA);

   psrEeEgressProtocolErrB.err = 0x0000FFFF; // enable errors for all channels
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_B_REGSET, (void *)&psrEeEgressProtocolErrB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_B_REGSET, (void *)&psrEeEgressProtocolErrB);

   // only PKTDMA_STARVE interrupt available for PktDMA exceptions
   pktdmaEeDescStarve.mop_err = TRUE;
   pktdmaEeDescStarve.sop_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET, (void *)&pktdmaEeDescStarve);
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_EN_SET_EV0;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET, (void *)&pktdmaEeDescStarve);
}

/* Disable  IQN2 TOP Errors and Alarms */
void IQN2_disableTopException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_PsrEeIngFlushA           psrEeIngFlushA;
Iqn2Fl_PsrEeIngFlushB           psrEeIngFlushB;
Iqn2Fl_PsrEeEgressProtocolErrA  psrEeEgressProtocolErrA;
Iqn2Fl_PsrEeEgressProtocolErrB  psrEeEgressProtocolErrB;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV0;
   }
   /* TOP errors */
   psrEeIngFlushA.err = 0xFFFFFFFF; // enable errors for all channels
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_A_REGSET, (void *)&psrEeIngFlushA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_A_REGSET, (void *)&psrEeIngFlushA);

   psrEeIngFlushB.err = 0x0000FFFF; // enable errors for all channels
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_B_REGSET, (void *)&psrEeIngFlushB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_B_REGSET, (void *)&psrEeIngFlushB);

   psrEeEgressProtocolErrA.err = 0xFFFFFFFF; // enable errors for all channels
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_A_REGSET, (void *)&psrEeEgressProtocolErrA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_A_REGSET, (void *)&psrEeEgressProtocolErrA);

   psrEeEgressProtocolErrB.err = 0x0000FFFF; // enable errors for all channels
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_B_REGSET, (void *)&psrEeEgressProtocolErrB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_B_REGSET, (void *)&psrEeEgressProtocolErrB);

}

/* Enable  IQN2 PKTDMA Errors and Alarms */
void IQN2_enablePktdmaException(
        IQN2_ConfigHandle  hIqn2
)
{
Iqn2Fl_PktdmaEeDescStarve       pktdmaEeDescStarve;

   // only PKTDMA_STARVE interrupt available for PktDMA exceptions
   pktdmaEeDescStarve.mop_err = TRUE;
   pktdmaEeDescStarve.sop_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET, (void *)&pktdmaEeDescStarve);
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_EN_SET_EV0;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET, (void *)&pktdmaEeDescStarve);
}

/* Disable  IQN2 PKTDMA Errors and Alarms */
void IQN2_disablePktdmaException(
        IQN2_ConfigHandle  hIqn2
)
{
Iqn2Fl_PktdmaEeDescStarve       pktdmaEeDescStarve;

   // only PKTDMA_STARVE interrupt available for PktDMA exceptions
   pktdmaEeDescStarve.mop_err = TRUE;
   pktdmaEeDescStarve.sop_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET, (void *)&pktdmaEeDescStarve);
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_EN_CLR_EV0;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET, (void *)&pktdmaEeDescStarve);
}

/* Enable  IQN2 Errors and Alarms */
void IQN2_enableAt2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_At2EeInfoErr             at2EeInfoErr;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_SET_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_SET_EV0;
   }
   if (hIqn2->timerSyncSource == IQN2_PA_TSCOMP_SYNC) at2EeInfoErr.pa_tscomp_info = TRUE;
   else                                               at2EeInfoErr.pa_tscomp_info = FALSE;
   if (hIqn2->timerSyncSource == IQN2_PHY_SYNC)       at2EeInfoErr.physync_info   = TRUE;
   else                                               at2EeInfoErr.physync_info   = FALSE;
   if (hIqn2->timerSyncSource == IQN2_RAD_SYNC)       at2EeInfoErr.radsync_info   = TRUE;
   else                                               at2EeInfoErr.radsync_info   = FALSE;
   if (hIqn2->timerSyncSource == IQN2_RP1_SYNC)       at2EeInfoErr.rp1_sync_info  = TRUE;
   else                                               at2EeInfoErr.rp1_sync_info  = FALSE;
   at2EeInfoErr.rp1_bit_err    = FALSE;
   at2EeInfoErr.rp1_crc_err    = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_AT_EE_0_REGSET, (void *)&at2EeInfoErr);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_AT_EE_0_REGSET, (void *)&at2EeInfoErr);
}

/* Disable  IQN2 Errors and Alarms */
void IQN2_disableAt2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_At2EeInfoErr             at2EeInfoErr;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV0;
   }
   if (hIqn2->timerSyncSource == IQN2_PA_TSCOMP_SYNC) at2EeInfoErr.pa_tscomp_info = TRUE;
   else                                               at2EeInfoErr.pa_tscomp_info = FALSE;
   if (hIqn2->timerSyncSource == IQN2_PHY_SYNC)       at2EeInfoErr.physync_info   = TRUE;
   else                                               at2EeInfoErr.physync_info   = FALSE;
   if (hIqn2->timerSyncSource == IQN2_RAD_SYNC)       at2EeInfoErr.radsync_info   = TRUE;
   else                                               at2EeInfoErr.radsync_info   = FALSE;
   if (hIqn2->timerSyncSource == IQN2_RP1_SYNC)       at2EeInfoErr.rp1_sync_info  = TRUE;
   else                                               at2EeInfoErr.rp1_sync_info  = FALSE;
   at2EeInfoErr.rp1_bit_err    = FALSE;
   at2EeInfoErr.rp1_crc_err    = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_AT_EE_0_REGSET, (void *)&at2EeInfoErr);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_AT_EE_0_REGSET, (void *)&at2EeInfoErr);
}

/* Enable  IQN2 IQS2 Errors and Alarms */
void IQN2_enableIqs2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_Iqs2EeChanErr        iqs2EeChanErr;
Iqn2Fl_Iqs2EeIngFlush       iqs2EeIngFlush;
Iqn2Fl_Iqs2EeEgrFlush       iqs2EeEgrFlush;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_SET_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_SET_EV0;
   }
   iqs2EeChanErr.egr_mux_err = TRUE;
   iqs2EeChanErr.ing_dio2_err = TRUE;
   iqs2EeChanErr.ing_pktdma_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_CHAN_ERR_REGSET, (void *)&iqs2EeChanErr);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_CHAN_ERR_REGSET, (void *)&iqs2EeChanErr);

   iqs2EeIngFlush.flush = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_ING_FLUSH_ERR_REGSET, (void *)&iqs2EeIngFlush);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_ING_FLUSH_ERR_REGSET, (void *)&iqs2EeIngFlush);

   iqs2EeEgrFlush.flush = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_EGR_FLUSH_ERR_REGSET, (void *)&iqs2EeEgrFlush);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_EGR_FLUSH_ERR_REGSET, (void *)&iqs2EeEgrFlush);
}

/* Disable  IQN2 IQS2 Errors and Alarms */
void IQN2_disableIqs2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_Iqs2EeChanErr        iqs2EeChanErr;
Iqn2Fl_Iqs2EeIngFlush       iqs2EeIngFlush;
Iqn2Fl_Iqs2EeEgrFlush       iqs2EeEgrFlush;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV0;
   }
   iqs2EeChanErr.egr_mux_err = TRUE;
   iqs2EeChanErr.ing_dio2_err = TRUE;
   iqs2EeChanErr.ing_pktdma_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_CHAN_ERR_REGSET, (void *)&iqs2EeChanErr);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_CHAN_ERR_REGSET, (void *)&iqs2EeChanErr);

   iqs2EeIngFlush.flush = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_ING_FLUSH_ERR_REGSET, (void *)&iqs2EeIngFlush);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_ING_FLUSH_ERR_REGSET, (void *)&iqs2EeIngFlush);

   iqs2EeEgrFlush.flush = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_EGR_FLUSH_ERR_REGSET, (void *)&iqs2EeEgrFlush);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_EGR_FLUSH_ERR_REGSET, (void *)&iqs2EeEgrFlush);
}

/* Enable  IQN2 AID2 Errors and Alarms */
void IQN2_enableAid2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_Aid2EeSiiA   aid2EeSiiA;
Iqn2Fl_Aid2EeSiiB   aid2EeSiiB;
Iqn2Fl_Aid2EeSiiC   aid2EeSiiC;
Iqn2Fl_Aid2EeSiiD   aid2EeSiiD;
Iqn2Fl_Aid2EeSiiE   aid2EeSiiE;
Iqn2Fl_Aid2EeSiiF   aid2EeSiiF;
Iqn2Fl_Aid2EeSiiG   aid2EeSiiG;
Iqn2Fl_Aid2EeSiiH   aid2EeSiiH;
Iqn2Fl_Aid2EeSieA   aid2EeSieA;
Iqn2Fl_Aid2EeSieB   aid2EeSieB;
Iqn2Fl_Aid2EeSieC   aid2EeSieC;
Iqn2Fl_Aid2EeSieD   aid2EeSieD;
Iqn2Fl_Aid2EeSieE   aid2EeSieE;
Iqn2Fl_Aid2EeSieF   aid2EeSieF;
Iqn2Fl_Aid2EeSieG   aid2EeSieG;
Iqn2Fl_Aid2EeDfe    aid2EeDfe;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_SET_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_SET_EV0;
   }
   aid2EeSiiA.si_ing_iq_fifo_ovfl_err = TRUE;
   aid2EeSiiA.si_ing_iq_icc_dat_info = FALSE;
   aid2EeSiiA.si_ing_iq_icc_sof_info = FALSE;
   aid2EeSiiA.si_ing_iq_ife_dat_info = FALSE;
   aid2EeSiiA.si_ing_iq_ife_eop_info = FALSE;
   aid2EeSiiA.si_ing_iq_ife_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_A_REGSET, (void *)&aid2EeSiiA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_A_REGSET, (void *)&aid2EeSiiA);

   aid2EeSiiB.si_ing_ctl_fifo_ovfl_err = TRUE;
   aid2EeSiiB.si_ing_ctl_icc_dat_info = FALSE;
   aid2EeSiiB.si_ing_ctl_icc_eop_info = FALSE;
   aid2EeSiiB.si_ing_ctl_pkt_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_B_REGSET, (void *)&aid2EeSiiB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_B_REGSET, (void *)&aid2EeSiiB);

   aid2EeSiiC.si_ing_iq_sof_err = 0xFFFFFFFF; // enable all IQ channels [31:0]
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_C_REGSET, (void *)&aid2EeSiiC);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_C_REGSET, (void *)&aid2EeSiiC);

   aid2EeSiiD.si_ing_ctl_icc_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_D_REGSET, (void *)&aid2EeSiiD);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_D_REGSET, (void *)&aid2EeSiiD);

   aid2EeSiiE.si_ing_iq_psi_dat_info = FALSE;
   aid2EeSiiE.si_ing_iq_psi_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_E_REGSET, (void *)&aid2EeSiiE);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_E_REGSET, (void *)&aid2EeSiiE);

   aid2EeSiiF.si_ing_ctl_psi_dat_info = FALSE;
   aid2EeSiiF.si_ing_ctl_psi_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_F_REGSET, (void *)&aid2EeSiiF);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_F_REGSET, (void *)&aid2EeSiiF);

   aid2EeSiiG.si_ing_iq_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_G_REGSET, (void *)&aid2EeSiiG);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_G_REGSET, (void *)&aid2EeSiiG);

   aid2EeSiiH.si_ing_ctl_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_H_REGSET, (void *)&aid2EeSiiH);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_H_REGSET, (void *)&aid2EeSiiH);

   aid2EeSieA.si_egr_iq_efe_pkt_err = TRUE;
   aid2EeSieA.si_egr_iq_efe_starve_err = TRUE;
   aid2EeSieA.si_egr_iq_efe_sym_err = TRUE;
   aid2EeSieA.si_egr_iq_icc_dat_info = FALSE;
   aid2EeSieA.si_egr_iq_icc_sof_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_A_REGSET, (void *)&aid2EeSieA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_A_REGSET, (void *)&aid2EeSieA);

   aid2EeSieB.si_egr_ctl_icc_dat_info = FALSE;
   aid2EeSieB.si_egr_ctl_icc_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_B_REGSET, (void *)&aid2EeSieB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_B_REGSET, (void *)&aid2EeSieB);

   aid2EeSieC.si_egr_ctl_icc_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_C_REGSET, (void *)&aid2EeSieC);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_C_REGSET, (void *)&aid2EeSieC);

   aid2EeSieD.si_egr_iq_psi_dat_info = FALSE;
   aid2EeSieD.si_egr_iq_psi_data_type_err = TRUE;
   aid2EeSieD.si_egr_iq_psi_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_D_REGSET, (void *)&aid2EeSieD);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_D_REGSET, (void *)&aid2EeSieD);

   aid2EeSieE.si_egr_ctl_psi_dat_info = FALSE;
   aid2EeSieE.si_egr_ctl_psi_data_type_err = TRUE;
   aid2EeSieE.si_egr_ctl_psi_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_E_REGSET, (void *)&aid2EeSieE);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_E_REGSET, (void *)&aid2EeSieE);

   aid2EeSieF.si_egr_iq_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_F_REGSET, (void *)&aid2EeSieF);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_F_REGSET, (void *)&aid2EeSieF);

   aid2EeSieG.si_egr_ctl_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_G_REGSET, (void *)&aid2EeSieG);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_G_REGSET, (void *)&aid2EeSieG);

   aid2EeDfe.dfe_iqn_aid2_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_DFE_REGSET, (void *)&aid2EeDfe);
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_EN_SET_EV1; // force to EV1 for DFE exceptions
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_DFE_REGSET, (void *)&aid2EeDfe);
}

/* Disable  IQN2 AID2 Errors and Alarms */
void IQN2_disableAid2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_Aid2EeSiiA   aid2EeSiiA;
Iqn2Fl_Aid2EeSiiB   aid2EeSiiB;
Iqn2Fl_Aid2EeSiiC   aid2EeSiiC;
Iqn2Fl_Aid2EeSiiD   aid2EeSiiD;
Iqn2Fl_Aid2EeSiiE   aid2EeSiiE;
Iqn2Fl_Aid2EeSiiF   aid2EeSiiF;
Iqn2Fl_Aid2EeSiiG   aid2EeSiiG;
Iqn2Fl_Aid2EeSiiH   aid2EeSiiH;
Iqn2Fl_Aid2EeSieA   aid2EeSieA;
Iqn2Fl_Aid2EeSieB   aid2EeSieB;
Iqn2Fl_Aid2EeSieC   aid2EeSieC;
Iqn2Fl_Aid2EeSieD   aid2EeSieD;
Iqn2Fl_Aid2EeSieE   aid2EeSieE;
Iqn2Fl_Aid2EeSieF   aid2EeSieF;
Iqn2Fl_Aid2EeSieG   aid2EeSieG;
Iqn2Fl_Aid2EeDfe    aid2EeDfe;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV0;
   }
   aid2EeSiiA.si_ing_iq_fifo_ovfl_err = TRUE;
   aid2EeSiiA.si_ing_iq_icc_dat_info = FALSE;
   aid2EeSiiA.si_ing_iq_icc_sof_info = FALSE;
   aid2EeSiiA.si_ing_iq_ife_dat_info = FALSE;
   aid2EeSiiA.si_ing_iq_ife_eop_info = FALSE;
   aid2EeSiiA.si_ing_iq_ife_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_A_REGSET, (void *)&aid2EeSiiA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_A_REGSET, (void *)&aid2EeSiiA);

   aid2EeSiiB.si_ing_ctl_fifo_ovfl_err = TRUE;
   aid2EeSiiB.si_ing_ctl_icc_dat_info = FALSE;
   aid2EeSiiB.si_ing_ctl_icc_eop_info = FALSE;
   aid2EeSiiB.si_ing_ctl_pkt_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_B_REGSET, (void *)&aid2EeSiiB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_B_REGSET, (void *)&aid2EeSiiB);

   aid2EeSiiC.si_ing_iq_sof_err = 0xFFFFFFFF; // enable all IQ channels [31:0]
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_C_REGSET, (void *)&aid2EeSiiC);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_C_REGSET, (void *)&aid2EeSiiC);

   aid2EeSiiD.si_ing_ctl_icc_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_D_REGSET, (void *)&aid2EeSiiD);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_D_REGSET, (void *)&aid2EeSiiD);

   aid2EeSiiE.si_ing_iq_psi_dat_info = FALSE;
   aid2EeSiiE.si_ing_iq_psi_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_E_REGSET, (void *)&aid2EeSiiE);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_E_REGSET, (void *)&aid2EeSiiE);

   aid2EeSiiF.si_ing_ctl_psi_dat_info = FALSE;
   aid2EeSiiF.si_ing_ctl_psi_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_F_REGSET, (void *)&aid2EeSiiF);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_F_REGSET, (void *)&aid2EeSiiF);

   aid2EeSiiG.si_ing_iq_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_G_REGSET, (void *)&aid2EeSiiG);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_G_REGSET, (void *)&aid2EeSiiG);

   aid2EeSiiH.si_ing_ctl_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_H_REGSET, (void *)&aid2EeSiiH);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_H_REGSET, (void *)&aid2EeSiiH);

   aid2EeSieA.si_egr_iq_efe_pkt_err = TRUE;
   aid2EeSieA.si_egr_iq_efe_starve_err = TRUE;
   aid2EeSieA.si_egr_iq_efe_sym_err = TRUE;
   aid2EeSieA.si_egr_iq_icc_dat_info = FALSE;
   aid2EeSieA.si_egr_iq_icc_sof_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_A_REGSET, (void *)&aid2EeSieA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_A_REGSET, (void *)&aid2EeSieA);

   aid2EeSieB.si_egr_ctl_icc_dat_info = FALSE;
   aid2EeSieB.si_egr_ctl_icc_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_B_REGSET, (void *)&aid2EeSieB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_B_REGSET, (void *)&aid2EeSieB);

   aid2EeSieC.si_egr_ctl_icc_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_C_REGSET, (void *)&aid2EeSieC);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_C_REGSET, (void *)&aid2EeSieC);

   aid2EeSieD.si_egr_iq_psi_dat_info = FALSE;
   aid2EeSieD.si_egr_iq_psi_data_type_err = TRUE;
   aid2EeSieD.si_egr_iq_psi_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_D_REGSET, (void *)&aid2EeSieD);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_D_REGSET, (void *)&aid2EeSieD);

   aid2EeSieE.si_egr_ctl_psi_dat_info = FALSE;
   aid2EeSieE.si_egr_ctl_psi_data_type_err = TRUE;
   aid2EeSieE.si_egr_ctl_psi_eop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_E_REGSET, (void *)&aid2EeSieE);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_E_REGSET, (void *)&aid2EeSieE);

   aid2EeSieF.si_egr_iq_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_F_REGSET, (void *)&aid2EeSieF);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_F_REGSET, (void *)&aid2EeSieF);

   aid2EeSieG.si_egr_ctl_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_G_REGSET, (void *)&aid2EeSieG);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_G_REGSET, (void *)&aid2EeSieG);

   aid2EeDfe.dfe_iqn_aid2_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_DFE_REGSET, (void *)&aid2EeDfe);
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_EN_SET_EV1; // force to EV1 for DFE exceptions
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_DFE_REGSET, (void *)&aid2EeDfe);
}

/* Enable  IQN2 DIO2 Errors and Alarms */
void IQN2_enableDio2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_Dio2EeDmaIngA    dio2EeDmaIngA;
Iqn2Fl_Dio2EeDmaIngB    dio2EeDmaIngB;
Iqn2Fl_Dio2EeDmaEgrA    dio2EeDmaEgrA;
Iqn2Fl_Dio2EeDtA        dio2EeDtA;
Iqn2Fl_Dio2EeSiiA       dio2EeSiiA;
Iqn2Fl_Dio2EeSiiC       dio2EeSiiC;
Iqn2Fl_Dio2EeSiiE       dio2EeSiiE;
Iqn2Fl_Dio2EeSiiG       dio2EeSiiG;
Iqn2Fl_Dio2EeSieA       dio2EeSieA;
Iqn2Fl_Dio2EeSieD       dio2EeSieD;
Iqn2Fl_Dio2EeSieF       dio2EeSieF;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_SET_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_SET_EV0;
   }

   dio2EeDmaIngA.dma_ing_pend_que_ovf_err = TRUE;
   dio2EeDmaIngA.dma_ing_prog_err = TRUE;
   dio2EeDmaIngA.dma_ing_xfer_done_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_A_REGSET, (void *)&dio2EeDmaIngA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_A_REGSET, (void *)&dio2EeDmaIngA);

   dio2EeDmaIngB.dma_ing_buf_ovf_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_B_REGSET, (void *)&dio2EeDmaIngB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_B_REGSET, (void *)&dio2EeDmaIngB);

   dio2EeDmaEgrA.dma_egr_pend_que_ovf_err = TRUE;
   dio2EeDmaEgrA.dma_egr_prog_err = TRUE;
   dio2EeDmaEgrA.dma_egr_xfer_done_info = FALSE;
   dio2EeDmaEgrA.si_ing_iq_fifo_full_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_EGR_A_REGSET, (void *)&dio2EeDmaEgrA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_EGR_A_REGSET, (void *)&dio2EeDmaEgrA);

   dio2EeDtA.dt_buff_ovf_err = TRUE;
   dio2EeDtA.dt_done_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DT_A_REGSET, (void *)&dio2EeDtA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DT_A_REGSET, (void *)&dio2EeDtA);

   dio2EeSiiA.si_ing_iq_icc_sof_info = FALSE;
   dio2EeSiiA.si_ing_iq_icc_dat_info = FALSE;
   dio2EeSiiA.si_ing_iq_ife_sop_info = FALSE;
   dio2EeSiiA.si_ing_iq_ife_eop_info = FALSE;
   dio2EeSiiA.si_ing_iq_ife_dat_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_A_REGSET, (void *)&dio2EeSiiA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_A_REGSET, (void *)&dio2EeSiiA);

   dio2EeSiiC.si_ing_iq_sof_err = 0xFFFFFFFF; // enable all channel errors
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_C_REGSET, (void *)&dio2EeSiiC);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_C_REGSET, (void *)&dio2EeSiiC);

   dio2EeSiiE.si_ing_iq_psi_eop_info = FALSE;
   dio2EeSiiE.si_ing_iq_psi_dat_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_E_REGSET, (void *)&dio2EeSiiE);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_E_REGSET, (void *)&dio2EeSiiE);

   dio2EeSiiG.si_ing_iq_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_G_REGSET, (void *)&dio2EeSiiG);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_G_REGSET, (void *)&dio2EeSiiG);

   dio2EeSieA.si_egr_iq_efe_starve_err = TRUE;
   dio2EeSieA.si_egr_iq_efe_pkt_err = TRUE;
   dio2EeSieA.si_egr_iq_efe_sym_err = TRUE;
   dio2EeSieA.si_egr_iq_icc_sof_info = FALSE;
   dio2EeSieA.si_egr_iq_icc_dat_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_A_REGSET, (void *)&dio2EeSieA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_A_REGSET, (void *)&dio2EeSieA);

   dio2EeSieD.si_egr_iq_psi_data_type_err = TRUE;
   dio2EeSieD.si_egr_iq_psi_eop_info = FALSE;
   dio2EeSieD.si_egr_iq_psi_dat_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_D_REGSET, (void *)&dio2EeSieD);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_D_REGSET, (void *)&dio2EeSieD);

   dio2EeSieF.si_egr_iq_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_F_REGSET, (void *)&dio2EeSieF);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_F_REGSET, (void *)&dio2EeSieF);

}

/* Disable  IQN2 DIO2 Errors and Alarms */
void IQN2_disableDio2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
Iqn2Fl_Dio2EeDmaIngA    dio2EeDmaIngA;
Iqn2Fl_Dio2EeDmaIngB    dio2EeDmaIngB;
Iqn2Fl_Dio2EeDmaEgrA    dio2EeDmaEgrA;
Iqn2Fl_Dio2EeDtA        dio2EeDtA;
Iqn2Fl_Dio2EeSiiA       dio2EeSiiA;
Iqn2Fl_Dio2EeSiiC       dio2EeSiiC;
Iqn2Fl_Dio2EeSiiE       dio2EeSiiE;
Iqn2Fl_Dio2EeSiiG       dio2EeSiiG;
Iqn2Fl_Dio2EeSieA       dio2EeSieA;
Iqn2Fl_Dio2EeSieD       dio2EeSieD;
Iqn2Fl_Dio2EeSieF       dio2EeSieF;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV0;
   }

   dio2EeDmaIngA.dma_ing_pend_que_ovf_err = TRUE;
   dio2EeDmaIngA.dma_ing_prog_err = TRUE;
   dio2EeDmaIngA.dma_ing_xfer_done_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_A_REGSET, (void *)&dio2EeDmaIngA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_A_REGSET, (void *)&dio2EeDmaIngA);

   dio2EeDmaIngB.dma_ing_buf_ovf_err = TRUE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_B_REGSET, (void *)&dio2EeDmaIngB);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_B_REGSET, (void *)&dio2EeDmaIngB);

   dio2EeDmaEgrA.dma_egr_pend_que_ovf_err = TRUE;
   dio2EeDmaEgrA.dma_egr_prog_err = TRUE;
   dio2EeDmaEgrA.dma_egr_xfer_done_info = FALSE;
   dio2EeDmaEgrA.si_ing_iq_fifo_full_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_EGR_A_REGSET, (void *)&dio2EeDmaEgrA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_EGR_A_REGSET, (void *)&dio2EeDmaEgrA);

   dio2EeDtA.dt_buff_ovf_err = TRUE;
   dio2EeDtA.dt_done_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DT_A_REGSET, (void *)&dio2EeDtA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DT_A_REGSET, (void *)&dio2EeDtA);

   dio2EeSiiA.si_ing_iq_icc_sof_info = FALSE;
   dio2EeSiiA.si_ing_iq_icc_dat_info = FALSE;
   dio2EeSiiA.si_ing_iq_ife_sop_info = FALSE;
   dio2EeSiiA.si_ing_iq_ife_eop_info = FALSE;
   dio2EeSiiA.si_ing_iq_ife_dat_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_A_REGSET, (void *)&dio2EeSiiA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_A_REGSET, (void *)&dio2EeSiiA);

   dio2EeSiiC.si_ing_iq_sof_err = 0xFFFFFFFF; // enable all channel errors
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_C_REGSET, (void *)&dio2EeSiiC);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_C_REGSET, (void *)&dio2EeSiiC);

   dio2EeSiiE.si_ing_iq_psi_eop_info = FALSE;
   dio2EeSiiE.si_ing_iq_psi_dat_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_E_REGSET, (void *)&dio2EeSiiE);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_E_REGSET, (void *)&dio2EeSiiE);

   dio2EeSiiG.si_ing_iq_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_G_REGSET, (void *)&dio2EeSiiG);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_G_REGSET, (void *)&dio2EeSiiG);

   dio2EeSieA.si_egr_iq_efe_starve_err = TRUE;
   dio2EeSieA.si_egr_iq_efe_pkt_err = TRUE;
   dio2EeSieA.si_egr_iq_efe_sym_err = TRUE;
   dio2EeSieA.si_egr_iq_icc_sof_info = FALSE;
   dio2EeSieA.si_egr_iq_icc_dat_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_A_REGSET, (void *)&dio2EeSieA);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_A_REGSET, (void *)&dio2EeSieA);

   dio2EeSieD.si_egr_iq_psi_data_type_err = TRUE;
   dio2EeSieD.si_egr_iq_psi_eop_info = FALSE;
   dio2EeSieD.si_egr_iq_psi_dat_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_D_REGSET, (void *)&dio2EeSieD);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_D_REGSET, (void *)&dio2EeSieD);

   dio2EeSieF.si_egr_iq_psi_sop_info = FALSE;
   hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_F_REGSET, (void *)&dio2EeSieF);
   hIqn2->hFl->arg_ee = eventNum;
   Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_F_REGSET, (void *)&dio2EeSieF);

}

/* Enable  IQN2 AIL Errors and Alarms */
void IQN2_enableAilException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
uint32_t i;
Iqn2Fl_AilEeSiiA    ailEeSiiA;
Iqn2Fl_AilEeSiiB    ailEeSiiB;
Iqn2Fl_AilEeSiiC0   ailEeSiiC0;
Iqn2Fl_AilEeSiiC1   ailEeSiiC1;
Iqn2Fl_AilEeSiiD    ailEeSiiD;
Iqn2Fl_AilEeSiiE    ailEeSiiE;
Iqn2Fl_AilEeSiiF    ailEeSiiF;
Iqn2Fl_AilEeSiiG0   ailEeSiiG0;
Iqn2Fl_AilEeSiiG1   ailEeSiiG1;
Iqn2Fl_AilEeSiiH    ailEeSiiH;
Iqn2Fl_AilEeSieA    ailEeSieA;
Iqn2Fl_AilEeSieB    ailEeSieB;
Iqn2Fl_AilEeSieC    ailEeSieC;
Iqn2Fl_AilEeSieD    ailEeSieD;
Iqn2Fl_AilEeSieE    ailEeSieE;
Iqn2Fl_AilEeSieF0   ailEeSieF0;
Iqn2Fl_AilEeSieF1   ailEeSieF1;
Iqn2Fl_AilEeSieG    ailEeSieG;
Iqn2Fl_AilEeRm0     ailEeRm0;
Iqn2Fl_AilEeRtTm0   ailEeRtTm0;
Iqn2Fl_AilEeCiCo0   ailEeCiCo0;
Iqn2Fl_AilEePd0     ailEePd0;
Iqn2Fl_AilEePd1     ailEePd1;
Iqn2Fl_AilEePe0     ailEePe0;
Iqn2Fl_AilEeSi0     ailEeSi0;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_SET_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_SET_EV0;
   }
    for(i= 0; i< IQN2_MAX_NUM_AIL; i++)
    {
          if(1==hIqn2->ailConfig[i].ailEnable)
          {
              hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;

              ailEeSiiA.si_ing_iq_icc_sof_info = FALSE;
              ailEeSiiA.si_ing_iq_icc_dat_info = FALSE;
              ailEeSiiA.si_ing_iq_ife_sop_info = FALSE;
              ailEeSiiA.si_ing_iq_ife_eop_info = FALSE;
              ailEeSiiA.si_ing_iq_ife_dat_info = FALSE;
              ailEeSiiA.si_ing_iq_fifo_ovfl_err = TRUE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_A_REGSET, (void *)&ailEeSiiA);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_A_REGSET, (void *)&ailEeSiiA);

              ailEeSiiB.si_ing_ctl_pkt_err = TRUE;
              ailEeSiiB.si_ing_ctl_icc_eop_info = FALSE;
              ailEeSiiB.si_ing_ctl_icc_dat_info = FALSE;
              ailEeSiiB.si_ing_ctl_fifo_ovfl_err = TRUE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_B_REGSET, (void *)&ailEeSiiB);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_B_REGSET, (void *)&ailEeSiiB);

              ailEeSiiC0.si_ing_iq_sof_err = 0xFFFFFFFF; // enable all channels
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_0_REGSET, (void *)&ailEeSiiC0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_0_REGSET, (void *)&ailEeSiiC0);

              ailEeSiiC1.si_ing_iq_sof_err_64_32 = 0xFFFFFFFF; // enable all channels
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_1_REGSET, (void *)&ailEeSiiC1);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_1_REGSET, (void *)&ailEeSiiC1);

              ailEeSiiD.si_ing_ctl_icc_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_D_REGSET, (void *)&ailEeSiiD);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_D_REGSET, (void *)&ailEeSiiD);

              ailEeSiiE.si_ing_iq_psi_eop_info = FALSE;
              ailEeSiiE.si_ing_iq_psi_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_E_REGSET, (void *)&ailEeSiiE);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_E_REGSET, (void *)&ailEeSiiE);

              ailEeSiiF.si_ing_ctl_psi_eop_info = FALSE;
              ailEeSiiF.si_ing_ctl_psi_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_F_REGSET, (void *)&ailEeSiiF);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_F_REGSET, (void *)&ailEeSiiF);

              ailEeSiiG0.si_ing_iq_psi_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_0_REGSET, (void *)&ailEeSiiG0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_0_REGSET, (void *)&ailEeSiiG0);

              ailEeSiiG1.si_ing_iq_psi_sop_info_64_32 = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_1_REGSET, (void *)&ailEeSiiG1);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_1_REGSET, (void *)&ailEeSiiG1);

              ailEeSiiH.si_ing_ctl_psi_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_H_REGSET, (void *)&ailEeSiiH);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_H_REGSET, (void *)&ailEeSiiH);

              ailEeSieA.si_egr_iq_efe_starve_err = TRUE;
              ailEeSieA.si_egr_iq_efe_pkt_err = TRUE;
              ailEeSieA.si_egr_iq_efe_sym_err = TRUE;
              ailEeSieA.si_egr_iq_icc_sof_info = FALSE;
              ailEeSieA.si_egr_iq_icc_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_A_REGSET, (void *)&ailEeSieA);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_A_REGSET, (void *)&ailEeSieA);

              ailEeSieB.si_egr_ctl_icc_eop_info = FALSE;
              ailEeSieB.si_egr_ctl_icc_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_B_REGSET, (void *)&ailEeSieB);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_B_REGSET, (void *)&ailEeSieB);

              ailEeSieC.si_egr_ctl_icc_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_C_REGSET, (void *)&ailEeSieC);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_C_REGSET, (void *)&ailEeSieC);

              ailEeSieD.si_egr_iq_psi_data_type_err = TRUE;
              ailEeSieD.si_egr_iq_psi_eop_info = FALSE;
              ailEeSieD.si_egr_iq_psi_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_D_REGSET, (void *)&ailEeSieD);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_D_REGSET, (void *)&ailEeSieD);

              ailEeSieE.si_egr_ctl_psi_data_type_err = TRUE;
              ailEeSieE.si_egr_ctl_psi_eop_info = FALSE;
              ailEeSieE.si_egr_ctl_psi_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_E_REGSET, (void *)&ailEeSieE);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_E_REGSET, (void *)&ailEeSieE);

              ailEeSieF0.si_egr_iq_psi_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_0_REGSET, (void *)&ailEeSieF0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_0_REGSET, (void *)&ailEeSieF0);

              ailEeSieF1.si_egr_iq_psi_sop_info_64_32 = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_1_REGSET, (void *)&ailEeSieF1);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_1_REGSET, (void *)&ailEeSieF1);

              ailEeSieG.si_egr_ctl_psi_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_G_REGSET, (void *)&ailEeSieG);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_G_REGSET, (void *)&ailEeSieG);

              ailEeRm0.sync_status_change = TRUE;
              ailEeRm0.rm_status_state0 = TRUE;
              ailEeRm0.rm_status_state1 = TRUE;
              ailEeRm0.rm_status_state2 = TRUE;
              ailEeRm0.rm_status_state3 = TRUE;
              ailEeRm0.rm_status_state4 = TRUE;
              ailEeRm0.rm_status_state5 = TRUE;
              ailEeRm0.num_los_det = TRUE;
              ailEeRm0.lcv_det = TRUE;
              ailEeRm0.frame_bndry_det = FALSE;
              ailEeRm0.block_bndry_det = FALSE;
              ailEeRm0.missing_k28p5 = TRUE;
              ailEeRm0.loc_det = TRUE;
              ailEeRm0.rx_fifo_ovf = TRUE;
              ailEeRm0.los_err = TRUE;
              ailEeRm0.lof_err = TRUE;
              ailEeRm0.lof_state = TRUE;
              ailEeRm0.rm_rst = TRUE;
              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEeRm0.missing_k28p7 = TRUE;
                  ailEeRm0.k30p7_det = TRUE;
                  ailEeRm0.rcvd_los = FALSE;
                  ailEeRm0.rcvd_lof = FALSE;
                  ailEeRm0.rcvd_rai = FALSE;
                  ailEeRm0.rcvd_sdi = FALSE;
                  ailEeRm0.rcvd_rst = FALSE;
                  ailEeRm0.hfnsync_state = FALSE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol) {
                  ailEeRm0.missing_k28p7 = FALSE;
                  ailEeRm0.k30p7_det = FALSE;
                  ailEeRm0.rcvd_los = TRUE;
                  ailEeRm0.rcvd_lof = TRUE;
                  ailEeRm0.rcvd_rai = TRUE;
                  ailEeRm0.rcvd_sdi = TRUE;
                  ailEeRm0.rcvd_rst = TRUE;
                  ailEeRm0.hfnsync_state = TRUE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RM_0_REGSET, (void *)&ailEeRm0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RM_0_REGSET, (void *)&ailEeRm0);

              ailEeRtTm0.rt_hdr_error = TRUE;
              ailEeRtTm0.rt_em_insert = TRUE;
              ailEeRtTm0.rt_unfl = TRUE;
              ailEeRtTm0.rt_ovfl = TRUE;
              ailEeRtTm0.rt_frm_err = TRUE;
              ailEeRtTm0.rt_unalign_err = TRUE;
              ailEeRtTm0.rt_aggr_state_info = FALSE;
              ailEeRtTm0.tm_frame_sync_state = FALSE;
              ailEeRtTm0.delta_inactive = TRUE;
              ailEeRtTm0.delta_modified = TRUE;
              ailEeRtTm0.frame_misalign = TRUE;
              ailEeRtTm0.fifo_undeflow = TRUE;
              ailEeRtTm0.tm_fail = TRUE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RT_TM_0_REGSET, (void *)&ailEeRtTm0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RT_TM_0_REGSET, (void *)&ailEeRtTm0);

              ailEeCiCo0.ci_tbltoolong = TRUE;
              ailEeCiCo0.co_tbltoolong = TRUE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_CI_CO_0_REGSET, (void *)&ailEeCiCo0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_CI_CO_0_REGSET, (void *)&ailEeCiCo0);

              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEePd0.pd_ee_obsai_frm_win_err = TRUE;
                  ailEePd0.pd_ee_sop_err = FALSE;
                  ailEePd0.pd_ee_obsai_ts_miss_err = TRUE;
                  ailEePd0.pd_ee_obsai_ts_wdog_err = TRUE;
                  ailEePd0.pd_ee_obsai_axc_fail_err = TRUE;
                  ailEePd0.pd_ee_obsai_crc_err = TRUE;
                  ailEePd0.pd_ee_rp3_01_soc_rst_info = FALSE;
                  ailEePd0.pd_ee_obsai_route_fail_info = FALSE;
                  ailEePd0.pd_ee_rp3_01_capture_info = FALSE;
                  ailEePd0.pd_ee_rp3_01_crc_fail_err = TRUE;
                  ailEePd0.pd_ee_obsai_gsm_off_stb_info = FALSE;
                  ailEePd0.pd_ee_cpri_cw_crc_err = FALSE;
                  ailEePd0.pd_ee_cpri_cw_ovfl_err = FALSE;
                  ailEePd0.pd_ee_cpri_cw_4b5b_eop_err = FALSE;
                  ailEePd0.pd_ee_cpri_cw_4b5b_char_err = FALSE;
                  ailEePd0.pd_ee_obsai_sop_info = FALSE;
                  ailEePd0.pd_ee_obsai_eop_info = FALSE;
                  ailEePd0.pd_ee_obsai_sof_info = FALSE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
                  ailEePd0.pd_ee_obsai_frm_win_err = FALSE;
                  ailEePd0.pd_ee_sop_err = TRUE;
                  ailEePd0.pd_ee_obsai_ts_miss_err = FALSE;
                  ailEePd0.pd_ee_obsai_ts_wdog_err = FALSE;
                  ailEePd0.pd_ee_obsai_axc_fail_err = FALSE;
                  ailEePd0.pd_ee_obsai_crc_err = FALSE;
                  ailEePd0.pd_ee_rp3_01_soc_rst_info = FALSE;
                  ailEePd0.pd_ee_obsai_route_fail_info = FALSE;
                  ailEePd0.pd_ee_rp3_01_capture_info = FALSE;
                  ailEePd0.pd_ee_rp3_01_crc_fail_err = FALSE;
                  ailEePd0.pd_ee_obsai_gsm_off_stb_info = FALSE;
                  ailEePd0.pd_ee_cpri_cw_crc_err = TRUE;
                  ailEePd0.pd_ee_cpri_cw_ovfl_err = TRUE;
                  ailEePd0.pd_ee_cpri_cw_4b5b_eop_err = TRUE;
                  ailEePd0.pd_ee_cpri_cw_4b5b_char_err = TRUE;
                  ailEePd0.pd_ee_obsai_sop_info = FALSE;
                  ailEePd0.pd_ee_obsai_eop_info = FALSE;
                  ailEePd0.pd_ee_obsai_sof_info = FALSE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_0_REGSET, (void *)&ailEePd0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_0_REGSET, (void *)&ailEePd0);

              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEePd1.pd_ee_cpri_bub_fsm_err = FALSE;
                  ailEePd1.pd_ee_cpri_tdm_fsm_err = FALSE;
                  ailEePd1.pd_ee_cpri_radstd_err = FALSE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
                  ailEePd1.pd_ee_cpri_bub_fsm_err = TRUE;
                  ailEePd1.pd_ee_cpri_tdm_fsm_err = TRUE;
                  ailEePd1.pd_ee_cpri_radstd_err = TRUE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_1_REGSET, (void *)&ailEePd1);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_1_REGSET, (void *)&ailEePd1);

              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEePe0.pe_ee_cpri_cw_null_starve_err = FALSE;
                  ailEePe0.pe_ee_cpri_cw_4b5b_starve_err = FALSE;
                  ailEePe0.pe_ee_cpri_cw_hypfm_starve_err = FALSE;
                  ailEePe0.pe_ee_cpri_cw_hdlc_starve_err = FALSE;
                  ailEePe0.pe_ee_cpri_cw_hypfm_oflow_err = FALSE;
                  ailEePe0.pe_ee_ofifo_oflow_err = TRUE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
                  ailEePe0.pe_ee_cpri_cw_null_starve_err = TRUE;
                  ailEePe0.pe_ee_cpri_cw_4b5b_starve_err = TRUE;
                  ailEePe0.pe_ee_cpri_cw_hypfm_starve_err = TRUE;
                  ailEePe0.pe_ee_cpri_cw_hdlc_starve_err = TRUE;
                  ailEePe0.pe_ee_cpri_cw_hypfm_oflow_err = TRUE;
                  ailEePe0.pe_ee_ofifo_oflow_err = TRUE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PE_0_REGSET, (void *)&ailEePe0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PE_0_REGSET, (void *)&ailEePe0);

              ailEeSi0.uat_pi_err = TRUE;
              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEeSi0.cpri_tdm_lut_err = FALSE;
                  ailEeSi0.cpri_bub_fsm_err = FALSE;
                  ailEeSi0.obsai_phy_sync_err = TRUE;
                  ailEeSi0.obsai_multrulefire_err = TRUE;
                  ailEeSi0.obsai_dbm_wrap_err = TRUE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
                  ailEeSi0.cpri_tdm_lut_err = TRUE;
                  ailEeSi0.cpri_bub_fsm_err = TRUE;
                  ailEeSi0.obsai_phy_sync_err = FALSE;
                  ailEeSi0.obsai_multrulefire_err = FALSE;
                  ailEeSi0.obsai_dbm_wrap_err = FALSE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SI_0_REGSET, (void *)&ailEeSi0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SI_0_REGSET, (void *)&ailEeSi0);
          }
      }
}

/* Disable  IQN2 AIL Errors and Alarms */
void IQN2_disableAilException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
uint32_t i;
Iqn2Fl_AilEeSiiA    ailEeSiiA;
Iqn2Fl_AilEeSiiB    ailEeSiiB;
Iqn2Fl_AilEeSiiC0   ailEeSiiC0;
Iqn2Fl_AilEeSiiC1   ailEeSiiC1;
Iqn2Fl_AilEeSiiD    ailEeSiiD;
Iqn2Fl_AilEeSiiE    ailEeSiiE;
Iqn2Fl_AilEeSiiF    ailEeSiiF;
Iqn2Fl_AilEeSiiG0   ailEeSiiG0;
Iqn2Fl_AilEeSiiG1   ailEeSiiG1;
Iqn2Fl_AilEeSiiH    ailEeSiiH;
Iqn2Fl_AilEeSieA    ailEeSieA;
Iqn2Fl_AilEeSieB    ailEeSieB;
Iqn2Fl_AilEeSieC    ailEeSieC;
Iqn2Fl_AilEeSieD    ailEeSieD;
Iqn2Fl_AilEeSieE    ailEeSieE;
Iqn2Fl_AilEeSieF0   ailEeSieF0;
Iqn2Fl_AilEeSieF1   ailEeSieF1;
Iqn2Fl_AilEeSieG    ailEeSieG;
Iqn2Fl_AilEeRm0     ailEeRm0;
Iqn2Fl_AilEeRtTm0   ailEeRtTm0;
Iqn2Fl_AilEeCiCo0   ailEeCiCo0;
Iqn2Fl_AilEePd0     ailEePd0;
Iqn2Fl_AilEePd1     ailEePd1;
Iqn2Fl_AilEePe0     ailEePe0;
Iqn2Fl_AilEeSi0     ailEeSi0;
Iqn2Fl_EeArgIndex               eventNum;

   if (interruptEventNum == 1) {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV1;
   } else {
       eventNum = IQN2FL_EE_INT_EN_CLR_EV0;
   }
    for(i= 0; i< IQN2_MAX_NUM_AIL; i++)
    {
          if(1==hIqn2->ailConfig[i].ailEnable)
          {
              hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;

              ailEeSiiA.si_ing_iq_icc_sof_info = FALSE;
              ailEeSiiA.si_ing_iq_icc_dat_info = FALSE;
              ailEeSiiA.si_ing_iq_ife_sop_info = FALSE;
              ailEeSiiA.si_ing_iq_ife_eop_info = FALSE;
              ailEeSiiA.si_ing_iq_ife_dat_info = FALSE;
              ailEeSiiA.si_ing_iq_fifo_ovfl_err = TRUE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_A_REGSET, (void *)&ailEeSiiA);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_A_REGSET, (void *)&ailEeSiiA);

              ailEeSiiB.si_ing_ctl_pkt_err = TRUE;
              ailEeSiiB.si_ing_ctl_icc_eop_info = FALSE;
              ailEeSiiB.si_ing_ctl_icc_dat_info = FALSE;
              ailEeSiiB.si_ing_ctl_fifo_ovfl_err = TRUE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_B_REGSET, (void *)&ailEeSiiB);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_B_REGSET, (void *)&ailEeSiiB);

              ailEeSiiC0.si_ing_iq_sof_err = 0xFFFFFFFF; // enable all channels
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_0_REGSET, (void *)&ailEeSiiC0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_0_REGSET, (void *)&ailEeSiiC0);

              ailEeSiiC1.si_ing_iq_sof_err_64_32 = 0xFFFFFFFF; // enable all channels
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_1_REGSET, (void *)&ailEeSiiC1);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_1_REGSET, (void *)&ailEeSiiC1);

              ailEeSiiD.si_ing_ctl_icc_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_D_REGSET, (void *)&ailEeSiiD);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_D_REGSET, (void *)&ailEeSiiD);

              ailEeSiiE.si_ing_iq_psi_eop_info = FALSE;
              ailEeSiiE.si_ing_iq_psi_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_E_REGSET, (void *)&ailEeSiiE);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_E_REGSET, (void *)&ailEeSiiE);

              ailEeSiiF.si_ing_ctl_psi_eop_info = FALSE;
              ailEeSiiF.si_ing_ctl_psi_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_F_REGSET, (void *)&ailEeSiiF);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_F_REGSET, (void *)&ailEeSiiF);

              ailEeSiiG0.si_ing_iq_psi_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_0_REGSET, (void *)&ailEeSiiG0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_0_REGSET, (void *)&ailEeSiiG0);

              ailEeSiiG1.si_ing_iq_psi_sop_info_64_32 = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_1_REGSET, (void *)&ailEeSiiG1);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_1_REGSET, (void *)&ailEeSiiG1);

              ailEeSiiH.si_ing_ctl_psi_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_H_REGSET, (void *)&ailEeSiiH);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_H_REGSET, (void *)&ailEeSiiH);

              ailEeSieA.si_egr_iq_efe_starve_err = TRUE;
              ailEeSieA.si_egr_iq_efe_pkt_err = TRUE;
              ailEeSieA.si_egr_iq_efe_sym_err = TRUE;
              ailEeSieA.si_egr_iq_icc_sof_info = FALSE;
              ailEeSieA.si_egr_iq_icc_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_A_REGSET, (void *)&ailEeSieA);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_A_REGSET, (void *)&ailEeSieA);

              ailEeSieB.si_egr_ctl_icc_eop_info = FALSE;
              ailEeSieB.si_egr_ctl_icc_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_B_REGSET, (void *)&ailEeSieB);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_B_REGSET, (void *)&ailEeSieB);

              ailEeSieC.si_egr_ctl_icc_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_C_REGSET, (void *)&ailEeSieC);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_C_REGSET, (void *)&ailEeSieC);

              ailEeSieD.si_egr_iq_psi_data_type_err = TRUE;
              ailEeSieD.si_egr_iq_psi_eop_info = FALSE;
              ailEeSieD.si_egr_iq_psi_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_D_REGSET, (void *)&ailEeSieD);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_D_REGSET, (void *)&ailEeSieD);

              ailEeSieE.si_egr_ctl_psi_data_type_err = TRUE;
              ailEeSieE.si_egr_ctl_psi_eop_info = FALSE;
              ailEeSieE.si_egr_ctl_psi_dat_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_E_REGSET, (void *)&ailEeSieE);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_E_REGSET, (void *)&ailEeSieE);

              ailEeSieF0.si_egr_iq_psi_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_0_REGSET, (void *)&ailEeSieF0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_0_REGSET, (void *)&ailEeSieF0);

              ailEeSieF1.si_egr_iq_psi_sop_info_64_32 = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_1_REGSET, (void *)&ailEeSieF1);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_1_REGSET, (void *)&ailEeSieF1);

              ailEeSieG.si_egr_ctl_psi_sop_info = FALSE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_G_REGSET, (void *)&ailEeSieG);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_G_REGSET, (void *)&ailEeSieG);

              ailEeRm0.sync_status_change = TRUE;
              ailEeRm0.rm_status_state0 = TRUE;
              ailEeRm0.rm_status_state1 = TRUE;
              ailEeRm0.rm_status_state2 = TRUE;
              ailEeRm0.rm_status_state3 = TRUE;
              ailEeRm0.rm_status_state4 = TRUE;
              ailEeRm0.rm_status_state5 = TRUE;
              ailEeRm0.num_los_det = TRUE;
              ailEeRm0.lcv_det = TRUE;
              ailEeRm0.frame_bndry_det = FALSE;
              ailEeRm0.block_bndry_det = FALSE;
              ailEeRm0.missing_k28p5 = TRUE;
              ailEeRm0.loc_det = TRUE;
              ailEeRm0.rx_fifo_ovf = TRUE;
              ailEeRm0.los_err = TRUE;
              ailEeRm0.lof_err = TRUE;
              ailEeRm0.lof_state = TRUE;
              ailEeRm0.rm_rst = TRUE;
              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEeRm0.missing_k28p7 = TRUE;
                  ailEeRm0.k30p7_det = TRUE;
                  ailEeRm0.rcvd_los = FALSE;
                  ailEeRm0.rcvd_lof = FALSE;
                  ailEeRm0.rcvd_rai = FALSE;
                  ailEeRm0.rcvd_sdi = FALSE;
                  ailEeRm0.rcvd_rst = FALSE;
                  ailEeRm0.hfnsync_state = FALSE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol) {
                  ailEeRm0.missing_k28p7 = FALSE;
                  ailEeRm0.k30p7_det = FALSE;
                  ailEeRm0.rcvd_los = TRUE;
                  ailEeRm0.rcvd_lof = TRUE;
                  ailEeRm0.rcvd_rai = TRUE;
                  ailEeRm0.rcvd_sdi = TRUE;
                  ailEeRm0.rcvd_rst = TRUE;
                  ailEeRm0.hfnsync_state = TRUE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RM_0_REGSET, (void *)&ailEeRm0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RM_0_REGSET, (void *)&ailEeRm0);

              ailEeRtTm0.rt_hdr_error = TRUE;
              ailEeRtTm0.rt_em_insert = TRUE;
              ailEeRtTm0.rt_unfl = TRUE;
              ailEeRtTm0.rt_ovfl = TRUE;
              ailEeRtTm0.rt_frm_err = TRUE;
              ailEeRtTm0.rt_unalign_err = TRUE;
              ailEeRtTm0.rt_aggr_state_info = FALSE;
              ailEeRtTm0.tm_frame_sync_state = FALSE;
              ailEeRtTm0.delta_inactive = TRUE;
              ailEeRtTm0.delta_modified = TRUE;
              ailEeRtTm0.frame_misalign = TRUE;
              ailEeRtTm0.fifo_undeflow = TRUE;
              ailEeRtTm0.tm_fail = TRUE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RT_TM_0_REGSET, (void *)&ailEeRtTm0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RT_TM_0_REGSET, (void *)&ailEeRtTm0);

              ailEeCiCo0.ci_tbltoolong = TRUE;
              ailEeCiCo0.co_tbltoolong = TRUE;
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_CI_CO_0_REGSET, (void *)&ailEeCiCo0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_CI_CO_0_REGSET, (void *)&ailEeCiCo0);

              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEePd0.pd_ee_obsai_frm_win_err = TRUE;
                  ailEePd0.pd_ee_sop_err = FALSE;
                  ailEePd0.pd_ee_obsai_ts_miss_err = TRUE;
                  ailEePd0.pd_ee_obsai_ts_wdog_err = TRUE;
                  ailEePd0.pd_ee_obsai_axc_fail_err = TRUE;
                  ailEePd0.pd_ee_obsai_crc_err = TRUE;
                  ailEePd0.pd_ee_rp3_01_soc_rst_info = FALSE;
                  ailEePd0.pd_ee_obsai_route_fail_info = FALSE;
                  ailEePd0.pd_ee_rp3_01_capture_info = FALSE;
                  ailEePd0.pd_ee_rp3_01_crc_fail_err = TRUE;
                  ailEePd0.pd_ee_obsai_gsm_off_stb_info = FALSE;
                  ailEePd0.pd_ee_cpri_cw_crc_err = FALSE;
                  ailEePd0.pd_ee_cpri_cw_ovfl_err = FALSE;
                  ailEePd0.pd_ee_cpri_cw_4b5b_eop_err = FALSE;
                  ailEePd0.pd_ee_cpri_cw_4b5b_char_err = FALSE;
                  ailEePd0.pd_ee_obsai_sop_info = FALSE;
                  ailEePd0.pd_ee_obsai_eop_info = FALSE;
                  ailEePd0.pd_ee_obsai_sof_info = FALSE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
                  ailEePd0.pd_ee_obsai_frm_win_err = FALSE;
                  ailEePd0.pd_ee_sop_err = TRUE;
                  ailEePd0.pd_ee_obsai_ts_miss_err = FALSE;
                  ailEePd0.pd_ee_obsai_ts_wdog_err = FALSE;
                  ailEePd0.pd_ee_obsai_axc_fail_err = FALSE;
                  ailEePd0.pd_ee_obsai_crc_err = FALSE;
                  ailEePd0.pd_ee_rp3_01_soc_rst_info = FALSE;
                  ailEePd0.pd_ee_obsai_route_fail_info = FALSE;
                  ailEePd0.pd_ee_rp3_01_capture_info = FALSE;
                  ailEePd0.pd_ee_rp3_01_crc_fail_err = FALSE;
                  ailEePd0.pd_ee_obsai_gsm_off_stb_info = FALSE;
                  ailEePd0.pd_ee_cpri_cw_crc_err = TRUE;
                  ailEePd0.pd_ee_cpri_cw_ovfl_err = TRUE;
                  ailEePd0.pd_ee_cpri_cw_4b5b_eop_err = TRUE;
                  ailEePd0.pd_ee_cpri_cw_4b5b_char_err = TRUE;
                  ailEePd0.pd_ee_obsai_sop_info = FALSE;
                  ailEePd0.pd_ee_obsai_eop_info = FALSE;
                  ailEePd0.pd_ee_obsai_sof_info = FALSE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_0_REGSET, (void *)&ailEePd0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_0_REGSET, (void *)&ailEePd0);

              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEePd1.pd_ee_cpri_bub_fsm_err = FALSE;
                  ailEePd1.pd_ee_cpri_tdm_fsm_err = FALSE;
                  ailEePd1.pd_ee_cpri_radstd_err = FALSE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
                  ailEePd1.pd_ee_cpri_bub_fsm_err = TRUE;
                  ailEePd1.pd_ee_cpri_tdm_fsm_err = TRUE;
                  ailEePd1.pd_ee_cpri_radstd_err = TRUE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_1_REGSET, (void *)&ailEePd1);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_1_REGSET, (void *)&ailEePd1);

              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEePe0.pe_ee_cpri_cw_null_starve_err = FALSE;
                  ailEePe0.pe_ee_cpri_cw_4b5b_starve_err = FALSE;
                  ailEePe0.pe_ee_cpri_cw_hypfm_starve_err = FALSE;
                  ailEePe0.pe_ee_cpri_cw_hdlc_starve_err = FALSE;
                  ailEePe0.pe_ee_cpri_cw_hypfm_oflow_err = FALSE;
                  ailEePe0.pe_ee_ofifo_oflow_err = TRUE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
                  ailEePe0.pe_ee_cpri_cw_null_starve_err = TRUE;
                  ailEePe0.pe_ee_cpri_cw_4b5b_starve_err = TRUE;
                  ailEePe0.pe_ee_cpri_cw_hypfm_starve_err = TRUE;
                  ailEePe0.pe_ee_cpri_cw_hdlc_starve_err = TRUE;
                  ailEePe0.pe_ee_cpri_cw_hypfm_oflow_err = TRUE;
                  ailEePe0.pe_ee_ofifo_oflow_err = TRUE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PE_0_REGSET, (void *)&ailEePe0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PE_0_REGSET, (void *)&ailEePe0);

              ailEeSi0.uat_pi_err = TRUE;
              if (IQN2FL_PROTOCOL_OBSAI==hIqn2->protocol){
                  ailEeSi0.cpri_tdm_lut_err = FALSE;
                  ailEeSi0.cpri_bub_fsm_err = FALSE;
                  ailEeSi0.obsai_phy_sync_err = TRUE;
                  ailEeSi0.obsai_multrulefire_err = TRUE;
                  ailEeSi0.obsai_dbm_wrap_err = TRUE;
              } else if (IQN2FL_PROTOCOL_CPRI==hIqn2->protocol){
                  ailEeSi0.cpri_tdm_lut_err = TRUE;
                  ailEeSi0.cpri_bub_fsm_err = TRUE;
                  ailEeSi0.obsai_phy_sync_err = FALSE;
                  ailEeSi0.obsai_multrulefire_err = FALSE;
                  ailEeSi0.obsai_dbm_wrap_err = FALSE;
              }
              hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SI_0_REGSET, (void *)&ailEeSi0);
              hIqn2->hFl->arg_ee = eventNum;
              Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SI_0_REGSET, (void *)&ailEeSi0);
          }
      }
}

/* Enable  IQN2 Errors and Alarms */
void IQN2_enableException(
		IQN2_ConfigHandle  hIqn2
)
{
uint32_t eoi;
#ifdef _TMS320C6X
uint32_t gie;
   gie = _disable_interrupts();
#endif

   reset_ExceptionStats(hIqn2);

   /* TOP errors on EV0 */
   IQN2_enableTopException(hIqn2,0);
   /* AT2 errors on EV0 */
   IQN2_enableAt2Exception(hIqn2,0);
   /* IQS2 errors on EV0 */
   IQN2_enableIqs2Exception(hIqn2,0);
   /* AID2 errors on EV0 */
   if (hIqn2->aidConfig.aidEnable)
       IQN2_enableAid2Exception(hIqn2,0);
   /* DIO2 errors on EV0 */
   IQN2_enableDio2Exception(hIqn2,0);
   /* AIL errors on EV0 */
   IQN2_enableAilException(hIqn2,0);

	eoi = 0;
	Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_EE_EOI_0_REG, (void *)&eoi);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_EE_EOI_1_REG, (void *)&eoi);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_EE_EOI_CPPI_REG, (void *)&eoi);
#ifdef _TMS320C6X
	 _restore_interrupts(gie);
#endif

}

/* Disable  IQN2 Errors and Alarms */
void IQN2_disableException(
        IQN2_ConfigHandle  hIqn2
)
{
#ifdef _TMS320C6X
uint32_t gie;
   gie = _disable_interrupts();
#endif

   /* TOP errors on EV0 */
   IQN2_disableTopException(hIqn2,0);
   /* AT2 errors on EV0 */
   IQN2_disableAt2Exception(hIqn2,0);
   /* IQS2 errors on EV0 */
   IQN2_disableIqs2Exception(hIqn2,0);
   /* AID2 errors on EV0 */
   if (hIqn2->aidConfig.aidEnable)
       IQN2_disableAid2Exception(hIqn2,0);
   /* DIO2 errors on EV0 */
   IQN2_disableDio2Exception(hIqn2,0);
   /* AIL errors on EV0 */
   IQN2_disableAilException(hIqn2,0);

#ifdef _TMS320C6X
     _restore_interrupts(gie);
#endif
}

/* Get IQN2 PSR Errors and Alarms status and clear */
void IQN2_getPsrException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
    Iqn2Fl_PsrEeIngFlushA           psrEeIngFlushA;
    Iqn2Fl_PsrEeIngFlushB           psrEeIngFlushB;
    Iqn2Fl_PsrEeEgressProtocolErrA  psrEeEgressProtocolErrA;
    Iqn2Fl_PsrEeEgressProtocolErrB  psrEeEgressProtocolErrB;
    Iqn2Fl_PsrEeOrigin              eePsrOrigin;
    Iqn2Fl_EeArgIndex               eventNum;

    if (interruptEventNum == 1) {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV1;
    } else {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV0;
    }
    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_PSR_EE_ORGN_STS, (void *)&eePsrOrigin);
    if (eePsrOrigin.psr_ee_ing_flush_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_PSR_EE_ING_FLUSH_A_STS, (void *)&psrEeIngFlushA);
        hIqn2->iqn2EeCount.eeTop.psrEeIngFlushACnt.err = psrEeIngFlushA.err; // error bit per channel [31:0]
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_A_REGSET, (void *)&psrEeIngFlushA);

    }
    if (eePsrOrigin.psr_ee_ing_flush_b_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_PSR_EE_ING_FLUSH_B_STS, (void *)&psrEeIngFlushB);
        hIqn2->iqn2EeCount.eeTop.psrEeIngFlushBCnt.err = psrEeIngFlushB.err; // error bit per channel [48:32]
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_ING_FLUSH_B_REGSET, (void *)&psrEeIngFlushB);

    }
    if (eePsrOrigin.psr_ee_egr_protocol_err_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_PSR_EE_EGR_PROTOCOL_ERR_A_STS, (void *)&psrEeEgressProtocolErrA);
        hIqn2->iqn2EeCount.eeTop.psrEeEgressProtocolErrACnt.err = psrEeEgressProtocolErrA.err; // error bit per channel [31:0]
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_A_REGSET, (void *)&psrEeEgressProtocolErrA);
    }
    if (eePsrOrigin.psr_ee_egr_protocol_err_b_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_PSR_EE_EGR_PROTOCOL_ERR_B_STS, (void *)&psrEeEgressProtocolErrB);
        hIqn2->iqn2EeCount.eeTop.psrEeEgressProtocolErrBCnt.err = psrEeEgressProtocolErrB.err; // error bit per channel [48:32]
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_B_REGSET, (void *)&psrEeEgressProtocolErrB);
    }
}

/* Get IQN2 PKTDMA Errors and Alarms status and clear */
void IQN2_getPktDMAException(
        IQN2_ConfigHandle  hIqn2
)
{
    Iqn2Fl_PktdmaEeDescStarve       pktdmaEeDescStarve;

    // only PKTDMA_STARVE interrupt available for PktDMA exceptions
    hIqn2->hFl->arg_ee = IQN2FL_EE_INT_EN_STATUS_EV0;
    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_PKTDMA_EE_PKTDMA_DESC_STARVE_STS, (void *)&pktdmaEeDescStarve);
    hIqn2->iqn2EeCount.eeTop.pktdmaEeDescStarveCnt.mop_err += pktdmaEeDescStarve.mop_err; if (pktdmaEeDescStarve.mop_err) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->iqn2EeCount.eeTop.pktdmaEeDescStarveCnt.sop_err += pktdmaEeDescStarve.sop_err; if (pktdmaEeDescStarve.sop_err) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET, (void *)&pktdmaEeDescStarve);
}

/* Get IQN2 AT2 Errors and Alarms status and clear */
void IQN2_getAt2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
    Iqn2Fl_At2EeInfoErr             at2EeInfoErr;
    Iqn2Fl_EeArgIndex               eventNum;

    if (interruptEventNum == 1) {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV1;
    } else {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV0;
    }
    hIqn2->hFl->arg_ee = eventNum;
    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AT2_AT_EE_0_STS, (void *)&at2EeInfoErr);
    hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.pa_tscomp_info += at2EeInfoErr.pa_tscomp_info; //if (at2EeInfoErr.pa_tscomp_info) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.physync_info   += at2EeInfoErr.physync_info; //if (at2EeInfoErr.physync_info) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.radsync_info   += at2EeInfoErr.radsync_info; //if (at2EeInfoErr.radsync_info) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.rp1_bit_err    += at2EeInfoErr.rp1_bit_err; if (at2EeInfoErr.rp1_bit_err) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.rp1_crc_err    += at2EeInfoErr.rp1_crc_err; if (at2EeInfoErr.rp1_crc_err) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.rp1_sync_info  += at2EeInfoErr.rp1_sync_info; //if (at2EeInfoErr.rp1_sync_info) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_AT_EE_0_REGSET, (void *)&at2EeInfoErr);
}

/* Get IQN2 IQS2 Errors and Alarms status and clear */
void IQN2_getIqs2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
    Iqn2Fl_Iqs2EeChanErr        iqs2EeChanErr;
    Iqn2Fl_Iqs2EeIngFlush       iqs2EeIngFlush;
    Iqn2Fl_Iqs2EeEgrFlush       iqs2EeEgrFlush;
    Iqn2Fl_Iqs2EeOrigin         eeIqs2Origin;
    Iqn2Fl_EeArgIndex           eventNum;

    if (interruptEventNum == 1) {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV1;
    } else {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV0;
    }

    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_IQS2_EE_ORGN_STS, (void *)&eeIqs2Origin);
    if (eeIqs2Origin.iqs2_ee_chan_err_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_IQS_EE_CHAN_ERR_STS, (void *)&iqs2EeChanErr);
        hIqn2->iqn2EeCount.eeIqs2.iqs2EeChanErrCnt.egr_mux_err += iqs2EeChanErr.egr_mux_err;
        hIqn2->iqn2EeCount.eeIqs2.iqs2EeChanErrCnt.ing_dio2_err += iqs2EeChanErr.ing_dio2_err;
        hIqn2->iqn2EeCount.eeIqs2.iqs2EeChanErrCnt.ing_pktdma_err += iqs2EeChanErr.ing_pktdma_err;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_CHAN_ERR_REGSET, (void *)&iqs2EeChanErr);
    }
    if (eeIqs2Origin.iqs2_ee_ing_flush_err_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_IQS_EE_ING_FLUSH_ERR_STS, (void *)&iqs2EeIngFlush);
        hIqn2->iqn2EeCount.eeIqs2.iqs2EeIngFlushCnt.flush += iqs2EeIngFlush.flush;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_ING_FLUSH_ERR_REGSET, (void *)&iqs2EeIngFlush);
    }
    if (eeIqs2Origin.iqs2_ee_egr_flush_err_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_IQS_EE_ING_FLUSH_ERR_STS, (void *)&iqs2EeEgrFlush);
        hIqn2->iqn2EeCount.eeIqs2.iqs2EeEgrFlushCnt.flush += iqs2EeEgrFlush.flush;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EE_ING_FLUSH_ERR_REGSET, (void *)&iqs2EeEgrFlush);
    }
}

/* Get IQN2 AID2 Errors and Alarms status and clear */
void IQN2_getAid2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
    Iqn2Fl_Aid2EeSiiA   aid2EeSiiA;
    Iqn2Fl_Aid2EeSiiB   aid2EeSiiB;
    Iqn2Fl_Aid2EeSiiC   aid2EeSiiC;
    Iqn2Fl_Aid2EeSiiD   aid2EeSiiD;
    Iqn2Fl_Aid2EeSiiE   aid2EeSiiE;
    Iqn2Fl_Aid2EeSiiF   aid2EeSiiF;
    Iqn2Fl_Aid2EeSiiG   aid2EeSiiG;
    Iqn2Fl_Aid2EeSiiH   aid2EeSiiH;
    Iqn2Fl_Aid2EeSieA   aid2EeSieA;
    Iqn2Fl_Aid2EeSieB   aid2EeSieB;
    Iqn2Fl_Aid2EeSieC   aid2EeSieC;
    Iqn2Fl_Aid2EeSieD   aid2EeSieD;
    Iqn2Fl_Aid2EeSieE   aid2EeSieE;
    Iqn2Fl_Aid2EeSieF   aid2EeSieF;
    Iqn2Fl_Aid2EeSieG   aid2EeSieG;
    Iqn2Fl_Aid2SysClkEeOrigin   eeAid2SysClkOrigin;
    Iqn2Fl_Aid2VbusClkEeOrigin  eeAid2VbusClkOrigin;
    Iqn2Fl_EeArgIndex           eventNum;

    if (interruptEventNum == 1) {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV1;
    } else {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV0;
    }

    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_SYSCLK_EE_ORGN_STS, (void *)&eeAid2SysClkOrigin);
    if (eeAid2SysClkOrigin.aid2_ee_sii_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SII_A_STS, (void *)&aid2EeSiiA);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiACnt.si_ing_iq_fifo_ovfl_err += aid2EeSiiA.si_ing_iq_fifo_ovfl_err;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiACnt.si_ing_iq_icc_dat_info += aid2EeSiiA.si_ing_iq_icc_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiACnt.si_ing_iq_icc_dat_info += aid2EeSiiA.si_ing_iq_icc_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiACnt.si_ing_iq_ife_dat_info += aid2EeSiiA.si_ing_iq_ife_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiACnt.si_ing_iq_ife_eop_info += aid2EeSiiA.si_ing_iq_ife_eop_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiACnt.si_ing_iq_ife_sop_info += aid2EeSiiA.si_ing_iq_ife_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_A_REGSET, (void *)&aid2EeSiiA);
    }
    if (eeAid2SysClkOrigin.aid2_ee_sii_b_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SII_B_STS, (void *)&aid2EeSiiB);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiBCnt.si_ing_ctl_fifo_ovfl_err += aid2EeSiiB.si_ing_ctl_fifo_ovfl_err;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiBCnt.si_ing_ctl_icc_dat_info += aid2EeSiiB.si_ing_ctl_icc_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiBCnt.si_ing_ctl_icc_eop_info += aid2EeSiiB.si_ing_ctl_icc_eop_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiBCnt.si_ing_ctl_pkt_err += aid2EeSiiB.si_ing_ctl_pkt_err;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_B_REGSET, (void *)&aid2EeSiiB);
    }
    if (eeAid2SysClkOrigin.aid2_ee_sii_c_sts) {
    	if (hIqn2->aidConfig.aidEnable != 1) //For AID mode, start of frame isn't critical as for CPRI, since the DFE protocol isn't carrying any sync signal in the data. DFE is using other sync signal to work with IQN2
    		hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SII_C_STS, (void *)&aid2EeSiiC);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiCCnt.si_ing_iq_sof_err = aid2EeSiiC.si_ing_iq_sof_err; // this is per-channel
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_C_REGSET, (void *)&aid2EeSiiC);
    }
    if (eeAid2SysClkOrigin.aid2_ee_sii_d_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SII_D_STS, (void *)&aid2EeSiiD);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiDCnt.si_ing_ctl_icc_sop_info += aid2EeSiiD.si_ing_ctl_icc_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_D_REGSET, (void *)&aid2EeSiiD);
    }
    if (eeAid2SysClkOrigin.aid2_ee_sie_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SIE_A_STS, (void *)&aid2EeSieA);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieACnt.si_egr_iq_efe_pkt_err += aid2EeSieA.si_egr_iq_efe_pkt_err;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieACnt.si_egr_iq_efe_starve_err += aid2EeSieA.si_egr_iq_efe_starve_err;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieACnt.si_egr_iq_efe_sym_err += aid2EeSieA.si_egr_iq_efe_sym_err;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieACnt.si_egr_iq_icc_dat_info += aid2EeSieA.si_egr_iq_icc_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieACnt.si_egr_iq_icc_sof_info += aid2EeSieA.si_egr_iq_icc_sof_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_A_REGSET, (void *)&aid2EeSieA);

    }
    if (eeAid2SysClkOrigin.aid2_ee_sie_b_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SIE_B_STS, (void *)&aid2EeSieB);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieBCnt.si_egr_ctl_icc_dat_info += aid2EeSieB.si_egr_ctl_icc_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieBCnt.si_egr_ctl_icc_eop_info += aid2EeSieB.si_egr_ctl_icc_eop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_B_REGSET, (void *)&aid2EeSieB);

    }
    if (eeAid2SysClkOrigin.aid2_ee_sie_c_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SIE_C_STS, (void *)&aid2EeSieC);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieCCnt.si_egr_ctl_icc_sop_info += aid2EeSieC.si_egr_ctl_icc_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_C_REGSET, (void *)&aid2EeSieC);
    }
    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_VBUSCLK_EE_ORGN_STS, (void *)&eeAid2VbusClkOrigin);
    if (eeAid2VbusClkOrigin.aid2_ee_sii_e_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SII_E_STS, (void *)&aid2EeSiiE);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiECnt.si_ing_iq_psi_dat_info += aid2EeSiiE.si_ing_iq_psi_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiECnt.si_ing_iq_psi_eop_info += aid2EeSiiE.si_ing_iq_psi_eop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_E_REGSET, (void *)&aid2EeSiiE);
    }
    if (eeAid2VbusClkOrigin.aid2_ee_sii_f_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SII_F_STS, (void *)&aid2EeSiiF);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiFCnt.si_ing_ctl_psi_dat_info += aid2EeSiiF.si_ing_ctl_psi_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiFCnt.si_ing_ctl_psi_eop_info += aid2EeSiiF.si_ing_ctl_psi_eop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_F_REGSET, (void *)&aid2EeSiiF);
    }
    if (eeAid2VbusClkOrigin.aid2_ee_sii_g_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SII_G_STS, (void *)&aid2EeSiiG);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiGCnt.si_ing_iq_psi_sop_info += aid2EeSiiG.si_ing_iq_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_G_REGSET, (void *)&aid2EeSiiG);
    }
    if (eeAid2VbusClkOrigin.aid2_ee_sii_h_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SII_H_STS, (void *)&aid2EeSiiH);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSiiHCnt.si_ing_ctl_psi_sop_info += aid2EeSiiH.si_ing_ctl_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SII_H_REGSET, (void *)&aid2EeSiiH);
    }
    if (eeAid2VbusClkOrigin.aid2_ee_sie_d_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SIE_D_STS, (void *)&aid2EeSieD);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieDCnt.si_egr_iq_psi_dat_info += aid2EeSieD.si_egr_iq_psi_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieDCnt.si_egr_iq_psi_data_type_err += aid2EeSieD.si_egr_iq_psi_data_type_err;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieDCnt.si_egr_iq_psi_eop_info += aid2EeSieD.si_egr_iq_psi_eop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_D_REGSET, (void *)&aid2EeSieD);
    }
    if (eeAid2VbusClkOrigin.aid2_ee_sie_e_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SIE_E_STS, (void *)&aid2EeSieE);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieECnt.si_egr_ctl_psi_dat_info += aid2EeSieE.si_egr_ctl_psi_dat_info;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieECnt.si_egr_ctl_psi_data_type_err += aid2EeSieE.si_egr_ctl_psi_data_type_err;
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieECnt.si_egr_ctl_psi_eop_info += aid2EeSieE.si_egr_ctl_psi_eop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_E_REGSET, (void *)&aid2EeSieE);
    }
    if (eeAid2VbusClkOrigin.aid2_ee_sie_f_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SIE_F_STS, (void *)&aid2EeSieF);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieFCnt.si_egr_iq_psi_sop_info += aid2EeSieF.si_egr_iq_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_F_REGSET, (void *)&aid2EeSieF);
    }
    if (eeAid2VbusClkOrigin.aid2_ee_sie_g_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_SIE_G_STS, (void *)&aid2EeSieG);
        hIqn2->iqn2EeCount.eeAid2.aid2EeSieGCnt.si_egr_ctl_psi_sop_info += aid2EeSieG.si_egr_ctl_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_SIE_G_REGSET, (void *)&aid2EeSieG);
    }
}

/* Get IQN2 DFE Errors and Alarms status and clear */
void IQN2_getDfeException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
    Iqn2Fl_Aid2EeDfe    aid2EeDfe;
    Iqn2Fl_EeArgIndex   eventNum;

    if (interruptEventNum == 1) {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV1;
    } else {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV0;
    }
    hIqn2->hFl->arg_ee = eventNum;
    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_EE_DFE_STS, (void *)&aid2EeDfe);
    hIqn2->iqn2EeCount.eeAid2.aid2EeDfeCnt.dfe_iqn_aid2_err += aid2EeDfe.dfe_iqn_aid2_err; if (aid2EeDfe.dfe_iqn_aid2_err) hIqn2->iqn2EeCount.eeFlag = 1;
    hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EE_DFE_REGSET, (void *)&aid2EeDfe);
}

/* Get IQN2 DIO2 Errors and Alarms status and clear */
void IQN2_getDio2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
)
{
    Iqn2Fl_Dio2EeDmaIngA    dio2EeDmaIngA;
    Iqn2Fl_Dio2EeDmaIngB    dio2EeDmaIngB;
    Iqn2Fl_Dio2EeDmaEgrA    dio2EeDmaEgrA;
    Iqn2Fl_Dio2EeDtA        dio2EeDtA;
    Iqn2Fl_Dio2EeSiiA       dio2EeSiiA;
    Iqn2Fl_Dio2EeSiiC       dio2EeSiiC;
    Iqn2Fl_Dio2EeSiiE       dio2EeSiiE;
    Iqn2Fl_Dio2EeSiiG       dio2EeSiiG;
    Iqn2Fl_Dio2EeSieA       dio2EeSieA;
    Iqn2Fl_Dio2EeSieD       dio2EeSieD;
    Iqn2Fl_Dio2EeSieF       dio2EeSieF;
    Iqn2Fl_Dio2EeOrigin     eeDio2Origin;
    Iqn2Fl_EeArgIndex       eventNum;

    if (interruptEventNum == 1) {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV1;
    } else {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV0;
    }

    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_ORGN_STS, (void *)&eeDio2Origin);
    if (eeDio2Origin.dio2_ee_dma_ing_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_DMA_ING_A_STS, (void *)&dio2EeDmaIngA);
        hIqn2->iqn2EeCount.eeDio2.dio2EeDmaIngACnt.dma_ing_pend_que_ovf_err += dio2EeDmaIngA.dma_ing_pend_que_ovf_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeDmaIngACnt.dma_ing_prog_err += dio2EeDmaIngA.dma_ing_prog_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeDmaIngACnt.dma_ing_xfer_done_info += dio2EeDmaIngA.dma_ing_xfer_done_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_A_REGSET, (void *)&dio2EeDmaIngA);
    }
    if (eeDio2Origin.dio2_ee_dma_ing_b_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_DMA_ING_B_STS, (void *)&dio2EeDmaIngB);
        hIqn2->iqn2EeCount.eeDio2.dio2EeDmaIngBCnt.dma_ing_buf_ovf_err += dio2EeDmaIngB.dma_ing_buf_ovf_err;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_ING_B_REGSET, (void *)&dio2EeDmaIngB);
    }
    if (eeDio2Origin.dio2_ee_dma_egr_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_DMA_EGR_A_STS, (void *)&dio2EeDmaEgrA);
        hIqn2->iqn2EeCount.eeDio2.dio2EeDmaEgrACnt.dma_egr_pend_que_ovf_err += dio2EeDmaEgrA.dma_egr_pend_que_ovf_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeDmaEgrACnt.dma_egr_prog_err += dio2EeDmaEgrA.dma_egr_prog_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeDmaEgrACnt.dma_egr_xfer_done_info += dio2EeDmaEgrA.dma_egr_xfer_done_info;
        hIqn2->iqn2EeCount.eeDio2.dio2EeDmaEgrACnt.si_ing_iq_fifo_full_info += dio2EeDmaEgrA.si_ing_iq_fifo_full_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DMA_EGR_A_REGSET, (void *)&dio2EeDmaEgrA);
    }
    if (eeDio2Origin.dio2_ee_dt_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_DT_A_STS, (void *)&dio2EeDtA);
        hIqn2->iqn2EeCount.eeDio2.dio2EeDtACnt.dt_buff_ovf_err += dio2EeDtA.dt_buff_ovf_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeDtACnt.dt_done_info += dio2EeDtA.dt_done_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_DT_A_REGSET, (void *)&dio2EeDtA);
    }
    if (eeDio2Origin.dio2_ee_sii_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_SII_A_STS, (void *)&dio2EeSiiA);
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiACnt.si_ing_iq_icc_sof_info += dio2EeSiiA.si_ing_iq_icc_sof_info;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiACnt.si_ing_iq_icc_dat_info += dio2EeSiiA.si_ing_iq_icc_dat_info;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiACnt.si_ing_iq_ife_sop_info += dio2EeSiiA.si_ing_iq_ife_sop_info;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiACnt.si_ing_iq_ife_eop_info += dio2EeSiiA.si_ing_iq_ife_eop_info;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiACnt.si_ing_iq_ife_dat_info += dio2EeSiiA.si_ing_iq_ife_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_A_REGSET, (void *)&dio2EeSiiA);

    }
    if (eeDio2Origin.dio2_ee_sii_c_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_SII_C_STS, (void *)&dio2EeSiiC);
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiCCnt.si_ing_iq_sof_err = dio2EeSiiC.si_ing_iq_sof_err; // this is one bit per-channel
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_C_REGSET, (void *)&dio2EeSiiC);

    }
    if (eeDio2Origin.dio2_ee_sii_e_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_SII_E_STS, (void *)&dio2EeSiiE);
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiECnt.si_ing_iq_psi_eop_info += dio2EeSiiE.si_ing_iq_psi_eop_info;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiECnt.si_ing_iq_psi_dat_info += dio2EeSiiE.si_ing_iq_psi_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_E_REGSET, (void *)&dio2EeSiiE);
    }
    if (eeDio2Origin.dio2_ee_sii_g_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_SII_G_STS, (void *)&dio2EeSiiG);
        hIqn2->iqn2EeCount.eeDio2.dio2EeSiiGCnt.si_ing_iq_psi_sop_info += dio2EeSiiG.si_ing_iq_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SII_G_REGSET, (void *)&dio2EeSiiG);
    }
    if (eeDio2Origin.dio2_ee_sie_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_SIE_A_STS, (void *)&dio2EeSieA);
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieACnt.si_egr_iq_efe_starve_err += dio2EeSieA.si_egr_iq_efe_starve_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieACnt.si_egr_iq_efe_pkt_err += dio2EeSieA.si_egr_iq_efe_pkt_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieACnt.si_egr_iq_efe_sym_err += dio2EeSieA.si_egr_iq_efe_sym_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieACnt.si_egr_iq_icc_sof_info += dio2EeSieA.si_egr_iq_icc_sof_info;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieACnt.si_egr_iq_icc_dat_info += dio2EeSieA.si_egr_iq_icc_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_A_REGSET, (void *)&dio2EeSieA);
    }
    if (eeDio2Origin.dio2_ee_sie_d_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_SIE_D_STS, (void *)&dio2EeSieD);
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieDCnt.si_egr_iq_psi_data_type_err += dio2EeSieD.si_egr_iq_psi_data_type_err;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieDCnt.si_egr_iq_psi_eop_info += dio2EeSieD.si_egr_iq_psi_eop_info;
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieDCnt.si_egr_iq_psi_dat_info += dio2EeSieD.si_egr_iq_psi_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_D_REGSET, (void *)&dio2EeSieD);
    }
    if (eeDio2Origin.dio2_ee_sie_f_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_EE_SIE_F_STS, (void *)&dio2EeSieF);
        hIqn2->iqn2EeCount.eeDio2.dio2EeSieFCnt.si_egr_iq_psi_sop_info += dio2EeSieF.si_egr_iq_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EE_SIE_F_REGSET, (void *)&dio2EeSieF);
    }
}

/* Get IQN2 AIL Errors and Alarms status and clear */
void IQN2_getAilException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum,
        Iqn2Fl_AilInstance ailNum
)
{
    Iqn2Fl_AilEeSiiA    ailEeSiiA;
    Iqn2Fl_AilEeSiiB    ailEeSiiB;
    Iqn2Fl_AilEeSiiC0   ailEeSiiC0;
    Iqn2Fl_AilEeSiiC1   ailEeSiiC1;
    Iqn2Fl_AilEeSiiD    ailEeSiiD;
    Iqn2Fl_AilEeSiiE    ailEeSiiE;
    Iqn2Fl_AilEeSiiF    ailEeSiiF;
    Iqn2Fl_AilEeSiiG0   ailEeSiiG0;
    Iqn2Fl_AilEeSiiG1   ailEeSiiG1;
    Iqn2Fl_AilEeSiiH    ailEeSiiH;
    Iqn2Fl_AilEeSieA    ailEeSieA;
    Iqn2Fl_AilEeSieB    ailEeSieB;
    Iqn2Fl_AilEeSieC    ailEeSieC;
    Iqn2Fl_AilEeSieD    ailEeSieD;
    Iqn2Fl_AilEeSieE    ailEeSieE;
    Iqn2Fl_AilEeSieF0   ailEeSieF0;
    Iqn2Fl_AilEeSieF1   ailEeSieF1;
    Iqn2Fl_AilEeSieG    ailEeSieG;
    Iqn2Fl_AilEeRm0     ailEeRm0;
    Iqn2Fl_AilEeRtTm0   ailEeRtTm0;
    Iqn2Fl_AilEeCiCo0   ailEeCiCo0;
    Iqn2Fl_AilEePd0     ailEePd0;
    Iqn2Fl_AilEePd1     ailEePd1;
    Iqn2Fl_AilEePe0     ailEePe0;
    Iqn2Fl_AilEeSi0     ailEeSi0;
    Iqn2Fl_AilVbusClkEeOrigin   eeAilVbusClkOrigin;
    Iqn2Fl_AilSysClkPhyEeOrigin eeAilSysClkPhyOrigin;
    Iqn2Fl_AilSysClkEeOrigin    eeAilSysClkOrigin;
    Iqn2Fl_EeArgIndex       eventNum;

    if (interruptEventNum == 1) {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV1;
    } else {
        eventNum = IQN2FL_EE_INT_EN_STATUS_EV0;
    }

    hIqn2->hFl->arg_ail = ailNum;

    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_SYSCLK_EE_ORGN_STS, (void *)&eeAilSysClkOrigin);
    if (eeAilSysClkOrigin.ail_ee_sii_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_A_STS, (void *)&ailEeSiiA);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiACnt.si_ing_iq_icc_sof_info += ailEeSiiA.si_ing_iq_icc_sof_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiACnt.si_ing_iq_icc_dat_info += ailEeSiiA.si_ing_iq_icc_dat_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiACnt.si_ing_iq_ife_sop_info += ailEeSiiA.si_ing_iq_ife_sop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiACnt.si_ing_iq_ife_eop_info += ailEeSiiA.si_ing_iq_ife_eop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiACnt.si_ing_iq_ife_dat_info += ailEeSiiA.si_ing_iq_ife_dat_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiACnt.si_ing_iq_fifo_ovfl_err += ailEeSiiA.si_ing_iq_fifo_ovfl_err;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_A_REGSET, (void *)&ailEeSiiA);
    }
    if (eeAilSysClkOrigin.ail_ee_sii_b_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_B_STS, (void *)&ailEeSiiB);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiBCnt.si_ing_ctl_pkt_err += ailEeSiiB.si_ing_ctl_pkt_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiBCnt.si_ing_ctl_icc_eop_info += ailEeSiiB.si_ing_ctl_icc_eop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiBCnt.si_ing_ctl_icc_dat_info += ailEeSiiB.si_ing_ctl_icc_dat_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiBCnt.si_ing_ctl_fifo_ovfl_err += ailEeSiiB.si_ing_ctl_fifo_ovfl_err;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_B_REGSET, (void *)&ailEeSiiB);
    }
    if (eeAilSysClkOrigin.ail_ee_sii_c_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_C_0_STS, (void *)&ailEeSiiC0);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiC0Cnt.si_ing_iq_sof_err = ailEeSiiC0.si_ing_iq_sof_err; // one bit per channel
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_0_REGSET, (void *)&ailEeSiiC0);
    }
    if (eeAilSysClkOrigin.ail_ee_sii_c_1_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_C_1_STS, (void *)&ailEeSiiC1);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiC1Cnt.si_ing_iq_sof_err_64_32 = ailEeSiiC1.si_ing_iq_sof_err_64_32;  // one bit per channel
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_C_1_REGSET, (void *)&ailEeSiiC1);
    }
    if (eeAilSysClkOrigin.ail_ee_sii_d_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_D_STS, (void *)&ailEeSiiD);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiDCnt.si_ing_ctl_icc_sop_info += ailEeSiiD.si_ing_ctl_icc_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_D_REGSET, (void *)&ailEeSiiD);
    }
    if (eeAilSysClkOrigin.ail_ee_sie_a_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SIE_A_STS, (void *)&ailEeSieA);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieACnt.si_egr_iq_efe_starve_err += ailEeSieA.si_egr_iq_efe_starve_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieACnt.si_egr_iq_efe_pkt_err += ailEeSieA.si_egr_iq_efe_pkt_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieACnt.si_egr_iq_efe_sym_err += ailEeSieA.si_egr_iq_efe_sym_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieACnt.si_egr_iq_icc_sof_info += ailEeSieA.si_egr_iq_icc_sof_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieACnt.si_egr_iq_icc_dat_info += ailEeSieA.si_egr_iq_icc_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_A_REGSET, (void *)&ailEeSieA);
    }
    if (eeAilSysClkOrigin.ail_ee_sie_b_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SIE_B_STS, (void *)&ailEeSieB);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieBCnt.si_egr_ctl_icc_eop_info += ailEeSieB.si_egr_ctl_icc_eop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieBCnt.si_egr_ctl_icc_dat_info += ailEeSieB.si_egr_ctl_icc_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_B_REGSET, (void *)&ailEeSieB);
    }
    if (eeAilSysClkOrigin.ail_ee_sie_c_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SIE_C_STS, (void *)&ailEeSieC);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieCCnt.si_egr_ctl_icc_sop_info += ailEeSieC.si_egr_ctl_icc_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_C_REGSET, (void *)&ailEeSieC);
    }
    if (eeAilSysClkOrigin.ail_ee_pd_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_PD_0_STS, (void *)&ailEePd0);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_frm_win_err += ailEePd0.pd_ee_obsai_frm_win_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_sop_err += ailEePd0.pd_ee_sop_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_ts_miss_err += ailEePd0.pd_ee_obsai_ts_miss_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_ts_wdog_err += ailEePd0.pd_ee_obsai_ts_wdog_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_axc_fail_err += ailEePd0.pd_ee_obsai_axc_fail_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_crc_err += ailEePd0.pd_ee_obsai_crc_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_rp3_01_soc_rst_info += ailEePd0.pd_ee_rp3_01_soc_rst_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_route_fail_info += ailEePd0.pd_ee_obsai_route_fail_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_rp3_01_capture_info += ailEePd0.pd_ee_rp3_01_capture_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_rp3_01_crc_fail_err += ailEePd0.pd_ee_rp3_01_crc_fail_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_gsm_off_stb_info += ailEePd0.pd_ee_obsai_gsm_off_stb_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_cpri_cw_crc_err += ailEePd0.pd_ee_cpri_cw_crc_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_cpri_cw_ovfl_err += ailEePd0.pd_ee_cpri_cw_ovfl_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_cpri_cw_4b5b_eop_err += ailEePd0.pd_ee_cpri_cw_4b5b_eop_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_cpri_cw_4b5b_char_err += ailEePd0.pd_ee_cpri_cw_4b5b_char_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_sop_info += ailEePd0.pd_ee_obsai_sop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_eop_info += ailEePd0.pd_ee_obsai_eop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd0Cnt.pd_ee_obsai_sof_info += ailEePd0.pd_ee_obsai_sof_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_0_REGSET, (void *)&ailEePd0);
    }
    if (eeAilSysClkOrigin.ail_ee_pd_1_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_PD_1_STS, (void *)&ailEePd1);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd1Cnt.pd_ee_cpri_bub_fsm_err += ailEePd1.pd_ee_cpri_bub_fsm_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd1Cnt.pd_ee_cpri_tdm_fsm_err += ailEePd1.pd_ee_cpri_tdm_fsm_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePd1Cnt.pd_ee_cpri_radstd_err += ailEePd1.pd_ee_cpri_radstd_err;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PD_1_REGSET, (void *)&ailEePd1);
    }
    if (eeAilSysClkOrigin.ail_ee_pe_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_PE_0_STS, (void *)&ailEePe0);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePe0Cnt.pe_ee_cpri_cw_null_starve_err += ailEePe0.pe_ee_cpri_cw_null_starve_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePe0Cnt.pe_ee_cpri_cw_4b5b_starve_err += ailEePe0.pe_ee_cpri_cw_4b5b_starve_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePe0Cnt.pe_ee_cpri_cw_hypfm_starve_err += ailEePe0.pe_ee_cpri_cw_hypfm_starve_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePe0Cnt.pe_ee_cpri_cw_hdlc_starve_err += ailEePe0.pe_ee_cpri_cw_hdlc_starve_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePe0Cnt.pe_ee_cpri_cw_hypfm_oflow_err += ailEePe0.pe_ee_cpri_cw_hypfm_oflow_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEePe0Cnt.pe_ee_ofifo_oflow_err += ailEePe0.pe_ee_ofifo_oflow_err;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_PE_0_REGSET, (void *)&ailEePe0);
    }
    if (eeAilSysClkOrigin.ail_ee_si_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SI_0_STS, (void *)&ailEeSi0);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSi0Cnt.uat_pi_err       += ailEeSi0.uat_pi_err;
        if (ailEeSi0.uat_pi_err)
            hIqn2->iqn2EeCount.eeAil[ailNum].ailPiCaptured = \
            CSL_FEXT(hIqn2->hFl->regs->Ail[ailNum].AIL_UAT_AIL_REGS.AIL_UAT_PI_BCN_CAPTURE_STS,IQN_AIL_AIL_UAT_PI_BCN_CAPTURE_STS_RD_VAL);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSi0Cnt.cpri_tdm_lut_err += ailEeSi0.cpri_tdm_lut_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSi0Cnt.cpri_bub_fsm_err += ailEeSi0.cpri_bub_fsm_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSi0Cnt.obsai_phy_sync_err += ailEeSi0.obsai_phy_sync_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSi0Cnt.obsai_multrulefire_err += ailEeSi0.obsai_multrulefire_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSi0Cnt.obsai_dbm_wrap_err += ailEeSi0.obsai_dbm_wrap_err;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SI_0_REGSET, (void *)&ailEeSi0);
    }

    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_VBUSCLK_EE_ORGN_STS, (void *)&eeAilVbusClkOrigin);
    if (eeAilVbusClkOrigin.ail_ee_sii_e_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_E_STS, (void *)&ailEeSiiE);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiECnt.si_ing_iq_psi_eop_info += ailEeSiiE.si_ing_iq_psi_eop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiECnt.si_ing_iq_psi_dat_info += ailEeSiiE.si_ing_iq_psi_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_E_REGSET, (void *)&ailEeSiiE);
    }
    if (eeAilVbusClkOrigin.ail_ee_sii_f_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_F_STS, (void *)&ailEeSiiF);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiFCnt.si_ing_ctl_psi_eop_info += ailEeSiiF.si_ing_ctl_psi_eop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiFCnt.si_ing_ctl_psi_dat_info += ailEeSiiF.si_ing_ctl_psi_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_F_REGSET, (void *)&ailEeSiiF);
    }
    if (eeAilVbusClkOrigin.ail_ee_sii_g_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_G_0_STS, (void *)&ailEeSiiG0);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiG0Cnt.si_ing_iq_psi_sop_info += ailEeSiiG0.si_ing_iq_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_0_REGSET, (void *)&ailEeSiiG0);
    }
    if (eeAilVbusClkOrigin.ail_ee_sii_g_1_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_G_1_STS, (void *)&ailEeSiiG1);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiG1Cnt.si_ing_iq_psi_sop_info_64_32 += ailEeSiiG1.si_ing_iq_psi_sop_info_64_32;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_G_1_REGSET, (void *)&ailEeSiiG1);
    }
    if (eeAilVbusClkOrigin.ail_ee_sii_h_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SII_H_STS, (void *)&ailEeSiiH);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSiiHCnt.si_ing_ctl_psi_sop_info += ailEeSiiH.si_ing_ctl_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SII_H_REGSET, (void *)&ailEeSiiH);
    }
    if (eeAilVbusClkOrigin.ail_ee_sie_d_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SIE_D_STS, (void *)&ailEeSieD);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieDCnt.si_egr_iq_psi_data_type_err += ailEeSieD.si_egr_iq_psi_data_type_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieDCnt.si_egr_iq_psi_eop_info += ailEeSieD.si_egr_iq_psi_eop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieDCnt.si_egr_iq_psi_dat_info += ailEeSieD.si_egr_iq_psi_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_D_REGSET, (void *)&ailEeSieD);
    }
    if (eeAilVbusClkOrigin.ail_ee_sie_e_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SIE_E_STS, (void *)&ailEeSieE);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieECnt.si_egr_ctl_psi_data_type_err += ailEeSieE.si_egr_ctl_psi_data_type_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieECnt.si_egr_ctl_psi_eop_info += ailEeSieE.si_egr_ctl_psi_eop_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieECnt.si_egr_ctl_psi_dat_info += ailEeSieE.si_egr_ctl_psi_dat_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_E_REGSET, (void *)&ailEeSieE);
    }
    if (eeAilVbusClkOrigin.ail_ee_sie_f_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SIE_F_0_STS, (void *)&ailEeSieF0);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieF0Cnt.si_egr_iq_psi_sop_info += ailEeSieF0.si_egr_iq_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_0_REGSET, (void *)&ailEeSieF0);
    }
    if (eeAilVbusClkOrigin.ail_ee_sie_f_1_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SIE_F_1_STS, (void *)&ailEeSieF1);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieF1Cnt.si_egr_iq_psi_sop_info_64_32 += ailEeSieF1.si_egr_iq_psi_sop_info_64_32;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_F_1_REGSET, (void *)&ailEeSieF1);
    }
    if (eeAilVbusClkOrigin.ail_ee_sie_g_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_SIE_G_STS, (void *)&ailEeSieG);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeSieGCnt.si_egr_ctl_psi_sop_info += ailEeSieG.si_egr_ctl_psi_sop_info;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_SIE_G_REGSET, (void *)&ailEeSieG);
    }

    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_SYSCLK_PHY_EE_ORGN_STS, (void *)&eeAilSysClkPhyOrigin);
    if (eeAilSysClkPhyOrigin.ail_ee_rm_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_RM_0_STS, (void *)&ailEeRm0);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.sync_status_change += ailEeRm0.sync_status_change;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeHfnsync.syncStatusChange = ailEeRm0.sync_status_change;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rm_status_state0 += ailEeRm0.rm_status_state0;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rm_status_state1 += ailEeRm0.rm_status_state1;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rm_status_state2 += ailEeRm0.rm_status_state2;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rm_status_state3 += ailEeRm0.rm_status_state3;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rm_status_state4 += ailEeRm0.rm_status_state4;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rm_status_state5 += ailEeRm0.rm_status_state5;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.num_los_det += ailEeRm0.num_los_det;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.lcv_det += ailEeRm0.lcv_det;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.frame_bndry_det += ailEeRm0.frame_bndry_det;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.block_bndry_det += ailEeRm0.block_bndry_det;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.missing_k28p5 += ailEeRm0.missing_k28p5;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.loc_det += ailEeRm0.loc_det;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rx_fifo_ovf += ailEeRm0.rx_fifo_ovf;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.los_err += ailEeRm0.los_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.lof_err += ailEeRm0.lof_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.lof_state += ailEeRm0.lof_state;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rm_rst += ailEeRm0.rm_rst;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.missing_k28p7 += ailEeRm0.missing_k28p7;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.k30p7_det += ailEeRm0.k30p7_det;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rcvd_los += ailEeRm0.rcvd_los;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rcvd_lof += ailEeRm0.rcvd_lof;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rcvd_rai += ailEeRm0.rcvd_rai;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rcvd_sdi += ailEeRm0.rcvd_sdi;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.rcvd_rst += ailEeRm0.rcvd_rst;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRm0Cnt.hfnsync_state += ailEeRm0.hfnsync_state;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeHfnsync.hfnsyncState = ailEeRm0.hfnsync_state;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RM_0_REGSET, (void *)&ailEeRm0);
    }
    if (eeAilSysClkPhyOrigin.ail_ee_rt_tm_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_RT_TM_0_STS, (void *)&ailEeRtTm0);
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.rt_hdr_error += ailEeRtTm0.rt_hdr_error;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.rt_em_insert += ailEeRtTm0.rt_em_insert;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.rt_unfl += ailEeRtTm0.rt_unfl;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.rt_ovfl += ailEeRtTm0.rt_ovfl;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.rt_frm_err += ailEeRtTm0.rt_frm_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.rt_unalign_err += ailEeRtTm0.rt_unalign_err;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.rt_aggr_state_info += ailEeRtTm0.rt_aggr_state_info;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.tm_frame_sync_state += ailEeRtTm0.tm_frame_sync_state;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.delta_inactive += ailEeRtTm0.delta_inactive;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.delta_modified += ailEeRtTm0.delta_modified;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.frame_misalign += ailEeRtTm0.frame_misalign;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.fifo_undeflow += ailEeRtTm0.fifo_undeflow;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeRtTm0Cnt.tm_fail += ailEeRtTm0.tm_fail;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_RT_TM_0_REGSET, (void *)&ailEeRtTm0);
    }
    if (eeAilSysClkPhyOrigin.ail_ee_ci_co_0_sts) {
        hIqn2->iqn2EeCount.eeFlag = 1;
        hIqn2->hFl->arg_ee = eventNum;
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_EE_CI_CO_0_STS, (void *)&ailEeCiCo0);
        ailEeCiCo0.ci_tbltoolong = TRUE;
                      ailEeCiCo0.co_tbltoolong = TRUE;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeCiCo0Cnt.ci_tbltoolong += ailEeCiCo0.ci_tbltoolong;
        hIqn2->iqn2EeCount.eeAil[ailNum].ailEeCiCo0Cnt.co_tbltoolong += ailEeCiCo0.co_tbltoolong;
        hIqn2->hFl->arg_ee = IQN2FL_EE_INT_CLR;
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EE_CI_CO_0_REGSET, (void *)&ailEeCiCo0);
    }

}


/* Get IQN2 Errors and Alarms status and clear */
void IQN2_getException(
		IQN2_ConfigHandle  hIqn2
)
{
Iqn2Fl_EeOrigin             eeOrigin;

	Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_EE_EV0_ORGN_STS, (void *)&eeOrigin);

	/* PSR errors */
    if (eeOrigin.psr_ee_sts) {
        IQN2_getPsrException(hIqn2,0);
    }
    /* PKTDMA errors */
    IQN2_getPktDMAException(hIqn2);
    /* AT2 errors */
    if (eeOrigin.at_ee_sts) {
        IQN2_getAt2Exception(hIqn2,0);
    }
    /* IQS2 errors */
    if (eeOrigin.iqs_ee_sts) {
        IQN2_getIqs2Exception(hIqn2,0);
    }
    /* AID2 errors */
    if (eeOrigin.aid_ee_sts) {
        IQN2_getAid2Exception(hIqn2,0);
    }
    /* DFE errors */
    if (eeOrigin.dfe_ee_sts) {
        IQN2_getDfeException(hIqn2,1);
    }
    /* DIO2 errors */
    if (eeOrigin.dio_ee_sts) {
        IQN2_getDio2Exception(hIqn2,0);
    }
    /* AIL0 errors */
    if (eeOrigin.ail0_ee_sts) {
        IQN2_getAilException(hIqn2,0,(Iqn2Fl_AilInstance)0);
    }
    /* AIL1 errors */
    if (eeOrigin.ail1_ee_sts) {
        IQN2_getAilException(hIqn2,0,(Iqn2Fl_AilInstance)1);
    }
}

/* Print IQN2 Errors and Alarms status and clear */
void IQN2_printException(
		IQN2_ConfigHandle  hIqn2
)
{
#ifdef _TMS320C6X
   uint32_t gie;
   gie = _disable_interrupts();
#endif


	if (hIqn2->iqn2EeCount.eeFlag == 1) Iqn2_osalLog("\n######### IQN2 ERRORS ###########\n");
/*	if (eeDbIntCnt.db_ee_i_trc_ram_ovfl_err)
		Iqn2_osalLog("DB:%d Data Trace RAM overflowed. This is not a fatal error because it only affects the Data Trace RAM\n",eeDbIntCnt.db_ee_i_trc_ram_ovfl_err);
*/
	if (hIqn2->iqn2EeCount.eeFlag == 1) Iqn2_osalLog("\n###############################\n");

#ifdef _TMS320C6X
	 _restore_interrupts(gie);
#endif
}

void IQN2_printStatus(
		IQN2_ConfigHandle  hIqn2
)
{
	IQN2_getException(hIqn2);
	IQN2_printException(hIqn2);
}

void IQN2_enableAilDataTrace(
        IQN2_ConfigHandle        hIqn2,
        IQN2_AilDataTraceHandle  hDataTrace
)
{
	if ((hIqn2) && (hDataTrace))
	{
		// To be supported in future releases
	}
}

void IQN2_disableAilDataTrace(
        IQN2_ConfigHandle    hIqn2
)
{
	if (hIqn2)
	{
		// To be supported in future releases
	}
}



////////////////////

