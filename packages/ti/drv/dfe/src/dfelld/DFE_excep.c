/********************************************************************
 * Copyright (C) 2013 Texas Instruments Incorporated.
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
/** @file DFE_excep.c
 *
 *  @path  $(DRVPATH)\dfe\src\dfelld
 *
 *  @brief File for LLD layer APIs of exception counting
 *
 *
 */
/* =============================================================================
 * Revision History
 * ===============
 *
 *
 * =============================================================================
 */

#include <math.h>
#include <string.h>
#include <ti/drv/dfe/dfe_drv.h>
#include <ti/drv/dfe/dfe_osal.h>
#include <ti/drv/dfe/dfe_internal.h>

#ifdef _TMS320C6X
extern uint32_t _disable_interrupts(void);
extern void _restore_interrupts(uint32_t key);
#endif

/* Reset exception stats */
static void reset_ExceptionStats(
        DFE_Handle  hDfe
)
{
    memset(&hDfe->dfeEeCount, (Uint8)0x00, sizeof(DFE_EeCountObj));
}

/** ============================================================================
 *   @n@b Dfe_resetException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
void Dfe_resetException(
        DFE_Handle  hDfe
)
{
    reset_ExceptionStats(hDfe);
}

/** ============================================================================
 *   @n@b Dfe_captureException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
void Dfe_captureException (
        DFE_Handle  hDfe,
        DFE_EeCountObj *capturePtr
)
{
    memcpy(capturePtr, &hDfe->dfeEeCount, sizeof(DFE_EeCountObj));
}

/** ============================================================================
 *   @n@b Dfe_enableBbException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE BB Errors and Alarms */
DFE_Err Dfe_enableBbException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_BbGeneralIntrGroup    bbErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    // BB general interrupt, TXPM_LDERR - Warning
    bbErr.txpmLoadErr = 1;
    // BB general interrupt, RXPM_LDERR - Warning
    bbErr.rxpmLoadErr = 0;
    // BB general interrupt, ANTCAL - Info
    bbErr.antcal = 0;
    // BB general interrupt, RXNOTCH_DONE - Info
    bbErr.rxNotchDone = 0;
    // BB general interrupt, RXNOTCH_ERR - Warning
    bbErr.rxNotchErr = 1;
    // BB general interrupt, BUFMEM_OUF (overflow/underflow) - Fatal
    for (i=0;i<8;i++)
    {
        bbErr.bufErr[i] = 1;
    }
    // BB general interrupt, RXAID_SYNCERR - Fatal
    bbErr.rxaidSyncErr = 1;
    // BB general interrupt, TXAID_UDF (under flow) - Fatal
    bbErr.txaidUnderflow = 1;
    // BB general interrupt, TXAID_OVF (over flow) - Fatal
    bbErr.txaidOverflow = 1;
    // BB general interrupt, JESDRX_SYNCERR - Fatal
    bbErr.jesdrxSyncErr = 1;
    // BB general interrupt, JESDTX_UDF (under flow) - Fatal
    bbErr.jesdtxUnderflow = 1;
    // BB general interrupt, JESDTX_OVF (over flow) - Fatal
    bbErr.jesdtxOverflow = 1;
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_GENERAL_INTRGRP_STATUS, &bbErr) );
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_ENB_GENERAL_INTRGRP, &bbErr) );

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BB;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableBbException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE BB Errors and Alarms */
DFE_Err Dfe_disableBbException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_BbGeneralIntrGroup    bbErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    // BB general interrupt, TXPM_LDERR - Warning
    bbErr.txpmLoadErr = 0;
    // BB general interrupt, RXPM_LDERR - Warning
    bbErr.rxpmLoadErr = 0;
    // BB general interrupt, ANTCAL - Info
    bbErr.antcal = 0;
    // BB general interrupt, RXNOTCH_DONE - Info
    bbErr.rxNotchDone = 0;
    // BB general interrupt, RXNOTCH_ERR - Warning
    bbErr.rxNotchErr = 0;
    // BB general interrupt, BUFMEM_OUF (overflow/underflow) - Fatal
    for (i=0;i<8;i++)
    {
        bbErr.bufErr[i] = 0;
    }
    // BB general interrupt, RXAID_SYNCERR - Fatal
    bbErr.rxaidSyncErr = 0;
    // BB general interrupt, TXAID_UDF (under flow) - Fatal
    bbErr.txaidUnderflow = 0;
    // BB general interrupt, TXAID_OVF (over flow) - Fatal
    bbErr.txaidOverflow = 0;
    // BB general interrupt, JESDRX_SYNCERR - Fatal
    bbErr.jesdrxSyncErr = 0;
    // BB general interrupt, JESDTX_UDF (under flow) - Fatal
    bbErr.jesdtxUnderflow = 0;
    // BB general interrupt, JESDTX_OVF (over flow) - Fatal
    bbErr.jesdtxOverflow = 0;
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_GENERAL_INTRGRP_STATUS, &bbErr) );
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_DIS_GENERAL_INTRGRP, &bbErr) );

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BB;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getBbException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE BB Errors and Alarms status and clear */
DFE_Err Dfe_getBbException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_BbGeneralIntrGroup    bbErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_GENERAL_INTRGRP_STATUS, &bbErr) );
    hDfe->dfeEeCount.eeFlag = 1;
    hDfe->dfeEeCount.bbErr.txpmLoadErr += bbErr.txpmLoadErr;
    hDfe->dfeEeCount.bbErr.rxpmLoadErr += bbErr.rxpmLoadErr;
    hDfe->dfeEeCount.bbErr.antcal += bbErr.antcal;
    hDfe->dfeEeCount.bbErr.rxNotchDone += bbErr.rxNotchDone;
    hDfe->dfeEeCount.bbErr.rxNotchErr += bbErr.rxNotchErr;
    for (i=0;i<8;i++)
    {
        hDfe->dfeEeCount.bbErr.bufErr[i] += bbErr.bufErr[i];
    }
    hDfe->dfeEeCount.bbErr.rxaidSyncErr += bbErr.rxaidSyncErr;
    hDfe->dfeEeCount.bbErr.txaidUnderflow += bbErr.txaidUnderflow;
    hDfe->dfeEeCount.bbErr.txaidOverflow += bbErr.txaidOverflow;
    hDfe->dfeEeCount.bbErr.jesdrxSyncErr  += bbErr.jesdrxSyncErr;
    hDfe->dfeEeCount.bbErr.jesdtxUnderflow += bbErr.jesdtxUnderflow;
    hDfe->dfeEeCount.bbErr.jesdtxOverflow += bbErr.jesdtxOverflow;

    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_GENERAL_INTRGRP_STATUS, &bbErr) );
    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BB;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableBbTxPmException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE BB TX PM Errors and Alarms */
DFE_Err Dfe_enableBbTxPmException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for (i=0;i<16;i++)
    {
    	// BB Tx power meter CT interrupt - INFO
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXPM_INTR_STATUS, &i) );
//		CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_ENB_TXPM_INTR, &i) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableBbTxPmException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE BB TX PM Errors and Alarms */
DFE_Err Dfe_disableBbTxPmException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for (i=0;i<16;i++)
    {
        // BB Tx power meter CT interrupt - INFO
        CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXPM_INTR_STATUS, &i) );
        CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_DIS_TXPM_INTR, &i) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getBbTxPmException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE BB Errors and Alarms status and clear */
DFE_Err Dfe_getBbTxPmException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_BbPowMtrIntrStatus bbtxpmErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for(i=0;i<16;i++)
    {
		bbtxpmErr.pmId = i;
		CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TXPM_INTR_STATUS, &(bbtxpmErr)) );
		if (bbtxpmErr.status) hDfe->dfeEeCount.eeFlag = 1;
		hDfe->dfeEeCount.bbtxpmErr[i] += bbtxpmErr.status;
		CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXPM_INTR_STATUS, &bbtxpmErr) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableBbRxPmException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE BB RX PM Errors and Alarms */
DFE_Err Dfe_enableBbRxPmException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for (i=0;i<16;i++)
    {
    	// BB Rx power meter CT interrupt - INFO
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXPM_INTR_STATUS, &i) );
//		CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_ENB_RXPM_INTR, &i) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableBbRxPmException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE BB RX PM Errors and Alarms */
DFE_Err Dfe_disableBbRxPmException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for (i=0;i<16;i++)
    {
        // BB Rx power meter CT interrupt - INFO
        CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXPM_INTR_STATUS, &i) );
        CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_DIS_RXPM_INTR, &i) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getBbRxPmException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE BB Errors and Alarms status and clear */
DFE_Err Dfe_getBbRxPmException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_BbPowMtrIntrStatus bbrxpmErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;
    for(i=0;i<16;i++)
    {
		bbrxpmErr.pmId = i;
		CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_RXPM_INTR_STATUS, &(bbrxpmErr)) );
		if (bbrxpmErr.status) hDfe->dfeEeCount.eeFlag = 1;
		hDfe->dfeEeCount.bbrxpmErr[i] += bbrxpmErr.status;
		CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXPM_INTR_STATUS, &bbrxpmErr) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableBbTxGainException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE BB gain update CT Errors and Alarms */
DFE_Err Dfe_enableBbTxGainException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for (i=0;i<16;i++)
    {
    	// BB Tx gain update CT interrupt - INFO
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXGAIN_INTR_STATUS, &i) );
//    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_ENB_TXGAIN_INTR, &i) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableBbTxGainException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE BB gain update CT Errors and Alarms */
DFE_Err Dfe_disableBbTxGainException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for (i=0;i<16;i++)
    {
        // BB Tx gain update CT interrupt - INFO
        CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXGAIN_INTR_STATUS, &i) );
        CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_DIS_TXGAIN_INTR, &i) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getBbtxGainException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE BB Errors and Alarms status and clear */
DFE_Err Dfe_getBbTxGainException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_BbTxRxGainCarrierTypeData bbtxgainErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for(i=0;i<16;i++)
    {
		bbtxgainErr.ct = i;
		CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TXGAIN_INTR_STATUS, &(bbtxgainErr)) );
		hDfe->dfeEeCount.eeFlag = 1;
		hDfe->dfeEeCount.bbtxgainErr[i] = bbtxgainErr.data;
		CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXPM_INTR_STATUS, &bbtxgainErr) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableBbRxGainException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE BB Rx gain update CT Errors and Alarms */
DFE_Err Dfe_enableBbRxGainException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for (i=0;i<16;i++)
    {
    	// BB Rx gain update CT interrupt - INFO
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXGAIN_INTR_STATUS, &i) );
//    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_ENB_RXGAIN_INTR, &i) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableBbRxGainException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE BB Rx gain update CT Errors and Alarms */
DFE_Err Dfe_disableBbRxGainException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for (i=0;i<16;i++)
    {
        // BB Rx gain update CT interrupt - INFO
        CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXGAIN_INTR_STATUS, &i) );
        CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_DIS_RXGAIN_INTR, &i) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getBbrxGainException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE BB Errors and Alarms status and clear */
DFE_Err Dfe_getBbRxGainException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_BbTxRxGainCarrierTypeData bbrxgainErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t                    i;

    for(i=0;i<16;i++)
    {
		bbrxgainErr.ct = i;
		CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_RXGAIN_INTR_STATUS, &(bbrxgainErr)) );
		hDfe->dfeEeCount.eeFlag = 1;
		hDfe->dfeEeCount.bbrxgainErr[i] = bbrxgainErr.data;
		CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXPM_INTR_STATUS, &bbrxgainErr) );
    }

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableDducException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE DDUC Errors and Alarms */
DFE_Err Dfe_enableDducException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_DducCicovIntrStatus   dducErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr[4];
    uint32_t                    i, cic_bypass;

    masterLowPrioErr[0] =  DFE_FL_MISC_MASTER_LOWPRI_DDUC0;
    masterLowPrioErr[1] =  DFE_FL_MISC_MASTER_LOWPRI_DDUC1;
    masterLowPrioErr[2] =  DFE_FL_MISC_MASTER_LOWPRI_DDUC2;
    masterLowPrioErr[3] =  DFE_FL_MISC_MASTER_LOWPRI_DDUC3;

    for (i=0;i<4;i++)
    {
        cic_bypass = CSL_FEXT(hDfe->hDfeDduc[i]->regs->config, DFE_DDUC_CONFIG_REG_CIC0_BYP);
		// DDUC cic accumulator overflow - FATAL
		dducErr.data = i;
		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_CLR_CICOV_INTR_STATUS, &(dducErr.data)) );
		if (cic_bypass != 1)
		{
		    CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_ENB_CICOV_INTR, &(dducErr.data)) );
		}

		//Asserts when hopper is completely through hopping sequence - INFO
		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_CLR_HOPROLLOVER_INTR_STATUS, NULL) );
//		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_ENB_HOPROLLOVER_INTR, NULL) );

		//Asserts when hopper is halfway through hopping sequence - INFO
		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_CLR_HOPHALFWAY_INTR_STATUS, NULL) );
//		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_ENB_HOPHALFWAY_INTR, NULL) );

		CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr[i]) );
		CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr[i]) );
    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableDducException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE DDUC Errors and Alarms */
DFE_Err Dfe_disableDducException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_DducCicovIntrStatus    dducErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr[4];
    uint32_t                    i;

    masterLowPrioErr[0] =  DFE_FL_MISC_MASTER_LOWPRI_DDUC0;
    masterLowPrioErr[1] =  DFE_FL_MISC_MASTER_LOWPRI_DDUC1;
    masterLowPrioErr[2] =  DFE_FL_MISC_MASTER_LOWPRI_DDUC2;
    masterLowPrioErr[3] =  DFE_FL_MISC_MASTER_LOWPRI_DDUC3;

    for (i=0;i<4;i++)
    {
        // DDUC cic accumulator overflow - FATAL
        dducErr.data = i;
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_CLR_CICOV_INTR_STATUS, &(dducErr.data)) );
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_DIS_CICOV_INTR, &(dducErr.data)) );

        //Asserts when hopper is completely through hopping sequence - INFO
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_CLR_HOPROLLOVER_INTR_STATUS, NULL) );
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_DIS_HOPROLLOVER_INTR, NULL) );

        //Asserts when hopper is halfway through hopping sequence - INFO
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_CLR_HOPHALFWAY_INTR_STATUS, NULL) );
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[i], DFE_FL_DDUC_CMD_DIS_HOPHALFWAY_INTR, NULL) );

        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr[i]) );
        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr[i]) );
    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getDducException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE DDUC Errors and Alarms status and clear */
DFE_Err Dfe_getDducException(
        DFE_Handle  hDfe,
        uint32_t	index
)
{
    DfeFl_Status status;
	DfeFl_DducCicovIntrStatus    dducErr;
    DfeFl_MiscMasterLowPriIntr   masterLowPrioErr;
    uint32_t                     err;


    if(index == 0)
    	masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_DDUC0;
    else if (index == 1)
    	masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_DDUC1;
    else if (index == 2)
    	masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_DDUC2;
    else if (index == 3)
    	masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_DDUC3;

    dducErr.intr = index;
	CSL_HW_QUERY( dfeFl_DducGetHwStatus(hDfe->hDfeDduc[index], DFE_FL_DDUC_QUERY_GET_CICOV_INTR_STATUS, (DfeFl_DducCicovIntrStatus *)&dducErr) );
	if (dducErr.data) hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.dducErr[index].cicovErr += dducErr.data;
	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[index], DFE_FL_DDUC_CMD_CLR_CICOV_INTR_STATUS, &dducErr) );

	CSL_HW_QUERY( dfeFl_DducGetHwStatus(hDfe->hDfeDduc[index], DFE_FL_DDUC_QUERY_GET_HOPROLLOVER_INTR_STATUS, &err) );
    if (err) hDfe->dfeEeCount.eeFlag = 1;
    hDfe->dfeEeCount.dducErr[index].hopRolloverErr += err;
	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[index], DFE_FL_DDUC_CMD_CLR_HOPROLLOVER_INTR_STATUS, NULL) );

	CSL_HW_QUERY( dfeFl_DducGetHwStatus(hDfe->hDfeDduc[index], DFE_FL_DDUC_QUERY_GET_HOPHALFWAY_INTR_STATUS, &err) );
	if (err) hDfe->dfeEeCount.eeFlag = 1;
    hDfe->dfeEeCount.dducErr[index].hopHalfwayErr += err;
	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[index], DFE_FL_DDUC_CMD_CLR_HOPHALFWAY_INTR_STATUS, NULL) );

	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableCfrException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Cfr Errors and Alarms */
DFE_Err Dfe_enableCfrException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_CfrCfrIntrStatus   cfrErr[2];
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr[2];
    uint32_t                    i;

    masterLowPrioErr[0] =  DFE_FL_MISC_MASTER_LOWPRI_CFR0;
    masterLowPrioErr[1] =  DFE_FL_MISC_MASTER_LOWPRI_CFR1;

    for (i=0;i<2;i++)
    {

		// CFR PDC A0S0 interrupt - INFO
		cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A0S0;
		cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A0S0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );

		// CFR PDC A0S1 interrupt - INFO
		cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A0S1;
		cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A0S1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );

		// CFR PDC A1S0 interrupt - INFO
		cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A1S0;
		cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A1S0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );

		// CFR PDC A1S1 interrupt - INFO
		cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A1S1;
		cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A1S1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );

		// CFR PDC DFE_FL_CFR_LUTS_UPDT_DONE interrupt - INFO // These are actually status bits and should not be handled by exception handler
		//cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_LUTS_UPDT_DONE;
		//cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_LUTS_UPDT_DONE;
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
		//CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );


		/// AGC inout interrupt min in threshold antenna 0 - FATAL
		cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_IN_TH;
		cfrErr[i].cfrAgcInOut.intrcfg.path = 0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

		/// AGC inout interrupt min out threshold antenna 0 - FATAL
		cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_OUT_TH;
		cfrErr[i].cfrAgcInOut.intrcfg.path = 0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

		/// AGC inout interrupt max in threshold antenna 0 - FATAL
		cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_IN_TH;
		cfrErr[i].cfrAgcInOut.intrcfg.path = 0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );

		/// AGC inout interrupt max out threshold antenna 0 - FATAL
		cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_OUT_TH;
		cfrErr[i].cfrAgcInOut.intrcfg.path = 0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

		/// AGC inout interrupt min in threshold antenna 1 - FATAL
		cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_IN_TH;
		cfrErr[i].cfrAgcInOut.intrcfg.path = 1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

		/// AGC inout interrupt min out threshold antenna 1 - FATAL
		cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_OUT_TH;
		cfrErr[i].cfrAgcInOut.intrcfg.path = 1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

		/// AGC inout interrupt max in threshold antenna 1 - FATAL
		cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_IN_TH;
		cfrErr[i].cfrAgcInOut.intrcfg.path = 1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

		/// AGC inout interrupt max out threshold antenna 1 - FATAL
		cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_OUT_TH;
		cfrErr[i].cfrAgcInOut.intrcfg.path = 1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

		/// AGC sync0 interrupt antenna 0 - INFO
		cfrErr[i].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC0;
		cfrErr[i].cfrAgcSync.intrcfg.path = 0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[i].cfrAgcSync.intrcfg)) );
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_SYNC_INTR, &(cfrErr[i].cfrAgcSync.intrcfg)) );

		/// AGC sync1 interrupt antenna 0 - INFO
		cfrErr[i].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC1;
		cfrErr[i].cfrAgcSync.intrcfg.path = 0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[i].cfrAgcSync.intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_SYNC_INTR, &(cfrErr[i].cfrAgcSync.intrcfg)) );

		/// AGC sync0 interrupt antenna 1 - INFO
		cfrErr[i].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC0;
		cfrErr[i].cfrAgcSync.intrcfg.path = 1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[i].cfrAgcSync.intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_SYNC_INTR, &(cfrErr[i].cfrAgcSync.intrcfg)) );

		/// AGC sync1 interrupt antenna 1 - INFO
		cfrErr[i].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC1;
		cfrErr[i].cfrAgcSync.intrcfg.path = 1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[i].cfrAgcSync.intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_AGC_SYNC_INTR, &(cfrErr[i].cfrAgcSync.intrcfg)) );

		/// DTH power change interrupt antenna 0 - INFO
		cfrErr[i].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_PWR_CNG;
		cfrErr[i].cfrDth.intrcfg.path = 0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[i].cfrDth.intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_DTH_INTR, &(cfrErr[i].cfrDth.intrcfg)) );

		/// DTH sync0 interrupt antenna 0 - INFO
		cfrErr[i].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_SYNC0;
		cfrErr[i].cfrDth.intrcfg.path = 0;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[i].cfrDth.intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_DTH_INTR, &(cfrErr[i].cfrDth.intrcfg)) );

		/// DTH power change interrupt antenna 1 - INFO
		cfrErr[i].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_PWR_CNG;
		cfrErr[i].cfrDth.intrcfg.path = 1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[i].cfrDth.intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_DTH_INTR, &(cfrErr[i].cfrDth.intrcfg)) );

		/// DTH sync0 interrupt antenna 1 - INFO
		cfrErr[i].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_SYNC0;
		cfrErr[i].cfrDth.intrcfg.path = 1;
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[i].cfrDth.intrcfg)) );
//		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_ENB_DTH_INTR, &(cfrErr[i].cfrDth.intrcfg)) );

		CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr[i]) );
		CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr[i]) );
    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableCfrException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE Cfr Errors and Alarms */
DFE_Err Dfe_disableCfrException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_CfrCfrIntrStatus   cfrErr[2];
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr[2];
    uint32_t                    i;

    masterLowPrioErr[0] =  DFE_FL_MISC_MASTER_LOWPRI_CFR0;
    masterLowPrioErr[1] =  DFE_FL_MISC_MASTER_LOWPRI_CFR1;

    for (i=0;i<2;i++)
    {

        // CFR PDC A0S0 interrupt - INFO
        cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A0S0;
        cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A0S0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );

        // CFR PDC A0S1 interrupt - INFO
        cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A0S1;
        cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A0S1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );

        // CFR PDC A1S0 interrupt - INFO
        cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A1S0;
        cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A1S0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );

        // CFR PDC A1S1 interrupt - INFO
        cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A1S1;
        cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A1S1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );

        // CFR PDC DFE_FL_CFR_LUTS_UPDT_DONE interrupt - INFO  // These are actually status bits and should not be handled by exception handler
        //cfrErr[i].cfrPdc[0].intrcfg = DFE_FL_CFR_LUTS_UPDT_DONE;
        //cfrErr[i].cfrPdc[1].intrcfg = DFE_FL_CFR_LUTS_UPDT_DONE;
        //CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        //CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC0_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );
        //CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[i].cfrPdc[1].intrcfg)) );
        //CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_PDC1_INTR, &(cfrErr[i].cfrPdc[1].intrcfg)) );


        /// AGC inout interrupt min in threshold antenna 0 - FATAL
        cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_IN_TH;
        cfrErr[i].cfrAgcInOut.intrcfg.path = 0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

        /// AGC inout interrupt min out threshold antenna 0 - FATAL
        cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_OUT_TH;
        cfrErr[i].cfrAgcInOut.intrcfg.path = 0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

        /// AGC inout interrupt max in threshold antenna 0 - FATAL
        cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_IN_TH;
        cfrErr[i].cfrAgcInOut.intrcfg.path = 0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR, &(cfrErr[i].cfrPdc[0].intrcfg)) );

        /// AGC inout interrupt max out threshold antenna 0 - FATAL
        cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_OUT_TH;
        cfrErr[i].cfrAgcInOut.intrcfg.path = 0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

        /// AGC inout interrupt min in threshold antenna 1 - FATAL
        cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_IN_TH;
        cfrErr[i].cfrAgcInOut.intrcfg.path = 1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

        /// AGC inout interrupt min out threshold antenna 1 - FATAL
        cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_OUT_TH;
        cfrErr[i].cfrAgcInOut.intrcfg.path = 1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

        /// AGC inout interrupt max in threshold antenna 1 - FATAL
        cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_IN_TH;
        cfrErr[i].cfrAgcInOut.intrcfg.path = 1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

        /// AGC inout interrupt max out threshold antenna 1 - FATAL
        cfrErr[i].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_OUT_TH;
        cfrErr[i].cfrAgcInOut.intrcfg.path = 1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS, &(cfrErr[i].cfrAgcInOut.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR, &(cfrErr[i].cfrAgcInOut.intrcfg)) );

        /// AGC sync0 interrupt antenna 0 - INFO
        cfrErr[i].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC0;
        cfrErr[i].cfrAgcSync.intrcfg.path = 0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[i].cfrAgcSync.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_SYNC_INTR, &(cfrErr[i].cfrAgcSync.intrcfg)) );

        /// AGC sync1 interrupt antenna 0 - INFO
        cfrErr[i].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC1;
        cfrErr[i].cfrAgcSync.intrcfg.path = 0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[i].cfrAgcSync.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_SYNC_INTR, &(cfrErr[i].cfrAgcSync.intrcfg)) );

        /// AGC sync0 interrupt antenna 1 - INFO
        cfrErr[i].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC0;
        cfrErr[i].cfrAgcSync.intrcfg.path = 1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[i].cfrAgcSync.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_SYNC_INTR, &(cfrErr[i].cfrAgcSync.intrcfg)) );

        /// AGC sync1 interrupt antenna 1 - INFO
        cfrErr[i].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC1;
        cfrErr[i].cfrAgcSync.intrcfg.path = 1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[i].cfrAgcSync.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_AGC_SYNC_INTR, &(cfrErr[i].cfrAgcSync.intrcfg)) );

        /// DTH power change interrupt antenna 0 - INFO
        cfrErr[i].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_PWR_CNG;
        cfrErr[i].cfrDth.intrcfg.path = 0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[i].cfrDth.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_DTH_INTR, &(cfrErr[i].cfrDth.intrcfg)) );

        /// DTH sync0 interrupt antenna 0 - INFO
        cfrErr[i].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_SYNC0;
        cfrErr[i].cfrDth.intrcfg.path = 0;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[i].cfrDth.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_DTH_INTR, &(cfrErr[i].cfrDth.intrcfg)) );

        /// DTH power change interrupt antenna 1 - INFO
        cfrErr[i].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_PWR_CNG;
        cfrErr[i].cfrDth.intrcfg.path = 1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[i].cfrDth.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_DTH_INTR, &(cfrErr[i].cfrDth.intrcfg)) );

        /// DTH sync0 interrupt antenna 1 - INFO
        cfrErr[i].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_SYNC0;
        cfrErr[i].cfrDth.intrcfg.path = 1;
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[i].cfrDth.intrcfg)) );
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[i], DFE_FL_CFR_CMD_DIS_DTH_INTR, &(cfrErr[i].cfrDth.intrcfg)) );

        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr[i]) );
        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr[i]) );
    }

    return DFE_ERR_NONE;
}


/** ============================================================================
 *   @n@b Dfe_getCfrException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]
         index   [index of the DDUC]

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
/* Get DFE CFR Errors and Alarms status and clear */
DFE_Err Dfe_getCfrException(
        DFE_Handle  hDfe,
        uint32_t index
)
{
    DfeFl_Status status;
	DfeFl_CfrCfrIntrStatus    cfrErr[2];
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    if (index == 0)
    	masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_CFR0;
    else if (index == 1)
    	masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_CFR1;

    hDfe->dfeEeCount.eeFlag = 1;
	cfrErr[index].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A0S0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_PDC0_INTR_STATUS, &(cfrErr[index].cfrPdc[0])) );
	hDfe->dfeEeCount.cfrErr[index].cfrPdc[0].a0s0Err += cfrErr[index].cfrPdc[0].status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[index].cfrPdc[0].intrcfg)) );

	cfrErr[index].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A0S1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_PDC0_INTR_STATUS, &(cfrErr[index].cfrPdc[0])) );
	hDfe->dfeEeCount.cfrErr[index].cfrPdc[0].a0s1Err += cfrErr[index].cfrPdc[0].status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[index].cfrPdc[0].intrcfg)) );

	cfrErr[index].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A1S0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_PDC0_INTR_STATUS, &(cfrErr[index].cfrPdc[0])) );
	hDfe->dfeEeCount.cfrErr[index].cfrPdc[0].a1s0Err += cfrErr[index].cfrPdc[0].status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[index].cfrPdc[0].intrcfg)) );

	cfrErr[index].cfrPdc[0].intrcfg = DFE_FL_CFR_CPAGE_A1S1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_PDC0_INTR_STATUS, &(cfrErr[index].cfrPdc[0])) );
	hDfe->dfeEeCount.cfrErr[index].cfrPdc[0].a1s1Err += cfrErr[index].cfrPdc[0].status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS, &(cfrErr[index].cfrPdc[0].intrcfg)) );

	cfrErr[index].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A0S0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_PDC1_INTR_STATUS, &(cfrErr[index].cfrPdc[1])) );
	hDfe->dfeEeCount.cfrErr[index].cfrPdc[1].a0s0Err += cfrErr[index].cfrPdc[1].status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[index].cfrPdc[1].intrcfg)) );

	cfrErr[index].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A0S1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_PDC1_INTR_STATUS, &(cfrErr[index].cfrPdc[1])) );
	hDfe->dfeEeCount.cfrErr[index].cfrPdc[1].a0s1Err += cfrErr[index].cfrPdc[1].status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[index].cfrPdc[1].intrcfg)) );

	cfrErr[index].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A1S0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_PDC1_INTR_STATUS, &(cfrErr[index].cfrPdc[1])) );
	hDfe->dfeEeCount.cfrErr[index].cfrPdc[1].a1s0Err += cfrErr[index].cfrPdc[1].status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[index].cfrPdc[1].intrcfg)) );

	cfrErr[index].cfrPdc[1].intrcfg = DFE_FL_CFR_CPAGE_A1S1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_PDC1_INTR_STATUS, &(cfrErr[index].cfrPdc[1])) );
	hDfe->dfeEeCount.cfrErr[index].cfrPdc[1].a1s1Err += cfrErr[index].cfrPdc[1].status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS, &(cfrErr[index].cfrPdc[1].intrcfg)) );

	cfrErr[index].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_IN_TH;
	cfrErr[index].cfrAgcInOut.intrcfg.path = 0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcInOut)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcMinInThErr[0] += cfrErr[index].cfrAgcInOut.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[index].cfrAgcInOut.intrcfg)) );

	cfrErr[index].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_IN_TH;
	cfrErr[index].cfrAgcInOut.intrcfg.path = 1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcInOut)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcMinInThErr[1] += cfrErr[index].cfrAgcInOut.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[index].cfrAgcInOut.intrcfg)) );

	cfrErr[index].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_OUT_TH;
	cfrErr[index].cfrAgcInOut.intrcfg.path = 0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcInOut)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcMinOutThErr[0] += cfrErr[index].cfrAgcInOut.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[index].cfrAgcInOut.intrcfg)) );

	cfrErr[index].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MIN_OUT_TH;
	cfrErr[index].cfrAgcInOut.intrcfg.path = 1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcInOut)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcMinOutThErr[1] += cfrErr[index].cfrAgcInOut.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[index].cfrAgcInOut.intrcfg)) );

	cfrErr[index].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_IN_TH;
	cfrErr[index].cfrAgcInOut.intrcfg.path = 0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcInOut)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcMaxInThErr[0] += cfrErr[index].cfrAgcInOut.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[index].cfrAgcInOut.intrcfg)) );

	cfrErr[index].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_IN_TH;
	cfrErr[index].cfrAgcInOut.intrcfg.path = 1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcInOut)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcMaxInThErr[1] += cfrErr[index].cfrAgcInOut.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[index].cfrAgcInOut.intrcfg)) );

	cfrErr[index].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_OUT_TH;
	cfrErr[index].cfrAgcInOut.intrcfg.path = 0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcInOut)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcMaxOutThErr[0] += cfrErr[index].cfrAgcInOut.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[index].cfrAgcInOut.intrcfg)) );

	cfrErr[index].cfrAgcInOut.intrcfg.intr = DFE_FL_CFR_AGC_MAX_OUT_TH;
	cfrErr[index].cfrAgcInOut.intrcfg.path = 1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcInOut)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcMaxOutThErr[1] += cfrErr[index].cfrAgcInOut.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR, &(cfrErr[index].cfrAgcInOut.intrcfg)) );

	cfrErr[index].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC0;
	cfrErr[index].cfrAgcSync.intrcfg.path = 0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcSync)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcSyncErr[0][0] += cfrErr[index].cfrAgcSync.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[index].cfrAgcSync.intrcfg)) );

	cfrErr[index].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC1;
	cfrErr[index].cfrAgcSync.intrcfg.path = 0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcSync)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcSyncErr[1][0] += cfrErr[index].cfrAgcSync.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[index].cfrAgcSync.intrcfg)) );

	cfrErr[index].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC0;
	cfrErr[index].cfrAgcSync.intrcfg.path = 1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcSync)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcSyncErr[0][1] += cfrErr[index].cfrAgcSync.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[index].cfrAgcSync.intrcfg)) );

	cfrErr[index].cfrAgcSync.intrcfg.intr = DFE_FL_CFR_AGC_SYNC1;
	cfrErr[index].cfrAgcSync.intrcfg.path = 1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS, &(cfrErr[index].cfrAgcSync)) );
	hDfe->dfeEeCount.cfrErr[index].cfrAgcSyncErr[1][1] += cfrErr[index].cfrAgcSync.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS, &(cfrErr[index].cfrAgcSync.intrcfg)) );

	cfrErr[index].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_PWR_CNG;
	cfrErr[index].cfrDth.intrcfg.path = 0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_DTH_INTR_STATUS, &(cfrErr[index].cfrDth)) );
	hDfe->dfeEeCount.cfrErr[index].cfrDthPwrCngErr[0] += cfrErr[index].cfrDth.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[index].cfrDth.intrcfg)) );

	cfrErr[index].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_SYNC0;
	cfrErr[index].cfrDth.intrcfg.path = 0;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_DTH_INTR_STATUS, &(cfrErr[index].cfrDth)) );
	hDfe->dfeEeCount.cfrErr[index].cfrDthSync0Err[0] += cfrErr[index].cfrDth.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[index].cfrDth.intrcfg)) );

	cfrErr[index].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_PWR_CNG;
	cfrErr[index].cfrDth.intrcfg.path = 1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_DTH_INTR_STATUS, &(cfrErr[index].cfrDth)) );
	hDfe->dfeEeCount.cfrErr[index].cfrDthPwrCngErr[1] += cfrErr[index].cfrDth.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[index].cfrDth.intrcfg)) );

	cfrErr[index].cfrDth.intrcfg.intr = DFE_FL_CFR_DTH_SYNC0;
	cfrErr[index].cfrDth.intrcfg.path = 1;
	CSL_HW_QUERY( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[index], DFE_FL_CFR_QUERY_GET_DTH_INTR_STATUS, &(cfrErr[index].cfrDth)) );
	hDfe->dfeEeCount.cfrErr[index].cfrDthSync0Err[1] += cfrErr[index].cfrDth.status;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[index], DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS, &(cfrErr[index].cfrDth.intrcfg)) );

	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableTxException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Tx Errors and Alarms */
DFE_Err Dfe_enableTxException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_TxIntrStatus    txErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_TX;

	//txa antenna a power is approaching saturation - ERROR
	txErr.txDev = DFE_FL_TXA_PATHA;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_ENB_PAPOWER_INTR, &(txErr.txDev)) );
	//txa antenna a peak is approaching saturation - ERROR
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_ENB_PAPEAK_INTR, &(txErr.txDev)) );

	//txa antenna b power is approaching saturation - ERROR
	txErr.txDev = DFE_FL_TXA_PATHB;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_ENB_PAPOWER_INTR, &(txErr.txDev)) );
	//txa antenna b peak is approaching saturation - ERROR
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_ENB_PAPEAK_INTR, &(txErr.txDev)) );

	//txb antenna a power is approaching saturation - ERROR
	txErr.txDev = DFE_FL_TXB_PATHA;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_ENB_PAPOWER_INTR, &(txErr.txDev)) );
	//txb antenna a peak is approaching saturation - ERROR
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_ENB_PAPEAK_INTR, &(txErr.txDev)) );

	//txb antenna b power is approaching saturation - ERROR
	txErr.txDev = DFE_FL_TXB_PATHB;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_ENB_PAPOWER_INTR, &(txErr.txDev)) );
	//txb antenna b peak is approaching saturation - ERROR
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_ENB_PAPEAK_INTR, &(txErr.txDev)) );

	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableTxException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE Tx Errors and Alarms */
DFE_Err Dfe_disableTxException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_TxIntrStatus    txErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_TX;

    //txa antenna a power is approaching saturation - ERROR
    txErr.txDev = DFE_FL_TXA_PATHA;
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_DIS_PAPOWER_INTR, &(txErr.txDev)) );
    //txa antenna a peak is approaching saturation - ERROR
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_DIS_PAPEAK_INTR, &(txErr.txDev)) );

    //txa antenna b power is approaching saturation - ERROR
    txErr.txDev = DFE_FL_TXA_PATHB;
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_DIS_PAPOWER_INTR, &(txErr.txDev)) );
    //txa antenna b peak is approaching saturation - ERROR
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_DIS_PAPEAK_INTR, &(txErr.txDev)) );

    //txb antenna a power is approaching saturation - ERROR
    txErr.txDev = DFE_FL_TXB_PATHA;
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_DIS_PAPOWER_INTR, &(txErr.txDev)) );
    //txb antenna a peak is approaching saturation - ERROR
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_DIS_PAPEAK_INTR, &(txErr.txDev)) );

    //txb antenna b power is approaching saturation - ERROR
    txErr.txDev = DFE_FL_TXB_PATHB;
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_DIS_PAPOWER_INTR, &(txErr.txDev)) );
    //txb antenna b peak is approaching saturation - ERROR
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_DIS_PAPEAK_INTR, &(txErr.txDev)) );

    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getTxException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE Tx Errors and Alarms status and clear */
DFE_Err Dfe_getTxException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_TxIntrStatus    txErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_TX;

    hDfe->dfeEeCount.eeFlag = 1;

    txErr.status = 0;
    txErr.txDev  = DFE_FL_TXA_PATHA;
	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
	hDfe->dfeEeCount.txPaPowerErr[0][0] += txErr.status;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );

	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
	hDfe->dfeEeCount.txPaPeakErr[0][0] += txErr.status;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );

    txErr.txDev = DFE_FL_TXA_PATHB;
	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
	hDfe->dfeEeCount.txPaPowerErr[0][1] += txErr.status;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );

	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
	hDfe->dfeEeCount.txPaPeakErr[0][1] += txErr.status;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );

    txErr.txDev = DFE_FL_TXB_PATHA;
	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
	hDfe->dfeEeCount.txPaPowerErr[1][0] += txErr.status;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );

	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
	hDfe->dfeEeCount.txPaPeakErr[1][0] += txErr.status;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );

    txErr.txDev = DFE_FL_TXB_PATHB;
	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PAPOWER_INTR_STATUS, &(txErr.txDev)) );
	hDfe->dfeEeCount.txPaPowerErr[1][1] += txErr.status;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS, &(txErr.txDev)) );

	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PAPEAK_INTR_STATUS, &(txErr.txDev)) );
	hDfe->dfeEeCount.txPaPeakErr[1][1] += txErr.status;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS, &(txErr.txDev)) );

	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableJesdException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Tx Errors and Alarms */
DFE_Err Dfe_enableJesdException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_JesdGroupIntrStatus    jesdErr;
    DfeFl_MiscMasterLowPriIntr   masterLowPrioErr;
    DfeFl_BbLoopbackConfig       isBbLoopbackEnabled;
    uint32_t                     isAidLoopbackEnabled;
    DfeFl_JesdRxLoopbackConfig   isJesdLoopbackEnabled;

    // Enable Jesd exceptions only if no AID nor buffer loopback is used
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_AID_LOOPBACK_CFG, &isAidLoopbackEnabled) );
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_LOOPBACK_CFG, &isBbLoopbackEnabled) );
    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LOOPBACK, &isJesdLoopbackEnabled) );

    if ( (isAidLoopbackEnabled == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc1 == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc3 == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc7 == 0) &&
         (isBbLoopbackEnabled.duc1ToDdc2 == 0) &&
         (isBbLoopbackEnabled.duc1ToDdc6 == 0) &&
         (isBbLoopbackEnabled.duc2ToDdc5 == 0) &&
         (isBbLoopbackEnabled.duc3ToDdc4 == 0) &&
         (isJesdLoopbackEnabled.lane0 == 0) &&
         (isJesdLoopbackEnabled.lane1 == 0) &&
         (isJesdLoopbackEnabled.lane2 == 0) &&
         (isJesdLoopbackEnabled.lane3 == 0) &&
         (isJesdLoopbackEnabled.tx0_rx0 == 0) &&
         (isJesdLoopbackEnabled.tx0_rx1fb0 == 0) &&
         (isJesdLoopbackEnabled.tx1_rx2fb1 == 0)
       )
    {

        masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_JESD;

        jesdErr.jesdTxLane.lane = DFE_FL_JESD_LANE_ALL;  //set to DFE_FL_JESD_LANE_ALL
        // FATAL
        jesdErr.jesdTxLane.fifoEmptyIntr = 1;
        // FATAL
        jesdErr.jesdTxLane.fifoFullIntr = 1;
        // FATAL
        jesdErr.jesdTxLane.fifoReadErrIntr = 1;
        // FATAL
        jesdErr.jesdTxLane.fifoWriteErrIntr = 1;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_LANE_INTRGRP_STATUS, &(jesdErr.jesdTxLane)) );
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_ENB_LANE_INTRGRP, &(jesdErr.jesdTxLane)) );

        // ERROR
        jesdErr.jesdTxSysref.errLink0Intr = 1;
        // ERROR
        jesdErr.jesdTxSysref.errLink1Intr = 1;
        // INFO
        jesdErr.jesdTxSysref.reqAssertIntr = 1;
        // INFO
        jesdErr.jesdTxSysref.reqDeassertIntr = 1;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_SYSREF_INTRGRP_STATUS, &(jesdErr.jesdTxSysref)) );
        jesdErr.jesdTxSysref.reqAssertIntr = 0;
        jesdErr.jesdTxSysref.reqDeassertIntr = 0;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_ENB_SYSREF_INTRGRP, &(jesdErr.jesdTxSysref)) );

        jesdErr.jesdRxLane.lane = DFE_FL_JESD_LANE_ALL;  //set to DFE_FL_JESD_LANE_ALL
        // ERROR
        jesdErr.jesdRxLane.decDispErrIntr = 1;
        // ERROR
        jesdErr.jesdRxLane.decCodeErrIntr = 1;
        // ERROR
        jesdErr.jesdRxLane.codeSyncErrIntr = 1;
        // ERROR
        jesdErr.jesdRxLane.bufMatchErrIntr = 1;
        // FATAL
        jesdErr.jesdRxLane.bufOverflowErrIntr = 1;
        // ERROR
        jesdErr.jesdRxLane.linkConfigErrIntr = 1;
        // ERROR
        jesdErr.jesdRxLane.frameAlignErrIntr = 1;
        // ERROR
        jesdErr.jesdRxLane.multiframeAlignErrIntr = 1;
        // FATAL
        jesdErr.jesdRxLane.fifoEmptyIntr = 1;
        // FATAL
        jesdErr.jesdRxLane.fifoReadErrIntr = 1;
        // FATAl
        jesdErr.jesdRxLane.fifoFullIntr = 1;
        // FATAL
        jesdErr.jesdRxLane.fifoWriteErrIntr = 1;
        // ERROR
        jesdErr.jesdRxLane.testSeqErrIntr = 1;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CLR_LANE_INTRGRP_STATUS, &(jesdErr.jesdRxLane)) );
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_ENB_LANE_INTRGRP, &(jesdErr.jesdRxLane)) );

        // FATAL
        jesdErr.jesdRxSysref.errLink0Intr = 1;
        // FATAL
        jesdErr.jesdRxSysref.errLink1Intr = 1;
        // INFO
        jesdErr.jesdRxSysref.reqAssertIntr = 1;
        // INFO
        jesdErr.jesdRxSysref.reqDeassertIntr = 1;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CLR_SYSREF_INTRGRP_STATUS, &(jesdErr.jesdRxSysref)) );
        jesdErr.jesdRxSysref.reqAssertIntr = 0;
        jesdErr.jesdRxSysref.reqDeassertIntr = 0;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_ENB_SYSREF_INTRGRP, &(jesdErr.jesdRxSysref)) );

        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableJesdException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Tx Errors and Alarms */
DFE_Err Dfe_disableJesdException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_JesdGroupIntrStatus    jesdErr;
    DfeFl_MiscMasterLowPriIntr   masterLowPrioErr;
    DfeFl_BbLoopbackConfig       isBbLoopbackEnabled;
    uint32_t                     isAidLoopbackEnabled;
    DfeFl_JesdRxLoopbackConfig   isJesdLoopbackEnabled;

    // Enable Jesd exceptions only if no AID nor buffer loopback is used
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_AID_LOOPBACK_CFG, &isAidLoopbackEnabled) );
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_LOOPBACK_CFG, &isBbLoopbackEnabled) );
    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LOOPBACK, &isJesdLoopbackEnabled) );

    if ( (isAidLoopbackEnabled == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc1 == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc3 == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc7 == 0) &&
         (isBbLoopbackEnabled.duc1ToDdc2 == 0) &&
         (isBbLoopbackEnabled.duc1ToDdc6 == 0) &&
         (isBbLoopbackEnabled.duc2ToDdc5 == 0) &&
         (isBbLoopbackEnabled.duc3ToDdc4 == 0) &&
         (isJesdLoopbackEnabled.lane0 == 0) &&
         (isJesdLoopbackEnabled.lane1 == 0) &&
         (isJesdLoopbackEnabled.lane2 == 0) &&
         (isJesdLoopbackEnabled.lane3 == 0) &&
         (isJesdLoopbackEnabled.tx0_rx0 == 0) &&
         (isJesdLoopbackEnabled.tx0_rx1fb0 == 0) &&
         (isJesdLoopbackEnabled.tx1_rx2fb1 == 0)
       )
    {

        masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_JESD;

        jesdErr.jesdTxLane.lane = DFE_FL_JESD_LANE_ALL;  //set to DFE_FL_JESD_LANE_ALL
        // FATAL
        jesdErr.jesdTxLane.fifoEmptyIntr = 0;
        // FATAL
        jesdErr.jesdTxLane.fifoFullIntr = 0;
        // FATAL
        jesdErr.jesdTxLane.fifoReadErrIntr = 0;
        // FATAL
        jesdErr.jesdTxLane.fifoWriteErrIntr = 0;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_LANE_INTRGRP_STATUS, &(jesdErr.jesdTxLane)) );
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_DIS_LANE_INTRGRP, &(jesdErr.jesdTxLane)) );

        // ERROR
        jesdErr.jesdTxSysref.errLink0Intr = 0;
        // ERROR
        jesdErr.jesdTxSysref.errLink1Intr = 0;
        // INFO
        jesdErr.jesdTxSysref.reqAssertIntr = 0;
        // INFO
        jesdErr.jesdTxSysref.reqDeassertIntr = 0;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_SYSREF_INTRGRP_STATUS, &(jesdErr.jesdTxSysref)) );
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_DIS_SYSREF_INTRGRP, &(jesdErr.jesdTxSysref)) );

        jesdErr.jesdRxLane.lane = DFE_FL_JESD_LANE_ALL;  //set to DFE_FL_JESD_LANE_ALL
        // ERROR
        jesdErr.jesdRxLane.decDispErrIntr = 0;
        // ERROR
        jesdErr.jesdRxLane.decCodeErrIntr = 0;
        // ERROR
        jesdErr.jesdRxLane.codeSyncErrIntr = 0;
        // ERROR
        jesdErr.jesdRxLane.bufMatchErrIntr = 0;
        // FATAL
        jesdErr.jesdRxLane.bufOverflowErrIntr = 0;
        // ERROR
        jesdErr.jesdRxLane.linkConfigErrIntr = 0;
        // ERROR
        jesdErr.jesdRxLane.frameAlignErrIntr = 0;
        // ERROR
        jesdErr.jesdRxLane.multiframeAlignErrIntr = 0;
        // FATAL
        jesdErr.jesdRxLane.fifoEmptyIntr = 0;
        // FATAL
        jesdErr.jesdRxLane.fifoReadErrIntr = 0;
        // FATAl
        jesdErr.jesdRxLane.fifoFullIntr = 0;
        // FATAL
        jesdErr.jesdRxLane.fifoWriteErrIntr = 0;
        // ERROR
        jesdErr.jesdRxLane.testSeqErrIntr = 0;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CLR_LANE_INTRGRP_STATUS, &(jesdErr.jesdRxLane)) );
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_DIS_LANE_INTRGRP, &(jesdErr.jesdRxLane)) );

        // FATAL
        jesdErr.jesdRxSysref.errLink0Intr = 0;
        // FATAL
        jesdErr.jesdRxSysref.errLink1Intr = 0;
        // INFO
        jesdErr.jesdRxSysref.reqAssertIntr = 0;
        // INFO
        jesdErr.jesdRxSysref.reqDeassertIntr = 0;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CLR_SYSREF_INTRGRP_STATUS, &(jesdErr.jesdRxSysref)) );
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_DIS_SYSREF_INTRGRP, &(jesdErr.jesdRxSysref)) );

        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getJesdException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE Jesd Errors and Alarms status and clear */
DFE_Err Dfe_getJesdException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_JesdGroupIntrStatus    jesdErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t index;
    DfeFl_BbLoopbackConfig       isBbLoopbackEnabled;
    uint32_t                     isAidLoopbackEnabled;
    DfeFl_JesdRxLoopbackConfig   isJesdLoopbackEnabled;

    // Enable Jesd exceptions only if no AID nor buffer loopback is used
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_AID_LOOPBACK_CFG, &isAidLoopbackEnabled) );
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_LOOPBACK_CFG, &isBbLoopbackEnabled) );
    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LOOPBACK, &isJesdLoopbackEnabled) );

    if ( (isAidLoopbackEnabled == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc1 == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc3 == 0) &&
         (isBbLoopbackEnabled.duc0ToDdc7 == 0) &&
         (isBbLoopbackEnabled.duc1ToDdc2 == 0) &&
         (isBbLoopbackEnabled.duc1ToDdc6 == 0) &&
         (isBbLoopbackEnabled.duc2ToDdc5 == 0) &&
         (isBbLoopbackEnabled.duc3ToDdc4 == 0) &&
         (isJesdLoopbackEnabled.lane0 == 0) &&
         (isJesdLoopbackEnabled.lane1 == 0) &&
         (isJesdLoopbackEnabled.lane2 == 0) &&
         (isJesdLoopbackEnabled.lane3 == 0) &&
         (isJesdLoopbackEnabled.tx0_rx0 == 0) &&
         (isJesdLoopbackEnabled.tx0_rx1fb0 == 0) &&
         (isJesdLoopbackEnabled.tx1_rx2fb1 == 0)
       )
    {

        masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_JESD;

        hDfe->dfeEeCount.eeFlag = 1;

        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_SYSREF_INTRGRP_STATUS, &(jesdErr.jesdTxSysref)) );
        hDfe->dfeEeCount.jesdTxSysref.errLink0Intr += jesdErr.jesdTxSysref.errLink0Intr;
        hDfe->dfeEeCount.jesdTxSysref.errLink1Intr += jesdErr.jesdTxSysref.errLink1Intr;
        hDfe->dfeEeCount.jesdTxSysref.reqAssertIntr += jesdErr.jesdTxSysref.reqAssertIntr;
        hDfe->dfeEeCount.jesdTxSysref.reqDeassertIntr += jesdErr.jesdTxSysref.reqDeassertIntr;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_SYSREF_INTRGRP_STATUS, &(jesdErr.jesdTxSysref)) );

        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_SYSREF_INTRGRP_STATUS, &(jesdErr.jesdRxSysref)) );
        hDfe->dfeEeCount.jesdRxSysref.errLink0Intr += jesdErr.jesdRxSysref.errLink0Intr;
        hDfe->dfeEeCount.jesdRxSysref.errLink1Intr += jesdErr.jesdRxSysref.errLink1Intr;
        hDfe->dfeEeCount.jesdRxSysref.reqAssertIntr += jesdErr.jesdRxSysref.reqAssertIntr;
        hDfe->dfeEeCount.jesdRxSysref.reqDeassertIntr += jesdErr.jesdRxSysref.reqDeassertIntr;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CLR_SYSREF_INTRGRP_STATUS, &(jesdErr.jesdRxSysref)) );

        for (index=0;index<4;index++)
        {
            jesdErr.jesdTxLane.lane = index;
            CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_LANE_INTRGRP_STATUS, &(jesdErr.jesdTxLane)) );
            hDfe->dfeEeCount.jesdTxLaneErr[index].fifoEmptyIntr += jesdErr.jesdTxLane.fifoEmptyIntr;
            hDfe->dfeEeCount.jesdTxLaneErr[index].fifoFullIntr += jesdErr.jesdTxLane.fifoFullIntr;
            hDfe->dfeEeCount.jesdTxLaneErr[index].fifoReadErrIntr += jesdErr.jesdTxLane.fifoReadErrIntr;
            hDfe->dfeEeCount.jesdTxLaneErr[index].fifoWriteErrIntr += jesdErr.jesdTxLane.fifoWriteErrIntr;
            CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_LANE_INTRGRP_STATUS, &(jesdErr.jesdTxLane)) );

            jesdErr.jesdRxLane.lane = index;
            CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LANE_INTRGRP_STATUS, &(jesdErr.jesdRxLane)) );
            hDfe->dfeEeCount.jesdRxLaneErr[index].bufMatchErrIntr += jesdErr.jesdRxLane.bufMatchErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].bufOverflowErrIntr += jesdErr.jesdRxLane.bufOverflowErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].codeSyncErrIntr += jesdErr.jesdRxLane.codeSyncErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].decCodeErrIntr += jesdErr.jesdRxLane.decCodeErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].fifoEmptyIntr += jesdErr.jesdRxLane.fifoEmptyIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].fifoFullIntr += jesdErr.jesdRxLane.fifoFullIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].fifoReadErrIntr += jesdErr.jesdRxLane.fifoReadErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].fifoWriteErrIntr += jesdErr.jesdRxLane.fifoWriteErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].frameAlignErrIntr += jesdErr.jesdRxLane.frameAlignErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].linkConfigErrIntr += jesdErr.jesdRxLane.linkConfigErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].multiframeAlignErrIntr += jesdErr.jesdRxLane.multiframeAlignErrIntr;
            hDfe->dfeEeCount.jesdRxLaneErr[index].testSeqErrIntr += jesdErr.jesdRxLane.testSeqErrIntr;
            CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CLR_LANE_INTRGRP_STATUS, &(jesdErr.jesdRxLane)) );
        }
        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableDpdaException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Dpda Errors and Alarms */
DFE_Err Dfe_enableDpdaException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    if (hDfe->dpdaIsDisabled == 0)
    {
        masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_DPDA;

        // INFO
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CLR_INT_READ_COMPLETE_INTR_STATUS, NULL) );
    //	CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_ENB_INT_READ_COMPLETE_INTR, NULL) );

        // INFO
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CLR_INT_PROCESSED_INTR_STATUS, NULL) );
    //	CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_ENB_INT_PROCESSED_INTR, NULL) );

        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );
    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableDpdaException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE Dpda Errors and Alarms */
DFE_Err Dfe_disableDpdaException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    if (hDfe->dpdaIsDisabled == 0)
    {
        masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_DPDA;

        // INFO
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CLR_INT_READ_COMPLETE_INTR_STATUS, NULL) );
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_DIS_INT_READ_COMPLETE_INTR, NULL) );

        // INFO
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CLR_INT_PROCESSED_INTR_STATUS, NULL) );
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_DIS_INT_PROCESSED_INTR, NULL) );

        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );
    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getDpdaException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE Dpda Errors and Alarms status and clear */
DFE_Err Dfe_getDpdaException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_DpdaIntrStatus    dpdaErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_DPDA;

    if (hDfe->dpdaIsDisabled == 0)
    {
        CSL_HW_QUERY( dfeFl_DpdaGetHwStatus(hDfe->hDfeDpda[0], DFE_FL_DPDA_QUERY_GET_INT_READ_COMPLETE_INTR_STATUS, &(dpdaErr.readCompleteStatus)) );
        if (dpdaErr.readCompleteStatus) hDfe->dfeEeCount.eeFlag = 1;
        hDfe->dfeEeCount.dpdaErr.readCompleteStatus += dpdaErr.readCompleteStatus;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CLR_INT_READ_COMPLETE_INTR_STATUS, NULL) );

        CSL_HW_QUERY( dfeFl_DpdaGetHwStatus(hDfe->hDfeDpda[0], DFE_FL_DPDA_QUERY_GET_INT_PROCESSED_INTR_STATUS, &(dpdaErr.intProcessedStatus)) );
        if (dpdaErr.intProcessedStatus) hDfe->dfeEeCount.eeFlag = 1;
        hDfe->dfeEeCount.dpdaErr.intProcessedStatus += dpdaErr.intProcessedStatus;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CLR_INT_PROCESSED_INTR_STATUS, NULL) );

        CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    }

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableRxException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Rx Errors and Alarms */
DFE_Err Dfe_enableRxException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t i;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM;

    // 4 bit interrupt (one for each antenna). Goes high when ibpm measurements are done - INFO
    for (i=0;i<4;i++)
    {
		CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CLR_POWMTR_INTR_STATUS, &(i)) );
//		CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_ENB_POWMTR_INTR, &(i)) );
    }

	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableRxException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE Rx Errors and Alarms */
DFE_Err Dfe_disableRxException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t i;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM;

    // 4 bit interrupt (one for each antenna). Goes high when ibpm measurements are done - INFO
    for (i=0;i<4;i++)
    {
        CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CLR_POWMTR_INTR_STATUS, &(i)) );
        CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_DIS_POWMTR_INTR, &(i)) );
    }

    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getRxException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE Rx Errors and Alarms status and clear */
DFE_Err Dfe_getRxException(
        DFE_Handle  hDfe,
        uint32_t index
)
{
    DfeFl_Status status;
	DfeFl_RxPowmtrGeneric rxErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM;

    hDfe->dfeEeCount.eeFlag = 1;

    rxErr.powmtr = index;
	CSL_HW_QUERY( dfeFl_RxGetHwStatus(hDfe->hDfeRx[0], DFE_FL_RX_QUERY_POWMTR_INTR_STATUS, &(rxErr)) );
	hDfe->dfeEeCount.rxIbpmInt[index] += rxErr.data;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CLR_POWMTR_INTR_STATUS, &(index)) );

	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableCbException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Cb Errors and Alarms */
DFE_Err Dfe_enableCbException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_CB;

    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableCbException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Cb Errors and Alarms */
DFE_Err Dfe_disableCbException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_CB;

    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getCbException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE Rx Errors and Alarms status and clear */
DFE_Err Dfe_getCbException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_CB;

    hDfe->dfeEeCount.infoFlag = 1;

    hDfe->dfeEeCount.cbDoneInterrupt += 1;

    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableMiscException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Misc Errors and Alarms */
DFE_Err Dfe_enableMiscException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_MiscCppIntrGroupStatus miscErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t i;

    // The entire 16-bit general sync bus - INFO
    for (i=0;i<16;i++)
    {
        // CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_SYNC_INTR_STATUS, &(i)) );
        //	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_SYNC_INTR, &(i)) );
    }

	// One interrupt per CPP DMA (up to 32) - INFO
	for (i=0;i<32;i++)
	{
		CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTR_STATUS, &(i)) );
	//	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_CPP_DMA_DONE_INTR, &(i)) );
	}

	// Fifo full error flag - ERROR
    miscErr.arbGpio.arbFifoErr = 1;
    // MPU read operation did not get Ack - ERROR
    miscErr.arbGpio.cppRdNack = 1;
    // poly2lut done flag - INFO
    miscErr.arbGpio.arbP2lDone = 0;
    //fb antenna is switched - INFO
    miscErr.arbGpio.arbFbSwitch = 1;
    // All the gpio input bits, regardless of what is being mapped. - INFO
    for (i=0;i<18;i++)
    {
    	miscErr.arbGpio.gpio[i] = 1;
    }
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MISC_INTRGRP_STATUS, &(miscErr.arbGpio)) );
    miscErr.arbGpio.arbP2lDone = 0;
    miscErr.arbGpio.arbFbSwitch = 0;
    for (i=0;i<18;i++)
    {
        miscErr.arbGpio.gpio[i] = 0;
    }
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MISC_INTRGRP, &(miscErr.arbGpio)) );

	masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_MISC;
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableMiscException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE Misc Errors and Alarms */
DFE_Err Dfe_disableMiscException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscCppIntrGroupStatus miscErr;
    //DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t i;

    // The entire 16-bit general sync bus - INFO
    for (i=0;i<16;i++)
    {
        //CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_SYNC_INTR_STATUS, &(i)) );
        //CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_SYNC_INTR, &(i)) );
    }

    // One interrupt per CPP DMA (up to 32) - INFO
    for (i=0;i<32;i++)
    {
    //  CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTR_STATUS, &(i)) );
    //  CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_CPP_DMA_DONE_INTR, &(i)) );
    }

    // Fifo full error flag - ERROR
    miscErr.arbGpio.arbFifoErr = 0;
    // MPU read operation did not get Ack - ERROR
    miscErr.arbGpio.cppRdNack = 0;
    // poly2lut done flag - INFO
    miscErr.arbGpio.arbP2lDone = 0;
    //fb antenna is switched - INFO
    miscErr.arbGpio.arbFbSwitch = 0;
    // All the gpio input bits, regardless of what is being mapped. - INFO
    for (i=0;i<18;i++)
    {
        miscErr.arbGpio.gpio[i] = 0;
    }
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MISC_INTRGRP_STATUS, &(miscErr.arbGpio)) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MISC_INTRGRP, &(miscErr.arbGpio)) );

    // Keep enabled as CPP DMA done could still be needed
    /*masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_MISC;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );*/

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getMiscException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE Misc Errors and Alarms status and clear */
DFE_Err Dfe_getMiscException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_MiscCppIntrGroupStatus miscErr;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;
    uint32_t i;

    for (i=0;i<16;i++)
    {
        //miscErr.syncSig.intr = i;
        //CSL_HW_QUERY( dfeFl_MiscGetHwStatus(hDfe->hDfeMisc[0], DFE_FL_MISC_QUERY_GET_SYNC_INTR_STATUS, &(miscErr.syncSig)) );
        //hDfe->dfeEeCount.miscErr.syncSigInt[i] += miscErr.syncSig.data; if (miscErr.syncSig.data) hDfe->dfeEeCount.eeFlag = 1;
        //CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_SYNC_INTR_STATUS, &(miscErr.syncSig.intr)) );
    }

	for (i=0;i<32;i++)
	{
		miscErr.cppDmaDone.intr = i;
		CSL_HW_QUERY( dfeFl_MiscGetHwStatus(hDfe->hDfeMisc[0], DFE_FL_MISC_QUERY_GET_CPP_DMA_DONE_INTR_STATUS, &(miscErr.cppDmaDone)) );
		hDfe->dfeEeCount.miscErr.cppDmaDoneInt[i] += miscErr.cppDmaDone.data; if (miscErr.cppDmaDone.data) hDfe->dfeEeCount.infoFlag = 1;
		CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTR_STATUS, &(miscErr.cppDmaDone.intr)) );
	}

	CSL_HW_QUERY( dfeFl_MiscGetHwStatus(hDfe->hDfeMisc[0], DFE_FL_MISC_QUERY_GET_MISC_INTRGRP_STATUS, &(miscErr.arbGpio)) );
	hDfe->dfeEeCount.miscErr.arbGpio.arbFbSwitch += miscErr.arbGpio.arbFbSwitch; if (miscErr.arbGpio.arbFbSwitch) hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.miscErr.arbGpio.arbFifoErr += miscErr.arbGpio.arbFifoErr; if (miscErr.arbGpio.arbFifoErr) hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.miscErr.arbGpio.arbP2lDone += miscErr.arbGpio.arbP2lDone; if (miscErr.arbGpio.arbP2lDone) hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.miscErr.arbGpio.cppRdNack += miscErr.arbGpio.cppRdNack; if (miscErr.arbGpio.cppRdNack) hDfe->dfeEeCount.eeFlag = 1;
	for(i=0;i<18;i++)
	{
		hDfe->dfeEeCount.miscErr.arbGpio.gpio[i] += miscErr.arbGpio.gpio[i]; if (miscErr.arbGpio.gpio[i]) hDfe->dfeEeCount.eeFlag = 1;
	}
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MISC_INTRGRP_STATUS, &(miscErr.arbGpio)) );

	masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_MISC;
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableCppDmaDoneException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE Cpp Dma Done Errors and Alarms */
DFE_Err Dfe_enableCppDmaDoneException(
        DFE_Handle  hDfe,
        uint32_t    cppDmaChannelNum
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

    // One interrupt per CPP DMA (up to 32) - INFO
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTR_STATUS, &(cppDmaChannelNum)) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_CPP_DMA_DONE_INTR, &(cppDmaChannelNum)) );

    masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_MISC;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableCppDmaDoneException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE Cpp Dma Done Errors and Alarms */
DFE_Err Dfe_disableCppDmaDoneException(
        DFE_Handle  hDfe,
        uint32_t    cppDmaChannelNum
)
{
    DfeFl_Status status;
    // One interrupt per CPP DMA (up to 32) - INFO
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_CPP_DMA_DONE_INTR, &(cppDmaChannelNum)) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableHiMiscException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable DFE High Misc Errors and Alarms */
DFE_Err Dfe_enableHiMiscException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_MiscMasterHiPriIntrGroup masterHiPrioErr;

    // - ERROR
    masterHiPrioErr.cppDmaErr = 1;
    // - ERROR
    masterHiPrioErr.txaAnt0CfrGain = 1;
    // - ERROR
    masterHiPrioErr.txaAnt0PeakClip = 1;
    // - ERROR
    masterHiPrioErr.txaAnt0PwrSat = 1;
    // - FATAL
    masterHiPrioErr.txaAnt0TxZero = 1;
    // - ERROR
    masterHiPrioErr.txaAnt1CfrGain = 1;
    // - ERROR
    masterHiPrioErr.txaAnt1PeakClip = 1;
    // - ERROR
    masterHiPrioErr.txaAnt1PwrSat = 1;
    // - FATAL
    masterHiPrioErr.txaAnt1TxZero = 1;
    // - ERROR
    masterHiPrioErr.txbAnt0CfrGain = 1;
    // - ERROR
    masterHiPrioErr.txbAnt0PeakClip = 1;
    // - ERROR
    masterHiPrioErr.txbAnt0PwrSat = 1;
    // - FATAL
    masterHiPrioErr.txbAnt0TxZero = 1;
    // - ERROR
    masterHiPrioErr.txbAnt1CfrGain = 1;
    // - ERROR
    masterHiPrioErr.txbAnt1PeakClip = 1;
    // - ERROR
    masterHiPrioErr.txbAnt1PwrSat = 1;
    // - FATAL
    masterHiPrioErr.txbAnt1TxZero = 1;
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_HIPRI_INTRGRP_STATUS, &(masterHiPrioErr)) );
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_HIPRI_INTRGRP, &(masterHiPrioErr)) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_disableHiMiscException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable DFE High Misc Errors and Alarms */
DFE_Err Dfe_disableHiMiscException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
    DfeFl_MiscMasterHiPriIntrGroup masterHiPrioErr;

    // - ERROR
    masterHiPrioErr.cppDmaErr = 0;
    // - ERROR
    masterHiPrioErr.txaAnt0CfrGain = 0;
    // - ERROR
    masterHiPrioErr.txaAnt0PeakClip = 0;
    // - ERROR
    masterHiPrioErr.txaAnt0PwrSat = 0;
    // - FATAL
    masterHiPrioErr.txaAnt0TxZero = 0;
    // - ERROR
    masterHiPrioErr.txaAnt1CfrGain = 0;
    // - ERROR
    masterHiPrioErr.txaAnt1PeakClip = 0;
    // - ERROR
    masterHiPrioErr.txaAnt1PwrSat = 0;
    // - FATAL
    masterHiPrioErr.txaAnt1TxZero = 0;
    // - ERROR
    masterHiPrioErr.txbAnt0CfrGain = 0;
    // - ERROR
    masterHiPrioErr.txbAnt0PeakClip = 0;
    // - ERROR
    masterHiPrioErr.txbAnt0PwrSat = 0;
    // - FATAL
    masterHiPrioErr.txbAnt0TxZero = 0;
    // - ERROR
    masterHiPrioErr.txbAnt1CfrGain = 0;
    // - ERROR
    masterHiPrioErr.txbAnt1PeakClip = 0;
    // - ERROR
    masterHiPrioErr.txbAnt1PwrSat = 0;
    // - FATAL
    masterHiPrioErr.txbAnt1TxZero = 0;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_HIPRI_INTRGRP_STATUS, &(masterHiPrioErr)) );
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_DIS_MASTER_HIPRI_INTRGRP, &(masterHiPrioErr)) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getHiMiscException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get DFE Misc Errors and Alarms status and clear */
DFE_Err Dfe_getHiMiscException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_MiscMasterHiPriIntrGroup masterHiPrioErr;

	CSL_HW_QUERY( dfeFl_MiscGetHwStatus(hDfe->hDfeMisc[0], DFE_FL_MISC_QUERY_GET_MASTER_HIPRI_INTRGRP_STATUS, &(masterHiPrioErr)) );
	hDfe->dfeEeCount.masterHiPrioErr.cppDmaErr += masterHiPrioErr.cppDmaErr;                if (masterHiPrioErr.cppDmaErr)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txaAnt0CfrGain += masterHiPrioErr.txaAnt0CfrGain;      if (masterHiPrioErr.txaAnt0CfrGain)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txaAnt0PeakClip += masterHiPrioErr.txaAnt0PeakClip;    if (masterHiPrioErr.txaAnt0PeakClip)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txaAnt0PwrSat += masterHiPrioErr.txaAnt0PwrSat;        if (masterHiPrioErr.txaAnt0PwrSat)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txaAnt0TxZero += masterHiPrioErr.txaAnt0TxZero;        if (masterHiPrioErr.txaAnt0TxZero)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txaAnt1CfrGain += masterHiPrioErr.txaAnt1CfrGain;      if (masterHiPrioErr.txaAnt1CfrGain)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txaAnt1PeakClip += masterHiPrioErr.txaAnt1PeakClip;    if (masterHiPrioErr.txaAnt1PeakClip)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txaAnt1PwrSat += masterHiPrioErr.txaAnt1PwrSat;        if (masterHiPrioErr.txaAnt1PwrSat)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txaAnt1TxZero += masterHiPrioErr.txaAnt1TxZero;        if (masterHiPrioErr.txaAnt1TxZero)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txbAnt0CfrGain += masterHiPrioErr.txbAnt0CfrGain;      if (masterHiPrioErr.txbAnt0CfrGain)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txbAnt0PeakClip += masterHiPrioErr.txbAnt0PeakClip;    if (masterHiPrioErr.txbAnt0PeakClip)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txbAnt0PwrSat += masterHiPrioErr.txbAnt0PwrSat;        if (masterHiPrioErr.txbAnt0PwrSat)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txbAnt0TxZero += masterHiPrioErr.txbAnt0TxZero;        if (masterHiPrioErr.txbAnt0TxZero)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txbAnt1CfrGain += masterHiPrioErr.txbAnt1CfrGain;      if (masterHiPrioErr.txbAnt1CfrGain)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txbAnt1PeakClip += masterHiPrioErr.txbAnt1PeakClip;    if (masterHiPrioErr.txbAnt1PeakClip)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txbAnt1PwrSat += masterHiPrioErr.txbAnt1PwrSat;        if (masterHiPrioErr.txbAnt1PwrSat)  hDfe->dfeEeCount.eeFlag = 1;
	hDfe->dfeEeCount.masterHiPrioErr.txbAnt1TxZero += masterHiPrioErr.txbAnt1TxZero;        if (masterHiPrioErr.txbAnt1TxZero)  hDfe->dfeEeCount.eeFlag = 1;
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_HIPRI_INTRGRP_STATUS, &(masterHiPrioErr)) );

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_getException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Get Dfe Errors and Alarms status and clear */
DFE_Err Dfe_getException(
        DFE_Handle  hDfe
)
{
    DfeFl_Status status;
	DfeFl_MiscMasterLowPriIntrGroup     eeMasterLowOrigin;
	uint32_t                    i;

	VALID_DFE_HANDLE(hDfe);

    CSL_HW_QUERY( dfeFl_MiscGetHwStatus(hDfe->hDfeMisc[0], DFE_FL_MISC_QUERY_GET_MASTER_LOWPRI_INTRGRP_STATUS, &eeMasterLowOrigin) );

    /* BB errors */
    if (eeMasterLowOrigin.bb) {
        Dfe_getBbException(hDfe);
    }

    /* BB tx pm errors */
    // These are actually status bits and should not be handled by exception handler
	//if (eeMasterLowOrigin.bbtxPowmtr) {
		//Dfe_getBbTxPmException(hDfe);
	//}

    /* BB rx pm errors */
    // These are actually status bits and should not be handled by exception handler
	//if (eeMasterLowOrigin.bbrxPowmtr) {
		//Dfe_getBbRxPmException(hDfe);
	//}

    /* BB tx Gain update errors */
    // These are actually status bits and should not be handled by exception handler
	//if (eeMasterLowOrigin.bbtxGainUpt) {
		//Dfe_getBbTxGainException(hDfe);
	//}

    /* BB rx Gain update errors */
    // These are actually status bits and should not be handled by exception handler
	//if (eeMasterLowOrigin.bbrxGainUpt) {
		//Dfe_getBbRxGainException(hDfe);
	//}

    /* DDUC errors */
    for (i=0;i<4;i++)
	{
		if (eeMasterLowOrigin.dduc[i]) {
			Dfe_getDducException(hDfe,i);
		}
	}

    /* CFR errors */
    for (i=0;i<2;i++)
   	{
		if (eeMasterLowOrigin.cfr[i]) {
			Dfe_getCfrException(hDfe,i);
		}
   	}

    /* TX errors */
    if (eeMasterLowOrigin.tx) {
        Dfe_getTxException(hDfe);
    }

    /* JESD errors */
	if (eeMasterLowOrigin.jesd) {
		Dfe_getJesdException(hDfe);
   	}

    /* DPDA errors */
	if (eeMasterLowOrigin.dpda) {
		Dfe_getDpdaException(hDfe);
   	}

    /* RXIBPM errors */
    // These are actually status bits and should not be handled by exception handler
    //if (eeMasterLowOrigin.rxIbpm)
    //{
    //    for (i=0;i<4;i++)
    //    {
    //        Dfe_getRxException(hDfe, i);
    //    }
    //}

    /* CB done interrupts */
    if (eeMasterLowOrigin.cb) {
        Dfe_getCbException(hDfe);
    }

    /* MISC errors */
	if (eeMasterLowOrigin.misc) {
		Dfe_getMiscException(hDfe);
   	}

    /* High MISC errors */
	Dfe_getHiMiscException(hDfe);

    return DFE_ERR_NONE;
}

/** ============================================================================
 *   @n@b Dfe_enableException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Enable  DFE Errors and Alarms */
DFE_Err Dfe_enableException(
        DFE_Handle  hDfe
)
{
#ifdef _TMS320C6X
uint32_t gie;
   gie = _disable_interrupts();
#endif

   VALID_DFE_HANDLE(hDfe);

   reset_ExceptionStats(hDfe);

   /* BB errors */
   Dfe_enableBbException(hDfe);
   /* BB Tx power meter errors */
   //Dfe_enableBbTxPmException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* BB Rx power meter errors */
   //Dfe_enableBbRxPmException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* BB Tx gain update errors */
   //Dfe_enableBbTxGainException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* BB Rx gain update errors */
   //Dfe_enableBbRxGainException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* DDUC errors */
   Dfe_enableDducException(hDfe);
   /* CFR errors */
   Dfe_enableCfrException(hDfe);
   /* TX errors */
   Dfe_enableTxException(hDfe);
   /* JESD errors */
   Dfe_enableJesdException(hDfe);
   /* DPDA errors */
   Dfe_enableDpdaException(hDfe);
   /* RX IBPM errors */
   //Dfe_enableRxException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* CB capture done interrupts */
   Dfe_enableCbException(hDfe);
   /* Misc errors */
   Dfe_enableMiscException(hDfe);
   /* High Misc errors */
   Dfe_enableHiMiscException(hDfe);

#ifdef _TMS320C6X
     _restore_interrupts(gie);
#endif

     return DFE_ERR_NONE;
 }

/** ============================================================================
 *   @n@b Dfe_disableException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Disable  DFE Errors and Alarms */
DFE_Err Dfe_disableException(
        DFE_Handle  hDfe
)
{
#ifdef _TMS320C6X
uint32_t gie;
   gie = _disable_interrupts();
#endif

   VALID_DFE_HANDLE(hDfe);

   /* BB errors */
   Dfe_disableBbException(hDfe);
   /* BB Tx power meter errors */
   //Dfe_disableBbTxPmException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* BB Rx power meter errors */
   //Dfe_disableBbRxPmException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* BB Tx gain update errors */
   //Dfe_disableBbTxGainException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* BB Rx gain update errors */
   //Dfe_disableBbRxGainException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* DDUC errors */
   Dfe_disableDducException(hDfe);
   /* CFR errors */
   Dfe_disableCfrException(hDfe);
   /* TX errors */
   Dfe_disableTxException(hDfe);
   /* JESD errors */
   Dfe_disableJesdException(hDfe);
   /* DPDA errors */
   Dfe_disableDpdaException(hDfe);
   /* RX IBPM errors */
   //Dfe_disableRxException(hDfe);  // These are actually status bits and should not be handled by exception handler
   /* CB capture done interrupts - keep it on */
   //Dfe_disableCbException(hDfe);
   /* Misc errors */
   Dfe_disableMiscException(hDfe);
   /* High Misc errors */
   Dfe_disableHiMiscException(hDfe);

#ifdef _TMS320C6X
     _restore_interrupts(gie);
#endif

     return DFE_ERR_NONE;
 }

/** ============================================================================
 *   @n@b Dfe_printException
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]

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
/* Print Dfe Errors and Alarms status and clear */
void Dfe_printException(
        DFE_Handle  hDfe
)
{
#ifdef _TMS320C6X
   uint32_t gie;
   gie = _disable_interrupts();
#endif

    //VALID_DFE_HANDLE(hDfe);

    if (hDfe->dfeEeCount.eeFlag == 1) Dfe_osalLog("\n######### DFE ERRORS ###########\n");
/*  if (eeDbIntCnt.db_ee_i_trc_ram_ovfl_err)
        Iqn2_osalLog("DB:%d Data Trace RAM overflowed. This is not a fatal error because it only affects the Data Trace RAM\n",eeDbIntCnt.db_ee_i_trc_ram_ovfl_err);
*/
    if (hDfe->dfeEeCount.eeFlag == 1) Dfe_osalLog("\n###############################\n");

#ifdef _TMS320C6X
     _restore_interrupts(gie);
#endif
}


