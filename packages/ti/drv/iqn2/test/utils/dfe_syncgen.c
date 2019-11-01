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

#include <ti/csl/csl.h>
//#include <val_util.h>
#include <constants.h>
#include <ti/csl/csl_chip.h>

#include "dfe_inc.h"

int issueSync(Uint32 syncSig, Uint32 wait)
{
	//CSL_DfeMiscSyncGenIssueSyncConfig cfg;
    DfeFl_MiscSyncGenIssueSyncConfig cfg;

	cfg.syncSig = syncSig;
	cfg.waitCnt = wait;

	//if(CSL_dfeMiscHwControl(hDfeMisc[0], CSL_DFE_MISC_CMD_ISSUE_SYNC, &cfg) != CSL_SOK)
	if(dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_ISSUE_SYNC, &cfg) != DFE_FL_SOK)
	{
		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_WAIT_SYNC);
		return FAIL;
	}
	
	return PASS;  
}

void resetSyncCntr(Uint32 cntrInst)
{
    // reset cntr
    Uint32 cntr = cntrInst;
	//CSL_dfeMiscHwControl(hDfeMisc[0], CSL_DFE_MISC_CMD_RST_SYNC_CNTR, &cntr);
    dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_RST_SYNC_CNTR, &cntr);
}

void activeSyncCntr(Uint32 cntrInst, Uint32 startSsel, Uint32 progSsel)
{
    //CSL_DfeMiscSyncGenCntrSsel syncCntrSsel;
    DfeFl_MiscSyncGenCntrSsel syncCntrSsel;

    // select syncs
    syncCntrSsel.cntr = cntrInst;
	syncCntrSsel.startSsel = startSsel;
    syncCntrSsel.progSsel = progSsel;
	//dfeFl_MiscHwControl(hDfeMisc[0], CSL_DFE_MISC_CMD_SET_SYNC_CNTR_SSEL, &syncCntrSsel);
    dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_SET_SYNC_CNTR_SSEL, &syncCntrSsel);
	
	// issue prog sync
	issueSync(progSsel, DFE_FL_MISC_SYNC_WAITFOREVER);
	// issue start sync
	issueSync(startSsel, DFE_FL_MISC_SYNC_WAITFOREVER);

	//
    // de-select prog sync
    syncCntrSsel.cntr = cntrInst;
    syncCntrSsel.progSsel = DFE_FL_SYNC_GEN_SIG_NEVER;
	//CSL_dfeMiscHwControl(hDfeMisc[0], CSL_DFE_MISC_CMD_SET_SYNC_CNTR_SSEL, &syncCntrSsel);
    dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_SET_SYNC_CNTR_SSEL, &syncCntrSsel);
}

void progSyncCntr(Uint32 cntrInst, Uint32 dly, Uint32 period, Uint32 pulseWidth)
{
	//CSL_DfeMiscSyncCntrConfig syncCntrCfg;
	DfeFl_MiscSyncCntrConfig syncCntrCfg;

	// config cntr
	syncCntrCfg.cntr = cntrInst;
	syncCntrCfg.repeat = 1;
	syncCntrCfg.delay = dly;
	syncCntrCfg.invert = 0;
	syncCntrCfg.period = period;
	syncCntrCfg.pulse = pulseWidth;
	//CSL_dfeMiscHwControl(hDfeMisc[0], CSL_DFE_MISC_CMD_CFG_SYNC_CNTR, &syncCntrCfg);
	dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_CFG_SYNC_CNTR, &syncCntrCfg);
	
}

void setSyncPulseWidth(Uint32 syncSig, Uint32 pulseWidth)
{
    //CSL_DfeMiscSyncGenGeneric syncGeneric;
    DfeFl_MiscSyncGenGeneric syncGeneric;

	syncGeneric.syncSig = syncSig;
	syncGeneric.data    = pulseWidth;
	//CSL_dfeMiscHwControl(hDfeMisc[0], CSL_DFE_MISC_CMD_CFG_SYNC_ONE_SHOT, &syncGeneric);
	dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_CFG_SYNC_ONE_SHOT, &syncGeneric);
}
