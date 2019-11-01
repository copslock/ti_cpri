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
#include <ti/drv/dfe/dfe_drv.h>
#include <ti/drv/dfe/dfe_osal.h>
#include <ti/drv/dfe/dfe_internal.h>

/**
 * @defgroup DFE_LLD_SYNC_FUNCTION SYNC
 * @ingroup DFE_LLD_FUNCTION
 */

/**
 * @brief Issue a sync signal.
 * @ingroup DFE_LLD_SYNC_FUNCTION
 *
 * Once the signal is issued:
 * 
 * - If waitCnt is #DFE_FL_MISC_SYNC_NOWAIT, the function just does issue the sync
 *   and returns #DFE_ERR_NONE immediately. #Dfe_getSyncStatus() can be called later
 *   to check if the sync has come.
 * - If waitCnt is #DFE_FL_MISC_SYNC_WAITFOREVER, the function waits until the
 *   signal has really come and then returns #DFE_ERR_NONE.
 * - If waitCnt is #DFE_FL_MISC_SYNC_WAIT(n), 0 < n < 0xfffffffu, the function waits until 
 *    - either the signal has come before n loops complete, returns #DFE_ERR_NONE;
 *    - or n loops complete but the signal not come, returns #DFE_ERR_SYNC_NOT_COME
 *
 * For performance and flexibility considerations, using #DFE_FL_MISC_SYNC_NOWAIT
 * for waitCnt is recommended.  
 *
 * @param hDfe	[in] DFE device handle
 * @param syncSig	[in] sync signal to be issued
 * @param waitCnt	[in] wait loop count
 *
 * @return
 * - #DFE_ERR_NONE, if issue sync done properly
 * - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 * - #DFE_ERR_SYNC_NOT_COME, if sync signal not coming within waitCnt loops.
 * - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 * - hDfe should be a valid handle opened by #Dfe_open().
 * - DFE PLL and PSCs shall be already up running.
 * - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_issueSync
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig syncSig,
    uint32_t waitCnt
)
{
    DfeFl_Status status;
	DfeFl_MiscSyncGenIssueSyncConfig cfg;

    VALID_DFE_HANDLE(hDfe);
    
	cfg.syncSig = syncSig;
	cfg.waitCnt = waitCnt;

    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_ISSUE_SYNC, &cfg) );
	
	return DFE_ERR_NONE;  
}

/**
 * @brief Get a sync signal status.
 * @ingroup DFE_LLD_SYNC_FUNCTION
 *
 * Get status of a sync signal. When return, if *signaled is 1, the sync has come; if *signaled is 0, the sync hasn't come.
 *
 * @param hDfe	[in] DFE device handle
 * @param syncSig	[in] sync signal to be issued
 * @param signalled	[out] pointer to signal status buffer
 *
 * @return
 * - #DFE_ERR_NONE, if issue sync done properly
 * - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 * - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
 *
 * @pre
 * - hDfe should be a valid handle opened by #Dfe_open().
 * - DFE PLL and PSCs shall be already up running.
 * - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_getSyncStatus
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig syncSig,
    uint32_t *signaled
)
{
    DfeFl_Status status;
    DfeFl_MiscIntrStatus intrSts;
    
    VALID_DFE_HANDLE(hDfe);

    if(signaled == NULL)
    {
        Dfe_osalLog("NULL pointers passed in!");
        return DFE_ERR_INVALID_PARAMS;
    }

    *signaled = 0;
    
    intrSts.intr = syncSig;
    intrSts.data = 0;
    CSL_HW_QUERY( dfeFl_MiscGetHwStatus(hDfe->hDfeMisc[0], DFE_FL_MISC_QUERY_GET_SYNC_INTR_STATUS, &intrSts) );

    *signaled = intrSts.data;
    return DFE_ERR_NONE;
}

/**
 * @brief Program a sync counter.
 * @ingroup DFE_LLD_SYNC_FUNCTION
 *
 * The API first resets the counter and then reprogram to specified parameters. 
 * NOTE: #Dfe_issueSyncStartSyncCounter() should be then called to start the counter.
 *
 * @param hDfe	[in] DFE device handle
 * @param cntr	[in] sync counter number
 * @param delay	     [in] number of clocks to wait after sync select source before sending
                     initial sync. If set to 0, sync counter output will be high if sync
				     select source is high
 * @param period	 [in] number of clocks to wait between syncs when repeat is 1.
 *                   Does nothing when repeat is 0.
 * @param pulseWidth [in] set to X for pulse width of X clocks; 0 means it will never go high.
 * @param repeat	 [in] If 0, counter counts down delay clocks once, sends a sync,
 *                   and stops. If 1, it counts down delay clocks sends a sync,
 *                   then continuously sends more syncs every period clocks.
 * @param invert	 [in] set to 1 to invert entire bus
 *
 * @return
 * - #DFE_ERR_NONE, if sync counter programmed properly
 * - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 * - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 * - hDfe should be a valid handle opened by #Dfe_open().
 * - DFE PLL and PSCs shall be already up running.
 * - DFE has loaded target config and completed initialize sequence.
 * - #Dfe_issueSyncStartSyncCounter() should be called later to issue sync to start the counter.
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progSyncCounter
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenCntr cntr,
    uint32_t delay,
    uint32_t period,
    uint32_t pulseWidth,
    uint32_t repeat,
    uint32_t invert
)
{
    DfeFl_Status status;
	DfeFl_MiscSyncCntrConfig syncCntrCfg;
    
    VALID_DFE_HANDLE(hDfe);

    // reset cntr
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_RST_SYNC_CNTR, &cntr) );
	
	// config cntr
	syncCntrCfg.cntr = cntr;
	syncCntrCfg.repeat = repeat;
	syncCntrCfg.delay = delay;
	syncCntrCfg.invert = invert;
	syncCntrCfg.period = period;
	syncCntrCfg.pulse = pulseWidth;
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CFG_SYNC_CNTR, &syncCntrCfg) );

    return DFE_ERR_NONE;
}

/**
 * @brief Issue sync to start the sync counter, and return without waiting.
 * @ingroup DFE_LLD_SYNC_FUNCTION
 *
 * Issue sync to start the sync counter that has been previously programmed with #Dfe_progSyncCounter().
 *
 * It programs the counter's starting sync select with ssel (using ALWAYS sync signal) and returns
 * immediately after issue the sync. So #Dfe_getSyncStatus() should be called later to check if
 * the sync has come.
 *
 * @param hDfe	[in] DFE device handle
 * @param cntr	[in] sync counter number
 * @param ssel	[in] sync select to re-start sync counter
 *
 * @return
 * - #DFE_ERR_NONE, if API complete properly
 * - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 * - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 * - hDfe should be a valid handle opened by #Dfe_open().
 * - DFE PLL and PSCs shall be already up running.
 * - DFE has loaded target config and completed initialize sequence. 
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_issueSyncStartSyncCounter
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenCntr cntr,
    DfeFl_MiscSyncGenSig ssel
)
{
    DfeFl_Status status;
    DfeFl_MiscSyncGenCntrSsel syncCntrSsel;
    
    VALID_DFE_HANDLE(hDfe);
    
    // commit to hardware by ALWAYS sync, on then off
    syncCntrSsel.cntr = cntr;
	syncCntrSsel.startSsel = ssel;
    syncCntrSsel.progSsel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_SET_SYNC_CNTR_SSEL, &syncCntrSsel) );	

	Dfe_issueSync(hDfe, DFE_FL_SYNC_GEN_SIG_ALWAYS, DFE_FL_MISC_SYNC_NOWAIT);

	if(ssel == DFE_FL_SYNC_GEN_SIG_NEVER)
	{
		syncCntrSsel.progSsel = DFE_FL_SYNC_GEN_SIG_NEVER;
		CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_SET_SYNC_CNTR_SSEL, &syncCntrSsel) );
	}

	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

