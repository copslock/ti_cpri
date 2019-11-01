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
#include <ti/drv/dfe/dfe_drv.h>
#include <ti/drv/dfe/dfe_osal.h>
#include <ti/drv/dfe/dfe_internal.h>

/**
 * @defgroup DFE_LLD_DEVICE_FUNCTION DEVICE
 * @ingroup DFE_LLD_FUNCTION
 */

// LLD software version
#define DFE_LLD_VERSION     0x00000001u

#define MK_INITS(pInits, _ssel, _initClkGate, _initState, _clearData) \
do { \
    (pInits)->ssel = (_ssel); \
    (pInits)->initClkGate = (_initClkGate); \
    (pInits)->initState = (_initState); \
    (pInits)->clearData = (_clearData); \
} while(0);

#define MK_INITS_JESD(pInits, _ssel, _initClkGate, _initState, _clearData) \
do { \
    (pInits)->cmn.ssel = (_ssel); \
    (pInits)->cmn.initClkGate = (_initClkGate); \
    (pInits)->cmn.initState = (_initState); \
    (pInits)->cmn.clearData = (_clearData); \
    (pInits)->clearDataLane[0] = (_clearData); \
    (pInits)->clearDataLane[1] = (_clearData); \
    (pInits)->clearDataLane[2] = (_clearData); \
    (pInits)->clearDataLane[3] = (_clearData); \
} while(0);

#define ISSUE_WAIT_SYNC(ssel) \
do { \
    if(dfeErr == DFE_ERR_NONE) dfeErr = Dfe_issueSync(hDfe, (ssel), DFE_FL_MISC_SYNC_NOWAIT); \
    if(dfeErr == DFE_ERR_NONE) dfeErr = waitSyncFxn(waitSyncCtx, hDfe, (ssel)); \
    if(dfeErr == DFE_ERR_TIMEDOUT) return DFE_ERR_SYNC_NOT_COME; else if(dfeErr != DFE_ERR_NONE) return dfeErr; \
} while(0);       

// =============================== Local Prototypes ===============================
static DFE_Err setSyncPulseWidth(DFE_Handle hDfe, DfeFl_MiscSyncGenSig syncSig, uint32_t pulseWidth);

// =============================== Global Functions ===============================
/**
 * @brief Load DFE target configuration, i.e. registers contents.
 * @ingroup DFE_LLD_DEVICE_FUNCTION
 *
 * Write target configuration i.e. registers contents, to DFE hardware.
 * The target config format is defined as, (addr, data) register pairs.
 *
 * The content of the registers to be written is specified in an array of type
 * #DFE_RegPair[].
 * 
 * The last entity of tgtCfgPairs must be (0xffffffffu, 0xffffffffu), which
 * is the end marker.
 *
 * @param hDfe        DFE device handle
 * @param tgtCfgPairs [in] pointer to (addr, adta) pairs buffer
 *
 * @return
 *  - #DFE_ERR_NONE, if write to DFE properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *
 * @pre
 *  - hDfe should be a valid handle opened by #Dfe_open().
 *  - The last entity of tgtCfgPairs must be (0xffffffffu, 0xffffffffu)
 *  - DFE PLL and PSCs shall be already up running.
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_loadTgtCfg
(
    DFE_Handle hDfe,
    DFE_RegPair tgtCfgPairs[]
)
{
    uint32_t ui, addr;
    
    VALID_DFE_HANDLE(hDfe);
    
    for(ui = 0; tgtCfgPairs[ui].addr != 0xffffffffu; ui++)
    {
        if (((hDfe->dpdIsDisabled == 1) || (hDfe->dpdaIsDisabled == 1)) &&
            ((tgtCfgPairs[ui].addr >= DFE_FL_DPD_0_OFFSET) && (tgtCfgPairs[ui].addr < DFE_FL_TX_0_OFFSET)))
        {
            // do nothing
        } else
        {
            addr = tgtCfgPairs[ui].addr >> 2;
            hDfe->hDfe->regs[addr] = tgtCfgPairs[ui].data;
        }
    }
    
    return DFE_ERR_NONE;
}

/**
 * @brief Soft reset DFE peripheral.
 * @ingroup DFE_LLD_DEVICE_FUNCTION
 *
 * The API does a software reset for DFE peripheral. Like power up reset, soft
 * reset does assert init_clk_gate, init_state, and clear_data to all sub-blocks
 * in DFE. On the other hand unlike power up reset, soft reset keeps all registers
 * values unchanged, except for init register in each block.
 *
 * Soft reset should be done after #Dfe_loadTgtCfg() but before #Dfe_initTgtTx(),
 * and #Dfe_initTgtRx().
 *
 * @param hDfe     DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if soft reset properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *   - hDfe should be a valid handle opened by #Dfe_open().
 *   - DFE PLL and PSCs shall be already up running.
 *   - #Dfe_loadTgtCfg() already called.
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_softReset(DFE_Handle hDfe)
{
    uint32_t ui;
    DfeFl_Status status;
    DfeFl_SublkInitsConfig inits;
    DfeFl_JesdInitsConfig jesdInits;
    
    VALID_DFE_HANDLE(hDfe);
    
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_ALWAYS, 1, 1, 1);
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_ALWAYS, 1, 1, 1);
    
    // reset BB
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits) );
    
    // reset DDUCs
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits) );
    }
    // reset CFRs
    for(ui = 0; ui < DFE_FL_CFR_PER_CNT; ui++)
    {
        CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[ui], DFE_FL_CFR_CMD_CFG_INITS, &inits) );
    }
    // reset AUTOCP
    CSL_HW_CTRL( dfeFl_AutocpHwControl(hDfe->hDfeAutocp[0], DFE_FL_AUTOCP_CMD_CFG_INITS, &inits) );
    // reset CDFR
    CSL_HW_CTRL( dfeFl_CdfrHwControl(hDfe->hDfeCdfr[0], DFE_FL_CDFR_CMD_CFG_INITS, &inits) );
    if (hDfe->dpdIsDisabled == 0)
    {
        // reset DPD
        CSL_HW_CTRL( dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits) );
    }
    if (hDfe->dpdaIsDisabled == 0)
    {
        // reset DPDA
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits) );
    }
    // reset TX
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits) );
    // reset RX
    CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits) );
    // reset CB
    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits) );
    // reset JESDTX
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits) );
    // reset JESDRX
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits) );
    // reset FB
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits) );
    // reset MISC
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits) );
    
    return DFE_ERR_NONE;
}

/**
 * @brief DFE initialization sequence for transmit path.
 * @ingroup DFE_LLD_DEVICE_FUNCTION
 *
 * DFE initialization sequence for tx path, BB => DDUC => CFR => DPD => TX => JESDTX
 *
 * The API runs initialization sequence for transmit path. 
 *
 * When JESD Tx lanes configured connecting to SERDES, this initialization shouldn't be
 * made until SERDES Tx lock (pll_ok).
 *
 * In the sequence, there are many steps that need to wait on a sync signal. This is done by a
 * using callback function waitSyncFxn() that has to be of type #DFE_CallbackWaitSync.
 *
 * When complete with #DFE_ERR_NONE, DFE transmit path is ready working, otherwise
 * the DFE peripheral is in unknown condition. 
 *
 *  @param hDfe	[in] DFE device handle
 *  @param waitSyncCtx	[in] callback context pass to waitSyncFxn
 *  @param waitSyncFxn	[in] callback function for waiting a sync signal
 *
 * @return
 * - #DFE_ERR_NONE, if initialization properly
 * - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 * - #DFE_ERR_SYNC_NOT_COME, if sync signal not come
 * - #DFE_ERR_CALLBACKFXN_IS_NULL, if callback function required but given a NULL pointer
 * - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 * - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
 *
 * @pre
 * - hDfe should be a valid handle opened by #Dfe_open().
 * - DFE PLL and PSCs shall be already up running.
 * - #Dfe_softReset() already called.
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
// use downlink frame sync to trigger sync counter0, use sync counter 0 for dfe tx path (except for jesd) initial steps
DfeFl_MiscSyncGenSig sync_cnt0_ssel = DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0;
// use sysref to trigger sync counter1, use sync counter 1 for jesd tx and jesd rx initial steps
DfeFl_MiscSyncGenSig sync_cnt1_ssel = DFE_FL_SYNC_GEN_SIG_SYSREF;
// use uplink frame sync to trigger sync counter2, use sync counter 2 for dfe rx path (except for jesd) initial steps
DfeFl_MiscSyncGenSig sync_cnt2_ssel = DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0;
 
DFE_Err Dfe_initTgtTx
(
    DFE_Handle hDfe,
    DFE_CallbackContext waitSyncCtx,
    DFE_CallbackWaitSync waitSyncFxn
)
{
    DFE_Err dfeErr = DFE_ERR_NONE;
    uint32_t ui, data, period;
    DfeFl_Status status;
    uint32_t dducDir[DFE_FL_DDUC_PER_CNT];
    DfeFl_SublkInitsConfig inits;
    DfeFl_JesdInitsConfig jesdInits;
    DfeFl_BbCarrierTypeUlSyncStrobeConfig ulStrobe;
    uint32_t bbtx_siggen_ssel, bbrx_chksum_ssel;
//    uint32_t txSysrefMode[2], rxSysrefMode[2];
//    DfeFl_JesdLinkSysrefModeConfig sysrefModeQry;
    DfeFl_DducMixNcoSsel mixNcoSsel;
    DfeFl_CfrMultSsel cfrMultSsel;
    DfeFl_BbTestGenSsel BbTestGenSsel;
    DfeFl_BbChksumSsel BbChksumSsel;
    
    VALID_DFE_HANDLE(hDfe);
    
    if(waitSyncFxn == NULL)
    {
        Dfe_osalLog("callback function pointer is NULL!");
        return DFE_ERR_CALLBACKFXN_IS_NULL;
    }
    
    //Release mem_mpu_access for all blocks
    data = DFE_FL_MEM_MPU_ACCESS_ALL;
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MEM_MPU_ACCESS, &data) );
    
    //Be sure the bb's frame start logic is set to never sync (it must not be synced until
    //the end of initialization). It will be put on the appropriate sync at the end of
    //initialization.
    ulStrobe.ct = DFE_FL_BB_CARRIER_TYPE_ALL;
    ulStrobe.strobe = DFE_FL_SYNC_GEN_SIG_NEVER;
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_CT_UL_SYNC_STROBE, &ulStrobe) );
        
    //determine which dducs are in tx and which are rx.
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        CSL_HW_QUERY( dfeFl_DducGetHwStatus(hDfe->hDfeDduc[ui], DFE_FL_DDUC_QUERY_DIRECT, &dducDir[ui]) );
    }
    
    BbTestGenSsel.tgDev = DFE_FL_BB_AID_TESTGEN_A;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TESTGEN_SSEL, &BbTestGenSsel) );
    bbtx_siggen_ssel = BbTestGenSsel.ssel;
    BbChksumSsel.chksumDev = DFE_FL_BB_AID_CHKSUM_A;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_CHKSUM_SSEL, &BbChksumSsel) );
    bbrx_chksum_ssel = BbChksumSsel.ssel;

    hDfe->bbtx_siggen_ssel = bbtx_siggen_ssel;
    hDfe->bbrx_chksum_ssel = bbrx_chksum_ssel;

//    //Store the sysref_modes and the sysref_internal_sel
//    for(ui = 0; ui <= DFE_FL_JESD_LINK_1; ui++)
//    {
//        sysrefModeQry.link = ui;
//
//        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_SYSREF_MODE, &sysrefModeQry) );
//        txSysrefMode[ui] = sysrefModeQry.mode;
//
//        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_SYSREF_MODE, &sysrefModeQry) );
//        rxSysrefMode[ui] = sysrefModeQry.mode;
//    }
            
    //All syncing will be aligned to the sysref signal, so that it all gets aligned
    //to the SERDES/JESD fifos (note that this will NOT align to the IQN/BB fifos, but
    //there is no way to align to both...this means there is still one source of latency
    //variability).
    //To do this, we will use the sysref sync to kick-off a repeating sync counter (ssel
    //12), and use that sync counter to sync everything.
    //The exception is if the sysref_mode is set to 0 in the jesd, which means jesd is not
    //using sysref anyways, in which case the jesd fifos would not align anyways, so just
    //use mpu always sync (this will not be deterministic unless in loopback mode, but it
    //is useful if sysref is not coming to the system).
    //Note that the sysref should already be coming at this time because the
    //sysref_request (from the JESD) is high on power-up. If the DSP is responsible for
    //starting sysref, then it can already see the sysref_request (via our standard
    //interrupt interface). If the CDC takes sysref_request directly from the GPIO, then
    //it also can already see the sysref_request because it is asserted and the .reg file
    //has been loaded (the .reg file should properly set up the gpio mapper to send the
    //sysref_request out the proper GPIO pin).
    //Finally, note to make it deterministic even from power-up to power-up and across
    //multiple Lamarrs, the sync counter must switch back to never at a deterministic time
    //using an LVDS sync. This is NOT being done for now...
    //First, set the sysref sync to be a one-shot (to detect its rising edges).
    dfeErr = setSyncPulseWidth(hDfe, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, 1);
    if(dfeErr != DFE_ERR_NONE)
        return dfeErr;
    dfeErr = setSyncPulseWidth(hDfe, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, 1);
    if(dfeErr != DFE_ERR_NONE)
        return dfeErr;
    dfeErr = setSyncPulseWidth(hDfe, DFE_FL_SYNC_GEN_SIG_SYSREF, 1);
    if(dfeErr != DFE_ERR_NONE)
        return dfeErr;
    dfeErr = setSyncPulseWidth(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 1);
    if(dfeErr != DFE_ERR_NONE)
        return dfeErr;

    //DFE Init sequence use 3 sync counters
    //setup sync counter0, use DL IQN frame sync to trigger sync counter 0, use sync counter 0 for DFE downlink init steps(except for jesd)

    //Reset the sync counter by setting the period to 0. Then, configure the counter
    //sync to listen to sysref. As sysref keeps coming, the counter keeps resetting.
    //the length of this counter must meet these requirements:
    //must be a multiple of sysref (note that clk/1024 is the default rate for sysref
    //because it should work for just about any config) and must be long enough
    //that mpu reads/writes deterministically fit within one count period (note mpu
    //reads/writes will take longer on the real system than on TB). Simply using
    //1024 for now (which is 100% fine for TB).
    //Exception:  if the sysref_mode is 0, then just use always sync.
    //The SYSREF is 122.88Mhz/1024, 120Khz.  The problem with the period is that it hides what is being done.
    //The software synchronization rate is picked to be statistically long enough to capture a sysref, which is at least 2x the sysref period.
    //We then turn off the sysref sync and let the sync counter0 free run, to create a software tick.
    //SYSREF period 1024/122.88e6 � this is what we use today it should be a parameter  8.33usec
    //Synccounter 0 period for statistical capture is 2x the SYSREF period 16.66usec.
    //There is another fudge factor of 2x.
    //So the 245.76 Clock and x2000 result in ticks of 33.33us
    //So the 368.64           x3000
    //synccounter0 = ceil((SYSREFcount/SYSCLK)*xfactor*selected_DFEclk)
    //This DFE procedure ASSUMES that no other software is taking over, it is explicitly used to transfer the 3 states (OFF->InitClkGate->InitState->ClearDatarelease) to the 12 DFE sub blocks
    // xfactor > 4 could be a better choice for a more robust system.  4 indicates that DFE routines will complete there register tasks in 33.33usec.
    if (hDfe->dfeSpeed == DFE_SPEED_245_76)
    {
        hDfe->dfeCustomSpeed = 245760;
    }
    if (hDfe->dfeSpeed == DFE_SPEED_368_64)
    {
        hDfe->dfeCustomSpeed = 368640;
    }
    // compute the period with a xfactor + fudge factor of 4, round up
    period  = (1024 * hDfe->dfeCustomSpeed * 4)  / (122880);
    period += (((1024 * hDfe->dfeCustomSpeed * 4) % 122880) != 0);

	{
		dfeErr = Dfe_progSyncCounter(hDfe,
				DFE_FL_SYNC_GEN_CNTR0,
				0, // delay
				period, // period
				1, // pulseWidth
				1, // repeat
				0); // invert

		if(dfeErr != DFE_ERR_NONE)
		        return dfeErr;

	    if(bbtx_siggen_ssel == 0)
	    {
            dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
                        DFE_FL_SYNC_GEN_CNTR0,
                        sync_cnt0_ssel);

            ISSUE_WAIT_SYNC(sync_cnt0_ssel);

	    }
	    else
	    {
	        dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
	                    DFE_FL_SYNC_GEN_CNTR0,
	                    DFE_FL_SYNC_GEN_SIG_ALWAYS);
	    }
	    if(dfeErr != DFE_ERR_NONE)
	        return dfeErr;

	    //Now, set the counter back to never. This releases it to begin counting, and the last
	    //sysref that came will be the one that everything syncs off of.
	    //***Here, if we use a deterministic LVDS sync to update the sync counter's ssel from
	    // shadow to active, we can align the DFE state machines across multiple Lamarrs
	    // IF it is a one-shot sync. Not doing this now, though...just using mpu always sync
	    // Also note that even mpu sync might be fine if the sysref rate is low enough (slow
	    dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
	                DFE_FL_SYNC_GEN_CNTR0,
	                DFE_FL_SYNC_GEN_SIG_NEVER);

	    if(dfeErr != DFE_ERR_NONE)
	        return dfeErr;
	    // enough to deterministically get in the mpu write before the next sysref arrives).

	    //Counter has begun counting. Clear the counter sync detect and set sync_poll vseq to
	    //use the counter.
	    //Align to the first counter period. Everything is aligned after this.
	    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
	}

    /* setup sync counter1, use sysref to trigger sync counter 1, use sync counter 1 for jesd init steps */
    {
        dfeErr = Dfe_progSyncCounter(hDfe,
                DFE_FL_SYNC_GEN_CNTR1,
                0, // delay
                period, // period
                4, // pulseWidth stretch the sync counter #1 output pulse to 4 clock cycles
                1, // repeat
                0); // invert

        if(dfeErr != DFE_ERR_NONE)
                return dfeErr;

        if(bbtx_siggen_ssel == 0)
        {
            dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
                        DFE_FL_SYNC_GEN_CNTR1,
                        sync_cnt1_ssel);

            ISSUE_WAIT_SYNC(sync_cnt1_ssel);
        }
        else
        {
            dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
                        DFE_FL_SYNC_GEN_CNTR1,
                        DFE_FL_SYNC_GEN_SIG_ALWAYS);
        }

        if(dfeErr != DFE_ERR_NONE)
            return dfeErr;

        dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
                    DFE_FL_SYNC_GEN_CNTR1,
                    DFE_FL_SYNC_GEN_SIG_NEVER);

        if(dfeErr != DFE_ERR_NONE)
            return dfeErr;

        ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1);
    }

    /* setup sync counter2, use UL IQN frame sync to trigger sync counter 2, use sync counter 2 for DFE uplink init steps(except for jesd) */
	{
		dfeErr = Dfe_progSyncCounter(hDfe,
				DFE_FL_SYNC_GEN_CNTR2,
				0, // delay
				period, // period
				1, // pulseWidth
				1, // repeat
				0); // invert

        if(dfeErr != DFE_ERR_NONE)
                return dfeErr;

        if(bbtx_siggen_ssel == 0)
        {
            dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
                        DFE_FL_SYNC_GEN_CNTR2,
                        sync_cnt2_ssel);

            ISSUE_WAIT_SYNC(sync_cnt2_ssel);
        }
        else
        {
            dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
                        DFE_FL_SYNC_GEN_CNTR2,
                        DFE_FL_SYNC_GEN_SIG_ALWAYS);
        }

        if(dfeErr != DFE_ERR_NONE)
            return dfeErr;

		dfeErr = Dfe_issueSyncStartSyncCounter(hDfe,
					DFE_FL_SYNC_GEN_CNTR2,
					DFE_FL_SYNC_GEN_SIG_NEVER);

        if(dfeErr != DFE_ERR_NONE)
            return dfeErr;

		ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
	}
   
    //Release each block in signal processing order. Each given block releases init_clk_gate,
    //then init_state, then clear_data, with delays between each release.
    //    
    //misc
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);    
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);

    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    
    //cb (being done first to support cb sourcing, so sourcing can begin before blocks are initialized)
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);    
    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);


    //bb (the bbtx and bbrx are both released here, which means bbrx will be released before
    //dducs send frames to it, but this is perfectly fine)
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);    
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);

    //tx dducs (dducs must be released at the same time; use mpu_sync)
    //
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);    
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits) );
        }
    }
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits) );
        }
    }
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // nco syncs
    mixNcoSsel.iMix = DFE_FL_DDUC_MIX_NCO_ALL;
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            mixNcoSsel.data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &mixNcoSsel) );
        }
    }
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // nco sync back to never
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            mixNcoSsel.data = DFE_FL_SYNC_GEN_SIG_NEVER;
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &mixNcoSsel) );
        }
    }
    //frw fifo takes time to clear, so wait before releasing clear_data
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits) );
        }
    }
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    
    //summer (does not have init_clk_gate)
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    CSL_HW_CTRL( dfeFl_SummerHwControl(hDfe->hDfeSummer[0], DFE_FL_SUMMER_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    CSL_HW_CTRL( dfeFl_SummerHwControl(hDfe->hDfeSummer[0], DFE_FL_SUMMER_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    

    //acl
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);    
    CSL_HW_CTRL( dfeFl_AutocpHwControl(hDfe->hDfeAutocp[0], DFE_FL_AUTOCP_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    CSL_HW_CTRL( dfeFl_AutocpHwControl(hDfe->hDfeAutocp[0], DFE_FL_AUTOCP_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    CSL_HW_CTRL( dfeFl_AutocpHwControl(hDfe->hDfeAutocp[0], DFE_FL_AUTOCP_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);

    //cfrs (both cfrs must be released at the same time;)
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);    
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[0], DFE_FL_CFR_CMD_CFG_INITS, &inits) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[1], DFE_FL_CFR_CMD_CFG_INITS, &inits) );
    // software wait 5ms in the CFR sections
    Dfe_osalSleep(5);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[0], DFE_FL_CFR_CMD_CFG_INITS, &inits) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[1], DFE_FL_CFR_CMD_CFG_INITS, &inits) );
    Dfe_osalSleep(5);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    //pre & post mult
    cfrMultSsel.path = DFE_FL_CFR_PATH_ALL;
    cfrMultSsel.ssel = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[0], DFE_FL_CFR_CMD_SET_PREM_SSEL, &cfrMultSsel) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[0], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &cfrMultSsel) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[1], DFE_FL_CFR_CMD_SET_PREM_SSEL, &cfrMultSsel) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[1], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &cfrMultSsel) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // back to never
    cfrMultSsel.ssel = DFE_FL_SYNC_GEN_SIG_NEVER;
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[0], DFE_FL_CFR_CMD_SET_PREM_SSEL, &cfrMultSsel) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[0], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &cfrMultSsel) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[1], DFE_FL_CFR_CMD_SET_PREM_SSEL, &cfrMultSsel) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[1], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &cfrMultSsel) );
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[0], DFE_FL_CFR_CMD_CFG_INITS, &inits) );
    CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[1], DFE_FL_CFR_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);

    //cdfr
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);    
    CSL_HW_CTRL( dfeFl_CdfrHwControl(hDfe->hDfeCdfr[0], DFE_FL_CDFR_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    CSL_HW_CTRL( dfeFl_CdfrHwControl(hDfe->hDfeCdfr[0], DFE_FL_CDFR_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    CSL_HW_CTRL( dfeFl_CdfrHwControl(hDfe->hDfeCdfr[0], DFE_FL_CDFR_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);

    if (hDfe->dpdaIsDisabled == 0)
    {
        //dpda
        // release clock gate
        MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits) );
        ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
        // release init state
        MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits) );
        ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
        // release clear data
        MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDfe->hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits) );
        ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    }
    if (hDfe->dpdIsDisabled == 0)
    {
        //dpd
        // release clock gate
        MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
        CSL_HW_CTRL( dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits) );
        ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
        // release init state
        MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
        CSL_HW_CTRL( dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits) );
        ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
        // release clear data
        MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
        CSL_HW_CTRL( dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits) );
        ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    }

    //tx
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);    
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);    
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);    
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);

    //jesdtx and jesdrx init_clk_gate. Need to release the jesdrx init_clk_gate simultaneous
    //with jesdtx init_clk_gate for alignment in certain funky jesd loopback configs. Since
    //it never HURTS any config to do this, just always do it (more robust).
    // release clock gate
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, 0, 1, 1);
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits) );
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits) );
    //software wait 1ms in the jesd tx sections
    Dfe_osalSleep(1);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1);
    // release init state
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, 0, 0, 1);
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits) );
    Dfe_osalSleep(1);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1);
    // release clear data
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, 0, 0, 0);
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits) );
    Dfe_osalSleep(1);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1);

    //Now, the DSP should determine which JESD lanes are enabled and not looping back, then
    //read from the SERDES until it sees that the stsrx[2] is high AND stsrx[1] is low for
    //those lanes. Again, we need to determine if this is gc_dfe_lamarr tb or
    //gc_dfe_iqn_lamarr tb.
    //***note that even this wait section here must finish within one sync counter period for 100% repeatability...
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);

    return DFE_ERR_NONE;    
}

/**
 * @brief DFE initialization sequence for receive path.
 * @ingroup DFE_LLD_DEVICE_FUNCTION
 *
 * DFE initialization sequence for tx path, BB <= DDUC <= RX/FB <= JESDRX
 *
 * The API runs initialization sequence for receive path. 
 *
 * When JESD Rx lanes configured connecting to SERDES, this initialization
 * shouldn't be made until SERDES Rx OK bit set and LOSS bit cleared.
 *
 * In the sequence, there are many steps that need to wait a sync signal. This is done
 * by using callback function waitSyncFxn() that has to be of type #DFE_CallbackWaitSync.
 *
 * When complete with #DFE_ERR_NONE, DFE receive path is ready working, otherwise
 * the DFE peripheral is in unknown condition. 
 *
 *  @param hDfe	[in] DFE device handle
 *  @param waitSyncCtx	[in] callback context pass to waitSyncFxn
 *  @param waitSyncFxn	[in] callback function for waiting a sync signal
 *
 * @return
 *  - #DFE_ERR_NONE, if initialization properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_SYNC_NOT_COME, if sync signal not come
 *  - #DFE_ERR_CALLBACKFXN_IS_NULL, if callback function required but given a NULL pointer
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *  - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
 *
 * @pre
 * - hDfe should be a valid handle opened by #Dfe_open().
 * - DFE PLL and PSCs shall be already up running.
 * - #Dfe_softReset() already called.
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_initTgtRx
(
    DFE_Handle hDfe,
    DFE_CallbackContext waitSyncCtx,
    DFE_CallbackWaitSync waitSyncFxn
)
{
    DFE_Err dfeErr = DFE_ERR_NONE;
    uint32_t ui, data;
    DfeFl_Status status;
    uint32_t dducDir[DFE_FL_DDUC_PER_CNT];
    DfeFl_SublkInitsConfig inits;
    DfeFl_JesdInitsConfig jesdInits;
    DfeFl_BbCarrierTypeUlSyncStrobeConfig ulStrobe;
    DfeFl_DducMixNcoSsel mixNcoSsel;
    DfeFl_RxDcGeneric dcGeneric;
    
    VALID_DFE_HANDLE(hDfe);
    
    if(waitSyncFxn == NULL)
    {
        Dfe_osalLog("callback function pointer is NULL!");
        return DFE_ERR_CALLBACKFXN_IS_NULL;
    }
    
    //determine which dducs are in tx and which are rx.
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        CSL_HW_QUERY( dfeFl_DducGetHwStatus(hDfe->hDfeDduc[ui], DFE_FL_DDUC_QUERY_DIRECT, &dducDir[ui]) );
    }
        
    //one additional sync re-alignment before starting uplink direction (needed for legacy
    //reasons to match old gold files)
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    
    //rx (except syncs)
    //There are several legacy requirements for initializing rx:
    //1) init_clk_gate, init_state and clear_data all need to be released after jesdrx cken comes but
    //BEFORE jesdrx frame comes)
    //2) the syncs must be done after the jesdrx cken AND frames come, and MUST be AFTER the clear_data
    //is released
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 1, 1);
    CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 0, 1);
    CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 0, 0);
    CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);

    //fb (except syncs)
    //There are several legacy requirements for initializing fb (which is based on rx):
    //1) init_clk_gate, init_state and clear_data all need to be released after jesdrx cken comes but
    //BEFORE jesdrx frame comes)
    //2) the syncs must be done after the jesdrx cken AND frames come, and MUST be AFTER the clear_data
    //is released
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 1, 1);
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 0, 1);
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 0, 0);
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);


    //jesdrx (already released cken above)
    // release init state
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, 0, 0, 1);
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits) );
    //software wait 1ms in the jesd rx sections
    Dfe_osalSleep(1);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1);
    // release clear data
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, 0, 0, 0);
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits) );
    Dfe_osalSleep(1);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1);

    //rx syncs (need to be after jesdrx frames arrive, and after rx clear_data is released)
    //dc accumulator syncs
    data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2;
    CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_DC_GSG_SSEL, &data) );
    dcGeneric.dc = DFE_FL_RX_DC_ALL;
    dcGeneric.data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2;
    CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_DKACC_SSEL, &dcGeneric) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // back to never
    data = DFE_FL_SYNC_GEN_SIG_NEVER;
    CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_DC_GSG_SSEL, &data) );
    dcGeneric.dc = DFE_FL_RX_DC_ALL;
    dcGeneric.data = DFE_FL_SYNC_GEN_SIG_NEVER;
    CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_DKACC_SSEL, &dcGeneric) );

    //fb syncs (need to be after jesdrx frames arrive, and after fb clear_data is released)
    //dc accumulator syncs
    data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2;
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_DC_GSG_SSEL, &data) );
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_DKACC_SSEL, &data) );
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // back to never
    data = DFE_FL_SYNC_GEN_SIG_NEVER;
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_DC_GSG_SSEL, &data) );
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_DKACC_SSEL, &data) );

    //rx dducs (dducs must be released at the same time)       
    //
    //
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 1, 1);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits) );
        }
    }
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 0, 1);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits) );
        }
    }
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // nco syncs
    mixNcoSsel.iMix = DFE_FL_DDUC_MIX_NCO_ALL;
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            mixNcoSsel.data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2;
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &mixNcoSsel) );
        }
    }
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // nco sync back to never
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            mixNcoSsel.data = DFE_FL_SYNC_GEN_SIG_NEVER;
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &mixNcoSsel) );
        }
    }
    //frw fifo takes time to clear, so wait before releasing clear_data
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, 0, 0, 0);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits) );
        }
    }
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);


    //Finally, set the bb's ul frame start logic to look at the IQN2's 5/10/20ms
    //(depending on the standard) frame. Note this will not align to sysref (there
    //is no way to align the IQN2-BB buffer to sysref). Note it is NOT important
    //to wait for a frame strobe from the IQN/agent before proceeding, so not doing
    //a sync poll. This way, this init sequence can work even if the IQN/agent are
    //disabled.
    //***for now the mux for WHICH frame strobe is used here is assumed to be done in the .cfg...
    //`DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0370, 32'h0000_2222)
    //`DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0374, 32'h0000_2222)
    //`DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0378, 32'h0000_2222)
    //`DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_037c, 32'h0000_2222)
    ulStrobe.ct = DFE_FL_BB_CARRIER_TYPE_ALL;
    ulStrobe.strobe = hDfe->ulStrobe_Sync;
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_CT_UL_SYNC_STROBE, &ulStrobe) );

    //Wait one last counter sequence to align everything before proceeding.
    ISSUE_WAIT_SYNC(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    
    return DFE_ERR_NONE;
}

/**
 * @brief Get DFE device information.
 * @ingroup DFE_LLD_DEVICE_FUNCTION
 *
 * Information such as PID, base address etc.
 *
 * @param hDfe	[in] DFE device handle
 * @param devInfo	[out] pointer to devInfo buffer
 *
 * @return
 *  - #DFE_ERR_NONE, if get info properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if devInfo is NULL
 *
 * @pre
 *  - hDfe should be a valid handle opened by #Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *
 * @post
 *   - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_getDevInfo
(
    DFE_Handle hDfe,
    DFE_DevInfo *devInfo
)
{
    VALID_DFE_HANDLE(hDfe);
    
    if(devInfo == NULL)
    {
        Dfe_osalLog("devInfo is NULL!");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    devInfo->pid = (uint32_t) hDfe->objDfe.regs[0];
    devInfo->baseAddr = (void *)hDfe->objDfe.regs;
    devInfo->version = DFE_LLD_VERSION;
    
    return DFE_ERR_NONE;
}

// =============================== Local Functions ===============================
/** ============================================================================
 *   @n@b setSyncPulseWidth
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]
         syncSig    [add content]
         pulseWidth    [add content]
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
static DFE_Err setSyncPulseWidth(DFE_Handle hDfe, DfeFl_MiscSyncGenSig syncSig, uint32_t pulseWidth)
{
    DfeFl_Status status;
    DfeFl_MiscSyncGenGeneric syncGeneric;

	syncGeneric.syncSig = syncSig;
	syncGeneric.data    = pulseWidth;
	status = dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CFG_SYNC_ONE_SHOT, &syncGeneric);
	if(status != DFE_FL_SOK)
	{
		Dfe_osalLog("dfeFl_MiscHwControl() error = %d", status);
		return DFE_ERR_HW_CTRL;
	}
	
	return DFE_ERR_NONE;	
}
