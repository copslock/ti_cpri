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

//This executes the full DFE initialization sequence. A few things to note:
//-block syncs being done w/mpu_sync...is this the right long-term solution?
//-currently init signals released on always sync (not a big deal, but not clean)
//-ASSUMING right now that the following have already happened prior to running this init sequence:
//-DFE reset was asserted and released
//-dfe_pll is warmed-up and running
//-DFE selects the running dfe_pll_clk
//-the .reg file was programmed already (OR that it was unneeded)
//-the .reg file KEEPS all the init signals ASSERTED (this might not matter as long as the inits_ssel isn't sent...)
//-TX's cc_* cken mux controls are already set (usually, this is done in the .reg file)

//the commands:
//vbusp_write32 (<address>, <data>);
//vbusp_read32  (<address>, <variable_to_store_read_result>);
//vbusp_rmw32   (<address>, <data>, <mask...high bits are written to>);

//Note:  simply for readability, I split all addresses into 3 chunks:
//<dfe_starting_address> + <block_starting_address> + <address_within_block>

//Note:  the repeat commands are wait commands.
//repeat(<number_of_clocks>) @(posedge top_tb.dut.dfe_pll_clk);
//A single clock is between 2.71 and 4ns, depending on the application
#include <ti/csl/csl.h>
//#include <val_util.h>
#include <constants.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_tsc.h>
#ifdef _TMS320C6X
#include <c6x.h>
#endif

#include "dfe_inc.h"

#undef CHK_MAX_TIME

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

#ifdef CHK_MAX_TIME
// we don't care sync
#define issueSync(a, b)
// we don't care sync_cntr
#define activeSyncCntr(a, b, c)
// update max time used
#define UPD_MAX_TIME(t_start, t_now, t_max) \
do { \
    Uint32 t_delta; \
    t_now = TSCL; \
    t_delta = t_now - t_start; \
    t_start = t_now; \
    if(t_delta > t_max) t_max = t_delta; \
} while(0);
#else
#define UPD_MAX_TIME(t_start, t_now, t_max)
#endif

Uint32 vbusp_read32  (Uint32 address)
{
	return( *(volatile Uint32 *)address );
}

/*static void vbusp_write32 (Uint32 address, Uint32 data)
{
	*(volatile Uint32 *)address = data;
}*/

void vbusp_rmw32   (Uint32 address, Uint32 data, Uint32 mask)
{
	Uint32 temp = *(volatile Uint32 *)address;

	temp = (temp & ~mask) | (data & mask);

	*(volatile Uint32 *)address = temp;
}

extern uint32_t serdes_cfg0_base; //MMR base address of SerDes config0
extern uint32_t serdes_cfg1_base; //MMR base address of SerDes config1

//
// Write '1's to inits' bits.
// 
void softResetDfe()
{
    Uint32 ui;
    DfeFl_SublkInitsConfig inits;
    DfeFl_JesdInitsConfig jesdInits;
    
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_ALWAYS, 1, 1, 1);
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_ALWAYS, 1, 1, 1);
    
    // reset BB
    dfeFl_BbHwControl(hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits);
    // reset DDUCs
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits);
    }
    // reset CFRs
    for(ui = 0; ui < DFE_FL_CFR_PER_CNT; ui++)
    {
        dfeFl_CfrHwControl(hDfeCfr[ui], DFE_FL_CFR_CMD_CFG_INITS, &inits);
    }
    // reset AUTOCP
    dfeFl_AutocpHwControl(hDfeAutocp[0], DFE_FL_AUTOCP_CMD_CFG_INITS, &inits);
    // reset CDFR
    dfeFl_CdfrHwControl(hDfeCdfr[0], DFE_FL_CDFR_CMD_CFG_INITS, &inits);
    // reset DPD
    dfeFl_DpdHwControl(hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits);
    // reset DPDA
    dfeFl_DpdaHwControl(hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits);
    // reset TX
    dfeFl_TxHwControl(hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits);
    // reset RX
    dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits);
    // reset CB
    dfeFl_CbHwControl(hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits);
    // reset JESDTX
    dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits);
    // reset JESDRX
    dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits);
    // reset FB
    dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits);
    // reset MISC
    dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits);
}

////////initialization sequence//////
void initializeSequence()
{
    Uint32 ui, data, tStart, tNow, tMax = 0;
    Uint32 ctUlSyncTrobe[16];
    Uint32 dducDir[DFE_FL_DDUC_PER_CNT];
    DfeFl_SublkInitsConfig inits;
    DfeFl_JesdInitsConfig jesdInits;
    DfeFl_BbCarrierTypeUlSyncStrobeConfig ulStrobe;
//  Uint32 txSysrefMode[2], rxSysrefMode[2];
    DfeFl_JesdLinkSysrefModeConfig sysrefModeQry;
    Uint32 txLaneEnb[4], rxLaneEnb[4];
    Uint32 bbtx_siggen_ssel, bbrx_chksum_ssel;
    DfeFl_JesdLaneConfig laneCfg;
    DfeFl_JesdRxLoopbackConfig rxLoopbackCfg;
    DfeFl_DducMixNcoSsel mixNcoSsel;
    DfeFl_RxDcGeneric dcGeneric;
    DfeFl_CfrMultSsel cfrMultSsel;
    DfeFl_BbTestGenSsel BbTestGenSsel;
    DfeFl_BbChksumSsel BbChksumSsel;
    Uint32 lane_lpbk;
    
    // save .reg values
//    for(ui = 0; ui <= DFE_FL_BB_CARRIER_TYPE_15; ui++)
//    {        
//        ulStrobe.ct = ui;
//        dfeFl_BbGetHwStatus(hDfeBb[0], DFE_FL_BB_QUERY_CT_UL_SYNC_STROBE, &ulStrobe);
//        ctUlSyncTrobe = ulStrobe.strobe;
//    }

	softResetDfe();

    //Release mem_mpu_access for all blocks
    data = DFE_FL_MEM_MPU_ACCESS_ALL;
    dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MEM_MPU_ACCESS, &data);
    
    //Be sure the bb's frame start logic is set to never sync (it must not be synced until
    //the end of initialization). It will be put on the appropriate sync at the end of
    //initialization.
    ulStrobe.ct = DFE_FL_BB_CARRIER_TYPE_ALL;
    ulStrobe.strobe = DFE_FL_SYNC_GEN_SIG_NEVER;
    dfeFl_BbHwControl(hDfeBb[0], DFE_FL_BB_CMD_CFG_CT_UL_SYNC_STROBE, &ulStrobe);
        
    //determine which dducs are in tx and which are rx.
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        dfeFl_DducGetHwStatus(hDfeDduc[ui], DFE_FL_DDUC_QUERY_DIRECT, &dducDir[ui]);
    }
    
    //Store the sysref_modes and the sysref_internal_sel
//  for(ui = 0; ui <= DFE_FL_JESD_LINK_1; ui++)
//  {
//      sysrefModeQry.link = ui;
//
//      dfeFl_JesdGetHwStatus(hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_SYSREF_MODE, &sysrefModeQry);
//      txSysrefMode[ui] = sysrefModeQry.mode;
//
//      dfeFl_JesdGetHwStatus(hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_SYSREF_MODE, &sysrefModeQry);
//      rxSysrefMode[ui] = sysrefModeQry.mode;
//  }

    //Store the bbtx signal generator and bbrx chksum ssel values
    //`DFE_INIT_VSEQ_READ32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0e14, read_data)
    BbTestGenSsel.tgDev = DFE_FL_BB_AID_TESTGEN_A;
    dfeFl_BbGetHwStatus(hDfeBb[0], DFE_FL_BB_QUERY_TESTGEN_SSEL, &BbTestGenSsel);
    bbtx_siggen_ssel = BbTestGenSsel.ssel;
    BbChksumSsel.chksumDev = DFE_FL_BB_AID_CHKSUM_A;
    dfeFl_BbGetHwStatus(hDfeBb[0], DFE_FL_BB_QUERY_CHKSUM_SSEL, &BbChksumSsel);
    bbrx_chksum_ssel = BbChksumSsel.ssel;
    
    //Now, the DSP should determine which JESD lanes are enabled and not looping back
    for(ui = 0; ui <= DFE_FL_JESD_LANE_3; ui++)
    {
        laneCfg.lane = ui;
        
        dfeFl_JesdGetHwStatus(hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_LANE_CFG, &laneCfg);
        txLaneEnb[ui] = laneCfg.laneEnable;
        
        dfeFl_JesdGetHwStatus(hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LANE_CFG, &laneCfg);
        rxLaneEnb[ui] = laneCfg.laneEnable;
    }
    dfeFl_JesdGetHwStatus(hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LOOPBACK, &rxLoopbackCfg);

    //Now, we should read from the SERDES until we see that the ststx[4] is high for
    //active lanes. Since this init sequence is used by both the gc_dfe_lamarr tb and
    //the gc_dfe_iqn_lamarr tb, we need to use the DFE_LEVEL_TB define to determine
    //what to do.
    
    // poll for tx lock (ststx[4]) - serdes0 - lane0 & lane1 ==> pll_ok = 28th bit
    // pll_ok is equivalent to ststx[4].
    if (txLaneEnb[0] && !rxLoopbackCfg.lane0 && txLaneEnb[1] && !rxLoopbackCfg.lane1) 
    {
        data = 0x0;
        while ( (data & 0x10000000) != 0x10000000 )
        {
            data=vbusp_read32(serdes_cfg0_base +0x00001FC0+0x00000034);
        }    
    }
    
    // poll for tx lock (ststx[4]) - serdes1 - lane0 & lane1 ==> pll_ok = 28th bit
    if (txLaneEnb[2] && !rxLoopbackCfg.lane2 && txLaneEnb[3] && !rxLoopbackCfg.lane3) 
    {
        data = 0x0;
        while ( (data & 0x10000000) != 0x10000000 ) 
        {
            data=vbusp_read32(serdes_cfg1_base +0x00001FC0+0x00000034);
        }
    }

    
    //All syncing will be aligned to the IQN ul sync, so that it all gets aligned to the
    //IQN (note that this will NOT align to the serdes sysref, but there is no way to
    //align to both...this means there is still one source of latency variability).
    //To do this, we will use the ul sync (ssel 2) sync to kick-off a repeating sync
    //counter (ssel 12), and use that sync counter to sync everything.
    //The exception is if the bbtx sig gen is enabled in the BB, which means IQN is not
    //being used anyways, so just use mpu always sync (this will not be deterministic
    //unless in loopback mode, but it is useful if IQN is not on).
    //Finally, note to make it deterministic even from power-up to power-up and across
    //multiple Lamarrs, the sync counter must switch back to never at a deterministic time
    //using an LVDS sync. This is NOT being done for now...
    //First, set the sysref sync to be a one-shot (to detect its rising edges).
    //`DFE_INIT_VSEQ_RMW32(32'h2400_0000 + 32'h01f0_0000 + 32'h0000_2404, 32'h0000_0010, 32'h0000_00f0)
    setSyncPulseWidth(DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, 1);

    //Reset the sync counter by setting the period to 0. Then, configure the counter
    //sync to listen to ul sync. As ul sync keeps coming, the counter keeps resetting.
    //The length of this counter must meet these requirements:
    //-must be a factor of ul sync (which comes every 5, 10 or 20ms. Note 10ms = 2,457,600,000 clks at 245.76Mhz)
    //-must be long enough that mpu reads/writes deterministically fit within one period
    //***note mpu reads/writes will take longer on the real system than on TB. Simply using 1024 for now (which is 100% fine for TB).
    //Exception:  if the bb_sig_gen is on, then just use always sync.
    resetSyncCntr(DFE_FL_SYNC_GEN_CNTR0);
    progSyncCntr(DFE_FL_SYNC_GEN_CNTR0, 0 /*dly*/, 0x2000 /*period*/, 1 /*pulseWidth*/);
    
//  if((txSysrefMode[0] | txSysrefMode[1] | rxSysrefMode[0] | rxSysrefMode[1]) != 0)
    if(bbtx_siggen_ssel == 0)
    {
        //activeSyncCntr(DFE_FL_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_SYSREF /*startSsel*/, DFE_FL_SYNC_GEN_SIG_ALWAYS /*progSsel*/);
        activeSyncCntr(DFE_FL_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0 /*startSsel*/, DFE_FL_SYNC_GEN_SIG_ALWAYS /*progSsel*/);
    }
    else
    {
        activeSyncCntr(DFE_FL_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_ALWAYS /*startSsel*/, DFE_FL_SYNC_GEN_SIG_ALWAYS /*progSsel*/);
    }

    //Now, set the counter back to never. This releases it to begin counting, and the last
    //ul sync that came will be the one that everything syncs off of.
    //***Here, if we use a deterministic LVDS sync to update the sync counter's ssel from 
    // shadow to active, we can align the DFE state machines across multiple Lamarrs 
    // IF it is a one-shot sync. Not doing this now, though...just using mpu always sync
    // Also note that even mpu sync might be fine if the sysref rate is low enough (slow
    // enough to deterministically get in the mpu write before the next sysref arrives).
    activeSyncCntr(DFE_FL_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_NEVER /*startSsel*/, DFE_FL_SYNC_GEN_SIG_ALWAYS /*progSsel*/);
           
    //Counter has begun counting. Clear the counter sync detect and set sync_poll vseq to
    //use the counter.
    //Align to the first counter period. Everything is aligned after this.
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
#ifdef _TMS320C6X
    tStart = TSCL;
#endif
    //Release each block in signal processing order. Each given block releases init_clk_gate,
    //then init_state, then clear_data, with delays between each release.
    //    
    //misc
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);                
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    
    //cb (being done first to support cb sourcing, so sourcing can begin before blocks are initialized)
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_CbHwControl(hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_CbHwControl(hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_CbHwControl(hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);


    //bb (the bbtx and bbrx are both released here, which means bbrx will be released before
    //dducs send frames to it, but this is perfectly fine)
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_BbHwControl(hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_BbHwControl(hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_BbHwControl(hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //tx dducs (dducs must be released at the same time; use mpu_sync)
    //
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits);
        }
    }
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits);
        }
    }
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // nco syncs
    mixNcoSsel.iMix = DFE_FL_DDUC_MIX_NCO_ALL;
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            mixNcoSsel.data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &mixNcoSsel);
        }
    }
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // nco sync back to never
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            mixNcoSsel.data = DFE_FL_SYNC_GEN_SIG_NEVER;
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &mixNcoSsel);
        }
    }
    //frw fifo takes time to clear, so wait before releasing clear_data
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_DL)
        {
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits);
        }
    }
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    
    //summer (does not have init_clk_gate)
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_SummerHwControl(hDfeSummer[0], DFE_FL_SUMMER_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_SummerHwControl(hDfeSummer[0], DFE_FL_SUMMER_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    

    //acl
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_AutocpHwControl(hDfeAutocp[0], DFE_FL_AUTOCP_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_AutocpHwControl(hDfeAutocp[0], DFE_FL_AUTOCP_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_AutocpHwControl(hDfeAutocp[0], DFE_FL_AUTOCP_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //cfrs
    //Note the 2 CFRs need to be release at the same time. We are using two async
    //writes to lower them, but these writes are aligned to the beginning of a
    //sync counter period already by virtue of the prior sync poll. So these two
    //writes should get through in time for the same sync counter period to
    //release them at the same time.
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_CfrHwControl(hDfeCfr[0], DFE_FL_CFR_CMD_CFG_INITS, &inits);
    dfeFl_CfrHwControl(hDfeCfr[1], DFE_FL_CFR_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_CfrHwControl(hDfeCfr[0], DFE_FL_CFR_CMD_CFG_INITS, &inits);
    dfeFl_CfrHwControl(hDfeCfr[1], DFE_FL_CFR_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    //pre & post mult
    cfrMultSsel.path = DFE_FL_CFR_PATH_ALL;
    cfrMultSsel.ssel = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
    dfeFl_CfrHwControl(hDfeCfr[0], DFE_FL_CFR_CMD_SET_PREM_SSEL, &cfrMultSsel);
    dfeFl_CfrHwControl(hDfeCfr[0], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &cfrMultSsel);
    dfeFl_CfrHwControl(hDfeCfr[1], DFE_FL_CFR_CMD_SET_PREM_SSEL, &cfrMultSsel);
    dfeFl_CfrHwControl(hDfeCfr[1], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &cfrMultSsel);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // back to never
    cfrMultSsel.ssel = DFE_FL_SYNC_GEN_SIG_NEVER;
    dfeFl_CfrHwControl(hDfeCfr[0], DFE_FL_CFR_CMD_SET_PREM_SSEL, &cfrMultSsel);
    dfeFl_CfrHwControl(hDfeCfr[0], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &cfrMultSsel);
    dfeFl_CfrHwControl(hDfeCfr[1], DFE_FL_CFR_CMD_SET_PREM_SSEL, &cfrMultSsel);
    dfeFl_CfrHwControl(hDfeCfr[1], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &cfrMultSsel);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_CfrHwControl(hDfeCfr[0], DFE_FL_CFR_CMD_CFG_INITS, &inits);
    dfeFl_CfrHwControl(hDfeCfr[1], DFE_FL_CFR_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //cdfr
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_CdfrHwControl(hDfeCdfr[0], DFE_FL_CDFR_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_CdfrHwControl(hDfeCdfr[0], DFE_FL_CDFR_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_CdfrHwControl(hDfeCdfr[0], DFE_FL_CDFR_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //dpda
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_DpdaHwControl(hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_DpdaHwControl(hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_DpdaHwControl(hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //dpd
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_DpdHwControl(hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_DpdHwControl(hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_DpdHwControl(hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //tx
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_TxHwControl(hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_TxHwControl(hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_TxHwControl(hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //jesdtx and jesdrx init_clk_gate. Need to release the jesdrx init_clk_gate simultaneous
    //with jesdtx init_clk_gate for alignment in certain funky jesd loopback configs. Since
    //it never HURTS any config to do this, just always do it (more robust).
    // release clock gate
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits);
    dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //Now, the DSP should determine which JESD lanes are enabled and not looping back, then
    //read from the SERDES until it sees that the stsrx[2] is high AND stsrx[1] is low for
    //those lanes. Again, we need to determine if this is gc_dfe_lamarr tb or
    //gc_dfe_iqn_lamarr tb.
    //***note that even this wait section here must finish within one sync counter period for 100% repeatability...
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

#if 0 // from DV
    // Coming out from the serdes TX Idle mode ..
    vbusp_write32( serdes_cfg0_base + 0x00001FC0 + 0x00000020, 0xF6C0F030 ); // LANE0CTL_STS - Enable TX, Full Rate, 20-bit, Enable RX, Full Rate, 20-bit, Disable TX Idle
    vbusp_write32( serdes_cfg0_base + 0x00001FC0 + 0x00000024, 0xF6C0F030 ); // LANE1CTL_STS - Enable TX, Enable RX, Disable TX Idle

    vbusp_write32( serdes_cfg1_base + 0x00001FC0 + 0x00000020, 0xF6C0F030 ); // LANE0CTL_STS - Enable TX, Full Rate, 20-bit, Enable RX, Full Rate, 20-bit, Disable TX Idle
    vbusp_write32( serdes_cfg1_base + 0x00001FC0 + 0x00000024, 0xF6C0F030 ); // LANE1CTL_STS - Enable TX, Enable RX, Disable TX Idle
#else // from Cliff
    // Disabling serdes TX IDLE
    vbusp_rmw32(serdes_cfg0_base + 0x00001FC0 + 0x00000020, 0x02000000, 0x03000000);
    vbusp_rmw32(serdes_cfg0_base + 0x00001FC0 + 0x00000024, 0x02000000, 0x03000000);
    vbusp_rmw32(serdes_cfg1_base + 0x00001FC0 + 0x00000020, 0x02000000, 0x03000000);
    vbusp_rmw32(serdes_cfg1_base + 0x00001FC0 + 0x00000024, 0x02000000, 0x03000000);
    //Released the serdes TX IDLE !!
#endif
    //This is the gc_dfe_iqn_lamarr testbench, which has the serdes. Poll the
    //stsrx[2] (the OK bit) and stsrx[1] (the LOSS bit) from the appropriate serdes
    //macros.
#if DFE_BB_LOOPBACK == 0
    // poll for rx OK (stsrx[2]) & rx LOSS (stsrx[1]) - serdes0 - lane0
    if (rxLaneEnb[0] && !rxLoopbackCfg.lane0)
    {
        data = 0x0;
        while ( (data & 0x00000003) != 0x00000002 )
            data=vbusp_read32( serdes_cfg0_base +0x00001FC0+0x00000020);
    }

    // poll for rx OK (stsrx[2]) & rx LOSS (stsrx[1]) - serdes0 - lane1
    if (rxLaneEnb[1] && !rxLoopbackCfg.lane1)
    {
        data = 0x0;
        while ( (data & 0x00000003) != 0x00000002 )
            data=vbusp_read32( serdes_cfg0_base +0x00001FC0+0x00000024);
    }

    // poll for rx OK (stsrx[2]) & rx LOSS (stsrx[1]) - serdes1 - lane0
    if (rxLaneEnb[2] && !rxLoopbackCfg.lane2)
    {
        data = 0x0;
        while ( (data & 0x00000003) != 0x00000002 )
            data=vbusp_read32( serdes_cfg1_base+0x00001FC0+0x00000020);
    }

    // poll for rx OK (stsrx[2]) & rx LOSS (stsrx[1]) - serdes1 - lane1
    if (rxLaneEnb[3] && !rxLoopbackCfg.lane3)
    {
        data = 0x0;
        while ( (data & 0x00000003) != 0x00000002 )
            data=vbusp_read32( serdes_cfg1_base +0x00001FC0+0x00000024);
    }
#endif
    //one additional sync re-alignment before starting uplink direction (needed for legacy
    //reasons to match old gold files)
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    
    //rx (except syncs)
    //There are several legacy requirements for initializing rx:
    //1) init_clk_gate, init_state and clear_data all need to be released after jesdrx cken comes but
    //BEFORE jesdrx frame comes)
    //2) the syncs must be done after the jesdrx cken AND frames come, and MUST be AFTER the clear_data
    //is released
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //fb (except syncs)
    //There are several legacy requirements for initializing fb (which is based on rx):
    //1) init_clk_gate, init_state and clear_data all need to be released after jesdrx cken comes but
    //BEFORE jesdrx frame comes)
    //2) the syncs must be done after the jesdrx cken AND frames come, and MUST be AFTER the clear_data
    //is released
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);


    //jesdrx (already released cken above)
    // release init state
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS_JESD(&jesdInits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //rx syncs (need to be after jesdrx frames arrive, and after rx clear_data is released)
    //dc accumulator syncs
    data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
    dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_SET_DC_GSG_SSEL, &data);
    dcGeneric.dc = DFE_FL_RX_DC_ALL;
    dcGeneric.data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
    dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_SET_DKACC_SSEL, &dcGeneric);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // back to never
    data = DFE_FL_SYNC_GEN_SIG_NEVER;
    dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_SET_DC_GSG_SSEL, &data);
    dcGeneric.dc = DFE_FL_RX_DC_ALL;
    dcGeneric.data = DFE_FL_SYNC_GEN_SIG_NEVER;
    dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_SET_DKACC_SSEL, &dcGeneric);

    //fb syncs (need to be after jesdrx frames arrive, and after fb clear_data is released)
    //dc accumulator syncs
    data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
    dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_SET_DC_GSG_SSEL, &data);
    dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_SET_DKACC_SSEL, &data);
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // back to never
    data = DFE_FL_SYNC_GEN_SIG_NEVER;
    dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_SET_DC_GSG_SSEL, &data);
    dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_SET_DKACC_SSEL, &data);

    //rx dducs (dducs must be released at the same time)       
    //
    //
    // release clock gate
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 1, 1);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits);
        }
    }
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release init state
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 1);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits);
        }
    }
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // nco syncs
    mixNcoSsel.iMix = DFE_FL_DDUC_MIX_NCO_ALL;
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            mixNcoSsel.data = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &mixNcoSsel);
        }
    }
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // nco sync back to never
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            mixNcoSsel.data = DFE_FL_SYNC_GEN_SIG_NEVER;
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &mixNcoSsel);
        }
    }
    //frw fifo takes time to clear, so wait before releasing clear_data
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
    // release clear data
    MK_INITS(&inits, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 0, 0, 0);
    for(ui = 0; ui < DFE_FL_DDUC_PER_CNT; ui++)
    {
        if(dducDir[ui] == DFE_FL_DDUC_DIR_UL)
        {
            dfeFl_DducHwControl(hDfeDduc[ui], DFE_FL_DDUC_CMD_CFG_INITS, &inits);
        }
    }
    UPD_MAX_TIME(tStart, tNow, tMax);
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);

    //Finally, set the bb's ul frame start logic to look at the correct sync. In
    //normal operation, this is the IQN2's 5/10/20ms (depending on the standard)
    //frame. Note it is NOT important to wait for a frame strobe from the
    //IQN/agent before proceeding, so not doing a sync poll (this way, this init
    //sequence can work even if the IQN/agent are disabled.
    //The exception to using the IQN2 frame is if we are running a checksum test.
    //In this case, this logic must sync on the sync counter being used by the
    //signal generators and checksum block. We will assume a checksum is being
    //run if:
    //-the jesd lanes are looping back
    //-the bbrx checksum ssel is set
    //-the bbtx signal generator ssel matches the bbrx checksum ssel
    //if (lane_lpbk && bbrx_chksum_ssel && (bbtx_siggen_ssel == bbrx_chksum_ssel)) //checksum
    //begin
    //    `DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0370, {16'h0000, {4{bbrx_chksum_ssel}}})
    //    `DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0374, {16'h0000, {4{bbrx_chksum_ssel}}})
    //    `DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0378, {16'h0000, {4{bbrx_chksum_ssel}}})
    //    `DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_037c, {16'h0000, {4{bbrx_chksum_ssel}}})
    //end
    //else //normal operation (not checksum)
    //begin
    //    `DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0370, 32'h0000_2222)
    //    `DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0374, 32'h0000_2222)
    //    `DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_0378, 32'h0000_2222)
    //    `DFE_INIT_VSEQ_WRITE32(32'h2400_0000 + 32'h0000_0000 + 32'h0004_037c, 32'h0000_2222)
    //end
    ulStrobe.ct = DFE_FL_BB_CARRIER_TYPE_ALL;
    lane_lpbk = rxLoopbackCfg.lane0 || rxLoopbackCfg.lane1 || rxLoopbackCfg.lane2 || rxLoopbackCfg.lane3;
    if (lane_lpbk && bbrx_chksum_ssel && (bbtx_siggen_ssel == bbrx_chksum_ssel)) //checksum
    	ulStrobe.strobe = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
    else
    	ulStrobe.strobe = DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0;
    dfeFl_BbHwControl(hDfeBb[0], DFE_FL_BB_CMD_CFG_CT_UL_SYNC_STROBE, &ulStrobe);

    //Wait one last counter sequence to align everything before proceeding.
    issueSync(DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_MISC_SYNC_WAITFOREVER);
}

void startDfe_local()
{
    // This is only used for test purposes - a production system will have this done externally
    //DFESS_ProgDfeTestCntr(3); // internal sysref

	initializeSequence();
        
    // config MPU sync to one shot
    setSyncPulseWidth(DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 1);

    // HACK - to avoid sysref jittering
//    setJesdTxSysrefMode(DFE_FL_JESD_LINK_ALL, 0);
//    setJesdRxSysrefMode(DFE_FL_JESD_LINK_ALL, 0);
}

