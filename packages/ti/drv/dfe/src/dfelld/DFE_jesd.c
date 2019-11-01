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
 * @defgroup DFE_LLD_JESD_FUNCTION JESD
 * @ingroup DFE_LLD_FUNCTION
 */
 
DFE_Err Dfe_getJesdTxLaneEnable
(
    DFE_Handle hDfe,
    uint32_t laneEnable[4],
    uint32_t linkAssign[4]
)
{
    uint32_t ui;
    DfeFl_Status status;
    DfeFl_JesdLaneConfig laneCfg;    
    
    VALID_DFE_HANDLE(hDfe);
    
    for(ui = 0; ui <= DFE_FL_JESD_LANE_3; ui++)
    {
        laneCfg.lane = ui;
        
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_LANE_CFG, &laneCfg) );
        laneEnable[ui] = laneCfg.laneEnable;
        linkAssign[ui] = laneCfg.linkAssign;
    }    
    
    return DFE_ERR_NONE;
}


/**
 * @brief Program JESD Tx to Lane Map
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * Program how to fill time slots of a lane with Tx bus time slots. 
 *
 * A Tx bus consists of four time slots. A Tx bus time slot, an element
 * of laneMap[], is selected by the bus# and slot# of the bus.
 *
 * ~~~{.c}
 * // Jesd Tx bus to lane map
 * typedef struct
 * {
 *     // bus#, one of DfeFl_JesdTxTxBus (I0, Q0, I1, Q1)
 *     uint32_t bus;
 *     // slot#, 0 ~ 3
 *     uint32_t busPos;
 * } DFE_JesdTxLaneMapPos;
 * ~~~
 *   
 * A lane also consists of four time slots: 
 *  - laneMap[0] is mapping to lane time slot 0 
 *  - laneMap[1] is mapping to lane time slot 1
 *  - laneMap[2] is mapping to lane time slot 2 
 *  - laneMap[3] is mapping to lane time slot 3
 *
 * For example, TX0 has two interleaved antenna streams, A0 and A1,
 * which are mapping to two lanes, lane0 and lane1, A0 => lane0, A1 => lane1.
 *
 * TX0 bus format:
 *
 *  | Bus#      | Slot0 | Slot1 | Slot2      | Slot3      |
 *  | --------- | ----- | ----- | ---------- | ---------- |
 *  | 0 (TX0_I) | A0_i	| A1_i	| Don't care | Don't care |
 *  | 1 (TX0_Q) | A0_q	| A1_q	| Don't care | Don't care |
 *
 * Lane bus format:
 *
 *  | Lane#     | Slot0 | Slot1 | Slot2      | Slot3      |
 *  | --------- | ----- | ----- | ---------- | ---------- |
 *  | 0 (lane0) | A0_i	| A0_q  | Don't care | Don't care |
 *  | 1 (lane1) | A1_i	| A1_q  | Don't care | Don't care |
 *
 * Then laneMap for lane0 should be:
 *
 *  | x     | laneMap[0] | laneMap[1] | laneMap[2] | laneMap[3] |
 *  | ----- | ---------- | ---------- | ---------- | ---------- |
 *  | Bus#  | 0          | 1          | Don't care | Don't care |
 *  | Slot# | 0          | 0          | Don't care | Don't care |
 *
 * Then laneMap for lane1 should be:
 *
 *  | x     | laneMap[0] | laneMap[1] | laneMap[2] | laneMap[3] |
 *  | ----- | ---------- | ---------- | ---------- | ---------- |
 *  | Bus#  | 0          | 1          | Don't care | Don't care |
 *  | Slot# | 1          | 1          | Don't care | Don't care |
 *
 *  @param hDfe	[in] DFE device handle
 *  @param lane	[in] Tx lane#
 *  @param laneMap	[in] Tx bus slot map
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_mapJesdTx2Lane
(
    DFE_Handle hDfe,
    uint32_t lane,
    DFE_JesdTxLaneMapPos laneMap[4]
)
{
    DfeFl_Status status;
    uint32_t lanePos, laneNibb, busNibb;
    DfeFl_JesdTxMapLaneConfig cfg;
    
    VALID_DFE_HANDLE(hDfe);
    
    cfg.lane = lane;
    
    for(lanePos = 0; lanePos < 4; lanePos++)
    {
        busNibb = laneMap[lanePos].bus * 4; 
        
        // array [nibble][position]
        for(laneNibb = 0; laneNibb < 4; laneNibb++)
        {
            cfg.nibbleSel[laneNibb][lanePos] = busNibb + laneNibb;
            cfg.framePos[laneNibb][lanePos] = laneMap[lanePos].busPos;
        }
    }
    
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_MAP_LANE, &cfg) );

    return DFE_ERR_NONE;    
}    

/**
 * @brief Program JESD Tx SigGen Ramp
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * Program a JESDTX signal generator to produce a ramp. The following
 * rules should be followed:
 * - start <= stop
 * - (stop - start) % slope = 0
 * - Start/stop/slope range, -131072 ~ 131071
 *
 * When start equals to stop and slope is 0, a constant is produced.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param sigGenDev	[in] JESD TX SigGen device
 *  @param enable	[in] 1 to enable; 0 to disable
 *  @param start	[in] ramp start values for the bus
 *  @param stop	[in] ramp stop values for the bus
 *  @param slope	[in] ramp step values for the bus
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progJesdTxSigGenRamp
(
    DFE_Handle hDfe,
    DfeFl_JesdTxSignalGen sigGenDev, 
    uint32_t enable,
    int32_t start,
    int32_t stop,
    int32_t slope
)
{
    DfeFl_Status status;
    DfeFl_JesdTxTestGenConfig testGenCfg;
    
    VALID_DFE_HANDLE(hDfe);
    
    testGenCfg.tgDev = sigGenDev;
    /// enable data generation
    testGenCfg.genData = 1;
    /// enbale frame generation
    testGenCfg.genFrame = 0;
    /// ramp (1), or LFSR (0)
    testGenCfg.rampMode = DFE_FL_JESDTX_TESTGEN_RAMP_MODE_RAMP;
    /// seed
    testGenCfg.seed = 0;
    /// number of clocks per frame minus 1
    testGenCfg.frameLenM1 = 0;
    /// ramp starting value
    testGenCfg.rampStart = start;
    /// ramp stop value    
    testGenCfg.rampStop = stop;
    /// ramp slop value
    testGenCfg.slope = slope;
    /// 0 = generate data forever, n = generate data for n clock cycles
    testGenCfg.genTimer = 0;

    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_TESTGEN, &testGenCfg) );

    return DFE_ERR_NONE;    
}    

/**
 * @brief Issue Sync Update JESDTX SigGen
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * Issue sync update JESDTX SigGen. Dfe_getSyncStatus() can be called
 * later to check if the sync has come.
 *
 * When sync ssel comes, the ramp restarts from start value and step up
 * the slope value per clock. When accumulated value equal to stop value,
 * the ramp restarts again.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param sigGenDev	[in] BB SigGen device
 *  @param ssel	[in] sync select to drive BB SigGen
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_issueSyncUpdateJesdTxSigGen
(
    DFE_Handle hDfe,
    DfeFl_JesdTxSignalGen sigGenDev,
    DfeFl_MiscSyncGenSig ssel
)
{
    DfeFl_Status status;
    DfeFl_JesdTxTestGenSselConfig sselCfg;
    
    VALID_DFE_HANDLE(hDfe);
    
    sselCfg.tgDev = sigGenDev;
    sselCfg.ssel = ssel;
    
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_SET_TESTGEN_SSEL, &sselCfg) );
    
    return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}    


/**
 * @brief Program JESD Tx Testbus
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * DFE has many test probe points scattered over all sub-modules.
 * CB can be used to capture a train of IQ bus signals at the probe. 
 * The API enables the specified JESDTX test probe to CB interface.
 *
 * NOTE, all test probes "AND together" shares single CB interface.
 * So software should enable no more than one probe at any time.
 * LLD internally disables all probes first before arm a new one.  
 *
 *  @param hDfe	[in] DFE device handle
 *  @param tp	[in] probe position 
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progJesdTxTestbus
(
    DFE_Handle hDfe,
    DfeFl_JesdTxTestBusSel tp
)
{
    DfeFl_Status status;
  	uint32_t data = tp;

    VALID_DFE_HANDLE(hDfe);
    
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_TEST_BUS, &data) );
    
    return DFE_ERR_NONE;
}    

/**
 * @brief Get JESD Tx Link Status
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * The API get back following Tx link status.
 *   @verbatim
      // Jesd Tx link status
      typedef struct
      {
          // first sync request received for the link
          //    0 - not seen first sync request
          //    1 - seen first sync request
          Uint32 firstSyncRequest[DFE_FL_JESD_NUM_LINK];
          // error count as reported over SYNC~ interface. 
          Uint32 syncErrCount[DFE_FL_JESD_NUM_LINK];
          // SYSREF alignment counter bits
          Uint32 sysrefAlignCount;
          // captured interrupt bit for sysref_request_assert
          //    0 - sysref request not asserted
          //    1 - sysref request asserted
          Uint32 sysrefReqAssert;
          // captured interrupt bit for sysref_request_deassert
          //    0 - sysref request not de-asserted
          //    1 - sysref request de-asserted
          Uint32 sysrefReqDeassert;
          // captured interrupt bit for sysref_err on the link
          //    0 - no sysref error
          //    1 - sysref error
          Uint32 sysrefErr[DFE_FL_JESD_NUM_LINK];
      } DFE_JesdTxLinkStatus;
     @endverbatim
 *
 *  @param hDfe	[in] DFE device handle
 *  @param linkStatus	[out] pointer to link status buffer 
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_getJesdTxLinkStatus
(
    DFE_Handle hDfe,
    DFE_JesdTxLinkStatus *linkStatus
)
{
    uint32_t ui;
    DfeFl_Status status;
    DfeFl_JesdTxFirstSyncRequestQuery qryFirstSyncReq;
    DfeFl_JesdLinkErrCntQuery qryErrCnt;
    DfeFl_JesdTxSysrefIntrs qrySysrefIntrs;
    
    VALID_DFE_HANDLE(hDfe);
    if(linkStatus == NULL)
    {
        Dfe_osalLog("NULL pointers passed in!");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    for(ui = 0; ui < DFE_FL_JESD_NUM_LINK; ui++)
    {
        qryFirstSyncReq.link = ui;
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_FIRST_SYNC_REQUEST, &qryFirstSyncReq) );
        linkStatus->firstSyncRequest[ui] = qryFirstSyncReq.request;
        
        qryErrCnt.link = ui;
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_LINK_ERR_CNT, &qryErrCnt) );
        linkStatus->syncErrCount[ui] = qryErrCnt.errCnt;
    }
    
    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_SYSREF_INTRGRP_STATUS, &qrySysrefIntrs) );
    linkStatus->sysrefErr[0]        = qrySysrefIntrs.errLink0Intr;
    linkStatus->sysrefErr[1]        = qrySysrefIntrs.errLink1Intr;
    linkStatus->sysrefReqAssert     = qrySysrefIntrs.reqAssertIntr;
    linkStatus->sysrefReqDeassert   = qrySysrefIntrs.reqDeassertIntr;
    
    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_SYSREF_ALGNCNT, &linkStatus->sysrefAlignCount) );
    
    return DFE_ERR_NONE;
}    

/**
 * @brief Get JESD Tx Lane Status
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * The API get back following Tx lane status:
 * @b Example
 *   @verbatim
     // Jesd Tx lane status
     typedef struct
     {
         // synchronization state machine status for lane    
         Uint32 syncState[DFE_FL_JESD_NUM_LANE];
         // FIFO status
         // 0 - fifo not empty; 1 - fifo has been empty
         Uint32 fifoEmpty[DFE_FL_JESD_NUM_LANE];
         // 0 - no read error; 1 - fifo read error
         Uint32 fifoReadErr[DFE_FL_JESD_NUM_LANE];
         // 0 - fifo not full; 1 - fifo has been full
         Uint32 fifoFull[DFE_FL_JESD_NUM_LANE];
         // 0 - no write error; 1 - fifo write error
         Uint32 fifoWriteErr[DFE_FL_JESD_NUM_LANE];
     } DFE_JesdTxLaneStatus;
     @endverbatim
 *
 *  @param hDfe	[in] DFE device handle
 *  @param laneStatus	[out] pointer to lane status buffer 
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_getJesdTxLaneStatus
(
    DFE_Handle hDfe,
    DFE_JesdTxLaneStatus *laneStatus
)
{
    uint32_t ui;
    DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);
    
    for(ui = 0; ui < DFE_FL_JESD_NUM_LANE; ui++)
    {
        DfeFl_JesdTxLaneSyncStateQuery qrySyncState;
        DfeFl_JesdTxLaneIntrs qryLaneIntrs;
        
        qrySyncState.lane = ui;
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_LANE_SYNC_STATE, &qrySyncState) );
        laneStatus->syncState[ui] = qrySyncState.state;
        
        qryLaneIntrs.lane = ui;
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_LANE_INTRGRP_STATUS, &qryLaneIntrs) );
        laneStatus->fifoEmpty[ui]      = qryLaneIntrs.fifoEmptyIntr;
        laneStatus->fifoReadErr[ui]    = qryLaneIntrs.fifoReadErrIntr;
        laneStatus->fifoFull[ui]       = qryLaneIntrs.fifoFullIntr;
        laneStatus->fifoWriteErr[ui]   = qryLaneIntrs.fifoWriteErrIntr;
    }
    
    return DFE_ERR_NONE;
}    


/**
 * @brief Clear JESD Tx Link Errors
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * The API clears following Tx link status,
 *  -	sysrefReqAssert
 *  -	sysrefReqDeassert
 *  -	sysrefErr[DFE_FL_JESD_NUM_LINK] for all links
 *  -	syncErrCount[DFE_FL_JESD_NUM_LINK] for all links
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_clearJesdTxLinkErrors
(
    DFE_Handle hDfe
)
{
    uint32_t ui;
    DfeFl_Status status;
    DfeFl_JesdTxSysrefIntrs clrSysrefIntrs;
    
    VALID_DFE_HANDLE(hDfe);

    clrSysrefIntrs.errLink0Intr     = 1;
    clrSysrefIntrs.errLink1Intr     = 1;
    clrSysrefIntrs.reqAssertIntr    = 1;
    clrSysrefIntrs.reqDeassertIntr  = 1;
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_SYSREF_INTRGRP_STATUS, &clrSysrefIntrs) );
    
    for(ui = 0; ui < DFE_FL_JESD_NUM_LINK; ui++)
    {
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_LINK_ERR_CNT, &ui) );
    }
   
    return DFE_ERR_NONE;
}    


/**
 * @brief Clear JESD Tx Lane Errors
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * The API clears following Tx lane status,
 *  -	fifoEmpty[DFE_FL_JESD_NUM_LANE] for all lanes
 *  -	fifoReadErr[DFE_FL_JESD_NUM_LANE] for all lanes
 *  -	fifoFull[DFE_FL_JESD_NUM_LANE] for all anes
 *  -	fifoWriteErr[DFE_FL_JESD_NUM_LANE] for all lanes
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_clearJesdTxLaneErrors
(
    DFE_Handle hDfe
)
{
    uint32_t ui;
    DfeFl_Status status;
    DfeFl_JesdTxLaneIntrs clrLaneIntrs;

    VALID_DFE_HANDLE(hDfe);

    clrLaneIntrs.fifoEmptyIntr      = 1;     
    clrLaneIntrs.fifoReadErrIntr    = 1; 
    clrLaneIntrs.fifoFullIntr       = 1;       
    clrLaneIntrs.fifoWriteErrIntr   = 1;
   
    for(ui = 0; ui < DFE_FL_JESD_NUM_LANE; ui++)
    {
        clrLaneIntrs.lane = ui;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_LANE_INTRGRP_STATUS, &clrLaneIntrs) );
    }
    
    return DFE_ERR_NONE;
}    


// Get JESD RX lane enable status
DFE_Err Dfe_getJesdRxLaneEnable
(
    DFE_Handle hDfe,
    uint32_t laneEnable[4],
    uint32_t linkAssign[4]
)
{
    uint32_t ui;
    DfeFl_Status status;
    DfeFl_JesdLaneConfig laneCfg;    
    
    VALID_DFE_HANDLE(hDfe);

    for(ui = 0; ui <= DFE_FL_JESD_LANE_3; ui++)
    {
        laneCfg.lane = ui;
        
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LANE_CFG, &laneCfg) );
        
        laneEnable[ui] = laneCfg.laneEnable;
        linkAssign[ui] = laneCfg.linkAssign;
    }    
    
    return DFE_ERR_NONE;
}

/**
 * @brief Program JESD Lane to Rx Map
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * Program how to fill time slots of a RX bus with lane time slots. 
 *
 * A Rx lane consists of four time slots. A Rx bus time slot, an
 * element of busMap[], is selected by the lane# and slot# of the lane.
 *
 * ~~~{.c}
 * // Jesd Rx lane to bus map
 * typedef struct
 * {
 *     // Rx lane#, 0 ~ 3
 *     Uint32 lane;
 *     // lane time slot
 *     Uint32 lanePos;
 *     // if zero data
 *     Uint32 zeroBits;
 * } DFE_JesdRxBusMapPos;
 * ~~~
 *
 * A Rx bus also consists of four time slots: 
 *  -	busMap[0] is mapping to bus time slot 0 
 *  -	busMap[1] is mapping to bus time slot 1
 *  -	busMap[2] is mapping to bus time slot 2 
 *  -	busMap[3] is mapping to bus time slot 3
 *
 * For example, two Rx lanes, lane0 and lane, are mapping to RX0,
 * which will carry two interleaved antenna streams A0 and A1,
 * lane0 => A0, lane1 => A1.
 *
 * RX0 bus format:
 *
 *  | Bus#      | Slot0 | Slot1 | Slot2      | Slot3      |
 *  | --------- | ----- | ----- | ---------- | ---------- |
 *  | 0 (RX0_I) | A0_i  | A1_i  | Don't care | Don't care |
 *  | 1 (RX0_Q) | A0_q  | A1_q  | Don't care | Don't care |
 *
 * Lane bus format:
 *
 *  | Lane#     | Slot0 | Slot1 | Slot2      | Slot3      |
 *  | --------- | ----- | ----- | ---------- | ---------- |
 *  | 0 (lane0) | A0_i  | A0_q  | Don't care | Don't care |
 *  | 1 (lane1) | A1_i  | A1_q  | Don't care | Don't care |
 *
 * Then busMap for RX0_I should be:
 *
 *  | x      | busMap[0] | busMap[1] | busMap[2]  | busMap[3]  |
 *  | ------ | --------- | --------- | ---------- | ---------- |
 *  | lane#  | 0         | 1         | Don't care | Don't care |
 *  | Slot#  | 0         | 0         | Don't care | Don't care |
 *
 * Then busMap for RX0_Q should be:
 *
 *  | x     | busMap[0] | busMap[1] | busMap[2]  | busMap[3]  |
 *  | ----- | --------- | --------- | ---------- | ---------- |
 *  | lane#	| 0         | 1         | Don't care | Don't care |
 *  | Slot#	| 1         | 1         | Don't care | Don't care |
 *
 *  @param hDfe	[in] DFE device handle
 *  @param rxBus	[in] Rx bus#
 *  @param busMap	[in] Rx lane slot map
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_mapJesdLane2Rx
(
    DFE_Handle hDfe,
    uint32_t rxBus,
    DFE_JesdRxBusMapPos busMap[4]
)
{
    DfeFl_Status status;
    uint32_t busPos, busNibb;
    DfeFl_JesdRxMapNibbConfig cfg;
    
    VALID_DFE_HANDLE(hDfe);

    cfg.rxBus = rxBus;
    
    for(busPos = 0; busPos < 4; busPos++)
    {
        for(busNibb = 0; busNibb < 4; busNibb++)
        {
            cfg.laneSel[busNibb][busPos]        = busMap[busPos].lane;
            cfg.laneNibbSel[busNibb][busPos]    = busNibb;
            cfg.timeSlotSel[busNibb][busPos]    = busMap[busPos].lanePos;
            cfg.zeroBits[busNibb][busPos]       = busMap[busPos].zeroBits;
        }
    }

    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_MAP_NIBB, &cfg) );    
    
    return DFE_ERR_NONE;
}    


/**
 * @brief Program JESD Rx Testbus
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * DFE has many test probe points scattered over all sub-modules.
 * CB can be used to capture a train of IQ bus signals at the probe. 
 * The API enables the specified JESDRX test probe to CB interface.
 *
 * NOTE, all test probes "AND together" shares single CB interface.
 * So software should enable no more than one probe at any time.
 * LLD internally disables all probes first before arm a new one.  
 *
 *  @param hDfe	[in] DFE device handle
 *  @param testCbCtrl	[in] probe position 
 *  @param testCbAxc	[in] probe axc/buf number  
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progJesdRxTestbus
(
    DFE_Handle hDfe,
    DfeFl_JesdRxTestBusSel tp
)
{
    DfeFl_Status status;
	uint32_t data = tp;
	
    VALID_DFE_HANDLE(hDfe);

	CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_SET_TESTBUS_SEL, &data) );
	
    return DFE_ERR_NONE;    	
}    


// Init JESD tx state machine.
/** ============================================================================
 *   @n@b Dfe_initJesdTx
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
 *       hDfe    [add content]
 *   @endverbatim
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
DFE_Err Dfe_initJesdTx
(
    DFE_Handle hDfe
)
{
    DfeFl_Status status;
    uint32_t initSsel, alwaysSync, initState;

    VALID_DFE_HANDLE(hDfe);

    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_INITSSEL, &initSsel) );

    alwaysSync = 0xF;
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITSSEL, &alwaysSync) );

    initState = 1;
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITSTATE, &initState) );

    initState = 0;
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITSTATE, &initState) );

    // change back to the original sync
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITSSEL, &initSsel) );
    return DFE_ERR_NONE;
}


// Init JESD rx state machine.
/** ============================================================================
 *   @n@b Dfe_initJesdRx
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
 *       hDfe    [add content]
 *   @endverbatim
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
DFE_Err Dfe_initJesdRx
(
    DFE_Handle hDfe
)
{
    DfeFl_Status status;
    uint32_t initSsel, alwaysSync, initState;

    VALID_DFE_HANDLE(hDfe);

    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_INITSSEL, &initSsel) );

    alwaysSync = 0xF;
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITSSEL, &alwaysSync) );

    initState = 1;
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITSTATE, &initState) );

    initState = 0;
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITSTATE, &initState) );

    // change back to the original sync
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITSSEL, &initSsel) );
    return DFE_ERR_NONE;
}

/**
 * @brief Program JESD Loopback
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * Enable/disable sync_n, lanes/links loopback between JESDTX and JESDRX. 
 *
 *  @param hDfe	[in] DFE device handle
 *  @param lpbkSync	[in] 1: enable rx sync out loopback to tx sync_n
 *  @param lpbkLaneLink	[in] lane/link loopback config
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progJesdLoopback
(
    DFE_Handle hDfe,
    uint32_t lpbkSync[DFE_FL_JESD_NUM_LINK],
    DfeFl_JesdRxLoopbackConfig *lpbkLaneLink
)
{
    DfeFl_Status status;
    DfeFl_JesdTxSyncnLoopbackConfig syncLpbk;
    
    VALID_DFE_HANDLE(hDfe);
    if(lpbkSync == NULL || lpbkLaneLink == NULL)
    {
        Dfe_osalLog("NULL pointers passed in!");
        return DFE_ERR_INVALID_PARAMS;
    }
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_LOOPBACK, lpbkLaneLink) );
    
    syncLpbk.link = DFE_FL_JESD_LINK_0;
    syncLpbk.syncnLoopback = lpbkSync[0];
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_SYNCN_LOOPBACK, &syncLpbk) );
    
    syncLpbk.link = DFE_FL_JESD_LINK_1;
    syncLpbk.syncnLoopback = lpbkSync[1];
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_SYNCN_LOOPBACK, &syncLpbk) );
    
    return DFE_ERR_NONE;    	
}    


// Get current JESD loopbacks for sync_n, lanes or links.
/** ============================================================================
 *   @n@b Dfe_getJesdLoopback
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]
         lpbkSync    [add content]
         lpbkLaneLink    [add content]
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
DFE_Err Dfe_getJesdLoopback
(
    DFE_Handle hDfe,
    uint32_t lpbkSync[DFE_FL_JESD_NUM_LINK],
    DfeFl_JesdRxLoopbackConfig *lpbkLaneLink
)
{
    DfeFl_Status status;
    DfeFl_JesdTxSyncnLoopbackConfig syncLpbk;
    
    VALID_DFE_HANDLE(hDfe);
    if(lpbkSync == NULL || lpbkLaneLink == NULL)
    {
        Dfe_osalLog("NULL pointers passed in!");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LOOPBACK, lpbkLaneLink) );
    
    syncLpbk.link = DFE_FL_JESD_LINK_0;
    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_SYNCN_LOOPBACK, &syncLpbk) );
    lpbkSync[0] = syncLpbk.syncnLoopback;
    
    syncLpbk.link = DFE_FL_JESD_LINK_1;
    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_SYNCN_LOOPBACK, &syncLpbk) );
    lpbkSync[1] = syncLpbk.syncnLoopback;
    
    return DFE_ERR_NONE;    
}

/**
 * @brief Get JESD Rx Link Status
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * The API get back following Rx link status,
 * ~~~{.c}
 * // Jesd Rx link status
 * typedef struct
 * {
 *     // SYSREF alignment counter bits
 *     Uint32 sysrefAlignCount;
 *     // captured interrupt bit for sysref_request_assert
 *     //    0 - sysref request not asserted
 *     //    1 - sysref request asserted
 *     Uint32 sysrefReqAssert;
 *     // captured interrupt bit for sysref_request_deassert
 *     //    0 - sysref request not de-asserted
 *     //    1 - sysref request de-asserted
 *     Uint32 sysrefReqDeassert;
 *     // captured interrupt bit for sysref_err on the link
 *     //    0 - no sysref error
 *     //    1 - sysref error
 *     Uint32 sysrefErr[DFE_FL_JESD_NUM_LINK];
 * } DFE_JesdRxLinkStatus; 
 * ~~~
 *
 *  @param hDfe	[in] DFE device handle
 *  @param linkStatus	[out] pointer to link status buffer 
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_getJesdRxLinkStatus
(
    DFE_Handle hDfe,
    DFE_JesdRxLinkStatus *linkStatus
)
{
    DfeFl_Status status;
    uint32_t ui;
    DfeFl_JesdLinkErrCntQuery qryErrCnt;
    DfeFl_JesdRxSysrefIntrs qrySysrefIntrs;

    VALID_DFE_HANDLE(hDfe);
    if(linkStatus == NULL)
    {
        Dfe_osalLog("NULL pointers passed in!");
        return DFE_ERR_INVALID_PARAMS;
    }

    for(ui = 0; ui < DFE_FL_JESD_NUM_LINK; ui++)
    {
        qryErrCnt.link = ui;
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LINK_ERR_CNT, &qryErrCnt) );
        linkStatus->syncErrCount[ui] = qryErrCnt.errCnt;
    }

    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_SYSREF_INTRGRP_STATUS, &qrySysrefIntrs) );
    linkStatus->sysrefErr[0]        = qrySysrefIntrs.errLink0Intr;
    linkStatus->sysrefErr[1]        = qrySysrefIntrs.errLink1Intr;
    linkStatus->sysrefReqAssert     = qrySysrefIntrs.reqAssertIntr;
    linkStatus->sysrefReqDeassert   = qrySysrefIntrs.reqDeassertIntr;

    CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_SYSREF_ALGNCNT, &linkStatus->sysrefAlignCount) );
    
    return DFE_ERR_NONE;
}    


/**
 * @brief Get JESD Rx Lane Status
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * The API get back following Tx lane status,
 * ~~~{.c}
 * // Jesd Rx lane status
 * typedef struct
 * {
 *     // code group synchronization state machine status for lane        
 *     Uint32 syncStatecodeState[DFE_FL_JESD_NUM_LANE];    
 *     // frame synchronization state machine status for lane
 * Uint32 frameState[DFE_FL_JESD_NUM_LANE];    
 * // 0 - no error; 1 - 8B/10B disparity error
 *     Uint32 decDispErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - no error; 1 - 8B/10B not-in-table code error
 *     Uint32 decCodeErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - no error; 1 - code group sync error
 *     Uint32 codeSyncErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - no error; 1 - elastic buffer match error 
 *     //(first non-/K/ doesn't match match_ctrl and match_data)
 *     Uint32 bufMatchErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - no error; 1 - elastic buffer overflow error (bad RBD value)
 *     Uint32 bufOverflowErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - no error; 1 - link configuration error
 *     Uint32 linkConfigErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - no error; 1 - frame alignment error
 *     Uint32 frameAlignErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - no error; 1 - multiframe alignment error
 *     Uint32 multiframeAlignErr[DFE_FL_JESD_NUM_LANE];
 *     // FIFO status
 *     // 0 - normal; 1 - fifo empty
 *     Uint32 fifoEmpty[DFE_FL_JESD_NUM_LANE];
 *     // 0 - normal; 1 - fifo read error
 *     Uint32 fifoReadErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - normal; 1 - fifo full
 *     Uint32 fifoFull[DFE_FL_JESD_NUM_LANE];
 *     // 0 - normal; 1 - fifo write error
 *     Uint32 fifoWriteErr[DFE_FL_JESD_NUM_LANE];
 *     // 0 - normal; 1 - test sequence verification failed
 *     Uint32 testSeqErr[DFE_FL_JESD_NUM_LANE];
 * } DFE_JesdRxLaneStatus;
 * ~~~
 *
 *  @param hDfe	[in] DFE device handle
 *  @param laneStatus	[out] pointer to lane status buffer 
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_getJesdRxLaneStatus
(
    DFE_Handle hDfe,
    DFE_JesdRxLaneStatus *laneStatus
)
{
    uint32_t ui;
    DfeFl_Status status;

    VALID_DFE_HANDLE(hDfe);

    for(ui = 0; ui < DFE_FL_JESD_NUM_LANE; ui++)
    {
        DfeFl_JesdRxLaneSyncStateQuery qrySyncState;
        DfeFl_JesdRxLaneIntrs qryLaneIntrs;
                
        qrySyncState.lane = ui;
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LANE_SYNC_STATE, &qrySyncState) );
        laneStatus->codeState[ui]              = qrySyncState.codeState;
        laneStatus->frameState[ui]             = qrySyncState.frameState;

        qryLaneIntrs.lane = ui;
        CSL_HW_QUERY( dfeFl_JesdGetHwStatus(hDfe->hDfeJesd[0], DFE_FL_JESDRX_QUERY_GET_LANE_INTRGRP_STATUS, &qryLaneIntrs) );        
        laneStatus->decDispErr[ui]             = qryLaneIntrs.decDispErrIntr;
        laneStatus->decCodeErr[ui]             = qryLaneIntrs.decCodeErrIntr;
        laneStatus->codeSyncErr[ui]            = qryLaneIntrs.codeSyncErrIntr;
        laneStatus->bufMatchErr[ui]            = qryLaneIntrs.bufMatchErrIntr;
        laneStatus->bufOverflowErr[ui]         = qryLaneIntrs.bufOverflowErrIntr;
        laneStatus->linkConfigErr[ui]          = qryLaneIntrs.linkConfigErrIntr;
        laneStatus->frameAlignErr[ui]          = qryLaneIntrs.frameAlignErrIntr;
        laneStatus->multiframeAlignErr[ui]     = qryLaneIntrs.multiframeAlignErrIntr;
        laneStatus->fifoEmpty[ui]              = qryLaneIntrs.fifoEmptyIntr;
        laneStatus->fifoReadErr[ui]            = qryLaneIntrs.fifoReadErrIntr;
        laneStatus->fifoFull[ui]               = qryLaneIntrs.fifoFullIntr;
        laneStatus->fifoWriteErr[ui]           = qryLaneIntrs.fifoWriteErrIntr;
        laneStatus->testSeqErr[ui]             = qryLaneIntrs.testSeqErrIntr;
    }
    
    return DFE_ERR_NONE;
}    


/**
 * @brief Clear JESD Rx Link Error
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * The API clears following Tx link status,
 *  -	sysrefReqAssert
 *  -	sysrefReqDeassert
 *  -	sysrefErr[DFE_FL_JESD_NUM_LINK] for all links
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_clearJesdRxLinkErrors
(
    DFE_Handle hDfe
)
{
    DfeFl_Status status;
    DfeFl_JesdRxSysrefIntrs clrSysrefIntrs;

    VALID_DFE_HANDLE(hDfe);

    clrSysrefIntrs.errLink0Intr     = 1;
    clrSysrefIntrs.errLink1Intr     = 1;
    clrSysrefIntrs.reqAssertIntr    = 1;
    clrSysrefIntrs.reqDeassertIntr  = 1;
    CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CLR_SYSREF_INTRGRP_STATUS, &clrSysrefIntrs) );
    
    return DFE_ERR_NONE;
}    


/**
 * @brief Clear JESD Rx Lane Error
 * @ingroup DFE_LLD_JESD_FUNCTION
 *
 * The API clears following lane status for all Tx lanes,
 *  -	decDispErr[DFE_FL_JESD_NUM_LANE];
 *  -	decCodeErr[DFE_FL_JESD_NUM_LANE];
 *  -	codeSyncErr[DFE_FL_JESD_NUM_LANE];
 *  -	bufMatchErr[DFE_FL_JESD_NUM_LANE];
 *  -	bufOverflowErr[DFE_FL_JESD_NUM_LANE];
 *  -	linkConfigErr[DFE_FL_JESD_NUM_LANE];
 *  -	frameAlignErr[DFE_FL_JESD_NUM_LANE];
 *  -	multiframeAlignErr[DFE_FL_JESD_NUM_LANE];
 *  -	fifoEmpty[DFE_FL_JESD_NUM_LANE];
 *  -	fifoReadErr[DFE_FL_JESD_NUM_LANE];
 *  -	fifoFull[DFE_FL_JESD_NUM_LANE];
 *  -	fifoWriteErr[DFE_FL_JESD_NUM_LANE]; 
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_clearJesdRxLaneErrors
(
    DFE_Handle hDfe
)
{
    uint32_t ui;
    DfeFl_Status status;
    DfeFl_JesdRxLaneIntrs clrLaneIntrs;
    
    VALID_DFE_HANDLE(hDfe);

    clrLaneIntrs.decDispErrIntr         = 1;         
    clrLaneIntrs.decCodeErrIntr         = 1;         
    clrLaneIntrs.codeSyncErrIntr        = 1;        
    clrLaneIntrs.bufMatchErrIntr        = 1;        
    clrLaneIntrs.bufOverflowErrIntr     = 1;     
    clrLaneIntrs.linkConfigErrIntr      = 1;      
    clrLaneIntrs.frameAlignErrIntr      = 1;      
    clrLaneIntrs.multiframeAlignErrIntr = 1; 
    clrLaneIntrs.fifoEmptyIntr          = 1;          
    clrLaneIntrs.fifoReadErrIntr        = 1;        
    clrLaneIntrs.fifoFullIntr           = 1;           
    clrLaneIntrs.fifoWriteErrIntr       = 1;       
    clrLaneIntrs.testSeqErrIntr         = 1;         
    
    for(ui = 0; ui < DFE_FL_JESD_NUM_LANE; ui++)
    {
        clrLaneIntrs.lane = ui;
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_CLR_LANE_INTRGRP_STATUS, &clrLaneIntrs) );
    }
    
    return DFE_ERR_NONE;
    
}    

