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

/** @file dfe_fl_jesdAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_JESD CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_JESDAUX_H_
#define _DFE_FL_JESDAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_jesd.h>

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// JESD TX /////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
dfeFl_JesdTxConfigInits(DfeFl_JesdHandle hJesd, DfeFl_JesdInitsConfig * arg)
{
    
    uint32_t data = hJesd->regs->jesdtx_base_inits;
    CSL_DFE_JESD_JESDTX_BASE_INITS_REG *pData = (CSL_DFE_JESD_JESDTX_BASE_INITS_REG *)&data;
    
    pData->inits_ssel       = arg->cmn.ssel;
    pData->init_clk_gate    = arg->cmn.initClkGate;
    pData->init_state       = arg->cmn.initState;
    pData->clear_data       = arg->cmn.clearData;
    pData->clear_data_lane0 = arg->clearDataLane[0];
    pData->clear_data_lane1 = arg->clearDataLane[1];   
    pData->clear_data_lane2 = arg->clearDataLane[2];   
    pData->clear_data_lane3 = arg->clearDataLane[3]; 
    
    hJesd->regs->jesdtx_base_inits = data;    
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigInitSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         initSsel    [add content]
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
dfeFl_JesdTxConfigInitSsel(DfeFl_JesdHandle hJesd, uint32_t initSsel)
{
    
    uint32_t data = hJesd->regs->jesdtx_base_inits;
	
	CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_INITS_SSEL, initSsel);

	hJesd->regs->jesdtx_base_inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetInitSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         initSsel    [add content]
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
dfeFl_JesdTxGetInitSsel(DfeFl_JesdHandle hJesd, uint32_t * initSsel)
{
    
    uint32_t data = hJesd->regs->jesdtx_base_inits;
	
	*initSsel = CSL_FEXT(data, DFE_JESD_JESDTX_BASE_INITS_REG_INITS_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigInitState
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         initState    [add content]
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
dfeFl_JesdTxConfigInitState(DfeFl_JesdHandle hJesd, uint32_t initState)
{
    
    uint32_t data = hJesd->regs->jesdtx_base_inits;
	
	CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_INIT_STATE, initState);

	hJesd->regs->jesdtx_base_inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetClearDataLane
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
         clearData    [add content]
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
 *       DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE1
 *       DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE0
 *       DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE3
 *       DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetClearDataLane(DfeFl_JesdHandle hJesd, uint32_t lane, uint32_t clearData)
{
    uint32_t data = hJesd->regs->jesdtx_base_inits;
    
    if(lane == DFE_FL_JESD_LANE_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE0, clearData);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE1, clearData);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE2, clearData);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE3, clearData);
    }
    else if(lane == DFE_FL_JESD_LANE_0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE0, clearData);
    }
    else if(lane == DFE_FL_JESD_LANE_1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE1, clearData);
    }
    else if(lane == DFE_FL_JESD_LANE_2)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE2, clearData);
    }
    else if(lane == DFE_FL_JESD_LANE_3)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_INITS_REG_CLEAR_DATA_LANE3, clearData);
    }                    
    
    hJesd->regs->jesdtx_base_inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigTxInputs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_CKEN_DLY_TX1
 *       DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_CKEN_DLY_TX0
 *       DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_RXTX_LPBK_ENA_TX0
 *       DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_RXTX_LPBK_ENA_TX1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxConfigTxInputs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxInputsConfig *arg)
{
    uint32_t data;
    
    data = hJesd->regs->jesdtx_base_tx_inputs;
    
    if(arg->txPath == DFE_FL_JESDTX_TX_ALL)
    {
        data  = CSL_FMK(DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_CKEN_DLY_TX0, arg->ckenDly)
              | CSL_FMK(DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_CKEN_DLY_TX1, arg->ckenDly)
              | CSL_FMK(DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_RXTX_LPBK_ENA_TX0, arg->rxtxLoopbackEnable)
              | CSL_FMK(DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_RXTX_LPBK_ENA_TX1, arg->rxtxLoopbackEnable);
    }
    else if(arg->txPath == DFE_FL_JESDTX_TX_0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_CKEN_DLY_TX0, arg->ckenDly);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_RXTX_LPBK_ENA_TX0, arg->rxtxLoopbackEnable);        
    }
    else if(arg->txPath == DFE_FL_JESDTX_TX_1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_CKEN_DLY_TX1, arg->ckenDly);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TX_INPUTS_REG_RXTX_LPBK_ENA_TX1, arg->rxtxLoopbackEnable);        
    }
    
    hJesd->regs->jesdtx_base_tx_inputs = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetTestBusSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         testBusSel    [add content]
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
 *       DFE_JESD_JESDTX_BASE_TEST_BUS_SEL_REG_TEST_BUS_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetTestBusSel(DfeFl_JesdHandle hJesd, uint32_t testBusSel)
{
    hJesd->regs->jesdtx_base_test_bus_sel = CSL_FMK(DFE_JESD_JESDTX_BASE_TEST_BUS_SEL_REG_TEST_BUS_SEL, testBusSel);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetTestSeqSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
         testSeqSel    [add content]
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
 *       DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE0
 *       DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE1
 *       DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE2
 *       DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE3
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetTestSeqSel(DfeFl_JesdHandle hJesd, uint32_t lane, uint32_t testSeqSel)
{
    uint32_t data = hJesd->regs->jesdtx_base_test_seq_sel;
    if(lane == DFE_FL_JESD_LANE_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE0, testSeqSel);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE1, testSeqSel);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE2, testSeqSel);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE3, testSeqSel);
    }
    else if(lane == DFE_FL_JESD_LANE_0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE0, testSeqSel);
    }
    else if(lane == DFE_FL_JESD_LANE_1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE1, testSeqSel);
    }
    else if(lane == DFE_FL_JESD_LANE_2)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE2, testSeqSel);
    }
    else if(lane == DFE_FL_JESD_LANE_3)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_TEST_SEQ_SEL_REG_LANE3, testSeqSel);
    }
    
    hJesd->regs->jesdtx_base_test_seq_sel = data;   
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetSyncnLoopback
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
         syncnLoopback    [add content]
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
 *       DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK1
 *       DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetSyncnLoopback(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t syncnLoopback)
{
    uint32_t data = hJesd->regs->jesdtx_base_sync_n;

    if(link == DFE_FL_JESD_LINK_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK0, syncnLoopback);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK1, syncnLoopback);
    }
    else if(link == DFE_FL_JESD_LINK_0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK0, syncnLoopback);
    }
    else if(link == DFE_FL_JESD_LINK_1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK1, syncnLoopback);
    }
    
    hJesd->regs->jesdtx_base_sync_n = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetSyncnLoopback
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
         syncnLoopback    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK1
 *       DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK0
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdTxGetSyncnLoopback(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t *syncnLoopback)
{
    uint32_t data = hJesd->regs->jesdtx_base_sync_n;

    if(link == DFE_FL_JESD_LINK_ALL)
    {
        *syncnLoopback = 0xdeadbeefu;
        return DFE_FL_INVPARAMS;
    }
    else if(link == DFE_FL_JESD_LINK_0)
    {
        *syncnLoopback = CSL_FEXT(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK0);
    }
    else if(link == DFE_FL_JESD_LINK_1)
    {
        *syncnLoopback = CSL_FEXT(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_LPBK_ENA_LINK1);
    }    
    
    return DFE_FL_SOK;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetSyncnPolarityInvert
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
         invert    [add content]
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
 *       DFE_JESD_JESDTX_BASE_SYNC_N_REG_INV_LINK1
 *       DFE_JESD_JESDTX_BASE_SYNC_N_REG_INV_LINK0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetSyncnPolarityInvert(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t invert)
{
    uint32_t data = hJesd->regs->jesdtx_base_sync_n;

    if(link == DFE_FL_JESD_LINK_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_INV_LINK0, invert);
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_INV_LINK1, invert);
    }
    else if(link == DFE_FL_JESD_LINK_0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_INV_LINK0, invert);
    }
    else if(link == DFE_FL_JESD_LINK_1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYNC_N_REG_INV_LINK1, invert);
    }

    hJesd->regs->jesdtx_base_sync_n = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetBbCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         laneEnable    [add content]
         linkSel    [add content]
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
 *       DFE_JESD_JESDTX_BASE_BB_CTRL_REG_BB_LINK_SEL
 *       DFE_JESD_JESDTX_BASE_BB_CTRL_REG_BB_LANE_ENA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetBbCtrl(DfeFl_JesdHandle hJesd, uint32_t laneEnable, uint32_t linkSel)
{
    hJesd->regs->jesdtx_base_bb_ctrl = CSL_FMK(DFE_JESD_JESDTX_BASE_BB_CTRL_REG_BB_LANE_ENA, laneEnable)
                                     | CSL_FMK(DFE_JESD_JESDTX_BASE_BB_CTRL_REG_BB_LINK_SEL, linkSel);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxClearBbErr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
dfeFl_JesdTxClearBbErr(DfeFl_JesdHandle hJesd)
{
    hJesd->regs->jesdtx_base_bb_err = ~1u;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetBbErr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
dfeFl_JesdTxGetBbErr(DfeFl_JesdHandle hJesd)
{
    return (hJesd->regs->jesdtx_base_bb_err);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetFifoReadDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         dly    [add content]
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
 *       DFE_JESD_JESDTX_BASE_FIFO_REG_FIFO_READ_DELAY
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetFifoReadDelay(DfeFl_JesdHandle hJesd, uint32_t dly)
{
    CSL_FINS(hJesd->regs->jesdtx_base_fifo, DFE_JESD_JESDTX_BASE_FIFO_REG_FIFO_READ_DELAY, dly);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetFifoReadDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_BASE_FIFO_REG_FIFO_READ_DELAY
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
dfeFl_JesdTxGetFifoReadDelay(DfeFl_JesdHandle hJesd)
{
    return CSL_FEXT(hJesd->regs->jesdtx_base_fifo, DFE_JESD_JESDTX_BASE_FIFO_REG_FIFO_READ_DELAY);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetFifoErrorZeroData
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         disableZeroData    [add content]
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
 *       DFE_JESD_JESDTX_BASE_FIFO_REG_DISABLE_FIFO_ERRORS_ZERO_DATA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetFifoErrorZeroData(DfeFl_JesdHandle hJesd, uint32_t disableZeroData)
{
    CSL_FINS(hJesd->regs->jesdtx_base_fifo, DFE_JESD_JESDTX_BASE_FIFO_REG_DISABLE_FIFO_ERRORS_ZERO_DATA, disableZeroData);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetFifoErrorZeroData
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         disableZeroData    [add content]
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
 *       DFE_JESD_JESDTX_BASE_FIFO_REG_DISABLE_FIFO_ERRORS_ZERO_DATA
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
dfeFl_JesdTxGetFifoErrorZeroData(DfeFl_JesdHandle hJesd, uint32_t *disableZeroData)
{
    *disableZeroData  = CSL_FEXT(hJesd->regs->jesdtx_base_fifo, DFE_JESD_JESDTX_BASE_FIFO_REG_DISABLE_FIFO_ERRORS_ZERO_DATA);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetSysrefDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         dly    [add content]
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
 *       DFE_JESD_JESDTX_BASE_SYSREF_REG_SYSREF_DLY_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetSysrefDelay(DfeFl_JesdHandle hJesd, uint32_t dly)
{
    CSL_FINS(hJesd->regs->jesdtx_base_sysref, DFE_JESD_JESDTX_BASE_SYSREF_REG_SYSREF_DLY_SEL, dly);
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxForceSysRefRequest
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         forceReq    [add content]
         autoOffCount    [add content]
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
 *       DFE_JESD_JESDTX_BASE_SYSREF_REG_FORCE_SYSREF_REQUEST_AUTO_OFF
 *       DFE_JESD_JESDTX_BASE_SYSREF_REG_FORCE_SYSREF_REQUEST
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxForceSysRefRequest(DfeFl_JesdHandle hJesd, uint32_t forceReq, uint32_t autoOffCount)
{
    uint32_t data = hJesd->regs->jesdtx_base_sysref;
    
    CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYSREF_REG_FORCE_SYSREF_REQUEST, forceReq);
    CSL_FINS(data, DFE_JESD_JESDTX_BASE_SYSREF_REG_FORCE_SYSREF_REQUEST_AUTO_OFF, autoOffCount);
    
    hJesd->regs->jesdtx_base_sysref = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetSysRefAlignmentCounter
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_BASE_SYSREF_CNTR_HI_REG_SYSREF_CNTR_31_16
 *       DFE_JESD_JESDTX_BASE_SYSREF_CNTR_LO_REG_SYSREF_CNTR_15_0
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
dfeFl_JesdTxGetSysRefAlignmentCounter(DfeFl_JesdHandle hJesd)
{
    return(CSL_FEXT(hJesd->regs->jesdtx_base_sysref_cntr_lo, DFE_JESD_JESDTX_BASE_SYSREF_CNTR_LO_REG_SYSREF_CNTR_15_0)
         |(CSL_FEXT(hJesd->regs->jesdtx_base_sysref_cntr_hi, DFE_JESD_JESDTX_BASE_SYSREF_CNTR_HI_REG_SYSREF_CNTR_31_16) << 16));
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetSyncState
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
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
 *       DFE_JESD_JESDTX_BASE_SYNC_STATE_REG_LANE0
 *       DFE_JESD_JESDTX_BASE_SYNC_STATE_REG_LANE1
 *       DFE_JESD_JESDTX_BASE_SYNC_STATE_REG_LANE2
 *       DFE_JESD_JESDTX_BASE_SYNC_STATE_REG_LANE3
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
dfeFl_JesdTxGetSyncState(DfeFl_JesdHandle hJesd, uint32_t lane)
{
    uint32_t data = hJesd->regs->jesdtx_base_sync_state;
    
    if(lane == DFE_FL_JESD_LANE_0)
    {
        return CSL_FEXT(data, DFE_JESD_JESDTX_BASE_SYNC_STATE_REG_LANE0);
    }
    else if(lane == DFE_FL_JESD_LANE_1)
    {
        return CSL_FEXT(data, DFE_JESD_JESDTX_BASE_SYNC_STATE_REG_LANE1);
    }
    else if(lane == DFE_FL_JESD_LANE_2)
    {
        return CSL_FEXT(data, DFE_JESD_JESDTX_BASE_SYNC_STATE_REG_LANE2);
    }
    else if(lane == DFE_FL_JESD_LANE_3)
    {
        return CSL_FEXT(data, DFE_JESD_JESDTX_BASE_SYNC_STATE_REG_LANE3);
    }
    else
        // return all
        return data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetFirstSyncReqStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
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
 *       DFE_JESD_JESDTX_BASE_FIRST_SYNC_REQUEST_REG_LINK1
 *       DFE_JESD_JESDTX_BASE_FIRST_SYNC_REQUEST_REG_LINK0
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
dfeFl_JesdTxGetFirstSyncReqStatus(DfeFl_JesdHandle hJesd, uint32_t link)
{
    uint32_t data = hJesd->regs->jesdtx_base_first_sync_request;
    
    if(link == DFE_FL_JESD_LINK_0)
    {
        return CSL_FEXT(data, DFE_JESD_JESDTX_BASE_FIRST_SYNC_REQUEST_REG_LINK0);
    }
    else if(link == DFE_FL_JESD_LINK_1)
    {
        return CSL_FEXT(data, DFE_JESD_JESDTX_BASE_FIRST_SYNC_REQUEST_REG_LINK1);
    }
    else 
        // return all
        return data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetSignalGenSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         sigGen    [add content]
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
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXQ1
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXQ0
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXI0
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXI1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetSignalGenSsel(DfeFl_JesdHandle hJesd, uint32_t sigGen, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdtx_ssel_ssel_addr_0;
    
    if(sigGen == DFE_FL_JESDTX_SIGNAL_GEN_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXI0, ssel);
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXQ0, ssel);
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXI1, ssel);
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXQ1, ssel);
    }
    else if(sigGen == DFE_FL_JESDTX_SIGNAL_GEN_TXI0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXI0, ssel);
    }
    else if(sigGen == DFE_FL_JESDTX_SIGNAL_GEN_TXQ0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXQ0, ssel);
    }
    else if(sigGen == DFE_FL_JESDTX_SIGNAL_GEN_TXI1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXI1, ssel);
    }
    else if(sigGen == DFE_FL_JESDTX_SIGNAL_GEN_TXQ1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_0_REG_SIGNAL_GEN_SSEL_TXQ1, ssel);
    }
        
    hJesd->regs->jesdtx_ssel_ssel_addr_0 = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         chksumLane    [add content]
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
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE1
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE0
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE3
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetChksumSsel(DfeFl_JesdHandle hJesd, uint32_t chksumLane, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdtx_ssel_ssel_addr_1;
    
    if(chksumLane == DFE_FL_JESDTX_CHKSUM_LANE_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE0, ssel);
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE1, ssel);
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE2, ssel);
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE3, ssel);
    }
    else if(chksumLane == DFE_FL_JESDTX_CHKSUM_LANE_0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE0, ssel);
    }        
    else if(chksumLane == DFE_FL_JESDTX_CHKSUM_LANE_1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE1, ssel);
    }        
    else if(chksumLane == DFE_FL_JESDTX_CHKSUM_LANE_2)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE2, ssel);
    }        
    else if(chksumLane == DFE_FL_JESDTX_CHKSUM_LANE_3)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_1_REG_CHECK_SUM_SSEL_LANE3, ssel);
    }  
    
    hJesd->regs->jesdtx_ssel_ssel_addr_1 = data;      
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetLinkInitStateSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
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
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_INIT_STATE_SSEL_LINK1
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_INIT_STATE_SSEL_LINK0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetLinkInitStateSsel(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdtx_ssel_ssel_addr_2;

    if(link == DFE_FL_JESD_LINK_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_INIT_STATE_SSEL_LINK0, ssel);
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_INIT_STATE_SSEL_LINK1, ssel);
    }
    else if(link == DFE_FL_JESD_LINK_0)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_INIT_STATE_SSEL_LINK0, ssel);
    }
    else if(link == DFE_FL_JESD_LINK_1)
    {
        CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_INIT_STATE_SSEL_LINK1, ssel);
    }

    hJesd->regs->jesdtx_ssel_ssel_addr_2 = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetSysRefAlignmentCounterSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_SYSREF_CNTR_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetSysRefAlignmentCounterSsel(DfeFl_JesdHandle hJesd, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdtx_ssel_ssel_addr_2;

    CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_SYSREF_CNTR_SSEL, ssel);
    
    hJesd->regs->jesdtx_ssel_ssel_addr_2 = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetSysRefModeSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_TX_SYSREF_MODE_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxSetSysRefModeSsel(DfeFl_JesdHandle hJesd, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdtx_ssel_ssel_addr_2;

    CSL_FINS(data, DFE_JESD_JESDTX_SSEL_SSEL_ADDR_2_REG_TX_SYSREF_MODE_SSEL, ssel);
    
    hJesd->regs->jesdtx_ssel_ssel_addr_2 = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigTestGen
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_RAMP_MODE
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_START_LO_REG_RAMP_START_15_0
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_STOP_HI_REG_RAMP_STOP_31_16
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_START_HI_REG_RAMP_START_31_16
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_FRAME_LEN_M1
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_GEN_DATA
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GEN_TIMER_REG_GEN_TIMER
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_GEN_FRAME
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_SEED
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_SLOPE_LO_REG_RAMP_SLOPE_15_0
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_STOP_LO_REG_RAMP_STOP_15_0
 *       DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_SLOPE_HI_REG_RAMP_SLOPE_31_16
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_JesdTxConfigTestGen(DfeFl_JesdHandle hJesd, DfeFl_JesdTxTestGenConfig * arg)
{
    volatile uint32_t *regs[4];
    uint32_t ui, general, startLo, startHi, stopLo, stopHi, slopLo, slopHi, genTimer;
    
    regs[0] = &hJesd->regs->jesdtx_signal_gen_txi0_general;
    regs[1] = &hJesd->regs->jesdtx_signal_gen_txq0_general;
    regs[2] = &hJesd->regs->jesdtx_signal_gen_txi1_general;
    regs[3] = &hJesd->regs->jesdtx_signal_gen_txq1_general;
    
    general = CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_GEN_DATA, arg->genData)
        | CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_GEN_FRAME, arg->genFrame)
        | CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_RAMP_MODE, arg->rampMode)
        | CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_SEED, arg->seed)
        | CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GENERAL_REG_FRAME_LEN_M1, arg->frameLenM1);

    startLo = CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_START_LO_REG_RAMP_START_15_0, arg->rampStart);
    startHi = CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_START_HI_REG_RAMP_START_31_16, arg->rampStart >> 16);
            
    stopLo = CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_STOP_LO_REG_RAMP_STOP_15_0, arg->rampStop);
    stopHi = CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_STOP_HI_REG_RAMP_STOP_31_16, arg->rampStop >> 16);

    slopLo = CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_SLOPE_LO_REG_RAMP_SLOPE_15_0, arg->slope);
    slopHi = CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_RAMP_SLOPE_HI_REG_RAMP_SLOPE_31_16, arg->slope >> 16);
    
    genTimer = CSL_FMK(DFE_JESD_JESDTX_SIGNAL_GEN_TXI0_GEN_TIMER_REG_GEN_TIMER, arg->genTimer);
    
    if(arg->tgDev == DFE_FL_JESDTX_SIGNAL_GEN_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            regs[ui][0] = general;
            regs[ui][1] = startLo;
            regs[ui][2] = startHi;
            regs[ui][3] = stopLo;
            regs[ui][4] = stopHi;
            regs[ui][5] = slopLo;
            regs[ui][6] = slopHi;
            regs[ui][7] = genTimer;
        }
    }
    else if(arg->tgDev <= DFE_FL_JESDTX_CHKSUM_LANE_3)
    {
        ui = (uint32_t) arg->tgDev;
        regs[ui][0] = general;
        regs[ui][1] = startLo;
        regs[ui][2] = startHi;
        regs[ui][3] = stopLo;
        regs[ui][4] = stopHi;
        regs[ui][5] = slopLo;
        regs[ui][6] = slopHi;
        regs[ui][7] = genTimer;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE0_CHAN_SEL_REG_CHAN_SEL
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE0_CTRL_REG_MODE
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE0_CTRL_REG_STABLE_LEN
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE0_SIGNAL_LEN_REG_SIGNAL_LEN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_JesdTxConfigChksum(DfeFl_JesdHandle hJesd, DfeFl_JesdTxChksumConfig *arg)
{
    volatile uint32_t *regs[4];
    uint32_t ui, ctrl, sigLen, chanSel;
    
    
    regs[0] = &hJesd->regs->jesdtx_check_sum_lane0_ctrl;
    regs[1] = &hJesd->regs->jesdtx_check_sum_lane1_ctrl;
    regs[2] = &hJesd->regs->jesdtx_check_sum_lane2_ctrl;
    regs[3] = &hJesd->regs->jesdtx_check_sum_lane3_ctrl;

    // ctrl, stable_len
    ctrl = CSL_FMK(DFE_JESD_JESDTX_CHECK_SUM_LANE0_CTRL_REG_MODE, arg->chksumMode)
         | CSL_FMK(DFE_JESD_JESDTX_CHECK_SUM_LANE0_CTRL_REG_STABLE_LEN, arg->latencyMode.stableLen);

    // signal_len
    sigLen = CSL_FMK(DFE_JESD_JESDTX_CHECK_SUM_LANE0_SIGNAL_LEN_REG_SIGNAL_LEN, arg->latencyMode.signalLen);

    // chan_sel
    chanSel = CSL_FMK(DFE_JESD_JESDTX_CHECK_SUM_LANE0_CHAN_SEL_REG_CHAN_SEL, arg->latencyMode.chanSel);
    
    if(arg->chksumDev == DFE_FL_JESDTX_CHKSUM_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            regs[ui][0] = ctrl;
            regs[ui][1] = sigLen;
            regs[ui][2] = chanSel;
        }
    }
    else if(arg->chksumDev <= DFE_FL_JESDTX_CHKSUM_LANE_3)
    {
        ui = arg->chksumDev;
        regs[ui][0] = ctrl;
        regs[ui][1] = sigLen;
        regs[ui][2] = chanSel;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetChecksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         chksumDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE3_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE2_RESULT_LO_REG_RESULT_15_0
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE0_RESULT_LO_REG_RESULT_15_0
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE2_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE1_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE1_RESULT_LO_REG_RESULT_15_0
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE0_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDTX_CHECK_SUM_LANE3_RESULT_LO_REG_RESULT_15_0
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdTxGetChecksumResult(DfeFl_JesdHandle hJesd, uint32_t chksumDev, uint32_t *chksumResult)
{
    if(chksumDev == DFE_FL_JESDTX_CHKSUM_LANE_0)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdtx_check_sum_lane0_result_lo, DFE_JESD_JESDTX_CHECK_SUM_LANE0_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdtx_check_sum_lane0_result_hi, DFE_JESD_JESDTX_CHECK_SUM_LANE0_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else if(chksumDev == DFE_FL_JESDTX_CHKSUM_LANE_1)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdtx_check_sum_lane1_result_lo, DFE_JESD_JESDTX_CHECK_SUM_LANE1_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdtx_check_sum_lane1_result_hi, DFE_JESD_JESDTX_CHECK_SUM_LANE1_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else if(chksumDev == DFE_FL_JESDTX_CHKSUM_LANE_2)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdtx_check_sum_lane2_result_lo, DFE_JESD_JESDTX_CHECK_SUM_LANE2_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdtx_check_sum_lane2_result_hi, DFE_JESD_JESDTX_CHECK_SUM_LANE2_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else if(chksumDev == DFE_FL_JESDTX_CHKSUM_LANE_3)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdtx_check_sum_lane3_result_lo, DFE_JESD_JESDTX_CHECK_SUM_LANE3_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdtx_check_sum_lane3_result_hi, DFE_JESD_JESDTX_CHECK_SUM_LANE3_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else 
    {
        *chksumResult = 0xdeadbeef;
        return DFE_FL_INVPARAMS;
    }
    
    return DFE_FL_SOK;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigClockGate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
         cgCfg    [add content]
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
dfeFl_JesdTxConfigClockGate(DfeFl_JesdHandle hJesd, uint32_t link, DfeFl_ClockGateConfig *cgCfg)
{
    volatile uint32_t *regs[2];
    uint32_t ui;
    
    regs[0] = &hJesd->regs->jesdtx_clk_gater_link0_time_step;
    regs[1] = &hJesd->regs->jesdtx_clk_gater_link1_time_step;
    
    if(link == DFE_FL_JESD_LINK_ALL)
    {
        for(ui = 0; ui < 2; ui++)
        {
            regs[ui][0] = cgCfg->timeStep & 0xffffu;
            //regs[ui][1] = cgCfg->timeStep >> 16;
            regs[ui][2] = cgCfg->resetInterval & 0xffffu;
            regs[ui][3] = cgCfg->resetInterval >> 16;
            regs[ui][0] = cgCfg->tddPeriod & 0xffffu;
            regs[ui][1] = cgCfg->tddPeriod >> 16;
            regs[ui][0] = cgCfg->tddOn0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn0 >> 16;
            regs[ui][0] = cgCfg->tddOff0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff0 >> 16;
            regs[ui][0] = cgCfg->tddOn1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn1 >> 16;
            regs[ui][0] = cgCfg->tddOff1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff1 >> 16;
        }
    }
    else if(link <= DFE_FL_JESD_LINK_1)
    {
        ui = link;
        {
            regs[ui][0] = cgCfg->timeStep & 0xffffu;
            //regs[ui][1] = cgCfg->timeStep >> 16;
            regs[ui][2] = cgCfg->resetInterval & 0xffffu;
            regs[ui][3] = cgCfg->resetInterval >> 16;
            regs[ui][0] = cgCfg->tddPeriod & 0xffffu;
            regs[ui][1] = cgCfg->tddPeriod >> 16;
            regs[ui][0] = cgCfg->tddOn0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn0 >> 16;
            regs[ui][0] = cgCfg->tddOff0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff0 >> 16;
            regs[ui][0] = cgCfg->tddOn1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn1 >> 16;
            regs[ui][0] = cgCfg->tddOff1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff1 >> 16;
        }
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigLane
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
         enable    [add content]
         linkAssign    [add content]
         lid    [add content]
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
 *       DFE_JESD_JESDTX_LANE0_CFG_REG_LID
 *       DFE_JESD_JESDTX_LANE0_CFG_REG_LINK_ASSIGN
 *       DFE_JESD_JESDTX_LANE0_CFG_REG_LANE_ENA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxConfigLane(DfeFl_JesdHandle hJesd, uint32_t lane, uint32_t enable, uint32_t linkAssign, uint32_t lid)
{
    volatile uint32_t *regs[4];
    uint32_t data;
    
    regs[0] = &hJesd->regs->jesdtx_lane0_cfg;
    regs[1] = &hJesd->regs->jesdtx_lane1_cfg;
    regs[2] = &hJesd->regs->jesdtx_lane2_cfg;
    regs[3] = &hJesd->regs->jesdtx_lane3_cfg;
    
    data = CSL_FMK(DFE_JESD_JESDTX_LANE0_CFG_REG_LANE_ENA, enable)
         | CSL_FMK(DFE_JESD_JESDTX_LANE0_CFG_REG_LINK_ASSIGN, linkAssign)
         | CSL_FMK(DFE_JESD_JESDTX_LANE0_CFG_REG_LID, lid);
         
    if(lane == DFE_FL_JESD_LANE_ALL)
    {
        *regs[0] = data;
        *regs[1] = data;
        *regs[2] = data;
        *regs[3] = data;
    }
    else if(lane <= DFE_FL_JESD_LANE_3)
    {
        *regs[lane] = data;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetLaneConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
         enable    [add content]
         linkAssign    [add content]
         lid    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDTX_LANE0_CFG_REG_LID
 *       DFE_JESD_JESDTX_LANE0_CFG_REG_LINK_ASSIGN
 *       DFE_JESD_JESDTX_LANE0_CFG_REG_LANE_ENA
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
CSL_IDEF_INLINE DfeFl_Status 
dfeFl_JesdTxGetLaneConfig(DfeFl_JesdHandle hJesd, uint32_t lane, uint32_t *enable, uint32_t *linkAssign, uint32_t *lid)
{
    volatile uint32_t *regs[4];
    uint32_t data;
    
    regs[0] = &hJesd->regs->jesdtx_lane0_cfg;
    regs[1] = &hJesd->regs->jesdtx_lane1_cfg;
    regs[2] = &hJesd->regs->jesdtx_lane2_cfg;
    regs[3] = &hJesd->regs->jesdtx_lane3_cfg;
    
    *enable = 0xdeadbeefu;
    *linkAssign = 0xdeadbeefu;
    *lid = 0xdeadbeefu;
             
    if(lane <= DFE_FL_JESD_LANE_3)
    {
        data = *regs[lane];
        
        *enable     = CSL_FEXT(data, DFE_JESD_JESDTX_LANE0_CFG_REG_LANE_ENA);
        *linkAssign = CSL_FEXT(data, DFE_JESD_JESDTX_LANE0_CFG_REG_LINK_ASSIGN);
        *lid        = CSL_FEXT(data, DFE_JESD_JESDTX_LANE0_CFG_REG_LID);
        
        return DFE_FL_SOK;
    }
    else
    {
        return DFE_FL_INVPARAMS;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigLink
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_LINK0_CFG4_REG_JESDV
 *       DFE_JESD_JESDTX_LINK0_CFG1_REG_SCR
 *       DFE_JESD_JESDTX_LINK0_CFG2_REG_F_M1
 *       DFE_JESD_JESDTX_LINK0_CFG7_REG_ILA_MF_M1
 *       DFE_JESD_JESDTX_LINK0_CFG1_REG_ADJDIR
 *       DFE_JESD_JESDTX_LINK0_CFG2_REG_K_M1
 *       DFE_JESD_JESDTX_LINK0_CFG7_REG_MP_LINK_ENA
 *       DFE_JESD_JESDTX_LINK0_CFG5_REG_CF
 *       DFE_JESD_JESDTX_LINK0_CFG1_REG_L_M1
 *       DFE_JESD_JESDTX_LINK0_CFG4_REG_SUBCLASSV
 *       DFE_JESD_JESDTX_LINK0_CFG7_REG_NO_LANE_SYNC
 *       DFE_JESD_JESDTX_LINK0_CFG4_REG_NPRIME_M1
 *       DFE_JESD_JESDTX_LINK0_CFG1_REG_PHADJ
 *       DFE_JESD_JESDTX_LINK0_CFG5_REG_RES1
 *       DFE_JESD_JESDTX_LINK0_CFG0_REG_BID
 *       DFE_JESD_JESDTX_LINK0_CFG3_REG_CS
 *       DFE_JESD_JESDTX_LINK0_CFG6_REG_RES2
 *       DFE_JESD_JESDTX_LINK0_CFG3_REG_N_M1
 *       DFE_JESD_JESDTX_LINK0_CFG8_REG_SYSREF_MODE
 *       DFE_JESD_JESDTX_LINK0_CFG5_REG_HD
 *       DFE_JESD_JESDTX_LINK0_CFG3_REG_M_M1
 *       DFE_JESD_JESDTX_LINK0_CFG4_REG_S_M1
 *       DFE_JESD_JESDTX_LINK0_CFG0_REG_DID
 *       DFE_JESD_JESDTX_LINK0_CFG0_REG_ADJCNT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxConfigLink(DfeFl_JesdHandle hJesd, DfeFl_JesdTxLinkConfig *arg)
{
    volatile uint32_t *regs[2];
    Uint16 ui, cfg[10];
    
    regs[0] = &hJesd->regs->jesdtx_link0_cfg0;
    regs[1] = &hJesd->regs->jesdtx_link1_cfg0;
    
    cfg[0] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG0_REG_DID, arg->deviceId)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG0_REG_BID, arg->bankId)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG0_REG_ADJCNT, arg->adjCnt);

    cfg[1] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG1_REG_PHADJ, arg->phyAdj)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG1_REG_ADJDIR, arg->adjDir)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG1_REG_L_M1, arg->numLanesM1)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG1_REG_SCR, arg->scramble);

    cfg[2] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG2_REG_F_M1, arg->numOctetsPerFrameM1)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG2_REG_K_M1, arg->numFramesPerMultiframe);
           
    cfg[3] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG3_REG_M_M1, arg->numCnvtsM1)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG3_REG_N_M1, arg->cnvtResolutionM1)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG3_REG_CS, arg->numCtrlBits);
    
    cfg[4] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG4_REG_NPRIME_M1, arg->totalBitsPerSample)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG4_REG_SUBCLASSV, arg->subclass)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG4_REG_S_M1, arg->numSamplesPerFrameM1)
           |  CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG4_REG_JESDV, arg->jesd204Ver);
    
    cfg[5] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG5_REG_CF, arg->numCtrlWordsPerFrame)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG5_REG_HD, arg->hiDesity)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG5_REG_RES1, arg->rsvd1);

    cfg[6] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG6_REG_RES2, arg->rsvd2);
           
    cfg[7] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG7_REG_ILA_MF_M1, arg->ILAnumMultiframesM1)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG7_REG_NO_LANE_SYNC, arg->noLaneSync)
           | CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG7_REG_MP_LINK_ENA, arg->mpLinkEnable);
           
    cfg[8] = CSL_FMK(DFE_JESD_JESDTX_LINK0_CFG8_REG_SYSREF_MODE, arg->sysrefMode);
    
    if(arg->link == DFE_FL_JESD_LINK_ALL)
    {
        for(ui = 0; ui <= 8; ui++)
        {
            regs[0][ui] = cfg[ui];
            regs[1][ui] = cfg[ui];
        }
    }
    else if(arg->link <= DFE_FL_JESD_LINK_1)
    {
        for(ui = 0; ui <= 8; ui++)
        {
            regs[arg->link][ui] = cfg[ui];
        }        
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetSysrefMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
         mode    [add content]
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
 *       DFE_JESD_JESDTX_LINK0_CFG8_REG_SYSREF_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_JesdTxSetSysrefMode(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t mode)
{
    volatile uint32_t *regs[2];
    
    regs[0] = &hJesd->regs->jesdtx_link0_cfg0;
    regs[1] = &hJesd->regs->jesdtx_link1_cfg0;

    if(link <= DFE_FL_JESD_LINK_1)
    {
        CSL_FINS(regs[link][8], DFE_JESD_JESDTX_LINK0_CFG8_REG_SYSREF_MODE, mode);
    }
    else
    {
        CSL_FINS(regs[0][8], DFE_JESD_JESDTX_LINK0_CFG8_REG_SYSREF_MODE, mode);
        CSL_FINS(regs[1][8], DFE_JESD_JESDTX_LINK0_CFG8_REG_SYSREF_MODE, mode);
    }    
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetSysrefMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDTX_LINK0_CFG8_REG_SYSREF_MODE
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdTxGetSysrefMode(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t *sysrefMode)
{
    volatile uint32_t *regs[2];
    
    regs[0] = &hJesd->regs->jesdtx_link0_cfg0;
    regs[1] = &hJesd->regs->jesdtx_link1_cfg0;

    if(link <= DFE_FL_JESD_LINK_1)
    {
        *sysrefMode = CSL_FEXT(regs[link][8], DFE_JESD_JESDTX_LINK0_CFG8_REG_SYSREF_MODE);
        return DFE_FL_SOK;
    }
    else
    {
        *sysrefMode = 0xdeadbeefu;
        return DFE_FL_INVPARAMS;
    }    
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxClearLinkErrCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
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
dfeFl_JesdTxClearLinkErrCnt(DfeFl_JesdHandle hJesd, uint32_t link)
{
    volatile uint32_t *regs[2];
    
    regs[0] = &hJesd->regs->jesdtx_link0_cfg9;
    regs[1] = &hJesd->regs->jesdtx_link1_cfg9;
    
    if(link == DFE_FL_JESD_LINK_ALL)
    {
        // write '1' to clear
        *regs[0] = 1;
        *regs[1] = 1;
    }
    else if(link <= DFE_FL_JESD_LINK_1)
    {
        // write '1' to clear
        *regs[link] = 1;        
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetLinkErrCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDTX_LINK0_CFG9_REG_ERR_CNT
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdTxGetLinkErrCnt(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t *errCnt)
{
    volatile uint32_t *regs[2];
    
    regs[0] = &hJesd->regs->jesdtx_link0_cfg9;
    regs[1] = &hJesd->regs->jesdtx_link1_cfg9;
    
    if(link <= DFE_FL_JESD_LINK_1)
    {
        *errCnt = CSL_FEXT(*regs[link], DFE_JESD_JESDTX_LINK0_CFG9_REG_ERR_CNT);
        return DFE_FL_SOK;
    }
    else
    {
        *errCnt = 0xdeadbeefu;
        return DFE_FL_INVPARAMS;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxEnableLaneIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxEnableLaneIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *)&data;
        
    regs[0] = &hJesd->regs->jesdtx_intr_lane0_mask;    
    regs[1] = &hJesd->regs->jesdtx_intr_lane1_mask;    
    regs[2] = &hJesd->regs->jesdtx_intr_lane2_mask;    
    regs[3] = &hJesd->regs->jesdtx_intr_lane3_mask;    
        
    mask->fifo_empty        = intrs->fifoEmptyIntr;
    mask->fifo_read_error   = intrs->fifoReadErrIntr;
    mask->fifo_full         = intrs->fifoFullIntr;
    mask->fifo_write_error  = intrs->fifoWriteErrIntr;
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] | data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        
        *regs[ui] = *regs[ui] | data;
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxDisableLaneIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxDisableLaneIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *)&data;
        
    regs[0] = &hJesd->regs->jesdtx_intr_lane0_mask;    
    regs[1] = &hJesd->regs->jesdtx_intr_lane1_mask;    
    regs[2] = &hJesd->regs->jesdtx_intr_lane2_mask;    
    regs[3] = &hJesd->regs->jesdtx_intr_lane3_mask;    
    
    mask->fifo_empty        = intrs->fifoEmptyIntr;
    mask->fifo_read_error   = intrs->fifoReadErrIntr;
    mask->fifo_full         = intrs->fifoFullIntr;
    mask->fifo_write_error  = intrs->fifoWriteErrIntr;
    data = ~data; 
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] & data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        
        *regs[ui] = *regs[ui] & data;
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxClearLaneIntrsStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxClearLaneIntrsStatus(DfeFl_JesdHandle hJesd, DfeFl_JesdTxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *)&data;
    
    regs[0] = &hJesd->regs->jesdtx_intr_lane0_intr;    
    regs[1] = &hJesd->regs->jesdtx_intr_lane1_intr;    
    regs[2] = &hJesd->regs->jesdtx_intr_lane2_intr;    
    regs[3] = &hJesd->regs->jesdtx_intr_lane3_intr;    
    
    mask->fifo_empty        = intrs->fifoEmptyIntr;
    mask->fifo_read_error   = intrs->fifoReadErrIntr;
    mask->fifo_full         = intrs->fifoFullIntr;
    mask->fifo_write_error  = intrs->fifoWriteErrIntr;

    //write 0 to clear
    data = ~data; 
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] & data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        
        *regs[ui] = *regs[ui] & data;
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetForceLaneIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxSetForceLaneIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *)&data;
    
    regs[0] = &hJesd->regs->jesdtx_intr_lane0_force;    
    regs[1] = &hJesd->regs->jesdtx_intr_lane1_force;    
    regs[2] = &hJesd->regs->jesdtx_intr_lane2_force;    
    regs[3] = &hJesd->regs->jesdtx_intr_lane3_force;    
    
    mask->fifo_empty        = intrs->fifoEmptyIntr;
    mask->fifo_read_error   = intrs->fifoReadErrIntr;
    mask->fifo_full         = intrs->fifoFullIntr;
    mask->fifo_write_error  = intrs->fifoWriteErrIntr;
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] | data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        
        *regs[ui] = *regs[ui] | data;
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxClearForceLaneIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxClearForceLaneIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *)&data;
    
    regs[0] = &hJesd->regs->jesdtx_intr_lane0_force;    
    regs[1] = &hJesd->regs->jesdtx_intr_lane1_force;    
    regs[2] = &hJesd->regs->jesdtx_intr_lane2_force;    
    regs[3] = &hJesd->regs->jesdtx_intr_lane3_force;    
    
    mask->fifo_empty        = intrs->fifoEmptyIntr;
    mask->fifo_read_error   = intrs->fifoReadErrIntr;
    mask->fifo_full         = intrs->fifoFullIntr;
    mask->fifo_write_error  = intrs->fifoWriteErrIntr;
    data = ~data; 
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] & data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        
        *regs[ui] = *regs[ui] & data;
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetLaneIntrsStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
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
CSL_IDEF_INLINE DfeFl_Status 
dfeFl_JesdTxGetLaneIntrsStatus(DfeFl_JesdHandle hJesd, DfeFl_JesdTxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t data = 0;
    CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_LANE0_MASK_REG *)&data;
    
    regs[0] = &hJesd->regs->jesdtx_intr_lane0_intr;    
    regs[1] = &hJesd->regs->jesdtx_intr_lane1_intr;    
    regs[2] = &hJesd->regs->jesdtx_intr_lane2_intr;    
    regs[3] = &hJesd->regs->jesdtx_intr_lane3_intr;    
    
    if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        data = *regs[intrs->lane];
        
        intrs->fifoEmptyIntr = mask->fifo_empty;
        intrs->fifoReadErrIntr = mask->fifo_read_error;
        intrs->fifoFullIntr = mask->fifo_full;
        intrs->fifoWriteErrIntr = mask->fifo_write_error;    
        
        return DFE_FL_SOK;
    }
    else
    {
        return DFE_FL_INVPARAMS;
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxEnableSysrefIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxEnableSysrefIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdtx_intr_sysref_mask |= data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxDisableSysrefIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxDisableSysrefIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdtx_intr_sysref_mask &= ~data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxClearSysrefIntrsStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxClearSysrefIntrsStatus(DfeFl_JesdHandle hJesd, DfeFl_JesdTxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdtx_intr_sysref_intr &= ~data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxSetForceSysrefIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxSetForceSysrefIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdtx_intr_sysref_force |= data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxClearForceSysrefIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxClearForceSysrefIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdTxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdtx_intr_sysref_force &= ~data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetSysrefIntrsStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdTxGetSysrefIntrsStatus(DfeFl_JesdHandle hJesd, DfeFl_JesdTxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *)&data;
    
    data = hJesd->regs->jesdtx_intr_sysref_intr;
    
    intrs->reqAssertIntr = mask->sysref_request_assert;
    intrs->reqDeassertIntr = mask->sysref_request_deassert;
    intrs->errLink0Intr = mask->sysref_err_link0;
    intrs->errLink1Intr = mask->sysref_err_link1;    
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigMapLane
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_MAP_LANE0_NIBBLE0_POSITION0_REG_FRAME_POS
 *       DFE_JESD_JESDTX_MAP_LANE0_NIBBLE0_POSITION0_REG_NIBBLE_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxConfigMapLane(DfeFl_JesdHandle hJesd, DfeFl_JesdTxMapLaneConfig *arg)
{
    volatile uint32_t *regs[4];
    uint32_t iNibb, iPos, data;
    
    regs[0] = &hJesd->regs->jesdtx_map_lane0_nibble0_position0;
    regs[1] = &hJesd->regs->jesdtx_map_lane1_nibble0_position0;
    regs[2] = &hJesd->regs->jesdtx_map_lane2_nibble0_position0;
    regs[3] = &hJesd->regs->jesdtx_map_lane3_nibble0_position0;
    
    if(arg->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(iNibb = 0; iNibb < 4; iNibb++)
        {
            for(iPos = 0; iPos < 4; iPos++)
            {
                data = CSL_FMK(DFE_JESD_JESDTX_MAP_LANE0_NIBBLE0_POSITION0_REG_NIBBLE_SEL, arg->nibbleSel[iNibb][iPos])
                     | CSL_FMK(DFE_JESD_JESDTX_MAP_LANE0_NIBBLE0_POSITION0_REG_FRAME_POS, arg->framePos[iNibb][iPos]);
                regs[0][iNibb*4+iPos] = data;                     
                regs[1][iNibb*4+iPos] = data;                     
                regs[2][iNibb*4+iPos] = data;                     
                regs[3][iNibb*4+iPos] = data;                     
            }
        }            
    }
    else if(arg->lane <= DFE_FL_JESD_LANE_3)
    {
        for(iNibb = 0; iNibb < 4; iNibb++)
        {
            for(iPos = 0; iPos < 4; iPos++)
            {
                data = CSL_FMK(DFE_JESD_JESDTX_MAP_LANE0_NIBBLE0_POSITION0_REG_NIBBLE_SEL, arg->nibbleSel[iNibb][iPos])
                     | CSL_FMK(DFE_JESD_JESDTX_MAP_LANE0_NIBBLE0_POSITION0_REG_FRAME_POS, arg->framePos[iNibb][iPos]);
                regs[arg->lane][iNibb*4+iPos] = data;                     
            }
        }                    
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigMapNibb
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_LINK_SEL
 *       DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_NUM_FRAME_BUF_M1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxConfigMapNibb(DfeFl_JesdHandle hJesd, DfeFl_JesdTxMapNibbConfig *arg)
{
    volatile uint32_t *regs[4];
    uint32_t iBus, iNibb, data;
    
    // txbus i0
    regs[0] = &hJesd->regs->jesdtx_map_nibble00_cfg;
    // txbus q0
    regs[1] = &hJesd->regs->jesdtx_map_nibble04_cfg;
    // txbus i1
    regs[2] = &hJesd->regs->jesdtx_map_nibble08_cfg;
    // txbus q1
    regs[3] = &hJesd->regs->jesdtx_map_nibble12_cfg;
    
    if(arg->txBus == DFE_FL_JESDTX_TXBUS_ALL)
    {
        for(iBus = 0; iBus < 4; iBus++)
        {
            for(iNibb = 0; iNibb < 4; iNibb++)
            {
                data = regs[iBus][iNibb];
                CSL_FINS(data, DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_NUM_FRAME_BUF_M1, arg->numFrameM1[iNibb]);
                CSL_FINS(data, DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_LINK_SEL, arg->linkSel[iNibb]);
                regs[iBus][iNibb] = data;
            }
        }
    }
    else if(arg->txBus <= DFE_FL_JESDTX_TXBUS_Q1)
    {
        iBus = arg->txBus;
        {
            for(iNibb = 0; iNibb < 4; iNibb++)
            {
                data = regs[iBus][iNibb];
                CSL_FINS(data, DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_NUM_FRAME_BUF_M1, arg->numFrameM1[iNibb]);
                CSL_FINS(data, DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_LINK_SEL, arg->linkSel[iNibb]);
                regs[iBus][iNibb] = data;
            }
        }
    }
}    

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigNibbTestEnable
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_TEST_PAT_ENA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxConfigNibbTestEnable(DfeFl_JesdHandle hJesd, DfeFl_JesdTxNibbTestEnableConfig *arg)
{
    volatile uint32_t *regs[4];
    uint32_t iBus, iNibb, data;
    
    // txbus i0
    regs[0] = &hJesd->regs->jesdtx_map_nibble00_cfg;
    // txbus q0
    regs[1] = &hJesd->regs->jesdtx_map_nibble04_cfg;
    // txbus i1
    regs[2] = &hJesd->regs->jesdtx_map_nibble08_cfg;
    // txbus q1
    regs[3] = &hJesd->regs->jesdtx_map_nibble12_cfg;
    
    if(arg->txBus == DFE_FL_JESDTX_TXBUS_ALL)
    {
        for(iBus = 0; iBus < 4; iBus++)
        {
            for(iNibb = 0; iNibb < 4; iNibb++)
            {
                data = regs[iBus][iNibb];
                CSL_FINS(data, DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_TEST_PAT_ENA, arg->testPatEnable[iNibb]);
                regs[iBus][iNibb] = data;
            }
        }
    }
    else if(arg->txBus <= DFE_FL_JESDTX_TXBUS_Q1)
    {
        iBus = arg->txBus;
        {
            for(iNibb = 0; iNibb < 4; iNibb++)
            {
                data = regs[iBus][iNibb];
                CSL_FINS(data, DFE_JESD_JESDTX_MAP_NIBBLE00_CFG_REG_TEST_PAT_ENA, arg->testPatEnable[iNibb]);
                regs[iBus][iNibb] = data;
            }
        }
    }
}    

/** ============================================================================
 *   @n@b dfeFl_JesdTxConfigNibbTestData
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDTX_MAP_TEST_NIBBLE00_POSITION0_REG_TEST_DATA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdTxConfigNibbTestData(DfeFl_JesdHandle hJesd, DfeFl_JesdTxNibbTestDataConfig *arg)
{
    volatile uint32_t *regs[4];
    uint32_t iNibb, iPos, data;
    
    // txbus i0
    regs[0] = &hJesd->regs->jesdtx_map_test_nibble00_position0;
    // txbus q0
    regs[1] = &hJesd->regs->jesdtx_map_test_nibble04_position0;
    // txbus i1
    regs[2] = &hJesd->regs->jesdtx_map_test_nibble08_position0;
    // txbus q1
    regs[3] = &hJesd->regs->jesdtx_map_test_nibble12_position0;
    
    if(arg->txBus == DFE_FL_JESDTX_TXBUS_ALL)
    {
        for(iNibb = 0; iNibb < 4; iNibb++)
        {
            for(iPos = 0; iPos < 4; iPos++)
            {
                data = CSL_FMK(DFE_JESD_JESDTX_MAP_TEST_NIBBLE00_POSITION0_REG_TEST_DATA, arg->testData[iNibb][iPos]);
                regs[0][iNibb*4+iPos] = data;                     
                regs[1][iNibb*4+iPos] = data;                     
                regs[2][iNibb*4+iPos] = data;                     
                regs[3][iNibb*4+iPos] = data;                     
            }
        }
    }
    else if(arg->txBus <= DFE_FL_JESDTX_TXBUS_Q1)
    {
        for(iNibb = 0; iNibb < 4; iNibb++)
        {
            for(iPos = 0; iPos < 4; iPos++)
            {
                data = CSL_FMK(DFE_JESD_JESDTX_MAP_TEST_NIBBLE00_POSITION0_REG_TEST_DATA, arg->testData[iNibb][iPos]);
                regs[arg->txBus][iNibb*4+iPos] = data;                     
            }
        }
    }
}    

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// JESD RX /////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
dfeFl_JesdRxConfigInits(DfeFl_JesdHandle hJesd, DfeFl_JesdInitsConfig * arg)
{
    
    uint32_t data = hJesd->regs->jesdrx_base_inits;
    CSL_DFE_JESD_JESDRX_BASE_INITS_REG *pData = (CSL_DFE_JESD_JESDRX_BASE_INITS_REG *)&data;
    
    pData->inits_ssel       = arg->cmn.ssel;
    pData->init_clk_gate    = arg->cmn.initClkGate;
    pData->init_state       = arg->cmn.initState;
    pData->clear_data       = arg->cmn.clearData;
    pData->clear_data_lane0 = arg->clearDataLane[0];
    pData->clear_data_lane1 = arg->clearDataLane[1];   
    pData->clear_data_lane2 = arg->clearDataLane[2];   
    pData->clear_data_lane3 = arg->clearDataLane[3]; 
    
    
    hJesd->regs->jesdrx_base_inits = data;    
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigInitSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         initSsel    [add content]
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
dfeFl_JesdRxConfigInitSsel(DfeFl_JesdHandle hJesd, uint32_t initSsel)
{
    
    uint32_t data = hJesd->regs->jesdrx_base_inits;
	
	CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_INITS_SSEL, initSsel);

	hJesd->regs->jesdrx_base_inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdTxGetInitSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         initSsel    [add content]
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
dfeFl_JesdRxGetInitSsel(DfeFl_JesdHandle hJesd, uint32_t * initSsel)
{
    
    uint32_t data = hJesd->regs->jesdrx_base_inits;
	
	*initSsel = CSL_FEXT(data, DFE_JESD_JESDRX_BASE_INITS_REG_INITS_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigInitState
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         initState    [add content]
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
dfeFl_JesdRxConfigInitState(DfeFl_JesdHandle hJesd, uint32_t initState)
{
    
    uint32_t data = hJesd->regs->jesdrx_base_inits;
	
	CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_INIT_STATE, initState);

	hJesd->regs->jesdrx_base_inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetClearDataLane
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
         clearData    [add content]
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
 *       DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE3
 *       DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE2
 *       DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE1
 *       DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxSetClearDataLane(DfeFl_JesdHandle hJesd, uint32_t lane, uint32_t clearData)
{
    uint32_t data = hJesd->regs->jesdrx_base_inits;
    
    if(lane == DFE_FL_JESD_LANE_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE0, clearData);
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE1, clearData);
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE2, clearData);
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE3, clearData);        
    }
    else if(lane == DFE_FL_JESD_LANE_0)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE0, clearData);
    }
    else if(lane == DFE_FL_JESD_LANE_1)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE1, clearData);
    }
    else if(lane == DFE_FL_JESD_LANE_2)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE2, clearData);
    }
    else if(lane == DFE_FL_JESD_LANE_3)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_INITS_REG_CLEAR_DATA_LANE3, clearData);
    }                    
    
    hJesd->regs->jesdrx_base_inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetTestBusSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         testBusSel    [add content]
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
 *       DFE_JESD_JESDRX_BASE_TEST_BUS_SEL_REG_TEST_BUS_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxSetTestBusSel(DfeFl_JesdHandle hJesd, uint32_t testBusSel)
{
    hJesd->regs->jesdrx_base_test_bus_sel = CSL_FMK(DFE_JESD_JESDRX_BASE_TEST_BUS_SEL_REG_TEST_BUS_SEL, testBusSel);
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetTestSeqSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
         testSeqSel    [add content]
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
 *       DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE2
 *       DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE3
 *       DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE0
 *       DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxSetTestSeqSel(DfeFl_JesdHandle hJesd, uint32_t lane, uint32_t testSeqSel)
{
    uint32_t data = hJesd->regs->jesdrx_base_test_seq_sel;
    if(lane == DFE_FL_JESD_LANE_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE0, testSeqSel);
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE1, testSeqSel);
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE2, testSeqSel);
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE3, testSeqSel);
    }
    else if(lane == DFE_FL_JESD_LANE_0)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE0, testSeqSel);
    }
    else if(lane == DFE_FL_JESD_LANE_1)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE1, testSeqSel);
    }
    else if(lane == DFE_FL_JESD_LANE_2)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE2, testSeqSel);
    }
    else if(lane == DFE_FL_JESD_LANE_3)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_BASE_TEST_SEQ_SEL_REG_LANE3, testSeqSel);
    }
    
    hJesd->regs->jesdrx_base_test_seq_sel = data;   
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigLoopback
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
dfeFl_JesdRxConfigLoopback(DfeFl_JesdHandle hJesd, DfeFl_JesdRxLoopbackConfig *arg)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_BASE_LPBK_ENA_REG *pData = (CSL_DFE_JESD_JESDRX_BASE_LPBK_ENA_REG *)&data;
    
    pData->lane0 = arg->lane0;
    pData->lane1 = arg->lane1;
    pData->lane2 = arg->lane2;
    pData->lane3 = arg->lane3;
    pData->rx0   = arg->tx0_rx0;    
    pData->rx1   = arg->tx0_rx1fb0;
    pData->rx2   = arg->tx1_rx2fb1;

    hJesd->regs->jesdrx_base_lpbk_ena = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetLoopback
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
dfeFl_JesdRxGetLoopback(DfeFl_JesdHandle hJesd, DfeFl_JesdRxLoopbackConfig *arg)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_BASE_LPBK_ENA_REG *pData = (CSL_DFE_JESD_JESDRX_BASE_LPBK_ENA_REG *)&data;
    
    data = hJesd->regs->jesdrx_base_lpbk_ena;
    
    arg->lane0      =  pData->lane0; 
    arg->lane1      =  pData->lane1; 
    arg->lane2      =  pData->lane2; 
    arg->lane3      =  pData->lane3; 
    arg->tx0_rx0    =  pData->rx0  ; 
    arg->tx0_rx1fb0 =  pData->rx1  ; 
    arg->tx1_rx2fb1 =  pData->rx2  ; 
}

/** ============================================================================
 *   @n@b CSL_defJesdRxConfigBbRxCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
CSL_defJesdRxConfigBbRxCtrl(DfeFl_JesdHandle hJesd, DfeFl_JesdRxBbRxCtrlConfig *arg)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_BASE_BB_RX_CTRL_REG *pData = (CSL_DFE_JESD_JESDRX_BASE_BB_RX_CTRL_REG *)&data;
    
    pData->bb_out_lane_sel       =arg->bbOutLaneSel;   
    pData->bb_out_ena            =arg->bbOutEna;        
    pData->rx_force_frame_rx0    =arg->forceFrameRx0;   
    pData->rx_force_frame_rx1    =arg->forceFrameRx1Fb0;
    pData->rx_force_frame_rx2    =arg->forceFrameRx2Fb1;
    
    hJesd->regs->jesdrx_base_bb_rx_ctrl = data;
}

/** ============================================================================
 *   @n@b CSL_defJesdRxConfigFifo
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDRX_BASE_FIFO_REG_FIFO_READ_DELAY
 *       DFE_JESD_JESDRX_BASE_FIFO_REG_DISABLE_FIFO_ERRORS_ZERO_DATA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
CSL_defJesdRxConfigFifo(DfeFl_JesdHandle hJesd, DfeFl_JesdRxFifoConfig *arg)
{
    hJesd->regs->jesdrx_base_fifo = CSL_FMK(DFE_JESD_JESDRX_BASE_FIFO_REG_FIFO_READ_DELAY, arg->readDly)
                                  | CSL_FMK(DFE_JESD_JESDRX_BASE_FIFO_REG_DISABLE_FIFO_ERRORS_ZERO_DATA, arg->disableZeroData);
}       

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigSyncnOut
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDRX_BASE_SYNC_N_OUT_REG_SEL_LINK1
 *       DFE_JESD_JESDRX_BASE_SYNC_N_OUT_REG_SEL_LINK0
 *       DFE_JESD_JESDRX_BASE_SYNC_N_OUT_REG_SYNC_BUS_ENA_1
 *       DFE_JESD_JESDRX_BASE_SYNC_N_OUT_REG_SYNC_BUS_ENA_0
 *       DFE_JESD_JESDRX_BASE_SYNC_N_OUT_INV_REG_LINK0
 *       DFE_JESD_JESDRX_BASE_SYNC_N_OUT_INV_REG_LINK1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxConfigSyncnOut(DfeFl_JesdHandle hJesd, DfeFl_JesdRxSyncnOutConfig *arg)
{
    hJesd->regs->jesdrx_base_sync_n_out = CSL_FMK(DFE_JESD_JESDRX_BASE_SYNC_N_OUT_REG_SEL_LINK0, arg->selLink0)
                                        | CSL_FMK(DFE_JESD_JESDRX_BASE_SYNC_N_OUT_REG_SEL_LINK1, arg->selLink1)
                                        | CSL_FMK(DFE_JESD_JESDRX_BASE_SYNC_N_OUT_REG_SYNC_BUS_ENA_0, arg->syncBusEnable0)
                                        | CSL_FMK(DFE_JESD_JESDRX_BASE_SYNC_N_OUT_REG_SYNC_BUS_ENA_1, arg->syncBusEnable1);
                                        
    hJesd->regs->jesdrx_base_sync_n_out = CSL_FMK(DFE_JESD_JESDRX_BASE_SYNC_N_OUT_INV_REG_LINK0, arg->invertLink0)
                                        | CSL_FMK(DFE_JESD_JESDRX_BASE_SYNC_N_OUT_INV_REG_LINK1, arg->invertLink1);
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetSysrefDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         dly    [add content]
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
 *       DFE_JESD_JESDRX_BASE_SYSREF_REG_SYSREF_DLY_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxSetSysrefDelay(DfeFl_JesdHandle hJesd, uint32_t dly)
{
    CSL_FINS(hJesd->regs->jesdrx_base_sysref, DFE_JESD_JESDRX_BASE_SYSREF_REG_SYSREF_DLY_SEL, dly);
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxForceSysRefRequest
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         forceReq    [add content]
         autoOffCount    [add content]
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
 *       DFE_JESD_JESDRX_BASE_SYSREF_REG_FORCE_SYSREF_REQUEST
 *       DFE_JESD_JESDRX_BASE_SYSREF_REG_FORCE_SYSREF_REQUEST_AUTO_OFF
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxForceSysRefRequest(DfeFl_JesdHandle hJesd, uint32_t forceReq, uint32_t autoOffCount)
{
    uint32_t data = hJesd->regs->jesdrx_base_sysref;
    
    CSL_FINS(data, DFE_JESD_JESDRX_BASE_SYSREF_REG_FORCE_SYSREF_REQUEST, forceReq);
    CSL_FINS(data, DFE_JESD_JESDRX_BASE_SYSREF_REG_FORCE_SYSREF_REQUEST_AUTO_OFF, autoOffCount);
    
    hJesd->regs->jesdrx_base_sysref = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetSysRefAlignmentCounter
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDRX_BASE_SYSREF_CNTR_HI_REG_SYSREF_CNTR_31_16
 *       DFE_JESD_JESDRX_BASE_SYSREF_CNTR_LO_REG_SYSREF_CNTR_15_0
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
dfeFl_JesdRxGetSysRefAlignmentCounter(DfeFl_JesdHandle hJesd)
{
    return(CSL_FEXT(hJesd->regs->jesdrx_base_sysref_cntr_lo, DFE_JESD_JESDRX_BASE_SYSREF_CNTR_LO_REG_SYSREF_CNTR_15_0)
         |(CSL_FEXT(hJesd->regs->jesdrx_base_sysref_cntr_hi, DFE_JESD_JESDRX_BASE_SYSREF_CNTR_HI_REG_SYSREF_CNTR_31_16) << 16));
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetSyncState
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDRX_BASE_CS_STATE_REG_LANE0~3
 *       DFE_JESD_JESDRX_BASE_FS_STATE_REG_LANE0~3
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdRxGetSyncState(DfeFl_JesdHandle hJesd, DfeFl_JesdRxLaneSyncStateQuery *qry)
{
    uint32_t cs_state = hJesd->regs->jesdrx_base_cs_state;
    uint32_t fs_state = hJesd->regs->jesdrx_base_fs_state;
    
    if(qry->lane == DFE_FL_JESD_LANE_0)
    {
         qry->codeState = CSL_FEXT(cs_state, DFE_JESD_JESDRX_BASE_CS_STATE_REG_LANE0);
         qry->frameState = CSL_FEXT(fs_state, DFE_JESD_JESDRX_BASE_FS_STATE_REG_LANE0);
    }
    else if(qry->lane == DFE_FL_JESD_LANE_1)
    {
         qry->codeState = CSL_FEXT(cs_state, DFE_JESD_JESDRX_BASE_CS_STATE_REG_LANE1);
         qry->frameState = CSL_FEXT(fs_state, DFE_JESD_JESDRX_BASE_FS_STATE_REG_LANE1);
    }
    else if(qry->lane == DFE_FL_JESD_LANE_2)
    {
         qry->codeState = CSL_FEXT(cs_state, DFE_JESD_JESDRX_BASE_CS_STATE_REG_LANE2);
         qry->frameState = CSL_FEXT(fs_state, DFE_JESD_JESDRX_BASE_FS_STATE_REG_LANE2);
    }
    else if(qry->lane == DFE_FL_JESD_LANE_3)
    {
         qry->codeState = CSL_FEXT(cs_state, DFE_JESD_JESDRX_BASE_CS_STATE_REG_LANE3);
         qry->frameState = CSL_FEXT(fs_state, DFE_JESD_JESDRX_BASE_FS_STATE_REG_LANE3);
    }
    else
    {
        // return 
         qry->codeState = 0xdeadbeefu;
         qry->frameState = 0xdeadbeefu;
        return DFE_FL_INVPARAMS;
    }
    
    return DFE_FL_SOK;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetLinkInitStateSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
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
 *       DFE_JESD_JESDRX_SSEL_SSEL_ADDR_0_REG_INIT_STATE_SSEL_LINK1
 *       DFE_JESD_JESDRX_SSEL_SSEL_ADDR_0_REG_INIT_STATE_SSEL_LINK0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxSetLinkInitStateSsel(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdrx_ssel_ssel_addr_0;

    if(link == DFE_FL_JESD_LINK_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_0_REG_INIT_STATE_SSEL_LINK0, ssel);
        CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_0_REG_INIT_STATE_SSEL_LINK1, ssel);
    }
    else if(link == DFE_FL_JESD_LINK_0)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_0_REG_INIT_STATE_SSEL_LINK0, ssel);
    }
    else if(link == DFE_FL_JESD_LINK_1)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_0_REG_INIT_STATE_SSEL_LINK1, ssel);
    }

    hJesd->regs->jesdrx_ssel_ssel_addr_0 = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         rxbus    [add content]
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
dfeFl_JesdRxSetChksumSsel(DfeFl_JesdHandle hJesd, uint32_t rxbus, uint32_t ssel)
{
    uint32_t data0, data1;
    CSL_DFE_JESD_JESDRX_SSEL_SSEL_ADDR_0_REG *pData0 = (CSL_DFE_JESD_JESDRX_SSEL_SSEL_ADDR_0_REG *)&data0;
    CSL_DFE_JESD_JESDRX_SSEL_SSEL_ADDR_1_REG *pData1 = (CSL_DFE_JESD_JESDRX_SSEL_SSEL_ADDR_1_REG *)&data1;
    
    data0 = hJesd->regs->jesdrx_ssel_ssel_addr_0;
    data1 = hJesd->regs->jesdrx_ssel_ssel_addr_1;
    
    if(rxbus == DFE_FL_JESDRX_RXBUS_ALL)
    {
        pData0->check_sum_ssel_rx0i = ssel;
        pData0->check_sum_ssel_rx0q = ssel;
        pData1->check_sum_ssel_rx1i = ssel;
        pData1->check_sum_ssel_rx1q = ssel;        
        pData1->check_sum_ssel_rx2i = ssel;
        pData1->check_sum_ssel_rx2q = ssel;
        
        hJesd->regs->jesdrx_ssel_ssel_addr_0 = data0;
        hJesd->regs->jesdrx_ssel_ssel_addr_1 = data1;
    }
    else if(rxbus == DFE_FL_JESDRX_RXBUS_RX0I)
    {
        pData0->check_sum_ssel_rx0i = ssel;
        
        hJesd->regs->jesdrx_ssel_ssel_addr_0 = data0;
    }        
    else if(rxbus == DFE_FL_JESDRX_RXBUS_RX0Q)
    {
        pData0->check_sum_ssel_rx0q = ssel;
        
        hJesd->regs->jesdrx_ssel_ssel_addr_0 = data0;
    }        
    else if(rxbus == DFE_FL_JESDRX_RXBUS_RX1I_FB0I)
    {
        pData1->check_sum_ssel_rx1i = ssel;

        hJesd->regs->jesdrx_ssel_ssel_addr_1 = data1;
    }        
    else if(rxbus == DFE_FL_JESDRX_RXBUS_RX1Q_FB0Q)
    {
        pData1->check_sum_ssel_rx1q = ssel;        

        hJesd->regs->jesdrx_ssel_ssel_addr_1 = data1;
    }        
    else if(rxbus == DFE_FL_JESDRX_RXBUS_RX2I_FB1I)
    {
        pData1->check_sum_ssel_rx2i = ssel;

        hJesd->regs->jesdrx_ssel_ssel_addr_1 = data1;
    }        
    else if(rxbus == DFE_FL_JESDRX_RXBUS_RX2Q_FB1Q)
    {
        pData1->check_sum_ssel_rx2q = ssel;
        
        hJesd->regs->jesdrx_ssel_ssel_addr_1 = data1;
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetSysRefAlignmentCounterSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_SYSREF_CNTR_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxSetSysRefAlignmentCounterSsel(DfeFl_JesdHandle hJesd, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdrx_ssel_ssel_addr_2;

    CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_SYSREF_CNTR_SSEL, ssel);
    
    hJesd->regs->jesdrx_ssel_ssel_addr_2 = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetSyncnOutSyncBusSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         syncBus    [add content]
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
 *       DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_SYNC_N_OUT_SYNC_BUS_SSEL_0
 *       DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_SYNC_N_OUT_SYNC_BUS_SSEL_1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxSetSyncnOutSyncBusSsel(DfeFl_JesdHandle hJesd, uint32_t syncBus, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdrx_ssel_ssel_addr_2;
    
    if(syncBus == DFE_FL_JESDRX_SYNCN_OUT_BUS_ALL)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_SYNC_N_OUT_SYNC_BUS_SSEL_0, ssel);
        CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_SYNC_N_OUT_SYNC_BUS_SSEL_1, ssel);
    }
    else if(syncBus == DFE_FL_JESDRX_SYNCN_OUT_BUS_0)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_SYNC_N_OUT_SYNC_BUS_SSEL_0, ssel);
    }
    else if(syncBus == DFE_FL_JESDRX_SYNCN_OUT_BUS_1)
    {
        CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_SYNC_N_OUT_SYNC_BUS_SSEL_1, ssel);
    }

    hJesd->regs->jesdrx_ssel_ssel_addr_2 = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetSysRefModeSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_RX_SYSREF_MODE_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxSetSysRefModeSsel(DfeFl_JesdHandle hJesd, uint32_t ssel)
{
    uint32_t data = hJesd->regs->jesdrx_ssel_ssel_addr_2;

    CSL_FINS(data, DFE_JESD_JESDRX_SSEL_SSEL_ADDR_2_REG_RX_SYSREF_MODE_SSEL, ssel);
    
    hJesd->regs->jesdrx_ssel_ssel_addr_2 = data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDRX_CHECK_SUM_RX0I_CTRL_REG_MODE
 *       DFE_JESD_JESDRX_CHECK_SUM_RX0I_SIGNAL_LEN_REG_SIGNAL_LEN
 *       DFE_JESD_JESDRX_CHECK_SUM_RX0I_CHAN_SEL_REG_CHAN_SEL
 *       DFE_JESD_JESDRX_CHECK_SUM_RX0I_CTRL_REG_STABLE_LEN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_JesdRxConfigChksum(DfeFl_JesdHandle hJesd, DfeFl_JesdRxChksumConfig *arg)
{
    volatile uint32_t *regs[6];
    uint32_t ui, ctrl, sigLen, chanSel;
    
    
    regs[0] = &hJesd->regs->jesdrx_check_sum_rx0i_ctrl;
    regs[1] = &hJesd->regs->jesdrx_check_sum_rx0q_ctrl;
    regs[2] = &hJesd->regs->jesdrx_check_sum_rx1i_ctrl;
    regs[3] = &hJesd->regs->jesdrx_check_sum_rx1q_ctrl;
    regs[4] = &hJesd->regs->jesdrx_check_sum_rx2i_ctrl;
    regs[5] = &hJesd->regs->jesdrx_check_sum_rx2q_ctrl;

    // ctrl, stable_len
    ctrl = CSL_FMK(DFE_JESD_JESDRX_CHECK_SUM_RX0I_CTRL_REG_MODE, arg->chksumMode)
         | CSL_FMK(DFE_JESD_JESDRX_CHECK_SUM_RX0I_CTRL_REG_STABLE_LEN, arg->latencyMode.stableLen);

    // signal_len
    sigLen = CSL_FMK(DFE_JESD_JESDRX_CHECK_SUM_RX0I_SIGNAL_LEN_REG_SIGNAL_LEN, arg->latencyMode.signalLen);

    // chan_sel
    chanSel = CSL_FMK(DFE_JESD_JESDRX_CHECK_SUM_RX0I_CHAN_SEL_REG_CHAN_SEL, arg->latencyMode.chanSel);
    
    if(arg->chksumDev == DFE_FL_JESDRX_RXBUS_ALL)
    {
        for(ui = 0; ui < 6; ui++)
        {
            regs[ui][0] = ctrl;
            regs[ui][1] = sigLen;
            regs[ui][2] = chanSel;
        }
    }
    else if(arg->chksumDev <= DFE_FL_JESDRX_RXBUS_RX2Q_FB1Q)
    {
        ui = arg->chksumDev;
        regs[ui][0] = ctrl;
        regs[ui][1] = sigLen;
        regs[ui][2] = chanSel;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetChecksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         chksumDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDRX_CHECK_SUM_RX2Q_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDRX_CHECK_SUM_RX1I_RESULT_LO_REG_RESULT_15_0
 *       DFE_JESD_JESDRX_CHECK_SUM_RX2I_RESULT_LO_REG_RESULT_15_0
 *       DFE_JESD_JESDRX_CHECK_SUM_RX1Q_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDRX_CHECK_SUM_RX1Q_RESULT_LO_REG_RESULT_15_0
 *       DFE_JESD_JESDRX_CHECK_SUM_RX0Q_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDRX_CHECK_SUM_RX1I_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDRX_CHECK_SUM_RX0I_RESULT_LO_REG_RESULT_15_0
 *       DFE_JESD_JESDRX_CHECK_SUM_RX0I_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDRX_CHECK_SUM_RX2I_RESULT_HI_REG_RESULT_31_16
 *       DFE_JESD_JESDRX_CHECK_SUM_RX0Q_RESULT_LO_REG_RESULT_15_0
 *       DFE_JESD_JESDRX_CHECK_SUM_RX2Q_RESULT_LO_REG_RESULT_15_0
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdRxGetChecksumResult(DfeFl_JesdHandle hJesd, uint32_t chksumDev, uint32_t *chksumResult)
{
    if(chksumDev == DFE_FL_JESDRX_RXBUS_RX0I)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx0i_result_lo, DFE_JESD_JESDRX_CHECK_SUM_RX0I_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx0i_result_hi, DFE_JESD_JESDRX_CHECK_SUM_RX0I_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else if(chksumDev == DFE_FL_JESDRX_RXBUS_RX0Q)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx0q_result_lo, DFE_JESD_JESDRX_CHECK_SUM_RX0Q_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx0q_result_hi, DFE_JESD_JESDRX_CHECK_SUM_RX0Q_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else if(chksumDev == DFE_FL_JESDRX_RXBUS_RX1I_FB0I)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx1i_result_lo, DFE_JESD_JESDRX_CHECK_SUM_RX1I_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx1i_result_hi, DFE_JESD_JESDRX_CHECK_SUM_RX1I_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else if(chksumDev == DFE_FL_JESDRX_RXBUS_RX1Q_FB0Q)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx1q_result_lo, DFE_JESD_JESDRX_CHECK_SUM_RX1Q_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx1q_result_hi, DFE_JESD_JESDRX_CHECK_SUM_RX1Q_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else if(chksumDev == DFE_FL_JESDRX_RXBUS_RX2I_FB1I)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx2i_result_lo, DFE_JESD_JESDRX_CHECK_SUM_RX2I_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx2i_result_hi, DFE_JESD_JESDRX_CHECK_SUM_RX2I_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else if(chksumDev == DFE_FL_JESDRX_RXBUS_RX2Q_FB1Q)
    {
        *chksumResult = (CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx2q_result_lo, DFE_JESD_JESDRX_CHECK_SUM_RX2Q_RESULT_LO_REG_RESULT_15_0)
             |(CSL_FEXT(hJesd->regs->jesdrx_check_sum_rx2q_result_hi, DFE_JESD_JESDRX_CHECK_SUM_RX2Q_RESULT_HI_REG_RESULT_31_16) << 16));
    }
    else
    { 
        *chksumResult = 0xdeadbeef;
        return DFE_FL_INVPARAMS;
    }
    
    return DFE_FL_SOK;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigLinkClockGate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
         cgCfg    [add content]
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
dfeFl_JesdRxConfigLinkClockGate(DfeFl_JesdHandle hJesd, uint32_t link, DfeFl_ClockGateConfig *cgCfg)
{
    volatile uint32_t *regs[2];
    uint32_t ui;
    
    regs[0] = &hJesd->regs->jesdrx_clk_gater_link0_time_step;
    regs[1] = &hJesd->regs->jesdrx_clk_gater_link1_time_step;
    
    if(link == DFE_FL_JESD_LINK_ALL)
    {
        for(ui = 0; ui < 2; ui++)
        {
            regs[ui][0] = cgCfg->timeStep & 0xffffu;
            //regs[ui][1] = cgCfg->timeStep >> 16;
            regs[ui][2] = cgCfg->resetInterval & 0xffffu;
            regs[ui][3] = cgCfg->resetInterval >> 16;
            regs[ui][0] = cgCfg->tddPeriod & 0xffffu;
            regs[ui][1] = cgCfg->tddPeriod >> 16;
            regs[ui][0] = cgCfg->tddOn0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn0 >> 16;
            regs[ui][0] = cgCfg->tddOff0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff0 >> 16;
            regs[ui][0] = cgCfg->tddOn1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn1 >> 16;
            regs[ui][0] = cgCfg->tddOff1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff1 >> 16;
        }
    }
    else if(link <= DFE_FL_JESD_LINK_1)
    {
        ui = link;
        {
            regs[ui][0] = cgCfg->timeStep & 0xffffu;
            //regs[ui][1] = cgCfg->timeStep >> 16;
            regs[ui][2] = cgCfg->resetInterval & 0xffffu;
            regs[ui][3] = cgCfg->resetInterval >> 16;
            regs[ui][0] = cgCfg->tddPeriod & 0xffffu;
            regs[ui][1] = cgCfg->tddPeriod >> 16;
            regs[ui][0] = cgCfg->tddOn0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn0 >> 16;
            regs[ui][0] = cgCfg->tddOff0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff0 >> 16;
            regs[ui][0] = cgCfg->tddOn1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn1 >> 16;
            regs[ui][0] = cgCfg->tddOff1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff1 >> 16;
        }
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigPathClockGate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         path    [add content]
         cgCfg    [add content]
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
dfeFl_JesdRxConfigPathClockGate(DfeFl_JesdHandle hJesd, uint32_t path, DfeFl_ClockGateConfig *cgCfg)
{
    volatile uint32_t *regs[3];
    uint32_t ui;
    
    regs[0] = &hJesd->regs->jesdrx_clk_gater_rx0_time_step;
    regs[1] = &hJesd->regs->jesdrx_clk_gater_rx1_time_step;
    regs[2] = &hJesd->regs->jesdrx_clk_gater_rx2_time_step;
    
    if(path == DFE_FL_JESDRX_RX_ALL)
    {
        for(ui = 0; ui < 2; ui++)
        {
            regs[ui][0] = cgCfg->timeStep & 0xffffu;
            //regs[ui][1] = cgCfg->timeStep >> 16;
            regs[ui][2] = cgCfg->resetInterval & 0xffffu;
            regs[ui][3] = cgCfg->resetInterval >> 16;
            regs[ui][0] = cgCfg->tddPeriod & 0xffffu;
            regs[ui][1] = cgCfg->tddPeriod >> 16;
            regs[ui][0] = cgCfg->tddOn0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn0 >> 16;
            regs[ui][0] = cgCfg->tddOff0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff0 >> 16;
            regs[ui][0] = cgCfg->tddOn1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn1 >> 16;
            regs[ui][0] = cgCfg->tddOff1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff1 >> 16;
        }
    }
    else if(path <= DFE_FL_JESDRX_RX2_FB1)
    {
        ui = path;
        {
            regs[ui][0] = cgCfg->timeStep & 0xffffu;
            //regs[ui][1] = cgCfg->timeStep >> 16;
            regs[ui][2] = cgCfg->resetInterval & 0xffffu;
            regs[ui][3] = cgCfg->resetInterval >> 16;
            regs[ui][0] = cgCfg->tddPeriod & 0xffffu;
            regs[ui][1] = cgCfg->tddPeriod >> 16;
            regs[ui][0] = cgCfg->tddOn0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn0 >> 16;
            regs[ui][0] = cgCfg->tddOff0 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff0 >> 16;
            regs[ui][0] = cgCfg->tddOn1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOn1 >> 16;
            regs[ui][0] = cgCfg->tddOff1 & 0xffffu;
            regs[ui][1] = cgCfg->tddOff1 >> 16;
        }
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigLane
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
         enable    [add content]
         linkAssign    [add content]
         lid    [add content]
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
 *       DFE_JESD_JESDRX_LANE0_CFG_REG_LANE_ENA
 *       DFE_JESD_JESDRX_LANE0_CFG_REG_LINK_ASSIGN
 *       DFE_JESD_JESDRX_LANE0_CFG_REG_LID
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxConfigLane(DfeFl_JesdHandle hJesd, uint32_t lane, uint32_t enable, uint32_t linkAssign, uint32_t lid)
{
    volatile uint32_t *regs[4];
    uint32_t data;
    
    regs[0] = &hJesd->regs->jesdrx_lane0_cfg;
    regs[1] = &hJesd->regs->jesdrx_lane1_cfg;
    regs[2] = &hJesd->regs->jesdrx_lane2_cfg;
    regs[3] = &hJesd->regs->jesdrx_lane3_cfg;
    
    data = CSL_FMK(DFE_JESD_JESDRX_LANE0_CFG_REG_LANE_ENA, enable)
         | CSL_FMK(DFE_JESD_JESDRX_LANE0_CFG_REG_LINK_ASSIGN, linkAssign)
         | CSL_FMK(DFE_JESD_JESDRX_LANE0_CFG_REG_LID, lid);
         
    if(lane == DFE_FL_JESD_LANE_ALL)
    {
        *regs[0] = data;
        *regs[1] = data;
        *regs[2] = data;
        *regs[3] = data;
    }
    else if(lane <= DFE_FL_JESD_LANE_3)
    {
        *regs[lane] = data;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetLaneConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         lane    [add content]
         enable    [add content]
         linkAssign    [add content]
         lid    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDRX_LANE0_CFG_REG_LANE_ENA
 *       DFE_JESD_JESDRX_LANE0_CFG_REG_LINK_ASSIGN
 *       DFE_JESD_JESDRX_LANE0_CFG_REG_LID
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
CSL_IDEF_INLINE DfeFl_Status 
dfeFl_JesdRxGetLaneConfig(DfeFl_JesdHandle hJesd, uint32_t lane, uint32_t *enable, uint32_t *linkAssign, uint32_t *lid)
{
    volatile uint32_t *regs[4];
    uint32_t data;
    
    regs[0] = &hJesd->regs->jesdrx_lane0_cfg;
    regs[1] = &hJesd->regs->jesdrx_lane1_cfg;
    regs[2] = &hJesd->regs->jesdrx_lane2_cfg;
    regs[3] = &hJesd->regs->jesdrx_lane3_cfg;
    
    *enable = 0xdeadbeefu;
    *linkAssign = 0xdeadbeefu;
    *lid = 0xdeadbeefu;
    
    if(lane <= DFE_FL_JESD_LANE_3)
    {
        data = *regs[lane];
        
        *enable     = CSL_FEXT(data, DFE_JESD_JESDRX_LANE0_CFG_REG_LANE_ENA);
        *linkAssign = CSL_FEXT(data, DFE_JESD_JESDRX_LANE0_CFG_REG_LINK_ASSIGN);
        *lid        = CSL_FEXT(data, DFE_JESD_JESDRX_LANE0_CFG_REG_LID);
        
        return DFE_FL_SOK;
    }
    else
    {
        return DFE_FL_INVPARAMS;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigLink
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDRX_LINK0_CFG2_REG_F_M1
 *       DFE_JESD_JESDRX_LINK0_CFG5_REG_CF
 *       DFE_JESD_JESDRX_LINK0_CFG7_REG_MATCH_SPECIFIC
 *       DFE_JESD_JESDRX_LINK0_CFG7_REG_MATCH_DATA
 *       DFE_JESD_JESDRX_LINK0_CFG5_REG_RES1
 *       DFE_JESD_JESDRX_LINK0_CFG5_REG_HD
 *       DFE_JESD_JESDRX_LINK0_CFG7_REG_MATCH_CTRL
 *       DFE_JESD_JESDRX_LINK0_CFG1_REG_ADJDIR
 *       DFE_JESD_JESDRX_LINK0_CFG9_REG_MP_LINK_ENA
 *       DFE_JESD_JESDRX_LINK0_CFG8_REG_ERROR_ENA
 *       DFE_JESD_JESDRX_LINK0_CFG9_REG_DISABLE_ERR_REPORT
 *       DFE_JESD_JESDRX_LINK0_CFG9_REG_NO_LANE_SYNC
 *       DFE_JESD_JESDRX_LINK0_CFG9_REG_SYSREF_MODE
 *       DFE_JESD_JESDRX_LINK0_CFG4_REG_S_M1
 *       DFE_JESD_JESDRX_LINK0_CFG4_REG_SUBCLASSV
 *       DFE_JESD_JESDRX_LINK0_CFG7_REG_MIN_LATENCY_ENA
 *       DFE_JESD_JESDRX_LINK0_CFG8_REG_SYNC_REQUEST_ENA
 *       DFE_JESD_JESDRX_LINK0_CFG3_REG_N_M1
 *       DFE_JESD_JESDRX_LINK0_CFG1_REG_PHADJ
 *       DFE_JESD_JESDRX_LINK0_CFG0_REG_BID
 *       DFE_JESD_JESDRX_LINK0_CFG1_REG_SCR
 *       DFE_JESD_JESDRX_LINK0_CFG3_REG_CS
 *       DFE_JESD_JESDRX_LINK0_CFG0_REG_DID
 *       DFE_JESD_JESDRX_LINK0_CFG7_REG_RBD_M1
 *       DFE_JESD_JESDRX_LINK0_CFG1_REG_L_M1
 *       DFE_JESD_JESDRX_LINK0_CFG4_REG_JESDV
 *       DFE_JESD_JESDRX_LINK0_CFG0_REG_ADJCNT
 *       DFE_JESD_JESDRX_LINK0_CFG3_REG_M_M1
 *       DFE_JESD_JESDRX_LINK0_CFG4_REG_NPRIME_M1
 *       DFE_JESD_JESDRX_LINK0_CFG6_REG_RES2
 *       DFE_JESD_JESDRX_LINK0_CFG2_REG_K_M1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxConfigLink(DfeFl_JesdHandle hJesd, DfeFl_JesdRxLinkConfig *arg)
{
    volatile uint32_t *regs[2];
    Uint16 ui, cfg[10];
    
    regs[0] = &hJesd->regs->jesdrx_link0_cfg0;
    regs[1] = &hJesd->regs->jesdrx_link1_cfg0;
    
    cfg[0] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG0_REG_DID, arg->deviceId)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG0_REG_BID, arg->bankId)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG0_REG_ADJCNT, arg->adjCnt);

    cfg[1] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG1_REG_PHADJ, arg->phyAdj)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG1_REG_ADJDIR, arg->adjDir)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG1_REG_L_M1, arg->numLanesM1)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG1_REG_SCR, arg->scramble);

    cfg[2] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG2_REG_F_M1, arg->numOctetsPerFrameM1)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG2_REG_K_M1, arg->numFramesPerMultiframe);
           
    cfg[3] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG3_REG_M_M1, arg->numCnvtsM1)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG3_REG_N_M1, arg->cnvtResolutionM1)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG3_REG_CS, arg->numCtrlBits);
    
    cfg[4] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG4_REG_NPRIME_M1, arg->totalBitsPerSample)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG4_REG_SUBCLASSV, arg->subclass)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG4_REG_S_M1, arg->numSamplesPerFrameM1)
           |  CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG4_REG_JESDV, arg->jesd204Ver);
    
    cfg[5] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG5_REG_CF, arg->numCtrlWordsPerFrame)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG5_REG_HD, arg->hiDesity)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG5_REG_RES1, arg->rsvd1);

    cfg[6] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG6_REG_RES2, arg->rsvd2);
           
    cfg[7] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG7_REG_RBD_M1, arg->rbdM1)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG7_REG_MIN_LATENCY_ENA, arg->minLatencyEnable)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG7_REG_MATCH_SPECIFIC, arg->matchSpecific)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG7_REG_MATCH_CTRL, arg->matchCtrl)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG7_REG_MATCH_DATA, arg->matchData);
           
    cfg[8] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG8_REG_SYNC_REQUEST_ENA, arg->syncRequestEnable)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG8_REG_ERROR_ENA, arg->errorEnable);

    cfg[9] = CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG9_REG_SYSREF_MODE, arg->sysrefMode)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG9_REG_NO_LANE_SYNC, arg->noLaneSync)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG9_REG_DISABLE_ERR_REPORT, arg->disableErrReport)
           | CSL_FMK(DFE_JESD_JESDRX_LINK0_CFG9_REG_MP_LINK_ENA, arg->mpLinkEnable);
    
    if(arg->link == DFE_FL_JESD_LINK_ALL)
    {
        for(ui = 0; ui <= 9; ui++)
        {
            regs[0][ui] = cfg[ui];
            regs[1][ui] = cfg[ui];
        }
    }
    else if(arg->link <= DFE_FL_JESD_LINK_1)
    {
        for(ui = 0; ui <= 9; ui++)
        {
            regs[arg->link][ui] = cfg[ui];
        }        
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetSysrefMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
         mode    [add content]
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
 *       DFE_JESD_JESDRX_LINK0_CFG9_REG_SYSREF_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_JesdRxSetSysrefMode(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t mode)
{
    volatile uint32_t *regs[2];
    
    regs[0] = &hJesd->regs->jesdrx_link0_cfg0;
    regs[1] = &hJesd->regs->jesdrx_link1_cfg0;

    if(link <= DFE_FL_JESD_LINK_1)
    {
        CSL_FINS(regs[link][9], DFE_JESD_JESDRX_LINK0_CFG9_REG_SYSREF_MODE, mode);
    }
    else
    {
        CSL_FINS(regs[0][9], DFE_JESD_JESDRX_LINK0_CFG9_REG_SYSREF_MODE, mode);
        CSL_FINS(regs[1][9], DFE_JESD_JESDRX_LINK0_CFG9_REG_SYSREF_MODE, mode);
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetSysrefMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDRX_LINK0_CFG9_REG_SYSREF_MODE
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdRxGetSysrefMode(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t *sysrefMode)
{
    volatile uint32_t *regs[2];
    
    regs[0] = &hJesd->regs->jesdrx_link0_cfg0;
    regs[1] = &hJesd->regs->jesdrx_link1_cfg0;

    if(link <= DFE_FL_JESD_LINK_1)
    {
        *sysrefMode = CSL_FEXT(regs[link][9], DFE_JESD_JESDRX_LINK0_CFG9_REG_SYSREF_MODE);
        return DFE_FL_SOK;
    }
    else
    {
        *sysrefMode = 0xdeadbeefu;
        return DFE_FL_INVPARAMS;
    }    
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxClearLinkErrCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
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
dfeFl_JesdRxClearLinkErrCnt(DfeFl_JesdHandle  hJesd, uint32_t link)
{
    volatile uint32_t *regs[2];
    
    regs[0] = &hJesd->regs->jesdrx_link0_cfg11;
    regs[1] = &hJesd->regs->jesdrx_link1_cfg11;
    
    if(link == DFE_FL_JESD_LINK_ALL)
    {
        // write '1' to clear
        *regs[0] = 1;
        *regs[1] = 1;
    }
    else if(link <= DFE_FL_JESD_LINK_1)
    {
        // write '1' to clear
        *regs[link] = 1;        
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetLinkErrCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         link    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_JESD_JESDRX_LINK0_CFG11_REG_ERR_CNT
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdRxGetLinkErrCnt(DfeFl_JesdHandle hJesd, uint32_t link, uint32_t *errCnt)
{
    volatile uint32_t *regs[2];
    
    regs[0] = &hJesd->regs->jesdrx_link0_cfg11;
    regs[1] = &hJesd->regs->jesdrx_link1_cfg11;
    
    if(link <= DFE_FL_JESD_LINK_1)
    {
        *errCnt = CSL_FEXT(*regs[link], DFE_JESD_JESDRX_LINK0_CFG11_REG_ERR_CNT);
        return DFE_FL_SOK;
    }
    else
    {
        *errCnt = 0xdeadbeefu;
        return DFE_FL_INVPARAMS;
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxEnableLaneIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxEnableLaneIntrs(DfeFl_JesdHandle  hJesd, DfeFl_JesdRxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *)&data;
        
    regs[0] = &hJesd->regs->jesdrx_intr_lane0_mask;    
    regs[1] = &hJesd->regs->jesdrx_intr_lane1_mask;    
    regs[2] = &hJesd->regs->jesdrx_intr_lane2_mask;    
    regs[3] = &hJesd->regs->jesdrx_intr_lane3_mask;    
    
    mask->decoder_disp_err      = intrs->decDispErrIntr;
    mask->decoder_code_err      = intrs->decCodeErrIntr;
    mask->code_sync_err         = intrs->codeSyncErrIntr;
    mask->buf_match_err         = intrs->bufMatchErrIntr;
    mask->buf_overflow_err      = intrs->bufOverflowErrIntr;
    mask->link_config_err       = intrs->linkConfigErrIntr;
    mask->frame_align_err       = intrs->frameAlignErrIntr;
    mask->multiframe_align_err  = intrs->multiframeAlignErrIntr;
    mask->fifo_empty            = intrs->fifoEmptyIntr;
    mask->fifo_read_error       = intrs->fifoReadErrIntr;
    mask->fifo_full             = intrs->fifoFullIntr;
    mask->fifo_write_error      = intrs->fifoWriteErrIntr;
    mask->test_seq_err          = intrs->testSeqErrIntr;
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] | data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        *regs[ui] = *regs[ui] | data;
    }            
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxDisableLaneIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxDisableLaneIntrs(DfeFl_JesdHandle  hJesd, DfeFl_JesdRxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *)&data;
        
    regs[0] = &hJesd->regs->jesdrx_intr_lane0_mask;    
    regs[1] = &hJesd->regs->jesdrx_intr_lane1_mask;    
    regs[2] = &hJesd->regs->jesdrx_intr_lane2_mask;    
    regs[3] = &hJesd->regs->jesdrx_intr_lane3_mask;    
            
    mask->decoder_disp_err      = intrs->decDispErrIntr;
    mask->decoder_code_err      = intrs->decCodeErrIntr;
    mask->code_sync_err         = intrs->codeSyncErrIntr;
    mask->buf_match_err         = intrs->bufMatchErrIntr;
    mask->buf_overflow_err      = intrs->bufOverflowErrIntr;
    mask->link_config_err       = intrs->linkConfigErrIntr;
    mask->frame_align_err       = intrs->frameAlignErrIntr;
    mask->multiframe_align_err  = intrs->multiframeAlignErrIntr;
    mask->fifo_empty            = intrs->fifoEmptyIntr;
    mask->fifo_read_error       = intrs->fifoReadErrIntr;
    mask->fifo_full             = intrs->fifoFullIntr;
    mask->fifo_write_error      = intrs->fifoWriteErrIntr;
    mask->test_seq_err          = intrs->testSeqErrIntr;
    data = ~data;
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] & data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        *regs[ui] = *regs[ui] & data;
    }                
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxClearLaneIntrsStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxClearLaneIntrsStatus(DfeFl_JesdHandle  hJesd, DfeFl_JesdRxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *)&data;
        
    regs[0] = &hJesd->regs->jesdrx_intr_lane0_intr;    
    regs[1] = &hJesd->regs->jesdrx_intr_lane1_intr;    
    regs[2] = &hJesd->regs->jesdrx_intr_lane2_intr;    
    regs[3] = &hJesd->regs->jesdrx_intr_lane3_intr;    
            
    mask->decoder_disp_err      = intrs->decDispErrIntr;
    mask->decoder_code_err      = intrs->decCodeErrIntr;
    mask->code_sync_err         = intrs->codeSyncErrIntr;
    mask->buf_match_err         = intrs->bufMatchErrIntr;
    mask->buf_overflow_err      = intrs->bufOverflowErrIntr;
    mask->link_config_err       = intrs->linkConfigErrIntr;
    mask->frame_align_err       = intrs->frameAlignErrIntr;
    mask->multiframe_align_err  = intrs->multiframeAlignErrIntr;
    mask->fifo_empty            = intrs->fifoEmptyIntr;
    mask->fifo_read_error       = intrs->fifoReadErrIntr;
    mask->fifo_full             = intrs->fifoFullIntr;
    mask->fifo_write_error      = intrs->fifoWriteErrIntr;
    mask->test_seq_err          = intrs->testSeqErrIntr;
    data = ~data;
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] & data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        *regs[ui] = *regs[ui] & data;
    }                
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetForceLaneIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxSetForceLaneIntrs(DfeFl_JesdHandle  hJesd, DfeFl_JesdRxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *)&data;
        
    regs[0] = &hJesd->regs->jesdrx_intr_lane0_force;    
    regs[1] = &hJesd->regs->jesdrx_intr_lane1_force;    
    regs[2] = &hJesd->regs->jesdrx_intr_lane2_force;    
    regs[3] = &hJesd->regs->jesdrx_intr_lane3_force;    
    
    mask->decoder_disp_err      = intrs->decDispErrIntr;
    mask->decoder_code_err      = intrs->decCodeErrIntr;
    mask->code_sync_err         = intrs->codeSyncErrIntr;
    mask->buf_match_err         = intrs->bufMatchErrIntr;
    mask->buf_overflow_err      = intrs->bufOverflowErrIntr;
    mask->link_config_err       = intrs->linkConfigErrIntr;
    mask->frame_align_err       = intrs->frameAlignErrIntr;
    mask->multiframe_align_err  = intrs->multiframeAlignErrIntr;
    mask->fifo_empty            = intrs->fifoEmptyIntr;
    mask->fifo_read_error       = intrs->fifoReadErrIntr;
    mask->fifo_full             = intrs->fifoFullIntr;
    mask->fifo_write_error      = intrs->fifoWriteErrIntr;
    mask->test_seq_err          = intrs->testSeqErrIntr;
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] | data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        *regs[ui] = *regs[ui] | data;
    }            
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxClearForceLaneIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxClearForceLaneIntrs(DfeFl_JesdHandle  hJesd, DfeFl_JesdRxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t ui, data = 0;
    CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *)&data;
        
    regs[0] = &hJesd->regs->jesdrx_intr_lane0_force;    
    regs[1] = &hJesd->regs->jesdrx_intr_lane1_force;    
    regs[2] = &hJesd->regs->jesdrx_intr_lane2_force;    
    regs[3] = &hJesd->regs->jesdrx_intr_lane3_force;    
            
    mask->decoder_disp_err      = intrs->decDispErrIntr;
    mask->decoder_code_err      = intrs->decCodeErrIntr;
    mask->code_sync_err         = intrs->codeSyncErrIntr;
    mask->buf_match_err         = intrs->bufMatchErrIntr;
    mask->buf_overflow_err      = intrs->bufOverflowErrIntr;
    mask->link_config_err       = intrs->linkConfigErrIntr;
    mask->frame_align_err       = intrs->frameAlignErrIntr;
    mask->multiframe_align_err  = intrs->multiframeAlignErrIntr;
    mask->fifo_empty            = intrs->fifoEmptyIntr;
    mask->fifo_read_error       = intrs->fifoReadErrIntr;
    mask->fifo_full             = intrs->fifoFullIntr;
    mask->fifo_write_error      = intrs->fifoWriteErrIntr;
    mask->test_seq_err          = intrs->testSeqErrIntr;
    data = ~data;
    
    if(intrs->lane == DFE_FL_JESD_LANE_ALL)
    {
        for(ui = 0; ui < 4; ui++)
        {
            *regs[ui] = *regs[ui] & data;
        }
    }
    else if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        ui = intrs->lane;
        *regs[ui] = *regs[ui] & data;
    }                
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetLaneIntrsStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
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
CSL_IDEF_INLINE DfeFl_Status 
dfeFl_JesdRxGetLaneIntrsStatus(DfeFl_JesdHandle hJesd, DfeFl_JesdRxLaneIntrs *intrs)
{
    volatile uint32_t *regs[4];
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_LANE0_MASK_REG *)&data;
    
    regs[0] = &hJesd->regs->jesdrx_intr_lane0_intr;    
    regs[1] = &hJesd->regs->jesdrx_intr_lane1_intr;    
    regs[2] = &hJesd->regs->jesdrx_intr_lane2_intr;    
    regs[3] = &hJesd->regs->jesdrx_intr_lane3_intr;    
    
    if(intrs->lane <= DFE_FL_JESD_LANE_3)
    {
        data = *regs[intrs->lane];
                
        intrs->decDispErrIntr           = mask->decoder_disp_err;
        intrs->decCodeErrIntr           = mask->decoder_code_err;  
        intrs->codeSyncErrIntr          = mask->code_sync_err;   
        intrs->bufMatchErrIntr          = mask->buf_match_err;       
        intrs->bufOverflowErrIntr       = mask->buf_overflow_err;
        intrs->linkConfigErrIntr        = mask->link_config_err;    
        intrs->frameAlignErrIntr        = mask->frame_align_err;     
        intrs->multiframeAlignErrIntr   = mask->multiframe_align_err;
        intrs->fifoEmptyIntr            = mask->fifo_empty;
        intrs->fifoReadErrIntr        = mask->fifo_read_error;
        intrs->fifoFullIntr             = mask->fifo_full;
        intrs->fifoWriteErrIntr       = mask->fifo_write_error;    
        intrs->testSeqErrIntr           = mask->test_seq_err;    
        
        return DFE_FL_SOK;
    }
    else
    {
        return DFE_FL_INVPARAMS;
    }        
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxEnableSysrefIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxEnableSysrefIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdRxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdrx_intr_sysref_mask |= data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxDisableSysrefIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxDisableSysrefIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdRxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdrx_intr_sysref_mask &= ~data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxClearSysrefIntrsStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxClearSysrefIntrsStatus(DfeFl_JesdHandle hJesd, DfeFl_JesdRxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdrx_intr_sysref_intr &= ~data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxSetForceSysrefIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxSetForceSysrefIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdRxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdrx_intr_sysref_force |= data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxClearForceSysrefIntrs
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxClearForceSysrefIntrs(DfeFl_JesdHandle hJesd, DfeFl_JesdRxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDRX_INTR_SYSREF_MASK_REG *)&data;
    
    mask->sysref_request_assert = intrs->reqAssertIntr;
    mask->sysref_request_deassert = intrs->reqDeassertIntr;
    mask->sysref_err_link0 = intrs->errLink0Intr;
    mask->sysref_err_link1 = intrs->errLink1Intr;
    
    hJesd->regs->jesdrx_intr_sysref_force &= ~data;
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetSysrefIntrsStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         intrs    [add content]
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
dfeFl_JesdRxGetSysrefIntrsStatus(DfeFl_JesdHandle hJesd, DfeFl_JesdRxSysrefIntrs *intrs)
{
    uint32_t data = 0;
    CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *mask = (CSL_DFE_JESD_JESDTX_INTR_SYSREF_MASK_REG *)&data;
    
    data = hJesd->regs->jesdtx_intr_sysref_intr;
    
    intrs->reqAssertIntr = mask->sysref_request_assert;
    intrs->reqDeassertIntr = mask->sysref_request_deassert;
    intrs->errLink0Intr = mask->sysref_err_link0;
    intrs->errLink1Intr = mask->sysref_err_link1;    
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxConfigMapNibb
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
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
 *       DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_ZERO_BITS
 *       DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_LANE_NIBBLE_SEL
 *       DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_TIME_SLOT_SEL
 *       DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_LANE_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_JesdRxConfigMapNibb(DfeFl_JesdHandle hJesd, DfeFl_JesdRxMapNibbConfig *arg)
{
    volatile uint32_t *regs[6];
    uint32_t nibbStep, iNibb, iPos, data;
    
    nibbStep = ((uint32_t)&hJesd->regs->jesdrx_map_nibble01_position0 - (uint32_t)&hJesd->regs->jesdrx_map_nibble00_position0)/4;
    // rxbus rx0i
    regs[0] = &hJesd->regs->jesdrx_map_nibble00_position0;
    // rxbus rx0q
    regs[1] = &hJesd->regs->jesdrx_map_nibble04_position0;
    // rxbus rx1i_fb0i
    regs[2] = &hJesd->regs->jesdrx_map_nibble08_position0;
    // rxbus rx1q_fb0q
    regs[3] = &hJesd->regs->jesdrx_map_nibble12_position0;
    // rxbus rx2i_fb1i
    regs[4] = &hJesd->regs->jesdrx_map_nibble16_position0;
    // rxbus rx2q_fb1q
    regs[5] = &hJesd->regs->jesdrx_map_nibble20_position0;
    
    if(arg->rxBus == DFE_FL_JESDRX_RXBUS_ALL)
    {
        for(iNibb = 0; iNibb < 4; iNibb++)
        {
            for(iPos = 0; iPos < 4; iPos++)
            {
                data = CSL_FMK(DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_LANE_SEL, arg->laneSel[iNibb][iPos])
                     | CSL_FMK(DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_LANE_NIBBLE_SEL, arg->laneNibbSel[iNibb][iPos])
                     | CSL_FMK(DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_TIME_SLOT_SEL, arg->timeSlotSel[iNibb][iPos])
                     | CSL_FMK(DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_ZERO_BITS, arg->zeroBits[iNibb][iPos]);

                regs[0][iNibb*nibbStep+iPos] = data;                     
                regs[1][iNibb*nibbStep+iPos] = data;                     
                regs[2][iNibb*nibbStep+iPos] = data;                     
                regs[3][iNibb*nibbStep+iPos] = data;                     
                regs[4][iNibb*nibbStep+iPos] = data;                     
                regs[5][iNibb*nibbStep+iPos] = data;                     
            }
        }
    }
    else if(arg->rxBus <= DFE_FL_JESDRX_RXBUS_RX2Q_FB1Q)
    {
        for(iNibb = 0; iNibb < 4; iNibb++)
        {
            for(iPos = 0; iPos < 4; iPos++)
            {
                data = CSL_FMK(DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_LANE_SEL, arg->laneSel[iNibb][iPos])
                     | CSL_FMK(DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_LANE_NIBBLE_SEL, arg->laneNibbSel[iNibb][iPos])
                     | CSL_FMK(DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_TIME_SLOT_SEL, arg->timeSlotSel[iNibb][iPos])
                     | CSL_FMK(DFE_JESD_JESDRX_MAP_NIBBLE00_POSITION0_REG_ZERO_BITS, arg->zeroBits[iNibb][iPos]);

                regs[arg->rxBus][iNibb*nibbStep+iPos] = data;                     
            }
        }
    }
}

/** ============================================================================
 *   @n@b dfeFl_JesdRxGetLaneTestData
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hJesd    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
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
CSL_IDEF_INLINE DfeFl_Status
dfeFl_JesdRxGetLaneTestData(DfeFl_JesdHandle hJesd, DfeFl_JesdRxLaneTestDataQuery *arg)
{
    volatile uint32_t *regs[4];
    
    regs[0] = &hJesd->regs->jesdrx_map_test_lane0_position0;
    regs[1] = &hJesd->regs->jesdrx_map_test_lane1_position0;
    regs[2] = &hJesd->regs->jesdrx_map_test_lane2_position0;
    regs[3] = &hJesd->regs->jesdrx_map_test_lane3_position0;
    
    if(arg->lane <= DFE_FL_JESD_LANE_3)
    {
        arg->testData[0] = regs[arg->lane][0];
        arg->testData[1] = regs[arg->lane][1];
        arg->testData[2] = regs[arg->lane][2];
        arg->testData[3] = regs[arg->lane][3];
        
        return DFE_FL_SOK;
    }
    else
    {
        arg->testData[0] = 0xdeadbeefu;
        arg->testData[1] = 0xdeadbeefu;
        arg->testData[2] = 0xdeadbeefu;
        arg->testData[3] = 0xdeadbeefu;
        
        return DFE_FL_INVPARAMS;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_JESDAUX_H_ */
