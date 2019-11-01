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

/** @file dfe_fl_dpdGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_DpdGetHwStatus()
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
#include <ti/drv/dfe/dfe_fl_dpdAux.h>

static DfeFl_Status  dfeDpdQueryBlk0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *Ssel);
static DfeFl_Status  dfeDpdQueryBlk1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *Ssel);
static DfeFl_Status  dfeDpdQueryBlk2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *Ssel);
static DfeFl_Status  dfeDpdQueryBlk3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *Ssel);
static DfeFl_Status  dfeDpdQueryBlk0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
);
static DfeFl_Status  dfeDpdQueryBlk1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
);
static DfeFl_Status  dfeDpdQueryBlk2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
);
static DfeFl_Status  dfeDpdQueryBlk3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
);


/** ============================================================================
 *   @n@b dfeFl_DpdGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Dpd module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd        Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_DpdObj objDfeDpd[DFE_FL_DPD_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_DpdHandle hDfeDpd[DFE_FL_DPD_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;
         DfeFl_DpdCellCurLutMpu CurLutMpu;

         // open DFE
         dfeFl_Init(&dfeCtx);
         dfeParam.flags = 0;
         hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, &status);
         if(status != DFE_FL_SOK)
         {
         	 return FAIL;
         }

         for(i = 0; i < DFE_FL_DPD_PER_CNT; i++)
         {
		 	 hDfeDpd[i] = dfeFl_DpdOpen(hDfe, &objDfeDpd[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // Get the curLutMpu
         CurLutMpu.idxBlk = 0;
         CurLutMpu.idxRow = 0;
         CurLutMpu.idxCell = 0;
         dfeFl_DpdGetHwStatus(hDfeDpd[0], DFE_FL_DPD_QUERY_CELL_CURRENT_LUT_MPU, &CurLutMpu);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_DpdGetHwStatus
(
    DfeFl_DpdHandle             hDfeDpd,
    DfeFl_DpdHwStatusQuery      queryId,
    void                         *arg
)
{
    uint32_t i, j, k;
    DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    // subchip_mode
    case DFE_FL_DPD_QUERY_SUBCHIP_MODE:
    	dfeFl_DpdQuerySubchipMode(hDfeDpd, (uint32_t *)arg);
    	break;
    // subsample
    case DFE_FL_DPD_QUERY_SUBSAMPLE:
    	dfeFl_DpdQuerySubsample(hDfeDpd, (uint32_t *)arg);
    	break;
	// mux square root
	case DFE_FL_DPD_QUERY_MUX_SQRT:
	{
		dfeFl_DpdQueryMuxSqrt(hDfeDpd, (DfeFl_DpdMuxSqrt *)arg);
		break;
	}
	// mux complex signal
	case DFE_FL_DPD_QUERY_MUX_COMPLX:
	{
		dfeFl_DpdQueryMuxComplx(hDfeDpd, (DfeFl_DpdMuxSignal *)arg);
		break;
	}
	// mux real magnitude
	case DFE_FL_DPD_QUERY_MUX_REAL_MAG:
	{
		dfeFl_DpdQueryMuxRealMag(hDfeDpd, (DfeFl_DpdMuxSignal *)arg);
		break;
	}
	// mux dpd output
	case DFE_FL_DPD_QUERY_MUX_OUTPUT:
	{
		dfeFl_DpdQueryMuxOutput(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
		break;
	}
	// dpdadapt update mode
	case DFE_FL_DPD_QUERY_DPDADAPT_MODE:
	{
		dfeFl_DpdQueryDpdadaptMode(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
		break;
	}
	// dpdadapt mux fsync
	case DFE_FL_DPD_QUERY_DPDADAPT_FSYNC:
	{
		dfeFl_DpdQueryDpdadaptFsync(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
		break;
	}
	// dpdadapt mux csync
	case DFE_FL_DPD_QUERY_DPDADAPT_CSYNC:
	{
		dfeFl_DpdQueryDpdadaptCsync(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
		break;
	}
    // f_sync selection for each block
    case DFE_FL_DPD_QUERY_BLK_F_SSEL:
    {
    	DfeFl_DpdBlkSsel *cfg = (DfeFl_DpdBlkSsel *)arg;

    	dfeFl_DpdQueryBlkfSsel(hDfeDpd, cfg->idxBlk, &cfg->Ssel);

    	break;
    }
    // f_sync selection for each subchip
    case DFE_FL_DPD_QUERY_SUBCHIP_F_SSEL:
    {
    	DfeFl_DpdSubchipSsel *cfg = (DfeFl_DpdSubchipSsel *)arg;

    	for (i = 0; i < cfg->numBlks; i++)
    	{
    		dfeFl_DpdQueryBlkfSsel(hDfeDpd, cfg->BlkSsel[i].idxBlk, &cfg->BlkSsel[i].Ssel);
    	}

    	break;
    }
    // c_sync selection for each block
    case DFE_FL_DPD_QUERY_BLK_C_SSEL:
    {
    	DfeFl_DpdBlkSsel *cfg = (DfeFl_DpdBlkSsel *)arg;

    	dfeFl_DpdQueryBlkcSsel(hDfeDpd, cfg->idxBlk, &cfg->Ssel);

    	break;
    }
    // c_sync selection for each subchip
    case DFE_FL_DPD_QUERY_SUBCHIP_C_SSEL:
    {
    	DfeFl_DpdSubchipSsel *cfg = (DfeFl_DpdSubchipSsel *)arg;

    	for (i = 0; i < cfg->numBlks; i++)
    	{
    		dfeFl_DpdQueryBlkcSsel(hDfeDpd, cfg->BlkSsel[i].idxBlk, &cfg->BlkSsel[i].Ssel);
    	}

    	break;
    }
    // diable dpd
    case DFE_FL_DPD_QUERY_DPD_DISABLE:
    {
    	dfeFl_DpdQueryDpdDisable(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
    	break;
    }
    // sync selection for syncB
    case DFE_FL_DPD_QUERY_SYNCB_SSEL:
    	dfeFl_DpdQuerySyncBSsel(hDfeDpd, (uint32_t *)arg);
    	break;
    // inits ssel
    case DFE_FL_DPD_QUERY_INITS_SSEL:
    	dfeFl_DpdQueryInitsSsel(hDfeDpd, (uint32_t *)arg);
    	break;
    // init clk gate
    case DFE_FL_DPD_QUERY_INIT_CLK_GATE:
    	dfeFl_DpdQueryInitClkGate(hDfeDpd, (uint32_t *)arg);
    	break;
    // init state
    case DFE_FL_DPD_QUERY_INIT_STATE:
    	dfeFl_DpdQueryInitState(hDfeDpd, (uint32_t *)arg);
    	break;
    // clear data
    case DFE_FL_DPD_QUERY_CLEAR_DATA:
    	dfeFl_DpdQueryClearData(hDfeDpd, (uint32_t *)arg);
    	break;
    // dpd input scale
    case DFE_FL_DPD_QUERY_DPDINPUT_SCALE:
    	dfeFl_DpdQueryDpdInputScale(hDfeDpd, (uint32_t *)arg);
    	break;
	// test signal generation config
	case DFE_FL_DPD_QUERY_TESTGEN_CFG:
		dfeFl_DpdQueryTestGenConfig(hDfeDpd, (DfeFl_DpdTestGenConfig *)arg);
		break;
	// test signal generation sync selection
	case DFE_FL_DPD_QUERY_TESTGEN_SSEL:
	{
		DfeFl_DpdTestGenSsel *cfg = (DfeFl_DpdTestGenSsel *) arg;

		dfeFl_DpdGetTestGenSsel(hDfeDpd, cfg->tgDev, &cfg->ssel);
		break;
	}
	// chksum
	case DFE_FL_DPD_QUERY_CHKSUM_RESULT:
		dfeFl_DpdGetChksumResult(hDfeDpd, (uint32_t *)arg);
		break;
	// clk gate delay
	case DFE_FL_DPD_QUERY_CLK_GATE_DELAY:
		dfeFl_DpdQueryClkgateDelay(hDfeDpd, (uint32_t *)arg);
		break;
	// test bus control
	case DFE_FL_DPD_QUERY_TEST_BUS_CTRL:
		dfeFl_DpdQueryTestbusCtrl(hDfeDpd, (uint32_t *)arg);
		break;
	// mux_blk in each dpd block
	case DFE_FL_DPD_QUERY_MUX_BLK:
	{
		dfeFl_DpdQueryMuxBlk(hDfeDpd, (DfeFl_DpdMuxBlk *)arg);
		break;
	}
	// mux_blk_row in each dpd row
	case DFE_FL_DPD_QUERY_MUX_BLK_ROW:
	{
    	DfeFl_DpdMuxBlkRow *cfg = (DfeFl_DpdMuxBlkRow *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		dfeFl_DpdQueryMuxBlk0Row(hDfeDpd, cfg);
    		break;
    	case DFE_FL_DPD_B1:
    		dfeFl_DpdQueryMuxBlk1Row(hDfeDpd, cfg);
    		break;
    	case DFE_FL_DPD_B2:
    	    dfeFl_DpdQueryMuxBlk2Row(hDfeDpd, cfg);
    	    break;
    	case DFE_FL_DPD_B3:
    	    dfeFl_DpdQueryMuxBlk3Row(hDfeDpd, cfg);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
	}
    // lut init for each row
    case DFE_FL_DPD_QUERY_ROW_LUT_INIT:
    {
    	DfeFl_DpdRowLutInit *cfg = (DfeFl_DpdRowLutInit *)arg;

    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		dfeFl_DpdQueryBlk0LutInit(hDfeDpd, cfg->idxRow, &cfg->data);
    		break;
    	case DFE_FL_DPD_B1:
    		dfeFl_DpdQueryBlk1LutInit(hDfeDpd, cfg->idxRow, &cfg->data);
    		break;
    	case DFE_FL_DPD_B2:
    	    dfeFl_DpdQueryBlk2LutInit(hDfeDpd, cfg->idxRow, &cfg->data);
    	    break;
    	case DFE_FL_DPD_B3:
    	    dfeFl_DpdQueryBlk3LutInit(hDfeDpd, cfg->idxRow, &cfg->data);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}

    	break;
    }
    // lut init for each block
    case DFE_FL_DPD_QUERY_BLK_LUT_INIT:
    {
    	DfeFl_DpdBlkLutInit *cfg = (DfeFl_DpdBlkLutInit *)arg;

    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdQueryBlk0LutInit(hDfeDpd, cfg->LutInit[i].idxRow, &cfg->LutInit[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B1:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdQueryBlk1LutInit(hDfeDpd, cfg->LutInit[i].idxRow, &cfg->LutInit[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B2:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdQueryBlk2LutInit(hDfeDpd, cfg->LutInit[i].idxRow, &cfg->LutInit[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B3:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdQueryBlk3LutInit(hDfeDpd, cfg->LutInit[i].idxRow, &cfg->LutInit[i].data);
    		}
    		break;
    	}
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}

    	break;
    }
    // lut toggle for each row
    case DFE_FL_DPD_QUERY_ROW_LUT_TOGGLE:
    {
    	DfeFl_DpdRowLutToggle *cfg = (DfeFl_DpdRowLutToggle *)arg;

    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		dfeFl_DpdQueryBlk0LutToggle(hDfeDpd, cfg->idxRow, &cfg->data);
    		break;
    	case DFE_FL_DPD_B1:
    		dfeFl_DpdQueryBlk1LutToggle(hDfeDpd, cfg->idxRow, &cfg->data);
    		break;
    	case DFE_FL_DPD_B2:
    	    dfeFl_DpdQueryBlk2LutToggle(hDfeDpd, cfg->idxRow, &cfg->data);
    	    break;
    	case DFE_FL_DPD_B3:
    	    dfeFl_DpdQueryBlk3LutToggle(hDfeDpd, cfg->idxRow, &cfg->data);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}

    	break;

    }
    // lut toggle for each block
    case DFE_FL_DPD_QUERY_BLK_LUT_TOGGLE:
    {
    	DfeFl_DpdBlkLutToggle *cfg = (DfeFl_DpdBlkLutToggle *)arg;

    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdQueryBlk0LutToggle(hDfeDpd, cfg->LutToggle[i].idxRow, &cfg->LutToggle[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B1:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdQueryBlk1LutToggle(hDfeDpd, cfg->LutToggle[i].idxRow, &cfg->LutToggle[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B2:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdQueryBlk2LutToggle(hDfeDpd, cfg->LutToggle[i].idxRow, &cfg->LutToggle[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B3:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdQueryBlk3LutToggle(hDfeDpd, cfg->LutToggle[i].idxRow, &cfg->LutToggle[i].data);
    		}
    		break;
    	}
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // sync selection for each cell
    case DFE_FL_DPD_QUERY_CELL_SYNC:
    {
    	DfeFl_DpdCellSync *cfg = (DfeFl_DpdCellSync *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		err = dfeDpdQueryBlk0CellSync(hDfeDpd, cfg->idxRow, cfg->idxCell, &cfg->Ssel);
    		break;
    	case DFE_FL_DPD_B1:
    		err = dfeDpdQueryBlk0CellSync(hDfeDpd, cfg->idxRow, cfg->idxCell, &cfg->Ssel);
    		break;
    	case DFE_FL_DPD_B2:
    		err = dfeDpdQueryBlk0CellSync(hDfeDpd, cfg->idxRow, cfg->idxCell, &cfg->Ssel);
    	    break;
    	case DFE_FL_DPD_B3:
    		err = dfeDpdQueryBlk0CellSync(hDfeDpd, cfg->idxRow, cfg->idxCell, &cfg->Ssel);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // sync selection for each row
    case DFE_FL_DPD_QUERY_ROW_SYNC:
    {
    	DfeFl_DpdRowSync *cfg = (DfeFl_DpdRowSync *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i < cfg->numCells; i++)
    			err = dfeDpdQueryBlk0CellSync(hDfeDpd, cfg->idxRow, cfg->CellSync[i].idxCell, &cfg->CellSync[i].data);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i < cfg->numCells; i++)
    			err = dfeDpdQueryBlk1CellSync(hDfeDpd, cfg->idxRow, cfg->CellSync[i].idxCell, &cfg->CellSync[i].data);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i < cfg->numCells; i++)
    			err = dfeDpdQueryBlk2CellSync(hDfeDpd, cfg->idxRow, cfg->CellSync[i].idxCell, &cfg->CellSync[i].data);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i < cfg->numCells; i++)
    			err = dfeDpdQueryBlk3CellSync(hDfeDpd, cfg->idxRow, cfg->CellSync[i].idxCell, &cfg->CellSync[i].data);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // sync selection for each block
    case DFE_FL_DPD_QUERY_BLK_SYNC:
    {
    	DfeFl_DpdBlkSync *cfg = (DfeFl_DpdBlkSync *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    				err = dfeDpdQueryBlk0CellSync(hDfeDpd, cfg->CellSync[i][j].idxRow, cfg->CellSync[i][j].idxCell, &cfg->CellSync[i][j].data);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    				err = dfeDpdQueryBlk1CellSync(hDfeDpd, cfg->CellSync[i][j].idxRow, cfg->CellSync[i][j].idxCell, &cfg->CellSync[i][j].data);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    				err = dfeDpdQueryBlk2CellSync(hDfeDpd, cfg->CellSync[i][j].idxRow, cfg->CellSync[i][j].idxCell, &cfg->CellSync[i][j].data);
    		break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    				err = dfeDpdQueryBlk3CellSync(hDfeDpd, cfg->CellSync[i][j].idxRow, cfg->CellSync[i][j].idxCell, &cfg->CellSync[i][j].data);
    		break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // current lut mpu for each cell
    case DFE_FL_DPD_QUERY_CELL_CURRENT_LUT_MPU:
    {
    	DfeFl_DpdCellCurLutMpu *cfg = (DfeFl_DpdCellCurLutMpu *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		dfeFl_DpdQueryBlk0CurLutMpu(hDfeDpd, cfg->idxRow, cfg->idxCell, &cfg->CurLutMpu);
    		break;
    	case DFE_FL_DPD_B1:
    		dfeFl_DpdQueryBlk1CurLutMpu(hDfeDpd, cfg->idxRow, cfg->idxCell, &cfg->CurLutMpu);
    		break;
    	case DFE_FL_DPD_B2:
    		dfeFl_DpdQueryBlk2CurLutMpu(hDfeDpd, cfg->idxRow, cfg->idxCell, &cfg->CurLutMpu);
    	    break;
    	case DFE_FL_DPD_B3:
    		dfeFl_DpdQueryBlk3CurLutMpu(hDfeDpd, cfg->idxRow, cfg->idxCell, &cfg->CurLutMpu);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // current lut mpu for each row
    case DFE_FL_DPD_QUERY_ROW_CURRENT_LUT_MPU:
    {
    	DfeFl_DpdRowCurLutMpu *cfg = (DfeFl_DpdRowCurLutMpu *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i < cfg->numCells; i++)
    			dfeFl_DpdQueryBlk0CurLutMpu(hDfeDpd, cfg->idxRow, cfg->CurLutMpu[i].idxCell, &cfg->CurLutMpu[i].data);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i < cfg->numCells; i++)
    			dfeFl_DpdQueryBlk1CurLutMpu(hDfeDpd, cfg->idxRow, cfg->CurLutMpu[i].idxCell, &cfg->CurLutMpu[i].data);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i < cfg->numCells; i++)
    			dfeFl_DpdQueryBlk2CurLutMpu(hDfeDpd, cfg->idxRow, cfg->CurLutMpu[i].idxCell, &cfg->CurLutMpu[i].data);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i < cfg->numCells; i++)
    			dfeFl_DpdQueryBlk3CurLutMpu(hDfeDpd, cfg->idxRow, cfg->CurLutMpu[i].idxCell, &cfg->CurLutMpu[i].data);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // current lut mpu for each cell
    case DFE_FL_DPD_QUERY_BLK_CURRENT_LUT_MPU:
    {
    	DfeFl_DpdBlkCurLutMpu *cfg = (DfeFl_DpdBlkCurLutMpu *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    			dfeFl_DpdQueryBlk0CurLutMpu(hDfeDpd, cfg->CurLutMpu[i][j].idxRow, cfg->CurLutMpu[i][j].idxCell, &cfg->CurLutMpu[i][j].data);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    			dfeFl_DpdQueryBlk1CurLutMpu(hDfeDpd, cfg->CurLutMpu[i][j].idxRow, cfg->CurLutMpu[i][j].idxCell, &cfg->CurLutMpu[i][j].data);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    			dfeFl_DpdQueryBlk2CurLutMpu(hDfeDpd, cfg->CurLutMpu[i][j].idxRow, cfg->CurLutMpu[i][j].idxCell, &cfg->CurLutMpu[i][j].data);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    			dfeFl_DpdQueryBlk3CurLutMpu(hDfeDpd, cfg->CurLutMpu[i][j].idxRow, cfg->CurLutMpu[i][j].idxCell, &cfg->CurLutMpu[i][j].data);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // lut value for each entry
    case DFE_FL_DPD_QUERY_ENTRY_LUT:
    {
    	DfeFl_DpdEntryLUT *cfg = (DfeFl_DpdEntryLUT *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		err = dfeDpdQueryBlk0LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->idxEntry, &cfg->lutGain, &cfg->lutSlope);
    		break;
    	case DFE_FL_DPD_B1:
    		err = dfeDpdQueryBlk1LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->idxEntry, &cfg->lutGain, &cfg->lutSlope);
    		break;
    	case DFE_FL_DPD_B2:
    		err = dfeDpdQueryBlk2LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->idxEntry, &cfg->lutGain, &cfg->lutSlope);
    	    break;
    	case DFE_FL_DPD_B3:
    		err = dfeDpdQueryBlk3LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->idxEntry, &cfg->lutGain, &cfg->lutSlope);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // lut value for each cell
    case DFE_FL_DPD_QUERY_CELL_LUT:
    {
    	DfeFl_DpdCellLUT *cfg = (DfeFl_DpdCellLUT *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i<cfg->numEntries; i++)
    			err = dfeDpdQueryBlk0LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->LUTval[i].idxEntry, &cfg->LUTval[i].lutGain, &cfg->LUTval[i].lutSlope);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i<cfg->numEntries; i++)
    			err = dfeDpdQueryBlk1LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->LUTval[i].idxEntry, &cfg->LUTval[i].lutGain, &cfg->LUTval[i].lutSlope);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i<cfg->numEntries; i++)
    			err = dfeDpdQueryBlk2LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->LUTval[i].idxEntry, &cfg->LUTval[i].lutGain, &cfg->LUTval[i].lutSlope);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i<cfg->numEntries; i++)
    			err = dfeDpdQueryBlk3LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->LUTval[i].idxEntry, &cfg->LUTval[i].lutGain, &cfg->LUTval[i].lutSlope);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // lut value for each row
    case DFE_FL_DPD_QUERY_ROW_LUT:
    {
    	DfeFl_DpdRowLUT *cfg = (DfeFl_DpdRowLUT *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i<cfg->numCells; i++)
    			for (j = 0; j<DFE_FL_DPD_MAX_LUT_SIZE; j++)
    				err = dfeDpdQueryBlk0LUT(hDfeDpd, cfg->idxRow, cfg->LUTval[i][j].idxCell, cfg->LUTval[i][j].idxEntry, &cfg->LUTval[i][j].lutGain, &cfg->LUTval[i][j].lutSlope);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i<cfg->numCells; i++)
    			for (j = 0; j<DFE_FL_DPD_MAX_LUT_SIZE; j++)
    				err = dfeDpdQueryBlk1LUT(hDfeDpd, cfg->idxRow, cfg->LUTval[i][j].idxCell, cfg->LUTval[i][j].idxEntry, &cfg->LUTval[i][j].lutGain, &cfg->LUTval[i][j].lutSlope);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i<cfg->numCells; i++)
    			for (j = 0; j<DFE_FL_DPD_MAX_LUT_SIZE; j++)
    				err = dfeDpdQueryBlk2LUT(hDfeDpd, cfg->idxRow, cfg->LUTval[i][j].idxCell, cfg->LUTval[i][j].idxEntry, &cfg->LUTval[i][j].lutGain, &cfg->LUTval[i][j].lutSlope);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i<cfg->numCells; i++)
    			for (j = 0; j<DFE_FL_DPD_MAX_LUT_SIZE; j++)
    				err = dfeDpdQueryBlk3LUT(hDfeDpd, cfg->idxRow, cfg->LUTval[i][j].idxCell, cfg->LUTval[i][j].idxEntry, &cfg->LUTval[i][j].lutGain, &cfg->LUTval[i][j].lutSlope);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // lut value for each block
    case DFE_FL_DPD_QUERY_BLK_LUT:
    {
    	DfeFl_DpdBlkLUT *cfg = (DfeFl_DpdBlkLUT *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i<cfg->numRows; i++)
    			for (j = 0; j<DFE_FL_DPD_NCEL; j++)
    				for (k = 0; k<DFE_FL_DPD_MAX_LUT_SIZE; k++)
    					err = dfeDpdQueryBlk0LUT(hDfeDpd, cfg->LUTval[i][j][k].idxRow, cfg->LUTval[i][j][k].idxCell, \
    							cfg->LUTval[i][j][k].idxEntry, &cfg->LUTval[i][j][k].lutGain, &cfg->LUTval[i][j][k].lutSlope);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i<cfg->numRows; i++)
    			for (j = 0; j<DFE_FL_DPD_NCEL; j++)
    				for (k = 0; k<DFE_FL_DPD_MAX_LUT_SIZE; k++)
    					err = dfeDpdQueryBlk1LUT(hDfeDpd, cfg->LUTval[i][j][k].idxRow, cfg->LUTval[i][j][k].idxCell, \
    							cfg->LUTval[i][j][k].idxEntry, &cfg->LUTval[i][j][k].lutGain, &cfg->LUTval[i][j][k].lutSlope);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i<cfg->numRows; i++)
    			for (j = 0; j<DFE_FL_DPD_NCEL; j++)
    				for (k = 0; k<DFE_FL_DPD_MAX_LUT_SIZE; k++)
    					err = dfeDpdQueryBlk2LUT(hDfeDpd, cfg->LUTval[i][j][k].idxRow, cfg->LUTval[i][j][k].idxCell, \
    							cfg->LUTval[i][j][k].idxEntry, &cfg->LUTval[i][j][k].lutGain, &cfg->LUTval[i][j][k].lutSlope);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i<cfg->numRows; i++)
    			for (j = 0; j<DFE_FL_DPD_NCEL; j++)
    				for (k = 0; k<DFE_FL_DPD_MAX_LUT_SIZE; k++)
    					err = dfeDpdQueryBlk3LUT(hDfeDpd, cfg->LUTval[i][j][k].idxRow, cfg->LUTval[i][j][k].idxCell, \
    							cfg->LUTval[i][j][k].idxEntry, &cfg->LUTval[i][j][k].lutGain, &cfg->LUTval[i][j][k].lutSlope);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
        
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}

/** ============================================================================
 *   @n@b dfeDpdQueryBlk0CellSync
 *
 *   @b Description
 *   @n Get the sync value of each row of Dpd block0
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid handle of Dpd
         idxRow     The index of row in Dpd block0
         idxCell    The index of cell in each Dpd row of Dpd block0
         Ssel       The pointer of the sync selection
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 * ===========================================================================
 */
static DfeFl_Status  dfeDpdQueryBlk0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *Ssel)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdQueryBlk0Row0CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdQueryBlk0Row1CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdQueryBlk0Row2CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdQueryBlk0Row3CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdQueryBlk0Row4CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdQueryBlk0Row5CellSync(hDfeDpd, idxCell, Ssel);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdQueryBlk1CellSync
 *
 *   @b Description
 *   @n Get the sync value of each row of Dpd block1
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid handle of Dpd
         idxRow     The index of row in Dpd block1
         idxCell    The index of cell in each Dpd row of Dpd block1
         Ssel       The pointer of the sync selection
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 * ===========================================================================
 */
static DfeFl_Status  dfeDpdQueryBlk1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *Ssel)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdQueryBlk1Row0CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdQueryBlk1Row1CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdQueryBlk1Row2CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdQueryBlk1Row3CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdQueryBlk1Row4CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdQueryBlk1Row5CellSync(hDfeDpd, idxCell, Ssel);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdQueryBlk2CellSync
 *
 *   @b Description
 *   @n Get the sync value of each row of Dpd block2
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid handle of Dpd
         idxRow     The index of row in Dpd block2
         idxCell    The index of cell in each Dpd row of Dpd block2
         Ssel       The pointer of the sync selection
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 * ===========================================================================
 */
static DfeFl_Status  dfeDpdQueryBlk2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *Ssel)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdQueryBlk2Row0CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdQueryBlk2Row1CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdQueryBlk2Row2CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdQueryBlk2Row3CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdQueryBlk2Row4CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdQueryBlk2Row5CellSync(hDfeDpd, idxCell, Ssel);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdQueryBlk3CellSync
 *
 *   @b Description
 *   @n Get the sync value of each row of Dpd block3
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid handle of Dpd
         idxRow     The index of row in Dpd block3
         idxCell    The index of cell in each Dpd row of Dpd block3
         Ssel       The pointer of the sync selection
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 * ===========================================================================
 */
static DfeFl_Status  dfeDpdQueryBlk3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *Ssel)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdQueryBlk3Row0CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdQueryBlk3Row1CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdQueryBlk3Row2CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdQueryBlk3Row3CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdQueryBlk3Row4CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdQueryBlk3Row5CellSync(hDfeDpd, idxCell, Ssel);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdQueryBlk0LUT
 *
 *   @b Description
 *   @n Get the Lut value of Dpd block0.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid Dpd handle
         idxRow     The index of each row in Dpd block0.
         idxCell    The index of each cell in each Dpd row of Dpd block0.
         idxEntry   The index of each entry in each Dpd lut.
         lutGain    The pointer to the gain value of each lut entry.
         lutSlope   The pointer to the slope value of each lut entry.
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 * ===========================================================================
 */
static DfeFl_Status  dfeDpdQueryBlk0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdQueryBlk0Row0LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdQueryBlk0Row1LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdQueryBlk0Row2LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdQueryBlk0Row3LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdQueryBlk0Row4LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdQueryBlk0Row5LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdQueryBlk1LUT
 *
 *   @b Description
 *   @n Get the Lut value of Dpd block1.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid Dpd handle
         idxRow     The index of each row in Dpd block1.
         idxCell    The index of each cell in each Dpd row of Dpd block1.
         idxEntry   The index of each entry in each Dpd lut.
         lutGain    The pointer to the gain value of each lut entry.
         lutSlope   The pointer to the slope value of each lut entry.
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 * ===========================================================================
 */
static DfeFl_Status  dfeDpdQueryBlk1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdQueryBlk1Row0LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdQueryBlk1Row1LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdQueryBlk1Row2LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdQueryBlk1Row3LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdQueryBlk1Row4LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdQueryBlk1Row5LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdQueryBlk2LUT
 *
 *   @b Description
 *   @n Get the Lut value of Dpd block2.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid Dpd handle
         idxRow     The index of each row in Dpd block2.
         idxCell    The index of each cell in each Dpd row of Dpd block2.
         idxEntry   The index of each entry in each Dpd lut.
         lutGain    The pointer to the gain value of each lut entry.
         lutSlope   The pointer to the slope value of each lut entry.
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 * ===========================================================================
 */
static DfeFl_Status  dfeDpdQueryBlk2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdQueryBlk2Row0LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdQueryBlk2Row1LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdQueryBlk2Row2LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdQueryBlk2Row3LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdQueryBlk2Row4LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdQueryBlk2Row5LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdQueryBlk3LUT
 *
 *   @b Description
 *   @n Get the Lut value of Dpd block3.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid Dpd handle
         idxRow     The index of each row in Dpd block3.
         idxCell    The index of each cell in each Dpd row of Dpd block3.
         idxEntry   The index of each entry in each Dpd lut.
         lutGain    The pointer to the gain value of each lut entry.
         lutSlope   The pointer to the slope value of each lut entry.
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 * ===========================================================================
 */
static DfeFl_Status  dfeDpdQueryBlk3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdQueryBlk3Row0LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdQueryBlk3Row1LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdQueryBlk3Row2LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdQueryBlk3Row3LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdQueryBlk3Row4LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdQueryBlk3Row5LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}
