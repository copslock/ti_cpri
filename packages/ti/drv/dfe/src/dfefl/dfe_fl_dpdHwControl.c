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

/** @file dfe_fl_dpdHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_DpdHwControl()
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

static DfeFl_Status  dfeDpdUpdateBlk0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t Ssel);
static DfeFl_Status  dfeDpdUpdateBlk1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t Ssel);
static DfeFl_Status  dfeDpdUpdateBlk2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t Ssel);
static DfeFl_Status  dfeDpdUpdateBlk3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t Ssel);
static DfeFl_Status  dfeDpdUpdateBlk0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
);
static DfeFl_Status  dfeDpdUpdateBlk1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
);
static DfeFl_Status  dfeDpdUpdateBlk2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
);
static DfeFl_Status  dfeDpdUpdateBlk3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
);
 
/** ============================================================================
 *   @n@b dfeFl_DpdHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd        Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
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
 *   @n  The hardware registers of Dfe Dpd.
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
         DfeFl_SublkInitsConfig inits;

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

         // reset Dpd
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_DpdHwControl(hDfeDpd[0], DFE_FL_DPD_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_DpdHwControl
(
    DfeFl_DpdHandle             hDfeDpd,
    DfeFl_DpdHwControlCmd       ctrlCmd,
    void                         *arg
)
{
    uint32_t i, j, k;
    DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {
    case DFE_FL_DPD_CMD_CFG_INITS:
        dfeFl_DpdConfigInits(hDfeDpd, (DfeFl_SublkInitsConfig *)arg);
        break;
	// subchip_mode
    case DFE_FL_DPD_CMD_UPD_SUBCHIP_MODE:
    {
    	dfeFl_DpdUpdateSubchipMode(hDfeDpd, *(uint32_t *)arg);
    	break;
    }
	// subsample
    case DFE_FL_DPD_CMD_UPD_SUBSAMPLE:
    {
    	dfeFl_DpdUpdateSubsample(hDfeDpd, *(uint32_t *)arg);
    	break;
    }
    // mux square root
    case DFE_FL_DPD_CMD_UPD_MUX_SQRT:
    {
    	dfeFl_DpdUpdateMuxSqrt(hDfeDpd, (DfeFl_DpdMuxSqrt *)arg);
    	break;
    }
    // mux complex signal
    case DFE_FL_DPD_CMD_UPD_MUX_COMPLX:
    {
    	dfeFl_DpdUpdateMuxComplx(hDfeDpd, (DfeFl_DpdMuxSignal *)arg);
    	break;
    }
    // mux real magnitude
    case DFE_FL_DPD_CMD_UPD_MUX_REAL_MAG:
    {
    	dfeFl_DpdUpdateMuxRealMag(hDfeDpd, (DfeFl_DpdMuxSignal *)arg);
    	break;
    }
    // mux dpd output
    case DFE_FL_DPD_CMD_UPD_MUX_OUTPUT:
    {
    	dfeFl_DpdUpdateMuxOutput(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
    	break;
    }
    // dpdadapt update mode
    case DFE_FL_DPD_CMD_UPD_DPDADAPT_MODE:
    {
    	dfeFl_DpdUpdateDpdadaptMode(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
    	break;
    }
    // dpdadapt mux fsync
    case DFE_FL_DPD_CMD_UPD_DPDADAPT_FSYNC:
    {
    	dfeFl_DpdUpdateDpdadaptFsync(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
    	break;
    }
    // dpdadapt mux csync
    case DFE_FL_DPD_CMD_UPD_DPDADAPT_CSYNC:
    {
    	dfeFl_DpdUpdateDpdadaptCsync(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
    	break;
    }
    // config f_sync selection for each block
    case DFE_FL_DPD_CMD_UPD_BLK_F_SSEL:
    {
    	DfeFl_DpdBlkSsel *cfg = (DfeFl_DpdBlkSsel *)arg;

   		dfeFl_DpdUpdateBlkfSsel(hDfeDpd, cfg->idxBlk, cfg->Ssel);

    	break;
    }
    // config f_sync selection for all blocks
    case DFE_FL_DPD_CMD_UPD_SUBCHIP_F_SSEL:
    {
    	DfeFl_DpdSubchipSsel *cfg = (DfeFl_DpdSubchipSsel *)arg;

    	for (i = 0; i < cfg->numBlks; i++)
    	{
    		dfeFl_DpdUpdateBlkfSsel(hDfeDpd, cfg->BlkSsel[i].idxBlk, cfg->BlkSsel[i].Ssel);
    	}

    	break;
    }
    // config c_sync selection for each block
    case DFE_FL_DPD_CMD_UPD_BLK_C_SSEL:
    {
    	DfeFl_DpdBlkSsel *cfg = (DfeFl_DpdBlkSsel *)arg;

    	dfeFl_DpdUpdateBlkcSsel(hDfeDpd, cfg->idxBlk, cfg->Ssel);

    	break;
    }
    // config c_sync selection for all blocks
    case DFE_FL_DPD_CMD_UPD_SUBCHIP_C_SSEL:
    {
    	DfeFl_DpdSubchipSsel *cfg = (DfeFl_DpdSubchipSsel *)arg;

    	for (i = 0; i < cfg->numBlks; i++)
    	{
    		dfeFl_DpdUpdateBlkcSsel(hDfeDpd, cfg->BlkSsel[i].idxBlk, cfg->BlkSsel[i].Ssel);
    	}

    	break;
    }
    // diable dpd
    case DFE_FL_DPD_CMD_SET_DPD_DISABLE:
    {
    	dfeFl_DpdSetDpdDisable(hDfeDpd, (DfeFl_DpdBlkCtrl *)arg);
    	break;
    }
    // config sync selection for syncB
    case DFE_FL_DPD_CMD_UPD_SYNCB_SSEL:
    	dfeFl_DpdUpdateSyncBSsel(hDfeDpd, *(uint32_t *)arg);
    	break;
    // update dpd input scale
    case DFE_FL_DPD_CMD_UPD_DPDINPUT_SCALE:
    	dfeFl_DpdUpdateDpdInputScale(hDfeDpd, *(uint32_t *)arg);
    	break;
    // test signal generation config
    case DFE_FL_DPD_CMD_CFG_TESTGEN:
    	dfeFl_DpdConfigTestGen(hDfeDpd, (DfeFl_DpdTestGenConfig *)arg);
    	break;
    // test signal generation sync selection
    case DFE_FL_DPD_CMD_SET_TESTGEN_SSEL:
    {
        DfeFl_DpdTestGenSsel *cfg = (DfeFl_DpdTestGenSsel *) arg;

        dfeFl_DpdSetTestGenSsel(hDfeDpd, cfg->tgDev, cfg->ssel);
    	break;
    }
    // chksum
    case DFE_FL_DPD_CMD_CFG_CHKSUM:
    	dfeFl_DpdConfigChksum(hDfeDpd, (DfeFl_DpdChksumConfig *)arg);
    	break;
    // chksum sync selection
    case DFE_FL_DPD_CMD_SET_CHKSUM_SSEL:
    	dfeFl_DpdSetChksumSsel(hDfeDpd, *(uint32_t *)arg);
    	break;
    // clk gate delay
    case DFE_FL_DPD_CMD_CLK_GATE_DELAY:
    	dfeFl_DpdSetClkgateDelay(hDfeDpd, *(uint32_t *)arg);
    	break;
    // test bus control
    case DFE_FL_DPD_CMD_TEST_BUS_CTRL:
    	dfeFl_DpdSetTestbusCtrl(hDfeDpd, *(uint32_t *)arg);
    	break;
    // update mux_blk in each dpd block
    case DFE_FL_DPD_CMD_UPD_MUX_BLK:
    {
    	dfeFl_DpdUpdateMuxBlk(hDfeDpd, (DfeFl_DpdMuxBlk *)arg);
    	break;
    }
    // update mux_blk_row in each dpd row
    case DFE_FL_DPD_CMD_UPD_MUX_BLK_ROW:
    {
    	DfeFl_DpdMuxBlkRow *cfg = (DfeFl_DpdMuxBlkRow *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		dfeFl_DpdUpdateMuxBlk0Row(hDfeDpd, cfg);
    		break;
    	case DFE_FL_DPD_B1:
    		dfeFl_DpdUpdateMuxBlk1Row(hDfeDpd, cfg);
    		break;
    	case DFE_FL_DPD_B2:
    	    dfeFl_DpdUpdateMuxBlk2Row(hDfeDpd, cfg);
    	    break;
    	case DFE_FL_DPD_B3:
    	    dfeFl_DpdUpdateMuxBlk3Row(hDfeDpd, cfg);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // update lut init for each row
    case DFE_FL_DPD_CMD_UPD_ROW_LUT_INIT:
    {
    	DfeFl_DpdRowLutInit *cfg = (DfeFl_DpdRowLutInit *)arg;

    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		dfeFl_DpdUpdateBlk0LutInit(hDfeDpd, cfg->idxRow, cfg->data);
    		break;
    	case DFE_FL_DPD_B1:
    		dfeFl_DpdUpdateBlk1LutInit(hDfeDpd, cfg->idxRow, cfg->data);
    		break;
    	case DFE_FL_DPD_B2:
    	    dfeFl_DpdUpdateBlk2LutInit(hDfeDpd, cfg->idxRow, cfg->data);
    	    break;
    	case DFE_FL_DPD_B3:
    	    dfeFl_DpdUpdateBlk3LutInit(hDfeDpd, cfg->idxRow, cfg->data);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}

    	break;
    }
    // update lut init for all rows of one block
    case DFE_FL_DPD_CMD_UPD_BLK_LUT_INIT:
    {
    	DfeFl_DpdBlkLutInit *cfg = (DfeFl_DpdBlkLutInit *)arg;

    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdUpdateBlk0LutInit(hDfeDpd, cfg->LutInit[i].idxRow, cfg->LutInit[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B1:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdUpdateBlk1LutInit(hDfeDpd, cfg->LutInit[i].idxRow, cfg->LutInit[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B2:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdUpdateBlk2LutInit(hDfeDpd, cfg->LutInit[i].idxRow, cfg->LutInit[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B3:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdUpdateBlk3LutInit(hDfeDpd, cfg->LutInit[i].idxRow, cfg->LutInit[i].data);
    		}
    		break;
    	}
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}

    	break;
    }
    // update lut toggle for each row
    case DFE_FL_DPD_CMD_UPD_ROW_LUT_TOGGLE:
    {
    	DfeFl_DpdRowLutToggle *cfg = (DfeFl_DpdRowLutToggle *)arg;

    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		dfeFl_DpdUpdateBlk0LutToggle(hDfeDpd, cfg->idxRow, cfg->data);
    		break;
    	case DFE_FL_DPD_B1:
    		dfeFl_DpdUpdateBlk1LutToggle(hDfeDpd, cfg->idxRow, cfg->data);
    		break;
    	case DFE_FL_DPD_B2:
    	    dfeFl_DpdUpdateBlk2LutToggle(hDfeDpd, cfg->idxRow, cfg->data);
    	    break;
    	case DFE_FL_DPD_B3:
    	    dfeFl_DpdUpdateBlk3LutToggle(hDfeDpd, cfg->idxRow, cfg->data);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}

    	break;

    }
    // update lut toggle for all rows of one block
    case DFE_FL_DPD_CMD_UPD_BLK_LUT_TOGGLE:
    {
    	DfeFl_DpdBlkLutToggle *cfg = (DfeFl_DpdBlkLutToggle *)arg;

    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdUpdateBlk0LutToggle(hDfeDpd, cfg->LutToggle[i].idxRow, cfg->LutToggle[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B1:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdUpdateBlk1LutToggle(hDfeDpd, cfg->LutToggle[i].idxRow, cfg->LutToggle[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B2:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdUpdateBlk2LutToggle(hDfeDpd, cfg->LutToggle[i].idxRow, cfg->LutToggle[i].data);
    		}
    		break;
    	}
    	case DFE_FL_DPD_B3:
    	{
    		for (i = 0; i < cfg->numRows; i++)
    		{
    			dfeFl_DpdUpdateBlk3LutToggle(hDfeDpd, cfg->LutToggle[i].idxRow, cfg->LutToggle[i].data);
    		}
    		break;
    	}
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // config sync selection for each cell
    case DFE_FL_DPD_CMD_UPD_CELL_SYNC:
    {
    	DfeFl_DpdCellSync *cfg = (DfeFl_DpdCellSync *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		err = dfeDpdUpdateBlk0CellSync(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->Ssel);
    		break;
    	case DFE_FL_DPD_B1:
    		err = dfeDpdUpdateBlk1CellSync(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->Ssel);
    		break;
    	case DFE_FL_DPD_B2:
    		err = dfeDpdUpdateBlk2CellSync(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->Ssel);
    	    break;
    	case DFE_FL_DPD_B3:
    		err = dfeDpdUpdateBlk3CellSync(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->Ssel);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // config sync selection for all cells of one row
    case DFE_FL_DPD_CMD_UPD_ROW_SYNC:
    {
    	DfeFl_DpdRowSync *cfg = (DfeFl_DpdRowSync *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i < cfg->numCells; i++)
    			err = dfeDpdUpdateBlk0CellSync(hDfeDpd, cfg->idxRow, cfg->CellSync[i].idxCell, cfg->CellSync[i].data);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i < cfg->numCells; i++)
    			err = dfeDpdUpdateBlk1CellSync(hDfeDpd, cfg->idxRow, cfg->CellSync[i].idxCell, cfg->CellSync[i].data);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i < cfg->numCells; i++)
    			err = dfeDpdUpdateBlk2CellSync(hDfeDpd, cfg->idxRow, cfg->CellSync[i].idxCell, cfg->CellSync[i].data);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i < cfg->numCells; i++)
    			err = dfeDpdUpdateBlk3CellSync(hDfeDpd, cfg->idxRow, cfg->CellSync[i].idxCell, cfg->CellSync[i].data);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // config sync selection for all cells of one block
    case DFE_FL_DPD_CMD_UPD_BLK_SYNC:
    {
    	DfeFl_DpdBlkSync *cfg = (DfeFl_DpdBlkSync *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    				err = dfeDpdUpdateBlk0CellSync(hDfeDpd, cfg->CellSync[i][j].idxRow, cfg->CellSync[i][j].idxCell, cfg->CellSync[i][j].data);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    				err = dfeDpdUpdateBlk1CellSync(hDfeDpd, cfg->CellSync[i][j].idxRow, cfg->CellSync[i][j].idxCell, cfg->CellSync[i][j].data);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    				err = dfeDpdUpdateBlk2CellSync(hDfeDpd, cfg->CellSync[i][j].idxRow, cfg->CellSync[i][j].idxCell, cfg->CellSync[i][j].data);
    		break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i < cfg->numRows; i++)
    			for (j = 0; j < DFE_FL_DPD_NCEL; j++)
    				err = dfeDpdUpdateBlk3CellSync(hDfeDpd, cfg->CellSync[i][j].idxRow, cfg->CellSync[i][j].idxCell, cfg->CellSync[i][j].data);
    		break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // update lut value for each entry
    case DFE_FL_DPD_CMD_UPD_ENTRY_LUT:
    {
    	DfeFl_DpdEntryLUT *cfg = (DfeFl_DpdEntryLUT *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		err = dfeDpdUpdateBlk0LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->idxEntry, cfg->lutGain, cfg->lutSlope);
    		break;
    	case DFE_FL_DPD_B1:
    		err = dfeDpdUpdateBlk1LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->idxEntry, cfg->lutGain, cfg->lutSlope);
    		break;
    	case DFE_FL_DPD_B2:
    		err = dfeDpdUpdateBlk2LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->idxEntry, cfg->lutGain, cfg->lutSlope);
    	    break;
    	case DFE_FL_DPD_B3:
    		err = dfeDpdUpdateBlk3LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->idxEntry, cfg->lutGain, cfg->lutSlope);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // update lut value for each cell
    case DFE_FL_DPD_CMD_UPD_CELL_LUT:
    {
    	DfeFl_DpdCellLUT *cfg = (DfeFl_DpdCellLUT *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i<cfg->numEntries; i++)
    			err = dfeDpdUpdateBlk0LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->LUTval[i].idxEntry, cfg->LUTval[i].lutGain, cfg->LUTval[i].lutSlope);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i<cfg->numEntries; i++)
    			err = dfeDpdUpdateBlk1LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->LUTval[i].idxEntry, cfg->LUTval[i].lutGain, cfg->LUTval[i].lutSlope);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i<cfg->numEntries; i++)
    			err = dfeDpdUpdateBlk2LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->LUTval[i].idxEntry, cfg->LUTval[i].lutGain, cfg->LUTval[i].lutSlope);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i<cfg->numEntries; i++)
    			err = dfeDpdUpdateBlk3LUT(hDfeDpd, cfg->idxRow, cfg->idxCell, cfg->LUTval[i].idxEntry, cfg->LUTval[i].lutGain, cfg->LUTval[i].lutSlope);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // update lut value for each row
    case DFE_FL_DPD_CMD_UPD_ROW_LUT:
    {
    	DfeFl_DpdRowLUT *cfg = (DfeFl_DpdRowLUT *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i<cfg->numCells; i++)
    			for (j = 0; j<DFE_FL_DPD_MAX_LUT_SIZE; j++)
    				err = dfeDpdUpdateBlk0LUT(hDfeDpd, cfg->idxRow, cfg->LUTval[i][j].idxCell, cfg->LUTval[i][j].idxEntry, cfg->LUTval[i][j].lutGain, cfg->LUTval[i][j].lutSlope);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i<cfg->numCells; i++)
    			for (j = 0; j<DFE_FL_DPD_MAX_LUT_SIZE; j++)
    				err = dfeDpdUpdateBlk1LUT(hDfeDpd, cfg->idxRow, cfg->LUTval[i][j].idxCell, cfg->LUTval[i][j].idxEntry, cfg->LUTval[i][j].lutGain, cfg->LUTval[i][j].lutSlope);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i<cfg->numCells; i++)
    			for (j = 0; j<DFE_FL_DPD_MAX_LUT_SIZE; j++)
    				err = dfeDpdUpdateBlk2LUT(hDfeDpd, cfg->idxRow, cfg->LUTval[i][j].idxCell, cfg->LUTval[i][j].idxEntry, cfg->LUTval[i][j].lutGain, cfg->LUTval[i][j].lutSlope);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i<cfg->numCells; i++)
    			for (j = 0; j<DFE_FL_DPD_MAX_LUT_SIZE; j++)
    				err = dfeDpdUpdateBlk3LUT(hDfeDpd, cfg->idxRow, cfg->LUTval[i][j].idxCell, cfg->LUTval[i][j].idxEntry, cfg->LUTval[i][j].lutGain, cfg->LUTval[i][j].lutSlope);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    // update lut value for each block
    case DFE_FL_DPD_CMD_UPD_BLK_LUT:
    {
    	DfeFl_DpdBlkLUT *cfg = (DfeFl_DpdBlkLUT *)arg;
    	switch (cfg->idxBlk)
    	{
    	case DFE_FL_DPD_B0:
    		for (i = 0; i<cfg->numRows; i++)
    			for (j = 0; j<DFE_FL_DPD_NCEL; j++)
    				for (k = 0; k<DFE_FL_DPD_MAX_LUT_SIZE; k++)
    					err = dfeDpdUpdateBlk0LUT(hDfeDpd, cfg->LUTval[i][j][k].idxRow, cfg->LUTval[i][j][k].idxCell, \
    							cfg->LUTval[i][j][k].idxEntry, cfg->LUTval[i][j][k].lutGain, cfg->LUTval[i][j][k].lutSlope);
    		break;
    	case DFE_FL_DPD_B1:
    		for (i = 0; i<cfg->numRows; i++)
    			for (j = 0; j<DFE_FL_DPD_NCEL; j++)
    				for (k = 0; k<DFE_FL_DPD_MAX_LUT_SIZE; k++)
    					err = dfeDpdUpdateBlk1LUT(hDfeDpd, cfg->LUTval[i][j][k].idxRow, cfg->LUTval[i][j][k].idxCell, \
    							cfg->LUTval[i][j][k].idxEntry, cfg->LUTval[i][j][k].lutGain, cfg->LUTval[i][j][k].lutSlope);
    		break;
    	case DFE_FL_DPD_B2:
    		for (i = 0; i<cfg->numRows; i++)
    			for (j = 0; j<DFE_FL_DPD_NCEL; j++)
    				for (k = 0; k<DFE_FL_DPD_MAX_LUT_SIZE; k++)
    					err = dfeDpdUpdateBlk2LUT(hDfeDpd, cfg->LUTval[i][j][k].idxRow, cfg->LUTval[i][j][k].idxCell, \
    							cfg->LUTval[i][j][k].idxEntry, cfg->LUTval[i][j][k].lutGain, cfg->LUTval[i][j][k].lutSlope);
    	    break;
    	case DFE_FL_DPD_B3:
    		for (i = 0; i<cfg->numRows; i++)
    			for (j = 0; j<DFE_FL_DPD_NCEL; j++)
    				for (k = 0; k<DFE_FL_DPD_MAX_LUT_SIZE; k++)
    					err = dfeDpdUpdateBlk3LUT(hDfeDpd, cfg->LUTval[i][j][k].idxRow, cfg->LUTval[i][j][k].idxCell, \
    							cfg->LUTval[i][j][k].idxEntry, cfg->LUTval[i][j][k].lutGain, cfg->LUTval[i][j][k].lutSlope);
    	    break;
    	default:
    		err = DFE_FL_INVPARAMS;
    		break;
    	}
    	break;
    }
    
    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}

/** ============================================================================
 *   @n@b dfeDpdUpdateBlk0CellSync
 *
 *   @b Description
 *   @n Set the sync value of each row of Dpd block0
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid handle of Dpd
         idxRow     The index of row in Dpd block0
         idxCell    The index of cell in each Dpd row of Dpd block0
         Ssel       The sync selection
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
static DfeFl_Status  dfeDpdUpdateBlk0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t Ssel)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdUpdateBlk0Row0CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdUpdateBlk0Row1CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdUpdateBlk0Row2CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdUpdateBlk0Row3CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdUpdateBlk0Row4CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdUpdateBlk0Row5CellSync(hDfeDpd, idxCell, Ssel);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdUpdateBlk1CellSync
 *
 *   @b Description
 *   @n Set the sync value of each row of Dpd block1
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid handle of Dpd
         idxRow     The index of row in Dpd block1
         idxCell    The index of cell in each Dpd row of Dpd block1
         Ssel       The sync selection
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
static DfeFl_Status  dfeDpdUpdateBlk1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t Ssel)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdUpdateBlk1Row0CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdUpdateBlk1Row1CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdUpdateBlk1Row2CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdUpdateBlk1Row3CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdUpdateBlk1Row4CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdUpdateBlk1Row5CellSync(hDfeDpd, idxCell, Ssel);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdUpdateBlk2CellSync
 *
 *   @b Description
 *   @n Set the sync value of each row of Dpd block2
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid handle of Dpd
         idxRow     The index of row in Dpd block2
         idxCell    The index of cell in each Dpd row of Dpd block2
         Ssel       The sync selection
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
static DfeFl_Status  dfeDpdUpdateBlk2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t Ssel)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdUpdateBlk2Row0CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdUpdateBlk2Row1CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdUpdateBlk2Row2CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdUpdateBlk2Row3CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdUpdateBlk2Row4CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdUpdateBlk2Row5CellSync(hDfeDpd, idxCell, Ssel);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdUpdateBlk3CellSync
 *
 *   @b Description
 *   @n Set the sync value of each row of Dpd block3
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid handle of Dpd
         idxRow     The index of row in Dpd block3
         idxCell    The index of cell in each Dpd row of Dpd block3
         Ssel       The sync selection
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
static DfeFl_Status  dfeDpdUpdateBlk3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t Ssel)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdUpdateBlk3Row0CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdUpdateBlk3Row1CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdUpdateBlk3Row2CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdUpdateBlk3Row3CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdUpdateBlk3Row4CellSync(hDfeDpd, idxCell, Ssel);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdUpdateBlk3Row5CellSync(hDfeDpd, idxCell, Ssel);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdUpdateBlk0LUT
 *
 *   @b Description
 *   @n Set the Lut value of Dpd block0.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid Dpd handle
         idxRow     The index of each row in Dpd block0.
         idxCell    The index of each cell in each Dpd row of Dpd block0.
         idxEntry   The index of each entry in each Dpd lut.
         lutGain    The gain value of each lut entry.
         lutSlope   The slope value of each lut entry.
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
static DfeFl_Status  dfeDpdUpdateBlk0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdUpdateBlk0Row0LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdUpdateBlk0Row1LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdUpdateBlk0Row2LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdUpdateBlk0Row3LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdUpdateBlk0Row4LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdUpdateBlk0Row5LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdUpdateBlk1LUT
 *
 *   @b Description
 *   @n Set the Lut value of Dpd block1.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid Dpd handle
         idxRow     The index of each row in Dpd block1.
         idxCell    The index of each cell in each Dpd row of Dpd block1.
         idxEntry   The index of each entry in each Dpd lut.
         lutGain    The gain value of each lut entry.
         lutSlope   The slope value of each lut entry.
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
static DfeFl_Status  dfeDpdUpdateBlk1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdUpdateBlk1Row0LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdUpdateBlk1Row1LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdUpdateBlk1Row2LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdUpdateBlk1Row3LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdUpdateBlk1Row4LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdUpdateBlk1Row5LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdUpdateBlk2LUT
 *
 *   @b Description
 *   @n Set the Lut value of Dpd block2.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid Dpd handle
         idxRow     The index of each row in Dpd block2.
         idxCell    The index of each cell in each Dpd row of Dpd block2.
         idxEntry   The index of each entry in each Dpd lut.
         lutGain    The gain value of each lut entry.
         lutSlope   The slope value of each lut entry.
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
static DfeFl_Status  dfeDpdUpdateBlk2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdUpdateBlk2Row0LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdUpdateBlk2Row1LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdUpdateBlk2Row2LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdUpdateBlk2Row3LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdUpdateBlk2Row4LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdUpdateBlk2Row5LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}

/** ============================================================================
 *   @n@b dfeDpdUpdateBlk3LUT
 *
 *   @b Description
 *   @n Set the Lut value of Dpd block3.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    Valid Dpd handle
         idxRow     The index of each row in Dpd block3.
         idxCell    The index of each cell in each Dpd row of Dpd block3.
         idxEntry   The index of each entry in each Dpd lut.
         lutGain    The gain value of each lut entry.
         lutSlope   The slope value of each lut entry.
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
static DfeFl_Status  dfeDpdUpdateBlk3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxRow,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	DfeFl_Status err = DFE_FL_SOK;

	switch (idxRow)
	{
	case DFE_FL_DPD_R0:
		dfeFl_DpdUpdateBlk3Row0LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R1:
		dfeFl_DpdUpdateBlk3Row1LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R2:
		dfeFl_DpdUpdateBlk3Row2LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R3:
		dfeFl_DpdUpdateBlk3Row3LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R4:
		dfeFl_DpdUpdateBlk3Row4LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	case DFE_FL_DPD_R5:
		dfeFl_DpdUpdateBlk3Row5LUT(hDfeDpd, idxCell, idxEntry, lutGain, lutSlope);
		break;
	default:
		err = DFE_FL_INVPARAMS;
		break;
	}
	return err;
}
