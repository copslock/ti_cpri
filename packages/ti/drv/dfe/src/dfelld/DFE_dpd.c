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
 * @defgroup DFE_LLD_DPD_FUNCTION DPD
 * @ingroup DFE_LLD_FUNCTION
 */
 
/**
 * @brief Enable Lut toggle
 * @ingroup DFE_LLD_DPD_FUNCTION
 *
 * Enable the Lut toggle for one dpd block
 *
 *  @param hDfe	[in] DFE device handle
 *  @param blkId	[in] block Id [0:3]
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
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
DFE_Err Dfe_enableDpdToggle
(
    DFE_Handle hDfe,
    uint32_t  blkId
)
{    
	DfeFl_Status status;
	DfeFl_DpdBlkLutToggle lutToggle;
	uint32_t j;

    VALID_DFE_HANDLE(hDfe);
    
    if(blkId >= DFE_FL_DPD_NBLK)
    {
        Dfe_osalLog("blkId is not valid");
        return DFE_ERR_INVALID_PARAMS;
    }
    
	lutToggle.idxBlk = blkId;
	lutToggle.numRows = DFE_FL_DPD_NROW;
    for(j = 0; j < DFE_FL_DPD_NROW; j++)
    {
    	lutToggle.LutToggle[j].idxRow = j;
    	lutToggle.LutToggle[j].data = 1;
    }

    if (hDfe->dpdIsDisabled == 0)
    {
        CSL_HW_CTRL(dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_UPD_BLK_LUT_TOGGLE, &lutToggle));
    }
    
    return DFE_ERR_NONE;
}

/**
 * @brief SetSyncSel for Lut
 * @ingroup DFE_LLD_DPD_FUNCTION
 *
 * Set synch selection for one dpd block
 *
 *  @param hDfe	[in] DFE device handle
 *  @param blkId	[in] block Id [0:3]
 *  @param Synch	[in] Synch: 
 *    - 0: f_synch
 *    - 1: c_synch
 *    - 2: f_synch || c_synch
 *    - 3:  the 'combined sync' generated internally based on 'sync_b' and sync from 'poly2LUT'
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
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
DFE_Err Dfe_setDpdSyncSel
(
    DFE_Handle hDfe,
    uint32_t  blkId,
    uint32_t  synch
)
{
	DfeFl_Status status;
	DfeFl_DpdBlkSync blkFCsync;
	uint32_t j, k;

    VALID_DFE_HANDLE(hDfe);

    if(blkId >= DFE_FL_DPD_NBLK)
    {
        Dfe_osalLog("blkId is not valid");
        return DFE_ERR_INVALID_PARAMS;
    }

    if(synch > 3)
    {
        Dfe_osalLog("synch is not valid");
        return DFE_ERR_INVALID_PARAMS;
    }

	blkFCsync.idxBlk = blkId;
    blkFCsync.numRows = DFE_FL_DPD_NROW;
    for(j = 0; j < DFE_FL_DPD_NROW; j++)
    {
    	for(k = 0; k < DFE_FL_DPD_NCEL; k++)
    	{
    		blkFCsync.CellSync[j][k].idxCell = k;
    		blkFCsync.CellSync[j][k].idxRow = j;
    		blkFCsync.CellSync[j][k].data = synch;
    	}
    }
    if (hDfe->dpdIsDisabled == 0)
    {
        CSL_HW_CTRL(dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_UPD_BLK_SYNC, &blkFCsync));
    }

    return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update Lut
 * @ingroup DFE_LLD_DPD_FUNCTION
 *
 * Issue sync to switch Lut table for one block.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param numBlks	[in] number of blocks which will be updated, [1:4]
 *  @param blkId[]	[in] array of block Ids which will be updated, [0:3]
 *  @param ssel	[in] sync select to copy gains from shadow to working memory.
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
DFE_Err Dfe_issueSyncUpdateDpdLut
(
    DFE_Handle hDfe,
    uint32_t numBlks,
    uint32_t blkId[],
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_DpdBlkSsel blkSsel;
	uint32_t i;

    VALID_DFE_HANDLE(hDfe);
    
    if(numBlks > DFE_FL_DPD_NBLK)
    {
        Dfe_osalLog("numBlks is not valid");
        return DFE_ERR_INVALID_PARAMS;
    }

    for(i = 0; i < numBlks; i++)
    {
        if (blkId[i] >= DFE_FL_DPD_NBLK)
        {
            Dfe_osalLog("blkId[%d] is not valid", i);
            return DFE_ERR_INVALID_PARAMS;
        }
        else
        {
        	blkSsel.idxBlk = blkId[i];
        	blkSsel.Ssel = ssel;
            if (hDfe->dpdIsDisabled == 0)
            {
                CSL_HW_CTRL(dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_UPD_BLK_C_SSEL, &blkSsel));
            }
        }
    }
    
    return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Get Current Lut memory index
 * @ingroup DFE_LLD_DPD_FUNCTION
 *
 * Get the current Lut memory index for one dpd block. There are total 6 rows in one dpd block. There are total 3 cells in one row. Each cell will return the Lut memory index. 0 means the datapath is using LUT0 (bottom half of memory), 1 means the datapath is using LUT1 (top half of memory).
 * The LutIdx will store the results in bit masking format.
 *
 *  | MSB       | B23      | B22         | B21         | B20	     | ... | B3       | B2          | B1          | B0          |
 *  | --------- | -------- | ----------- | ----------- | ----------- | -   | -------- | ----------- | ----------- | ----------- |
 * 	|           | Not used | Row5, cell2 | Row5, cell1 | Row5, cell0 | ... | Not used | Row0, cell2 | Row0, cell1 | Row0, cell0 |
 *
 *  @param hDfe	[in] DFE device handle
 *  @param blkId	[in] block Id [0:3]
 *  @param LutIdx	[out] current Lut memory index
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
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
DFE_Err Dfe_getDpdLutIdx
(
    DFE_Handle hDfe,
    uint32_t blkId,
    uint32_t *LutIdx
)
{
	DfeFl_Status status;
	DfeFl_DpdCellCurLutMpu CurLutMpu;
	uint32_t t, j, k;

    VALID_DFE_HANDLE(hDfe);
    if(blkId >= DFE_FL_DPD_NBLK)
    {
        Dfe_osalLog("blkId is not valid");
        return DFE_ERR_INVALID_PARAMS;
    }

    if(LutIdx == NULL)
    {
        Dfe_osalLog("LutIdx pointer is NULL");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    *LutIdx = 0;
	CurLutMpu.idxBlk = blkId;
	for(j = 0; j < DFE_FL_DPD_NROW; j++)
	{
		CurLutMpu.idxRow = j;
		t = j*4;
		for(k = 0; k < DFE_FL_DPD_NCEL; k++)
		{
            if (hDfe->dpdIsDisabled == 0)
            {
                CurLutMpu.idxCell = k;
		        CSL_HW_QUERY(dfeFl_DpdGetHwStatus(hDfe->hDfeDpd[0], DFE_FL_DPD_QUERY_CELL_CURRENT_LUT_MPU, &CurLutMpu));
		        if (CurLutMpu.CurLutMpu == 1)
		            *LutIdx += CurLutMpu.CurLutMpu << (k+t);
            }
		}
	}
    
    return DFE_ERR_NONE;
 }

/**
 * @brief Program Lut table
 * @ingroup DFE_LLD_DPD_FUNCTION
 *
 * Program Lut table for one cell. The dpd lut will be used in dpd datapath after issue sync to update lut.
 * typedef struct _DFE_DpdData
 * {
 * 	// lutGain
 * 	DfeFl_DpdComplexInt lutGain;
 * 	// lutSlope
 * 	DfeFl_DpdComplexInt lutSlope;
 * } DFE_DpdData;
 *
 *  @param hDfe	[in] DFE device handle
 *  @param blkId	[in] block Id [0:3]
 *  @param rowId	[in] row Id [0:5]
 *  @param cellId	[in] cell Id [0:2]
 *  @param DpdData	[in] pointer to the dpd data
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
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
DFE_Err Dfe_progDpdLutTable
(
    DFE_Handle hDfe,
    uint32_t blkId,
    uint32_t rowId,
    uint32_t cellId,
    DFE_DpdData *DpdData
)
{
	DfeFl_Status status;
	DfeFl_DpdEntryLUT entrylut;
	uint32_t ilut;
	
    VALID_DFE_HANDLE(hDfe);
    if(blkId >= DFE_FL_DPD_NBLK)
    {
        Dfe_osalLog("blkId is not valid");
        return DFE_ERR_INVALID_PARAMS;
    }

    if(rowId >= DFE_FL_DPD_NROW)
    {
        Dfe_osalLog("rowId is not valid");
        return DFE_ERR_INVALID_PARAMS;
    }

    if(cellId >= DFE_FL_DPD_NCEL)
    {
        Dfe_osalLog("cellId is not valid");
        return DFE_ERR_INVALID_PARAMS;
    }

	entrylut.idxBlk = blkId;
	entrylut.idxRow = rowId;
	entrylut.idxCell = cellId;
	for(ilut = 0; ilut < DFE_FL_DPD_MAX_LUT_SIZE; ilut++)
	{
		entrylut.idxEntry = ilut;
		entrylut.lutGain.real = DpdData->lutGain.real;
		entrylut.lutGain.imag = DpdData->lutGain.imag;
		entrylut.lutSlope.real = DpdData->lutSlope.real;
		entrylut.lutSlope.imag = DpdData->lutSlope.imag;
		DpdData++;
        if (hDfe->dpdIsDisabled == 0)
        {
            CSL_HW_CTRL(dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_UPD_ENTRY_LUT, &entrylut));
        }
	}


	return DFE_ERR_NONE;
}

/**
 * @brief Get Dpd configuration
 * @ingroup DFE_LLD_DPD_FUNCTION
 *
 * Read back the dpd configuration
 *
 * ~~~{.c}
 * typedef struct _DFE_DpdCfg
 * {
 * 	  // subchip mode
 * 	  uint32_t subchip_mode;
 * 	  // subsample
 * 	  uint32_t subsample;
 * 	  // dpd input scale
 * 	  uint32_t dpdInputScale;
 * 	  // x2 sqrt
 * 	  uint32_t x2_sqrt;
 * } DFE_DpdCfg;
 * ~~~
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dpdCfg	[out] pointer to the dpd configure
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
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
DFE_Err Dfe_getDpdCfg
(
    DFE_Handle hDfe,
    DFE_DpdCfg *dpdCfg
)
{
	DfeFl_Status status;
	DfeFl_DpdMuxSqrt MuxSqrt;
	uint32_t data;


    VALID_DFE_HANDLE(hDfe);

    if (hDfe->dpdIsDisabled == 0)
    {
        // read subchip mode
        CSL_HW_QUERY(dfeFl_DpdGetHwStatus(hDfe->hDfeDpd[0], DFE_FL_DPD_QUERY_SUBCHIP_MODE, &data));
        dpdCfg->subchip_mode = data;
        // read subsample
        CSL_HW_QUERY(dfeFl_DpdGetHwStatus(hDfe->hDfeDpd[0], DFE_FL_DPD_QUERY_SUBSAMPLE, &data));
        dpdCfg->subsample = data;
        // read dpd input scale
        CSL_HW_QUERY(dfeFl_DpdGetHwStatus(hDfe->hDfeDpd[0], DFE_FL_DPD_QUERY_DPDINPUT_SCALE, &data));
        dpdCfg->dpdInputScale = data;

        // read dpd input scale
        MuxSqrt.Mux = DFE_FL_DPD_MUX_SQRT_MAGX2;
        CSL_HW_QUERY(dfeFl_DpdGetHwStatus(hDfe->hDfeDpd[0], DFE_FL_DPD_QUERY_MUX_SQRT, &MuxSqrt));
        dpdCfg->x2_sqrt = MuxSqrt.data;
    }

	return DFE_ERR_NONE;
}
