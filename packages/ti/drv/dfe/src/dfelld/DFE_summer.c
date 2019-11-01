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
 * @defgroup DFE_LLD_SUMMER_FUNCTION SUMMER
 * @ingroup DFE_LLD_FUNCTION
 */
 
/**
 * @brief Program Summer Shift
 * @ingroup DFE_LLD_SUMMER_FUNCTION
 *
 * Write new Summer shift. The range of gain is -36dB ~ 6dB, with step 6dB. 
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrId	[in] cfr Id, 0 ~ 1
 *  @param StrId	[in] stream Id, 0 ~ 1
 *  @param gain	[in] new gain, in dB
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
DFE_Err Dfe_progSummerShift
(
    DFE_Handle hDfe,
    uint32_t cfrId,
    uint32_t strId,
    int  gain
)
{
	DfeFl_Status status;
	DfeFl_SummerShiftCfg SumShift;
	int igain, rgain;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(cfrId > 1 || strId > 1)
	{
		Dfe_osalLog("Invalid parameter in summer shift!");
		return DFE_ERR_INVALID_PARAMS;
	}

	igain = gain/6;
	rgain = gain-igain*6;
	if(igain < -6 || igain > 1 || rgain != 0)
	{
		Dfe_osalLog("Invalid parameter in summer shift!");
		return DFE_ERR_INVALID_PARAMS;
	}

	SumShift.idx = (DfeFl_SummerCfrStr) (cfrId*2+strId);
	SumShift.data = igain+6;
	CSL_HW_CTRL( dfeFl_SummerHwControl(hDfe->hDfeSummer[0], DFE_FL_SUMMER_CMD_SET_SHIFT, &SumShift) );

	return DFE_ERR_NONE;
}

/**
 * @brief Program Summer Map
 * @ingroup DFE_LLD_SUMMER_FUNCTION
 *
 * Write new Summer map configuration. 
 * NOTE, Dfe_issueSyncUpdateSummerMap () should be called later to let hardware take the configuration.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrId	[in] Cfr Id, 0 ~ 1
 *  @param strId	[in] stream Id, 0 ~ 1
 *  @param sumMap	[in] summer map for 4 DDUC. Each word uses the 12 LSB bits to map 12 carriers for each DDUC.
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
 *  - Dfe_issueSyncUpdateSummerMap() should be called later to let hardware take new configuration.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progSummerMap
(
	DFE_Handle hDfe,
	uint32_t cfrId,
	uint32_t strId,
    uint32_t sumMap[4]
)
{
	DfeFl_Status status;
	DfeFl_SummerSelCfg SummerSel;
	uint32_t i, j, val;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(cfrId > 1 || strId > 1)
	{
		Dfe_osalLog("Invalid parameter in summer map!");
		return DFE_ERR_INVALID_PARAMS;
	}

	SummerSel.idx = (DfeFl_SummerCfrStr) (cfrId*2+strId);
	for(i = 0; i < 4; i++)
	{
		SummerSel.iDduc = (DfeFl_SummerDduc) i;
		val = sumMap[i];
		for(j = 0; j < 3; j++)
		{
			SummerSel.iPort = (DfeFl_SummerDducPort) j;
			SummerSel.data = val&0xF;
			val = val>>4;
			CSL_HW_CTRL( dfeFl_SummerHwControl(hDfe->hDfeSummer[0], DFE_FL_SUMMER_CMD_SET_SEL, &SummerSel) );
		}
	}
	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update Summer Map
 * @ingroup DFE_LLD_SUMMER_FUNCTION
 *
 * Issue sync to let Summer run with new configuration. Dfe_getSyncStatus() may be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ssel	[in] sync select to update power meter
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
DFE_Err Dfe_issueSyncUpdateSummerMap
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	// set sync
	CSL_HW_CTRL( dfeFl_SummerHwControl(hDfe->hDfeSummer[0], DFE_FL_SUMMER_CMD_SET_SUMMER_SSEL, &ssel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}
