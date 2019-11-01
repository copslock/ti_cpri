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
#include "math.h"

/**
 * @defgroup DFE_LLD_CFR_FUNCTION CFR
 * @ingroup DFE_LLD_FUNCTION
 */
 
/**
 * @brief Program CFR Coefficients
 * @ingroup DFE_LLD_CFR_FUNCTION
 *
 * Write new Cfr coefficients to the shadow memory. The range of each coefficient
 * is 0 ~ 4095. The maximum number of coefficients is 256 for each Cfr instance.
 * It is shared by all antennas in each Cfr instance.
 *
 * NOTE, Dfe_issueSyncUpdateCfrCoeff () should be called later to let hardware
 * copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrDev	[in] Cfr device Id, 0 - 1
 *  @param numCoeffs	[in] number of coefficients
 *  @param cfrCoeff_i	[in] pointer to the real part coefficient
 *  @param cfrCoeff_q	[in] pointer to the image part coefficient
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
 *  - Dfe_issueSyncUpdateCfrCoeff () should be called later to copy gains
 *    to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progCfrCoeff
(
    DFE_Handle hDfe,
	uint32_t cfrDev,
	uint32_t numCoeffs,
    uint32_t *cfrCoeff_i,
    uint32_t *cfrCoeff_q
)
{
	DfeFl_Status status;
	DfeFl_CfrLutCoeffCfg CfrCoeff;
	DfeFl_CfrBusyStatus CfrStatus;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(cfrDev > 1)
	{
		Dfe_osalLog("Invalid parameter in issue sync for cfr coeff!");
		return DFE_ERR_INVALID_PARAMS;
	}

	if(numCoeffs > 256)
	{
		Dfe_osalLog("Invalid parameter in issue sync for cfr coeff!");
		return DFE_ERR_INVALID_PARAMS;
	}

	CfrStatus.path = DFE_FL_CFR_PATH_ALL;
	CfrStatus.status = 0;
	CSL_HW_CTRL( dfeFl_CfrGetHwStatus(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_QUERY_GET_BUSY_STATUS, &CfrStatus) );
	if(CfrStatus.status != 0)
	{
		Dfe_osalLog("cfr is busy, can not update coeffs!");
		return DFE_ERR_CFR_BUSY;
	}

	CfrCoeff.location = DFE_FL_CFR_PDC01_I;
	CfrCoeff.numCoeff = numCoeffs;
	CfrCoeff.coeff = cfrCoeff_i;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_UPDT_LUT_COEFF, &CfrCoeff) );

	CfrCoeff.location = DFE_FL_CFR_PDC01_Q;
	CfrCoeff.numCoeff = numCoeffs;
	CfrCoeff.coeff = cfrCoeff_q;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_UPDT_LUT_COEFF, &CfrCoeff) );

	return DFE_ERR_NONE;

}

/**
 * @brief Issue Sync Update CFR Coefficients
 * @ingroup DFE_LLD_CFR_FUNCTION
 *
 * Issue sync update Cfr coefficients. Dfe_getSyncStatus() can be
 * called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrDev	[in] Cfr device Id, 0 - 1
 *  @param coeffType	[in] Cfr coefficient type,
 *    - 0 means the base coefficient
 *    - 1 means the half delay coefficient
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
DFE_Err Dfe_issueSyncUpdateCfrCoeff
(
    DFE_Handle hDfe,
	uint32_t cfrDev,
	uint32_t coeffType,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_CfrMultSsel CfrSsel;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(cfrDev > 1 || coeffType > 1)
	{
		Dfe_osalLog("Invalid parameter in issue sync for cfr coeff!");
		return DFE_ERR_INVALID_PARAMS;
	}

	CfrSsel.path = DFE_FL_CFR_PATH_ALL;
	CfrSsel.ssel = ssel;

	if(coeffType == 0)// the base coeff
	{
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_LUT_SSEL, &CfrSsel) );
	}
	else // the half delay coeff
	{
		CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_HDLY_SSEL, &CfrSsel) );
	}

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);

}

/**
 * @brief Program CFR preGain
 * @ingroup DFE_LLD_CFR_FUNCTION
 *
 * Write new Cfr preGain to shadow memory. The range is -Inf ~ 6dB.
 * NOTE, Dfe_issueSyncUpdateCfrPreGain () should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrDev	[in] Cfr device Id, 0 - 1
 *  @param cfrPath	[in] Cfr path Id, 0 - 1
 *  @param gainGain	[in] Cfr pre gain value in dB
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
 *  - Dfe_issueSyncUpdateCfrPreGain() should be called later to let hardware take new configuration.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progCfrPreGain
(
	DFE_Handle hDfe,
	uint32_t cfrDev,
	DfeFl_CfrPath cfrPath,
	float gain
)
{
	DfeFl_Status status;
	DfeFl_CfrMultGain CfrGain;
	unsigned short cfr_gain;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(cfrDev > 1 || cfrPath > 1)
	{
		Dfe_osalLog("Invalid parameter in cfr preGain!");
		return DFE_ERR_INVALID_PARAMS;
	}

	if(gain > 6.0)
	{
		Dfe_osalLog("Invalid parameter in cfr postGain!");
		return DFE_ERR_INVALID_PARAMS;
	}

	cfr_gain =	(unsigned short)((pow(10,(gain/20))) * (float) 0x8000);
	CfrGain.path = cfrPath;
	CfrGain.gain = cfr_gain;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_SET_PREGAIN, &CfrGain) );

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update CFR preGain
 * @ingroup DFE_LLD_CFR_FUNCTION
 *
 * Issue sync to copy CFR pre gain from shadow to working memory.
 * Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrDev	[in] Cfr device Id, 0 - 1
 *  @param cfrPath	[in] Cfr path Id, 0 - 1
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
DFE_Err Dfe_issueSyncUpdatCfrPreGain
(
    DFE_Handle hDfe,
	uint32_t cfrDev,
	DfeFl_CfrPath cfrPath,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_CfrMultSsel CfrSsel;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(cfrDev > 1 || cfrPath > 1)
	{
		Dfe_osalLog("Invalid parameter in issue sync for cfr preGain!");
		return DFE_ERR_INVALID_PARAMS;
	}

	CfrSsel.path = cfrPath;
	CfrSsel.ssel = ssel;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_SET_PREM_SSEL, &CfrSsel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program CFR postGain
 * @ingroup DFE_LLD_CFR_FUNCTION
 *
 * Write new Cfr postGain to shadow memory. The range is -Inf ~ 6dB.
 *
 * NOTE, Dfe_issueSyncUpdateCfrPostGain () should be called later to
 * let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrDev	[in] Cfr device Id, 0 - 1
 *  @param cfrPath	[in] Cfr path Id, 0 - 1
 *  @param gainGain	[in] Cfr post gain value in dB
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
 *  - Dfe_issueSyncUpdateCfrPostGain() should be called later to let
 *    hardware take new configuration.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progCfrPostGain
(
	DFE_Handle hDfe,
	uint32_t cfrDev,
	DfeFl_CfrPath cfrPath,
	float gain
)
{
	DfeFl_Status status;
	DfeFl_CfrMultGain CfrGain;
	unsigned short cfr_gain;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(cfrDev > 1 || cfrPath > 1)
	{
		Dfe_osalLog("Invalid parameter in cfr postGain!");
		return DFE_ERR_INVALID_PARAMS;
	}

	if(gain > 6.0)
	{
		Dfe_osalLog("Invalid parameter in cfr postGain!");
		return DFE_ERR_INVALID_PARAMS;
	}

	cfr_gain =	(unsigned short)((pow(10,(gain/20))) * (float) 0x8000);
	CfrGain.path = cfrPath;
	CfrGain.gain = cfr_gain;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_SET_POSTGAIN, &CfrGain) );

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update CFR postGain
 * @ingroup DFE_LLD_CFR_FUNCTION
 *
 * Issue sync to copy CFR pre gain from shadow to working memory.
 * Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrDev	[in] Cfr device Id, 0 - 1
 *  @param cfrPath	[in] Cfr path Id, 0 - 1
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
DFE_Err Dfe_issueSyncUpdatCfrPostGain
(
    DFE_Handle hDfe,
	uint32_t cfrDev,
	DfeFl_CfrPath cfrPath,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_CfrMultSsel CfrSsel;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(cfrDev > 1 || cfrPath > 1)
	{
		Dfe_osalLog("Invalid parameter in issue sync for cfr postGain!");
		return DFE_ERR_INVALID_PARAMS;
	}

	CfrSsel.path = cfrPath;
	CfrSsel.ssel = ssel;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_SET_POSTM_SSEL, &CfrSsel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program CFR Protection Gain
 * @ingroup DFE_LLD_CFR_FUNCTION
 *
 * Write new Cfr protection gain. It is a backoff gain when PA protection
 * is in alarm mode. The range is -Inf ~ 6dB.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cfrDev	[in] Cfr device Id, 0 - 1
 *  @param cfrPath	[in] Cfr path Id, 0 - 1
 *  @param gainGain	[in] Cfr protection gain value in dB
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
DFE_Err Dfe_progCfrProtGain
(
	DFE_Handle hDfe,
	uint32_t cfrDev,
	DfeFl_CfrPath cfrPath,
	float gain
)
{
	DfeFl_Status status;
	DfeFl_CfrMultGain CfrGain;
	unsigned short cfr_gain;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(cfrDev > 1 || cfrPath > 1)
	{
		Dfe_osalLog("Invalid parameter in cfr protGain!");
		return DFE_ERR_INVALID_PARAMS;
	}

	if(gain > 6.0)
	{
		Dfe_osalLog("Invalid parameter in cfr postGain!");
		return DFE_ERR_INVALID_PARAMS;
	}

	cfr_gain =	(unsigned short)((pow(10,(gain/20))) * (float) 0x8000);
	CfrGain.path = cfrPath;
	CfrGain.gain = cfr_gain;
	CSL_HW_CTRL( dfeFl_CfrHwControl(hDfe->hDfeCfr[cfrDev], DFE_FL_CFR_CMD_SET_PROTPAGAIN, &CfrGain) );

	return DFE_ERR_NONE;
}
