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
#include <math.h>
//#ifndef __GNUC__
//#include <mathf.h>
//#endif

/**
 * @defgroup DFE_LLD_RX_FUNCTION RX
 * @ingroup DFE_LLD_FUNCTION
 */
 
/**
 * @brief Program RX IBPM Global
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Program global settings of RX IBPM. The power meters measure RMS and
 * peak powers at Rx block input.
 *
 * There're total 4 IBPMs, which can be individually configured by
 * Dfe_progRxIbpm() to meter one antenna.
 *
 * IBPM has two reading modes, hardware interrupt mode and software
 * handshake mode. 
 *
 * - In the hardware interrupt mode, the power meters run per the
 *   programmed configuration and set interrupts on a per antenna
 *   basis when a new set of results have been computed and captured
 *   in the read registers.
 *
 *   It is the responsibility of the user to read the results before
 *   the next set of results are computed and captured in the read
 *   registers.
 *
 * - In the software handshake mode, the power meters run per the
 *   programmed configuration. The user makes a request to read a
 *   current set of results and waits until receiving an acknowledge
 *   signal from DFE hardware before reading.
 *
 *   Upon receiving the request DFE completes the current power meter
 *   cycle, stores the results for reading, sets the acknowledge and
 *   halts any further updates until the user resets the request.
 *
 * Each IBPM has two histogram counters to count number of samples
 * whose magnitude is above the specified histThresh. 
 *
 * To convert between raw register values to friendly floatings in dB,
 * LLD needs knowing the raw value of unity magnitude square (unityMagSq),
 * i.e. I^2 + Q^2.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param readMode	[in] meter reading mode 
 *  @param histThread1	[in] threshold in dB for histogram counter1
 *  @param histThread2	[in] threshold in dB for histogram counter2
 *  @param unityMagsq	[in] raw value of unity magnitude square
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
DFE_Err Dfe_progRxIbpmGlobal
(
    DFE_Handle hDfe,
    DfeFl_RxPowmtrReadMode readMode,
    float histThresh1,
    float histThresh2,
    uint64_t unityMagsq
)
{
	DfeFl_Status status;
	DfeFl_RxPowmtrGlobalConfig RxPmGbCfg;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	RxPmGbCfg.readMode = readMode;
	RxPmGbCfg.histThresh1 = (uint32_t) (powf(10.0f, histThresh1*0.1f)*unityMagsq/16.f);
	RxPmGbCfg.histThresh2 = (uint32_t) (powf(10.0f, histThresh2*0.1f)*unityMagsq/16.f);
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CFG_POWMTR_GLOBAL, &RxPmGbCfg) );

	hDfe->rxIbpmUnityMagsq = unityMagsq;

	return DFE_ERR_NONE;
}

/**
 * @brief Program RX IBPM
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Program settings of an individual RX IBPM, which meters RMS and
 * peak power at Rx block input for one antenna. 
 *
 * When a sync event comes, IBPM starts a new meter cycle after
 * syncDelay samples. The cycle completes at integration time of
 * nSamples.
 *
 * When oneShot is enabled(1), IBPM does no more measurement until
 * next sync event.
 *
 * When oneShot is disabled(0), IBPM starts a new measure after
 * interval samples elapsed, and this repeats every interval.
 * The interval shall be no less than integration time + syncDelay,
 * otherwise IBPM never completes.
 *
 * When meterMode is 0, IBPM is disabled; when meterMode is 1, IBPM
 * is running according to configuration of sync delay, integration
 * period and interval.
 *
 * When meter mode is 2, IBPM is running over a gated stream. 
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] power meter device ID
 *  @param oneShot	[in] one shot mode
 *  @param meterMode	[in] meter operational mode
 *    - 0 = off
 *    - 1 = normal mode
 *    - 2 = gated mode
 *  @param syncDelay	[in] delay from sync event, in samples
 *  @param nSamples	[in] integration period, in samples
 *  @param interval	[in] interval period, in samples
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
DFE_Err Dfe_progRxIbpm
(
    DFE_Handle hDfe,
    uint32_t pmId,
    uint32_t oneShot,
    uint32_t meterMode,
    uint32_t syncDelay,
    uint32_t nSamples,
    uint32_t interval
)
{
	DfeFl_Status status;
	DfeFl_RxPowmtrConfig RxPmCfg;
	DfeFl_RxPowmtrGeneric RxPmData;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(pmId > 3)
    {
		Dfe_osalLog("Invalid parameter in program Rx Ibpm!");
		return DFE_ERR_INVALID_PARAMS;
    }

	RxPmCfg.powmtr = pmId;
	RxPmCfg.syncDelay = syncDelay;
	RxPmCfg.nSamples = nSamples;
	RxPmCfg.interval = interval;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CFG_POWMTR, &RxPmCfg) );

	RxPmData.powmtr = pmId;
	RxPmData.data = oneShot;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_ONE_SHOT_MODE, &RxPmData) );
	RxPmData.data = meterMode;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_METER_MODE, &RxPmData) );

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update RX IBPM
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Issue sync update a RX IBPM. Dfe_getSyncStatus() can be called later
 * to check if the sync has come.
 *
 * When sync ssel comes, the power meter starts a new measurement cycle.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] Rx IBPM device Id
 *  @param ssel	[in] sync select
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
DFE_Err Dfe_issueSyncUpdateRxIbpm
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_RxPowmtrGeneric RxPmData;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(pmId > 3)
    {
		Dfe_osalLog("Invalid parameter in issue sync to update Rx Ibpm!");
		return DFE_ERR_INVALID_PARAMS;
    }

	RxPmData.powmtr = pmId;
	RxPmData.data = ssel;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_POWMTR_SSEL, &RxPmData) );

	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Issue RX IBPM Read Request
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * The API is used for the software handshake mode only. In this mode,
 * the user makes a request (via this API) to read a current set of
 * results and waits until receiving an acknowledge signal (via polling
 * Dfe_getRxIbpmReadAck()) from DFE hardware before reading.
 *
 * Upon receiving the request DFE completes the current power meter
 * cycle, stores the results for reading, sets the acknowledge and
 * halts any further updates until the user resets the request
 * (via Dfe_readRxIbpmResult()).
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] Rx IBPM device Id
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
DFE_Err Dfe_issueRxIbpmReadRequest
(
    DFE_Handle hDfe,
    uint32_t pmId
)
{
	DfeFl_Status status;
	DfeFl_RxPowmtrGeneric RxPmData;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(pmId > 3)
    {
		Dfe_osalLog("Invalid parameter in issue Rx Ibpm read request!");
		return DFE_ERR_INVALID_PARAMS;
    }

	RxPmData.powmtr = pmId;
	RxPmData.data = 1;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_POWMTR_HANDSHAKE_READ_REQ, &RxPmData) );

	return DFE_ERR_NONE;
}

/**
 * @brief Get RX IBPM Read Ack
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * The API is used for the software handshake mode only. In this mode,
 * the user makes a request (via Dfe_issueRxIbpmReadRequest()) to read
 * a current set of results and waits until receiving an acknowledge
 * signal (via polling this API) from DFE hardware before reading.
 *
 * Upon receiving the request DFE completes the current power meter
 * cycle, stores the results for reading, sets the acknowledge and
 * halts any further updates until the user resets the request (via
 * Dfe_readRxIbpmResult()).
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] Rx IBPM device Id
 *  @param ackRead	[in] acknowledge status
 *    - -	0, still updating
 *    - -	1, read acknowledged
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
DFE_Err Dfe_getRxIbpmReadAck
(
    DFE_Handle hDfe,
    uint32_t pmId,
    uint32_t *ackRead
)
{
	DfeFl_Status status;
	DfeFl_RxPowmtrGeneric RxPmData;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(pmId > 3)
    {
		Dfe_osalLog("Invalid parameter in issue Rx Ibpm read request!");
		return DFE_ERR_INVALID_PARAMS;
    }

	RxPmData.powmtr = pmId;
	RxPmData.data = 0;

	CSL_HW_QUERY( dfeFl_RxGetHwStatus(hDfe->hDfeRx[0], DFE_FL_RX_QUERY_POWMTR_HANDSHAKE_READ_ACK, &RxPmData) );

	*ackRead = RxPmData.data;

	return DFE_ERR_NONE;
}

/**
 * @brief Read RX IBPM Result
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Read back results of a running RX IBPM and after all the results are
 * retrieved:
 * - for hardware interrupt mode, the API writes done reading flag to DFE.
 * - for software handshake mode, the API clears read request, which was
 *   set by Dfe_IssueRxIbpmReadRequest().
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] Rx IBPM device Id
 *  @param power	    [out] Power value
 *  @param peak	        [out] peak value
 *  @param histCount1	[out] the number of samples which power is greater than the histogram one threshold
 *  @param histCount2	[out] the number of samples which power is greater than the histogram two threshold
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
DFE_Err Dfe_readRxIbpmResult
(
    DFE_Handle hDfe,
    uint32_t pmId,
    float  *power,
    float  *peak,
    uint32_t *histCount1,
    uint32_t *histCount2
)
{
	DfeFl_Status status;
	DfeFl_RxPowmtrConfig RxPmCfg;
	DfeFl_RxPowmtrResult RxPmResult;
	uint64_t max_magsq;
	uint32_t nSamples;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(pmId > 3)
    {
		Dfe_osalLog("Invalid parameter in read Rx Ibpm result!");
		return DFE_ERR_INVALID_PARAMS;
    }

	RxPmCfg.powmtr = pmId;
	CSL_HW_QUERY( dfeFl_RxGetHwStatus(hDfe->hDfeRx[0], DFE_FL_RX_QUERY_GET_POWMTR, &RxPmCfg) );
	nSamples = RxPmCfg.nSamples;

	RxPmResult.powmtr = pmId;
	CSL_HW_QUERY( dfeFl_RxGetHwStatus(hDfe->hDfeRx[0], DFE_FL_RX_QUERY_POWMTR_RESULT, &RxPmResult) );

	max_magsq = hDfe->rxIbpmUnityMagsq;
	*power = (float)(10.f*log10((double)RxPmResult.power / nSamples / max_magsq));
	*peak = (float)(10.f*log10((double)RxPmResult.magsq  / max_magsq));
	*histCount1 = RxPmResult.histcnt1;
	*histCount2 = RxPmResult.histcnt2;

	return DFE_ERR_NONE;
}

// Clear Read Request to Rx Ibpm
/** ============================================================================
 *   @n@b Dfe_clearRxIbpmReadRequest
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfe    [add content]
         pmId    [add content]
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
DFE_Err Dfe_clearRxIbpmReadRequest
(
    DFE_Handle hDfe,
    uint32_t pmId
)
{
	DfeFl_Status status;
	DfeFl_RxPowmtrGeneric RxPmData;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(pmId > 3)
    {
		Dfe_osalLog("Invalid parameter in issue Rx Ibpm read request!");
		return DFE_ERR_INVALID_PARAMS;
    }

	RxPmData.powmtr = pmId;
	RxPmData.data = 0;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_POWMTR_HANDSHAKE_READ_REQ, &RxPmData) );

	return DFE_ERR_NONE;
}

/**
 * @brief Program RX Equalizer
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Write new RX Equalizer to shadow memory. The range of each tap
 * is -1 ~ 0.9998. Eqr bypass needs to be disabled.
 *
 * NOTE, Dfe_issueSyncUpdateRxEqr () should be called later to let
 * hardware copy gains from shadow to working memory.
 *
 * ~~~{.c}
 * typedef struct
 * {
 * 	// ii taps
 * 	float taps_ii[DFE_FL_RX_EQR_LEN];
 * 	// iq taps
 * 	float taps_iq[DFE_FL_RX_EQR_LEN];
 * 	// qi taps
 * 	float taps_qi[DFE_FL_RX_EQR_LEN];
 * 	// qq taps
 * 	float taps_qq[DFE_FL_RX_EQR_LEN];
 * } DFE_RxEqrTaps;
 * ~~~
 *
 *  @param hDfe	[in] DFE device handle
 *  @param rxDev	[in] Rx Id, 0 ~ 3
 *  @param shift	[in] shift value, 0 ~ 3
 *  @param numCoeff	[in] number of coefficients
 *  @param RxEqrTaps	[in] pointer to the Rx Eqr taps
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
 *  - Dfe_issueSyncUpdateRxEqr () should be called later to copy EQ taps to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progRxEqr
(
	DFE_Handle hDfe,
	uint32_t rxDev,
	uint32_t shift,
	uint32_t numCoeff,
	DFE_RxEqrTaps *RxEqrTaps
)
{
	DfeFl_Status status;
	DfeFl_RxEqrTapsConfig RxEqr;
	DfeFl_RxEqrGeneric RxShift;
	uint32_t i;
	short temp;
	float one;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

    if(numCoeff > DFE_FL_RX_EQR_LEN)
    {
		Dfe_osalLog("Invalid parameter in Rx Eqr!");
		return DFE_ERR_INVALID_PARAMS;
    }

	// change Rx Gain
	RxEqr.eqr = rxDev;
	for(i = 0; i < DFE_FL_RX_EQR_LEN; i++)
	{
    	RxEqr.iiTaps[i] = 0;
    	RxEqr.iqTaps[i] = 0;
    	RxEqr.qiTaps[i] = 0;
    	RxEqr.qqTaps[i] = 0;
	}

	if(shift == 2)
		one = 16383.f;
	else if(shift == 3)
		one = 8191.f;
	else
		one = 32767.f;

	for(i = 0; i < numCoeff; i++)
	{
		temp = RxEqrTaps->taps_ii[i]*one + 0.5;
		RxEqr.iiTaps[i] = (uint32_t) temp;
		temp = RxEqrTaps->taps_iq[i]*one + 0.5;
		RxEqr.iqTaps[i] = (uint32_t) temp;
		temp = RxEqrTaps->taps_qi[i]*one + 0.5;
		RxEqr.qiTaps[i] = (uint32_t) temp;
		temp = RxEqrTaps->taps_qq[i]*one + 0.5;
		RxEqr.qqTaps[i] = (uint32_t) temp;
	}
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CFG_EQR_TAPS, &RxEqr) );

	RxShift.eqr = rxDev;
	RxShift.data = shift;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_EQR_SHIFT, &RxShift) );

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update RX Equalizer
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Issue sync to copy Rx equalizer from shadow to working memory.
 * Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param rxDev	[in] Rx Id, 0 ~ 3
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
 *  - Only the FB block which matches with IO mux config will be updated.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_issueSyncUpdateRxEqr
(
    DFE_Handle hDfe,
    uint32_t rxDev,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_RxEqrGeneric RxEqrSsel;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(rxDev > 3)
	{
		Dfe_osalLog("Invalid parameter in issue sync update Rx Eqr!");
		return DFE_ERR_INVALID_PARAMS;
	}

	// set sync
	RxEqrSsel.eqr = rxDev;
	RxEqrSsel.data = ssel;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_EQR_SSEL, &RxEqrSsel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program RX Mixer NCO
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Write new RX Mixer NCO to shadow memory. The precision of the frequency
 * is refClock/2^48. NCO bypass needs to be disabled.
 *
 * NOTE, Dfe_issueSyncUpdateRxMixerNCO () should be called later to let
 * hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param rxDev	[in] Rx Id, 0 ~ 3
 *  @param refClock	[in] reference sample rate
 *  @param freq	[in] frequency value
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
 *  - Dfe_issueSyncUpdateRxMixerNCO () should be called later to copy gains to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progRxMixerNCO
(
    DFE_Handle hDfe,
	uint32_t rxDev,
	float refClock,
    float freq
)
{
	DfeFl_Status status;
	DfeFl_RxNcoFreqWord RxNcoFreq;
	long long numi64, deni64;
	uint64_t freq_eng;
	double numdbl, dendbl, temp, temp1;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(rxDev > 3)
	{
		Dfe_osalLog("Invalid parameter in Rx mixer NCO!");
		return DFE_ERR_INVALID_PARAMS;
	}


	// frequency conversion
	// freq (in decimal) = 2^48 x Tuning Frequency / Fclk
	numi64 = freq*1000000;
	deni64 = refClock*1000000;
	numdbl = numi64;
	dendbl = deni64;
	temp = numdbl/dendbl;
	temp1 = (temp<0)?temp+1:temp;
	temp1 *= 281474976710656.0;
	freq_eng = temp1 + 0.5;

	RxNcoFreq.nco = rxDev;
	RxNcoFreq.freqWord = freq_eng;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_CFG_NCO_FREQ, &RxNcoFreq) );

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update RX Mixer NCO Frequency
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Issue sync to copy RX Mixer NCO frequency from shadow to working memory.
 * Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
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
DFE_Err Dfe_issueSyncUpdateRxMixerNCO
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_RxNcoSsel RxNcoSsel;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	// set sync
	RxNcoSsel.sselDither = DFE_FL_SYNC_GEN_SIG_NEVER;
	RxNcoSsel.sselFreq = ssel;
	RxNcoSsel.sselPhase = DFE_FL_SYNC_GEN_SIG_NEVER;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_NCO_SSEL, &RxNcoSsel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program Rx Testbus
 * @ingroup DFE_LLD_RX_FUNCTION
 *
 * Program RX Test bus. 
 *
 *  @param hDfe	[in] DFE device handle
 *  @param top_ctrl	[in] top testbus control.
 *  @param imb_ctrl	[in] imb testbus control.
 *  @param feagc_dc_ctrl	[in] feagc_dc testbus control.
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
DFE_Err Dfe_progRxTestbus
(
    DFE_Handle hDfe,
	uint32_t top_ctrl,
	uint32_t imb_ctrl,
	uint32_t feagc_dc_ctrl
)
{
	DfeFl_Status status;
	uint32_t RxTestCtrl;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	RxTestCtrl = (DfeFl_RxTestCtrl) top_ctrl;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_TOP_TEST_CTRL, &RxTestCtrl) );
	RxTestCtrl = (DfeFl_RxImbTestCtrl) imb_ctrl;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_IMB_TEST_CTRL, &RxTestCtrl) );
	RxTestCtrl = (DfeFl_RxFeagcDcTestCtrl) feagc_dc_ctrl;
	CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_FEAGC_DC_TEST_CTRL, &RxTestCtrl) );

	return DFE_ERR_NONE;
}
