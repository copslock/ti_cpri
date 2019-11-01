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
 * @defgroup DFE_LLD_DDUC_FUNCTION DDUC
 * @ingroup DFE_LLD_FUNCTION
 */

/**
 * @brief Program DDUC Mixer NCO Frequency
 * @ingroup DFE_LLD_DDUC_FUNCTION
 *
 * Write new DDUC Mixer NCO to shadow memory, this is only in the static frequency mode. The precision of the frequency is refClock/2^48. 
 * NOTE, Dfe_issueSyncUpdateDducMixerNCO () should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dducDev	[in] Dduc Id, 0 ~ 3
 *  @param refClock	[in] reference sample rate
 *  @param freq	[in] array of frequency value
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
 *  - Dfe_issueSyncUpdateDducMixerNCO () should be called later to copy gains to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progDducMixerNCO
(
    DFE_Handle hDfe,
	uint32_t dducDev,
	float refClock,
    float freq[12]
)
{
	DfeFl_Status status;
	uint32_t SyncDelay, cic_ndata, carriers_num;
	uint32_t i, ichan, rchan, idx;
	int64_t numi64, deni64;
	uint64_t freq_eng[12];
	double numdbl, dendbl, temp, temp1;
	DfeFl_DducCicCfg DducCicCfg;
	DfeFl_DducHopFrqwordConfig DducMixFreq;

	if(hDfe == NULL)
	{
		Dfe_osalLog("hDfe is NULL!");
		return DFE_ERR_INVALID_HANDLE;
	}

	if(dducDev > 3)
	{
		Dfe_osalLog("Invalid parameter dducDev!");
		return DFE_ERR_INVALID_PARAMS;
	}

	// need to read/write sync_delay (0x12BC) to make sure NCO frequency been updated.
	CSL_HW_QUERY( dfeFl_DducGetHwStatus(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_QUERY_SYNC_DELAY, &SyncDelay) );
	if(SyncDelay == 0)
	{
		Dfe_osalLog("Unsupport: sync delay is zero!");
		return DFE_ERR_DDUC_MIXER_NCO;

	}

	// need to check cic_ndata (0x126C) to load properly for multiple carriers
	CSL_HW_QUERY( dfeFl_DducGetHwStatus(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_QUERY_CIC_CONFIG, &DducCicCfg) );

	cic_ndata = (DducCicCfg.cic_ndata+1)>>1;
	carriers_num = 12;

	if((cic_ndata != 1) && (cic_ndata != 2) && (cic_ndata != 4))
	{
		Dfe_osalLog("Invalid cic_ndata!");
		return DFE_ERR_DDUC_MIXER_NCO;
	}
	else if(carriers_num > cic_ndata*3)
	{
//		Dfe_osalLog("Only the first %d freq is used due to limited CIC!", cic_ndata*3);
		carriers_num = cic_ndata*3;
	}

	// frequency conversion
	// freq (in decimal) = 2^48 x Tuning Frequency / Fclk
	for(i = 0; i < carriers_num; i++)
	{
		numi64 = freq[i]*1000000;
		deni64 = refClock*1000000;
		numdbl = numi64;
		dendbl = deni64;
		temp = numdbl/dendbl;
		temp1 = (temp<0)?temp+1:temp;
		temp1 *= 281474976710656.0;
		freq_eng[i] = temp1 + 0.5;
	}

 	// change mix freq
	for(i = 0; i < 12; i++)
	{
		DducMixFreq.frqWord[i].low = 0;
		DducMixFreq.frqWord[i].mid = 0;
		DducMixFreq.frqWord[i].high = 0;
	}

	if(cic_ndata == 1)
	{
		// only 3*1 = 3 channels
		for(i = 0; i < carriers_num; i++)
		{
			idx = i*4;
			DducMixFreq.frqWord[idx].low = FREQLOW(freq_eng[i]);
			DducMixFreq.frqWord[idx].mid = FREQMID(freq_eng[i]);
			DducMixFreq.frqWord[idx].high = FREQHIGH(freq_eng[i]);
		}
	}
	else if(cic_ndata == 2)
	{
		// only 3*2 = 6 channels
		for(i = 0; i < carriers_num; i++)
		{
			ichan = i>>1;
			rchan = i-ichan*2;
			idx = ichan*4+1-rchan;
			DducMixFreq.frqWord[idx].low = FREQLOW(freq_eng[i]);
			DducMixFreq.frqWord[idx].mid = FREQMID(freq_eng[i]);
			DducMixFreq.frqWord[idx].high = FREQHIGH(freq_eng[i]);
		}
	}
	else if(cic_ndata == 4)
	{
		// only 3*4 = 12 channels
		for(i = 0; i < carriers_num; i++)
		{
			ichan = i>>2;
			rchan = i-ichan*4;
			idx = ichan*4+3-rchan;
			DducMixFreq.frqWord[idx].low = FREQLOW(freq_eng[i]);
			DducMixFreq.frqWord[idx].mid = FREQMID(freq_eng[i]);
			DducMixFreq.frqWord[idx].high = FREQHIGH(freq_eng[i]);
		}
	}

	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_CFG_HOP_FRQWORD, &DducMixFreq) );


	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update DDUC Mixer NCO Frequency
 * @ingroup DFE_LLD_DDUC_FUNCTION
 *
 * Issue sync to copy DDUC Mixer NCO frequency from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dducDev	[in] Dduc Id, 0 ~ 3
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
DFE_Err Dfe_issueSyncUpdateDducMixerNCO
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_DducMixNcoSsel DducMixNcoSsel;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(dducDev > 3)
	{
		Dfe_osalLog("Invalid parameter dducDev!");
		return DFE_ERR_INVALID_PARAMS;
	}

	// set sync
	DducMixNcoSsel.iMix = DFE_FL_DDUC_MIX_NCO_ALL;
	DducMixNcoSsel.data = ssel;
	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL, &DducMixNcoSsel) );

	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_SET_HOP_SSEL, &ssel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);

}

/**
 * @brief Program DDUC Mixer Phase
 * @ingroup DFE_LLD_DDUC_FUNCTION
 *
 * Write new DDUC Mixer phase to shadow memory. The precision for the phase is 360/65536 degree.
 * NOTE, Dfe_issueSyncUpdateDducMixerPhase () should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dducDev	[in] Dduc Id, 0 ~ 3
 *  @param phase	[in] array of new phase, in degree
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
 *  - Dfe_issueSyncUpdateDducMixerPhase () should be called later to copy gains to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progDducMixerPhase
(
    DFE_Handle hDfe,
    uint32_t  dducDev,
    float  phase[12]
)
{
	DfeFl_Status status;
	DfeFl_DducMixNcoPhase DducMixPhase;
	uint32_t i, temp;

	if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(dducDev > 3)
	{
		Dfe_osalLog("Invalid parameter dducDev!");
		return DFE_ERR_INVALID_PARAMS;
	}

	// phase equals 360 degrees * value/65536
	for(i = 0; i < 12; i++)
	{
		DducMixPhase.iMix = (DfeFl_DducMixNco) i;
		temp = phase[i]/360.0*65536;
		DducMixPhase.data = temp&0xFFFF;
		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_SET_MIX_PHASE, &DducMixPhase) );
	}

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update DDUC Mixer Phase
 * @ingroup DFE_LLD_DDUC_FUNCTION
 *
 * Issue sync to copy DDUC Mixer phase from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dducDev	[in] Dduc Id, 0 ~ 3
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
DFE_Err Dfe_issueSyncUpdateDducMixerPhase
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;

	if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(dducDev > 3)
	{
		Dfe_osalLog("Invalid parameter dducDev!");
		return DFE_ERR_INVALID_PARAMS;
	}

	// set sync
	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_SET_MIX_PHASE_SSEL, &ssel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program DDUC Farrow Phase
 * @ingroup DFE_LLD_DDUC_FUNCTION
 *
 * Write new DDUC Farrow phase to shadow memory. The fifo is from 0 ~ 63. The phase range is -0.5 ~ 0.5.
 * NOTE, Dfe_issueSyncUpdateDducFarrowPhase () should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dducDev	[in] Dduc Id, 0 ~ 3
 *  @param fifo	[in] array of fifo
 *  @param phase	[in] array of new phase
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
 *  - Dfe_issueSyncUpdateDducFarrowPhase () should be called later to copy gains to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progDducFarrowPhase
(
    DFE_Handle hDfe,
    uint32_t  dducDev,
    uint32_t fifo[12],
    float  phase[12]
)
{
	DfeFl_Status status;
	DfeFl_DducFrwPhaseConfig DducFrwPhase;
	float temp;
	uint32_t i, phase_fp;

	if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(dducDev > 3)
	{
		Dfe_osalLog("Invalid parameter dducDev!");
		return DFE_ERR_INVALID_PARAMS;
	}

	// change phase
	for(i = 0; i < 12; i++)
	{
		temp = phase[i]*1048576.0;
		phase_fp = temp;

		DducFrwPhase.phase[i] = (fifo[i]&0x3F)<<20+(phase_fp&0xFFFFF);
		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_CFG_FRW_PHASE, &DducFrwPhase) );
	}

	return DFE_ERR_NONE;

}

/**
 * @brief Issue Sync Update DDUC Farrow Phase
 * @ingroup DFE_LLD_DDUC_FUNCTION
 *
 * Issue sync to copy DDUC farrow phase from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dducDev	[in] Dduc Id, 0 ~ 3
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
DFE_Err Dfe_issueSyncUpdateDducFarrowPhase
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;

	if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(dducDev > 3)
	{
		Dfe_osalLog("Invalid parameter dducDev!");
		return DFE_ERR_INVALID_PARAMS;
	}

	// set sync
	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_SET_FRW_PHASE_SSEL, &ssel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program Distributor Map
 * @ingroup DFE_LLD_DDUC_FUNCTION
 *
 * Write new DDUC distributor map to shadow memory. Each rxSel and chanSel value is associated with one DDUC channel. The first 4 channels are for mixer0, the second 4 channels are for mixer1 and the last 4 channels are for mixer2.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dducDev	[in] Dduc Id, 0 ~ 3
 *  @param rxSel	[in] array of rx selection
 *    - 0: from Rx
 *    - 1: from feedback
 *  @param chanSel	[in] array of chan selection, which channel from selected rx.
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
 *  - Dfe_issueSyncUpdateDducDistMap () should be called later to copy gains to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progDducDistMap
(
    DFE_Handle hDfe,
    uint32_t  dducDev,
    uint32_t  rxSel[12],
    uint32_t  chanSel[12]
)
{
	DfeFl_Status status;
	uint32_t i, iMix, iChan;
	DfeFl_DducMixSel DducMixSel;

	if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(dducDev > 3)
	{
		Dfe_osalLog("Invalid parameter dducDev!");
		return DFE_ERR_INVALID_PARAMS;
	}

	for(i = 0; i < 12; i++)
	{
		if(rxSel[i]>3 || chanSel[i]>3)
		{
			Dfe_osalLog("Invalid parameter!");
			return DFE_ERR_INVALID_PARAMS;
		}
	}

	for(i = 0; i < 12; i++)
	{
		iMix = i>>2;
		iChan = i-iMix*4;
		DducMixSel.iMix = iMix;
		DducMixSel.iChan = iChan;
		DducMixSel.chan_or_rx = DFE_FL_DDUC_SELECTOR_MIX_RX;
		DducMixSel.data = rxSel[i];
		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_SET_MIX_SEL, &DducMixSel) );

		DducMixSel.chan_or_rx = DFE_FL_DDUC_SELECTOR_MIX_CHAN;
		DducMixSel.data = chanSel[i];
		CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_SET_MIX_SEL, &DducMixSel) );
	}

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update Distributor Map
 * @ingroup DFE_LLD_DDUC_FUNCTION
 *
 * Issue sync to copy DDUC distributor map from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param dducDev	[in] Dduc Id, 0 ~ 3
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
DFE_Err Dfe_issueSyncUpdateDducDistMap
(
    DFE_Handle hDfe,
    uint32_t dducDev,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;

	if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(dducDev > 3)
	{
		Dfe_osalLog("Invalid parameter dducDev!");
		return DFE_ERR_INVALID_PARAMS;
	}

	// set sync
	CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[dducDev], DFE_FL_DDUC_CMD_SET_SELECTOR_SSEL, &ssel) );


	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}
