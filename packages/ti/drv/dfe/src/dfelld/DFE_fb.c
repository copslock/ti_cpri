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
#include <math.h>
#include <ti/drv/dfe/dfe_drv.h>
#include <ti/drv/dfe/dfe_osal.h>
#include <ti/drv/dfe/dfe_internal.h>

/**
 * @defgroup DFE_LLD_FB_FUNCTION FB
 * @ingroup DFE_LLD_FUNCTION
 */


/** ============================================================================
 *   @n@b EqTapConvert
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         eqtap    [add content]
         one    [add content]
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
static short EqTapConvert(float eqtap, float one)
{
	short eqtap_fix;

	eqtap_fix = eqtap*one+0.5;

	return eqtap_fix;
}

/**
 * @brief Program FB Equalizer
 * @ingroup DFE_LLD_FB_FUNCTION
 *
 * Write new FB Equalizer to shadow memory. The range of each tap is -1 ~ 0.9998.
 * NOTE, Dfe_issueSyncUpdateFbEqr should be called later to let hardware copy gains from shadow to working memory.
 * typedef struct
 * {
 * 	// ii taps
 * 	float taps_ii[DFE_FL_FB_EQR_LEN];
 * 	// iq taps
 * 	float taps_iq[DFE_FL_FB_EQR_LEN];
 * 	// qi taps
 * 	float taps_qi[DFE_FL_FB_EQR_LEN];
 * 	// qq taps
 * 	float taps_qq[DFE_FL_FB_EQR_LEN];
 * } DFE_FbEqrTaps;
 *
 *  @param hDfe	[in] DFE device handle
 *  @param FbBlkId	[in] Fb block Id, 0 ~ 4
 *  @param numCoeff	[in] number of coefficients
 *  @param FbEqrTaps	[in] pointer to the Fb Eqr taps.
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
 *  - Dfe_issueSyncUpdateFbEqr () should be called later to copy EQ taps to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progFbEqr
(
	DFE_Handle hDfe,
	DfeFl_FbBlk FbBlkId,
	uint32_t numCoeff,
    DFE_FbEqrTaps *FbEqrTaps
)
{
	DfeFl_Status status;
	DfeFl_FbEqrTaps FbEqr;
	uint32_t i, idx;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(FbBlkId  > 4)
    {
		Dfe_osalLog("Invalid parameter in Fb Eqr!");
		return DFE_ERR_INVALID_PARAMS;
    }

    if(numCoeff > DFE_FL_FB_EQR_LEN)
    {
		Dfe_osalLog("Invalid parameter in Fb Eqr!");
		return DFE_ERR_INVALID_PARAMS;
    }

    FbEqr.blk = FbBlkId;
    FbEqr.numCoeff = numCoeff;
    for(i = 0; i < DFE_FL_FB_EQR_TAPS; i++)
    {
    	FbEqr.taps_ii[i] = 0;
    	FbEqr.taps_iq[i] = 0;
    	FbEqr.taps_qi[i] = 0;
    	FbEqr.taps_qq[i] = 0;
    }

    for(i = 0; i < numCoeff; i+=2)
    {
    	idx = i>>1;
    	FbEqr.taps_ii[idx] = EqTapConvert(FbEqrTaps->taps_ii[i+1], 8192.0);
    	FbEqr.taps_ii[idx] = (FbEqr.taps_ii[idx]<<16) + EqTapConvert(FbEqrTaps->taps_ii[i], 8192.0);

    	FbEqr.taps_iq[idx] = EqTapConvert(FbEqrTaps->taps_iq[i+1], 8192.0);
    	FbEqr.taps_iq[idx] = (FbEqr.taps_iq[idx]<<16) + EqTapConvert(FbEqrTaps->taps_iq[i], 8192.0);

    	FbEqr.taps_qi[idx] = EqTapConvert(FbEqrTaps->taps_qi[i+1], 8192.0);
    	FbEqr.taps_qi[idx] = (FbEqr.taps_qi[idx]<<16) + EqTapConvert(FbEqrTaps->taps_qi[i], 8192.0);

    	FbEqr.taps_qq[idx] = EqTapConvert(FbEqrTaps->taps_qq[i+1], 8192.0);
    	FbEqr.taps_qq[idx] = (FbEqr.taps_qq[idx]<<16) + EqTapConvert(FbEqrTaps->taps_qq[i], 8192.0);

    }

    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_EQR_TAPS, &FbEqr) );
	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update FB Equalizer
 * @ingroup DFE_LLD_FB_FUNCTION
 *
 * Issue sync to copy Fb equalizer from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
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
DFE_Err Dfe_issueSyncUpdateFbEqr
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
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_EQR_SSEL, &ssel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program FB Mixer NCO
 * @ingroup DFE_LLD_FB_FUNCTION
 *
 * Write new FB Mixer NCO frequency to shadow memory. The precision is refClock/2^48.
 * NOTE, Dfe_issueSyncUpdateFbMixerNCO should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param FbBlkId	[in] Fb block Id, 0~4
 *  @param refClock	[in] Fb reference clock
 *  @param freq	[in] frequence value in degree
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
 *  - Dfe_issueSyncUpdateFbMixerNCO () should be called later to copy NCO to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progFbMixerNCO
(
	DFE_Handle hDfe,
	DfeFl_FbBlk FbBlkId,
	float refClock,
    float freq
)
{
	DfeFl_Status status;
	DfeFl_FbNcoFreq FbNcoFreq;
	int64_t numi64, deni64;
	uint64_t freq_eng;
	double numdbl, dendbl, temp, temp1;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(FbBlkId  > 4)
    {
		Dfe_osalLog("Invalid parameter in Fb Mixer NCO!");
		return DFE_ERR_INVALID_PARAMS;
    }

	// change Fb Mix freq
	// freq (in decimal) = 2^48 x Tuning Frequency / Fclk
	numi64 = freq*1000000;
	deni64 = refClock*1000000;
	numdbl = numi64;
	dendbl = deni64;
	temp = numdbl/dendbl;
	temp1 = (temp<0)?temp+1:temp;
	temp1 *= 281474976710656.0;
	freq_eng = temp1 + 0.5;

	FbNcoFreq.blk = FbBlkId;
	FbNcoFreq.freq_word_w0 = FREQLOW(freq_eng);
	FbNcoFreq.freq_word_w1 = FREQMID(freq_eng);
	FbNcoFreq.freq_word_w2 = FREQHIGH(freq_eng);

	CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_NCO_FREQ, &FbNcoFreq) );
	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update FB Mixer NCO
 * @ingroup DFE_LLD_FB_FUNCTION
 *
 * Issue sync to copy Fb mixer NCO from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
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
DFE_Err Dfe_issueSyncUpdateFbMixerNCO
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_FbNcoSsel FbNcoSsel;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	// set sync
	FbNcoSsel.dither = DFE_FL_SYNC_GEN_SIG_NEVER;
	FbNcoSsel.freq = ssel;
	FbNcoSsel.phase = DFE_FL_SYNC_GEN_SIG_NEVER;
	CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_NCO_SSEL, &FbNcoSsel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program FB IO Mux
 * @ingroup DFE_LLD_FB_FUNCTION
 *
 * Write new FB IO mux. 
 * NOTE, Additional sync needs to be issued to set new values for EQ, gain or NCO in the new Fb channel.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ioMux	[in] io mux mapping.
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
DFE_Err Dfe_progFbIOMux
(
	DFE_Handle hDfe,
    DfeFl_FbIoMux ioMux
)
{
	DfeFl_Status status;
	DfeFl_FbIOCtrl FbIoCtrl = {0, 0, 0};

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    FbIoCtrl.cb_output_select = 0; // not used here
    FbIoCtrl.host_cfg_select = (uint32_t)ioMux >> 1;
    FbIoCtrl.cfg_select_mode = (uint32_t)ioMux & 1u;

    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_IO_MUX, &FbIoCtrl) );

    return DFE_ERR_NONE;
}

/**
 * @brief Program FB pre-CB Gain
 * @ingroup DFE_LLD_FB_FUNCTION
 *
 * Write new FB pre-CB gain to shadow memory. The range of gain is -Inf ~ 6.02dB. To check the gain changing through capture buffer, the cb_output_select need to be enabled.
 * NOTE, Dfe_issueSyncUpdateFbGain should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param FbBlkId	[in] Fb block Id, 0 ~ 4
 *  @param FbGain	[in] Array of Fb gain, [0] is real part and [1] is image part.
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
 *  - Dfe_issueSyncUpdateFbGain () should be called later to copy gains to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progFbGain
(
	DFE_Handle hDfe,
	DfeFl_FbBlk FbBlkId,
    float FbGain[2]
)
{
	DfeFl_Status status;
	DfeFl_FbGainVal FbGainVal;
//	DfeFl_FbIOCtrl FbIoCtrl;
	unsigned short fb_gain;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(FbBlkId  > 4)
    {
		Dfe_osalLog("Invalid parameter in Fb Gain!");
		return DFE_ERR_INVALID_PARAMS;
    }

//	// check the FB IO Control setup
//	CSL_HW_CTRL( dfeFl_FbGetHwStatus(hDfe->hDfeFb[0], DFE_FL_FB_QUERY_GET_IO_CONTROL, &FbIoCtrl) );
//	if (FbIoCtrl.cb_output_select == 0)
//	{
//		Dfe_osalLog("Fb cb output select is 0!");
//		return DFE_ERR_FB_PRECB_GAIN;
//	}

    FbGainVal.blk = FbBlkId;
    fb_gain = (unsigned short)((pow(10,(FbGain[0]/20))) * (float) 0x2000);
    FbGainVal.real = fb_gain;
    fb_gain = (unsigned short)((pow(10,(FbGain[1]/20))) * (float) 0x2000);
    FbGainVal.imag = fb_gain;
    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_GAIN_VAL, &FbGainVal) );

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update Fb pre-CB Gain
 * @ingroup DFE_LLD_FB_FUNCTION
 *
 * Issue sync to copy Fb gains from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
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
DFE_Err Dfe_issueSyncUpdateFbGain
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

    CSL_HW_CTRL( dfeFl_FbHwControl(hDfe->hDfeFb[0], DFE_FL_FB_CMD_SET_GAIN_SSEL, &ssel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}
