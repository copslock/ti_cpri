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
 * @defgroup DFE_LLD_TX_FUNCTION TX
 * @ingroup DFE_LLD_FUNCTION
 */
 
/**
 * @brief Program TX Mixer
 * @ingroup DFE_LLD_TX_FUNCTION
 *
 * Write new Tx Mixer NCO frequency to shadow memory. The precision is refClock/2^48.
 * NOTE, Dfe_issueSyncUpdateTxMixer () should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param txPath	[in] Tx path Id, 0 - 1
 *  @param refClock	[in] reference clock
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
 *  - Dfe_issueSyncUpdateTxMixer() should be called later to let hardware take new configuration.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progTxMixer
(
	DFE_Handle hDfe,
	DfeFl_TxPath txPath,
	float refClock,
    float freq[2]
)
{
	DfeFl_Status status;
	DfeFl_TxMixFreq TxMixFreq;
	uint32_t i;
	int64_t numi64, deni64;
	uint64_t freq_eng;
	double numdbl, dendbl, temp, temp1;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(txPath > 1)
    {
		Dfe_osalLog("Invalid parameter in tx mixer!");
		return DFE_ERR_INVALID_PARAMS;
    }

	// change Tx Mix freq
    for(i = 0; i < 2; i++)
    {
		numi64 = freq[i]*1000000;
		deni64 = refClock*1000000;
		numdbl = numi64;
		dendbl = deni64;
		temp = numdbl/dendbl;
		temp1 = (temp<0)?temp+1:temp;
		temp1 *= 281474976710656.0;
		freq_eng = temp1 + 0.5;
    	TxMixFreq.txDev = (DfeFl_TxDev) (txPath*2+i);
    	TxMixFreq.lower32 = freq_eng & 0xFFFFFFFF;
    	TxMixFreq.upper16 = (freq_eng>>32) & 0xFFFF;
    	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_MIX_FREQ, &TxMixFreq) );
    }

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update TX Mixer
 * @ingroup DFE_LLD_TX_FUNCTION
 *
 * Issue sync to let TX Mixer run with new frequency. Dfe_getSyncStatus() may be called later to check if the sync has come.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param txPath	[in] Tx path Id, 0 ~ 1
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
DFE_Err Dfe_issueSyncUpdateTxMixer
(
	DFE_Handle hDfe,
	DfeFl_TxPath txPath,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_TxSsel TxSsel;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(txPath > 1)
    {
		Dfe_osalLog("Invalid parameter in issue sync for tx mixer!");
		return DFE_ERR_INVALID_PARAMS;
    }

    // set sync
	TxSsel.txPath = txPath;
	TxSsel.ssel = ssel;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_MIX_SSEL, &TxSsel) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Program TX PA Protection
 * @ingroup DFE_LLD_TX_FUNCTION
 *
 * Write new Tx PA protection configuration. 
 * typedef struct
 * {
 * 	// square clipper threshold
 * 	Uint32 threshold;
 * 	// clipper counter threshold (C1)
 * 	Uint32 cc_thr;
 * 	// peak threshold (TH0)
 * 	Uint32 TH0;
 * 	// peak counter threshold (C0)
 * 	Uint32 peak_thr;
 * 	// peakgain counter threshold (C2)
 * 	Uint32 peakgain_thr;
 * } DFE_TxPAprotPeak;
 * typedef struct
 * {
 * 	// mu_p for IIR
 * 	Uint32 mu0;
 * 	// mu_q for IIR
 * 	Uint32 mu1;
 * 	// RMS threshold to reduce CFR gain(TH1)
 * 	Uint32 TH1;
 * 	// RMS threshold to shut down (TH2)
 * 	Uint32 TH2;
 * 	// RMS threshold to peak approaching saturation (TH4)
 * 	Uint32 TH4;
 * 	// threshold selection for a1
 * 	Uint32 th1Sel;
 * 	// threshold selection for a2
 * 	Uint32 th2Sel;
 * 	// threshold selection for a6
 * 	Uint32 th6Sel;
 * } DFE_TxPAprotRms;
 * a1 = 1, RMS power is greater than TH1, the CFR gain will be reduced; 
 * a2 = 1, RMS power is greater than TH2, the TX output will be shut down.
 * a3 = 1, RMS power gain is less than 1, the power is saturating.
 * a4 = 1, peak gain counter is greater than peakgain_thr, peak is clipping.
 * a5 = 1, circular clipper counter is greater than cc_thr, peak is clipping.
 * a6 = 1, RMS power is greater than TH4, power is approaching saturation.
 * a7 = 1, peak counter is greater than peak_thr, peak is approaching saturation.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param txDev	[in] Tx Dev Id, 0 ~ 3
 *  @param txPAprotPeak	[in] Tx PA protection for peak adjustment
 *  @param txPAProtRms	[in] Tx PA protection for rms adjustment
 *  @param mask	[in] Tx PA protection mask configuration for stopDPD interrupt, the 5 LSB bits correspond to a5, a4, a3, a2 and a1
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
DFE_Err Dfe_progTxPaProtection
(
	DFE_Handle hDfe,
	DfeFl_TxDev txDev,
	DFE_TxPAprotPeak txPAprotPeak,
	DFE_TxPAprotRms txPAprotRms,
	uint32_t mask
)
{
	DfeFl_Status status;
	DfeFl_TxPaMask TxPaMask;
	DfeFl_TxPaIirMu TxPaIirMu;
	DfeFl_TxPaIirThVal TxPaIirThVal;
	DfeFl_TxPaIirThSel TxPaIirThSel;
	DfeFl_TxPaTh TxPaTh;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(txDev > 3)
    {
		Dfe_osalLog("Invalid parameter in tx pa protection!");
		return DFE_ERR_INVALID_PARAMS;
    }

	// set PA IIR mu
	TxPaIirMu.txDev = txDev;
	TxPaIirMu.mu0 = txPAprotRms.mu0;
	TxPaIirMu.mu1 = txPAprotRms.mu1;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_IIR_MU, &TxPaIirMu) );

	// set PA IIR threshold
	// Unit gain is 0xFFFF;
	TxPaIirThVal.txDev = txDev;
	TxPaIirThVal.TH1 = txPAprotRms.TH1;
	TxPaIirThVal.TH2 = txPAprotRms.TH2;
	TxPaIirThVal.TH4 = txPAprotRms.TH4;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_IIR_TH_VAL, &TxPaIirThVal) );

	// set PA IIR threshold select
	TxPaIirThSel.txDev = txDev;
	TxPaIirThSel.th1Sel = txPAprotRms.th1Sel;
	TxPaIirThSel.th2Sel = txPAprotRms.th2Sel;
	TxPaIirThSel.th6Sel = txPAprotRms.th6Sel;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_IIR_TH_SEL, &TxPaIirThSel) );

	// set PA mask
	TxPaMask.txDev = txDev;
	TxPaMask.mask = mask;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_MASK, &TxPaMask) );

	// set PA threshold
	TxPaTh.txDev = txDev;
	TxPaTh.theshld = txPAprotPeak.threshold;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_TH, &TxPaTh) );

	// set PA cc threshold
	TxPaTh.theshld = txPAprotPeak.cc_thr;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_CC_TH, &TxPaTh) );

	// set PA TH0
	TxPaTh.theshld = txPAprotPeak.TH0;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_PEAK_CNT_TH, &TxPaTh) );

	// set PA peak threshold
	TxPaTh.theshld = txPAprotPeak.peak_thr;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_PEAK_TH, &TxPaTh) );

	// set PA peak gain threshold
	TxPaTh.theshld = txPAprotPeak.peakgain_thr;
	CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_PEAK_GAIN_TH, &TxPaTh) );

	return DFE_ERR_NONE;
}

/**
 * @brief Get TX PA Protection Interrupt Status
 * @ingroup DFE_LLD_TX_FUNCTION
 *
 * Get Tx PA protection interrupt status of one Tx path.
 * Interrupt 1: 	a1 = 1;
 * Interrupt 2: 	a2 = 1;
 * Interrupt 3: 	a3 = 1;
 * Interrupt 4: 	a4 = 1 or a5 = 1;
 * Interrupt 5: 	a6 = 1;
 * Interrupt 6: 	a7 = 1;
 * Shutdown: 		a2 = 1;
 * lowCFRgain: 	a1 = 1;
 * stopDPD:		programmable with a1, a2, a3, a4, a5 (programmable  				mask, then OR all unmasked bits)
 *
 *  @param hDfe	[in] DFE device handle
 *  @param txPAprotIntr	[out] Tx PA protection interrupt status
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
DFE_Err Dfe_getTxPAprotIntrStatus
(
    DFE_Handle hDfe,
    DfeFl_TxPaIntrpt *txPAprotIntr
)
{
	DfeFl_Status status;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(txPAprotIntr->txDev > 3)
    {
		Dfe_osalLog("Invalid parameter in get tx pa protect interrupt status!");
		return DFE_ERR_INVALID_PARAMS;
    }

    CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PA_INTERRUPT, txPAprotIntr) );

	return DFE_ERR_NONE;
}

/**
 * @brief Clear TX PA Protection Interrupt Status
 * @ingroup DFE_LLD_TX_FUNCTION
 *
 * Clear complete interrupt status of a TX PA protection.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param txDev	[in] Tx device Id, 0 ~ 3
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
DFE_Err Dfe_clearTxPAprotIntrStatus
(
    DFE_Handle hDfe,
    DfeFl_TxDev txDev
)
{
	DfeFl_Status status;
	DfeFl_TxPaIntrpt TxPaIntrpt;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(txDev > 3)
    {
		Dfe_osalLog("Invalid parameter in clear tx pa protect interrupt status!");
		return DFE_ERR_INVALID_PARAMS;
    }

    TxPaIntrpt.txDev = txDev;
	TxPaIntrpt.intrpt1 = 0;
	TxPaIntrpt.intrpt2 = 0;
	TxPaIntrpt.intrpt3 = 0;
	TxPaIntrpt.intrpt6 = 0;
	TxPaIntrpt.lowerCfrGain = 0;
	TxPaIntrpt.shutdown = 0;
	TxPaIntrpt.stopDpd = 0;
    CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_PA_INTERRUPT, &TxPaIntrpt));


	return DFE_ERR_NONE;
}

/**
 * @brief Read TX PA Protection Power Status
 * @ingroup DFE_LLD_TX_FUNCTION
 *
 * Read Tx PA protection internal status of one Tx path.
 * typedef struct
 * {
 * 	// maximum magnitude of D3
 * 	Uint32 mag;
 * 	// IIR output at D50
 * 	Uint32 d50;
 * 	// IIR output at D51
 * 	Uint32 d51;
 * } DFE_TxPAprotPwrStatus;
 *
 *  @param hDfe	[in] DFE device handle
 *  @param txDev	[in] Tx device Id, 0 ~ 1
 *  @param clrRead	[in] flag to set clear after reading for magnitude
 *    - 0 means no clear after reading
 *    - 1 means clear after reading
 *  @param txPAprotPwrStatus	[out] Tx PA protection internal status
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
DFE_Err Dfe_getTxPAprotPwrStatus
(
    DFE_Handle hDfe,
    DfeFl_TxDev txDev,
    uint32_t clrRead,
    DFE_TxPAprotPwrStatus *txPAprotPwrStatus
)
{
	DfeFl_Status status;
	DfeFl_TxDevData TxData;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(txDev > 3)
    {
		Dfe_osalLog("Invalid parameter in get tx pa protect power status!");
		return DFE_ERR_INVALID_PARAMS;
    }


	TxData.txDev = txDev;
	if(clrRead == 0) // not clear after read
	{
		CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PA_MAXMAG_NO_CLR, &TxData) );
	}
	else
	{
		CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PA_MAXMAG_CLR, &TxData) );
	}
	txPAprotPwrStatus->mag = TxData.data;

	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PA_D50, &TxData) );
	txPAprotPwrStatus->d50 = TxData.data;

	CSL_HW_QUERY( dfeFl_TxGetHwStatus(hDfe->hDfeTx[0], DFE_FL_TX_QUERY_GET_PA_D51, &TxData) );
	txPAprotPwrStatus->d51 = TxData.data;


	return DFE_ERR_NONE;
}
