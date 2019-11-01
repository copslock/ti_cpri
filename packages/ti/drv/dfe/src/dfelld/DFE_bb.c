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
/** @file dfe_fl_cbGetHwStatus.c
 *
 *  @path  $(DRVPATH)\dfe\src\dfelld
 *
 *  @brief File for LLD layer APIs of BB block
 *
 *
 */

#include <math.h>
#include <ti/drv/dfe/dfe_drv.h>
#include <ti/drv/dfe/dfe_osal.h>
#include <ti/drv/dfe/dfe_internal.h>

/**
 * @defgroup DFE_LLD_BB_FUNCTION BB
 * @ingroup DFE_LLD_FUNCTION
 */

/**
 * @brief Program BBTX Gain
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Write new BBTX AxCs' gains to shadow memory. The range for a gain is 
 * -84dB ~ +6dB. The API changes gain's real part only.
 * NOTE, Dfe_issueSyncUpdateBbtxGain should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param numAxCs	[in] number of AxCs whose gains are to be changed
 *  @param axc	[in] array of AxCs whose gains are to be changed
 *  @param gain	[in] array of new real gains, in dB
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *  - Dfe_issueSyncUpdateBbtxGain () should be called later to copy gains to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progBbtxGain
(
    DFE_Handle hDfe,
    uint32_t  numAxCs,
    uint32_t  axc[],
    float  gain[]
)
{    
	const float BB_UNITY_GAIN = 0x4000;   //max 6 dB  min -84dB
	uint32_t ui;
	DfeFl_Status status;
	DfeFl_BbTxGainConfig bbGainCfg;

    VALID_DFE_HANDLE(hDfe);
    
    if(numAxCs > DFE_FL_BB_ANTENNA_MAX_AXCS || axc == NULL || gain == NULL)
    {
        Dfe_osalLog("too many AxCs or NULL pointers");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    bbGainCfg.numAxCs = numAxCs;
    for(ui = 0; ui < numAxCs; ui++)
    {
        bbGainCfg.axcGain[ui].axc = axc[ui];
        bbGainCfg.axcGain[ui].gainI = (unsigned short)((pow(10,(gain[ui]/20.0))) * (float) BB_UNITY_GAIN);
        bbGainCfg.axcGain[ui].gainQ = 0;
    }
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_UPD_TXGAIN, &bbGainCfg) );    
    
    return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update BBTX Gain
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Issue sync to copy BBTX gains from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
 * This API also clears update complete interrupt status bit of the corresponding carrier type.
 * To check if update complete, call Dfe_getBbtxGainUpdateComplete().
 * NOTE, Both axc_valid bit and gain_en bit (in register dfe.bb.bbtxif_axc_config0) have to be set '1', in order to see effectiveness of the gains.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ct	[in] carrier type
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
DFE_Err Dfe_issueSyncUpdateBbtxGain
(
    DFE_Handle hDfe,
    uint32_t ct,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_BbTxRxGainCarrierTypeData bbGainCtData;

    VALID_DFE_HANDLE(hDfe);
    
    if(ssel != DFE_FL_SYNC_GEN_SIG_NEVER)
    {
    	// clear gain update complete interrupt status
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXGAIN_INTR_STATUS, &ct) );
    }
    
    // update gain sync select
	bbGainCtData.ct = ct;
	bbGainCtData.data = ssel;
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_TXGAIN_SSEL, &bbGainCtData) );
    
    return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Get BBTX Gain Update Complete
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Get BBTX gain update complete status. The API first reads txgain_update_status, if corresponding ct bit is clear, then the update is still in progress. Otherwise, it further reads back and returns the update complete interrupt status.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ct	[in] carrier type
 *  @param complete	[out] buffer of update complete status
 *    - 0 = still in progress
 *    - 1 = update complete
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_QUERY, if CSL HwGetStatus() failed
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
DFE_Err Dfe_getBbtxGainUpdateComplete
(
    DFE_Handle hDfe,
    uint32_t ct,
    uint32_t *complete
)
{
	DfeFl_Status status;
	DfeFl_BbTxRxGainCarrierTypeData bbGainCtData;

    VALID_DFE_HANDLE(hDfe);
    if(complete == NULL)
    {
        Dfe_osalLog("complete pointer is NULL");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    // check update pending
	bbGainCtData.ct = ct;
	bbGainCtData.data = 0;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TXGAIN_UPDATE_STATUS, &bbGainCtData) );
    if(bbGainCtData.data != 0)
    {
        *complete = 0;
        return DFE_ERR_NONE;
    }
    
    // check complete interrupt
    bbGainCtData.data = 0;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TXGAIN_INTR_STATUS, &bbGainCtData) );
    *complete = bbGainCtData.data;
    
    return DFE_ERR_NONE;
 }

/**
 * @brief Clear BBTX Gain Update Complete Interrupt Status
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Clear complete interrupt status of a BBTX gain update.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ct	[in] carrier type
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
DFE_Err Dfe_clearBbtxGainUpdateCompleteIntrStatus
(
    DFE_Handle hDfe,
    uint32_t ct
)
{
	DfeFl_Status status;
	
    VALID_DFE_HANDLE(hDfe);
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXGAIN_INTR_STATUS, &ct) );
	
	return DFE_ERR_NONE;
}

/**
 * @brief Program BBTX Power Meter
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Write new BBTX power meter configuration. 
 *
 * NOTE, Dfe_issueSyncUpdateBbtxPowmtr should be called later to let hardware take the configuration.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] BBTX power meter Id, 0 ~ 15
 *  @param mtrCfg	[in] new meter configuration
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
 *  - Dfe_issueSyncUpdateBbtxPowmtr() should be called later to let hardware take new configuration.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progBbtxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DFE_BbtxPowmtrConfig *mtrCfg
)
{
	DfeFl_Status status;
	DfeFl_BbPowerMeterConfig bbPmCfg;
	
    VALID_DFE_HANDLE(hDfe);

	bbPmCfg.pmId = pmId;
	bbPmCfg.enable = mtrCfg->enable;
	bbPmCfg.countSource = mtrCfg->countSource;
	bbPmCfg.inSource = mtrCfg->inSource;
	bbPmCfg.tddMode = mtrCfg->tddMode;
	bbPmCfg.outFormat = mtrCfg->outFormat;
	bbPmCfg.syncDly = mtrCfg->syncDly;
	bbPmCfg.interval = mtrCfg->interval;
	bbPmCfg.intgPd = mtrCfg->intgPd;
	bbPmCfg.pwrUpdate = mtrCfg->powUptIntvl;
	bbPmCfg.maxDb = 0; // for RXPM only, initialize here to meet coverity requirement
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_TXPM, &bbPmCfg) );

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update BBTX Power Meter
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Issue sync to let BBTX power meter run with new configuration. Dfe_getSyncStatus() may be called later to check if the sync has come.
 * NOTE, Both axc_valid bit and pm_en bit (in register dfe.bb.bbtxif_axc_config0) have to be set '1', in order to make power meter run measurement.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] BBTX power meter Id, 0 ~ 15
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
DFE_Err Dfe_issueSyncUpdateBbtxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_BbPowerMeterSsel bbPmSsel;
	
    VALID_DFE_HANDLE(hDfe);
    
	bbPmSsel.pmId = pmId;
	bbPmSsel.ssel = ssel;
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_TXPM_SSEL, &bbPmSsel) );
    
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Clear BBTX Power Meter Done Status
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Clear complete interrupt status of a BBTX power meter.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] BBTX power meter Id, 0 ~ 15
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
DFE_Err Dfe_clearBbtxPowmtrDoneIntrStatus
(
    DFE_Handle hDfe,
    uint32_t pmId
)
{
	DfeFl_Status status;
	
    VALID_DFE_HANDLE(hDfe);
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_TXPM_INTR_STATUS, &pmId) );
	
	return DFE_ERR_NONE;
}

/**
 * @brief Get BBTX Power Meter Done Status
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Get complete interrupt status of a BBTX power meter.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] BBTX power meter Id, 0 ~ 15
 *  @param complete	[out] BBTX power meter complete status, 
 *    - 0 = measurement not complete yet
 *    - 1 = measurement complete
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *  - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
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
DFE_Err Dfe_getBbtxPowmtrDoneIntrStatus
(
    DFE_Handle hDfe,
    uint32_t pmId,
    uint32_t *complete
)
{
	DfeFl_Status status;
    DfeFl_BbPowMtrIntrStatus	intrSts;
    
    VALID_DFE_HANDLE(hDfe);
    if(complete == NULL)
    {
        Dfe_osalLog("complete pointer is NULL");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    intrSts.pmId = pmId;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TXPM_INTR_STATUS, &intrSts) );
    *complete = intrSts.status;
    
    return DFE_ERR_NONE;
}

/**
 * @brief Read BBTX Power Meter
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Read peak and average power results of a BBTX power meter via CPU.
 *
 * This API reads the DFE registers directly and depending on the output format selected for the
 * power meter at configuration time the results will be expressed either in:
 *
 *  - logarithmic dBfs scale if the logarithmic output format has been selected
 *  - linear magnitude square scale if the floating point output format has been selected
 *
 * For the linear scale the result is computed with I and Q in the range [-1.0, +1.0].
 *
 * NOTE: Both axc_valid bit and pm_en bit (in register dfe.bb.bbtxif_axc_config0) have to
 * be set '1', in order for a power meter measurement to run.
 *
 * The API doesn't wait for a new measurement; it reads back whatever is the current result.
 *
 *  @param hDfe	    [in] DFE device handle
 *  @param pmId	    [in] BBTX power meter Id, 0 ~ 15
 *  @param peak	    [out] peak power of AxC
 *  @param average	[out] average power of AxC
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *  - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
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
DFE_Err Dfe_readBbtxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    float *peak,
    float *average
)
{
	DfeFl_Status status;
    DfeFl_BbPowMtrResult     bbpmRead;
    DfeFl_BbPowerMeterConfig bbpmConfig;
    int integer, fraction, exponent;

    VALID_DFE_HANDLE(hDfe);
    if(peak == NULL || average == NULL)
    {
        Dfe_osalLog("peak/average pointer is NULL");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    bbpmRead.pmId = pmId;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TXPM_RESULT, &bbpmRead) );
    
    /* Query the power meter configuration to check the output format and the integration period
       for this pmId and adjust the average with the integration period */
    bbpmConfig.pmId =  pmId;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TXPM_CFG, &bbpmConfig) );

    if (bbpmConfig.outFormat == DFE_FL_BB_POWMTR_OUTFMT_FLOAT_10P16E6) {
    	/* In this case the power meter returns results in magnitude squared floating point format.  */
		integer = CSL_FEXTR(bbpmRead.peakPower, 15, 6);
		exponent = CSL_FEXTR(bbpmRead.peakPower, 5, 0);
		fraction = bbpmRead.peakPower_extend;

		*peak = ((float)integer+((float)fraction/65536.0))*pow(2,exponent);
		*peak /= (32767.0*32767.0); /* Scale the result so that I and Q are in the range [-1.0,+1.0] */

		integer = CSL_FEXTR(bbpmRead.rmsPower, 15, 6);
		exponent = CSL_FEXTR(bbpmRead.rmsPower, 5, 0);
		fraction = bbpmRead.rmsPower_extend;

		*average = ((float)integer+((float)fraction/65536.0))*pow(2,exponent);
		*average /= (32767.0*32767.0); /* Scale the result so that I and Q are in the range [-1.0,+1.0] */

		/* Return the average from accumulated power, so divide by the number of samples
		   in the integration period. */
		if (bbpmConfig.intgPd) {
			*average /= (float)(bbpmConfig.intgPd);
			/* TODO Handle cases where there are multiple integration periods.  */
		}

    } else {
    	/* In this case the power meter returns results in .1 dB increment integer format.  */
		*peak = ((float)bbpmRead.peakPower/10.0);
		*average = ((float)bbpmRead.rmsPower/10.0);

		/* Return the average from accumulated power, so divide by the number of samples
		   in the integration period. */
		if (bbpmConfig.intgPd) {
		    *average -= 10.0*log10((float)(bbpmConfig.intgPd));
			/* TODO Handle cases where there are multiple integration periods.  */
		}
		/* Convert to dBfs scale, by substracting the fullscale power in dB */
		*peak    -= 93.319; /* 10.0*log10((float)(32767*32767*2)) */
		*average -= 93.319; /* 10.0*log10((float)(32767*32767*2)) */
    }
    return DFE_ERR_NONE;
}


/**
 * @brief Open BBTX Power Meter DMA
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Allocate CPP/DMA channel and descriptor for BBTX power meter results uploading. 
 * Every power meter results 4 words, first twos for peak power, last twos for average power. Both peak power and average power are formatted in floating point (26 bit mantissa + 6 bit exponent). The mantissa is a <10.16> format.
 *
 * | Word#	| Bit[31..16]  | Bit[15..0]                                        |
 * | ------ | ------------ | ------------------------------------------------- |
 * | 0	    | Not Used	   | Peak power                                        |
 * |        |              | Bit[15..6], 10-bits integer portion of mantissa.  |
 * |        |              | Bit[5..0], 6-bits exponent (of 2-based)           |
 * | 1	    | Not Used     | Peak power                                        |
 * |        |              | 16-bits fraction portion of mantissa.             |
 * | 2	    | Not Used     | Average power                                     |
 * |        |              | Bit[15..6], 10-bits integer portion of mantissa.  |
 * |        |              | Bit[5..0], 6-bits exponent (of 2-based)           |
 * | 3	    | Not Used	   | Average power                                     |
 * |        |              | 16-bits fraction portion of mantissa.             |
 *
 * There're total 16 power meters for BBTX, result total 64 words. A DMA transact uploads all 64 words in one shot.
 * When BBTX power meter DMA already opened, call this API again will get error DFE_ERR_ALREADY_OPENED.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cppDmaId	[in] CPP/DMA channel Id
 *    - 0 ~ 31, open with specified channel
 *    - DFE_FL_CPP_OPEN_ANY, open with any available channel
 *  @param cppDescripId	[in] CPP/DMA descriptor Id
 *    - 0 ~ 127, open with specified descriptor
 *    - DFE_FL_CPP_OPEN_ANY, open with any available descriptor
 *  @param iqnChnl	[in] IQN2 CTL Ingress channel number, 0 ~ 15
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_AVAILABLE, if CPP/DMA channel not available
 *  - #DFE_ERR_CPP_DESCRIP_NOT_AVAILABLE, if CPP/Descriptor not available
 *  - #DFE_ERR_ALREADY_OPENED, if BBTX power meter DMA already opened
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
DFE_Err Dfe_openBbtxPowmtrDma
(
    DFE_Handle hDfe,
    uint32_t cppDmaId,
    uint32_t cppDescripId,
    uint32_t iqnChnl
)
{
	DfeFl_Status status;
	DfeFl_CppDmaHandle hDma;
	DfeFl_CppDescriptorHandle hDescrip;
	DfeFl_CppDescripConfig cfgDescrip;
	
	VALID_DFE_HANDLE(hDfe);
	if(iqnChnl > 15)
    {
        Dfe_osalLog("iqnChnl out of range");
        return DFE_ERR_INVALID_PARAMS;        
    }
	
	// check if already opened
	if(hDfe->hDmaBbtxPowmtr != NULL)
    {
        Dfe_osalLog("CPP/DMA for BB TXPM already opened");
        return DFE_ERR_ALREADY_OPENED;        
    }
	
    // open CPP/DMA for UL, PROG mode
    hDma = dfeFl_CppDmaOpen(
        hDfe->hDfeMisc[0],
        cppDmaId, // dmaId
        DFE_FL_CPP_DMA_MODE_PROG, // mode
        DFE_FL_CPP_OPEN_NONE, // trig
        &hDfe->cppResMgr, // resMgr
        &status);
	if(status != DFE_FL_SOK)
    {
        Dfe_osalLog("dfeFl_CppDmaOpen() failed, CSL error %d", status);
        return DFE_ERR_CPP_DMA_NOT_AVAILABLE;
    }
    // alloc CPP/DMA descriptor
    hDescrip = dfeFl_CppDecripOpen(
        hDfe->hDfeMisc[0],
        cppDescripId, // descriptor id
        &hDfe->cppResMgr, // resMgr
        &status);
    if(status != DFE_FL_SOK)
    {
        dfeFl_CppDmaClose(hDma);
        
        Dfe_osalLog("dfeFl_CppDecripOpen() failed, CSL error %d", status);
        return DFE_ERR_CPP_DESCRIP_NOT_AVAILABLE;
    }
    // config descriptor
    //  
    cfgDescrip.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((uint32_t)&hDfe->hDfeBb[0]->regs->bbtxpwrmeter[0] - (uint32_t)(hDfe->objDfe.regs));
    cfgDescrip.chanNum = iqnChnl;
    cfgDescrip.rw = DFE_FL_CPP_DMA_UL;
    cfgDescrip.numBytes = 256; // = 16x4x4
    cfgDescrip.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
    cfgDescrip.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
    cfgDescrip.pktSize = DFE_FL_CPP_DMA_PKT_SIZE_512;
    cfgDescrip.midImm = 0;
    cfgDescrip.linkNext = dfeFl_CppDescripGetId(hDescrip); // end of link
    dfeFl_CppDescripWrite(hDescrip, &cfgDescrip);
        
    // save resource allocated to context
    hDfe->hDmaBbtxPowmtr = hDma;
    hDfe->hDescripBbtxPowmtr = hDescrip;
    hDfe->bbtxPowmtrIqnChnl = iqnChnl;
    
    return DFE_ERR_NONE;
}

/**
 * @brief Close BBTX Power Meter DMA
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Close BBTX power meter DMA and free resources allocated by Dfe_openBbtxPowmtrDma(). 
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_VALID, if CPP/DMA handle not valid
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *  - Dfe_openBbtxPowmtrDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_closeBbtxPowmtrDma
(
    DFE_Handle hDfe
)
{
	//DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);
    
	// check if valid CPP/DMA opened
	if(hDfe->hDmaBbtxPowmtr == NULL)
    {
        Dfe_osalLog("CPP/DMA handle not (bbtxpm) valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;        
    }

    dfeFl_CppDmaClose(hDfe->hDmaBbtxPowmtr);
    dfeFl_CppDescripClose(hDfe->hDescripBbtxPowmtr);
    
    // clean context
    hDfe->hDmaBbtxPowmtr = NULL;
    hDfe->hDescripBbtxPowmtr = NULL;
    hDfe->bbtxPowmtrIqnChnl = DFE_FL_CPP_OPEN_NONE;
    
    return DFE_ERR_NONE;
}

/**
 * @brief Enable BBTX Power Meter DMA
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Enable CPP/DMA for BBTX Power Meter DMA by doing following
 * - Set dma_ssel to sense ALT_BBTX_PWRMTR
 * - Set txpm_auxint_mask to (1u << pmId)
 * When power meter of pmId completes a measurement, ALT_BBTX_PWRMTR interrupt will trigger CPP/DMA to start transferring.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] Id of BBTX power meter that triggers CPP/DMA
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_VALID, if CPP/DMA handle not valid
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *  - Dfe_openBbtxPowmtrDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_enableBbtxPowmtrDma
(
    DFE_Handle hDfe,
    uint32_t pmId
)
{
	DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);
    
	// check if valid CPP/DMA opened
	if(hDfe->hDmaBbtxPowmtr == NULL)
    {
        Dfe_osalLog("CPP/DMA handle not (bbtxpm) valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;        
    }
    
    // set txpm_auxint_mask for corresponding power meter
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_ENB_TXPM_AUXINTR, &pmId) );
	
	// Set dma_ssel to sense ALT_BBTX_PWRMTR
    CSL_HW_CTRL( dfeFl_CppDmaArm(hDfe->hDmaBbtxPowmtr, dfeFl_CppDescripGetId(hDfe->hDescripBbtxPowmtr), DFE_FL_CPP_DMA_SSEL_ALT_SYNC(DFE_FL_CPP_DMA_SSEL_ALT_BBTX_PWRMTR)) );
    
    hDfe->bbtxPowmtr = pmId;

    return DFE_ERR_NONE;
}

/**
 * @brief Disable BBTX Power Meter DMA
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Disable BBTX power meter DMA by doing following steps,
 * - Clear txpm_auxint_mask, cut off BBTX power meter complete signal to CPP/DMA
 * - Change dma_ssel not to sense any signal
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_VALID, if CPP/DMA handle not valid
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *  - Dfe_openBbtxPowmtrDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_disableBbtxPowmtrDma
(
    DFE_Handle hDfe
)
{
	DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);
    
	// check if valid CPP/DMA opened
	if(hDfe->hDmaBbtxPowmtr == NULL)
    {
        Dfe_osalLog("CPP/DMA handle (bbtxpm) not valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;        
    }
    
    // clear txpm_auxint_mask for corresponding power meter
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_DIS_TXPM_AUXINTR, &hDfe->bbtxPowmtr) );
	
	// Set dma_ssel to sense NEVER
    dfeFl_CppDmaDismissSync(hDfe->hDmaBbtxPowmtr);
    
    return DFE_ERR_NONE;
}

/**
 * @brief Program BBRX Gain
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Write new BBRX AxCs' gains to shadow memory. The range of gain is -48.16dB ~ +96.3dB. When gain less than -48.2dB, samples are clamped to 0. 
 *
 * NOTE, Dfe_issueSyncUpdateBbrxGain() should be called later to let hardware copy gains from shadow to working memory.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param numAxCs	[in] number of AxCs whose gains are to be changed
 *  @param axc	[in] array of AxCs whose gains are to be changed
 *  @param gain	[in] array of new gains, in dB
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
 *  - Dfe_issueSyncUpdateBbtxGain () should be called later to copy gains to working memory.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progBbrxGain
(
    DFE_Handle hDfe,
    uint32_t  numAxCs,
    uint32_t  axc[],
    float  gain[]
)
{
	uint32_t ui;
	float linerGain;
	DfeFl_Status status;
	DfeFl_BbRxGainConfig bbGainCfg;

    VALID_DFE_HANDLE(hDfe);
    
    if(numAxCs > DFE_FL_BB_ANTENNA_MAX_AXCS || axc == NULL || gain == NULL)
    {
        Dfe_osalLog("too manay AxCs or NULL pointers");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    bbGainCfg.numAxCs = numAxCs;
    for(ui = 0; ui < numAxCs; ui++)
    {
        bbGainCfg.axcGain[ui].axc = axc[ui];
        linerGain = pow(10,(gain[ui]/20.0));
        bbGainCfg.axcGain[ui].gainInteger = (uint32_t)linerGain;
        bbGainCfg.axcGain[ui].gainFraction = (linerGain - bbGainCfg.axcGain[ui].gainInteger) * 256u;
    }
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_UPD_RXGAIN, &bbGainCfg) );    
    
    return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update BBRX Gain
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Issue sync to copy BBRX gains from shadow to working memory. Dfe_getSyncStatus() should be called later to check if the sync has come.
 *
 * NOTE, in order to make gains effectively, dfe.bb.bbrxif_axc_config0.axc_valid bit have be '1', and dfe.bb.bbrxif_axc_config0.beagc_mode value must be in range 0 ~ 3.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ct	[in] carrier type
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
DFE_Err Dfe_issueSyncUpdateBbrxGain
(
    DFE_Handle hDfe,
    uint32_t ct,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_BbTxRxGainCarrierTypeData bbGainCtData;

    VALID_DFE_HANDLE(hDfe);
    
    if(ssel != DFE_FL_SYNC_GEN_SIG_NEVER)
    {
    	// clear gain update complete interrupt status
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXGAIN_INTR_STATUS, &ct) );
    }
    
    // update gain sync select
	bbGainCtData.ct = ct;
	bbGainCtData.data = ssel;
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_RXGAIN_SSEL, &bbGainCtData) );
    
    return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Get BBRX Gain Update Complete
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Get BBRX gain update complete status. The API first reads rxgain_update_status, if corresponding ct bit is clear, then the update is still in progress. Otherwise, it further reads back and returns the update complete interrupt status.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ct	[in] carrier type
 *  @param complete	[out] buffer of update complete status
 *    - 0 = still in progress
 *    - 1 = update complete
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_QUERY, if CSL HwGetStatus() failed
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
DFE_Err Dfe_getBbrxGainUpdateComplete
(
    DFE_Handle hDfe,
    uint32_t ct,
    uint32_t *complete
)
{
	DfeFl_Status status;
	DfeFl_BbTxRxGainCarrierTypeData bbGainCtData;

    VALID_DFE_HANDLE(hDfe);
    if(complete == NULL)
    {
        Dfe_osalLog("complete pointer is NULL");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    // check update pending
	bbGainCtData.ct = ct;
	bbGainCtData.data = 0;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_RXGAIN_UPDATE_STATUS, &bbGainCtData) );
    if(bbGainCtData.data != 0)
    {
        *complete = 0;
        return DFE_ERR_NONE;
    }
    
    // check complete interrupt
    bbGainCtData.data = 0;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_RXGAIN_INTR_STATUS, &bbGainCtData) );
    *complete = bbGainCtData.data;
    
    return DFE_ERR_NONE;
}
 
/**
 * @brief Clear BBRX Gain Update Complete Interrupt Status
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Clear complete interrupt status of a BBRX gain update.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ct	[in] carrier type
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
DFE_Err Dfe_clearBbrxGainUpdateCompleteIntrStatus
(
    DFE_Handle hDfe,
    uint32_t ct
)
{
	DfeFl_Status status;
	
    VALID_DFE_HANDLE(hDfe);
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXGAIN_INTR_STATUS, &ct) );
	
	return DFE_ERR_NONE;
}

/**
 * @brief Program BBRX Power Meter
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Write new BBRX power meter configuration. 
 * NOTE, Dfe_issueSyncUpdateBbrxPowmtr should be called later to let hardware take the configuration.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] BBRX power meter Id, 0 ~ 15
 *  @param mtrCfg	[in] new meter configuration
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
 *  - Dfe_issueSyncUpdateBbrxPowmtr() should be called later to let hardware take new configuration.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         // BBTX power meter config
         typedef struct
         {
             // enable power meter function
             DfeFl_BbPowMtrEnable enable;
             // carrier type 
             Uint32 countSource;
             // power meter input source
             DfeFl_BbPowMtrInSource inSource;
             // tdd mode
             DfeFl_BbPowMtrTddMode tddMode;
             // delay from sync
             Uint32 syncDly;
             // meter interval
             Uint32 interval;
             // integration period
             Uint32 intgPd;
             // count of measurements, i.e. count of intervals
             Uint32 powUptIntvl;
             // RX Maximum full scale power in dB for the programmed power interval time. 
             // Used by feed forward power update.  
             // Value is in units of 0.05dB resolution
             Uint32 maxdB;
             // Output scale of the BB RX Power Meter. The power meters
             // can report the results on a linear scale (that is I^2+Q^2),
             // or on logarithmic scale (that is 10*log10(I^2+Q^2)).
             // * When outputScale is set to 0 the linear scale is used.
             // * When outputScale is set to 1 the logarithmic scale is used.
             // In both cases the result will be reported as a floating point
             // value, and in the case of the logarithmic scale the value in
             // in dB is adjusted so that 0 dB corresponds to full power.
             uint32_t outputScale;
         } DFE_BbrxPowmtrConfig;
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progBbrxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DFE_BbrxPowmtrConfig *mtrCfg
)
{
	DfeFl_Status status;
	DfeFl_BbPowerMeterConfig bbPmCfg;
	
    VALID_DFE_HANDLE(hDfe);

	bbPmCfg.pmId = pmId;
	bbPmCfg.enable = mtrCfg->enable;
	bbPmCfg.countSource = mtrCfg->countSource;
	bbPmCfg.inSource = mtrCfg->inSource;
	bbPmCfg.tddMode = mtrCfg->tddMode;
	bbPmCfg.outFormat = mtrCfg->outFormat;
	bbPmCfg.syncDly = mtrCfg->syncDly;
	bbPmCfg.interval = mtrCfg->interval;
	bbPmCfg.intgPd = mtrCfg->intgPd;
	bbPmCfg.pwrUpdate = mtrCfg->powUptIntvl;
	bbPmCfg.maxDb = mtrCfg->maxdB;
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_RXPM, &bbPmCfg) );

	return DFE_ERR_NONE;
}

/**
 * @brief Issue Sync Update BBRX Power Meter
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Issue sync to let BBRX power meter run with new configuration. Dfe_getSyncStatus() may be called later to check if the sync has come.
 *
 * NOTE, Both dfe.bb.bbrxif_axc_config0.axc_valid bit and dfe.bb.bbrxif_axc_config1.pm_en bit have to be set '1', in order to make power meter run measurement.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] BBRX power meter Id, 0 ~ 15
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
DFE_Err Dfe_issueSyncUpdateBbrxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_BbPowerMeterSsel bbPmSsel;
	
    VALID_DFE_HANDLE(hDfe);
    
	bbPmSsel.pmId = pmId;
	bbPmSsel.ssel = ssel;
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_RXPM_SSEL, &bbPmSsel) );
    
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Clear BBRX Power Meter Done Status
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Clear complete interrupt status of a BBRX power meter.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] BBRX power meter Id, 0 ~ 15
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
DFE_Err Dfe_clearBbrxPowmtrDoneIntrStatus
(
    DFE_Handle hDfe,
    uint32_t pmId
)
{
	DfeFl_Status status;
	
    VALID_DFE_HANDLE(hDfe);
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CLR_RXPM_INTR_STATUS, &pmId) );
	
	return DFE_ERR_NONE;
}

/**
 * @brief Get BBRX Power Meter Done Status
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Get complete interrupt status of a BBRX power meter.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] BBRX power meter Id, 0 ~ 15
 *  @param complete	[out] BBRX power meter complete status, 
 *    - 0 = measurement not complete yet
 *    - 1 = measurement complete
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
DFE_Err Dfe_getBbrxPowmtrDoneIntrStatus
(
    DFE_Handle hDfe,
    uint32_t pmId,
    uint32_t *complete
)
{
	DfeFl_Status status;
    DfeFl_BbPowMtrIntrStatus	intrSts;
    
    VALID_DFE_HANDLE(hDfe);
    if(complete == NULL)
    {
        Dfe_osalLog("complete pointer is NULL");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    intrSts.pmId = pmId;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_RXPM_INTR_STATUS, &intrSts) );
    *complete = intrSts.status;
    
    return DFE_ERR_NONE;
}

/**
 * @brief Read BBRX Power Meter
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * This API reads the DFE registers directly and depending on the output format selected for the
 * power meter at configuration time the results will be expressed either in:
 *
 *  - logarithmic dBfs scale if the logarithmic output format has been selected
 *  - magnitude square scale if the floating point output format has been selected
 *
 * For the linear scale the result is computed with I and Q in the range [-1.0, +1.0].
 *
 * NOTE: Both axc_valid bit and pm_en bit (in register dfe.bb.bbtxif_axc_config0) have to
 * be set '1', in order for a power meter measurement to run.
 *
 * The API doesn't wait for a new measurement; it reads back whatever is the current result.
 *
 * NOTE, Both dfe.bb.bbrxif_axc_config0.axc_valid bit and dfe.bb.bbrxif_axc_config1.pm_en bit have to be set '1', in order to make power meter run measurement.
 *
 *  @param hDfe    [in] DFE device handle
 *  @param pmId    [in] BBRX power meter Id, 0 ~ 15
 *  @param peak    [out] peak power of AxC
 *  @param average [out] average power of AxC
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
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
DFE_Err Dfe_readBbrxPowmtr
(
    DFE_Handle hDfe,
    uint32_t pmId,
    float *peak,
    float *average
)
{
	DfeFl_Status status;
    DfeFl_BbPowMtrResult     bbpmRead;
    DfeFl_BbPowerMeterConfig bbpmConfig;
    int integer, fraction, exponent;

    VALID_DFE_HANDLE(hDfe);
    if(peak == NULL || average == NULL)
    {
        Dfe_osalLog("peak/average pointer is NULL");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    bbpmRead.pmId = pmId;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_RXPM_RESULT, &bbpmRead) );
    
    /* Query the power meter configuration to check the output format and the integration period
       for this pmId and adjust the average with the integration period */
    bbpmConfig.pmId =  pmId;
    CSL_HW_QUERY( dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_RXPM_CFG, &bbpmConfig) );

    if (bbpmConfig.outFormat == DFE_FL_BB_POWMTR_OUTFMT_FLOAT_10P16E6) {
    	/* In this case the power meter returns results in magnitude squared floating point format.  */
		integer = CSL_FEXTR(bbpmRead.peakPower, 15, 6);
		exponent = CSL_FEXTR(bbpmRead.peakPower, 5, 0);
		fraction = bbpmRead.peakPower_extend;

		*peak = ((float)integer+((float)fraction/65536.0))*pow(2,exponent);
		*peak /= (32767.0*32767.0); /* Scale the result so that I and Q are in the range [-1.0,+1.0] */
		/* Rx samples are on 24 bits and scaled down to 16 bits after the power meter/beAGC block,
		   so shifting the measurement result by 8 bits. */
		*peak /= (float)(256*256);

		integer = CSL_FEXTR(bbpmRead.rmsPower, 15, 6);
		exponent = CSL_FEXTR(bbpmRead.rmsPower, 5, 0);
		fraction = bbpmRead.rmsPower_extend;

		*average = ((float)integer+((float)fraction/65536.0))*pow(2,exponent);
		*average /= (32767.0*32767.0); /* Scale the result so that I and Q are in the range [-1.0,+1.0] */

		/* Return the average from accumulated power, so divide by the number of samples
		   in the integration period. */
		if (bbpmConfig.intgPd) {
			/* Rx samples are on 24 bits and scaled down to 16 bits after the power meter/beAGC block,
			   so shifting the measurement result by 8 bits (explains the 256*256 below) */
			*average /= (float)(bbpmConfig.intgPd*256*256);
			/* TODO Handle cases where there are multiple integration periods.  */
		}

    } else {
    	/* In this case the power meter returns results in .1 dB increment integer format.  */
		*peak = ((float)bbpmRead.peakPower/10.0);
		*average = ((float)bbpmRead.rmsPower/10.0);

		/* Rx samples are on 24 bits and scaled down to 16 bits after the power meter/beAGC block,
		   so shifting the measurement result by 8 bits (explains the 256*256 below) */
		*peak -= 10.0*log10((float)(256*256));

		/* Return the average from accumulated power, so divide by the number of samples
		   in the integration period. */
		if (bbpmConfig.intgPd) {
			/* Rx samples are on 24 bits and scaled down to 16 bits after the power meter/beAGC block,
			   so shifting the measurement result by 8 bits (explains the 256*256 below) */
		    *average -= 10.0*log10((float)(bbpmConfig.intgPd*256*256));
			/* TODO Handle cases where there are multiple integration periods.  */
		}
		/* Convert to dBfs scale, by substracting the fullscale power in dB */
		*peak    -= 93.319; /* 10.0*log10((float)(32767*32767*2)) */
		*average -= 93.319; /* 10.0*log10((float)(32767*32767*2)) */
    }
    return DFE_ERR_NONE;
}

/**
 * @brief Open BBRX Power Meter DMA
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Allocate CPP/DMA channel and descriptor for BBRX power meter results uploading. 
 * Every power meter results 4 words, first twos for peak power, last twos for average power. Both peak power and average power are formatted in floating point (26 bit mantissa + 6 bit exponent). The mantissa is a <10.16> format.
 *
 *  | Word#	| Bit[31..16] | Bit[15..0]                                       |
 *  | ----- | ----------- | ------------------------------------------------ |
 *  | 0     | Not Used    | Peak power                                       |
 *  |       |             | Bit[15..6], 10-bits integer portion of mantissa. |
 *  |       |             | Bit[5..0], 6-bits exponent (of 2-based)          |
 *  | 1	    | Not Used    | Peak power                                       |
 *  |       |             | 16-bits fraction portion of mantissa.            |
 *  | 2     | Not Used    | Average power                                    |
 *  |       |             | Bit[15..6], 10-bits integer portion of mantissa. |
 *  |       |             | Bit[5..0], 6-bits exponent (of 2-based)          |
 *  | 3     | Not Used    | Average power                                    |
 *  |       |             | 16-bits fraction portion of mantissa.            |
 * 
 * There're total 16 power meters for BBRX, result total 64 words. A DMA transact uploads all 64 words in one shot.
 * When BBRX power meter DMA already opened, call this API again will get error DFE_ERR_ALREADY_OPENED.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cppDmaId	[in] CPP/DMA channel Id
 *    - 0 ~ 31, open with specified channel
 *    - DFE_FL_CPP_OPEN_ANY, open with any available channel
 *  @param cppDescripId	[in] CPP/DMA descriptor Id
 *    - 0 ~ 127, open with specified descriptor
 *    - DFE_FL_CPP_OPEN_ANY, open with any available descriptor
 *  @param iqnChnl	[in] IQN2 CTL Ingress channel number, 0 ~ 15
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_AVAILABLE, if CPP/DMA channel not available
 *  - #DFE_ERR_CPP_DESCRIP_NOT_AVAILABLE, if CPP/Descriptor not available
 *  - #DFE_ERR_ALREADY_OPENED, if BBRX power meter DMA already opened
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
DFE_Err Dfe_openBbrxPowmtrDma
(
    DFE_Handle hDfe,
    uint32_t cppDmaId,
    uint32_t cppDescripId,
    uint32_t iqnChnl
)
{
	DfeFl_Status status;
	DfeFl_CppDmaHandle hDma;
	DfeFl_CppDescriptorHandle hDescrip;
	DfeFl_CppDescripConfig cfgDescrip;
	
	VALID_DFE_HANDLE(hDfe);
	if(iqnChnl > 15)
    {
        Dfe_osalLog("iqnChnl out of range");
        return DFE_ERR_INVALID_PARAMS;        
    }
	
	// check if already opened
	if(hDfe->hDmaBbrxPowmtr != NULL)
    {
        Dfe_osalLog("CPP/DMA for BB RXPM already opened");
        return DFE_ERR_ALREADY_OPENED;        
    }
	
    // open CPP/DMA for UL, PROG mode
    hDma = dfeFl_CppDmaOpen(
        hDfe->hDfeMisc[0],
        cppDmaId, // dmaId
        DFE_FL_CPP_DMA_MODE_PROG, // mode
        DFE_FL_CPP_OPEN_NONE, // trig
        &hDfe->cppResMgr, // resMgr
        &status);
	if(status != DFE_FL_SOK)
    {
        Dfe_osalLog("dfeFl_CppDmaOpen() failed, CSL error %d", status);
        return DFE_ERR_CPP_DMA_NOT_AVAILABLE;
    }
    // alloc CPP/DMA descriptor
    hDescrip = dfeFl_CppDecripOpen(
        hDfe->hDfeMisc[0],
        cppDescripId, // descriptor id
        &hDfe->cppResMgr, // resMgr
        &status);
    if(status != DFE_FL_SOK)
    {
        dfeFl_CppDmaClose(hDma);
        
        Dfe_osalLog("dfeFl_CppDecripOpen() failed, CSL error %d", status);
        return DFE_ERR_CPP_DESCRIP_NOT_AVAILABLE;
    }
    // config descriptor
    //  
    cfgDescrip.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((uint32_t)&hDfe->hDfeBb[0]->regs->bbrxpwrmeter[0] - (uint32_t)(hDfe->objDfe.regs));
    cfgDescrip.chanNum = iqnChnl;
    cfgDescrip.rw = DFE_FL_CPP_DMA_UL;
    cfgDescrip.numBytes = 256; // = 16x4x4
    cfgDescrip.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
    cfgDescrip.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
    cfgDescrip.pktSize = DFE_FL_CPP_DMA_PKT_SIZE_512;
    cfgDescrip.midImm = 0;
    cfgDescrip.linkNext = dfeFl_CppDescripGetId(hDescrip); // end of link
    dfeFl_CppDescripWrite(hDescrip, &cfgDescrip);
        
    // save resource allocated to context
    hDfe->hDmaBbrxPowmtr = hDma;
    hDfe->hDescripBbrxPowmtr = hDescrip;
    hDfe->bbrxPowmtrIqnChnl = iqnChnl;
    
    return DFE_ERR_NONE;
}

/**
 * @brief Close BBRX Power Meter DMA
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Close BBRX power meter DMA and free resources allocated by Dfe_openBbrxPowmtrDma(). 
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_VALID, if CPP/DMA handle not valid
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *  - Dfe_openBbrxPowmtrDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_closeBbrxPowmtrDma
(
    DFE_Handle hDfe
)
{
	//DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);
    
	// check if valid CPP/DMA opened
	if(hDfe->hDmaBbrxPowmtr == NULL)
    {
        Dfe_osalLog("CPP/DMA handle (bbrxpm) not valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;        
    }

    dfeFl_CppDmaClose(hDfe->hDmaBbrxPowmtr);
    dfeFl_CppDescripClose(hDfe->hDescripBbrxPowmtr);
    
    // clean context
    hDfe->hDmaBbrxPowmtr = NULL;
    hDfe->hDescripBbrxPowmtr = NULL;
    hDfe->bbrxPowmtrIqnChnl = DFE_FL_CPP_OPEN_NONE;
    
    return DFE_ERR_NONE;
}

/**
 * @brief Enable BBRX Power Meter DMA
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Enable CPP/DMA for BBRX Power Meter DMA by doing following:
 * - Set dma_ssel to sense ALT_BBRX_PWRMTR
 * - Set rxpm_auxint_mask to (1u << pmId)
 *
 * When power meter of pmId completes a measurement, ALT_BBRX_PWRMTR interrupt will trigger CPP/DMA to start transferring.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pmId	[in] Id of BBRX power meter that triggers CPP/DMA
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_VALID, if CPP/DMA handle not valid
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *  - Dfe_openBbrxPowmtrDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_enableBbrxPowmtrDma
(
    DFE_Handle hDfe,
    uint32_t pmId
)
{
	DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);
    
	// check if valid CPP/DMA opened
	if(hDfe->hDmaBbrxPowmtr == NULL)
    {
        Dfe_osalLog("CPP/DMA handle (bbrxpm) not valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;        
    }
    
    // set rxpm_auxint_mask for corresponding power meter
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_ENB_RXPM_AUXINTR, &pmId) );
	
	// Set dma_ssel to sense ALT_BBRX_PWRMTR
    CSL_HW_CTRL( dfeFl_CppDmaArm(hDfe->hDmaBbrxPowmtr, dfeFl_CppDescripGetId(hDfe->hDescripBbrxPowmtr), DFE_FL_CPP_DMA_SSEL_ALT_SYNC(DFE_FL_CPP_DMA_SSEL_ALT_BBRX_PWRMTR)) );
    
    hDfe->bbrxPowmtr = pmId;

    return DFE_ERR_NONE;
}

/**
 * @brief Disable BBRX Power Meter DMA
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Disable BBRX power meter DMA by doing following steps,
 *  -	Clear rxpm_auxint_mask, cut off BBRX power meter complete signal to CPP/DMA
 *  -	Change dma_ssel not to sense any signal
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_VALID, if CPP/DMA handle not valid
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *  - Dfe_openBbrxPowmtrDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_disableBbrxPowmtrDma
(
    DFE_Handle hDfe
)
{
	DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);
    
	// check if valid CPP/DMA opened
	if(hDfe->hDmaBbrxPowmtr == NULL)
    {
        Dfe_osalLog("CPP/DMA handle (bbrxpm) not valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;        
    }
    
    // clear rxpm_auxint_mask for corresponding power meter
	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_DIS_RXPM_AUXINTR, &hDfe->bbrxPowmtr) );
	
	// Set dma_ssel to sense NEVER
    dfeFl_CppDmaDismissSync(hDfe->hDmaBbrxPowmtr);
    
    return DFE_ERR_NONE;
}


/**
 * @brief Enable Disable BB AID Loopback
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * BB AID loopback is very useful to test IQ data flow before DFE. 
 *
 * For LTE the loop is like:
 * - BB DL buffer => Queue => IQN2 (PktDMA, AID2) => DFE (BB_AID) => IQN2 (AID2, PktDMA) => BB UL Buffer; 
 *
 * For WCDMA the loop is like,
 * - BB DL buffer => IQN2 (DIO2, AID2) => DFE (BB_AID) => IQN2 (AID2, DIO2) => BB UL buffer
 *
 * Set enable to '1' to enable BB AID loopback; clear it to '0' to disable BB AID loopback.
 *
 * NOTE, BB AID loopback isn't working well when AxCs having different rates. In this case, BB Buf loopback can be used, see Dfe_progBbbufLoopback().
 *
 *  @param hDfe	[in] DFE device handle
 *  @param Enable	[in] 1 to enable BB AID loopback, 0 to disable
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
DFE_Err Dfe_enableDisableBbaidLoopback
(
    DFE_Handle hDfe,
    uint32_t enable
)
{
	DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);

    if(enable != 0)
    {
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_ENB_AID_LOOPBACK, NULL) );
    }
    else
    {
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_DIS_AID_LOOPBACK, NULL) );
    }
    
    return DFE_ERR_NONE;    
}


/**
 * @brief Program BB BUF Loopback
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * BB Buf loopback loops back IQ data flow before DFE DDUC. 
 *
 * For LTE: 
 *  - BB DL buffer => Queue => IQN2 (PktDMA, AID2) => DFE (BBAID, BBBUF) => DFE (BB_BUF, BB_AID) => IQN2 (AID2, PktDMA) => BB UL Buffer; 
 *
 * For WCDMA: 
 *  - BB DL buffer => IQN2 (DIO2, AID2) => DFE (BB_AID, BB_BUF) => DFE (BB_BUF, BB_AID) => IQN2 (AID2, DIO2) => BB UL buffer
 *
 * Typical setup for argument bufLoopback:
 *
 *  | Use Case	           | bufLoopback member setup | Description           |
 *  | -------------------- | ------------------------ | --------------------- |
 *  | No loopback          | All = '0'	              | Normal operation mode |
 *  | Two dducs loopback   | duc0ToDdc1 = '1';        | dduc0 => dduc1        |
 *  |                      | other = '0'	          |                       |
 *  | Four dducs loopback  | duc1ToDdc2 = '1';        | dduc0 => dduc3        |
 *  |                      | duc0ToDdc3 = '1';        | dduc1 => dduc2        |
 *  |                      | other = '0'	          |                       |
 *  |                      |                          |                       |
 *  | Eight dducs loopback | duc3ToDdc4 = '1';        | dduc0 => dduc7        |
 *  |                      | duc2ToDdc5 = '1';        | dduc1 => dduc6        |
 *  |                      | duc1ToDdc6 = '1';        | dduc2 => dduc5        |
 *  |                      | duc0ToDdc7 = '1';        | dduc3 => dduc4        |
 *  |                      | other = '0'	          |                       |
 *  |               
 *  |               
 *  |               
 *
 *  @param hDfe	[in] DFE device handle
 *  @param bufLoopback	[in] Buf loopback configuration
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
DFE_Err Dfe_progBbbufLoopback
(
    DFE_Handle hDfe,
    DfeFl_BbLoopbackConfig *bufLoopback
)
{
	DfeFl_Status status;
    
    VALID_DFE_HANDLE(hDfe);
    
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_LOOPBACK, bufLoopback) );
    
    return DFE_ERR_NONE;    
}


/**
 * @brief Set BB AID UL Strobe Delay
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Set BB AID UL Strobe delay to adjust the time from the UL_STROBE to when FRAME_START is generated. This API is very useful to align uplink FRAME_START to the first sample in radio frame out of BB AID.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ct	[in] carrier type of the UL_STROBE
 *  @param dly	[in] Delay in carrier type samples
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
DFE_Err Dfe_setBbaidUlstrobeDelay
(
    DFE_Handle hDfe,
    uint32_t ct,
    uint32_t dly
)
{
	DfeFl_Status status;
    DfeFl_BbAidUlStrobeDelayConfig cfg;
    
    VALID_DFE_HANDLE(hDfe);
    
    cfg.ct = ct;
    cfg.dly = dly;
    
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_AID_ULSTROBE_DLY, &cfg) );
    
    return DFE_ERR_NONE;        
}

/**
 * @brief Program BB SigGen Ramp
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Program a BB signal geneator to produce a ramp. The startIq[0], stopIq[0], and slopeIq[0] are used to control I bus; startIq[1], stopIq[1], and slopeIq[1] are used to control Q bus. The rules should be followed,
 * - startIq <= stopIq
 * - (stopIq - startIq) % slopeIq = 0
 *
 * When startIq equal to stopIq and slopeIq is 0, a constant is produced.
 *
 * NOTE: BB AID signal generator produces same ramp to all BBTX channels.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param sigGenDev	[in] BB SigGen device
 *  @param Enable	[in] 1 to enable; 0 to disable
 *  @param startIq	[in] ramp start values for I bus and Q bus
 *  @param stopIq	[in] ramp stop values for I bus and Q bus
 *  @param slopeIq	[in] ramp step values for I bus and Q bus
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
DFE_Err Dfe_progBbsigGenRamp
(
    DFE_Handle hDfe,
    DfeFl_BbTestGenDev sigGenDev, 
    uint32_t enable,
    int16_t startIq[2],
    int16_t stopIq[2],
    int16_t slopeIq[2]
)
{
	DfeFl_Status status;
    DfeFl_BbTestGenConfig testGenCfg;
    
    VALID_DFE_HANDLE(hDfe);
    
    if(slopeIq[0] != 0)
    {        
        if((startIq[0] > stopIq[0]) || ((stopIq[0] - startIq[0]) % slopeIq[0] != 0))
        {
            Dfe_osalLog("invalid BB ramp config for I");
            return DFE_ERR_INVALID_PARAMS;                    
        }
    }
    if(slopeIq[1] != 0)
    {        
        if((startIq[1] > stopIq[1]) || ((stopIq[1] - startIq[1]) % slopeIq[1] != 0))
        {
            Dfe_osalLog("invalid BB ramp config for Q");
            return DFE_ERR_INVALID_PARAMS;                    
        }
    }
    
    testGenCfg.tgDev = sigGenDev;
    // only valid for AID
    testGenCfg.testEnable = enable;
    // enable data generation
    testGenCfg.genData = enable;
    // enable frame generation
    testGenCfg.genFrame = 0;
    // ramp (1), or LFSR (0)
    testGenCfg.rampMode = DFE_FL_BB_TESTGEN_RAMP_MODE_RAMP;
    // seed
    testGenCfg.seed = 0;
    // number of clocks per frame minus 1
    testGenCfg.frameLenM1 = 0;
    // ramp starting value
    testGenCfg.rampStart = ((uint32_t)startIq[0] << 16) | ((uint32_t)startIq[1] & 0xffffu);
    // ramp stop value
    testGenCfg.rampStop = ((uint32_t)stopIq[0] << 16) | ((uint32_t)stopIq[1] & 0xffffu);
    // ramp slop value
    testGenCfg.slope = ((uint32_t)slopeIq[0] << 16) | ((uint32_t)slopeIq[1] & 0xffffu);
    // 0 = generate data forever, n = generate data for n clock cycles
    testGenCfg.genTimer = 0;
    // number of data bits inverted (read-only)
    testGenCfg.numDataBits = 0;
    // to HW
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_TESTGEN, &testGenCfg) );
    
    return DFE_ERR_NONE;        
}

/**
 * @brief Issue Sync Update BB SigGen
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * Issue sync update BB SigGen. Dfe_getSyncStatus() can be called later to check if the sync has come.
 * When sync ssel comes, the ramp restarts from start value and step up the slope value per clock. When accumulated value equal to stop value, the ramp restarts again.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param sigGenDev	[in] BB SigGen device
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
DFE_Err Dfe_issueSyncUpdateBbsigGen
(
    DFE_Handle hDfe,
    DfeFl_BbTestGenDev sigGenDev, 
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_BbTestGenSsel testGenSsel;
    
    VALID_DFE_HANDLE(hDfe);
    
    testGenSsel.tgDev = sigGenDev;
    testGenSsel.ssel = ssel;
    CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_TESTGEN_SSEL, &testGenSsel) );
 
    return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);   
}

/**
 * @brief Program BB TestBus
 * @ingroup DFE_LLD_BB_FUNCTION
 *
 * DFE has many test probe points scattered over all sub-modules. CB can be used to capture a train of IQ bus signals at the probe. 
 * The API enables the specified BB probe to CB interface.
 * NOTE, all test probes "AND together" shares single CB interface. So software should enable no more than one probe at any time. LLD internally disables all probes first before arm a new one.  
 *
 *  @param hDfe	[in] DFE device handle
 *  @param testCbCtrl	[in] probe position 
 *  @param testCbAxc	[in] probe axc/buf number  
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
DFE_Err Dfe_progBbtestbus
(
    DFE_Handle hDfe,
    DfeFl_BbTestCbCtrl testCbCtrl, 
    uint32_t testCbAxc
)
{
	DfeFl_Status status;
	uint32_t data = 0;
	DfeFl_BbCapBuffConfig *pCfg = (DfeFl_BbCapBuffConfig *) &data;

    VALID_DFE_HANDLE(hDfe);

	pCfg->testCbCtrl = testCbCtrl;
	pCfg->testCbAxc = testCbAxc;

	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_CAPBUFF, pCfg) );

	return DFE_ERR_NONE;
}
