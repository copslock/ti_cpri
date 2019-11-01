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
 * @defgroup DFE_LLD_CB_FUNCTION CB
 * @ingroup DFE_LLD_FUNCTION
 */
 
/**
 * @brief Program CB Node Config
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * Write new CB node configuration for one CB node.
 *
 * NOTE, CB node ID defined as nodeCfg->cbNode is 0 ~ 8:
 *
 *  - Node 0 = DPD input
 *  - Node 1 = DPD output
 *  - Node 2 = DDUC input (FB output, IQ interleaved)
 *  - Node 3 = FB, resampler input or output (FB input, IQ parallel)
 *  - Node 4 = CFR block0 input
 *  - Node 5 = CFR block1 input
 *  - Node 6 = CFR block0 output
 *  - Node 7 = CFR block1 output
 *  - Node 8 = testbus
 *
 *  @param hDfe	[in] DFE device handle
 *  @param nodeCfg	[in] CB node configuration
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
DFE_Err Dfe_progCbNodecfg
(
    DFE_Handle hDfe,
    DfeFl_CbNodeCfg *nodeCfg
)
{
	DfeFl_Status status;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(nodeCfg->cbNode > DFE_FL_CB_NUM_NODE)
    {
		Dfe_osalLog("Invalid parameter in cb node cfg!");
		return DFE_ERR_INVALID_PARAMS;
    }

    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_NODE_CONFIG, nodeCfg) );

    return DFE_ERR_NONE;
}

/**
 * @brief Program CB Buf Config
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * Write new CB buf configuration for one CB buffer.
 * NOTE, CB buf ID defined as bufCfg->cbSet.cbBuf is 0 ~ 3.
 *   @verbatim
     // CB buf configuration
     typedef struct
     {
     	  // cb buf mode set
     	  DfeFl_CbModeSet  cbSet;
     	  // cb buf delay from sync
     	  Uint32 dly;
     	  // 0 = 1s/1c mode; 1 = 2s/1c mode
     	  Uint32 rate_mode;
     	  // capture buffer A fractional counter length minus 1;
   	      // range 0-15; value depends on the relative sampling rates for different buffers
     	  Uint32 frac_cnt;
     	  // fractional counter sync select
     	  Uint32 frac_cnt_ssel;
     	  // length counter sync select
     	  Uint32 len_cnt_ssel;
     	  // cb buf length, upto 8192 complex data
     	  Uint32 length;
     } DFE_CbBufCfg;
     @endverbatim
 *
 *  @param hDfe	[in] DFE device handle
 *  @param bufCfg	[in] CB buffer configuration
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
DFE_Err Dfe_progCbBufcfg
(
    DFE_Handle hDfe,
    DFE_CbBufCfg *bufCfg
)
{
	DfeFl_Status status;
	DfeFl_CbDly cbData;
	DfeFl_CbSsel cbSsel;


    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(bufCfg->cbSet.cbBuf > DFE_FL_CB_NUM_BUF)
    {
		Dfe_osalLog("Invalid parameter in cb buf cfg!");
		return DFE_ERR_INVALID_PARAMS;
    }

    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_MODE_SET, &bufCfg->cbSet) );

    cbData.cbBuf = bufCfg->cbSet.cbBuf;
    cbData.data = bufCfg->dly + bufCfg->length;
    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_DLY, &cbData) );

    cbData.data = bufCfg->rate_mode;
    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_RATE_MODE, &cbData) );

    cbData.data = bufCfg->frac_cnt;
    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_FRAC_CNT, &cbData) );

    cbSsel.cbBuf = bufCfg->cbSet.cbBuf;
    cbSsel.frac_cnt_ssel = bufCfg->frac_cnt_ssel;
    cbSsel.len_cnt_ssel = bufCfg->len_cnt_ssel;
    CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_SSEL, &cbSsel) );

    return DFE_ERR_NONE;
}

/**
 * @brief Arm CB and Issue Sync
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * Arm CB and issue sync to prepare for capture. The cb buf mode is also changed to capture mode.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param ssel	[in] sync select to capture
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
DFE_Err Dfe_armCbIssueSync
(
    DFE_Handle hDfe,
    DfeFl_MiscSyncGenSig ssel
)
{
	DfeFl_Status status;
	DfeFl_CbBufMode cbBufMode;
	DfeFl_CbArm cbArm;
	uint32_t i;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	// change to capture mode
	for(i = 0; i < DFE_FL_CB_NUM_BUF; i++)
	{
		cbBufMode.cbBuf = (DfeFl_CbBuf)i;
		cbBufMode.data = (uint32_t)DFE_FL_CB_C_STATIC;
		CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_BUF_MODE, &cbBufMode) );
	}

	CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CBC_START_SSEL, &ssel) );

	// issue cb arm
	cbArm.sync = 1;
	cbArm.cbDone = 1;
	CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CBC_ARM, &cbArm) );

	// issue sync
	return Dfe_issueSync(hDfe, ssel, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Get CB Done Status
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * The API gets back the cbDone Status.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cbDoneStatus	[out] pointer to cb done status
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
DFE_Err Dfe_getCbDoneStatus
(
	DFE_Handle hDfe,
	DfeFl_CbArm *cbDoneStatus
)
{
	DfeFl_Status status;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    CSL_HW_QUERY( dfeFl_CbGetHwStatus(hDfe->hDfeCb[0], DFE_FL_CB_QUERY_GET_CBC_ARM, cbDoneStatus) );

    return DFE_ERR_NONE;
}

/**
 * @brief Read CB Buf
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * Read CB buf data and CB status via CPU. It reads DFE registers directly.
 * The API changes to MPU mode and capture the 16MSB or total 18bit buffer based on requirement.
 * @verbatim
  // CB data
  typedef struct
  {
  	// I data
  	Uint32 *Idata;
  	// Q data
  	Uint32 *Qdata;
  } DFE_CbData;
   @endverbatim
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cBufId	[in] Cb Buffer Id, 0 ~ 3
 *  @param cbLength	[in] Cb length, 0 ~ 8192
 *  @param flag_18bit	[in] flag to read 18bit buffer
 *    - 0 = 16bit MSB
 *    - 1 = 18bit
 *  @param cbTemp	[in] pointer to the temporary buffer, size 8192*4 bytes
 *  @param cbStatus	[out] pointer to the cb status
 *  @param cbData	[out] pointer to the cb data
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
DFE_Err Dfe_readCbBuf
(
    DFE_Handle hDfe,
	DfeFl_CbBuf cbBufId,
	uint32_t cbLength,
	uint32_t flag_18bit,
	DfeFl_CbComplexInt *cbTemp,
    DfeFl_CbStatus *cbStatus,
    DFE_CbData *cbData
)
{
	DfeFl_Status status;
	DfeFl_CbBufMode cbBufMode;
	DfeFl_CbData cbRawData;
	uint32_t j, dstIdx, srcIdx;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

    if(cbBufId > DFE_FL_CB_NUM_BUF)
    {
		Dfe_osalLog("Invalid parameter in read cb buf!");
		return DFE_ERR_INVALID_PARAMS;
    }

	cbBufMode.cbBuf = cbBufId;
	cbBufMode.data = (uint32_t)DFE_FL_CB_MPU;
	CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_BUF_MODE, &cbBufMode) );

	cbStatus->cbBuf = cbBufId;
	CSL_HW_QUERY( dfeFl_CbGetHwStatus(hDfe->hDfeCb[0], DFE_FL_CB_QUERY_GET_CB_STATUS, cbStatus) );

	// start reading position
	srcIdx = (DFE_FL_MAX_CB_LENGTH + cbStatus->doneAddr - cbLength) % DFE_FL_MAX_CB_LENGTH;

	// read from done_addr to end of buffer
	cbRawData.cbBuf = cbBufId;
	cbRawData.startPos = srcIdx;
	cbRawData.size = min(DFE_FL_MAX_CB_LENGTH-srcIdx, cbLength);
	cbRawData.data = cbTemp;
	CSL_HW_QUERY( dfeFl_CbGetHwStatus(hDfe->hDfeCb[0], DFE_FL_CB_QUERY_GET_CB_MSB, &cbRawData) );
	for(j = 0; j < cbRawData.size; j++)
	{
		cbData[j].Idata = (uint32_t)cbTemp[j].real;
		cbData[j].Qdata = (uint32_t)cbTemp[j].imag;
	}
	if(flag_18bit == 1)
	{
		CSL_HW_QUERY( dfeFl_CbGetHwStatus(hDfe->hDfeCb[0], DFE_FL_CB_QUERY_GET_CB_LSB, &cbRawData) );
		for(j = 0; j < cbRawData.size; j++)
		{
			cbData[j].Idata = (cbData[j].Idata<<2)|(cbTemp[j].real&0x3u);
			cbData[j].Qdata = (cbData[j].Qdata<<2)|(cbTemp[j].imag&0x3u);
		}
	}
	// wrap back, read from 0
	dstIdx = cbRawData.size;
	if(dstIdx < cbLength)
	{
		cbRawData.cbBuf = cbBufId;
		cbRawData.startPos = 0;
		cbRawData.size = cbLength - dstIdx;
		cbRawData.data = cbTemp;
		CSL_HW_QUERY( dfeFl_CbGetHwStatus(hDfe->hDfeCb[0], DFE_FL_CB_QUERY_GET_CB_MSB, &cbRawData) );
		for(j = 0; j < cbRawData.size; j++)
		{
			cbData[dstIdx+j].Idata = (uint32_t)cbTemp[j].real;
			cbData[dstIdx+j].Qdata = (uint32_t)cbTemp[j].imag;
		}
		if(flag_18bit == 1)
		{
			CSL_HW_QUERY( dfeFl_CbGetHwStatus(hDfe->hDfeCb[0], DFE_FL_CB_QUERY_GET_CB_LSB, &cbRawData) );
			for(j = 0; j < cbRawData.size; j++)
			{
				cbData[dstIdx+j].Idata = (cbData[dstIdx+j].Idata<<2)|(cbTemp[j].real&0x3u);
				cbData[dstIdx+j].Qdata = (cbData[dstIdx+j].Qdata<<2)|(cbTemp[j].imag&0x3u);
			}
		}
	}

    return DFE_ERR_NONE;
}

/**
 * @brief Open CB Buf DMA
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * Allocate CPP/DMA channel and descriptor for CB buf reading.
 * Every CB buf has 8192 words for 16bit MSB and 8192 words for 2bit LSB.
 * Each word has I data in bit [31:16] and Q data in bit [15:0].
 * To transfer all 4 CB buffers, there are total 8 descriptors if all 18bit data are needed.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param flag_18bit	[in] flag to read 18bit buffer
 *    - 0 = 16bit MSB
 *    - 1 = 18bit
 *  @param cppDmaId	[in] CPP/DMA channel Id
 *    - 0 ~ 31, open with specified channel
 *    - DFE_FL_CPP_OPEN_ANY, open with any available channel
 *  @param cppDescripId	[in] pointer to CPP/DMA descriptor Id
 *    - 0 ~ 127, open with specified descriptor
 *    - DFE_FL_CPP_OPEN_ANY, open with any available descriptor
 *  @param iqnChnl	[in] IQN2 CTL Ingress channel number, 0 ~ 15
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_AVAILABLE, if CPP/DMA channel not available
 *  - #DFE_ERR_CPP_DESCRIP_NOT_AVAILABLE, if CPP/Descriptor not available
 *  - #DFE_ERR_ALREADY_OPENED, if BBTX power meter DMA already opened
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
DFE_Err Dfe_openCbBufDma
(
	DFE_Handle hDfe,
	uint32_t flag_18bit,
	uint32_t cppDmaId,
    uint32_t cppDescripId[8],
    uint32_t iqnChnl
)
{
	DfeFl_Status status;
	DfeFl_CppDmaHandle hDma;
	DfeFl_CppDescriptorHandle hDescrip[8];
	DfeFl_CppDescripConfig cfgDescrip;
	uint32_t LinkLen, i;

	if(flag_18bit == 1)
		LinkLen = 8;
	else
		LinkLen = 4;


    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	if(iqnChnl > 15)
    {
        Dfe_osalLog("iqnChnl out of range");
        return DFE_ERR_INVALID_PARAMS;
    }

	// check if already opened
	if(hDfe->hDmaCb != NULL)
	{
        Dfe_osalLog("CPP/DMA for CB already opened");
        return DFE_ERR_ALREADY_OPENED;
	}

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
	for(i = 0; i < LinkLen; i++)
	{
		hDescrip[i] = dfeFl_CppDecripOpen(
			hDfe->hDfeMisc[0],
			cppDescripId[i], // descriptor id
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
		if(flag_18bit == 1)
		{
			if (i%2 == 0)
				cfgDescrip.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((uint32_t)&hDfe->hDfeCb[0]->regs->capture_buffer_a_16msb - (uint32_t)(hDfe->objDfe.regs)) + 0x8000 * i/2;
			else
				cfgDescrip.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((uint32_t)&hDfe->hDfeCb[0]->regs->capture_buffer_a_2lsb - (uint32_t)(hDfe->objDfe.regs)) + 0x8000 * (i-1)/2;
		}
		else
		{
			cfgDescrip.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((uint32_t)&hDfe->hDfeCb[0]->regs->capture_buffer_a_16msb - (uint32_t)(hDfe->objDfe.regs)) + 0x8000 * i;
		}
		cfgDescrip.chanNum = iqnChnl;
		cfgDescrip.rw = DFE_FL_CPP_DMA_UL;
		cfgDescrip.numBytes = 32768; //8192*4
		cfgDescrip.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
		cfgDescrip.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
		cfgDescrip.pktSize = DFE_FL_CPP_DMA_PKT_SIZE_32K;
		cfgDescrip.midImm = 0;
		cfgDescrip.linkNext = 0; // change later
		dfeFl_CppDescripWrite(hDescrip[i], &cfgDescrip);

	}

    // build linklist
    for(i = 0; i < LinkLen-1; i++)
    {
        dfeFl_CppDescripLink(hDescrip[i], hDescrip[i+1]);
    }
    // end of linklist
    dfeFl_CppDescripLink(hDescrip[LinkLen-1], hDescrip[LinkLen-1]);

	// save resource allocated to context
    hDfe->flag_18bit = flag_18bit;
	hDfe->hDmaCb = hDma;
	hDfe->cbIqnChnl = iqnChnl;
	for(i = 0; i < LinkLen; i++)
	{
		hDfe->hDescripCb[i] = hDescrip[i];
	}

    return DFE_ERR_NONE;
}

/**
 * @brief Close CB Buf DMA
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * Close Cb Buf DMA and free resources allocated by Dfe_openCbBufDma(). 
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
 *  - Dfe_openCbBufDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_closeCbBufDma
(
    DFE_Handle hDfe
)
{
	//DfeFl_Status status;

	uint32_t LinkLen, i;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	// check if valid CPP/DMA opened
	if(hDfe->hDmaCb == NULL)
    {
        Dfe_osalLog("CPP/DMA handle not (Cb) valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;
    }

    dfeFl_CppDmaClose(hDfe->hDmaCb);

	if(hDfe->flag_18bit == 1)
		LinkLen = 8;
	else
		LinkLen = 4;

    // clean context
    hDfe->hDmaCb = NULL;
    for(i = 0; i < LinkLen; i++)
    {
    	dfeFl_CppDescripClose(hDfe->hDescripCb[i]);
    	hDfe->hDescripCb[i] = NULL;
    }

    hDfe->cbIqnChnl = DFE_FL_CPP_OPEN_NONE;

    return DFE_ERR_NONE;
}

/**
 * @brief Enable CB Buf DMA
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * Enable CPP/DMA for CB Buf by setting dma_ssel to sense MPU sync.
 * MPU sync will trigger CPP/DMA to start transferring.
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
 *  - Dfe_openCbBufDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_enableCbBufDma
(
	DFE_Handle hDfe
)
{
	DfeFl_Status status;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	// check if valid CPP/DMA opened
	if(hDfe->hDmaCb == NULL)
    {
        Dfe_osalLog("CPP/DMA handle not (cb) valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;
    }

	// Set dma_ssel to sense MPU
    CSL_HW_CTRL( dfeFl_CppDmaArm(hDfe->hDmaCb, dfeFl_CppDescripGetId(hDfe->hDescripCb[0]), DFE_FL_CPP_DMA_SSEL_GSYNC(DFE_FL_SYNC_GEN_SIG_MPU_SYNC)) );

	// issue sync
	return Dfe_issueSync(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_MISC_SYNC_NOWAIT);
}

/**
 * @brief Disable CB Buf DMA
 * @ingroup DFE_LLD_CB_FUNCTION
 *
 * Disable CB buf DMA by changing dma_ssel not to sense any signal
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
 *  - Dfe_openCbBufDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_disableCbBufDma
(
	DFE_Handle hDfe
)
{
	//DfeFl_Status status;

    if(hDfe == NULL)
    {
        Dfe_osalLog("hDfe is NULL!");
        return DFE_ERR_INVALID_HANDLE;
    }

	// check if valid CPP/DMA opened
	if(hDfe->hDmaCb == NULL)
    {
        Dfe_osalLog("CPP/DMA handle (cb) not valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;
    }


	// Set dma_ssel to sense NEVER
    dfeFl_CppDmaDismissSync(hDfe->hDmaCb);

    return DFE_ERR_NONE;
}
