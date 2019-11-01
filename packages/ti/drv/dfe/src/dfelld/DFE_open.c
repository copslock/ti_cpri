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
#include <stdio.h>
#include <string.h>

#ifdef __LINUX_USER_SPACE__
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#endif // __LINUX_USER_SPACE__

#include <ti/drv/dfe/dfe_drv.h>
#include <ti/drv/dfe/dfe_osal.h>


/**
 * @brief Open DFE LLD device
 * @ingroup DFE_LLD_FUNCTION
 *
 * Open DFE low level driver device instance. Before call the API, the caller
 * should allocate the memory buffer for DFE_Obj, which will be the context
 * for the life of DFE LLD. The buffer shall not be allocated from stack. 
 *
 * Upon Dfe_open() succeeds, the error code is set to DFE_ERR_NONE.
 * - DFE LLD object dfeObj has been initialized.
 * - User defined resource management table has been saved to instance context.
 * - A valid DFE_Handle value is returned. 
 *
 * When the error code is DFE_ERR_INVALID_DEVICE, the LLD doesn’t find the specified
 * dfeInst device.
 *
 * The API should only be called once. 
 *
 * @b Arguments
 *
 * @param dfeInst	[in] DFE device instance number
 * @param dfeObj	[out] DFE LLD device context
 * @param dfeResTbl	[in] DFE resource management table 
 * @param baseAddress  [to be documented]
 * @param err	[out] exit error code
 *
 * @return A valid DFE_Handle value is returned when the call succeeds; otherwise a NULL is returned.
 *
 * @pre
 *   - dfeObj buffer shall not be allocated in stack.
 *	 - #Dfe_open() should be called only once.
 *
 * @post
 *   @n None
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Handle Dfe_open
(
    int dfeInst, 
    DFE_Obj *dfeObj, 
    DFE_CppResTbl *dfeResTbl,
    uint32_t baseAddress,
    DFE_Err *err
)
{
    int i;
    DfeFl_Status status = DFE_FL_SOK;
    
    
    if(dfeObj == NULL || dfeResTbl == NULL || err == NULL)
    {
        Dfe_osalLog("NULL pointers passed into Dfe_open()!");
        return NULL;
    }
    
	// open DFE
	dfeFl_Init(&dfeObj->dfeCtx);
	//dfeParam.flags = 0;
	dfeObj->hDfe = dfeFl_Open(&dfeObj->objDfe, dfeInst, &dfeObj->dfeParam, baseAddress, &status);
	if(status != DFE_FL_SOK)
	{
	    *err = DFE_ERR_INVALID_DEVICE;
	    Dfe_osalLog("dfeFl_Open() error = %d", status);
	    return NULL;
	}

    // open BB
    for(i = 0; i < DFE_FL_BB_PER_CNT; i++)
    {
    	dfeObj->hDfeBb[i] = dfeFl_BbOpen(dfeObj->hDfe, &dfeObj->objDfeBb[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_BbOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // open DDUC
    for(i = 0; i < DFE_FL_DDUC_PER_CNT; i++)
    {
    	dfeObj->hDfeDduc[i] = dfeFl_DducOpen(dfeObj->hDfe, &dfeObj->objDfeDduc[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_DducOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // open SUMMER
    for(i = 0; i < DFE_FL_SUMMER_PER_CNT; i++)
    {
    	dfeObj->hDfeSummer[i] = dfeFl_SummerOpen(dfeObj->hDfe, &dfeObj->objDfeSummer[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_SummerOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // open AUTOCP
    for(i = 0; i < DFE_FL_AUTOCP_PER_CNT; i++)
    {
    	dfeObj->hDfeAutocp[i] = dfeFl_AutocpOpen(dfeObj->hDfe, &dfeObj->objDfeAutocp[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_AutocpOpen() error = %d", status);
    	    return NULL;
    	}        
    }

    // open CFR
    for(i = 0; i < DFE_FL_CFR_PER_CNT; i++)
    {
    	dfeObj->hDfeCfr[i] = dfeFl_CfrOpen(dfeObj->hDfe, &dfeObj->objDfeCfr[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_CfrOpen() error = %d", status);
    	    return NULL;
    	}        
    }

    // open CDFR
    for(i = 0; i < DFE_FL_CDFR_PER_CNT; i++)
    {
    	dfeObj->hDfeCdfr[i] = dfeFl_CdfrOpen(dfeObj->hDfe, &dfeObj->objDfeCdfr[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_CdfrOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    if (dfeObj->dpdIsDisabled == 0)
    {
        // open DPD
        for(i = 0; i < DFE_FL_DPD_PER_CNT; i++)
        {
            dfeObj->hDfeDpd[i] = dfeFl_DpdOpen(dfeObj->hDfe, &dfeObj->objDfeDpd[i], i, &status);
            if(status != DFE_FL_SOK)
            {
                *err = DFE_ERR_INVALID_DEVICE;
                Dfe_osalLog("dfeFl_DpdOpen() error = %d", status);
                return NULL;
            }
        }
    }
    
    if (dfeObj->dpdaIsDisabled == 0)
    {
        // open DPDA
        for(i = 0; i < DFE_FL_DPDA_PER_CNT; i++)
        {
            dfeObj->hDfeDpda[i] = dfeFl_DpdaOpen(dfeObj->hDfe, &dfeObj->objDfeDpda[i], i, &status);
            if(status != DFE_FL_SOK)
            {
                *err = DFE_ERR_INVALID_DEVICE;
                Dfe_osalLog("dfeFl_DpdaOpen() error = %d", status);
                return NULL;
            }
        }
    }
    
    // open TX
    for(i = 0; i < DFE_FL_TX_PER_CNT; i++)
    {
    	dfeObj->hDfeTx[i] = dfeFl_TxOpen(dfeObj->hDfe, &dfeObj->objDfeTx[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_TxOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // open RX
    for(i = 0; i < DFE_FL_RX_PER_CNT; i++)
    {
    	dfeObj->hDfeRx[i] = dfeFl_RxOpen(dfeObj->hDfe, &dfeObj->objDfeRx[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_RxOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // open JESD
    for(i = 0; i < DFE_FL_JESD_PER_CNT; i++)
    {
    	dfeObj->hDfeJesd[i] = dfeFl_JesdOpen(dfeObj->hDfe, &dfeObj->objDfeJesd[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_JesdOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // open CB
    for(i = 0; i < DFE_FL_CB_PER_CNT; i++)
    {
    	dfeObj->hDfeCb[i] = dfeFl_CbOpen(dfeObj->hDfe, &dfeObj->objDfeCb[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_CbOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // open FB
    for(i = 0; i < DFE_FL_FB_PER_CNT; i++)
    {
    	dfeObj->hDfeFb[i] = dfeFl_FbOpen(dfeObj->hDfe, &dfeObj->objDfeFb[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_FbOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // open MISC
    for(i = 0; i < DFE_FL_MISC_PER_CNT; i++)
    {
    	dfeObj->hDfeMisc[i] = dfeFl_MiscOpen(dfeObj->hDfe, &dfeObj->objDfeMisc[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    	    *err = DFE_ERR_INVALID_DEVICE;
    	    Dfe_osalLog("dfeFl_MiscOpen() error = %d", status);
    	    return NULL;
    	}        
    }
    
    // reserved dma bitmask
    dfeObj->cppResMgr.dmaRsvd = dfeResTbl->dmaRsvd;
    dfeObj->cppResMgr.dmaOpened = 0u;
    
    // discrete trigger out
    for(i = 0; i < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS; i++)
    {
        dfeObj->cppResMgr.discreteTrig[i] = dfeResTbl->discreteTrig[i];
    }
    // four 32-bits words, each bit corresponding to one descriptor
    // reserved descriptor bitmask
    for(i = 0; i < 4; i++)
    {
        dfeObj->cppResMgr.descripRsvd[i] = dfeResTbl->descripRsvd[i];
        dfeObj->cppResMgr.descripOpened[i] = 0u;
    }
    
    // dma handle for BBTX power meter
    dfeObj->hDmaBbtxPowmtr = NULL;
    // descriptor handle for BBTX power meter
    dfeObj->hDescripBbtxPowmtr = NULL;
    // IQN2 CTL Ingress Channel for BBTX power meter
    dfeObj->bbtxPowmtrIqnChnl = DFE_FL_CPP_OPEN_NONE;
    
    // dma handle for BBRX power meter
    dfeObj->hDmaBbrxPowmtr = NULL;
    // descriptor handle for BBRX power meter
    dfeObj->hDescripBbrxPowmtr = NULL;
    // descriptor handle for BBTX power meter
    dfeObj->bbrxPowmtrIqnChnl = DFE_FL_CPP_OPEN_NONE;

    // CB flag
    dfeObj->flag_18bit = 0;
    // dma handle for CB
    dfeObj->hDmaCb = NULL;
    // descriptor handle for CB
    for(i = 0; i < 8; i++)
    {
    	dfeObj->hDescripCb[i] = NULL;
    }
    // descriptor handle for BBTX power meter
    dfeObj->cbIqnChnl = DFE_FL_CPP_OPEN_NONE;

    // RX IBPM Unity Magnitude Sqaure value (=I^2 + Q^2)
    dfeObj->rxIbpmUnityMagsq = 0ll;
    
    // generic DMA hndle
    dfeObj->hDmaGeneric = NULL;
    // IQN2 CTL Egress channel for generic writing DMA
    dfeObj->genericDmaIqnChnlDl = DFE_FL_CPP_OPEN_NONE;
    // IQN2 CTL Ingress channel for generic reading DMA
    dfeObj->genericDmaIqnChnlUl = DFE_FL_CPP_OPEN_NONE;
        
    *err = DFE_ERR_NONE;
    return dfeObj;	
}

/**
 * @brief Close DFE LLD device.
 * @ingroup DFE_LLD_FUNCTION
 *
 * The DFE low level driver device instance, previously opened by #Dfe_open() is closed. 
 *
 * @param hDfe	[in] DFE device handle
 *
 * @return
 * - #DFE_ERR_NONE, if close properly
 * - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *
 * @pre
 *   - hDfe should be a valid handle opened by #Dfe_open().
 *
 * @post
 *   @n None
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_close
(
    DFE_Handle hDfe
)
{
    if(hDfe == NULL)
    {
        return DFE_ERR_INVALID_HANDLE;
    }

    return DFE_ERR_NONE;

}
