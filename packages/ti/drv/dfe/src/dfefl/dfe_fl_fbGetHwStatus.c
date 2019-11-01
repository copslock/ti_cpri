/********************************************************************
* Copyright (C) 2012-2013 Texas Instruments Incorporated.
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

/** @file dfe_fl_fbGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_SummerGetHwStatus()
 *
 *
 */
/* =============================================================================
 * Revision History
 * ===============
 *
 *
 * =============================================================================
 */
#include <ti/drv/dfe/dfe_fl_fbAux.h>

/** ============================================================================
 *   @n@b dfeFl_FbGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Summer module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer     Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_SummerOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_FbObj objDfeFb[DFE_FL_FB_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_FbHandle hDfeFb[DFE_FL_FB_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;
         DfeFl_FbIOCtrl FbIOCtrl;

         // open DFE
         dfeFl_Init(&dfeCtx);
         dfeParam.flags = 0;
         hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, &status);
         if(status != DFE_FL_SOK)
         {
         	 return FAIL;
         }

         for(i = 0; i < DFE_FL_FB_PER_CNT; i++)
         {
		 	 hDfeFb[i] = dfeFl_FbOpen(hDfe, &objDfeFb[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // check the FB IO Control setup
         dfeFl_FbGetHwStatus(hDfeFb[0], DFE_FL_FB_QUERY_GET_IO_CONTROL, &FbIOCtrl);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_FbGetHwStatus
(
    DfeFl_FbHandle             hDfeFb,
    DfeFl_FbHwStatusQuery      queryId,
    void                        *arg
)
{
	uint32_t i;
	DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    case DFE_FL_FB_QUERY_GET_INITS:
        dfeFl_FbGetInits(hDfeFb, (DfeFl_SublkInitsConfig *)arg);
        break;

    case DFE_FL_FB_QUERY_GET_INITS_CKENDLY:
    	dfeFl_FbGetInitsCkendly(hDfeFb, (uint32_t *)arg);
    	break;
    /*
     * Signal Generator and CheckSum
     */
    case DFE_FL_FB_QUERY_GET_TESTBUS:
    {
    	dfeFl_FbGetTestbus(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_DC_TESTBUS:
    {
    	dfeFl_FbGetDcTestbus(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_CHKSUM_RESULT:
    {
    	dfeFl_FbGetChksumResult(hDfeFb, (uint32_t *)arg);
    	break;
    }

	/*
	 * IO Control
	 */
    case DFE_FL_FB_QUERY_GET_IO_CONTROL:
    {
    	dfeFl_FbQueryGetIOControl(hDfeFb, (DfeFl_FbIOCtrl *)arg);
    }

    /*
     * DC
     */
    case DFE_FL_FB_QUERY_GET_DKACC_SSEL:
    {
    	dfeFl_FbGetDcSsel(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_DC_GSG_SSEL:
    {
    	dfeFl_FbGetDcGsgSsel(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_DC_GLOBAL:
    {
    	dfeFl_FbGetDcGlobal(hDfeFb, (DfeFl_FbDcGlobal *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_DC_INTERVAL:
    {
    	dfeFl_FbGetDcInterval(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_DC_DELAY:
    {
    	dfeFl_FbGetDcDelay(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_DC_SHIFT_MODE:
    {
    	dfeFl_FbGetDcShiftMode(hDfeFb, (DfeFl_FbDcShiftMode *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_DC_INIT:
    {
    	dfeFl_FbGetDcInit(hDfeFb, (DfeFl_FbDcInit *)arg);
    	break;
    }

    /*
     * r2c
     */
    case DFE_FL_FB_QUERY_GET_R2C_SSEL:
    {
    	dfeFl_FbGetR2cSsel(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_R2C_REALIN:
    {
    	dfeFl_FbGetR2cRealin(hDfeFb, (DfeFl_FbR2cRealin *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_R2C_SPECINV:
    {
    	dfeFl_FbGetR2cSpecinv(hDfeFb, (DfeFl_FbR2cSpecinv *)arg);
    	break;
    }

    /*
     * EQR
     */
    case DFE_FL_FB_QUERY_GET_EQR_SSEL:
    {
    	dfeFl_FbGetEqrSsel(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_EQR_TAPS:
    {
    	DfeFl_FbEqrTaps *cfg = (DfeFl_FbEqrTaps *)arg;
    	for (i = 0; i<cfg->numCoeff; i++)
    		dfeFl_FbGetEqrTaps(hDfeFb, cfg->blk, i, &cfg->taps_ii[i], &cfg->taps_iq[i], &cfg->taps_qi[i], &cfg->taps_qq[i]);
    	break;
    }

	/*
	 * NCO
	 */
    case DFE_FL_FB_QUERY_GET_NCO_SSEL:
    {
    	dfeFl_FbGetNcoSsel(hDfeFb, (DfeFl_FbNcoSsel *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_NCO_BYPASS:
    {
    	dfeFl_FbGetNcoBypass(hDfeFb, (DfeFl_FbNcoBypass *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_NCO_FREQ:
    {
    	dfeFl_FbGetNcoFreq(hDfeFb, (DfeFl_FbNcoFreq *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_NCO_DITHER:
    {
    	dfeFl_FbGetNcoDither(hDfeFb, (DfeFl_FbNcoDither *)arg);
    	break;
    }

    /*
     * LUT
     */
    case DFE_FL_FB_QUERY_GET_LUT_BYPASS:
    {
    	dfeFl_FbGetLutBypass(hDfeFb, (DfeFl_FbLutBypass *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_LUT_MEM:
    {
    	DfeFl_FbLut *cfg = (DfeFl_FbLut *)arg;

    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_FbGetLutMem(hDfeFb, i, (cfg->dc+i), (cfg->slope+i));
    	break;
    }

    /*
     * Decimation filter
     */
    case DFE_FL_FB_QUERY_GET_DF_RATE:
    {
    	dfeFl_FbGetDfRate(hDfeFb, (uint32_t *)arg);
    	break;
    }

    /*
     * Gain
     */
    case DFE_FL_FB_QUERY_GET_GAIN_SSEL:
    {
    	dfeFl_FbGetGainSsel(hDfeFb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_QUERY_GET_GAIN_VAL:
    {
    	dfeFl_FbGetGainVal(hDfeFb, (DfeFl_FbGainVal *)arg);
    	break;
    }
        
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
