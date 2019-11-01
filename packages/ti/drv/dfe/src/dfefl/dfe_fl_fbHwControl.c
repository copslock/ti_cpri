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

/** @file dfe_fl_fbHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_SummerHwControl()
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
 *   @n@b dfeFl_FbHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer     Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
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
 *   @n  The hardware registers of Dfe Summer.
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
         DfeFl_SublkInitsConfig inits;

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

         // reset Fb
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_FbHwControl(hDfeFb[0], DFE_FL_FB_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_FbHwControl
(
    DfeFl_FbHandle             hDfeFb,
    DfeFl_FbHwControlCmd       ctrlCmd,
    void                        *arg
)
{
	int i;
	DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {
    case DFE_FL_FB_CMD_CFG_INITS:
        dfeFl_FbConfigInits(hDfeFb, (DfeFl_SublkInitsConfig *)arg);
        break;

    case DFE_FL_FB_CMD_SET_INITS_CKENDLY:
    	dfeFl_FbSetInitsCkendly(hDfeFb, *(uint32_t *)arg);
    	break;
    /*
     *  Signal Generator and CheckSum
     */
    case DFE_FL_FB_CMD_SET_TESTBUS:
    {
    	dfeFl_FbSetTestbus(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_DC_TESTBUS:
    {
    	dfeFl_FbSetDcTestbus(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_SIGGEN_SSEL:
    {
    	dfeFl_FbSetSiggenSsel(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_CHKSUM_SSEL:
    {
    	dfeFl_FbSetChksumSsel(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_SIGGEN_MODE:
    {
    	dfeFl_FbSetSiggenMode(hDfeFb, (DfeFl_FbSiggenMode *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_CFG_SIGGEN_RAMP:
    {
    	dfeFl_FbCfgSiggenRamp(hDfeFb, (DfeFl_FbSiggenRampCfg *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_CFG_CHKSUM:
    {
    	dfeFl_FbCfgChksum(hDfeFb, (DfeFl_FbChksumCfg *)arg);
    	break;
    }

    /*
     * IO control
     */
    case DFE_FL_FB_CMD_SET_IO_CTRL:
    {
    	dfeFl_FbSetIOCtrl(hDfeFb, (DfeFl_FbIOCtrl *)arg);
    	break;
    }

    case DFE_FL_FB_CMD_SET_IO_MUX:
    {
    	dfeFl_FbSetIOMux(hDfeFb, (DfeFl_FbIOCtrl *)arg);
    	break;
    }
    /*
     * DC
     */
    case DFE_FL_FB_CMD_SET_DKACC_SSEL:
    {
    	dfeFl_FbSetDcSsel(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_DC_GSG_SSEL:
    {
    	dfeFl_FbSetDcGsgSsel(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_DC_GLOBAL:
    {
    	dfeFl_FbSetDcGlobal(hDfeFb, (DfeFl_FbDcGlobal *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_DC_INTERVAL:
    {
    	dfeFl_FbSetDcInterval(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_DC_DELAY:
    {
    	dfeFl_FbSetDcDelay(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_DC_SHIFT_MODE:
    {
    	dfeFl_FbSetDcShiftMode(hDfeFb, (DfeFl_FbDcShiftMode *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_DC_INIT:
    {
    	dfeFl_FbSetDcInit(hDfeFb, (DfeFl_FbDcInit *)arg);
    	break;
    }

    /*
     * r2c
     */
    case DFE_FL_FB_CMD_SET_R2C_SSEL:
    {
    	dfeFl_FbSetR2cSsel(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_R2C_REALIN:
    {
    	dfeFl_FbSetR2cRealin(hDfeFb, (DfeFl_FbR2cRealin *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_R2C_SPECINV:
    {
    	dfeFl_FbSetR2cSpecinv(hDfeFb, (DfeFl_FbR2cSpecinv *)arg);
    	break;
    }

    /*
     * EQR
     */
    case DFE_FL_FB_CMD_SET_EQR_SSEL:
    {
    	dfeFl_FbSetEqrSsel(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_EQR_TAPS:
    {
    	DfeFl_FbEqrTaps *cfg = (DfeFl_FbEqrTaps *)arg;
    	for (i = 0; i<cfg->numCoeff; i++)
    		dfeFl_FbSetEqrTaps(hDfeFb, cfg->blk, i, cfg->taps_ii[i], cfg->taps_iq[i], cfg->taps_qi[i], cfg->taps_qq[i]);
    	break;
    }

    /*
     * NCO
     */
    case DFE_FL_FB_CMD_SET_NCO_SSEL:
    {
    	dfeFl_FbSetNcoSsel(hDfeFb, (DfeFl_FbNcoSsel *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_NCO_BYPASS:
    {
    	dfeFl_FbSetNcoBypass(hDfeFb, (DfeFl_FbNcoBypass *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_NCO_FREQ:
    {
    	dfeFl_FbSetNcoFreq(hDfeFb, (DfeFl_FbNcoFreq *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_NCO_DITHER:
    {
    	dfeFl_FbSetNcoDither(hDfeFb, (DfeFl_FbNcoDither *)arg);
    	break;
    }

    /*
     * LUT
     */
    case DFE_FL_FB_CMD_SET_LUT_BYPASS:
    {
    	dfeFl_FbSetLutBypass(hDfeFb, (DfeFl_FbLutBypass *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_LUT_MEM:
    {
    	DfeFl_FbLut *cfg = (DfeFl_FbLut *)arg;

    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_FbSetLutMem(hDfeFb, i, *(cfg->dc+i), *(cfg->slope+i));
    	break;
    }

    /*
     * Decimation filter
     */
    case DFE_FL_FB_CMD_SET_DF_RATE:
    {
    	dfeFl_FbSetDfRate(hDfeFb, *(uint32_t *)arg);
    	break;
    }

    /*
     * Gain
     */
    case DFE_FL_FB_CMD_SET_GAIN_SSEL:
    {
    	dfeFl_FbSetGainSsel(hDfeFb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_FB_CMD_SET_GAIN_VAL:
    {
    	dfeFl_FbSetGainVal(hDfeFb, (DfeFl_FbGainVal *)arg);
    	break;
    }


    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}
