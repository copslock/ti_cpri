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

/** @file dfe_fl_rxHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_RxHwControl()
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
#include <ti/drv/dfe/dfe_fl_rxAux.h>
 
/** ============================================================================
 *   @n@b dfeFl_RxHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx         Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_RxOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  The hardware registers of Dfe Rx.
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_RxObj objDfeRx[DFE_FL_RX_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_RxHandle hDfeRx[DFE_FL_RX_PER_CNT];
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

         for(i = 0; i < DFE_FL_RX_PER_CNT; i++)
         {
		 	 hDfeRx[i] = dfeFl_RxOpen(hDfe, &objDfeRx[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // reset RX
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_RxHwControl(hDfeRx[0], DFE_FL_RX_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_RxHwControl
(
    DfeFl_RxHandle             hDfeRx,
    DfeFl_RxHwControlCmd       ctrlCmd,
    void                        *arg
)
{
    DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {
    case DFE_FL_RX_CMD_CFG_INITS:
        dfeFl_RxConfigInits(hDfeRx, (DfeFl_SublkInitsConfig *)arg);
        break;
    /*
     *  Signal Generator and CheckSum
     */
    case DFE_FL_RX_CMD_SET_SIGGEN_SSEL:
    {
        dfeFl_RxSetSiggenSsel(hDfeRx, *(uint32_t *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_SET_CHKSUM_SSEL:
    {
        dfeFl_RxSetChksumSsel(hDfeRx, *(uint32_t *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_SET_SIGGEN_MODE:
    {
        dfeFl_RxSetSiggenMode(hDfeRx, (DfeFl_RxSiggenMode *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_CFG_SIGGEN_RAMP:
    {
        dfeFl_RxConfigSiggenRamp(hDfeRx, (DfeFl_RxSiggenRampConfig *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_CFG_CHKSUM:
    {
        dfeFl_RxConfigChksum(hDfeRx, (DfeFl_RxChksumConfig *)arg);
        break;
    }
    /*
     * Test Bus
     */
    case DFE_FL_RX_CMD_SET_TOP_TEST_CTRL:
    {
    	dfeFl_RxSetTopTestCtrl(hDfeRx, *(DfeFl_RxTestCtrl *)arg);
    	break;
    }
    case DFE_FL_RX_CMD_SET_IMB_TEST_CTRL:
    {
    	dfeFl_RxSetImbTestCtrl(hDfeRx, *(DfeFl_RxImbTestCtrl *)arg);
    	break;
    }
    case DFE_FL_RX_CMD_SET_FEAGC_DC_TEST_CTRL:
    {
    	dfeFl_RxSetFeagcDcTestCtrl(hDfeRx, *(DfeFl_RxFeagcDcTestCtrl *)arg);
    	break;
    }
    
    /*
     *  Power meter
     */
    case DFE_FL_RX_CMD_ENB_POWMTR_INTR:
    {
        dfeFl_RxEnablePowmtrInterrupt(hDfeRx, *(uint32_t *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_DIS_POWMTR_INTR:
    {
        dfeFl_RxDisablePowmtrInterrupt(hDfeRx, *(uint32_t *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_SET_FORCE_POWMTR_INTR:
    {
        dfeFl_RxSetForcePowmtrInterrupt(hDfeRx, *(uint32_t *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_CLR_FORCE_POWMTR_INTR:
    {
        dfeFl_RxClearForcePowmtrInterrupt(hDfeRx, *(uint32_t *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_CLR_POWMTR_INTR_STATUS:
    {
        dfeFl_RxClearPowmtrInterruptStatus(hDfeRx, *(uint32_t *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_SET_POWMTR_SSEL:
    {
        dfeFl_RxSetPowmtrSsel(hDfeRx, (DfeFl_RxPowmtrGeneric *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_CFG_POWMTR_GLOBAL:
    {
        dfeFl_RxConfigPowmtrGlobal(hDfeRx, (DfeFl_RxPowmtrGlobalConfig *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_SET_POWMTR_HANDSHAKE_DONE:
    {
        dfeFl_RxSetPowmtrHandshakeDone(hDfeRx, (DfeFl_RxPowmtrGeneric *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_SET_POWMTR_HANDSHAKE_READ_REQ:
    {
        dfeFl_RxSetPowmtrHandshakeReadReq(hDfeRx, (DfeFl_RxPowmtrGeneric *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_CFG_POWMTR:
    {
        dfeFl_RxConfigPowmtr(hDfeRx, (DfeFl_RxPowmtrConfig *)arg);
        break;
    }
    case DFE_FL_RX_CMD_SET_ONE_SHOT_MODE:
    {
    	dfeFl_RxSetOneShotMode(hDfeRx, (DfeFl_RxPowmtrGeneric *)arg);
    	break;
    }
    case DFE_FL_RX_CMD_SET_METER_MODE:
    {
    	dfeFl_RxSetMeterMode(hDfeRx, (DfeFl_RxPowmtrGeneric *)arg);
    	break;
    }
    /*
     * RX Switch
     */
    case DFE_FL_RX_CMD_SET_SWITCH_BYPASS:
    {
    	dfeFl_RxSetSwitchBypass(hDfeRx, *(uint32_t *)arg);
    }
        
    /*
     *  Rx NCO
     */
    case DFE_FL_RX_CMD_SET_NCO_SSEL:
    {
        dfeFl_RxSetNcoSsel(hDfeRx, (DfeFl_RxNcoSsel *)arg);
        break;
    }
    case DFE_FL_RX_CMD_SET_NCO_BYPASS:
    {
        dfeFl_RxSetNcoBypass(hDfeRx, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_RX_CMD_ENB_NCO_DITHER:
    {
        dfeFl_RxEnableNcoDither(hDfeRx, (DfeFl_RxNcoDitherConfig *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_CFG_NCO_FREQ:
    {
        dfeFl_RxConfigNcoFreq(hDfeRx, (DfeFl_RxNcoFreqWord *)arg);
        break;
    }        
     
    /*
     *  Rx EQR
     */
    case DFE_FL_RX_CMD_SET_EQR_SSEL:
    {
        dfeFl_RxSetEqrSsel(hDfeRx, (DfeFl_RxEqrGeneric *)arg);
        break;
    }
    case DFE_FL_RX_CMD_SET_EQR_BYPASS:
    {
        dfeFl_RxSetEqrBypass(hDfeRx, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_RX_CMD_CFG_EQR_TAPS:
    {
        dfeFl_RxConfigEqrTaps(hDfeRx, (DfeFl_RxEqrTapsConfig *)arg);
        break;
    }        
    case DFE_FL_RX_CMD_SET_EQR_SHIFT:
    {
        dfeFl_RxSetEqrShift(hDfeRx, (DfeFl_RxEqrGeneric *)arg);
        break;
    }        
    
    case DFE_FL_RX_CMD_SET_DC_GSG_SSEL:
    {
        dfeFl_RxSetDcGsgSsel(hDfeRx, *(uint32_t *)arg);
        break;
    }    
    case DFE_FL_RX_CMD_SET_DKACC_SSEL:
    {
        DfeFl_RxDcGeneric *cfg = (DfeFl_RxDcGeneric *)arg;
        
        dfeFl_RxSetDkaccSsel(hDfeRx, cfg->dc, cfg->data);
        break;
    }        
        
    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}
