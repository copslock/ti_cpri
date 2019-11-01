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

/** @file dfe_fl_dpdaHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_DpdaHwControl()
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
#include <ti/drv/dfe/dfe_fl_dpdaAux.h>

/** ============================================================================
 *   @n@b dfeFl_DpdaHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpda       Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdaOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  The hardware registers of Dfe Dpda.
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_DpdaObj objDfeDpda[DFE_FL_DPDA_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_DpdaHandle hDfeDpda[DFE_FL_DPDA_PER_CNT];
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

         for(i = 0; i < DFE_FL_DPDA_PER_CNT; i++)
         {
		 	 hDfeDpda[i] = dfeFl_DpdaOpen(hDfe, &objDfeDpda[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // reset Dpda
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_DpdaHwControl(hDfeDpda[0], DFE_FL_DPDA_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_DpdaHwControl
(
    DfeFl_DpdaHandle         hDfeDpda,
    DfeFl_DpdaHwControlCmd   ctrlCmd,
    void                        *arg
)
{
    uint32_t i;
    DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {   
    case DFE_FL_DPDA_CMD_CFG_INITS:
        dfeFl_DpdaConfigInits(hDfeDpda, (DfeFl_SublkInitsConfig *)arg);
        break;
	/**
	 * interrupt
	 */
    case DFE_FL_DPDA_CMD_ENB_INT_READ_COMPLETE_INTR:
    {
    	dfeFl_DpdaEnableIntreadcompleteIntr(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_DIS_INT_READ_COMPLETE_INTR:
    {
    	dfeFl_DpdaDisableIntreadcompleteIntr(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_FORCE_INT_READ_COMPLETE_INTR:
    {
    	dfeFl_DpdaSetForceIntreadcompleteIntr(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_CLR_FORCE_INT_READ_COMPLETE_INTR:
    {
    	dfeFl_DpdaClearForceIntreadcompleteIntr(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_CLR_INT_READ_COMPLETE_INTR_STATUS:
    {
    	dfeFl_DpdaClearIntreadcompleteIntrStatus(hDfeDpda);
    	break;
    }

    case DFE_FL_DPDA_CMD_ENB_INT_PROCESSED_INTR:
    {
    	dfeFl_DpdaEnableIntprocessedIntr(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_DIS_INT_PROCESSED_INTR:
    {
    	dfeFl_DpdaDisableIntprocessedIntr(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_FORCE_INT_PROCESSED_INTR:
    {
    	dfeFl_DpdaSetForceIntprocessedIntr(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_CLR_FORCE_INT_PROCESSED_INTR:
    {
    	dfeFl_DpdaClearForceIntprocessedIntr(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_CLR_INT_PROCESSED_INTR_STATUS:
    {
    	dfeFl_DpdaClearIntprocessedIntrStatus(hDfeDpda);
    	break;
    }
    case DFE_FL_DPDA_CMD_CLR_IDLE_INTR_STATUS:
    {
      dfeFl_DpdaClearIdleIntrStatus(hDfeDpda);
      break;
    }
     /**
	 * main control
	 */
    case DFE_FL_DPDA_CMD_SET_DSP_INTR:
    {
    	dfeFl_DpdaSetDspIntr(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_DSP_ANT_EN:
    {
    	dfeFl_DpdaSetDspAntEn(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_JACOB_P2LSCALE:
    {
    	dfeFl_DpdaSetJacobp2lscale(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_JACOB_INPUTSCALE:
    {
    	dfeFl_DpdaSetJacobInputscale(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_EXP_FP2I:
    {
    	dfeFl_DpdaSetExpFp2i(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_EXP_I2FP:
    {
    	dfeFl_DpdaSetExpI2fp(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_INTR_ADDRESS_DPD:
    {
    	dfeFl_DpdaSetIntrAddressDpd(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_ANT_ENABLED_DPD:
    {
    	dfeFl_DpdaSetAntEnabledDpd(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_NEW_INT_DPD:
    {
    	dfeFl_DpdaSetNewIntDpd(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_PARAM1_DPD:
    {
    	dfeFl_DpdaSetParam1Dpd(hDfeDpda, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_PARAM2_DPD:
    {
    	dfeFl_DpdaSetParam2Dpd(hDfeDpda, *(uint32_t *)arg);
    	break;
    }

    /**
     * Ig_reg
     */
    case DFE_FL_DPDA_CMD_SET_IGREG:
    {
    	DfeFl_DpdaGeneric *cfg = (DfeFl_DpdaGeneric *)arg;
    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_DpdaSetIgreg(hDfeDpda, i, *(cfg->data+i));
    	break;
    }

    /**
     * scalar
     */
    case DFE_FL_DPDA_CMD_SET_SCALAR:
    {
    	DfeFl_DpdaScalar *cfg = (DfeFl_DpdaScalar *)arg;
    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_DpdaSetScalar(hDfeDpda, i, *(cfg->iedata+i), *(cfg->qdata+i));
    	break;
    }
        
	/**
	 * lut
	 */
    case DFE_FL_DPDA_CMD_SET_LUT_MASTER:
    {
    	DfeFl_DpdaGeneric *cfg = (DfeFl_DpdaGeneric *)arg;
    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_DpdaSetLutMaster(hDfeDpda, i, *(cfg->data+i));
    	break;
    }
    case DFE_FL_DPDA_CMD_SET_LUT:
    {
    	DfeFl_DpdaLut *cfg = (DfeFl_DpdaLut *)arg;
    	for (i=0; i<cfg->lut.numEntry; i++)
    		dfeFl_DpdaSetLut(hDfeDpda, cfg->idx, i, *(cfg->lut.data+i));
    	break;
    }

	/*
	 * iram
	 */
    case DFE_FL_DPDA_CMD_SET_IRAM:
    {
    	DfeFl_DpdaGeneric *cfg = (DfeFl_DpdaGeneric *)arg;
    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_DpdaSetIram(hDfeDpda, i, *(cfg->data+i));
    	break;
    }

	/*
	 * preg
	 */
    case DFE_FL_DPDA_CMD_SET_PREG:
    {
    	DfeFl_DpdaPreg *cfg = (DfeFl_DpdaPreg *)arg;
    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_DpdaSetPreg(hDfeDpda, cfg->idx, i, *(cfg->iedata+i), *(cfg->qdata+i));
    	break;
    }

    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}
