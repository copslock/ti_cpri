/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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

#include <ti/csl/csl.h>
//#include <val_util.h>
#include <constants.h>
#include <ti/csl/csl_chip.h>

#include "dfe_inc.h"

DfeFl_Handle hDfe;
DfeFl_BbHandle hDfeBb[DFE_FL_BB_PER_CNT];
DfeFl_DducHandle hDfeDduc[DFE_FL_DDUC_PER_CNT];
DfeFl_SummerHandle hDfeSummer[DFE_FL_SUMMER_PER_CNT];
DfeFl_AutocpHandle hDfeAutocp[DFE_FL_AUTOCP_PER_CNT];
DfeFl_CfrHandle hDfeCfr[DFE_FL_CFR_PER_CNT];
DfeFl_CdfrHandle hDfeCdfr[DFE_FL_CDFR_PER_CNT];
DfeFl_DpdHandle hDfeDpd[DFE_FL_DPD_PER_CNT];
DfeFl_DpdaHandle hDfeDpda[DFE_FL_DPDA_PER_CNT];
DfeFl_TxHandle hDfeTx[DFE_FL_TX_PER_CNT];
DfeFl_RxHandle hDfeRx[DFE_FL_RX_PER_CNT];
DfeFl_CbHandle hDfeCb[DFE_FL_CB_PER_CNT];
DfeFl_JesdHandle hDfeJesd[DFE_FL_JESD_PER_CNT];
DfeFl_FbHandle hDfeFb[DFE_FL_FB_PER_CNT];
DfeFl_MiscHandle hDfeMisc[DFE_FL_MISC_PER_CNT];

DfeFl_Obj     objDfe;
DfeFl_BbObj objDfeBb[DFE_FL_BB_PER_CNT];
DfeFl_DducObj objDfeDduc[DFE_FL_DDUC_PER_CNT];
DfeFl_SummerObj objDfeSummer[DFE_FL_SUMMER_PER_CNT];
DfeFl_AutocpObj objDfeAutocp[DFE_FL_AUTOCP_PER_CNT];
DfeFl_CfrObj objDfeCfr[DFE_FL_CFR_PER_CNT];
DfeFl_CdfrObj objDfeCdfr[DFE_FL_CDFR_PER_CNT];
DfeFl_DpdObj objDfeDpd[DFE_FL_DPD_PER_CNT];
DfeFl_DpdaObj objDfeDpda[DFE_FL_DPDA_PER_CNT];
DfeFl_TxObj objDfeTx[DFE_FL_TX_PER_CNT];
DfeFl_RxObj objDfeRx[DFE_FL_RX_PER_CNT];
DfeFl_CbObj objDfeCb[DFE_FL_CB_PER_CNT];
DfeFl_JesdObj objDfeJesd[DFE_FL_JESD_PER_CNT];
DfeFl_FbObj objDfeFb[DFE_FL_FB_PER_CNT];
DfeFl_MiscObj objDfeMisc[DFE_FL_MISC_PER_CNT];

DfeFl_Context dfeCtx;
DfeFl_Param   dfeParam;

static Uint32 result;
Uint32 *g_testResultCode = &result;
extern uint32_t dfe_cfg_base;

int openDfe_local()
{
    int i;
    DfeFl_Status status = DFE_FL_SOK;
    
	// open DFE
	dfeFl_Init(&dfeCtx);
	dfeParam.flags = 0;
	hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, dfe_cfg_base /*CSL_DFE_CFG_REGS*/, &status);
	if(status != DFE_FL_SOK)
	{
		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_DFE);
	    return FAIL;
	}

    // open BB
    for(i = 0; i < DFE_FL_BB_PER_CNT; i++)
    {
    	hDfeBb[i] = dfeFl_BbOpen(hDfe, &objDfeBb[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_BB);
    		return FAIL;
    	}        
    }
    
    // open DDUC
    for(i = 0; i < DFE_FL_DDUC_PER_CNT; i++)
    {
    	hDfeDduc[i] = dfeFl_DducOpen(hDfe, &objDfeDduc[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_DDUC0+i);
    		return FAIL;
    	}        
    }
    
    // open SUMMER
    for(i = 0; i < DFE_FL_SUMMER_PER_CNT; i++)
    {
    	hDfeSummer[i] = dfeFl_SummerOpen(hDfe, &objDfeSummer[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_SUMMER);
    		return FAIL;
    	}        
    }
    
    // open AUTOCP
    for(i = 0; i < DFE_FL_AUTOCP_PER_CNT; i++)
    {
    	hDfeAutocp[i] = dfeFl_AutocpOpen(hDfe, &objDfeAutocp[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_AUTOCP);
    		return FAIL;
    	}        
    }

    // open CFR
    for(i = 0; i < DFE_FL_CFR_PER_CNT; i++)
    {
    	hDfeCfr[i] = dfeFl_CfrOpen(hDfe, &objDfeCfr[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CFR0+i);
    		return FAIL;
    	}        
    }

    // open CDFR
    for(i = 0; i < DFE_FL_CDFR_PER_CNT; i++)
    {
    	hDfeCdfr[i] = dfeFl_CdfrOpen(hDfe, &objDfeCdfr[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CDFR);
    		return FAIL;
    	}        
    }
    
    // open DPD
    for(i = 0; i < DFE_FL_DPD_PER_CNT; i++)
    {
    	hDfeDpd[i] = dfeFl_DpdOpen(hDfe, &objDfeDpd[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_DPD);
    		return FAIL;
    	}        
    }
    
    // open DPDA
    for(i = 0; i < DFE_FL_DPDA_PER_CNT; i++)
    {
    	hDfeDpda[i] = dfeFl_DpdaOpen(hDfe, &objDfeDpda[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_DPDA);
    		return FAIL;
    	}        
    }
    
    // open TX
    for(i = 0; i < DFE_FL_TX_PER_CNT; i++)
    {
    	hDfeTx[i] = dfeFl_TxOpen(hDfe, &objDfeTx[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_TX);
    		return FAIL;
    	}        
    }
    
    // open RX
    for(i = 0; i < DFE_FL_RX_PER_CNT; i++)
    {
    	hDfeRx[i] = dfeFl_RxOpen(hDfe, &objDfeRx[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_RX);
    		return FAIL;
    	}        
    }
    
    // open JESD
    for(i = 0; i < DFE_FL_JESD_PER_CNT; i++)
    {
    	hDfeJesd[i] = dfeFl_JesdOpen(hDfe, &objDfeJesd[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_JESD);
    		return FAIL;
    	}        
    }
    
    // open CB
    for(i = 0; i < DFE_FL_CB_PER_CNT; i++)
    {
    	hDfeCb[i] = dfeFl_CbOpen(hDfe, &objDfeCb[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_CB);
    		return FAIL;
    	}        
    }
    
    // open FB
    for(i = 0; i < DFE_FL_FB_PER_CNT; i++)
    {
    	hDfeFb[i] = dfeFl_FbOpen(hDfe, &objDfeFb[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_FB);
    		return FAIL;
    	}        
    }
    
    // open MISC
    for(i = 0; i < DFE_FL_MISC_PER_CNT; i++)
    {
    	hDfeMisc[i] = dfeFl_MiscOpen(hDfe, &objDfeMisc[i], i, &status);
    	if(status != DFE_FL_SOK)
    	{
    		SET_TEST_RESULT_CODE(TEST_RESULT_CODE_FAIL_OPEN_MISC);
    		return FAIL;
    	}        
    }

    return PASS;    
}
