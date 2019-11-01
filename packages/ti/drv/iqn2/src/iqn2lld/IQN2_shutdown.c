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

#include <string.h>
#include <stdio.h>

#include <ti/csl/csl.h>

#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>
#include <ti/drv/iqn2/iqn2_osal.h>

#include <ti/drv/iqn2/include/IQN2_defs.h>
#include <ti/drv/iqn2/include/IQN2_runtime.h>

#define __IQN2_SHUTDOWN_C
#include <ti/drv/iqn2/include/IQN2_shutdown.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(IQN2_resetAt2, ".text:iqn2Init");
#pragma CODE_SECTION(IQN2_resetIqn2, ".text:iqn2Init"); 
#pragma CODE_SECTION(IQN2_cleanMMRs, ".text:iqn2Init");
#pragma DATA_SECTION(Iqn2Obj, ".far:iqn2Init_dat");
#endif

static Iqn2Fl_Obj                 Iqn2Obj;

void IQN2_cleanMMRs(IQN2_ConfigHandle  hIqn2);

// This function is used to disable events and stop timers
void IQN2_resetAt2(
        IQN2_ConfigHandle    hIqn2,
        Iqn2Fl_InitCfg*      hIqn2BaseAddr
)
{
    uint32_t ctrlArg;
    Iqn2Fl_Status iqn2Status;
    /* To be tested in future release */
    hIqn2->hFl = Iqn2Fl_open(&Iqn2Obj, CSL_IQN, hIqn2BaseAddr, &iqn2Status);  //CSL_IQN is set instead of previously 0
    if ((hIqn2->hFl == NULL) || (iqn2Status != IQN2FL_SOK))
    {
       Iqn2_osalLog ("\nError opening CSL_IQN");
       exit(1);
    }
    // Disable all AT2 events
    ctrlArg = 0x00000000;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG, (void *)&ctrlArg);
    // Disable all RADTs
    ctrlArg = 0x00000000;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN, (void *)&ctrlArg);
    // Disable BCN
    ctrlArg = 0x00000000;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RUN_BCN, (void *)&ctrlArg);

    //if (hIqn2->hFl != NULL) {Iqn2Fl_close(hIqn2->hFl); hIqn2->hFl = NULL;}

}

// This function is used to reset IQN2 to its default state.
/*
 * The VC_SW_RESET_STB MMR gives the programmer the ability to reset specific sections of the IQN.
 * The SW reset feature will typically be used in initial bringup and debug, not typically in normal operation.
 * The following endpoints will each have one bit associated with it�s software reset:
 *   AIL_0: PHY
 *   AIL_1: PHY
 *   AID2
 *   DIO
 *   PktDMA
 *   SW RESET resets all of the above with one bit.
 * Great care must be taken when implementing any of these software reset. All VBUS_DATA or VBUS_CFG must be stopped first.
 * CAUTION with PktDMA:
 *   All Tx and Rx channels must be teared down, disabled and closed first using Cppi_channelTeardown()/Cppi_channelDisable()/Cppi_channelClose()
 *   PktDMA must be closed using Cppi_close()
 */
void IQN2_resetIqn2(
    IQN2_ConfigHandle    hIqn2,
    Iqn2Fl_InitCfg*      hIqn2BaseAddr
)
{
    uint32_t                    ctrlArg, j;
    Iqn2Fl_Status               iqn2Status;
    Iqn2Fl_TopVCSwResetStbSetup reset;

    hIqn2->hFl = Iqn2Fl_open(&Iqn2Obj, CSL_IQN, hIqn2BaseAddr, &iqn2Status);
    if ((hIqn2->hFl == NULL) || (iqn2Status != IQN2FL_SOK))
    {
       Iqn2_osalLog ("\nError opening CSL_IQN");
       exit(1);
    }
    //AT2 disable all events and halt timer
    IQN2_resetAt2(hIqn2,hIqn2BaseAddr);
    // Stop  the DIO2 engines
    IQN2_dio2DisableEngines(hIqn2);
    // Disable DIO2
    ctrlArg = 0x55555555;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
    // Disable AIL0 and AIL1
    for (j=0; j<IQN2_MAX_NUM_AIL; j++)
    {
        ctrlArg = 0xAAAAAAAA;
        hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance) j;
        //global disable of AIL0/1 (Egress, Ingress)
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
    }

    // Disable AID2
    if(hIqn2->aidConfig.aidEnable)
    {
		ctrlArg = 0x55555555;
		//global disable of AID (Egress, Ingress)
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_CLEAR, (void *)&ctrlArg);
    }

    // Global SW reset IQN2
    memset(&reset,0x00,sizeof(Iqn2Fl_TopVCSwResetStbSetup));
    reset.sw_reset =1;//IQN2 full reset
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB, (void *)&reset);

    if (hIqn2->hIqn2Setup != NULL) IQN2_cleanMMRs(hIqn2);

    if (hIqn2->hFl != NULL) {Iqn2Fl_close(hIqn2->hFl); hIqn2->hFl = NULL;}

}

void IQN2_cleanMMRs(
		IQN2_ConfigHandle  hIqn2
)
{
    /* To be implemented in future release if needed by Iqn2 reset  */
	if (hIqn2) {

	}
}


////////////////////
