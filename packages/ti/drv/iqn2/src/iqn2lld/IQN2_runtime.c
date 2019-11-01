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
#include <stdint.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_cgem.h>

#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>
#include <ti/drv/iqn2/iqn2_osal.h>

#include <ti/drv/iqn2/iqn2ver.h>
#include <ti/drv/iqn2/include/IQN2_defs.h>

#define __IQN2_RUNTIME_C
#include <ti/drv/iqn2/include/IQN2_runtime.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(IQN2_at2GetCapturedBCN, ".text:iqn2");
#pragma CODE_SECTION(IQN2_at2AdjustBCNOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailUatGetCapturedBCN, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailUatAdjustBCNOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailUatGetCapturedEgrRADTs, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailUatAdjustEgrRADTsOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailUatGetCapturedIngRADTs, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailUatAdjustIngRADTsOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_aid2UatGetCapturedEgrRADTs, ".text:iqn2");
#pragma CODE_SECTION(IQN2_aid2UatAdjustEgrRADTsOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_aid2UatGetCapturedIngRADTs, ".text:iqn2");
#pragma CODE_SECTION(IQN2_aid2UatAdjustIngRADTsOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2UatGetCapturedEgrRADTs, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2UatGetCapturedEgrEngineRADTs, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2UatAdjustEgrRADTsOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2UatAdjustEgrEngineRADTsOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2UatGetCapturedIngRADTs, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2UatGetCapturedIngEngineRADTs, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2UatAdjustIngRADTsOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2UatAdjustIngEngineRADTsOffset, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2DisableEngines, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2EnableEngines, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2DisableIngressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2EnableIngressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2DisableEgressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_dio2EnableEgressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailDisableIngressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailEnableIngressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailDisableEgressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_ailEnableEgressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_aid2DisableIngressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_aid2EnableIngressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_aid2DisableEgressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_aid2EnableEgressChannels, ".text:iqn2");
#pragma CODE_SECTION(IQN2_enableEgressChannel, ".text:iqn2");
#pragma CODE_SECTION(IQN2_enableIngressChannel, ".text:iqn2");
#pragma CODE_SECTION(IQN2_disableEgressChannel, ".text:iqn2");
#pragma CODE_SECTION(IQN2_disableIngressChannel, ".text:iqn2");
#pragma CODE_SECTION(IQN2_enableAt2Event, ".text:iqn2");
#pragma CODE_SECTION(IQN2_disableAt2Event, ".text:iqn2");
#pragma CODE_SECTION(IQN2_resyncProcedure, ".text:iqn2");
#endif

/*
 * IQN2 UG - sync procedure
 *
 - AT: Sync boundary detect and Resynchronization
    - When synchronization has already been established and a new sync arrives while the BCN and RADT timers are counting,
     there may be an interrupt on sync (if enabled, (IQN2_enableAt2Exception) and the SW will determine if the BCN offset has changed from the original sync time.
     If it has, the SW may then readjust BCN offset or offset comparators for RADTs and uATs.
 - AT Re-synchronization procedure
    - When new external sync arrives, one of the BCN sync capture status registers will show new captured value (IQN2_at2GetCapturedBCN())
    - SW application calculates new offset based on captured value and update BCN offset register (IQN2_ailUatAdjustBCNOffset())
    - It takes more than one frame time to make both BCN and RADT fully synchronize and run correct with new offset. (Application responsibility)
     new mst_slv_sync signal will be delivered to uATs (AIL, AID, DIO) for uAT BCN and RADT synchronization
 - Micro AT Re-synchronization procedure (applied to AIL, AID, DIO)
    - new mst_slv_sync sync from AT arrives, all BCN and RADT sync capture status registers will show new captured value
    - SW application calculates new offset based on captured value and update BCN or RADT offset register if required.
     AID, DIO requires RADTs, AIL OBSAI requires BCN and RADTs and AIL CPRI only requires BCN
     (IQN2_ailUatGetCapturedBCN(), IQN2_ailUatAdjustBCNOffset(), ..... )
    - It takes more than two or three frame time to make both BCN and RADT fully synchronize and run correct with new offset
 - DIO module Re-synchronization: there are two restrictions to make DIO run correct after re-synchronization.
    - DIO engine should be disabled before updating the micro AT timer offset registers and enabled again after the re-sync process is fully done (IQN2_dio2DisableEngines())
    - All DIO EFE channels (used for SoC level ingress) must be disabled before updating the micro AT timer offset registers and enabled again after the re-sync process is fully done (IQN2_dio2DisableIngressChannels())
 *
 */


/* A sync event from one of the sync source was detected. BCN value was captured for use by APP SW to calculate
 * and correct RADT alignment */
void IQN2_at2GetCapturedBCN(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValue
)
{
    if (hIqn2->timerSyncSource == IQN2_PA_TSCOMP_SYNC) {
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AT2_BCN_PA_TSCOMP_CAPT_STS, (void *)hCapturedValue);
    }
    if (hIqn2->timerSyncSource == IQN2_PHY_SYNC) {
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AT2_BCN_PHYSYNC_CAPT_STS, (void *)hCapturedValue);
    }
    if (hIqn2->timerSyncSource == IQN2_RAD_SYNC) {
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AT2_BCN_RADSYNC_CAPT_STS, (void *)hCapturedValue);
    }
    if (hIqn2->timerSyncSource == IQN2_RP1_SYNC) {
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AT2_BCN_RP1_SYNC_CAPT_STS, (void *)hCapturedValue);
    }
}


void IQN2_at2AdjustBCNOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValue
)
{
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_BCN_OFFSET_CFG_VAL, (void *)hOffsetValue);
}

void IQN2_ailUatGetCapturedBCN(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
)
{
    uint32_t i;
    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_UAT_SYNC_BCN_CAPTURE_STS, (void *)&hCapturedValues[i]);
        }
    }
}

void IQN2_ailUatAdjustBCNOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
)
{
    uint32_t i;
    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_OFFSET_REG, (void *)&hOffsetValues[i]);
        }
    }
}

void IQN2_ailUatGetCapturedEgrRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t**         hCapturedValues
)
{
    uint32_t i;

    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            // offset RADT count[0:7]
            Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_UAT_EGR_SYNC_RADT_CAPTURE_STS, (void *)hCapturedValues[i]);
        }
    }
}

void IQN2_ailUatAdjustEgrRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t**         hOffsetValues
)
{
    uint32_t i,j;
    Iqn2Fl_RadtOffsetCfg radoffset;

    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            for (j=0;j<8;j++)
            {
                radoffset.radio_std = j;//radio standard index
                radoffset.offset = hOffsetValues[i][j];
                // offset RADT count[0:7]
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_UAT_EGR_RADT_OFFSET_CFG_REG, (void *)&radoffset);
            }
        }
    }
}


void IQN2_ailUatGetCapturedIngRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t**          hCapturedValues
)
{
    uint32_t i;

    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            // offset RADT count[0:7]
            Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AIL_UAT_ING_SYNC_RADT_CAPTURE_STS, (void *)hCapturedValues[i]);
        }
    }
}


void IQN2_ailUatAdjustIngRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t**         hOffsetValues
)
{
    uint32_t i,j;
    Iqn2Fl_RadtOffsetCfg radoffset;

    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            for (j=0;j<8;j++)
            {
                radoffset.radio_std = j;//radio standard index
                radoffset.offset = hOffsetValues[i][j];
                // offset RADT count[0:7]
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_UAT_ING_RADT_OFFSET_CFG_REG, (void *)&radoffset);
            }
        }
    }
}

void IQN2_aid2UatGetCapturedEgrRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*         hCapturedValues
)
{
    if (hIqn2->aidConfig.aidEnable)
    {
        // offset RADT count[0:7]
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS, (void *)hCapturedValues);
    }

}

void IQN2_aid2UatAdjustEgrRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
)
{
    uint32_t j;
    Iqn2Fl_RadtOffsetCfg radoffset;

    for (j=0;j<8;j++)
    {
        radoffset.radio_std = j;//radio standard index
        radoffset.offset = hOffsetValues[j];
        // offset RADT count[0:7]
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_UAT_EGR_RADT_OFFSET_CFG, (void *)&radoffset);
    }

}

void IQN2_aid2UatGetCapturedIngRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*         hCapturedValues
)
{
    if (hIqn2->aidConfig.aidEnable)
    {
        // offset RADT count[0:7]
        Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AID2_UAT_ING_SYNC_RADT_CAPTURE_STS, (void *)hCapturedValues);
    }

}

void IQN2_aid2UatAdjustIngRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
)
{
    uint32_t j;
    Iqn2Fl_RadtOffsetCfg radoffset;

    for (j=0;j<8;j++)
    {
        radoffset.radio_std = j;//radio standard index
        radoffset.offset = hOffsetValues[j];
        // offset RADT count[0:7]
        Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_UAT_ING_RADT_OFFSET_CFG, (void *)&radoffset);
    }

}

void IQN2_dio2UatGetCapturedEgrRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
)
{
    uint32_t i;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPeDBCH != 0) // check if any DIO engine is enabled
        {
            // offset RADT count[0:7]
            Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS, (void *)hCapturedValues);
            break;
        }
    }
}

void IQN2_dio2UatGetCapturedEgrEngineRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
)
{
    uint32_t i;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPeDBCH != 0) // check if any DIO engine is enabled
        {
            Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_UAT_DIO_EGR_SYNC_RADT_CAPTURE_STS, (void *)hCapturedValues[i]);
            break;
        }
    }
}

void IQN2_dio2UatAdjustEgrRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
)
{
    uint32_t i,j;
    Iqn2Fl_RadtOffsetCfg radoffset;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPeDBCH != 0) // check if any DIO engine is enabled
        {
            for (j=0;j<8;j++)
            {
                radoffset.radio_std = j;//radio standard index
                radoffset.offset = hOffsetValues[j];
                // offset RADT count[0:7]
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_UAT_EGR_RADT_OFFSET_CFG, (void *)&radoffset);
            }
            break;
        }
    }
}

void IQN2_dio2UatAdjustEgrEngineRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
)
{
    uint32_t i;
    Iqn2Fl_RadtOffsetCfg radoffset;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPeDBCH != 0) // check if any DIO engine is enabled
        {
            radoffset.radio_std = i;//dio engine index
            radoffset.offset = hOffsetValues[i];
             // offset DIO engine count[0:3], one at a time
             Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG, (void *)&radoffset);
        }
    }
}


void IQN2_dio2UatGetCapturedIngRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
)
{
    uint32_t i;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPdDBCH != 0) // check if any DIO engine is enabled
        {
            // offset RADT count[0:7]
            Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS, (void *)hCapturedValues);
            break;
        }
    }
}

void IQN2_dio2UatGetCapturedIngEngineRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
)
{
    uint32_t i;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPdDBCH != 0) // check if any DIO engine is enabled
        {
            Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_DIO2_UAT_DIO_ING_SYNC_RADT_CAPTURE_STS, (void *)hCapturedValues[i]);
            break;
        }
    }
}


void IQN2_dio2UatAdjustIngRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
)
{
    uint32_t i,j;
    Iqn2Fl_RadtOffsetCfg radoffset;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPdDBCH != 0) // check if any DIO engine is enabled
        {
            for (j=0;j<8;j++)
            {
                radoffset.radio_std = j;//radio standard index
                radoffset.offset = hOffsetValues[j];
                // offset RADT count[0:7]
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_UAT_ING_RADT_OFFSET_CFG, (void *)&radoffset);
            }
            break;
        }
    }
}

void IQN2_dio2UatAdjustIngEngineRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
)
{
    uint32_t i;
    Iqn2Fl_RadtOffsetCfg radoffset;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPdDBCH != 0) // check if any DIO engine is enabled
        {
            radoffset.radio_std = i;//dio engine index
            radoffset.offset = hOffsetValues[i];
             // offset DIO engine count[0:3], one at a time
             Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_UAT_DIO_ING_RADT_OFFSET_CFG, (void *)&radoffset);
        }
    }
}

void IQN2_dio2DisableEngines(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t dma_egr_eng_en[3] = {0,0,0};
    uint32_t dma_ing_eng_en[3] = {0,0,0};

    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_CORE_INGRESS_DMA_ENGINE_EN, (void *)dma_ing_eng_en);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_CORE_EGRESS_DMA_ENGINE_EN,  (void *)dma_egr_eng_en);
}

void IQN2_dio2EnableEngines(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i;
    uint32_t dma_egr_eng_en[3] = {0,0,0};
    uint32_t dma_ing_eng_en[3] = {0,0,0};

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPeDBCH != 0) // check if any DIO engine is enabled
        {
            dma_egr_eng_en[i] = 1; //egress dio engine i enable
        }
        if(hIqn2->dioConfig[i].numPdDBCH != 0) // check if any DIO engine is enabled
        {
            dma_ing_eng_en[i] = 1; //ingress dio engine i enable
        }
    }
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_CORE_INGRESS_DMA_ENGINE_EN, (void *)dma_ing_eng_en);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_CORE_EGRESS_DMA_ENGINE_EN,  (void *)dma_egr_eng_en);
}

void IQN2_dio2DisableIngressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPdDBCH != 0) // check if any ingress DIO engine is enabled
        {
            for (j = hIqn2->dioConfig[i].offsetPdDBCH ; j < (hIqn2->dioConfig[i].offsetPdDBCH + hIqn2->dioConfig[i].numPdDBCH) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 0;         //disable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
    }
}

void IQN2_dio2EnableIngressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPdDBCH != 0) // check if any ingress DIO engine is enabled
        {
            for (j = hIqn2->dioConfig[i].offsetPdDBCH ; j < (hIqn2->dioConfig[i].offsetPdDBCH + hIqn2->dioConfig[i].numPdDBCH) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 1;         //enable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
    }
}

void IQN2_dio2DisableEgressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPeDBCH != 0) // check if any ingress DIO engine is enabled
        {
            for (j = hIqn2->dioConfig[i].offsetPeDBCH ; j < (hIqn2->dioConfig[i].offsetPeDBCH + hIqn2->dioConfig[i].numPeDBCH) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 0;         //disable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
    }
}

void IQN2_dio2EnableEgressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0;i<IQN2_MAX_NUM_DIO_ENGINE;i++)
    {
        if(hIqn2->dioConfig[i].numPeDBCH != 0) // check if any ingress DIO engine is enabled
        {
            for (j = hIqn2->dioConfig[i].offsetPeDBCH ; j < (hIqn2->dioConfig[i].offsetPeDBCH + hIqn2->dioConfig[i].numPeDBCH) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 1;         //enable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_DIO2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
    }
}

void IQN2_ailDisableIngressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            if (hIqn2->ailConfig[i].numLtePdAxC)
            {
                for (j = hIqn2->ailConfig[i].firstLteAxC; j < (hIqn2->ailConfig[i].firstLteAxC + hIqn2->ailConfig[i].numLtePdAxC) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 0;         //disable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
            if (hIqn2->ailConfig[i].numWcdmaPdAxC)
            {
                for (j = hIqn2->ailConfig[i].firstWcdmaAxC; j < (hIqn2->ailConfig[i].firstWcdmaAxC + hIqn2->ailConfig[i].numWcdmaPdAxC) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 0;         //disable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
            if (hIqn2->ailConfig[i].numCtlChannel)
            {
                for (j = hIqn2->ailConfig[i].firstCtlChannel; j < (hIqn2->ailConfig[i].firstCtlChannel + hIqn2->ailConfig[i].numCtlChannel) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 0;         //disable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
        }
    }
}

void IQN2_ailEnableIngressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            if (hIqn2->ailConfig[i].numLtePdAxC)
            {
                for (j = hIqn2->ailConfig[i].firstLteAxC; j < (hIqn2->ailConfig[i].firstLteAxC + hIqn2->ailConfig[i].numLtePdAxC) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
            if (hIqn2->ailConfig[i].numWcdmaPdAxC)
            {
                for (j = hIqn2->ailConfig[i].firstWcdmaAxC; j < (hIqn2->ailConfig[i].firstWcdmaAxC + hIqn2->ailConfig[i].numWcdmaPdAxC) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
            if (hIqn2->ailConfig[i].numCtlChannel)
            {
                for (j = hIqn2->ailConfig[i].firstCtlChannel; j < (hIqn2->ailConfig[i].firstCtlChannel + hIqn2->ailConfig[i].numCtlChannel) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
        }
    }
}


void IQN2_ailDisableEgressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            if (hIqn2->ailConfig[i].numLtePeAxC)
            {
                for (j = hIqn2->ailConfig[i].firstLteAxC; j < (hIqn2->ailConfig[i].firstLteAxC + hIqn2->ailConfig[i].numLtePeAxC) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 0;         //disable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
            if (hIqn2->ailConfig[i].numWcdmaPeAxC)
            {
                for (j = hIqn2->ailConfig[i].firstWcdmaAxC; j < (hIqn2->ailConfig[i].firstWcdmaAxC + hIqn2->ailConfig[i].numWcdmaPeAxC) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 0;         //disable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
            if (hIqn2->ailConfig[i].numCtlChannel)
            {
                for (j = hIqn2->ailConfig[i].firstCtlChannel; j < (hIqn2->ailConfig[i].firstCtlChannel + hIqn2->ailConfig[i].numCtlChannel) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 0;         //disable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
        }
    }
}

void IQN2_ailEnableEgressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i,j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0; i<IQN2_MAX_NUM_AIL; i++)
    {
        if (hIqn2->ailConfig[i].ailEnable)
        {
            hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
            if (hIqn2->ailConfig[i].numLtePeAxC)
            {
                for (j = hIqn2->ailConfig[i].firstLteAxC; j < (hIqn2->ailConfig[i].firstLteAxC + hIqn2->ailConfig[i].numLtePeAxC) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
            if (hIqn2->ailConfig[i].numWcdmaPeAxC)
            {
                for (j = hIqn2->ailConfig[i].firstWcdmaAxC; j < (hIqn2->ailConfig[i].firstWcdmaAxC + hIqn2->ailConfig[i].numWcdmaPeAxC) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
            if (hIqn2->ailConfig[i].numCtlChannel)
            {
                for (j = hIqn2->ailConfig[i].firstCtlChannel; j < (hIqn2->ailConfig[i].firstCtlChannel + hIqn2->ailConfig[i].numCtlChannel) ; j++)
                {
                    Args.reg_index = j;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
        }
    }
}

void IQN2_aid2DisableIngressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
        if (hIqn2->aidConfig.numLteIngressAxC)
        {
            for (j = hIqn2->aidConfig.firstLteAxC; j < (hIqn2->aidConfig.firstLteAxC + hIqn2->aidConfig.numLteIngressAxC) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 0;         //disable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
        if (hIqn2->aidConfig.numWcdmaIngressAxC)
        {
            for (j = hIqn2->aidConfig.firstWcdmaAxC; j < (hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaIngressAxC) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 0;         //disable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
        if (hIqn2->aidConfig.numCtlChannel)
        {
            for (j = 0 /*hIqn2->aidConfig.firstCtlChannel*/; j < (/*hIqn2->aidConfig.firstCtlChannel +*/ hIqn2->aidConfig.numCtlChannel) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 0;         //disable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ICTL_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
    }
}

void IQN2_aid2EnableIngressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
        if (hIqn2->aidConfig.numLteIngressAxC)
        {
            for (j = hIqn2->aidConfig.firstLteAxC; j < (hIqn2->aidConfig.firstLteAxC + hIqn2->aidConfig.numLteIngressAxC) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 1;         //enable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
        if (hIqn2->aidConfig.numWcdmaIngressAxC)
        {
            for (j = hIqn2->aidConfig.firstWcdmaAxC; j < (hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaIngressAxC) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 1;         //enable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
        if (hIqn2->aidConfig.numCtlChannel)
        {
            for (j = 0 /*hIqn2->aidConfig.firstCtlChannel*/; j < (/*hIqn2->aidConfig.firstCtlChannel +*/ hIqn2->aidConfig.numCtlChannel) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 1;         //enable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ICTL_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
    }
}

void IQN2_aid2DisableEgressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
        if (hIqn2->aidConfig.numLteEgressAxC)
        {
            for (j = hIqn2->aidConfig.firstLteAxC; j < (hIqn2->aidConfig.firstLteAxC + hIqn2->aidConfig.numLteEgressAxC) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 0;         //disable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
        if (hIqn2->aidConfig.numWcdmaEgressAxC)
        {
            for (j = hIqn2->aidConfig.firstWcdmaAxC; j < (hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaEgressAxC) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 0;         //disable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
        if (hIqn2->aidConfig.numCtlChannel)
        {
            for (j = 0/*hIqn2->aidConfig.firstCtlChannel*/; j < (/*hIqn2->aidConfig.firstCtlChannel +*/ hIqn2->aidConfig.numCtlChannel) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 0;         //disable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ECTL_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
    }
}

void IQN2_aid2EnableEgressChannels(
        IQN2_ConfigHandle  hIqn2
)
{
    uint32_t j,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
        if (hIqn2->aidConfig.numLteEgressAxC)
        {
            for (j = hIqn2->aidConfig.firstLteAxC; j < (hIqn2->aidConfig.firstLteAxC + hIqn2->aidConfig.numLteEgressAxC) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 1;         //enable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
        if (hIqn2->aidConfig.numWcdmaEgressAxC)
        {
            for (j = hIqn2->aidConfig.firstWcdmaAxC; j < (hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaEgressAxC) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 1;         //enable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
        if (hIqn2->aidConfig.numCtlChannel)
        {
            for (j = 0/*hIqn2->aidConfig.firstCtlChannel*/; j < (/*hIqn2->aidConfig.firstCtlChannel +*/ hIqn2->aidConfig.numCtlChannel) ; j++)
            {
                Args.reg_index = j;  //channel number
                ctrlArg = 1;         //enable the channel
                Args.reg_arg = (void *)&ctrlArg;
                Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ECTL_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
            }
        }
    }
}

void IQN2_enableEgressChannel(
        IQN2_ConfigHandle  hIqn2,
        uint32_t index
)
{
    uint32_t i,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
        if  ((index >= hIqn2->aidConfig.firstLteAxC) && (index <= (hIqn2->aidConfig.firstLteAxC + hIqn2->aidConfig.numLteEgressAxC)))
        {
            Args.reg_index = index;  //channel number
            ctrlArg = 1;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }

        if  ((index >= hIqn2->aidConfig.firstWcdmaAxC) && (index <= (hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaEgressAxC)))
        {
            Args.reg_index = index;  //channel number
            ctrlArg = 1;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }

        if  ((index >= hIqn2->aidConfig.firstCtlChannel) && (index <= (hIqn2->aidConfig.firstCtlChannel + hIqn2->aidConfig.numCtlChannel)))
        {
            Args.reg_index = (index - hIqn2->aidConfig.firstCtlChannel);  //channel number
            ctrlArg = 1;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ECTL_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }
    } else {

        for (i=0; i<IQN2_MAX_NUM_AIL; i++)
        {
            if (hIqn2->ailConfig[i].ailEnable)
            {
                hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
                if ((index >= hIqn2->ailConfig[i].firstLteAxC) && (index <= (hIqn2->ailConfig[i].firstLteAxC + hIqn2->ailConfig[i].numLtePeAxC)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }

                if ((index >= hIqn2->ailConfig[i].firstWcdmaAxC) && (index <= (hIqn2->ailConfig[i].firstWcdmaAxC + hIqn2->ailConfig[i].numWcdmaPeAxC)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }

                if ((index >= hIqn2->ailConfig[i].firstCtlChannel) && (index <= (hIqn2->ailConfig[i].firstCtlChannel + hIqn2->ailConfig[i].numCtlChannel)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
        }
    }
}

void IQN2_enableIngressChannel(
        IQN2_ConfigHandle  hIqn2,
        uint32_t index
)
{
    uint32_t i, ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
        if  ((index >= hIqn2->aidConfig.firstLteAxC) && (index <= (hIqn2->aidConfig.firstLteAxC + hIqn2->aidConfig.numLteIngressAxC)))
        {
            Args.reg_index = index;  //channel number
            ctrlArg = 1;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }

        if  ((index >= hIqn2->aidConfig.firstWcdmaAxC) && (index <= (hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaIngressAxC)))
        {
            Args.reg_index = index;  //channel number
            ctrlArg = 1;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }

        if  ((index >= hIqn2->aidConfig.firstCtlChannel) && (index <= (hIqn2->aidConfig.firstCtlChannel + hIqn2->aidConfig.numCtlChannel)))
        {
            Args.reg_index = (index - hIqn2->aidConfig.firstCtlChannel);  //channel number
            ctrlArg = 1;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ICTL_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }
    } else {

        for (i=0; i<IQN2_MAX_NUM_AIL; i++)
        {
            if (hIqn2->ailConfig[i].ailEnable)
            {
                hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
                if ((index >= hIqn2->ailConfig[i].firstLteAxC) && (index <= (hIqn2->ailConfig[i].firstLteAxC + hIqn2->ailConfig[i].numLtePdAxC)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }

                if ((index >= hIqn2->ailConfig[i].firstWcdmaAxC) && (index <= (hIqn2->ailConfig[i].firstWcdmaAxC + hIqn2->ailConfig[i].numWcdmaPdAxC)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }

                if ((index >= hIqn2->ailConfig[i].firstCtlChannel) && (index <= (hIqn2->ailConfig[i].firstCtlChannel + hIqn2->ailConfig[i].numCtlChannel)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 1;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
        }
    }
}

void IQN2_disableEgressChannel(
        IQN2_ConfigHandle  hIqn2,
        uint32_t index
)
{
    uint32_t i,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
        if  ((index >= hIqn2->aidConfig.firstLteAxC) && (index <= (hIqn2->aidConfig.firstLteAxC + hIqn2->aidConfig.numLteEgressAxC)))
        {
            Args.reg_index = index;  //channel number
            ctrlArg = 0;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }

        if  ((index >= hIqn2->aidConfig.firstWcdmaAxC) && (index <= (hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaEgressAxC)))
        {
            Args.reg_index = index;  //channel number
            ctrlArg = 0;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }

        if  ((index >= hIqn2->aidConfig.firstCtlChannel) && (index <= (hIqn2->aidConfig.firstCtlChannel + hIqn2->aidConfig.numCtlChannel)))
        {
            Args.reg_index = (index - hIqn2->aidConfig.firstCtlChannel);  //channel number
            ctrlArg = 0;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ECTL_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }
    } else {

        for (i=0; i<IQN2_MAX_NUM_AIL; i++)
        {
            if (hIqn2->ailConfig[i].ailEnable)
            {
                hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
                if ((index >= hIqn2->ailConfig[i].firstLteAxC) && (index <= (hIqn2->ailConfig[i].firstLteAxC + hIqn2->ailConfig[i].numLtePeAxC)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 0;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }

                if ((index >= hIqn2->ailConfig[i].firstWcdmaAxC) && (index <= (hIqn2->ailConfig[i].firstWcdmaAxC + hIqn2->ailConfig[i].numWcdmaPeAxC)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 0;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }

                if ((index >= hIqn2->ailConfig[i].firstCtlChannel) && (index <= (hIqn2->ailConfig[i].firstCtlChannel + hIqn2->ailConfig[i].numCtlChannel)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 0;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
        }
    }
}

void IQN2_disableIngressChannel(
        IQN2_ConfigHandle  hIqn2,
        uint32_t index
)
{
    uint32_t i,ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
        if  ((index >= hIqn2->aidConfig.firstLteAxC) && (index <= (hIqn2->aidConfig.firstLteAxC + hIqn2->aidConfig.numLteIngressAxC)))
        {
            Args.reg_index = index;  //channel number
            ctrlArg = 0;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }

        if  ((index >= hIqn2->aidConfig.firstWcdmaAxC) && (index <= (hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaIngressAxC)))
        {
            Args.reg_index = index;  //channel number
            ctrlArg = 0;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }

        if  ((index >= hIqn2->aidConfig.firstCtlChannel) && (index <= (hIqn2->aidConfig.firstCtlChannel + hIqn2->aidConfig.numCtlChannel)))
        {
            Args.reg_index = (index - hIqn2->aidConfig.firstCtlChannel);  //channel number
            ctrlArg = 0;         //enable the channel
            Args.reg_arg = (void *)&ctrlArg;
            Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_ICTL_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
        }
    } else {

        for (i=0; i<IQN2_MAX_NUM_AIL; i++)
        {
            if (hIqn2->ailConfig[i].ailEnable)
            {
                hIqn2->hFl->arg_ail = (Iqn2Fl_AilInstance)i;
                if ((index >= hIqn2->ailConfig[i].firstLteAxC) && (index <= (hIqn2->ailConfig[i].firstLteAxC + hIqn2->ailConfig[i].numLtePdAxC)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 0;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }

                if ((index >= hIqn2->ailConfig[i].firstWcdmaAxC) && (index <= (hIqn2->ailConfig[i].firstWcdmaAxC + hIqn2->ailConfig[i].numWcdmaPdAxC)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 0;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }

                if ((index >= hIqn2->ailConfig[i].firstCtlChannel) && (index <= (hIqn2->ailConfig[i].firstCtlChannel + hIqn2->ailConfig[i].numCtlChannel)))
                {
                    Args.reg_index = index;  //channel number
                    ctrlArg = 0;         //enable the channel
                    Args.reg_arg = (void *)&ctrlArg;
                    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG, (void *)&Args);
                }
            }
        }
    }
}

void IQN2_iqs2EgressAid2Reconfigure(
        IQN2_ConfigHandle  hIqn2,
        uint32_t dma_channel,
        uint32_t aid_channel
)
{

    uint32_t ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;


    if (hIqn2->aidConfig.aidEnable)
    {
    	if((hIqn2->aidConfig.lteDio==0) && (!((hIqn2->aidConfig.firstWcdmaAxC > aid_channel) && ((hIqn2->aidConfig.numWcdmaEgressAxC + hIqn2->aidConfig.firstWcdmaAxC) <= aid_channel))))
    	{
			Args.reg_index = dma_channel;  //pktDma channel number
			ctrlArg = aid_channel;         //aid channel number
			Args.reg_arg = (void *)&ctrlArg;
			Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EGR_PKTDMA_CFG_CHAN, (void *)&Args);
    	} else {
    		Args.reg_index = dma_channel;  //pktDma channel number
			ctrlArg = aid_channel;         //aid channel number
			Args.reg_arg = (void *)&ctrlArg;
			Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_EGR_DIO2_CFG_CHAN, (void *)&Args);
    	}
    }
}

void IQN2_iqs2IngressAid2Reconfigure(
        IQN2_ConfigHandle  hIqn2,
        uint32_t dma_channel,
        uint32_t aid_channel
)
{
    uint32_t ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    if (hIqn2->aidConfig.aidEnable)
    {
		/* Setup IQS ING AID2 AXC LUT CFG */
		Args.reg_index = aid_channel;  //pktDma channel number
		ctrlArg = dma_channel;         //aid channel number
		Args.reg_arg = (void *)&ctrlArg;
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_ING_AID2_AXC_LUT_CFG_CHAN, (void *)&Args);

		if((hIqn2->aidConfig.lteDio==0) && (!((hIqn2->aidConfig.firstWcdmaAxC > aid_channel) && ((hIqn2->aidConfig.numWcdmaEgressAxC + hIqn2->aidConfig.firstWcdmaAxC) <= aid_channel))))
    	{
			Args.reg_index = aid_channel;
			ctrlArg = 0;         		//dest is PKTDMA
			Args.reg_arg = (void *)&ctrlArg;
			Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_ING_AID2_AXC_LUT_CFG_DEST, (void *)&Args);
    	} else {
    		Args.reg_index = aid_channel;
			ctrlArg = 1;         		//dest is DIO2
			Args.reg_arg = (void *)&ctrlArg;
			Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_IQS_ING_AID2_AXC_LUT_CFG_DEST, (void *)&Args);
    	}
	}
}

void IQN2_aid2EfeTdmEnable(
		IQN2_ConfigHandle  hIqn2,
		uint32_t radStdIndex
)
{
    uint32_t ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    Args.reg_index = radStdIndex;
	ctrlArg = 1;         		//dest is DIO2
	Args.reg_arg = (void *)&ctrlArg;
	Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IQ_EFE_RAD_STD_SCH_CFG_EN, (void *)&Args);

}

void IQN2_aid2EfeTdmReconfigureLength(
		IQN2_ConfigHandle  hIqn2,
		uint32_t radStdIndex,
		uint32_t len
)
{
	uint32_t ctrlArg;
	Iqn2Fl_HwControlMultiArgs Args;

	/* Setup AID2 IQ EFE RADIO STANDARD SCHEDULER GROUP - EFE RAD STD SCH CFG */
	Args.reg_index = radStdIndex;
	ctrlArg = len;         		//dest is DIO2
	Args.reg_arg = (void *)&ctrlArg;
	Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IQ_EFE_RAD_STD_SCH_CFG_LEN, (void *)&Args);
}

void IQN2_aid2EfeTdmDisableAll(
		IQN2_ConfigHandle  hIqn2
)
{
    uint32_t i, ctrlArg;
    Iqn2Fl_HwControlMultiArgs Args;

    for (i=0;i<8;i++)
    {
		Args.reg_index = i;
		ctrlArg = 0;         		//disable TDM LUT
		Args.reg_arg = (void *)&ctrlArg;
		Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AID2_IQ_EFE_RAD_STD_SCH_CFG_EN, (void *)&Args);
    }
}

void
IQN2_enableAt2Event(
        IQN2_ConfigHandle    hIqn2,
        uint32_t             EventSelect
)
{
    uint32_t ctrlArg;

    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AT2_EVENTS_ENABLE_STS, (void *)&ctrlArg);
    // Set this At event bit
    ctrlArg |= (1<<EventSelect);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG, (void *)&ctrlArg);
}

void
IQN2_disableAt2Event(
        IQN2_ConfigHandle    hIqn2,
        uint32_t             EventSelect
)
{
    uint32_t ctrlArg;

    Iqn2Fl_getHwStatus(hIqn2->hFl, IQN2FL_QUERY_AT2_EVENTS_ENABLE_STS, (void *)&ctrlArg);
    // Clear this At event bit
    ctrlArg &= ~(1 << EventSelect);
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG, (void *)&ctrlArg);
}


void
IQN2_disableRadioTimers(
        IQN2_ConfigHandle    hIqn2
)
{
    uint32_t ctrlArg;

    ctrlArg = 0x00000000;
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN, (void *)&ctrlArg);
}


void
IQN2_enableRadioTimers(
        IQN2_ConfigHandle    hIqn2
)
{
    uint32_t ctrlArg, j;

    ctrlArg = 0x00000000;
    for (j=0; j<IQN2_MAX_NUM_RADT; j++)
    {
          if (hIqn2->radTimerConfig[j].mode != IQN2_RADT_UNUSED_MODE) {
              ctrlArg |= (1<<j);
          }
    }
    Iqn2Fl_hwControl(hIqn2->hFl, IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN, (void *)&ctrlArg);
}


/*
 * IQN2 UG - sync procedure
 *
 - AT: Sync boundary detect and Resynchronization
    - When synchronization has already been established and a new sync arrives while the BCN and RADT timers are counting,
     there may be an interrupt on sync (if enabled, (IQN2_enableAt2Exception) and the SW will determine if the BCN offset has changed from the original sync time.
     If it has, the SW may then readjust BCN offset or offset comparators for RADTs and uATs.
 - AT Re-synchronization procedure
    - When new external sync arrives, one of the BCN sync capture status registers will show new captured value (IQN2_at2GetCapturedBCN())
    - SW application calculates new offset based on captured value and update BCN offset register (IQN2_ailUatAdjustBCNOffset())
    - It takes more than one frame time to make both BCN and RADT fully synchronize and run correct with new offset. (Application responsibility)
     new mst_slv_sync signal will be delivered to uATs (AIL, AID, DIO) for uAT BCN and RADT synchronization
 - Micro AT Re-synchronization procedure (applied to AIL, AID, DIO)
    - new mst_slv_sync sync from AT arrives, all BCN and RADT sync capture status registers will show new captured value
    - SW application calculates new offset based on captured value and update BCN or RADT offset register if required.
     AID, DIO requires RADTs, AIL OBSAI requires BCN and RADTs and AIL CPRI only requires BCN
     (IQN2_ailUatGetCapturedBCN(), IQN2_ailUatAdjustBCNOffset(), ..... )
    - It takes more than two or three frame time to make both BCN and RADT fully synchronize and run correct with new offset
 - DIO module Re-synchronization: there are two restrictions to make DIO run correct after re-synchronization.
    - DIO engine should be disabled before updating the micro AT timer offset registers and enabled again after the re-sync process is fully done (IQN2_dio2DisableEngines())
    - All DIO EFE channels (used for SoC level ingress) must be disabled before updating the micro AT timer offset registers and enabled again after the re-sync process is fully done (IQN2_dio2DisableIngressChannels())
 *
 */
static uint32_t rp1_sync_info = 0;
static uint32_t radsync_info = 0;
static uint32_t physync_info = 0;
static uint32_t pa_tscomp_info = 0;

void
IQN2_resyncProcedureReset()
{
    rp1_sync_info = 0;
    radsync_info = 0;
    physync_info = 0;
    pa_tscomp_info = 0;
}

uint32_t
IQN2_resyncProcedure(
        IQN2_ConfigHandle hIqn2,
        int32_t           delay
)
{
uint32_t i, ext_sync=0;
uint32_t capturedVal[16], newOffsetVal[16], frameClockCount=0, frameNum;

    if ((hIqn2->timerSyncSource == IQN2_RP1_SYNC) && (hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.rp1_sync_info != rp1_sync_info))
    {
    	if (hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.rp1_sync_info > 0) {
    		ext_sync = 1;
    		rp1_sync_info = hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.rp1_sync_info;
    	}
    }
    if ((hIqn2->timerSyncSource == IQN2_RAD_SYNC) && (hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.radsync_info  != radsync_info))
    {
    	if (hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.radsync_info > 0) {
    		ext_sync = 1;
    		radsync_info = hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.radsync_info;
    	}
    }
    if ((hIqn2->timerSyncSource == IQN2_PHY_SYNC) && (hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.physync_info  != physync_info))
    {
    	if (hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.physync_info > 0) {
    		ext_sync = 1;
    		physync_info = hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.physync_info;
    	}
    }
    if ((hIqn2->timerSyncSource == IQN2_PA_TSCOMP_SYNC) && (hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.pa_tscomp_info != pa_tscomp_info))
    {
    	if (hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.pa_tscomp_info > 0) {
    		ext_sync = 1;
    		pa_tscomp_info = hIqn2->iqn2EeCount.eeAt2.at2EeInfoErrCnt.pa_tscomp_info;
    	}
    }

    if (ext_sync == 1)
    {
        // DIO module Re-synchronization restriction
        IQN2_dio2DisableIngressChannels(hIqn2);
        IQN2_dio2DisableEngines(hIqn2);

        // AID2 channels disabling
        IQN2_aid2DisableIngressChannels(hIqn2);
        IQN2_aid2DisableEgressChannels(hIqn2);

        // Disable radio timers
        IQN2_disableRadioTimers(hIqn2);

        if (hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)            frameClockCount = (IQN2_CPRI_CLOCK_COUNT_TC_PHY_TIMER+1);
        else if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)      frameClockCount = (IQN2_OBSAI_CLOCK_COUNT_TC_PHY_TIMER+1);
        else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76) frameClockCount = (IQN2_DFE_245_76_CLOCK_COUNT_TC_PHY_TIMER+1);
        else if (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64) frameClockCount = (IQN2_DFE_368_64_CLOCK_COUNT_TC_PHY_TIMER+1);

        // AT Re-synchronization procedure
        IQN2_at2GetCapturedBCN(hIqn2,&capturedVal[0]);
        newOffsetVal[0] = (frameClockCount - capturedVal[0] + delay) % frameClockCount;
        IQN2_at2AdjustBCNOffset(hIqn2,&newOffsetVal[0]);

        // Wait for BCN and RADT to be synchronized (takes more than one frame time)
        frameNum = hIqn2->hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
        while (hIqn2->hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS <= (2+frameNum)) {
            Iqn2_osalSleep ();
        }

        // Micro AT Re-synchronization procedure
        // AID, DIO requires RADTs, AIL OBSAI requires BCN and RADTs and AIL CPRI only requires BCN
        if ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) || (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)){
            IQN2_ailUatGetCapturedBCN(hIqn2,capturedVal);
            newOffsetVal[0] = (frameClockCount - capturedVal[0] + delay) % frameClockCount;
            newOffsetVal[1] = (frameClockCount - capturedVal[1] + delay) % frameClockCount;
            IQN2_ailUatAdjustBCNOffset(hIqn2,newOffsetVal);
        }
        if (hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI){
            IQN2_ailUatGetCapturedEgrRADTs(hIqn2,(uint32_t**)capturedVal);
            for (i=0;i<16;i++)
            {
                newOffsetVal[i] = (frameClockCount - capturedVal[i] + delay) % frameClockCount;
            }
            IQN2_ailUatAdjustEgrRADTsOffset(hIqn2,(uint32_t**)newOffsetVal);
            IQN2_ailUatGetCapturedIngRADTs(hIqn2,(uint32_t**)capturedVal);
            for (i=0;i<16;i++)
            {
                newOffsetVal[i] = (frameClockCount - capturedVal[i] + delay) % frameClockCount;
            }
            IQN2_ailUatAdjustIngRADTsOffset(hIqn2,(uint32_t**)newOffsetVal);
        }
        if ((hIqn2->protocol == IQN2FL_PROTOCOL_DFE_245_76) || (hIqn2->protocol == IQN2FL_PROTOCOL_DFE_368_64)){
            IQN2_aid2UatGetCapturedEgrRADTs(hIqn2,capturedVal);
            for (i=0;i<8;i++)
            {
                newOffsetVal[i] = (frameClockCount - capturedVal[i] + delay) % frameClockCount;
            }
            IQN2_aid2UatAdjustEgrRADTsOffset(hIqn2,newOffsetVal);
            IQN2_aid2UatGetCapturedIngRADTs(hIqn2,capturedVal);
            for (i=0;i<8;i++)
            {
                newOffsetVal[i] = (frameClockCount - capturedVal[i] + delay) % frameClockCount;
            }
            IQN2_aid2UatAdjustIngRADTsOffset(hIqn2,newOffsetVal);
        }
        IQN2_dio2UatGetCapturedEgrRADTs(hIqn2,capturedVal);
        for (i=0;i<8;i++)
        {
            newOffsetVal[i] = (frameClockCount - capturedVal[i] + delay) % frameClockCount;
        }
        IQN2_dio2UatAdjustEgrRADTsOffset(hIqn2,newOffsetVal);
        IQN2_dio2UatGetCapturedEgrEngineRADTs(hIqn2,capturedVal);
        for (i=0;i<3;i++)
        {
            newOffsetVal[i] = (frameClockCount - capturedVal[i] + delay) % frameClockCount;
        }
        IQN2_dio2UatAdjustEgrEngineRADTsOffset(hIqn2,newOffsetVal);
        IQN2_dio2UatGetCapturedIngEngineRADTs(hIqn2,capturedVal);
        for (i=0;i<3;i++)
        {
            newOffsetVal[i] = (frameClockCount - capturedVal[i] + delay) % frameClockCount;
        }
        IQN2_dio2UatAdjustIngEngineRADTsOffset(hIqn2,newOffsetVal);

        // takes more than two or three frame time to make both BCN and RADT fully synchronize and run correct with new offset
        frameNum = hIqn2->hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
        while (hIqn2->hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS <= (4+frameNum)) {
            Iqn2_osalSleep ();
        }

        // enable radio timers
        //IQN2_enableRadioTimers(hIqn2);

        // DIO module Re-synchronization restriction
        IQN2_dio2EnableIngressChannels(hIqn2);
        IQN2_dio2EnableEngines(hIqn2);

        // AID2 channels enabling
        IQN2_aid2EnableIngressChannels(hIqn2);
        IQN2_aid2EnableEgressChannels(hIqn2);
    }

    return (ext_sync);
}



////////////////////

