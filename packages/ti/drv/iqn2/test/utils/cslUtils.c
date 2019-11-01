/****************************************************************************\
 *           (C) Copyright 2013, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#ifdef __ARMv7
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#else
#include <c6x.h>
#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif
#endif

#include <ti/csl/csl.h>
#include <ti/csl/cslver.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_serdes.h>
#include <ti/csl/csl_serdes_iqn.h>
#include <ti/csl/csl_serdes_dfe.h>
#include <ti/csl/csl_serdes_restore_default.h>

#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_device_interrupt.h>
#include <ti/csl/csl_qm_queue.h>

#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/csl/cslr_cpintc.h>
#include <ti/csl/cslr_bootcfg.h>

#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>

#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/iqn2/device/iqn2_device.h>
#include <ti/drv/iqn2/device/k2l/src/iqn2_device.c>

#include <ti/drv/dfe/dfe_drv.h>

#define __CSLUTILS_C
#include "cslUtils.h"

volatile uint32_t           ext_sync = 0;
#ifdef _TMS320C6X
/*define interrupt vector ID for events*/
#define 	COM_IQN2_AT_EVT0_EVENT 				CSL_INTC_VECTID_4
#define 	COM_IQN2_AT_EVT1_EVENT 				CSL_INTC_VECTID_5
#define 	COM_IQN2_AT_EVT2_EVENT 				CSL_INTC_VECTID_6
#define     COM_IQN2_EV0EXCEP_EVENT             CSL_INTC_VECTID_10
#define     COM_DFE_EV1EXCEP_EVENT              CSL_INTC_VECTID_11
#define     COM_IQN2_PKTDMAEXCEP_EVENT          CSL_INTC_VECTID_12

uint8_t                     DSP_procId;

CSL_IntcRegs                *IntcRegs= (CSL_IntcRegs *)CSL_C66X_COREPAC_REG_BASE_ADDRESS_REGS;
CSL_CPINTCRegs              *CicRegs[3];
CSL_CPINTC_Handle           hIntc0Handle;

#ifndef USESYSBIOS
CSL_IntcObj                 intcObj[16];
CSL_IntcHandle              hIntc[16];
static CSL_IntcContext      intcContext;
CSL_IntcEventHandlerRecord  EventRecord[16];
#endif

CSL_TmrHandle               hTmr = NULL;
CSL_TmrObj                  TmrObj;
CSL_TmrHwSetup              TmrSetup;

#ifndef RUNTIME
extern IQN2_ConfigObj  iqn2LldObj;
#if defined DFE_BB_LOOPBACK || defined DFE_JESD_LOOPBACK || defined DFE_NORMAL_OPERATION
extern DFE_Obj         dfeLldObj;
#endif
#endif

#ifdef _TMS320C6X
#pragma CODE_SECTION(UTILS_fillIqn2Obj, ".text:tools");
#pragma CODE_SECTION(UTILS_fillIqn2Aid2Obj, ".text:tools");
#pragma CODE_SECTION(UTILS_waitForHw, ".text:tools");
#pragma CODE_SECTION(UTILS_getIqn2EV0Exception, ".text:tools");
#pragma CODE_SECTION(UTILS_getIqn2PktDMAException, ".text:tools");
#pragma CODE_SECTION(UTILS_iqn2IntcSetup, ".text:tools");
#pragma CODE_SECTION(UTILS_iqn2ExceptIntDisable, ".text:tools");
#pragma CODE_SECTION(UTILS_setCache, ".text:tools");
#pragma CODE_SECTION(UTILS_cacheInvalidate, ".text:tools");
#pragma CODE_SECTION(UTILS_cacheWriteBack, ".text:tools");
#pragma CODE_SECTION(UTILS_cacheWriteBackInvalidate, ".text:tools");
#pragma CODE_SECTION(UTILS_resetTimer, ".text:tools");
#pragma CODE_SECTION(UTILS_startTimer, ".text:tools");
#pragma CODE_SECTION(UTILS_initTimer, ".text:tools");
#pragma CODE_SECTION(UTILS_triggerExternalSync, ".text:tools");
#pragma CODE_SECTION(UTILS_iqn2DfeEnable, ".text:tools");
#pragma CODE_SECTION(UTILS_iqn2DfeDisable, ".text:tools");
#pragma CODE_SECTION(UTILS_iqn2DfeDisableResetIso, ".text:tools");
#pragma CODE_SECTION(UTILS_doCleanup, ".text:tools");
#pragma CODE_SECTION(UTILS_iqn2SerdesConfig, ".text:tools");
#pragma CODE_SECTION(UTILS_dfeSerdesConfig, ".text:tools");

#pragma CODE_SECTION(dummyIsr, ".text:tools");
#pragma CODE_SECTION(Iqn2_AT_evt0_ISR, ".text:tools");
#pragma CODE_SECTION(Iqn2_AT_evt1_ISR, ".text:tools");
#pragma CODE_SECTION(Iqn2_AT_evt2_ISR, ".text:tools");
#pragma CODE_SECTION(Iqn2_EV0Exception_ISR, ".text:tools");
#pragma CODE_SECTION(Dfe_EV1Exception_ISR, ".text:tools");
#pragma CODE_SECTION(Iqn2_PktDMAException_ISR, ".text:tools");
#pragma CODE_SECTION(iqn2_mmap, ".text:tools");
#pragma CODE_SECTION(iqn2_mapDevice, ".text:tools");
#pragma CODE_SECTION(iqn2_unmapDevice, ".text:tools");
#pragma CODE_SECTION(dfe_mapDevice, ".text:tools");
#pragma CODE_SECTION(dfe_unmapDevice, ".text:tools");
#pragma CODE_SECTION(serdesCfg0_mapDevice, ".text:tools");
#pragma CODE_SECTION(serdesCfg0_unmapDevice, ".text:tools");
#pragma CODE_SECTION(serdesCfg1_mapDevice, ".text:tools");
#pragma CODE_SECTION(serdesCfg1_unmapDevice, ".text:tools");
#pragma CODE_SECTION(bootCfg_mapDevice, ".text:tools");
#pragma CODE_SECTION(bootCfg_unmapDevice, ".text:tools");
#pragma CODE_SECTION(pscEnable, ".text:tools");
#pragma CODE_SECTION(pscDisable, ".text:tools");
#pragma CODE_SECTION(pscPowerOff, ".text:tools");
#pragma CODE_SECTION(pscDisableResetIso, ".text:tools");
#pragma CODE_SECTION(serdesCfg0RestoreDefault, ".text:tools");
#pragma CODE_SECTION(serdesCfg1RestoreDefault, ".text:tools");
#pragma CODE_SECTION(iqn2SerdesGetLaneCtrlRate, ".text:tools");
#endif

#ifndef USESYSBIOS
interrupt
#endif
void           dummyIsr();

UTILS_FxnPtr   atevt0_userIsr     = dummyIsr;
UTILS_FxnPtr   atevt1_userIsr     = dummyIsr;
UTILS_FxnPtr   atevt2_userIsr     = dummyIsr;
UTILS_FxnPtr   iqn2ev0except_userIsr = NULL;
UTILS_FxnPtr   dfeev1except_userIsr = NULL;
UTILS_FxnPtr   iqn2pktdmaexcept_userIsr = NULL;

#ifndef USESYSBIOS
interrupt
#endif
void dummyIsr() {}

#endif

void UTILS_fillIqn2Obj (TestObj *testObjTab, IQN2_ConfigHandle hIqn2)
{
    uint32_t numAil, i, axcOffset;
    uint32_t multiradstdOffset = 0;

    axcOffset = 0;

    hIqn2->protocol = testObjTab->protocol;
    for (numAil=0; numAil<IQN2_MAX_NUM_AIL;numAil++)
    {
        if ((testObjTab->numWcdmaAxC[numAil]!= 0) || (testObjTab->numLte20AxC[numAil]) || (testObjTab->numLte10AxC[numAil]) || (testObjTab->numLte5AxC[numAil]) || (testObjTab->numLte15AxC[numAil]))
            hIqn2->ailConfig[numAil].ailEnable = 1;
        hIqn2->ailConfig[numAil].linkRate = testObjTab->linkRate[numAil];
        hIqn2->ailConfig[numAil].numWcdmaPeAxC = testObjTab->numWcdmaAxC[numAil];
        hIqn2->ailConfig[numAil].numWcdmaPdAxC = testObjTab->numWcdmaAxC[numAil];
        hIqn2->ailConfig[numAil].numLtePeAxC = testObjTab->numLte20AxC[numAil] + testObjTab->numLte10AxC[numAil] + testObjTab->numLte5AxC[numAil] + testObjTab->numLte15AxC[numAil];
        hIqn2->ailConfig[numAil].numLtePdAxC = hIqn2->ailConfig[numAil].numLtePeAxC;
        hIqn2->ailConfig[numAil].firstWcdmaAxC = 0;
        axcOffset += testObjTab->numWcdmaAxC[numAil];
        hIqn2->ailConfig[numAil].firstLteAxC = axcOffset;
        axcOffset += hIqn2->ailConfig[numAil].numLtePeAxC;
        hIqn2->ailConfig[numAil].pe2Offset = testObjTab->peOffset[numAil];
        hIqn2->ailConfig[numAil].deltaOffset = hIqn2->ailConfig[numAil].pe2Offset + 80;

        for(i=hIqn2->ailConfig[numAil].firstWcdmaAxC; i<hIqn2->ailConfig[numAil].firstWcdmaAxC + hIqn2->ailConfig[numAil].numWcdmaPeAxC; i++)
        {
            hIqn2->AxCconfig[i].sampleRate = IQN2_SRATE_3P84MHZ;
#ifdef WCDMA_UL
            if(hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
            {
                hIqn2->AxCconfig[i].inboundDataWidth = IQN2FL_DATA_WIDTH_8_BIT;
                hIqn2->AxCconfig[i].outboundDataWidth = IQN2FL_DATA_WIDTH_8_BIT;
            } else {
                hIqn2->AxCconfig[i].inboundDataWidth = IQN2FL_DATA_WIDTH_8_BIT;
                hIqn2->AxCconfig[i].outboundDataWidth = IQN2FL_DATA_WIDTH_8_BIT;
            }
#else
            if(hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)
            {
                hIqn2->AxCconfig[i].inboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
                hIqn2->AxCconfig[i].outboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            } else {
                hIqn2->AxCconfig[i].inboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
                hIqn2->AxCconfig[i].outboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            }
#endif
        }
        multiradstdOffset = hIqn2->ailConfig[numAil].firstLteAxC;
        for(i=0; i < testObjTab->numLte20AxC[numAil]; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_30P72MHZ;
            hIqn2->AxCconfig[multiradstdOffset].cpriPackMode =IQN2_LTE_CPRI_8b8;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            multiradstdOffset++;
        }
        for(i=0; i < testObjTab->numLte10AxC[numAil]; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_15P36MHZ;
            hIqn2->AxCconfig[multiradstdOffset].cpriPackMode =IQN2_LTE_CPRI_4b4;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            multiradstdOffset++;
        }
        for(i=0; i < testObjTab->numLte5AxC[numAil]; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_7P68MHZ;
            hIqn2->AxCconfig[multiradstdOffset].cpriPackMode =IQN2_LTE_CPRI_2b2;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            multiradstdOffset++;
        }
        for(i=0; i < testObjTab->numLte15AxC[numAil]; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_23P04MHZ;
            hIqn2->AxCconfig[multiradstdOffset].cpriPackMode =IQN2_LTE_CPRI_6b6;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            multiradstdOffset++;
        }
        hIqn2->ailConfig[numAil].comType = testObjTab->comType[numAil];
    }
}

void UTILS_fillIqn2Aid2Obj (TestAidObj *testObjTab, IQN2_ConfigHandle hIqn2)
{
    uint32_t i, axcOffset;
    uint32_t multiradstdOffset = 0;

    axcOffset = 0;
    hIqn2->protocol = testObjTab->protocol;
    if ((testObjTab->numWcdmaAxC!= 0) || (testObjTab->numLte20AxC) || (testObjTab->numLte10AxC) || (testObjTab->numLte5AxC) || (testObjTab->numLte15AxC) || (testObjTab->numLte40AxC) || (testObjTab->numLte80AxC))
    {
        hIqn2->aidConfig.aidEnable = 1;
        hIqn2->aidConfig.numWcdmaEgressAxC = testObjTab->numWcdmaAxC;
        hIqn2->aidConfig.numWcdmaIngressAxC = testObjTab->numWcdmaAxC;
        hIqn2->aidConfig.numLteEgressAxC = testObjTab->numLte20AxC + testObjTab->numLte10AxC + testObjTab->numLte5AxC + testObjTab->numLte15AxC + testObjTab->numLte40AxC + testObjTab->numLte80AxC;
        hIqn2->aidConfig.numLteIngressAxC = hIqn2->aidConfig.numLteEgressAxC;
        hIqn2->aidConfig.firstWcdmaAxC = hIqn2->aidConfig.numLteEgressAxC;
        axcOffset += testObjTab->numWcdmaAxC;
//        hIqn2->aidConfig.firstLteAxC = axcOffset;
        axcOffset += hIqn2->aidConfig.numLteEgressAxC;

        for(i=hIqn2->aidConfig.firstWcdmaAxC; i<hIqn2->aidConfig.firstWcdmaAxC + hIqn2->aidConfig.numWcdmaEgressAxC; i++)
        {
            hIqn2->AxCconfig[i].sampleRate = IQN2_SRATE_3P84MHZ;
#ifdef WCDMA_UL
            hIqn2->AxCconfig[i].inboundDataWidth = IQN2FL_DATA_WIDTH_8_BIT;
            hIqn2->AxCconfig[i].outboundDataWidth = IQN2FL_DATA_WIDTH_8_BIT;
#else
            hIqn2->AxCconfig[i].inboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
            hIqn2->AxCconfig[i].outboundDataWidth = IQN2FL_DATA_WIDTH_15_BIT;
#endif
        }
        multiradstdOffset = hIqn2->aidConfig.firstLteAxC;
        for(i=0; i < testObjTab->numLte20AxC; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_30P72MHZ;
            hIqn2->AxCconfig[multiradstdOffset].cpriPackMode =IQN2_LTE_CPRI_8b8;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            multiradstdOffset++;
        }
        for(i=0; i < testObjTab->numLte15AxC; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_23P04MHZ;
            hIqn2->AxCconfig[multiradstdOffset].cpriPackMode =IQN2_LTE_CPRI_6b6;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            multiradstdOffset++;
        }
        for(i=0; i < testObjTab->numLte10AxC; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_15P36MHZ;
            hIqn2->AxCconfig[multiradstdOffset].cpriPackMode =IQN2_LTE_CPRI_4b4;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            multiradstdOffset++;
        }
        for(i=0; i < testObjTab->numLte5AxC; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_7P68MHZ;
            hIqn2->AxCconfig[multiradstdOffset].cpriPackMode =IQN2_LTE_CPRI_2b2;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            multiradstdOffset++;
        }
        for(i=0; i < testObjTab->numLte40AxC; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_61P44MHZ;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            multiradstdOffset++;
        }
        for(i=0; i < testObjTab->numLte80AxC; i++)
        {
            hIqn2->AxCconfig[multiradstdOffset].sampleRate = IQN2_SRATE_122P88MHZ;
            hIqn2->AxCconfig[multiradstdOffset].inboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            hIqn2->AxCconfig[multiradstdOffset].outboundDataWidth = IQN2FL_DATA_WIDTH_16_BIT;
            multiradstdOffset++;
        }
    }
}


/* Spin in a delay loop */
void
UTILS_waitForHw(
   uint32_t delay
)
{
    volatile uint32_t i, n;

    n = 0;
    for (i = 0; i < delay; i++)
    {
        n = n + 1;
    }
}

#ifdef _TMS320C6X
// 10ms frame boundary from PhyTimer
#ifndef USESYSBIOS
interrupt
#endif
void
Iqn2_AT_evt0_ISR()
{

	atevt0_userIsr();
}

// 10ms frame boundary from PhyTimer
#ifndef USESYSBIOS
interrupt
#endif
void
Iqn2_AT_evt1_ISR()
{
	atevt1_userIsr();
}

// 10ms frame boundary from PhyTimer
#ifndef USESYSBIOS
interrupt
#endif
void
Iqn2_AT_evt2_ISR()
{
	atevt2_userIsr();
}

/*
* IQN2 exception handler
* Enter this one only a 1000 times per 10ms frame
* And allow test case to complete without hanging in the exceptions
*/
#ifndef RUNTIME
static uint32_t iqn2EEventCnt = 0;
static Int32  iqn2EFrameCur  = -1;
static uint32_t iqn2ECpriLossCnt = 0;
uint32_t        eeLastCpriLossInFrame = 0;
#endif

#ifndef USESYSBIOS
interrupt
#endif
void
Iqn2_EV0Exception_ISR()
{
    iqn2ev0except_userIsr();
}

#ifndef USESYSBIOS
interrupt
#endif
void
Dfe_EV1Exception_ISR()
{
    dfeev1except_userIsr();
}

#ifndef USESYSBIOS
interrupt
#endif
void
Iqn2_PktDMAException_ISR()
{
    iqn2pktdmaexcept_userIsr();
}
#ifndef RUNTIME
//uint8_t tm_status[100];
//uint16_t excep_ts[100];
void
UTILS_getIqn2EV0Exception()
{
uint8_t eoi = 0;

      if (iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS>iqn2EFrameCur) {
            iqn2EFrameCur = iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
            iqn2EEventCnt = 0;
      }

      /*if (iqn2LldObj.ailConfig[0].ailEnable)
          tm_status[iqn2EEventCnt] = CSL_FEXT(iqn2LldObj.hFl->regs->Ail[0].AIL_PHY_TM.AIL_PHY_TM_STS,IQN_AIL_AIL_PHY_TM_STS_FRM_STATE);
       */
      /* Do the required servicing here */
      IQN2_getException(&iqn2LldObj);

      /*if ((iqn2LldObj.aidConfig.aidEnable)&&(iqn2LldObj.iqn2EeCount.eeAid2.aid2EeSiiCCnt.si_ing_iq_sof_err)) {
          excep_ts[iqn2EEventCnt] = iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
      }
       */

      /* Tag at which frame last Cpri los occurred */
      if ((iqn2LldObj.iqn2EeCount.eeAil[0].ailEeRm0Cnt.los_err > 0) || (iqn2LldObj.iqn2EeCount.eeAil[1].ailEeRm0Cnt.los_err > 0))
      {
          if (iqn2ECpriLossCnt < iqn2LldObj.iqn2EeCount.eeAil[0].ailEeRm0Cnt.los_err)
          {
              eeLastCpriLossInFrame = iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
              iqn2ECpriLossCnt      = iqn2LldObj.iqn2EeCount.eeAil[0].ailEeRm0Cnt.los_err;
          }
          if (iqn2ECpriLossCnt < iqn2LldObj.iqn2EeCount.eeAil[1].ailEeRm0Cnt.los_err)
          {
              eeLastCpriLossInFrame = iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
              iqn2ECpriLossCnt      = iqn2LldObj.iqn2EeCount.eeAil[1].ailEeRm0Cnt.los_err;
          }
      }

      if (iqn2LldObj.timerSyncSource != IQN2_DIAG_SW_SYNC)
      {
          // Check if resync is required - doing it once when first sync signal comes
          if (ext_sync == 0) {
              ext_sync = IQN2_resyncProcedure(&iqn2LldObj,0); // no extra delay adjustments for LLD unit tests
              if (ext_sync == 1) {
                  IQN2_enableRadioTimers(&iqn2LldObj);
              }
          } else {
              IQN2_disableAt2Exception(&iqn2LldObj,0);
          }
      }

      /* Re-arming for next interruption */
      if (iqn2EEventCnt < 100) Iqn2Fl_hwControl(iqn2LldObj.hFl, IQN2FL_CMD_EE_EOI_0_REG, (void *)&eoi);
      iqn2EEventCnt++;
}

void
UTILS_getDfeEV1Exception()
{
uint8_t eoi = 0;

      if (iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS>iqn2EFrameCur) {
            iqn2EFrameCur = iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
            iqn2EEventCnt = 0;
      }

      /* Do the required servicing here */
#if defined DFE_BB_LOOPBACK || defined DFE_JESD_LOOPBACK || defined DFE_NORMAL_OPERATION
      Dfe_getException(&dfeLldObj);
#endif
      IQN2_getDfeException(&iqn2LldObj,1);

      /* Re-arming for next interruption */
      if (iqn2EEventCnt < 100) Iqn2Fl_hwControl(iqn2LldObj.hFl, IQN2FL_CMD_EE_EOI_1_REG, (void *)&eoi);
      iqn2EEventCnt++;
}

void
UTILS_getIqn2PktDMAException()
{
uint8_t eoi = 0;

      if (iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS>iqn2EFrameCur) {
            iqn2EFrameCur = iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
            iqn2EEventCnt = 0;
      }

      /* For interrupts routed through the chip-level INTC
      * Disable CPINTC0 Host interrupt (CPINTC output)
      */
      CSL_CPINTC_disableHostInterrupt((CSL_CPINTC_Handle)CSL_CIC_0_REGS, 8);
      /* Clear the CPINTC0 Interrupt */
      CSL_CPINTC_clearSysInterrupt((CSL_CPINTC_Handle)CSL_CIC_0_REGS,CSL_CIC0_IQNET_PKTDMA_STARVE);
      /* Do the required servicing here */
      IQN2_getException(&iqn2LldObj);
      /* Enable CPINTC0 Host interrupt (CPINTC output) */
      CSL_CPINTC_enableHostInterrupt((CSL_CPINTC_Handle)CSL_CIC_0_REGS, 8);

      /* Re-arming for next interruption */
      if (iqn2EEventCnt < 100) Iqn2Fl_hwControl(iqn2LldObj.hFl, IQN2FL_CMD_EE_EOI_CPPI_REG, (void *)&eoi);
      iqn2EEventCnt++;
}
#endif

void
UTILS_iqn2IntcSetup()
{

    /* Routing Iqn2 exception event thru INTC0 */
    hIntc0Handle = CSL_CPINTC_open(0); // handle for INTC0
    CSL_CPINTC_disableAllHostInterrupt(hIntc0Handle);
    CSL_CPINTC_setNestingMode(hIntc0Handle, CPINTC_NO_NESTING);
    CSL_CPINTC_mapSystemIntrToChannel (hIntc0Handle, CSL_CIC0_IQNET_PKTDMA_STARVE , 8); // assuming core0
    CSL_CPINTC_clearSysInterrupt (hIntc0Handle, CSL_CIC0_IQNET_PKTDMA_STARVE );
    CSL_CPINTC_enableSysInterrupt (hIntc0Handle, CSL_CIC0_IQNET_PKTDMA_STARVE );

#ifndef USESYSBIOS
    CSL_IntcParam                   vectId;
  	CSL_IntcGlobalEnableState       state;

	intcContext.eventhandlerRecord = EventRecord;
	intcContext.numEvtEntries = 16;
	CSL_intcInit(&intcContext);

	/* map AIF2_SEVT7 for time tracking - 10ms tick */
	vectId = COM_IQN2_AT_EVT0_EVENT;
	/* Opening a intc handle for this event */
	hIntc[COM_IQN2_AT_EVT0_EVENT] = CSL_intcOpen (&intcObj[COM_IQN2_AT_EVT0_EVENT], CSL_C66X_COREPAC_IQNET_ATEVT_0, &vectId , NULL);
	// hook ISR for this CPU interrupt
	CSL_intcHookIsr(COM_IQN2_AT_EVT0_EVENT,&Iqn2_AT_evt0_ISR);

	/* map AIF2_SEVT7 for time tracking - 10ms tick */
	vectId = COM_IQN2_AT_EVT1_EVENT;
	/* Opening a intc handle for this event */
	hIntc[COM_IQN2_AT_EVT1_EVENT] = CSL_intcOpen (&intcObj[COM_IQN2_AT_EVT1_EVENT], CSL_C66X_COREPAC_IQNET_ATEVT_1, &vectId , NULL);
	// hook ISR for this CPU interrupt
	CSL_intcHookIsr(COM_IQN2_AT_EVT1_EVENT,&Iqn2_AT_evt1_ISR);

	/* map AIF2_SEVT7 for time tracking - 10ms tick */
	vectId = COM_IQN2_AT_EVT2_EVENT;
	/* Opening a intc handle for this event */
	hIntc[COM_IQN2_AT_EVT2_EVENT] = CSL_intcOpen (&intcObj[COM_IQN2_AT_EVT2_EVENT], CSL_C66X_COREPAC_IQNET_ATEVT_2, &vectId , NULL);
	// hook ISR for this CPU interrupt
	CSL_intcHookIsr(COM_IQN2_AT_EVT2_EVENT,&Iqn2_AT_evt2_ISR);

	/* map IQN2_EXCEPTION for Debug*/
    vectId = COM_IQN2_EV0EXCEP_EVENT;
    /* Opening a intc handle for this event */
    hIntc[COM_IQN2_EV0EXCEP_EVENT] = CSL_intcOpen (&intcObj[COM_IQN2_EV0EXCEP_EVENT], CSL_C66X_COREPAC_IQNET_INT0, &vectId , NULL);
    // hook ISR for this CPU interrupt
    CSL_intcHookIsr(COM_IQN2_EV0EXCEP_EVENT,&Iqn2_EV0Exception_ISR);

    /* map DFE_EXCEPTION for Debug*/
    vectId = COM_DFE_EV1EXCEP_EVENT;
    /* Opening a intc handle for this event */
    hIntc[COM_DFE_EV1EXCEP_EVENT] = CSL_intcOpen (&intcObj[COM_DFE_EV1EXCEP_EVENT], CSL_C66X_COREPAC_IQNET_INT1, &vectId , NULL);
    // hook ISR for this CPU interrupt
    CSL_intcHookIsr(COM_DFE_EV1EXCEP_EVENT,&Dfe_EV1Exception_ISR);

    /* map IQN2_EXCEPTION for Debug*/
    vectId = COM_IQN2_PKTDMAEXCEP_EVENT;
    /* Opening a intc handle for this event */
    hIntc[COM_IQN2_PKTDMAEXCEP_EVENT] = CSL_intcOpen (&intcObj[COM_IQN2_PKTDMAEXCEP_EVENT], CSL_C66X_COREPAC_CIC_OUT8_PLUS_16_MUL_N, &vectId , NULL);
    // hook ISR for this CPU interrupt
    CSL_intcHookIsr(COM_IQN2_PKTDMAEXCEP_EVENT,&Iqn2_PktDMAException_ISR);


	/* Clear the Event & the interrupt */
	CSL_intcHwControl(hIntc[COM_IQN2_AT_EVT0_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
	CSL_intcHwControl(hIntc[COM_IQN2_AT_EVT1_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
	CSL_intcHwControl(hIntc[COM_IQN2_AT_EVT2_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
    CSL_intcHwControl(hIntc[COM_IQN2_EV0EXCEP_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
	CSL_intcHwControl(hIntc[COM_DFE_EV1EXCEP_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
	CSL_CPINTC_clearSysInterrupt((CSL_CPINTC_Handle)CSL_CIC_0_REGS,CSL_CIC0_IQNET_PKTDMA_STARVE);
	CSL_intcHwControl(hIntc[COM_IQN2_PKTDMAEXCEP_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);

    /* Configure the combiner 0 masks for FSYNC1 */
  	//NA use direct CSL_GEM_AIF_EVENT7 so disable all combined event
  	//IntcRegs->EVTMASK[0]= 	~((1<<CSL_INTC_EVENTID_FSEVT1));
	IntcRegs->EVTMASK[0]= 	0xFFFFFFFF; 	//mask out other events
	IntcRegs->EVTMASK[1]= 	0xFFFFFFFF; 	//mask out other events
	IntcRegs->EVTMASK[2]= 	0xFFFFFFFF; 	//mask out other events
	IntcRegs->EVTMASK[3]= 	0xFFFFFFFF; 	//mask out other events

	/* Enable the Event & the interrupt */
    // Enable global host interrupts flag
	CSL_CPINTC_enableAllHostInterrupt(hIntc0Handle);
	CSL_intcHwControl(hIntc[COM_IQN2_AT_EVT0_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
	CSL_intcHwControl(hIntc[COM_IQN2_AT_EVT1_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
	CSL_intcHwControl(hIntc[COM_IQN2_AT_EVT2_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
	if (iqn2ev0except_userIsr != NULL)
	CSL_intcHwControl(hIntc[COM_IQN2_EV0EXCEP_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
	if (dfeev1except_userIsr != NULL)
	CSL_intcHwControl(hIntc[COM_DFE_EV1EXCEP_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
	if (iqn2pktdmaexcept_userIsr != NULL)
	CSL_intcHwControl(hIntc[COM_IQN2_PKTDMAEXCEP_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);

	CSL_intcGlobalEnable(&state);
	/* Enable NMIs */
	CSL_intcGlobalNmiEnable();
#else
	// modifications for BIOS6 usage
	Hwi_Params hwiParams;
	Hwi_disable();

	Hwi_disableInterrupt(COM_IQN2_AT_EVT0_EVENT);
	Hwi_disableInterrupt(COM_IQN2_AT_EVT1_EVENT);
	Hwi_disableInterrupt(COM_IQN2_AT_EVT2_EVENT);
	Hwi_disableInterrupt(COM_IQN2_PKTDMAEXCEP_EVENT);
	Hwi_disableInterrupt(COM_DFE_EV1EXCEP_EVENT);
	Hwi_disableInterrupt(COM_IQN2_EV0EXCEP_EVENT);


		/* Map events to CPU interrupts */
		/* And plug ISRs into BIOS dispatcher */

	Hwi_Params_init(&hwiParams);
	hwiParams.enableInt = FALSE;
	hwiParams.eventId = CSL_C66X_COREPAC_IQNET_ATEVT_0;
	Hwi_create (COM_IQN2_AT_EVT0_EVENT, (ti_sysbios_interfaces_IHwi_FuncPtr) Iqn2_AT_evt0_ISR, &hwiParams, NULL);

	hwiParams.eventId = CSL_C66X_COREPAC_IQNET_ATEVT_1;
	Hwi_create (COM_IQN2_AT_EVT1_EVENT, (ti_sysbios_interfaces_IHwi_FuncPtr) Iqn2_AT_evt1_ISR, &hwiParams, NULL);

	hwiParams.eventId = CSL_C66X_COREPAC_IQNET_ATEVT_2;
	Hwi_create (COM_IQN2_AT_EVT2_EVENT, (ti_sysbios_interfaces_IHwi_FuncPtr) Iqn2_AT_evt2_ISR, &hwiParams, NULL);

	hwiParams.eventId = CSL_C66X_COREPAC_IQNET_INT0;
	Hwi_create (COM_IQN2_EV0EXCEP_EVENT, (ti_sysbios_interfaces_IHwi_FuncPtr) Iqn2_EV0Exception_ISR, &hwiParams, NULL);

	hwiParams.eventId = CSL_C66X_COREPAC_CIC_OUT8_PLUS_16_MUL_N;
	Hwi_create (COM_IQN2_PKTDMAEXCEP_EVENT, (ti_sysbios_interfaces_IHwi_FuncPtr) Iqn2_PktDMAException_ISR, &hwiParams, NULL);

	Hwi_clearInterrupt (COM_IQN2_AT_EVT0_EVENT);
	Hwi_clearInterrupt (COM_IQN2_AT_EVT1_EVENT);
	Hwi_clearInterrupt (COM_IQN2_AT_EVT2_EVENT);
	Hwi_clearInterrupt (COM_IQN2_EV0EXCEP_EVENT);
	Hwi_clearInterrupt (COM_DFE_EV1EXCEP_EVENT);
	Hwi_clearInterrupt (COM_IQN2_PKTDMAEXCEP_EVENT);

	CSL_CPINTC_enableAllHostInterrupt (hIntc0Handle);

	Hwi_enableInterrupt (COM_IQN2_AT_EVT0_EVENT);
	Hwi_enableInterrupt (COM_IQN2_AT_EVT1_EVENT);
	Hwi_enableInterrupt (COM_IQN2_AT_EVT2_EVENT);
	if (iqn2ev0except_userIsr != NULL)
	Hwi_enableInterrupt (COM_IQN2_EV0EXCEP_EVENT);
	if (dfeev1except_userIsr != NULL)
	Hwi_enableInterrupt (COM_DFE_EV1EXCEP_EVENT);
	if (iqn2pktdmaexcept_userIsr != NULL)
	Hwi_enableInterrupt (COM_IQN2_PKTDMAEXCEP_EVENT);

	Hwi_enable ();
#endif
}

void
UTILS_iqn2ExceptIntDisable()
{
#ifndef USESYSBIOS
    CSL_intcHwControl(hIntc[COM_IQN2_EV0EXCEP_EVENT], CSL_INTC_CMD_EVTDISABLE, NULL);
	CSL_intcHwControl(hIntc[COM_DFE_EV1EXCEP_EVENT], CSL_INTC_CMD_EVTDISABLE, NULL);
    CSL_intcHwControl(hIntc[COM_IQN2_PKTDMAEXCEP_EVENT], CSL_INTC_CMD_EVTDISABLE, NULL);
#else
    Hwi_disableInterrupt(COM_IQN2_EV0EXCEP_EVENT);
	Hwi_disableInterrupt(COM_DFE_EV1EXCEP_EVENT);
    Hwi_disableInterrupt(COM_IQN2_PKTDMAEXCEP_EVENT);
#endif
}


/* Cache operations */

void UTILS_setCache()
{
    uint32_t  key;

    // Disable Interrupts
    key = _disable_interrupts();

#ifndef USESYSBIOS

    //  Cleanup the prefetch buffer.
    CSL_XMC_invalidatePrefetchBuffer();

    // Change cache sizes
    CACHE_setL1DSize (CACHE_L1_32KCACHE);
    CACHE_setL1PSize (CACHE_L1_32KCACHE);

#else
    Cache_Size size;

    size.l1pSize = Cache_L1Size_32K;
    size.l1dSize = Cache_L1Size_32K;
    size.l2Size  = Cache_L2Size_0K;
    Cache_setSize(&size);

    ti_sysbios_family_c66_Cache_invPrefetchBuffer();
    Cache_setMar((xdc_Ptr*)0x0c000000,0x00200000,Cache_Mar_ENABLE);
#endif

    // Reenable Interrupts.
    _restore_interrupts(key);

}

void UTILS_cacheInvalidate(void* ptr, uint32_t size)
{
    uint32_t  key;

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0xA0000000) == 0xA0000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYSBIOS
        printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        System_printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0x80000000) == 0x80000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYSBIOS
        printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        System_printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if MSMC address is 64 byte aligned */
    if ((((int)ptr & 0x0C000000) == 0x0C000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYSBIOS
        printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        System_printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    // Disable Interrupts
    key = _disable_interrupts();

#ifndef USESYSBIOS
    //  Cleanup the prefetch buffer also.
    CSL_XMC_invalidatePrefetchBuffer();
    // Invalidate the cache.
    CACHE_invL1d(ptr, size, CACHE_FENCE_WAIT);
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
#else
    ti_sysbios_family_c66_Cache_invPrefetchBuffer();
    Cache_inv((void *)ptr, size, Cache_Type_L1D, 1);
#endif

    // Reenable Interrupts.
    _restore_interrupts(key);
}

void UTILS_cacheWriteBack(void* ptr, uint32_t size)
{
    uint32_t  key;

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0xA0000000) == 0xA0000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYSBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        System_printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0x80000000) == 0x80000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if MSMC address is 64 byte aligned */
    if ((((int)ptr & 0x0C000000) == 0x0C000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    // Disable Interrupts
    key = _disable_interrupts();
#ifndef USESYSBIOS
    // Writeback the contents of the cache.
    CACHE_wbL1d(ptr, size, CACHE_FENCE_WAIT);
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
#else
    // Writeback the contents of the cache.
    Cache_wb((void *)ptr, size, Cache_Type_L1D, 1);
#endif
    // Reenable Interrupts.
    _restore_interrupts(key);
}

void UTILS_cacheWriteBackInvalidate(void* ptr, uint32_t size)
{
    uint32_t  key;

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0xA0000000) == 0xA0000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0x80000000) == 0x80000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if MSMC address is 64 byte aligned */
    if ((((int)ptr & 0x0C000000) == 0x0C000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    // Disable Interrupts
    key = _disable_interrupts();
#ifndef USESYSBIOS
    // Writeback and Invalidate the contents of the cache.
    CACHE_wbInvL1d(ptr, size, CACHE_FENCE_WAIT);
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
#else
    // Writeback and Invalidate the contents of the cache.
    Cache_wbInv((void *)ptr, size, Cache_Type_L1D, 1);
#endif
    // Reenable Interrupts.
    _restore_interrupts(key);
}

void
UTILS_resetTimer(
        Uint32 timer
)
{
CSL_Status     status;
CSL_TmrEnamode tmrMode;

   tmrMode = CSL_TMR_ENAMODE_DISABLE;
   if (hTmr == NULL) {
      CSL_tmrInit(NULL);
      hTmr = CSL_tmrOpen(&TmrObj,timer,NULL,&status);
   }
   CSL_tmrHwControl(hTmr,CSL_TMR_CMD_STOP64,&tmrMode);
   hTmr->regs->TGCR  = 0x00000000;
   hTmr->regs->TCR   = 0x00000000;
   if (hTmr != NULL){CSL_tmrClose(hTmr);hTmr=NULL;}
}

void
UTILS_startTimer(
)
{
    CSL_TmrEnamode tmrMode;

#if 1
    // one shot timer in clock mode as signal integrity of the setup is not optimal for timer 0 output -> physync path
    tmrMode = CSL_TMR_ENAMODE_ENABLE;
#else
    tmrMode = CSL_TMR_ENAMODE_CONT;
#endif
    //TmrSetup.tmrTimerPeriodLo  = prdLo;
    /*------------------------------------------------------------------*/
    /* Start timer in continuous mode                                   */
    /*------------------------------------------------------------------*/
    if (hTmr != NULL) CSL_tmrHwControl(hTmr,CSL_TMR_CMD_START64,&tmrMode);
#ifndef USESYSBIOS
    else printf (" ERROR no timer start \n");
#else
    else System_printf (" ERROR no timer start \n");
#endif

}

//Timer is set as a parameter to allow the user to choose any timer for EVM external physync or radsync.
void
UTILS_initTimer(
        Uint32 timer
)

{
    /*init timer for EVM external sync */
#if 1
    Uint32 prdLo = 0x00000200;
#else
    Uint32 prdLo = 0x00190000; // 10 ms, timer runs at CPU/6
#endif

    Uint32 prdHi = 0x00000000;
    Uint32 cntLo = 0x00000000;
    Uint32 cntHi = 0x00000000;
    CSL_Status status;

    // Module Initialization
    CSL_tmrInit(NULL);

    //Open Handle
    hTmr = CSL_tmrOpen(&TmrObj,timer,NULL,&status);

    CSL_tmrHwControl(hTmr,CSL_TMR_CMD_RESET_TIMLO,NULL);
    CSL_tmrHwControl(hTmr,CSL_TMR_CMD_RESET_TIMHI,NULL);

    CSL_tmrGetHwSetup(hTmr, &TmrSetup);

    TmrSetup.tmrClksrcLo       = CSL_TMR_CLKSRC_INTERNAL;
#if 1
    TmrSetup.tmrClockPulseLo   = CSL_TMR_CP_CLOCK;
#else
    TmrSetup.tmrClockPulseLo   = CSL_TMR_CP_PULSE;
#endif
    TmrSetup.tmrInvInpLo       = CSL_TMR_INVINP_INVERTED;
    TmrSetup.tmrInvOutpLo      = CSL_TMR_INVOUTP_INVERTED;
    TmrSetup.tmrIpGateLo       = CSL_TMR_CLOCK_INP_NOGATE;
    TmrSetup.tmrPulseWidthLo   = CSL_TMR_PWID_THREECLKS;
    TmrSetup.tmrTimerPeriodLo  = prdLo;
    TmrSetup.tmrClksrcHi      = CSL_TMR_CLKSRC_INTERNAL;
#if 1
    TmrSetup.tmrClockPulseHi   = CSL_TMR_CP_CLOCK;
#else
    TmrSetup.tmrClockPulseHi   = CSL_TMR_CP_PULSE;
#endif
    TmrSetup.tmrInvInpHi       = CSL_TMR_INVINP_INVERTED;
    TmrSetup.tmrInvOutpHi      = CSL_TMR_INVOUTP_INVERTED;
    TmrSetup.tmrIpGateHi       = CSL_TMR_CLOCK_INP_NOGATE;
    TmrSetup.tmrPulseWidthHi   = CSL_TMR_PWID_THREECLKS;
    TmrSetup.tmrTimerPeriodHi  = prdHi;
    TmrSetup.tmrTimerMode      = CSL_TMR_TIMMODE_GPT;

      /*------------------------------------------------------------------*/
      /* Set count to 0                         */
      /*------------------------------------------------------------------*/
    TmrSetup.tmrTimerCounterLo     = cntLo;
    TmrSetup.tmrTimerCounterHi     = cntHi;
    CSL_tmrHwSetup(hTmr,&TmrSetup);

    {
        uint8_t tout0,tout1,tout2,tout3;
        CSL_BootCfgUnlockKicker();
        CSL_BootCfgGetTimerOutputSelection0(&tout0,&tout1,&tout2,&tout3);
        CSL_BootCfgSetTimerOutputSelection0(tout0,(uint8_t)(2*timer),tout2,tout3);
        //CSL_FINS(*(volatile int*)0x026202F8,BOOTCFG_TOUTPSEL0_TOUTPSEL1,(2*timer));
        // Uboot unlocks the kicker and Linux expects the kicker to be unlocked
        // CSL_BootCfgLockKicker();
    }
}

void
UTILS_triggerExternalSync(
        IQN2_ConfigHandle         hIqn2
)
{
    if (hIqn2->timerSyncSource != IQN2_DIAG_SW_SYNC) {
        UTILS_startTimer();
    }
}

#endif // _TMS320C6X

#ifdef _VIRTUAL_ADDR_SUPPORT
static int dev_mem_fd;
#endif

uint32_t iqn2_mmap(uint32_t addr, uint32_t size)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
        uint32_t virt_addr;
        uint32_t page_size;
        page_size = sysconf(_SC_PAGE_SIZE);
        if (size%page_size)
        {
#ifndef USESYSBIOS
                printf("Size does not align with page size. Size given: %d\n", size);
#else
                System_printf("Size does not align with page size. Size given: %d\n", size);
#endif
                return 0;
        }
        if ((uint32_t)addr % page_size)
        {
#ifndef USESYSBIOS
                printf("Address does not align with page size. Address given: 0x%08x\n", (uint32_t) addr);
#else
                System_printf("Address does not align with page size. Address given: 0x%08x\n", (uint32_t) addr);
#endif
                return 0;
        }
        virt_addr = (uint32_t) mmap(0, size, (PROT_READ|PROT_WRITE), MAP_SHARED, dev_mem_fd, (off_t)addr);
        if (virt_addr == -1)
        {
#ifndef USESYSBIOS
                printf("mmap failed!\n");
#else
                System_printf("mmap failed!\n");
#endif
                return 0;
        }
        return virt_addr;
#else
    return addr;
#endif
}

uint32_t iqn2_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
#ifndef USESYSBIOS
        printf("Failed to open /dev/mem \n");
#else
        System_printf("Failed to open /dev/mem \n");
#endif
        return -1;
    }
    iqn2InitCfg.dev.bases[0].cfgBase = (void *) iqn2_mmap((uint32_t)(iqn2InitCfg.dev.bases[0].cfgBase), 0x400000);
    return 0;
#else
    return 0;
#endif
}

uint32_t iqn2_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap(iqn2InitCfg.dev.bases[0].cfgBase, 0x400000);
    close(dev_mem_fd);
#endif
    return 0;
}

extern uint32_t dfe_cfg_base;
uint32_t dfe_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
#ifndef USESYSBIOS
        printf("Failed to open /dev/mem \n");
#else
        System_printf("Failed to open /dev/mem \n");
#endif
        return -1;
    }
    dfe_cfg_base = iqn2_mmap((uint32_t)(dfe_cfg_base), 0x2000000);
    return 0;
#else
    return 0;
#endif
}

uint32_t dfe_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap((uint32_t*)dfe_cfg_base, 0x2000000);
    close(dev_mem_fd);
#endif
    return 0;
}

extern uint32_t serdes_cfg0_base;
uint32_t serdesCfg0_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
#ifndef USESYSBIOS
        printf("Failed to open /dev/mem \n");
#else
        System_printf("Failed to open /dev/mem \n");
#endif
        return -1;
    }
    serdes_cfg0_base = iqn2_mmap((uint32_t)(serdes_cfg0_base), 0x2000);
    return 0;
#else
    return 0;
#endif
}

uint32_t serdesCfg0_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap((uint32_t*)serdes_cfg0_base, 0x2000);
    close(dev_mem_fd);
#endif
    return 0;
}

extern uint32_t serdes_cfg1_base;
uint32_t serdesCfg1_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
#ifndef USESYSBIOS
        printf("Failed to open /dev/mem \n");
#else
        System_printf("Failed to open /dev/mem \n");
#endif
        return -1;
    }
    serdes_cfg1_base = iqn2_mmap((uint32_t)(serdes_cfg1_base), 0x2000);
    return 0;
#else
    return 0;
#endif
}

uint32_t serdesCfg1_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap((uint32_t*)serdes_cfg1_base, 0x2000);
    close(dev_mem_fd);
#endif
    return 0;
}

extern uint32_t boot_cfg_base;
uint32_t bootCfg_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
#ifndef USESYSBIOS
        printf("Failed to open /dev/mem \n");
#else
        System_printf("Failed to open /dev/mem \n");
#endif
        return -1;
    }
    boot_cfg_base = iqn2_mmap((uint32_t)(boot_cfg_base), 0x1000);
    return 0;
#else
    return 0;
#endif
}

uint32_t bootCfg_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap((uint32_t*)boot_cfg_base, 0x1000);
    close(dev_mem_fd);
#endif
    return 0;
}

void pscEnable(uint32_t pwrDmnNum, uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = iqn2_mmap(CSL_PSC_REGS, 0x10000);
    if (CSL_FEXT(((CSL_PscRegs *) mem_base_PSC)->PDSTAT[pwrDmnNum], PSC_PDSTAT_STATE) != PSC_PDSTATE_ON)
    {
        /* Enable the domain */
        CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->PDCTL[pwrDmnNum], PSC_PDCTL_NEXT, ON);
    }
    /* Enable MDCTL */
    CSL_FINS (((CSL_PscRegs *) mem_base_PSC)->MDCTL[lpscNum], PSC_MDCTL_NEXT, PSC_MODSTATE_ENABLE);
    /* Apply the domain */
    ((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << pwrDmnNum);
    /* Wait for it to finish */
    while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, pwrDmnNum, pwrDmnNum) == 1);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    if (CSL_PSC_getModuleState (lpscNum) != PSC_MODSTATE_ENABLE) {
        /* Turn on the power domain */
        CSL_PSC_enablePowerDomain (pwrDmnNum);
        /* Enable MDCTL */
        CSL_PSC_setModuleNextState (lpscNum, PSC_MODSTATE_ENABLE);
        /* Apply the domain */
        CSL_PSC_startStateTransition (pwrDmnNum);
        /* Wait for it to finish */
        while (! CSL_PSC_isStateTransitionDone (pwrDmnNum));

        /* Log  PSC status if not ok */
        if (!((CSL_PSC_getPowerDomainState(pwrDmnNum) == PSC_PDSTATE_ON) &&
                (CSL_PSC_getModuleState (lpscNum) == PSC_MODSTATE_ENABLE)))
        {
#ifndef USESYSBIOS
            printf("Error: failed to turn this power domain on\n");
#else
            System_printf("Error: failed to turn this power domain on\n");
#endif
        }
    }
#endif
}

void pscDisable(uint32_t pwrDmnNum, uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = iqn2_mmap(CSL_PSC_REGS, 0x10000);
    /* Disable MDCTL */
    CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->MDCTL[lpscNum], PSC_MDCTL_NEXT, DISABLE);
    /* Apply the domain */
    ((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << pwrDmnNum);
    /* Wait for it to finish */
    while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, pwrDmnNum, pwrDmnNum) == 1);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    /* Disable MDCTL */
    CSL_PSC_setModuleNextState (lpscNum, PSC_MODSTATE_DISABLE);
    /* Apply the domain */
    CSL_PSC_startStateTransition (pwrDmnNum);
    /* Wait for it to finish */
    while (! CSL_PSC_isStateTransitionDone (pwrDmnNum));

#endif
}


void pscPowerOff(uint32_t pwrDmnNum, uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = iqn2_mmap(CSL_PSC_REGS, 0x10000);
    /* Power Off */
    CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->PDCTL[pwrDmnNum], PSC_PDCTL_NEXT, OFF);
    /* Apply the domain */
    ((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << pwrDmnNum);
    /* Wait for it to finish */
    while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, pwrDmnNum, pwrDmnNum) == 1);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    //Wait for any previous transitions to complete
    while (!CSL_PSC_isStateTransitionDone (pwrDmnNum));
    //Write Switch input into the corresponding PDCTL register
    CSL_PSC_disablePowerDomain (pwrDmnNum);
    //Write PTCMD to start the transition
    CSL_PSC_startStateTransition (pwrDmnNum);
    //Wait for the transition to complete
    while (!CSL_PSC_isStateTransitionDone (pwrDmnNum));
#endif
}

void pscDisableResetIso(uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = iqn2_mmap(CSL_PSC_REGS, 0x10000);
    /* Disable MDCTL */
    CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->MDCTL[lpscNum], PSC_MDCTL_RSTISO, DISABLE);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    /* Disable reset isolation */
    CSL_PSC_disableModuleResetIsolation(lpscNum);
#endif
}

void UTILS_iqn2DfeEnable(uint32_t aid_enable)
{
    //AID, IQS, PSR, AT, DIO power on
    pscEnable(CSL_PSC_PD_ALWAYSON, CSL_PSC_LPSC_DFE_IQN_SYS);
    // Do not change this enable order
    pscEnable(CSL_PSC_PD_IQN_AIL, CSL_PSC_LPSC_IQN_AIL);
    if (aid_enable) {
        pscEnable(CSL_PSC_PD_DFE_PD1, CSL_PSC_LPSC_DFE_PD1);
        pscEnable(CSL_PSC_PD_DFE_PD0, CSL_PSC_LPSC_DFE_PD0);
    }
    pscEnable(CSL_PSC_PD_DFE_PD2, CSL_PSC_LPSC_DFE_PD2);

    // Disable IQN reset isolation mode (bootloader sets it on)
    UTILS_iqn2DfeDisableResetIso(aid_enable);
}

void UTILS_iqn2DfeDisable(uint32_t aid_enable)
{
    pscPowerOff(CSL_PSC_PD_DFE_PD2, CSL_PSC_LPSC_DFE_PD2);
    if (aid_enable) {
        pscPowerOff(CSL_PSC_PD_DFE_PD0, CSL_PSC_LPSC_DFE_PD0);
        pscPowerOff(CSL_PSC_PD_DFE_PD1, CSL_PSC_LPSC_DFE_PD1);
    }
    pscPowerOff(CSL_PSC_PD_IQN_AIL, CSL_PSC_LPSC_IQN_AIL);
    pscDisable(CSL_PSC_PD_ALWAYSON, CSL_PSC_LPSC_DFE_IQN_SYS);
}

void UTILS_iqn2DfeDisableResetIso(uint32_t aid_enable)
{
    pscDisableResetIso(CSL_PSC_LPSC_DFE_IQN_SYS);
    pscDisableResetIso(CSL_PSC_LPSC_IQN_AIL);
    if (aid_enable) {
        pscDisableResetIso(CSL_PSC_LPSC_DFE_PD1);
        pscDisableResetIso(CSL_PSC_LPSC_DFE_PD0);
    }
    pscDisableResetIso(CSL_PSC_LPSC_DFE_PD2);
}

uint8_t  keepIqn2DfeOff = 0;

void
UTILS_doCleanup(
    IQN2_ConfigHandle         hIqn2
)
{
#ifndef USESYSBIOS
    /* Clear the Interrupt */
#ifdef _TMS320C6X
        //disable global interrupt
    CSR&= 0xFFFFFFFE;
    //CSL_CPINTC_clearSysInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS,CSL_INTC0_AIF_INTD);
    IER= 0;
    ICR= 0xFFF0;

#endif
#else
	Hwi_disableInterrupt(COM_IQN2_AT_EVT0_EVENT);
	Hwi_disableInterrupt(COM_IQN2_AT_EVT1_EVENT);
	Hwi_disableInterrupt(COM_IQN2_AT_EVT2_EVENT);
	Hwi_disableInterrupt(COM_IQN2_PKTDMAEXCEP_EVENT);
	Hwi_disableInterrupt(COM_IQN2_EV0EXCEP_EVENT);
#endif
    // disable all reset isolation bits
    * (volatile int*)0x023100F0 = 0;

    //if (DSP_procId == 1) UTILS_resetTimer(timer);
    IQN2_resetAt2(hIqn2,&iqn2InitCfg);

    // Disable IQN2 reset isolation mode (bootloader sets it on)
    UTILS_iqn2DfeDisableResetIso(hIqn2->aidConfig.aidEnable);

    IQN2_resetIqn2(hIqn2,&iqn2InitCfg);

    UTILS_iqn2DfeDisable(hIqn2->aidConfig.aidEnable);

    /* Take AIF out of power saver again since AIF_resetAif is powering it off */
    if (keepIqn2DfeOff == 0) UTILS_iqn2DfeEnable(hIqn2->aidConfig.aidEnable);

}


uint32_t serdes_cfg0_base =  CSL_CSISC2_0_SERDES_CFG_REGS; //MMR base address of SerDes config0
uint32_t serdes_cfg1_base =  CSL_CSISC2_1_SERDES_CFG_REGS; //MMR base address of SerDes config1
uint32_t boot_cfg_base    =  CSL_BOOT_CFG_REGS;

#if CSL_VERSION_ID <= (0x02010006)
static void serdesCfg0RestoreDefault(
)
{
	uint32_t i;
	CSL_SerDes_COMLANE_Restore_Default(serdes_cfg0_base);
	for(i=0; i < 4; i++)
	{
	    CSL_SerDes_Lane_Restore_Default(serdes_cfg0_base, i);
	}
	CSL_SerDes_CMU_Restore_Default(serdes_cfg0_base);
}

static void serdesCfg1RestoreDefault(
)
{
    uint32_t i;
    CSL_SerDes_COMLANE_Restore_Default(serdes_cfg1_base);
    for(i=0; i < 4; i++)
    {
        CSL_SerDes_Lane_Restore_Default(serdes_cfg1_base, i);
    }
    CSL_SerDes_CMU_Restore_Default(serdes_cfg1_base);
}
#endif

static CSL_SERDES_LANE_CTRL_RATE iqn2SerdesGetLaneCtrlRate(
        IQN2_ConfigHandle         hIqn2,
        CSL_SERDES_LINK_RATE      serdesRate,
        Iqn2Fl_LinkRate           linkRate
)
{
    CSL_SERDES_LANE_CTRL_RATE laneCtrlRate = CSL_SERDES_LANE_FULL_RATE;
    if (  ((hIqn2->protocol == IQN2FL_PROTOCOL_OBSAI)&&(serdesRate == CSL_SERDES_LINK_RATE_6p144G))
        ||((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) &&(serdesRate == CSL_SERDES_LINK_RATE_6p144G)&&(linkRate == IQN2FL_LINK_RATE_5x))
        ||((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI) &&(serdesRate == CSL_SERDES_LINK_RATE_6p144G)&&(linkRate == IQN2FL_LINK_RATE_10x))
       )
    {
        if ((linkRate == IQN2FL_LINK_RATE_8x)||(linkRate == IQN2FL_LINK_RATE_10x)) laneCtrlRate = CSL_SERDES_LANE_FULL_RATE;
        if ((linkRate == IQN2FL_LINK_RATE_4x)||(linkRate == IQN2FL_LINK_RATE_5x))  laneCtrlRate = CSL_SERDES_LANE_HALF_RATE;
        if (linkRate == IQN2FL_LINK_RATE_2x) laneCtrlRate = CSL_SERDES_LANE_QUARTER_RATE;

    } else if (  ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)&&(serdesRate == CSL_SERDES_LINK_RATE_4p9152G)&&(linkRate == IQN2FL_LINK_RATE_8x))
               ||((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)&&(serdesRate == CSL_SERDES_LINK_RATE_4p9152G)&&(linkRate == IQN2FL_LINK_RATE_4x))
               ||((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)&&(serdesRate == CSL_SERDES_LINK_RATE_4p9152G)&&(linkRate == IQN2FL_LINK_RATE_2x))
              )
    {
        if (linkRate == IQN2FL_LINK_RATE_8x) laneCtrlRate = CSL_SERDES_LANE_FULL_RATE;
        if (linkRate == IQN2FL_LINK_RATE_4x) laneCtrlRate = CSL_SERDES_LANE_HALF_RATE;
        if (linkRate == IQN2FL_LINK_RATE_2x) laneCtrlRate = CSL_SERDES_LANE_QUARTER_RATE;

    } else if (  ((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)&&(serdesRate == CSL_SERDES_LINK_RATE_9p8304G)&&(linkRate == IQN2FL_LINK_RATE_16x))
               ||((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)&&(serdesRate == CSL_SERDES_LINK_RATE_9p8304G)&&(linkRate == IQN2FL_LINK_RATE_8x))
               ||((hIqn2->protocol == IQN2FL_PROTOCOL_CPRI)&&(serdesRate == CSL_SERDES_LINK_RATE_9p8304G)&&(linkRate == IQN2FL_LINK_RATE_4x))
              )
    {
        if (linkRate == IQN2FL_LINK_RATE_16x) laneCtrlRate = CSL_SERDES_LANE_FULL_RATE;
        if (linkRate == IQN2FL_LINK_RATE_8x)  laneCtrlRate  = CSL_SERDES_LANE_HALF_RATE;
        if (linkRate == IQN2FL_LINK_RATE_4x)  laneCtrlRate  = CSL_SERDES_LANE_QUARTER_RATE;
    } else {
#ifndef USESYSBIOS
        printf("Error: incorrect combination of protocol, serdes rate, and link rate\n");
#else
        System_printf("Error: incorrect combination of protocol, serdes rate, and link rate\n");
#endif
    }
    return(laneCtrlRate);
}

#if CSL_VERSION_ID <= (0x02010006)
void UTILS_iqn2SerdesConfig(
		IQN2_ConfigHandle         hIqn2,
		CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
)
{
	uint32_t                    retval, i;
	CSL_SERDES_LOOPBACK         loopback;
    CSL_SERDES_LANE_CTRL_RATE   serdesLaneCtrlRate;
    uint32_t                    serdes_mux_iqn_sel=0, numlanes=0;

    /* Check CSISC2_0_MUXSEL bit */
    if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 26, 26) == 1) serdes_mux_iqn_sel = 1;

    if (serdes_mux_iqn_sel == 0) {
#ifndef USESYSBIOS
    	printf("Error: can't configure iqn2 serdes given CSISC2_0_MUXSEL\n");
#else
    	System_printf("Error: can't configure iqn2 serdes given CSISC2_0_MUXSEL\n");
#endif
    } else {

		// Call the shutdown for each SerDes and restore default values if SerDes were already in use
		CSL_IQNSerdesShutdown(serdes_cfg0_base);
		if (!CSL_IQN2SerdesIsReset(serdes_cfg0_base)) {
			serdesCfg0RestoreDefault();
		}

		CSL_IQNSerdesInit(serdes_cfg0_base, refClock, serdesRate);

		for(i=0; i < 2; i++)
		{
			if(1==hIqn2->ailConfig[i].ailEnable) {
				if (hIqn2->ailConfig[i].comType == IQN2_COM_SD_LOOPBACK) {       //setup links for internal loopback mode
					loopback = CSL_SERDES_LOOPBACK_ENABLED;
				} else {
					loopback = CSL_SERDES_LOOPBACK_DISABLED;
				}
				serdesLaneCtrlRate = iqn2SerdesGetLaneCtrlRate(hIqn2, serdesRate,hIqn2->ailConfig[i].linkRate);
				CSL_IQNSerdesLaneEnable(serdes_cfg0_base, i, loopback, serdesLaneCtrlRate);
				numlanes++;
			}
		}

		//IQN2 SerDes PLL Enable
		CSL_IQNSerdesPllEnable(serdes_cfg0_base);

		//IQN2 SerDes PLL Status Poll
		retval = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
		while(retval == CSL_SERDES_STATUS_PLL_NOT_LOCKED)
		{
			retval = CSL_IQNSerdesGetStatus(serdes_cfg0_base,numlanes);
		}

    }
}

void UTILS_dfeSerdesConfig(
        UTILS_dfeSerDesCfg        *hDfeSerDesCfg,
        CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
)
{
    uint32_t                    retval, i, tcnt;
    CSL_SERDES_LOOPBACK         loopback;
    uint32_t                    serdes_mux_dfe_sel=0, serdes_common_ref_clock=0, numlanescfg0=0, numlanescfg1=0;

    /* Check CSISC2_0_MUXSEL bit */
    if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 26, 26) == 0) serdes_mux_dfe_sel = 1;
    //serdes_mux_dfe_sel = 1;

    /* Check CSISC2_0_CLKCTL bit */
    //if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 27, 27) == 1) serdes_common_ref_clock = 1;
    serdes_common_ref_clock = 1;

    if (serdes_mux_dfe_sel == 0) {
#ifndef USESYSBIOS
        printf("Error: can't configure dfe serdes given CSISC2_0_MUXSEL\n");
#else
        System_printf("Error: can't configure dfe serdes given CSISC2_0_MUXSEL\n");
#endif
    } else if (serdes_common_ref_clock == 0) {
#ifndef USESYSBIOS
        printf("Error: can't configure dfe serdes given both blocks don't use the same ref clock\n");
#else
        System_printf("Error: can't configure dfe serdes given both blocks don't use the same ref clock\n");
#endif
    } else {
        // Call the shutdown for each SerDes and restore default values if SerDes were already in use
        CSL_DFESerdesShutdown(serdes_cfg0_base);
        if (!CSL_DFESerdesIsReset(serdes_cfg0_base)) {
            serdesCfg0RestoreDefault();
        }
        CSL_DFESerdesShutdown(serdes_cfg1_base);
        if (!CSL_DFESerdesIsReset(serdes_cfg1_base)) {
            serdesCfg1RestoreDefault();
        }

        CSL_DFESerdesInit(serdes_cfg0_base, refClock, serdesRate);
        CSL_DFESerdesInit(serdes_cfg1_base, refClock, serdesRate);

        for(i=0; i < 2; i++)
        {
            if(1==hDfeSerDesCfg->laneEnable[i]) {
                if (1==hDfeSerDesCfg->lbackEnable[i]) {       //setup links for internal loopback mode
                    loopback = CSL_SERDES_LOOPBACK_ENABLED;
                } else {
                    loopback = CSL_SERDES_LOOPBACK_DISABLED;
                }
                CSL_DFESerdesLaneEnable(serdes_cfg0_base, i, loopback, hDfeSerDesCfg->laneCtrlRate[i]);
                numlanescfg0++;
            }
        }
        for(i=2; i < 4; i++)
        {
            if(1==hDfeSerDesCfg->laneEnable[i]) {
                if (1==hDfeSerDesCfg->lbackEnable[i]) {       //setup links for internal loopback mode
                    loopback = CSL_SERDES_LOOPBACK_ENABLED;
                } else {
                    loopback = CSL_SERDES_LOOPBACK_DISABLED;
                }
                CSL_DFESerdesLaneEnable(serdes_cfg1_base, i-2, loopback, hDfeSerDesCfg->laneCtrlRate[i]);
                numlanescfg1++;
            }
        }

        //DFE SerDes PLL Enable
        CSL_DFESerdesPllEnable(serdes_cfg0_base);
        CSL_DFESerdesPllEnable(serdes_cfg1_base);

        //DFE SerDes PLL Status Poll
        retval = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
        tcnt = 0;
        while(retval == CSL_SERDES_STATUS_PLL_NOT_LOCKED)
        {
            retval = CSL_DFESerdesGetStatus(serdes_cfg0_base,numlanescfg0);
            tcnt++;
            if (tcnt > 50000)
            	break;
        }
        if (tcnt > 50000)
#ifndef USESYSBIOS
        	printf("serdes0 PLL is unlocked!\n");
        else
        	printf("serdes0 PLL is locked!\n");
#else
			System_printf("serdes0 PLL is unlocked!\n");
		else
			System_printf("serdes0 PLL is locked!\n");
#endif
        //DFE SerDes PLL Status Poll
        retval = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
        tcnt = 0;
        while(retval == CSL_SERDES_STATUS_PLL_NOT_LOCKED)
        {
            retval = CSL_DFESerdesGetStatus(serdes_cfg1_base,numlanescfg1);
            tcnt++;
            if (tcnt > 50000)
            	break;
        }
        if (tcnt > 50000)
#ifndef USESYSBIOS
        	printf("serdes1 PLL is unlocked!\n");
        else
        	printf("serdes1 PLL is locked!\n");
#else
			System_printf("serdes1 PLL is unlocked!\n");
		else
			System_printf("serdes1 PLL is locked!\n");
#endif
    }
}

#else
void UTILS_iqn2SerdesConfig(
		IQN2_ConfigHandle         hIqn2,
		CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
)
{
	uint32_t                    i;
    uint32_t                    serdes_mux_iqn_sel=0;
    CSL_SERDES_RESULT status;
    CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
    CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params1;

#ifdef _TMS320C6X
    // start CPU timestamp as it is needed by the SerDes CSL to compute delays
    TSCL=0;
#endif

    memset(&serdes_lane_enable_params1, 0, sizeof(serdes_lane_enable_params1));

    /* Check CSISC2_0_MUXSEL bit */
    if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 26, 26) == 1) serdes_mux_iqn_sel = 1;

    if (serdes_mux_iqn_sel == 0) {
#ifndef USESYSBIOS
    	printf("Error: can't configure iqn2 serdes given CSISC2_0_MUXSEL\n");
#else
    	System_printf("Error: can't configure iqn2 serdes given CSISC2_0_MUXSEL\n");
#endif
    } else {

         serdes_lane_enable_params1.base_addr = serdes_cfg0_base;
         serdes_lane_enable_params1.ref_clock = refClock;
         serdes_lane_enable_params1.linkrate = serdesRate;
         serdes_lane_enable_params1.num_lanes = 2;
         serdes_lane_enable_params1.phy_type = SERDES_IQN;
         serdes_lane_enable_params1.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
         serdes_lane_enable_params1.lane_mask = 0x3;
         serdes_lane_enable_params1.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

         CSL_SERDES_SHUTDOWN(serdes_lane_enable_params1.base_addr, serdes_lane_enable_params1.num_lanes);

         for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
  		{
  			if(1==hIqn2->ailConfig[i].ailEnable)
  			{
  			    if (hIqn2->ailConfig[i].comType == IQN2_COM_SD_LOOPBACK)
  			    {   //setup links for internal loopback mode
  				    serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_ENABLED;
  				}
  			    else
  			    {
  					serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
  				}
  			}
  			serdes_lane_enable_params1.lane_ctrl_rate[i] = iqn2SerdesGetLaneCtrlRate(hIqn2, serdesRate,hIqn2->ailConfig[i].linkRate);

  			/* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
  			serdes_lane_enable_params1.rx_coeff.att_start[i] = 7;
  			serdes_lane_enable_params1.rx_coeff.boost_start[i] = 5;

  			/* For higher speeds PHY-A, force attenuation and boost values  */
  			serdes_lane_enable_params1.rx_coeff.force_att_val[i] = 1;
  			serdes_lane_enable_params1.rx_coeff.force_boost_val[i] = 1;

  			/* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
  			serdes_lane_enable_params1.tx_coeff.cm_coeff[i] = 0;
  			serdes_lane_enable_params1.tx_coeff.c1_coeff[i] = 0;
  			serdes_lane_enable_params1.tx_coeff.c2_coeff[i] = 0;
  			serdes_lane_enable_params1.tx_coeff.tx_att[i] = 12;
  			serdes_lane_enable_params1.tx_coeff.tx_vreg[i] = 4;
  		}

        status = CSL_IQNSerdesInit(serdes_lane_enable_params1.base_addr,
                                         serdes_lane_enable_params1.ref_clock,
                                         serdes_lane_enable_params1.linkrate);
        if (status != 0)
        {
#ifndef USESYSBIOS
            printf ("Invalid Serdes Init Params\n");
#else
            System_printf ("Invalid Serdes Init Params\n");
#endif
        }

    	/* Common Init Mode */
    	/* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
    	/* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
    	serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
    	serdes_lane_enable_params1.lane_mask = 0x3;
    	lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);

    	/* Lane Init Mode */
    	/* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
    	   iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
    	/* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
    	serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
    	for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
    	{
    		serdes_lane_enable_params1.lane_mask = 1<<i;
    		lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
            if (lane_retval != 0)
            {
#ifndef USESYSBIOS
            printf ("Invalid Serdes Lane Enable Init\n");
#else
            System_printf ("Invalid Serdes Lane Enable Init\n");
#endif
            }
    	}

    }
}

void UTILS_dfeSerdesConfig(
        UTILS_dfeSerDesCfg        *hDfeSerDesCfg,
        CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
)
{
    uint32_t                    i;
//    CSL_SERDES_LOOPBACK         loopback;
    uint32_t                    serdes_mux_dfe_sel=0, serdes_common_ref_clock=0;// numlanescfg0=0, numlanescfg1=0;
    CSL_SERDES_RESULT status;
    CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
    CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params1, serdes_lane_enable_params2;

#ifdef _TMS320C6X
    // start CPU timestamp as it is needed by the SerDes CSL to compute delays
    TSCL=0;
#endif

    memset(&serdes_lane_enable_params1, 0, sizeof(serdes_lane_enable_params1));
    memset(&serdes_lane_enable_params2, 0, sizeof(serdes_lane_enable_params2));

    /* Check CSISC2_0_MUXSEL bit */
    if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 26, 26) == 0) serdes_mux_dfe_sel = 1;
    //serdes_mux_dfe_sel = 1;

    /* Check CSISC2_0_CLKCTL bit */
    //if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 27, 27) == 1) serdes_common_ref_clock = 1;
    serdes_common_ref_clock = 1;

    if (serdes_mux_dfe_sel == 0) {
#ifndef USESYSBIOS
        printf("Error: can't configure dfe serdes given CSISC2_0_MUXSEL\n");
#else
        System_printf("Error: can't configure dfe serdes given CSISC2_0_MUXSEL\n");
#endif
    } else if (serdes_common_ref_clock == 0) {
#ifndef USESYSBIOS
        printf("Error: can't configure dfe serdes given both blocks don't use the same ref clock\n");
#else
        System_printf("Error: can't configure dfe serdes given both blocks don't use the same ref clock\n");
#endif
    } else {

        serdes_lane_enable_params1.base_addr = serdes_cfg0_base;
        serdes_lane_enable_params1.ref_clock = refClock;
        serdes_lane_enable_params1.linkrate = serdesRate;
        serdes_lane_enable_params1.num_lanes = 2;
        serdes_lane_enable_params1.phy_type = SERDES_DFE;
        serdes_lane_enable_params1.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
        serdes_lane_enable_params1.lane_mask = 0x3;
        serdes_lane_enable_params1.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

        CSL_SERDES_SHUTDOWN(serdes_lane_enable_params1.base_addr, serdes_lane_enable_params1.num_lanes);

        serdes_lane_enable_params2.base_addr = serdes_cfg1_base;
        serdes_lane_enable_params2.ref_clock = refClock;
        serdes_lane_enable_params2.linkrate = serdesRate;
        serdes_lane_enable_params2.num_lanes = 2;
        serdes_lane_enable_params2.phy_type = SERDES_DFE;
        serdes_lane_enable_params2.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
        serdes_lane_enable_params2.lane_mask = 0x3;
        serdes_lane_enable_params2.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

        CSL_SERDES_SHUTDOWN(serdes_lane_enable_params2.base_addr, serdes_lane_enable_params2.num_lanes);

        for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
		{
			if(1==hDfeSerDesCfg->laneEnable[i])
			{
			    if (1==hDfeSerDesCfg->lbackEnable[i])
			    {   //setup links for internal loopback mode
				    serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_ENABLED;
				}
			    else
			    {
					serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
				}
			}
			serdes_lane_enable_params1.lane_ctrl_rate[i] = hDfeSerDesCfg->laneCtrlRate[i];

			/* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
			serdes_lane_enable_params1.rx_coeff.att_start[i] = 7;
			serdes_lane_enable_params1.rx_coeff.boost_start[i] = 5;

			/* For higher speeds PHY-A, force attenuation and boost values  */
			serdes_lane_enable_params1.rx_coeff.force_att_val[i] = 1;
			serdes_lane_enable_params1.rx_coeff.force_boost_val[i] = 1;

			/* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
			serdes_lane_enable_params1.tx_coeff.cm_coeff[i] = 0;
			serdes_lane_enable_params1.tx_coeff.c1_coeff[i] = 0;
			serdes_lane_enable_params1.tx_coeff.c2_coeff[i] = 0;
			serdes_lane_enable_params1.tx_coeff.tx_att[i] = 12;
			serdes_lane_enable_params1.tx_coeff.tx_vreg[i] = 4;
		}

        for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
		{
			if(1==hDfeSerDesCfg->laneEnable[i+2])
			{
			    if (1==hDfeSerDesCfg->lbackEnable[i+2])
			    {   //setup links for internal loopback mode
					serdes_lane_enable_params2.loopback_mode[i] = CSL_SERDES_LOOPBACK_ENABLED;
				}
			    else
			    {
					serdes_lane_enable_params2.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
				}
			}
			serdes_lane_enable_params2.lane_ctrl_rate[i] = hDfeSerDesCfg->laneCtrlRate[i+2];
			/* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
			serdes_lane_enable_params2.rx_coeff.att_start[i] = 7;
			serdes_lane_enable_params2.rx_coeff.boost_start[i] = 5;

			/* For higher speeds PHY-A, force attenuation and boost values  */
			serdes_lane_enable_params2.rx_coeff.force_att_val[i] = 1;
			serdes_lane_enable_params2.rx_coeff.force_boost_val[i] = 1;

			/* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
			serdes_lane_enable_params2.tx_coeff.cm_coeff[i] = 0;
			serdes_lane_enable_params2.tx_coeff.c1_coeff[i] = 0;
			serdes_lane_enable_params2.tx_coeff.c2_coeff[i] = 0;
			serdes_lane_enable_params2.tx_coeff.tx_att[i] = 12;
			serdes_lane_enable_params2.tx_coeff.tx_vreg[i] = 4;
		}

        status = CSL_DFESerdesInit(serdes_lane_enable_params1.base_addr,
                                        serdes_lane_enable_params1.ref_clock,
                                        serdes_lane_enable_params1.linkrate);
        status = CSL_DFESerdesInit(serdes_lane_enable_params2.base_addr,
                                        serdes_lane_enable_params2.ref_clock,
                                        serdes_lane_enable_params2.linkrate);

        if (status != 0)
        {
#ifndef USESYSBIOS
            printf ("Invalid Serdes Init Params\n");
#else
            System_printf ("Invalid Serdes Init Params\n");
#endif
        }

    	/* Common Init Mode */
    	/* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
    	/* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
    	serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
    	serdes_lane_enable_params1.lane_mask = 0x3;
    	lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);

    	serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
    	serdes_lane_enable_params2.lane_mask = 0x3;
    	lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);

    	/* Lane Init Mode */
    	/* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
    	   iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
    	/* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
    	serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
    	for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
    	{
    		serdes_lane_enable_params1.lane_mask = 1<<i;
    		lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
            if (lane_retval != 0)
            {
#ifndef USESYSBIOS
                printf ("Invalid Serdes Lane Enable Init\n");
#else
                System_printf ("Invalid Serdes Lane Enable Init\n");
#endif
            }
    	}

    	serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
    	for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
    	{
    		serdes_lane_enable_params2.lane_mask = 1<<i;
    		lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
            if (lane_retval != 0)
            {
#ifndef USESYSBIOS
                printf ("Invalid Serdes Lane Enable Init\n");
#else
                System_printf ("Invalid Serdes Lane Enable Init\n");
#endif
            }
    	}

#ifndef USESYSBIOS
        printf("JESD Serdes Init Complete\n");
#else
        System_printf ("JESD Serdes Init Complete\n");
#endif

    }
}
#endif
////////////////////
