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


#ifndef __CSLUTILS_H
#define __CSLUTILS_H

#include <ti/csl/csl.h>
#include <ti/csl/csl_pllc.h>
#include <ti/csl/csl_cpIntc.h>
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_serdes.h>
#include <ti/csl/cslr_device.h>

#ifdef _TMS320C6X
#ifdef USESYSBIOS
#include <ti/sysbios/family/c66/Cache.h>
#else
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>
#endif
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_tmr.h>
#include <ti/csl/csl_tmrAux.h>
#endif

#include <ti/drv/iqn2/iqn2fl.h>

#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/iqn2/device/iqn2_device.h>

typedef struct {
        // Test name
        const unsigned char name[32];
        Iqn2Fl_Protocol 		protocol;
        Iqn2Fl_LinkRate			linkRate[IQN2_MAX_NUM_AIL];
        uint32_t 				numWcdmaAxC[IQN2_MAX_NUM_AIL];
        uint32_t 				numLte20AxC[IQN2_MAX_NUM_AIL];
        uint32_t                numLte10AxC[IQN2_MAX_NUM_AIL];
        uint32_t                numLte5AxC[IQN2_MAX_NUM_AIL];
        uint32_t                numLte15AxC[IQN2_MAX_NUM_AIL];
        uint32_t				peOffset[IQN2_MAX_NUM_AIL];
        IQN2_ComMode            comType[IQN2_MAX_NUM_AIL];
        } TestObj;

typedef struct {
        // Test name
        const unsigned char name[32];
        Iqn2Fl_Protocol         protocol;
        Iqn2Fl_LinkRate         linkRate;
        uint32_t                numWcdmaAxC;
        uint32_t                numLte20AxC;
        uint32_t                numLte10AxC;
        uint32_t                numLte5AxC;
        uint32_t                numLte15AxC;
        uint32_t                numLte40AxC;
        uint32_t                numLte80AxC;
        } TestAidObj;

typedef struct {
        uint32_t                    laneEnable[4];
        uint32_t                    lbackEnable[4];
        CSL_SERDES_LANE_CTRL_RATE   laneCtrlRate[4];
        } UTILS_dfeSerDesCfg;


typedef void (*UTILS_FxnPtr)();

extern uint8_t      	       DSP_procId;
extern volatile uint32_t       ext_sync;
#ifdef _TMS320C6X
extern CSL_CPINTCRegs     *CicRegs[3];
extern CSL_IntcRegs       *IntcRegs;
#endif
extern UTILS_FxnPtr        atevt0_userIsr;
extern UTILS_FxnPtr        atevt1_userIsr;
extern UTILS_FxnPtr        atevt2_userIsr;
extern UTILS_FxnPtr        iqn2ev0except_userIsr;
extern UTILS_FxnPtr        dfeev1except_userIsr;
extern UTILS_FxnPtr        iqn2pktdmaexcept_userIsr;

void UTILS_fillIqn2Obj (TestObj *testObjTab, IQN2_ConfigHandle hIqn2);

void UTILS_fillIqn2Aid2Obj (TestAidObj *testObjTab, IQN2_ConfigHandle hIqn2);

void UTILS_waitForHw(Uint32 delay);

#ifdef _TMS320C6X

void UTILS_iqn2IntcSetup();

void UTILS_iqn2ExceptIntDisable();

void UTILS_setCache();

void UTILS_cacheInvalidate(void* ptr, uint32_t size);

void UTILS_cacheWriteBack(void* ptr, uint32_t size);

void UTILS_cacheWriteBackInvalidate(void* ptr, uint32_t size);

void UTILS_resetTimer(uint32_t timer);

void UTILS_startTimer();

void UTILS_initTimer(uint32_t timer);

void UTILS_triggerExternalSync(
        IQN2_ConfigHandle         hIqn2
);

void UTILS_getIqn2EV0Exception();

void UTILS_getDfeEV1Exception();


void UTILS_getIqn2PktDMAException();

#endif

uint32_t iqn2_mapDevice();

uint32_t iqn2_mmap(uint32_t addr, uint32_t size);

uint32_t iqn2_unmapDevice();

uint32_t dfe_mapDevice();

uint32_t dfe_unmapDevice();

uint32_t serdesCfg0_mapDevice();

uint32_t serdesCfg0_unmapDevice();

uint32_t serdesCfg1_mapDevice();

uint32_t serdesCfg1_unmapDevice();

uint32_t bootCfg_mapDevice();

uint32_t bootCfg_unmapDevice();

void UTILS_iqn2DfeEnable(uint32_t aid_enable);

void UTILS_iqn2DfeDisable(uint32_t aid_enable);

void UTILS_iqn2DfeDisableResetIso(uint32_t aid_enable);

void UTILS_doCleanup(
        IQN2_ConfigHandle         hIqn2
);

void UTILS_iqn2SerdesConfig(
		IQN2_ConfigHandle         hIqn2,
		CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
);

void UTILS_dfeSerdesConfig(
        UTILS_dfeSerDesCfg        *hDfeSerDesCfg,
        CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
);

#endif//__CSLUTILS_H

/** @} */ // end of module additions

