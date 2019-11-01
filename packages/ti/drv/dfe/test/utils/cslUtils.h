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
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#endif

typedef struct {
        uint32_t                    laneEnable[4];
        uint32_t                    lbackEnable[4];
        CSL_SERDES_LANE_CTRL_RATE   laneCtrlRate[4];
        } UTILS_dfeSerDesCfg;
#ifdef _TMS320C6X
void UTILS_setCache();

void UTILS_cacheInvalidate(void* ptr, uint32_t size);

void UTILS_cacheWriteBack(void* ptr, uint32_t size);

void UTILS_cacheWriteBackInvalidate(void* ptr, uint32_t size);


#endif

uint32_t dfe_mapDevice();

uint32_t dfe_unmapDevice();

uint32_t serdesCfg0_mapDevice();

uint32_t serdesCfg0_unmapDevice();

uint32_t serdesCfg1_mapDevice();

uint32_t serdesCfg1_unmapDevice();

uint32_t bootCfg_mapDevice();

uint32_t bootCfg_unmapDevice();

void UTILS_iqn2DfeEnable(uint32_t dfe_enable, uint32_t dpd_enable);

void UTILS_iqn2DfeDisable();

void UTILS_dfeSerdesConfig(
        UTILS_dfeSerDesCfg        *hDfeSerDesCfg,
        CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
);

#endif//__CSLUTILS_H

/** @} */ // end of module additions

