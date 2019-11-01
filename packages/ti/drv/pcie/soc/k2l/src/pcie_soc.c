/**
 *   @file  k2l/src/pcie_soc.c
 *
 *   @brief   
 *      This file contains the device specific configuration and initialization routines
 *      for pcie Low Level Driver.
 *
 *  ============================================================================
 *  @n   (C) Copyright 2013-2016, Texas Instruments, Inc.
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
 *  \par
*/

/** 
 * This file contains an example device configuration for the pcie LLD.
 * It is not precompiled to facilitate user modification of the file.
 */

#include <stdint.h>
#include <stdlib.h>

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_bootcfg.h>

#include <ti/csl/csl_bootcfg.h>

/* pcie LLD includes */
#include <ti/drv/pcie/soc/pcie_soc.h>
#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/v0/pcie.h>

/** @addtogroup PCIE_LLD_DATASTRUCT
@{ 
*/

/** @brief PCIE v0 calltable */
Pcie_FxnTable fxnTablev0 =
{
  /*! Function to set PCIE to EP or RC for one device */
  &Pciev0_setInterfaceMode,
  /*! Function to get the PCIE data area reserved size */
  &Pciev0_getMemSpaceReserved,
  /*! Function to get the PCIE data area base address & size */
  &Pciev0_getMemSpaceRange,
  /*! Function to read any PCIE register(s) */
  &Pciev0_readRegs,
  /*! Function to write any PCIE register(s) */
  &Pciev0_writeRegs,
  /*! Function to configure outbound translation registers */
  &Pciev0_cfgObOffset,
  /*! Function to configure inbound translation registers */
  &Pciev0_cfgIbTrans,
  /*! Function to configure a BAR register */
  &Pciev0_cfgBar,
  /*! Function to configure an ATU region */
  NULL, /* unsupported on rev 0 */
  /*! Function to read functional (MSI/INTX) pending bits with low overhead. */
  NULL, /* unsupported on rev 0 */
  /*! Function to clear functional (MSI/INTX) pending bits with low overhead. */
  NULL  /* unsupported on rev 0 */
};

Pciev0_DevParams modeSelDev0 = 
{
  (volatile uint32_t *)&hBootCfg->DEVCFG,
  CSL_BOOTCFG_DEVCFG_PCIESS_0_MODE_MASK,
  CSL_BOOTCFG_DEVCFG_PCIESS_0_MODE_SHIFT
};

Pcie_DeviceCfgBaseAddr baseAddrDev0 = 
{
  (void *)CSL_PCIE_0_SLV_CFG_REGS,
  (void *)CSL_PCIE_0_SLV_DATA,
  0U,
  (void *)&modeSelDev0
};

Pciev0_DevParams modeSelDev1 = 
{
  (volatile uint32_t *)&hBootCfg->DEVCFG,
  CSL_BOOTCFG_DEVCFG_PCIESS_1_MODE_MASK,
  CSL_BOOTCFG_DEVCFG_PCIESS_1_MODE_SHIFT
};

Pcie_DeviceCfgBaseAddr baseAddrDev1 = 
{
  (void *)CSL_PCIE_1_SLV_CFG_REGS,
  (void *)CSL_PCIE_1_SLV_DATA,
  0U,
  (void *)&modeSelDev1
};

/** @brief PCIE LLD initialization parameters */
const Pcie_InitCfg pcieInitCfg =
{
  {
    {
      &baseAddrDev0,
      &baseAddrDev1,
      NULL,
      NULL
    },
    {
      &fxnTablev0,
      &fxnTablev0,
      NULL,
      NULL
    }
  }
};

/**
@}
*/

