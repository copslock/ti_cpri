/**
 *   @file  am65xx/src/pcie_soc.c
 *
 *   @brief
 *      This file contains the device specific configuration and initialization routines
 *      for pcie Low Level Driver.
 *
 *  ============================================================================
 *  @n   (C) Copyright 2015-2018, Texas Instruments, Inc.
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

/* pcie LLD includes */
#include <ti/drv/pcie/soc/pcie_soc.h>
#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/v2/pcie.h>

/** @addtogroup PCIE_LLD_DATASTRUCT
@{
*/

/** @brief PCIE v2 calltable */
Pcie_FxnTable pcieFxnTablev2 =
{
  /*! Function to set PCIE to EP or RC for one device */
  &Pciev2_setInterfaceMode,
  /*! Function to get the PCIE data area reserved size */
  &Pciev2_getMemSpaceReserved,
  /*! Function to get the PCIE data area base address & size */
  &Pciev2_getMemSpaceRange,
  /*! Function to read any PCIE register(s) */
  &Pciev2_readRegs,
  /*! Function to write any PCIE register(s) */
  &Pciev2_writeRegs,
  /*! Function to configure outbound translation registers */
  NULL, /* not supported */
  /*! Function to configure inbound translation registers */
  NULL, /* not supported */
  /*! Function to configure a BAR register */
  &Pciev2_cfgBar,
  /*! Function to configure an ATU region */
  &Pciev2_atuRegionConfig,
  /*! Function to read functional (MSI/INTX) pending bits with low overhead. */
  &Pciev2_getPendingFuncInts,
  /*! Function to clear functional (MSI/INTX) pending bits with low overhead. */
  &Pciev2_clrPendingFuncInts
};

Pciev2_DevParams pcieDevParamsDev1 =
{
  (volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL)
};

Pciev2_DevParams pcieDevParamsDev2 =
{
  (volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_PCIE1_CTRL)
};

Pciev2_DeviceCfgBaseAddrs pciev2CfgBaseAddrDev1 =
{
  (void *)CSL_PCIE0_DAT_BASE,
/* The mapped offset is 0x10010000, but that points to registers
   that normally occur at base + 0x1000, hence the - 0x1000.
   No access to 0x0 to 0x0FFF is normal for other devices but
   they have separate structure that can be pointed here */
  (uint32_t)0x10000u - 0x1000u
};

Pciev2_DeviceCfgBaseAddrs pciev2CfgBaseAddrDev2 =
{
  (void *)CSL_PCIE1_DAT_BASE,
  (uint32_t)0x10000u - 0x1000u
};

Pcie_DeviceCfgBaseAddr pcieBaseAddrDev1 =
{
  (void *)&pciev2CfgBaseAddrDev1,
  (void *)CSL_PCIE0_DAT0_BASE,
  0U,
  (void *)&pcieDevParamsDev1
};

Pcie_DeviceCfgBaseAddr pcieBaseAddrDev2 =
{
  (void *)&pciev2CfgBaseAddrDev2,
  (void *)CSL_PCIE1_DAT0_BASE,
  0U,
  (void *)&pcieDevParamsDev2
};

/** @brief PCIE LLD initialization parameters */
const Pcie_InitCfg pcieInitCfg =
{
  {
    {
      &pcieBaseAddrDev1,
      &pcieBaseAddrDev2,
      NULL,
      NULL
    },
    {
      &pcieFxnTablev2,
      &pcieFxnTablev2,
      NULL,
      NULL
    }
  }
};

/**
@}
*/

