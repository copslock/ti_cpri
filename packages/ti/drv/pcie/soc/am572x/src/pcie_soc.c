/**
 *   @file  am572x/src/pcie_soc.c
 *
 *   @brief
 *      This file contains the device specific configuration and initialization routines
 *      for pcie Low Level Driver.
 *
 *  ============================================================================
 *  @n   (C) Copyright 2015-2016, Texas Instruments, Inc.
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
#include <ti/drv/pcie/src/v1/pcie.h>

/** @addtogroup PCIE_LLD_DATASTRUCT
@{
*/

/** @brief PCIE v1 calltable */
Pcie_FxnTable fxnTablev1 =
{
  /*! Function to set PCIE to EP or RC for one device */
  &Pciev1_setInterfaceMode,
  /*! Function to get the PCIE data area reserved size */
  &Pciev1_getMemSpaceReserved,
  /*! Function to get the PCIE data area base address & size */
  &Pciev1_getMemSpaceRange,
  /*! Function to read any PCIE register(s) */
  &Pciev1_readRegs,
  /*! Function to write any PCIE register(s) */
  &Pciev1_writeRegs,
  /*! Function to configure outbound translation registers */
  NULL, /* not supported */
  /*! Function to configure inbound translation registers */
  NULL, /* not supported */
  /*! Function to configure a BAR register */
  &Pciev1_cfgBar,
  /*! Function to configure an ATU region */
  &Pciev1_atuRegionConfig,
  /*! Function to read functional (MSI/INTX) pending bits with low overhead. */
  &Pciev1_getPendingFuncInts,
  /*! Function to clear functional (MSI/INTX) pending bits with low overhead. */
  &Pciev1_clrPendingFuncInts
};

#ifdef _TMS320C6X
Pciev1_DeviceCfgBaseAddrs cfgBaseAddrsDev1 =
{
  (void *)CSL_DSP_PCIE_SS1_CONF_REGS_I_RC_CFG_DBICS_REGS,
  (void *)CSL_DSP_PCIE_SS1_CONF_REGS_I_RC_CFG_DBICS2_REGS,
  (void *)CSL_DSP_PCIE_SS1_CONF_REGS_I_TI_CONF_REGS,
  (void *)CSL_DSP_PCIE_SS1_CONF_REGS_I_PL_CONF_REGS,
  (uint32_t)0x1000
};

Pcie_DeviceCfgBaseAddr baseAddrDev1 =
{
  &cfgBaseAddrsDev1,
  (void *)CSL_DSP_PCIE_SS1_REGS,
  0U,
  NULL
};

Pciev1_DeviceCfgBaseAddrs cfgBaseAddrsDev2 =
{
  (void *)CSL_DSP_PCIE_SS2_CONF_REGS_I_RC_CFG_DBICS_REGS,
  (void *)CSL_DSP_PCIE_SS2_CONF_REGS_I_RC_CFG_DBICS2_REGS,
  (void *)CSL_DSP_PCIE_SS2_CONF_REGS_I_TI_CONF_REGS,
  (void *)CSL_DSP_PCIE_SS2_CONF_REGS_I_PL_CONF_REGS,
  (uint32_t)0x1000
};

Pcie_DeviceCfgBaseAddr baseAddrDev2 =
{
  &cfgBaseAddrsDev2,
  (void *)CSL_DSP_PCIE_SS2_REGS,
  0U,
  NULL
};

#elif defined(__ARM_ARCH_7A__)

Pciev1_DeviceCfgBaseAddrs cfgBaseAddrsDev1 =
{
  (void *)CSL_MPU_PCIE_SS1_CONF_REGS_I_RC_CFG_DBICS_REGS,
  (void *)CSL_MPU_PCIE_SS1_CONF_REGS_I_RC_CFG_DBICS2_REGS,
  (void *)CSL_MPU_PCIE_SS1_CONF_REGS_I_TI_CONF_REGS,
  (void *)CSL_MPU_PCIE_SS1_CONF_REGS_I_PL_CONF_REGS,
  (uint32_t)0x1000
};

Pcie_DeviceCfgBaseAddr baseAddrDev1 =
{
  &cfgBaseAddrsDev1,
  (void *)CSL_MPU_PCIE_SS1_REGS,
  0U,
  NULL
};

Pciev1_DeviceCfgBaseAddrs cfgBaseAddrsDev2 =
{
  (void *)CSL_MPU_PCIE_SS2_CONF_REGS_I_RC_CFG_DBICS_REGS,
  (void *)CSL_MPU_PCIE_SS2_CONF_REGS_I_RC_CFG_DBICS2_REGS,
  (void *)CSL_MPU_PCIE_SS2_CONF_REGS_I_TI_CONF_REGS,
  (void *)CSL_MPU_PCIE_SS2_CONF_REGS_I_PL_CONF_REGS,
  (uint32_t)0x1000
};

Pcie_DeviceCfgBaseAddr baseAddrDev2 =
{
  &cfgBaseAddrsDev2,
  (void *)CSL_MPU_PCIE_SS2_REGS,
  0U,
  NULL
};

#else /* M4 */

Pciev1_DeviceCfgBaseAddrs cfgBaseAddrsDev1 =
{
  (void *)CSL_IPU_PCIE_SS1_CONF_REGS_I_RC_CFG_DBICS_REGS,
  (void *)CSL_IPU_PCIE_SS1_CONF_REGS_I_RC_CFG_DBICS2_REGS,
  (void *)CSL_IPU_PCIE_SS1_CONF_REGS_I_TI_CONF_REGS,
  (void *)CSL_IPU_PCIE_SS1_CONF_REGS_I_PL_CONF_REGS,
  (uint32_t)0x1000
};

Pcie_DeviceCfgBaseAddr baseAddrDev1 =
{
  &cfgBaseAddrsDev1,
  (void *)CSL_IPU_PCIE_SS1_REGS,
  0x04000000U,
  NULL
};

Pciev1_DeviceCfgBaseAddrs cfgBaseAddrsDev2 =
{
  (void *)CSL_IPU_PCIE_SS2_CONF_REGS_I_RC_CFG_DBICS_REGS,
  (void *)CSL_IPU_PCIE_SS2_CONF_REGS_I_RC_CFG_DBICS2_REGS,
  (void *)CSL_IPU_PCIE_SS2_CONF_REGS_I_TI_CONF_REGS,
  (void *)CSL_IPU_PCIE_SS2_CONF_REGS_I_PL_CONF_REGS,
  (uint32_t)0x1000
};

Pcie_DeviceCfgBaseAddr baseAddrDev2 =
{
  &cfgBaseAddrsDev2,
  (void *)CSL_IPU_PCIE_SS2_REGS,
  0x04000000U,
  NULL
};
#endif

/** @brief PCIE LLD initialization parameters */
const Pcie_InitCfg pcieInitCfg =
{
  {
    {
      &baseAddrDev1,
      &baseAddrDev2,
      NULL,
      NULL
    },
    {
      &fxnTablev1,
      &fxnTablev1,
      NULL,
      NULL
    }
  }
};

/**
@}
*/

