/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#include "edmaPktMsiBench.h"
#include <ti/csl/soc.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/csl_pcie.h>

/* CSL-only implementation of send-msi to save latency */
void edmaPktBenchSendMsi (void)
{   
  HW_WR_REG32(
      CSL_MPU_PCIE_SS1_CONF_REGS_I_RC_CFG_DBICS_REGS + 
      PCIECTRL_TI_CONF_OFFSET + 
      PCIECTRL_TI_CONF_MSI_XMT, 1);
}

/* Wait for previous MSI to clear */
int32_t edmaPktBenchWaitMsi (volatile uint32_t *ticks, uint32_t timeout)
{
  uint32_t start_ticks = *ticks;
  int32_t retVal = 0;
  /* Check previous MSI went away */
  while (((HW_RD_REG32(
           CSL_MPU_PCIE_SS1_CONF_REGS_I_RC_CFG_DBICS_REGS + 
           PCIECTRL_TI_CONF_OFFSET + 
           PCIECTRL_TI_CONF_MSI_XMT) & 1u) != 0u) &&
          (((*ticks) - start_ticks) < timeout));
  if (((*ticks) - start_ticks) >= timeout)
  {
    retVal = 1;
  }

  return retVal;
}
