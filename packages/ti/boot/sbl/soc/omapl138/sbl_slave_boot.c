/*
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <ti/csl/tistdtypes.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_psc.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_syscfgAux.h>

#include "sbl_slave_core_boot.h"

int32_t SBL_ImageCopy(sblEntryPoint_t *pEntry)
{
    int32_t retval = 0;

#if defined(BOOT_MMCSD)
    /* MMCSD Boot Mode Image Copy function. */
    if (SBL_MMCBootImage(pEntry) != 0)
    {
        retval = -1;
    }
#endif  

    return retval;
}

void SBL_DSPBringUp(uint8_t core, uint32_t entry)
{
	volatile uint32_t dspState;
	/* Add DSP init code */
	Board_unlockMMR();
	hSysCfg->HOST1CFG = entry;

	/* Set NEXT state to ENABLE */
	CSL_PSC_setModuleNextState(CSL_PSC_DSP, PSC_MODSTATE_ENABLE);
	/* Set GO bit to initiate state transition */
	CSL_PSC_startStateTransition(1);
	/* Wait for power state transition to finish */
	while (!CSL_PSC_isStateTransitionDone(1));
	/* Wait for state transition */
	while (CSL_PSC_getModuleState(CSL_PSC_DSP) != PSC_MODSTATE_ENABLE);
	/* Trigger DSP local reset */
	CSL_FINS (hPscRegs0->MDCTL[CSL_PSC_DSP], PSC_MDCTL_LRST, 1);
}
