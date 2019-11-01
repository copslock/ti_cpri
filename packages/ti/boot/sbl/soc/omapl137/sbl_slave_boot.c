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

#include <ti/csl/csl.h>
#include <ti/csl/tistdtypes.h>
#include <ti/csl/cslr_icss.h>
#include <ti/csl/cslr_psc.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_syscfgAux.h>
#include <ti/csl/soc.h>
#include "sbl_slave_core_boot.h"

#if defined(BOOT_SPI)
#include "sbl_spi.h"
#endif

/************************************************************
* Local Function Definitions                                *
************************************************************/
static const uint32_t armBootPruCodeConst[] = {
	0x24000080,             /* Load ARM vector table address to r0 */
	0x24ffffc0,
	0x24000081,             /* Load ARM code address to r1 */
	0x240000c1,
	0xf500e182,             /* Read ARM code from *r1 to r2,... */
	0xe500e082,             /* Write ARM code from r2,... to *r0 */
	0x79000000,             /* Self loop */
};

static const uint32_t armBootArmCodeConst[] = {
	0xEA000007,             /* vt0: B boot */
	0xEAFFFFFE,             /* vt0: Self loop */
	0xEAFFFFFE,             /* vt0: Self loop */
	0xEAFFFFFE,             /* vt0: Self loop */
	0xEAFFFFFE,             /* vt0: Self loop */
	0xEAFFFFFE,             /* vt0: Self loop */
	0xEAFFFFFE,             /* vt0: Self loop */
	0xEAFFFFFE,             /* vt0: Self loop */
	/* jump:, DATA: */
	0x00000000,
	/* boot: */
	0xE51F000C,             /* LDR R0, jump */
	0xE1A0F000,             /* MOV PC, R0 */
};

/* Configures ARM reset vector */
static void SBL_SetArmResetVector (uint32_t address)
{
    uint32_t index;
    uint32_t *pruRam = (uint32_t *) PRUCORE_0_IRAM_BASEADDR;
    uint32_t *armCode = (uint32_t *) armBootArmCodeConst;
    uint32_t *pruCode = (uint32_t *) armBootPruCodeConst;
    CSL_IcssPruCtrlRegs *pruRegs = (CSL_IcssPruCtrlRegs *)PRUCORE_0_REGS;

    /* Update ARM entry point address */
    armCode[8] = address;

    /* Tell location of armCode to PRUSS code */
    pruCode[2] |= ((uint32_t) armCode & 0xffff) << 8;
    pruCode[3] |= ((uint32_t) armCode & 0xffff0000) >> 8;

    /* Power on PRUSS */
    Board_moduleClockEnable(CSL_PSC_PRU);

    /* Reset PRUSS */
    pruRegs->CONTROL = 0;

    /* Copy PRUSS code to its instruction RAM */
    for (index = 0; index < sizeof (armBootPruCodeConst) / sizeof (uint32_t); index++)
    {
        *pruRam++ = pruCode[index];
    }

    /* Enable PRUSS, let it execute the code we just copied */
    pruRegs->CONTROL |= (1 << 3);
    pruRegs->CONTROL |= (1 << 1);

    /* Wait for PRUSS to finish */
    while (pruRegs->STATUS != (sizeof (armBootPruCodeConst) / sizeof (uint32_t)) - 1);

    /* Reset PRUSS */
    pruRegs->CONTROL = 0;

    /* Wake-up ARM */
    hSysCfg->HOST0CFG = 0x00000001;
}

/* Function to copy the image from flash to memory */
int32_t SBL_ImageCopy(sblEntryPoint_t *pEntry)
{
    int32_t retval = 0;

#if defined(BOOT_SPI)
    if (SBL_SPIBootImage(pEntry) != 0U)
#endif
    {
        retval = -1;
    }

    return retval;
}

/* Function to bring-up the ARM core out of reset to exeute the code
   loaded by DSP core. */
void SBL_ARMBringUp(uint8_t core, uint32_t entry)
{
    Board_unlockMMR();
    /* Turn on ARM RAM */
    Board_moduleClockEnable(CSL_PSC_ARM_RAMROM);

    /* Configure ARM reset vector */
    SBL_SetArmResetVector(entry);

    /* Turn on ARM */
    Board_moduleClockEnable(CSL_PSC_ARM);

    /* Trigger ARM local reset */
    CSL_FINS (hPscRegs0->MDCTL[CSL_PSC_ARM], PSC_MDCTL_LRST, 1);
}
