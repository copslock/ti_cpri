/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_pscAux.h>
#include "sbl_slave_core_boot.h"

#if defined(BOOT_QSPI)
#include "sbl_qspi.h"
#elif defined(BOOT_MMCSD)
#include "sbl_mmcsd.h"
#endif

/* Entry address for DSP1 applications in the memory region where trampolene is located */
#define DSP1_ENTRY_ADDR         (0x0C0FFC00)

/*
 * DSP instructions to be copied to the Trampolene to avoid the alignment
 * requirement.
 */
uint32_t dsp1Instruction[10] =
{
    0x0500002a, /* MVK.S2  destAddr, B10      */
    0x0500006a, /* MVKH.S2 destAddr, B10      */
    0x00280362, /* B.S2    B10                */
    0x00006000, /* NOP     4                  */
    0x00000000, /* NOP                        */
    0x00000000  /* NOP                        */
};

int32_t SBL_ImageCopy(sblEntryPoint_t *pEntry)
{
    int32_t retval = 0;

#if defined(BOOT_MMCSD)
    if (SBL_MMCBootImage(pEntry) != 1U)
#elif defined(BOOT_QSPI)
    if (SBL_QSPIBootImage(pEntry) != 1U)
#endif
    {
        retval = -1;
    }

    return retval;
}

static void SBL_DspEntryPointSet(uint32_t entryPoint, uint32_t *pDspInstr)
{
    uint32_t entryVal = 0U;
    uint32_t dspOpcode = 0U;

    entryVal = (entryPoint & 0x0000FFFF);
    dspOpcode = *pDspInstr;
    /*
    ** Mask and update the lower 16 bits of entry address within the MVK
    ** instruction opcode.
    */
    *pDspInstr = (((dspOpcode) & ~(0x007FFF80)) | (( (entryVal) << 0x7) & 0x007FFF80));

    entryVal = ((entryPoint & 0xFFFF0000) >> 16);
    dspOpcode = *(pDspInstr + 1);
    /*
    ** Mask and update the upper 16 bits of entry address within the MVK
    ** instruction opcode.
    */
    *(pDspInstr + 1) = (((dspOpcode) & ~(0x007FFF80)) | (( (entryVal) << 0x7) & 0x007FFF80));
}

void SBL_DSPBringUp(uint8_t core, uint32_t entry)
{
    CSL_PSC_setModuleLocalReset(18, PSC_MDLRST_ASSERTED);

    SBL_DspEntryPointSet(entry, dsp1Instruction);

    memcpy((void *) DSP1_ENTRY_ADDR, (void *)dsp1Instruction,
        sizeof(dsp1Instruction));

    /* Set DSP boot address to trampoline for both Secure and NS mode */
    CSL_BootCfgSetDSPBootAddress(0, (DSP1_ENTRY_ADDR >> 10));
    CSL_FINS(hBootCfg->DSP_BOOT_ADDR0_NS, BOOTCFG_DSP_BOOT_ADDR0_NS_ISTP_RST_VAL, (DSP1_ENTRY_ADDR >> 10));

    CSL_PSC_setModuleLocalReset(18, PSC_MDLRST_DEASSERTED);
}
