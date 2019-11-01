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
#include <ti/csl/cslr_tetris_vbusp.h>

#include "sbl_a15.h"
#include "sbl_slave_core_boot.h"

#if defined(BOOT_SPI)
#include "sbl_spi.h"
#endif

/* RBL address for DSP entry */
#define DSP_MAGIC_ADDRESS   0x008FFFFC
/* RBL address for ARM entry */
#define ARM_MAGIC_ADDRESS   0x0C5AD000
/* SBL address for ARM subcore entry */
#define SBL_ARM_MAGIC       0x0C0FFF00

/**
 * \brief - SBL_armInit() - Trampoline code for ARM cores 1-3.
 *          - Enables CP10 and CP11 non-secure access
 *          - Enables SMP bit in ACTLR
 *          - Enables non-secure mode
 *          - Branches to SBL_ARM_MAGIC when above inits are done
 *
 * \param   none
 *
 * \return  none
 *
 */
void SBL_armInit() {
    void (*monitorFunction) (void (*)(void), ...);
    void (*entry)();

    /* A15 startup calls */
    monitorFunction = (void (*)) 0x1000;
    (*monitorFunction)(SBL_setNSMode);
    (*monitorFunction)(SBL_a15EnableNeon);
    (*monitorFunction)(SBL_a15EnableSMP);

    entry = (void (*)) (*(uint32_t *) SBL_ARM_MAGIC);
    entry();
}

int32_t SBL_ImageCopy(sblEntryPoint_t *pEntry)
{
    int32_t retval = 0;

#if defined(BOOT_SPI)
    if (SBL_SPIBootImage(pEntry) != 1U)
#endif
    {
        retval = -1;
    }

    return retval;
}

void SBL_enableARMCores()
{
    CSL_TetrisVbuspRegs *hTetrisRegs = (CSL_TetrisVbuspRegs *) CSL_TETRIS_REGS;
    hTetrisRegs->PD_CPU1_PDCTL = 0x00000000;
    hTetrisRegs->PD_CPU1_PTCMD = 0x00000001;
    hTetrisRegs->PD_CPU2_PDCTL = 0x00000000;
    hTetrisRegs->PD_CPU2_PTCMD = 0x00000001;
    hTetrisRegs->PD_CPU3_PDCTL = 0x00000000;
    hTetrisRegs->PD_CPU3_PTCMD = 0x00000001;
}

void SBL_DSPBringUp(uint8_t core, uint32_t entry)
{
    /* IPC Acknowledgement */
    hBootCfg->IPCAR[core] = 0xFFFFFFF0U;

    /* Write boot address */
    *(uint32_t *) (DSP_MAGIC_ADDRESS | (1<<28) | (core<<24)) = entry;

    /* IPC Generation */
    hBootCfg->IPCGR[core] = (1 << 31) | 1;

}

void SBL_ARMBringUp(uint8_t core, uint32_t entry)
{
    /* Only boot ARM core 1-3 */
    if (core < 1 || core > 3)
    {
        return;
    }

    switch (core) {
        case 1:
            /* IPC Acknowledgement */
            hBootCfg->IPCAR[9] = 0xFFFFFFF0U;

            /* Set entry point to app after SBL_armInit */
            *(uint32_t *) (SBL_ARM_MAGIC) = entry;

            /* Write boot address */
            *(uint32_t *) (ARM_MAGIC_ADDRESS + (core<<2)) = (uint32_t) &SBL_armInit;

            /* IPC Generation */
            hBootCfg->IPCGR[9] = (1 << 31) | 1;
            break;
        case 2:
            /* IPC Acknowledgement */
            hBootCfg->IPCAR[10] = 0xFFFFFFF0U;

            /* Set entry point to app after SBL_armInit */
            *(uint32_t *) (SBL_ARM_MAGIC) = entry;

            /* Write boot address */
            *(uint32_t *) (ARM_MAGIC_ADDRESS + (core<<2)) = (uint32_t) &SBL_armInit;

            /* IPC Generation */
            hBootCfg->IPCGR[10] = (1 << 31) | 1;
            break;
        case 3:
            /* IPC Acknowledgement */
            hBootCfg->IPCAR[11] = 0xFFFFFFF0U;

            /* Set entry point to app after SBL_armInit */
            *(uint32_t *) (SBL_ARM_MAGIC) = entry;

            /* Write boot address */
            *(uint32_t *) (ARM_MAGIC_ADDRESS + (core<<2)) = (uint32_t) &SBL_armInit;

            /* IPC Generation */
            hBootCfg->IPCGR[11] = (1 << 31) | 1;
            break;
        default:
            break;
    }

}
