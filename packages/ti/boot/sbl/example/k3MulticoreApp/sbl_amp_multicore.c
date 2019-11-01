/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "sbl_amp_multicore_sections.h"

int sblTestmain(void)
{
    volatile int *pokeMemAddr = (volatile int *)POKE_MEM_ADDR;
    volatile int *bootFlagAddr = (volatile int *)POKE_MEM_ADDR_MCU1_0;
    volatile int boot_delay = BOOT_DELAY, num_cores_booted = 0;

    // if we have run before, someone
    // reset the system, dont print message
    if (*pokeMemAddr != 0xC0DEBABE)
    {
        while (boot_delay--);
        sbl_puts(CORE_NAME);
        sbl_puts(" running\n\r");
        // log completion
        *pokeMemAddr = 0xC0DEBABE;
    }

    // Check if all cores have run by checking flags
    // left in MSMC by each cores testcase
    while (bootFlagAddr <= (int *)POKE_MEM_ADDR_MPU2_1)
    {
        if (*bootFlagAddr == 0xC0DEBABE)
        {
            num_cores_booted++;
        }

        bootFlagAddr += 0x800;
    }

    if (num_cores_booted == SBL_AMP_TEST_NUM_BOOT_CORES)
    {
        sbl_puts(CORE_NAME);
        sbl_puts(" reports: All tests have passed\n\r");

        // Clean up pokemem flags for the next run
        for (bootFlagAddr = (volatile int *)POKE_MEM_ADDR_MCU1_0;
             bootFlagAddr <= (int *)POKE_MEM_ADDR_MPU2_1;
             bootFlagAddr += 0x800)
        {
            *bootFlagAddr = 0xFEEDFACE;
        }
    }

    return 0XFEEDFACE;
}

