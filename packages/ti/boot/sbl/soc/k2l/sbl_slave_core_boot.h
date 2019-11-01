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

#ifndef SBL_SLAVE_CORE_BOOT_H_
#define SBL_SLAVE_CORE_BOOT_H_

/*
 *    SOC core definitions
 */
typedef enum cpu_core_id
{
    MPU_CPU0_ID = 0,
    MPU_CPU1_ID = 1,
    MPU_SMP_ID = 4,
    DSP0_ID = 5,
    DSP1_ID = 6,
    DSP2_ID = 7,
    DSP3_ID = 8
} cpu_core_id_t;

/* Structure holding the entry address of the applications for different cores. */
typedef struct sblEntryPoint
{
    uint32_t entryPoint_MPU_CPU0;
    uint32_t entryPoint_MPU_CPU1;
    uint32_t entryPoint_DSP0;
    uint32_t entryPoint_DSP1;
    uint32_t entryPoint_DSP2;
    uint32_t entryPoint_DSP3;
} sblEntryPoint_t;

/*
 *  \brief    SBL_ImageCopy function is a wrapper to Multicore Image parser
 *            function. Based on boot-mode jumps into specific Image copy
 *            function for the particular bootmode.
 *
 *  \param    pointer to the structure holding the entry pointers for different
 *            cores.
 *
 *  \return   error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 *
 */
int32_t SBL_ImageCopy(sblEntryPoint_t *pEntry);

/*
 * \brief    SBL_BootCore populates the entry point for each core
 *
 * \param
 *      entry = entry address
 *      CoreID = Core ID. Differs depending on SOC, refer to cpu_core_id enum
 *      pAppEntry = SBL entry point struct
 *
 */
void SBL_BootCore(uint32_t entry, uint32_t CoreID, sblEntryPoint_t *pAppEntry);

/*
 * \brief    SBL_DSPBringUp sets the entry point and kick off application
 *           for DSP cores
 *
 * \param
 *      core = DSP core number
 *      entry = application entry address
 *
 */
void SBL_DSPBringUp(uint8_t core, uint32_t entry);

/*
 * \brief    SBL_ARMBringUp sets the entry point and kick off application
 *           for ARM cores
 *
 * \param
 *      core = ARM core number. Cannot be 0.
 *      entry = application entry address
 *
 */
void SBL_ARMBringUp(uint8_t core, uint32_t entry);

#endif
