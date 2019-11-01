/**
 *  \file   sbl_slave_core_boot.h
 *
 *  \brief  This file contains prototypes of functions for slave core bring up.
 *
 */

/*
 * Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com/
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/*
 *    SOC core definitions
 */
typedef enum cpu_core_id
{
    MPU_CPU0_ID = 0,
    MPU_CPU1_ID,
    IPU1_CPU0_ID,
    IPU1_CPU1_ID,
    IPU1_CPU_SMP_ID,
    IPU2_CPU0_ID,
    IPU2_CPU1_ID,
    IPU2_CPU_SMP_ID,
    DSP1_ID,
    DSP2_ID,
} cpu_core_id_t;

/* Structure holding the entry address of the applications for different cores. */
typedef struct sblEntryPoint
{
    uint32_t    entryPoint_MPU_CPU0;
    /* Value holding the entry address of MPU_CPU0. */
    uint32_t    entryPoint_MPU_CPU1;
    /* Value holding the entry address of MPU_CPU1. */
    uint32_t    entryPoint_DSP1;
    /* Value holding the entry address of MPU_DSP1. */
    uint32_t    entryPoint_DSP2;
    /* Value holding the entry address of MPU_DSP2. */
    uint32_t    entryPoint_IPU1_CPU0;
    /* Value holding the entry address of IPU_CPU0. */
    uint32_t    entryPoint_IPU1_CPU1;
    /* Value holding the entry address of IPU1_CPU1. */
    uint32_t    entryPoint_IPU2_CPU0;
    /* Value holding the entry address of IPU2_CPU0. */
    uint32_t    entryPoint_IPU2_CPU1;
    /* Value holding the entry address of IPU2_CPU1. */
}sblEntryPoint_t;

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

/**
 * \brief    SBL_SlaveCorePrcmEnable function enables clock to the DSP and IPU
 *           slave cores and brings them out of reset to enable booting
 *           applications on them.
 *
 */
void SBL_SlaveCorePrcmEnable();

/**
 * \brief    SBL_DSP1_BringUp function asserts reset, sets the entry point
 *           and releases the DSP1 core from reset.
 *
 * \param    EntryPoint - CPU entry location on reset
 *
 **/
void SBL_DSP1_BringUp(uint32_t EntryPoint);

#if defined (AM572x_BUILD) || defined (AM574x_BUILD)
/**
 * \brief    SBL_DSP2_BringUp function asserts reset, sets the entry point
 *           and releases the DSP2 core from reset.
 *
 * \param    EntryPoint - CPU entry location on reset
 *
 **/
void SBL_DSP2_BringUp(uint32_t EntryPoint);
#endif

/**
 * \brief    SBL_IPU1_CPU0_BringUp function asserts reset, sets the entry point
 *           and release the CPU0 from reset.
 *
 * \param    EntryPoint - CPU entry location on reset
 *
 **/
void SBL_IPU1_CPU0_BringUp(uint32_t EntryPoint);

/**
 * \brief    SBL_IPU1_CPU1_BringUp function asserts reset, sets the entry point
 *           and release the CPU1 from reset.
 *
 * \param    EntryPoint - CPU entry location on reset
 *
 **/
void SBL_IPU1_CPU1_BringUp(uint32_t EntryPoint);

#if defined (AM572x_BUILD) || defined (AM574x_BUILD)
/**
 * \brief    SBL_IPU2_CPU0_BringUp function asserts reset, sets the entry point
 *           and release the CPU0 from reset.
 *
 * \param    EntryPoint - CPU entry location on reset
 *
 **/
void SBL_IPU2_CPU0_BringUp(uint32_t EntryPoint);

/**
 * \brief    SBL_IPU2_CPU1_BringUp function asserts reset, sets the entry point
 *           and release the CPU1 from reset.
 *
 * \param    EntryPoint - CPU entry location on reset
 *
 **/
void SBL_IPU2_CPU1_BringUp(uint32_t EntryPoint);

/**
 * \brief    SBL_MPU_CPU1_BringUp function asserts reset, sets the entry point
 *           and release the CPU1 from reset.
 *
 * \param    EntryPoint - CPU entry location on reset
 *
 **/
void SBL_MPU_CPU1_BringUp(uint32_t EntryPoint);
#endif

#ifdef __cplusplus
}
#endif

#endif
