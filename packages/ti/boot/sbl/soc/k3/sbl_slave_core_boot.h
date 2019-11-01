/**
 *  \file   sbl_slave_core_boot.h
 *
 *  \brief  This file contains prototypes of functions for slave core bring up.
 *
 */

/*
 * Copyright (C) 2018-2019 Texas Instruments Incorporated - http://www.ti.com/
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

#ifdef __cplusplus
extern "C" {
#endif

/*
 *    SOC core definitions
 */
typedef uint32_t cpu_core_id_t;
#define MPU1_CPU0_ID                    (0U)
#define MPU1_CPU1_ID                    (1U)
#define MPU2_CPU0_ID                    (2U)
#define MPU2_CPU1_ID                    (3U)
#define MCU1_CPU0_ID                    (4U)
#define MCU1_CPU1_ID                    (5U)
#define MCU2_CPU0_ID                    (6U)
#define MCU2_CPU1_ID                    (7U)
#define MCU3_CPU0_ID                    (8U)
#define MCU3_CPU1_ID                    (9U)
#define DSP1_C66X_ID                    (10U)
#define DSP2_C66X_ID                    (11U)
#define DSP1_C7X_ID                     (12U)
/* add additional MPU/MCU cores before this */
#define DSP2_C7X_ID                     (13U)
/* only SMP core ID should be after this */
#define MPU1_SMP_ID                     (14U)
#define MPU2_SMP_ID                     (15U)
#define MPU_SMP_ID                      (16U)
#define MCU1_SMP_ID                     (17U)
#define MCU2_SMP_ID                     (18U)
#define MCU3_SMP_ID                     (19U)
#define ONLY_LOAD_ID                    (20U)
#define NUM_CORES                       (21U)

/* Structure holding the entry address of the applications for different cores. */
typedef struct sblEntryPoint
{
    /* Holds Entry point of each core. */
    uint32_t    CpuEntryPoint[NUM_CORES];
}sblEntryPoint_t;

/* Structure holding information about a core that is needed to reset it. */
typedef struct
{
    /* Proc id of a core. */
    int32_t    tisci_proc_id;
    /* Device id of a core. */
    int32_t    tisci_dev_id;
    /* Clk id of a core. */
    int32_t    tisci_clk_id;
    /* Startup freq of a core in Hz */
    int32_t    slave_clk_freq_hz;

}sblSlaveCoreInfo_t;

#define SBL_MCU_LOCKSTEP_ADDR    (SBL_INVALID_ENTRY_ADDR + 1)
/*
 *  \brief    SBL_ImageCopy function is a wrapper to Multicore Image parser
 *            function. Based on boot-mode jumps into specific Image copy
 *            function for the particular bootmode.
 *
 *  \param    pointer to the structure holding the entry pointers for different
 *            cores. This gets updated with the entry point of the images for
 *            different cores by this funciton.
 *
 *  \return   error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 *
 */
int32_t SBL_ImageCopy(sblEntryPoint_t *pEntry);

/**
 * \brief    SBL_SlaveCoreBoot function sets the entry point, sets up clocks
 *           and enable to core to start executing from entry point.
 *
 * \param    CoreID = Core ID. Differs depending on SOC, refer to cpu_core_id enum
 *           freqHz = Speed of core at boot up, 0 indicates use SBL default freqs.
 *           pAppEntry = SBL entry point struct
 *
 **/
void SBL_SlaveCoreBoot(cpu_core_id_t core_id, uint32_t freqHz, sblEntryPoint_t *pAppEntry);


#ifdef __cplusplus
}
#endif

#endif
