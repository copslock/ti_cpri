/**
 *  \file   sbl_slave_core_boot.c
 *
 *  \brief  This file contain functions related to slave core boot-up.
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

 #include <stdint.h>
 #include <string.h>
 #include <ti/csl/csl_types.h>
 #include <ti/csl/cslr_device.h>
 #include <ti/csl/hw_types.h>
 #include <ti/csl/arch/csl_arch.h>
 #include <ti/drv/uart/UART_stdio.h>

#include "sbl_soc.h"
#include "sbl_log.h"
#include "sbl_soc_cfg.h"
#include "sbl_profile.h"
#include "sbl_err_trap.h"
#include "sbl_sci_client.h"
#include "sbl_slave_core_boot.h"

#if defined(BOOT_OSPI)
#include "sbl_ospi.h"
#endif

#if defined(BOOT_MMCSD)
#include "sbl_mmcsd.h"
#endif

#if defined(BOOT_UART)
#include "sbl_uart.h"
#endif

#if defined(BOOT_HYPERFLASH)
#include "sbl_hyperflash.h"
#endif
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define SBL_DISABLE_MCU_LOCKSTEP    (0)
#define SBL_ENABLE_MCU_LOCKSTEP     (1)

/* Don't forget to update parameter OPP of the AVS   */
/* setup function in SBL_SocLateInit if the CPU freq */
/* are changed to a higher or lower operating  point */
static const sblSlaveCoreInfo_t sbl_slave_core_info[] =
{
    /* MPU1_CPU0 info */
    {
    SBL_PROC_ID_MPU1_CPU0,
    SBL_DEV_ID_MPU1_CPU0,
    SBL_CLK_ID_MPU1_CPU0,
    SBL_MPU1_CPU0_FREQ_HZ,
    },
    /* MPU1_CPU1 info */
    {
    SBL_PROC_ID_MPU1_CPU1,
    SBL_DEV_ID_MPU1_CPU1,
    SBL_CLK_ID_MPU1_CPU1,
    SBL_MPU1_CPU1_FREQ_HZ,
    },
    /* MPU2_CPU0 info */
    {
    SBL_PROC_ID_MPU2_CPU0,
    SBL_DEV_ID_MPU2_CPU0,
    SBL_CLK_ID_MPU2_CPU0,
    SBL_MPU2_CPU0_FREQ_HZ,
    },
    /* MPU2_CPU1 info */
    {
    SBL_PROC_ID_MPU2_CPU1,
    SBL_DEV_ID_MPU2_CPU1,
    SBL_CLK_ID_MPU2_CPU1,
    SBL_MPU2_CPU1_FREQ_HZ,
    },
    /* MCU1_CPU0 info */
    {
    SBL_PROC_ID_MCU1_CPU0,
    SBL_DEV_ID_MCU1_CPU0,
    SBL_CLK_ID_MCU1_CPU0,
    SBL_MCU1_CPU0_FREQ_HZ,
    },
    /* MCU1_CPU1 info */
    {
    SBL_PROC_ID_MCU1_CPU1,
    SBL_DEV_ID_MCU1_CPU1,
    SBL_CLK_ID_MCU1_CPU1,
    SBL_MCU1_CPU1_FREQ_HZ,
    },
    /* MCU2_CPU0 info */
    {
    SBL_PROC_ID_MCU2_CPU0,
    SBL_DEV_ID_MCU2_CPU0,
    SBL_CLK_ID_MCU2_CPU0,
    SBL_MCU2_CPU0_FREQ_HZ,
    },
    /* MCU2_CPU1 info */
    {
    SBL_PROC_ID_MCU2_CPU1,
    SBL_DEV_ID_MCU2_CPU1,
    SBL_CLK_ID_MCU2_CPU1,
    SBL_MCU2_CPU1_FREQ_HZ,
    },
    /* MCU3_CPU0 info */
    {
    SBL_PROC_ID_MCU3_CPU0,
    SBL_DEV_ID_MCU3_CPU0,
    SBL_CLK_ID_MCU3_CPU0,
    SBL_MCU3_CPU0_FREQ_HZ,
    },
    /* MCU3_CPU1 info */
    {
    SBL_PROC_ID_MCU3_CPU1,
    SBL_DEV_ID_MCU3_CPU1,
    SBL_CLK_ID_MCU3_CPU1,
    SBL_MCU3_CPU1_FREQ_HZ,
    },
    /* DSP1_C66X info */
    {
    SBL_PROC_ID_DSP1_C66X,
    SBL_DEV_ID_DSP1_C66X,
    SBL_CLK_ID_DSP1_C66X,
    SBL_DSP1_C66X_FREQ_HZ,
    },
    /* DSP2_C66X info */
    {
    SBL_PROC_ID_DSP2_C66X,
    SBL_DEV_ID_DSP2_C66X,
    SBL_CLK_ID_DSP2_C66X,
    SBL_DSP2_C66X_FREQ_HZ,
    },
    /* DSP1_C7X info */
    {
    SBL_PROC_ID_DSP1_C7X,
    SBL_DEV_ID_DSP1_C7X,
    SBL_CLK_ID_DSP1_C7X,
    SBL_DSP1_C7X_FREQ_HZ,
    },
    /* DSP2_C7X info */
    {
    SBL_PROC_ID_DSP2_C7X,
    SBL_DEV_ID_DSP2_C7X,
    SBL_CLK_ID_DSP2_C7X,
    SBL_DSP2_C7X_FREQ_HZ,
    }
};

static const uint32_t SblAtcmAddr[] =
{
SBL_MCU_ATCM_BASE,
SBL_MCU1_CPU1_ATCM_BASE_ADDR_SOC,
SBL_MCU2_CPU0_ATCM_BASE_ADDR_SOC,
SBL_MCU2_CPU1_ATCM_BASE_ADDR_SOC,
SBL_MCU3_CPU0_ATCM_BASE_ADDR_SOC,
SBL_MCU3_CPU1_ATCM_BASE_ADDR_SOC
};

static const uint32_t SblBtcmAddr[] =
{
SBL_MCU_BTCM_BASE,
SBL_MCU1_CPU1_BTCM_BASE_ADDR_SOC,
SBL_MCU2_CPU0_BTCM_BASE_ADDR_SOC,
SBL_MCU2_CPU1_BTCM_BASE_ADDR_SOC,
SBL_MCU3_CPU0_BTCM_BASE_ADDR_SOC,
SBL_MCU3_CPU1_BTCM_BASE_ADDR_SOC
};
/* ========================================================================== */
/*                           Internal Functions                               */
/* ========================================================================== */

static void SBL_RequestAllCores(void)
{
#if !defined(SBL_SKIP_BRD_CFG_BOARD) && !defined(SBL_SKIP_SYSFW_INIT)
    uint32_t i;
    int32_t status = CSL_EFAIL;
    uint32_t num_cpus = sizeof(sbl_slave_core_info)/ sizeof(sblSlaveCoreInfo_t);

    SBL_ADD_PROFILE_POINT;

    for (i = 0; i < num_cpus; i++)
    {
        if(sbl_slave_core_info[i].tisci_proc_id != 0xBAD00000)
        {
            SBL_log(SBL_LOG_MAX, "Calling Sciclient_procBootRequestProcessor, ProcId 0x%x... \n", sbl_slave_core_info[i].tisci_proc_id);
            status = Sciclient_procBootRequestProcessor(sbl_slave_core_info[i].tisci_proc_id, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootRequestProcessor, ProcId 0x%x...FAILED \n", sbl_slave_core_info[i].tisci_proc_id);
                SblErrLoop(__FILE__, __LINE__);
            }
        }
    }

    SBL_ADD_PROFILE_POINT;
#endif

    return;
}

static void SBL_ReleaseAllCores(void)
{
#if !defined(SBL_SKIP_BRD_CFG_BOARD) && !defined(SBL_SKIP_SYSFW_INIT)
    uint32_t i;
    int32_t status = CSL_EFAIL;
    uint32_t num_cpus = sizeof(sbl_slave_core_info)/sizeof(sblSlaveCoreInfo_t);

    SBL_ADD_PROFILE_POINT;

    for (i = 0; i < num_cpus; i++)
    {
        if(sbl_slave_core_info[i].tisci_proc_id != 0xBAD00000)
        {
            SBL_log(SBL_LOG_MAX, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...\n", sbl_slave_core_info[i].tisci_proc_id);
            status = Sciclient_procBootReleaseProcessor(sbl_slave_core_info[i].tisci_proc_id, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...FAILED \n", sbl_slave_core_info[i].tisci_proc_id);
                SblErrLoop(__FILE__, __LINE__);
            }
        }
    }

    SBL_ADD_PROFILE_POINT;
#endif

    return;
}

static void SBL_ConfigMcuLockStep(uint8_t enableLockStep, const sblSlaveCoreInfo_t *sblCoreInfoPtr)
{
    int32_t status = CSL_EFAIL;
    struct tisci_msg_proc_get_status_resp cpuStatus;
    struct tisci_msg_proc_set_config_req  proc_set_config_req;

    SBL_ADD_PROFILE_POINT;

    SBL_log(SBL_LOG_MAX, "Calling Sciclient_procBootGetProcessorState, ProcId 0x%x... \n", sblCoreInfoPtr->tisci_proc_id);
    status = Sciclient_procBootGetProcessorState(sblCoreInfoPtr->tisci_proc_id, &cpuStatus, SCICLIENT_SERVICE_WAIT_FOREVER);
    if (status != CSL_PASS)
    {
        SBL_log(SBL_LOG_ERR, "Sciclient_procBootGetProcessorState...FAILED \n");
        SblErrLoop(__FILE__, __LINE__);
    }

    proc_set_config_req.processor_id = cpuStatus.processor_id;
    proc_set_config_req.bootvector_lo = cpuStatus.bootvector_lo;
    proc_set_config_req.bootvector_hi = cpuStatus.bootvector_hi;
    proc_set_config_req.config_flags_1_set = 0;
    proc_set_config_req.config_flags_1_clear = 0;

    if (enableLockStep)
    {
        SBL_log(SBL_LOG_MAX, "Sciclient_procBootSetProcessorCfg, ProcId 0x%x, enabling Lockstep mode...\n");
        proc_set_config_req.config_flags_1_set |= TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_LOCKSTEP;
    }
    else
    {
        SBL_log(SBL_LOG_MAX, "Sciclient_procBootSetProcessorCfg, ProcId 0x%x, enabling split mode...\n");
        proc_set_config_req.config_flags_1_clear |= TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_LOCKSTEP;
    }

    SBL_ADD_PROFILE_POINT;

    status =  Sciclient_procBootSetProcessorCfg(&proc_set_config_req,  SCICLIENT_SERVICE_WAIT_FOREVER);
    if (status != CSL_PASS)
    {
        SBL_log(SBL_LOG_ERR, "Sciclient_procBootSetProcessorCfg...FAILED \n");
        SblErrLoop(__FILE__, __LINE__);
    }

    SBL_ADD_PROFILE_POINT;

    return;
}

int32_t SBL_ImageCopy(sblEntryPoint_t *pEntry)
{
    int32_t retval = 0;
    cpu_core_id_t core_id;

    SBL_ADD_PROFILE_POINT;

    /* Initialize the entry point array to 0. */
    for (core_id = MPU1_CPU0_ID; core_id < NUM_CORES; core_id ++)
        pEntry->CpuEntryPoint[core_id] = SBL_INVALID_ENTRY_ADDR;

    /* Request SYSW for control of all cores */
    SBL_RequestAllCores();

    SBL_ADD_PROFILE_POINT;

#if defined(BOOT_MMCSD)
    /* MMCSD Boot Mode Image Copy function. */
    if (SBL_MMCBootImage(pEntry) != E_PASS)
#elif defined(BOOT_OSPI)
    if (SBL_OSPIBootImage(pEntry) != E_PASS)
#elif defined(BOOT_UART)
    if (SBL_UARTBootImage(pEntry) != E_PASS)
#elif defined(BOOT_HYPERFLASH)
    if (SBL_HYPERFLASHBootImage(pEntry) != E_PASS)
#endif
    {
        retval = E_FAIL;
    }

    SBL_ADD_PROFILE_POINT;

    /* Release control of all cores */
    SBL_ReleaseAllCores();

    SBL_ADD_PROFILE_POINT;

    return retval;
}

/**
 * \brief        SBL_SetupCoreMem function sets up the CPUs internal memory
 *
 * \param[in]    core_id - CPU ID
 * \param[in]    pAppEntry - Core info struct
 *
 * \return   none
 */
void SBL_SetupCoreMem(uint32_t core_id)
{
    int32_t status = CSL_EFAIL;
    uint8_t runLockStep = 0;
    struct tisci_msg_proc_get_status_resp cpuStatus;
    struct tisci_msg_proc_set_config_req  proc_set_config_req;
    const sblSlaveCoreInfo_t *sblSlaveCoreInfoPtr;

    SBL_ADD_PROFILE_POINT;

    /* Remap virtual core-ids if needed */
    switch (core_id)
    {
        case MCU1_SMP_ID:
            runLockStep = 1;
            core_id = MCU1_CPU0_ID;
            break;
        case MCU2_SMP_ID:
            runLockStep = 1;
            core_id = MCU2_CPU0_ID;
            break;
        case MCU3_SMP_ID:
            runLockStep = 1;
            core_id = MCU3_CPU0_ID;
            break;
        default:
            break;
    }

    sblSlaveCoreInfoPtr = &(sbl_slave_core_info[core_id]);

    if(runLockStep)
    {
        SBL_log(SBL_LOG_MAX, "Detected locktep for core_id %d, proc_id 0x%x... \n", core_id, sblSlaveCoreInfoPtr->tisci_proc_id);
        SBL_ConfigMcuLockStep(SBL_ENABLE_MCU_LOCKSTEP, sblSlaveCoreInfoPtr);
    }

    switch (core_id)
    {

        case DSP1_C66X_ID:
            break;
        case DSP2_C66X_ID:
            break;
        case DSP1_C7X_ID:
            break;
        case DSP2_C7X_ID:
            break;

        case MCU1_CPU1_ID:
        case MCU2_CPU1_ID:
        case MCU3_CPU1_ID:
            SBL_log(SBL_LOG_MAX, "Switching core id %d, proc_id 0x%x to split mode %d... \n", core_id-1, sbl_slave_core_info[core_id-1].tisci_proc_id);
            /* Image for second MCU core present, disable lock step for the cluster */
            SBL_ConfigMcuLockStep(SBL_DISABLE_MCU_LOCKSTEP, &(sbl_slave_core_info[core_id-1]));
            /* DOnt break, fall through for enabling TCMs */
        case MCU1_CPU0_ID:
        case MCU2_CPU0_ID:
        case MCU3_CPU0_ID:
            SBL_log(SBL_LOG_MAX, "Calling Sciclient_procBootGetProcessorState, ProcId 0x%x... \n", sblSlaveCoreInfoPtr->tisci_proc_id);
            status = Sciclient_procBootGetProcessorState(sblSlaveCoreInfoPtr->tisci_proc_id, &cpuStatus, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootGetProcessorState...FAILED \n");
                SblErrLoop(__FILE__, __LINE__);
            }

            proc_set_config_req.processor_id = cpuStatus.processor_id;
            proc_set_config_req.bootvector_lo = cpuStatus.bootvector_lo;
            proc_set_config_req.bootvector_hi = cpuStatus.bootvector_hi;
            proc_set_config_req.config_flags_1_set = 0;
            proc_set_config_req.config_flags_1_clear = 0;
            SBL_log(SBL_LOG_MAX, "Enabling MCU TCMs after reset for core %d\n", core_id);
            proc_set_config_req.config_flags_1_set |= (TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_BTCM_EN |
                                                       TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_ATCM_EN |
                                                       TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_TCM_RSTBASE);

            SBL_log(SBL_LOG_MAX, "Sciclient_procBootSetProcessorCfg, ProcId 0x%x, enabling TCMs...\n");
            status =  Sciclient_procBootSetProcessorCfg(&proc_set_config_req,  SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootSetProcessorCfg...FAILED \n");
                SblErrLoop(__FILE__, __LINE__);
            }
            /* SBL running on MCU0, don't fool around with its power */
            if (core_id != MCU1_CPU0_ID)
            {
                SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState Off, DevId 0x%x... \n", sblSlaveCoreInfoPtr->tisci_dev_id);
                Sciclient_pmSetModuleState(sblSlaveCoreInfoPtr->tisci_dev_id, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            SBL_log(SBL_LOG_MAX, "Setting HALT for ProcId 0x%x...\n", sblSlaveCoreInfoPtr->tisci_proc_id);
            status =  Sciclient_procBootSetSequenceCtrl(sblSlaveCoreInfoPtr->tisci_proc_id, TISCI_MSG_VAL_PROC_BOOT_CTRL_FLAG_R5_CORE_HALT, 0, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootSetSequenceCtrl...FAILED \n");
                SblErrLoop(__FILE__, __LINE__);
            }
            /* SBL running on MCU0, don't fool around with its power & TCMs */
            if (core_id != MCU1_CPU0_ID)
            {
                SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState On, DevId 0x%x... \n", sblSlaveCoreInfoPtr->tisci_dev_id);
                Sciclient_pmSetModuleState(sblSlaveCoreInfoPtr->tisci_dev_id, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);

                /* Initialize the TCMs - TCMs of MCU running SBL are already initialized by ROM & SBL */
                SBL_log(SBL_LOG_MAX, "Clearing core_id %d  ATCM @ 0x%x ", core_id, SblAtcmAddr[core_id - MCU1_CPU0_ID]);
                memset(((void *)(SblAtcmAddr[core_id - MCU1_CPU0_ID])), 0xFF, 0x8000);
                SBL_log(SBL_LOG_MAX, "& BTCM @0x%x\n", SblBtcmAddr[core_id - MCU1_CPU0_ID]);
                memset(((void *)(SblBtcmAddr[core_id - MCU1_CPU0_ID])), 0xFF, 0x8000);
            }
            break;
        case MPU1_SMP_ID:
        case MPU1_CPU0_ID:
        case MPU1_CPU1_ID:
            SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState On, DevId 0x%x... \n", SBL_DEV_ID_MPU_CLUSTER0);
            Sciclient_pmSetModuleState(SBL_DEV_ID_MPU_CLUSTER0, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            break;
        case MPU2_SMP_ID:
        case MPU2_CPU0_ID:
        case MPU2_CPU1_ID:
            SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState On, DevId 0x%x... \n", SBL_DEV_ID_MPU_CLUSTER1);
            Sciclient_pmSetModuleState(SBL_DEV_ID_MPU_CLUSTER1, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            break;
        case MPU_SMP_ID:
            /* Enable SMP on all MPU clusters. Enable SMP only if cluster is present */
            if (SBL_DEV_ID_MPU_CLUSTER0 != 0xBAD00000)
            {
                SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState On, DevId 0x%x... \n", SBL_DEV_ID_MPU_CLUSTER0);
                Sciclient_pmSetModuleState(SBL_DEV_ID_MPU_CLUSTER0, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            if (SBL_DEV_ID_MPU_CLUSTER1 != 0xBAD00000)
            {
                SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState On, DevId 0x%x... \n", SBL_DEV_ID_MPU_CLUSTER1);
                Sciclient_pmSetModuleState(SBL_DEV_ID_MPU_CLUSTER1, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            break;
        default:
            /* No special memory setup needed */
            break;
    }

    SBL_ADD_PROFILE_POINT;

    return;
}

/**
 * \brief    SBL_SlaveCoreBoot function sets the entry point, sets up clocks
 *           and enable to core to start executing from entry point.
 *
 * \param    core_id = Selects a core on the SOC, refer to cpu_core_id_t enum
 *           freqHz = Speed of core at boot up, 0 indicates use SBL default freqs.
 *           pAppEntry = SBL entry point struct
 *
 **/
void SBL_SlaveCoreBoot(cpu_core_id_t core_id, uint32_t freqHz, sblEntryPoint_t *pAppEntry)
{
    int32_t status = CSL_EFAIL;
    struct tisci_msg_proc_set_config_req  proc_set_config_req;
    const sblSlaveCoreInfo_t *sblSlaveCoreInfoPtr = &(sbl_slave_core_info[core_id]);

    SBL_ADD_PROFILE_POINT;

#if defined(SBL_SKIP_MCU_RESET) && (defined(SBL_SKIP_BRD_CFG_BOARD) || defined(SBL_SKIP_BRD_CFG_PM) || defined(SBL_SKIP_SYSFW_INIT))
    /* Skip copy if R5 app entry point is already 0 */
    if ((core_id == MCU1_CPU0_ID) &&
       (pAppEntry->CpuEntryPoint[core_id]) &&
       (pAppEntry->CpuEntryPoint[core_id] <  SBL_INVALID_ENTRY_ADDR))
    {
        SBL_log(SBL_LOG_MAX, "Copying first 128 byptes from app to MCU ATCM @ 0x%x for core %d\n", SblAtcmAddr[core_id - MCU1_CPU0_ID], core_id);
        memcpy(((void *)(SblAtcmAddr[core_id - MCU1_CPU0_ID])), (void *)(pAppEntry->CpuEntryPoint[core_id]), 128);
        return;
    }

    /* Finished processing images for all cores, start MCU_0 */
    if ((core_id == MCU1_CPU1_ID) &&
        (pAppEntry->CpuEntryPoint[core_id] >=  SBL_INVALID_ENTRY_ADDR))
    {
            /* Display profile logs */
            SBL_printProfileLog();

            SBL_log(SBL_LOG_MAX, "Starting app, branching to 0x0 \n");
            /* Branch to start of ATCM */
            ((void(*)(void))0x0)();
    }
#endif

    SBL_log(SBL_LOG_MAX, "Calling Sciclient_procBootRequestProcessor, ProcId 0x%x... \n", sblSlaveCoreInfoPtr->tisci_proc_id);
    status = Sciclient_procBootRequestProcessor(sblSlaveCoreInfoPtr->tisci_proc_id, SCICLIENT_SERVICE_WAIT_FOREVER);
    if (status != CSL_PASS)
    {
        SBL_log(SBL_LOG_ERR, "Sciclient_procBootRequestProcessor...FAILED \n");
        SblErrLoop(__FILE__, __LINE__);
    }

    proc_set_config_req.processor_id = sblSlaveCoreInfoPtr->tisci_proc_id;
    proc_set_config_req.bootvector_lo = pAppEntry->CpuEntryPoint[core_id];
    proc_set_config_req.bootvector_hi = 0x0;
    proc_set_config_req.config_flags_1_set = 0;
    proc_set_config_req.config_flags_1_clear = 0;


    if (pAppEntry->CpuEntryPoint[core_id] <  SBL_INVALID_ENTRY_ADDR) /* Set entry point only is valid */
    {
        SBL_log(SBL_LOG_MAX, "Sciclient_procBootSetProcessorCfg, ProcId 0x%x, EntryPoint 0x%x...\n", proc_set_config_req.processor_id, proc_set_config_req.bootvector_lo);
        SBL_ADD_PROFILE_POINT;
        status =  Sciclient_procBootSetProcessorCfg(&proc_set_config_req,  SCICLIENT_SERVICE_WAIT_FOREVER);
        if (status != CSL_PASS)
        {
            SBL_log(SBL_LOG_ERR, "Sciclient_procBootSetProcessorCfg...FAILED \n");
            SblErrLoop(__FILE__, __LINE__);
        }

        SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleClkFreq, DevId 0x%x @ %dHz... \n", sblSlaveCoreInfoPtr->tisci_dev_id, sblSlaveCoreInfoPtr->slave_clk_freq_hz);
        SBL_ADD_PROFILE_POINT;
        Sciclient_pmSetModuleClkFreq(sblSlaveCoreInfoPtr->tisci_dev_id,
                                     sblSlaveCoreInfoPtr->tisci_clk_id,
                                     sblSlaveCoreInfoPtr->slave_clk_freq_hz,
                                     TISCI_MSG_FLAG_AOP,
                                     SCICLIENT_SERVICE_WAIT_FOREVER);
        SBL_ADD_PROFILE_POINT;
    }
    else
    {
        SBL_log(SBL_LOG_MAX, "Skipping Sciclient_procBootSetProcessorCfg for ProcId 0x%x, EntryPoint 0x%x...\n", proc_set_config_req.processor_id, proc_set_config_req.bootvector_lo);
    }
    /* Power down and then power up each core*/
    switch (core_id)
    {
        case MCU1_CPU1_ID:
            /* Display profile logs */
            SBL_printProfileLog();

            if (pAppEntry->CpuEntryPoint[core_id] <  SBL_INVALID_ENTRY_ADDR)
            {
                /* Skip copy if R5 app entry point is already 0 */
                if (pAppEntry->CpuEntryPoint[core_id])
                {
                    SBL_log(SBL_LOG_MAX, "Copying first 128 byptes from app to MCU ATCM @ 0x%x for core %d\n", SblAtcmAddr[core_id - MCU1_CPU0_ID], core_id);
                    memcpy(((void *)(SblAtcmAddr[core_id - MCU1_CPU0_ID])), (void *)(pAppEntry->CpuEntryPoint[core_id]), 128);
                }
            }

#ifdef SBL_SKIP_MCU_RESET
            /* Release the CPU and branch to app */
            status = Sciclient_procBootReleaseProcessor(sblSlaveCoreInfoPtr->tisci_proc_id, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...FAILED \n", sblSlaveCoreInfoPtr->tisci_proc_id);
                SblErrLoop(__FILE__, __LINE__);
            }

            SBL_log(SBL_LOG_MAX, "Starting app, branching to 0x0 \n");
            /* Branch to start of ATCM */
            ((void(*)(void))0x0)();
#else
            SBL_log(SBL_LOG_MAX, "Sciclient_procBootRequestProcessor, ProcId 0x%x... \n", SBL_PROC_ID_MCU1_CPU0);
            status = Sciclient_procBootRequestProcessor(SBL_PROC_ID_MCU1_CPU0, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
            SBL_log(SBL_LOG_ERR, "Sciclient_procBootRequestProcessorProcId 0x%x...FAILED \n", SBL_PROC_ID_MCU1_CPU0);
            SblErrLoop(__FILE__, __LINE__);
            }

            /* Setting up DMSC to wait for WFI */
            SBL_log(SBL_LOG_MAX, "Sciclient_procBootWaitProcessorState, ProcId 0x%x... \n", SBL_PROC_ID_MCU1_CPU0);
            status = Sciclient_procBootWaitProcessorState(SBL_PROC_ID_MCU1_CPU0, 1, 1, 0, 3, 0, 0, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootWaitProcessorState...FAILED \n");
                SblErrLoop(__FILE__, __LINE__);
            }

            /* Power down core running SBL */
            Sciclient_pmSetModuleState(SBL_DEV_ID_MCU1_CPU0, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, 0, SCICLIENT_SERVICE_WAIT_FOREVER);

            /* Both cores halted at this point. Now un-halt them as needed */
            Sciclient_procBootSetSequenceCtrl(SBL_PROC_ID_MCU1_CPU0, 0, TISCI_MSG_VAL_PROC_BOOT_CTRL_FLAG_R5_CORE_HALT, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (pAppEntry->CpuEntryPoint[core_id] <  SBL_INVALID_ENTRY_ADDR)
            {
                Sciclient_procBootSetSequenceCtrl(SBL_PROC_ID_MCU1_CPU1, 0, TISCI_MSG_VAL_PROC_BOOT_CTRL_FLAG_R5_CORE_HALT, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            }

            /* Notifying SYSFW that the SBL is relinquishing the MCU cluster running the SBL */
            status = Sciclient_procBootReleaseProcessor(SBL_PROC_ID_MCU1_CPU0, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            status = Sciclient_procBootReleaseProcessor(SBL_PROC_ID_MCU1_CPU1, 0, SCICLIENT_SERVICE_WAIT_FOREVER);

            /* Power up cores as needed */
            Sciclient_pmSetModuleState(SBL_DEV_ID_MCU1_CPU0, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (pAppEntry->CpuEntryPoint[core_id] <  SBL_INVALID_ENTRY_ADDR)
            {
                Sciclient_pmSetModuleState(SBL_DEV_ID_MCU1_CPU1, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
                Sciclient_pmSetModuleState(SBL_DEV_ID_MCU1_CPU1, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            /* Execute a WFI */
            asm volatile ("    wfi");

#endif
            break;

        case MCU1_CPU0_ID:
            /* Skip copy if R5 app entry point is already 0 */
            if (pAppEntry->CpuEntryPoint[core_id])
            {
                SBL_log(SBL_LOG_MAX, "Copying first 128 byptes from app to MCU ATCM @ 0x%x for core %d\n", SblAtcmAddr[core_id - MCU1_CPU0_ID], core_id);
                memcpy(((void *)(SblAtcmAddr[core_id - MCU1_CPU0_ID])), (void *)(proc_set_config_req.bootvector_lo), 128);
            }
            SBL_log(SBL_LOG_MAX, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...\n", sblSlaveCoreInfoPtr->tisci_proc_id);
            status = Sciclient_procBootReleaseProcessor(sblSlaveCoreInfoPtr->tisci_proc_id, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...FAILED \n", sblSlaveCoreInfoPtr->tisci_proc_id);
                SblErrLoop(__FILE__, __LINE__);
            }
            break;
        case MCU2_CPU0_ID:
        case MCU2_CPU1_ID:
        case MCU3_CPU0_ID:
        case MCU3_CPU1_ID:
            if (pAppEntry->CpuEntryPoint[core_id] <  SBL_INVALID_ENTRY_ADDR)
            {
                /* Skip copy if R5 app entry point is already 0 */
                if (pAppEntry->CpuEntryPoint[core_id])
                {
                    SBL_log(SBL_LOG_MAX, "Copying first 128 byptes from app to MCU ATCM @ 0x%x for core %d\n", SblAtcmAddr[core_id - MCU1_CPU0_ID], core_id);
                    memcpy(((void *)(SblAtcmAddr[core_id - MCU1_CPU0_ID])), (void *)(proc_set_config_req.bootvector_lo), 128);
                }
                SBL_log(SBL_LOG_MAX, "Clearing HALT for ProcId 0x%x...\n", sblSlaveCoreInfoPtr->tisci_proc_id);
                status =  Sciclient_procBootSetSequenceCtrl(sblSlaveCoreInfoPtr->tisci_proc_id, 0, TISCI_MSG_VAL_PROC_BOOT_CTRL_FLAG_R5_CORE_HALT, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
                if (status != CSL_PASS)
                {
                    SBL_log(SBL_LOG_ERR, "Sciclient_procBootSetSequenceCtrl...FAILED \n");
                    SblErrLoop(__FILE__, __LINE__);
                }
                SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState On, DevId 0x%x... \n", sblSlaveCoreInfoPtr->tisci_dev_id);
                Sciclient_pmSetModuleState(sblSlaveCoreInfoPtr->tisci_dev_id, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            SBL_log(SBL_LOG_MAX, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...\n", sblSlaveCoreInfoPtr->tisci_proc_id);
            status = Sciclient_procBootReleaseProcessor(sblSlaveCoreInfoPtr->tisci_proc_id, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...FAILED \n", sblSlaveCoreInfoPtr->tisci_proc_id);
                SblErrLoop(__FILE__, __LINE__);
            }
            SBL_ADD_PROFILE_POINT;
            break;
        default:
            SBL_log(SBL_LOG_MAX, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...\n", sblSlaveCoreInfoPtr->tisci_proc_id);
            status = Sciclient_procBootReleaseProcessor(sblSlaveCoreInfoPtr->tisci_proc_id, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                SBL_log(SBL_LOG_ERR, "Sciclient_procBootReleaseProcessor, ProcId 0x%x...FAILED \n", sblSlaveCoreInfoPtr->tisci_proc_id);
                SblErrLoop(__FILE__, __LINE__);
            }
            SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState Off, DevId 0x%x... \n", sblSlaveCoreInfoPtr->tisci_dev_id);
            Sciclient_pmSetModuleState(sblSlaveCoreInfoPtr->tisci_dev_id, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            SBL_log(SBL_LOG_MAX, "Sciclient_pmSetModuleState On, DevId 0x%x... \n", sblSlaveCoreInfoPtr->tisci_dev_id);
            Sciclient_pmSetModuleState(sblSlaveCoreInfoPtr->tisci_dev_id, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            SBL_ADD_PROFILE_POINT;
            break;
    }
}
