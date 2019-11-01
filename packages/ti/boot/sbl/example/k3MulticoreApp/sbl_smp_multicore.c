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

#include <string.h>
#include <ti/board/board.h>
#include <ti/drv/uart/UART_stdio.h>
#include "sbl_slave_core_boot.h"
#include "sbl_soc_cfg.h"
#include "sbl_soc.h"
#include <ti/csl/arch/csl_arch.h>
#include <ti/drv/sciclient/sciclient.h>

#define MPU_POKE_MEM_ADDR ((volatile int *)0x7002FFFC)

#if defined (SOC_AM65XX)
#define SBL_SMP_TEST_NUM_CORES 4
#endif

#if defined (SOC_J721E)
#define SBL_SMP_TEST_NUM_CORES 4
#endif

#ifdef BUILD_MPU
int main()
{
    CSL_a53v8InvalidateDcacheMvaPoC((uint64_t)MPU_POKE_MEM_ADDR);
    *(MPU_POKE_MEM_ADDR) += 1;
    CSL_a53v8CleanInvalidateDcacheMvaPoC((uint64_t)MPU_POKE_MEM_ADDR);
    CSL_archMemoryFence();
    asm volatile("	wfi");
    return 0;
}
#endif

#if defined (BUILD_MCU2_0)|| defined (BUILD_MCU3_0)
int main()
{

    *(MPU_POKE_MEM_ADDR) += 1;
    CSL_armR5CacheWbInv((const void *)MPU_POKE_MEM_ADDR, sizeof (int));
    asm volatile("	wfi");
    return 0;
}
#endif

#if defined (BUILD_MCU1_0)
static unsigned int getCoreEntry(int32_t proc_id)
{
    int32_t status = CSL_EFAIL;
    struct tisci_msg_proc_get_status_resp cpuStatus;

    if (proc_id == 0xBAD00000)
    {
        return SBL_INVALID_ENTRY_ADDR;
    }

    status = Sciclient_procBootRequestProcessor(proc_id, SCICLIENT_SERVICE_WAIT_FOREVER);
    if (status != CSL_PASS)
    {
        UART_printf("Sciclient_procBootRequestProcessor...FAILED\r\n");
        for(;;);
    }

    status = Sciclient_procBootGetProcessorState(proc_id, &cpuStatus, SCICLIENT_SERVICE_WAIT_FOREVER);
    if (status != CSL_PASS)
    {
        UART_printf("Sciclient_procBootGetProcessorState...FAILED\r\n");
        for(;;);
    }

    /* Halt only R5 cores */
    if (proc_id < SBL_PROC_ID_MPU1_CPU0)
    {
        status = Sciclient_procBootSetSequenceCtrl(proc_id, TISCI_MSG_VAL_PROC_BOOT_CTRL_FLAG_R5_CORE_HALT, 0, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
        if (status != CSL_PASS)
        {
            UART_printf("Sciclient_procBootSetSequenceCtrl...FAILED\r\n");
            for(;;);
        }
    }

    status = Sciclient_procBootReleaseProcessor(proc_id, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
    if (status != CSL_PASS)
    {
        UART_printf("Sciclient_procBootReleaseProcessor...FAILED \n");
        for(;;);
    }

    return cpuStatus.bootvector_lo;
}

int main()
{
    sblEntryPoint_t sblSmpTestEntry = {SBL_INVALID_ENTRY_ADDR};
    cpu_core_id_t core_id;
    int32_t status = CSL_EFAIL;
    int32_t proc_id_list [] =
    {
    SBL_PROC_ID_MPU1_CPU0,
    SBL_PROC_ID_MPU1_CPU1,
    SBL_PROC_ID_MPU2_CPU0,
    SBL_PROC_ID_MPU2_CPU1,
    SBL_PROC_ID_MCU1_CPU0,
    SBL_PROC_ID_MCU1_CPU1,
    SBL_PROC_ID_MCU2_CPU0,
    SBL_PROC_ID_MCU2_CPU1,
    SBL_PROC_ID_MCU3_CPU0,
    SBL_PROC_ID_MCU3_CPU1
    };

    /* Board Init UART for logging. */
    Board_init(BOARD_INIT_UART_STDIO);
    UART_printf("MPU SMP boot test\r\n");

    /* Get MPU SMP entry points */
    for (core_id = MPU1_CPU0_ID; core_id < MPU2_CPU1_ID; core_id+= 2)
    {
        sblSmpTestEntry.CpuEntryPoint[core_id] = getCoreEntry(proc_id_list[core_id]);
        sblSmpTestEntry.CpuEntryPoint[core_id + 1] = sblSmpTestEntry.CpuEntryPoint[core_id];
        if (sblSmpTestEntry.CpuEntryPoint[core_id] < SBL_INVALID_ENTRY_ADDR)
        {
            UART_printf("Cores %d & %d will boot from 0x%x\r\n", core_id, core_id + 1, sblSmpTestEntry.CpuEntryPoint[core_id]);
        }
    }

    /* Get MCU lock step entry points */
    for (core_id = MCU2_CPU0_ID; core_id < MCU3_CPU1_ID; core_id += 2)
    {
        sblSmpTestEntry.CpuEntryPoint[core_id] = getCoreEntry(proc_id_list[core_id]);
        sblSmpTestEntry.CpuEntryPoint[core_id + 1] = SBL_MCU_LOCKSTEP_ADDR ;
        if (sblSmpTestEntry.CpuEntryPoint[core_id] < SBL_INVALID_ENTRY_ADDR)
        {
            UART_printf("Cores %d & %d will boot in lockstep from 0x%x\r\n", core_id, core_id + 1, sblSmpTestEntry.CpuEntryPoint[core_id]);
        }
    }

    /* Reset the boot flags */
    *(MPU_POKE_MEM_ADDR) = 0;
    CSL_armR5CacheWbInv((const void *)MPU_POKE_MEM_ADDR, sizeof (int));

    /* SMP apps is already loaded, simply reset the cores to run it */
    UART_printf("Resetting all ARM cores now...\r\n");
    for (core_id = MPU1_CPU0_ID; core_id <= MPU2_CPU1_ID; core_id ++)
    {
        /* Try booting all cores */
        if (proc_id_list[core_id] != 0xBAD00000)
        {
            SBL_SlaveCoreBoot(core_id, NULL, &sblSmpTestEntry);
            UART_printf("No of Cortex-A core(s) running: ");
            CSL_armR5CacheInv((const void *)MPU_POKE_MEM_ADDR, sizeof (int));
            UART_printf("%d\r\n", *(MPU_POKE_MEM_ADDR));
        }
    }
    for (core_id = MCU2_CPU0_ID; core_id <= MCU3_CPU1_ID; core_id += 2)
    {
        /* Try booting all cores */
        if (proc_id_list[core_id] != 0xBAD00000)
        {
            const int32_t dev_id_list [] =
            {
            SBL_DEV_ID_MPU1_CPU0,
            SBL_DEV_ID_MPU1_CPU1,
            SBL_DEV_ID_MPU2_CPU0,
            SBL_DEV_ID_MPU2_CPU1,
            SBL_DEV_ID_MCU1_CPU0,
            SBL_DEV_ID_MCU1_CPU1,
            SBL_DEV_ID_MCU2_CPU0,
            SBL_DEV_ID_MCU2_CPU1,
            SBL_DEV_ID_MCU3_CPU0,
            SBL_DEV_ID_MCU3_CPU1
            };
            const uint32_t SblAtcmAddr[] =
            {
            SBL_MCU_ATCM_BASE,
            SBL_MCU1_CPU1_ATCM_BASE_ADDR_SOC,
            SBL_MCU2_CPU0_ATCM_BASE_ADDR_SOC,
            SBL_MCU2_CPU1_ATCM_BASE_ADDR_SOC,
            SBL_MCU3_CPU0_ATCM_BASE_ADDR_SOC,
            SBL_MCU3_CPU1_ATCM_BASE_ADDR_SOC
            };
            const uint32_t SblBtcmAddr[] =
            {
            SBL_MCU_BTCM_BASE,
            SBL_MCU1_CPU1_BTCM_BASE_ADDR_SOC,
            SBL_MCU2_CPU0_BTCM_BASE_ADDR_SOC,
            SBL_MCU2_CPU1_BTCM_BASE_ADDR_SOC,
            SBL_MCU3_CPU0_BTCM_BASE_ADDR_SOC,
            SBL_MCU3_CPU1_BTCM_BASE_ADDR_SOC
            };

            /* Prepare the cores for going down. TCMs */
            /* will be lost, so re-init restore them */
            status = Sciclient_pmSetModuleState(dev_id_list[core_id], TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                UART_printf("Sciclient_pmSetModuleState OFF...FAILED\r\n");
                for(;;);
            }
            /* Cores are halted, safe to turn on. */
            status = Sciclient_pmSetModuleState(dev_id_list[core_id], TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status != CSL_PASS)
            {
                UART_printf("Sciclient_pmSetModuleState ON...FAILED\r\n");
                for(;;);
            }
            /* This testcase does not load code in TCMs, so re-init the TCMs */
            memset(((void *)(SblAtcmAddr[core_id - MCU1_CPU0_ID])), 0xFF, 0x8000);
            memset(((void *)(SblBtcmAddr[core_id - MCU1_CPU0_ID])), 0xFF, 0x8000);

            /* Restart the core */
            SBL_SlaveCoreBoot(core_id, NULL, &sblSmpTestEntry);
            UART_printf("No of Cortex-R core(s) running: ");
            CSL_armR5CacheInv((const void *)MPU_POKE_MEM_ADDR, sizeof (int));
            UART_printf("%d\r\n", *(MPU_POKE_MEM_ADDR));
        }
    }
    
    /* Check if MPUs have run in AMP mode */
    if (*(MPU_POKE_MEM_ADDR) != SBL_SMP_TEST_NUM_CORES)
        UART_printf("Some tests have failed\r\n");
    else
        UART_printf("All tests have passed\r\n");

    return 0;
}
#endif
