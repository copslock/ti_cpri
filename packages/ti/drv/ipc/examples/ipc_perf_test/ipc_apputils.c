/*
 *  Copyright (c) Texas Instruments Incorporated 2019
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file ipc_apputils.c
 *
 *  \brief Common IPC application utility used in all IPC example.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drv/ipc/ipc.h>
#include <ti/drv/sciclient/sciclient.h>
#include "ipc_apputils.h"
#if defined (__C7100__)
#include <ti/csl/csl_clec.h>
#include <ti/csl/arch/csl_arch.h>
#endif
#include <ti/osal/osal.h>
#include <ti/drv/ipc/include/ipc_config.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Ipc_appUtilsCacheWb(const void *addr, int32_t size)
{
    uint32_t    isCacheCoherent = Ipc_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        CacheP_wb(addr, size);
    }

    return;
}

void Ipc_appUtilsCacheInv(const void * addr, int32_t size)
{
    uint32_t    isCacheCoherent = Ipc_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        CacheP_Inv(addr, size);
    }

    return;
}

void Ipc_appUtilsCacheWbInv(const void * addr, int32_t size)
{
    uint32_t    isCacheCoherent = Ipc_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        CacheP_wbInv(addr, size);
    }

    return;
}

uint64_t Ipc_appVirtToPhyFxn(const void *virtAddr, uint32_t chNum, void *appData)
{
    uint64_t    phyAddr;

    phyAddr = (uint64_t) virtAddr;
#if defined (BUILD_C66X_1) || defined (BUILD_C66X_2)
    /* Convert local L2RAM address to global space */
    if((phyAddr >= CSL_C66_COREPAC_L2_BASE) &&
       (phyAddr < (CSL_C66_COREPAC_L2_BASE + CSL_C66_COREPAC_L2_SIZE)))
    {
#if defined (BUILD_C66X_1)
        phyAddr -= CSL_C66_COREPAC_L2_BASE;
        phyAddr += CSL_C66SS0_C66_SDMA_L2SRAM_0_BASE;
#endif
#if defined (BUILD_C66X_2)
        phyAddr -= CSL_C66_COREPAC_L2_BASE;
        phyAddr += CSL_C66SS1_C66_SDMA_L2SRAM_0_BASE;
#endif
    }
#endif

    return (phyAddr);
}

void *Ipc_appPhyToVirtFxn(uint64_t phyAddr, uint32_t chNum, void *appData)
{
    void       *virtAddr;

#if defined (__aarch64__) || defined (__C7100__)
    virtAddr = (void *) phyAddr;
#else
    uint32_t temp;

    /* Convert global L2RAM address to local space */
#if defined (BUILD_C66X_1)
    if((phyAddr >= CSL_C66SS0_C66_SDMA_L2SRAM_0_BASE) &&
       (phyAddr < (CSL_C66SS0_C66_SDMA_L2SRAM_0_BASE + CSL_C66_COREPAC_L2_SIZE)))
    {
        phyAddr -= CSL_C66SS0_C66_SDMA_L2SRAM_0_BASE;
        phyAddr += CSL_C66_COREPAC_L2_BASE;
    }
#endif
#if defined (BUILD_C66X_2)
    if((phyAddr >= CSL_C66SS1_C66_SDMA_L2SRAM_0_BASE) &&
       (phyAddr < (CSL_C66SS1_C66_SDMA_L2SRAM_0_BASE + CSL_C66_COREPAC_L2_SIZE)))
    {
        phyAddr -= CSL_C66SS1_C66_SDMA_L2SRAM_0_BASE;
        phyAddr += CSL_C66_COREPAC_L2_BASE;
    }
#endif

    /* R5/C66x is 32-bit; need to truncate to avoid void * typecast error */
    temp = (uint32_t) phyAddr;
    virtAddr = (void *) temp;
#endif

    return (virtAddr);
}

uint32_t Ipc_appIsPrintSupported(void)
{
    uint32_t retVal = TRUE;

    /* Semi hosting not supported for MPU on Simulator */
#if (defined (BUILD_MPU) && defined (SIMULATOR))
    retVal = FALSE;
#endif

    /* Printf doesn't work for MPU when run from SBL with no CCS connection
     * There is no flag to detect SBL or CCS mode. Hence disable the print
     * for MPU unconditionally */
#if defined (BUILD_MPU)
    retVal = FALSE;
#endif

    return (retVal);
}

void Ipc_appC66xIntrConfig(void)
{
#if defined (_TMS320C6X)
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    /* On C66x builds we define OS timer tick in the configuration file to
     * trigger event #21 for C66x_1 and #20 for C66x_2. Map
     * DMTimer 0 interrupt to these events through DMSC RM API.
     */
    rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                                      TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id                 = TISCI_DEV_TIMER0;
    rmIrqReq.src_index              = 0U;
#if defined (BUILD_C66X_1)
    rmIrqReq.dst_id                 = TISCI_DEV_C66SS0_CORE0;
    rmIrqReq.dst_host_irq           = 21U;
#endif
#if defined (BUILD_C66X_2)
    rmIrqReq.dst_id                 = TISCI_DEV_C66SS1_CORE0;
    rmIrqReq.dst_host_irq           = 20U;
#endif
    /* Unused params */
    rmIrqReq.global_event           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, IPC_SCICLIENT_TIMEOUT);
#endif

    return;
}

int32_t Ipc_setCpuHz(uint32_t freq)
{
    uint32_t cookie;
    Types_FreqHz cpuHz;
    Types_FreqHz oldCpuHz;

    BIOS_getCpuFreq(&oldCpuHz);

    cookie = HwiP_disable();
    cpuHz.lo = freq;
    cpuHz.hi = 0;
    BIOS_setCpuFreq(&cpuHz);
    Clock_tickStop();
    Clock_tickReconfig();
    Clock_tickStart();
    HwiP_restore(cookie);

    BIOS_getCpuFreq(&cpuHz);

    return 0;
}

uint64_t Ipc_getTimestampFrq(void)
{
    Types_FreqHz frq;
    uint64_t     retFrq;

    Timestamp_getFreq(&frq);
    retFrq = ((uint64_t) frq.hi << 32) | frq.lo;
    return retFrq;
}

uint64_t Ipc_getTimeInUsec(uint64_t frq)
{
    Types_Timestamp64 bios_timestamp64;
    uint64_t cur_ts;

    Timestamp_get64(&bios_timestamp64);
    cur_ts = ((uint64_t) bios_timestamp64.hi << 32) | bios_timestamp64.lo;
    return (cur_ts*1000000u)/frq;
}

void Ipc_setCoreFrq(uint32_t selfId)
{
    uint32_t clkMhz = 2000;
#if defined(SOC_J721E)
    switch(selfId)
    {
        case IPC_MPU1_0:    clkMhz = 2000;    break;
        case IPC_MCU1_0:
        case IPC_MCU1_1:    clkMhz = 1000;    break; 
        case IPC_MCU2_0:
        case IPC_MCU2_1:    clkMhz = 1000;    break;
        case IPC_MCU3_0:
        case IPC_MCU3_1:    clkMhz = 1000;    break;
        case IPC_C66X_1:
        case IPC_C66X_2:    clkMhz = 1350;    break;
        case IPC_C7X_1:     clkMhz = 1000;    break;

        default:            clkMhz = 2000;    break;
    }
#else
    switch(selfId)
    {
        case IPC_MPU1_0:    clkMhz = 2000;    break;
        case IPC_MCU1_0:
        case IPC_MCU1_1:    clkMhz = 1000;    break;                            

        default:            clkMhz = 2000;    break;
    }

#endif

    /* convert to Hz before setting */
    Ipc_setCpuHz(clkMhz*1000*1000);
}


void sysIdleLoop(void)
{
#if defined(BUILD_C66X_1) || defined(BUILD_C66X_2)
    __asm(" IDLE");
#elif defined(BUILD_C7X_1)
    __asm(" IDLE");
#elif defined(BUILD_MCU1_0) || defined(BUILD_MCU1_1) || \
      defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1) || \
      defined(BUILD_MCU3_0) || defined(BUILD_MCU3_1)
   asm(" wfi");
#endif
}


