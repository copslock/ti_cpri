/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 *  \file ipc_utils.c
 *
 *  \brief IPC  utility function used for common setup
 *  
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/drv/ipc/ipc.h>

void SetManualBreak()
{
#if  defined (__aarch64__)
    volatile uint32_t x = 0;
    while(x)
        ;
#endif
}

#if defined(BUILD_MCU1_0) || defined(BUILD_MCU1_1) || defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1) || defined(BUILD_MCU3_0) || defined(BUILD_MCU3_1)
void sysIdleLoop(void)
{
   asm(" wfi");
}

#endif

#ifdef BUILD_C7X_1
#include <ti/csl/csl_clec.h>
#include <ti/csl/arch/csl_arch.h>

void sysIdleLoop(void)
{
   __asm(" IDLE");
}

void Ipc_appC7xPreInit(void)
{
    CSL_ClecEventConfig cfgClec;
    CSL_CLEC_EVTRegs   *clecBaseAddr = (CSL_CLEC_EVTRegs*) CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE;
    uint32_t            i, maxInputs = 2048U;

    /* make secure claim bit to FALSE so that after we switch to non-secure mode
     * we can program the CLEC MMRs
     */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = FALSE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_DISABLE;
    cfgClec.extEvtNum         = 0U;
    cfgClec.c7xEvtNum         = 0U;
    for(i = 0U; i < maxInputs; i++)
    {
        CSL_clecConfigEvent(clecBaseAddr, i, &cfgClec);
    }

    /* Switch now */
    CSL_c7xSecSupv2NonSecSupv();

    return;
}

#include <ti/sysbios/family/c7x/Hwi.h>
#include <ti/csl/csl_clec.h>
void C7x_ConfigureTimerOutput()
{
    CSL_ClecEventConfig   cfgClec;
    CSL_CLEC_EVTRegs     *clecBaseAddr = (CSL_CLEC_EVTRegs*)C7X_CLEC_BASE_ADDR;

    uint32_t input         = 1248; /* Used for Timer Interrupt */
    uint32_t corepackEvent = 14;

    /* Configure CLEC */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = TRUE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_CPU_ALL;
    cfgClec.extEvtNum         = 0;
    cfgClec.c7xEvtNum         = corepackEvent;
    CSL_clecConfigEvent(clecBaseAddr, input, &cfgClec);
}

#endif

#if defined(BUILD_C66X_1) || defined(BUILD_C66X_2)
/* To set C66 timer interrupts on J7ES VLAB */
void C66xTimerInterruptInit(void)
{
    /*
     * The C66 INTR_ROUTER on J7ES VLAB is not directly addressable by the
     * C66 core itself, so we need to use the RAT.
     *
     * RAT is hard-coded at 0x07ff0000 for J7ES C66.  Choose an arbitrary
     * RAT entry (2nd entry) at offset 0x30.  Region entries start at 0x20.
     */
    volatile int *RAT = (volatile int *)0x07ff0030;
    /* Choose an arbitrary virtual address for intr_router RAT mapping */
    volatile int *intr_router = (volatile int *)0x18000000;

    /* program virtual address to REGION_BASE */
    RAT[1] = (int)intr_router;
    /* program C66_0 INTR_ROUTER physical addr to REGION_TRANS_L */
#ifdef BUILD_C66X_1
    RAT[2] = 0x00ac0000;
#endif
#ifdef BUILD_C66X_2
    RAT[2] = 0x00ad0000;
#endif
    /* enable region and set size to 512 B */
    RAT[0] = 0x80000009;

    /*
     * intr_router[12] corresponds to output event #21, which is what we
     * set eventId to in .cfg file.
     *     - bit 16 enables the entry
     *     - lower bits define input event (#0 for dmtimer #0)
     */
#ifdef BUILD_C66X_1
    intr_router[12] = 0x00010001;
#endif
#ifdef BUILD_C66X_2
    intr_router[11] = 0x00010001;
#endif
}

void sysIdleLoop(void)
{
   __asm(" IDLE"); 
}

#endif

#if defined (__aarch64__)
#include <ti/sysbios/family/arm/v8a/Mmu.h>

volatile int32_t emuwait_mmu = 1;
void InitMmu(void)
{
    Bool            retVal;
    Mmu_MapAttrs    attrs;

    Mmu_initMapAttrs(&attrs);
    attrs.attrIndx = 0;

    retVal = Mmu_map(0x00000000, 0x00000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x0100000, 0x0100000, 0x00900000, &attrs); /* PLL_MMR_CFG registers regs       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x00400000, 0x00400000, 0x00001000, &attrs); /* PSC0          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

#if defined(SOC_J721E)
    retVal = Mmu_map(0x01800000, 0x01800000, 0x00200000, &attrs); /* gicv3       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

	/* SCICLIENT UDMA */
	retVal = Mmu_map(0x20000000ul, 0x20000000ul, 0x10000000ul, &attrs);
    if (retVal == FALSE)
    {
         goto mmu_exit;
    }
#else
    retVal = Mmu_map(0x01800000, 0x01800000, 0x00100000, &attrs); /* gicv3       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
#endif

    retVal = Mmu_map(0x02400000, 0x02400000, 0x000c0000, &attrs); /* dmtimer     */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x02800000, 0x02800000, 0x00040000, &attrs); /* uart        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x02000000, 0x02000000, 0x00100000, &attrs); /* I2C            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x02100000, 0x02100000, 0x00080000, &attrs); /* McSPI          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x40f00000, 0x40f00000, 0x00020000, &attrs); /* MCU MMR0 CFG   */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
    retVal = Mmu_map(0x40d00000, 0x40d00000, 0x00002000, &attrs); /* PLL0 CFG       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x43000000, 0x43000000, 0x00020000, &attrs); /* WKUP MMR0 cfg  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x02C40000, 0x02C40000, 0x00100000, &attrs); /* pinmux ctrl    */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x2A430000, 0x2A430000, 0x00001000, &attrs); /* ctrcontrol0 */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x030800000, 0x030800000, 0xC000000, &attrs); /* navss        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x28380000, 0x28380000, 0xC000000, &attrs); /* MCU NAVSS */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x6D000000, 0x6D000000, 0x1000000, &attrs); /* DRU */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x42000000, 0x42000000, 0x00001000, &attrs); /* PSC WKUP*/
    if (retVal == FALSE)
    {
         goto mmu_exit;
    }

    /*
     * DDR range 0xA0000000 - 0xAA000000 : Used as RAM by multiple
     * remote cores, no need to mmp_map this range.
     * IPC VRing Buffer - uncached
     * */
    attrs.attrIndx = 4;
#ifdef SOC_AM65XX
    retVal = Mmu_map(0xA2000000, 0xA2000000, 0x00200000, &attrs);
#else
    retVal = Mmu_map(0xAA000000, 0xAA000000, 0x02000000, &attrs);
#endif
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    attrs.attrIndx = 7;
    retVal = Mmu_map(0x80000000, 0x80000000, 0x20000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x70000000, 0x70000000, 0x04000000, &attrs); /* msmc        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

mmu_exit:
    if(retVal == FALSE)
    {
         System_printf("Mmu_map returned error %d",retVal);
         while(emuwait_mmu);
    }

    return;
}
#endif

