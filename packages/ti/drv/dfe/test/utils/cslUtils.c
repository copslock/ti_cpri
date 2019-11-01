/****************************************************************************\
 *           (C) Copyright 2013, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#ifdef __ARMv7
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#else
#include <c6x.h>
#ifdef USESYSBIOS
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c66/Cache.h>
#endif
#endif

#include <ti/csl/csl.h>
#include <ti/csl/cslver.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_serdes.h>
#include <ti/csl/csl_serdes_iqn.h>
#include <ti/csl/csl_serdes_dfe.h>
#include <ti/csl/csl_serdes_restore_default.h>

#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_device_interrupt.h>
#include <ti/csl/csl_qm_queue.h>

#include <ti/drv/dfe/dfe_drv.h>
#define __CSLUTILS_C
#include "cslUtils.h"


/* Spin in a delay loop */
void
UTILS_waitForHw(
   uint32_t delay
)
{
    volatile uint32_t i, n;

    n = 0;
    for (i = 0; i < delay; i++)
    {
        n = n + 1;
    }
}

#ifdef _TMS320C6X

/* Cache operations */

void UTILS_setCache()
{
    uint32_t  key;

    // Disable Interrupts
    key = _disable_interrupts();

#ifndef USESYSBIOS

    //  Cleanup the prefetch buffer.
    CSL_XMC_invalidatePrefetchBuffer();

    // Change cache sizes
    CACHE_setL1DSize (CACHE_L1_32KCACHE);
    CACHE_setL1PSize (CACHE_L1_32KCACHE);

#else
    Cache_Size size;

    size.l1pSize = Cache_L1Size_32K;
    size.l1dSize = Cache_L1Size_32K;
    size.l2Size  = Cache_L2Size_0K;
    Cache_setSize(&size);

    ti_sysbios_family_c66_Cache_invPrefetchBuffer();
    Cache_setMar((xdc_Ptr*)0x0c000000,0x00200000,Cache_Mar_ENABLE);
#endif

    // Reenable Interrupts.
    _restore_interrupts(key);

}

void UTILS_cacheInvalidate(void* ptr, uint32_t size)
{
    uint32_t  key;

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0xA0000000) == 0xA0000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYSBIOS
        printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        System_printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0x80000000) == 0x80000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYSBIOS
        printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        System_printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if MSMC address is 64 byte aligned */
    if ((((int)ptr & 0x0C000000) == 0x0C000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYSBIOS
        printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        System_printf("UTILS_cacheInvalidate(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    // Disable Interrupts
    key = _disable_interrupts();

#ifndef USESYSBIOS
    //  Cleanup the prefetch buffer also.
    CSL_XMC_invalidatePrefetchBuffer();
    // Invalidate the cache.
    CACHE_invL1d(ptr, size, CACHE_FENCE_WAIT);
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
#else
    ti_sysbios_family_c66_Cache_invPrefetchBuffer();
    Cache_inv((void *)ptr, size, Cache_Type_L1D, 1);
#endif

    // Reenable Interrupts.
    _restore_interrupts(key);
}

void UTILS_cacheWriteBack(void* ptr, uint32_t size)
{
    uint32_t  key;

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0xA0000000) == 0xA0000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYSBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        System_printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0x80000000) == 0x80000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if MSMC address is 64 byte aligned */
    if ((((int)ptr & 0x0C000000) == 0x0C000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    // Disable Interrupts
    key = _disable_interrupts();
#ifndef USESYSBIOS
    // Writeback the contents of the cache.
    CACHE_wbL1d(ptr, size, CACHE_FENCE_WAIT);
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
#else
    // Writeback the contents of the cache.
    Cache_wb((void *)ptr, size, Cache_Type_L1D, 1);
#endif
    // Reenable Interrupts.
    _restore_interrupts(key);
}

void UTILS_cacheWriteBackInvalidate(void* ptr, uint32_t size)
{
    uint32_t  key;

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0xA0000000) == 0xA0000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if DDR3 address is 64 byte aligned */
    if ((((int)ptr & 0x80000000) == 0x80000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    /* Check if MSMC address is 64 byte aligned */
    if ((((int)ptr & 0x0C000000) == 0x0C000000) && (((int)ptr % 0x40) != 0)){
#ifndef USESYBIOS
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#else
        printf("UTILS_cacheWriteBack(): DISALIGNMENT OF CACHE AT ADDRESS: 0x%.8X\n", ptr);
#endif
    }

    // Disable Interrupts
    key = _disable_interrupts();
#ifndef USESYSBIOS
    // Writeback and Invalidate the contents of the cache.
    CACHE_wbInvL1d(ptr, size, CACHE_FENCE_WAIT);
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
    asm (" nop  4");
#else
    // Writeback and Invalidate the contents of the cache.
    Cache_wbInv((void *)ptr, size, Cache_Type_L1D, 1);
#endif
    // Reenable Interrupts.
    _restore_interrupts(key);
}
#endif // _TMS320C6X

#ifdef _VIRTUAL_ADDR_SUPPORT
static int dev_mem_fd;
#endif

uint32_t iqn2_mmap(uint32_t addr, uint32_t size)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
        uint32_t virt_addr;
        uint32_t page_size;
        page_size = sysconf(_SC_PAGE_SIZE);
        if (size%page_size)
        {
                printf("Size does not align with page size. Size given: %d\n", size);
                return 0;
        }
        if ((uint32_t)addr % page_size)
        {
                printf("Address does not align with page size. Address given: 0x%08x\n", (uint32_t) addr);
                return 0;
        }
        virt_addr = (uint32_t) mmap(0, size, (PROT_READ|PROT_WRITE), MAP_SHARED, dev_mem_fd, (off_t)addr);
        if (virt_addr == -1)
        {
                printf("mmap failed!\n");
                return 0;
        }
        return virt_addr;
#else
    return addr;
#endif
}

extern uint32_t dfe_cfg_base;
uint32_t dfe_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
        printf("Failed to open /dev/mem \n");
        return -1;
    }
    dfe_cfg_base = iqn2_mmap((uint32_t)(dfe_cfg_base), 0x2000000);
    return 0;
#else
    return 0;
#endif
}

uint32_t dfe_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap((uint32_t*)dfe_cfg_base, 0x2000000);
    close(dev_mem_fd);
#endif
    return 0;
}

extern uint32_t serdes_cfg0_base;
uint32_t serdesCfg0_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
        printf("Failed to open /dev/mem \n");
        return -1;
    }
    serdes_cfg0_base = iqn2_mmap((uint32_t)(serdes_cfg0_base), 0x2000);
    return 0;
#else
    return 0;
#endif
}

uint32_t serdesCfg0_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap((uint32_t*)serdes_cfg0_base, 0x2000);
    close(dev_mem_fd);
#endif
    return 0;
}

extern uint32_t serdes_cfg1_base;
uint32_t serdesCfg1_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
        printf("Failed to open /dev/mem \n");
        return -1;
    }
    serdes_cfg1_base = iqn2_mmap((uint32_t)(serdes_cfg1_base), 0x2000);
    return 0;
#else
    return 0;
#endif
}

uint32_t serdesCfg1_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap((uint32_t*)serdes_cfg1_base, 0x2000);
    close(dev_mem_fd);
#endif
    return 0;
}

extern uint32_t boot_cfg_base;
uint32_t bootCfg_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
        printf("Failed to open /dev/mem \n");
        return -1;
    }
    boot_cfg_base = iqn2_mmap((uint32_t)(boot_cfg_base), 0x1000);
	
    return 0;
#else
    return 0;
#endif
}

uint32_t bootCfg_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap((uint32_t*)boot_cfg_base, 0x1000);
    close(dev_mem_fd);
#endif
    return 0;
}

void pscEnable(uint32_t pwrDmnNum, uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = iqn2_mmap(CSL_PSC_REGS, 0x10000);
    if (CSL_FEXT(((CSL_PscRegs *) mem_base_PSC)->PDSTAT[pwrDmnNum], PSC_PDSTAT_STATE) != PSC_PDSTATE_ON)
    {
        /* Enable the domain */
        CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->PDCTL[pwrDmnNum], PSC_PDCTL_NEXT, ON);
    }
    /* Enable MDCTL */
    CSL_FINS (((CSL_PscRegs *) mem_base_PSC)->MDCTL[lpscNum], PSC_MDCTL_NEXT, PSC_MODSTATE_ENABLE);
    /* Apply the domain */
    ((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << pwrDmnNum);
    /* Wait for it to finish */
    while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, pwrDmnNum, pwrDmnNum) == 1);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    if (CSL_PSC_getModuleState (lpscNum) != PSC_MODSTATE_ENABLE) {
        /* Turn on the power domain */
        CSL_PSC_enablePowerDomain (pwrDmnNum);
        /* Enable MDCTL */
        CSL_PSC_setModuleNextState (lpscNum, PSC_MODSTATE_ENABLE);
        /* Apply the domain */
        CSL_PSC_startStateTransition (pwrDmnNum);
        /* Wait for it to finish */
        while (! CSL_PSC_isStateTransitionDone (pwrDmnNum));

        /* Log  PSC status if not ok */
        if (!((CSL_PSC_getPowerDomainState(pwrDmnNum) == PSC_PDSTATE_ON) &&
                (CSL_PSC_getModuleState (lpscNum) == PSC_MODSTATE_ENABLE)))
        {
            printf("Error: failed to turn this power domain on\n");
        }
    }
#endif
} /* iqn2dfePSCSetup */

void pscDisable(uint32_t pwrDmnNum, uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = iqn2_mmap(CSL_PSC_REGS, 0x10000);
    /* Disable MDCTL */
    CSL_FINS (((CSL_PscRegs *) mem_base_PSC)->MDCTL[lpscNum], PSC_MDCTL_NEXT, PSC_MODSTATE_DISABLE);
    /* Apply the domain */
    ((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << pwrDmnNum);
    /* Wait for it to finish */
    while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, pwrDmnNum, pwrDmnNum) == 1);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    /* Disable MDCTL */
    CSL_PSC_setModuleNextState (lpscNum, PSC_MODSTATE_DISABLE);
    /* Apply the domain */
    CSL_PSC_startStateTransition (pwrDmnNum);
    /* Wait for it to finish */
    while (! CSL_PSC_isStateTransitionDone (pwrDmnNum));

#endif
} /* iqn2dfePSCDisable */


void pscPowerOff(uint32_t pwrDmnNum, uint32_t lpscNum)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = iqn2_mmap(CSL_PSC_REGS, 0x10000);
    /* Power Off */
    CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->PDCTL[pwrDmnNum], PSC_PDCTL_NEXT, OFF);
    /* Apply the domain */
    ((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << pwrDmnNum);
    /* Wait for it to finish */
    while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, pwrDmnNum, pwrDmnNum) == 1);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    //Wait for any previous transitions to complete
    while (!CSL_PSC_isStateTransitionDone (pwrDmnNum));
    //Write Switch input into the corresponding PDCTL register
    CSL_PSC_disablePowerDomain (pwrDmnNum);
    //Write PTCMD to start the transition
    CSL_PSC_startStateTransition (pwrDmnNum);
    //Wait for the transition to complete
    while (!CSL_PSC_isStateTransitionDone (pwrDmnNum));
#endif
}

void UTILS_iqn2DfeEnable(uint32_t dfe_enable, uint32_t dpd_enable)
{
    //AID, IQS, PSR, AT, DIO power on
    pscEnable(CSL_PSC_PD_ALWAYSON, CSL_PSC_LPSC_DFE_IQN_SYS);
    // Do not change this enable order
    pscEnable(CSL_PSC_PD_IQN_AIL, CSL_PSC_LPSC_IQN_AIL);
    if (dfe_enable) {
        if (dpd_enable) pscEnable(CSL_PSC_PD_DFE_PD1, CSL_PSC_LPSC_DFE_PD1);
        pscEnable(CSL_PSC_PD_DFE_PD0, CSL_PSC_LPSC_DFE_PD0);
    }
    pscEnable(CSL_PSC_PD_DFE_PD2, CSL_PSC_LPSC_DFE_PD2);
}

void UTILS_iqn2DfeDisable()
{
    pscDisable(CSL_PSC_PD_ALWAYSON, CSL_PSC_LPSC_DFE_IQN_SYS);
    pscPowerOff(CSL_PSC_PD_IQN_AIL, CSL_PSC_LPSC_IQN_AIL);
    pscPowerOff(CSL_PSC_PD_DFE_PD1, CSL_PSC_LPSC_DFE_PD1);
    pscPowerOff(CSL_PSC_PD_DFE_PD0, CSL_PSC_LPSC_DFE_PD0);
    pscPowerOff(CSL_PSC_PD_DFE_PD2, CSL_PSC_LPSC_DFE_PD2);
}


uint32_t serdes_cfg0_base =  CSL_CSISC2_0_SERDES_CFG_REGS; //MMR base address of SerDes config0
uint32_t serdes_cfg1_base =  CSL_CSISC2_1_SERDES_CFG_REGS; //MMR base address of SerDes config1
uint32_t boot_cfg_base    =  CSL_BOOT_CFG_REGS;

#if CSL_VERSION_ID <= (0x02010006)
static void serdesCfg0RestoreDefault(
)
{
	uint32_t i;
	CSL_SerDes_COMLANE_Restore_Default(serdes_cfg0_base);
	for(i=0; i < 4; i++)
	{
	    CSL_SerDes_Lane_Restore_Default(serdes_cfg0_base, i);
	}
	CSL_SerDes_CMU_Restore_Default(serdes_cfg0_base);
}

static void serdesCfg1RestoreDefault(
)
{
    uint32_t i;
    CSL_SerDes_COMLANE_Restore_Default(serdes_cfg1_base);
    for(i=0; i < 4; i++)
    {
        CSL_SerDes_Lane_Restore_Default(serdes_cfg1_base, i);
    }
    CSL_SerDes_CMU_Restore_Default(serdes_cfg1_base);
}

void UTILS_dfeSerdesConfig(
        UTILS_dfeSerDesCfg        *hDfeSerDesCfg,
        CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
)
{
    uint32_t                    retval, i, tcnt;
    CSL_SERDES_LOOPBACK         loopback;
    uint32_t                    serdes_mux_dfe_sel=0, serdes_common_ref_clock=0, numlanescfg0=0, numlanescfg1=0;

    /* Check CSISC2_0_MUXSEL bit */
    if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 26, 26) == 0) serdes_mux_dfe_sel = 1;
    //serdes_mux_dfe_sel = 1;

    /* Check CSISC2_0_CLKCTL bit */
    //if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 27, 27) == 1) serdes_common_ref_clock = 1;
    serdes_common_ref_clock = 1;

    if (serdes_mux_dfe_sel == 0) {
#ifndef USESYSBIOS
        printf("Error: can't configure dfe serdes given CSISC2_0_MUXSEL\n");
#else
        System_printf("Error: can't configure dfe serdes given CSISC2_0_MUXSEL\n");
#endif
    } else if (serdes_common_ref_clock == 0) {
#ifndef USESYSBIOS
        printf("Error: can't configure dfe serdes given both blocks don't use the same ref clock\n");
#else
        System_printf("Error: can't configure dfe serdes given both blocks don't use the same ref clock\n");
#endif
    } else {
        // Call the shutdown for each SerDes and restore default values if SerDes were already in use
        CSL_DFESerdesShutdown(serdes_cfg0_base);
        if (!CSL_DFESerdesIsReset(serdes_cfg0_base)) {
            serdesCfg0RestoreDefault();
        }
        CSL_DFESerdesShutdown(serdes_cfg1_base);
        if (!CSL_DFESerdesIsReset(serdes_cfg1_base)) {
            serdesCfg1RestoreDefault();
        }

        CSL_DFESerdesInit(serdes_cfg0_base, refClock, serdesRate);
        CSL_DFESerdesInit(serdes_cfg1_base, refClock, serdesRate);

        for(i=0; i < 2; i++)
        {
            if(1==hDfeSerDesCfg->laneEnable[i]) {
                if (1==hDfeSerDesCfg->lbackEnable[i]) {       //setup links for internal loopback mode
                    loopback = CSL_SERDES_LOOPBACK_ENABLED;
                } else {
                    loopback = CSL_SERDES_LOOPBACK_DISABLED;
                }
                CSL_DFESerdesLaneEnable(serdes_cfg0_base, i, loopback, hDfeSerDesCfg->laneCtrlRate[i]);
                numlanescfg0++;
            }
        }
        for(i=2; i < 4; i++)
        {
            if(1==hDfeSerDesCfg->laneEnable[i]) {
                if (1==hDfeSerDesCfg->lbackEnable[i]) {       //setup links for internal loopback mode
                    loopback = CSL_SERDES_LOOPBACK_ENABLED;
                } else {
                    loopback = CSL_SERDES_LOOPBACK_DISABLED;
                }
                CSL_DFESerdesLaneEnable(serdes_cfg1_base, i-2, loopback, hDfeSerDesCfg->laneCtrlRate[i]);
                numlanescfg1++;
            }
        }

        //DFE SerDes PLL Enable
        CSL_DFESerdesPllEnable(serdes_cfg0_base);
        CSL_DFESerdesPllEnable(serdes_cfg1_base);

        //DFE SerDes PLL Status Poll
        retval = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
        tcnt = 0;
        while(retval == CSL_SERDES_STATUS_PLL_NOT_LOCKED)
        {
            retval = CSL_DFESerdesGetStatus(serdes_cfg0_base,numlanescfg0);
            tcnt++;
            if (tcnt > 50000)
            	break;
        }
        if (tcnt > 50000)
#ifndef USESYSBIOS
        	printf("serdes0 PLL is unlocked!\n");
        else
        	printf("serdes0 PLL is locked!\n");
#else
			System_printf("serdes0 PLL is unlocked!\n");
		else
			System_printf("serdes0 PLL is locked!\n");
#endif
        //DFE SerDes PLL Status Poll
        retval = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
        tcnt = 0;
        while(retval == CSL_SERDES_STATUS_PLL_NOT_LOCKED)
        {
            retval = CSL_DFESerdesGetStatus(serdes_cfg1_base,numlanescfg1);
            tcnt++;
            if (tcnt > 50000)
            	break;
        }
        if (tcnt > 50000)
#ifndef USESYSBIOS
        	printf("serdes1 PLL is unlocked!\n");
        else
        	printf("serdes1 PLL is locked!\n");
#else
			System_printf("serdes1 PLL is unlocked!\n");
		else
			System_printf("serdes1 PLL is locked!\n");
#endif
    }
}

#else

void UTILS_dfeSerdesConfig(
        UTILS_dfeSerDesCfg        *hDfeSerDesCfg,
        CSL_SERDES_REF_CLOCK      refClock,
        CSL_SERDES_LINK_RATE      serdesRate
)
{
    uint32_t                    i;
//    CSL_SERDES_LOOPBACK         loopback;
    uint32_t                    serdes_mux_dfe_sel=0, serdes_common_ref_clock=0;// numlanescfg0=0, numlanescfg1=0;
    CSL_SERDES_RESULT status;
    CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
    CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params1, serdes_lane_enable_params2;

#ifdef _TMS320C6X
    // start CPU timestamp as it is needed by the SerDes CSL to compute delays
    TSCL=0;
#endif

    memset(&serdes_lane_enable_params1, 0, sizeof(serdes_lane_enable_params1));
    memset(&serdes_lane_enable_params2, 0, sizeof(serdes_lane_enable_params2));

    /* Check CSISC2_0_MUXSEL bit */
    if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 26, 26) == 0) serdes_mux_dfe_sel = 1;
    //serdes_mux_dfe_sel = 1;

    /* Check CSISC2_0_CLKCTL bit */
    //if (CSL_FEXTR(*(volatile uint32_t *)(boot_cfg_base + 0x20), 27, 27) == 1) serdes_common_ref_clock = 1;
    serdes_common_ref_clock = 1;

    if (serdes_mux_dfe_sel == 0) {
#ifndef USESYSBIOS
        printf("Error: can't configure dfe serdes given CSISC2_0_MUXSEL\n");
#else
        System_printf("Error: can't configure dfe serdes given CSISC2_0_MUXSEL\n");
#endif
    } else if (serdes_common_ref_clock == 0) {
#ifndef USESYSBIOS
        printf("Error: can't configure dfe serdes given both blocks don't use the same ref clock\n");
#else
        System_printf("Error: can't configure dfe serdes given both blocks don't use the same ref clock\n");
#endif
    } else {

        serdes_lane_enable_params1.base_addr = serdes_cfg0_base;
        serdes_lane_enable_params1.ref_clock = refClock;
        serdes_lane_enable_params1.linkrate = serdesRate;
        serdes_lane_enable_params1.num_lanes = 2;
        serdes_lane_enable_params1.phy_type = SERDES_DFE;
        serdes_lane_enable_params1.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
        serdes_lane_enable_params1.lane_mask = 0x3;
        serdes_lane_enable_params1.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

        CSL_SERDES_SHUTDOWN(serdes_lane_enable_params1.base_addr, serdes_lane_enable_params1.num_lanes);

        serdes_lane_enable_params2.base_addr = serdes_cfg1_base;
        serdes_lane_enable_params2.ref_clock = refClock;
        serdes_lane_enable_params2.linkrate = serdesRate;
        serdes_lane_enable_params2.num_lanes = 2;
        serdes_lane_enable_params2.phy_type = SERDES_DFE;
        serdes_lane_enable_params2.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
        serdes_lane_enable_params2.lane_mask = 0x3;
        serdes_lane_enable_params2.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

        CSL_SERDES_SHUTDOWN(serdes_lane_enable_params2.base_addr, serdes_lane_enable_params2.num_lanes);

        for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
        {
            if(1==hDfeSerDesCfg->laneEnable[i])
            {
                if (1==hDfeSerDesCfg->lbackEnable[i])
                {   //setup links for internal loopback mode
                    serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_ENABLED;
                }
                else
                {
                    serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
                }
            }
            serdes_lane_enable_params1.lane_ctrl_rate[i] = hDfeSerDesCfg->laneCtrlRate[i];

            /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
            serdes_lane_enable_params1.rx_coeff.att_start[i] = 7;
            serdes_lane_enable_params1.rx_coeff.boost_start[i] = 5;

            /* For higher speeds PHY-A, force attenuation and boost values  */
            serdes_lane_enable_params1.rx_coeff.force_att_val[i] = 1;
            serdes_lane_enable_params1.rx_coeff.force_boost_val[i] = 1;

            /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
            serdes_lane_enable_params1.tx_coeff.cm_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.c1_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.c2_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.tx_att[i] = 12;
            serdes_lane_enable_params1.tx_coeff.tx_vreg[i] = 4;
        }

        for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
        {
            if(1==hDfeSerDesCfg->laneEnable[i+2])
            {
                if (1==hDfeSerDesCfg->lbackEnable[i+2])
                {   //setup links for internal loopback mode
                    serdes_lane_enable_params2.loopback_mode[i] = CSL_SERDES_LOOPBACK_ENABLED;
                }
                else
                {
                    serdes_lane_enable_params2.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
                }
            }
            serdes_lane_enable_params2.lane_ctrl_rate[i] = hDfeSerDesCfg->laneCtrlRate[i+2];
            /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
            serdes_lane_enable_params2.rx_coeff.att_start[i] = 7;
            serdes_lane_enable_params2.rx_coeff.boost_start[i] = 5;

            /* For higher speeds PHY-A, force attenuation and boost values  */
            serdes_lane_enable_params2.rx_coeff.force_att_val[i] = 1;
            serdes_lane_enable_params2.rx_coeff.force_boost_val[i] = 1;

            /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
            serdes_lane_enable_params2.tx_coeff.cm_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.c1_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.c2_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.tx_att[i] = 12;
            serdes_lane_enable_params2.tx_coeff.tx_vreg[i] = 4;
        }

        status = CSL_DFESerdesInit(serdes_lane_enable_params1.base_addr,
                                        serdes_lane_enable_params1.ref_clock,
                                        serdes_lane_enable_params1.linkrate);
        status = CSL_DFESerdesInit(serdes_lane_enable_params2.base_addr,
                                        serdes_lane_enable_params2.ref_clock,
                                        serdes_lane_enable_params2.linkrate);

        if (status != 0)
        {
#ifndef USESYSBIOS
            printf ("Invalid Serdes Init Params\n");
#else
            System_printf ("Invalid Serdes Init Params\n");
#endif
        }

        /* Common Init Mode */
        /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
        /* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
        serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
        serdes_lane_enable_params1.lane_mask = 0x3;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);

        serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
        serdes_lane_enable_params2.lane_mask = 0x3;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);

        /* Lane Init Mode */
        /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
           iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
        /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
        serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
        for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
        {
            serdes_lane_enable_params1.lane_mask = 1<<i;
            lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
            if (lane_retval != 0)
            {
#ifndef USESYSBIOS
                printf ("Invalid Serdes Lane Enable Init\n");
#else
                System_printf ("Invalid Serdes Lane Enable Init\n");
#endif						
            }
        }

        serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
        for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
        {
            serdes_lane_enable_params2.lane_mask = 1<<i;
            lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
            if (lane_retval != 0)
            {
#ifndef USESYSBIOS
                printf ("Invalid Serdes Lane Enable Init\n");
#else
                System_printf ("Invalid Serdes Lane Enable Init\n");
#endif
            }
    	}
#ifndef USESYSBIOS
        printf("JESD Serdes Init Complete\n");
#else
        System_printf("JESD Serdes Init Complete\n");
#endif				
}
}

#endif

////////////////////
