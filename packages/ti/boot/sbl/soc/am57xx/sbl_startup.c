/**
 * \file   sbl_startup.c
 *
 * \brief  Includes Functions which set up the MMU and enable Cache for the
 *         device. It also relocates the vector table to another location.
 *
 *  \copyright Copyright (C) 2015-2017 Texas Instruments Incorporated -
 *             http://www.ti.com/
 */

/**
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
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/tistdtypes.h>
#include <ti/csl/csl_a15.h>
#include <ti/csl/arch/a15/csl_a15_startup.h>
#include "sbl_startup.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#define MMU_PAGETABLE_ALIGN_SIZE                           (16U * 1024U)

/** \brief Page table configuration.*/
/** \brief Page tables to hold physical to virtual address mapping. The start
           address of the page table must be aligned at 16K boundary */
CSL_A15MmuLongDescObj mmuObj 
    __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)))
    __attribute__((section("SBL_MMU_TABLE")));

extern void CSL_A15_INIT_copyVectorTable(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void SBL_MMUInit(void)
{
    uint32_t phyAddr = 0U;
    int32_t cacheType;
    CSL_A15MmuLongDescAttr mmuAttr0;
    CSL_A15MmuLongDescAttr mmuAttr1;

    cacheType = CSL_a15GetCacheType();

    if(cacheType & CSL_A15_CACHE_TYPE_ALL)
    {
        /* Invalidate all of L1 Instruction Cache. */
        CSL_a15InvAllInstrCache();

        /* Disable Cache. */
        CSL_a15DisableCache();
    }

    mmuObj.numFirstLvlEntires = CSL_A15_MMU_LONG_DESC_LVL1_ENTIRES;
    mmuObj.numSecondLvlEntires = CSL_A15_MMU_LONG_DESC_LVL2_ENTIRES;
    mmuObj.mairEntires = CSL_A15_MMU_MAIR_LEN_BYTES;
    mmuObj.mairAttr[0] = 0x44U;
    mmuObj.mairAttr[1] = 0x00U;
    mmuObj.mairAttr[2] = 0xFFU;
    CSL_a15InitMmuLongDesc(&mmuObj);
    CSL_a15InitMmuLongDescAttrs(&mmuAttr0);
    CSL_a15InitMmuLongDescAttrs(&mmuAttr1);

    mmuAttr0.type = CSL_A15_MMU_LONG_DESC_TYPE_BLOCK;
    mmuAttr0.accPerm = 0U;
    mmuAttr0.shareable = 0U;
    mmuAttr0.attrIndx = 1U;

    for (phyAddr = 0x00000000U; phyAddr < 0x60000000U; phyAddr += 0x00200000U)
    {
        CSL_a15SetMmuSecondLevelLongDesc(&mmuObj, (void *)phyAddr, (void *)phyAddr, &mmuAttr0);
    }

    mmuAttr1.type = CSL_A15_MMU_LONG_DESC_TYPE_BLOCK;
    mmuAttr1.accPerm = 0U;
    mmuAttr1.shareable = 2U;
    mmuAttr1.attrIndx = 2U;

    for (phyAddr = 0x80000000; phyAddr < 0xC0000000; phyAddr += 0x200000U)
    {
        CSL_a15SetMmuSecondLevelLongDesc(&mmuObj, (void *)phyAddr, (void *)phyAddr, &mmuAttr1);
    }

#if defined(AM572x_BUILD) || defined(AM574x_BUILD)
    for (phyAddr = 0xC0000000; phyAddr >= 0xC0000000 && phyAddr < 0xFFFFFFFF; phyAddr += 0x200000U)
    {
        CSL_a15SetMmuSecondLevelLongDesc(&mmuObj, (void *)phyAddr, (void *)phyAddr, &mmuAttr1);
    }
#endif

    CSL_a15EnableMmu();
    CSL_a15EnableCache();
}

void SBL_startBoot(void)
{
    CSL_ArmgicCfg_t cfg;

    /* Copy vector table */
    CSL_A15_INIT_copyVectorTable();

    /**
    * Note: The application aborts if the MMU and Cache is re-enabled.
    **/
    /* if MMU is already enabled then do not re-enable*/
    if (1 != CSL_a15IsMmuEnabled())
    {
        SBL_MMUInit();
    }

    /* Initialize GIC */
    cfg.ctrlBipMap = 0;
    CSL_A15_INIT_startup2(&cfg);

    /* Calling the main */
    main();
}
