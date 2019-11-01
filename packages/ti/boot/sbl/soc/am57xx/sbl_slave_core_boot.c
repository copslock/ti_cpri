/**
 *  \file   sbl_slave_core_boot.c
 *
 *  \brief  This file contain functions related to slave core boot-up.
 *
 */

/*
 * Copyright (C) 2015-2018 Texas Instruments Incorporated - http://www.ti.com/
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
 #include <ti/csl/cslr_device.h>
 #include <ti/csl/hw_types.h>
 #include <ti/drv/uart/UART_stdio.h>

 #include "sbl_prcm.h"
 #include "sbl_slave_core_boot.h"

#if defined(BOOT_QSPI)
#include "sbl_qspi.h"
#endif

#if defined(BOOT_MMCSD)
#include "sbl_mmcsd.h"
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Macro defining the reg address where DSP1 Core boot address is to be written */
#define DSP1BOOTADDR            (CSL_DSP_CTRL_MODULE_CORE_REGS + 0x55CU)
#define DSP1BOOTADDRVALUE       (0x00800000U)

/* Entry address for DSP1 applications in the memory region where trampolene is located */
#define DSP1_ENTRY_ADDR         (0x40330400)

/* Macro defining the reg address where DSP2 Core boot address is to be written */
#define DSP2BOOTADDR            (CSL_DSP_CTRL_MODULE_CORE_REGS + 0x560U)
#define DSP2BOOTADDRVALUE       (0x00800000U)

/* Entry address for DSP2 applications in the memory region where trampolene is located */
#define DSP2_ENTRY_ADDR         (0x40330800)

#define MPU_IPU1_RAM             (CSL_IPU_IPU1_TARGET_REGS + \
                                     (uint32_t) 0x20000)

#define MPU_IPU2_RAM			 (CSL_IPU_IPU1_ROM_REGS + \
                                     (uint32_t) 0x20000)

/**
 * \brief This API enables the clock for DSP1 core.
 *
 */
void DSP1_ClkEnable();

/**
 * \brief This API enables the clock for DSP2 core.
 *
 */
void DSP2_ClkEnable();

/**
 * \brief This API enables the clock for IPU1 core.
 *
 */
void IPU1_ClkEnable();

/**
 * \brief This API enables the clock for IPU2 core.
 *
 */
void IPU2_ClkEnable();

/**
 * \brief This API brings the IPU1 core out of reset.
 *
 */
void IPU1_SystemReset();

/**
 * \brief This API brings the IPU2 core out of reset.
 *
 */
void IPU2_SystemReset();

/**
 * \brief        This is a generic function to get the DSP cores out of reset.
 *
 * \param  cpu   core id to identify the core which has to be reset.
 */
void SBL_CPU_Reset(cpu_core_id_t cpu);

/**
 *  \brief   IPU1_AMMU_Config function to configure the IPU1 AMMU. Add
 *           entries to access DDR, L4Per1,L4Per2 & L4Per3,
 *           L3 RAM & IRAM
**/
void IPU1_AMMU_Config(void);

/**
 *  \brief   IPU2_AMMU_Config function to configure the IPU2 AMMU. Add
 *           entries to access DDR, L4Per1,L4Per2 & L4Per3,
 *           L3 RAM & IRAM
**/
void IPU2_AMMU_Config(void);

/**
 *  \brief    Sets the Entry Point for C66 cores within the Instruction opcodes
 *            maintained for each C66 cores.
 *
**/
static void SBL_DspEntryPointSet(uint32_t entryPoint, uint32_t *pDspInstr);

/**
 *  \brief    Sends event to be signalled to all the CPUS.
**/
void CPUSendEvent(void);

/*
** DSP instructions to be copied to the Trampolene to avoid the alignment
** requirement.
*/
uint32_t dsp1Instruction[10] =
{
    0x0500002a, /* MVK.S2  destAddr, B10      */
    0x0500006a, /* MVKH.S2 destAddr, B10      */
    0x00280362, /* B.S2    B10                */
    0x00006000, /* NOP     4                  */
    0x00000000, /* NOP                        */
    0x00000000 /* NOP                        */
};

uint32_t dsp2Instruction[10] =
{
    0x0500002a, /* MVK.S2  destAddr, B10      */
    0x0500006a, /* MVKH.S2 destAddr, B10      */
    0x00280362, /* B.S2    B10                */
    0x00006000, /* NOP     4                  */
    0x00000000, /* NOP                        */
    0x00000000 /* NOP                        */
};

/* ========================================================================== */
/*                           Internal Functions                               */
/* ========================================================================== */

int32_t SBL_ImageCopy(sblEntryPoint_t *pEntry)
{
    int32_t retval = 0;

#if defined(BOOT_MMCSD) || defined(BOOT_EMMC)
    /* MMCSD Boot Mode Image Copy function. */
    if (SBL_MMCBootImage(pEntry) != 1U)
#elif defined(BOOT_QSPI)
    if (SBL_QSPIBootImage(pEntry) != 1U)
#endif
    {
        retval = -1;
    }

    return retval;
}

void SBL_SlaveCorePrcmEnable()
{
	/* Enable Clocks and Bring the slave cores out of Reset. */
	IPU1_ClkEnable();
	IPU1_SystemReset();

	#if defined (AM572x_BUILD) || defined (AM574x_BUILD)
	IPU2_ClkEnable();
	IPU2_SystemReset();
	#endif

    DSP1_ClkEnable();
    SBL_CPU_Reset(DSP1_ID);

    #if defined (AM572x_BUILD) || defined (AM574x_BUILD)
    DSP2_ClkEnable();
    SBL_CPU_Reset(DSP2_ID);
    #endif
}

void DSP1_ClkEnable()
{
    SBL_PRCMModuleEnable(CSL_MPU_DSP1_CM_CORE_AON_REGS,
                    CSL_DSP1_CM_CORE_AON_CM_DSP1_DSP1_CLKCTRL_REG,
                    CSL_DSP1_CM_CORE_AON_CM_DSP1_CLKSTCTRL_REG,
                    CM_DSP1_CLKSTCTRL_CLKACTIVITY_DSP1_GFCLK_MASK);

    SBL_PRCMSetClkOperMode(CSL_MPU_DSP1_CM_CORE_AON_REGS,
        CSL_DSP1_CM_CORE_AON_CM_DSP1_CLKSTCTRL_REG,
        PRCM_CD_CLKTRNMODES_SW_WAKEUP);
}

void DSP2_ClkEnable()
{
    SBL_PRCMModuleEnable(CSL_MPU_DSP2_CM_CORE_AON_REGS,
                    CSL_DSP2_CM_CORE_AON_CM_DSP2_DSP2_CLKCTRL_REG,
                    CSL_DSP2_CM_CORE_AON_CM_DSP2_CLKSTCTRL_REG,
                    CM_DSP2_CLKSTCTRL_CLKACTIVITY_DSP2_GFCLK_MASK);

    SBL_PRCMSetClkOperMode(CSL_MPU_DSP2_CM_CORE_AON_REGS,
        CSL_DSP2_CM_CORE_AON_CM_DSP2_CLKSTCTRL_REG,
        PRCM_CD_CLKTRNMODES_SW_WAKEUP);
}

void IPU1_ClkEnable()
{
	/* Select CORE_IPU_ISS_BOOST_CLK from CORE_DPLL for IPU1_GFCLK */
	HW_WR_FIELD32_RAW(CSL_IPU_IPU_CM_CORE_AON_REGS + CM_IPU1_IPU1_CLKCTRL,
	CM_IPU1_IPU1_CLKCTRL_CLKSEL_MASK, CM_IPU1_IPU1_CLKCTRL_CLKSEL_SHIFT,
	CM_IPU1_IPU1_CLKCTRL_CLKSEL_SEL_CORE_IPU_ISS_BOOST_CLK);

    SBL_PRCMModuleEnable(CSL_IPU_IPU_CM_CORE_AON_REGS,
                    CM_IPU1_IPU1_CLKCTRL,
                    CM_IPU1_CLKSTCTRL,
                    CM_IPU_CLKSTCTRL_CLKACTIVITY_IPU_L3_GICLK_MASK);

    SBL_PRCMSetClkOperMode(CSL_IPU_IPU_CM_CORE_AON_REGS,
        CM_IPU1_CLKSTCTRL,
        PRCM_CD_CLKTRNMODES_SW_WAKEUP);
}

void IPU2_ClkEnable()
{
    SBL_PRCMModuleEnable(CSL_IPU_CORE_CM_CORE_REGS,
                    CM_IPU2_IPU2_CLKCTRL,
                    CM_IPU2_CLKSTCTRL,
                    CM_IPU2_CLKSTCTRL_CLKACTIVITY_IPU2_GFCLK_MASK);

    SBL_PRCMSetClkOperMode(CSL_MPU_DSP1_CM_CORE_AON_REGS,
        CM_IPU2_CLKSTCTRL,
        PRCM_CD_CLKTRNMODES_SW_WAKEUP);
}

void SBL_CPU_Reset(cpu_core_id_t cpu)
{
    uint32_t retVal;

    if(cpu == DSP1_ID)
    {
        /* SYSTEM Reset */
        SBL_ResetAssert((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTCTRL),
                        RM_DSP1_RSTCTRL_RST_DSP1_SHIFT);

        /* Local Reset */
        SBL_ResetAssert((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTCTRL),
                        RM_DSP1_RSTCTRL_RST_DSP1_LRST_SHIFT);

        SBL_ResetGetStatus((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTST),
                RM_DSP1_RSTST_RST_DSP1_LRST_SHIFT, &retVal);

        if(0x1U == retVal)
        {
            /* clear Status */
            SBL_ResetClearStatus((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTST),
                RM_DSP1_RSTST_RST_DSP1_LRST_SHIFT);
        }

        /* System Reset */
        SBL_ResetGetStatus((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTST),
                RM_DSP1_RSTST_RST_DSP1_SHIFT, &retVal);

        if(0x1U == retVal)
        {
            /* Clear Status */
            SBL_ResetClearStatus((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTST),
                RM_DSP1_RSTST_RST_DSP1_SHIFT);
        }

        SBL_ResetRelease((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTCTRL),
                          RM_DSP1_RSTCTRL_RST_DSP1_SHIFT,
                        (CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTST),
                            RM_DSP1_RSTST_RST_DSP1_SHIFT);
    }
    else if(cpu == DSP2_ID)
    {
        /* SYSTEM Reset */
        SBL_ResetAssert((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTCTRL),
                        RM_DSP2_RSTCTRL_RST_DSP2_SHIFT);

        /* Local Reset */
        SBL_ResetAssert((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTCTRL),
                        RM_DSP2_RSTCTRL_RST_DSP2_LRST_SHIFT);

        SBL_ResetGetStatus((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTST),
                RM_DSP2_RSTST_RST_DSP2_LRST_SHIFT, &retVal);

        if(0x1U == retVal)
        {
            /* clear Status */
            SBL_ResetClearStatus((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTST),
                RM_DSP2_RSTST_RST_DSP2_LRST_SHIFT);
        }

        /* System Reset */
        SBL_ResetGetStatus((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTST),
                RM_DSP2_RSTST_RST_DSP2_SHIFT, &retVal);

        if(0x1U == retVal)
        {
            /* Clear Status */
            SBL_ResetClearStatus((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTST),
                RM_DSP2_RSTST_RST_DSP2_SHIFT);
        }

        SBL_ResetRelease((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTCTRL),
                          RM_DSP2_RSTCTRL_RST_DSP2_SHIFT,
                        (CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTST),
                            RM_DSP2_RSTST_RST_DSP2_SHIFT);
    }
}

void SBL_DSP1_BringUp(uint32_t EntryPoint)
{
    uint32_t i = 0U;

    if(EntryPoint != 0U)
    {
        /* Set the Entry point of the application in the trampolene code. */
        SBL_DspEntryPointSet(EntryPoint, dsp1Instruction);
        /*
        ** Copy the dsp opcodes to create a trampoline at the specified memory
        ** location which is aligned at 0x400. This helps us in removing the
        ** entry address alignment limitation.
        */
        memcpy((void *) DSP1_ENTRY_ADDR, (void *)dsp1Instruction,
            sizeof(dsp1Instruction));
    }

    /*DSP L2RAM*/
    /* Set the Entry point */
    if (EntryPoint != 0U)
    {
        HW_WR_REG32(DSP1BOOTADDR, (DSP1_ENTRY_ADDR >> 0xAU));
    }
    else
    {
        HW_WR_REG32(DSP1BOOTADDR, (EntryPoint >> 0xAU));
        /* Self branch loop for DSP */
         for (i = 0; i < 8; i++) {
            HWREG(CSL_MPU_DSP1_L2_SRAM_REGS + 4 * i) = 0x12;
        }
    }

    /* Reset de-assertion for DSP CPUs */
    HW_WR_REG32((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTCTRL), 0x0);
    /* Check the reset state: DSPSS Core, Cache and Slave interface */
    while ((HW_RD_REG32((CSL_DSP_DSP1_PRM_REGS + RM_DSP1_RSTST)) & 0x3) != 0x3);
    /* Check module mode */
    while ((HW_RD_REG32(CSL_MPU_DSP1_CM_CORE_AON_REGS + CSL_DSP1_CM_CORE_AON_CM_DSP1_DSP1_CLKCTRL_REG) & 0x30000) != 0x0);

}

#if defined (AM572x_BUILD) || defined (AM574x_BUILD)
void SBL_DSP2_BringUp(uint32_t EntryPoint)
{
    uint32_t i = 0U;

     if(EntryPoint != 0U)
    {
        /* Set the Entry point of the application in the trampolene code. */
        SBL_DspEntryPointSet(EntryPoint, dsp2Instruction);
        /*
        ** Copy the updated dsp opcodes to create a trampoline at the specified
        ** memory location.
        */
        memcpy((void *) DSP2_ENTRY_ADDR, (void *)dsp2Instruction,
            sizeof(dsp2Instruction));
    }

    /*DSP L2RAM*/
    /*Set the Entry point*/
    if (EntryPoint != 0U)
    {
        HW_WR_REG32(DSP2BOOTADDR, (DSP2_ENTRY_ADDR >> 0xAU));
    }
    else
    {
        HW_WR_REG32(DSP2BOOTADDR, (EntryPoint >> 0xAU));
        /* Self branch loop for DSP */
         for (i = 0; i < 8; i++) {
            HWREG(CSL_MPU_DSP2_L2_SRAM_REGS + 4 * i) = 0x12;
        }
    }

    /* Reset de-assertion for DSP CPUs */
    HW_WR_REG32((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTCTRL), 0x0);
    /* Check the reset state: DSPSS Core, Cache and Slave interface */
    while ((HW_RD_REG32((CSL_DSP_DSP2_PRM_REGS + RM_DSP2_RSTST)) & 0x3) != 0x3);
    /* Check module mode */
    while ((HW_RD_REG32(CSL_MPU_DSP2_CM_CORE_AON_REGS + CSL_DSP2_CM_CORE_AON_CM_DSP2_DSP2_CLKCTRL_REG) & 0x30000) != 0x0);

}
#endif

void IPU1_SystemReset()
{
	uint32_t retStat = 0U;

	/* Assert the Reset */
	SBL_ResetAssert((CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTCTRL),
					RM_IPU1_RSTCTRL_RST_CPU1_SHIFT);

	SBL_ResetAssert((CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTCTRL),
					RM_IPU1_RSTCTRL_RST_CPU0_SHIFT);

	SBL_ResetAssert((CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTCTRL),
					RM_IPU1_RSTCTRL_RST_IPU_SHIFT);

	/* Check the Reset Status and Clear */
	SBL_ResetGetStatus(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
		RM_IPU1_RSTST_RST_CPU1_SHIFT, &retStat);

	if(0x1U == retStat)
	{
		SBL_ResetClearStatus(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
			RM_IPU1_RSTST_RST_CPU1_SHIFT);
		retStat = 0U;
	}

	SBL_ResetGetStatus(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
		RM_IPU1_RSTST_RST_CPU0_SHIFT, &retStat);

	if(0x1U == retStat)
	{
		SBL_ResetClearStatus(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
			RM_IPU1_RSTST_RST_CPU0_SHIFT);
		retStat = 0U;
	}

	SBL_ResetGetStatus(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
		RM_IPU1_RSTST_RST_IPU_SHIFT, &retStat);

	if(0x1U == retStat)
	{
		SBL_ResetClearStatus(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
			RM_IPU1_RSTST_RST_IPU_SHIFT);
		retStat = 0U;
	}

	/*Configure the boot translation page of IPU1 to 0x55020000 - IPU_RAM*/
    HW_WR_REG32((
        CSL_IPU_CTRL_MODULE_CORE_CORE_REGISTERS_REGS + 0x35CU), 0x00000);

    HW_WR_REG32((
        CSL_IPU_CTRL_MODULE_CORE_CORE_REGISTERS_REGS + 0x358U), 0x55020);

	SBL_ResetRelease(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTCTRL,
						RM_IPU1_RSTCTRL_RST_IPU_SHIFT,
                        CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
                        RM_IPU1_RSTST_RST_IPU_SHIFT);

	/* Configure the MMU settings for IPU1 M4 */
	IPU1_AMMU_Config();
}


void SBL_IPU1_CPU0_BringUp(uint32_t EntryPoint)
{
	uint32_t regVal = 0U;

    /*Set the Entry point*/
    if (EntryPoint == 0)
    {
        HW_WR_REG32(MPU_IPU1_RAM, 0x10000);
        HW_WR_REG32((MPU_IPU1_RAM + 0x4), 0x9);
        HW_WR_REG32((MPU_IPU1_RAM + 0x8), 0xE7FEE7FE);
    }
    else
    {
        HW_WR_REG32(MPU_IPU1_RAM, 0x10000);
        HW_WR_REG32((MPU_IPU1_RAM + 0x4), EntryPoint);
    }

    /* Bring-out of Reset - CPU0*/
	regVal = HW_RD_REG32(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTCTRL);
	regVal &= ~0x1;
	HW_WR_REG32((CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTCTRL), regVal);
    while ((HW_RD_REG32(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST) & 0x1) != 0x1);

    /*Check the Status of IPU1 Module mode*/
    while ((HW_RD_REG32(CSL_IPU_IPU_CM_CORE_AON_REGS +
                  CM_IPU1_IPU1_CLKCTRL) & 0x30000) != 0x0) ;
}

void SBL_IPU1_CPU1_BringUp(uint32_t EntryPoint)
{
	uint32_t retStat = 0U;

    /*Assert the CPU1 Reset*/
	SBL_ResetAssert((CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTCTRL),
					RM_IPU1_RSTCTRL_RST_CPU1_SHIFT);

	/* Check the Reset Status */
	SBL_ResetGetStatus(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
		RM_IPU1_RSTST_RST_CPU1_SHIFT, &retStat);

	if(retStat == 0x1U)
	{
		SBL_ResetClearStatus(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST,
			RM_IPU1_RSTST_RST_CPU1_SHIFT);
		retStat = 0U;
	}

    /*Set the Entry point*/
    if (EntryPoint == 0)
    {
        HW_WR_REG32(MPU_IPU1_RAM, 0x10000);
        HW_WR_REG32((MPU_IPU1_RAM + 0x4), 0x9);
        HW_WR_REG32((MPU_IPU1_RAM + 0x8), 0xE7FEE7FE);
    }
    else
    {
        HW_WR_REG32(MPU_IPU1_RAM, 0x10000);
        HW_WR_REG32((MPU_IPU1_RAM + 0x4), EntryPoint);
    }

    /*Bring-out of Reset - CPU1*/
    HWREG(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTCTRL) &= ~0x2;
    while ((HWREG(CSL_IPU_IPU_PRM_REGS + RM_IPU1_RSTST) & 0x2) != 0x2) ;

    /*Check the Status of IPU1 Module mode*/
    while ((HWREG(CSL_IPU_IPU_CM_CORE_AON_REGS +
                  CM_IPU1_IPU1_CLKCTRL) & 0x30000) != 0x0);
}

#if defined (AM572x_BUILD) || defined (AM574x_BUILD)
void IPU2_SystemReset()
{
	uint32_t retStat = 0U;

	/* Assert the Reset */
	SBL_ResetAssert((CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL),
                     RM_IPU2_RSTCTRL_RST_CPU1_SHIFT);

	SBL_ResetAssert((CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL),
					RM_IPU2_RSTCTRL_RST_CPU0_SHIFT);

	SBL_ResetAssert((CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL),
					RM_IPU2_RSTCTRL_RST_IPU_SHIFT);

	/* Check the Reset Status and Clear */
	SBL_ResetGetStatus(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
		RM_IPU2_RSTST_RST_CPU1_SHIFT, &retStat);

	if(0x1U == retStat)
	{
		SBL_ResetClearStatus(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
			RM_IPU2_RSTST_RST_CPU1_SHIFT);
		retStat = 0U;
	}

	SBL_ResetGetStatus(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
		RM_IPU2_RSTST_RST_CPU0_SHIFT, &retStat);

	if(0x1U == retStat)
	{
		SBL_ResetClearStatus(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
			RM_IPU2_RSTST_RST_CPU0_SHIFT);
		retStat = 0U;
	}

	SBL_ResetGetStatus(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
		RM_IPU2_RSTST_RST_IPU_SHIFT, &retStat);

	if(0x1U == retStat)
	{
		SBL_ResetClearStatus(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
			RM_IPU2_RSTST_RST_IPU_SHIFT);
		retStat = 0U;
	}

	/*Configure the boot translation page of IPU1 to 0x55020000 - IPU_RAM*/
    HW_WR_REG32((
        CSL_IPU_CTRL_MODULE_CORE_CORE_REGISTERS_REGS + 0x35CU), 0x00000);

    HW_WR_REG32((
        CSL_IPU_CTRL_MODULE_CORE_CORE_REGISTERS_REGS + 0x358U), 0x55020);

	SBL_ResetRelease(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL,
                        RM_IPU2_RSTCTRL_RST_IPU_SHIFT,
                        CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
                        RM_IPU2_RSTST_RST_IPU_SHIFT);

	/* Configure the MMU settings for IPU2 M4 */
	IPU2_AMMU_Config();
}

void SBL_IPU2_CPU0_BringUp(uint32_t EntryPoint)
{
    uint32_t regVal = 0U;

    /*Set the Entry point*/
    if (EntryPoint == 0)
    {
        HW_WR_REG32(MPU_IPU2_RAM, 0x10000);
        HW_WR_REG32((MPU_IPU2_RAM + 0x4), 0x9);
        HW_WR_REG32((MPU_IPU2_RAM + 0x8), 0xE7FEE7FE);
    }
    else
    {
        HW_WR_REG32(MPU_IPU2_RAM, 0x10000);
        HW_WR_REG32((MPU_IPU2_RAM + 0x4), EntryPoint);
    }

    /* Bring-out of Reset - CPU0*/
	regVal = HW_RD_REG32(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL);
	regVal &= ~0x1;
	HW_WR_REG32((CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL), regVal);
	while ((HW_RD_REG32(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST) & 0x1) != 0x1);

    /*Check the Status of IPU1 Module mode*/
    while ((HW_RD_REG32(CSL_IPU_CORE_CM_CORE_REGS +
                  CM_IPU2_IPU2_CLKCTRL) & 0x30000) != 0x0);
}

void SBL_IPU2_CPU1_BringUp(uint32_t EntryPoint)
{
    uint32_t regVal = 0;
	uint32_t retStat = 0U;

	/*Assert the CPU1 Reset*/
	SBL_ResetAssert((CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL),
					RM_IPU2_RSTCTRL_RST_CPU1_SHIFT);

	/* Check the Reset Status */
	SBL_ResetGetStatus(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
		RM_IPU2_RSTST_RST_CPU1_SHIFT, &retStat);

	if(retStat == 0x1U)
	{
		SBL_ResetClearStatus(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST,
			RM_IPU2_RSTST_RST_CPU1_SHIFT);
		retStat = 0U;
	}

    /*Set the Entry point*/
    if (EntryPoint == 0)
    {
        HW_WR_REG32(MPU_IPU2_RAM, 0x10000);
        HW_WR_REG32((MPU_IPU2_RAM + 0x4), 0x9);
        HW_WR_REG32((MPU_IPU2_RAM + 0x8), 0xE7FEE7FE);
    }
    else
    {
        HW_WR_REG32(MPU_IPU2_RAM, 0x10000);
        HW_WR_REG32((MPU_IPU2_RAM + 0x4), EntryPoint);
    }

    /*Bring-out of Reset - CPU1*/
	regVal = HW_RD_REG32(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL);
	regVal &= ~0x2;
	HW_WR_REG32((CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTCTRL), regVal);
	while((HW_RD_REG32(CSL_IPU_CORE_PRM_REGS + RM_IPU2_RSTST) & 0x2) != 0x2);

    /*Check the Status of IPU2 Module mode*/
    while((HW_RD_REG32(CSL_IPU_CORE_CM_CORE_REGS +
                  CM_IPU2_IPU2_CLKCTRL) & 0x30000) != 0x0);
}

/**
 * \brief     This function brings up the MPU Core1. Core0 will program the
 *            AUXBOOT registers and then it will send the SEV instruction.
 *
 * \param     EntryPoint       CPU entry point location
 *
 **/
void SBL_MPU_CPU1_BringUp(uint32_t EntryPoint)
{
    if (0U != EntryPoint)
    {
        /* Write the entry-point into AUXBOOT1 */
        HW_WR_REG32(CSL_MPU_MPU_WUGEN_REGS + 0x804, EntryPoint);

        /* Write the enable indicator into AUXBOOT0 */
        HW_WR_REG32(CSL_MPU_MPU_WUGEN_REGS + 0x800, 0x10U);

        /* Send event to wake-up CPU1 */
        CPUSendEvent();
    }
    else
    {
        HW_WR_REG32(CSL_MPU_MPU_WUGEN_REGS + 0x410U, 0);
        HW_WR_REG32(CSL_MPU_MPU_WUGEN_REGS + 0x414U, 0);
        HW_WR_REG32(CSL_MPU_MPU_WUGEN_REGS + 0x418U, 0);
        HW_WR_REG32(CSL_MPU_MPU_WUGEN_REGS + 0x41CU, 0);
        HW_WR_REG32(CSL_MPU_MPU_WUGEN_REGS + 0x420U, 0);
    }
}
#endif

void IPU1_AMMU_Config(void)
{
    /*Large Page Translations */
    /* Logical Address */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS, 0x40000000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x04U, 0x80000000U);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x08U, 0xa0000000U);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x0CU, 0x60000000);

    /* Physical Address */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x20U, 0x40000000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x24U, 0x80000000U);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x28U, 0xa0000000U);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x2CU, 0x40000000);

    /* Policy Register */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x40U, 0x00000007);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x44U, 0x000B0007);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x48U, 0x00020007);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x4CU, 0x00000007);

    /*Medium Page*/
    /* Logical Address */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x60U, 0x00300000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x64U, 0x00400000);

    /* Physical Address */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0xA0U, 0x40300000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0xA4U, 0x40400000);

    /* Policy Register */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0xE0U, 0x00000007);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0xE4U, 0x00020007);

    /*Small Page*/
    /* Logical Address */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x120U, 0x00000000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x124U, 0x40000000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x128U, 0x00004000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x12CU, 0x00008000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x130U, 0x20000000);

    /* Physical Address */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x1A0U, 0x55020000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x1A4U, 0x55080000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x1A8U, 0x55024000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x1ACU, 0x55028000);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x1B0U, 0x55020000);

    /* Policy Register */
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x220U, 0x0001000B);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x224U, 0x0000000B);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x228U, 0x00010007);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x22CU, 0x00000007);
    HW_WR_REG32(CSL_IPU_UNICACHE_MMU_REGS + 0x230U, 0x00000007);
}

void IPU2_AMMU_Config(void)
{
    /*Large Page Translations */
    /* Logical Address */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE, 0x40000000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x04U, 0x80000000U);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x08U, 0xa0000000U);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x0CU, 0x60000000);

    /* Physical Address */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x20U, 0x40000000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x24U, 0x80000000U);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x28U, 0xa0000000U);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x2CU, 0x40000000);

    /* Policy Register */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x40U, 0x00000007);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x44U, 0x000B0007);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x48U, 0x00020007);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x4CU, 0x00000007);

    /*Medium Page*/
    /* Logical Address */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x60U, 0x00300000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x64U, 0x00400000);

    /* Physical Address */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0xA0U, 0x40300000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0xA4U, 0x40400000);

    /* Policy Register */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0xE0U, 0x00000007);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0xE4U, 0x00020007);

    /*Small Page*/
    /* Logical Address */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x120U, 0x00000000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x124U, 0x40000000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x128U, 0x00004000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x12CU, 0x00008000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x130U, 0x20000000);

    /* Physical Address */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x1A0U, 0x55020000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x1A4U, 0x55080000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x1A8U, 0x55024000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x1ACU, 0x55028000);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x1B0U, 0x55020000);

    /* Policy Register */
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x220U, 0x0001000B);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x224U, 0x0000000B);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x228U, 0x00010007);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x22CU, 0x00000007);
    HW_WR_REG32(SOC_IPU2_TARGET_UNICACHE_MMU_BASE + 0x230U, 0x00000007);
}

static void SBL_DspEntryPointSet(uint32_t entryPoint, uint32_t *pDspInstr)
{
    uint32_t entryVal = 0U;
    uint32_t dspOpcode = 0U;

    entryVal = (entryPoint & 0x0000FFFF);
    dspOpcode = *pDspInstr;
    /*
    ** Mask and update the lower 16 bits of entry address within the MVK
    ** instruction opcode.
    */
    *pDspInstr = (((dspOpcode) & ~(0x007FFF80)) | (( (entryVal) << 0x7) & 0x007FFF80));

    entryVal = ((entryPoint & 0xFFFF0000) >> 16);
    dspOpcode = *(pDspInstr + 1);
    /*
    ** Mask and update the upper 16 bits of entry address within the MVK
    ** instruction opcode.
    */
    *(pDspInstr + 1) = (((dspOpcode) & ~(0x007FFF80)) | (( (entryVal) << 0x7) & 0x007FFF80));
}

void CPUSendEvent(void)
{
    /*Data Synchronization Barrier */
    asm (" DSB");
    /*SEV Instruction */
    asm (" SEV");
}
