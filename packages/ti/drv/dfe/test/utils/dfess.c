/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
*/
#ifndef CONFIG_SRC
#define CONFIG_SRC
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "dfetest.h"
#include <ti/csl/csl.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_tsc.h>

// BOOT and CONFIG dsp system modules Definitions
#define CHIP_LEVEL_REG              (0x02620000)


#define KICK0                       *(volatile unsigned int*)(CHIP_LEVEL_REG + 0x0038)
#define KICK1                       *(volatile unsigned int*)(CHIP_LEVEL_REG + 0x003C)
#define KICK0_UNLOCK                (0x83E70B13)
#define KICK1_UNLOCK                (0x95A4F1E0)
#define KICK_LOCK                   0

#define DFEPLLCTL0                  0x378
#define DFEPLLCTL1                  0x37C
#define DFE_CLKDIV_CTL              0x6E8
#define DFE_CLKSYNC_CTL             0x6EC
#define DFE_TEST_CNTR               0xCBC

#define PLL_MULT_MASK               0x003F
#define PLL_MULT_SHIFT              6
#define PLL_ODIV_MASK               0xF
#define PLL_ODIV_SHIFT              19    
#define PLL_DIVMODE_MASK            0x3
#define PLL_DIVMODE_SHIFT           0  
#define PLL_BWADJ_SHIFT             24     
#define PLL_BWADJ_MASK              0x00FFFFFF    

#define    PLL_BYPASS_SET           0x00800000
#define    PLL_RESET_SET            0x00004000
#define    PLL_PLLSEL_SET           0x00002000
#define    PLL_PLLSEL_RESET         0xFFFFDFFF

void waitfor(int cond, int timeout)       
{
    volatile int i; 
    
    for (i=0; ((i < (timeout)) && !(cond)); i++)
    {
        // spin loop
    }
}

#define sleep(loopCnt)      waitfor(0, loopCnt)

#define mkptr(base,offset)        ((unsigned int *)((base)+(offset)))

/*static inline void CSL_BtCfgLockKicker( uint32_t base_addr )
{
	*(volatile uint32_t *)(base_addr + 0x0038) = KICK_LOCK;
	*(volatile uint32_t *)(base_addr + 0x003C) = KICK_LOCK;
}*/

static inline void CSL_BtCfgUnLockKicker( uint32_t base_addr )
{
    // Uboot unlocks the kicker and Linux expects the kicker to be unlocked
    *(volatile uint32_t *)(base_addr + 0x0038) = KICK0_UNLOCK;
	*(volatile uint32_t *)(base_addr + 0x003C) = KICK1_UNLOCK;
}

/*static inline void CSL_BtCfgInitDfePll( uint32_t base_addr )
{
	*(volatile uint32_t *)(base_addr + DFEPLLCTL0) = 0x098803C0;
	*(volatile uint32_t *)(base_addr + DFEPLLCTL1) = 0x00000040;
}

static inline void CSL_BtCfgInitDfeClk( uint32_t base_addr )
{
	*(volatile uint32_t *)(base_addr + DFE_CLKDIV_CTL) = 0x00000000;
	*(volatile uint32_t *)(base_addr + DFE_CLKSYNC_CTL) = 0x00000000;
}*/


// Per Lamarr Clockspec
// DFE PLL REFCLK = 122.88 MHz
// DFE PLL CLKR = 0x0 (Default: no prediv)
// DFE PLL CLKF = 0x7(Default) or 0x5 (multiplier = n+1 = 8,or 6)
// DFE PLL CLKOD = 0x0(Default) (postdiv = 1, no postdiv)
// DFE PLL CLKOUT = 983.04 (Default) or 737.28MHz

//DFECLKSYNC_CTL register DIVMODE configured to /4 (default) or /2, to support
//DFE operating frequency of 245.76 (default), 368.64 or 491.52 MHz. Note 491.52 is
//frequency is only supported for internal testing.

//DFE Frequency PLL Configuration DFECLKSYNC Divider Mode
//245.76 MHz x8 (983.04 MHz) /4 (0)
//491.52 MHz x8 (983.04 MHz) /2 (1)
//368.64 MHz x6 (737.28 MHz) /2 (1)
extern uint32_t boot_cfg_base;
int DFESS_ProgPll_csl(unsigned int clkr, unsigned int clkf, unsigned int clkod,unsigned int divmode)
{
//    volatile unsigned int *dfepllctl0 = mkptr(CHIP_LEVEL_REG,DFEPLLCTL0);
//    volatile unsigned int *dfepllctl1 = mkptr(CHIP_LEVEL_REG,DFEPLLCTL1);
//    volatile unsigned int *dfe_clkdiv_ctl = mkptr(CHIP_LEVEL_REG,DFE_CLKDIV_CTL);
//    volatile unsigned int *dfe_clksync_ctl = mkptr(CHIP_LEVEL_REG,DFE_CLKSYNC_CTL);

    // check disallowed here.
    if (clkod && ((clkod & 0x1) != 0x1)) return(-1);
    // Check some unimplemented (yet) features.
    if (clkr) return(-1);
    if (clkf > 63) return(-1);

    /* Unlock Boot Config Registers */
    CSL_BtCfgUnLockKicker(boot_cfg_base);
//    KICK0 = KICK0_UNLOCK;
//    KICK1 = KICK1_UNLOCK;

    // Reset DFEPLLCTL0, DFEPLLCTL1, DFE_CLKDIV_CTL, DFE_CLKSYNC_CTL
//    CSL_BtCfgInitDfePll(boot_cfg_base);
 //   CSL_BtCfgInitDfeClk(boot_cfg_base);

    /*The Main PLL and PLL Controller must always be initialized prior to initializing the
    DFE PLL. The sequence shown below must be followed to initialize the DFE PLL.*/

    // 1. In DFEPLLCTL1, write ENSAT = 1 (for optimal PLL operation)
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL1), 6, 6, (uint32_t)0x1);

    // 2. In DFEPLLCTL0, write BYPASS = 1 (set the PLL in Bypass)
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL0), 23, 23, (uint32_t)0x1);

    // 3. Program PLLM and PLLD inthe DFEPLLCTL0 register
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL0), 11, 6, clkf);
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL0), 22, 19, clkod);

    // 4. Program BWADJ[7:0] in DFEPLLCTL0 and BWADJ[11:8] in the
    //DFEPLLCTL1 register. BWADJ value mustbe programmed to a value equal to
    //half of PLLM[12:0] value (round down if PLLM has an odd value) Example: If
    //PLLM = 15, then BWADJ = 7
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL0), 31, 24, clkf/2);

    // 5. In DFEPLLCTL1, write PLLRST = 1 (PLL is asserted)
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL1), 14, 14, (uint32_t)0x1);

    // 6. Wait for at least 5 µs based on the reference clock (PLL reset time)
    sleep(5000);
    // usleep(500);

    // 7. In DFEPLLCTL1, write PLLSELECT = 1 (for selecting the output of DFE PLL
    // as the input to DFE)
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL1), 13, 13, (uint32_t)0x1);

    // Extra: Set Divmode in DFE_CLKDIV_CTL
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFE_CLKDIV_CTL), 1, 0, divmode);

    // 8. In DFEPLLCTL1, write PLLRST = 0 (PLL reset is de-asserted)
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL1), 14, 14, (uint32_t)0x0);

    // 9. Wait for at least 500 * REFCLK cycles * (PLLD + 1) (PLL lock time)
    // usleep(500);
    sleep(5000);

    // 10. In DFEPLLCTL0, write BYPASS = 0 (switch to PLL mode)
    *(volatile uint32_t *)(boot_cfg_base + DFE_CLKSYNC_CTL) = 0x00000001;
    CSL_FINSR(*(volatile uint32_t *)(boot_cfg_base + DFEPLLCTL0), 23, 23, (uint32_t)0x0);

    // 11. PASS PLL is now initialized

    /* Lock Boot Config Registers */
    // Uboot unlocks the kicker and Linux expects the kicker to be unlocked
    //CSL_BtCfgLockKicker(boot_cfg_base);
    // KICK0 = KICK_LOCK;
    // KICK1 = KICK_LOCK;

    return(PASS);
}

// DFE internal SYSREF counter
void DFESS_ProgDfeTestCntr(Uint32 testCntr)
{
	//volatile unsigned int *dfe_test_cntr = mkptr(CHIP_LEVEL_REG,DFE_TEST_CNTR);
	volatile unsigned int *dfe_test_cntr = (volatile unsigned int *)(boot_cfg_base + DFE_TEST_CNTR);

	/* Unlock Boot Config Registers */
    KICK0 = KICK0_UNLOCK;
    KICK1 = KICK1_UNLOCK;

    *dfe_test_cntr = testCntr;

    /* Lock Boot Config Registers */
    // Uboot unlocks the kicker and Linux expects the kicker to be unlocked
    // KICK0 = KICK_LOCK;
    // KICK1 = KICK_LOCK;
}
