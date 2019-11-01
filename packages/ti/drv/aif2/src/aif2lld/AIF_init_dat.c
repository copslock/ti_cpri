/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
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

/* Include files for CSL ................................................... */
#include <ti/csl/csl.h>
#include <ti/drv/aif2/aif2fl.h>
#include <ti/csl/soc.h>

#include <ti/drv/aif2/AIF_init_dat.h>

/* Global variable declarations ............................................ */
#ifdef _TMS320C6X
#pragma DATA_SECTION(aifFsyncInitDone, ".far:aifDriver");
#pragma DATA_SECTION(aifFsyncEventCount, ".far:aifDriver");
#pragma DATA_SECTION(aif2DioIntCount, ".far:aifDriver");
#pragma DATA_SECTION(aifRegs, ".far:aifDriver");
#endif

/** Initialization flag */
uint32_t aifFsyncInitDone = 0;
uint32_t aifInitDone = 0;

/** FSYNC event and error counts */
volatile int32_t aifFsyncEventCount[30];

/* DIO engine interrupt counter */
volatile uint32_t aif2DioIntCount[AIF_MAX_NUM_LINKS];

/** AIF registers */
#ifndef K2
CSL_Aif2Regs * 	aifRegs= (CSL_Aif2Regs *)CSL_AIF2_CONTROL_REGS;
#else
CSL_Aif2Regs * 	aifRegs= (CSL_Aif2Regs *)0x01F00000;
#endif

/////////////////







