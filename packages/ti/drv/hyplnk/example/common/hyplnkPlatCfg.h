/*
 *
 * Copyright (C) 2010-2016 Texas Instruments Incorporated - http://www.ti.com/ 
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

/* ================================================================= */
/*  file  hyplnkPlatCfg.h
 *
 * This contains the definitions required to abstract out different
 * platforms including different chips and different EVMs.
 *
 */
#ifndef _HYPLNKPLATCFG_H
#define _HYPLNKPLATCFG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ti/csl/cslr_device.h"

#ifndef __ARMv7
extern cregister volatile unsigned int DNUM;
#endif 

/* For each known chip, pick the interrupt to be used, and the
 * EVM's hyperlink reference clock speed.
 *
 * The following defines are used:
 *
 * hyplnk_EXAMPLE_COREPAC_INT_INPUT: Selects which event is presented to the
 *   CorePac's internal INTC.
 *
 * hyplnk_EXAMPLE_INTC_OUTPUT: Selects which output of chip level INTC 
 *   (CP_INTC) will be used.  Note that this is hard wired to the CorePac
 *   event specified in hyplnk_EXAMPLE_COREPAC_INT_INPUT.  However, the
 *   number (INTC output vs CorePac input) are not the same.  These
 *   are specified in the chip data sheet.
 *
 * hyplnk_EXAMPLE_COREPAC_VEC: Selects which CorePac interrupt vector
 *   is used.  This is purely software selectable.  This is the output
 *   of the CorePac's INTC and is in the range 4-15
 *
 * hyplnk_EXAMPLE_NUM_CORES: Selects number of cores to use on
 *   multicore examples.  It could be defined lower than reality to
 *   run fewer cores.
 *
 * hyplnk_EXAMPLE_CPU_SPEED_MHZ: Identifies core's operating speed.  It is 
 *   OK if this is off by 20%.  It is only used to scale timing loops.
 *
 * hyplnk_EXAMPLE_HYPLINK_REF_KHZ: Identfies the reference clock on
 *   the broad market EVM for the device.  On other hardware implementations
 *   this will need to be changed (by the user) in hyplnkLLDCfg.h.  The
 *   speed must be exact.  Only discrete speeds like 156250, 250000, 
 *   and 312500 are supported. 
 */
#if defined(_TCI6614_Atrenta_DSP1_H_)
  /* tci6614 */
  #define hyplnk_EXAMPLE_INTC_OUTPUT       (64 + 10 * DNUM)
  #define hyplnk_EXAMPLE_COREPAC_INT_INPUT CSL_GEM_INTC0_OUT_64_PLUS_10_MUL_N
  #define hyplnk_EXAMPLE_COREPAC_VEC       CSL_INTC_VECTID_4
  #define hyplnk_EXAMPLE_NUM_CORES         4
  #define hyplnk_EXAMPLE_CPU_SPEED_MHZ     1200
  #define hyplnk_EXAMPLE_HYPLNK_REF_KHZ    312500
#elif defined(_C6657_Atrenta_DSP1_H_) || defined(SOC_C6657)
  /* c6657 */
  #define hyplnk_EXAMPLE_INTC_OUTPUT       (0 + 20 * DNUM)
  #define hyplnk_EXAMPLE_COREPAC_INT_INPUT CSL_GEM_INTC0_OUT_0_PLUS_20_MUL_N
  #define hyplnk_EXAMPLE_COREPAC_VEC       CSL_INTC_VECTID_4
  #define hyplnk_EXAMPLE_NUM_CORES         2
  #define hyplnk_EXAMPLE_CPU_SPEED_MHZ     800
  #define hyplnk_EXAMPLE_HYPLNK_REF_KHZ    250000
#elif defined(_C6678_Atrenta_DSP1_H_) || defined(_TCI6608_Atrenta_DSP1_H_) || defined(SOC_C6678)
  /* c6678, tci6608 */
  #define hyplnk_EXAMPLE_INTC_OUTPUT       (32 + 11 * DNUM)
  #define hyplnk_EXAMPLE_COREPAC_INT_INPUT CSL_GEM_INTC0_OUT_32_PLUS_11_MUL_N_OR_INTC0_OUT_32_PLUS_11_MUL_N_MINUS_4
  #define hyplnk_EXAMPLE_COREPAC_VEC       CSL_INTC_VECTID_4
  #define hyplnk_EXAMPLE_NUM_CORES         8
  #define hyplnk_EXAMPLE_CPU_SPEED_MHZ     1000
  #define hyplnk_EXAMPLE_HYPLNK_REF_KHZ    312500
#elif defined(_C6670_Atrenta_DSP1_H_) || defined(_TCI6616_Atrenta_DSP1_H_) || defined(_TCI6618_Atrenta_DSP1_H_)
  /* c6670, tci6616, tci6618 */
  #define hyplnk_EXAMPLE_INTC_OUTPUT       (64 + 10 * DNUM)
  #define hyplnk_EXAMPLE_COREPAC_INT_INPUT CSL_GEM_INTC0_OUT_64_PLUS_10_MUL_N
  #define hyplnk_EXAMPLE_COREPAC_VEC       CSL_INTC_VECTID_4
  #define hyplnk_EXAMPLE_NUM_CORES         4
  #define hyplnk_EXAMPLE_CPU_SPEED_MHZ     983
  #define hyplnk_EXAMPLE_HYPLNK_REF_KHZ    250000
#elif defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(SOC_K2K) || defined(SOC_K2H) 
  /* Temporarily set to same values as c6670, tci6616 and tci6618 */
  #define hyplnk_EXAMPLE_INTC_OUTPUT       (64 + 10 * DNUM)
  #define hyplnk_EXAMPLE_COREPAC_INT_INPUT CSL_C66X_COREPAC_CIC_OUT64_PLUS_10_MUL_N
  #define hyplnk_EXAMPLE_COREPAC_VEC       CSL_INTC_VECTID_4
  #define hyplnk_EXAMPLE_NUM_CORES         4
  #define hyplnk_EXAMPLE_CPU_SPEED_MHZ     983
  #define hyplnk_EXAMPLE_HYPLNK_REF_KHZ    312500
  //#define hyplnk_EXAMPLE_HYPLNK_REF_KHZ    156250
#elif defined(DEVICE_K2E) || defined(SOC_K2E)
  #define hyplnk_EXAMPLE_INTC_OUTPUT       (68)
  #define hyplnk_EXAMPLE_COREPAC_INT_INPUT CSL_C66X_COREPAC_CIC_0_OUT68
  #define hyplnk_EXAMPLE_COREPAC_VEC       CSL_INTC_VECTID_4
  #define hyplnk_EXAMPLE_NUM_CORES         1
  #define hyplnk_EXAMPLE_CPU_SPEED_MHZ     983
  #define hyplnk_EXAMPLE_HYPLNK_REF_KHZ    312500
  //#define hyplnk_EXAMPLE_HYPLNK_REF_KHZ    156250
#else
/* The cslr_device.h doesn't provide a known device identifier define */
#error Unknown device identified.
#endif


#ifdef __cplusplus
}
#endif

#endif  /* _HYPLNKPLATCFG_H */


