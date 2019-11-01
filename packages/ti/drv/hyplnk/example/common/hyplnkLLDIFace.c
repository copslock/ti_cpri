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

/*  
 * This file contains the prototypes for functions that interface with 
 * Hyperlink and the Hyperlink LLD.  They are common among all tests/examples.
 */

#include <ti/drv/hyplnk/hyplnk.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_bootcfgAux.h>
#include "hyplnkResource.h"

#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(DEVICE_K2E) || defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_K2E)
#include <ti/csl/csl_device_interrupt.h>
#include <ti/csl/csl_serdes_hyperlink.h>
#include <ti/csl/csl_serdes_restore_default.h>
#endif /* DEVICE_K2K || DEVICE_K2H || DEVICE_K2E || defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_K2E)*/

#include "hyplnkLLDIFace.h"
#include "hyplnkPlatCfg.h"

/* Pick parameters that changed based on loopback (loopback is faster) */
#ifdef hyplnk_EXAMPLE_LOOPBACK
  #define hyplnk_EXAMPLE_ILOOP_BIT 1
  #define hyplnk_EXAMPLE_SYNC_WAIT_LIMIT          hyplnk_EXAMPLE_uS_TO_CYCLES(100000)
#else
  #define hyplnk_EXAMPLE_ILOOP_BIT 0
  #define hyplnk_EXAMPLE_SYNC_WAIT_LIMIT          hyplnk_EXAMPLE_uS_TO_CYCLES( 1000000)
#endif

#define hyplnk_EXAMPLE_WAIT_STABILITY_TIME    hyplnk_EXAMPLE_uS_TO_CYCLES( 5000000)
#define hyplnk_EXAMPLE_SERIAL_STOP_WAIT_ITER  100000

/* Convert REFCLK_USE_PLATCFG to a refclk */
#ifdef hyplnk_EXAMPLE_REFCLK_USE_PLATCFG
  #if defined(hyplnk_EXAMPLE_REFCLK_156p25) || \
      defined(hyplnk_EXAMPLE_REFCLK_250p00) || \
      defined(hyplnk_EXAMPLE_REFCLK_312p50)
    #error Cannot define a specific REFCLK together with USE_PLATCFG
  #endif
  #if   (hyplnk_EXAMPLE_HYPLNK_REF_KHZ == 312500)
    #define hyplnk_EXAMPLE_REFCLK_312p50
  #elif (hyplnk_EXAMPLE_HYPLNK_REF_KHZ == 250000)
    #define hyplnk_EXAMPLE_REFCLK_250p00
  #elif (hyplnk_EXAMPLE_HYPLNK_REF_KHZ == 156250)
    #define hyplnk_EXAMPLE_REFCLK_156p25
  #else 
    #error Unknown value for hyplnk_EXAMPLE_HYPLNK_REF_KHZ
  #endif
#endif

/* Check REFCLK definition */
#if defined(hyplnk_EXAMPLE_REFCLK_156p25)
  #if defined(hyplnk_EXAMPLE_REFCLK_250p00) || \
      defined(hyplnk_EXAMPLE_REFCLK_312p50)
    #error Exactly one REFCLK must be defined (#1)
  #endif
#elif defined(hyplnk_EXAMPLE_REFCLK_250p00)
  #if defined(hyplnk_EXAMPLE_REFCLK_156p25) || \
      defined(hyplnk_EXAMPLE_REFCLK_312p50)
    #error Exactly one REFCLK must be defined (#2)
  #endif
#elif defined(hyplnk_EXAMPLE_REFCLK_312p50)
  #if defined(hyplnk_EXAMPLE_REFCLK_156p25) || \
      defined(hyplnk_EXAMPLE_REFCLK_250p00)
    #error Exactly one REFCLK must be defined (#3)
  #endif
#else
  #error Exactly one REFCLK must be defined (#4)
#endif

/* Check SERRATE definition */
#if defined(hyplnk_EXAMPLE_SERRATE_01p250)
  #if defined(hyplnk_EXAMPLE_SERRATE_03p125) || \
      defined(hyplnk_EXAMPLE_SERRATE_06p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_07p500) || \
      defined(hyplnk_EXAMPLE_SERRATE_10p000) || \
      defined(hyplnk_EXAMPLE_SERRATE_12p500)
    #error Exactly one SERRATE must be defined (#1)
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_03p125)
  #if defined(hyplnk_EXAMPLE_SERRATE_01p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_06p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_07p500) || \
      defined(hyplnk_EXAMPLE_SERRATE_10p000) || \
      defined(hyplnk_EXAMPLE_SERRATE_12p500)
    #error Exactly one SERRATE must be defined (#2)
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_06p250)
  #if defined(hyplnk_EXAMPLE_SERRATE_01p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_03p125) || \
      defined(hyplnk_EXAMPLE_SERRATE_07p500) || \
      defined(hyplnk_EXAMPLE_SERRATE_10p000) || \
      defined(hyplnk_EXAMPLE_SERRATE_12p500)
    #error Exactly one SERRATE must be defined (#3)
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_07p500)
  #if defined(hyplnk_EXAMPLE_SERRATE_01p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_03p125) || \
      defined(hyplnk_EXAMPLE_SERRATE_06p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_10p000) || \
      defined(hyplnk_EXAMPLE_SERRATE_12p500)
    #error Exactly one SERRATE must be defined (#4)
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_10p000)
  #if defined(hyplnk_EXAMPLE_SERRATE_01p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_03p125) || \
      defined(hyplnk_EXAMPLE_SERRATE_06p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_07p500) || \
      defined(hyplnk_EXAMPLE_SERRATE_12p500)
    #error Exactly one SERRATE must be defined (#5)
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_12p500)
  #if defined(hyplnk_EXAMPLE_SERRATE_01p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_03p125) || \
      defined(hyplnk_EXAMPLE_SERRATE_06p250) || \
      defined(hyplnk_EXAMPLE_SERRATE_07p500) || \
      defined(hyplnk_EXAMPLE_SERRATE_10p000)
    #error Exactly one SERRATE must be defined (#6)
  #endif
#else
  #error Exactly one SERRATE must be defined (#7)
#endif

/*****************************************************************************
 * These parameters can be changed for SERDES diagnostics
 *****************************************************************************/
#define hyplnk_EXAMPLE_VUSR_RX_RES      0
#define hyplnk_EXAMPLE_VUSR_RX_TESTPAT  0 /* disabled */
#define hyplnk_EXAMPLE_VUSR_RX_LOOPBACK 0 /* disabled */

#define hyplnk_EXAMPLE_VUSR_TX_RES      0
#define hyplnk_EXAMPLE_VUSR_TX_TESTPAT  0 /* disabled */
#define hyplnk_EXAMPLE_VUSR_TX_LOOPBACK 0 /* disabled */

#define hyplnk_EXAMPLE_VUSR_PLL_RES     0
#define hyplnk_EXAMPLE_VUSR_PLL_LOOPBW  0

/*****************************************************************************
 * These parameters should remain fixed.  They are defined by the physical
 * connection between the SERDES and the Hyperlink inside the device
 *****************************************************************************/
#define hyplnk_EXAMPLE_VUSR_RX_BUSWIDTH 2 /* 20 bits */
#define hyplnk_EXAMPLE_VUSR_RX_ENRX     1 /* enabled */
#define hyplnk_EXAMPLE_VUSR_RX_INVPAIR  0 /* normal */
#define hyplnk_EXAMPLE_VUSR_RX_TERM     1 /* 0.7V */
#define hyplnk_EXAMPLE_VUSR_RX_ALIGN    1 /* comma alignment */

#define hyplnk_EXAMPLE_VUSR_TX_FIRUPT   1 /* tie high */
#define hyplnk_EXAMPLE_VUSR_TX_BUSWIDTH 2 /* 20 bits */
#define hyplnk_EXAMPLE_VUSR_TX_ENTX     1 /* enable */
#define hyplnk_EXAMPLE_VUSR_TX_MSYNC    0 /* filled in by code below */
#define hyplnk_EXAMPLE_VUSR_TX_INVPAIR  0 /* normal */

#define hyplnk_EXAMPLE_VUSR_PLL_ENPLL   0
#define hyplnk_EXAMPLE_VUSR_PLL_SLEEP   0
#define hyplnk_EXAMPLE_VUSR_PLL_CLKBYP  0

/*****************************************************************************
 * These parameters do not depend on wire rate or reference clock
 *****************************************************************************/
#define hyplnk_EXAMPLE_VUSR_RX_ENOC     1 /* compensation enabled */
#define hyplnk_EXAMPLE_VUSR_RX_EQHLD    0 /* adapation allowed (0=on!) */
#define hyplnk_EXAMPLE_VUSR_RX_EQ       1 /* adaptive */
#define hyplnk_EXAMPLE_VUSR_RX_CDR      5 /* clock recovery */
#define hyplnk_EXAMPLE_VUSR_RX_LOS      4 /* enabled */

/*****************************************************************************
 * The remaining parameters vary by wire rate and reference clock
 *
 *  #define hyplnk_EXAMPLE_VUSR_RX_RATE
 *  #define hyplnk_EXAMPLE_VUSR_TX_RATE
 *
 *  #define hyplnk_EXAMPLE_VUSR_TX_TWPST1
 *  #define hyplnk_EXAMPLE_VUSR_TX_TWPRE
 *  #define hyplnk_EXAMPLE_VUSR_TX_SWIN
 *****************************************************************************/

/* Set PLL & SERDES configuration */
#if defined(hyplnk_EXAMPLE_SERRATE_01p250)
  #define hyplnk_EXAMPLE_VUSR_TX_TWPST1   0
  #define hyplnk_EXAMPLE_VUSR_TX_TWPRE    0
  #define hyplnk_EXAMPLE_VUSR_TX_SWING    0xf

  /* 1/8th rate implies MPY/2 below, vrange is 0 since 1.25*2 > 2.17 */
  #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE  0

  #define hyplnk_EXAMPLE_VUSR_RX_RATE     3 /* 1/8th rate */
  #define hyplnk_EXAMPLE_VUSR_TX_RATE     3 /* 1/8th */

  #if defined(hyplnk_EXAMPLE_REFCLK_156p25)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY   64  /* MPY in 0.25 units = 16x */
                                              /* 16/2 * 156.25 = 1.25G */
  #elif defined(hyplnk_EXAMPLE_REFCLK_250p00)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY   40  /* MPY in 0.25 units = 10x */
                                              /* 10/2 * 250 = 1.25G */
  #elif defined(hyplnk_EXAMPLE_REFCLK_312p50)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY   32  /* MPY in 0.25 units, = 8x */
                                              /* 8/2 * 312.5 = 1.25G */
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_03p125)
  #define hyplnk_EXAMPLE_VUSR_TX_TWPST1   19
  #define hyplnk_EXAMPLE_VUSR_TX_TWPRE    1
  #define hyplnk_EXAMPLE_VUSR_TX_SWING    0xf

  #if defined(hyplnk_EXAMPLE_REFCLK_156p25)
    /* 1/2 rate implies MPY*2 below, vrange is 1 since 3.125*0.5 < 2.17 */
    #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE 1
    #define hyplnk_EXAMPLE_VUSR_RX_RATE    1  /* 1/2th rate */
    #define hyplnk_EXAMPLE_VUSR_TX_RATE    1  /* 1/2th */
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    40 /* MPY in 0.25 units = 10x */
                                              /* 10*2 * 156.25 = 3.125G */

  #elif defined(hyplnk_EXAMPLE_REFCLK_250p00)
    /* 1/4 rate implies MPY*1 below, vrange is 0 since 3.125*1 > 2.17 */
    #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE 0
    #define hyplnk_EXAMPLE_VUSR_RX_RATE    2 /* 1/4th rate */
    #define hyplnk_EXAMPLE_VUSR_TX_RATE    2 /* 1/4th */
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    50 /* MPY in 0.25 units = 12.5x */
                                              /* 12.5*1 * 250 = 3.125G */

  #elif defined(hyplnk_EXAMPLE_REFCLK_312p50)
    /* 1/2 rate implies MPY*2 below, vrange is 1 since 3.125*0.5 < 2.17 */
    #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE 1
    #define hyplnk_EXAMPLE_VUSR_RX_RATE    1 /* 1/2th rate */
    #define hyplnk_EXAMPLE_VUSR_TX_RATE    1 /* 1/2th */
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    20 /* MPY in 0.25 units = 5x */
                                              /* 5*2 * 312.5 = 3.125G */
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_06p250)
  #define hyplnk_EXAMPLE_VUSR_TX_TWPST1   19 /* -7.5% */
  #define hyplnk_EXAMPLE_VUSR_TX_TWPRE    0  /* 0% */
  #define hyplnk_EXAMPLE_VUSR_TX_SWING    0x6

  #if defined(hyplnk_EXAMPLE_REFCLK_156p25)
    /* full rate implies MPY*4 below, vrange is 1 since 6.25*0.25 < 2.17 */
    #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE 1
    #define hyplnk_EXAMPLE_VUSR_RX_RATE    0  /* full rate */
    #define hyplnk_EXAMPLE_VUSR_TX_RATE    0  /* full */
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    40 /* MPY in 0.25 units = 10x */
                                              /* 10*4 * 156.25 = 6.25G */

  #elif defined(hyplnk_EXAMPLE_REFCLK_250p00)
    /* 1/2 rate implies MPY*2 below, vrange is 0 since 6.25*0.5 > 2.17 */
    #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE 0
    #define hyplnk_EXAMPLE_VUSR_RX_RATE    1 /* 1/2th rate */
    #define hyplnk_EXAMPLE_VUSR_TX_RATE    1 /* 1/2th */
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    50 /* MPY in 0.25 units = 12.5x */
                                              /* 12.5*2 * 250 = 6.25G */

  #elif defined(hyplnk_EXAMPLE_REFCLK_312p50)
    /* full rate implies MPY*4 below, vrange is 1 since 6.25*0.25 < 2.17 */
    #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE 1
    #define hyplnk_EXAMPLE_VUSR_RX_RATE    0  /* full rate */
    #define hyplnk_EXAMPLE_VUSR_TX_RATE    0  /* full */
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    20 /* MPY in 0.25 units = 5x */
                                              /* 5*4 * 312.5 = 6.25G */
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_07p500)
  #define hyplnk_EXAMPLE_VUSR_TX_TWPST1   19 /* -7.5% */
  #define hyplnk_EXAMPLE_VUSR_TX_TWPRE    0  /* 0% */
  #define hyplnk_EXAMPLE_VUSR_TX_SWING    0x6

  /* full rate implies MPY*4 below, vrange is 1 since 8.125*0.25 < 2.17 */
  #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE  0
  #define hyplnk_EXAMPLE_VUSR_RX_RATE     0  /* full rate */
  #define hyplnk_EXAMPLE_VUSR_TX_RATE     0  /* full */

  #if defined(hyplnk_EXAMPLE_REFCLK_156p25)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    48 /* MPY in 0.25 units = 12x */
                                              /* 12*4 * 156.25 = 7.5G */
  #elif defined(hyplnk_EXAMPLE_REFCLK_250p00)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    30 /* MPY in 0.25 units = 7.5x */
                                              /* 7.5*4 * 250 = 7.5G */
  #elif defined(hyplnk_EXAMPLE_REFCLK_312p50)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    24 /* MPY in 0.25 units = 6x */
                                              /* 6.5*4 * 312.5 = 7.5G */
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_10p000)
  #define hyplnk_EXAMPLE_VUSR_TX_TWPST1   20 /* -10% */
  #define hyplnk_EXAMPLE_VUSR_TX_TWPRE    1  /* -10.0% */
  #define hyplnk_EXAMPLE_VUSR_TX_SWING    0x8

  /* full rate implies MPY*4 below, vrange is 0 since 10.0*0.25 > 2.17 */
  #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE  0
  #define hyplnk_EXAMPLE_VUSR_RX_RATE     0  /* full rate */
  #define hyplnk_EXAMPLE_VUSR_TX_RATE     0  /* full */

  #if defined(hyplnk_EXAMPLE_REFCLK_156p25)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    80 /* MPY in 0.25 units = 16x */
                                              /* 16*4 * 156.25 = 10.0G */
  #elif defined(hyplnk_EXAMPLE_REFCLK_250p00)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    40 /* MPY in 0.25 units = 10x */
                                              /* 10*4 * 250 = 10.0G */
  #elif defined(hyplnk_EXAMPLE_REFCLK_312p50)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    32 /* MPY in 0.25 units = 8x */
                                              /* 8*4 * 312.5 = 10.0G */
  #endif
#elif defined(hyplnk_EXAMPLE_SERRATE_12p500)
  #define hyplnk_EXAMPLE_VUSR_TX_TWPST1   20 /* -10% */
  #define hyplnk_EXAMPLE_VUSR_TX_TWPRE    4  /* -10.0% */
  #define hyplnk_EXAMPLE_VUSR_TX_SWING    0x8

  /* full rate implies MPY*4 below, vrange is 0 since 12.5*0.25 > 2.17 */
  #define hyplnk_EXAMPLE_VUSR_PLL_VRANGE  0
  #define hyplnk_EXAMPLE_VUSR_RX_RATE     0  /* full rate */
  #define hyplnk_EXAMPLE_VUSR_TX_RATE     0  /* full */

  #if defined(hyplnk_EXAMPLE_REFCLK_156p25)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    80 /* MPY in 0.25 units = 20x */
                                              /* 20*4 * 156.25 = 12.5G */
  #elif defined(hyplnk_EXAMPLE_REFCLK_250p00)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    50 /* MPY in 0.25 units = 12.5x */
                                              /* 12.5*4 * 250 = 12.5G */
  #elif defined(hyplnk_EXAMPLE_REFCLK_312p50)
    #define hyplnk_EXAMPLE_VUSR_PLL_MPY    40 /* MPY in 0.25 units = 10x */
                                              /* 10*4 * 312.5 = 12.5G */
  #endif
#endif


#define  hyplnk_EXAMPLE_VUSR_PLL                  \
        (hyplnk_EXAMPLE_VUSR_PLL_RES     << 15) | \
        (hyplnk_EXAMPLE_VUSR_PLL_CLKBYP  << 13) | \
        (hyplnk_EXAMPLE_VUSR_PLL_LOOPBW  << 11) | \
        (hyplnk_EXAMPLE_VUSR_PLL_SLEEP   << 10) | \
        (hyplnk_EXAMPLE_VUSR_PLL_VRANGE  <<  9) | \
        (hyplnk_EXAMPLE_VUSR_PLL_MPY     <<  1) | \
        (hyplnk_EXAMPLE_VUSR_PLL_ENPLL   <<  0)


#define  hyplnk_EXAMPLE_VUSR_RX_CONFIG            \
        (hyplnk_EXAMPLE_VUSR_RX_RES      << 28) | \
        (hyplnk_EXAMPLE_VUSR_RX_TESTPAT  << 25) | \
        (hyplnk_EXAMPLE_VUSR_RX_LOOPBACK << 23) | \
        (hyplnk_EXAMPLE_VUSR_RX_ENOC     << 22) | \
        (hyplnk_EXAMPLE_VUSR_RX_EQHLD    << 21) | \
        (hyplnk_EXAMPLE_VUSR_RX_EQ       << 18) | \
        (hyplnk_EXAMPLE_VUSR_RX_CDR      << 15) | \
        (hyplnk_EXAMPLE_VUSR_RX_LOS      << 12) | \
        (hyplnk_EXAMPLE_VUSR_RX_ALIGN    << 10) | \
        (hyplnk_EXAMPLE_VUSR_RX_TERM     <<  7) | \
        (hyplnk_EXAMPLE_VUSR_RX_INVPAIR  <<  6) | \
        (hyplnk_EXAMPLE_VUSR_RX_RATE     <<  4) | \
        (hyplnk_EXAMPLE_VUSR_RX_BUSWIDTH <<  1) | \
        (hyplnk_EXAMPLE_VUSR_RX_ENRX     <<  0)

#define  hyplnk_EXAMPLE_VUSR_TX_CONFIG            \
        (hyplnk_EXAMPLE_VUSR_TX_RES      << 26) | \
        (hyplnk_EXAMPLE_VUSR_TX_TESTPAT  << 23) | \
        (hyplnk_EXAMPLE_VUSR_TX_LOOPBACK << 21) | \
        (hyplnk_EXAMPLE_VUSR_TX_MSYNC    << 20) | \
        (hyplnk_EXAMPLE_VUSR_TX_FIRUPT   << 19) | \
        (hyplnk_EXAMPLE_VUSR_TX_TWPST1   << 14) | \
        (hyplnk_EXAMPLE_VUSR_TX_TWPRE    << 11) | \
        (hyplnk_EXAMPLE_VUSR_TX_SWING    <<  7) | \
        (hyplnk_EXAMPLE_VUSR_TX_INVPAIR  <<  6) | \
        (hyplnk_EXAMPLE_VUSR_TX_RATE     <<  4) | \
        (hyplnk_EXAMPLE_VUSR_TX_BUSWIDTH <<  1) | \
        (hyplnk_EXAMPLE_VUSR_TX_ENTX     <<  0)

#ifdef hyplnk_EXAMPLE_ALLOW_4_LANES
  #define hyplnk_EXAMPLE_MAX_LANES 4
#else
  #define hyplnk_EXAMPLE_MAX_LANES 1
#endif


#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(DEVICE_K2E) || defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_K2E)
void hyplnkExampleDefSerdesSetup()
{
    CSL_SERDES_REF_CLOCK          refClock;
    CSL_SERDES_LINK_RATE          linkRate;
    uint32_t                      baseAddr, i;
    CSL_SERDES_RESULT             csl_retval;
    CSL_SERDES_LANE_CTRL_RATE     lane_rate;
    
    CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params;
    CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;

    memset(&serdes_lane_enable_params, 0, sizeof(serdes_lane_enable_params));

#if (hyplnk_EXAMPLE_HYPLNK_REF_KHZ == 312500)
    refClock = CSL_SERDES_REF_CLOCK_312p5M;

#ifdef hyplnk_EXAMPLE_SERRATE_12p500
    linkRate = CSL_SERDES_LINK_RATE_12p5G;
    lane_rate = CSL_SERDES_LANE_FULL_RATE;
#elif defined hyplnk_EXAMPLE_SERRATE_06p250
    linkRate = CSL_SERDES_LINK_RATE_6p25G;
    lane_rate = CSL_SERDES_LANE_FULL_RATE;
#elif defined hyplnk_EXAMPLE_SERRATE_03p125
    linkRate = CSL_SERDES_LINK_RATE_6p25G;
    lane_rate = CSL_SERDES_LANE_HALF_RATE;
#else
    #error Unsupported Link Rate
#endif /* Link Rate */

#elif (hyplnk_EXAMPLE_HYPLNK_REF_KHZ == 156250)
    refClock = CSL_SERDES_REF_CLOCK_156p25M;

#ifdef hyplnk_EXAMPLE_SERRATE_06p250
    linkRate = CSL_SERDES_LINK_RATE_6p25G;
    lane_rate = CSL_SERDES_LANE_FULL_RATE;
#elif defined hyplnk_EXAMPLE_SERRATE_03p125
    linkRate = CSL_SERDES_LINK_RATE_6p25G;
    lane_rate = CSL_SERDES_LANE_HALF_RATE;
#else
    #error Unsupported Link Rate
#endif /* Link Rate */

#else
    #error Unsupported SB SERDES Config
#endif

    if(hyplnk_EXAMPLE_PORT ==0)
    {
        baseAddr = hyplnk_mmap(CSL_HYPERLINK_0_SERDES_CFG_REGS,0x2000);
    }
#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(SOC_K2K) || defined(SOC_K2H)
    else
    {
        baseAddr = hyplnk_mmap(CSL_HYPERLINK_1_SERDES_CFG_REGS,0x2000);
    }
#endif
    
    serdes_lane_enable_params.base_addr = baseAddr;
    serdes_lane_enable_params.ref_clock = refClock;
    serdes_lane_enable_params.linkrate = linkRate;
    serdes_lane_enable_params.num_lanes = hyplnk_EXAMPLE_MAX_LANES;
    serdes_lane_enable_params.phy_type = SERDES_HYPERLINK;
    for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
    {
        serdes_lane_enable_params.lane_ctrl_rate[i] = lane_rate;

        /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
        serdes_lane_enable_params.rx_coeff.att_start[i] = 7;
        serdes_lane_enable_params.rx_coeff.boost_start[i] = 5;

        /* For higher speeds PHY-A, force attenuation and boost values  */
        serdes_lane_enable_params.rx_coeff.force_att_val[i] = 1;
        serdes_lane_enable_params.rx_coeff.force_boost_val[i] = 1;

        /* CM, C1, C2, Att and Vreg are obtained through Serdes Diagnostic BER test */
        serdes_lane_enable_params.tx_coeff.cm_coeff[i] = 0;
        serdes_lane_enable_params.tx_coeff.c1_coeff[i] = 0;
        serdes_lane_enable_params.tx_coeff.c2_coeff[i] = 0;
        serdes_lane_enable_params.tx_coeff.tx_att[i] = 12;
        serdes_lane_enable_params.tx_coeff.tx_vreg[i] = 4;
    }
    serdes_lane_enable_params.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;

    /* Att and Boost values are obtained through Serdes Diagnostic PRBS calibration test */
    /* For higher speeds PHY-A, force attenuation and boost values  */
    serdes_lane_enable_params.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

#ifndef hyplnk_EXAMPLE_LOOPBACK
    for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
    {
        serdes_lane_enable_params.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
    }
#else
    for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
    {
        serdes_lane_enable_params.loopback_mode[i] = CSL_SERDES_LOOPBACK_ENABLED;
    }
#endif
    serdes_lane_enable_params.lane_mask = 0xF;

    csl_retval = CSL_HyperlinkSerdesInit(baseAddr, 
                                        refClock, 
                                        linkRate);
    if (csl_retval != 0)
    {
        printf ("Invalid Serdes Init Params: %d\n", csl_retval);
    }

    /* Common Init Mode */
    /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
    /* The lane_mask is a don't care for Common Init as it operates on all lanes */
    serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
    serdes_lane_enable_params.lane_mask = 0xF;
    lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
    if (lane_retval != 0)
    {
        printf ("Invalid Serdes Common Init %d\n", lane_retval);
        exit(0);
    }
    printf("Hyperlink Serdes Common Init Complete\n");

    /* Lane Init Mode */
    /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be enabled by setting
     * iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
    /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 2 is being configured, lane mask needs to be 0x4 etc */
    serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
    for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
    {
        serdes_lane_enable_params.lane_mask = 1<<i;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
        if (lane_retval != 0)
        {
            printf ("Invalid Serdes Lane Enable Init %d\n", lane_retval);
            exit(0);
        }
        printf("Hyperlink Serdes Lane %d Init Complete\n", i);
    }

}
#endif

/*****************************************************************************
 * Wait # of CPU cycles (excluding function call and loop entry/exit overhead)
 ****************************************************************************/
void hyplnkExampleDelay (uint32_t cycles)
{
#ifdef __ARMv7
    volatile uint32_t sat;
    if (cycles <= 0) return;
    for (sat=0; sat < cycles; sat++);
#else
    uint32_t now = TSCL;
    while ((TSCL - now) < cycles);
#endif
}

/*****************************************************************************
 * Break down and print the revision register.
 ****************************************************************************/
void hyplnkExamplePrintRevReg (hyplnkRevReg_t *rev) {
#if (hyplnk_EXAMPLE_PRINT_REV_REG >= hyplnk_EXAMPLE_PRINT_RAW)
  System_printf ("Revision register contents:\n"
                 "  Raw    = 0x%08x\n"
#if (hyplnk_EXAMPLE_PRINT_REV_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
                 "  Scheme = %d\n"
                 "  BU     = %d\n"
                 "  Func   = 0x%04x\n"
                 "  RTL    = %d\n"
                 "  cust   = %d\n"
                 "  revMaj = %d\n"
                 "  revMin = %d\n"
#endif
               , rev->raw
#if (hyplnk_EXAMPLE_PRINT_REV_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
               , rev->scheme, 
                 rev->bu, 
                 rev->func, 
                 rev->rtl, 
                 rev->cust, 
                 rev->revMaj, 
                 rev->revMin
#endif
                );
#endif
}

/*****************************************************************************
 * Break down and print the status register.
 ****************************************************************************/
void hyplnkExamplePrintStatusReg (hyplnkStatusReg_t *status) {
#if (hyplnk_EXAMPLE_PRINT_STATUS_REG >= hyplnk_EXAMPLE_PRINT_RAW)
  System_printf ("Status register contents:\n"
                 "  Raw        = 0x%08x\n"
#if (hyplnk_EXAMPLE_PRINT_STATUS_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
                 "  swidthin   = %d\n"
                 "  swidthout  = %d\n"
                 "  serialHalt = %d\n"
                 "  pllUnlock  = %d\n"
                 "  rPend      = %d\n"
                 "  iFlow      = %d\n"
                 "  oFlow      = %d\n"
                 "  rError     = %d\n"
                 "  lError     = %d\n"
                 "  nfEmpty3   = %d\n"
                 "  nfEmpty2   = %d\n"
                 "  nfEmpty1   = %d\n"
                 "  nfEmpty0   = %d\n"
                 "  sPend      = %d\n"
                 "  mPend      = %d\n"
                 "  link       = %d\n"
#endif
               , status->raw
#if (hyplnk_EXAMPLE_PRINT_STATUS_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
               , status->swidthin,
                 status->swidthout,
                 status->serialHalt,
                 status->pllUnlock,
                 status->rPend,
                 status->iFlow,
                 status->oFlow,
                 status->rError,
                 status->lError,
                 status->nfEmpty3,
                 status->nfEmpty2,
                 status->nfEmpty1,
                 status->nfEmpty0,
                 status->sPend,
                 status->mPend,
                 status->link
#endif
                );
#endif
}

/*****************************************************************************
 * Break down and print the link status register.
 ****************************************************************************/
void hyplnkExamplePrintLinkStatusReg (hyplnkLinkStatusReg_t *status) {
#if (hyplnk_EXAMPLE_PRINT_LINK_STATUS_REG >= hyplnk_EXAMPLE_PRINT_RAW)
  System_printf ("Link status register contents:\n"
                 "  Raw       = 0x%08x\n"
#if (hyplnk_EXAMPLE_PRINT_LINK_STATUS_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
                 "  txPlsReq  = %d\n"
                 "  txPlsAck  = %d\n"
                 "  txPmReq   = %d\n"
                 "  txRSync   = %d\n"
                 "  txPlsOK   = %d\n"
                 "  txPhyEn   = %d\n"
                 "  txFlowSts = %d\n"
                 "  rxPlsReq  = %d\n"
                 "  rxPlsAck  = %d\n"
                 "  rxPmReq   = %d\n"
                 "  rxLSync   = %d\n"
                 "  rxPhyEn   = %d\n"
                 "  rxPhyPol  = %d\n"
#endif
               , status->raw
#if (hyplnk_EXAMPLE_PRINT_LINK_STATUS_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
               , status->txPlsReq,
                 status->txPlsAck,
                 status->txPmReq,
                 status->txRSync,
                 status->txPlsOK,
                 status->txPhyEn,
                 status->txFlowSts,
                 status->rxPlsReq,
                 status->rxPlsAck,
                 status->rxPmReq,
                 status->rxLSync,
                 status->rxPhyEn,
                 status->rxPhyPol
#endif
                );
#endif
}

/*****************************************************************************
 * Break down and print the control register.
 ****************************************************************************/
void hyplnkExamplePrintControlReg (hyplnkControlReg_t *control) {
#if (hyplnk_EXAMPLE_PRINT_CONTROL_REG >= hyplnk_EXAMPLE_PRINT_RAW)
  System_printf ("Control register contents:\n"
                 "  Raw             = 0x%08x\n"
#if (hyplnk_EXAMPLE_PRINT_CONTROL_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
                 "  intLocal        = %d\n"
                 "  statusIntEnable = %d\n"
                 "  statusIntVec    = %d\n"
                 "  int2cfg         = %d\n"
                 "  serialStop      = %d\n"
                 "  iLoop           = %d\n"
                 "  reset           = %d\n"
#endif
               , control->raw
#if (hyplnk_EXAMPLE_PRINT_CONTROL_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
               , control->intLocal,
                 control->statusIntEnable,
                 control->statusIntVec,
                 control->int2cfg,
                 control->serialStop,
                 control->iLoop,
                 control->reset
#endif
                );
#endif
}

/*****************************************************************************
 * Break down and print the ECC errors register.
 ****************************************************************************/
void hyplnkExamplePrintECCErrorsReg (hyplnkECCErrorsReg_t *errors) {
#if (hyplnk_EXAMPLE_PRINT_ECC_ERRORS_REG >= hyplnk_EXAMPLE_PRINT_RAW)
  System_printf ("Control register contents:\n"
                 "  Raw        = 0x%08x\n"
#if (hyplnk_EXAMPLE_PRINT_ECC_ERRORS_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
                 "  sglErrCor  = %d\n"
                 "  dblErrDet  = %d\n"
#endif
               , errors->raw
#if (hyplnk_EXAMPLE_PRINT_ECC_ERRORS_REG >= hyplnk_EXAMPLE_PRINT_DETAILS)
               , errors->sglErrCor,
                 errors->dblErrDet
#endif
                );
#endif
}

void hyplnkExamplePrintOneSerdesStatus (uint32_t sts, int n)
{
#if (hyplnk_EXAMPLE_SERDES_STS_REGS >= hyplnk_EXAMPLE_PRINT_RAW)
  System_printf ("SERDES_STS%d:\n"
                 "  raw = 0x%08x\n"
#if (hyplnk_EXAMPLE_SERDES_STS_REGS >= hyplnk_EXAMPLE_PRINT_DETAILS)
               , "  Tx Tesfail = %d\n"
                 "  EQover     = %d\n"
                 "  EQunder    = %d\n"
                 "  OCIP       = %d\n"
                 "  Losdtct    = %d\n"
                 "  Sync       = %d\n"
                 "  Rx Tesfail = %d\n"
#endif
               , n,
                 sts
#if (hyplnk_EXAMPLE_SERDES_STS_REGS >= hyplnk_EXAMPLE_PRINT_DETAILS)
                 (sts >> 6) & 1,
                 (sts >> 5) & 1,
                 (sts >> 4) & 1,
                 (sts >> 3) & 1,
                 (sts >> 2) & 1,
                 (sts >> 1) & 1,
                 (sts >> 0) & 1
#endif
                );
#endif
}

uint32_t hyplnkExamplePrintSerdesStatus (uint32_t lastStatus)
{
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E)
  uint32_t thisStatus = hBootCfg->STS_VUSR;
  if (thisStatus == lastStatus) {
    return lastStatus;
  }

  System_printf ("SERDES_STS (32 bits) contents: 0x%08x; lock = %d\n", thisStatus, thisStatus & 1);
  hyplnkExamplePrintOneSerdesStatus((thisStatus >> 1) & 0x7F, 0);
  hyplnkExamplePrintOneSerdesStatus((thisStatus >> 8) & 0x7F, 1);
  hyplnkExamplePrintOneSerdesStatus((thisStatus >> 15) & 0x7F, 2);
  hyplnkExamplePrintOneSerdesStatus((thisStatus >> 22) & 0x7F, 3);

  return thisStatus;
#else /*  !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E) */
  return lastStatus;
#endif /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E) */
}

/*****************************************************************************
 * Place peripheral into reset
 ****************************************************************************/
hyplnkRet_e hyplnkExampleAssertReset (int val)
{
  hyplnkControlReg_t control;
  hyplnkRegisters_t  setRegs;
  Hyplnk_Handle      handle = NULL;
  hyplnkRet_e        retVal;

  memset (&setRegs, 0, sizeof(setRegs));
  setRegs.control        = &control;

  if ((retVal = Hyplnk_open(hyplnk_EXAMPLE_PORT, &handle)) != hyplnk_RET_OK) {
    System_printf ("Open failed\n");
    return retVal;
  }
  if ((retVal = Hyplnk_readRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs)) != hyplnk_RET_OK) {
    System_printf ("Read failed!\n");
    return retVal;
  }
  control.reset = val;
  if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs)) != hyplnk_RET_OK) {
    System_printf ("Write failed!\n");
    return retVal;
  }
  if ((retVal = Hyplnk_close (&handle)) != hyplnk_RET_OK) {
    System_printf ("close failed!\n");
    return retVal;
  }
  return hyplnk_RET_OK;
}

/*****************************************************************************
 * Sets the SERDES configuration registers
 ****************************************************************************/
void hyplnkExampleSerdesCfg (uint32_t rx, uint32_t tx)
{
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E)
  CSL_BootCfgUnlockKicker();
  CSL_BootCfgSetVUSRRxConfig (0, rx);
  CSL_BootCfgSetVUSRRxConfig (1, rx);
  CSL_BootCfgSetVUSRRxConfig (2, rx);
  CSL_BootCfgSetVUSRRxConfig (3, rx);

  CSL_BootCfgSetVUSRTxConfig (0, tx);
  CSL_BootCfgSetVUSRTxConfig (1, tx);
  CSL_BootCfgSetVUSRTxConfig (2, tx);
  CSL_BootCfgSetVUSRTxConfig (3, tx);
  CSL_BootCfgLockKicker();
#endif /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E) */

} /* hyplnkExampleSerdesCfg */

#ifndef CSL_PSC_PD_HYPERBRIDGE
#ifdef CSL_PSC_PD_HYPERLINK_0
#if (hyplnk_EXAMPLE_PORT == 0)
#define CSL_PSC_PD_HYPERBRIDGE CSL_PSC_PD_HYPERLINK_0
#else
#define CSL_PSC_PD_HYPERBRIDGE CSL_PSC_PD_HYPERLINK_1
#endif
#else
#define CSL_PSC_PD_HYPERBRIDGE CSL_PSC_PD_HYPERLINK
#endif
#endif
#ifndef CSL_PSC_LPSC_HYPERBRIDGE
#ifdef CSL_PSC_LPSC_HYPERLINK_0
#if (hyplnk_EXAMPLE_PORT == 0)
#define CSL_PSC_LPSC_HYPERBRIDGE CSL_PSC_LPSC_HYPERLINK_0
#else
#define CSL_PSC_LPSC_HYPERBRIDGE CSL_PSC_LPSC_HYPERLINK_1
#endif
#else
#define CSL_PSC_LPSC_HYPERBRIDGE CSL_PSC_LPSC_HYPERLINK
#endif
#endif
/*****************************************************************************
 * Performs some of the system setup (HyperLink parameters outside of the
 * HyperLink block).
 *
 * The HyperLink power domain is enabled, and the SERDES configuration 
 * registers are programmed per the clock configuration (input=312.5 mhz).
 *
 * The system PLL is not touched (but could be).
 ****************************************************************************/
hyplnkRet_e hyplnkExampleSysSetup (void)
{
  /* The system PLLs are set up via the gel file described in the comments
   * at the header of this file.
   */

#if (defined(CSL_PSC_PD_HYPERBRIDGE) && defined(CSL_PSC_LPSC_HYPERBRIDGE))
   hyplnkPSCSetup(CSL_PSC_PD_HYPERBRIDGE, CSL_PSC_LPSC_HYPERBRIDGE);

#endif
#if 0
  hyplnkExampleAssertReset (1);
#endif
  /* Set up the HyperLink (vUSR) related portions of the boot config registers */
  
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E)
  CSL_BootCfgUnlockKicker();
  CSL_BootCfgSetVUSRConfigPLL (hyplnk_EXAMPLE_VUSR_PLL);
  while (((*(volatile unsigned int *)0x2620160) & 1) == 0);
  CSL_BootCfgLockKicker();
#endif /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E) */
  
#ifndef _VIRTUAL_ADDR_SUPPORT
  hyplnkExampleSerdesCfg (hyplnk_EXAMPLE_VUSR_RX_CONFIG, hyplnk_EXAMPLE_VUSR_TX_CONFIG);
#endif

  printf("Constructed SERDES configs: PLL=0x%08x; RX=0x%08x; TX=0x%08x\n",
         hyplnk_EXAMPLE_VUSR_PLL,
         hyplnk_EXAMPLE_VUSR_RX_CONFIG,
         hyplnk_EXAMPLE_VUSR_TX_CONFIG);
  return hyplnk_RET_OK;
}

#ifdef hyplnk_EXAMPLE_EQ_ANALYSIS
/*****************************************************************************
 * These functions performe equalization analysis on each lane.
 *
 * This will analyze the precursor (TX_TWPRE) and postcursor (TX_TWPST1) as
 * seen by the receiver on this side of the connection.  For each lane and
 * each coefficient, it will print out whether the coefficient is too large
 * and/or too small.
 *
 * The analysis can be run several times with the same configuration in
 * order to decide if the configuration is optimal.  When adjusting the
 * coefficients, remember that the transmitter (other device) is being
 * analyzed by this device.
 *
 * The result will be 4 ordered pairs, one ordered pair per lane.
 * The first digit in the pair is whether the coefficient is too big;
 * the second digit is whether it is too small.  It is possible to get
 * both 0 or both 1s.
 *
 * Example:
 *                          ______LANES______
 *                          | 0 | 1 | 2 | 3 |
 * -------------------RUN 1-------------------
 * Precursors :  0 Analysis: 1,0|1,0|0,1|1,0
 * Postcursors: 19 Analysis: 1,0|1,0|1,0|1,0
 * -------------------RUN 2-------------------
 * Precursors :  0 Analysis: 1,0|1,0|1,0|1,0
 * Postcursors: 19 Analysis: 1,0|1,0|1,0|1,0
 * -------------------RUN 3-------------------
 * Precursors :  0 Analysis: 1,0|1,0|1,0|1,0
 * Postcursors: 19 Analysis: 1,0|1,0|1,0|1,0
 *
 * This example shows that both the precursor and postcursor coefficients are
 * too big and should be lowered if possible.
 * Adjusting these coefficients to the best results will then produce the best
 * connection between devices.
 *
 ****************************************************************************/
void hyplnkExampleEQLaneAnalysis (uint32_t lane, uint32_t status) 
{
  if (lane) {
    System_printf(",");
  }
  System_printf ("%d,%d", (status >> 5) & 1, (status >> 4) & 1);
} /* hyplnkExampleEQLaneAnalysis */

void hyplnkExampleEQAnalysisOneEq (int precursor) 
{
  uint32_t eqVal;
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E)
  uint32_t thisStatus;
#endif /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E) */

  eqVal = precursor ? 2 : 3;
  /* EQHLD is set to 0 and EQ is set to 1 */
  
  /* Set EQHLD to 1 and wait at least 48UI */
#undef hyplnk_EXAMPLE_VUSR_RX_EQHLD
#define hyplnk_EXAMPLE_VUSR_RX_EQHLD  1
  hyplnkExampleSerdesCfg (hyplnk_EXAMPLE_VUSR_RX_CONFIG, hyplnk_EXAMPLE_VUSR_TX_CONFIG);
  hyplnkExampleDelay (1000);

  /* Set EQHLD to 0 and EQ to eqVal (to select pre (2) or post (3) cursor) and wait 150K UI */
#undef hyplnk_EXAMPLE_VUSR_RX_EQ
#undef hyplnk_EXAMPLE_VUSR_RX_EQHLD
#define hyplnk_EXAMPLE_VUSR_RX_EQ     0
#define hyplnk_EXAMPLE_VUSR_RX_EQHLD  0 
  /* EQ is set to 0, the eqVal is added in below */
  hyplnkExampleSerdesCfg (hyplnk_EXAMPLE_VUSR_RX_CONFIG | (eqVal << 18), 
                          hyplnk_EXAMPLE_VUSR_TX_CONFIG);
  hyplnkExampleDelay (1000000);

#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E)
  thisStatus = hBootCfg->STS_VUSR;
#endif /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E) */

  /* Set EQHLD to 1 */
#undef hyplnk_EXAMPLE_VUSR_RX_EQHLD
#define hyplnk_EXAMPLE_VUSR_RX_EQHLD  1 
  hyplnkExampleSerdesCfg (hyplnk_EXAMPLE_VUSR_RX_CONFIG, hyplnk_EXAMPLE_VUSR_TX_CONFIG);
  hyplnkExampleDelay (1000);

  /* Set EQ to 1 and EQHLD to 0 */
#undef hyplnk_EXAMPLE_VUSR_RX_EQ
#undef hyplnk_EXAMPLE_VUSR_RX_EQHLD
#define hyplnk_EXAMPLE_VUSR_RX_EQ     1
#define hyplnk_EXAMPLE_VUSR_RX_EQHLD  0 
  hyplnkExampleSerdesCfg (hyplnk_EXAMPLE_VUSR_RX_CONFIG, hyplnk_EXAMPLE_VUSR_TX_CONFIG);
  hyplnkExampleDelay (1000000);

  if (precursor) {
    System_printf ("Precursors %d ", hyplnk_EXAMPLE_VUSR_TX_TWPRE);
  } else {
    System_printf ("Postcursors: %d ", hyplnk_EXAMPLE_VUSR_TX_TWPST1);
  }
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E)
  System_printf ("Analysis: ");
  hyplnkExampleEQLaneAnalysis (0, (thisStatus >> 1) & 0x7f);
  hyplnkExampleEQLaneAnalysis (1, (thisStatus >> 8) & 0x7f);
  hyplnkExampleEQLaneAnalysis (2, (thisStatus >> 15) & 0x7f);
  hyplnkExampleEQLaneAnalysis (3, (thisStatus >> 22) & 0x7f);
#endif /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2E) */
  System_printf ("\n");
} /* hyplnkExampleEQAnalysisOneEq */

void hyplnkExampleEQAnalysis (void)
{
  hyplnkExampleEQAnalysisOneEq (1);
  hyplnkExampleEQAnalysisOneEq (0);
} /* hyplnkExampleEQAnalysis */

#endif /* hyplnk_EXAMPLE_EQ_ANALYSIS */

/*****************************************************************************
 * Forcibly reset the peripheral, and place it into loopback.  The
 * reset protocol is not performed, so pending transactions could leave
 * either device in a bad state.  Therefore it is assumed there are
 * no such transactions.
 ****************************************************************************/
hyplnkRet_e hyplnkExamplePeriphSetup (void)
{
  hyplnkRevReg_t            rev;
  hyplnkControlReg_t        control;
  hyplnkStatusReg_t         status;
  hyplnkLinkStatusReg_t     linkStatus;
  hyplnkECCErrorsReg_t      ECCErrors;
  hyplnkLanePwrMgmtReg_t    lanePwrMgmt;
  hyplnkSERDESControl1Reg_t serdesControl1;

  hyplnkRegisters_t setRegs, setRegs2;
  hyplnkRegisters_t getRegs;
  uint64_t wait_start, time_waited;
#ifndef hyplnk_EXAMPLE_LOOPBACK
  int retryCount = 0;
#endif
  int i;
  uint32_t lastSerdesSTS = 0;

  hyplnkRet_e retVal;
  Hyplnk_Handle handle = NULL;

  /* To prove the APIs work -- expose uninitialized variables */
  memset (&rev,            0xff, sizeof(rev));
  memset (&control,        0xff, sizeof(control));
  memset (&status,         0xff, sizeof(status));
  memset (&linkStatus,     0xff, sizeof(linkStatus));
  memset (&ECCErrors,      0xff, sizeof(ECCErrors));
  memset (&lanePwrMgmt,    0xff, sizeof(lanePwrMgmt));
  memset (&serdesControl1, 0xff, sizeof(serdesControl1));

  /* Will always write these regs when writing */
  memset (&setRegs, 0, sizeof(setRegs));
  setRegs.control        = &control;
  setRegs.status         = &status;
  setRegs.ECCErrors      = &ECCErrors;
  setRegs.lanePwrMgmt    = &lanePwrMgmt;
  setRegs.serdesControl1 = &serdesControl1;

  /* For stopping traffic */
  memset (&setRegs2, 0, sizeof(setRegs2));
  setRegs2.control       = &control;

  /* Will always read these regs when reading */
  memset (&getRegs, 0, sizeof(getRegs));
  getRegs.control        = &control;
  getRegs.rev            = &rev;
  getRegs.status         = &status;
  getRegs.linkStatus     = &linkStatus;
  getRegs.lanePwrMgmt    = &lanePwrMgmt;
  getRegs.ECCErrors      = &ECCErrors;
  getRegs.serdesControl1 = &serdesControl1;


  if ((retVal = Hyplnk_open(hyplnk_EXAMPLE_PORT, &handle)) != hyplnk_RET_OK) {
    System_printf ("Open failed\n");
    return retVal;
  }
  System_printf ("============================");
  System_printf ("Hyperlink Testing Port %d \n",hyplnk_EXAMPLE_PORT);
  System_printf ("============================");
  
  /* Read rev and control */
  if ((retVal = Hyplnk_readRegs (handle, hyplnk_LOCATION_LOCAL, &getRegs)) != hyplnk_RET_OK) {
    System_printf ("Read revision register failed!\n");
    return retVal;
  }

  System_printf ("============== begin registers before initialization ===========\n");
  hyplnkExamplePrintRevReg(&rev);
  hyplnkExamplePrintStatusReg(&status);
  hyplnkExamplePrintLinkStatusReg(&linkStatus);
  hyplnkExamplePrintControlReg(&control);
  hyplnkExamplePrintECCErrorsReg(&ECCErrors);
  System_printf ("============== end registers before initialization ===========\n");

  if ((rev.revMaj == 0) && (rev.revMin == 0)) {
    System_printf ("The revision register seems to be invalid.  This example doesn't run on the simulator\n");
    exit(1);
  }

  /* Stop traffic before reset */
  control.serialStop  = 1;
  if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs2)) != hyplnk_RET_OK) {
    System_printf ("Control register write failed!\n");
    return retVal;
  }
  /* Wait for traffic to stop */
  for (i = 0; i < hyplnk_EXAMPLE_SERIAL_STOP_WAIT_ITER; i++) {
    if ((retVal = Hyplnk_readRegs (handle, hyplnk_LOCATION_LOCAL, &getRegs)) != hyplnk_RET_OK) {
      System_printf ("Read registers failed!\n");
      return retVal;
    }
    if (status.rPend == 0) {
      break; /* done waiting */
    }
  }
  if (i == hyplnk_EXAMPLE_SERIAL_STOP_WAIT_ITER)
  {
     System_printf ("ERROR: rpend never went away\n");
     return hyplnk_RET_OK;
  }

  /* Reset the peripheral */
  control.reset       = 1;
  status.lError       = 1; /* Clear any error */
  status.rError       = 1; /* Clear any error */
  ECCErrors.sglErrCor = 0;
  ECCErrors.dblErrDet = 0;

  /* Force single lane operation */
#ifdef hyplnk_EXAMPLE_ALLOW_1_LANE
  lanePwrMgmt.singleLane = 1;
#else
  lanePwrMgmt.singleLane = 0;
#endif
#ifdef hyplnk_EXAMPLE_ALLOW_0_LANES
  lanePwrMgmt.zeroLane   = 1;
#else
  lanePwrMgmt.zeroLane   = 0;
#endif
#ifdef hyplnk_EXAMPLE_ALLOW_4_LANES
  lanePwrMgmt.quadLane   = 1;
#else
  lanePwrMgmt.quadLane   = 0;
#endif
#if !defined(hyplnk_EXAMPLE_ALLOW_4_LANES) && !defined(hyplnk_EXAMPLE_ALLOW_1_LANE)
#error Must allow 1, 4, or 1 and 4 lanes.
#endif

#ifdef hyplnk_EXAMPLE_ASYNC_CLOCKS
  serdesControl1.sleepCnt   = 0xff;
  serdesControl1.disableCnt = 0xff;
#endif

  if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs)) != hyplnk_RET_OK) {
    System_printf ("Control register write failed!\n");
    return retVal;
  }

  /* Don't need to clear any more status bits */
  setRegs.status         = NULL; 
  setRegs.ECCErrors      = NULL;
  setRegs.lanePwrMgmt    = NULL;
  setRegs.serdesControl1 = NULL;

#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(DEVICE_K2E) || defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_K2E)

  hyplnkExampleDefSerdesSetup();

#endif
  /* Take out of reset loopback */
  control.reset           = 0;
  control.statusIntEnable = 1; /* enable interrupt on lerror or rerror */
  control.intLocal        = 1;
#ifdef hyplnk_EXAMPLE_ERROR_INTERRUPT
  control.statusIntVec    = hyplnk_EXAMPLE_ISRNUM_FATAL;
#endif
  control.iLoop           = hyplnk_EXAMPLE_ILOOP_BIT;
  if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs)) != hyplnk_RET_OK) {
    System_printf ("Control register write failed!\n");
    return retVal;
  }

  /* Don't care about rev reg any more */
  getRegs.rev            = NULL;
  getRegs.serdesControl1 = NULL;

  /* Wait for peripheral to come up */
  wait_start = hyplnkExampleReadTime();

  do {
    if ((retVal = Hyplnk_readRegs (handle, hyplnk_LOCATION_LOCAL, &getRegs)) != hyplnk_RET_OK) {
      System_printf ("Read status failed!\n");
      return retVal;
    }
    time_waited = (hyplnkExampleReadTime() - wait_start);
#ifdef __ARMv7
    if ((time_waited > 20)) {
#else
    if ((! time_waited) || (time_waited > hyplnk_EXAMPLE_SYNC_WAIT_LIMIT)) {
#endif
#ifdef hyplnk_EXAMPLE_LOOPBACK
      System_printf ("Waited too long (FAIL)\n");
      exit(1);
#else
      System_printf ("Waiting for other side to come up (%8d)\n", retryCount);
      wait_start = hyplnkExampleReadTime();
      retryCount++;
      lastSerdesSTS = hyplnkExamplePrintSerdesStatus (lastSerdesSTS);
#endif
    }
  } while ((!status.link) || (status.pllUnlock));

  hyplnkExampleCheckOneStat (hyplnk_LOCATION_LOCAL, "immediately after link up", 0);

  lastSerdesSTS = hyplnkExamplePrintSerdesStatus (lastSerdesSTS);

  System_printf ("============== begin registers after initialization ===========\n");
  hyplnkExamplePrintStatusReg(&status);
  hyplnkExamplePrintLinkStatusReg(&linkStatus);
  hyplnkExamplePrintControlReg(&control);
  System_printf ("============== end registers after initialization ===========\n");
  System_printf ("Waiting 5 seconds to check link stability\n");
  hyplnkExampleCheckOneStat (hyplnk_LOCATION_LOCAL, "before stability wait", 0);
  wait_start = hyplnkExampleReadTime();
#ifdef __ARMv7
  sleep(5);
#else
  while ((hyplnkExampleReadTime() - wait_start) < (uint64_t)hyplnk_EXAMPLE_WAIT_STABILITY_TIME);
#endif
#ifdef hyplnk_EXAMPLE_EQ_ANALYSIS
  hyplnkExampleCheckOneStat (hyplnk_LOCATION_LOCAL, "before eq analysis", 0);
  System_printf ("Analyzing the connection for each lane\n");
  hyplnkExampleEQAnalysis();
#endif
  hyplnkExampleCheckOneStat (hyplnk_LOCATION_LOCAL, "after stability wait", 0);
  System_printf ("Link seems stable\n");
  System_printf("About to try to read remote registers\n");
  
  /* Enable remote communication */
  control.serialStop = 0;
  if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs2)) != hyplnk_RET_OK) {
    System_printf ("Control register write failed!\n");
    return retVal;
  }

  hyplnkExampleCheckOneStat (hyplnk_LOCATION_REMOTE, "after stability wait", 0);

  /* Get the remote registers */
  if ((retVal = Hyplnk_readRegs (handle, hyplnk_LOCATION_REMOTE, &getRegs)) != hyplnk_RET_OK) {
    System_printf ("REMOTE Read status failed!\n");
    return retVal;
  }

  System_printf ("============== begin REMOTE registers after initialization ===========\n");
  hyplnkExamplePrintStatusReg(&status);
  hyplnkExamplePrintLinkStatusReg(&linkStatus);
  hyplnkExamplePrintControlReg(&control);
  System_printf ("============== end REMOTE registers after initialization ===========\n");

  if ((retVal = Hyplnk_close (&handle)) != hyplnk_RET_OK) {
    System_printf ("close failed!\n");
    return retVal;
  }

  return hyplnk_RET_OK;
}

/*****************************************************************************
 * Program the memory map registers so the dataBuffer can be
 * seen through HyperLink
 ****************************************************************************/
hyplnkRet_e hyplnkExampleAddrMap (void *dataBuffer, void **dataBufferViaHlink, int segmentID)
{
  hyplnkRet_e retVal;
  Hyplnk_Handle handle = NULL;
  hyplnkTXAddrOvlyReg_t TXAddrOvly;
  hyplnkRXAddrSelReg_t  RXAddrSel;
  hyplnkRXPrivIDTbl_t   RXPrivIDs;
  hyplnkRXSegValReg_t   RXSegVal;
  hyplnkRegisters_t     localRegs;
  hyplnkRegisters_t     remoteRegs;
  hyplnkRXSegIdxReg_t   RXSegIdx;
  uint32_t globalAddr = (uint32_t)dataBuffer;
  uint32_t globalAddrBase, globalAddrOffset;
  int i;
  void *hlinkBase;

  memset (&TXAddrOvly, 0, sizeof(TXAddrOvly));
  memset (&RXAddrSel,  0, sizeof(RXAddrSel));
  memset (&RXPrivIDs,  0, sizeof(RXPrivIDs));
  memset (&RXSegVal,   0, sizeof(RXSegVal));
  memset (&localRegs,  0, sizeof(localRegs));
  memset (&remoteRegs, 0, sizeof(remoteRegs));
  memset (&RXSegIdx,   0, sizeof(RXSegIdx));

  localRegs.TXAddrOvly   = &TXAddrOvly;
  remoteRegs.RXAddrSel   = &RXAddrSel;
  remoteRegs.RXSegVal    = &RXSegVal;
  remoteRegs.RXPrivIDTbl = &RXPrivIDs;
  remoteRegs.RXSegIdx    = &RXSegIdx;

  if ((retVal = Hyplnk_open(hyplnk_EXAMPLE_PORT, &handle)) != hyplnk_RET_OK) {
    System_printf ("Open failed\n");
    return retVal;
  }

  /* This is "my side" */
  TXAddrOvly.txSecOvl    = 0;  /* Ignore the secure bit */
  TXAddrOvly.txPrivIDOvl = 12; /* Put privid in the MS 4 bits of address */
  TXAddrOvly.txIgnMask   = 10; /* Open 128MB window (which is common on all hyperlinks on all devices) */
  /* This is the "other side" (but it is me, because of loopback) */
  RXAddrSel.rxSecHi     = 0; /* Don't care about secure */
  RXAddrSel.rxSecLo     = 0; /* Don't care about secure */
  RXAddrSel.rxSecSel    = 0; /* Don't care about secure */
  RXAddrSel.rxPrivIDSel = 12; /* Symmetric with TXAddrOvly.txPrivIDOvl */
  RXAddrSel.rxSegSel    = 6;  /* Make segments of 4MB each */
  /* Make PrivID transparent */
  for (i = 0; i < hyplnk_RX_PRIVID_TBL_ENTS; i++) {
    RXPrivIDs[i].rxPrivIDVal = i;
  }
  /* Make one segment which can see dataBuffer */
#ifndef _VIRTUAL_ADDR_SUPPORT
  if ((globalAddr >= 0x800000) && (globalAddr < 0xa00000)) {
    /* Address is in L2 */
    globalAddr |= 0x10000000 | (DNUM << 24);


  }
#endif

  /* Align to 4MB */
  globalAddrBase   = globalAddr & ~0x3fffff;
  globalAddrOffset = globalAddr &  0x3fffff;
  RXSegVal.rxSegVal = globalAddrBase >> 16; /* 2MB */
  RXSegVal.rxLenVal = 21; /* 2MB */
  RXSegIdx.rxSegIdx = segmentID; //This selects which index in the look up table to write to

  if ((retVal = Hyplnk_getWindow (handle, &hlinkBase, NULL)) != hyplnk_RET_OK) {
    System_printf ("getWindow failed\n");
    return retVal;
  }

  *dataBufferViaHlink = (void *)( (char *)hlinkBase + globalAddrOffset + (segmentID<<(RXAddrSel.rxSegSel+16)));

  if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &localRegs)) != hyplnk_RET_OK) {
    System_printf ("Local register write failed!\n");
    return retVal;
  }

  if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_REMOTE, &remoteRegs)) != hyplnk_RET_OK) {
    System_printf ("Remote register write failed!\n");
    return retVal;
  }

  if ((retVal = Hyplnk_close (&handle)) != hyplnk_RET_OK) {
    System_printf ("close failed!\n");
    return retVal;
  }

  return hyplnk_RET_OK;
}


/*****************************************************************************
 * Make sure there are no errors on the serial link on one side
 ****************************************************************************/
void hyplnkExampleCheckOneStat (hyplnkLocation_e  location,
                                const char       *name,
                                int               noWarn)
{
  int                       pass = 1;
  Hyplnk_Handle             handle = NULL;
  hyplnkRegisters_t         regs;
  hyplnkStatusReg_t         status;
  hyplnkECCErrorsReg_t      ECCErrors;
  const char               *locStr;
  static uint32_t           lastRemoteCor = 0, lastLocalCor = 0;
  uint32_t                 *lastCor;

  memset(&regs, 0, sizeof(regs));
  regs.ECCErrors = &ECCErrors;
  regs.status    = &status;

  if (location == hyplnk_LOCATION_LOCAL) {
    locStr  = "Local";
    lastCor = &lastLocalCor;
  } else {
    locStr  = "Remote";
    lastCor = &lastRemoteCor;
  }

  if (Hyplnk_open(hyplnk_EXAMPLE_PORT, &handle) != hyplnk_RET_OK) {
    System_printf ("Open failed\n");
    exit(1);
  }

  if (Hyplnk_readRegs (handle, location, &regs) != hyplnk_RET_OK) {
    System_printf ("%s read for %s failed!\n", locStr, name);
    exit(1);
  }

  if (Hyplnk_close (&handle) != hyplnk_RET_OK) {
    System_printf ("close failed!\n");
    exit(1);
  }

  if (status.lError) {
    System_printf ("%s %s status.lError = %d\n", 
                   locStr, name, status.lError);
    pass = 0;
  }
  if (status.rError) {
    System_printf ("%s %s status.rError = %d\n", 
                   locStr, name, status.rError);
    pass = 0;
  }
  if (status.pllUnlock) {
    System_printf ("%s %s status.pllUnlock = %d\n", 
                   locStr, name, status.pllUnlock);
    pass = 0;
  }
  if (ECCErrors.sglErrCor != *lastCor) {
    if (! noWarn) {
      System_printf ("%s %s WARNING: ECCErrors.sglErrCor = %d\n", 
                     locStr, name, ECCErrors.sglErrCor);
      *lastCor = ECCErrors.sglErrCor;
    }
  }
  if (ECCErrors.dblErrDet) {
    System_printf ("%s %s ECCErrors.dblErrDet = %d\n", 
                   locStr, name, ECCErrors.dblErrDet);
    pass = 0;
  }

  if (! pass) {
    exit(1);
  }
} /* hyplnkExampleCheckOneStat */


hyplnkRet_e hyplnkReset(int portNum)
{
    hyplnkControlReg_t        control;
    hyplnkStatusReg_t         status;
    hyplnkRegisters_t setRegs;
    memset (&control, 0, sizeof(control));
    memset (&status, 0, sizeof(status));
    memset (&setRegs, 0, sizeof(setRegs));
    setRegs.control        = &control;
    setRegs.status         = &status;
    control.serialStop = 1;
    hyplnkRet_e retVal;
    Hyplnk_Handle handle = NULL;
    uint32_t baseAddr;
    if ((retVal = Hyplnk_open(portNum, &handle)) != hyplnk_RET_OK) {
        printf ("Open failed\n");
        return retVal;
    }
    if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs)) != hyplnk_RET_OK) {
        printf ("Control register write failed!\n");
        return retVal;
    }
    memset (&control, 0, sizeof(control));
    do{
          /* Read rev and control */
          if ((retVal = Hyplnk_readRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs)) != hyplnk_RET_OK) {
            printf ("Read revision register failed!\n");
          }
    } while(setRegs.status->rPend);
    control.serialStop = 0;
    control.reset = 1;
    if ((retVal = Hyplnk_writeRegs (handle, hyplnk_LOCATION_LOCAL, &setRegs)) != hyplnk_RET_OK) {
        System_printf ("Control register write failed!\n");
        return retVal;
    }
    hyplnkPSCDisable(CSL_PSC_PD_HYPERBRIDGE, CSL_PSC_LPSC_HYPERBRIDGE);
#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(DEVICE_K2E) || defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_K2E)
    baseAddr = hyplnk_mmap(CSL_HYPERLINK_0_SERDES_CFG_REGS,0x2000);
    CSL_SERDES_SHUTDOWN(baseAddr, 4);
#endif    
    Hyplnk_close(&handle);
    return hyplnk_RET_OK;
} /* hyplnkReset */

/* Nothing past this point */

