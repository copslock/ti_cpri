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

#ifndef _HYPLNK_LLD_IFACE_H_
#define _HYPLNK_LLD_IFACE_H_
/*  
 * This file contains the prototypes for functions that interface with 
 * Hyperlink and the Hyperlink LLD.  They are common among all tests/examples.
 */

#include <ti/drv/hyplnk/hyplnk.h>
#ifdef _ARMv7
#include <time.h>
#include "hplib_util.h"
#endif
#ifndef __ARMv7
extern cregister volatile unsigned int TSCL;
extern cregister volatile unsigned int TSCH;
#endif

#define hyplnk_EXAMPLE_uS_TO_CYCLES(x) ((uint64_t)hyplnk_EXAMPLE_CPU_SPEED_MHZ * (x))

void hyplnkExampleCheckOneStat (hyplnkLocation_e  location,
                                const char       *name,
                                int               noWarn);
/*****************************************************************************
 * Read TSCL+TSCH
 ****************************************************************************/
static inline uint64_t hyplnkExampleReadTime () 
{
#ifdef __ARMv7
    return 0;
#else
    uint32_t low = TSCL;
    uint32_t high = TSCH;
    return _itoll(high,low);
#endif
}

/*****************************************************************************
 * Wait # of CPU cycles (excluding function call and loop entry/exit overhead)
 ****************************************************************************/
void hyplnkExampleDelay (uint32_t cycles);

/*****************************************************************************
 * Break down and print the revision register.
 ****************************************************************************/
void hyplnkExamplePrintRevReg (hyplnkRevReg_t *rev);

/*****************************************************************************
 * Break down and print the status register.
 ****************************************************************************/
void hyplnkExamplePrintStatusReg (hyplnkStatusReg_t *status);

/*****************************************************************************
 * Break down and print the link status register.
 ****************************************************************************/
void hyplnkExamplePrintLinkStatusReg (hyplnkLinkStatusReg_t *status);

/*****************************************************************************
 * Break down and print the control register.
 ****************************************************************************/
void hyplnkExamplePrintControlReg (hyplnkControlReg_t *control);

/*****************************************************************************
 * Break down and print the ECC errors register.
 ****************************************************************************/
void hyplnkExamplePrintECCErrorsReg (hyplnkECCErrorsReg_t *errors);

/*****************************************************************************
 * Break down and print the SERDES status register
 ****************************************************************************/
uint32_t hyplnkExamplePrintSerdesStatus (uint32_t lastStatus);

/*****************************************************************************
 * Place peripheral into reset
 ****************************************************************************/
hyplnkRet_e hyplnkExampleAssertReset (int val);

/*****************************************************************************
 * Sets the SERDES configuration registers
 ****************************************************************************/
void hyplnkExampleSerdesCfg (uint32_t rx, uint32_t tx);

/*****************************************************************************
 * Performs some of the system setup (HyperLink parameters outside of the
 * HyperLink block).
 ****************************************************************************/
hyplnkRet_e hyplnkExampleSysSetup (void);

#ifdef hyplnk_EXAMPLE_EQ_ANALYSIS
/*****************************************************************************
 * These functions performe equalization analysis on each lane.
 ****************************************************************************/
void hyplnkExampleEQAnalysis (void);
#endif /* hyplnk_EXAMPLE_EQ_ANALYSIS */

/*****************************************************************************
 * Forcibly reset the peripheral, and place it into loopback.  The
 * reset protocol is not performed, so pending transactions could leave
 * either device in a bad state.  Therefore it is assumed there are
 * no such transactions.
 ****************************************************************************/
hyplnkRet_e hyplnkExamplePeriphSetup (void);

/*****************************************************************************
 * Program the memory map registers so the dataBuffer can be
 * seen through HyperLink
 ****************************************************************************/
hyplnkRet_e hyplnkExampleAddrMap (void *dataBuffer, void **dataBufferViaHlink, int segmentID);

/*****************************************************************************
 * Reset the HyperLink peripheral and SerDes to allow for re-run
 ****************************************************************************/
hyplnkRet_e hyplnkReset(int portNum);

#endif /* _HYPLNK_LLD_IFACE_H_ */

/* Nothing past this point */

