/*
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
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

/**
 *   @file  srio_laneconfig.h
 *
 *   @brief
 *      This header file includes the types and prototypes for the functions to
 *      set the PLL, lane rate and port width contained in the srio_laneconfig.c file.
 *
 *
*/
#ifndef __SRIO_LANECONFIG_H__
#define __SRIO_LANECONFIG_H__

#include <ti/csl/csl_srio.h>
#include <benchmarking.h>

/**
 *  @ingroup sriolld_hwconfig_constants
 *
 *  @{
 */
/** These are the possible values for SRIO PLL input reference clock in MHZ */
typedef enum 
{
  srio_ref_clock_125p00Mhz = 0,    /**< Reference clock of 125.00Mhz for SRIO PLL */
  srio_ref_clock_156p25Mhz,        /**< Reference clock of 156.25Mhz for SRIO PLL */
  srio_ref_clock_250p00Mhz,        /**< Reference clock of 250.00Mhz for SRIO PLL */
  srio_ref_clock_312p50Mhz         /**< Reference clock of 312.50Mhz for SRIO PLL */
} srioRefClockMhz_e;
/* @} */

/**********************************************************************
 **************************** EXPORTED APIs ***************************
 **********************************************************************/
extern int32_t setEnableSrioPllRxTx (CSL_SrioHandle hSrio, srioRefClockMhz_e refClockMhz, srioLaneRateGbps_e linkRateGbps, int isLoopbackMode);
extern int32_t setSrioLanes (CSL_SrioHandle hSrio, srioLanesMode_e laneMode);
extern int32_t waitAllSrioPortsOperational (CSL_SrioHandle hSrio, srioLanesMode_e laneMode);
extern int32_t displaySrioLanesStatus (CSL_SrioHandle hSrio);

#endif /* __SRIO_LANECONFIG_H__*/
