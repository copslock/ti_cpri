/**
 *   @file  ti/drv/nwal/k2l/src/nwal_device.c
 *
 *   @brief   
 *      This file contains the device specific configuration and initialization routines
 *      for NWAL library.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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
 *  \par
*/

/* CPPI Types includes */
#include <stdint.h>
#include <stdlib.h>

/* CPPI includes */
#include <ti/drv/nwal/nwal.h>

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_cpsgmiiAux.h>

#define NWAL_PA_NUM_CPPI_RX_CHANNELS_NSS_GEN1        24
#define NWAL_PA_NUM_CPPI_TX_CHANNELS_NSS_GEN1        9


/** @addtogroup TBD
@{ 
*/
/** @brief NWAL LLD initialization parameters */
nwalGlobalDeviceConfigParams_t nwalDeviceGblCfgParam =
{
   NWAL_CFG_NSS_GEN1,
   QMSS_PASS_QUEUE_BASE,
   CSL_NETCP_CFG_REGS,
   CSL_NETCP_CFG_SA_CFG_REGS,
   NWAL_PA_NUM_CPPI_RX_CHANNELS_NSS_GEN1,
   NWAL_PA_NUM_CPPI_TX_CHANNELS_NSS_GEN1
};
