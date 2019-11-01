/** 
 *   @file  fftc_device_cfg.c
 *
 *   @brief  
 *      This file contains APIs that are used by the driver to retrieve
 *      SoC specific configuration for FFTC.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012, Texas Instruments, Inc.
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
/* FFTC types include */
#include <fftc_types.h>

/* SoC definitions file */
#include <ti/csl/soc.h>

/* CHIP module include */
#include <ti/csl/csl_chip.h>

/** @addtogroup FFTC_FUNCTION
 @{ */

/**
* ============================================================================
*  @n@b Fftc_getDeviceAccumulatorConfig
*
*  @b   brief
*  @n   Given an FFTC instance number, this function retrieves any SoC specific
*       QM accumulator configuration such as accumulator channel number, queue
*       number to use for the FFTC interrupt setup by the driver.
*
*  @param[in]
*       instNum         FFTC instance number for which the configuration needs
*                       to be retrieved.
*
*  @param[out]
*       pAccChannelNum  Pointer to hold the accumulator channel returned by
*                       this function.
*
*  @param[out]
*       pAccRxQNum      Pointer to hold the Rx queue number returned by
*                       this function.
*
*  @return      Int32
*       -1      -   Error populating accumulator configuration.
*       0       -   Success. Configuration successfully populated
*                   in output parameter handles.
*
*  @pre
*  @n   None.
*
*  @post
*  @n   SoC specific accumulator configuration returned. 
* ============================================================================
*/
Int32 Fftc_getDeviceAccumulatorConfig
(
    uint8_t                       instNum,
    uint8_t*                      pAccChannelNum,
    uint32_t*                     pAccRxQNum
)
{
    uint8_t                       coreNum =   CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Pick the accumulator channel and a Rx queue number to
     * use for the FFTC instance specified on this core.
     */
    if (instNum == CSL_FFTC_0)
    {
        *pAccChannelNum     =   8;
        *pAccRxQNum         =   708;
    }
    else
    {
        *pAccChannelNum     =   24;
        *pAccRxQNum         =   716;
    }
    *pAccChannelNum         +=  coreNum;        
    *pAccRxQNum             +=  coreNum;        

    /* Return success. */
    return 0;
}

/**
@}
*/


