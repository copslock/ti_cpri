/**
 *   @file  k2e/src/hyplnk_device.c
 *
 *   @brief   
 *      This file contains the device specific configuration and initialization routines
 *      for hyperlink Low Level Driver.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2013-2016, Texas Instruments, Inc.
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

/** 
 * This file contains an example device configuration for the hyperlink LLD.
 * It is not precompiled to facilitate user modification of the file.
 */

#include <stdint.h>
#include <stdlib.h>

/* CSL RL includes */
#include <ti/csl/cslr_device.h>

/* hyplnk LLD includes */
#include <ti/drv/hyplnk/device/hyplnk_device.h>
#include <ti/drv/hyplnk/hyplnk.h>

/** @addtogroup HYPLNK_LLD_DATASTRUCT
@{ 
*/

/** @brief HYPLNK LLD initialization parameters */
Hyplnk_InitCfg hyplnkInitCfg =
{
  {
    {
      {
        (void *)CSL_MCM_CONFIG_REGS,
        (void *)CSL_MCM_REGS
      },
      {
        NULL,
        NULL
      },
      {
        NULL,
        NULL
      },
      {
        NULL,
        NULL
      }
    } 
  }
};

/**
@}
*/

