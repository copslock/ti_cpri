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
 *  File Name: hyplnkinit.c
 *
 *  Initialization functions for the HyperLink driver.
 *
 *  This code is executed only during driver initialization.
 *
 */

#include "hyplnk.h"
#include "hyplnkloc.h"
#include <ti/csl/cslr_vusr.h>

/* HYPLNK Local Object */
Hyplnk_LocalObj hyplnkLObj;

/* Set to 1 once user calls Hyplnk_init */
uint8_t hyplnkLObjIsValid = 0;

#pragma CODE_SECTION (Hyplnk_init, ".text:Hyplnk:init:init");
/*********************************************************************
 * FUNCTION PURPOSE: Per-core hyperlink configuration
 *********************************************************************
 * DESCRIPTION: Stores device base addreses.  Performs phys2virt
 *              on addresses if required (MMU present).
 *********************************************************************/
hyplnkRet_e Hyplnk_init 
(
  Hyplnk_InitCfg *cfg /**< [in] configuration */
)
{
  hyplnkLObj.cfg    = *cfg;
  hyplnkLObjIsValid = 1;

  return hyplnk_RET_OK;
}

#pragma CODE_SECTION (Hyplnk_open, ".text:Hyplnk:init:open");
/*********************************************************************
 * FUNCTION PURPOSE: Opens an LLD instance
 *********************************************************************
 * DESCRIPTION: The LLD instance is associated with the HyperLink
 *              register base address
 *********************************************************************/
hyplnkRet_e Hyplnk_open 
(
  int            deviceNum,  /**< [in] HyperLink device number (0,1,...) */
  Hyplnk_Handle *pHandle     /**< [out] Resulting instance handle */
)
{
  hyplnkRet_e retVal = hyplnk_RET_OK;

  if (! hyplnkLObjIsValid) {
    return hyplnk_RET_NO_INIT;
  }

  if (deviceNum >= hyplnk_MAX_PERIPHS) {
    retVal = hyplnk_RET_INV_DEVICENUM;
  } else if (! pHandle) {
    retVal = hyplnk_RET_INV_HANDLE;
  } else {
#ifdef hyplnk_DEBUG
    if (*pHandle != NULL) {
      retVal = hyplnk_RET_DBG_NON_NULL;
    }
#endif
    if (hyplnkLObj.cfg.dev.bases[deviceNum].cfgBase && 
        hyplnkLObj.cfg.dev.bases[deviceNum].dataBase) {
      *pHandle = hyplnkLObj.cfg.dev.bases[deviceNum].cfgBase;
    } else {
      retVal = hyplnk_RET_INV_DEVICENUM;
    }
  }
  return retVal;
} /* Hyplnk_open */

#pragma CODE_SECTION (Hyplnk_close, ".text:Hyplnk:init:close");
/*********************************************************************
 * FUNCTION PURPOSE: Opens an LLD instance
 *********************************************************************
 * DESCRIPTION: The LLD instance is associated with the HyperLink
 *              register base address
 *********************************************************************/
hyplnkRet_e Hyplnk_close 
(
  Hyplnk_Handle *pHandle /**< [in] The HYPLNK LLD instance indentifier */
)
{
  Hyplnk_Handle handle;

  if (! hyplnkLObjIsValid) {
    return hyplnk_RET_NO_INIT;
  }

  if (pHandle) {
    handle = *pHandle;
    hyplnk_check_handle(handle);
    *pHandle = NULL;
  } else {
    return hyplnk_RET_INV_HANDLE;
  }
  return hyplnk_RET_OK;
} /* Hyplnk_close */

/* Nothing past this point */

