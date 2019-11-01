/*
 *
 * Copyright (C) 2010-2018 Texas Instruments Incorporated - http://www.ti.com/ 
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
 *  File Name: pcieinit.c
 *
 *  Initialization functions for the PCIe driver.
 *
 *  This code is executed only during driver initialization.
 *
 */

#include "pcie.h"
#include "pcieloc.h"

#include <string.h>

/* PCIE Local Object */
Pcie_LocalObj pcieLObj;

/* Set to 1 once user calls Pcie_init */
uint8_t pcieLObjIsValid = 0;

/*********************************************************************
 * FUNCTION PURPOSE: Per-core hyperlink configuration
 *********************************************************************
 * DESCRIPTION: Stores device base addreses.  Performs phys2virt
 *              on addresses if required (MMU present).
 *********************************************************************/
pcieRet_e Pcie_init 
(
  const Pcie_InitCfg *cfg /**< [in] configuration */
)
{
  int32_t i;

  memset (&pcieLObj, 0, sizeof(pcieLObj));

  for (i = 0; i < pcie_MAX_PERIPHS; i++)
  {
    if (cfg->dev.fxnTablePtr[i])
    {
      pcieLObj.insts[i].fxnTable = *cfg->dev.fxnTablePtr[i];
    }
    if (cfg->dev.basesPtr[i])
    {
      pcieLObj.insts[i].bases    = *cfg->dev.basesPtr[i];
    }
    pcieLObj.insts[i].pcie_index = i;
  }
  pcieLObjIsValid = 1U;

  return pcie_RET_OK;
} /* Pcie_init */

/*********************************************************************
 * FUNCTION PURPOSE: Opens an LLD instance
 *********************************************************************
 * DESCRIPTION: The LLD instance is associated with the pcie
 *              register base address
 *********************************************************************/
pcieRet_e Pcie_open 
(
  int32_t          deviceNum,         /**< [in] pcie device number (0,1,...) */
  Pcie_Handle  *pHandle           /**< [out] Resulting instance handle for application regs*/
)
{
  pcieRet_e retVal = pcie_RET_OK;

  if (pcieLObjIsValid == 0) {
    retVal = pcie_RET_NO_INIT;
  }else{
    if (deviceNum >= pcie_MAX_PERIPHS) {
      retVal = pcie_RET_INV_DEVICENUM;
    }else if (pHandle == 0) {
      retVal = pcie_RET_INV_HANDLE;
    }else {
      if ((pcieLObj.insts[deviceNum].bases.cfgBase != 0) && 
          (pcieLObj.insts[deviceNum].bases.dataBase != 0)) {
        *pHandle = &pcieLObj.insts[deviceNum];
      }else {
        retVal = pcie_RET_INV_DEVICENUM;
      }
    }
  }
  return retVal;
} /* Pcie_open */

/*********************************************************************
 * FUNCTION PURPOSE: Opens an LLD instance
 *********************************************************************
 * DESCRIPTION: The LLD instance is associated with the pcie
 *              register base address
 *********************************************************************/
pcieRet_e Pcie_close 
(
  Pcie_Handle *pHandle /**< [in] The PCIe LLD instance indentifier */
)
{
  Pcie_Handle handle;
  pcieRet_e ret_val;
  
  if (pcieLObjIsValid == 0) {
    ret_val = pcie_RET_NO_INIT;
  }else {
    if (pHandle) {
      handle = *pHandle;
      if (pcie_check_handle_fcn(handle) == 0) {
        ret_val = pcie_RET_INV_HANDLE;
      }
      *pHandle = NULL;
      ret_val = pcie_RET_OK;
    }else  {
      ret_val = pcie_RET_INV_HANDLE;
    }
  }
  return ret_val;
} /* Pcie_close */

/* Nothing past this point */

