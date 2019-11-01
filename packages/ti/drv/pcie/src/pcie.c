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
 *  File Name: pcie.c
 *
 *  Processing/configuration functions for the PCIe driver.
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>

#include <string.h>

/* Global Variable which describes the PCIE LLD Version Information */
const char PCIELLDVersionStr[] = pcie_LLD_VERSION_STR ":" __DATE__  ":" __TIME__;

/*****************************************************************************
 * check if handle is valid
 ****************************************************************************/
int32_t pcie_check_handle_fcn (Pcie_Handle handle)
{
  int32_t i;
  int32_t ret_val = 0;
  if (handle) {
    for (i = 0; i < pcie_MAX_PERIPHS; i++) {
      if (handle == &pcieLObj.insts[i]) {
        ret_val = 1;
        break;
      }
    }
  }
  return ret_val;
} /* pcie_check_handle_fcn */


/*****************************************************************************
 * Convert the handle (which is a configuration base address) into
 * associated Pcie_DeviceCfgBaseAddr
 ****************************************************************************/
Pcie_DeviceCfgBaseAddr *pcie_handle_to_cfg (Pcie_Handle handle)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;

  return &h->bases;
} /* pcie_handle_to_cfg */


/*****************************************************************************
 **********  External APIs **********************
 ****************************************************************************/

/*********************************************************************
 * FUNCTION PURPOSE: Sets PCIe mode to RC or EP for interface
 * specified by handle
 *********************************************************************/
pcieRet_e Pcie_setInterfaceMode
(
  Pcie_Handle handle,     /**< [in]  The PCIE LLD instance identifier */
  pcieMode_e  mode        /**< [in] PCIE Mode */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.setInterfaceMode) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.setInterfaceMode (handle, mode);
    }
  }
  return ret_val;
} /* Pcie_setInterfaceMode */


/*********************************************************************
 * FUNCTION PURPOSE: Returns amount of reserved space between beginning
 *                   of hardware's data area and the base returned
 *                   by @ref Pcie_getMemSpaceRange.  This enables
 *                   sw to position windows correctly
 *********************************************************************/
pcieRet_e Pcie_getMemSpaceReserved 
(
  Pcie_Handle  handle,     /**< [in]  The PCIE LLD instance identifier */
  uint32_t    *resSize     /**< [out] Reserved space */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.getMemSpaceReserved) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.getMemSpaceReserved (handle, resSize);
    }
  }
  return ret_val;
} /* Pcie_getMemSpaceReserved */


/*********************************************************************
 * FUNCTION PURPOSE: Returns the PCIe Internal Address Range for the 
 *                   Memory Space. This range is used for accessing memory.
 *********************************************************************/
pcieRet_e Pcie_getMemSpaceRange 
(
  Pcie_Handle  handle,     /**< [in]  The PCIE LLD instance identifier */
  void         **base,     /**< [out] Memory Space base address */
  uint32_t      *size      /**< [out] Memory Space total size */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.getMemSpaceRange) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.getMemSpaceRange (handle, base, size);
    }
  }
  return ret_val;
} /* Pcie_getMemSpaceRange */

/*********************************************************************
 * FUNCTION PURPOSE: Reads any register
 ********************************************************************/
pcieRet_e Pcie_readRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *readRegs  /**< [in/out] List of registers to read */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.readRegs) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.readRegs (handle, location, readRegs);
    }
  }
  return ret_val;
} /* Pcie_readRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Writes any register
 ********************************************************************/
pcieRet_e Pcie_writeRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *writeRegs /**< [in] List of registers to write */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.writeRegs) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.writeRegs (handle, location, writeRegs);
    }
  }
  return ret_val;
} /* Pcie_writeRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Configures the Outbound Offset registers 
 *                   for outbound address translation
 ********************************************************************/
pcieRet_e Pcie_cfgObOffset 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  uint32_t         obAddrLo, /**< [in] Low  Outbound address offset (32bits) */
  uint32_t         obAddrHi, /**< [in] High Outbound address offset (32bits) */
  uint8_t          region    /**< [in] Identifies the Outbound region (0-31) */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.cfgObOffset) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.cfgObOffset (handle, obAddrLo, obAddrHi, region);
    }
  }
  return ret_val;
} /* Pcie_cfgObOffset */


/*********************************************************************
 * FUNCTION PURPOSE: Configures the Inbound Translation registers 
 ********************************************************************/
pcieRet_e Pcie_cfgIbTrans 
(
  Pcie_Handle             handle,  /**< [in] The PCIE LLD instance identifier */
  const pcieIbTransCfg_t *ibCfg    /**< [in] Inbound Translation Configuration parameters */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.cfgIbTrans) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.cfgIbTrans (handle, ibCfg);
    }
  }
  return ret_val;
} /* Pcie_cfgIbTrans */


/*********************************************************************
 * FUNCTION PURPOSE: Configures a BAR Register (32bits)
 ********************************************************************/
pcieRet_e Pcie_cfgBar 
(
  Pcie_Handle            handle,   /**< [in] The PCIE LLD instance identifier */
  const pcieBarCfg_t    *barCfg    /**< [in] BAR configuration parameters */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.cfgBar) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.cfgBar (handle, barCfg);
    }
  }
  return ret_val;
} /* Pcie_cfgBar */


/*********************************************************************
 * FUNCTION PURPOSE: Configures the ATU (address translation) unit
 ********************************************************************/
pcieRet_e Pcie_atuRegionConfig 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] local/remote */
  uint32_t         atuRegionIndex, /* [in] index number to configure */
  const            pcieAtuRegionParams_t *atuRegionParams /* [in] config structure */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.cfgAtu) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.cfgAtu (handle, location, atuRegionIndex, atuRegionParams);
    }
  }
  return ret_val;
} /* Pcie_atuRegionConfig */


/*********************************************************************
 * FUNCTION PURPOSE: Read which functional (MSI/INTX) interrupts are pending
 ********************************************************************/
pcieRet_e Pcie_getPendingFuncInts 
(
  Pcie_Handle      handle,     /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [out] revision specific interrupt spec */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [out] rev-specific msi pending bits to check */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.getPendingFuncInts) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.getPendingFuncInts (handle, pendingBits, 
                                                sizeMsiBits, msiBits);
    }
  }
  return ret_val;
} /* Pcie_getPendingFuncInts */


/*********************************************************************
 * FUNCTION PURPOSE: Clear specified pending functional (MSI/INTX) interrupts
 ********************************************************************/
pcieRet_e Pcie_clrPendingFuncInts 
(
  Pcie_Handle      handle,     /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [in] revision specific interrupt spec */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [in] rev-specific msi pending bits to ack */
)
{
  Pcie_IntHandle h = (Pcie_IntHandle)handle;
  pcieRet_e ret_val;
  if (h == 0)
  {
    ret_val = pcie_RET_INV_HANDLE;
  }
  else
  {
    if ((h->fxnTable.clrPendingFuncInts) == 0)
    {
      ret_val = pcie_RET_INV_FXNPTR;
    }
    else
    {
      ret_val = h->fxnTable.clrPendingFuncInts (handle, pendingBits, 
                                                sizeMsiBits, msiBits);
    }
  }
  return ret_val;
} /* Pcie_clrPendingFuncInts */


/*********************************************************************
 * FUNCTION PURPOSE: Returns version number
 ********************************************************************/
uint32_t Pcie_getVersion 
(
  void
)
{
  return pcie_LLD_VERSION_ID;
} /* Pcie_getVersion */

/*********************************************************************
 * FUNCTION PURPOSE: Returns version string
 ********************************************************************/
const char* Pcie_getVersionStr
(
  void
)
{
  return PCIELLDVersionStr;
} /* Pcie_getVersionStr */

/* Nothing past this point */

