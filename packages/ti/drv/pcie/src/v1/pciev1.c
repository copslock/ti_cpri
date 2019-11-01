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
 *  File Name: pciev1.c
 *
 *  Processing/configuration functions for the PCIe driver.
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v1/pcieloc.h>

#include <string.h>
/*****************************************************************************
 * Set the mode of one interface without depending directly on device 
 * dependant registers (via device.c)
 ****************************************************************************/
static void pcie_set_mode (Pciev1_DeviceCfgBaseAddrs *iface, pcieMode_e mode); /*for misra warning*/
static void pcie_set_mode (Pciev1_DeviceCfgBaseAddrs *iface, pcieMode_e mode)
{
  pcieTiConfDeviceTypeReg_t typeReg;
  uint32_t regVal;

  memset (&typeReg, 0, sizeof(typeReg));
  switch (mode)
  {
    case pcie_EP_MODE:
      regVal = 0;
      break;
    case pcie_LEGACY_EP_MODE:
      regVal = 1U;
      break;
    case pcie_RC_MODE:
    default:
      regVal = 4U;
      break;
  }
  typeReg.type = regVal;
  pciev1_write_tiConfDeviceType_reg ((CSL_PcieRegs*)iface->tiConf, &typeReg);
  do {
    /* Poll until complete */
    pciev1_read_tiConfDeviceType_reg ((CSL_PcieRegs*)iface->tiConf, &typeReg);
  } while (typeReg.type != regVal);
} /* pcie_set_mode */

/*****************************************************************************
 **********  External APIs **********************
 ****************************************************************************/

/*********************************************************************
 * FUNCTION PURPOSE: Sets PCIe mode to RC or EP for interface
 * specified by handle
 *********************************************************************/
pcieRet_e Pciev1_setInterfaceMode
(
  Pcie_Handle handle,     /**< [in]  The PCIE LLD instance identifier */
  pcieMode_e  mode        /**< [in] PCIE Mode */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases = (Pciev1_DeviceCfgBaseAddrs*)cfg->cfgBase;
  pcieRet_e retVal = pcie_RET_INV_HANDLE;

  if (bases) {
     pcie_set_mode (bases, mode);
     retVal = pcie_RET_OK;
  }

  return retVal;
} /* Pciev1_setInterfaceMode */

/*********************************************************************
 * FUNCTION PURPOSE: Returns amount of reserved space between beginning
 *                   of hardware's data area and the base returned
 *                   by @ref Pcie_getMemSpaceRange.  This enables
 *                   sw to position windows correctly
 *********************************************************************/
pcieRet_e Pciev1_getMemSpaceReserved 
(
  Pcie_Handle  handle,     /**< [in]  The PCIE LLD instance identifier */
  uint32_t    *resSize     /**< [out] Reserved space */
)
{
  pcieRet_e retVal = pcie_RET_OK;

  if (pcieLObjIsValid == 0) {
    retVal = pcie_RET_NO_INIT;
  }
  else {
    if (pcie_check_handle_fcn(handle) == 0) {
      retVal = pcie_RET_INV_HANDLE;
    }
    else {
      if (resSize) {
        Pcie_DeviceCfgBaseAddr *bases = pcie_handle_to_cfg (handle);
        if (bases) {
          *resSize = bases->dataReserved;
        } else {
          retVal = pcie_RET_INV_HANDLE;
        }
      }
    }
  }

  return retVal;
} /* Pciev1_getMemSpaceReserved */

/*********************************************************************
 * FUNCTION PURPOSE: Returns the PCIe Internal Address Range for the 
 *                   Memory Space. This range is used for accessing memory.
 *********************************************************************/
pcieRet_e Pciev1_getMemSpaceRange 
(
  Pcie_Handle  handle,     /**< [in]  The PCIE LLD instance identifier */
  void         **base,     /**< [out] Memory Space base address */
  uint32_t      *size      /**< [out] Memory Space total size */
)
{
  pcieRet_e retVal = pcie_RET_OK;

  if (pcieLObjIsValid == 0) {
    retVal = pcie_RET_NO_INIT;
  }
  else {
    if (pcie_check_handle_fcn(handle) == 0) {
      retVal = pcie_RET_INV_HANDLE;
    }
    else {
      if (base) {
        Pcie_DeviceCfgBaseAddr *bases = pcie_handle_to_cfg (handle);
        if (bases) {
          *base = bases->dataBase;
        } else {
          retVal = pcie_RET_INV_HANDLE;
        }
      }

      if (size) {
        *size = (uint32_t)0x10000000; /* 256 MB */
      }
    }
  }

  return retVal;
} /* Pciev1_getMemSpaceRange */

/*********************************************************************
 * FUNCTION PURPOSE: Reads any register
 ********************************************************************/
pcieRet_e Pciev1_readRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *readRegs  /**< [in/out] List of registers to read */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases = (Pciev1_DeviceCfgBaseAddrs*)cfg->cfgBase;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_RcCfgDbIcsRegs  *baseCfgRcRegs     = (CSL_RcCfgDbIcsRegs*)bases->rcDbics;
  CSL_EpCfgDbIcsRegs  *baseCfgEpRegs     = (CSL_EpCfgDbIcsRegs*)bases->rcDbics;  
  CSL_RcCfgDbIcsRegs  *baseCfgRcCS2Regs  = (CSL_RcCfgDbIcsRegs*)bases->rcDbics2;
  CSL_EpCfgDbIcsRegs  *baseCfgEpCS2Regs  = (CSL_EpCfgDbIcsRegs*)bases->rcDbics2;  
  CSL_PcieRegs        *baseCfgTiConfRegs = (CSL_PcieRegs*)bases->tiConf;
  CSL_PlConfRegs      *baseCfgPlRegs     = (CSL_PlConfRegs*)bases->plConf;
  
  pcieRet_e retVal = pcie_RET_OK;
  int32_t i;

  if (pcieLObjIsValid == 0) {
    retVal = pcie_RET_NO_INIT;
  }
  else {
    if (pcie_check_handle_fcn(handle) == 0) {
      retVal = pcie_RET_INV_HANDLE;
    }
    else {
      /* Get base address for Local or Remote config space */
      if (location != pcie_LOCATION_LOCAL) 
      {
        char *remoteBase  = (char *)cfg->dataBase + bases->remoteOffset;
        uint32_t delta    = 0;
        baseCfgRcRegs     = (CSL_RcCfgDbIcsRegs *)(remoteBase + delta);
        baseCfgEpRegs     = (CSL_EpCfgDbIcsRegs *)(remoteBase + delta);
        delta             = (char *)bases->plConf - (char *)bases->rcDbics;
        baseCfgPlRegs     = (CSL_PlConfRegs *)    (remoteBase + delta);
      }
    }
  }
  /*****************************************************************************************
  * Reject hw rev 0 app registers (these are similar but not identical to TI CONF on rev 1)
  *****************************************************************************************/
  if ((retVal == pcie_RET_OK) && (readRegs->pid != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->cmdStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->cfgTrans != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ioBase != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tlpCfg != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rstCmd != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ptmCfg != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmCmd != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmCfg != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->actStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->obSize != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->diagCtrl != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->endian != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->priority != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->irqEOI != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiIrq != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->epIrqSet != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->epIrqClr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->epIrqStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  for (i = 0; i < 4; i++) {
    if ((retVal == pcie_RET_OK) && (readRegs->genPurpose[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  for (i = 0; i < 8; i++) {
    if ((retVal == pcie_RET_OK) && (readRegs->msiIrqStatusRaw[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->msiIrqStatus[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->msiIrqEnableSet[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->msiIrqEnableClr[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  for (i = 0; i < 4; i++) {
    if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqStatusRaw[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqStatus[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqEnableSet[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqEnableClr[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errIrqStatusRaw != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errIrqStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errIrqEnableSet != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errIrqEnableClr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqStatusRaw != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqEnableSet != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqEnableClr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqStatusRaw != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqEnableSet != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqEnableClr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  for (i = 0; i < 8; i ++) {
    if ((retVal == pcie_RET_OK) && (readRegs->obOffsetLo[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->obOffsetHi[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }

  for (i = 0; i < 4; i ++) {
      if ((retVal == pcie_RET_OK) && (readRegs->ibBar[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->ibStartLo[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->ibStartHi[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->ibOffset[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }

  if ((retVal == pcie_RET_OK) && (readRegs->pcsCfg0 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pcsCfg1 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pcsStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  if ((retVal == pcie_RET_OK) && (readRegs->serdesCfg0 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->serdesCfg1 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  /*****************************************************************************************
  *Configuration Registers
  *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/

  if ((retVal == pcie_RET_OK) && (readRegs->vndDevId != NULL)) {
    retVal = pciev1_read_vndDevId_reg (&baseCfgEpRegs->DEVICE_VENDORID, readRegs->vndDevId);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->statusCmd != NULL)) {
    retVal = pciev1_read_statusCmd_reg (&baseCfgEpRegs->STATUS_COMMAND_REGISTER, readRegs->statusCmd);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->revId != NULL)) {
    retVal = pciev1_read_revId_reg (&baseCfgEpRegs->CLASSCODE_REVISIONID, readRegs->revId);
  }

  if ((retVal == pcie_RET_OK) && (readRegs->bist != NULL)) {
    retVal = pciev1_read_bist_reg (baseCfgEpRegs, readRegs->bist);
  }

  /*Type 0 Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->type0BarIdx != NULL)) {
    retVal = pciev1_read_type0Bar_reg (baseCfgEpRegs, &(readRegs->type0BarIdx->reg), 
                                                                       readRegs->type0BarIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type0Bar32bitIdx != NULL)) {
    retVal = pciev1_read_type0Bar32bit_reg (baseCfgEpRegs, &(readRegs->type0Bar32bitIdx->reg),
                                                                            readRegs->type0Bar32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type0BarMask32bitIdx != NULL)) {
    retVal = pciev1_read_type0Bar32bit_reg (baseCfgEpCS2Regs, &(readRegs->type0BarMask32bitIdx->reg),
                                                                                 readRegs->type0BarMask32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->subId != NULL)) {
    retVal = pciev1_read_subId_reg (baseCfgEpRegs, readRegs->subId);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->cardbusCisPointer != NULL)) {
    retVal = pciev1_read_cardbusCisPointer_reg (baseCfgEpRegs, readRegs->cardbusCisPointer);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->expRom != NULL)) {
    retVal = pciev1_read_expRom_reg (baseCfgEpRegs, readRegs->expRom);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->capPtr != NULL)) {
    retVal = pciev1_read_capPtr_reg (baseCfgEpRegs, readRegs->capPtr);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->intPin != NULL)) {
    retVal = pciev1_read_intPin_reg (baseCfgEpRegs, readRegs->intPin);
  }

  /*Type 1 Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->type1BistHeader != NULL)) {
    retVal = pciev1_read_type1BistHeader_reg (baseCfgRcRegs, readRegs->type1BistHeader);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1BarIdx != NULL)) {
    retVal = pciev1_read_type1Bar_reg (baseCfgRcRegs, &(readRegs->type1BarIdx->reg), 
                                                                       readRegs->type1BarIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1Bar32bitIdx != NULL)) {
    retVal = pciev1_read_type1Bar32bit_reg (baseCfgRcRegs, &(readRegs->type1Bar32bitIdx->reg),
                                                                            readRegs->type1Bar32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1BarMask32bitIdx != NULL)) {
    retVal = pciev1_read_type1Bar32bit_reg (baseCfgRcCS2Regs, &(readRegs->type1BarMask32bitIdx->reg),
                                                                               readRegs->type1BarMask32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1BusNum != NULL)) {
    retVal = pciev1_read_type1BusNum_reg (baseCfgRcRegs, readRegs->type1BusNum);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1SecStat != NULL)) {
    retVal = pciev1_read_type1SecStat_reg (baseCfgRcRegs, readRegs->type1SecStat);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1Memspace != NULL)) {
    retVal = pciev1_read_type1Memspace_reg (baseCfgRcRegs, readRegs->type1Memspace);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->prefMem != NULL)) {
    retVal = pciev1_read_prefMem_reg (baseCfgRcRegs, readRegs->prefMem);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->prefBaseUpper != NULL)) {
    retVal = pciev1_read_prefBaseUpper_reg (baseCfgRcRegs, readRegs->prefBaseUpper);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->prefLimitUpper != NULL)) {
    retVal = pciev1_read_prefLimitUpper_reg (baseCfgRcRegs, readRegs->prefLimitUpper);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1IOSpace != NULL)) {
    retVal = pciev1_read_type1IOSpace_reg (baseCfgRcRegs, readRegs->type1IOSpace);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1CapPtr != NULL)) {
    retVal = pciev1_read_type1CapPtr_reg (baseCfgRcRegs, readRegs->type1CapPtr);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1ExpnsnRom != NULL)) {
    retVal = pciev1_read_type1ExpnsnRom_reg (baseCfgRcRegs, readRegs->type1ExpnsnRom);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1BridgeInt != NULL)) {
    retVal = pciev1_read_type1BridgeInt_reg (baseCfgRcRegs, readRegs->type1BridgeInt);
  }

  /* Power Management Capabilities Registers */
  if ((retVal == pcie_RET_OK) && (readRegs->pmCap != NULL)) {
    retVal = pciev1_read_pmCap_reg (baseCfgEpRegs, readRegs->pmCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmCapCtlStat != NULL)) {
    retVal = pciev1_read_pmCapCtlStat_reg (baseCfgEpRegs, readRegs->pmCapCtlStat);
  }

  /*MSI Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->msiCap != NULL)) {
    retVal = pciev1_read_msiCap_reg (baseCfgEpRegs, readRegs->msiCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiLo32 != NULL)) {
    retVal = pciev1_read_msiLo32_reg (baseCfgEpRegs, readRegs->msiLo32);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiUp32 != NULL)) {
    retVal = pciev1_read_msiUp32_reg (baseCfgEpRegs, readRegs->msiUp32);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiData != NULL)) {
    retVal = pciev1_read_msiData_reg (baseCfgEpRegs, readRegs->msiData);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiCapOff10H != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiCapOff14H != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  /*Capabilities Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->pciesCap != NULL)) {
    retVal = pciev1_read_pciesCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.PCIE_CAP, readRegs->pciesCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->deviceCap != NULL)) {
    retVal = pciev1_read_deviceCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAP, readRegs->deviceCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->devStatCtrl != NULL)) {
    retVal = pciev1_read_devStatCtrl_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAS, readRegs->devStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->linkCap != NULL)) {
    retVal = pciev1_read_linkCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAP, readRegs->linkCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->linkStatCtrl != NULL)) {
    retVal = pciev1_read_linkStatCtrl_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAS, readRegs->linkStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->slotCap != NULL)) {
    retVal = pciev1_read_slotCap_reg (baseCfgRcRegs, readRegs->slotCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->slotStatCtrl != NULL)) {
    retVal = pciev1_read_slotStatCtrl_reg (baseCfgRcRegs, readRegs->slotStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rootCtrlCap != NULL)) {
    retVal = pciev1_read_rootCtrlCap_reg (baseCfgRcRegs, readRegs->rootCtrlCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rootStatus != NULL)) {
    retVal = pciev1_read_rootStatus_reg (baseCfgRcRegs, readRegs->rootStatus);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->devCap2 != NULL)) {
    retVal = pciev1_read_devCap2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAP_2, readRegs->devCap2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->devStatCtrl2 != NULL)) {
    retVal = pciev1_read_devStatCtrl2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAS_2, readRegs->devStatCtrl2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->linkCap2 != NULL)) {
    retVal = pciev1_read_linkCap2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAP_2, readRegs->linkCap2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->linkCtrl2 != NULL)) {
    retVal = pciev1_read_linkCtrl2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAS_2, readRegs->linkCtrl2);
  }


  /*Capabilities Extended Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->extCap != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->uncErr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->uncErrMask != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->uncErrSvrty != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->corErr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->corErrMask != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->accr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  for (i = 0; i < 4; i ++) {
    if ((retVal == pcie_RET_OK) && (readRegs->hdrLog[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rootErrCmd != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rootErrSt != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errSrcID != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  /*Port Logic Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->plAckTimer != NULL)) {
    retVal = pciev1_read_plAckTimer_reg (baseCfgPlRegs, readRegs->plAckTimer);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plOMsg != NULL)) {
    retVal = pciev1_read_plOMsg_reg (baseCfgPlRegs, readRegs->plOMsg);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plForceLink != NULL)) {
    retVal = pciev1_read_plForceLink_reg (baseCfgPlRegs, readRegs->plForceLink);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ackFreq != NULL)) {
    retVal = pciev1_read_ackFreq_reg (baseCfgPlRegs, readRegs->ackFreq);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->lnkCtrl != NULL)) {
    retVal = pciev1_read_lnkCtrl_reg (baseCfgPlRegs, readRegs->lnkCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->laneSkew != NULL)) {
    retVal = pciev1_read_laneSkew_reg (baseCfgPlRegs, readRegs->laneSkew);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->symNum != NULL)) {
    retVal = pciev1_read_symNum_reg (baseCfgPlRegs, readRegs->symNum);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->symTimerFltMask != NULL)) {
    retVal = pciev1_read_symTimerFltMask_reg (baseCfgPlRegs, readRegs->symTimerFltMask);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->fltMask2 != NULL)) {
    retVal = pciev1_read_fltMask2_reg (baseCfgPlRegs, readRegs->fltMask2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->debug0 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->debug1 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->gen2 != NULL)) {
    retVal = pciev1_read_gen2_reg (baseCfgPlRegs, readRegs->gen2);
  }

  /* hw rev 1 PLCONF registers */
  if ((retVal == pcie_RET_OK) && (readRegs->plconfObnpSubreqCtrl != NULL)) {
    retVal = pciev1_read_plconfObnpSubreqCtrl_reg (baseCfgPlRegs, readRegs->plconfObnpSubreqCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfTrPStsR != NULL)) {
    retVal = pciev1_read_plconfTrPStsR_reg (baseCfgPlRegs, readRegs->plconfTrPStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfTrNpStsR != NULL)) {
    retVal = pciev1_read_plconfTrNpStsR_reg (baseCfgPlRegs, readRegs->plconfTrNpStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfTrCStsR != NULL)) {
    retVal = pciev1_read_plconfTrCStsR_reg (baseCfgPlRegs, readRegs->plconfTrCStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfQStsR != NULL)) {
    retVal = pciev1_read_plconfQStsR_reg (baseCfgPlRegs, readRegs->plconfQStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVcTrAR1 != NULL)) {
    retVal = pciev1_read_plconfVcTrAR1_reg (baseCfgPlRegs, readRegs->plconfVcTrAR1);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVcTrAR2 != NULL)) {
    retVal = pciev1_read_plconfVcTrAR2_reg (baseCfgPlRegs, readRegs->plconfVcTrAR2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0PrQC != NULL)) {
    retVal = pciev1_read_plconfVc0PrQC_reg (baseCfgPlRegs, readRegs->plconfVc0PrQC);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0NprQC != NULL)) {
    retVal = pciev1_read_plconfVc0NprQC_reg (baseCfgPlRegs, readRegs->plconfVc0NprQC);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0CrQC != NULL)) {
    retVal = pciev1_read_plconfVc0CrQC_reg (baseCfgPlRegs, readRegs->plconfVc0CrQC);
  }
  for (i = 0; i < 3; i++)
  {
    if ((retVal == pcie_RET_OK) && (readRegs->plconfVcPrQC[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->plconfVcNprQC[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->plconfVcCrQC[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfPhyStsR != NULL)) {
    retVal = pciev1_read_plconfPhyStsR_reg (baseCfgPlRegs, readRegs->plconfPhyStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfPhyCtrlR != NULL)) {
    retVal = pciev1_read_plconfPhyCtrlR_reg (baseCfgPlRegs, readRegs->plconfPhyCtrlR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlAddress != NULL)) {
    retVal = pciev1_read_plconfMsiCtrlAddress_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlAddress);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlUpperAddress != NULL)) {
    retVal = pciev1_read_plconfMsiCtrlUpperAddress_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlUpperAddress);
  }
  for (i = 0; i < 8; i++) {
    if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntEnable[i] != NULL)) {
      retVal = pciev1_read_plconfMsiCtrlIntEnable_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlIntEnable[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntMask[i] != NULL)) {
      retVal = pciev1_read_plconfMsiCtrlIntMask_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlIntMask[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntStatus[i] != NULL)) {
      retVal = pciev1_read_plconfMsiCtrlIntStatus_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlIntStatus[i], i);
    }
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlGpio != NULL)) {
    retVal = pciev1_read_plconfMsiCtrlGpio_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlGpio);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfPipeLoopback != NULL)) {
    retVal = pciev1_read_plconfPipeLoopback_reg (baseCfgPlRegs, readRegs->plconfPipeLoopback);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfDbiRoWrEn != NULL)) {
    retVal = pciev1_read_plconfDbiRoWrEn_reg (baseCfgPlRegs, readRegs->plconfDbiRoWrEn);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfAxiSlvErrResp != NULL)) {
    retVal = pciev1_read_plconfAxiSlvErrResp_reg (baseCfgPlRegs, readRegs->plconfAxiSlvErrResp);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfAxiSlvTimeout != NULL)) {
    retVal = pciev1_read_plconfAxiSlvTimeout_reg (baseCfgPlRegs, readRegs->plconfAxiSlvTimeout);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuIndex != NULL)) {
    retVal = pciev1_read_plconfIatuIndex_reg (baseCfgPlRegs, readRegs->plconfIatuIndex);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl1 != NULL)) {
    retVal = pciev1_read_plconfIatuRegCtrl1_reg (baseCfgPlRegs, readRegs->plconfIatuRegCtrl1);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl2 != NULL)) {
    retVal = pciev1_read_plconfIatuRegCtrl2_reg (baseCfgPlRegs, readRegs->plconfIatuRegCtrl2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLowerBase != NULL)) {
    retVal = pciev1_read_plconfIatuRegLowerBase_reg (baseCfgPlRegs, readRegs->plconfIatuRegLowerBase);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegUpperBase != NULL)) {
    retVal = pciev1_read_plconfIatuRegUpperBase_reg (baseCfgPlRegs, readRegs->plconfIatuRegUpperBase);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLimit != NULL)) {
    retVal = pciev1_read_plconfIatuRegLimit_reg (baseCfgPlRegs, readRegs->plconfIatuRegLimit);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLowerTarget != NULL)) {
    retVal = pciev1_read_plconfIatuRegLowerTarget_reg (baseCfgPlRegs, readRegs->plconfIatuRegLowerTarget);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegUpperTarget != NULL)) {
    retVal = pciev1_read_plconfIatuRegUpperTarget_reg (baseCfgPlRegs, readRegs->plconfIatuRegUpperTarget);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl3 != NULL)) {
    retVal = pciev1_read_plconfIatuRegCtrl3_reg (baseCfgPlRegs, readRegs->plconfIatuRegCtrl3);
  }


  /* TI CONF registers */
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfRevision != NULL)) {
    retVal = pciev1_read_tiConfRevision_reg (baseCfgTiConfRegs, readRegs->tiConfRevision);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfSysConfig != NULL)) {
    retVal = pciev1_read_tiConfSysConfig_reg (baseCfgTiConfRegs, readRegs->tiConfSysConfig);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEoi != NULL)) {
    retVal = pciev1_read_tiConfIrqEoi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEoi);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusRawMain != NULL)) {
    retVal = pciev1_read_tiConfIrqStatusRawMain_reg (baseCfgTiConfRegs, readRegs->tiConfIrqStatusRawMain);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusMain != NULL)) {
    retVal = pciev1_read_tiConfIrqStatusMain_reg (baseCfgTiConfRegs, readRegs->tiConfIrqStatusMain);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableSetMain != NULL)) {
    retVal = pciev1_read_tiConfIrqEnableSetMain_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEnableSetMain);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableClrMain != NULL)) {
    retVal = pciev1_read_tiConfIrqEnableClrMain_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEnableClrMain);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusRawMsi != NULL)) {
    retVal = pciev1_read_tiConfIrqStatusRawMsi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqStatusRawMsi);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusMsi != NULL)) {
    retVal = pciev1_read_tiConfIrqStatusMsi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqStatusMsi);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableSetMsi != NULL)) {
    retVal = pciev1_read_tiConfIrqEnableSetMsi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEnableSetMsi);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableClrMsi != NULL)) {
    retVal = pciev1_read_tiConfIrqEnableClrMsi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEnableClrMsi);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDeviceType != NULL)) {
    retVal = pciev1_read_tiConfDeviceType_reg (baseCfgTiConfRegs, readRegs->tiConfDeviceType);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDeviceCmd != NULL)) {
    retVal = pciev1_read_tiConfDeviceCmd_reg (baseCfgTiConfRegs, readRegs->tiConfDeviceCmd);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfPmCtrl != NULL)) {
    retVal = pciev1_read_tiConfPmCtrl_reg (baseCfgTiConfRegs, readRegs->tiConfPmCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfPhyCs != NULL)) {
    retVal = pciev1_read_tiConfPhyCs_reg (baseCfgTiConfRegs, readRegs->tiConfPhyCs);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIntxAssert != NULL)) {
    retVal = pciev1_read_tiConfIntxAssert_reg (baseCfgTiConfRegs, readRegs->tiConfIntxAssert);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIntxDeassert != NULL)) {
    retVal = pciev1_read_tiConfIntxDeassert_reg (baseCfgTiConfRegs, readRegs->tiConfIntxDeassert);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfMsiXmt != NULL)) {
    retVal = pciev1_read_tiConfMsiXmt_reg (baseCfgTiConfRegs, readRegs->tiConfMsiXmt);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDebugCfg != NULL)) {
    retVal = pciev1_read_tiConfDebugCfg_reg (baseCfgTiConfRegs, readRegs->tiConfDebugCfg);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDebugData != NULL)) {
    retVal = pciev1_read_tiConfDebugData_reg (baseCfgTiConfRegs, readRegs->tiConfDebugData);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDiagCtrl != NULL)) {
    retVal = pciev1_read_tiConfDiagCtrl_reg (baseCfgTiConfRegs, readRegs->tiConfDiagCtrl);
  }

  return retVal;
} /* Pciev1_readRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Writes any register
 ********************************************************************/
pcieRet_e Pciev1_writeRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *writeRegs /**< [in] List of registers to write */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases = (Pciev1_DeviceCfgBaseAddrs*)cfg->cfgBase;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_RcCfgDbIcsRegs  *baseCfgRcRegs     = (CSL_RcCfgDbIcsRegs*)bases->rcDbics;
  CSL_EpCfgDbIcsRegs  *baseCfgEpRegs     = (CSL_EpCfgDbIcsRegs*)bases->rcDbics;  
  CSL_RcCfgDbIcsRegs  *baseCfgRcCS2Regs  = (CSL_RcCfgDbIcsRegs*)bases->rcDbics2;
  CSL_EpCfgDbIcsRegs  *baseCfgEpCS2Regs  = (CSL_EpCfgDbIcsRegs*)bases->rcDbics2;  
  CSL_PcieRegs        *baseCfgTiConfRegs = (CSL_PcieRegs*)bases->tiConf;
  CSL_PlConfRegs      *baseCfgPlRegs     = (CSL_PlConfRegs*)bases->plConf;
  
  pcieRet_e retVal = pcie_RET_OK;
  int32_t i;

  if (pcieLObjIsValid == 0) {
    retVal = pcie_RET_NO_INIT;
  }
  else {
    if (pcie_check_handle_fcn(handle) == 0) {
      retVal = pcie_RET_INV_HANDLE;
    }
    else {
      /* Get base address for Local/Remote config space */
      if (location != pcie_LOCATION_LOCAL) 
      {
        char *remoteBase  = (char *)cfg->dataBase + bases->remoteOffset;
        uint32_t delta    = 0;
        baseCfgRcRegs     = (CSL_RcCfgDbIcsRegs *)(remoteBase + delta);
        baseCfgEpRegs     = (CSL_EpCfgDbIcsRegs *)(remoteBase + delta);
        delta             = (char *)bases->plConf - (char *)bases->rcDbics;
        baseCfgPlRegs     = (CSL_PlConfRegs *)    (remoteBase + delta);
      }
    }
  }
  /*****************************************************************************************
  * Reject hw rev 0 app registers (these are similar but not identical to TI CONF on rev 1)
  *****************************************************************************************/
  if ((retVal == pcie_RET_OK) && (writeRegs->cmdStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->cfgTrans != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ioBase != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tlpCfg != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rstCmd != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ptmCfg != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmCmd != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmCfg != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->obSize != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->diagCtrl != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->endian != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->priority != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->irqEOI != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiIrq != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->epIrqSet != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->epIrqClr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->epIrqStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  for (i = 0; i < 4; i++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->genPurpose[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  for (i = 0; i < 8; i++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqStatusRaw[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqStatus[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqEnableSet[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqEnableClr[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  for (i = 0; i < 4; i++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqStatusRaw[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqStatus[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqEnableSet[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqEnableClr[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errIrqStatusRaw != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errIrqStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errIrqEnableSet != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errIrqEnableClr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqStatusRaw != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqEnableSet != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqEnableClr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqStatusRaw != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqStatus != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqEnableSet != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqEnableClr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  for (i = 0; i < 8; i ++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->obOffsetLo[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->obOffsetHi[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }

  for (i = 0; i < 4; i ++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->ibBar[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->ibStartLo[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->ibStartHi[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->ibOffset[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->pcsCfg0 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pcsCfg1 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->serdesCfg0 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->serdesCfg1 != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  /*****************************************************************************************
  *Configuration Registers
  *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/

  if ((retVal == pcie_RET_OK) && (writeRegs->vndDevId != NULL)) {
    retVal = pciev1_write_vndDevId_reg (&baseCfgEpRegs->DEVICE_VENDORID, writeRegs->vndDevId);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->statusCmd != NULL)) {
    retVal = pciev1_write_statusCmd_reg (&baseCfgEpRegs->STATUS_COMMAND_REGISTER, writeRegs->statusCmd);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->revId != NULL)) {
    retVal = pciev1_write_revId_reg (&baseCfgEpRegs->CLASSCODE_REVISIONID, writeRegs->revId);
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->bist != NULL)) {
    retVal = pciev1_write_bist_reg (baseCfgEpRegs, writeRegs->bist);
  }

  /*Type 0 Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->type0BarIdx != NULL)) {
    retVal = pciev1_write_type0Bar_reg (baseCfgEpRegs, &(writeRegs->type0BarIdx->reg), 
                                                                       writeRegs->type0BarIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type0BarMask32bitIdx != NULL)) {
    retVal = pciev1_write_type0Bar32bit_reg (baseCfgEpCS2Regs, &(writeRegs->type0BarMask32bitIdx->reg),
                                                                                writeRegs->type0BarMask32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type0Bar32bitIdx != NULL)) {
    retVal = pciev1_write_type0Bar32bit_reg (baseCfgEpRegs, &(writeRegs->type0Bar32bitIdx->reg),
                                                                            writeRegs->type0Bar32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->subId != NULL)) {
    retVal = pciev1_write_subId_reg (baseCfgEpRegs, writeRegs->subId);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->cardbusCisPointer != NULL)) {
    retVal = pciev1_write_cardbusCisPointer_reg (baseCfgEpRegs, writeRegs->cardbusCisPointer);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->expRom != NULL)) {
    retVal = pciev1_write_expRom_reg (baseCfgEpRegs, writeRegs->expRom);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->capPtr != NULL)) {
    retVal = pciev1_write_capPtr_reg (baseCfgEpRegs, writeRegs->capPtr);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->intPin != NULL)) {
    retVal = pciev1_write_intPin_reg (baseCfgEpRegs, writeRegs->intPin);
  }

  /*Type 1 Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BistHeader != NULL)) {
    retVal = pciev1_write_type1BistHeader_reg (baseCfgRcRegs, writeRegs->type1BistHeader);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BarIdx != NULL)) {
    retVal = pciev1_write_type1Bar_reg (baseCfgRcRegs, &(writeRegs->type1BarIdx->reg), 
                                                                       writeRegs->type1BarIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BarMask32bitIdx != NULL)) {
    retVal = pciev1_write_type1Bar32bit_reg (baseCfgRcCS2Regs, &(writeRegs->type1BarMask32bitIdx->reg),
                                                                                writeRegs->type1BarMask32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1Bar32bitIdx != NULL)) {
    retVal = pciev1_write_type1Bar32bit_reg (baseCfgRcRegs, &(writeRegs->type1Bar32bitIdx->reg),
                                                                            writeRegs->type1Bar32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BusNum != NULL)) {
    retVal = pciev1_write_type1BusNum_reg (baseCfgRcRegs, writeRegs->type1BusNum);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1SecStat != NULL)) {
    retVal = pciev1_write_type1SecStat_reg (baseCfgRcRegs, writeRegs->type1SecStat);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1Memspace != NULL)) {
    retVal = pciev1_write_type1Memspace_reg (baseCfgRcRegs, writeRegs->type1Memspace);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->prefMem != NULL)) {
    retVal = pciev1_write_prefMem_reg (baseCfgRcRegs, writeRegs->prefMem);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->prefBaseUpper != NULL)) {
    retVal = pciev1_write_prefBaseUpper_reg (baseCfgRcRegs, writeRegs->prefBaseUpper);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->prefLimitUpper != NULL)) {
    retVal = pciev1_write_prefLimitUpper_reg (baseCfgRcRegs, writeRegs->prefLimitUpper);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1IOSpace != NULL)) {
    retVal = pciev1_write_type1IOSpace_reg (baseCfgRcRegs, writeRegs->type1IOSpace);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1CapPtr != NULL)) {
    retVal = pciev1_write_type1CapPtr_reg (baseCfgRcRegs, writeRegs->type1CapPtr);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1ExpnsnRom != NULL)) {
    retVal = pciev1_write_type1ExpnsnRom_reg (baseCfgRcRegs, writeRegs->type1ExpnsnRom);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BridgeInt != NULL)) {
    retVal = pciev1_write_type1BridgeInt_reg (baseCfgRcRegs, writeRegs->type1BridgeInt);
  }

  /* Power Management Capabilities Registers */
  if ((retVal == pcie_RET_OK) && (writeRegs->pmCap != NULL)) {
    retVal = pciev1_write_pmCap_reg (baseCfgEpRegs, writeRegs->pmCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmCapCtlStat != NULL)) {
    retVal = pciev1_write_pmCapCtlStat_reg (baseCfgEpRegs, writeRegs->pmCapCtlStat);
  }

  /*MSI Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->msiCap != NULL)) {
    retVal = pciev1_write_msiCap_reg (baseCfgEpRegs, writeRegs->msiCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiLo32 != NULL)) {
    retVal = pciev1_write_msiLo32_reg (baseCfgEpRegs, writeRegs->msiLo32);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiUp32 != NULL)) {
    retVal = pciev1_write_msiUp32_reg (baseCfgEpRegs, writeRegs->msiUp32);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiData != NULL)) {
    retVal = pciev1_write_msiData_reg (baseCfgEpRegs, writeRegs->msiData);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiCapOff10H != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiCapOff14H != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  /*Capabilities Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->pciesCap != NULL)) {
    retVal = pciev1_write_pciesCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.PCIE_CAP, writeRegs->pciesCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->deviceCap != NULL)) {
    retVal = pciev1_write_deviceCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAP, writeRegs->deviceCap);
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->devStatCtrl != NULL)) {
    retVal = pciev1_write_devStatCtrl_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAS, writeRegs->devStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->linkCap != NULL)) {
    retVal = pciev1_write_linkCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAP, writeRegs->linkCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->linkStatCtrl != NULL)) {
    retVal = pciev1_write_linkStatCtrl_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAS, writeRegs->linkStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->slotCap != NULL)) {
    retVal = pciev1_write_slotCap_reg (baseCfgRcRegs, writeRegs->slotCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->slotStatCtrl != NULL)) {
    retVal = pciev1_write_slotStatCtrl_reg (baseCfgRcRegs, writeRegs->slotStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rootCtrlCap != NULL)) {
    retVal = pciev1_write_rootCtrlCap_reg (baseCfgRcRegs, writeRegs->rootCtrlCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rootStatus != NULL)) {
    retVal = pciev1_write_rootStatus_reg (baseCfgRcRegs, writeRegs->rootStatus);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->devCap2 != NULL)) {
    retVal = pciev1_write_devCap2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAP_2, writeRegs->devCap2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->devStatCtrl2 != NULL)) {
    retVal = pciev1_write_devStatCtrl2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAS_2, writeRegs->devStatCtrl2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->linkCap2 != NULL)) {
    retVal = pciev1_write_linkCap2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAP_2, writeRegs->linkCap2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->linkCtrl2 != NULL)) {
    retVal = pciev1_write_linkCtrl2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAS_2, writeRegs->linkCtrl2);
  }

  /*Capabilities Extended Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->extCap != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->uncErr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->uncErrMask != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->uncErrSvrty != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->corErr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->corErrMask != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->accr != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  for (i = 0; i < 4; i ++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->hdrLog[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rootErrCmd != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rootErrSt != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errSrcID != NULL)) {
    /* Not supported on rev 1 */
    retVal = pcie_RET_INV_REG;
  }

  /*Port Logic Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->plAckTimer != NULL)) {
    retVal = pciev1_write_plAckTimer_reg (baseCfgPlRegs, writeRegs->plAckTimer);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plOMsg != NULL)) {
    retVal = pciev1_write_plOMsg_reg (baseCfgPlRegs, writeRegs->plOMsg);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plForceLink != NULL)) {
    retVal = pciev1_write_plForceLink_reg (baseCfgPlRegs, writeRegs->plForceLink);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ackFreq != NULL)) {
    retVal = pciev1_write_ackFreq_reg (baseCfgPlRegs, writeRegs->ackFreq);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->lnkCtrl != NULL)) {
    retVal = pciev1_write_lnkCtrl_reg (baseCfgPlRegs, writeRegs->lnkCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->laneSkew != NULL)) {
    retVal = pciev1_write_laneSkew_reg (baseCfgPlRegs, writeRegs->laneSkew);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->symNum != NULL)) {
    retVal = pciev1_write_symNum_reg (baseCfgPlRegs, writeRegs->symNum);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->symTimerFltMask != NULL)) {
    retVal = pciev1_write_symTimerFltMask_reg (baseCfgPlRegs, writeRegs->symTimerFltMask);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->fltMask2 != NULL)) {
    retVal = pciev1_write_fltMask2_reg (baseCfgPlRegs, writeRegs->fltMask2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->gen2 != NULL)) {
    retVal = pciev1_write_gen2_reg (baseCfgPlRegs, writeRegs->gen2);
  }

  /* hw rev 1 PLCONF registers */
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfObnpSubreqCtrl != NULL)) {
    retVal = pciev1_write_plconfObnpSubreqCtrl_reg (baseCfgPlRegs, writeRegs->plconfObnpSubreqCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfTrPStsR != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfTrNpStsR != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfTrCStsR != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfQStsR != NULL)) {
    retVal = pciev1_write_plconfQStsR_reg (baseCfgPlRegs, writeRegs->plconfQStsR);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcTrAR1 != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcTrAR2 != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfVc0PrQC != NULL)) {
    retVal = pciev1_write_plconfVc0PrQC_reg (baseCfgPlRegs, writeRegs->plconfVc0PrQC);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfVc0NprQC != NULL)) {
    retVal = pciev1_write_plconfVc0NprQC_reg (baseCfgPlRegs, writeRegs->plconfVc0NprQC);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfVc0CrQC != NULL)) {
    retVal = pciev1_write_plconfVc0CrQC_reg (baseCfgPlRegs, writeRegs->plconfVc0CrQC);
  }
  for (i = 0; i < 3; i++)
  {
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcPrQC[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcNprQC[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcCrQC[i] != NULL)) {
      /* Not supported on rev 1 */
      retVal = pcie_RET_INV_REG;
    }
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfPhyStsR != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfPhyCtrlR != NULL)) {
    retVal = pciev1_write_plconfPhyCtrlR_reg (baseCfgPlRegs, writeRegs->plconfPhyCtrlR);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlAddress != NULL)) {
    retVal = pciev1_write_plconfMsiCtrlAddress_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlAddress);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlUpperAddress != NULL)) {
    retVal = pciev1_write_plconfMsiCtrlUpperAddress_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlUpperAddress);
  }
  for (i = 0; i < 8; i++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntEnable[i] != NULL)) {
      retVal = pciev1_write_plconfMsiCtrlIntEnable_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlIntEnable[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntMask[i] != NULL)) {
      retVal = pciev1_write_plconfMsiCtrlIntMask_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlIntMask[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntStatus[i] != NULL)) {
      retVal = pciev1_write_plconfMsiCtrlIntStatus_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlIntStatus[i], i);
    }
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlGpio != NULL)) {
    retVal = pciev1_write_plconfMsiCtrlGpio_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlGpio);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfPipeLoopback != NULL)) {
    retVal = pciev1_write_plconfPipeLoopback_reg (baseCfgPlRegs, writeRegs->plconfPipeLoopback);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfDbiRoWrEn != NULL)) {
    retVal = pciev1_write_plconfDbiRoWrEn_reg (baseCfgPlRegs, writeRegs->plconfDbiRoWrEn);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfAxiSlvErrResp != NULL)) {
    retVal = pciev1_write_plconfAxiSlvErrResp_reg (baseCfgPlRegs, writeRegs->plconfAxiSlvErrResp);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfAxiSlvTimeout != NULL)) {
    retVal = pciev1_write_plconfAxiSlvTimeout_reg (baseCfgPlRegs, writeRegs->plconfAxiSlvTimeout);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuIndex != NULL)) {
    retVal = pciev1_write_plconfIatuIndex_reg (baseCfgPlRegs, writeRegs->plconfIatuIndex);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl2 != NULL)) {
    retVal = pciev1_write_plconfIatuRegCtrl2_reg (baseCfgPlRegs, writeRegs->plconfIatuRegCtrl2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLowerBase != NULL)) {
    retVal = pciev1_write_plconfIatuRegLowerBase_reg (baseCfgPlRegs, writeRegs->plconfIatuRegLowerBase);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegUpperBase != NULL)) {
    retVal = pciev1_write_plconfIatuRegUpperBase_reg (baseCfgPlRegs, writeRegs->plconfIatuRegUpperBase);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLimit != NULL)) {
    retVal = pciev1_write_plconfIatuRegLimit_reg (baseCfgPlRegs, writeRegs->plconfIatuRegLimit);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLowerTarget != NULL)) {
    retVal = pciev1_write_plconfIatuRegLowerTarget_reg (baseCfgPlRegs, writeRegs->plconfIatuRegLowerTarget);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegUpperTarget != NULL)) {
    retVal = pciev1_write_plconfIatuRegUpperTarget_reg (baseCfgPlRegs, writeRegs->plconfIatuRegUpperTarget);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl3 != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  /* Ctrl1 is done last since it has enable bit */
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl1 != NULL)) {
    retVal = pciev1_write_plconfIatuRegCtrl1_reg (baseCfgPlRegs, writeRegs->plconfIatuRegCtrl1);
  }

  /* TI CONF registers */
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfRevision != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfSysConfig != NULL)) {
    retVal = pciev1_write_tiConfSysConfig_reg (baseCfgTiConfRegs, writeRegs->tiConfSysConfig);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEoi != NULL)) {
    retVal = pciev1_write_tiConfIrqEoi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEoi);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusRawMain != NULL)) {
    retVal = pciev1_write_tiConfIrqStatusRawMain_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqStatusRawMain);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusMain != NULL)) {
    retVal = pciev1_write_tiConfIrqStatusMain_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqStatusMain);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableSetMain != NULL)) {
    retVal = pciev1_write_tiConfIrqEnableSetMain_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEnableSetMain);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableClrMain != NULL)) {
    retVal = pciev1_write_tiConfIrqEnableClrMain_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEnableClrMain);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusRawMsi != NULL)) {
    retVal = pciev1_write_tiConfIrqStatusRawMsi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqStatusRawMsi);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusMsi != NULL)) {
    retVal = pciev1_write_tiConfIrqStatusMsi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqStatusMsi);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableSetMsi != NULL)) {
    retVal = pciev1_write_tiConfIrqEnableSetMsi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEnableSetMsi);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableClrMsi != NULL)) {
    retVal = pciev1_write_tiConfIrqEnableClrMsi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEnableClrMsi);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDeviceType != NULL)) {
    retVal = pciev1_write_tiConfDeviceType_reg (baseCfgTiConfRegs, writeRegs->tiConfDeviceType);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDeviceCmd != NULL)) {
    retVal = pciev1_write_tiConfDeviceCmd_reg (baseCfgTiConfRegs, writeRegs->tiConfDeviceCmd);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfPmCtrl != NULL)) {
    retVal = pciev1_write_tiConfPmCtrl_reg (baseCfgTiConfRegs, writeRegs->tiConfPmCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfPhyCs != NULL)) {
    retVal = pciev1_write_tiConfPhyCs_reg (baseCfgTiConfRegs, writeRegs->tiConfPhyCs);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIntxAssert != NULL)) {
    retVal = pciev1_write_tiConfIntxAssert_reg (baseCfgTiConfRegs, writeRegs->tiConfIntxAssert);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIntxDeassert != NULL)) {
    retVal = pciev1_write_tiConfIntxDeassert_reg (baseCfgTiConfRegs, writeRegs->tiConfIntxDeassert);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfMsiXmt != NULL)) {
    retVal = pciev1_write_tiConfMsiXmt_reg (baseCfgTiConfRegs, writeRegs->tiConfMsiXmt);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDebugCfg != NULL)) {
    retVal = pciev1_write_tiConfDebugCfg_reg (baseCfgTiConfRegs, writeRegs->tiConfDebugCfg);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDebugData != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDiagCtrl != NULL)) {
    retVal = pciev1_write_tiConfDiagCtrl_reg (baseCfgTiConfRegs, writeRegs->tiConfDiagCtrl);
  }

  return retVal;
} /* Pciev1_writeRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Configures a BAR Register (32bits)
 ********************************************************************/
pcieRet_e Pciev1_cfgBar 
(
  Pcie_Handle            handle,   /**< [in] The PCIE LLD instance identifier */
  const pcieBarCfg_t    *barCfg    /**< [in] BAR configuration parameters */
)
{
  pcieRet_e          retVal = pcie_RET_OK;
  pcieType0BarIdx_t  type0BarIdx;  
  pcieType1BarIdx_t  type1BarIdx;  
  pcieRegisters_t    setRegs;
  uint32_t           barAddrField = 0;

  if (pcieLObjIsValid == 0) {
    retVal = pcie_RET_NO_INIT;
  }
  else {
    if (pcie_check_handle_fcn(handle) == 0) {
      retVal = pcie_RET_INV_HANDLE;
    }
    else { 
      memset (&setRegs,     0, sizeof(setRegs));
      memset (&type0BarIdx, 0, sizeof(type0BarIdx));
      memset (&type1BarIdx, 0, sizeof(type1BarIdx));

      if(barCfg->mode == pcie_RC_MODE)
      {
        pcie_getbits(barCfg->base, PCIE_RC_BAR_BASE_FULL, barAddrField);

        type1BarIdx.reg.base     = barAddrField;
        type1BarIdx.reg.prefetch = barCfg->prefetch;
        type1BarIdx.reg.type     = barCfg->type;
        type1BarIdx.reg.memSpace = barCfg->memSpace;
        type1BarIdx.idx          = barCfg->idx;

        setRegs.type1BarIdx = &type1BarIdx;   
      }
      else
      {
        pcie_getbits(barCfg->base, PCIE_EP_BAR_BASE_FULL, barAddrField);

        type0BarIdx.reg.base     = barAddrField;
        type0BarIdx.reg.prefetch = barCfg->prefetch;
        type0BarIdx.reg.type     = barCfg->type;
        type0BarIdx.reg.memSpace = barCfg->memSpace;
        type0BarIdx.idx          = barCfg->idx;

        setRegs.type0BarIdx = &type0BarIdx;   
      }

      retVal = Pciev1_writeRegs (handle, barCfg->location, &setRegs);
    }
  }
  return retVal;
} /* Pciev1_cfgBar */


/*********************************************************************
 * FUNCTION PURPOSE: Configures an ATU (address translation) region
 ********************************************************************/
pcieRet_e Pciev1_atuRegionConfig 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] local/remote */
  uint32_t         atuRegionIndex, /* [in] index number to configure */
  const            pcieAtuRegionParams_t *atuRegionParams /* [in] config structure */
)
{
  pcieRet_e                         retVal = pcie_RET_OK;
  pciePlconfIatuIndexReg_t          index;
  pciePlconfIatuRegCtrl1Reg_t       ctrl1;
  pciePlconfIatuRegCtrl2Reg_t       ctrl2;
  pciePlconfIatuRegLowerBaseReg_t   lowerBase;
  pciePlconfIatuRegUpperBaseReg_t   upperBase;
  pciePlconfIatuRegLimitReg_t       limit;
  pciePlconfIatuRegLowerTargetReg_t lowerTarget;
  pciePlconfIatuRegUpperTargetReg_t upperTarget;
  pcieRegisters_t                   regs;

  /* Set up register pointer for interesting registers */
  memset (&regs, 0, sizeof(regs));
  regs.plconfIatuIndex       = &index;

  /* Read current values for index */
  retVal = Pciev1_readRegs (handle, location, &regs);
  if (retVal == pcie_RET_OK)
  {
    /* Update ATU index register with new region direction and region index.
    **/
    switch (atuRegionParams->regionDir)
    {
      /* translate arguments to avoid CSL in public header files */
      case PCIE_ATU_REGION_DIR_OUTBOUND:
        index.regionDirection = CSL_PLCONF_IATU_INDEX_REGION_DIRECTION_OUTBOUND;
        break;
      case PCIE_ATU_REGION_DIR_INBOUND:
      default:
        index.regionDirection = CSL_PLCONF_IATU_INDEX_REGION_DIRECTION_INBOUND;
        break;
    }
    index.regionIndex = atuRegionIndex;

    /* Writeback the new values for index */
    retVal = Pciev1_writeRegs (handle, location, &regs);
    if (retVal == pcie_RET_OK)
    {
      regs.plconfIatuIndex          = NULL;
      regs.plconfIatuRegCtrl1       = &ctrl1;
      regs.plconfIatuRegCtrl2       = &ctrl2;
      regs.plconfIatuRegLowerBase   = &lowerBase;
      regs.plconfIatuRegUpperBase   = &upperBase;
      regs.plconfIatuRegLimit       = &limit;
      regs.plconfIatuRegLowerTarget = &lowerTarget;
      regs.plconfIatuRegUpperTarget = &upperTarget;

      /* Read current values of rest of registers for this index */
      retVal = Pciev1_readRegs (handle, location, &regs);
      if (retVal == pcie_RET_OK)
      {
        /* Set TLP(Transaction Layer packet) type. */
        switch (atuRegionParams->tlpType)
        {
          case PCIE_TLP_TYPE_IO:
            ctrl1.type = 2U;
            break;
          case PCIE_TLP_TYPE_CFG:
            ctrl1.type = 4U;
            break;
          case PCIE_TLP_TYPE_MEM:
          default:
            ctrl1.type = 0U;
            break;
        }

        /* Configure ATU control2 register. */
        /* Enable region. */
        ctrl2.regionEnable = atuRegionParams->enableRegion;
        if (PCIE_ATU_REGION_DIR_INBOUND == atuRegionParams->regionDir)
        {
          /* Set match mode. */
          switch (atuRegionParams->matchMode)
          {
            case PCIE_ATU_REGION_MATCH_MODE_ADDR:
             ctrl2.matchMode = CSL_PLCONF_IATU_REG_CTRL_2_MATCH_MODE__0;
             break;
            case PCIE_ATU_REGION_MATCH_MODE_BAR:
            default:
             ctrl2.matchMode = CSL_PLCONF_IATU_REG_CTRL_2_MATCH_MODE__1;
             break;
          }

          /* Set BAR number. */
          ctrl2.barNumber = atuRegionParams->barNumber;
        }

        /* Configure lower base. */
        lowerBase.iatuRegLowerBase = atuRegionParams->lowerBaseAddr >> 12;

        /* Configure upper base. */
        upperBase.iatuRegUpperBase = atuRegionParams->upperBaseAddr;

        /* Configure window size. */
        limit.iatuRegLimit = (atuRegionParams->lowerBaseAddr +
                  atuRegionParams->regionWindowSize) >> 12;

        /* Configure lower target. */
        lowerTarget.iatuRegLowerTarget = atuRegionParams->lowerTargetAddr >> 12;

        /* Configure Upper target. */
        upperTarget.iatuRegUpperTarget = atuRegionParams->upperTargetAddr;

        /* Writeback the new values */
        retVal = Pciev1_writeRegs (handle, location, &regs);
      }
    }
  }
  return retVal;
} /* Pciev1_atuRegionConfig */

/*********************************************************************
 * FUNCTION PURPOSE: Get pending functional (MSI/legacy) interrupts
 ********************************************************************/
pcieRet_e Pciev1_getPendingFuncInts
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [out] rev-specific pending bits */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [out] rev-specific msi pending bits to check */
)
{
  pcieRet_e retVal;
  pcieTiConfIrqStatusMsiReg_t *statusReg = 
    (pcieTiConfIrqStatusMsiReg_t *)pendingBits;

  /* Get register pointer */
  Pcie_DeviceCfgBaseAddr    *cfg               = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases             = (Pciev1_DeviceCfgBaseAddrs*)cfg->cfgBase;

  /* Get the pending bits */
  retVal = pciev1_read_tiConfIrqStatusMsi_reg ((CSL_PcieRegs*)bases->tiConf, statusReg);
  if (retVal == pcie_RET_OK)
  {
    /* If MSI is pending find which one(s) */
    if ((statusReg->msi != 0) && (msiBits != 0)) 
    {
      int32_t i;
      int32_t n = sizeMsiBits / sizeof(pciePlconfMsiCtrlIntStatusReg_t);
      pciePlconfMsiCtrlIntStatusReg_t *swRegs = 
        (pciePlconfMsiCtrlIntStatusReg_t *)msiBits;
      for (i = 0; i < n; i++)
      {
        retVal = pciev1_read_plconfMsiCtrlIntStatus_reg (
                   (CSL_PlConfRegs*)bases->plConf, swRegs + i, i);
        if (retVal != pcie_RET_OK)
        {
          break;
        }
      }
    }
  }

  return retVal;
} /* Pciev1_getPendingFuncInts */

/*********************************************************************
 * FUNCTION PURPOSE: Clear pending functional (MSI/legacy) interrupts
 ********************************************************************/
pcieRet_e Pciev1_clrPendingFuncInts
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [in] rev-specific pending bits */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [in] rev-specific msi pending bits to ack */
)
{
  pcieRet_e retVal = pcie_RET_OK;
  pcieTiConfIrqStatusMsiReg_t *statusReg = 
    (pcieTiConfIrqStatusMsiReg_t *)pendingBits;

  /* Get register pointer */
  Pcie_DeviceCfgBaseAddr    *cfg               = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases             = (Pciev1_DeviceCfgBaseAddrs*)cfg->cfgBase;

  /* If MSI are provided, clear them (write 1 to clear) */
  if (msiBits) 
  {
    int32_t i;
    int32_t n = sizeMsiBits / sizeof(pciePlconfMsiCtrlIntStatusReg_t);
    pciePlconfMsiCtrlIntStatusReg_t *swRegs = 
      (pciePlconfMsiCtrlIntStatusReg_t *)msiBits;
    for (i = 0; i < n; i++)
    {
      retVal = pciev1_write_plconfMsiCtrlIntStatus_reg (
                 (CSL_PlConfRegs*)bases->plConf, swRegs + i, i);
      if (retVal != pcie_RET_OK)
      {
        break;
      }
    }
  }

  /* Clear the specified pending bits */
  if (retVal == pcie_RET_OK)
  {
    retVal = pciev1_write_tiConfIrqStatusMsi_reg ((CSL_PcieRegs*)bases->tiConf, statusReg);
  }

  return retVal;
} /* Pciev1_clrPendingFuncInts */

/* Nothing past this point */

