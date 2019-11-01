/*
 *
 * Copyright (C) 2010-2019 Texas Instruments Incorporated - http://www.ti.com/
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
 *  File Name: pciev2.c
 *
 *  Processing/configuration functions for the PCIe driver.
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v2/pcieloc.h>

#include <ti/csl/soc.h>

#include <string.h>

/* Local object simulating IATU window selection from V1.
 * No different in terms of re-entrancy compared to hw
 * reg in v1
 */
Pciev2_LocalObj pciev2LocalObj =
{
  {
    { 0u, 0u },
    { 0u, 0u }
  }
};

/*****************************************************************************
 * Set the mode of one interface without depending directly on device
 * dependant registers (via device.c)
 ****************************************************************************/
static void pcie_set_mode (Pciev2_DevParams *devParams, uint32_t index, pcieMode_e mode); /*for misra warning*/
static void pcie_set_mode (Pciev2_DevParams *devParams, uint32_t index, pcieMode_e mode)
{
  uint32_t get_value;
  uint32_t regVal;

  switch (mode)
  {
    case pcie_EP_MODE:
      regVal = 0U;
      break;
    case pcie_LEGACY_EP_MODE:
      regVal = 1U;
      break;
    case pcie_RC_MODE:
    default:
      regVal = 2U;
      break;
  }

  /* reset pcie -- moves to sciclient */
  *((volatile uint32_t *) (uintptr_t)(CSL_PSC0_BASE + 0x324)) = 0x00000001;
  *((volatile uint32_t *) (uintptr_t)(CSL_PSC0_BASE + 0xa4c + 0x4 * index)) = 0x00000101;
  *((volatile uint32_t *) (uintptr_t)(CSL_PSC0_BASE + 0x120)) = 0x00000200;

  get_value = ~0x00000000;
  while (get_value != 0x00000000)
  {
    get_value = *((volatile uint32_t *) (uintptr_t)(CSL_PSC0_BASE + 0x128));
  }

  *devParams->pcieSSModeAddr = regVal;

  /* un-reset pcie -- moves to sciclient */
  *((volatile uint32_t *) (uintptr_t)(CSL_PSC0_BASE + 0x324)) = 0x00000001;
  *((volatile uint32_t *) (uintptr_t)(CSL_PSC0_BASE + 0xa4c + 0x4 * index)) = 0x00000103;
  *((volatile uint32_t *) (uintptr_t)(CSL_PSC0_BASE + 0x120)) = 0x00000200;

  get_value = ~0x00000000;
  while (get_value != 0x00000000)
  {
    get_value = *((volatile uint32_t *) (uintptr_t)(CSL_PSC0_BASE + 0x128));
  }
} /* pcie_set_mode */

/*****************************************************************************
 **********  External APIs **********************
 ****************************************************************************/

/*********************************************************************
 * FUNCTION PURPOSE: Sets PCIe mode to RC or EP for interface
 * specified by handle
 *********************************************************************/
pcieRet_e Pciev2_setInterfaceMode
(
  Pcie_Handle handle,     /**< [in]  The PCIE LLD instance identifier */
  pcieMode_e  mode        /**< [in] PCIE Mode */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev2_DevParams *devParams;
  Pcie_IntHandle iHandle = (Pcie_IntHandle)handle;

  pcieRet_e retVal = pcie_RET_INV_HANDLE;

  if (cfg) {
    devParams = (Pciev2_DevParams *)cfg->devParams;
    if (devParams) {
      pcie_set_mode (devParams, iHandle->pcie_index, mode);
      retVal = pcie_RET_OK;
    }
  }

  return retVal;
} /* Pciev2_setInterfaceMode */

/*********************************************************************
 * FUNCTION PURPOSE: Returns amount of reserved space between beginning
 *                   of hardware's data area and the base returned
 *                   by @ref Pcie_getMemSpaceRange.  This enables
 *                   sw to position windows correctly
 *********************************************************************/
pcieRet_e Pciev2_getMemSpaceReserved
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
} /* Pciev2_getMemSpaceReserved */

/*********************************************************************
 * FUNCTION PURPOSE: Returns the PCIe Internal Address Range for the
 *                   Memory Space. This range is used for accessing memory.
 *********************************************************************/
pcieRet_e Pciev2_getMemSpaceRange
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
} /* Pciev2_getMemSpaceRange */

/*********************************************************************
 * FUNCTION PURPOSE: Reads any register
 ********************************************************************/
pcieRet_e Pciev2_readRegs
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *readRegs  /**< [in/out] List of registers to read */
)
{
  Pcie_IntHandle iHandle = (Pcie_IntHandle)handle;
  pciePlconfIatuIndexReg_t *simIatuWindow = &pciev2LocalObj.simIatuWindow[iHandle->pcie_index];
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev2_DeviceCfgBaseAddrs *bases = (Pciev2_DeviceCfgBaseAddrs*)cfg->cfgBase;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_pcie_rc_coreRegs *baseCfgRcRegs = (CSL_pcie_rc_coreRegs *)bases->cfgBase;
  CSL_pcie_ep_coreRegs *baseCfgEpRegs = (CSL_pcie_ep_coreRegs *)bases->cfgBase;

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
        baseCfgRcRegs     = (CSL_pcie_rc_coreRegs *)(remoteBase);
        baseCfgEpRegs     = (CSL_pcie_ep_coreRegs *)(remoteBase);
      }
    }
  }

  /*****************************************************************************************
  *Application Registers
  *****************************************************************************************/
  if ((retVal == pcie_RET_OK) && (readRegs->pid != NULL)) {
    retVal = pciev2_read_pid_reg (baseCfgEpRegs, readRegs->pid);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->cmdStatus != NULL)) {
    retVal = pciev2_read_cmdStatus_reg (baseCfgEpRegs, readRegs->cmdStatus);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->cfgTrans != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ioBase != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tlpCfg != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rstCmd != NULL)) {
    retVal = pciev2_read_rstCmd_reg (baseCfgEpRegs, readRegs->rstCmd);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ptmCfg != NULL)) {
    retVal = pciev2_read_ptmCfg_reg (baseCfgEpRegs, readRegs->ptmCfg);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmCmd != NULL)) {
    retVal = pciev2_read_pmCmd_reg (baseCfgEpRegs, readRegs->pmCmd);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmCfg != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->actStatus != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->obSize != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->diagCtrl != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->endian != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->priority != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->irqEOI != NULL)) {
    retVal = pciev2_read_irqEOI_reg (baseCfgEpRegs, readRegs->irqEOI);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiIrq != NULL)) {
    retVal = pciev2_read_msiIrq_reg (baseCfgEpRegs, readRegs->msiIrq);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->epIrqSet != NULL)) {
    retVal = pciev2_read_epIrqSet_reg (baseCfgEpRegs, readRegs->epIrqSet);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->epIrqClr != NULL)) {
    retVal = pciev2_read_epIrqClr_reg (baseCfgEpRegs, readRegs->epIrqClr);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->epIrqStatus != NULL)) {
    retVal = pciev2_read_epIrqStatus_reg (baseCfgEpRegs, readRegs->epIrqStatus);
  }
  for (i = 0; i < 4; i++) {
    if ((retVal == pcie_RET_OK) && (readRegs->genPurpose[i] != NULL)) {
      retVal = pciev2_read_genPurpose_reg (baseCfgEpRegs, readRegs->genPurpose[i], i);
    }
  }
  for (i = 0; i < 8; i++) {
    if ((retVal == pcie_RET_OK) && (readRegs->msiIrqStatusRaw[i] != NULL)) {
      retVal = pciev2_read_msiIrqStatusRaw_reg (baseCfgEpRegs, readRegs->msiIrqStatusRaw[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->msiIrqStatus[i] != NULL)) {
      retVal = pciev2_read_msiIrqStatus_reg (baseCfgEpRegs, readRegs->msiIrqStatus[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->msiIrqEnableSet[i] != NULL)) {
      retVal = pciev2_read_msiIrqEnableSet_reg (baseCfgEpRegs, readRegs->msiIrqEnableSet[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->msiIrqEnableClr[i] != NULL)) {
      retVal = pciev2_read_msiIrqEnableClr_reg (baseCfgEpRegs, readRegs->msiIrqEnableClr[i], i);
    }
  }
  for (i = 0; i < 4; i++) {
    if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqStatusRaw[i] != NULL)) {
      retVal = pciev2_read_legacyIrqStatusRaw_reg (baseCfgEpRegs, readRegs->legacyIrqStatusRaw[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqStatus[i] != NULL)) {
      retVal = pciev2_read_legacyIrqStatus_reg (baseCfgEpRegs, readRegs->legacyIrqStatus[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqEnableSet[i] != NULL)) {
      retVal = pciev2_read_legacyIrqEnableSet_reg (baseCfgEpRegs, readRegs->legacyIrqEnableSet[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqEnableClr[i] != NULL)) {
      retVal = pciev2_read_legacyIrqEnableClr_reg (baseCfgEpRegs, readRegs->legacyIrqEnableClr[i], i);
    }
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errIrqStatusRaw != NULL)) {
    retVal = pciev2_read_errIrqStatusRaw_reg (baseCfgEpRegs, readRegs->errIrqStatusRaw);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errIrqStatus != NULL)) {
    retVal = pciev2_read_errIrqStatus_reg (baseCfgEpRegs, readRegs->errIrqStatus);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errIrqEnableSet != NULL)) {
    retVal = pciev2_read_errIrqEnableSet_reg (baseCfgEpRegs, readRegs->errIrqEnableSet);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errIrqEnableClr != NULL)) {
    retVal = pciev2_read_errIrqEnableClr_reg (baseCfgEpRegs, readRegs->errIrqEnableClr);
  }

  if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqStatusRaw != NULL)) {
    retVal = pciev2_read_pmRstIrqStatusRaw_reg (baseCfgEpRegs, readRegs->pmRstIrqStatusRaw);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqStatus != NULL)) {
    retVal = pciev2_read_pmRstIrqStatus_reg (baseCfgEpRegs, readRegs->pmRstIrqStatus);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqEnableSet != NULL)) {
    retVal = pciev2_read_pmRstIrqEnableSet_reg (baseCfgEpRegs, readRegs->pmRstIrqEnableSet);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqEnableClr != NULL)) {
    retVal = pciev2_read_pmRstIrqEnableClr_reg (baseCfgEpRegs, readRegs->pmRstIrqEnableClr);
  }

  if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqStatusRaw != NULL)) {
    retVal = pciev2_read_ptmIrqStatusRaw_reg (baseCfgEpRegs, readRegs->ptmIrqStatusRaw);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqStatus != NULL)) {
    retVal = pciev2_read_ptmIrqStatus_reg (baseCfgEpRegs, readRegs->ptmIrqStatus);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqEnableSet != NULL)) {
    retVal = pciev2_read_ptmIrqEnableSet_reg (baseCfgEpRegs, readRegs->ptmIrqEnableSet);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqEnableClr != NULL)) {
    retVal = pciev2_read_ptmIrqEnableClr_reg (baseCfgEpRegs, readRegs->ptmIrqEnableClr);
  }

  for (i = 0; i < 8; i ++) {
    if ((retVal == pcie_RET_OK) && (readRegs->obOffsetLo[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->obOffsetHi[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
  }

  for (i = 0; i < 4; i ++) {
      if ((retVal == pcie_RET_OK) && (readRegs->ibBar[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->ibStartLo[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->ibStartHi[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (readRegs->ibOffset[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
  }

  if ((retVal == pcie_RET_OK) && (readRegs->pcsCfg0 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pcsCfg1 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pcsStatus != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }

  if ((retVal == pcie_RET_OK) && (readRegs->serdesCfg0 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->serdesCfg1 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }

  /*****************************************************************************************
  *Configuration Registers
  *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/

  if ((retVal == pcie_RET_OK) && (readRegs->vndDevId != NULL)) {
    retVal = pciev2_read_vndDevId_reg (&baseCfgEpRegs->DEVICE_ID_VENDOR_ID_REG, readRegs->vndDevId);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->statusCmd != NULL)) {
    retVal = pciev2_read_statusCmd_reg (&baseCfgEpRegs->STATUS_COMMAND_REG, readRegs->statusCmd);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->revId != NULL)) {
    retVal = pciev2_read_revId_reg (&baseCfgEpRegs->CLASS_CODE_REVISION_ID, readRegs->revId);
  }

  if ((retVal == pcie_RET_OK) && (readRegs->bist != NULL)) {
    retVal = pciev2_read_bist_reg (baseCfgEpRegs, readRegs->bist);
  }

  /*Type 0 Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->type0BarIdx != NULL)) {
    retVal = pciev2_read_type0Bar_reg (baseCfgEpRegs, &(readRegs->type0BarIdx->reg),
                                                        readRegs->type0BarIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type0Bar32bitIdx != NULL)) {
    retVal = pciev2_read_type0Bar32bit_reg (baseCfgEpRegs, &(readRegs->type0Bar32bitIdx->reg),
                                                             readRegs->type0Bar32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type0BarMask32bitIdx != NULL)) {
    retVal = pciev2_read_type0Bar32bit_reg (baseCfgEpRegs, &(readRegs->type0BarMask32bitIdx->reg),
                                                            readRegs->type0BarMask32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->subId != NULL)) {
    retVal = pciev2_read_subId_reg (baseCfgEpRegs, readRegs->subId);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->cardbusCisPointer != NULL)) {
    retVal = pciev2_read_cardbusCisPointer_reg (baseCfgEpRegs, readRegs->cardbusCisPointer);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->expRom != NULL)) {
    retVal = pciev2_read_expRom_reg (baseCfgEpRegs, readRegs->expRom);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->capPtr != NULL)) {
    retVal = pciev2_read_capPtr_reg (baseCfgEpRegs, readRegs->capPtr);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->intPin != NULL)) {
    retVal = pciev2_read_intPin_reg (baseCfgEpRegs, readRegs->intPin);
  }

  /*Type 1 Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->type1BistHeader != NULL)) {
    retVal = pciev2_read_type1BistHeader_reg (baseCfgRcRegs, readRegs->type1BistHeader);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1BarIdx != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1Bar32bitIdx != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1BarMask32bitIdx != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1BusNum != NULL)) {
    retVal = pciev2_read_type1BusNum_reg (baseCfgRcRegs, readRegs->type1BusNum);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1SecStat != NULL)) {
    retVal = pciev2_read_type1SecStat_reg (baseCfgRcRegs, readRegs->type1SecStat);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1Memspace != NULL)) {
    retVal = pciev2_read_type1Memspace_reg (baseCfgRcRegs, readRegs->type1Memspace);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->prefMem != NULL)) {
    retVal = pciev2_read_prefMem_reg (baseCfgRcRegs, readRegs->prefMem);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->prefBaseUpper != NULL)) {
    retVal = pciev2_read_prefBaseUpper_reg (baseCfgRcRegs, readRegs->prefBaseUpper);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->prefLimitUpper != NULL)) {
    retVal = pciev2_read_prefLimitUpper_reg (baseCfgRcRegs, readRegs->prefLimitUpper);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1IOSpace != NULL)) {
    retVal = pciev2_read_type1IOSpace_reg (baseCfgRcRegs, readRegs->type1IOSpace);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1CapPtr != NULL)) {
    retVal = pciev2_read_type1CapPtr_reg (baseCfgRcRegs, readRegs->type1CapPtr);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1ExpnsnRom != NULL)) {
    retVal = pciev2_read_type1ExpnsnRom_reg (baseCfgRcRegs, readRegs->type1ExpnsnRom);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->type1BridgeInt != NULL)) {
    retVal = pciev2_read_type1BridgeInt_reg (baseCfgRcRegs, readRegs->type1BridgeInt);
  }

  /* Power Management Capabilities Registers */
  if ((retVal == pcie_RET_OK) && (readRegs->pmCap != NULL)) {
    retVal = pciev2_read_pmCap_reg (baseCfgEpRegs, readRegs->pmCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->pmCapCtlStat != NULL)) {
    retVal = pciev2_read_pmCapCtlStat_reg (baseCfgEpRegs, readRegs->pmCapCtlStat);
  }

  /*MSI Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->msiCap != NULL)) {
    retVal = pciev2_read_msiCap_reg (baseCfgEpRegs, readRegs->msiCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiLo32 != NULL)) {
    retVal = pciev2_read_msiLo32_reg (baseCfgEpRegs, readRegs->msiLo32);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiUp32 != NULL)) {
    retVal = pciev2_read_msiUp32_reg (baseCfgEpRegs, readRegs->msiUp32);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiData != NULL)) {
    retVal = pciev2_read_msiData_reg (baseCfgEpRegs, readRegs->msiData);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiCapOff10H != NULL)) {
    retVal = pciev2_read_msiCapOff10H_reg (baseCfgEpRegs, readRegs->msiCapOff10H);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->msiCapOff14H != NULL)) {
    retVal = pciev2_read_msiCapOff14H_reg (baseCfgEpRegs, readRegs->msiCapOff14H);
  }

  /*Capabilities Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->pciesCap != NULL)) {
    retVal = pciev2_read_pciesCap_reg (&baseCfgEpRegs->PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG, readRegs->pciesCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->deviceCap != NULL)) {
    retVal = pciev2_read_deviceCap_reg (&baseCfgEpRegs->DEVICE_CAPABILITIES_REG, readRegs->deviceCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->devStatCtrl != NULL)) {
    retVal = pciev2_read_devStatCtrl_reg (&baseCfgEpRegs->DEVICE_CONTROL_DEVICE_STATUS, readRegs->devStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->linkCap != NULL)) {
    retVal = pciev2_read_linkCap_reg (&baseCfgEpRegs->LINK_CAPABILITIES_REG, readRegs->linkCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->linkStatCtrl != NULL)) {
    retVal = pciev2_read_linkStatCtrl_reg (&baseCfgEpRegs->LINK_CONTROL_LINK_STATUS_REG, readRegs->linkStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->slotCap != NULL)) {
    retVal = pciev2_read_slotCap_reg (baseCfgRcRegs, readRegs->slotCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->slotStatCtrl != NULL)) {
    retVal = pciev2_read_slotStatCtrl_reg (baseCfgRcRegs, readRegs->slotStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rootCtrlCap != NULL)) {
    retVal = pciev2_read_rootCtrlCap_reg (baseCfgRcRegs, readRegs->rootCtrlCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rootStatus != NULL)) {
    retVal = pciev2_read_rootStatus_reg (baseCfgRcRegs, readRegs->rootStatus);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->devCap2 != NULL)) {
    retVal = pciev2_read_devCap2_reg (&baseCfgEpRegs->DEVICE_CAPABILITIES2_REG, readRegs->devCap2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->devStatCtrl2 != NULL)) {
    retVal = pciev2_read_devStatCtrl2_reg (&baseCfgEpRegs->DEVICE_CONTROL2_DEVICE_STATUS2_REG, readRegs->devStatCtrl2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->linkCap2 != NULL)) {
    retVal = pciev2_read_linkCap2_reg (&baseCfgEpRegs->LINK_CAPABILITIES2_REG, readRegs->linkCap2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->linkCtrl2 != NULL)) {
    retVal = pciev2_read_linkCtrl2_reg (&baseCfgEpRegs->LINK_CONTROL2_LINK_STATUS2_REG, readRegs->linkCtrl2);
  }

  /*Capabilities Extended Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->extCap != NULL)) {
    retVal = pciev2_read_extCap_reg (baseCfgEpRegs, readRegs->extCap);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->uncErr != NULL)) {
    retVal = pciev2_read_uncErr_reg (baseCfgEpRegs, readRegs->uncErr);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->uncErrMask != NULL)) {
    retVal = pciev2_read_uncErrMask_reg (baseCfgEpRegs, readRegs->uncErrMask);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->uncErrSvrty != NULL)) {
    retVal = pciev2_read_uncErrSvrty_reg (baseCfgEpRegs, readRegs->uncErrSvrty);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->corErr != NULL)) {
    retVal = pciev2_read_corErr_reg (baseCfgEpRegs, readRegs->corErr);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->corErrMask != NULL)) {
    retVal = pciev2_read_corErrMask_reg (baseCfgEpRegs, readRegs->corErrMask);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->accr != NULL)) {
    retVal = pciev2_read_accr_reg (baseCfgEpRegs, readRegs->accr);
  }
  for (i = 0; i < 4; i ++) {
    if ((retVal == pcie_RET_OK) && (readRegs->hdrLog[i] != NULL)) {
      retVal = pciev2_read_hdrLog_reg (baseCfgEpRegs, readRegs->hdrLog[i], i);
    }
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rootErrCmd != NULL)) {
    retVal = pciev2_read_rootErrCmd_reg (baseCfgRcRegs, readRegs->rootErrCmd);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->rootErrSt != NULL)) {
    retVal = pciev2_read_rootErrSt_reg (baseCfgRcRegs, readRegs->rootErrSt);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->errSrcID != NULL)) {
    retVal = pciev2_read_errSrcID_reg (baseCfgRcRegs, readRegs->errSrcID);
  }

  /*Port Logic Registers*/
  if ((retVal == pcie_RET_OK) && (readRegs->plAckTimer != NULL)) {
    retVal = pciev2_read_plAckTimer_reg (baseCfgEpRegs, readRegs->plAckTimer);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plOMsg != NULL)) {
    retVal = pciev2_read_plOMsg_reg (baseCfgEpRegs, readRegs->plOMsg);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plForceLink != NULL)) {
    retVal = pciev2_read_plForceLink_reg (baseCfgEpRegs, readRegs->plForceLink);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->ackFreq != NULL)) {
    retVal = pciev2_read_ackFreq_reg (baseCfgEpRegs, readRegs->ackFreq);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->lnkCtrl != NULL)) {
    retVal = pciev2_read_lnkCtrl_reg (baseCfgEpRegs, readRegs->lnkCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->laneSkew != NULL)) {
    retVal = pciev2_read_laneSkew_reg (baseCfgEpRegs, readRegs->laneSkew);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->symNum != NULL)) {
    retVal = pciev2_read_symNum_reg (baseCfgEpRegs, readRegs->symNum);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->symTimerFltMask != NULL)) {
    retVal = pciev2_read_symTimerFltMask_reg (baseCfgEpRegs, readRegs->symTimerFltMask);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->fltMask2 != NULL)) {
    retVal = pciev2_read_fltMask2_reg (baseCfgEpRegs, readRegs->fltMask2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->debug0 != NULL)) {
    retVal = pciev2_read_debug0_reg (baseCfgEpRegs, readRegs->debug0);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->debug1 != NULL)) {
    retVal = pciev2_read_debug1_reg (baseCfgEpRegs, readRegs->debug1);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->gen2 != NULL)) {
    retVal = pciev2_read_gen2_reg (baseCfgEpRegs, readRegs->gen2);
  }

  /* hw rev 2 PLCONF registers */
  if ((retVal == pcie_RET_OK) && (readRegs->plconfObnpSubreqCtrl != NULL)) {
    retVal = pciev2_read_plconfObnpSubreqCtrl_reg (baseCfgEpRegs, readRegs->plconfObnpSubreqCtrl);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfTrPStsR != NULL)) {
    retVal = pciev2_read_plconfTrPStsR_reg (baseCfgEpRegs, readRegs->plconfTrPStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfTrNpStsR != NULL)) {
    retVal = pciev2_read_plconfTrNpStsR_reg (baseCfgEpRegs, readRegs->plconfTrNpStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfTrCStsR != NULL)) {
    retVal = pciev2_read_plconfTrCStsR_reg (baseCfgEpRegs, readRegs->plconfTrCStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfQStsR != NULL)) {
    retVal = pciev2_read_plconfQStsR_reg (baseCfgEpRegs, readRegs->plconfQStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVcTrAR1 != NULL)) {
    retVal = pciev2_read_plconfVcTrAR1_reg (baseCfgEpRegs, readRegs->plconfVcTrAR1);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVcTrAR2 != NULL)) {
    retVal = pciev2_read_plconfVcTrAR2_reg (baseCfgEpRegs, readRegs->plconfVcTrAR2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0PrQC != NULL)) {
    retVal = pciev2_read_plconfVcPrQC_reg (baseCfgEpRegs, readRegs->plconfVc0PrQC, 0);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0NprQC != NULL)) {
    retVal = pciev2_read_plconfVcNprQC_reg (baseCfgEpRegs, readRegs->plconfVc0NprQC, 0);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0CrQC != NULL)) {
    retVal = pciev2_read_plconfVcCrQC_reg (baseCfgEpRegs, readRegs->plconfVc0CrQC, 0);
  }
  for (i = 0; i < 3; i++)
  {
    if ((retVal == pcie_RET_OK) && (readRegs->plconfVcPrQC[i] != NULL)) {
      retVal = pciev2_read_plconfVcPrQC_reg (baseCfgEpRegs, readRegs->plconfVcPrQC[i], 1 + i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->plconfVcNprQC[i] != NULL)) {
      retVal = pciev2_read_plconfVcNprQC_reg (baseCfgEpRegs, readRegs->plconfVcNprQC[i], 1 + i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->plconfVcCrQC[i] != NULL)) {
      retVal = pciev2_read_plconfVcCrQC_reg (baseCfgEpRegs, readRegs->plconfVcCrQC[i], 1 + i);
    }
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfPhyStsR != NULL)) {
    retVal = pciev2_read_plconfPhyStsR_reg (baseCfgEpRegs, readRegs->plconfPhyStsR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfPhyCtrlR != NULL)) {
    retVal = pciev2_read_plconfPhyCtrlR_reg (baseCfgEpRegs, readRegs->plconfPhyCtrlR);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlAddress != NULL)) {
    retVal = pciev2_read_plconfMsiCtrlAddress_reg (baseCfgEpRegs, readRegs->plconfMsiCtrlAddress);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlUpperAddress != NULL)) {
    retVal = pciev2_read_plconfMsiCtrlUpperAddress_reg (baseCfgEpRegs, readRegs->plconfMsiCtrlUpperAddress);
  }
  for (i = 0; i < 8; i++) {
    if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntEnable[i] != NULL)) {
      retVal = pciev2_read_plconfMsiCtrlIntEnable_reg (baseCfgEpRegs, readRegs->plconfMsiCtrlIntEnable[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntMask[i] != NULL)) {
      retVal = pciev2_read_plconfMsiCtrlIntMask_reg (baseCfgEpRegs, readRegs->plconfMsiCtrlIntMask[i], i);
    }
    if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntStatus[i] != NULL)) {
      retVal = pciev2_read_plconfMsiCtrlIntStatus_reg (baseCfgEpRegs, readRegs->plconfMsiCtrlIntStatus[i], i);
    }
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlGpio != NULL)) {
    retVal = pciev2_read_plconfMsiCtrlGpio_reg (baseCfgEpRegs, readRegs->plconfMsiCtrlGpio);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfPipeLoopback != NULL)) {
    retVal = pciev2_read_plconfPipeLoopback_reg (baseCfgEpRegs, readRegs->plconfPipeLoopback);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfDbiRoWrEn != NULL)) {
    retVal = pciev2_read_plconfDbiRoWrEn_reg (baseCfgEpRegs, readRegs->plconfDbiRoWrEn);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfAxiSlvErrResp != NULL)) {
    retVal = pciev2_read_plconfAxiSlvErrResp_reg (baseCfgEpRegs, readRegs->plconfAxiSlvErrResp);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfAxiSlvTimeout != NULL)) {
    retVal = pciev2_read_plconfAxiSlvTimeout_reg (baseCfgEpRegs, readRegs->plconfAxiSlvTimeout);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuIndex != NULL)) {
    /* Return the simulated window address */
    *readRegs->plconfIatuIndex = *simIatuWindow;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl1 != NULL)) {
    retVal = pciev2_read_plconfIatuRegCtrl1_reg (baseCfgEpRegs, simIatuWindow, readRegs->plconfIatuRegCtrl1);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl2 != NULL)) {
    retVal = pciev2_read_plconfIatuRegCtrl2_reg (baseCfgEpRegs, simIatuWindow, readRegs->plconfIatuRegCtrl2);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLowerBase != NULL)) {
    retVal = pciev2_read_plconfIatuRegLowerBase_reg (baseCfgEpRegs, simIatuWindow, readRegs->plconfIatuRegLowerBase);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegUpperBase != NULL)) {
    retVal = pciev2_read_plconfIatuRegUpperBase_reg (baseCfgEpRegs, simIatuWindow, readRegs->plconfIatuRegUpperBase);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLimit != NULL)) {
    retVal = pciev2_read_plconfIatuRegLimit_reg (baseCfgEpRegs, simIatuWindow, readRegs->plconfIatuRegLimit);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLowerTarget != NULL)) {
    retVal = pciev2_read_plconfIatuRegLowerTarget_reg (baseCfgEpRegs, simIatuWindow, readRegs->plconfIatuRegLowerTarget);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegUpperTarget != NULL)) {
    retVal = pciev2_read_plconfIatuRegUpperTarget_reg (baseCfgEpRegs, simIatuWindow, readRegs->plconfIatuRegUpperTarget);
  }
  if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl3 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }


  /* TI CONF registers */
  /* reject hw rev 2 TI CONF registers */
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfRevision != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfSysConfig != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEoi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusRawMain != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusMain != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableSetMain != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableClrMain != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusRawMsi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusMsi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableSetMsi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableClrMsi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDeviceType != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDeviceCmd != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfPmCtrl != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfPhyCs != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIntxAssert != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfIntxDeassert != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfMsiXmt != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDebugCfg != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDebugData != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (readRegs->tiConfDiagCtrl != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }

  return retVal;
} /* Pciev2_readRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Writes any register
 ********************************************************************/
pcieRet_e Pciev2_writeRegs
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *writeRegs /**< [in] List of registers to write */
)
{
  Pcie_IntHandle iHandle = (Pcie_IntHandle)handle;
  pciePlconfIatuIndexReg_t *simIatuWindow = &pciev2LocalObj.simIatuWindow[iHandle->pcie_index];
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev2_DeviceCfgBaseAddrs *bases = (Pciev2_DeviceCfgBaseAddrs*)cfg->cfgBase;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_pcie_rc_coreRegs *baseCfgRcRegs = (CSL_pcie_rc_coreRegs *)bases->cfgBase;
  CSL_pcie_ep_coreRegs *baseCfgEpRegs = (CSL_pcie_ep_coreRegs *)bases->cfgBase;

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
        baseCfgRcRegs     = (CSL_pcie_rc_coreRegs *)(remoteBase);
        baseCfgEpRegs     = (CSL_pcie_ep_coreRegs *)(remoteBase);
      }
    }
  }
  /*****************************************************************************************
  * Reject hw rev 0 app registers (these are similar but not identical to TI CONF on rev 2)
  *****************************************************************************************/
  if ((retVal == pcie_RET_OK) && (writeRegs->cmdStatus != NULL)) {
    retVal = pciev2_write_cmdStatus_reg (baseCfgEpRegs, writeRegs->cmdStatus);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->cfgTrans != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ioBase != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tlpCfg != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rstCmd != NULL)) {
    retVal = pciev2_write_rstCmd_reg (baseCfgEpRegs, writeRegs->rstCmd);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ptmCfg != NULL)) {
    retVal = pciev2_write_ptmCfg_reg (baseCfgEpRegs, writeRegs->ptmCfg);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmCmd != NULL)) {
    retVal = pciev2_write_pmCmd_reg (baseCfgEpRegs, writeRegs->pmCmd);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmCfg != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->obSize != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->diagCtrl != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->endian != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->priority != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->irqEOI != NULL)) {
    retVal = pciev2_write_irqEOI_reg (baseCfgEpRegs, writeRegs->irqEOI);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiIrq != NULL)) {
    retVal = pciev2_write_msiIrq_reg (baseCfgEpRegs, writeRegs->msiIrq);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->epIrqSet != NULL)) {
    retVal = pciev2_write_epIrqSet_reg (baseCfgEpRegs, writeRegs->epIrqSet);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->epIrqClr != NULL)) {
    retVal = pciev2_write_epIrqClr_reg (baseCfgEpRegs, writeRegs->epIrqClr);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->epIrqStatus != NULL)) {
    retVal = pciev2_write_epIrqStatus_reg (baseCfgEpRegs, writeRegs->epIrqStatus);
  }
  for (i = 0; i < 4; i++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->genPurpose[i] != NULL)) {
      retVal = pciev2_write_genPurpose_reg (baseCfgEpRegs, writeRegs->genPurpose[i], i);
    }
  }
  for (i = 0; i < 8; i++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqStatusRaw[i] != NULL)) {
      retVal = pciev2_write_msiIrqStatusRaw_reg (baseCfgEpRegs, writeRegs->msiIrqStatusRaw[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqStatus[i] != NULL)) {
      retVal = pciev2_write_msiIrqStatus_reg (baseCfgEpRegs, writeRegs->msiIrqStatus[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqEnableSet[i] != NULL)) {
      retVal = pciev2_write_msiIrqEnableSet_reg (baseCfgEpRegs, writeRegs->msiIrqEnableSet[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqEnableClr[i] != NULL)) {
      retVal = pciev2_write_msiIrqEnableClr_reg (baseCfgEpRegs, writeRegs->msiIrqEnableClr[i], i);
    }
  }
  for (i = 0; i < 4; i++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqStatusRaw[i] != NULL)) {
      retVal = pciev2_write_legacyIrqStatusRaw_reg (baseCfgEpRegs, writeRegs->legacyIrqStatusRaw[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqStatus[i] != NULL)) {
      retVal = pciev2_write_legacyIrqStatus_reg (baseCfgEpRegs, writeRegs->legacyIrqStatus[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqEnableSet[i] != NULL)) {
      retVal = pciev2_write_legacyIrqEnableSet_reg (baseCfgEpRegs, writeRegs->legacyIrqEnableSet[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqEnableClr[i] != NULL)) {
      retVal = pciev2_write_legacyIrqEnableClr_reg (baseCfgEpRegs, writeRegs->legacyIrqEnableClr[i], i);
    }
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errIrqStatusRaw != NULL)) {
    retVal = pciev2_write_errIrqStatusRaw_reg (baseCfgEpRegs, writeRegs->errIrqStatusRaw);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errIrqStatus != NULL)) {
    retVal = pciev2_write_errIrqStatus_reg (baseCfgEpRegs, writeRegs->errIrqStatus);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errIrqEnableSet != NULL)) {
    retVal = pciev2_write_errIrqEnableSet_reg (baseCfgEpRegs, writeRegs->errIrqEnableSet);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errIrqEnableClr != NULL)) {
    retVal = pciev2_write_errIrqEnableClr_reg (baseCfgEpRegs, writeRegs->errIrqEnableClr);
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqStatusRaw != NULL)) {
    retVal = pciev2_write_pmRstIrqStatusRaw_reg (baseCfgEpRegs, writeRegs->pmRstIrqStatusRaw);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqStatus != NULL)) {
    retVal = pciev2_write_pmRstIrqStatus_reg (baseCfgEpRegs, writeRegs->pmRstIrqStatus);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqEnableSet != NULL)) {
    retVal = pciev2_write_pmRstIrqEnableSet_reg (baseCfgEpRegs, writeRegs->pmRstIrqEnableSet);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqEnableClr != NULL)) {
    retVal = pciev2_write_pmRstIrqEnableClr_reg (baseCfgEpRegs, writeRegs->pmRstIrqEnableClr);
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqStatusRaw != NULL)) {
    retVal = pciev2_write_ptmIrqStatusRaw_reg (baseCfgEpRegs, writeRegs->ptmIrqStatusRaw);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqStatus != NULL)) {
    retVal = pciev2_write_ptmIrqStatus_reg (baseCfgEpRegs, writeRegs->ptmIrqStatus);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqEnableSet != NULL)) {
    retVal = pciev2_write_ptmIrqEnableSet_reg (baseCfgEpRegs, writeRegs->ptmIrqEnableSet);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqEnableClr != NULL)) {
    retVal = pciev2_write_ptmIrqEnableClr_reg (baseCfgEpRegs, writeRegs->ptmIrqEnableClr);
  }

  for (i = 0; i < 8; i ++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->obOffsetLo[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->obOffsetHi[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
  }

  for (i = 0; i < 4; i ++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->ibBar[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->ibStartLo[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->ibStartHi[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->ibOffset[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->pcsCfg0 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pcsCfg1 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->serdesCfg0 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->serdesCfg1 != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }

  /*****************************************************************************************
  *Configuration Registers
  *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/

  if ((retVal == pcie_RET_OK) && (writeRegs->vndDevId != NULL)) {
    retVal = pciev2_write_vndDevId_reg (&baseCfgEpRegs->DEVICE_ID_VENDOR_ID_REG, writeRegs->vndDevId);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->statusCmd != NULL)) {
    retVal = pciev2_write_statusCmd_reg (&baseCfgEpRegs->STATUS_COMMAND_REG, writeRegs->statusCmd);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->revId != NULL)) {
    retVal = pciev2_write_revId_reg (&baseCfgEpRegs->CLASS_CODE_REVISION_ID, writeRegs->revId);
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->bist != NULL)) {
    retVal = pciev2_write_bist_reg (baseCfgEpRegs, writeRegs->bist);
  }

  /*Type 0 Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->type0BarIdx != NULL)) {
    retVal = pciev2_write_type0Bar_reg (baseCfgEpRegs, &(writeRegs->type0BarIdx->reg),
                                                         writeRegs->type0BarIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type0BarMask32bitIdx != NULL)) {
    retVal = pciev2_write_type0Bar32bit_reg (baseCfgEpRegs, &(writeRegs->type0BarMask32bitIdx->reg),
                                                              writeRegs->type0BarMask32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type0Bar32bitIdx != NULL)) {
    retVal = pciev2_write_type0Bar32bit_reg (baseCfgEpRegs, &(writeRegs->type0Bar32bitIdx->reg),
                                                              writeRegs->type0Bar32bitIdx->idx);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->subId != NULL)) {
    retVal = pciev2_write_subId_reg (baseCfgEpRegs, writeRegs->subId);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->cardbusCisPointer != NULL)) {
    retVal = pciev2_write_cardbusCisPointer_reg (baseCfgEpRegs, writeRegs->cardbusCisPointer);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->expRom != NULL)) {
    retVal = pciev2_write_expRom_reg (baseCfgEpRegs, writeRegs->expRom);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->capPtr != NULL)) {
    retVal = pciev2_write_capPtr_reg (baseCfgEpRegs, writeRegs->capPtr);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->intPin != NULL)) {
    retVal = pciev2_write_intPin_reg (baseCfgEpRegs, writeRegs->intPin);
  }

  /*Type 1 Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BistHeader != NULL)) {
    retVal = pciev2_write_type1BistHeader_reg (baseCfgRcRegs, writeRegs->type1BistHeader);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BarIdx != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BarMask32bitIdx != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1Bar32bitIdx != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BusNum != NULL)) {
    retVal = pciev2_write_type1BusNum_reg (baseCfgRcRegs, writeRegs->type1BusNum);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1SecStat != NULL)) {
    retVal = pciev2_write_type1SecStat_reg (baseCfgRcRegs, writeRegs->type1SecStat);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1Memspace != NULL)) {
    retVal = pciev2_write_type1Memspace_reg (baseCfgRcRegs, writeRegs->type1Memspace);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->prefMem != NULL)) {
    retVal = pciev2_write_prefMem_reg (baseCfgRcRegs, writeRegs->prefMem);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->prefBaseUpper != NULL)) {
    retVal = pciev2_write_prefBaseUpper_reg (baseCfgRcRegs, writeRegs->prefBaseUpper);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->prefLimitUpper != NULL)) {
    retVal = pciev2_write_prefLimitUpper_reg (baseCfgRcRegs, writeRegs->prefLimitUpper);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1IOSpace != NULL)) {
    retVal = pciev2_write_type1IOSpace_reg (baseCfgRcRegs, writeRegs->type1IOSpace);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1CapPtr != NULL)) {
    retVal = pciev2_write_type1CapPtr_reg (baseCfgRcRegs, writeRegs->type1CapPtr);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1ExpnsnRom != NULL)) {
    retVal = pciev2_write_type1ExpnsnRom_reg (baseCfgRcRegs, writeRegs->type1ExpnsnRom);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->type1BridgeInt != NULL)) {
    retVal = pciev2_write_type1BridgeInt_reg (baseCfgRcRegs, writeRegs->type1BridgeInt);
  }

  /* Power Management Capabilities Registers */
  if ((retVal == pcie_RET_OK) && (writeRegs->pmCap != NULL)) {
    retVal = pciev2_write_pmCap_reg (baseCfgEpRegs, writeRegs->pmCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->pmCapCtlStat != NULL)) {
    retVal = pciev2_write_pmCapCtlStat_reg (baseCfgEpRegs, writeRegs->pmCapCtlStat);
  }

  /*MSI Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->msiCap != NULL)) {
    retVal = pciev2_write_msiCap_reg (baseCfgEpRegs, writeRegs->msiCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiLo32 != NULL)) {
    retVal = pciev2_write_msiLo32_reg (baseCfgEpRegs, writeRegs->msiLo32);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiUp32 != NULL)) {
    retVal = pciev2_write_msiUp32_reg (baseCfgEpRegs, writeRegs->msiUp32);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiData != NULL)) {
    retVal = pciev2_write_msiData_reg (baseCfgEpRegs, writeRegs->msiData);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiCapOff10H != NULL)) {
    retVal = pciev2_write_msiCapOff10H_reg (baseCfgEpRegs, writeRegs->msiCapOff10H);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->msiCapOff14H != NULL)) {
    retVal = pciev2_write_msiCapOff10H_reg (baseCfgEpRegs, writeRegs->msiCapOff10H);
  }

  /*Capabilities Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->pciesCap != NULL)) {
    retVal = pciev2_write_pciesCap_reg (&baseCfgEpRegs->PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG, writeRegs->pciesCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->deviceCap != NULL)) {
    retVal = pciev2_write_deviceCap_reg (&baseCfgEpRegs->DEVICE_CAPABILITIES_REG, writeRegs->deviceCap);
  }

  if ((retVal == pcie_RET_OK) && (writeRegs->devStatCtrl != NULL)) {
    retVal = pciev2_write_devStatCtrl_reg (&baseCfgEpRegs->DEVICE_CONTROL_DEVICE_STATUS, writeRegs->devStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->linkCap != NULL)) {
    retVal = pciev2_write_linkCap_reg (&baseCfgEpRegs->LINK_CAPABILITIES_REG, writeRegs->linkCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->linkStatCtrl != NULL)) {
    retVal = pciev2_write_linkStatCtrl_reg (&baseCfgEpRegs->LINK_CONTROL_LINK_STATUS_REG, writeRegs->linkStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->slotCap != NULL)) {
    retVal = pciev2_write_slotCap_reg (baseCfgRcRegs, writeRegs->slotCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->slotStatCtrl != NULL)) {
    retVal = pciev2_write_slotStatCtrl_reg (baseCfgRcRegs, writeRegs->slotStatCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rootCtrlCap != NULL)) {
    retVal = pciev2_write_rootCtrlCap_reg (baseCfgRcRegs, writeRegs->rootCtrlCap);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rootStatus != NULL)) {
    retVal = pciev2_write_rootStatus_reg (baseCfgRcRegs, writeRegs->rootStatus);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->devCap2 != NULL)) {
    retVal = pciev2_write_devCap2_reg (&baseCfgEpRegs->DEVICE_CAPABILITIES2_REG, writeRegs->devCap2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->devStatCtrl2 != NULL)) {
    retVal = pciev2_write_devStatCtrl2_reg (&baseCfgEpRegs->DEVICE_CONTROL2_DEVICE_STATUS2_REG, writeRegs->devStatCtrl2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->linkCap2 != NULL)) {
    retVal = pciev2_write_linkCap2_reg (&baseCfgEpRegs->LINK_CAPABILITIES2_REG, writeRegs->linkCap2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->linkCtrl2 != NULL)) {
    retVal = pciev2_write_linkCtrl2_reg (&baseCfgEpRegs->LINK_CONTROL2_LINK_STATUS2_REG, writeRegs->linkCtrl2);
  }

  /*Capabilities Extended Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->extCap != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->uncErr != NULL)) {
        retVal = pciev2_write_uncErr_reg (baseCfgEpRegs, writeRegs->uncErr);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->uncErrMask != NULL)) {
        retVal = pciev2_write_uncErrMask_reg (baseCfgEpRegs, writeRegs->uncErrMask);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->uncErrSvrty != NULL)) {
        retVal = pciev2_write_uncErrSvrty_reg (baseCfgEpRegs, writeRegs->uncErrSvrty);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->corErr != NULL)) {
        retVal = pciev2_write_corErr_reg (baseCfgEpRegs, writeRegs->corErr);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->corErrMask != NULL)) {
        retVal = pciev2_write_corErrMask_reg (baseCfgEpRegs, writeRegs->corErrMask);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->accr != NULL)) {
        retVal = pciev2_write_accr_reg (baseCfgEpRegs, writeRegs->accr);
  }
  for (i = 0; i < 4; i ++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->hdrLog[i] != NULL)) {
      /* Not supported on rev 2 */
      retVal = pcie_RET_INV_REG;
    }
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rootErrCmd != NULL)) {
        retVal = pciev2_write_rootErrCmd_reg (baseCfgRcRegs, writeRegs->rootErrCmd);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->rootErrSt != NULL)) {
        retVal = pciev2_write_rootErrSt_reg (baseCfgRcRegs, writeRegs->rootErrSt);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->errSrcID != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }

  /*Port Logic Registers*/
  if ((retVal == pcie_RET_OK) && (writeRegs->plAckTimer != NULL)) {
    retVal = pciev2_write_plAckTimer_reg (baseCfgEpRegs, writeRegs->plAckTimer);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plOMsg != NULL)) {
    retVal = pciev2_write_plOMsg_reg (baseCfgEpRegs, writeRegs->plOMsg);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plForceLink != NULL)) {
    retVal = pciev2_write_plForceLink_reg (baseCfgEpRegs, writeRegs->plForceLink);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->ackFreq != NULL)) {
    retVal = pciev2_write_ackFreq_reg (baseCfgEpRegs, writeRegs->ackFreq);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->lnkCtrl != NULL)) {
    retVal = pciev2_write_lnkCtrl_reg (baseCfgEpRegs, writeRegs->lnkCtrl);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->laneSkew != NULL)) {
    retVal = pciev2_write_laneSkew_reg (baseCfgEpRegs, writeRegs->laneSkew);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->symNum != NULL)) {
    retVal = pciev2_write_symNum_reg (baseCfgEpRegs, writeRegs->symNum);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->symTimerFltMask != NULL)) {
    retVal = pciev2_write_symTimerFltMask_reg (baseCfgEpRegs, writeRegs->symTimerFltMask);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->fltMask2 != NULL)) {
    retVal = pciev2_write_fltMask2_reg (baseCfgEpRegs, writeRegs->fltMask2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->gen2 != NULL)) {
    retVal = pciev2_write_gen2_reg (baseCfgEpRegs, writeRegs->gen2);
  }

  /* hw rev 2 PLCONF registers */
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfObnpSubreqCtrl != NULL)) {
    retVal = pciev2_write_plconfObnpSubreqCtrl_reg (baseCfgEpRegs, writeRegs->plconfObnpSubreqCtrl);
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
    retVal = pciev2_write_plconfQStsR_reg (baseCfgEpRegs, writeRegs->plconfQStsR);
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
    retVal = pciev2_write_plconfVcPrQC_reg (baseCfgEpRegs, writeRegs->plconfVc0PrQC, 0);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfVc0NprQC != NULL)) {
    retVal = pciev2_write_plconfVcNprQC_reg (baseCfgEpRegs, writeRegs->plconfVc0NprQC, 0);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfVc0CrQC != NULL)) {
    retVal = pciev2_write_plconfVcCrQC_reg (baseCfgEpRegs, writeRegs->plconfVc0CrQC, 0);
  }
  for (i = 0; i < 3; i++)
  {
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcPrQC[i] != NULL)) {
      retVal = pciev2_write_plconfVcPrQC_reg (baseCfgEpRegs, writeRegs->plconfVcPrQC[i], 1 + i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcNprQC[i] != NULL)) {
      retVal = pciev2_write_plconfVcNprQC_reg (baseCfgEpRegs, writeRegs->plconfVcNprQC[i], 1 + i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcCrQC[i] != NULL)) {
      retVal = pciev2_write_plconfVcCrQC_reg (baseCfgEpRegs, writeRegs->plconfVcCrQC[i], 1 + i);
    }
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfPhyStsR != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfPhyCtrlR != NULL)) {
    retVal = pciev2_write_plconfPhyCtrlR_reg (baseCfgEpRegs, writeRegs->plconfPhyCtrlR);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlAddress != NULL)) {
    retVal = pciev2_write_plconfMsiCtrlAddress_reg (baseCfgEpRegs, writeRegs->plconfMsiCtrlAddress);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlUpperAddress != NULL)) {
    retVal = pciev2_write_plconfMsiCtrlUpperAddress_reg (baseCfgEpRegs, writeRegs->plconfMsiCtrlUpperAddress);
  }
  for (i = 0; i < 8; i++) {
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntEnable[i] != NULL)) {
      retVal = pciev2_write_plconfMsiCtrlIntEnable_reg (baseCfgEpRegs, writeRegs->plconfMsiCtrlIntEnable[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntMask[i] != NULL)) {
      retVal = pciev2_write_plconfMsiCtrlIntMask_reg (baseCfgEpRegs, writeRegs->plconfMsiCtrlIntMask[i], i);
    }
    if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntStatus[i] != NULL)) {
      retVal = pciev2_write_plconfMsiCtrlIntStatus_reg (baseCfgEpRegs, writeRegs->plconfMsiCtrlIntStatus[i], i);
    }
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlGpio != NULL)) {
    retVal = pciev2_write_plconfMsiCtrlGpio_reg (baseCfgEpRegs, writeRegs->plconfMsiCtrlGpio);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfPipeLoopback != NULL)) {
    retVal = pciev2_write_plconfPipeLoopback_reg (baseCfgEpRegs, writeRegs->plconfPipeLoopback);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfDbiRoWrEn != NULL)) {
    retVal = pciev2_write_plconfDbiRoWrEn_reg (baseCfgEpRegs, writeRegs->plconfDbiRoWrEn);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfAxiSlvErrResp != NULL)) {
    retVal = pciev2_write_plconfAxiSlvErrResp_reg (baseCfgEpRegs, writeRegs->plconfAxiSlvErrResp);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfAxiSlvTimeout != NULL)) {
    retVal = pciev2_write_plconfAxiSlvTimeout_reg (baseCfgEpRegs, writeRegs->plconfAxiSlvTimeout);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuIndex != NULL)) {
    /* Set the simulated window address */
    *simIatuWindow = *writeRegs->plconfIatuIndex;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl2 != NULL)) {
    retVal = pciev2_write_plconfIatuRegCtrl2_reg (baseCfgEpRegs, simIatuWindow, writeRegs->plconfIatuRegCtrl2);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLowerBase != NULL)) {
    retVal = pciev2_write_plconfIatuRegLowerBase_reg (baseCfgEpRegs, simIatuWindow, writeRegs->plconfIatuRegLowerBase);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegUpperBase != NULL)) {
    retVal = pciev2_write_plconfIatuRegUpperBase_reg (baseCfgEpRegs, simIatuWindow, writeRegs->plconfIatuRegUpperBase);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLimit != NULL)) {
    retVal = pciev2_write_plconfIatuRegLimit_reg (baseCfgEpRegs, simIatuWindow, writeRegs->plconfIatuRegLimit);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLowerTarget != NULL)) {
    retVal = pciev2_write_plconfIatuRegLowerTarget_reg (baseCfgEpRegs, simIatuWindow, writeRegs->plconfIatuRegLowerTarget);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegUpperTarget != NULL)) {
    retVal = pciev2_write_plconfIatuRegUpperTarget_reg (baseCfgEpRegs, simIatuWindow, writeRegs->plconfIatuRegUpperTarget);
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl3 != NULL)) {
    /* Pure RO register */
    retVal = pcie_RET_INV_REG;
  }
  /* Ctrl1 is done last since it has enable bit */
  if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl1 != NULL)) {
    retVal = pciev2_write_plconfIatuRegCtrl1_reg (baseCfgEpRegs, simIatuWindow, writeRegs->plconfIatuRegCtrl1);
  }

  /* Reject hw rev 2 TI CONF registers */
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfRevision != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfSysConfig != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEoi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusRawMain != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusMain != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableSetMain != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableClrMain != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusRawMsi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusMsi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableSetMsi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableClrMsi != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDeviceType != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDeviceCmd != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfPmCtrl != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfPhyCs != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIntxAssert != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIntxDeassert != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfMsiXmt != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDebugCfg != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDebugData != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }
  if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDiagCtrl != NULL)) {
    /* Not supported on rev 2 */
    retVal = pcie_RET_INV_REG;
  }

  return retVal;
} /* Pciev2_writeRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Configures a BAR Register (32bits)
 ********************************************************************/
pcieRet_e Pciev2_cfgBar
(
  Pcie_Handle            handle,   /**< [in] The PCIE LLD instance identifier */
  const pcieBarCfg_t    *barCfg    /**< [in] BAR configuration parameters */
)
{
  pcieRet_e          retVal = pcie_RET_OK;
  pcieType0BarIdx_t  type0BarIdx;
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

      if(barCfg->mode == pcie_RC_MODE)
      {
         retVal = pcie_RET_UNSUPPORTED;
      }
      else
      {
        pcie_getbits(barCfg->base, CSL_PCIE_EP_CORE_BAR_REG_BAR_START, barAddrField);

        type0BarIdx.reg.base     = barAddrField;
        type0BarIdx.reg.prefetch = barCfg->prefetch;
        type0BarIdx.reg.type     = barCfg->type;
        type0BarIdx.reg.memSpace = barCfg->memSpace;
        type0BarIdx.idx          = barCfg->idx;

        setRegs.type0BarIdx = &type0BarIdx;

        retVal = Pciev2_writeRegs (handle, barCfg->location, &setRegs);
      }
    }
  }
  return retVal;
} /* Pciev2_cfgBar */


/*********************************************************************
 * FUNCTION PURPOSE: Configures an ATU (address translation) region
 ********************************************************************/
pcieRet_e Pciev2_atuRegionConfig
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
  retVal = Pciev2_readRegs (handle, location, &regs);
  if (retVal == pcie_RET_OK)
  {
    /* Update ATU index register with new region direction and region index.
    **/
    switch (atuRegionParams->regionDir)
    {
      case PCIE_ATU_REGION_DIR_OUTBOUND:
        index.regionDirection = 0U; /* Outbound - emulates v1 CSL */
        break;
      case PCIE_ATU_REGION_DIR_INBOUND:
      default:
        index.regionDirection = 1U; /* Inbound - emulates v1 CSL */
        break;
    }
    index.regionIndex = atuRegionIndex;

    /* Writeback the new values for index */
    retVal = Pciev2_writeRegs (handle, location, &regs);
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
      retVal = Pciev2_readRegs (handle, location, &regs);
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
             ctrl2.matchMode = 0u;
             break;
            case PCIE_ATU_REGION_MATCH_MODE_BAR:
            default:
             ctrl2.matchMode = 1u;
             break;
          }

          /* Set BAR number. */
          ctrl2.barNumber = atuRegionParams->barNumber;
        }

        /* Configure lower base. */
        lowerBase.iatuRegLowerBase = atuRegionParams->lowerBaseAddr >> 16;

        /* Configure upper base. */
        upperBase.iatuRegUpperBase = atuRegionParams->upperBaseAddr;

        /* Configure window size. */
        limit.iatuRegLimit = (atuRegionParams->lowerBaseAddr +
                  atuRegionParams->regionWindowSize) >> 16;

        /* Configure lower target. */
        lowerTarget.iatuRegLowerTarget = atuRegionParams->lowerTargetAddr >> 16;

        /* Configure Upper target. */
        upperTarget.iatuRegUpperTarget = atuRegionParams->upperTargetAddr;

        /* Writeback the new values */
        retVal = Pciev2_writeRegs (handle, location, &regs);
      }
    }
  }
  return retVal;
} /* Pciev2_atuRegionConfig */

/*********************************************************************
 * FUNCTION PURPOSE: Get pending functional (MSI/legacy) interrupts
 ********************************************************************/
pcieRet_e Pciev2_getPendingFuncInts
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [out] rev-specific pending bits */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [out] rev-specific msi pending bits to check */
)
{
  pcieRet_e retVal = pcie_RET_OK;

  /* As all MSI or Legacy interrupts can be routed to individual
   * SPI or LPI, there is no need for special demux here */
  return retVal;
} /* Pciev2_getPendingFuncInts */

/*********************************************************************
 * FUNCTION PURPOSE: Clear pending functional (MSI/legacy) interrupts
 ********************************************************************/
pcieRet_e Pciev2_clrPendingFuncInts
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [in] rev-specific pending bits */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [in] rev-specific msi pending bits to ack */
)
{
  pcieRet_e retVal = pcie_RET_OK;

  /* As all MSI or Legacy interrupts can be routed to individual
   * SPI or LPI, there is no need for special clear here */
  return retVal;
} /* Pciev2_clrPendingFuncInts */

/* Nothing past this point */

