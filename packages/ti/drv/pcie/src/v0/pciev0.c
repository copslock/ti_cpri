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
 *  File Name: pciev0.c
 *
 *  Processing/configuration functions for the PCIe driver.
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v0/pcieloc.h>

#include <string.h>

/*****************************************************************************
 * Set the mode of one interface without depending directly on device 
 * dependant registers (via device.c)
 ****************************************************************************/
static void pcie_set_mode (const Pciev0_DevParams *iface, pcieMode_e mode); /*for misra warning*/
static void pcie_set_mode (const Pciev0_DevParams *iface, pcieMode_e mode)
{
  uint32_t modeReg;
  uint32_t newMode = (uint32_t)mode;

  /* Set the mode, without depending on device specific csl files */
  modeReg = *iface->pcieSSModeAddr;
  modeReg &= ~iface->pcieSSModeMask;
  newMode <<= iface->pcieSSModeShift;
  newMode &= iface->pcieSSModeMask;
  modeReg |= newMode;
  *iface->pcieSSModeAddr = modeReg;
} /* pcie_set_mode */

/*****************************************************************************
 **********  External APIs **********************
 ****************************************************************************/

/*********************************************************************
 * FUNCTION PURPOSE: Sets PCIe mode to RC or EP for interface
 * specified by handle
 *********************************************************************/
pcieRet_e Pciev0_setInterfaceMode
(
  Pcie_Handle handle,     /**< [in]  The PCIE LLD instance identifier */
  pcieMode_e  mode        /**< [in] PCIE Mode */
)
{
  pcieRet_e ret_val;
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);

  if (cfg) {
     pcie_set_mode ((Pciev0_DevParams *)cfg->devParams, mode);
     ret_val = pcie_RET_OK;
  }
  else{
     ret_val = pcie_RET_INV_HANDLE;
  }
  return ret_val;
} /* Pciev0_setInterfaceMode */


/*********************************************************************
 * FUNCTION PURPOSE: Returns amount of reserved space between beginning
 *                   of hardware's data area and the base returned
 *                   by @ref Pcie_getMemSpaceRange.  This enables
 *                   sw to position windows correctly
 *********************************************************************/
pcieRet_e Pciev0_getMemSpaceReserved 
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
} /* Pciev0_getMemSpaceReserved */


/*********************************************************************
 * FUNCTION PURPOSE: Returns the PCIe Internal Address Range for the 
 *                   Memory Space. This range is used for accessing memory.
 *********************************************************************/
pcieRet_e Pciev0_getMemSpaceRange 
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
        Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
        if (cfg) {
          *base = cfg->dataBase;
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
} /* Pciev0_getMemSpaceRange */

/*********************************************************************
 * FUNCTION PURPOSE: Reads any register
 ********************************************************************/
pcieRet_e Pciev0_readRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *readRegs  /**< [in/out] List of registers to read */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  /* Base Address for the Application Registers */
  CSL_Pciess_appRegs *baseAppRegs;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseCfgRcRegs = NULL;
  CSL_Pcie_cfg_space_endpointRegs     *baseCfgEpRegs = NULL;  
  
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
      baseAppRegs = (CSL_Pciess_appRegs*)cfg->cfgBase;

      /* Get base address for Local or Remote config space */
      if (location == pcie_LOCATION_LOCAL) 
      {
        pcie_get_loc_cfg_base(baseAppRegs, baseCfgEpRegs, baseCfgRcRegs)
      }
      else
      {
        pcie_get_rem_cfg_base(baseAppRegs, baseCfgEpRegs, baseCfgRcRegs) 
      }

      /*****************************************************************************************
      *Application Registers
      *****************************************************************************************/
      if ((retVal == pcie_RET_OK) && (readRegs->pid != NULL)) {
        retVal = pciev0_read_pid_reg (baseAppRegs, readRegs->pid);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->cmdStatus != NULL)) {
        retVal = pciev0_read_cmdStatus_reg (baseAppRegs, readRegs->cmdStatus);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->cfgTrans != NULL)) {
        retVal = pciev0_read_cfgTrans_reg (baseAppRegs, readRegs->cfgTrans);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->ioBase != NULL)) {
        retVal = pciev0_read_ioBase_reg (baseAppRegs, readRegs->ioBase);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tlpCfg != NULL)) {
        retVal = pciev0_read_tlpCfg_reg (baseAppRegs, readRegs->tlpCfg);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->rstCmd != NULL)) {
        retVal = pciev0_read_rstCmd_reg (baseAppRegs, readRegs->rstCmd);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->ptmCfg != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->pmCmd != NULL)) {
        retVal = pciev0_read_pmCmd_reg (baseAppRegs, readRegs->pmCmd);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->pmCfg != NULL)) {
        retVal = pciev0_read_pmCfg_reg (baseAppRegs, readRegs->pmCfg);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->actStatus != NULL)) {
        retVal = pciev0_read_actStatus_reg (baseAppRegs, readRegs->actStatus);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->obSize != NULL)) {
        retVal = pciev0_read_obSize_reg (baseAppRegs, readRegs->obSize);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->diagCtrl != NULL)) {
        retVal = pciev0_read_diagCtrl_reg (baseAppRegs, readRegs->diagCtrl);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->endian != NULL)) {
        retVal = pciev0_read_endian_reg (baseAppRegs, readRegs->endian);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->priority != NULL)) {
        retVal = pciev0_read_priority_reg (baseAppRegs, readRegs->priority);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->irqEOI != NULL)) {
        retVal = pciev0_read_irqEOI_reg (baseAppRegs, readRegs->irqEOI);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->msiIrq != NULL)) {
        retVal = pciev0_read_msiIrq_reg (baseAppRegs, readRegs->msiIrq);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->epIrqSet != NULL)) {
        retVal = pciev0_read_epIrqSet_reg (baseAppRegs, readRegs->epIrqSet);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->epIrqClr != NULL)) {
        retVal = pciev0_read_epIrqClr_reg (baseAppRegs, readRegs->epIrqClr);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->epIrqStatus != NULL)) {
        retVal = pciev0_read_epIrqStatus_reg (baseAppRegs, readRegs->epIrqStatus);
      }
      for (i = 0; i < 4; i++) {
        if ((retVal == pcie_RET_OK) && (readRegs->genPurpose[i] != NULL)) {
          retVal = pciev0_read_genPurpose_reg (baseAppRegs, readRegs->genPurpose[i], i);
        }
      }
      for (i = 0; i < 8; i++) {
        if ((retVal == pcie_RET_OK) && (readRegs->msiIrqStatusRaw[i] != NULL)) {
          retVal = pciev0_read_msiIrqStatusRaw_reg (baseAppRegs, readRegs->msiIrqStatusRaw[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->msiIrqStatus[i] != NULL)) {
          retVal = pciev0_read_msiIrqStatus_reg (baseAppRegs, readRegs->msiIrqStatus[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->msiIrqEnableSet[i] != NULL)) {
          retVal = pciev0_read_msiIrqEnableSet_reg (baseAppRegs, readRegs->msiIrqEnableSet[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->msiIrqEnableClr[i] != NULL)) {
          retVal = pciev0_read_msiIrqEnableClr_reg (baseAppRegs, readRegs->msiIrqEnableClr[i], i);
        }
      }
      for (i = 0; i < 4; i++) {
        if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqStatusRaw[i] != NULL)) {
          retVal = pciev0_read_legacyIrqStatusRaw_reg (baseAppRegs, readRegs->legacyIrqStatusRaw[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqStatus[i] != NULL)) {
          retVal = pciev0_read_legacyIrqStatus_reg (baseAppRegs, readRegs->legacyIrqStatus[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqEnableSet[i] != NULL)) {
          retVal = pciev0_read_legacyIrqEnableSet_reg (baseAppRegs, readRegs->legacyIrqEnableSet[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->legacyIrqEnableClr[i] != NULL)) {
          retVal = pciev0_read_legacyIrqEnableClr_reg (baseAppRegs, readRegs->legacyIrqEnableClr[i], i);
        }
      }
      if ((retVal == pcie_RET_OK) && (readRegs->errIrqStatusRaw != NULL)) {
        retVal = pciev0_read_errIrqStatusRaw_reg (baseAppRegs, readRegs->errIrqStatusRaw);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->errIrqStatus != NULL)) {
        retVal = pciev0_read_errIrqStatus_reg (baseAppRegs, readRegs->errIrqStatus);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->errIrqEnableSet != NULL)) {
        retVal = pciev0_read_errIrqEnableSet_reg (baseAppRegs, readRegs->errIrqEnableSet);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->errIrqEnableClr != NULL)) {
        retVal = pciev0_read_errIrqEnableClr_reg (baseAppRegs, readRegs->errIrqEnableClr);
      }

      if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqStatusRaw != NULL)) {
        retVal = pciev0_read_pmRstIrqStatusRaw_reg (baseAppRegs, readRegs->pmRstIrqStatusRaw);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqStatus != NULL)) {
        retVal = pciev0_read_pmRstIrqStatus_reg (baseAppRegs, readRegs->pmRstIrqStatus);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqEnableSet != NULL)) {
        retVal = pciev0_read_pmRstIrqEnableSet_reg (baseAppRegs, readRegs->pmRstIrqEnableSet);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->pmRstIrqEnableClr != NULL)) {
        retVal = pciev0_read_pmRstIrqEnableClr_reg (baseAppRegs, readRegs->pmRstIrqEnableClr);
      }

      if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqStatusRaw != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqStatus != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqEnableSet != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->ptmIrqEnableClr != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }

      for (i = 0; i < 8; i ++) {
        if ((retVal == pcie_RET_OK) && (readRegs->obOffsetLo[i] != NULL)) {
          retVal = pciev0_read_obOffsetLo_reg (baseAppRegs, readRegs->obOffsetLo[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->obOffsetHi[i] != NULL)) {
          retVal = pciev0_read_obOffsetHi_reg (baseAppRegs, readRegs->obOffsetHi[i], i);
        }
      }

      for (i = 0; i < 4; i ++) {
        if ((retVal == pcie_RET_OK) && (readRegs->ibBar[i] != NULL)) {
          retVal = pciev0_read_ibBar_reg (baseAppRegs,  readRegs->ibBar[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->ibStartLo[i] != NULL)) {
          retVal = pciev0_read_ibStartLo_reg (baseAppRegs,  readRegs->ibStartLo[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->ibStartHi[i] != NULL)) {
          retVal = pciev0_read_ibStartHi_reg (baseAppRegs,  readRegs->ibStartHi[i], i);
        }
        if ((retVal == pcie_RET_OK) && (readRegs->ibOffset[i] != NULL)) {
          retVal = pciev0_read_ibOffset_reg (baseAppRegs,  readRegs->ibOffset[i], i);
        }
      }

      if ((retVal == pcie_RET_OK) && (readRegs->pcsCfg0 != NULL)) {
        retVal = pciev0_read_pcsCfg0_reg (baseAppRegs, readRegs->pcsCfg0);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->pcsCfg1 != NULL)) {
        retVal = pciev0_read_pcsCfg1_reg (baseAppRegs, readRegs->pcsCfg1);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->pcsStatus != NULL)) {
        retVal = pciev0_read_pcsStatus_reg (baseAppRegs, readRegs->pcsStatus);
      }

      if ((retVal == pcie_RET_OK) && (readRegs->serdesCfg0 != NULL)) {
        retVal = pciev0_read_serdesCfg0_reg (baseAppRegs, readRegs->serdesCfg0);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->serdesCfg1 != NULL)) {
        retVal = pciev0_read_serdesCfg1_reg (baseAppRegs, readRegs->serdesCfg1);
      }

      /*****************************************************************************************
      *Configuration Registers
      *****************************************************************************************/

      /*Type 0, Type1 Common Registers*/

      if ((retVal == pcie_RET_OK) && (readRegs->vndDevId != NULL)) {
        retVal = pciev0_read_vndDevId_reg (baseCfgEpRegs, readRegs->vndDevId);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->statusCmd != NULL)) {
        retVal = pciev0_read_statusCmd_reg (baseCfgEpRegs, readRegs->statusCmd);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->revId != NULL)) {
        retVal = pciev0_read_revId_reg (baseCfgEpRegs, readRegs->revId);
      }

      /*Type 0 Registers*/
      if ((retVal == pcie_RET_OK) && (readRegs->bist != NULL)) {
        retVal = pciev0_read_bist_reg (baseCfgEpRegs, readRegs->bist);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type0BarIdx != NULL)) {
        retVal = pciev0_read_type0Bar_reg (baseCfgEpRegs, &(readRegs->type0BarIdx->reg), 
                                                                           readRegs->type0BarIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type0Bar32bitIdx != NULL)) {
        retVal = pciev0_read_type0Bar32bit_reg (baseCfgEpRegs, &(readRegs->type0Bar32bitIdx->reg),
                                                                                readRegs->type0Bar32bitIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type0BarMask32bitIdx != NULL)) {
        retVal = pciev0_read_type0Bar32bit_reg (baseCfgEpRegs, &(readRegs->type0BarMask32bitIdx->reg),
                                                                                readRegs->type0BarMask32bitIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->subId != NULL)) {
        retVal = pciev0_read_subId_reg (baseCfgEpRegs, readRegs->subId);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->cardbusCisPointer != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->expRom != NULL)) {
        retVal = pciev0_read_expRom_reg (baseCfgEpRegs, readRegs->expRom);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->capPtr != NULL)) {
        retVal = pciev0_read_capPtr_reg (baseCfgEpRegs, readRegs->capPtr);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->intPin != NULL)) {
        retVal = pciev0_read_intPin_reg (baseCfgEpRegs, readRegs->intPin);
      }

      /*Type 1 Registers*/
      if ((retVal == pcie_RET_OK) && (readRegs->type1BistHeader != NULL)) {
        retVal = pciev0_read_type1BistHeader_reg (baseCfgRcRegs, readRegs->type1BistHeader);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1BarIdx != NULL)) {
        retVal = pciev0_read_type1Bar_reg (baseCfgRcRegs, &(readRegs->type1BarIdx->reg), 
                                                                           readRegs->type1BarIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1Bar32bitIdx != NULL)) {
        retVal = pciev0_read_type1Bar32bit_reg (baseCfgRcRegs, &(readRegs->type1Bar32bitIdx->reg),
                                                                                readRegs->type1Bar32bitIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1BarMask32bitIdx != NULL)) {
        retVal = pciev0_read_type1Bar32bit_reg (baseCfgRcRegs, &(readRegs->type1BarMask32bitIdx->reg),
                                                                                readRegs->type1BarMask32bitIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1BusNum != NULL)) {
        retVal = pciev0_read_type1BusNum_reg (baseCfgRcRegs, readRegs->type1BusNum);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1SecStat != NULL)) {
        retVal = pciev0_read_type1SecStat_reg (baseCfgRcRegs, readRegs->type1SecStat);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1Memspace != NULL)) {
        retVal = pciev0_read_type1Memspace_reg (baseCfgRcRegs, readRegs->type1Memspace);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->prefMem != NULL)) {
        retVal = pciev0_read_prefMem_reg (baseCfgRcRegs, readRegs->prefMem);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->prefBaseUpper != NULL)) {
        retVal = pciev0_read_prefBaseUpper_reg (baseCfgRcRegs, readRegs->prefBaseUpper);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->prefLimitUpper != NULL)) {
        retVal = pciev0_read_prefLimitUpper_reg (baseCfgRcRegs, readRegs->prefLimitUpper);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1IOSpace != NULL)) {
        retVal = pciev0_read_type1IOSpace_reg (baseCfgRcRegs, readRegs->type1IOSpace);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1CapPtr != NULL)) {
        retVal = pciev0_read_type1CapPtr_reg (baseCfgRcRegs, readRegs->type1CapPtr);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1ExpnsnRom != NULL)) {
        retVal = pciev0_read_type1ExpnsnRom_reg (baseCfgRcRegs, readRegs->type1ExpnsnRom);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->type1BridgeInt != NULL)) {
        retVal = pciev0_read_type1BridgeInt_reg (baseCfgRcRegs, readRegs->type1BridgeInt);
      }

      /* Power Management Capabilities Registers */
      if ((retVal == pcie_RET_OK) && (readRegs->pmCap != NULL)) {
        retVal = pciev0_read_pmCap_reg (baseCfgEpRegs, readRegs->pmCap);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->pmCapCtlStat != NULL)) {
        retVal = pciev0_read_pmCapCtlStat_reg (baseCfgEpRegs, readRegs->pmCapCtlStat);
      }

      /*MSI Registers*/
      if ((retVal == pcie_RET_OK) && (readRegs->msiCap != NULL)) {
        retVal = pciev0_read_msiCap_reg (baseCfgEpRegs, readRegs->msiCap);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->msiLo32 != NULL)) {
        retVal = pciev0_read_msiLo32_reg (baseCfgEpRegs, readRegs->msiLo32);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->msiUp32 != NULL)) {
        retVal = pciev0_read_msiUp32_reg (baseCfgEpRegs, readRegs->msiUp32);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->msiData != NULL)) {
        retVal = pciev0_read_msiData_reg (baseCfgEpRegs, readRegs->msiData);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->msiCapOff10H != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->msiCapOff14H != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }

      /*Capabilities Registers*/
      if ((retVal == pcie_RET_OK) && (readRegs->pciesCap != NULL)) {
        retVal = pciev0_read_pciesCap_reg (baseCfgEpRegs, readRegs->pciesCap);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->deviceCap != NULL)) {
        retVal = pciev0_read_deviceCap_reg (baseCfgEpRegs, readRegs->deviceCap);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->devStatCtrl != NULL)) {
        retVal = pciev0_read_devStatCtrl_reg (baseCfgEpRegs, readRegs->devStatCtrl);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->linkCap != NULL)) {
        retVal = pciev0_read_linkCap_reg (baseCfgEpRegs, readRegs->linkCap);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->linkStatCtrl != NULL)) {
        retVal = pciev0_read_linkStatCtrl_reg (baseCfgEpRegs, readRegs->linkStatCtrl);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->slotCap != NULL)) {
        retVal = pciev0_read_slotCap_reg (baseCfgRcRegs, readRegs->slotCap);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->slotStatCtrl != NULL)) {
        retVal = pciev0_read_slotStatCtrl_reg (baseCfgRcRegs, readRegs->slotStatCtrl);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->rootCtrlCap != NULL)) {
        retVal = pciev0_read_rootCtrlCap_reg (baseCfgRcRegs, readRegs->rootCtrlCap);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->rootStatus != NULL)) {
        retVal = pciev0_read_rootStatus_reg (baseCfgRcRegs, readRegs->rootStatus);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->devCap2 != NULL)) {
        retVal = pciev0_read_devCap2_reg (baseCfgEpRegs, readRegs->devCap2);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->devStatCtrl2 != NULL)) {
        retVal = pciev0_read_devStatCtrl2_reg (baseCfgEpRegs, readRegs->devStatCtrl2);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->linkCap2 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->linkCtrl2 != NULL)) {
        retVal = pciev0_read_linkCtrl2_reg (baseCfgEpRegs, readRegs->linkCtrl2);
      }


      /*Capabilities Extended Registers*/
      if ((retVal == pcie_RET_OK) && (readRegs->extCap != NULL)) {
        retVal = pciev0_read_extCap_reg (baseCfgEpRegs, readRegs->extCap);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->uncErr != NULL)) {
        retVal = pciev0_read_uncErr_reg (baseCfgEpRegs, readRegs->uncErr);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->uncErrMask != NULL)) {
        retVal = pciev0_read_uncErrMask_reg (baseCfgEpRegs, readRegs->uncErrMask);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->uncErrSvrty != NULL)) {
        retVal = pciev0_read_uncErrSvrty_reg (baseCfgEpRegs, readRegs->uncErrSvrty);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->corErr != NULL)) {
        retVal = pciev0_read_corErr_reg (baseCfgEpRegs, readRegs->corErr);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->corErrMask != NULL)) {
        retVal = pciev0_read_corErrMask_reg (baseCfgEpRegs, readRegs->corErrMask);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->accr != NULL)) {
        retVal = pciev0_read_accr_reg (baseCfgEpRegs, readRegs->accr);
      }
      for (i = 0; i < 4; i ++) {
        if ((retVal == pcie_RET_OK) && (readRegs->hdrLog[i] != NULL)) {
          retVal = pciev0_read_hdrLog_reg (baseCfgEpRegs, readRegs->hdrLog[i], i);
        }
      }
      if ((retVal == pcie_RET_OK) && (readRegs->rootErrCmd != NULL)) {
        retVal = pciev0_read_rootErrCmd_reg (baseCfgEpRegs, readRegs->rootErrCmd);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->rootErrSt != NULL)) {
        retVal = pciev0_read_rootErrSt_reg (baseCfgEpRegs, readRegs->rootErrSt);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->errSrcID != NULL)) {
        retVal = pciev0_read_errSrcID_reg (baseCfgEpRegs, readRegs->errSrcID);
      }

      /*Port Logic Registers*/
      if ((retVal == pcie_RET_OK) && (readRegs->plAckTimer != NULL)) {
        retVal = pciev0_read_plAckTimer_reg (baseCfgEpRegs, readRegs->plAckTimer);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plOMsg != NULL)) {
        retVal = pciev0_read_plOMsg_reg (baseCfgEpRegs, readRegs->plOMsg);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plForceLink != NULL)) {
        retVal = pciev0_read_plForceLink_reg (baseCfgEpRegs, readRegs->plForceLink);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->ackFreq != NULL)) {
        retVal = pciev0_read_ackFreq_reg (baseCfgEpRegs, readRegs->ackFreq);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->lnkCtrl != NULL)) {
        retVal = pciev0_read_lnkCtrl_reg (baseCfgEpRegs, readRegs->lnkCtrl);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->laneSkew != NULL)) {
        retVal = pciev0_read_laneSkew_reg (baseCfgEpRegs, readRegs->laneSkew);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->symNum != NULL)) {
        retVal = pciev0_read_symNum_reg (baseCfgEpRegs, readRegs->symNum);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->symTimerFltMask != NULL)) {
        retVal = pciev0_read_symTimerFltMask_reg (baseCfgEpRegs, readRegs->symTimerFltMask);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->fltMask2 != NULL)) {
        retVal = pciev0_read_fltMask2_reg (baseCfgEpRegs, readRegs->fltMask2);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->debug0 != NULL)) {
        retVal = pciev0_read_debug0_reg (baseCfgEpRegs, readRegs->debug0);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->debug1 != NULL)) {
        retVal = pciev0_read_debug1_reg (baseCfgEpRegs, readRegs->debug1);
      }
      if ((retVal == pcie_RET_OK) && (readRegs->gen2 != NULL)) {
        retVal = pciev0_read_gen2_reg (baseCfgEpRegs, readRegs->gen2);
      }

      /* Reject hw rev 1 PLCONF registers */
      if ((retVal == pcie_RET_OK) && (readRegs->plconfObnpSubreqCtrl != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfTrPStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfTrNpStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfTrCStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfQStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfVcTrAR1 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfVcTrAR2 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0PrQC != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0NprQC != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfVc0CrQC != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      for (i = 0; i < 3; i++)
      {
        if ((retVal == pcie_RET_OK) && (readRegs->plconfVcPrQC[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
        if ((retVal == pcie_RET_OK) && (readRegs->plconfVcNprQC[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
        if ((retVal == pcie_RET_OK) && (readRegs->plconfVcCrQC[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfPhyStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfPhyCtrlR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlAddress != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlUpperAddress != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      for (i = 0; i < 8; i++) {
        if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntEnable[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
        if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntMask[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
        if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlIntStatus[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfMsiCtrlGpio != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfPipeLoopback != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfDbiRoWrEn != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfAxiSlvErrResp != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfAxiSlvTimeout != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuIndex != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl1 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl2 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLowerBase != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegUpperBase != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLimit != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegLowerTarget != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegUpperTarget != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->plconfIatuRegCtrl3 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }


      /* reject hw rev 1 TI CONF registers */
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfRevision != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfSysConfig != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEoi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusRawMain != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusMain != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableSetMain != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableClrMain != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusRawMsi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqStatusMsi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableSetMsi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIrqEnableClrMsi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfDeviceType != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfDeviceCmd != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfPmCtrl != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfPhyCs != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIntxAssert != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfIntxDeassert != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfMsiXmt != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfDebugCfg != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfDebugData != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (readRegs->tiConfDiagCtrl != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
    }
  }

  return retVal;
} /* Pciev0_readRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Writes any register
 ********************************************************************/
pcieRet_e Pciev0_writeRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *writeRegs /**< [in] List of registers to write */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  /* Base Address for the Application Registers */
  CSL_Pciess_appRegs *baseAppRegs;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseCfgRcRegs = NULL;
  CSL_Pcie_cfg_space_endpointRegs     *baseCfgEpRegs = NULL;  
  
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
      baseAppRegs = (CSL_Pciess_appRegs*)cfg->cfgBase;

      /* Get base address for Local or Remote config space */
      if (location == pcie_LOCATION_LOCAL) 
      {
        pcie_get_loc_cfg_base(baseAppRegs, baseCfgEpRegs, baseCfgRcRegs)
      }
      else
      {
        pcie_get_rem_cfg_base(baseAppRegs, baseCfgEpRegs, baseCfgRcRegs) 
      }
      /*****************************************************************************************
      *Application Registers
      *****************************************************************************************/
      if ((retVal == pcie_RET_OK) && (writeRegs->cmdStatus != NULL)) {
        retVal = pciev0_write_cmdStatus_reg (baseAppRegs, writeRegs->cmdStatus);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->cfgTrans != NULL)) {
        retVal = pciev0_write_cfgTrans_reg (baseAppRegs, writeRegs->cfgTrans);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->ioBase != NULL)) {
        retVal = pciev0_write_ioBase_reg (baseAppRegs, writeRegs->ioBase);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tlpCfg != NULL)) {
        retVal = pciev0_write_tlpCfg_reg (baseAppRegs, writeRegs->tlpCfg);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->rstCmd != NULL)) {
        retVal = pciev0_write_rstCmd_reg (baseAppRegs, writeRegs->rstCmd);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->ptmCfg != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->pmCmd != NULL)) {
        retVal = pciev0_write_pmCmd_reg (baseAppRegs, writeRegs->pmCmd);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->pmCfg != NULL)) {
        retVal = pciev0_write_pmCfg_reg (baseAppRegs, writeRegs->pmCfg);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->obSize != NULL)) {
        retVal = pciev0_write_obSize_reg (baseAppRegs, writeRegs->obSize);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->diagCtrl != NULL)) {
        retVal = pciev0_write_diagCtrl_reg (baseAppRegs, writeRegs->diagCtrl);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->endian != NULL)) {
        retVal = pciev0_write_endian_reg (baseAppRegs, writeRegs->endian);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->priority != NULL)) {
        retVal = pciev0_write_priority_reg (baseAppRegs, writeRegs->priority);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->irqEOI != NULL)) {
        retVal = pciev0_write_irqEOI_reg (baseAppRegs, writeRegs->irqEOI);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->msiIrq != NULL)) {
        retVal = pciev0_write_msiIrq_reg (baseAppRegs, writeRegs->msiIrq);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->epIrqSet != NULL)) {
        retVal = pciev0_write_epIrqSet_reg (baseAppRegs, writeRegs->epIrqSet);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->epIrqClr != NULL)) {
        retVal = pciev0_write_epIrqClr_reg (baseAppRegs, writeRegs->epIrqClr);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->epIrqStatus != NULL)) {
        retVal = pciev0_write_epIrqStatus_reg (baseAppRegs, writeRegs->epIrqStatus);
      }
      for (i = 0; i < 4; i++) {
        if ((retVal == pcie_RET_OK) && (writeRegs->genPurpose[i] != NULL)) {
          retVal = pciev0_write_genPurpose_reg (baseAppRegs, writeRegs->genPurpose[i], i);
        }
      }
      for (i = 0; i < 8; i++) {
        if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqStatusRaw[i] != NULL)) {
          retVal = pciev0_write_msiIrqStatusRaw_reg (baseAppRegs, writeRegs->msiIrqStatusRaw[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqStatus[i] != NULL)) {
          retVal = pciev0_write_msiIrqStatus_reg (baseAppRegs, writeRegs->msiIrqStatus[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqEnableSet[i] != NULL)) {
          retVal = pciev0_write_msiIrqEnableSet_reg (baseAppRegs, writeRegs->msiIrqEnableSet[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->msiIrqEnableClr[i] != NULL)) {
          retVal = pciev0_write_msiIrqEnableClr_reg (baseAppRegs, writeRegs->msiIrqEnableClr[i], i);
        }
      }
      for (i = 0; i < 4; i++) {
        if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqStatusRaw[i] != NULL)) {
          retVal = pciev0_write_legacyIrqStatusRaw_reg (baseAppRegs, writeRegs->legacyIrqStatusRaw[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqStatus[i] != NULL)) {
          retVal = pciev0_write_legacyIrqStatus_reg (baseAppRegs, writeRegs->legacyIrqStatus[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqEnableSet[i] != NULL)) {
          retVal = pciev0_write_legacyIrqEnableSet_reg (baseAppRegs, writeRegs->legacyIrqEnableSet[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->legacyIrqEnableClr[i] != NULL)) {
          retVal = pciev0_write_legacyIrqEnableClr_reg (baseAppRegs, writeRegs->legacyIrqEnableClr[i], i);
        }
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->errIrqStatusRaw != NULL)) {
        retVal = pciev0_write_errIrqStatusRaw_reg (baseAppRegs, writeRegs->errIrqStatusRaw);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->errIrqStatus != NULL)) {
        retVal = pciev0_write_errIrqStatus_reg (baseAppRegs, writeRegs->errIrqStatus);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->errIrqEnableSet != NULL)) {
        retVal = pciev0_write_errIrqEnableSet_reg (baseAppRegs, writeRegs->errIrqEnableSet);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->errIrqEnableClr != NULL)) {
        retVal = pciev0_write_errIrqEnableClr_reg (baseAppRegs, writeRegs->errIrqEnableClr);
      }

      if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqStatusRaw != NULL)) {
        retVal = pciev0_write_pmRstIrqStatusRaw_reg (baseAppRegs, writeRegs->pmRstIrqStatusRaw);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqStatus != NULL)) {
        retVal = pciev0_write_pmRstIrqStatus_reg (baseAppRegs, writeRegs->pmRstIrqStatus);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqEnableSet != NULL)) {
        retVal = pciev0_write_pmRstIrqEnableSet_reg (baseAppRegs, writeRegs->pmRstIrqEnableSet);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->pmRstIrqEnableClr != NULL)) {
        retVal = pciev0_write_pmRstIrqEnableClr_reg (baseAppRegs, writeRegs->pmRstIrqEnableClr);
      }

      if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqStatusRaw != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqStatus != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqEnableSet != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->ptmIrqEnableClr != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }

      for (i = 0; i < 8; i ++) {
        if ((retVal == pcie_RET_OK) && (writeRegs->obOffsetLo[i] != NULL)) {
          retVal = pciev0_write_obOffsetLo_reg (baseAppRegs, writeRegs->obOffsetLo[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->obOffsetHi[i] != NULL)) {
          retVal = pciev0_write_obOffsetHi_reg (baseAppRegs, writeRegs->obOffsetHi[i], i);
        }
      }

      for (i = 0; i < 4; i ++) {
        if ((retVal == pcie_RET_OK) && (writeRegs->ibBar[i] != NULL)) {
          retVal = pciev0_write_ibBar_reg (baseAppRegs,  writeRegs->ibBar[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->ibStartLo[i] != NULL)) {
          retVal = pciev0_write_ibStartLo_reg (baseAppRegs,  writeRegs->ibStartLo[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->ibStartHi[i] != NULL)) {
          retVal = pciev0_write_ibStartHi_reg (baseAppRegs,  writeRegs->ibStartHi[i], i);
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->ibOffset[i] != NULL)) {
          retVal = pciev0_write_ibOffset_reg (baseAppRegs,  writeRegs->ibOffset[i], i);
        }
      }

      if ((retVal == pcie_RET_OK) && (writeRegs->pcsCfg0 != NULL)) {
        retVal = pciev0_write_pcsCfg0_reg (baseAppRegs, writeRegs->pcsCfg0);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->pcsCfg1 != NULL)) {
        retVal = pciev0_write_pcsCfg1_reg (baseAppRegs, writeRegs->pcsCfg1);
      }

      if ((retVal == pcie_RET_OK) && (writeRegs->serdesCfg0 != NULL)) {
        retVal = pciev0_write_serdesCfg0_reg (baseAppRegs, writeRegs->serdesCfg0);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->serdesCfg1 != NULL)) {
        retVal = pciev0_write_serdesCfg1_reg (baseAppRegs, writeRegs->serdesCfg1);
      }

      /*****************************************************************************************
      *Configuration Registers
      *****************************************************************************************/

      /*Type 0, Type1 Common Registers*/

      if ((retVal == pcie_RET_OK) && (writeRegs->vndDevId != NULL)) {
        retVal = pciev0_write_vndDevId_reg (baseCfgEpRegs, writeRegs->vndDevId);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->statusCmd != NULL)) {
        retVal = pciev0_write_statusCmd_reg (baseCfgEpRegs, writeRegs->statusCmd);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->revId != NULL)) {
        retVal = pciev0_write_revId_reg (baseCfgEpRegs, writeRegs->revId);
      }

      /*Type 0 Registers*/
      if ((retVal == pcie_RET_OK) && (writeRegs->bist != NULL)) {
        retVal = pciev0_write_bist_reg (baseCfgEpRegs, writeRegs->bist);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type0BarIdx != NULL)) {
        retVal = pciev0_write_type0Bar_reg (baseCfgEpRegs, &(writeRegs->type0BarIdx->reg), 
                                                                           writeRegs->type0BarIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type0Bar32bitIdx != NULL)) {
        retVal = pciev0_write_type0Bar32bit_reg (baseCfgEpRegs, &(writeRegs->type0Bar32bitIdx->reg),
                                                                                writeRegs->type0Bar32bitIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type0BarMask32bitIdx != NULL)) {
        retVal = pciev0_write_type0Bar32bit_reg (baseCfgEpRegs, &(writeRegs->type0BarMask32bitIdx->reg),
                                                                                writeRegs->type0BarMask32bitIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->subId != NULL)) {
        retVal = pciev0_write_subId_reg (baseCfgEpRegs, writeRegs->subId);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->cardbusCisPointer != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->expRom != NULL)) {
        retVal = pciev0_write_expRom_reg (baseCfgEpRegs, writeRegs->expRom);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->capPtr != NULL)) {
        retVal = pciev0_write_capPtr_reg (baseCfgEpRegs, writeRegs->capPtr);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->intPin != NULL)) {
        retVal = pciev0_write_intPin_reg (baseCfgEpRegs, writeRegs->intPin);
      }

      /*Type 1 Registers*/
      if ((retVal == pcie_RET_OK) && (writeRegs->type1BistHeader != NULL)) {
        retVal = pciev0_write_type1BistHeader_reg (baseCfgRcRegs, writeRegs->type1BistHeader);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1BarIdx != NULL)) {
        retVal = pciev0_write_type1Bar_reg (baseCfgRcRegs, &(writeRegs->type1BarIdx->reg), 
                                                                           writeRegs->type1BarIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1Bar32bitIdx != NULL)) {
        retVal = pciev0_write_type1Bar32bit_reg (baseCfgRcRegs, &(writeRegs->type1Bar32bitIdx->reg),
                                                                                writeRegs->type1Bar32bitIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1BarMask32bitIdx != NULL)) {
        retVal = pciev0_write_type1Bar32bit_reg (baseCfgRcRegs, &(writeRegs->type1BarMask32bitIdx->reg),
                                                                                writeRegs->type1BarMask32bitIdx->idx);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1BusNum != NULL)) {
        retVal = pciev0_write_type1BusNum_reg (baseCfgRcRegs, writeRegs->type1BusNum);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1SecStat != NULL)) {
        retVal = pciev0_write_type1SecStat_reg (baseCfgRcRegs, writeRegs->type1SecStat);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1Memspace != NULL)) {
        retVal = pciev0_write_type1Memspace_reg (baseCfgRcRegs, writeRegs->type1Memspace);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->prefMem != NULL)) {
        retVal = pciev0_write_prefMem_reg (baseCfgRcRegs, writeRegs->prefMem);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->prefBaseUpper != NULL)) {
        retVal = pciev0_write_prefBaseUpper_reg (baseCfgRcRegs, writeRegs->prefBaseUpper);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->prefLimitUpper != NULL)) {
        retVal = pciev0_write_prefLimitUpper_reg (baseCfgRcRegs, writeRegs->prefLimitUpper);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1IOSpace != NULL)) {
        retVal = pciev0_write_type1IOSpace_reg (baseCfgRcRegs, writeRegs->type1IOSpace);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1CapPtr != NULL)) {
        retVal = pciev0_write_type1CapPtr_reg (baseCfgRcRegs, writeRegs->type1CapPtr);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1ExpnsnRom != NULL)) {
        retVal = pciev0_write_type1ExpnsnRom_reg (baseCfgRcRegs, writeRegs->type1ExpnsnRom);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->type1BridgeInt != NULL)) {
        retVal = pciev0_write_type1BridgeInt_reg (baseCfgRcRegs, writeRegs->type1BridgeInt);
      }

      /* Power Management Capabilities Registers */
      if ((retVal == pcie_RET_OK) && (writeRegs->pmCap != NULL)) {
        retVal = pciev0_write_pmCap_reg (baseCfgEpRegs, writeRegs->pmCap);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->pmCapCtlStat != NULL)) {
        retVal = pciev0_write_pmCapCtlStat_reg (baseCfgEpRegs, writeRegs->pmCapCtlStat);
      }

      /*MSI Registers*/
      if ((retVal == pcie_RET_OK) && (writeRegs->msiCap != NULL)) {
        retVal = pciev0_write_msiCap_reg (baseCfgEpRegs, writeRegs->msiCap);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->msiLo32 != NULL)) {
        retVal = pciev0_write_msiLo32_reg (baseCfgEpRegs, writeRegs->msiLo32);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->msiUp32 != NULL)) {
        retVal = pciev0_write_msiUp32_reg (baseCfgEpRegs, writeRegs->msiUp32);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->msiData != NULL)) {
        retVal = pciev0_write_msiData_reg (baseCfgEpRegs, writeRegs->msiData);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->msiCapOff10H != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->msiCapOff14H != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }

      /*Capabilities Registers*/
      if ((retVal == pcie_RET_OK) && (writeRegs->pciesCap != NULL)) {
        retVal = pciev0_write_pciesCap_reg (baseCfgEpRegs, writeRegs->pciesCap);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->deviceCap != NULL)) {
        retVal = pciev0_write_deviceCap_reg (baseCfgEpRegs, writeRegs->deviceCap);
      }

      if ((retVal == pcie_RET_OK) && (writeRegs->devStatCtrl != NULL)) {
        retVal = pciev0_write_devStatCtrl_reg (baseCfgEpRegs, writeRegs->devStatCtrl);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->linkCap != NULL)) {
        retVal = pciev0_write_linkCap_reg (baseCfgEpRegs, writeRegs->linkCap);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->linkStatCtrl != NULL)) {
        retVal = pciev0_write_linkStatCtrl_reg (baseCfgEpRegs, writeRegs->linkStatCtrl);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->slotCap != NULL)) {
        retVal = pciev0_write_slotCap_reg (baseCfgRcRegs, writeRegs->slotCap);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->slotStatCtrl != NULL)) {
        retVal = pciev0_write_slotStatCtrl_reg (baseCfgRcRegs, writeRegs->slotStatCtrl);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->rootCtrlCap != NULL)) {
        retVal = pciev0_write_rootCtrlCap_reg (baseCfgRcRegs, writeRegs->rootCtrlCap);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->rootStatus != NULL)) {
        retVal = pciev0_write_rootStatus_reg (baseCfgRcRegs, writeRegs->rootStatus);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->devCap2 != NULL)) {
        retVal = pciev0_write_devCap2_reg (baseCfgEpRegs, writeRegs->devCap2);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->devStatCtrl2 != NULL)) {
        retVal = pciev0_write_devStatCtrl2_reg (baseCfgEpRegs, writeRegs->devStatCtrl2);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->linkCap2 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->linkCtrl2 != NULL)) {
        retVal = pciev0_write_linkCtrl2_reg (baseCfgEpRegs, writeRegs->linkCtrl2);
      }

      /*Capabilities Extended Registers*/
      if ((retVal == pcie_RET_OK) && (writeRegs->uncErr != NULL)) {
        retVal = pciev0_write_uncErr_reg (baseCfgEpRegs, writeRegs->uncErr);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->uncErrMask != NULL)) {
        retVal = pciev0_write_uncErrMask_reg (baseCfgEpRegs, writeRegs->uncErrMask);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->uncErrSvrty != NULL)) {
        retVal = pciev0_write_uncErrSvrty_reg (baseCfgEpRegs, writeRegs->uncErrSvrty);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->corErr != NULL)) {
        retVal = pciev0_write_corErr_reg (baseCfgEpRegs, writeRegs->corErr);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->corErrMask != NULL)) {
        retVal = pciev0_write_corErrMask_reg (baseCfgEpRegs, writeRegs->corErrMask);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->accr != NULL)) {
        retVal = pciev0_write_accr_reg (baseCfgEpRegs, writeRegs->accr);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->rootErrCmd != NULL)) {
        retVal = pciev0_write_rootErrCmd_reg (baseCfgEpRegs, writeRegs->rootErrCmd);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->rootErrSt != NULL)) {
        retVal = pciev0_write_rootErrSt_reg (baseCfgEpRegs, writeRegs->rootErrSt);
      }

      /*Port Logic Registers*/
      if ((retVal == pcie_RET_OK) && (writeRegs->plAckTimer != NULL)) {
        retVal = pciev0_write_plAckTimer_reg (baseCfgEpRegs, writeRegs->plAckTimer);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plOMsg != NULL)) {
        retVal = pciev0_write_plOMsg_reg (baseCfgEpRegs, writeRegs->plOMsg);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plForceLink != NULL)) {
        retVal = pciev0_write_plForceLink_reg (baseCfgEpRegs, writeRegs->plForceLink);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->ackFreq != NULL)) {
        retVal = pciev0_write_ackFreq_reg (baseCfgEpRegs, writeRegs->ackFreq);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->lnkCtrl != NULL)) {
        retVal = pciev0_write_lnkCtrl_reg (baseCfgEpRegs, writeRegs->lnkCtrl);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->laneSkew != NULL)) {
        retVal = pciev0_write_laneSkew_reg (baseCfgEpRegs, writeRegs->laneSkew);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->symNum != NULL)) {
        retVal = pciev0_write_symNum_reg (baseCfgEpRegs, writeRegs->symNum);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->symTimerFltMask != NULL)) {
        retVal = pciev0_write_symTimerFltMask_reg (baseCfgEpRegs, writeRegs->symTimerFltMask);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->fltMask2 != NULL)) {
        retVal = pciev0_write_fltMask2_reg (baseCfgEpRegs, writeRegs->fltMask2);
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->gen2 != NULL)) {
        retVal = pciev0_write_gen2_reg (baseCfgEpRegs, writeRegs->gen2);
      }

      /* Reject hw rev 1 PL CONF registers */
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfObnpSubreqCtrl != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfTrPStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfTrNpStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfTrCStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfQStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcTrAR1 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcTrAR2 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfVc0PrQC != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfVc0NprQC != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfVc0CrQC != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      for (i = 0; i < 3; i++)
      {
        if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcPrQC[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcNprQC[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->plconfVcCrQC[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfPhyStsR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfPhyCtrlR != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlAddress != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlUpperAddress != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      for (i = 0; i < 8; i++) {
        if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntEnable[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntMask[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
        if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlIntStatus[i] != NULL)) {
          /* Not supported on rev 0 */
          retVal = pcie_RET_INV_REG;
        }
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfMsiCtrlGpio != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfPipeLoopback != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfDbiRoWrEn != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfAxiSlvErrResp != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfAxiSlvTimeout != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuIndex != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl1 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl2 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLowerBase != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegUpperBase != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLimit != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegLowerTarget != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegUpperTarget != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->plconfIatuRegCtrl3 != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }


      /* Reject hw rev 1 TI CONF registers */
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfRevision != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfSysConfig != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEoi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusRawMain != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusMain != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableSetMain != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableClrMain != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusRawMsi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqStatusMsi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableSetMsi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIrqEnableClrMsi != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDeviceType != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDeviceCmd != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfPmCtrl != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfPhyCs != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIntxAssert != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfIntxDeassert != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfMsiXmt != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDebugCfg != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDebugData != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
      if ((retVal == pcie_RET_OK) && (writeRegs->tiConfDiagCtrl != NULL)) {
        /* Not supported on rev 0 */
        retVal = pcie_RET_INV_REG;
      }
    }
  }

  return retVal;
} /* Pciev0_writeRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Configures the Outbound Offset registers 
 *                   for outbound address translation
 ********************************************************************/
pcieRet_e Pciev0_cfgObOffset 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  uint32_t         obAddrLo, /**< [in] Low  Outbound address offset (32bits) */
  uint32_t         obAddrHi, /**< [in] High Outbound address offset (32bits) */
  uint8_t          region    /**< [in] Identifies the Outbound region (0-31) */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  CSL_Pciess_appRegs *baseAppRegs;
  pcieRet_e           retVal      = pcie_RET_OK;
  pcieObOffsetLoReg_t obOffsetLo;
  pcieObOffsetHiReg_t obOffsetHi;
  uint16_t            obAddrLoField;

  if (pcieLObjIsValid == 0) {
    retVal = pcie_RET_NO_INIT;
  }
  else {
    if (pcie_check_handle_fcn(handle) == 0) {
      retVal = pcie_RET_INV_HANDLE;
    }
    else {
      baseAppRegs = (CSL_Pciess_appRegs*)cfg->cfgBase;
      
      memset (&obOffsetLo, 0, sizeof(obOffsetLo));
      memset (&obOffsetHi, 0, sizeof(obOffsetHi));

      uint32_t temp_var = 0U;
      pcie_getbits(obAddrLo, CSL_PCIESS_APP_OB_OFFSET_INDEX_OB_OFFSET_LO, temp_var);
      obAddrLoField = (uint16_t)temp_var;
      
      obOffsetLo.enable = 1;
      obOffsetLo.offsetLo = obAddrLoField;

      obOffsetHi.offsetHi = obAddrHi;

      retVal = pciev0_write_obOffsetLo_reg(baseAppRegs, &obOffsetLo, (int_fast32_t)region);
      retVal = pciev0_write_obOffsetHi_reg(baseAppRegs, &obOffsetHi, (int_fast32_t)region);
    }
  }
  return retVal;
} /* Pciev0_cfgObOffset */


/*********************************************************************
 * FUNCTION PURPOSE: Configures the Inbound Translation registers 
 ********************************************************************/
pcieRet_e Pciev0_cfgIbTrans 
(
  Pcie_Handle             handle,   /**< [in] The PCIE LLD instance identifier */
  const pcieIbTransCfg_t *ibCfg     /**< [in] Inbound Translation Configuration parameters */
)
{

  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  CSL_Pciess_appRegs *baseAppRegs;
  pcieRet_e           retVal      = pcie_RET_OK;

  pcieIbBarReg_t      ibBar;
  pcieIbStartLoReg_t  ibStartLo;
  pcieIbStartHiReg_t  ibStartHi;
  pcieIbOffsetReg_t   ibOffset;
  
  uint32_t            ibStartLoField;
  uint32_t            ibOffsetField;

  if (pcieLObjIsValid == 0) {
    retVal = pcie_RET_NO_INIT;
  }
  else {
    if (pcie_check_handle_fcn(handle) == 0) {
      retVal = pcie_RET_INV_HANDLE;
    }
    else {
      baseAppRegs = (CSL_Pciess_appRegs*)cfg->cfgBase;
      
      memset (&ibBar,     0, sizeof(ibBar));
      memset (&ibStartLo, 0, sizeof(ibStartLo));
      memset (&ibStartHi, 0, sizeof(ibStartHi));
      memset (&ibOffset,  0, sizeof(ibOffset));

      ibBar.ibBar = ibCfg->ibBar;
      
      pcie_getbits(ibCfg->ibStartAddrLo, CSL_PCIESS_APP_IB_START_LO_IB_START_LO, ibStartLoField);
      ibStartLo.ibStartLo = ibStartLoField;

      ibStartHi.ibStartHi = ibCfg->ibStartAddrHi;

      pcie_getbits(ibCfg->ibOffsetAddr, CSL_PCIESS_APP_IB_OFFSET_IB_OFFSET, ibOffsetField);
      ibOffset.ibOffset = ibOffsetField;
      

      retVal = pciev0_write_ibBar_reg    (baseAppRegs, &ibBar,     ibCfg->region);
      retVal = pciev0_write_ibStartLo_reg(baseAppRegs, &ibStartLo, ibCfg->region);
      retVal = pciev0_write_ibStartHi_reg(baseAppRegs, &ibStartHi, ibCfg->region);
      retVal = pciev0_write_ibOffset_reg (baseAppRegs, &ibOffset,  ibCfg->region);
    }
  }
  return retVal;
} /* Pciev0_cfgIbTrans */


/*********************************************************************
 * FUNCTION PURPOSE: Configures a BAR Register (32bits)
 ********************************************************************/
pcieRet_e Pciev0_cfgBar 
(
  Pcie_Handle            handle,   /**< [in] The PCIE LLD instance identifier */
  const pcieBarCfg_t    *barCfg    /**< [in] BAR configuration parameters */
)
{
  pcieType0BarIdx_t  type0BarIdx;  
  pcieType1BarIdx_t  type1BarIdx;  
  pcieRegisters_t    setRegs;
  uint32_t           barAddrField = 0;
  pcieRet_e          retVal = pcie_RET_OK;

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
        pcie_getbits(barCfg->base, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_BASE_ADDRESS, barAddrField);

        type1BarIdx.reg.base     = barAddrField;
        type1BarIdx.reg.prefetch = barCfg->prefetch;
        type1BarIdx.reg.type     = barCfg->type;
        type1BarIdx.reg.memSpace = barCfg->memSpace;
        type1BarIdx.idx          = barCfg->idx;

        setRegs.type1BarIdx = &type1BarIdx;   
      }
      else
      {
        pcie_getbits(barCfg->base, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_BASE_ADDRESS, barAddrField);

        type0BarIdx.reg.base     = barAddrField;
        type0BarIdx.reg.prefetch = barCfg->prefetch;
        type0BarIdx.reg.type     = barCfg->type;
        type0BarIdx.reg.memSpace = barCfg->memSpace;
        type0BarIdx.idx          = barCfg->idx;

        setRegs.type0BarIdx = &type0BarIdx;   
      }

      pcieRet_e temp_var = Pcie_writeRegs (handle, barCfg->location, &setRegs);
      if ((temp_var) != pcie_RET_OK)
      {
        retVal = temp_var;
      }
    }
  }
  return retVal;
} /* Pciev0_cfgBar */


/*********************************************************************
 * FUNCTION PURPOSE: Configures an ATU (address translation) region
 ********************************************************************/
/* Not supported on rev 0 hw */

/* Nothing past this point */

