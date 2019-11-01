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

#ifndef PCIEV0__H
#define PCIEV0__H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <stdint.h>
#include <stdlib.h>

/* ============================================================== */
/**
 *   @file  ti/drv/pcie/src/v0/pcie.h
 *
 *   @brief PCIe hw rev 0 sub-system API and Data Definitions
 *
 */
 
/** revision dependant config (in soc/device.c) */
typedef struct
{
  volatile uint32_t *pcieSSModeAddr; /**< address of PCIESSMODE register */
  uint32_t           pcieSSModeMask;  /**< mask for PCIESSMODE field in @ref pcieSSModeAddr */
  uint32_t           pcieSSModeShift; /**< shift for PCIESSMODE field in @ref pcieSSModeAddr */
} Pciev0_DevParams;

/** 
 * v0 does not have multiple config addresses, it has one common base, hence
 * no Pciev0_DeviceCfgBaseAddrs.
 */

/** v0 version of @ref Pcie_open */
pcieRet_e Pciev0_open 
(
  int32_t            deviceNum,  /**< [in] PCIe device number (0,1,...) */
  Pcie_Handle   *pHandle     /**< [out] Resulting instance handle */
);

/** v0 version of @ref Pcie_close */
pcieRet_e Pciev0_close 
(
  Pcie_Handle *pHandle /**< [in] The PCIE LLD instance indentifier */
);

/** v0 version of @ref Pcie_readRegs */
pcieRet_e Pciev0_readRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *readRegs  /**< [in/out] List of registers to read */
);

/** v0 version of @ref Pcie_writeRegs */
pcieRet_e Pciev0_writeRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *writeRegs /**< [in] List of registers to write */
);

/** v0 version of @ref Pcie_setInterfaceMode */
pcieRet_e Pciev0_setInterfaceMode
(
  Pcie_Handle handle, /**< [in] specified interface */
  pcieMode_e mode     /**< [in] PCIE Mode */
);

/** v0 version of @ref Pcie_getMemSpaceReserved */
pcieRet_e Pciev0_getMemSpaceReserved 
(
  Pcie_Handle  handle,     /**< [in]  The PCIE LLD instance identifier */
  uint32_t    *resSize     /**< [out] Reserved space */
);

/** v0 version of @ref Pcie_getMemSpaceRange */
pcieRet_e Pciev0_getMemSpaceRange 
(
  Pcie_Handle  handle,   /**< [in] The PCIE LLD instance identifier */
  void         **base,   /**< [out] The memory space base address */
  uint32_t      *size    /**< [out] Total size of the memory space [bytes] */
);

/** v0 version of @ref Pcie_cfgObOffset */
pcieRet_e Pciev0_cfgObOffset 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  uint32_t         obAddrLo, /**< [in] Low  Outbound address offset (32bits) */
  uint32_t         obAddrHi, /**< [in] High Outbound address offset (32bits) */
  uint8_t          region    /**< [in] Identifies the Outbound region (0-7) */
);

/** v0 version of @ref Pcie_cfgIbTrans */
pcieRet_e Pciev0_cfgIbTrans 
(
  Pcie_Handle             handle,  /**< [in] The PCIE LLD instance identifier */
  const pcieIbTransCfg_t *ibCfg    /**< [in] Inbound Address Translation Configuration parameters */
);

/** v0 version of @ref Pcie_cfgBar */
pcieRet_e Pciev0_cfgBar 
(
  Pcie_Handle            handle,   /**< [in] The PCIE LLD instance identifier */
  const pcieBarCfg_t    *barCfg    /**< [in] BAR configuration parameters */
);

#ifdef __cplusplus
}
#endif
  
#endif  /* _PCIEV0_H */

/* Nothing past this point */

