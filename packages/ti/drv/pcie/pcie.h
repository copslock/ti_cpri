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

#ifndef PCIE__H
#define PCIE__H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <stdint.h>
#include <stdlib.h>
#include "pciever.h"

/* ============================================================== */
/**
 *   @file  ti/drv/pcie/pcie.h
 *
 *   @brief PCIe sub-system API and Data Definitions
 *
 */

/**  @mainpage PCIe Low Level Driver
 *
 *   @section intro  Introduction
 *
 *
 *   This document describes the Low Level Driver (LLD) for the Peripheral Component Interconnect Express (PCIe).
 *
 *   The PCI Express module supports dual operation mode: End Point (EP or Type0) or Root Complex (RC or Type1).
 *   This driver focuses on EP mode but it also provides access to some basic RC configuration/functionalities.
 *
 *   The PCIe userguide can be found at <http://www.ti.com/lit/sprugs6a>.
 *
 *   The PCIe subsystem has two address spaces. The first (Address Space 0)
 *   is dedicated for local application registers, local configuration accesses and remote
 *   configuration accesses. The second (Address Space 1) is dedicated for data transfer.
 *
 *   The PCIe driver focuses on the registers for Address Space 0.
 *
 *   Throughout the LLD, the registers/APIs are grouped into the following:\n\n
 *   -# PCIe Application Registers/APIs\n\n
 *   -# PCIe Configuration Registers/APIs (Local and Remote)\n
 *   2.1  Type0 and Type1 Common Registers/APIs\n
 *   2.2  Type0 Registers/APIs\n
 *   2.3  Type1 Registers/APIs\n
 *   2.4  MSI Registers/APIs\n
 *   2.5  Capabylity Registers/APIs\n
 *   2.6  Extended Capability Registers/APIs\n
 *   2.7  Port Logic Registers/APIs\n
 *
 *   The normal sequence of events to enable the peripheral is listed below.\n
 *   There is a C code example in ti/drv/pcie/example/sample.\n\n
 *
 *   -# Set up the SERDES PLL, reference clock
 *   -# Set up the peripheral mode (EP/RC)
 *   -# Power up the peripheral
 *   -# Disable link training
 *   -# Configure the peripheral, including BAR masks
 *   -# Configure Inbound Address Translation
 *   -# Configure Outbound Address Translation
 *   -# Enable Link Training
 *   -# Insure Link Training completion\n
 *   PCIe link is up and ready to be used.
 *
 * In order to check that all values are within bounds, the LLD can be
 * recompiled with the -Dpcie_DEBUG compiler flag.  This will ensure that
 * all values passed to writes fit within their assigned bitfields.
 */

/* Define pcie LLD Module as a master group in Doxygen format and add all PCIE LLD API
   definitions to this group. */

/** @defgroup pcielld_module PCIE LLD Module API
 *  @{
 */
/** @} */

/** @defgroup pcielld_api_functions PCIE LLD Functions
 *  @ingroup pcielld_module
 */

/** @defgroup pcielld_api_macros PCIE LLD Macros
 *  @ingroup pcielld_module
 */

/** @defgroup pcielld_api_structures PCIE LLD API Data Structures
 *  @ingroup pcielld_module
 */

/** @defgroup pcielld_reg_structures PCIE LLD Register Definitions
 *  @ingroup pcielld_module
 */

/** @defgroup pcielld_reg_app_structures PCIE LLD Application Register Definitions
 *  @ingroup pcielld_reg_structures
 */

/** @defgroup pcielld_reg_cfg_structures PCIE LLD Configuration Register Definitions
 *  @ingroup pcielld_reg_structures
 */

/** @defgroup pcielld_reg_cfg_com_structures PCIE LLD Common (Type0/Type1) Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_type0_structures PCIE LLD Type0 (endpoint) Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_type1_structures PCIE LLD Type1 (root) Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_pwr_structures PCIE LLD Power Management Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_msi_structures PCIE LLD Message Signaled Interrupt Capabilities Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_cap_structures PCIE LLD Capabilities Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_cap_ext_structures PCIE LLD Extended Capabilities Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_pl_structures PCIE LLD Port Logic Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_api_constants PCIE LLD Constants (enum's and define's)
 *  @ingroup pcielld_module
 */

/** These are the possible values for PCIe mode */
typedef enum
{
  pcie_EP_MODE = 0,    /**< Required when setting the PCIe Mode to End Point using the @ref Pcie_setInterfaceMode function */
  pcie_LEGACY_EP_MODE, /**< Required when setting the PCIe Mode to Legacy End Point using the @ref Pcie_setInterfaceMode function */
  pcie_RC_MODE         /**< Required when setting the PCIe Mode to Root Complex using the @ref Pcie_setInterfaceMode function */
} pcieMode_e;
/* @} */


/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible values for Prefetch BAR configuration */
typedef enum
{
  pcie_BAR_NON_PREF = 0,    /**< Non Prefetchable Region*/
  pcie_BAR_PREF             /**< Prefetchable Region*/
} pcieBarPref_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible values for Type BAR configuration */
typedef enum
{
  pcie_BAR_TYPE32 = 0,    /**< 32 bits BAR */
  pcie_BAR_RSVD,          /**< Reserved */
  pcie_BAR_TYPE64         /**< 64 bits BAR */
} pcieBarType_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible values for Memory BAR configuration */
typedef enum
{
  pcie_BAR_MEM_MEM = 0,    /**< Memory BAR */
  pcie_BAR_MEM_IO          /**< IO BAR */
} pcieBarMem_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible return values from all PCIE LLD functions */
typedef enum
{
#ifdef pcie_DEBUG
  /**
   * The call succeeded, but the application could have leaked memory
   * since a non-NULL pointer was overwritten.  This only
   */
  pcie_RET_DBG_WRITE_OVERFLOW = -100L, /**< write value too big for bitfield */
#endif
  pcie_RET_OK = 0,        /**< Call succeeded */
  pcie_RET_RO_CHANGED,    /**< API called with RO bits changed */
  pcie_RET_INV_REG,       /**< readRegs/writeRegs unsupported register */
  pcie_RET_INV_HANDLE,    /**< Invalid handle */
  pcie_RET_INV_DEVICENUM, /**< @ref Pcie_open deviceNum invalid */
  pcie_RET_INV_INITCFG,   /**< Invalid Pcie_InitCfg */
  pcie_RET_INV_FXNPTR,    /**< Top level API doesn't have dev specific fxn */
  pcie_RET_NO_INIT,       /**< Forgot to call Pcie_init() ? */
  pcie_RET_UNSUPPORTED,   /**< Unsupported API */
  pcie_RET_RANGECHK       /**< Rangecheck failed */
} pcieRet_e;
/* @} */


/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible values for the Encoding of LTSSM State in
 * @ref pcieDebug0Reg_t::ltssmState (for hw rev 0) or
 * @ref pcieTiConfDeviceCmdReg_t::ltssmState (for hw rev 1)
 */
typedef enum
{
  pcie_LTSSM_DETECT_QUIET=0,      /* 0x00 */
  pcie_LTSSM_DETECT_ACT,          /* 0x01 */
  pcie_LTSSM_POLL_ACTIVE,         /* 0x02 */
  pcie_LTSSM_POLL_COMPLIANCE,     /* 0x03 */
  pcie_LTSSM_POLL_CONFIG,         /* 0x04 */
  pcie_LTSSM_PRE_DETECT_QUIET,    /* 0x05 */
  pcie_LTSSM_DETECT_WAIT,         /* 0x06 */
  pcie_LTSSM_CFG_LINKWD_START,    /* 0x07 */
  pcie_LTSSM_CFG_LINKWD_ACEPT,    /* 0x08 */
  pcie_LTSSM_CFG_LANENUM_WAIT,    /* 0x09 */
  pcie_LTSSM_CFG_LANENUM_ACEPT,   /* 0x0a */
  pcie_LTSSM_CFG_COMPLETE,        /* 0x0b */
  pcie_LTSSM_CFG_IDLE,            /* 0x0c */
  pcie_LTSSM_RCVRY_LOCK,          /* 0x0d */
  pcie_LTSSM_RCVRY_SPEED,         /* 0x0e */
  pcie_LTSSM_RCVRY_RCVRCFG,       /* 0x0f */
  pcie_LTSSM_RCVRY_IDLE,          /* 0x10 */
  pcie_LTSSM_L0,                  /* 0x11 */
  pcie_LTSSM_L0S,                 /* 0x12 */
  pcie_LTSSM_L123_SEND_EIDLE,     /* 0x13 */
  pcie_LTSSM_L1_IDLE,             /* 0x14 */
  pcie_LTSSM_L2_IDLE,             /* 0x15 */
  pcie_LTSSM_L2_WAKE,             /* 0x16 */
  pcie_LTSSM_DISABLED_ENTRY,      /* 0x17 */
  pcie_LTSSM_DISABLED_IDLE,       /* 0x18 */
  pcie_LTSSM_DISABLED,            /* 0x19 */
  pcie_LTSSM_LPBK_ENTRY,          /* 0x1a */
  pcie_LTSSM_LPBK_ACTIVE,         /* 0x1b */
  pcie_LTSSM_LPBK_EXIT,           /* 0x1c */
  pcie_LTSSM_LPBK_EXIT_TIMEOUT,   /* 0x1d */
  pcie_LTSSM_HOT_RESET_ENTRY,     /* 0x1e */
  pcie_LTSSM_HOT_RESET,           /* 0x1f */
  pcie_LTSSM_RCVRY_EQ0,           /* 0x20 - hw rev 1 only */
  pcie_LTSSM_RCVRY_EQ1,           /* 0x21 - hw rev 1 only */
  pcie_LTSSM_RCVRY_EQ2,           /* 0x22 - hw rev 1 only */
  pcie_LTSSM_RCVRY_EQ3            /* 0x23 - hw rev 1 only */
} pcieLtssmState_e;
/* @} */


/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
 /** Selects whether to query or modify the local or remote PCIe registers.\n\n
  *  Important note: PCIe registers are grouped into Application and Configuration registers.\n
  *  This definition of Local/Remote is only applicable to PCIe configuration registers.\n
  *  It is NOT applicable to PCIe application registers. For application registers, the LLD *always* accesses
  *  LOCAL PCIe application registers.
  *
  *  **IMPORTANT** : on Rev 1 hardware, only RC can see EP's registers through
  *  pcie_LOCATION_REMOTE.  In order for this to work, both Pciev1_DeviceCfgBaseAddrs.remoteOffset,
  *  and outbound region 0 in ATU need to be configured for the *same* offset.
  */
typedef enum
{
  pcie_LOCATION_LOCAL,     /**< Access the local PCIe peripheral */
  pcie_LOCATION_REMOTE     /**< Access the remote PCIe peripheral */
} pcieLocation_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible sizes for the PCIe Outbound translation regions */
typedef enum
{
  pcie_OB_SIZE_1MB = 0,        /**< Corresponds to a region size of 1MB */
  pcie_OB_SIZE_2MB,            /**< Corresponds to a region size of 2MB */
  pcie_OB_SIZE_4MB,            /**< Corresponds to a region size of 4MB */
  pcie_OB_SIZE_8MB             /**< Corresponds to a region size of 8MB */
} pcieObSize_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the Enable/Disable values used by the PCIe Driver */
typedef enum
{
  pcie_DISABLE = 0,        /**< Disable */
  pcie_ENABLE              /**< Enable  */
} pcieState_e;

/**
 * @ingroup pcielld_api_constants
 *
 * @brief  Enum to select PCIe ATU(Address translation unit) region
 *         direction(Inbound or Outbound).
 *         This enum is used while configuring inbound or outbound region.
 *
 * @{
 */
typedef enum pcieAtuRegionDir
{
    PCIE_ATU_REGION_DIR_OUTBOUND, /**< Select PCIe outbound region. */
    PCIE_ATU_REGION_DIR_INBOUND   /**< Select PCIe inbound region. */
} pcieAtuRegionDir_t;
/* @} */

/**
 * @ingroup pcielld_api_constants
 *
 * @brief  This enum is used to select PCIe TLP(Transaction layer packet) type
 *         while configuring inbound or outbound region.
 *
 *  @{
 */
typedef enum pcieTlpType
{
    PCIE_TLP_TYPE_MEM, /**< MEM type is selected while doing memory transfer */
    PCIE_TLP_TYPE_IO,  /**< IO type is selected while doing I/O transfer */
    PCIE_TLP_TYPE_CFG  /**< CFG type is selected while doing configuration
                         * access */
} pcieTlpType_t;
/* @} */

/**
 * @ingroup pcielld_api_constants
 *
 * @brief  Enum to select address or BAR match mode.
 *
 * @{
 */
typedef enum pcieAtuRegionMatchMode
{
    PCIE_ATU_REGION_MATCH_MODE_ADDR, /**< Inbound packets are filtered by address match mode */
    PCIE_ATU_REGION_MATCH_MODE_BAR   /**< Inbound packets are filtered by BAR
                                       * match mode */
} pcieAtuRegionMatchMode_t;

/* @} */


/*****************************************************************************
 **********  PCIe APPLICATION REGISTERS  *****************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PCIe Peripheral ID Register
 *
 * This Register contains the major and minor revisions
 * for the PCIe module.
 *
 * This register is only used on rev 0/2 hw, but is very similar to rev 1's
 * @ref pcieTiConfRevisionReg_t
 *
 * On rev 0 hw, this corresponds to PID
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PID
 *
 * @{
 */
typedef struct pciePidReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Scheme
   *
   * On rev 0 hw, this corresponds to SCHEME
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t scheme;
  /**
   * @brief [ro] Function code
   *
   * 0xe30 is PCIe
   *
   * On rev 0 hw, this corresponds to FUNC
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 12 bits
   */
  uint16_t func;
  /**
   * @brief [ro] Module ID of the Peripheral
   *
   * 0x6810 is PCIe
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MODID
   *
   * Field size: 16 bits
   */
  uint16_t modId;
  /**
   * @brief [ro] RTL Version
   *
   * On rev 0 hw, this corresponds to RTL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RTL
   *
   * Field size: 5 bits
   */
  uint8_t rtl;
  /**
   * @brief [ro] Major revision
   *
   * On rev 0 hw, this corresponds to MAJOR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MAJOR
   *
   * Field size: 3 bits
   */
  uint8_t revMaj;
  /**
   * @brief [ro] Customer special version
   *
   * On rev 0 hw, this corresponds to CUSTOM
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CUSTOM
   *
   * Field size: 2 bits
   */
  uint8_t cust;
  /**
   * @brief [ro] Minor revision
   *
   * On rev 0 hw, this corresponds to MINOR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MINOR
   *
   * Field size: 6 bits
   */
  uint8_t revMin;
} pciePidReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Command Status Register
 *
 * This Register is used to enable address translation, link training
 * and writing to BAR mask registers.
 *
 * On rev 0 hw, this corresponds to CMD_STATUS
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to CMD_STATUS
 *
 * @{
 */
typedef struct pcieCmdStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Set to enable manual reversal for RX lanes.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RX_LANE_FLIP_EN
   *
   * Field size: 1 bit
   */
  uint8_t rxLaneFlipEn;
  /**
   * @brief [rw] Set to enable manual reversal for TX lanes.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to TX_LANE_FLIP_EN
   *
   * Field size: 1 bit
   */
  uint8_t txLaneFlipEn;
  /**
   * @brief [rw] Set to enable writing to BAR mask registers that are overlaid on BAR registers.
   *
   * On rev 0 hw, this corresponds to DBI_CS2
   * On rev 1 hw, unsupported (but see @ref pciePlconfDbiRoWrEnReg_t::cxDbiRoWrEn)
   * On rev 2 hw, this corresponds to DBI_CS2
   *
   * Field size: 1 bit
   */
  uint8_t dbi;
  /**
   * @brief [rw] Application retry Enable
   *
   * This feature can be used if initialization can take longer than PCIe
   * stipulated time frame.
   *
   * 1 = Enable all incoming PCIe transactions to be returned with a retry response.
   *
   * On rev 0 hw, this corresponds to APP_RETRY_EN
   * On rev 1 hw, unsupported (but see @ref pcieTiConfDeviceCmdReg_t::appReqRetryEn)
   * On rev 2 hw, this corresponds to APP_RETRY_EN
   *
   * Field size: 1 bit
   */
  uint8_t appRetryEn;
  /**
   * @brief [rw] Posted Write Enable
   *
   * Default is 0 with all internal bus master writes defaulting to non-posted.
   *
   * 1 = Enable the internal bus master to use posted write commands.
   *
   * On rev 0 hw, this corresponds to POSTED_WR_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t postedWrEn;
  /**
   * @brief [rw] Inbound Translation Enable
   *
   * 1 = Enable translation of inbound memory/IO read/write requests
   * into memory read/write requests.
   *
   * On rev 0 hw, this corresponds to IB_XLT_EN
   * On rev 1 hw, unsupported (but see @ref pciePlconfIatuIndexReg_t)
   * On rev 2 hw, unsupported (but see @ref pciePlconfIatuIndexReg_t)
   *
   * Field size: 1 bit
   */
  uint8_t ibXltEn;
  /**
   * @brief [rw] Outbound Translation Enable
   *
   * 1 = Enable translation of outbound memory read/write requests into
   * memory/IO/configuration read/write requests.
   *
   * On rev 0 hw, this corresponds to OB_XLT_EN
   * On rev 1 hw, unsupported (but see @ref pciePlconfIatuIndexReg_t)
   * On rev 2 hw, unsupported (but see @ref pciePlconfIatuIndexReg_t)
   *
   * Field size: 1 bit
   */
  uint8_t obXltEn;
  /**
   * @brief [rw] Link Training Enable
   *
   * 1 = Enable LTSSM in PCI Express core and link negotiation with
   * link partner will begin.
   *
   * On rev 0 hw, this corresponds to LTSSM_EN
   * On rev 1 hw, unsupported (but see @ref pcieTiConfDeviceCmdReg_t::ltssmEn)
   * On rev 2 hw, this corresponds to LTSSM_EN
   *
   * Field size: 1 bit
   */
  uint8_t ltssmEn;
} pcieCmdStatusReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Configuration Transaction Setup Register
 *
 * On rev 0 hw, this corresponds to CFG_SETUP
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pcieCfgTransReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Configuration type for outbound configuration accesses
   *
   * 0 = Type 0 access.
   * 1 = Type 1 access.
   *
   * On rev 0 hw, this corresponds to CFG_TYPE
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t type;
  /**
   * @brief [rw] PCIe bus number for outbound configuration accesses
   *
   * On rev 0 hw, this corresponds to CFG_BUS
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 8 bits
   */
  uint8_t bus;
  /**
   * @brief [rw] PCIe device number for outbound configuration accesses
   *
   * On rev 0 hw, this corresponds to CFG_DEVICE
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 5 bit
   */
  uint8_t device;
  /**
   * @brief [rw] PCIe function number for outbound configuration accesses
   *
   * On rev 0 hw, this corresponds to CFG_FUNC
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t func;
} pcieCfgTransReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the IO TLP Base Register
 *
 * On rev 0 hw, this corresponds to IOBASE
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pcieIoBaseReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] outgoing IO TLP. RC mode only
   *
   * On rev 0 hw, this corresponds to IOBASE
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 20 bits
   *
   */
  uint32_t ioBase;
} pcieIoBaseReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the TLP configuration Register
 *
 * On rev 0 hw, this corresponds to TLPCFG
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pcieTlpCfgReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Enable relaxed ordering for all outgoing TLPs
   *
   * On rev 0 hw, this corresponds to RELAXED
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t relaxed;
  /**
   * @brief [rw] Enable No Snoop attribute on all outgoing TLPs
   *
   * On rev 0 hw, this corresponds to NO_SNOOP
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t noSnoop;
} pcieTlpCfgReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Reset Command Register
 *
 * On rev 0 hw, this corresponds to RSTCMD
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to RSTCMD
 *
 * @{
 */
typedef struct pcieRstCmdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Bridge flush status
   *
   * Used to ensure no pending transactions prior to issuing warm reset.
   * 0 = No transaction is pending.
   * 1 = There are transactions pending.
   *
   * On rev 0 hw, this corresponds to FLUSH_N
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t flush;
  /**
   * @brief [ro] Bridge flush status
   *
   * Used to ensure no pending transactions prior to issuing warm reset.
   * 0 = No transaction is pending.
   * 1 = There are transactions pending.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to FLR_PF_ACTIVE
   *
   * Field size: 1 bit
   */
  uint8_t flrPfActive;
  /**
   * @brief [w1] Write 1 to initiate a downstream hot reset sequence on downstream.
   *
   * On rev 0 hw, this corresponds to INIT_RST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INIT_RST
   *
   * Field size: 1 bit
   */
  uint8_t initRst;
} pcieRstCmdReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PTM Config register
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PTMCFG
 *
 * @{
 */
typedef struct pciePtmCfgReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Select ptm_local_clk bit input to CPTS.
   *
   * 0 will select ptm_local_clk[0], 
   * 1 will select ptm_local_clk[1] ...
   * 63 will select ptm_local_clk[63]
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PTM_CLK_SEL
   *
   * Field size: 6 bit
   */
  uint8_t ptmClkSel;
  /**
   * @brief [ro] '1' indicates PTM context is valid-EP only
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PTM_CONTEXT_VALID
   *
   * Field size: 1 bit
   */
  uint8_t ptmContextValid;
  /**
   * @brief [w1] Write '1' to enable PTM transaction.EP only.
   *
   * EP will initiate one PTM transaction when this field is updated.
   * Always reads '0'
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PTM_MANUAL_UPDATE
   *
   * Field size: 1 bit
   */
  uint8_t ptmManualUpdate;
  /**
   * @brief [rw] Write '1' to enable PTM auto-update-EP only
   *
   * EP will automatically initiate PTM transaction every 10ms
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PTM_AUTO_UPDATE
   *
   * Field size: 1 bit
   */
  uint8_t ptmAutoUpdate;
} pciePtmCfgReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Power Management Command Register
 *
 * On rev 0 hw, this corresponds to PMCMD
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PMCMD
 *
 * @{
 */
typedef struct pciePmCmdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [w1] PM Turn off
   *
   * Write 1 to transmit a PM_TURNOFF message. Reads 0. Applicable in RC mode only.
   *
   * 0 = No effect\n
   * 1 = Transmit a PM_TURNOFF message
   *
   * On rev 0 hw, this corresponds to PM_XMT_TURNOFF
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_XMT_TURNOFF
   *
   * Field size: 1 bit
   */
  uint8_t turnOff;
  /**
   * @brief [w1] Transmit PM PME message
   *
   * Write 1 to transmit a PM_PME message. Reads 0. Applicable to EP mode only.
   *
   * 0 = No effect\n
   * 1 = Transmit a PM_PME message
   *
   * On rev 0 hw, this corresponds to PM_XMT_PME
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_XMT_PME
   *
   * Field size: 1 bit
   */
  uint8_t pme;
} pciePmCmdReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Power Management Configuration Register
 *
 * On rev 0 hw, this corresponds to PMCFG
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pciePmCfgReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] PM Turn off
   *
   * Write 1 to enable entry to L2/L3 ready state. Read to check L2/L3 entry readiness. Applicable to RC and EP.
   *
   * 0 = Disable entry to L2/L3 ready state.\n
   * 1 = Enable entry to L2/L3 ready state.
   *
   * On rev 0 hw, this corresponds to ENTR_L23
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t entrL23;
} pciePmCfgReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Activity Status Register
 *
 * On rev 0 hw, this corresponds to ACT_STATUS
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pcieActStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Outbound Buffers Not Empty
   *
   * On rev 0 hw, this corresponds to OB_NOT_EMPTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t obNotEmpty;
  /**
   * @brief [ro] Inbound Buffers Not Empty
   *
   * On rev 0 hw, this corresponds to IB_NOT_EMPTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t ibNotEmpty;
} pcieActStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Outbound Size Register
 *
 * On rev 0 hw, this corresponds to OB_SIZE
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pcieObSizeReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Set each outbound translation window size
   *
   * <TABLE>
   * <TR><TH>@ref size</TH><TH>Window Size</TH></TR>
   * <TR><TD>0</TD>        <TD>1 MB</TD></TR>
   * <TR><TD>1</TD>        <TD>2 MB</TD></TR>
   * <TR><TD>2</TD>        <TD>4 MB</TD></TR>
   * <TR><TD>3</TD>        <TD>8 MB</TD></TR>
   * <TR><TD>others</TD>   <TD>reserved</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to OB_SIZE
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t size;
} pcieObSizeReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Diagnostic Control register
 *
 * On rev 0 hw, this corresponds to DIAG_CTRL
 * On rev 1 hw, unsupported (but see @ref pcieTiConfDiagCtrlReg_t)
 * On rev 2 hw
 *
 * @{
 */
typedef struct pcieDiagCtrlReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Force ECRC error
   *
   * Write 1 to force inversion of LSB of ECRC for the next one packet.
   * It is self cleared when the ECRC error has been injected on one TLP.
   *
   * On rev 0 hw, this corresponds to INV_ECRC
   * On rev 1 hw, unsupported (but see @ref pcieTiConfDiagCtrlReg_t::invEcrc)
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t invEcrc;
  /**
   * @brief [rw] Force LCRC error
   *
   * Write 1 to force inversion of LSB of LCRC for the next one packet.
   * It is self cleared when the LCRC error has been injected on one TLP.
   *
   * On rev 0 hw, this corresponds to INV_LCRC
   * On rev 1 hw, unsupported (but see @ref pcieTiConfDiagCtrlReg_t::invLcrc)
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t invLcrc;
} pcieDiagCtrlReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Endian Register
 *
 * On rev 0 hw, this corresponds to ENDIAN
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pcieEndianReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Endian mode.
   *
   * <TABLE>
   * <TR><TH>@ref mode</TH><TH>Endian Swap Mode</TH></TR>
   * <TR><TD>0</TD>        <TD>Swap on 1 byte</TD></TR>
   * <TR><TD>1</TD>        <TD>Swap on 2 bytes</TD></TR>
   * <TR><TD>2</TD>        <TD>Swap on 4 bytes</TD></TR>
   * <TR><TD>3</TD>        <TD>Swap on 8 bytes</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to ENDIAN_MODE
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t mode;
} pcieEndianReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Transaction Priority Register
 *
 * On rev 0 hw, this corresponds to PRIORITY
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pciePriorityReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Master PRIV value on master transactions
   *
   * On rev 0 hw, this corresponds to MST_PRIV
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t mstPriv;
  /**
   * @brief [rw] Master PRIVID value on master transactions
   *
   * On rev 0 hw, this corresponds to MST_PRIVID
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 4 bits
   */
  uint8_t mstPrivID;
  /**
   * @brief [rw] Priority level for each inbound transaction on the
   * internal master port
   *
   * On rev 0 hw, this corresponds to MST_PRIORITY
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t mstPriority;
} pciePriorityReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the End of Interrupt Register
 *
 * On rev 0 hw, this corresponds to IRQ_EOI
 * On rev 1 hw, unsupported (but see @ref pcieTiConfIrqEoiReg_t)
 * On rev 2 hw, this corresponds to IRQ_EOI
 *
 * @{
 */
typedef struct pcieIrqEOIReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [wo] EOI for interrupts.
   *
   * Write to indicate end-of-interrupt for the interrupt events.
   * Write 0 to mark EOI for INTA, 1 for INTB and so on.
   *
   * On rev 0 hw, this corresponds to EOI
   * On rev 1 hw, unsupported (but see @ref pcieTiConfIrqEoiReg_t::lineNumber)
   * On rev 2 hw, this corresponds to EOI
   *
   * Field size: 4 bits
   */
  uint8_t EOI;
} pcieIrqEOIReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the MSI Interrupt IRQ Register
 *
 * On rev 0 hw, this corresponds to MSI_IRQ
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to MMR_IRQ
 *
 * @{
 */
typedef struct pcieMsiIrqReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] To generate MSI Interrupt 0, the EP should write 0x0000_0000 to this register.
   *
   * On rev 0 hw, this corresponds to MSI_IRQ
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MMR_IRQ
   *
   * Field size: 32 bits (rev 0) or 31 bits (rev 2)
   */
  uint32_t msiIrq;
} pcieMsiIrqReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Endpoint Interrupt Request Set Register
 *
 * On rev 0 hw, this corresponds to EP_IRQ_SET
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to LEGACY_IRQ_SET
 *
 * @{
 */
typedef struct pcieEpIrqSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Write 1 to generate assert interrupt message
   *
   * If MSI is disabled, legacy interrupt assert message will
   * be generated. On read, a 1 indicates currently asserted interrupt.
   *
   * On rev 0 hw, this corresponds to EP_IRQ_SET
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to LEGACY_IRQ_SET_0
   *
   * Field size: 1 bit
   */
  uint8_t epIrqSet;
} pcieEpIrqSetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Endpoint Interrupt Request Clear Register
 *
 * On rev 0 hw, this corresponds to EP_IRQ_CLR
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to LEGACY_IRQ_CLR
 *
 * @{
 */
typedef struct pcieEpIrqClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Write 1 to generate deassert interrupt message.
   *
   * If MSI is disabled, legacy interrupt deassert message will be generated.
   * On read, a 1 indicates currently asserted interrupt.
   *
   * On rev 0 hw, this corresponds to EP_IRQ_CLR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to LEGACY_IRQ_CLR_0
   *
   * Field size: 1 bit
   */
  uint8_t epIrqClr;
} pcieEpIrqClrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Endpoint Interrupt status Register
 *
 * On rev 0 hw, this corresponds to EP_IRQ_STATUS
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to LEGACY_IRQ_STATUS
 *
 * @{
 */
typedef struct pcieEpIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Indicates whether interrupt for function 0 is asserted or not
   *
   * On rev 0 hw, this corresponds to EP_IRQ_STATUS
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to LEGACY_IRQ_STATUS_0
   *
   * Field size: 1 bit
   */
  uint8_t epIrqStatus;
} pcieEpIrqStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of a General Purpose register
 *
 * On rev 0 hw, this corresponds to GPR# where # = 0..3
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to GPR# where # = 0..3
 *
 * @{
 */
typedef struct pcieGenPurposeReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Gen Purpose register value
   *
   * On rev 0 hw, this corresponds to GENERIC# where # = 0..3
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to GENERIC# where # = 0..3
   *
   * Field size: 32 bit
   */
  uint8_t genPurpose;
} pcieGenPurposeReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the MSI Raw Interrupt Status Register Register
 *
 * On rev 0 hw, this corresponds to MSI#_IRQ_STATUS_RAW where # = 0..7
 * On rev 1 hw, unsupported (but similar to @ref pciePlconfMsiCtrlIntStatusReg_t)
 * On rev 2 hw, this corresponds to MMR#_IRQ_STATUS_RAW where # = 0..7
 *
 * @{
 */
typedef struct pcieMsiIrqStatusRawReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Each bit indicates raw status of MSI vectors (24, 16, 8, 0) associated with the bit
   *
   * Typically, writes to this register are only done for debug purposes.
   *
   * On rev 0 hw, this corresponds to MSI#_IRQ_STATUS_RAW where # = 0..7
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MMR#_IRQ_STATUS_RAW where # = 0..7
   *
   * Field size: 4 bits
   */
  uint8_t msiRawStatus;
} pcieMsiIrqStatusRawReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the MSI Interrupt Enabled Status Register Register
 *
 * On rev 0 hw, this corresponds to MSI#_IRQ_STATUS where # = 0..7
 * On rev 1 hw, unsupported (but similar to @ref pciePlconfMsiCtrlIntStatusReg_t)
 * On rev 2 hw, this corresponds to MMR#_IRQ_STATUS where # = 0..7
 *
 * @{
 */
typedef struct pcieMsiIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Each bit indicates status of MSI vector (24, 16, 8, 0) associated with the bit
   *
   * On rev 0 hw, this corresponds to MSI#_IRQ_STATUS where # = 0..7
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MMR#_IRQ_STATUS where # = 0..7
   *
   * Field size: 4 bits
   */
  uint8_t msiIrqStatus;
} pcieMsiIrqStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the MSI Interrupt Enable Set Register
 *
 * On rev 0 hw, this corresponds to MSI#_IRQ_ENABLE_SET where # = 0..7
 * On rev 1 hw, unsupported (but similar to @ref pcieTiConfIrqEnableSetMsiReg_t)
 * On rev 2 hw, this corresponds to MMR#_IRQ_ENABLE_SET where # = 0..7
 *
 * @{
 */
typedef struct pcieMsiIrqEnableSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Each bit, when written to, enables the MSI interrupt (24, 16, 8, 0) associated with the bit
   *
   * On rev 0 hw, this corresponds to MSI#_IRQ_ENABLE_SET where # = 0..7
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MMR#_IRQ_ENABLE_SET where # = 0..7
   *
   * Field size: 4 bits
   */
  uint8_t msiIrqEnSet;
} pcieMsiIrqEnableSetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the MSI Interrupt Enable Clear Register
 *
 * On rev 0 hw, this corresponds to MSI#_IRQ_ENABLE_CLR where # = 0..7
 * On rev 1 hw, unsupported (but similar to @ref pcieTiConfIrqEnableClrMsiReg_t)
 * On rev 2 hw, this corresponds to MMR#_IRQ_ENABLE_CLR where # = 0..7
 *
 * @{
 */
typedef struct pcieMsiIrqEnableClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Each bit, when written to, disables the MSI interrupt (24, 16, 8, 0) associated with the bit
   *
   * On rev 0 hw, this corresponds to MSI#_IRQ_ENABLE_CLR where # = 0..7
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MMR#_IRQ_ENABLE_CLR where # = 0..7
   *
   * Field size: 4 bits
   */
  uint8_t msiIrqEnClr;
} pcieMsiIrqEnableClrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Legacy Raw Interrupt Status Register
 *
 * On rev 0 hw, this corresponds to LEGACY_#_IRQ_STATUS_RAW where # = A..D (0-3)
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to LEGACY_#_IRQ_STATUS_RAW where # = A..D (0-3)
 *
 * @{
 */
typedef struct pcieLegacyIrqStatusRawReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Legacy Interrupt Raw Status, RC mode only
   *
   * On rev 0 hw, this corresponds to INT_# where # = A..D
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INT_# where # = A..D
   *
   * Field size: 1 bit
   */
  uint8_t legacyRawStatus;
} pcieLegacyIrqStatusRawReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Legacy Interrupt Enabled Status Register
 *
 * On rev 0 hw, this corresponds to LEGACY_#_IRQ_STATUS where # = A..D (0-3)
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to LEGACY_#_IRQ_STATUS where # = A..D (0-3)
 *
 * @{
 */
typedef struct pcieLegacyIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Legacy Interrupt status
   *
   * Set when interrupt is active. Write one to clear the interrupt event.
   * RC mode only.
   *
   * On rev 0 hw, this corresponds to INT_# where # = A..D
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INT_# where # = A..D
   *
   * Field size: 1 bit
   */
  uint8_t legacyIrqStatus;
} pcieLegacyIrqStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Legacy Interrupt Enable Set Register
 *
 * On rev 0 hw, this corresponds to LEGACY_#_IRQ_ENABLE_SET where # = A..D (0-3)
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to LEGACY_#_IRQ_ENABLE_SET where # = A..D (0-3)
 *
 * @{
 */
typedef struct pcieLegacyIrqEnableSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] 0: has no effect; 1: enables the interrupt
   *
   * On rev 0 hw, this corresponds to INT_# where # = A..D
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INT_# where # = A..D
   *
   * Field size: 1 bit
   */
  uint8_t legacyIrqEnSet;
} pcieLegacyIrqEnableSetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Legacy Interrupt Enable Clear Register
 *
 * On rev 0 hw, this corresponds to LEGACY_#_IRQ_ENABLE_CLR where # = A..D (0-3)
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to LEGACY_#_IRQ_ENABLE_CLR where # = A..D (0-3)
 *
 * @{
 */
typedef struct pcieLegacyIrqEnableClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] 0 has no effect; 1 disables the interrupt
   *
   * On rev 0 hw, this corresponds to INT_# where # = A..D
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INT_# where # = A..D
   *
   * Field size: 1 bit
   */
  uint8_t legacyIrqEnClr;
} pcieLegacyIrqEnableClrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Raw ERR Interrupt Status Register
 *
 * On rev 0 hw, this corresponds to ERR_IRQ_STATUS_RAW
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to ERR_IRQ_STATUS_RAW
 *
 * @{
 */
typedef struct pcieErrIrqStatusRawReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] ECRC error raw status
   *
   * On rev 0 hw, this corresponds to ERR_AER
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_AER_RAW
   *
   * Field size: 1 bit
   */
  uint8_t errAer;
  /**
   * @brief [rw] AXI tag lookup fatal error raw status
   *
   * On rev 0 hw, this corresponds to ERR_AXI
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t errAxi;
  /**
   * @brief [rw] correctable error raw status
   *
   * On rev 0 hw, this corresponds to ERR_CORR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_CORR_RAW
   *
   * Field size: 1 bit
   */
  uint8_t errCorr;
  /**
   * @brief [rw] nonfatal error raw status
   *
   * On rev 0 hw, this corresponds to ERR_NONFATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_NONFATAL_RAW
   *
   * Field size: 1 bit
   */
  uint8_t errNonFatal;
  /**
   * @brief [rw] fatal error raw status
   *
   * On rev 0 hw, this corresponds to ERR_FATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_FATAL_RAW
   *
   * Field size: 1 bit
   */
  uint8_t errFatal;
  /**
   * @brief [rw] system error (fatal, nonfatal, correctable error) raw status
   *
   * On rev 0 hw, this corresponds to ERR_SYS
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_SYS_RAW
   *
   * Field size: 1 bit
   */
  uint8_t errSys;
} pcieErrIrqStatusRawReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the ERR Interrupt Enabled Status Register
 *
 * On rev 0 hw, this corresponds to ERR_IRQ_STATUS
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to ERR_IRQ_STATUS
 *
 * @{
 */
typedef struct pcieErrIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] ECRC error status
   *
   * On rev 0 hw, this corresponds to ERR_AER
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_AER
   *
   * Field size: 1 bit
   */
  uint8_t errAer;
  /**
   * @brief [rw] AXI tag lookup fatal error status
   *
   * On rev 0 hw, this corresponds to ERR_AXI
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t errAxi;
  /**
   * @brief [rw] correctable error status
   *
   * On rev 0 hw, this corresponds to ERR_CORR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_CORR
   *
   * Field size: 1 bit
   */
  uint8_t errCorr;
  /**
   * @brief [rw] nonfatal error status
   *
   * On rev 0 hw, this corresponds to ERR_NONFATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_NONFATAL
   *
   * Field size: 1 bit
   */
  uint8_t errNonFatal;
  /**
   * @brief [rw] fatal error status
   *
   * On rev 0 hw, this corresponds to ERR_FATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_FATAL
   *
   * Field size: 1 bit
   */
  uint8_t errFatal;
  /**
   * @brief [rw] system error (fatal, nonfatal, correctable error) status
   *
   * On rev 0 hw, this corresponds to ERR_SYS
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_SYS
   *
   * Field size: 1 bit
   */
  uint8_t errSys;
} pcieErrIrqStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the ERR Interrupt Enable Set Register
 *
 * On rev 0 hw, this corresponds to ERR_IRQ_ENABLE_SET
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to ERR_IRQ_ENABLE_SET
 *
 * @{
 */
typedef struct pcieErrIrqEnableSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] set to enable the ECRC error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_AER
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_AER
   *
   * Field size: 1 bit
   */
  uint8_t errAer;
  /**
   * @brief [rw] set to enable the AXI tag lookup fatal error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_AXI
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t errAxi;
  /**
   * @brief [rw] set to enable the correctable error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_CORR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_CORR_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t errCorr;
  /**
   * @brief [rw] set to enable the nonfatal error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_NONFATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_NONFATAL_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t errNonFatal;
  /**
   * @brief [rw] set to enable the fatal error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_FATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_FATAL_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t errFatal;
  /**
   * @brief [rw] set to enable the system error (fatal, nonfatal, correctable error) interrupt
   *
   * On rev 0 hw, this corresponds to ERR_SYS
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_SYS_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t errSys;
} pcieErrIrqEnableSetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the ERR Interrupt Enable Clear Register
 *
 * On rev 0 hw, this corresponds to ERR_IRQ_ENABLE_CLR
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to ERR_IRQ_ENABLE_CLR
 *
 * @{
 */
typedef struct pcieErrIrqEnableClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] set to disable the ECRC error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_AER
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_AER_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t errAer;
  /**
   * @brief [rw] set to disable the AXI tag lookup fatal error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_AXI
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t errAxi;
  /**
   * @brief [rw] set to disable the correctable error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_CORR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_CORR_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t errCorr;
  /**
   * @brief [rw] set to disable the nonfatal error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_NONFATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_NONFATAL_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t errNonFatal;
  /**
   * @brief [rw] set to disable the fatal error interrupt
   *
   * On rev 0 hw, this corresponds to ERR_FATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_FATAL_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t errFatal;
  /**
   * @brief [rw] set to disable the system error (fatal, nonfatal, correctable error) interrupt
   *
   * On rev 0 hw, this corresponds to ERR_SYS
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_SYS_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t errSys;
} pcieErrIrqEnableClrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Raw Power Management and Reset Interrupt Status Register
 *
 * On rev 0 hw, this corresponds to PMRST_IRQ_STATUS_RAW
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PMRST_IRQ_STATUS_RAW
 *
 * @{
 */
typedef struct pciePmRstIrqStatusRawReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Link Request Reset interrupt raw status
   *
   * On rev 0 hw, this corresponds to LNK_RST_REQ
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to LNK_RST_REQ_RAW
   *
   * Field size: 1 bit
   */
  uint8_t linkRstReq;
  /**
   * @brief [rw] Power management PME message received interrupt raw status
   *
   * On rev 0 hw, this corresponds to PM_PME
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_PME_RAW
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /**
   * @brief [rw] Power mangement ACK received interrupt raw status
   *
   * On rev 0 hw, this corresponds to PM_TO_ACK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_TO_ACK_RAW
   *
   * Field size: 1 bit
   */
  uint8_t pmToAck;
  /**
   * @brief [rw] Power management turnoff messages received raw status
   *
   * On rev 0 hw, this corresponds to PM_TURNOFF
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_TURNOFF_RAW
   *
   * Field size: 1 bit
   */
  uint8_t pmTurnoff;
} pciePmRstIrqStatusRawReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Power Management and Reset Interrupt Enabled Status Register
 *
 * On rev 0 hw, this corresponds to PMRST_IRQ_STATUS
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PMRST_IRQ_STATUS
 *
 * @{
 */
typedef struct pciePmRstIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Link Request Reset interrupt status
   *
   * On rev 0 hw, this corresponds to LNK_RST_REQ
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to LNK_RST_REQ
   *
   * Field size: 1 bit
   */
  uint8_t linkRstReq;
  /**
   * @brief [rw] Power management PME message received interrupt status
   *
   * On rev 0 hw, this corresponds to PM_PME
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_PME
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /**
   * @brief [rw] Power mangement ACK received interrupt status
   *
   * On rev 0 hw, this corresponds to PM_TO_ACK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_TO_ACK
   *
   * Field size: 1 bit
   */
  uint8_t pmToAck;
  /**
   * @brief [rw] Power management turnoff messages received status
   *
   * On rev 0 hw, this corresponds to PM_TURNOFF
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_TURNOFF
   *
   * Field size: 1 bit
   */
  uint8_t pmTurnoff;
} pciePmRstIrqStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Power Management and Reset Interrupt Enable Set Register
 *
 * On rev 0 hw, this corresponds to PMRST_ENABLE_SET
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PMRST_IRQ_ENABLE_SET
 *
 * @{
 */
typedef struct pciePmRstIrqEnableSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] set to enable the Link Request Reset interrupt
   *
   * On rev 0 hw, this corresponds to LNK_RST_REQ
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to LNK_RST_REQ_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t linkRstReq;
  /**
   * @brief [rw] set to enable the Power management PME message received interrupt
   *
   * On rev 0 hw, this corresponds to PM_PME
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_PME_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /**
   * @brief [rw] set to enable the Power mangement ACK received interrupt
   *
   * On rev 0 hw, this corresponds to PM_TO_ACK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_TO_ACK_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t pmToAck;
  /**
   * @brief [rw] set to enable the Power management turnoff messages received interrupt
   *
   * On rev 0 hw, this corresponds to PM_TURNOFF
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_TURNOFF_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t pmTurnoff;
} pciePmRstIrqEnableSetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Power Management and Reset Interrupt Enable Clear Register
 *
 * On rev 0 hw, this corresponds to PMRST_ENABLE_CLR
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PMRST_IRQ_ENABLE_CLR
 *
 * @{
 */
typedef struct pciePmRstIrqEnableClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] set to disable the Link Request Reset interrupt
   *
   * On rev 0 hw, this corresponds to LNK_RST_REQ
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to LNK_RST_REQ_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t linkRstReq;
  /**
   * @brief [rw] set to disable the Power management PME message received interrupt
   *
   * On rev 0 hw, this corresponds to PM_PME
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_PME_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /**
   * @brief [rw] set to disable the Power mangement ACK received interrupt
   *
   * On rev 0 hw, this corresponds to PM_TO_ACK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_TO_ACK_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t pmToAck;
  /**
   * @brief [rw] set to disable the Power management turnoff messages received interrupt
   *
   * On rev 0 hw, this corresponds to PM_TURNOFF
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PM_TURNOFF_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t pmTurnoff;
} pciePmRstIrqEnableClrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Precision Time Measurement Raw Status Register
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PTM_IRQ_STATUS_RAW
 *
 * @{
 */
typedef struct pciePtmIrqStatusRawReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Link Request Reset interrupt raw status
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PTM_CLK_UPDATED_RAW
   *
   * Field size: 1 bit
   */
  uint8_t ptmClkUpdated;
} pciePtmIrqStatusRawReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Precision Time Measurement Status Register
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PTM_IRQ_STATUS
 *
 * @{
 */
typedef struct pciePtmIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Link Request Reset interrupt raw status
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PTM_CLK_UPDATED
   *
   * Field size: 1 bit
   */
  uint8_t ptmClkUpdated;
} pciePtmIrqStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Precision Time Measurement Raw Status Register
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PTM_IRQ_STATUS_ENABLE_SET
 *
 * @{
 */
typedef struct pciePtmIrqEnableSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Link Request Reset interrupt raw status
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PTM_CLK_UPDATED_EN_SET
   *
   * Field size: 1 bit
   */
  uint8_t ptmClkUpdated;
} pciePtmIrqEnableSetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Precision Time Measurement Raw Status Register
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to PTM_IRQ_STATUS_ENABLE_CLR
 *
 * @{
 */
typedef struct pciePtmIrqEnableClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Link Request Reset interrupt raw status
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PTM_CLK_UPDATED_EN_CLR
   *
   * Field size: 1 bit
   */
  uint8_t ptmClkUpdated;
} pciePtmIrqEnableClrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Outbound Translation Region Offset Low and Index Register
 *
 * On rev 0 hw, this corresponds to OB_OFFSET_INDEXn where n = 0..7
 * On rev 1 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 * On rev 2 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 *
 * @{
 */
typedef struct pcieObOffsetLoReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Offset bits for the translation region
   *
   * On rev 0 hw, this corresponds to OB_OFFSETn_LO (n = 0..7)
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 12 bits
   */
  uint16_t offsetLo;
  /**
   * @brief [rw] Enable translation region
   *
   * On rev 0 hw, this corresponds to OB_ENABLEn (n = 0..7)
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t enable;
} pcieObOffsetLoReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Outbound Translation Region Offset High Register
 *
 * On rev 0 hw, this corresponds to OB_OFFSETn_HI where n = 0..7
 * On rev 1 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 * On rev 2 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 *
 * @{
 */
typedef struct pcieObOffsetHiReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Offset high bits [63:32] for translation region
   *
   * On rev 0 hw, this corresponds to OB_OFFSETn_HI (n = 0..7)
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 32 bits
   */
  uint32_t offsetHi;
} pcieObOffsetHiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Inbound Translation BAR Match Register
 *
 * On rev 0 hw, this corresponds to IB_BARn where n = 0..3
 * On rev 1 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 * On rev 2 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 *
 * @{
 */
typedef struct pcieIbBarReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] BAR number to match for inbound translation region
   *
   * On rev 0 hw, this corresponds to IB_BARn (n = 0..3)
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t ibBar;
} pcieIbBarReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Inbound Translation Start Address Low Register
 *
 * On rev 0 hw, this corresponds to IB_STARTn_LO where n = 0..3
 * On rev 1 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 * On rev 2 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 *
 * @{
 */
typedef struct pcieIbStartLoReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Start address bits [31:8] for inbound translation region
   *
   * On rev 0 hw, this corresponds to IB_STARTn_LO (n = 0..3)
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 24 bits
   */
  uint32_t ibStartLo;
} pcieIbStartLoReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Inbound Translation Start Address High Register
 *
 * On rev 0 hw, this corresponds to IB_STARTn_LO where n = 0..3
 * On rev 1 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 * On rev 2 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 * @{
 */
typedef struct pcieIbStartHiReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Start address high bits [63:32] for inbound translation region
   *
   * On rev 0 hw, this corresponds to IB_STARTn_HI (n = 0..3)
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 32 bits
   */
  uint32_t ibStartHi;
} pcieIbStartHiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Inbound Translation Address Offset Register
 *
 * On rev 0 hw, this corresponds to IB_OFFSETn where n = 0..3
 * On rev 1 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 * On rev 2 hw, unsupported (but similar to iATU
 *              starting at @ref pciePlconfIatuIndexReg_t)
 *
 * @{
 */
typedef struct pcieIbOffsetReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Offset address bits [31:8] for inbound translation region
   *
   * On rev 0 hw, this corresponds to IB_OFFSETn (n = 0..3)
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 24 bits
   */
  uint32_t ibOffset;
} pcieIbOffsetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PCS Configuration 0 Register
 *
 * On rev 0 hw, this corresponds to PCS_CFG0
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pciePcsCfg0Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Receiver lock/sync control.
   *
   * On rev 0 hw, this corresponds to PCS_SYNC
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 5 bits
   */
  uint8_t pcsSync;
  /**
   * @brief [rw] Receiver initialization holdoff control.
   *
   * On rev 0 hw, this corresponds to PCS_HOLDOFF
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 8 bits
   */
  uint8_t pcsHoldOff;
  /**
   * @brief [rw] Rate change delay.
   *
   * On rev 0 hw, this corresponds to PCS_RC_DELAY
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t pcsRCDelay;
  /**
   * @brief [rw] Detection delay.
   *
   * On rev 0 hw, this corresponds to PCS_DET_DELAY
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 4 bits
   */
  uint8_t pcsDetDelay;
  /**
   * @brief [rw] Enable short times for debug purposes.
   *
   * On rev 0 hw, this corresponds to PCS_SHRT_TM
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pcsShrtTM;
  /**
   * @brief [rw] Enable PIPE Spec 1.86 for phystatus behavior.
   *
   * On rev 0 hw, this corresponds to PCS_STAT186
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pcsStat186;
  /**
   * @brief [rw] Fed term output to 3'b100 during reset.
   *
   * On rev 0 hw, this corresponds to PCS_FIX_TERM
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pcsFixTerm;
  /**
   * @brief [rw] Fix std output to 2'b10.
   *
   * On rev 0 hw, this corresponds to PCS_FIX_STD
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pcsFixStd;
  /**
   * @brief [rw] Deassert enidl during L2 state.
   *
   * On rev 0 hw, this corresponds to PCS_L2_ENIDL_OFF
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pcsL2EnidlOff;
  /**
   * @brief [rw] Deassert Rx enable in L0s state.
   *
   * On rev 0 hw, this corresponds to PCS_L0S_RX_OFF
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pcsL2L0SRxOff;
  /**
   * @brief [rw] RX and TX on during reset and TX also on in P1 state.
   *
   * On rev 0 hw, this corresponds to PCS_RXTX_ON
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pcsRxTxOn;
  /**
   * @brief [rw] RX and TX on during reset.
   *
   * On rev 0 hw, this corresponds to PCS_RXTX_RST
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pcsRxTxRst;
} pciePcsCfg0Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PCS Configuration 1 Register
 *
 * On rev 0 hw, this corresponds to PCS_CFG1
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pciePcsCfg1Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Error bit enable.
   *
   * On rev 0 hw, this corresponds to PCS_ERR_BIT
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 10 bits
   */
  uint16_t pcsErrBit;
  /**
   * @brief [rw] Error lane enable
   *
   * On rev 0 hw, this corresponds to PCS_ERR_LN
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t pcsErrLn;
  /**
   * @brief [rw] Error injection mode
   *
   * On rev 0 hw, this corresponds to PCS_ERR_MODE
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t pcsErrMode;
} pciePcsCfg1Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PCS Status Register
 *
 * On rev 0 hw, this corresponds to PCS_STATUS
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pciePcsStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] PCS RTL Revision.
   *
   * On rev 0 hw, this corresponds to PCS_REV
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t pcsRev;
  /**
   * @brief [ro] PCS lanes enabled status.
   *
   * On rev 0 hw, this corresponds to PCS_LN_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t pcsLnEn;
  /**
   * @brief [ro] PCS transmitters enabled status.
   *
   * On rev 0 hw, this corresponds to PCS_TX_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t pcsTxEn;
  /**
   * @brief [ro] PCS receivers enabled status.
   *
   * On rev 0 hw, this corresponds to PCS_RX_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t pcsRxEn;
} pciePcsStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the SERDES config 0 Register
 *
 * On rev 0 hw, this corresponds to SERDES_CFG0
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pcieSerdesCfg0Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Enable Tx loopback. Set both bits high to enable.
   *
   * On rev 0 hw, this corresponds to TX_LOOPBACK
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t txLoopback;
  /**
   * @brief [rw] Master mode for synchronization.
   *
   * On rev 0 hw, this corresponds to TX_MSYNC
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t txMsync;
  /**
   * @brief [rw] Enable common mode adjustment.
   *
   * On rev 0 hw, this corresponds to TX_CM
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t txCm;
  /**
   * @brief [rw] Invert Tx pair polarity.
   *
   * On rev 0 hw, this corresponds to TX_INVPAIR
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t txInvpair;
  /**
   * @brief [rw] Enable Rx loopback. Set both bits to high to enable loopback.
   *
   * On rev 0 hw, this corresponds to RX_LOOPBACK
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t rxLoopback;
  /**
   * @brief [rw] Enable Rx offset compensation.
   *
   * On rev 0 hw, this corresponds to RX_ENOC
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t rxEnoc;
  /**
   * @brief [rw] Enable Rx adaptive equalization.
   *
   * On rev 0 hw, this corresponds to RX_EQ
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 4 bits
   */
  uint8_t rxEq;
  /**
   * @brief [rw] Enable Rx clock data recovery.
   *
   * On rev 0 hw, this corresponds to RX_CDR
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t rxCdr;
  /**
   * @brief [rw] Enable Rx loss of signal detection.
   *
   * On rev 0 hw, this corresponds to RX_LOS
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t rxLos;
  /**
   * @brief [rw] Enable Rx symbol alignment.
   *
   * On rev 0 hw, this corresponds to RX_ALIGN
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t rxAlign;
  /**
   * @brief [rw] Invert Rx pair polarity.
   *
   * On rev 0 hw, this corresponds to RX_INVPAIR
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t rxInvpair;
} pcieSerdesCfg0Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the SERDES config 1 Register
 *
 * On rev 0 hw, this corresponds to SERDES_CFG1
 * On rev 1 hw, unsupported
 * On rev 2 hw, unsupported
 *
 * @{
 */
typedef struct pcieSerdesCfg1Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Enable Tx loopback. Set both bits high to enable.
   *
   * On rev 0 hw, this corresponds to TX_LOOPBACK
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t txLoopback;
  /**
   * @brief [rw] Master mode for synchronization.
   *
   * On rev 0 hw, this corresponds to TX_MSYNC
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t txMsync;
  /**
   * @brief [rw] Enable common mode adjustment.
   *
   * On rev 0 hw, this corresponds to TX_CM
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t txCm;
  /**
   * @brief [rw] Invert Tx pair polarity.
   *
   * On rev 0 hw, this corresponds to TX_INVPAIR
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t txInvpair;
  /**
   * @brief [rw] Enable Rx loopback. Set both bits to high to enable loopback.
   *
   * On rev 0 hw, this corresponds to RX_LOOPBACK
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t rxLoopback;
  /**
   * @brief [rw] Enable Rx offset compensation.
   *
   * On rev 0 hw, this corresponds to RX_ENOC
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t rxEnoc;
  /**
   * @brief [rw] Enable Rx adaptive equalization.
   *
   * On rev 0 hw, this corresponds to RX_EQ
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 4 bits
   */
  uint8_t rxEq;
  /**
   * @brief [rw] Enable Rx clock data recovery.
   *
   * On rev 0 hw, this corresponds to RX_CDR
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t rxCdr;
  /**
   * @brief [rw] Enable Rx loss of signal detection.
   *
   * On rev 0 hw, this corresponds to RX_LOS
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 3 bits
   */
  uint8_t rxLos;
  /**
   * @brief [rw] Enable Rx symbol alignment.
   *
   * On rev 0 hw, this corresponds to RX_ALIGN
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t rxAlign;
  /**
   * @brief [rw] Invert Rx pair polarity.
   *
   * On rev 0 hw, this corresponds to RX_INVPAIR
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t rxInvpair;
} pcieSerdesCfg1Reg_t;
/* @} */



/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 0 and TYPE 1 REGISTERS ************
 **********        Registers that are common to both Types        ************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_com_structures
 * @brief Specification of the Vendor Device ID Register
 *
 * On rev 0 hw, this corresponds to VENDOR_DEVICE_ID
 * On rev 1 hw, this corresponds to DEVICE_VENDORID
 * On rev 2 hw, this corresponds to DEVICE_ID_VENDOR_ID_REG and TYPE1_DEV_ID_VEND_ID_REG
 *
 * @{
 */
typedef struct pcieVndDevIdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Device ID
   *
   * On rev 0 hw, this corresponds to DEVICE_ID
   * On rev 1 hw, this corresponds to DEVICEID
   * On rev 2 hw, this corresponds to PCI_TYPE0_DEVICE_ID or DEVICE_ID_VENDOR_ID_REG
   *
   * Field size: 16 bits
   */
  uint16_t devId;
  /**
   * @brief [rw] Vendor ID
   *
   * On rev 0 hw, this corresponds to VENDOR_ID
   * On rev 1 hw, this corresponds to VENDORID
   * On rev 2 hw, this corresponds to PCI_TYPE0_VENDOR_ID and VENDOR_ID
   *
   * Field size: 16 bits
   */
  uint16_t vndId;
} pcieVndDevIdReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_com_structures
 * @brief Specification of the Status Command Register
 *
 * On rev 0 hw, this corresponds to STATUS_COMMAND
 * On rev 1 hw, this corresponds to STATUS_COMMAND_REGISTER
 * On rev 2 hw, this corresponds to STATUS_COMMAND_REG and TYPE1_STATUS_COMMAND_REG
 *
 * @{
 */
typedef struct pcieStatusCmdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] parity
   *
   * Set if received a poisoned TLP
   *
   * On rev 0 hw, this corresponds to PARITY_ERROR
   * On rev 1 hw, this corresponds to DETECT_PARERR
   * On rev 2 hw, this corresponds to DETECTED_PARITY_ERR
   *
   * Field size: 1 bit
   */
  uint8_t parity;
  /**
   * @brief [rw] sys error
   *
   * Set if function sends an ERR_FATAL or ERR_NONFATAL message and
   * @ref serrEn bit is set
   *
   * On rev 0 hw, this corresponds to SIG_SYS_ERROR
   * On rev 1 hw, this corresponds to SIGNAL_SYSERR
   * On rev 2 hw, this corresponds to SIGNALED_SYS_ERR
   *
   * Field size: 1 bit
   */
  uint8_t sysError;
  /**
   * @brief [rw] mst abort
   *
   * Set when a requester receives a completion with unsupported request
   * completion status
   *
   * On rev 0 hw, this corresponds to RX_MST_ABORT
   * On rev 1 hw, this corresponds to RCVD_MASTERABORT
   * On rev 2 hw, this corresponds to RCVD_MASTER_ABORT
   *
   * Field size: 1 bit
   */
  uint8_t mstAbort;
  /**
   * @brief [rw] tgt abort
   *
   * Set when a requester receives a completion with completer abort status.
   *
   * On rev 0 hw, this corresponds to RX_TGT_ABORT
   * On rev 1 hw, this corresponds to RCVD_TRGTABORT
   * On rev 2 hw, this corresponds to RCVD_TARGET_ABORT
   *
   * Field size: 1 bit
   */
  uint8_t tgtAbort;
  /**
   * @brief [rw] sig tgt abort
   *
   * Set when a function acting as a completer terminates a request by issuing
   * completer abort completion status to the requester.
   *
   * On rev 0 hw, this corresponds to SIG_TGT_ABORT
   * On rev 1 hw, this corresponds to SIGNAL_TRGTABORT
   * On rev 2 hw, this corresponds to SIGNALED_TARGET_ABORT
   *
   * Field size: 1 bit
   */
  uint8_t sigTgtAbort;
  /**
   * @brief [ro] DevSel Timing
   *
   * Hardwired to 0 for PCIExpress.  Only defined on rev 1 hw version.
   *
   * Forced to 0 on rev 0 hw read; ignored on rev 0 hw write.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to DEVSEL_TIME
   * On rev 2 hw, this corresponds to DEV_SEL_TIMING
   *
   * Field size: 2 bits
   */
  uint8_t devSelTime;
  /**
   * @brief [rw] par error
   *
   * This bit is set by a requester if the @ref parError bit is set
   * in its Command register and either the condition that the requester
   * receives a poisoned completion or the condition that the
   * requester poisons a write request is true.
   *
   * On rev 0 hw, this corresponds to DAT_PAR_ERRROR
   * On rev 1 hw, this corresponds to MASTERDATA_PARERR
   * On rev 2 hw, this corresponds to MASTER_DPE
   *
   * Field size: 1 bit
   */
  uint8_t parError;
  /**
   * @brief [ro] Back to Back Capable
   *
   * Hardwired to 0 for PCIExpress.  Only defined on rev 1 hw version.
   *
   * Forced to 0 on rev 0 hw read; ignored on rev 0 hw write.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to FAST_B2B
   * On rev 2 hw, this corresponds to FAST_B2B_CAP
   *
   * Field size: 1 bit
   */
  uint8_t fastB2B;
  /**
   * @brief [ro] 66MHz Capable
   *
   * Hardwired to 0 for PCIExpress.  Only defined on rev 1 hw version.
   *
   * Forced to 0 on rev 0 hw read; ignored on rev 0 hw write.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to C66MHZ_CAP
   * On rev 2 hw, this corresponds to FAST_66MHZ_CAP
   *
   * Field size: 1 bit
   */
  uint8_t c66MhzCap;
  /**
   * @brief [ro] cap list
   *
   * For PCIe, this field must be set to 1.
   *
   * On rev 0 hw, this corresponds to CAP_LIST
   * On rev 1 hw, this corresponds to CAP_LIST
   * On rev 2 hw, this corresponds to CAP_LIST
   *
   * Field size: 1 bit
   */
  uint8_t capList;
  /**
   * @brief [rw] stat
   *
   * Indicates that the function has received an interrupt.
   *
   * On rev 0 hw, this corresponds to INT_STAT
   * On rev 1 hw, this corresponds to INTX_STATUS
   * On rev 2 hw, this corresponds to INT_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t stat;
  /**
   * @brief [ro] dis
   *
   * Setting this bit disables generation of INTx messages.
   *
   * On rev 0 hw, this corresponds to INTX_DIS
   * On rev 1 hw, this corresponds to INTX_ASSER_DIS
   * On rev 2 hw, this corresponds to PCI_TYPE0_INT_EN and INT_EN
   *
   * Field size: 1 bit
   */
  uint8_t dis;
  /**
   * @brief [rw] serr en
   *
   * When set, it enables generation of the appropriate PCI Express error
   * messages to the Root Complex.
   *
   * On rev 0 hw, this corresponds to SERR_EN
   * On rev 1 hw, this corresponds to SERR_EN
   * On rev 2 hw, this corresponds to PCI_TYPE0_SERREN and SERREN
   *
   * Field size: 1 bit
   */
  uint8_t serrEn;
  /**
   * @brief [ro] Bit hardwired to 0 for PCIExpress
   *
   * Hardwired to 0 for PCIExpress.  Only defined on rev 1/2 hw version.
   *
   * Forced to 0 on rev 0 hw read; ignored on rev 0 hw write.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to IDSEL_CTRL
   * On rev 2 hw, this corresponds to PCI_TYPE_IDSEL_STEPPING and IDSEL
   *
   * Field size: 1 bit
   */
  uint8_t idselCtrl;
  /**
   * @brief [rw] resp
   *
   * This bit controls whether or not the device responds to detected
   * parity errors (poisoned TLP). This error is typically reported as an
   * unsupported request and may also result in a non-fatal error
   * message if @ref serrEn = 1. If this bit is set, the PCIESS will respond
   * normally to parity errors. If this bit is cleared, the PCIESS
   * will ignore detected parity errors.
   *
   * On rev 0 hw, this corresponds to PAR_ERR_RESP
   * On rev 1 hw, this corresponds to PARITYERRRESP
   * On rev 2 hw, this corresponds to PCI_TYPE0_PARITY_ERR_EN and PERREN
   *
   * Field size: 1 bit
   */
  uint8_t resp;
  /**
   * @brief [ro] Bit hardwired to 0 for PCIExpress
   *
   * Hardwired to 0 for PCIExpress.  Only defined on rev 1/2 hw version.
   *
   * Forced to 0 on rev 0 hw read; ignored on rev 0 hw write.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to VGA_SNOOP
   * On rev 2 hw, this corresponds to PCI_TYPE_VGA_PALETTE_SNOOP and VGAPS
   *
   * Field size: 1 bit
   */
  uint8_t vgaSnoop;
  /**
   * @brief [ro] Bit hardwired to 0 for PCIExpress
   *
   * Hardwired to 0 for PCIExpress.  Only defined on rev 1/2 hw version.
   *
   * Forced to 0 on rev 0 hw read; ignored on rev 0 hw write.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to MEMWR_INVA
   * On rev 2 hw, this corresponds to PCI_TYPE_MWI_ENABLE / MWI_EN
   *
   * Field size: 1 bit
   */
  uint8_t memWrInva;
  /**
   * @brief [ro] Bit hardwired to 0 for PCIExpress
   *
   * Hardwired to 0 for PCIExpress.  Only defined on rev 1/2 hw version.
   *
   * Forced to 0 on rev 0 hw read; ignored on rev 0 hw write.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to SPEC_CYCLE_EN
   * On rev 2 hw, this corresponds to PCI_TYPE0_SPECIAL_CYCLE_OPERATION and SCO
   *
   * Field size: 1 bit
   */
  uint8_t specCycleEn;
  /**
   * @brief [rw] enables mastership of the bus
   *
   * On rev 0 hw, this corresponds to BUS_MS
   * On rev 1 hw, this corresponds to BUSMASTER_EN
   * On rev 2 hw, this corresponds to PCI_TYPE0_BUS_MASTER_EN and BME
   *
   * Field size: 1 bit
   */
  uint8_t busMs;
  /**
   * @brief [rw] enables device to respond to memory access
   *
   * On rev 0 hw, this corresponds to MEM_SP
   * On rev 1 hw, this corresponds to MEM_SPACE_EN
   * On rev 2 hw, this corresponds to PCI_TYPE0_MEM_SPACE_EN and MSE
   *
   * Field size: 1 bit
   */
  uint8_t memSp;
  /**
   * @brief [rw] enables device to respond to IO access
   *
   * This functionality is not supported in PCIESS and therefore
   * this bit is set to 0.
   *
   * On rev 0 hw, this corresponds to IO_SP
   * On rev 1 hw, this corresponds to IO_SPACE_EN
   * On rev 2 hw, this corresponds to PCI_TYPE0_IO_EN / IO_EN
   *
   * Field size: 1 bit
   */
  uint8_t ioSp;
} pcieStatusCmdReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_com_structures
 * @brief Specification of the Class code and revision ID Register
 *
 * On rev 0 hw, this corresponds to CLASSCODE_REVID
 * On rev 1 hw, this corresponds to CLASSCODE_REVISIONID
 * On rev 2 hw, this corresponds to CLASS_CODE_REVISION_ID
 *
 * @{
 */
typedef struct pcieRevIdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Class Code
   *
   * On rev 0 hw, the register presents this as 24 bits.
   * On rev 1/2 hw, the register presents as 3 8bit fields, but they will all be
   * packed/unpacked from classCode for backward/forward compatibility.
   *
   * On rev 0 hw, this corresponds to CLASSCODE
   * On rev 1 hw, this corresponds to BASE_CLS_CD, SUBCLS_CD and PROG_IF_CODE
   * On rev 2 hw, this corresponds to BASE_CLASS_CODE, SUBCLASS_CODE and PROGRAM_INTERFACE
   *
   * Field size: 24 bits
   */
  uint32_t classCode;
  /**
   * @brief [ro] Revision ID
   *
   * On rev 0 hw, this corresponds to REVID
   * On rev 1 hw, this corresponds to REVID
   * On rev 2 hw, this corresponds to REVID
   *
   * Field size: 8 bits
   */
  uint8_t revId;
} pcieRevIdReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Base Address Register (BAR)
 *
 * This should be used to access a BAR register.
 *
 * There are two situations when this structure should be used:\n
 * 1. When setting up a 32 bit BAR\n
 * 2. When setting up the lower 32bits of a 64bits BAR
 *
 * Refer to @ref pcieBar32bitReg_t for the other possible BAR configurations
 *
 * On rev 0 hw, this corresponds to BARn
 * On rev 1 hw, this corresponds to BARn
 * On rev 2 hw, this corresponds to BAR_REGn
 *
 * @{
 */
typedef struct pcieBarReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Base Address
   *
   * Rev 0 hw: 28 bits are modifiable
   * Rev 1 hw: only upper 12 bits are modifyable (eg 16-27).  Rest are read-only.  If users tries
   * to modify r/o bits will return @ref pcie_RET_RO_CHANGED.
   * Rev 2 hw: 28 bits are modifiable
   *
   * Field size: 28 bits
   */
  uint32_t base;
  /**
   * @brief [rw] Prefetchable region?
   *
   * For memory BARs, it indicates whether the region is prefetchable.\n
   * 0 = Non-prefetchable.\n
   * 1 = Prefetchable.
   *
   * For I/O Bars, it is used as second least significant bit (LSB)
   * of the base address.
   *
   * Field size: 1 bit
   */
  uint8_t prefetch;
  /**
   * @brief [rw] Bar Type
   *
   * For memory BARs, they determine the BAR type.\n
   * 0h = 32-bit BAR.\n
   * 2h = 64-bit BAR.\n
   * Others = Reserved.
   *
   * For I/O BARs, bit 2 is the least significant bit (LSB) of the
   * base address and bit 1 is 0.
   *
   * Field size: 2 bits
   */
  uint8_t type;
  /**
   * @brief [rw] Memory or IO BAR
   *
   * 0 = Memory BAR.\n
   * 1 = I/O BAR.
   *
   * Field size: 1 bit
   */
  uint8_t memSpace;
} pcieBarReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Base Address Register (BAR).
 *
 * This should be used to read/write a 32bit word to the BAR register.
 *
 * There are two situations when this structure should be used:\n
 * 1. When setting up BAR masks\n
 * 2. When setting up the upper 32bits of a 64bits BAR
 *
 * Refer to @ref pcieBarReg_t for the other possible BAR configurations
 *
 * @{
 */
typedef struct pcieBar32bitReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] 32bits word (BAR mask or BAR address)
   *
   * On rev 0 hardware, may set bar masks through either 
   * @ref pcieRegisters_t::type0BarMask32bitIdx or
   * @ref pcieRegisters_t::type0Bar32bitIdx.  The determination on 
   * whether its a bar mask or bar is made by current value of 
   * @ref pcieCmdStatusReg_s::dbi.
   *
   * On rev 1 hardware, MUST set bar masks only through 
   * @ref pcieRegisters_t::type0BarMask32bitIdx as this routes through
   * DBICS2.  Also, MUST set upper 32 bits using only 
   * @ref pcieRegisters_t::type0Bar32bitIdx to route through DBICS.
   *
   * Field size: 32 bits
   */
  uint32_t reg32;
} pcieBar32bitReg_t;
/* @} */

/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 0 REGISTERS  **********************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the BIST Header Register
 *
 * On rev 0 hw, this corresponds to BIST_HEADER
 * On rev 1 hw, this corresponds to BIST_HEAD_LAT_CACH
 * On rev 2 hw, this corresponds to BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG
 *
 * @{
 */
typedef struct pcieBistReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Bist capability
   *
   * Returns a one for BIST capability and zero otherwise. Not supported by PCIESS.
   *
   * This field is only used on rev 0 hw.  A value of 0 is returned on rev 1 hw.
   *
   * On rev 0 hw, this corresponds to BIST_CAP
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t bistCap;
  /**
   * @brief [ro] Start Bist
   *
   * Write a one to start BIST. Not supported by PCIESS.
   *
   * This field is only used on rev 0 hw.  A value of 0 is returned on rev 1 hw.
   *
   * On rev 0 hw, this corresponds to START_BIST
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t startBist;
  /**
   * @brief [ro]  Completion code
   *
   * Not supported by PCIESS.
   *
   * This field is only used on rev 0 hw.  A value of 0 is returned on rev 1 hw.
   *
   * On rev 0 hw, this corresponds to COMP_CODE
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 4 bits
   */
  uint8_t compCode;
  /**
   * @brief [ro]  hw rev 1 bist field
   *
   * This field is only used on rev 1 hw.  A value of 0 is returned on rev 0 hw.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to BIST
   * On rev 2 hw, this corresponds to BIST
   *
   * Field size: 8 bits
   */
  uint8_t bist;
  /**
   * @brief [ro]  Multifunction device
   *
   * On rev 0 hw, this corresponds to MULFUN_DEV
   * On rev 1 hw, this corresponds to MFD
   * On rev 2 hw, this corresponds to MULTI_FUNC
   *
   * Field size:  1 bit
   */
  uint8_t mulfunDev;
  /**
   * @brief [ro]  Header type
   *
   * Configuration header format.
   *
   * 0 = EP mode\n
   * 1 = RC mode
   *
   * On rev 0 hw, this corresponds to HDR_TYPE
   * On rev 1 hw, this corresponds to HEAD_TYP
   * On rev 1 hw, this corresponds to HEADER_TYPE
   *
   * Field size: 7 bits
   */
  uint8_t hdrType;
  /**
   * @brief [ro] Not applicable in PCIe
   *
   * On rev 0 hw, this corresponds to LAT_TMR
   * On rev 1 hw, this corresponds to MSTR_LAT_TIM
   * On rev 2 hw, this corresponds to LATENCY_MASTER_TIMER
   *
   * Field size:  8 bits
   */
  uint8_t latTmr;
  /**
   * @brief [ro] Not applicable in PCIe
   *
   * On rev 0 hw, this corresponds to CACHELN_SIZ
   * On rev 1 hw, this corresponds to CACH_LN_SZE
   * On rev 2 hw, this corresponds to CACHE_LINE_SIZE
   *
   * Field size:  8 bits
   */
  uint8_t cacheLnSize;
} pcieBistReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief @ref pcieBarReg_s register plus an index (End Point BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access an End Point BAR. For more details, please refer to @ref pcieBarReg_t
 *
 * @{
 */
typedef struct pcieType0BarIdx_s {
  pcieBarReg_t reg;  /**< @brief Register Structure */
  uint8_t      idx;  /**< @brief Index in the array of registers of this type */
} pcieType0BarIdx_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief @ref pcieBar32bitReg_s register plus an index (End Point BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access an End Point BAR. For more details, please refer to @ref pcieBar32bitReg_t
 *
 * @{
 */
typedef struct pcieType0Bar32bitIdx_s {
  pcieBar32bitReg_t reg;  /**< @brief Register Structure */
  uint8_t           idx;  /**< @brief Index in the array of registers of this type */
} pcieType0Bar32bitIdx_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Cardbus CIS pointer register
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, this corresponds to CARDBUS_CIS_POINTER
 * On rev 2 hw, this corresponds to CARDBUS_CIS_PTR_REG
 *
 * @{
 */
typedef struct pcieCardbusCisPointerReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Cardbus CIS pointer (CS)
   *
   * This register is only used in rev 1/2 hw.  It is physically present but marked reserved
   * in rev 0 hardware, so this structure/API isn't used.
   *
   * Field size: 32 bits
   */
  uint32_t cisPointer;
} pcieCardbusCisPointerReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Subsystem Vendor ID Register
 *
 * On rev 0 hw, this corresponds to SUBSYS_VNDR_ID
 * On rev 1 hw, this corresponds to SUBID_SUBVENDORID
 * On rev 2 hw, this corresponds to SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG
 *
 * @{
 */
typedef struct pcieSubIdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Subsystem ID
   *
   * On rev 0 hw, this corresponds to SUBSYS_ID
   * On rev 1 hw, this corresponds to SUBSYS_DEV_ID_N
   * On rev 2 hw, this corresponds to SUBSYS_DEV_ID
   *
   * Field size: 16 bits
   */
  uint16_t subId;
  /**
   * @brief [ro] Subsystem Vendor ID
   *
   * On rev 0 hw, this corresponds to SUBSYS_VEN_ID
   * On rev 1 hw, this corresponds to SUBSYS_VENDOR_ID_N
   * On rev 2 hw, this corresponds to SUBSYS_VENDOR_ID
   *
   * Field size: 16 bits
   */
  uint16_t subVndId;
} pcieSubIdReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Expansion ROM Register
 *
 * On rev 0 hw, this corresponds to EXPNSN_ROM
 * On rev 1 hw, this corresponds to EXPANSION_ROM_BAR
 * On rev 2 hw, this corresponds to EXP_ROM_BASE_ADDR_REG
 *
 * @{
 */
typedef struct pcieExpRomReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] rev 0; [rw] rev 1 Address of Expansion ROM
   *
   * Rev 0 hw: entire 21 bits are r/o and are always 0.
   *
   * Rev 1 hw: only upper 16 bits are modifyable (eg 16-31).  Lower 5 bits are
   * r/o.  Attempt to modify r/o bits will return @ref pcie_RET_RO_CHANGED.
   *
   * Rev 2 hw: entire 21 bits are r/w
   *
   * On rev 0 hw, this corresponds to EXP_ROM_BASE_ADDR and EXROM_ADDRESS_RO
   * On rev 1 hw, this corresponds to EXROM_ADDRESS
   * On rev 2 hw, this corresponds to EXP_ROM_BASE_ADDRES
   *
   * Field size: 21 bits
   */
  uint32_t expRomAddr;
  /**
   * @brief [ro] rev 0; [rw] rev 1: Expansion ROM Enable
   *
   * On rev 0 hw, this corresponds to EXP_ROM_EN
   * On rev 1 hw, this corresponds to EXROM_EN
   * On rev 2 hw, this corresponds to ROM_BAR_ENABLE
   *
   * Field size: 1 bit
   */
  uint8_t enable;
} pcieExpRomReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Capability Pointer Register
 *
 * On rev 0 hw, this corresponds to CAP_PTR
 * On rev 1 hw, this corresponds to CAPPTR
 * On rev 2 hw, this corresponds to PCI_CAP_PTR_REG
 *
 * @{
 */
typedef struct pcieCapPtrReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] First Capability Pointer
   *
   * By default, it points to Power Management Capability structure.
   *
   * On rev 0 hw, this corresponds to CAP_PTR
   * On rev 1 hw, this corresponds to CAPPTR
   * On rev 2 hw, this corresponds to CAP_POINTER
   *
   * Field size: 8 bits
   */
  uint8_t ptr;
} pcieCapPtrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Interrupt Pin Register
 *
 * On rev 0 hw, this corresponds to INT_PIN
 * On rev 1 hw, this corresponds to INTERRUPT
 * On rev 2 hw, this corresponds to MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG
 *
 * @{
 */
typedef struct pcieIntPinReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] interrupt Pin
   *
   * It identifies the legacy interrupt message that the device uses.
   * For single function configuration, the core only uses INTA.
   *
   * <TABLE>
   * <TR><TH>@ref intPin</TH><TH>Legacy Interrupt</TH></TR>
   * <TR><TD>0</TD>          <TD>none</TD></TR>
   * <TR><TD>1</TD>          <TD>INTA</TD></TR>
   * <TR><TD>2</TD>          <TD>INTB</TD></TR>
   * <TR><TD>3</TD>          <TD>INTC</TD></TR>
   * <TR><TD>4</TD>          <TD>INTD</TD></TR>
   * <TR><TD>others</TD>     <TD>reserved</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to INT_PIN
   * On rev 1 hw, this corresponds to INT_PIN
   * On rev 2 hw, this corresponds to INT_PIN
   *
   * Field size: 8 bits
   */
  uint8_t intPin;
  /**
   * @brief [rw] interrupt line
   *
   * On rev 0 hw, this corresponds to INT_LINE
   * On rev 1 hw, this corresponds to INT_LIN
   * On rev 2 hw, this corresponds to INT_LIN
   *
   * Field size: 8 bits
   */
  uint8_t intLine;
} pcieIntPinReg_t;
/* @} */


/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 1 REGISTERS  **********************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief @ref pcieBarReg_s register plus an index (Root Complex BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access a Root Complex BAR. For more details, please refer to @ref pcieBarReg_t.
 *
 * On rev 2 hw, not supported (use IATU instead)
 *
 * @{
 */
typedef struct pcieType1BarIdx_s {
  pcieBarReg_t reg;  /**< @brief Register Structure */
  uint8_t      idx;  /**< @brief Index in the array of registers of this type */
} pcieType1BarIdx_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief @ref pcieBar32bitReg_s register plus an index (Root Complex BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access a Root Complex BAR. For more details, please refer to @ref pcieBar32bitReg_t.
 *
 * On rev 2 hw, not supported (use IATU instead)
 *
 * @{
 */
typedef struct pcieType1Bar32bitIdx_s {
  pcieBar32bitReg_t reg;  /**< @brief Register Structure */
  uint8_t           idx;  /**< @brief Index in the array of registers of this type */
} pcieType1Bar32bitIdx_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the BIST, Header Type, Latency Time and Cache Line Size Regiser
 *
 * On rev 0 hw, this corresponds to BIST_HEADER
 * On rev 1 hw, this corresponds to BIST_HEAD_LAT_CACH
 * On rev 2 hw, this corresponds to TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG
 *
 * @{
 */
typedef struct pcieType1BistHeaderReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Returns a 1 for BIST capability and 0 otherwise.
   *
   * Not supported by PCIESS.
   *
   * On rev 0 hw, this corresponds to BIST_CAP
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t bistCap;
  /**
   * @brief [ro] Write a one to start BIST.
   *
   * Not supported by PCIESS.
   *
   * On rev 0 hw, this corresponds to START_BIST
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t startBist;
  /**
   * @brief [rw] Completion Code.
   *
   * Not supported by PCIESS.
   *
   * On rev 0 hw, this corresponds to COMP_CODE
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 4 bits
   */
  uint8_t compCode;
  /**
   * @brief [ro]  hw rev 1 bist field
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to BIST
   * On rev 2 hw, this corresponds to BIST
   *
   * Field size: 8 bits
   */
  uint8_t bist;
  /**
   * @brief [rw] Returns 1 if it is a multi-function device.
   *
   * On rev 0 hw, this corresponds to MULFUN_DEV
   * On rev 1 hw, this corresponds to MFD
   * On rev 2 hw, this corresponds to MULTI_FUNC
   *
   * Field size: 1 bit
   */
  uint8_t mulFunDev;
  /**
   * @brief [rw] Configuration Header Format.
   *
   * 0 = EP mode\n
   * 1 = RC mode
   *
   * On rev 0 hw, this corresponds to HDR_TYPE
   * On rev 1 hw, this corresponds to HEAD_TYP
   * On rev 2 hw, this corresponds to HEADER_TYPE
   *
   * Field size: 7 bits
   */
  uint8_t hdrType;
  /**
   * @brief [ro] Not applicable in PCIe
   *
   * On rev 0 hw, this corresponds to LAT_TMR
   * On rev 1 hw, this corresponds to MSTR_LAT_TIM
   * On rev 2 hw, this corresponds to LATENCY_MASTER_TIMER
   *
   * Field size: 8 bits
   */
  uint8_t latTmr;
  /**
   * @brief [ro] Not applicable in PCIe
   *
   * On rev 0 hw, this corresponds to CACHELN_SIZE
   * On rev 1 hw, this corresponds to CACH_LN_SIZE
   * On rev 2 hw, this corresponds to CACHE_LINE_SIZE
   *
   * Field size: 8 bits
   */
  uint8_t cacheLnSize;
} pcieType1BistHeaderReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Latency Timer and Bus Number Register
 *
 * On rev 0 hw, this corresponds to BUSNUM
 * On rev 1 hw, this corresponds to BUS_NUM_REG
 * On rev 2 hw, this corresponds to SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG
 *
 * @{
 */
typedef struct pcieType1BusNumReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Secondary Latency Timer (N/A for PCIe)
   *
   * On rev 0 hw, this corresponds to SEC_LAT_TMR
   * On rev 1 hw, this corresponds to SEC_LAT_TIMER
   * On rev 2 hw, this corresponds to SEC_LAT_TIMER
   *
   * Field size: 8 bits
   */
  uint8_t secLatTmr;
  /**
   * @brief [rw] Subordinate Bus Number. This is highest bus
   *             number on downstream interface.
   *
   * On rev 0 hw, this corresponds to SUB_BUS_NUM
   * On rev 1 hw, this corresponds to SUBORD_BUS_NUM
   * On rev 2 hw, this corresponds to SUB_BUS
   *
   * Field size: 8 bits
   */
  uint8_t subBusNum;
  /**
   * @brief [rw] Secondary Bus Number. It is typically 1h for RC.
   *
   * On rev 0 hw, this corresponds to SEC_BUS_NUM
   * On rev 1 hw, this corresponds to SEC_BUS_NUM
   * On rev 2 hw, this corresponds to SEC_BUS
   *
   * Field size: 8 bits
   */
  uint8_t secBusNum;
  /**
   * @brief [rw] Primary Bus Number. It is 0 for RC and nonzero for
   *             switch devices only.
   *
   * On rev 0 hw, this corresponds to PRI_BUS_NUM
   * On rev 1 hw, this corresponds to PRIM_BUS_NUM
   * On rev 2 hw, this corresponds to PRIM_BUS
   *
   * Field size: 8 bits
   */
  uint8_t priBusNum;
} pcieType1BusNumReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Secondary Status and IO Base/Limit Register
 *
 * On rev 0 hw, this corresponds to SECSTAT
 * On rev 1 hw, this corresponds to IOBASE_LIMIT_SEC_STATUS
 * On rev 2 hw, this corresponds to SEC_STAT_IO_LIMIT_IO_BASE_REG
 *
 * @{
 */
typedef struct pcieType1SecStatReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Detected Parity Error.
   *
   * Read 1 if received a poisoned TLP.
   * Write 1 to clear; write 0 has no effect.
   *
   * On rev 0 hw, this corresponds to DTCT_PERROR
   * On rev 1 hw, this corresponds to DET_PAR_ERR
   * On rev 2 hw, this corresponds to SEC_STAT_DPE
   *
   * Field size: 1 bit
   */
  uint8_t dtctPError;
  /**
   * @brief [rw] Received System Error.
   *
   * Read 1 if received an ERR_FATAL or ERR_NONFATAL message.
   * Write 1 to clear; write 0 has no effect.
   *
   * On rev 0 hw, this corresponds to RX_SYS_ERROR
   * On rev 1 hw, this corresponds to RCVD_SYS_ERR
   * On rev 2 hw, this corresponds to SEC_STAT_RCVD_SYS_ERR
   *
   * Field size: 1 bit
   */
  uint8_t rxSysError;
  /**
   * @brief [rw] Received Master Abort.
   *
   * Read 1 if received a completion with unsupported request completion status.
   * Write 1 to clear; write 0 has no effect.
   *
   * On rev 0 hw, this corresponds to RX_MST_ABORT
   * On rev 1 hw, this corresponds to RCVD_MSTR_ABORT
   * On rev 2 hw, this corresponds to SEC_STAT_RCVD_MSTR_ABRT
   *
   * Field size: 1 bit
   */
  uint8_t rxMstAbort;
  /**
   * @brief [rw] Received Target Abort.
   *
   * Read 1 if received a completion with completer abort completion status.
   * Write 1 to clear; write 0 has no effect.
   *
   * On rev 0 hw, this corresponds to RX_TGT_ABORT
   * On rev 1 hw, this corresponds to RCVD_TRGT_ABORT
   * On rev 2 hw, this corresponds to SEC_STAT_RCVD_TRGT_ABRT
   *
   * Field size: 1 bit
   */
  uint8_t rxTgtAbort;
  /**
   * @brief [rw] Signaled Target Abort.
   *
   * Read 1 if sent a posted or non-posted request as a completer abort error.
   * Write 1 to clear; write 0 has no effect.
   *
   * On rev 0 hw, this corresponds to TX_TGT_ABORT
   * On rev 1 hw, this corresponds to SGNLD_TRGT_ABORT
   * On rev 2 hw, this corresponds to SEC_STAT_SIG_TRGT_ABRT
   *
   * Field size: 1 bit
   */
  uint8_t txTgtAbort;
  /**
   * @brief [ro] DEVSEL Timing
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to DEVSEL_TIMING
   * On rev 2 hw, unsupported
   *
   * Field size: 2 bits
   */
  uint8_t devselTiming;
  /**
   * @brief [rw] Master Data Parity Error.
   *
   * Read 1 if the parity error enable bit
   * @ref pcieType1BridgeIntReg_s::pErrRespEn is set and either the condition
   * that the requester receives a poisoned completion or the condition
   * that the requester poisons a write request is true.
   * Write 1 to clear; write 0 has no effect.
   *
   * On rev 0 hw, this corresponds to MST_DPERR
   * On rev 1 hw, this corresponds to MSTR_DATA_PRTY_ERR
   * On rev 2 hw, this corresponds to SEC_STAT_MDPE
   *
   * Field size: 1 bit
   */
  uint8_t mstDPErr;
  /**
   * @brief [ro] Fast Back to Back Capable.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to FAST_B2B_CAP
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t fastB2bCap;
  /**
   * @brief [ro] 66Mhz Capable.
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to C66MHZ_CAPA
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t c66mhzCapa;
  /**
   * @brief [rw] Upper 4 bits of 16bit IO Space Limit Address.
   *
   * On rev 0 hw, this corresponds to IO_LIMIT
   * On rev 1 hw, this corresponds to IO_SPACE_LIMIT
   * On rev 2 hw, this corresponds to IO_LIMIT
   *
   * Field size: 4 bits
   */
  uint8_t IOLimit;
  /**
   * @brief [rw] Indicates addressing for IO Limit Address.
   *
   * Writable from internal bus interface.
   * 0 = 16-bit IO addressing.
   * 1 = 32-bit IO addressing.
   *
   * On rev 0 hw, this corresponds to IO_LIMIT_ADDR
   * On rev 1 hw, this corresponds to IODECODE_32
   * On rev 2 hw, this corresponds to IO_DECODE_BITS
   *
   * Field size: 1 bit
   */
  uint8_t IOLimitAddr;
  /**
   * @brief [rw] Upper 4 bits of 16bit IO Space Base Address.
   *
   * On rev 0 hw, this corresponds to IO_BASE
   * On rev 1 hw, this corresponds to IO_SPACE_BASE
   * On rev 2 hw, this corresponds to IO_BASE
   *
   * Field size: 4 bits
   */
  uint8_t IOBase;
  /**
   * @brief [rw] Indicates addressing for the IO Base Address.
   *
   * Writable from internal bus interface.
   * 0 = 16-bit IO addressing.
   * 1 = 32-bit IO addressing.
   *
   * On rev 0 hw, this corresponds to IO_BASE_ADDR
   * On rev 1 hw, this corresponds to IODECODE_32_0
   * On rev 2 hw, this corresponds to IO_DECODE
   *
   * Field size: 1 bit
   */
  uint8_t IOBaseAddr;
} pcieType1SecStatReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Memory Limit and Base Register
 *
 * On rev 0 hw, this corresponds to MEMSPACE
 * On rev 1 hw, this corresponds to MEM_BASE_LIMIT
 * On rev 2 hw, this corresponds to MEM_LIMIT_MEM_BASE_REG
 *
 * @{
 */
typedef struct pcieType1MemspaceReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Upper 12 bits of 32bit Memory Limit Address.
   *
   * On rev 0 hw, this corresponds to MEM_LIMIT
   * On rev 1 hw, this corresponds to MEM_LIMIT_ADDR
   * On rev 2 hw, this corresponds to MEM_LIMIT
   *
   * Field size: 12 bits
   */
  uint16_t limit;
  /**
   * @brief [rw] Upper 12 bits of 32bit Memory Base Address.
   *
   * On rev 0 hw, this corresponds to MEM_BASE
   * On rev 1 hw, this corresponds to MEM_BASE_ADDR
   * On rev 2 hw, this corresponds to MEM_BASE
   *
   * Field size: 12 bit
   */
  uint16_t base;
} pcieType1MemspaceReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Prefetchable Memory Limit and Base Register
 *
 * On rev 0 hw, this corresponds to PREFETCH_MEM
 * On rev 1 hw, this corresponds to PREF_MEM_BASE_LIMIT
 * On rev 2 hw, this corresponds to PREF_MEM_LIMIT_PREF_MEM_BASE_REG
 *
 * @{
 */
typedef struct pciePrefMemReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Memory limit
   *
   * Upper 12 bits of 32bit prefetchable memory limit address (end address).
   *
   * On rev 0 hw, this corresponds to PREFETCH_LIMIT
   * On rev 1 hw, this corresponds to PREF_MEM_ADDR
   * On rev 2 hw, this corresponds to PREF_MEM_LIMIT
   *
   * Field size: 12 bits
   */
  uint16_t limit;
  /**
   * @brief [rw] 32 or 64 bit addressing
   *
   * Indicates addressing for prefetchable memory limit address (end address).
   *
   * 0 = 32-bit memory addressing\n
   * 1 = 64-bit memory addressing
   *
   * On rev 0 hw, this corresponds to PRE_LIMIT_ADDR
   * On rev 1 hw, this corresponds to MEMDECODE_64
   * On rev 2 hw, this corresponds to MEM_LIMIT_DECODE
   *
   * Field size: 1 bit
   */
  uint8_t limitAddr;
  /**
   * @brief [rw] Memory base
   *
   * Upper 12 bits of 32bit prefetchable memory base address (start address).
   *
   * On rev 0 hw, this corresponds to PREFETCH_BASE
   * On rev 1 hw, this corresponds to UPPPREF_MEM_ADDR
   * On rev 2 hw, this corresponds to PREF_MEM_BASE
   *
   * Field size: 12 bits
   */
  uint16_t base;
  /**
   * @brief [rw] 32 or 64 bit addressing
   *
   * Indicates addressing for the prefetchable memory base address (start address).
   *
   * 0 = 32-bit memory addressing\n
   * 1 = 64-bit memory addressing
   *
   * On rev 0 hw, this corresponds to PRE_BASE_ADDR
   * On rev 1 hw, this corresponds to MEMDECODE_64_0
   * On rev 2 hw, this corresponds to PREF_MEM_DECODE
   *
   * Field size: 1 bit
   */
  uint8_t baseAddr;
} pciePrefMemReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Prefetchable Memory Base Upper Register
 *
 * On rev 0 hw, this corresponds to PREFETCH_BASE
 * On rev 1 hw, this corresponds to UPPER_32BIT_PREF_BASEADDR
 * On rev 2 hw, this corresponds to PREF_BASE_UPPER_REG
 *
 * @{
 */
typedef struct pciePrefBaseUpperReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Base upper 32bits
   *
   * Upper 32 bits of Prefetchable Memory Base Address. Used with 64bit
   * prefetchable memory addressing only.
   *
   * On rev 0 hw, this corresponds to PREFETCH_BASE
   * On rev 1 hw, this corresponds to ADDRUPP
   * On rev 2 hw, this corresponds to PREF_MEM_BASE_UPPER
   *
   * Field size: 32 bits
   */
  uint32_t base;
} pciePrefBaseUpperReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Prefetchable Memory Limit Upper Register
 *
 * On rev 0 hw, this corresponds to PREFETCH_LIMIT
 * On rev 1 hw, this corresponds to UPPER_32BIT_PREF_LIMITADDR
 * On rev 2 hw, this corresponds to PREF_LIMIT_UPPER_REG
 *
 * @{
 */
typedef struct pciePrefLimitUpperReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Base upper 32bits
   *
   * Upper 32 bits of Prefetchable Memory Limit Address. Used with 64 bit
   * prefetchable memory addressing only.
   *
   * On rev 0 hw, this corresponds to PREFETCH_LIMIT
   * On rev 1 hw, this corresponds to ADDRUPP_LIMIT
   * On rev 2 hw, this corresponds to PREF_MEM_LIMIT_UPPER
   *
   * Field size: 32 bits
   */
  uint32_t limit;
} pciePrefLimitUpperReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the IO Base and Limit Upper 16 bits Register
 *
 * On rev 0 hw, this corresponds to IOSPACE
 * On rev 1 hw, this corresponds to IO_BASE_LIMIT
 * On rev 2 hw, this corresponds to IO_LIMIT_UPPER_IO_BASE_UPPER_REG
 *
 * @{
 */
typedef struct pcieType1IOSpaceReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Upper 16 bits of IO Base Address.
   *
   * Used with 32 bit IO space addressing only.
   *
   * On rev 0 hw, this corresponds to IOBASE
   * On rev 1 hw, this corresponds to UPP16_IOBASE
   * On rev 2 hw, this corresponds to IO_BASE_UPPER
   *
   * Field size: 16 bits
   */
  uint16_t IOBase;
  /**
   * @brief [rw] Upper 16 bits of IO Limit Address.
   *
   * Used with 32 bit IO space addressing only.
   *
   * On rev 0 hw, this corresponds to IOLIMIT
   * On rev 1 hw, this corresponds to UPP16_IOLIMIT
   * On rev 2 hw, this corresponds to IO_LIMIT_UPPER
   *
   * Field size: 16 bits
   */
  uint16_t IOLimit;
} pcieType1IOSpaceReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Capabilities Pointer Register
 *
 * On rev 0 hw, this corresponds to CAP_PTR
 * On rev 1 hw, this corresponds to CAPPTR
 * On rev 2 hw, this corresponds to TYPE1_CAP_PTR_REG
 *
 * @{
 */
typedef struct pcieType1CapPtrReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] First Capability Pointer.
   *
   * By default, it points to Power Management Capability structure.
   * Writable from internal bus interface.
   *
   * On rev 0 hw, this corresponds to CAP_PTR
   * On rev 1 hw, this corresponds to CAPPTR
   * On rev 2 hw, this corresponds to CAP_POINTER
   *
   * Field size: 8 bits
   */
  uint8_t capPtr;
} pcieType1CapPtrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Expansion ROM Base Address Register
 *
 * On rev 0 hw, this corresponds to EXPNSN_ROM
 * On rev 1 hw, this corresponds to EXPANSION_ROM_BAR
 * On rev 2 hw, this corresponds to TYPE1_EXP_ROM_BASE_REG
 *
 * @{
 */
typedef struct pcieType1ExpnsnRomReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Address of Expansion ROM
   *
   * Rev 0 hw: 21 bits are modifiable
   * Rev 1 hw: only upper 16 bits are modifyable (eg 16-20).  Rest are read-only.  If users tries
   * to modify r/o bits will return @ref pcie_RET_RO_CHANGED.
   * Rev 2 hw: 21 bits are modifiable
   *
   * On rev 0 hw, this corresponds to EXP_ROM_BASE_ADDR
   * On rev 1 hw, this corresponds to EXROM_ADDRESS
   * On rev 2 hw, this corresponds to EXP_ROM_BASE_ADDRESS
   *
   * Field size: 21CAPPTR bits [0-0x1FFFFF]
   */
  uint32_t expRomBaseAddr;
  /**
   * @brief [rw] Expansion ROM enable
   *
   * On rev 0 hw, this corresponds to EXP_ROM_EN
   * On rev 1 hw, this corresponds to EXP_ROM_EN
   * On rev 2 hw, this corresponds to ROM_BAR_ENABLE
   *
   * Field size: 1 bit
   */
  uint8_t expRomEn;
} pcieType1ExpnsnRomReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Bridge Control and Interrupt Register
 *
 * On rev 0 hw, this corresponds to BRIDGE_INT
 * On rev 1 hw, this corresponds to BRIDGE_INT
 * On rev 2 hw, this corresponds to BRIDGE_CTRL_INT_PIN_INT_LINE_REG
 *
 * @{
 */
typedef struct pcieType1BridgeIntReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Discard Timer SERR Enable Status.
   *
   * Not Applicable to PCI Express. Hardwired to 0.
   *
   * On rev 0 hw, this corresponds to SERREN_STATUS
   * On rev 1 hw, this corresponds to DT_SERR_EN
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t serrEnStatus;
  /**
   * @brief [ro] Discard Timer Status.
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * On rev 0 hw, this corresponds to TIMER_STATUS
   * On rev 1 hw, this corresponds to DT_STS
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t timerStatus;
  /**
   * @brief [ro] Secondary Discard Timer.
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * On rev 0 hw, this corresponds to SEC_TIMER
   * On rev 1 hw, this corresponds to SEC_DT
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t secTimer;
  /**
   * @brief [ro] Primary Discard Timer.
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * On rev 0 hw, this corresponds to PRI_TIMER
   * On rev 1 hw, this corresponds to PRI_DT
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t priTimer;
  /**
   * @brief [ro] Fast Back to Back Transactions Enable.
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * On rev 0 hw, this corresponds to B2B_EN
   * On rev 1 hw, this corresponds to FAST_B2B_EN
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t b2bEn;
  /**
   * @brief [rw] Secondary Bus Reset.
   *
   * On rev 0 hw, this corresponds to SEC_BUS_RST
   * On rev 1 hw, this corresponds to SEC_BUS_RST
   * On rev 2 hw, this corresponds to SBR
   *
   * Field size: 1 bit
   */
  uint8_t secBusRst;
  /**
   * @brief [ro] Master Abort Mode.
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * On rev 0 hw, this corresponds to MST_ABORT_MODE
   * On rev 1 hw, this corresponds to MST_ABT_MODE
   * On rev 2 hw, this corresponds to MSTR_ABORT_MODE
   *
   * Field size: 1 bit
   */
  uint8_t mstAbortMode;
  /**
   * @brief [rw] VGA 16 bit Decode
   *
   * On rev 0 hw, this corresponds to VGA_DECODE
   * On rev 1 hw, this corresponds to VGA_16B_DEC
   * On rev 2 hw, this corresponds to VGA_16B_DEC
   *
   * Field size: 1 bit
   */
  uint8_t vgaDecode;
  /**
   * @brief [rw] VGA Enable
   *
   * On rev 0 hw, this corresponds to VGA_EN
   * On rev 1 hw, this corresponds to VGA_EN
   * On rev 2 hw, this corresponds to VGA_EN
   *
   * Field size: 1 bit
   */
  uint8_t vgaEn;
  /**
   * @brief [rw] ISA Enable
   *
   * On rev 0 hw, this corresponds to ISA_EN
   * On rev 1 hw, this corresponds to ISA_EN
   * On rev 2 hw, this corresponds to ISA_EN
   *
   * Field size: 1 bit
   */
  uint8_t isaEn;
  /**
   * @brief [rw] SERR Enable.
   *
   * Set to enable forwarding of ERR_COR, ERR_NONFATAL and ERR_FATAL messages.
   *
   * On rev 0 hw, this corresponds to SERR_EN
   * On rev 1 hw, this corresponds to SERR_EN
   * On rev 2 hw, this corresponds to SERR_EN
   *
   * Field size: 1 bit
   */
  uint8_t serrEn;
  /**
   * @brief [rw] Parity Error Response Enable.
   *
   * This bit controls the logging of poisoned TLPs in
   * @ref pcieType1SecStatReg_s::mstDPErr
   *
   * On rev 0 hw, this corresponds to PERR_RESP_EN
   * On rev 1 hw, this corresponds to PERR_RESP_EN
   * On rev 2 hw, this corresponds to PERE
   *
   * Field size: 1 bit
   */
  uint8_t pErrRespEn;
  /**
   * @brief [rw] Interrupt Pin.
   *
   * It identifies the legacy interrupt message that the device uses.
   * For single function configuration, the core only uses INTA. This register
   * is writable through internal bus interface.
   *
   *  0  = Legacy interrupt is not being used
   *  1h = INTA
   *  2h = INTB
   *  3h = INTC
   *  4h = INTD
   * Others = Reserved.
   *
   * On rev 0 hw, this corresponds to INT_PIN
   * On rev 1 hw, this corresponds to INT_PIN
   * On rev 2 hw, this corresponds to INT_PIN
   *
   * Field size: 8 bits
   */
  uint8_t intPin;
  /**
   * @brief [rw] Interrupt Line. Value is system software specified.
   *
   * On rev 0 hw, this corresponds to INT_LINE
   * On rev 1 hw, this corresponds to INT_LIN
   * On rev 2 hw, this corresponds to INT_LINE
   *
   * Field size: 8 bits
   */
  uint8_t intLine;
} pcieType1BridgeIntReg_t;
/* @} */

/*****************************************************************************
 **********  Power Management Capabilities REGISTERS  ************************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_pwr_structures
 * @brief Specification of the Power Management Capability Register
 *
 * On rev 0 hw, this corresponds to PMCAP  for EP and RC
 * On rev 1 hw, this corresponds to PM_CAP for EP only
 * On rev 2 hw, this corresponds to CAP_ID_NXT_PTR_REG for EP and RC
 *
 * @{
 */
typedef struct pciePMCapReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] PME Support.
   *
   * Identifies the power states from which generates PME Messages. A value of
   * 0 for any bit indicates that the device (or function) is not capable
   * of generating PME Messages while in that power state.
   *
   * bit 0x10: If set, PME Messages can be generated from D3cold.\n
   * bit 0x08: If set, PME Messages can be generated from D3hot.\n
   * bit 0x04: If set, PME Messages can be generated from D2.\n
   * bit 0x02: If set, PME Messages can be generated from D1.\n
   * bit 0x01: If set, PME Messages can be generated from D0.
   *
   * On rev 0 hw, this corresponds to PME_SUPP_N
   * On rev 1 hw, this corresponds to PME_SP
   * On rev 2 hw, this corresponds to PME_SUPPORT
   *
   * Field size: 5 bits
   *
   */
  uint8_t pmeSuppN;
  /**
   * @brief [rw] D2 Support.
   *
   * On rev 0 hw, this corresponds to D2_SUPP_N
   * On rev 1 hw, this corresponds to D2_SP
   * On rev 2 hw, this corresponds to D2_SUPPORT
   *
   * Field size: 1 bit
   *
   */
  uint8_t d2SuppN;
  /**
   * @brief [rw] D1 Support.
   *
   * On rev 0 hw, this corresponds to D1_SUPP_N
   * On rev 1 hw, this corresponds to D1_SP
   * On rev 2 hw, this corresponds to D1_SUPPORT
   *
   * Field size: 1 bit
   *
   */
  uint8_t d1SuppN;
  /**
   * @brief [rw] Auxiliary Current
   *
   * On rev 0 hw, this corresponds to AUX_CURR_N
   * On rev 1 hw, this corresponds to AUX_CUR
   * On rev 2 hw, this corresponds to AUX_CURR
   *
   * Field size: 3 bits
   *
   */
  uint8_t auxCurrN;
  /**
   * @brief [rw] Device Specific Initialization
   *
   * On rev 0 hw, this corresponds to DSI_N
   * On rev 1 hw, this corresponds to DSI
   * On rev 2 hw, this corresponds to DSI
   *
   * Field size: 1 bit
   *
   */
  uint8_t dsiN;
  /**
   * @brief [ro] PME clock.  Hardwired to zero.
   *
   * On rev 0 hw, this corresponds to PME_CLK
   * On rev 1 hw, this corresponds to PME_CLK
   * On rev 2 hw, this corresponds to PME_CLK
   *
   * Field size: 1 bit
   *
   */
  uint8_t pmeClk;
  /**
   * @brief [rw] Power Management Specification Version
   *
   * On rev 0 hw, this corresponds to PME_SPEC_VER
   * On rev 1 hw, this corresponds to PMC_VER
   * On rev 2 hw, this corresponds to PM_SPEC_VER
   *
   * Field size: 3 bits
   *
   */
  uint8_t pmeSpecVer;
  /**
   * @brief [rw] Next capability pointer.
   *
   * By default, it points to Message Signaled Interrupt structure.
   *
   * On rev 0 hw, this corresponds to PM_NEXT_PTR
   * On rev 1 hw, this corresponds to PM_NX_PTR
   * On rev 2 hw, this corresponds to PM_NEXT_POINTER
   *
   * Field size: 8 bits
   *
   */
  uint8_t pmNextPtr;
  /**
   * @brief [ro] Power Management Capability ID.
   *
   * On rev 0 hw, this corresponds to PM_CAP_ID
   * On rev 1 hw, this corresponds to CAP_ID
   * On rev 2 hw, this corresponds to PM_CAP_ID
   *
   * Field size: 8 bits
   *
   */
  uint8_t pmCapID;
} pciePMCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pwr_structures
 * @brief Specification of the Power Management Capabilities Control and Status Register
 *
 * On rev 0 hw, this corresponds to PM_CTL_STAT for EP and RC
 * On rev 1 hw, this corresponds to PM_CSR for EP only
 * On rev 2 hw, this corresponds to CON_STATUS_REG for EP and RC only
 *
 * @{
 */
typedef struct pciePMCapCtlStatReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Data register for additional information. Not supported.
   *
   * On rev 0 hw, this corresponds to DATA_REG
   * On rev 1 hw, this corresponds to DATA1
   * On rev 2 hw, this corresponds to DATA_REG_ADD_INFO
   *
   * Field size: 8 bits
   *
   */
  uint8_t dataReg;
  /**
   * @brief [ro] Bus Power/Clock Control Enable. Hardwired to zero.
   *
   * On rev 0 hw, this corresponds to CLK_CTRL_EN
   * On rev 1 hw, this corresponds to BP_CCE
   * On rev 2 hw, this corresponds to BUS_PWR_CLK_CON_EN
   *
   * Field size: 1 bit
   *
   */
  uint8_t clkCtrlEn;
  /**
   * @brief [ro] B2 and B3 support. Hardwired to zero.
   *
   * On rev 0 hw, this corresponds to B2_B3_SUPPORT
   * On rev 1 hw, this corresponds to B2B3_SP
   * On rev 2 hw, this corresponds to B2_B3_SUPPORT
   *
   * Field size: 1 bit
   *
   */
  uint8_t b2b3Support;
  /**
   * @brief [rw] PME Status. Indicates if a previously enabled PME event occurred or not.
   *
   * On rev 0 hw, this corresponds to PME_STATUS
   * On rev 1 hw, this corresponds to PME_STATUS
   * On rev 2 hw, this corresponds to PME_STATUS
   *
   * Write 1 to clear.
   *
   * Field size: 1 bit
   *
   */
  uint8_t pmeStatus;
  /**
   * @brief [ro] Data Scale. Not supported.
   *
   * On rev 0 hw, this corresponds to DATA_SCALE
   * On rev 1 hw, this corresponds to DATA_SCALE
   * On rev 2 hw, this corresponds to DATA_SCALE
   *
   * Field size: 2 bits
   *
   */
  uint8_t dataScale;
  /**
   * @brief [ro] Data Select. Not supported.
   *
   * On rev 0 hw, this corresponds to DATA_SELECT
   * On rev 1 hw, this corresponds to DATA_SEL
   * On rev 2 hw, this corresponds to DATA_SELECT
   *
   * Field size: 4 bits
   *
   */
  uint8_t dataSelect;
  /**
   * @brief [rw] PME Enable. Value of 1 indicates device is enabled to generate PME.
   *
   * On rev 0 hw, this corresponds to PME_EN
   * On rev 1 hw, this corresponds to PME_EN
   * On rev 2 hw, this corresponds to PME_ENABLE
   *
   * Field size: 1 bit
   *
   */
  uint8_t pmeEn;
  /**
   * @brief [rw] No Soft Reset.
   *
   * It is set to disable reset during a transition from D3 to D0.
   *
   * On rev 0 hw, this corresponds to NO_SOFT_RST
   * On rev 1 hw, this corresponds to NSR
   * On rev 2 hw, this corresponds to NO_SOFT_RST
   *
   * Field size: 1 bit
   *
   */
  uint8_t noSoftRst;
  /**
   * @brief [rw] Power State.
   *
   * Controls the device power state. Writes are ignored if the state is not
   * supported.
   * 0 = D0 power state
   * 1h = D1 power state
   * 2h = D2 power state
   * 3h = D3 power states
   *
   * On rev 0 hw, this corresponds to PWR_STATE
   * On rev 1 hw, this corresponds to PM_STATE
   * On rev 2 hw, this corresponds to POWER_STATE
   *
   * Field size: 2 bits
   *
   */
  uint8_t pwrState;
} pciePMCapCtlStatReg_t;
/* @} */

/*****************************************************************************
 **********  Message Signaling Interrupt  REGISTERS  *************************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI capabilities Register
 *
 * On rev 0 hw, this corresponds to MSI_CAP for EP and RC
 * On rev 1 hw, this corresponds to MSI_CAP for EP only
 * On rev 2 hw, this corresponds to PCI_MSI_CAP_ID_NEXT_CTRL_REG for EP only
 *
 * @{
 */
typedef struct pcieMsiCapReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI Per Vector Masking supported
   *
   * on rev 0 hw: unsupported
   * on rev 1 hw: unsupported
   * On rev 2 hw, this corresponds to PCI_MSI_EXT_DATA_EN
   *
   * Field size: 1 bit
   *
   */
  uint8_t extDataEn;
  /**
   * @brief [ro] Extended message data capable
   *
   * on rev 0 hw: unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PCI_MSI_EXT_DATA_CAP
   *
   * Field size: 1 bit
   *
   */
  uint8_t extDataCap;
  /**
   * @brief [ro] MSI Per Vector Masking supported
   *
   * on rev 0 hw: unsupported
   * On rev 1 hw, this corresponds to PVM_EN
   * On rev 2 hw, this corresponds to PCI_PVM_SUPPORT
   *
   * Field size: 1 bit
   *
   */
  uint8_t pvmEn;
  /**
   * @brief [rw] 64bit addressing enabled
   *
   * On rev 0 hw, this corresponds to 64BIT_EN
   * On rev 1 hw, this corresponds to MSI_64_EN
   * On rev 2 hw, this corresponds to PCI_MSI_64_BIT_ADDR_CAP
   *
   * Field size: 1 bit
   *
   */
  uint8_t en64bit;
  /**
   * @brief [rw] Multiple Msg enabled
   *
   * Indicates that multiple message mode is enabled by software. Number
   * of messages enabled must not be greater than @ref multMsgCap
   *
   * <TABLE>
   * <TR><TH>@ref multMsgEn</TH><TH>Number of messages</TH></TR>
   * <TR><TD>0</TD>             <TD>1</TD></TR>
   * <TR><TD>1</TD>             <TD>2</TD></TR>
   * <TR><TD>2</TD>             <TD>4</TD></TR>
   * <TR><TD>3</TD>             <TD>8</TD></TR>
   * <TR><TD>4</TD>             <TD>16</TD></TR>
   * <TR><TD>5</TD>             <TD>32</TD></TR>
   * <TR><TD>others</TD>        <TD>reserved</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to MULT_MSG_EN
   * On rev 1 hw, this corresponds to MME
   * On rev 2 hw, this corresponds to PCI_MSI_MULTIPLE_MSG_EN
   *
   * Field size: 3 bits
   */
  uint8_t multMsgEn;
  /**
   * @brief [rw] Multipe Msg capable
   *
   * Multiple message capable.
   *
   * <TABLE>
   * <TR><TH>@ref multMsgCap</TH><TH>Number of messages</TH></TR>
   * <TR><TD>0</TD>              <TD>1</TD></TR>
   * <TR><TD>1</TD>              <TD>2</TD></TR>
   * <TR><TD>2</TD>              <TD>4</TD></TR>
   * <TR><TD>3</TD>              <TD>8</TD></TR>
   * <TR><TD>4</TD>              <TD>16</TD></TR>
   * <TR><TD>5</TD>              <TD>32</TD></TR>
   * <TR><TD>others</TD>         <TD>reserved</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to MULT_MSG_CAP
   * On rev 1 hw, this corresponds to MMC
   * On rev 2 hw, this corresponds to PCI_MSI_MULTIPLE_MSG_CAP
   *
   * Field size: 3 bits
   */
  uint8_t multMsgCap;
  /**
   * @brief [rw] MSI enabled
   *
   * MSI Enabled. When set, INTx must be disabled.
   *
   * On rev 0 hw, this corresponds to MSI_EN
   * On rev 1 hw, this corresponds to MSI_EN
   * On rev 2 hw, this corresponds to PCI_MSI_ENABLE
   *
   * Field size: 1 bit
   */
  uint8_t msiEn;
  /**
   * @brief [rw] Next capability pointer
   *
   * By default, it points to PCI Express Capabilities structure.
   *
   * On rev 0 hw, this corresponds to NEXT_CAP
   * On rev 1 hw, this corresponds to MSI_NX_PTR
   * On rev 2 hw, this corresponds to PCI_MSI_CAP_NEXT_OFFSET
   *
   * Field size: 8 bits
   */
  uint8_t nextCap;
  /**
   * @brief [ro] MSI capability ID
   *
   * On rev 0 hw, this corresponds to CAP_ID
   * On rev 1 hw, this corresponds to CAP_ID
   * On rev 2 hw, this corresponds to PCI_MSI_CAP_ID
   *
   * Field size: 8 bits
   */
  uint8_t capId;
} pcieMsiCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI lower 32 bits Register
 *
 * On rev 0 hw, this corresponds to MSI_LOW32 for EP and RC
 * On rev 1 hw, this corresponds to MSI_ADD_L32 for EP only
 * On rev 2 hw, this corresponds to MSI_CAP_OFF_04H_REG for EP only
 *
 * @{
 */
typedef struct pcieMsiLo32Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Lower 32bits address
   *
   * On rev 0 hw, this corresponds to LOW32_ADDR
   * On rev 1 hw, this corresponds to ADDR
   * On rev 2 hw, this corresponds to PCI_MSI_CAP_OFF_04H
   *
   * Field size: 30 bits
   *
   */
  uint32_t addr;
} pcieMsiLo32Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI upper 32 bits Register
 *
 * On rev 0 hw, this corresponds to MSI_UP32 for EP and RC
 * On rev 1 hw, this corresponds to MSI_ADD_L32 for EP only
 * On rev 2 hw, this corresponds to MSI_CAP_OFF_08H_REG for EP only
 *
 * @{
 */
typedef struct pcieMsiUp32Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Upper 32bits address
   *
   * On rev 0 hw, this corresponds to UP32_ADDR
   * On rev 1 hw, this corresponds to ADDR
   * On rev 2 hw, this corresponds to PCI_MSI_CAP_OFF_08H
   *
   * Field size: 32 bits
   *
   */
  uint32_t addr;
} pcieMsiUp32Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI Data Register
 *
 * On rev 0 hw, this corresponds to MSI_DATA for EP and RC
 * On rev 1 hw, this corresponds to MSI_DATA for EP only
 * On rev 2 hw, this corresponds to MSI_CAP_OFF_0CH_REG for EP only
 *
 * @{
 */
typedef struct pcieMsiDataReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI data
   *
   * On rev 0 hw, this corresponds to MSI_DATA
   * On rev 1 hw, this corresponds to DATA
   * On rev 2 hw, this corresponds to PCI_MSI_CAP_OFF_0CH
   *
   * Field size: 16 bits
   *
   */
  uint16_t data;
} pcieMsiDataReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI_CAP_OFF_14H_REG Register
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to MSI_CAP_OFF_10H_REG for EP only
 *
 * This register is only available for rev 2 hw.
 *
 * @{
 */
typedef struct pcieMsiCapOff10H {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI data
   *
   * On rev 2 hw, this corresponds to PCI_MSI_CAP_OFF_10H
   *
   * Field size: 32 bits
   *
   */
  uint32_t data;
} pcieMsiCapOff10HReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI Data Register
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to MSI_CAP_OFF_14H_REG for EP only
 *
 * This register is only available for rev 2 hw.
 *
 * @{
 */
typedef struct pcieMsiCapOff14H {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI data
   *
   * On rev 2 hw, this corresponds to PCI_MSI_CAP_OFF_14H
   *
   * Field size: 32 bits
   *
   */
  uint32_t data;
} pcieMsiCapOff14HReg_t;
/* @} */


/*****************************************************************************
 **********  PCIe CAPABILITIES  REGISTERS ************************************
 ****************************************************************************/
/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the PCI Express Capabilities Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to PCIE_CAP
 * On rev 1 hw, this corresponds to PCIE_CAP
 * On rev 2 hw, this corresponds to PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG
 *
 * @{
 */
typedef struct pciePciesCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Interrupt Message Number. Updated by hardware and writable through internal bus Interface.
   *
   * On rev 0 hw, this corresponds to INT_MSG
   * On rev 1 hw, this corresponds to IM_NUM
   * On rev 2 hw, this corresponds to PCIE_INT_MSG_NUM
   *
   * Field size: 5 bits
   */
  uint8_t intMsg;
  /**
   * @brief [rw] Slot Implemented. Writable from internal bus interface.
   *
   * On rev 0 hw, this corresponds to SLT_IMPL_N
   * On rev 1 hw, this corresponds to SLOT
   * On rev 2 hw, this corresponds to PCIE_SLOT_IMP
   *
   * Field size: 1 bit
   */
  uint8_t sltImplN;
  /**
   * @brief [rw] Device Port Type.
   *
   * 0 = EP type\n
   * 4h = RC type\n
   * Others = Reserved
   *
   * On rev 0 hw, this corresponds to DPORT_TYPE
   * On rev 1 hw, this corresponds to DEV_TYPE
   * On rev 2 hw, this corresponds to PCIE_DEV_PORT_TYPE
   *
   * Field size: 4 bits
   */
  uint8_t dportType;
  /**
   * @brief [rw] PCI Express Capability Version
   *
   * On rev 0 hw, this corresponds to PCIE_CAP
   * On rev 1 hw, this corresponds to PCIE_VER
   * On rev 2 hw, this corresponds to PCIE_CAP_REG
   *
   * Field size: 4 bits
   */
  uint8_t pcieCap;
  /**
   * @brief [rw] Next capability pointer. Writable from internal bus interface.
   *
   * On rev 0 hw, this corresponds to NEXT_CAP
   * On rev 1 hw, this corresponds to PCIE_NX_PTR
   * On rev 2 hw, this corresponds to PCIE_CAP_NEXT_PTR
   *
   * Field size: 8 bits
   */
  uint8_t nextCap;
  /**
   * @brief [rw] PCIe Capability ID.
   *
   * On rev 0 hw, this corresponds to CAP_ID
   * On rev 1 hw, this corresponds to CAP_ID
   * On rev 2 hw, this corresponds to PCIE_CAP_ID
   *
   * Field size: 8 bits
   */
  uint8_t capId;
} pciePciesCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Device Capabilities Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to DEVICE_CAP
 * On rev 1 hw, this corresponds to DEV_CAP
 * On rev 2 hw, this corresponds to DEVICE_CAPABILITIES_REG
 *
 * @{
 */
typedef struct pcieDeviceCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Function Level Reset Capability
   *
   * used on EP only where it is rw.  On RC reserved and should be 0
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to FLR_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_FLR_CAP
   *
   * Field size: 1 bit
   */
  uint8_t flrEn;
  /**
   * @brief [rw] Captured Slot Power Limit Scale. For upstream ports (EP ports) only.
   *
   * On rev 0 hw, this corresponds to PWR_LIMIT_SCALE
   * On rev 1 hw, this corresponds to CAPT_SLOW_PWRLIMIT_SCALE
   * On rev 2 hw, this corresponds to PCIE_CAP_CAP_SLOT_PWR_LMT_SCALE
   *
   * Field size: 2 bits
   */
  uint8_t pwrLimitScale;
  /**
   * @brief [rw] Captured Slow Power Limit Value. For upstream ports (EP ports) only.
   *
   * On rev 0 hw, this corresponds to PWR_LIMIT_VALUE
   * On rev 1 hw, this corresponds to CAPT_SLOW_PWRLIMIT_VALUE
   * On rev 2 hw, this corresponds to PCIE_CAP_CAP_SLOT_PWR_LMT_VALUE
   *
   * Field size: 8 bits
   */
  uint8_t pwrLimitValue;
  /**
   * @brief [rw] Role-based Error Reporting. Writable from internal bus interface.
   *
   * On rev 0 hw, this corresponds to ERR_RPT
   * On rev 1 hw, this corresponds to ROLEBASED_ERRRPT
   * On rev 2 hw, this corresponds to PCIE_CAP_ROLE_BASED_ERR_REPORT
   *
   * Field size: 1 bit
   */
  uint8_t errRpt;
  /**
   * @brief [rw] Endpoint L1 Acceptable Latency. Must be 0 in RC mode. It is 3h for EP mode.
   *
   * On rev 0 hw, this corresponds to L1_LATENCY
   * On rev 1 hw, this corresponds to DEFAULT_EP_L1_LATENCY
   * On rev 2 hw, this corresponds to PCIE_CAP_EP_L1_ACCPT_LATENCY
   *
   * Field size: 3 bits
   */
  uint8_t l1Latency;
  /**
   * @brief [rw] Endpoint L0s Acceptable Latency. Must be 0 in RC mode. It is 4h for EP mode.
   *
   * On rev 0 hw, this corresponds to L0_LATENCY
   * On rev 1 hw, this corresponds to DEFAULT_EP_L0S_LATENCY
   * On rev 2 hw, this corresponds to PCIE_CAP_EP_L0S_ACCPT_LATENCY
   *
   * Field size: 3 bits
   */
  uint8_t l0Latency;
  /**
   * @brief [rw] Extended Tag Field Supported. Writable from internal interface
   *
   * On rev 0 hw, this corresponds to EXT_TAG_FLD
   * On rev 1 hw, this corresponds to EXTTAGFIELD_SUPPORT
   * On rev 2 hw, this corresponds to PCIE_CAP_EXT_TAG_SUPP
   *
   * Field size: 1 bit
   */
  uint8_t extTagFld;
  /**
   * @brief [rw] Phantom Field Supported. Writable from internal bus interface.
   *
   * On rev 0 hw, this corresponds to PHANTOM_FLD
   * On rev 1 hw, this corresponds to PHANTOMFUNC
   * On rev 2 hw, this corresponds to PCIE_CAP_PHANTOM_FUNC_SUPPORT
   *
   * Field size: 2 bits
   */
  uint8_t phantomFld;
  /**
   * @brief [rw] Maximum Payload size supported. Writable from internal bus interface.
   *
   * On rev 0 hw, this corresponds to MAX_PAYLD_SZ
   * On rev 1 hw, this corresponds to MAX_PAYLOAD_SIZE
   * On rev 2 hw, this corresponds to PCIE_CAP_MAX_PAYLOAD_SIZE
   *
   * Field size: 3 bits
   */
  uint8_t maxPayldSz;
} pcieDeviceCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Device Status and Control Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to DEV_STAT_CTRL
 * On rev 1 hw, this corresponds to DEV_CAS
 * On rev 2 hw, this corresponds to DEVICE_CONTROL_DEVICE_STATUS
 *
 * @{
 */
typedef struct pcieDevStatCtrlReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Transaction Pending
   *
   * On rev 0 hw, this corresponds to TPEND
   * On rev 1 hw, this corresponds to TRANS_PEND
   * On rev 2 hw, this corresponds to PCIE_CAP_TRANS_PENDING
   *
   * Field size: 1 bit
   */
  uint8_t tpend;
  /**
   * @brief [ro] Auxiliary Power Detected
   *
   * On rev 0 hw, this corresponds to AUX_PWR
   * On rev 1 hw, this corresponds to AUXP_DET
   * On rev 2 hw, this corresponds to PCIE_CAP_AUX_POWER_DETECTED
   *
   * Field size: 1 bit
   */
  uint8_t auxPwr;
  /**
   * @brief [rw] Unsupported Request Detected
   *
   * On rev 0 hw, this corresponds to UNSUP_RQ_DET
   * On rev 1 hw, this corresponds to UR_DET
   * On rev 2 hw, this corresponds to PCIE_CAP_UNSUPPORTED_REQ_DETECTED
   *
   * Field size: 1 bit
   */
  uint8_t rqDet;
  /**
   * @brief [rw] Fatal Error Detected
   *
   * On rev 0 hw, this corresponds to FATAL_ERR
   * On rev 1 hw, this corresponds to FT_DET
   * On rev 2 hw, this corresponds to PCIE_CAP_FATAL_ERR_DETECTED
   *
   * Field size: 1 bit
   */
  uint8_t fatalEr;
  /**
   * @brief [rw] Non-fatal Error Detected
   *
   * On rev 0 hw, this corresponds to NFATAL_ERR
   * On rev 1 hw, this corresponds to NFT_DET
   * On rev 2 hw, this corresponds to PCIE_CAP_NON_FATAL_ERR_DETECTED
   *
   * Field size: 1 bit
   */
  uint8_t nFatalEr;
  /**
   * @brief [rw] Correctable Error Detected
   *
   * On rev 0 hw, this corresponds to CORR_ERR
   * On rev 1 hw, this corresponds to COR_DET
   * On rev 2 hw, this corresponds to PCIE_CAP_CORR_ERR_DETECTED
   *
   * Field size: 1 bit
   */
  uint8_t corrEr;
  /**
   * @brief [rw] Initiate Function Level Reset (for EP)
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 1 hw, this corresponds to PCIE_CAP_INITIATE_FLR
   *
   * Field size: 3 bits
   */
  uint8_t initFLR;
  /**
   * @brief [rw] Maximum Read Request Size
   *
   * On rev 0 hw, this corresponds to MAX_REQ_SZ
   * On rev 1 hw, this corresponds to MRRS
   * On rev 1 hw, this corresponds to PCIE_CAP_MAX_READ_REQ_SIZE
   *
   * Field size: 3 bits
   */
  uint8_t maxSz;
  /**
   * @brief [rw] Enable no snoop
   *
   * On rev 0 hw, this corresponds to NO_SNOOP
   * On rev 1 hw, this corresponds to NOSNP_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_EN_NO_SNOOP
   *
   * Field size: 1 bit
   */
  uint8_t noSnoop;
  /**
   * @brief [rw] AUX Power PM Enable
   *
   * On rev 0 hw, this corresponds to AUX_PWR_PM_EN
   * On rev 1 hw, this corresponds to AUXM_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_AUX_POWER_PM_EN
   *
   * Field size: 1 bit
   */
  uint8_t auxPwrEn;
  /**
   * @brief [rw] Phantom Function Enable
   *
   * On rev 0 hw, this corresponds to PHANTOM_EN
   * On rev 1 hw, this corresponds to PHFUN_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_PHANTOM_FUNC_EN
   *
   * Field size: 1 bit
   */
  uint8_t phantomEn;
  /**
   * @brief [rw] Extended Tag Field Enable
   *
   * On rev 0 hw, this corresponds to XTAG_FIELD_EN
   * On rev 1 hw, this corresponds to EXTAG_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_EXT_TAG_EN
   *
   * Field size: 1 bit
   */
  uint8_t xtagEn;
  /**
   * @brief [rw] Maximum Payload Size
   *
   * On rev 0 hw, this corresponds to MAX_PAYLD
   * On rev 1 hw, this corresponds to MPS
   * On rev 2 hw, this corresponds to PCIE_CAP_MAX_PAYLOAD_SIZE_CS
   *
   * Field size: 3 bits
   */
  uint8_t maxPayld;
  /**
   * @brief [rw] Enable Relaxed Ordering
   *
   * On rev 0 hw, this corresponds to RELAXED
   * On rev 1 hw, this corresponds to EN_RO
   * On rev 2 hw, this corresponds to PCIE_CAP_EN_REL_ORDER
   *
   * Field size: 1 bit
   */
  uint8_t relaxed;
  /**
   * @brief [rw] Enable Unsupported Request Reporting
   *
   * On rev 0 hw, this corresponds to UNSUP_REQ_RP
   * On rev 1 hw, this corresponds to UR_RE
   * On rev 2 hw, this corresponds to PCIE_CAP_UNSUPPORT_REQ_REP_EN
   *
   * Field size: 1 bit
   */
  uint8_t reqRp;
  /**
   * @brief [rw] Fatal Error Reporting Enable
   *
   * On rev 0 hw, this corresponds to FATAL_ERR_RP
   * On rev 1 hw, this corresponds to FT_RE
   * On rev 2 hw, this corresponds to PCIE_CAP_FATAL_ERR_REPORT_E
   *
   * Field size: 1 bit
   */
  uint8_t fatalErRp;
  /**
   * @brief [rw] Non-fatal Error Reporting Enable
   *
   * On rev 0 hw, this corresponds to NFATAL_ERR_RP
   * On rev 1 hw, this corresponds to NFT_RE
   * On rev 2 hw, this corresponds to PCIE_CAP_NON_FATAL_ERR_REPORT_EN
   *
   * Field size: 1 bit
   */
  uint8_t nFatalErRp;
  /**
   * @brief [rw] Correctable Error Reporting Enable
   *
   * On rev 0 hw, this corresponds to CORR_ERR_RP
   * On rev 1 hw, this corresponds to COR_RE
   * On rev 2 hw, this corresponds to PCIE_CAP_CORR_ERR_REPORT_EN
   *
   * Field size: 1 bit
   */
  uint8_t corErRp;
} pcieDevStatCtrlReg_t;
/* @} */

/**
 *  @ingroup pcielld_reg_cfg_cap_structures
 *  @brief Specification of the Link Capabilities Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to LINK_CAP
 * On rev 1 hw, this corresponds to LNK_CAP
 * On rev 2 hw, this corresponds to LINK_CAPABILITIES_REG
 *
 * @{
 */
typedef struct pcieLinkCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Port Number. Writable from internal bus interface.
   *
   * On rev 0 hw, this corresponds to PORT_NUM
   * On rev 1 hw, this corresponds to PORT_NUM
   * On rev 2 hw, this corresponds to PCIE_CAP_PORT_NUM
   *
   * Field size: 8 bits
   */
  uint8_t portNum;
  /**
   * @brief [rw] ASPM Optionality Compliance
   *
   * On rev 0 hw, unsuppported
   * On rev 1 hw, this corresponds to ASPM_OPT_COMP
   * On rev 2 hw, this corresponds to PCIE_CAP_ASPM_OPT_COMPLIANCE
   *
   * Field size: 1 bit
   */
  uint8_t aspmOptComp;
  /**
   * @brief [rw] Bandwidth Notification Capable.
   *
   * 0 = For upstream ports (EP ports)\n
   * 1 = For downstream ports (RC ports)
   *
   * On rev 0 hw, this corresponds to BW_NOTIFY_CAP
   * On rev 1 hw, this corresponds to LNK_BW_NOT_CAP
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_BW_NOT_CAP
   *
   * Field size: 1 bit
   */
  uint8_t bwNotifyCap;
  /**
   * @brief [rw] Link Layer Active Reporting Capable.
   *
   * 0 = For upstream ports (EP ports)\n
   * 1 = For downstream ports (RC ports)
   *
   * On rev 0 hw, this corresponds to DLL_REP_CAP
   * On rev 1 hw, this corresponds to DLL_ACTRPT_CAP
   * On rev 2 hw, this corresponds to PCIE_CAP_DLL_ACTIVE_REP_CAP
   *
   * Field size: 1 bit
   */
  uint8_t dllRepCap;
  /**
   * @brief [rw] Surprise Down Error Reporting Capable. Not supported. Always zero.
   *
   * On rev 0 hw, this corresponds to DOWN_ERR_REP_CAP
   * On rev 1 hw, this corresponds to UNSUP
   * On rev 2 hw, this corresponds to PCIE_CAP_SURPRISE_DOWN_ERR_REP_CAP
   *
   * Field size: 1 bit
   */
  uint8_t downErrRepCap;
  /**
   * @brief [rw] Clock Power Management. Writable from internal bus interface.
   *
   * For upstream ports (EP Ports), a value of 1h in this bit indicates that
   * the component tolerates the removal of any reference clock(s) in the L1
   * and L2/L3 Ready Link states. A value of 0 indicates the reference
   * clock(s) must not be removed in these Link states.
   *
   * For downstream ports (RC Ports), this bit is always 0.
   *
   * On rev 0 hw, this corresponds to CLK_PWR_MGMT
   * On rev 1 hw, this corresponds to CLK_PWR_MGMT
   * On rev 2 hw, this corresponds to PCIE_CAP_CLOCK_POWER_MAN
   *
   * Field size: 1 bit
   */
  uint8_t clkPwrMgmt;
  /**
   * @brief [rw] L1 Exit Latency when common clock is used. Writable from internal bus interface.
   *
   * <TABLE>
   * <TR><TH>@ref l1ExitLat</TH><TH>low range</TH><TH>high range</TH></TR>
   * <TR><TD>0</TD>             <TD>0</TD>        <TD>64 ns</TD></TR>
   * <TR><TD>1</TD>             <TD>64ns</TD>     <TD>128ns</TD></TR>
   * <TR><TD>2</TD>             <TD>128ns</TD>    <TD>256ns</TD></TR>
   * <TR><TD>3</TD>             <TD>256ns</TD>    <TD>512ns</TD></TR>
   * <TR><TD>4</TD>             <TD>512ns</TD>    <TD>1�s</TD></TR>
   * <TR><TD>5</TD>             <TD>1�s</TD>      <TD>2�s</TD></TR>
   * <TR><TD>6</TD>             <TD>2�s</TD>      <TD>4�s</TD></TR>
   * <TR><TD>7</TD>             <TD>4�s</TD>      <TD>and up</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to L1_EXIT_LAT
   * On rev 1 hw, this corresponds to L1_EXIT_LAT
   * On rev 2 hw, this corresponds to PCIE_CAP_L1_EXIT_LATENCY
   *
   * Field size: 3 bits
   */
  uint8_t l1ExitLat;
  /**
   * @brief [rw] L0s Exit Latency. Writable from internal bus interface.
   *
   * <TABLE>
   * <TR><TH>@ref l1ExitLat</TH><TH>low range</TH><TH>high range</TH></TR>
   * <TR><TD>0</TD>             <TD>0</TD>        <TD>64 ns</TD></TR>
   * <TR><TD>1</TD>             <TD>64ns</TD>     <TD>128ns</TD></TR>
   * <TR><TD>2</TD>             <TD>128ns</TD>    <TD>256ns</TD></TR>
   * <TR><TD>3</TD>             <TD>256ns</TD>    <TD>512ns</TD></TR>
   * <TR><TD>4</TD>             <TD>512ns</TD>    <TD>1�s</TD></TR>
   * <TR><TD>5</TD>             <TD>1�s</TD>      <TD>2�s</TD></TR>
   * <TR><TD>6</TD>             <TD>2�s</TD>      <TD>4�s</TD></TR>
   * <TR><TD>7</TD>             <TD>4�s</TD>      <TD>and up</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to L0S_EXIT_LAT
   * On rev 1 hw, this corresponds to L0S_EXIT_LAT
   * On rev 2 hw, this corresponds to PCIE_CAP_L0S_EXIT_LATENCY
   *
   * Field size: 3 bits
   */
  uint8_t losExitLat;
  /**
   * @brief [rw] Active State Link Power Management Support. Writable from internal bus interface.
   *
   * 1h = L0s entry supported.\n
   * 3h = L0s and L1 supported.\n
   * Others = Reserved.
   *
   * On rev 0 hw, this corresponds to AS_LINK_PM
   * On rev 1 hw, this corresponds to AS_LINK_PM_SUPPORT
   * On rev 2 hw, this corresponds to PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT
   *
   * Field size: 2 bits
   */
  uint8_t asLinkPm;
  /**
   * @brief [rw] Maximum Link Width. Writable from internal bus interface.
   *
   * 1h = �1\n
   * 2h = �2\n
   * Others = Reserved.
   *
   * On rev 0 hw, this corresponds to MAX_LINK_WIDTH
   * On rev 1 hw, this corresponds to MAX_LINK_WIDTH
   * On rev 2 hw, this corresponds to PCIE_CAP_MAX_LINK_WIDTH
   *
   * Field size: 6 bits
   */
  uint8_t maxLinkWidth;
  /**
   * @brief [rw] Maximum Link Speed. Writable from internal bus interface.
   *
   * 1h = 2.5GT/s Link speed supported.\n
   * 2h = 5.0 GT/s and 2.5 GT/s Link speeds supported.\n
   * Others = Reserved.
   *
   * On rev 0 hw, this corresponds to MAX_LINK_SPEED
   * On rev 1 hw, this corresponds to MAX_LINK_SPEEDS
   * On rev 2 hw, this corresponds to PCIE_CAP_MAX_LINK_SPEED
   *
   * Field size: 4 bits
   */
  uint8_t maxLinkSpeed;
} pcieLinkCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Link Status and Control Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to LINK_STAT_CTRL
 * On rev 1 hw, this corresponds to LNK_CAS
 * On rev 2 hw, this corresponds to LINK_CONTROL_LINK_STATUS_REG
 *
 * @{
 */
typedef struct pcieLinkStatCtrlReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Link Autonomous Bandwidth Status.
   *
   * This bit is Set by hardware to indicate that hardware has autonomously
   * changed Link speed or width, without the Port transitioning through
   * DL_Down status, for reasons other than to attempt to correct unreliable
   * Link operation.  This bit must be set if the Physical Layer reports a
   * speed or width change was initiated by the downstream component that
   * was indicated as an autonomous change.
   *
   * Not applicable and reserved for EP.
   *
   * On rev 0 hw, this corresponds to LINK_BW_STATUS
   * On rev 1 hw, this corresponds to LAB_STATUS
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_AUTO_BW_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t linkBwStatus;
  /**
   * @brief [rw] Link Bandwidth Management Status.
   *
   * This bit is Set by hardware to indicate that either of the following
   * has occurred without the Port transitioning through DL_Down status:
   *
   * - A Link retraining has completed following a write of 1b to the
   *   Retrain Link bit
   * - Hardware has changed Link speed or width to attempt to correct
   *   unreliable Link operation, either through an LTSSM timeout or
   *   a higher level process.
   *
   * This bit must be set if the Physical Layer reports a speed or width
   * change was initiated by the downstream component that was not
   * indicated as an autonomous change.
   *
   * Not applicable and reserved for EP.
   *
   * On rev 0 hw, this corresponds to LINK_BW_MGMT_STATUS
   * On rev 1 hw, this corresponds to LBW_STATUS
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_BW_MAN_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t linkBwMgmtStatus;
  /**
   * @brief [rw] Data Link Layer Active
   *
   * This bit indicates the status of the Data Link Control and
   * Management State Machine. It returns a 1 to indicate the DL_Active state,
   * 0 otherwise.
   *
   * On rev 0 hw, this corresponds to DLL_ACTIVE
   * On rev 1 hw, this corresponds to DLL_ACT
   * On rev 2 hw, this corresponds to PCIE_CAP_DLL_ACTIVE
   *
   * Field size: 1 bit
   */
  uint8_t dllActive;
  /**
   * @brief [rw] Slot Clock Configuration. Writable from internal bus interface.
   *
   * This bit indicates that the component uses the same
   * physical reference clock that the platform provides on the connector.
   *
   * On rev 0 hw, this corresponds to SLOT_CLK_CFG
   * On rev 1 hw, this corresponds to SLOT_CLK_CONFIG
   * On rev 2 hw, this corresponds to PCIE_CAP_SLOT_CLK_CONFIG
   *
   * Field size: 1 bit
   */
  uint8_t slotClkCfg;
  /**
   * @brief [rw] Link Training. Not applicable to EP.
   *
   * On rev 0 hw, this corresponds to LINK_TRAINING
   * On rev 1 hw, this corresponds to LINK_TRAIN
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_TRAINING
   *
   * Field size: 1 bit
   */
  uint8_t linkTraining;
  /**
   * @brief [rw] Undefined for PCI Express.
   *
   * Field size: 1 bit
   */
  uint8_t undef;
  /**
   * @brief [rw] Negotiated Link Width. Set automatically by hardware after link initialization.
   *
   * On rev 0 hw, this corresponds to NEGOTIATED_LINK_WD
   * On rev 1 hw, this corresponds to NEG_LW
   * On rev 2 hw, this corresponds to PCIE_CAP_NEGO_LINK_WIDTH
   *
   * Field size: 6 bits
   */
  uint8_t negotiatedLinkWd;
  /**
   * @brief [rw] Link Speed. Set automatically by hardware after link initialization.
   *
   * On rev 0 hw, this corresponds to LINK_SPEED
   * On rev 1 hw, this corresponds to LINK_SPEED
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_SPEED
   *
   * Field size: 4 bits
   */
  uint8_t linkSpeed;
  /**
   * @brief [rw]  DRS Signalling Control
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PCIE_CAP_DRS_SIGNALING_CONTROL
   *
   * Field size: 1 bit
   */
  uint8_t drsSigCtrl;
  /**
   * @brief [rw]  Link Autonomous Bandwidth Interrupt Enable. Not applicable and is reserved for EP
   *
   * On rev 0 hw, this corresponds to LINK_BW_INT_EN
   * On rev 1 hw, this corresponds to LABIE
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_AUTO_BW_INT_EN
   *
   * Field size: 1 bit
   */
  uint8_t linkBwIntEn;
  /**
   * @brief [rw] Link Bandwidth Management Interrupt Enable. Not applicable and is reserved for EP.
   *
   * On rev 0 hw, this corresponds to LINK_BW_MGMT_INT_EN
   * On rev 1 hw, this corresponds to LBMIE
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_BW_MAN_INT_EN
   *
   * Field size: 1 bit
   */
  uint8_t linkBwMgmtIntEn;
  /**
   * @brief [rw] Hardware Autonomous Width Disable. Not supported and hardwired to zero.
   *
   * On rev 0 hw, this corresponds to HW_AUTO_WIDTH_DIS
   * On rev 1 hw, this corresponds to HAWD
   * On rev 2 hw, this corresponds to PCIE_CAP_HW_AUTO_WIDTH_DISABLE
   *
   * Field size: 1 bit
   */
  uint8_t hwAutoWidthDis;
  /**
   * @brief [rw] Enable Clock Power Management.
   *
   * On rev 0 hw, this corresponds to CLK_PWR_MGMT_EN
   * On rev 1 hw, this corresponds to EN_CPM
   * On rev 2 hw, this corresponds to PCIE_CAP_EN_CLK_POWER_MAN
   *
   * Field size: 1 bit
   */
  uint8_t clkPwrMgmtEn;
  /**
   * @brief [rw] Extended Synchronization.
   *
   * On rev 0 hw, this corresponds to EXT_SYNC
   * On rev 1 hw, this corresponds to EXT_SYN
   * On rev 2 hw, this corresponds to PCIE_CAP_EXTENDED_SYNCH
   *
   * Field size: 1 bit
   */
  uint8_t extSync;
  /**
   * @brief [rw] Common Clock Configuration.
   *
   * 0 = Indicates that this device and the device at the opposite end of the
   * link are operating with separate reference clock sources.\n
   * 1 = Indicates that this device and the device at the opposite end of the
   * link are operating with a common clock source.
   *
   * On rev 0 hw, this corresponds to COMMON_CLK_CFG
   * On rev 1 hw, this corresponds to COM_CLK_CFG
   * On rev 2 hw, this corresponds to PCIE_CAP_COMMON_CLK_CONFIG
   *
   * Field size: 1 bit
   */
  uint8_t commonClkCfg;
  /**
   * @brief [rw] Retrain Link. Not applicable and reserved for EP.
   *
   * On rev 0 hw, this corresponds to RETRAIN_LINK
   * On rev 1 hw, this corresponds to RETRAIN_LINK
   * On rev 2 hw, this corresponds to PCIE_CAP_RETRAIN_LINK
   *
   * Field size: 1 bit
   */
  uint8_t retrainLink;
  /**
   * @brief [rw] Disables the link by directing the LTSSM to the Disabled state when set.
   *
   * On rev 0 hw, this corresponds to LINK_DISABLE
   * On rev 1 hw, this corresponds to LINK_DIS
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_DISABLE
   *
   * Field size: 1 bit
   */
  uint8_t linkDisable;
  /**
   * @brief [rw] Read Completion Boundary.
   *
   * 0 = 64 bytes\n
   * 1 = 128 bytes
   *
   * On rev 0 hw, this corresponds to RCB
   * On rev 1 hw, this corresponds to RCB
   * On rev 2 hw, this corresponds to PCIE_CAP_RCB
   *
   * Field size: 1 bit
   */
  uint8_t rcb;
  /**
   * @brief [rw] Active State Link Power Management Control
   *
   * 0 = Disabled.\n
   * 1h = L0s entry enabled.\n
   * 2h = L1 entry enabled.\n
   * 3h = L0s and L1 entry enabled.\n
   *
   * On rev 0 hw, this corresponds to ACTIVE_LINK_PM
   * On rev 1 hw, this corresponds to ASPM_CTRL
   * On rev 2 hw, this corresponds to PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL
   *
   * Field size: 2 bits
   */
  uint8_t activeLinkPm;
} pcieLinkStatCtrlReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Slot Capabilities register
 *
 * This register may only be used for root complex mode.
 *
 * On rev 0 hw, this corresponds to SLOT_CAP
 * On rev 1 hw, this corresponds to SLOT_CAP
 * On rev 2 hw, this corresponds to SLOT_CAPABILITIES_REG
 *
 * @{
 */
typedef struct pcieSlotCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Physical Slot Number.
   *
   * On rev 0 hw, this corresponds to SLOT_NUM
   * On rev 1 hw, this corresponds to PSN
   * On rev 2 hw, this corresponds to PCIE_CAP_PHY_SLOT_NUM
   *
   * Field size: 13 bits [0-0x1FFF]
   */
  uint16_t slotNum;
  /**
   * @brief [rw] No Command Complete Support
   *
   * When Set, this bit indicates that this slot does not generate software
   * notification when an issued command is completed by the Hot-Plug Controller
   *
   * On rev 0 hw, this corresponds to CMD_COMP_SUPP
   * On rev 1 hw, this corresponds to NCCS
   * On rev 2 hw, this corresponds to PCIE_CAP_NO_CMD_CPL_SUPPORT
   *
   * Field size: 1 bit
   */
  uint8_t cmdCompSupp;
  /**
   * @brief [rw] Electromechanical Interlock Present.
   *
   * When Set, this bit indicates that an Electromechanical Interlock
   * is implemented on the chassis for this slot.
   *
   * On rev 0 hw, this corresponds to EML_PRESENT
   * On rev 1 hw, this corresponds to EIP
   * On rev 2 hw, this corresponds to PCIE_CAP_ELECTROMECH_INTERLOCK
   *
   * Field size: 1 bit
   */
  uint8_t emlPresent;
  /**
   * @brief [rw] Slot Power Limit Scale.
   *
   * On rev 0 hw, this corresponds to PWR_LMT_SCALE
   * On rev 1 hw, this corresponds to SPLS
   * On rev 2 hw, this corresponds to PCIE_CAP_SLOT_POWER_LIMIT_SCALE
   *
   * Field size: 2 bits
   */
  uint8_t pwrLmtScale;
  /**
   * @brief [rw] Slot Power Limit Value.
   *
   * On rev 0 hw, this corresponds to PWR_LMT_VALUE
   * On rev 1 hw, this corresponds to SPLV
   * On rev 2 hw, this corresponds to PCIE_CAP_SLOT_POWER_LIMIT_VALUE
   *
   * Field size: 8 bits
   */
  uint8_t pwrLmtValue;
  /**
   * @brief [rw] Hot Plug Capable.
   *
   * On rev 0 hw, this corresponds to HP_CAP
   * On rev 1 hw, this corresponds to HPC
   * On rev 2 hw, this corresponds to PCIE_CAP_HOT_PLUG_CAPABLE
   *
   * Field size: 1 bit
   */
  uint8_t hpCap;
  /**
   * @brief [rw] Hot Plug Surprise.
   *
   * On rev 0 hw, this corresponds to HP_SURPRISE
   * On rev 1 hw, this corresponds to HPS
   * On rev 2 hw, this corresponds to PCIE_CAP_HOT_PLUG_SURPRISE
   *
   * Field size: 1 bit
   */
  uint8_t hpSurprise;
  /**
   * @brief [rw] Power Indicator Present.
   *
   * On rev 0 hw, this corresponds to PWR_IND
   * On rev 1 hw, this corresponds to PIP
   * On rev 2 hw, this corresponds to PCIE_CAP_POWER_INDICATOR
   *
   * Field size: 1 bit
   */
  uint8_t pwrInd;
  /**
   * @brief [rw] Attention Indicator Present.
   *
   * On rev 0 hw, this corresponds to ATTN_IND
   * On rev 1 hw, this corresponds to AIP
   * On rev 2 hw, this corresponds to PCIE_CAP_ATTENTION_INDICATOR
   *
   * Field size: 1 bit
   */
  uint8_t attnInd;
  /**
   * @brief [rw] MRL Sensor Present.
   *
   * On rev 0 hw, this corresponds to MRL_SENSOR
   * On rev 1 hw, this corresponds to MRLSP
   * On rev 2 hw, this corresponds to PCIE_CAP_MRL_SENSOR
   *
   * Field size: 1 bit
   */
  uint8_t mrlSensor;
  /**
   * @brief [rw] Power Controller Present.
   *
   * If there is no power controller, software must ensure that system power
   * is up before reading Presence Detect state
   *
   * On rev 0 hw, this corresponds to PWR_CTL
   * On rev 1 hw, this corresponds to PCP
   * On rev 2 hw, this corresponds to PCIE_CAP_POWER_CONTROLLER
   *
   * Field size: 1 bit
   */
  uint8_t pwrCtl;
  /**
   * @brief [rw] Attention Indicator Present.
   *
   * On rev 0 hw, this corresponds to ATTN_BUTTON
   * On rev 1 hw, this corresponds to ABP
   * On rev 2 hw, this corresponds to PCIE_CAP_ATTENTION_INDICATOR_BUTTON
   *
   * Field size: 1 bit
   */
  uint8_t attnButton;
} pcieSlotCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Slot Status and Control register
 *
 * This register may only be used for root complex mode.
 *
 * On rev 0 hw, this corresponds to SLOT_STAT_CTRL
 * On rev 1 hw, this corresponds to SLOT_CAS
 * On rev 2 hw, this corresponds to SLOT_CONTROL_SLOT_STATUS
 *
 * @{
 */
typedef struct pcieSlotStatCtrlReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Data Link Layer State Changed
   *
   * Write 1 to clear.
   *
   * On rev 0 hw, this corresponds to DLL_STATE
   * On rev 1 hw, this corresponds to DSC
   * On rev 2 hw, this corresponds to PCIE_CAP_DLL_STATE_CHANGED
   *
   * Field size: 1 bit
   */
  uint8_t dllState;
  /**
   * @brief [ro] Electromechanical Lock Status
   *
   * On rev 0 hw, this corresponds to EM_LOCK
   * On rev 1 hw, this corresponds to EIS
   * On rev 2 hw, this corresponds to PCIE_CAP_ELECTROMECH_INTERLOCK_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t emLock;
  /**
   * @brief [ro] Presence Detect State
   *
   * On rev 0 hw, this corresponds to PRESENCE_DET
   * On rev 1 hw, this corresponds to PDS
   * On rev 2 hw, this corresponds to PCIE_CAP_PRESENCE_DETECT_STATE
   *
   * Field size: 1 bit
   */
  uint8_t presenceDet;
  /**
   * @brief [ro] MRL Sensor State
   *
   * On rev 0 hw, this corresponds to MRL_STATE
   * On rev 1 hw, this corresponds to MRLSS
   * On rev 2 hw, this corresponds to PCIE_CAP_MRL_SENSOR_STATE
   *
   * Field size: 1 bit
   */
  uint8_t mrlState;
  /**
   * @brief [rw] Command Completed
   *
   * Write 1 to clear.
   *
   * On rev 0 hw, this corresponds to CMD_COMPLETE
   * On rev 1 hw, this corresponds to CC
   * On rev 2 hw, this corresponds to PCIE_CAP_CMD_CPLD
   *
   * Field size: 1 bit
   */
  uint8_t cmdComplete;
  /**
   * @brief [rw] Presence Detect Changed
   *
   * Write 1 to clear.
   *
   * On rev 0 hw, this corresponds to PRESENCE_CHG
   * On rev 1 hw, this corresponds to PDC
   * On rev 2 hw, this corresponds to PCIE_CAP_PRESENCE_DETECTED_CHANGED
   *
   * Field size: 1 bit
   */
  uint8_t presenceChg;
  /**
   * @brief [rw] MRL Sensor Changed
   *
   * Write 1 to clear.
   *
   * On rev 0 hw, this corresponds to MRL_CHANGE
   * On rev 1 hw, this corresponds to MRCSC
   * On rev 2 hw, this corresponds to PCIE_CAP_MRL_SENSOR_CHANGED
   *
   * Field size: 1 bit
   */
  uint8_t mrlChange;
  /**
   * @brief [rw] Power Fault Detected
   *
   * Write 1 to clear.
   *
   * On rev 0 hw, this corresponds to PWR_FAULT
   * On rev 1 hw, this corresponds to PFD
   * On rev 2 hw, this corresponds to PCIE_CAP_POWER_FAULT_DETECTED
   *
   * Field size: 1 bit
   */
  uint8_t pwrFault;
  /**
   * @brief [rw] Attention Button Pressed.
   *
   * Write 1 to clear.
   *
   * On rev 0 hw, this corresponds to ATTN_PRESSED
   * On rev 1 hw, this corresponds to ABP
   * On rev 2 hw, this corresponds to PCIE_CAP_ATTENTION_BUTTON_PRESSED
   *
   * Field size: 1 bit
   */
  uint8_t attnPressed;
  /**
   * @brief [rw] Data Link Layer State Changed Enable.
   *
   * On rev 0 hw, this corresponds to DLL_CHG_EN
   * On rev 1 hw, this corresponds to DSC_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_DLL_STATE_CHANGED_EN
   *
   * Field size: 1 bit
   */
  uint8_t dllChgEn;
  /**
   * @brief [rw] Electromechanical Interlock Control.
   *
   * On rev 0 hw, this corresponds to EM_LOCK_CTL
   * On rev 1 hw, this corresponds to EIC
   * On rev 2 hw, this corresponds to PCIE_CAP_ELECTROMECH_INTERLOCK_CTRL
   *
   * Field size: 1 bit
   */
  uint8_t emLockCtl;
  /**
   * @brief [rw] Power Controller Control
   *
   * On rev 0 hw, this corresponds to PM_CTL
   * On rev 1 hw, this corresponds to PCC
   * On rev 2 hw, this corresponds to PCIE_CAP_POWER_INDICATOR_CTRL
   *
   * Field size: 1 bit
   */
  uint8_t pmCtl;
  /**
   * @brief [rw] Power Indicator Control
   *
   * On rev 0 hw, this corresponds to PM_IND_CTL
   * On rev 1 hw, this corresponds to PIC
   * On rev 2 hw, this corresponds to PCIE_CAP_POWER_INDICATOR_CTRL
   *
   * Field size: 2 bits
   */
  uint8_t pmIndCtl;
  /**
   * @brief [rw] Attention Indicator Control.
   *
   * On rev 0 hw, this corresponds to ATTN_IND_CTL
   * On rev 1 hw, this corresponds to AIC
   * On rev 2 hw, this corresponds to PCIE_CAP_ATTENTION_INDICATOR_CTR
   *
   * Field size: 2 bits
   */
  uint8_t attnIndCtl;
  /**
   * @brief [rw] Hot Plug Interrupt Enable.
   *
   * On rev 0 hw, this corresponds to HP_INT_EN
   * On rev 1 hw, this corresponds to HPI_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_HOT_PLUG_INT
   *
   * Field size: 1 bit
   */
  uint8_t hpIntEn;
  /**
   * @brief [rw] Command Completed Interrupt Enable.
   *
   * On rev 0 hw, this corresponds to CMD_CMP_INT_EN
   * On rev 1 hw, this corresponds to CCI_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_CMD_CPL_INT_EN
   *
   * Field size: 1 bit
   */
  uint8_t cmdCmpIntEn;
  /**
   * @brief [rw] Presence Detect Changed Enable.
   *
   * On rev 0 hw, this corresponds to PRS_DET_CHG_EN
   * On rev 1 hw, this corresponds to PDC_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_PRESENCE_DETECT_CHANGE_EN
   *
   * Field size: 1 bit
   */
  uint8_t prsDetChgEn;
  /**
   * @brief [rw] MRL Sensor Changed Enable.
   *
   * On rev 0 hw, this corresponds to MRL_CHG_EN
   * On rev 1 hw, this corresponds to MRLSC_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_MRL_SENSOR_CHANGED_EN
   *
   * Field size: 1 bit
   */
  uint8_t mrlChgEn;
  /**
   * @brief [rw] Power Fault Detected Enable.
   *
   * On rev 0 hw, this corresponds to PWR_FLT_DET_EN
   * On rev 1 hw, this corresponds to PFD_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_POWER_FAULT_DETECTED_EN
   *
   * Field size: 1 bit
   */
  uint8_t pwrFltDetEn;
  /**
   * @brief [rw] Attention Button Pressed Enable.
   *
   * On rev 0 hw, this corresponds to ATTN_BUTT_EN
   * On rev 1 hw, this corresponds to ABP_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_ATTENTION_BUTTON_PRESSED_EN
   *
   * Field size: 1 bit
   */
  uint8_t attnButtEn;
} pcieSlotStatCtrlReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Root Control and Capabilities Register
 *
 * This register may only be used for root complex mode.
 *
 * On rev 0 hw, this corresponds to ROOT_CTRL_CAP
 * On rev 1 hw, this corresponds to ROOT_CAC
 * On rev 2 hw, this corresponds to ROOT_CONTROL_ROOT_CAPABILITIES_REG
 *
 * @{
 */
typedef struct pcieRootCtrlCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] CRS Software Visibility. Not supported and set to 0.
   *
   * On rev 0 hw, this corresponds to CRS_SW
   * On rev 1 hw, this corresponds to CRSSV
   * On rev 2 hw, this corresponds to PCIE_CAP_CRS_SW_VISIBILITY
   *
   * Field size: 1 bit
   */
  uint8_t crsSw;
  /**
   * @brief [ro] CRS Software Visibility Enable. Not supported and set to 0x0.
   *
   * On rev 0 hw, this corresponds to CRS_SW_EN
   * On rev 1 hw, this corresponds to CRSSV_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_CRS_SW_VISIBILITY
   *
   * Field size: 1 bit
   */
  uint8_t crsSwEn;
  /**
   * @brief [rw] PME Interrupt Enable
   *
   * On rev 0 hw, this corresponds to PME_INT_EN
   * On rev 1 hw, this corresponds to PMEI_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_PME_INT_EN
   *
   * Field size: 1 bit
   */
  uint8_t pmeIntEn;
  /**
   * @brief [rw] System Error on Fatal Error Enable
   *
   * On rev 0 hw, this corresponds to SERR_FATAL_ERR
   * On rev 1 hw, this corresponds to SEFE_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN
   *
   * Field size: 1 bit
   */
  uint8_t serrFatalErr;
  /**
   * @brief [rw] System Error on Non-fatal Error Enable
   *
   * On rev 0 hw, this corresponds to SERR_NFATAL_ERR
   * On rev 1 hw, this corresponds to SENE_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN
   *
   * Field size: 1 bit
   */
  uint8_t serrNFatalErr;
  /**
   * @brief [rw] System Error on Correctable Error Enable
   *
   * On rev 0 hw, this corresponds to SERR_EN
   * On rev 1 hw, this corresponds to SECE_EN
   * On rev 2 hw, this corresponds to PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN
   *
   * Field size: 1 bit
   */
  uint8_t serrEn;
} pcieRootCtrlCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Root Status and Control register
 *
 * This register may only be used for root complex mode.
 *
 * On rev 0 hw, this corresponds to ROOT_STATUS
 * On rev 1 hw, this corresponds to ROOT_STS
 * On rev 2 hw, this corresponds to ROOT_STATUS_REG
 *
 * @{
 */
typedef struct pcieRootStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Indicates that another PME is pending when the PME Status bit is Set.
   *
   * On rev 0 hw, this corresponds to PME_PEND
   * On rev 1 hw, this corresponds to PME_PND
   * On rev 2 hw, this corresponds to PCIE_CAP_PME_PENDING
   *
   * Field size: 1 bit
   */
  uint8_t pmePend;
  /**
   * @brief [rw] Indicates that PME was asserted by the PME Requester.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to PME_STATUS
   * On rev 1 hw, this corresponds to PME_STS
   * On rev 2 hw, this corresponds to PCIE_CAP_PME_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t pmeStatus;
  /**
   * @brief [ro] ID of the last PME Requester.
   *
   * This field is only valid when the PME Status bit is Set.
   *
   * On rev 0 hw, this corresponds to PME_REQ_ID
   * On rev 1 hw, this corresponds to PME_RID
   * On rev 2 hw, this corresponds to PCIE_CAP_PME_REQ
   *
   * Field size: 16 bits
   */
  uint16_t pmeReqID;
} pcieRootStatusReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Device Capabilities 2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to DEV_CAP2
 * On rev 1 hw, this corresponds to DEV_CAP_2
 * On rev 2 hw, this corresponds to DEVICE_CAPABILITIES2_REG
 *
 * @{
 */
typedef struct pcieDevCap2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Completion timeout disable supported
   *
   * On rev 0 hw, this corresponds to CMPL_TO_DIS_SUPP
   * On rev 1 hw, this corresponds to CPL_TIMEOUT_DIS_SUPPORTED
   * On rev 2 hw, this corresponds to PCIE_CAP_CPL_TIMEOUT_DISABLE_SUPPORT
   *
   * Field size: 1 bit
   */
  uint8_t cmplToDisSupp;
  /**
   * @brief [rw] Completion timeout ranges supported. Applicable to RC/EP that issue requests on own behalf.
   *
   * On rev 0 hw, this corresponds to CMPL_TO_EN
   * On rev 1 hw, this corresponds to CPL_TIMEOUT_RNG_SUPPORTED
   * On rev 2 hw, this corresponds to PCIE_CAP_CPL_TIMEOUT_RANGE
   *
   * Field size: 4 bits
   */
  uint8_t cmplToEn;
  /**
   * @brief [ro] ARI Forwarding Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to ARI_FWD_SP
   * On rev 2 hw, this corresponds to PCIE_CAP_ARI_FORWARD_SUPPORT
   *
   * Field size: 1 bit
   */
  uint8_t ariFwdSp;
  /**
   * @brief [ro] AtomicOp Routing Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to AOR_SP
   * On rev 2 hw, this corresponds to PCIE_CAP_ATOMIC_ROUTING_SUPP
   *
   * Field size: 1 bit
   */
  uint8_t aorSp;
  /**
   * @brief [ro] 32-bit AtomicOp Completer Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to AOC32_SP
   * On rev 2 hw, this corresponds to PCIE_CAP_32_ATOMIC_CPL_SUPP
   *
   * Field size: 1 bit
   */
  uint8_t aoc32Sp;
  /**
   * @brief [ro] 64-bit AtomicOp Completer Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to AOC64_SP
   * On rev 2 hw, this corresponds to PCIE_CAP_64_ATOMIC_CPL_SUPP
   *
   * Field size: 1 bit
   */
  uint8_t aoc64Sp;
  /**
   * @brief [ro] 128-bit CAS Completer Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to CASC128_SP
   * On rev 2 hw, this corresponds to PCIE_CAP_128_CAS_CPL_SUPP
   *
   * Field size: 1 bit
   */
  uint8_t casc128Sp;
  /**
   * @brief [ro] No RO-enabled PR-PR Passing
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to NOROPR
   * On rev 2 hw, this corresponds to PCIE_CAP_NO_RO_EN_PR2PR_PAR
   *
   * Field size: 1 bit
   */
  uint8_t noRoPR;
  /**
   * @brief [ro] LTR Mechanism Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PCIE_CAP_LTR_SUPP
   *
   * Field size: 2 bit
   */
  uint8_t ltrSupp;
  /**
   * @brief [ro] TPH Completer Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to TPHC_SP
   * On rev 2 hw, this corresponds to PCIE_CAP_TPH_CMPLT_SUPPORT_0 | PCIE_CAP_TPH_CMPLT_SUPPORT_1
   *
   * Field size: 2 bit
   */
  uint8_t tphcSp;
  /**
   * @brief [ro] LN System CLS
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PCIE_CAP2_LN_SYS_CLS
   *
   * Field size: 1 bit
   */
  uint8_t lnSysCls;
  /**
   * @brief [ro] 10-Bit Tag Completer Supported (ep only)
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PCIE_CAP2_10_BIT_TAG_COMP_SUPPORT
   *
   * Field size: 1 bit
   */
  uint8_t tag10bitCompSupp;
  /**
   * @brief [ro] 10-Bit Tag Requester Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PCIE_CAP2_10_BIT_TAG_REQ_SUPPORT
   *
   * Field size: 1 bit
   */
  uint8_t tag10bitReqSupp;
  /**
   * @brief [ro] [OBFF] Optimized Buffer Flush/fill Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PCIE_CAP_OBFF_SUPPORT
   *
   * Field size: 1 bit
   */
  uint8_t obffSupp;

} pcieDevCap2Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Device Status and Control Register 2
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to DEV_STAT_CTRL2
 * On rev 1 hw, this corresponds to DEV_CAS_2
 * On rev 2 hw, this corresponds to DEVICE_CONTROL2_DEVICE_STATUS2_REG
 *
 * @{
 */
typedef struct pcieDevStatCtrl2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Completion timeout disable
   *
   * On rev 0 hw, this corresponds to CMPL_TO_DIS
   * On rev 1 hw, this corresponds to CPL_TIMEOUT_DIS
   * On rev 2 hw, this corresponds to PCIE_CAP_CPL_TIMEOUT_DISABLE
   *
   * Field size: 1 bit
   */
  uint8_t cmplToDis;
  /**
   * @brief [rw] Completion timeout value.
   *
   * It is strongly recommended that the Completion Timeout mechanism
   * not expire in less than 10 ms.
   *
   * <TABLE>
   * <TR><TH>@ref cmplTo</TH><TH>low range</TH><TH>high range</TH></TR>
   * <TR><TD>0x0</TD>        <TD>50ms</TD>     <TD>50s</TD></TR>
   * <TR><TD>0x1</TD>        <TD>50s</TD>      <TD>100s</TD></TR>
   * <TR><TD>0x2</TD>        <TD>1ms</TD>      <TD>10ms</TD></TR>
   * <TR><TD>0x5</TD>        <TD>16ms</TD>     <TD>55ms</TD></TR>
   * <TR><TD>0x6</TD>        <TD>65ms</TD>     <TD>210ms</TD></TR>
   * <TR><TD>0x9</TD>        <TD>260ms</TD>    <TD>900ms</TD></TR>
   * <TR><TD>0xA</TD>        <TD>1s</TD>       <TD>3.5s</TD></TR>
   * <TR><TD>0xD</TD>        <TD>4s</TD>       <TD>13s</TD></TR>
   * <TR><TD>0xE</TD>        <TD>17s</TD>      <TD>64s</TD></TR>
   * <TR><TD>others</TD>     <TD>reserved</TD> <TD>reserved</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to CMPL_TO
   * On rev 1 hw, this corresponds to CPL_TIMEOUT_VALUE
   * On rev 2 hw, this corresponds to PCIE_CAP_CPL_TIMEOUT_VALUE
   *
   * Field size: 4 bits
   */
  uint8_t cmplTo;
  /**
   * @brief [rw] ARI Forwarding Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to ARI_FWD_SP
   * On rev 2 hw, this corresponds to PCIE_CAP_ARI_FORWARD_SUPPORT_CS
   *
   * Field size: 1 bit
   */
  uint8_t ariFwdSp;
  /**
   * @brief [rw] AtomicOp Requester Enabled
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to AOP_REQ_EN
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t aopReqEn;
  /**
   * @brief [rw] AtomicOp Egress Blocking
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to AOP_EG_BLK
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t aopEgBlk;
  /**
   * @brief [rw] IDO Request Enable
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to IDO_REQ_EN
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t idoReqEn;
  /**
   * @brief [rw] IDO Completion Enable
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to IDO_CPL_EN
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t idoCplEn;
  /**
   * @brief [rw] LTR Mechanism Enable
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to LTR_EN
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t ltrEn;
  /**
   * @brief [rw] OBFF Enable
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to OBFF_EN
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t obffEn;
} pcieDevStatCtrl2Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Link Capabilities 2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, unsupported
 * On rev 1 hw, this corresponds to LNK_CAP_2
 * On rev 2 hw, this corresponds to LINK_CAPABILITIES2_REG
 *
 * @{
 */
/* @} */
typedef struct pcieLnkCap2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Supported Link Speeds Vector
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to SP_LS_VEC
   * On rev 2 hw, this corresponds to PCIE_CAP_SUPPORT_LINK_SPEED_VECTOR
   *
   * Field size: 7 bits
   */
  uint8_t spLsVec;
  /**
   * @brief [ro] Crosslink Supported
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to CROSSLINK_SP
   * On rev 2 hw, this corresponds to PCIE_CAP_CROSS_LINK_SUPPORT
   *
   * Field size: 1 bit
   */
  uint8_t crosslinkSp;
} pcieLnkCap2Reg_t;

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Link Control 2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to LINK_CTRL2
 * On rev 1 hw, this corresponds to LNK_CAS_2
 * On rev 2 hw, this corresponds to LINK_CONTROL2_LINK_STATUS2_REG
 *
 * @{
 */
typedef struct pcieLinkCtrl2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Current De-emphasis level
   *
   * 0 = -6 dB\n
   * 1 = -3.5 dB
   *
   * On rev 0 hw, this corresponds to DE_EMPH
   * On rev 1 hw, this corresponds to DEEMPH_LEVEL
   * On rev 2 hw, this corresponds to PCIE_CAP_CURR_DEEMPHASIS
   *
   * Field size: 1 bit
   */
  uint8_t deEmph;
  /**
   * @brief [rw] De-emphasis level in polling-compliance state
   *
   * This bit sets the de-emphasis level in Polling Compliance state if the
   * entry occurred due to the Enter Compliance bit being 1.
   *
   * 0 = -6 dB\n
   * 1 = -3.5 dB
   *
   * On rev 0 hw, this corresponds to POLL_DEEMPH
   * On rev 1 hw, unsupported
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t pollDeemph;
  /**
   * @brief [rw] Compliance SOS.
   *
   * When this bit is set to 1, the LTSSM is required to send SKP
   * Ordered Sets periodically in between the modified compliance patterns.
   *
   * On rev 0 hw, this corresponds to CMPL_SOS
   * On rev 1 hw, this corresponds to COMPL_SOS
   * On rev 2 hw, this corresponds to PCIE_CAP_COMPLIANCE_SOS
   *
   * Field size: 1 bit
   */
  uint8_t cmplSos;
  /**
   * @brief [rw] Enter modified compliance.
   *
   * When this bit is set to 1, the device transmits Modified Compliance
   * Pattern if the LTSSM enters Polling Compliance substate.
   *
   * On rev 0 hw, this corresponds to ENTR_MOD_COMPL
   * On rev 1 hw, this corresponds to ENT_MOD_COMPL
   * On rev 2 hw, this corresponds to PCIE_CAP_ENTER_MODIFIED_COMPLIANCE
   *
   * Field size: 1 bit
   */
  uint8_t entrModCompl;
  /**
   * @brief [rw] Value of non-de-emphasized voltage level at transmitter pins.
   *
   * On rev 0 hw, this corresponds to TX_MARGIN
   * On rev 1 hw, this corresponds to TX_MARGIN
   * On rev 2 hw, this corresponds to PCIE_CAP_TX_MARGIN
   *
   * Field size: 3 bits
   */
  uint8_t txMargin;
  /**
   * @brief [rw] Selectable De-emphasis.
   *
   * When the Link is operating at 5.0 GT/s speed, this bit selects the level
   * of de-emphasis for an upstream component.  When the Link is operating at
   * 2.5 GT/s speed, the setting of this bit has no effect.
   *
   * 0 = -6 dB\n
   * 1 = -3.5 dB
   *
   * On rev 0 hw, this corresponds to SEL_DEEMPH
   * On rev 1 hw, this corresponds to SEL_DEEMP
   * On rev 2 hw, this corresponds to PCIE_CAP_SEL_DEEMPHASIS
   *
   * Field size: 1 bit
   */
  uint8_t selDeemph;
  /**
   * @brief [rw] Hardware Autonomous Speed Disable.
   *
   * 0 = Enable hardware to change the link speed.\n
   * 1 = Disables hardware from changing the Link speed for device specific
   * reasons other than attempting to correct unreliable Link operation by
   * reducing Link speed.
   *
   * On rev 0 hw, this corresponds to HW_AUTO_SPEED_DIS
   * On rev 1 hw, this corresponds to HW_AUTO_SP_DIS
   * On rev 2 hw, this corresponds to PCIE_CAP_HW_AUTO_SPEED_DISABLE
   *
   * Field size: 1 bit
   */
  uint8_t hwAutoSpeedDis;
  /**
   * @brief [rw] Enter Compliance.
   *
   * Software is permitted to force a Link to enter Compliance mode at the
   * speed indicated in the Target Link Speed field by setting this bit to
   * 1 in both components on a Link and then initiating a hot reset on the Link.
   *
   * On rev 0 hw, this corresponds to ENTR_COMPL
   * On rev 1 hw, this corresponds to ENTR_COMPL
   * On rev 2 hw, this corresponds to PCIE_CAP_ENTER_COMPLIANCE
   *
   * Field size: 1 bit
   */
  uint8_t entrCompl;
  /**
   * @brief [rw] Target Link Speed.
   *
   * 1h = 2.5 GT/s Target Link Speed.\n
   * 2h = 5.0 GT/s Target Link Speed.\n
   * Others = Reserved.
   *
   * On rev 0 hw, this corresponds to TGT_SPEED
   * On rev 1 hw, this corresponds to TRGT_LINK_SPEED
   * On rev 2 hw, this corresponds to PCIE_CAP_TARGET_LINK_SPEED
   *
   * Field size: 4 bits
   */
  uint8_t tgtSpeed;
  /**
   * @brief [rw] Compliance Pre-set/De-emphasis
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to COMPL_PRST_DEEMPH
   * On rev 2 hw, this corresponds to PCIE_CAP_COMPLIANCE_PRESET
   *
   * Field size: 4 bits
   */
  uint8_t complPrstDeemph;
  /**
   * @brief [ro] Equalization Complete, Gen3 Only
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to EQ_COMPLETE
   * On rev 2 hw, this corresponds to PCIE_CAP_EQ_CPL
   *
   * Field size: 1 bit
   */
  uint8_t eqComplete;
  /**
   * @brief [ro] Equalization Ph1 Success, Gen3 Only
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to EQ_PH1
   * On rev 2 hw, this corresponds to PCIE_CAP_EQ_CPL_P1
   *
   * Field size: 1 bit
   */
  uint8_t eqPh1;
  /**
   * @brief [ro] Equalization Ph2 Success, Gen3 Only
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to EQ_PH2
   * On rev 2 hw, this corresponds to PCIE_CAP_EQ_CPL_P2
   *
   * Field size: 1 bit
   */
  uint8_t eqPh2;
  /**
   * @brief [ro] Equalization Ph3 Success, Gen3 Only
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to EQ_PH3
   * On rev 2 hw, this corresponds to PCIE_CAP_EQ_CPL_P3
   *
   * Field size: 1 bit
   */
  uint8_t eqPh3;
  /**
   * @brief [rw] Link Equilization Request
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, this corresponds to LINK_EQ_REQ
   * On rev 2 hw, this corresponds to PCIE_CAP_LINK_EQ_REQ
   *
   * Field size: 1 bit
   */
  uint8_t linkEqReq;
  /**
   * @brief [rw] Downstream Component Presence
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to DOWNSTREAM_COMPO_PRESENCE
   *
   * Field size: 1 bit
   */
  uint8_t downCompPres;
  /**
   * @brief [rw] DRS Message Received
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to DRS_MESSAGE_RECEIVED
   *
   * Field size: 1 bit
   */
  uint8_t drsMsgRecv;
} pcieLinkCtrl2Reg_t;
/* @} */


/*****************************************************************************
 **********  PCIe EXTENDED CAPABILITIES  REGISTERS ***************************
 ****************************************************************************/
/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Extended Capabilities Header register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to PCIE_EXTCAP
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to AER_EXT_CAP_HDR_OFF
 *
 * @{
 */
typedef struct pcieExtCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Next Capability Pointer
   *
   * On rev 0 hw, this corresponds to NEXT_CAP
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to NEXT_OFFSET
   *
   * Field size: 12 bits
   */
  uint16_t nextCap;
  /**
   * @brief [ro] Extended Capability Version
   *
   * On rev 0 hw, this corresponds to EXT_CAP_VER
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CAP_VERSION
   *
   * Field size: 4 bits
   */
  uint8_t extCapVer;
  /**
   * @brief [ro] PCIe Extended Capability ID
   *
   * On rev 0 hw, this corresponds to EXT_CAP_ID
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CAP_ID
   *
   * Field size: 16 bits
   */
  uint16_t extCapID;
} pcieExtCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Uncorrectable Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to PCIE_UNCERR
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to UNCORR_ERR_STATUS_OFF
 *
 * @{
 */
typedef struct pcieUncErrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
 /**
   * @brief [rw] TLP Prefix Blocked Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to TLP_PRFX_BLOCKED_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t tlpPrfxBlockedErrSt;
  /**
   * @brief [rw] Uncorrectable Internal Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INTERNAL_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t intErrSt;
  /**
   * @brief [rw] Unsupported Request Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to UR_ERR_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to UNSUPPORTED_REQ_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t urErrSt;
  /**
   * @brief [rw] ECRC Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to ECRC_ERR_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ECRC_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t ecrcErrSt;
  /**
   * @brief [rw] Malformed TLP Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to MTLP_ERR_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MALF_TLP_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t mtlpErrSt;
  /**
   * @brief [rw] Receiver Overflow Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to RCVR_OF_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to REC_OVERFLOW_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t rcvrOfSt;
  /**
   * @brief [rw] Unexpected Completion Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to UCMP_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to UNEXP_CMPLT_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t ucmpSt;
  /**
   * @brief [rw] Completer Abort Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to CMPL_ABRT_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CMPLT_ABORT_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t cmplAbrtSt;
  /**
   * @brief [rw] Completion Timeout Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to CMPL_TMOT_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CMPLT_TIMEOUT_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t cmplTmotSt;
  /**
   * @brief [rw] Flow Control Protocol Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to FCP_ERR_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to FC_PROTOCOL_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t fcpErrSt;
  /**
   * @brief [rw] Poisoned TLP Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to PSND_TLP_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to POIS_TLP_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t psndTlpSt;
  /**
   * @brief [ro] Surprise Down Error Status. Not supported (always 0)
   *
   * On rev 0 hw, this corresponds to SRPS_DN_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to SURPRISE_DOWN_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t srpsDnSt;
  /**
   * @brief [rw] Data Link Protocol Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to DLP_ERR_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to DL_PROTOCOL_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t dlpErrSt;
} pcieUncErrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Uncorrectable Error Mask register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to PCIE_UNCERR_MASK
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to UNCORR_ERR_MASK_OFF
 *
 * @{
 */
typedef struct pcieUncErrMaskReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
 /**
   * @brief [ro] TLP Prefix Blocked Error Mask
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to TLP_PRFX_BLOCKED_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t tlpPrfxBlockedErrMsk;
 /**
   * @brief [ro] AtomicOp Egress Block Mask
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ATOMIC_EGRESS_BLOCKED_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t atomicEgressBlockedErrMsk;
  /**
   * @brief [rw] Uncorrectable Internal Error Mask
   *
   * Write 1 to clear
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INTERNAL_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t intErrMsk;
  /**
   * @brief [rw] Unsupported Request Error Mask
   *
   * On rev 0 hw, this corresponds to UR_ERR_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to UNSUPPORTED_REQ_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t urErrMsk;
  /**
   * @brief [rw] ECRC Error Mask
   *
   * On rev 0 hw, this corresponds to ECRC_ERR_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ECRC_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t ecrcErrMsk;
  /**
   * @brief [rw] Malformed TLP Mask
   *
   * On rev 0 hw, this corresponds to MTLP_ERR_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MALF_TLP_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t mtlpErrMsk;
  /**
   * @brief [rw] Receiver Overflow Mask
   *
   * On rev 0 hw, this corresponds to RCVR_OF_MASK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to REC_OVERFLOW_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t rcvrOfMsk;
  /**
   * @brief [rw] Unexpected Completion Mask
   *
   * On rev 0 hw, this corresponds to UCMP_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to NEXP_CMPLT_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t ucmpMsk;
  /**
   * @brief [rw] Completer Abort Mask
   *
   * On rev 0 hw, this corresponds to CMPL_ABRT_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CMPLT_ABORT_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t cmplAbrtMsk;
  /**
   * @brief [rw] Completion Timeout Mask
   *
   * On rev 0 hw, this corresponds to CMPL_TMOT_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CMPLT_TIMEOUT_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t cmplTmotMsk;
  /**
   * @brief [rw] Flow Control Protocol Error Mask
   *
   * On rev 0 hw, this corresponds to FCP_ERR_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to FC_PROTOCOL_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t fcpErrMsk;
  /**
   * @brief [rw] Poisoned TLP Mask
   *
   * On rev 0 hw, this corresponds to PSND_TLP_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to POIS_TLP_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t psndTlpMsk;
  /**
   * @brief [ro] Surprise Down Error Mask. Not supported (always 0)
   *
   * On rev 0 hw, this corresponds to SRPS_DN_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to SURPRISE_DOWN_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t srpsDnMsk;
  /**
   * @brief [rw] Data Link Protocol Error Mask
   *
   * On rev 0 hw, this corresponds to DLP_ERR_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to DL_PROTOCOL_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t dlpErrMsk;
} pcieUncErrMaskReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Uncorrectable Error Severity register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * Set each bit to 0 to indicate the error is non-fatal
 * Set each bit to 1 to indicate the error is fatal.
 *
 * On rev 0 hw, this corresponds to PCIE_UNCERR_SVRTY
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to UNCORR_ERR_SEV_OFF
 *
 * @{
 */
typedef struct pcieUncErrSvrtyReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
 /**
   * @brief [ro] TLP Prefix Blocked Error Mask
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to TLP_PRFX_BLOCKED_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t tlpPrfxBlockedErrSvrty;
 /**
   * @brief [ro] AtomicOp Egress Block Mask
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ATOMIC_EGRESS_BLOCKED_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t atomicEgressBlockedErrSvrty;
  /**
   * @brief [rw] Uncorrectable Internal Error Mask
   *
   * Write 1 to clear
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INTERNAL_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t intErrSvrty;
  /**
   * @brief [rw] Unsupported Request Error Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to UR_ERR_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to UNSUPPORTED_REQ_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t urErrSvrty;
  /**
   * @brief [rw] ECRC Error Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to ECRC_ERR_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ECRC_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t ecrcErrSvrty;
  /**
   * @brief [rw] Malformed TLP Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to MTLP_ERR_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MALF_TLP_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t mtlpErrSvrty;
  /**
   * @brief [rw] Receiver Overflow Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to RCVR_OF_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to REC_OVERFLOW_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t rcvrOfSvrty;
  /**
   * @brief [rw] Unexpected Completion Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to UCMP_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to UNEXP_CMPLT_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t ucmpSvrty;
  /**
   * @brief [rw] Completer Abort Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to CMPL_ABRT_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CMPLT_ABORT_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t cmplAbrtSvrty;
  /**
   * @brief [rw] Completion Timeout Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to CMPL_TMOT_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CMPLT_TIMEOUT_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t cmplTmotSvrty;
  /**
   * @brief [rw] Flow Control Protocol Error Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to FCP_ERR_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to FC_PROTOCOL_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t fcpErrSvrty;
  /**
   * @brief [rw] Poisoned TLP Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to PSND_TLP_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to POIS_TLP_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t psndTlpSvrty;
  /**
   * @brief [ro] Surprise Down Error Severity. Not supported (always 0)
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * On rev 0 hw, this corresponds to SRPS_DN_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to SURPRISE_DOWN_ERR_SVRITY
   *
   * Field size: 1 bit
   */
  uint8_t srpsDnSvrty;
  /**
   * @brief [rw] Data Link Protocol Error Severity
   *
   * 0 = Non-fatal; 1 = Fatal
   *
   * On rev 0 hw, this corresponds to DLP_ERR_SVRTY
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to DL_PROTOCOL_ERR_SEVERITY
   *
   * Field size: 1 bit
   */
  uint8_t dlpErrSvrty;
} pcieUncErrSvrtyReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Correctable Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to PCIE_CERR
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to CORR_ERR_STATUS_OFF
 *
 * @{
 */
typedef struct pcieCorErrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Header Log Overflow Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to HEADER_LOG_OVERFLOW_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t hdrLogOverflowErrSt;
  /**
   * @brief [rw] Corrected Internal Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CORRECTED_INT_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t corrIntErrSt;
  /**
   * @brief [rw] Advisory Non-Fatal Error Status
   *
   * This bit is Set by default to enable compatibility with software
   * that does not comprehend Role-Based Error Reporting.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to ADV_NFERR_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ADVISORY_NON_FATAL_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t advNFErrSt;
  /**
   * @brief [rw] Replay Timer Timeout Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to RPLY_TMR_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RPL_TIMER_TIMEOUT_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t rplyTmrSt;
  /**
   * @brief [rw] REPLAY_NUM Rollover Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to RPLT_RO_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to REPLAY_NO_ROLEOVER_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t rpltRoSt;
  /**
   * @brief [rw] Bad DLLP Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to BAD_DLLP_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to BAD_DLLP_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t badDllpSt;
  /**
   * @brief [rw] Bad TLP Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to BAD_TLP_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to BAD_TLP_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t badTlpSt;
  /**
   * @brief [rw] Receiver Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to RCV_ERR_ST
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RX_ERR_STATUS
   *
   * Field size: 1 bit
   */
  uint8_t rcvrErrSt;
} pcieCorErrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Correctable Error Mask register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to PCIE_CERR_MASK
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to CORR_ERR_MASK_OFF
 *
 * @{
 */
typedef struct pcieCorErrMaskReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Header Log Overflow Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to HEADER_LOG_OVERFLOW_MASK
   *
   * Field size: 1 bit
   */
  uint8_t hdrLogOverflowErrMsk;
  /**
   * @brief [rw] Corrected Internal Error Status
   *
   * Write 1 to clear
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CORRECTED_INT_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t corrIntErrMsk;
  /**
   * @brief [rw] Advisory Non-Fatal Error Mask
   *
   * On rev 0 hw, this corresponds to ADV_NFERR_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ADVISORY_NON_FATAL_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t advNFErrMsk;
  /**
   * @brief [rw] Replay Timer Timeout Mask
   *
   * On rev 0 hw, this corresponds to RPLY_TMR_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RPL_TIMER_TIMEOUT_MASK
   *
   * Field size: 1 bit
   */
  uint8_t rplyTmrMsk;
  /**
   * @brief [rw] REPLAY_NUM Rollover Mask
   *
   * On rev 0 hw, this corresponds to RPLT_RO_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to REPLAY_NO_ROLEOVER_MASK
   *
   * Field size: 1 bit
   */
  uint8_t rpltRoMsk;
  /**
   * @brief [rw] Bad DLLP Mask
   *
   * On rev 0 hw, this corresponds to BAD_DLLP_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to BAD_DLLP_MASK
   *
   * Field size: 1 bit
   */
  uint8_t badDllpMsk;
  /**
   * @brief [rw] Bad TLP Mask
   *
   * On rev 0 hw, this corresponds to BAD_TLP_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to BAD_TLP_MASK
   *
   * Field size: 1 bit
   */
  uint8_t badTlpMsk;
  /**
   * @brief [rw] Receiver Error Mask
   *
   * On rev 0 hw, this corresponds to RCVR_ERR_MSK
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RX_ERR_MASK
   *
   * Field size: 1 bit
   */
  uint8_t rcvrErrMsk;
} pcieCorErrMaskReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Advanced capabilities and control Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 0 hw.
 * It is not available on rev 1 hw.
 *
 * On rev 0 hw, this corresponds to PCIE_ACCR
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to ADV_ERR_CAP_CTRL_OFF
 *
 * @{
 */
typedef struct pcieAccrReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Multiple Header Recording Enable
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MULTIPLE_HEADER_EN
   *
   * Field size:  1bit
   */
  uint8_t multHdrEn;
  /**
   * @brief [ro] Multiple Header Recording Capable
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MULTIPLE_HEADER_CAP
   *
   * Field size:  1bit
   */
  uint8_t multHdrCap;
  /**
   * @brief [rw] ECRC Check Enable
   *
   * On rev 0 hw, this corresponds to ECRC_CHK_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ECRC_CHK_EN
   *
   * Field size:  1bit
   */
  uint8_t chkEn;
  /**
   * @brief [rw] ECRC Check Capable
   *
   * On rev 0 hw, this corresponds to ECRC_CHK_CAP
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ECRC_CHK_CAP
   *
   * Field size:  1 bit
   */
  uint8_t chkCap;
  /**
   * @brief [rw] ECRC Generation Enable
   *
   * On rev 0 hw, this corresponds to ECRC_GEN_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ECRC_GEN_EN
   *
   * Field size:  1 bit
   */
  uint8_t genEn;
  /**
   * @brief [rw] ECRC Generation Capability
   *
   * On rev 0 hw, this corresponds to ECRC_GEN_CAP
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ECRC_GEN_CAP
   *
   * Field size:  1 bit
   */
  uint8_t genCap;
  /**
   * @brief [rw] First Error Pointer
   *
   * The First Error Pointer is a field that identifies the bit position
   * of the first error reported in the @ref pcieUncErrReg_s
   *
   * On rev 0 hw, this corresponds to FRST_ERR_PTR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to FIRST_ERR_POINTER
   *
   * Field size:  5 bits
   */
  uint8_t erPtr;
} pcieAccrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Header Log registers
 *
 * These registers may be used for both endpoint and root complex modes.
 *
 * There are 4 Header Log registers
 *
 * On rev 0 hw, this corresponds to HDR_LOGn (n = 0..3)
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to HDR_LOG_n_OFF (n = 0..3)
 *
 * @{
 */
typedef struct pcieHdrLogReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] DWORD of header for a detected error
   *
   * On rev 0 hw, this corresponds to HDR_DWn (n = 0..3)
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to (n=FIRST..FOURTH)
   *              (n_DWORD_FOURTH_BYTE << 24) | (n_DWORD_THIRD_BYTE << 16) |
   *              (n_DWORD_SECOND_BYTE << 8 ) | (n_DWORD_FIRST_BYTE << 0 )
   * Field size: 32 bits
   */
  uint32_t hdrDW;
} pcieHdrLogReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Root Error Command register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to ROOT_ERR_CMD
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to ROOT_ERR_CMD_OFF (root complex only)
 *
 * @{
 */
typedef struct pcieRootErrCmdReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Fatal Error Reporting Enable.
   *
   * On rev 0 hw, this corresponds to FERR_RPT_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to FATAL_ERR_REPORTING_EN
   *
   * Field size:  1 bit
   */
  uint8_t ferrRptEn;
  /**
   * @brief [rw] Nonfatal Error Reporting Enable.
   *
   * On rev 0 hw, this corresponds to NFERR_RPT_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to NON_FATAL_ERR_REPORTING_EN
   *
   * Field size:  1 bit
   */
  uint8_t nferrRptEn;
  /**
   * @brief [rw] Correctable Error Reporting Enable.
   *
   * On rev 0 hw, this corresponds to CERR_RPT_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to CORR_ERR_REPORTING_EN
   *
   * Field size:  1 bit
   */
  uint8_t cerrRptEn;
} pcieRootErrCmdReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Root Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * On rev 0 hw, this corresponds to ROOT_ERR_ST
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to ROOT_ERR_STATUS_OFF (root complex only)
 *
 * @{
 */
typedef struct pcieRootErrStReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] AER Interrupt Message Number.
   *
   * On rev 0 hw, this corresponds to AER_INT_MSG
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ADV_ERR_INT_MSG_NUM
   *
   * Field size:  5 bits
   */
  uint8_t aerIntMsg;
  /**
   * @brief [rw] Fatal Error Messages Received.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to FERR_RCV
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to _FATAL_ERR_MSG_RX
   *
   * Field size:  1 bit
   */
  uint8_t ferrRcv;
  /**
   * @brief [rw] Non-Fatal Error Messages Received.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to NFERR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to NON_FATAL_ERR_MSG_RX
   *
   * Field size:  1 bit
   */
  uint8_t nfErr;
  /**
   * @brief [rw] First Uncorrectable Fatal Received.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to UNCOR_FATAL
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to FIRST_UNCORR_FATAL
   *
   * Field size:  1 bit
   */
  uint8_t uncorFatal;
  /**
   * @brief [rw] Multiple Uncorrectable Error (ERR_FATAL/NONFATAL) Received.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to MULT_FNF
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MUL_ERR_FATAL_NON_FATAL_RX
   *
   * Field size:  1 bit
   */
  uint8_t multFnf;
  /**
   * @brief [rw] Uncorrectable Error (ERR_FATAL/NONFATAL) Received.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to ERR_FNF
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_FATAL_NON_FATAL_RX
   *
   * Field size:  1 bit
   */
  uint8_t errFnf;
  /**
   * @brief [rw] Multiple Correctable Error (ERR_COR) Received.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to MULT_COR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MUL_ERR_COR_RX
   *
   * Field size:  1 bit
   */
  uint8_t multCor;
  /**
   * @brief [rw] Correctable Error (ERR_COR) Received.
   *
   * Write 1 to clear
   *
   * On rev 0 hw, this corresponds to CORR_ERR
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_COR_RX
   *
   * Field size:  1 bit
   */
  uint8_t corrErr;
} pcieRootErrStReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Error Source Identification register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 0 hw.
 * It is not available on rev 1 hw.
 *
 * On rev 0 hw, this corresponds to ERR_SRC_ID
 * On rev 1 hw, unsupported
 * On rev 2 hw, this corresponds to ERR_SRC_ID_OFF
 *
 * @{
 */
typedef struct pcieErrSrcIDReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Fatal or Non-Fatal error source identification
   *
   * On rev 0 hw, this corresponds to FNF_SRC_ID
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_FATAL_NON_FATAL_SOURCE_ID
   *
   * Field size:  16 bits
   */
  uint16_t fnfSrcID;
  /**
   * @brief [ro] Correctable error source identification
   *
   * On rev 0 hw, this corresponds to CORR_SRC_ID
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ERR_COR_SOURCE_ID
   *
   * Field size:  16 bits
   */
  uint16_t corrSrcID;
} pcieErrSrcIDReg_t;
/* @} */

/*****************************************************************************
 **********  PCIe LOCAL/REMOTE PORT LOGIC REGISTERS **************************
 ****************************************************************************/
/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Ack Latency Time and Replay Timer register
 *
 * On rev 0 hw, this corresponds to PL_ACKTIMER
 * On rev 1 hw, this corresponds to PCIECTRL_PL_LAT_REL_TIM
 * On rev 2 hw, this corresponds to ACK_LATENCY_TIMER_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pciePlAckTimerReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Replay Time Limit.
   *
   * For rev 0 hw:
   *
   * The replay timer expires when it reaches this limit.
   * Write sticky, a reset will not clear this field
   *
   * For rev 1 hw:
   *
   * The replay timer expires when it reaches this limit; RW 0xc0
   * The core initiates a replay upon reception of a Nak or
   * when the replay timer expires;
   * The default value depends on number of bytes (NB) per
   * cycle, which is defined by the maximum core base
   * frequency of the device PCIe core, corresponding to 250
   * MHz for PCIe-Gen2 (5 Gbps) operation.
   * The default is then updated based on the Negotiated Link
   * Width and Max_Payload_Size;
   * Note: If operating at 5 Gb/s, then the rounded-up value of
   * an additional (153 / CX_NB) cycles is added, where
   * CX_NB correspond to the number of PCIEPCS 8-bit input
   * symbols per single 16-bit lane, i.e. CX_NB=2. This
   * means at 5Gbps, 77 extra cycles should be considered
   * for the replay time limit.
   * This is for additional internal processing for received
   * TLPs and transmitted DLLPs.
   *
   * For rev 2 hw: REPLAY_TIME_LIMIT
   *
   * Field size:  16 bits
   */
  uint16_t rplyLmt;
  /**
   * @brief [rw] Round Trip Latency Time Limit.
   *
   * For rev 0 hw:
   *
   * The Ack/Nak latency timer expires when it reaches this limit.
   * Write sticky, a reset will not clear this field
   *
   * For rev 1 hw:
   *
   * The Ack/Nak latency timer expires when it reaches this limit;
   * The default value depends on number of bytes (NB) per
   * cycle, which is defined by the maximum core base
   * frequency of the device PCIe core, corresponding to 250
   * MHz for PCIe-Gen2 (5 Gbps) operation.
   * The default is then updated based on the Negotiated Link
   * Width and Max_Payload_Size.
   * Note: If operating at 5 Gb/s, then the rounded-up value of
   * an additional (51 / CX_NB) cycles is added, where
   * CX_NB correspond to the number of PCIEPCS 8-bit input
   * symbols per single 16-bit lane, i.e. CX_NB=2. This
   * means at 5Gbps, 26 extra cycles should be considered
   * for the acknowledge latency time limit.
   * This is for additional internal processing for received
   * TLPs and transmitted DLLPs.
   *
   * For rev 2 hw: ROUND_TRIP_LATENCY_TIME_LIMIT
   *
   * Field size:  16 bits
   */
  uint16_t rndTrpLmt;
} pciePlAckTimerReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Other Message register
 *
 * On rev 0 hw, this corresponds to PL_OMSG
 * On rev 1 hw, this corresponds to PCIECTRL_PL_VENDOR_SPECIFIC_DLLP
 * On rev 2 hw, this corresponds to VENDOR_SPEC_DLLP_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pciePlOMsgReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Other Message Register
   *
   * It can be used to send a specific PCI Express message in which
   * case this register is programmed with the payload and bit
   * @ref pcieLnkCtrlReg_s::msgReq set to transmit the message.
   *
   * On rev 0 hw, this corresponds to OMSG
   * On rev 1 hw, this corresponds to PCIECTRL_PL_VENDOR_SPECIFIC_DLLP.
   * On rev 2 hw, this corresponds to VENDOR_SPEC_DLLP.
   *
   * Field size:  32 bits
   */
  uint32_t oMsg;
} pciePlOMsgReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Port Force Link register
 *
 * On rev 0 hw, this corresponds to PL_FORCE_LINK
 * On rev 1 hw, this corresponds to PCIECTRL_PL_PT_LNK_R
 * On rev 2 hw, this corresponds to PORT_FORCE_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pciePlForceLinkReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Low Power Entrance Count
   *
   * On rev 0 hw, this corresponds to LPE_CNT
   * On rev 1 hw, this corresponds to LOW_POWER_ENTR_CNT.
   * On rev 2 hw, this is reserved.
   *
   * Field size:  8 bits
   */
  uint8_t lpeCnt;
  /**
   * @brief [rw] Do Deskew for SRIS
   *
   * Use the transitions from TS2 to Logical Idle Symbol, SKP OS to
   * Logical Idle Symbol, and FTS Sequence to SKP OS to do deskew
   * for SRIS instead of using received SKP OS if
   * doDeskewForSris is set to 1
   * Note: This register field is sticky
   *
   * On rev 0 & 1 hw, this is reserved
   * On rev 2 hw, this corresponds to DO_DESKEW_FOR_SRIS
   *
   * Field size:  1 bits
   */
  uint8_t doDeskewForSris;
  /**
   * @brief [rw] Link State.
   *
   * The link state that the PCIe will be forced to when
   * @ref forceLink field is set.  See @ref pcieLtssmState_e
   * for LTSSM states encoded values.
   *
   * On rev 0 hw, this corresponds to LNK_STATE
   * On rev 1 hw, this corresponds to FORCED_LINK_COMMAND
   * On rev 2 hw, this corresponds to LINK_STATE
   *
   * Field size:  6 bits
   */
  uint8_t lnkState;
  /**
   * @brief [rw] Force Link.
   *
   * Forces the link to the state specified by the @ref lnkState field.
   * The Force Link pulse will trigger link re-negotiation.  Self clears.
   *
   * on rev 0 hw, this corresponds to FORCE_LINK
   * on rev 1 hw, this corresponds to FORCE_LINK
   * on rev 2 hw, this corresponds to FORCE_EN
   *
   * Field size:  1 bit
   */
  uint8_t forceLink;
  /**
   * @brief [rw] LTSSM state forced by setting @ref forceLink
   *
   * on rev 0 hw: unsupported
   * on rev 1 hw, this corresponds to FORCED_LTSSM_STATE
   * on rev 2 hw, this corresponds to FORCED_LTSSM
   *
   * Field size:  4 bits
   */
  uint8_t forcedLtssmState;
  /**
   * @brief [rw] Link Number. Not used for EP.
   *
   * on rev 0 hw, this corresponds to LINK_NUM
   * on rev 1 hw, this corresponds to LINK_NUM
   * on rev 2 hw, this corresponds to LINK_NUM
   *
   * Field size:  8 bits
   */
  uint8_t linkNum;
} pciePlForceLinkReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Ack Frequency register
 *
 * On rev 0 hw, this corresponds to ACK_FREQ
 * On rev 1 hw, this corresponds to PCIECTRL_PL_ACK_FREQ_ASPM
 * On rev 2 hw, this corresponds to ACK_F_ASPM_CTRL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieAckFreqReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Allow ASPM L1 without partner going to L0s.
   *
   * Set to allow entering ASPM L1 even when link partner did not
   * go to L0s. When cleared, the ASPM L1 state is entered only after idle
   * period during which both RX and TX are in L0s.
   *
   * on rev 0 hw, this corresponds to ASPM_L1
   * on rev 1 hw, this corresponds to L1_ENTR_WO_L0S
   * on rev 2 hw, this corresponds to ENTER_ASPM
   *
   * Field size:  1 bit
   */
  uint8_t aspmL1;
  /**
   * @brief [rw] L1 entrance latency.
   *
   * The latency is set to 2^@ref l1EntryLatency microseconds with
   * the max being 64 microseconds.
   * <TABLE>
   * <TR><TH>@ref l1EntryLatency</TH><TH>latency in �s</TH></TR>
   * <TR><TD>0</TD>                  <TD>1�s</TD></TR>
   * <TR><TD>1</TD>                  <TD>2�s</TD></TR>
   * <TR><TD>2</TD>                  <TD>4�s</TD></TR>
   * <TR><TD>3</TD>                  <TD>8�s</TD></TR>
   * <TR><TD>4</TD>                  <TD>16�s</TD></TR>
   * <TR><TD>5</TD>                  <TD>32�s</TD></TR>
   * <TR><TD>6</TD>                  <TD>64�s</TD></TR>
   * <TR><TD>7</TD>                  <TD>64�s</TD></TR>
   * </TABLE>
   *
   * on rev 0 hw, this corresponds to L1_ENTRY_LATENCY
   * on rev 1 hw, this corresponds to L1_ENTR_LAT
   * on rev 2 hw, this corresponds to L1_ENTRANCE_LATENCY
   *
   * Field size:  3 bits
   */
  uint8_t l1EntryLatency;
  /**
   * @brief [rw] L0s entrance latency.
   *
   * The latency is set to @ref l0sEntryLatency + 1 microseconds.
   * Maximum is 7 microseconds.
   *
   * <TABLE>
   * <TR><TH>@ref l0sEntryLatency</TH><TH>latency in �s</TH></TR>
   * <TR><TD>0</TD>                   <TD>1�s</TD></TR>
   * <TR><TD>1</TD>                   <TD>2�s</TD></TR>
   * <TR><TD>2</TD>                   <TD>3�s</TD></TR>
   * <TR><TD>3</TD>                   <TD>4�s</TD></TR>
   * <TR><TD>4</TD>                   <TD>5�s</TD></TR>
   * <TR><TD>5</TD>                   <TD>6�s</TD></TR>
   * <TR><TD>6</TD>                   <TD>7�s</TD></TR>
   * <TR><TD>7</TD>                   <TD>7�s</TD></TR>
   * </TABLE>
   *
   * on rev 0 hw, this corresponds to L0S_ENTRY_LATENCY
   * on rev 1 hw, this corresponds to L0S_ENTR_LAT
   * on rev 2 hw, this corresponds to L0S_ENTRANCE_LATENCY
   *
   * Field size:  3 bits
   */
  uint8_t l0sEntryLatency;
  /**
   * @brief [rw] Number of fast training sequences for common clock
   *
   * Number of fast training sequences when common clock is used
   * and when transitioning from L0s to L0.
   *
   * On rev 0 hw, this corresponds to COMM_NFTS
   * On rev 1 hw, this corresponds to COMMON_CLK_N_FTS
   * On rev 2 hw, this corresponds to COMMON_CLK_N_FTS
   *
   * Field size:  8 bits
   */
  uint8_t commNFts;
  /**
   * @brief [rw] Number of fast training sequences to be transmitted
   *
   * Number of fast training sequences to be transmitted
   * when transitioning from L0s to L0. Value of 0 is not supported.
   *
   * On rev 0 hw, this corresponds to NFTS
   * On rev 1 hw, this corresponds to N_FTS
   * On rev 2 hw, this corresponds to ACK_N_FTS
   *
   * Field size:  8 bits
   */
  uint8_t nFts;
  /**
   * @brief [rw] Ack Frequency.
   *
   * Default is to wait until 255 Ack DLLPs are pending before it is sent.
   *
   * On rev 0 hw, this corresponds to ACK_FREQ
   * On rev 1 hw, this corresponds to ACK_FREQ
   * On rev 2 hw, this corresponds to ACK_FREQ
   *
   * Field size:  8 bits
   */
  uint8_t ackFreq;
} pcieAckFreqReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Port Link Control Register
 *
 * On rev 0 hw, this corresponds to PL_LINK_CTRL
 * On rev 1 hw, this corresponds to PCIECTRL_PL_PT_LNK_CTRL_R
 * On rev 2 hw, this corresponds to PORT_LINK_CTRL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieLnkCtrlReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Crosslink Active
   *
   * On rev 0 hw: unsupported
   * On rev 1 hw, this corresponds to CROSSLINK_ACT
   * On rev 2 hw: unsupported
   *
   * Field size: 1 bit
   */
  uint8_t crosslinkAct;
  /**
   * @brief [rw] Crosslink Enable
   *
   * On rev 0 hw: unsupported
   * On rev 1 hw, this corresponds to CROSSLINK_EN
   * On rev 2 hw: unsupported
   *
   * Field size: 1 bit
   */
  uint8_t crosslinkEn;
  /**
   * @brief [rw] Link Mode
   *
   * <TABLE>
   * <TR><TH>@ref lnkMode</TH><TH># of lanes</TH></TR>
   * <TR><TD>0x1</TD>         <TD>1</TD></TR>
   * <TR><TD>0x3</TD>         <TD>2</TD></TR>
   * <TR><TD>0x7</TD>         <TD>4</TD></TR>
   * <TR><TD>0xf</TD>         <TD>8</TD></TR>
   * <TR><TD>0x1f</TD>        <TD>16</TD></TR>
   * <TR><TD>0x3f</TD>        <TD>32</TD></TR>
   * <TR><TD>others</TD>      <TD>reserved</TD></TR>
   * </TABLE>
   *
   * On rev 0 hw, this corresponds to LNK_MODE
   * On rev 1 hw, this corresponds to LINK_MODE
   * On rev 2 hw, this corresponds to LINK_CAPABLE
   *
   * Field size: 6 bits
   */
  uint8_t lnkMode;
  /**
   * @brief [rw] Link Rate
   *
   * For 2.5 GT/s it is 0x1. This register does not affect any functionality.
   *
   * On rev 0 hw, this corresponds to LINK_RATE
   * On rev 1 hw: unsupported.
   * On rev 2 hw: unsupported.
   *
   * Field size: 4 bits
   */
  uint8_t lnkRate;
  /**
   * @brief [rw] Fast link mode
   *
   * Set all internal timers to fast mode for simulation purposes.
   *
   * On rev 0 hw, this corresponds to FLNK_MODE
   * On rev 1 hw, this corresponds to FAST_LINK
   * On rev 2 hw, this corresponds to FAST_LINK_MODE
   *
   * Field size: 1 bit
   */
  uint8_t fLnkMode;
  /**
   * @brief [rw] DLL link enable
   *
   * DLL Link Enable. Enable link initialization.
   *
   * On rev 0 hw, this corresponds to DLL_EN
   * On rev 1 hw, this corresponds to DL_EN
   * On rev 2 hw, this corresponds to DLL_LINK_EN
   *
   * Field size: 1 bit
   */
  uint8_t dllEn;
  /**
   * @brief [rw] Reset Assert
   *
   * Triggers a recovery and forces the LTSSM to the Hot Reset state.
   * Downstream ports (RC ports) only.
   *
   * On rev 0 hw, this corresponds to RST_ASRT
   * On rev 1 hw, this corresponds to RESET_ASSERT
   * On rev 2 hw, this corresponds to RESET_ASSERT
   *
   * Field size: 1 bit
   */
  uint8_t rstAsrt;
  /**
   * @brief [rw] Loopback Enable
   *
   * On rev 0 hw, this corresponds to LPBK_EN
   * On rev 1 hw, this corresponds to LB_EN
   * On rev 2 hw, this corresponds to LOOPBACK_ENABLE
   *
   * Field size: 1 bit
   */
  uint8_t lpbkEn;
  /**
   * @brief [rw] Scramble Disable
   *
   * On rev 0 hw, this corresponds to SCRM_DIS
   * On rev 1 hw, this corresponds to SCRAMBLE_DIS
   * On rev 2 hw, this corresponds to SCRAMBLE_DISABLE
   *
   * Field size: 1 bit
   */
  uint8_t scrmDis;
  /**
   * @brief [rw] Other Message Request
   *
   * Set to transmit the message contained in @ref pciePlOMsgReg_s
   *
   * On rev 0 hw, this corresponds to OMSG_REQ
   * On rev 1 hw, this corresponds to VEN_DLLP_REQ
   * On rev 2 hw, this corresponds to VENDOR_SPECIFIC_DLLP_REQ
   *
   * Field size: 1 bit
   */
  uint8_t msgReq;
} pcieLnkCtrlReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Lane Skew register
 *
 * On rev 0 hw, this corresponds to LANE_SKEW
 * On rev 1 hw, this corresponds to PCIECTRL_PL_LN_SKW_R
 * On rev 2 hw, this corresponds to LANE_SKEW_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieLaneSkewReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Set to Disable Lane to Lane Deskew.
   *
   * On rev 0 hw, this corresponds to L2L_DESKEW
   * On rev 1 hw, this corresponds to DIS_L2L_SKEW
   * On rev 2 hw, this corresponds to DISABLE_LANE_TO_LANE_DESKEW
   *
   * Field size: 1 bit
   */
  uint8_t l2Deskew;
  /**
   * @brief [rw] Set lanes allowed for loopback
   *
   * Implementation-specific Number of Lanes Set the implementation specific
   * number of lanes
   * Allowed values are:
   * 0000b: 1 lane
   * 0001b: 2 lanes
   * 0011b: 4 lanes
   * 0111b: 8 lanes
   * 1111b: 16 lanes
   * The number of lanes to be used when in Loopback
   * Master The number of lanes programmed must be equal to or less
   * than the valid number of lanes set in LINK_CAPABLE field You must
   * configure this field before initiating Loopback by writing in the
   * LOOPBACK_ENABLE field The controller will transition from
   * LoopbackEntry to LoopbackActive after receiving two consecutive
   * TS1 Ordered Sets with the Loopback bit asserted on the
   * implementation specific number of lanes configured in this field Note:
   * This register field is sticky
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to IMPLEMENT_NUM_LANES
   *
   * Field size: 4 bits
   */
  uint8_t implementNumLanes;
  /**
   * @brief [rw] Set to disable Ack and Nak DLLP transmission.
   *
   * On rev 0 hw, this corresponds to ACK_DISABLE
   * On rev 1 hw, this corresponds to ACKNAK_DIS
   * On rev 2 hw, this corresponds to ACK_NAK_DISABLE
   *
   * Field size: 1 bit
   */
  uint8_t ackDisable;
  /**
   * @brief [rw] Set to disable transmission of Flow Control DLLPs.
   *
   * On rev 0 hw, this corresponds to FC_DISABLE
   * On rev 1 hw, this corresponds to FC_DIS
   * On rev 2 hw, this corresponds to FLOW_CTRL_DISABLE
   *
   * Field size: 1 bit
   */
  uint8_t fcDisable;
  /**
   * @brief [rw] Insert Lane Skew for Transmit.
   *
   * The value is in units of one symbol time. Thus a value 0x02 will
   * force a skew of two symbol times for that lane. Max allowed is
   * 5 symbol times. This 24 bit field is used for programming skew
   * for eight lanes with three bits per lane.
   *
   * On rev 0 hw, this corresponds to LANE_SKEW
   * On rev 1 hw, this corresponds to LANE_SKEW
   * On rev 2 hw, this corresponds to INSERT_LANE_SKEW
   *
   * Field size: 24 bits
   */
  uint32_t laneSkew;
} pcieLaneSkewReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Symbol Number register
 *
 * On rev 0 hw, this corresponds to SYM_NUM
 * On rev 1 hw, this corresponds to PCIECTRL_PL_SYMB_N_R
 * On rev 2 hw, this corresponds to TIMER_CTRL_MAX_FUNC_NUM_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieSymNumReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Configuration requests targeted at function numbers above this
   * value will result in UR response.
   *
   * On rev 0 hw, this corresponds to MAX_FUNC  (3 bits)
   * On rev 1 hw, this corresponds to MAX_FUNC (8 bits)
   * On rev 2 hw, this corresponds to MAX_FUNC_NUM (8 bits)
   *
   * Field size: 8 bits
   */
  uint8_t maxFunc;
  /**
   * @brief [rw] Timer Modifier for Flow Control Watchdog Timer.
   *
   * Increases the timer value for Flow Control watchdog timer in
   * increments of 16 clock cycles.
   *
   * On rev 0 hw, this corresponds to FCWATCH_TIMER
   * On rev 1 hw: unsupported
   * On rev 2 hw: unsupported
   *
   * Field size: 5 bits
   */
  uint8_t fcWatchTimer;
  /**
   * @brief [rw] Timer Modifier for Ack/Nak Latency Timer.
   *
   * Increases the timer value for the Ack/Nak latency timer in
   * increments of 64 clock periods.
   *
   * On rev 0 hw, this corresponds to ACK_LATENCY_TIMER
   * On rev 1 hw, this corresponds to ACK_LATENCY_INC
   * On rev 2 hw, this corresponds to TIMER_MOD_ACK_NAK
   *
   * Field size: 5 bits
   */
  uint8_t ackLatencyTimer;
  /**
   * @brief [rw] Timer for replaying TLPs in increments of 64 clock cycles.
   *
   * Increases the timer value for Ack/Nak latency timer in increments of
   * 64 clock cycles.
   *
   * On rev 0 hw, this corresponds to REPLAY_TIMER
   * On rev 1 hw, this corresponds to REPLAY_ADJ
   * On rev 2 hw, this corresponds to TIMER_MOD_REPLAY_TIMER
   *
   * Field size: 5 bits
   */
  uint8_t replayTimer;
  /**
   * @brief [rw] Number of SKP Symbols.
   *
   * On rev 0 hw, this corresponds to SKP_COUNT
   * On rev 1 hw: unsupported
   * On rev 2 hw: unsupported
   *
   * Field size: 3 bits
   */
  uint8_t skpCount;
  /**
   * @brief [rw] Number of TS2 Symbols.
   *
   * This field does not affect any functionality.
   *
   * On rev 0 hw, this corresponds to NUM_TS2_SYMBOLS
   * On rev 1 hw: unsupported
   * On rev 2 hw: unsupported
   *
   * Field size: 4 bits
   */
  uint8_t numTs2Symbols;
  /**
   * @brief [rw] Number of TS Symbols.
   *
   * Set the number of TS identifier symbols that are sent in TS1 and TS2
   * ordered sets.
   *
   * On rev 0 hw, this corresponds to TS_COUNT
   * On rev 1 hw: unsupported
   * On rev 2 hw: unsupported
   *
   * Field size: 4 bits
   */
  uint8_t tsCount;
  /**
   * @brief [rw] Number of TS Symbols.
   *
   * Set the number of TS identifier symbols that are sent in TS1 and TS2
   * ordered sets.
   *
   * On rev 0 hw: unsupported
   * On rev 1 hw: unsupported
   * On rev 2 hw: FAST_LINK_SCALING_FACTOR
   *
   * Fast Link Timer Scaling Factor Sets the scaling factor of LTSSM
   * timer when FAST_LINK_MODE field in PCIE_EP_PORT_LINK_CTRL_OFF is set to '1'
   * 0: Scaling Factor is 1024 [1ms is 1us]
   * 1: Scaling Factor is 256 [1ms is 4us]
   * 2: Scaling Factor is 64 [1ms is 16us]
   * 3: Scaling Factor is 16 [1ms is 64us]
   * Not used for M-PCIe Note: This register field is sticky
   *
   * Field size: 3 bits
   */
  uint8_t fastLinkScalingFactor;
} pcieSymNumReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Symbol Timer and Filter Mask register
 *
 * On rev 0 hw, this corresponds to SYMTIMER_FLTMASK
 * On rev 1 hw, this corresponds to PCIECTRL_PL_SYMB_T_R
 * On rev 2 hw, this corresponds to SYMBOL_TIMER_FILTER_1_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieSymTimerFltMaskReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] 1 = Allow CFG transaction being received on RC.
   *
   * On rev 0 hw, this corresponds to F1_CFG_DROP
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_RC_CFG_DISCARD
   *
   * Field size: 1 bit
   */
  uint8_t f1CfgDrop;
  /**
   * @brief [rw] 1 = Allow IO transaction being received on RC.
   *
   * On rev 0 hw, this corresponds to F1_IO_DROP
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_RC_IO_DISCARD
   *
   * Field size: 1 bit
   */
  uint8_t f1IoDrop;
  /**
   * @brief [rw] 1 = Allow MSG transaction being received on RC.
   *
   * On rev 0 hw, this corresponds to F1_MSG_DROP
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_MSG_DROP
   *
   * Field size: 1 bit
   */
  uint8_t f1MsgDrop;
  /**
   * @brief [rw] 1 = Allow completion TLPs with ECRC errors to be passed up.
   *
   * On rev 0 hw, this corresponds to F1_CPL_ECRC_DROP
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_CPL_ECRC_DISCARD
   *
   * Field size: 1 bit
   */
  uint8_t f1CplEcrcDrop;
  /**
   * @brief [rw] 1 = Allow TLPs with ECRC errors to be passed up.
   *
   * On rev 0 hw, this corresponds to F1_ECRC_DROP
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_ECRC_DISCARD
   *
   * Field size: 1 bit
   */
  uint8_t f1EcrcDrop;
  /**
   * @brief [rw] 1 = Mask length match for received completion TLPs.
   *
   * On rev 0 hw, this corresponds to F1_CPL_LEN_TEST
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_CPL_LEN_MATCH
   *
   * Field size: 1 bit
   */
  uint8_t f1CplLenTest;
  /**
   * @brief [rw] 1 = Mask attribute match on received completion TLPs.
   *
   * On rev 0 hw, this corresponds to F1_CPL_ATTR_TEST
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_CPL_ATTR_MATCH
   *
   * Field size: 1 bit
   */
  uint8_t f1CplAttrTest;
  /**
   * @brief [rw] 1 = Mask traffic class match on received completion TLPs.
   *
   * On rev 0 hw, this corresponds to F1_CPL_TC_TEST
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_CPL_TC_MATCH
   *
   * Field size: 1 bit
   */
  uint8_t f1CplTcTest;
  /**
   * @brief [rw] 1 = Mask function match for received completion TLPs.
   *
   * On rev 0 hw, this corresponds to F1_CPL_FUNC_TEST
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_CPL_FUNC_MATCH
   *
   * Field size: 1 bit
   */
  uint8_t f1CplFuncTest;
  /**
   * @brief [rw] 1 = Mask request ID match for received completion TLPs.
   *
   * On rev 0 hw, this corresponds to F1_CPL_REQID_TEST
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_CPL_REQID_MATCH
   *
   * Field size: 1 bit
   */
  uint8_t f1CplReqIDTest;
  /**
   * @brief [rw] 1 = Mask tag error rules for received completion TLPs.
   *
   * On rev 0 hw, this corresponds to F1_CPL_TAGERR_TEST
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_CPL_TAGERR_MATCH
   *
   * Field size: 1 bit
   */
  uint8_t f1CplTagErrTest;
  /**
   * @brief [rw] 1 = Treat locked read TLPs as supported for EP, UR for RC.
   *
   * On rev 0 hw, this corresponds to F1_LOCKED_RD_AS_UR
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_LOCKED_RD_AS_UR
   *
   * Field size: 1 bit
   */
  uint8_t f1LockedRdAsUr;
  /**
   * @brief [rw] 1 = Treat type 1 CFG TLPs as supported for EP and UR for RC.
   *
   * On rev 0 hw, this corresponds to F1_RE_AS_US
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_CFG_TYPE1_RE_AS_UR
   *
   * Field size: 1 bit
   */
  uint8_t f1Cfg1ReAsUs;
  /**
   * @brief [rw] 1 = Treat out-of-BAR TLPs as supported requests.
   *
   * On rev 0 hw, this corresponds to F1_UR_OUT_OF_BAR
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_UR_OUTSIDE_BAR
   *
   * Field size: 1 bit
   */
  uint8_t f1UrOutOfBar;
  /**
   * @brief [rw] 1 = Treat poisoned TLPs as supported requests.
   *
   * On rev 0 hw, this corresponds to F1_UR_POISON
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_UR_POIS
   *
   * Field size: 1 bit
   */
  uint8_t f1UrPoison;
  /**
   * @brief [rw] 1 = Treat function mismatched TLPs as supported requests.
   *
   * On rev 0 hw, this corresponds to F1_UR_FUN_MISMATCH
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_1
   * On rev 2 hw: this corresponds to MASK_RADM_1.CX_FLT_MASK_UR_FUNC_MISMATCH
   *
   * Field size: 1 bit
   */
  uint8_t f1UrFunMismatch;
  /**
   * @brief [rw] 1 = Disable Flow Control watchdog timer.
   *
   * On rev 0 hw, this corresponds to FC_WDOG_DISABLE
   * On rev 1 hw, this corresponds to DIS_FC_TIM
   * On rev 2 hw, this corresponds to DISABLE_FC_WD_TIMER
   *
   * Field size: 1 bit
   */
  uint8_t fcWdogDisable;
  /**
   * @brief [rw] Wait time between SKP ordered sets
   *
   * Number of symbol times to wait between transmitting SKP
   * ordered sets. For example, for a setting of 1536 decimal,
   * the wait will be for 1537 symbol times.
   *
   * On rev 0 hw, this corresponds to SKP_VALUE
   * On rev 1 hw, this corresponds to SKP_INT
   * On rev 2 hw, this corresponds to SKP_INT_VAL
   *
   * Field size: 11 bits
   */
  uint16_t skpValue;
} pcieSymTimerFltMaskReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Filter Mask 2 register
 *
 * On rev 0 hw, this corresponds to FLT_MASK2
 * On rev 1 hw, this corresponds to FLT_MSK_2
 * On rev 2 hw, this corresponds to FILTER_MASK_2_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieFltMask2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] 1 = Drop PRS messages silently
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw: this corresponds to MASK_RADM_2.CX_FLT_MASK_PRS_DROP
   *
   * Field size: 1 bit
   */
  uint8_t dropPRS;
  /**
   * @brief [rw] 1 = Enable unmask TD bit if CX_STRIP_ECRC_ENABLE
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw: this corresponds to MASK_RADM_2.CX_FLT_UNMASK_TD
   *
   * Field size: 1 bit
   */
  uint8_t unmaskTD;
  /**
   * @brief [rw] 1 = Enable unmask CX_FLT_MASK_UR_POIS with TRGT0 destination
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw: this corresponds to MASK_RADM_2.CX_FLT_UNMASK_UR_POIS_TRGT0
   *
   * Field size: 1 bit
   */
  uint8_t unmaskUrPOIS;
  /**
   * @brief [rw] 1 = Drop LN Messages silently
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw: this corresponds to MASK_RADM_2.CX_FLT_MASK_LN_VENMSG1_DROP
   *
   * Field size: 1 bit
   */
  uint8_t dropLN;
  /**
   * @brief [rw] 1 = Enable the filter to handle flush request.
   *
   * On rev 0 hw, this corresponds to FLUSH_REQ
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_2
   * On rev 2 hw: this corresponds to MASK_RADM_2.CX_FLT_MASK_HANDLE_FLUSH
   *
   * Field size: 1 bit
   */
  uint8_t flushReq;
  /**
   * @brief [rw] 1 = Disable DLLP abort for unexpected CPL.
   *
   * On rev 0 hw, this corresponds to DLLP_ABORT
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_2
   * On rev 2 hw: this corresponds to MASK_RADM_2.CX_FLT_MASK_DABORT_4UCPL
   *
   * Field size: 1 bit
   */
  uint8_t dllpAbort;
  /**
   * @brief [rw] 1 = Disable dropping of Vendor MSG Type 1.
   *
   * On rev 0 hw, this corresponds to VMSG1_DROP
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_2
   * On rev 2 hw: this corresponds to MASK_RADM_2.CX_FLT_MASK_VENMSG1_DROP
   *
   * Field size: 1 bit
   */
  uint8_t vmsg1Drop;
  /**
   * @brief [rw] 1 = Disable dropping of Vendor MSG Type 0 with UR error reporting.
   *
   * On rev 0 hw, this corresponds to VMSG0_DROP
   * On rev 1 hw: While not in TRM, these bits are part of FLT_MSK_2
   * On rev 2 hw: this corresponds to MASK_RADM_2.CX_FLT_MASK_VENMSG0_DROP
   *
   * Field size: 1 bit
   */
  uint8_t vmsg0Drop;
} pcieFltMask2Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Debug0 Register
 *
 * On rev 0 hw, this corresponds to DEBUG0
 * On rev 1 hw, this corresponds to reserved
 * On rev 2 hw, this corresponds to PL_DEBUG0_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieDebug0Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Link control bits advertised by link partner.
   *
   * On rev 0 hw, this corresponds to TS_LNK_CTRL
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 4 bits
   */
  uint8_t tsLnkCtrl;
  /**
   * @brief [ro] Currently receiving k237 (PAD) in place of lane number.
   *
   * On rev 0 hw, this corresponds to TS_LANE_K237
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t tsLaneK237;
  /**
   * @brief [ro] Currently receiving k237 (PAD) in place of link number.
   *
   * On rev 0 hw, this corresponds to TS_LINK_K237
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t tsLinkK237;
  /**
   * @brief [ro] Receiver is receiving logical idle
   *
   * On rev 0 hw, this corresponds to RCVD_IDLE0
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t rcvdIdle0;
  /**
   * @brief [ro] 2nd symbol is also idle
   *
   * On rev 0 hw, this corresponds to RCVD_IDLE1
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t rcvdIdle1;
  /**
   * @brief [ro] Pipe TX data
   *
   * On rev 0 hw, this corresponds to PIPE_TXDATA
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 16 bits
   */
  uint16_t pipeTxData;
  /**
   * @brief [ro] Pipe transmit K indication
   *
   * On rev 0 hw, this corresponds to PIPE_TXDATAK
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 2 bits
   */
  uint8_t pipeTxDataK;
  /**
   * @brief [ro] A skip ordered set has been transmitted.
   *
   * On rev 0 hw, this corresponds to TXB_SKIP_TX
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t skipTx;
  /**
   * @brief [ro] LTSSM current state @ref pcieLtssmState_e
   *
   * On rev 0 hw, this corresponds to LTSSM_STATE
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 5 bits
   */
  uint8_t ltssmState;
} pcieDebug0Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Debug 1 Register
 *
 * On rev 0 hw, this corresponds to DEBUG1
 * On rev 1 hw, this corresponds to reserved
 * On rev 2 hw, this corresponds to PL_DEBUG1_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieDebug1Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Scrambling disabled for the link.
   *
   * On rev 0 hw, this corresponds to SCRAMBLER_DISABLE
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t scramblerDisable;
  /**
   * @brief [rw] LTSSM in DISABLE state. Link inoperable.
   *
   * On rev 0 hw, this corresponds to LINK_DISABLE
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t linkDisable;
  /**
   * @brief [rw] LTSSM performing link training.
   *
   * On rev 0 hw, this corresponds to LINK_IN_TRAINING
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t linkInTraining;
  /**
   * @brief [rw] LTSSM testing for polarity reversal.
   *
   * On rev 0 hw, this corresponds to RCVR_REVRS_POL_EN
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t rcvrRevrsPolEn;
  /**
   * @brief [rw] LTSSM-negotiated link reset.
   *
   * On rev 0 hw, this corresponds to TRAINING_RST_N
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t trainingRstN;
  /**
   * @brief [rw] PIPE receiver detect/loopback request.
   *
   * On rev 0 hw, this corresponds to PIPE_TXDETECTRX_LB
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t pipeTxdetectrxLb;
  /**
   * @brief [rw] PIPE transmit electrical idle request.
   *
   * On rev 0 hw, this corresponds to PIPE_TXELECIDLE
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t pipeTxelecidle;
  /**
   * @brief [rw] PIPE transmit compliance request.
   *
   * On rev 0 hw, this corresponds to PIPE_TXCOMPLIANCE
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t pipeTxcompliance;
  /**
   * @brief [rw] Application request to initiate training reset.
   *
   * On rev 0 hw, this corresponds to APP_INIT_RST
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t appInitRst;
  /**
   * @brief [rw] Link number advertised/confirmed by link partner.
   *
   * On rev 0 hw, this corresponds to RMLH_TS_LINK_NUM
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 8 bits
   */
  uint8_t rmlhTsLinkNum;
  /**
   * @brief [rw] LTSSM reports PHY link up.
   *
   * On rev 0 hw, this corresponds to XMLH_LINK_UP
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t xmlhLinkUp;
  /**
   * @brief [rw] Receiver reports skip reception.
   *
   * On rev 0 hw, this corresponds to RMLH_INSKIP_RCV
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t rmlhInskipRcv;
  /**
   * @brief [rw] TS1 training sequence received (pulse).
   *
   * On rev 0 hw, this corresponds to RMLH_TS1_RCVD
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t rmlhTs1Rcvd;
  /**
   * @brief [rw] TS2 training sequence received (pulse).
   *
   * On rev 0 hw, this corresponds to RMLH_TS2_RCVD
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t rmlhTs2Rcvd;
  /**
   * @brief [rw] Receiver detected lane reversal.
   *
   * On rev 0 hw, this corresponds to RMLH_RCVD_LANE_REV
   * On rev 1 hw, unsupported
   * On rev 2 hw, presumed same as rev 0
   *
   * Field size: 1 bit
   */
  uint8_t rmlhRcvdLaneRev;
} pcieDebug1Reg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Gen2 Register
 *
 * On rev 0 hw, this corresponds to PL_GEN2
 * On rev 1 hw, this corresponds to PCIECTRL_PL_WIDTH_SPEED_CTL
 * On rev 2 hw, this corresponds to GEN2_CTRL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieGen2Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Electrical Idle Inference Mode at Gen1 Rate
   *
   *  0: Use RxElecIdle signal to infer Electrical Idle -
   *  1: Use RxValid signal to infer Electrical Idle
   *
   * Note: This register field is sticky
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to GEN1_EI_INFERENCE
   *
   * Field size: 1 bit
   */
  uint8_t gen1EiInference;
  /**
   * @brief [rw] Set de-emphasis level for upstream (EP) ports
   *
   * On rev 0 hw, this corresponds to DEEMPH
   * On rev 1 hw, this corresponds to CFG_UP_SEL_DEEMPH
   * On rev 2 hw, this corresponds to SEL_DEEMPHASIS
   *
   * Field size: 1 bit
   */
  uint8_t deemph;
  /**
   * @brief [rw] Configure TX compliance receive bit.
   *
   * On rev 0 hw, this corresponds to CFG_TX_CMPL
   * On rev 1 hw, this corresponds to CFG_TX_COMPLIANCE_RCV
   * On rev 1 hw, this corresponds to CONFIG_TX_COMP_RX
   *
   * Field size: 1 bit
   */
  uint8_t txCmpl;
  /**
   * @brief [rw] Configure PHY TX Swing
   *
   * 0 = Low Swing\n
   * 1 = Full Swing
   *
   * On rev 0 hw, this corresponds to CFG_TX_SWING
   * On rev 1 hw, this corresponds to CFG_PHY_TXSWING
   * On rev 2 hw, this corresponds to CONFIG_PHY_TX_CHANGE
   *
   * Field size: 1 bit
   */
  uint8_t txSwing;
  /**
   * @brief [rw] direct speed change
   *
   * 0 = Indicates to the LTSSM not to initiate a speed change to Gen2
   * after the link is initialized at Gen1 speed.\n
   * 1 = Indicates to the LTSSM to initiate a speed change to Gen2
   * after the link is initialized at Gen1 speed.
   *
   * On rev 0 hw, this corresponds to DIR_SPD
   * On rev 1 hw, this corresponds to CFG_DIRECTED_SPEED_CHANGE
   * On rev 2 hw, this corresponds to DIRECT_SPEED_CHANGE
   *
   * Field size: 1 bit
   */
  uint8_t dirSpd;
  /**
   * @brief [rw] Enable Auto flipping of the lanes
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to AUTO_LANE_FLIP_CTRL_EN
   *
   * Field size: 1 bits
   */
  uint8_t autoFlipEn;
  /**
   * @brief [rw] Predetermined Lane for Auto Flip
   *
   * Flip This field defines which physical
   * lane is connected to logical Lane0 by the flip operation 
   * performed in Detect 
   * Allowed values are: - 3'b
   * 000: Connect logical Lane0 to physical lane 0 or CX_NL-1 or CX_NL/
   *      2-1 or CX_NL/
   *      4-1 or CX_NL/
   *      8-1, depending on which lane is detected
   * 001: Connect logical Lane0 to physical lane 1
   * 010: Connect logical Lane0 to physical lane 3
   * 011: Connect logical Lane0 to physical lane 7
   * 100: Connect logical Lane0 to physical lane 15 
   *
   * On rev 0 hw, unsupported
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to PRE_DET_LANE
   *
   * Field size: 3 bits
   */
  uint8_t preDetLane;
  /**
   * @brief [rw]  Lane enable. 1h=x1, 2h=x2. Other values reserved.
   *
   * On rev 0 hw, this corresponds to LN_EN
   * On rev 1 hw, this corresponds to CFG_LANE_EN
   * On rev 2 hw, this corresponds to NUM_OF_LANES
   *
   * Field size: 9 bits (5 bits rev 2)
   */
  uint16_t lnEn;
  /**
   * @brief [rw] number of fast training sequences
   *
   * On rev 0 hw, this corresponds to NUM_FTS
   * On rev 1 hw, this corresponds to CFG_GEN2_N_FTS
   * On rev 2 hw, this corresponds to FAST_TRAINING_SEQ
   *
   * Field size: 8 bit
   */
  uint8_t numFts;
} pcieGen2Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the AXI Multiple Outbound Decomposed NP SubRequests
 * Control Register (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_OBNP_SUBREQ_CTRL
 * On rev 2 hw, this corresponds to AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfObnpSubreqCtrlReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Enable AXI Multiple Outbound Decomposed NP Sub-Requests
   *
   * On rev 0 hw: unavailable
   * On rev 1 hw, this corresponds to EN_OBNP_SUBREQ
   * On rev 2 hw, this corresponds to OB_RD_SPLIT_BURST_EN
   *
   * Field size: 1 bit
   */
  uint8_t enObnpSubreq;
} pciePlconfObnpSubreqCtrlReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Transmit Posted FC Credit Status Register (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_TR_P_STS_R
 * On rev 2 hw, this corresponds to TX_P_FC_CREDIT_STATUS_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 *
 * @{
 */
typedef struct  pciePlconfTrPStsRReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Transmit Posted Data FC Credits
   *
   * On rev 1 hw, this corresponds to PD_CRDT
   * On rev 2 hw, this corresponds to TX_P_DATA_FC_CREDIT
   *
   * Field size: 12 bits
   */
  uint16_t pdCrdt;
  /**
   * @brief [ro] Transmit Posted Header FC Credits
   *
   * On rev 1 hw, this corresponds to PH_CRDT
   * On rev 2 hw, this corresponds to TX_P_HEADER_FC_CREDIT
   *
   * Field size: 8 bits
   */
  uint8_t phCrdt;
} pciePlconfTrPStsRReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Transmit Non-Posted FC Credit Status Register
 * (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_TR_NP_STS_R
 * On rev 2 hw, this corresponds to TX_NP_FC_CREDIT_STATUS_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 *
 * @{
 */
typedef struct  pciePlconfTrNpStsRReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Transmit Non-Posted Data FC Credits
   *
   * On rev 1 hw, this corresponds to NPD_CRDT
   * On rev 2 hw, this corresponds to TX_NP_DATA_FC_CREDIT
   *
   * Field size: 12 bits
   */
  uint16_t npdCrdt;
  /**
   * @brief [ro] Transmit Non-Posted Header FC Credits
   *
   * On rev 1 hw, this corresponds to NPH_CRDT
   * On rev 2 hw, this corresponds to TX_NP_HEADER_FC_CREDIT
   *
   * Field size: 8 bits
   */
  uint8_t nphCrdt;
} pciePlconfTrNpStsRReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Transmit Completion FC Credit Status Register
 * (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_TR_C_STS_R
 * On rev 2 hw, this corresponds to TX_CPL_FC_CREDIT_STATUS_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 *
 * @{
 */
typedef struct  pciePlconfTrCStsRReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Transmit Completion Data FC Credits
   *
   * On rev 1 hw, this corresponds to NPD_CRDT
   * On rev 2 hw, this corresponds to TX_CPL_DATA_FC_CREDIT
   *
   * Field size: 12 bits
   */
  uint16_t cpldCrdt;
  /**
   * @brief [ro] Transmit Completion Header FC Credits
   *
   * On rev 1 hw, this corresponds to NPH_CRDT
   * On rev 2 hw, this corresponds to TX_CPL_HEADER_FC_CREDIT
   *
   * Field size: 8 bits
   */
  uint8_t cplhCrdt;
} pciePlconfTrCStsRReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Queue Status Register (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_Q_STS_R
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfQStsRReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Received TLP FC Credits Not Returned
   *
   * On rev 1 hw, this corresponds to CRDT_NOT_RTRN
   * On rev 2 hw, this corresponds to RX_TLP_FC_CREDIT_NON_RETURN
   *
   * Field size: 1 bit
   */
  uint8_t crdtNotRtrn;
  /**
   * @brief [ro] Transmit Retry Buffer Not Empty
   *
   * On rev 1 hw, this corresponds to RTYB_NOT_EMPTY
   * On rev 2 hw, this corresponds to TX_RETRY_BUFFER_NE
   *
   * Field size: 1 bit
   */
  uint8_t rtybNotEmpty;
  /**
   * @brief [ro] Received Queue Not Empty
   *
   * On rev 1 hw, this corresponds to RCVQ_NOT_EMPTY
   * On rev 2 hw, this corresponds to RX_QUEUE_NON_EMPTY
   *
   * Field size: 1 bit
   */
  uint8_t rcvqNotEmpty;
  /**
   * @brief [r/w1c] Receive Credit Queue Overflow
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RX_QUEUE_OVERFLOW
   *
   * Field size: 1 bit
   */
  uint8_t rxQueueOverflow;
  /**
   * @brief [r/w1c] Receive Serialization Queue Not Empty
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RX_SERIALIZATION_Q_NON_EMPTY
   *
   * Field size: 1 bit
   */
  uint8_t rxSerQNEmpty;
  /**
   * @brief [r/w1c] Receive Serialization Queue Write Error
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RX_SERIALIZATION_Q_WRITE_ERR
   *
   * Field size: 1 bit
   */
  uint8_t rxSerQWErr;
  /**
   * @brief [r/w1c] Receive Serialization Read Error
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to RX_SERIALIZATION_Q_READ_ERR
   *
   * Field size: 1 bit
   */
  uint8_t rxSerRErr;
  /**
   * @brief [rw] FC Latency Timer Override Value
   *
   * On rev 1 hw, this corresponds to FC_LATENCY_OVR
   * On rev 2 hw, this corresponds to TIMER_MOD_FLOW_CONTROL
   *
   * Field size: 13 bits
   */
  uint16_t fcLatencyOvr;
  /**
   * @brief [rw] FC Latency Timer Override Enable
   *
   * On rev 1 hw, this corresponds to FC_LATENCY_OVR_EN
   * On rev 2 hw, this corresponds to TIMER_MOD_FLOW_CONTROL_EN
   *
   * Field size: 1 bit
   */
  uint8_t fcLatencyOvrEn;
} pciePlconfQStsRReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the VC Transmit Arbitration Register 1 (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_VC_TR_A_R1
 * On rev 2 hw, this corresponds to VC_TX_ARBI_1_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 *
 * @{
 */
typedef struct  pciePlconfVcTrAR1Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] WRR Weight for VC0
   *
   * On rev 1 hw, this corresponds to WRR_VC0
   * On rev 2 hw, this corresponds to WRR_WEIGHT_VC_0
   *
   * Field size: 8 bits
   */
  uint8_t wrrVc0;
  /**
   * @brief [ro] WRR Weight for VC1
   *
   * On rev 1 hw, this corresponds to WRR_VC1
   * On rev 2 hw, this corresponds to WRR_WEIGHT_VC_1
   *
   * Field size: 8 bits
   */
  uint8_t wrrVc1;
  /**
   * @brief [ro] WRR Weight for VC2
   *
   * On rev 1 hw, this corresponds to WRR_VC2
   * On rev 2 hw, this corresponds to WRR_WEIGHT_VC_2
   *
   * Field size: 8 bits
   */
  uint8_t wrrVc2;
  /**
   * @brief [ro] WRR Weight for VC3
   *
   * On rev 1 hw, this corresponds to WRR_VC3
   * On rev 2 hw, this corresponds to WRR_WEIGHT_VC_3
   *
   * Field size: 8 bits
   */
  uint8_t wrrVc3;
} pciePlconfVcTrAR1Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the VC Transmit Arbitration Register 2 (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_VC_TR_A_R2
 * On rev 2 hw, this corresponds to VC_TX_ARBI_2_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 *
 * @{
 */
typedef struct  pciePlconfVcTrAR2Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] WRR Weight for VC4
   *
   * On rev 1 hw, this corresponds to WRR_VC4
   * On rev 2 hw, this corresponds to WRR_WEIGHT_VC_4
   *
   * Field size: 8 bits
   */
  uint8_t wrrVc4;
  /**
   * @brief [ro] WRR Weight for VC5
   *
   * On rev 1 hw, this corresponds to WRR_VC5
   * On rev 2 hw, this corresponds to WRR_WEIGHT_VC_5
   *
   * Field size: 8 bits
   */
  uint8_t wrrVc5;
  /**
   * @brief [ro] WRR Weight for VC6
   *
   * On rev 1 hw, this corresponds to WRR_VC6
   * On rev 2 hw, this corresponds to WRR_WEIGHT_VC_6
   *
   * Field size: 8 bits
   */
  uint8_t wrrVc6;
  /**
   * @brief [ro] WRR Weight for VC7
   *
   * On rev 1 hw, this corresponds to WRR_VC7
   * On rev 2 hw, this corresponds to WRR_WEIGHT_VC_7
   *
   * Field size: 8 bits
   */
  uint8_t wrrVc7;
} pciePlconfVcTrAR2Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the VC0 Posted Receive Queue Control (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_VC0_PR_Q_C
 * On rev 2 hw, this corresponds to VC0_P_RX_Q_CTRL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfVcPrQCReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] VC0 Posted Data Credits
   *
   * On rev 1 hw, this corresponds to P_DCRD
   * On rev 2 hw, this corresponds to VC0_P_DATA_CREDIT
   *
   * Field size: 12 bits
   */
  uint16_t pDcrd;
  /**
   * @brief [ro] VC0 Posted Header Credits
   *
   * On rev 1 hw, this corresponds to P_HCRD
   * On rev 2 hw, this corresponds to VC0_P_HEADER_CREDIT
   *
   * Field size: 8 bits
   */
  uint8_t pHcrd;
  /**
   * @brief [rw] VC0 Scale Posted Data Credits
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to VC0_P_DATA_SCALE
   *
   * Field size: 2 bits
   */
  uint8_t pDataScale;
  /**
   * @brief [rw] VC0 Scale Posted Header Credits
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to VC0_P_HDR_SCALE
   *
   * Field size: 2 bits
   */
  uint8_t pHdrScale;
  /**
   * @brief [rw] VC0 TLP Type Ordering Rules
   *
   * On rev 1 hw, this corresponds to ORDERING_RULES
   * On rev 2 hw, this corresponds to TLP_TYPE_ORDERING_VC0
   *
   * <table>
   * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
   * <tr><td>0x0</td><td>STRICT</td><td>Posted, then Completion, then
   *                                    Non-Posted</td></tr>
   * <tr><td>0x1</td><td>STANDARD</td><td>As per PCIe standard</td></tr>
   * </table>
   *
   * Field size: 1 bit
   */
  uint8_t orderingRules;
    /**
   * @brief [rw] VC0 TLP Type Ordering Rules
   *
   * On rev 1 hw, this corresponds to STRICT_VC_PRIORITY
   * On rev 2 hw, this corresponds to VC_ORDERING_RX_Q
   *
   * <table>
   * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
   * <tr><td>0x0</td><td>ROUND_ROBIN</td></tr>
   * <tr><td>0x1</td><td>STRICT</td><td>Ordering by VC</td></tr>
   * </table>
   *
   * Field size: 1 bit
   */
  uint8_t strictVcPriority;
  /**
   * @brief [rw] VC0 Poster TLP Queue Mode
   *
   * On rev 1 hw, this corresponds to P_QMODE
   * On rev 2 hw, unsupported
   *
   * <table>
   * <tr><th>Action/Value</th><th>Mode</th></tr>
   * <tr><td>Read 0x1</td><td>STORE_AND_FORWARD</td>/tr>
   * <tr><td>Read 0x2</td><td>CUT_THROUGH</td>/tr>
   * <tr><td>Read 0x4</td><td>BYPASS</td>/tr>
   * <tr><td>Others</td><td>Reserved</td>/tr>
   * </table>
   *
   * Field size: 3 bits
   */
  uint8_t pQmode;
} pciePlconfVcPrQCReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief backwards compatibility alias for Vc0 */
typedef pciePlconfVcPrQCReg_t pciePlconfVc0PrQCReg_t;

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the VC0 Non-Posted Receive Queue Control (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_VC0_NPR_Q_C
 * On rev 2 hw, this corresponds to VC0_NP_RX_Q_CTRL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfVcNprQCReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] VC0 Non-Posted Data Credits
   *
   * On rev 1 hw, this corresponds to NP_DCRD
   * On rev 2 hw, this corresponds to VC0_NP_DATA_CREDIT
   *
   * Field size: 12 bits
   */
  uint16_t npDcrd;
  /**
   * @brief [ro] VC0 Non-Posted Header Credits
   *
   * On rev 1 hw, this corresponds to NP_HCRD
   * On rev 2 hw, this corresponds to VC0_NP_HEADER_CREDIT
   *
   * Field size: 8 bits
   */
  uint8_t npHcrd;
  /**
   * @brief [rw] VC0 Scale Non-Posted Data Credits
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to VC0_NP_DATA_SCALE
   *
   * Field size: 2 bits
   */
  uint8_t npDataScale;
  /**
   * @brief [rw] VC0 Scale Non-Posted Header Credits
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to VC0_NP_HDR_SCALE
   *
   * Field size: 2 bits
   */
  uint8_t npHdrScale;
  /**
   * @brief [rw] VC0 Non-Poster TLP Queue Mode
   *
   * On rev 1 hw, this corresponds to NP_QMODE
   * On rev 2 hw, unsupported
   *
   * <table>
   * <tr><th>Action/Value</th><th>Mode</th></tr>
   * <tr><td>Read 0x1</td><td>STORE_AND_FORWARD</td>/tr>
   * <tr><td>Read 0x2</td><td>CUT_THROUGH</td>/tr>
   * <tr><td>Read 0x4</td><td>BYPASS</td>/tr>
   * <tr><td>Others</td><td>Reserved</td>/tr>
   * </table>
   *
   * Field size: 3 bits
   */
  uint8_t npQmode;
} pciePlconfVcNprQCReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief backwards compatibility alias for Vc0 */
typedef pciePlconfVcNprQCReg_t pciePlconfVc0NprQCReg_t;

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the VC0 Completion Receive Queue Control (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_VC0_CR_Q_C
 * On rev 1 hw, this corresponds to VC0_CPL_RX_Q_CTRL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfVcCrQCReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] VC0 Completion Data Credits
   *
   * On rev 1 hw, this corresponds to CPL_DCRD
   * On rev 2 hw, this corresponds to VC0_NP_DATA_CREDIT
   *
   * Field size: 12 bits
   */
  uint16_t cplDcrd;
  /**
   * @brief [ro] VC0 Completion Header Credits
   *
   * On rev 1 hw, this corresponds to CPL_HCRD
   * On rev 2 hw, this corresponds to VC0_NP_HEADER_CREDIT
   *
   * Field size: 8 bits
   */
  uint8_t cplHcrd;
  /**
   * @brief [rw] VC0 Scale CPL Data Credits
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to VC0_CPL_DATA_SCALE
   *
   * Field size: 2 bits
   */
  uint8_t cplDataScale;
  /**
   * @brief [rw] VC0 Scale CPL Header Credits
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to VC0_CPL_HDR_SCALE
   *
   * Field size: 2 bits
   */
  uint8_t cplHdrScale;
  /**
   * @brief [rw] VC0 Completion TLP Queue Mode
   *
   * On rev 1 hw, this corresponds to CPL_QMODE
   * On rev 2 hw, unsupported
   *
   * <table>
   * <tr><th>Action/Value</th><th>Mode</th></tr>
   * <tr><td>Read 0x1</td><td>STORE_AND_FORWARD</td>/tr>
   * <tr><td>Read 0x2</td><td>CUT_THROUGH</td>/tr>
   * <tr><td>Read 0x4</td><td>BYPASS</td>/tr>
   * <tr><td>Others</td><td>Reserved</td>/tr>
   * </table>
   *
   * Field size: 3 bits
   */
  uint8_t cplQmode;
} pciePlconfVcCrQCReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief backwards compatibility alias for Vc0 */
typedef pciePlconfVcCrQCReg_t pciePlconfVc0CrQCReg_t;

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the PHY Status Register (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_PHY_STS_R
 * On rev 2 hw, this corresponds to PHY_STATUS_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 *
 * @{
 */
typedef struct  pciePlconfPhyStsRReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] PHY Status
   *
   * On rev 1 hw, this corresponds to PHY_STS
   * On rev 2 hw, this corresponds to PHY_STATUS
   *
   * Field size: 32 bits
   */
  uint32_t phySts;
} pciePlconfPhyStsRReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the PHY Control Register (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_PHY_CTRL_R
 * On rev 2 hw, this corresponds to PHY_CONTROL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfPhyCtrlRReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] PHY Control
   *
   * On rev 1 hw, this corresponds to PHY_CTRL
   * On rev 2 hw, this corresponds to PHY_CONTROL
   *
   * Field size: 32 bits
   */
  uint32_t phyCtrl;
} pciePlconfPhyCtrlRReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the MSI Controller Address Register
 * (RC-mode MSI receiver)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_MSI_CTRL_ADDRESS
 * On rev 2 hw, this corresponds to MSI_CTRL_ADDR_OFF
 *
 * This register may be used only for root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfMsiCtrlAddressReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI CTRL ADDRESS
   *
   * On rev 1 hw, this corresponds to MSI_CTRL_ADDRESS
   * On rev 2 hw, this corresponds to MSI_CTRL_ADDR
   *
   * Field size: 32 bits
   */
  uint32_t msiCtrlAddress;
} pciePlconfMsiCtrlAddressReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the MSI Controller Upper Address Register
 * (RC-mode MSI receiver)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_MSI_CTRL_UPPER_ADDRESS
 * On rev 2 hw, this corresponds to MSI_CTRL_UPPER_ADDR_OFF
 *
 * This register may be used only for root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfMsiCtrlUpperAddressReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI CTRL UPPER ADDRESS
   *
   * On rev 1 hw, this corresponds to MSI_CTRL_UPPER_ADDRESS
   * On rev 2 hw, this corresponds to MSI_CTRL_UPPER_ADDR
   *
   * Field size: 32 bits
   */
  uint32_t msiCtrlUpperAddress;
} pciePlconfMsiCtrlUpperAddressReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the MSI Controller Interrupt # N(1) Enable Register
 *
 * (RC-mode MSI receiver) with N = MSI data [7:5] and ENABLE[i] = enable MSI vector # i,
 * with i = MSI data [4:0]
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_MSI_CTRL_INT_ENABLE_N
 * On rev 2 hw, this corresponds to MSI_CTRL_INT_0_EN_OFF
 *
 * This register may be used only for root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfMsiCtrlIntEnableReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Status of an enabled bit (vectors) is set upon incoming MSI
   *
   * On rev 1 hw, this corresponds to MSI_CTRL_INT_ENABLE
   * On rev 2 hw, this corresponds to MSI_CTRL_INT_0_EN
   *
   * Field size: 32 bits
   */
  uint32_t msiCtrlIntEnable;
} pciePlconfMsiCtrlIntEnableReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the MSI Controller Interrupt # N(1) Mask Register
 *
 * (RC-mode MSI receiver) with N = MSI data [7:5] and MASK[i] = mask of MSI
 * vector # i, with i = MSI data [4:0]
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_MSI_CTRL_INT_MASK_N
 * On rev 2 hw, this corresponds to MSI_CTRL_INT_0_MASK_OFF
 *
 * This register may be used only for root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfMsiCtrlIntMaskReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Status of a masked bit (vector) triggers no IRQ to MPU when set
   *
   * On rev 1 hw, this corresponds to MSI_CTRL_INT_MASK
   * On rev 2 hw, this corresponds to MSI_CTRL_INT_0_MASK
   *
   * Field size: 32 bits
   */
  uint32_t msiCtrlIntMask;
} pciePlconfMsiCtrlIntMaskReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the MSI Controller Interrupt # N(1) Status Register
 *
 * (RC-mode MSI receiver) with N = MSI data [7:5] and STATUS[i] = status
 * of MSI vector # i, with i = MSI data [4:0]
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_MSI_CTRL_INT_STATUS_N
 * On rev 2 hw, this corresponds to MSI_CTRL_INT_0_STATUS_OFF
 *
 * This register may be used only for root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfMsiCtrlIntStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Status of an enabled bit (vectors) is set upon incoming MSI
   *
   * On rev 1 hw, this corresponds to MSI_CTRL_INT_STATUS
   * On rev 2 hw, this corresponds to MSI_CTRL_INT_0_STATUS
   *
   * Field size: 32 bits
   */
  uint32_t msiCtrlIntStatus;
} pciePlconfMsiCtrlIntStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the MSI Controller General Purpose IO Register
 * (RC-mode MSI receiver)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_MSI_CTRL_GPIO
 * On rev 2 hw, this corresponds to MSI_GPIO_IO_OFF
 *
 * This register may be used only for root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfMsiCtrlGpioReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI CTRL GPIO
   *
   * On rev 1 hw, this corresponds to MSI_CTRL_GPIO
   * On rev 2 hw, this corresponds to MSI_GPIO_REG
   *
   * Field size: 32 bits
   */
  uint32_t msiCtrlGpio;
} pciePlconfMsiCtrlGpioReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the PIPE loopback control register (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_PIPE_LOOPBACK
 * On rev 2 hw, this corresponds to PIPE_LOOPBACK_CONTROL_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfPipeLoopbackReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] PIPE Loopback Enable
   *
   * On rev 1 hw, this corresponds to LOOPBACK_EN
   * On rev 2 hw, this corresponds to PIPE_LOOPBACK
   *
   * Field size: 1 bit
   */
  uint8_t loopbackEn;
} pciePlconfPipeLoopbackReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the DIF Read-Only register Write Enable (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_DBI_RO_WR_EN
 * On rev 2 hw, this corresponds to MISC_CONTROL_1_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfDbiRoWrEnReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Control the writability over DIF of certain configuration
   *
   * Control the writability over DIF of certain configuration fields that
   * are RO over the PCIe wire
   *
   * <table>
   * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
   * <tr><td>0x0</td><td>WRDIS</td><td>
   * RO fields are also RO over DIF; Use for RC mode (Type-1) config
   * to mimic PCIe wire access when using DIF</td></tr>
   * <tr><td>0x1</td><td>WREN</td><td>
   * Some RO fields are writable over DIF</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to CX_DBI_RO_WR_EN
   * On rev 2 hw, this corresponds to DBI_RO_WR_EN
   *
   * Field size: 1 bit
   */
  uint8_t cxDbiRoWrEn;
  /**
   * @brief [rw] Default target a received IO or MEM request with UR/CA/CRS
   *
   * 0: The controller drops all incoming I/O or MEM requests [after
   *    corresponding error reporting] A completion with UR status will be
   *    generated for non-posted requests -
   * 1: The controller forwards all incoming I/O or MEM requests with
   *    UR/CA/CRS status to your application Default value is
   *    DEFAULT_TARGET configuration parameter Note: This register field
   *    is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to DEFAULT_TARGET
   *
   * Field size: 1 bit
   */
  uint8_t defaultTarget;
  /**
   * @brief [rw] UR_CA_MASK_4_TRGT1
   *
   * This field only applies to request TLPs [with UR filtering status] that
   * you have chosen to forward to the application [when you set
   * DEFAULT_TARGET in this register] -
   *
   * '1' the core suppresses error logging, Error Message generation, and
   * CPL generation [for non-posted requests] - You should set this if you
   * have set the Default Target port logic register to '1' Default is
   * CX_MASK_UR_CA_4_TRGT1 configuration parameter
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to UR_CA_MASK_4_TRGT1
   *
   * Field size: 1 bit
   */
   uint8_t urCaMask4Trgt1;
  /**
   * @brief [rw] Enables Simplified Replay Timer [Gen4]
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to SIMPLIFIED_REPLAY_TIMER
   *
   * Field size: 1 bit
   */
   uint8_t simpReplayTimer;
  /**
   * @brief [rw] When ARI is enabled, enables use of the device ID
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to ARI_DEVICE_NUMBER
   *
   * Field size: 1 bit
   */
   uint8_t ariDevNumber;
} pciePlconfDbiRoWrEnReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the AXI Slave Error Response Register (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_AXI_SLV_ERR_RESP
 * On rev 2 hw, this corresponds to AMBA_ERROR_RESPONSE_DEFAULT_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfAxiSlvErrRespReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Global Slave Error Response Mapping
   *
   * On rev 1 hw, this corresponds to SLAVE_ERR_MAP
   * On rev 2 hw, this corresponds to AMBA_ERROR_RESPONSE_GLOBAL
   *
   * Field size: 1 bit
   */
  uint8_t slaveErrMap;
  /**
   * @brief [rw] DIF Slave Error Response Mapping
   *
   * On rev 1 hw, this corresponds to DBI_ERR_MAP
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t dbiErrMap;
  /**
   * @brief [rw] Vendor ID Non-existent Slave Error Response Mapping
   *
   * On rev 1 hw, this corresponds to NO_VID_ERR_MAP
   * On rev 2 hw, this corresponds to AMBA_ERROR_RESPONSE_VENDORID
   *
   * Field size: 1 bit
   */
  uint8_t noVidErrMap;
  /**
   * @brief [rw] Graceful Reset and Link Timeout Slave Error Response Mapping
   *
   * On rev 1 hw, this corresponds to RESET_TIMEOUT_ERR_MAP
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t resetTimeoutErrMap;
  /**
   * @brief [rw] CRS Slave Error Response Mapping
   *
   * CRS Slave Error Response Mapping Determines the AXI slave
   * response for CRS completions AHB: - always returns OKAY AXI: -
   * 00: OKAY
   * 01: OKAY with all FFFF_FFFF data for all CRS completions
   * 10: OKAY with FFFF_0001 data for CRS completions to vendor ID
   *     read requests, OKAY with FFFF_FFFF data for all other CRS
   *     completions
   * 11: SLVERR/DECERR [the AXI_ERROR_RESPONSE_MAP field
   *     determines the PCIe-to-AXI Slave error response mapping]
   *
   * This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to AMBA_ERROR_RESPONSE_CRS
   *
   * Field size: 2 bit
   */
  uint8_t errorResponseCrs;
  /**
   * @brief [rw] AXI Slave Response Error Map
   *
   * AXI Slave Response Error Map Allows you to selectively map the
   * errors received from the PCIe completion [for non-posted requests]
   * to the AXI slave responses, slv_rresp or slv_bresp The
   * recommended setting is SLVERR CRS is always mapped to OKAY -
   * [bit 0] 0: UR [unsupported request] -> DECERR
   *         1: UR [unsupported request] -> SLVERR
   * [bit 1] 0: CRS [configuration retry status] -> DECERR
   *         1: CRS [configuration retry status] -> SLVERR
   * [bit 2] 0: CA [completer abort] -> DECERR
   *         1: CA [completer abort] -> SLVERR
   * [bit 3]: Reserved
   * [bit 4]: Reserved
   * [bit 5]: 0: Completion Timeout -> DECERR
   *          1: Completion Timeout -> SLVERR
   * The AXI bridge internally drops
   * [processes internally but not passed to your application] a
   * completion that has been marked by the Rx filter as UC or MLF, and
   * does not pass its status directly down to the slave interface It waits
   * for a timeout and then signals "Completion Timeout" to the slave
   * interface The controller sets the AXI slave read databus to 0xFFFF
   * for all error responses
   *
   * Note: This register field is sticky
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to AMBA_ERROR_RESPONSE_MAP
   *
   * Field size: 6 bits
   */
  uint8_t errorResponseMap;
} pciePlconfAxiSlvErrRespReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Link Down AXI Slave Timeout Register (Sticky)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_AXI_SLV_TIMEOUT
 * On rev 2 hw, this corresponds to AMBA_LINK_TIMEOUT_OFF
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfAxiSlvTimeoutReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Timeout Value (ms)
   *
   * On rev 1 hw, this corresponds to TIMEOUT_VALUE
   *
   * Field size: 8 bits
   */
  uint8_t timeoutValue;
  /**
   * @brief [rw] Enable flush
   *
   * On rev 1 hw, this corresponds to FLUSH_EN
   *
   * Field size: 1 bit
   */
  uint8_t flushEn;
} pciePlconfAxiSlvTimeoutReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Viewport Register
 *
 * makes the registers of the corresponding iATU region accessible
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_INDEX
 * On rev 2 hw: the index selection emulated in SW (all windows are available in HW all the time)
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfIatuIndexReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Region Direction
   *
   * <table>
   * <tr><th>Value</th><th>Mode</th></tr>
   * <tr><td>0x0</td><td>OUTBOUND</td></tr>
   * <tr><td>0x1</td><td>INBOUND</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to REGION_DIRECTION
   * On rev 2 hw, this is virtual sw register to demux 16 inbound/outbound regions
   *
   * Field size: 1 bit
   */
  uint8_t regionDirection;
  /**
   * @brief [rw] Region Index
   *
   * On rev 1 hw, this corresponds to REGION_INDEX
   * On rev 2 hw, this is virtual sw register to demux 16 inbound/outbound regions
   *
   * Outbound region, from 0 to 15.
   * Inbound region, from 0 to 3 (rev 1) or 0 to 15 (rev 2)
   *
   * Field size: 4 bits
   */
  uint8_t regionIndex;
} pciePlconfIatuIndexReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Region Control 1 Register
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_REG_CTRL_1
 * On rev 2 hw, this corresponds to IATU_REGION_CTRL_1_OFF_OUTBOUND and
 *              IATU_REGION_CTRL_1_OFF_INBOUND
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfIatuRegCtrl1Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] type
   *
   * On rev 1 hw, this corresponds to TYPE
   * On rev 2 hw, this corresponds to TYPE
   *
   * Outbound: TYPE applied to outgoing TLP with matching addess
   * Inbound: TYPE-match criteria
   *
   * Field size: 5 bits
   */
  uint8_t type;
  /**
   * @brief [rw] TC
   *
   * On rev 1 hw, this corresponds to TC
   * On rev 2 hw, this corresponds to TC
   *
   * Outbound: TC applied to outgoing TLP with matching addess
   * Inbound: TC-match criteria (if TC_match_enable=1)
   *
   * Field size: 3 bits
   */
  uint8_t tc;
  /**
   * @brief [rw] TD
   *
   * On rev 1 hw, this corresponds to TD
   * On rev 2 hw, this corresponds to TD
   *
   * Outbound: TD applied to outgoing TLP with matching addess
   * Inbound: TD-match criteria (if TD_match_enable=1)
   *
   * Field size: 1 bit
   */
  uint8_t td;
  /**
   * @brief [rw] ATTR
   *
   * On rev 1 hw, this corresponds to ATTR
   * On rev 2 hw, this corresponds to ATTR
   *
   * Outbound: ATTR applied to outgoing TLP with matching addess
   * Inbound: ATTR-match criteria (if ATTR_match_enable=1)
   *
   * Field size: 2 bits
   */
  uint8_t attr;
  /**
   * @brief [rw] AT
   *
   * On rev 1 hw, this corresponds to AT
   * On rev 2 hw, unsupported
   *
   * Outbound: AT applied to outgoing TLP with matching addess
   * Inbound: AT-match criteria for matching TLP (if AT_match_enable=1)
   *
   * Field size: 2 bits
   */
  uint8_t at;
  /**
   * @brief [rw] Increase the maximum ATU Region size
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INCREASE_REGION_SIZE
   *
   * Outbound: F.N; applied to outgoing TLP (RID) with RW 0x0 matching addess
   * Inbound: F.N.-match criteria for incoming TLP
   *          (if Function_Number_match_enable=1)
   *
   * Field size: rev 1: 5 bits; rev 2: 3 bits
   */
  uint8_t increaseRegionSize;
  /**
   * @brief [rw] function number
   *
   * On rev 1 hw, this corresponds to FUNCTION_NUMBER
   * On rev 2 hw, this corresponds to CTRL_1_FUNC_NUM
   *
   * Outbound: F.N; applied to outgoing TLP (RID) with RW 0x0 matching addess
   * Inbound: F.N.-match criteria for incoming TLP
   *          (if Function_Number_match_enable=1)
   *
   * Field size: rev 1: 5 bits; rev 2: 3 bits
   */
  uint8_t functionNumber;
} pciePlconfIatuRegCtrl1Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Region Control 2 Register
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_REG_CTRL_2
 * On rev 2 hw, this corresponds to IATU_REGION_CTRL_2_OFF_OUTBOUND and
 *              IATU_REGION_CTRL_2_OFF_INBOUND
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfIatuRegCtrl2Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MESSAGECODE
   *
   * On rev 1 hw, this corresponds to MESSAGECODE
   * On rev 2 hw, this corresponds to MSG_CODE
   *
   * Outbound: MessageCode applied to outgoing message RW 0x0 TLP with
   *           matching addess
   * Inbound: MessageCode-match criteria for infoming message TLP
   *          (if Message_Code_match_enable=1)
   *
   * Field size: 8 bits
   */
  uint8_t messagecode;
  /**
   * @brief [rw] BAR_NUMBER
   *
   * On rev 1 hw, this corresponds to BAR_NUMBER
   * On rev 2 hw, this corresponds to incoming BAR_NUM (not used for outgoing)
   *
   * BAR number for mayching with incoming MEM, I/O TLP RW 0x0
   * (if Match_Mode = 1)
   *
   * <table>
   * <tr><th>Value</th><th>Mode</th></tr>
   * <tr><td>0x0</td><td>BAR0</td></tr>
   * <tr><td>0x1</td><td>BAR1</td></tr>
   * <tr><td>0x2</td><td>BAR2</td></tr>
   * <tr><td>0x3</td><td>BAR3</td></tr>
   * <tr><td>0x4</td><td>BAR4</td></tr>
   * <tr><td>0x5</td><td>BAR5</td></tr>
   * <tr><td>0x6</td><td>ROM</td></tr>
   * </table>
   *
   * Field size: 3 bits
   */
  uint8_t barNumber;
  /**
   * @brief [rw] TAG
   *
   * The substituted TAG field [byte 6] in the outgoing TLP header
   * when TAG_SUBSTITUTE_EN is set
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to TAG for outbound only
   *
   * Field size: 8 bits
   */
  uint8_t tag;
  /**
   * @brief [rw] 
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to MSG_TYPE_MATCH_MODE (inbound)
   *
   * Field size: 1 bit
   */
  uint8_t msgTypeMatchMode;
  /**
   * @brief [rw] Enable TC match criteria on inbound TLP
   *
   * On rev 1 hw, this corresponds to TC_MATCH_ENABLE
   * On rev 2 hw, this corresponds to TC_MATCH_EN (inbound)
   *
   * Field size: 1 bit
   */
  uint8_t tcMatchEnable;
  /**
   * @brief [rw] Enable TD match criteria on inbound TLP
   *
   * On rev 1 hw, this corresponds to TD_MATCH_ENABLE
   * On rev 2 hw, this corresponds to TD_MATCH_EN (inbound)
   *
   * Field size: 1 bit
   */
  uint8_t tdMatchEnable;
  /**
   * @brief [rw] Enable ATTR match criteria on inbound TLP
   *
   * On rev 1 hw, this corresponds to ATTR_MATCH_ENABLE
   * On rev 2 hw, this corresponds to ATTR_MATCH_EN (inbound)
   *
   * Field size: 1 bit
   */
  uint8_t attrMatchEnable;
  /**
   * @brief [rw] Enable AT match criteria on inbound TLP
   *
   * On rev 1 hw, this corresponds to AT_MATCH_ENABLE
   * On rev 2 hw, unsupported
   *
   * ATS NOT SUPPORTED: DO NOT USE
   *
   * Field size: 1 bit
   */
  uint8_t atMatchEnable;
  /**
   * @brief [rw] TAG Substitute Enable
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to TAG_SUBSTITUTE_EN
   *
   * Field size: 1 bit
   */
  uint8_t tagSubstEn;
  /**
   * @brief [rw] function number match enable
   *
   * On rev 1 hw, this corresponds to FUNCTION_NUMBER_MATCH_ENABLE
   * On rev 2 hw, this corresponds to FUNC_BYPASS(outbound) or FUNC_NUM_MATCH_EN (inbound)
   *
   * Outbound: Function Number Translation Bypass
   * Inbound: Enable Function Number match criteria
   *
   * Field size: 1 bit
   */
  uint8_t functionNumberMatchEnable;
  /**
   * @brief [rw] VIRTUAL FUNCTIONS NOT IMPLEMENTED
   *
   * On rev 1 hw, this corresponds to VIRTUAL_FUNCTION_NUMBER_MATCH_ENABLE
   * On rev 2 hw, unsupported
   *
   * Field size: 1 bit
   */
  uint8_t virtualFunctionNumberMatchEnable;
  /**
   * @brief [rw] Serialize Non-Posted Requests
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to SNP (outbound)
   *
   * Field size: 1 bit
   */
  uint8_t SNP;
  /**
   * @brief [rw] Enable MessageCode match criteria on inbound TLP
   *
   * On rev 1 hw, this corresponds to MESSAGE_CODE_MATCH_ENABLE
   * On rev 2 hw, this corresponds to MSG_CODE_MATCH_EN(inbound)
   *
   * Field size: 1 bit
   */
  uint8_t messageCodeMatchEnable;
  /**
   * @brief [rw] Inhibit TLP Payload Data for TLP's in Matched Region
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to INHIBIT_PAYLOAD
   *
   * Field size: 1 bit
   */
  uint8_t inhibitPayload;
  /**
   * @brief [rw] Header Substitute Enable (outbound)
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to HEADER_SUBSTITUTE_EN (outbound)
   *
   * Field size: 1 bit
   */
  uint8_t headerSubstEn;
  /**
   * @brief [rw] Single Address Location Translate Enable (inbound)
   *
   * On rev 1 hw, unsupported
   * On rev 2 hw, this corresponds to SINGLE_ADDR_LOC_TRANS_EN (inbound)
   *
   * Field size: 1 bit
   */
  uint8_t singleAddrLocTransEn;
  /**
   * @brief [rw] Override HW-generated completion status when responding inbound TLP
   *
   * On rev 1 hw, this corresponds to RESPONSE_CODE
   * On rev 2 hw, this corresponds to RESPONSE_CODE (inbound)
   *
   * <table>
   * <tr><th>Value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>No override, use HW-generated CS</td></tr>
   * <tr><td>0x1</td><td>Unsupported Request: CS= 3'b001</td></tr>
   * <tr><td>0x2</td><td>Completer Abort: CS= 3'b100</td></tr>
   * </table>
   *
   * Field size: 2 bits
   */
  uint8_t responseCode;
  /**
   * @brief [rw] fuzzy type match mode
   *
   * On rev 1 hw, this corresponds to FUZZY_TYPE_MATCH_MODE
   * On rev 2 hw, this corresponds to FUZZY_TYPE_MATCH_CODE (inbound)
   *
   * Outbound: DMA Bypass Mode RW 0x0
   * Inbound: Relax matching on inbound TLP TYPE:
   * CfgRd0 == CfgRd1
   * CfgWr0 == CfgWr1
   * MRd == MRdLk
   * routing field of Msg/MsgD ignored
   *
   * Field size: 1 bit
   */
  uint8_t fuzzyTypeMatchMode;
  /**
   * @brief [rw] Enable the shifting of CFG CID (BDF),
   *
   * On rev 1 hw, this corresponds to CFG_SHIFT_MODE
   * On rev 2 hw, this corresponds to CFG_SHIFT_MODE
   *
   * Incoming and outgoing TLP;
   * CFG get mapped to a contiguous 2**28 = * 256 MByte address space
   * Untranslated CID = CFG_DW#3[31:16]
   * Shifted CID = CFG_DW#3[27:12]
   *
   * Field size: 1 bit
   */
  uint8_t cfgShiftMode;
  /**
   * @brief [rw] Redefine match criteria as outside the defined range
   *
   * On rev 1 hw, this corresponds to INVERT_MODE
   * On rev 2 hw, this corresponds to INVERT_MODE
   *
   * (instead of inside)
   *
   * Field size: 1 bit
   */
  uint8_t invertMode;
  /**
   * @brief [rw] Sets inbound TLP match mode
   *
   * On rev 1 hw, this corresponds to MATCH_MODE
   * On rev 2 hw, this corresponds to MATCH_MODE (inbound)
   *
   * Depending on TYPE
   * 0x0: MEM,I/O: Address Match: as per region base & limit registers;
   *      CFG0: Routing ID Match: Completer ID (BDF) + reg
   *            address matches base & limit-defined region;
   *      MSG[D]: Address Match: as per region base & limit registers
   * 0x1: MEM,I/O: BAR match: as defined in BAR_number field;
   *      CFG0: Accept mode: Completer ID (BDF) is ignored;
   *      MSG[D]: VendorID match: VendorID = upper_base[15:0] +
   *              VendorDefined = lower_base/limit
   *
   * Field size: 1 bit
   */
  uint8_t matchMode;
  /**
   * @brief [rw] Enable AT for this region
   *
   * On rev 1 hw, this corresponds to REGION_ENABLE
   * On rev 2 hw, this corresponds to REGION_EN
   *
   * Field size: 1 bit
   */
  uint8_t regionEnable;
} pciePlconfIatuRegCtrl2Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Region Lower Base Address Register
 * (2**12 = 4kbyte - aligned)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_REG_LOWER_BASE
 * On rev 2 hw, this corresponds to IATU_LWR_BASE_ADDR_OFF_OUTBOUND and
 *              IATU_LWR_BASE_ADDR_OFF_INBOUND
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */

typedef struct  pciePlconfIatuRegLowerBaseReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Lower Base Address (read-write part)
   *
   * On rev 1 hw, this corresponds to IATU_REG_LOWER_BASE
   * On rev 2 hw, this corresponds to LWR_BASE_RW
   *
   * Note: On rev 1 @ref iatuRegLowerBase is >> 12 version of address, with 0 LSBs cut off
   * Note: On rev 2 @ref iatuRegLowerBase is >> 16 version of address, with 0 LSBs cut off
   *
   * Field size: 20 bits (rev 1) or 16 bits (rev 2)
   */
  uint32_t iatuRegLowerBase;
  /**
   * @brief [ro] Lower Base Address (read-only part)
   *
   * On rev 1 hw, this corresponds to ZERO
   * On rev 2 hw, this corresponds to LWR_BASE_HW
   *
   * This portion is always 0 to enforce 4K alignment (rev 1) or 64K (rev 2)
   *
   * Field size: 12 bits (rev 1) or 16 bits (rev 2)
   */
  uint16_t zero;
} pciePlconfIatuRegLowerBaseReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Region Upper Base Address Register
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_REG_UPPER_BASE
 * On rev 2 hw, this corresponds to IATU_UPPER_BASE_ADDR_OFF_OUTBOUND and
 *              IATU_LIMIT_ADDR_OFF_INBOUND
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfIatuRegUpperBaseReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Upper Base Address
   *
   * On rev 1 hw, this corresponds to IATU_REG_UPPER_BASE
   * On rev 2 hw, this corresponds to UPPER_BASE_RW
   *
   * Field size: 32 bits
   */
  uint32_t iatuRegUpperBase;
} pciePlconfIatuRegUpperBaseReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Region Limit Address Register
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_REG_LIMIT
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfIatuRegLimitReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Region limit address
   *
   * On rev 1 hw, this corresponds to IATU_REG_LIMIT
   * On rev 2 hw, this corresponds to LIMIT_ADDR_RW
   *
   * Note: On rev 1 @ref iatuRegLimit is >> 12 version of address, with 0 LSBs cut off
   * Note: On rev 2 @ref iatuRegLimit is >> 16 version of address, with 0 LSBs cut off
   *
   * Field size: 20 bits (rev 1) or 16 bits (rev 2)
   */
  uint32_t iatuRegLimit;
  /**
   * @brief [ro] Region limit address (mask)
   *
   * On rev 1 hw, this corresponds to ONES
   * On rev 2 hw, this corresponds to LIMIT_ADDR_HW
   *
   * This portion is always all 1s to enforce 4K alignment (rev 1) or 64K (rev 2)
   *
   * Field size: 12 bits (rev 1) or 16 bits (rev 2)
   */
  uint16_t ones;
} pciePlconfIatuRegLimitReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Region Lower Target Address Register
 * (2**12 = 4kbyte - aligned)
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_REG_LOWER_TARGET
 * On rev 2 hw, this corresponds to IATU_LWR_TARGET_ADDR_OFF_OUTBOUND and
 *              IATU_LWR_TARGET_ADDR_OFF_INBOUND
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */

typedef struct  pciePlconfIatuRegLowerTargetReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Lower Target Address (read-write part)
   *
   * On rev 1 hw, this corresponds to IATU_REG_LOWER_TARGET
   * On rev 2 hw, this corresponds to LWR_TARGET_RW_OUTBOUND and LWR_TARGET_RW
   *
   * Note: On rev 1 @ref iatuRegLowerTarget is >> 12 version of address, with 0 LSBs cut off
   * Note: On rev 2 @ref iatuRegLowerTarget is >> 16 version of address, with 0 LSBs cut off
   *
   * Field size: 20 bits (rev 1) or 16 bits (rev 2)
   */
  uint32_t iatuRegLowerTarget;
  /**
   * @brief [ro] Lower Target Address (read-only part)
   *
   * On rev 1 hw, this corresponds to ZERO
   * On rev 2 hw, this corresponds to LWR_TARGET_RW_OUTBOUND (low 16 bits) and
   *              LWR_TARGET_HW
   *
   * This portion is always 0 to enforce 4K alignment (rev 1) or 64K (rev 2)
   *
   * Field size: 12 bits (rev 1) or 16 bits (rev 2)
   */
  uint16_t zero;
} pciePlconfIatuRegLowerTargetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Region Upper Target Address Register
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_REG_UPPER_TARGET
 * On rev 2 hw, this corresponds to IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND and
 *              IATU_UPPER_TARGET_ADDR_OFF_INBOUND
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct  pciePlconfIatuRegUpperTargetReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Upper Target Address
   *
   * On rev 1 hw, this corresponds to ATU_REG_UPPER_TARGET
   * On rev 2 hw, this corresponds to UPPER_TARGET_RW
   *
   * Field size: 32 bits
   */
  uint8_t iatuRegUpperTarget;
} pciePlconfIatuRegUpperTargetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the iATU Region Control 3 Register
 *
 * On rev 0 hw: unavailable
 * On rev 1 hw, this corresponds to PCIECTRL_PL_IATU_REG_CTRL_3
 * On rev 2 hw: unavailable
 *
 * VIRTUAL FUNCTIONS NOT IMPLEMENTED: NOT USED
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 *
 * @{
 */
typedef struct  pciePlconfIatuRegCtrl3Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] IATU CTRL 3
   *
   * On rev 1 hw, this corresponds to IATU_REG_CTRL_3
   *
   * This register is not used as virtual functions are not supported.  It is
   * always 0.
   *
   * Field size: 32 bits
   */
  uint32_t iatuRegCtrl3;
} pciePlconfIatuRegCtrl3Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF Revision register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_REVISION)
 * This register is completely RO.
 *
 * This register is very similar to hw rev 0's @ref pciePidReg_t
 *
 * @{
 */
typedef struct pcieTiConfRevisionReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  uint8_t yMinor; /**< @brief [ro] minor rev - 6 bits*/
  uint8_t custom; /**< @brief [ro] customer special rev - 2 bits */
  uint8_t xMajor; /**< @brief [ro] major rev - 3 bits*/
  uint8_t rRtl;   /**< @brief [ro] RTL rev - 5 bits */
  uint8_t func;   /**< @brief [ro] function code - 12 bits */
  uint8_t scheme; /**< @brief [ro] scheme - 2 bits */
  uint8_t bu;     /**< @brief [ro] business unit - 2 bits */
} pcieTiConfRevisionReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF Sys Config Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_SYSCONFIG)
 *
 * @{
 */
typedef struct pcieTiConfSysConfigReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] PM mode of local target (slave)
   *
   * PM mode of local target (slave); Target shall be capable
   * RW 0x2 of handling read/write transaction as long as it is
   * out of IDLE state.
   *
   * <table>
   * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
   * <tr><td>0x0</td><td>Force-idle</td><td>
   * local target's idle state follows (acknowledges) the system's idle
   * requests unconditionally, regardless of the IP module's internal
   * requirements.
   * </tr><tr><td>0x1</td><td>No-idle</td><td>
   * local target never enters idle state.
   * </tr><tr><td>0x2</td><td>Smart-idl</td><td>
   * local target's idle state eventually follows (acknowledges) the
   * system's idle requests, depending on the IP module's internal
   * requirements.  Module shall not generate (IRQ- or DMA-request-related)
   * wakeup events.
   * </tr><tr><td>0x3</td><td>Smart-idle wakeup-capable</td><td>
   * local target's idle state eventually follows (acknowledges) the
   * system's idle requests, depending on the IP module's internal
   * requirements. IP module may generate (IRQ- or DMArequest-
   * related) wakeup events when in idle state.
   * </td></tr></table>
   *
   * On rev 1 hw, this corresponds to IDLEMODE
   *
   * Field size: 2 bits
   */
  uint8_t idlemode;
  /**
   * @brief [rw] number of fast training sequences
   *
   * PM mode of local initiator (master); Initiator may generate read/write
   * transaction as long as it is out of STANDBY state.
   *
   * <table>
   * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
   * <tr><td>0x0</td><td>Force-standby</td><td>
   * Initiator is unconditionally placed in standby state.
   * </tr><tr><td>0x1</td><td>No-standby</td><td>
   * initiator is unconditionally placed out of standby state.
   * </tr><tr><td>0x2</td><td>Smart-standby</td><td>
   * initiator's standby state depends on internal conditions, i.e. the
   * module's functional requirements. Asynchronous wakeup events
   * cannot be generated.
   * </tr><tr><td>0x3</td><td>Smart-standby, wakeup-capable</td><td>
   * initiator's standby state depends on internal conditions, ie the
   * module's functional requirements. Asynchronous wakeup events can
   * be generated.
   * </td></tr></table>
   *
   * On rev 1 hw, this corresponds to STANDBYMODE
   *
   * Field size: 2 bits
   */
  uint8_t standbymode;
  /**
   * @brief [rw] no-snoop to coherent mapping
   *
   * Allows the no-snoop (NS) attribute of inbound PCIe TLPs to be passed
   * to SoC system bus (AXI) master as a 'coherent' inband flag.
   *
   * <table>
   * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
   * <tr><td>0x0</td><td>DIS</td><td>AXI not coherent</td></tr>
   * <tr><td>0x1</td><td>EN</td><td>
   * AXI coherent = not(PCIE "NS") i.e. cache-coherence is preserved
   * </td></tr></table>
   *
   * On rev 1 hw, this corresponds to MCOHERENT_EN
   *
   * Field size: 1 bit
   */
  uint8_t mcoherentEn;
} pcieTiConfSysConfigReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ EOI Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQ_EOI)
 * @{
 */
typedef struct pcieTiConfIrqEoiReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] IRQ Line Number to EOI
   *
   * Write the IRQ line number to apply SW EOI to it.
   * Write 0x0: SW EOI on main interrupt line
   * Write 0x1: SW EOI on message-signalled (MSI) interrupt line
   *
   * Read always returns zeros
   *
   * On rev 1 hw, this corresponds to LINE_NUMBER
   *
   * Field size: 4 bits
   */
  uint8_t lineNumber;
} pcieTiConfIrqEoiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ Main common spec
 *
 * This structure is used for reading/writing 4 different registers:
 * @ref pcieTiConfIrqStatusRawMainReg_t
 * @ref pcieTiConfIrqStatusMainReg_t
 * @ref pcieTiConfIrqEnableSetMainReg_t
 * @ref pcieTiConfIrqEnableClrMainReg_t.
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw.
 *
 * @{
 */
typedef struct pcieTiConfIrqMain_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  uint8_t errSys;      /**< @brief System Error IRQ */
  uint8_t errFatal;    /**< @brief Fatal Error message received IRQ */
  uint8_t errNonfatal; /**< @brief Non-Fatal Error message received IRQ */
  uint8_t errCor;      /**< @brief Correctable Error message received IRQ */
  uint8_t errAxi;      /**< @brief AXI tag lookup fatal Error IRQ */
  uint8_t errEcrc;     /**< @brief ECRC Error IRQ */
  uint8_t pmeTurnOff;  /**< @brief Power Management Event Turn-Off message received IRQ */
  uint8_t pmeToAck;    /**< @brief Power Management Event Turn-Off Ack message IRQ */
  uint8_t pmPme;       /**< @brief PM Power Management Event message received IRQ */
  uint8_t linkReqRst;  /**< @brief Link Request Reset IRQ */
  uint8_t linkUpEvt;   /**< @brief Link-up state change IRQ */
  uint8_t cfgBmeEvt;   /**< @brief CFG "Bus Master Enable" change IRQ */
  uint8_t cfgMseEvt;   /**< @brief CFG "Memory Space Enable" change IRQ */
} pcieTiConfIrqMain_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ Status Raw main Register
 *
 * Raw status of 'main' interrupt requests; Set even if event is not enabled.
 * Write 1 to set the (raw) status, mostly for debug (regular status also gets set).
 *
 * For each IRQ in @ref pcieTiConfIrqMain_t
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Triggers IRQ Event by software</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event not pending</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is pending</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQSTATUS_RAW_MAIN)
 *
 */
typedef pcieTiConfIrqMain_t pcieTiConfIrqStatusRawMainReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ Status Main register
 *
 * Regular status of 'main' interrupt requests; Set only when enabled.
 * Write 1 to clear after interrupt has been serviced
 * (raw status also gets cleared).
 *
 * For each IRQ in @ref pcieTiConfIrqMain_t
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Clear pending IRQ Event, if any</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event not pending</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is pending</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQSTATUS_MAIN)
 *
 * @{
 */
typedef pcieTiConfIrqMain_t pcieTiConfIrqStatusMainReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ Enable Set register
 *
 * Enable of 'main' interrupt requests; Write 1 to set (ie to enable
 * interrupt). Readout is the same as corresponding _CLR register.
 *
 * For each IRQ in @ref pcieTiConfIrqMain_t
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Enable event</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event is disabled</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is enabled</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQSTATUS_MAIN)
 *
 * @{
 */
typedef pcieTiConfIrqMain_t pcieTiConfIrqEnableSetMainReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ Enable Clear register
 *
 * Enable of 'main' interrupt requests; Write 1 to clear
 * (ie to disable interrupt). Readout is the same
 * as corresponding _SET register.
 *
 * For each IRQ in @ref pcieTiConfIrqMain_t
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Disable event</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event is disabled</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is enabled</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQENABLE_CLR_MAIN)
 *
 * @{
 */
typedef pcieTiConfIrqMain_t pcieTiConfIrqEnableClrMainReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ MSI common spec
 *
 * This structure is used for reading/writing 4 different registers:
 * @ref pcieTiConfIrqStatusRawMsiReg_t
 * @ref pcieTiConfIrqStatusMsiReg_t
 * @ref pcieTiConfIrqEnableSetMsiReg_t
 * @ref pcieTiConfIrqEnableClrMsiReg_t.
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw.
 *
 * @{
 */
typedef struct pcieTiConfIrqMsi_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  uint8_t inta; /**< @brief INTA IRQ */
  uint8_t intb; /**< @brief INTB IRQ */
  uint8_t intc; /**< @brief INTC IRQ */
  uint8_t intd; /**< @brief INTD IRQ */
  uint8_t msi;  /**< @brief Message Signaled Interrupt (MSI) IRQ */
} pcieTiConfIrqMsi_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ Status Raw MSI Register
 *
 * Raw status of legacy and MSI interrupt requests; Set even if
 * event is not enabled. Write 1 to set the (raw) status, mostly
 * for debug (regular status also gets set).
 *
 * For each IRQ in @ref pcieTiConfIrqMsi_t
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Triggers IRQ Event by software</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event not pending</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is pending</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQSTATUS_RAW_MSI)
 *
 * @{
 */
typedef pcieTiConfIrqMsi_t pcieTiConfIrqStatusRawMsiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ Status MSI register
 *
 * Regular status of legacy and MSI interrupt requests; Set only
 * when enabled. Write 1 to clear after interrupt has been serviced
 * (raw status also gets cleared). HW-generated events are self-clearing.
 *
 * For each IRQ in @ref pcieTiConfIrqMsi_t
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Clear pending IRQ Event, if any</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event not pending</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is pending</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQSTATUS_MSI)
 *
 * @{
 */
typedef pcieTiConfIrqMsi_t pcieTiConfIrqStatusMsiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ MSI Enable Set register
 *
 * Enable of legacy and MSI interrupt requests; Write 1 to set (ie to
 * enable interrupt). Readout is the same as corresponding _CLR register.
 *
 * For each IRQ in @ref pcieTiConfIrqMsi_t
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Enable event</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event is disabled</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is enabled</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQENABLE_SET_MSI)
 *
 * @{
 */
typedef pcieTiConfIrqMsi_t pcieTiConfIrqEnableSetMsiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF IRQ MSI Enable Clear register
 *
 * Enable of legacy and MSI interrupt requests; Write 1 to clear
 * (ie to disable interrupt).  Readout is the same as corresponding
 * _SET register.
 *
 * For each IRQ in @ref pcieTiConfIrqMsi_t
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Disable event</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event is disabled</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is enabled</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQENABLE_CLR_MSI)
 *
 * @{
 */
typedef pcieTiConfIrqMsi_t pcieTiConfIrqEnableClrMsiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF Device Type register
 *
 * Sets the Dual-Mode device's type
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_DEVICE_TYPE)
 *
 * @{
 */
typedef struct pcieTiConfDeviceTypeReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Device Type
   *
   * PCIe device type including the contents of the PCI config
   * space (Type-0 for EP, Type-1 for RC); Apply fundamental
   * reset after change; Do not change during core operation.
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>PCIe endpoint (EP)</td></tr>
   * <tr><td>0x1</td><td>Legacy PCIe endpoint (LEG_EP)</td></tr>
   * <tr><td>0x4</td><td>Root Complex (RC)</td></tr>
   * <tr><td>Other values</td><td>Reserved</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to LINE_NUMBER
   *
   * Field size: 4 bits
   */
  uint8_t type;
} pcieTiConfDeviceTypeReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF Device Command register
 *
 * Device command (startup control and status);
 * WARNING: cleared by all reset conditions, including fundamental reset
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_DEVICE_CMD)
 *
 * @{
 */
typedef struct pcieTiConfDeviceCmdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] LTSSM state/substate implementation-specific, for
   * debug (@ref pcieLtssmState_e)
   *
   * <table>
   * <tr><th>read value</th><th>description</th></tr>
   * <tr><td>0x00</td><td>DETECT_QUIET</td></tr>
   * <tr><td>0x01</td><td>DETECT_ACT</td></tr>
   * <tr><td>0x02</td><td>POLL_ACTIVE</td></tr>
   * <tr><td>0x03</td><td>POLL_COMPLIANCE</td></tr>
   * <tr><td>0x04</td><td>POLL_CONFIG</td></tr>
   * <tr><td>0x05</td><td>PRE_DETECT_QUIET</td></tr>
   * <tr><td>0x06</td><td>DETECT_WAIT</td></tr>
   * <tr><td>0x07</td><td>CFG_LINKWD_START</td></tr>
   * <tr><td>0x08</td><td>CFG_LINKWD_ACEPT</td></tr>
   * <tr><td>0x09</td><td>CFG_LANENUM_WAIT</td></tr>
   * <tr><td>0x0A</td><td>CFG_LANENUM_ACEPT</td></tr>
   * <tr><td>0x0B</td><td>CFG_COMPLETE</td></tr>
   * <tr><td>0x0C</td><td>CFG_IDLE</td></tr>
   * <tr><td>0x0D</td><td>RCVRY_LOCK</td></tr>
   * <tr><td>0x0E</td><td>RCVRY_SPEED</td></tr>
   * <tr><td>0x0F</td><td>RCVRY_RCVRCFG</td></tr>
   * <tr><td>0x10</td><td>RCVRY_IDLE</td></tr>
   * <tr><td>0x11</td><td>L0</td></tr>
   * <tr><td>0x12</td><td>L0S</td></tr>
   * <tr><td>0x13</td><td>L123_SEND_EIDLE</td></tr>
   * <tr><td>0x14</td><td>L1_IDLE</td></tr>
   * <tr><td>0x15</td><td>L2_IDLE</td></tr>
   * <tr><td>0x16</td><td>L2_WAKE</td></tr>
   * <tr><td>0x17</td><td>DISABLED_ENTRY</td></tr>
   * <tr><td>0x18</td><td>DISABLED_IDLE</td></tr>
   * <tr><td>0x19</td><td>DISABLED</td></tr>
   * <tr><td>0x1A</td><td>LPBK_ENTRY</td></tr>
   * <tr><td>0x1B</td><td>LPBK_ACTIVE</td></tr>
   * <tr><td>0x1C</td><td>LPBK_EXIT</td></tr>
   * <tr><td>0x1D</td><td>LPBK_EXIT_TIMEOUT</td></tr>
   * <tr><td>0x1E</td><td>HOT_RESET_ENTRY</td></tr>
   * <tr><td>0x1F</td><td>HOT_RESET</td></tr>
   * <tr><td>0x20</td><td>RCVRY_EQ0</td></tr>
   * <tr><td>0x21</td><td>RCVRY_EQ1</td></tr>
   * <tr><td>0x22</td><td>RCVRY_EQ2</td></tr>
   * <tr><td>0x23</td><td>RCVRY_EQ3</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to LTSSM_STATE
   *
   * Field size: 6 bits
   */
  uint8_t ltssmState;
  /**
   * @brief [rw] LTSSM enable: start the PCI link
   *
   * Set bit to start PCIE link training.  Note: this bit is
   * CLEARED BY FUNDAMENTAL RESET)
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>DISABLED</td></tr>
   * <tr><td>0x1</td><td>ENABLED</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to LTSSM_EN
   *
   * Field size: 1 bit
   */
  uint8_t ltssmEn;
  /**
   * @brief [rw] Application Request Retry Enable
   *
   * Application Request Retry Enable.  Note: this bit is CLEARED
   * BY FUNDAMENTAL RESET)
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>DISABLED:
   *   Incoming PCI transactions are processed normally</td></tr>
   * <tr><td>0x1</td><td>ENABLED:
   *   Incoming PCI transactions are responded with "retry"</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to APP_REQ_RETRY_EN
   *
   * Field size: 1 bit
   */
  uint8_t appReqRetryEn;
  uint8_t devNum; /**< @brief [ro] PCIe device number (5 bits) */
  uint8_t busNum; /**< @brief [ro] PCIe bus number (8 bits) */
} pcieTiConfDeviceCmdReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF PM Control Register
 *
 * Power Management Control
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_PM_CTRL)
 *
 * @{
 */
typedef struct pcieTiConfPmCtrlReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [wo] Transmits PME_Turn_Off message downstream (RC mode only)
   *
   * Eventually sends all links of hierarchy domain to L2L/3_ready
   *
   * <table>
   * <tr><th>write value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>NOACTION</td></tr>
   * <tr><td>0x1</td><td>TRANSMIT</td></tr>
   * </table>
   *
   * Reads always return 0.
   *
   * On rev 1 hw, this corresponds to PME_TURN_OFF
   *
   * Field size: 1 bit
   */
  uint8_t pmeTurnOff;
  /**
   * @brief [wo] Transmits PM_PME wakeup message (EP mode only)
   *
   * <table>
   * <tr><th>write value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>NOACTION</td></tr>
   * <tr><td>0x1</td><td>TRANSMIT</td></tr>
   * </table>
   *
   * Reads always return 0.
   *
   * On rev 1 hw, this corresponds to PM_PME
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /**
   * @brief [rw] Indicates system readiness for the link to enter L2/L3
   *
   * Allows the transmission of PM_Enter_L23 following
   * PM_Turn_OFF / PME_TO_Ack handshake. Self-cleared upon transition to L2/L3.
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>NOT_READY</td></tr>
   * <tr><td>0x1</td><td>READY</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to L23_READY
   *
   * Field size: 1 bit
   */
  uint8_t l23Ready;
  /**
   * @brief [rw] Request to transition to L1 state
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>INACTIVE (No request)</td></tr>
   * <tr><td>0x1</td><td>ACTIVE (L1 entry request)</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to REQ_ENTR_L1
   *
   * Field size: 1 bit
   */
  uint8_t reqEntrL1;
  /**
   * @brief [rw] Request to exit L1 state (to L0)
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>INACTIVE (No request)</td></tr>
   * <tr><td>0x1</td><td>ACTIVE (L1 exit request)</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to REQ_EXIT_L1
   *
   * Field size: 1 bit
   */
  uint8_t reqExitL1;
  /**
   * @brief [rw] Auxilliary Power Detection
   *
   * Status of Vaux detection for the PCIe controller.
   * Determines transition to L2 vs L3 upon Vmain turn-off.
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>UNPOWERED:
   *         Vaux not present: D3cold maps to L3 link state</td></tr>
   * <tr><td>0x1</td><td>POWERED:
   *         Vaux present: D3cold maps to L2 link state</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to AUX_PWR_DET
   *
   * Field size: 1 bit
   */
  uint8_t auxPwrDet;
} pcieTiConfPmCtrlReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF PHY CS
 *
 * Physical Layer Control and Status
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_PHY_CS)
 *
 * @{
 */
typedef struct pcieTiConfPhyCsReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Link status, from LTSSM
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>DOWN</td></tr>
   * <tr><td>0x1</td><td>UP</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to LINK_UP
   *
   * Field size: 1 bit
   */
  uint8_t linkUp;
  /**
   * @brief [rw] Manual lane reversal control
   *
   * Manual lane reversal control, allowing lane 0 and lane 1
   * to be swapped by default; Both Tx and Rx are reversed;
   * Polarity of the individual lane is unchanged
   *
   * <table>
   * <tr><th>value</th><th>description</th></tr>
   * <tr><td>0x0</td><td>STRAIGHT</td></tr>
   * <tr><td>0x1</td><td>REVERSED</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to REVERSE_LANES
   *
   * Field size: 1 bit
   */
  uint8_t reverseLanes;
} pcieTiConfPhyCsReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF INTX ASSERT
 *
 * Legacy INTx ASSERT message control, with 'x' in (A,B,C,D) set by
 * the 'Interrupt Pin' field.
 *
 * Write 1 to send message, read to get the status; EP mode only
 *
 * This register may be used for endpoint mode only.
 *
 * This register is only available on rev 1 hw (TI_CONF_INTX_ASSERT)
 *
 * @{
 */
typedef struct pcieTiConfIntxAssertReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] INTx ASSERT for function 0
   *
   * <table>
   * <tr><th>action/value</th><th>description</th></tr>
   * <tr><td>Write 0x0</td><td>No action</td></tr>
   * <tr><td>Write 0x1</td><td>Transmit INTx ASSERT to RC</td></tr>
   * <tr><td>Read 0x0</td><td>INTx is inactive (deasserted)</td></tr>
   * <tr><td>Read 0x1</td><td>INTx is active (asserted)</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to ASSERT_F0
   *
   * Field size: 1 bit
   */
  uint8_t assertF0;
} pcieTiConfIntxAssertReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF INTX DEASSERT
 *
 * Legacy INTx DEASSERT message control, with 'x' in (A,B,C,D) set by
 * the 'Interrupt Pin' field.
 *
 * Write 1 to send message, read to get the status; EP mode only
 *
 * This register may be used for endpoint mode only.
 *
 * This register is only available on rev 1 hw (TI_CONF_INTX_DEASSERT)
 *
 * @{
 */
typedef struct pcieTiConfIntxDeassertReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] INTx ASSERT for function 0
   *
   * <table>
   * <tr><th>action/value</th><th>description</th></tr>
   * <tr><td>Write 0x0</td><td>No action</td></tr>
   * <tr><td>Write 0x1</td><td>Transmit INTx DEASSERT to RC</td></tr>
   * <tr><td>Read 0x0</td><td>INTx is inactive (deasserted)</td></tr>
   * <tr><td>Read 0x1</td><td>INTx is active (asserted)</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to DEASSERT_F0
   *
   * Field size: 1 bit
   */
  uint8_t deassertF0;
} pcieTiConfIntxDeassertReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF MSI XMT Register
 *
 * MSI transmitter (EP mode); Specifies parameters of MSI, together
 * with MSI capability descriptor already configured by remote RC.
 *
 * This register may be used for endpoint mode only.
 *
 * This register is only available on rev 1 hw (TI_CONF_MSI_XMT)
 *
 * @{
 */
typedef struct pcieTiConfMsiXmtReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI transmit request (and grant status)
   *
   * <table>
   * <tr><th>action/value</th><th>description</th></tr>
   * <tr><td>Write 0x0</td><td>No action</td></tr>
   * <tr><td>Write 0x1</td><td>Request MSI transmission</td></tr>
   * <tr><td>Read 0x0</td><td>MSI transmission request pending</td></tr>
   * <tr><td>Read 0x1</td><td>No MSI request pending (last request granted)</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to MSI_REQ_GRANT
   *
   * Field size: 1 bit
   */
  uint8_t msiReqGrant;
  /**
   * @brief [rw] Function number for transmitted MSI
   *
   * Always 0 for single-function EP
   *
   * On rev 1 hw, this corresponds to MSI_FUNC_NUM
   *
   * Field size: 3 bits
   */
  uint8_t msiFuncNum;
  /**
   * @brief [rw] Vector number for transmitted MSI
   *
   * (as allowed by RC at RW 0x0 enumeration)
   *
   * On rev 1 hw, this corresponds to MSI_VECTOR
   *
   * Field size: 5 bits
   */
  uint8_t msiVector;
  /**
   * @brief [rw] Traffic class (TC) for transmitted MSI
   *
   * On rev 1 hw, this corresponds to MSI_TC
   *
   * Field size: 3 bits
   */
  uint8_t msiTc;
} pcieTiConfMsiXmtReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF Debug Config
 *
 * Configuration of debug_data output and register (observability)
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_DEBUG_CFG)
 *
 * @{
 */
typedef struct pcieTiConfDebugCfgReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Debug_data mode
   *
   * On rev 1 hw, this corresponds to SEL
   *
   * Field size: 6 bits
   */
  uint8_t sel;
} pcieTiConfDebugCfgReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF Debug Data
 *
 * Debug data vector, depending on DEBUG_CFG.sel value
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_DEBUG_DATA
 *
 * Entire register is [RO]
 *
 * @{
 */
typedef struct pcieTiConfDebugDataReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Debug
   *
   * On rev 1 hw, this corresponds to DEBUG
   *
   * Field size: 32 bits
   */
  uint32_t debug;
} pcieTiConfDebugDataReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_ti_conf
 * @brief Specification of the TI CONF Diag Control
 *
 * Diagnostic control
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_DIAG_CTRL)
 *
 * @{
 */
typedef struct pcieTiConfDiagCtrlReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Corrupts LSB of LCRC in the next packet, then self-clears.
   *
   * <table>
   * <tr><th>action/value</th><th>description</th></tr>
   * <tr><td>Write 0x0</td><td>not defined</td></tr>
   * <tr><td>Write 0x1</td><td>Request CRC corruption</td></tr>
   * <tr><td>Read 0x0</td><td>No CRC corruption pending</td></tr>
   * <tr><td>Read 0x1</td><td>CRC corruption pending</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to INV_LCRC
   *
   * Field size: 1 bit
   */
  uint8_t invLcrc;
  /**
   * @brief [rw] Corrupts LSB of ECRC in the next packet, then self-clears.
   *
   * <table>
   * <tr><th>action/value</th><th>description</th></tr>
   * <tr><td>Write 0x0</td><td>not defined</td></tr>
   * <tr><td>Write 0x1</td><td>Request CRC corruption</td></tr>
   * <tr><td>Read 0x0</td><td>No CRC corruption pending</td></tr>
   * <tr><td>Read 0x1</td><td>CRC corruption pending</td></tr>
   * </table>
   *
   * On rev 1 hw, this corresponds to INV_ECRC
   *
   * Field size: 1 bit
   */
  uint8_t invEcrc;
  /**
   * @brief [rw] SW must write 0
   *
   * On rev 1 hw, this corresponds to RESERVED(2)
   *
   * Field size: 1 bit
   */
  uint8_t fastLinkMode;
} pcieTiConfDiagCtrlReg_t;
/* @} */


/**
 * @ingroup pcielld_api_structures
 * @brief Specification all registers
 *
 * This structure allows one or more registers to be read or written
 * through a single call.
 *
 * The user populates one or more pointers to structures.  All structures
 * that are non-NULL are read or written.
 *
 * Once the pointers are populated, use @ref Pcie_readRegs and/or
 * @ref Pcie_writeRegs to perform the actual register accesses
 *
 */
typedef struct pcieRegisters_s {

  /*****************************************************************************************
  *Application Registers
  *****************************************************************************************/
  pciePidReg_t                      *pid;                     /**< @brief PID */
  pcieCmdStatusReg_t                *cmdStatus;               /**< @brief Command Status*/
  pcieCfgTransReg_t                 *cfgTrans;                /**< @brief Config Transaction*/
  pcieIoBaseReg_t                   *ioBase;                  /**< @brief IO TLP base*/
  pcieTlpCfgReg_t                   *tlpCfg;                  /**< @brief TLP Config*/
  pcieRstCmdReg_t                   *rstCmd;                  /**< @brief Reset Command*/
  pciePtmCfgReg_t                   *ptmCfg;                  /**< @brief PTM Config Command*/
  pciePmCmdReg_t                    *pmCmd;                   /**< @brief Power Management Command*/
  pciePmCfgReg_t                    *pmCfg;                   /**< @brief Power Management Config*/
  pcieActStatusReg_t                *actStatus;               /**< @brief Activity Status */
  pcieObSizeReg_t                   *obSize;                  /**< @brief Outbound Translation region size*/
  pcieDiagCtrlReg_t                 *diagCtrl;                /**< @brief Diagnostic Control */
  pcieEndianReg_t                   *endian;                  /**< @brief Endian Register*/
  pciePriorityReg_t                 *priority;                /**< @brief Transaction Priority Register */
  pcieIrqEOIReg_t                   *irqEOI;                  /**< @brief End of Interrupt Register */
  pcieMsiIrqReg_t                   *msiIrq;                  /**< @brief MSI Interrupt IRQ Register*/
  pcieEpIrqSetReg_t                 *epIrqSet;                /**< @brief Endpoint Interrupt Request Set Register*/
  pcieEpIrqClrReg_t                 *epIrqClr;                /**< @brief Endpoint Interrupt Request clear Register*/
  pcieEpIrqStatusReg_t              *epIrqStatus;             /**< @brief Endpoint Interrupt status Register*/
  pcieGenPurposeReg_t               *genPurpose[4];           /**< @brief General Purpose Registers */
  pcieMsiIrqStatusRawReg_t          *msiIrqStatusRaw[8];      /**< @brief MSI Raw Interrupt Status Register*/
  pcieMsiIrqStatusReg_t             *msiIrqStatus[8];         /**< @brief MSI Interrupt Enabled Status Register*/
  pcieMsiIrqEnableSetReg_t          *msiIrqEnableSet[8];      /**< @brief MSI Interrupt Enable Set Register*/
  pcieMsiIrqEnableClrReg_t          *msiIrqEnableClr[8];      /**< @brief MSI Interrupt Enable Clear Register*/
  pcieLegacyIrqStatusRawReg_t       *legacyIrqStatusRaw[4];   /**< @brief Raw Interrupt Status Register*/
  pcieLegacyIrqStatusReg_t          *legacyIrqStatus[4];      /**< @brief Interrupt Enabled Status Register*/
  pcieLegacyIrqEnableSetReg_t       *legacyIrqEnableSet[4];   /**< @brief Interrupt Enable Set Register*/
  pcieLegacyIrqEnableClrReg_t       *legacyIrqEnableClr[4];   /**< @brief Interrupt Enable Clear Register*/
  pcieErrIrqStatusRawReg_t          *errIrqStatusRaw;         /**< @brief Raw Interrupt Status Register*/
  pcieErrIrqStatusReg_t             *errIrqStatus;            /**< @brief Interrupt Enabled Status Register*/
  pcieErrIrqEnableSetReg_t          *errIrqEnableSet;         /**< @brief Interrupt Enable Set Register*/
  pcieErrIrqEnableClrReg_t          *errIrqEnableClr;         /**< @brief Interrupt Enable Clear Register*/
  pciePmRstIrqStatusRawReg_t        *pmRstIrqStatusRaw;       /**< @brief Power Management and Reset Raw Interrupt Status Register*/
  pciePmRstIrqStatusReg_t           *pmRstIrqStatus;          /**< @brief Power Management and Reset Interrupt Enabled Status Register*/
  pciePmRstIrqEnableSetReg_t        *pmRstIrqEnableSet;       /**< @brief Power Management and Reset Interrupt Enable Set Register*/
  pciePmRstIrqEnableClrReg_t        *pmRstIrqEnableClr;       /**< @brief Power Management and Reset Interrupt Enable Clear Register*/
  pciePtmIrqStatusRawReg_t          *ptmIrqStatusRaw;         /**< @brief Precision Time Measurement Raw Interrupt Status Register*/
  pciePtmIrqStatusReg_t             *ptmIrqStatus;            /**< @brief Precision Time Measurement Interrupt Enabled Status Register*/
  pciePtmIrqEnableSetReg_t          *ptmIrqEnableSet;         /**< @brief Precision Time Measurement Interrupt Enable Set Register*/
  pciePtmIrqEnableClrReg_t          *ptmIrqEnableClr;         /**< @brief Precision Time Measurement Interrupt Enable Clear Register*/
  pcieObOffsetLoReg_t               *obOffsetLo[8];           /**< @brief Outbound Translation region offset Low*/
  pcieObOffsetHiReg_t               *obOffsetHi[8];           /**< @brief Outbound Translation region offset High*/
  pcieIbBarReg_t                    *ibBar[4];                /**< @brief Inbound Translation BAR*/
  pcieIbStartLoReg_t                *ibStartLo[4];            /**< @brief Inbound Translation start Low*/
  pcieIbStartHiReg_t                *ibStartHi[4];            /**< @brief Inbound Translation start High*/
  pcieIbOffsetReg_t                 *ibOffset[4];             /**< @brief Inbound Translation offset*/
  pciePcsCfg0Reg_t                  *pcsCfg0;                 /**< @brief PCS Configuration 0 Register */
  pciePcsCfg1Reg_t                  *pcsCfg1;                 /**< @brief PCS Configuration 1 Register */
  pciePcsStatusReg_t                *pcsStatus;               /**< @brief PCS Status Register */
  pcieSerdesCfg0Reg_t               *serdesCfg0;              /**< @brief SERDES config 0 Register*/
  pcieSerdesCfg1Reg_t               *serdesCfg1;              /**< @brief SERDES config 1 Register*/

  /*****************************************************************************************
  *Configuration Registers
  *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/
  pcieVndDevIdReg_t                 *vndDevId;                /**< @brief Vendor and device ID*/
  pcieStatusCmdReg_t                *statusCmd;               /**< @brief Status Command*/
  pcieRevIdReg_t                    *revId;                   /**< @brief Class code and Revision ID*/

  /*Type 0 Registers*/
  pcieBistReg_t                     *bist;                    /**< @brief Bist Header*/
  pcieType0BarIdx_t                 *type0BarIdx;             /**< @brief Type 0 (EP) BAR register*/
  pcieType0Bar32bitIdx_t            *type0Bar32bitIdx;        /**< @brief Type 0 BAR 32bits register*/
  pcieType0Bar32bitIdx_t            *type0BarMask32bitIdx;    /**< @brief Type 0 BAR mask register*/
  pcieCardbusCisPointerReg_t        *cardbusCisPointer;       /**< @brief cardbus CIS pointer register*/
  pcieSubIdReg_t                    *subId;                   /**< @brief Subsystem ID*/
  pcieExpRomReg_t                   *expRom;                  /**< @brief Expansion ROM base addr*/
  pcieCapPtrReg_t                   *capPtr;                  /**< @brief Capabilities Pointer*/
  pcieIntPinReg_t                   *intPin;                  /**< @brief Interrupt Pin*/

  /*Type 1 Registers*/
  pcieType1BistHeaderReg_t          *type1BistHeader;         /**< @brief Bist Header, Latency Timer, Cache Line */
  pcieType1BarIdx_t                 *type1BarIdx;             /**< @brief Type 1 (RC) BAR register*/
  pcieType1Bar32bitIdx_t            *type1Bar32bitIdx;        /**< @brief Type 1 BAR 32bits register*/
  pcieType1Bar32bitIdx_t            *type1BarMask32bitIdx;    /**< @brief Type 1 bar mask register*/
  pcieType1BusNumReg_t              *type1BusNum;             /**< @brief Latency Timer and Bus Number */
  pcieType1SecStatReg_t             *type1SecStat;            /**< @brief Secondary Status and IO space */
  pcieType1MemspaceReg_t            *type1Memspace;           /**< @brief Memory Limit*/
  pciePrefMemReg_t                  *prefMem;                 /**< @brief Prefetch Memory Limit and Base*/
  pciePrefBaseUpperReg_t            *prefBaseUpper;           /**< @brief Prefetch Memory Base Upper*/
  pciePrefLimitUpperReg_t           *prefLimitUpper;          /**< @brief Prefetch Memory Limit Upper*/
  pcieType1IOSpaceReg_t             *type1IOSpace;            /**< @brief IO Base and Limit Upper 16 bits */
  pcieType1CapPtrReg_t              *type1CapPtr;             /**< @brief Capabilities pointer */
  pcieType1ExpnsnRomReg_t           *type1ExpnsnRom;          /**< @brief Expansion ROM base addr */
  pcieType1BridgeIntReg_t           *type1BridgeInt;          /**< @brief Bridge Control and Interrupt Pointer */

  /* Power Management Capabilities Registers */
  pciePMCapReg_t                    *pmCap;                   /**< @brief Power Management Capabilities */
  pciePMCapCtlStatReg_t             *pmCapCtlStat;            /**< @brief Power Management Control and Status */

  /*MSI Capabilities Registers*/
  pcieMsiCapReg_t                   *msiCap;                  /**< @brief MSI Capabilities */
  pcieMsiLo32Reg_t                  *msiLo32;                 /**< @brief MSI Lower 32 bits */
  pcieMsiUp32Reg_t                  *msiUp32;                 /**< @brief MSI Upper 32 bits */
  pcieMsiDataReg_t                  *msiData;                 /**< @brief MSI Data */
  pcieMsiCapOff10HReg_t             *msiCapOff10H;            /**< @brief MSI Cap Off 10H */
  pcieMsiCapOff14HReg_t             *msiCapOff14H;            /**< @brief MSI Cap Off 14H */

  /*Capabilities Registers*/
  pciePciesCapReg_t                 *pciesCap;                /**< @brief PCI Express Capabilities Register*/
  pcieDeviceCapReg_t                *deviceCap;               /**< @brief Device Capabilities Register*/
  pcieDevStatCtrlReg_t              *devStatCtrl;             /**< @brief Device Status and Control*/
  pcieLinkCapReg_t                  *linkCap;                 /**< @brief Link Capabilities Register*/
  pcieLinkStatCtrlReg_t             *linkStatCtrl;            /**< @brief Link Status and Control Register*/
  pcieSlotCapReg_t                  *slotCap;                 /**< @brief Slot Capabilities Register */
  pcieSlotStatCtrlReg_t             *slotStatCtrl;            /**< @brief Slot Status and Control Register */
  pcieRootCtrlCapReg_t              *rootCtrlCap;             /**< @brief Root Control and Capabilities Register */
  pcieRootStatusReg_t               *rootStatus;              /**< @brief Root Status and Control Register */
  pcieDevCap2Reg_t                  *devCap2;                 /**< @brief Device Capabilities 2 Register*/
  pcieDevStatCtrl2Reg_t             *devStatCtrl2;            /**< @brief Device Status and Control 2 Register*/
  pcieLnkCap2Reg_t                  *linkCap2;                /**< @brief Link Capabilities 2 Register*/
  pcieLinkCtrl2Reg_t                *linkCtrl2;               /**< @brief Link Control 2 Register*/

  /*Capabilities Extended Registers*/
  pcieExtCapReg_t                   *extCap;                  /**< @brief Extended Capabilties Header */
  pcieUncErrReg_t                   *uncErr;                  /**< @brief Uncorrectable Error Status */
  pcieUncErrMaskReg_t               *uncErrMask;              /**< @brief Uncorrectable Error Mask */
  pcieUncErrSvrtyReg_t              *uncErrSvrty;             /**< @brief Uncorrectable Error Severity */
  pcieCorErrReg_t                   *corErr;                  /**< @brief Correctable Error Status */
  pcieCorErrMaskReg_t               *corErrMask;              /**< @brief Correctable Error Mask */
  pcieAccrReg_t                     *accr;                    /**< @brief Advanced Capabilities and Control*/
  pcieHdrLogReg_t                   *hdrLog[4];               /**< @brief Header Log Registers */
  pcieRootErrCmdReg_t               *rootErrCmd;              /**< @brief Root Error Command */
  pcieRootErrStReg_t                *rootErrSt;               /**< @brief Root Error Status */
  pcieErrSrcIDReg_t                 *errSrcID;                /**< @brief Error Source Identification */

  /*Port Logic Registers*/
  pciePlAckTimerReg_t               *plAckTimer;              /**< @brief Ack Latency Time and Replay Timer */
  pciePlOMsgReg_t                   *plOMsg;                  /**< @brief Other Message */
  pciePlForceLinkReg_t              *plForceLink;             /**< @brief Port Force Link */
  pcieAckFreqReg_t                  *ackFreq;                 /**< @brief Ack Frequency */
  pcieLnkCtrlReg_t                  *lnkCtrl;                 /**< @brief Port Link Control*/
  pcieLaneSkewReg_t                 *laneSkew;                /**< @brief Lane Skew */
  pcieSymNumReg_t                   *symNum;                  /**< @brief Symbol Number */
  pcieSymTimerFltMaskReg_t          *symTimerFltMask;         /**< @brief Symbol Timer and Filter Mask */
  pcieFltMask2Reg_t                 *fltMask2;                /**< @brief Filter Mask 2 */
  pcieDebug0Reg_t                   *debug0;                  /**< @brief Debug 0*/
  pcieDebug1Reg_t                   *debug1;                  /**< @brief Debug 1 Register*/
  pcieGen2Reg_t                     *gen2;                    /**< @brief Gen2 */

  /* Rev 1 PLCONF */
  pciePlconfObnpSubreqCtrlReg_t      *plconfObnpSubreqCtrl;   /**< @brief PCIECTRL_PL_OBNP_SUBREQ_CTRL*/
  pciePlconfTrPStsRReg_t             *plconfTrPStsR;          /**< @brief PCIECTRL_PL_TR_P_STS_R*/
  pciePlconfTrNpStsRReg_t            *plconfTrNpStsR;         /**< @brief PCIECTRL_PL_TR_NP_STS_R*/
  pciePlconfTrCStsRReg_t             *plconfTrCStsR;          /**< @brief PCIECTRL_PL_TR_C_STS_R*/
  pciePlconfQStsRReg_t               *plconfQStsR;            /**< @brief PCIECTRL_PL_Q_STS_R*/
  pciePlconfVcTrAR1Reg_t             *plconfVcTrAR1;          /**< @brief PCIECTRL_PL_VC_TR_A_R1*/
  pciePlconfVcTrAR2Reg_t             *plconfVcTrAR2;          /**< @brief PCIECTRL_PL_VC_TR_A_R2*/
  pciePlconfVc0PrQCReg_t             *plconfVc0PrQC;          /**< @brief PCIECTRL_PL_VC0_PR_Q_C*/
  pciePlconfVc0NprQCReg_t            *plconfVc0NprQC;         /**< @brief PCIECTRL_PL_VC0_NPR_Q_C*/
  pciePlconfVc0CrQCReg_t             *plconfVc0CrQC;          /**< @brief PCIECTRL_PL_VC0_CR_Q_C*/
  pciePlconfVcPrQCReg_t              *plconfVcPrQC[3];        /**< @brief for VC1..VC3 */
  pciePlconfVcNprQCReg_t             *plconfVcNprQC[3];       /**< @brief for VC1..VC3 */
  pciePlconfVcCrQCReg_t              *plconfVcCrQC[3];        /**< @brief for VC1..VC3 */
  /* note: plconfWidthSpeedCtlReg_t is mapped to gen2 above */
  pciePlconfPhyStsRReg_t             *plconfPhyStsR;          /**< @brief PCIECTRL_PL_PHY_STS_R*/
  pciePlconfPhyCtrlRReg_t            *plconfPhyCtrlR;         /**< @brief PCIECTRL_PL_PHY_CTRL_R*/
  pciePlconfMsiCtrlAddressReg_t      *plconfMsiCtrlAddress;   /**< @brief PCIECTRL_PL_MSI_CTRL_ADDRESS*/
  pciePlconfMsiCtrlUpperAddressReg_t *plconfMsiCtrlUpperAddress; /**< @brief PCIECTRL_PL_MSI_CTRL_UPPER_ADDRESS*/
  pciePlconfMsiCtrlIntEnableReg_t    *plconfMsiCtrlIntEnable[8]; /**< @brief PCIECTRL_PL_MSI_CTRL_INT_ENABLE_N*/
  pciePlconfMsiCtrlIntMaskReg_t      *plconfMsiCtrlIntMask[8];   /**< @brief PCIECTRL_PL_MSI_CTRL_INT_MASK_N*/
  pciePlconfMsiCtrlIntStatusReg_t    *plconfMsiCtrlIntStatus[8]; /**< @brief PCIECTRL_PL_MSI_CTRL_INT_STATUS_N*/
  pciePlconfMsiCtrlGpioReg_t         *plconfMsiCtrlGpio;      /**< @brief PCIECTRL_PL_MSI_CTRL_GPIO*/
  pciePlconfPipeLoopbackReg_t        *plconfPipeLoopback;     /**< @brief PCIECTRL_PL_PIPE_LOOPBACK*/
  pciePlconfDbiRoWrEnReg_t           *plconfDbiRoWrEn;        /**< @brief PCIECTRL_PL_DBI_RO_WR_EN*/
  pciePlconfAxiSlvErrRespReg_t       *plconfAxiSlvErrResp;    /**< @brief PCIECTRL_PL_AXI_SLV_ERR_RESP*/
  pciePlconfAxiSlvTimeoutReg_t       *plconfAxiSlvTimeout;    /**< @brief PCIECTRL_PL_AXI_SLV_TIMEOUT*/
  pciePlconfIatuIndexReg_t           *plconfIatuIndex;        /**< @brief PCIECTRL_PL_IATU_INDEX*/
  pciePlconfIatuRegCtrl1Reg_t        *plconfIatuRegCtrl1;     /**< @brief PCIECTRL_PL_IATU_REG_CTRL_1*/
  pciePlconfIatuRegCtrl2Reg_t        *plconfIatuRegCtrl2;     /**< @brief PCIECTRL_PL_IATU_REG_CTRL_2*/
  pciePlconfIatuRegLowerBaseReg_t    *plconfIatuRegLowerBase; /**< @brief PCIECTRL_PL_IATU_REG_LOWER_BASE*/
  pciePlconfIatuRegUpperBaseReg_t    *plconfIatuRegUpperBase; /**< @brief PCIECTRL_PL_IATU_REG_UPPER_BASE*/
  pciePlconfIatuRegLimitReg_t        *plconfIatuRegLimit;     /**< @brief PCIECTRL_PL_IATU_REG_LIMIT*/
  pciePlconfIatuRegLowerTargetReg_t  *plconfIatuRegLowerTarget; /**< @brief PCIECTRL_PL_IATU_REG_LOWER_TARGET*/
  pciePlconfIatuRegUpperTargetReg_t  *plconfIatuRegUpperTarget; /**< @brief PCIECTRL_PL_IATU_REG_UPPER_TARGET*/
  pciePlconfIatuRegCtrl3Reg_t        *plconfIatuRegCtrl3;     /**< @brief PCIECTRL_PL_IATU_REG_CTRL_3*/

  /*****************************************************************************************
  * HW Rev 1 configuration registers
  *****************************************************************************************/
  /* TI Configuration registers (PCIECTRL_TI_CONF* in TRM) */
  pcieTiConfRevisionReg_t           *tiConfRevision;          /**< @brief PCIECTRL_TI_CONF_REVISION*/
  pcieTiConfSysConfigReg_t          *tiConfSysConfig;         /**< @brief PCIECTRL_TI_CONF_SYSCONFIG*/
  pcieTiConfIrqEoiReg_t             *tiConfIrqEoi;            /**< @brief PCIECTRL_TI_CONF_IRQ_EOI*/
  pcieTiConfIrqStatusRawMainReg_t   *tiConfIrqStatusRawMain;  /**< @brief PCIECTRL_TI_CONF_IRQSTATUS_RAW_MAIN*/
  pcieTiConfIrqStatusMainReg_t      *tiConfIrqStatusMain;     /**< @brief PCIECTRL_TI_CONF_IRQSTATUS_MAIN*/
  pcieTiConfIrqEnableSetMainReg_t   *tiConfIrqEnableSetMain;  /**< @brief PCIECTRL_TI_CONF_IRQENABLE_SET_MAIN*/
  pcieTiConfIrqEnableClrMainReg_t   *tiConfIrqEnableClrMain;  /**< @brief PCIECTRL_TI_CONF_IRQENABLE_CLR_MAIN*/
  pcieTiConfIrqStatusRawMsiReg_t    *tiConfIrqStatusRawMsi;   /**< @brief PCIECTRL_TI_CONF_IRQSTATUS_RAW_MSI*/
  pcieTiConfIrqStatusMsiReg_t       *tiConfIrqStatusMsi;      /**< @brief PCIECTRL_TI_CONF_IRQSTATUS_MSI*/
  pcieTiConfIrqEnableSetMsiReg_t    *tiConfIrqEnableSetMsi;   /**< @brief PCIECTRL_TI_CONF_IRQENABLE_SET_MSI*/
  pcieTiConfIrqEnableClrMsiReg_t    *tiConfIrqEnableClrMsi;   /**< @brief PCIECTRL_TI_CONF_IRQENABLE_CLR_MSI*/
  pcieTiConfDeviceTypeReg_t         *tiConfDeviceType;        /**< @brief PCIECTRL_TI_CONF_DEVICE_TYPE*/
  pcieTiConfDeviceCmdReg_t          *tiConfDeviceCmd;         /**< @brief PCIECTRL_TI_CONF_DEVICE_CMD*/
  pcieTiConfPmCtrlReg_t             *tiConfPmCtrl;            /**< @brief PCIECTRL_TI_CONF_PM_CTRL*/
  pcieTiConfPhyCsReg_t              *tiConfPhyCs;             /**< @brief PCIECTRL_TI_CONF_PHY_CS*/
  pcieTiConfIntxAssertReg_t         *tiConfIntxAssert;        /**< @brief PCIECTRL_TI_CONF_INTX_ASSERT*/
  pcieTiConfIntxDeassertReg_t       *tiConfIntxDeassert;      /**< @brief PCIECTRL_TI_CONF_INTX_DEASSERT*/
  pcieTiConfMsiXmtReg_t             *tiConfMsiXmt;            /**< @brief PCIECTRL_TI_CONF_MSI_XMT*/
  pcieTiConfDebugCfgReg_t           *tiConfDebugCfg;          /**< @brief PCIECTRL_TI_CONF_DEBUG_CFG*/
  pcieTiConfDebugDataReg_t          *tiConfDebugData;         /**< @brief PCIECTRL_TI_CONF_DEBUG_DATA*/
  pcieTiConfDiagCtrlReg_t           *tiConfDiagCtrl;          /**< @brief PCIECTRL_TI_CONF_DIAG_CTRL*/
} pcieRegisters_t;

/*****************************************************************************
 **********  Configuration Structures **********************
 ****************************************************************************/

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of pcieIbTransCfg
 *
 * The pcieIbTransCfg is used to configure the Inbound Translation Registers
 */
typedef struct pcieIbTransCfg_s {
  /**
   * @brief Inbound Translation BAR match
   */
  uint8_t    ibBar;
  /**
   * @brief Low Inbound Start address (32bits)
   */
  uint32_t   ibStartAddrLo;
  /**
   * @brief High Inbound Start address (32bits)
   */
  uint32_t   ibStartAddrHi;
  /**
   * @brief Inbound Translation Address Offset (32bits)
   */
  uint32_t   ibOffsetAddr;
  /**
   * @brief Identifies the translation region (0-3)
   */
  uint8_t    region;
} pcieIbTransCfg_t;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of pcieBarCfg
 *
 * The pcieBarCfg is used to configure a 32bits BAR Register
 * or the lower 32bits of a 64bits BAR register. \n
 * This should NOT be used to configure BAR masks.\n
 * This should NOT be used to configure the Upper 32bits of
 * a 64bits BAR register.
 */
typedef struct pcieBarCfg_s {
  /**
   * @brief Local or remote peripheral
   */
  pcieLocation_e location;
  /**
   * @brief PCIe mode
   */
  pcieMode_e mode;
  /**
   * @brief Base Address (32bits)
   */
  uint32_t base;
  /**
   * @brief Prefetch
   */
  pcieBarPref_e prefetch;
  /**
   * @brief Type
   */
  pcieBarType_e type;
  /**
   * @brief Memory Space
   */
  pcieBarMem_e memSpace;
  /**
   * @brief BAR index
   */
  uint8_t idx;
} pcieBarCfg_t;

/**
 *  \brief This Structure defines the ATU region parameters
 *
 *  These parameters are used for configuring inbound or outbound
 *  ATU(Address translation unit) region
 */
typedef struct pcieAtuRegionParams
{
    pcieAtuRegionDir_t       regionDir;
    /**< Region direction Inbound or Outbound
     * Values given by enum #pcieAtuRegionDir_t
     */
    pcieTlpType_t            tlpType;
    /**< TLP(transaction layer packet) type
     * Values given by enum #pcieTlpType_t
     */
    uint32_t                 enableRegion;
    /**< Region enable or disable */
    pcieAtuRegionMatchMode_t matchMode;
    /**< Region match mode Address match or BAR match
     * Values given by enum #pcieAtuRegionMatchMode_t
     */
    uint32_t                 barNumber;
    /**< BAR number with which the region is associated
     *   Possible values for EP : 0 to 5 for 32bit and 0 to 2 for 64bit
     *   Possible values for RC : 0 to 1 for 32bit and 0 for 64bit
     */
    uint32_t                 lowerBaseAddr;
    /**< Lower base address : should be 4K aligned
     *   For outbound configuration this contains outbound region offset
     *   For inbound  configuration this contains inbound PCIe start address
     */
    uint32_t                 upperBaseAddr;
    /**< Upper base address
     *   Higher 32 bits in case of 64 bit addressing
     *   Configured to 0 for 32 bit addressing
     */
    uint32_t                 regionWindowSize;
    /**< Region window size
     *   For outbound configuration this contains outbound window size
     *   For inbound  configuration this contains PCIe inbound window size
     */
    uint32_t                 lowerTargetAddr;
    /**< Lower Target address: should be 4K aligned
     *   For outbound configuration this contains outbound PCIe start offset
     *   For inbound  configuration this contains destination address
     */
    uint32_t                 upperTargetAddr;
    /**< Upper target address
     *   Higher 32 bits in case of 64 bit addressing
     *   Configured to 0 for 32 bit addressing
     */
} pcieAtuRegionParams_t;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_DeviceCfgBaseAddr
 *
 * The Pcie_DeviceCfg is used to specify device level configuration
 * to the LLD.
 */
typedef struct
{
  /** 
   * Device-dependant configuration base address.  If hw rev has one
   * base, then it is directly here.  If it has multiple bases, look in
   * src/v#/pcie.h for pciev#_DeviceCfgBaseAddrs
   */
  void              *cfgBase;
  /** 
   * Base address of the "data" area (remote device memory) 
   */
  void              *dataBase;
  /**
   * Reserved part of real data area due to memory mapping.  For example
   * on some M4s the PCIe address is at 0x20000000 which is illegal for
   * M4.  Thus its moved to 0x24000000.  In this case dataReserved
   * is 0x04000000.
   */
  uint32_t           dataReserved;
  /** 
   * Revision-dependant device params.  Look for Pciev#_DevParams in
   * src/v#/pcie.h.  Not all HW will have these (put NULL).
   */
  void              *devParams; 
} Pcie_DeviceCfgBaseAddr;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_Handle
 *
 * The Pcie_Handle is used to identify a PCIE LLD instance
 */
typedef void *Pcie_Handle;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_FxnTable
 *
 * The Pcie_FxnTable is used to abstract multiple implementations of
 * pcie hardware.
 */
typedef struct
{
  /*! Function to set PCIE to EP or RC for one device */
  pcieRet_e (*setInterfaceMode) (Pcie_Handle handle, pcieMode_e mode);
  /*! Function to get the PCIE data area reserved size */
  pcieRet_e (*getMemSpaceReserved) (Pcie_Handle handle, uint32_t *resSize);
  /*! Function to get the PCIE data area base address & size */
  pcieRet_e (*getMemSpaceRange) (Pcie_Handle handle, void **base,
                                 uint32_t *size);
  /*! Function to read any PCIE register(s) */
  pcieRet_e (*readRegs) (Pcie_Handle handle, pcieLocation_e location,
                         pcieRegisters_t *readRegs);
  /*! Function to write any PCIE register(s) */
  pcieRet_e (*writeRegs) (Pcie_Handle handle, pcieLocation_e location,
                         pcieRegisters_t *writeRegs);
  /*! Function to configure outbound translation registers */
  pcieRet_e (*cfgObOffset) (Pcie_Handle handle, uint32_t obAddrLo,
                            uint32_t obAddrHi, uint8_t region);
  /*! Function to configure inbound translation registers */
  pcieRet_e (*cfgIbTrans) (Pcie_Handle handle, const pcieIbTransCfg_t *ibCfg);
  /*! Function to configure a BAR register */
  pcieRet_e (*cfgBar) (Pcie_Handle handle, const pcieBarCfg_t *barCfg);
  /*! Function to configure an ATU region */
  pcieRet_e (*cfgAtu) (Pcie_Handle handle, pcieLocation_e location, 
                       uint32_t atuRegionIndex, 
                       const pcieAtuRegionParams_t *atuRegionParams);
  /*! Function to read functional (MSI/INTX) pending bits with low overhead.
   *  For v0 hardware not supported
   *  For v1 hardware second argument is @ref pcieTiConfIrqStatusMsiReg_t
   *  and fourth argument is array of pciePlconfMsiCtrlIntStatusReg_t */
  pcieRet_e (*getPendingFuncInts) (Pcie_Handle handle, 
                                   void *pendingBits,
                                   int32_t sizeMsiBits,
                                   void *msiBits);
  /*! Function to clear functional (MSI/INTX) pending bits with low overhead.
   *  For v0 hardware not supported
   *  For v1 hardware second argument is @ref pcieTiConfIrqStatusMsiReg_t,
   *  and fourth argument is array of pciePlconfMsiCtrlIntStatusReg_t */
  pcieRet_e (*clrPendingFuncInts) (Pcie_Handle handle, 
                                   void *pendingBits,
                                   int32_t sizeMsiBits,
                                   void *msiBits);
} Pcie_FxnTable;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_DeviceCfg
 *
 * The Pcie_DeviceCfg is used to specify device level configuration
 * to the LLD.
 */
#define pcie_MAX_PERIPHS 4 /**< Maximum peripherals (base addresses) supported by LLD */
typedef struct
{
  Pcie_DeviceCfgBaseAddr *basesPtr[pcie_MAX_PERIPHS]; /**< base addreses */
  Pcie_FxnTable *fxnTablePtr[pcie_MAX_PERIPHS]; /**< function pointers */
} Pcie_DeviceCfg;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_InitCfg
 *
 * The Pcie_InitCfg is used to specify per-core
 * configuration to the LLD.  It is used with @ref Pcie_init ()
 * once per core.
 */
typedef struct
{
  Pcie_DeviceCfg dev; /**< Device Configuration */
} Pcie_InitCfg;

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_init sets device configuration
 *
 *  @details This function binds a device configuration to
 *           the LLD.  This must be done before calling
 *           the other APIs.
 *
 *           Calling init is nondestrictive, ie it can be
 *           done more than once without side effects (assuming
 *           same argument is passed each time).
 *
 *  @pre     No assumptions
 *
 *  @retval  pcieRet_e status
 *
 *  @post    pcieLObj.device gets set to argument
 */
pcieRet_e Pcie_init
(
  const Pcie_InitCfg *cfg /**< [in] configuration */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_open creates/opens a PCIe instance
 *
 *  @details This function creates a handle.  The peripheral itself
 *           is not modified.  More than one handle to the same
 *           PCIe peripheral can exist at the same time.
 *
 *  @pre     pHandle != NULL
 *  @pre     *pHandle == NULL
 *
 *  @retval  pcieRet_e status
 *
 *  @post    *pHandle == valid handle
 */
pcieRet_e Pcie_open
(
  int32_t            deviceNum,  /**< [in] PCIe device number (0,1,...) */
  Pcie_Handle   *pHandle     /**< [out] Resulting instance handle */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief Pcie_close Closes (frees) the driver handle.
 *
 *  @details The handle is released.  The peripheral itself is not
 *           modified.
 *
 *  @pre     pHandle != NULL
 *  @pre     *pHandle == valid handle
 *
 *  @retval  pcieRet_e status
 *
 *  @post    *pHandle == NULL
 */
pcieRet_e Pcie_close
(
  Pcie_Handle *pHandle /**< [in] The PCIE LLD instance indentifier */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief Pcie_readRegs Performs a register read
 *
 *  @details Reads one or more of the device registers
 *
 *  Each non-NULL register pointer in readRegs will be read and
 *  broken into its fields.
 *
 *  Some registers have multiple instances (e.g. BAR registers, there is
 *  BAR0, BAR1, etc). In these cases an "index" is used to select which
 *  instance of the register will be accessed (e.g. use index 0 to access BAR0 and so on).
 *
 *  Registers that need an "index" input
 *  can only be read one index at a time. Also, "index" must be set
 *  by the code before issuing a read request.
 *
 *  It is the users responsibility to ensure that no other tasks
 *  or cores will modify the registers while they are read,
 *  or betwen the time the registers are read and they are
 *  later written back.
 *
 *
 *  Since the peripheral is shared across the device, and even
 *  between peripherals, it is not expected to be dynamically
 *  reprogramed (such as between thread or task switches).  It
 *  should only be reprogrammed at startup or when changing
 *  applications.  Therefore, there is a single-entry API instead
 *  of a set of inlines since it is not time-critical code.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_readRegs
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *readRegs  /**< [in/out] List of registers to read */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief Pcie_writeRegs Performs a configuration write
 *
 *  @details Writes one or more of the device registers.
 *
 *  Each non-NULL register pointer in writeRegs will be writen.
 *
 *  Some registers have multiple instances (e.g. BAR registers, there is
 *  BAR0, BAR1, etc). In these cases an "index" is used to select which
 *  instance of the register will be accessed (e.g. use index 0 to access BAR0 and so on).
 *
 *  Registers that need an "index" input can only be written
 *  one index at a time.
 *
 *  It is the users responsibility to ensure that no other tasks
 *  or cores will modify the registers while they are read,
 *  or betwen the time the registers are read and they are
 *  later written back.
 *
 *  The user will typically use @ref Pcie_readRegs to read the current
 *  values in the registers, modify them in the local copies, then
 *  write back using @ref Pcie_writeRegs.
 *
 *  On exit, the actual written values are returned in each register's
 *  reg->raw.
 *
 *  Since the peripheral is shared across the device, and even
 *  between peripherals, it is not expected to be dynamically
 *  reprogramed (such as between thread or task switches).  It
 *  should only be reprogrammed at startup or when changing
 *  applications.  Therefore, there is a single-entry API instead
 *  of a set of inlines since it is not time-critical code.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_writeRegs
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *writeRegs /**< [in] List of registers to write */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_setInterfaceMode sets the PCI mode for specified interface
 *
 *  Note: if the PCIESSMODE field is in a register protected by a kicker
 *  mechanism, unlock it before calling this function.  It is not
 *  multicore safe to repeatedly unlock/lock the kicker.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_setInterfaceMode
(
  Pcie_Handle handle, /**< [in] specified interface */
  pcieMode_e mode     /**< [in] PCIE Mode */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_getMemSpaceReserved returns amount of reserved space 
 *          between beginning of hardware's data area and the base returned
 *          by @ref Pcie_getMemSpaceRange. 
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_getMemSpaceReserved
(
  Pcie_Handle  handle,   /**< [in] The PCIE LLD instance identifier */
  uint32_t    *resSize   /**< [out] Total size of the memory space [bytes] */
);


/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_getMemSpaceRange Returns the PCIe Internal Address
 *  Range for the device's internal address range 1, which is
 *  the Memory Space.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_getMemSpaceRange
(
  Pcie_Handle  handle,   /**< [in] The PCIE LLD instance identifier */
  void         **base,   /**< [out] The memory space base address */
  uint32_t      *size    /**< [out] Total size of the memory space [bytes] */
);


/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_cfgObOffset Configures the Outbound Offset registers
 *          for outbound address translation
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_cfgObOffset
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  uint32_t         obAddrLo, /**< [in] Low  Outbound address offset (32bits) */
  uint32_t         obAddrHi, /**< [in] High Outbound address offset (32bits) */
  uint8_t          region    /**< [in] Identifies the Outbound region (0-7) */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_cfgIbTrans Configures the Inbound Address Translation registers.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_cfgIbTrans
(
  Pcie_Handle             handle,  /**< [in] The PCIE LLD instance identifier */
  const pcieIbTransCfg_t *ibCfg    /**< [in] Inbound Address Translation Configuration parameters */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_cfgBar is used to configure a 32bits BAR Register
 *
 *  A BAR register can represent any of the following:\n
 *  (a) a 32bit BAR\n
 *  (b) the lower 32bits of a 64bits BAR\n
 *  (c) the upper 32bits of a 64bits BAR\n
 *  (d) a BAR mask\n
 *
 *  BAR registers can always be accessed using @ref Pcie_readRegs
 *  and/or @ref Pcie_writeRegs.
 *
 *  @ref Pcie_cfgBar is used to configure a 32bits BAR Register or the lower
 *  32bits of a 64bits BAR register. That is, (a) and (b) above.
 *
 *  @ref Pcie_cfgBar should NOT be used to configure the Upper 32bits of a 64bits BAR register (c).\n
 *  @ref Pcie_cfgBar should NOT be used to configure BAR masks (d). \n\n
 *  In order to access BAR masks or Upper 32bits BAR, use @ref Pcie_readRegs and/or
 *  @ref Pcie_writeRegs to perform the actual 32bits register accesses, using
 *  @ref pcieType0Bar32bitIdx_t (for a End point BAR) or @ref pcieType1Bar32bitIdx_t (for a Root Complex BAR).
 *
 *  In order to set a BAR MASK on rev 1 hw, must put it in 
 *
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_cfgBar
(
  Pcie_Handle            handle,   /**< [in] The PCIE LLD instance identifier */
  const pcieBarCfg_t    *barCfg    /**< [in] BAR configuration parameters */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_atuRegionConfig is used to program hw rev 1 address translation entries
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_atuRegionConfig 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] local/remote */
  uint32_t         atuRegionIndex, /**< [in] index number to configure */
  const            pcieAtuRegionParams_t *atuRegionParams /**< [in] config structure */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_atuRegionConfig is used to program hw rev 1 address translation entries
 *
 *  For rev 0 hw, not supported.
 *  For rev 1 hw, pendingBits is @ref pcieTiConfIrqStatusMsiReg_t
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_getPendingFuncInts
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [out] rev-specific pending bits */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [out] rev-specific msi pending bits to check */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_clrPendingFuncInts is used to clear processed pending interrupts
 *
 *  For rev 0 hw, not supported.
 *  For rev 1 hw, pendingBits is @ref pcieTiConfIrqStatusMsiReg_t
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_clrPendingFuncInts
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [in] rev-specific pending bits */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [in] rev-specific msi pending bits to ack */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_getVersion returns the PCIE LLD version information
 *
 *  @details This function is used to get the version information of the PCIE LLD in 0xAABBCCDD format.
 *           where Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 *
 *  @retval                32-bit version information
 */
uint32_t Pcie_getVersion (void);


/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_getVersionStr returns the PCIE LLD version string
 *
 *  @details This function is used to get the version string of the PCIE LLD.
 *
 *  @retval                Version string
 */
const char* Pcie_getVersionStr (void);

#ifdef __cplusplus
}
#endif

#endif  /* _PCIE_H */

/* Nothing past this point */

