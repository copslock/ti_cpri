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

#ifndef _HYPLNK_H
#define _HYPLNK_H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <stdint.h>
#include <stdlib.h>
#include "hyplnk_cfg.h"
#include "hyplnkver.h"

/* ============================================================= */
/**
 *   @file  hyplnk.h
 *
 *   path  ti/drv/hyplnk/hyplnk.h
 *
 *   @brief  Hyperlink transport sub-system API and Data Definitions
 *
 */
 
/**  @mainpage hyperlink Low Level Driver
 *
 *   @section intro  Introduction
 *
 *   The Hyperlink peripheral has also been known as vUSR and MCM.  Some chip-specific
 *   definitions in CSL and documentation may have references to the old names.
 *
 *   The Hyperlink peripheral provides a high-speed, low-latency, and low-power 
 *   point-to-point link between two KeyStone devices.  
 *
 *   There is a userguide <http://www.ti.com/lit/sprugw8>.
 *
 *   The HyperLink provides three basic functionalities between two devices:
 *
 *     -# There is a memory mapped window on the local device which can
 *        be mapped by the remote device.  This is the top part of the
 *        diagram below.
 *     -# There is a mapping functionality in the receive direction
 *        which allows the incoming addresses to be mapped across the device's
 *        memory space.  This is the bottom part of the diagram below\n
 *        @image html address_translation.jpg
 *     -# There is the ability to send events (which can trigger hardware
 *        such as the EDMA or produce interrupts).
 *     -# Finally there is an ability to bootload the secondary device.
 *        See <http://www.ti.com/lit/sprugy5>.
 *
 *  The primary APIs are:\n
 *  -- @ref Hyplnk_open : Opens the driver (by pointing to the peripheral's base address)\n
 *  -- @ref Hyplnk_readRegs : Reads one or more registers or tables\n
 *  -- @ref Hyplnk_writeRegs : Writes one or more registers or tables\n
 *  -- @ref Hyplnk_close : Close the driver instance (NOP since no memory is used by the driver)\n
 *  -- @ref Hyplnk_getWindow : Get the base address of the memory window used to access remote device\n
 *
 *  The normal sequence of events to enable peripheral are below.  There is a C
 *  code example in ti/drv/hyplnk/example/memoryMappedExample.
 *
 *  -# Set up the system PLL (in the example, this is done via 
 *  ti/drv/hyplnk/example/board_setup.gel)
 *  -# Enable the power domain for the peripheral via the CSL_PSC_enablePowerDomain()
 *  CSL_PSC_setModuleNextState() and CSL_PSC_startStateTransition.  This sequence
 *  is in the example in function hyplnkExampleSysSetup().
 *  -# Set up the SERDES configuration in CSL_BootCfgSetVUSRConfigPLL, 
 *  CSL_BootCfgSetVUSRRxConfig, and CSL_BootCfgSetVUSRTxConfig.  
 *  -# Confirm that the power is enabled by reading the revision register
 *  @ref hyplnkRevReg_s.
 *  -# Enable the peripherial via the control register @ref hyplnkControlReg_s
 *  -# The peripheral is ready when @ref hyplnkStatusReg_s::pllUnlock becomes 0,
 *  the @ref hyplnkStatusReg_s::link becomes 1.  Since both endpoints of the
 *  HyperLink must perform the synchronization, busywaiting as in the example
 *  may not be applicable.  Once the peripheral is up, one of the two endpoints
 *  can program the communication for both since both the local and remote
 *  registers are available to both sides.
 *  -# The errors in @ref hyplnkStatusReg_s::lError, @ref hyplnkStatusReg_s::rError,
 *  and in @ref hyplnkECCErrorsReg_s should not occur.  These indicate
 *  problems with the link.  Troubleshooting can involve reducing the link speed
 *  via the SERDES registers in the boot config peripheral.
 *
 *  Once the link is up, both devices can see each other's peripheral registers.
 *  Thus, each device can program its own parameters, or one can act as a master
 *  and control both device's parameters.
 *
 *  The following actions are generally taken to set up the logical connection
 *  once the link is established
 *
 *  -# Use @ref hyplnkRegisters_s::TXAddrOvly and @ref hyplnkRegisters_s::RXAddrSel
 *  to specify the agreed address encoding between the two devices.
 *  -# Use @ref hyplnkRegisters_s::RXPrivIDTbl and @ref hyplnkRegisters_s::RXSegTbl
 *  to specify the address decoding.
 *  -# Use @ref hyplnkRegisters_s::intCtrlTbl and @ref hyplnkRegisters_s::intPtrTbl
 *  to specify event/interrupt configuration (consider @ref hyplnkControlReg_s::int2cfg).
 *
 *  Once the local memory window is identified by @ref Hyplnk_getWindow, the remote 
 *  device can be read/written via this memory window using any master (direct CPU
 *  access (memcpy), peripherals, or DMA (EDMA).
 *  
 */
 
/* Define hyplnk LLD Module as a master group in Doxygen format and add all HYPLNK LLD API 
   definitions to this group. */
/** @defgroup hyplnklld_module HYPLNK LLD Module API
 *  @{
 */
/** @} */
   
/** @defgroup hyplnklld_api_functions HYPLNK LLD Functions
 *  @ingroup hyplnklld_module
 */
 
/** @defgroup hyplnklld_api_macros HYPLNK LLD Macros
 *  @ingroup hyplnklld_module
 */

/** @defgroup hyplnklld_api_structures HYPLNK LLD API Data Structures
 *  @ingroup hyplnklld_module
 */

/** @defgroup hyplnklld_reg_structures HYPLNK LLD Register Definitions
 *  @ingroup hyplnklld_module
 */

/** @defgroup hyplnklld_api_constants HYPLNK LLD Constants (enum's and define's)
 *  @ingroup hyplnklld_module
 */

/** These are the possible return values from all HYPLNK LLD functions */
typedef enum 
{
#ifdef hyplnk_DEBUG
  /**
   * The call succeeded, but the application could have leaked memory
   * since a non-NULL pointer was overwritten.  This only
   */
  hyplnk_RET_DBG_NON_NULL = -100L,
  hyplnk_RET_DBG_USELESS_WRITE,  /**< register write to readonly register */
  hyplnk_RET_DBG_WRITE_OVERFLOW, /**< write value too big for bitfield */
#endif
  hyplnk_RET_OK = 0,        /**< Call succeeded */
  hyplnk_RET_INV_HANDLE,    /**< Invalid handle */
  hyplnk_RET_INV_DEVICENUM, /**< @ref Hyplnk_open deviceNum invalid */
  hyplnk_RET_INV_INITCFG,   /**< Invalid Hyplnk_InitCfg */
  hyplnk_RET_NO_INIT        /**< Forgot to call Hyplnk_init() ? */
} hyplnkRet_e;
/* @} */

/**
 *  @ingroup hyplnklld_api_constants
 *
 *  @{
 */
 /** Selects whether to query or modify the local or remote HyperLink peripheral. */
typedef enum 
{
  hyplnk_LOCATION_LOCAL,     /**< Access the local HyperLink peripheral */
  hyplnk_LOCATION_REMOTE     /**< Access the remote HyperLink peripheral */
} hyplnkLocation_e;
/* @} */

/**
 *  @ingroup hyplnklld_api_constants
 *
 *  @{
 */
 /** Specifies what portion of the Hyperlink peripheral is changed */
typedef enum 
{
  hyplnk_CONFIG_TYPE_TX_ADDR,  /**< Configures the TX address mapping */
  hyplnk_CONFIG_TYPE_RX_ADDR,  /**< Configures one or more RX address maps */
  hyplnk_CONFIG_CONTROL        /**< Configures the rest of the registers */
} hyplnkConfigType_e;
/* @} */


/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the HyperLink Revision Register
 *
 * The Revision Register contains the major and minor revisions 
 * for the HyperLink module.
 * 
 * @{
 */
typedef struct hyplnkRevReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Scheme 
   *
   * Field size: 2 bits
   */
  uint8_t scheme;
  /**
   * @brief [ro] Business Unit
   *
   * Field size: 2 bits
   */
  uint8_t bu;
  /**
   * @brief [ro] Function identifier
   *
   * 0xe90 is HyperLink
   *
   * Field size: 12 bits
   */
  uint16_t func;
  /**
   * @brief [ro] RTL Version
   *
   * Field size: 5 bits
   */
  uint8_t rtl;
  /**
   * @brief [ro] Customer special version
   *
   * Field size: 2 bits
   */
  uint8_t cust;
  /**
   * @brief [ro] Major revision
   *
   * Field size: 3 bits
   */
  uint8_t revMaj;
  /**
   * @brief [ro] Minor revision
   *
   * Field size: 6 bits
   */
  uint8_t revMin;
} hyplnkRevReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the HyperLink Control Register
 *
 * The Control Register determines operation of the HyperLink module.
 *
 * @{
 */
typedef struct hyplnkControlReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Interrupt Local: set to 1 to keep interrupts local
   *
   * Field size: 1 bit
   *
   * This bit determines whether interrupts are posted in the 
   * Interrupt Status/Clear Register or forwarded via the serial interface. 
   * When set, interrupts are posted in @ref hyplnkIntStatusClrReg_s.
   * When clear, interrupts are forwarded out the serial interface to the 
   * remote device.
   */
  uint8_t intLocal;
  /**
   * @brief [rw] Module Status (Error) Interrupt Enable. 
   *
   * Field size: 1 bit
   *
   * This bit causes HyperLink module status interrupts to be posted to 
   * @ref hyplnkIntPendSetReg_s.
   * 
   * When this bit is enabled and one of the 
   * @ref hyplnkStatusReg_t::rError and @ref hyplnkStatusReg_t::lError fields
   * become set, then this bit decides whether an interrupt is generated
   */
  uint8_t statusIntEnable;
  /**
   * @brief [rw] Vector (number) used by the Module Status (Error) Interrupt
   *        
   * Field size: 5 bits
   *
   * When a Module Status interrupt allowed by @ref intLocal occurs, 
   * specifies which interrupt number in the Interrupt Pending/Set Register 
   * will be triggered.
   */
  uint8_t statusIntVec;
  /**
   * @brief [rw] Selects whether to use @ref hyplnkIntStatusClrReg_s.
   *
   * Field size: 1 bit
   *
   * When this bit is set, @ref hyplnkIntStatusClrReg_s is written 
   * directly with the status contained in interrupt packets. When this bit is 
   * set, the least significant 8 bits of the Interrupt Pointer Register are 
   * used to point to a local configuration register (typically the Interrupt 
   * Pending/Set Register). When clear, bits [31-2] of the Interrupt Pointer 
   * Register are used by the VBUSM Master interface as the address of interrupt
   * set register.
   */
  uint8_t int2cfg;
  /**
   * @brief [rw] Stops/kills all pending transactions then stops serial tx/rx.
   *
   * Field size: 1 bit
   *
   * When set will disable all partial or remote register operation, and 
   * error them with an bad_address status. This bit should be set before 
   * @ref iLoop or @ref reset bits are changed.
   */
  uint8_t serialStop;
  /**
   * @brief [rw] Internal (serial level) loopback enable
   *
   * Field size: 1 bit
   *
   * This bit when set causes the serial transmit data to be wrapped back 
   * to the serial receive data.  When changing this bit, it is recommended 
   * that the serial_stop bit be set and that all outstanding transactions have
   * completed before the iloop bit is changes at which time the @ref serialStop 
   * bit can be cleared to resume normal operation.
   */
  uint8_t iLoop;
  /**
   * @brief [rw] Resets the peripheral
   *
   * Field size: 1 bit
   *
   * When this bit is set, all internal state machines are reset, the 
   * serial interface is disabled, and link is lost.  
   *
   * Note: Any bus transaction in flight between the devices will be lost. 
   * When changing this bit, it is recommended that the 
   * @ref serialStop bit be set and that all outstanding 
   * transactions have completed @ref hyplnkStatusReg_t::rPend
   * before the reset bit is set.  After the @ref reset bit is cleared 
   * the @ref serialStop bit can be cleared to resume normal operation.
   */
  uint8_t reset;
} hyplnkControlReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the HyperLink Status Register
 *
 * The Status Register is used to detect conditions that may be of 
 * interest to the system designer
 * 
 * @{
 */
typedef struct hyplnkStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Size of the inbound serial data capability. 
   *
   * Field size: 4 bits
   */
  uint8_t swidthin;
  /**
   * @brief [ro] Size of the outbound serial data capability. 
   *
   * Field size: 4 bits
   */
  uint8_t swidthout;
  /**
   * @brief [ro] Serial logic in halted state
   *
   * Field size: 1 bit
   *
   * This could be because @ref hyplnkControlReg_t::reset, 
   * @ref hyplnkControlReg_t::serialStop, or @ref pllUnlock are active 
   */
  uint8_t serialHalt;
  /**
   * @brief [ro] The SerDes PLL is not locked
   *
   * Field size: 1 bit
   *
   * This disables all serial operations
   */
  uint8_t pllUnlock;
  /**
   * @brief [ro] Remote Request Pending
   *
   * Field size: 1 bit
   *
   * The user should monitor this bit after setting 
   * @ref hyplnkControlReg_t::serialStop and before 
   * changing @ref hyplnkControlReg_t::iLoop or 
   * @ref hyplnkControlReg_t::reset bits
   * 
   */
  uint8_t rPend;
  /**
   * @brief [ro] Inbound flow control is blocking outbound data
   *
   * Field size: 1 bit
   *
   * Indicates that a flow control enable request has been received 
   * and has stalled transmit until a flow control disable request is received.
   */ 
  uint8_t iFlow;
  /**
   * @brief [ro] Outbound flow control has been requested
   *
   * Field size: 1 bit
   *
   * Indicates that a flow control enable request has been received 
   * and has stalled transmit until a flow control disable request is received.
   */ 
  uint8_t oFlow;
  /**
   * @brief [rw] Remote uncorrectable error
   *
   * Field size: 1 bit
   *
   * This bit indicates that a downstream HyperLink module 
   * has detected an uncorrectable ECC error. This bit is set when an ECC 
   * status is received from the management interface. This bit is cleared 
   * by writing a one to it and the remote serial interface has been reset. 
   *
   * When this bit is set by the peripheral, an interrupt will occur if 
   * enabled in @ref hyplnkControlReg_t::statusIntEnable.  The remote device 
   * will need to perform a serial reset @ref hyplnkControlReg_t::reset or 
   * device reset to recover from this catastrophic failure.  Since this 
   * indicates that a transaction has been lost, if the @ref rPend bit is 
   * set along with this bit, it is possible that this device will also 
   * require a reset as a transaction from this device may have been lost.
   */
  uint8_t rError;
  /**
   * @brief [rw] Local uncorrectable error. 
   *
   * Field size: 1 bit
   *
   * This bit indicates that an inbound packet contains an uncorrectable 
   * ECC error. This bit is cleared by writing a one to it after the error 
   * has been corrected. When set by the peripheral, this bit will cause an 
   * interrupt if enabled in @ref hyplnkControlReg_t::statusIntEnable.
   *
   * This error indicate a catastrophic failure and that the receive link 
   * is down. The @ref hyplnkControlReg_t::reset bit must be toggled to 
   * recover from this error.  It is possible that returning transactions 
   * may have been lost in which case a device reset maybe necessary to recover.
   */
  uint8_t lError;
  /** 
   * @brief [ro] FIFO 3 (Slave Commands) is not empty 
   *
   * Field size: 1 bit
   */
  uint8_t nfEmpty3;
  /** 
   * @brief [ro] FIFO 2 (Slave Data) is not empty 
   *
   * Field size: 1 bit
   */
  uint8_t nfEmpty2;
  /** 
   * @brief [ro] FIFO 1 (Master Commands) is not empty 
   *
   * Field size: 1 bit
   */
  uint8_t nfEmpty1;
  /** 
   * @brief [ro] FIFO 0 (Master Data) is not empty 
   *
   * Field size: 1 bit
   */
  uint8_t nfEmpty0;
  /** 
   * @brief [ro] There are pending slave requests 
   *
   * Field size: 1 bit
   */
  uint8_t sPend;
  /** 
   * @brief [ro] There are pending master requests 
   *
   * Field size: 1 bit
   */
  uint8_t mPend;
  /**
   * @brief [ro] The serial interface is working 
   *
   * Field size: 1 bit
   */
  uint8_t link;
} hyplnkStatusReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Hyperlink Interrupt Priority Vector Status/Clear Register
 *
 * When read, the Interrupt Priority Vector Status/Clear register 
 * displays the highest priority vector with a pending interrupt. 
 * When writing, only @ref hyplnkIntPriVecReg_s::intStat "intStat" is valid, 
 * and the value represents the vector of the interrupt to be cleared.
 * 
 * @{
 */
typedef struct hyplnkIntPriVecReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] 1 indicates NO interrupts are in @ref hyplnkIntStatusClrReg_s 
   *
   * Field size: 1 bit
   */
  uint8_t noIntPend; /**< @brief [ro] 1 indicates NO interrupts are in @ref hyplnkIntStatusClrReg_s */
  /**
   * @brief [rw] Interrupt Priority Vector Status
   * 
   * Field size: 5 bits
   *
   * When read, this field displays the vector that is mapped to the highest 
   * priority interrupt bit that is pending from @ref hyplnkIntStatusClrReg_s, 
   * with Bit 0 as the highest priority, and Bit 31 as the lowest. Writing back 
   * the vector value into this field will clear the interrupt.
   */ 
  uint8_t intStat; 
} hyplnkIntPriVecReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Hyperlink Interrupt Status/Clear Register
 *
 * The Interrupt Status/Clear Register indicates the unmasked interrupt 
 * status. Writing 1 to any bit in this register will clear the 
 * corresponding interrupt.
 * 
 * @{
 */
typedef struct hyplnkIntStatusClrReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Interrupt Status/Clear Bitfield
   *
   * Field size: 32 bits
   *
   * Each bit represents the unmasked status of its respective interrupt.  Writing
   * a 1 to a particular bit will clear it.
   */
  uint32_t intClr;
} hyplnkIntStatusClrReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Hyperlink Interrupt Pending/Set Register
 *
 * The Interrupt Pending/Set Register indicates the pending interrupt 
 * status. This register can be written by the local host or by remote 
 * interrupt packets when the @ref hyplnkControlReg_s::int2cfg bit is
 * set. When bits are set in this register, an interrupt output is signaled,
 * if not already pending. Any write with a value of 0x0000 to this register 
 * will EOI the @ref hyplnkIntPendSetReg_s interrupt output pins so they 
 * retrigger the interrupt. That is, writing a zero to @ref hyplnkIntPendSetReg_s
 * retriggers the output interrupt lines if any bits are still set in this 
 * register.
 * 
 * @{
 */
typedef struct hyplnkIntPendSetReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Interrupt Pending/Set Bitfield
   *
   * Field size: 32 bits
   *
   * Each bit represents the unmasked status of its respective pending 
   * interrupt.  Writing a 1 to a particular bit will activate it.
   */
  uint32_t intSet;
} hyplnkIntPendSetReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Hyperlink Generate Soft Interrupt Value Register
 *
 * The Generate Soft Interrupt Register should be written with a vector 
 * of the hardware index of the interrupt that is enabled for software 
 * interrupts. If the @ref hyplnkIntCtrlValReg_s::iSec bit is also set,
 * the csecure interface pin must be set to set the software interrupt. 
 * This register is also used to EOI Hyperlink_int_i hardware interrupts 
 * programmed in level mode.
 *
 * @{
 */
typedef struct hyplnkGenSoftIntReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Force end of interrupt
   *
   * Field size: 8 bits
   *
   * Reads as 0.
   *
   * Writing 0xff to @ref eoiFlag and @ref iVector will deassert (EOI) all 
   * level sensitive interrupts.
   *
   * Writing 0xfe to @ref eoiFlag and an interrupt number to @ref iVector
   * will deassert (EOI) the specified interrupt.
   *
   * Writing 0 to the @ref eoiFlag will assert the interrupt specified
   * by @ref iVector.
   */
  uint8_t eoiFlag;
  /**
   * @brief [rw] Interrupt number to assert or deassert (EOI)
   *
   * Field size: 8 bits
   *
   * Hardware index for the interrupt that will be asserted or deasserted. 
   * If the @ref hyplnkIntCtrlValReg_s::SIEN bit is set
   * and the security level is met, the internal hardware pending bit will be set. 
   * This interrupt is forwarded based on @ref hyplnkControlReg_t::intLocal.
   */ 
  uint8_t iVector;
} hyplnkGenSoftIntReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Tx Address Overlay Control Register
 *
 * The Tx Address Map Mask Register is used to trim the transmitted packet 
 * address to remote device VBUSM addresses.
 *
 * @{
 */
typedef struct hyplnkTXAddrOvlyReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Secure bit location in outgoing address
   *
   * Field size: 4 bits
   *
   * This is the inverse of @ref hyplnkRXAddrSelReg_s::rxSecSel
   * 
   * <TABLE>
   * <TR><TH>@ref txSecOvl</TH><TH>Secure bit position in outgoing addr</TH></TR>
   * <TR><TD>0</TD>  <TD>none</TD></TR>
   * <TR><TD>1</TD>  <TD>17</TD></TR>
   * <TR><TD>2</TD>  <TD>18</TD></TR>
   * <TR><TD>3</TD>  <TD>19</TD></TR>
   * <TR><TD>4</TD>  <TD>20</TD></TR>
   * <TR><TD>5</TD>  <TD>21</TD></TR>
   * <TR><TD>6</TD>  <TD>22</TD></TR>
   * <TR><TD>7</TD>  <TD>23</TD></TR>
   * <TR><TD>8</TD>  <TD>24</TD></TR>
   * <TR><TD>9</TD>  <TD>25</TD></TR>
   * <TR><TD>10</TD> <TD>26</TD></TR>
   * <TR><TD>11</TD> <TD>27</TD></TR>
   * <TR><TD>12</TD> <TD>28</TD></TR>
   * <TR><TD>13</TD> <TD>29</TD></TR>
   * <TR><TD>14</TD> <TD>30</TD></TR>
   * <TR><TD>15</TD> <TD>31</TD></TR>
   * </TABLE>
   */
  uint8_t txSecOvl;
  /**
   * @brief [rw] PrivID bit location in outgoing address
   *
   * Field size: 4 bits
   *
   * This is the inverse of @ref hyplnkRXAddrSelReg_s::rxPrivIDSel
   * 
   * <TABLE>
   * <TR><TH>@ref txPrivIDOvl</TH><TH>Outgoing Address bits</TH><TH>PrivID bits</TH></TR>
   * <TR><TD>0</TD>  <TD>none</TD>     <TD>none</TD></TR>
   * <TR><TD>1</TD>  <TD>[20-17]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>2</TD>  <TD>[21-18]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>3</TD>  <TD>[22-19]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>4</TD>  <TD>[23-20]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>5</TD>  <TD>[24-21]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>6</TD>  <TD>[25-22]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>7</TD>  <TD>[26-23]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>8</TD>  <TD>[27-24]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>9</TD>  <TD>[28-25]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>10</TD> <TD>[29-26]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>11</TD> <TD>[30-27]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>12</TD> <TD>[31-28]</TD>  <TD>[3-0]</TD></TR>
   * <TR><TD>13</TD> <TD>[31-29]</TD>  <TD>[2-0]</TD></TR>
   * <TR><TD>14</TD> <TD>[31-30]</TD>  <TD>[1-0]</TD></TR>
   * <TR><TD>15</TD> <TD>[31]</TD>     <TD>[0]</TD></TR>
   * </TABLE>
   */
  uint8_t txPrivIDOvl;
  /**
   * @brief [rw] Mask applied to each address before transmitting
   *
   * Field size: 4 bits
   *
   * <TABLE>
   * <TR><TH>@ref txIgnMask</TH><TH>AND-mask</TH></TR>
   * <TR><TD>0</TD>  <TD>0x0001FFFF</TD></TR>
   * <TR><TD>1</TD>  <TD>0x0003FFFF</TD></TR>
   * <TR><TD>2</TD>  <TD>0x0007FFFF</TD></TR>
   * <TR><TD>3</TD>  <TD>0x000FFFFF</TD></TR>
   * <TR><TD>4</TD>  <TD>0x001FFFFF</TD></TR>
   * <TR><TD>5</TD>  <TD>0x003FFFFF</TD></TR>
   * <TR><TD>6</TD>  <TD>0x007FFFFF</TD></TR>
   * <TR><TD>7</TD>  <TD>0x00FFFFFF</TD></TR>
   * <TR><TD>8</TD>  <TD>0x01FFFFFF</TD></TR>
   * <TR><TD>9</TD>  <TD>0x03FFFFFF</TD></TR>
   * <TR><TD>10</TD> <TD>0x07FFFFFF</TD></TR>
   * <TR><TD>11</TD> <TD>0x0FFFFFFF</TD></TR>
   * <TR><TD>12</TD> <TD>0x1FFFFFFF</TD></TR>
   * <TR><TD>13</TD> <TD>0x3FFFFFFF</TD></TR>
   * <TR><TD>14</TD> <TD>0x7FFFFFFF</TD></TR>
   * <TR><TD>15</TD> <TD>0xFFFFFFFF</TD></TR>
   * </TABLE>
   */
  uint8_t txIgnMask;
} hyplnkTXAddrOvlyReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Rx Address Selector Control Register
 *
 * The Rx Address Selector Control Register is used to configure 
 * which Rx Address bits select the Secure, PrivID and Segment/Length 
 * value arrays. This register also holds the secure value when the 
 * secure selection is one or zero.
 *
 * @image html address_translation_RX.jpg
 * 
 * @{
 */
typedef struct hyplnkRXAddrSelReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief Value of secure signal when bit referenced by @ref rxSecSel is 1
   *
   * Field size: 1 bit
   */
  uint8_t rxSecHi;
  /**
   * @brief Value of secure signal when bit referenced by @ref rxSecSel is 0
   *
   * Field size: 1 bit
   */
  uint8_t rxSecLo;
  /**
   * @brief Select RX address bit used as Secure bit.
   *
   * Field size: 4 bits
   *
   * This is the inverse of @ref hyplnkTXAddrOvlyReg_s::txSecOvl
   *
   * The secure signal is determined by selecting the bit specified by
   * @ref rxSecSel from the incoming address, then deciding the signal value
   * by apply @ref rxSecHi when that address bit is 1, or @ref rxSecLo when that
   * address bit is 0.
   *
   * <TABLE>
   * <TR><TH>@ref rxSecSel</TH><TH>Secure bit position in incoming addr</TH></TR>
   * <TR><TD>0</TD>  <TD>16</TD></TR>
   * <TR><TD>1</TD>  <TD>17</TD></TR>
   * <TR><TD>2</TD>  <TD>18</TD></TR>
   * <TR><TD>3</TD>  <TD>19</TD></TR>
   * <TR><TD>4</TD>  <TD>20</TD></TR>
   * <TR><TD>5</TD>  <TD>21</TD></TR>
   * <TR><TD>6</TD>  <TD>22</TD></TR>
   * <TR><TD>7</TD>  <TD>23</TD></TR>
   * <TR><TD>8</TD>  <TD>24</TD></TR>
   * <TR><TD>9</TD>  <TD>25</TD></TR>
   * <TR><TD>10</TD> <TD>26</TD></TR>
   * <TR><TD>11</TD> <TD>27</TD></TR>
   * <TR><TD>12</TD> <TD>28</TD></TR>
   * <TR><TD>13</TD> <TD>29</TD></TR>
   * <TR><TD>14</TD> <TD>30</TD></TR>
   * <TR><TD>15</TD> <TD>31</TD></TR>
   * </TABLE>
   */
  uint8_t rxSecSel;
  /**
   * @brief [rw] The RX address bits to use as PrivID lookup
   *
   * Field size: 4 bits
   *
   * The PrivID index is extracted from the specified incoming address bits.
   * The index is then looked up in the table defined through
   * @ref hyplnkRXPrivIDIdxReg_s and @ref hyplnkRXPrivIDValReg_s
   *
   * This is the inverse of @ref hyplnkTXAddrOvlyReg_s::txPrivIDOvl
   *
   * <TABLE>
   * <TR><TH>@ref rxPrivIDSel</TH><TH>PrivID index</TH></TR>
   * <TR><TD>0</TD>  <TD>0</TD></TR>
   * <TR><TD>1</TD>  <TD>RxAddress[20-17]</TD></TR>
   * <TR><TD>2</TD>  <TD>RxAddress[21-18]</TD></TR>
   * <TR><TD>3</TD>  <TD>RxAddress[22-19]</TD></TR>
   * <TR><TD>4</TD>  <TD>RxAddress[23-20]</TD></TR>
   * <TR><TD>5</TD>  <TD>RxAddress[24-21]</TD></TR>
   * <TR><TD>6</TD>  <TD>RxAddress[25-22]</TD></TR>
   * <TR><TD>7</TD>  <TD>RxAddress[26-23]</TD></TR>
   * <TR><TD>8</TD>  <TD>RxAddress[27-24]</TD></TR>
   * <TR><TD>9</TD>  <TD>RxAddress[28-25]</TD></TR>
   * <TR><TD>10</TD> <TD>RxAddress[29-26]</TD></TR>
   * <TR><TD>11</TD> <TD>RxAddress[30-27]</TD></TR>
   * <TR><TD>12</TD> <TD>RxAddress[31-28]</TD></TR>
   * <TR><TD>13</TD> <TD>RxAddress[31-29]</TD></TR>
   * <TR><TD>14</TD> <TD>RxAddress[31-30]</TD></TR>
   * <TR><TD>15</TD> <TD>RxAddress[31]</TD></TR>
   * </TABLE>
   */
  uint8_t rxPrivIDSel;
  /**
   * @brief [rw] Segment and Offset Selector
   *
   * Field size: 4 bits
   *
   * The segment number and offset are extracted from the incoming
   * address as specified in the following table.
   *
   * The segment number is then looked up in @ref hyplnkRXSegValReg_s
   *
   * This is the inverse of @ref hyplnkTXAddrOvlyReg_s::txIgnMask
   *
   * <TABLE>
   * <TR><TH>@ref rxSegSel</TH><TH>Segment index</TH><TH>Offset mask</TH></TR>
   * <TR><TD>0</TD>  <TD>0</TD>                 <TD>0xFFFFFFFF</TD></TR>
   * <TR><TD>1</TD>  <TD>RxAddress[22-17]</TD>  <TD>0x0001FFFF</TD></TR>
   * <TR><TD>2</TD>  <TD>RxAddress[23-18]</TD>  <TD>0x0003FFFF</TD></TR>
   * <TR><TD>3</TD>  <TD>RxAddress[24-19]</TD>  <TD>0x0007FFFF</TD></TR>
   * <TR><TD>4</TD>  <TD>RxAddress[25-20]</TD>  <TD>0x000FFFFF</TD></TR>
   * <TR><TD>5</TD>  <TD>RxAddress[26-21]</TD>  <TD>0x001FFFFF</TD></TR>
   * <TR><TD>6</TD>  <TD>RxAddress[27-22]</TD>  <TD>0x003FFFFF</TD></TR>
   * <TR><TD>7</TD>  <TD>RxAddress[28-23]</TD>  <TD>0x007FFFFF</TD></TR>
   * <TR><TD>8</TD>  <TD>RxAddress[29-24]</TD>  <TD>0x00FFFFFF</TD></TR>
   * <TR><TD>9</TD>  <TD>RxAddress[30-25]</TD>  <TD>0x01FFFFFF</TD></TR>
   * <TR><TD>10</TD> <TD>RxAddress[31-26]</TD>  <TD>0x03FFFFFF</TD></TR>
   * <TR><TD>11</TD> <TD>RxAddress[31-17]</TD>  <TD>0x07FFFFFF</TD></TR>
   * <TR><TD>12</TD> <TD>RxAddress[31-28]</TD>  <TD>0x0FFFFFFF</TD></TR>
   * <TR><TD>13</TD> <TD>RxAddress[31-29]</TD>  <TD>0x1FFFFFFF</TD></TR>
   * <TR><TD>14</TD> <TD>RxAddress[31-30]</TD>  <TD>0x3FFFFFFF</TD></TR>
   * <TR><TD>15</TD> <TD>RxAddress[31]</TD>     <TD>0x7FFFFFFF</TD></TR>
   * </TABLE>
   */
  uint8_t rxSegSel;
} hyplnkRXAddrSelReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Rx Address PrivID Index Register
 *
 * @{
 */
typedef struct hyplnkRXPrivIDIdxReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Selects PrivID array element number 
   *
   * Field size: 4 bits
   *
   * The specified array element can be read or written via
   * @ref hyplnkRXPrivIDValReg_s::rxPrivIDVal after the index is 
   * written.
   */
  uint8_t rxPrivIDIdx;
} hyplnkRXPrivIDIdxReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Rx Address PrivID Value Register
 *
 * @{
 */
typedef struct hyplnkRXPrivIDValReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Selects PrivID value
   *
   * Field size: 4 bits
   *
   * Specifies the PrivID in the PrivID lookup table at the position
   * specified by @ref hyplnkRXPrivIDIdxReg_s::rxPrivIDIdx.  This
   * is used to dereference the PrivID from the address bits selected
   * by @ref hyplnkRXAddrSelReg_s::rxPrivIDSel.
   */
  uint8_t rxPrivIDVal;
} hyplnkRXPrivIDValReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Rx Address Segment Index Register
 *
 * @{
 */
typedef struct hyplnkRXSegIdxReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Selects segment array element number
   *
   * Field size: 6 bits
   *
   * The specified array element can be read or written via
   * @ref hyplnkRXSegValReg_s after the index is written.
   */
  uint8_t rxSegIdx;
} hyplnkRXSegIdxReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Rx Address Segment Value Register
 *
 * There is an array of 64 segment value registers.  The particular
 * segment value register accessed via @ref hyplnkRXSegValReg_s
 * is specified in @ref hyplnkRXSegIdxReg_s::rxSegIdx.
 *
 * After the segment index is extracted from the incoming address
 * based on @ref hyplnkRXAddrSelReg_s::rxSegSel, the index
 * is dereferenced through this table to get a base address and
 * length.
 *
 * The local address (which will be presented through the SES
 * or SMS MPAX) is constructed by:
 *
 * ((@ref hyplnkRXSegValReg_s::rxSegVal "rxSegVal" << 16) | RxAddress) & 
 * ((1 << (1 + @ref hyplnkRXSegValReg_s::rxLenVal "rxLenVal")) - 1)
 * @{
 */
typedef struct hyplnkRXSegValReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Segment address value (base address)
   *
   * Field size: 16 bits
   */
  uint16_t rxSegVal;
  /**
   * @brief [rw] Selects segment size
   *
   * Field size: 5 bits
   *
   * <TABLE>
   * <TR><TH>@ref rxLenVal</TH><TH>Segment Size</TH></TR>
   * <TR><TD>0-7</TD> <TD>0</TD></TR>
   * <TR><TD>8</TD>   <TD>0x0 0000 0200</TD></TR>
   * <TR><TD>9</TD>   <TD>0x0 0000 0400</TD></TR>
   * <TR><TD>10</TD>  <TD>0x0 0000 0800</TD></TR>
   * <TR><TD>11</TD>  <TD>0x0 0000 1000</TD></TR>
   * <TR><TD>12</TD>  <TD>0x0 0000 2000</TD></TR>
   * <TR><TD>13</TD>  <TD>0x0 0000 4000</TD></TR>
   * <TR><TD>14</TD>  <TD>0x0 0000 8000</TD></TR>
   * <TR><TD>15</TD>  <TD>0x0 0001 0000</TD></TR>
   * <TR><TD>16</TD>  <TD>0x0 0002 0000</TD></TR>
   * <TR><TD>17</TD>  <TD>0x0 0004 0000</TD></TR>
   * <TR><TD>18</TD>  <TD>0x0 0008 0000</TD></TR>
   * <TR><TD>19</TD>  <TD>0x0 0010 0000</TD></TR>
   * <TR><TD>20</TD>  <TD>0x0 0020 0000</TD></TR>
   * <TR><TD>21</TD>  <TD>0x0 0040 0000</TD></TR>
   * <TR><TD>22</TD>  <TD>0x0 0080 0000</TD></TR>
   * <TR><TD>23</TD>  <TD>0x0 0100 0000</TD></TR>
   * <TR><TD>24</TD>  <TD>0x0 0200 0000</TD></TR>
   * <TR><TD>25</TD>  <TD>0x0 0400 0000</TD></TR>
   * <TR><TD>26</TD>  <TD>0x0 0800 0000</TD></TR>
   * <TR><TD>27</TD>  <TD>0x0 1000 0000</TD></TR>
   * <TR><TD>28</TD>  <TD>0x0 2000 0000</TD></TR>
   * <TR><TD>29</TD>  <TD>0x0 4000 0000</TD></TR>
   * <TR><TD>30</TD>  <TD>0x0 8000 0000</TD></TR>
   * <TR><TD>31</TD>  <TD>0x1 0000 0000</TD></TR>
   * </TABLE>
   */
  uint8_t  rxLenVal;
} hyplnkRXSegValReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Chip Version Register
 *
 * @{
 */
typedef struct hyplnkChipVerReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Device Revision
   *
   * Field size: 16 bits
   *
   * Reflects value of device_rev pins (not brought out to chip pins)
   */
  uint16_t devRev;
  /**
   * @brief [ro] Device ID
   *
   * Field size: 16 bits
   *
   * Reflects value of device_id pins (not brought out to chip pins)
   */
  uint16_t devID;
} hyplnkChipVerReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Lane Power Management Control Register
 *
 * The Power Management Control Register configures how the HyperLink
 * peripheral dynamically changes the number of lanes to save power.
 * 
 * @{
 */
typedef struct hyplnkLanePwrMgmtReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] High to Low clocks
   *
   * Field size: 3 bits
   *
   * Number of clocks [in peripheral domain] that the FIFO falls below 1/4 
   * rate before a transition from high speed to low speed is taken.
   *
   * <TABLE>
   * <TR><TH>@ref H2L</TH><TH>Clocks</TH></TR>
   * <TR><TD>0</TD> <TD>64</TD></TR>
   * <TR><TD>1</TD> <TD>128</TD></TR>
   * <TR><TD>2</TD> <TD>256</TD></TR>
   * <TR><TD>3</TD> <TD>512</TD></TR>
   * <TR><TD>4</TD> <TD>1024</TD></TR>
   * <TR><TD>5</TD> <TD>2048</TD></TR>
   * <TR><TD>6</TD> <TD>4096</TD></TR>
   * <TR><TD>7</TD> <TD>8192</TD></TR>
   * </TABLE>
   *
   */
  uint8_t H2L;
  /**
   * @brief [rw] Low to High clocks
   *
   * Field size: 3 bits
   *
   * Number of clocks [in peripheral domain] that the FIFO is busy before 
   * a transition to high speed is taken.
   *
   * <TABLE>
   * <TR><TH>@ref L2H</TH><TH>Clocks</TH></TR>
   * <TR><TD>0</TD> <TD>64</TD></TR>
   * <TR><TD>1</TD> <TD>128</TD></TR>
   * <TR><TD>2</TD> <TD>256</TD></TR>
   * <TR><TD>3</TD> <TD>512</TD></TR>
   * <TR><TD>4</TD> <TD>1024</TD></TR>
   * <TR><TD>5</TD> <TD>2048</TD></TR>
   * <TR><TD>6</TD> <TD>4096</TD></TR>
   * <TR><TD>7</TD> <TD>8192</TD></TR>
   * </TABLE>
   */
  uint8_t L2H;
  /**
   * @brief [rw] Periodic Wakeup Control
   *
   * Field size: 8 bits
   *
   * The wakeup period is (@ref PWC + 1) * 65536 SERDES clocks.
   * 
   * The expected value is 94 (10ms).  
   *
   * When the timer expires, the SerDes will be woken up at full speed. 
   * If the SerDes is operated at full speed due to other reasons, the
   * timer will start over. That is this controls the maximum interval between 
   * full speed events.
   *
   */
  uint8_t PWC;
  /**
   * @brief [rw] or [ro] High Speed operation is possible
   *
   * Field size: 1 bits
   *
   * It is only settable when using the 45nm SerDes. Otherwise it is
   * a [ro] bit with a value of zero.
   */
  uint8_t highSpeed;
  /**
   * @brief [rw] Quad lane enable
   *
   * Field size: 1 bits
   *
   * Setting this bit allows four lane capability.  Clearing this bit
   * forces one lane operation.
   *
   * (Automatic shifting between 0, 1 and 4 lane mode happens based on
   * the configuration of @ref H2L and @ref L2H)
   */
  uint8_t quadLane;
  /**
   * @brief [rw] Single lane enable
   *
   * Field size: 1 bits
   *
   * Setting this bit allows one lane operation.  Clearing this bit
   * forces four lane operation.
   *
   * (Automatic shifting between 0, 1 and 4 lane mode happens based on
   * the configuration of @ref H2L and @ref L2H)
   */
  uint8_t singleLane;
  /**
   * @brief [rw] Zero lane enable
   *
   * Field size: 1 bits
   *
   * Setting this bit allows zero lane operation.  Zero lane operation
   * shuts down the SERDES.
   *
   * Clearing this bit prohibits zero lane operation.
   */
  uint8_t zeroLane;
} hyplnkLanePwrMgmtReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the ECC Error Counters Register
 *
 * The ECC Error Counter register counts the number of correctable 
 * single bit errors detected by the receive PLS as well as the number 
 * of detectable double bit errors. This value can be used to determine 
 * the integrity of the SerDes Rx signal. Writing to this register clears 
 * the current counts to zero.
 * 
 * @{
 */
typedef struct hyplnkECCErrorsReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Corrected Single Error Counter
   *
   * Field size: 16 bits
   *
   * Counter is incremented when a correctable error is detected by the 
   * Rx PLS layer.  Writing any value to this register will clear this counter.
   */
  uint16_t sglErrCor;
  /**
   * @brief [rw] Detected Double Error Counter
   *
   * Field size: 8 bits
   *
   * Counter is incremented when a detectable two bit error is detected 
   * by the Rx PLS layer. This indicates that the receive channel signal is
   * marginal. Writing any value to this register will clear this counter.
   */
  uint8_t  dblErrDet;
} hyplnkECCErrorsReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Link Status Register
 *
 * The Link status register is used to debug failed link conditions. 
 * It contains valuable information aoout the start of the link-state machines. 
 * It is used only to determine what might be causing the link failure. Because 
 * the source of this register can change quickly, this register updates only 
 * when a change has been detected and it is capable of transferring it to 
 * the bus clock domain.
 * 
 * @{
 */
typedef struct hyplnkLinkStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Number of lanes requested by PLS layer
   *
   * Field size: 2 bits
   *
   * <TABLE>
   * <TR><TH>@ref txPlsReq</TH><TH># of lanes</TH></TR>
   * <TR><TD>00b</TD> <TD>0</TD></TR>
   * <TR><TD>01b</TD> <TD>1</TD></TR>
   * <TR><TD>10b</TD> <TD>4</TD></TR>
   * <TR><TD>11b</TD> <TD>4</TD></TR>
   * </TABLE>
   */
  uint8_t txPlsReq;
  /**
   * @brief [ro] Number of lanes active by PLS layer
   *
   * Field size: 2 bits
   *
   * <TABLE>
   * <TR><TH>@ref txPlsAck</TH><TH># of lanes</TH></TR>
   * <TR><TD>00b</TD> <TD>0</TD></TR>
   * <TR><TD>01b</TD> <TD>1</TD></TR>
   * <TR><TD>10b</TD> <TD>4</TD></TR>
   * <TR><TD>11b</TD> <TD>4</TD></TR>
   * </TABLE>
   */
  uint8_t txPlsAck;
  /**
   * @brief [ro] TX Power Management Sideband Control State
   *
   * Field size: 2 bits
   *
   * <TABLE>
   * <TR><TH>@ref txPmReq</TH><TH># of lanes</TH></TR>
   * <TR><TD>00b</TD> <TD>0</TD></TR>
   * <TR><TD>01b</TD> <TD>1</TD></TR>
   * <TR><TD>10b</TD> <TD>4, transitioning to/from 0 lanes</TD></TR>
   * <TR><TD>11b</TD> <TD>4</TD></TR>
   * </TABLE>
   */
  uint8_t txPmReq;
  /**
   * @brief [ro] Remote device has synced to the transmit traning sequence
   *
   * Field size: 1 bits
   *
   * This is a latched version of the station management sync bit.
   */
  uint8_t txRSync;
  /**
   * @brief [ro] TX PLS layer is linked to the remote device
   *
   * Field size: 1 bits
   */
  uint8_t txPlsOK;
  /**
   * @brief [ro] Indicates which SERDES lanes are enabled
   *
   * Field size: 4 bits
   */
  uint8_t txPhyEn;
  /**
   * @brief [ro] Indicates which flow bit are set from the remote receiver.
   *
   * Only bits 0 and 1 are used.
   *
   * Field size: 4 bits
   */
  uint8_t txFlowSts;
  /**
   * @brief [ro] Number of lanes requested by PLS for RX
   *
   * Field size: 2 bits
   *
   * <TABLE>
   * <TR><TH>@ref rxPlsReq</TH><TH># of lanes</TH></TR>
   * <TR><TD>00b</TD> <TD>0</TD></TR>
   * <TR><TD>01b</TD> <TD>1</TD></TR>
   * <TR><TD>10b</TD> <TD>4</TD></TR>
   * <TR><TD>11b</TD> <TD>NS</TD></TR>
   * </TABLE>
   */
  uint8_t rxPlsReq;
  /**
   * @brief [ro] Number of lanes active by PLS for RX
   *
   * Field size: 2 bits
   *
   * <TABLE>
   * <TR><TH>@ref rxPlsAck</TH><TH># of lanes</TH></TR>
   * <TR><TD>00b</TD> <TD>0</TD></TR>
   * <TR><TD>01b</TD> <TD>1</TD></TR>
   * <TR><TD>10b</TD> <TD>4</TD></TR>
   * <TR><TD>11b</TD> <TD>4</TD></TR>
   * </TABLE>
   */
  uint8_t rxPlsAck;
  /**
   * @brief [ro] RX Power Management Sideband Control State
   *
   * Field size: 2 bits
   *
   * <TABLE>
   * <TR><TH>@ref rxPmReq</TH><TH># of lanes</TH></TR>
   * <TR><TD>00b</TD> <TD>0</TD></TR>
   * <TR><TD>01b</TD> <TD>1</TD></TR>
   * <TR><TD>10b</TD> <TD>4, transitioning to/from 0 lanes</TD></TR>
   * <TR><TD>11b</TD> <TD>4</TD></TR>
   * </TABLE>
   */
  uint8_t rxPmReq;
  /**
   * @brief [ro] Receive has synced to training sequence
   *
   * Field size: 1 bits
   *
   * This is the source of the station management sync bit.
   */
  uint8_t rxLSync;
  /**
   * @brief [ro] Lane zero has been identified during training
   *
   * Field size: 1 bits
   */
  uint8_t rxOneID;
  /**
   * @brief [ro] Indicates which SERDES lanes are enabled
   *
   * Field size: 4 bits
   */
  uint8_t rxPhyEn;
  /*
   * @brief [ro] Indicates which SERDES lanes are reversed polarity 
   *
   * Field size: 4 bits
   */
  uint8_t rxPhyPol;
} hyplnkLinkStatusReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Interupt Control Index Register
 *
 * The Interrupt Control Index Register is used to control which 
 * hardware or software internal control register is read or written 
 * via hyplnkIntCtrlValReg_s.
 *
 * @{
 */
typedef struct hyplnkIntCtrlIdxReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Selects interrupt control value register
   *
   * Field size: 8 bits
   *
   * The specified array element can be read or written via
   * @ref hyplnkIntCtrlValReg_s after the index is written.
   */
  uint8_t intCtrlIdx;
} hyplnkIntCtrlIdxReg_t;
/* @} */


/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Interrupt Control Value Register
 *
 * The Interrupt Control Register reads or writes the associated fields to the 
 * @ref hyplnkIntCtrlIdxReg_s::intCtrlIdx "intCtrlIdx" interrupt channel. All channels not 
 * supported will return zero and be unsettable.
 * 
 * @{
 */
typedef struct hyplnkIntCtrlValReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Interrupt Enable
   *
   * Field size: 1 bit
   *
   * When set, this bit indicates that interrupts detected on the 
   * Hyperlink_int_i[@ref hyplnkIntCtrlIdxReg_s::intCtrlIdx "intCtrlIdx"] pin 
   * (not a physical device pin) should be should be forwarded to the 
   * (@ref DNID, @ref mps, @ref vector) interrupt vector.
   */
  uint8_t intEn;
  /**
   * @brief [rw] Interrupt Type (level or pulse)
   *
   * Field size: 1 bit
   *
   * When set, this bit indicates that the 
   * Hyperlink_int_i[@ref hyplnkIntCtrlIdxReg_s::intCtrlIdx "intCtrlIdx"] 
   * interrupt is pulsed. When clear, this bit indicates that 
   * Hyperlink_int_i[@ref hyplnkIntCtrlIdxReg_s::intCtrlIdx "intCtrlIdx"] 
   * is level sensitive.
   */
  uint8_t intType;
  /**
   * @brief [rw] Interrupt Polarity (active high or active low)
   *
   * Field size: 1 bit
   *
   * Interrupt Polarity When set, this bit indicates that this 
   * is active low. When clear, it is active high
   */
  uint8_t intPol;
  /**
   * @brief [rw] Interrupt security
   *
   * Field size: 1 bit
   *
   * Indicates the security level that the master must have to set a 
   * software interrupt for this interrupt.
   */
  uint8_t iSec;
  /** @brief [rw] Software interrupt enable
   *
   * Field size: 1 bit
   *
   * Software Interrupt Enable indicate if this interrupt can be issued 
   * via software writing to @ref hyplnkGenSoftIntReg_s.
   */
  uint8_t SIEN;
  /** @brief [rw] Destination Network Identifier
   *
   * Field size: 2 bits
   */
  uint8_t DNID;
  /** @brief [rw] Microprocessor Select
   *
   * Field size: 4 bits
   */
  uint8_t mps;
  /** @brief [rw] Selects which interrupt pending to set on interrupt assertion
   *
   * Field size: 5 bits
   *
   * When the local device has @ref hyplnkControlReg_s::intLocal set, this field indicates 
   * which bit of interrupt pending register (@ref hyplnkIntPendSetReg_s) to 
   * set. When the local device has (@ref hyplnkControlReg_s::intLocal "intLocal")
   * clear, this field is transferred to the remote device, which is used to 
   * indicate which bit of the interrupt pending register to set in the 
   * remote device.
   */
  uint8_t vector;
} hyplnkIntCtrlValReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Interupt Control Index Register
 *
 * The Interrupt Pointer Index Register is used to control which 
 * Interrupt Pointer Register is read or written via @ref hyplnkIntPtrIdxReg_s. 
 * The Interrupt Pointer Registers typically map to microprocessor interrupt 
 * controller set registers which get set to a one to interrupt that processor.
 * 
 * @{
 */
typedef struct hyplnkIntPtrIdxReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Selects which Interrupt Pointer is used
   *
   * Field size: 4 bits
   *
   * The interrupt pointer specified by @ref intPtrIdx can be read or written via
   * @ref hyplnkIntPtrValReg_s;
   */
  uint8_t intPtrIdx;
} hyplnkIntPtrIdxReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the Interrupt Control Value Register
 *
 * The Interrupt Pointer Value is used to write or read to the interrupt
 * pointer number (@ref hyplnkIntPtrIdxReg_s::intPtrIdx "intPtrIdx") 
 * This allows the remote interrupts to interrupt any 
 * MicroProcessor. Values written are stored in the Interrupt Pointer for 
 * that MicroProcessor.
 * 
 * @{
 */
typedef struct hyplnkIntPtrValReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Specifies an interrupt pointer
   *
   * Field size: 30 bits
   *
   * This allows the remote interrupts to interrupt any MicroProcessor. Values written 
   * are stored in the Interrupt Pointer for that MicroProcessor.
   */
  uint32_t intPtr;
} hyplnkIntPtrValReg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the SerDes Control And Status 1 Register
 *
 * The SerDes Control and Status 1 Register is used to define the mask time 
 * that the receive lane data is ignored after enabling the lane(s) from either 
 * a sleep or disabled state. The default numbers of these counters are not 
 * yet determined. When these counters are zero, there are no delays in link 
 * establishment. This register delays the start of link establishment or step 
 * up link by a number of symbol times sixteen.
 *
 * @{
 */
typedef struct hyplnkSERDESControl1Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Sleep mask count
   *
   * Field size: 8 bits
   *
   * This count times 16 SerDes symbol times are masked for SerDes 
   * lanes that enter a sleep/enable state. This allows the internal 
   * SerDes power supplies to stabilize before the link is established.
   * 
   */
  uint8_t sleepCnt;
  /**
   * @brief [rw] Disable mask count
   *
   * Field size: 8 bits
   *
   * This count times 16 SerDes symbol times are masked for SerDes 
   * lanes that enter a disabled state. This allows the SerDes CDR and 
   * equalizer to stabilize before the link is established
   */
  uint8_t disableCnt;
} hyplnkSERDESControl1Reg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the SerDes Control And Status 2 Register
 *
 * This register is Reserved for SerDes control and status operations. 
 * There is no defined functionality for this register.
 * 
 * @{
 */
typedef struct hyplnkSERDESControl2Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Unused
   *
   * Field size: 16 bits 
   */
  uint16_t s2Ctl;
} hyplnkSERDESControl2Reg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the SerDes Control And Status 3 Register
 *
 * This register is Reserved for SerDes control and status operations. 
 * There is no defined functionality for this register.
 * 
 * @{
 */
typedef struct hyplnkSERDESControl3Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Unused
   *
   * Field size: 16 bits 
   */
  uint16_t s3Ctl;
} hyplnkSERDESControl3Reg_t;
/* @} */

/**
 * @ingroup hyplnklld_reg_structures
 * @brief Specification of the SerDes Control And Status 4 Register
 *
 * The SerDes Control and Status Register is used to quicken DV
 * so that the periodic wake-up timer can be tested more easily and to 
 * allow change to the SerDes non-runtime power levels. The power-level
 * controls were added to reduce SerDes sleep mode functionality risk. 
 * It provides the ability to enable or disable sleep functionality
 * for both the transmit and receive SerDes lanes.
 *
 * @{
 */
typedef struct hyplnkSERDESControl4Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Unused
   *
   * Field size: 11 bits
   */
  uint16_t s4Ctl;
  /**
   * @brief [rw] For testing purposes
   *
   * DVQUICK reduces the periodic wake up event prescaler to 256 clocks 
   * instead of 65,536 clocks. It allow for more exhaustive testing of the PWC.
   */
  uint8_t  dvQuick;
  /* 
   * @brief Transmit SERDES power control
   * 
   * The transmit SerDes power control is used to modify the SLEEP and enable 
   * characteristics of the SerDes. 01=Full Sleep configuration, 
   * 00=fast sleep configuration, 1X=No sleep configuration
   */
  uint8_t  txSpc;
  /* 
   * @brief Receive SERDES power control
   * 
   * The receive SerDes power control is used to modify the SLEEP and enable 
   * characteristics of the SerDes. 01=Full Sleep configuration, 
   * 00=fast sleep configuration, 1X=No sleep configuration
   */
  uint8_t  rxSpc;
} hyplnkSERDESControl4Reg_t;
/* @} */

/* By repeating the define here, we don't require cslr_vusr.h */
/* There is a compile time check in hyplnk.c to make sure it matches */
#define hyplnk_RX_PRIVID_TBL_ENTS 16 /**< Number of RX PrivID table entries */
/**
 * @ingroup hyplnklld_reg_structures
 * @brief Access to the entire RX PrivID table
 *
 * @{
 */
typedef hyplnkRXPrivIDValReg_t hyplnkRXPrivIDTbl_t[hyplnk_RX_PRIVID_TBL_ENTS];
/* @} */

/* By repeating the define here, we don't require cslr_vusr.h */
/* There is a compile time check in hyplnk.c to make sure it matches */
#define hyplnk_RX_SEG_TBL_ENTS 64 /**< Number of RX segment table entries */
/**
 * @ingroup hyplnklld_reg_structures
 * @brief Access to the entire RX segment table
 *
 * @{
 */
typedef hyplnkRXSegValReg_t hyplnkRXSegTbl_t[hyplnk_RX_SEG_TBL_ENTS];
/* @} */

/* By repeating the define here, we don't require cslr_vusr.h */
/* There is a compile time check in hyplnk.c to make sure it matches */
#define hyplnk_INT_CTRL_TBL_ENTS 256 /**< size of the interupt control table */
/**
 * @ingroup hyplnklld_reg_structures
 * @brief Access to the entire interrupt control table
 *
 * @{
 */
typedef hyplnkIntCtrlValReg_t hyplnkIntCtrlTbl_t[hyplnk_INT_CTRL_TBL_ENTS];
/* @} */

/* By repeating the define here, we don't require cslr_vusr.h */
/* There is a compile time check in hyplnk.c to make sure it matches */
#define hyplnk_INT_PTR_TBL_ENTS 16 /**< Size of the interrupt pointer table */
/**
 * @ingroup hyplnklld_reg_structures
 * @brief Access to the entire interrupt pointer table
 *
 * @{
 */
typedef hyplnkIntPtrValReg_t hyplnkIntPtrTbl_t[hyplnk_INT_PTR_TBL_ENTS];
/* @} */

/**
 * @ingroup hyplnklld_api_structures
 * @brief Specification all registers
 *
 * This structure allows one or more registers to be read or written
 * through a single call.  Also, all of the registers that operate on
 * tables can be programmed in one shot as well.
 *
 * The user populates one or more pointers to structures.  All structures
 * that are non-NULL are read or written.
 *
 * Once the pointers are populated, use @ref Hyplnk_readRegs and/or 
 * @ref Hyplnk_writeRegs to perform the actual register accesses
 *
 */
typedef struct hyplnkRegisters_s {
  hyplnkRevReg_t            *rev;            /**< @brief Revision */ 
  hyplnkControlReg_t        *control;        /**< @brief Control */
  hyplnkStatusReg_t         *status;         /**< @brief Status */
  hyplnkIntPriVecReg_t      *intPriVec;      /**< @brief Interrupt Priority Vector Status/Clear */
  hyplnkIntStatusClrReg_t   *intStatusClr;   /**< @brief Interrupt Status/Clear */
  hyplnkIntPendSetReg_t     *intPendSet;     /**< @brief Interrupt Pending/Set */
  hyplnkGenSoftIntReg_t     *genSoftInt;     /**< @brief Generate Soft Interrupt Value */
  hyplnkTXAddrOvlyReg_t     *TXAddrOvly;     /**< @brief TX Address Overlay */
  hyplnkRXAddrSelReg_t      *RXAddrSel;      /**< @brief RX Address Selector */
  hyplnkRXPrivIDIdxReg_t    *RXPrivIDIdx;    /**< @brief RX Address PrivID Index.  See also @ref RXPrivIDTbl */
  hyplnkRXPrivIDValReg_t    *RXPrivIDVal;    /**< @brief RX Address PrivID Value.  See also @ref RXPrivIDTbl */
  hyplnkRXSegIdxReg_t       *RXSegIdx;       /**< @brief RX Address Segment Index.  See also @ref RXSegTbl */
  hyplnkRXSegValReg_t       *RXSegVal;       /**< @brief RX Address Segment Value.  See also @ref RXSegTbl */
  hyplnkChipVerReg_t        *chipVer;        /**< @brief Chip Version */
  hyplnkLanePwrMgmtReg_t    *lanePwrMgmt;    /**< @brief Lane Power Management Control Register */
  hyplnkECCErrorsReg_t      *ECCErrors;      /**< @brief ECC Error Counters Register */
  hyplnkLinkStatusReg_t     *linkStatus;     /**< @brief Link status */
  hyplnkIntCtrlIdxReg_t     *intCtrlIdx;     /**< @brief Interrupt Control Index.  See also @ref intCtrlTbl */
  hyplnkIntCtrlValReg_t     *intCtrlVal;     /**< @brief Interrupt Control Value.  See also @ref intCtrlTbl */
  hyplnkIntPtrIdxReg_t      *intPtrIdx;      /**< @brief Interrupt Pointer Index.  See also @ref intPtrTbl */
  hyplnkIntPtrValReg_t      *intPtrVal;      /**< @brief Interrupt Pointer Value.  See also @ref intPtrTbl */
  hyplnkSERDESControl1Reg_t *serdesControl1; /**< @brief SERDES Control and Status #1 */
  hyplnkSERDESControl2Reg_t *serdesControl2; /**< @brief SERDES Control and Status #2 */
  hyplnkSERDESControl3Reg_t *serdesControl3; /**< @brief SERDES Control and Status #3 */
  hyplnkSERDESControl4Reg_t *serdesControl4; /**< @brief SERDES Control and Status #4 */
  hyplnkRXPrivIDTbl_t       *RXPrivIDTbl;    /**< @brief Entire RX PrivID table in one shot */
  hyplnkRXSegTbl_t          *RXSegTbl;       /**< @brief Entire RX Segment table in one shot */
  hyplnkIntCtrlTbl_t        *intCtrlTbl;     /**< @brief Entire Interrupt Control table in one shot */
  hyplnkIntPtrTbl_t         *intPtrTbl;      /**< @brief Entire Interrupt Pointer table in one shot */
} hyplnkRegisters_t;

/**
 * @ingroup hyplnklld_api_structures
 * @brief Specification of Hyplnk_Handle 
 *
 * The Hyplnk_Handle is used to identify a HYPLNK LLD instance
 */
typedef void *Hyplnk_Handle;

/**
 * @ingroup hyplnklld_api_structures
 * @brief Specification of Hyplnk_DeviceCfgBaseAddr 
 *
 * The Hyplnk_DeviceCfg is used to specify device level configuration
 * to the LLD.
 */
typedef struct
{
  void *cfgBase;
  void *dataBase;
} Hyplnk_DeviceCfgBaseAddr;

/**
 * @ingroup hyplnklld_api_structures
 * @brief Specification of Hyplnk_DeviceCfg 
 *
 * The Hyplnk_DeviceCfg is used to specify device level configuration
 * to the LLD.
 */
#define hyplnk_MAX_PERIPHS 4 /**< Maximum peripherals (base addresses) supported by LLD */
typedef struct
{
  Hyplnk_DeviceCfgBaseAddr bases[hyplnk_MAX_PERIPHS]; /**< base addreses */
} Hyplnk_DeviceCfg;

/**
 * @ingroup hyplnklld_api_structures
 * @brief Specification of Hyplnk_InitCfg 
 *
 * The Hyplnk_InitCfg is used to specify per-core
 * configuration to the LLD.  It is used with @ref Hyplnk_init ()
 * once per core.
 */
typedef struct
{
  Hyplnk_DeviceCfg dev; /**< Device Configuration */
} Hyplnk_InitCfg;

/**
 *  @ingroup hyplnklld_api_functions
 *  @brief  Hyplnk_init sets device configuration
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
 *  @retval  hyplnkRet_e status
 *
 *  @post    hyplnkLObj.device gets set to argument
 */
hyplnkRet_e Hyplnk_init
(
  Hyplnk_InitCfg *cfg /**< [in] configuration */
);

/**
 *  @ingroup hyplnklld_api_functions
 *  @brief  Hyplnk_open creates/opens a HyperLink instance
 *
 *  @details This function creates a handle.  The peripheral itself
 *           is not modified.  More than one handle to the same
 *           hyperlink peripheral can exist at the same time.
 *
 *  @pre     pHandle != NULL
 *  @pre     *pHandle == NULL
 *
 *  @retval  hyplnkRet_e status
 *
 *  @post    *pHandle == valid handle
 */
hyplnkRet_e Hyplnk_open 
(
  /** [in] HyperLink port number (0,1,...)
   * 
   * Current KeyStone devices only have one HyperLink port (port 0).  
   * The port number allows forwards compatibility if future devices
   * have multiple ports.
   */
  int            portNum,
  Hyplnk_Handle *pHandle   /**< [out] Resulting instance handle */
);

/**
 *  @ingroup hyplnklld_api_functions
 *  @brief Hyplnk_close Closes (frees) the driver handle.
 *
 *  @details The handle is released.  The peripheral itself is not
 *           modified.
 *
 *  @pre     pHandle != NULL
 *  @pre     *pHandle == valid handle
 *
 *  @retval  hyplnkRet_e status
 *
 *  @post    *pHandle == NULL
 */
hyplnkRet_e Hyplnk_close 
(
  Hyplnk_Handle *pHandle /**< [in] The HYPLNK LLD instance indentifier */
);

/**
 *  @ingroup hyplnklld_api_functions
 *  @brief Performs a configuration read
 *
 *  @details Reads one or more of the device registers
 *
 *  Each non-NULL register pointer in readRegs will be read and 
 *  broken into its fields.  
 *
 *  It is the users responsibility to ensure that no other tasks
 *  or cores will modify the registers while they are read,
 *  or betwen the time the registers are read and they are 
 *  later written back.
 *
 *  When the table-at-once fields are used
 *  (@ref hyplnkRegisters_s::RXPrivIDTbl
 *   @ref hyplnkRegisters_s::RXSegTbl
 *   @ref hyplnkRegisters_s::intCtrlTbl
 *   @ref hyplnkRegisters_s::intPtrTbl), then this API will
 *  generate <b>implicit register writes</b>, so it should not 
 *  be assumed that multiple simutaneous reads are safe.
 *
 *  Since the peripheral is shared across the device, and even
 *  between peripherals, it is not expected to be dynamically
 *  reprogramed (such as between thread or task switches).  It
 *  should only be reprogrammed at startup or when changing
 *  applications.  Therefore, there is a single-entry API instead
 *  of a set of inlines since it is not time-critical code.
 *
 *  @retval  hyplnkRet_e status
 */
hyplnkRet_e Hyplnk_readRegs 
(
  Hyplnk_Handle      handle,   /**< [in] The HYPLNK LLD instance identifier */
  hyplnkLocation_e   location, /**< [in] Local or remote peripheral */
  hyplnkRegisters_t *readRegs  /**< [in/out] List of registers to read */
);

/**
 *  @ingroup hyplnklld_api_functions
 *  @brief Performs a configuration write
 *
 *  @details Writes one or more of the device registers
 *
 *  It is the users responsibility to ensure that no other tasks
 *  or cores will modify the registers while they are read,
 *  or betwen the time the registers are read and they are 
 *  later written back.
 *
 *  The user will typically use @ref Hyplnk_readRegs to read the current 
 *  values in the registers, modify them in the local copies, then
 *  write back using @ref Hyplnk_writeRegs.
 *
 *  It is guaranteed that all registers can be written together.  The
 *  actual ordering will, for example, write index registers before
 *  the associated value registers 
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
 *  @retval  hyplnkRet_e status
 */
hyplnkRet_e Hyplnk_writeRegs 
(
  Hyplnk_Handle      handle,   /**< [in] The HYPLNK LLD instance identifier */
  hyplnkLocation_e   location, /**< [in] Local or remote peripheral */
  hyplnkRegisters_t *writeRegs /**< [in] List of registers to write */
);

/**
 *  @ingroup hyplnklld_api_functions
 *  @brief  Hyplnk_getWindow returns the address and size of the local memory window
 *
 *  @retval  hyplnkRet_e status
 */
hyplnkRet_e Hyplnk_getWindow 
(
  Hyplnk_Handle  handle,   /**< [in] The HYPLNK LLD instance identifier */
  void         **base,     /**< [out] The data base address */
  uint32_t      *size      /**< [out] Data window size [bytes] */
);

/**
 *  @ingroup hyplnklld_api_functions
 *  @brief  Hyplnk_getVersion returns the HYPLNK LLD version information
 *
 *  @details This function is used to get the version information of the HYPLNK LLD in 0xAABBCCDD format.
 *           where Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 *
 *  @retval                32-bit version information
 */
uint32_t Hyplnk_getVersion (void);


/**
 *  @ingroup hyplnklld_api_functions
 *  @brief  Hyplnk_getVersionStr returns the HYPLNK LLD version string
 *
 *  @details This function is used to get the version string of the HYPLNK LLD.
 *
 *  @retval                Version string
 */
const char* Hyplnk_getVersionStr (void);

#ifdef __cplusplus
}
#endif
  
#endif  /* _HYPLNK_H */

/* Nothing past this point */

