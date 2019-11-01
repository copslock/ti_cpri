/* parasoft suppress item  MISRA2012-DIR-4_8 "Consider hiding implementation of structure" */
/**********************************************************************
* Copyright (C) 2012-2019 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 12.03.9e11b77(origin/DRV-3827_extract_sanity_to_c_file)
*          Do not edit it manually.
**********************************************************************
* Cadence Core Driver for the Cadence DP_SD0801 PHY for DisplayPort (DP)
* core. This header file lists the API providing a HAL (hardware
* abstraction layer) interface for the DP_SD0801 "Torrent" core.
**********************************************************************/
#ifndef DP_SD0801_STRUCTS_IF_H
#define DP_SD0801_STRUCTS_IF_H

#include "cdn_stdtypes.h"
#include "dp_sd0801_if.h"

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Structures and unions
**********************************************************************/
/**
 * Structure used to store values of PHY registers for voltage-related
 * coefficients, for particular voltage swing and re-emphasis level. Values
 * are shared across all physical lanes.
 */
struct DP_SD0801_VoltageCoefficients_s
{
    /** Value of DRV_DIAG_TX_DRV register to use */
    uint16_t DiagTxDrv;
    /** Value of TX_TXCC_MGNFS_MULT_000 register to use */
    uint16_t MgnfsMult;
    /** Value of TX_TXCC_CPOST_MULT_00 register to use */
    uint16_t CpostMult;
};

/** Structure containing parameters of physical link. */
struct DP_SD0801_LinkState_s
{
    /** Link Rate */
    DP_SD0801_LinkRate linkRate;
    /** Lane count */
    uint8_t laneCount;
    /** Voltage swing level, one per lane. */
    uint8_t voltageSwing[DP_SD0801_MAX_LANE_COUNT];
    /** Pre-emphasis level, one per lane. */
    uint8_t preEmphasis[DP_SD0801_MAX_LANE_COUNT];
    /** SSC (Spread-Spectrum Clock) enabled. */
    bool ssc;
};

/** Configuration parameters passed to probe & init functions. */
struct DP_SD0801_Config_s
{
    /** Base address of the PHY APB register space. */
    uint32_t* regBase;
    /** Base address of the DPTX controller's register space. */
    struct DP_Regs_s* regBaseDp;
};

/**
 *  @}
 */

#endif  /* DP_SD0801_STRUCTS_IF_H */
