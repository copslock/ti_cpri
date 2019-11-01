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

#ifndef DP_SD0801_IF_H
#define DP_SD0801_IF_H

/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */

#include "cdn_errno.h"
#include "cdn_stdtypes.h"

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

/**********************************************************************
* Defines
**********************************************************************/
/** Number of voltage swing levels. */
#define DP_SD0801_SWING_LEVEL_COUNT 4U

/** Number of pre-emphasis levels. */
#define DP_SD0801_EMPHASIS_LEVEL_COUNT 4U

#define DP_SD0801_MAX_LANE_COUNT (4U)

/**
 *  @}
 */

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Forward declarations
**********************************************************************/
typedef struct DP_SD0801_VoltageCoefficients_s DP_SD0801_VoltageCoefficients;
typedef struct DP_SD0801_LinkState_s DP_SD0801_LinkState;
typedef struct DP_SD0801_Config_s DP_SD0801_Config;

typedef struct DP_SD0801_PrivateData_s DP_SD0801_PrivateData;

/**********************************************************************
* Enumerations
**********************************************************************/
typedef enum
{
    DP_SD0801_LINK_RATE_1_62 = 0x00U,
    DP_SD0801_LINK_RATE_2_16 = 0x01U,
    DP_SD0801_LINK_RATE_2_43 = 0x02U,
    DP_SD0801_LINK_RATE_2_70 = 0x03U,
    DP_SD0801_LINK_RATE_3_24 = 0x04U,
    DP_SD0801_LINK_RATE_4_32 = 0x05U,
    DP_SD0801_LINK_RATE_5_40 = 0x06U,
    DP_SD0801_LINK_RATE_8_10 = 0x07U
} DP_SD0801_LinkRate;

/** Specifies, how physical PHY lanes are mapped to ports of receiver(s) */
typedef enum
{
    /** Single controller is used. Lanes are not crossed (0->0, 1->1, 2->2, 3->3). */
    DP_SD0801_LANE_MAPPING_REGULAR_SINGLE = 0xE4U,
    /**
     * Up to two controllers are used with a PHY. Currently configured
     * controller uses only up to 2 lanes: lane 0 and lane 1. (0->0, 1->1)
     */
    DP_SD0801_LANE_MAPPING_LANES_01_DUAL = 0x04U,
    /**
     * Up to two controllers are used with a PHY. Currently configured
     * controller uses only up to 2 lanes: lane 2 and lane 3. (0->2, 1->3)
     */
    DP_SD0801_LANE_MAPPING_LANES_23_DUAL = 0x0EU
} DP_SD0801_LaneMapping;

/** Number of controllers per PHY. */
typedef enum
{
    /** Only one controller is used with PHY. */
    DP_SD0801_SINGLE_CONTROLLER = 0x01U,
    /** Two controllers are used with PHY. */
    DP_SD0801_DUAL_CONTROLLER = 0x02U
} DP_SD0801_ControllersPerPhy;

/**
 *  @}
 */

/** @defgroup DriverFunctionAPI Driver Function API
 *  Prototypes for the driver API functions. The user application can link statically to the
 *  necessary API functions and call them directly.
 *  @{
 */

/**********************************************************************
* API methods
**********************************************************************/

/**
 * Get the driver's memory requirements.
 * @param[in] config Driver/hardware configuration.
 * @param[out] memReq Size of memory, that needs to be allocated (in bytes).
 */
uint32_t DP_SD0801_Probe(const DP_SD0801_Config* config, uint32_t* memReq);

/**
 * Brief set up of driver, must be called before any other driver's
 * API function call.
 * @param[in,out] pD Driver state info specific to this instance.
 * @param[in] config Specifies driver/hardware configuration.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD, config or callbacks is NULL.
 * @return CDN_ENOTSUP If HW is not supported by this driver.
 */
uint32_t DP_SD0801_Init(DP_SD0801_PrivateData* pD, const DP_SD0801_Config* config);

/**
 * Initialize part of PHY responsible for AUX channel. Has to be
 * called before performing any DPCD or EDID operations.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SD0801_ConfigurePhyAuxCtrl(const DP_SD0801_PrivateData* pD);

/**
 * Automatically initialize and configure DP Main Link on PHY. Has to
 * be called before performing Link Training. This is a recommended
 * way to bring up PHY, instead of manual initialization. AUX channel
 * still has to be initialized separately.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] laneCount Number of lanes to initialize PHY with.
 * @param[in] linkRate Link rate to initialize PHY with.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL or parameters are invalid.
 */
uint32_t DP_SD0801_PhyStartUp(DP_SD0801_PrivateData* pD, uint8_t laneCount, DP_SD0801_LinkRate linkRate);

/**
 * Part of manual DP PHY Main Link initialization. Performs operations
 * to be done before releasing PHY reset.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] laneCount Number of lanes to initialize PHY with.
 * @param[in] linkRate Link rate to initialize PHY with.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL or parameters are invalid.
 */
uint32_t DP_SD0801_PhyInit(DP_SD0801_PrivateData* pD, uint8_t laneCount, DP_SD0801_LinkRate linkRate);

/**
 * Part of manual DP PHY Main Link initialization. Assert or release
 * PHY reset signal.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] reset 'true' - Assert PHY reset. 'false' - Release PHY reset.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SD0801_PhySetReset(const DP_SD0801_PrivateData* pD, bool reset);

/**
 * Part of manual DP PHY Main Link initialization. Wait, until PHY
 * gets ready after releasing PHY reset signal.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SD0801_WaitPmaCmnReady(const DP_SD0801_PrivateData* pD);

/**
 * Part of manual DP PHY Main Link initialization. Enable DP Main Link
 * lanes, after releasing PHY reset and waiting for PHY to get ready.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] laneCount Number of lanes to initialize PHY with.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SD0801_PhyRun(const DP_SD0801_PrivateData* pD, uint8_t laneCount);

/**
 * Configure voltage swing and pre-emphasis values for a specified
 * lane, according to values in linkState.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] lane Index of lane to configure.
 * @param[in] linkState Pointer to structure with link state, containing lane settings to use.
 *
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SD0801_ConfigLane(DP_SD0801_PrivateData* pD, uint8_t lane, const DP_SD0801_LinkState* linkState);

/**
 * Set link rate for main link, according to value in linkState. This
 * function also enables or disables SSC for main link, as SSC affects
 * the link rate slightly.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] linkState Pointer to structure with link state, containing link rate to use, and
 *    SSC enable flag.
 *
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SD0801_SetLinkRate(DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState);

/**
 * Set number of lanes to be used by main link, according to value in
 * linkState.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] linkState Pointer to structure with link state, containing lane count to use.
 *
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SD0801_EnableLanes(DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState);

/**
 * Get default values of voltage-related PHY registers, for given
 * voltage swing and pre-emphasis. Value of 0xFFFF means, that such
 * combination of swing and emphasis is not supported by PHY or is
 * forbidden, or that PHY does not contain particular register.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] voltageSwing Voltage swing level to use to select register values.
 * @param[in] preEmphasis Pre-emphasis level to use to select register values.
 * @param[out] coefficients Pointer to struct to be filled with default values of coefficients.
 * @return CDN_EOK success
 * @return CDN_EINVAL If voltage swing or pre-emphasis is outside of range (0-3).
 */
uint32_t DP_SD0801_GetDefaultCoeffs(const DP_SD0801_PrivateData* pD, uint8_t voltageSwing, uint8_t preEmphasis, DP_SD0801_VoltageCoefficients* coefficients);

/**
 * Get current values of voltage-related PHY registers, for given
 * voltage swing and pre-emphasis. After driver initialization values
 * will be the same, as default ones.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] voltageSwing Voltage swing level to use to select register values.
 * @param[in] preEmphasis Pre-emphasis level to use to select register values.
 * @param[out] coefficients Pointer to struct to be filled with current values of coefficients.
 * @return CDN_EOK success
 * @return CDN_EINVAL If voltage swing or pre-emphasis is outside of range (0-3).
 */
uint32_t DP_SD0801_GetCoefficients(const DP_SD0801_PrivateData* pD, uint8_t voltageSwing, uint8_t preEmphasis, DP_SD0801_VoltageCoefficients* coefficients);

/**
 * Set values of voltage-related PHY registers, to be used for given
 * voltage swing and pre-emphasis. Values for non-existent registers,
 * or for unsupported or forbidden swing/emphasis combinations will
 * have no effect.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] voltageSwing Voltage swing level to use to select register values.
 * @param[in] preEmphasis Pre-emphasis level to use to select register values.
 * @param[in] coefficients Pointer to struct containing values of coefficients to use.
 * @return CDN_EOK success
 * @return CDN_EINVAL If voltage swing or pre-emphasis is outside of range (0-3).
 */
uint32_t DP_SD0801_SetCoefficients(DP_SD0801_PrivateData* pD, uint8_t voltageSwing, uint8_t preEmphasis, const DP_SD0801_VoltageCoefficients* coefficients);

/**
 * Get current link status (link rate, lane count, voltage sing level
 * and pre-emphasis level). Values are valid after successful Link
 * Training.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] linkState Link State.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or linkState pointer is NULL.
 */
uint32_t DP_SD0801_ReadLinkStat(const DP_SD0801_PrivateData* pD, DP_SD0801_LinkState* linkState);

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */

#endif  /* DP_SD0801_IF_H */
