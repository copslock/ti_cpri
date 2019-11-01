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
#ifndef DP_SD0801_OBJ_IF_H
#define DP_SD0801_OBJ_IF_H

#include "dp_sd0801_if.h"

/** @defgroup DriverObject Driver API Object
 *  API listing for the driver. The API is contained in the object as
 *  function pointers in the object structure. As the actual functions
 *  resides in the Driver Object, the client software must first use the
 *  global GetInstance function to obtain the Driver Object Pointer.
 *  The actual APIs then can be invoked using obj->(api_name)() syntax.
 *  These functions are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* API methods
**********************************************************************/
typedef struct DP_SD0801_OBJ_s
{
    /**
     * Get the driver's memory requirements.
     * @param[in] config Driver/hardware configuration.
     * @param[out] memReq Size of memory, that needs to be allocated (in bytes).
     */
    uint32_t (*probe)(const DP_SD0801_Config* config, uint32_t* memReq);

    /**
     * Brief set up of driver, must be called before any other driver's
     * API function call.
     * @param[in,out] pD Driver state info specific to this instance.
     * @param[in] config Specifies driver/hardware configuration.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD, config or callbacks is NULL.
     * @return CDN_ENOTSUP If HW is not supported by this driver.
     */
    uint32_t (*init)(DP_SD0801_PrivateData* pD, const DP_SD0801_Config* config);

    /**
     * Initialize part of PHY responsible for AUX channel. Has to be
     * called before performing any DPCD or EDID operations.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*configurePhyAuxCtrl)(const DP_SD0801_PrivateData* pD);

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
    uint32_t (*phyStartUp)(DP_SD0801_PrivateData* pD, uint8_t laneCount, DP_SD0801_LinkRate linkRate);

    /**
     * Part of manual DP PHY Main Link initialization. Performs
     * operations to be done before releasing PHY reset.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] laneCount Number of lanes to initialize PHY with.
     * @param[in] linkRate Link rate to initialize PHY with.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL or parameters are invalid.
     */
    uint32_t (*PhyInit)(DP_SD0801_PrivateData* pD, uint8_t laneCount, DP_SD0801_LinkRate linkRate);

    /**
     * Part of manual DP PHY Main Link initialization. Assert or release
     * PHY reset signal.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] reset 'true' - Assert PHY reset. 'false' - Release PHY reset.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*phySetReset)(const DP_SD0801_PrivateData* pD, bool reset);

    /**
     * Part of manual DP PHY Main Link initialization. Wait, until PHY
     * gets ready after releasing PHY reset signal.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*waitPmaCmnReady)(const DP_SD0801_PrivateData* pD);

    /**
     * Part of manual DP PHY Main Link initialization. Enable DP Main
     * Link lanes, after releasing PHY reset and waiting for PHY to get
     * ready.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] laneCount Number of lanes to initialize PHY with.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*phyRun)(const DP_SD0801_PrivateData* pD, uint8_t laneCount);

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
    uint32_t (*configLane)(DP_SD0801_PrivateData* pD, uint8_t lane, const DP_SD0801_LinkState* linkState);

    /**
     * Set link rate for main link, according to value in linkState. This
     * function also enables or disables SSC for main link, as SSC
     * affects the link rate slightly.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] linkState Pointer to structure with link state, containing link rate to use, and
     *    SSC enable flag.
     *
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setLinkRate)(DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState);

    /**
     * Set number of lanes to be used by main link, according to value in
     * linkState.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] linkState Pointer to structure with link state, containing lane count to use.
     *
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*enableLanes)(DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState);

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
    uint32_t (*getDefaultCoeffs)(const DP_SD0801_PrivateData* pD, uint8_t voltageSwing, uint8_t preEmphasis, DP_SD0801_VoltageCoefficients* coefficients);

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
    uint32_t (*getCoefficients)(const DP_SD0801_PrivateData* pD, uint8_t voltageSwing, uint8_t preEmphasis, DP_SD0801_VoltageCoefficients* coefficients);

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
    uint32_t (*setCoefficients)(DP_SD0801_PrivateData* pD, uint8_t voltageSwing, uint8_t preEmphasis, const DP_SD0801_VoltageCoefficients* coefficients);

    /**
     * Get current link status (link rate, lane count, voltage sing level
     * and pre-emphasis level). Values are valid after successful Link
     * Training.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] linkState Link State.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or linkState pointer is NULL.
     */
    uint32_t (*readLinkStat)(const DP_SD0801_PrivateData* pD, DP_SD0801_LinkState* linkState);

} DP_SD0801_OBJ;

/**
 * In order to access the DP_SD0801 APIs, the upper layer software must call
 * this global function to obtain the pointer to the driver object.
 * @return DP_SD0801_OBJ* Driver Object Pointer
 */
extern DP_SD0801_OBJ *DP_SD0801_GetInstance(void);

/**
 *  @}
 */

#endif  /* DP_SD0801_OBJ_IF_H */
