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
* Cadence Core Driver for the Cadence DisplayPort (DP) core. This header
* file lists the API providing a HAL (hardware abstraction layer)
* interface for the DP core
**********************************************************************/
#ifndef DP_OBJ_IF_H
#define DP_OBJ_IF_H

#include "dp_if.h"

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
typedef struct DP_OBJ_s
{
    /**
     * Get the driver's memory requirements.
     * @param[in] config Proposed driver/hardware configuration.
     * @param[out] memReq Size of memory, that needs to be allocated (in bytes).
     */
    uint32_t (*probe)(const DP_Config* config, uint32_t* memReq);

    /**
     * brief set up API, must be called before any other API call
     * @param[in,out] pD Driver state info specific to this instance.
     * @param[in] config Specifies driver/hardware configuration.
     * @param[in] callbacks Client-supplied callback functions.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD, config or callbacks is NULL.
     * @return CDN_ENOTSUP If HW is not supported by this driver.
     */
    uint32_t (*init)(DP_PrivateData* pD, const DP_Config* config, const DP_Callbacks* callbacks);

    /**
     * Driver ISR. Platform-specific code is responsible for ensuring
     * this gets called when the corresponding hardware's interrupt is
     * asserted. Registering the ISR should be done after calling init,
     * and before calling start. The driver's ISR will not attempt to
     * lock any locks, but will perform client callbacks. If the client
     * wishes to defer processing to non-interrupt time, it is
     * responsible for doing so. This function must not be called after
     * calling destroy and releasing private data memory.
     * @param[in] pD Driver instance data.
     */
    void (*isr)(DP_PrivateData* pD);

    /**
     * Start the Display Port Core Driver by enabling interrupts. This is
     * called after the client has successfully initialized the driver
     * and hooked the driver's ISR (the isr member of this struct) to the
     * IRQ.
     * @param[in] pD Driver instance data.
     * @return CDN_EOK success
     * @return CDN_EINVAL pD is NULL
     */
    uint32_t (*start)(DP_PrivateData* pD);

    /**
     * Stops the Display Port Core Driver by disabling interrupts.
     * @param[in] pD Driver instance data.
     * @return CDN_EOK success
     * @return CDN_EINVAL pD is NULL
     */
    uint32_t (*stop)(DP_PrivateData* pD);

    /**
     * Destroy the driver.
     * @param[in] pD Driver instance data.
     * @return CDN_EOK success
     * @return CDN_EINVAL pD is NULL
     */
    uint32_t (*destroy)(const DP_PrivateData* pD);

    /**
     * Set address of PHY driver's private data structure.
     * @param[in] pD Driver instance data.
     * @param[out] phyPd Pointer to private data of PHY driver's instance to use.
     * @return CDN_EOK success
     * @return CDN_EINVAL pD or phyPd is NULL
     */
    uint32_t (*setPhyPd)(DP_PrivateData* pD, DP_SD0801_PrivateData* phyPd);

    /**
     * Releases uCPU reset and stalls it, then loads firmware from
     * provided image.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] image Pointer to structure with FW image information.
     * @return CDN_EOK success
     * @return CDN_EINVAL If IMEM size or DMEM size is not divisible by 4.
     */
    uint32_t (*loadFirmware)(const DP_PrivateData* pD, const DP_FirmwareImage* image);

    /**
     * Starts uCPU. Should be called after loading firmware.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*startUcpu)(const DP_PrivateData* pD);

    /**
     * Check, if there is a response awaiting in mailbox.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] awaits Pointer to store flag indicating presence of response.
     * @param[in] busType
     * @return CDN_EOK success
     * @return CDN_EINVAL If busType is outside enum range.
     */
    uint32_t (*checkResponse)(const DP_PrivateData* pD, bool* awaits, DP_BusType busType);

    /**
     * Debug echo command for APB.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] message Value to echo
     * @param[in] busType
     * @return CDN_EOK success
     * @return CDN_EIO reply message doesn't match request
     * @return CDN_EINVAL If busType is outside enum range.
     */
    uint32_t (*testEcho)(DP_PrivateData* pD, uint32_t message, DP_BusType busType);

    /**
     * Extended Echo test for mailbox. This test will send msg buffer to
     * firmware's mailbox and receive it back to the resp buffer.
     * Received data will be check against data sent and status will be
     * returned as well as received data.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] message Pointer to a buffer to send.
     * @param[out] response Pointer to buffer for receiving msg payload back.
     * @param[in] messageSize Number of bytes to send and receive.
     * @param[in] busType
     * @return CDN_EOK success
     * @return CDN_EIO reply message's size or content doesn't match request's.
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If number of bytes to use is invalid.
     */
    uint32_t (*testEchoExt)(DP_PrivateData* pD, const uint8_t* message, uint8_t* response, uint16_t messageSize, DP_BusType busType);

    /**
     * Get current FW version.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] ver fw version
     * @param[out] verlib lib version
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or fw/lib version pointer is NULL.
     */
    uint32_t (*getCurVersion)(const DP_PrivateData* pD, uint16_t* ver, uint16_t* verlib);

    /**
     * Read event registers (SW_EVENTS) value.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] events pointer to store 32-bit events value
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or events pointer is NULL.
     */
    uint32_t (*getEvent)(const DP_PrivateData* pD, uint32_t* events);

    /**
     * Read debug register value.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] debug pointer to store 16-bit debug reg value
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or debug pointer is NULL.
     */
    uint32_t (*getDebugRegVal)(const DP_PrivateData* pD, uint16_t* debug);

    /**
     * Check, if KEEP_ALIVE register has changed. To check if FW on uCPU
     * is still running, it is necessary to keep calling this function,
     * until 'updated' parameter is set to 'true' twice.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] updated Pointer to store flag, whether (true) or not (false) KEEP_ALIVE register
     *    changed since initialization or last call of this function, whichever
     *    happened later.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or updated pointer is NULL.
     */
    uint32_t (*checkAlive)(DP_PrivateData* pD, bool* updated);

    /**
     * Wait, until KEEP_ALIVE register changes. WARNING: If Firmware on
     * uCPU is not running, this function will block indefinitely.
     * DP_CheckAlive is a non-blocking equivalent.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*waitAlive)(DP_PrivateData* pD);

    /**
     * Sends request for setting uCPU/FW mode. DP_CheckResponse for
     * (regular) APB may be used to check, if reply is available.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mode 1 for active, 0 for standby
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendMainControlRequest)(DP_PrivateData* pD, uint8_t mode);

    /**
     * Gets response for setting uCPU/FW mode.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] resp pointer to store response.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or resp pointer is NULL.
     */
    uint32_t (*getMainControlResponse)(DP_PrivateData* pD, uint8_t* resp);

    /**
     * Set uCPU/FW to standby or active. Await for response.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mode 1 for active, 0 for standby
     * @param[out] resp Response: 1 for active, 0 for standby
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or resp pointer is NULL.
     */
    uint32_t (*mainControl)(DP_PrivateData* pD, uint8_t mode, uint8_t* resp);

    /**
     * Specify the Xtensa clock. This function shall be called before
     * turning on the CPU.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] ucpuClock Clock, that Xtensa uCPU is running at.
     * @return CDN_EOK success
     * @return CDN_EINVAL If clock is outside range (1.0 - 255.99 MHz).
     */
    uint32_t (*setClock)(DP_PrivateData* pD, const DP_UcpuClock* ucpuClock);

    /**
     * set maximum and minimum watchdog counters
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] watchdogMin Minimum value of the watchdog counter.
     * @param[in] watchdogMax Maximum value of the watchdog counter.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setWatchdogConfig)(DP_PrivateData* pD, uint32_t watchdogMin, uint32_t watchdogMax);

    /**
     * Inject memory error. Function is used for testing ECC mechanism.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] errorMask It is the bit flip mask for check bits and databits for both IRAM and DRAM.
     * @param[in] memType Select memory where shall be injected IRAM or DRAM.
     * @param[in] errorType Select whether error shall be injected either in data bits or in check bits.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*injectEccError)(DP_PrivateData* pD, uint32_t errorMask, DP_EccErrorMemType memType, DP_EccErrorType errorType);

    /**
     * Force fatal error. Fuction is used to test PFatalError output.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*forceFatalError)(DP_PrivateData* pD);

    /**
     * set audio video clock configuration
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId stream number which should be configured
     *    for SST it should be always 0
     *    for MST proper values are from to DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] audioVideoClkCfg audio video clock configuration to be set
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is has wrong value
     */
    uint32_t (*setAudioVideoClkCfg)(DP_PrivateData* pD, uint8_t streamId, const DP_AudioVideoClkCfg* audioVideoClkCfg);

    /**
     * set audio video clock configuration
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId stream number which should be configured
     *    for SST it should be always 0
     *    for MST proper values are from to DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[out] audioVideoClkCfg audio video clock configuration to be set
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is has wrong value
     */
    uint32_t (*getAudioVideoClkCfg)(DP_PrivateData* pD, uint8_t streamId, DP_AudioVideoClkCfg* audioVideoClkCfg);

    /**
     * set HDCP clock configuration
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] hdcpClockEnable HDCP clock config
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setHdcpClockConfig)(DP_PrivateData* pD, bool hdcpClockEnable);

    /**
     * get HDCP clock configuration
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] hdcpClockEnable HDCP clock config
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*getHdcpClockConfig)(DP_PrivateData* pD, bool* hdcpClockEnable);

    /**
     * Initialize part of PHY responsible for AUX channel. Has to be
     * called before performing any DPCD or EDID operations.
     * Alternatively, respective PHY driver's function may be called
     * instead.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*configurePhyAuxCtrl)(const DP_PrivateData* pD);

    /**
     * Automatically initialize and configure DP Main Link on PHY. Has to
     * be called before performing Link Training. This is a recommended
     * way to bring up PHY, instead of manual initialization. AUX channel
     * still has to be initialized separately. alternatively, respective
     * PHY driver's function may be called instead.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] laneCount Number of lanes to initialize PHY with.
     * @param[in] linkRate Link rate to initialize PHY with.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL or parameters are invalid.
     */
    uint32_t (*configurePhyStartUp)(DP_PrivateData* pD, uint8_t laneCount, DP_LinkRate linkRate);

    /**
     * Sends request for reading EDID from sink device. DP_checkResponse
     * for (regular) APB may be used to check, if reply is available.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] segment EDID segment to read.
     * @param[in] extension EDID extension to read
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendEdidReadRequest)(DP_PrivateData* pD, uint8_t segment, uint8_t extension);

    /**
     * Gets response with EDID read from sink device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] resp Pointer structure to be filled with response.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Size of EDID read was not 128 bytes.
     * @return CDN_EINVAL If pD or resp pointer is NULL.
     */
    uint32_t (*getEdidReadResponse)(DP_PrivateData* pD, DP_ReadEdidResponse* resp);

    /**
     * Reads EDID from sink device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] segment EDID segment to read.
     * @param[in] extension EDID extension to read
     * @param[in] resp Pointer structure to be filled with response.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Size of EDID read was not 128 bytes.
     * @return CDN_EINVAL If pD or resp pointer is NULL.
     */
    uint32_t (*readEdid)(DP_PrivateData* pD, uint8_t segment, uint8_t extension, DP_ReadEdidResponse* resp);

    /**
     * Set power mode of sink device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mode Power mode
     * @return CDN_EOK success
     * @return CDN_EINVAL If PwrMode value is outside respective enum.
     */
    uint32_t (*setPowerMode)(DP_PrivateData* pD, DP_PwrMode mode);

    /**
     * Sets source capabilities, according to filled
     * DP_SourceDeviceCapabilities structure, provided by user.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] caps Pointer to structure with source capabilities.
     * @return CDN_EOK success
     * @return CDN_EINVAL If there are invalid values in capabilities (outside range or enum).
     */
    uint32_t (*setSourceCapabilities)(DP_PrivateData* pD, const DP_SourceDeviceCapabilities* caps);

    /**
     * Perform a series of DPCD register reads, used to determine
     * capabilities of sink device, and get structure with them.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] caps Pointer to structure to be filled with sink capabilities.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or caps is NULL.
     */
    uint32_t (*getSinkCapabilities)(DP_PrivateData* pD, DP_SinkDeviceCapabilities* caps);

    /**
     * Set custom 80-bit pattern to be transmitted for testing. To
     * transmit that pattern, DP_setTestPattern needs to be called, with
     * PATTERN_80_BIT as "pattern" argument.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] customPattern Array of 10 bytes (80 bits) to be transmitted as the pattern. Bytes are
     *    to be transmitted in order they're placed in the table ([0] -> [9]).
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or customPattern is NULL.
     */
    uint32_t (*setCustomPattern)(DP_PrivateData* pD, uint8_t customPattern[10]);

    /**
     * Transmit (or stop transmitting) selected test pattern at requested
     * main link parameters (link rate, lane count, voltage swing, pre-
     * emphasis). Does not inform sink about transmitted pattern (DPCD
     * registers 10Bh - 10Eh).
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] pattern Test pattern to be transmitted. Selecting PATTERN_DISABLE will stop
     *    transmitting test pattern.
     * @param[in] linkParams Pointer to structure containing main link parameters to be used. If
     *    pattern is PATTERN_DISABLE, main link parameters will not be changed.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pattern or any member of linkParams is out of range.
     */
    uint32_t (*setTestPattern)(DP_PrivateData* pD, DP_TestPattern pattern, DP_LinkState* linkParams);

    /**
     * Sends request for reading DPCD register(s) from sink device.
     * DP_checkResponse for (regular) APB may be used to check, if reply
     * is available.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] request Parameters related to DPCD read operation.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or request pointer is NULL.
     */
    uint32_t (*sendDpcdReadRequest)(DP_PrivateData* pD, const DP_DpcdTransfer* request);

    /**
     * Gets response with DPCD register(s) read from sink device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in,out] transfer Pointer with request structure, to be filled with results from the
     *    response. If, as result of an error, more bytes were read, than were
     *    requested, only as many bytes as were requested will be copied to the
     *    buffer, to avoid overflow.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Address or count of DPCD registers, that were read, does not match request
     * @return CDN_EINVAL If pD or transfer pointer is NULL.
     */
    uint32_t (*getDpcdReadResponse)(DP_PrivateData* pD, DP_DpcdTransfer* transfer);

    /**
     * Reads DPCD register(s) from sink device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in,out] transfer Pointer with request structure, to be filled with results from the
     *    response. If, as result of an error, more bytes were read, than were
     *    requested, only as many bytes as were requested will be copied to the
     *    buffer, to avoid overflow.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Address or count of DPCD registers, that were read, does not match request
     * @return CDN_EINVAL If pD or transfer pointer is NULL.
     */
    uint32_t (*readDpcd)(DP_PrivateData* pD, DP_DpcdTransfer* transfer);

    /**
     * Sends request for writing DPCD register(s) to sink device.
     * DP_checkResponse for (regular) APB may be used to check, if reply
     * is available.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] request Parameters related to DPCD write operation.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or request pointer is NULL.
     */
    uint32_t (*SendDpcdWriteRequest)(DP_PrivateData* pD, const DP_DpcdTransfer* request);

    /**
     * Gets response with DPCD register(s) write operation status.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in,out] transfer Pointer with request structure, to be filled with results from the
     *    response.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Address or count of DPCD registers, that were written, does not match request
     * @return CDN_EINVAL If pD or transfer pointer is NULL.
     */
    uint32_t (*GetDpcdWriteResponse)(DP_PrivateData* pD, DP_DpcdTransfer* transfer);

    /**
     * Writes DPCD register(s) to sink device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in,out] transfer Pointer with request structure, to be filled with results from
     *    response.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Address or count of DPCD registers, that were written, does not match request
     * @return CDN_EINVAL If pD or transfer pointer is NULL.
     */
    uint32_t (*writeDpcd)(DP_PrivateData* pD, DP_DpcdTransfer* transfer);

    /**
     * Sends request for I2C-over-AUX read from given I2C address, using
     * sink device. DP_checkResponse for (regular) APB may be used to
     * check, if reply is available.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] request Parameters related to I2C-over-AUX read operation.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or request pointer is NULL.
     */
    uint32_t (*sendI2cReadRequest)(DP_PrivateData* pD, const DP_I2cTransfer* request);

    /**
     * Gets response with bytes read over I2C-over-AUX, using sink
     * device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in,out] transfer Pointer with request structure, to be filled with results from the
     *    response. If, as result of an error, more bytes were read, than were
     *    requested, only as many bytes as were requested will be copied to the
     *    buffer, to avoid overflow.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Address or I2C slave or number of bytes, that were read, does not match request
     * @return CDN_EINVAL If pD or transfer pointer is NULL.
     */
    uint32_t (*getI2cReadResponse)(DP_PrivateData* pD, DP_I2cTransfer* transfer);

    /**
     * Reads bytes over I2C-over-AUX, using sink device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in,out] transfer Pointer with request structure, to be filled with results from the
     *    response. If, as result of an error, more bytes were read, than were
     *    requested, only as many bytes as were requested will be copied to the
     *    buffer, to avoid overflow.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Address or I2C slave or number of bytes, that were read, does not match request
     * @return CDN_EINVAL If pD or transfer pointer is NULL.
     */
    uint32_t (*i2cRead)(DP_PrivateData* pD, DP_I2cTransfer* transfer);

    /**
     * Sends request for I2C-over-AUX write to given I2C address, using
     * sink device. DP_checkResponse for (regular) APB may be used to
     * check, if reply is available.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] request Parameters related to I2C-over-AUX write operation.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or request pointer is NULL.
     */
    uint32_t (*SendI2cWriteRequest)(DP_PrivateData* pD, const DP_I2cTransfer* request);

    /**
     * Gets response with I2C-over-AUX write operation status.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in,out] transfer Pointer with request structure, to be filled with results from the
     *    response.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Address or I2C slave or number of bytes, that were written, does not match request
     * @return CDN_EINVAL If pD or transfer pointer is NULL.
     */
    uint32_t (*GetI2cWriteResponse)(DP_PrivateData* pD, DP_I2cTransfer* transfer);

    /**
     * Writes bytes over I2C-over-AUX, using sink device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in,out] transfer Pointer with request structure, to be filled with results from
     *    response.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO Address or I2C slave or number of bytes, that were written, does not match request
     * @return CDN_EINVAL If pD or transfer pointer is NULL.
     */
    uint32_t (*i2cWrite)(DP_PrivateData* pD, DP_I2cTransfer* transfer);

    /**
     * Enables or disables ASSR (Alternate Scrambler Seed Reset) in sink
     * and source devices, if it's supported by sink.
     * DP_getSinkCapabilities function may also be used to check, if sink
     * supports that feature. Standard value of 0xFFFE is used as an
     * alternate seed.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] enable Whether to enable ('true') or disable ('false') ASSR.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EIO DPCD transaction or register write was unsuccessful.
     * @return CDN_ENOTSUP Enabling ASSR was requested, but sink device does not support it.
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setAssrEnable)(DP_PrivateData* pD, bool enable);

    /**
     * Enables or disables shortening of AUX transaction preamble (sync
     * pattern) from 16 to 8 pulses (as required by eDP standard).
     * Default (false) is 16 pulses, as required by regular DisplayPort
     * standard. It is required, by the eDP standard, to enable shortened
     * preamble.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] enable Whether to shorten ('true') or not ('false') the AUX preamble.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setShortenedAuxPreamble)(DP_PrivateData* pD, bool enable);

    /**
     * Performs DisplayPort Link Training, according to capabilities set
     * via setHostCapabilities and sink capabilities read from DPCD.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] resultLt Result of Link Training process, according to TrainingStatus enum.
     * @return CDN_EOK success
     * @return CDN_ENOENT Source device capabilities were not set.
     * @return CDN_ECANCELED Sink device capabilities are not correct.
     * @return CDN_EINVAL If pD or result pointer is NULL.
     */
    uint32_t (*linkTraining)(DP_PrivateData* pD, DP_TrainingStatus* resultLt);

    /**
     * Checks, if link is stable and synchronized, according to sink
     * device.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] resultLs Whether ('true') or not link is stable and synchronized.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or result pointer is NULL.
     */
    uint32_t (*checkLinkStable)(DP_PrivateData* pD, bool* resultLs);

    /**
     * Sets mask disabling events.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mask Event mask (bit '1' at given position disables event).
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setEventMask)(DP_PrivateData* pD, uint32_t mask);

    /**
     * Gets mask disabling events.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] mask Event mask (bit '1' at given position disables event).
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or mask pointer is NULL.
     * @return CDN_ENOTSUP Operation currently not implemented
     */
    uint32_t (*getEventMask)(const DP_PrivateData* pD, const uint32_t* mask);

    /**
     * Sends request for reading events related to HPD from uCPU.
     * DP_CheckResponse for (regular) APB may be used to check, if reply
     * is available.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendReadHpdEventRequest)(DP_PrivateData* pD);

    /**
     * Gets response for reading events related to HPD from uCPU.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] hpdEvents pointer to store response.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or hpdEvents pointer is NULL.
     */
    uint32_t (*getReadHpdEventResponse)(DP_PrivateData* pD, uint8_t* hpdEvents);

    /**
     * Reads events.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] hpdEvents Set of HPD-related events, as bits defined in DP_HpdEvents enum.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or hpdEvents pointer is NULL.
     */
    uint32_t (*readHpdEvent)(DP_PrivateData* pD, uint8_t* hpdEvents);

    /**
     * Fills DP_VideoFormatParams structure, based on entry in VIC
     * parameters table, as defined in VicModes enum.
     * @param[out] vicParams Structure with VIC-relared parameters to fill.
     * @param[in] vicMode VIC mode to take from table and fill into structure.
     * @return CDN_EOK success
     * @return CDN_EINVAL If vicMode is outside enum range.
     */
    uint32_t (*fillVideoFormat)(DP_VideoFormatParams* vicParams, DP_VicModes vicMode);

    /**
     * Set vic mode according to vic table, or requested video
     * parameters. May be called after successful Link Training. If
     * different synchronization pulse is desired for MSA data, than one
     * present on the Video Interface (VIF), DP_SetMsaSyncPolarity may be
     * called after this function.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1 DSC can be enabled only for stream 0 or 1.
     * @param[in] parameters Structure with video parameters to set.
     * @return CDN_EOK success
     * @return CDN_ENOENT PHY was not initialized and Link Training was not done yet.
     * @return CDN_EINVAL If incorrect video parameters were detected.
     */
    uint32_t (*setVic)(DP_PrivateData* pD, uint8_t streamId, const DP_VideoParameters* parameters);

    /**
     * DP_SetVic function sets the same synchronization pulse polarity
     * for VIF (Video Interface) input and MSA (Main Steam Attribute). If
     * it is desired to use different polarity in MSA data, this function
     * may be called after DP_SetVic to override sync polarity for MSA
     * (leaving polarity for VIF as DP_SetVic had set it).
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] hSyncPolarity Horizontal sync pulse polarity to set in MSA.
     * @param[in] vSyncPolarity Vertical sync pulse polarity to set in MSA.
     * @return CDN_EOK success
     * @return CDN_EINVAL If there are invalid values in parameters (outside of enum).
     */
    uint32_t (*setMsaSyncPolarity)(DP_PrivateData* pD, uint8_t streamId, DP_SyncPolarity hSyncPolarity, DP_SyncPolarity vSyncPolarity);

    /**
     * Turn framer on or off. Enabling framer on is required before
     * enabling video. Disabling framer is required before performing
     * Link Training.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] enable Framer enable (true - on, false - off)
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setFramerEnable)(DP_PrivateData* pD, bool enable);

    /**
     * Turn video on or off. Applicable to SST mode only. Disabling video
     * is required before performing Link Training.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mode Video Mode (true - on, false - off)
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setVideoSst)(DP_PrivateData* pD, bool mode);

    /**
     * Get current link status (link rate, lane count, voltage sing level
     * and pre-emphasis level). Values are valid after successful Link
     * Training.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] linkState Link State.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or linkState pointer is NULL.
     */
    uint32_t (*readLinkStat)(DP_PrivateData* pD, DP_LinkState* linkState);

    /**
     * Sends request for reading status of last AUX transaction.
     * DP_CheckResponse for (regular) APB may be used to check, if reply
     * is available.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendAuxStatusRequest)(DP_PrivateData* pD);

    /**
     * Gets response with status of last AUX transaction.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] status Latest AUX status
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or status pointer is NULL.
     */
    uint32_t (*getAuxStatusResponse)(DP_PrivateData* pD, DP_AuxStatus* status);

    /**
     * Reads and returns status of most recent AUX transaction.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] status Latest AUX status
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or status pointer is NULL.
     */
    uint32_t (*getAuxStatus)(DP_PrivateData* pD, DP_AuxStatus* status);

    /**
     * Sends request for reading status of last I2C-over-AUX transaction.
     * DP_CheckResponse for (regular) APB may be used to check, if reply
     * is available.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendI2cStatusRequest)(DP_PrivateData* pD);

    /**
     * Gets response with status of last I2C-over-AUX transaction.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] status Latest I2C-over-AUX status
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or status pointer is NULL.
     */
    uint32_t (*getI2cStatusResponse)(DP_PrivateData* pD, DP_I2cStatus* status);

    /**
     * Reads and returns status of most recent I2C-over-AUX transaction.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] status Latest I2C-over-AUX status
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or status pointer is NULL.
     */
    uint32_t (*getI2cStatus)(DP_PrivateData* pD, DP_I2cStatus* status);

    /**
     * Sends request for reading HPD status. DP_CheckResponse for
     * (regular) APB may be used to check, if reply is available.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD pointer is NULL.
     */
    uint32_t (*sendHpdStatusRequest)(DP_PrivateData* pD);

    /**
     * Gets response with HPD status.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] status Whether or not HPD is connected and stable.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or status pointer is NULL.
     */
    uint32_t (*getHpdStatusResponse)(DP_PrivateData* pD, bool* status);

    /**
     * Reads and returns status of HPD.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] status Whether or not HPD is connected and stable.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or status pointer is NULL.
     */
    uint32_t (*getHpdStatus)(DP_PrivateData* pD, bool* status);

    /**
     * Enables/Disables FEC on DP TX Controller
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] fecEnable informs if FEC should be enabled (true), or disabled (false).
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setFecEnable)(DP_PrivateData* pD, bool fecEnable);

    /**
     * Sets/clears FEC_READY bit in DPCD on sink device and in source
     * registers.     Function must be called before link training.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] enable informs if FEC_READY should be set (true), or cleared (false).
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setFecReady)(DP_PrivateData* pD, bool enable);

    /**
     * Sets provided SDP under corresponding entry_id into controller.
     * SDP currently residing under entry_id gets invalidated first.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] entryID 4 Most significant bits of the packet memory address to write.
     * @param[in] packetData Data of SDP entry to write
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is outside range.
     */
    uint32_t (*setSdp)(DP_PrivateData* pD, uint8_t streamId, uint8_t entryID, const DP_SdpEntry* packetData);

    /**
     * Invalidates SDP under corresponding entry_id in controller.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] entryID 4 Most significant bits of the packet memory containing SDP to remove
     * @return CDN_EOK success
     * @return CDN_EINVAL If entryId is outside range.
     */
    uint32_t (*removeSdp)(DP_PrivateData* pD, uint8_t streamId, uint8_t entryID);

    /**
     * Set configuration of HDCP transmitter
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] config Pointer to structure with configuration for HDCP transmitter.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or config pointer is NULL.
     */
    uint32_t (*configureHdcpTx)(DP_PrivateData* pD, const DP_HdcpTxConfiguration* config);

    /**
     * Set public key for HDCP 2.x transmitter
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] key Pointer to structure with HDCP 2.x public key.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or key pointer is NULL.
     */
    uint32_t (*setHdcp2TxPublicKey)(DP_PrivateData* pD, const DP_Hdcp2TxPublicKey* key);

    /**
     * Set km-key used to decrypt other HDCP keys.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] key Pointer to structure with km-key.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or key pointer is NULL.
     */
    uint32_t (*setHdcpKmEncCustomKey)(DP_PrivateData* pD, const DP_HdcpTxKmEncCustomKey* key);

    /**
     * Set HDCP 2.x random numbers for debug purposes.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] numbers Pointer to structure with random numbers.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or numbers pointer is NULL.
     */
    uint32_t (*setHdcp2DebugRandom)(DP_PrivateData* pD, const DP_HdcpDebugRandomNumbers* numbers);

    /**
     * Send a response to HDCP 2.x engine, that pairing data (including
     * km) associated with Receiver ID is currently not stored.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*hdcp2RespondKmNotStored)(DP_PrivateData* pD);

    /**
     * Send a response to HDCP 2.x engine containing stored pairing data
     * (including km) associated with Receiver ID.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] pairingData Pointer to filled structure containing HDCP 2.x pairing data.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or pairingData pointer is NULL.
     */
    uint32_t (*hdcp2RespondKmStored)(DP_PrivateData* pD, const DP_HdcpPairingData* pairingData);

    /**
     * Set private keys for HDCP 1.x transmitter.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] keySet Pointer to filled structure containing HDCP 1.x Device Key Set.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or keySet pointer is NULL.
     */
    uint32_t (*setHdcp1TxKeys)(DP_PrivateData* pD, const DP_Hdcp1Keys* keySet);

    /**
     * Set 'An' random number (for debug only), used in HDCP 1.x
     * authentication.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] an 8-byte array containing 'An' number.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or address of 'an' array is NULL.
     */
    uint32_t (*setHdcp1RandomAn)(DP_PrivateData* pD, const uint8_t an[8]);

    /**
     * Send request for status of HDCP transmitter. DP_CheckResponse for
     * (secure) SAPB may be used to check, if reply is available.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendHdcpTxStatusRequest)(DP_PrivateData* pD);

    /**
     * Get response with status of HDCP transmitter.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] status Pointer to structure with HDCP TX status, to be filled.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or status pointer is NULL.
     */
    uint32_t (*getHdcpTxStatusResponse)(DP_PrivateData* pD, DP_HdcpTxStatus* status);

    /**
     * Get status of HDCP transmitter. Should be called in response to
     * HDCP status event.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] status Pointer to structure with HDCP TX status, to be filled.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or status pointer is NULL.
     */
    uint32_t (*getHdcpTxStatus)(DP_PrivateData* pD, DP_HdcpTxStatus* status);

    /**
     * Send request for Receiver ID, which should looked up in storage,
     * to get pairing data (if exists in storage). DP_CheckResponse for
     * (secure) SAPB may be used to check, if reply is available.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendHdcp2RecvIdRequest)(DP_PrivateData* pD);

    /**
     * Get response with Receiver ID, which should looked up in storage.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] id 5-byte array to be filled with Receiver ID.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or address of 'id' array is NULL.
     */
    uint32_t (*getHdcp2RecvIdResponse)(DP_PrivateData* pD, uint8_t id[5]);

    /**
     * Get Receiver ID, which should looked up in storage, to get pairing
     * data (if exists in storage).
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] id 5-byte array to be filled with Receiver ID.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or address of 'id' array is NULL.
     */
    uint32_t (*getHdcp2RecvId)(DP_PrivateData* pD, uint8_t id[5]);

    /**
     * Send request for pairing data (receiver ID and associated
     * cryptographic keys and values) of receiver being currently
     * authenticated. DP_CheckResponse for (secure) SAPB may be used to
     * check, if reply is available.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendHdcp2PairingDataRequest)(DP_PrivateData* pD);

    /**
     * Get response with pairing data of receiver being currently
     * authenticated.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] pairingData Pointer to structure with HDCP pairing data to be filled.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or pairingData pointer is NULL.
     */
    uint32_t (*getHdcp2PairingDataResponse)(DP_PrivateData* pD, DP_HdcpPairingData* pairingData);

    /**
     * Get pairing data (receiver ID and associated cryptographic keys
     * and values) of receiver being currently authenticated.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] pairingData Pointer to structure with HDCP pairing data to be filled.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or pairingData pointer is NULL.
     */
    uint32_t (*getHdcp2PairingData)(DP_PrivateData* pD, DP_HdcpPairingData* pairingData);

    /**
     * Send request for list of all receiver IDs connected directly or
     * via repeaters, for checking them in revocation list.
     * DP_CheckResponse for (secure) SAPB may be used to check, if reply
     * is available.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*sendHdcpRecvIdListRequest)(DP_PrivateData* pD);

    /**
     * Get response with list of all receiver IDs connected directly or
     * via repeaters.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] list Pointer to structure with list of all receiver IDs, and their count,
     *    to be filled.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or list pointer is NULL.
     */
    uint32_t (*getHdcpRecvIdListResponse)(DP_PrivateData* pD, DP_HdcpRecvIdList* list);

    /**
     * Get list of all receiver IDs connected directly or via repeaters,
     * for checking them in revocation list.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] list Pointer to structure with list of all receiver IDs, and their count,
     *    to be filled.
     * @return CDN_EOK success
     * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
     * @return CDN_EINVAL If pD or list pointer is NULL.
     */
    uint32_t (*getHdcpRecvIdList)(DP_PrivateData* pD, DP_HdcpRecvIdList* list);

    /**
     * Inform HDCP engine, if all receiver IDs are valid (not present on
     * the revocation list)
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] valid 'true'  - no receiver ID was found on revocation list.
     *    'false' - at least one receiver was found on revocation list.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD is NULL.
     */
    uint32_t (*setHdcpRecvValid)(DP_PrivateData* pD, bool valid);

    /**
     * Set 128-bit Global Constant lc.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] lc128 16-byte array with LC128 value to use. If km-key encryption is enabled,
     *    it is expected to be provided in encrypted form.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or address of lc128 array is NULL.
     */
    uint32_t (*setHdcp2Lc)(DP_PrivateData* pD, const uint8_t lc128[16]);

    /**
     * Set random seed from external TRNG.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] seed 32-byte array with random seed. Number is expected to come from TRNG
     *    and have sufficiently high entropy level.
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or address of seed array is NULL.
     */
    uint32_t (*setHdcpSeed)(DP_PrivateData* pD, const uint8_t seed[32]);

    /**
     * Mute or unmute audio.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] muteMode Select mute/unmute.
     * @return CDN_EOK success
     * @return CDN_EINVAL If muteMode is outside of enum range.
     */
    uint32_t (*audioSetMute)(DP_PrivateData* pD, uint8_t streamId, DP_AudioMuteMode muteMode);

    /**
     * Start playing audio with the given parameters.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] params Parameters to configure audio with.
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is outside range.
     */
    uint32_t (*audioAutoConfig)(DP_PrivateData* pD, uint8_t streamId, const DP_AudioParams* params);

    /**
     * Stop current audio, to allow setting up new one.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is outside range.
     */
    uint32_t (*audioStop)(DP_PrivateData* pD, uint8_t streamId);

    /**
     * Set audio on or off in internal registers
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] mode Audio on/off mode.
     * @return CDN_EOK success
     * @return CDN_EINVAL If mode is outside of enum range.
     */
    uint32_t (*audioSetMode)(DP_PrivateData* pD, uint8_t streamId, DP_AudioMode mode);

    /**
     * Set DSC configuration.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is 1.
     * @param[in] dscConfig Parameters to configure DSC module
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is outside range.
     */
    uint32_t (*setDscConfig)(DP_PrivateData* pD, uint8_t streamId, const DP_DscConfig* dscConfig);

    /**
     * Get DSC configuration.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is 1.
     * @param[out] dscConfig Parameters current DSC configuration
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is outside range.
     */
    uint32_t (*getDscConfig)(DP_PrivateData* pD, uint8_t streamId, DP_DscConfig* dscConfig);

    /**
     * Send PPS packet.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is 1.
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is outside range.
     */
    uint32_t (*dscSendPps)(DP_PrivateData* pD, uint8_t streamId);

    /**
     * Function set CompressedStream_Flag in VB_ID register.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] val Whether or not to set CompressedStreamFlag.
     * @return CDN_EOK success
     * @return CDN_EINVAL If streamId is outside range.
     */
    uint32_t (*setCompressedStreamFlag)(DP_PrivateData* pD, uint8_t streamId, bool val);

    /**
     * Function resets DSC module.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL if pD is NULL
     */
    uint32_t (*dscReset)(DP_PrivateData* pD);

    /**
     * Function enables MST in DP controller and in directly connected
     * device.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL if pD is NULL
     */
    uint32_t (*MstEnable)(DP_PrivateData* pD);

    /**
     * Function disables MST in DP controller and in directly connected
     * device
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL if pD is NULL
     */
    uint32_t (*MstDisable)(DP_PrivateData* pD);

    /**
     * Function enables stream for MST mode in DP controller.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @return CDN_EOK success
     * @return CDN_EINVAL if streamID is bigger than DP_MAX_NUMBER_OF_STREAMS - 1
     * @return CDN_ENOTSUP if MST feature is not enabled
     */
    uint32_t (*MstStreamEnable)(DP_PrivateData* pD, uint8_t streamId);

    /**
     * Function disables stream for MST mode in DP controller.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @return CDN_EOK success
     * @return CDN_EINVAL if streamID is bigger than DP_MAX_NUMBER_OF_STREAMS - 1
     * @return CDN_ENOTSUP if MST feature is not enabled
     */
    uint32_t (*MstStreamDisable)(DP_PrivateData* pD, uint8_t streamId);

    /**
     * Function calculate payload base on current stream configuration
     * Next configures controller and sends payload allocate request.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] sinkDevice Sink device associated with stream
     * @return CDN_EOK success
     * @return CDN_EINVAL if streamID is bigger than DP_MAX_NUMBER_OF_STREAMS - 1
     */
    uint32_t (*MstAllocateVcpi)(DP_PrivateData* pD, uint8_t streamId, DP_SinkDevice* sinkDevice);

    /**
     * Function clear payload for given stream and sink device Next
     * configures controller and sends payload allocate request.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] sinkDevice Sink device associated with stream
     * @return CDN_EOK success
     * @return CDN_EINVAL if streamID is bigger than DP_MAX_NUMBER_OF_STREAMS - 1
     */
    uint32_t (*MstDeallocateVcpi)(DP_PrivateData* pD, uint8_t streamId, DP_SinkDevice* sinkDevice);

    /**
     * Get number of connected sink devices for MST mode
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] sinkCount Number of connected sinks
     * @return CDN_EOK success
     * @return CDN_EINVAL if cannot count all sink devices
     */
    uint32_t (*MstGetSinkCount)(DP_PrivateData* pD, uint8_t* sinkCount);

    /**
     * Returns pointers to sink devices according to number returned by
     * DP_MstGetSinkCount.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] sinkList Array for pointers to sink devices
     * @return CDN_EOK success
     * @return CDN_EINVAL If pD or sinkList pointer is NULL.
     */
    uint32_t (*MstGetSinkList)(const DP_PrivateData* pD, const DP_SinkDevice** sinkList);

    /**
     * Function used to enable encryption in MST mode. Function should be
     * called after authentication     succeed. Before any payload is
     * allocated.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] enable 'true'  - enable encryption
     *    'false' - disable encryption
     * @return CDN_EOK success
     * @return CDN_EINVAL if pD is NULL
     */
    uint32_t (*MstSetEncryptionEnable)(DP_PrivateData* pD, bool enable);

    /**
     * Function used to configure encryption for given stream. It enables
     * or disables encryption for all slots used by given stream.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] streamId ID of stream which encryption is configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
     * @param[in] enable 'true'  - enable encryption
     *    'false' - disable encryption
     * @return CDN_EOK success
     * @return CDN_EINVAL If no payload allocated for given streamId
     */
    uint32_t (*MstSetEncryption)(DP_PrivateData* pD, uint8_t streamId, bool enable);

    /**
     * Scan current topology. User can scan topology at initial state.
     * Scanning is done automatically by DP_MstHpdIrq function     always
     * when something is changed in topology.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL if pD is NULL
     * @return CDN_ENOTSUP if MST is disabled
     */
    uint32_t (*MstScanTopology)(DP_PrivateData* pD);

    /**
     * Reads EDID from remote sink device using sideband channel.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] sinkDevice sink device from which EDID shall be read
     * @param[in] block EDID block to read
     * @param[in] edidResponse Pointer structure to be filled with response
     * @return CDN_EOK success
     * @return CDN_EINVAL if any of input parameter has wrong value or is a NULL pointer
     * @return CDN_EIO if MST is not configured correctly or if an transmission error occurs
     */
    uint32_t (*MstReadRemoteEdid)(const DP_PrivateData* pD, DP_SinkDevice* sinkDevice, uint32_t block, DP_ReadEdidResponse* edidResponse);

    /**
     * Handle HPD IRQ. It handles incoming sideband messages.     It
     * should be called if MST is enabled and HPD pulse event occurs.
     * This function must not be called inside interrupt handling
     * routine.
     * @param[in] pD Driver state info specific to this instance.
     * @return CDN_EOK success
     * @return CDN_EINVAL if pD is NULL
     * @return CDN_EIO if MST is not configured correctly
     */
    uint32_t (*MstHpdIrq)(DP_PrivateData* pD);

} DP_OBJ;

/**
 * In order to access the DP APIs, the upper layer software must call
 * this global function to obtain the pointer to the driver object.
 * @return DP_OBJ* Driver Object Pointer
 */
extern DP_OBJ *DP_GetInstance(void);

/**
 *  @}
 */

#endif  /* DP_OBJ_IF_H */
