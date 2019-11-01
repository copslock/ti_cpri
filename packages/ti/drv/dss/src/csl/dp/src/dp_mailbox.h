/******************************************************************************
 *
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
 *
 ******************************************************************************
 *
 * dp_mailbox.h
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-36 "Function called from more then 5 functions, DRV-3823" */

#ifndef DP_MAILBOX_H
#define DP_MAILBOX_H

#define DP_MAILBOX_HEADER_SIZE 4U

/**
 * Mailbox module IDs
 */
#define MB_MODULE_ID_DP_TX          0x01
#define MB_MODULE_ID_HDCP_TX        0x07
#define MB_MODULE_ID_HDCP_GENERAL   0x09
#define MB_MODULE_ID_GENERAL        0x0A

/**
 * Start mailbox message.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] module ID of firmware module responsible for processing command.
 * @param[in] opcode ID (code) of the command.
 */
void messageStart(DP_PrivateData* pD, uint8_t module, uint8_t opcode);

/**
 * Append 8-bit value to message.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] value Value to append to message.
 */
void messageWriteUint8(DP_PrivateData* pD, uint8_t value);

/**
 * Append 16-bit value to message, in Big Endian.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] value Value to append to message.
 */
void messageWriteUint16(DP_PrivateData* pD, uint16_t value);

/**
 * Append 32-bit value to message, in Big Endian.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] value Value to append to message.
 */
void messageWriteUint32(DP_PrivateData* pD, uint32_t value);

/**
 * Append 3-byte value to message, in Big Endian. MSB is unused.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] value Value to append to message.
 */
void messageWrite3Bytes(DP_PrivateData* pD, uint32_t value);

/**
 * Copy bytes from buffer to mailbox command.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] buff Pointer to buffer to copy data from.
 * @param[in] length Length (in bytes) of data to copy. Max value is defined by
 * DP_MAX_MAILBOX_PAYLOAD.
 */
void messageWriteBuffer(DP_PrivateData *pD, const uint8_t *buffer, uint16_t length);

/**
 * Finish mailbox message, by writing its size to appropriate
 * location.
 * @param[in] pD Driver state info specific to this instance.
 */
void messageFinish(DP_PrivateData* pD);

/**
 * Transmit mailbox message. Function blocks, until transmission is
 * complete.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] busType Bus type (APB or SAPB) to be used for transmission.
 */
void messageTransmit(DP_PrivateData* pD, DP_BusType busType);

/**
 * Receive mailbox message. Function blocks, until message is received.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] busType Bus type (APB or SAPB) to be used for receive.
 */
void messageReceive(DP_PrivateData* pD, DP_BusType busType);

/**
 * Get mailbox message header. NULL pointer may be used for any value, except
 * pD, if that value is not needed.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] module ID of firmware module that sent response.
 * @param[out] opcode ID (code) of the response.
 * @param[out] length Length of the response, excluding header.
 */
void messageGetHeader(DP_PrivateData *pD, uint8_t *module, uint8_t *opcode, uint16_t *length);

/**
 * Verify message and opcode of the message
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] module ID of firmware module, that should be in the response.
 * @param[in] opcode ID (code) of the command, that should be in the response.
 * @return 1 module and opcode matches response.
 * @return 0 module and opcode does not match response.
 */
uint8_t messageHeaderMatches(const DP_PrivateData *pD, uint8_t module, uint8_t opcode);

/**
 * Read 8-bit value from message.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] value Pointer to variable to be filled with value read.
 */
void messageReadUint8(DP_PrivateData *pD, uint8_t *value);

/**
 * Read 16-bit Big Endian value from message.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] value Pointer to variable to be filled with value read.
 */
void messageReadUint16(DP_PrivateData *pD, uint16_t *value);

/**
 * Read 32-bit Big Endian value from message.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] value Pointer to variable to be filled with value read.
 */
void messageReadUint32(DP_PrivateData *pD, uint32_t *value);

/**
 * Read 3-byte Big Endian value from message. MSB is unused.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] value Pointer to variable to be filled with value read.
 */
void messageRead3Bytes(DP_PrivateData* pD, uint32_t* value);

/**
 * Copy bytes from mailbox response to a buffer.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] buff Pointer to buffer to be filled with values read.
 * @param[in] length Length (in bytes) of data to copy. Max value is defined by
 * DP_MAX_MAILBOX_PAYLOAD.
 */
void messageReadBuffer(DP_PrivateData *pD, uint8_t *buffer, uint16_t length);

#endif

/* parasoft-end-suppress METRICS-36 */
