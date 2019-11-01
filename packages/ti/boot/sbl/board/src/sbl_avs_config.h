/**
 *  \file   sbl_avs_config.h
 *
 *  \brief  This file contains functions and data related to AVS and ABB
 *          configuration for the voltage rails.
 *
 */

/*
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

 #include <stdint.h>
 #include <string.h>

/* Macro values defining the different OPP Configuration supported. */
#define OPP_MODE_NOM                  (0)
#define OPP_MODE_OD                   (1)
#define OPP_MODE_HIGH                 (2)

/**
 * struct volts_efuse_data - efuse definition for voltage
 * @reg:    register address for efuse
 * @reg_bits:   Number of bits in a register address, mandatory.
 */
typedef struct efuse_data
{
    uint32_t reg;
    uint8_t  reg_bits;
}efuse_data_t;

typedef struct voltage_rail {
    uint32_t     value;
    uint32_t     addr;
    efuse_data_t efuse;
}voltage_rail_t;

typedef struct vcores_data
{
    voltage_rail_t mpu;
    voltage_rail_t core;
    voltage_rail_t dsp;
    voltage_rail_t gpu;
    voltage_rail_t iva;
}vcores_data_t;

typedef struct abb_data {
    uint32_t efuseReg;
    uint32_t setupReg;
    uint32_t ctrlReg;
    uint32_t irqReg;
    uint32_t irqBit;
    uint32_t ldoVbbCtrlReg;
}abb_data_t;

typedef struct boardOppData
{
    uint32_t oppVal;
    const vcores_data_t *pboardOppData;
}boardOppData_t;

/**
 *  \brief EVM Board ID.
 */
typedef enum boardId
{
    BOARD_MIN,
    /**< Should be the first value of this enumeration. */
    BOARD_UNKNOWN = BOARD_MIN,
    /**< Unknown board. */
    BOARD_GPEVMAM572x = 0x1U,
    /**< General Purpose EVM */
    BOARD_IDKAM572x = 0x2U,
    /**< AM572x IDK */
    BOARD_IDKAM571x = 0x3U,
    /**< AM571x IDK */
	BOARD_IDKAM574x = 0x4U,
    /**< AM574x IDK */
    BOARD_CUSTOM = 0x5U,
    /**< Any other custom board. */
    BOARD_MAX = BOARD_CUSTOM
    /**< Max board ID. */
} boardId_t;

/**
 *  \brief    This API performs the AVS by scaling the the domain voltage level
 *            to specific values within the operational voltage range of the
 *            device.
 *
 *  \param    oppMode   OPP value for which the OPP configuration is to be done.
 */
void SBL_Configure_AVS(uint32_t oppMode);
