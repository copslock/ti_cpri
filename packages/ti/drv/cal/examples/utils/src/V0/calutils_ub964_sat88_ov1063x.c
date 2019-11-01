/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 */

/**
 *  \file calutils_ub964_sat88_ov1063x.c
 *
 *  \brief Implements APIs to initialize, de-initialize UB964 EVM,
 *         With SAT0088 module and the OV1063x 
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/soc.h>
#include <calutils_ub964_sat88_ov1063x.h>
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define I2C_TRANSACTION_TIMEOUT     (2000U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
I2C_Handle gUb960I2cHandle = NULL;
uint8_t    gLvdsOv1063xAddr[4] = {0x38, 0x39, 0x3A, 0x3B};

extern uint32_t gUb960Cfg_SAT0088_OV10635_size;
extern uint32_t gUb960Cfg_Ov1063xLvdsSensorsDefault_size;
extern const CalUtils_SensorConfigParams gUb960Cfg_Ov1063xLvdsSensorsDefault[];
extern const CalUtils_Ub960I2cParams gUb960Cfg_SAT0088_OV10635[];

/* ========================================================================== */
/*                          Function Implementation                           */
/* ========================================================================== */
void CalUtils_appInitUb964_sat88_ov1063x(void)
{
    I2C_Params i2cParams;
    I2C_Transaction i2cTransaction;
    I2C_HwAttrs   i2c_cfg;
    int16_t transferStatus;
    UInt8  nRegValue[5];
    UInt16 nAddr;

    uint32_t idx, numRegs, sensorIdx;

    /* Get the default I2C init configurations */
    I2C_socGetInitCfg(CALUTILS_UB964_I2C_INST, &i2c_cfg);
    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(CALUTILS_UB964_I2C_INST, &i2c_cfg);

    I2C_init();

    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.bitRate      = I2C_400kHz;

    gUb960I2cHandle = I2C_open(CALUTILS_UB964_I2C_INST, &i2cParams);

    numRegs = gUb960Cfg_SAT0088_OV10635_size;
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = UB960_I2C_ADDRESS;

    GT_0trace(calUtilsTrace, GT_INFO, "Configuring UB 964\r\n");
    for (idx = 0U; idx < numRegs; idx++)
    {
        i2cTransaction.writeBuf = (uint8_t *)&gUb960Cfg_SAT0088_OV10635[idx];
        i2cTransaction.writeCount = 2U;
        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0U;
        i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
        transferStatus = I2C_transfer(gUb960I2cHandle, &i2cTransaction);
    
        if (transferStatus != I2C_STS_SUCCESS)
        {
            GT_1trace(calUtilsTrace, GT_ERR,
                      "Could not configure UB954 in regWrite of idx: %d!!!\r\n", idx);
            break;
        }
        if (gUb960Cfg_SAT0088_OV10635[idx].nDelay != 0U)
        {
            Osal_delay(gUb960Cfg_SAT0088_OV10635[idx].nDelay);
        }
        GT_1trace(calUtilsTrace, GT_DEBUG, "Written I2C reg index: %d \r\n", idx);
    }
    
    /* Now Configure All the 4 ov1063x sensors connected over LVDS. */
    for (sensorIdx = 0U; sensorIdx < 4U; sensorIdx++)
    {
        GT_1trace(calUtilsTrace, GT_INFO, "Configuring Sensor %d\r\n", sensorIdx);
        I2C_transactionInit(&i2cTransaction);
        i2cTransaction.slaveAddress = gLvdsOv1063xAddr[sensorIdx];
        numRegs = gUb960Cfg_Ov1063xLvdsSensorsDefault_size;
        for (idx = 0U; idx < numRegs; idx++)
        {
            nAddr = gUb960Cfg_Ov1063xLvdsSensorsDefault[idx].nRegAddr & (UInt16) 0xFFFFU;
            /* MSB of the address */
            nRegValue[0] = (UInt8) ((UInt16) (nAddr & 0xFFFFU) >> 8);
            /* LSB of the address */
            nRegValue[1] = (UInt8) (nAddr & 0x00FFU);
            /* Data */
            nRegValue[2] = gUb960Cfg_Ov1063xLvdsSensorsDefault[idx].nRegValue;

            i2cTransaction.writeBuf = nRegValue;
            i2cTransaction.writeCount = 3U;
            i2cTransaction.readBuf = NULL;
            i2cTransaction.readCount = 0U;
            i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
            transferStatus = I2C_transfer(gUb960I2cHandle, &i2cTransaction);
            if (transferStatus != I2C_STS_SUCCESS)
            {
                GT_1trace(calUtilsTrace, GT_ERR,
                        "Could not configure OV1063x in regWrite of idx: %d!!!\r\n", idx);
                break;
            }
            if (gUb960Cfg_Ov1063xLvdsSensorsDefault[idx].nDelay != 0U)
            {
                Osal_delay((gUb960Cfg_Ov1063xLvdsSensorsDefault[idx].nDelay) * 100);
            }
            GT_2trace(calUtilsTrace, GT_DEBUG, "Written I2C Sensor index: %d, reg index: %d \r\n", sensorIdx, idx);
        }
    }
    
    GT_0trace(calUtilsTrace, GT_INFO, "Configured UB964. !!!\r\n");

    return;
}

void CalUtils_appDeinitUb964_sat88_ov1063x(void)
{
    I2C_Transaction i2cTransaction;
    uint8_t         data[2];
    int16_t transferStatus;

    /* Disable Forwarding
     *   regAddr = 0x20;
     *   regValue = 0x48;
     */
    data[0] = 0x20;
    data[1] = 0x48;
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = UB960_I2C_ADDRESS;
    i2cTransaction.writeBuf = (uint8_t *)&data[0];
    i2cTransaction.writeCount = 2U;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0U;
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
    transferStatus = I2C_transfer(gUb960I2cHandle, &i2cTransaction);

    if (transferStatus != I2C_STS_SUCCESS)
    {
        GT_0trace(calUtilsTrace, GT_ERR, "Could not disable forwarding UB954 !!!\r\n");
    }
    else
    {
        /* Reset
        *   regAddr = 0x01;
        *   regValue = 0x03;
        */
        data[0] = 0x01;
        data[1] = 0x03;
        i2cTransaction.writeBuf = (uint8_t *)&data[0];
        i2cTransaction.writeCount = 2U;
        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0U;
        i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
        transferStatus = I2C_transfer(gUb960I2cHandle, &i2cTransaction);
        /* TODO: Removing delay results in I2C error. Need to be checked.     */
        Osal_delay(100);
        if (transferStatus != I2C_STS_SUCCESS)
        {
            GT_0trace(calUtilsTrace, GT_ERR, "Could not reset UB954 !!!\r\n");
        }
    }
    I2C_close(gUb960I2cHandle);
    return;
}


