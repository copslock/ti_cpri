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
 *  \file calutils_ub95x.c
 *
 *  \brief Implements APIs to initialize, de-initialize UB954 * UB953 EVM,
 *           address aliases & reset sensors.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/soc.h>
#include <calutils_ub95x.h>
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**< UB954 I2C Address */
#define UB954_I2C_ADDRESS           (0x3D)

/**< UB953 I2C Address */
#define UB953_I2C_ADDRESS           (0x18)
/**< UB953 Alias I2C Address */
#define UB953_I2C_ALIAS_ADDRESS     (0x74)

/**< OV7261 I2C Address */
#define OV7261_I2C_ADDRESS          (0x60)
/**< OV7261 Alias I2C Address */
#define OV7261_I2C_ALIAS_ADDRESS    (0x38)

/**< Temperature sensor I2C Address */
#define TEMP_SENS_I2C_ADDRESS       (0x48)
/**< Temperature sensor Alias I2C Address */
#define TEMP_SENS_I2C_ALIAS_ADDRESS (0x76)

/**< LED Driver I2C Address */
#define LED_DRV_I2C_ADDRESS         (0x49)
/**< LED Driver Alias I2C Address */
#define LED_DRV_I2C_ALIAS_ADDRESS   (0x78)

#define I2C_TRANSACTION_TIMEOUT     (2000U)

/**
 *  \brief Register address and value pair, with delay.
 */
typedef struct
{
    uint8_t nRegAddr;
    /**< Register Address */
    uint8_t nRegValue;
    /**< Slave Address */
    uint32_t nDelay;
    /**< Delay to be applied, after the register is programmed */
} CalUtils_Ub95xI2cParams;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
I2C_Handle gI2cHandle = NULL;
static CalUtils_Ub95xI2cParams gUb954Cfg[]={
    {0x4c, 0x01, 0x1},
    {0x58, 0X5e, 0x2},
    {0x5b, (UB953_I2C_ADDRESS           << 1U), 0x1},
    {0x5c, (UB953_I2C_ALIAS_ADDRESS     << 1U), 0x1},
    {0x5D, (OV7261_I2C_ADDRESS          << 1U), 0x1},
    {0x65, (OV7261_I2C_ALIAS_ADDRESS    << 1U), 0x1},
    {0x5e, (TEMP_SENS_I2C_ADDRESS       << 1U), 0x1},
    {0x66, (TEMP_SENS_I2C_ALIAS_ADDRESS << 1U), 0x1},
    {0x5f, (LED_DRV_I2C_ADDRESS         << 1U), 0x1},
    {0x67, (LED_DRV_I2C_ALIAS_ADDRESS   << 1U), 0x1},
    {0x1f, 0x02, 0x1},
    {0x12, 0x13, 0x1},
    {0x32, 0x01, 0x1},
    {0x33, 0x03, 0x1},
    {0xB0, 0x00, 0x1}, /* Indirect access to pattern genrator */
    {0xB1, 0x01, 0x1}, /* Select Reg PGEN_CTL */
    {0xB2, 0x01, 0x1}, /* Write 1 to it */
    {0x20, 0x00, 0x1}
    };
/* NOTE : Expect to see color bar, where the color values should be default
    i.e. first bar = 0xAA, 2nd Bar = 0x33, 3rd bar = 0xF0, 4th bar = 0x7F
        5th bar = 0x55, 6th bar = 0xCC, 7th bar = 0x0F and 8th bar = 0x80 */
/* ========================================================================== */
/*                          Function Implementation                           */
/* ========================================================================== */
void CalUtils_appInitUb954_Ub953(void)
{
    I2C_Params i2cParams;
    I2C_Transaction i2cTransaction;
    I2C_HwAttrs   i2c_cfg;
    int16_t transferStatus;

    uint32_t idx, numRegs;

    /* Get the default I2C init configurations */
    I2C_socGetInitCfg(CALUTILS_UB954_I2C_INST, &i2c_cfg);
    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(CALUTILS_UB954_I2C_INST, &i2c_cfg);

    I2C_init();

    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.bitRate      = I2C_400kHz;

    gI2cHandle = I2C_open(CALUTILS_UB954_I2C_INST, &i2cParams);

    numRegs = sizeof(gUb954Cfg) / (sizeof(gUb954Cfg[0]));
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = UB954_I2C_ADDRESS;

    for (idx = 0U; idx < numRegs; idx++)
    {
        i2cTransaction.writeBuf = (uint8_t *)&gUb954Cfg[idx];
        i2cTransaction.writeCount = 2U;
        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0U;
        i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
        transferStatus = I2C_transfer(gI2cHandle, &i2cTransaction);
    
        if (transferStatus != I2C_STS_SUCCESS)
        {
            GT_1trace(calUtilsTrace, GT_ERR,
                      "Could not configure UB954 in regWrite of idx: %d!!!\r\n", idx);
            break;
        }
    #if defined (BARE_METAL)
        Osal_delay((gUb954Cfg[idx].nDelay + 1));
    #else
        Osal_delay((gUb954Cfg[idx].nDelay + 1) * 100);
    #endif
        GT_1trace(calUtilsTrace, GT_DEBUG, "Written I2C reg index: %d \r\n", idx);
    }
    GT_1trace(calUtilsTrace, GT_DEBUG,
              "Configured UB954. Number of I2C regs Configured [%d]!!!\r\n", idx);

    return;
}

void CalUtils_appDeInitUb954_Ub953(void)
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
    i2cTransaction.slaveAddress = UB954_I2C_ADDRESS;
    i2cTransaction.writeBuf = (uint8_t *)&data[0];
    i2cTransaction.writeCount = 2U;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0U;
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
    transferStatus = I2C_transfer(gI2cHandle, &i2cTransaction);

    if (transferStatus != I2C_STS_SUCCESS)
    {
        GT_0trace(calUtilsTrace, GT_ERR, "Could not disable forwarding UB954 !!!\r\n");
    }
    else
    {
		/* TODO: Removing delay results in I2C error. Need to be checked.     */
		Osal_delay(1000);
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
        transferStatus = I2C_transfer(gI2cHandle, &i2cTransaction);
		/* TODO: Removing delay results in I2C error. Need to be checked.     */
		Osal_delay(1000);
        if (transferStatus != I2C_STS_SUCCESS)
        {
            GT_0trace(calUtilsTrace, GT_ERR, "Could not reset UB954 !!!\r\n");
        }
    }

    I2C_close(gI2cHandle);
    return;
}


