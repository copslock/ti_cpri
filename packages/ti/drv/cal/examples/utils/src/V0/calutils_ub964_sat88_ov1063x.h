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
 *  \addtogroup CALUTILS_UB960_API
 *  @{
 */

/**
 *  \file calutils_ub960.h
 *
 *  \brief Defines APIs to initialize, de-initialize UB960 EVM, address aliases
 *          & reset sensors.
 *
 *  TODO: Get serializer address function should be common for all modules.
 *          Right now, its seperate for TIDA, SAT0088 & IMI modules
 */

#ifndef CALUTILS_UB964_SAT88_OV10640_H_
#define CALUTILS_UB964_SAT88_OV10640_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/drv/fvid2/fvid2.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**< I2C Instance to be used */
#define CALUTILS_UB964_I2C_INST                  (0x1U)

/**< I2C Inst Base addr */
#define CALUTILS_UB964_I2C_INST_BASEADDR         (0x02010000U)

/**< UB960 I2C Address */
#define UB960_I2C_ADDRESS           (0x3D)

#define SAT0088_OV10635_SER_ADDR                    (0x58U) /* 0xB0/2 */
/**< 7 Bit serailizer address */
#define SAT0088_OV10635_SENSOR_ADDR                 (0x30U) /* 0x60/2 */
/**< 7 Bit sensor address */

#define SAT0088_OV10635_PORT_0_SER_ALIAS_ADDR       (0x74U)
/**< Serializer address */
#define SAT0088_OV10635_PORT_0_SENSOR_ALIAS_ADDR    (0x38U) /* 0x70/2 */
/**< Sensor address */
#define SAT0088_OV10635_PORT_1_SER_ALIAS_ADDR       (0x76U)
/**< Serializer address */
#define SAT0088_OV10635_PORT_1_SENSOR_ALIAS_ADDR    (0x39U) /* 0x72/2 */
/**< Sensor address */
#define SAT0088_OV10635_PORT_2_SER_ALIAS_ADDR       (0x78U)
/**< Serializer address */
#define SAT0088_OV10635_PORT_2_SENSOR_ALIAS_ADDR    (0x3AU) /* 0x74/2 */
/**< Sensor address */
#define SAT0088_OV10635_PORT_3_SER_ALIAS_ADDR       (0x7AU)
/**< Serializer address */
#define SAT0088_OV10635_PORT_3_SENSOR_ALIAS_ADDR    (0x3BU) /* 0x76/2 */
/**< Sensor address */

/** \brief Log enable for CAL modules. */
#define calUtilsTrace       (GT_INFO | GT_TraceState_Enable)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

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
} CalUtils_Ub960I2cParams;

/**
 *  \brief Register address and value pair, with delay.
 */
typedef struct
{
    uint16_t nRegAddr;
    /**< Register Address */
    uint8_t nRegValue;
    /**< Slave Address */
    uint32_t nDelay;
    /**< Delay to be applied, after the register is programmed */
} CalUtils_SensorConfigParams;



/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Initializes the UB954 & UB953.
 *          Initializes UB954, UB953 and configures to access 2 other remote
 *          I2C slaves
 */
void CalUtils_appInitUb964_sat88_ov1063x(void);

/**
 *  \brief De Initializes the UB954 & UB953
 *              Turns off power to 1 slave, resets UB953 & UB954
 */
void CalUtils_appDeinitUb964_sat88_ov1063x(void);


#ifdef __cplusplus
}
#endif

#endif  /* #define CALUTILS_UB95X_H_ */

/* @} */
