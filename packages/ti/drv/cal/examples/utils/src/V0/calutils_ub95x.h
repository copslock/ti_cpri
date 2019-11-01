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

#ifndef CALUTILS_UB95X_H_
#define CALUTILS_UB95X_H_

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
#define CALUTILS_UB954_I2C_INST                  (0x1U)

/**< I2C Inst Base addr */
#define CALUTILS_UB954_I2C_INST_BASEADDR         (0x02010000U)

/** \brief Log enable for CAL modules. */
#define calUtilsTrace       (GT_INFO | GT_TraceState_Enable)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Initializes the UB954 & UB953.
 *          Initializes UB954, UB953 and configures to access 2 other remote
 *          I2C slaves
 */
void CalUtils_appInitUb954_Ub953(void);

/**
 *  \brief De Initializes the UB954 & UB953
 *              Turns off power to 1 slave, resets UB953 & UB954
 */
void CalUtils_appDeInitUb954_Ub953(void);


#ifdef __cplusplus
}
#endif

#endif  /* #define CALUTILS_UB95X_H_ */

/* @} */
