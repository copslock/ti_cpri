/*
 *  Copyright (c) Texas Instruments Incorporated 2019
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
 *  \file udma_test_soc.h
 *
 *  \brief UT SOC specific file
 */

#ifndef UDMA_TEST_SOC_H_
#define UDMA_TEST_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if defined (BUILD_MCU1_0) || defined (BUILD_MCU1_1)
#define UDMA_TEST_DEFAULT_UDMA_INST     (UDMA_INST_ID_MCU_0)
#else
#define UDMA_TEST_DEFAULT_UDMA_INST     (UDMA_INST_ID_MAIN_0)
#endif

#define UDMA_TEST_RF_SOC                (UDMA_TEST_RF_SOC_J721E)

#define UDMA_TEST_RF_MAIN_BC_HC         (UDMA_TEST_RF_SOC | \
                                         UDMA_TEST_RF_CORE_MPU1_0 | \
                                         UDMA_TEST_RF_CORE_MCU2_0 | \
                                         UDMA_TEST_RF_CORE_MCU2_1 | \
                                         UDMA_TEST_RF_CFG_ALL)
#define UDMA_TEST_RF_MAIN_BC            (UDMA_TEST_RF_SOC | UDMA_TEST_RF_CORE_ALL | UDMA_TEST_RF_CFG_ALL)
#define UDMA_TEST_RF_MCU_BC_HC          (UDMA_TEST_RF_SOC | \
                                         UDMA_TEST_RF_CORE_MCU1_0 | \
                                         UDMA_TEST_RF_CFG_ALL)
#define UDMA_TEST_RF_MCU_BC             (UDMA_TEST_RF_SOC | \
                                         UDMA_TEST_RF_CORE_MPU1_0 | \
                                         UDMA_TEST_RF_CORE_MCU2_0 | \
                                         UDMA_TEST_RF_CORE_MCU2_1 | \
                                         UDMA_TEST_RF_CORE_MCU3_0 | \
                                         UDMA_TEST_RF_CORE_MCU3_1 | \
                                         UDMA_TEST_RF_CORE_C7X_1 | \
                                         UDMA_TEST_RF_CORE_C66X_1 | \
                                         UDMA_TEST_RF_CORE_C66X_2 | \
                                         UDMA_TEST_RF_CORE_MCU1_0 | \
                                         UDMA_TEST_RF_CFG_ALL)
#define UDMA_TEST_RF_DRU                (UDMA_TEST_RF_SOC | \
                                         UDMA_TEST_RF_CORE_MCU2_1 | \
                                         UDMA_TEST_RF_CORE_C7X_1 | \
                                         UDMA_TEST_RF_CORE_C66X_1 | \
                                         UDMA_TEST_RF_CORE_C66X_2 | \
                                         UDMA_TEST_RF_CFG_ALL)

/* Multipe task testcases - some have only one instance. Doesn't make sense to run from 1 task */
#define UDMA_TEST_RF_MAIN_BC_HC_MT      (UDMA_TEST_RF_MAIN_BC_HC)
#define UDMA_TEST_RF_MAIN_BC_MT         (UDMA_TEST_RF_MAIN_BC)
#define UDMA_TEST_RF_MCU_BC_HC_MT       (UDMA_TEST_RF_MCU_BC_HC)
#define UDMA_TEST_RF_MCU_BC_MT          (UDMA_TEST_RF_MCU_BC)
#define UDMA_TEST_RF_DRU_MT             (UDMA_TEST_RF_DRU)

#define UDMA_TEST_RF_FLOW               (UDMA_TEST_RF_SOC | \
                                         UDMA_TEST_RF_CORE_MPU1_0 | \
                                         UDMA_TEST_RF_CORE_MCU2_0 | \
                                         UDMA_TEST_RF_CORE_MCU2_1 | \
                                         UDMA_TEST_RF_CFG_ALL)

#define UDMA_TEST_RF_MAIN_BC_PACING     (UDMA_TEST_RF_MAIN_BC)
#define UDMA_TEST_RF_MAIN_BC_PAUSE      (UDMA_TEST_RF_MAIN_BC)

#define UDMA_TEST_MAIN_UHC_START        (0U)
#define UDMA_TEST_MCU_UHC_START         (0U)
#if defined (BUILD_MPU1_0)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MPU1_0)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (4U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (4U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (3U)
#define UDMA_TEST_MAX_DRU_CH            (0U)
#define UDMA_TEST_MAIN_HC_START         (4U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_MCU2_0)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU2_0)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (4U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (3U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (1U)
#define UDMA_TEST_MAX_DRU_CH            (0U)
#define UDMA_TEST_MAIN_HC_START         (8U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_MCU2_1)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU2_1)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (4U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (4U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (1U)
#define UDMA_TEST_MAX_DRU_CH            (2U)
#define UDMA_TEST_MAIN_HC_START         (12U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_MCU3_0)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU3_0)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (0U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (2U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (1U)
#define UDMA_TEST_MAX_DRU_CH            (0U)
#define UDMA_TEST_MAIN_HC_START         (4U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_MCU3_1)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU3_1)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (0U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (2U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (1U)
#define UDMA_TEST_MAX_DRU_CH            (0U)
#define UDMA_TEST_MAIN_HC_START         (4U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_C7X_1)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_C7X_1)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (0U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (2U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (1U)
#define UDMA_TEST_MAX_DRU_CH            (4U)
#define UDMA_TEST_MAIN_HC_START         (4U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_C66X_1)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_C66X_1)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (0U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (2U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (1U)
#define UDMA_TEST_MAX_DRU_CH            (4U)
#define UDMA_TEST_MAIN_HC_START         (4U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_C66X_2)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_C66X_2)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (0U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (2U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (1U)
#define UDMA_TEST_MAX_DRU_CH            (4U)
#define UDMA_TEST_MAIN_HC_START         (4U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_MCU1_0)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU1_0)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (0U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (2U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (2U)
#define UDMA_TEST_MAX_MCU_BC_CH         (2U)
#define UDMA_TEST_MAX_DRU_CH            (0U)
#define UDMA_TEST_MAIN_HC_START         (4U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif
#if defined (BUILD_MCU1_1)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU1_1)
#define UDMA_TEST_MAX_MAIN_BC_UHC_CH    (0U)
#define UDMA_TEST_MAX_MAIN_BC_HC_CH     (0U)
#define UDMA_TEST_MAX_MAIN_BC_CH        (2U)
#define UDMA_TEST_MAX_MCU_BC_UHC_CH     (0U)
#define UDMA_TEST_MAX_MCU_BC_HC_CH      (0U)
#define UDMA_TEST_MAX_MCU_BC_CH         (0U)
#define UDMA_TEST_MAX_DRU_CH            (0U)
#define UDMA_TEST_MAIN_HC_START         (4U)
#define UDMA_TEST_MCU_HC_START          (0U)
#endif

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* UDMA_TEST_SOC_H_ */
