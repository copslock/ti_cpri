/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 *  \ingroup DRV_IPC_MODULE
 *  \defgroup DRV_IPC_SOC_MODULE IPC SoC Config
 *            This is IPC documentation specific to AM65xx SoC
 *
 *  @{
 */

/**
 *  \file ipc_soc.h
 *
 *  \brief IPC Low Level Driver AM65XX SOC specific file.
 */
#ifndef IPC_SOC_V0_H_
#define IPC_SOC_V0_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief VRing Buffer Size required for all core
 * combinations.
 */
#define IPC_VRING_BUFFER_SIZE   (0x200000)

/** \brief Core definitions */
#define    IPC_MPU1_0           (0)    /**< ARM A53 - VM0 */
#define    IPC_MCU1_0           (1)    /**< ARM MCU  R5F - core0 */
#define    IPC_MCU1_1           (2)    /**< ARM MCU  R5F - core1 */
#define    IPC_MAX_PROCS        (3)    /**< Maximum Processors */

#define    IPC_MAILBOX_CLUSTER_CNT              (12)
#define    IPC_MAILBOX_USER_CNT                  (4)
#define    MAIN_NAVSS_MAILBOX_INPUTINTR_MAX    (440)
#define    MAIN_NAVSS_MAILBOX_OUTPUTINTR_MAX   (512)

#define IPC_MCU_NAVSS0_INTR0_CFG_BASE    (CSL_NAVSS0_INTR0_INTR_ROUTER_CFG_BASE)

/* Refer to Maxwell TRM (section 10.1.1.3) - NAVSS Integration */
#define    NAVSS512_MPU1_0_OUTPUT_OFFSET                   (112)

#define    NAVSS512_MCU1R5F0_OUTPUT_OFFSET                  (120)
#define    NAVSS512_MCU1R5F1_OUTPUT_OFFSET                  (121)

#define    MAIN2MCU0_INTR_ROUTER_INPUT_BASE                 (184)
#define    MAIN2MCU1_INTR_ROUTER_INPUT_BASE                 (186)

#define    MAIN2MCU0_INTR_ROUTER_OUTPUT_BASE                 (0)
#define    MAIN2MCU1_INTR_ROUTER_OUTPUT_BASE                 (1)

/* CPU local interrupt Number */
#define    MPU1_0_MBINTR_OFFSET             (496)

#define    MCU1R5F0_MBINTR_OFFSET           (160)
#define    MCU1R5F1_MBINTR_OFFSET           (162)

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */



/* @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#ifdef IPC_SUPPORT_SCICLIENT
int32_t Ipc_sciclientIrqRelease(uint16_t remoteId, int32_t clusterId,
        int32_t userId, uint32_t intNumber);
int32_t Ipc_sciclientIrqSet(uint16_t remoteId, int32_t clusterId, int32_t userId,
            uint32_t intNumber);
#endif
 
int32_t Ipc_main2mcu_intRouter(Ipc_MbConfig *cfg);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_SOC_V0_H_ */

/* @} */
