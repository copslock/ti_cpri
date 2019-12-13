/*
 *  Copyright (C) 2018 Texas Instruments Incorporated
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
 *
 */

/**
 *  \file   V1/sciclient_fmwMsgParams.h
 *
 *  \brief  This file contains the definition of all the parameter IDs for
 *          PM, RM, Security.
 */

#ifndef SCICLIENT_FMWMSGPARAMS_H_
#define SCICLIENT_FMWMSGPARAMS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** Undefined Param Undefined */
#define TISCI_PARAM_UNDEF                                        (0xFFFFFFFFU)

/**
 *  \anchor Sciclient_FirmwareABI
 *  \name Sciclient Firmware ABI revisions
 *  @{
 *  ABI revisions for compatibility check.
 */
/* ABI Major revision - Major revision changes
*       indicate backward compatibility breakage */
#define SCICLIENT_FIRMWARE_ABI_MAJOR                     (2U)
/* ABI Minor revision - Minor revision changes
*       indicate backward compatibility is maintained,
*       however, new messages OR extensions to existing
*       messages might have been adde */
#define SCICLIENT_FIRMWARE_ABI_MINOR                     (4U)
/* @} */

/**
 *  \anchor Sciclient_ContextIds
 *  \name Sciclient Context Ids
 *  @{
 *  Context IDs for Sciclient_ConfigPrms_t .
 */
/** R5_0(Non Secure): Cortex R5 context 0 on MCU island */
#define SCICLIENT_CONTEXT_R5_NONSEC_0 (0U)
/** R5_1(Secure): Cortex R5 context 1 on MCU island(Boot) */
#define SCICLIENT_CONTEXT_R5_SEC_0 (1U)
/** R5_2(Non Secure): Cortex R5 context 2 on MCU island */
#define SCICLIENT_CONTEXT_R5_NONSEC_1 (2U)
/** R5_3(Secure): Cortex R5 context 3 on MCU island */
#define SCICLIENT_CONTEXT_R5_SEC_1 (3U)
/** A72_0(Secure): Cortex A72 context 0 on Main island */
#define SCICLIENT_CONTEXT_A72_SEC_0 (4U)
/** A72_1(Secure): Cortex A72 context 1 on Main island */
#define SCICLIENT_CONTEXT_A72_SEC_1 (5U)
/** A72_2(Non Secure): Cortex A72 context 2 on Main island */
#define SCICLIENT_CONTEXT_A72_NONSEC_0 (6U)
/** A72_3(Non Secure): Cortex A72 context 3 on Main island */
#define SCICLIENT_CONTEXT_A72_NONSEC_1 (7U)
/** A72_4(Non Secure): Cortex A72 context 4 on Main island */
#define SCICLIENT_CONTEXT_A72_NONSEC_2 (8U)
/** C7X_0(Secure): C7x Context 0 on Main island */
#define SCICLIENT_CONTEXT_C7X_SEC_0 (9U)
/** C7X_1(Non Secure): C7x context 1 on Main island */
#define SCICLIENT_CONTEXT_C7X_NONSEC_0 (10U)
/** C6X_0_0(Secure): C6x_0 Context 0 on Main island */
#define SCICLIENT_CONTEXT_C6X_0_SEC_0 (11U)
/** C6X_0_1(Non Secure): C6x_0 context 1 on Main island */
#define SCICLIENT_CONTEXT_C6X_0_NONSEC_0 (12U)
/** C6X_1_0(Secure): C6x_1 Context 0 on Main island */
#define SCICLIENT_CONTEXT_C6X_1_SEC_0 (13U)
/** C6X_1_1(Non Secure): C6x_1 context 1 on Main island */
#define SCICLIENT_CONTEXT_C6X_1_NONSEC_0 (14U)
/** GPU_0(Non Secure): RGX context 0 on Main island */
#define SCICLIENT_CONTEXT_GPU_NONSEC_0 (15U)
/** MAIN_0_R5_0(Non Secure): Cortex R5_0 context 0 on Main island */
#define SCICLIENT_CONTEXT_MAIN_0_R5_NONSEC_0 (16U)
/** MAIN_0_R5_1(Secure): Cortex R5_0 context 1 on Main island */
#define SCICLIENT_CONTEXT_MAIN_0_R5_SEC_0 (17U)
/** MAIN_0_R5_2(Non Secure): Cortex R5_0 context 2 on Main island */
#define SCICLIENT_CONTEXT_MAIN_0_R5_NONSEC_1 (18U)
/** MAIN_0_R5_3(Secure): Cortex R5_0 context 3 on MCU island */
#define SCICLIENT_CONTEXT_MAIN_0_R5_SEC_1 (19U)
/** MAIN_1_R5_0(Non Secure): Cortex R5_1 context 0 on Main island */

#ifdef SOC_J721E
#define SCICLIENT_CONTEXT_MAIN_1_R5_NONSEC_0 (20U)
/** MAIN_1_R5_1(Secure): Cortex R5_1 context 1 on Main island */
#define SCICLIENT_CONTEXT_MAIN_1_R5_SEC_0 (21U)
/** MAIN_1_R5_2(Non Secure): Cortex R5_1 context 2 on Main island */
#define SCICLIENT_CONTEXT_MAIN_1_R5_NONSEC_1 (22U)
/** MAIN_1_R5_3(Secure): Cortex R5_1 context 3 on MCU island */
#define SCICLIENT_CONTEXT_MAIN_1_R5_SEC_1 (23U)
/** ICSSG_0(Non Secure): ICSSG context 0 on Main island */
#define SCICLIENT_CONTEXT_ICSSG_NONSEC_0 (24U)
/** Total number of possible contexts for application. */
#define SCICLIENT_CONTEXT_MAX_NUM                        (25U)
#else
#define SCICLIENT_CONTEXT_MAX_NUM                        (20U)
#endif
/* @} */

/**
 *  \anchor Sciclient_ProcessorIds
 *  \name Sciclient Processor Ids
 *  @{
 *  Processor IDs for the Processor Boot Configuration APIs.
 */
/**
 * COMPUTE_CLUSTER_J7ES_TB_VDC_MAIN_0: (Cluster 0 Processor 0)
 */
#define SCICLIENT_PROC_ID_COMPUTE_CLUSTER_J7ES_TB_VDC_MAIN_0_P0_A72_DUAL_1MB (0x20U)

/**
 * COMPUTE_CLUSTER_J7ES_TB_VDC_MAIN_0: (Cluster 0 Processor 1)
 */
#define SCICLIENT_PROC_ID_COMPUTE_CLUSTER_J7ES_TB_VDC_MAIN_0_P0_A72_DUAL_1MB_1 (0x21U)

/**
 * COMPUTE_CLUSTER_J7ES_TB_VDC_MAIN_0: (Cluster 4 Processor 0)
 */
#define SCICLIENT_PROC_ID_COMPUTE_CLUSTER_J7ES_TB_VDC_MAIN_0_P4_C711_512KB (0x30U)

/**
 * J7_MAIN_SEC_MMR_MAIN_0: (Cluster 2 Processor 0)
 */
#define SCICLIENT_PROC_ID_K3_C66_COREPAC_MAIN_0_K3_C66_CGEM_288KB_0 (0x03U)

/**
 * J7_MAIN_SEC_MMR_MAIN_0: (Cluster 3 Processor 0)
 */
#define SCICLIENT_PROC_ID_K3_C66_COREPAC_MAIN_1_K3_C66_CGEM_288KB_0 (0x04U)

/**
 * J7_MAIN_SEC_MMR_MAIN_0: (Cluster 0 Processor 0)
 */
#define SCICLIENT_PROC_ID_PULSAR_SL_MAIN_0_CPU0 (0x06U)

/**
 * J7_MAIN_SEC_MMR_MAIN_0: (Cluster 0 Processor 1)
 */
#define SCICLIENT_PROC_ID_PULSAR_SL_MAIN_0_CPU1 (0x07U)

/**
 * J7_MAIN_SEC_MMR_MAIN_0: (Cluster 1 Processor 0)
 */
#define SCICLIENT_PROC_ID_PULSAR_SL_MAIN_1_CPU0 (0x08U)

/**
 * J7_MAIN_SEC_MMR_MAIN_0: (Cluster 1 Processor 1)
 */
#define SCICLIENT_PROC_ID_PULSAR_SL_MAIN_1_CPU1 (0x09U)

/**
 * J7_MCU_SEC_MMR_MCU_0: (Cluster 0 Processor 0)
 */
#define SCICLIENT_PROC_ID_PULSAR_SL_MCU_0_CPU0 (0x01U)

/**
 * J7_MCU_SEC_MMR_MCU_0: (Cluster 0 Processor 1)
 */
#define SCICLIENT_PROC_ID_PULSAR_SL_MCU_0_CPU1 (0x02U)

/**
 * Total Number of processors in J721E
 */
#define SCICLIENT_SOC_NUM_PROCESSORS (0x0BU)
/* @} */

/** -------------------- Resource Management Parameters ---------------------*/

/**
 * Null Ring type
 */
#define TISCI_MSG_VALUE_RM_NULL_RING_TYPE     (0xFFFFu)
/**
 * Null Ring Index
 */
#define TISCI_MSG_VALUE_RM_NULL_RING_INDEX    (0xFFFFFFFFu)
/**
 * The ring base address register fields are not be modified if this value
 * is used for:
 * @ref tisci_msg_rm_ring_cfg_req::addr_lo
 * @ref tisci_msg_rm_ring_cfg_req::addr_hi
 */
#define TISCI_MSG_VALUE_RM_NULL_RING_ADDR     (0xFFFFFFFFu)
/**
 * The ring size field of the RING_SIZE register is not modified if this value
 * is used for:
 * @ref tisci_msg_rm_ring_cfg_req::count
 */
#define TISCI_MSG_VALUE_RM_NULL_RING_COUNT    (0xFFFFFFFFu)
/**
 * The ring mode field of the RING_SIZE register is not modified if this value
 * is used for:
 * @ref tisci_msg_rm_ring_cfg_req::mode
 */
#define TISCI_MSG_VALUE_RM_NULL_RING_MODE     (0xFFu)
/**
 * The ring elsize field of the RING_SIZE register is not modified if this
 * value is used for:
 * @ref tisci_msg_rm_ring_cfg_req::size
 */
#define TISCI_MSG_VALUE_RM_NULL_RING_SIZE     (0xFFu)
/**
 * Default value for @ref tisci_msg_rm_ring_cfg_req::order_id.
 * No changes to the order ID field of the ring will take place if this value is used.
 */
#define TISCI_MSG_VALUE_RM_NULL_ORDER_ID      (0xFFu)

/**
 * Value specified for @ref tisci_msg_rm_udmap_tx_ch_cfg_req::tx_chan_type and
 * @ref tisci_msg_rm_udmap_rx_ch_cfg_req::rx_chan_type when an explicit channel index is
 * provided.  DMSC RM will return a NACK if a non-NULL channel type is passed
 * along with an explicit channel index.
 */
#define TISCI_MSG_VALUE_RM_UDMAP_NULL_CH_TYPE                  (0xFFu)
/**
 * Value specified for @ref tisci_msg_rm_udmap_tx_ch_cfg_req::index and
 * tisci_msg_rm_udmap_rx_ch_cfg_req::index when the user wants to request DMSC RM
 * allocate an unused UDMAP channel.  When specified for the channel index a
 * valid channel type must be provided.
 */
#define TISCI_MSG_VALUE_RM_UDMAP_NULL_CH_INDEX                 (0xFFFFFFFFu)

/** ------------ Power Management Messages Parameters -----------------------*/
/**
 *  \anchor Sciclient_PmDeviceIds
 *  \name Power Management Device IDs
 *  @{
 *  Power Management Module Device IDs
 */
#include <ti/drv/sciclient/soc/sysfw/include/j721e/tisci_devices.h>
/* @} */

/**
 *  \anchor Sciclient_PmModuleClockIds
 *  \name Power Management Clock IDs Module Wise
 *  @{
 *  Power Management Module Clock IDs for individual modules.
 */
#include <ti/drv/sciclient/soc/sysfw/include/j721e/tisci_clocks.h>
/* @} */

/**
 * \brief Special ISC ID to refer to compute cluster privid registers
 */
#define TISCI_ISC_CC_ID                (160U)

/**
 *  \anchor Sciclient_IrqSrcIdxStart
 *  \name IRQ source index start
 *  @{
 *  Start offset of IRQ source index.
 */
#define TISCI_RINGACC0_OES_IRQ_SRC_IDX_START        (0U)
#define TISCI_RINGACC0_MON_IRQ_SRC_IDX_START        (1024U)
#define TISCI_RINGACC0_EOES_IRQ_SRC_IDX_START       (2048U)
#define TISCI_UDMAP0_TX_OES_IRQ_SRC_IDX_START       (0U)
#define TISCI_UDMAP0_TX_EOES_IRQ_SRC_IDX_START      (512U)
#define TISCI_UDMAP0_RX_OES_IRQ_SRC_IDX_START       (1024U)
#define TISCI_UDMAP0_RX_EOES_IRQ_SRC_IDX_START      (1280U)
#define TISCI_UDMAP0_RX_FLOW_EOES_IRQ_SRC_IDX_START (1536U)
/* @} */

#define SCICLIENT_C7X_NON_SECURE_INTERRUPT_NUM (9U)
#define SCICLIENT_C7X_SECURE_INTERRUPT_NUM     (10U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_FMWMSGPARAMS_H_ */
