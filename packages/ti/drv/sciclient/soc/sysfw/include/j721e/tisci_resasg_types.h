/*
 *  Copyright (C) 2017-2019 Texas Instruments Incorporated
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
 * \ingroup TISCI
 * \defgroup tisci_resasg_types tisci_resasg_types
 *
 * DMSC controls the power management, security and resource management
 * of the device.
 *
 *
 * @{
 */
/**
 *
 *  \brief  This file contains:
 *
 *          WARNING!!: Autogenerated file from SYSFW. DO NOT MODIFY!!
 * Resource Management
 *
 * Resource Assignment Subtype definitions
 *
 * Data version: 190425_000000
 *
 */
#ifndef TISCI_RESASG_TYPES_H
#define TISCI_RESASG_TYPES_H

/**
 * Resource assignment type shift
 */
#define TISCI_RESASG_TYPE_SHIFT (0x0006U)
/**
 * Resource assignment type mask
 */
#define TISCI_RESASG_TYPE_MASK (0xFFC0U)
/**
 * Resource assignment subtype shift
 */
#define TISCI_RESASG_SUBTYPE_SHIFT (0x0000U)
/**
 * Resource assignment subtype mask
 */
#define TISCI_RESASG_SUBTYPE_MASK (0x003FU)
/**
 * Macro to create unique resource assignment types using type and subtype
 */

#define TISCI_RESASG_UTYPE(type, subtype) \
    (((type << TISCI_RESASG_TYPE_SHIFT) & TISCI_RESASG_TYPE_MASK) | \
     ((subtype << TISCI_RESASG_SUBTYPE_SHIFT) & TISCI_RESASG_SUBTYPE_MASK))

/**
 * IA subtypes definitions
 */
#define TISCI_RESASG_SUBTYPE_IA_VINT (0x000AU)
#define TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_GEVT (0x000BU)
#define TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_MEVT (0x000CU)
#define TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT (0x000DU)
#define TISCI_RESASG_SUBTYPES_IA_CNT (0x0004U)

/**
 * IRQ subtypes definitions
 */
#define TISCI_RESASG_SUBTYPE_C66SS0_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP0_FROM_C66SS0_INTROUTER0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_C66SS0_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP1_FROM_C66SS0_INTROUTER0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_C66SS0_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP2_FROM_C66SS0_INTROUTER0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_C66SS0_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP3_FROM_C66SS0_INTROUTER0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_C66SS0_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP4_FROM_C66SS0_INTROUTER0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_C66SS1_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP0_FROM_C66SS1_INTROUTER0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_C66SS1_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP1_FROM_C66SS1_INTROUTER0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_C66SS1_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP2_FROM_C66SS1_INTROUTER0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_C66SS1_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP3_FROM_C66SS1_INTROUTER0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_C66SS1_CORE0_C66_EVENT_IN_SYNC_IRQ_GROUP4_FROM_C66SS1_INTROUTER0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_IRQ_GROUP0_FROM_CMPEVENT_INTRTR0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_IRQ_GROUP0_FROM_WKUP_GPIOMUX_INTRTR0 (0x0005U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_IRQ_GROUP1_FROM_NAVSS0_INTR_ROUTER_0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_IRQ_GROUP2_FROM_NAVSS0_INTR_ROUTER_0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_GIC500SS_SPI_IRQ_GROUP0_FROM_CMPEVENT_INTRTR0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_GIC500SS_SPI_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_GIC500SS_SPI_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_GIC500SS_SPI_IRQ_GROUP0_FROM_WKUP_GPIOMUX_INTRTR0 (0x0005U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_GIC500SS_SPI_IRQ_GROUP1_FROM_NAVSS0_INTR_ROUTER_0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_COMPUTE_CLUSTER0_GIC500SS_SPI_IRQ_GROUP2_FROM_NAVSS0_INTR_ROUTER_0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_CPSW0_CPTS_HW1_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_CPSW0_CPTS_HW2_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_CPSW0_CPTS_HW3_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_CPSW0_CPTS_HW4_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_CPSW0_CPTS_HW5_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_CPSW0_CPTS_HW6_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0005U)
#define TISCI_RESASG_SUBTYPE_CPSW0_CPTS_HW7_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0006U)
#define TISCI_RESASG_SUBTYPE_CPSW0_CPTS_HW8_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0007U)
#define TISCI_RESASG_SUBTYPE_ESM0_ESM_PLS_EVENT0_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_ESM0_ESM_PLS_EVENT1_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_ESM0_ESM_PLS_EVENT2_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_MCU_CPSW0_CPTS_HW3_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_MCU_CPSW0_CPTS_HW4_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_MCU_NAVSS0_INTAGGR_0_INTAGGR_LEVI_PEND_IRQ_GROUP0_FROM_WKUP_GPIOMUX_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE0_INTR_IRQ_GROUP0_FROM_MAIN2MCU_LVL_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE0_INTR_IRQ_GROUP0_FROM_MAIN2MCU_PLS_INTRTR0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE0_INTR_IRQ_GROUP0_FROM_MCU_NAVSS0_INTR_ROUTER_0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE0_INTR_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE0_INTR_IRQ_GROUP0_FROM_WKUP_GPIOMUX_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE1_INTR_IRQ_GROUP0_FROM_MAIN2MCU_LVL_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE1_INTR_IRQ_GROUP0_FROM_MAIN2MCU_PLS_INTRTR0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE1_INTR_IRQ_GROUP0_FROM_MCU_NAVSS0_INTR_ROUTER_0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE1_INTR_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_MCU_R5FSS0_CORE1_INTR_IRQ_GROUP0_FROM_WKUP_GPIOMUX_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_NAVSS0_UDMASS_INTAGGR_0_INTAGGR_LEVI_PEND_IRQ_GROUP0_FROM_CMPEVENT_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_NAVSS0_UDMASS_INTAGGR_0_INTAGGR_LEVI_PEND_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_NAVSS0_UDMASS_INTAGGR_0_INTAGGR_LEVI_PEND_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_NAVSS512L_MAIN_0_CPTS0_HW1_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_NAVSS512L_MAIN_0_CPTS0_HW2_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_NAVSS512L_MAIN_0_CPTS0_HW3_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_NAVSS512L_MAIN_0_CPTS0_HW4_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_NAVSS512L_MAIN_0_CPTS0_HW5_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_NAVSS512L_MAIN_0_CPTS0_HW6_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0005U)
#define TISCI_RESASG_SUBTYPE_NAVSS512L_MAIN_0_CPTS0_HW7_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0006U)
#define TISCI_RESASG_SUBTYPE_NAVSS512L_MAIN_0_CPTS0_HW8_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0007U)
#define TISCI_RESASG_SUBTYPE_PCIE0_PCIE_CPTS_HW2_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_PCIE1_PCIE_CPTS_HW2_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_PCIE2_PCIE_CPTS_HW2_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_PCIE3_PCIE_CPTS_HW2_PUSH_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG0_PR1_EDC0_LATCH0_IN_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG0_PR1_EDC0_LATCH1_IN_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG0_PR1_EDC1_LATCH0_IN_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG0_PR1_EDC1_LATCH1_IN_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG0_PR1_IEP0_CAP_INTR_REQ_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG0_PR1_IEP1_CAP_INTR_REQ_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0005U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG0_PR1_SLV_INTR_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0006U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG1_PR1_EDC0_LATCH0_IN_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG1_PR1_EDC0_LATCH1_IN_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG1_PR1_EDC1_LATCH0_IN_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG1_PR1_EDC1_LATCH1_IN_IRQ_GROUP0_FROM_TIMESYNC_INTRTR0 (0x0003U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG1_PR1_IEP0_CAP_INTR_REQ_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0004U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG1_PR1_IEP1_CAP_INTR_REQ_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0005U)
#define TISCI_RESASG_SUBTYPE_PRU_ICSSG1_PR1_SLV_INTR_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0006U)
#define TISCI_RESASG_SUBTYPE_R5FSS0_CORE0_INTR_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_R5FSS0_CORE0_INTR_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_R5FSS0_CORE0_INTR_IRQ_GROUP0_FROM_R5FSS0_INTROUTER0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_R5FSS0_CORE1_INTR_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_R5FSS0_CORE1_INTR_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_R5FSS0_CORE1_INTR_IRQ_GROUP0_FROM_R5FSS0_INTROUTER0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_R5FSS1_CORE0_INTR_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_R5FSS1_CORE0_INTR_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_R5FSS1_CORE0_INTR_IRQ_GROUP0_FROM_R5FSS1_INTROUTER0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_R5FSS1_CORE1_INTR_IRQ_GROUP0_FROM_GPIOMUX_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_R5FSS1_CORE1_INTR_IRQ_GROUP0_FROM_NAVSS0_INTR_ROUTER_0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_R5FSS1_CORE1_INTR_IRQ_GROUP0_FROM_R5FSS1_INTROUTER0 (0x0002U)
#define TISCI_RESASG_SUBTYPE_WKUP_ESM0_ESM_PLS_EVENT0_IRQ_GROUP0_FROM_WKUP_GPIOMUX_INTRTR0 (0x0000U)
#define TISCI_RESASG_SUBTYPE_WKUP_ESM0_ESM_PLS_EVENT1_IRQ_GROUP0_FROM_WKUP_GPIOMUX_INTRTR0 (0x0001U)
#define TISCI_RESASG_SUBTYPE_WKUP_ESM0_ESM_PLS_EVENT2_IRQ_GROUP0_FROM_WKUP_GPIOMUX_INTRTR0 (0x0002U)
#define TISCI_RESASG_SUBTYPES_IRQ_CNT (0x005AU)

/**
 * Proxy subtypes definitions
 */
#define TISCI_RESASG_SUBTYPE_PROXY_PROXIES (0x0000U)
#define TISCI_RESASG_SUBTYPES_PROXY_CNT (0x0001U)

/**
 * RA subtypes definitions
 */
#define TISCI_RESASG_SUBTYPE_RA_ERROR_OES (0x0000U)
#define TISCI_RESASG_SUBTYPE_RA_GP (0x0001U)
#define TISCI_RESASG_SUBTYPE_RA_UDMAP_RX (0x0002U)
#define TISCI_RESASG_SUBTYPE_RA_UDMAP_TX (0x0003U)
#define TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_EXT (0x0004U)
#define TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H (0x0005U)
#define TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_UH (0x0006U)
#define TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H (0x0007U)
#define TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_UH (0x0008U)
#define TISCI_RESASG_SUBTYPE_RA_VIRTID (0x000AU)
#define TISCI_RESASG_SUBTYPE_RA_MONITORS (0x000BU)
#define TISCI_RESASG_SUBTYPES_RA_CNT (0x000BU)

/**
 * UDMAP subtypes definitions
 */
#define TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON (0x0000U)
#define TISCI_RESASG_SUBTYPE_UDMAP_INVALID_FLOW_OES (0x0001U)
#define TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_TRIGGER (0x0002U)
#define TISCI_RESASG_SUBTYPE_UDMAP_GLOBAL_CONFIG (0x0003U)
#define TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN (0x000AU)
#define TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN (0x000BU)
#define TISCI_RESASG_SUBTYPE_UDMAP_RX_UHCHAN (0x000CU)
#define TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN (0x000DU)
#define TISCI_RESASG_SUBTYPE_UDMAP_TX_ECHAN (0x000EU)
#define TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN (0x000FU)
#define TISCI_RESASG_SUBTYPE_UDMAP_TX_UHCHAN (0x0010U)
#define TISCI_RESASG_SUBTYPES_UDMAP_CNT (0x000BU)


/**
 * Total number of unique resource types for SoC
 */
#define TISCI_RESASG_UTYPE_CNT 138U

#endif /* TISCI_RESASG_TYPES_H */

/* @} */
