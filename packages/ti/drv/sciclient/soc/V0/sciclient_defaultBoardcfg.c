/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  \file V0/sciclient_defaultBoardcfg.c
 *
 *  \brief File containing the boardcfg default data structure to
 *      send TISCI_MSG_BOARD_CONFIG message.
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/drv/sciclient/soc/V0/sciclient_defaultBoardcfg.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if defined (BUILD_MCU1_0)
const struct tisci_boardcfg gBoardConfigLow
__attribute__(( aligned(128), section(".boardcfg_data") )) =
{
    /* tisci_boardcfg_abi_rev */
    .rev = {
        .tisci_boardcfg_abi_maj = TISCI_BOARDCFG_ABI_MAJ_VALUE,
        .tisci_boardcfg_abi_min = TISCI_BOARDCFG_ABI_MIN_VALUE,
    },

    /* tisci_boardcfg_control */
    .control = {
        .subhdr = {
            .magic = TISCI_BOARDCFG_CONTROL_MAGIC_NUM,
            .size = sizeof(struct tisci_boardcfg_control),
        },
        /* Enable/disable support for System Firmware main isolation.
         * If disabled, main isolation SCI message will be rejected with NAK.
         */
        .main_isolation_enable = 0x5A,
        /* Host-ID allowed to send SCI-message for main isolation.
         * If mismatch, SCI message will be rejected with NAK.
         */
        .main_isolation_hostid = TISCI_HOST_ID_R5_1,
    },

    /* tisci_boardcfg_sec_proxy */
    .secproxy = {
        .subhdr = {
            .magic = TISCI_BOARDCFG_SECPROXY_MAGIC_NUM,
            .size = sizeof(struct tisci_boardcfg_secproxy),
        },
        /* Memory allocation for messages scaling factor. In current design,
         * only value of “1” is supported. For future design, a value of “2”
         * would double all memory allocations and credits, “3” would triple,
         * and so on.
         */
        .scaling_factor = 0x1,
        /* Memory allocation for messages profile number. In current design,
         * only a value of “1” is supported. “0” is always invalid due to
         * fault tolerance.
         */
        .scaling_profile = 0x1,
        /* Do not configure main nav secure proxy. This removes all MSMC memory
         * demands from System Firmware but limits MPU channels to one set of
         * secure and one set of insecure. In current design, supports only “0”.
         */
        .disable_main_nav_secure_proxy = 0,
    },

    /* tisci_boardcfg_msmc */
    .msmc = {
        .subhdr = {
            .magic = TISCI_BOARDCFG_MSMC_MAGIC_NUM,
            .size = sizeof(struct tisci_boardcfg_msmc),
        },
        /* If the whole memory is X MB the value you write to this field is n.
         * The value of n sets the cache size as n * X/32. The value of n should
         * be given in steps of 4, which makes the size of cache to be
         * configured in steps on X/8 MB.
         */
        .msmc_cache_size = 0x00,
    },

    /* boardcfg_dbg_cfg */
    .debug_cfg = {
        .subhdr = {
            .magic = TISCI_BOARDCFG_DBG_CFG_MAGIC_NUM,
            .size = sizeof(struct tisci_boardcfg_dbg_cfg),
        },
        /* This enables the trace for DMSC logging. Should be used only for
         * debug.
         */
        /* Uncomment for Debug */
        /*
        .trace_dst_enables = (TISCI_BOARDCFG_TRACE_DST_UART0 |
                              TISCI_BOARDCFG_TRACE_DST_ITM |
                              TISCI_BOARDCFG_TRACE_DST_MEM),
        .trace_src_enables = (TISCI_BOARDCFG_TRACE_SRC_PM |
                              TISCI_BOARDCFG_TRACE_SRC_RM |
                              TISCI_BOARDCFG_TRACE_SRC_SEC |
                              TISCI_BOARDCFG_TRACE_SRC_BASE |
                              TISCI_BOARDCFG_TRACE_SRC_USER |
                              TISCI_BOARDCFG_TRACE_SRC_SUPR)
        */
        .trace_dst_enables = 0,
        .trace_src_enables = 0
    },
};
#endif

#if defined (BUILD_MCU1_0)
const struct tisci_local_rm_boardcfg gBoardConfigLow_rm
__attribute__(( aligned(128), section(".boardcfg_data") )) =
{
    .rm_boardcfg = {
        /* tisci_boardcfg_abi_rev */
        .rev = {
            .tisci_boardcfg_abi_maj = TISCI_BOARDCFG_RM_ABI_MAJ_VALUE,
            .tisci_boardcfg_abi_min = TISCI_BOARDCFG_RM_ABI_MIN_VALUE,
        },
        .host_cfg = {
            .subhdr = {
                .magic = TISCI_BOARDCFG_RM_HOST_CFG_MAGIC_NUM,
                .size  = sizeof(struct tisci_boardcfg_rm_host_cfg),
            },
            /* For now allowing all the atypes, QoS, orderId and priority */
            .host_cfg_entries = {
             [0] = {
                    /* Allowed atype configuration for the host ID. The host ID
                     * gets assigned a list of atypes which are allowed. atype
                     * is a 2-bit field with 3 possible values. Thus in one
                     * 8-bit word, flags are set specifying whether or not an
                     * atype value is allowed for the host ID. For each atype,
                     * the value of 01b means not allowed, 10b means allowed,
                     * and 11b and 00b are invalid/errors. These are encoded in
                     * a bitfield because there is one set of allowed atypes
                     * for every host ID.
                     */
                    .allowed_atype = 0b101010,
                    /* Allowed QoS level configuration for host ID. The host
                     * ID gets assigned a list of QoS levels which are allowed.
                     * As QoS level is a 3-bit field, there are 8 possible
                     * order-IDs. Thus in one 16-bit word, flags are set
                     * specifying whether or not the QoS level is allowed for
                     * the host ID. For each QoS level, the value of 01b means
                     * not allowed, 10b means allowed, and 11b and 00b are
                     * invalid/errors. These are encoded in a bitfield because
                     * there is one set of allowed QoS levels for every host ID.
                     */
                    .allowed_qos   = 0xAAAA,
                    /* Allowed order-ID configuration for the host ID. The host
                     * ID gets assigned a list of order-IDs which are allowed.
                     * As order-ID is a 4-bit field, there are 16 possible
                     * order-IDs. Thus in one 32-bit word, flags are set
                     * specifying whether or not the order-ID is allowed for
                     * the host ID. For each order-ID, the value of 01b means
                     * not allowed, 10b means allowed, and 11b and 00b are
                     * invalid/errors. These are encoded in a bitfield because
                     * there is one set of allowed order-IDs for every host ID.
                     */
                    .allowed_orderid = 0xAAAAAAAA,
                    /* Allowed bus priority configuration for host ID. The host
                     * ID gets assigned a list of bus priorities which are
                     * allowed. As bus priority is a 3-bit field, there are 8
                     * possible bus priorities. Thus in one 16-bit word, flags
                     * are set specifying whether or not the bus priority is
                     * allowed for the host ID. For each bus priority, the
                     * value of 01b means not allowed, 10b means allowed, and
                     * 11b and 00b are invalid/errors. These are encoded in a
                     * bitfield because there is one set of allowed bus
                     * priorities for every host ID.
                     */
                    .allowed_priority = 0xAAAA,
                    /* Allowed UDMAP channel scheduling priority configuration
                     * for host ID. The host ID gets assigned a list of UDMAP
                     * channel scheduling priorities which are allowed. As
                     * UDMAP channel scheduling priority is a 2-bit field,
                     * there are 4 possible UDMAP channel scheduling priorities.
                     * Thus in one 8-bit word, flags are set specifying whether
                     * or not UDMAP channel scheduling priority is allowed for
                     * the host ID. For each priority, the value of 01b means
                     * not allowed, 10b means allowed, and 11b and 00b are
                     * invalid/errors. These are encoded in a bitfield because
                     * there is one set of allowed UDMAP channel scheduling
                     * priorities for every host ID.
                     */
                    .allowed_sched_priority = 0xAA
                    },
                [1] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [2] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [3] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [4] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [5] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [6] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [7] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [8] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [9] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [10] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [11] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [12] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [13] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [14] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [15] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [16] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [17] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [18] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [19] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [20] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [21] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [22] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [23] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [24] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [25] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [26] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [27] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [28] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [29] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [30] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    },
                [31] = {
                        .allowed_atype = 0b101010,
                        .allowed_qos   = 0xAAAA,
                        .allowed_orderid = 0xAAAAAAAA,
                        .allowed_priority = 0xAAAA,
                        .allowed_sched_priority = 0xAA
                    }
            },
        },
        .resasg = {
            .subhdr = {
                .magic = TISCI_BOARDCFG_RM_RESASG_MAGIC_NUM,
                .size  = sizeof(struct tisci_boardcfg_rm_resasg),
            },
            .resasg_entries_size = TISCI_BOARDCFG_RM_RESASG_ENTRIES *
                    sizeof(struct tisci_boardcfg_rm_resasg_entry),
        },
    },
    .resasg_entries = {
        /* Refer to the AM65xx TRM Table 10-102. Global Event Map to make
         * sense of the following numbers.
         */
        {
            /* Main Nav UDMASS IA VINT 0 - 15 reserved for use by DMSC */
            .start_resource = 16,
            .num_resource = 240,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMASS_IA0_VINT),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* MCU Nav UDMASS IA SEVI 0 - 15 reserved for use by DMSC */
            .start_resource = 16,
            .num_resource = 4592,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMASS_IA0_SEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 32768,
            .num_resource = 512,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMASS_IA0_MEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 36864,
            .num_resource = 512,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMASS_IA0_GEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_MODSS_IA0,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_MODSS_IA0_VINT),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 20480,
            .num_resource = 1024,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_MODSS_IA0,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_MODSS_IA0_SEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_MODSS_IA1,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_MODSS_IA1_VINT),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 22528,
            .num_resource = 1024,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_MODSS_IA1,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_MODSS_IA1_SEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* MCU Nav UDMASS IA VINT 0 - 7 reserved for use by DMSC */
            .start_resource = 8,
            .num_resource = 248,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMASS_IA0_VINT),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* MCU Nav UDMASS IA SEVI 16384 - 16391 reserved for use by DMSC */
            .start_resource = 16392,
            .num_resource = 1000,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMASS_IA0_SEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 17392,
            .num_resource = 528,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMASS_IA0_SEVI),
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .start_resource = 34816,
            .num_resource = 128,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMASS_IA0_MEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 39936,
            .num_resource = 256,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMASS_IA0,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMASS_IA0_GEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 43008,
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_MCRC,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_MCRC_LEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 43136,
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_MCRC,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_MCRC_LEVI),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 49152,
            .num_resource = 1024,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_TRIGGER),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* Main Nav UDMAP tx channel 0 reserved for use by DMSC */
            .start_resource = 1,
            .num_resource = 7,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_TX_HCHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 8,
            .num_resource = 112,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_TX_CHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 120,
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_TX_ECHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* Main Nav UDMAP rx channel 0 - 1 reserved for use by DMSC */
            .start_resource = 2,
            .num_resource = 6,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_RX_HCHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 8,
            .num_resource = 142,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_RX_CHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 150,
            .num_resource = 150,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                    TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_RX_FLOW_COMMON),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                    TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_INVALID_FLOW_OES),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_UDMAP,
                    TISCI_RESASG_SUBTYPE_MAIN_NAV_UDMAP_GCFG),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 56320,
            .num_resource = 256,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMAP_TRIGGER),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMAP_TX_HCHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 2,
            .num_resource = 46,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMAP_TX_CHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMAP_RX_HCHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 2,
            .num_resource = 46,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMAP,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_UDMAP_RX_CHAN),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 48,
            .num_resource = 48,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMAP,
                    TISCI_RESASG_SUBTYPE_MCU_NAV_UDMAP_RX_FLOW_COMMON),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMAP,
                    TISCI_RESASG_SUBTYPE_MCU_NAV_UDMAP_INVALID_FLOW_OES),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_UDMAP,
                    TISCI_RESASG_SUBTYPE_MCU_NAV_UDMAP_GCFG),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 61440,
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MSMC,
                         TISCI_RESASG_SUBTYPE_MSMC_DRU),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* Ring 0 is reserved for use by DMSC */
            .start_resource = 1,
            .num_resource = 151,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_RA,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_RA_RING_UDMAP_TX),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* Ring 152 is reserved for use by DMSC */
            .start_resource = 153,
            .num_resource = 149,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_RA,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_RA_RING_UDMAP_RX),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* Rings 302 and 303 are reserved for use by DMSC */
            .start_resource = 304,
            .num_resource = 464,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_RA,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_RA_RING_GP),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MAIN_NAV_RA,
                         TISCI_RESASG_SUBTYPE_MAIN_NAV_RA_ERROR_OES),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 48,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_RA,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_RA_RING_UDMAP_TX),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 48,
            .num_resource = 48,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_RA,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_RA_RING_UDMAP_RX),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 96,
            .num_resource = 160,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_RA,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_RA_RING_GP),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 0,
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_MCU_NAV_RA,
                         TISCI_RESASG_SUBTYPE_MCU_NAV_RA_ERROR_OES),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* GIC inputs 64 - 79 reserved for use by DMSC */
            .start_resource = 80,
            .num_resource = 48,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_GIC_IRQ,
                         TISCI_RESASG_SUBTYPE_GIC_IRQ_MAIN_NAV_SET0),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 392,
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_GIC_IRQ,
                         TISCI_RESASG_SUBTYPE_GIC_IRQ_MAIN_GPIO),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 498,
            .num_resource = 6,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_GIC_IRQ,
                         TISCI_RESASG_SUBTYPE_GIC_IRQ_MAIN_NAV_SET1),
            .host_id = TISCI_HOST_ID_A53_0,
        },
        {
            .start_resource = 448,
            .num_resource = 50,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_GIC_IRQ,
                         TISCI_RESASG_SUBTYPE_GIC_IRQ_MAIN_NAV_SET1),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 544,
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_GIC_IRQ,
                         TISCI_RESASG_SUBTYPE_GIC_IRQ_COMP_EVT),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 712,
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_GIC_IRQ,
                         TISCI_RESASG_SUBTYPE_GIC_IRQ_WKUP_GPIO),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* VIM inputs 64 - 67 reserved for use by DMSC */
            .start_resource = 68,
            .num_resource = 28,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_PULSAR_C0_IRQ,
                         TISCI_RESASG_SUBTYPE_PULSAR_C0_IRQ_MCU_NAV),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 124,
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_PULSAR_C0_IRQ,
                         TISCI_RESASG_SUBTYPE_PULSAR_C0_IRQ_WKUP_GPIO),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 160,
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_PULSAR_C0_IRQ,
                         TISCI_RESASG_SUBTYPE_PULSAR_C0_IRQ_MAIN2MCU_LVL),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 224,
            .num_resource = 48,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_PULSAR_C0_IRQ,
                         TISCI_RESASG_SUBTYPE_PULSAR_C0_IRQ_MAIN2MCU_PLS),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            /* VIM inputs 64 - 67 reserved for use by DMSC */
            .start_resource = 68,
            .num_resource = 28,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_PULSAR_C1_IRQ,
                         TISCI_RESASG_SUBTYPE_PULSAR_C1_IRQ_MCU_NAV),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 124,
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_PULSAR_C1_IRQ,
                         TISCI_RESASG_SUBTYPE_PULSAR_C1_IRQ_WKUP_GPIO),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 160,
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_PULSAR_C1_IRQ,
                         TISCI_RESASG_SUBTYPE_PULSAR_C1_IRQ_MAIN2MCU_LVL),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 224,
            .num_resource = 48,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_PULSAR_C1_IRQ,
                         TISCI_RESASG_SUBTYPE_PULSAR_C1_IRQ_MAIN2MCU_PLS),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 46,
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_ICSSG0_IRQ,
                         TISCI_RESASG_SUBTYPE_ICSSG0_IRQ_MAIN_NAV),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 88,
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_ICSSG0_IRQ,
                         TISCI_RESASG_SUBTYPE_ICSSG0_IRQ_MAIN_GPIO),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 46,
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_ICSSG1_IRQ,
                         TISCI_RESASG_SUBTYPE_ICSSG1_IRQ_MAIN_NAV),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 88,
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_ICSSG1_IRQ,
                         TISCI_RESASG_SUBTYPE_ICSSG1_IRQ_MAIN_GPIO),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 46,
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_ICSSG2_IRQ,
                         TISCI_RESASG_SUBTYPE_ICSSG2_IRQ_MAIN_NAV),
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .start_resource = 88,
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE(TISCI_RESASG_TYPE_ICSSG2_IRQ,
                         TISCI_RESASG_SUBTYPE_ICSSG2_IRQ_MAIN_GPIO),
            .host_id = TISCI_HOST_ID_ALL,
        },
    },
};
#endif

#if defined (BUILD_MCU1_0)
const struct tisci_boardcfg_sec gBoardConfigLow_security
__attribute__(( aligned(128), section(".boardcfg_data") )) =
{
    /* boardcfg_abi_rev */
    .rev = {
        .tisci_boardcfg_abi_maj = TISCI_BOARDCFG_SEC_ABI_MAJ_VALUE,
        .tisci_boardcfg_abi_min = TISCI_BOARDCFG_SEC_ABI_MIN_VALUE,
    },

    /* boardcfg_proc_acl */
    .processor_acl_list = {
        .subhdr = {
            .magic = TISCI_BOARDCFG_PROC_ACL_MAGIC_NUM,
            .size = sizeof(struct tisci_boardcfg_proc_acl),
        },
        .proc_acl_entries = {0},
    },

    /* boardcfg_host_hierarchy */
    .host_hierarchy = {
        .subhdr = {
            .magic = TISCI_BOARDCFG_HOST_HIERARCHY_MAGIC_NUM,
            .size = sizeof(struct tisci_boardcfg_host_hierarchy),
        },
        .host_hierarchy_entries = {0},
    },
};
#endif
