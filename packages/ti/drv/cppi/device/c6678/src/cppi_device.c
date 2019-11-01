/**
 *   @file  c6678/src/cppi_device.c
 *
 *   @brief   
 *      This file contains the device specific configuration and initialization routines
 *      for CPPI Low Level Driver.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2015, Texas Instruments, Inc.
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
 *  \par
*/

/* CPPI Types includes */
#include <stdint.h>
#include <stdlib.h>

/* CPPI includes */
#include <ti/drv/cppi/cppi_drv.h>

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_cppidma_global_config.h>
#include <ti/csl/cslr_cppidma_rx_channel_config.h>
#include <ti/csl/cslr_cppidma_rx_flow_config.h>
#include <ti/csl/cslr_cppidma_tx_channel_config.h>
#include <ti/csl/cslr_cppidma_tx_scheduler_config.h>
#include <ti/csl/csl_cppi.h>

/** @addtogroup CPPI_LLD_DATASTRUCT
@{ 
*/
/** @brief CPPI LLD initialization parameters for each CPDMA */
Cppi_GlobalCPDMAConfigParams cppiGblCPDMACfgParams[CPPI_MAX_CPDMA] =
{
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_SRIO_CPDMA,
        /** Maximum supported Rx Channels */
        16u,
        /** Maximum supported Tx Channels */
        16u,
        /** Maximum supported Rx Flows */
        20u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) CSL_SRIO_CONFIG_CPPI_DMA_GLOBAL_CFG_REGS,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) CSL_SRIO_CONFIG_CPPI_DMA_TX_CFG_REGS,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) CSL_SRIO_CONFIG_CPPI_DMA_RX_CFG_REGS,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) CSL_SRIO_CONFIG_CPPI_DMA_TX_SCHEDULER_CFG_REGS,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) CSL_SRIO_CONFIG_CPPI_DMA_RX_FLOW_CFG_REGS,
        /** RM DTS resource name for CPDMA rx channels */
        "srio-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "srio-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "srio-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "srio-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_AIF_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "aif-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "aif-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "aif-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "aif-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_FFTC_A_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "fftc-a-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "fftc-a-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "fftc-a-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "fftc-a-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_FFTC_B_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "fftc-b-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "fftc-b-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "fftc-b-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "fftc-b-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_FFTC_C_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "fftc-c-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "fftc-c-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "fftc-c-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "fftc-c-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_FFTC_D_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "fftc-d-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "fftc-d-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "fftc-d-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "fftc-d-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_FFTC_E_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "fftc-e-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "fftc-e-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "fftc-e-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "fftc-e-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_FFTC_F_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "fftc-f-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "fftc-f-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "fftc-f-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "fftc-f-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_PASS_CPDMA,
        /** Maximum supported Rx Channels */
        24u,
        /** Maximum supported Tx Channels */
        9u,
        /** Maximum supported Rx Flows */
        32u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) CSL_PA_SS_CFG_CPPI_DMA_GLOBAL_CFG_REGS,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) CSL_PA_SS_CFG_CPPI_DMA_TX_CFG_REGS,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) CSL_PA_SS_CFG_CPPI_DMA_RX_CFG_REGS,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) CSL_PA_SS_CFG_CPPI_DMA_TX_SCHEDULER_CFG_REGS,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) CSL_PA_SS_CFG_CPPI_DMA_RX_FLOW_CFG_REGS,
        /** RM DTS resource name for CPDMA rx channels */
        "pass-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "pass-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "pass-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "pass-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_QMSS_CPDMA,
        /** Maximum supported Rx Channels */
        32u,
        /** Maximum supported Tx Channels */
        32u,
        /** Maximum supported Rx Flows */
        64u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) CSL_QM_SS_CFG_CPPI_DMA_GLOBAL_CFG_REGS,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) CSL_QM_SS_CFG_CPPI_DMA_TX_CFG_REGS,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) CSL_QM_SS_CFG_CPPI_DMA_RX_CFG_REGS,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) CSL_QM_SS_CFG_CPPI_DMA_TX_SCHEDULER_CFG_REGS,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) CSL_QM_SS_CFG_CPPI_DMA_RX_FLOW_CFG_REGS,
        /** RM DTS resource name for CPDMA rx channels */
        "qmss-qm1-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "qmss-qm1-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "qmss-qm1-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "qmss-qm1-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_QMSS_QM2_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "qmss-qm2-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "qmss-qm2-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "qmss-qm2-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "qmss-qm2-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_BCP_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
        /** RM DTS resource name for CPDMA rx channels */
        "bcp-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "bcp-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "bcp-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "bcp-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_XGE_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,

        /** RM DTS resource name for CPDMA rx channels */
        "xge-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "xge-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "xge-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "xge-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_NETCP_LOCAL_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
  
        /** RM DTS resource name for CPDMA rx channels */
        "netcp-local-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "netcp-local-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "netcp-local-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "netcp-local-hw-open"
    },
    {
        /** CPDMA this configuration belongs to */
        Cppi_CpDma_IQN_CPDMA,
        /** Maximum supported Rx Channels */
        0u,
        /** Maximum supported Tx Channels */
        0u,
        /** Maximum supported Rx Flows */
        0u,
        /** Priority for all Rx transactions of this CPDMA */
        0u,
        /** Priority for all Tx transactions of this CPDMA */
        0u,

        /** Base address for the CPDMA overlay registers */

        /** Global Config registers */
        (CSL_Cppidma_global_configRegs *) NULL,
        /** Tx Channel Config registers */
        (CSL_Cppidma_tx_channel_configRegs *) NULL,
        /** Rx Channel Config registers */
        (CSL_Cppidma_rx_channel_configRegs *) NULL,
        /** Tx Channel Scheduler registers */
        (CSL_Cppidma_tx_scheduler_configRegs *) NULL,
        /** Rx Flow Config registers */
        (CSL_Cppidma_rx_flow_configRegs *) NULL,
  
        /** RM DTS resource name for CPDMA rx channels */
        "iqn-rx-ch",
        /** RM DTS resource name for CPDMA tx channels */
        "iqn-tx-ch",
        /** RM DTS resource name for CPDMA rx flows */
        "iqn-rx-flow-id",
        /** RM DTS resource name for register writes in @ref Cppi_open */
        "iqn-hw-open"
    }
};

/** @brief CPPI LLD initialization parameters for system */
Cppi_GlobalConfigParams cppiGblCfgParams =
{
    /** Configurations of each CPDMA */
    cppiGblCPDMACfgParams,
    /** Base address for first 4K queues */
    CSL_QM_SS_DATA_QM_QUEUE_DEQUEUE_REGS,
    /** Base address for second 4K queues */
    CSL_QM_SS_DATA_QM_QUEUE_DEQUEUE_REGS + 0x10000,
    /** Base address for third 4K queues */
    0,
    /** Base address for fourth 4K queues */
    0
};

/**
@}
*/

