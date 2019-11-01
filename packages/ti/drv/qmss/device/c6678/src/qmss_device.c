/**
 *   @file  c6678/src/qmss_device.c
 *
 *   @brief   
 *      This file contains the device specific configuration and initialization routines
 *      for QMSS Low Level Driver.
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

/* QMSS Types includes */
#include <stdint.h>
#include <stdlib.h>

/* QMSS includes */
#include <ti/drv/qmss/qmss_qm.h>
#include <ti/drv/qmss/qmss_osal.h>

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_qm_config.h>
#include <ti/csl/cslr_qm_descriptor_region_config.h>
#include <ti/csl/cslr_qm_queue_management.h>
#include <ti/csl/cslr_qm_queue_status_config.h>
#include <ti/csl/cslr_qm_intd.h>
#include <ti/csl/cslr_pdsp.h>
#include <ti/csl/csl_qm_queue.h>

/** @addtogroup QMSS_LLD_DATASTRUCT
@{ 
*/

Qmss_QueueNumRange GlobalQmGroup0[] =
{
    /** Base queue number and Maximum supported low priority queues */
    {
        Qmss_QueueType_LOW_PRIORITY_QUEUE,
        QMSS_LOW_PRIORITY_QUEUE_BASE, 
        QMSS_MAX_LOW_PRIORITY_QUEUE,
        "LOW_PRIORITY_QUEUE-qm1"       
    },
        
    /** Base queue number and Maximum supported PASS 0 queues */
    {
        Qmss_QueueType_PASS_QUEUE,
        QMSS_PASS_QUEUE_BASE,
        QMSS_MAX_PASS_QUEUE,
        "PASS_QUEUE-qm1" 
    },
        
    /** Base queue number and Maximum supported Intc Pend queues in set 1 */
    {
        Qmss_QueueType_INTC_QUEUE,
        QMSS_INTC_QUEUE_BASE,
        QMSS_MAX_INTC_QUEUE,
        "INTC_QUEUE-qm1" 
    },

    /** Base queue number and Maximum supported SRIO queues */
    {
        Qmss_QueueType_SRIO_QUEUE, 
        QMSS_SRIO_QUEUE_BASE, 
        QMSS_MAX_SRIO_QUEUE,
        "SRIO_QUEUE-qm1" 
    },
        
    /** Base queue number and Maximum supported high priority queues */
    {
        Qmss_QueueType_HIGH_PRIORITY_QUEUE,
        QMSS_HIGH_PRIORITY_QUEUE_BASE,
        QMSS_MAX_HIGH_PRIORITY_QUEUE,
        "HIGH_PRIORITY_QUEUE-qm1" 
    },
        
    /** Base queue number and Maximum supported starvation counter queues */
    {
        Qmss_QueueType_STARVATION_COUNTER_QUEUE,
        QMSS_STARVATION_COUNTER_QUEUE_BASE,
        QMSS_MAX_STARVATION_COUNTER_QUEUE,
        "STARVATION_COUNTER_QUEUE-qm1" 
    },
        
    /** Base queue number and Maximum supported infrastructure queues on first QM */
    {
        Qmss_QueueType_INFRASTRUCTURE_QUEUE,
        QMSS_INFRASTRUCTURE_QUEUE_BASE,
        QMSS_MAX_INFRASTRUCTURE_QUEUE,
        "INFRASTRUCTURE_QUEUE-qm1" 
    },
        
    /** Base queue number and Maximum supported traffic shaping queues */
    {
        Qmss_QueueType_TRAFFIC_SHAPING_QUEUE,
        QMSS_TRAFFIC_SHAPING_QUEUE_BASE,
        QMSS_MAX_TRAFFIC_SHAPING_QUEUE,
        "TRAFFIC_SHAPING_QUEUE-qm1" 
    },
       
    /** Base queue number and Maximum supported general purpose queues */
    {
        Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
        QMSS_GENERAL_PURPOSE_QUEUE_BASE, 
        QMSS_MAX_GENERAL_PURPOSE_QUEUE,
        "GENERAL_PURPOSE_QUEUE-qm1" 
    }
}; /* GlobalQmGroup0 */


/** @brief QMSS LLD initialization parameters */
Qmss_GlobalConfigParams qmssGblCfgParams =
{
    /** Maximum number of queue manager groups */
    1u,
    /** Maximum number of queue Managers */
    2u,
    /** Maximum number of queues */
    8192u,
    /** Maximum number of descriptor regions */
    20u,
    /** Maximum number of PDSP */
    2u,
    /** Size of internal linkram */
    16384u,
    /** Requires ordered memory regions */
    1,
    {
        /** queue types for group 0 */
        sizeof(GlobalQmGroup0)/sizeof(Qmss_QueueNumRange),
        /** queue types for group 1 */
        0
    },
    {   /* numQueueNum[] */
        /** Queue groups for group 1 of global QM */
        GlobalQmGroup0,
        /** No second group on global QM on this device */
        NULL
    },
    { 
        { /* Registers for first QM */
            /** QM Global Config registers */
            (CSL_Qm_configRegs *) CSL_QM_SS_CFG_CONFIG_STARVATION_COUNTER_REGS,
            /** QM Descriptor Config registers */
            (CSL_Qm_descriptor_region_configRegs *) CSL_QM_SS_CFG_DESCRIPTION_REGS, 
            /** QM queue Management registers */
            (CSL_Qm_queue_managementRegs *) CSL_QM_SS_CFG_QM_QUEUE_DEQUEUE_REGS,
            /** QM queue Management Proxy registers */
            (CSL_Qm_queue_managementRegs *) CSL_QM_SS_CFG_PROXY_QUEUE_DEQUEUE_REGS,
            /** QM queue status registers */
            (CSL_Qm_queue_status_configRegs *) CSL_QM_SS_CFG_QUE_PEEK_REGS,
            /** QM Status RAM */
            (CSL_Qm_Queue_Status *) CSL_QM_SS_CFG_QM_STATUS_RAM_REGS,
            /** QM queue Management registers, accessed via DMA port */
            (CSL_Qm_queue_managementRegs *) CSL_QM_SS_DATA_QM_QUEUE_DEQUEUE_REGS,
            /** QM queue Management Proxy registers, accessed via DMA port */
            (CSL_Qm_queue_managementRegs *) CSL_QM_SS_DATA_PROXY_QUEUE_DEQUEUE_REGS
        },
        { /* Registers for second QM */
            /** QM Global Config registers */
            (CSL_Qm_configRegs *) NULL,
            /** QM Descriptor Config registers */
            (CSL_Qm_descriptor_region_configRegs *) NULL, 
            /** QM queue Management registers */
            (CSL_Qm_queue_managementRegs *) NULL,
            /** QM queue Management Proxy registers */
            (CSL_Qm_queue_managementRegs *) NULL,
            /** QM queue status registers */
            (CSL_Qm_queue_status_configRegs *) NULL,
            /** QM Status RAM */
            (CSL_Qm_Queue_Status *) NULL,
            /** QM queue Management registers, accessed via DMA port */
            (CSL_Qm_queue_managementRegs *) NULL,
            /** QM queue Management Proxy registers, accessed via DMA port */
            (CSL_Qm_queue_managementRegs *) NULL
        }
    },
    { /* QMSS global registers */
        /** QM INTD N registers */
        {
            /** QM INTD 1 registers */
            (CSL_Qm_intdRegs *) CSL_QM_SS_CFG_INTD_REGS,
            /** QM INTD 2 registers */
            (CSL_Qm_intdRegs *) NULL
        },
        /** QM PDSP N command register */
        {
            /** QM PDSP 1 command register */
            (volatile uint32_t *) CSL_QM_SS_CFG_SCRACH_RAM1_REGS,
            /** QM PDSP 2 command register */
            (volatile uint32_t *) CSL_QM_SS_CFG_SCRACH_RAM2_REGS,
            /** QM PDSP 3 command register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 4 command register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 5 command register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 6 command register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 7 command register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 8 command register */
            (volatile uint32_t *) NULL
        },
        /** QM PDSP N control register */
        {
            /** QM PDSP 1 control register */
            (CSL_PdspRegs *) CSL_QM_SS_CFG_ADSP1_REGS,
            /** QM PDSP 2 control register */
            (CSL_PdspRegs *) CSL_QM_SS_CFG_ADSP2_REGS,
            /** QM PDSP 3 control register */
            (CSL_PdspRegs *) NULL,
            /** QM PDSP 4 control register */
            (CSL_PdspRegs *) NULL,
            /** QM PDSP 5 control register */
            (CSL_PdspRegs *) NULL,
            /** QM PDSP 6 control register */
            (CSL_PdspRegs *) NULL,
            /** QM PDSP 7 control register */
            (CSL_PdspRegs *) NULL,
            /** QM PDSP 8 control register */
            (CSL_PdspRegs *) NULL
        },
        /** QM PDSP N IRAM register */
        {
            /** QM PDSP 1 IRAM register */
            (volatile uint32_t *) CSL_QM_SS_CFG_APDSP1_RAM_REGS,
            /** QM PDSP 2 IRAM register */
            (volatile uint32_t *) CSL_QM_SS_CFG_APDSP2_RAM_REGS,
            /** QM PDSP 3 IRAM register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 4 IRAM register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 5 IRAM register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 6 IRAM register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 7 IRAM register */
            (volatile uint32_t *) NULL,
            /** QM PDSP 8 IRAM register */
            (volatile uint32_t *) NULL
        },
        /** QM Linking RAM register */
        (volatile uint32_t *) CSL_QM_SS_CFG_LINKING_RAM_REGS,
        /** QM peripheral base address used to calculate internal addresses */
        (void *) CSL_QM_SS_CFG_QUE_PEEK_REGS
    },
#ifdef _VIRTUAL_ADDR_SUPPORT
    /** virt2Phy */
    Osal_qmssVirtToPhy,
    /** phy2Virt */
    Osal_qmssPhyToVirt,
    /** virt2PhyDesc */
    Osal_qmssConvertDescVirtToPhy,
    /** phy2VirtDesc */
    Osal_qmssConvertDescPhyToVirt,
#else
    /** virt2Phy */
    NULL,
    /** phy2Virt */
    NULL,
    /** virt2PhyDesc */
    NULL,
    /** phy2VirtDesc */
    NULL,
#endif
    /** Resource Manager Handle should be NULL by default */
    (Qmss_RmServiceHnd)NULL,
    { 
        { /* Resource names for first QM */
            /** RM DTS resource name for first QM overall QM control */
            "control-qm1",
            /** RM DTS resource name for first QM allowing Link RAM configurability */
            "linkram-control-qm1",
            /** RM DTS resource name for first QM Link RAM indices (internal/region 0) */
            "linkram-int-qm1",
            /** RM DTS resource name for first QM Link RAM indices (external/region 1) */
            "linkram-ext-qm1",
            /** RM DTS resource name for first QM Memory regions */
            "memory-regions-qm1",
        },
        { /* Resource names for second QM */
            /** RM DTS resource name for second QM overall QM control */
            "control-qm2",         
            /** RM DTS resource name for second QM allowing Link RAM configurability */
            "linkram-control-qm2",
            /** RM DTS resource name for second QM Link RAM indices (internal/region 0) */
            "linkram-int-qm2",
            /** RM DTS resource name for second QM Link RAM indices (external/region 1) */
            "linkram-ext-qm2",
            /** RM DTS resource name for second QM Memory regions */
            "memory-regions-qm2",
        }
    },    
    /** RM DTS resource name for PDSP Firmware download */
    "firmware-pdsp",
    {
        /** RM DST resource name for accumulator channels on accumulator 0 (INTD0) */
        "accumulator0-ch",
        /** RM DST resource name for accumulator channels on accumulator 1 (INTD1) */
        "accumulator1-ch"
    },
    /** RM nameserver pattern for per region queues */
    "qm-reg-q-ss%d-gr%d-r%d",
    /** Map of PDSP # to its associated intd (which defines which set of logical channels/interrupts are used */
    {
        0, 0, 0, 0, 0, 0, 0, 0
    }
};

/**
@}
*/

