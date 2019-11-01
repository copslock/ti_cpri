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
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_CFG_MODULE UDMA Driver Configuration
 *            This is UDMA driver configuration parameters
 *
 *  @{
 */

/**
 *  \file udma_cfg.h
 *
 *  \brief UDMA configuration parameters.
 */

#ifndef UDMA_CFG_H_
#define UDMA_CFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor Udma_RmMaxSize
 *  Resource management related macros.
 *
 *  These values are based on an optimal value typically used for allocation
 *  per core and not based on actual resources in a given SOC.
 *
 *  Note: Kept to be multiple of 32 to store as bit fields in uint32_t
 *  @{
 */
#define UDMA_RM_MAX_BLK_COPY_CH             (32U)
#define UDMA_RM_MAX_BLK_COPY_HC_CH          (32U)
#define UDMA_RM_MAX_BLK_COPY_UHC_CH         (32U)
#define UDMA_RM_MAX_TX_CH                   (256U)
#define UDMA_RM_MAX_TX_HC_CH                (32U)
#define UDMA_RM_MAX_TX_UHC_CH               (32U)
#define UDMA_RM_MAX_RX_CH                   (256U)
#define UDMA_RM_MAX_RX_HC_CH                (32U)
#define UDMA_RM_MAX_RX_UHC_CH               (32U)
#define UDMA_RM_MAX_UTC_CH_PER_INST         (64U)
#define UDMA_RM_MAX_FREE_RING               (1024U)
#define UDMA_RM_MAX_FREE_FLOW               (256U)
#define UDMA_RM_MAX_GLOBAL_EVENT            (1024U)
#define UDMA_RM_MAX_VINTR                   (512U)
#define UDMA_RM_MAX_CORE_INTR               (128U)
#define UDMA_RM_MAX_PROXY                   (32U)
#define UDMA_RM_MAX_RING_MON                (32U)

/* Array allocation macros */
#define UDMA_RM_BLK_COPY_CH_ARR_SIZE        (UDMA_RM_MAX_BLK_COPY_CH >> 5U)
#define UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE     (UDMA_RM_MAX_BLK_COPY_HC_CH >> 5U)
#define UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE    (UDMA_RM_MAX_BLK_COPY_UHC_CH >> 5U)
#define UDMA_RM_TX_CH_ARR_SIZE              (UDMA_RM_MAX_TX_CH >> 5U)
#define UDMA_RM_TX_HC_CH_ARR_SIZE           (UDMA_RM_MAX_TX_HC_CH >> 5U)
#define UDMA_RM_TX_UHC_CH_ARR_SIZE          (UDMA_RM_MAX_TX_UHC_CH >> 5U)
#define UDMA_RM_RX_CH_ARR_SIZE              (UDMA_RM_MAX_RX_CH >> 5U)
#define UDMA_RM_RX_HC_CH_ARR_SIZE           (UDMA_RM_MAX_RX_HC_CH >> 5U)
#define UDMA_RM_RX_UHC_CH_ARR_SIZE          (UDMA_RM_MAX_RX_UHC_CH >> 5U)
#define UDMA_RM_UTC_CH_ARR_SIZE             (UDMA_RM_MAX_UTC_CH_PER_INST >> 5U)
#define UDMA_RM_FREE_RING_ARR_SIZE          (UDMA_RM_MAX_FREE_RING >> 5U)
#define UDMA_RM_FREE_FLOW_ARR_SIZE          (UDMA_RM_MAX_FREE_FLOW >> 5U)
#define UDMA_RM_GLOBAL_EVENT_ARR_SIZE       (UDMA_RM_MAX_GLOBAL_EVENT >> 5U)
#define UDMA_RM_VINTR_ARR_SIZE              (UDMA_RM_MAX_VINTR >> 5U)
#define UDMA_RM_CORE_INTR_ARR_SIZE          (UDMA_RM_MAX_CORE_INTR >> 5U)
#define UDMA_RM_PROXY_ARR_SIZE              (UDMA_RM_MAX_PROXY >> 5U)
#define UDMA_RM_RING_MON_ARR_SIZE           (UDMA_RM_MAX_RING_MON >> 5U)
/* @} */

/** \brief Default proxy thread number to start the allocation per core */
#define UDMA_DEFAULT_RM_PROXY_THREAD_START  (4U)

/** \brief Default ring order ID */
#define UDMA_DEFAULT_RING_ORDER_ID          (0U)

/** \brief Default TX channel DMA priority */
#define UDMA_DEFAULT_TX_CH_DMA_PRIORITY                                     \
                                    (TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_MEDHIGH)
/** \brief Default RX channel DMA priority */
#define UDMA_DEFAULT_RX_CH_DMA_PRIORITY                                     \
                                    (TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_MEDHIGH)
/** \brief Default RX channel DMA priority */
#define UDMA_DEFAULT_UTC_CH_DMA_PRIORITY                                    \
                                    (TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_MEDHIGH)

/** \brief Default TX channel bus priority */
#define UDMA_DEFAULT_TX_CH_BUS_PRIORITY     (4U)
/** \brief Default RX channel bus priority */
#define UDMA_DEFAULT_RX_CH_BUS_PRIORITY     (4U)
/** \brief Default RX channel bus priority */
#define UDMA_DEFAULT_UTC_CH_BUS_PRIORITY    (4U)

/** \brief Default TX channel bus QOS */
#define UDMA_DEFAULT_TX_CH_BUS_QOS          (4U)
/** \brief Default RX channel bus QOS */
#define UDMA_DEFAULT_RX_CH_BUS_QOS          (4U)
/** \brief Default RX channel bus QOS */
#define UDMA_DEFAULT_UTC_CH_BUS_QOS         (4U)

/** \brief Default TX channel bus order ID */
#define UDMA_DEFAULT_TX_CH_BUS_ORDERID      (0U)
/** \brief Default RX channel bus order ID */
#define UDMA_DEFAULT_RX_CH_BUS_ORDERID      (0U)
/** \brief Default RX channel bus order ID */
#define UDMA_DEFAULT_UTC_CH_BUS_ORDERID     (0U)

/** \brief Default DRU queue ID */
#define UDMA_DEFAULT_UTC_DRU_QUEUE_ID       (CSL_DRU_QUEUE_ID_3)

/** \brief UDMA print buffer length */
#define UDMA_CFG_PRINT_BUF_LEN              ((uint32_t) 1024U)

/** \brief Default UDMA channel disable timeout */
#define UDMA_DEFAULT_CH_DISABLE_TIMEOUT     (100U)

/** \brief SCICLIENT API timeout */
#define UDMA_SCICLIENT_TIMEOUT              (SCICLIENT_SERVICE_WAIT_FOREVER)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_CFG_H_ */

/* @} */
