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
 *  \defgroup DRV_UDMA_RM_MODULE UDMA Driver RM API
 *            This is UDMA driver resource manager related configuration
 *            parameters and API
 *
 *  @{
 */

/**
 *  \file udma_rm.h
 *
 *  \brief UDMA RM related parameters and API.
 */

#ifndef UDMA_RM_H_
#define UDMA_RM_H_

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

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA resource manager init parameters.
 *
 *  This assumes contiguos allocation of 'N' resources from a start offset
 *  to keep the interface simple.
 *
 *  Note: This is applicable for the driver handle as given during init call.
 *  The init call doesn't (can't rather) check for resource overlap across
 *  handles and across cores. It is the callers responsibility to ensure that
 *  resources overlaps are not present.
 */
typedef struct
{
    uint32_t                startBlkCopyUhcCh;
    /**< Start ultra high capacity block copy channel from which this UDMA
     *   driver instance manages */
    uint32_t                numBlkCopyUhcCh;
    /**< Number of ultra high capacity block copy channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_BLK_COPY_UHC_CH */
    uint32_t                startBlkCopyHcCh;
    /**< Start high capacity block copy channel from which this UDMA
     *   driver instance manages */
    uint32_t                numBlkCopyHcCh;
    /**< Number of ultra high capacity block copy channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_BLK_COPY_HC_CH */
    uint32_t                startBlkCopyCh;
    /**< Start Block copy channel from which this UDMA driver instance manages */
    uint32_t                numBlkCopyCh;
    /**< Number of Block copy channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_BLK_COPY_CH */

    uint32_t                startTxUhcCh;
    /**< Start ultra high capacity TX channel from which this UDMA driver
     *   instance manages */
    uint32_t                numTxUhcCh;
    /**< Number of ultra high capacity TX channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_TX_UHC_CH */
    uint32_t                startTxHcCh;
    /**< Start high capacity TX channel from which this UDMA driver instance
     *   manages */
    uint32_t                numTxHcCh;
    /**< Number of high capacity TX channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_TX_HC_CH */
    uint32_t                startTxCh;
    /**< Start TX channel from which this UDMA driver instance manages */
    uint32_t                numTxCh;
    /**< Number of TX channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_TX_CH */

    uint32_t                startRxUhcCh;
    /**< Start ultra high capacity RX channel from which this UDMA driver
     *   instance manages */
    uint32_t                numRxUhcCh;
    /**< Number of high capacity RX channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_RX_UHC_CH */
    uint32_t                startRxHcCh;
    /**< Start high capacity RX channel from which this UDMA driver instance
     *   manages */
    uint32_t                numRxHcCh;
    /**< Number of high capacity RX channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_RX_HC_CH */
    uint32_t                startRxCh;
    /**< Start RX channel from which this UDMA driver instance manages */
    uint32_t                numRxCh;
    /**< Number of RX channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_RX_CH */

    uint32_t                startUtcCh[UDMA_NUM_UTC_INSTANCE];
    /**< Start External UTC channel from which this UDMA driver instance
     *   manages */
    uint32_t                numUtcCh[UDMA_NUM_UTC_INSTANCE];
    /**< Number of External UTC channel to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_UTC_CH_PER_INST */

    uint32_t                startFreeFlow;
    /**< Start free flow from which this UDMA driver instance manages */
    uint32_t                numFreeFlow;
    /**< Number of free flow to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_FREE_FLOW */
    uint32_t                startFreeRing;
    /**< Start free ring from which this UDMA driver instance manages */
    uint32_t                numFreeRing;
    /**< Number of free ring to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_FREE_RING */

    uint32_t                startGlobalEvent;
    /**< Start global event from which this UDMA driver instance manages */
    uint32_t                numGlobalEvent;
    /**< Number of global event to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_GLOBAL_EVENT */
    uint32_t                startVintr;
    /**< Start VINT number from which this UDMA driver instance manages */
    uint32_t                numVintr;
    /**< Number of VINT to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_VINTR */
    uint32_t                startIrIntr;
    /**< Start core interrupt from which this UDMA driver instance manages.
     *
     *   Note: Incase of C7x, this represents the GIC SPI events to the CLEC.
     *   For routing this event, the driver further uses the startC7xCoreIntr
     *   parameter as the start C7x interrupt and assumes that numIrIntr
     *   C7x interrupt are used by UDMA driver for one to one mapping.
     *   The UDMA driver directly programs the CLEC for this routing
     *
     *   Example: startIrIntr = 700, numIrIntr = 3, startC7xCoreIntr = 32
     *
     *   First Event registration:
     *   CLEC input         : 700+1024-32
     *   CLEC output        : 32
     *   OSAL registration  : 32
     *
     *   Second Event registration:
     *   CLEC input         : 701+1024-32
     *   CLEC output        : 33
     *   OSAL registration  : 33
     */
    uint32_t                numIrIntr;
    /**< Number of core interrupts to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_CORE_INTR */

    uint32_t                proxyThreadNum;
    /**< Proxy thread to push/pop to ring in proxy mode.
     *   By default driver will initialize to a default value based on
     *   core and NAVSS instance. User can override this based on need.
     *   The default proxy allocation starts from #UDMA_DEFAULT_RM_PROXY_THREAD_START
     *   and will allocate 1 per core. So total allocation will be from
     *   #UDMA_DEFAULT_RM_PROXY_THREAD_START to
     *   (#UDMA_DEFAULT_RM_PROXY_THREAD_START + num cores) in an SOC.
     *
     *   The proxy thread number should be allocated within a NAVSS instance
     *   as a proxy can access ring only within the same NAVSS instance. The
     *   driver assumes the right proxy instance to use based on the
     *   instance ID (instId) provided in #Udma_init API
     *
     *   Also this should be set a unique number across core and NAVSS
     *   instance. Care should be taken not to use the same proxy across
     *   the system.
     */
    uint32_t                startC7xCoreIntr;
    /**< Start C7x core interrupt from which this UDMA driver instance manages.
     *   This assumes numIrIntr contiguous interrupts from this offset is
     *   reserved for the UDMA driver.
     *   This is NA for other cores and could be set to 0.
     */
    uint32_t                startProxy;
    /**< Start proxy from which this UDMA driver instance manages.
     *   Note this should not overlap with proxyThreadNum */
    uint32_t                numProxy;
    /**< Number of proxy to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_PROXY */
    uint32_t                startRingMon;
    /**< Start monitor from which this UDMA driver instance manages */
    uint32_t                numRingMon;
    /**< Number of monitors to be managed.
     *   Note: This cannot exceed #UDMA_RM_MAX_RING_MON */
} Udma_RmInitPrms;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Returns the default RM config structure based on instance and core.
 *  User can use this API to get the default config and override as per need.
 *
 *  Note: The driver internally uses this same API to init the #Udma_RmInitPrms
 *  structure in #UdmaInitPrms_init API
 *
 *  \param instId       [IN] \ref Udma_InstanceId
 *
 *  \return Const pointer to default RM init config #Udma_RmInitPrms
 */
const Udma_RmInitPrms *Udma_rmGetDefaultCfg(uint32_t instId);

/**
 *  \brief API to check the default configuration across all instance and cores.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_rmCheckDefaultCfg(void);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_RM_H_ */

/* @} */
