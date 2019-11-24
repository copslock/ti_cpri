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
 *  \defgroup DRV_UDMA_MODULE UDMA Driver
 *
 *  @{
 */
/* @} */

/**
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_API_MODULE UDMA Driver API
 *            This is UDMA driver init, deinit and common API.
 *
 *  @{
 */

/**
 *  \file udma.h
 *
 *  \brief UDMA Driver API/interface file.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2494)
 */

#ifndef UDMA_H_
#define UDMA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#include <ti/csl/soc.h>
#include <ti/csl/csl_cppi.h>
#include <ti/csl/csl_psilcfg.h>
#include <ti/csl/csl_ringacc.h>
#include <ti/csl/csl_udmap.h>
#include <ti/csl/csl_intaggr.h>
#include <ti/csl/csl_intr_router.h>
#include <ti/csl/csl_dru.h>
#include <ti/csl/csl_proxy.h>
#include <ti/csl/csl_clec.h>

#include <ti/osal/osal.h>
#include <ti/drv/sciclient/sciclient.h>

#include <ti/drv/udma/include/udma_cfg.h>
#include <ti/drv/udma/include/udma_types.h>
#include <ti/drv/udma/soc/udma_soc.h>
#include <ti/drv/udma/include/udma_osal.h>
#include <ti/drv/udma/include/udma_ring.h>
#include <ti/drv/udma/include/udma_proxy.h>
#include <ti/drv/udma/include/udma_flow.h>
#include <ti/drv/udma/include/udma_event.h>
#include <ti/drv/udma/include/udma_rm.h>
#include <ti/drv/udma/include/udma_ch.h>
#include <ti/drv/udma/include/udma_dru.h>
#include <ti/drv/udma/include/udma_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief UDMA Virtual to Physical address translation callback function.
 *
 *  This function is used by the driver to convert virtual address to physical
 *  address.
 *
 *  \param virtAddr [IN] Virtual address
 *  \param chNum    [IN] Channel number passed during channel open
 *  \param appData  [IN] Callback pointer passed during channel open
 *
 *  \return Corresponding physical address
 */
typedef uint64_t (*Udma_VirtToPhyFxn)(const void *virtAddr,
                                      uint32_t chNum,
                                      void *appData);
/**
 *  \brief UDMA Physical to Virtual address translation callback function.
 *
 *  This function is used by the driver to convert physical address to virtual
 *  address.
 *
 *  \param phyAddr  [IN] Physical address
 *  \param chNum    [IN] Channel number passed during channel open
 *  \param appData  [IN] Callback pointer passed during channel open
 *
 *  \return Corresponding virtual address
 */
typedef void *(*Udma_PhyToVirtFxn)(uint64_t phyAddr,
                                   uint32_t chNum,
                                   void *appData);

/**
 *  \brief UDMA info/debug print function prototype.
 *
 *  This function is used by the driver to print info/debug messages.
 *
 *  \param str      [OUT] Info string to print.
 */
typedef void (*Udma_PrintFxn)(const char *str);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA initialization parameters.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2631)
 */
typedef struct
{
    uint32_t                instId;
    /**< [IN] \ref Udma_InstanceId */

    Udma_RmInitPrms         rmInitPrms;
    /**< RM init parameters */
    uint32_t                skipRmOverlapCheck;
    /**< Skips the resource overlap check - useful when running from pre-silicon
     *   environment as well as in production code when the resource overlap
     *   is already checked */
    uint32_t                skipGlobalEventReg;
    /**< Skips the global event registeration for the handle. By default this
     *   is set to FALSE and application can use this common handle to set the
     *   master event to limit the number of IA/IR registration per core
     *   This can be set to TRUE to skip this registration as in the case
     *   of having multiple handles per core in usecases */
    Udma_VirtToPhyFxn       virtToPhyFxn;
    /**< If not NULL, this function will be called to convert virtual address
     *   to physical address to be provided to UDMA.
     *   If NULL, the driver will assume a one-one mapping.
     *
     *   Note: The init fxn will initialize this to the default one-one map
     *   function #Udma_defaultVirtToPhyFxn
     */
    Udma_PhyToVirtFxn       phyToVirtFxn;
    /**< If not NULL, this function will be called to convert physical address
     *   to virtual address to access the pointer returned by the UDMA.
     *   If NULL, the driver will assume a one-one mapping.
     *
     *   Note: The init fxn will initialize this to the default one-one map
     *   function #Udma_defaultPhyToVirtFxn
     */
    Udma_PrintFxn           printFxn;
    /**< If not NULL, this function will be called to print debug/info message
     *   with appropriate string. */

    Udma_OsalPrms           osalPrms;
    /**< OSAL callback  parameters */
} Udma_InitPrms;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA init function.
 *
 *  Initializes the UDMA drivers.
 *  This function should be called before calling any of driver API's and
 *  should be called only once.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2576)
 *
 *  \param drvHandle    [IN] UDMA driver handle - static memory needs to
 *                           allocated by caller. This is used by the driver to
 *                           maintain the driver states.
 *                           This cannot be NULL.
 *  \param initPrms     [IN] UDMA Initialization parameters.
 *                           If NULL is passed, the default parameters will be
 *                           assumed - address translation disabled.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_init(Udma_DrvHandle drvHandle, const Udma_InitPrms *initPrms);

/**
 *  \brief UDMA deinit function.
 *
 *  Uninitializes the drivers and the hardware and should be called during
 *  system shutdown. Should not be called if Udma_init() is not called.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2577)
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_deinit(Udma_DrvHandle drvHandle);

/*
 * Structure Init functions
 *
 * Requirement: DOX_REQ_TAG(PDK-2600)
 */
/**
 *  \brief Udma_InitPrms structure init function.
 *
 *  \param instId       [IN] \ref Udma_InstanceId
 *  \param initPrms     [IN] Pointer to #Udma_InitPrms structure.
 *
 */
void UdmaInitPrms_init(uint32_t instId, Udma_InitPrms *initPrms);

/* ========================================================================== */
/*      Internal Function Declarations (Needed for other static inlines)      */
/* ========================================================================== */

/**
 *  \brief Default virtual to physical translation function.
 *
 *  \param virtAddr [IN] Virtual address
 *  \param chNum    [IN] Channel number passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to #UDMA_DMA_CH_INVALID.
 *  \param appData  [IN] Callback pointer passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to NULL.
 *
 *  \return Corresponding physical address
 */
static inline uint64_t Udma_defaultVirtToPhyFxn(const void *virtAddr,
                                                uint32_t chNum,
                                                void *appData);

/**
 *  \brief Default physical to virtual translation function.
 *
 *  \param phyAddr  [IN] Physical address
 *  \param chNum    [IN] Channel number passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to #UDMA_DMA_CH_INVALID.
 *  \param appData  [IN] Callback pointer passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to NULL.
 *
 *  \return Corresponding virtual address
 */
static inline void *Udma_defaultPhyToVirtFxn(uint64_t phyAddr,
                                             uint32_t chNum,
                                             void *appData);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static uint64_t Udma_defaultVirtToPhyFxn(const void *virtAddr,
                                         uint32_t chNum,
                                         void *appData)
{
    return ((uint64_t) virtAddr);
}

static void *Udma_defaultPhyToVirtFxn(uint64_t phyAddr,
                                      uint32_t chNum,
                                      void *appData)
{
#if defined (__aarch64__)
    uint64_t temp = phyAddr;
#else
    /* R5 is 32-bit machine, need to truncate to avoid void * typecast error */
    uint32_t temp = (uint32_t) phyAddr;
#endif

    return ((void *) temp);
}

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief UDMA driver object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
struct Udma_DrvObj
{
    /*
     * NAVSS instance parameters
     */
    CSL_UdmapCfg            udmapRegs;
    /**< UDMAP register configuration */
    CSL_RingAccCfg          raRegs;
    /**< RA register configuration */
    CSL_IntaggrCfg          iaRegs;
    /**< Interrupt Aggregator configuration */
#if (UDMA_NUM_UTC_INSTANCE > 0)
    Udma_UtcInstInfo        utcInfo[UDMA_NUM_UTC_INSTANCE];
    /**< UTC instance information */
#endif
    uint32_t                udmapSrcThreadOffset;
    /**< UDMAP Source/TX thread offset */
    uint32_t                udmapDestThreadOffset;
    /**< UDMAP Dest/RX thread offset */
    uint32_t                maxRings;
    /**< Maximun number of rings present in the NAVSS instance */
    uint32_t                maxProxy;
    /**< Maximun number of proxy present in the NAVSS instance */
    uint32_t                maxRingMon;
    /**< Maximun number of ring monitors present in the NAVSS instance */

    /*
     * Proxy parameters
     */
    CSL_ProxyCfg            proxyCfg;
    /*< Proxy register configuration */
    CSL_ProxyTargetParams   proxyTargetRing;
    /*< Proxy ring target register configuration */
    uint32_t                proxyTargetNumRing;
    /*< Proxy ring target index */

    /*
     * Clec parameters
     */
    CSL_CLEC_EVTRegs       *clecRegs;
    /**< CLEC baseaddress. */
    uint32_t                clecRtMap;
    /**< Route map bit field - differs from core to core.
     *   Refer \ref CSL_ClecRouteMap. */
    uint32_t                clecOffset;
    /**< GIC SPI to CLEC interrupt offset (they are not directly connected */

    /*
     * TISCI RM parameters
     */
    uint16_t                devIdRing;
    /**< Ring RM ID */
    uint16_t                devIdUdma;
    /**< UDMA RM ID */
    uint16_t                devIdPsil;
    /**< PSIL RM ID */
    uint16_t                devIdIa;
    /**< IA RM ID */
    uint16_t                devIdCore;
    /**< Core RM ID */
    uint32_t                druCoreId;
    /**< DRU core ID register to use for direct TR submission.
     *   Each CPU should have a unique submit register to avoid corrupting
     *   submit word when SW is running from multiple CPU at the same time.
     *   Refer \ref Udma_DruSubmitCoreId */

    uint32_t                txChOffset;
    /**< TX channel offset. */
    uint32_t                extChOffset;
    /**< External channel offset. */
    uint32_t                rxChOffset;
    /**< RX channel offset. */
    uint32_t                iaGemOffset;
    /**< IA global event map offset to differentiate between main and MCU NAVSS */
    uint32_t                trigGemOffset;
    /**< UDMAP trigger global event map offset to differentiate between main
     *   and MCU NAVSS */

    struct Udma_EventObj    globalEventObj;
    /**< Object to store global event. */
    Udma_EventHandle        globalEventHandle;
    /**< Global event handle. */

    Udma_InitPrms           initPrms;
    /**< Object to store the init params. */
    uint32_t                drvInitDone;
    /**< Flag to check if the driver object is init properly or not. */

    /*
     * RM objects.
     * This is a bitwise flag
     * 1 - free, 0 - allocated
     */
    uint32_t                blkCopyChFlag[UDMA_RM_BLK_COPY_CH_ARR_SIZE];
    /**< UDMA Block copy channel allocation flag */
    uint32_t                blkCopyHcChFlag[UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE];
    /**< UDMA high capacity Block copy channel allocation flag */
    uint32_t                blkCopyUhcChFlag[UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE];
    /**< UDMA ultra high capacity Block copy channel allocation flag */

    uint32_t                txChFlag[UDMA_RM_TX_CH_ARR_SIZE];
    /**< UDMA TX channel allocation flag */
    uint32_t                txHcChFlag[UDMA_RM_TX_HC_CH_ARR_SIZE];
    /**< UDMA high capacity TX channel allocation flag */
    uint32_t                txUhcChFlag[UDMA_RM_TX_UHC_CH_ARR_SIZE];

    /**< UDMA ultra high capacity TX channel allocation flag */
    uint32_t                rxChFlag[UDMA_RM_RX_CH_ARR_SIZE];
    /**< UDMA RX channel allocation flag */
    uint32_t                rxHcChFlag[UDMA_RM_RX_HC_CH_ARR_SIZE];
    /**< UDMA high capacity RX channel allocation flag */
    uint32_t                rxUhcChFlag[UDMA_RM_RX_UHC_CH_ARR_SIZE];

#if (UDMA_NUM_UTC_INSTANCE > 0)
    /**< UDMA ultra high capacity RX channel allocation flag */
    uint32_t                utcChFlag[UDMA_NUM_UTC_INSTANCE][UDMA_RM_UTC_CH_ARR_SIZE];
#endif
    /**< UDMA external UTC channel allocation flag */
    uint32_t                freeRingFlag[UDMA_RM_FREE_RING_ARR_SIZE];
    /**< UDMA free ring allocation flag */
    uint32_t                freeFlowFlag[UDMA_RM_FREE_FLOW_ARR_SIZE];
    /**< UDMA free flow allocation flag */
    uint32_t                globalEventFlag[UDMA_RM_GLOBAL_EVENT_ARR_SIZE];
    /**< IA global event allocation flag */
    uint32_t                vintrFlag[UDMA_RM_VINTR_ARR_SIZE];
    /**< IA VINTR allocation flag */
    uint32_t                coreIntrFlag[UDMA_RM_CORE_INTR_ARR_SIZE];
    /**< Core interrupt allocation flag */
    uint32_t                proxyFlag[UDMA_RM_PROXY_ARR_SIZE];
    /**< UDMA proxy allocation flag */
    uint32_t                ringMonFlag[UDMA_RM_RING_MON_ARR_SIZE];
    /**< UDMA ring monitor allocation flag */

    void                   *rmLock;
    /**< Mutex to protect RM allocation. */
    void                   *printLock;
    /**< Mutex to protect print buffer. */
    char                    printBuf[UDMA_CFG_PRINT_BUF_LEN];
    /**< Print buffer */
};

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_H_ */

/* @} */
