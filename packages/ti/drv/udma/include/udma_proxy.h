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
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_PROXY_MODULE UDMA Driver Proxy API
 *            This is UDMA driver proxy related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file udma_proxy.h
 *
 *  \brief UDMA proxy related parameters and API.
 */

#ifndef UDMA_PROXY_H_
#define UDMA_PROXY_H_

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
 * \brief Macro used to specify that proxy ID is invalid.
 */
#define UDMA_PROXY_INVALID              ((uint16_t) 0xFFFFU)
/**
 * \brief Macro used to specify any available free proxy while requesting
 * one. Used in the API #Udma_proxyAlloc.
 */
#define UDMA_PROXY_ANY                  ((uint16_t) 0xFFFEU)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief This structure contains configuration parameters for each proxy
 *  thread
 */
typedef struct
{
    uint32_t        proxyMode;
    /**< Initial queue access mode (see CSL_ProxyQueueAccessMode) */
    uint32_t        elemSize;
    /**< Ring element size.
     *   Refer \ref Udma_RingElemSize for supported values. */
    uint16_t        ringNum;
    /**< Ring number in the target to use for the proxy thread */
} Udma_ProxyCfg;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA proxy allocation API.
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *  \param proxyHandle  [IN/OUT] UDMA proxy handle. The caller need to
 *                           allocate memory for this object and pass this
 *                           pointer to all further APIs. The caller should
 *                           not change any parameters as this is owned and
 *                           maintained by the driver.
 *  \param proxyNum     [IN] Proxy number. If set to #UDMA_PROXY_ANY, will
 *                           allocate from free proxy pool. Else will try to
 *                           allocate the mentioned proxy itself.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_proxyAlloc(Udma_DrvHandle drvHandle,
                        Udma_ProxyHandle proxyHandle,
                        uint16_t proxyNum);

/**
 *  \brief UDMA free proxy.
 *
 *  Freeup the proxy resources.
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param proxyHandle  [IN] UDMA proxy handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_proxyFree(Udma_ProxyHandle proxyHandle);

/**
 *  \brief UDMA proxy config API.
 *
 *  This API should be called before performing any proxy operation.
 *  This same API can be used to reconfigure the proxy if the ring number
 *  changes or the ring (queue/dequeue) operation changes.
 *  If there is no change, the proxy queue/dequeue API can be called without
 *  the need to reconfigure the proxy (Thus saving CPU cycles)
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param proxyHandle  [IN] UDMA proxy handle.
 *                           This parameter can't be NULL.
 *  \param proxyCfg     [IN] UDMA proxy configuration.
 *                           This parameter can't be NULL.
 *                           For queue operation the mode parameter
 *                           should be set to CSL_PROXY_QUEUE_ACCESS_MODE_TAIL
 *                           For dequeue operation the mode parameter
 *                           should be set to CSL_PROXY_QUEUE_ACCESS_MODE_HEAD
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_proxyConfig(Udma_ProxyHandle proxyHandle,
                         const Udma_ProxyCfg *proxyCfg);

/**
 *  \brief UDMA queue descriptor to a proxy which is pre-configured to
 *  queue to a ring.
 *
 *  Caution: This API doesn't do any error check for performance reasons
 *  The user should ensure that the proxy is configured to perform a dequeue
 *  or queue operation to a ring.
 *  Caution: User should also ensure that the corresponding ring is not full.
 *  Performing a proxy queue operation to an already full ring will result in
 *  ring error (overflow).
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param proxyHandle  [IN] UDMA proxy handle.
 *                           This parameter can't be NULL.
 *  \param phyDescMem   [IN] Descriptor memory physical pointer to push to the
 *                           ring.
 */
static inline void Udma_proxyQueue(Udma_ProxyHandle proxyHandle,
                                   uint64_t phyDescMem);

/**
 *  \brief UDMA dequeue descriptor from a proxy which is pre-configured to
 *  dequeue from a ring.
 *
 *  Caution: This API doesn't do any error check for performance reasons
 *  The user should ensure that the proxy is configured to perform a dequeue
 *  or queue operation to a ring.
 *  Caution: User should also ensure that the corresponding ring is not empty.
 *  Performing a proxy dequeue operation to an empty ring will result in
 *  ring error (underflow).
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param proxyHandle  [IN] UDMA proxy handle.
 *                           This parameter can't be NULL.
 *  \param phyDescMem   [OUT] Descriptor memory physical pointer read from the
 *                            ring. This will be NULL if there is
 *                            nothing to pop from the ring.
 */
static inline void Udma_proxyDequeue(Udma_ProxyHandle proxyHandle,
                                     uint64_t *phyDescMem);

/**
 *  \brief API to write 64-bit data to proxy
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param proxyAddr    [IN] Proxy data address (pre-calculated for a 64-bit
 *                           write).
 *  \param data         [IN] 64-bit data to write.
 */
static inline void Udma_proxyWrite64(uintptr_t proxyAddr, uint64_t data);

/**
 *  \brief API to read 64-bit data from proxy
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param proxyAddr    [IN] Proxy data address (pre-calculated for a 64-bit
 *                           read).
 *  \param data         [OUT] Pointer to 64-bit word where the data is
 *                            read to.
 */
static inline void Udma_proxyRead64(uintptr_t proxyAddr, uint64_t *data);

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief UDMA proxy object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
struct Udma_ProxyObj
{
    Udma_DrvHandle              drvHandle;
    /**< Pointer to global driver handle. */
    uint16_t                    proxyNum;
    /**< Proxy number */

    /* Proxy address for the ring operation. Calculated at config time to reduce
     * cycles at runtime */
    uintptr_t                   proxyAddr;
    /**< Proxy address for push/pop ring operation through proxy */

    uint32_t                    proxyInitDone;
    /**< Flag to set the proxy object is init. */
};

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void Udma_proxyWrite64(uintptr_t proxyAddr, uint64_t data)
{
#if defined (__aarch64__) || defined (__C7100__)
    CSL_REG64_WR((uint64_t *) proxyAddr, data);
#else
    /* For 32-bit cores enforce the order as the compiler may not guarantee
     * the order */
    volatile uint32_t  *proxyAddr32 = (volatile uint32_t *) proxyAddr;
    uint32_t            wordLow, wordHigh;

    /* Write low word first and then the high word which triggers the actual
     *  write from proxy */
    wordLow = (uint32_t) data;
    wordHigh = (uint32_t) (data >> 32U);
    CSL_REG32_WR(proxyAddr32, wordLow);
    CSL_REG32_WR(proxyAddr32 + 1U, wordHigh);
#endif

    return;
}

static inline void Udma_proxyRead64(uintptr_t proxyAddr, uint64_t *data)
{
#if defined (__aarch64__) || defined (__C7100__)
    *data = CSL_REG64_RD((uint64_t *) proxyAddr);
#else
    /* For 32-bit cores enforce the order as the compiler may not guarantee
     * the order */
    volatile uint32_t  *proxyAddr32 = (volatile uint32_t *) proxyAddr;
    uint32_t            wordLow, wordHigh;

    /* Read low word first (this triggers the entire proxy read) and then the
     * high word which ends the proxy read */
    wordLow = CSL_REG32_RD(proxyAddr32);
    wordHigh = CSL_REG32_RD(proxyAddr32 + 1U);
    *data = ((uint64_t) wordLow) | (((uint64_t) wordHigh) << 32U);
#endif

    return;
}

static inline void Udma_proxyQueue(Udma_ProxyHandle proxyHandle,
                                   uint64_t phyDescMem)
{
    Udma_proxyWrite64(proxyHandle->proxyAddr, phyDescMem);
}

static inline void Udma_proxyDequeue(Udma_ProxyHandle proxyHandle,
                                     uint64_t *phyDescMem)
{
    Udma_proxyRead64(proxyHandle->proxyAddr, phyDescMem);
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_PROXY_H_ */

/* @} */
