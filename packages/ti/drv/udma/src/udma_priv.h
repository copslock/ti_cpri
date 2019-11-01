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
 *  \file udma_priv.h
 *
 *  \brief UDMA private header file.
 */

#ifndef UDMA_PRIV_H_
#define UDMA_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <ti/drv/udma/udma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if defined (UDMA_CFG_ASSERT_ENABLE)
#define Udma_assert(drvHandle, cond)                                     \
    (Udma_assertLocal((drvHandle), (bool) (cond), (const char *) # cond, \
                    (const char *) __FILE__, (int32_t) __LINE__))
#else
#define Udma_assert(drvHandle, cond)
#endif

/** \brief Macro used to specify that the thread ID is invalid. */
#define UDMA_THREAD_ID_INVALID          ((uint32_t) 0xFFFF0004U)

/** \brief Macro used to specify shift value for RX flow threshold before passing to SysFw */
#define UDMA_RFLOW_RX_SIZE_THRESH_VAL_SHIFT      ((uint32_t) 0x00000005U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* SOC APIs */
void Udma_initDrvHandle(Udma_DrvHandle drvHandle);
void UdmaRmInitPrms_init(uint32_t instId, Udma_RmInitPrms *rmInitPrms);

/* Private APIs */
const Udma_UtcInstInfo *Udma_chGetUtcInst(Udma_DrvHandle drvHandle,
                                          uint32_t utcId);

/**
 *  \brief Default RA memory fence API used for CSL-FL to perform cache ops
 *
 *  \param pVirtAddr        [IN]    The virtual memory address written to
 *  \param size             [IN]    Number of bytes to writeback
 *  \param opsType          [IN]    \ref CSL_RingAccMemoryOpsType
 */
void Udma_ringaccMemOps(void *pVirtAddr, uint32_t size, uint32_t opsType);

/*
 * RM APIs
 */
int32_t Udma_rmInit(Udma_DrvHandle drvHandle);
int32_t Udma_rmDeinit(Udma_DrvHandle drvHandle);
/* Channel RM APIs */
uint32_t Udma_rmAllocBlkCopyCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeBlkCopyCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocBlkCopyHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeBlkCopyHcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocBlkCopyUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeBlkCopyUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocTxCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeTxCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocRxCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeRxCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocTxHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeTxHcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocRxHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeRxHcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocTxUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeTxUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocRxUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle);
void Udma_rmFreeRxUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocExtCh(uint32_t preferredChNum,
                           Udma_DrvHandle drvHandle,
                           const Udma_UtcInstInfo *utcInfo);
void Udma_rmFreeExtCh(uint32_t chNum,
                      Udma_DrvHandle drvHandle,
                      const Udma_UtcInstInfo *utcInfo);
/* Ring RM APIs */
uint16_t Udma_rmAllocFreeRing(Udma_DrvHandle drvHandle);
void Udma_rmFreeFreeRing(uint16_t ringNum, Udma_DrvHandle drvHandle);
uint16_t Udma_rmAllocRingMon(Udma_DrvHandle drvHandle);
void Udma_rmFreeRingMon(uint16_t ringMonNum, Udma_DrvHandle drvHandle);
/* Proxy RM APIs */
uint16_t Udma_rmAllocProxy(Udma_DrvHandle drvHandle);
void Udma_rmFreeProxy(uint16_t proxyNum, Udma_DrvHandle drvHandle);
/* Event RM APIs */
uint32_t Udma_rmAllocEvent(Udma_DrvHandle drvHandle);
void Udma_rmFreeEvent(uint32_t globalEvent, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocVintr(Udma_DrvHandle drvHandle);
void Udma_rmFreeVintr(uint32_t vintrNum, Udma_DrvHandle drvHandle);
uint32_t Udma_rmAllocVintrBit(Udma_EventHandle eventHandle);
void Udma_rmFreeVintrBit(uint32_t vintrBitNum,
                         Udma_DrvHandle drvHandle,
                         Udma_EventHandle eventHandle);
uint32_t Udma_rmAllocCoreIntr(uint32_t preferredCoreIntrNum,
                              Udma_DrvHandle drvHandle);
void Udma_rmFreeCoreIntr(uint32_t coreIntrNum, Udma_DrvHandle drvHandle);

/* Utils APIs */
uint64_t Udma_virtToPhyFxn(const void *virtAddr,
                           Udma_DrvHandle drvHandle,
                           Udma_ChHandle chHandle);
void *Udma_phyToVirtFxn(uint64_t phyAddr,
                        Udma_DrvHandle drvHandle,
                        Udma_ChHandle chHandle);
/**
 *  \brief Prints to Shared memory and CCS console
 *
 *  This function prints the provided formatted string to shared memory and CCS
 *  console
 *
 *  \param drvHandle    [IN] Driver handle
 *  \param format       [IN] Formatted string followed by variable arguments
 */
void Udma_printf(Udma_DrvHandle drvHandle, const char *format, ...);

#if defined (UDMA_CFG_ASSERT_ENABLE)
static inline void Udma_assertLocal(Udma_DrvHandle drvHandle,
                                    bool           condition,
                                    const char    *str,
                                    const char    *fileName,
                                    int32_t        lineNum);
#endif  /* if defined (UDMA_CFG_ASSERT_ENABLE) */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

#if defined (UDMA_CFG_ASSERT_ENABLE)
static inline void Udma_assertLocal(Udma_DrvHandle drvHandle,
                                    bool           condition,
                                    const char    *str,
                                    const char    *fileName,
                                    int32_t        lineNum)
{
    if(!(condition))
    {
        Udma_printf(drvHandle,
                       "Assertion @ Line: %d in %s: %s : failed !!!\n",
                       lineNum, fileName, str);
    }

    return;
}
#endif  /* if defined (UDMA_CFG_ASSERT_ENABLE) */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_PRIV_H_ */
