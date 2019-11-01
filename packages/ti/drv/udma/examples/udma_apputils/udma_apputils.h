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
 *  \file udma_apputils.h
 *
 *  \brief Common UDMA application utility used in all UDMA example.
 *
 *  NOTE: This library is meant only for UDMA examples. Customers are not
 *  encouraged to use this layer as these are very specific to the examples
 *  written and the API behaviour and signature can change at any time to
 *  suit the examples.
 *
 */

#ifndef UDMA_APPUTILS_H_
#define UDMA_APPUTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

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

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Perform cache WB operation based on whether cache is
 *  coherent or not
 *
 *  This internally uses the OSAL API.
 *
 *  \param  addr  Start address of the cache line/s
 *  \param  size  Size (in bytes) of the memory to be written back
 *
 */
void Udma_appUtilsCacheWb(const void *addr, int32_t size);

/**
 *  \brief Perform cache invalidate operation based on whether cache is
 *  coherent or not
 *
 *  This internally uses the OSAL API.
 *
 *  \param  addr  Start address of the cache line/s
 *  \param  size  Size (in bytes) of the memory to invalidate
 *
 */
void Udma_appUtilsCacheInv(const void * addr, int32_t size);

/**
 *  \brief Perform cache writeback and invalidate operation based on
 *  whether cache is coherent or not
 *
 *  This internally uses the OSAL API.
 *
 *  \param  addr  Start address of the cache line/s
 *  \param  size  Size (in bytes) of the memory to writeback and invalidate
 *
 */
void Udma_appUtilsCacheWbInv(const void * addr, int32_t size);

/**
 *  \brief Virtual to physical translation function.
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
uint64_t Udma_appVirtToPhyFxn(const void *virtAddr, uint32_t chNum, void *appData);

/**
 *  \brief Physical to virtual translation function.
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
void *Udma_appPhyToVirtFxn(uint64_t phyAddr, uint32_t chNum, void *appData);

/**
 *  \brief Returns if print can be supported for a platform/build
 *
 *  \return TRUE if print can be supported for a platform. Else FALSE
 */
uint32_t Udma_appIsPrintSupported(void);

/**
 *  \brief Setup C66x timer interrupt as required by SYSBIOS
 */
void Udma_appC66xIntrConfig(void);

/**
 *  \brief C7x pre-init function.
 *  Should be called in TI-RTOS main before calling any CLEC or SCICLIENT calls
 *
 *  This switches C7x to non-secure mode, this is needed so that coherency
 *  between A72 (which runs in non-secure mode) and C7x can be effective.
 *  Since SCICLIENT uses non-secure thread, all application should be switched
 *  to non-secure mode as well.
 */
void Udma_appC7xPreInit(void);

/**
 *  \brief Returns if UDMA IP statistics is supported in the SoC
 *
 *  \return TRUE if statistics is supported. Else FALSE
 */
uint32_t Udma_appIsUdmapStatsSupported(void);

#ifdef __cplusplus
}
#endif

#endif  /* #define UDMA_APPUTILS_H_ */
