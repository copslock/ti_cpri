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
 *  \file ipc_apputils.h
 *
 *  \brief Common IPC application utility used in all IPC example.
 *
 *  NOTE: This library is meant only for IPC examples. Customers are not
 *  encouraged to use this layer as these are very specific to the examples
 *  written and the API behaviour and signature can change at any time to
 *  suit the examples.
 *
 */

#ifndef IPC_APPUTILS_H_
#define IPC_APPUTILS_H_

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
void Ipc_appUtilsCacheWb(const void *addr, int32_t size);

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
void Ipc_appUtilsCacheInv(const void * addr, int32_t size);

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
void Ipc_appUtilsCacheWbInv(const void * addr, int32_t size);

/**
 *  \brief Virtual to physical translation function.
 *
 *  \param virtAddr [IN] Virtual address
 *  \param chNum    [IN] Channel number passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to #IPC_DMA_CH_INVALID.
 *  \param appData  [IN] Callback pointer passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to NULL.
 *
 *  \return Corresponding physical address
 */
uint64_t Ipc_appVirtToPhyFxn(const void *virtAddr, uint32_t chNum, void *appData);

/**
 *  \brief Physical to virtual translation function.
 *
 *  \param phyAddr  [IN] Physical address
 *  \param chNum    [IN] Channel number passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to #IPC_DMA_CH_INVALID.
 *  \param appData  [IN] Callback pointer passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to NULL.
 *
 *  \return Corresponding virtual address
 */
void *Ipc_appPhyToVirtFxn(uint64_t phyAddr, uint32_t chNum, void *appData);

/**
 *  \brief Returns if print can be supported for a platform/build
 *
 *  \return TRUE if print can be supported for a platform. Else FALSE
 */
uint32_t Ipc_appIsPrintSupported(void);

/**
 *  \brief Setup C66x timer interrupt as required by SYSBIOS
 */
void Ipc_appC66xIntrConfig(void);

/**
 * \brief Sets core frequency
 *
 * \param selfId Core ID
 * */
void Ipc_setCoreFrq(uint32_t selfId);

/**
 * brief Get timestam in micro-second
 *
 * \param frq frequency
 *
 * \returns timestamp
 * */
uint64_t Ipc_getTimeInUsec(uint64_t frq);

/**
 * \brief Gets the Time Stamp
 *
 * */
uint64_t Ipc_getTimestampFrq(void);

/**
 * \brief Sets CPU frequency in Hz
 * */
int32_t Ipc_setCpuHz(uint32_t freq);


#ifdef __cplusplus
}
#endif

#endif  /* #define IPC_APPUTILS_H_ */
