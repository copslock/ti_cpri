/**
 *   @file  qmss_osal.h
 *
 *   @brief
 *      This is the sample OS Adaptation layer which is used by the QMSS low level
 *      driver. The OSAL layer can be ported in either of the following
 *      manners to a native OS:
 *
 *      <b> Approach 1: </b>
 *      @n  Use Prebuilt Libraries
 *           - Ensure that the provide an implementation of all
 *             Osal_XXX API for their native OS.
 *           - Link the prebuilt libraries with their application.
 *           - Refer to the "example" directory for an example of this
 *       @n <b> Pros: </b>
 *           - Customers can reuse prebuilt TI provided libraries
 *       @n <b> Cons: </b>
 *           - Level of indirection in the API to get to the actual OS call
 *
 *      <b> Approach 2: </b>
 *      @n  Rebuilt Library
 *           - Create a copy of this file and modify it to directly
 *             inline the native OS calls
 *           - Rebuild the QMSS low level driver library; ensure that the Include
 *             path points to the directory where the copy of this file
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - QMSS LLD Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009-2012 Texas Instruments, Inc.
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
#ifndef __QMSS_OSAL_H__
#define __QMSS_OSAL_H__

/** @addtogroup QMSS_LLD_OSAL
 @{ */

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

extern void* Osal_qmssMalloc (uint32_t num_bytes);
extern void Osal_qmssFree (void *ptr, uint32_t size);
extern void* Osal_qmssCsEnter (void);
extern void Osal_qmssCsExit (void *CsHandle);
extern void* Osal_qmssAccCsEnter (void);
extern void Osal_qmssAccCsExit (void *CsHandle);
extern void* Osal_qmssMtCsEnter (void);
extern void Osal_qmssMtCsExit (void *CsHandle);
extern void Osal_qmssLog (char *fmt, ... );
extern void Osal_qmssBeginMemAccess (void *ptr, uint32_t size);
extern void Osal_qmssEndMemAccess (void *ptr, uint32_t size);
/**
 * @brief   This function is used to convert virtual addresses to physical
 *          addresses for objects other than descriptors, such as
 *          linking RAM or descriptor region base addresses.
 *
 *          Implementation is specified as function pointer in qmss_device.c.
 *
 *  @param[in]  ptr
 *      Virtual/logical address
 *
 *  @retval
 *      Physical address
 *
 */
extern void* Osal_qmssVirtToPhy (void *ptr);
/**
 * @brief   This function is used to convert physical addresses to virtual
 *          addresses for objects other than descriptors, such as
 *          linking RAM or descriptor region base addresses.
 *
 *          Implementation is specified as function pointer in qmss_device.c.
 *
 *  @param[in]  ptr
 *      Physical address
 *
 *  @retval
 *      Virtual/logical address
 *
 */
extern void* Osal_qmssPhyToVirt (void *ptr);
/**
 * @brief   This function is used to convert virtual addresses to physical
 *          addresses for individual descriptors.
 *
 *          Implementation is specified as function pointer in qmss_device.c.
 *
 *  @param[in]  QID
 *      16-bit queue ID which can be used to quickly access translation
 *      table which could be different per queue.
 *
 *  @param[in]  descAddr
 *      Virtual/logical address
 *
 *  @retval
 *      Physical address
 *
 */
extern void* Osal_qmssConvertDescVirtToPhy(uint32_t QID, void *descAddr);
/**
 * @brief   This function is used to convert physical addresses to virtual
 *          addresses for individual descriptors.
 *
 *          Implementation is specified as function pointer in qmss_device.c.
 *
 *  @param[in]  QID
 *      16-bit queue ID which can be used to quickly access translation
 *      table which could be different per queue.
 *
 *  @param[in]  descAddr
 *      Physical address
 *
 *  @retval
 *      Virtual/logical address
 *
 */
extern void* Osal_qmssConvertDescPhyToVirt(uint32_t QID, void *descAddr);

/**
 * @brief   This function is used to issue memory barrier.
 *
 *          Implementation is specified as function pointer in qmss_device.c.
 *
 *  <b> Parameter </b>
 *  @n  None.
 *
 *  <b> Return Value </b>
 *  @n  None.
 *
 */
extern void* Osal_qmssMemBarrier(uint32_t QID, void *descAddr);

/**
 * @brief   The macro is used by the QMSS LLD to allocate memory of specified
 * size
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_qmssMalloc (uint32_t numBytes)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Number of bytes to be allocated
 *
 *  <b> Return Value </b>
 *  @n  Pointer to the allocated block size
 */

#define Qmss_osalMalloc             Osal_qmssMalloc

/**
 * @brief   The macro is used by the QMSS LLD to free a allocated block of
 * memory
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssFree (void *ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Pointer to the block of memory to be cleaned up.
 *  @n  Size of the allocated memory which is being freed.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */

#define Qmss_osalFree               Osal_qmssFree

/**
 * @brief   The macro is used by the QMSS LLD accumulator functions
 *      to provide critical sections to protect global and shared
 *      variables from access from multiple cores and
 *      access from multiple threads on single core
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_qmssAccCsEnter (void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  None.
 *
 *  <b> Return Value </b>
 *  @n  Handle used to lock critical section.
 */
#define Qmss_osalAccCsEnter            Osal_qmssAccCsEnter

/**
 * @brief   The macro is used by the QMSS LLD accumulator functions
 *      to exit a critical section
 *      protected using Osal_qmssAccCsEnter() API.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssAccCsExit (void *CsHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Handle for unlocking critical section.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Qmss_osalAccCsExit             Osal_qmssAccCsExit

/**
 * @brief   The macro is used by the QMSS LLD to provide critical sections to
 * protect global and shared variables from
 *
 *      access from multiple cores
 *      and
 *      access from multiple threads on single core
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_qmssCsEnter (void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  None.
 *
 *  <b> Return Value </b>
 *  @n  Handle used to lock critical section.
 */
#define Qmss_osalCsEnter            Osal_qmssCsEnter

/**
 * @brief   The macro is used by the QMSS LLD to exit a critical section
 *      protected using Osal_qmssCsEnter() API.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssCsExit (void *CsHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Handle for unlocking critical section.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Qmss_osalCsExit             Osal_qmssCsExit

/**
 * @brief   The macro is used by the QMSS LLD to provide critical sections to
 * protect global and shared variables from
 *
 *      access from multiple threads on single core
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_qmssMtCsEnter (void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  None.
 *
 *  <b> Return Value </b>
 *  @n  Handle used to lock critical section.
 */
#define Qmss_osalMtCsEnter          Osal_qmssMtCsEnter

/**
 * @brief   The macro is used by the QMSS LLD to exit a critical section
 *      protected using Osal_qmssMtCsEnter() API.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssMtCsExit (void *CsHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Handle for unlocking critical section.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Qmss_osalMtCsExit           Osal_qmssMtCsExit

/**
 * @brief   The macro is used by the QMSS LLD to log various
 * messages.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssLog( char *fmt, ... )
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  printf-style format string
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Qmss_osalLog                Osal_qmssLog

/**
 * @brief   The macro is used by the QMSS LLD to indicate that a block
 * of memory is about to be accessed. If the memory block is cached then
 * this indicates that the application would need to ensure that the cache
 * is updated with the data from the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssBeginMemAccess (void *ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Qmss_osalBeginMemAccess     Osal_qmssBeginMemAccess

/**
 * @brief   The macro is used by the QMSS LLD to indicate that the block of
 * memory has finished being accessed. If the memory block is cached then the
 * application would need to ensure that the contents of the cache are updated
 * immediately to the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssEndMemAccess (void *ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Qmss_osalEndMemAccess       Osal_qmssEndMemAccess

/**
@}
*/
#endif /* __QMSS_OSAL_H__ */

