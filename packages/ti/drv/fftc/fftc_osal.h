/**
 *   @file  fftc_osal.h
 *
 *   @brief   
 *      Default OS Adaptation Layer (OSAL) header file for FFTC Driver. 
 *      Contains the FFTC OSAL API prototype definitions and porting 
 *      guidelines for each of the OSAL APIs.
 *
 *      The system integrator could take either one of the following
 *      approaches in order to port the FFTC driver to his system:
 *      
 *      <b> Approach 1: </b>
 *      @n  Use Prebuilt FFTC Libraries
 *           -  To do so, the integrator would have to ensure that 
 *              a suitable implementation for all the default mapped
 *              OSAL APIs (Osal_xxx defined here) is provided using
 *              the native OS.
 *           -  Link the prebuilt libraries with their application.
 *           -  The "example" directory application demonstrates the
 *              usage of this approach.
 *       @n <b> Pros: </b>
 *           -  Customers can reuse prebuilt TI provided libraries
 *       @n <b> Cons: </b>
 *           -  One extra level of indirection in the API to get to 
 *              the actual OS call
 *             
 *      <b> Approach 2: </b>
 *      @n  Rebuild FFTC Library from sources
 *           -  Create a copy of this file and modify it to map the
 *              FFTC OSAL APIs used in the driver (Fftc_osalXxx APIs 
 *              defined here) to the native OS calls.
 *           -  Rebuild the FFTC library; ensure that the Include 
 *              path points to the directory where the copy of this file 
 *              with OSAL implementation has been provided.
 *           -  Please refer to the "test" directory for an example 
 *              usage of this approach.
 *       @n <b> Pros: </b>
 *           -  Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           -  FFTC Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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
*/
#ifndef _FFTC_OSAL_H_
#define _FFTC_OSAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
@addtogroup FFTC_OSAL_FUNCTION
@{
*/

/**
 * ============================================================================
 *  @n@b Fftc_osalMalloc
 *
 *  @b  brief
 *  @n  This API is called from the FFTC driver to allocate memory
 *      for the FFT request and result data buffers used in Host mode CPPI
 *      descriptors. It can also be called to allocate memory for driver's
 *      internal book-keeping objects.
 *
 *      This memory can be allocated from a core's local heap.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @param[in]  bGlobalAddress
 *      Indicates whether the address returned by this API should be
 *      a global address or a core local address. Global addresses are
 *      required by driver when allocating CPPI descriptors and buffers. 
 *
 *  @return
 *      Allocated block address
 *
 *  @pre
 *  @n  The buffers allocated using this API MUST be translated to 
 *      global addresses when 'bGlobalAddress' is set to 1. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void*  Osal_fftcMalloc (uint32_t num_bytes, uint8_t bGlobalAddress)
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalMalloc         Osal_fftcMalloc

/**
 * ============================================================================
 *  @n@b Fftc_osalFree
 *
 *  @b  brief
 *  @n  This API is called from the FFTC driver to free up the memory
 *      allocated for buffers earlier using @a Fftc_osalMalloc () API.
 *
 *  @param[in]  datavoid*
 *      Pointer to the memory block to be cleaned up. 
 *
 *  @param[in]  num_bytes
 *      Size of the memory block to be cleaned up.
 *
 *  @param[in]  bGlobalAddress
 *      Indicates that the address passed here to this function is 
 *      a Global address.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_fftcFree (void* dataPtr, uint32_t num_bytes, uint8_t bGlobalAddress)
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalFree           Osal_fftcFree

/**
 * ============================================================================
 *  @n@b Fftc_osalMultiCoreCsEnter
 *
 *  @b  brief
 *  @n  This API is called from the FFTC driver to acquire a multi-core, 
 *      multi-threaded synchronization lock, i.e., a lock that once obtained
 *      that would ensure that no process/thread on the current core or any
 *      of the cores can access the FFTC library APIs until its released by
 *      the current user.
 *
 *      This API is called from the FFTC driver mainly from the control path 
 *      APIs, i.e., @a Fftc_init (), @a Fftc_open () etc to ensure that
 *      the data structures shared between all the driver users are updated
 *      correctly. 
 *
 *  @param[in]  void
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_fftcMultiCoreCsEnter (void)
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalMultiCoreCsEnter       Osal_fftcMultiCoreCsEnter

/**
 * ============================================================================
 *  @n@b Fftc_osalMultiCoreCsExit
 *
 *  @b  brief
 *  @n  This API is called from the FFTC driver to release a multi-core and 
 *      multi-threaded lock previously obtained using @a Fftc_osalMultiCoreCsEnter () 
 *      API. This API should reset the multi-core, multi-threaded lock 
 *      enabling another process/core to grab it to access the FFTC library.
 *
 *  @param[in]  void
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_fftcMultiCoreCsExit (void)
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalMultiCoreCsExit        Osal_fftcMultiCoreCsExit

/**
 * ============================================================================
 *  @n@b Fftc_osalInterruptCsEnter
 *
 *  @b  brief
 *  @n  This API is called from the FFTC driver to acquire an interrupt lock, 
 *      i.e., a lock that can protect the driver against context switch to an
 *      interrupt when its manipulating data structures shared between the
 *      application process and interrupt handler (ISR).
 *
 *      This API is called from the FFTC @a Fftc_rxGetResult () API to protect
 *      the driver maintained list of results posted by ISR against manipulation 
 *      from ISR at the same time its being accessed by the application via
 *      @a Fftc_rxGetResult () API. This API should disable the interrupts
 *      to ensure that protection against interrupts.
 *
 *  @param[in]  void
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_fftcInterruptCsEnter (void)
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalInterruptCsEnter       Osal_fftcInterruptCsEnter

/**
 * ============================================================================
 *  @n@b Fftc_osalInterruptCsExit
 *
 *  @b  brief
 *  @n  This API is called from the FFTC driver to release an interrupt
 *      lock previously obtained using @a Fftc_osalInterruptCsEnter () 
 *      API. This API should enable back interrupts disabled earlier in
 *      @a Fftc_osalInterruptCsEnter () API.
 *
 *  @param[in]  void
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_fftcInterruptCsExit (void)
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalInterruptCsExit        Osal_fftcInterruptCsExit

/**
 * ============================================================================
 *  @n@b Fftc_osalLog
 *
 *  @b  brief
 *  @n  This API is called from the FFTC driver to log useful debug information. 
 *
 *  @param[in]  void
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_fftcLog (char* fmt, ... ) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalLog            Osal_fftcLog

/**
 * ============================================================================
 *  @n@b Fftc_osalCreateSem
 *
 *  @b  brief
 *  @n  This API is called from the FFTC driver to create a software semaphore. 
 *
 *  @param[in]  void
 *
 *  @return     
 *  @n  void*       -   Semaphore handle.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_fftcCreateSem (void) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalCreateSem      Osal_fftcCreateSem

/**
 * ============================================================================
 *  @n@b Fftc_osalDeleteSem
 *
 *  @b  brief
 *  @n  This API is called from the driver to delete a software semaphore
 *      created earlier using @a Fftc_osalCreateSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Fftc_osalCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_fftcDeleteSem (void* hSem) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalDeleteSem      Osal_fftcDeleteSem


/**
 * ============================================================================
 *  @n@b Fftc_osalPendSem
 *
 *  @b  brief
 *  @n  This API is called from the driver to acquire a software semaphore
 *      created earlier using @a Fftc_osalCreateSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Fftc_osalCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_fftcPendSem (void* hSem) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalPendSem        Osal_fftcPendSem


/**
 * ============================================================================
 *  @n@b Fftc_osalPostSem
 *
 *  @b  brief
 *  @n  This API is called from the driver to release a semaphore acquired 
 *      earlier using @a Fftc_osalPendSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Fftc_osalCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_fftcPostSem (void* hSem) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalPostSem        Osal_fftcPostSem

/**
 * ============================================================================
 *  @n@b Fftc_osalBeginMemAccess
 *
 *  @b  brief
 *  @n  This API is called from the driver before it makes a read access to its 
 *      multicore shared datastructures. If these data structures were placed
 *      in cacheable memory region, then the application would have to ensure
 *      that the cache is updated with the contents from actual physical memory.
 *
 *  @param[in]  
 *      blockPtr    -   Address of the memory block that the driver is trying
 *                      to access.
 *
 *  @param[in]  
 *      size        -   Size of the memory block being accessed.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_fftcBeginMemAccess (void *blockPtr, uint32_t size) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalBeginMemAccess Osal_fftcBeginMemAccess


/**
 * ============================================================================
 *  @n@b Fftc_osalEndMemAccess
 *
 *  @b  brief
 *  @n  This API is called from the driver once its done writing to its
 *      multicore shared datastructures. If these data structures were placed
 *      in cacheable memory region, then the application would have to ensure
 *      that the actual physical memory is updated as per the contents of cache.
 *
 *  @param[in]  
 *      blockPtr    -   Address of the memory block that the driver is has written
 *                      to.
 *
 *  @param[in]  
 *      size        -   Size of the memory block being accessed.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_fftcEndMemAccess (void *blockPtr, uint32_t size) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalEndMemAccess   Osal_fftcEndMemAccess

/**
 * ============================================================================
 *  @n@b Fftc_osalBeginDataBufMemAccess
 *
 *  @b  brief
 *  @n  This API is called from the driver before it makes a read access to  
 *      data buffers on the data path. If the data buffers were allocated
 *      in cacheable memory region, then the application would have to ensure
 *      that the cache is updated with the contents from actual physical memory.
 *
 *  @param[in]  
 *      dataBufPtr  -   Address of the data buffer memory block that the 
 *                      driver is trying to access.
 *
 *  @param[in]  
 *      size        -   Size of the memory block being accessed.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_fftcBeginDataBufMemAccess (void *dataBufPtr, uint32_t size) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalBeginDataBufMemAccess Osal_fftcBeginDataBufMemAccess


/**
 * ============================================================================
 *  @n@b Fftc_osalEndDataBufMemAccess
 *
 *  @b  brief
 *  @n  This API is called from the driver once its done writing to a data buffer
 *      on the data path. If the data buffers were allocated in cacheable memory 
 *      region, then the application would have to ensure that the actual physical 
 *      memory is updated as per the contents of cache.
 *
 *  @param[in]  
 *      dataBufPtr  -   Address of the data buffer memory block that the driver 
 *                      has written to.
 *
 *  @param[in]  
 *      size        -   Size of the memory block being accessed.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_fftcEndDataBufMemAccess (void *dataBufPtr, uint32_t size) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalEndDataBufMemAccess   Osal_fftcEndDataBufMemAccess

/**
 * ============================================================================
 *  @n@b Fftc_osalBeginDescMemAccess
 *
 *  @b  brief
 *  @n  This API is called from the driver before it makes a read access to  
 *      CPPI descriptors on the Rx data path. If the descriptors were allocated
 *      in cacheable memory region, then the application would have to ensure
 *      that the cache is updated with the contents from actual physical memory.
 *
 *  @param[in]  
 *      hRx         -   Rx object handle. Can be used by application to retrieve
 *                      correct descriptor size, and location of the descriptors 
 *                      if the descriptors and Rx flow were allocated using 
 *                      application managed configuration.
 *
 *  @param[in]  
 *      descPtr     -   Address of the descriptors that the driver is trying to 
 *                      access.
 *
 *  @param[in]  
 *      size        -   Size of the descriptor being accessed. Can be zero if
 *                      descriptors are not managed by the driver. In this case,
 *                      using the hRx parameter passed the application must decide
 *                      descriptor size and if it was allocated in cacheable memory
 *                      region. 
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Fftc_osalBeginDescMemAccess (void* hRx, void *descPtr, uint32_t size) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalBeginDescMemAccess Osal_fftcBeginDescMemAccess


/**
 * ============================================================================
 *  @n@b Fftc_osalEndDescMemAccess
 *
 *  @b  brief
 *  @n  This API is called from the driver once its done writing to a descriptor
 *      on the data path/control path during descriptor initialization. 
 *      If the descriptors were allocated in cacheable memory region, then the 
 *      application would have to ensure that the actual physical memory is 
 *      updated as per the contents of cache.
 *
 *  @param[in]  
 *      descPtr     -   Address of the descriptor that the driver has written to.
 *
 *  @param[in]  
 *      size        -   Size of the memory block being accessed.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Fftc_osalEndDescMemAccess (void *descPtr, uint32_t size) 
    @endverbatim
 * =============================================================================
 */
#define     Fftc_osalEndDescMemAccess   Osal_fftcEndDescMemAccess

/**
@}
*/

extern void*  Osal_fftcMalloc 
(
    uint32_t                        num_bytes,
    uint8_t                         bGlobalAddress
);

extern void Osal_fftcFree 
(
    void*                           dataPtr, 
    uint32_t                        num_bytes,
    uint8_t                         bGlobalAddress
);

extern void Osal_fftcMultiCoreCsEnter 
(
);

extern void Osal_fftcMultiCoreCsExit 
(
);

extern void Osal_fftcInterruptCsEnter 
(
);

extern void Osal_fftcInterruptCsExit 
(
);

extern void Osal_fftcLog 
(
    char*                           fmt, 
    ... 
);

extern void* Osal_fftcCreateSem
(
    void
);

extern void  Osal_fftcDeleteSem
(
    void*                           hSem
);

extern void  Osal_fftcPendSem
(
    void*                           hSem
);

extern void  Osal_fftcPostSem
(
    void*                           hSem
);

extern void*  memcpy 
(
    void*                           dest, 
    const void*                     src, 
    size_t                          count
);

extern void*  memset 
(
    void*                           ptr, 
    int32_t                         val, 
    size_t                          count
);

extern void  Osal_fftcBeginMemAccess
(
    void*                           pBlockPtr,
    uint32_t                        byteCnt
);

extern void  Osal_fftcEndMemAccess
(
    void*                           pBlockPtr,
    uint32_t                        byteCnt
);

extern void  Osal_fftcBeginDataBufMemAccess
(
    void*                           pDataBuf,
    uint32_t                        byteCnt
);

extern void  Osal_fftcEndDataBufMemAccess
(
    void*                           pDataBuf,
    uint32_t                        byteCnt
);

extern void  Osal_fftcBeginDescMemAccess
(
    void*                           hRx,
    void*                           pDesc,
    uint32_t                        byteCnt
);

extern void  Osal_fftcEndDescMemAccess
(
    void*                           pDesc,
    uint32_t                        byteCnt
);

#ifdef __cplusplus
}
#endif

#endif /* _FFTC_OSAL_H_ */
