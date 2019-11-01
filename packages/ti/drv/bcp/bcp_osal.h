/**
 *   @file  bcp_osal.h
 *
 *   @brief   
 *      Default OS Adaptation Layer (OSAL) header file for BCP Driver. 
 *      Contains the BCP OSAL API prototype definitions and porting 
 *      guidelines for each of the OSAL APIs.
 *
 *      The system integrator could take either one of the following
 *      approaches in order to port the BCP driver to his system:
 *      
 *      <b> Approach 1: </b>
 *      @n  Use Prebuilt BCP Libraries
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
 *      @n  Rebuild BCP Library from sources
 *           -  Create a copy of this file and modify it to map the
 *              BCP OSAL APIs used in the driver (Bcp_osalXxx APIs 
 *              defined here) to the native OS calls.
 *           -  Rebuild the BCP library; ensure that the Include 
 *              path points to the directory where the copy of this file 
 *              with OSAL implementation has been provided.
 *           -  Please refer to the "test" directory for an example 
 *              usage of this approach.
 *       @n <b> Pros: </b>
 *           -  Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           -  BCP Libraries need to be rebuilt by the customer.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
#ifndef _BCP_OSAL_H_
#define _BCP_OSAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
@addtogroup BCP_OSAL_FUNCTION
@{
*/

/**
 * ============================================================================
 *  @n@b Bcp_osalMalloc
 *
 *  @b  brief
 *  @n  This API is called from the BCP driver to allocate memory
 *      for the FFT request and result buffers used in Host mode CPPI
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
        void*  Osal_bcpMalloc (uint32_t num_bytes, uint8_t bGlobalAddress)
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalMalloc         Osal_bcpMalloc

/**
 * ============================================================================
 *  @n@b Bcp_osalFree
 *
 *  @b  brief
 *  @n  This API is called from the BCP driver to free up the memory
 *      allocated for buffers earlier using @a Bcp_osalMalloc () API.
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
        void Osal_bcpFree (void* dataPtr, uint32_t num_bytes, uint8_t bGlobalAddress)
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalFree                Osal_bcpFree

/**
 * ============================================================================
 *  @n@b Bcp_osalMultiCoreCsEnter
 *
 *  @b  brief
 *  @n  This API is called from the BCP driver to acquire a multi-core, 
 *      multi-threaded synchronization lock, i.e., a lock that once obtained
 *      that would ensure that no process/thread on the current core or any
 *      of the cores can access the BCP library APIs until its released by
 *      the current user.
 *
 *      This API is called from the BCP driver mainly from the control path 
 *      APIs, i.e., @a Bcp_init (), @a Bcp_open () etc to ensure that
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
        void Osal_bcpMultiCoreCsEnter (void)
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalMultiCoreCsEnter       Osal_bcpMultiCoreCsEnter

/**
 * ============================================================================
 *  @n@b Bcp_osalMultiCoreCsExit
 *
 *  @b  brief
 *  @n  This API is called from the BCP driver to release a multi-core and 
 *      multi-threaded lock previously obtained using @a Bcp_osalMultiCoreCsEnter () 
 *      API. This API should reset the multi-core, multi-threaded lock 
 *      enabling another process/core to grab it to access the BCP library.
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
        void Osal_bcpMultiCoreCsExit (void)
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalMultiCoreCsExit        Osal_bcpMultiCoreCsExit

/**
 * ============================================================================
 *  @n@b Bcp_osalInterruptCsEnter
 *
 *  @b  brief
 *  @n  This API is called from the BCP driver to acquire an interrupt lock, 
 *      i.e., a lock that can protect the driver against context switch to an
 *      interrupt when its manipulating data structures shared between the
 *      application process and interrupt handler (ISR).
 *
 *      This API is called from the BCP @a Bcp_rxGetResult () API to protect
 *      the driver maintained list of results posted by ISR against manipulation 
 *      from ISR at the same time its being accessed by the application via
 *      @a Bcp_rxGetResult () API. This API should disable the interrupts
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
        void Osal_bcpInterruptCsEnter (void)
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalInterruptCsEnter       Osal_bcpInterruptCsEnter

/**
 * ============================================================================
 *  @n@b Bcp_osalInterruptCsExit
 *
 *  @b  brief
 *  @n  This API is called from the BCP driver to release an interrupt
 *      lock previously obtained using @a Bcp_osalInterruptCsEnter () 
 *      API. This API should enable back interrupts disabled earlier in
 *      @a Bcp_osalInterruptCsEnter () API.
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
        void Osal_bcpInterruptCsExit (void)
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalInterruptCsExit        Osal_bcpInterruptCsExit

/**
 * ============================================================================
 *  @n@b Bcp_osalLog
 *
 *  @b  brief
 *  @n  This API is called from the BCP driver to log useful debug information. 
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
       void Osal_bcpLog (char* fmt, ... ) 
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalLog            Osal_bcpLog

/**
 * ============================================================================
 *  @n@b Bcp_osalCreateSem
 *
 *  @b  brief
 *  @n  This API is called from the BCP driver to create a software semaphore. 
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
       void* Osal_bcpCreateSem (void) 
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalCreateSem      Osal_bcpCreateSem

/**
 * ============================================================================
 *  @n@b Bcp_osalDeleteSem
 *
 *  @b  brief
 *  @n  This API is called from the driver to delete a software semaphore
 *      created earlier using @a Bcp_osalCreateSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Bcp_osalCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_bcpDeleteSem (void* hSem) 
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalDeleteSem      Osal_bcpDeleteSem


/**
 * ============================================================================
 *  @n@b Bcp_osalPendSem
 *
 *  @b  brief
 *  @n  This API is called from the driver to acquire a software semaphore
 *      created earlier using @a Bcp_osalCreateSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Bcp_osalCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_bcpPendSem (void* hSem) 
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalPendSem        Osal_bcpPendSem


/**
 * ============================================================================
 *  @n@b Bcp_osalPostSem
 *
 *  @b  brief
 *  @n  This API is called from the driver to release a semaphore acquired 
 *      earlier using @a Bcp_osalPendSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Bcp_osalCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_bcpPostSem (void* hSem) 
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalPostSem        Osal_bcpPostSem

/**
 * ============================================================================
 *  @n@b Bcp_osalBeginMemAccess
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
       void Osal_bcpBeginMemAccess (void *blockPtr, uint32_t size) 
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalBeginMemAccess Osal_bcpBeginMemAccess


/**
 * ============================================================================
 *  @n@b Bcp_osalEndMemAccess
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
       void Osal_bcpEndMemAccess (void *blockPtr, uint32_t size) 
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalEndMemAccess   Osal_bcpEndMemAccess

/**
 * ============================================================================
 *  @n@b Bcp_osalBeginDescMemAccess
 *
 *  @b  brief
 *  @n  This API is called from the driver before it makes a read access to  
 *      CPPI descriptors on the Rx data path. If the descriptors were allocated
 *      in cacheable memory region, then the application would have to ensure
 *      that the cache is updated with the contents from actual physical memory.
 *
 *  @param[in]  
 *      hRx         -   Rx object handle. Must be used by application to retrieve
 *                      correct descriptor size, and location of the descriptors.
 *
 *  @param[in]  
 *      descPtr     -   Address of the descriptors that the driver is trying to 
 *                      access.
 *  @return     
 *  @n  void
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Bcp_osalBeginDescMemAccess (void* hRx, void *descPtr) 
    @endverbatim
 * =============================================================================
 */
#define     Bcp_osalBeginDescMemAccess Osal_bcpBeginDescMemAccess

/**
@}
*/

extern void*  Osal_bcpMalloc 
(
    uint32_t                        num_bytes,
    uint8_t                         bGlobalAddress
);

extern void Osal_bcpFree 
(
    void*                           dataPtr, 
    uint32_t                        num_bytes,
    uint8_t                         bGlobalAddress
);

extern void Osal_bcpMultiCoreCsEnter 
(
);

extern void Osal_bcpMultiCoreCsExit 
(
);

extern void Osal_bcpInterruptCsEnter 
(
);

extern void Osal_bcpInterruptCsExit 
(
);

extern void Osal_bcpLog 
(
    char*                           fmt, 
    ... 
);

extern void* Osal_bcpCreateSem
(
    void
);

extern void  Osal_bcpDeleteSem
(
    void*                           hSem
);

extern void  Osal_bcpPendSem
(
    void*                           hSem
);

extern void  Osal_bcpPostSem
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

extern void  Osal_bcpBeginMemAccess
(
    void*                           pBlockPtr,
    uint32_t                        byteCnt
);

extern void  Osal_bcpEndMemAccess
(
    void*                           pBlockPtr,
    uint32_t                        byteCnt
);

extern void  Osal_bcpBeginDescMemAccess
(
    void*                           hRx,
    void*                           pDesc
);

#ifdef __cplusplus
}
#endif

#endif /* _BCP_OSAL_H_ */
