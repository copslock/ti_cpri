/**
 *   @file  multicore_osal.c
 *
 *   @brief   
 *      This is a sample OS Abstraction Layer (AL) file implemented
 *      using XDC/BIOS APIs.
 *
 *      System integrator is advised to review these implementations and
 *      modify them to suit it to application requirements.
 *
 *      This OSAL implementation uses the <b> Approach 1 </b> documented.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, 2014, Texas Instruments, Inc.
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
/* Standard C-native includes  */
#include <stdlib.h>
#include <string.h>

/* XDC/BIOS includes */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <xdc/cfg/global.h>

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>

/* CSL CHIP, SEM Functional layer includes */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_semAux.h>

/* CSL Cache module includes */
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

/* CSL XMC includes */
#include <ti/csl/csl_xmc.h>
#include <ti/csl/csl_xmcAux.h>

/**********************************************************************
 ****************************** Defines *******************************
 **********************************************************************/

/* Number of cores on c6498 */
#define     NUM_CORES           4

/* Hardware Semaphore to synchronize access from
 * multiple FFTC applications across different cores to
 * the FFTC driver.
 */
#define     FFTC_HW_SEM         2 

/* Hardware Semaphore to synchronize access from
 * multiple applications (FFTC applications and non-FFTC applications)
 * across different cores to the QMSS library.
 */
#define     QMSS_HW_SEM         3 

/* Hardware Semaphore to synchronize access from
 * multiple applications (FFTC applications and non-FFTC applications)
 * across different cores to the CPPI library.
 */
#define     CPPI_HW_SEM         4 

/* Hardware Semaphore to synchronize access from
 * multiple applications (FFTC applications and non-FFTC applications)
 * for accumulator queues across different cores to the QMSS library.
 */
#define     QMSS_ACC_HW_SEM     5 

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
UInt32      fftcMallocCounter       =   0;
UInt32      fftcFreeCounter         =   0;
UInt32      fftcCppiMallocCounter   =   0;
UInt32      fftcCppiFreeCounter     =   0;
UInt32      fftcQmssMallocCounter   =   0;
UInt32      fftcQmssFreeCounter     =   0;

UInt32      coreKey [NUM_CORES];

#undef		FFTC_TEST_DEBUG

/**********************************************************************
 *********************** FFTC OSAL Functions **************************
 **********************************************************************/

/**
 * ============================================================================
 *  @n@b Osal_fftcLocal2Global
 *
 *  @b  brief
 *  @n  Utility function which converts a core local address to a global
 *      address.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @return
 *      Global Address
 * =============================================================================
 */
static UInt32 Osal_fftcLocal2Global (UInt32 addr)
{
    UInt32 corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM); 

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum * 0x01000000)));
}

/**
 * ============================================================================
 *  @n@b Osal_fftcGlobal2Local
 *
 *  @b  brief
 *  @n  Utility function which converts a global to core local address.
 *
 *  @param[in]  gaddr
 *      Global address to be converted
 *
 *  @return
 *      Local Address
 * =============================================================================
 */
static UInt32 Osal_fftcGlobal2Local (UInt32 gaddr)
{
    UInt32 corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM);

    /* Compute the global address. */
    return (gaddr & ~((1 << 28) | (corenum << 24)));    
}

/**
 * ============================================================================
 *  @n@b Osal_fftcMalloc
 *
 *  @b  brief
 *  @n  This API allocates a memory block of a given
 *      size specified by input parameter 'num_bytes'.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @param[in]  bGlobalAddress
 *      Indicates whether the address returned by this API should be
 *      a global address or a core local address. Global addresses are
 *      required when allocating CPPI descriptors and buffers.
 *
 *  @return
 *      Allocated block address
 * =============================================================================
 */
Ptr Osal_fftcMalloc (UInt32 num_bytes, Bool bGlobalAddress)
{
	Error_Block	    errorBlock;
    Ptr             destPtr;

    /* Increment the allocation counter. */
    fftcMallocCounter++;	

	/* Allocate memory from the heap */
	if (destPtr = Memory_alloc(NULL, num_bytes, 0, &errorBlock))
    {
        /* Convert the core local address obtained from
         * Memory_alloc API to a Global Address. The 
         * CPPI/QMSS libraries cannot handle buffers and
         * descriptors that are core local addresses.
         */
        if (bGlobalAddress)
            return ((Ptr) Osal_fftcLocal2Global ((UInt32) destPtr));
        else
            return ((Ptr) destPtr);
    }
    else
        return destPtr;
}

/**
 * ============================================================================
 *  @n@b Osal_fftcFree
 *
 *  @b  brief
 *  @n  This API frees and restores a given memory location 
 *      pointer 'dataPtr' of size 'num_bytes' to its
 *      original heap location.
 *
 *  @param[in]  dataPtr
 *      Pointer to the memory block to be cleaned up.
 *
 *  @param[in]  num_bytes
 *      Size of the memory block to be cleaned up.
 *
 *  @param[in]  bGlobalAddress
 *      Indicates that the address passed here to this function is 
 *      a Global address.
 *
 *  @return
 *      Not Applicable
 * =============================================================================
 */
Void Osal_fftcFree (Ptr dataPtr, UInt32 num_bytes, Bool bGlobalAddress)
{
    /* Increment the free counter. */
    fftcFreeCounter++;	
    
    /* Free up the memory */
    if (dataPtr)
    {
        /* Convert the global address to local address since
         * thats what the heap understands.
         */
        if (bGlobalAddress)
            Memory_free(NULL, (Ptr) Osal_fftcGlobal2Local ((UInt32) dataPtr), num_bytes);
        else
            Memory_free(NULL, (Ptr) dataPtr, num_bytes);
    }
}

/**
 * ============================================================================
 *  @n@b Osal_fftcMultiCoreCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization to the caller.
 *
 *      This is a BLOCKING API.
 *
 *      This API ensures multi-core synchronization between
 *      multiple processes trying to access FFTC shared
 *      library at the same time.
 *
 *  @param[in]  None
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_fftcMultiCoreCsEnter (Void)
{
    /* Get the hardware semaphore. 
     *
     * Acquire Multi core synchronization lock 
     */
    while ((CSL_semAcquireDirect (FFTC_HW_SEM)) == 0);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_fftcMultiCoreCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_fftcFftcCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab it.
 *
 *  @param[in]  None
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_fftcMultiCoreCsExit (Void)
{
    /* Release the hardware semaphore 
     *
     * Release multi-core lock.
     */ 
    CSL_semReleaseSemaphore (FFTC_HW_SEM);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_fftcInterruptCsEnter
 *
 *  @b  brief
 *  @n  This API ensures protection against interrupts to the caller. It prevents
 *      the caller from switching to interrupt context from the application 
 *      thread/process context.
 *
 *  @param[in]  None
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_fftcInterruptCsEnter ()
{
    /* Disable all interrupts. 
     *
     * Acquire interrupt lock to protect from any context switches
     * from application thread/process context.
     */
    coreKey [CSL_chipReadDNUM ()] = Hwi_disable();

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_fftcInterruptCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_fftcInterruptCsEnter ()
 *      API. It restores the saved interrupt context and enables back the
 *      interrupts.
 *
 *  @param[in]  None
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_fftcInterruptCsExit ()
{
    /* Enable all interrupts.
     *
     * Release interrupt lock.
     */
    Hwi_restore(coreKey [CSL_chipReadDNUM ()]);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_fftcLog
 *
 *  @b brief
 *  @n  
 *      The function is the FFTC OSAL Logging API which logs 
 *      the messages on the console.
 *
 *  @param[in]  fmt
 *      Formatted String.
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_fftcLog ( String fmt, ... )
{
    return;
}

/**
 * ============================================================================
 *  @n@b Osal_fftcCreateSem
 *
 *  @b  brief
 *  @n  This API creates a software semaphore.
 *
 *  @param[in]  Void
 *
 *  @return     
 *  @n  Void*       -   Semaphore handle.
 *
 * =============================================================================
 */
Void* Osal_fftcCreateSem (Void)
{
    return ((Void *) Semaphore_create (0, NULL, NULL));
}

/**
 * ============================================================================
 *  @n@b Osal_fftcDeleteSem
 *
 *  @b  brief
 *  @n  This API deletes a semaphore created earlier using 
 *      @a Osal_fftcCreateSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Osal_fftcCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  Void
 * =============================================================================
 */
Void Osal_fftcDeleteSem (Void*  hSem)
{
    Semaphore_delete ((Semaphore_Handle *)&hSem);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_fftcPendSem
 *
 *  @b  brief
 *  @n  This API acquires a software semaphore created earlier using 
 *      @a Osal_fftcCreateSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Osal_fftcCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  Void
 * =============================================================================
 */
Void Osal_fftcPendSem (Void*  hSem)
{
    Semaphore_pend (hSem, BIOS_WAIT_FOREVER);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_fftcPostSem
 *
 *  @b  brief
 *  @n  This API releases a semaphore acquired earlier using @a 
 *      Osal_fftcPendSem () API. 
 *
 *  @param[in]  
 *      hSem        -   Semaphore handle obtained using @a Osal_fftcCreateSem ()
 *                      API.
 *
 *  @return     
 *  @n  Void
 * =============================================================================
 */
Void Osal_fftcPostSem (Void*  hSem)
{
    Semaphore_post (hSem);

    return;
}

/* ============================================================================
 *  @n@b Osal_fftcBeginMemAccess
 *
 *  @b  brief
 *  @n  This function invalidates the cached copy of the memory block being
 *      accessed, so as to ensure that any reads to it result in valid data
 *      fetches from the actual physical memory.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be read
 *
 *  @param[in]  size
 *       Size of the block to be read

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
Void Osal_fftcBeginMemAccess (Void *blockPtr, UInt32 size)
{
    /* Recommended sequence for cache operations is:
     *  1) Disable all interrupts
     *  2) Perform the cache block operation
     *  3) Wait until the cache operation is done either by polling 
     *     the corresponding WC register or using _mfence () 
     *     instruction.
     *  4) Enable interrupts back.
     */
    /* Disable all interrupts */        
    Osal_fftcInterruptCsEnter ();  
        
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_invL1d (blockPtr, size, CACHE_FENCE_WAIT);
    
    /* Invalidate the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    

    /* Enable back interrupts */
    Osal_fftcInterruptCsExit ();

    return;
}

/* ============================================================================
 *  @n@b Osal_fftcEndMemAccess
 *
 *  @b  brief
 *  @n  This function issues a writeback operation to ensure that the contents
 *      of cached copy of the memory block are updated in the actual physical 
 *      memory too.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be read
 *
 *  @param[in]  size
 *       Size of the block to be read

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
Void Osal_fftcEndMemAccess (Void *blockPtr, UInt32 size)
{
    /* Recommended sequence for cache operations is:
     *  1) Disable all interrupts
     *  2) Perform the cache block operation
     *  3) Wait until the cache operation is done either by polling 
     *     the corresponding WC register or using _mfence () 
     *     instruction.
     *  4) Enable interrupts back.
     */
    /* Disable all interrupts */        
    Osal_fftcInterruptCsEnter ();
        
    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_wbL1d (blockPtr, size, CACHE_FENCE_WAIT);

    /* Enable back interrupts */
    Osal_fftcInterruptCsExit ();
    
    return;
}

/* ============================================================================
 *  @n@b Osal_fftcBeginDataBufMemAccess
 *
 *  @b  brief
 *  @n  This function invalidates the cached copy of the data buffer memory block 
 *      being accessed, so as to ensure that any reads to it result in valid data
 *      fetches from the actual physical memory.
 *
 *  @param[in]  dataBufPtr
 *       Address of the data buffer block which is to be read
 *
 *  @param[in]  size
 *       Size of the block to be read

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
Void Osal_fftcBeginDataBufMemAccess (Void *dataBufPtr, UInt32 size)
{
    /* The example program allocates data buffers by default from LL2 and 
     * L2 caches are not enabled by default. Hence, no cache synchronization is needed here. */
    return;
}

/* ============================================================================
 *  @n@b Osal_fftcEndDataBufMemAccess
 *
 *  @b  brief
 *  @n  This function issues a writeback operation to ensure that the contents
 *      of cached copy of the data buffer memory block are updated in the actual 
 *      physical memory too.
 *
 *  @param[in]  dataBufPtr
 *       Address of the data buffer block which is to be read
 *
 *  @param[in]  size
 *       Size of the block to be read

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
Void Osal_fftcEndDataBufMemAccess (Void *dataBufPtr, UInt32 size)
{
    /* The example program allocates data buffers by default from LL2 and 
     * L2 caches are not enabled by default. Hence, no cache synchronization is needed here. */
    return;
}

/* ============================================================================
 *  @n@b Osal_fftcBeginDescMemAccess
 *
 *  @b  brief
 *  @n  This function invalidates the cached copy of the descriptor 
 *      being accessed, so as to ensure that any reads to it result in valid data
 *      fetches from the actual physical memory.
 *
 *  @param[in]  hRx
 *       Rx object handle to identify the size of descriptor correctly if descriptors
 *       are application managed for this Rx object.
 *
 *  @param[in]  descPtr
 *       Descriptor address which is to be read
 *
 *  @param[in]  size
 *       Size of the descriptor to be read

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
Void Osal_fftcBeginDescMemAccess (Void* hRx, Void *descPtr, UInt32 size)
{
    /* Descriptors in the example program are allocated by default from LL2 and the 
     * L2 cache is disabled by default. Hence, no cache synchronization is needed
     * here. This hook needs a valid implementation if descriptors are placed in
     * cacheable memory region. */
    return;
}

/* ============================================================================
 *  @n@b Osal_fftcEndDescMemAccess
 *
 *  @b  brief
 *  @n  This function issues a writeback operation to ensure that the contents
 *      of cached copy of the descriptor are updated in the actual physical memory 
 *      too.
 *
 *  @param[in]  descPtr
 *       Descriptor address thats been updated
 *
 *  @param[in]  size
 *       Size of the descriptor to be written back

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
Void Osal_fftcEndDescMemAccess (Void *descPtr, UInt32 size)
{
    /* Descriptors in the example program are allocated by default from LL2 and the 
     * L2 cache is disabled by default. Hence, no cache synchronization is needed
     * here. This hook needs a valid implementation if descriptors are placed in
     * cacheable memory region. */
    return;
}

/**********************************************************************
 *********************** CPPI OSAL Functions **************************
 **********************************************************************/

/**
 * ============================================================================
 *  @n@b Osal_cppiCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization to the caller.
 *
 *      This is a BLOCKING API.
 *
 *      This API ensures multi-core synchronization between
 *      multiple processes trying to access CPPI shared
 *      library at the same time.
 *
 *  @param[in]  
 *  @n  None
 *
 *  @return     
 *  @n  Handle used to lock critical section
 * =============================================================================
 */
Ptr Osal_cppiCsEnter (Void)
{
    /* Get the hardware semaphore. 
     *
     * Acquire Multi core CPPI synchronization lock 
     */
    while ((CSL_semAcquireDirect (CPPI_HW_SEM)) == 0);

    /* Disable all interrupts and OS scheduler. 
     *
     * Acquire Multi threaded / process synchronization lock.
     */
    coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] = Hwi_disable();

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_cppiCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_cppiCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab CPPI access.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_cppiCsExit (Ptr CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    Hwi_restore(coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)]);

    /* Release the hardware semaphore 
     *
     * Release multi-core lock.
     */ 
    CSL_semReleaseSemaphore (CPPI_HW_SEM);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_cppiMalloc
 *
 *  @b  brief
 *  @n  This API allocates a memory block of a given
 *      size specified by input parameter 'num_bytes'.
 *
 *      This API should allocate memory from shared memory if the test applications
 *      are to be run on multiple cores.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @return
 *      Allocated block address
 * =============================================================================
 */
Ptr Osal_cppiMalloc (UInt32 num_bytes)
{
	Error_Block	    errorBlock;

    /* Increment the allocation counter. */
    fftcCppiMallocCounter++;	

	/* Allocate memory.  */
	return Memory_alloc((xdc_runtime_IHeap_Handle) SharedRegion_getHeap(0), num_bytes, 0, &errorBlock);
}

/**
 * ============================================================================
 *  @n@b Osal_cppiFree
 *
 *  @b  brief
 *  @n  This API frees and restores a given memory location 
 *      pointer 'dataPtr' of size 'num_bytes' to its
 *      original heap location. Frees up memory allocated using 
 *      @a Osal_cppiMalloc ()
 *
 *  @param[in]  dataPtr
 *      Pointer to the memory block to be cleaned up.
 *
 *  @param[in]  num_bytes
 *      Size of the memory block to be cleaned up.
 *
 *  @return
 *      Not Applicable
 * =============================================================================
 */
Void Osal_cppiFree (Ptr dataPtr, UInt32 num_bytes)
{
    /* Increment the free counter. */
    fftcCppiFreeCounter++;	

    /* Free up the memory */
    if (dataPtr)
    {
        /* Convert the global address to local address since
         * thats what the heap understands.
         */
        Memory_free ((xdc_runtime_IHeap_Handle) SharedRegion_getHeap(0), dataPtr, num_bytes);
    }
}

/**
 * ============================================================================
 *  @n@b Osal_cppiBeginMemAccess
 *
 *  @b  brief
 *  @n  The function is used to indicate that a block of memory is 
 *      about to be accessed. If the memory block is cached then this 
 *      indicates that the application would need to ensure that the 
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *
 *  @param[in]  size
 *       Size of memory block

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
void Osal_cppiBeginMemAccess (void *ptr, uint32_t size)
{
    /* Recommended sequence for cache operations is:
     *  1) Disable all interrupts
     *  2) Perform the cache block operation
     *  3) Wait until the cache operation is done either by polling 
     *     the corresponding WC register or using _mfence () 
     *     instruction.
     *  4) Enable interrupts back.
     */
    /* Disable all interrupts */        
    Osal_fftcInterruptCsEnter ();

    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
    
    /* Invalidate the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    

    /* Enable back interrupts */
    Osal_fftcInterruptCsExit ();

    return;
}


/**
 * ============================================================================
 *  @n@b Osal_cppiEndMemAccess
 *
 *  @b  brief
 *  @n  The function is used to indicate that the block of memory has 
 *      finished being accessed. If the memory block is cached then the 
 *      application would need to ensure that the contents of the cache 
 *      are updated immediately to the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *
 *  @param[in]  size
 *       Size of memory block

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
void Osal_cppiEndMemAccess (void *ptr, uint32_t size)
{
    /* Recommended sequence for cache operations is:
     *  1) Disable all interrupts
     *  2) Perform the cache block operation
     *  3) Wait until the cache operation is done either by polling 
     *     the corresponding WC register or using _mfence () 
     *     instruction.
     *  4) Enable interrupts back.
     */
    /* Disable all interrupts */        
    Osal_fftcInterruptCsEnter ();

    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Enable back interrupts */
    Osal_fftcInterruptCsExit ();
    
    return;
}

/**********************************************************************
 *********************** QMSS OSAL Functions **************************
 **********************************************************************/

/**
 * ============================================================================
 *  @n@b Osal_qmssCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization to the caller.
 *
 *      This is a BLOCKING API.
 *
 *      This API ensures multi-core synchronization between
 *      multiple processes trying to access QMSS shared
 *      library at the same time.
 *
 *  @param[in]  None
 *
 *  @return     
 *      Handle used to lock critical section
 * =============================================================================
 */
Ptr Osal_qmssCsEnter (Void)
{
    /* Get the hardware semaphore. 
     *
     * Acquire Multi core QMSS synchronization lock 
     */
    while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);

    /* Disable all interrupts and OS scheduler. 
     *
     * Acquire Multi threaded / process synchronization lock.
     */
    coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] = Hwi_disable();

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_fftcQmssCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab QMSS access.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_qmssCsExit (Ptr CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    Hwi_restore(coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)]);

    /* Release the hardware semaphore 
     *
     * Release multi-core lock.
     */ 
    CSL_semReleaseSemaphore (QMSS_HW_SEM);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssAccCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization to the caller.
 *
 *      This is a BLOCKING API.
 *
 *      This API ensures multi-core synchronization between
 *      multiple processes trying to access QMSS shared
 *      library at the same time.
 *
 *  @param[in]  None
 *
 *  @return     
 *      Handle used to lock critical section
 * =============================================================================
 */
Void* Osal_qmssAccCsEnter (Void)
{
    /* Get the hardware semaphore. 
     *
     * Acquire Multi core QMSS synchronization lock 
     */
    while ((CSL_semAcquireDirect (QMSS_ACC_HW_SEM)) == 0);

    /* Disable all interrupts and OS scheduler. 
     *
     * Acquire Multi threaded / process synchronization lock.
     */
    coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] = Hwi_disable();

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssAccCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_qmssAccCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab QMSS access.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_qmssAccCsExit (Void *CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    Hwi_restore(coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)]);

    /* Release the hardware semaphore 
     *
     * Release multi-core lock.
     */ 
    CSL_semReleaseSemaphore (QMSS_ACC_HW_SEM);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssMtCsEnter
 *
 *  @b  brief
 *  @n  This API ensures ONLY multi-threaded
 *      synchronization to the QMSS user.
 *
 *      This is a BLOCKING API.
 *
 *  @param[in] None
 *
 *  @return     
 *       Handle used to lock critical section
 * =============================================================================
 */
Ptr Osal_qmssMtCsEnter (Void)
{
    /* Disable all interrupts and OS scheduler. 
     *
     * Acquire Multi threaded / process synchronization lock.
     */
    //coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] = Hwi_disable();

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssMtCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_fftcQmssMtCsEnter ()
 *      API. It resets the multi-threaded lock, enabling another process
 *      on the current core to grab it.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_qmssMtCsExit (Ptr CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    //Hwi_restore(key);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssMalloc
 *
 *  @b  brief
 *  @n  This API allocates a memory block of a given
 *      size specified by input parameter 'num_bytes'.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @return
 *      Allocated block address
 * =============================================================================
 */
Ptr Osal_qmssMalloc (UInt32 num_bytes)
{
	Error_Block	    errorBlock;

    /* Increment the allocation counter. */
    fftcQmssMallocCounter++;	

	/* Allocate memory.  */
	return Memory_alloc(NULL, num_bytes, 0, &errorBlock);
}

/**
 * ============================================================================
 *  @n@b Osal_qmssFree
 *
 *  @b  brief
 *  @n  This API frees and restores a given memory location 
 *      pointer 'dataPtr' of size 'num_bytes' to its
 *      original heap location. Frees up memory allocated using 
 *      @a Osal_qmssMalloc ()
 *
 *  @param[in]  dataPtr
 *      Pointer to the memory block to be cleaned up.
 *
 *  @param[in]  num_bytes
 *      Size of the memory block to be cleaned up.
 *
 *  @return
 *      Not Applicable
 * =============================================================================
 */
Void Osal_qmssFree (Ptr dataPtr, UInt32 num_bytes)
{
    /* Increment the free counter. */
    fftcQmssFreeCounter++;	

    /* Free up the memory */
    if (dataPtr)
    {
        /* Convert the global address to local address since
         * thats what the heap understands.
         */
        Memory_free(NULL, dataPtr, num_bytes);
    }
}

/* ============================================================================
 *  @n@b Osal_qmssBeginMemAccess
 *
 *  @b  brief
 *  @n  The function is used to indicate that a block of memory is 
 *      about to be accessed. If the memory block is cached then this 
 *      indicates that the application would need to ensure that the 
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *
 *  @param[in]  size
 *       Size of memory block

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
void Osal_qmssBeginMemAccess (void *ptr, uint32_t size)
{
    /* Recommended sequence for cache operations is:
     *  1) Disable all interrupts
     *  2) Perform the cache block operation
     *  3) Wait until the cache operation is done either by polling 
     *     the corresponding WC register or using _mfence () 
     *     instruction.
     *  4) Enable interrupts back.
     */
    /* Disable all interrupts */        
    Osal_fftcInterruptCsEnter ();
        
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
    
    /* Invalidate the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    

    /* Enable back interrupts */
    Osal_fftcInterruptCsExit ();

    return;
}

/* ============================================================================
 *  @n@b Osal_qmssEndMemAccess
 *
 *  @b  brief
 *  @n  The function is used to indicate that the block of memory has 
 *      finished being accessed. If the memory block is cached then the 
 *      application would need to ensure that the contents of the cache 
 *      are updated immediately to the actual memory..
 *
 *  @param[in]  ptr
 *       Address of memory block
 *
 *  @param[in]  size
 *       Size of memory block

 *  @retval
 *      Not Applicable
 * =============================================================================
 */
void Osal_qmssEndMemAccess (void *ptr, uint32_t size)
{
    /* Recommended sequence for cache operations is:
     *  1) Disable all interrupts
     *  2) Perform the cache block operation
     *  3) Wait until the cache operation is done either by polling 
     *     the corresponding WC register or using _mfence () 
     *     instruction.
     *  4) Enable interrupts back.
     */
    /* Disable all interrupts */        
    Osal_fftcInterruptCsEnter ();

    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);
    
    /* Enable back interrupts */
    Osal_fftcInterruptCsExit ();

    return;
}
