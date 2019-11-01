/**
 *   @file  test_osal.c
 *
 *   @brief   
 *      This is a sample OS Abstraction Layer (AL) file implemented
 *      using XDC/BIOS APIs.
 *
 *      System integrator is advised to review these implementations and
 *      modify them to suit it to application requirements.
 *
 *      This OSAL implementation uses the <b> Approach 2 </b> documented.
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

/* BCP Test application OSAL include */
#include <bcp_osal.h>

/**********************************************************************
 ****************************** Defines *******************************
 **********************************************************************/

/* Number of cores on c6498 */
#define     NUM_CORES           4

/* Hardware Semaphore to synchronize access from
 * multiple BCP applications across different cores to
 * the BCP driver.
 */
#define     BCP_HW_SEM          2 

/* Hardware Semaphore to synchronize access from
 * multiple applications (BCP applications and non-BCP applications)
 * across different cores to the QMSS library.
 */
#define     QMSS_HW_SEM         3 

/* Hardware Semaphore to synchronize access from
 * multiple applications (BCP applications and non-BCP applications)
 * across different cores to the CPPI library.
 */
#define     CPPI_HW_SEM         4 

/* Hardware Semaphore to synchronize access from
 * multiple applications (BCP applications and non-BCP applications)
 * across different cores to the SRIO library.
 */
#define     SRIO_HW_SEM         5

/* Hardware Semaphore to synchronize access from
 * multiple applications (BCP applications and non-BCP applications)
 * for accumulator queues across different cores to the QMSS library.
 */
#define     QMSS_ACC_HW_SEM     6
 
/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
UInt32      bcpMallocCounter       =   0;
UInt32      bcpFreeCounter         =   0;
UInt32      bcpCppiMallocCounter   =   0;
UInt32      bcpCppiFreeCounter     =   0;
UInt32      bcpQmssMallocCounter   =   0;
UInt32      bcpQmssFreeCounter     =   0;
UInt32      bcpSrioMallocCounter   =   0;
UInt32      bcpSrioFreeCounter     =   0;

UInt32      coreKey [NUM_CORES];

#undef		BCP_TEST_DEBUG

/**********************************************************************
 *********************** BCP OSAL Functions **************************
 **********************************************************************/

/**
 * ============================================================================
 *  @n@b Osal_bcpLocal2Global
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
static UInt32 Osal_bcpLocal2Global (UInt32 addr)
{
    UInt32 corenum;

    /* Get the core number. */
    corenum = CSL_chipReadDNUM (); 

    /* Compute the global address. */
    return ((1 << 28) | (corenum << 24) | (addr & 0x00ffffff));
}

/**
 * ============================================================================
 *  @n@b Osal_bcpGlobal2Local
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
static UInt32 Osal_bcpGlobal2Local (UInt32 gaddr)
{
    UInt32 corenum;

    /* Get the core number. */
    corenum = CSL_chipReadDNUM ();

    /* Compute the global address. */
    return (gaddr & ~((1 << 28) | (corenum << 24)));    
}

/**
 * ============================================================================
 *  @n@b Osal_biosMalloc
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
Void* Osal_biosMalloc (UInt32 num_bytes, Bool bGlobalAddress)
{
	Error_Block	    errorBlock;
    Void*           destPtr;

    /* Increment the allocation counter. */
    bcpMallocCounter++;	

	/* Allocate memory from the heap */
	if (destPtr = Memory_alloc(NULL, num_bytes, 0, &errorBlock))
    {
        /* Convert the core local address obtained from
         * Memory_alloc API to a Global Address. The 
         * CPPI/QMSS libraries cannot handle buffers and
         * descriptors that are core local addresses.
         */
#ifdef BCP_TEST_DEBUG
		Bcp_osalLog ("bcpMalloc Alloc DataP: %p GlobalDataP: %p size: %d \n", destPtr, Osal_bcpLocal2Global ((UInt32) destPtr), num_bytes);         
#endif
        if (bGlobalAddress)
            return ((Ptr) Osal_bcpLocal2Global ((UInt32) destPtr));
        else
            return ((Ptr) destPtr);
    }
    else
        return destPtr;
}

/**
 * ============================================================================
 *  @n@b Osal_biosFree
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
Void Osal_biosFree (Void* dataPtr, UInt32 num_bytes, Bool bGlobalAddress)
{
    /* Increment the free counter. */
    bcpFreeCounter++;	

    /* Free up the memory */
    if (dataPtr)
    {
        /* Convert the global address to local address since
         * thats what the heap understands.
         */
#ifdef BCP_TEST_DEBUG
         Bcp_osalLog ("BcpFree DataP: %p GDP: %p size: %d\n", dataPtr, (UInt32)Osal_bcpGlobal2Local ((UInt32) dataPtr), num_bytes);
#endif
         if (bGlobalAddress)
            Memory_free(NULL, (Ptr) Osal_bcpGlobal2Local ((UInt32) dataPtr), num_bytes);
         else
            Memory_free(NULL, (Ptr) dataPtr, num_bytes);
    }
}

/**
 * ============================================================================
 *  @n@b Osal_biosMultiCoreCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core synchronization to the caller.
 *
 *      This is a BLOCKING API.
 *
 *      This API ensures multi-core synchronization between
 *      multiple processes trying to access BCP shared
 *      library at the same time.
 *
 *  @param[in]  None
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_biosMultiCoreCsEnter ()
{
    /* Get the hardware semaphore. 
     *
     * Acquire Multi core synchronization lock 
     */
    while ((CSL_semAcquireDirect (BCP_HW_SEM)) == 0);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_biosMultiCoreCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_biosMultiCoreCsEnter ()
 *      API. It resets the multi-core lock, enabling another process/core 
 *      to grab it.
 *
 *  @param[in]  None
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_biosMultiCoreCsExit ()
{
    /* Release the hardware semaphore 
     *
     * Release multi-core lock.
     */ 
    CSL_semReleaseSemaphore (BCP_HW_SEM);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_biosInterruptCsEnter
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
Void Osal_biosInterruptCsEnter ()
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
 *  @n@b Osal_biosInterruptCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_biosInterruptCsEnter ()
 *      API. It restores the saved interrupt context and enables back the
 *      interrupts.
 *
 *  @param[in]  None
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_biosInterruptCsExit ()
{
    /* Enable all interrupts.
     *
     * Release interrupt lock.
     */
    Hwi_restore(coreKey [CSL_chipReadDNUM ()]);

    return;
}

/* ============================================================================
 *  @n@b Osal_bcpBeginMemAccess
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
Void Osal_bcpBeginMemAccess (Void *blockPtr, UInt32 size)
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
    Osal_biosInterruptCsEnter ();  
        
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_invL1d (blockPtr, size, CACHE_FENCE_WAIT);
    
    /* Invalidate the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    

    /* Enable back interrupts */
    Osal_biosInterruptCsExit ();

    return;
}

/* ============================================================================
 *  @n@b Osal_bcpEndMemAccess
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
Void Osal_bcpEndMemAccess (Void *blockPtr, UInt32 size)
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
    Osal_biosInterruptCsEnter ();
        
    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_wbL1d (blockPtr, size, CACHE_FENCE_WAIT);

    /* Enable back interrupts */
    Osal_biosInterruptCsExit ();

    return;
}

/* ============================================================================
 *  @n@b Osal_bcpBeginDescMemAccess
 *
 *  @b  brief
 *  @n  This function invalidates the cached copy of the descriptor 
 *      being accessed, so as to ensure that any reads to it result in valid data
 *      fetches from the actual physical memory.
 *
 *  @param[in]  hRx
 *       Rx object handle to identify the size of descriptor correctly since 
 *       descriptors are allocated and managed entirely by the application.
 *
 *  @param[in]  descPtr
 *       Descriptor address which is to be read
 *
 *  @retval
 *      Not Applicable
 * =============================================================================
 */
Void Osal_bcpBeginDescMemAccess (Void* hRx, Void *descPtr)
{
    /* Descriptors in the test program are allocated by default from LL2 and the 
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
Void* Osal_cppiCsEnter (Void)
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
    coreKey [CSL_chipReadDNUM ()] = Hwi_disable();

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
Void Osal_cppiCsExit (Void* CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    Hwi_restore(coreKey [CSL_chipReadDNUM ()]);

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
Void* Osal_cppiMalloc (UInt32 num_bytes)
{
	Error_Block	    errorBlock;
    Void*           dataPtr;

    /* Increment the allocation counter. */
    bcpCppiMallocCounter++;	

    /* Allocate a buffer from the default HeapMemMp */
    if (SharedRegion_getHeap(0) != NULL)
    {
        dataPtr = Memory_alloc ((xdc_runtime_IHeap_Handle) SharedRegion_getHeap(0), num_bytes, 0, &errorBlock);
    }
    else
    {
#ifdef BCP_TEST_DEBUG    	
		Bcp_osalLog ("CppiAlloc Failed for size: %d \n", num_bytes);  
#endif 	
       	return NULL;
    }

#ifdef BCP_TEST_DEBUG
    Bcp_osalLog ("CppiAlloc DataP: %p size: %d \n", dataPtr, num_bytes);
#endif        

    return dataPtr;
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
Void Osal_cppiFree (Void* dataPtr, UInt32 num_bytes)
{
    /* Increment the free counter. */
    bcpCppiFreeCounter++;	

    /* Free up the memory */
    if (dataPtr)
    {
#ifdef BCP_TEST_DEBUG         
        Bcp_osalLog ("CppiFree: DataP: %p size: %d\n", dataPtr, num_bytes);
#endif
        Memory_free ((xdc_runtime_IHeap_Handle) SharedRegion_getHeap(0), dataPtr, num_bytes);
    }
}

#if 0
Void Osal_cppiLog( String fmt, ... ) 
{
}
#endif

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
    Osal_biosInterruptCsEnter ();  
        
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
    
    /* Invalidate the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    

    /* Enable back interrupts */
    Osal_biosInterruptCsExit ();

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
    Osal_biosInterruptCsEnter ();
        
    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Enable back interrupts */
    Osal_biosInterruptCsExit ();

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
Void* Osal_qmssCsEnter (Void)
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
    coreKey [CSL_chipReadDNUM ()] = Hwi_disable();

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_bcpQmssCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab QMSS access.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_qmssCsExit (Void* CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    Hwi_restore(coreKey [CSL_chipReadDNUM ()]);

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
    coreKey [CSL_chipReadDNUM ()] = Hwi_disable();

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
    Hwi_restore(coreKey [CSL_chipReadDNUM ()]);

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
Void* Osal_qmssMtCsEnter (Void)
{
    /* Disable all interrupts and OS scheduler. 
     *
     * Acquire Multi threaded / process synchronization lock.
     */
    //coreKey [CSL_chipReadDNUM ()] = Hwi_disable();

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssMtCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_bcpQmssMtCsEnter ()
 *      API. It resets the multi-threaded lock, enabling another process
 *      on the current core to grab it.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
Void Osal_qmssMtCsExit (Void* CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    //Hwi_restore(coreKey [CSL_chipReadDNUM ()]);

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
Void* Osal_qmssMalloc (UInt32 num_bytes)
{
	Error_Block	    errorBlock;
    Void*           dataPtr;

    /* Increment the allocation counter. */
    bcpQmssMallocCounter++;	

	/* Allocate memory.  */
	dataPtr =  Memory_alloc(NULL, num_bytes, 0, &errorBlock);

#ifdef BCP_TEST_DEBUG
    Bcp_osalLog ("QmssAlloc DataP: %p size: %d \n", dataPtr, num_bytes);
#endif

    return dataPtr;
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
Void Osal_qmssFree (Void* dataPtr, UInt32 num_bytes)
{
    /* Increment the free counter. */
    bcpQmssFreeCounter++;	

    /* Free up the memory */
    if (dataPtr)
    {
        /* Convert the global address to local address since
         * thats what the heap understands.
         */
#ifdef BCP_TEST_DEBUG
        Bcp_osalLog ("QmssFree DataP: %p size: %d\n", dataPtr, num_bytes);
#endif

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
    Osal_biosInterruptCsEnter ();  
        
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
    
    /* Invalidate the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    

    /* Enable back interrupts */
    Osal_biosInterruptCsExit ();

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
    Osal_biosInterruptCsEnter ();
        
    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled 
     */    
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Enable back interrupts */
    Osal_biosInterruptCsExit ();

    return;
}

