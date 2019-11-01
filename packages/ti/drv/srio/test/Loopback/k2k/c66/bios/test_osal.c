/**
 *   @file  test_osal.c
 *
 *   @brief   
 *      This is the OS abstraction layer and is used by the SRIO, CPPI and QMSS
 *      drivers for the SRIO Unit Test Application.
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
 *  \par
*/
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/hal/Hwi.h>
#include <srio_osal.h>
#include <xdc/cfg/global.h>

/* CSL Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/

#define CPPI_HW_SEM             1
#define QMSS_HW_SEM             2
#define SRIO_HW_SEM             3

#define MAX_MEM_MGR_ENTRIES     30

typedef struct MEM_MGMT_ENTRY
{
    uint8_t*    ptrMemory;
    uint32_t    isFree;
}MEM_MGMT_ENTRY;

MEM_MGMT_ENTRY     gDataBufferMemMgr[MAX_MEM_MGR_ENTRIES];
int32_t            gDataBufferMemMgrMaxSize = 0;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
UInt32 malloc_counter = 0;
UInt32 free_counter   = 0;

/**********************************************************************
 ************************** Extern Definitions ************************
 **********************************************************************/

extern Srio_DrvHandle  hAppManagedSrioDrv;
extern Srio_DrvHandle  hDrvManagedSrioDrv;

/**********************************************************************
 **************************** OSAL Functions **************************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      Utility function which converts a local address to global.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Global Address
 */
static UInt32 Osal_local2Global (UInt32 addr)
{
	UInt32 corenum;

	/* Get the core number. */
	corenum = CSL_chipReadReg(CSL_CHIP_DNUM); 

	/* Compute the global address. */
	return (addr + (0x10000000 + (corenum*0x1000000)));
}

/**
 *  @b Description
 *  @n  
 *      The function is used to allocate a memory block of the specified size.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Allocated block address
 */
Void* biosMalloc (UInt32 num_bytes)
{
	Error_Block	errorBlock;
    UInt32*     ptr;
    
    /* Increment the allocation counter. */
    malloc_counter++;

    /* Allocate memory */
    ptr = (UInt32*)Memory_alloc(NULL, num_bytes, 16, &errorBlock);
    if (ptr == NULL)
        return NULL;

    /* Return the allocated memory block. */
	return ptr;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to clean up a specific memory block and is called
 *      from the SRIO Driver. 
 *
 *  @param[in]  ptr
 *      Pointer to the memory block to be cleaned up.
 *  @param[in]  numBytes
 *      Size of the memory block being cleaned up.
 *
 *  @retval
 *      Not Applicable
 */
Void biosFree (Void* ptr, UInt32 numBytes)
{
    /* Increment the free counter. */
    free_counter++;	
    Memory_free(NULL, ptr, numBytes);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to allocate a block of memory for all the data buffer
 *      operations. This function is called by the application.
 *
 *  @param[in]  numBuffers
 *      Number of data buffers
 *  @param[in]  dataBufferSize
 *      Size of each data buffer
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize)
{
	Error_Block	errorBlock;
    uint8_t*    ptrMemory;
    uint32_t    index;

    /* Allocate memory for all the data buffers */
    ptrMemory = (uint8_t*)Memory_alloc(NULL, MAX_MEM_MGR_ENTRIES*dataBufferSize, 16, &errorBlock);
    if (ptrMemory == NULL)
        return -1;

    /* Convert to a global address */
    ptrMemory = (Void*)Osal_local2Global((UInt32)ptrMemory);
 
    /* Now we chop up the memory and add it to the memory manager. */
    for (index = 0; index < MAX_MEM_MGR_ENTRIES; index++)
    {
        /* Populate the data memory management entry. */
        gDataBufferMemMgr[index].isFree    = 1;
        gDataBufferMemMgr[index].ptrMemory = ptrMemory;

        /* Increment the memory to the next address */
        ptrMemory = ptrMemory + dataBufferSize;
    }

    /* Remember the memory buffer size */
    gDataBufferMemMgrMaxSize = dataBufferSize;

    /* Memory Manager has been created. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to allocate a memory block of the specified size.
 *      This is used to allocate the data buffers which are attached to the
 *      descriptors in the driver managed configuration and we need to ensure
 *      that the buffers returned here are always in the global address space.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Allocated block address
 */
Void* Osal_DataBufferMalloc (UInt32 num_bytes)
{
    uint32_t    index;
    void*       ptrMemory = NULL;

    /* Basic Validation: Ensure that the memory size requested is within range. */
    if (num_bytes > gDataBufferMemMgrMaxSize)
        return NULL;

    /* Increment the allocation counter. */
    malloc_counter++;

    /* Lock out interrupts */
    Hwi_disable();

    /* Cycle through for a free entry. */
    for (index = 0; index < MAX_MEM_MGR_ENTRIES; index++)
    {
        /* Check if the entry is free or not? */
        if (gDataBufferMemMgr[index].isFree == 1) 
        {
            /* Entry was free. We can use it. */
            ptrMemory = gDataBufferMemMgr[index].ptrMemory;

            /* Mark the entry as used. */
            gDataBufferMemMgr[index].isFree = 0;

            /* We have found a match. */
            break;
        }
    }

    /* Unlock interrupts. */
    Hwi_enable();

    /* Return the allocated memory. */
    return ptrMemory;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to clean a previouslt allocated data buffer.
 *
 *  @param[in]  ptr
 *      Pointer to the memory block to be cleaned up.
 *  @param[in]  numBytes
 *      Size of the memory block being cleaned up.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_DataBufferFree (Void* ptr, UInt32 numBytes)
{
    uint32_t    index;

    /* Increment the free counter. */
    free_counter++;	

    /* Lock out interrupts */
    Hwi_disable();

    /* Cycle through and clean up */
    for (index = 0; index < MAX_MEM_MGR_ENTRIES; index++)
    {
        /* Check if the entry is free or not? */
        if (gDataBufferMemMgr[index].ptrMemory == (uint8_t*)ptr) 
        {
            /* Mark the entry as free. */
            gDataBufferMemMgr[index].isFree = 1;

            /* We have found a match. */
            break;
        }
    }

    /* Unlock interrupts. */
    Hwi_enable();
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to provide critical section to prevent access of shared
 *      resources from multiple cores
 *
 *  @retval
 *      NULL
 */
Void* Osal_MultiCoreEnter(Void)
{
    /* Get the hardware semaphore */
    while ((CSL_semAcquireDirect (SRIO_HW_SEM)) == 0);
    return NULL;
}

/**
 *  @b Description
 *  @n  
 *      The function is called to end the critical section access of shared resources
 *      from multiple cores.
 *
 *  @param[in]  critSectHandle
 *      Semaphore opaque handle.
 *
 *  @retval
 *      None
 */
Void  Osal_MultiCoreExit(Void* critSectHandle)
{
    /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (SRIO_HW_SEM);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to provide critical section to prevent access of shared
 *      resources from single core and multiple threads.  
 *
 *  @param[in]  drvHandle
 *      Driver Handle which needs critical section to protect its resources.
 *
 *  @retval
 *      Opaque handle
 */
Void* Osal_SingleCoreEnter(Srio_DrvHandle drvHandle)
{
    /* In the Test code; we have 2 driver instances 
     *  - Driver Managed 
     *      This configuration uses interrupts and calls the SRIO Driver API's
     *      from 2 contexts (Thread and ISR)
     *  - Application Managed 
     *      This configuration uses polled mode only and thus there is only 
     *      one context from which the SRIO Driver API's are called from */
    if (drvHandle == hDrvManagedSrioDrv)
    {
        /* Driver Managed Configuration: We need to disable interrupts. */
        return (Void*)Hwi_disable();
    }
    else
    {
        /* Application Managed Configuration: No need for any protection */
        return NULL;
    }
}

/**
 *  @b Description
 *  @n  
 *      The function is called to end the critical section access of shared resources
 *      from single cores.
 *
 *  @param[in]  drvHandle
 *      Driver Handle which needed critical section to protect its resources.
 *  @param[in]  critSectHandle
 *      Opaque handle retreived when the Single Core Protection Enter API was called
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_SingleCoreExit(Srio_DrvHandle drvHandle, Void* critSectHandle)
{
    /* In the Test code; we have 2 driver instances 
     *  - Driver Managed 
     *      This configuration uses interrupts and calls the SRIO Driver API's
     *      from 2 contexts (Thread and ISR)
     *  - Application Managed 
     *      This configuration uses polled mode only and thus there is only 
     *      one context from which the SRIO Driver API's are called from */
    if (drvHandle == hDrvManagedSrioDrv)
    {
        /* Driver Managed Configuration: We need to enable interrupts. */
        Hwi_enable();
    }
    else
    {
        /* Application Managed Configuration: This is a NOP. */
        return; 
    }
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the SRIO driver to invalidate the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      None
 */
void Osal_CacheInvalidate(void* ptr, uint32_t size)
{
#if 0
    CACHE_invL1d (ptr, size, CACHE_WAIT);

    /*  Cleanup the prefectch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();
#else
    UInt  key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

    /* Invalidate the cache. */
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Reenable Interrupts. */
    Hwi_restore(key);
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the SRIO driver to writeback the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      None
 */
void Osal_CacheWriteback(void* ptr, uint32_t size)
{
#if 0        
    CACHE_wbL1d (ptr, size, CACHE_WAIT);
    _mfence();
#else
    UInt  key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Writeback the cache. */
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Reenable Interrupts. */
    Hwi_restore(key);
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is invoked by the SRIO Driver to indicate that a
 *      descriptor is being accessed. 
 *
 *  @param[in]  drvHandle
 *      Driver Instance for which descriptor is being accessed.
 *  @param[in]  ptr
 *      Pointer to the descriptor being accessed
 *  @param[in]  size
 *      Size of the descriptor (Only valid for Driver Managed)
 *
 *  @retval
 *      None
 */
void Osal_srioBeginDescriptorAccess (Srio_DrvHandle drvHandle, void* ptr, uint32_t descSize)
{
    /* In the Test Application all descriptor are located in LL2 memory we dont need to
     * add any special cache invalidation hook here. However if the descriptors were located
     * in SL2 (MSMC) this would be required. */
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is invoked by the SRIO Driver to indicate that a
 *      descriptor is finished being accessed.
 *
 *  @param[in]  drvHandle
 *      Driver Instance for which descriptor is being accessed.
 *  @param[in]  ptr
 *      Pointer to the descriptor being accessed
 *  @param[in]  size
 *      Size of the descriptor (Only valid for Driver Managed)
 *
 *  @retval
 *      None
 */
void Osal_srioEndDescriptorAccess (Srio_DrvHandle drvHandle, void* ptr, uint32_t descSize)
{
    /* In the Test Application the descriptors are located in Core 0 Global address space
     * If they are accessed from other cores we need to ensure that there is an MFENCE so 
     * that all the access to the descriptors are complete before we start pushing them
     * out. */
    _mfence();
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to allocate a memory block of the specified size.
 *      from shared memory.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Allocated block address
 */
Ptr Osal_cppiMalloc (UInt32 num_bytes)
{
    Error_Block	errorBlock;

    /* Allocate a buffer from the default HeapMemMp */
    return Memory_alloc ((xdc_runtime_IHeap_Handle)  SharedRegion_getHeap(0), num_bytes, 0, &errorBlock);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to free a memory block of the specified size allocated 
 *      using Osal_cppiMalloc() API.
 *
 *  @param[in]  ptr
 *      Pointer to the memory block to be cleaned up.
 *  @param[in]  size
 *      Size of the memory block to be cleaned up.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_cppiFree (Ptr ptr, UInt32 size)
{
    Memory_free ((xdc_runtime_IHeap_Handle) SharedRegion_getHeap(0), ptr, size);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to enter a critical section.
 *      Function protects against 
 *      
 *      access from multiple cores 
 *      and 
 *      access from multiple threads on single core
 *
 *  @retval
 *      Handle used to lock critical section
 */
Ptr Osal_cppiCsEnter (Void)
{
    /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (CPPI_HW_SEM)) == 0);
    return NULL;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to exit a critical section 
 *      protected using Osal_cppiCsEnter() API.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_cppiCsExit (Ptr CsHandle)
{
    /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (CPPI_HW_SEM);
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to indicate that a block of memory is 
 *      about to be accessed. If the memory block is cached then this 
 *      indicates that the application would need to ensure that the 
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_cppiBeginMemAccess (void *ptr, uint32_t size)
{
#if 0        
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled */    
    CACHE_invL1d (ptr, size, CACHE_WAIT);        

    /*  Cleanup the prefectch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();
#else
    UInt  key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /*  Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

    /* Invalidate the cache. */
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Reenable Interrupts. */
    Hwi_restore(key);
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is used to indicate that the block of memory has 
 *      finished being accessed. If the memory block is cached then the 
 *      application would need to ensure that the contents of the cache 
 *      are updated immediately to the actual memory. 
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_cppiEndMemAccess (void *ptr, uint32_t size)
{
#if 0        
    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled */    
    CACHE_wbL1d (ptr, size, CACHE_WAIT);
    _mfence();
#else
    UInt  key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Writeback the contents of the cache. */
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Reenable Interrupts. */
    Hwi_restore(key);
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is used to enter a critical section.
 *      Function protects against 
 *      
 *      access from multiple cores 
 *      and 
 *      access from multiple threads on single core
 *
 *  @retval
 *      Handle used to lock critical section
 */
Ptr Osal_qmssCsEnter (Void)
{
    /* Get the hardware semaphore */
    while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);
    return NULL;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to exit a critical section 
 *      protected using Osal_qmssCsEnter() API.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_qmssCsExit (Ptr CsHandle)
{
    /* Release the hardware semaphore */ 
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
Ptr Osal_qmssAccCsEnter (Void)
{
    /* This is a suboptimal implementation for this OSAL, please refer to
	 * QMSS examples for optimal implementation of this function 
	 */

  return (Osal_qmssCsEnter());
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
Void Osal_qmssAccCsExit (Ptr CsHandle)
{
    /* This is a suboptimal implementation for this OSAL, please refer to
	 * QMSS examples for optimal implementation of this function 
	 */   
   Osal_qmssCsExit(CsHandle);
   return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to enter a critical section.
 *      Function protects against 
 *      access from multiple threads on single core
 *
 *  @retval
 *      Handle used to lock critical section
 */
Ptr Osal_qmssMtCsEnter (Void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to exit a critical section
 *      protected using Osal_qmssCsEnter() API.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_qmssMtCsExit (Ptr CsHandle)
{
    /* Release Semaphore using handle */
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to indicate that a block of memory is 
 *      about to be accessed. If the memory block is cached then this 
 *      indicates that the application would need to ensure that the 
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_qmssBeginMemAccess (void *ptr, uint32_t size)
{
#if 0        
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled */    
    CACHE_invL1d (ptr, size, CACHE_WAIT);

    /*  Cleanup the prefectch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    
#else
    UInt  key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /*  Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    

    /* Invalidate the cache. */
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Reenable Interrupts. */
    Hwi_restore(key);
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is used to indicate that the block of memory has 
 *      finished being accessed. If the memory block is cached then the 
 *      application would need to ensure that the contents of the cache 
 *      are updated immediately to the actual memory. 
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_qmssEndMemAccess (void *ptr, uint32_t size)
{
#if 0
    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled */    
    CACHE_wbL1d (ptr, size, CACHE_WAIT);        
    _mfence();
#else
    UInt  key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Writeback the contents of the cache. */
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);

    /* Reenable Interrupts. */
    Hwi_restore(key);    
#endif    
}

