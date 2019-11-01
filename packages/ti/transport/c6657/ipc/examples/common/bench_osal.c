/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>

#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/GateMP.h>

/* BIOS modules */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/c66/Cache.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>

 /* CSL modules */
#include <ti/csl/csl_semAux.h>
/* CSL Cache module includes */
#include <ti/csl/csl_cacheAux.h>
/* CSL XMC module includes */
#include <ti/csl/csl_xmcAux.h>

/* SRIO LLD */
#include <ti/drv/srio/srio_drv.h>
 
#include "bench_common.h"

 /**********************************************************************
 ****************************** Defines *******************************
 **********************************************************************/
#define USE_BIOS_MUTEX	1
 
#define CPPI_HW_SEM     1
#define QMSS_HW_SEM     2
#define SRIO_HW_SEM      3
#define PLATFORM_SPI_HW_SEM 4

#define MAX_MEM_MGR_ENTRIES     30

typedef struct MEM_MGMT_ENTRY
{
    uint8_t *ptrMemory;
    uint32_t isFree;
} MEM_MGMT_ENTRY;

MEM_MGMT_ENTRY gDataBufferMemMgr[MAX_MEM_MGR_ENTRIES];
int32_t gDataBufferMemMgrMaxSize = 0;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
uint32_t qmssMallocCounter = 0;
uint32_t qmssFreeCounter = 0;
uint32_t cppiMallocCounter = 0;
uint32_t cppiFreeCounter = 0;
uint32_t srioMallocCounter = 0;
uint32_t srioFreeCounter = 0;
uint32_t srioDataBufferMallocCounter = 0;
uint32_t srioDataBufferFreeCounter = 0;

/**********************************************************************
 *********************** Platform Library OSAL Functions **************
 **********************************************************************/
/**
 * ============================================================================
 *  @n@b Osal_platformMalloc
 *
 *  @b  brief
 *  @n  This routine implements the memory handler for Platform Library.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @return
 *      Allocated block address
 * =============================================================================
 */
uint8_t *Osal_platformMalloc (uint32_t num_bytes, uint32_t alignment)
{
	Error_Block	    errorBlock;

	/* Allocate memory.  */
	Error_init(&errorBlock);

	return Memory_alloc(NULL, num_bytes, alignment, &errorBlock);
}

/**
 * ============================================================================
 *  @n@b Osal_platformFree
 *
 *  @b  brief
 *  @n  This routine implements the memory free handler for Platform Library.
 *		Frees up memory allocated using
 *      @a Osal_platformMalloc ()
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
void Osal_platformFree (uint8_t *dataPtr, uint32_t num_bytes)
{

    /* Free up the memory */
    if (dataPtr)
    {
        Memory_free(NULL, dataPtr, num_bytes);
    }
}

/**
 * ============================================================================
 *  @n@b Osal_platformSpiCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization for the SPI bus.
 *
 *      This is a BLOCKING API.
 *.
 *
 *  @param[in]
 *  @n  None
 *
 *  @return
 *  @n  Nothing
 * =============================================================================
 */
void Osal_platformSpiCsEnter(void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (PLATFORM_SPI_HW_SEM)) == 0);

    return;
}

/**
 * ============================================================================
 *  @n@b Osal_platformSpiCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_platformSpiCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab the SPI bus.
 *     
 *
 *  @return     None
 * =============================================================================
 */
void Osal_platformSpiCsExit (void)
{
    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (PLATFORM_SPI_HW_SEM);

    return;
}

/*===================QMSS OSAL===================*/

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
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

#ifdef L2_CACHE
    /* Invalidate L2 cache. This should invalidate L1D as well.
     * Wait until operation is complete. */
    CACHE_invL2 (ptr, size, CACHE_FENCE_WAIT);
#else
    /* Invalidate L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
#endif

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);

    return;
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
 *
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_qmssEndMemAccess (void *ptr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

#ifdef L2_CACHE
    /* Writeback L2 cache. This should Writeback L1D as well.
     * Wait until operation is complete. */
    CACHE_wbL2 (ptr, size, CACHE_FENCE_WAIT);

#else
    /* Writeback L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);
#endif

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);
    return;
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
void* Osal_qmssCsEnter (void)
{
#if USE_BIOS_MUTEX
  IArg key = 0;

  if (GateMP_getDefaultRemote()) 
  {
    key = GateMP_enter(GateMP_getDefaultRemote());
  }
  return ((Ptr)key);
#else
  /* Get the hardware semaphore */
  while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);

  /* Create Semaphore for protection against access from multiple threads 
    * Not created here becasue application is not multithreaded */
  return NULL;
#endif
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
void Osal_qmssCsExit (void *CsHandle)
{
#if USE_BIOS_MUTEX
  if (GateMP_getDefaultRemote()) 
  {
    GateMP_leave(GateMP_getDefaultRemote(), (IArg)CsHandle);
  }

  return;
#else
  /* Release Semaphore using handle */
  /* Release the hardware semaphore */ 
  CSL_semReleaseSemaphore (QMSS_HW_SEM);
  return;
#endif
}

/**
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
 */
Ptr Osal_qmssMalloc (UInt32 num_bytes)
{
  Error_Block	    errorBlock;

  /* Allocate memory.  */
  Error_init(&errorBlock);
  
  qmssMallocCounter++;

  /* Allocate memory.  */
  return Memory_alloc(NULL, num_bytes, 0, &errorBlock);
}

/**
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
 */
Void Osal_qmssFree (Ptr dataPtr, UInt32 num_bytes)
{
  /* Free up the memory */
  if (dataPtr)
  {
    qmssFreeCounter++;
    /* Convert the global address to local address since
      * thats what the heap understands. */
    Memory_free(NULL, dataPtr, num_bytes);
  }
}

/*===================CPPI OSAL===================*/

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
void* Osal_cppiCsEnter (void)
{
#if USE_BIOS_MUTEX
  IArg key = 0;

  if (GateMP_getDefaultRemote()) 
  {
    key = GateMP_enter(GateMP_getDefaultRemote());
  }

  return ((Ptr)key);
#else
  /* Get the hardware semaphore for protection against multiple core access */
  while ((CSL_semAcquireDirect (CPPI_HW_SEM)) == 0);

  /* Create Semaphore for protection against access from multiple threads 
   * Not created here becasue application is not multithreaded */
  return NULL;
#endif
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
void Osal_cppiCsExit (void *CsHandle)
{
#if USE_BIOS_MUTEX
  if (GateMP_getDefaultRemote())
  {
    GateMP_leave(GateMP_getDefaultRemote(), (IArg)CsHandle);
  }
  return;
#else
  /* Release Semaphore using handle */
  /* Release the hardware semaphore */ 
  CSL_semReleaseSemaphore (CPPI_HW_SEM);
  return;
#endif
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
void Osal_cppiBeginMemAccess (void *ptr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

#ifdef L2_CACHE
    /* Invalidate L2 cache. This should invalidate L1D as well.
     * Wait until operation is complete. */
    CACHE_invL2 (ptr, size, CACHE_FENCE_WAIT);
#else
    /* Invalidate L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
#endif

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);

    return;
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
 *
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_cppiEndMemAccess (void *ptr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

#ifdef L2_CACHE
    /* Writeback L2 cache. This should Writeback L1D as well.
     * Wait until operation is complete. */
    CACHE_wbL2 (ptr, size, CACHE_FENCE_WAIT);

#else
    /* Writeback L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);
#endif

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);
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

  /* Allocate memory.  */
  Error_init(&errorBlock);
  cppiMallocCounter++;

  /* Allocate a buffer from the default HeapMemMp */
  return Memory_alloc (SharedRegion_getHeap(0), num_bytes, 0, &errorBlock);
}

/**
 *  @b Description
 *  @n
 *      The function is used to free a memory block of the specified size allocated
 *      using Osal_cppiMalloc() API.
 *
 *  @param[in]  ptr
 *      Pointer to the memory block to be cleaned up.
 *
 *  @param[in]  size
 *      Size of the memory block to be cleaned up.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_cppiFree (Ptr ptr, UInt32 size)
{
  if (ptr)
  {
    cppiFreeCounter++;
    Memory_free (SharedRegion_getHeap(0), ptr, size);
  }
}

/*===================SRIO OSAL===================*/

/**
 *  @b Description
 *  @n  
 *      The function is used to allocate a memory block of the specified size.
 *
 *  @param[in]  numBytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Allocated block address
 */
void* Osal_srioMalloc(UInt32 numBytes)
{
  Error_Block	errorBlock;
  void *ptr;

  /* Allocate memory.  */
  Error_init(&errorBlock);
  /* Increment the allocation counter. */
  srioMallocCounter++;	

  /* Allocate the memory. */
  ptr = Memory_alloc(NULL, numBytes, 0, &errorBlock);

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
 *  @param[in]  size
 *      Size of the memory block being cleaned up.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_srioFree (void* ptr, uint32_t size)
{
  if (ptr)
  {
    /* Increment the free counter. */
    srioFreeCounter++;	
    Memory_free(NULL, ptr, size);
  }
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

  /* Allocate memory.  */
  Error_init(&errorBlock);
  /* Allocate memory for all the data buffers */
  ptrMemory = (uint8_t*)Memory_alloc(NULL, MAX_MEM_MGR_ENTRIES*dataBufferSize, 16, &errorBlock);
  if (ptrMemory == NULL)
  {
    return -1;
  }

  /* Convert to a global address */
  ptrMemory = (Void*)l2_global_address((UInt32)ptrMemory);

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
 *      The function is used to allocate a data buffer of the specified
 *      size. Data buffers should always be allocated from the global
 *      address space.
 *
 *  @param[in]  numBytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Allocated block address
 */
Void* Osal_srioDataBufferMalloc(UInt32 numBytes)
{
  uint32_t    index;
  void*       ptrMemory = NULL;

  /* Basic Validation: Ensure that the memory size requested is within range. */
  if (numBytes > gDataBufferMemMgrMaxSize)
  {
    return NULL;
  }

  /* Increment the allocation counter. */
  srioDataBufferMallocCounter++;

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
 *      The function is used to clean up a previously allocated data buffer 
 *      block. All data buffers are in the global address space
 *
 *  @param[in]  ptr
 *      Pointer to the data buffer block to be cleaned up
 *  @param[in]  size
 *      Size of the data buffer
 *
 *  @retval
 *      Not Applicable
 */
void Osal_srioDataBufferFree(void* ptr, uint32_t numBytes)
{
  uint32_t    index;

  /* Increment the free counter. */
  srioDataBufferFreeCounter++;	

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
 *      The function is the SRIO OSAL Logging API which logs 
 *      the messages on the console.
 *
 *  @param[in]  fmt
 *      Formatted String.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_srioLog( String fmt, ... )
{
  /* System_printf(fmt); */
}

/**
 *  @b Description
 *  @n  
 *      The function is used to create a critical section.
 *
 *  @retval
 *      Semaphore Handle created
 */
Void* Osal_srioCreateSem(Void)
{
  return (Void*)Semaphore_create(0, NULL, NULL);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to delete a critical section.
 *
 *  @param[in]  semHandle
 *      Semaphore handle to be deleted
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_srioDeleteSem(Void* semHandle)
{
  Semaphore_delete(semHandle);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to pend on a semaphore
 *
 *  @param[in]  semHandle
 *      Semaphore handle on which the API will pend
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_srioPendSem(Void* semHandle)
{
  Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to post a semaphore
 *
 *  @param[in]  semHandle
 *      Semaphore handle which will be posted
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_srioPostSem(Void* semHandle)
{
  Semaphore_post(semHandle);
}

/**
 *  @b Description
 *  @n  
 *      This is the Multicore OSAL Implementation to protect the driver shared
 *      resources across multiple cores.
 *
 *  @retval
 *      Semaphore Opaque Handle
 */
void* Osal_srioEnterMultipleCoreCriticalSection(void)
{
#if USE_BIOS_MUTEX
  IArg key = 0;

  if (GateMP_getDefaultRemote()) 
  {
    key = GateMP_enter(GateMP_getDefaultRemote());
  }

  return ((Ptr)key);
#else
  /* Get the hardware semaphore for protection against multiple core access */
  while ((CSL_semAcquireDirect (SRIO_HW_SEM)) == 0);

  /* Create Semaphore for protection against access from multiple threads 
   * Not created here becasue application is not multithreaded */
  return NULL;
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is called to end the critical section which was protecting
 *      shared resources from access across multiple cores.
 *
 *  @param[in]  critSectHandle
 *      Semaphore opaque handle.
 *
 *  @retval
 *      None
 */
Void  Osal_srioExitMultipleCoreCriticalSection(Void* critSectHandle)
{
#if USE_BIOS_MUTEX
    if (GateMP_getDefaultRemote())
    {
      GateMP_leave(GateMP_getDefaultRemote(), (IArg)critSectHandle);
    }
    return;
#else
  /* Release Semaphore using handle */
  /* Release the hardware semaphore */ 
  CSL_semReleaseSemaphore (SRIO_HW_SEM);
  return;
#endif
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
Void* Osal_srioEnterSingleCoreCriticalSection(Srio_DrvHandle drvHandle)
{
  return (Void*)Hwi_disable();
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
 *      Not Applicable.
 */
Void Osal_srioExitSingleCoreCriticalSection(Void* critSectHandle)
{
  Hwi_enable();
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the SRIO driver to indicate that
 *      its about to access a block of memory and we need to ensure
 *      that the cache contents for this block are invalidated before
 *      we try and use it.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer which is being accessed
 *  @param[in]  size
 *      Size of the buffer which is to be accessed.
 *
 *  @retval
 *      None
 */
void Osal_srioBeginMemAccess(void* ptr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

#ifdef L2_CACHE
    /* Invalidate L2 cache. This should invalidate L1D as well.
     * Wait until operation is complete. */
    CACHE_invL2 (ptr, size, CACHE_FENCE_WAIT);
#else
    /* Invalidate L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
#endif

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);

    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the SRIO driver to indicate that its 
 *      ending access to a block of memory. We need to ensure that the
 *      contents of the cache are written back to the actual memory.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer 
 *  @param[in]  size
 *      Size of the buffer 
 *
 *  @retval
 *      None
 */
void Osal_srioEndMemAccess(void* ptr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

#ifdef L2_CACHE
    /* Writeback L2 cache. This should Writeback L1D as well.
     * Wait until operation is complete. */
    CACHE_wbL2 (ptr, size, CACHE_FENCE_WAIT);

#else
    /* Writeback L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);
#endif

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);
    return;
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
 *  @param[in]  descSize
 *      Size of the descriptor (Only valid for Driver Managed)
 *
 *  @retval
 *      None
 */
void Osal_srioBeginDescriptorAccess (Srio_DrvHandle drvHandle, void* ptr, uint32_t descSize)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

#ifdef L2_CACHE
    /* Invalidate L2 cache. This should invalidate L1D as well.
     * Wait until operation is complete. */
    CACHE_invL2 (ptr, descSize, CACHE_FENCE_WAIT);
#else
    /* Invalidate L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
    CACHE_invL1d (ptr, descSize, CACHE_FENCE_WAIT);
#endif

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);

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
 *  @param[in]  descSize
 *      Size of the descriptor (Only valid for Driver Managed)
 *
 *  @retval
 *      None
 */
void Osal_srioEndDescriptorAccess (Srio_DrvHandle drvHandle, void* ptr, uint32_t descSize)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

#ifdef L2_CACHE
    /* Writeback L2 cache. This should Writeback L1D as well.
     * Wait until operation is complete. */
    CACHE_wbL2 (ptr, descSize, CACHE_FENCE_WAIT);

#else
    /* Writeback L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
    CACHE_wbL1d (ptr, descSize, CACHE_FENCE_WAIT);
#endif

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);
    return;
}

