/**
 *   @file  mcbspMasterDigLpbk_osal.c
 *
 *   @brief   
 *      This is the OS abstraction layer and is used by the MCBSP
 *      driver for the MCBSP Example Digital Loopback Application. 
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012, Texas Instruments, Inc.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/hal/Hwi.h>
#include <xdc/cfg/global.h>
#include <ti/drv/mcbsp/mcbsp_drv.h>
#if 0
/* CSL Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#endif
/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/

#define MCBSP_HW_SEM         1
#define PLATFORM_SPI_HW_SEM	 2   /**< SPI BUS arbitration - Used by platform library 	*/

#ifdef MCBSP_LOOP_PING_PONG
#define MAX_MEM_MGR_ENTRIES     8
#else
#define MAX_MEM_MGR_ENTRIES     4
#endif

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
void *Osal_local2Global (void *addr)
{
	UInt32 corenum;

	/* Get the core number. */
	corenum = 0;//CSL_chipReadReg(CSL_CHIP_DNUM);
	return addr;

	if(((UInt32)addr & 0xff000000) == 0) {
	/* Compute the global address. */
		return ((void *)((UInt32)(addr) + (0x10000000 + (corenum*0x1000000))));
	}
	else
		return(addr);
}

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
Void* Osal_mcbspMalloc(UInt32 numBytes)
{
	Error_Block	errorBlock;
    Void*       ptr;

    /* Increment the allocation counter. */
    malloc_counter++;	

    /* Allocate the memory. */
    ptr = Memory_alloc(NULL, numBytes, 0, &errorBlock);

    /* Return the allocated memory block. */
	return ptr;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to clean up a specific memory block and is called
 *      from the MCBSP Driver. 
 *
 *  @param[in]  ptr
 *      Pointer to the memory block to be cleaned up.
 *  @param[in]  size
 *      Size of the memory block being cleaned up.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_mcbspFree (Void* ptr, UInt32 size)
{
    /* Increment the free counter. */
    free_counter++;	
	Memory_free(NULL, ptr, size);
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
    ptrMemory = (uint8_t*)Memory_alloc(NULL, MAX_MEM_MGR_ENTRIES*dataBufferSize, 128, &errorBlock);
    if (ptrMemory == NULL)
        return -1;

    /* Convert to a global address */
    ptrMemory = Osal_local2Global(ptrMemory);
 
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
Void* Osal_mcbspDataBufferMalloc(UInt32 numBytes)
{
    uint32_t    index;
    void*       ptrMemory = NULL;

    /* Basic Validation: Ensure that the memory size requested is within range. */
    if (numBytes > gDataBufferMemMgrMaxSize)
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
void Osal_mcbspDataBufferFree(void* ptr, uint32_t numBytes)
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
 *      The function is the MCBSP OSAL Logging API which logs 
 *      the messages on the console.
 *
 *  @param[in]  fmt
 *      Formatted String.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_mcbspLog( String fmt, ... )
{
}

/**
 *  @b Description
 *  @n  
 *      The function is used to create a critical section.
 *
 *  @retval
 *      Semaphore Handle created
 */
Void* Osal_mcbspCreateSem(Void)
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
Void Osal_mcbspDeleteSem(Void* semHandle)
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
Void Osal_mcbspPendSem(Void* semHandle)
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
Void Osal_mcbspPostSem(Void* semHandle)
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
void* Osal_mcbspEnterMultipleCoreCriticalSection(void)
{
    /* Get the hardware semaphore */
   // while ((CSL_semAcquireDirect (MCBSP_HW_SEM)) == 0);
    return NULL;
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
Void  Osal_mcbspExitMultipleCoreCriticalSection(Void* critSectHandle)
{
    //CSL_semReleaseSemaphore (MCBSP_HW_SEM);
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
Void* Osal_mcbspEnterSingleCoreCriticalSection()
{
    /* Disable interrupts */
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
Void Osal_mcbspExitSingleCoreCriticalSection(Void* critSectHandle)
{
    /* Driver Managed Configuration: We need to enable interrupts. */
    Hwi_enable();
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the MCBSP driver to indicate that
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
#include "ti/sysbios/hal/Cache.h"
void Osal_mcbspBeginMemAccess(void* ptr, uint32_t size)
{
    UInt  key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    //CSL_XMC_invalidatePrefetchBuffer();

    /* Invalidate the cache. */
    //CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);

    Cache_inv ((void *) ptr,size, 0x7fff, 1);

    /* Reenable Interrupts. */
    Hwi_restore(key);
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the MCBSP driver to indicate that its 
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
void Osal_mcbspEndMemAccess(void* ptr, uint32_t size)
{
    UInt  key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Writeback the cache. */
    //CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);
    Cache_wb ((void *) ptr,size, 0x7fff, 1);
    /* Reenable Interrupts. */
    Hwi_restore(key);
}

/**
 * @brief   The function is used by the MCBSP driver to test for an 
 *          empty queue.
 *
 *  <b> Parameter </b>
 *  @n  handle - Handle of a previously created Queue instance object 
 *
 *  <b> Return Value </b>
 *  @n  TRUE - If the queue is empty
 */
Bool Osal_mcbspQueueEmpty(void* handle)
{
    return Queue_empty(handle);
}

/**
 * @brief   The macro is used by the MCBSP driver to get an 
 *          element from the front of queue. The function 
 *          removes the element from the front of queue and
 *          returns a pointer to it.
 *
 *  <b> Parameter </b>
 *  @n  handle - Handle of a previously created Queue instance object 
 *
 *  <b> Return Value </b>
 *  @n  Handle (pointer) to former first element
 */
void* Osal_mcbspQueueGet(void* handle)
{
    return Queue_get(handle);
}

/**
 * @brief   The macro is used by the MCBSP driver to put an 
 *          element at the end of queue. 
 *
 *  <b> Parameter </b>
 *  @n  handle - Handle of a previously created Queue instance object 
 *  @n  elem - Pointer to new queue element 
 *
 *  <b> Return Value </b>
 *  @n  None
 */
void  Osal_mcbspQueuePut(void* handle, Mcbsp_QueueElem* elem)
{
    Queue_put(handle, (Queue_Elem *)elem);
    return;
}

/**
 * @brief   The macro is used by the MCBSP driver to wait 'n' 
 *          bit clocks to ensure proper synchronization internally. 
 *
 *  <b> Parameter </b>
 *  @n  nticks - Number of bit clocks to wait 
 *
 *  <b> Return Value </b>
 *  @n  None
 */
void  Osal_mcbspWaitNBitClocks(uint32_t nticks)
{
    Task_sleep(nticks);
    return;
}

/* OSAL functions for Platform Library */
uint8_t *Osal_platformMalloc (uint32_t num_bytes, uint32_t alignment)
{
	return malloc(num_bytes);
}

void Osal_platformFree (uint8_t *dataPtr, uint32_t num_bytes)
{
    /* Free up the memory */
    if (dataPtr)
    {
        free(dataPtr);
    }
}

void Osal_platformSpiCsEnter(void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    //while ((CSL_semAcquireDirect (PLATFORM_SPI_HW_SEM)) == 0);

    return;
}

void Osal_platformSpiCsExit (void)
{
    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    //CSL_semReleaseSemaphore (PLATFORM_SPI_HW_SEM);

    return;
}
