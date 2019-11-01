/******************************************************************************
 * FILE PURPOSE:  Functions to OSAL related routines for running Example
 ******************************************************************************
 * FILE NAME:   fw_osal.c
 *
 * DESCRIPTION: Functions to initialize framework resources for running Example
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2015
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
#include "fw_test.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>
#include "fw_mem_allocator.h"
#include "linuxutil.h"
#include "ti/drv/srio/srio_drv.h"
#include "srio_test.h"

uint32_t    Osal_qmss_MallocCounter = 0;
uint32_t    Osal_qmss_FreeCounter = 0;
uint32_t    Osal_cppi_MallocCounter = 0;
uint32_t    Osal_cppi_FreeCounter = 0;
uint32_t    Osal_rm_MallocCounter = 0;
uint32_t    Osal_rm_FreeCounter   = 0;
uint32_t    Osal_srio_MallocCounter = 0;
uint32_t    Osal_srio_FreeCounter = 0;

#define MAX_MEM_MGR_ENTRIES     30

typedef struct MEM_MGMT_ENTRY
{
    uint8_t*    ptrMemory;
    uint32_t    isFree;
}MEM_MGMT_ENTRY;

MEM_MGMT_ENTRY     gDataBufferMemMgr[MAX_MEM_MGR_ENTRIES];
int32_t            gDataBufferMemMgrMaxSize = 0;
 
uint32_t malloc_counter = 0;
uint32_t free_counter   = 0;

/* Lock to be used for critical section */
pthread_mutex_t mutex_lock;

/**
 *  @b Description
 *  @n  
 *     General Memory Barrier guarantees that all LOAD and STORE operations that were issued before the
 *     barrier occur before the LOAD and STORE operations issued after the barrier
 *      
 */
static inline void fw_mMemBarrier(void) {__sync_synchronize();}

void fw_osalInit() 
{
    pthread_mutex_init(&mutex_lock, NULL);
    return;
}

void fw_osalshutdown() 
{
    pthread_mutex_destroy(&mutex_lock);
    return;
}

static inline void fw_osalEnterCS()
{
#if 0
    pthread_mutex_lock(&mutex_lock);
#endif 
    return;
}

static inline void fw_osalLeaveCS()
{

#if 0
    pthread_mutex_unlock(&mutex_lock);
#endif
    return;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Cache Invalidation Routine
 ***************************************************************************** 
 * DESCRIPTION: Cache Invalidation Routine
 *****************************************************************************/
void Osal_invalidateCache (void *blockPtr, uint32_t size) 
{
    /* Stub Function. TBD: Would need to handle when cache is enabled for ARM */
    return;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Cache Writeback Routine
 ***************************************************************************** 
 * DESCRIPTION: Cache Invalidation Routine
 *****************************************************************************/
void Osal_writeBackCache (void *blockPtr, uint32_t size) 
{
    /* Stub Function. TBD: Would need to handle when cache is enabled for ARM */
    return;
}


void *  Osal_qmssMtCsEnter()
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return NULL;
}


void Osal_qmssMtCsExit(void *key)
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return;
}


void Osal_qmssLog ( String fmt, ... )
{
}

void*  Osal_qmssAccCsEnter ()
{
    
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled once infrastructure is available from Kernel
     */
    return(NULL);
}

void Osal_qmssAccCsExit (void *  key)
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return;
}

void*  Osal_qmssCsEnter ()
{
    
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled once infrastructure is available from Kernel
     */
    return(NULL);
}

void Osal_qmssCsExit (void *  key)
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return;
}

Ptr Osal_qmssMalloc (uint32_t num_bytes)
{
    Ptr ret;
   
    Osal_qmss_MallocCounter++;
    ret = malloc (num_bytes);    
    if(ret==NULL)
    {
      printf("\nERROR! QMSS Malloc failed!\n");
    }    
    
    return ret;
}

void Osal_qmssFree (Ptr ptr, uint32_t size)
{
    /* Increment the free counter. */
    Osal_qmss_FreeCounter++;
    free(ptr);
}


void Osal_qmssBeginMemAccess (void *blockPtr, uint32_t size)
{
    Osal_invalidateCache(blockPtr,size);
    return;
}

void  Osal_qmssEndMemAccess (void *blockPtr, uint32_t size)
{
    Osal_writeBackCache(blockPtr,size);
    return;
}
void* Osal_qmssVirtToPhy (void *ptr)
{
    if(ptr == NULL) 
    {
        return NULL;
    }
    return (void *)(((uint8_t *)((uint32_t)fw_mem_start_iovirt)) + ((uint8_t*)ptr - fw_mem_start));
}

void* Osal_qmssPhyToVirt (void *ptr)
{
    if(ptr == NULL) 
    {
        return NULL;
    }
    return (void *)(fw_mem_start + ((uint8_t*)ptr -  (((uint8_t *)((uint32_t)fw_mem_start_iovirt)))));
}

/******************************************************************************
* Function to traverse a CPPI descriptor and convert all address references
* from virtual to physical.
******************************************************************************/
void* Osal_qmssConvertDescVirtToPhy(uint32_t QID, void *descAddr)
{
    if (!descAddr) return (void *)0;

    if (Cppi_getDescType((Cppi_Desc *)QMSS_DESC_PTR(descAddr)) == Cppi_DescType_HOST)
    {
        Cppi_HostDesc *nextBDPtr = (Cppi_HostDesc *)QMSS_DESC_PTR(descAddr);
        Cppi_HostDesc *prevBDPtr = 0;
        while (nextBDPtr)
        {
            void *buffPtr;
            if (nextBDPtr->buffPtr)
            {
                buffPtr = (void *)nextBDPtr->buffPtr;
                nextBDPtr->buffPtr = (uint32_t)Osal_qmssVirtToPhy((void *)(nextBDPtr->buffPtr));
                if (!(nextBDPtr->buffPtr)) return (void *)0;
            }

            if (nextBDPtr->origBuffPtr)
            {
                nextBDPtr->origBuffPtr = (uint32_t)Osal_qmssVirtToPhy((void *)(nextBDPtr->origBuffPtr));
                if (!(nextBDPtr->origBuffPtr)) return (void *)0;
            }

            prevBDPtr = nextBDPtr;
            nextBDPtr = (Cppi_HostDesc *)QMSS_DESC_PTR((nextBDPtr->nextBDPtr));
            if (prevBDPtr->nextBDPtr)
            {
                prevBDPtr->nextBDPtr = (uint32_t)Osal_qmssVirtToPhy((void *)(prevBDPtr->nextBDPtr));
                if (!(prevBDPtr->nextBDPtr)) return (void *)0;
            }

            Qmss_osalEndMemAccess(buffPtr, prevBDPtr->buffLen);
            Qmss_osalEndMemAccess(prevBDPtr, sizeof(Cppi_HostDesc));
        }

        descAddr = Osal_qmssVirtToPhy(descAddr);
        if (!descAddr) return (void *)0;
    } 
    else if (Cppi_getDescType((Cppi_Desc *)QMSS_DESC_PTR(descAddr)) == Cppi_DescType_MONOLITHIC)
    {
        descAddr = Osal_qmssVirtToPhy(descAddr);
        if (!descAddr) return (void *)0;
    } 
    /* Issue memory barrier */
    fw_mMemBarrier();
    return descAddr;
}

/******************************************************************************
* Function to traverse a CPPI descriptor and convert all address references
* from physical to virtual.
******************************************************************************/
void* Osal_qmssConvertDescPhyToVirt(uint32_t QID, void *descAddr)
{
    if (!descAddr) return (void *)0;
    descAddr = Osal_qmssPhyToVirt(descAddr);    
    if (!descAddr) return (void *)0;

    if (Cppi_getDescType((Cppi_Desc *)QMSS_DESC_PTR(descAddr)) == Cppi_DescType_HOST)
    {
        Cppi_HostDesc *nextBDPtr = (Cppi_HostDesc *)QMSS_DESC_PTR(descAddr);
        while (nextBDPtr)
        {
            Qmss_osalBeginMemAccess(nextBDPtr, sizeof(Cppi_HostDesc));
            if (nextBDPtr->buffPtr)
            {
                nextBDPtr->buffPtr = (uint32_t)Osal_qmssPhyToVirt((void *)(nextBDPtr->buffPtr));
                if (!(nextBDPtr->buffPtr)) return (void *)0;
            }

            if (nextBDPtr->origBuffPtr)
            {
                nextBDPtr->origBuffPtr = (uint32_t)Osal_qmssPhyToVirt((void *)(nextBDPtr->origBuffPtr));
                if (!(nextBDPtr->origBuffPtr)) return (void *)0;
            }

            if (nextBDPtr->nextBDPtr)
            {
                nextBDPtr->nextBDPtr = (uint32_t)Osal_qmssPhyToVirt((void *)(nextBDPtr->nextBDPtr));
                if (!(nextBDPtr->nextBDPtr)) return (void *)0;
            }

            Qmss_osalBeginMemAccess((void *)(nextBDPtr->buffPtr), nextBDPtr->buffLen);
            nextBDPtr = (void *)QMSS_DESC_PTR((nextBDPtr->nextBDPtr));
        }
    } 
    return descAddr;
}

void Osal_cppiCsEnter (uint32_t *key)
{ 

    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return;
}

void Osal_cppiCsExit (uint32_t key)
{

    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return;
}

void Osal_cppiLog ( String fmt, ... )
{
}

Ptr Osal_cppiMalloc (uint32_t num_bytes)
{
    Ptr ret;    
    Osal_cppi_MallocCounter++;
    num_bytes += (CACHE_LINESZ-1);
    ret = malloc (num_bytes);
    if(ret==NULL)
    {
      printf("\nERROR! CPPI Malloc failed!\n");
    }       
    return ret;
}

void Osal_cppiFree (Ptr ptr, uint32_t size)
{
    /* Increment the free counter. */
    Osal_cppi_FreeCounter++;
    free(ptr);    
}

void Osal_cppiBeginMemAccess (void *blockPtr, uint32_t size)
{
    Osal_invalidateCache(blockPtr,size);
    return;
}

void Osal_cppiEndMemAccess (void *blockPtr, uint32_t size)
{
    Osal_writeBackCache(blockPtr,size);
    return;
}

void *Osal_rmMalloc (uint32_t num_bytes)
{
    /* Increment the allocation counter. */
    Osal_rm_MallocCounter++;

    /* Allocate memory. */
    return calloc(1, num_bytes);
}
 
void Osal_rmFree (void *ptr, uint32_t size)
{
    /* Increment the free counter. */
    Osal_rm_FreeCounter++;
    free(ptr);
}

void *Osal_rmCsEnter(void)
{
    return NULL;
}

void Osal_rmCsExit(void *CsHandle)
{

}

void *Osal_rmMtCsEnter(void *mtSemObj)
{

    return NULL;
}

void Osal_rmMtCsExit(void *mtSemObj, void *CsHandle)
{

}

void Osal_rmBeginMemAccess(void *ptr, uint32_t size)
{
    return;
}
 
void Osal_rmEndMemAccess(void *ptr, uint32_t size)
{
    return;
}

void *Osal_rmTaskBlockCreate(void)
{
    return(NULL);
}

void Osal_rmTaskBlock(void *handle)
{

}

void Osal_rmTaskUnblock(void *handle)
{

}

void Osal_rmTaskBlockDelete(void *handle)
{

}

void Osal_rmLog (char *fmt, ... )
{
    va_list ap;
    
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

void Osal_fwCsEnter (uint32_t *key)
{

    /* Stub Function. TBD: Would need to handle when for multi proc access
     * To be handled using infrastructure available from Kernel
     */
    return;
}

void Osal_fwCsExit (uint32_t key)
{

    /* Stub Function. TBD: Would need to handle when for multi proc access
     * To be handled using infrastructure available from Kernel
     */
    return;
}

Ptr Osal_srioMalloc (uint32_t num_bytes)
{
    Ptr ret;    
    Osal_srio_MallocCounter++;
    num_bytes += (CACHE_LINESZ-1);
    ret = malloc (num_bytes);
    if(ret==NULL)
    {
      printf("\nERROR! SRIO Malloc failed!\n");
    }       
    return ret;
}

void Osal_srioFree (Ptr ptr, uint32_t size)
{
    /* Increment the free counter. */
    Osal_srio_FreeCounter++;
    free(ptr);    
}

void Osal_srioBeginMemAccess (void *blockPtr, uint32_t size)
{
    Osal_invalidateCache(blockPtr,size);
    return;
}

void  Osal_srioEndMemAccess (void *blockPtr, uint32_t size)
{
    Osal_writeBackCache(blockPtr,size);
    return;
}

void *Osal_srioEnterMultipleCoreCriticalSection()
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return NULL;
}


void Osal_srioExitMultipleCoreCriticalSection (void *key)
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return;
}

void *Osal_srioEnterSingleCoreCriticalSection()
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return NULL;
}


void Osal_srioExitSingleCoreCriticalSection (void *key)
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
    return;
}

void Osal_srioLog ( String fmt, ... )
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
void* Osal_srioCreateSem(void)
{
    return NULL;
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
void Osal_srioDeleteSem(void* semHandle)
{
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
void Osal_srioPendSem(void* semHandle)
{
    extern int qpendFd;
    extern Srio_DrvHandle isr_hSrioDriver;
    if (waitForInterrupt (qpendFd) < 0)
    {
        printf ("waitForInterrupt failed\n");
    }
    else
    {
        /* flush received packets */
        Srio_rxCompletionIsr (isr_hSrioDriver);
    }
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
void Osal_srioPostSem(void* semHandle)
{
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
    /* In the Application all descriptor are located in LL2 memory we dont need to
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
    /* In the Application the descriptors are located in Core 0 Global address space
     * If they are accessed from other cores we need to ensure that there is an MFENCE so 
     * that all the access to the descriptors are complete before we start pushing them
     * out. */
    __sync_synchronize();
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
    uint8_t*    ptrMemory;
    uint32_t    index;

    /* Get this cores memory area.  This implementation of data
     * buffer heap does not allow sharing buffer allocation 
     * across cores */
    if (SIZE_DATA_BUFFER < MAX_MEM_MGR_ENTRIES * dataBufferSize)
    {
        printf ("Data buffer area too small (%d/%d*%d)\n", SIZE_DATA_BUFFER,
                MAX_MEM_MGR_ENTRIES, dataBufferSize);
	return -1;
    }

    /* Allocate memory for all the data buffers */
    ptrMemory = dataBuff[coreNum];
    if (ptrMemory == NULL)
        return -1;

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
void* Osal_srioDataBufferMalloc(uint32_t numBytes)
{
    uint32_t    index;
    void*       ptrMemory = NULL;

    /* Basic Validation: Ensure that the memory size requested is within range. */
    if (numBytes > gDataBufferMemMgrMaxSize)
        return NULL;

    /* Increment the allocation counter. */
    malloc_counter++;

    /* Lock out threads */
    fw_osalEnterCS();

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

    /* Unlock threads. */
    fw_osalLeaveCS();

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
    free_counter++;	

    /* Lock out threads */
    fw_osalEnterCS();

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

    /* Unlock threads. */
    fw_osalLeaveCS();
    return;
}

