/******************************************************************************
 * FILE PURPOSE:  Functions to OSAL related routines for running Example
 ******************************************************************************
 * FILE NAME:   fw_osal.c
 *
 * DESCRIPTION: Functions to initialize framework resources for running Example
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2013
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

uint32_t    Osal_qmss_MallocCounter = 0;
uint32_t    Osal_qmss_FreeCounter = 0;
uint32_t    Osal_cppi_MallocCounter = 0;
uint32_t    Osal_cppi_FreeCounter = 0;
uint32_t    Osal_rm_MallocCounter = 0;
uint32_t    Osal_rm_FreeCounter   = 0;

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

void Osal_qmssLog ( String fmt, ... )
{
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
    return (void *)(fw_mem_start_phy + ((uint8_t*)ptr - fw_mem_start));
}

void* Osal_qmssPhyToVirt (void *ptr)
{
    if(ptr == NULL) 
    {
        return NULL;
    }
    return (void *)(fw_mem_start + ((uint8_t*)ptr - fw_mem_start_phy));
}

/******************************************************************************
* Function to convert descriptor address references
* from virtual to physical.
* NOTE: QMSS unit tests are not using CPPI descriptors
******************************************************************************/
void* Osal_qmssConvertDescVirtToPhy(uint32_t QID, void *descAddr)
{
    if (!descAddr) return (void *)0;
    descAddr = Osal_qmssVirtToPhy(descAddr);
    /* Issue memory barrier */
    fw_mMemBarrier();
    return descAddr;
}

/******************************************************************************
* Function to convert descriptor address references
* from physical to virtual.
* NOTE: QMSS unit tests are not using CPPI descriptors
******************************************************************************/
void* Osal_qmssConvertDescPhyToVirt(uint32_t QID, void *descAddr)
{
    if (!descAddr) return (void *)0;
    descAddr = Osal_qmssPhyToVirt(descAddr);    
    if (!descAddr) return (void *)0;
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

