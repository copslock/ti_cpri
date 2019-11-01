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
#include "fw_mem_allocator.h"

#define System_printf   printf

uint32_t    Osal_qmss_MallocCounter = 0;
uint32_t    Osal_qmss_FreeCounter = 0;
uint32_t    Osal_cppi_MallocCounter = 0;
uint32_t    Osal_cppi_FreeCounter = 0;
uint32_t    cpswPaMemProtNestedLevel = 0;
uint32_t    globalCritkey;
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

void*  Osal_qmssAccCsEnter ()
{
    
    /* This is a suboptimal implementation for this OSAL, please refer to
	 * QMSS examples for optimal implementation of this function 
	 */
    return(Osal_qmssCsEnter ());
}

void Osal_qmssAccCsExit (void *  key)
{
    /* This is a suboptimal implementation for this OSAL, please refer to
	 * QMSS examples for optimal implementation of this function 
	 */
    Osal_qmssCsExit (key);
	return;
}

Ptr Osal_qmssMalloc (uint32_t num_bytes)
{
    Ptr ret;
   
    Osal_qmss_MallocCounter++;
    ret = malloc (num_bytes);    
    if(ret==NULL)
    {
      System_printf("\nERROR! QMSS Malloc failed!\n");
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
      System_printf("\nERROR! CPPI Malloc failed!\n");
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

void Osal_paBeginMemAccess (Ptr addr, uint32_t size)
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */

}

void Osal_paEndMemAccess (Ptr addr, uint32_t size)
{      
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
}


/**
 *  @b Description
 *  @n  
 *      The function is used to enter a critical section.
 *      Function protects against 
 *
 *      access from multiple threads on single core
 *      and
 *      access from multiple cores 
 *
 *  @param[in]  key
 *      Key used to lock the critical section.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_paMtCsEnter (uint32_t *key)
{

    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
}

/**
 *  @b Description
 *  @n  
 *      The function is used to exit a critical section 
 *      protected using Osal_salldCsEnter() API.
 *
 *  @param[in]  key
 *      Key used to unlock the critical section.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_paMtCsExit (uint32_t key)
{
    /* Stub Function. TBD: Would need to handle when for multi proc access 
     * To be handled using infrastructure available from Kernel
     */
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

/* FUNCTION PURPOSE: Multi-threaded critical section enter
***********************************************************************
* DESCRIPTION: The function is used to enter a multi-threaded critical
*              section. Function protects against 
 *      
 *              access from multiple threads on single core
*/  
void *Osal_rmMtCsEnter(void *mtSemObj)
{

    return NULL;
}

/* FUNCTION PURPOSE: Multi-threaded critical section exit
***********************************************************************
* DESCRIPTION: The function is used to exit a multi-threaded critical
*              section protected using Osal_rmMtCsEnter() API.
*/  
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
}

System_flush()
{
    fflush(stdout);
}

Task_exit()
{
    return;
}

