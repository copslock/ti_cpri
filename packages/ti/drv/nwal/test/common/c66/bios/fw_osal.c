/******************************************************************************
 * FILE PURPOSE:  Functions to OSAL related routines for running NWAL
 ******************************************************************************
 * FILE NAME:   fw_osal.c
 *
 * DESCRIPTION: Functions to initialize framework resources for running NWAL
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/sysbios/BIOS.h>
#ifdef NWAL_ENABLE_SA
#include <ti/drv/sa/sa_osal.h>
#endif
#include <ti/sysbios/hal/Hwi.h>
#include <c6x.h>
#include <stdlib.h>
#include <stdio.h>

uint32_t              Osal_qmss_MallocCounter =0;
uint32_t              Osal_qmss_FreeCounter =0;
uint32_t              Osal_cppi_MallocCounter =0;
uint32_t              Osal_cppi_FreeCounter =0;

#define COMMON_HW_SEM           3
/* Hardware Semaphore to synchronize access 
 * across different cores to the QMSS library.
 */
#define     QMSS_HW_SEM         4 

/* Hardware Semaphore to synchronize access 
 * across different cores to the CPPI library.
 */
#define     CPPI_HW_SEM         COMMON_HW_SEM 

/* Hardware Semaphore to synchronize access
 * across different cores to the PA library.
 */
#define     PA_HW_SEM           COMMON_HW_SEM 

/* Hardware Semaphore to synchronize access
 * across different cores to the SA library.
 */
#define     SA_HW_SEM           COMMON_HW_SEM 

/* Hardware Semaphore to synchronize access
 * across different cores for NWAL library.
 * Semaphore should be different from the one allocated for
 * CPPI/QMSS/PA/SA
 */
#define     NWAL_HW_SEM           2 


extern cregister volatile unsigned int DNUM;


uint32_t globalCritkey;

unsigned int sumCacheCycles=0;
unsigned int numCacheOps=0;
void Osal_cache_op_measure_reset(void) { sumCacheCycles=0; numCacheOps=0;}
unsigned int Osal_cache_op_measure(unsigned long long * p_n) { *p_n = numCacheOps;  return sumCacheCycles;}

/*****************************************************************************
 * FUNCTION PURPOSE: Cache Invalidation Routine
 ***************************************************************************** 
 * DESCRIPTION: Cache Invalidation Routine
 *****************************************************************************/
void Osal_invalidateCache (void *blockPtr, uint32_t size) 
{
    UInt  key;
    register unsigned int v1;
    register unsigned int v2;
    
    if(((uint32_t)blockPtr & 0xF0000000ul)== 0x10000000ul)
    {
        return;
    }   

    v1= fw_read_clock();

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

    size = size + (((uint32_t)blockPtr) &(CACHE_L2_LINESIZE-1));
    /* Invalidate the cache. */
    CACHE_invL2 (blockPtr,size ,CACHE_FENCE_WAIT);

    /* Reenable Interrupts. */
    Hwi_restore(key);

    v2= fw_read_clock();
    sumCacheCycles += (v2-v1); 
    numCacheOps+=1;

    return;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Cache Writeback Routine
 ***************************************************************************** 
 * DESCRIPTION: Cache Invalidation Routine
 *****************************************************************************/
void Osal_writeBackCache (void *blockPtr, uint32_t size) 
{
    UInt  key;
    register unsigned int v1;
    register unsigned int v2;
#if 0
    if(((uint32_t)blockPtr & 0xF0000000ul)== 0x10000000ul)
    {
        return;
    }  
#endif
    v1= fw_read_clock();

    /* Disable Interrupts */
    key = Hwi_disable();

    size = size + (((uint32_t)blockPtr) &(CACHE_L2_LINESIZE-1));

    /* Invalidate the cache. */
    CACHE_wbL2 (blockPtr, size,CACHE_FENCE_WAIT);

    /* Reenable Interrupts. */
    Hwi_restore(key);

    v2= fw_read_clock();
    sumCacheCycles += (v2-v1); 
    numCacheOps+=1;
    return;
}


Void *  Osal_qmssMtCsEnter()
{
     /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);       
    return NULL;
}


Void Osal_qmssMtCsExit(void *key)
{
    /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (QMSS_HW_SEM);        
    return;
}

Void Osal_nwalCsEnter(uint32_t *key)
{
    /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (NWAL_HW_SEM)) == 0); 
    return;
}

Void Osal_nwalCsExit(uint32_t key)
{
    /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (NWAL_HW_SEM); 
    return;
}


Void Osal_qmssLog ( String fmt, ... )
{
}


Void Osal_cppiCsEnter (uint32_t *key)
{
    /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (CPPI_HW_SEM)) == 0); 
    return;
}

Void Osal_cppiCsExit (uint32_t key)
{
    /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (CPPI_HW_SEM);    
    return;
}

Void Osal_cppiLog ( String fmt, ... )
{
}

void Osal_paBeginMemAccess (Ptr addr, uint32_t size)
{
    Osal_invalidateCache(addr,size);

}

void Osal_paEndMemAccess (Ptr addr, uint32_t size)
{
    Osal_writeBackCache(addr,size);
}


void Osal_paMtCsEnter (uint32_t *key)
{

    /* Get the hardware semaphore. 
     *
     * Acquire Multi core PA synchronization lock 
     */
     while ((CSL_semAcquireDirect (PA_HW_SEM)) == 0);
    return;
}

void Osal_paMtCsExit (uint32_t key)
{
    /* Release the hardware semaphore */
    CSL_semReleaseSemaphore (PA_HW_SEM);
}

void*  Osal_qmssCsEnter ()
{
    
    /* Disable all interrupts */
    while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);
    return(NULL);
}

Void Osal_qmssCsExit (void *  key)
{
    /* Enable all interrupts */
    CSL_semReleaseSemaphore (QMSS_HW_SEM);  
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

Void Osal_qmssFree (Ptr ptr, uint32_t size)
{
    /* Increment the free counter. */
    Osal_qmss_FreeCounter++;	
    free(ptr);
}

Ptr Osal_cppiMalloc (uint32_t num_bytes)
{
    Ptr ret;
    
    Osal_cppi_MallocCounter++;
    ret = malloc (num_bytes);

    if(ret==NULL)
    {
      System_printf("\nERROR! CPPI Malloc failed!\n");
    }   
    
    return ret;
}

Void Osal_cppiFree (Ptr ptr, uint32_t size)
{
    /* Increment the free counter. */
    Osal_cppi_FreeCounter++;	
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

void Osal_nwalInvalidateCache (void *blockPtr, uint32_t size)
{
    Osal_invalidateCache(blockPtr,size);
    return;
}

void Osal_nwalWriteBackCache (void *blockPtr, uint32_t size)
{
    Osal_writeBackCache(blockPtr,size);
    return;
}

UInt32 Osal_nwalGetCacheLineSize (void )
{
    /* By default assumes L2 cache line is enabled. If not return CACHE_L1D_LINESIZE */
    return (CACHE_L2_LINESIZE);
}

/********************************************************************
 * FUNCTION PURPOSE: Convert local address to global
 ********************************************************************
 * DESCRIPTION: Returns global address
 ********************************************************************/
unsigned int Osal_nwalLocToGlobAddr(unsigned int x)
{
  if ((x >= 0x800000) && (x < 0x900000))
  {
    x = (1 << 28) | (DNUM << 24) | x;
  }
    return (x);
}

uint16_t Osal_nwalGetProcId (void )
{
    return DNUM;
}

extern cregister volatile unsigned int TSCL;
uint64_t Osal_nwalGetTimeStamp(void)
{
    /* Returning local TSCL counter. In the event if application need to have unique timer count 
     * globally per SOC, routine would need to be modified 
     */
    return(TSCL);
}
#ifdef NWAL_ENABLE_SA
Void Osal_saCsEnter (uint32_t *key)
{
    /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (SA_HW_SEM)) == 0); 
    return;
}

Void Osal_saCsExit (uint32_t key)
{
     /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (SA_HW_SEM);   
    return;
}


Void Osal_saMtCsEnter (uint32_t *key)
{
    /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (SA_HW_SEM)) == 0); 
    return;
}

Void Osal_saMtCsExit (uint32_t key)
{
    /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (SA_HW_SEM); 
    return;
}

void Osal_saBeginMemAccess (void *blockPtr, uint32_t size)
{
    Osal_invalidateCache(blockPtr,size);
    return;
}

void Osal_saEndMemAccess (void *blockPtr, uint32_t size)
{
    Osal_writeBackCache(blockPtr,size);
    return;
}



Void Osal_saBeginScAccess (void* addr, UInt32 size)
{
   Osal_invalidateCache(addr,size);  
        
}
 
Void Osal_saEndScAccess   (void* addr, UInt32 size)
{
    Osal_writeBackCache(addr,size);
    
}

void* Osal_saGetSCPhyAddr(void* vaddr)
{
    return vaddr;
}

uint16_t Osal_saGetProcId (void )
{
    return (uint16_t)CSL_chipReadReg(CSL_CHIP_DNUM);
}
int   Osal_saGetSysEndianMode(void)
{
#if defined( _BIG_ENDIAN )
    return((int)sa_SYS_ENDIAN_MODE_BIG);
#else
    return((int)sa_SYS_ENDIAN_MODE_LITTLE);
#endif
}

#endif
/**
 *  @b Description
 *  @n  
 *      The function is used by the packet library to invalidate the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      None
 */
void Osal_pktLibBeginMemAccess(void* ptr, uint32_t size)
{
    Osal_invalidateCache(ptr,size);
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the packet library to writeback the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      None
 */
void Osal_pktLibEndMemAccess(void* ptr, uint32_t size)
{
    Osal_writeBackCache(ptr,size);
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the packet library to invalidate the cache.
 *
 *  @param[in]  heapHandle
 *      Heap Handle to which the packet belongs. 
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      None
 */
void Osal_pktLibBeginPktAccess(Pktlib_HeapHandle heapHandle, Ti_Pkt* ptrPkt, uint32_t size)
{
    /* TODO: We should use the 'heapHandle' and compare it with what we got from the
     * 'create/find' HEAP API & depending upon the comparison take appropriate action. 
     * Just for testing we are always invalidating the cache here. */

     Osal_invalidateCache(ptrPkt,size);
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the packet library to writeback the packet which hasd been 
 *      accessed & updated.
 *
 *  @param[in]  heapHandle
 *      Heap Handle to which the packet belongs. 
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      None
 */
void Osal_pktLibEndPktAccess(Pktlib_HeapHandle heapHandle, Ti_Pkt* ptrPkt, uint32_t size)
{
    /* TODO: We should use the 'heapHandle' and compare it with what we got from the
     * 'create/find' HEAP API & depending upon the comparison take appropriate action. 
     * Just for testing we are always writing back the cache here. */

    /* Writeback the contents of the cache. */
    Osal_writeBackCache(ptrPkt,size);
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the packet library to enter a critical section since
 *      a resource is about to be modified.
 *
 *  @param[in]  heapHandle
 *      Heap Handle 
 *
 *  @retval
 *      Critical Section Handle
 */
void* Osal_pktLibEnterCriticalSection(Pktlib_HeapHandle heapHandle)
{
    /* TODO: We should use the 'heapHandle' and compare it with what we got from the
     * 'create/find' HEAP API & depending upon the comparison take appropriate action. 
     * Implementations here could range from a MULTI-THREAD protection if the packets in 
     * the heap are being accessed across multiple threads or MULTI-CORE if the packets
     * are being accessed across multiple cores and features: split and clone are used.
     * For NWAL layer no protection required.
     *
     * For testing we are not doing any of this so we are simply setting it to NOOP */
    return NULL;
}

/**
 *  @b Description
 *  @n  
 *      The function is used by the packet library to enter a critical section since
 *      a resource is about to be modified.
 *
 *  @param[in]  heapHandle
 *      Heap Handle 
 *  @param[in]  csHandle
 *      Critical Section Handle
 *
 *  @retval
 *      Not Applicable.    
 */
void  Osal_pktLibExitCriticalSection(Pktlib_HeapHandle heapHandle, void* csHandle)
{
    /* TODO: We should use the 'heapHandle' and compare it with what we got from the
     * 'create/find' HEAP API & depending upon the comparison take appropriate action. 
     * Implementations here could range from a MULTI-THREAD protection if the packets in 
     * the heap are being accessed across multiple threads or MULTI-CORE if the packets
     * are being accessed across multiple cores and features: split and clone are used.
     * For NWAL layer no protection required.. 
     *
     * For testing we are not doing any of this so we are simply setting it to NOOP */
    return;        
}



