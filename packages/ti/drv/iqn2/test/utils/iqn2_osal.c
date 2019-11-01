/**
 *   @file  iqn2_osal.c
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
 *  @n   (C) Copyright 2013, Texas Instruments, Inc.
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
#ifdef __ARMv7
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdarg.h>
#else
#include <xdc/std.h>
#include <c6x.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <ti/drv/iqn2/iqn2_osal.h>

#ifdef _TMS320C6X
/* CSL Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_semAux.h>

#ifndef USERESMGR
/* Hardware Semaphore to synchronize access from
 * multiple applications across different cores to the QMSS library.
 */
#define     QMSS_HW_SEM         3

/* Hardware Semaphore to synchronize access from
 * multiple applications across different cores to the CPPI library.
 */
#define     CPPI_HW_SEM         4

/* Hardware Semaphore to synchronize access from
 * multiple applications across different cores to the AIF2 library.
 */
#define     BARRIER_HW_SEM      5

/////// SIMPLE OSAL for IQN2 TESTS //////////////
void* Osal_cppiCsEnter (void)
{
    /* Get the hardware semaphore */
    while ((CSL_semAcquireDirect (CPPI_HW_SEM)) == 0);
    return NULL;
}

void Osal_cppiCsExit (void *CsHandle)
{
    /* Release the hardware semaphore */
    CSL_semReleaseSemaphore (CPPI_HW_SEM);
    return;
}

void* Osal_qmssCsEnter (void)
{
    /* Get the hardware semaphore */
    while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);

    /* Create Semaphore for protection against access from multiple threads
     * Not created here becasue application is not multithreaded
     * */
    return NULL;
}

void Osal_qmssCsExit (void *CsHandle)
{
    /* Release Semaphore using handle */

    /* Release the hardware semaphore */
    CSL_semReleaseSemaphore (QMSS_HW_SEM);

    return;
}

void Osal_cppiBeginMemAccess (void *ptr, uint32_t size)
{
    CACHE_invL1d (ptr, size, CACHE_WAIT);
    return;
}

void Osal_cppiEndMemAccess (void *ptr, uint32_t size)
{
    CACHE_wbL1d (ptr, size, CACHE_WAIT);
    return;
}

void Osal_qmssBeginMemAccess (void *ptr, uint32_t size)
{
    CACHE_invL1d (ptr, size, CACHE_WAIT);
    return;
}

void Osal_qmssEndMemAccess (void *ptr, uint32_t size)
{
    CACHE_wbL1d (ptr, size, CACHE_WAIT);
    return;
}

void* Osal_cppiMalloc (uint32_t num_bytes)
{
    void*           dataPtr = NULL;

    /* Allocate a buffer from system heap */
    dataPtr = malloc(num_bytes);

    if (dataPtr == NULL) printf("CPPI malloc issue\n");

    return (dataPtr);
}

void Osal_cppiFree (void *ptr, uint32_t num_bytes)
{
      free (ptr);
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
#endif
#endif

void Osal_dfeLog ( char *fmt, ... )
{
    va_list args;

    va_start (args, fmt);
    vprintf (fmt, args);
    va_end (args);

}

void Osal_iqn2Log ( char *fmt, ... )
{
    va_list args;

    va_start (args, fmt);
    vprintf (fmt, args);
    va_end (args);
	
}

/* Used by the IQN2 LLD to implement a time delay while the AT and uAT re-synchronization procedure runs */
void Osal_iqn2Sleep (  )
{
    volatile uint32_t i, n;

    n = 0;
    for (i = 0; i < 1000; i++)
    {
        n = n + 1;
    }
}

/* FUNCTION PURPOSE: Allocates memory
 ***********************************************************************
 * DESCRIPTION: The function is used to allocate a memory block of the
 *              specified size.
 */
void *Osal_rmMalloc (uint32_t num_bytes)
{
    void*           dataPtr = NULL;

    /* Allocate a buffer from system heap */
    dataPtr = malloc(num_bytes);

    if (dataPtr == NULL) printf("RM malloc issue\n");

    return (dataPtr);
}

/* FUNCTION PURPOSE: Frees memory
 ***********************************************************************
 * DESCRIPTION: The function is used to free a memory block of the
 *              specified size.
 */
void Osal_rmFree (void *ptr, uint32_t size)
{
    free (ptr);
}

/* FUNCTION PURPOSE: Critical section enter
 ***********************************************************************
 * DESCRIPTION: The function is used to enter a critical section.
 *              Function protects against
 *
 *              access from multiple cores
 *              and
 *              access from multiple threads on single core
 */
void *Osal_rmCsEnter(void)
{

    return NULL;
}

/* FUNCTION PURPOSE: Critical section exit
 ***********************************************************************
 * DESCRIPTION: The function is used to exit a critical section
 *              protected using Osal_cppiCsEnter() API.
 */
void Osal_rmCsExit(void *CsHandle)
{

}

/* FUNCTION PURPOSE: Critical section exit
 ***********************************************************************
 * DESCRIPTION: The function is used to indicate that a block of memory is
 *              about to be accessed. If the memory block is cached then this
 *              indicates that the application would need to ensure that the
 *              cache is updated with the data from the actual memory.
 */
void Osal_rmBeginMemAccess(void *ptr, uint32_t size)
{

}

/* FUNCTION PURPOSE: Critical section exit
 ***********************************************************************
 * DESCRIPTION: The function is used to indicate that the block of memory has
 *              finished being accessed. If the memory block is cached then the
 *              application would need to ensure that the contents of the cache
 *              are updated immediately to the actual memory.
 */
void Osal_rmEndMemAccess(void *ptr, uint32_t size)
{

}

/* FUNCTION PURPOSE: Creates a task blocking object
 ***********************************************************************
 * DESCRIPTION: The function is used to create a task blocking object
 *              capable of blocking the task a RM instance is running
 *              within
 */
void *Osal_rmTaskBlockCreate(void)
{
    /* No task blocking needed since there is only a RM Server */
    return(NULL);
}

/* FUNCTION PURPOSE: Blocks a RM instance
 ***********************************************************************
 * DESCRIPTION: The function is used to block a task whose context a
 *              RM instance is running within.
 */
void Osal_rmTaskBlock(void *handle)
{
    /* No task blocking needed since there is only a RM Server */
}

/* FUNCTION PURPOSE: unBlocks a RM instance
 ***********************************************************************
 * DESCRIPTION: The function is used to unblock a task whose context a
 *              RM instance is running within.
 */
void Osal_rmTaskUnblock(void *handle)
{
    /* No task blocking needed since there is only a RM Server */
}

/* FUNCTION PURPOSE: Deletes a task blocking object
 ***********************************************************************
 * DESCRIPTION: The function is used to delete a task blocking object
 *              provided to a RM instance
 */
void Osal_rmTaskBlockDelete(void *handle)
{
    /* No task blocking needed since there is only a RM Server */
}

/* FUNCTION PURPOSE: Prints a variable list
 ***********************************************************************
 * DESCRIPTION: The function is used to print a string to the console
 */
void Osal_rmLog (char *fmt, ... )
{
    va_list args;

    va_start (args, fmt);
    vprintf (fmt, args);
    va_end (args);
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
