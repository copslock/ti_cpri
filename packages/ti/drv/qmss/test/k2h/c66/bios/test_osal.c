/**
 *   @file  test_osal.c
 *
 *   @brief   
 *      This is the OS abstraction layer and is used by the the QMSS LLD.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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
#include <stdarg.h>

#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/hal/Hwi.h>

#include <xdc/cfg/global.h>

/* CSL Semaphore module includes */
#include <ti/csl/csl_semAux.h>

/**********************************************************************
 ****************************** Defines *******************************
 **********************************************************************/

#define QMSS_HW_SEM     2
#define QMSS_ACC_HW_SEM 3

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
uint32_t qmssMallocCounter = 0;
uint32_t qmssFreeCounter   = 0;
uint32_t rmMallocCounter   = 0;
uint32_t rmFreeCounter     = 0;

/**********************************************************************
 *************************** OSAL Functions **************************
 **********************************************************************/

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
void* Osal_biosMalloc (uint32_t num_bytes)
{
    Error_Block errorBlock;

    /* Increment the allocation counter. */
    qmssMallocCounter++;

    /* Allocate memory. */
    return Memory_alloc(NULL, num_bytes, 0, &errorBlock);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to free a memory block of the specified size.
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
void Osal_biosFree (void *ptr, uint32_t size)
{
    /* Increment the free counter. */
    qmssFreeCounter++;
    Memory_free(NULL, ptr, size);
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
    /* Get the hardware semaphore */
    while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);

    /* Create Semaphore for protection against access from multiple threads 
     * Not created here becasue application is not multithreaded 
     * */
    return NULL;

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
void* Osal_qmssAccCsEnter (void)
{
    /* Get the hardware semaphore */
    while ((CSL_semAcquireDirect (QMSS_ACC_HW_SEM)) == 0);

    /* Create Semaphore for protection against access from multiple threads 
     * Not created here becasue application is not multithreaded 
     * */
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
void Osal_qmssCsExit (void *CsHandle)
{
    /* Release Semaphore using handle */

    /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (QMSS_HW_SEM);

    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to exit a critical section 
 *      protected using Osal_qmssAccCsEnter() API.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_qmssAccCsExit (void *CsHandle)
{
    /* Release Semaphore using handle */

    /* Release the hardware semaphore */ 
    CSL_semReleaseSemaphore (QMSS_ACC_HW_SEM);

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
void* Osal_qmssMtCsEnter (void)
{
    /* Create Semaphore for protection against access from multiple threads 
     * Not created here becasue application is not multithreaded 
     * */
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
void Osal_qmssMtCsExit (void *CsHandle)
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
    /* QMSS data is located in L2 memory. 
     * No update is required */
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
    /* QMSS data is located in L2 memory. 
     * No update is required */
    return;
}

/* FUNCTION PURPOSE: Allocates memory
 ***********************************************************************
 * DESCRIPTION: The function is used to allocate a memory block of the
 *              specified size.
 */
void *Osal_rmMalloc (uint32_t num_bytes)
{
    Error_Block errorBlock;

    /* Increment the allocation counter. */
    rmMallocCounter++;

    /* Allocate memory. */
    return Memory_alloc(NULL, num_bytes, 0, &errorBlock);
}

/* FUNCTION PURPOSE: Frees memory
 ***********************************************************************
 * DESCRIPTION: The function is used to free a memory block of the
 *              specified size.
 */ 
void Osal_rmFree (void *ptr, uint32_t size)
{
    /* Increment the free counter. */
    rmFreeCounter++;
    Memory_free(NULL, ptr, size);
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
    VaList ap;
    
    va_start(ap, fmt);
    System_vprintf(fmt, ap);
    va_end(ap);
}

