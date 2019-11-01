/**
 *   @file  rm_linux_osal.c
 *
 *   @brief
 *      This is the OS abstraction layer used by the Resource Manager in Linux.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2015, Texas Instruments, Inc.
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

/* 
 * Shut off: remark #880-D: parameter "CsHandle, mtSemObj, etc." are function
 *           input parameters but not referenced if the OSAL function is not
 *           used.
 *
 * The input arguments can't be removed without breaking the OSAL usage
 */
#ifdef _TMS320C6X
#pragma diag_suppress 880
#pragma diag_suppress 681
#elif defined(__GNUC__)
/* Same for GCC:
 * warning: unused parameter CsHandle, mtSemObj, etc. [-Wunused-parameter]
 */
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

/* Standard Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <pthread.h>

/**********************************************************************
 ****************************** Defines *******************************
 **********************************************************************/

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
uint32_t rmMallocCounter = 0;
uint32_t rmFreeCounter   = 0;

int32_t rmByteAlloc = 0;
int32_t rmByteFree = 0;

/**********************************************************************
 *************************** OSAL Functions **************************
 **********************************************************************/

/* FUNCTION PURPOSE: Allocates memory
 ***********************************************************************
 * DESCRIPTION: The function is used to allocate a memory block of the
 *              specified size.
 */
void *Osal_rmMalloc (uint32_t num_bytes)
{
    /* Increment the allocation counter. */
    rmMallocCounter++;
    rmByteAlloc += num_bytes;

    /* Allocate memory. */
    return calloc(1, num_bytes);
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
    rmByteFree += size;
    free(ptr);
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
    return(NULL);
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
    pthread_mutex_t *mutex = (pthread_mutex_t *)mtSemObj;

    pthread_mutex_lock(mutex);
    return NULL;
}

/* FUNCTION PURPOSE: Multi-threaded critical section exit
 ***********************************************************************
 * DESCRIPTION: The function is used to exit a multi-threaded critical
 *              section protected using Osal_rmMtCsEnter() API.
 */  
void Osal_rmMtCsExit(void *mtSemObj, void *CsHandle)
{
    pthread_mutex_t *mutex = (pthread_mutex_t *)mtSemObj;
    
    pthread_mutex_unlock(mutex);
}

/* FUNCTION PURPOSE: Cache invalidate
 ***********************************************************************
 * DESCRIPTION: The function is used to indicate that a block of memory is 
 *              about to be accessed. If the memory block is cached then this 
 *              indicates that the application would need to ensure that the 
 *              cache is updated with the data from the actual memory.
 */  
void Osal_rmBeginMemAccess(void *ptr, uint32_t size)
{
    return;
}

/* FUNCTION PURPOSE: Cache writeback
 ***********************************************************************
 * DESCRIPTION: The function is used to indicate that the block of memory has 
 *              finished being accessed. If the memory block is cached then the 
 *              application would need to ensure that the contents of the cache 
 *              are updated immediately to the actual memory. 
 */  
void Osal_rmEndMemAccess(void *ptr, uint32_t size)
{
    return;
}

/* FUNCTION PURPOSE: Creates a task blocking object
 ***********************************************************************
 * DESCRIPTION: The function is used to create a task blocking object
 *              capable of blocking the task a RM instance is running
 *              within
 */
void *Osal_rmTaskBlockCreate(void)
{
    return(NULL);
}

/* FUNCTION PURPOSE: Blocks a RM instance
 ***********************************************************************
 * DESCRIPTION: The function is used to block a task whose context a
 *              RM instance is running within.
 */
void Osal_rmTaskBlock(void *handle)
{

}

/* FUNCTION PURPOSE: unBlocks a RM instance
 ***********************************************************************
 * DESCRIPTION: The function is used to unblock a task whose context a
 *              RM instance is running within.
 */
void Osal_rmTaskUnblock(void *handle)
{

}

/* FUNCTION PURPOSE: Deletes a task blocking object
 ***********************************************************************
 * DESCRIPTION: The function is used to delete a task blocking object
 *              provided to a RM instance
 */
void Osal_rmTaskBlockDelete(void *handle)
{

}

/* FUNCTION PURPOSE: Prints a variable list
 ***********************************************************************
 * DESCRIPTION: The function is used to print a string to the console
 */
void Osal_rmLog (char *fmt, ... )
{
    va_list ap;
    
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

