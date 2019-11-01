/*
 *
 * Copyright (C) 2010-2014 Texas Instruments Incorporated - http://www.ti.com/
 *
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



/* Generate and verify the system test framework
 *
 * The test framework consists of the pa driver instance, a cppi/cdma/qm configuration,
 * memory for packet transmission and reception, and semaphores that are used
 * for every test in the PA unit test.
 *
 */

#include "../../pautest.h"

#include <ti/drv/pa/pa_osal.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

//#include <ti/csl/cslr_cp_ace.h>

/* CSL CHIP, SEM Functional layer includes */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cpsw.h>

/* Firmware images */
#include <ti/drv/pa/fw/pafw.h>
#include <ti/drv/qmss/qmss_firmware.h>

/**********************************************************************
 ****************************** Defines *******************************
 **********************************************************************/
#define     MAX_NUM_CORES       8

/* Hardware Semaphore to synchronize access from
 * multiple applications (PA applications and non-PASS applications)
 * across different cores to the QMSS library.
 */
#define     QMSS_HW_SEM         3

/* Hardware Semaphore to synchronize access from
 * multiple applications (PASS applications and non-PASS applications)
 * across different cores to the CPPI library.
 */
#define     CPPI_HW_SEM         4

/* Hardware Semaphore to synchronize access from
 * multiple applications (PASS applications and non-PASS applications)
 * across different cores to the PA library.
 */
#define     PA_HW_SEM           5

#undef L2_CACHE
#ifdef L2_CACHE
    /* Invalidate L2 cache. This should invalidate L1D as well.
     * Wait until operation is complete. */
#define SYS_CACHE_INV(addr, size, code)    CACHE_invL2 (addr, size, code)

    /* Writeback L2 cache. This should Writeback L1D as well.
     * Wait until operation is complete. */
#define SYS_CACHE_WB(addr, size, code)     CACHE_wbL2 (addr, size, code)

#else
    /* Invalidate L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
#define SYS_CACHE_INV(addr, size, code)    CACHE_invL1d (addr, size, code)
    /* Writeback L1D cache and wait until operation is complete.
     * Use this approach if L2 cache is not enabled */
#define SYS_CACHE_WB(addr, size, code)     CACHE_wbL1d (addr, size, code)

#endif


/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
uint32_t      qmssMallocCounter   = 0;
uint32_t      qmssFreeCounter     = 0;
uint32_t      cppiMallocCounter   = 0;
uint32_t      cppiFreeCounter     = 0;
uint32_t      paMemProtNestedLevel= 0;
uint32_t      rmMallocCounter     = 0;
uint32_t      rmFreeCounter       = 0;

uint32_t      coreKey [MAX_NUM_CORES];

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;
extern Qmss_GlobalConfigParams  qmssNetssGblCfgParams;

/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

#undef DBG_MULTI_CORE

/* Simulator debug level regs */
paLog_t *paLogLevel = (paLog_t *)PA_LOG_IF;


/**********************************************************************
 *************************** OSAL Functions **************************
 **********************************************************************/

/*
 * Netss Local PKTDMA related convert functions
 */

void* Netss_qmssVirtToPhy (void *ptr)
{
    uint32_t addr = (uint32_t) ptr;

    {
        if ((addr & 0xFF000000) == CSL_NETCP_CFG_REGS)
        {
            addr = (addr & 0x00FFFFFF) | 0xFF000000;
        }
    }

    return ((void *)addr);
}

void* Netss_qmssPhyToVirt (void *ptr)
{
    uint32_t addr = (uint32_t) ptr;

    {
        if ((addr & 0xFF000000) == 0xFF000000)
        {
            addr = (addr & 0x00FFFFFF) | CSL_NETCP_CFG_REGS;
        }
    }

    return ((void *)addr);
}

void* Netss_qmssConvertDescVirtToPhy(uint32_t QID, void *descAddr)
{
    uint32_t addr = (uint32_t) descAddr;

    {
        if ((addr & 0xFF000000) == CSL_NETCP_CFG_REGS)
        {
            addr = (addr & 0x00FFFFFF) | 0xFF000000;
        }
    }

    return ((void *)addr);
}

void* Netss_qmssConvertDescPhyToVirt(uint32_t QID, void *descAddr)
{
    uint32_t addr = (uint32_t) descAddr;

    {
        if ((addr & 0xFF000000) == 0xFF000000)
        {
            addr = (addr & 0x00FFFFFF) | CSL_NETCP_CFG_REGS;
        }
    }

    return ((void *)addr);
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
Ptr Osal_qmssMalloc (uint32_t num_bytes)
{
	Error_Block	errorBlock;
    Ptr dataPtr;

    /* Increment the allocation counter. */
    qmssMallocCounter++;

	/* Allocate memory. */
    dataPtr = Memory_alloc(NULL, num_bytes, 0, &errorBlock);
	return (dataPtr);
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
void Osal_qmssFree (Ptr ptr, uint32_t size)
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
 *  @param[in]  key
 *      Key used to lock the critical section.
 *
 *  @retval
 *      Not Applicable
 */
Ptr Osal_qmssCsEnter (void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core QMSS synchronization lock
     */
    while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);

    /* Disable all interrupts and OS scheduler.
     *
     * Acquire Multi threaded / process synchronization lock.
     */
    coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] = Hwi_disable();

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to exit a critical section
 *      protected using Osal_qmssCsEnter() API.
 *
 *  @param[in]  key
 *      Key used to unlock the critical section.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_qmssCsExit (Ptr CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    Hwi_restore(coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)]);

    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
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
 * ============================================================================
 *  @n@b Osal_qmssMtCsEnter
 *
 *  @b  brief
 *  @n  This API ensures ONLY multi-threaded
 *      synchronization to the QMSS user.
 *
 *      This is a BLOCKING API.
 *
 *  @param[in] None
 *
 *  @return
 *       Handle used to lock critical section
 * =============================================================================
 */
Ptr Osal_qmssMtCsEnter (void)
{
    /* Disable all interrupts and OS scheduler.
     *
     * Acquire Multi threaded / process synchronization lock.
     */
    //coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] = Hwi_disable();

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_qmssMtCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_cpswQmssMtCsEnter ()
 *      API. It resets the multi-threaded lock, enabling another process
 *      on the current core to grab it.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
void Osal_qmssMtCsExit (Ptr CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    //Hwi_restore(key);

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the QMSS OSAL Logging API which logs
 *      the messages on the console.
 *
 *  @param[in]  fmt
 *      Formatted String.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_qmssLog ( String fmt, ... )
{
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be invalidated
 *
 *  @param[in]  size
 *       Size of the block to be invalidated

 *  @retval
 *      Not Applicable
 */
void Osal_qmssBeginMemAccess (void *blockPtr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

    SYS_CACHE_INV (blockPtr, size, CACHE_FENCE_WAIT);

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
 *  @param[in]  blockPtr
 *       Address of the block which is to be written back
 *
 *  @param[in]  size
 *       Size of the block to be written back

 *  @retval
 *      Not Applicable
 */
void Osal_qmssEndMemAccess (void *blockPtr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    SYS_CACHE_WB (blockPtr, size, CACHE_FENCE_WAIT);

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
 *
 *      Note: If the LLD is used by applications on multiple core, the "cppiHeap"
 *      should be in shared memory
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Allocated block address
 */
Ptr Osal_cppiMalloc (uint32_t num_bytes)
{
	Error_Block	errorBlock;
    Ptr dataPtr;

    /* Increment the allocation counter. */
    cppiMallocCounter++;

	/* Allocate memory. */
    dataPtr = Memory_alloc(NULL, num_bytes, 0, &errorBlock);
	return (dataPtr);
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
void Osal_cppiFree (Ptr ptr, uint32_t size)
{
    /* Increment the free counter. */
    cppiFreeCounter++;
	Memory_free (NULL, ptr, size);
}

/**
 * ============================================================================
 *  @n@b Osal_cppiCsEnter
 *
 *  @b  brief
 *  @n  This API ensures multi-core and multi-threaded
 *      synchronization to the caller.
 *
 *      This is a BLOCKING API.
 *
 *      This API ensures multi-core synchronization between
 *      multiple processes trying to access CPPI shared
 *      library at the same time.
 *
 *  @param[in]
 *  @n  None
 *
 *  @return
 *  @n  Handle used to lock critical section
 * =============================================================================
 */
Ptr Osal_cppiCsEnter (void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (CPPI_HW_SEM)) == 0);

    /* Disable all interrupts and OS scheduler.
     *
     * Acquire Multi threaded / process synchronization lock.
     */
    coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] = Hwi_disable();

    return NULL;
}

/**
 * ============================================================================
 *  @n@b Osal_cppiCsExit
 *
 *  @b  brief
 *  @n  This API needs to be called to exit a previously
 *      acquired critical section lock using @a Osal_cppiCsEnter ()
 *      API. It resets the multi-core and multi-threaded lock,
 *      enabling another process/core to grab CPPI access.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @return     None
 * =============================================================================
 */
void Osal_cppiCsExit (Ptr CsHandle)
{
    /* Enable all interrupts and enables the OS scheduler back on.
     *
     * Release multi-threaded / multi-process lock on this core.
     */
    Hwi_restore(coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)]);

    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (CPPI_HW_SEM);

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the CPPI OSAL Logging API which logs
 *      the messages on the console.
 *
 *  @param[in]  fmt
 *      Formatted String.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_cppiLog ( String fmt, ... )
{
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  blockPtr
 *       Address of the block which is to be invalidated
 *
 *  @param[in]  size
 *       Size of the block to be invalidated

 *  @retval
 *      Not Applicable
 */
void Osal_cppiBeginMemAccess (void *blockPtr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

    SYS_CACHE_INV (blockPtr, size, CACHE_FENCE_WAIT);

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
 *  @param[in]  blockPtr
 *       Address of the block which is to be written back
 *
 *  @param[in]  size
 *       Size of the block to be written back

 *  @retval
 *      Not Applicable
 */
void Osal_cppiEndMemAccess (void *blockPtr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    SYS_CACHE_WB (blockPtr, size, CACHE_FENCE_WAIT);

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);
    return;
}


/**
 * @brief  This macro is used to alert the application that the PA is
 *         going to access table memory. The application must ensure
 *         cache coherency and semaphores for multi-core applications
 *
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_paBeginMemAccess (void* addr, uint32_t sizeWords)
    @endverbatim
 *
 *  <b> Parameters </b>
 *  @n  The address of the table to be accessed
 *  @n  The number of bytes in the table
 *
 *  @note PA will make nested calls to this function for memory access
 *        protection of different memory tables. The multicore semaphore
 *        should be allocated only for the first call of a nested group
 *        of calls.
 */


void Osal_paBeginMemAccess (Ptr addr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    /* Cleanup the prefetch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();

    SYS_CACHE_INV (addr, size, CACHE_FENCE_WAIT);

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);

}

/**
 * @brief  This macro is used to alert the application that the PA
 *         has completed access to table memory. This call will always
 *         be made following a call to Osal_paBeginMemAccess and have
 *         the same parameters
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_paEndMemAccess (void* addr, uint32_t sizeWords)
    @endverbatim
 *
 *  <b> Parameters </b>
 *  @n The address of the table to be accessed
 *  @n The number of bytes in the table
 *
 *  @note PA will make nested calls to this function for memory access
 *        protection of different memory tables. The multicore semaphore
 *        should be freed when all previous memory access has completed,
 *        in other words, when the nested call level reaches 0.
 */

void Osal_paEndMemAccess (Ptr addr, uint32_t size)
{
    uint32_t    key;

    /* Disable Interrupts */
    key = Hwi_disable();

    SYS_CACHE_WB (addr, size, CACHE_FENCE_WAIT);

    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");
    asm   (" nop      4");

    /* Reenable Interrupts. */
    Hwi_restore(key);

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

    /* Get the hardware semaphore.
     *
     * Acquire Multi core PA synchronization lock
     */
     while ((CSL_semAcquireDirect (PA_HW_SEM)) == 0);
     *key = 0;
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
    /* Release the hardware semaphore */
    CSL_semReleaseSemaphore (PA_HW_SEM);
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
void *Osal_rmMalloc (uint32_t num_bytes)
{
	Error_Block	errorBlock;

    /* Increment the allocation counter. */
    rmMallocCounter++;

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

    /* Reenable Interrupts. */
    Hwi_restore(key);

    return;
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

    /* Reenable Interrupts. */
    Hwi_restore(key);

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a task blocking object
 *      capable of blocking the task a RM instance is running
 *      within
 *
 *  @retval
 *      Allocated task blocking object
 */
void *Osal_rmTaskBlockCreate(void)
{
    Semaphore_Params semParams;

    Semaphore_Params_init(&semParams);
    return((void *)Semaphore_create(0, &semParams, NULL));
}

/**
 *  @b Description
 *  @n
 *      The function is used to block a task whose context a
 *      RM instance is running within.
 *
 *  @param[in]  handle
 *      Task blocking object handle.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmTaskBlock(void *handle)
{
    Semaphore_pend((Semaphore_Handle)handle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *      The function is used to unblock a task whose context a
 *      RM instance is running within.
 *
 *  @param[in]  handle
 *      Task blocking object handle.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmTaskUnblock(void *handle)
{
    Semaphore_post((Semaphore_Handle)handle);
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a task blocking object
 *      provided to a RM instance
 *
 *  @param[in]  handle
 *      Task blocking object handle.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmTaskBlockDelete(void *handle)
{
    Semaphore_delete((Semaphore_Handle *)&handle);
}

/**
 *  @b Description
 *  @n
 *      The function is the RM OSAL Logging API which logs
 *      the messages on the console.
 *
 *  @param[in]  fmt
 *      Formatted String.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmLog (char *fmt, ... )
{
    VaList ap;

    va_start(ap, fmt);
    System_vprintf(fmt, ap);
    va_end(ap);
    System_flush();
}


int downloadPaFirmware (void)
{

  int ret = pa_OK, i;
  uint32_t  version;

  Pa_resetControl (tFramework.passHandle, pa_STATE_RESET);

#ifndef NSS_GEN2

  /* PDPSs 0-2 use image c1 */
  Pa_downloadImage (tFramework.passHandle, 0, (Ptr)c1_0, c1_0Size);
  Pa_downloadImage (tFramework.passHandle, 1, (Ptr)c1_1, c1_1Size);
  Pa_downloadImage (tFramework.passHandle, 2, (Ptr)c1_2, c1_2Size);

  /* PDSP 3 uses image c2 */
  Pa_downloadImage (tFramework.passHandle, 3, (Ptr)c2, c2Size);

  /* PDSPs 4-5 use image m */
  for (i = 4; i < 6; i++)
    Pa_downloadImage (tFramework.passHandle, i, (Ptr)m, mSize);

#else

  Pa_downloadImage (tFramework.passHandle, 0, (Ptr)in0_pdsp0, in0_pdsp0Size);
  Pa_downloadImage (tFramework.passHandle, 1, (Ptr)in0_pdsp1, in0_pdsp1Size);
  Pa_downloadImage (tFramework.passHandle, 2, (Ptr)in1_pdsp0, in1_pdsp0Size);
  Pa_downloadImage (tFramework.passHandle, 3, (Ptr)in1_pdsp1, in1_pdsp1Size);
  Pa_downloadImage (tFramework.passHandle, 4, (Ptr)in2_pdsp0, in2_pdsp0Size);
  Pa_downloadImage (tFramework.passHandle, 5, (Ptr)in3_pdsp0, in3_pdsp0Size);
  Pa_downloadImage (tFramework.passHandle, 6, (Ptr)in4_pdsp0, in4_pdsp0Size);
  Pa_downloadImage (tFramework.passHandle, 7, (Ptr)in4_pdsp1, in4_pdsp1Size);
  Pa_downloadImage (tFramework.passHandle, 8, (Ptr)post_pdsp0, post_pdsp0Size);
  Pa_downloadImage (tFramework.passHandle, 9, (Ptr)post_pdsp1, post_pdsp1Size);
  Pa_downloadImage (tFramework.passHandle, 10, (Ptr)eg0_pdsp0, eg0_pdsp0Size);
  Pa_downloadImage (tFramework.passHandle, 11, (Ptr)eg0_pdsp1, eg0_pdsp1Size);
  Pa_downloadImage (tFramework.passHandle, 12, (Ptr)eg0_pdsp2, eg0_pdsp2Size);
  Pa_downloadImage (tFramework.passHandle, 13, (Ptr)eg1_pdsp0, eg1_pdsp0Size);
  Pa_downloadImage (tFramework.passHandle, 14, (Ptr)eg2_pdsp0, eg2_pdsp0Size);

#endif

  ret = Pa_resetControl (tFramework.passHandle, pa_STATE_ENABLE);

  if (ret != pa_STATE_ENABLE)
  {
    System_printf ("downloadPaFirmware: Pa_resetControl return with error code %d\n", ret);
    System_flush();
    //return (-1);
  }

  for ( i = 0; i < TF_NUM_PDSPS; i++)
  {
    Pa_getPDSPVersion(tFramework.passHandle, i, &version);
    System_printf ("PDSP %d version = 0x%08x\n", i, version);
    System_flush();
  }

  return (0);

}

/* The PA LLD instance is created, the PA firmware is
 * downloaded and started */
int initPa (void)
{
  paSizeInfo_t  paSize;
  paConfig_t    paCfg;
  paRaConfig_t  raCfg;
  paTimestampConfig_t tsCfg;
  int           ret;
  int           sizes[pa_N_BUFS];
  int           aligns[pa_N_BUFS];
  void*         bases[pa_N_BUFS];


  /* The maximum number of handles that can exists are 32 for L2, and 64 for L3. */
  memset(&paSize, 0, sizeof(paSizeInfo_t));
  memset(&paCfg, 0, sizeof(paConfig_t));

  memset(bases, 0, sizeof(bases));
  memset(sizes, 0, sizeof(sizes));
  memset(aligns, 0, sizeof(aligns));
  memset(&raCfg, 0, sizeof(paRaConfig_t));
  paSize.nMaxL2 = TF_MAX_NUM_L2_HANDLES;
  paSize.nMaxL3 = TF_MAX_NUM_L3_HANDLES;
  paSize.nMaxVlnk = TF_MAX_NUM_VLINK_HANDLES;
  paSize.nUsrStats = pa_USR_STATS_MAX_COUNTERS;

#ifdef NSS_GEN2

  paSize.nMaxAcl  = TF_MAX_NUM_ACL_HANDLES;
  paSize.nMaxFc   = TF_MAX_NUM_FC_HANDLES;
  paSize.nMaxEoam = TF_MAX_NUM_EOAM_HANDLES;
#endif

  ret = Pa_getBufferReq(&paSize, sizes, aligns);

  if (ret != pa_OK)  {
    System_printf ("initPa: Pa_getBufferReq() return with error code %d\n", ret);
    return (-1);
  }

  /* The first buffer is used as the instance buffer */
  if ((uint32_t)memPaInst & (aligns[pa_BUF_INST] - 1))  {
    System_printf ("initPa: Pa_getBufferReq requires %d alignment for instance buffer, but address is 0x%08x\n", aligns[pa_BUF_INST], (uint32_t)memPaInst);
    return (-1);
  }

  if (sizeof(memPaInst) < sizes[pa_BUF_INST])  {
    System_printf ("initPa: Pa_getBufferReq requires size %d for instance buffer, have only %d\n", sizes[pa_BUF_INST], sizeof(memPaInst));
    return (-1);
  }

  bases[pa_BUF_INST] = (void *)memPaInst;


  /* The second buffer is the L2 table */
  if ((uint32_t)memL2Ram & (aligns[pa_BUF_L2_TABLE] - 1))  {
    System_printf ("initPa: Pa_getBufferReq requires %d alignment for L2 buffer, but address is 0x%08x\n", aligns[pa_BUF_L2_TABLE], (uint32_t)memL2Ram);
    return (-1);
  }

  if (sizeof(memL2Ram) <  sizes[pa_BUF_L2_TABLE])  {
    System_printf ("initPa: Pa_getBufferReq requires %d bytes for L2 buffer, have only %d\n", sizes[pa_BUF_L2_TABLE], sizeof(memL2Ram));
    return (-1);
  }

  bases[pa_BUF_L2_TABLE] = (void *)memL2Ram;

  /* The third buffer is the L3 table */
  if ((uint32_t)memL3Ram & (aligns[pa_BUF_L3_TABLE] - 1))  {
    System_printf ("initPa: Pa_getBufferReq requires %d alignment for L3 buffer, but address is 0x%08x\n", aligns[pa_BUF_L3_TABLE], (uint32_t)memL3Ram);
    return (-1);
  }

  if (sizeof(memL3Ram) <  sizes[pa_BUF_L3_TABLE])  {
    System_printf ("initPa: Pa_getBufferReq requires %d bytes for L3 buffer, have only %d\n", sizes[pa_BUF_L3_TABLE], sizeof(memL3Ram));
    return (-1);
  }

  bases[pa_BUF_L3_TABLE] = (void *)memL3Ram;

  /* The fourth buffer is the User Statistics Link table */
  if ((uint32_t)memUsrStatsLnkTbl & (aligns[pa_BUF_USR_STATS_TABLE] - 1))  {
    System_printf ("initPa: Pa_getBufferReq requires %d alignment for User Statistics buffer, but address is 0x%08x\n", aligns[pa_BUF_USR_STATS_TABLE], (uint32_t)memUsrStatsLnkTbl);
    return (-1);
  }

  if (sizeof(memUsrStatsLnkTbl) <  sizes[pa_BUF_USR_STATS_TABLE])  {
    System_printf ("initPa: Pa_getBufferReq requires %d bytes for User Statistics buffer, have only %d\n", sizes[pa_BUF_USR_STATS_TABLE], sizeof(memUsrStatsLnkTbl));
    return (-1);
  }

  bases[pa_BUF_USR_STATS_TABLE] = (void *)memUsrStatsLnkTbl;


  /* The fifth buffer is the Virtual Link table */
  if ((uint32_t)memVLinkRam & (aligns[pa_BUF_VLINK_TABLE] - 1))  {
    System_printf ("initPa: Pa_getBufferReq requires %d alignment for Virtual Link buffer, but address is 0x%08x\n", aligns[pa_BUF_VLINK_TABLE], (uint32_t)memVLinkRam);
    return (-1);
  }

  if (sizeof(memVLinkRam) <  sizes[pa_BUF_VLINK_TABLE])  {
    System_printf ("initPa: Pa_getBufferReq requires %d bytes for Virtual Link buffer, have only %d\n", sizes[pa_BUF_VLINK_TABLE], sizeof(memVLinkRam));
    return (-1);
  }

  bases[pa_BUF_VLINK_TABLE] = (void *)memVLinkRam;

#ifdef NSS_GEN2

  /* The 6th buffer is the ACL table */
  if ((uint32_t)memAclRam & (aligns[pa_BUF_ACL_TABLE] - 1))  {
    System_printf ("initPa: Pa_getBufferReq requires %d alignment for ACL buffer, but address is 0x%08x\n", aligns[pa_BUF_ACL_TABLE], (uint32_t)memAclRam);
    return (-1);
  }

  if (sizeof(memAclRam) <  sizes[pa_BUF_ACL_TABLE])  {
    System_printf ("initPa: Pa_getBufferReq requires %d bytes for ACL buffer, have only %d\n", sizes[pa_BUF_ACL_TABLE], sizeof(memAclRam));
    return (-1);
  }

  bases[pa_BUF_ACL_TABLE] = (void *)memAclRam;

  /* The 7th buffer is the Flow Cache table */
  if ((uint32_t)memFcRam & (aligns[pa_BUF_FC_TABLE] - 1))  {
    System_printf ("initPa: Pa_getBufferReq requires %d alignment for Flow Cache buffer, but address is 0x%08x\n", aligns[pa_BUF_FC_TABLE], (uint32_t)memFcRam);
    return (-1);
  }

  if (sizeof(memFcRam) <  sizes[pa_BUF_FC_TABLE])  {
    System_printf ("initPa: Pa_getBufferReq requires %d bytes for Flow Cache buffer, have only %d\n", sizes[pa_BUF_FC_TABLE], sizeof(memFcRam));
    return (-1);
  }

  bases[pa_BUF_FC_TABLE] = (void *)memFcRam;

  /* The 8th buffer is the Eoam table */
  if ((uint32_t)memEoamRam & (aligns[pa_BUF_EOAM_TABLE] - 1))  {
    System_printf ("initPa: Pa_getBufferReq requires %d alignment for Ethernet OAM buffer, but address is 0x%08x\n", aligns[pa_BUF_EOAM_TABLE], (uint32_t)memEoamRam);
    return (-1);
  }

  if (sizeof(memEoamRam) <  sizes[pa_BUF_EOAM_TABLE])  {
    System_printf ("initPa: Pa_getBufferReq requires %d bytes for Ethernet OAM buffer, have only %d\n", sizes[pa_BUF_EOAM_TABLE], sizeof(memEoamRam));
    return (-1);
  }

  bases[pa_BUF_EOAM_TABLE] = (void *)memEoamRam;

  /* set default RA system configuration */
  raCfg.ipv4MinPktSize      = 28;   /* 20-byte IPv4 header plus 8-byte payload */
  raCfg.numCxts             = 250;
  raCfg.cxtDiscardThresh    = 250;
  raCfg.nodeDiscardThresh   = 1000;
  raCfg.cxtTimeout          = 60000;
  raCfg.clockRate           = 350;
  raCfg.heapRegionThresh    = 250;
#ifndef SIMULATOR_SUPPORT
  raCfg.heapBase[0]         = 0xFF000000UL;
#else
  raCfg.heapBase[0]         = 0x80000000UL;
#endif

  paCfg.raCfg = &raCfg;
#endif
  paCfg.initTable = TRUE;
#ifndef SIMULATOR_SUPPORT
  paCfg.initDefaultRoute = TRUE;
#endif
  paCfg.instPoolBaseAddr = (void *)memPaInst;
  paCfg.baseAddr = CSL_NETCP_CFG_REGS;
  paCfg.sizeCfg = &paSize;

#if RM
    paCfg.rmServiceHandle = rmServiceHandle;
#endif /* RM */

  ret = Pa_create (&paCfg, bases, &tFramework.passHandle);
  if (ret != pa_OK)  {
    System_printf ("initPa: Pa_create returned with error code %d\n", ret);
    return (-1);
  }

  /* Download the firmware */
  if (downloadPaFirmware ())
    return (-1);

  /* Enable Timer for timestamp */
  memset(&tsCfg, 0, sizeof(paTimestampConfig_t));
  tsCfg.enable = TRUE;
  tsCfg.factor = pa_TIMESTAMP_SCALER_FACTOR_2;

  if(Pa_configTimestamp(tFramework.passHandle, &tsCfg) != pa_OK)
    return (-1);

   return (0);

}

int setupQmMem (void)
{
  Qmss_InitCfg     qmssInitConfig;
  Qmss_MemRegInfo  memInfo;
  Cppi_DescCfg     descCfg;
  int32_t            result;
  int              n;

  memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));
  memset (memDescRam,      0, sizeof (memDescRam));

  //qmssInitConfig.linkingRAM0Base = utilgAddr((uint32_t)memLinkRam);
  qmssInitConfig.linkingRAM0Base = 0;
  qmssInitConfig.linkingRAM0Size = TF_NUM_DESC;
  qmssInitConfig.linkingRAM1Base = 0;
  qmssInitConfig.maxDescNum      = TF_NUM_DESC;

  qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
#ifdef _LITTLE_ENDIAN
  qmssInitConfig.pdspFirmware[0].firmware = (void *) &acc48_le;
  qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_le);
#else
  qmssInitConfig.pdspFirmware[0].firmware = (void *) &acc48_be;
  qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_be);
#endif

#if RM
    if (rmServiceHandle)
		qmssGblCfgParams.qmRmServiceHandle = rmServiceHandle;
#endif

  result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
  if (result != QMSS_SOK)  {
    System_printf ("setupQmMem: Qmss_Init failed with error code %d\n", result);
    return (-1);
  }

  result = Qmss_start();
  if (result != QMSS_SOK)  {
    System_printf ("setupQmMem: Qmss_start failed with error code %d\n", result);
    return (-1);
  }

  /* Setup a single memory region for descriptors */
  memset(&memInfo, 0, sizeof(memInfo));
  memset (memDescRam, 0, sizeof(memDescRam));
  memInfo.descBase       = (uint32_t *)utilgAddr((uint32_t)memDescRam);
  memInfo.descSize       = TF_SIZE_DESC;
  memInfo.descNum        = TF_NUM_DESC;
  memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
  memInfo.memRegion      = Qmss_MemRegion_MEMORY_REGION0;
  memInfo.startIndex     = 0;

  result = Qmss_insertMemoryRegion (&memInfo);
  if (result < QMSS_SOK)  {
    System_printf ("setupQmMem: Qmss_insertMemoryRegion returned error code %d\n", result);
    return (-1);
  }


  /* Initialize the descriptors. This function opens a general
   * purpose queue and intializes the memory from region 0, placing
   * the initialized descriptors onto that queue */
  memset(&descCfg, 0, sizeof(descCfg));
  descCfg.queueGroup        = 0;
  descCfg.memRegion         = Qmss_MemRegion_MEMORY_REGION0;
  descCfg.descNum           = TF_NUM_DESC;
  descCfg.destQueueNum      = TF_Q_FREE_DESC;
  descCfg.queueType         = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
  descCfg.initDesc          = Cppi_InitDesc_INIT_DESCRIPTOR;
  descCfg.descType          = Cppi_DescType_HOST;
  descCfg.returnQueue.qNum  = QMSS_PARAM_NOT_SPECIFIED;
  descCfg.returnQueue.qMgr  = 0;
  descCfg.epibPresent       = Cppi_EPIB_EPIB_PRESENT;

  //descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
  descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_BUFFER;
  descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

  tFramework.QfreeDesc = Cppi_initDescriptor (&descCfg, (uint32_t *)&n);

  if (n != descCfg.descNum)  {
    System_printf ("setupQmMem: expected %d descriptors to be initialized, only %d are initialized\n", descCfg.descNum, n);
    return (-1);
  }

  return (0);

}

int setupPassQmMem (void)
{

#ifdef NETSS_INTERNAL_PKTDMA

  Qmss_InitCfg     qmssInitConfig;
  Qmss_StartCfg    qmssStartConfig;
  Qmss_MemRegInfo  memInfo;
  Cppi_DescCfg     descCfg;
  int32_t          result;
  int              n;

  memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));
  memset (&qmssStartConfig, 0, sizeof (Qmss_StartCfg));

  //qmssInitConfig.linkingRAM0Base = utilgAddr((uint32_t)memLinkRam);  // It should be 0x0 for internal RAM
  qmssInitConfig.linkingRAM0Base = 0;
  qmssInitConfig.linkingRAM0Size = TF_NUM_DESC;
  qmssInitConfig.linkingRAM1Base = 0;
  qmssInitConfig.maxDescNum      = TF_NUM_DESC;

  // Supply virtual-2-physical conversion functions
  qmssNetssGblCfgParams.virt2Phy     = Netss_qmssVirtToPhy;
  qmssNetssGblCfgParams.phy2Virt     = Netss_qmssPhyToVirt;
  qmssNetssGblCfgParams.virt2PhyDesc = Netss_qmssConvertDescVirtToPhy;
  qmssNetssGblCfgParams.phy2VirtDesc = Netss_qmssConvertDescPhyToVirt;

  result = Qmss_initSubSys (&tFramework.tfPaQmssHandle, Qmss_SubSys_NETSS, &qmssInitConfig, &qmssNetssGblCfgParams);
  if (result != QMSS_SOK)  {
    System_printf ("setupPassQmMem: Qmss_Init failed with error code %d\n", result);
    return (-1);
  }

  result = Qmss_startSubSysCfg(&tFramework.tfPaQmssHandle, Qmss_SubSys_NETSS, &qmssStartConfig);
  if (result != QMSS_SOK)  {
    System_printf ("setupPassQmMem: Qmss_start failed with error code %d\n", result);
    return (-1);
  }

  /* Setup a single memory region for descriptors */
  memset(&memInfo, 0, sizeof(memInfo));
  memset (passDescRam, 0, TF_SIZE_DESC*TF_NUM_DESC);
  memInfo.descBase       = (uint32_t *)(passDescRam);
  memInfo.descSize       = TF_SIZE_DESC;
  memInfo.descNum        = TF_NUM_DESC;
  memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
  memInfo.memRegion      = Qmss_MemRegion_MEMORY_REGION0;
  memInfo.startIndex     = 0;

  result = Qmss_insertMemoryRegionSubSys (tFramework.tfPaQmssHandle, &memInfo);
  if (result < QMSS_SOK)  {
    System_printf ("setupQmMem: Qmss_insertMemoryRegion returned error code %d\n", result);
    return (-1);
  }

  /* Initialize the descriptors. This function opens a general
   * purpose queue and intializes the memory from region 0, placing
   * the initialized descriptors onto that queue */
  memset(&descCfg, 0, sizeof(descCfg));
  descCfg.queueGroup        = 0;
  descCfg.memRegion         = Qmss_MemRegion_MEMORY_REGION0;
  descCfg.descNum           = TF_NUM_DESC;
  descCfg.destQueueNum      = TF_Q_LOC_FREE_DESC;
  descCfg.queueType         = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
  descCfg.initDesc          = Cppi_InitDesc_INIT_DESCRIPTOR;
  descCfg.descType          = Cppi_DescType_HOST;
  descCfg.returnQueue.qNum  = QMSS_PARAM_NOT_SPECIFIED;
  descCfg.returnQueue.qMgr  = 0;
  descCfg.epibPresent       = Cppi_EPIB_EPIB_PRESENT;

  //descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
  descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_BUFFER;
  descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

  tFramework.QLocfreeDesc = Cppi_initDescriptorSubSys (tFramework.tfPaQmssHandle, &descCfg, (uint32_t *)&n);

  if (n != descCfg.descNum)  {
    System_printf ("setupPassQmMem: expected %d descriptors to be initialized, only %d are initialized\n", descCfg.descNum, n);
    return (-1);
  }
#endif

  return (0);

}

int setupCpdma (void)
{
  Cppi_CpDmaInitCfg cpdmaCfg;
  Cppi_RxChInitCfg  rxChCfg;
  Cppi_TxChInitCfg  txChCfg;
#if RM
  Cppi_StartCfg     cppiStartCfg;
#endif

  int32_t result;
  int   i;
  uint8_t isAlloc;

  result = Cppi_init (&cppiGblCfgParams);
  if (result != CPPI_SOK)  {
    System_printf ("setupCpdma: cpp_Init returned error %d\n", result);
    return (-1);
  }

  memset(&cpdmaCfg, 0, sizeof(Cppi_CpDmaInitCfg));
  cpdmaCfg.dmaNum           = Cppi_CpDma_NETCP_CPDMA;

  tFramework.tfPaCppiHandle = Cppi_open (&cpdmaCfg);
  if (tFramework.tfPaCppiHandle == NULL)  {
    System_printf ("setupCpdma: cppi_Open returned NULL PA cppi handle\n");
    return (-1);
  }

#if RM
  if (rmServiceHandle)
  {
     cppiStartCfg.rmServiceHandle = rmServiceHandle;
     Cppi_startCfg(&cppiStartCfg);
  }
#endif


#ifdef NETSS_INTERNAL_PKTDMA

  memset(&cpdmaCfg, 0, sizeof(Cppi_CpDmaInitCfg));
  cpdmaCfg.dmaNum           = Cppi_CpDma_NETCP_LOCAL_CPDMA;
  cpdmaCfg.qm0BaseAddress   = 0xff1b8000;                    // will CSL definition
  cpdmaCfg.qm1BaseAddress   = 0xff1b8400;                    // will CSL definition
  cpdmaCfg.qm2BaseAddress   = 0xff1b8000;                    // will CSL definition
  cpdmaCfg.qm3BaseAddress   = 0xff1b8400;                    // will CSL definition

  tFramework.tfPaLocCppiHandle = Cppi_open (&cpdmaCfg);
  if (tFramework.tfPaLocCppiHandle == NULL)  {
    System_printf ("setupCpdma: cppi_Open returned NULL PA local cppi handle\n");
    return (-1);
  }

#endif

  /* Open all rx channels */
  rxChCfg.rxEnable = Cppi_ChState_CHANNEL_DISABLE;

  for (i = 0; i < TF_PA_NUM_RX_CPDMA_CHANNELS; i++)  {
    rxChCfg.channelNum        = i;
    tFramework.tfPaRxChHnd[i] = Cppi_rxChannelOpen (tFramework.tfPaCppiHandle, &rxChCfg, &isAlloc);

    if (tFramework.tfPaRxChHnd[i] == NULL)  {
      System_printf ("setupCpdma: cppi_RxChannelOpen returned NULL handle for channel number %d\n", i);
      return (-1);
    }

    Cppi_channelEnable (tFramework.tfPaRxChHnd[i]);
  }

  /* Open all tx channels.  */
  txChCfg.priority     = 2;
  txChCfg.txEnable     = Cppi_ChState_CHANNEL_DISABLE;
  txChCfg.filterEPIB   = FALSE;
  txChCfg.filterPS     = FALSE;
  txChCfg.aifMonoMode  = FALSE;


  for (i = 0; i < TF_PA_NUM_TX_CPDMA_CHANNELS; i++)  {
    txChCfg.channelNum 	 	  = i;
    tFramework.tfPaTxChHnd[i] = Cppi_txChannelOpen (tFramework.tfPaCppiHandle, &txChCfg, &isAlloc);

    if (tFramework.tfPaTxChHnd[i] == NULL)  {
      System_printf ("setupCpdma: cppi_TxChannelOpen returned NULL handle for channel number %d\n", i);
      return (-1);
    }

    Cppi_channelEnable (tFramework.tfPaTxChHnd[i]);
  }

  /* Clear CPPI Loobpack bit in PASS CDMA Global Emulation Control Register */
  Cppi_setCpdmaLoopback(tFramework.tfPaCppiHandle, 0);

#ifdef NETSS_INTERNAL_PKTDMA

  /* Open all local rx channels */
  rxChCfg.rxEnable = Cppi_ChState_CHANNEL_DISABLE;

  for (i = 0; i < TF_PA_NUM_RX_CPDMA_CHANNELS; i++)  {
    rxChCfg.channelNum        = i;
    tFramework.tfPaLocRxChHnd[i] = Cppi_rxChannelOpen (tFramework.tfPaLocCppiHandle, &rxChCfg, &isAlloc);

    if (tFramework.tfPaLocRxChHnd[i] == NULL)  {
      System_printf ("setupCpdma: cppi_RxChannelOpen returned NULL handle for local rx channel number %d\n", i);
      return (-1);
    }

    Cppi_channelEnable (tFramework.tfPaLocRxChHnd[i]);
  }

  /* Open all locL tx channels.  */
  txChCfg.priority     = 2;
  txChCfg.txEnable     = Cppi_ChState_CHANNEL_DISABLE;
  txChCfg.filterEPIB   = FALSE;
  txChCfg.filterPS     = FALSE;
  txChCfg.aifMonoMode  = FALSE;


  for (i = 0; i < TF_PA_NUM_TX_CPDMA_CHANNELS; i++)  {
    txChCfg.channelNum 	 	  = i;
    tFramework.tfPaLocTxChHnd[i] = Cppi_txChannelOpen (tFramework.tfPaLocCppiHandle, &txChCfg, &isAlloc);

    if (tFramework.tfPaLocTxChHnd[i] == NULL)  {
      System_printf ("setupCpdma: cppi_TxChannelOpen returned NULL handle for local tx channel number %d\n", i);
      return (-1);
    }

    Cppi_channelEnable (tFramework.tfPaLocTxChHnd[i]);
  }

  /* Clear CPPI Loobpack bit in PASS CDMA Global Emulation Control Register */
  Cppi_setCpdmaLoopback(tFramework.tfPaLocCppiHandle, 0);

#endif

  return (0);

}


/* Setup all the queues used in the example */
int setupQueues (void)
{
  int           i;
  uint8_t       isAlloc;
  Qmss_Queue    q;
  Cppi_HostDesc *hd;


  /* The 10 PA transmit queues (corresponding to the 10 tx cdma channels */
  for (i = 0; i < TF_PA_NUM_TX_CPDMA_CHANNELS; i++)  {

    tFramework.QPaTx[i] = Qmss_queueOpen (Qmss_QueueType_PASS_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAlloc);

    if (tFramework.QPaTx[i] < 0)  {
      System_printf ("setupQueues: Qmss_queueOpen failed for PA transmit queue number %d\n", TF_PA_TX_QUEUE_BASE+i);
      return (-1);
    }

    Qmss_setQueueThreshold (tFramework.QPaTx[i], 1, 1);
  }


  /* The default return queue for descriptors with linked buffers */
  tFramework.QDefRet = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_DEF_RET_Q, &isAlloc);
  if (tFramework.QDefRet < 0)  {
    System_printf ("setupQueues: Qmss_queueOpen failed for queue %d\n", TF_DEF_RET_Q);
    return (-1);
  }

  /* The queues with attached buffers */
  tFramework.QLinkedBuf1 = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_LINKED_BUF_Q1, &isAlloc);

  if (tFramework.QLinkedBuf1 < 0)  {
  	System_printf ("setupQueues: Qmss_queueOpen failed for queue %d\n", TF_LINKED_BUF_Q1);
  	return (-1);
  }

  tFramework.QLinkedBuf2 = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_LINKED_BUF_Q2, &isAlloc);

  if (tFramework.QLinkedBuf2 < 0)  {
  	System_printf ("SetupQueues: Qmss_queueOpen failed for queue %d\n", TF_LINKED_BUF_Q2);
  	return (-1);
  }

  tFramework.QLinkedBuf3 = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_LINKED_BUF_Q3, &isAlloc);

  if (tFramework.QLinkedBuf3 < 0)  {
  	System_printf ("SetupQueues: Qmss_queueOpen failed for queue %d\n", TF_LINKED_BUF_Q3);
  	return (-1);
  }

  /* Attach buffers to the queues and push them onto the queue */
  q.qMgr = 0;

  q.qNum = TF_LINKED_BUF_Q1;
  for (i = 0; i < TF_LINKED_BUF_Q1_NBUFS; i++)   {

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tFramework.QfreeDesc)) & ~15);
    if (hd == NULL)  {
      System_printf ("setupQueues: Qmss_queuePop returned NULL on pop from queue number %d\n", tFramework.QfreeDesc);
      return (-1);
    }

    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ1[i])), sizeof(memQ1[i]));
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ1[i])), sizeof(memQ1[i]));
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ1[i])), sizeof(memQ1[i]));
    hd->nextBDPtr = NULL;
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
    Qmss_queuePushDesc (tFramework.QLinkedBuf1, (Ptr)hd);

  }

  q.qNum = TF_LINKED_BUF_Q2;
  for (i = 0; i < TF_LINKED_BUF_Q2_NBUFS; i++)   {

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tFramework.QfreeDesc)) & ~15);
    if (hd == NULL)  {
      System_printf ("setupQueues: Qmss_queuePop returned NULL on pop from queue number %d\n", tFramework.QfreeDesc);
      return (-1);
    }

    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ2[i])), sizeof(memQ2[i]));
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ2[i])), sizeof(memQ2[i]));
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ2[i])), sizeof(memQ2[i]));
    hd->nextBDPtr = NULL;
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
    Qmss_queuePushDesc (tFramework.QLinkedBuf2, (Ptr)hd);

  }

  q.qNum = TF_LINKED_BUF_Q3;
  for (i = 0; i < TF_LINKED_BUF_Q3_NBUFS; i++)   {

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tFramework.QfreeDesc)) & ~15);
    if (hd == NULL)  {
      System_printf ("setupQueues: Qmss_queuePop returned NULL on pop from queue number %d\n", tFramework.QfreeDesc);
      return (-1);
    }

    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ3[i])), sizeof(memQ3[i]));
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ3[i])), sizeof(memQ3[i]));
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)(memQ3[i])), sizeof(memQ3[i]));
    hd->nextBDPtr = NULL;
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
    Qmss_queuePushDesc (tFramework.QLinkedBuf3, (Ptr)hd);

  }

  tFramework.QCommonCmdRep = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_COMMON_CMD_REPL_Q, &isAlloc);
  if (tFramework.QCommonCmdRep < 0)  {
  	  System_printf ("SetupQueues: Qmss_queueOpen failed for queue %d\n", TF_COMMON_CMD_REPL_Q);
  	  return (-1);
  }



  /* General purpose queues */
  for (i = 0; i < TF_NUM_GEN_QUEUES; i++)  {

  	tFramework.QGen[i] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_FIRST_GEN_QUEUE + i, &isAlloc);

 	  if (tFramework.QGen[i] < 0)  {
  	  System_printf ("SetupQueues: Qmss_queueOpen failed for queue %d\n", TF_FIRST_GEN_QUEUE + i);
  	  return (-1);
  	}
  }

#ifdef NETSS_INTERNAL_PKTDMA

  /* The queues with attached buffers */
  tFramework.QLocLinkedBuf1 = Qmss_queueOpenSubSys (tFramework.tfPaQmssHandle, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_LOC_LINKED_BUF_Q1, &isAlloc);

  if (tFramework.QLinkedBuf1 < 0)  {
  	System_printf ("setupQueues: Qmss_queueOpenSubSys failed for queue %d\n", TF_LOC_LINKED_BUF_Q1);
  	return (-1);
  }

  tFramework.QLocLinkedBuf2 = Qmss_queueOpenSubSys (tFramework.tfPaQmssHandle, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_LOC_LINKED_BUF_Q2, &isAlloc);

  if (tFramework.QLinkedBuf2 < 0)  {
  	System_printf ("SetupQueues: Qmss_queueOpenSubSys failed for queue %d\n", TF_LOC_LINKED_BUF_Q2);
  	return (-1);
  }

  tFramework.QLocLinkedBuf3 = Qmss_queueOpenSubSys (tFramework.tfPaQmssHandle, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, TF_LOC_LINKED_BUF_Q3, &isAlloc);

  if (tFramework.QLinkedBuf3 < 0)  {
  	System_printf ("SetupQueues: Qmss_queueOpenSubSys failed for queue %d\n", TF_LOC_LINKED_BUF_Q3);
  	return (-1);
  }

  /* Attach buffers to the queues and push them onto the queue */
  q.qMgr = 0;

  q.qNum = TF_LOC_LINKED_BUF_Q1;
  for (i = 0; i < TF_LINKED_BUF_Q1_NBUFS; i++)   {

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tFramework.QLocfreeDesc)) & ~15);
    if (hd == NULL)  {
      System_printf ("setupQueues: Qmss_queuePop returned NULL on pop from queue number %d\n", tFramework.QLocfreeDesc);
      return (-1);
    }

    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ1[i], sizeof(memLocQ1[i]));
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ1[i], sizeof(memLocQ1[i]));
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ1[i], sizeof(memLocQ1[i]));
    hd->nextBDPtr = NULL;
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
    Qmss_queuePushDesc (tFramework.QLocLinkedBuf1, (Ptr)hd);

  }

  q.qNum = TF_LOC_LINKED_BUF_Q2;
  for (i = 0; i < TF_LINKED_BUF_Q2_NBUFS; i++)   {

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tFramework.QLocfreeDesc)) & ~15);
    if (hd == NULL)  {
      System_printf ("setupQueues: Qmss_queuePop returned NULL on pop from queue number %d\n", tFramework.QLocfreeDesc);
      return (-1);
    }

    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ2[i], sizeof(memLocQ2[i]));
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ2[i], sizeof(memLocQ2[i]));
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ2[i], sizeof(memLocQ2[i]));
    hd->nextBDPtr = NULL;
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
    Qmss_queuePushDesc (tFramework.QLocLinkedBuf2, (Ptr)hd);

  }

  q.qNum = TF_LOC_LINKED_BUF_Q3;
  for (i = 0; i < TF_LINKED_BUF_Q3_NBUFS; i++)   {

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tFramework.QLocfreeDesc)) & ~15);
    if (hd == NULL)  {
      System_printf ("setupQueues: Qmss_queuePop returned NULL on pop from queue number %d\n", tFramework.QLocfreeDesc);
      return (-1);
    }

    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ3[i], sizeof(memLocQ3[i]));
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ3[i], sizeof(memLocQ3[i]));
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)memLocQ3[i], sizeof(memLocQ3[i]));
    hd->nextBDPtr = NULL;
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
    Qmss_queuePushDesc (tFramework.QLocLinkedBuf3, (Ptr)hd);

  }

#endif

  return (0);

}

/* Configure flows */
int setupFlows (void)
{
  Cppi_RxFlowCfg  rxFlowCfg;
  uint8_t           isAlloc;
  int i;

  /* Configure Rx flow */
  rxFlowCfg.flowIdNum      = CPPI_PARAM_NOT_SPECIFIED;
  rxFlowCfg.rx_dest_qnum   = TF_FIRST_GEN_QUEUE + TF_NUM_GEN_QUEUES -1;   /* Override in PA */
  rxFlowCfg.rx_dest_qmgr   = 0;
  rxFlowCfg.rx_sop_offset  = 0;
  rxFlowCfg.rx_ps_location = Cppi_PSLoc_PS_IN_DESC;
  rxFlowCfg.rx_desc_type   = Cppi_DescType_HOST;
  rxFlowCfg.rx_error_handling = 1;
  rxFlowCfg.rx_psinfo_present = 1;
  rxFlowCfg.rx_einfo_present  = 1;

  rxFlowCfg.rx_dest_tag_lo = 0;
  rxFlowCfg.rx_dest_tag_hi = 0;
  rxFlowCfg.rx_src_tag_lo  = 0;
  rxFlowCfg.rx_src_tag_hi  = 0;

  rxFlowCfg.rx_size_thresh0_en = 1;
  rxFlowCfg.rx_size_thresh1_en = 1;
  rxFlowCfg.rx_size_thresh2_en = 0;
  rxFlowCfg.rx_dest_tag_lo_sel = 4;
  rxFlowCfg.rx_dest_tag_hi_sel = 0;
  rxFlowCfg.rx_src_tag_lo_sel  = 0;
  rxFlowCfg.rx_src_tag_hi_sel  = 0;

  rxFlowCfg.rx_fdq1_qnum = tFramework.QLinkedBuf2;
  rxFlowCfg.rx_fdq1_qmgr = 0;
  rxFlowCfg.rx_fdq0_sz0_qnum = tFramework.QLinkedBuf1;
  rxFlowCfg.rx_fdq0_sz0_qmgr = 0;

  rxFlowCfg.rx_fdq3_qnum = tFramework.QLinkedBuf2;
  rxFlowCfg.rx_fdq3_qmgr = 0;
  rxFlowCfg.rx_fdq2_qnum = tFramework.QLinkedBuf2;
  rxFlowCfg.rx_fdq2_qmgr = 0;

  rxFlowCfg.rx_size_thresh1 = TF_LINKED_BUF_Q2_BUF_SIZE;
  rxFlowCfg.rx_size_thresh0 = TF_LINKED_BUF_Q1_BUF_SIZE;

  rxFlowCfg.rx_fdq0_sz1_qnum = tFramework.QLinkedBuf2;
  rxFlowCfg.rx_fdq0_sz1_qmgr = 0;
  rxFlowCfg.rx_size_thresh2  = 0;

  rxFlowCfg.rx_fdq0_sz3_qnum = tFramework.QLinkedBuf3;
  rxFlowCfg.rx_fdq0_sz3_qmgr = 0;
  rxFlowCfg.rx_fdq0_sz2_qnum = tFramework.QLinkedBuf3;
  rxFlowCfg.rx_fdq0_sz2_qmgr = 0;

  tFramework.tfPaFlowHnd[0] = Cppi_configureRxFlow (tFramework.tfPaCppiHandle, &rxFlowCfg, &isAlloc);
  if (tFramework.tfPaFlowHnd[0] == NULL)  {
    System_printf ("setupFlows: cppi_ConfigureRxFlow returned NULL on flow 0\n");
    return (-1);
  }

  tFramework.tfFlowNum[0] = (uint8_t)Cppi_getFlowId(tFramework.tfPaFlowHnd[0]);
  /* Create multiple identical flow for multi-interface testing */
  for ( i = 1; i < 4; i++)
  {
    rxFlowCfg.flowIdNum    =  CPPI_PARAM_NOT_SPECIFIED;
    tFramework.tfPaFlowHnd[i] = Cppi_configureRxFlow (tFramework.tfPaCppiHandle, &rxFlowCfg, &isAlloc);
    if (tFramework.tfPaFlowHnd[i] == NULL)  {
        System_printf ("setupFlows: cppi_ConfigureRxFlow returned NULL on flow %d\n", i);
        return (-1);
    }
    tFramework.tfFlowNum[i] = (uint8_t)Cppi_getFlowId(tFramework.tfPaFlowHnd[i]);
  }

  /* Rx Flow for Outer IP RA */
  rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;
  rxFlowCfg.rx_dest_qnum   = TF_FIRST_GEN_QUEUE + 6;
  rxFlowCfg.rx_src_tag_lo  = 1;
  rxFlowCfg.rx_src_tag_lo_sel  = 1;

  tFramework.tfPaFlowHnd1 = Cppi_configureRxFlow (tFramework.tfPaCppiHandle, &rxFlowCfg, &isAlloc);
  if (tFramework.tfPaFlowHnd1 == NULL)  {
    System_printf ("setupFlows: cppi_ConfigureRxFlow returned NULL on flow 1\n");
    return (-1);
  }
  tFramework.tfFlowNum1 = (uint8_t)Cppi_getFlowId(tFramework.tfPaFlowHnd1);

  /* Rx Flow for Inner IP RA */
  rxFlowCfg.flowIdNum      = CPPI_PARAM_NOT_SPECIFIED;
  rxFlowCfg.rx_dest_qnum   = TF_FIRST_GEN_QUEUE + 7;
  rxFlowCfg.rx_src_tag_lo  = 2;
  rxFlowCfg.rx_src_tag_lo_sel  = 1;

  tFramework.tfPaFlowHnd2 = Cppi_configureRxFlow (tFramework.tfPaCppiHandle, &rxFlowCfg, &isAlloc);
  if (tFramework.tfPaFlowHnd2 == NULL)  {
    System_printf ("setupFlows: cppi_ConfigureRxFlow returned NULL on flow 2\n");
    return (-1);
  }

  tFramework.tfFlowNum2 = (uint8_t)Cppi_getFlowId(tFramework.tfPaFlowHnd2);

  #ifdef NETSS_INTERNAL_PKTDMA

  /* Configure Local Rx flow */
  rxFlowCfg.flowIdNum      = CPPI_PARAM_NOT_SPECIFIED;
  rxFlowCfg.rx_dest_qnum   = 0;   /* Override in PA */
  rxFlowCfg.rx_dest_qmgr   = 0;
  rxFlowCfg.rx_sop_offset  = 0;
  rxFlowCfg.rx_ps_location = Cppi_PSLoc_PS_IN_DESC;
  rxFlowCfg.rx_desc_type   = Cppi_DescType_HOST;
  rxFlowCfg.rx_error_handling = 0;
  rxFlowCfg.rx_psinfo_present = 1;
  rxFlowCfg.rx_einfo_present  = 1;

  rxFlowCfg.rx_dest_tag_lo = 0;
  rxFlowCfg.rx_dest_tag_hi = 0;
  rxFlowCfg.rx_src_tag_lo  = 0;
  rxFlowCfg.rx_src_tag_hi  = 0;

  rxFlowCfg.rx_size_thresh0_en = 1;
  rxFlowCfg.rx_size_thresh1_en = 1;
  rxFlowCfg.rx_size_thresh2_en = 0;
  rxFlowCfg.rx_dest_tag_lo_sel = 4;
  rxFlowCfg.rx_dest_tag_hi_sel = 0;
  rxFlowCfg.rx_src_tag_lo_sel  = 0;
  rxFlowCfg.rx_src_tag_hi_sel  = 0;

  rxFlowCfg.rx_fdq1_qnum = TF_LOC_LINKED_BUF_Q2;
  rxFlowCfg.rx_fdq1_qmgr = 0;
  rxFlowCfg.rx_fdq0_sz0_qnum = TF_LOC_LINKED_BUF_Q1;
  rxFlowCfg.rx_fdq0_sz0_qmgr = 0;

  rxFlowCfg.rx_fdq3_qnum = TF_LOC_LINKED_BUF_Q2;
  rxFlowCfg.rx_fdq3_qmgr = 0;
  rxFlowCfg.rx_fdq2_qnum = TF_LOC_LINKED_BUF_Q2;
  rxFlowCfg.rx_fdq2_qmgr = 0;

  rxFlowCfg.rx_size_thresh1 = TF_LINKED_BUF_Q2_BUF_SIZE;
  rxFlowCfg.rx_size_thresh0 = TF_LINKED_BUF_Q1_BUF_SIZE;

  rxFlowCfg.rx_fdq0_sz1_qnum = TF_LOC_LINKED_BUF_Q2;
  rxFlowCfg.rx_fdq0_sz1_qmgr = 0;
  rxFlowCfg.rx_size_thresh2  = 0;

  rxFlowCfg.rx_fdq0_sz3_qnum = TF_LOC_LINKED_BUF_Q3;
  rxFlowCfg.rx_fdq0_sz3_qmgr = 0;
  rxFlowCfg.rx_fdq0_sz2_qnum = TF_LOC_LINKED_BUF_Q3;
  rxFlowCfg.rx_fdq0_sz2_qmgr = 0;

  tFramework.tfPaLocFlowHnd0 = Cppi_configureRxFlow (tFramework.tfPaLocCppiHandle, &rxFlowCfg, &isAlloc);
  if (tFramework.tfPaLocFlowHnd0 == NULL)  {
    System_printf ("setupFlows: cppi_ConfigureRxFlow returned NULL on local flow 0\n");
    return (-1);
  }
  tFramework.tfLocFlowNum = (uint8_t)Cppi_getFlowId(tFramework.tfPaLocFlowHnd0);

  #endif

  return (0);

}

/* Clear flows */
int clearFlows (void)
{
    int  ret, i;

    for (i = 0; i < 4; i++)
    {
        if ((ret = Cppi_closeRxFlow (tFramework.tfPaFlowHnd[i])) != CPPI_SOK)
        {
            System_printf ("clearFlows: Cppi_closeRxFlow returned error code (%d) for global flow id %d\n", ret, tFramework.tfFlowNum[i]);
            return (-1);
        }
    }

    if ((ret = Cppi_closeRxFlow (tFramework.tfPaFlowHnd1)) != CPPI_SOK)
    {
        System_printf ("clearFlows: Cppi_closeRxFlow returned error code (%d) for global flow id %d\n", ret, tFramework.tfFlowNum1);
        return (-1);
    }

    if ((ret = Cppi_closeRxFlow (tFramework.tfPaFlowHnd2)) != CPPI_SOK)
    {
        System_printf ("clearFlows: Cppi_closeRxFlow returned error code (%d) for global flow id %d\n", ret, tFramework.tfFlowNum2);
        return (-1);
    }

    #ifdef NETSS_INTERNAL_PKTDMA
    if ((ret = Cppi_closeRxFlow (tFramework.tfPaLocFlowHnd0)) != CPPI_SOK)
    {
        System_printf ("clearFlows: Cppi_closeRxFlow returned error code (%d) for local flow id %d\n", ret, tFramework.tfLocFlowNum);
        return (-1);
    }
    #endif

    return (0);
}

int closeQueues(void) {

	  int i;
	  tFramework_t *tf=&tFramework;

	  /* Clean and close all PASS transmit queues (corresponding to the tx cdma channels */
	  for (i = 0; i < TF_PA_NUM_TX_CPDMA_CHANNELS; i++)  {
	  	 Qmss_queueEmpty (tf->QPaTx[i]);
	  	 Qmss_queueClose (tf->QPaTx[i]);
	  }

	  /* Empty the remaining queues */
	  Qmss_queueEmpty (tf->QfreeDesc);
	  Qmss_queueEmpty (tf->QDefRet);
	  Qmss_queueEmpty (tf->QLinkedBuf1);
	  Qmss_queueEmpty (tf->QLinkedBuf2);
	  Qmss_queueEmpty (tf->QLinkedBuf3);
	  Qmss_queueEmpty (tf->QCommonCmdRep);

	  /* Close the remaining queues */
	  Qmss_queueClose (tf->QfreeDesc);
	  Qmss_queueClose (tf->QDefRet);
	  Qmss_queueClose (tf->QLinkedBuf1);
	  Qmss_queueClose (tf->QLinkedBuf2);
	  Qmss_queueClose (tf->QLinkedBuf3);
	  Qmss_queueClose (tf->QCommonCmdRep);

	  /* Empty General purpose queues */
	  for (i = 0; i < TF_NUM_GEN_QUEUES; i++)  {
		  Qmss_queueEmpty (tf->QGen[i]);
		  Qmss_queueClose (tf->QGen[i]);
	  }

#ifdef NETSS_INTERNAL_PKTDMA
	  /* Empty the remaining queues */
	  Qmss_queueEmpty (tf->QLocfreeDesc);
	  Qmss_queueEmpty (tf->QLocLinkedBuf1);
	  Qmss_queueEmpty (tf->QLocLinkedBuf2);
	  Qmss_queueEmpty (tf->QLocLinkedBuf3);

	  /* Close the remaining queues */
	  Qmss_queueClose (tf->QLocfreeDesc);
	  Qmss_queueClose (tf->QLocLinkedBuf1);
	  Qmss_queueClose (tf->QLocLinkedBuf2);
	  Qmss_queueClose (tf->QLocLinkedBuf3);

#endif

    return (0);

}

/* Clear  PKTDMA*/
int clearCpdma (void)
{
    int  i, ret;

    /* Close the cpDma setup */
    for (i = 0; i < TF_PA_NUM_RX_CPDMA_CHANNELS; i++)  {
	    if ((ret = Cppi_channelClose (tFramework.tfPaRxChHnd[i])) != CPPI_SOK) {
            System_printf ("clearCpdma: Cppi_channelClose returned error code (%d) for PASS rx channel %d\n", ret, i);
	 	    return (-1);
	    }
    }
    for (i = 0; i < TF_PA_NUM_TX_CPDMA_CHANNELS; i++)  {
	    if ((ret = Cppi_channelClose (tFramework.tfPaTxChHnd[i])) != CPPI_SOK) {
            System_printf ("clearCpdma: Cppi_channelClose returned error code (%d) for PASS tx channel %d\n", ret, i);
	 	    return (-1);
	    }
    }

#ifdef NETSS_INTERNAL_PKTDMA

    /* Close the local cpDma setup */
    for (i = 0; i < TF_PA_NUM_RX_CPDMA_CHANNELS; i++)  {
	    if ((ret = Cppi_channelClose (tFramework.tfPaLocRxChHnd[i])) != CPPI_SOK) {
            System_printf ("clearCpdma: Cppi_channelClose returned error code (%d) for PASS local rx channel %d\n", ret, i);
	 	    return (-1);
	    }
    }
    for (i = 0; i < TF_PA_NUM_TX_CPDMA_CHANNELS; i++)  {
	    if ((ret = Cppi_channelClose (tFramework.tfPaLocTxChHnd[i])) != CPPI_SOK) {
            System_printf ("clearCpdma: Cppi_channelClose returned error code (%d) for PASS local tx channel %d\n", ret, i);
	 	    return (-1);
	    }
    }

#endif

    return (0);

}


/* The QM/CPDMA are cleared */
int clearQm(void)
{
  int result;

  /* clear the flows */
  if (clearFlows ())  {
  	System_printf ("clearQm: clearFlows failed\n");
    return (-1);
  }

  /* Close the queues that were setup */
  if (closeQueues())  {
  	System_printf ("clearQm: closeQueues failed\n");
    return (-1);
  }

  if (clearCpdma ())  {
  	System_printf ("clearQm: clearCpdma failed\n");
    return (-1);
  }

   /* Free the memory regions */
   if ((result = Qmss_removeMemoryRegion (Qmss_MemRegion_MEMORY_REGION0, 0)) != QMSS_SOK)
   {
       System_printf ("clearQm: Remove QMSS memory region error code : %d\n", result);
       return (-1);
   }

#ifdef NETSS_INTERNAL_PKTDMA

   /* Free the PASS Local QMSS memory regions */
   if ((result = Qmss_removeMemoryRegionSubSys (Qmss_SubSys_NETSS, Qmss_MemRegion_MEMORY_REGION0, 0)) != QMSS_SOK)
   {
       System_printf ("clearQm: Remove PASS internal QMSS memory region error code : %d\n", result);
       return (-1);
   }

   /* Exit QMSS */
   if ((result = Qmss_exitSubSys (&tFramework.tfPaQmssHandle)) != QMSS_SOK)
   {
       System_printf ("clearQm: PASS internal QMSS_exit error code : %d\n", result);
       return (-1);
   }

#endif

   /* Exit QMSS */
   if ((result = Qmss_exit ()) != QMSS_SOK)
   {
       System_printf ("clearQm: QMSS_exit error code : %d\n", result);
       return (-1);
   }

   return (0);

}

/* The QM/CPDMA are setup */
int initQm (void)
{
  if (setupQmMem())  {
  	System_printf ("initQm: setupQmMem failed\n");
    return (-1);
  }

  if (setupPassQmMem())  {
  	System_printf ("initQm: setupPassQmMem failed\n");
    return (-1);
  }

  if (setupCpdma ())  {
  	System_printf ("initQm: setupCpdma failed\n");
    return (-1);
  }

  if (setupQueues ())  {
  	System_printf ("initQm: setupQueues failed\n");
    return (-1);
  }

  if (setupFlows ())  {
  	System_printf ("initQm: setupFlows failed\n");
    return (-1);
  }

  return (0);

}


int initSems (void)
{
#if 0
/* TBD: no longer used */
	Semaphore_Params params;
	Error_Block      eb;

	Semaphore_Params_init (&params);
	params.mode = Semaphore_Mode_BINARY;

 #endif

	return (0);
}

/***************************************************************************************
 * FUNCTION PURPOSE: Power up PA subsystem
 ***************************************************************************************
 * DESCRIPTION: this function powers up the PA subsystem domains
 ***************************************************************************************/
void passPowerUp (void)
{

    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any
     * PASS device register access. This not required for the simulator. */

    /* Set PASS Power domain to ON */
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_NETCP);

    /* Enable the clocks for PASS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_PA,      PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_CPGMAC,  PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_SA,      PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_NETCP);

    /* Wait until the state transition process is completed. */
    utilCycleDelay (1000);
    //while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_NETCP));

#ifdef SOC_K2L
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_OSR);

    /* Enable the clocks for OSR modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_OSR, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_OSR);

    /* Wait until the state transition process is completed. */
    utilCycleDelay (1000);
#endif

}

/* Function to clear the command set use index and exception index use for PA */
void clearPaInfo(void)
{
    memset(&tFramework, 0, sizeof(tFramework_t));
}

/** ============================================================================
 *   @n@b Init_Switch
 *
 *   @b Description
 *   @n This API sets up the ethernet switch subsystem and its Address Lookup
 *      Engine (ALE) in "Switch" mode.
 *
 *   @param[in]
 *   @n mtu             Maximum Frame length to configure on the switch.
 *
 *   @return
 *   @n None
 * =============================================================================
 */
void initSwitch (uint32_t mtu)
{
    CSL_CPSW_PORTSTAT               portStatCfg;

    /* Enable the CPPI port, i.e., port 0 that does all
     * the data streaming in/out of EMAC.
     */
    CSL_CPSW_enablePort0 ();
    CSL_CPSW_disableVlanAware ();
    CSL_CPSW_setPort0VlanReg (0, 0, 0);
    CSL_CPSW_setPort0RxMaxLen (mtu);

    /* Enable statistics on both the port groups:
     *
     * MAC Sliver ports -   Port 1, Port 2
     * CPPI Port        -   Port 0
     */
    #if defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_C6678)
    portStatCfg.p0AStatEnable   =   1;
    portStatCfg.p0BStatEnable   =   1;
    portStatCfg.p1StatEnable    =   1;
    portStatCfg.p2StatEnable    =   1;
    #else
    portStatCfg.p0StatEnable    =   1;
    portStatCfg.p1StatEnable    =   1;
    portStatCfg.p2StatEnable    =   1;
    portStatCfg.p3StatEnable    =   1;
    portStatCfg.p4StatEnable    =   1;
    portStatCfg.p5StatEnable    =   1;
    portStatCfg.p6StatEnable    =   1;
    portStatCfg.p7StatEnable    =   1;
    portStatCfg.p8StatEnable    =   1;
    #endif
    CSL_CPSW_setPortStatsEnableReg (&portStatCfg);

    /* Setup the Address Lookup Engine (ALE) Configuration:
     *      (1) Enable ALE.
     *      (2) Clear stale ALE entries.
     *      (3) Disable VLAN Aware lookups in ALE since
     *          we are not using VLANs by default.
     *      (4) No Flow control
     *      (5) Configure the Unknown VLAN processing
     *          properties for the switch, i.e., which
     *          ports to send the packets to.
     */
    CSL_CPSW_enableAle ();
    CSL_CPSW_clearAleTable ();

    CSL_CPSW_disableAleVlanAware ();
    CSL_CPSW_disableAleTxRateLimit ();
    CSL_CPSW_setAlePrescaleReg (125000000u/1000u);
    CSL_CPSW_setAleUnkownVlanReg (7, 3, 3, 7);

    /* Done with switch configuration */
    return;
}

/* Initialize the test framework */
int setupTestFramework (void)
{
	GateHwi_Params prms;

	/* Create the HW disable gate. It is used by QM call backs */
	GateHwi_Params_init(&prms);
	tFramework.gateHwi = GateHwi_create(&prms, NULL);

    /* Power up PA sub-systems */
    passPowerUp();

	/* Clear the logs */
	clearPaInfo();

#if RM
    /* Setup the RM client */
    if (setupRm ())
    {
      System_printf ("setupTestFramework: setupRm returned error, exiting\n");
      System_flush();
      return (-1);
    }
#endif

    /* Setup the semaphores used for access to the PA tables.
	 * This has to be done before the PA is initialized */
	if (initSems())  {
		System_printf ("setupTestFramework: initQm returned error, exiting\n");
		return (-1);
	}

	/* Create the PA driver instance */
	if (initPa())  {
		System_printf ("setupTestFramework: initPa returned error, exiting\n");
		return (-1);
	}

	/* Setup the QM with associated buffers and descriptors */
	if (initQm())  {
		System_printf ("setupTestFramework: initQm returned error, exiting\n");
		return (-1);
	}

	return (0);

}

/* Clear the Test Framework */
int clearTestFramework (void)
{
	int retVal = 0;

    /* QM Clean ups */
	clearQm();

    return (retVal);
}

/* Check that all the queues are setup correctly */
int verifyTestFramework (void)
{
	int i, j;
	int count;
	int returnVal = 0;
	Cppi_HostDesc *hd;
	uint8_t *bufp;
	uint32_t bufLen;

	int32_t linkedQ[3];
	int32_t nbufs[]  = { TF_LINKED_BUF_Q1_NBUFS,    TF_LINKED_BUF_Q2_NBUFS,    TF_LINKED_BUF_Q3_NBUFS };
	int32_t bSize[]  = { TF_LINKED_BUF_Q1_BUF_SIZE, TF_LINKED_BUF_Q2_BUF_SIZE, TF_LINKED_BUF_Q3_BUF_SIZE };
    #ifdef NETSS_INTERNAL_PKTDMA
	int32_t linkedLocQ[3];
    #endif

	linkedQ[0] = tFramework.QLinkedBuf1;
	linkedQ[1] = tFramework.QLinkedBuf2;
	linkedQ[2] = tFramework.QLinkedBuf3;

    #ifdef NETSS_INTERNAL_PKTDMA
	linkedLocQ[0] = tFramework.QLocLinkedBuf1;
	linkedLocQ[1] = tFramework.QLocLinkedBuf2;
	linkedLocQ[2] = tFramework.QLocLinkedBuf3;
    #endif

    /* clear up the gobal PASS settings */
    if (testCommonSetDefaultGlobalConfig())
    {
		System_printf ("verifyTestFramework: testCommonSetDefaultGlobalConfig returned error!\n");
		returnVal = -1;
    }

    /* clean up exception routes */
    if (testExceptionSetRecover())
    {
		System_printf ("verifyTestFramework: testExceptionSetRecover returned error!\n");
		returnVal = -1;
    }

    /* clean up multi-route groups */
    if (testCommonMultiRouteRecover())
    {
		System_printf ("verifyTestFramework: testCommonMultiRouteRecover returned error!\n");
		returnVal = -1;
    }

	/* clean up command sets */
	if (testCommonCmdSetRecover()) {
		System_printf ("verifyTestFramework: testCommonCmdSetRecover returned error!\n");
		returnVal = -1;
	}

 	if (testCommonClearPaStats ())  {
 		System_printf ("verifyTestFramework: testCommonCmdSetRecover returned error!\n");
		returnVal = -1;
 	}


	/* Verify that all of the general purpose queues are empty */
	for (i = 0; i < TF_NUM_GEN_QUEUES; i++)  {
		if ((count = Qmss_getQueueEntryCount (tFramework.QGen[i])) != 0)  {
			System_printf ("verifyTestFramework: Expected 0 entry count for queue %d, found %d entries\n", tFramework.QGen[i], count);
			returnVal = -1;
		}
	}

	/* Verify that the number of descriptors in the free descriptor queue is correct */
	count = Qmss_getQueueEntryCount (tFramework.QfreeDesc);
	if (count != (TF_NUM_DESC - TF_LINKED_BUF_Q1_NBUFS - TF_LINKED_BUF_Q2_NBUFS - TF_LINKED_BUF_Q3_NBUFS))  {
		System_printf ("verifyTestFramework: Expected %d entry count in the free descriptor queue (%d), found %d\n",
						TF_NUM_DESC - TF_LINKED_BUF_Q1_NBUFS - TF_LINKED_BUF_Q2_NBUFS - TF_LINKED_BUF_Q3_NBUFS,
						tFramework.QfreeDesc, count);
		returnVal = -1;
	}

	/* Verify the number and sizing of descriptors with linked buffers in the three queues */
	for (j = 0; j < 3; j++)  {

		count = Qmss_getQueueEntryCount (linkedQ[j]);
		if (count != nbufs[j])  {
		    System_printf ("verifyTestFramework: Expected %d entry count in linked buffer queue 1 (%d), found %d\n",
						    nbufs[j], linkedQ[j], count);
		    returnVal = -1;
		}

		for (i = 0; i < count; i++)  {
			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (linkedQ[j])) & ~15);
			Cppi_getOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bufp, &bufLen);
			Qmss_queuePush (linkedQ[j], (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);

			if (bufLen != bSize[j])  {
				System_printf ("verifyTestFramework: Linked buffer queue %d (%d) expected orignal length of %d, found %d\n",
								j, linkedQ[j], bSize[j], bufLen);
				returnVal = -1;
				break;
			}
		}
	}

  #ifdef NETSS_INTERNAL_PKTDMA

	/* Verify that the number of descriptors in the free descriptor queue is correct */
	count = Qmss_getQueueEntryCount (tFramework.QLocfreeDesc);
	if (count != (TF_NUM_DESC - TF_LINKED_BUF_Q1_NBUFS - TF_LINKED_BUF_Q2_NBUFS - TF_LINKED_BUF_Q3_NBUFS))  {
		System_printf ("verifyTestFramework: Expected %d entry count in the free descriptor queue (%d), found %d\n",
						TF_NUM_DESC - TF_LINKED_BUF_Q1_NBUFS - TF_LINKED_BUF_Q2_NBUFS - TF_LINKED_BUF_Q3_NBUFS,
						tFramework.QLocfreeDesc, count);
		returnVal = -1;
	}



	/* Verify the number and sizing of descriptors with linked buffers in the three queues */
	for (j = 0; j < 3; j++)  {

		count = Qmss_getQueueEntryCount (linkedLocQ[j]);
		if (count != nbufs[j])  {
		System_printf ("verifyTestFramework: Expected %d entry count in Loc linked buffer queue %d (%d), found %d\n",
						nbufs[j], j, linkedQ[j], count);
		returnVal = -1;
		}

		for (i = 0; i < count; i++)  {
			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (linkedLocQ[j])) & ~15);
			Cppi_getOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, &bufp, &bufLen);
            //Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);
            Qmss_queuePushDesc(linkedLocQ[j], (Ptr)hd);

			if (bufLen != bSize[j])  {
				System_printf ("verifyTestFramework: Linked buffer queue %d (%d) expected orignal length of %d, found %d\n",
								j, linkedQ[j], bSize[j], bufLen);
				returnVal = -1;
				break;
			}
		}
	}

    #endif

	return (returnVal);
}

int  setupPktTestInfo(pktTestInfo_t* testInfo, int count, char* tfname)
{
  /* This is transperent function for the DSP *
   * Do not alter anything since everything needed is already in place */
  return (0);
}

int  setupIfPktTestInfo(ifPktTestInfo_t* testInfo, int count, char* tfname)
{
  /* This is transperent function for the DSP *
   * Do not alter anything since everything needed is already in place */
  return (0);
}

/* Nothing past this point */


