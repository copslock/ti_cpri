/**  
 * @file fw_test.h
 *
 * @brief 
 *  Holds all the constants and API definitions required by the example
 *  application to run.
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
*/
#ifndef _FW_TEST_H_
#define _FW_TEST_H_

/* C Standard library Include */
#include <stdio.h>
/* System level header files */
#include <stdint.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

#define System_printf   printf

#define CACHE_LINESZ    64

typedef uint16_t fw_Bool_t;
#define  fw_TRUE  1
#define  fw_FALSE  0

/* Mapping between /dev, /proc, memories allowed, and fd's */
#define FW_MAX_PATH 256


/* User supplied list of possible memory mapping devices in /dev, and
 * /proc/device-tree entry which lists the memory regions allowed */
typedef struct
{
    const char devPath[FW_MAX_PATH];
    const char procPath[FW_MAX_PATH];
} fwTestMemMapDevProc_t;

/* Total Permanent memory required in Example
 * for Packet buffers & descriptor buffers
 * Allocating entire MSMC
 */

/* Physical address map & size for various subsystems */
#define QMSS_CFG_BASE_ADDR         CSL_QMSS_CFG_BASE
#define QMSS_CFG_BLK_SZ            (0x00100000)
#define QMSS_DATA_BASE_ADDR        CSL_QMSS_DATA_BASE
#define QMSS_DATA_BLK_SZ           (0x00100000)
#define SRIO_CFG_BASE_ADDR         CSL_SRIO_CFG_REGS
#define SRIO_CFG_BLK_SZ            (0x00040000)
#define SRIO_SERDES_CFG_BASE_ADDR  CSL_SRIO_SERDES_CFG_REGS
#define SRIO_SERDES_CFG_BLK_SZ     (0x00002000)

/* Amount of CMA/CMEM needed */
#define FW_QMSS_CMEM_SIZE (1024*1024)

/* Global variables to hold virtual address of various subsystems */
extern void *fw_qmssCfgVaddr;
extern void *fw_qmssDataVaddr;
extern void *fw_srioCfgVaddr;
extern void *fw_srioSerdesCfgVaddr;

/** Enable Extended Debug with printfs */
//#define EXT_DEBUG 1

/* Invalidate  cache. This should invalidate Cache
 * Wait until operation is complete. Currently stub function
 */    
#define SYS_CACHE_INV(addr, size, code)  Osal_invalidateCache(addr,size)  

/* Writeback L2 cache. This should Writeback L1D as well. 
 * Wait until operation is complete. Currently stub function
 */ 
#define SYS_CACHE_WB(addr, size, code) Osal_writeBackCache(addr,size) 

/* GCC align attribute */
#define ALIGN(x)    __attribute__((aligned (x)))

/* Convert physical (36 bit) to IO virtual address */
#define FW_PLATFORM_PHYS2IOVIRT(x) ((x) - 0x780000000ull)

/* Prototypes */
void Osal_fwCsEnter (uint32_t *key);
void Osal_fwCsExit (uint32_t key);
void fw_osalInit();
void fw_osalshutdown();
int initRm();

int setupTestFramework (Cppi_Handle *cppiHnd);

int fw_task_create (void *(start_routine)(void *), void *args, void *handle);
void fw_task_wait (void *handle);

#endif

