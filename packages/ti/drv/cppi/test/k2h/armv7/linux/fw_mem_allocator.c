/******************************************************************************
 * FILE PURPOSE:  Memory allocator for running Example
 ******************************************************************************
 * FILE NAME:   fw_mem_allocator.c
 *
 * DESCRIPTION: Memory allocator for running test.
 * This is only a permanent memory allocator.
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#include "fw_mem_allocator.h"

/* Macro to align x to y */
#define align(x,y)   ((x + y) & (~y))

uint8_t *fw_mem_start_phy = (uint8_t*)0;
uint8_t *fw_mem_start = (uint8_t*)0;

static uint8_t *fw_mem_end = (uint8_t*)0;
static uint8_t *fw_mem_alloc_ptr = (uint8_t*)0;
static uint32_t fw_mem_size = 0;

/* File descriptor for /dev/mem */ 
static int dev_mem_fd;

/***************************************************************************************
 * FUNCTION PURPOSE: Initialize memory allocation
 ***************************************************************************************
 * DESCRIPTION: Initialize memory allocation
 ***************************************************************************************/
fw_Bool_t fw_memAllocInit
(
    uint8_t     *addr, /* Physical address */
    uint32_t    size   /* Size of block */
)
{
    void *map_base; 

    if((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
        printf("fw_memAllocInit: Failed to open \"dev/mem\" err=%s\n",
               strerror(errno));
        return fw_FALSE;
    }

    map_base = fw_memMap ((void *)addr, size); 

    if (!map_base)
    {
        printf("fw_memAllocInit: Failed to mmap addr (0x%p)", addr);
        return fw_FALSE;
    }

#ifdef EXT_DEBUG
    printf("fw_memAllocInit: Memory mapped at address %p.\n", map_base); 
#endif

    fw_mem_alloc_ptr = fw_mem_start = map_base;
    fw_mem_size = size;
    fw_mem_end = fw_mem_start + fw_mem_size;
    fw_mem_start_phy = addr;
    return fw_TRUE;
}
/***************************************************************************************
 * FUNCTION PURPOSE:  
 ***************************************************************************************
 * DESCRIPTION:  
 ***************************************************************************************/
void* fw_memAlloc
(
    uint32_t    size,
    uint32_t    align
)
{
    uint32_t key;
    uint8_t *alloc_ptr;
    void *p_block =NULL;

    Osal_fwCsEnter(&key);
    alloc_ptr = (uint8_t*)align((uint32_t)fw_mem_alloc_ptr, align);
    if ((alloc_ptr + size) < fw_mem_end)
    {
        p_block =(void *)alloc_ptr;
        fw_mem_alloc_ptr = alloc_ptr + size;
        Osal_fwCsExit(key);
        memset (p_block, 0, size);
    }
    else 
    {
        Osal_fwCsExit(key);
    }

#ifdef EXT_DEBUG
    printf("fw_memAlloc: Allocated address %p.\n", p_block); 
#endif

    return p_block;
}
/***************************************************************************************
 * FUNCTION PURPOSE:  Function to map physical address to virtual memory space 
 ***************************************************************************************
 * DESCRIPTION:  Function to map physical address to virtual memory space 
 ***************************************************************************************/
void *fw_memMap
(
    void        *addr, /* Physical address */
    uint32_t    size   /* Size of block */
)
{
    void            *map_base, *virt_addr;
    uint32_t        page_sz;
    long            retval;

    retval = sysconf(_SC_PAGE_SIZE);
    if (retval == -1)
    {
        printf("fw_memMap: Failed to get page size err=%s\n",
               strerror(errno));
        return (void *)0;
    }

    page_sz = (uint32_t)retval;

    if (size%page_sz)
    {
        printf("fw_memMap: error: block size not aligned to page size\n");
        return (void *)0;
    }

    if ((uint32_t)addr%page_sz)
    {
        printf("fw_memMap: error: addr not aligned to page size\n");
        return (void *)0;
    }

    map_base = mmap(0, size, (PROT_READ|PROT_WRITE), MAP_SHARED, dev_mem_fd, (off_t)addr);
    if(map_base == (void *) -1) 
    {
        printf("fw_memMap: Failed to mmap \"dev/mem\" err=%s\n",
               strerror(errno));
        return (void *)0;
    }
    virt_addr = map_base;
    return(virt_addr);
}

