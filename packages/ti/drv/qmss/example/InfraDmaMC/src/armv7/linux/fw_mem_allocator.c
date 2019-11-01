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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#include <ti/cmem.h>
#include "fw_mem_allocator.h"
#include "fw_test.h"
#include "qmssPlatCfg.h"
#include "qmss_test.h"

#include "uio_module_drv.h"

/* Macro to align x to y */
#define align(x,y)   ((x + y) & (~y))

off_t    fw_mem_start_iovirt = 0;
uint8_t *fw_mem_start = (uint8_t*)0;
int      fw_mem_allocated = 0;

static uint8_t *fw_mem_end = (uint8_t*)0;
static uint8_t *fw_mem_alloc_ptr = (uint8_t*)0;
static uint32_t fw_mem_size = 0;

/* Create device list */
static fwTestMemMapDevProc_t fwDevProcMap[] = MEM_DEV_PROC; 
#define NUM_MAPS (sizeof(fwDevProcMap)/sizeof(fwTestMemMapDevProc_t))
/* One memory block definition */
typedef struct
{
    uint32_t base;
    uint32_t length;
} fwTestMemMapBlock_t;

/* List of file descriptors for each device in fwTestMemMapDevProc_t
 * together with a list of base addresses/sizes allowed for the device.
 */
static struct
{
    int                  fd;
    int                  nBlocks;
    fwTestMemMapBlock_t *blocks;
} fwTestMemMapSizeFds[NUM_MAPS];

CMEM_AllocParams  cmemParams;

/***************************************************************************************
 * FUNCTION PURPOSE: Initialize fwTestMemMapSizeFds
 ***************************************************************************************
 * DESCRIPTION: Initialize list of fd's per memory map device as well as its
 * list of base+len each can map
 ***************************************************************************************/
fw_Bool_t fw_initMemMapSizeFds (void)
{
    int    i, j;
    int    procFd;
    off_t  fileSize;
    struct stat statbuf;
    char   *scratch;

    for (i = 0; i < NUM_MAPS; i++)
    {
        /* open fd for mmap */
        if((fwTestMemMapSizeFds[i].fd = open(fwDevProcMap[i].devPath, (O_RDWR | O_SYNC))) == -1)
        {
            printf("fw_initMemMapSizeFds: Failed to open \"%s\" err=%s\n", 
                   fwDevProcMap[i].devPath, strerror(errno));
            return fw_FALSE;
        }
        /* populate base+len list from /proc */
        if (strlen(fwDevProcMap[i].procPath))
        {
            if ((procFd = open (fwDevProcMap[i].procPath, O_RDONLY)) == -1)
            {
                printf("fw_initMemMapSizeFds: Failed to open \"%s\" err=%s\n", 
                       fwDevProcMap[i].procPath, strerror(errno));
                return fw_FALSE;
            }
            if (fstat (procFd, &statbuf) == -1)
            {
                printf ("fw_initMemMapSizeFds: Failed to find size of %s (%s)\n", 
                        fwDevProcMap[i].procPath, strerror(errno));
                return fw_FALSE;
            }
            fileSize = statbuf.st_size;
            if (fileSize && (fileSize % sizeof (fwTestMemMapBlock_t)) != 0)
            {
                printf ("fw_initMemMapSizeFds: Error: size of %s(%d) zero or %% %d != 0\n", 
                        fwDevProcMap[i].procPath, (int)fileSize, sizeof(fwTestMemMapBlock_t));
                return fw_FALSE;
            }
            fwTestMemMapSizeFds[i].nBlocks          = fileSize / sizeof (fwTestMemMapBlock_t);
            fwTestMemMapSizeFds[i].blocks           = malloc (sizeof(fwTestMemMapSizeFds[i].blocks[0]) * 
                                                               fwTestMemMapSizeFds[i].nBlocks);
            scratch = malloc (sizeof(fwTestMemMapSizeFds[i].blocks[0]) * 
                              fwTestMemMapSizeFds[i].nBlocks);
            if (! fwTestMemMapSizeFds[i].blocks || ! scratch)
            {
                 printf ("fw_initMemMapSizeFds: failed to malloc size blocks for %d\n", i);
                 return fw_FALSE;
            }
            if ( read (procFd, scratch, fileSize) != fileSize)
            {
                 printf ("fw_initMemMapSizeFds: failed to read %d bytes from %s\n", 
                         (int)fileSize, fwDevProcMap[i].procPath);
                 return fw_FALSE;
            }
            close (procFd);
            for (j = 0; j < fwTestMemMapSizeFds[i].nBlocks; j++)
            {
                 /* Swizzle from big endian in proc */
                 fwTestMemMapSizeFds[i].blocks[j].base = 
                     (scratch[j*8 + 0] << 24) |
                     (scratch[j*8 + 1] << 16) |
                     (scratch[j*8 + 2] <<  8) |
                     (scratch[j*8 + 3]      );
                 fwTestMemMapSizeFds[i].blocks[j].length = 
                     (scratch[j*8 + 4] << 24) |
                     (scratch[j*8 + 5] << 16) |
                     (scratch[j*8 + 6] <<  8) |
                     (scratch[j*8 + 7]      );
            }
            free (scratch);
        }
        else
        {
            /* No proc entry defined, so assume it can map entire memory */
            fwTestMemMapSizeFds[i].nBlocks          = 1;
            fwTestMemMapSizeFds[i].blocks           = malloc (sizeof(fwTestMemMapSizeFds[i].blocks[0]) * 
                                                               fwTestMemMapSizeFds[i].nBlocks);
            if (! fwTestMemMapSizeFds[i].blocks)
            {
                printf ("fw_initMemMapSizeFds: failed to malloc size blocks for %d\n", i);
                return fw_FALSE;
            }
            fwTestMemMapSizeFds[i].blocks[0].base   = 0;
            fwTestMemMapSizeFds[i].blocks[0].length = ~(uint32_t)0; 
        }
    }

    return fw_TRUE;
}

/***************************************************************************************
 * FUNCTION PURPOSE: Initialize memory allocation
 ***************************************************************************************
 * DESCRIPTION: Initialize memory allocation
 ***************************************************************************************/
fw_Bool_t fw_memAllocInit
(
    off_t      *pAddr,  /* Physical address */
    uint32_t    size    /* Size of block */
)
{
    void *map_base; 
    int blockId;

    if (! pAddr)
    {
        return fw_FALSE;
    }

    /* initialize the CMEM module */
    if (CMEM_init() == -1) {
        printf("Failed to initialize CMEM\n");
        return fw_FALSE;
    }

    if ( fw_initMemMapSizeFds () == fw_FALSE)
    {
        return fw_FALSE;
    }

    /* Set up whether or not allocating, becuase CMEM_unregister needs it too */
    cmemParams.type      = CMEM_HEAP;
    cmemParams.flags     = CMEM_CACHED;
    cmemParams.alignment = 14;
    blockId              = CMEM_CMABLOCKID;

    if (! *pAddr)
    {
        /* Allocate from cmem */
        if ( ! (map_base = CMEM_alloc2 (blockId, size, &cmemParams)))
        {
            printf("fw_memAllocInit: Failed to allocated %d from cmem\n", size);
            return fw_FALSE;
        }

        if ( ! (*pAddr = CMEM_getPhys (map_base)))
        {
            printf("fw_memAllocInit: CMEM_getPhys failed for 0x%x.\n", (uint32_t)map_base);
            return fw_FALSE;
        }
        fw_mem_allocated = 1;
    }
    else
    {
        /* Get virtual address from CMEM */
        if ( ! (map_base = CMEM_registerAlloc (*pAddr)))
        {
            printf("fw_memAllocInit: Failed to map from cmem 0x%llx\n", *pAddr);
            return fw_FALSE;
        }
    }

    printf ("core %d: got CMEM CMA phys 0x%llx; virt 0x%08x\n",
            coreNum, *pAddr, (uint32_t)map_base);

#ifdef EXT_DEBUG
    printf("fw_memAllocInit: Memory mapped at address %p.\n", map_base); 
#endif

    fw_mem_alloc_ptr = fw_mem_start = map_base;
    fw_mem_size = size;
    fw_mem_end = fw_mem_start + fw_mem_size;
    fw_mem_start_iovirt = FW_PLATFORM_PHYS2IOVIRT(*pAddr);
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
    uint32_t    alignment
)
{
    uint32_t key;
    uint8_t *alloc_ptr;
    void *p_block =NULL;

    Osal_fwCsEnter(&key);
    alloc_ptr = (uint8_t*)align((uint32_t)fw_mem_alloc_ptr, alignment);
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
    uint32_t        page_sz, base_correction, offset, pg_offset, mmap_length;
    long            retval;
    int             fd = -1, i, j;

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

    /* Find the best (first match) fd to map from */
    for (i = 0; i < NUM_MAPS; i++)
    {
        for (j = 0; j < fwTestMemMapSizeFds[i].nBlocks; j++)
        {
            if (((((uint32_t)addr)       ) >= fwTestMemMapSizeFds[i].blocks[j].base) &&
                ((((uint32_t)addr) + size) <= (fwTestMemMapSizeFds[i].blocks[j].base +
                                               fwTestMemMapSizeFds[i].blocks[j].length)))
            {
                printf ("core %d: Mapping %d bytes from %s at 0x%08x\n",
                        coreNum, size, fwDevProcMap[i].devPath, (uint32_t)addr);
                fd = fwTestMemMapSizeFds[i].fd;
                base_correction = fwTestMemMapSizeFds[i].blocks[j].base & (page_sz - 1);
                offset = (uint32_t)addr - fwTestMemMapSizeFds[i].blocks[j].base;

                break;
            }
        }
        if (fd != -1)
        {
            break;
        }
    }

    if (fd == -1)
    {
        printf ("fw_memMap: Failed to find fd to map 0x%08x.\n", (uint32_t)addr);
        return (void *)0;
    }
    pg_offset = (offset + base_correction)
        & (~((page_sz<< UIO_MODULE_DRV_MAP_OFFSET_SHIFT) - 1));

    mmap_length = size + offset - pg_offset + base_correction;

    map_base = mmap(0, mmap_length, (PROT_READ|PROT_WRITE), MAP_SHARED,
        fd, (off_t)(j * page_sz) + (pg_offset));
    if(map_base == (void *) -1)
    {
        printf("fw_memMap: Failed to mmap \"dev/mem\" err=%s\n",
               strerror(errno));
        return (void *)0;
    }
    virt_addr = map_base + base_correction + offset - pg_offset;
    return(virt_addr);
}


/***************************************************************************************
 * FUNCTION PURPOSE: Deinitialize memory allocation
 ***************************************************************************************
 * DESCRIPTION: Deinitialize memory allocation.  Frees CMA/CMEM allocation if this
 * task was allocator.
 ***************************************************************************************/
void fw_memAllocExit
(
)
{
    int retVal;
    if (fw_mem_allocated)
    {
        if ((retVal = CMEM_free (fw_mem_start, NULL)) < 0)
        {
            printf("fw_memAllocExit: failed to free 0x%08x: %d\n", (uint32_t)fw_mem_start, retVal);
            return;
        }
    } 
    else if ((retVal = CMEM_unregister (fw_mem_start, &cmemParams)) < 0) 
    {
        {
            printf("fw_memAllocExit: failed to unregister 0x%08x: %d\n", (uint32_t)fw_mem_start, retVal);
            return;
        }
    }
    else
    {
    }
    fw_mem_start = NULL;
    fw_mem_start_iovirt = 0;
    fw_mem_end = NULL;
    fw_mem_alloc_ptr = NULL;
    fw_mem_size = 0;
}

