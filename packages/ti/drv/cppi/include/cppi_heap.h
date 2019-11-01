/**
 *   @file  cppi_heap.h
 *
 *   @brief   
 *      Header file for the linked list library
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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
#ifndef __CPPI_HEAP_H__
#define __CPPI_HEAP_H__

#include <ti/drv/cppi/include/cppi_listlib.h>

/* Needs to be large enough to hold either a Cppi_ChObj or a Cppi_FlowObj.
 * This MUST be aligned such that it doesn't SPAN a cache line.  It is
 * OK to have more than one chunk per cache line.  There are no constraints
 * if Cppi_osalBeginMemAccess is blank. */
#define CPPI_BLOCK_CHUNK_SIZE 32

/* Check assumption that Cppi_ChObj and Cppi_FlowObj fit in CPPI_BLOCK_CHUNK_SIZE */
#define CPPI_BLOCK_CHUNK_SIZE_CHECK \
        CPPI_COMPILE_TIME_SIZE_CHECK(CPPI_BLOCK_CHUNK_SIZE >= sizeof(Cppi_ChObj)); \
        CPPI_COMPILE_TIME_SIZE_CHECK(CPPI_BLOCK_CHUNK_SIZE >= sizeof(Cppi_FlowObj))

/**************************************************************************
 * STRUCTURE -  Cppi_BlockTracker
 **************************************************************************
 * Keeps track of system allocated memory such that it can be later
 * freed.
 **************************************************************************/
typedef struct
{
    Cppi_ListNode  links; /* double linked list header */
    uint32_t       numBlocksRem; /* Number of blocks remaining in this record */
/* Calculate the number of void *'s that fit in the remaining space of a CPPI_BLOCK_CHUNK_SIZE */
#define CPPI_BLOCK_TRACKER_MAX_BLOCKS ((CPPI_BLOCK_CHUNK_SIZE - \
                                       sizeof(Cppi_ListNode) - sizeof(uint32_t))/sizeof(void *))
    void          *blocks[CPPI_BLOCK_TRACKER_MAX_BLOCKS];
} Cppi_BlockTracker;

/**************************************************************************
 * STRUCTURE -  Cppi_HeapDesc
 **************************************************************************
 * The structure defines the configurable parameters and lists used
 * by the internal heap
 **************************************************************************/
typedef struct
{
    /* Free list for private heap */
    Cppi_ListNode          *blockFreeList;
    /* List of sections allocated from Cppi_osalMalloc() for later free */
    Cppi_BlockTracker      *allocatedBlockList;
    /* Size to request from Cppi_osalMalloc when asking for more memory */
    uint32_t                blockSize;
    /* Alignment for results from Cppi_osalMalloc.  This will be
     * thrown away from begin and end of malloc results */
    uint32_t                alignPow2;
} Cppi_HeapDesc;

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/
void cppi_internal_heap_cache_begin (Cppi_HeapDesc *desc);
void *cppi_internal_heap_malloc (Cppi_HeapDesc *desc, uint32_t size);
void cppi_internal_heap_free (Cppi_HeapDesc *desc, void *ptr);
void cppi_internal_heap_release (Cppi_HeapDesc *desc);
void cppi_internal_heap_add (Cppi_HeapDesc *desc, void *heap, uint32_t heapSize);

#endif /* __CPPI_HEAP_H__ */

/* Nothing past this point */

