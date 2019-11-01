
/******************************************************************************
 * FILE PURPOSE:  Memory allocator for running Example
 ******************************************************************************
 * FILE NAME:   fw_mem_allocator.h
 *
 * DESCRIPTION: Memory allocator for running Example
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

#ifndef __FW_MEM_ALLOCATOR_H__
#define __FW_MEM_ALLOCATOR_H__

/* C Standard library Include */
#include <stdio.h>
/* System level header files */
#include <stdint.h>
#include <stdlib.h>
#include "fw_test.h"

/* Physical address of the memory pool */
extern uint8_t *fw_mem_start_phy;
/* virtual address of the memory pool */
extern uint8_t *fw_mem_start;

/* Function to initialize memory allocator */
fw_Bool_t fw_memAllocInit
(
    uint8_t     *addr, /* Physical address */
    uint32_t    size   /* Size of block */
);

/* Function to allocate memory */
void* fw_memAlloc
(
    uint32_t    size, /* Size of block needed */
    uint32_t    align /* Alignment of the block needed */
);

/* Function to map the give physical address to virtual memory space */
void *fw_memMap
(
    void        *addr, /* Physical address */
    uint32_t    size   /* Size of block */
);

#endif

