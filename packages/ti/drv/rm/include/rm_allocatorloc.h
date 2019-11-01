/*
 *  file  rm_allocatorloc.h
 *
 *  Private data structures and APIS for Resource Manager allocators.
 *
 *  ============================================================================
 *      (C) Copyright 2012-2015, Texas Instruments, Inc.
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

#ifndef RM_ALLOCATORLOC_H_
#define RM_ALLOCATORLOC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* RM external includes */
#include <ti/drv/rm/rm.h>

/* RM internal includes */
#include <ti/drv/rm/include/rm_internal.h>
#include <ti/drv/rm/include/rm_dtb_utilloc.h>
#include <ti/drv/rm/include/rm_treeloc.h>

/* Resource properties extracted from the GRL DTB */
typedef struct {
    /* Pointer to a resource's range data within the GRL DTB */
    const void *rangeData;
    /* Length in bytes of a resource's range data in the GRL DTB */
    int32_t     rangeLen;
    /* Pointer to a resource's NameServer assignment data within the GRL DTB.
     * Will be NULL if not defined for the resource node. */
    const void *nsAssignData;
    /* Length in bytes of a resource's NameServer assignment data in the
     * GRL DTB.  Will be zero if not defined for the resource node. */
    int32_t     nsAssignLen;
    /* Pointer to a resource's Linux alias data within the GRL DTB.  Will be
     * NULL if not defined for the resource node. */
    const void *linuxAliasData;
    /* Length in bytes of a resource's Linux alias data data in the GRL DTB.
     * Will be zero if not defined for the resource node. */
    int32_t     linuxAliasLen;
} Rm_ResourceProperties;

/* Resource allocator operations */
typedef enum {
    /* Allocate init operation */
    Rm_allocatorOp_ALLOCATE_INIT = 0,
    /* Allocate use operation */
    Rm_allocatorOp_ALLOCATE_USE,
    /* Get resource status operation */
    Rm_allocatorOp_GET_STATUS,
    /* Free operation */
    Rm_allocatorOp_FREE,
    /* Preallocate to use based on Policy DTB information operation */
    Rm_allocatorOp_PRE_ALLOCATE_INIT,
    /* Preallocate to init based on Policy DTB information operation */
    Rm_allocatorOp_PRE_ALLOCATE_USE
} Rm_AllocatorOp;

/* Allocator operation configuration structure */
typedef struct {
    /* RM instance for which the allocator operation is taking place */
    Rm_PolicyValidInstNode *serviceInstNode;
    /* Allocator operation type */
    Rm_AllocatorOp          operation;
    /* Resources for which the allocator operation will affect */
    Rm_ResourceInfo        *resourceInfo;
} Rm_AllocatorOpInfo;

Rm_AllocatorNode *rmAllocatorFind(Rm_Handle rmHandle, const char *resourceName);
int rmAllocatorGetNodeLocalization(Rm_Handle rmHandle, char *resourceName,
                                   int32_t *resBase, uint32_t *resLen);
int32_t rmAllocatorOperation(Rm_Handle rmHandle, Rm_AllocatorOpInfo *opInfo);
int32_t rmAllocatorTreeInit(Rm_Handle rmHandle, void *grlDtb,
                            void *policyDtb, void *linuxDtb);
int32_t rmAllocatorAddResNode(Rm_Handle rmHandle, Rm_AllocatorNode *allocator,
                              int32_t resBase, uint32_t resLen);
void rmAllocatorDeleteResNode(Rm_Handle rmHandle, Rm_AllocatorNode *allocator,
                              int32_t resBase, uint32_t resLen);
void rmAllocatorTreeDelete(Rm_Handle rmHandle);

#ifdef __cplusplus
}
#endif

#endif /* RM_ALLOCATORLOC_H_ */

