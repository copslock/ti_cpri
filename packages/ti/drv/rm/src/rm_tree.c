/**
 *   @file  rm_tree.c
 *
 *   @brief   
 *      Resource Manager Tree Manipulation.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2016, Texas Instruments, Inc.
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

/* RM external API includes */
#include <ti/drv/rm/rm.h>

/* RM internal API includes */
#include <ti/drv/rm/include/rm_internal.h>
#include <ti/drv/rm/include/rm_treeloc.h>

/* RM OSAL layer */
#include <rm_osal.h>

/**********************************************************************
 ********************* NameServer Tree Functions **********************
 **********************************************************************/

/* FUNCTION PURPOSE: Invalidates an entire NameServer tree
 ***********************************************************************
 * DESCRIPTION: Uses the cache handling versions of the RB tree
 *              macros to walk an entire NameServer tree and invalidate
 *              it.
 */
void rmNameServerTreeInv(Rm_NameServerTree *treeRoot)
{
    Rm_NameServerNode *node;

    /* Invalidate the tree root */
    Rm_osalBeginMemAccess((void *)treeRoot, sizeof(*treeRoot));
    /* Walk the tree which will invalidate each element in the tree */
    node = RB_MIN_CACHED(_Rm_NameServerTree, treeRoot);
    while (node) {
        node = RB_NEXT_CACHED(_Rm_NameServerTree, treeRoot, node);
    }
}

/* FUNCTION PURPOSE: Writebacks an entire NameServer tree
 ***********************************************************************
 * DESCRIPTION: Walks the entire NameServer tree writing back
 *              each element to shared memory
 */
void rmNameServerTreeWb(Rm_NameServerTree *treeRoot)
{
    Rm_NameServerNode *node;

    /* Writeback each element in the tree */
    node = RB_MIN(_Rm_NameServerTree, treeRoot);
    if (node) {
        do {
            Rm_osalEndMemAccess((void *)node, sizeof(*node));
            node = RB_NEXT(_Rm_NameServerTree, treeRoot, node);
        } while (node);
    }

    /* Writeback the tree root */
    Rm_osalEndMemAccess((void *)treeRoot, sizeof(*treeRoot));
}

/* FUNCTION PURPOSE: Creates a new NameServer tree node
 ***********************************************************************
 * DESCRIPTION: Creates a new NameServer tree node with the
 *              specified name and the resource values it is
 *              tied to.
 */
Rm_NameServerNode *rmNameServerNodeNew(Rm_NameServerNodeCfg *nodeCfg)
{
    Rm_NameServerNode *newNode = NULL;

    newNode = Rm_osalMalloc(sizeof(Rm_NameServerNode));
    if (newNode) {
        rm_strncpy(newNode->objName, nodeCfg->objName, RM_NAME_MAX_CHARS);
        rm_strncpy(newNode->resourceName, nodeCfg->resourceName,
                   RM_NAME_MAX_CHARS);
        newNode->resourceBase   = nodeCfg->resourceBase;
        newNode->resourceLength = nodeCfg->resourceLength;
    }

    return(newNode);
}

/* FUNCTION PURPOSE: Deletes a NameServer tree node
 ***********************************************************************
 * DESCRIPTION: Deletes the specified NameServer tree node.
 */
void rmNameServerNodeFree(Rm_NameServerNode *node)
{
    if (node) {
        Rm_osalFree((void *)node, sizeof(*node));
    }
}

/* FUNCTION PURPOSE: Compares two NameServer tree nodes
 ***********************************************************************
 * DESCRIPTION: Returns the result of a comparison of two 
 *              NameServer tree node names.
 *              node1 name < node2 name --> return < 0
 *              node1 name = node2 name --> return 0
 *              node1 name > node2 name --> return > 0
 */
int rmNameServerNodeCompare(Rm_NameServerNode *node1, Rm_NameServerNode *node2)
{
    return(strncmp(node1->objName, node2->objName, RM_NAME_MAX_CHARS));
}

/* FUNCTION PURPOSE: Invalidates a NameServer tree node
 ***********************************************************************
 * DESCRIPTION: Uses RM OSAL layer to invalidate the specified
 *              NameServer tree node.
 */
void rmNameServerNodeInv(Rm_NameServerNode *node)
{
    Rm_osalBeginMemAccess((void *)node, sizeof(*node));
}

/* Generate the NameServer tree manipulation functions */
RB_GENERATE(_Rm_NameServerTree, _Rm_NameServerNode, linkage,
            rmNameServerNodeCompare, rmNameServerNodeInv);

/**********************************************************************
 *************** Policy Valid Instance Tree Functions *****************
 **********************************************************************/

/* FUNCTION PURPOSE: Invalidates an entire valid instance tree
 ***********************************************************************
 * DESCRIPTION: Uses the cache handling versions of the RB tree
 *              macros to walk an entire valid instance tree and invalidate
 *              it.
 */
void rmPolicyValidInstTreeInv(Rm_PolicyValidInstTree *treeRoot)
{
    Rm_PolicyValidInstNode *node;

    /* Invalidate the tree root */
    Rm_osalBeginMemAccess((void *)treeRoot, sizeof(*treeRoot));
    /* Walk the tree which will invalidate each element in the tree */
    node = RB_MIN_CACHED(_Rm_PolicyValidInstTree, treeRoot);
    while (node) {
        node = RB_NEXT_CACHED(_Rm_PolicyValidInstTree, treeRoot, node);
    }
}

/* FUNCTION PURPOSE: Writebacks an entire valid instance tree
 ***********************************************************************
 * DESCRIPTION: Walks the entire valid instance tree writing back
 *              each element to shared memory
 */
void rmPolicyValidInstTreeWb(Rm_PolicyValidInstTree *treeRoot)
{
    Rm_PolicyValidInstNode *node;

    /* Writeback each element in the tree */
    node = RB_MIN(_Rm_PolicyValidInstTree, treeRoot);
    if (node) {
        do {
            Rm_osalEndMemAccess((void *)node, sizeof(*node));
            node = RB_NEXT(_Rm_PolicyValidInstTree, treeRoot, node);
        } while (node);
    }

    /* Writeback the tree root */
    Rm_osalEndMemAccess((void *)treeRoot, sizeof(*treeRoot));
}

/* FUNCTION PURPOSE: Creates a new valid instance tree node
 ***********************************************************************
 * DESCRIPTION: Creates a new valid instance tree node with the
 *              specified name.
 */
Rm_PolicyValidInstNode *rmPolicyValidInstNodeNew(const char *instName,
                                                 const int32_t instIdx)
{
    Rm_PolicyValidInstNode *newNode = NULL;

    newNode = Rm_osalMalloc(sizeof(Rm_PolicyValidInstNode));
    if (newNode) {
        rm_strncpy(newNode->name, instName, RM_NAME_MAX_CHARS);
        newNode->allocRefCount = 0;
        newNode->deletePending = RM_FALSE;
        newNode->instIdx       = instIdx;
    }
    return(newNode);
}

/* FUNCTION PURPOSE: Deletes a valid instance tree node
 ***********************************************************************
 * DESCRIPTION: Deletes the specified valind instance tree node
 *              if it has zero allocation references.
 */
void rmPolicyValidInstNodeFree(Rm_PolicyValidInstNode *node)
{
 /* TODO: Add allocation reference count check when reference count
  *       addition/subtraction bugs are resolved */
 //   if (node->allocRefCount == 0) {
        Rm_osalFree((void *)node, sizeof(*node));
 //   }
}

/* FUNCTION PURPOSE: Compares two valid instance tree nodes
 ***********************************************************************
 * DESCRIPTION: Returns the result of a comparison of two 
 *              valid instance tree node names.
 *              node1 name < node2 name --> return < 0
 *              node1 name = node2 name --> return 0
 *              node1 name > node2 name --> return > 0
 */
int rmPolicyValidInstNodeCompare(Rm_PolicyValidInstNode *node1,
                                 Rm_PolicyValidInstNode *node2)
{
    return(strncmp(node1->name, node2->name, RM_NAME_MAX_CHARS));
}

/* FUNCTION PURPOSE: Invalidates a valid instance tree node
 ***********************************************************************
 * DESCRIPTION: Uses RM OSAL layer to invalidate the specified
 *              valid instance tree node.
 */
void rmPolicyValidInstNodeInv(Rm_PolicyValidInstNode *node)
{
    Rm_osalBeginMemAccess((void *)node, sizeof(*node));
}

/* Generate the valid instance tree manipulation functions */
RB_GENERATE(_Rm_PolicyValidInstTree, _Rm_PolicyValidInstNode, linkage,
            rmPolicyValidInstNodeCompare, rmPolicyValidInstNodeInv);

/**********************************************************************
 ***************** Allocator Resource Tree Functions ******************
 **********************************************************************/

/* FUNCTION PURPOSE: Invalidates an entire resource tree
 ***********************************************************************
 * DESCRIPTION: Uses the cache handling versions of the RB tree
 *              macros to walk an entire resource tree and invalidate
 *              it.
 */
void rmResourceTreeInv(Rm_ResourceTree *treeRoot)
{
    Rm_ResourceNode *node;

    /* Invalidate the tree root */
    Rm_osalBeginMemAccess((void *)treeRoot, sizeof(*treeRoot));
    /* Walk the tree which will invalidate each element in the tree */
    node = RB_MIN_CACHED(_Rm_AllocatorResourceTree, treeRoot);
    while (node) {
        node = RB_NEXT_CACHED(_Rm_AllocatorResourceTree, treeRoot, node);
    }
}

/* FUNCTION PURPOSE: Writebacks an entire resource tree
 ***********************************************************************
 * DESCRIPTION: Walks the entire resource tree writing back
 *              each element to shared memory
 */
void rmResourceTreeWb(Rm_ResourceTree *treeRoot)
{
    Rm_ResourceNode *node;

    /* Writeback each element in the tree */
    node = RB_MIN(_Rm_AllocatorResourceTree, treeRoot);
    if (node) {
        do {
            Rm_osalEndMemAccess((void *)node, sizeof(*node));
            node = RB_NEXT(_Rm_AllocatorResourceTree, treeRoot, node);
        } while (node);
    }

    /* Writeback the tree root */
    Rm_osalEndMemAccess((void *)treeRoot, sizeof(*treeRoot));
}

/* FUNCTION PURPOSE: Creates a new resource tree node
 ***********************************************************************
 * DESCRIPTION: Creates a new resource tree node with the
 *              specified resource values.
 */
Rm_ResourceNode *rmResourceNodeNew(uint32_t resourceBase,
                                   uint32_t resourceLength)
{
    Rm_ResourceNode *newNode = NULL;

    newNode = Rm_osalMalloc(sizeof(*newNode));
    if (newNode) {
        newNode->base            = resourceBase;
        newNode->length          = resourceLength;
        newNode->allocationCount = 0;
        newNode->ownerList       = NULL;
    }
    return(newNode);
}

/* FUNCTION PURPOSE: Deletes a resource tree node
 ***********************************************************************
 * DESCRIPTION: Deletes the specified resource tree node
 *              if its allocation count is zero.
 */
void rmResourceNodeFree(Rm_ResourceNode *node)
{
    if (node->allocationCount == 0) {
        Rm_osalFree((void *)node, sizeof(*node));
    }
}

/* FUNCTION PURPOSE: Compares two resource tree nodes
 ***********************************************************************
 * DESCRIPTION: Returns the result of a comparison of two 
 *              resource tree node value ranges.
 *
 *              |node1 range||node2 range| --> return < 0
 *
 *                 |node1 range|
 *                   |node2 range|         --> return 0 (any overlap in ranges)
 *
 *              |node2 range||node1 range| --> return > 0
 */
int rmResourceNodeCompare(Rm_ResourceNode *node1, Rm_ResourceNode *node2)
{
    uint32_t node1End = node1->base + node1->length - 1;
    uint32_t node2End = node2->base + node2->length - 1;

    if (node1End < node2->base) {
        /* End of node1 range is less than the start of node2's range.  Return
         * a negative value */
        return(-1);
    } else if (node1->base > node2End) {
        /* Start of node1 range is after end of node2's range.  Return a
         * positive value */
        return(1);
    } else {
        /* If neither of the latter conditions were satisfied there is some
         * overlap between node1 and node2.  Return 0 since the application
         * must handle this overlap. */
        return(0);
    }
}

/* FUNCTION PURPOSE: Invalidates a resource tree node
 ***********************************************************************
 * DESCRIPTION: Uses RM OSAL layer to invalidate the specified
 *              resource tree node.
 */
void rmResourceNodeInv(Rm_ResourceNode *node)
{
    Rm_osalBeginMemAccess((void *)node, sizeof(*node));
}

/* Generate the resource tree manipulation functions */
RB_GENERATE(_Rm_AllocatorResourceTree, _Rm_ResourceNode, linkage,
            rmResourceNodeCompare, rmResourceNodeInv);

/**********************************************************************
 ****************** Allocator Policy Tree Functions *******************
 **********************************************************************/

/* FUNCTION PURPOSE: Invalidates an entire policy tree
 ***********************************************************************
 * DESCRIPTION: Uses the cache handling versions of the RB tree
 *              macros to walk an entire policy tree and invalidate
 *              it.
 */
void rmPolicyTreeInv(Rm_PolicyTree *treeRoot)
{
    Rm_PolicyNode *node;

    /* Invalidate the tree root */
    Rm_osalBeginMemAccess((void *)treeRoot, sizeof(*treeRoot));
    /* Walk the tree which will invalidate each element in the tree */
    node = RB_MIN_CACHED(_Rm_AllocatorPolicyTree, treeRoot);
    while (node) {
        node = RB_NEXT_CACHED(_Rm_AllocatorPolicyTree, treeRoot, node);
    }
}

/* FUNCTION PURPOSE: Writebacks an entire policy tree
 ***********************************************************************
 * DESCRIPTION: Walks the entire policy tree writing back
 *              each element to shared memory
 */
void rmPolicyTreeWb(Rm_PolicyTree *treeRoot)
{
    Rm_PolicyNode *node;

    /* Writeback each element in the tree */
    node = RB_MIN(_Rm_AllocatorPolicyTree, treeRoot);
    if (node) {
        do {
            Rm_osalEndMemAccess((void *)node, sizeof(*node));
            Rm_osalEndMemAccess((void *)node->perms, node->permsLen);
            node = RB_NEXT(_Rm_AllocatorPolicyTree, treeRoot, node);
        } while (node);
    }

    /* Writeback the tree root */
    Rm_osalEndMemAccess((void *)treeRoot, sizeof(*treeRoot));
}

/* FUNCTION PURPOSE: Creates a new policy tree node
 ***********************************************************************
 * DESCRIPTION: Creates a new policy tree node with the
 *              specified policy values.
 */
Rm_PolicyNode *rmPolicyNodeNew(uint32_t resourceBase, uint32_t resourceLength)
{
    Rm_PolicyNode *newNode = NULL;

    newNode = Rm_osalMalloc(sizeof(*newNode));
    if (newNode) {
        newNode->base  = resourceBase;
        newNode->len   = resourceLength;
        newNode->perms = NULL;
        newNode->permsLen = 0;
        newNode->allocAlign = 1;
        newNode->cdAllocSize = 0;
    }
    return(newNode);
}

/* FUNCTION PURPOSE: Deletes a policy tree node
 ***********************************************************************
 * DESCRIPTION: Deletes the specified policy tree node.
 */
void rmPolicyNodeFree(Rm_PolicyNode *node)
{
    if (node->perms) {
        Rm_osalFree((void *)node->perms, node->permsLen);
    }
    Rm_osalFree((void *)node, sizeof(*node));
}

/* FUNCTION PURPOSE: Compares two policy tree nodes
 ***********************************************************************
 * DESCRIPTION: Returns the result of a comparison of two
 *              policy tree node value ranges.
 *
 *              |node1 range||node2 range| --> return < 0
 *
 *                 |node1 range|
 *                   |node2 range|         --> return 0 (any overlap in ranges)
 *
 *              |node2 range||node1 range| --> return > 0
 */
int rmPolicyNodeCompare(Rm_PolicyNode *node1, Rm_PolicyNode *node2)
{
    uint32_t node1End = node1->base + node1->len - 1;
    uint32_t node2End = node2->base + node2->len - 1;

    if (node1End < node2->base) {
        /* End of node1 range is less than the start of node2's range.  Return
         * a negative value */
        return(-1);
    } else if (node1->base > node2End) {
        /* Start of node1 range is after end of node2's range.  Return a
         * positive value */
        return(1);
    } else {
        /* If neither of the latter conditions were satisfied there is some
         * overlap between node1 and node2.  Return 0 since the application
         * must handle this overlap. */
        return(0);
    }
}

/* FUNCTION PURPOSE: Invalidates a policy tree node
 ***********************************************************************
 * DESCRIPTION: Uses RM OSAL layer to invalidate the specified
 *              policy tree node.
 */
void rmPolicyNodeInv(Rm_PolicyNode *node)
{
    Rm_osalBeginMemAccess((void *)node, sizeof(*node));
    Rm_osalBeginMemAccess((void *)node->perms, node->permsLen);
}

/* Generate the policy tree manipulation functions */
RB_GENERATE(_Rm_AllocatorPolicyTree, _Rm_PolicyNode, linkage,
            rmPolicyNodeCompare, rmPolicyNodeInv);

/**********************************************************************
 ********************* Allocator Tree Functions ***********************
 **********************************************************************/

/* FUNCTION PURPOSE: Invalidates an entire allocator tree
 ***********************************************************************
 * DESCRIPTION: Uses the cache handling versions of the RB tree
 *              macros to walk and invalidate an entire allocator tree.
 */
void rmAllocatorTreeInv(Rm_AllocatorTree *treeRoot)
{
    Rm_AllocatorNode *node;

    /* Invalidate the tree root */
    Rm_osalBeginMemAccess((void *)treeRoot, sizeof(*treeRoot));
    /* Walk the tree which will invalidate each element in the tree */
    node = RB_MIN_CACHED(_Rm_AllocatorTree, treeRoot);
    while (node) {
        rmResourceTreeInv(node->resourceRoot);
        rmPolicyTreeInv(node->policyRoot);
        node = RB_NEXT_CACHED(_Rm_AllocatorTree, treeRoot, node);
    }
}

/* FUNCTION PURPOSE: Writebacks an entire allocator tree
 ***********************************************************************
 * DESCRIPTION: Walks the entire allocator tree writing back
 *              each element to shared memory
 */
void rmAllocatorTreeWb(Rm_AllocatorTree *treeRoot)
{
    Rm_AllocatorNode *node;

    /* Writeback each element in the tree */
    node = RB_MIN(_Rm_AllocatorTree, treeRoot);
    if (node) {
        do {
            rmResourceTreeWb(node->resourceRoot);
            rmPolicyTreeWb(node->policyRoot);

            Rm_osalEndMemAccess((void *)node, sizeof(*node));
            node = RB_NEXT(_Rm_AllocatorTree, treeRoot, node);
        } while (node);
    }

    /* Writeback the tree root */
    Rm_osalEndMemAccess((void *)treeRoot, sizeof(*treeRoot));
}

/* FUNCTION PURPOSE: Creates a new allocator node
 ***********************************************************************
 * DESCRIPTION: Creates a new allocator node with the
 *              specified resource name.
 */
Rm_AllocatorNode *rmAllocatorNodeNew(const char *resourceName)
{
    Rm_AllocatorNode *newNode = NULL;

    newNode = Rm_osalMalloc(sizeof(*newNode));
    if (newNode) {
        rm_strncpy(newNode->resourceName, resourceName, RM_NAME_MAX_CHARS);
        newNode->resourceRoot = NULL;
        newNode->policyRoot   = NULL;
    }
    return(newNode);
}

/* FUNCTION PURPOSE: Deletes an allocator node
 ***********************************************************************
 * DESCRIPTION: Deletes the specified allocator node.
 */
void rmAllocatorNodeFree(Rm_AllocatorNode *node)
{
    Rm_osalFree((void *)node, sizeof(*node));
}

/* FUNCTION PURPOSE: Compares two allocator nodes
 ***********************************************************************
 * DESCRIPTION: Returns the result of a comparison of two
 *              allocator node resource names.
 *              node1 name < node2 name --> return < 0
 *              node1 name = node2 name --> return 0
 *              node1 name > node2 name --> return > 0
 */
int rmAllocatorNodeCompare(Rm_AllocatorNode *node1, Rm_AllocatorNode *node2)
{
    return(strncmp(node1->resourceName, node2->resourceName,
                   RM_NAME_MAX_CHARS));
}

/* FUNCTION PURPOSE: Invalidates an allocator tree node
 ***********************************************************************
 * DESCRIPTION: Uses RM OSAL layer to invalidate the specified
 *              allocator tree node.
 */
void rmAllocatorNodeInv(Rm_AllocatorNode *node)
{
    Rm_osalBeginMemAccess((void *)node, sizeof(Rm_AllocatorNode));
}

/* Generate the allocator tree manipulation functions */
RB_GENERATE(_Rm_AllocatorTree, _Rm_AllocatorNode, linkage,
            rmAllocatorNodeCompare, rmAllocatorNodeInv);
