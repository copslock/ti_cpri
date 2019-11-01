/**
 *   @file  rm_allocator.c
 *
 *   @brief   
 *      This is the Resource Manager allocator source.
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

/* RM external includes */
#include <ti/drv/rm/rm.h>

/* RM internal includes */
#include <ti/drv/rm/include/rm_internal.h>
#include <ti/drv/rm/include/rm_loc.h>
#include <ti/drv/rm/include/rm_allocatorloc.h>
#include <ti/drv/rm/include/rm_dtb_utilloc.h>
#include <ti/drv/rm/include/rm_policyloc.h>
#include <ti/drv/rm/include/rm_treeloc.h>

/* RM LIBFDT includes */
#include <ti/drv/rm/util/libfdt/libfdt.h>

/* Tree algorithm includes */
#include <ti/drv/rm/util/tree.h>

/* RM OSAL layer */
#include <rm_osal.h>

/**********************************************************************
 ************************ Local Functions *****************************
 **********************************************************************/

/* FUNCTION PURPOSE: Checks a resource node's ownership
 ***********************************************************************
 * DESCRIPTION: Returns the owner reference count if the provided
 *              instance node is in the list of resource node owners.
 *              Otherwise, returns 0.
 */
static int allocatorResNodeIsOwnedBy(Rm_Handle rmHandle, Rm_ResourceNode *node,
                                     Rm_PolicyValidInstNode *serviceInstNode)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;
    Rm_Owner *owner = node->ownerList;

    while (owner) {
        RM_SS_OBJ_INV(rmInst, owner, Rm_Owner);
        if (owner->instNameNode == serviceInstNode) {
            return(owner->refCnt);
        }
        owner = owner->nextOwner;
    }
    return(0);
}

/* FUNCTION PURPOSE: Increments an owner's refCnt
 ***********************************************************************
 * DESCRIPTION: Increments a resource owner's reference count
 */
static void allocatorResNodeOwnerRefCntInc(Rm_Handle rmHandle,
                                           Rm_ResourceNode *node,
                                           Rm_PolicyValidInstNode *serviceInstNode)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;
    Rm_Owner *owner = node->ownerList;

    while (owner) {
        RM_SS_OBJ_INV(rmInst, owner, Rm_Owner);
        if (owner->instNameNode == serviceInstNode) {
            owner->refCnt++;
            RM_SS_OBJ_WB(rmInst, owner, Rm_Owner);
            break;
        }
        owner = owner->nextOwner;
    }
}

/* FUNCTION PURPOSE: Decrements an owner's refCnt
 ***********************************************************************
 * DESCRIPTION: Decrements a resource owner's reference count
 */
static void allocatorResNodeOwnerRefCntDec(Rm_Handle rmHandle,
                                           Rm_ResourceNode *node,
                                           Rm_PolicyValidInstNode *serviceInstNode)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;
    Rm_Owner *owner = node->ownerList;

    while (owner) {
        RM_SS_OBJ_INV(rmInst, owner, Rm_Owner);
        if (owner->instNameNode == serviceInstNode) {
            owner->refCnt--;
            RM_SS_OBJ_WB(rmInst, owner, Rm_Owner);
            break;
        }
        owner = owner->nextOwner;
    }
}

/* FUNCTION PURPOSE: Returns an owner's refCnt
 ***********************************************************************
 * DESCRIPTION: Returns a resource owner's reference count
 */
static uint16_t allocatorResNodeOwnerGetRefCnt(Rm_Handle rmHandle,
                                               Rm_ResourceNode *node,
                                               Rm_PolicyValidInstNode *serviceInstNode)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;
    Rm_Owner *owner = node->ownerList;

    while (owner) {
        RM_SS_OBJ_INV(rmInst, owner, Rm_Owner);
        if (owner->instNameNode == serviceInstNode) {
            return (owner->refCnt);
        }
        owner = owner->nextOwner;
    }

    return(0);
}

/* FUNCTION PURPOSE: Adds an owner to an allocator resource
 ***********************************************************************
 * DESCRIPTION: Adds a RM instance node to a resource node's
 *              list of owners.  If the owner is already present that
 *              owner's reference count is incremented
 */
static void allocatorResNodeOwnerAdd(Rm_Handle rmHandle, Rm_ResourceNode *node,
                                     Rm_PolicyValidInstNode *serviceInstNode)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;
    Rm_Owner *ownerList = node->ownerList;
    Rm_Owner *newOwner = NULL;

    if (allocatorResNodeIsOwnedBy(rmHandle, node, serviceInstNode)) {
        allocatorResNodeOwnerRefCntInc(rmHandle, node, serviceInstNode);
    } else {
        newOwner = Rm_osalMalloc(sizeof(*newOwner));

        if (newOwner) {
            newOwner->instNameNode = serviceInstNode;
            newOwner->refCnt = 0;
            newOwner->nextOwner = NULL;

            /* Add owner entry to end of list */
            if (ownerList) {
                RM_SS_OBJ_INV(rmInst, ownerList, Rm_Owner);
                while (ownerList->nextOwner) {
                    ownerList = ownerList->nextOwner;
                    RM_SS_OBJ_INV(rmInst, ownerList, Rm_Owner);
                }
                ownerList->nextOwner = newOwner;
                RM_SS_OBJ_WB(rmInst, ownerList, Rm_Owner);
            } else {
                node->ownerList = newOwner;
            }

            node->allocationCount++;
            newOwner->refCnt++;
            newOwner->instNameNode->allocRefCount++;
            RM_SS_OBJ_WB(rmInst, newOwner, Rm_Owner);
            RM_SS_OBJ_WB(rmInst, newOwner->instNameNode,
                         Rm_PolicyValidInstNode);
        }
    }
}

/* FUNCTION PURPOSE: Compares two resource node's boundaries
 ***********************************************************************
 * DESCRIPTION: Returns TRUE if the resource nodes are neighbors from
 *              a base+length perspective.  Otherwise, returns FALSE.
 */
static int allocatorResNodeBoundaryCompare(Rm_ResourceNode *node1,
                                           Rm_ResourceNode *node2)
{
    uint32_t node1End;
    uint32_t node2End;

    if (node1 && node2) {
        node1End = node1->base + node1->length - 1;
        node2End = node2->base + node2->length - 1;

        if (node1->base < node2->base) {
            if (node1End == (node2->base - 1)) {
                return(RM_TRUE);
            }
        } else if (node2->base < node1->base) {
            if (node2End == (node1->base - 1)) {
                return(RM_TRUE);
            }
        }
        /* else: fall through to return false, not neighbors */
    }
    return(RM_FALSE);
}

/* FUNCTION PURPOSE: Compares two resource node's owners
 ***********************************************************************
 * DESCRIPTION: Returns TRUE if the owners of two resource nodes 
 *              are equivalent.  Otherwise, returns FALSE.
 */
static int allocatorResNodeOwnerCompare(Rm_Handle rmHandle,
                                        Rm_ResourceNode *node1,
                                        Rm_ResourceNode *node2)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;    
    Rm_Owner *node1Owners = node1->ownerList;
    Rm_Owner *node2Owners = node2->ownerList;
    int       matchedInst;

    if (rmInst->instType == Rm_instType_SHARED_SERVER) {
        while(node2Owners) {
            Rm_osalBeginMemAccess((void *)node2Owners, sizeof(*node2Owners));
            node2Owners = node2Owners->nextOwner;
        }
        node2Owners = node2->ownerList;
    }

    if (node1->allocationCount == node2->allocationCount) {
        while (node1Owners) {
            RM_SS_OBJ_INV(rmInst, node1Owners, Rm_Owner);
            matchedInst = RM_FALSE;
            while (node2Owners) {
                if ((node1Owners->instNameNode == node2Owners->instNameNode) &&
                    (node1Owners->refCnt == node2Owners->refCnt)) {
                    matchedInst = RM_TRUE;
                    break;
                }
                node2Owners = node2Owners->nextOwner;
            }

            if (matchedInst) {
                node2Owners = node2->ownerList;
                node1Owners = node1Owners->nextOwner;
            } else {
                return(RM_FALSE);
            }
        }
    } else {
        return(RM_FALSE);
    }

    return(RM_TRUE);
}

/* FUNCTION PURPOSE: Deletes an owner from an allocator resource
 ***********************************************************************
 * DESCRIPTION: Removes a RM owner entry from a resource node's
 *              list of owners.  If the refCnt for the specified
 *              owner is greater than 1 only the refCnt is
 *              decremented
 */
static void allocatorResNodeOwnerDelete(Rm_Handle rmHandle,
                                        Rm_ResourceNode *node,
                                        void *serviceInstNode)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;    
    Rm_Owner *owner = node->ownerList;
    Rm_Owner *prevOwner = NULL;

    if (allocatorResNodeIsOwnedBy(rmHandle, node, serviceInstNode) > 1) {
        allocatorResNodeOwnerRefCntDec(rmHandle, node, serviceInstNode);
    } else {
        while (owner) {
            RM_SS_OBJ_INV(rmInst, owner, Rm_Owner);
            if (owner->instNameNode == serviceInstNode) {
                break;
            }
            prevOwner = owner;
            owner = owner->nextOwner;
        }

        if (owner) {
            if (prevOwner == NULL) {
                node->ownerList = owner->nextOwner;
            } else {
                prevOwner->nextOwner = owner->nextOwner;
                RM_SS_OBJ_WB(rmInst, prevOwner, Rm_Owner);
            }

            node->allocationCount--;
            owner->instNameNode->allocRefCount--;
            RM_SS_OBJ_WB(rmInst, owner->instNameNode, Rm_PolicyValidInstNode);
            Rm_osalFree((void *)owner, sizeof(*owner));
        }
    }
}

/* FUNCTION PURPOSE: Copies the owners of a resource node
 ***********************************************************************
 * DESCRIPTION: Creates a list of resource owners for the destination
 *              resource node that is equivalent to the source resource
 *              node's owners
 *
 *              dstNode must be a newly created node without any owners.
 */
static void allocatorResNodeOwnerCopy(Rm_Handle rmHandle,
                                      Rm_ResourceNode *dstNode,
                                      Rm_ResourceNode *srcNode)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;
    Rm_Owner *srcOwnerList = srcNode->ownerList;
    Rm_Owner *dstNewOwner;
    Rm_Owner *dstPrevOwner;

    if (dstNode->ownerList != NULL) {
        return;
    }
    dstNode->allocationCount = srcNode->allocationCount;

    while (srcOwnerList) {
        RM_SS_OBJ_INV(rmInst, srcOwnerList, Rm_Owner);
        dstNewOwner = Rm_osalMalloc(sizeof(*dstNewOwner));
        dstNewOwner->instNameNode = srcOwnerList->instNameNode;
        dstNewOwner->refCnt = srcOwnerList->refCnt;
        dstNewOwner->nextOwner = NULL;
        RM_SS_OBJ_WB(rmInst, dstNewOwner, Rm_Owner);

        if (dstNode->ownerList == NULL) {
            dstNode->ownerList = dstNewOwner;
        } else {
            dstPrevOwner->nextOwner = dstNewOwner;
            RM_SS_OBJ_WB(rmInst, dstPrevOwner, Rm_Owner);
        }
        dstPrevOwner = dstNewOwner;
        srcOwnerList = srcOwnerList->nextOwner;
    }
}

/* FUNCTION PURPOSE: Clears a resource node's owners
 ***********************************************************************
 * DESCRIPTION: Deletes all owners from the owners list of a 
 *              resource node.
 */
static void allocatorResNodeOwnerClear(Rm_Handle rmHandle,
                                       Rm_ResourceNode *node)
{
    Rm_Inst  *rmInst = (Rm_Inst *)rmHandle;
    Rm_Owner *owner = node->ownerList;
    Rm_Owner *nextOwner;

    while (owner) {
        RM_SS_OBJ_INV(rmInst, owner, Rm_Owner);
        nextOwner = owner->nextOwner;
        node->allocationCount--;
        owner->instNameNode->allocRefCount--;
        RM_SS_OBJ_WB(rmInst, owner->instNameNode, Rm_PolicyValidInstNode);
        Rm_osalFree((void *)owner, sizeof(*owner));
        owner = nextOwner;
    }
}

/* FUNCTION PURPOSE: Get the status for an allocator resource
 ***********************************************************************
 * DESCRIPTION: Called when a resource status request is made.  The
 *              resource's allocator is searched for the resource base
 *              and length specified in the transaction.  The 
 *              resource's owner reference count is returned if the 
 *              resource range is found.
 */
static int32_t allocatorStatus(Rm_Handle rmHandle, Rm_AllocatorNode *allocator,
                               Rm_AllocatorOpInfo *opInfo)
{
    Rm_ResourceNode  findNode;
    Rm_ResourceNode *matchingNode = NULL;
    uint32_t         matchingEnd;
    uint32_t         findEnd;
    int32_t          retVal;

    memset((void *)&findNode, 0, sizeof(findNode));
    findNode.base = opInfo->resourceInfo->base;
    findNode.length = opInfo->resourceInfo->length;
    matchingNode = RB_FIND(_Rm_AllocatorResourceTree, allocator->resourceRoot,
                           &findNode);

    if (matchingNode) {
        matchingEnd = matchingNode->base + matchingNode->length - 1;
        findEnd = findNode.base + findNode.length - 1;
        if ((findNode.base >= matchingNode->base) && (findEnd <= matchingEnd)) {
            opInfo->resourceInfo->ownerCount = matchingNode->allocationCount;
            opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                      matchingNode,
                                                      opInfo->serviceInstNode);
            retVal = RM_SERVICE_APPROVED;
        } else {
            retVal = RM_SERVICE_DENIED_PARTIAL_STATUS;
        }
    } else {
        retVal = RM_SERVICE_DENIED_RES_RANGE_DOES_NOT_EXIST;
    }

    return(retVal);
}

/* FUNCTION PURPOSE: Preallocates an allocator resource
 ***********************************************************************
 * DESCRIPTION: Called when an allocate request is made but the base 
 *              is unspecified.  The preallocation algorithm looks at 
 *              available resources as well as policy permissions to 
 *              determine a resource range that satisfies the request.
 *              If a valid range is found it will be returned for the 
 *              treeAllocate algorithm to handle.
 */
static int32_t allocatorPreAllocate(Rm_Handle rmHandle,
                                    Rm_AllocatorNode *allocator,
                                    Rm_AllocatorOpInfo *opInfo)
{   
    Rm_Inst           *rmInst = (Rm_Inst *)rmHandle;
    Rm_ResourceNode    findNode;
    Rm_ResourceNode   *matchingNode = NULL;
    Rm_ResourceNode   *nextNode;
    uint32_t           matchingEnd;
    uint32_t           findEnd;
    uint32_t           rangeIndex;
    int                resourceFound = RM_FALSE;
    Rm_PolicyCheckType policyCheckType;
    Rm_PolicyCheckCfg  policyCheckCfg;
    int                nodePassesPolicy;
    int32_t            retVal = RM_OK;

    if (opInfo->operation == Rm_allocatorOp_PRE_ALLOCATE_INIT) {
        policyCheckType = Rm_policyCheck_INIT;
    } else if (opInfo->operation == Rm_allocatorOp_PRE_ALLOCATE_USE) {
        policyCheckType = Rm_policyCheck_USE;
    } else {
        retVal = RM_ERROR_INVALID_SERVICE_TYPE;
        return(retVal);
    }

    if (rmInst->instType == Rm_instType_CLIENT_DELEGATE) {
        /* Set base to first node's base since CD will not have all resources
         * like Server */
        matchingNode = RB_MIN(_Rm_AllocatorResourceTree, allocator->resourceRoot);
        opInfo->resourceInfo->base = matchingNode->base;
    } else {
        int32_t tmpBase;

        retVal = rmPolicyGetResourceBase(allocator->policyRoot,
                                         opInfo->serviceInstNode,
                                         policyCheckType, &tmpBase);
        if (retVal == RM_OK) {
            opInfo->resourceInfo->base = tmpBase;
        } else {
            return(retVal);
        }
    }

    if (opInfo->resourceInfo->alignment == RM_RESOURCE_ALIGNMENT_UNSPECIFIED) {
        /* Get alignment from policy */
        opInfo->resourceInfo->alignment = rmPolicyGetAllocAlign(allocator->policyRoot);
    }

    if (opInfo->resourceInfo->alignment == 0) {
        opInfo->resourceInfo->alignment = 1;
    }

    memset((void *)&findNode, 0, sizeof(findNode));
    findNode.base = opInfo->resourceInfo->base;
    findNode.length = opInfo->resourceInfo->length;

    /* Configure policy checking structure */
    memset((void *)&policyCheckCfg, 0, sizeof(policyCheckCfg));
    policyCheckCfg.polTree = allocator->policyRoot;

    do {
        matchingNode = RB_FIND(_Rm_AllocatorResourceTree,
                               allocator->resourceRoot, &findNode);

        if (matchingNode) {
            matchingEnd = matchingNode->base + matchingNode->length - 1;
            findEnd = findNode.base + findNode.length - 1;
            nodePassesPolicy = RM_BOOL_UNDEF;
            if ((matchingNode->allocationCount == 0) &&
                (findNode.base >= matchingNode->base) &&
                (findEnd <= matchingEnd)) {
                /* Attempt to preallocate from node only if not owned by anyone
                 * and sits within a matching node. */
                policyCheckCfg.type           = policyCheckType;
                policyCheckCfg.negCheck       = RM_FALSE;
                policyCheckCfg.validInstNode  = opInfo->serviceInstNode;
                policyCheckCfg.resourceBase   = findNode.base;
                policyCheckCfg.resourceLength = findNode.length;
                nodePassesPolicy = rmPolicyCheckPrivilege(&policyCheckCfg);

                if (nodePassesPolicy) {
                    /* Is range excluded from UNSPECIFIED allocations? */
                    policyCheckCfg.type     = Rm_policyCheck_UNSPEC_EXCLUSION;
                    policyCheckCfg.negCheck = RM_TRUE;
                    nodePassesPolicy = rmPolicyCheckPrivilege(&policyCheckCfg);
                }

                if (nodePassesPolicy) {
                    /* Initialize indexer to be first resource value that
                     * alignment satisfies */
                    rangeIndex = findNode.base;
                    if (rangeIndex % opInfo->resourceInfo->alignment) {
                        rangeIndex += (opInfo->resourceInfo->alignment -
                                      (rangeIndex %
                                       opInfo->resourceInfo->alignment));
                    }

                    if ((rangeIndex + opInfo->resourceInfo->length - 1) <=
                        matchingEnd) {
                        /* Block of unallocated resources within matchingNode
                         * that satisfies allocate requirements */
                        opInfo->resourceInfo->base = rangeIndex;
                        resourceFound = RM_TRUE;
                        retVal = RM_SERVICE_PROCESSING;
                    }
                }
            }

            if (!resourceFound) {
                /* Check next resource node for available resources */
                if (findNode.base < matchingNode->base) {
                    findNode.base = matchingNode->base;
                } else {
                    if (!nodePassesPolicy) {
                        findNode.base += findNode.length;
                    } else {
                        /* Matching node allocated, move to next node */
                        if (rmInst->instType == Rm_instType_SHARED_SERVER) {
                            nextNode = RB_NEXT_CACHED(_Rm_AllocatorResourceTree,
                                                      allocator->resourceRoot,
                                                      matchingNode);
                        } else {
                            nextNode = RB_NEXT(_Rm_AllocatorResourceTree,
                                               allocator->resourceRoot,
                                               matchingNode);
                        }
                        if (nextNode) {
                            findNode.base = nextNode->base;
                        } else {
                            retVal = RM_SERVICE_DENIED_RES_ALLOC_REQS_NOT_MET;
                        }
                    }
                }
            }
        } else {
            retVal = RM_SERVICE_DENIED_RES_ALLOC_REQS_NOT_MET;
        }
    } while ((!resourceFound) && 
             (retVal != RM_SERVICE_DENIED_RES_ALLOC_REQS_NOT_MET));

    return(retVal);
}

/* FUNCTION PURPOSE: Allocates an allocator resource
 ***********************************************************************
 * DESCRIPTION: Will attempt to allocate the resource with specified
 *              base and length from the resource's allocator.  The
 *              allocation algorithm will verify the allocation against
 *              the policy permissions for the instance requesting the
 *              allocation.  If the policy allows the allocation the 
 *              algorithm will allocate the resource then combine any
 *              resource nodes that may have become equivalent (in terms
 *              of ownership) after the allocation.
 */
static int32_t allocatorAllocate(Rm_Handle rmHandle,
                                 Rm_AllocatorNode *allocator,
                                 Rm_AllocatorOpInfo *opInfo)
{
    Rm_ResourceNode     findNode;
    Rm_ResourceNode    *matchingNode = NULL;
    Rm_ResourceNode    *leftNode = NULL;
    Rm_ResourceNode    *rightNode = NULL;
    Rm_PolicyCheckType  policyCheckType;
    Rm_PolicyCheckCfg   checkCfg;
    int32_t             allocPassesPolicy;
    int                 combineLeft = RM_FALSE;
    int                 combineRight = RM_FALSE;
    uint32_t            findEnd;
    uint32_t            matchingEnd;
    int32_t             retVal;

    if (opInfo->operation == Rm_allocatorOp_ALLOCATE_INIT) {
        policyCheckType = Rm_policyCheck_INIT;
    } else if (opInfo->operation == Rm_allocatorOp_ALLOCATE_USE) {
        policyCheckType = Rm_policyCheck_USE;
    } else {
        retVal = RM_ERROR_INVALID_SERVICE_TYPE;
        return (retVal);
    }

    memset((void *)&findNode, 0, sizeof(findNode));
    findNode.base = opInfo->resourceInfo->base;
    findNode.length = opInfo->resourceInfo->length;
    matchingNode = RB_FIND(_Rm_AllocatorResourceTree, allocator->resourceRoot,
                           &findNode);

    /* Prepare privilege checks */
    memset((void *)&checkCfg, 0, sizeof(checkCfg));

    if (matchingNode) {
        findEnd = findNode.base + findNode.length - 1;
        matchingEnd = matchingNode->base + matchingNode->length - 1;

        if ((findNode.base >= matchingNode->base) && (findEnd <= matchingEnd)) {
            if (opInfo->serviceInstNode == rmPolicyGetLinuxInstNode(rmHandle)) {
                /* Bypass policy checks since Linux Kernel has full
                 * privileges */
                allocPassesPolicy = RM_TRUE;
            } else {
                checkCfg.type           = policyCheckType;
                checkCfg.negCheck       = RM_FALSE;
                checkCfg.polTree        = allocator->policyRoot;
                checkCfg.validInstNode  = opInfo->serviceInstNode;
                checkCfg.resourceBase   = findNode.base;
                checkCfg.resourceLength = findNode.length;
                allocPassesPolicy = rmPolicyCheckPrivilege(&checkCfg);
                if (!allocPassesPolicy) {
                    if (policyCheckType == Rm_policyCheck_INIT) {
                        retVal = RM_SERVICE_DENIED_INIT_PERM_NOT_GIVEN;
                    } else {
                        retVal = RM_SERVICE_DENIED_USE_PERM_NOT_GIVEN;
                    }
                }

                if (!allocatorResNodeIsOwnedBy(rmHandle, matchingNode,
                                               opInfo->serviceInstNode)) {
                    if (allocPassesPolicy &&
                        (matchingNode->allocationCount > 0)) {
                        if (allocatorResNodeIsOwnedBy(rmHandle, matchingNode,
                                          rmPolicyGetLinuxInstNode(rmHandle))) {
                            /* Check if instance requesting resource has
                             * privileges to share a resource already reserved
                             * by Linux */
                            checkCfg.type          = Rm_policyCheck_SHARED_LINUX;
                            checkCfg.negCheck      = RM_FALSE;
                            checkCfg.validInstNode = opInfo->serviceInstNode;
                            allocPassesPolicy = rmPolicyCheckPrivilege(&checkCfg);
                            if (!allocPassesPolicy) {
                                retVal = RM_SERVICE_DENIED_RES_NOT_SHARED_LINUX;
                            }
                        }
                        if (allocPassesPolicy) {
                            /* Check exclusive privileges of instance
                             * requesting resource.  Requesting instance with
                             * exclusive privileges can't reserve resource if
                             * already owned*/
                            checkCfg.type          = Rm_policyCheck_EXCLUSIVE;
                            checkCfg.negCheck      = RM_TRUE;
                            checkCfg.validInstNode = opInfo->serviceInstNode;
                            allocPassesPolicy = rmPolicyCheckPrivilege(&checkCfg);
                            if (!allocPassesPolicy) {
                                retVal = RM_SERVICE_DENIED_EXCLUSIVE_RES_ALLOCD;
                            }
                        }
                    }
                    if (allocPassesPolicy &&
                        (matchingNode->allocationCount == 1)) {
                        /* Check exclusive privileges of instance that
                         * currently owns resource */
                        checkCfg.type          = Rm_policyCheck_EXCLUSIVE;
                        checkCfg.negCheck      = RM_TRUE;
                        checkCfg.validInstNode = matchingNode->ownerList->instNameNode;
                        allocPassesPolicy = rmPolicyCheckPrivilege(&checkCfg);
                        if (!allocPassesPolicy) {
                            retVal = RM_SERVICE_DENIED_ALLOCD_TO_EXCLUSIVE_INST;
                        }
                    }
                }
            }

            if (allocPassesPolicy) {
                /* Handle any possible node combinations if requesting instance
                 * is not already in resource's owner list.  Automatic approval
                 * if requesting instance is already in owner list. */
                if ((findNode.base == matchingNode->base) &&
                    (findEnd == matchingEnd)) {
                    /* findNode range matches matchingNode range
                     *
                     * |<--left node-->||<--matched  node-->||<--right node-->|
                     *                  |<--alloc request-->|
                     */
                    leftNode = RB_PREV(_Rm_AllocatorResourceTree,
                                       allocator->resourceRoot, matchingNode);
                    rightNode = RB_NEXT(_Rm_AllocatorResourceTree,
                                        allocator->resourceRoot, matchingNode);
                    RB_REMOVE(_Rm_AllocatorResourceTree,
                              allocator->resourceRoot, matchingNode);
                    allocatorResNodeOwnerAdd(rmHandle, matchingNode,
                                             opInfo->serviceInstNode);

                    if (leftNode &&
                        allocatorResNodeOwnerCompare(rmHandle,
                                                     leftNode,
                                                     matchingNode) &&
                        allocatorResNodeBoundaryCompare(leftNode,
                                                        matchingNode)) {
                        RB_REMOVE(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, leftNode);
                        combineLeft = RM_TRUE;
                    }
                    if (rightNode &&
                        allocatorResNodeOwnerCompare(rmHandle,
                                                     rightNode,
                                                     matchingNode) &&
                        allocatorResNodeBoundaryCompare(rightNode,
                                                        matchingNode)) {
                        RB_REMOVE(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, rightNode);
                        combineRight = RM_TRUE;
                    }

                    if (combineLeft && combineRight) {
                        /* Combine all three nodes into matchingNode */
                        matchingNode->base = leftNode->base;
                        matchingNode->length = leftNode->length +
                                               matchingNode->length +
                                               rightNode->length;

                        allocatorResNodeOwnerClear(rmHandle, leftNode);
                        rmResourceNodeFree(leftNode);
                        allocatorResNodeOwnerClear(rmHandle, rightNode);
                        rmResourceNodeFree(rightNode);
                    } else if (combineLeft) {
                        /* Combine left and matching nodes.  Reinsert right. */
                        matchingNode->base = leftNode->base;
                        matchingNode->length += leftNode->length;

                        allocatorResNodeOwnerClear(rmHandle, leftNode);
                        rmResourceNodeFree(leftNode);
                        if (rightNode) {
                            RB_INSERT(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, rightNode);  
                        }
                    } else if (combineRight) {
                        /* Combine right and matching nodes.  Reinsert left. */
                        matchingNode->length += rightNode->length;

                        allocatorResNodeOwnerClear(rmHandle, rightNode);
                        rmResourceNodeFree(rightNode);
                        if (leftNode) {
                            RB_INSERT(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, leftNode);
                        }
                    } else {
                        /* No combine. */
                        if (leftNode) {
                            RB_INSERT(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, leftNode);
                        }
                        if (rightNode) {
                            RB_INSERT(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, rightNode);
                        }
                    }

                    /* Always reinsert matchingNode */
                    RB_INSERT(_Rm_AllocatorResourceTree,
                              allocator->resourceRoot, matchingNode);

                    /* Matching node contains new reference count after alloc.
                     * Return new owner count and originating instance
                     * allocation reference count. */
                    opInfo->resourceInfo->ownerCount = matchingNode->allocationCount;
                    opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                      matchingNode,
                                                      opInfo->serviceInstNode);
                } else if ((findNode.base > matchingNode->base) &&
                           (findEnd < matchingEnd)) {
                    /* findNode range is subset of matchingNode range and
                     * neither boundary is equivalent.
                     *
                     * |<----------matched node---------->|
                     *        |<---alloc request--->|
                     */
                    RB_REMOVE(_Rm_AllocatorResourceTree,
                              allocator->resourceRoot, matchingNode);
                    leftNode = rmResourceNodeNew(matchingNode->base,
                                                 findNode.base -
                                                 matchingNode->base);
                    allocatorResNodeOwnerCopy(rmHandle, leftNode, matchingNode);
                    rightNode = rmResourceNodeNew(findNode.base +
                                                  findNode.length,
                                                  matchingEnd - findEnd);
                    allocatorResNodeOwnerCopy(rmHandle, rightNode,
                                              matchingNode);

                    matchingNode->base = findNode.base;
                    matchingNode->length = findNode.length;
                    allocatorResNodeOwnerAdd(rmHandle, matchingNode,
                                             opInfo->serviceInstNode);

                    /* Insert all the nodes */
                    RB_INSERT(_Rm_AllocatorResourceTree,
                              allocator->resourceRoot, matchingNode);
                    RB_INSERT(_Rm_AllocatorResourceTree,
                              allocator->resourceRoot, leftNode);
                    RB_INSERT(_Rm_AllocatorResourceTree,
                              allocator->resourceRoot, rightNode);

                    /* Matching node contains new reference count after alloc.
                     * Return new owner count and originating instance
                     * allocation reference count. */
                    opInfo->resourceInfo->ownerCount = matchingNode->allocationCount;
                    opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                     matchingNode,
                                                     opInfo->serviceInstNode);
                } else {
                    if (findNode.base == matchingNode->base) {
                        /* findNode base and matchingNode base are equivalent.
                         * May be combine possibilities to the left
                         *
                         * |<-left node (alloc'd)->||<-----matched node------->|
                         *                          |<-findNode (alloc req)->|
                         */
                        leftNode = RB_PREV(_Rm_AllocatorResourceTree,
                                           allocator->resourceRoot,
                                           matchingNode);
                        RB_REMOVE(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, matchingNode);
                        /* Add allocating instance to owner list for compare
                         * with leftNode */
                        allocatorResNodeOwnerAdd(rmHandle, matchingNode,
                                                 opInfo->serviceInstNode);

                        if (leftNode &&
                            allocatorResNodeOwnerCompare(rmHandle,
                                                         leftNode,
                                                         matchingNode) &&
                            allocatorResNodeBoundaryCompare(leftNode,
                                                            matchingNode)) {
                            RB_REMOVE(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, leftNode);
                            /* Combine leftNode and findNode */
                            leftNode->length += findNode.length;
                        } else {
                            leftNode = rmResourceNodeNew(findNode.base,
                                                         findNode.length);
                            allocatorResNodeOwnerCopy(rmHandle, leftNode,
                                                      matchingNode);
                        }

                        /* Account for leftNode in matchingNode */
                        matchingNode->base = findNode.base + findNode.length;
                        matchingNode->length = matchingEnd - findEnd;

                        RB_INSERT(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, leftNode);
                        /* Left node contains new reference count after alloc.
                         * Return new owner count and originating instance
                         * allocation reference count. */
                        opInfo->resourceInfo->ownerCount = leftNode->allocationCount;
                        opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                    leftNode,
                                                    opInfo->serviceInstNode);
                    } else /* (findEnd == matchingEnd) */ {
                        /* findNode end and matchingNode end are equivalent.
                         * May be combine possibilities to the right
                         *
                         * |<------matched node----->||<-right node (alloc'd)->|
                         *  |<-findNode (alloc req)->|
                         */
                        rightNode = RB_NEXT(_Rm_AllocatorResourceTree,
                                            allocator->resourceRoot,
                                            matchingNode);
                        RB_REMOVE(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, matchingNode);
                        /* Add allocating instance to owner list for compare
                         * with rightNode */
                        allocatorResNodeOwnerAdd(rmHandle, matchingNode,
                                                 opInfo->serviceInstNode);

                        if (rightNode &&
                            allocatorResNodeOwnerCompare(rmHandle,
                                                         rightNode,
                                                         matchingNode) &&
                            allocatorResNodeBoundaryCompare(rightNode,
                                                            matchingNode)) {
                            RB_REMOVE(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, rightNode);
                            /* Combine rightNode and findNode */
                            rightNode->base = findNode.base;
                            rightNode->length += findNode.length;
                        } else {
                            rightNode = rmResourceNodeNew(findNode.base,
                                                          findNode.length);
                            allocatorResNodeOwnerCopy(rmHandle, rightNode,
                                                      matchingNode);
                        }

                        /* Account for rightNode in matchingNode */
                        matchingNode->length -= findNode.length;

                        RB_INSERT(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, rightNode);
                        /* Right node contains new reference count after alloc.
                         * Return new owner count and originating instance
                         * allocation reference count. */
                        opInfo->resourceInfo->ownerCount = rightNode->allocationCount;
                        opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                    rightNode,
                                                    opInfo->serviceInstNode);
                    }
                    /* Remove allocating instance from leftover matchingNode */
                    allocatorResNodeOwnerDelete(rmHandle, matchingNode,
                                                opInfo->serviceInstNode);
                    RB_INSERT(_Rm_AllocatorResourceTree,
                              allocator->resourceRoot, matchingNode);
                }
                retVal = RM_SERVICE_APPROVED;
            }
        } else {
            retVal = RM_SERVICE_DENIED_PARTIAL_ALLOCATION;
        }
    } else {
        retVal = RM_SERVICE_DENIED_RES_RANGE_DOES_NOT_EXIST;
    }

    return(retVal);
}

/* FUNCTION PURPOSE: Frees an allocator resource
 ***********************************************************************
 * DESCRIPTION: Will attempt to free the resource with specified
 *              base and length from the resource's allocator.  The
 *              free algorithm will verify the free request parameters
 *              match an allocated range for the resource and that the
 *              range is owned by the instance requesting the free. If
 *              the free is validated the algorithm will free the 
 *              resource then combine any resource nodes that may have
 *              become equivalent (in terms of ownership) after the
 *              allocation.
 */
static int32_t allocatorFree(Rm_Handle rmHandle, Rm_AllocatorNode *allocator,
                             Rm_AllocatorOpInfo *opInfo)
{
    Rm_ResourceNode  findNode;
    Rm_ResourceNode *matchingNode = NULL;
    Rm_ResourceNode *leftNode = NULL;
    Rm_ResourceNode *rightNode = NULL;
    int              combineLeft = RM_FALSE;
    int              combineRight = RM_FALSE;
    uint32_t         findEnd;
    uint32_t         matchingEnd;
    int32_t          retVal;

    memset((void *)&findNode, 0, sizeof(findNode));
    findNode.base = opInfo->resourceInfo->base;
    findNode.length = opInfo->resourceInfo->length;
    matchingNode = RB_FIND(_Rm_AllocatorResourceTree, allocator->resourceRoot,
                           &findNode);

    if (matchingNode) {
        findEnd = findNode.base + findNode.length - 1;
        matchingEnd = matchingNode->base + matchingNode->length - 1;
        
        if ((findNode.base >= matchingNode->base) && (findEnd <= matchingEnd)) {
            if (matchingNode->allocationCount) {
                if (allocatorResNodeIsOwnedBy(rmHandle, matchingNode,
                    opInfo->serviceInstNode)) {
                    if ((findNode.base == matchingNode->base) &&
                        (findEnd == matchingEnd)) {
                        /* Case 1: Free range equals allocated matched node
                         *         exactly. Attempt to combine freed node with
                         *         nodes to left and right.
                         *
                         * |<-left node->||<---matched node--->||<-right node->|
                         *                |<---free request--->|
                         */
                        leftNode = RB_PREV(_Rm_AllocatorResourceTree,
                                           allocator->resourceRoot,
                                           matchingNode);
                        rightNode = RB_NEXT(_Rm_AllocatorResourceTree,
                                            allocator->resourceRoot,
                                            matchingNode);
                        RB_REMOVE(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, matchingNode);
                        allocatorResNodeOwnerDelete(rmHandle, matchingNode,
                                                    opInfo->serviceInstNode);

                        if (leftNode &&
                            allocatorResNodeOwnerCompare(rmHandle,
                                                         leftNode,
                                                         matchingNode) &&
                            allocatorResNodeBoundaryCompare(leftNode,
                                                            matchingNode)) {
                            RB_REMOVE(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, leftNode);
                            combineLeft = RM_TRUE;
                        }
                        if (rightNode &&
                            allocatorResNodeOwnerCompare(rmHandle,
                                                         rightNode,
                                                         matchingNode) &&
                            allocatorResNodeBoundaryCompare(rightNode,
                                                            matchingNode)) {
                            RB_REMOVE(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, rightNode);
                            combineRight = RM_TRUE;
                        }

                        if (combineLeft && combineRight) {
                            /* Combine all three nodes into matchingNode */
                            matchingNode->base = leftNode->base;
                            matchingNode->length = leftNode->length +
                                                   matchingNode->length +
                                                   rightNode->length;

                            allocatorResNodeOwnerClear(rmHandle, leftNode);
                            rmResourceNodeFree(leftNode);
                            allocatorResNodeOwnerClear(rmHandle, rightNode);
                            rmResourceNodeFree(rightNode);
                        } else if (combineLeft) {
                            /* Combine left and matching nodes.
                             * Reinsert right. */
                            matchingNode->base = leftNode->base;
                            matchingNode->length += leftNode->length;

                            allocatorResNodeOwnerClear(rmHandle, leftNode);
                            rmResourceNodeFree(leftNode);
                            if (rightNode) {
                                RB_INSERT(_Rm_AllocatorResourceTree,
                                          allocator->resourceRoot, rightNode);
                            }
                        } else if (combineRight) {
                            /* Combine right and matching nodes.
                             * Reinsert left. */
                            matchingNode->length += rightNode->length;

                            allocatorResNodeOwnerClear(rmHandle, rightNode);
                            rmResourceNodeFree(rightNode);
                            if (leftNode) {
                                RB_INSERT(_Rm_AllocatorResourceTree,
                                          allocator->resourceRoot, leftNode);
                            }
                        } else {
                            /* No combine. */
                            if (leftNode) {
                                RB_INSERT(_Rm_AllocatorResourceTree,
                                          allocator->resourceRoot, leftNode);
                            }
                            if (rightNode) {
                                RB_INSERT(_Rm_AllocatorResourceTree,
                                          allocator->resourceRoot, rightNode);
                            }
                        }

                        /* Always reinsert matchingNode */
                        RB_INSERT(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, matchingNode);
                        
                        /* Matching node is what remains after free.  Return
                         * remaining owner count and originating instance
                         * allocation reference count. */
                        opInfo->resourceInfo->ownerCount = matchingNode->allocationCount;
                        opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                    matchingNode,
                                                    opInfo->serviceInstNode);
                    } else if ((findNode.base > matchingNode->base) &&
                               (findEnd < matchingEnd)) {
                        /* Case 2: Free range is less than range in matched
                         *         node. Split matched node into three nodes.
                         *
                         * |<----------matched node---------->|
                         *        |<---free request--->|
                         *
                         * Remove instance from owner list then add it back in
                         * for side nodes for proper accounting of allocations
                         * in validInstance list
                         */
                        RB_REMOVE(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, matchingNode);
                        allocatorResNodeOwnerDelete(rmHandle, matchingNode,
                                                    opInfo->serviceInstNode);

                        leftNode = rmResourceNodeNew(matchingNode->base,
                                                     findNode.base -
                                                     matchingNode->base);
                        allocatorResNodeOwnerCopy(rmHandle, leftNode,
                                                  matchingNode);
                        allocatorResNodeOwnerAdd(rmHandle, leftNode,
                                                 opInfo->serviceInstNode);
                        RB_INSERT(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, leftNode);

                        rightNode = rmResourceNodeNew(findNode.base +
                                                      findNode.length,
                                                      matchingEnd - findEnd);
                        allocatorResNodeOwnerCopy(rmHandle, rightNode,
                                                  matchingNode);
                        allocatorResNodeOwnerAdd(rmHandle, rightNode,
                                                 opInfo->serviceInstNode);
                        RB_INSERT(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, rightNode);

                        matchingNode->base = findNode.base;
                        matchingNode->length = findNode.length;
                        RB_INSERT(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, matchingNode);

                        /* Matching node is what remains after free.  Return
                         * remaining owner count and originating instance
                         * allocation reference count. */
                        opInfo->resourceInfo->ownerCount = matchingNode->allocationCount;
                        opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                    matchingNode,
                                                    opInfo->serviceInstNode);
                    } else {
                        if (findNode.base == matchingNode->base) {
                            /* Case 3: Free range is on left boundary of
                             *         matched node. Try to combine free range
                             *         with left node.
                             *
                             * |<-left node (free)->||<-----matched node------>|
                             *                       |<-findNode (free req)->|
                             */

                            leftNode = RB_PREV(_Rm_AllocatorResourceTree,
                                               allocator->resourceRoot,
                                               matchingNode);
                            RB_REMOVE(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, matchingNode);
                            /* Remove freeing instance from owner list for
                             * compare with leftNode */
                            allocatorResNodeOwnerDelete(rmHandle, matchingNode,
                                                       opInfo->serviceInstNode);

                            if (leftNode &&
                                allocatorResNodeOwnerCompare(rmHandle,
                                                             leftNode,
                                                             matchingNode) &&
                                allocatorResNodeBoundaryCompare(leftNode,
                                                                matchingNode)) {
                                RB_REMOVE(_Rm_AllocatorResourceTree,
                                          allocator->resourceRoot, leftNode);
                                /* Combine leftNode and findNode */
                                leftNode->length += findNode.length;
                            } else {
                                leftNode = rmResourceNodeNew(findNode.base,
                                                             findNode.length);
                                allocatorResNodeOwnerCopy(rmHandle, leftNode,
                                                          matchingNode);
                            }

                            /* Remove leftNode range from matchingNode */
                            matchingNode->base = findNode.base +
                                                 findNode.length;
                            matchingNode->length = matchingEnd - findEnd;
                            RB_INSERT(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, leftNode);

                            /* Left node is what remains after free.  Return
                             * remaining owner count and originating instance
                             * allocation reference count. */
                            opInfo->resourceInfo->ownerCount = leftNode->allocationCount;
                            opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                    leftNode,
                                                    opInfo->serviceInstNode);
                        } else /* (findEnd == matchingEnd) */ {
                            /* Case 4: Free range is on right boundary of
                             *         matched node. Try to combine free range
                             *         with right node.
                             *
                             * |<-----matched node----->||<-right node (free)->|
                             *  |<-findNode (free req)->|
                             */

                            rightNode = RB_NEXT(_Rm_AllocatorResourceTree,
                                                allocator->resourceRoot,
                                                matchingNode);
                            RB_REMOVE(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, matchingNode);
                            /* Remove freeing instance from owner list for
                             * compare with rightNode */
                            allocatorResNodeOwnerDelete(rmHandle, matchingNode,
                                                       opInfo->serviceInstNode);
                            
                            if (rightNode &&
                                allocatorResNodeOwnerCompare(rmHandle,
                                                             rightNode,
                                                             matchingNode) &&
                                allocatorResNodeBoundaryCompare(rightNode,
                                                                matchingNode)) {
                                RB_REMOVE(_Rm_AllocatorResourceTree,
                                          allocator->resourceRoot, rightNode);
                                /* Combine rightNode and findNode */
                                rightNode->base = findNode.base;
                                rightNode->length += findNode.length;
                            } else {
                                rightNode = rmResourceNodeNew(findNode.base,
                                                              findNode.length);
                                allocatorResNodeOwnerCopy(rmHandle, rightNode,
                                                          matchingNode);
                            }

                            /* Remove rightNode range from matchingNode */
                            matchingNode->length -= findNode.length;
                            RB_INSERT(_Rm_AllocatorResourceTree,
                                      allocator->resourceRoot, rightNode);

                            /* Right node is what remains after free.  Return
                             * remaining owner count and originating instance
                             * allocation reference count. */
                            opInfo->resourceInfo->ownerCount = rightNode->allocationCount;
                            opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                    rightNode,
                                                    opInfo->serviceInstNode);
                        }

                        /* Add freeing instance back into matchingNode
                         * allocations */
                        allocatorResNodeOwnerAdd(rmHandle, matchingNode,
                                                 opInfo->serviceInstNode);
                        RB_INSERT(_Rm_AllocatorResourceTree,
                                  allocator->resourceRoot, matchingNode);
                    }
                    retVal = RM_SERVICE_APPROVED;
                } else {
                    /* Return owner count and instance alloc count.  In case
                     * it's a reference count check in application */
                    opInfo->resourceInfo->ownerCount = matchingNode->allocationCount;
                    opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                    matchingNode,
                                                    opInfo->serviceInstNode);
                    retVal = RM_SERVICE_DENIED_RES_NOT_ALLOCD_TO_INST;
                }
            } else {
                /* Return owner count and instance alloc count.  In case it's
                 * a reference count check in application */
                opInfo->resourceInfo->ownerCount = matchingNode->allocationCount;
                opInfo->resourceInfo->instAllocCount = allocatorResNodeOwnerGetRefCnt(rmHandle,
                                                    matchingNode,
                                                    opInfo->serviceInstNode);
                retVal = RM_SERVICE_DENIED_RES_ALREADY_FREE;
            }
        } else {
            retVal = RM_SERVICE_DENIED_PARTIAL_FREE;
        }
    } else {
        retVal = RM_SERVICE_DENIED_RES_RANGE_DOES_NOT_EXIST;
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Reserves a Linux resource
 ***********************************************************************
 * DESCRIPTION: Reserves resources for Linux using the base and length
 *              values retrieved from the Linux DTB via the
 *              "linux-dtb-alias" properties within the GRL.
 */
static int32_t allocatorReserveLinuxResource(Rm_Handle rmHandle,
                                             Rm_LinuxAlias *linuxAlias,
                                             Rm_LinuxValueRange *linuxValues,
                                             Rm_AllocatorOpInfo *opInfo)
{
    int32_t  retVal = RM_OK;
    int      baseFound = RM_FALSE;
    int      lengthFound = RM_FALSE;
    uint32_t valueIndex = 0;

    while ((linuxValues) && (!baseFound || !lengthFound)) {
        if (linuxAlias->baseOffset == valueIndex) {
            opInfo->resourceInfo->base = linuxValues->value;
            baseFound = RM_TRUE;

            if (linuxAlias->lengthOffset ==
                RM_DTB_UTIL_LINUX_ALIAS_OFFSET_NOT_SET) {
                opInfo->resourceInfo->length = 1;
                lengthFound = RM_TRUE;
            }
        } else if (linuxAlias->lengthOffset == valueIndex) {
            opInfo->resourceInfo->length = linuxValues->value;
            lengthFound = RM_TRUE;
        }
        /* else: value was not a base or length so skip to next alias value */

        linuxValues = (Rm_LinuxValueRange *)linuxValues->nextValue;
        valueIndex++;
    }

    if (!baseFound || !lengthFound) {
        retVal = RM_ERROR_DATA_NOT_FOUND_AT_LINUX_ALIAS;
    } else {
        /* Allocate resource to Linux */
        retVal = rmAllocatorOperation(rmHandle, opInfo);
        if (retVal == RM_SERVICE_APPROVED) {
            retVal = RM_OK;
        }
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Finds and reserves Linux resources
 ***********************************************************************
 * DESCRIPTION: Parses the Linux DTB for resources consumed by the
 *              Linux kernel.  If the resource is found via the
 *              "linux-dtb-alias" property defined in the GRL it is 
 *              reserved.
 */
static int32_t allocatorFindLinuxResource(Rm_Handle rmHandle,
                                          const char *resourceName,
                                          void *linuxDtb,
                                          Rm_LinuxAlias *linuxAlias)
{
    Rm_AllocatorOpInfo  opInfo;
    Rm_ResourceInfo     resourceInfo;
    uint32_t            pathOffset;
    uint32_t            pathSize;
    char               *tempAliasPath;
    char               *spacePtr;
    int32_t             propOffset;
    int32_t             nodeOffset = RM_DTB_UTIL_STARTING_NODE_OFFSET;
    int32_t             prevDepth = RM_DTB_UTIL_STARTING_DEPTH;
    int32_t             depth = RM_DTB_UTIL_STARTING_DEPTH;
    int32_t             propertyLen;
    const char         *propertyName;
    const void         *propertyData;
    Rm_LinuxValueRange *linValRange;
    int32_t             retVal = RM_OK;

    memset((void *)&opInfo, 0, sizeof(opInfo));
    memset((void *)&resourceInfo, 0, sizeof(resourceInfo));

    rm_strncpy(resourceInfo.name, resourceName, RM_NAME_MAX_CHARS);
    opInfo.serviceInstNode = rmPolicyGetLinuxInstNode(rmHandle);
    opInfo.operation       = Rm_allocatorOp_ALLOCATE_INIT;
    opInfo.resourceInfo    = &resourceInfo;

    if (!opInfo.serviceInstNode) {
        retVal = RM_SERVICE_DENIED_INST_NAME_NOT_VALID;
        goto errorExit;
    }

    while(linuxAlias) {
        /* Reset parsing variables */
        pathOffset = 0;
        pathSize = strlen(linuxAlias->path) + 1;
        tempAliasPath = Rm_osalMalloc(pathSize);
        rm_strncpy(tempAliasPath, linuxAlias->path, pathSize);
        nodeOffset = RM_DTB_UTIL_STARTING_NODE_OFFSET;
        prevDepth = RM_DTB_UTIL_STARTING_DEPTH;
        resourceInfo.base = 0;
        resourceInfo.length = 0;

        spacePtr = strpbrk(tempAliasPath, " ");
        if (spacePtr) {
            *spacePtr = '\0';
        }

        while(pathOffset < pathSize) {
            /* Move through DTB nodes until next alias path node found */
            if (strcmp(tempAliasPath + pathOffset,
                       fdt_get_name(linuxDtb, nodeOffset, NULL))) {
                nodeOffset = fdt_next_node(linuxDtb, nodeOffset, &depth);

                if ((depth < prevDepth) || (nodeOffset == -FDT_ERR_NOTFOUND)) {
                    /* Returning from subnode that matched part of alias path
                     * without finding resource values */
                    retVal = RM_ERROR_DATA_NOT_FOUND_AT_LINUX_ALIAS;
                    break;
                }
            } else {
                /* Found next alias path node.  Move to next node name in path
                 * string. */
                pathOffset += (strlen(tempAliasPath + pathOffset) + 1);
                spacePtr = strpbrk(tempAliasPath + pathOffset, " ");
                if (spacePtr) {
                    *spacePtr = '\0';
                }

                prevDepth = fdt_node_depth(linuxDtb, nodeOffset);
                propOffset = fdt_first_property_offset(linuxDtb, nodeOffset);
                while ((propOffset >= RM_DTB_UTIL_STARTING_NODE_OFFSET) &&
                       (pathOffset < pathSize)) {
                    propertyData = fdt_getprop_by_offset(linuxDtb, propOffset,
                                                         &propertyName,
                                                         &propertyLen);

                    if (strcmp(tempAliasPath + pathOffset, propertyName) == 0) {
                        /* Found resource at end of alias path */
                        pathOffset += (strlen(tempAliasPath + pathOffset) + 1);
                        linValRange = rmDtbUtilLinuxExtractValues(propertyData,
                                                                  propertyLen);
                        retVal = allocatorReserveLinuxResource(rmHandle,
                                                               linuxAlias,
                                                               linValRange,
                                                               &opInfo);
                        rmDtbUtilLinuxFreeValues(linValRange);
                    }
                    propOffset = fdt_next_property_offset(linuxDtb, propOffset);
                }

                if (propOffset < -FDT_ERR_NOTFOUND) {
                    retVal = propOffset;
                    break;
                }
            }
        }

        Rm_osalFree(tempAliasPath, pathSize);
        if (retVal < RM_OK) {
            break;
        }
        linuxAlias = linuxAlias->nextLinuxAlias;
    }
errorExit:
    return(retVal);
}

/* FUNCTION PURPOSE: Populates an allocator's resource tree
 ***********************************************************************
 * DESCRIPTION: Uses resource range information pulled from GRL to
 *              populate an allocator's resource tree
 */
static int32_t allocatorPopulateResTree(Rm_ResourceTree *resTree,
                                        Rm_ResourceRange *range)
{
    Rm_ResourceNode *resNode = NULL;
    int32_t retVal = RM_OK;

    while (range != NULL) {
        if ((resNode = rmResourceNodeNew(range->base, range->length))) {
            RB_INSERT(_Rm_AllocatorResourceTree, resTree,
                      resNode);
        } else {
            retVal = RM_ERROR_MALLOC_FAILED_RES_NODE;
            break;
        }
        range = range->nextRange;
    }

    return(retVal);
}

/* FUNCTION PURPOSE: Initializes a new allocator
 ***********************************************************************
 * DESCRIPTION: allocates and initializes a new allocator for the
 *              provided resource name.  The resource and policy tree
 *              root nodes are allocated and initialized as part of
 *              the allocator node initialization.
 */
static Rm_AllocatorNode *allocatorInitNode(Rm_Inst *rmInst,
                                           const char *resourceName,
                                           int32_t *retVal)
{
    Rm_AllocatorTree *allocTree  = rmInst->allocatorTree;
    Rm_AllocatorNode *newAllocNode = NULL;
    Rm_ResourceTree  *resTree = NULL;
    Rm_PolicyTree    *polTree = NULL;

    *retVal = RM_OK;

    if ((strlen(resourceName) + 1) > RM_NAME_MAX_CHARS) {
        *retVal = RM_ERROR_RESOURCE_NAME_TOO_LONG;
        goto errorExit;
    }

    newAllocNode = rmAllocatorNodeNew(resourceName);
    if (newAllocNode) {
        if (RB_INSERT(_Rm_AllocatorTree, allocTree, newAllocNode)) {
            /* Collision */
            *retVal = RM_ERROR_RES_SPECIFIED_MORE_THAN_ONCE;
            goto errorExit;
        }

        if ((resTree = Rm_osalMalloc(sizeof(*resTree)))) {
            RB_INIT(resTree);
            newAllocNode->resourceRoot = resTree;
        } else {
            *retVal = RM_ERROR_MALLOC_FAILED_RES_TREE;
            goto errorExit;
        }

        if ((polTree = Rm_osalMalloc(sizeof(*polTree)))) {
            RB_INIT(polTree);
            newAllocNode->policyRoot = polTree;
        } else {
            *retVal = RM_ERROR_MALLOC_FAILED_POL_TREE;
            goto errorExit;
        }
    } else {
        *retVal = RM_ERROR_COULD_NOT_CREATE_NEW_ALLOCATOR;
        goto errorExit;
    }

errorExit:
    if ((*retVal != RM_OK) && newAllocNode) {
        rmAllocatorNodeFree(newAllocNode);
    }
    return(newAllocNode);
}

/* FUNCTION PURPOSE: Creates and initializes an allocator node
 ***********************************************************************
 * DESCRIPTION: Creates an allocator for the provided resource name.
 *              The resource properties retrieved from the GRL are used
 *              to create a resource tree.  Resources will be reserved
 *              for the Linux kernel if the Linux DTB is provided and
 *              there are "linux-dtb-alias" properties specified in
 *              the GRL.  A policy tree will be created using the
 *              resource's entry in the policy.
 */
static int32_t allocatorCreateNode(Rm_Inst *rmInst, void *policyDtb,
                                   void *linuxDtb, const char *resourceName,
                                   Rm_ResourceProperties *resProps)
{
    Rm_AllocatorNode *allocNode = NULL;
    Rm_ResourceRange *range = NULL;
    Rm_LinuxAlias    *linuxAlias = NULL;
    int32_t           retVal = RM_OK;

    allocNode = allocatorInitNode(rmInst, resourceName, &retVal);
    if (allocNode) {
        range = rmDtbUtilResExtractRange(resProps->rangeData,
                                         resProps->rangeLen);
        retVal = allocatorPopulateResTree(allocNode->resourceRoot, range);
        if (retVal != RM_OK) {
            goto errorExit;
        }
        /* Create the companion policy tree for the resource */
        retVal = rmPolicyPopulateTree((Rm_Handle)rmInst, allocNode->policyRoot,
                                      policyDtb, resourceName);
        if (retVal != RM_OK) {
            goto errorExit;
        }

        if (rmInst->instType == Rm_instType_SHARED_SERVER) {
            /* Writeback resource tree for Linux resource reservation which
             * uses path through rmAllocatorOperation function.  This function
             * performs an invalidate of resource tree */
            rmResourceTreeWb(allocNode->resourceRoot);
        }

        if (resProps->linuxAliasData && linuxDtb) {
            linuxAlias = rmDtbUtilResExtractLinuxAlias(resProps->linuxAliasData,
                                                       resProps->linuxAliasLen,
                                                       &retVal);
            /* linuxAlias will be NULL if retVal contains error code */
            if (linuxAlias) {
                retVal = allocatorFindLinuxResource(rmInst, resourceName,
                                                    linuxDtb, linuxAlias);
            }

            if (retVal != RM_OK) {
                goto errorExit;
            }
        }

errorExit:
        if (range) {
            rmDtbUtilResFreeRange(range);
        }
        if (linuxAlias) {
            rmDtbUtilResFreeLinuxAlias(linuxAlias);
        }
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Creates NameServer assignment entries
 ***********************************************************************
 * DESCRIPTION: Creates a NameServer entry for each NameServer assignment
 *              found in a GRL's resource node
 */
static int32_t allocatorNsAdd(Rm_Inst *rmInst, const char *resourceName,
                              Rm_ResourceProperties *resProps)
{
    Rm_NsAssignment     *nsAssigns = NULL;
    Rm_NsAssignment     *nsAssignsBase = NULL;
    Rm_NameServerObjCfg  nsCfg;
    int32_t              retVal = RM_OK;

    if (resProps->nsAssignData) {
        nsAssigns = rmDtbUtilResExtractNsAssignment(resProps->nsAssignData,
                                                    resProps->nsAssignLen,
                                                    &retVal);
        /* nsAssignments will be NULL if retVal contains error code */
        if (nsAssigns) {
            nsAssignsBase = nsAssigns;
            while (nsAssigns) {
                memset((void *)&nsCfg, 0, sizeof(nsCfg));
                nsCfg.nameServerTree = rmInst->u.server.nameServer;
                nsCfg.nodeCfg.objName = nsAssigns->nsName;
                nsCfg.nodeCfg.resourceName = (char *)resourceName;
                nsCfg.nodeCfg.resourceBase= nsAssigns->resourceBase;
                nsCfg.nodeCfg.resourceLength = nsAssigns->resourceLength;
                rmNameServerAddObject(&nsCfg);
                nsAssigns = nsAssigns->nextNsAssignment;
            }
            rmDtbUtilResFreeNsAssignmentList(nsAssignsBase);
        }
    }

    return(retVal);
}


/* FUNCTION PURPOSE: Populate the allocator tree based on the GRL
 ***********************************************************************
 * DESCRIPTION: Populates the allocator tree using the GRL and policy DTBs.
 *              Optionally, the Linux DTB will be scanned for resources used
 *              by the Kernel if the Linux DTB is non-NULL.
 */
static int32_t allocatorPopulateGrlBased(Rm_Inst *rmInst, void *grlDtb,
                                         void *policyDtb, void *linuxDtb)
{
    int32_t                nodeOffset = RM_DTB_UTIL_STARTING_NODE_OFFSET;
    int32_t                nodeDepth = RM_DTB_UTIL_STARTING_DEPTH;
    Rm_ResourceProperties  resProps;
    int32_t                propOffset;
    int32_t                propertyLen;
    const char            *propertyName;
    const void            *propertyData;
    Rm_ResourcePropType    propertyType;
    int32_t                retVal = RM_OK;

    /* Create allocator tree node with resource and policy trees for
     * each resource found in the GRL. */
    while ((nodeOffset >= RM_DTB_UTIL_STARTING_NODE_OFFSET) &&
           (nodeDepth >= RM_DTB_UTIL_STARTING_DEPTH)) {

        memset((void *)&resProps, 0, sizeof(resProps));
        /* Get properties of resource node */
        propOffset = fdt_first_property_offset(grlDtb, nodeOffset);

        while (propOffset >= RM_DTB_UTIL_STARTING_NODE_OFFSET) {
            propertyData = fdt_getprop_by_offset(grlDtb, propOffset,
                                                 &propertyName, &propertyLen);
            propertyType = rmDtbUtilResGetPropertyType(propertyName);
            switch(propertyType) {
                case Rm_resourcePropType_RESOURCE_RANGE:
                    resProps.rangeData = propertyData;
                    resProps.rangeLen = propertyLen;
                    break;
                case Rm_resourcePropType_NSASSIGNMENT:
                    resProps.nsAssignData = propertyData;
                    resProps.nsAssignLen = propertyLen;
                    break;
                case Rm_resourcePropType_RESOURCE_LINUX_ALIAS:
                    resProps.linuxAliasData = propertyData;
                    resProps.linuxAliasLen = propertyLen;
                    break;
                default:
                    retVal = RM_ERROR_GRL_UNKNOWN_RESOURCE_PROPERTY;
                    goto errorExit;
            }

            propOffset = fdt_next_property_offset(grlDtb, propOffset);
            if (propOffset == -FDT_ERR_NOTFOUND) {
                const char *resName = fdt_get_name(grlDtb, nodeOffset,
                                                   NULL);

                if ((!resProps.rangeData) && (!resProps.nsAssignData)) {
                    retVal = RM_ERROR_GRL_INVALID_NODE_DEF;
                    goto errorExit;
                }

                if (resProps.rangeData) {
                    /* At least range property found.  Create allocator node
                     * using extracted values for resource tree and
                     * resource's policy entry for policy tree */
                    retVal = allocatorCreateNode(rmInst, policyDtb, linuxDtb,
                                                 resName, &resProps);
                    if (retVal != RM_OK) {
                        goto errorExit;
                    }
                }

                if (resProps.nsAssignData) {
                    retVal = allocatorNsAdd(rmInst, resName, &resProps);
                    if (retVal != RM_OK) {
                        goto errorExit;
                    }
                }
            } else if (propOffset < -FDT_ERR_NOTFOUND) {
                /* Error returned by LIBFDT */
                retVal = propOffset;
                goto errorExit;
            }
            /* else: fall through to get next property */
        }
        if (propOffset < -FDT_ERR_NOTFOUND) {
            /* Error returned by LIBFDT */
            retVal = propOffset;
            goto errorExit;
        }

        nodeOffset = fdt_next_node(grlDtb, nodeOffset, &nodeDepth);
        if (nodeOffset < -FDT_ERR_NOTFOUND) {
            /* Error returned by LIBFDT */
            retVal = nodeOffset;
            goto errorExit;
        }
    }

errorExit:
    return(retVal);
}

/* FUNCTION PURPOSE: Populate the allocator tree based on the Policy DTB
 ***********************************************************************
 * DESCRIPTION: Populates the allocator tree using the policy DTB.  The
 *              resource trees in each allocator will be created on the fly
 *              as a CD requests resources from the server.  Client instances
 *              will never create resource trees since they'll only use the
 *              allocator's policy trees for the static initialization phase.
 */
static int32_t allocatorPopulatePolicyBased(Rm_Inst *rmInst, void *policyDtb)
{
    int32_t            nodeOffset = RM_DTB_UTIL_STARTING_NODE_OFFSET;
    int32_t            nodeDepth = RM_DTB_UTIL_STARTING_DEPTH;
    int32_t            propOffset;
    const char        *resName;
    const char        *propName;
    Rm_PolicyPropType  propType;
    Rm_AllocatorNode  *newAllocNode = NULL;
    int32_t            retVal = RM_OK;

    /* Search for resource node definitions in the policy */
    while ((nodeOffset >= RM_DTB_UTIL_STARTING_NODE_OFFSET) &&
           (nodeDepth >= RM_DTB_UTIL_STARTING_DEPTH)) {

        propOffset = fdt_first_property_offset(policyDtb, nodeOffset);
        while (propOffset > RM_DTB_UTIL_STARTING_NODE_OFFSET) {
            fdt_getprop_by_offset(policyDtb, propOffset, &propName, NULL);
            propType = rmDtbUtilPolicyGetPropertyType(propName);
            if (propType == Rm_policyPropType_ASSIGNMENTS) {
                /* Found a resource node's assignment property.  Create an
                 * allocator node for the resource and populate it with a
                 * policy tree */
                resName = fdt_get_name(policyDtb, nodeOffset, NULL);

                newAllocNode = allocatorInitNode(rmInst, resName, &retVal);
                if (newAllocNode) {
                    retVal = rmPolicyPopulateTree((Rm_Handle)rmInst,
                                                  newAllocNode->policyRoot,
                                                  policyDtb, resName);
                    if (retVal != RM_OK) {
                        goto errorExit;
                    }
                } else {
                    goto errorExit;
                }

                /* Move on to next resource node */
                break;
            } else if (propType == Rm_policyPropType_UNKNOWN) {
                retVal = RM_ERROR_UNKNOWN_POLICY_RESOURCE_PROPERTY;
                goto errorExit;
            }
            /* else: fall through to get next property since read property
             *       wasn't an assignment */

            propOffset = fdt_next_property_offset(policyDtb, propOffset);
        }
        if (propOffset < -FDT_ERR_NOTFOUND) {
            /* Error returned by LIBFDT */
            retVal = propOffset;
            goto errorExit;
        }

        nodeOffset = fdt_next_node(policyDtb, nodeOffset, &nodeDepth);
        if (nodeOffset < -FDT_ERR_NOTFOUND) {
            /* Error returned by LIBFDT */
            retVal = nodeOffset;
            goto errorExit;
        }
    }

errorExit:
    return(retVal);
}

/* FUNCTION PURPOSE: Initializes the allocator tree root
 ***********************************************************************
 * DESCRIPTION: Initializes the allocator tree root structure
 */
static int32_t allocatorTreeRootInit(Rm_Inst *rmInst)
{
    Rm_AllocatorTree *root = NULL;
    int32_t           retVal = RM_OK;

    root = Rm_osalMalloc(sizeof(*root));
    if (root) {
        RB_INIT(root);
        rmInst->allocatorTree = root;
    } else {
        retVal = RM_ERROR_COULD_NOT_INIT_ALLOC_TREE;
    }
    return(retVal);
}

/**********************************************************************
 ********************** Internal Functions ****************************
 **********************************************************************/

/* FUNCTION PURPOSE: Finds an allocator
 ***********************************************************************
 * DESCRIPTION: Returns a pointer to an allocator that matches the 
 *              provided resource name.
 */
Rm_AllocatorNode *rmAllocatorFind(Rm_Handle rmHandle, const char *resourceName)
{
    Rm_Inst          *rmInst = (Rm_Inst *)rmHandle;
    Rm_AllocatorTree *tree = rmInst->allocatorTree;
    Rm_AllocatorNode  findNode;

    memset((void *)&findNode, 0, sizeof(findNode));
    rm_strncpy(findNode.resourceName, resourceName, RM_NAME_MAX_CHARS);

    return(RB_FIND(_Rm_AllocatorTree, tree, &findNode));
}

/* FUNCTION PURPOSE: Checks if a resource node is localized
 ***********************************************************************
 * DESCRIPTION: Checks if a resource node is localized.  A localized
 *              node is one that is free and has no neighboring nodes
 *              or neighboring nodes that do not have resource values
 *              contiguous with the node being checked.  The function
 *              will return RM_TRUE if the node is localized.  
 *              Otherwise, the function returns RM_FALSE
 */
int rmAllocatorGetNodeLocalization(Rm_Handle rmHandle, char *resourceName,
                                   int32_t *resBase, uint32_t *resLen)
{
    uint32_t          allocSize;
    Rm_AllocatorNode *allocator = NULL;
    Rm_ResourceNode   findNode;
    Rm_ResourceNode  *matchingNode = NULL;
    Rm_ResourceNode  *neighborNode = NULL;
    int               nodeIsLocalized = RM_FALSE;

    allocator = rmAllocatorFind(rmHandle, resourceName);
    allocSize = rmPolicyGetCdAllocSize(allocator->policyRoot);

    /* Nothing to free back to server if policy never specified blocks could
     * be allocated to CD */
    if (allocSize) {
        memset((void *)&findNode, 0, sizeof(findNode));
        findNode.base = *resBase;
        findNode.length = *resLen;
        matchingNode = RB_FIND(_Rm_AllocatorResourceTree,
                               allocator->resourceRoot, &findNode);

        if (matchingNode) {
            /* Node can be freed back to Server from CD if:
             * - allocationCount == 0
             * - node's resource range is multiple of policy allocation size
             * - node's resource range boundaries are not contiguous with
             *   surrounding nodes */
            if (matchingNode->allocationCount) {
                goto exitLocalization;
            }

            if (matchingNode->length % allocSize) {
                goto exitLocalization;
            }

            /* Check left neighbor */
            neighborNode = RB_PREV(_Rm_AllocatorResourceTree,
                                   allocator->resourceRoot, matchingNode);
            if (neighborNode &&
                allocatorResNodeBoundaryCompare(neighborNode, matchingNode)) {
                goto exitLocalization; 
            }

            /* Check right neighbor */
            neighborNode = RB_NEXT(_Rm_AllocatorResourceTree,
                                   allocator->resourceRoot, matchingNode);
            if (neighborNode &&
                allocatorResNodeBoundaryCompare(neighborNode, matchingNode)) {
                goto exitLocalization; 
            }

            /* All localization checks passed.  Return the base and length of
             * localized node. */
            nodeIsLocalized = RM_TRUE;
            *resBase = matchingNode->base;
            *resLen = matchingNode->length;
        } else {
            nodeIsLocalized = RM_FALSE;
        }
    }

exitLocalization:
    return(nodeIsLocalized);
}

/* FUNCTION PURPOSE: Issues an allocator operation
 ***********************************************************************
 * DESCRIPTION: Issues an allocator preallocate, allocate, or free
 *              for an RM resource.
 */
int32_t rmAllocatorOperation(Rm_Handle rmHandle, Rm_AllocatorOpInfo *opInfo)
{
    Rm_Inst          *rmInst = (Rm_Inst *)rmHandle;
    Rm_AllocatorNode *allocator = NULL;
    int32_t           retVal;

    if ((allocator = rmAllocatorFind(rmHandle, opInfo->resourceInfo->name))) {
        if (rmInst->instType == Rm_instType_SHARED_SERVER) {
            rmResourceTreeInv(allocator->resourceRoot);
        }

        if (opInfo->operation == Rm_allocatorOp_GET_STATUS) {
            retVal = allocatorStatus(rmHandle, allocator, opInfo);
        } else if ((opInfo->operation == Rm_allocatorOp_PRE_ALLOCATE_INIT) ||
                   (opInfo->operation == Rm_allocatorOp_PRE_ALLOCATE_USE)) {
            retVal = allocatorPreAllocate(rmHandle, allocator, opInfo);
        } else if ((opInfo->operation == Rm_allocatorOp_ALLOCATE_INIT) ||
                   (opInfo->operation == Rm_allocatorOp_ALLOCATE_USE)) {
            retVal = allocatorAllocate(rmHandle, allocator, opInfo);
        } else if (opInfo->operation == Rm_allocatorOp_FREE) {
            retVal = allocatorFree(rmHandle, allocator, opInfo);
        } else {
            retVal = RM_ERROR_INVALID_SERVICE_TYPE;
        }

        if ((rmInst->instType == Rm_instType_SHARED_SERVER) &&
            (opInfo->operation != Rm_allocatorOp_GET_STATUS) &&
            (retVal == RM_SERVICE_APPROVED)) {
            rmResourceTreeWb(allocator->resourceRoot);
        }
    } else {
        /* Resource could not be found in policy and/or allocator */
        retVal = RM_SERVICE_DENIED_RES_DOES_NOT_EXIST;
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Initializes the allocator tree
 ***********************************************************************
 * DESCRIPTION: Initializes a RM instance's allocator tree using the
 *              supplied GRL and policy
 */
int32_t rmAllocatorTreeInit(Rm_Handle rmHandle, void *grlDtb,
                            void *policyDtb, void *linuxDtb)
{
    Rm_Inst *rmInst = (Rm_Inst *)rmHandle;
    int32_t  retVal = RM_OK;

    if ((retVal = allocatorTreeRootInit(rmInst)) != RM_OK) {
        goto errorExit;
    }

    if (grlDtb && policyDtb &&
        ((rmInst->instType == Rm_instType_SERVER) ||
         (rmInst->instType == Rm_instType_SHARED_SERVER))) {
        /* Create an allocator for each resource node in GRL.  Companion
         * policy info will be pulled and placed into policy tree */
        retVal = allocatorPopulateGrlBased(rmInst, grlDtb, policyDtb, linuxDtb);
    } else if (policyDtb &&
               ((rmInst->instType == Rm_instType_CLIENT_DELEGATE) ||
                (rmInst->instType == Rm_instType_CLIENT))) {
        /* Create an allocator for each resource node in the policy.
         * Resource tree portion of the allocator will be NULL.  Resource
         * trees will be added at run time for Client Delegate instances.
         * Client instances with static policy just need allocators with
         * policy information for each resource. */
        retVal = allocatorPopulatePolicyBased(rmInst, policyDtb);
    } else if ((rmInst->instType != Rm_instType_CLIENT) &&
               (rmInst->instType != Rm_instType_SHARED_CLIENT)) {
        retVal = RM_ERROR_INVALID_ALLOCATOR_INIT;
    } else {
        retVal = RM_ERROR_INVALID_INST_TYPE;
    }

errorExit:
    return(retVal);
}

/* FUNCTION PURPOSE: Adds a node to a resource tree
 ***********************************************************************
 * DESCRIPTION: Adds a node to an allocator's resource tree based on the
 *              given base and length.
 */
int32_t rmAllocatorAddResNode(Rm_Handle rmHandle, Rm_AllocatorNode *allocator,
                              int32_t resBase, uint32_t resLen)
{
    Rm_Inst         *rmInst = (Rm_Inst *)rmHandle;
    Rm_ResourceNode *resNode = NULL;
    int32_t retVal = RM_OK;

    if (rmInst->instType == Rm_instType_SHARED_SERVER) {
        rmResourceTreeInv(allocator->resourceRoot);
    }

    if ((resNode = rmResourceNodeNew(resBase, resLen))) {
        if (RB_INSERT(_Rm_AllocatorResourceTree, allocator->resourceRoot,
                      resNode)) {
            retVal = RM_ERROR_RES_SPECIFIED_MORE_THAN_ONCE;
        }
    } else {
        retVal = RM_ERROR_MALLOC_FAILED_RES_NODE;
    }

    return(retVal);
}

/* FUNCTION PURPOSE: Deletes a node from a resource tree
 ***********************************************************************
 * DESCRIPTION: Deletes a node from an allocator's resource tree based on the
 *              given base and length.
 */
void rmAllocatorDeleteResNode(Rm_Handle rmHandle, Rm_AllocatorNode *allocator,
                              int32_t resBase, uint32_t resLen)
{
    Rm_Inst         *rmInst = (Rm_Inst *)rmHandle;
    Rm_ResourceNode  find;
    Rm_ResourceNode *match;

    memset((void *)&find, 0, sizeof(find));
    find.base = resBase;
    find.length = resLen;
    match = RB_FIND(_Rm_AllocatorResourceTree, allocator->resourceRoot,
                    &find);
    
    if (match) {
        RB_REMOVE(_Rm_AllocatorResourceTree, allocator->resourceRoot,
                  match);
        rmResourceNodeFree(match);
        if (rmInst->instType == Rm_instType_SHARED_SERVER) {
            rmResourceTreeWb(allocator->resourceRoot);
        }
    }
}

/* FUNCTION PURPOSE: Deletes allocator tree
 ***********************************************************************
 * DESCRIPTION: Removes all resource nodes for each allocator node and then
 *              deletes the allocator tree root.
 */
void rmAllocatorTreeDelete(Rm_Handle rmHandle)
{
    Rm_Inst          *rmInst = (Rm_Inst *)rmHandle;
    Rm_AllocatorTree *allocTree = rmInst->allocatorTree;
    Rm_AllocatorNode *allocNode;
    Rm_AllocatorNode *nextAllocNode;
    Rm_ResourceTree  *resTree;
    Rm_ResourceNode  *resNode;
    Rm_ResourceNode  *nextResNode;
    Rm_PolicyTree    *polTree;
    Rm_PolicyNode    *polNode;
    Rm_PolicyNode    *nextPolNode;

    if (allocTree) {
        if (rmInst->instType == Rm_instType_SHARED_SERVER) {
            rmAllocatorTreeInv(allocTree);
        }

        for (allocNode = RB_MIN(_Rm_AllocatorTree, allocTree);
             allocNode != NULL;
             allocNode = nextAllocNode) {
            nextAllocNode = RB_NEXT(_Rm_AllocatorTree, allocTree, allocNode);

            resTree = allocNode->resourceRoot;
            polTree = allocNode->policyRoot;

            if (rmInst->instType == Rm_instType_SHARED_SERVER) {
                rmResourceTreeInv(resTree);
                rmPolicyTreeInv(polTree);
            }

            /* Delete each node in the resource tree */
            for (resNode = RB_MIN(_Rm_AllocatorResourceTree, resTree);
                 resNode != NULL;
                 resNode = nextResNode) {
                nextResNode = RB_NEXT(_Rm_AllocatorResourceTree, resTree,
                                      resNode);
                RB_REMOVE(_Rm_AllocatorResourceTree, resTree, resNode);
                if (resNode->allocationCount) {
                    /* Delete all the owners in the resource's owner list */
                    allocatorResNodeOwnerClear(rmHandle, resNode);
                }
                rmResourceNodeFree(resNode);
            }
            Rm_osalFree((void *)resTree, sizeof(*resTree));

            /* Delete each node in the policy tree */
            for (polNode = RB_MIN(_Rm_AllocatorPolicyTree, polTree);
                 polNode != NULL;
                 polNode = nextPolNode) {
                nextPolNode = RB_NEXT(_Rm_AllocatorPolicyTree, polTree,
                                      polNode);
                RB_REMOVE(_Rm_AllocatorPolicyTree, polTree, polNode);
                rmPolicyNodeFree(polNode);
            }
            Rm_osalFree((void *)polTree, sizeof(*polTree));

            RB_REMOVE(_Rm_AllocatorTree, allocTree, allocNode);
            rmAllocatorNodeFree(allocNode);
        }
        Rm_osalFree((void *)allocTree, sizeof(*allocTree));
        RM_SS_OBJ_WB(rmInst, rmInst, Rm_Inst);
    }
}
