/*
 *  file  rm_treeloc.h
 *
 *  Prototypes and data structures for the various RM Trees.
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

#ifndef RM_TREELOC_H_
#define RM_TREELOC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes */
#include <stdint.h>

/* RM external includes */
#include <ti/drv/rm/rm.h>

/* Tree algorithm includes */
#include <ti/drv/rm/util/tree.h>

/**********************************************************************
 *************** Tree Node Data Structure Definitions *****************
 **********************************************************************/

/* NameServer node configuration structure */
typedef struct {
    /* Name to assign to the resource values */
    char     *objName;
    /* Resource name assigned to the NameServer name */
    char     *resourceName;
    /* Resource base value assigned to the NameServer name */
    uint32_t  resourceBase;
    /* Resource length value (starting from base) assigned to the NameServer
     * name */
    uint32_t  resourceLength;
} Rm_NameServerNodeCfg;

/* NameServer node */
typedef struct _Rm_NameServerNode {
    /* Tree algorithm data structure */
    RB_ENTRY(_Rm_NameServerNode) linkage;
    /* Name string */
    char                         objName[RM_NAME_MAX_CHARS];
    /* Resource name */
    char                         resourceName[RM_NAME_MAX_CHARS];
    /* Resource base value */
    uint32_t                     resourceBase;
    /* Resource length value */
    uint32_t                     resourceLength;
} Rm_NameServerNode;

/* NameServer tree root entry type definition */
typedef RB_HEAD(_Rm_NameServerTree, _Rm_NameServerNode) Rm_NameServerTree;

/* Valid instance node */
typedef struct _Rm_PolicyValidInstNode {
    /* Tree algorithm data structure */
    RB_ENTRY(_Rm_PolicyValidInstNode) linkage;
    /* Valid instance name string */
    char                              name[RM_NAME_MAX_CHARS];
    /* Number of existing resource allocations the instance is
     * reference in.  Resource frees involving the instance
     * will decrement this count.  A valid instance node cannot
     * be deleted until this value is zero */
    uint32_t                          allocRefCount;
    /* TRUE: Delete this valid instance node once the allocRefCount
     *       reaches zero
     * FALSE: Do not delete */
    int32_t                           deletePending;
    /* Instance index used in policy permission assignments */
    int32_t                           instIdx;
} Rm_PolicyValidInstNode;

/* Valid instance tree root entry type definition */
typedef RB_HEAD(_Rm_PolicyValidInstTree,
                _Rm_PolicyValidInstNode) Rm_PolicyValidInstTree;

/* Resource node owner linked list node */
typedef struct Rm_Owner_s {
    /* Pointer to the valid instance node that currently
     * owns or partially owns the resource */
    Rm_PolicyValidInstNode *instNameNode;
    /* Number of times this owner has allocated the resource it is
     * linked to. */
    uint16_t                refCnt;
    /* Link to the next owner of the resoruce if the resource is shared */
    struct Rm_Owner_s      *nextOwner;
} Rm_Owner;

/* Resource node */
typedef struct _Rm_ResourceNode {
    /* Tree algorithm data structure */
    RB_ENTRY(_Rm_ResourceNode) linkage;
    /* Resource base value */
    uint32_t                   base;
    /* Resource length value.  With base this node covers a resource's values
     * from base to base+length-1 */
    uint32_t                   length;
    /* Number of times this resource node has been allocated to a valid
     * instance.  This value will decrement for each free operation */
    uint16_t                   allocationCount;
    /* Linked list of existing owners.  Will be NULL if no owners exist
     * for the resource node */
    Rm_Owner                   *ownerList;
} Rm_ResourceNode;

/* Resource tree root entry type definition */
typedef RB_HEAD(_Rm_AllocatorResourceTree, _Rm_ResourceNode) Rm_ResourceTree;

/* Policy permission bit storage definition for Rm_PolicyPermBits
 * Bit : Description
 *-----------------------------
 *  0  : Init         (i) - RM instance has initialization permission for
 *                          resource
 *  1  : Use          (u) - RM instance has usage permission for resource
 *  2  : Exclusive    (x) - RM instance has exclusive allocation privilege for
 *                          resource i.e. No other RM instance can reserve the
 *                          resource if a RM instance with exclusive privilege
 *                          reserves the resource
 *  3  : Shared Linux (s) - Resource has been reserved by the Linux kernel but
 *                          can be allocated by the specified RM instances
 */

/* Maximum number of bits used to represent all permissions.  Must be
 * changed as new permissions are added */
#define RM_POLICY_PERM_MAX_BITS                5

/* Initialization permission characters */
#define RM_POLICY_PERM_INIT_LOWER             'i'
#define RM_POLICY_PERM_INIT_UPPER             'I'
/* Initialization permission bit shift */
#define RM_POLICY_PERM_INIT_SHIFT              0
/* Usage permission characters */
#define RM_POLICY_PERM_USE_LOWER              'u'
#define RM_POLICY_PERM_USE_UPPER              'U'
/* Usage permission bit shift */
#define RM_POLICY_PERM_USE_SHIFT               1
/* Exclusive permission characters */
#define RM_POLICY_PERM_EXCLUSIVE_LOWER        'x'
#define RM_POLICY_PERM_EXCLUSIVE_UPPER        'X'
/* Exclusive permission bit shift */
#define RM_POLICY_PERM_EXCLUSIVE_SHIFT         2
/* Shared Linux permission characters */
#define RM_POLICY_PERM_SHARED_LINUX_LOWER     's'
#define RM_POLICY_PERM_SHARED_LINUX_UPPER     'S'
/* Shared Linux permission bit shift */
#define RM_POLICY_PERM_SHARED_LINUX_SHIFT      3
/* UNSPECIFIED allocation exclusion permission characters */
#define RM_POLICY_PERM_UNSPEC_EXCLUSION_LOWER 'e'
#define RM_POLICY_PERM_UNSPEC_EXCLUSION_UPPER 'E'
/* UNSPECIFIED allocation exclusion permission bit shift */
#define RM_POLICY_PERM_UNSPEC_EXCLUSION_SHIFT  4

/* Permissions subgroup start character */
#define RM_POLICY_PERM_SUBGROUP_START         '('
/* Permissions subgroup end character */
#define RM_POLICY_PERM_SUBGROUP_END           ')'
/* Permissions subgroup terminator */
#define RM_POLICY_PERM_TERMINATOR             '&'
/* Permissions assignment character */
#define RM_POLICY_PERM_ASSIGNMENT             '='

/* Policy permission bit storage type */
typedef uint32_t Rm_PolicyPermBits;

/* Policy node */
typedef struct _Rm_PolicyNode {
    /* Tree algorithm data structure */
    RB_ENTRY(_Rm_PolicyNode)  linkage;
    /* Policy base value */
    uint32_t                  base;
    /* Policy length value.  With base this node covers a policy's values
     * from base to base+length-1 */
    uint32_t                  len;
    /* Pointer to array containing permission bitfields for the node's 
     * resource range */
    Rm_PolicyPermBits        *perms;
    /* Size of permissions array in bytes pointed to by the perms pointer */
    uint32_t                  permsLen;
    /* Allocation alignment.  Allocations must be aligned to a base that is a
     * multiple of the allocation alignment */
    uint32_t                  allocAlign;
    /* Client Delegate allocation block size */
    uint32_t                  cdAllocSize;
} Rm_PolicyNode;

/* Policy tree root entry type definition */
typedef RB_HEAD(_Rm_AllocatorPolicyTree, _Rm_PolicyNode) Rm_PolicyTree;

/* Allocator node */
typedef struct _Rm_AllocatorNode {
    /* Tree algorithm data structure */
    RB_ENTRY(_Rm_AllocatorNode)  linkage;
    /* Resource name for which the allocator was created.  The resource name
     * must match a resource node defined in both the GRL and the Policy */
    char                         resourceName[RM_NAME_MAX_CHARS];
    /* Pointer to root entry of allocator's resource tree */
    Rm_ResourceTree             *resourceRoot;
    /* Pointer to root entry of allocator's policy tree */
    Rm_PolicyTree               *policyRoot;
} Rm_AllocatorNode;

/* Allocator tree root entry type definition */
typedef RB_HEAD(_Rm_AllocatorTree, _Rm_AllocatorNode) Rm_AllocatorTree;

/**********************************************************************
 ****************** Tree Node Function Definitions ********************
 **********************************************************************/

void rmNameServerTreeInv(Rm_NameServerTree *treeRoot);
void rmNameServerTreeWb(Rm_NameServerTree *treeRoot);
Rm_NameServerNode *rmNameServerNodeNew(Rm_NameServerNodeCfg *nodeCfg);
void rmNameServerNodeFree(Rm_NameServerNode *node);
int rmNameServerNodeCompare(Rm_NameServerNode *node1,
                            Rm_NameServerNode *node2);
void rmNameServerNodeInv(Rm_NameServerNode *node);

void rmPolicyValidInstTreeInv(Rm_PolicyValidInstTree *treeRoot);
void rmPolicyValidInstTreeWb(Rm_PolicyValidInstTree *treeRoot);
Rm_PolicyValidInstNode *rmPolicyValidInstNodeNew(const char *instName,
                                                 const int32_t instIdx);
void rmPolicyValidInstNodeFree(Rm_PolicyValidInstNode *node);
int rmPolicyValidInstNodeCompare(Rm_PolicyValidInstNode *node1,
                                 Rm_PolicyValidInstNode *node2);
void rmPolicyValidInstNodeInv(Rm_PolicyValidInstNode *node);

void rmResourceTreeInv(Rm_ResourceTree *treeRoot);
void rmResourceTreeWb(Rm_ResourceTree *treeRoot);
Rm_ResourceNode *rmResourceNodeNew(uint32_t resourceBase,
                                   uint32_t resourceLength);
void rmResourceNodeFree(Rm_ResourceNode *node);
int rmResourceNodeCompare(Rm_ResourceNode *node1, Rm_ResourceNode *node2);
void rmResourceNodeInv(Rm_ResourceNode *node);

void rmPolicyTreeInv(Rm_PolicyTree *treeRoot);
void rmPolicyTreeWb(Rm_PolicyTree *treeRoot);
Rm_PolicyNode *rmPolicyNodeNew(uint32_t resourceBase, uint32_t resourceLength);
void rmPolicyNodeFree(Rm_PolicyNode *node);
int rmPolicyNodeCompare(Rm_PolicyNode *node1, Rm_PolicyNode *node2);
void rmPolicyNodeInv(Rm_PolicyNode *node);

void rmAllocatorTreeInv(Rm_AllocatorTree *treeRoot);
void rmAllocatorTreeWb(Rm_AllocatorTree *treeRoot);
Rm_AllocatorNode *rmAllocatorNodeNew(const char *resourceName);
void rmAllocatorNodeFree(Rm_AllocatorNode *node);
int rmAllocatorNodeCompare(Rm_AllocatorNode *node1, Rm_AllocatorNode *node2);
void rmAllocatorNodeInv(Rm_AllocatorNode *node);

/**********************************************************************
 ******************** Tree Prototype Generation ***********************
 **********************************************************************/
 
RB_PROTOTYPE(_Rm_NameServerTree, _Rm_NameServerNode, linkage,
             rmNameServerNodeCompare, rmNameServerNodeInv)
RB_PROTOTYPE(_Rm_PolicyValidInstTree, _Rm_PolicyValidInstNode, linkage,
             rmPolicyValidInstNodeCompare, rmPolicyValidInstNodeInv)
RB_PROTOTYPE(_Rm_AllocatorResourceTree, _Rm_ResourceNode, linkage,
             rmResourceNodeCompare, rmResourceNodeInv)
RB_PROTOTYPE(_Rm_AllocatorPolicyTree, _Rm_PolicyNode, linkage,
             rmPolicyNodeCompare, rmPolicyNodeInv)
RB_PROTOTYPE(_Rm_AllocatorTree, _Rm_AllocatorNode, linkage,
             rmAllocatorNodeCompare, rmAllocatorNodeInv)

#ifdef __cplusplus
}
#endif

#endif /* RM_TREELOC_H_ */

