/*
 *  file  rm_policyloc.h
 *
 *  Internal prototypes and data structures for the Resource Manager Policies.
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

#ifndef RM_POLICYLOC_H_
#define RM_POLICYLOC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* RM external includes */
#include <ti/drv/rm/rm.h>

/* RM internal includes */
#include <ti/drv/rm/include/rm_treeloc.h>

/* Index of global permission bits in policy tree node permission array */
#define RM_POLICY_GLOBAL_PERM_INDEX        0

/* String stored for resource elements that are reserved by the Linux kernel.
 * These resources will be in use for the lifetime of the system
 *
 * TODO: MOVE TO policy.h LATER SO INTEGRATORS KNOW NOT TO NAME RM INSTANCES
 * THIS NAME */
#define RM_ALLOCATED_TO_LINUX "Linux-Kernel"

/* Full permissions bit mask */
#define RM_policy_PERM_FULL_MASK          ((1u << RM_POLICY_PERM_MAX_BITS) - 1)

/* Calculates number of instance permission sets per permission word */
#define RM_policy_PERM_INST_PER_WORD \
    ((sizeof(Rm_PolicyPermBits) * 8) / RM_POLICY_PERM_MAX_BITS)
/* Calculates the instance's word index into the permission word array */
#define RM_policy_PERM_INDEX(instIdx) \
    (instIdx / RM_policy_PERM_INST_PER_WORD)
/* Calculates the instance's bit offset into the permission word */
#define RM_policy_PERM_OFFSET(instIdx)          \
    ((instIdx % RM_policy_PERM_INST_PER_WORD) * \
     RM_POLICY_PERM_MAX_BITS)

/* Sets a permission for an instance */
#define RM_policy_PERM_SET(permArray, wordIdx, wordOffset, permOffset, val) \
    permArray[wordIdx] &= ~(0x1u << (wordOffset + permOffset));             \
    permArray[wordIdx] |= (val & 0x1u) << (wordOffset + permOffset);
/* Gets a permission for an instance */
#define RM_policy_PERM_GET(permArray, wordIdx, wordOffset, permOffset)  \
    ((permArray[wordIdx] >> (wordOffset + permOffset)) & 0x1u)

/* Assigned instance permissions linked list node */
typedef struct Rm_Permission_s {
    /* Instance name of instance assigned the permissions */
    char                    instName[RM_NAME_MAX_CHARS];
    /* Permissions assigned to the RM instance with instName */
    Rm_PolicyPermBits       permissionBits;
    /* Next permissions node */
    struct Rm_Permission_s *nextPermission;
} Rm_PolicyPermission;

/* Permission validation types */
typedef enum {
    /* Validate exclusive permissions for a resource */
    Rm_policyCheck_EXCLUSIVE = 0,
    /* Validate initialization permissions for a resource */
    Rm_policyCheck_INIT,
    /* Validate usage permissions for a resource */
    Rm_policyCheck_USE,
    /* Validate shared Linux permissions for a resource */
    Rm_policyCheck_SHARED_LINUX,
    /* Validate shared UNSPECIFIED allocation exclusion for a resource */
    Rm_policyCheck_UNSPEC_EXCLUSION
} Rm_PolicyCheckType;

/* Permissions validation configuration structure */
typedef struct {
    /* The type of validation to perform */
    Rm_PolicyCheckType      type;
    /* Negative check:
     * RM_FALSE (0) - Check if passes policy
     * RM_TRUE  (1) - Check if fails policy */
    uint32_t                negCheck;
    /* Resource's policy tree used to validate permissions */
    Rm_PolicyTree          *polTree;
    /* Valid instance node having its permissions checked for the resource */
    Rm_PolicyValidInstNode *validInstNode;
    /* Resource base to validate permissions for the valid instance node */
    uint32_t                resourceBase;
    /* Resource length to validate permissions for the valid instance node */
    uint32_t                resourceLength;
} Rm_PolicyCheckCfg;

Rm_PolicyValidInstNode *rmPolicyGetValidInstNode(Rm_Handle rmHandle,
                                                 const char *instName);
Rm_PolicyValidInstNode *rmPolicyGetLinuxInstNode(Rm_Handle rmHandle);
int32_t rmPolicyCheckPrivilege(Rm_PolicyCheckCfg *privilegeCfg);
int32_t rmPolicyGetResourceBase(Rm_PolicyTree *policyTree,
                                 Rm_PolicyValidInstNode *validInstNode,
                                 Rm_PolicyCheckType policyCheckType,
                                 int32_t *resBase);
uint32_t rmPolicyGetAllocAlign(Rm_PolicyTree *policyTree);
uint32_t rmPolicyGetCdAllocSize(Rm_PolicyTree *policyTree);
Rm_PolicyValidInstTree *rmPolicyVInstTreeInit(Rm_Handle rmHandle,
                                              void *policyDtb,
                                              int addLinux,
                                              int32_t *result);
void rmPolicyVInstTreeDelete(Rm_Handle rmHandle);
int32_t rmPolicyPopulateTree(Rm_Handle rmHandle, Rm_PolicyTree *policyTree,
                             void *policyDtb, const char *resName);

#ifdef __cplusplus
}
#endif

#endif /* RM_POLICYLOC_H_ */

