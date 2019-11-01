/**
 *   @file  rm_policy.c
 *
 *   @brief   
 *      Resource Manager Policy source.
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

/* Standard includes */
#include <ctype.h>

/* RM external API includes */
#include <ti/drv/rm/rm.h>

/* RM internal API includes */
#include <ti/drv/rm/include/rm_internal.h>
#include <ti/drv/rm/include/rm_loc.h>
#include <ti/drv/rm/include/rm_allocatorloc.h>
#include <ti/drv/rm/include/rm_policyloc.h>
#include <ti/drv/rm/include/rm_dtb_utilloc.h>

/* RM LIBFDT includes */
#include <ti/drv/rm/util/libfdt/libfdt.h>

/* Tree algorithm includes */
#include <ti/drv/rm/util/tree.h>

/* RM OSAL layer */
#include <rm_osal.h>

/**********************************************************************
 *********************** Policy Globals *******************************
 **********************************************************************/

/* Character used in policies to specify all RM instances receive
 * the defined permissions for a resource node */
const char Rm_policyGlobalInst[] = "*";

/**********************************************************************
 ******************** Local Policy Functions **************************
 **********************************************************************/

/* FUNCTION PURPOSE: Parses a permissions subgroup
 ***********************************************************************
 * DESCRIPTION: Returns a linked list of policy permissions defining
 *              which RM instance referenced in the permissions subgroup
 *              get which permissions.  Returns NULL if any syntax
 *              errors are encountered during the parsing.  The error
 *              is returned via the result pointer parameter.
 */
static Rm_PolicyPermission *policyParseSubPermission(char *permStrStart,
                                                     char *permStrEnd,
                                                     int32_t *result)
{
    Rm_PolicyPermission *startPerm = NULL;
    Rm_PolicyPermission *newPerm = NULL;
    Rm_PolicyPermission *prevPerm = NULL;
    Rm_PolicyPermission *nextPerm = NULL;
    char                *permStrPtr = NULL;
    char                *subgroupStart = NULL;
    char                *subgroupEnd = NULL;
    uint32_t             permStrLen = (uint32_t)(permStrEnd - permStrStart + 1);
    char                 instNameTemp[RM_NAME_MAX_CHARS];
    uint32_t             instNameIndex;
    int                  foundInstName;
    int                  instNameComplete;
    int                  assignmentLeft;
    int                  assignmentRight;

    /* Create a local copy of the sub-permission string */
    permStrPtr = Rm_osalMalloc(permStrLen);
    rm_strncpy(permStrPtr, permStrStart, permStrLen);

    permStrStart = permStrPtr;
    permStrEnd = permStrPtr + strlen(permStrPtr);

    /* Find the beginning and end of the sub-permission instance group */
    subgroupStart = strchr(permStrStart, RM_POLICY_PERM_SUBGROUP_START);
    subgroupEnd = strchr(permStrStart, RM_POLICY_PERM_SUBGROUP_END);

    if ((!subgroupStart) || (!subgroupEnd) || (subgroupStart > subgroupEnd) ||
        ((subgroupStart != strrchr(permStrStart,
                                   RM_POLICY_PERM_SUBGROUP_START)) &&
         (subgroupEnd != strrchr(permStrStart,
                                 RM_POLICY_PERM_SUBGROUP_END)))) {
        /* Free the memory associated with the temp string and return an
         * error if:
         * a) Could not find the instance group start
         * b) Could not find the instance group end
         * c) Subgroup start and end are out of order
         * d) There is more than one instance subgroup specified in the string.
         *    There should only be one subgroup per sub-permission string */
        *result = RM_ERROR_PERM_STR_TOO_MANY_INST_GROUPS;
        goto parseError;
    }

    /* Create a permission entry for each instance specified in the instance group.
     * Instances names are separated by one or more spaces. */
    permStrPtr = subgroupStart + 1;
    instNameIndex = 0;
    foundInstName = RM_FALSE;
    instNameComplete = RM_FALSE;
    while (permStrPtr <= subgroupEnd) {
        if ((isspace(*permStrPtr) ||
            (*permStrPtr == RM_POLICY_PERM_SUBGROUP_END))
            && foundInstName) {
            /* First space encountered after copying an instance name.  This
             * terminates the instance name.  All other space characters are
             * ignored. */
            instNameTemp[instNameIndex] = '\0';
            instNameComplete = RM_TRUE; 
        } else {
            if (!foundInstName) {
                /* First non-whitespace character encountered is the start of an
                 * instance name */
                foundInstName = RM_TRUE;
            }

            /* Copy the character into the temporary instance name string */
            instNameTemp[instNameIndex++] = *permStrPtr;
        }

        if (instNameComplete) {
            newPerm = Rm_osalMalloc(sizeof(*newPerm));
            memset((void *)newPerm, 0, sizeof(*newPerm));

            rm_strncpy(newPerm->instName, instNameTemp, RM_NAME_MAX_CHARS);
            newPerm->nextPermission = NULL;

            if (prevPerm == NULL) {
                /* Save the first instance so it can be returned */
                startPerm = newPerm;
            } else {
                prevPerm->nextPermission = newPerm;
            }
            prevPerm = newPerm;

            instNameComplete = RM_FALSE;
            instNameIndex = 0;
            foundInstName = RM_FALSE;
        }

        if (instNameIndex == RM_NAME_MAX_CHARS) {
            /* Instance name is longer than max length */
            *result = RM_ERROR_INST_NAME_IN_ASSIGNMENT_TOO_LONG;
            goto parseError;
        }

        permStrPtr++;
    }

    /* Fill in the permissions for each instance name */

    /* Look on left of instance group for permission assignments. */
    permStrPtr = subgroupStart - 1;
    assignmentLeft = RM_FALSE;
    while (permStrPtr >= permStrStart)
    {
        if (*permStrPtr == RM_POLICY_PERM_ASSIGNMENT) {
            if (assignmentLeft) {
                /* Assignment character has been found more than once.  This
                 * is a syntax error.  Free the permission list and the
                 * temporary string and return. */
                *result = RM_ERROR_PERM_STR_TOO_MANY_ASSIGN_CHARS;
                goto parseError;
            } else {
                assignmentLeft = RM_TRUE;
            }
        } else if (!isspace(*permStrPtr)) {
            if (assignmentLeft) {
                if ((*permStrPtr == RM_POLICY_PERM_INIT_LOWER) ||
                    (*permStrPtr == RM_POLICY_PERM_INIT_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                           RM_POLICY_PERM_INIT_SHIFT, 1);
                        newPerm = newPerm->nextPermission;
                    }
                } else if ((*permStrPtr == RM_POLICY_PERM_USE_LOWER) ||
                         (*permStrPtr == RM_POLICY_PERM_USE_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                           RM_POLICY_PERM_USE_SHIFT, 1);
                        newPerm = newPerm->nextPermission;
                    }
                } else if ((*permStrPtr == RM_POLICY_PERM_EXCLUSIVE_LOWER) ||
                         (*permStrPtr == RM_POLICY_PERM_EXCLUSIVE_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                           RM_POLICY_PERM_EXCLUSIVE_SHIFT, 1);
                        newPerm = newPerm->nextPermission;
                    }
                } else if ((*permStrPtr == RM_POLICY_PERM_SHARED_LINUX_LOWER) ||
                         (*permStrPtr == RM_POLICY_PERM_SHARED_LINUX_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                           RM_POLICY_PERM_SHARED_LINUX_SHIFT,
                                           1);
                        newPerm = newPerm->nextPermission;
                    }
                } else if ((*permStrPtr ==
                            RM_POLICY_PERM_UNSPEC_EXCLUSION_LOWER) ||
                           (*permStrPtr ==
                            RM_POLICY_PERM_UNSPEC_EXCLUSION_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                      RM_POLICY_PERM_UNSPEC_EXCLUSION_SHIFT, 1);
                        newPerm = newPerm->nextPermission;
                    }
                } else {
                    /* Invalid permission character.  This is a syntax error.
                     * Free the permission list and the temporary string
                     * and return. */
                    *result = RM_ERROR_PERM_STR_INVALID_CHAR;
                    goto parseError;
                }
            } else {
                /* Character found without the assignment character being found.
                 * This is a syntax error.  Free the permission list and the
                 * temporary string and return. */
                *result = RM_ERROR_PERM_CHAR_WITHOUT_ASSIGN_CHAR;
                goto parseError;
            }
        }
        permStrPtr--;
    }

    /* Look on right of instance group for permission assignments. */
    permStrPtr = subgroupEnd + 1;
    assignmentRight = RM_FALSE;
    while (permStrPtr < permStrEnd) {
        if (assignmentLeft && (!isspace(*permStrPtr))) {
            /* There should be nothing but spaces on right if assignment was
             * already found on left */
            *result = RM_ERROR_INVALID_PERMS_CHAR_ON_RIGHT;
            goto parseError;
        }

        if (*permStrPtr == RM_POLICY_PERM_ASSIGNMENT) {
            if (assignmentRight) {
                /* Assignment character has been found more than once.  This is
                 * a syntax error.  Free the permission list and the temporary
                 * string and return. */
                *result = RM_ERROR_PERM_STR_TOO_MANY_ASSIGN_CHARS;
                goto parseError;
            } else {
                assignmentRight = RM_TRUE;
            }
        } else if (!isspace(*permStrPtr)) {
            if (assignmentRight) {
                if ((*permStrPtr == RM_POLICY_PERM_INIT_LOWER) ||
                    (*permStrPtr == RM_POLICY_PERM_INIT_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                           RM_POLICY_PERM_INIT_SHIFT, 1);
                        newPerm = newPerm->nextPermission;
                    }
                } else if ((*permStrPtr == RM_POLICY_PERM_USE_LOWER) ||
                         (*permStrPtr == RM_POLICY_PERM_USE_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                           RM_POLICY_PERM_USE_SHIFT, 1);
                        newPerm = newPerm->nextPermission;
                    }
                } else if ((*permStrPtr == RM_POLICY_PERM_EXCLUSIVE_LOWER) ||
                         (*permStrPtr == RM_POLICY_PERM_EXCLUSIVE_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                           RM_POLICY_PERM_EXCLUSIVE_SHIFT, 1);
                        newPerm = newPerm->nextPermission;
                    }
                } else if ((*permStrPtr == RM_POLICY_PERM_SHARED_LINUX_LOWER) ||
                         (*permStrPtr == RM_POLICY_PERM_SHARED_LINUX_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                           RM_POLICY_PERM_SHARED_LINUX_SHIFT,
                                           1);
                        newPerm = newPerm->nextPermission;
                    }
                } else if ((*permStrPtr ==
                            RM_POLICY_PERM_UNSPEC_EXCLUSION_LOWER) ||
                           (*permStrPtr ==
                            RM_POLICY_PERM_UNSPEC_EXCLUSION_UPPER)) {
                    newPerm = startPerm;
                    while (newPerm) {
                        Rm_PolicyPermBits *pBits = &(newPerm->permissionBits);
                        RM_policy_PERM_SET(pBits, 0, 0,
                                      RM_POLICY_PERM_UNSPEC_EXCLUSION_SHIFT, 1);
                        newPerm = newPerm->nextPermission;
                    }
                } else {
                    /* Invalid permission character.  This is a syntax error.
                     * Free the permission list and the temporary string
                     * and return. */
                    *result = RM_ERROR_PERM_STR_INVALID_CHAR;
                    goto parseError;
                }
            } else {
                /* Character found without the assignment character being found.
                 * This is a syntax error.  Free the permission list and the
                 * temporary string and return. */
                *result = RM_ERROR_PERM_STR_TOO_MANY_ASSIGN_CHARS;
                goto parseError;
            }
        }
        permStrPtr++;
    }

    Rm_osalFree((void *)permStrStart, permStrLen);
    *result = RM_OK;
    return (startPerm);

parseError:
    while (startPerm) {
        nextPerm = startPerm->nextPermission;
        Rm_osalFree((void *)startPerm, sizeof(Rm_PolicyPermission));
        startPerm = nextPerm;
    }
    Rm_osalFree((void *)permStrStart, permStrLen);
    return(NULL);
}

/* FUNCTION PURPOSE: Frees a linked list of assignment permissions
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a linked list of
 *              assignment permissions extracted from a permissions
 *              assignment subgroup in a policy DTB.
 */
static void policyFreeAssignmentPermissions(Rm_PolicyPermission *permissionList)
{
    Rm_PolicyPermission *nextPerm;

    while (permissionList) {
        nextPerm = permissionList->nextPermission;
        Rm_osalFree((void *)permissionList, sizeof(Rm_PolicyPermission));
        permissionList = nextPerm;
    }
}

/* FUNCTION PURPOSE: Extracts permissions from a Policy "assignment"
 ***********************************************************************
 * DESCRIPTION: Returns a linked list of permissions for a resource node
 *              containing an "assignment" property in the Policy DTB.
 *              Each node in the linked list will contain a valid instance
 *              name along with the permissions assigned to the instance
 */
static Rm_PolicyPermission *policyGetAssignmentPermissions(Rm_PolicyAssignment *assignment,
                                                           int32_t *result)
{
    Rm_PolicyPermission *startPerm = NULL;
    Rm_PolicyPermission *newPerm = NULL;
    Rm_PolicyPermission *prevPerm = NULL;
    char                *permStrStart = assignment->permissionsList;
    char                *permStrEnd;
    uint32_t             permStrLen = strlen(assignment->permissionsList) + 1;
    uint32_t             i = 0;

    *result = RM_OK;

    while(i < permStrLen) {
        /* Find the first sub-permission specification and parse it.  A
         * sub-permission can be terminated by the termination character or the
         * end of the string. */
        if (!(permStrEnd = strchr(permStrStart, RM_POLICY_PERM_TERMINATOR))) {
            /* Sub-permission termination character not found.  The permission
             * string end is the end of the entire permission string */
            permStrEnd = permStrStart + strlen(permStrStart);
        }

        newPerm = policyParseSubPermission(permStrStart, permStrEnd, result);

        if (*result != RM_OK) {
            /* Delete the permission list that's been created thus far, return
             * the error and NULL for the permission list */
            policyFreeAssignmentPermissions(startPerm);
            return(NULL);
        }

        if (prevPerm == NULL) {
            startPerm = newPerm;
        } else {
            prevPerm->nextPermission = newPerm;
        }

        /* Set prevPerm to the last sub-permission returned by the
         * sub-permission parser */
        prevPerm = newPerm;
        while(prevPerm->nextPermission != NULL) {
            prevPerm = prevPerm->nextPermission;
        }

        /* Update the number of characters parsed from the permission list and
         * point to the start of the next sub-permission */
        i += ((uint32_t)(permStrEnd - permStrStart + 1));
        permStrStart = permStrEnd + 1;
    }

    return(startPerm);
}

/* FUNCTION PURPOSE: Returns number of bytes needed to store permissions
 ***********************************************************************
 * DESCRIPTION: Calculates and returns the number of bytes needed to store
 *              permissions for all valid instances.  Words are packed
 *              with permissions for as many instances as can fit
 */
static uint32_t policyGetPermBufSize(uint32_t maxValidInst)
{
    uint32_t instPerWord;
    uint32_t numWords;
    uint32_t totalBytes = 0;

    instPerWord = RM_policy_PERM_INST_PER_WORD;
    /* Round up */
    numWords = (maxValidInst + instPerWord - 1) / instPerWord;

    totalBytes = sizeof(Rm_PolicyPermBits) * numWords;
    return(totalBytes);
}

/* FUNCTION PURPOSE: Stores policy permissions in a policy tree node
 ***********************************************************************
 * DESCRIPTION: Parses permissions and stores them in a word array in a
 *              compacted format.  The array is attached to the resource's
 *              policy tree node.
 */
static int32_t policyStorePermissions(Rm_Inst *rmInst, Rm_PolicyNode *polNode,
                                      Rm_PolicyPermission *extractedPerms)
{
    Rm_PolicyValidInstNode *vInst = NULL;
    uint32_t                instIdx;
    uint32_t                wordIndex;
    uint32_t                wordOffset;
    int32_t                 retVal = RM_OK;

    while (extractedPerms) {
        if (!strncmp(extractedPerms->instName, Rm_policyGlobalInst,
                     RM_NAME_MAX_CHARS)) {
            instIdx = RM_POLICY_GLOBAL_PERM_INDEX;
        } else if ((vInst = rmPolicyGetValidInstNode((Rm_Handle)rmInst,
                                                   extractedPerms->instName))) {
            instIdx = vInst->instIdx;
        } else {
            retVal = RM_ERROR_PERM_STR_INST_NOT_VALID;
            goto errorExit;
        }

        /* Calculate word index into policy node's permission array
         * for the instance index */
        wordIndex  = RM_policy_PERM_INDEX(instIdx);
        wordOffset = RM_policy_PERM_OFFSET(instIdx);

        polNode->perms[wordIndex] |= ((extractedPerms->permissionBits &
                                      RM_policy_PERM_FULL_MASK) << wordOffset);

        extractedPerms = extractedPerms->nextPermission;
    }

errorExit:
    return(retVal);
}

/* FUNCTION PURPOSE: Returns resource's allocation alignment from policy
 ***********************************************************************
 * DESCRIPTION: Parses the policy DTB to find and return a resource's 
 *              allocation alignment.
 */
static uint32_t policyGetDtbAllocAlign(void *policyDtb, int32_t resourceOffset)
{
    int32_t           offset;
    const char       *name;
    int32_t           len;
    const void       *data;
    Rm_ResourceValue *alignList = NULL;
    uint32_t          align = 0;

    offset = fdt_first_property_offset(policyDtb, resourceOffset);
    while (offset > RM_DTB_UTIL_STARTING_NODE_OFFSET) {
        data = fdt_getprop_by_offset(policyDtb, offset, &name, &len);
        if (rmDtbUtilPolicyGetPropertyType(name) ==
            Rm_policyPropType_ALLOCATION_ALIGNMENT) {
            alignList = rmDtbUtilPolicyExtractResourceAlignments(data, len);
            align = alignList->value;
            break;
        }
        offset = fdt_next_property_offset(policyDtb, offset);
    }

    if (alignList) {
        rmDtbUtilPolicyFreeResourceAlignments(alignList);
    }

    if (align == 0) {
        align = 1;
    }
    return(align);
}

/* FUNCTION PURPOSE: Returns resource CD allocation size defined in policy
 ***********************************************************************
 * DESCRIPTION: Parses the policy DTB to find and return a resource's 
 *              allocation size.
 */
static uint32_t policyGetDtbCdAllocSize(void *policyDtb, int32_t resourceOffset)
{
    int32_t           offset;
    const char       *name;
    int32_t           len;
    const void       *data;
    Rm_ResourceValue *allocSizeList = NULL;
    uint32_t          allocSize = 0;

    offset = fdt_first_property_offset(policyDtb, resourceOffset);
    while (offset > RM_DTB_UTIL_STARTING_NODE_OFFSET) {
        data = fdt_getprop_by_offset(policyDtb, offset, &name, &len);
        if (rmDtbUtilPolicyGetPropertyType(name) ==
            Rm_policyPropType_CD_ALLOCATION_SIZE) {
            allocSizeList = rmDtbUtilPolicyExtractCdAllocationSizes(data, len);
            allocSize = allocSizeList->value;
            break;
        }
        offset = fdt_next_property_offset(policyDtb, offset);
    }

    if (allocSizeList) {
        rmDtbUtilPolicyFreeCdAllocationSizes(allocSizeList);
    }
    return(allocSize);
}

/* FUNCTION PURPOSE: Get a resource's offset into a policy
 ***********************************************************************
 * DESCRIPTION: Returns the location of the specified resource node
 *              within the specified Policy in the form of an offset
 *              into the DTB.  The resourceName and the Policy
 *              node name must match.
 */
static int32_t policyGetDtbResourceOffset(void *policyDtb,
                                          const char *resourceName)
{
    int32_t     nodeOffset;
    int32_t     depth;
    const char *nodeName;

    if (policyDtb) {
        depth = RM_DTB_UTIL_STARTING_DEPTH;
        nodeOffset = RM_DTB_UTIL_STARTING_NODE_OFFSET;

        /* Find node offset for provided resource name */
        while (nodeOffset >= RM_DTB_UTIL_STARTING_NODE_OFFSET) {
            nodeOffset = fdt_next_node(policyDtb, nodeOffset, &depth);
            if (depth < RM_DTB_UTIL_STARTING_DEPTH) {
                /* Resource name not found */
                nodeOffset = RM_ERROR_RES_DOES_NOT_EXIST_IN_POLICY;
                break;
            } else {
                nodeName = fdt_get_name(policyDtb, nodeOffset, NULL);
                if (strncmp(nodeName, resourceName, RM_NAME_MAX_CHARS) == 0) {
                    break;
                }
            }
        }
    } else {
        nodeOffset = RM_ERROR_INSTANCE_HAS_NO_POLICY;
    }
    return(nodeOffset);
}

/**********************************************************************
 ************************ Internal Policy APIs ************************
 **********************************************************************/

/* FUNCTION PURPOSE: Get a valid instace node from the valid inst tree
 ***********************************************************************
 * DESCRIPTION: Returns a valid instance node from the valid instance
 *              tree that matches the specified instName
 */
Rm_PolicyValidInstNode *rmPolicyGetValidInstNode(Rm_Handle rmHandle,
                                                 const char *instName)
{
    Rm_Inst                *rmInst = (Rm_Inst *)rmHandle;
    Rm_PolicyValidInstTree *treeRoot = rmInst->validInstTree;
    Rm_PolicyValidInstNode  findNode;

    rm_strncpy(findNode.name, instName, RM_NAME_MAX_CHARS);
    return(RB_FIND(_Rm_PolicyValidInstTree, treeRoot, &findNode));
}

/* FUNCTION PURPOSE: Gets the Linux Valid instance node
 ***********************************************************************
 * DESCRIPTION: Returns a pointer to the valid instance node in the
 *              valid instance tree that matches the instance name
 *              reserved for resource assigned to the Linux kernel.
 */
Rm_PolicyValidInstNode *rmPolicyGetLinuxInstNode(Rm_Handle rmHandle)
{
    const char linuxName[] = RM_ALLOCATED_TO_LINUX;

    return(rmPolicyGetValidInstNode(rmHandle, linuxName));
}

/* FUNCTION PURPOSE: Validates resource permissions against a Policy DTB
 ***********************************************************************
 * DESCRIPTION: Returns TRUE if the instance name has the specified
 *              permissions for the specified resource in the Policy
 *              DTB.  Otherwise, returns FALSE.
 */
int32_t rmPolicyCheckPrivilege(Rm_PolicyCheckCfg *privilegeCfg)
{
    uint32_t       permShift;
    uint32_t       globPermIdx;
    uint32_t       globPermOffset;
    uint32_t       instPermIdx;
    uint32_t       instPermOffset;
    Rm_PolicyNode  findNode;
    Rm_PolicyNode *matchNode = NULL;
    uint32_t       findEnd;
    uint32_t       matchEnd;
    int32_t        isApproved = RM_FALSE;

    switch (privilegeCfg->type) {
        case Rm_policyCheck_INIT:
            permShift = RM_POLICY_PERM_INIT_SHIFT;
            break;
        case Rm_policyCheck_USE:
            permShift = RM_POLICY_PERM_USE_SHIFT;
            break;
        case Rm_policyCheck_EXCLUSIVE:
            permShift = RM_POLICY_PERM_EXCLUSIVE_SHIFT;
            break;
        case Rm_policyCheck_SHARED_LINUX:
            permShift = RM_POLICY_PERM_SHARED_LINUX_SHIFT;
            break;
        case Rm_policyCheck_UNSPEC_EXCLUSION:
            permShift = RM_POLICY_PERM_UNSPEC_EXCLUSION_SHIFT;
            break;
        default:
            return(isApproved);
    }

    /* Calculate the word indices and offsets for the global permissions and
     * the specific instance */
    globPermIdx    = RM_policy_PERM_INDEX(RM_POLICY_GLOBAL_PERM_INDEX);
    globPermOffset = RM_policy_PERM_OFFSET(RM_POLICY_GLOBAL_PERM_INDEX);
    instPermIdx    = RM_policy_PERM_INDEX(privilegeCfg->validInstNode->instIdx);
    instPermOffset = RM_policy_PERM_OFFSET(privilegeCfg->validInstNode->instIdx);

    memset((void *)&findNode, 0, sizeof(findNode));
    findNode.base = privilegeCfg->resourceBase;
    findNode.len  = privilegeCfg->resourceLength;
    /* Get first matching node. */
    matchNode = RB_FIND(_Rm_AllocatorPolicyTree, privilegeCfg->polTree,
                        &findNode);
    if (matchNode) {
        /* Request range may not be completely contained within the first
         * matching node.  Find furthest left matching node for the request
         * range */
        while (findNode.base < matchNode->base) {
            matchNode = RB_PREV(_Rm_AllocatorPolicyTree,
                                privilegeCfg->polTree, matchNode);
        }
    }

    /* Check permissions across all policy nodes which the request range
     * spans.  Assume approved until denial found */
    isApproved = RM_TRUE;
    while (matchNode) {
        if (privilegeCfg->negCheck) {
            /* Not approved if any matching node is assigned an exclusion
             * permission */
            if ((RM_policy_PERM_GET(matchNode->perms, globPermIdx,
                                    globPermOffset, permShift)) ||
                (RM_policy_PERM_GET(matchNode->perms, instPermIdx,
                                    instPermOffset, permShift))) {
                isApproved = RM_FALSE;
                break;
            }
        } else {
            /* Not approved if any matching node does not have permission */
            if ((!RM_policy_PERM_GET(matchNode->perms, globPermIdx,
                                     globPermOffset, permShift)) &&
                (!RM_policy_PERM_GET(matchNode->perms, instPermIdx,
                                     instPermOffset, permShift))) {
                isApproved = RM_FALSE;
                break;
            }
        }

        matchEnd = matchNode->base + matchNode->len - 1;
        findEnd  = findNode.base + findNode.len - 1;

        /* Check node to right if request range spans matching node to right */
        if (findEnd > matchEnd) {
            matchNode = RB_NEXT(_Rm_AllocatorPolicyTree, privilegeCfg->polTree,
                                matchNode);
            if (matchNode == NULL) {
                /* Request range outspans actual resource range */
                isApproved = RM_FALSE;
            }
        } else {
            break;
        }
    }

    return(isApproved);
}

/* FUNCTION PURPOSE: Returns resource base value according to the Policy
 ***********************************************************************
 * DESCRIPTION: Returns a resource base value based on the resource
 *              ranges assigned to the specified valid instance by the
 *              Policy DTB.
 */
int32_t rmPolicyGetResourceBase(Rm_PolicyTree *policyTree,
                                 Rm_PolicyValidInstNode *validInstNode,
                                 Rm_PolicyCheckType checkType,
                                 int32_t *resBase)
{
    uint32_t       permShift = RM_POLICY_PERM_INIT_SHIFT;
    uint32_t       globPermIdx;
    uint32_t       globPermOffset;
    uint32_t       instPermIdx;
    uint32_t       instPermOffset;
    Rm_PolicyNode *polNode;

    *resBase = RM_RESOURCE_BASE_UNSPECIFIED;

    if (checkType == Rm_policyCheck_INIT) {
        permShift = RM_POLICY_PERM_INIT_SHIFT;
    } else if (checkType == Rm_policyCheck_USE) {
        permShift = RM_POLICY_PERM_USE_SHIFT;
    } else {
        return(RM_ERROR_INVALID_SERVICE_TYPE);
    }

    /* Calculate the word indices and offsets for the global permissions and
     * the specific instance */
    globPermIdx    = RM_policy_PERM_INDEX(RM_POLICY_GLOBAL_PERM_INDEX);
    globPermOffset = RM_policy_PERM_OFFSET(RM_POLICY_GLOBAL_PERM_INDEX);
    instPermIdx    = RM_policy_PERM_INDEX(validInstNode->instIdx);
    instPermOffset = RM_policy_PERM_OFFSET(validInstNode->instIdx);

    RB_FOREACH(polNode, _Rm_AllocatorPolicyTree, policyTree) {
        if (RM_policy_PERM_GET(polNode->perms, globPermIdx, globPermOffset,
                               permShift) ||
            RM_policy_PERM_GET(polNode->perms, instPermIdx, instPermOffset,
                               permShift)) {
            *resBase = polNode->base;
            break;
        }
    }

    if (*resBase == RM_RESOURCE_BASE_UNSPECIFIED) {
        return(RM_SERVICE_DENIED_RES_ALLOC_REQS_NOT_MET);
    }

    return(RM_OK);
}

/* FUNCTION PURPOSE: Returns resource's allocation alignment
 ***********************************************************************
 * DESCRIPTION: Returns a resource's allocation alignment.
 */
uint32_t rmPolicyGetAllocAlign(Rm_PolicyTree *policyTree)
{
    Rm_PolicyNode *node = NULL;
    uint32_t       align = 1;

    /* Resource's alignment is global at the moment so all policy tree nodes
     * store the same value.  Just get the min node and return the allocation
     * alignment from it */
    node = RB_MIN(_Rm_AllocatorPolicyTree, policyTree);
    if (node) {
        align = node->allocAlign;
    }

    return(align);
}

/* FUNCTION PURPOSE: Returns resource's CD allocation size
 ***********************************************************************
 * DESCRIPTION: Returns a resource's CD allocation size.
 */
uint32_t rmPolicyGetCdAllocSize(Rm_PolicyTree *policyTree)
{
    Rm_PolicyNode *node = NULL;
    uint32_t       allocSize = 0;

    /* Resource's CD allocation size is global at the moment so all policy
     * tree nodes store the same value.  Just get the min node and return
     * allocation size from it */
    node = RB_MIN(_Rm_AllocatorPolicyTree, policyTree);
    if (node) {
        allocSize = node->cdAllocSize;
    }

    return(allocSize);
}

/* FUNCTION PURPOSE: Initializes the valid instance tree for a RM instance
 ***********************************************************************
 * DESCRIPTION: Creates the valid instance tree for a RM instance 
 *              that has been provided a global or static policy
 *              The valid instance tree is created from the
 *              "valid-instances" property at the top of the Policy.
 *              The root entry of the valid instance tree is returned.
 */
Rm_PolicyValidInstTree *rmPolicyVInstTreeInit(Rm_Handle rmHandle,
                                              void *policyDtb,
                                              int addLinux,
                                              int32_t *result)
{
    Rm_Inst                *rmInst = (Rm_Inst *)rmHandle;
    int32_t                 validInstOffset;
    const char             *validInstName = NULL;
    int32_t                 validInstLen;
    const void             *validInstData = NULL;
    Rm_PolicyPropType       propertyType;
    Rm_PolicyValidInst     *vInstListStart = NULL;
    Rm_PolicyValidInst     *validInstList = NULL;
    Rm_PolicyValidInstTree *rootEntry = NULL;
    Rm_PolicyValidInstNode *newNode = NULL;
    char                    linuxName[] = RM_ALLOCATED_TO_LINUX;
    int32_t                 instIndex;

    /* Valid instance list must be first and only property in the root node of
     * the policyDtb */
    validInstOffset = fdt_first_property_offset(policyDtb,
                            RM_DTB_UTIL_STARTING_NODE_OFFSET);
    if (validInstOffset < -FDT_ERR_NOTFOUND) {
        *result = validInstOffset;
        return(NULL);
    }

    if (validInstOffset == -FDT_ERR_NOTFOUND) {
        *result = RM_ERROR_NO_VALID_INST_IN_POLICY;
        return(NULL);
    }
    validInstData = fdt_getprop_by_offset(policyDtb, validInstOffset,
                                          &validInstName, &validInstLen);
    propertyType = rmDtbUtilPolicyGetPropertyType(validInstName);
    if (propertyType != Rm_policyPropType_VALID_INSTANCES) {
        *result = RM_ERROR_NO_VALID_INST_IN_POLICY;
        return(NULL);
    }

    if (!(validInstList = rmDtbUtilPolicyExtractValidInstances(validInstData,
                                                               validInstLen,
                                                               result))) {
        return(NULL);
    }

    /* Create the tree */
    rootEntry = Rm_osalMalloc(sizeof(Rm_PolicyValidInstTree));
    RB_INIT(rootEntry);

    vInstListStart = validInstList;
    /* Zeroeth index is reserved for global permissions */
    instIndex = 1;
    while (validInstList) {
        newNode = rmPolicyValidInstNodeNew(validInstList->instName,
                                           instIndex);
        RB_INSERT(_Rm_PolicyValidInstTree, rootEntry, newNode);

        instIndex++;
        validInstList = validInstList->nextValidInst;
    }
    rmDtbUtilPolicyFreeValidInstances(vInstListStart);

    /* Add the Linux kernel node */
    if (addLinux) {
        newNode = rmPolicyValidInstNodeNew(linuxName, instIndex++);
        RB_INSERT(_Rm_PolicyValidInstTree, rootEntry, newNode);
    }

    /* Save the max index for proper accounting when validating against
     * policy */
    rmInst->maxInstIdx = instIndex;

    *result = RM_OK;
    return(rootEntry);
}

/* FUNCTION PURPOSE: Deletes a valid instance tree
 ***********************************************************************
 * DESCRIPTION: Frees all memory associated with a Policy valid
 *              instance tree.
 */
void rmPolicyVInstTreeDelete(Rm_Handle rmHandle)
{
    Rm_Inst                *rmInst = (Rm_Inst *)rmHandle;
    Rm_PolicyValidInstTree *treeRoot = rmInst->validInstTree;
    Rm_PolicyValidInstNode *node;
    Rm_PolicyValidInstNode *nextNode;

    if (treeRoot) {
        if (rmInst->instType == Rm_instType_SHARED_SERVER) {
            rmPolicyValidInstTreeInv(treeRoot);
        }

        for (node = RB_MIN(_Rm_PolicyValidInstTree, treeRoot);
             node != NULL;
             node = nextNode) {
            nextNode = RB_NEXT(_Rm_PolicyValidInstTree, treeRoot, node);
            RB_REMOVE(_Rm_PolicyValidInstTree, treeRoot, node);
            rmPolicyValidInstNodeFree(node);
        }

        /* Don't need to writeback tree node changes since valid instance will
         * be made NULL in instance */

        if (RB_MIN(_Rm_PolicyValidInstTree, treeRoot) == NULL) {
            /* No more valid instance nodes in tree */
            Rm_osalFree((void *)treeRoot, sizeof(treeRoot));
        }

        rmInst->validInstTree = NULL;
    }
}

/* FUNCTION PURPOSE: Populates an allocator's policy tree
 ***********************************************************************
 * DESCRIPTION: Populates an allocator's policy tree using the policy
 *              DTB
 */
int32_t rmPolicyPopulateTree(Rm_Handle rmHandle, Rm_PolicyTree *policyTree,
                             void *policyDtb, const char *resName)
{
    Rm_Inst             *rmInst = (Rm_Inst *)rmHandle;
    int32_t              resOffset;
    uint32_t             allocAlignment;
    uint32_t             cdAllocSize;
    uint32_t             permBufSizeBytes = 0;
    int32_t              propOffset;
    const void          *propData;
    const char          *propName;
    int32_t              propLen;
    Rm_PolicyAssignment *assign = NULL;
    Rm_PolicyAssignment *assignStart = NULL;
    Rm_PolicyNode       *polNode = NULL;
    Rm_PolicyPermission *extractedPerms = NULL;
    int32_t              retVal = RM_OK;

    /* Offset of resource in policy DTB */
    resOffset = policyGetDtbResourceOffset(policyDtb, resName);
    if (resOffset < 0) {
        retVal = resOffset;
        goto errorExit;
    }

    /* Get the allocation alignment and the CD allocation size since these
     * will be stored in each tree node */
    allocAlignment = policyGetDtbAllocAlign(policyDtb, resOffset);
    cdAllocSize = policyGetDtbCdAllocSize(policyDtb, resOffset);
    permBufSizeBytes = policyGetPermBufSize(rmInst->maxInstIdx);

    /* Get the resource assignments - should only be one per resource node */
    propOffset = fdt_first_property_offset(policyDtb, resOffset);
    while (propOffset > RM_DTB_UTIL_STARTING_NODE_OFFSET) {
        propData = fdt_getprop_by_offset(policyDtb, propOffset, &propName,
                                         &propLen);
        if (rmDtbUtilPolicyGetPropertyType(propName) ==
            Rm_policyPropType_ASSIGNMENTS) {
            assign = assignStart = rmDtbUtilPolicyExtractAssignments(propData,
                                                                     propLen);
            break;
        }
        propOffset = fdt_next_property_offset(policyDtb, propOffset);
    }

    /* Insert a new node into resource's allocator for each assignment range */
    while (assign) {
        polNode = rmPolicyNodeNew(assign->resourceBase,
                                  assign->resourceLength);
        polNode->allocAlign = allocAlignment;
        polNode->cdAllocSize = cdAllocSize;
        polNode->perms =  Rm_osalMalloc(permBufSizeBytes);
        polNode->permsLen = permBufSizeBytes;

        memset(polNode->perms, 0, polNode->permsLen);

        extractedPerms = policyGetAssignmentPermissions(assign, &retVal);
        if (retVal != RM_OK) {
            goto errorExit;
        }

        retVal = policyStorePermissions(rmInst, polNode, extractedPerms);
        if (retVal != RM_OK) {
            goto errorExit;
        }
        RB_INSERT(_Rm_AllocatorPolicyTree, policyTree, polNode);

        policyFreeAssignmentPermissions(extractedPerms);
        extractedPerms = NULL;
        polNode = NULL;

        assign = assign->nextAssignment;
    }

errorExit:
    if (polNode) {
        rmPolicyNodeFree(polNode);
    }
    if (extractedPerms) {
        policyFreeAssignmentPermissions(extractedPerms);
    }
    if (assignStart) {
        rmDtbUtilPolicyFreeAssignments(assignStart);
    }

    return(retVal);
}
