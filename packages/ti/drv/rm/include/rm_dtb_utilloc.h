/*
 *  file  rm_dtb_utilloc.h
 *
 *  Private Resource List and Policy DTB Parsing Utilities
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

#ifndef RM_DTB_UTILLOC_H_
#define RM_DTB_UTILLOC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* RM external includes */
#include <ti/drv/rm/rm.h>

/* DTB starting node offset for parsing */
#define RM_DTB_UTIL_STARTING_NODE_OFFSET 0
/* DTB starting depth for parsing */
#define RM_DTB_UTIL_STARTING_DEPTH 0
/* Linux DTB alias offset not set */
#define RM_DTB_UTIL_LINUX_ALIAS_OFFSET_NOT_SET 0xFFFF

/**********************************************************************
 ******************* Common DTB Parsing Definitions *******************
 **********************************************************************/

/* Resource range linked list node */
typedef struct Rm_ResourceRange_s {
    /* Resource base */
    uint32_t                   base;
    /* Resource length */
    uint32_t                   length;
    /* Link to next resource range linked list node */
    struct Rm_ResourceRange_s *nextRange;
} Rm_ResourceRange;

/* Resource value linked list node */
typedef struct Rm_ResourceValue_s {
    /* Numerical resource value */
    uint32_t                   value;
    /* Link to next resource value linked list node */
    struct Rm_ResourceValue_s *nextValue;
} Rm_ResourceValue;

/**********************************************************************
 ************* Global Resource List Parsing Definitions ***************
 **********************************************************************/

/* GRL resource node property types */
typedef enum {
    /** Resource DTB unknown property type */
    Rm_resourcePropType_UNKNOWN = 0,  
    /** Resource DTB resource range property type */
    Rm_resourcePropType_RESOURCE_RANGE,
    /** Resource DTB resource alias path in Linux DTB */
    Rm_resourcePropType_RESOURCE_LINUX_ALIAS,    
    /** Resource DTB NameServer assignment property type */
    Rm_resourcePropType_NSASSIGNMENT  
} Rm_ResourcePropType;

/* Linux alias path linked list node */
typedef struct Rm_LinuxAlias_s {
    /* Pointer to the alias path string */
    char                   *path;
    /* Base value offset into Linux value range located at the end
     * of the path in the Linux DTB */
    uint32_t                baseOffset;
    /* Length value offset into the Linux value range located at 
     * the end of the path in the Linux DTB.  Will be set to
     * RM_DTB_UTIL_LINUX_ALIAS_OFFSET_NOT_SET if there is only
     * a base value to read from the Linux DTB */
    uint32_t                lengthOffset;
    /* Link to next Linux alias linked list node */
    struct Rm_LinuxAlias_s *nextLinuxAlias;
} Rm_LinuxAlias;

/* NameServer assignment linked list node */
typedef struct Rm_NsAssignment_s {
    /* NameServer name to be created */
    char                      nsName[RM_NAME_MAX_CHARS];
    /* Resource base value to tie to the name */
    uint32_t                  resourceBase;
    /* Resource length value to tie to the name */
    uint32_t                  resourceLength;
    /* Link to next NameServer assignment linked list node */
    struct Rm_NsAssignment_s *nextNsAssignment;
} Rm_NsAssignment;

/**********************************************************************
 ********************* Policy Parsing Definitions *********************
 **********************************************************************/

/* Policy resource node property types */
typedef enum {
    /** Policy DTB unknown property type */
    Rm_policyPropType_UNKNOWN = 0,     
    /** Policy DTB resource assignments property type */
    Rm_policyPropType_ASSIGNMENTS,
    /** Policy DTB Client Delegate resource allocation size property type */
    Rm_policyPropType_CD_ALLOCATION_SIZE,
    /** Policy DTB allocation alignment property type */
    Rm_policyPropType_ALLOCATION_ALIGNMENT,
    /** Policy DTB valid RM instances property type */
    Rm_policyPropType_VALID_INSTANCES
} Rm_PolicyPropType;

/* Policy assignment linked list node */
typedef struct Rm_PolicyAssignment_s {
    /* Resource base affected by the permissions */
    uint32_t                      resourceBase;
    /* Resource length (started from base) affected by the permissions */
    uint32_t                      resourceLength;
    /* Permissions string defining which RM instances get
     * which permissions for the defined resource range starting
     * at resourceBase and ending at
     * resourceBase+resourceLength-1 */
    char                         *permissionsList;
    /* Link to next policy assignment linked list node */
    struct Rm_PolicyAssignment_s *nextAssignment;
} Rm_PolicyAssignment;

/* Valid instance linked list node */
typedef struct Rm_PolicyValidInst_s {
    /* RM instance name */
    char                         instName[RM_NAME_MAX_CHARS];
    /* Link to next valid instance linked list node */
    struct Rm_PolicyValidInst_s *nextValidInst;
} Rm_PolicyValidInst;

/**********************************************************************
 ******************** Linux DTB Parsing Definitions *******************
 **********************************************************************/

/* Linux value range linked list node */
typedef struct Rm_LinuxValueRange_s {
    /* Numerical value extracted from Linux DTB */
    uint32_t                     value;
    /* Link to next Linux value linked list node */
    struct Rm_LinuxValueRange_s *nextValue;
} Rm_LinuxValueRange;

/**********************************************************************
 *************** Global Resource List Parsing Functions ***************
 **********************************************************************/

Rm_ResourcePropType rmDtbUtilResGetPropertyType(const char *propertyName);
Rm_ResourceRange *rmDtbUtilResExtractRange(const void *dtbDataPtr,
                                           int32_t dtbDataLen);
void rmDtbUtilResFreeRange(Rm_ResourceRange *rangeList);
Rm_LinuxAlias *rmDtbUtilResExtractLinuxAlias(const void *dtbDataPtr,
                                             int32_t dtbDataLen,
                                             int32_t *result);
void rmDtbUtilResFreeLinuxAlias(Rm_LinuxAlias *aliasList);
Rm_NsAssignment *rmDtbUtilResExtractNsAssignment(const void *dtbDataPtr,
                                                 int32_t dtbDataLen,
                                                 int32_t *result);
void rmDtbUtilResFreeNsAssignmentList(Rm_NsAssignment *nsAssignmentList);

/**********************************************************************
 ********************** Policy Parsing Functions***********************
 **********************************************************************/

Rm_PolicyPropType rmDtbUtilPolicyGetPropertyType(const char *propertyName);
Rm_PolicyAssignment *rmDtbUtilPolicyExtractAssignments(const void *dtbDataPtr,
                                                       int32_t dtbDataLen);
void rmDtbUtilPolicyFreeAssignments(Rm_PolicyAssignment *assignmentList);
Rm_ResourceValue *rmDtbUtilPolicyExtractCdAllocationSizes(const void *dtbDataPtr,
                                                          int32_t dtbDataLen);
void rmDtbUtilPolicyFreeCdAllocationSizes(Rm_ResourceValue *allocationSizeList);
Rm_ResourceValue *rmDtbUtilPolicyExtractResourceAlignments(const void *dtbDataPtr,
                                                           int32_t dtbDataLen);
void rmDtbUtilPolicyFreeResourceAlignments (Rm_ResourceValue *alignmentList);
Rm_PolicyValidInst *rmDtbUtilPolicyExtractValidInstances(const void *dtbDataPtr,
                                                         int32_t dtbDataLen,
                                                         int32_t *result);
void rmDtbUtilPolicyFreeValidInstances (Rm_PolicyValidInst *validInstList);

/**********************************************************************
 ********************* Linux DTB Parsing Functions ********************
 **********************************************************************/

Rm_LinuxValueRange *rmDtbUtilLinuxExtractValues(const void *dtbDataPtr,
                                                int32_t dtbDataLen);
void rmDtbUtilLinuxFreeValues(Rm_LinuxValueRange *valueList);

#ifdef __cplusplus
}
#endif

#endif /* RM_DTB_UTILLOC_H_ */

