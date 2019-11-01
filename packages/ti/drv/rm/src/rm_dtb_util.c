/**
 *   @file  rm_dtb_util.c
 *
 *   @brief   
 *      This is the Resource Manager Resource List and Policy DTB parsing 
 *      utility source
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

/* RM external API includes */
#include <ti/drv/rm/rm.h>

/* RM internal API includes */
#include <ti/drv/rm/include/rm_internal.h>
#include <ti/drv/rm/include/rm_dtb_utilloc.h>

/* RM OSAL layer */
#include <rm_osal.h>

/* LIBFDT includes */
#include <ti/drv/rm/util/libfdt/libfdt.h>
#include <ti/drv/rm/util/libfdt/libfdt_env.h>

/**********************************************************************
 ************ Global Resource List (GRL) Parsing Globals **************
 **********************************************************************/

/* Resource List Properties - These are the property values used
 * by an application integrator to define the properties of resources
 * listed in the Device Global Resource List (GRL) */
const char dtbUtilResRangeProp[]        = "resource-range";
const char dtbUtilResLinuxAliasProp[]   = "linux-dtb-alias";
const char dtbUtilResNsAssignmentProp[] = "ns-assignment";

/**********************************************************************
 *********************** Policy Parsing Globals ***********************
 **********************************************************************/

/* Policy Properties - These are the property values used
 * by an application integrator to define the properties of resources
 * listed in a Resource Manager Policy */
const char dtbUtilPolicyValidInstances[]      = "valid-instances";
const char dtbUtilPolicyAssignments[]         = "assignments";
const char dtbUtilPolicyCdAllocationSize[]    = "cd-allocation-size";
const char dtbUtilPolicyAllocationAlignment[] = "allocation-alignment";

/**********************************************************************
 ******************* Common DTB Parsing Functions *********************
 **********************************************************************/

/* FUNCTION PURPOSE: Extracts ranges from a DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of ranges extracted from a DTB
 *              file.  A range in the DTB is specified as <x y> where
 *              x is a base value and y is a length value.  Ranges can
 *              be chained in the DTB with comma separation.
 *              Example:
 *                  <0 10>,
 *                  <20 100>,
 *                  <200 100>;
 *                  Results in a list of three ranges.
 */
static Rm_ResourceRange *dtbUtilCommonExtractRange(const void *dtbDataPtr,
                                                   int32_t dtbDataLen)
{
    uint32_t         *dtbRangeData = (uint32_t *)dtbDataPtr;
    Rm_ResourceRange *startRange = NULL;
    Rm_ResourceRange *newRange = NULL;
    Rm_ResourceRange *prevRange = NULL;
    uint32_t i;
    
    /* Ranges are stored in the DTB as a list of 32-bit words.  The number of
     * 32-bit words in the DTB ranges field should be even since ranges are
     * specified in a base, lenth format. The DTB gives properties lengths in
     * bytes so the length returned from the DTB should be a multiple of the
     * number of bytes in two uint32_ts. */
    if (dtbDataLen % (2 * sizeof(uint32_t))) {
        return (NULL);
    }

    for (i = 0; i < (dtbDataLen / sizeof(uint32_t)); i+=2) {
        newRange = (Rm_ResourceRange *) Rm_osalMalloc(sizeof(Rm_ResourceRange));
        newRange->base = fdt32_to_cpu(dtbRangeData[i]);
        newRange->length = fdt32_to_cpu(dtbRangeData[i+1]);
        newRange->nextRange = NULL;
        
        if (prevRange == NULL) {
            startRange = newRange;
        } else {
            prevRange->nextRange = newRange;
        }
        prevRange = newRange;
    }
    return(startRange);
}

/* FUNCTION PURPOSE: Deletes a range list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of ranges 
 *              that were extracted from a DTB.
 */
static void dtbUtilCommonFreeRangeList(Rm_ResourceRange *rangeList)
{
    Rm_ResourceRange *nextRange;

    while (rangeList) {
        nextRange = rangeList->nextRange;
        Rm_osalFree((void *)rangeList, sizeof(Rm_ResourceRange));
        rangeList = nextRange;
    }
}

/* FUNCTION PURPOSE: Extracts values from a DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of values extracted from a DTB
 *              file.  A value in the DTB is specified as <x> where
 *              x is a numerical value.  Values can be chained in the 
 *              DTB with comma separation.
 *              Example:
 *                  <10>,
 *                  <20>,
 *                  <30>;
 *                  Results in a list of three values.
 */
static Rm_ResourceValue *dtbUtilCommonExtractValueList(const void *dtbDataPtr,
                                                       int32_t dtbDataLen)
{
    uint32_t         *dtbRangeData = (uint32_t *)dtbDataPtr;
    Rm_ResourceValue *startValue = NULL;
    Rm_ResourceValue *newValue = NULL;
    Rm_ResourceValue *prevValue = NULL;
    uint32_t          i;
    
    /* Values are stored in the DTB as a list of 32-bit words. The DTB 
     * gives properties lengths in bytes so the length returned from the DTB 
     * should be a multiple of the number of bytes a uint32_t. */
    if (dtbDataLen % sizeof(uint32_t)) {
        return (NULL);
    }
    
    for (i = 0; i < (dtbDataLen / sizeof(uint32_t)); i++) {
        newValue = (Rm_ResourceValue *) Rm_osalMalloc(sizeof(Rm_ResourceValue));
        newValue->value = fdt32_to_cpu(dtbRangeData[i]);
        newValue->nextValue = NULL;

        if (prevValue == NULL) {
            startValue = newValue;
        } else {
            prevValue->nextValue = newValue;
        }
        prevValue = newValue;
    }
    return(startValue);
}

/* FUNCTION PURPOSE: Deletes a value list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of values
 *              that were extracted from a DTB.
 */
static void dtbUtilCommonFreeValueList (Rm_ResourceValue *valueList)
{
    Rm_ResourceValue *nextValue;

    while (valueList) {
        nextValue = valueList->nextValue;
        Rm_osalFree((void *)valueList, sizeof(Rm_ResourceValue));
        valueList = nextValue;
    }
}

/**********************************************************************
 ************** Global Resource List Parsing Functions ****************
 **********************************************************************/

/* FUNCTION PURPOSE: Returns the GRL property type
 ***********************************************************************
 * DESCRIPTION: Returns the GRL property type associated with the
 *              property name specified.
 */
Rm_ResourcePropType rmDtbUtilResGetPropertyType(const char * propertyName)
{
    Rm_ResourcePropType propertyType;

    if(strcmp(dtbUtilResRangeProp, propertyName) == 0) {
        propertyType = Rm_resourcePropType_RESOURCE_RANGE;
    } else if(strcmp(dtbUtilResLinuxAliasProp, propertyName) == 0) {
        propertyType = Rm_resourcePropType_RESOURCE_LINUX_ALIAS;
    } else if(strcmp(dtbUtilResNsAssignmentProp, propertyName) == 0) {
        propertyType = Rm_resourcePropType_NSASSIGNMENT;
    } else {
        propertyType = Rm_resourcePropType_UNKNOWN;
    }
    return(propertyType);
}

/* FUNCTION PURPOSE: Extracts a resource range list from a GRL DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of resource ranges extracted from a GRL
 *              DTB.  A resource range in the DTB is specified as <x y> 
 *              where x is resource base and y is a resource length
 *              starting from x.  Resource ranges can be chained in the 
 *              GRL DTB with comma separation.
 *              Example:
 *                  <0 10>,
 *                  <10 10>,
 *                  <20 10>;
 *                  Results in a list of three resource ranges.
 */
Rm_ResourceRange *rmDtbUtilResExtractRange(const void *dtbDataPtr,
                                           int32_t dtbDataLen)
{
    return(dtbUtilCommonExtractRange(dtbDataPtr, dtbDataLen));
}

/* FUNCTION PURPOSE: Deletes a resource range list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of resource 
 *              ranges that were extracted from a GRL DTB.
 */
void rmDtbUtilResFreeRange(Rm_ResourceRange *rangeList)
{
    dtbUtilCommonFreeRangeList(rangeList);
}

/* FUNCTION PURPOSE: Extracts a Linux alias list from a GRL DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of Linux alias paths extracted from a GRL
 *              DTB.  A Linux alias path in the DTB is specified as a 
 *              space separated string following by list of numerical 
 *              values.  Each word within the space separated string 
 *              defines a node from the Linux DTB in the path to the
 *              Linux alias value for the resource.  The numerical values
 *              specify the number of values at the end of the path in
 *              the Linux DTB.  Linux alias paths can be chained in the 
 *              GRL DTB with comma separation.
 *              Example:
 *                  "LinuxDtbNode1 LinuxDtbNode2 resourcehere", <1 1>,
 *                  "LinuxDtbNode1 LinuxDtbNode2 anotherRes", <2 2 3>,
 *                  Results in a list of two Linux alias paths.
 */
Rm_LinuxAlias *rmDtbUtilResExtractLinuxAlias(const void *dtbDataPtr,
                                             int32_t dtbDataLen,
                                             int32_t *result)
{
    uint8_t       *dtbAliasData = (uint8_t *)dtbDataPtr;
    uint32_t       pathLenBytes;
    uint32_t       extractedValue;
    uint8_t       *extractedValueBytePtr;
    Rm_LinuxAlias *startAlias = NULL;
    Rm_LinuxAlias *newAlias = NULL;
    Rm_LinuxAlias *prevAlias = NULL;
    uint32_t       numOffsets;
    int32_t        i = 0;
    uint16_t       j;

    /* Linux aliases are stored in the DTB as a list space-separated path node
     * names within null terminated string.  Following the path string  will be
     * two or three values specifying the fields in the linux DTB property that
     * contain the relevant data.  The first value specifies the number of
     * offsets that follow.  If two field offsets are specified the Linux DTB
     * contains a base + length. If   one field offset is specified the Linux
     * DTB contains a single value for reservation.  There is no padding
     * between the path string and the 32-bit offsets.  Therefore the 32-bit
     * offsets may not be on a 4-byte boundary and must be constructed upon
     * extraction */
    while(i < dtbDataLen) {
        newAlias = (Rm_LinuxAlias *) Rm_osalMalloc(sizeof(Rm_LinuxAlias));

        pathLenBytes = strlen((char *) &dtbAliasData[i]) + 1;
        newAlias->path = (char *) Rm_osalMalloc(pathLenBytes);
        rm_strncpy(newAlias->path, ((char *) &dtbAliasData[i]), pathLenBytes);

        /* Extract 32-bit value specifying number of offsets that follow */
        i += pathLenBytes;
        extractedValueBytePtr = (uint8_t *)&extractedValue;
        for (j = 0; j < sizeof(uint32_t); j++, i++) {
            extractedValueBytePtr[j] = dtbAliasData[i];
        }
        numOffsets = fdt32_to_cpu(extractedValue);

        if ((numOffsets == 1) || (numOffsets == 2)) {
            /* Always extract the base value */
            extractedValueBytePtr = (uint8_t *)&extractedValue;
            for (j = 0; j < sizeof(uint32_t); j++, i++) {
                extractedValueBytePtr[j] = dtbAliasData[i];
            }
            newAlias->baseOffset = fdt32_to_cpu(extractedValue);

            if (numOffsets == 2) {
                for (j = 0; j < sizeof(uint32_t); j++, i++) {
                    extractedValueBytePtr[j] = dtbAliasData[i];
                }
                newAlias->lengthOffset = fdt32_to_cpu(extractedValue);
            } else {
                newAlias->lengthOffset = RM_DTB_UTIL_LINUX_ALIAS_OFFSET_NOT_SET;
            }

            newAlias->nextLinuxAlias = NULL;
            if (prevAlias == NULL) {
                startAlias = newAlias;
            } else {
                prevAlias->nextLinuxAlias = newAlias;
            }
            prevAlias = newAlias;
        } else {
            Rm_osalFree((void *)newAlias->path, pathLenBytes);
            Rm_osalFree((void *)newAlias, sizeof(Rm_LinuxAlias));
            while (startAlias) {
                newAlias = startAlias->nextLinuxAlias;
                pathLenBytes = strlen(startAlias->path);
                Rm_osalFree((void *)startAlias->path, pathLenBytes + 1);
                Rm_osalFree((void *)startAlias, sizeof(Rm_LinuxAlias));
                startAlias = newAlias;
            }
            *result = RM_ERROR_GRL_INVALID_LINUX_ALIAS_FORMAT;
            return(NULL);
        }
    }
    return(startAlias);
}

/* FUNCTION PURPOSE: Deletes a Linux Alias list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of Linux
 *              aliases that were extracted from a GRL DTB.
 */
void rmDtbUtilResFreeLinuxAlias(Rm_LinuxAlias *aliasList)
{
    Rm_LinuxAlias *nextAlias;
    int32_t        pathSize;

    while (aliasList) {
        nextAlias = aliasList->nextLinuxAlias;
        pathSize = strlen(aliasList->path);
        Rm_osalFree((void *)aliasList->path, pathSize + 1);
        Rm_osalFree((void *)aliasList, sizeof(Rm_LinuxAlias));
        aliasList = nextAlias;
    }
}

/* FUNCTION PURPOSE: Extracts a NS assignment list from a GRL DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of NameServer assignments extracted from
 *              a GRL DTB.  A NameServer assignment is specified with
 *              a string representing the NameServer name following by
 *              a range in <base length> format.  NameServer assignments
 *              can be chained in the GRL DTB with comma separation.
 *              Example:
 *                  "Name1", <0 10>,
 *                  "Name2", <10 10>,
 *                  "Another Name", <20 10>;
 *                  Results in a list of three NameServer assignments.
 */
Rm_NsAssignment *rmDtbUtilResExtractNsAssignment(const void *dtbDataPtr,
                                                 int32_t dtbDataLen,
                                                 int32_t *result)
{
    uint8_t         *dtbNsAssignmentData = (uint8_t *)dtbDataPtr;
    uint32_t         nameLenBytes;
    uint32_t         extractedValue;
    uint8_t         *extractedValueBytePtr;
    Rm_NsAssignment *startAssignment = NULL;
    Rm_NsAssignment *newAssignment = NULL;
    Rm_NsAssignment *prevAssignment = NULL;
    int32_t          i = 0;
    uint16_t         j;

    /* NameServer assignments are stored in the DTB as a null-terminated
     * character string followed by a 32-bit word containing the value to be
     * assigned to the name in the string.  There is no padding between the
     * string and the 32-bit word.  Therefore the 32-bit word may not be on a
     * 4-byte boundary and must be constructed upon extraction */
    while(i < dtbDataLen) {
        newAssignment = (Rm_NsAssignment *)Rm_osalMalloc(sizeof(Rm_NsAssignment));

        nameLenBytes = strlen((char *) &dtbNsAssignmentData[i]) + 1;
        if (nameLenBytes > RM_NAME_MAX_CHARS) {
            Rm_osalFree((void *)newAssignment, sizeof(Rm_NsAssignment));
            while (startAssignment) {
                newAssignment = startAssignment->nextNsAssignment;
                Rm_osalFree((void *)startAssignment, sizeof(Rm_NsAssignment));
                startAssignment = newAssignment;
            }
            *result = RM_ERROR_NAMESERVER_NAME_TOO_LONG;
            return(NULL);
        }
        rm_strncpy(newAssignment->nsName, ((char *) &dtbNsAssignmentData[i]),
                   RM_NAME_MAX_CHARS);

        /* Extract 32-bit base value and flip endian */
        i += nameLenBytes;
        extractedValueBytePtr = (uint8_t *)&extractedValue;
        for (j = 0; j < sizeof(uint32_t); j++, i++) {
            extractedValueBytePtr[j] = dtbNsAssignmentData[i];
        }
        newAssignment->resourceBase = fdt32_to_cpu(extractedValue);

        /* Extract 32-bit length value and flip endian */
        for (j = 0; j < sizeof(uint32_t); j++, i++) {
            extractedValueBytePtr[j] = dtbNsAssignmentData[i];
        }
        newAssignment->resourceLength = fdt32_to_cpu(extractedValue);

        newAssignment->nextNsAssignment = NULL;
        if (prevAssignment == NULL) {
            startAssignment = newAssignment;
        } else {
            prevAssignment->nextNsAssignment = newAssignment;
        }
        prevAssignment = newAssignment;
    }

    return(startAssignment);
}

/* FUNCTION PURPOSE: Deletes a NameServer assignment list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of NameServer 
 *              assignments that were extracted from a GRL DTB.
 */
void rmDtbUtilResFreeNsAssignmentList (Rm_NsAssignment *nsAssignmentList)
{
    Rm_NsAssignment *nextAssignment;

    while (nsAssignmentList) {
        nextAssignment = nsAssignmentList->nextNsAssignment;
        Rm_osalFree((void *)nsAssignmentList, sizeof(Rm_NsAssignment));
        nsAssignmentList = nextAssignment;
    }
}

/**********************************************************************
 ********************** Policy Parsing Functions **********************
 **********************************************************************/

/* FUNCTION PURPOSE: Returns the Policy property type
 ***********************************************************************
 * DESCRIPTION: Returns the Policy property type associated with the 
 *              property name specified.
 */
Rm_PolicyPropType rmDtbUtilPolicyGetPropertyType(const char * propertyName)
{
    Rm_PolicyPropType propertyType;

    if(strcmp(dtbUtilPolicyAssignments, propertyName) == 0) {
        propertyType = Rm_policyPropType_ASSIGNMENTS;
    } else if(strcmp(dtbUtilPolicyCdAllocationSize, propertyName) == 0) {
        propertyType = Rm_policyPropType_CD_ALLOCATION_SIZE;
    } else if(strcmp(dtbUtilPolicyAllocationAlignment, propertyName) == 0) {
        propertyType = Rm_policyPropType_ALLOCATION_ALIGNMENT;
    } else if(strcmp(dtbUtilPolicyValidInstances, propertyName) == 0) {
        propertyType = Rm_policyPropType_VALID_INSTANCES;
    } else {
        propertyType = Rm_policyPropType_UNKNOWN;
    }
    return(propertyType);
}

/* FUNCTION PURPOSE: Extracts a policy assignment list from a Policy DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of policy assignments extracted from a 
 *              Policy DTB.  A policy assignment in the DTB is 
 *              specified as <x y>, "permissions string" where x is 
 *              resource base and y is a resource length starting 
 *              from x.  "permissions string" is string specifying
 *              which RM instances are allowed to use the resource
 *              range.  Policy assignments can be chained in the 
 *              Policy DTB with comma separation.
 *              Example:
 *                  <0 10>, "iux = (Rm_Client)",
 *                  <10 10>, "iu = (*);
 *                  Results in a list of two policy assignments.
 */
Rm_PolicyAssignment *rmDtbUtilPolicyExtractAssignments(const void *dtbDataPtr,
                                                       int32_t dtbDataLen)
{
    uint8_t             *dtbAssignmentData = (uint8_t *)dtbDataPtr;
    uint32_t             permissionsLenBytes;
    uint32_t             extractedValue;
    uint8_t             *extractedValueBytePtr;
    Rm_PolicyAssignment *startAssignment = NULL;
    Rm_PolicyAssignment *newAssignment = NULL;
    Rm_PolicyAssignment *prevAssignment = NULL;
    int32_t              i = 0;
    uint16_t             j;

    /* Policy assignments are stored in the DTB as two 32-bit words containing
     * a resource base and length to be assigned the permissions in the defined
     * in the string that follows.  There is no padding between the 32-bit
     * words and the string.  Therefore the 32-bit word may not be on a 4-byte
     * boundary and must be constructed upon extraction */
    while(i < dtbDataLen) {
        newAssignment = (Rm_PolicyAssignment *)Rm_osalMalloc(sizeof(Rm_PolicyAssignment));

        /* Extract 32-bit resource base value and flip endianness */
        extractedValueBytePtr = (uint8_t *)&extractedValue;
        for (j = 0; j < sizeof(uint32_t); j++, i++) {
            extractedValueBytePtr[j] = dtbAssignmentData[i];
        }
        newAssignment->resourceBase = fdt32_to_cpu(extractedValue);

        /* Extract 32-bit resource length value and flip endianness */
        for (j = 0; j < sizeof(uint32_t); j++, i++) {
            extractedValueBytePtr[j] = dtbAssignmentData[i];
        }
        newAssignment->resourceLength = fdt32_to_cpu(extractedValue);

        permissionsLenBytes = strlen((char *) &dtbAssignmentData[i]) + 1;
        newAssignment->permissionsList = (char *)Rm_osalMalloc(permissionsLenBytes);
        rm_strncpy(newAssignment->permissionsList,
                   ((char *)&dtbAssignmentData[i]), permissionsLenBytes);
        i += permissionsLenBytes;

        newAssignment->nextAssignment = NULL;
        if (prevAssignment == NULL) {
            startAssignment = newAssignment;
        } else {
            prevAssignment->nextAssignment = newAssignment;
        }
        prevAssignment = newAssignment;
    }
    return(startAssignment);
}

/* FUNCTION PURPOSE: Deletes a policy assignment list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of policy 
 *              assignments that were extracted from a Policy DTB.
 */
void rmDtbUtilPolicyFreeAssignments(Rm_PolicyAssignment *assignmentList)
{
    Rm_PolicyAssignment *nextAssignment;
    int32_t              permissionsSize;

    while (assignmentList) {
        nextAssignment = assignmentList->nextAssignment;
        permissionsSize = strlen(assignmentList->permissionsList);
        Rm_osalFree((void *)assignmentList->permissionsList, permissionsSize+1);
        Rm_osalFree((void *)assignmentList, sizeof(Rm_PolicyAssignment));
        assignmentList = nextAssignment;
    }
}

/* FUNCTION PURPOSE: Extracts a CD allocation size from a Policy DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of allocation sizes extracted from a
 *              Policy DTB.  A allocation size in the DTB is specified
 *              as <x> where x is the allocation size for a resource.
 *              Allocation sizes can be chained in the Policy DTB with
 *              comma separation.
 *              Example:
 *                  <10>,
 *                  <100>;
 *                  Results in a list of two allocation sizes.
 */
Rm_ResourceValue *rmDtbUtilPolicyExtractCdAllocationSizes(const void *dtbDataPtr,
                                                          int32_t dtbDataLen)
{
    return(dtbUtilCommonExtractValueList(dtbDataPtr, dtbDataLen));
}

/* FUNCTION PURPOSE: Deletes a CD allocation sizes list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of allocation
 *              sizes that were extracted from a Policy DTB.
 */
void rmDtbUtilPolicyFreeCdAllocationSizes (Rm_ResourceValue *allocationSizeList)
{
    dtbUtilCommonFreeValueList(allocationSizeList);
}

/* FUNCTION PURPOSE: Extracts an resource alignment from a Policy DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of resource alignments extracted from a
 *              Policy DTB.  A resource alignment in the DTB is specified
 *              as <x> where x is the resource alignment for a resource.
 *              Resource alignments can be chained in the Policy DTB with
 *              comma separation.
 *              Example:
 *                  <10>,
 *                  <100>;
 *                  Results in a list of two resource alignments.
 */
Rm_ResourceValue *rmDtbUtilPolicyExtractResourceAlignments(const void *dtbDataPtr,
                                                           int32_t dtbDataLen)
{
    return(dtbUtilCommonExtractValueList(dtbDataPtr, dtbDataLen));
}

/* FUNCTION PURPOSE: Deletes a resource alignments list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of resource
 *              alignments that were extracted from a Policy DTB.
 */
void rmDtbUtilPolicyFreeResourceAlignments(Rm_ResourceValue *alignmentList)
{
    dtbUtilCommonFreeValueList(alignmentList);
}

/* FUNCTION PURPOSE: Extracts an valid instance list from a Policy DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of valid instances extracted from a
 *              Policy DTB.  A valid instance in the DTB is specified
 *              as "instName" where instName is the name of an RM
 *              instance.  Valid instances can be chained in the 
 *              Policy DTB with comma separation.
 *              Example:
 *                  <Rm_Client>,
 *                  <Rm_Server>;
 *                  Results in a list of two valid instances.
 */
Rm_PolicyValidInst *rmDtbUtilPolicyExtractValidInstances(const void *dtbDataPtr,
                                                         int32_t dtbDataLen,
                                                         int32_t *result)
{
    uint8_t            *dtbValidInstData = (uint8_t *)dtbDataPtr;
    uint32_t            instLenBytes;       
    Rm_PolicyValidInst *startInst = NULL;
    Rm_PolicyValidInst *newInst = NULL;
    Rm_PolicyValidInst *prevInst = NULL;
    int32_t             i = 0;

    /* Valid RM instances are stored in the DTB as a list of null-terminated
     * character strings. */
    while(i < dtbDataLen) {
        newInst = (Rm_PolicyValidInst *)Rm_osalMalloc(sizeof(Rm_PolicyValidInst));

        instLenBytes = strlen((char *) &dtbValidInstData[i]) + 1;
        if (instLenBytes > RM_NAME_MAX_CHARS) {
            Rm_osalFree((void *)newInst, sizeof(Rm_PolicyValidInst));
            while (startInst) {
                newInst = startInst->nextValidInst;
                Rm_osalFree((void *)startInst, sizeof(Rm_PolicyValidInst));
                startInst = newInst;
            }
            *result = RM_ERROR_VALID_INST_NAME_TOO_LONG;
            return(NULL);
        }
        rm_strncpy(newInst->instName, ((char *) &dtbValidInstData[i]),
                   instLenBytes);

        i += instLenBytes;

        newInst->nextValidInst = NULL;

        if (prevInst == NULL) {
            startInst = newInst;
        } else {
            prevInst->nextValidInst = newInst;
        }
        prevInst = newInst;
    }
    return(startInst);
}

/* FUNCTION PURPOSE: Deletes a valid instnace list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of valid
 *              instances that were extracted from a Policy DTB.
 */
void rmDtbUtilPolicyFreeValidInstances (Rm_PolicyValidInst *validInstList)
{
    Rm_PolicyValidInst *nextInst;

    while (validInstList) {
        nextInst = validInstList->nextValidInst;
        Rm_osalFree((void *)validInstList, sizeof(Rm_PolicyValidInst));
        validInstList = nextInst;
    }
}

/**********************************************************************
 ****************Linux DTB Parsing Defines and Functions***************
 **********************************************************************/

/* FUNCTION PURPOSE: Extracts a linux value list from a Linux DTB
 ***********************************************************************
 * DESCRIPTION: Returns a list of values extracted from a Linux
 *              DTB.  Linux value in the Linux DTB is specified
 *              as <w x y z ...> where w x y z ... are a list of
 *              numerical values.
 *              Example:
 *                  <1 5 10 15 20 25>;
 *                  Results in a list of six Linux values.
 */
Rm_LinuxValueRange *rmDtbUtilLinuxExtractValues(const void *dtbDataPtr,
                                                int32_t dtbDataLen)
{
    uint32_t           *dtbValueData = (uint32_t *)dtbDataPtr;
    Rm_LinuxValueRange *startValue = NULL;
    Rm_LinuxValueRange *newValue = NULL;
    Rm_LinuxValueRange *prevValue = NULL;
    uint32_t            i;

    /* Values are stored in the Linux DTB as a list of 32-bit words.  The
     * number of 32-bit words in the value field can differ.  depending on the
     * number of values specified. */
    for (i = 0; i < (dtbDataLen / sizeof(uint32_t)); i++) {
        newValue = (Rm_LinuxValueRange *)Rm_osalMalloc(sizeof(Rm_LinuxValueRange));
        /* Endianness of extracted value must be flipped */
        newValue->value = fdt32_to_cpu(dtbValueData[i]);
        newValue->nextValue = NULL;

        if (prevValue == NULL) {
            startValue = newValue;
        } else {
            prevValue->nextValue = newValue;
        }
        prevValue = newValue;
    }
    return(startValue);
}

/* FUNCTION PURPOSE: Deletes a Linux values list
 ***********************************************************************
 * DESCRIPTION: Frees the memory associated with a list of Linux
 *              values that were extracted from a Linux DTB.
 */
void rmDtbUtilLinuxFreeValues(Rm_LinuxValueRange *valueList)
{
    Rm_LinuxValueRange *nextValue;

    while (valueList) {
        nextValue = valueList->nextValue;
        Rm_osalFree((void *)valueList, sizeof(Rm_LinuxValueRange));
        valueList = nextValue;
    }
}

