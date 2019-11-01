/*
 *  file  rm_internal.h
 *
 *  Data structures used by more than one internal RM module.
 *
 *  ============================================================================
 *      (C) Copyright 2012-2014, Texas Instruments, Inc.
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

#ifndef RM_INTERNAL_H_
#define RM_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes */
#include <string.h>

/* RM external includes */
#include <ti/drv/rm/rm.h>

/* RM OSAL layer */
#include <rm_osal.h>

/* RM true/false definitions */
#define RM_BOOL_UNDEF (-1)
#define RM_FALSE      0
#define RM_TRUE       1

/* RM cache alignment */
#define RM_MAX_CACHE_ALIGN 128

/* Shared server enter critical section macro */
#define RM_SS_INST_INV_ENTER_CS(rmInst, csKey)                  \
    if (rmInst->instType == Rm_instType_SHARED_SERVER) {        \
        csKey = Rm_osalCsEnter();                               \
        Rm_osalBeginMemAccess((void *)rmInst, sizeof(Rm_Inst)); \
    }

/* Shared client enter critical section macro */
#define RM_SC_INST_INV_ENTER_CS(rmInst, csKey)                                   \
    if (rmInst->instType == Rm_instType_SHARED_CLIENT) {                         \
        csKey = Rm_osalCsEnter();                                                \
        Rm_osalBeginMemAccess((void *)rmInst->u.sharedClient.sharedServerHandle, \
                              sizeof(Rm_Inst));                                  \
    }

/* Shared server exit critical section macro */
#define RM_SS_INST_WB_EXIT_CS(rmInst, csKey)                  \
    if (rmInst->instType == Rm_instType_SHARED_SERVER) {      \
        Rm_osalEndMemAccess((void *)rmInst, sizeof(Rm_Inst)); \
        Rm_osalCsExit(csKey);                                 \
    }

/* Shared server invalidate macro */
#define RM_SS_OBJ_INV(rmInst, obj, objType)                  \
    if (rmInst->instType == Rm_instType_SHARED_SERVER) {     \
        Rm_osalBeginMemAccess((void *)obj, sizeof(objType)); \
    }

/* Shared server writeback macro */
#define RM_SS_OBJ_WB(rmInst, obj, objType)                 \
    if (rmInst->instType == Rm_instType_SHARED_SERVER) {   \
        Rm_osalEndMemAccess((void *)obj, sizeof(objType)); \
    }

/* RM resource information */
typedef struct {
    /* Resource name */
    char     name[RM_NAME_MAX_CHARS];
    /* Resource base value */
    int32_t  base;
    /* Resource length value */
    uint32_t length;
    /* Resource alignment */
    int32_t  alignment;
    /* Resource owner count - number of instances that are in the
     * resource's owner list */
    int32_t  ownerCount;
    /* Requesting instance's allocation count for the resource */
    int32_t  instAllocCount;
    /* NameServer name tied to resource */
    char     nameServerName[RM_NAME_MAX_CHARS];
} Rm_ResourceInfo;

/* wrapper to strncpy to avoid coverity overrun warning */
static inline void rm_strncpy (char *dst, const char *src, size_t len)
{
    strncpy (dst, src, len);
    dst[len - 1] = 0;
}

#ifdef __cplusplus
}
#endif

#endif /* RM_INTERNAL_H_ */

