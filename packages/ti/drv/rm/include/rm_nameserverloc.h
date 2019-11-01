/*
 *  file  rm_nameserverloc.h
 *
 *  Prototypes and data structures for the Resource Manager NameServer.
 *
 *  ============================================================================
 *      (C) Copyright 2012-2013, Texas Instruments, Inc.
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

#ifndef RM_NAMESERVERLOC_H_
#define RM_NAMESERVERLOC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* RM external API includes */
#include <ti/drv/rm/rm.h>

/* RM internal includes */
#include <ti/drv/rm/include/rm_treeloc.h>

/**********************************************************************
 ********************* Internal NameServer APIs ***********************
 **********************************************************************/

/* NameServer configuration structure */
typedef struct {
    /* Pointer to the root entry of the NameServer tree structure */
    Rm_NameServerTree    *nameServerTree;
    /* NameServer tree node configuration struction */
    Rm_NameServerNodeCfg  nodeCfg;
} Rm_NameServerObjCfg;

int32_t rmNameServerAddObject(Rm_NameServerObjCfg *objCfg);
int32_t rmNameServerFindObject(Rm_NameServerObjCfg *objCfg);
int32_t rmNameServerDeleteObject(Rm_NameServerObjCfg *objCfg);
void rmNameServerPrintObjects(Rm_Handle rmHandle);
void rmNameServerInit(Rm_Handle rmHandle);
void rmNameServerDelete(Rm_Handle rmHandle);

#ifdef __cplusplus
}
#endif

#endif /* RM_NAMESERVERLOC_H_ */

