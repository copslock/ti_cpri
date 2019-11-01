/*
 *  file  qmss_pvt.h
 *
 *  Private data structures of Queue Manager Low Level Driver.
 *
 *  ============================================================================
 *      (C) Copyright 2009-2014, Texas Instruments, Inc.
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

#ifndef QMSS_PVT_H_
#define QMSS_PVT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drv/qmss/qmss_qm.h>

/* RM includes */
#include <ti/drv/rm/rm_services.h>

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_qm_config.h>
#include <ti/csl/cslr_qm_descriptor_region_config.h>
#include <ti/csl/cslr_qm_queue_management.h>
#include <ti/csl/cslr_qm_queue_status_config.h>
#include <ti/csl/cslr_qm_intd.h>
#include <ti/csl/cslr_pdsp.h>

/* Maximum cache line size for alignment */
#define QMSS_MAX_CACHE_ALIGN 128

/* QMSS Global object definition. */
typedef struct 
{
    /** Store the configuration structure passed during Qmss_init */
    Qmss_GlobalConfigParams             qmssGblCfgParams;
    /** Store the intialization structure passed during Qmss_init */
    Qmss_InitCfg                        initCfg;
    /** Current Memory regions configuration */
    Qmss_MemRegInfo                     memRegInfo[QMSS_MAX_QMGR_GROUPS][QMSS_MAX_MEM_REGIONS];
    /** General purpose source queue handles */
    int32_t                             descQueue[QMSS_MAX_QMGR_GROUPS][QMSS_MAX_MEM_REGIONS];
    /** Current descriptor count */
    uint32_t                            currDescCnt;
    /** Operating mode : joined/split */
    Qmss_Mode                           mode;
} Qmss_GlobalObj_Unpadded;

typedef struct
{
    /** Data structure without padding, so sizeof() can compute padding */
    Qmss_GlobalObj_Unpadded obj[QMSS_MAX_SUBSYS];
    /** Pad out to end of QMSS_MAX_CACHE_ALIGN bytes to prevent something else
     * from being placed on same cache line as Cppi_Obj.  Note that 
     * pad[0] is illegal, so must add full QMSS_MAX_CACHE_ALIGN if structure
     * is already padded by chance. */
    uint8_t                 pad[QMSS_MAX_CACHE_ALIGN - 
                            ((QMSS_MAX_SUBSYS * sizeof(Qmss_GlobalObj_Unpadded)) % QMSS_MAX_CACHE_ALIGN)];
} Qmss_GlobalObj;

/** Alias type for local object parameters, so it can be extended beyond global config params,
 * if required. */
typedef Qmss_GlobalConfigParams Qmss_LocalObjParams;

typedef struct
{
    /** Store the configuration structure passed during Qmss_init */
    Qmss_LocalObjParams                 p;
    /** Operating mode : joined/split */
    Qmss_Mode                           mode;
} Qmss_LocalObj;

extern int Qmss_rmService (Rm_ServiceHandle *rmService, Rm_ServiceType type, const char *resName, 
                           int32_t *resNum, uint32_t resLen, int32_t resAlign, uint8_t *isAllocated);
extern int32_t Qmss_getMemRegQueueHandleSubSys (Qmss_SubSysHnd subSysHnd, uint32_t memRegion);
extern uint32_t Qmss_getMemRegDescSizeSubSys (Qmss_SubSysHnd subSysHnd, uint32_t memRegion, uint32_t qGroup);
extern void * Qmss_internalVirtToPhy (uint32_t subSys, void *addr);

/* Combines pdspId and channel together into an accumulator channel handle.  Application
 * SHALL NOT call this function.  Handle may be abstracted to a complex type at a later date
 */
static inline int32_t Qmss_accMakeHandle (uint32_t subSys, Qmss_PdspId pdspId, uint8_t channel)
{
    return (((int32_t)subSys) << 16) | ((((int32_t)pdspId) + 1) << 8) | (channel & 0xff);
} /* Qmss_accMakeHandle */

/* Splits pdsp channel handle into a pdspId and channel.  Application
 * SHALL NOT call this function.  Handle may be abstracted to a complex type at a later date
 */
static inline void Qmss_accBreakHandle (int32_t handle, uint32_t *subSys, Qmss_PdspId *pdspId, uint8_t *channel)
{
    *subSys = (uint32_t)((handle >> 16) & 0xff);
    *pdspId = (Qmss_PdspId)(((handle >> 8) - 1) & 0xff);
    *channel = handle & 0xff;
} /* Qmss_accBreakHandle */

/* This macro generates compilier error if postulate is false, so 
 * allows 0 overhead compile time size check.  This "works" when
 * the expression contains sizeof() which otherwise doesn't work
 * with preprocessor */
#define QMSS_COMPILE_TIME_SIZE_CHECK(postulate)                         \
   do {                                                                 \
       typedef struct {                                                 \
         uint8_t NegativeSizeIfPostulateFalse[((int)(postulate))*2 - 1];\
       } PostulateCheck_t;                                              \
   }                                                                    \
   while (0)

#ifdef __cplusplus
}
#endif

#endif /* QMSS_PVT_H_ */

