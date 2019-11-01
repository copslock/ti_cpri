/**
 *   @file  qmss_drv.c
 *
 *   @brief
 *      This is the Queue Manager Low Level Driver file.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2016, Texas Instruments, Inc.
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

/* QMSS Types includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/include/qmss_pvt.h>
#include <ti/drv/qmss/include/qmss_intd.h>

/* QMSS OSAL layer */
#include <qmss_osal.h>

/* RM includes */
#include <ti/drv/rm/rm_services.h>

/**
 * @mainpage
 *
 * The QMSS LLD supplies APIs to manage the Queue Manager subsystems
 * within Keystone SoCs.  It provides configuration APIs which typically
 * involve @ref Qmss_initSubSys (or @ref Qmss_init) on a master core,
 * and @ref Qmss_startSubSysCfg (or @ref Qmss_start or @ref Qmss_startCfg)
 * on the other cores.  @ref Qmss_initSubSys and @ref Qmss_startSubSysCfg
 * are run once per subsystem on the device, and return the
 * @ref Qmss_SubSysHnd used to refer to the subsystem in other APIs.
 *
 * Next, descriptor regions are created using
 * @ref Qmss_insertMemoryRegionSubSys (or @ref Qmss_insertMemoryRegion).
 *
 * Next, descriptors are set up for DMAs using Cppi_initDescriptor or can
 * be used as-is for software use only.
 *
 * Single queues are allocated using @ref Qmss_queueOpenSubSys
 * (or @ref Qmss_queueOpen).  Contiguous blocks of queues are opened
 * with @ref Qmss_queueBlockOpenSubSys (or @ref Qmss_queueBlockOpen).
 *
 * Data transactions with virtual/physical address translation is
 * are performed using @ref Qmss_queuePush, @ref Qmss_queuePushDesc,
 * @ref Qmss_queuePushDescSize, @ref Qmss_queuePop,
 * and @ref Qmss_queuePopDescSize) depending whether sizes of the
 * descriptors or payloads need to be set.
 *
 * Data transactions that don't have virtual/physical address translation
 * required is performed through @ref Qmss_queuePopRaw and
 * @ref Qmss_queuePushDescSizeRaw.
 *
 * Finally the QMSS LLD supplies APIs to control firmware running
 * on PDSPs that support features like interrupt accumulation (see
 * qmss_acc.h) and traffic shaping (QoS) (see @ref qmss_qosSched.h
 * @ref qmss_qos.h).
 *
 * Note on backwards/forwards compatibility strategy.
 *
 * All APIs which contain "SubSys" in the name can address all QMSS
 * subsystems on the device.  Those APIs which do not have "SubSys" in the
 * name nor take a @ref Qmss_SubSysHnd nor a @ref Qmss_QueueHnd
 * can only address the global QMSS subsystem on the device.
 *
 * There is no plan to deprecate the versions without "SubSys" in the
 * name.
 */

/**********************************************************************
 ************************** Globals ***********************************
 **********************************************************************/

/* Maintain status of queues */
#ifdef _TMS320C6X
#pragma DATA_SECTION (qmssQueueFreeSS0, ".qmss");
#pragma DATA_ALIGN (qmssQueueFreeSS0, QMSS_MAX_CACHE_ALIGN)
#pragma DATA_SECTION (qmssQueueFreeSS1, ".qmss");
#pragma DATA_ALIGN (qmssQueueFreeSS1, QMSS_MAX_CACHE_ALIGN)
#endif
static uint8_t       qmssQueueFreeSS0[QMSS_MAX_QUEUES_SS_0];
#if (QMSS_MAX_QUEUES_SS_0 % QMSS_MAX_CACHE_ALIGN)
#error QMSS_MAX_QUEUES_SS_0 results in unpadded structure
#endif
static uint8_t       qmssQueueFreeSS1[QMSS_MAX_QUEUES_SS_1];
#if (QMSS_MAX_QUEUES_SS_1 % QMSS_MAX_CACHE_ALIGN)
#error QMSS_MAX_QUEUES_SS_1 results in unpadded structure
#endif

uint8_t *const qmssQueueFree[QMSS_MAX_SUBSYS] =
{
    qmssQueueFreeSS0,
    qmssQueueFreeSS1
};

const uint32_t qmssQueueFreeSize[QMSS_MAX_SUBSYS] =
{
    sizeof(qmssQueueFreeSS0),
    sizeof(qmssQueueFreeSS1)
};

/* It is assumed QMSS_MAX_QUEUES is divisible by 128, so no padding is needed */

#ifdef _TMS320C6X
#pragma DATA_SECTION (qmssGroupControl, ".qmss");
#pragma DATA_ALIGN (qmssGroupControl, QMSS_MAX_CACHE_ALIGN)
#endif
struct {
    /** Used to do load balanced allocation */
    int32_t groupFreeQueues[QMSS_MAX_QMGR_GROUPS];
    /** Used to perform round robin allocation */
    uint32_t alternate;
    /** Padding required to make sure linker doesn't put something else
     * on the same cache line.
     */
    uint8_t pad[QMSS_MAX_CACHE_ALIGN - sizeof(uint32_t) * (QMSS_MAX_QMGR_GROUPS + 1)];
} qmssGroupControl[QMSS_MAX_SUBSYS];

/* QMSS Global object */
#ifdef _TMS320C6X
#pragma DATA_SECTION (qmssGObjIsValid, ".qmss");
#pragma DATA_ALIGN (qmssGObjIsValid, QMSS_MAX_CACHE_ALIGN)
#pragma DATA_SECTION (qmssGObj, ".qmss");
#pragma DATA_ALIGN (qmssGObj, QMSS_MAX_CACHE_ALIGN)
/* force these two to a local memory when default is shared */
#pragma DATA_SECTION (qmssLObj, ".far:local");
#pragma DATA_SECTION (qmssLObjIsValid, ".far:local");
#endif
Qmss_GlobalObj              qmssGObj;

/* QMSS Local object */
Qmss_LocalObj               qmssLObj[QMSS_MAX_SUBSYS];

/* QMSS Global Object is valid flag */
uint8_t                     qmssGObjIsValid[QMSS_MAX_SUBSYS] = { 0, 0 };

/* QMSS Local Object is valid flag */
uint8_t                     qmssLObjIsValid[QMSS_MAX_SUBSYS] = { 0, 0 };

/** @brief Global Variable which describes the QMSS LLD Version Information */
const char   qmssLldVersionStr[] = QMSS_LLD_VERSION_STR ":" __DATE__  ":" __TIME__;

/**********************************************************************
 ************************** Functions *********************************
 **********************************************************************/

static Qmss_QueueHnd Qmss_internalQueueOpen (Qmss_SubSysHnd subSysHnd, int qGroupIsSpecified, uint32_t qGroup, Qmss_QueueType queType, int queNum, uint8_t *isAllocated);
static Qmss_Result Qmss_internaldownloadFirmware (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, void *image, uint32_t size);
Qmss_Result Qmss_internalQueueClose (Qmss_QueueHnd hnd);

/** @addtogroup QMSS_LLD_FUNCTION
@{
*/

/**
 *  @internal
 *  @b Description
 *  @n
 *      Internal function get the maxQueueNum index for a que number on a particulare
 *      subsystem.
 */
static int32_t Qmss_internalFindMaxQueNumIdxByQue (
    Qmss_SubSysHnd subSysHnd,
    uint32_t queNum)
{
    uint32_t maxQueNumIdx;
    uint32_t queTypeStart, queTypeEnd;
    uint32_t qGroup = QMSS_QUEUE_GROUP(queNum);
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_QueueNumRange *maxQueueNumP = qmssLObj[subSys].p.maxQueueNum[qGroup];

    for (maxQueNumIdx = 0; maxQueNumIdx < qmssLObj[subSys].p.numQueueNum[qGroup]; maxQueNumIdx++)
    {
        /* Ignore queue types that don't exist for a queue group.  These queue types will have a startIndex
         * and maxNum of zero */
        if ((maxQueueNumP[maxQueNumIdx].startIndex != 0) ||
            (maxQueueNumP[maxQueNumIdx].maxNum != 0))
        {
            queTypeStart = maxQueueNumP[maxQueNumIdx].startIndex;
            queTypeEnd = maxQueueNumP[maxQueNumIdx].startIndex +
                         maxQueueNumP[maxQueNumIdx].maxNum - 1;

            if ((queNum >= queTypeStart) && (queNum <= queTypeEnd))
            {
                return maxQueNumIdx;
            }
        }
    }
    return -1;
} /* Qmss_internalFindMaxQueNumIdxByQue */

/**
 *  @internal
 *  @b Description
 *  @n
 *      Internal function get the maxQueueNum index for a queue type in a queue group
 *      on a particular subsystem.
 */
static int32_t Qmss_internalFindMaxQueNumIdxByType (
    Qmss_SubSysHnd subSysHnd,
    Qmss_QueueType type,
    int32_t        qGroup,
    int32_t        startIdx)
{
    uint32_t maxQueNumIdx;
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_QueueNumRange *maxQueueNumP = qmssLObj[subSys].p.maxQueueNum[qGroup];

    for (maxQueNumIdx = startIdx; maxQueNumIdx < qmssLObj[subSys].p.numQueueNum[qGroup]; maxQueNumIdx++)
    {
        /* Ignore queue types that don't exist for a queue group.  These queue types will have a startIndex
         * and maxNum of zero */
        if ((maxQueueNumP[maxQueNumIdx].startIndex != 0) ||
            (maxQueueNumP[maxQueNumIdx].maxNum != 0))
        {
            if (maxQueueNumP[maxQueNumIdx].queueType == type)
            {
                return maxQueNumIdx;
            }
        }
    }
    return -1;
} /* Qmss_internalFindMaxQueNumIdxByType */

/**
 *  @internal
 *  @b Description
 *  @n
 *      Internal function wrapper to do virt2phy for base addresses
 *      (not descriptors)
 */
void * Qmss_internalVirtToPhy (uint32_t subSys, void *addr)
{
   void *result = addr;

   if (qmssLObj[subSys].p.virt2Phy)
   {
       result = qmssLObj[subSys].p.virt2Phy (addr);
   }

   return result;
}

/**
 *  @internal
 *  @b Description
 *  @n
 *      Internal function wrapper to do phy2virt for base addresses
 *      (not descriptors)
 */
void * Qmss_internalPhyToVirt (uint32_t subSys, void *addr)
{
   void *result = addr;

   if (qmssLObj[subSys].p.phy2Virt)
   {
       result = qmssLObj[subSys].p.phy2Virt (addr);
   }

   return result;
}

/**
 *  @internal
 *  @b Description
 *  @n
 *      Internal function to increment or decrement the number of queues open
 *      in each group.
 */
static void Qmss_internalUpdateGroupControl (uint32_t subSys, uint32_t qGroup, int sign)
{
    /* Invalidate the qmssGroupControl structure */
    Qmss_osalBeginMemAccess ((void *) &qmssGroupControl[subSys], sizeof(qmssGroupControl[subSys]));

    /* One fewer free queues in group */
    qmssGroupControl[subSys].groupFreeQueues[qGroup] += sign;

    /* Writeback qmssGroupControl structure */
    Qmss_osalEndMemAccess ((void *) &qmssGroupControl[subSys], sizeof(qmssGroupControl[subSys]));
} /* Qmss_internalUpdateGroupControl */


/**
 *  @internal
 *  @b Description
 *  @n
 *      Internal function to program linking RAM configuration for each
 *      queue manager group
 */

static void Qmss_internalInsertLinkingRAM (
    uint32_t         subSys,
    uint32_t         group,
    Qmss_LinkingRAM *linkingRAM,
    uint32_t        *actSize)
{
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;
    uint32_t size;

    if (linkingRAM->linkingRAM0Base == 0)
    {
        lObjPtr->groupRegs[group].qmConfigReg->LINKING_RAM_REGION_0_BASE_ADDRESS_REG =
           (uint32_t)lObjPtr->regs.qmLinkingRAMReg - (uint32_t)lObjPtr->regs.qmBaseAddr;
    }
    else
    {
        lObjPtr->groupRegs[group].qmConfigReg->LINKING_RAM_REGION_0_BASE_ADDRESS_REG =
            (uint32_t)Qmss_internalVirtToPhy(subSys, (void *)linkingRAM->linkingRAM0Base);
    }

    if (linkingRAM->linkingRAM0Size == 0)
    {
        size = *actSize = lObjPtr->intLinkRamSize - 1;
        lObjPtr->groupRegs[group].qmConfigReg->LINKING_RAM_REGION_0_SIZE_REG = size;
    }
    else
        lObjPtr->groupRegs[group].qmConfigReg->LINKING_RAM_REGION_0_SIZE_REG = linkingRAM->linkingRAM0Size;

    lObjPtr->groupRegs[group].qmConfigReg->LINKING_RAM_REGION_1_BASE_ADDRESS_REG =
        (uint32_t)Qmss_internalVirtToPhy(subSys, (void *)linkingRAM->linkingRAM1Base);
} /* Qmss_internalInsertLinkingRAM */

/**
 *  @internal
 *  @b Description
 *  @n
 *      Internal function to validate linking RAM configuration for each
 *      queue manager group
 *
 *  @retval
 *      Success -   QMSS_SOK
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 */

static Qmss_Result Qmss_internalCheckLinkingRAM (Qmss_LocalObjParams *lObjPtr, Qmss_LinkingRAM *linkingRAM)
{
    /* Check if LinkingRAM0 can hold all the descriptors if LinkingRAM1 is NULL */
    if (linkingRAM->linkingRAM1Base == 0)
    {
        if (linkingRAM->maxDescNum > lObjPtr->intLinkRamSize)
        {
            return QMSS_INVALID_PARAM;
        }
    }
    return QMSS_SOK;
} /* Qmss_internalCheckLinkingRAM */

/*
 *  @b Description
 *  @n
 *      This function sends a name server request to RM
 *
 *  @param[in]  rmService
 *      rm service handle
 *
 *  @param[in]  type
 *      Service request type (Rm_service_RESOURCE_GET_BY_NAME,
 *      Rm_service_RESOURCE_MAP_TO_NAME,
 *      Rm_service_RESOURCE_UNMAP_NAME
 *
 *  @param[in]  nsName
 *      Pointer to the RM name server name
 *  @param[in]  val
 *      Pointer to integer paired with name
 *
 *  @retval
 *      1 - RM approved request.
 *      0 - RM denied request or experienced an error
 */
static int Qmss_rmNameService (Rm_ServiceHandle *rmService, Rm_ServiceType type,
                               const char *nsName, int32_t *val)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    int                 succeed;

    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    rmServiceReq.type             = type;
    rmServiceReq.resourceName     = nsName;
    rmServiceReq.resourceNsName   = nsName;
    rmServiceReq.resourceLength   = 1;
    if (val)
    {
        rmServiceReq.resourceBase = *val;
    }

    /* RM will block until resource is returned since callback is NULL */
    rmServiceReq.callback.serviceCallback = NULL;
    rmService->Rm_serviceHandler(rmService->rmHandle, &rmServiceReq, &rmServiceResp);
    succeed = (rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
              (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC);

    if (succeed)
    {
        if ((type == Rm_service_RESOURCE_GET_BY_NAME) && (val))
        {
            *val = rmServiceResp.resourceBase;
        }
    }

    return succeed;
} /* Qmss_rmNameService */

/*
 *  @b Description
 *  @n
 *      This function sets a name/value pair
 *
 *  @param[in]  rmService
 *      rm service handle
 *  @param[in]  nsName
 *      Pointer to the RM name server name
 *  @param[in]  val
 *      integer to be paired with name
 *
 *  @retval
 *      1 - RM approved request.
 *      0 - RM denied request or experienced an error
 */
static int Qmss_nameServiceSet (Rm_ServiceHandle *rmService, const char *nsName, int32_t val)
{
    return Qmss_rmNameService (rmService, Rm_service_RESOURCE_MAP_TO_NAME, nsName, &val);
} /* Qmss_nameServiceSet */

/*
 *  @b Description
 *  @n
 *      This function gets value associated with name
 *
 *  @param[in]  rmService
 *      rm service handle
 *  @param[in]  nsName
 *      Pointer to the RM name server name
 *  @param[in]  val
 *      Pointer to integer result
 *
 *  @retval
 *      1 - RM approved request.
 *      0 - RM denied request or experienced an error
 */
static int Qmss_nameServiceGet (Rm_ServiceHandle *rmService, const char *nsName, int32_t *val)
{
    return Qmss_rmNameService (rmService, Rm_service_RESOURCE_GET_BY_NAME, nsName, val);
} /* Qmss_nameServiceGet */

/*
 *  @b Description
 *  @n
 *      This function deletes a name/value pair
 *
 *  @param[in]  rmService
 *      rm service handle
 *  @param[in]  nsName
 *      Pointer to the RM name server name
 *
 *  @retval
 *      1 - RM approved request.
 *      0 - RM denied request or experienced an error
 */
static int Qmss_nameServiceDel (Rm_ServiceHandle *rmService, const char *nsName)
{
    return Qmss_rmNameService (rmService, Rm_service_RESOURCE_UNMAP_NAME, nsName, NULL);
} /* Qmss_nameServiceDel */

/*
 *  @b Description
 *  @n
 *      This function sends a service (alloc/free) request to RM
 *
 *  @param[in]  rmService
 *      rm service handle
 *  @param[in]  type
 *      Service request type
 *  @param[in]  resName
 *      Pointer to the RM resource name
 *  @param[out]  resNum
 *      Pointer to the resource value to check.  If the input value is
 *      unspecified the resource value returned by RM will be returned
 *      through this parameter.
 *  @param[in]  resLen
 *      The resource length (starting from resNum) to check.
 *  @param[out] isAllocated
 *      Returns the reference count for the resource
 *
 *  @retval
 *      1 - RM approved request.
 *      0 - RM denied request or experienced an error
 */
int Qmss_rmService (Rm_ServiceHandle *rmService, Rm_ServiceType type, const char *resName,
                    int32_t *resNum, uint32_t resLen, int32_t resAlign, uint8_t *isAllocated)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    int                 retVal = 0;

    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    rmServiceReq.type = type;
    rmServiceReq.resourceName = resName;
    if (*resNum >= 0)
    {
        rmServiceReq.resourceBase = *resNum;
    }
    else
    {
        rmServiceReq.resourceBase = RM_RESOURCE_BASE_UNSPECIFIED;
    }
    rmServiceReq.resourceLength = resLen;
    rmServiceReq.resourceAlignment = resAlign;
    /* RM will block until resource is returned since callback is NULL */
    rmServiceReq.callback.serviceCallback = NULL;
    rmService->Rm_serviceHandler(rmService->rmHandle, &rmServiceReq, &rmServiceResp);
    if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
        (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        /* Only return value through resNum if the service type is
         * ALLOCATE... */
        if ((type == Rm_service_RESOURCE_ALLOCATE_INIT) ||
            (type == Rm_service_RESOURCE_ALLOCATE_USE))
        {
            *resNum = rmServiceResp.resourceBase;
        }
        retVal = 1;
    }

    if (isAllocated)
    {
        if (rmServiceResp.resourceNumOwners == RM_RESOURCE_NUM_OWNERS_INVALID)
        {
            *isAllocated = 0;
        }
        else
        {
            /* QM would like to have the global allocation count, but RM doesn't provide that.
             * Most use cases rely on 0, 1, or 2 (last instance closed, first instance opened,
             * or subsequent instance opened.
             * The following logic will always return 0,1, or 2 correctly, but values >=2
             * only mean that this is not first open request
             */
            if ((rmServiceResp.resourceNumOwners == 1) &&
                (rmServiceResp.instAllocCount != RM_INST_ALLOC_COUNT_INVALID) &&
                (rmServiceResp.instAllocCount >= 1))
            {
                /* If this is only owner, return how many times it was opened on this instance; */
                *isAllocated = rmServiceResp.instAllocCount;
            }
            else
            {
                /* If there are multiple owners or no owners, return the owner count */
                *isAllocated = rmServiceResp.resourceNumOwners;
            }
            /* Error or denial occurred in RM but not total failure since refCount was returned */
            retVal = 1;
        }
    }
    return (retVal);
} /* Qmss_rmService */

/*
 *  @b Description
 *  @n
 *      This function takes or frees startIndex from rm
 *
 *  @param[in]  lObjPtr
 *      subsuystem specific local opject pointer
 *  @param[in]     qGroup
 *      queue group within subsystem to allocate from
 *  @param[inout]  startIndex_p
 *      pointer to proposed startIndex
 *  @param[in]  numDesc
 *      number of descriptors to allocate from linking ram
 *  @param[in]  isTake
 *      nonzero: take the resource
 *      0: release the resource
 *  @param[out] isAllocated
 *      1 is first allocation (initialized); 2+ is second (user)
 *
 *  @retval
 *      QMSS_SOK: no error
 *      QMSS_RESOURCE_LINKING_RAM_INIT_DENIED: RM failed to allocate
 *      Rm_service_RESOURCE_FREE: RM failed to free (on isTake==0) or double error (isTake==1)
 */
Qmss_Result Qmss_internalRmStartIndex (Qmss_LocalObjParams *lObjPtr, uint32_t qGroup, int32_t *startIndex_p, uint32_t numDesc, int isTake, uint8_t *isAllocated)
{
    Qmss_Result result = QMSS_SOK;
    uint32_t reg0Descs = lObjPtr->groupRegs[qGroup].qmConfigReg->LINKING_RAM_REGION_0_SIZE_REG + 1;
    int32_t  region0Base;
    int32_t  region1Base;
    uint32_t region0NumDesc = 0;
    uint32_t region1NumDesc = 0;
    int32_t  tryBoth        = 0;
    int32_t  startIndex     = *startIndex_p;
    Rm_ServiceType rmType;

    if (startIndex == QMSS_START_INDEX_NOT_SPECIFIED)
    {
        tryBoth        = 1; /* do trial and error in both regions */
        region0Base    = QMSS_PARAM_NOT_SPECIFIED;
        region0NumDesc = numDesc;
    }
    else if (startIndex == QMSS_START_INDEX_INTERNAL)
    {
        region0Base    = QMSS_PARAM_NOT_SPECIFIED;
        region0NumDesc = numDesc;
    }
    else if (startIndex == QMSS_START_INDEX_EXTERNAL)
    {
        region1Base    = QMSS_PARAM_NOT_SPECIFIED;
        region1NumDesc = numDesc;
    }
    else if (startIndex >= 0)
    {
        int32_t endIndex = startIndex + numDesc;
        if (startIndex < (int32_t) reg0Descs)
        {
            /* At least some are in internal */
            region0Base    = startIndex;
            region0NumDesc = numDesc;
        }
        if (endIndex > (int32_t) reg0Descs)
        {
            /* At least some are in external */
            region1Base    = startIndex;
            region1NumDesc = numDesc;
        }
        if (region0NumDesc && region1NumDesc)
        {
            /* some are in both internal and external */
            region0NumDesc = reg0Descs - region0Base;
            region1NumDesc = numDesc - region0NumDesc;
            region1Base    = reg0Descs;
        }
    }
    else
    {
        return QMSS_INVALID_PARAM;
    }
    rmType = isTake ? Rm_service_RESOURCE_ALLOCATE_INIT : Rm_service_RESOURCE_FREE;
    if (region0NumDesc)
    {
        if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, rmType,
                            lObjPtr->groupRm[qGroup].rmLinkRamInt, &region0Base, region0NumDesc, 0, isAllocated))
        {
            if (tryBoth)
            {
                /* try again in region 1 */
                region1Base    = QMSS_PARAM_NOT_SPECIFIED;
                region1NumDesc = region0NumDesc;
                region0NumDesc = 0;
            }
            else
            {
                /* failure */
                return (isTake ? QMSS_RESOURCE_LINKING_RAM_INIT_DENIED : QMSS_RESOURCE_FREE_DENIED);
            }
        }
        /* Assume this result is return value */
        *startIndex_p = region0Base;
    }
    if (region1NumDesc)
    {
        if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, rmType,
                            lObjPtr->groupRm[qGroup].rmLinkRamExt, &region1Base, region1NumDesc, 0, isAllocated))
        {
            /* failure */
            result = (isTake ? QMSS_RESOURCE_LINKING_RAM_INIT_DENIED : QMSS_RESOURCE_FREE_DENIED);
            /* return region 0 if taken */
            if (region0NumDesc && isTake)
            {
                if (! Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                     lObjPtr->groupRm[qGroup].rmLinkRamInt, &region0Base, region0NumDesc, 0, isAllocated))
                {
                    result = QMSS_RESOURCE_FREE_DENIED; /* double error */
                }
            }
        }
        else if (! region0NumDesc)
        {
            /* Only region - return the result */
            *startIndex_p = region1Base;
        }
    }

    return result;
} /* Qmss_internalRmStartIndex */

/*
 *  @b Description
 *  @n
 *      This function sends a service request to RM for each group supported by the QM
 *
 *  @param[in]  type
 *      Service request type
 *  @param[in]  nameOffset
 *      Offset to name pointer in Qmss_GlobalConfigGroupRm
 *  @param[out]  resNum
 *      Pointer to the resource value to check.  If the input value is
 *      unspecified the resource value returned by RM will be returned
 *      through this parameter.
 *  @param[in]  resLen
 *      The resource length (starting from resNum) to check.
 *  @param[out] isAllocated
 *      Returns the reference count for the resource
 *  @param[in]  qmssGblCfgParams
 *      Structure describing groups
 *
 *  @retval
 *      1 - RM approved request.
 *      0 - RM denied request or experienced an error
 */
int Qmss_rmServiceGroups (Rm_ServiceHandle *rmService, Rm_ServiceType type, size_t nameOffset,
                          int32_t *resNum, uint32_t resLen, int32_t resAlign, uint8_t *isAllocated,
                          Qmss_GlobalConfigParams *qmssGblCfgParams)
{
    int   qm, qm2;
    char *namePtr;

    /* Attempt to allocate control of all the QMs.  Right now both JOINT and SPLIT modes use all QMs */
    for (qm = 0; qm < (int) qmssGblCfgParams->maxQueMgrGroups; qm++)
    {
        namePtr = ((char *)&qmssGblCfgParams->groupRm[qm]) + nameOffset;
        if ((!Qmss_rmService((Rm_ServiceHandle *)qmssGblCfgParams->qmRmServiceHandle, type,
                             namePtr, resNum, resLen, resAlign, isAllocated)))
        {
            /* Free any partially allocated resourcse */
            if (type != Rm_service_RESOURCE_FREE)
            {
                for (qm2 = qm - 1; qm2 >= 0; qm2--)
                {
                    namePtr = ((char *)&qmssGblCfgParams->groupRm[qm]) + nameOffset;
                    Qmss_rmService((Rm_ServiceHandle *)qmssGblCfgParams->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                   namePtr, resNum, resLen, resAlign, isAllocated);
                }
            }

            return 0;
        }
    }

    return 1;
}

/**
 *  @b Description
 *  @n
 *      This function initializes the Queue Manager subsystem low level driver
 *      This function is called once in the system to setup the queue manager
 *      low level driver with information pertaining to linking RAM and descriptor
 *      memory region.
 *
 *      The @ref Qmss_init API can also be used to initialize the global QMSS
 *      subsystem and is backwards compatible with previous releases.
 *
 *  @param[out] subSysHnd
 *      Handle to the subsystem used to designate subystem to other APIs such as
 *      @ref Qmss_queueOpenSubSys
 *
 *  @param[in]  subSys
 *      Enum identification of the subsystem to identify.
 *
 *  @param[in]  initCfg
 *      Initialization structure that contains memory region configuration information.
 *      Linking RAM memory address should be a global address
 *
 *  @param[in]  qmssGblCfgParams
 *      Initialization structure that contains the QMSS device specific information.
 *
 *  @post
 *      QMSS instance is created.
 *
 *  @retval
 *      Success -   QMSS_SOK
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 *  @retval
 *      Failure -   QMSS_RESOURCE_LINKING_RAM_INIT_DENIED
 *  @retval
 *      Failure -   QMSS_QM_CONTROL_DENIED
 */
Qmss_Result Qmss_initSubSys (Qmss_SubSysHnd *subSysHnd, Qmss_SubSys subSys, Qmss_InitCfg *initCfg, Qmss_GlobalConfigParams *qmssGblCfgParams)
{
    Qmss_GlobalObj_Unpadded *gObjPtr = &qmssGObj.obj[subSys];
    Qmss_LocalObjParams     *lObjPtr = &qmssLObj[subSys].p;
    uint32_t        index;
    void            *key;
    uint32_t        qGroup;
    uint32_t        qPerGroup;
    Qmss_LinkingRAM linkingRAMCfg;
    Qmss_Result     retVal = QMSS_SOK;
    Qmss_StartCfg   startCfg;

    if (subSys >= QMSS_MAX_SUBSYS)
    {
        return QMSS_INVALID_PARAM;
    }

    if (initCfg == NULL)
    {
        return QMSS_INVALID_PARAM;
    }

    if (qmssGblCfgParams->maxPDSP > QMSS_MAX_PDSP)
    {
        return QMSS_INVALID_PARAM;
    }

    if (qmssGblCfgParams->maxQueMgrGroups > QMSS_MAX_QMGR_GROUPS)
    {
        return QMSS_INVALID_PARAM;
    }

    for (index = 0; index < qmssGblCfgParams->maxPDSP; index++)
    {
        if ((initCfg->pdspFirmware[index].firmware != NULL) && (initCfg->pdspFirmware[index].size == 0))
            return QMSS_INVALID_PARAM;
    }

    if (qmssGblCfgParams->qmRmServiceHandle)
    {
        int32_t qmControl = 0;

        /* Attempt to allocate control of all the QMs.  Right now both JOINT and SPLIT modes use all QMs */
        if ((!Qmss_rmServiceGroups((Rm_ServiceHandle *)qmssGblCfgParams->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_INIT,
                                   offsetof(Qmss_GlobalConfigGroupRm, rmQmControl), &qmControl, 1, 0, NULL, qmssGblCfgParams)))
        {
            return QMSS_QM_CONTROL_DENIED;
        }
    }

    /* Check the max queue number configuration table(s) */
    qPerGroup = qmssGblCfgParams->maxQue / qmssGblCfgParams->maxQueMgrGroups;
    for (qGroup = 0; qGroup < qmssGblCfgParams->maxQueMgrGroups; qGroup++)
    {
        for (index = 0; index < qmssGblCfgParams->numQueueNum[qGroup]; index++)
        {
             uint32_t        startIndex = qmssGblCfgParams->maxQueueNum[qGroup][index].startIndex;
             /* Check that the queue index matches the queue group */
             if (startIndex && ((startIndex < (qPerGroup * qGroup)) ||
                                (startIndex >= (qPerGroup * (qGroup + 1)))))
             {
                 return QMSS_INVALID_QUEUE_NUMBER_TAB;
             }
        }
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    /* Initialize private data structures */
    memset ((void *) qmssQueueFree[subSys], 0, qmssQueueFreeSize[subSys]);
    Qmss_osalEndMemAccess ((void *) qmssQueueFree[subSys], qmssQueueFreeSize[subSys]);
    for (qGroup = 0; qGroup < qmssGblCfgParams->maxQueMgrGroups; qGroup++)
    {
        qmssGroupControl[subSys].groupFreeQueues[qGroup] = qPerGroup;
    }
    qmssGroupControl[subSys].alternate = 0;

    Qmss_osalEndMemAccess ((void *)&qmssGroupControl[subSys], sizeof (qmssGroupControl[subSys]));

    memset (gObjPtr, 0, sizeof (Qmss_GlobalObj_Unpadded));
    for (qGroup = 0; qGroup < qmssGblCfgParams->maxQueMgrGroups; qGroup++)
    {
        for (index = 0; index < QMSS_MAX_MEM_REGIONS; index++)
        {
            gObjPtr->descQueue[qGroup][index] = -1;
        }
    }

    /* Store a local copy of the initial memory region configuration */
    memcpy ((void *) &gObjPtr->initCfg, initCfg, sizeof (Qmss_InitCfg));

    linkingRAMCfg.linkingRAM0Base = initCfg->linkingRAM0Base;
    linkingRAMCfg.linkingRAM0Size = initCfg->linkingRAM0Size;
    linkingRAMCfg.linkingRAM1Base = initCfg->linkingRAM1Base;
    linkingRAMCfg.maxDescNum      = initCfg->maxDescNum;

    if (initCfg->qmssHwStatus != QMSS_HW_INIT_COMPLETE)
    {
        /* Check the linking RAM config */
        if ((retVal = Qmss_internalCheckLinkingRAM (qmssGblCfgParams, &linkingRAMCfg)) != QMSS_SOK)
        {
           goto exitCs;
        }

        /* If split mode check the rest of the linking RAM */
        if (initCfg->mode == Qmss_Mode_SPLIT)
        {
            for (qGroup = 1; qGroup < qmssGblCfgParams->maxQueMgrGroups; qGroup++)
            {
                if ((retVal = Qmss_internalCheckLinkingRAM (qmssGblCfgParams, initCfg->splitLinkingRAMs + qGroup)) != QMSS_SOK)
                {
                   goto exitCs;
                }
            }
        }
    }


    /* Copy global configuration parameters */
    memcpy ((void *) &gObjPtr->qmssGblCfgParams, qmssGblCfgParams, sizeof (Qmss_GlobalConfigParams));
    /* Writeback now, as Qmss_startSubSysCfg will invalidate for external callability */
    Qmss_osalEndMemAccess ((void *) gObjPtr, sizeof (*gObjPtr));
    /* Mark global object valid valid */
    qmssGObjIsValid[subSys] = 1;
    Qmss_osalEndMemAccess ((void *) &qmssGObjIsValid[subSys], sizeof (qmssGObjIsValid[subSys]));

    startCfg.rmServiceHandle = qmssGblCfgParams->qmRmServiceHandle;
    startCfg.pQmssGblCfgParams = NULL;
    Qmss_startSubSysCfg (subSysHnd, subSys, &startCfg);

    if (lObjPtr->qmRmServiceHandle && (gObjPtr->initCfg.qmssHwStatus != QMSS_HW_INIT_COMPLETE))
    {
        /* Check Linking RAM 0 and 1 control permissions */
        int32_t linkRamControl = 0;
        /* Attempt to allocate control of all the QMs.  Right now both JOINT and SPLIT modes use all QMs */
        if ((!Qmss_rmServiceGroups((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_INIT,
                                   offsetof(Qmss_GlobalConfigGroupRm,rmLinkRamControl),
                                   &linkRamControl, 1, 0, NULL, qmssGblCfgParams)))
        {
            /* Set the return value to signify setting base address registers was denied and set the
             * HW_INIT_COMPLETE to be true in the global config.  This signifies another entity has
             * configured the QMSS and some entries of the qmssGObj may not be valid, such as
             * the initCfg.maxDescNum field */
            retVal = QMSS_RESOURCE_LINKING_RAM_INIT_DENIED;
            gObjPtr->initCfg.qmssHwStatus = QMSS_HW_INIT_COMPLETE;
        }
    }

    if (gObjPtr->initCfg.qmssHwStatus != QMSS_HW_INIT_COMPLETE)
    {
        /* Insert the linking RAM for QM 0 */
        Qmss_internalInsertLinkingRAM (subSys, 0, &linkingRAMCfg, &gObjPtr->initCfg.linkingRAM0Size);

        /* Insert the linking RAM for other QMs in split mode */
        for (qGroup = 1; qGroup < qmssGblCfgParams->maxQueMgrGroups; qGroup++)
        {
           Qmss_LinkingRAM *lram_p;
           uint32_t        *size_p;
           if (initCfg->mode == Qmss_Mode_SPLIT)
           {
               /* Each group gets its own config */
               lram_p = initCfg->splitLinkingRAMs + qGroup - 1;
               size_p = &gObjPtr->initCfg.splitLinkingRAMs[qGroup - 1].linkingRAM0Size;
           }
           else
           {
               /* Same config into all groups */
               lram_p = &linkingRAMCfg;
               size_p = &gObjPtr->initCfg.linkingRAM0Size;
           }
           Qmss_internalInsertLinkingRAM (subSys, qGroup, lram_p, size_p);
        }
        /* Note: qmssHwStatus is NOT QMSS_HW_INIT_COMPLETE, this is used in Qmss_exit
         * to avoid freeing linking ram */
    }

    /* Download Firmware */
    for (index = 0; index < qmssGblCfgParams->maxPDSP; index++)
    {
        if (initCfg->pdspFirmware[index].firmware != NULL)
        {
            retVal = Qmss_internaldownloadFirmware (*subSysHnd, initCfg->pdspFirmware[index].pdspId, initCfg->pdspFirmware[index].firmware,
                                    initCfg->pdspFirmware[index].size);
        }
    }

    /* Writeback Global Object - required because Qmss_internalInsertLinkingRAM can write to initCfg.linkingRAM0Size */
    Qmss_osalEndMemAccess (gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));

exitCs:
    /* End Critical Section */
    Qmss_osalCsExit (key);
    return retVal;
} /* Qmss_initSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_initSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_init (Qmss_InitCfg *initCfg, Qmss_GlobalConfigParams *qmssGblCfgParams)
{
    Qmss_SubSysHnd subSysHnd;

    return Qmss_initSubSys (&subSysHnd, Qmss_SubSys_GLOBAL, initCfg, qmssGblCfgParams);
    /* subSysHnd is intentionally discarded, for now QMSS_SUBSYS_HND_GLOBAL is used in
     * its place.  It would be saved in qmss_lObj if it becomes required.
     */
} /* Qmss_init */

/**
 *  @b Description
 *  @n
 *      This function deinitializes the CPPI low level driver.
 *      The LLD is deinitialized only if all the queues (opened via this instance)
 *      are closed and regions inserted through this instance are removed.
 *      It is user responsibility to dispose of all descriptors added via this
 *      instance.
 *
 *  @pre
 *      Qmss_init function should be called before calling this function.
 *      All descriptors added by this instance should be removed with Qmss_queueEmpty().
 *      All regions added by this instance shoudl be removed with Qmss_removeMemoryRegion().
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @retval
 *      Success -   QMSS_SOK
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 *                  QMSS_EXIT_QUEUES_OPEN
 *                  QMSS_EXIT_REGIONS_PRESENT
 *                  QMSS_QM_CONTROL_DENIED
 *                  QMSS_RESOURCE_LINKING_RAM_INIT_DENIED
 */
Qmss_Result Qmss_exitSubSys (Qmss_SubSysHnd *subSysHnd)
{
    Qmss_Result              retVal = QMSS_SOK;
    void                    *key;
    int32_t                  qPerGroup;
    uint32_t                 qGroup;
    uint32_t                 pdsp;
    Qmss_GlobalConfigParams *qmssGblCfgParams;
    uint32_t                 subSys = (uint32_t)*subSysHnd;
    Qmss_GlobalObj_Unpadded *gObjPtr = &qmssGObj.obj[subSys];
    Qmss_LocalObjParams     *lObjPtr = &qmssLObj[subSys].p;

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    /* Invalidate Global Object */
    Qmss_osalBeginMemAccess ((void *) gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));
    /* Invalidate group control */
    Qmss_osalBeginMemAccess ((void *) &qmssGroupControl[subSys], sizeof (qmssGroupControl[subSys]));

    /* Check if all regions are removed */
    if (gObjPtr->currDescCnt)
    {
        retVal = QMSS_EXIT_REGIONS_PRESENT;
        goto exitCs;
    }

    qmssGblCfgParams = &gObjPtr->qmssGblCfgParams;

    /* Check if all queues are removed */
    qPerGroup = qmssGblCfgParams->maxQue / qmssGblCfgParams->maxQueMgrGroups;
    for (qGroup = 0; qGroup < qmssGblCfgParams->maxQueMgrGroups; qGroup++)
    {
        if (qmssGroupControl[subSys].groupFreeQueues[qGroup] != qPerGroup)
        {
            retVal = QMSS_EXIT_QUEUES_OPEN;
            goto exitCs;
        }
    }

    if (lObjPtr->qmRmServiceHandle)
    {
        int32_t qmControl = 0;
        int32_t linkRamControl = 0;

        /* Release the control */
        if ((!Qmss_rmServiceGroups((Rm_ServiceHandle *)qmssGblCfgParams->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                   offsetof(Qmss_GlobalConfigGroupRm, rmQmControl), &qmControl, 1, 0, NULL, qmssGblCfgParams)))
        {
            /* Indicate RM wasn't able to free control */
            retVal = QMSS_QM_CONTROL_DENIED;
        }

        /* Release the linking RAM control only if we took it */
        if (gObjPtr->initCfg.qmssHwStatus != QMSS_HW_INIT_COMPLETE)
        {
            if ((!Qmss_rmServiceGroups((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                       offsetof(Qmss_GlobalConfigGroupRm,rmLinkRamControl),
                                       &linkRamControl, 1, 0, NULL, qmssGblCfgParams)))
            {
                /* Indicate RM couldn't free linking RAM */
                retVal = QMSS_RESOURCE_LINKING_RAM_INIT_DENIED;
            }
        }

        /* Release the firmware if we took it */
        for (pdsp = 0; pdsp < gObjPtr->qmssGblCfgParams.maxPDSP; pdsp++)
        {
            if (gObjPtr->initCfg.pdspFirmware[pdsp].firmware != NULL)
            {
                /* Release firmware "use" */
                int32_t pdspIdNum = gObjPtr->initCfg.pdspFirmware[pdsp].pdspId;
                if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                    lObjPtr->rmFirmwarePdsp, &pdspIdNum, 1, 0, NULL))
                {
                    retVal = QMSS_RESOURCE_ALLOCATE_USE_DENIED;
                }
                else
                {
                    /* Release firmware "init" */
                    if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                        lObjPtr->rmFirmwarePdsp, &pdspIdNum, 1, 0, NULL))
                    {
                        retVal = QMSS_RESOURCE_ALLOCATE_INIT_DENIED;
                    }
                }
            }
        }
    }

    /* Clear local object */
    qmssLObjIsValid[subSys] = 0;
    /* Clear global object */
    qmssGObjIsValid[subSys] = 0;
    Qmss_osalEndMemAccess ((void *) &qmssGObjIsValid[subSys], sizeof (qmssGObjIsValid[subSys]));
    /* Clear handle */
    *subSysHnd = (Qmss_SubSysHnd)NULL;

exitCs:
    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_exitSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_exitSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_exit (void)
{
    /* This changes to qmssLObj.subSysHnd, if there becomes need to save global handle */
    Qmss_SubSysHnd subSysHnd = QMSS_SUBSYS_HND_GLOBAL;

    return Qmss_exitSubSys (&subSysHnd);
} /* Qmss_exit */

/**
 *  @b Description
 *  @n
 *      This function initializes the Queue Manager SubsSystem local object blocks in memory
 *      local to each core and initializes any start parameters like the RM instance.
 *
 *      This function must be called at least once for every core.  It can either be called directly
 *      through the application if RM is to be used or it will be called via Qmss_startSubSys if RM is
 *      not to be used.
 *
 *      The @ref Qmss_startCfg and @ref Qmss_start are backwards compatible wrapper APIs that
 *      operate on the global subsystem.
 *
 *  @pre
 *      @ref Qmss_initSubSys function should be called before calling this function.
 *
 *  @retval
 *      Success -   QMSS_SOK
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 */
Qmss_Result Qmss_startSubSysCfg (Qmss_SubSysHnd *subSysHnd, Qmss_SubSys subSys, Qmss_StartCfg *startCfg)
{
    Qmss_LocalObjParams     *lObjPtr = &qmssLObj[subSys].p;

    if (subSys >= QMSS_MAX_SUBSYS)
    {
        return QMSS_INVALID_PARAM;
    }

    if (qmssLObjIsValid[subSys] == 0)
    {
        /* Invalidate Global Object Valid */
        Qmss_osalBeginMemAccess ((void *) &qmssGObjIsValid[subSys], sizeof (qmssGObjIsValid[subSys]));
        if (qmssGObjIsValid[subSys])
        {
            memset (lObjPtr, 0, sizeof (Qmss_LocalObjParams));
            if(startCfg->pQmssGblCfgParams)
            {
                /* Copy global configuration parameters */
                memcpy ((void *) lObjPtr, startCfg->pQmssGblCfgParams, sizeof (Qmss_GlobalConfigParams));
            }
            else
            {
                Qmss_GlobalObj_Unpadded *gObjPtr = &qmssGObj.obj[subSys];

                /* Invalidate Global Object */
                Qmss_osalBeginMemAccess (gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));
                *lObjPtr = gObjPtr->qmssGblCfgParams;
            }
            lObjPtr->qmRmServiceHandle = startCfg->rmServiceHandle;
            qmssLObjIsValid[subSys] = 1;
        }
        else
        {
            return QMSS_NOT_INITIALIZED;
        }
    }

    *subSysHnd = (Qmss_SubSysHnd)subSys;

    return QMSS_SOK;
} /* Qmss_startSubSysCfg */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_startSubSysCfg
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_startCfg (Qmss_StartCfg *startCfg)
{
    Qmss_SubSysHnd subSysHnd;

    return Qmss_startSubSysCfg (&subSysHnd, Qmss_SubSys_GLOBAL, startCfg);
} /* Qmss_startCfg */

/**
 *  @b Description
 *  @n
 *      This function initializes the Queue Manager SubsSystem local object blocks in memory
 *      local to each core.
 *
 *      This function must be called atleast once for every core.
 *
 *  @pre
 *      @ref Qmss_initSubSys (or @ref Qmss_init) function should be called before calling this function.
 *
 *  @retval
 *      Success -   QMSS_SOK
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 */
Qmss_Result Qmss_start (void)
{
    Qmss_StartCfg startCfg;

    /* For backwards compatability make the RM service handle NULL */
    startCfg.rmServiceHandle = NULL;
    startCfg.pQmssGblCfgParams = NULL;

    return(Qmss_startCfg(&startCfg));
} /* Qmss_start */

/**
 *  @b Description
 *  @n
 *      This function is used to obtain the QMSS memory region configuration.
 *      The function returns the descriptor base address, descriptor size and
 *      the number of descriptors for each configured memory region.
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[out]  memRegInfo
 *      Pointer to the memory region configuration structure allocated by caller.
 *
 *  @pre
 *      @ref Qmss_initSubSys function should be called before calling this function.
 *
 *  @retval
 *      Success -   QMSS_SOK
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 */
Qmss_Result Qmss_getMemoryRegionCfgSubSys (Qmss_SubSysHnd subSysHnd, Qmss_MemRegCfg *memRegInfo)
{
    uint32_t          i;
    uint32_t          qGroup;
    uint32_t          subSys = (uint32_t)subSysHnd;
    Qmss_GlobalObj_Unpadded *gObjPtr = &qmssGObj.obj[subSys];

    if (memRegInfo == NULL)
        return QMSS_INVALID_PARAM;

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    /* No RM check since readonly operation */

    memset ((void *) memRegInfo, 0, sizeof (Qmss_MemRegCfg));

    /* Invalidate Global Object */
    Qmss_osalBeginMemAccess (gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));

    /* Chose which group based on split/joint mode */
    if (gObjPtr->mode == Qmss_Mode_SPLIT)
    {
        /* In split mode, the config selects the group */
        qGroup = memRegInfo->queueGroup;
    }
    else
    {
        /* In joint mode, the LLD tracks configuration use group 0, but the
         * configuration is inserted into all of the group registers
         */
        qGroup = 0;
    }

    if (qGroup > gObjPtr->qmssGblCfgParams.maxQueMgrGroups)
    {
        return QMSS_INVALID_PARAM;
    }

    for (i = 0; i < QMSS_MAX_MEM_REGIONS; i++)
    {
        memRegInfo->memRegInfo[i] = gObjPtr->memRegInfo[qGroup][i];
    }
    memRegInfo->currDescCnt = gObjPtr->currDescCnt;
    return QMSS_SOK;
} /* Qmss_getMemoryRegionCfgSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_getMemoryRegionCfgSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_getMemoryRegionCfg (Qmss_MemRegCfg *memRegInfo)
{
    return Qmss_getMemoryRegionCfgSubSys (QMSS_SUBSYS_HND_GLOBAL, memRegInfo);
} /* Qmss_getMemoryRegionCfg */

/**
 *  @b Description
 *  @n
 *      This function is used to configure memory region at runtime.
 *      The function configures specified memory region with descriptor base address,
 *      descriptor size and the number of descriptors.
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[out]  memRegCfg
 *      Pointer to the memory region configuration structure allocated by caller.
 *      Descriptor memory address should be a global address
 *
 *  @pre
 *      @ref Qmss_initSubSys function should be called before calling this function.
 *      Memory Region specified should not have been configured before.
 *
 *  @retval
 *      Success -   Inserted Memory region index. Range is 0 to 63.
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure -   QMSS_MEMREGION_ALREADY_INITIALIZED
 *  @retval
 *      Failure -   QMSS_MEMREGION_INVALID_PARAM
 *  @retval
 *      Failure -   QMSS_MAX_DESCRIPTORS_CONFIGURED
 *  @retval
 *      Failure -   QMSS_MEMREGION_INVALID_INDEX
 *  @retval
 *      Failure -   QMSS_MEMREGION_OVERLAP
 *  @retval
 *      Failure -   QMSS_QUEUE_OPEN_ERROR
 *  @retval
 *      Failure -   QMSS_RESOURCE_MEMORY_REGION_INIT_DENIED
 *  @retval
 *      Failure -   QMSS_RESOURCE_LINKING_RAM_INIT_DENIED
 *  @retval
 *      Failure -   QMSS_FEATURE_REQUIRES_RM
 *  @retval
 *      Failure -   QMSS_NS_NAME_FAIL
 *  @retval
 *      Failure -   QMSS_NS_FAIL
 */
Qmss_Result Qmss_insertMemoryRegionSubSys (Qmss_SubSysHnd subSysHnd, Qmss_MemRegInfo *memRegCfg)
{
    uint32_t          count, powRegSize, reg = 0;
    int32_t           i, nextIndex;
    uint32_t          descNum, oneBits;
    uint8_t           *descPtr, isAllocated;
    int32_t           queueHnd, index = -1, prevIndex, startIndex;
    void             *key;
    uint32_t         *descBasePhy;
    Qmss_Result       result;
    uint8_t          *newRegBegAddr, *newRegEndAddr;
    int32_t           newRegBegIndex, newRegEndIndex;
    uint32_t          qGroup = 0;
    /* For installing same config over all queue groups */
    uint32_t          firstQGroup, regQGroup, lastQGroup;
    int               rmMemRegionTaken = 0, rmStartIdxTaken = 0, hwProgrammed = 0;
    uint32_t          subSys = (uint32_t)subSysHnd;
    Qmss_GlobalObj_Unpadded *gObjPtr = &qmssGObj.obj[subSys];
    Qmss_LocalObjParams     *lObjPtr = &qmssLObj[subSys].p;
    char              rmName[QMSS_RM_RESOURCE_NAME_MAX_CHARS];

    if (memRegCfg == NULL)
        return QMSS_INVALID_PARAM;

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    /* Descriptor size should be a multiple of 16 */
    if ((memRegCfg->descSize % 16) != 0)
    {
        return QMSS_INVALID_PARAM;
    }

    /* Number of descriptors should be a power of 2 >= 2^5.  The upper limit is checked
     * below inside Qmss_osalCsEnter() */
    descNum = memRegCfg->descNum;
    if (descNum < 32)
    {
        return QMSS_INVALID_PARAM;
    }

    /* count the # of 1 bits, in order to see if its a power of 2 */
    for (oneBits = 0, descNum = memRegCfg->descNum; descNum; descNum >>=1)
    {
        if (descNum & 1) {
            oneBits++;
        }
    }

    /* Check if # of descriptors is a power of 2 */
    if (oneBits != 1) {
        return QMSS_INVALID_PARAM;
    }

    descBasePhy = (uint32_t*)Qmss_internalVirtToPhy(subSys, (void *)(memRegCfg->descBase));
    if (!descBasePhy) {
        return QMSS_INVALID_PARAM;
    }

    /* The region base address must be aligned to 16 bytes.  It is suggested
     * to align to 64 bytes if the descriptor size > 32 bytes for performance
     * reasons.
     */
    if (((uint32_t)descBasePhy) & 0xf) {
        return QMSS_INVALID_PARAM;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    /* Invalidate Global Object */
    Qmss_osalBeginMemAccess (gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));

    /* Choose which group based on split/joint mode */
    if (gObjPtr->mode == Qmss_Mode_SPLIT)
    {
        /* In split mode, the config selects the group */
        qGroup = memRegCfg->queueGroup;
    }
    else
    {
        /* In joint mode, the LLD tracks configuration use group 0, but the
         * configuration is inserted into all of the group registers
         */
        qGroup = 0;
    }
    if (qGroup > gObjPtr->qmssGblCfgParams.maxQueMgrGroups)
    {
        result = QMSS_INVALID_PARAM;
        goto exitCs;
    }

    /* Select correct memory region */

    /* Check if maximum descriptors already configured */
    if (gObjPtr->currDescCnt + memRegCfg->descNum > gObjPtr->initCfg.maxDescNum)
    {
        result = QMSS_MAX_DESCRIPTORS_CONFIGURED;
        goto exitCs; /* common exit to close CsEnter and MemAccess */
    }

    /* Memory region to configure is not specified. Find the next available one */
    if (memRegCfg->memRegion == Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED)
    {
        if (lObjPtr->qmRmServiceHandle)
        {
            int32_t region = memRegCfg->memRegion;
            if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_INIT,
                                lObjPtr->groupRm[qGroup].rmMemRegion, &region, 1, 0, NULL))
            {
                result = QMSS_RESOURCE_MEM_REGION_INIT_DENIED;
                goto exitCs;
            }
            rmMemRegionTaken = 1;
            memRegCfg->memRegion = (Qmss_MemRegion)region;
            index = memRegCfg->memRegion;
        }
        else
        {
            for (i = 0; i < QMSS_MAX_MEM_REGIONS; i++)
            {
                if (gObjPtr->memRegInfo[qGroup][i].descNum == 0)
                {
                    index = i;
                    break;
                }
            }
        }
        if (lObjPtr->qmRmServiceHandle) {
            startIndex = memRegCfg->startIndex;
            if (startIndex >= 0)
            {
                /* Let RM override user specified (memset(0)) region */
                startIndex = QMSS_PARAM_NOT_SPECIFIED;
            }
        }
        else {
            if (index == 0)
                startIndex = 0;
            else
                startIndex = gObjPtr->memRegInfo[qGroup][index - 1].descNum + gObjPtr->memRegInfo[qGroup][index - 1].startIndex;
        }
    }
    else
    {
        if (lObjPtr->qmRmServiceHandle)
        {
            int32_t region = memRegCfg->memRegion;
            if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_INIT,
                                lObjPtr->groupRm[qGroup].rmMemRegion, &region, 1, 0, &isAllocated))
            {
                result = QMSS_RESOURCE_MEM_REGION_INIT_DENIED;
                goto exitCs;
            }
            memRegCfg->memRegion = (Qmss_MemRegion)region;
            rmMemRegionTaken = 1;
            if (isAllocated != 1)
            {
                /* Somebody else already initialized this region */
                result = QMSS_MEMREGION_ALREADY_INITIALIZED;
                goto exitCs;
            }
        }
        index = memRegCfg->memRegion;
        startIndex = memRegCfg->startIndex;
    }

    if (lObjPtr->qmRmServiceHandle)
    {
        if (memRegCfg->manageDescFlag)
        {
            if (snprintf (rmName, QMSS_RM_RESOURCE_NAME_MAX_CHARS,  lObjPtr->rmRegQueueFmt,
                          subSys, qGroup, index) >= QMSS_RM_RESOURCE_NAME_MAX_CHARS)
            {
                /* the name got chopped off */
                result = QMSS_NS_NAME_FAIL;
                goto exitCs;
            }
        }
        result = Qmss_internalRmStartIndex (lObjPtr, qGroup, &startIndex, memRegCfg->descNum, 1, NULL);
        if (result != QMSS_SOK)
        {
            goto exitCs;
        }
        rmStartIdxTaken = 1;
    }
    else if (startIndex < 0)
    {
        /* Using startIndex = -1,-2,-3 requires having RM installed */
        result = QMSS_FEATURE_REQUIRES_RM;
        goto exitCs;
    }

    /* Invalid memory region index */
    if (index < 0 || index >= QMSS_MAX_MEM_REGIONS)
    {
        result = QMSS_MEMREGION_INVALID_INDEX;
        goto exitCs; /* common exit to close CsEnter and MemAccess */
    }

    /* If specified memory region is already configured don't overwrite that configuration */
    if (gObjPtr->memRegInfo[qGroup][index].descNum != 0)
    {
        result = QMSS_MEMREGION_ALREADY_INITIALIZED;
        goto exitCs; /* common exit to close CsEnter and MemAccess */
    }

    /* startIndex should be a multiple of 32 */
    if ((startIndex % 32) != 0)
    {
        result = QMSS_INVALID_PARAM;
        goto exitCs; /* common exit to close CsEnter and MemAccess */
    }

    /* Compute boundaries of the new region */
    newRegBegAddr  = (uint8_t *)Qmss_internalVirtToPhy(subSys, (void *)(memRegCfg->descBase));
    newRegEndAddr  = newRegBegAddr + memRegCfg->descNum * memRegCfg->descSize;
    newRegBegIndex = startIndex;
    newRegEndIndex = newRegBegIndex + memRegCfg->descNum;

    /* Check if the new region overlaps any existing regions.  This is done
     * with a brute force search that doesn't depend on ordering
     *
     * This allows overlap check to be kept if future hardware relaxes the
     * ordering requirements.
     */
    for (i = 0; i < QMSS_MAX_MEM_REGIONS; i++)
    {
        Qmss_MemRegInfo *checkReg = &gObjPtr->memRegInfo[qGroup][i];
        uint8_t *checkRegBegAddr  = 0;
        uint8_t *checkRegEndAddr  = 0;
        int32_t checkRegBegIndex = checkReg->startIndex;
        int32_t checkRegEndIndex = checkRegBegIndex + checkReg->descNum;

        if(checkReg->descBase)
        {
            /* Address translation if active */
            checkRegBegAddr = (uint8_t *)Qmss_internalVirtToPhy(subSys, (void *)(checkReg->descBase));
        }
        checkRegEndAddr = checkRegBegAddr + (checkReg->descNum * checkReg->descSize);

        if (i == index)
        {
            continue; /* don't test against myself */
        }

        if (! checkRegBegAddr) {
            continue; /* Check region hasn't been initialized */
        }

        /* Check the physical addresses for overlap */
        /* A) Is the new region's start address overlapping the check region? */
        if ((newRegBegAddr >= checkRegBegAddr) &&
            (newRegBegAddr < checkRegEndAddr))
        {   /* The new region starts inside the check region */
            result = QMSS_MEMREGION_OVERLAP;
            goto exitCs; /* common exit to close CsEnter and MemAccess */
        }
        /* B) Is the new region's end address overlapping the check region? */
        if ((newRegEndAddr > checkRegBegAddr) &&
            (newRegEndAddr <= checkRegEndAddr))
        {   /* The new region ends inside the check region */
            result = QMSS_MEMREGION_OVERLAP;
            goto exitCs; /* common exit to close CsEnter and MemAccess */
        }
        /* C) Is the check region completely inside the new region? */
        if ((checkRegBegAddr >= newRegBegAddr) &&
            (checkRegEndAddr <= newRegEndAddr))
        {   /* The check region is contained within the new region */
            result = QMSS_MEMREGION_OVERLAP;
            goto exitCs; /* common exit to close CsEnter and MemAccess */
        }
        /* It is not necessary to check if the new region is completely inside
         * the check region because that is covered by A and B above since both
         * its start and end addresses are inside the check region.
         */

        /* Check the linking RAM index for overlap */
        /* Is the new region's start index overlapping the check region? */
        if ((newRegBegIndex >= checkRegBegIndex) &&
            (newRegBegIndex < checkRegEndIndex))
        {   /* The new region starts inside the check region */
            result = QMSS_MEMREGION_OVERLAP;
            goto exitCs; /* common exit to close CsEnter and MemAccess */
        }
        /* Is the new region's end index overlapping the check region? */
        if ((newRegEndIndex > checkRegBegIndex) &&
            (newRegEndIndex <= checkRegEndIndex))
        {   /* The new region ends inside the check region */
            result = QMSS_MEMREGION_OVERLAP;
            goto exitCs; /* common exit to close CsEnter and MemAccess */
        }
        /* Is the check region completely inside the new region? */
        if ((checkRegBegIndex >= newRegBegIndex) &&
            (checkRegEndIndex <= newRegEndIndex))
        {   /* The check region is contained within the new region */
            result = QMSS_MEMREGION_OVERLAP;
            goto exitCs; /* common exit to close CsEnter and MemAccess */
        }
        /* It is not necessary to check if the new region is completely inside
         * the check region because that is covered by A and B above since both
         * its start and end addresses are inside the check region.
         */
    }

    /* Some devices don't constrain the order of memory regions */
    if (lObjPtr->orderedMemReg)
    {
        /* Find next and previous initialized regions to check for ordering */
        for (prevIndex = index-1;
             (prevIndex >= 0 && (gObjPtr->memRegInfo[qGroup][prevIndex].descNum == 0));
             prevIndex--);
        for (nextIndex = index+1;
             ((nextIndex <= (QMSS_MAX_MEM_REGIONS-1) &&
              (gObjPtr->memRegInfo[qGroup][nextIndex].descNum == 0)));
             nextIndex++);

        /* If there is a previous region make sure its index and its physical
         * address is lower than the new region's.
         *
         * It is assumed that the index and address do not overlap the previous
         * region since that is already verified above.
         */
        if (prevIndex >= 0)
        {
            Qmss_MemRegInfo *prevReg       = &gObjPtr->memRegInfo[qGroup][prevIndex];

            /* Check that the linking RAM index is strictly increasing */
            if (prevReg->startIndex >= newRegBegIndex) {
                result = QMSS_MEMREGION_ORDERING;
                goto exitCs; /* common exit to close CsEnter and MemAccess */
            }

            /* Check that the physical memory address is strictly increasing */
            if (((uint8_t *)Qmss_internalVirtToPhy(subSys, (void *)(prevReg->descBase))) >= newRegBegAddr) {
                result = QMSS_MEMREGION_ORDERING;
                goto exitCs; /* common exit to close CsEnter and MemAccess */
            }
        }

        /* If there is a next region make sure its index and its physical
         * address is lower than the new region's.
         *
         * It is assumed that the index and address do not overlap the next
         * region since that is already verified above.
         */
        if (nextIndex < QMSS_MAX_MEM_REGIONS)
        {
            Qmss_MemRegInfo *nextReg       = &gObjPtr->memRegInfo[qGroup][nextIndex];

            /* Check that the linking RAM index is strictly increasing */
            if (nextReg->startIndex <= newRegBegIndex) {
                result = QMSS_MEMREGION_ORDERING;
                goto exitCs; /* common exit to close CsEnter and MemAccess */
            }

            /* Check that the physical memory address is strictly increasing */
            if (((uint8_t *)Qmss_internalVirtToPhy(subSys, (void *)(nextReg->descBase))) <= newRegBegAddr) {
                result = QMSS_MEMREGION_ORDERING;
                goto exitCs; /* common exit to close CsEnter and MemAccess */
            }
        }
    }

    /* If the qmssGObj is valid (i.e. HW INIT was performed in the DSP subsystem
      * check that there is sufficient linking RAM */
    if (gObjPtr->initCfg.qmssHwStatus != QMSS_HW_INIT_COMPLETE)
    {
        if (newRegEndIndex > (int32_t) gObjPtr->initCfg.maxDescNum)
        {
            result = QMSS_INVALID_PARAM;
            goto exitCs; /* common exit to close CsEnter and MemAccess */
        }
    }

    /* Configure the QMSS memory regions */
    if (gObjPtr->mode == Qmss_Mode_SPLIT)
    {
        /* Apply configuration to selected group */
        firstQGroup = qGroup;
        lastQGroup  = qGroup;
    }
    else
    {
        /* Apply configuration to all groups */
        firstQGroup = 0;
        lastQGroup  = gObjPtr->qmssGblCfgParams.maxQueMgrGroups - 1;
    }

    for (regQGroup = firstQGroup; regQGroup <= lastQGroup; regQGroup++)
    {
        CSL_FINS (lObjPtr->groupRegs[regQGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[index].MEMORY_REGION_START_INDEX_REG,
            QM_DESCRIPTOR_REGION_CONFIG_MEMORY_REGION_START_INDEX_REG_START_INDEX, startIndex);

        CSL_FINS (reg, QM_DESCRIPTOR_REGION_CONFIG_MEMORY_REGION_DESCRIPTOR_SETUP_REG_DESC_SIZE,
            ((memRegCfg->descSize / 16) - 1));

        for (powRegSize = 0; (32UL << powRegSize) < memRegCfg->descNum; powRegSize++);

        CSL_FINS (reg, QM_DESCRIPTOR_REGION_CONFIG_MEMORY_REGION_DESCRIPTOR_SETUP_REG_REG_SIZE, powRegSize);

        lObjPtr->groupRegs[regQGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[index].MEMORY_REGION_DESCRIPTOR_SETUP_REG = reg;

        /* Writing nonzero to base address "activates" region */
        lObjPtr->groupRegs[regQGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[index].MEMORY_REGION_BASE_ADDRESS_REG = (uint32_t)descBasePhy;
    }

    hwProgrammed = 1;

    /* Check if LLD is supposed to manage the descriptors, if so slice up the
     * descriptors and push to queues. else do nothing the caller will manage
     * the descriptors in memory region */
    if (memRegCfg->manageDescFlag)
    {
        /* Clear the descriptor area to avoid virt2phy/phy2virt traversals */
        memset (memRegCfg->descBase, 0, memRegCfg->descSize * memRegCfg->descNum);

        /* Split and push to general purpose queues */

        /* Open general purpose queue */
        queueHnd = Qmss_internalQueueOpen (subSysHnd, 0, qGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (queueHnd < 0)
        {
            result = QMSS_QUEUE_OPEN_ERROR;
            goto exitCs; /* common exit to close osalCsEnter and osalBeginMemAccess */
        }

        /* Store the handle */
        gObjPtr->descQueue[qGroup][index] = queueHnd;

        /* Publish the queue number in a name */
        if (lObjPtr->qmRmServiceHandle)
        {
            /* Name was made above, before region was inserted, where failure is easier to handle */
            if (! Qmss_nameServiceSet (lObjPtr->qmRmServiceHandle, rmName, Qmss_getQIDFromHandle (queueHnd)))
            {
                result = QMSS_NS_FAIL;
                goto exitCs;
            }
        }

        descPtr = (uint8_t *) memRegCfg->descBase;
        for (count = 0; count < memRegCfg->descNum; count ++)
        {
            /* TODO QueueProxy is not working add this later
             * Qmss_queuePushDesc (queueHnd, (uint32_t *)descPtr, initCfgPtr->memRegionInfo[index].descSize); */

            Qmss_queuePushDesc (queueHnd, (uint32_t *) descPtr);
            descPtr = descPtr + memRegCfg->descSize;
        }
    }

    /* Store in internal data structures */
    gObjPtr->memRegInfo[qGroup][index]            = *memRegCfg;
    gObjPtr->memRegInfo[qGroup][index].startIndex = startIndex;
    gObjPtr->memRegInfo[qGroup][index].memRegion  = (Qmss_MemRegion) index;
    gObjPtr->currDescCnt += memRegCfg->descNum;

    /* Return value */
    result = index;

exitCs:
    /* Writeback Global Object */
    Qmss_osalEndMemAccess (gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));

    /* RM Error cleanup */
    if (lObjPtr->qmRmServiceHandle && (result < QMSS_SOK) && (! hwProgrammed))
    {
        if (rmMemRegionTaken)
        {
            int32_t region = memRegCfg->memRegion;
            if (! Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                 lObjPtr->groupRm[qGroup].rmMemRegion, &region, 1, 0, NULL))
            {
                result = QMSS_RESOURCE_FREE_DENIED; /* double error */
            }
            memRegCfg->memRegion = (Qmss_MemRegion)region;
        }
        if (rmStartIdxTaken)
        {
            if (Qmss_internalRmStartIndex (lObjPtr, qGroup, &startIndex, memRegCfg->descNum, 0, NULL) != QMSS_SOK)
            {
                result = QMSS_RESOURCE_FREE_DENIED; /* double error */
            }
        }
    }

    /* End Critical Section */
    Qmss_osalCsExit (key);
    return result;
} /* Qmss_insertMemoryRegionSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_insertMemoryRegionSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_insertMemoryRegion (Qmss_MemRegInfo *memRegCfg)
{
    return Qmss_insertMemoryRegionSubSys (QMSS_SUBSYS_HND_GLOBAL, memRegCfg);
} /* Qmss_insertMemoryRegion */

/**
 *  @b Description
 *  @n
 *      Registers usage of region memRegion in this contect.
 *
 *      This allows RM to know which contexts are using the region, so it
 *      won't be removed until all contexts are done.  It also enables the
 *      use of @ref Qmss_getMemoryRegionCfg / @ref Qmss_getMemoryRegionCfgSubSys,
 *      @ref Qmss_getMemRegQueueHandle / @ref Qmss_getMemRegQueueHandleSubSys,
 *      and
 *      @ref Qmss_getMemRegDescSize / @ref Qmss_getMemRegDescSizeSubSys
 *      in this context when region was created in another context.
 *
 *      Verfies that virt2phys/phys2virt is available for descriptors in
 *      this context.
 *
 *      This API requires RM to be initalized.  If RM is not available,
 *      it is presumed there is exactly one context in entire
 *      system (qmssGObj).
 *
 *      The inverse of this API is @ref Qmss_closeMemoryRegionSubSys /
 *      @ref Qmss_closeMemoryRegion
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]   memRegion
 *      Region number to use
 *
 *  @param[in]     qGroup
 *      queue group within subsystem to open from (should be 0 unless split mode)
 *
 *  @retval
 *      Success -   QMSS_SOK
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure -   QMSS_MEMREGION_ALREADY_INITIALIZED
 *  @retval
 *      Failure -   QMSS_MEMREGION_NOT_ALREADY_INITIALIZED
 *  @retval
 *      Failure -   QMSS_MEMREGION_INVALID_PARAM
 *  @retval
 *      Failure -   QMSS_MEMREGION_INVALID_INDEX
 *  @retval
 *      Failure -   QMSS_QUEUE_OPEN_ERROR
 *  @retval
 *      Failure -   QMSS_FEATURE_REQUIRES_RM
 *  @retval
 *      Failure -   QMSS_VIRTPHYS_FAIL
 */
Qmss_Result Qmss_openMemoryRegionSubSys (Qmss_SubSysHnd subSysHnd, Qmss_MemRegion memRegion, uint32_t qGroup)
{
    Qmss_Result              retVal   = QMSS_SOK;
    void                    *key;
    uint32_t                 subSys   = (uint32_t)subSysHnd;
    Qmss_GlobalObj_Unpadded *gObjPtr  = &qmssGObj.obj[subSys];
    Qmss_LocalObjParams     *lObjPtr  = &qmssLObj[subSys].p;
    int32_t                  index    = (int32_t)memRegion;
    Qmss_MemRegInfo         *gMemInfo;
    int32_t                 *gDescQ;
    uint32_t                 reg;
    int32_t                  rmQID;
    uint32_t                *descBasePhy;
    uint32_t                *descBasePhy2;
    uint8_t                  isAllocated;
    int                      rmMemRegionTaken = 0;
    int                      rmStartIdxTaken = 0;
    int                      qOpened = 0;
    char                     rmName[QMSS_RM_RESOURCE_NAME_MAX_CHARS];

    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    if (! lObjPtr->qmRmServiceHandle)
    {
        return QMSS_FEATURE_REQUIRES_RM;
    }

    /* Invalid memory region index */
    if (index < 0 || index >= QMSS_MAX_MEM_REGIONS)
    {
        return QMSS_MEMREGION_INVALID_INDEX;
    }

    /* Initialize after checking index */
    gMemInfo = &gObjPtr->memRegInfo[qGroup][index];
    gDescQ   = &gObjPtr->descQueue[qGroup][index];

    if (qGroup > gObjPtr->qmssGblCfgParams.maxQueMgrGroups)
    {
        return QMSS_INVALID_PARAM;
    }

    if (snprintf (rmName, QMSS_RM_RESOURCE_NAME_MAX_CHARS,  lObjPtr->rmRegQueueFmt,
                  subSys, qGroup, index) >= QMSS_RM_RESOURCE_NAME_MAX_CHARS)
    {
        /* the name got chopped off */
        return QMSS_NS_NAME_FAIL;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    /* Invalidate Global Object */
    Qmss_osalBeginMemAccess (gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));

    /* Region must not exist in this context */
    if (gMemInfo->descNum != 0)
    {
        retVal = QMSS_MEMREGION_ALREADY_INITIALIZED;
        goto exitCs; /* common exit to close CsEnter and MemAccess */
    }

    /* Take region from RM to increase usage count */
    if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_INIT,
                        lObjPtr->groupRm[qGroup].rmMemRegion, &index, 1, 0, &isAllocated))
    {
        retVal = QMSS_RESOURCE_MEM_REGION_INIT_DENIED;
        goto exitCs;
    }
    rmMemRegionTaken = 1; /* Since we took the region, need to free it if error occurs after this point */
    /* Can't be first taker */
    if (isAllocated == 1)
    {
        /* If this happened, RM is out of sync */
        retVal = QMSS_MEMREGION_NOT_ALREADY_INITIALIZED;
        goto exitCs;
    }

    /* get descSize from hw */
    reg = lObjPtr->groupRegs[qGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[index].MEMORY_REGION_DESCRIPTOR_SETUP_REG;
    gMemInfo->descSize = (1 + CSL_FEXT (reg, QM_DESCRIPTOR_REGION_CONFIG_MEMORY_REGION_DESCRIPTOR_SETUP_REG_DESC_SIZE)) * 16;
    /* get descNum from hw */
    gMemInfo->descNum = 1 << (5 + CSL_FEXT (reg, QM_DESCRIPTOR_REGION_CONFIG_MEMORY_REGION_DESCRIPTOR_SETUP_REG_REG_SIZE));

    /* get startIndex from hw */
    gMemInfo->startIndex =
        CSL_FEXT (lObjPtr->groupRegs[qGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[index].MEMORY_REGION_START_INDEX_REG,
                  QM_DESCRIPTOR_REGION_CONFIG_MEMORY_REGION_START_INDEX_REG_START_INDEX);

    /* "take" the startIndex range from RM */
    retVal = Qmss_internalRmStartIndex (lObjPtr, qGroup, &gMemInfo->startIndex, gMemInfo->descNum, 1, &isAllocated);
    if (retVal != QMSS_SOK)
    {
        goto exitCs;
    }
    rmStartIdxTaken = 1;
    /* Can't be first taker */
    if (isAllocated == 1)
    {
        /* If this happened, RM is out of sync */
        retVal = QMSS_MEMREGION_NOT_ALREADY_INITIALIZED;
        goto exitCs;
    }

    /* Get descQueue from RM.  This infers manageDescFlag */
    if (Qmss_nameServiceGet (lObjPtr->qmRmServiceHandle, rmName, &rmQID))
    {
        if ((*gDescQ =
            Qmss_internalQueueOpen (subSysHnd, 0, qGroup, (Qmss_QueueType) QMSS_PARAM_NOT_SPECIFIED, rmQID, &isAllocated)) < QMSS_SOK)
        {
            retVal = QMSS_QUEUE_OPEN_ERROR;
            goto exitCs;
        }
        gMemInfo->manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
        qOpened = 1;
    }
    else
    {
        /* No queue just implies manage descriptors wasn't enabled */
        gMemInfo->manageDescFlag = Qmss_ManageDesc_UNMANAGED_DESCRIPTOR;
    }

    /* get descBase from hw */
    descBasePhy =  (uint32_t *)lObjPtr->groupRegs[qGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[index].MEMORY_REGION_BASE_ADDRESS_REG;

    /* convert to virt */
    gMemInfo->descBase = Qmss_internalPhyToVirt (subSys, descBasePhy);
    if (! gMemInfo->descBase) {
        retVal = QMSS_VIRTPHYS_FAIL;
        /* v2p or p2v not correctly configured */
        goto exitCs;
    }

    /* Test other direction */
    descBasePhy2 = Qmss_internalVirtToPhy (subSys, gMemInfo->descBase);
    if (descBasePhy2 != descBasePhy)
    {
        /* v2p or p2v not correctly configured */
        retVal = QMSS_VIRTPHYS_FAIL;
        goto exitCs;
    }

    /* save queueGroup */
    gMemInfo->queueGroup = qGroup;

    /* Update descriptor count */
    gObjPtr->currDescCnt += gMemInfo->descNum;
exitCs:
    /* RM Error cleanup */
    if (retVal < QMSS_SOK)
    {
        if (rmMemRegionTaken)
        {
            if (! Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                 lObjPtr->groupRm[qGroup].rmMemRegion, &index, 1, 0, NULL))
            {
                retVal = QMSS_RESOURCE_FREE_DENIED; /* double error */
            }
        }
        if (rmStartIdxTaken)
        {
            if (Qmss_internalRmStartIndex (lObjPtr, qGroup, &gMemInfo->startIndex, gMemInfo->descNum, 0, NULL) != QMSS_SOK)
            {
                retVal = QMSS_RESOURCE_FREE_DENIED; /* double error */
            }
        }

        /* Free queue on error -- don't check error since already double error */
        if (qOpened)
        {
            Qmss_internalQueueClose (*gDescQ);
        }

        /* Set number of descriptors back to 0 to indicate region not ready */
        gMemInfo->descNum = 0;
    }

    /* Writeback Global Object */
    Qmss_osalEndMemAccess (gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));

    /* End Critical Section */
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_openMemoryRegionSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_openMemoryRegionSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_openMemoryRegion (Qmss_MemRegion memRegion, uint32_t qGroup)
{
    return Qmss_openMemoryRegionSubSys (QMSS_SUBSYS_HND_GLOBAL, memRegion, qGroup);
} /* Qmss_openMemoryRegion */

/**
 *  @b Description
 *  @n
 *      This function is used to deconfigure memory region at runtime.
 *      The hardware region is set to NULL, and any associated queue
 *      is empted and closed if the usage count for the region becomes 0.
 *
 *      This function is inverse of both @ref Qmss_insertMemoryRegionSubSys
 *      and Qmss_openMemoryRegionSubSys
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in] region
 *      Memory region to clear (return value of Qmss_insertMemoryRegion)
 *
 *  @param[in] qGroup
 *      Queue group to clear region from (only used in split mode)
 *
 *  @pre
 *      All descriptors must be discarded by application from all queues
 *      before using this function, or it will leave hw in unknown state.
 *
 *  @retval
 *      Success -   QMSS_SOK
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM, QMSS_NOT_INITIALIZED
 */
Qmss_Result Qmss_removeMemoryRegionSubSys (Qmss_SubSysHnd subSysHnd, int32_t region, uint32_t qGroup)
{
    void             *key;
    /* For installing same config over all queue groups */
    uint32_t          firstQGroup, regQGroup, lastQGroup;
    Qmss_QueueHnd     freeQ = -1;
    Qmss_Result       retVal = QMSS_SOK;
    Qmss_MemRegInfo  *pRegCfg;
    uint32_t                 subSys = (uint32_t)subSysHnd;
    Qmss_GlobalObj_Unpadded *gObjPtr = &qmssGObj.obj[subSys];
    Qmss_LocalObjParams     *lObjPtr = &qmssLObj[subSys].p;
    uint8_t                  isAllocated = 0; /* make no-RM case work */
    char                     rmName[QMSS_RM_RESOURCE_NAME_MAX_CHARS];

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    /* Chose which group based on split/joint mode */
    if (gObjPtr->mode != Qmss_Mode_SPLIT)
    {
        /* In joint mode, the LLD tracks configuration use group 0, but the
         * configuration is inserted into all of the group registers
         */
        qGroup      = 0;
        firstQGroup = 0;
        lastQGroup  = gObjPtr->qmssGblCfgParams.maxQueMgrGroups - 1;
    }
    else
    {
        firstQGroup = qGroup;
        lastQGroup  = qGroup;
    }

    if (qGroup > gObjPtr->qmssGblCfgParams.maxQueMgrGroups)
    {
        return QMSS_INVALID_PARAM;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    /* Invalidate Global Object */
    Qmss_osalBeginMemAccess ((void *) &qmssGObj.obj, sizeof (Qmss_GlobalObj_Unpadded));

    pRegCfg = &gObjPtr->memRegInfo[qGroup][region];

    if (lObjPtr->qmRmServiceHandle)
    {
        int32_t  linkRamStartIndex = pRegCfg->startIndex;
        uint32_t linkRamDescNum = pRegCfg->descNum;

        if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                            lObjPtr->groupRm[qGroup].rmMemRegion, &region, 1, 0, &isAllocated))
        {
            retVal = QMSS_RESOURCE_FREE_DENIED;
            goto exitCs;
        }

        if (Qmss_internalRmStartIndex (lObjPtr, qGroup, &linkRamStartIndex, linkRamDescNum, 0, NULL) != QMSS_SOK)
        {
            retVal = QMSS_RESOURCE_FREE_DENIED;
            goto exitCs;
        }

        /* Last user frees name */
        if (isAllocated == 0)
        {
            /* Delete the name if last user exits */
            if (pRegCfg->manageDescFlag)
            {
                if (snprintf (rmName, QMSS_RM_RESOURCE_NAME_MAX_CHARS,  lObjPtr->rmRegQueueFmt,
                              subSys, qGroup, region) >= QMSS_RM_RESOURCE_NAME_MAX_CHARS)
                {
                    /* the name got chopped off */
                    retVal = QMSS_NS_NAME_FAIL;
                    goto exitCs;
                }

                if (! Qmss_nameServiceDel (lObjPtr->qmRmServiceHandle, rmName))
                {
                    retVal = QMSS_NS_FAIL;
                    goto exitCs;
                }
            }
        }
    }

    /* If the LLD is keeping a free descriptor queue, clear it and close it */
    if (pRegCfg->manageDescFlag)
    {
        freeQ = gObjPtr->descQueue[qGroup][region];
        gObjPtr->descQueue[qGroup][region] = -1;
        /* trash the descriptors if last user exits */
        if (isAllocated == 0)
        {
           Qmss_queueEmpty (freeQ);
        }
        Qmss_internalQueueClose (freeQ);
    }

    /* Clear the config */
    gObjPtr->currDescCnt -= pRegCfg->descNum;
    memset (pRegCfg, 0, sizeof(*pRegCfg));

    /* Writeback */
    Qmss_osalEndMemAccess ((void *) gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));

    /* Clear the registers if last user exits */
    if (isAllocated == 0)
    {
        for (regQGroup = firstQGroup; regQGroup <= lastQGroup; regQGroup++)
        {
            /* Writing 0 to MEMORY_REGION_BASE_ADDRESS_REG deactivates the region */
            lObjPtr->groupRegs[regQGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[region].MEMORY_REGION_BASE_ADDRESS_REG = (uint32_t)0;
            lObjPtr->groupRegs[regQGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[region].MEMORY_REGION_DESCRIPTOR_SETUP_REG = 0;
        }
    }

exitCs:
    /* End Critical Section */
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_removeMemoryRegionSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_removeMemoryRegionSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_removeMemoryRegion (int32_t region, uint32_t qGroup)
{
    return Qmss_removeMemoryRegionSubSys (QMSS_SUBSYS_HND_GLOBAL, region, qGroup);
} /* Qmss_removeMemoryRegion */

/**
 *  @b Description
 *  @n
 *      Inverse of Qmss_openMemoryRegionSubSys.  Currently equivalent to Qmss_removeMemoryRegionSubSys.
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_closeMemoryRegionSubSys (Qmss_SubSysHnd subSysHnd, int32_t region, uint32_t qGroup)
{
    return Qmss_removeMemoryRegionSubSys (subSysHnd, region, qGroup);
} /* Qmss_closeMemoryRegionSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_closeMemoryRegionSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_closeMemoryRegion (int32_t region, uint32_t qGroup)
{
    return Qmss_closeMemoryRegionSubSys (QMSS_SUBSYS_HND_GLOBAL, region, qGroup);
} /* Qmss_closeMemoryRegion */

/**
 *  @b Description
 *  @n
 *      This function is used to obtain previously allocated descriptors.
 *      The function opens a destination queue as specified in the input parameters
 *      and pushes the requested number of descriptors if available onto it.
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  descCfg
 *      Specifies the number of descriptors, memory region from where it should be allocated
 *      and the destination queue to push to.
 *
 *  @param[out]  numAllocated
 *      Number of descriptors actually allocated.
 *
 *  @pre
 *      @ref Qmss_initSubSys function should be called before calling this function.
 *
 *  @retval
 *      Success -   Destination queue handle on which the allocated descriptors are stored.
 *                  The queue must be closed by the caller using Qmss_queueClose() when done.
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 *  @retval
 *      Failure -   QMSS_MEMREGION_NOT_INITIALIZED
 *  @retval
 *      Failure -   QMSS_QUEUE_OPEN_ERROR
 *  @retval
 *      Failure -   QMSS_RESOURCE_MEM_REGION_USE_DENIED
 */
Qmss_QueueHnd Qmss_initDescriptorSubSys (Qmss_SubSysHnd subSysHnd, Qmss_DescCfg *descCfg, uint32_t *numAllocated)
{
    uint32_t              count;
    uint32_t              *descPtr;
    uint8_t               isAllocated;
    int32_t               srcQueNum, srcQueHnd, destQueHnd;
    uint32_t              qGroup;
    uint32_t              subSys = (uint32_t)subSysHnd;
    Qmss_GlobalObj_Unpadded *gObjPtr = &qmssGObj.obj[subSys];

    /* Get the descriptor from the source queue */

    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    if (descCfg == NULL)
    {
        return QMSS_INVALID_PARAM;
    }

    /* Find the source queue given the memory region */
    if ((srcQueNum = Qmss_getMemRegQueueHandleSubSys (subSysHnd, descCfg->memRegion)) < 0)
    {
        return QMSS_MEMREGION_NOT_INITIALIZED;
    }

    if (qmssLObj[subSys].mode == Qmss_Mode_SPLIT)
    {
        /* In split mode, the config selects the group */
        qGroup = descCfg->queueGroup;
    }
    else
    {
        /* In joint mode, the LLD tracks configuration use group 0, but the
         * configuration is inserted into all of the group registers
         */
        qGroup = 0;
    }

    if (qGroup > gObjPtr->qmssGblCfgParams.maxQueMgrGroups)
    {
        return QMSS_INVALID_PARAM;
    }


    /* Find the descriptor size in the given the memory region */
    if ((Qmss_getMemRegDescSizeSubSys (subSysHnd, descCfg->memRegion, qGroup)) == 0)
    {
        return QMSS_MEMREGION_NOT_INITIALIZED;
    }

    /* Open destination queue specified in the configuration */
    if ((destQueHnd = Qmss_queueOpenInGroupSubSys (subSysHnd, qGroup, descCfg->queueType, descCfg->destQueueNum, &isAllocated)) < 0)
    {
        return QMSS_QUEUE_OPEN_ERROR;
    }

    /* Open general purpose queue on which descriptors are stored */
    if ((srcQueHnd = Qmss_queueOpenInGroupSubSys (subSysHnd, qGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, srcQueNum, &isAllocated)) < 0)
    {
        return QMSS_QUEUE_OPEN_ERROR;
    }
    *numAllocated = 0;
    for (count = 0; count < descCfg->descNum; count++)
    {
        /* Pop descriptor from source queue */
        if ((descPtr = Qmss_queuePop (srcQueHnd)) == NULL)
        {
            break;
        }

        /* Push descriptor to the specified destination queue */

        /* TODO QueueProxy is not working add this later
         * Qmss_queuePushDesc (destQueHnd, descPtr, descSize); */

        Qmss_queuePushDesc (destQueHnd, descPtr);
        *numAllocated += 1;
    }

    Qmss_queueClose (srcQueHnd);

    return destQueHnd;
} /* Qmss_initDescriptorSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_initDescriptorSubSys
 *      which operates on the global subsystem.
 */
Qmss_QueueHnd Qmss_initDescriptor (Qmss_DescCfg *descCfg, uint32_t *numAllocated)
{
    return Qmss_initDescriptorSubSys (QMSS_SUBSYS_HND_GLOBAL, descCfg, numAllocated);
} /* Qmss_initDescriptor */

/**
 *  @b Description
 *  @n
 *      This function opens the requested queue.
 *      A queue can be opened in two ways:
 *          1) If "queNum" is set to QMSS_PARAM_NOT_SPECIFIED, then a new
 *          available queue of type "queType" is allocated.
 *          2) If "queNum" is a valid queue number i.e., >= 0,
 *               then a queue is allocated if free
 *               else
 *               a handle to a previously opened queue is returned.
 *               In this case "queType" parameter is not used.
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  queGroup
 *      Specifies the queue manager group to find a queue.  This must be specified
 *      in SPLIT mode.  It may be specified (not QMSS_PARAM_NOT_SPECIFIED) in JOINT
 *      mode to allocate a queue in the same group as another queue.
 *
 *  @param[in]  queType
 *      Specifies the type of queue that should be opened.
 *
 *  @param[in]  queNum
 *      Specifies the queue number that should be opened.
 *
 *  @param[out]  isAllocated
 *      Indicates whether the requested queue is a new queue allocation(1).
 *      or was already allocated. If the queue was previously allocated this parameter returns
 *      the reference count.
 *
 *  @retval
 *      Success -   Queue Handle. Used as an input parameter for operation on this queue.
 *  @retval
 *      Failure -   QMSS_RESOURCE_ALLOCATE_USE_DENIED
 *  @retval
 *      Failure -   <0
 */
Qmss_QueueHnd Qmss_queueOpenInGroupSubSys (Qmss_SubSysHnd subSysHnd, int32_t queGroup, Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated)
{
    Qmss_QueueHnd   result;
    void            *key;
    uint32_t        openQGroup;
    int             qGroupIsSpecified;
    uint32_t        subSys = (uint32_t)subSysHnd;

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    /* Convert signed PARAM_NOT_SPECIFIED into two arguments.  0 is legal queue group. */
    openQGroup        = (uint32_t)queGroup;
    qGroupIsSpecified = 1;
    if (queGroup == QMSS_PARAM_NOT_SPECIFIED)
    {
        openQGroup = 0;
        qGroupIsSpecified = 0;
    }
    result = Qmss_internalQueueOpen (subSysHnd, qGroupIsSpecified, openQGroup, queType, queNum, isAllocated);

    /* End Critical Section */
    Qmss_osalCsExit (key);

    return (Qmss_QueueHnd) result;
} /* Qmss_queueOpenInGroupSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_queueOpenInGroupSubSys
 *      which operates on the global subsystem.
 */
Qmss_QueueHnd Qmss_queueOpenInGroup (int32_t queGroup, Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated)
{
    return Qmss_queueOpenInGroupSubSys (QMSS_SUBSYS_HND_GLOBAL, queGroup, queType, queNum, isAllocated);
} /* Qmss_queueOpenInGroup */

/**
 *  @b Description
 *  @n
 *      This function opens the requested queue from queue group 0.  For
 *      additional details, see @ref Qmss_queueOpenInGroupSubSys.
 *
 *      When operating in SPLIT mode, this function will allocate the queue
 *      from group 0.  To use other groups, must use @ref Qmss_queueOpenInGroup
 *      or @ref Qmss_queueOpenInGroupSubSys
 *
 *      When operating in JOINT mode, this function provides load balancing
 *      using the configured load balancing mode.
 *
 *  @retval
 *      Success -   Queue Handle. Used as an input parameter for operation on this queue.
 *  @retval
 *      Failure -   <0
 */
Qmss_QueueHnd Qmss_queueOpenSubSys (Qmss_SubSysHnd subSysHnd, Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated)
{
    return Qmss_queueOpenInGroupSubSys (subSysHnd, QMSS_PARAM_NOT_SPECIFIED, queType, queNum, isAllocated);
} /* Qmss_queueOpenSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_queueOpenSubSys
 *      which operates on the global subsystem.
 */
Qmss_QueueHnd Qmss_queueOpen (Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated)
{
    return Qmss_queueOpenSubSys (QMSS_SUBSYS_HND_GLOBAL, queType, queNum, isAllocated);
} /* Qmss_queueOpen */

/**
 *  @b Description
 *  @n
 *      This function opens the requested queue if a free queue is avialable within the given start and end range.
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  startQueNum
 *      Specifies the start of the queue number range.
 *
 *  @param[in]  endQueNum
 *      Specifies the end of the queue number range.
 *
 *  @param[out]  isAllocated
 *      Indicates whether the requested queue is a new queue allocation(1).
 *      or was already allocated. If the queue was previously allocated this parameter returns
 *      the reference count.
 *
 *  @retval
 *      Success -   Queue Handle. Used as an input parameter for operation on this queue.
 *
 *  @retval
 *      Failure -   <0
 */
Qmss_QueueHnd Qmss_queueOpenInRangeSubSys (Qmss_SubSysHnd subSysHnd, uint32_t startQueNum, uint32_t endQueNum, uint8_t *isAllocated)
{
    void               *key;
    uint32_t           index;
    int                foundQueue;
    int32_t            maxQueNumIdx;
    uint32_t           queTypeEnd;
    Qmss_QueueHnd      retVal = (Qmss_QueueHnd)(-1);
    uint32_t           subSys = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    if ((maxQueNumIdx = Qmss_internalFindMaxQueNumIdxByQue(subSysHnd, startQueNum)) < 0)
    {
        /* startQueNum does not map to valid queue type */
        return QMSS_INVALID_PARAM;
    }

    queTypeEnd = lObjPtr->maxQueueNum[QMSS_QUEUE_GROUP(startQueNum)][maxQueNumIdx].startIndex +
                 lObjPtr->maxQueueNum[QMSS_QUEUE_GROUP(startQueNum)][maxQueNumIdx].maxNum;
    if (endQueNum > queTypeEnd)
    {
        /* Queue range crosses queue type boundary */
        return QMSS_INVALID_PARAM;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    /* Invalidate qmssQueueFree Object */
    Qmss_osalBeginMemAccess ((void *) &qmssQueueFree[subSys][startQueNum], endQueNum - startQueNum);

    *isAllocated = 0;
    foundQueue = 0;

    for (index = startQueNum; index < endQueNum; index++)
    {
        /* If the queue is alread open in this context, skip it */
        if (! qmssQueueFree[subSys][index])
        {
            if (lObjPtr->qmRmServiceHandle)
            {
                /* Check with RM and see if somebody else has it */
                if (Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_USE,
                                   lObjPtr->maxQueueNum[QMSS_QUEUE_GROUP(index)][maxQueNumIdx].rmQueue, (int32_t *)&index, 1, 0, isAllocated))
                {
                    if (*isAllocated != 1)
                    {
                        /* Don't like this queue, some other context got it, give it back */
                        if (! Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                                             lObjPtr->maxQueueNum[QMSS_QUEUE_GROUP(index)][maxQueNumIdx].rmQueue, (int32_t *)&index, 1, 0, isAllocated))
                        {
                            retVal = QMSS_RM_INTERNAL_ERROR;
                            goto exitCs;
                        }
                    }
                    else
                    {
                        /* Nobody else has this queue, take it */
                        foundQueue = 1;
                    }
                }
            }
            else
            {
                /* Non-RM case */
                *isAllocated = 1;
                foundQueue = 1;
            }
        }

        if (foundQueue)
        {
            qmssQueueFree[subSys][index]++;
            /* Writeback qmssQueueFree Object */
            Qmss_osalEndMemAccess ((void *) &qmssQueueFree[subSys][index], QMSS_MAX_CACHE_ALIGN);

            /* Balance the groups */
            Qmss_internalUpdateGroupControl (subSys, QMSS_QUEUE_GROUP(index), -1);

            retVal = (Qmss_QueueHnd) index;

            break;
        }
    }

exitCs:
    /* End Critical Section */
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_queueOpenInRangeSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_queueOpenInRangeSubSys
 *      which operates on the global subsystem.
 */
Qmss_QueueHnd Qmss_queueOpenInRange (uint32_t startQueNum, uint32_t endQueNum, uint8_t *isAllocated)
{
    return Qmss_queueOpenInRangeSubSys (QMSS_SUBSYS_HND_GLOBAL, startQueNum, endQueNum, isAllocated);
} /* Qmss_queueOpenInRange */

/**
 *  @b Description
 *  @n
 *      This function opens a contiguous block of queues of type queType aligned
 *      to align.  This API may fail even if sufficient queues are available
 *      due to fragmentation; thus it should be called soon after boot.
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[out] queueHnds
 *      List of queue handles.  This must be pre-allocated to at least
 *      numQ entries.
 *
 *  @param[in]  queGroup
 *      Specifies the queue manager group to find a queue.  This must be specified
 *      in SPLIT mode.  It may be specified (not QMSS_PARAM_NOT_SPECIFIED) in JOINT
 *      mode to allocate a queue in the same group as another queue.
 *
 *  @param[in]  queType
 *      Specifies the type of queues that should be opened.
 *
 *  @param[in]  numQ
 *      Specifies the number of queues that should be opened
 *
 *  @param[in]  align
 *      Queue number of first queue shall be such that queue number % align == 0.
 *
 *  @retval
 *      Success -   Queue Handle of first queue
 *  @retval
 *      Failure -   QMSS_RESOURCE_ALLOCATE_USE_DENIED
 *  @retval
 *      Failure -   <0
 */
Qmss_QueueHnd Qmss_queueBlockOpenInGroupSubSys (Qmss_SubSysHnd subSysHnd, Qmss_QueueHnd *queueHnds, int32_t queGroup, Qmss_QueueType queType, int32_t numQ, int32_t align)
{
    int queueNum, baseQueue, firstQ, lastQ;
    uint8_t isAllocated;
    void *key;
    Qmss_QueueHnd errorCode;
    int errorFound;
    uint32_t openQGroup;
    int qGroupIsSpecified;
    Qmss_QueueHnd retVal = QMSS_SOK;
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;
    int32_t maxQueNumIdx;

    if (! queueHnds)
    {
        return QMSS_INVALID_PARAM;
    }

    if (queType == QMSS_PARAM_NOT_SPECIFIED)
    {
        return QMSS_INVALID_PARAM;
    }

    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }


    /* Convert signed PARAM_NOT_SPECIFIED into two arguments.  0 is legal queue group. */
    openQGroup        = (uint32_t)queGroup;
    qGroupIsSpecified = 1;
    if (queGroup == QMSS_PARAM_NOT_SPECIFIED)
    {
        openQGroup = 0;
        qGroupIsSpecified = 0;
    }

    maxQueNumIdx = Qmss_internalFindMaxQueNumIdxByType (
                      subSysHnd,
                      queType,
                      openQGroup,
                      0);

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    if (lObjPtr->qmRmServiceHandle)
    {
        queueHnds[0] = QMSS_RESOURCE_ALLOCATE_USE_DENIED;
        baseQueue = RM_RESOURCE_BASE_UNSPECIFIED;
        if (Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_USE,
                           lObjPtr->maxQueueNum[openQGroup][maxQueNumIdx].rmQueue, (int32_t *)&baseQueue, numQ, align, 
                           &isAllocated))
        {
            /* Invalidate qmssQueueFree Object */
            Qmss_osalBeginMemAccess ((void *) &qmssQueueFree[subSys][baseQueue], numQ);
            for (queueNum = 0; queueNum < numQ; queueNum++)
            {
                int thisQueue = baseQueue++;
                queueHnds[queueNum] = (Qmss_QueueHnd) thisQueue;
                if (qmssQueueFree[subSys][thisQueue])
                {
                    /* book keeping in qmssQueueFree mismatches RM */
                    retVal = QMSS_RM_INTERNAL_ERROR;
                }
                qmssQueueFree[subSys][thisQueue]++;
            }
            /* writeback qmssQueueFree Object */
            Qmss_osalEndMemAccess ((void *) &qmssQueueFree[subSys][baseQueue], numQ);
            Qmss_internalUpdateGroupControl (subSys, openQGroup, -numQ);
        }
    }
    else
    {
        firstQ = lObjPtr->maxQueueNum[openQGroup][maxQueNumIdx].startIndex;
        lastQ  = lObjPtr->maxQueueNum[openQGroup][maxQueNumIdx].startIndex + lObjPtr->maxQueueNum[openQGroup][maxQueNumIdx].maxNum;
        queueHnds[0] = QMSS_QUEUE_OPEN_ERROR;


        /* Allocate a contigous block of numQ queues aligned to align */
        for (baseQueue = ((firstQ + align - 1) / align) * align;
             baseQueue < (lastQ - numQ);
             baseQueue += align)
        {
            for (queueNum = 0; queueNum < numQ; queueNum++)
            {
                queueHnds[queueNum] =
                    Qmss_internalQueueOpen (subSysHnd, qGroupIsSpecified, openQGroup, queType, baseQueue + queueNum, &isAllocated);
                errorFound = 0;
                if (queueHnds[queueNum] < 0)
                {
                    /* Open failed close the rest of the queues (except error queue) */
                    errorCode = queueHnds[queueNum];
                    queueNum--;
                    errorFound = 1;
                }
                if (isAllocated != 1)
                {
                    /* Somebody else got the queue. Close what we got, and try next range */
                    errorCode = QMSS_QUEUE_OPEN_ERROR;
                    errorFound = 1;
                }
                if (errorFound)
                {
                    /* Close any nonerrored queues */
                    for (; queueNum >= 0; queueNum--)
                    {
                        Qmss_internalQueueClose (queueHnds[queueNum]);
                        queueHnds[queueNum] = errorCode;
                    }
                    break;
                }
            }
            if (queueNum == numQ)
            {
                break; /* successful */
            }
        }

    }

    Qmss_osalCsExit (key);

    if (retVal == QMSS_SOK)
    {
        retVal = queueHnds[0];
    }

    return retVal;
} /* Qmss_queueBlockOpenInGroupSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_queueBlockOpenInGroupSubSys
 *      which operates on the global subsystem.
 */
Qmss_QueueHnd Qmss_queueBlockOpenInGroup (Qmss_QueueHnd *queueHnds, int32_t queGroup, Qmss_QueueType queType, int32_t numQ, int32_t align)
{
    return Qmss_queueBlockOpenInGroupSubSys (QMSS_SUBSYS_HND_GLOBAL, queueHnds, queGroup, queType, numQ, align);
} /* Qmss_queueBlockOpenInGroup */


/**
 *  @b Description
 *  @n
 *      This function opens a contiguous block of queues of type queType aligned
 *      to align in queue group 0.  This API may fail even if sufficient queues are available
 *      due to fragmentation; thus it should be called soon after boot.
 *
 *  @param[in]   subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[out] queueHnds
 *      List of queue handles.  This must be pre-allocated to at least
 *      numQ entries.
 *
 *  @param[in]  queType
 *      Specifies the type of queues that should be opened.
 *
 *  @param[in]  numQ
 *      Specifies the number of queues that should be opened
 *
 *  @param[in]  align
 *      Queue number of first queue shall be such that queue number % align == 0.
 *
 *  @retval
 *      Success -   Queue Handle of first queue
 *  @retval
 *      Failure -   QMSS_RESOURCE_ALLOCATE_USE_DENIED
 *  @retval
 *      Failure -   <0
 */
Qmss_QueueHnd Qmss_queueBlockOpenSubSys (Qmss_SubSysHnd subSysHnd, Qmss_QueueHnd *queueHnds, Qmss_QueueType queType, int32_t numQ, int32_t align)
{
    return Qmss_queueBlockOpenInGroup (queueHnds, QMSS_PARAM_NOT_SPECIFIED, queType, numQ, align);
} /* Qmss_queueBlockOpenSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_queueBlockOpenSubSys
 *      which operates on the global subsystem.
 */
Qmss_QueueHnd Qmss_queueBlockOpen (Qmss_QueueHnd *queueHnds, Qmss_QueueType queType, int32_t numQ, int32_t align)
{
    return Qmss_queueBlockOpenSubSys (QMSS_SUBSYS_HND_GLOBAL, queueHnds, queType, numQ, align);
} /* Qmss_queueBlockOpen */

/**
 *  @b Description
 *  @n
 *      This function opens the specified queue if and only if it is already open elsewhere.
 *      OpenUse means "open to use queue already allocated by someone else".
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  QID
 *      Specifies the queue number that should be opened.
 *
 *  @param[out]  useCount
 *      Reference count, which must be >= 2.
 *
 *  @retval
 *      Success -   Queue Handle
 *  @retval
 *      Failure -   QMSS_RESOURCE_ALLOCATE_USE_DENIED
 *      Failure -   QMSS_QUEUE_NOT_ALREADY_OPEN
 *  @retval
 *      Failure -   <0
 */
Qmss_QueueHnd Qmss_queueOpenUseSubSys (Qmss_SubSysHnd subSysHnd, uint32_t QID, uint8_t *useCount)
{
    Qmss_QueueHnd        retVal;
    void                *key;
    int                  tryOpening;
    uint32_t             subSys = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;


    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    /* Invalidate qmssQueueFree before using */
    Qmss_osalBeginMemAccess ((void *) &qmssQueueFree[subSys][QID], QMSS_MAX_CACHE_ALIGN);

    if (qmssQueueFree[subSys][QID] == 0)
    {
        if (lObjPtr->qmRmServiceHandle)
        {
            /* implementation of RM requires trial and error */
            tryOpening = 1;
        }
        else
        {
            /* No need to try the queue -- its not already open */
            tryOpening = 0;
            retVal     = QMSS_QUEUE_NOT_ALREADY_OPEN;
        }
    } else {
        /* Queue is already open in this context */
        tryOpening = 1;
    }

    if (tryOpening)
    {
        retVal = Qmss_internalQueueOpen (subSys, 0, 0, (Qmss_QueueType)QMSS_PARAM_NOT_SPECIFIED, QID, useCount);
        if ((retVal >= QMSS_SOK) && (*useCount == 1) && (qmssQueueFree[subSys][QID] == 1))
        {
            /* The qmssQueueFree[subSys][QID] isn't needed if rm returns use count instead of owner count */
            Qmss_internalQueueClose (retVal);
            *useCount = 0;
            retVal    = QMSS_QUEUE_NOT_ALREADY_OPEN;
        }
        else
        {
            if (*useCount == 1)
            {
                *useCount = 2; /* spoof 2 for RM */
            }
        }
    }

    /* End Critical Section */
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_queueOpenUseSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_queueOpenUseSubSys
 *      which operates on the global subsystem.
 */
Qmss_QueueHnd Qmss_queueOpenUse (uint32_t QID, uint8_t *useCount)
{
    return Qmss_queueOpenUseSubSys (QMSS_SUBSYS_HND_GLOBAL, QID, useCount);
} /* Qmss_queueOpenUse */

/* Internal function to close a queue, must be inside a critical section */
Qmss_Result Qmss_internalQueueClose (Qmss_QueueHnd hnd)
{
    uint8_t              refCount;
    Qmss_Result          retVal = QMSS_INVALID_PARAM;
    uint32_t             subSys = QMSS_QUEUE_SUBSYS(hnd);
    uint32_t             QID    = QMSS_QUEUE_QID(hnd);
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;


    if (lObjPtr->qmRmServiceHandle)
    {
        int32_t maxQueNumIdx;

        /* Get the queue type */
        maxQueNumIdx = Qmss_internalFindMaxQueNumIdxByQue(QMSS_QUEUE_SUBSYS(hnd), QID);

        if (Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE,
                           lObjPtr->maxQueueNum[QMSS_QUEUE_GROUP(hnd)][maxQueNumIdx].rmQueue, (int32_t *)&QID, 1, 0, &refCount))
        {
            retVal = refCount;
        }
        else
        {
            retVal = QMSS_RESOURCE_FREE_DENIED;
            goto exit; /* close failed don't do internal book keeping */
        }
    }

    /* Invalidate qmssQueueFree Object */
    Qmss_osalBeginMemAccess ((void *) &qmssQueueFree[subSys][QID], QMSS_MAX_CACHE_ALIGN);

    if (qmssQueueFree[subSys][QID] == 0)
    {
        if (lObjPtr->qmRmServiceHandle)
        {
            /* Out of sync with RM */
            retVal = QMSS_RM_INTERNAL_ERROR;
        }
        else
        {
            /* No RM, close failed */
            retVal = QMSS_INVALID_PARAM;
        }
    }
    else
    {
        qmssQueueFree[subSys][QID]--;

        if (! qmssQueueFree[subSys][QID])
        {
            /* When last reference is freed, increase free queues for group */
            Qmss_internalUpdateGroupControl (subSys, QMSS_QUEUE_GROUP(hnd), +1);
        }

        /* Writeback qmssQueueFree Object */
        Qmss_osalEndMemAccess ((void *) &qmssQueueFree[subSys][QID], QMSS_MAX_CACHE_ALIGN);

        if (! lObjPtr->qmRmServiceHandle)
        {
            /* When using RM, use the RM's owner count */
            retVal = qmssQueueFree[subSys][QID];
        }
    }

exit:
    return retVal;
} /* Qmss_internalQueueClose */

/**
 *  @b Description
 *  @n
 *      This function closes a queue.
 *      The queue reference count is decremented. The queue is freed only if the
 *      reference count is zero.
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @pre
 *      Qmss_queueOpenSubSys function should be called before calling this function.
 *
 *  @post
 *      Queue is freed if the reference count is zero and available for reallocation.
 *
 *  @retval
 *      Success -   Current reference count.
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 */
Qmss_Result Qmss_queueClose (Qmss_QueueHnd hnd)
{
    void        *key;
    Qmss_Result retVal;

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalCsEnter ();

    retVal = Qmss_internalQueueClose (hnd);

    /* End Critical Section */
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_queueClose */

/**
 *  @b Description
 *  @n
 *      This function returns the threshold at which the queue threshold pin is asserted
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @pre
 *      Qmss_queueOpenSubSys function should be called before calling this function.
 *
 *  @retval
 *      Queue threshold
 */
uint32_t Qmss_getQueueThreshold (Qmss_QueueHnd hnd)
{
    /* No RM check since read only */

    uint32_t qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    return (uint32_t) CSL_FEXT (qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueStatReg->QUEUE_STATUS_CONFIG_GROUP[qNumber].QUEUE_STATUS_CONFIG_REG_D,
                    QM_QUEUE_STATUS_CONFIG_QUEUE_STATUS_CONFIG_REG_D_THRESHOLD);
} /* Qmss_getQueueThreshold */

/**
 *  @b Description
 *  @n
 *      This function sets the threshold at which the queue threshold pin is asserted and
 *      threshold bit in threshold status register is set.
 *      "hilo" indicates whether the number of items in the queue should be greater than/equal to
 *      OR less than the confgiured threshold value before the queue ecnt status bit is asserted and
 *      threshold bit in threshold status register is set.
 *      The threshold value is 10'h3ff when it is ten or higher. It is (2^threshold-1)  for other values.
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @param[in]  hilo
 *      1 - High
 *      0 - Low.
 *
 *  @param[in]  threshold
 *      Threshold value.
 *
 *  @pre
 *      Qmss_queueOpenSubSys function should be called before calling this function.
 *
 *  @retval
 *      Qmss_Result
 */
Qmss_Result Qmss_setQueueThreshold (Qmss_QueueHnd hnd, uint16_t hilo, uint8_t threshold)
{
    uint32_t           qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t           qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t           qSubSys = QMSS_QUEUE_SUBSYS(hnd);
    uint32_t           temp = 0;

    if (threshold > 10)
        threshold = 10;
    CSL_FINS (temp, QM_QUEUE_STATUS_CONFIG_QUEUE_STATUS_CONFIG_REG_D_THRESHOLD_HILO, hilo);
    CSL_FINS (temp, QM_QUEUE_STATUS_CONFIG_QUEUE_STATUS_CONFIG_REG_D_THRESHOLD, threshold);

    qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueStatReg->QUEUE_STATUS_CONFIG_GROUP[qNumber].QUEUE_STATUS_CONFIG_REG_D = temp;
    return QMSS_SOK;
} /* Qmss_setQueueThreshold */

/**
 *  @b Description
 *  @n
 *      API to return a group of 4 starvation count available in the group
 *
 *      Note: if handle is unaligned to 4 queues, it will be aligned.  Thus
 *      if hnd%4 = 2, the requested result will be in pStarvationCounts[2],
 *      not pStarvationCounts[0].
 *
 *  @param[in]  hnd
 *      Queue handle.
 *      Number of starvation counts from the group
 *      Array of starvation counts. Max 4
 *
 *  @param[in] numCounts
 *      Maximum number of elements in pStarvationCounts
 *
 *  @param[out] pStarvationCounts
 *      Array of starvation counts (return value)
 *
 *  @pre
 *      Qmss_queueOpenSubSys function should be called before calling this function.
 *
 *  @retval
 *      None
 */
Qmss_Result Qmss_getStarvationCounts (Qmss_QueueHnd hnd,
                                      uint32_t      numCounts,
                                      uint32_t*     pStarvationCounts)
{
    uint32_t starvationCount;
    uint32_t qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t queNum  = QMSS_QUEUE_NUMBER(hnd);
    uint32_t qSubSys = QMSS_QUEUE_SUBSYS(hnd);
    uint32_t regIndex;
    int32_t maxQueNumIdx;

    /* No RM check since read only */
    maxQueNumIdx = Qmss_internalFindMaxQueNumIdxByType (
                      (Qmss_SubSysHnd)qSubSys,
                      Qmss_QueueType_STARVATION_COUNTER_QUEUE,
                      qGroup,
                      0);
    if (maxQueNumIdx < 0)
    {
        return QMSS_INVALID_QUEUE_TYPE;
    }

    queNum -= QMSS_QUEUE_NUMBER(qmssLObj[qSubSys].p.maxQueueNum[qGroup][maxQueNumIdx].startIndex);
    regIndex = queNum / 4; /* this will align an queue number to 4 queues */

    starvationCount =
      qmssLObj[qSubSys].p.groupRegs[qGroup].qmConfigReg->FREE_DESCRIPTOR_STARVE_COUNT_REG[regIndex];
    if (numCounts >= 1)
    {
        pStarvationCounts[0] = (uint8_t)CSL_FEXTR (starvationCount, 7, 0);
    }
    if (numCounts >= 2)
    {
        pStarvationCounts[1] = (uint8_t)CSL_FEXTR (starvationCount, 15, 8);
    }
    if (numCounts >= 3)
    {
        pStarvationCounts[2] = (uint8_t)CSL_FEXTR (starvationCount, 23, 16);
    }
    if (numCounts >= 4)
    {
        pStarvationCounts[3] = (uint8_t)CSL_FEXTR (starvationCount, 31, 24);
    }

    return QMSS_SOK;
} /* Qmss_getStarvationCounts */

/**
 *  @b Description
 *  @n
 *      The starvation count is incremented every time the Free Descriptor/Buffer
 *      queue is read when empty. This function returns the starvation count of queue.
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @pre
 *      Qmss_queueOpenSubSys function should be called before calling this function.
 *
 *  @retval
 *      8 bit Starvation count or 0xffffffff on error;
 */
uint32_t Qmss_getStarvationCount (Qmss_QueueHnd hnd)
{
    uint32_t queNum = QMSS_QUEUE_NUMBER(hnd);
    uint32_t regIndex;
    uint32_t starvationCount[4];

    /* No RM check since read only */
    if (Qmss_getStarvationCounts(hnd,sizeof(starvationCount)/sizeof(uint32_t),starvationCount) != QMSS_SOK)
    {
        return 0xffffffff;
    }
    regIndex = queNum % 4;
    return (uint32_t)(starvationCount[regIndex]);
} /* Qmss_getStarvationCount */

/**
 *  @b Description
 *  @n
 *      This function returns the threshold status of the queue. There are 2 conditions under which the
 *      threshold bit is set
 *
 *      The threshold bit is set for a queue if the number of element in a queue is above or below
 *      a certain threshold number of items configured using Qmss_setQueueThreshold() API.
 *
 *      The threshold bit is set for a queue if there is atleast 1 element on the queue when the threshold
 *      is not set Qmss_setQueueThreshold() API
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @pre
 *      Qmss_queueOpenSubSys function should be called before calling this function.
 *
 *  @retval
 *      0 - Configured threshold is not met
 *  @retval
 *      1 - Configured threshold is met
 */
uint16_t Qmss_getQueueThresholdStatus (Qmss_QueueHnd hnd)
{
    uint32_t qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t qSubSys = QMSS_QUEUE_SUBSYS(hnd);
    uint32_t regIndex, bitField;

    /* No RM check since read only */

    regIndex = qNumber / 32;
    bitField = qNumber % 32;

    return (uint16_t) CSL_FEXTR (qmssLObj[qSubSys].p.groupRegs[qGroup].qmStatusRAM->QUEUE_THRESHOLD_STATUS_REG[regIndex], bitField, bitField);
} /* Qmss_getQueueThresholdStatus */

/**
 *  @b Description
 *  @n
 *      Given the queue handle the function returns the queue manager and queue number within
 *      the queue manager corresponding to the handle
 *
 *  @param[in]  hnd
 *      Queue handle
 *
 *  @retval
 *      Queue Manager and Queue Number within the Queue Manager
 */
Qmss_Queue Qmss_getQueueNumber (Qmss_QueueHnd hnd)
{
    Qmss_Queue              queue;

    /* No RM check since read only */

    queue.qMgr = CSL_FEXTR (hnd, 13, 12);
    queue.qNum = CSL_FEXTR (hnd, 11, 0);
    return (queue);
} /* Qmss_getQueueNumber */

/**
 *  @b Description
 *  @n
 *      Given the queue handle the function returns the queue manager
 *      group containing the queue.  This can be used to allocate other
 *      queues in the same group as a particular handle.  This is required
 *      if it is necessary to use @ref Qmss_queueDivert on the pair of
 *      queues.
 *
 *  @param[in]  hnd
 *      Queue handle
 *
 *  @retval
 *      Queue group
 */
uint32_t Qmss_getQueueGroup (Qmss_QueueHnd hnd)
{
    /* No RM check since read only */
    return (QMSS_QUEUE_GROUP(hnd));
} /* Qmss_getQueueGroup */

/**
 *  @b Description
 *  @n
 *      Given the queue manager and queue number within the queue manager
 *      the function returns the corresponding queue handle
 *
 *  @param[in]  queue
 *      Queue Manager number and Queue Number within the Queue Manager
 *
 *  @retval
 *      Queue handle corresponding to the queue manager and queue number
 */
Qmss_QueueHnd Qmss_getQueueHandle (Qmss_Queue queue)
{
    Qmss_QueueHnd           hnd = 0;

    /* No RM check since read only */

    CSL_FINSR (hnd, 13, 12, queue.qMgr);
    CSL_FINSR (hnd, 11, 0, queue.qNum);
    return (hnd);
} /* Qmss_getQueueHandle */

/**
 *  @b Description
 *  @n
 *      Downloads the PDSP firmware to PDSP.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      PDSP number where the firmware should be downloaded
 *
 *  @param[in]  image
 *      Pointer to the firmware image
 *
 *  @param[in]  size
 *      Size of firmware image in bytes
 *
 *  @retval
 *      Success -   Firmware is downloaded to PDSP
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure -   QMSS_INVALID_PARAM
 *  @retval
 *      Failure -   QMSS_RESOURCE_ALLOCATE_USE_DENIED
 */
Qmss_Result Qmss_downloadFirmwareSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, void *image, uint32_t size)
{
    uint32_t subSys = (uint32_t)subSysHnd;

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    if ((image == NULL) || (size == 0))
        return QMSS_INVALID_PARAM;

    return (Qmss_internaldownloadFirmware (subSysHnd, pdspId, image, size));
} /* Qmss_downloadFirmwareSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_downloadFirmwareSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_downloadFirmware (Qmss_PdspId pdspId, void *image, uint32_t size)
{
    return Qmss_downloadFirmwareSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, image, size);
} /* Qmss_downloadFirmware */

static Qmss_Result Qmss_internaldownloadFirmware (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, void *image, uint32_t size)
{
    uint32_t           i, count;
    volatile uint32_t  enable;
    uint32_t           *data = (uint32_t *) image;
    uint32_t           subSys = (uint32_t)subSysHnd;

    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;

    if (lObjPtr->qmRmServiceHandle)
    {
        int32_t pdspIdNum = (int32_t)pdspId;
        /* Check Firmware use privileges */
        if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_USE,
                            lObjPtr->rmFirmwarePdsp, &pdspIdNum, 1, 0, NULL))
        {
            return(QMSS_RESOURCE_ALLOCATE_USE_DENIED);
        }
        /* Check Firmware init privileges */
        if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_INIT,
                            lObjPtr->rmFirmwarePdsp, &pdspIdNum, 1, 0, NULL))
        {
            /* Use but don't init - Check firmware revision */
        }
    }

    /* Reset the PDSP */
    CSL_FINS (lObjPtr->regs.qmPdspCtrlReg[pdspId]->PDSP_CONTROL_REG, PDSP_PDSP_CONTROL_REG_PDSP_ENABLE, 0);

    /* Confirm PDSP has halted */
    do
    {
        enable = CSL_FEXT (lObjPtr->regs.qmPdspCtrlReg[pdspId]->PDSP_CONTROL_REG, PDSP_PDSP_CONTROL_REG_PDSP_STATE);
    }while (enable);

    count = size / 4;

    /* Some firmwares need the 1 based PDSP number written to scratch + 0x18 */
    *(lObjPtr->regs.qmPdspCmdReg[pdspId] + 6) = pdspId + 1;

    /* Download the firmware */
    for(i = 0; i < count; i++)
        lObjPtr->regs.qmPdspIRamReg[pdspId][i] = data[i];

    /* Use the command register to sync the PDSP */
    *lObjPtr->regs.qmPdspCmdReg[pdspId] = 0xFFFFFFFF;

    /* Wait to the memory write to land */
    for(i = 0; i < 20000 && *lObjPtr->regs.qmPdspCmdReg[pdspId] != 0xFFFFFFFF; i++);

    /* Reset program counter to zero */
    CSL_FINS (lObjPtr->regs.qmPdspCtrlReg[pdspId]->PDSP_CONTROL_REG, PDSP_PDSP_CONTROL_REG_PCOUNTER_RST_VAL, 0);

    /* Set soft reset to zero to load the program counter */
    CSL_FINS (lObjPtr->regs.qmPdspCtrlReg[pdspId]->PDSP_CONTROL_REG, PDSP_PDSP_CONTROL_REG_SOFT_RST_N, 0);

    /* Enable the PDSP */
    CSL_FINS (lObjPtr->regs.qmPdspCtrlReg[pdspId]->PDSP_CONTROL_REG, PDSP_PDSP_CONTROL_REG_PDSP_ENABLE, 1);

    /* Wait for the command register to clear */
    while (*lObjPtr->regs.qmPdspCmdReg[pdspId]);

    return QMSS_SOK;
} /* Qmss_internaldownloadFirmware */

/**
 *  @b Description
 *  @n
 *      This function sets the end of interrupt vector value for the specified interrupt.
 *      This API is used by the host to indicate
 *          - The end of interrupt processing to the accumulator. The accumulator
 *              is then free to write to the accumulator ping/pong page released by the host.
 *          - The end of interrupt processing for buffer descriptor starvation event on
 *              receive SOP and MOP for any of the RX DMA units in the CDMA.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.

 *  @param[in]  intdNum
 *      Which QMSS INTD to program.  As this is a fast path API it is NOT bounds checked.
 *
 *
 *  @param[in]  type
 *      Interrupt  to program.
 *          Qmss_IntdInterruptType_HIGH selects the interrupts corresponding to the high priority accumulator
 *          Qmss_IntdInterruptType_LOW selects the interrupts corresponding to the low priority accumulator
 *          Qmss_IntdInterruptType_CDMA selects the interrupts corresponding to the low priority accumulator
 *
 *  @param[in]  accumCh
 *      Accumulator channel which is mapped to interrupt number via Qmss_xxxEoiVector mapping
 *      table.  The resulting interrupt number is set.
 *
 *      **No validation is done to check if the interrupt number is within range to reduce overhead.**
 *
 *  @post
 *      EOI vector value is set indicating end of host interrupt processing.
 *
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED, QMSS_INVALID_PARAM
 */

Qmss_Result Qmss_setEoiVectorByIntdSubSys (Qmss_SubSysHnd subSysHnd, uint8_t intdNum, Qmss_IntdInterruptType type, uint8_t accumCh)
{
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;
    CSL_Qm_intdRegs *intd;

    if (qmssLObjIsValid[subSys] == 0)
    {


        return QMSS_NOT_INITIALIZED;
    }

    if (subSys > QMSS_MAX_SUBSYS)
    {
        return QMSS_INVALID_PARAM;
    }

    if (! lObjPtr->regs.qmQueIntdReg[0])
    {
        return QMSS_SUBSYS_UNSUPPORTED; /* Not supported on this subsys */
    }

    intd = lObjPtr->regs.qmQueIntdReg[intdNum];

    /* It is assumed that all intd's share the same channel to vector map.  If this ever
     * varies, Qmss_*EoiVector[] needs to be indexted by intd as well */
    if (type == Qmss_IntdInterruptType_HIGH)
        CSL_FINS (intd->EOI_REG, QM_INTD_EOI_REG_EOI_VECTOR, Qmss_highEoiVector[accumCh]);
    else if (type == Qmss_IntdInterruptType_LOW)
        CSL_FINS (intd->EOI_REG, QM_INTD_EOI_REG_EOI_VECTOR, Qmss_lowEoiVector[accumCh]);
    else
        CSL_FINS (intd->EOI_REG, QM_INTD_EOI_REG_EOI_VECTOR, Qmss_cdmaEoiVector[accumCh]);
    return QMSS_SOK;
} /* Qmss_setEoiVectorByIntdSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_setEoiVectorByIntdSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_setEoiVectorByIntd (uint8_t intdNum, Qmss_IntdInterruptType type, uint8_t accumCh)
{
    return Qmss_setEoiVectorByIntdSubSys (QMSS_SUBSYS_HND_GLOBAL, intdNum, type, accumCh);
} /* Qmss_setEoiVectorByIntd */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_setEoiVectorByIntdSubSys
 *      which operates on the first intd of the specified subsystem.
 */
Qmss_Result Qmss_setEoiVectorSubSys (Qmss_SubSysHnd subSysHnd, Qmss_IntdInterruptType type, uint8_t accumCh)
{
    return Qmss_setEoiVectorByIntdSubSys (subSysHnd, 0, type, accumCh);
} /* Qmss_setEoiVectorSubSys */

/**
 *  @b Description
 *  @n
 *      Calls @ref Qmss_setEoiVectorByIntd for the first intd in global QMSS
 *      (for backwards compatibility).
 *
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 */
Qmss_Result Qmss_setEoiVector (Qmss_IntdInterruptType type, uint8_t accumCh)
{
    return Qmss_setEoiVectorByIntdSubSys (QMSS_SUBSYS_HND_GLOBAL, 0, type, accumCh);
} /* Qmss_setEoiVector */

/**
 *  @b Description
 *  @n
 *      This function sets the end of interrupt vector value for the specified interrupt.
 *      This API is used by the host to indicate
 *              host is done processing interrupt number intNum.  This API is normally
 *              not used.  See @ref Qmss_setEoiVectorByIntd.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  intdNum
 *      Which QMSS INTD to program
 *
 *  @param[in]  intNum
 *      Interrupt number to program.
 *
 *  @post
 *      EOI vector value is set indicating end of host interrupt processing.
 *
 *  @retval
 *      Success - QMSS_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 */
Qmss_Result Qmss_setEoiIntNumByIntdSubSys (Qmss_SubSysHnd subSysHnd, uint8_t intdNum, uint8_t intNum)
{
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    if (! lObjPtr->regs.qmQueIntdReg[0])
    {
        return QMSS_SUBSYS_UNSUPPORTED; /* Not supported on this subsys */
    }

    CSL_FINS (lObjPtr->regs.qmQueIntdReg[intdNum]->EOI_REG, QM_INTD_EOI_REG_EOI_VECTOR, intNum);

    return QMSS_SOK;
} /* Qmss_setEoiIntNumByIntdSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_setEoiIntNumSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_setEoiIntNumByIntd (uint8_t intdNum, uint8_t intNum)
{
    return Qmss_setEoiIntNumByIntdSubSys (QMSS_SUBSYS_HND_GLOBAL, intdNum, intNum);
}

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_setEoiIntNumByIntdSubSys
 *      which operates on the first intd of the specified subsystem.
 */
Qmss_Result Qmss_setEoiIntNumSubSys (Qmss_SubSysHnd subSysHnd, uint8_t intNum)
{
    return Qmss_setEoiIntNumByIntdSubSys (subSysHnd, 0, intNum);
}

/**
 *  @b Description
 *  @n Calls @ref Qmss_setEoiIntNumByIntdSubSys on first intd of the
 *  global subsystem (for backwards compatibility)
 *
 *  @retval
 *      Success - QMSS_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 */
Qmss_Result Qmss_setEoiIntNum (uint8_t intNum)
{
    return Qmss_setEoiIntNumByIntdSubSys (QMSS_SUBSYS_HND_GLOBAL, 0, intNum);
} /* Qmss_setEoiIntNum */

/**
 *  @b Description
 *  @n
 *      This function decrements the interrupt count. The Interrupt Count Registers contains a
 *      count of the interrupts that have triggered and not been processed. Writing a non-zero
 *      value to the register subtracts that value from the count. Writing a 0 clears the count.
 *      The count saturates at 3.
 *
 *      Note: Clearing the count does not clear the interrupt. The EOI Vector must be set using Qmss_setEoiVector API.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.

 *  @param[in]  intdNum
 *      Which QMSS INTD to program.  As this is a fast path API it is NOT bounds checked.
 *
 *
 *  @param[in]  interruptNum
 *      Interrupt Number that should be acknowledged.
 *
 *  @param[in]  value
 *      Value to decrement by or zero to clear.
 *
 *      **No validation is done to check if the interrupt number is within range to reduce overhead.**
 *
 *  @post
 *      Count is decremented or cleared depending on input parameter.
 *
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 */
Qmss_Result Qmss_ackInterruptByIntdSubSys (Qmss_SubSysHnd subSysHnd, uint8_t intdNum, uint8_t interruptNum, uint8_t value)
{
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;

    if (! lObjPtr->regs.qmQueIntdReg[0])
    {
        return QMSS_SUBSYS_UNSUPPORTED; /* Not supported on this subsys */
    }

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    lObjPtr->regs.qmQueIntdReg[intdNum]->INTCNT_REG[interruptNum] = value;

    return QMSS_SOK;
} /* Qmss_ackInterruptByIntdSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_ackInterruptByIntdSubSys
 *      which operates on the specified rst intd of the global subsystem.
 */
Qmss_Result Qmss_ackInterruptByIntd (uint8_t intdNum, uint8_t interruptNum, uint8_t value)
{
    return Qmss_ackInterruptByIntdSubSys (QMSS_SUBSYS_HND_GLOBAL, intdNum, interruptNum, value);
} /* Qmss_ackInterruptByIntd */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_ackInterruptByIntdSubSys
 *      which operates on the first intd of the specified subsystem.
 */
Qmss_Result Qmss_ackInterruptSubSys (Qmss_SubSysHnd subSysHnd, uint8_t interruptNum, uint8_t value)
{
    return Qmss_ackInterruptByIntdSubSys (subSysHnd, 0, interruptNum, value);
} /* Qmss_ackInterruptSubSys */

/**
 *  @b Description
 *  @n Calls @ref Qmss_ackInterruptByIntdSubSys with intdNum = 0 (for backwards compatibility)
 *     on global subsystem.
 *
 *  @retval
 *      Success - QMSS_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 */
Qmss_Result Qmss_ackInterrupt (uint8_t interruptNum, uint8_t value)
{
    return Qmss_ackInterruptByIntdSubSys (QMSS_SUBSYS_HND_GLOBAL, 0, interruptNum, value);
} /* Qmss_ackInterrupt */

/**
 *  @b Description
 *  @n
 *      The function is used to get the version information of the QMSS LLD.
 *
 *  @retval
 *      Version Information.
 */
uint32_t Qmss_getVersion (void)
{
    return QMSS_LLD_VERSION_ID;
} /* Qmss_getVersion */

/**
 *  @b Description
 *  @n
 *      The function is used to get the version string for the QMSS LLD.
 *
 *  @retval
 *      Version String.
 */
const char* Qmss_getVersionStr (void)
{
    return qmssLldVersionStr;
} /* Qmss_getVersionStr */

/**
@}
*/

/**
 *  @b Description
 *  @n
 *      The function return the source queue handle on which the descriptors configured for
 *      the specified memory region are stored.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  memRegion
 *      Memory region number
 *
 *  @param[in]  qGroup
 *      Queue group (for split mode)
 *
 *  @retval
 *      Success -   Queue Handle
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 */
Qmss_QueueHnd Qmss_getMemRegQueueHandleQGroupSubSys (Qmss_SubSysHnd subSysHnd, uint32_t memRegion, uint32_t qGroup)
{
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_GlobalObj_Unpadded *gObjPtr = &qmssGObj.obj[subSys];

    /* No RM check since internal LLD usage */
    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    /* No RM check since internal LLD usage */
    if (qGroup > gObjPtr->qmssGblCfgParams.maxQueMgrGroups)
    {
        return QMSS_INVALID_PARAM;
    }

    /* Invalidate Global Object */
    Qmss_osalBeginMemAccess (gObjPtr, sizeof (Qmss_GlobalObj_Unpadded));
    return (gObjPtr->descQueue[qGroup][memRegion]);
}

/**
 *  @b Description
 *  @n
 *      The function return the source queue handle on which the descriptors configured for
 *      the specified memory region are stored.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  memRegion
 *      Memory region number
 *
 *  @retval
 *      Success -   Queue Handle
 *  @retval
 *      Failure -   QMSS_NOT_INITIALIZED
 *      Failure -   QMSS_API_NO_SPLIT_MODE
 */
Qmss_QueueHnd Qmss_getMemRegQueueHandleSubSys (Qmss_SubSysHnd subSysHnd, uint32_t memRegion)
{
    uint32_t subSys = (uint32_t)subSysHnd;

    if (qmssLObj[subSys].mode == Qmss_Mode_SPLIT)
    {
        /* Can't assume qGroup is 0 */
        return QMSS_API_NO_SPLIT_MODE;
    }

    return Qmss_getMemRegQueueHandleQGroupSubSys (subSysHnd, memRegion, 0);
} /* Qmss_getMemRegQueueHandleSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_getMemRegQueueHandleSubSys
 *      which operates on the global subsystem.
 */
Qmss_QueueHnd Qmss_getMemRegQueueHandle (uint32_t memRegion)
{
    return Qmss_getMemRegQueueHandleSubSys (QMSS_SUBSYS_HND_GLOBAL, memRegion);
} /* Qmss_getMemRegQueueHandle */

/**
 *  @b Description
 *  @n
 *      The function returns size of the descriptors in the specified memory region.
 *
 *      This is for *internal* LLD usage.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *  @param[in]  memRegion
 *      Memory region number.
 *  @param[in]  qGroup
 *      Queue manager group number
 *
 *  @retval
 *      Descriptor size
 */
uint32_t Qmss_getMemRegDescSizeSubSys (Qmss_SubSysHnd subSysHnd, uint32_t memRegion, uint32_t qGroup)
{
    uint32_t  descVal;
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;

    /* No RM check since internal LLD usage */

    descVal = CSL_FEXT (lObjPtr->groupRegs[qGroup].qmDescReg->MEMORY_REGION_BASE_ADDRESS_GROUP[memRegion].MEMORY_REGION_DESCRIPTOR_SETUP_REG,
            QM_DESCRIPTOR_REGION_CONFIG_MEMORY_REGION_DESCRIPTOR_SETUP_REG_DESC_SIZE);
    return ((descVal + 1) * 16);
} /* Qmss_getMemRegDescSizeSubSys */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_getMemRegDescSizeSubSys
 *      which operates on the global subsystem.
 */
uint32_t Qmss_getMemRegDescSize (uint32_t memRegion, uint32_t qGroup)
{
    return Qmss_getMemRegDescSizeSubSys (QMSS_SUBSYS_HND_GLOBAL, memRegion, qGroup);
} /* Qmss_getMemRegDescSize */

/**
 *  @b Description
 *  @n
 *      Internal queue open function; is called to allocate a queue when critical section is provided
 *      by the caller.
 *
 *      A queue can be opened in two ways:
 *          1) If "queNum" is set to QMSS_PARAM_NOT_SPECIFIED, then a new
 *          available queue of type "queType" is allocated.
 *          2) If "queNum" is a valid queue number i.e., >= 0,
 *               then a queue is allocated if free
 *               else
 *               a handle to a previously opened queue is returned.
 *               In this case "queType" parameter is not used.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  qGroupIsSpecified
 *      0: Ignore qGroup in joint mode and allocate per balancing scheme
 *      1: Use gGroup in joint mode
 *
 *  @param[in]  qGroup
 *      Specifies queue group to open from (only used in SPLIT mode
 *
 *  @param[in]  queType
 *      Specifies the type of queue that should be opened.
 *
 *  @param[in]  queNum
 *      Specifies the queue number that should be opened.
 *
 *  @param[out]  isAllocated
 *      Indicates whether the requested queue is a new queue allocation(1).
 *      or was already allocated. If the queue was previously allocated this parameter returns
 *      the reference count.
 *
 *  *  @pre
 *      Appropriate Critical section protection should be provided by caller.
 *
 *  @retval
 *      Success -   Queue Handle. Used as an input parameter for operation on this queue.
 *  @retval
 *      Failure -   QMSS_RESOURCE_ALLOCATE_USE_DENIED
 *  @retval
 *      Failure -   <0
 */
static Qmss_QueueHnd Qmss_internalQueueOpen (Qmss_SubSysHnd subSysHnd, int qGroupIsSpecified, uint32_t qGroup, Qmss_QueueType queType, int queNum, uint8_t *isAllocated)
{
    uint32_t      index;
    uint32_t      startQueue, endQueue;
    uint32_t      firstQGroupIdx, qGroupIdx, lastQGroupIdx;
    uint32_t      qGroupSearch[QMSS_MAX_QMGR_GROUPS];
    uint32_t      subSys = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;
    uint32_t      nGroups = lObjPtr->maxQueMgrGroups;
    int           foundQ;
    int32_t       maxQueNumIdx;
    Qmss_QueueNumRange *maxQueueNumP;
    int32_t queTypeIdx;

    *isAllocated = 0;

    /* If queue is already specified just open it */
    if (queNum >= 0)
    {
        /* Use QMSS_QUEUE_GROUP to find que group because the qGroup may not always be specified */
        int32_t qGroupFromQueNum = QMSS_QUEUE_GROUP(queNum);
        if (queType == QMSS_PARAM_NOT_SPECIFIED)
        {
            if ((queTypeIdx = Qmss_internalFindMaxQueNumIdxByQue(subSysHnd, queNum)) < 0)
            {
                /* queNum does not map to valid queue type */
                return QMSS_INVALID_PARAM;
            }
            maxQueueNumP = qmssLObj[subSys].p.maxQueueNum[qGroupFromQueNum];
            queType = maxQueueNumP[queTypeIdx].queueType;
        }

        if (lObjPtr->qmRmServiceHandle)
        {
            int32_t maxQueNumIdx;
            maxQueNumIdx = Qmss_internalFindMaxQueNumIdxByType (
                              subSysHnd,
                              queType,
                              qGroupFromQueNum,
                              0);
            if (maxQueNumIdx < 0)
            {
                return QMSS_INVALID_QUEUE_TYPE;
            }

            if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_USE,
                                lObjPtr->maxQueueNum[qGroupFromQueNum][maxQueNumIdx].rmQueue, (int32_t *)&queNum, 1, 0, isAllocated))
            {
                return QMSS_RESOURCE_ALLOCATE_USE_DENIED;
            }
        }

        /* Invalidate qmssQueueFree Object */
        Qmss_osalBeginMemAccess ((void *) &qmssQueueFree[subSys][queNum], QMSS_MAX_CACHE_ALIGN);

        qmssQueueFree[subSys][queNum]++;

        /* With RM, the isAllocated is the "owner count" */
        if (! lObjPtr->qmRmServiceHandle)
        {
            *isAllocated = qmssQueueFree[subSys][queNum];
        }

        /* Internal book keeping always uses internal ref count */
        if (qmssQueueFree[subSys][queNum] == 1)
        {
            /* If new queue is opened then count it against its group */
            Qmss_internalUpdateGroupControl (subSys, QMSS_QUEUE_GROUP(queNum), -1);
        }

        /* Writeback qmssQueueFree Object */
        Qmss_osalEndMemAccess ((void *) &qmssQueueFree[subSys][queNum], QMSS_MAX_CACHE_ALIGN);
        return (Qmss_QueueHnd) QMSS_QUEUE_HNDL(subSys, queNum);

    }

    if (queType == QMSS_PARAM_NOT_SPECIFIED)
    {
        /* Queue type must be specified when queNum is not specified */
        return QMSS_INVALID_PARAM;
    }


    /* Get the group list to allocate from */
    if ((qmssLObj[subSys].mode == Qmss_Mode_SPLIT) || (qGroupIsSpecified))
    {
        /* Search only specified group */
        firstQGroupIdx  = 0;
        lastQGroupIdx   = 0;
        qGroupSearch[0] = qGroup;
    }
    else
    {
        /* Search all queue groups using requested schedule */
        firstQGroupIdx = 0;
        lastQGroupIdx  = nGroups - 1;
        /* Invalidate the qmssGroupControl structure */
        Qmss_osalBeginMemAccess ((void *) &qmssGroupControl[subSys], sizeof(qmssGroupControl[subSys]));
        if (qmssLObj[subSys].mode == Qmss_Mode_JOINT_ROUNDROBIN)
        {
            uint32_t alternate = qmssGroupControl[subSys].alternate;
            uint32_t i;
            /* Allocate first from QM specified by qmssGroupFree */
            for (qGroupIdx = alternate, i = 0; i < nGroups; i++)
            {
                qGroupSearch[qGroupIdx] = qGroupIdx;
                qGroupIdx++;
                if (qGroupIdx >= nGroups)
                {
                    qGroupIdx = 0;
                }
            }
            /* Move to next QM, regardless on allocation success */
            alternate++;
            if (alternate >= nGroups)
            {
                alternate = 0;
            }
            qmssGroupControl[subSys].alternate = alternate;
        }
        else
        {
            uint32_t i,j,temp;
            /* Find lowest used QM (with most free queues) and try to allocate there first */

            /* Start by initializing the group indicies in order */
            for (qGroupIdx = 0; qGroupIdx < nGroups; qGroupIdx ++)
            {
               qGroupSearch[qGroupIdx] = qGroupIdx;
            }

            /* Sort the indicies by number of free queues.  Inefficient sort
             * used since only 1-4 groups are expected
             */
            for (i = 0; i < nGroups; i++)
            {
                for (j = i + 1; j < nGroups; j++)
                {
                    if (qmssGroupControl[subSys].groupFreeQueues[qGroupSearch[i]] <
                        qmssGroupControl[subSys].groupFreeQueues[qGroupSearch[j]])
                    {
                        temp = qGroupSearch[i];
                        qGroupSearch[i] = qGroupSearch[j];
                        qGroupSearch[j] = temp;
                    }
                }
            }
        }
        /* Writeback qmssGroupControl structure */
        Qmss_osalEndMemAccess ((void *) &qmssGroupControl[subSys], sizeof(qmssGroupControl[subSys]));
    }
    /* Now qGroupSearch contains the list of queue groups to try to allocate
     * from in the priority order.
     */

    /* Try each permitted queue group */
    for (qGroupIdx = firstQGroupIdx; qGroupIdx <= lastQGroupIdx; qGroupIdx++)
    {
        qGroup = qGroupSearch[qGroupIdx];
        maxQueNumIdx = -1;

        /* Try each maxQueNum for the queue type (to cover more than one area of gen purpose queues) */
        while (1)
        {
            maxQueNumIdx = Qmss_internalFindMaxQueNumIdxByType (
                               subSysHnd,
                               queType,
                               qGroup,
                               maxQueNumIdx + 1);
            if (maxQueNumIdx < 0)
            {
                break;
            }

            foundQ = 0;

            if (lObjPtr->qmRmServiceHandle)
            {
                index = (uint32_t)RM_RESOURCE_BASE_UNSPECIFIED;
                if (Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_INIT,
                                   lObjPtr->maxQueueNum[qGroup][maxQueNumIdx].rmQueue, (int32_t *)&index, 1, 0, isAllocated))
                {
                    foundQ = 1;

                    /* Invalidate qmssQueueFree Object */
                    Qmss_osalBeginMemAccess ((void *) &qmssQueueFree[subSys][index], QMSS_MAX_CACHE_ALIGN);
                    if (qmssQueueFree[subSys][index] != 0)
                    {
                        return QMSS_RM_INTERNAL_ERROR;
                    }
                    qmssQueueFree[subSys][index]++;
                    /* Writeback qmssQueueFree Object */
                    Qmss_osalEndMemAccess ((void *) &qmssQueueFree[subSys][index], QMSS_MAX_CACHE_ALIGN);
                }
            }
            else
            {
                /* Get the queue range to allocate from */
                startQueue = lObjPtr->maxQueueNum[qGroup][maxQueNumIdx].startIndex;
                endQueue = lObjPtr->maxQueueNum[qGroup][maxQueNumIdx].startIndex + lObjPtr->maxQueueNum[qGroup][maxQueNumIdx].maxNum;

                /* Invalidate qmssQueueFree Object */
                Qmss_osalBeginMemAccess ((void *) &qmssQueueFree[subSys][startQueue], lObjPtr->maxQueueNum[qGroup][maxQueNumIdx].maxNum);

                for (index = startQueue; index < endQueue; index++)
                {
                    if (qmssQueueFree[subSys][index] == 0)
                    {
                        /* Queues ending in 0xfff are reserved by hw as return
                         * queues for CPPI.  Thus, keep them out of circulation
                         * so they don't unintentionally end up as return
                         * queues.  This removes at most 4 queues: 4095,
                         * 8191, 12287, 16383.  */
                        if ( (index & 0xfffu) != 0xfffu)
                        {
                            qmssQueueFree[subSys][index]++;

                            /* Writeback qmssQueueFree Object */
                            Qmss_osalEndMemAccess ((void *) &qmssQueueFree[subSys][index], QMSS_MAX_CACHE_ALIGN);

                            *isAllocated = qmssQueueFree[subSys][index];
                            foundQ = 1;
                            break;
                        }
                    }
                }
            }

            if (foundQ)
            {
                /* If new queue is opened then count it against its group */
                Qmss_internalUpdateGroupControl (subSys, QMSS_QUEUE_GROUP(index), -1);
                return (Qmss_QueueHnd) QMSS_QUEUE_HNDL(subSys, index);
            }
        }
    }

    return (Qmss_QueueHnd) -1;
} /* Qmss_internalQueueOpen */

