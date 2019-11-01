/**
 *   @file  qmss_mgmt.h
 *
 *   @brief   
 *      This is the Queue Manager queue management APIs.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010-2015, Texas Instruments, Inc.
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


#ifndef QMSS_MGMT_H_
#define QMSS_MGMT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_qm.h>
#include <ti/drv/qmss/include/qmss_pvt.h>
        
/* QMSS OSAL layer */
#include <ti/drv/qmss/qmss_osal.h>


/**********************************************************************
 ************************** Externs *********************************
 **********************************************************************/
        
/* QMSS Local object */
extern Qmss_LocalObj  qmssLObj[];

/** @addtogroup QMSS_LLD_FUNCTION
@{ 
*/

/**
 *  @internal
 *  @b Description
 *  @n  
 *      Internal function wrapper to do phy2virt for descriptors
 */
static inline void * Qmss_internalPhyToVirtDesc (Qmss_QueueHnd hnd, void *addr)
{
    uint32_t subSys = QMSS_QUEUE_SUBSYS(hnd);
    uint32_t QID    = QMSS_QUEUE_QID(hnd);
    void *result    = addr;

    if (qmssLObj[subSys].p.phy2VirtDesc)
    {
        result = qmssLObj[subSys].p.phy2VirtDesc (QID, addr);
    }

    return result;
}

/**
 *  @internal
 *  @b Description
 *  @n  
 *      Internal function wrapper to do virt2phy for descriptors
 */
static inline void * Qmss_internalVirtToPhyDesc (Qmss_QueueHnd hnd, void *addr)
{
    uint32_t subSys = QMSS_QUEUE_SUBSYS(hnd);
    uint32_t QID    = QMSS_QUEUE_QID(hnd);
    void *result    = addr;

    if (qmssLObj[subSys].p.virt2PhyDesc)
    {
        result = qmssLObj[subSys].p.virt2PhyDesc (QID, addr);
    }

    return result;
}

/**
 *  @b Description
 *  @n  
 *      This function pushes a descriptor onto a queue specified by the queue handle. 
 *      The "descSize" is used to specify the size of the descriptor being pushed.
 *      The optional parameter "packetSize" is used specify the size of packet during pop 
 *      operation. 
 *      The optional parameter "location" is used to override the default(tail) and push the packet 
 *      to the head of the queue.
 *
 *      **No validation is done on the input parameters**.
 *
 *      This function has some special considerations regarding critical section/mutex:
 *
 *      -# On c66x no mutex is ever needed, since it supports 64-bit stores.  This is 
 *         true whether or not a proxy is used.  Thus "#ifdef _TMS320C6600" is used
 *         to save cycles that would be spent calling an empty critical section.
 *      -# On other processors such as ARM, it may not support 64-bit stores.  Thus
 *         -# A proxy must be used to merge together the 32-bit stores
 *         -# a critical section/mutex is needed to protect from both threads
 *            and other cores who are trying to push to the same queue.
 *      If QoS scheduler with drop scheduler is in use, it has a push proxy that
 *      can be used if errata prevent the use of the HW proxy.  Please see
 *      @ref Qmss_qosSchedDropSchedPushProxySubSys.
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @param[in]  descAddr
 *      Memory address of the descriptor. Should be a global address.
 * 
 *  @param[in]  packetSize
 *      Size of packet pointed to by the descriptor.
 * 
 *  @param[in]  descSize
 *      Size of the descriptor. Minimum size is 16 bytes. Maximum size is 256 bytes
 * 
 *  @param[in]  location
 *      0 - Tail.
 *      1 - Head
 *
 *  @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      None
 */
static inline void Qmss_queuePush (Qmss_QueueHnd hnd, void *descAddr, uint32_t packetSize, uint32_t descSize, Qmss_Location location)
{
    uint32_t            regc = 0, regd = 0;
#if defined(_TMS320C6600) || defined(__ARM_ARCH_7A__)
#ifdef __ARM_ARCH_7A__
    volatile /* arm needs to go to real memory */
#endif
    uint64_t            dWord = 0;
    volatile uint64_t   *regCregDPtr;
#else
    void                *key;
#endif
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);
    descAddr = Qmss_internalVirtToPhyDesc (hnd, descAddr);
    CSL_FINS (regc, QM_QUEUE_MANAGEMENT_QUEUE_REG_C_HEAD_TAIL, location);
    
    CSL_FINS (regc, QM_QUEUE_MANAGEMENT_QUEUE_REG_C_PACKET_SIZE, packetSize);
   
    regd = ((uint32_t) descAddr | ((descSize >> 4) - 1));

    /* List all the processors that support atomic 64 bit write */
#if defined(_TMS320C6600) || defined(__ARM_ARCH_7A__)
    regCregDPtr = (volatile uint64_t *) (&qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtDataReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_C);
#ifdef _BIG_ENDIAN
    dWord = ((((uint64_t)regc)<<32)|regd);
#elif defined (_LITTLE_ENDIAN) || defined(__ARM_ARCH_7A__) 
/* preceding __ARM_ARCH_7A__ avoids requirement to define _LITTLE_ENDIAN on ARM */
    dWord = ((((uint64_t)regd)<<32)|regc);
#else
#error "Define _BIG_ENDIAN for Big Endian or _LITTLE_ENDIAN for Little Endian"
#endif

#if defined(_TMS320C6600)
    /* Atomic 64 bit store -- c6000 compiler figures this out from C code */
    *regCregDPtr = dWord;
#elif defined (__ARM_ARCH_7A__)
    /* Use ARM NEON to do 64 bit load/stores.  It is documented that moving
     * between neon and regular registers via vmov may or may not be faster than
     * going through memory.
     * Use of this function has some side effects: increase Linux context 
     * switch times due toNEON register saves.  It could also dynamically 
     * power on NEON, leading small step increase of power.  
     * Due to these side effects Qmss_queuePush should only be used 
     * when software requires head-push instead of tail-push or needs 
     * the specify the payload size (usually for QoS).  When neither of 
     * these features are required, use Qmss_queuePushDesc or 
     * Qmss_queuePushDescSize which doesn't activate NEON. */
    /* Memory keyword needed to avoid optimizer reordering memory access */
    /* put dWord into neon register d0 (which is caller preserve so
     * we can safely use it */
    asm volatile("vldr.i64 d0, [%0]" ::  "r"(&dWord) : "memory");
    /* put dWord into C+D with neon */
    asm volatile("vstr.i64 d0, [%0]" ::  "r"(regCregDPtr) : "memory");
#endif
#else
    /* Code for processors that don't support 64 bit store */
    /* This code MUST write to a proxy in order to work reliable with simutaneous accesses */

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalMtCsEnter ();

    /* MUST write C first */
    qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtProxyReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_C = regc;
    qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtProxyReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_D = regd;
    
    /* End Critical Section */   
    Qmss_osalMtCsExit (key);
#endif
    return;
}

/**
 *  @b Description
 *  @n  
 *      It pushes a descriptor onto a queue specified by the queue handle. Does not allow
 *      specifying optional parameters. The descriptor size is not written to the queue. This 
 *      function should be used to push descriptors that will not be prefetched by the CPDMA.
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @param[in]  descAddr
 *      Memory address of the descriptor. Should be a global address.
 * 
 *  @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      None
 */
static inline void Qmss_queuePushDesc (Qmss_QueueHnd hnd, void *descAddr)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);
    descAddr = Qmss_internalVirtToPhyDesc (hnd, descAddr);

    qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtDataReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_D = (uint32_t) descAddr;
    return;
}

/**
 *  @b Description
 *  @n  
 *      It pushes a descriptor onto a queue specified by the queue handle. Does not allow
 *      specifying optional parameters.
 *
 *      The "descSize" is used to specify the size of the descriptor being pushed. This 
 *      function should be used to push descriptors that will be prefetched by the CPDMA.
 *
 *      The raw version of the API doesn't perform any checks or address translation.
 *      The regular version, @ref Qmss_queuePushDescSize, performs address translation
 *      on the descriptor before the push.
 *
 *      The primary reason for use of this API is for highly optimized applications that
 *      manage their own address translation (virtual memory).  Normal SW should use
 *      @ref Qmss_queuePushDescSize instead.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @param[in]  descAddr
 *      Memory address of the descriptor. Should be a global address.
 * 
 *  @param[in]  descSize
 *      Size of the descriptor. Minimum size is 16 bytes. Maximum size is 256 bytes
 * 
 *  @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      None
 */
static inline void Qmss_queuePushDescSizeRaw (Qmss_QueueHnd hnd, void *descAddr, uint32_t descSize)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtDataReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_D = ((uint32_t) descAddr | ((descSize >> 4) - 1)); 
    return;
}

/**
 *  @b Description
 *  @n  
 *      It pushes a descriptor onto a queue specified by the queue handle. Does not allow
 *      specifying optional parameters.
 *
 *      The "descSize" is used to specify the size of the descriptor being pushed. This 
 *      function should be used to push descriptors that will be prefetched by the CPDMA.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @param[in]  descAddr
 *      Memory address of the descriptor. Should be a global address.
 * 
 *  @param[in]  descSize
 *      Size of the descriptor. Minimum size is 16 bytes. Maximum size is 256 bytes
 * 
 *  @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      None
 */
static inline void Qmss_queuePushDescSize (Qmss_QueueHnd hnd, void *descAddr, uint32_t descSize)
{

    descAddr = Qmss_internalVirtToPhyDesc (hnd, descAddr);

    Qmss_queuePushDescSizeRaw (hnd, descAddr, descSize);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function pop's a descriptor off the queue specified by the queue handle.
 *
 *      The lower 4 bits of the descriptor address contain the size of the descriptor 
 *      that was specified during the queue push operation.
 *      Caller should mask the lower order 4 bits before using the descriptor.
 *
 *      Address translation is not performed on the descriptor before the return.
 *      In order to have address translation done, use @ref Qmss_queuePop
 *      
 *      The primary reason for use of this API is for highly optimized applications that
 *      manage their own address translation (virtual memory).  Normal SW should use
 *      @ref Qmss_queuePop instead.
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 * @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      Success - Descriptor address and size
 *  @retval
 *      0 -   If queue is empty
 */
static inline void* Qmss_queuePopRaw (Qmss_QueueHnd hnd)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    return (void *) (qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_D);
}

/**
 *  @b Description
 *  @n  
 *      This function pop's a descriptor off the queue specified by the queue handle.
 *
 *      The lower 4 bits of the descriptor address contain the size of the descriptor 
 *      that was specified during the queue push operation.
 *      Caller should mask the lower order 4 bits before using the descriptor.
 *
 *      **No validation is done on the input parameters**
 *      
 *  @param[in]  hnd
 *      Queue handle.
 *
 * @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      Success - Descriptor address and size
 *  @retval
 *      0 -   If queue is empty
 */
static inline void* Qmss_queuePop (Qmss_QueueHnd hnd)
{
    return Qmss_internalPhyToVirtDesc (hnd, Qmss_queuePopRaw(hnd));
}

/**
 *  @b Description
 *  @n  
 *      This function pop's a descriptor off the queue specified by the queue handle.
 *      It also returns the packet size of the popped decriptor. The packet size is 
 *      available only if it was specified during the push operation. 
 *
 *      If the processor supports atomic 64 bit reads, then this can pop via
 *      proxy or via non proxy without critical section or other issues.
 *
 *      If processor doesn't support 64 bit reads, then a critical section is required
 *      if multiple readers can pop the same queue.
 *
 *      Special protection is also needed to make sure reg C isn't read on an empty queue,
 *      and reg D read once a descriptor comes in.  This can be done externally by checking
 *      @ref Qmss_getQueueEntryCount before popping.
 *
 *      The packet size field is part of the descriptor and should be used to ensure correctness.
 *
 *      The lower 4 bits of the descriptor address contain the size of the descriptor 
 *      that was specified during the queue push operation.
 *      Caller should mask the lower order 4 bits before using the descriptor.
 *
 *      **No validation is done on the input parameters**
 *      
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @param[out]  descAddr
 *      Descriptor address and size.
 *
 *  @param[out]  packetSize
 *      Packet size in bytes.
 *
 * @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      None
 */
static inline void Qmss_queuePopDescSize (Qmss_QueueHnd hnd, void **descAddr, uint32_t *packetSize)
{
    uint32_t            regc, regd;
#if defined(_TMS320C6600)
    uint64_t            dWord = 0;
    volatile uint64_t   *regCregDPtr;
#else
    void                *key;
#endif
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

#if defined(_TMS320C6600)
    /* List all the processors that support atomic 64 bit read */
    regCregDPtr = (volatile uint64_t *) (& qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtDataReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_C);
    dWord = *regCregDPtr;

#ifdef _BIG_ENDIAN
    regc = (uint32_t)(dWord >> 32);
    regd = (uint32_t)(dWord);
#else
#ifdef _LITTLE_ENDIAN
    regd = (uint32_t)(dWord >> 32);
    regc = (uint32_t)(dWord);
#else
#error "Define _BIG_ENDIAN for Big Endian or _LITTLE_ENDIAN for Little Endian"
#endif
#endif
#else

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalMtCsEnter ();

    /* MUST write D first */
    regc = qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtDataReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_C;
    regd = qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtDataReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_D;
    
    /* End Critical Section */   
    Qmss_osalMtCsExit (key);
#endif
    
    *packetSize = CSL_FEXT (regc, 
                    QM_QUEUE_STATUS_CONFIG_QUEUE_STATUS_CONFIG_REG_C_PACKET_SIZE); 
    *descAddr = (void *)Qmss_internalPhyToVirtDesc(hnd, (void *)regd);
    return;
}

/** 
 *  @b Description
 *  @n  
 *      Given the queue handle the function returns the address of queue management register D. 
 *      register D is used to write the descriptor address during a queue push operation. 
 *
 *      The D register is available through "data" space and is retrievable using
 *      this API.  It is also available through config address space, 
 *      which is available through @ref Qmss_getQueuePushHandleCfg.  Please check device errata 
 *      document for usage advisories for each address.
 *
 *  @param[in]  hnd      
 *      Queue handle
 *
 *  @retval
 *      4 byte memory address.
 */
static inline Qmss_QueuePushHnd Qmss_getQueuePushHandle (Qmss_QueueHnd hnd)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    return ((Qmss_QueuePushHnd) &(qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtDataReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_D));
}

/** 
 *  @b Description
 *  @n  
 *      Given the queue handle the function returns the address of queue management register D. 
 *      register D is used to write the descriptor address during a queue push operation. 
 *
 *      The D register is available through "data" space and is retrievable using
 *      @ref Qmss_getQueuePushHandle.  It is also available through config address space, 
 *      which is available with this API.  Please check device errata document for
 *      usage advisories for each address.
 *
 *  @param[in]  hnd      
 *      Queue handle
 *
 *  @retval
 *      4 byte memory address.
 */
static inline Qmss_QueuePushHnd Qmss_getQueuePushHandleCfg (Qmss_QueueHnd hnd)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    return ((Qmss_QueuePushHnd) &(qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_D));
}

/**
 *  @b Description
 *  @n  
 *      This function diverts the entire content of source queue to the destination queue. 
 *      "location" indicates whether the contents should be merged on to the head or tail 
 *      of the destination queue.
 * 
 *  @param[in]  srcQnum
 *      Source queue handle.
 *
 *  @param[in]  dstQnum
 *      Destination queue handle.
 * 
 *  @param[in]  location
 *      Head/Tail.
 * 
 *  @pre  
 *      Qmss_queueOpen function for source queue and destination queue should be called before 
 *      calling this function.
 *
 *  @post  
 *      Contents of source queue is moved to the destination queue. Source queue is empty.
 *
 *  @retval
 *      Failure - QMSS_INVALID_PARAM - queues are in different groups
 *  @retval
 *      Success - QMSS_SOK
 *      
 */
static inline Qmss_Result Qmss_queueDivert (Qmss_QueueHnd srcQnum, Qmss_QueueHnd dstQnum, Qmss_Location location)
{
    uint32_t  temp = 0;
    uint32_t             srcQGroup  = QMSS_QUEUE_GROUP(srcQnum);
    uint32_t             srcQNumber = QMSS_QUEUE_NUMBER(srcQnum);
    uint32_t             srcQSubSys = QMSS_QUEUE_SUBSYS(srcQnum);
    uint32_t             dstQGroup  = QMSS_QUEUE_GROUP(dstQnum);
    uint32_t             dstQNumber = QMSS_QUEUE_NUMBER(dstQnum);
    uint32_t             dstQSubSys = QMSS_QUEUE_SUBSYS(dstQnum);
    
    /* Both queues must be in same group and subsys */
    if ((srcQGroup != dstQGroup) || (srcQSubSys != dstQSubSys))
    {
        return QMSS_INVALID_PARAM;
    }
    CSL_FINS (temp, QM_CONFIG_QUEUE_DIVERSION_REG_SOURCE_QNUM, srcQNumber);
    CSL_FINS (temp, QM_CONFIG_QUEUE_DIVERSION_REG_DEST_QNUM, dstQNumber);
    CSL_FINS (temp, QM_CONFIG_QUEUE_DIVERSION_REG_HEAD_TAIL, location);

    qmssLObj[srcQSubSys].p.groupRegs[srcQGroup].qmConfigReg->QUEUE_DIVERSION_REG = temp;
    return QMSS_SOK; 
}

/**
 *  @b Description
 *  @n  
 *      This function deletes all the contents of the queue.
 * 
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @post  
 *      Queue is empty
 *
 *  @retval
 *      None
 */
static inline void Qmss_queueEmpty (Qmss_QueueHnd hnd)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueMgmtReg->QUEUE_MGMT_GROUP[qNumber].QUEUE_REG_D = (uint32_t) 0x0;
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function returns the number of packets that are currently queued on the queue.
 * 
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      Queue entry count 
 */
static inline uint32_t Qmss_getQueueEntryCount (Qmss_QueueHnd hnd)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    return (uint32_t) CSL_FEXT (qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueStatReg->QUEUE_STATUS_CONFIG_GROUP[qNumber].QUEUE_STATUS_CONFIG_REG_A, 
                    QM_QUEUE_STATUS_CONFIG_QUEUE_STATUS_CONFIG_REG_A_QUEUE_ENTRY_COUNT); 
}

/**
 *  @b Description
 *  @n  
 *      This function returns the total number of bytes that are contained in all of the 
 *      packets that are currently queued on the queue.
 * 
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      Queue byte count 
 */
static inline uint32_t Qmss_getQueueByteCount (Qmss_QueueHnd hnd)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    return (uint32_t) CSL_FEXT (qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueStatReg->QUEUE_STATUS_CONFIG_GROUP[qNumber].QUEUE_STATUS_CONFIG_REG_B, 
                    QM_QUEUE_STATUS_CONFIG_QUEUE_STATUS_CONFIG_REG_B_QUEUE_BYTE_COUNT); 
}

/**
 *  @b Description
 *  @n  
 *      This function returns the packet size of the packet queued at the head of the queue.
 * 
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @pre  
 *      Qmss_queueOpen function should be called before calling this function.
 *
 *  @retval
 *      Queue packet size
 */
static inline uint32_t Qmss_getQueuePacketSize (Qmss_QueueHnd hnd)
{
    uint32_t             qGroup  = QMSS_QUEUE_GROUP(hnd);
    uint32_t             qNumber = QMSS_QUEUE_NUMBER(hnd);
    uint32_t             qSubSys = QMSS_QUEUE_SUBSYS(hnd);

    return (uint32_t) CSL_FEXT (qmssLObj[qSubSys].p.groupRegs[qGroup].qmQueStatReg->QUEUE_STATUS_CONFIG_GROUP[qNumber].QUEUE_STATUS_CONFIG_REG_C, 
                    QM_QUEUE_STATUS_CONFIG_QUEUE_STATUS_CONFIG_REG_C_PACKET_SIZE); 
}

/**
 *  @b Description
 *  @n  
 *      This function returns Queue ID that can be used with other hardware that
 *      request a 14 or 16 bit QID.
 * 
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @retval
 *      QID
 */
static inline uint32_t Qmss_getQIDFromHandle (Qmss_QueueHnd hnd)
{
    /* This needs to change if the handle is abstracted from the queue ID */
    return QMSS_QUEUE_QID(hnd);
} /* Qmss_getQIDFromHandle */

/**
 *  @b Description
 *  @n  
 *      This function returns Queue Handle associated with a QID.
 *      A QID is a 14 or 16 bit number used by other hardware.
 *
 *      No checking is performed on whether this context is allowed to
 *      use the generated handle resource.
 * 
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  QID
 *      14 or 16 bit QID.
 *
 *  @retval
 *      Queue handle
 */
static inline Qmss_QueueHnd Qmss_getHandleFromQIDSubSys (Qmss_SubSysHnd subSysHnd, uint32_t QID)
{
    uint32_t subSys = (uint32_t)subSysHnd;
    /* This needs to change if the handle is abstracted from the queue ID */
    return QMSS_QUEUE_HNDL(subSys,QID);
} /* Qmss_getHandleFromQIDSubSys */

/**
 *  @b Description
 *  @n  
 *      This function returns Queue Handle associated with a QID in
 *      the global subsystem.  Do not use this API to generate handles
 *      for queues in other subsystems.
 *
 *      A QID is a 14 or 16 bit number used by other hardware.
 *
 *      No checking is performed on whether this context is allowed to
 *      use the generated handle resource.
 * 
 *  @param[in]  QID
 *      14 or 16 bit QID.
 *
 *  @retval
 *      Queue handle
 */
static inline Qmss_QueueHnd Qmss_getHandleFromQID (uint32_t QID)
{
    /* This needs to change if the handle is abstracted from the queue ID */
    return Qmss_getHandleFromQIDSubSys (QMSS_SUBSYS_HND_GLOBAL, QID);
} /* Qmss_getHandleFromQID */

/** 
@} 
*/


#ifdef __cplusplus
}
#endif

#endif /* QMSS_MGMT_H_ */

