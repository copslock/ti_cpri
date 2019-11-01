/**
 *   @file  qmss_qm.h
 *
 *   @brief   
 *      This is the Queue Manager module include file.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2015, Texas Instruments, Inc.
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


#ifndef QMSS_QM_H_
#define QMSS_QM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* CSL RL includes */
#include <ti/csl/cslr_qm_config.h>
#include <ti/csl/cslr_qm_descriptor_region_config.h>
#include <ti/csl/cslr_qm_queue_management.h>
#include <ti/csl/cslr_qm_queue_status_config.h>
#include <ti/csl/cslr_pdsp.h>
#include <ti/csl/cslr_qm_intd.h>
#include <ti/csl/cslr_mcdma.h>
#include <ti/csl/cslr_cp_timer16.h>

/**
@defgroup QMSS_LLD_SYMBOL  QMSS Low Level Driver Symbols Defined
@ingroup QMSS_LLD_API
*/
/**
@defgroup QMSS_LLD_ENUM  QMSS Low Level Driver Enums
@ingroup QMSS_LLD_API
*/
/**
@defgroup QMSS_LLD_DATASTRUCT  QMSS Low Level Driver Data Structures
@ingroup QMSS_LLD_API
*/
/**
@defgroup QMSS_LLD_FUNCTION  QMSS Low Level Driver Functions
@ingroup QMSS_LLD_API
*/
/**
@defgroup QMSS_LLD_OSAL  QMSS Low Level Driver OSAL Functions
@ingroup QMSS_LLD_API
*/

/**
@addtogroup QMSS_LLD_SYMBOL
@{
*/

/** Used as input parameter when queue number is 
 * not known and not specified */
#define QMSS_PARAM_NOT_SPECIFIED                    -1

/** Used to indicate that QMSS HW Initialization is complete */
#define QMSS_HW_INIT_COMPLETE                       0xABCD

/** QMSS Low level Driver return and Error Codes */
/** QMSS successful return code */
#define QMSS_SOK                                    0
/** QMSS Error Base */       
#define QMSS_LLD_EBASE                              (-128)
/** QMSS LLD invalid parameter */
#define QMSS_INVALID_PARAM                          QMSS_LLD_EBASE-1
/** QMSS LLD not initialized */
#define QMSS_NOT_INITIALIZED                        QMSS_LLD_EBASE-2
/** QMSS LLD queue open error */
#define QMSS_QUEUE_OPEN_ERROR                       QMSS_LLD_EBASE-3
/** QMSS memory region not initialized */
#define QMSS_MEMREGION_NOT_INITIALIZED              QMSS_LLD_EBASE-4
/** QMSS memory region already initialized */
#define QMSS_MEMREGION_ALREADY_INITIALIZED          QMSS_LLD_EBASE-5
/** QMSS memory region invalid parameter */
#define QMSS_MEMREGION_INVALID_PARAM                QMSS_LLD_EBASE-6
/** QMSS maximum number of allowed descriptor are already configured */
#define QMSS_MAX_DESCRIPTORS_CONFIGURED             QMSS_LLD_EBASE-7
/** QMSS Specified memory region index is invalid or no memory regions are available */
#define QMSS_MEMREGION_INVALID_INDEX                QMSS_LLD_EBASE-8
/** QMSS memory region overlap */
#define QMSS_MEMREGION_OVERLAP                      QMSS_LLD_EBASE-9
/** QMSS memory region not in acscending order */
#define QMSS_MEMREGION_ORDERING                     QMSS_LLD_EBASE-10
/** QMSS PDSP firmware download failure */
#define QMSS_FIRMWARE_DOWNLOAD_FAILED               QMSS_LLD_EBASE-11
/** QMSS resource allocator to initialize denied */
#define QMSS_RESOURCE_ALLOCATE_INIT_DENIED          QMSS_LLD_EBASE-12
/** QMSS resource allocation to use denied */
#define QMSS_RESOURCE_ALLOCATE_USE_DENIED           QMSS_LLD_EBASE-13
#define QMSS_RESOURCE_USE_DENIED                    QMSS_RESOURCE_ALLOCATE_USE_DENIED /* Keep for backwards compatability */
/** QMSS memory region initialization permission denied */
#define QMSS_RESOURCE_MEM_REGION_INIT_DENIED        QMSS_LLD_EBASE-14
/** QMSS memory region usage permission denied */
#define QMSS_RESOURCE_MEM_REGION_USE_DENIED         QMSS_LLD_EBASE-15
/** QMSS general linking RAM initialization permission denied */
#define QMSS_RESOURCE_LINKING_RAM_INIT_DENIED       QMSS_LLD_EBASE-16
/** QMSS firmware revision difference */
#define QMSS_FIRMWARE_REVISION_DIFFERENCE           QMSS_LLD_EBASE-17
/** QMSS firmware invalid garbage return queue */
#define QMSS_INVALID_SRIO_GARBAGE_QUEUE             QMSS_LLD_EBASE-18
/** QMSS Bad Queue Number Configuration Table */
#define QMSS_INVALID_QUEUE_NUMBER_TAB               QMSS_LLD_EBASE-19
/** QMSS resource free was denied by RM */
#define QMSS_RESOURCE_FREE_DENIED                   QMSS_LLD_EBASE-20
/** QMSS control of either QM was denied by RM */
#define QMSS_QM_CONTROL_DENIED                      QMSS_LLD_EBASE-21
/** QMSS feature not supported on this subsystem */
#define QMSS_SUBSYS_UNSUPPORTED                     QMSS_LLD_EBASE-22
/** Queue type not found */
#define QMSS_INVALID_QUEUE_TYPE                     QMSS_LLD_EBASE-23
/** QMSS exit failed (queues still open) */
#define QMSS_EXIT_QUEUES_OPEN                       QMSS_LLD_EBASE-24
/** QMSS exit failed (regions still inserted) */
#define QMSS_EXIT_REGIONS_PRESENT                   QMSS_LLD_EBASE-25
/** Internal bookkeeping out of sync with RM */
#define QMSS_RM_INTERNAL_ERROR                      QMSS_LLD_EBASE-26
/** Qmss_queueOpenUse called on queue not already open */
#define QMSS_QUEUE_NOT_ALREADY_OPEN                 QMSS_LLD_EBASE-27
/** QMSS feature requires RM */
#define QMSS_FEATURE_REQUIRES_RM                    QMSS_LLD_EBASE-28
/** API doesn't support split mode (because qGroup is needed) */
#define QMSS_API_NO_SPLIT_MODE                      QMSS_LLD_EBASE-29
/** virt2phys or phys2virt failed */
#define QMSS_VIRTPHYS_FAIL                          QMSS_LLD_EBASE-30
/** QMSS memory region not already initialized */
#define QMSS_MEMREGION_NOT_ALREADY_INITIALIZED      QMSS_LLD_EBASE-31
/** Failed nameserver insert/query/delete */
#define QMSS_NS_FAIL                                QMSS_LLD_EBASE-32
/** Failed to format nameserver namer */
#define QMSS_NS_NAME_FAIL                           QMSS_LLD_EBASE-33

/** Number of subsystems supported by LLD */
#define QMSS_MAX_SUBSYS                             2

/** QMSS maximum number of memory regions in subsystem 0 (Qmss_SubSys_GLOBAL) */
#define QMSS_MAX_MEM_REGIONS_SS_0                   64
/** QMSS maximum number of PDSPS in subsystem 0 (Qmss_SubSys_GLOBAL) */
#define QMSS_MAX_PDSP_SS_0                          8
/** QMSS maximum number of INTDs in subsystem 0 (Qmss_SubSys_GLOBAL) */
#define QMSS_MAX_INTD_SS_0                          2
/** QMSS maximum number of queue groups (QMGRs that run independently) in subsystem 0 (Qmss_SubSys_GLOBAL) */
#define QMSS_MAX_QMGR_GROUPS_SS_0                   2
/** QMSS maximum number of queue types (per group) in subsystem 0 (Qmss_SubSys_GLOBAL) */
#define QMSS_MAX_QUEUE_TYPES_SS_0                   35
/** QMSS maximum number of queues in subsystem 0 (Qmss_SubSys_GLOBAL) */
#define QMSS_MAX_QUEUES_SS_0                        16384

/** QMSS maximum number of memory regions in subsystem 1 (Qmss_SubSys_NETSS) */
#define QMSS_MAX_MEM_REGIONS_SS_1                   16
/** QMSS maximum number of PDSPS in subsystem 1 (Qmss_SubSys_NETSS) */
#define QMSS_MAX_PDSP_SS_1                          0
/** QMSS maximum number of INTDs in subsystem 1 (Qmss_SubSys_NETSS) */
#define QMSS_MAX_INTD_SS_1                          0
/** QMSS maximum number of queue groups (QMGRs that run independently) in subsystem 1 (Qmss_SubSys_NETSS) */
#define QMSS_MAX_QMGR_GROUPS_SS_1                   2
/** QMSS maximum number of queue types (per group) in subsystem 1 (Qmss_SubSys_NETSS) */
#define QMSS_MAX_QUEUE_TYPES_SS_1                   2
/** QMSS maximum number of queues in subsystem 1 (Qmss_SubSys_NETSS) */
#define QMSS_MAX_QUEUES_SS_1                        128

/* For the following definition, internal memories are sized the same for all subsystems.
 * Only the large memories, such as number of queues and number of regions, are sized individually
 */
/** QMSS maximum number of memory regions in all subsystems */
#define QMSS_MAX_MEM_REGIONS                        64
#if (QMSS_MAX_MEM_REGIONS < QMSS_MAX_MEM_REGIONS_SS_0) || \
    (QMSS_MAX_MEM_REGIONS < QMSS_MAX_MEM_REGIONS_SS_1)
#error check QMSS_MAX_MEM_REGIONS
#endif

/** QMSS maximum number of PDSPS in all subsystems */
#define QMSS_MAX_PDSP                               8
#if (QMSS_MAX_PDSP < QMSS_MAX_PDSP_SS_0) || \
    (QMSS_MAX_PDSP < QMSS_MAX_PDSP_SS_1)
#error check QMSS_MAX_PDSP
#endif

/** QMSS maximum number of INTDs in all subsystems */
#define QMSS_MAX_INTD                               2
#if (QMSS_MAX_INTD < QMSS_MAX_INTD_SS_0) || \
    (QMSS_MAX_INTD < QMSS_MAX_INTD_SS_1)
#error check QMSS_MAX_INTD
#endif

/** QMSS maximum number of queue groups (QMGRs that run independently) in all subsystems */
#define QMSS_MAX_QMGR_GROUPS                        2
#if (QMSS_MAX_QMGR_GROUPS < QMSS_MAX_QMGR_GROUPS_SS_0) || \
    (QMSS_MAX_QMGR_GROUPS < QMSS_MAX_QMGR_GROUPS_SS_1)
#error check QMSS_MAX_QMGR_GROUPS
#endif

/** QMSS RM resource name maximum characters */
#define QMSS_RM_RESOURCE_NAME_MAX_CHARS             32

/** Macro to get the descriptor pointer if the popped descriptor contains the descriptor size. 
 * If Qmss_queuePushDescSize() API is used to push a descriptor onto a queue, the descriptor when 
 * popped will have the descriptor size information in the lower 4 bits. This macro is provided to 
 * clear out the size information */
#define QMSS_DESC_PTR(desc)                         ((uint32_t)desc & 0xFFFFFFF0)

/** Macro to get the descriptor size if the popped descriptor contains the descriptor size. 
 * If Qmss_queuePushDescSize() API is used to push a descriptor onto a queue, the descriptor when 
 * popped will have the descriptor size information in the lower 4 bits. This macro is provided to 
 * obtain the size information. Minimum size is 16 bytes. Maximum size is 256 bytes */
#define QMSS_DESC_SIZE(desc)                        ((((uint32_t)desc & 0x0000000F) + 1) << 4) 

/** Macro to extract the queue manager group from the queue handle 
 * These are not to be confused with @ref Qmss_getQueueNumber (to be used with CPPI) or
 * @ref Qmss_getQIDFromHandle (to be used with firmware) and are intended for internal LLD and
 * unit test use only.  */
#define QMSS_QUEUE_SUBSYS(hndl)                     ((uint32_t)((hndl) >> 14) & 1)
#define QMSS_QUEUE_GROUP(hndl)                      ((uint32_t)((hndl) >> 13) & 1)
#define QMSS_QUEUE_NUMBER(hndl)                     ((uint32_t)(hndl) & 0x1FFF)
/* User code shouldn't use following two macros.  Use Qmss_getHandleFromQID and Qmss_getQIDFromHandle. */
#define QMSS_QUEUE_QID(hndl)                        ((uint32_t)(hndl) & 0x3FFF)
#define QMSS_QUEUE_HNDL(subSys,qid)                 (((uint32_t)(qid) & 0x3FFFF) | (((subSys) & 1) << 14))

/** Used to auto allocate @ref Qmss_MemRegInfo.startIndex from any valid linking RAM */
#define QMSS_START_INDEX_NOT_SPECIFIED (QMSS_PARAM_NOT_SPECIFIED)
/** Used to auto allocate @ref Qmss_MemRegInfo.startIndex only from internal linking RAM or 
 * first linking RAM region */
#define QMSS_START_INDEX_INTERNAL      (-2)
/** Used to auto allocate @ref Qmss_MemRegInfo.startIndex only from external linking RAM or 
 * second linking RAM region */
#define QMSS_START_INDEX_EXTERNAL      (-3)

/**
@}
*/

/**
@addtogroup QMSS_LLD_ENUM
@{
*/

/** 
 * @brief location where the packet is queued
 */
typedef enum
{
    /** Queue packet to the tail of the queue. Default behavior. */
    Qmss_Location_TAIL = 0,
    /** Queue packet to the head of the queue. */
    Qmss_Location_HEAD 
}Qmss_Location;

/** 
 * @brief Descriptor resource management
 */
typedef enum
{
    /** LLD doesnot manage the descriptors. The caller should manage them. */
    Qmss_ManageDesc_UNMANAGED_DESCRIPTOR = 0,
    /** LLD manages the descriptors. The descriptors are reclaimed using 
     * the QMSS_initDescriptor() or CPPI_initDescriptor() APIs
     * */
    Qmss_ManageDesc_MANAGE_DESCRIPTOR 
}Qmss_ManageDesc;

/** 
 * @brief Queue Manager's memory regions
 */
typedef enum
{
    /** Memory region not specified. LLD allocates the next available memory region */
    Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED = -1,
    /** Configure memory region0. */
    Qmss_MemRegion_MEMORY_REGION0 = 0,
    /** Configure memory region 1. */
    Qmss_MemRegion_MEMORY_REGION1,
    /** Configure memory region 2. */
    Qmss_MemRegion_MEMORY_REGION2,
    /** Configure memory region 3. */
    Qmss_MemRegion_MEMORY_REGION3,
    /** Configure memory region 4. */
    Qmss_MemRegion_MEMORY_REGION4,
    /** Configure memory region 5. */
    Qmss_MemRegion_MEMORY_REGION5,
    /** Configure memory region 6. */
    Qmss_MemRegion_MEMORY_REGION6,
    /** Configure memory region 7. */
    Qmss_MemRegion_MEMORY_REGION7,
    /** Configure memory region 8. */
    Qmss_MemRegion_MEMORY_REGION8,
    /** Configure memory region 9. */
    Qmss_MemRegion_MEMORY_REGION9,
    /** Configure memory region 10. */
    Qmss_MemRegion_MEMORY_REGION10,
    /** Configure memory region 11. */
    Qmss_MemRegion_MEMORY_REGION11,
    /** Configure memory region 12. */
    Qmss_MemRegion_MEMORY_REGION12,
    /** Configure memory region 13. */
    Qmss_MemRegion_MEMORY_REGION13,
    /** Configure memory region 14. */
    Qmss_MemRegion_MEMORY_REGION14,
    /** Configure memory region 15. */
    Qmss_MemRegion_MEMORY_REGION15,
    /** Configure memory region 16. */
    Qmss_MemRegion_MEMORY_REGION16,
    /** Configure memory region 17. */
    Qmss_MemRegion_MEMORY_REGION17,
    /** Configure memory region 18. */
    Qmss_MemRegion_MEMORY_REGION18,
    /** Configure memory region 19. */
    Qmss_MemRegion_MEMORY_REGION19,
    /** Configure memory region 20. */
    Qmss_MemRegion_MEMORY_REGION20,
    /** Configure memory region 21. */
    Qmss_MemRegion_MEMORY_REGION21,
    /** Configure memory region 22. */
    Qmss_MemRegion_MEMORY_REGION22,
    /** Configure memory region 23. */
    Qmss_MemRegion_MEMORY_REGION23,
    /** Configure memory region 24. */
    Qmss_MemRegion_MEMORY_REGION24,
    /** Configure memory region 25. */
    Qmss_MemRegion_MEMORY_REGION25,
    /** Configure memory region 26. */
    Qmss_MemRegion_MEMORY_REGION26,
    /** Configure memory region 27. */
    Qmss_MemRegion_MEMORY_REGION27,
    /** Configure memory region 28. */
    Qmss_MemRegion_MEMORY_REGION28,
    /** Configure memory region 29. */
    Qmss_MemRegion_MEMORY_REGION29,
    /** Configure memory region 30. */
    Qmss_MemRegion_MEMORY_REGION30,
    /** Configure memory region 31. */
    Qmss_MemRegion_MEMORY_REGION31,
    /** Configure memory region 32. */
    Qmss_MemRegion_MEMORY_REGION32,
    /** Configure memory region 33. */
    Qmss_MemRegion_MEMORY_REGION33,
    /** Configure memory region 34. */
    Qmss_MemRegion_MEMORY_REGION34,
    /** Configure memory region 35. */
    Qmss_MemRegion_MEMORY_REGION35,
    /** Configure memory region 36. */
    Qmss_MemRegion_MEMORY_REGION36,
    /** Configure memory region 37. */
    Qmss_MemRegion_MEMORY_REGION37,
    /** Configure memory region 38. */
    Qmss_MemRegion_MEMORY_REGION38,
    /** Configure memory region 39. */
    Qmss_MemRegion_MEMORY_REGION39,
    /** Configure memory region 40. */
    Qmss_MemRegion_MEMORY_REGION40,
    /** Configure memory region 41. */
    Qmss_MemRegion_MEMORY_REGION41,
    /** Configure memory region 42. */
    Qmss_MemRegion_MEMORY_REGION42,
    /** Configure memory region 43. */
    Qmss_MemRegion_MEMORY_REGION43,
    /** Configure memory region 44. */
    Qmss_MemRegion_MEMORY_REGION44,
    /** Configure memory region 45. */
    Qmss_MemRegion_MEMORY_REGION45,
    /** Configure memory region 46. */
    Qmss_MemRegion_MEMORY_REGION46,
    /** Configure memory region 47. */
    Qmss_MemRegion_MEMORY_REGION47,
    /** Configure memory region 48. */
    Qmss_MemRegion_MEMORY_REGION48,
    /** Configure memory region 49. */
    Qmss_MemRegion_MEMORY_REGION49,
    /** Configure memory region 50. */
    Qmss_MemRegion_MEMORY_REGION50,
    /** Configure memory region 51. */
    Qmss_MemRegion_MEMORY_REGION51,
    /** Configure memory region 52. */
    Qmss_MemRegion_MEMORY_REGION52,
    /** Configure memory region 53. */
    Qmss_MemRegion_MEMORY_REGION53,
    /** Configure memory region 54. */
    Qmss_MemRegion_MEMORY_REGION54,
    /** Configure memory region 55. */
    Qmss_MemRegion_MEMORY_REGION55,
    /** Configure memory region 56. */
    Qmss_MemRegion_MEMORY_REGION56,
    /** Configure memory region 57. */
    Qmss_MemRegion_MEMORY_REGION57,
    /** Configure memory region 58. */
    Qmss_MemRegion_MEMORY_REGION58,
    /** Configure memory region 59. */
    Qmss_MemRegion_MEMORY_REGION59,
    /** Configure memory region 60. */
    Qmss_MemRegion_MEMORY_REGION60,
    /** Configure memory region 61. */
    Qmss_MemRegion_MEMORY_REGION61,
    /** Configure memory region 62. */
    Qmss_MemRegion_MEMORY_REGION62,
    /** Configure memory region 63. */
    Qmss_MemRegion_MEMORY_REGION63
}Qmss_MemRegion;

/** 
 * @brief PDSP ID
 */
typedef enum
{
    /** PDSP 1 */
    Qmss_PdspId_PDSP1 = 0,
    /** PDSP 2 */
    Qmss_PdspId_PDSP2,
    /** PDSP 3 */
    Qmss_PdspId_PDSP3,
    /** PDSP 4 */
    Qmss_PdspId_PDSP4,
    /** PDSP 5 */
    Qmss_PdspId_PDSP5,
    /** PDSP 6 */
    Qmss_PdspId_PDSP6,
    /** PDSP 7 */
    Qmss_PdspId_PDSP7,
    /** PDSP 8 */
    Qmss_PdspId_PDSP8,
    /** Simulate PDSP with C Model on DSP N=1.  This is for
     * testing purposes, only works with certain FW that have
     * cmodel */
    Qmss_PdspId_CMODEL_DSP1 = 0x101
}Qmss_PdspId;

/**
 * @brief QMSS operation modes
 */
typedef enum
{
    /** JOINT modes have all the specified QMSS operating together as one
     * larger QMSS.  Descriptors can be pushed to any queue on any of the 
     * QMSS.  Linking RAM and descriptor regions automatically added to 
     * all specified QMSS.
     *
     * In LOAD BALANCED mode, queues are opened from the QMSS which
     * has least opened queues.  
     *
     * In ROUND ROBIN mode, queues are opened alternating evenly across
     * all the QMSS, without regard to number of open queues on each QMSS.
     */
    Qmss_Mode_JOINT_LOADBALANCED = 0,
    /** See Qmss_Mode_JOINT_LOADBALANCED for description of ROUND ROBIN */
    Qmss_Mode_JOINT_ROUNDROBIN,
    /** In SPLIT mode, each QMSS operates independantly with its own 
     * descriptor regions and linking RAM regions.  Descriptors can
     * only be pushed into queues on the same QMSS.  Note: this mode
     * is not tested, because while the low level QMSS HW supports it,
     * the SOC doesn't.  The TX queues for peripherals are hardwired
     * to specific sides.  Thus devices that have both QMSS simutaneously
     * need both accessible.  It is not recommended to use this mode.
     */
    Qmss_Mode_SPLIT
} Qmss_Mode;

/** 
 * @brief INTD interrupt types
 */
typedef enum
{
    /** Interrupt generated for the high priority accumulator.
     * 32 interrupts are generated in response to events in the 32 high-priority queues.
     */
    Qmss_IntdInterruptType_HIGH = 0,
    /** Interrupt generated for the low priority accumulator.
     * 16 interrupts are generated in response to events in the 512 low-priority queues.
     */
    Qmss_IntdInterruptType_LOW,
    /** Interrupt generated for QMSS CDMA.
     * 2 interrupts are generated for buffer descriptor starvation event on 
     * receive SOP (start of packet) and MOP (middle of packet) for any of the receive DMA units in the CDMA.
     */
    Qmss_IntdInterruptType_CDMA
}Qmss_IntdInterruptType;

/**
 * @brief QMSS subystem names
 */
typedef enum
{
    /** Global (or only, for devices with just one) subsystem.  This is used
     * by all peripherals that do not have their own subsystem.
     */
    Qmss_SubSys_GLOBAL,
    /**
     * QMSS subsystem inside the networking subsystem */
    Qmss_SubSys_NETSS
} Qmss_SubSys;

/** 
 * @brief Queue Type. Specifies different queue classifications
 */
/* Note: when adding new types, update qmssQTypeStr[] in queueAlloc.c */
typedef enum
{
    /** Unspecified queue type */
    Qmss_QueueType_UNSPECIFIED = QMSS_PARAM_NOT_SPECIFIED,
    /** Low priority queue */
    Qmss_QueueType_LOW_PRIORITY_QUEUE = 0,
    /** AIF queue */
    Qmss_QueueType_AIF_QUEUE,
    /** PASS 0 queue */
    Qmss_QueueType_PASS_QUEUE,
    /** INTC/CIC pending queue (from SET1) */
    Qmss_QueueType_INTC_QUEUE,
    /** INTC/CIC SET2 queue */
    Qmss_QueueType_INTC_SET2_QUEUE,
    /** INTC/CIC SET3 queue */
    Qmss_QueueType_INTC_SET3_QUEUE,
    /** INTC/CIC SET4 queue */
    Qmss_QueueType_INTC_SET4_QUEUE,
    /** INTC/CIC SET5 queue */
    Qmss_QueueType_INTC_SET5_QUEUE,
    /** INTC/CIC & EDMA queue in SET 0 */
    Qmss_QueueType_INTC_EDMA_SET0_QUEUE,
    /** INTC/CIC & EDMA queue in SET 1 */
    Qmss_QueueType_INTC_EDMA_SET1_QUEUE,
    /** INTC/CIC & EDMA queue in SET 2 */
    Qmss_QueueType_INTC_EDMA_SET2_QUEUE,
    /** INTC/CIC & EDMA & GIC queue in set 0 */
    Qmss_QueueType_SOC_SET0_QUEUE,
    /** INTC/CIC & EDMA & GIC queue in set 1 */
    Qmss_QueueType_SOC_SET1_QUEUE,
    /** SRIO queue */
    Qmss_QueueType_SRIO_QUEUE,
    /** FFTC queue A */
    Qmss_QueueType_FFTC_A_QUEUE,
    /** FFTC queue B */
    Qmss_QueueType_FFTC_B_QUEUE,
    /** FFTC queue C */
    Qmss_QueueType_FFTC_C_QUEUE,
    /** FFTC queue D */
    Qmss_QueueType_FFTC_D_QUEUE,
    /** FFTC queue E */
    Qmss_QueueType_FFTC_E_QUEUE,
    /** FFTC queue F */
    Qmss_QueueType_FFTC_F_QUEUE,
    /** BCP queue */
    Qmss_QueueType_BCP_QUEUE,
    /** High priority queue */
    Qmss_QueueType_HIGH_PRIORITY_QUEUE,
    /** starvation counter queue */
    Qmss_QueueType_STARVATION_COUNTER_QUEUE,
    /** Infrastructure queue first QM */
    Qmss_QueueType_INFRASTRUCTURE_QUEUE,
    /** Infrastructure queue second QM */
    Qmss_QueueType_QM2_INFRASTRUCTURE_QUEUE,
    /** Traffic shaping queue */
    Qmss_QueueType_TRAFFIC_SHAPING_QUEUE,
    /** GIC400 queue */
    Qmss_QueueType_GIC400_QUEUE,
    /** EDMA 4 queue */
    Qmss_QueueType_EDMA_4_QUEUE,
    /** Broadcast to Hyperlink 0 and 1 queue */
    Qmss_QueueType_HLINK_BROADCAST_QUEUE,
    /** Hyperlink 0 queue */
    Qmss_QueueType_HLINK_0_QUEUE,
    /** Hyperlink 1 queue */
    Qmss_QueueType_HLINK_1_QUEUE,
    /** XGE (10 gigabit ethernet) queue */
    Qmss_QueueType_XGE_QUEUE,
    /** DXB queue */
    Qmss_QueueType_DXB_QUEUE,
    /** IQNET queue */
    Qmss_QueueType_IQNET_QUEUE,
    /** EDMA 0 queue */
    Qmss_QueueType_EDMA_0_QUEUE,
    /** EDMA 1 queue */
    Qmss_QueueType_EDMA_1_QUEUE,
    /** EDMA 2 queue */
    Qmss_QueueType_EDMA_2_QUEUE,
    /** EDMA 3 queue */
    Qmss_QueueType_EDMA_3_QUEUE,
    /** Receive (GIC/CIC) queue to receive packets for ARM or DSP */
    Qmss_QueueType_RECEIVE_QUEUE,
    /** General purpose queue -- MUST be last */
    Qmss_QueueType_GENERAL_PURPOSE_QUEUE
} Qmss_QueueType;

/**
@}
*/

/** @addtogroup QMSS_LLD_DATASTRUCT
@{ 
*/
/** 
 * @brief QMSS RM Service Handle
 */
typedef void *  Qmss_RmServiceHnd;

/** 
 * @brief Queue definition
 */
typedef struct
{
    /** Queue manager number */
    int32_t qMgr;        
    /** Queue number within Queue Manager */
    int32_t qNum;
}Qmss_Queue;

/** 
 * @brief Queue definition
 */
typedef struct
{
    /** Queue type */
    Qmss_QueueType queueType;
    /** Queue manager number */
    int32_t        startIndex;        
    /** Queue number within Queue Manager */
    int32_t        maxNum;
    /** RM DTS resource name for queues */
    char           rmQueue[QMSS_RM_RESOURCE_NAME_MAX_CHARS];
}Qmss_QueueNumRange;

/** 
 * @brief descriptor configuration structure
 */
typedef struct 
{
    /** Memory region # to split into descriptors */
    uint32_t          memRegion;
    /** Number of descriptors that should be created */
    uint32_t          descNum;
    /** Queue where the descriptor is stored. If QueueNum is set to QMSS_PARAM_NOT_SPECIFIED then the next 
     * available queue of type Qmss_QueueType will be allocated */
    int32_t           destQueueNum;
    /** If QueueNum is set to QMSS_PARAM_NOT_SPECIFIED then the next available queue of type 
     * Qmss_QueueType will be allocated */
    Qmss_QueueType    queueType;
    /** Group for memRegion (only applicable in split mode) */
    uint32_t          queueGroup;
}Qmss_DescCfg;

/** 
 * @brief Memory region configuration information structure
 */
typedef struct 
{
    /** The base address of descriptor region. Note the 
     * descriptor Base address must be specified in ascending memory order
     * */
    uint32_t          *descBase;
    /** Size of each descriptor in the memory region. Must be a multiple of 16 */
    uint32_t          descSize;
    /** Number of descriptors in the memory region. 
     * Must be a minimum of 32. 
     * Must be 2^(5 or greater) 
     * Maximum supported value 2^20
     * */
    uint32_t          descNum;

    /** Memory Region corresponding to the descriptor. 
     * At init time this field must have a valid memory region 
     * index (0 to Maximum number of memory regions supported).
     *
     * At runtime this field is used to either 
     *      * set to Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED, in this case the LLD 
     *      * will decide which memory region to use.
     *      * OR 
     *      * specify the descriptor memory region, must be a valid memory 
     *      * region index (0 to Maximum number of memory regions supported).
     */
    Qmss_MemRegion  memRegion;
    /** Flag control whether the descriptors are managed 
     * by LLD or by the caller allocating descriptor memory */
    Qmss_ManageDesc manageDescFlag;
    /** Used to leave holes by configuring dummy regions which can be later 
     * configured with actual values. May either be specified by the user 
     * (for example, to select "internal" or "external" linking RAM, or
     * set to @ref QMSS_START_INDEX_NOT_SPECIFIED/@ref QMSS_PARAM_NOT_SPECIFIED 
     * to have LLD/RM select a compatible startIndex from anywhere with 
     * free indicies, or @ref QMSS_START_INDEX_INTERNAL for internal linking RAM,
     * or @ref QMSS_START_INDEX_EXTERNAL for external linking RAM. 
     */
    int32_t           startIndex;
    /** In SPLIT mode only, used to specify which queue manager group
     *  this region is intended for
     */
    uint32_t          queueGroup;
} Qmss_MemRegInfo;

/** 
 * @brief Memory region configuration information structure of all memory regions
 */
typedef struct 
{
    /** Descriptor information for each region passed during cppi_Init */
    Qmss_MemRegInfo memRegInfo[QMSS_MAX_MEM_REGIONS];
    /** Current descriptor count. Sum of descriptors in all memory regions */
    uint32_t          currDescCnt;
    /** Group to return (only applicable in split mode) */
    uint32_t          queueGroup;
} Qmss_MemRegCfg;

/** 
 * @brief QMSS PDSP firmware download information structure
 */
typedef struct
{
    /** ID of the PDSP to download this firmware to */
    Qmss_PdspId     pdspId;
    /** Pointer to the firmware image, If the firmware pointer is NULL, LLD will not 
     * download the firmware */
    void            *firmware;
    /** Size of firmware in bytes */
    uint32_t          size; 
}Qmss_PdspCfg;

/** 
 * @brief QMSS configuration structure
 */
typedef struct
{
    /** Base address of Linking RAM 0. LLD will configure linking RAM0 address to internal linking RAM 
     * address if a value of zero is specified. */
    uint32_t          linkingRAM0Base;
    /** Linking RAM 0 Size. LLD will configure linking RAM0 size to maximum internal linking RAM 
     * size if a value of zero is specified*/
    uint32_t          linkingRAM0Size; 
    /** Base address of Linking RAM 1. Depends on RAM 0 Size and total number of 
     * descriptors. If linkingRAM1Base is zero then linkingRAM0Size must be large 
     * enough to store all descriptors in the system */
    uint32_t          linkingRAM1Base;
    /** Maximum number of descriptors in the system. Should be equal to less than 
     * the RAM0+RAM1 size */
    uint32_t          maxDescNum;
} Qmss_LinkingRAM;

/** 
 * @brief QMSS configuration structure
 *
 * The linkingRAMs are specified in two parts.  This allows backwards 
 * compatibility to those * who memset() this structure to 0.  It will default 
 * to JOINT mode so existing code runs without modification since it will be
 * able to see all queue types in the system.
 *
 * In JOINT mode: only linkingRAM0Base, linkingRAM0Size, and linkingRAM1Base are
 * used to configure all QM groups simutaneously.
 *
 * In SPLIT mode: linkingRAM0Base, linkingRAM0Size, and linkingRAM1Base are used
 * to configure the first QM group.  The subsequent groups are configured using
 * splitLinkingRAMs and 
 */
typedef struct
{
    /** Base address of Linking RAM 0. LLD will configure linking RAM0 address to internal linking RAM 
     * address if a value of zero is specified. */
    uint32_t          linkingRAM0Base;
    /** Linking RAM 0 Size. LLD will configure linking RAM0 size to maximum internal linking RAM 
     * size if a value of zero is specified*/
    uint32_t          linkingRAM0Size; 
    /** Base address of Linking RAM 1. Depends on RAM 0 Size and total number of 
     * descriptors. If linkingRAM1Base is zero then linkingRAM0Size must be large 
     * enough to store all descriptors in the system */
    uint32_t          linkingRAM1Base;
    /** Maximum number of descriptors in the system. Should be equal to less than 
     * the RAM0+RAM1 size */
    uint32_t          maxDescNum;
    /** PDSP firmware to download. If the firmware pointer is NULL, LLD will not download the firmware */
    Qmss_PdspCfg      pdspFirmware[QMSS_MAX_PDSP];
    /** Status of QMSS HW. Set this to QMSS_HW_INIT_COMPLETE in case Initialization is already complete.
      * Setting this flag will bypass any QMSS Hardware initialization
      */
    uint32_t          qmssHwStatus;

    /** Allocation mode */
    Qmss_Mode         mode;
#if (QMSS_MAX_QMGR_GROUPS > 1)
    /** Linking RAMs for other QM groups when in SPLIT mode */
    Qmss_LinkingRAM   splitLinkingRAMs[QMSS_MAX_QMGR_GROUPS - 1];
#endif
} Qmss_InitCfg;

/** 
 * @brief QMSS Global register definition for registers that replicate
 * per QM group
 */
typedef struct 
{
    /** QM Global Config registers */
    CSL_Qm_configRegs                       *qmConfigReg;
    /** QM Descriptor Config registers */
    CSL_Qm_descriptor_region_configRegs     *qmDescReg;
    /** QM queue Management registers, accessed via CFG port */
    CSL_Qm_queue_managementRegs             *qmQueMgmtReg;
    /** QM queue Management Proxy registers, accessed via CFG port */
    CSL_Qm_queue_managementRegs             *qmQueMgmtProxyReg;
    /** QM queue status registers */
    CSL_Qm_queue_status_configRegs          *qmQueStatReg;
    /** QM Status RAM */
    CSL_Qm_Queue_Status                     *qmStatusRAM;
    /** QM queue Management registers, accessed via DMA port */
    CSL_Qm_queue_managementRegs             *qmQueMgmtDataReg;
    /** QM queue Management Proxy registers, accessed via DMA port */
    CSL_Qm_queue_managementRegs             *qmQueMgmtProxyDataReg;
} Qmss_GlobalConfigGroupRegs;

/** 
 * @brief QMSS Global RM resource name definitions replicate per QM group
 */
typedef struct
{
    /** RM DTS resource name for overall QM control */
    char rmQmControl[QMSS_RM_RESOURCE_NAME_MAX_CHARS];
    /** RM DTS resource name for allowing Link RAM configurability */
    char rmLinkRamControl[QMSS_RM_RESOURCE_NAME_MAX_CHARS];
    /** RM DTS resource name for Link RAM indices in internal/region 0 */
    char rmLinkRamInt[QMSS_RM_RESOURCE_NAME_MAX_CHARS];
    /** RM DTS resource name for Link RAM indices in external/region 1 */
    char rmLinkRamExt[QMSS_RM_RESOURCE_NAME_MAX_CHARS];
    /** RM DTS resource name for Memory regions */
    char rmMemRegion[QMSS_RM_RESOURCE_NAME_MAX_CHARS];
} Qmss_GlobalConfigGroupRm;

/** 
 * @brief QMSS Global register definition for registers that do not replicate
 * per QM group
 */
typedef struct 
{
    /** QM INTD registers */
    CSL_Qm_intdRegs                         *qmQueIntdReg[QMSS_MAX_INTD];
    /** QM PDSP command register */
    volatile uint32_t                       *qmPdspCmdReg[QMSS_MAX_PDSP];
    /** QM PDSP control register */
    CSL_PdspRegs                            *qmPdspCtrlReg[QMSS_MAX_PDSP];
    /** QM PDSP IRAM register */
    volatile uint32_t                       *qmPdspIRamReg[QMSS_MAX_PDSP];
    /** QM Linking RAM register */
    volatile uint32_t                       *qmLinkingRAMReg;
    /** QM peripheral base address used to calculate internal addresses */
    void                                    *qmBaseAddr;
} Qmss_GlobalConfigRegs;

/** 
 * @brief QMSS Global configuration structure definition
 */
typedef struct
{
    /** Maximum number of queue manager groups */
    uint32_t                                maxQueMgrGroups;
    /** Maximum number of queue Managers */
    uint32_t                                maxQueMgr;
    /** Maximum number of queues */
    uint32_t                                maxQue;
    /** Maximum number of memory regions */
    uint32_t                                maxMemReg;
    /** Maximum number of PDSPs */
    uint32_t                                maxPDSP;
    /** Size of internal linkram */
    uint32_t                                intLinkRamSize;
    /** Requires ordered memory regions? */
    int                                     orderedMemReg;
    /** Number of queue ranges defined in maxQueueNum */
    uint32_t                                numQueueNum[QMSS_MAX_QMGR_GROUPS];
    /** Queue start index and maximum number of queues of each queue type for each queue group */
    Qmss_QueueNumRange                     *maxQueueNum[QMSS_MAX_QMGR_GROUPS];
    /** Register definitions for each QM group */
    Qmss_GlobalConfigGroupRegs              groupRegs[QMSS_MAX_QMGR_GROUPS];
    /** Register definitions for whole SS */
    Qmss_GlobalConfigRegs                   regs;
    /** virtual to physical address translation function for base addr */
    void                                   *(*virt2Phy) (void *virt);
    /** physical to virtual address translation function for base addr */
    void                                   *(*phy2Virt) (void *phy);
    /** virtual to physical address translation function for descriptors */
    void                                   *(*virt2PhyDesc) (uint32_t QID, void *virt);
    /** physical to virtual address translation function for descriptors */
    void                                   *(*phy2VirtDesc) (uint32_t QID, void *phy);
    /** QM stores the Resource Manager service handle for internal use */
    Qmss_RmServiceHnd                       qmRmServiceHandle;
    /** RM DTS resource names for each QM group */
    Qmss_GlobalConfigGroupRm                groupRm[QMSS_MAX_QMGR_GROUPS];
    /** RM DTS resource name for PDSP Firmware download */
    char                                    rmFirmwarePdsp[QMSS_RM_RESOURCE_NAME_MAX_CHARS];
    /** RM DTS resource name for accumulator channels */
    char                                    rmAccumCh[QMSS_MAX_INTD][QMSS_RM_RESOURCE_NAME_MAX_CHARS];
    /** RM nameserver pattern to store region queues (printf format) */
    char                                    rmRegQueueFmt[QMSS_RM_RESOURCE_NAME_MAX_CHARS];
    uint32_t                                pdspIntdMap[QMSS_MAX_PDSP];
} Qmss_GlobalConfigParams;

/** 
 * @brief QMSS start configuration structure
 */
typedef struct
{
    /** Resource Manager service handle */
    Qmss_RmServiceHnd           rmServiceHandle;
    /** Optional parameter in the case of different application
      * running on different cores
      */
    Qmss_GlobalConfigParams*    pQmssGblCfgParams;
} Qmss_StartCfg;
/** 
 * @brief Queue handle
 */
typedef int32_t   Qmss_QueueHnd;

/** 
 * @brief QMSS return result
 */
typedef int32_t   Qmss_Result;

/** 
 * @brief Handle used in the "Fast Push" set of APIs
 */
typedef uint32_t* Qmss_QueuePushHnd;

/**
 * @brief Subsystem Handle used to designate which QMSS on the device 
 */
typedef uint32_t Qmss_SubSysHnd;

/* Predefined global handle, or reference to qmssLObj */
#define QMSS_SUBSYS_HND_GLOBAL (Qmss_SubSysHnd)Qmss_SubSys_GLOBAL

/** 
@} 
*/

/* Exported functions */
extern Qmss_Result Qmss_initSubSys (Qmss_SubSysHnd *subSysHnd, Qmss_SubSys subSys, Qmss_InitCfg *initCfg, Qmss_GlobalConfigParams *qmssGblCfgParams);
extern Qmss_Result Qmss_init (Qmss_InitCfg *initCfg, Qmss_GlobalConfigParams *qmssGblCfgParams);
extern Qmss_Result Qmss_exitSubSys (Qmss_SubSysHnd *subSysHnd);
extern Qmss_Result Qmss_exit (void);
extern Qmss_Result Qmss_start (void);
extern Qmss_Result Qmss_startSubSysCfg(Qmss_SubSysHnd *subSysHnd, Qmss_SubSys subSys, Qmss_StartCfg *startCfg);
extern Qmss_Result Qmss_startCfg(Qmss_StartCfg *startCfg);
extern Qmss_Result Qmss_getMemoryRegionCfgSubSys (Qmss_SubSysHnd subSysHnd, Qmss_MemRegCfg *memRegInfo);
extern Qmss_Result Qmss_getMemoryRegionCfg (Qmss_MemRegCfg *memRegInfo);
extern Qmss_Result Qmss_insertMemoryRegionSubSys (Qmss_SubSysHnd subSysHnd, Qmss_MemRegInfo *memRegCfg);
extern Qmss_Result Qmss_insertMemoryRegion (Qmss_MemRegInfo *memRegCfg);
extern Qmss_Result Qmss_openMemoryRegionSubSys (Qmss_SubSysHnd subSysHnd, Qmss_MemRegion memRegion, uint32_t qGroup);
extern Qmss_Result Qmss_openMemoryRegion (Qmss_MemRegion memRegion, uint32_t qGroup);
extern Qmss_Result Qmss_removeMemoryRegionSubSys (Qmss_SubSysHnd subSysHnd, int32_t region, uint32_t qGroup);
extern Qmss_Result Qmss_removeMemoryRegion (int32_t region, uint32_t qGroup);
extern Qmss_Result Qmss_closeMemoryRegionSubSys (Qmss_SubSysHnd subSysHnd, int32_t region, uint32_t qGroup);
extern Qmss_Result Qmss_closeMemoryRegion (int32_t region, uint32_t qGroup);
extern Qmss_QueueHnd Qmss_initDescriptorSubSys (Qmss_SubSysHnd subSysHnd, Qmss_DescCfg *descCfg, uint32_t *numAllocated);
extern Qmss_QueueHnd Qmss_initDescriptor (Qmss_DescCfg *descCfg, uint32_t *numAllocated);
extern Qmss_QueueHnd Qmss_queueOpenSubSys (Qmss_SubSysHnd subSysHnd, Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated);
extern Qmss_QueueHnd Qmss_queueOpen (Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated);
extern Qmss_QueueHnd Qmss_queueOpenInGroupSubSys (Qmss_SubSysHnd subSysHnd, int32_t qGroup, Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated);
extern Qmss_QueueHnd Qmss_queueOpenInGroup (int32_t qGroup, Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated);
extern Qmss_QueueHnd Qmss_queueOpenInRangeSubSys (Qmss_SubSysHnd subSysHnd, uint32_t startQueNum, uint32_t endQueNum, uint8_t *isAllocated);
extern Qmss_QueueHnd Qmss_queueOpenInRange (uint32_t startQueNum, uint32_t endQueNum, uint8_t *isAllocated);
extern Qmss_QueueHnd Qmss_queueOpenUseSubSys (Qmss_SubSysHnd subSysHnd, uint32_t QID, uint8_t *useCount);
extern Qmss_QueueHnd Qmss_queueOpenUse (uint32_t QID, uint8_t *useCount);
extern Qmss_Result Qmss_queueBlockOpenInGroupSubSys (Qmss_SubSysHnd subSysHnd, Qmss_QueueHnd *queueHnds, int32_t qGroup, Qmss_QueueType queType, int32_t numQ, int32_t align);
extern Qmss_Result Qmss_queueBlockOpenInGroup (Qmss_QueueHnd *queueHnds, int32_t qGroup, Qmss_QueueType queType, int32_t numQ, int32_t align);
extern Qmss_Result Qmss_queueBlockOpenSubSys (Qmss_SubSysHnd subSysHnd, Qmss_QueueHnd *queueHnds, Qmss_QueueType queType, int32_t numQ, int32_t align);
extern Qmss_Result Qmss_queueBlockOpen (Qmss_QueueHnd *queueHnds, Qmss_QueueType queType, int32_t numQ, int32_t align);
extern Qmss_Result Qmss_queueClose (Qmss_QueueHnd hnd);
extern uint32_t Qmss_getQueueThreshold (Qmss_QueueHnd hnd);
extern Qmss_Result Qmss_setQueueThreshold (Qmss_QueueHnd hnd, uint16_t hilo, uint8_t threshold);
extern uint32_t Qmss_getStarvationCount (Qmss_QueueHnd hnd);
extern uint16_t Qmss_getQueueThresholdStatus (Qmss_QueueHnd hnd);
extern Qmss_Queue Qmss_getQueueNumber (Qmss_QueueHnd hnd);
extern Qmss_QueueHnd Qmss_getQueueHandle (Qmss_Queue queue);
extern uint32_t Qmss_getQueueGroup (Qmss_QueueHnd hnd);
extern uint32_t Qmss_getMemRegDescSizeSubSys (Qmss_SubSysHnd subSysHnd, uint32_t memRegion, uint32_t qGroup);
extern uint32_t Qmss_getMemRegDescSize (uint32_t memRegion, uint32_t qGroup);

extern Qmss_Result Qmss_downloadFirmwareSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, void *image, uint32_t size);
extern Qmss_Result Qmss_downloadFirmware (Qmss_PdspId pdspId, void *image, uint32_t size);

extern Qmss_Result Qmss_setEoiVectorByIntdSubSys (Qmss_SubSysHnd subSysHnd, uint8_t intdNum, Qmss_IntdInterruptType type, uint8_t accumCh);
extern Qmss_Result Qmss_setEoiVectorByIntd (uint8_t intdNum, Qmss_IntdInterruptType type, uint8_t accumCh);
extern Qmss_Result Qmss_setEoiVectorSubSys (Qmss_SubSysHnd subSysHnd, Qmss_IntdInterruptType type, uint8_t accumCh);
extern Qmss_Result Qmss_setEoiVector (Qmss_IntdInterruptType type, uint8_t accumCh);

extern Qmss_Result Qmss_setEoiIntNumByIntdSubSys (Qmss_SubSysHnd subSysHnd, uint8_t intdNum, uint8_t interruptNum);
extern Qmss_Result Qmss_setEoiIntNumByIntd (uint8_t intdNum, uint8_t interruptNum);
extern Qmss_Result Qmss_setEoiIntNumSubSys (Qmss_SubSysHnd subSysHnd, uint8_t interruptNum);
extern Qmss_Result Qmss_setEoiIntNum (uint8_t interruptNum);

extern Qmss_Result Qmss_ackInterruptByIntdSubSys (Qmss_SubSysHnd subSysHnd, uint8_t intdNum, uint8_t interruptNum, uint8_t value);
extern Qmss_Result Qmss_ackInterruptByIntd (uint8_t intdNum, uint8_t interruptNum, uint8_t value);
extern Qmss_Result Qmss_ackInterruptSubSys (Qmss_SubSysHnd subSysHnd, uint8_t interruptNum, uint8_t value);
extern Qmss_Result Qmss_ackInterrupt (uint8_t interruptNum, uint8_t value);

extern Qmss_QueueHnd Qmss_getMemRegQueueHandleSubSys (Qmss_SubSysHnd subSysHnd, uint32_t memRegion);
extern Qmss_QueueHnd Qmss_getMemRegQueueHandle (uint32_t memRegion);

extern Qmss_Result Qmss_getStarvationCounts (Qmss_QueueHnd hnd, uint32_t numCounts, uint32_t *pStarvationCounts);
extern uint32_t Qmss_getVersion (void);
extern const char* Qmss_getVersionStr (void);
 
#ifdef __cplusplus
}
#endif

#endif /* QMSS_QM_H_ */

