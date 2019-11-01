/**
 *   @file  cppi_desc.h
 *
 *   @brief   
 *      This is the CPPI Descriptor Management include file.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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


/** 
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 *
 * @subsection References
 *   -# CPPI Functional Specification 
 *   -# Queue Manager Subsystem Specification 
 *
 * @subsection Assumptions
 *    
 */
#ifndef CPPI_DESC_H_
#define CPPI_DESC_H_


#ifdef __cplusplus
extern "C" {
#endif
/* 
 * Shut off: remark #880-D: parameter "descType" was never referenced
 *
 * This is better than removing the argument since removal would break
 * backwards compatibility
 */
#ifdef _TMS320C6X
#pragma diag_suppress 880
#pragma diag_suppress 681
#elif defined(__GNUC__)
/* Same for GCC:
 * warning: unused parameter descType [-Wunused-parameter]
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif

#include <ti/drv/cppi/cppi_drv.h>
/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_drv.h>

/* System includes */
#include <string.h>

/** @addtogroup CPPI_LLD_SYMBOL
@{
*/

/** Monolithic descriptor extended packet information block size */
#define CPPI_MONOLITHIC_DESC_EPIB_SIZE      20
/** Host descriptor extended packet information block size */
#define CPPI_HOST_DESC_EPIB_SIZE            16

/** 
@} 
*/

/** @addtogroup CPPI_LLD_ENUM
@{ 
*/

/** 
 * @brief CPPI descriptor types
 */
typedef enum
{
    /** Host descriptor */
    Cppi_DescType_HOST = 0,
    /** Monolithic descriptor */
    Cppi_DescType_MONOLITHIC = 2
}Cppi_DescType;

/** 
 * @brief Packet return policy
 */
typedef enum
{
    /** Return entire packet */
    Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET = 0,
    /** Return one buffer at a time */
    Cppi_ReturnPolicy_RETURN_BUFFER 
}Cppi_ReturnPolicy;

/** 
 * @brief protocol specific information location 
 */
typedef enum
{
    /** protocol specific information is located in descriptor */
    Cppi_PSLoc_PS_IN_DESC = 0,
    /** protocol specific information is located in SOP buffer */
    Cppi_PSLoc_PS_IN_SOP
}Cppi_PSLoc;

/** 
 * @brief extended packet information block
 */
typedef enum
{
    /** extended packet information block is not present in descriptor */
    Cppi_EPIB_NO_EPIB_PRESENT = 0,
    /** extended packet information block is present in descriptor  */
    Cppi_EPIB_EPIB_PRESENT
}Cppi_EPIB;

/** 
 * @brief Descriptor resource management
 */
typedef enum
{
    /** LLD initializes the descriptors with specified values */
    Cppi_InitDesc_INIT_DESCRIPTOR = 0,
    /** LLD does not initialize the descriptor with specified values */    
    Cppi_InitDesc_BYPASS_INIT 
}Cppi_InitDesc;

/** 
@} 
*/

/** @addtogroup CPPI_LLD_DATASTRUCT
@{ 
*/

/** 
 * @brief CPPI host descriptor configuration structure
 */
typedef struct 
{
    /** Indicates return policy for the packet. 
     * Valid only for host descriptor */
    Cppi_ReturnPolicy       returnPolicy;
    /** Indicates protocol specific location CPPI_PS_DESC - located in descriptor, CPPI_PS_SOP - located in SOP buffer 
     * Valid only for host descriptor */
    Cppi_PSLoc              psLocation;
}Cppi_HostDescCfg;

/** 
 * @brief CPPI monolithic descriptor configuration structure
 */
typedef struct 
{
    /** Byte offset from byte 0 of monolithic descriptor to the location where the valid data begins */
    uint32_t                  dataOffset;
}Cppi_MonolithicDescCfg;

/** 
 * @brief CPPI descriptor configuration structure
 */
typedef struct 
{
    /** Memory Region corresponding to the descriptor. */
    Qmss_MemRegion          memRegion;
    /** Group for memRegion (only applicable in when QMSS is in split mode) */
    uint32_t                queueGroup;
    /** Number of descriptors that should be configured with value below */
    uint32_t                  descNum;
    /** Queue where the descriptor is stored. If destQueueNum is set to QMSS_PARAM_NOT_SPECIFIED then the next 
     * available queue of type Qmss_QueueType will be allocated.  This is
     * actually a 16-bit QID including queue manager number. */
    int32_t                   destQueueNum;
    /** If destQueueNum is set to QMSS_PARAM_NOT_SPECIFIED then the next available queue of type 
     * Qmss_QueueType will be allocated */
    Qmss_QueueType          queueType;

    /** Descriptor configuration parameters */
    /** Indicates if the descriptor should be initialized with parameters listed below */
    Cppi_InitDesc           initDesc;

    /** Type of descriptor - Host or Monolithic */
    Cppi_DescType           descType;
    /** Indicates return Queue Manager and Queue Number. If both qMgr and qNum in returnQueue is 
     * set to QMSS_PARAM_NOT_SPECIFIED then the destQueueNum is configured in returnQueue of the descriptor */
    Qmss_Queue              returnQueue;
    /** Indicates how the CPDMA returns descriptors to free queue */
    Qmss_Location           returnPushPolicy;
    /** Indicates presence of EPIB */
    Cppi_EPIB               epibPresent;

    /** Union contains configuration that should be initialized in for host or monolithic descriptor. 
     * The configuration for host or monolithic descriptor is choosen based on "descType" field. 
     * The approriate structure fields must be specified if "initDesc" field is set to CPPI_INIT_DESCRIPTOR. */
    union{
    /** Host descriptor configuration parameters */
    Cppi_HostDescCfg        host;
    /** Monolithic  descriptor configuration parameters */
    Cppi_MonolithicDescCfg  mono;
    }cfg;

}Cppi_DescCfg;

/** 
 * @brief CPPI descriptor Word 1 Tag information
 */
typedef struct {
    uint8_t srcTagHi;
    uint8_t srcTagLo;
    uint8_t destTagHi;
    uint8_t destTagLo;
}Cppi_DescTag;

/** 
 * @brief CPPI host descriptor layout
 */
typedef struct {
    /** Descriptor type, packet type, protocol specific region location, packet length */
    uint32_t          descInfo;  
    /** Source tag, Destination tag */
    uint32_t          tagInfo;
    /** EPIB present, PS valid word count, error flags, PS flags, return policy, return push policy, 
     * packet return QM number, packet return queue number */
    uint32_t          packetInfo;
    /** Number of valid data bytes in the buffer */
    uint32_t          buffLen;
    /** Byte aligned memory address of the buffer associated with this descriptor */
    uint32_t          buffPtr;
    /** 32-bit word aligned memory address of the next buffer descriptor */
    uint32_t          nextBDPtr;       
    /** Completion tag, original buffer size */
    uint32_t          origBufferLen;
    /** Original buffer pointer */
    uint32_t          origBuffPtr;
    /** Optional EPIB word0 */
    uint32_t          timeStamp;
    /** Optional EPIB word1 */
    uint32_t          softwareInfo0;
    /** Optional EPIB word2 */
    uint32_t          softwareInfo1;
    /** Optional EPIB word3 */
    uint32_t          softwareInfo2;
    /** Optional protocol specific data */
    uint32_t          psData; 
}Cppi_HostDesc;

/** 
 * @brief CPPI monolithic descriptor layout
 */
typedef struct {
    /** Descriptor type, packet type, data offset, packet length */
    uint32_t          descInfo;  
    /** Source tag, Destination tag */
    uint32_t          tagInfo;
    /** EPIB present, PS valid word count, error flags, PS flags, return push policy, 
     * packet return QM number, packet return queue number */
    uint32_t          packetInfo;
    /** NULL word to align the extended packet words to a 128 bit boundary */
    uint32_t          Reserved;
    /** Optional EPIB word0 */
    uint32_t          timeStamp;
    /** Optional EPIB word1 */
    uint32_t          softwareInfo0;
    /** Optional EPIB word2 */
    uint32_t          softwareInfo1;
    /** Optional EPIB word3 */
    uint32_t          softwareInfo2;
    /** Optional protocol specific data */
    uint32_t          psData; 
}Cppi_MonolithicDesc;

/** 
 * @brief CPPI descriptor
 */
typedef union {
    /** Host descriptor */
    Cppi_HostDesc       *ptrHostDesc;  
    /** Monolithic descriptor */
    Cppi_MonolithicDesc *ptrMonoDesc;
}Cppi_Desc;

/** 
@} 
*/

/** @addtogroup CPPI_LLD_FUNCTION
@{ 
*/

/**
 *  @b Description
 *  @n  
 *      This function is used to set the type of descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC, Cppi_DescType_TEARDOWN
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      None.
 */
static inline void Cppi_setDescType (Cppi_Desc *descAddr, Cppi_DescType descType)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    /* No validation of input parameters is done */

    CSL_FINSR (descPtr->descInfo, 31, 30, descType);
    return;
}
/**
 *  @b Description
 *  @n  
 *      This function is used to get the type of descriptor.
 *      Call this function if the descriptor type is not known so it 
 *      can be passed to the remaining descriptor manipulation functions.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      Word 0(bits 30 to 31) are returned
 *
 *      0 - Host descriptor
 *      2 - Monolithic descriptor
 */
static inline Cppi_DescType Cppi_getDescType (Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    /* No validation of input parameters is done */
    return (Cppi_DescType) CSL_FEXTR (descPtr->descInfo, 31, 30);
}

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the error flags (Word 2 bits 20:23) from the host or 
 *      monolithic descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      4 bit Error flag value from descriptor.
 */
static inline uint32_t Cppi_getDescError (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    /* No validation of input parameters is done */

    return (CSL_FEXTR (descPtr->packetInfo, 23, 20));
}

/**
 *  @b Description
 *  @n  
 *      This function copies the data buffer to the host or monolithic descriptor. 
 *      It is assumed that enough words are available to store the payload in case 
 *      of monolithic descriptor.
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  buffAddr
 *      Memory address of data buffer.
 *      
 *  @param[in]  buffLen
 *      Size of the data buffer.
 *
 *  @pre  
 *      Descriptor and data buffer must be allocated and be valid.
 *
 *  @post  
 *      In case of host descriptor 
 *          Word 3 and Word 4 are updated
 *
 *      In case of monolithic descriptor 
 *          Word 0 (bits 0 to 15) and payload are updated.
 *
 *  @retval
 *      None.
 */
static inline void Cppi_setData (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t *buffAddr, uint32_t buffLen)
{
    Cppi_HostDesc   *hostDescPtr;

    /* Does not update packet length */
    if (descType == Cppi_DescType_HOST)
    {
        hostDescPtr = (Cppi_HostDesc *) descAddr;
        hostDescPtr->buffPtr = (uint32_t) buffAddr;
        hostDescPtr->buffLen = (uint32_t) buffLen;
    }
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        uint16_t              dataOffset;

        dataOffset = CSL_FEXTR (monolithicDescPtr->descInfo, 24, 16);
        memcpy ((void *) (((uint8_t *) monolithicDescPtr) + dataOffset), buffAddr, buffLen);        
        CSL_FINSR (monolithicDescPtr->descInfo, 15, 0, buffLen);
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the data buffer pointer from the host descriptor
 *      and payload from monolithic descriptor. 
 *      Note that if PS data is present in SOP buffer, the offset to data must calculated by the caller
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[out]  buffAddr
 *      Memory address of data buffer.
 *      
 *  @param[out]  buffLen
 *      Size of the data buffer.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      In case of host descriptor Word 3 and Word 4 are returned
 *      In case of monolithic descriptor Word 0 (bits 0 to 15) and payload are returned.
 *
 *  @retval
 *      None.
 */
static inline void Cppi_getData (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t **buffAddr, uint32_t *buffLen)
{
    /* For monolithic, start of payload is returned. Check if need to skip data offset bytes */
    Cppi_HostDesc   *hostDescPtr;

    if (descType == Cppi_DescType_HOST)
    {
        hostDescPtr = (Cppi_HostDesc *) descAddr;
        *buffAddr = (uint8_t *) hostDescPtr->buffPtr;
        *buffLen = hostDescPtr->buffLen;
    }
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        uint16_t              dataOffset;

        dataOffset = CSL_FEXTR (monolithicDescPtr->descInfo, 24, 16);

        *buffAddr = (uint8_t *) (((uint8_t *) monolithicDescPtr) + dataOffset);
        *buffLen = CSL_FEXTR (monolithicDescPtr->descInfo, 15, 0);
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function sets the data length in host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  buffLen
 *      Size of the data.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      In case of host descriptor 
 *          Word 3 is updated
 *
 *      In case of monolithic descriptor 
 *          Word 0 (bits 0 to 15) is updated.
 *
 *  @retval
 *      None.
 */
static inline void Cppi_setDataLen (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t buffLen)
{
    Cppi_HostDesc   *hostDescPtr;

    /* Does not update packet length */
    if (descType == Cppi_DescType_HOST)
    {
        hostDescPtr = (Cppi_HostDesc *) descAddr;
        hostDescPtr->buffLen = (uint32_t) buffLen;
    }
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        CSL_FINSR (monolithicDescPtr->descInfo, 15, 0, buffLen);
    }
    return;
}


/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the data length from host or monolithic descriptor
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      In case of host descriptor Word 3 is returned
 *      In case of monolithic descriptor Word 0 (bits 0 to 15) is returned.
 *   
 */
static inline uint32_t Cppi_getDataLen (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    /* For monolithic, start of payload is returned. Check if need to skip data offset bytes */
    Cppi_HostDesc   *hostDescPtr;

    if (descType == Cppi_DescType_HOST)
    {
        hostDescPtr = (Cppi_HostDesc *) descAddr;
        return (hostDescPtr->buffLen);
    }
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        return (CSL_FEXTR (monolithicDescPtr->descInfo, 15, 0));
    }
}

/**
 *  @b Description
 *  @n  
 *      This function is used to link a host descriptor with the next buffer descriptor.
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  nextBD
 *      Memory address of buffer descriptor that should be linked.
 *      
 *  @pre  
 *      Both descriptors must be allocated and be valid.
 *
 *  @post  
 *      Word5 is updated
 *
 *  @retval
 *      None.
 */
static inline void Cppi_linkNextBD (Cppi_DescType descType, Cppi_Desc *descAddr, Cppi_Desc *nextBD)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    hostDescPtr->nextBDPtr = (uint32_t) nextBD;
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the next buffer descriptor pointer from a host packet. 
 *      If the value is zero, then the current buffer is the last buffer in the packet.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      Word 5 of host descriptor 
 *      32 bit word aligned memory address - if current buffer is not the last buffer in packet.
 *      0  - if current buffer is the last buffer in packet.
 */
static inline Cppi_Desc* Cppi_getNextBD (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    return (Cppi_Desc *) hostDescPtr->nextBDPtr;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the original buffer information in the host descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  buffAddr
 *      Memory address of data buffer pointer.
 *      
 *  @param[in]  buffLen
 *      Size of the original data buffer.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      In case of host descriptor Word 6(bits 0 to 21) and Word 7 are updated
 *
 *  @retval
 *      None.
 */
static inline void Cppi_setOriginalBufInfo (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t *buffAddr, uint32_t buffLen)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    hostDescPtr->origBuffPtr = (uint32_t) buffAddr;
    CSL_FINSR (hostDescPtr->origBufferLen, 21, 0, buffLen);
 
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the original buffer information from host descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[out]  buffAddr
 *      Memory address of data buffer pointer.
 *      
 *  @param[out]  buffLen
 *      Size of the original data buffer.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      In case of host descriptor Word 6(bits 0 to 21) and Word 7 are returned
 *
 *  @retval
 *      None.
 */
static inline void Cppi_getOriginalBufInfo (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t **buffAddr, uint32_t *buffLen)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    *buffAddr = (uint8_t *) hostDescPtr->origBuffPtr;
    *buffLen = CSL_FEXTR (hostDescPtr->origBufferLen, 21, 0);

    return;
}
/**
 *  @b Description
 *  @n  
 *      This function is used to set the packet type in host or monolithic descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  packetType
 *      Indicates type of packet. Valid range is 0 to 31.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 0 (bits 25 to 29) of host or monolithic descriptor are updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setPacketType (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t packetType)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (descPtr->descInfo, 29, 25, packetType);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the packet type from host or monolithic descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      5 bit packet type field. Valid range is 0 to 31.
 */
static inline uint8_t Cppi_getPacketType (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    return (CSL_FEXTR (descPtr->descInfo, 29, 25));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the timestamp field in host or 
 *      monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  timeStamp
 *      Timestamp value.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated. 
 *
 *  @post
 *      Word 2 (bit 31) is updated.
 *      Word 0 of Extended Packet Info Block is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setTimeStamp (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t timeStamp)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (hostDescPtr->packetInfo, 31, 31, (uint32_t) 1);
    if (descType == Cppi_DescType_HOST)
        hostDescPtr->timeStamp = timeStamp;
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        monolithicDescPtr->timeStamp = timeStamp;
    }
    return;
}
/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the timestamp field from the host or 
 *      monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[out]  timeStamp
 *      Timestamp value.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated and EPIB bit must be set in the descriptor 
 *
 *  @post  
 *      Word 0 of Extended Packet Info Block is returned.
 *
 *  @retval
 *      Success -   CPPI_SOK 
 *  @retval
 *      Failure -   CPPI_EPIB_NOT_PRESENT if EPIB bit is not set in the descriptor. 
 */
static inline Cppi_Result Cppi_getTimeStamp (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t *timeStamp)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    if (CSL_FEXTR (hostDescPtr->packetInfo, 31, 31) == 0)
    {
        return CPPI_EPIB_NOT_PRESENT;
    }
    if (descType == Cppi_DescType_HOST)
        *timeStamp = hostDescPtr->timeStamp;
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        *timeStamp = monolithicDescPtr->timeStamp;
    }
    return CPPI_SOK;
}

/**
 *  @b Description
 *  @n  
 *      This function is used copy the optional software info field to the host or 
 *      monolithic descriptor. 
 *      It is assumed that enough words are available in the descriptor to store 
 *      information block.
 *      The API copies 3 words of software info into the descriptor.
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  infoAddr
 *      Pointer to the first word of software information block.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated.
 *
 *  @post 
 *   *  @post  
 *      Word 2 (bit 31) is updated.
 *      Word 1 to Word 3 of Extended Packet Info Block is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setSoftwareInfo (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t *infoAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    /* Does not update packet length */
    CSL_FINSR (hostDescPtr->packetInfo, 31, 31, (uint32_t) 1);
    
    if (descType == Cppi_DescType_HOST)
        memcpy ((void *) &hostDescPtr->softwareInfo0, infoAddr, 12);
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        memcpy ((void *) &monolithicDescPtr->softwareInfo0, infoAddr, 12);
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the 3 words of software info field from the host or 
 *      monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[out]  infoAddr
 *      Pointer to the first word of software information block.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Pointer to Word 1 of Extended Packet Info Block is returned.
 *      EPIB block must be allocated and EPIB bit must be set in the descriptor 
 *
 *  @retval
 *      Success -   CPPI_SOK
 *  @retval
 *      Failure -   CPPI_EPIB_NOT_PRESENT if EPIB bit is not set in the descriptor.
 */
static inline Cppi_Result Cppi_getSoftwareInfo (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t **infoAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    if (CSL_FEXTR (hostDescPtr->packetInfo, 31, 31) == 0)
    {
        return CPPI_EPIB_NOT_PRESENT;
    }
    if (descType == Cppi_DescType_HOST)
        *infoAddr = (uint8_t *) &hostDescPtr->softwareInfo0;
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        *infoAddr = (uint8_t *) &monolithicDescPtr->softwareInfo0;
    }
    return CPPI_SOK;
}

/**
 *  @b Description
 *  @n  
 *      This function is used copy the 1st word of optional software info field to the host or 
 *      monolithic descriptor. 
 *      It is assumed that enough words are available in the descriptor to store 
 *      information block.
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  value
 *      Value to write to word 1 of software information block.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated.
 *
 *  @post 
 *      Word 2 (bit 31) is updated.
 *      Word 1 of Extended Packet Info Block is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setSoftwareInfo0 (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t value)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    /* Does not update packet length */
    CSL_FINSR (hostDescPtr->packetInfo, 31, 31, (uint32_t) 1);
    
    if (descType == Cppi_DescType_HOST)
        hostDescPtr->softwareInfo0 = value;
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        monolithicDescPtr->softwareInfo0 = value;
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the 1st word of software info field from the host or 
 *      monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated and EPIB bit must be set in the descriptor
 *
 *  @retval
 *      4 bytes of software info word 1
 *
 */
static inline uint32_t Cppi_getSoftwareInfo0 (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    if (descType == Cppi_DescType_HOST)
        return (hostDescPtr->softwareInfo0);
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        return (monolithicDescPtr->softwareInfo0);
    }
}


/**
 *  @b Description
 *  @n  
 *      This function is used copy the 2nd word of optional software info field to the host or 
 *      monolithic descriptor. 
 *      It is assumed that enough words are available in the descriptor to store 
 *      information block.
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *      **Does not update EPIB present bit**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  value
 *      Value to write to word 2 of software information block.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated.
 *
 *  @post 
 *      Word 2 of Extended Packet Info Block is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setSoftwareInfo1 (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t value)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    if (descType == Cppi_DescType_HOST)
        hostDescPtr->softwareInfo1 = value;
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        monolithicDescPtr->softwareInfo1 = value;
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the 2nd word of software info field from the host or 
 *      monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated and EPIB bit must be set in the descriptor 
 *
 *  @retval
 *      4 bytes of software info word 2
 *
 *  @retval
 *      Failure -   CPPI_EPIB_NOT_PRESENT if EPIB bit is not set in the descriptor.
 */
static inline uint32_t Cppi_getSoftwareInfo1 (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    if (descType == Cppi_DescType_HOST)
        return (hostDescPtr->softwareInfo1);
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        return (monolithicDescPtr->softwareInfo1);
    }
}

/**
 *  @b Description
 *  @n  
 *      This function is used copy the 3rd word of optional software info field to the host or 
 *      monolithic descriptor. 
 *      It is assumed that enough words are available in the descriptor to store 
 *      information block.
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *      **Does not update EPIB present bit**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  value
 *      Value to write to word 3 of software information block.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated.
 *
 *  @post 
 *      Word 3 of Extended Packet Info Block is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setSoftwareInfo2 (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t value)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    if (descType == Cppi_DescType_HOST)
        hostDescPtr->softwareInfo2 = value;
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        monolithicDescPtr->softwareInfo2 = value;
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the 3rd word of software info field from the host or 
 *      monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *      EPIB block must be allocated and EPIB bit must be set in the descriptor 
 *
 *  @retval
 *      4 bytes of software info word 3
 *
 *  @retval
 *      Failure -   CPPI_EPIB_NOT_PRESENT if EPIB bit is not set in the descriptor.
 */
static inline uint32_t Cppi_getSoftwareInfo2 (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    if (descType == Cppi_DescType_HOST)
        return (hostDescPtr->softwareInfo2);
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        return (monolithicDescPtr->softwareInfo2);
    }
}

/**
 *  @b Description
 *  @n  
 *      This function is used to copy the protocol specific data to the host or 
 *      monolithic descriptor. Used when PS data is located in the descriptor.
 *      This function should not be used to copy PS data to SOP buffer.
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  dataAddr
 *      Pointer to the first word of protocol specific data.
 *
 *  @param[in]  dataLen
 *      Size of the PS data.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 2 (bits 24 to 29) are updated.
 *      Word 0-N of PS data block is updated.
 *
 *  @retval
 *      Address of the PSData that has already been set.
 */
static inline uint8_t *Cppi_setPSData (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t *dataAddr, uint32_t dataLen)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;
    uint8_t          epibPresent;
    uint8_t         *psDataAddr;
 
    /* Does not update packet length */
    CSL_FINSR (hostDescPtr->packetInfo, 29, 24, (dataLen / 4));

    epibPresent = CSL_FEXTR (hostDescPtr->packetInfo, 31, 31);

    if (descType == Cppi_DescType_HOST)
    {
        psDataAddr = (((uint8_t *) &hostDescPtr->psData) - (!epibPresent * CPPI_HOST_DESC_EPIB_SIZE));
    }
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        psDataAddr = (((uint8_t *) &monolithicDescPtr->psData) - (!epibPresent * CPPI_MONOLITHIC_DESC_EPIB_SIZE));
    }

    memcpy ((void *) psDataAddr, dataAddr, dataLen);

    return (psDataAddr);
}

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the protocol specific data from the host or 
 *      monolithic descriptor. In case of host descriptor the PS data is read from descriptor 
 *      or SOP buffer based on the PS location. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  location
 *      PS region location 
 *          CPPI_PS_DESC - PS words are located in the descriptor
 *          CPPI_PS_SOP  - PS words are located inthe SOP buffer immediately prior to data.
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[out]  dataAddr
 *      Pointer to the first word of protocol specific data.
 *
 *  @param[out]  dataLen
 *      Size of the PS data.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      For host descriptor, Pointer to Word 0 of PS data block or SOP buffer is returned.
 *      For monolithic descriptor, Pointer to Word 0 of PS data block is returned.
 *
 *  @retval
 *      Success -   CPPI_SOK
 *  @retval
 *      Failure -   CPPI_PSDATA_NOT_PRESENT if PS length is zero in the descriptor.
 */
static inline Cppi_Result Cppi_getPSData (Cppi_DescType descType, Cppi_PSLoc location, Cppi_Desc *descAddr, uint8_t **dataAddr, uint32_t *dataLen)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;
    uint8_t           epibPresent;

    if ((*dataLen = CSL_FEXTR (hostDescPtr->packetInfo, 29, 24) * 4) == 0)
    {
        return CPPI_PSDATA_NOT_PRESENT;
    }

    epibPresent = CSL_FEXTR (hostDescPtr->packetInfo, 31, 31);

    if (descType == Cppi_DescType_HOST)
    {
        if (location == Cppi_PSLoc_PS_IN_SOP)
            *dataAddr = (uint8_t *) hostDescPtr->buffPtr;
        else
            *dataAddr = (uint8_t *) (((uint8_t *) &hostDescPtr->psData) - (!epibPresent * CPPI_HOST_DESC_EPIB_SIZE));
    }
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        *dataAddr = (uint8_t *) (((uint8_t *) &monolithicDescPtr->psData) - (!epibPresent * CPPI_MONOLITHIC_DESC_EPIB_SIZE));
    }
    return CPPI_SOK;
}

/**
 *  @b Description
 *  @n  
 *      This function is update protocol specific data length in host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *      **Does not update packet length**.
 *
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  dataLen
 *      Size of PS data.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 2 (bits 24 to 29) are updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setPSLen (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t dataLen)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;
 
    /* Does not update packet length */
    CSL_FINSR (descPtr->packetInfo, 29, 24, (dataLen / 4));
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the protocol specific data length from host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      Size of PS data in bytes. Valid range is 0 to 128 bytes
 */
static inline uint32_t Cppi_getPSLen (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    return ((CSL_FEXTR (descPtr->packetInfo, 29, 24) * 4));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the packet length in host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  packetLen
 *      Size of packet.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 0 (bits 0 to 21) of host descriptor are updated.
 *      Word 0 (bits 0 to 15) of monolithic descriptor are updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setPacketLen (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t packetLen)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    if (descType == Cppi_DescType_HOST)
        CSL_FINSR (descPtr->descInfo, 21, 0, packetLen);
    else
        CSL_FINSR (descPtr->descInfo, 15, 0, packetLen);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the packet length from host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      Size of packet in bytes
 */
static inline uint32_t Cppi_getPacketLen (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    if (descType == Cppi_DescType_HOST)
        return (CSL_FEXTR (descPtr->descInfo, 21, 0));
    else
        return (CSL_FEXTR (descPtr->descInfo, 15, 0));
}
/**
 *  @b Description
 *  @n  
 *      This function is used to set the protocol specific region location in host descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *      
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  location
 *      PS region location 
 *          CPPI_PS_DESC - PS words are located in the descriptor
 *          CPPI_PS_SOP  - PS words are located inthe SOP buffer immediately prior to data.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 0 (bit 22) of host descriptor is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setPSLocation (Cppi_DescType descType, Cppi_Desc *descAddr, Cppi_PSLoc location)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (hostDescPtr->descInfo, 22, 22, location);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the protocol specific region location from host descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *   
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *    
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      CPPI_PS_DESC - PS words are located in the descriptor
 *  @retval
 *      CPPI_PS_SOP  - PS words are located inthe SOP buffer immediately prior to data.
 */
static inline Cppi_PSLoc Cppi_getPSLocation (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    return ((Cppi_PSLoc) CSL_FEXTR (hostDescPtr->descInfo, 22, 22));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the protocol specific flags in host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *    
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  psFlags
 *      4 bit Protocol Specific flags value.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 2 (bits 16 to 19) of descriptor are updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setPSFlags (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t psFlags)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (descPtr->packetInfo, 19, 16, psFlags);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the protocol specific flags from host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      4 bit PS flags value from descriptor.
 */
static inline uint8_t Cppi_getPSFlags (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    return (CSL_FEXTR (descPtr->packetInfo, 19, 16));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the original buffer pool index from which the attached buffer 
 *      was originally allocated from. This is different from the descriptor pool/queue index 
 *      since a single buffer may be referenced by more than one descriptor.
 *      
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *    
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  poolIndex
 *      index of buffer pool.
 *          
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 6 (Bit 28 to 31) of host descriptor is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setOrigBufferpooIndex (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t poolIndex)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (hostDescPtr->origBufferLen, 31, 28, poolIndex);

    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the original buffer pool index from which the attached buffer 
 *      was originally allocated from. This is different from the descriptor pool/queue index 
 *      since a single buffer may be referenced by more than one descriptor.
 *      
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *    
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      Word 6 (bit 28 to 31) of host descriptor 
 */
static inline uint8_t Cppi_getOrigBufferpooIndex (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    return (CSL_FEXTR (hostDescPtr->origBufferLen, 31, 28));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to increment the number of references that have been made to the 
 *      attached buffer by different descriptors.  Multiple buffer references are commonly used to 
 *      implement broadcast and multicast packet forwarding when zero packet data copies are desired. 
 *      
 *      **No validation is done on the input parameters**.
 *      **No check is made to prevent overflow **.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *    
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 6 (Bit 22 to 27) of host descriptor is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_incrementRefCount (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;
    uint8_t        count;

    count = CSL_FEXTR (hostDescPtr->origBufferLen, 27, 22);
    CSL_FINSR (hostDescPtr->origBufferLen, 27, 22, ++count);

    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to decrement the number of references that have been made to the 
 *      attached buffer by different descriptors.  Multiple buffer references are commonly used to 
 *      implement broadcast and multicast packet forwarding when zero packet data copies are desired. 
 *      
 *      **No validation is done on the input parameters**.
 *      **No check is made to prevent overflow **.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *    
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 6 (Bit 22 to 27) of host descriptor is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_decrementRefCount (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;
    uint8_t        count;

    count = CSL_FEXTR (hostDescPtr->origBufferLen, 27, 22);
    CSL_FINSR (hostDescPtr->origBufferLen, 27, 22, --count);

    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the number of references that have been made 
 *      to the attached buffer by different descriptors.  Multiple buffer references are commonly 
 *      used to implement broadcast and multicast packet forwarding when zero packet data copies are desired. 
 *      
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *    
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      Word 6 (bit 22 to 27) of host descriptor 
 */
static inline uint8_t Cppi_getRefCount (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    return (CSL_FEXTR (hostDescPtr->origBufferLen, 27, 22));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the payload data offset in the monolithic descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_MONOLITHIC
 *    
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  dataOffset
 *      Data offset from byte 0 of word 0 of the descriptor to the location where 
 *      valid data begins.
 *      Valid range is 0 to 511 bytes.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 0 (bits 16 to 24) of monolithic descriptor are updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setDataOffset (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t dataOffset)
{
    Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;

    CSL_FINSR (monolithicDescPtr->descInfo, 24, 16, dataOffset);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the payload data offset from the monolithic descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      9 bit data offset. Valid range is 0 to 511 bytes
 */
static inline uint32_t Cppi_getDataOffset (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;

    return (CSL_FEXTR (monolithicDescPtr->descInfo, 24, 16));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the return policy for the packet in host descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  returnPolicy
 *      RETURN_ENTIRE_PACKET - Entire packet (linked with MOP and EOP BDs and data buffers) should be returned 
 *                      to the return queue specified in SOP descriptor.
 *      RETURN_BUFFER - Each buffer in this packet should be returned to the queue specified in its respective 
 *                      SOP, MOP or EOP descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 2 (bit 15) of host descriptor is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setReturnPolicy (Cppi_DescType descType, Cppi_Desc *descAddr, Cppi_ReturnPolicy returnPolicy)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (hostDescPtr->packetInfo, 15, 15, returnPolicy);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the return policy for the packet from host descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      RETURN_ENTIRE_PACKET - Entire packet (linked with MOP and EOP BDs and data buffers) should be returned 
 *                      to the return queue specified in SOP descriptor.
 *  @retval
 *      RETURN_BUFFER - Each buffer in this packet should be returned to the queue specified in its respective 
 *                      SOP, MOP or EOP descriptor.
 */
static inline Cppi_ReturnPolicy Cppi_getReturnPolicy (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    return ((Cppi_ReturnPolicy) CSL_FEXTR (hostDescPtr->packetInfo, 15, 15));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the return push policy in host or monolithic descriptor. 
 *      Return push policy is valid only if Return Policy is set to 1 in host descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  returnPushPolicy
 *      TAIL - Descriptor must be returned to tail of queue.
 *      HEAD - Descriptor must be returned to head of queue.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 2 (bit 14) of descriptor is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setReturnPushPolicy (Cppi_DescType descType, Cppi_Desc *descAddr, Qmss_Location returnPushPolicy)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (hostDescPtr->packetInfo, 14, 14, returnPushPolicy);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the return push policy for the packet from host or monolithic descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      TAIL - Descriptor must be returned to tail of queue.
 *  @retval
 *      HEAD - Descriptor must be returned to head of queue.
 */
static inline Qmss_Location Cppi_getReturnPushPolicy (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;

    return ((Qmss_Location) CSL_FEXTR (hostDescPtr->packetInfo, 14, 14));
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set the return queue manager and queue number in 
 *      host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  queue
 *      Queue Manager - 0 or 1.
 *      Queue Number - 0 to 4094 with in queue manager 0 or 1.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 2 (bit 14) of host or monolithic descriptor is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setReturnQueue (Cppi_DescType descType, Cppi_Desc *descAddr, Qmss_Queue queue)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (descPtr->packetInfo, 13, 12, queue.qMgr);
    CSL_FINSR (descPtr->packetInfo, 11, 0, queue.qNum);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the return queue manager and queue number from host 
 *      or monolithic descriptor.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      Queue Manager - 0 or 1.
 *  @retval
 *      Queue Number - 0 to 4094 with in queue manager 0 or 1.
 */
static inline Qmss_Queue Cppi_getReturnQueue (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;
    Qmss_Queue      queue;

    queue.qMgr = CSL_FEXTR (descPtr->packetInfo, 13, 12);
    queue.qNum = CSL_FEXTR (descPtr->packetInfo, 11, 0);
    return (queue);
}

/**
 *  @b Description
 *  @n  
 *      This function is used to set source and destination tags in 
 *      host or monolithic descriptor. 
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *
 *  @param[in]  tag
 *      Destination and source low and High tag value.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @post  
 *      Word 1 of host or monolithic descriptor is updated.
 *
 *  @retval
 *      None
 */
static inline void Cppi_setTag (Cppi_DescType descType, Cppi_Desc *descAddr, Cppi_DescTag *tag)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;

    CSL_FINSR (descPtr->tagInfo, 7, 0, tag->destTagLo);
    CSL_FINSR (descPtr->tagInfo, 15, 8, tag->destTagHi);
    CSL_FINSR (descPtr->tagInfo, 23, 16, tag->srcTagLo);
    CSL_FINSR (descPtr->tagInfo, 31, 24, tag->srcTagHi);
    return;
}

/**
 *  @b Description
 *  @n  
 *      This function is used to get the source and destination tags from 
 *      host or monolithic descriptor. 
 *      
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *      
 *  @pre  
 *      Descriptor must be allocated and be valid.
 *
 *  @retval
 *      8 bit destination low tag value.
 *  @retval
 *      8 bit destination high tag value.
 *  @retval
 *      8 bit source low tag value.
 *  @retval
 *      8 bit source high tag value.
 */
static inline Cppi_DescTag Cppi_getTag (Cppi_DescType descType, Cppi_Desc *descAddr)
{
    Cppi_HostDesc   *descPtr = (Cppi_HostDesc *) descAddr;
    Cppi_DescTag    tag;

    tag.destTagLo = CSL_FEXTR (descPtr->tagInfo, 7, 0);
    tag.destTagHi = CSL_FEXTR (descPtr->tagInfo, 15, 8);
    tag.srcTagLo = CSL_FEXTR (descPtr->tagInfo, 23, 16);
    tag.srcTagHi = CSL_FEXTR (descPtr->tagInfo, 31, 24);

    return (tag);
}

/**
@}
*/

extern Qmss_QueueHnd Cppi_initDescriptor (Cppi_DescCfg *descCfg, uint32_t *numAllocated);
extern Qmss_QueueHnd Cppi_initDescriptorSubSys (Qmss_SubSysHnd subSysHnd, Cppi_DescCfg *descCfg, uint32_t *numAllocated);

/* 
 * Restore remark state for: remark #880-D: parameter "descType" was never referenced
 *
 * This allows the remark to still come out in user code
 */
#ifdef _TMS320C6X
#pragma diag_default 880
#pragma diag_default 681
#elif defined(__GNUC__)
/* Same for GCC
 */
#pragma GCC diagnostic pop
#endif

#ifdef __cplusplus
}
#endif

#endif /* CPPI_DESC_H_ */

