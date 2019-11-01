/********************************************************************
* Copyright (C) 2012-2013 Texas Instruments Incorporated.
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
*/

/**
 *  @defgroup DFE_FL_CPP_API CPP
 *  @ingroup DFE_FL_API
 */
 
/** @file dfe_fl_cpp.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_CPP CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
 
/**
 * @defgroup DFE_FL_CPP_SYMBOL DFE Cpp Symbols
 * @ingroup DFE_FL_CPP_API
 */

/**
 * @defgroup DFE_FL_CPP_DATASTRUCT DFE Cpp Data Structures
 * @ingroup DFE_FL_CPP_API
 */

/**
 * @defgroup DFE_FL_CPP_ENUM DFE Cpp Enumverated Data Types
 * @ingroup DFE_FL_CPP_API
 */

/**
 * @defgroup DFE_FL_CPP_FUNCTION DFE Cpp Functions
 * @ingroup DFE_FL_CPP_API
 */

#ifndef _DFE_FL_CPP_H_
#define _DFE_FL_CPP_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/drv/dfe/dfe_fl_misc.h>
#include <ti/drv/dfe/dfe_fl_cppParams.h>


/**
 * @addtogroup DFE_FL_CPP_SYMBOL
 * @{
 */
/** special value to indicate open any un-reserved and available resources, such as  
 *      dma, descriptor, discrete trigger
 */
/// open any available CPP/DMA channel/descriptor/trigger
#define DFE_FL_CPP_OPEN_ANY                0xffffffffu
/// there's no resource allocated yet
#define DFE_FL_CPP_OPEN_NONE               0xfffffffeu

/** macro from dsp address to dfe address */
#define DFE_FL_CPP_ADDR_DSP2DFE(dspAddr)   ((uint32_t)((dspAddr) & 0x3FFFFFFu))
/// total number of CPP/DMA trigger destinations
#define DFE_FL_CPP_NUM_DISCRETE_TRIGGERS   8

/** CPP/DMA ssel */
/// one of SynGen sync
#define DFE_FL_CPP_DMA_SSEL_GSYNC(n)               (n)
/// data available on the DL CTL channel
#define DFE_FL_CPP_DMA_SSEL_DL_CTL_DATA_AVAIL(n)   (16+n)
/// one of CPP/DMA alternative sync
#define DFE_FL_CPP_DMA_SSEL_ALT_SYNC(n)            (32+n)
/// timer14 of CPP/DMA
#define DFE_FL_CPP_DMA_SSEL_CTL_TIMER14            (46)
/// timer15 of CPP/DMA
#define DFE_FL_CPP_DMA_SSEL_CTL_TIMER15            (47)
/// MANUAL firing
#define DFE_FL_CPP_DMA_SSEL_MANUAL                 (255)

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CPP_ENUM
 * @{
 */

/** per xfer MPU increment in bytes */
typedef enum
{
    /// MPU address increment of 4 bytes 
    DFE_FL_CPP_MPU_INC_SIZE_4 = 0,
    /// MPU address increment of 8 bytes 
    DFE_FL_CPP_MPU_INC_SIZE_8,
    /// MPU address increment of 16 bytes 
    DFE_FL_CPP_MPU_INC_SIZE_16,
    /// MPU address increment of 32 bytes 
    DFE_FL_CPP_MPU_INC_SIZE_32
} DfeFl_CppMpuIncSize;

/** per xfer CTL increment in bytes */
typedef enum
{
    /// CTL address increment of 2 bytes 
    DFE_FL_CPP_CTL_INC_SIZE_2 = 0,
    /// CTL address increment of 4 bytes 
    DFE_FL_CPP_CTL_INC_SIZE_4,
    /// CTL address increment of 8 bytes 
    DFE_FL_CPP_CTL_INC_SIZE_8,
    /// CTL address increment of 16 bytes 
    DFE_FL_CPP_CTL_INC_SIZE_16
} DfeFl_CppCtlIncSize;

/** CPP/DMA packet size */
typedef enum
{
    /// CPP/DMA packet size: 64K bytes
    DFE_FL_CPP_DMA_PKT_SIZE_64K = 0,
    /// CPP/DMA packet size: 32K bytes
    DFE_FL_CPP_DMA_PKT_SIZE_32K,
    /// CPP/DMA packet size: 16K bytes
    DFE_FL_CPP_DMA_PKT_SIZE_16K,
    /// CPP/DMA packet size: 8K bytes
    DFE_FL_CPP_DMA_PKT_SIZE_8K,
    /// CPP/DMA packet size: 4K bytes
    DFE_FL_CPP_DMA_PKT_SIZE_4K,
    /// CPP/DMA packet size: 2K bytes
    DFE_FL_CPP_DMA_PKT_SIZE_2K,
    /// CPP/DMA packet size: 1K bytes
    DFE_FL_CPP_DMA_PKT_SIZE_1K,
    /// CPP/DMA packet size: 512 bytes
    DFE_FL_CPP_DMA_PKT_SIZE_512
} DfeFl_CppDmaPktSize;

/** CPP/DMA transfer direction, DL or UL */
typedef enum
{
    /// Downlink, from IQN2 to DFE
    DFE_FL_CPP_DMA_DL = 0,
    /// Downlink, from DFE to IQN2
    DFE_FL_CPP_DMA_UL = 1
} DfeFl_CppDmaDir;

/** CPP/DMA channel mode */
typedef enum
{
    /// CPP/DMA channel is disabled
    DFE_FL_CPP_DMA_MODE_DISABLE = 0,
    /// CPP/DMA channel is using PROG mode
    DFE_FL_CPP_DMA_MODE_PROG,
    /// CPP/DMA channel is using EMBED mode
    DFE_FL_CPP_DMA_MODE_EMBED,
    
    DFE_FL_CPP_DMA_MODE_MAX
} DfeFl_CppDmaMode;

/** CPP/DMA Alternative Sync Signals */
typedef enum
{
    /// Poly2Lut status send
    DFE_FL_CPP_DMA_SSEL_ALT_POLY2LUT_STATUS_SEND = 0,
    /// Poly2Lut coefficients send
    DFE_FL_CPP_DMA_SSEL_ALT_POLY2LUT_COEFF_SEND,
    /// Poly2Lut coefficients read
    DFE_FL_CPP_DMA_SSEL_ALT_POLY2LUT_COEFF_READ,
    /// Arbiter Log Event
    DFE_FL_CPP_DMA_SSEL_ALT_ARB_LOG,

    /// not used 4
    DFE_FL_CPP_DMA_SSEL_ALT_NA_4,
    /// Completion of coarse capture
    DFE_FL_CPP_DMA_SSEL_ALT_CBC_DONE,

    /// Complete of BBRX power meters        
    DFE_FL_CPP_DMA_SSEL_ALT_BBRX_PWRMTR,
    /// Complete of BBTX power meters        
    DFE_FL_CPP_DMA_SSEL_ALT_BBTX_PWRMTR,
    
    /// not used 8
    DFE_FL_CPP_DMA_SSEL_ALT_NA_8,
    /// not used 9
    DFE_FL_CPP_DMA_SSEL_ALT_NA_9,
    /// not used 10
    DFE_FL_CPP_DMA_SSEL_ALT_NA_10,
    /// not used 11
    DFE_FL_CPP_DMA_SSEL_ALT_NA_11
} DfeFl_CppDmaSselAlt;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CPP_DATASTRUCT
 * @{
 */

/** descriptor config */
typedef struct
{
#ifdef _BIG_ENDIAN
    /// next descriptor# 
    uint32_t linkNext : 8;
    /// UL: mid pkt; DL: immediate xfer
    uint32_t midImm   : 1;
    /// packet size
    uint32_t pktSize  : 3;
    /// MPU address increment
    uint32_t mpuIncr  : 2;
    /// CTL data word increment
    uint32_t ctlIncr  : 2;
    /// size of data block 
    uint32_t numBytes : 16;

    /// DMA direction
    ///  1 = read from DFE
    ///  0 = write to DFE
    uint32_t rw       : 1;
    /// DL/UL IQN CTL channel number
    uint32_t chanNum  : 5;
    /// 26-bit address within DFE scope
    uint32_t mpuAddr  : 26;

#else
    /// size of data block 
    uint32_t numBytes : 16;
    /// CTL data word increment
    uint32_t ctlIncr  : 2;
    /// MPU address increment
    uint32_t mpuIncr  : 2;
    /// packet size
    uint32_t pktSize  : 3;
    /// UL: mid pkt; DL: immediate xfer
    uint32_t midImm   : 1;
    /// next descriptor# 
    uint32_t linkNext : 8;

    /// 26-bit address within DFE scope
    uint32_t mpuAddr  : 26;
    /// DL/UL IQN CTL channel number
    uint32_t chanNum  : 5;
    /// DMA direction
    ///  1 = read from DFE
    ///  0 = write to DFE
    uint32_t rw       : 1;
#endif    
} DfeFl_CppDescripConfig;

/* 128-bits CPP Embedded header */
typedef struct
{
    /// CPP/DMA descriptor configuration
    DfeFl_CppDescripConfig descripCfg;
    /// first data word
    uint32_t                  data0;
    /// second data word
    uint32_t                  data1;
} DfeFl_CppEmbedHeader;

struct DfeFl_CppResMgr;

/** CPP/DMA channel object */
typedef struct
{
    /// DMA channel ID, valid range 0 ~ 31
    uint32_t id;
    
    /// mode
    DfeFl_CppDmaMode   mode;
    
    /// trigger destination selection
    uint32_t iTrig;
    
    /// Misc instance
    DfeFl_MiscHandle   hMisc;

    /// resource manager
    struct DfeFl_CppResMgr *resMgr;
} DfeFl_CppDmaObj;
/// CPP/DMA channel handle
typedef DfeFl_CppDmaObj *DfeFl_CppDmaHandle;

/** CPP/DMA descriptor object */
typedef struct
{
    /// descriptor Id, valid range 0 ~ 127
    uint32_t id;
    
    /// Misc instance
    DfeFl_MiscHandle   hMisc;
    
    /// resource manager
    struct DfeFl_CppResMgr *resMgr;
} DfeFl_CppDescriptorObj;
/// CPP/DMA descriptor handle
typedef DfeFl_CppDescriptorObj *DfeFl_CppDescriptorHandle;

/** CPP/DMA resource manager */
typedef struct DfeFl_CppResMgr
{
    /// reserved channel bitmask
    uint32_t dmaRsvd;
    /// opened channel bitmask
    uint32_t dmaOpened;
    
    /// discrete trigger out
    uint32_t discreteTrig[DFE_FL_CPP_NUM_DISCRETE_TRIGGERS];
    
    /// four 32-bits words, each bit corresponding to one descriptor
    /// reserved descriptor bitmask
    uint32_t descripRsvd[4];
    /// opened descriptor bitmask
    uint32_t descripOpened[4];
    
    /// dma channel objects table
    DfeFl_CppDmaObj dmaTbl[DFE_FL_CPP_NUM_DMA];
    
    /// descriptor objects table
    DfeFl_CppDescriptorObj descripTbl[DFE_FL_CPP_NUM_DESCRIPTORS];
    
} DfeFl_CppResMgr;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CPP_FUNCTION
 * @{
 */



// open a CPP dma channel
DfeFl_CppDmaHandle dfeFl_CppDmaOpen
(
    DfeFl_MiscHandle   hMisc,
    uint32_t              dmaId, 
    DfeFl_CppDmaMode   mode, 
    uint32_t              iTrig,
    DfeFl_CppResMgr    *resMgr,
    DfeFl_Status          *status
);

// close prior opened dma
void dfeFl_CppDmaClose
(
    DfeFl_CppDmaHandle hDma
);

/*  start a dma via sync,
 *  for manual mode, dmaSsel ignored
 */
DfeFl_Status dfeFl_CppDmaArm
(
    DfeFl_CppDmaHandle hDma,
    uint32_t dmaStartAddr,
    uint32_t dmaSsel
);

// Dismiss DMA sync
void dfeFl_CppDmaDismissSync
(
    DfeFl_CppDmaHandle hDma
);

// manual abort a dma
void dfeFl_CppDmaAbort
(
    DfeFl_CppDmaHandle hDma
);

// Get if CPP/DMA channel busy transferring
uint32_t dfeFl_CppDmaGetBusy
(
    DfeFl_CppDmaHandle hDma
);

// return current dma Id
uint32_t dfeFl_CppDmaGetId
(
    DfeFl_CppDmaHandle hDma
);

// open a CPP dma decriptor
DfeFl_CppDescriptorHandle dfeFl_CppDecripOpen
(
    DfeFl_MiscHandle   hMisc,
    uint32_t              descripId,
    DfeFl_CppResMgr    *resMgr,
    DfeFl_Status          *status
);

// close prior opened descriptor
void dfeFl_CppDescripClose
(
    DfeFl_CppDescriptorHandle hDescrip
);

// return current id
uint32_t dfeFl_CppDescripGetId(DfeFl_CppDescriptorHandle hDescrip);

// read a descriptor configure
void dfeFl_CppDescripRead
(
    DfeFl_CppDescriptorHandle hDescrip,
    DfeFl_CppDescripConfig *descripConfig
);
  
// write a descriptor configure
void dfeFl_CppDescripWrite
(
    DfeFl_CppDescriptorHandle hDescrip,
    DfeFl_CppDescripConfig *descripConfig
);

// link two descriptors together, hDescrip1->linkNext = hDescrip2
void dfeFl_CppDescripLink
(
    DfeFl_CppDescriptorHandle hDescrip1,
    DfeFl_CppDescriptorHandle hDescrip2
);

/**
 * @}
 */
 
#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_CPP_H_ */
