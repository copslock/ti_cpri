/*
 *  Copyright (c) Texas Instruments Incorporated 2019
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
 */


/**
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_DMAUTILS_MODULE  DMA Utils API
 *            This is DMA Utils related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file dmautils_autoincrement_3d.h
 *
 *  \brief This file contains the data types and util function prototype for
 *              configuring DMA Utility autoincrement 3D usecase.
 *
 *  Typical sequence of operation :
 *      DmaUtilsAutoInc3d_getContextSize(numChannels) : Request memory required based on the number of
 *                                                                  channels required for a usecase
 *     DmaUtilsAutoInc3d_init : To allocate hardware resources required for for a usecase ( Need to be called only
 *                                             once per frame
 *     DmaUtilsAutoInc3d_getTrMemReq : Request memory required to prepare TR/TR descriptor based on the
 *                                             number of transfer records that needs to be submitted on a particular channel
 *    DmaUtilsAutoInc3d_prepareTr : Populate the tr based on the actual transfer property. This can be computed
 *                                              upfront to avoid cycles consumed to prepare it
 *    DmaUtilsAutoInc3d_configure : Configure a a particular channel by submitting the TR memory prepared earlier.
 *                                               This can be called multiple times within a frame
 *    DmaUtilsAutoInc3d_trigger : Trigger a particular channel
 *    DmaUtilsAutoInc3d_wait : wait for transfer completion of  particular channel
 *
 *    Trigger and wait can be called multiple times till we finish requrested number of transfers
 *
 *    DmaUtilsAutoInc3d_deconfigure :
 *    DmaUtilsAutoInc3d_deint :
 *
 *  Requirement: DOX_REQ_TAG(PDK-2644), DOX_REQ_TAG(PDK-2643), DOX_REQ_TAG(PDK-2642),
 *                         DOX_REQ_TAG(PDK-2645), DOX_REQ_TAG(PDK-2646), DOX_REQ_TAG(PDK-2650),
 *                         DOX_REQ_TAG(PDK-2652), DOX_REQ_TAG(PDK-3241)
 */


#ifndef DMAUTILS_AUTOINCREMENT_3D_H_
#define DMAUTILS_AUTOINCREMENT_3D_H_

#include <stdint.h>
#include <stdarg.h>
#include "ti/drv/udma/udma.h"

#ifdef __cplusplus
extern "C" {
#endif



//:TODO: Actual value on SOC is 3 but currently VLAB supports only on
/** \brief Number of TR's that can be submitted back to back channel  */
#define DMAUTILS_MAX_NUM_TR_DIRECT_TR_MODE (1U)


/**
 *  @enum   DmaUtilsAutoInc3d_SyncType
 *
 *  @brief   Describes the boundary at which the DMA is suppose to get
 *          synced
 *
 */
typedef enum{
  DMAUTILSAUTOINC3D_SYNC_1D = 0,/*!< DMA is synced at each 1D transfer i.e. it is synced
                                                                            whenever icnt1 is decremented by 1 */
  DMAUTILSAUTOINC3D_SYNC_2D = 1,/*!< DMA is synced at each 2D transfer i.e. it is synced
                                                                            whenever icnt2 is decremented by 1 */
  DMAUTILSAUTOINC3D_SYNC_3D = 2,/*!< DMA is synced at each 3D transfer i.e. it is synced
                                                                            whenever icnt3 is decremented by 1 */
  DMAUTILSAUTOINC3D_SYNC_4D = 3/*!< DMA is synced at each 4D transfer i.e. it is synced
                                                                            whenever one TR is completed */
}DmaUtilsAutoInc3d_SyncType;

/**
 *  @enum    DmaUtilsAutoInc3d_AddrType
 *
 *  @brief    Describes the boundary at which the DMA is suppose to get
 *          synced
 *
  */
typedef enum{
  DMAUTILSAUTOINC3D_ADDR_LINEAR = 0 , /*!< Linear addressing, addresses will be updated linearly as
                                                                                        per the dim's and icnt's*/
  DMAUTILSAUTOINC3D_ADDR_CIRC1 = 1,/*!< Circular addressing, address will hold the upper bits of the
                                                                                       addresses to be constant. This enum is to use circSize1 for
                                                                                       circularity */
  DMAUTILSAUTOINC3D_ADDR_CIRC2 = 2 /*!< Circular addressing, address will hold the upper bits of the
                                                                                       addresses to be constant. This enum is to use circSize2 for
                                                                                       circularity*/
}DmaUtilsAutoInc3d_AddrType;


/**
 *  @enum    DmaUtilsAutoInc3d_CircDirType
 *
 *  @brief    Describes the direction in which circular addressing is applied
 *
 */
typedef enum{
  DMAUTILSAUTOINC3D_CIRCDIR_SRC = 0,/*!< Circular addressing if enabled is applied on source side */
  DMAUTILSAUTOINC3D_CIRCDIR_DST = 1/*!<  Circular addressing if enabled is applied on desination side */
}DmaUtilsAutoInc3d_CircDirType;

/**
 *  @enum    DmaUtilsAutoInc3d_CircDirType
 *
 *  @brief    Describes the direction in which circular addressing is applied
 *
 */
typedef enum{
  DMAUTILSAUTOINC3D_DRUOWNER_DIRECT_TR = 0,/*!< DRU channel is used in Direct TR mode */
  DMAUTILSAUTOINC3D_DRUOWNER_UDMA= 1/*!<  Dru channel is used via ring based mechanism*/
}DmaUtilsAutoInc3d_druOwner;


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief This structure specifies the properties of the transfer for
 *          auto increment usecase.
 *
 */

typedef struct
{
    /** Total loop iteration count for level 0 (innermost) for source*/
    uint16_t    sicnt0;
    /**  Total loop iteration count for level 1 for source*/
    uint16_t    sicnt1;
     /** Total loop iteration count for level 2 for source*/
    uint16_t    sicnt2;
    /** Total loop iteration count for level 3  for source*/
    uint16_t    sicnt3;
    /** Jump in bytes when moving from from icnt0 to icnt1 for source*/
    int32_t     sdim1;
    /** Jump in bytes when moving from from icnt1 to icnt2 for source*/
    int32_t     sdim2;
    /** Jump in bytes when moving from from icnt2 to icnt3 for source*/
    int32_t     sdim3;
    /** Total loop iteration count for level 0 (innermost) for destination*/
    uint16_t    dicnt0;
    /** Total loop iteration count for level 1 for destination*/
    uint16_t    dicnt1;
    /** Total loop iteration count for level 2 for destination*/
    uint16_t    dicnt2;
    /** Total loop iteration count for level 3  for destination*/
    uint16_t    dicnt3;
    /** Jump in bytes when moving from from icnt0 to icnt1 for destination*/
    int32_t     ddim1;
    /** Jump in bytes when moving from from icnt1 to icnt2 for destination*/
    int32_t     ddim2;
    /** Jump in bytes when moving from from icnt2 to icnt3 for destination*/
    int32_t     ddim3;
}DmaUtilsAutoInc3d_TransferDim;


/**
 *
 *  \brief   This structure specifies the circular properties for each level of
 *          the transfer count. User can specify 2 different sizes for each level
 *
 */

typedef struct
{
  /** Size in Bytes of the first circular buffer. The minimum value is
        512 and it should be a power of 2. Maximum value can be 16M */
  uint64_t    circSize1;
  /** Size in Bytes of the first circular buffer. The minimum value is
      512 and it should be a power of 2. Maximum value can be 16M */
  uint64_t    circSize2;
  /**  Which addressing to be used for icnt0. Please refer DmaUtilsAutoInc3d_AddrType
          for the valid values*/
  uint8_t     addrModeIcnt0;
    /**  Which addressing to be used for icnt1. Please refer DmaUtilsAutoInc3d_AddrType
          for the valid values*/
  uint8_t     addrModeIcnt1;
  /**  Which addressing to be used for icnt2. Please refer DmaUtilsAutoInc3d_AddrType
      for the valid values*/
  uint8_t     addrModeIcnt2;
  /**  Which addressing to be used for icnt3. Please refer DmaUtilsAutoInc3d_AddrType
        for the valid values*/
  uint8_t     addrModeIcnt3;
  /**  Direction in which circular addressing needs to be applied, i.e. whether to apply
         circular addressing on source side or destination side. Please refer
         DmaUtilsAutoInc3d_CircDirType for the valid values. Circular addressing can only
             be applied to one side (src/dst) of transfer other side will use linear addressing*/
  uint8_t     circDir;
}DmaUtilsAutoInc3d_TransferCirc;

/**
 *  \brief   This structure specifies the input and output pointers for the
 *          transfer
 */
typedef struct
{
  /** Pointer to memory buffer for the source */
  uint8_t    *srcPtr;
  /** Pointer to memory buffer for the destination */
  uint8_t    *dstPtr;
}DmaUtilsAutoInc3d_IOPointers;


/**
 *  \brief   This structure specifies the properties of the transfer for
 *          auto increment usecase.
 *
 *  ===============================================================
 */
typedef struct
{
    /** Trasnfer sync boundary, Refer DmaUtilsAutoInc3d_SyncType for valid values */
    int32_t syncType;
    /** Properties to describe transfer dimensions in terms of icnt's
          and dim's */
    DmaUtilsAutoInc3d_TransferDim transferDim;
    /** Properties describing circularity */
    DmaUtilsAutoInc3d_TransferCirc circProp;
    /** Input and output pointers */
    DmaUtilsAutoInc3d_IOPointers ioPointers;
}DmaUtilsAutoInc3d_TransferProp;

/** ========================================================
 *
 *  \brief   This structure specifies the parameter to initialize
 *          auto increment related properties.
 *
 *  ===============================================================
 */

typedef struct
{
    /**  DRU queue number to which a particular channel should submit its
         transfers */
    uint8_t dmaQueNo;
    /**  Owner of the DRU. Refer DmaUtilsAutoInc3d_druOwner for valid values*/
    uint8_t druOwner;

}DmaUtilsAutoInc3d_ChannelInitParam;

typedef struct
{
    /**  DRU queue number to which a particular channel should submit its
         transfers */
    int32_t numChannels;
    /**  Owner of the DRU. Refer DmaUtilsAutoInc3d_druOwner for valid values*/
    int32_t contextSize;
    /** Level for debug messages */
    int32_t traceLogLevel;
    /** Call back Function pointer to Write Log*/
    int32_t(*DmaUtilsVprintf)(const char * format, va_list arg);
    /** Handle to the UDMA driver to be used for the utility. If user sets
    it to NULL then utility will use a default udma driver handle */
    Udma_DrvHandle                    udmaDrvHandle;
}DmaUtilsAutoInc3d_InitParam;


typedef struct
{
    /**  Allocated memory for TR descriptor, Use DmaUtilsAutoInc3d_getTrMemReq AP
    to know the actual size required for this memory*/
    uint8_t * trMem;
    /**  Size of the memory allocated for trMem*/
    int32_t trMemSize;
    /** Number of Transfer Records (TR's) that will be submitted to a particular channel */
    int32_t numTRs;
    /** channelId for which TR is prepared It is important to note
    that this id is a virtual id for channel and is not related to the actual channel
    used internally to do the transfer  */
    int32_t channelId;
}DmaUtilsAutoInc3d_TrPrepareParam;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Returns the size of dmautils context.
 *
 *  This function returns the size of the DMA UTILS autoincrement 3D handle. User is
 *  then expected to allocate this memory and provide it to DMA UTILS autoincrement 3D
 *  function as an input.
 *
 *  \param numChannels [IN] Number of channels required for a usecase
 *
 *  \return  Size of the DMA UTILS context in bytes
 */

int32_t DmaUtilsAutoInc3d_getContextSize(int32_t numChannels);

/**
 *  \brief     This function initializes the DMA UTILS context for autoincrement usecase.
 *
 *  This function initializes the DMA UTILS context for autoincrement usecase. Internally it allocates
 *   numChannels DRU channels for the use case
 *
 *  \param    autoIncrementContext [IN] Memory allocated by the user as per DmaUtilsAutoInc3d_getContextSize
 *                                                      API. Try to allocate this memory in the fastest available memory for optimal performance
 *
 *
 *  \param    initParams [IN] Init params for the dmautils
 *
 *  \param    chInitParams [IN] Init parameter for each channel. This is an array and number of enteries should be same as numChannels
 *
 *
 *  \return \ref Udma_ErrorCodes
 *
 *  =======================================================
 */
 int32_t DmaUtilsAutoInc3d_init(void * autoIncrementContext , DmaUtilsAutoInc3d_InitParam * initParams, DmaUtilsAutoInc3d_ChannelInitParam chInitParams[]);


/**
 *
 *  \brief     This function returns the size of the TR descriptor required for the transfer configuration
 *            given by the user.
 *
 * \param numTRs [IN] Number of Transfer Records (TR's) that will be submitted to a particular channel
 *
 *  \return   Size of the TR descriptor in bytes.
 *
 *  =======================================================
 */
 int32_t DmaUtilsAutoInc3d_getTrMemReq(int32_t numTRs);

/**
 *
 *  \brief     This function will prepare a TR/ TR descriptor depending on the number of Tr's to be submitted
 *                on a particular channel
 *
 *
 *  \param trPrepParam [IN] Parameters required for preparing TR
 *
 * \param transferProp [IN] Transfer properties of all the TR'sr which needs to be submitted to a single channel
 *
 *  \return \ref Udma_ErrorCodes
 *
 */
 int32_t DmaUtilsAutoInc3d_prepareTr(DmaUtilsAutoInc3d_TrPrepareParam* trPrepParam ,  DmaUtilsAutoInc3d_TransferProp transferProp[]);



/**
 *
 *  \brief   This function configures autoincrement for a particular channel
 *
 *  \param autoIncrementContext [IN] Context allocated for dmautils usecase
 *
 *  \param channelId  [IN] Channel Id which needs to be configured. It is important to note
 *                      that this id is a virtual id for channel and is not related to the actual channel
 *                      used internally to do the transfer
 *
 *  \param trMem [IN] Populated TrMem after calling DmaUtilsAutoInc3d_prepareTr function
 *
  * \param numTr [IN] Number of Transfer Records (TR's) that will be submitted to a particular channel
 *
 *  \return \ref Udma_ErrorCodes
 *
 */
 int32_t DmaUtilsAutoInc3d_configure(void * autoIncrementContext, int32_t channelId, uint8_t * trMem, int32_t numTr);


/**
 *
 *  \brief   This function triggers transfer on a particular channel
 *
 *  \param autoIncrementContext [IN] Context allocated for dmautils usecase
 *
 *  \param  channelId [IN] Channel Id which needs to be triggered. It is important to note
 *                      that this id is a virtual id for channel and is not related to the actual channel
 *                      used internally to do the transfer
 *
 *  \return Number of triggered required to complete the full transfer
 *
 */
int32_t DmaUtilsAutoInc3d_trigger(void * autoIncrementContext, int32_t channelId);

/**
 *
 *  \brief   This function waits for completion transfer on a particular channel
 *
 *  \param autoIncrementContext [IN] Context allocated for dmautils usecase
 *
 *  \param channelId  [IN] Channel Id for which we need to wait for transfer completion. It is important to note
 *                      that this id is a virtual id for channel and is not related to the actual channel
 *                      used internally to do the transfer
 *
 *  \return Number of triggered required to complete the full transfer
 *
 */
void  DmaUtilsAutoInc3d_wait(void * autoIncrementContext, int32_t channelId);

/**
 *
 *  \brief   This function deconfigures autoincrement for a particular channel
 *
 *  \param autoIncrementContext [IN] Context allocated for dmautils usecase
 *
 *  \param channelId [IN] Channel Id which needs to be de-configured. It is important to note
 *                      that this id is a virtual id for channel and is not related to the actual channel
 *                      used internally to do the transfer
 *
 *  \param trMem [IN] Populated TrMem after calling DmaUtilsAutoInc3d_prepareTr function
 *
  * \param numTr [IN] Number of Transfer Records (TR's) that will be submitted to a particular channel
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t DmaUtilsAutoInc3d_deconfigure(void * autoIncrementContext, int32_t channelId, uint8_t * trMem, int32_t numTr);

/**
 *
 *  \brief   This function deinit autoincrement by releasing all the resorces allocated
 *
 *  \param autoIncrementContext [IN] Context allocated for dmautils usecase
 *
 *  \return   \ref  Udma_ErrorCodes
 *
 */
int32_t DmaUtilsAutoInc3d_deinit(void * autoIncrementContext);


#ifdef __cplusplus
}
#endif

/* @} */

#endif /*#define DMAUTILS_AUTOINCREMENT_3D_H_*/

