/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 *  \file cal_common.h
 *
 *  \brief CAL header file containing commonly used functions.
 *
 */

#ifndef CAL_COMMON_H_
#define CAL_COMMON_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/soc.h>

#include <ti/drv/cal/cal.h>

#include <ti/drv/cal/src/hal/cal_hal.h>
#include <ti/drv/cal/src/core/cal_core.h>
#include <ti/drv/cal/src/core/cal_evtmgr.h>
#include <ti/drv/cal/src/core/cal_resrcMgr.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief CAL Lib interrupt initialization parameters.
 */
typedef struct
{
    uint32_t calIrqNum;
    /**< CAL interrupt number. */
} CalLib_IrqParams;
/**
 *  \brief CAL Lib initialization parameters.
 */
typedef struct
{
    uint32_t isAddrTransReq;
    /**< Set this flag to TRUE if the driver has to perform address translation
     *   of the descriptor memory before submitting the descriptor to the
     *   hardware. This is used when the physical memory of the descriptor
     *   is mapped to a different virtual memory.
     *
     *   When address translation is enabled, the dirver performs the following
     *   operations to convert the virtual address to physical address and
     *   vice versa.
     *
     *   physAddr = (virtAddr - virtBaseAddr) + physBaseAddr;
     *   virtAddr = (physAddr - physBaseAddr) + virtBaseAddr;
     *
     *   Important: The descriptor memory should in a physically continuous
     *   memory.
     *
     *   Note: The buffer address will not be translated using the above
     *   translation and hence the application should provide the physical
     *   address to be programmed to the hardware.
     */
    uint32_t virtBaseAddr;
    /**< Virtual memory base address. */
    uint32_t physBaseAddr;
    /**< Physical memory base address. */
    uint32_t isCacheOpsReq;
    /**< This will enable cache flush and invalidate operations on the
     *   descriptor memory in case the descriptor memory is cache region.
     *
     *   Note: This is not supported in the current implementation and is meant
     *   for future use. */
    uint32_t isCacheFlushReq;
    /**< This will enable cache flush operations on the
     *   descriptor memory in case the descriptor memory is cache region.
     *   In case of write-through cache mode, this flag could be set to FALSE
     *   to save cycles as flush operation is not required in write-through
     *   mode.
     *   This parameter is valid/used only if isCacheOpsReq is TRUE.
     *
     *   Note: This is not supported in the current implementation and is meant
     *   for future use. */
    CalLib_IrqParams irqParams;
    /**< This will enable to configure different interrupt numnbers for
     *   different CAL modules. This will be initialized to default values as
     *   specified in the CalInitParams_init function. User may override these
     *   values.
     *
     *   Note: Driver will only register for the specified interrupt numbers.
     *   The corresponding crossbar mapping for the device interrupt should be
     *   done by the application.
     */
} CalLib_InitParams;

/**
 *  \brief Platform specific data containing base address information of
 *  various modules.
 */
typedef struct
{
    const void                **calHal;
    /**< CAL Hal information */
    uint32_t                     numCalEmInst;
    /**< Number CAL Event Manager instances */
    void                      *calEmInitPrms;
    /**< Cal Event Manager Init Params */
} CalLib_PlatformData;

/**
 *  struct Cal_CoreSubFrameParams
 *  \brief Configuration for sub-frame level processing at create time used
 */
typedef struct
{
    uint32_t subFrameEnable;
    /**< TRUE : SubFrame level capture/processing is enabled.
     *   FALSE: SubFrame level capture/processing is disabled.
     *   Must be FALSE for multi-channel capture mode. */
    uint32_t numLinesPerSubFrame;
    /**< Number of lines per subframes.
     *
     *   MUST be multiple of the output size.
     *   Not valid, ignored for ancillary data capture.
     *
     *   In case of capture,
     *   SubFrame callback gets called after every numLinesPerSubFrame
     *   for every output stream, except ancillary data stream.
     *
     *   Ignored when subFrameEnable = FALSE */
} Cal_CoreSubFrameParams;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief CAL Lib init function.
 *
 *  Initializes the CAL library.
 *  This function should be called before calling any of driver API's and
 *  should be called only once.
 *
 *  \param initPrms     [IN] CAL Initialization parameters.
 *                           If NULL is passed, the default parameters will be
 *                           assumed - address translation disabled.
 *
 *  \return FVID2_SOK on success else appropiate FVID2 error code on failure.
 */
int32_t CalLib_init(const CalLib_InitParams *initPrms);

/**
 *  \brief CAL Lib deinit function.
 *
 *  Uninitializes the CAL library and should be called during
 *  system shutdown. Should not be called if Cal_init() is not called.
 *
 *  \param args         [IN] Not used currently. Set to NULL.
 *
 *  \return FVID2_SOK on success, else appropriate FVID2 error code on failure.
 */
int32_t CalLib_deInit(void *args);

/**
 *  \brief Returns CAL Lib platform data.
 */
CalLib_PlatformData *CalLib_getPlatformData(void);

/**
 *  \brief CalLib_InitParams structure init function.
 *
 *  \param initPrms     [IN] Pointer to #CalLib_InitParams structure.
 *
 */
static inline void CalLibInitParams_init(CalLib_InitParams *initPrms);

Bool CalLib_platformIsTda3xxFamilyBuild(void);

int32_t CalLib_commonInit(const CalLib_PlatformData *platData);

int32_t CalLib_commonDeInit(void *arg);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void CalLibInitParams_init(CalLib_InitParams *initPrms)
{
    if (NULL != initPrms)
    {
        initPrms->isAddrTransReq  = (uint32_t) FALSE;
        initPrms->virtBaseAddr    = 0U;
        initPrms->physBaseAddr    = 0U;
        initPrms->isCacheOpsReq   = (uint32_t) FALSE;
        initPrms->isCacheFlushReq = (uint32_t) FALSE;
        memset(&initPrms->irqParams, 0, sizeof (CalLib_IrqParams));
        initPrms->irqParams.calIrqNum    = CSL_CAL_IRQ_NUM;
    }

    return;
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef CAL_COMMON_H_ */
