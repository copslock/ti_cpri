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
 * \file cal_resrcMgr.c
 *
 * \brief CAL Resource Manager Source file
 * This file exposes the APIs of the CAL Resource Manager.
 *
 *  \warning Following are the allocation policies followed.
 *      . CAL Resource allocation
 *          . All resources / instances are allocated starting from instance 0,
 *              exceptions are below.
 *          . DMA Write instance / channel allocation.
 *              0x0 is the least preferred channel, will allocate 1 to max,
 *              channel 0x0 would be allocated last
 *
 */

/* BUG TODO Sujith
 * Pixel Processing sub blocks cannot be allocated individually. Since there is
 *  1 cport ID for an instance of pix proc */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stddef.h>

#include <ti/drv/cal/cal.h>
#include <ti/drv/cal/src/core/cal_common.h>
#include <ti/drv/cal/src/core/cal_resrcMgr.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define CAL_MAX_WR_DMA_CHANNEL          (0x8U)
/**< Maximum Pixel Processing context */
#define CAL_MAX_CPORT_ID                (0x8U)
/**< Maximum Pixel Processing context */
#define CAL_MAX_NUM_CH_ON_A_PPI_INST    (0x4U)
/**< Maximum number of channel that can received on a given PPI */
#define CAL_MAX_CSI2_INSTANCE           (0x2U)
/**< Maximum number of csi2 phy instance present */
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  struct Cal_RmCalInstObj
 *  \brief CAL Instance object.
 */
typedef struct Cal_RmCalInstObj
{
    uint32_t instId;
    /**< CAL Instance identifier. */
    uint32_t numChOnPpi0;
    /**< Number of channels on PPI 0, max is CAL_MAX_NUM_CH_ON_A_PPI_INST */
    uint32_t numChOnPpi1;
    /**< Number of channels on PPI 1, max is CAL_MAX_NUM_CH_ON_A_PPI_INST */
    uint32_t pixExtract[CAL_CAPT_MAX_PIX_PROC_CONTEXT];
    /**< Pixel Extract */
    uint32_t dpmDecode[CAL_CAPT_MAX_PIX_PROC_CONTEXT];
    /**< DPM Decode */
    uint32_t dpmEncode[CAL_CAPT_MAX_PIX_PROC_CONTEXT];
    /**< DPM Encode */
    uint32_t pixPack[CAL_CAPT_MAX_PIX_PROC_CONTEXT];
    /**< Pixel Pack */
    uint32_t bysOut;
    /**< BYS Out. TRUE is allocated, FALSE otherwise */
    uint32_t bysIn;
    /**< BYS IN. TRUE is allocated, FALSE otherwise */
    uint32_t vPort;
    /**< VPORT IN. TRUE is allocated, FALSE otherwise */
    uint32_t rdDma;
    /**< RD Dma. TRUE is allocated, FALSE otherwise */
    uint32_t wrDma[CAL_MAX_WR_DMA_CHANNEL];
    /**< WR Dma */
    uint32_t cport[CAL_MAX_CPORT_ID];
    /**< CPort */
    uint32_t csi2Ctx[CAL_MAX_CSI2_INSTANCE][CAL_MAX_CPORT_ID];
    /**< CSI2 Processing conext */
    uint32_t lvds;
    /**< LVDS port */
    uint32_t cpi;
    /**< Parallel / CPI interface */
} Cal_RmCaptInstObj_t;

/**
 *  struct Cal_RmInstObj
 *  \brief Resource object.
 */
typedef struct
{
    uint32_t             isInited;
    /**< Flag to indicate if the its been initialized */
    uint32_t             numCalInst;
    /**< Valid number of CAL instances */
    Cal_RmCaptInstObj_t *resCal[CAL_RM_MAX_CAL_BLOCKS];
    /**< Capture Resources */
    SemaphoreP_Handle  lockSem;
    /**< Mutual exclusion */
} Cal_RmInstObj_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  allocCaptRes
 *  \brief Function to allocate resource from instannce of CAL.
 */
static int32_t allocCaptRes(Cal_RmCaptInstObj_t *pAvailRes, uint32_t reqRes,
                          Cal_CaptBlocks_t *pAllocRes, uint32_t flags);

/**
 *  releaseCaptRes
 *  \brief Function to allocate resource from instannce of CAL.
 */
static int32_t releaseCaptRes(Cal_RmCaptInstObj_t           *pAvailRes,
                            const Cal_CaptBlocks_t *pAllocRes);
/* ========================================================================== */
/*                        Global Variables                                    */
/* ========================================================================== */

/** \brief Resource object. */
/* Require to initialize isInited flag alone Not much worried about others */
static Cal_RmInstObj_t gCalRmInstObj = {FALSE, 0x0, {NULL, NULL}, NULL};

/** \brief CAL Resource object. */
Cal_RmCaptInstObj_t    gCalRmCalObjs[CAL_RM_MAX_CAL_BLOCKS];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  Cal_rmInit
 *  \brief Function to initialize Resource manager.
 */
int32_t Cal_rmInit(uint32_t numInst, const Cal_RmInitParams_t *instPrms)
{
    uint32_t i;
    int32_t  retVal = FVID2_EBADARGS;
    SemaphoreP_Params params;

    /* Args checks */
    if (NULL != instPrms)
    {
        if (instPrms->numCalBlocks <= CAL_RM_MAX_CAL_BLOCKS)
        {
            retVal = FVID2_SOK;
        }
    }

    if ((FALSE == gCalRmInstObj.isInited) && (FVID2_SOK == retVal))
    {
        /* Init objects and mark flags as free */
        memset((void *) &gCalRmInstObj, 0U, sizeof (Cal_RmInstObj_t));
        for (i = 0; i < CAL_RM_MAX_CAL_BLOCKS; i++)
        {
            gCalRmInstObj.resCal[i] = &gCalRmCalObjs[i];
            /* Initialize all the resources */
            memset((void *) (&gCalRmCalObjs[i]), 0U,
                            sizeof (Cal_RmCaptInstObj_t));
            gCalRmCalObjs[i].instId = i;
        }

        /* Get the semaphore handle */
        SemaphoreP_Params_init(&params);
        params.mode = SemaphoreP_Mode_BINARY;
        gCalRmInstObj.lockSem = SemaphoreP_create(1U, &params);
        if (NULL == gCalRmInstObj.lockSem)
        {
            retVal = FVID2_EALLOC;
        }

        if (FVID2_SOK == retVal)
        {
            gCalRmInstObj.isInited = (uint32_t) TRUE;
        }
    }

    return (retVal);
}

/**
 *  Vrm_deInit
 *  \brief Function to de-initialize the resource manager.
 */
int32_t Cal_rmDeInit(void)
{
    int32_t  rtnVal = FVID2_SOK;
    Cal_RmCaptInstObj_t *pAvailRes;
    uint32_t i, j, k;

    if (TRUE == gCalRmInstObj.isInited)
    {
        for (i = 0; ((FVID2_SOK == rtnVal) && (i < CAL_RM_MAX_CAL_BLOCKS)); i++)
        {
            pAvailRes = gCalRmInstObj.resCal[i];
            if (pAvailRes != NULL)
            {
                if ((pAvailRes->numChOnPpi0 != 0x0U) ||
                    (pAvailRes->numChOnPpi1 != 0x0U))
                {
                    rtnVal = FVID2_EDEVICE_INUSE;
                }
                if (pAvailRes->bysOut != FALSE)
                {
                    rtnVal = FVID2_EDEVICE_INUSE;
                }
                else if ((pAvailRes->bysIn != FALSE) ||
                         (pAvailRes->vPort != FALSE))
                {
                    rtnVal = FVID2_EDEVICE_INUSE;
                }
                else if (pAvailRes->rdDma != FALSE)
                {
                    rtnVal = FVID2_EDEVICE_INUSE;
                }
                else
                {
                    /* Nothing to be done here */
                }

                if (rtnVal == FVID2_EDEVICE_INUSE)
                {
                    break;
                }

                for (j = 0; j < CAL_CAPT_MAX_PIX_PROC_CONTEXT; j++)
                {
                    if ((pAvailRes->pixExtract[j] != FALSE) ||
                        (pAvailRes->dpmDecode[j] != FALSE) ||
                        (pAvailRes->dpmEncode[j] != FALSE) ||
                        (pAvailRes->pixPack[j] != FALSE))
                    {
                        rtnVal = FVID2_EDEVICE_INUSE;
                        break;
                    }
                }

                for (j = 0; j < CAL_MAX_WR_DMA_CHANNEL; j++)
                {
                    if (pAvailRes->wrDma[j] != FALSE)
                    {
                        rtnVal = FVID2_EDEVICE_INUSE;
                        break;
                    }
                }
                for (j = 0; j < CAL_MAX_CPORT_ID; j++)
                {
                    if (pAvailRes->cport[j] != FALSE)
                    {
                        rtnVal = FVID2_EDEVICE_INUSE;
                        break;
                    }
                }
                for (j = 0; j < CAL_MAX_CPORT_ID; j++)
                {
                    for (k = 0; k < CAL_MAX_CSI2_INSTANCE; k++)
                    {
                        if (pAvailRes->csi2Ctx[k][j] != FALSE)
                        {
                            rtnVal = FVID2_EDEVICE_INUSE;
                            break;
                        }
                    }
                }
            }
            else
            {
                rtnVal = FVID2_EBADARGS;
            }
        }
        if (FVID2_SOK == rtnVal)
        {
            SemaphoreP_delete(gCalRmInstObj.lockSem);
            memset((void *) &gCalRmInstObj, 0, sizeof (Cal_RmInstObj_t));
        }
    }

    return (FVID2_SOK);
}

int32_t Cal_rmAllocResource(uint32_t instId, Cal_RmModules_t module,
                        uint32_t reqRes, void *allocRes, uint32_t flags)
{
    int32_t rtnVal = FVID2_EBADARGS;
    if ((TRUE == gCalRmInstObj.isInited) && (NULL != allocRes))
    {
        if ((CAL_RM_MODULE_CAL_A == module) || (CAL_RM_MODULE_CAL_B == module))
        {
            SemaphoreP_pend(gCalRmInstObj.lockSem, SemaphoreP_WAIT_FOREVER);
            rtnVal = allocCaptRes(gCalRmInstObj.resCal[module], reqRes,
                                  (Cal_CaptBlocks_t *) allocRes, flags);
            SemaphoreP_post(gCalRmInstObj.lockSem);
        }
    }
    return rtnVal;
}

int32_t Cal_rmReleaseResource(uint32_t instId, Cal_RmModules_t module,
                          void *relRes)
{
    int32_t rtnVal = FVID2_EBADARGS;
    if ((TRUE == gCalRmInstObj.isInited) && (NULL != relRes))
    {
        if ((CAL_RM_MODULE_CAL_A == module) || (CAL_RM_MODULE_CAL_B == module))
        {
            SemaphoreP_pend(gCalRmInstObj.lockSem, SemaphoreP_WAIT_FOREVER);
            rtnVal = releaseCaptRes(gCalRmInstObj.resCal[module],
                                    (Cal_CaptBlocks_t *) relRes);
            SemaphoreP_post(gCalRmInstObj.lockSem);
        }
    }
    return rtnVal;
}

/* ========================================================================== */
/*                        Local Function Definitions                          */
/* ========================================================================== */

static int32_t allocCaptRes(Cal_RmCaptInstObj_t *pAvailRes, uint32_t reqRes,
                          Cal_CaptBlocks_t *pAllocRes, uint32_t flags)
{
    uint32_t i;
    int32_t  rtnVal = FVID2_SOK;

    uint32_t csi2Instance = 0U;

    if ((NULL == pAllocRes) || (NULL == pAvailRes))
    {
        rtnVal = FVID2_EBADARGS;
    }
    else
    {
        memset((void *) pAllocRes, (uint8_t) 0xFF,
                        sizeof (Cal_CaptBlocks_t));
    }

    csi2Instance = 0U;

    if (CAL_CAPT_INST_ID_SUB_PPI_ID_1 ==
            (reqRes & CAL_CAPT_INST_ID_SUB_PPI_ID_1))
    {
        csi2Instance = 1U;
    }

    if (FVID2_SOK == rtnVal)
    {
        if (CAL_CAPT_INST_ID_SUB_PPI_ID_0 ==
            (reqRes & CAL_CAPT_INST_ID_SUB_PPI_ID_0))
        {
            if (CAL_MAX_NUM_CH_ON_A_PPI_INST > pAvailRes->numChOnPpi0)
            {
                reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_PPI_ID_0);
                pAvailRes->numChOnPpi0++;
                pAllocRes->ppi0Inst = 0x0U;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }
        if (CAL_CAPT_INST_ID_SUB_PPI_ID_1 ==
            (reqRes & CAL_CAPT_INST_ID_SUB_PPI_ID_1))
        {
            if (CAL_MAX_NUM_CH_ON_A_PPI_INST > pAvailRes->numChOnPpi1)
            {
                reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_PPI_ID_1);
                pAvailRes->numChOnPpi1++;
                pAllocRes->ppi1Inst = 0x1U;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID);
            rtnVal  = FVID2_EALLOC;
            for (i = 0; i < CAL_CAPT_MAX_PIX_PROC_CONTEXT; i++)
            {
                if (FALSE == pAvailRes->pixExtract[i])
                {
                    pAvailRes->pixExtract[i] = (uint32_t) TRUE;
                    pAllocRes->pixExtract    = i;
                    rtnVal = FVID2_SOK;
                    break;
                }
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID);
            rtnVal  = FVID2_EALLOC;
            for (i = 0; i < CAL_CAPT_MAX_PIX_PROC_CONTEXT; i++)
            {
                if (FALSE == pAvailRes->dpmDecode[i])
                {
                    pAvailRes->dpmDecode[i] = (uint32_t) TRUE;
                    pAllocRes->dpmDecode    = i;
                    rtnVal = FVID2_SOK;
                    break;
                }
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID);
            rtnVal  = FVID2_EALLOC;
            for (i = 0; i < CAL_CAPT_MAX_PIX_PROC_CONTEXT; i++)
            {
                if (FALSE == pAvailRes->dpmEncode[i])
                {
                    pAvailRes->dpmEncode[i] = (uint32_t) TRUE;
                    pAllocRes->dpmEncode    = i;
                    rtnVal = FVID2_SOK;
                    break;
                }
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_PIX_PACK_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_PIX_PACK_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_PIX_PACK_ID);
            rtnVal  = FVID2_EALLOC;
            for (i = 0; i < CAL_CAPT_MAX_PIX_PROC_CONTEXT; i++)
            {
                if (FALSE == pAvailRes->pixPack[i])
                {
                    pAvailRes->pixPack[i] = (uint32_t) TRUE;
                    pAllocRes->pixPack    = i;
                    rtnVal = FVID2_SOK;
                    break;
                }
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_BYS_OUT_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_BYS_OUT_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_BYS_OUT_ID);
            if (FALSE == pAvailRes->bysOut)
            {
                pAvailRes->bysOut = (uint32_t) TRUE;
                pAllocRes->bysOut = 0x0;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_BYS_IN_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_BYS_IN_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_BYS_IN_ID);
            if (FALSE == pAvailRes->bysIn)
            {
                pAvailRes->bysIn = (uint32_t) TRUE;
                pAllocRes->bysIn = 0x0;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_VPORT_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_VPORT_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_VPORT_ID);
            if (FALSE == pAvailRes->vPort)
            {
                pAvailRes->vPort = (uint32_t) TRUE;
                pAllocRes->vPort = 0x0;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_DMA_RD_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_DMA_RD_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_DMA_RD_ID);
            if (FALSE == pAvailRes->rdDma)
            {
                pAvailRes->rdDma = (uint32_t) TRUE;
                pAllocRes->rdDma = 0x0;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }

        /* CPORT Allocation should be before WR DMA allocation, as based on DMA
         *  type, the allocation policy changes. */

        if ((CAL_CAPT_INST_ID_SUB_CPORT_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_CPORT_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            uint32_t cportIdStart = 1U;

            /* Check for cport allocation policies */
            if (CAL_RM_CAL_ALLOC_POLICY_CPORTID_0_LEAST_PREFFERED !=
                (CAL_RM_CAL_ALLOC_POLICY_CPORTID_0_LEAST_PREFFERED & flags))
            {
                cportIdStart = 0;
            }

            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_CPORT_ID);
            rtnVal  = FVID2_EALLOC;
            for (i = cportIdStart; i < CAL_MAX_CPORT_ID; i++)
            {
                if (FALSE == pAvailRes->cport[i])
                {
                    pAvailRes->cport[i] = (uint32_t) TRUE;
                    pAllocRes->cport    = i;
                    rtnVal = FVID2_SOK;
                    break;
                }
            }
            if ((FVID2_EALLOC == rtnVal) && (1U == cportIdStart))
            {
                if (FALSE == pAvailRes->cport[0])
                {
                    pAvailRes->cport[0] = (uint32_t) TRUE;
                    pAllocRes->cport    = 0;
                    rtnVal = FVID2_SOK;
                }
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_DMA_WR_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_DMA_WR_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_DMA_WR_ID);
            rtnVal  = FVID2_EALLOC;

            /* Policy - write channel 0x0, is least preferred. Reserve it for
             *  last */
            i = 1U;
            if (CAL_RM_CAL_ALLOC_POLICY_WRDMA_0_LEAST_PREFFERED !=
                (CAL_RM_CAL_ALLOC_POLICY_WRDMA_0_LEAST_PREFFERED & flags))
            {
                i = 0;
            }

            for (; i < CAL_MAX_WR_DMA_CHANNEL; i++)
            {
                if (FALSE == pAvailRes->wrDma[i])
                {
                    pAvailRes->wrDma[i] = (uint32_t) TRUE;
                    pAllocRes->wrDma    = i;
                    rtnVal = FVID2_SOK;
                    break;
                }
            }

            if (FVID2_EALLOC == rtnVal)
            {
                if (FALSE == pAvailRes->wrDma[0x0U])
                {
                    pAvailRes->wrDma[0x0U] = (uint32_t) TRUE;
                    pAllocRes->wrDma       = 0x0U;
                    rtnVal = FVID2_SOK;
                }
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_CSI2_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_CSI2_ID)) &&
            (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_CSI2_ID);
            rtnVal  = FVID2_EALLOC;
            for (i = 0; i < CAL_MAX_CPORT_ID; i++)
            {
                if (FALSE == pAvailRes->csi2Ctx[csi2Instance][i])
                {
                    pAvailRes->csi2Ctx[csi2Instance][i] = (uint32_t) TRUE;
                    pAllocRes->csi2Ctx    = i;
                    rtnVal = FVID2_SOK;
                    break;
                }
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_LVDS_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_LVDS_ID)) && (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_LVDS_ID);
            if (FALSE == pAvailRes->lvds)
            {
                pAvailRes->lvds = (uint32_t) TRUE;
                pAllocRes->lvds = 0x0;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }

        if ((CAL_CAPT_INST_ID_SUB_CPI_ID ==
             (reqRes & CAL_CAPT_INST_ID_SUB_CPI_ID)) && (FVID2_SOK == rtnVal))
        {
            reqRes &= ((uint32_t) ~CAL_CAPT_INST_ID_SUB_CPI_ID);
            if (FALSE == pAvailRes->cpi)
            {
                pAvailRes->cpi = (uint32_t) TRUE;
                pAllocRes->cpi = 0x0;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }

        if (FVID2_SOK != rtnVal)
        {
            /* TODO release acquired resources */
            rtnVal = FVID2_EBADARGS;
        }
    }

    return rtnVal;
}

static int32_t releaseCaptRes(Cal_RmCaptInstObj_t           *pAvailRes,
                            const Cal_CaptBlocks_t *pAllocRes)
{
    int32_t rtnVal = FVID2_SOK;
    uint32_t csi2Instance = 0;
    if ((NULL == pAllocRes) || (NULL == pAvailRes))
    {
        rtnVal = FVID2_EBADARGS;
    }

    if (FVID2_SOK == rtnVal)
    {
        if ((0xFFFFFFFFU != pAllocRes->ppi0Inst) &&
            (0x0U != pAvailRes->numChOnPpi0))
        {
            pAvailRes->numChOnPpi0--;
        }
        if ((0xFFFFFFFFU != pAllocRes->ppi1Inst) &&
            (0x0U != pAvailRes->numChOnPpi1))
        {
            pAvailRes->numChOnPpi1--;
        }
        if ((0xFFFFFFFFU != pAllocRes->pixExtract) &&
            (CAL_CAPT_MAX_PIX_PROC_CONTEXT > pAllocRes->pixExtract))
        {
            pAvailRes->pixExtract[pAllocRes->pixExtract] = (uint32_t) FALSE;
        }
        if ((0xFFFFFFFFU != pAllocRes->dpmDecode) &&
            (CAL_CAPT_MAX_PIX_PROC_CONTEXT > pAllocRes->dpmDecode))
        {
            pAvailRes->dpmDecode[pAllocRes->dpmDecode] = (uint32_t) FALSE;
        }
        if ((0xFFFFFFFFU != pAllocRes->dpmEncode) &&
            (CAL_CAPT_MAX_PIX_PROC_CONTEXT > pAllocRes->dpmEncode))
        {
            pAvailRes->dpmEncode[pAllocRes->dpmEncode] = (uint32_t) FALSE;
        }
        if ((0xFFFFFFFFU != pAllocRes->pixPack) &&
            (CAL_CAPT_MAX_PIX_PROC_CONTEXT > pAllocRes->pixPack))
        {
            pAvailRes->pixPack[pAllocRes->pixPack] = (uint32_t) FALSE;
        }
        if (0xFFFFFFFFU != pAllocRes->bysOut)
        {
            pAvailRes->bysOut = (uint32_t) FALSE;
        }
        if (0xFFFFFFFFU != pAllocRes->bysIn)
        {
            pAvailRes->bysIn = (uint32_t) FALSE;
        }
        if (0xFFFFFFFFU != pAllocRes->vPort)
        {
            pAvailRes->vPort = (uint32_t) FALSE;
        }
        if (0xFFFFFFFFU != pAllocRes->rdDma)
        {
            pAvailRes->rdDma = (uint32_t) FALSE;
        }
        if ((0xFFFFFFFFU != pAllocRes->wrDma) &&
            (CAL_MAX_WR_DMA_CHANNEL > pAllocRes->wrDma))
        {
            pAvailRes->wrDma[pAllocRes->wrDma] = (uint32_t) FALSE;
        }
        if ((0xFFFFFFFFU != pAllocRes->cport) &&
            (CAL_MAX_CPORT_ID > pAllocRes->cport))
        {
            pAvailRes->cport[pAllocRes->cport] = (uint32_t) FALSE;
        }
        if (0xFFFFFFFFU != pAllocRes->lvds)
        {
            pAvailRes->lvds = (uint32_t) FALSE;
        }
        if (0xFFFFFFFFU != pAllocRes->cpi)
        {
            pAvailRes->cpi = (uint32_t) FALSE;
        }
        if ((0xFFFFFFFFU != pAllocRes->csi2Ctx) &&
            (CAL_MAX_CPORT_ID > pAllocRes->csi2Ctx))
        {
            csi2Instance = 0U;

            if(pAllocRes->ppi1Inst != 0xFFFFFFFFU)
            {
                csi2Instance = 1U;
            }

            pAvailRes->csi2Ctx[csi2Instance][pAllocRes->csi2Ctx] = (uint32_t) FALSE;
        }
    }
    return rtnVal;
}

