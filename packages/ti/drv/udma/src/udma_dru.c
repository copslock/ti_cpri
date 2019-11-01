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
 *  \file udma_dru.c
 *
 *  \brief File containing the UDMA driver DRU related APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/udma/src/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Udma_chDruSubmitTr(Udma_ChHandle chHandle, const CSL_UdmapTR *tr)
{
    uint32_t                utcChNum;
    Udma_DrvHandle          drvHandle = chHandle->drvHandle;
    const Udma_UtcInstInfo *utcInfo;

    utcInfo = chHandle->utcInfo;
    utcChNum = chHandle->extChNum - utcInfo->startCh;

    CSL_druChSubmitTr(utcInfo->druRegs, utcChNum, drvHandle->druCoreId, tr);

    return;
}

uint32_t Udma_druGetNumQueue(Udma_DrvHandle drvHandle, uint32_t utcId)
{
    int32_t                 retVal = UDMA_SOK;
    const Udma_UtcInstInfo *utcInfo;
    uint32_t                numQueue = 0U;

    /* Error check */
    if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }

    if(UDMA_SOK == retVal)
    {
        utcInfo = Udma_chGetUtcInst(drvHandle, utcId);
        if(NULL_PTR == utcInfo)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle, "[Error] Invalid UTC ID!!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);
        numQueue = utcInfo->numQueue;
    }

    return (numQueue);
}

int32_t Udma_druQueueConfig(Udma_DrvHandle               drvHandle,
                            uint32_t                     utcId,
                            uint32_t                     queueId,
                            const CSL_DruQueueConfig    *queueCfg)
{
    int32_t                 retVal = UDMA_SOK;
    const Udma_UtcInstInfo *utcInfo;

    /* Error check */
    if((NULL_PTR == drvHandle) ||
       (drvHandle->drvInitDone != UDMA_INIT_DONE) ||
       (NULL_PTR == queueCfg))
    {
        retVal = UDMA_EBADARGS;
    }

    if(UDMA_SOK == retVal)
    {
        utcInfo = Udma_chGetUtcInst(drvHandle, utcId);
        if(NULL_PTR == utcInfo)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle, "[Error] Invalid UTC ID!!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);
        retVal = CSL_druQueueConfig(utcInfo->druRegs, queueId, queueCfg);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] DRU queue config failed!!!\n");
        }
    }

    return (retVal);
}

volatile uint64_t *Udma_druGetTriggerRegAddr(Udma_ChHandle chHandle)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandle          drvHandle;
    const Udma_UtcInstInfo *utcInfo;
    volatile uint64_t      *pSwTrigReg = (volatile uint64_t *) NULL_PTR;

    /* Error check */
    if((NULL_PTR == chHandle) ||
       (chHandle->chInitDone != UDMA_INIT_DONE) ||
       ((chHandle->chType & UDMA_CH_FLAG_UTC) != UDMA_CH_FLAG_UTC))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        utcInfo = chHandle->utcInfo;
        Udma_assert(drvHandle, utcInfo != NULL_PTR);
        if(UDMA_UTC_TYPE_DRU == utcInfo->utcType)
        {
            Udma_assert(drvHandle, chHandle->pDruRtRegs != NULL_PTR);
            pSwTrigReg = &chHandle->pDruRtRegs->CHRT_SWTRIG;
        }
    }

    return (pSwTrigReg);
}

void UdmaDruQueueConfig_init(CSL_DruQueueConfig *queueCfg)
{
    if(NULL_PTR != queueCfg)
    {
        queueCfg->priority          = 0U;
        queueCfg->qos               = 0U;
        queueCfg->orderId           = 0U;
        queueCfg->consecuitveTrans  = 0U;
        queueCfg->rearbWait         = 0U;
    }

    return;
}
