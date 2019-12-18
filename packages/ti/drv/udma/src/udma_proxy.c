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
 *  \file udma_proxy.c
 *
 *  \brief File containing the UDMA driver proxy related APIs.
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

int32_t Udma_proxyAlloc(Udma_DrvHandle drvHandle,
                        Udma_ProxyHandle proxyHandle,
                        uint16_t proxyNum)
{
    int32_t     retVal = UDMA_SOK;
    struct tisci_msg_rm_proxy_cfg_req req;

    /* Error check */
    if((NULL_PTR == drvHandle) || (NULL_PTR == proxyHandle))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(drvHandle->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_PROXY_ANY == proxyNum)
        {
            /* Alloc free proxy */
            proxyHandle->proxyNum = Udma_rmAllocProxy(drvHandle);
            if(UDMA_PROXY_INVALID == proxyHandle->proxyNum)
            {
                retVal = UDMA_EALLOC;
            }
        }
        else
        {
            if(proxyNum >= drvHandle->maxProxy)
            {
                Udma_printf(drvHandle, "[Error] Out of range proxy index!!!\n");
                retVal = UDMA_EINVALID_PARAMS;
            }
            else
            {
                proxyHandle->proxyNum = proxyNum;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        req.valid_params = 0U;
        req.nav_id       = drvHandle->devIdProxy;
        req.index        = proxyHandle->proxyNum;
        retVal = Sciclient_rmSetProxyCfg(&req, UDMA_SCICLIENT_TIMEOUT);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] SciClient Set proxy config failed!!!\n");
            if (UDMA_PROXY_ANY == proxyNum)
            {
                /* Free-up resources */
                Udma_rmFreeProxy(proxyHandle->proxyNum, drvHandle);
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        proxyHandle->drvHandle      = drvHandle;
        proxyHandle->proxyAddr      = 0U;   /* Will be set during config */
        proxyHandle->proxyInitDone  = UDMA_INIT_DONE;
    }

    return (retVal);
}

int32_t Udma_proxyFree(Udma_ProxyHandle proxyHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if(NULL_PTR == proxyHandle)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(proxyHandle->proxyInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = proxyHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Free-up resources */
        Udma_assert(drvHandle, proxyHandle->proxyNum != UDMA_PROXY_INVALID);
        Udma_rmFreeProxy(proxyHandle->proxyNum, drvHandle);
        proxyHandle->drvHandle       = (Udma_DrvHandle) NULL_PTR;
        proxyHandle->proxyNum        = UDMA_PROXY_INVALID;
        proxyHandle->proxyAddr       = 0U;
        proxyHandle->proxyInitDone   = UDMA_DEINIT_DONE;
    }

    return (retVal);
}

int32_t Udma_proxyConfig(Udma_ProxyHandle proxyHandle,
                         const Udma_ProxyCfg *proxyCfg)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandle      drvHandle;
    CSL_ProxyThreadCfg  threadCfg;

    /* Error check */
    if(NULL_PTR == proxyHandle)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(proxyHandle->proxyInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = proxyHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        threadCfg.mode      = proxyCfg->proxyMode;
        /* CSL expects ring size in bytes */
        threadCfg.elSz      = ((uint32_t) 1U << (proxyCfg->elemSize + 2U));
        threadCfg.queueNum  = (uint32_t) proxyCfg->ringNum;
        threadCfg.errEvtNum = UDMA_EVENT_INVALID;
        retVal = CSL_proxyCfgThread(
                     &drvHandle->proxyCfg,
                     drvHandle->proxyTargetNumRing,
                     proxyHandle->proxyNum,
                     &threadCfg);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] Proxy config failed!!!\n");
        }
        else
        {
            proxyHandle->proxyAddr =
                CSL_proxyGetDataAddr(
                    &drvHandle->proxyCfg,
                    drvHandle->proxyTargetNumRing,
                    proxyHandle->proxyNum,
                    threadCfg.elSz);
        }
    }

    return (retVal);
}

