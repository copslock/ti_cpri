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
 *  \file udma_test_bug.c
 *
 *  \brief UDMA test case file to test bugs filed.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <udma_test.h>

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

int32_t udmaTestBugTcPDK_3863(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chType;
    UdmaTestChObj      *chObj;
    Udma_ChPrms         chPrms;
    CSL_UdmapTdResponse tdResponse;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: PDK-3863 Bug Testcase: Udma_chDequeueTdResponse NULL check ::\r\n", taskObj->taskId);

    chObj = taskObj->chObj[0U];
    GT_assert(taskObj->traceMask, chObj != NULL);

    chType = chObj->chPrms->chType;
    UdmaChPrms_init(&chPrms, chType);

    /* Open channel with all ring memory as NULL */
    retVal = Udma_chOpen(chObj->drvHandle, &chObj->drvChObj, chType, &chPrms);
    if(UDMA_SOK != retVal)
    {
        GT_0trace(taskObj->traceMask, GT_ERR, " UDMA channel open failed!!\n");
    }
    else
    {
        chObj->chHandle = &chObj->drvChObj;
        GT_2trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: Allocated Ch   : %d ::\r\n",
                  taskObj->taskId, Udma_chGetNum(chObj->chHandle));
    }

    if(UDMA_SOK == retVal)
    {
        retVal = Udma_chDequeueTdResponse(chObj->chHandle, &tdResponse);
        if(retVal == UDMA_EFAIL)
        {
            /* NULL pointer check returns EFAIL. So pass the test */
            retVal = UDMA_SOK;
        }
        else
        {
            GT_0trace(taskObj->traceMask, GT_ERR,
                " UDMA Udma_chDequeueTdResponse didnot return right error!!\n");
        }
    }

    if(NULL != chObj->chHandle)
    {
        retVal += Udma_chClose(chObj->chHandle);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR,
                " UDMA channel close failed!!\n");
        }
    }

    return (retVal);
}

int32_t udmaTestBugTcPDK_4654(UdmaTestTaskObj *taskObj)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                instId;
    Udma_DrvHandle          drvHandle;
    struct Udma_ProxyObj    proxyObj;
    Udma_ProxyHandle        proxyHandle = &proxyObj;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: PDK-4654 Bug Testcase: Deinit RM check ::\r\n", taskObj->taskId);

    /* Deinit already init driver */
    retVal = udmaTestDeinitDriver(taskObj->testObj);
    if(UDMA_SOK != retVal)
    {
        GT_0trace(taskObj->traceMask, GT_ERR, " UDMA deinit failed!!\n");
    }

    if(UDMA_SOK == retVal)
    {
        /* Do a fresh init */
        retVal = udmaTestInitDriver(taskObj->testObj);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " UDMA re-init failed!!\n");
        }

        if(UDMA_SOK == retVal)
        {
            for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
            {
                drvHandle = &taskObj->testObj->drvObj[instId];

                /* Alloc a proxy */
                retVal = Udma_proxyAlloc(drvHandle, proxyHandle, UDMA_PROXY_ANY);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy alloc failed!!\n");
                }

                if(UDMA_SOK == retVal)
                {
                    /* Deinit with a resource not-freed */
                    retVal = Udma_deinit(drvHandle);
                    if(UDMA_SOK == retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR,
                            " UDMA passed when it should have failed!!\n");
                    }
                    else
                    {
                        retVal = Udma_proxyFree(proxyHandle);
                        if(UDMA_SOK != retVal)
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR,
                                " Proxy free failed!!\n");
                        }
                    }
                }

                if(UDMA_SOK != retVal)
                {
                    break;
                }
            }

            /* This deinit should pass */
            retVal += udmaTestDeinitDriver(taskObj->testObj);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " UDMA deinit failed!!\n");
            }
        }

        /* Init before ending the testcase */
        retVal = udmaTestInitDriver(taskObj->testObj);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " UDMA re-init failed!!\n");
        }
    }

    return (retVal);
}
