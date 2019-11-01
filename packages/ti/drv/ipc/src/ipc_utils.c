/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 *  \file ipc_utils.c
 *
 *  \brief Implementation of ipc utility functions such as queues,
 *          buffer manager, etc...
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "ipc_osal.h"
#include "ipc_utils.h"

#include <ti/csl/csl_types.h>
#include <ti/drv/ipc/soc/ipc_soc.h>
#include <ti/drv/ipc/include/ipc_types.h>

/* ========================================================================== */
/*                            Local Types/Defines                             */
/* ========================================================================== */
#define IPC_UTILS_UN_INITIALIZED        (0U)
/**< State Un Initialized */
#define IPC_UTILS_INITIALIZED           (1U)
/**< State Un Initialized */

/** \brief Utils Object
 */
typedef struct Ipc_UtilsObj_s
{
    uint32_t                    state;
    /**< State indicator */
}Ipc_UtilsObj;

/* ========================================================================== */
/*                               Globals                                      */
/* ========================================================================== */
Ipc_UtilsObj gIpcUtilsObj = {IPC_UTILS_UN_INITIALIZED};
/**< Function disable all interrupts */

/* ========================================================================== */
/*                              API's                                         */
/* ========================================================================== */
int32_t IpcUtils_Init(Ipc_OsalPrms *pOsalInit)
{
    int32_t rtnVal = IPC_EINVALID_PARAMS;
    if (NULL != pOsalInit)
    {
        gIpcUtilsObj.state          = IPC_UTILS_INITIALIZED;
        rtnVal = IPC_SOK;
    }
    return (rtnVal);
}

int32_t IpcUtils_DeInit(void)
{
    gIpcUtilsObj.state          = IPC_UTILS_UN_INITIALIZED;

    return (IPC_SOK);
}

/* Queue Related */
void* IpcUtils_QgetHead(IpcUtils_QHandle *handle)
{
    IpcUtils_QElem *elem, *rtnVal = NULL;

    if (IPC_UTILS_INITIALIZED == gIpcUtilsObj.state)
    {
        elem = handle->next;

        handle->next = elem->next;
        elem->next->prev = handle;
        rtnVal = (IpcUtils_QElem *) elem->pData;
    }

    return (rtnVal);
}

uint8_t IpcUtils_QisEmpty(IpcUtils_QHandle *handle)
{
    uint8_t rtnVal = 0U;
    if (handle->next == handle)
    {
        rtnVal = TRUE;
    }
    return (rtnVal);
}

void IpcUtils_Qput(IpcUtils_QHandle *handle, IpcUtils_QElem *elem, void *data)
{
    if ((IPC_UTILS_INITIALIZED == gIpcUtilsObj.state) &&
        ((NULL != elem) && (NULL != data)))
    {
        elem->pData = data;
        elem->next = handle;
        elem->prev = handle->prev;
        handle->prev->next = elem;
        handle->prev = elem;
    }
    return;
}

void* IpcUtils_QgetHeadNode(IpcUtils_QHandle *handle)
{
    void *rtnVal = NULL;

    if ((IPC_UTILS_INITIALIZED == gIpcUtilsObj.state) &&
        (NULL != handle))
    {
        rtnVal = handle->next->pData;
    }

    return (rtnVal);
}

void* IpcUtils_Qnext(IpcUtils_QElem *qelem)
{
    void *rtnVal = NULL;
    if (NULL != qelem)
    {
        rtnVal = qelem->next->pData;
    }

    return rtnVal;
}

void IpcUtils_Qremove(IpcUtils_QElem *qelem)
{
    if (NULL != qelem)
    {
        qelem->prev->next = qelem->next;
        qelem->next->prev = qelem->prev;
    }

    return;
}

void IpcUtils_Qdelete(IpcUtils_QHandle *handle)
{
    if (NULL != handle)
    {
        handle->next = NULL;
        handle->prev = NULL;
        handle->pData = NULL;
    }

    return;
}

int32_t IpcUtils_Qcreate(IpcUtils_QHandle *handle)
{
    int32_t rtnVal = IPC_EINVALID_PARAMS;
    if ((NULL != handle) && (IPC_UTILS_INITIALIZED == gIpcUtilsObj.state))
    {
        handle->next = handle;
        handle->prev = handle;
        handle->pData = handle;
        rtnVal = IPC_SOK;
    }

    return (rtnVal);
}

int32_t IpcUtils_HeapCreate(IpcUtils_HeapHandle *pHndl,
                            IpcUtils_HeapParams *param)
{
    uint8_t *tempBufPtr;
    uint32_t idx;
    int32_t rtnVal = IPC_EINVALID_PARAMS;

    if (((NULL != param) && (IPC_UTILS_INITIALIZED == gIpcUtilsObj.state)) &&
        ((NULL != param->buf) && (NULL != pHndl)))
    {
        if (param->bufSize >= (param->numBlocks * param->blockSize))
        {
            rtnVal = IPC_SOK;
        }
    }

    if (IPC_SOK == rtnVal)
    {
        /*
            . Setup Heap Handle variables
            . Create Q
            . Slice buffer provided and insert them into Q
        */
        pHndl->numFreeBlocks = 0U;
        pHndl->blockSize = param->blockSize;

        rtnVal = IpcUtils_Qcreate(&pHndl->qHandle);
        if (IPC_SOK == rtnVal)
        {
            for (idx = 0U; idx < param->numBlocks; idx++)
            {
                tempBufPtr = (uint8_t *) param->buf + (idx * param->blockSize);

                /* Will flag MISRA C Violation for tempBufPtr casting,
                    no fix? */
                IpcUtils_Qput(&pHndl->qHandle, (IpcUtils_QElem *) tempBufPtr,
                                (void *) tempBufPtr);

                pHndl->numFreeBlocks++;
            }
        }
    }
    return rtnVal;
}

void IpcUtils_HeapDelete(IpcUtils_HeapHandle *pHndl)
{
    if (NULL != pHndl)
    {
        IpcUtils_Qdelete(&pHndl->qHandle);
        pHndl->numFreeBlocks = 0U;
    }
    return;
}

void *IpcUtils_HeapAlloc(IpcUtils_HeapHandle *pHndl, uint32_t size,
                            uint32_t align)
{
    void *rtnVal = NULL;
    if (NULL != pHndl)
    {
        rtnVal = IpcUtils_QgetHead(&pHndl->qHandle);
        pHndl->numFreeBlocks--;
    }

    return (rtnVal);
}

void IpcUtils_HeapFree(IpcUtils_HeapHandle *pHndl, void* block, uint32_t size)
{
    if ((NULL != pHndl) && (NULL != block))
    {
        /* Will flag MISRA C Violation for tempBufPtr casting,
            no fix? */
        IpcUtils_Qput(&pHndl->qHandle, (IpcUtils_QElem *) block, block);
        pHndl->numFreeBlocks++;
    }

    return;
}

#ifdef IPC_EXCLUDE_CTRL_TASKS
/* Fix Me : Move it from here */
int32_t SystemP_printf(const char* fmt, ...)
{
    int32_t ret = 0U;

    return ret;
}
#endif


/* ========================================================================== */
/*                          Local Functions                                   */
/* ========================================================================== */
