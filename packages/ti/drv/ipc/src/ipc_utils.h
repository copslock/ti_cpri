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
 *  \file ipc_utils.h
 *
 *  \brief Defines utility function prototypes such as queue, buffer manager,
 *          etc...
 */

#ifndef IPC_UTILS_H_
#define IPC_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/osal/osal.h>
#include <ti/drv/ipc/include/ipc_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Queue Element type */
typedef struct IpcUtils_QElem_s
{
    struct  IpcUtils_QElem_s *next;
    /**< Next Node */
    struct  IpcUtils_QElem_s *prev;
    /**< Next Node */
    void    *pData;
    /**< Pointer to data */
} IpcUtils_QElem;

/** \brief Queue Handle */
typedef IpcUtils_QElem IpcUtils_QHandle;

/** \brief Heap Create Parameters */
typedef struct IpcUtils_HeapParams_s
{
    uint32_t    align;
    /**< Not used in this implementation */
    uint32_t    blockSize;
    /**< Maximum block size, expected to be same for all blocks */
    void        *buf;
    /**< Heap Memory */
    uint32_t    bufSize;
    /**< Size of the heap provided */
    uint32_t    numBlocks;
    /**< Total number of blocks */
} IpcUtils_HeapParams;

/** \brief Heap Object type */
typedef struct IpcUtils_HeapElem_s
{
    IpcUtils_QHandle    qHandle;
    /**< Q required to store free blocks */
    uint32_t            numFreeBlocks;
    /**< Bookkeep of number of free blocks */
    uint32_t            blockSize;
    /**< Size of blocks */
}IpcUtils_HeapElem;

/** \brief Heap Handle */
typedef IpcUtils_HeapElem IpcUtils_HeapHandle;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \brief  IpcUtils_Init
 *
 *          Initializes the IPC utilites, requires function to disable and
 *          enable all interrupts.
 *
 *  \param pOsalInit     [IN]
 *
 *  \return IPC_SOK on success OR IPC_EINVALID_PARAMS if required parameters
 *          are not provided
 */
int32_t IpcUtils_Init(Ipc_OsalPrms *pOsalInit);

/**
 *  \brief  IpcUtils_DeInit
 *
 *          DeInitializes the IPC utilites.
 *
 *  \param None
 *
 *  \return IPC_SOK
 */
int32_t IpcUtils_DeInit(void);

/**
 *  \brief  IpcUtils_QgetHead
 *
 *          Returns the head node of the queue
 *
 *  \param IpcUtils_QHandle     [IN]
 *
 *  \return Queued Element or NULL
 */
void* IpcUtils_QgetHead(IpcUtils_QHandle *handle);
/**
 *  \brief  IpcUtils_QisEmpty
 *
 *          Checks if the queue is empty
 *
 *  \param IpcUtils_QHandle     [IN]
 *
 *  \return TRUE in case queue is empty FALSE otherwise
 */
uint8_t IpcUtils_QisEmpty(IpcUtils_QHandle *handle);

/**
 *  \brief  IpcUtils_Qput
 *
 *          Insert the node into queue
 *
 *  \param IpcUtils_QHandle     [IN]
 *  \param IpcUtils_QElem       [IN]    Node to be inserted
 *  \param void *               [IN]    Data associated with this node
 *
 *  \note Taking additional argument of void *data is done to ensure
 *          MISRA C 2012, Rule 11.5 can be honored
 *
 *  \return None
 */
void IpcUtils_Qput(IpcUtils_QHandle *handle, IpcUtils_QElem *elem, void *data);

/**
 *  \brief  IpcUtils_Qhead
 *
 *          Data contained in the head node is returned, but not dequeued
 *
 *  \param IpcUtils_QHandle     [IN]
 *
 *  \return void * Data associated with this node
 */
void* IpcUtils_QgetHeadNode(IpcUtils_QHandle *handle);

/**
 *  \brief  IpcUtils_Qnext
 *
 *          Data contained in the next to the specified node is returned,
 *          but not dequeued
 *
 *  \param IpcUtils_QHandle     [IN]
 *
 *  \return void * Data associated with the node
 */
void* IpcUtils_Qnext(IpcUtils_QElem *qelem);

/**
 *  \brief  IpcUtils_Qremove
 *
 *          Remove the given node from the list
 *
 *  \param IpcUtils_QHandle     [IN]
 *
 *  \return None
 */
void IpcUtils_Qremove(IpcUtils_QElem *qelem);

/**
 *  \brief  IpcUtils_Qdelete
 *
 *          Delete the queue
 *
 *  \param IpcUtils_QHandle     [IN]
 *
 *  \return None
 */
void IpcUtils_Qdelete(IpcUtils_QHandle *handle);

/**
 *  \brief  IpcUtils_Qcreate
 *
 *          Create the queue, setups the queue with no nodes
 *
 *  \param IpcUtils_QHandle     [IN/OUT]
 *
 *  \return IPC_SOK on success and IPC_EINVALID_PARAMS on bad arguments/ utils
 *          not yet initialized
 */
int32_t IpcUtils_Qcreate(IpcUtils_QHandle *handle);

/**
 *  \brief  IpcUtils_HeapCreate
 *
 *          Create a heap of fixed size units. Expects the caller provides the
 *          required memory space. Please note that no alignments are checked
 *          caller shall ensure the required memory alignment
 *
 *  \param IpcUtils_HeapHandle      [IN/OUT]
 *  \param IpcUtils_HeapParams      [IN]
 *
 *  \return IPC_SOK on success and IPC_EINVALID_PARAMS on bad arguments/ utils
 *          not yet initialized
 */
int32_t IpcUtils_HeapCreate(IpcUtils_HeapHandle *pHndl,
                            IpcUtils_HeapParams *param);
/**
 *  \brief  IpcUtils_HeapDelete
 *
 *          Deletes previously created heap
 *
 *  \param IpcUtils_HeapHandle      [IN/OUT]
 *
 *  \return None
 */
void IpcUtils_HeapDelete(IpcUtils_HeapHandle *pHndl);

/**
 *  \brief  IpcUtils_HeapAlloc
 *
 *          Allocate memory of fixed size, the size was specified while creating
 *          the heap.
 *
 *  \param IpcUtils_HeapHandle      [IN/OUT]
 *  \param size                     [Not Used]
 *  \param align                    [Not Used]
 *
 *  \return NON NULL Pointer on success and NULL in cases of error.
 */
void *IpcUtils_HeapAlloc(IpcUtils_HeapHandle *pHndl, uint32_t size,
                            uint32_t align);
/**
 *  \brief  IpcUtils_HeapFree
 *
 *          DeAllocates memory of fixed size, the size was specified while
 *          creating the heap.
 *
 *  \param IpcUtils_HeapHandle      [IN/OUT]
 *  \param size                     [Not Used]
 *  \param align                    [Not Used]
 *
 *  \return None
 */
void IpcUtils_HeapFree(IpcUtils_HeapHandle *pHndl, void* block, uint32_t size);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_UTILS_H_ */

