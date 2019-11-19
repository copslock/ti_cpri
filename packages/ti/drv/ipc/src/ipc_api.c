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
 *  \file ipc_api.c
 *
 *  \brief File containing the IPC driver APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/drv/ipc/ipc.h>
#include <string.h>

#ifndef IPC_EXCLUDE_CTRL_TASKS
#include <ti/osal/osal.h>
#include <ti/osal/TaskP.h>
#endif

#include "ipc_priv.h"
#include "ipc_virtioPrivate.h"
#include "ipc_osal.h"
#include "ipc_utils.h"
#include "ipc_mailbox.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define MAXCALLBACKS           IPC_MAX_PROCS

#define MSGBUFFERSIZE          512U
#define SERVICENAMELEN         32U
#define HEAPALIGNMENT          8U

struct RPMessage_Object_s;
typedef struct RPMessage_Object_s*            RPMessage_EndptPool[MAXENDPOINTS];

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

#define CNTRLMSG_ANNOUNCE (0x00000000U)

/* Message Header: Must match mp_msg_hdr in virtio_rp_msg.h on Linux side. */
typedef struct RPMessage_MsgHeader_s
{
    uint32_t  srcAddr;             /* source endpoint addr       */
    uint32_t  dstAddr;             /* destination endpoint addr  */
    uint32_t  srcProcId;           /* reserved                   */
    uint16_t  dataLen;             /* data length                */
    uint16_t  flags;               /* bitmask of different flags */
    uint8_t   payload[];           /* Data payload               */
}RPMessage_MsgHeader;


/* common fields of a control message */
typedef struct RPMessage_CntrlMsg_s
{
    uint32_t  type;
}RPMessage_CntrlMsg;

/* announcement message */
typedef struct RPMessage_Announcement_s
{
    char                 name[SERVICENAMELEN];
    uint32_t             endPt;
    RPMessage_CntrlMsg   ctrl;
} RPMessage_Announcement;


/* list element for the announcedEndpts list */
typedef struct RPMessage_NameEntry_s
{
    IpcUtils_QElem   elem;
    uint32_t         procId;
    uint32_t         endPt;
    char             name[SERVICENAMELEN];
} RPMessage_NameEntry;


/* list element for the waitingTasks list */
typedef struct RPMessage_Waiter_s
{
    IpcUtils_QElem     elem;
    void               *semHandle;
    uint32_t           procId;
    uint32_t           endPt;
    char               name[SERVICENAMELEN];
} RPMessage_Waiter;
/**
 *  \brief Element to hold payload copied onto receiver's queue.
 */
typedef struct RPMessage_MsgElem_s
{
    IpcUtils_QElem    elem;          /**< Allow list linking.          */
    uint32_t          len;           /**< Length of data               */
    uint32_t          src;           /**< Src address/endPt of the msg */
    uint32_t          procId;        /**< Processor sending the msg    */
    uint8_t           data[];        /**< payload begins here          */
} RPMessage_MsgElem;

/* The RPMessage Object */
typedef struct RPMessage_Object_s
{
    uint32_t             endPt;        /**< Unique id (procId | queueIndex) */
    void                 *semHandle;    /**< I/O Completion                 */
    RPMessage_Callback   cb;           /**< RPMessage Callback */
    void*                arg;          /**< Callback argument */
    IpcUtils_QHandle     queue;       /**< Queue of pending messages             */
    uint8_t                 unblocked;    /**< Use with signal to unblock _receive() */
    IpcUtils_HeapParams  heapParams;   /* Save the heapBuf params to delete Heap   */
    IpcUtils_HeapHandle  heap;         /* Heap from which to allocate free messages */
    RPMessage_EndptPool  *pool;        /**< track associated pool for cleanup */
    void*                recv_buffer;  /**< Application buffer from RPMessage_recv() */
    RPMessage_MsgElem    payload;      /**< Message info for RPMessage_recv() */
} RPMessage_Object;

/**
 *  \brief This struct is used as private data supplied to the VQ
 *         callback function.  It is also passed to the SWI
 *         functions through arg0. */
typedef struct RPMessage_CallbackData_s
{
    Virtio_Handle        vq;
    RPMessage_EndptPool  *pool;
    Ipc_OsalHIsrHandle   swi;
} RPMessage_CallbackData;

/**
 *  \brief Module_State
 */
typedef struct RPMessage_Module_s
{
    /* Instance gate: */
    Ipc_OsalHIsrGateHandle gateSwi;

    /* Lookup VQ handle from procId for sending messages. */
    /* Array indexed by procId.                           */
    Virtio_Handle tx_VQs[IPC_MAX_PROCS];

    /* Array of callback structs. Used only for RX VirtQueues */
    RPMessage_CallbackData VQ_callbacks[MAXCALLBACKS];

    uint32_t numCallbacks;

    /* Global endpoint pool */
    RPMessage_EndptPool globalPool;

    /* Lookup endpoint pool from procId.  Array indexed by procId. */
    /* RPMessage_EndptPool pools[MAXREMOTEPROCS];            */

    /* Queue to track announcements from other processors */
    IpcUtils_QHandle announcedEndpts;

    /* Queue to hold the semaphores of waiting tasks */
    IpcUtils_QHandle waitingTasks;

    /* Array of RPMessage Object, memory supplied by application */
    RPMessage_Object*   objArry;

    /* Current object Count */
    uint16_t objCnt;

    /* NameEntry count */
    uint16_t  nameEntryCnt;

    RPMessage_NameEntry  nameEntry[IPC_MAX_PROCS];

} RPMessage_Module;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static RPMessage_Object* RPMessage_rawCreate(RPMessage_Params *params,
        RPMessage_EndptPool* pool, uint32_t *endPt);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static RPMessage_Module module;

static Ipc_Object gIpcObject;
/**< IPC Object */
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

RPMessage_Object* RPMessage_lookupEndpt(void* p, uint32_t e);
void RPMessage_assignEndpt(void* p, uint32_t e, RPMessage_Object* obj);
void UNUSED(const uint32_t* x);
uint8_t is_aligned(const void* ptr, uint32_t byteCnt);
RPMessage_Object*  RPMessage_allocObject(void);


inline RPMessage_Object* RPMessage_lookupEndpt(void* p, uint32_t e)
{
    return (((RPMessage_Object**)(p))[(e)]);
}

inline void RPMessage_assignEndpt(void* p, uint32_t e, RPMessage_Object* obj)
{
    (((RPMessage_Object**)(p))[(e)] = (obj));
}

inline void UNUSED(const uint32_t* x)
{
    (void)(x);
}

inline uint8_t is_aligned(const void* ptr, uint32_t byteCnt)
{
    uint8_t retVal;
    uint32_t bufAddr = (uint32_t)(uintptr_t)ptr;
    retVal = (0 == (bufAddr%byteCnt)) ? TRUE : FALSE;
    return retVal;
}



int32_t RPMessageParams_init(RPMessage_Params *params)
{
    int32_t    retVal = IPC_SOK;

    if(params == NULL)
    {
        retVal = IPC_EBADARGS;
    }
    else
    {
        params->requestedEndpt = RPMESSAGE_ANY;
        params->numBufs        = RPMessage_Buffer_Count_Default;
        params->stackBuffer    = NULL;
        params->stackSize      = 0U;
    }

    return (retVal);
}

uint32_t RPMessage_getMessageBufferSize(void)
{
    return MSGBUFFERSIZE;
}

uint32_t RPMessage_getObjMemRequired(void)
{
    uint32_t objSize = sizeof(RPMessage_Object);
    objSize = ((objSize + HEAPALIGNMENT-1) & ~(HEAPALIGNMENT-1));
    return objSize;
}


RPMessage_Object*  RPMessage_allocObject(void)
{
    RPMessage_Object* obj = NULL;

    if(module.objArry != NULL)
    {
         obj = (RPMessage_Object*)&(module.objArry[module.objCnt]);
         module.objCnt++;
    }

    return obj;
}

/*
 *  ======== RPMessage_enqueMsg ========
 */
/* Always assumed to be called in SWI context */
static int32_t RPMessage_enqueMsg(RPMessage_EndptPool *pool, RPMessage_MsgHeader *msg)
{
    int32_t             status = IPC_SOK;
    int32_t             key;
    uint32_t            size;
    RPMessage_MsgElem  *payload;
    RPMessage_Object   *obj = NULL;
    Ipc_OsalPrms        *pOsalPrms = &gIpcObject.initPrms.osalPrms;

    /* Protect from RPMessage_delete */
    if (NULL != pOsalPrms)
    {
        if ((NULL != pOsalPrms->lockHIsrGate) &&
            (NULL != pOsalPrms->unLockHIsrGate))
        {
            key = pOsalPrms->lockHIsrGate(module.gateSwi);
            obj = RPMessage_lookupEndpt(pool, msg->dstAddr);
            pOsalPrms->unLockHIsrGate(module.gateSwi, key);
        }
    }

    if (NULL != obj)
    {
        if(NULL != obj->recv_buffer)
        {
            memcpy(obj->recv_buffer, msg->payload, msg->dataLen);
            obj->recv_buffer = NULL;
            obj->payload.len = msg->dataLen;
            obj->payload.src = msg->srcAddr;
            obj->payload.procId = msg->srcProcId;

            if (NULL != gIpcObject.initPrms.newMsgFxn)
            {
                gIpcObject.initPrms.newMsgFxn(msg->srcAddr, msg->srcProcId);
            }

            if (NULL != pOsalPrms->unlockMutex)
            {
                pOsalPrms->unlockMutex(obj->semHandle);
            }
        }
        else
        {
            /* Allocate a buffer to copy the payload: */
            size = msg->dataLen + sizeof(RPMessage_MsgElem);

            /* HeapBuf_alloc() is non-blocking, so needs protection: */
            key = pOsalPrms->disableAllIntr();
            payload = (RPMessage_MsgElem *)IpcUtils_HeapAlloc(&obj->heap, size, 0);

            if (payload != NULL)
            {
                memcpy(payload->data, msg->payload, msg->dataLen);
                payload->len = msg->dataLen;
                payload->src = msg->srcAddr;
                payload->procId = msg->srcProcId;

                IpcUtils_Qput(&obj->queue, &payload->elem);

                pOsalPrms->restoreAllIntr(key);

                if (NULL != gIpcObject.initPrms.newMsgFxn)
                {
                    gIpcObject.initPrms.newMsgFxn(msg->srcAddr, msg->srcProcId);
                }

                if (NULL != pOsalPrms->unlockMutex)
                {
                    pOsalPrms->unlockMutex(obj->semHandle);
                }
            }
            else
            {
                pOsalPrms->restoreAllIntr(key);
                status = IPC_EFAIL;
            }
        }
    }

    return status;
}

/**
 * \brief RPMessage SWI function to talk to Linux
 */
static void RPMessage_swiLinuxFxn(uintptr_t arg0, uintptr_t arg1)
{
    RPMessage_CallbackData *cbdata = (RPMessage_CallbackData*)arg0;
    RPMessage_MsgHeader    *msg;
    Int16                  token;
    int32_t                len;
    uint8_t                usedBufAdded = FALSE;

    /* Process all available buffers: */
    while ((token = Virtio_getAvailBuf(cbdata->vq, (void **)&msg, &len)) >= 0)
    {
       /* Pass to desitination queue (which is on this proc): */
        RPMessage_enqueMsg(cbdata->pool, msg);

        Virtio_addUsedBuf(cbdata->vq, token, MSGBUFFERSIZE);
        usedBufAdded = TRUE;
    }

    if (TRUE == usedBufAdded)
    {
       /* Tell host we've processed the buffers: */
       Virtio_kick(cbdata->vq);
    }
}

/**
 * \brief RPMessage SWI function
 */
static void RPMessage_swiFxn(uintptr_t arg0, uintptr_t arg1)
{
    RPMessage_CallbackData *cbdata = (RPMessage_CallbackData*)arg0;
    RPMessage_MsgHeader    *msg;

    /* Process all available buffers: */
    while ((msg = (RPMessage_MsgHeader *) Virtio_getUsedBuf(cbdata->vq)) != NULL)
    {
       /* Pass to desitination queue (which is on this proc): */
        RPMessage_enqueMsg(cbdata->pool, msg);

        Virtio_addAvailBuf(cbdata->vq, msg);
    }
}

/**
 *  \brief virtio_callback - This function executes in HWI context
 */
static void virtio_callback(uint32_t* priv)
{
    RPMessage_CallbackData *cbdata = (RPMessage_CallbackData*)priv;

    if (NULL != cbdata)
    {
        gIpcObject.initPrms.osalPrms.postHIsr(&cbdata->swi);
    }
}

/**
 *  \brief RPMessage_announce : Announces the availabilty of an
 *          endpoint to all processors or only one.
 */
int32_t RPMessage_announce(uint32_t remoteProcId, uint32_t endPt, const char* name)
{
    RPMessage_EndptPool     *pool = &module.globalPool;
    RPMessage_Announcement  msg;
    Virtio_Handle           vq;
    uint32_t                c;
    uint32_t                procId;
    int32_t                 status = IPC_SOK;
    uint32_t                namelen;

#if  DEBUG_PRINT
    SystemP_printf("RPMessage_announce : remote %d, endpt %d, name %s\n",
        remoteProcId, endPt, name);
#endif

    namelen = strlen(name);
    if(namelen >= SERVICENAMELEN || namelen == 0)
    {
        status = IPC_EFAIL;
    }

    if(IPC_EFAIL != status)
    {
        msg.ctrl.type = CNTRLMSG_ANNOUNCE;
        msg.endPt = endPt;
        strncpy(msg.name, name, SERVICENAMELEN-1);
        msg.name[SERVICENAMELEN-1] = '\0';

        if(remoteProcId == RPMESSAGE_ALL)
        {
            for(c = 0; c < module.numCallbacks; c++)
            {
                /* Find callback for RX VQs that have matching pool. */
                if(module.VQ_callbacks[c].pool == pool)
                {
                    vq = module.VQ_callbacks[c].vq;
                    procId = Virtio_getProcId(vq);
#if DEBUG_PRINT
                    SystemP_printf("RPMessage_announce.....c%d ProcId %d\n", c, procId);
#endif
                    status = RPMessage_send(NULL, procId, IPC_CTRL_ENDPOINT_ID,
                            IPC_CTRL_ENDPOINT_ID, &msg, sizeof(msg));
                    if (status != IPC_SOK)
                    {
                        SystemP_printf("RPMessage_announce.....failed to send c %d (%s)\n", c, Ipc_mpGetName(procId));
                        /* even if failed to annouce to one CPU continue to other CPUs */
                        continue;
                    }
                }
            }
        }
        else
        {
            status = RPMessage_send(NULL, remoteProcId, IPC_CTRL_ENDPOINT_ID,
                    IPC_CTRL_ENDPOINT_ID, &msg, sizeof(msg));
            if (status != IPC_SOK)
            {
                SystemP_printf("RPMessage_announce.....failed to send\n");
            }
        }
    }

    return status;
}

#ifndef IPC_EXCLUDE_CTRL_TASKS
/**
 *  \brief RPMessage_processAnnounceMsg : Handle an endpoint annoucement
 *         message from another processor
 */
static void RPMessage_processAnnounceMsg(RPMessage_Announcement *amsg, uint32_t procId)
{
    int32_t key;
    RPMessage_NameEntry *p;
    RPMessage_Waiter *w;
    IpcUtils_QElem *elem, *head;
    int32_t rtnVal = IPC_SOK;
    Ipc_OsalPrms *pOsalPrms = &gIpcObject.initPrms.osalPrms;

#if DEBUG_PRINT
    SystemP_printf("RPMessage_processAnnounceMsg : announcement from %d for endPt %d\n",
             procId, amsg->endPt );
#endif

    if (NULL != pOsalPrms)
    {
        if (((NULL == pOsalPrms->lockHIsrGate) ||
             (NULL == pOsalPrms->unLockHIsrGate)) ||
            (NULL == pOsalPrms->unlockMutex))
        {
            rtnVal = IPC_EFAIL;
        }
    }

    if (IPC_SOK == rtnVal)
    {
        key = pOsalPrms->lockHIsrGate(module.gateSwi);

        /* Save the annoucement for future calls to */
        /* RPMessage_peerIsReady().                 */
        if(module.nameEntryCnt == IPC_MAX_PROCS-1)
        {
            pOsalPrms->unLockHIsrGate(module.gateSwi, key);
            SystemP_printf("RPMessage_processAnnounceMsg : all remote core done\n");
            rtnVal = IPC_EFAIL;
        }

        if(rtnVal != IPC_EFAIL)
        {
            p = &module.nameEntry[module.nameEntryCnt];
            p->endPt = amsg->endPt;
            p->procId = procId;
            strncpy(p->name, amsg->name, SERVICENAMELEN-1);
            p->name[SERVICENAMELEN-1] = '\0';
            module.nameEntryCnt++;

            /* No interrupt / SWI protection, required here again.
               We are already in SWI protected region */
            IpcUtils_Qput(&module.announcedEndpts, &p->elem);

            /* Wakeup all the tasks that are waiting on the */
            /* announced name.                              */
            if (!IpcUtils_QisEmpty(&module.waitingTasks))
            {
                /* No interrupt / SWI protection, required here again.
                   We are already in SWI protected region */
                elem = head = (IpcUtils_QElem *) IpcUtils_QgetHeadNode(&module.waitingTasks);
                do
                {
                    w = (RPMessage_Waiter*)elem;
                    if( (NULL != w) && 
                            (strncmp(w->name, amsg->name, SERVICENAMELEN-1) == 0) &&
                            (w->procId == procId || w->procId == RPMESSAGE_ANY))
                    {
                        /* Update the waiter's entry with actual */
                        /* announcement values.                   */
                        w->procId = procId;
                        w->endPt = amsg->endPt;
#if DEBUG_PRINT
                        SystemP_printf("RPMessage_processAnnounceMsg :Semphore Handle 0x%x\n",
                                (uint32_t)w->semHandle);
#endif

                        pOsalPrms->unlockMutex(w->semHandle);
                    }
                    elem = (IpcUtils_QElem *) IpcUtils_Qnext(elem);
                } while (elem != head);
            }
            pOsalPrms->unLockHIsrGate(module.gateSwi, key);
        }
    }
}

#endif /* IPC_EXCLUDE_CTRL_TASKS */

/**
 *  \brief RPMessage_lookupName : Checks if an announcement has already been
 *           received.
 *           non-blocking query - must be protected by caller
 */
static uint8_t RPMessage_lookupName(uint32_t procId, const char* name, uint32_t *remoteProcId,
                              uint32_t *remoteEndPt)
{
    uint8_t               found = FALSE;
    RPMessage_NameEntry  *p;
    IpcUtils_QElem       *elem, *head;

    if (FALSE == IpcUtils_QisEmpty(&module.announcedEndpts))
    {
        elem = head = (IpcUtils_QElem *) IpcUtils_QgetHeadNode(&module.announcedEndpts);
        do
        {
            p = (RPMessage_NameEntry*)elem;
            if( (NULL != p) &&
                (strncmp(p->name, name, SERVICENAMELEN) == 0) &&
                (p->procId == procId || procId == RPMESSAGE_ANY))
            {
                found = TRUE;
                *remoteEndPt = p->endPt;
                *remoteProcId = p->procId;
#if DEBUG_PRINT
                SystemP_printf("RPMessage_lookupName found endpoint %d, remote %d\n",
                    *remoteEndPt, *remoteProcId);
#endif
                break;
            }
            elem = (IpcUtils_QElem *) IpcUtils_Qnext(elem);
        } while (elem != head);
    }
    return found;
}

/**
 *  \brief RPMessage_waitForProc
 */
int32_t RPMessage_getRemoteEndPt(uint32_t selfProcId, const char* name, uint32_t *remoteProcId,
                             uint32_t *remoteEndPt, uint32_t timeout)
{
    int32_t            key;
    uint8_t            lookupStatus = FALSE;
    int32_t            rtnVal = IPC_SOK;
    void              *semHandle;
    RPMessage_Waiter   taskWaiter;
    uint32_t           namelen;
    Ipc_OsalPrms      *pOsalPrms = &gIpcObject.initPrms.osalPrms;

    namelen = strlen(name);
    if ((namelen >= SERVICENAMELEN || namelen == 0))
    {
        rtnVal = IPC_EFAIL;
    }

    if (NULL != pOsalPrms)
    {
        if ((NULL == pOsalPrms->createMutex ||
             NULL == pOsalPrms->lockMutex) ||
             (NULL == pOsalPrms->deleteMutex))
        {
            rtnVal = IPC_EFAIL;
        }
        if ((NULL == pOsalPrms->lockHIsrGate) ||
            (NULL == pOsalPrms->unLockHIsrGate))
        {
            rtnVal = IPC_EFAIL;
        }
    }

    if (IPC_SOK == rtnVal)
    {
        semHandle   = pOsalPrms->createMutex();
        taskWaiter.semHandle = semHandle;
        strncpy(taskWaiter.name, name, SERVICENAMELEN-1);
        taskWaiter.name[SERVICENAMELEN-1] = '\0';
        taskWaiter.procId = selfProcId;
        taskWaiter.endPt  = 0;

        /* The order of steps is critical here.  There must
         * not be an unprotected time between calling
         * RPMessage_lookupName() and the IpcUtils_Qput().
         */
        key = pOsalPrms->disableAllIntr();
        lookupStatus = RPMessage_lookupName(selfProcId, name,
                                            remoteProcId, remoteEndPt);
        if(FALSE == lookupStatus)
        {
            IpcUtils_Qput(&module.waitingTasks, &taskWaiter.elem);
        }
        pOsalPrms->restoreAllIntr(key);

        if(FALSE == lookupStatus)
        {
            rtnVal = pOsalPrms->lockMutex(semHandle, timeout);
            if(rtnVal == IPC_SOK)
            {
                /* The endPt and procId in taskWaiter is assigned
                 * by RPMessage_processAnnounceMsg() when it
                 * wakes up this task.
                 */
                *remoteEndPt = taskWaiter.endPt;
                *remoteProcId = taskWaiter.procId;
            }
            key = pOsalPrms->lockHIsrGate(module.gateSwi);
            IpcUtils_Qremove((IpcUtils_QElem*)&taskWaiter.elem);
            pOsalPrms->unLockHIsrGate(module.gateSwi, key);
        }

        pOsalPrms->deleteMutex(semHandle);
    }

    return rtnVal;
}

#ifndef IPC_EXCLUDE_CTRL_TASKS

/**
 *  \brief RPMessage_checkForMessages : This function checks for
 *         messages in all virtio that are associated with a endpoint
 *         pool.  This operation has the side effect that it may find
 *         messages for end points that do not yet have an endpoint
 *         created for them.  Those messages will be dropped.
 */
static void RPMessage_checkForMessages(RPMessage_EndptPool *pool)
{
    uint32_t   c;
    int32_t   key;

    for(c = 0; c < module.numCallbacks; c++)
    {
        /* Find callback for RX VQs that have matching pool. */
        if(module.VQ_callbacks[c].pool == pool)
        {
            if(TRUE == Virtio_isReady(module.VQ_callbacks[c].vq))
            {
                key = gIpcObject.initPrms.osalPrms.lockHIsrGate(module.gateSwi);
                RPMessage_swiFxn((uintptr_t)&module.VQ_callbacks[c], 0);
                gIpcObject.initPrms.osalPrms.unLockHIsrGate(module.gateSwi, key);
            }
        }
    }
}
#endif /* IPC_EXCLUDE_CTRL_TASKS */

#ifndef IPC_EXCLUDE_CTRL_TASKS

/**
 *  \brief RPMessage_ctrlMsgTask : Create an endpoint to receive
 *         control messages and dispatch any control messages
 *         received.
 *         This function runs in its own task.
 */
static void RPMessage_ctrlMsgTask(uint32_t* arg0, uint32_t* arg1)
{
    RPMessage_Object *obj = (RPMessage_Object *)arg0;
    uint32_t      remoteEndpoint;
    uint32_t      remoteProcId;
    int32_t       status;
    uint16_t      len = MSGBUFFERSIZE;
    uint32_t      buffer[MSGBUFFERSIZE/sizeof(uint32_t)]; /* alignment */
    RPMessage_Announcement *amsg = (RPMessage_Announcement*)&buffer;

    UNUSED(arg1);

    while(1)
    {
        status = RPMessage_recv((RPMessage_Handle)obj, (void*)amsg, &len,
                                      &remoteEndpoint, &remoteProcId,
                                      IPC_RPMESSAGE_TIMEOUT_FOREVER);
        if(status != IPC_SOK)
        {
            SystemP_printf("RPMessage_recv failed with code %d for endPt %d\n"
                       "RPMessage_ctrlMsgTask terminating early",
                       status, obj->endPt);
            break;
        }

        if(amsg->ctrl.type == CNTRLMSG_ANNOUNCE)
        {
#if DEBUG_PRINT
            SystemP_printf("RPMessage_ctrlMsgTask ...CNTRLMSG_ANNOUNCE\n");
#endif
            RPMessage_processAnnounceMsg((RPMessage_Announcement*)amsg,
                                         remoteProcId);
        }
        else
        {
            SystemP_printf("Invalid cntrl msg type 0x%x from procId %d\n",
                       amsg->ctrl.type, remoteProcId);
        }
    }
}

/**
 * \brief Kick off the control message task.
 * Typically this is called from Main context.
 * It is best to call this before setting the
 * VQ callbacks to avoid dropping cntrl messages.
 */
static int32_t RPMessage_startCtrlMsgTask(RPMessage_Params *params)
{
    TaskP_Params       tparams;
    RPMessage_Object  *obj;
    uint32_t           myEndpoint = 0;
    int32_t            retVal     = IPC_SOK;
    TaskP_Handle       tskHandle  = NULL;

    /* Create the endpoint to receive control messages.
     * This is done as early as possible to avoid missing
     * any control messages.
     */
    params->requestedEndpt = IPC_CTRL_ENDPOINT_ID;
    obj = RPMessage_rawCreate(params, &module.globalPool, &myEndpoint);
    if(NULL != obj)
    {
        TaskP_Params_init(&tparams);
        tparams.priority  = 10;
        tparams.arg0      = (uint32_t*)obj;
        tparams.stacksize = params->stackSize;
        tparams.stack     = params->stackBuffer;
        tskHandle = TaskP_create((void *) RPMessage_ctrlMsgTask, &tparams);
        if(tskHandle == NULL)
        {
            retVal = IPC_EFAIL;
        }
    }
    else
    {
        SystemP_printf("Failed to create control endpoint.\n");
        retVal = IPC_EFAIL;
    }

    return retVal;
}

#endif /* IPC_EXCLUDE_CTRL_TASKS */

static RPMessage_Object* RPMessage_rawCreate(
        RPMessage_Params *params,
        RPMessage_EndptPool* pool,
        uint32_t *endPt)
{
    RPMessage_Object *obj   = NULL;
    uint8_t           found = FALSE;
    int32_t           i;
    uint16_t          queueIndex = 0;
    int32_t           key;
    uint32_t          objSize = RPMessage_getObjMemRequired();
    Ipc_OsalPrms     *pOsalPrms = &gIpcObject.initPrms.osalPrms;
    int32_t           status = IPC_SOK;

    /* Returning NULL, here Fix ME TBD
        This function error checks would require an overhaul */
    if ( (NULL == pOsalPrms) ||
         (NULL == pOsalPrms->lockHIsrGate) ||
         (NULL == pOsalPrms->unLockHIsrGate))
    {
        status = IPC_EFAIL;
    }

    if(IPC_SOK == status)
    {
        key = pOsalPrms->lockHIsrGate(module.gateSwi);

        /* Allocate the endPt */
        if (params->requestedEndpt == RPMESSAGE_ANY)
        {
            /* Search the array for a free slot above reserved: */
            for (i = RPMessage_MAX_RESERVED_ENDPOINT + 1;
                    (i < MAXENDPOINTS) && (found == 0U); i++)
            {
                if (RPMessage_lookupEndpt(pool, i) == NULL)
                {
                    queueIndex = i;
                    found = TRUE;
                    break;
                }
            }
        }
        else if (params->requestedEndpt <= RPMessage_MAX_RESERVED_ENDPOINT)
        {
            if (RPMessage_lookupEndpt(pool, params->requestedEndpt) == NULL)
            {
                queueIndex = params->requestedEndpt;
                found = TRUE;
            }
        }
    }

    if ((FALSE == found) || 
        (params->numBufs < 1) ||
        (params->bufSize < ((params->numBufs * MSGBUFFERSIZE) + objSize)))
    {
        status = IPC_EFAIL;
    }

    if(IPC_SOK == status)
    {
        obj = (RPMessage_Object *) params->buf;
        if (NULL != obj)
        {
            obj->heapParams.blockSize    = MSGBUFFERSIZE;
            obj->heapParams.numBlocks    = params->numBufs;
            obj->heapParams.buf          = (void*)((uintptr_t)params->buf + objSize);
            obj->heapParams.bufSize      = params->bufSize - objSize ;
            obj->heapParams.align        = HEAPALIGNMENT;
            status = IpcUtils_HeapCreate(&obj->heap, &obj->heapParams);
            if (IPC_SOK == status)
            {
                /* Allocate a semaphore to signal when messages received: */
                if (NULL != pOsalPrms->createMutex)
                {
                    obj->semHandle = pOsalPrms->createMutex();
                }

                /* Create our queue of to be received messages: */
                IpcUtils_Qcreate(&obj->queue);

                /* Store our endPt, and object: */
                obj->endPt = queueIndex;
                RPMessage_assignEndpt(pool, queueIndex, obj);

                /* See RPMessage_unblock() */
                obj->unblocked   = FALSE;
                obj->pool        = pool;
                obj->recv_buffer = NULL;
                *endPt           = queueIndex;
            }
            else
            {
                obj = NULL;
            }

            pOsalPrms->unLockHIsrGate(module.gateSwi, key);
        }
    }

    return (obj);
}

int32_t RPMessage_lateInit(uint32_t proc)
{
    uint32_t         c;
    uint32_t         tx_vqId, rx_vqId;
    int32_t          retVal = IPC_SOK;
    Ipc_OsalPrms     *pOsalPrms = &gIpcObject.initPrms.osalPrms;

    if(TRUE == VirtioIPC_getVirtQueues(VIRTIOIPC_RPMSG, proc, 0, &tx_vqId, &rx_vqId))
    {
        uintptr_t arg0;
        c = module.numCallbacks;
        module.numCallbacks++;

        /* setup the receiving path */
        module.VQ_callbacks[c].pool = &module.globalPool;
        module.VQ_callbacks[c].vq = Virtio_getHandle(proc, VIRTIO_RX);

        arg0 = (uintptr_t)&module.VQ_callbacks[c];
        if (TRUE == Virtio_isRemoteLinux(proc))
        {
            pOsalPrms->createHIsr(&module.VQ_callbacks[c].swi,
                    &RPMessage_swiLinuxFxn, (void *)arg0);
        }
        else
        {
            pOsalPrms->createHIsr(&module.VQ_callbacks[c].swi,
                    &RPMessage_swiFxn, (void *)arg0);
        }
        Virtio_setCallback(proc, virtio_callback, (uint32_t *)&module.VQ_callbacks[c]);

        /* setup the sending path */
        module.tx_VQs[proc] = Virtio_getHandle(proc, VIRTIO_TX);
    }
    else
    {
        retVal = IPC_EFAIL;
    }

#ifndef IPC_EXCLUDE_CTRL_TASKS
    if(retVal == IPC_SOK)
        RPMessage_checkForMessages(&module.globalPool);
#endif /* IPC_EXCLUDE_CTRL_TASKS */

    return retVal;
}

/**
 *  \brief RPMessage_init : Initializing the framework
 */
int32_t RPMessage_init(RPMessage_Params *params)
{
    uint32_t         p;
    uint32_t         c;
    uint32_t         tx_vqId, rx_vqId;
    int32_t          retVal = IPC_SOK;
    Ipc_OsalPrms     *pOsalPrms = &gIpcObject.initPrms.osalPrms;

    /* Clear module state */
    memset(&module, 0, sizeof(module));

    /* Gate to protect module object and lists: */
    if ((1U == is_aligned(params->buf,HEAPALIGNMENT)) &&
        (NULL != pOsalPrms->createHIsrGate) &&
        (NULL != pOsalPrms->createHIsr))
    {
        module.gateSwi = pOsalPrms->createHIsrGate();
    }
    else
    {
        retVal = IPC_EFAIL;
        SystemP_printf("RPMessage_init : Memory misalignment  or invalid HIstGate fxn\n");
    }

    if( retVal != IPC_EFAIL)
    {
        IpcUtils_Qcreate(&module.announcedEndpts);
        IpcUtils_Qcreate(&module.waitingTasks);

#ifndef IPC_EXCLUDE_CTRL_TASKS
        /* Call startCtrlMsgTask() before setting the VQ
         * callbacks to avoid dropping control messages.
         */
        retVal = RPMessage_startCtrlMsgTask(params);
#endif /* IPC_EXCLUDE_CTRL_TASKS */

        /* Connect to the VQs by setting the VQ callbacks and
         * creating the assocated SWIs.  Once the callbacks
         * are installed is it possilbe to drop messages if
         * the dest endpoint has not yet been created.
         */
        for(p = 0; ((p < IPC_MAX_PROCS) && (IPC_SOK == retVal)); p++)
        {
            if(TRUE == VirtioIPC_getVirtQueues(VIRTIOIPC_RPMSG, p, 0, &tx_vqId, &rx_vqId))
            {
                uintptr_t arg0;
                c = module.numCallbacks;
                module.numCallbacks++;

                /* setup the receiving path */
                module.VQ_callbacks[c].pool = &module.globalPool;
                module.VQ_callbacks[c].vq = Virtio_getHandle(p, VIRTIO_RX);

                arg0 = (uintptr_t)&module.VQ_callbacks[c];
                if (TRUE == Virtio_isRemoteLinux(p))
                {
                    pOsalPrms->createHIsr(&module.VQ_callbacks[c].swi,
                            &RPMessage_swiLinuxFxn, (void *)arg0);
                }
                else
                {
                    pOsalPrms->createHIsr(&module.VQ_callbacks[c].swi,
                            &RPMessage_swiFxn, (void *)arg0);
                }
                Virtio_setCallback(p, virtio_callback, (uint32_t *)&module.VQ_callbacks[c]);

                /* setup the sending path */
                module.tx_VQs[p] = Virtio_getHandle(p, VIRTIO_TX);
            }
            else
            {
                module.tx_VQs[p] = NULL;
            }
        }

#ifndef IPC_EXCLUDE_CTRL_TASKS
        /*
         * At this point, the control end point would not have been created.
         *  Let apps, decide when to process pending messages
         */
        RPMessage_checkForMessages(&module.globalPool);

#endif /* IPC_EXCLUDE_CTRL_TASKS */
    }

    return retVal;
}


/*
 *  ======== RPMessage_create ========
 */
RPMessage_Handle RPMessage_create(RPMessage_Params *params, uint32_t *endPt)
{
    RPMessage_Object *obj = NULL;
    RPMessage_Params defaultParams;

    if(params == NULL)
    {
        params = &defaultParams;
        RPMessageParams_init(params);
    }

    if( 0U == is_aligned(params->buf,HEAPALIGNMENT))
    {
        SystemP_printf("RPMessage_create : Memory alignment failed\n");
    }
    else
    {
        obj = RPMessage_rawCreate(params, &module.globalPool, endPt);
    }

    return (RPMessage_Handle)obj;
}

/**
 *  \brief RPMessage_deinit : Tear down the module
 */
void RPMessage_deInit(void)
{
    uint32_t c;

    for(c = 0; c < module.numCallbacks; c++)
    {
        gIpcObject.initPrms.osalPrms.deleteHIsr(&(module.VQ_callbacks[c].swi));
    }

    if (NULL != gIpcObject.initPrms.osalPrms.deleteHIsrGate)
    {
        gIpcObject.initPrms.osalPrms.deleteHIsrGate(&module.gateSwi);
    }
}

/*
 *  ======== RPMessage_delete ========
 */
int32_t RPMessage_delete(RPMessage_Handle *handlePtr)
{
    int32_t                    status = IPC_EFAIL;
    RPMessage_Object          *obj;
    RPMessage_MsgElem         *payload;
    int32_t                    key;
    Ipc_OsalPrms              *pOsalPrms = &gIpcObject.initPrms.osalPrms;

    if (NULL != pOsalPrms)
    {
        if ((NULL != pOsalPrms->lockHIsrGate) &&
             (NULL != pOsalPrms->unLockHIsrGate))
        {
            status = IPC_SOK;
        }
    }
    if ((handlePtr && (obj = (RPMessage_Object *)(*handlePtr))) &&
        (IPC_SOK == status))
    {
       /* Null out our slot in the endpoint pool. */
       key = pOsalPrms->lockHIsrGate(module.gateSwi);
       RPMessage_assignEndpt(obj->pool, obj->endPt, NULL);
       obj->pool = NULL;
       pOsalPrms->unLockHIsrGate(module.gateSwi, key);

       if (NULL != pOsalPrms->deleteMutex)
       {
            pOsalPrms->deleteMutex(obj->semHandle);
       }

       key = gIpcObject.initPrms.osalPrms.disableAllIntr();

       /* Free/discard all queued message buffers: */
       while (0U == IpcUtils_QisEmpty(&obj->queue))
       {
            payload = (RPMessage_MsgElem *)IpcUtils_QgetHead(&obj->queue);
            if(NULL != payload)
            {
                IpcUtils_HeapFree(&obj->heap, (void *)payload, MSGBUFFERSIZE);
            }
       }

       gIpcObject.initPrms.osalPrms.restoreAllIntr(key);

       IpcUtils_HeapDelete(&obj->heap);

       IpcUtils_Qdelete(&(obj->queue));

       *handlePtr = NULL;
    }

    return(status);
}

/*
 *  ======== RPMessage_recv ========
 */
int32_t RPMessage_recv(RPMessage_Handle handle, void* data, uint16_t *len,
                   uint32_t *rplyEndPt, uint32_t *rplyProcId, uint32_t timeout)
{
#ifndef IPC_EXCLUDE_BLOCKING_RX
    int32_t             status = IPC_SOK;
    RPMessage_Object   *obj = (RPMessage_Object *)handle;
    int32_t             semStatus;
    uint8_t             skiplist = FALSE;
    RPMessage_MsgElem  *payload;
    uint32_t            key;
    /* Fix ME TBD, skipping the null tests, as this function check's/error
        handling would require an overhaul */
    Ipc_OsalPrms *pOsalPrms = &gIpcObject.initPrms.osalPrms;

    key = pOsalPrms->lockHIsrGate(module.gateSwi);
    if (TRUE == IpcUtils_QisEmpty(&obj->queue))
    {
        obj->recv_buffer = data;
        skiplist = TRUE;
    }
    pOsalPrms->unLockHIsrGate(module.gateSwi, key);

    /*  Block until notified. */
    semStatus = IPC_SOK;
    semStatus = pOsalPrms->lockMutex(obj->semHandle, timeout);

    if (semStatus == IPC_ETIMEOUT)
    {
        status = IPC_ETIMEOUT;
    }
    else if (TRUE == obj->unblocked)
    {
        status = IPC_E_UNBLOCKED;
    }
    else if(TRUE == skiplist)
    {
        *len = obj->payload.len;
        *rplyEndPt = obj->payload.src;
        *rplyProcId = obj->payload.procId;
    }
    else
    {
        key = gIpcObject.initPrms.osalPrms.disableAllIntr();

        payload = (RPMessage_MsgElem *)IpcUtils_QgetHead(&obj->queue);
        if ( (NULL == payload) ||
             (payload == (RPMessage_MsgElem *)&obj->queue))
        {
            status = IPC_EFAIL;
        }

        if(status != IPC_EFAIL)
        {
            /* Now, copy payload to client and free our internal msg */
            memcpy(data, payload->data, payload->len);
            *len        = payload->len;
            *rplyEndPt  = payload->src;
            *rplyProcId = payload->procId;
            IpcUtils_HeapFree(&obj->heap, (void *)payload,
                    (payload->len + sizeof(RPMessage_MsgElem)));
            gIpcObject.initPrms.osalPrms.restoreAllIntr(key);
        }
    }

    return (status);
#else

    return (IPC_EUNSUPPORTED);

#endif /* IPC_EXCLUDE_BLOCKING_RX */
}

/*
 *  ======== RPMessage_recvNb ========
 */
int32_t RPMessage_recvNb(RPMessage_Handle handle, void* data, uint16_t *len,
                   uint32_t *rplyEndPt, uint32_t *rplyProcId)
{
    RPMessage_Object    *obj;
    RPMessage_MsgElem   *payload;
    uint32_t            key;
    int32_t             status = IPC_EBADARGS;

    if (((NULL != data) && (NULL != len)) &&
        ((NULL != rplyEndPt) && (NULL != rplyProcId)))
    {
        obj = (RPMessage_Object *)handle;
        status = IPC_ETIMEOUT;
        if (FALSE == IpcUtils_QisEmpty(&obj->queue))
        {
            key = gIpcObject.initPrms.osalPrms.disableAllIntr();

            payload = (RPMessage_MsgElem *)IpcUtils_QgetHead(&obj->queue);
            if ((NULL != payload) && (payload != (RPMessage_MsgElem *)&obj->queue))
            {
                /* Now, copy payload to client and free our internal msg */
                memcpy(data, payload->data, payload->len);
                *len = payload->len;
                *rplyEndPt = payload->src;
                *rplyProcId = payload->procId;

                IpcUtils_HeapFree(&obj->heap, (void *)payload,
                    (payload->len + sizeof(RPMessage_MsgElem)));

                status = IPC_SOK;
            }
            else
            {
                /* No elements in the queue, but why? */
                status = IPC_EFAIL;
            }
            gIpcObject.initPrms.osalPrms.restoreAllIntr(key);
        }
    }

    return (status);
}


/*
 *  ======== RPMessage_rawSend ========
 */
static int32_t RPMessage_rawSend(Virtio_Handle vq,
                      uint32_t dstEndPt,
                      uint32_t srcEndPt,
                      void*    data,
                      uint16_t len)
{
    int32_t               status = IPC_SOK;
    int32_t               token = 0;
    int32_t               key;
    int32_t               length = 0;
    uint32_t              bufSize;
    RPMessage_MsgHeader*  msg = NULL;
    Ipc_OsalPrms          *pOsalPrms = &gIpcObject.initPrms.osalPrms;

    bufSize = sizeof(RPMessage_MsgHeader) + len;

    if (NULL != pOsalPrms)
    {
        if ((NULL != pOsalPrms->lockHIsrGate) &&
            (NULL != pOsalPrms->unLockHIsrGate))
        {
            /* Send to remote processor: */
            key = pOsalPrms->lockHIsrGate(module.gateSwi);
            token = Virtio_getAvailBuf(vq, (void **)&msg, &length);
            pOsalPrms->unLockHIsrGate(module.gateSwi, key);
        }
    }
    if(!msg)
    {
        SystemP_printf("RPMessage_rawSend ...NULL MsgHdr\n");
        status = IPC_EFAIL;
    }
    if(length < bufSize)
    {
        SystemP_printf("RPMessage_rawSend ...length %d, reqrd %d\n", length, bufSize);
        status = IPC_EFAIL;
    }
    if(status != IPC_EFAIL)
    {
        if (token >= 0)
        {
            /* Copy the payload and set message header: */
            memcpy(msg->payload, data, len);
            msg->dataLen = len;
            msg->dstAddr = dstEndPt;
            msg->srcAddr = srcEndPt;
            msg->flags = 0;
            msg->srcProcId = Ipc_mpGetSelfId();

            key = pOsalPrms->lockHIsrGate(module.gateSwi);
            Virtio_addUsedBuf(vq, token, bufSize);
            Virtio_kick(vq);
            pOsalPrms->unLockHIsrGate(module.gateSwi, key);
        }
        else
        {
            SystemP_printf("RPMessage_rawSend ...Invalid token %d\n", token);
            status = IPC_EFAIL;
        }
    }

    return (status);
}

/*
 *  ======== RPMessage_send ========
 */
int32_t RPMessage_send(RPMessage_Handle handle, uint32_t procId, uint32_t dstEndPt,
     uint32_t srcEndPt, void* data, uint16_t len)
{
    Virtio_Handle       vq;
    uint32_t            endPt;
    int32_t             status = IPC_SOK;

    if(procId >= IPC_MAX_PROCS)
    {
        status = IPC_EFAIL;
    }

    if(procId == Ipc_mpGetSelfId())
    {
        status = IPC_EFAIL;
    }

    if(status != IPC_EFAIL)
    {
        vq = module.tx_VQs[procId];
        if(NULL == vq)
        {
            SystemP_printf("RPMessage_send ...NULL vq\n");
            status = IPC_EFAIL;
        }
        else
        {
            endPt = (handle == NULL) ? srcEndPt : handle->endPt;
            status = RPMessage_rawSend(vq, dstEndPt, endPt, data, len);
        }
    }

    return status;
}


/*
 *  ======== RPMessage_unblock ========
 */
void RPMessage_unblock(RPMessage_Handle handle)
{
    RPMessage_Object *obj = (RPMessage_Object *)handle;

    /* Set instance to 'unblocked' state, and post */
    obj->unblocked = TRUE;

    if (NULL != gIpcObject.initPrms.osalPrms.unlockMutex)
    {
        gIpcObject.initPrms.osalPrms.unlockMutex(obj->semHandle);
    }
}

#ifndef IPC_EXCLUDE_INIT_PARAMS_INIT
void IpcInitPrms_init(uint32_t instId, Ipc_InitPrms *initPrms)
{
    if(NULL != initPrms)
    {
        memset(initPrms, 0, sizeof(*initPrms));
        initPrms->instId                = instId;

        initPrms->virtToPhyFxn          = NULL;
        initPrms->phyToVirtFxn          = NULL;

        IpcOsalPrms_init(&initPrms->osalPrms);
    }

    return;
}
#endif /* IPC_EXCLUDE_INIT_PARAMS_INIT */

Ipc_Object *getIpcObjInst(uint32_t instId)
{
    return &gIpcObject;
}

int32_t Ipc_init(Ipc_InitPrms *cfg)
{
    int32_t retVal = IPC_EINVALID_PARAMS;

    memset(&gIpcObject, 0, sizeof(gIpcObject));

#ifndef IPC_EXCLUDE_INIT_PARAMS_INIT
    if (NULL == cfg)
    {
        IpcInitPrms_init(0U, &gIpcObject.initPrms);

        Ipc_mailboxModuleStartup();

        retVal = IPC_SOK;
    }
    else
#endif /* IPC_EXCLUDE_INIT_PARAMS_INIT */
    {
        if (0U == cfg->instId)
        {
            if ((NULL == cfg->osalPrms.disableAllIntr) ||
                (NULL == cfg->osalPrms.restoreAllIntr))
            {
                retVal = IPC_EINVALID_PARAMS;
            }
            else
            {
                memcpy(&gIpcObject.initPrms, cfg, sizeof(gIpcObject.initPrms));
                retVal = IPC_SOK;
            }
        }
    }
    if (IPC_SOK == retVal)
    {
        IpcUtils_Init(&gIpcObject.initPrms.osalPrms);
    }
    gIpcObject.instId = 0U;

    return (retVal);
}

int32_t Ipc_deinit(void)
{
    int32_t  retVal = IPC_SOK;

    return (retVal);
}

void Ipc_newMessageIsr(uint32_t srcProcId)
{
#ifdef IPC_EXCLUDE_INTERRUPT_REG
    Ipc_mailboxIsr(srcProcId);
#endif /* IPC_EXCLUDE_INTERRUPT_REG */

    return;
}
