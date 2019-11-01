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
 *  \defgroup DRV_IPC_MODULE IPC Driver
 *
 *  @{
 *
 * The IPC low level driver provides the communication mechanism between
 * various cores on J7 device.
 * This is IPC lld driver documentation.
 */
/* @} */

/**
 *  \ingroup DRV_IPC_MODULE
 *  \defgroup IPC_TOP_LEVEL IPC Driver Header
 *            This is IPC's top level include for applications
 *
 *  @{
 */

/**
 *  \file ipc.h
 *
 *  \brief IPC Low Level Driver API/interface data types file.
 */

#ifndef IPC_H_
#define IPC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/csl/soc.h>
#include <ti/csl/csl_navss_main.h>

#include <ti/drv/ipc/include/ipc_types.h>
#include <ti/drv/ipc/include/ipc_rsctypes.h>
#include <ti/drv/ipc/include/ipc_config.h>
#include <ti/drv/ipc/include/ipc_mp.h>
#include <ti/drv/ipc/include/ipc_virtio.h>
#include <ti/drv/ipc/soc/ipc_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
 *  \brief  RPMessage_Handle type
 */
typedef struct RPMessage_Object_s* RPMessage_Handle;

/**
 *  \brief  RPMessage_Callback
 *
 *  \param RPMessage_Handle    [IN]
 *  \param arg   [IN] void* arguments to function
 *  \param data  [IN] data pointer
 *  \param len   [IN] length of data
 *  \param src   [IN] source
 *
 */
typedef void (*RPMessage_Callback)(RPMessage_Handle handle, void* arg, void* data,
                                      uint16_t len, uint32_t src);


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief IPC initialization parameters.
 *
 */
typedef struct Ipc_InitPrms_s
{
    uint32_t                instId;
    /**< [IN] Ipc_InstanceId. Only 1 instance is supported in this
        implementation. The value shall be 0 */
    Ipc_PhyToVirtFxn        phyToVirtFxn;
    /**< If not NULL, this function will be called to convert physical address
     *   to virtual address to access the pointer returned by the IPC.
     *   If NULL, the driver will assume a one-one mapping.
     *
     *   Note: The init fxn will initialize this to the default one-one map
     *   function #Ipc_defaultPhyToVirtFxn
     */
    Ipc_VirtToPhyFxn        virtToPhyFxn;
    /**< If not NULL, this function will be called to convert virtual address
     *   to physical address to be provided to IPC.
     *   If NULL, the driver will assume a one-one mapping.
     *
     *   Note: The init fxn will initialize this to the default one-one map
     *   function #Ipc_defaultVirtToPhyFxn
     */
    Ipc_OsalPrms            osalPrms;
    /**< OSAL callback  parameters */

    Ipc_NewMsgReceivedFxn   newMsgFxn;
    /**< Optional callback function, that would be invoked when a new message
        is received */
} Ipc_InitPrms;

/**
 *  \brief Parameter structure for creating RPMessage endpoints
 */
typedef struct RPMessage_Params_s
{
    uint32_t  requestedEndpt;
    /**< Requested Endpoint - Any or next available */

    uint32_t  numBufs;
    /**< Maximum number of buffers to allocate for queuing received messages. */

    void*     buf;
    /**<  Buffer pointer to store RPMessage Object */

    uint32_t  bufSize;
    /**< Buffer Size. Recommended Size is (512*256 + 256).
          - 256 buffers of 512 bytes.
          - 256 size of RPMessage object used for bookkeeping. */

    void*     stackBuffer;
    /**<  Buffer used for stack for control task */

    uint32_t  stackSize;
    /**< StackSize used for the task */
} RPMessage_Params;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \brief      Initialize IPC module
 *
 *              Very first API to be invoked to initialize internal data
 *              structure and utilities required.
 *
 *  \param cfg [IN] Initialization parameters. Expected to be NULL in case of
 *              expected use in tirtos environment and non-null in case of
 *              baremetal
 *
 *  \return      #IPC_SOK or #IPC_EINVALID_PARAMS or #IPC_EFAIL
 */
int32_t Ipc_init(Ipc_InitPrms *cfg);

/**
 *  \brief      De Initialize IPC module
 *
 *              Very last API to be invoked to de initialize.
 *
 *  \return      #IPC_SOK
 */
int32_t Ipc_deinit(void);

/**
 * \brief       Returns Message Buffer Size
 */
uint32_t RPMessage_getMessageBufferSize(void);


/**
 * \brief       Returns local memory for RPMessage Object
 */
uint32_t RPMessage_getObjMemRequired(void);

/**
 *  \brief      Initialize RPMessage Module
 *
 *  Can be called from Main or Task context.  Must be called before
 *  calling any other RPMessage function;
 *
 *  \return      #IPC_SOK or #IPC_EFAIL
 */
int32_t RPMessage_init(RPMessage_Params *params);

/**
 *  \brief      Add a proc to RPMessage Module
 *
 *  Can be called from Main or Task context.  Must be called after
 *  calling RPMessage_init. Must be called before any other RPMessage
 *  function to communicate with this proc;
 *
 *  \return      #IPC_SOK or #IPC_EFAIL
 */
int32_t RPMessage_lateInit(uint32_t proc);


/**
 *  \brief      Tear down the RPMessage Module.  The module API
 *              should not be used after this is called.
 */
void RPMessage_deInit(void);


/**
 *  \brief Initialize an RPMessage_Params structure to default values.
 *
 *  This function must be called before on a RPMessage_Params before passing
 *  it to RPMessage_create().
 *
 *  \param params    [IN] Address of the RPMessage_Params structure
 *                        to be initialized.
 *
 *  \return      #IPC_SOK or #IPC_EFAIL
 */
int32_t RPMessageParams_init(RPMessage_Params *params);

/**
 *  \brief Create a endpoint instance for receiving.
 *
 *  The returned handle is to an object containing a queue for receiving
 *  messages from the transport, and a 32 bit endpoint ID unique to this local
 *  processor.
 *
 *  \param params    [IN] \ref RPMessage_Params
                          Address of an initialized RPMessage_Params
 *                        structure or NULL will use defaults.
 *
 *  \param endPt    [OUT] Endpoint ID for this side of the connection.
 *
 *
 *  \return RPMessage Handle, or NULL if:
 *           - reserved endpoint already taken;
 *           - could not allocate object
 *
 */
RPMessage_Handle RPMessage_create(RPMessage_Params *params, uint32_t *endPt);

/**
 *  \brief Sets callback.
 *
 *  Sets the callback function and function arguments, so that
 *  it does not need to wai for semaphore.
 *
 *  \param handle    [IN] An RPMessage_handle
 *  \param cb    [IN] \ref RPMessage_Callback
 *                    Callback function.
 *  \param arg   [IN] Arguments of the callback function
 *
 *
 *  \return     - #IPC_SOK: Message successfully returned
 *              - #IPC_ETIMEOUT: RPMessage_recv timed out
 *              - #IPC_EFAIL:    A general failure has occurred
 *
 */
int32_t RPMessage_setCallback(RPMessage_Handle handle, RPMessage_Callback cb, void* arg);

/**
 *  \brief Receives a message from an endpoint instance
 *
 *  This function returns a status. It also copies data to the client.
 *  If no message is available, it blocks on the semaphore object until
 *  the semaphore is signaled or a timeout occurs.  The semaphore is
 *  signaled, when Message_send is called to deliver a message to this
 *  endpoint. If a timeout occurs, len is set to zero and the status is
 *  RPMessage_E_TIMEOUT. If a timeout of zero is specified, the
 *  function returns immediately and if no message is available, the len
 *  is set to zero and the status is RPMessage_E_TIMEOUT.
 *  RPMessage_E_UNBLOCKED status is returned, if RPMessage_unblock() is
 *  called on the RPMessage handle.  When a message is retrieved, the
 *  message data is copied into the memory location of the data pointer,
 *  and RPMessage_S_SUCCESS status is returned.
 *
 *  \param handle    [IN] An RPMessage_handle
 *  \param data    [OUT] Pointer to the client's data buffer.
 *  \param len    [OUT] Amount of data received.
 *  \param rplyEndPt    [OUT] Endpoint of source (for replies).
 *  \param fromProcId    [OUT] ProcId of source (for replies).
 *  \param timeout    [IN] Maximum duration to wait for a message in
 *                              microseconds.
 *
 *  \return     - #IPC_SOK: Message successfully returned
 *              - #IPC_ETIMEOUT: RPMessage_recv timed out
 *              - #IPC_E_UNBLOCKED: RPMessage_recv was unblocked
 *              - #IPC_EFAIL:    A general failure has occurred
 *
 *  \ref RPMessage_send RPMessage_unblock
 */
int32_t RPMessage_recv(RPMessage_Handle handle, void* data, uint16_t *len,
                      uint32_t *rplyEndPt, uint32_t *fromProcId, uint32_t timeout);

/**
 *  \brief A non blocking API to receive message
 *
 *  This function check if there any received messages, if found the received
 *      message is copied into buffer provided by the caller.
 *  The caller will have to allocate sufficient buffer to account for the
 *      reception of the largest message. Configured while creating the driver.
 *
 *  \param handle       [IN] An RPMessage_handle
 *  \param data         [OUT] Pointer to the buffer, allocted by the caller
 *  \param len          [OUT] Size of the received buffer, in bytes
 *  \param rplyEndPt    [OUT] Endpoint of source (for replies)
 *  \param fromProcId   [OUT] ProcId of source (for replies)
 *
 *  \return     - #IPC_SOK      : Message successfully returned
 *              - #IPC_EBADARGS : In case where one or more arguments is NULL
 *              - #IPC_ETIMEOUT : No messages available at this point
 *              - #IPC_EFAIL    : Internal error
 *
 *  \ref RPMessage_send
 */
int32_t RPMessage_recvNb(RPMessage_Handle handle, void* data, uint16_t *len,
                   uint32_t *rplyEndPt, uint32_t *rplyProcId);

/**
 *  \brief Sends data to a remote processor.
 *
 *  \param handle     [IN] Handle of RPMessage Object
 *  \param dstProc    [IN] Destination ProcId.
 *  \param dstEndPt   [IN] Destination Endpoint.
 *  \param srcEndPt   [IN] Source Endpoint.
 *  \param data       [IN] Data payload to be copied and sent.
 *  \param len        [IN] Amount of data to be copied.
 *
 *  \return     - #IPC_SOK
 *              - #IPC_EFAIL
 */
int32_t RPMessage_send(RPMessage_Handle handle,
                      uint32_t dstProc,
                      uint32_t dstEndPt,
                      uint32_t srcEndPt,
                      void*    data,
                      uint16_t len);

/**
 *  \brief Delete an endpoint instance.
 *
 *  This function deletes a created endpoint instance. If the
 *  message queue is non-empty, any messages remaining in the queue
 *  will be lost.
 *
 *  \param handlePtr [IN,OUT] Pointer to handle to delete.  Set to
 *                             NULL.
 *
 *  \return     - #IPC_EFAIL: delete failed
 *              - #IPC_SOK: delete successful
 */
int32_t RPMessage_delete(RPMessage_Handle *handlePtr);

/**
 *  \brief      Unblocks an RPMessage_recv()
 *
 *  Unblocks a reader thread that is blocked on a RPMessage_recv.  The
 *  RPMessage_recv call will return with status #IPC_E_UNBLOCKED
 *  indicating that it returned due to a RPMessage_unblock rather than by
 *  a timeout or receiving a message.
 *
 *  Restrictions:
 *  -  A queue may not be used after it has been unblocked.
 *  -  RPMessage_unblock may only be called on a local queue.
 *
 *  \param[in]  handle      RPMessage handle
 *
 *  \sa         RPMessage_recv
 */
void RPMessage_unblock(RPMessage_Handle handle);


/**
 *  \brief Wait for an endpoint to become available on another
 *         processor.
 *
 *  Block the current task until the specified processor announces the
 *  named endpoint.  The name is a string that identifies the service
 *  that is offered on the endpoint.  This allows an application to both
 *  wait for the remote processor to signal that it is ready to
 *  communicate and to lookup services by name.  The procId can be that
 *  of a specific processor or PRMessage_ANY to wait for any processor
 *  to announce the named endpoint.  Suitable values for timeout are the
 *  same as for the ti.sysbios.knl.Semaphore module.
 *
 *  \param selfProcId    [IN] Remote processor ID
 *  \param name      [IN] Name of the endpoint
 *  \param remoteProcId    [OUT] Remote processor ID
 *  \param remoteEndPt    [OUT] Remote endpoint ID
 *  \param timeout    [IN] Timeout value (in system ticks)
 *
 *  \return     - #IPC_SOK: Endpoint successfully returned
 *              - #IPC_ETIMEOUT: Time out occured
 *              - #IPC_EFAIL:    Invalid input
 */
int32_t RPMessage_getRemoteEndPt(uint32_t selfProcId, const char* name, uint32_t *remoteProcId,
                             uint32_t *remoteEndPt, uint32_t timeout);

/**
 *  \brief Annouce the name of an endpoint and that it is ready to
 *         to receive messages.
 *
 *  Announcing an endpoint to other processesors acheives two goals.
 *  First it signals that the endpoint is ready to recieve, and second,
 *  the endpoint is offering the named service.  Announcements can
 *  be sent to all remote processors by using RPMessage_ALL for
 *  remoteProcId, otherwise only the specified processor recieves the
 *  announcement.  The name may not be an empty string or NULL.  The
 *  value of endPt should have been returned by a prior call to
 *  RPMessage_create().
 *
 *  \param remoteProcId    [IN] \ref RPMessage_Params
                                The remote processor to receive the
 *                              announcement
 *  \param endPt    [IN] Endpoint ID to announce
 *  \param name     [IN] Name of the service on the endpoint
 *
 *
 *  \return     - #IPC_SOK: Endpoint successfully announced
 *              - #IPC_EFAIL:    Invalid input
 *
 */
int32_t RPMessage_announce(uint32_t remoteProcId, uint32_t endPt,
       const char* name);

/**
 *  \brief New Message Interrupt Handler
 *
 *  \warning To be used only when built for baremetal
 *
 *  When built for baremetal applications, the IPC driver do not register
 *  interrupt handler to handle interrupts from mailbox.
 *  It's expected that user of this driver shall invoke this function, on
 *  reception of interrupt from the mailbox associated with remote processor
 *
 *  \param remoteProcId    [IN] The remote processor ID
 *
 *  \return     - None
 *
 */
void Ipc_newMessageIsr(uint32_t srcProcId);

/** \brief API Mailbox Enable new MSG interrupt for a given remote processor
 *
 *  \warning To be used only when built for baremetal
 *
 *  \param  uint16_t    Remote Processor Identifier
 *  \param  uint16_t    Own Proc ID
 *
 *  \return None
 */
void Ipc_mailboxEnableNewMsgInt(uint16_t selfId, uint16_t remoteProcId);

/** \brief API Mailbox Disable new MSG interrupt for a given remote processor
 *
 *  \warning To be used only when built for baremetal
 *
 *  \param  uint16_t    Remote Processor Identifier
 *  \param  uint16_t    Own Proc ID
 *
 *  \return None
 */
void Ipc_mailboxDisableNewMsgInt(uint16_t selfId, uint16_t remoteProcId);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_H_ */

/* @} */
