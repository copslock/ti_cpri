/*
 *  file  rm_loc.h
 *
 *  General private data structures of Resource Manager.
 *
 *  ============================================================================
 *      (C) Copyright 2012-2015, Texas Instruments, Inc.
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
 *
 *  \par
*/

#ifndef RM_LOC_H_
#define RM_LOC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* RM external includes */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>

/* RM internal includes */
#include <ti/drv/rm/include/rm_internal.h>
#include <ti/drv/rm/include/rm_allocatorloc.h>
#include <ti/drv/rm/include/rm_policyloc.h>
#include <ti/drv/rm/include/rm_transportloc.h>
#include <ti/drv/rm/include/rm_nameserverloc.h>
#include <ti/drv/rm/include/rm_treeloc.h>

/* Service transaction linked list node */
typedef struct Rm_Transaction_s {
    /* Transaction service type */
    Rm_ServiceType           type;
    /* Local ID of the transaction. */
    uint32_t                 localId;
    /* ID of transaction in RM instance that generated the packet that
     * resulted in the creation of the transaction.  The originating ID
     * will be placed in the transaction's response packet.  The RM
     * instance that receives the response will match the response packet
     * with the originating request using the ID */
    uint32_t                 remoteOriginatingId;
    /* Transaction response mechanism union */
    union {
        /* Service callback function if received locally via Service API */
        Rm_ServiceCallback   callback;
        /* Transport handle to send response packet on if received via
         * Transport rcv API */
        Rm_Transport        *respTrans;
    } u;
    /* Transaction state.  The codes are defined in rm.h */
    int32_t                  state;
    /* Transaction has been forwarded to CD or Server instance.  Waiting for
     * response */
    int8_t                   hasBeenForwarded;
    /* The transaction ID for a transaction pending on a CD while this
     * transaction is sent to the Server as a request for data required to
     * complete pending transaction */
    uint32_t                 pendingTransactionId;
    /* Name of the RM instance the service originated from */
    char                     serviceSrcInstName[RM_NAME_MAX_CHARS];
    /* Resource information */
    Rm_ResourceInfo          resourceInfo;
    /* Link to the next transaction in the queue */
    struct Rm_Transaction_s *nextTransaction;
} Rm_Transaction;

/* Server-specific instance data */
typedef struct {
    /* Pointer to the root entry of the NameServer */
    Rm_NameServerTree *nameServer;
} Rm_ServerInstData;

/* Shared Client-specific instance data */
typedef struct {
    /* Shared Server instance handle */
    Rm_Handle sharedServerHandle;
} Rm_SharedClientInstData;

/* RM instance structure */
typedef struct {
    /* Name given to the RM instance.  Policy will assign resources based
     * on the names assigned to instances at instance init */
    char                    instName[RM_NAME_MAX_CHARS];
    /* Instance type */
    Rm_InstType             instType;
    /* Instance lock status.  Instance locks if static request fails when
     * checked against global policy */
    int8_t                  isLocked;
    /* Tracks whether the instance has registered with a CD or Server.
     * Applicable to CD and Client instances */
    int8_t                  registeredWithDelegateOrServer;
    /* Pointer to the serviceHandle opened from the instance */
    Rm_ServiceHandle       *serviceHandle;
    /* Linked list of transports registered by the application */
    Rm_Transport           *transports;
    /* Service transaction sequence number tracker */
    uint32_t                transactionSeqNum;
    /* Service transaction linked list queue */
    Rm_Transaction         *transactionQueue;
    /* Block handle provided through OSAL for when RM needs to block due to
     * a service request that requires a blocking operation to complete and
     * a service callback function has not been provided */
    void                   *blockHandle;
    /* Multi-threaded semaphore object */
    uint32_t               *mtSemObj;
    /* Pointer to root entry of valid instance tree */
    Rm_PolicyValidInstTree *validInstTree;
    /* Max valid instance index */
    int32_t                 maxInstIdx;
    /* Pointer to allocator tree */
    Rm_AllocatorTree       *allocatorTree;
    /* Instance-type specific constructs */
    union {
        /* Server-specific instance data */
        Rm_ServerInstData         server;
        /* Shared Client-specific instance data */
        Rm_SharedClientInstData   sharedClient;
    } u;
} Rm_Inst;

Rm_Transaction *rmTransactionQueueAdd(Rm_Inst *rmInst);
Rm_Transaction *rmTransactionQueueFind(Rm_Inst *rmInst, uint32_t transactionId);
int32_t rmTransactionQueueDelete(Rm_Inst *rmInst, uint32_t transactionId);
void rmProcessRouter(Rm_Inst *rmInst, Rm_Transaction *transaction);

#ifdef __cplusplus
}
#endif

#endif /* RM_LOC_H_ */

